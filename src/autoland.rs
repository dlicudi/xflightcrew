// Stage-1 scaffolding: the runway finder is built and tested but not yet called
// from the running app (Stage 3's control loop consumes it). Allow dead_code until
// then, so the WIP module doesn't spam the build — removed once it's wired in.
#![allow(dead_code)]

//! Native autoland — **Stage 1: runway data**.
//!
//! Finds the nearest suitable landing runway, parsed straight from X-Plane's
//! `apt.dat` (Global Airports): type-`1`/`16`/`17` airport headers carry the ICAO,
//! type-`100` land runways carry both thresholds' lat/lon (fields 9-10 and 18-19).
//! One streaming pass, proximity-filtered. Each physical runway yields two landing
//! options (one per direction). Later stages plan the approach and fly it.

use std::io::{BufRead, BufReader};

pub(crate) use crate::nav::{bearing_deg, haversine_nm, project_latlon};

static TRACKING_ERROR: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);
pub fn last_tracking_error() -> Option<f64> {
    if !ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
        return None;
    }
    *TRACKING_ERROR.lock().unwrap_or_else(|e| e.into_inner())
}

const M_PER_NM: f64 = 1852.0;

/// apt.dat surface codes we will put wheels on.
///
/// 1 and 2 are the legacy asphalt/concrete pair. X-Plane 12 replaced that pair
/// with a MATERIAL system numbered 20-38 and 50-57, and a whitelist of just
/// [1, 2] therefore hid **36% of every usable runway on earth** from the
/// emergency autoland: 4,870 of the 13,390 runways at or above our 1,200 m
/// minimum. Not obscure strips either — EGLL is surface 25 and 29, and PANC,
/// ENGM, EKCH, ESSA, KSEA, OMDB, EBBR, CYYC, EGAA and LTFM are all in the new
/// range. Heathrow was invisible to the autoland.
///
/// Southampton, 2026-07-30: EGHI 20 is surface 22, so it was never a candidate
/// at 2.16 nm and the search took Lasham's legacy asphalt 15.81 nm away. Note
/// this gate gutted BOTH entry points — `runway_by_id` filters on it too, so
/// even the circuit FORCING its own runway would have been rejected here and
/// fallen back to the same wrong field.
///
/// Still a whitelist and not a blacklist, because the soft codes must never
/// win by default: 3 turf, 4 dirt, 5 gravel, 12 dry lakebed, 13 water, 14
/// snow, 15 transparent. 39-49 are unused in the current global apt.dat and
/// are deliberately NOT accepted — an unknown code should not become a runway.
const fn is_hard_surface(s: i32) -> bool {
    matches!(s, 1 | 2 | 20..=38 | 50..=57)
}

/// One landing option: touch down AT `thr`, tracking `course_deg` toward the far end.
#[derive(Debug, Clone, PartialEq)]
pub struct Runway {
    pub icao: String,
    /// Runway identifier for this end (e.g. "27L").
    pub id: String,
    pub thr_lat: f64,
    pub thr_lon: f64,
    /// True course from this threshold toward the far end (degrees, 0-360).
    pub course_deg: f64,
    pub length_m: f64,
    /// apt.dat surface code (1/2 = asphalt/concrete = hard).
    pub surface: i32,
    /// Distance from the query position to this threshold, nm.
    pub dist_nm: f64,
    /// Field elevation, ft MSL (from the airport header) — the glidepath aim.
    pub elev_ft: f64,
}

impl Runway {
    fn hard(&self) -> bool {
        is_hard_surface(self.surface)
    }
}

/// Smallest absolute difference between two headings, degrees (0-180).
pub fn heading_diff(a: f64, b: f64) -> f64 {
    let d = ((a - b) % 360.0 + 540.0) % 360.0 - 180.0;
    d.abs()
}

/// Parse a type-`100` apt.dat line into its two landing ends (threshold lat/lon,
/// id, and the course to fly landing on that end). `None` if the line is malformed.
/// Pure — the streaming pass adds ICAO + distance. Returns (end1, end2) as
/// (id, thr_lat, thr_lon, course_deg), plus (length_m, surface).
fn parse_runway_100(line: &str) -> Option<(RunwayEnd, RunwayEnd, f64, i32)> {
    let f: Vec<&str> = line.split_whitespace().collect();
    if f.len() < 20 || f[0] != "100" {
        return None;
    }
    let surface: i32 = f[2].parse().ok()?;
    let lat1: f64 = f[9].parse().ok()?;
    let lon1: f64 = f[10].parse().ok()?;
    let lat2: f64 = f[18].parse().ok()?;
    let lon2: f64 = f[19].parse().ok()?;
    let length_m = haversine_nm(lat1, lon1, lat2, lon2) * M_PER_NM;
    let end1 = RunwayEnd {
        id: f[8].to_string(),
        lat: lat1,
        lon: lon1,
        course: bearing_deg(lat1, lon1, lat2, lon2),
    };
    let end2 = RunwayEnd {
        id: f[17].to_string(),
        lat: lat2,
        lon: lon2,
        course: bearing_deg(lat2, lon2, lat1, lon1),
    };
    Some((end1, end2, length_m, surface))
}

struct RunwayEnd {
    id: String,
    lat: f64,
    lon: f64,
    course: f64,
}

/// Candidate landing runways within `max_nm` of (lat, lon), nearest threshold
/// first. Streams the whole apt.dat once (cheap per line; ~a second on 363 MB — an
/// emergency arms this once). Later stages filter by suitability and plan the approach.
pub fn nearby_runways(
    apt_dat: &str,
    lat: f64,
    lon: f64,
    max_nm: f64,
) -> std::io::Result<Vec<Runway>> {
    let rdr = BufReader::new(std::fs::File::open(apt_dat)?);
    let mut out: Vec<Runway> = Vec::new();
    let mut icao = String::new();
    let mut elev_ft = 0.0;
    for line in rdr.lines() {
        let line = line?;
        // Cheap first-token check before any allocation-heavy parse.
        let mut toks = line.split_whitespace();
        let rec = toks.next().unwrap_or("");
        match rec {
            "1" | "16" | "17" => {
                // "1 <elev_ft> <tower> <default> <ICAO> <name...>"
                elev_ft = toks.next().and_then(|s| s.parse().ok()).unwrap_or(0.0);
                icao = line.split_whitespace().nth(4).unwrap_or("").to_string();
            }
            "100" => {
                let Some((e1, e2, length_m, surface)) = parse_runway_100(&line) else {
                    continue;
                };
                let width_m: Option<f64> =
                    line.split_whitespace().nth(1).and_then(|s| s.parse().ok());
                for e in [e1, e2] {
                    let dist_nm = haversine_nm(lat, lon, e.lat, e.lon);
                    if dist_nm <= max_nm {
                        if let Some(w) = width_m {
                            note_runway_width(&icao, &e.id, w);
                        }
                        out.push(Runway {
                            icao: icao.clone(),
                            id: e.id,
                            thr_lat: e.lat,
                            thr_lon: e.lon,
                            course_deg: e.course,
                            length_m,
                            surface,
                            dist_nm,
                            elev_ft,
                        });
                    }
                }
            }
            _ => {}
        }
    }
    out.sort_by(|a, b| {
        a.dist_nm
            .partial_cmp(&b.dist_nm)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    Ok(out)
}

/// Pick the best emergency-landing runway near (lat, lon) given the current track:
/// hard-surfaced and long enough, scored by distance plus a penalty for how much
/// heading change it needs (a runway roughly ahead beats one behind you). `None`
/// if nothing suitable is in range. `min_length_m` gates out too-short strips.
pub fn best_landing_runway(
    apt_dat: &str,
    lat: f64,
    lon: f64,
    track_deg: f64,
    max_nm: f64,
    min_length_m: f64,
) -> std::io::Result<Option<Runway>> {
    best_landing_runway_in_wind(
        apt_dat,
        lat,
        lon,
        track_deg,
        max_nm,
        min_length_m,
        &|_| None,
        None,
    )
}

/// What landing at the plan's DESTINATION is worth, nm. "Land the aircraft"
/// with a plan to LIPK on the shelf picked Cervia's 11 over Forli's 30 at the
/// same six miles (2026-09-25): the self-pick knew nothing of where the
/// flight was going. A diversion field still wins when the destination is
/// much further, behind, or downwind.
pub const DESTINATION_BONUS_NM: f64 = 8.0;

/// The shortest strip the self-pick will take, metres: the airframe's landing
/// distance with a 1.3 margin, never under the 1,200 m every airframe was
/// held to. The flat 1,200 offered an A330 (1,448 m before any margin) strips
/// it cannot stop on; the planned path has checked the distance at the arm
/// since 2026-09-15 and the emergency path never did.
pub const SELF_PICK_MIN_LENGTH_M: f64 = 1200.0;
pub fn self_pick_min_length_m(landing_distance_m: Option<f64>) -> f64 {
    landing_distance_m
        .filter(|d| d.is_finite() && *d > 0.0)
        .map_or(SELF_PICK_MIN_LENGTH_M, |d| {
            (d * 1.3).max(SELF_PICK_MIN_LENGTH_M)
        })
}

/// Nautical miles of detour one knot of TAILWIND is worth: ten knots on the
/// tail costs five miles, so the into-wind end of the same runway wins
/// whichever threshold is nearer, and a far field has to be far indeed.
pub const TAILWIND_NM_PER_KT: f64 = 0.5;
/// The most a LONGER strip is worth, nm: decides between parallels (whose
/// thresholds are a few hundred metres apart) and nothing else.
pub const LENGTH_BONUS_MAX_NM: f64 = 0.5;

/// The self-pick's score for one runway end; lower wins. Distance, plus the
/// turn needed to point at the threshold (30 degrees a mile), plus the
/// tailwind, less a small bonus for pavement beyond the minimum. Pure.
pub fn landing_score(
    dist_nm: f64,
    turn_deg: f64,
    course_deg: f64,
    wind: Option<(f64, f64)>,
    length_m: f64,
    min_length_m: f64,
) -> f64 {
    let tailwind = wind.map_or(0.0, |(dir, kts)| {
        (-kts * (dir - course_deg).to_radians().cos()).max(0.0)
    });
    let spare = ((length_m - min_length_m) / 2000.0).clamp(0.0, 1.0) * LENGTH_BONUS_MAX_NM;
    dist_nm + turn_deg / 30.0 + tailwind * TAILWIND_NM_PER_KT - spare
}

/// [`best_landing_runway`] with the surface wind at each candidate's field
/// (`wind_at(icao)` = direction true, knots; `None` = unknown, no penalty).
pub fn best_landing_runway_in_wind(
    apt_dat: &str,
    lat: f64,
    lon: f64,
    track_deg: f64,
    max_nm: f64,
    min_length_m: f64,
    wind_at: &dyn Fn(&str) -> Option<(f64, f64)>,
    destination: Option<&str>,
) -> std::io::Result<Option<Runway>> {
    let mut cands = nearby_runways(apt_dat, lat, lon, max_nm)?;
    // Suitability: hard surface and long enough.
    cands.retain(|r| r.hard() && r.length_m >= min_length_m);
    // One wind lookup per field, not per end.
    let mut winds: std::collections::HashMap<String, Option<(f64, f64)>> = Default::default();
    for r in &cands {
        winds
            .entry(r.icao.clone())
            .or_insert_with(|| wind_at(r.icao.trim()));
    }
    let score = |r: &Runway| {
        let brg = bearing_deg(lat, lon, r.thr_lat, r.thr_lon);
        landing_score(
            r.dist_nm,
            heading_diff(track_deg, brg),
            r.course_deg,
            winds.get(&r.icao).copied().flatten(),
            r.length_m,
            min_length_m,
        ) - if destination.is_some_and(|d| d.eq_ignore_ascii_case(r.icao.trim())) {
            DESTINATION_BONUS_NM
        } else {
            0.0
        }
    };
    let best = cands.iter().min_by(|a, b| {
        score(a)
            .partial_cmp(&score(b))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    Ok(best.cloned())
}

/// The emergency self-pick as autoland arms it: the AIRFRAME's minimum length
/// and the surface wind from each field's METAR (never the aircraft's own
/// wind datarefs: at the arm they are the winds aloft). Blackboxed with the
/// numbers it chose on.
fn self_pick(apt_dat: &str, lat: f64, lon: f64, track_deg: f64) -> std::io::Result<Option<Runway>> {
    let min_length_m = self_pick_min_length_m(crate::aircraft::landing_distance_m());
    let root = crate::tools::autoland::xplane_root();
    let wind_at = |icao: &str| crate::weather::surface_wind(&root, icao);
    let destination = crate::simbrief::loaded_plan().map(|p| p.destination);
    let picked = best_landing_runway_in_wind(
        apt_dat,
        lat,
        lon,
        track_deg,
        40.0,
        min_length_m,
        &wind_at,
        destination.as_deref(),
    )?;
    crate::blackbox::record(
        "autoland",
        serde_json::json!({
            "event": "self_pick", "min_length_m": min_length_m, "destination": destination,
            "rwy": picked.as_ref().map(|r| format!("{} {}", r.icao.trim(), r.id)),
            "length_m": picked.as_ref().map(|r| r.length_m),
            "wind": picked.as_ref().and_then(|r| wind_at(r.icao.trim())),
        }),
    );
    Ok(picked)
}

/// The runway an ARRIVAL chose, and the airport it belongs to.
///
/// The two travel together, and that is the entire point: **a runway ident is
/// not unique**. Handed only "24", the lookup below resolved it by proximity
/// and found EGPJ (Fife) instead of the planned EGPH (Edinburgh), nine miles
/// away across the Forth — a plan with a destination in it flew an ILS to a
/// different airport (2026-09-20). When a plan names where we are going, the
/// airport is not a search result.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedRunway {
    /// The runway END ("24", "27L").
    pub id: String,
    /// The airport that end belongs to. `None` only where nothing named one —
    /// a spoken "land the aircraft" with no plan — and only then may the
    /// ident be resolved by proximity.
    pub icao: Option<String>,
}

impl PlannedRunway {
    pub fn at(icao: &str, id: &str) -> Self {
        PlannedRunway {
            id: id.to_string(),
            icao: Some(icao.to_string()),
        }
    }
    /// An ident with no airport behind it — proximity is all there is.
    pub fn anywhere(id: &str) -> Self {
        PlannedRunway {
            id: id.to_string(),
            icao: None,
        }
    }
}

/// The specific hard runway END matching `id` within `max_nm`, AT `icao` when
/// one is given — used when the arrival has ALREADY chosen the runway (by
/// wind) and autoland must honour that end instead of re-picking by track.
/// `None` if no such end is in range (the caller falls back to
/// [`best_landing_runway`]).
///
/// With `icao: None` this is the old nearest-ident behaviour, which is right
/// for a self-pick and wrong for anything that knows its destination.
pub fn runway_by_id(
    apt_dat: &str,
    lat: f64,
    lon: f64,
    id: &str,
    max_nm: f64,
    icao: Option<&str>,
) -> std::io::Result<Option<Runway>> {
    let want = id.trim();
    let at = icao.map(str::trim);
    // nearby_runways is already nearest-threshold-first.
    let cands = nearby_runways(apt_dat, lat, lon, max_nm)?;
    Ok(cands.into_iter().find(|r| {
        r.hard() && r.id.trim() == want && at.is_none_or(|a| r.icao.trim().eq_ignore_ascii_case(a))
    }))
}

// ---- Stage 3: the control loop (AP-first, like the reference plugin) ----------

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

/// Set true to cancel an active autoland ("cancel autoland" / "my controls").
pub static ABORT: AtomicBool = AtomicBool::new(false);
/// True while an autoland is flying (so cancel can acknowledge it).
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// This approach's landing configuration (spoilers, autobrake) is set: the
/// `landing_config` event. Cleared when an approach starts and ends.
static LANDING_CONFIGURED: AtomicBool = AtomicBool::new(false);

/// Has the approach flying now set its landing configuration? The Monitor
/// BEFORE LANDING list waits for it under an automation
/// (`flow::landing_config_for`): a circuit hands to autoland on the final,
/// and the list audited the A330's spoilers and autobrake 48 s before
/// autoland set them (LEMG 30, sortie 1790633832-full_auto, #169).
pub fn landing_configured() -> bool {
    LANDING_CONFIGURED.load(Ordering::SeqCst)
}
static GEN: AtomicU64 = AtomicU64::new(0);

/// One-line emergency status for the panel's live strip ("AUTOLAND LELC 23R
/// 12.4nm FINAL"); empty when nothing is flying. Written by the autoland and
/// recover watchers, composed into the strip by the monitor each tick.
pub static PANEL: std::sync::Mutex<String> = std::sync::Mutex::new(String::new());

/// Set/clear the emergency line (empty = clear). Swallows a poisoned lock — the
/// strip is display-only.
pub fn set_panel(text: &str) {
    if let Ok(mut p) = PANEL.lock() {
        text.clone_into(&mut p);
    }
}

// State reads.
const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
/// TRUE altitude (meters MSL), not the baro-indicated `h_ind`: the glidepath
/// aims at the runway's TRUE elevation from apt.dat, so a misset altimeter
/// displaces the whole path by the baro error. Seen live 2026-07-14 (LXGB 09):
/// h_ind read 0.5 ft with 47 ft of radio altitude over the bay — the vertical
/// law flew the final against an altitude ~46 ft low.
const ELEV_M: &str = "sim/flightmodel/position/elevation";
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
/// Weight-on-wheels for the player aircraft. Radio altitude and airspeed are
/// not touchdown evidence: a normal flare can be below five feet and a knot
/// below target while the wheels are still airborne.
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
/// TRUE heading and TRUE ground track: their difference is the wind crab the
/// lateral law feeds forward on final (heading vs track, the reference recipe).
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const HPATH_TRUE: &str = "sim/flightmodel/position/hpath";
/// Bank, for the arrest's wings-level law once the servos are off.
const PHI_TRUE: &str = "sim/flightmodel/position/true_phi";
/// Yaw rate, deg/s nose right, for a declared rollout steer (`RolloutSteer`):
/// read through the state frame, `sim::state::F::R` (`position/R`).
/// The vertical speed actually flown (the pilot's VSI). The shared name, not a
/// local spelling: `cockpit2/gauges/indicators/vvi_dial_fpm` does not exist
/// in X-Plane 12, and from 2026-07 to 2026-09-14 every read of it failed and
/// fell back to the COMMANDED rate, so the VS watchdog below could never fire
/// and the landing arrest's lead (`vs_dial_fpm`) saw zero error on every tick
/// of the LXGB 09 arrival it was written for (339 fpm at the wheels).
const VVI_FPM: &str = crate::datarefs::VVI_FPM;
// AP + controls (mirror of autopilot.rs / recover.rs).
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
const CMD_SERVOS_OFF: &str = "sim/autopilot/servos_off_any";
/// Servos AND flight director off: the wheels' disconnect.
const CMD_SERVOS_FDIR_OFF: &str = "sim/autopilot/servos_fdir_off";
const CMD_YAW_DAMPER_OFF: &str = "sim/systems/yaw_damper_off";
/// The autopilot master as the sim has it (power and failures included).
const AP_MASTER_ON: &str = "sim/cockpit2/autopilot/autopilot_on";
const YAW_DAMPER_ON: &str = "sim/cockpit2/switches/yaw_damper_on";
const SERVOS_ON: &str = "sim/cockpit2/autopilot/servos_on";
const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
const HDG_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
const HDG_STATUS: &str = "sim/cockpit2/autopilot/heading_status";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
const ALT_IND_FT: &str = crate::datarefs::ALTITUDE_FT;
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}
const GEAR_HANDLE: &str = "sim/cockpit2/controls/gear_handle_down";
const FLAP_RATIO: &str = "sim/cockpit2/controls/flap_ratio";
// Rollout: nose steering + proportional wheel brakes through the plugin, the
// same channels the taxi governor drives.
const TAXI_STEER: &str = "xcopilot/taxi_steer";
const TAXI_BRAKE: &str = "xcopilot/taxi_brake";
use sim::state::F;
/// What a tick reads past the position (`sim::state`; each field names its
/// dataref): the constants above (`MAG_VAR`, `IAS`, `HPATH_TRUE`,
/// `ON_GROUND`, `PHI_TRUE`, `VVI_FPM`, `SERVOS_ON`, `HDG_STATUS`,
/// `VS_STATUS`, `HDG_BUG`) and the plugin's yields.
const AUTOLAND_WANT: &[F] = &[
    F::MagVar,
    F::Ias,
    F::Hpath,
    F::OnGround,
    F::Phi,
    F::Vvi,
    F::Servos,
    F::HdgStatus,
    F::VsStatus,
    F::Bug,
    F::PitchYield,
    F::RollYield,
    F::R,
];
/// ...and with the ILS tuned, its needles and modes (`crate::ils`).
const AUTOLAND_WANT_ILS: &[F] = &[
    F::MagVar,
    F::Ias,
    F::Hpath,
    F::OnGround,
    F::Phi,
    F::Vvi,
    F::Servos,
    F::HdgStatus,
    F::VsStatus,
    F::Bug,
    F::PitchYield,
    F::RollYield,
    F::R,
    F::Hdef,
    F::Vdef,
    F::LocValid,
    F::GsValid,
    F::GsStatus,
    F::ApprStatus,
];
/// A lever read this close to the last command took it (`lever_took`).
const LEVER_TOOK_TOL: f64 = 0.01;
/// The gentler brake hold once stopped.
const ROLLOUT_BRAKE_HOLD: f64 = 0.5;

/// The line at a full stop, from the PARKING brake's read-back: "Brakes on"
/// only when it reads set. A brake that did not take is named, because the
/// rollout's squeeze expires within a second of the hand-off and the aircraft
/// is then the pilot's to hold (EGSS 2026-09-22: said on, read 0.03).
pub(crate) fn stopped_line(parking_brake_held: bool) -> &'static str {
    if parking_brake_held {
        "Down and stopped. Brakes on, your aircraft."
    } else {
        "Down and stopped. The parking brake did not set, hold the brakes. Your aircraft."
    }
}
/// Throttle a give-up hands back with: enough to hold level flight with gear
/// down and landing flap out, which is the state a balked landing leaves the
/// aircraft in. Half throttle was not (110 → 80 kt, EGHI 2026-08-02).
const GIVE_UP_THROTTLE: f64 = 0.75;
/// The VS watchdog: ticks of >700 fpm divergence before the warning (about
/// five seconds) and before the give-up (about a minute — long enough for a
/// re-engaged autopilot to catch the dial, short enough that a descent that
/// is really a climb does not become a cruise).
///
/// In SECONDS since 2026-09-30: they were 12 and 140 ticks, which is 4 s
/// and 46 s at the MacBook's 0.33 s Final tick the watchdog was tuned on
/// (6.7 s and 78 s at the iMac's 0.56 s).
const VS_DIVERGED_WARN_S: f64 = 4.0;
/// How far the vertical speed may sit from the command before the watchdog
/// looks at it, and how fast the gap must be CLOSING for the autopilot to
/// count as holding. The gap alone is a 172 number: the A330's VS mode takes
/// longer than the watchdog's five seconds to reach a -1,200 step from
/// level, and "not holding the descent" was said at the arm of an approach
/// the autopilot was flying correctly, just slowly (EGPF 23, 2026-09-20).
pub const VS_DIVERGED_FPM: f64 = 700.0;
/// Per SECOND since 2026-09-30: it was 15 fpm a tick, 45 fpm/s at the
/// MacBook's 0.33 s Final tick (27 fpm/s at the iMac's 0.56 s).
pub const VS_CONVERGING_FPM_PER_S: f64 = 45.0;

/// Is the autopilot NOT holding the commanded vertical speed? Far from it,
/// and not closing on it. `prev_err` is last tick's error (None = first
/// sight, which counts: there is no trend to excuse it yet).
/// `dt_s` is the time since that error was read.
pub fn vs_not_held(err_fpm: f64, prev_err_fpm: Option<f64>, dt_s: f64) -> bool {
    if err_fpm.abs() <= VS_DIVERGED_FPM {
        return false;
    }
    prev_err_fpm
        .is_none_or(|p| err_fpm.abs() > p.abs() - VS_CONVERGING_FPM_PER_S * dt_s.max(0.0))
}

/// The watchdog's words name what is actually commanded. "Not holding the
/// descent" was said four times in a missed approach climbing at 970 fpm.
pub fn vs_not_held_line(target_vs_fpm: f64) -> &'static str {
    if target_vs_fpm < -50.0 {
        "Autopilot is not holding the descent. Watch the pitch, be ready to take it."
    } else if target_vs_fpm > 50.0 {
        "Autopilot is not holding the climb. Watch the pitch, be ready to take it."
    } else {
        "Autopilot is not holding level. Watch the pitch, be ready to take it."
    }
}
const VS_DIVERGED_GIVE_UP_S: f64 = 46.0;

/// THE AUTOPILOT IS ENGAGED AND CONFIRMED, never assumed. The arm used to
/// fire `servos_on` once and never read it back. After `setup_approach` the
/// sim is still starting the new flight when autoland arms (IAS reads 0 to
/// 12 kt for the first second), and that one press was lost: LEMG 13
/// (sortie 1790587073-approach, 2026-09-28) flew with `ap_state` 132 and the
/// servos off from the arm until the VS watchdog's re-press at +9.5 s, the
/// aircraft flown by nobody while autoland commanded a climb: -900 fpm at
/// the start growing to -3,450, 2,318 to 1,697 ft MSL, 405 ft AGL over the
/// rising ground. Two of six runs lost the press the same way (1790583434,
/// 1790583663 reached 200 and 145 ft AGL); the re-arms that engaged did so
/// on the re-arm's own press.
///
/// So the press is re-fired every [`ENGAGE_RETRY_S`] until the servos READ
/// on for [`ENGAGE_CONFIRM_READS`] ticks running with the sim live (an
/// airspeed that is a flight's, [`ENGAGE_LIVE_IAS_KTS`]: the flight start
/// can drop an engage made before it), each try blackboxed `ap_engage`. A
/// failed read is no measurement: it neither confirms nor breaks the run.
/// Until confirmed, with the servos read OFF, the loop holds the aircraft
/// level itself on the elevator channel ([`engage_hold_gains`]). No
/// confirmation within [`ENGAGE_GIVE_UP_S`] of live flight is said and the
/// approach handed over.
#[derive(Debug, Clone, Default)]
pub struct ApEngage {
    tries: u32,
    last_fire_s: Option<f64>,
    live_since_s: Option<f64>,
    on_reads: u32,
    /// The last GOOD servo read (true = on); `None` = never read.
    last_read: Option<bool>,
    confirmed: bool,
}

/// Seconds between presses while the servos do not read on: every tick at
/// the loop's 0.6 s (`servos_on` is not a toggle).
pub const ENGAGE_RETRY_S: f64 = 0.5;
/// Consecutive live ticks the servos must read on to be confirmed.
pub const ENGAGE_CONFIRM_READS: u32 = 2;
/// An airspeed that is a flight's, not a flight still being started.
pub const ENGAGE_LIVE_IAS_KTS: f64 = 30.0;
/// Seconds of live flight without a confirmed engage before the approach is
/// handed over.
pub const ENGAGE_GIVE_UP_S: f64 = 10.0;

/// What the engage watch wants this tick.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EngageStep {
    /// Press the flight director and `servos_on` now (try number `try_n`).
    pub fire: bool,
    pub try_n: u32,
    /// Hold the aircraft on the elevator channel this tick: not confirmed,
    /// and the servos last READ off.
    pub hold: bool,
    /// Confirmed on this tick (the record is written once).
    pub confirmed_now: bool,
    /// No engage within the bound: say it and hand over.
    pub give_up: bool,
}

impl ApEngage {
    /// The press at the arm (try 1): the arm fires before the first tick.
    pub fn arm(&mut self, now_s: f64) -> EngageStep {
        self.tries = 1;
        self.last_fire_s = Some(now_s);
        EngageStep {
            fire: true,
            try_n: 1,
            hold: false,
            ..Default::default()
        }
    }

    pub fn confirmed(&self) -> bool {
        self.confirmed
    }

    pub fn tries(&self) -> u32 {
        self.tries
    }

    /// One tick. `servos` is the `servos_on` read (`None` = failed read),
    /// `ias_kts` the airspeed read this tick.
    pub fn step(&mut self, now_s: f64, servos: Option<f64>, ias_kts: f64) -> EngageStep {
        if self.confirmed {
            return EngageStep::default();
        }
        let live = ias_kts.is_finite() && ias_kts >= ENGAGE_LIVE_IAS_KTS;
        if live && self.live_since_s.is_none() {
            self.live_since_s = Some(now_s);
        }
        match servos {
            Some(v) if v > 0.5 => {
                self.last_read = Some(true);
                if live {
                    self.on_reads += 1;
                } else {
                    self.on_reads = 0;
                }
            }
            Some(_) => {
                self.last_read = Some(false);
                self.on_reads = 0;
            }
            None => {}
        }
        if self.on_reads >= ENGAGE_CONFIRM_READS {
            self.confirmed = true;
            return EngageStep {
                confirmed_now: true,
                try_n: self.tries,
                ..Default::default()
            };
        }
        if self
            .live_since_s
            .is_some_and(|t0| now_s - t0 >= ENGAGE_GIVE_UP_S)
        {
            return EngageStep {
                give_up: true,
                try_n: self.tries,
                ..Default::default()
            };
        }
        let due = self
            .last_fire_s
            .is_none_or(|t| now_s - t >= ENGAGE_RETRY_S - 1e-9);
        // Pressed while the servos last READ off, or were never read; a
        // failed read after an "on" waits for the next measurement.
        let fire = due && self.last_read != Some(true);
        if fire {
            self.tries += 1;
            self.last_fire_s = Some(now_s);
        }
        EngageStep {
            fire,
            try_n: self.tries,
            hold: self.last_read == Some(false),
            ..Default::default()
        }
    }
}

/// The elevator law that holds the aircraft LEVEL until the autopilot is
/// confirmed: the flare's free tracker ([`FlarePitch`] outside the
/// ratchet) at two thirds of its default gain and a wider pull. Not the
/// airframe's flare numbers: those are sized for the last forty feet (the
/// 737's is pull-only to 0.45), and the recorded new-flight start at LEMG 13
/// needs about 0.7 of standing pull to hold level (fitted to 1790587073:
/// -900 fpm at the start plus 0.54 of nose-down mistrim through the 737's
/// flare plant, 112 fpm rms). Swept against that plant and the LEMG 12 and
/// fastest flare plants at the loop's 0.6 s tick: the least height lost
/// with the least ringing.
pub fn engage_hold_gains() -> FlareGains {
    FlareGains {
        p: 0.04,
        d: 0.05,
        trim_rate: 0.015,
        max: 0.8,
        min: -0.2,
        balloon_ease_per_s: None,
    }
}

/// The words when the autopilot will not come in.
pub const ENGAGE_GIVE_UP_LINE: &str =
    "The autopilot will not engage. Abandoning the approach. Your aircraft.";
/// Past the arm distance by this much, still not on a missed approach, the
/// approach is abandoned: the aircraft is leaving, not arriving.
const RECEDING_GIVE_UP_NM: f64 = 5.0;
const M_TO_FT: f64 = 3.280_84;
const MS_TO_KTS: f64 = 1.943_84;

/// The deceleration a rollout aims for when the runway leaves the choice open,
/// m/s². A flat 0.45 brake ratio measured ~3.3 m/s² on the Baron — maximum
/// autobrake territory, applied from touchdown speed to a dead stop, and the
/// pilot called it "brakes really really hard" (EGHI, 2026-08-02). Real
/// autobrake LOW is ~1.2-1.8; this errs on the firm side of comfortable
/// because the distance term below, not this figure, is what guarantees the
/// stop.
const ROLLOUT_COMFORT_DECEL: f64 = 1.6;
/// Brake ratio per m/s² of wanted deceleration — calibrated from the same
/// measurement (0.45 ratio ≈ 3.3 m/s² including aero drag, so the wheel share
/// is a little under that).
const BRAKE_RATIO_PER_MPS2: f64 = 0.15;
/// The rollout hands off to a following vacate at or below this groundspeed —
/// taxi pace, never a stop. Stopping dead on an active runway and then
/// re-accelerating to leave it was neither procedure nor comfortable.
const VACATE_HANDOFF_KTS: f64 = 10.0;

/// Brake ratio for the rollout: a comfortable deceleration, unless stopping
/// within the pavement actually remaining demands more. `target_kts` is the
/// speed the rollout is aiming for — zero for a full stop, taxi pace when a
/// vacate follows.
fn rollout_brake_ratio(gs_kts: f64, target_kts: f64, remaining_m: f64) -> f64 {
    let v = (gs_kts.max(0.0)) / MS_TO_KTS;
    let vt = (target_kts.max(0.0)) / MS_TO_KTS;
    // What physics requires to be at target speed with 30% of the remaining
    // runway still in hand — the safety term, immune to the comfort figure.
    let usable = (remaining_m * 0.7).max(30.0);
    let required = ((v * v - vt * vt) / (2.0 * usable)).max(0.0);
    let decel = required.max(ROLLOUT_COMFORT_DECEL);
    (decel * BRAKE_RATIO_PER_MPS2).clamp(0.2, 0.8)
}

/// The rollout's differential brake on an airframe that steers on its brakes
/// (a castoring nosewheel, `taxi.brake_steer`), as the real aircraft is
/// steered down the runway: the stopping brake moved from the outside wheel
/// to the inside one, the total unchanged.
///
/// LEMG 13 (issue #166, 2026-09-28, the SR22, sortie 1790619465): the
/// rollout steered on the rudder alone with both brakes at 0.24. The nose
/// swung 6 degrees right in the first two seconds on the wheels, and for ten
/// seconds from 70 kt it sat 3 to 6 degrees right of the runway while the
/// steer asked for left, growing to 0.69 and the trim to its 0.3 cap: fitted
/// over those ticks the rudder's yaw is under 2 deg/s per unit of steer, no
/// authority at all, and the aircraft rolled from 1.6 m to 12.1 m right of
/// the centreline. The takeoff's split (`autotakeoff::roll_brake_split`)
/// fades out from 20 to 40 kt because a brake on the roll costs the
/// acceleration; on the rollout the brake is wanted anyway, so it steers at
/// every speed. The heading error is the takeoff roll's target less the
/// heading (`roll_target_heading`), damped on its own rate; the SR22's taxi
/// ticks give its brakes 40 to 75 deg/s of yaw per unit of difference at
/// taxi pace, and the gain holds the recorded rollout inside 2.6 m from 15 to
/// 60 deg/s per unit.
///
/// On a FREE castoring nose (`aircraft::nose_free`: the SR22, the Evolution,
/// the RV-10) the difference is the one ground law's (`ground_steer`): the
/// identified plant inverted over the tick, the rollout's rudder a known
/// input, the standing yaw (the idle propeller's) learnt by its observer.
/// It replaces the SR22's declared split (`rollout_brakes:`, a gain
/// schedule, lead and trim fitted to LEMG 13 1790689843 and 1790694457,
/// 2026-09-29) and the shared law the Evolution flew into 8.3 m at LILB 03
/// (1790762771, 2026-09-30: 0.14 of difference at 0.75 of stop, the nose
/// 3.7 degrees right of its target for six seconds from 66 kt).
#[derive(Debug, Default, Clone)]
pub struct RolloutBrakeSteer {
    prev_err: Option<f64>,
    rate: f64,
    /// The one ground law on a free castoring nose, with the last heading
    /// (its yaw rate is the heading differenced over the tick).
    free: Option<(crate::ground_steer::GroundSteer, Option<f64>)>,
    /// The free law's last tick, for the record.
    last: Option<crate::ground_steer::Steer>,
}

/// Brake difference per degree of heading error...
const ROLLOUT_BRAKE_STEER_PER_DEG: f64 = 0.03;
/// ...led by this many seconds of the error's rate...
const ROLLOUT_BRAKE_STEER_LEAD_S: f64 = 0.8;
/// ...never more than this difference between the wheels...
const ROLLOUT_BRAKE_STEER_MAX: f64 = 0.3;
/// ...and none below this groundspeed, where the stop owns the brakes.
const ROLLOUT_BRAKE_STEER_MIN_KTS: f64 = 4.0;
/// A free nose's most wheel difference on the rollout: an inside wheel at
/// full brake takes the rest off the outside (`ground_steer::stop_wheels`).
const ROLLOUT_FREE_DIFF_MAX: f64 = 0.5;

/// One tick of [`RolloutBrakeSteer`]: the brakes, as (the symmetric channel,
/// the left wheel, the right wheel). The plugin gives each wheel the harder
/// of the symmetric channel and its own, so the outside wheel's brake is the
/// symmetric channel and the inside one's its own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RolloutBrakes {
    pub sym: f64,
    pub left: f64,
    pub right: f64,
}

impl RolloutBrakeSteer {
    /// The law for the live airframe: the one ground law on a free
    /// castoring nose, else the shared split.
    pub fn for_airframe() -> Self {
        if crate::aircraft::nose_free() {
            Self::free(crate::ground_steer::GroundPlant::for_airframe())
        } else {
            Self::default()
        }
    }

    /// The one ground law on this plant.
    pub fn free(plant: crate::ground_steer::GroundPlant) -> Self {
        Self {
            free: Some((crate::ground_steer::GroundSteer::new(plant), None)),
            ..Self::default()
        }
    }

    /// The learnt standing yaw (the free law's, deg/s^2), for the record.
    pub fn trim(&self) -> f64 {
        self.free.as_ref().map_or(0.0, |(law, _)| law.w_hat())
    }

    /// The free law's last tick, for the record.
    pub fn last(&self) -> Option<crate::ground_steer::Steer> {
        self.last
    }

    /// The heading error (degrees, target right of the nose positive), the
    /// tick, groundspeed and the stopping brake the rollout wants.
    pub fn step(&mut self, err_deg: f64, dt_s: f64, gs_kts: f64, brake: f64) -> RolloutBrakes {
        self.step_with(err_deg, None, None, 0.0, dt_s, gs_kts, brake)
    }

    /// [`Self::step`] with the heading and the track (true degrees: the free
    /// law's yaw rate and the slip its tail weathervanes on) and the rudder
    /// written this tick (echoed; the roll law's).
    #[allow(clippy::too_many_arguments)]
    pub fn step_with(
        &mut self,
        err_deg: f64,
        psi_deg: Option<f64>,
        track_deg: Option<f64>,
        rudder: f64,
        dt_s: f64,
        gs_kts: f64,
        brake: f64,
    ) -> RolloutBrakes {
        let brake = if brake.is_finite() {
            brake.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let plain = RolloutBrakes {
            sym: brake,
            left: 0.0,
            right: 0.0,
        };
        if !err_deg.is_finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            // A failed read is no measurement: the plain stop.
            return plain;
        }
        if let Some((law, prev_psi)) = self.free.as_mut() {
            let mean = match (psi_deg, *prev_psi) {
                (Some(p), Some(q)) => ((p - q + 540.0).rem_euclid(360.0) - 180.0) / dt_s.clamp(0.05, 3.0),
                _ => 0.0,
            };
            *prev_psi = psi_deg;
            if !gs_kts.is_finite() || gs_kts < ROLLOUT_BRAKE_STEER_MIN_KTS {
                law.release();
                return plain;
            }
            let _ = track_deg;
            let s = law.step(crate::ground_steer::Demand {
                io_delay_s: crate::ground_steer::ROLLOUT_IO_DELAY_S,
                ..crate::ground_steer::heading_demand(
                    dt_s,
                    gs_kts / MS_TO_KTS,
                    crate::ground_steer::Yaw::Mean(mean),
                    err_deg,
                    crate::ground_steer::Rudder::Given(rudder),
                    ROLLOUT_FREE_DIFF_MAX,
                    crate::aircraft::live_profile()
                        .and_then(|p| p.taxi.as_ref().and_then(|t| t.max_yaw_dps))
                        .unwrap_or(crate::ground_steer::DEFAULT_MAX_YAW_DPS),
                )
            });
            self.last = Some(s);
            let (sym, left, right) = crate::ground_steer::stop_wheels(brake, s.diff);
            return RolloutBrakes { sym, left, right };
        }
        if let Some(p) = self.prev_err {
            self.rate = (err_deg - p) / dt_s.clamp(0.05, 3.0);
        }
        self.prev_err = Some(err_deg);
        if !gs_kts.is_finite() || gs_kts < ROLLOUT_BRAKE_STEER_MIN_KTS {
            return plain;
        }
        let diff = (ROLLOUT_BRAKE_STEER_PER_DEG * (err_deg + ROLLOUT_BRAKE_STEER_LEAD_S * self.rate))
            .clamp(-ROLLOUT_BRAKE_STEER_MAX, ROLLOUT_BRAKE_STEER_MAX);
        let half = diff.abs() / 2.0;
        let outside = (brake - half).max(0.0);
        let inside = (brake + half).min(1.0);
        if diff > 0.0 {
            // Nose right: the right wheel brakes harder.
            RolloutBrakes {
                sym: outside,
                left: 0.0,
                right: inside,
            }
        } else if diff < 0.0 {
            RolloutBrakes {
                sym: outside,
                left: inside,
                right: 0.0,
            }
        } else {
            plain
        }
    }
}

/// The rollout's steer on a STEERED nosewheel that declares its own law
/// (`rollout_steer:`, [`crate::aircraft::RolloutSteerProfile`], issue #197);
/// every other airframe keeps the takeoff roll's default law. Pure.
///
/// LEMG 13 (iMac 2026-09-30, the King Air C90B): every contact yawed the
/// nose 8 to 15 deg/s left with nothing steering, and the default law, its
/// whole authority 0.35 at 80 kt, answered on a heading DIFFERENCED over
/// a 0.45 s tick: 0.35 of right steer for a second and a half, the nose
/// swung 30 to 80 degrees right, and full left steer held none of it until
/// 45 kt (1790757702 off the runway, 176 m right). Fitted over the three
/// rollouts the steer is 45 to 65 deg/s per unit at 80 kt after 0.3 s and
/// the slip between the nose and the track feeds its own yaw: the swing is
/// seen first in the yaw RATE, so the law damps on the rate the sim
/// measures (`sim/flightmodel/position/R`), holds its gain low at speed and
/// raises it as `ref_kts / gs` toward the nosewheel's own steering.
#[derive(Debug, Clone, Copy)]
pub struct RolloutSteer {
    prof: crate::aircraft::RolloutSteerProfile,
    prev_err: Option<f64>,
    out: f64,
}

/// The rollout steer's slew, units per second (the takeoff roll's).
const ROLLOUT_STEER_SLEW_PER_S: f64 = 1.5;
/// The rollout steer's authority under `min_kts` (the takeoff roll's).
const ROLLOUT_STEER_MAX: f64 = 0.85;

impl RolloutSteer {
    pub fn new(prof: crate::aircraft::RolloutSteerProfile) -> Self {
        Self {
            prof,
            prev_err: None,
            out: 0.0,
        }
    }

    /// The gain per degree at this groundspeed.
    pub fn per_deg_at(&self, gs_kts: f64) -> f64 {
        let p = self.prof;
        let v = if gs_kts.is_finite() { gs_kts } else { p.ref_kts };
        p.per_deg * p.ref_kts / v.clamp(p.min_kts, p.ref_kts)
    }

    /// The most steer at this groundspeed: the full authority at `min_kts`
    /// and under, `max_fast` from `fast_kts`.
    pub fn cap_at(&self, gs_kts: f64) -> f64 {
        let p = self.prof;
        let v = if gs_kts.is_finite() { gs_kts } else { p.fast_kts };
        let share = ((v - p.min_kts) / (p.fast_kts - p.min_kts)).clamp(0.0, 1.0);
        ROLLOUT_STEER_MAX + (p.max_fast - ROLLOUT_STEER_MAX) * share
    }

    /// Off the wheels (a bounce): nothing is written, so the next contact
    /// slews from neutral.
    pub fn in_air(&mut self) {
        self.out = 0.0;
        self.prev_err = None;
    }

    /// The heading error (degrees, target right of the nose positive), the
    /// tick, groundspeed and the sim's yaw rate (deg/s, nose right
    /// positive; `None` on a failed read, when the error's own rate stands
    /// in).
    pub fn step(&mut self, err_deg: f64, dt_s: f64, gs_kts: f64, yaw_dps: Option<f64>) -> f64 {
        if !err_deg.is_finite() {
            return self.out;
        }
        let dt = if dt_s.is_finite() { dt_s.clamp(0.05, 1.0) } else { 0.4 };
        let differenced = self.prev_err.map_or(0.0, |p| -(err_deg - p) / dt);
        self.prev_err = Some(err_deg);
        let yaw = yaw_dps.filter(|y| y.is_finite()).unwrap_or(differenced);
        let cap = self.cap_at(gs_kts);
        let want =
            (self.per_deg_at(gs_kts) * (err_deg - self.prof.damp_s * yaw)).clamp(-cap, cap);
        let step = ROLLOUT_STEER_SLEW_PER_S * dt;
        self.out += (want - self.out).clamp(-step, step);
        self.out
    }
}

/// The rollout's view of the wheels, tick by tick: on them, off them after
/// the latch (a bounce, blackboxed `bounce` once per spell with its height),
/// or down again. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct BounceHold {
    in_air: bool,
    spells: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BounceTick {
    OnWheels,
    /// The first airborne tick of a spell.
    Left,
    InAir,
    /// The first tick back on the wheels after a spell.
    Landed,
}

impl BounceHold {
    pub fn tick(&mut self, airborne: bool) -> BounceTick {
        let t = match (self.in_air, airborne) {
            (false, false) => BounceTick::OnWheels,
            (false, true) => {
                self.spells += 1;
                BounceTick::Left
            }
            (true, true) => BounceTick::InAir,
            (true, false) => BounceTick::Landed,
        };
        self.in_air = airborne;
        t
    }

    /// Spells in the air since the latch.
    pub fn spells(&self) -> u32 {
        self.spells
    }
}

/// A light single HELD ON after its first contact (a hold-off flare,
/// `flare_elevator.model.hold_off_ft`, issue #166). From the first tick the
/// wheels read on, or the radio under [`TOUCH_RADIO_FT`], the column holds
/// the pull it touched with: never more (a pull added on the wheels lifts a
/// wing that still flies back off) and never less (no push, and no let-go:
/// the flare's balloon ease and the rollout's expiring channel took the
/// SR22's 0.10 to neutral within a second, and at 75 to 80 kt neutral
/// floated it 3 to 10 ft; LEMG 13 1790702352, 1790702623, 2026-09-29), with
/// the lever closed (never the balloon's approach thrust: each of those hops
/// had it back at 0.265). It lets go only on the wheels, slower than
/// [`TOUCH_HOLD_RELEASE_KTS`], easing to neutral at
/// [`TOUCH_HOLD_EASE_PER_S`]. A go-around forgets it. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct TouchHold {
    held: Option<f64>,
    released: bool,
}

/// Under this radio height a hold-off is on the runway, ft: at rest the
/// SR22's radio reads 0.1 to 0.2, and a contact between two 0.45 s ticks
/// was read at 0.3 ft on its way back up with no wheels flag (1790702352).
pub const TOUCH_RADIO_FT: f64 = 0.5;
/// The held pull goes on the wheels below this groundspeed, kt: the hops of
/// the four iMac SR22 rollouts died out between 54 and 60 kt.
pub const TOUCH_HOLD_RELEASE_KTS: f64 = 45.0;
/// ...eased to neutral this fast, elevator per second.
pub const TOUCH_HOLD_EASE_PER_S: f64 = 0.05;

impl TouchHold {
    /// This tick's contact (`on_wheels`, or the radio under
    /// [`TOUCH_RADIO_FT`]) with the elevator last written; true on the
    /// first one, which takes that pull (a push is taken as neutral).
    pub fn note(&mut self, contact: bool, written: Option<f64>) -> bool {
        if self.held.is_some() || !contact {
            return false;
        }
        self.held = Some(written.filter(|e| e.is_finite()).unwrap_or(0.0).max(0.0));
        true
    }

    /// The wheels have touched this approach.
    pub fn touched(&self) -> bool {
        self.held.is_some()
    }

    /// The pull held, for the blackbox.
    pub fn held(&self) -> Option<f64> {
        self.held
    }

    /// The column this tick: the pull held; on the wheels below the release
    /// speed eased toward neutral; `None` once let go (the channel expires
    /// by itself) or before any contact.
    pub fn elev(&mut self, gs_kts: f64, on_wheels: bool, dt: f64) -> Option<f64> {
        let h = self.held?;
        if self.released {
            return None;
        }
        if on_wheels && gs_kts.is_finite() && gs_kts < TOUCH_HOLD_RELEASE_KTS {
            let next = h - TOUCH_HOLD_EASE_PER_S * dt.clamp(0.0, 1.0);
            if next <= 0.0 {
                self.released = true;
                self.held = Some(0.0);
                return Some(0.0);
            }
            self.held = Some(next);
            return Some(next);
        }
        Some(h)
    }

    /// A go-around: the next approach starts free.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Whether the live airframe steers on its brakes (the takeoff's own test).
fn rollout_brake_steers() -> bool {
    crate::aircraft::nose_castors()
        || crate::aircraft::live_profile()
            .and_then(|p| p.taxi.as_ref().map(|t| t.brake_steer))
            .unwrap_or(false)
}

/// Set by full auto before its landing: a vacate drive follows this rollout,
/// so end it at taxi pace instead of a dead stop. Consumed (swapped false) by
/// the rollout that honours it and cleared on abort, so a standalone "land for
/// me" — where nobody follows — always brakes to a stop as before.
pub static VACATE_FOLLOWS: AtomicBool = AtomicBool::new(false);

/// The runway this autoland actually rolled out on, set at rollout entry.
///
/// Where we ARE, as against where a clearance says we should be — and for
/// vacating, being on the strip is the whole of the question. A landing
/// clearance can be missing for reasons that have nothing to do with the
/// pavement under the wheels: at Malaga the tower ordered a go-around, which
/// voided the clearance, the second approach was never re-cleared, and the
/// vacate — gated on a STANDING clearance — refused. The aircraft braked to
/// a stop on runway 13 and sat there through four incursion warnings with an
/// arrival closing to 1.3 nm (2026-08-05). Landing without a clearance makes
/// clearing the runway more urgent, not less.
static LANDED_RUNWAY: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

/// The runway the last autoland rolled out on, if it has not been consumed.
pub fn landed_runway() -> Option<String> {
    LANDED_RUNWAY
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
}

/// Wheels on: the autopilot's approach is over and its servos come off. It
/// flew the heading bug and the VS dial to the ground, and nothing ever took
/// it out: on the ground a heading servo cannot bank the aircraft, so its
/// error never closes and it winds the ailerons over (EGCC 23L, 2026-09-21:
/// the 737 taxied in with the yoke half way to the right stop, servos on,
/// the bug a quarter of a degree off the nose). Read first, read back, and
/// blackboxed `ap_disconnected`. `None` = it was already off, or no reading.
async fn disconnect_autopilot(xp: &dyn crate::xplane::XPlane) -> Option<bool> {
    // The whole autopilot, not only the servos: the arrest takes the servos
    // out at the roundout, so `servos_on` read 0 on the wheels while the
    // master (`autopilot_on`) held HDG and VS lit and the yaw damper stayed
    // on, and nothing came out (King Air LEMG 13, 1790780099-approach and
    // 1790780311-approach, iMac 2026-09-30: `ap` true down to 32 kt; one
    // swung 70 degrees right from 91 kt under full left steer, #197).
    let on = |r: Result<f64, _>| r.is_ok_and(|v: f64| v > 0.5);
    let (servos, master, damper) = tokio::join!(
        xp.get_float(SERVOS_ON),
        xp.get_float(AP_MASTER_ON),
        xp.get_float(YAW_DAMPER_ON),
    );
    let (servos, master, damper) = (on(servos), on(master), on(damper));
    if !servos && !master && !damper {
        return None;
    }
    if servos || master {
        let _ = xp.run_command(CMD_SERVOS_FDIR_OFF).await;
    }
    if damper {
        let _ = xp.run_command(CMD_YAW_DAMPER_OFF).await;
    }
    let (s2, m2, d2) = tokio::join!(
        xp.get_float(SERVOS_ON),
        xp.get_float(AP_MASTER_ON),
        xp.get_float(YAW_DAMPER_ON),
    );
    let off = |r: Result<f64, _>| r.is_ok_and(|v: f64| v < 0.5);
    let ok = off(s2) && (!master || off(m2)) && (!damper || off(d2));
    crate::blackbox::record(
        "autoland",
        serde_json::json!({"event": "ap_disconnected", "ok": ok,
            "servos": servos, "master": master, "yaw_damper": damper}),
    );
    Some(ok)
}

/// Rollout starts only after weight-on-wheels. The speed terms distinguish a
/// landing from a low pass once on the surface; they must never substitute for
/// the ground signal because a normal flare can satisfy both in the air.
///
/// ...but the wheels themselves are the best evidence there is. A 737 that
/// touches down a couple of knots over its approach speed (LEMG 12,
/// 2026-09-26: IAS 147 against 145, 146 kt over the ground) satisfied
/// neither speed term, so the loop went on flying the FLARE on the runway
/// for 2.8 s, then balked `overflew_the_runway` from the pavement and
/// firewalled the throttle. `ground_ticks` consecutive guidance ticks with
/// the wheels on and the radio under [`WHEELS_ON_RADIO_FT`] is a landing at
/// any speed; a bounce breaks the count.
fn should_start_rollout(
    already_rolling: bool,
    on_ground: bool,
    radio_ft: f64,
    gs_kts: f64,
    ias_kts: f64,
    approach_ias: f64,
    wheels_on_s: Option<f64>,
) -> bool {
    already_rolling
        || (on_ground
            && radio_ft < WHEELS_ON_RADIO_FT
            && wheels_on_s.is_some_and(|s| s >= WHEELS_ON_S - 1e-9))
        || (on_ground && radio_ft < 5.0 && (gs_kts < 45.0 || ias_kts < approach_ias))
}

/// Seconds of wheels on, from the first read that saw them, that make a
/// landing whatever the speed. It was three consecutive TICKS, which is
/// 0.4 s from first contact at the MacBook's 0.2 s flare tick (the recorded
/// 1790448340 contact below latched 0.4 s after it) and 0.8 s at the
/// iMac's 0.39 s (2026-09-30): a count of ticks is a time only on the
/// machine it was tuned on.
const WHEELS_ON_S: f64 = 0.4;
/// ...with the radio altimeter under this.
const WHEELS_ON_RADIO_FT: f64 = 2.0;

/// The clock [`should_start_rollout`] reads: seconds since the first read
/// with the wheels on under [`WHEELS_ON_RADIO_FT`] (`Some(0.0)` on that
/// read, plus the tick's `dt_s` on each after), or `None` (a bounce, a
/// failed read).
fn wheels_on_time(prev: Option<f64>, on_ground: bool, radio_ft: f64, dt_s: f64) -> Option<f64> {
    if on_ground && radio_ft.is_finite() && radio_ft < WHEELS_ON_RADIO_FT {
        Some(prev.map_or(0.0, |s| s + dt_s.max(0.0)))
    } else {
        None
    }
}

/// NO BALK FROM THE PAVEMENT. With the wheels on the runway a landing has
/// happened: a go-around from there at 145 kt with the throttle closed, the
/// spoilers out and the autobrake armed is the most dangerous thing the loop
/// could ask for. LEMG 12 (2026-09-26): 2.8 s on the wheels in the flare,
/// then `overflew_the_runway` at 0.33 ft radio, full power, "going around"
/// to the tower, and a landing clearance voided for an aircraft that then
/// stopped on the runway. On the ground any balk, and a flare turned into
/// anything else, stays a flare down the runway's course, which the rollout
/// latch takes over within ticks. Pure; the balk refused is returned.
pub fn wheels_hold_the_landing(
    g: Guidance,
    on_ground: bool,
    was_flaring: bool,
    course_deg: f64,
) -> (Guidance, Option<&'static str>) {
    if !on_ground || !(g.balk.is_some() || (was_flaring && g.phase != ApproachPhase::Flare)) {
        return (g, None);
    }
    let refused = g.balk.or(Some("phase_change"));
    (
        Guidance {
            target_heading_deg: course_deg,
            phase: ApproachPhase::Flare,
            balk: None,
            arresting: true,
            ..g
        },
        refused,
    )
}

/// A balked landing called at or above this radio height is a go-around the
/// moment it is called: from here no spool-up ends on the runway (a 737's
/// idle-to-go-around spool, 6 to 8 s at the path's 800 fpm, loses about 100
/// ft), so the tower hears it at once. Below it the call waits for a climb.
const GO_AROUND_CALL_AGL_FT: f64 = 200.0;
/// The climb that makes a low balk real: the radio this far above its
/// lowest since the balk, with the VSI (one tick old) positive.
const GO_AROUND_RISE_FT: f64 = 10.0;

/// A balk being flown, watched until it is a go-around or a landing: the
/// balk's own gate and height, the lowest radio since, and whether the
/// wheels have touched.
#[derive(Debug, Clone, Copy)]
struct BalkWatch {
    why: Option<&'static str>,
    balk_radio_ft: f64,
    low_ft: f64,
    touched: bool,
}

impl BalkWatch {
    fn new(why: Option<&'static str>, radio_ft: f64) -> Self {
        Self {
            why,
            balk_radio_ft: radio_ft,
            low_ft: radio_ft,
            touched: false,
        }
    }

    fn note(&mut self, radio_ft: f64, on_ground: bool) {
        if radio_ft.is_finite() {
            self.low_ft = self.low_ft.min(radio_ft);
        }
        self.touched |= on_ground;
    }
}

/// The "going around" radio call goes out once per balk that was actually
/// FLOWN as a go-around, and never with the wheels on the runway. A balk
/// called low may still land: LFBD 11 (2026-09-27) the 737 balked
/// `overflew_the_runway` at 7.9 ft radio, the call went out on that tick,
/// the tower answered "roger, go around" and dropped the landing clearance,
/// and the wheels touched 1.3 s later with the engines still spooling; the
/// crew had called a go-around and then landed uncleared. So a low balk is
/// called once it CLIMBS (`GO_AROUND_RISE_FT` above its lowest radio, the
/// VSI positive when it could be read), and a balk whose wheels touched
/// only once it is clear of [`GO_AROUND_CALL_AGL_FT`] (a touch that then
/// really left); one that rolls out never calls. A balk at or above that
/// height calls at once; the first LFBD approach's `off_centreline_low` at
/// 96 ft calls as soon as it climbs. Pure.
fn go_around_call_due(
    missed: bool,
    said: bool,
    on_ground: bool,
    watch: Option<&BalkWatch>,
    radio_ft: f64,
    vvi_fpm: Option<f64>,
) -> bool {
    if !missed || said || on_ground {
        return false;
    }
    let Some(w) = watch else { return true };
    if radio_ft >= GO_AROUND_CALL_AGL_FT {
        return true;
    }
    !w.touched && radio_ft - w.low_ft >= GO_AROUND_RISE_FT && vvi_fpm.is_none_or(|v| v > 0.0)
}

/// Radio height at which a final the tower is working but has NOT cleared
/// asks again: "final runway one five, request landing clearance" (a final
/// report, which the tower judges afresh against the strip).
pub const CLEARANCE_QUERY_FT: f64 = 1000.0;
/// Radio height at which a final still without its landing clearance goes
/// around: a crew never lands uncleared (EGKK-EGBB 15, sortie
/// 1790601822-full_auto, 2026-09-28: "expect late landing clearance" at
/// 7 nm, then nothing, and autoland landed; "you landed on runway one five
/// without clearance"). Above the flare by far: the order is the gate path's
/// (`GoAroundLatch::order`), flown from wherever we are.
pub const UNCLEARED_GO_AROUND_FT: f64 = 300.0;

/// What the crew owes a final the tower is working and has not cleared.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnclearedStep {
    /// Ask again, once per approach.
    Ask,
    /// Go around.
    GoAround,
}

/// The crew's watch on its own landing clearance. `working` is the tower's
/// book saying it has us in sequence and has not cleared us (`Inbound`: the
/// "continue approach, expect late landing clearance" answer, or a final
/// report not yet answered); without a tower working us (an uncontrolled
/// field, no two-way contact) there is nothing to be cleared by and the
/// watch is silent. Pure.
#[derive(Debug, Default)]
pub struct UnclearedWatch {
    asked: bool,
}

impl UnclearedWatch {
    pub fn step(
        &mut self,
        working: bool,
        radio_ft: f64,
        on_ground: bool,
        missed: bool,
    ) -> Option<UnclearedStep> {
        if missed {
            // The next approach asks again.
            self.asked = false;
            return None;
        }
        if !working || on_ground || !radio_ft.is_finite() {
            return None;
        }
        if radio_ft <= UNCLEARED_GO_AROUND_FT {
            return Some(UnclearedStep::GoAround);
        }
        if radio_ft <= CLEARANCE_QUERY_FT && !self.asked {
            self.asked = true;
            return Some(UnclearedStep::Ask);
        }
        None
    }
}

/// Cancel an active autoland; returns whether one was flying.
pub fn abort() -> bool {
    ABORT.store(true, Ordering::SeqCst);
    VACATE_FOLLOWS.store(false, Ordering::SeqCst);
    ACTIVE.swap(false, Ordering::SeqCst)
}

/// Fly a full emergency autoland to the nearest suitable runway. AP-first: HDG bug
/// tracks the guidance heading, VS mode flies the glidepath, throttle holds the
/// approach speed, gear/flaps come down on final, and it eases to idle in the
/// flare. EXPERIMENTAL — untuned; test high, over an airport, with a hand ready.
/// Own thread/runtime/client (like the recover watcher).
#[allow(clippy::too_many_arguments)]
pub fn run_autoland(
    base: String,
    apt_dat: String,
    cfg: ApConfig,
    approach_ias: f64,
    voice: Option<Voice>,
    status: Option<crate::plugin_status::StatusSink>,
    forced_rwy: Option<PlannedRunway>,
) {
    run_autoland_scoped(
        base,
        apt_dat,
        cfg,
        approach_ias,
        voice,
        status,
        forced_rwy,
        Vec::new(),
    );
}

#[allow(clippy::too_many_arguments)]
pub fn run_autoland_scoped(
    base: String,
    apt_dat: String,
    cfg: ApConfig,
    approach_ias: f64,
    voice: Option<Voice>,
    // The chat mirror now rides `voice.alert`; an explicit `s.action` here too
    // double-wrote every autoland line to the transcript (2026-07-24).
    _status: Option<crate::plugin_status::StatusSink>,
    // The arrival's already-chosen landing runway (by wind) AND its airport.
    // `Some` forces that end; `None` self-picks by track — the emergency and
    // manual path, unchanged.
    forced_rwy: Option<PlannedRunway>,
    parents: Vec<std::sync::Arc<AtomicBool>>,
) {
    let my_gen = GEN.fetch_add(1, Ordering::SeqCst) + 1;
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async move {
            // Kept for a re-arm on a different runway (see the clearance
            // watch in the tick).
            let base_url = base.clone();
            let xp = XPlaneClient::new(base).with_parents(parents);
            if xp.cancelled() { return; }
            ACTIVE.store(true, Ordering::SeqCst);
            LANDING_CONFIGURED.store(false, Ordering::SeqCst);
            let announce = |line: String, sev: Priority| {
                tracing::info!(target: "xvoicelayer::autoland", spoke = %line, "copilot");
                if let Some(v) = &voice { v.alert(sev, line); }
            };
            // The approach speed is a number somebody handed us; the airframe's
            // stall floor is a fact. A Cessna's 65 kt on an RV-10 whose clean
            // stall is 65 flew the speed schedule into a stall (2026-09-11).
            // The LANDING configuration's floor, not the clean one: the 737's
            // Vref 145 floored to the clean 167 overflew LFLL 35L twice
            // (2026-09-17; `aircraft::approach_floor_kts`).
            let approach_ias = {
                let floored = crate::aircraft::floored_target(approach_ias, crate::aircraft::approach_floor_kts());
                if floored > approach_ias {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "approach_ias_floored", "asked": approach_ias, "floor": floored}),
                    );
                    announce(
                        format!("Approach speed {approach_ias:.0} is below this airframe's {floored:.0} knot floor. Using {floored:.0}."),
                        Priority::Caution,
                    );
                }
                floored
            };

            // This approach's path angle: a steeper one handed down for a
            // visual segment (LSZS), else 3. Taken now so no approach
            // inherits another's.
            let gp_deg = arm_glidepath();
            // The stabilised gate this final is judged at (#199): a
            // circuit's is visual, every other the instrument gate.
            let gate = arm_gate();
            if (gp_deg - 3.0).abs() > 1e-9 {
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "glidepath", "deg": gp_deg}),
                );
            }
            // Find the runway (a full apt.dat pass, a few seconds — that's why the
            // tool says "searching"). Needs a valid position first.
            let (Ok(lat0), Ok(lon0), Ok(track0)) = (
                xp.get_float(LAT).await, xp.get_float(LON).await, xp.get_float(MAG_VAR).await,
            ) else { ACTIVE.store(false, Ordering::SeqCst); return };
            let _ = track0;
            let hdg0 = xp.get_float("sim/flightmodel/position/psi").await.unwrap_or(0.0);
            // A forced end (arrival chose by wind) is honoured when it's at the
            // field; a stale/unfound id falls back to the track-based search
            // rather than bailing on the landing. The self-pick is the
            // EMERGENCY path: at a field with parallels it picks whichever
            // threshold is nearer, which at LEMG put the ILS 12 arrival on 13
            // against a "cleared to land runway one two" (2026-09-12) — every
            // planned arrival must hand its runway down (see `EndAction`).
            let cleared = crate::party_line::clearance_runway();
            let mut from_clearance = false;
            let picked = match forced_rwy.as_ref() {
                Some(planned_rwy) => match runway_by_id(
                    &apt_dat,
                    lat0,
                    lon0,
                    &planned_rwy.id,
                    40.0,
                    planned_rwy.icao.as_deref(),
                ) {
                    Ok(None) => {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "forced_runway_missing",
                                "id": planned_rwy.id, "icao": planned_rwy.icao}),
                        );
                        self_pick(&apt_dat, lat0, lon0, hdg0)
                    }
                    Ok(Some(planned)) => {
                        // The tower's clearance outranks the plan, but only for
                        // an end at the SAME field — a stale departure clearance
                        // must never send us to a "09" forty miles away.
                        // An ARRIVAL's clearance only (`clearance_switch`):
                        // the ident on the party line is also a departure's
                        // runway, or merely the runway in use.
                        let switch_to = clearance_switch(
                            &planned.id,
                            crate::party_line::clearance_state(),
                            cleared.as_deref(),
                            None,
                            false,
                            None,
                        );
                        match switch_to.as_deref() {
                            Some(c) => match runway_by_id(
                                &apt_dat,
                                lat0,
                                lon0,
                                c,
                                40.0,
                                Some(planned.icao.as_str()),
                            ) {
                                Ok(Some(r)) if r.icao == planned.icao => {
                                    crate::blackbox::record(
                                        "autoland",
                                        serde_json::json!({"event": "runway_from_clearance",
                                            "planned": planned.id, "cleared": r.id}),
                                    );
                                    announce(
                                        format!(
                                            "Tower cleared us for runway {}, not the planned {}. Landing runway {}.",
                                            crate::circuit::spell_runway(&r.id),
                                            crate::circuit::spell_runway(&planned.id),
                                            crate::circuit::spell_runway(&r.id)
                                        ),
                                        Priority::Caution,
                                    );
                                    from_clearance = true;
                                    Ok(Some(r))
                                }
                                _ => Ok(Some(planned)),
                            },
                            None => Ok(Some(planned)),
                        }
                    }
                    err => err,
                },
                None => self_pick(&apt_dat, lat0, lon0, hdg0),
            };
            let mut rwy = match picked {
                Ok(Some(r)) => r,
                other => {
                    tracing::warn!(target: "xvoicelayer::autoland", ?other, from=?(lat0, lon0), "no runway found");
                    announce("No suitable runway within range. You have the aircraft.".into(), Priority::Warning);
                    ACTIVE.store(false, Ordering::SeqCst);
                    return;
                }
            };
            tracing::info!(
                target: "xvoicelayer::autoland",
                icao = %rwy.icao, id = %rwy.id, dist_nm = rwy.dist_nm,
                len_m = rwy.length_m, course = rwy.course_deg, elev = rwy.elev_ft,
                "runway selected"
            );
            // The runway flown IS the mission's arrival runway, from here to
            // the vacate: a tower switch at the arm moves it, and a re-arm
            // on the approach arrives here with it already moved (#64).
            crate::full_auto::retarget_arrival_runway(
                &rwy.id,
                armed_runway_why(forced_rwy.as_ref().map(|p| p.id.as_str()), &rwy.id, from_clearance),
            );
            announce(
                format!("Autoland to {} runway {}, {:.0} miles. Flying the approach.", rwy.icao, rwy.id, rwy.dist_nm),
                Priority::Warning,
            );
            // The runway's ILS on NAV1 for every landing that has one: the
            // raw data cross-checks our path, and flies it where the
            // airframe allows (`ils`).
            let ils = crate::ils::tune(&xp, &rwy.icao, &rwy.id, lat0, lon0).await;
            let coupled_ok = ils.is_some() && crate::ils::coupled_allowed();

            // The pavement's OWN elevation, not the field's. apt.dat carries one
            // elevation per airport and the glidepath aimed at it: LEAX's
            // header says 160 ft, its runway 30 threshold lies at ~117, and the
            // 172 crossed the numbers 99 ft up on a path that was "on" in MSL
            // terms, never reached the roundout band, and balked "overflew the
            // runway" 300 m in (2026-09-15). Two probes of the sim's own mesh,
            // the threshold and the touchdown aim; the higher wins (the aim
            // must clear both), the header stays when they agree or the probe
            // is absent or absurd. Said aloud when it moves the aim.
            {
                let (aim_lat, aim_lon) =
                    project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, TOUCHDOWN_AIM_NM);
                let probed = crate::terrain::sample_ft(
                    &xp,
                    &[(rwy.thr_lat, rwy.thr_lon), (aim_lat, aim_lon)],
                )
                .await;
                let (pavement_aim, seen) = aim_elevation_ft(rwy.elev_ft, probed.as_deref());
                // With the ILS tuned, the path lies on its glideslope as far
                // as the threshold crossing allows (`aim_with_glideslope`).
                let gs_aim = if ils.is_some() {
                    let (nav, icao, id) = (
                        crate::settings::live().nav_data_path.clone(),
                        rwy.icao.clone(),
                        rwy.id.clone(),
                    );
                    tokio::task::spawn_blocking(move || crate::ils::find_glideslope(&nav, &icao, &id))
                        .await
                        .ok()
                        .flatten()
                        .and_then(|gs| {
                            crate::ils::glideslope_aim_ft(
                                &gs, rwy.thr_lat, rwy.thr_lon, rwy.course_deg, TOUCHDOWN_AIM_NM, gp_deg,
                            )
                        })
                } else {
                    None
                };
                let aim = aim_with_glideslope(pavement_aim, gs_aim);
                crate::blackbox::record("autoland", serde_json::json!({
                    "event": "threshold_elev", "rwy": format!("{} {}", rwy.icao, rwy.id),
                    "header_ft": rwy.elev_ft, "probed_ft": seen, "aim_ft": aim,
                    "pavement_aim_ft": pavement_aim, "gs_aim_ft": gs_aim,
                    // The pavement the lateral gates judge against.
                    "width_m": runway_width_m(&rwy),
                    "xtk_limit_m": landing_xtk_limit_m(runway_width_m(&rwy)),
                    "flare_xtk_limit_m": flare_xtk_limit_m(runway_width_m(&rwy)),
                }));
                if (aim - rwy.elev_ft).abs() > THRESHOLD_ELEV_TRUST_FT {
                    let diff = aim - rwy.elev_ft;
                    let line = if gs_aim.is_some() {
                        format!(
                            "Runway {} on the glideslope, aiming at {:.0} feet.",
                            crate::circuit::spell_runway(&rwy.id),
                            aim
                        )
                    } else {
                        format!(
                            "Runway {} threshold is {:.0} feet {} the field elevation. Aiming at {:.0} feet.",
                            crate::circuit::spell_runway(&rwy.id),
                            diff.abs(),
                            if diff < 0.0 { "below" } else { "above" },
                            aim
                        )
                    };
                    announce(line, Priority::Advisory);
                    rwy.elev_ft = aim;
                } else if (aim - pavement_aim).abs() > 0.5 {
                    // The glideslope put the aim back near the header: the
                    // header's number, said nothing (a correct aim earns no
                    // chatter).
                    rwy.elev_ft = aim;
                }
            }

            // Engage the AP (FD up first for airframes that need it), and
            // CONFIRM it: the tick re-presses until the servos read on
            // (`ApEngage`), holding the aircraft level until then.
            let engage_t0 = std::time::Instant::now();
            let mut engage = ApEngage::default();
            let mut engage_pitch = FlarePitch::with_gains(engage_hold_gains());
            let mut engage_held = false;
            {
                let e = engage.arm(0.0);
                for c in &cfg.fd_commands { let _ = xp.run_command(c).await; }
                let ok = xp.run_command(CMD_SERVOS_ON).await.is_ok();
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "ap_engage", "try": e.try_n, "sent": ok,
                        "at_s": 0.0, "servos": null, "ias": null}),
                );
            }
            // The altitude dial OUT OF THE WAY. Autoland flies the vertical
            // through the VS dial and never touched the altitude dial, so it
            // sat wherever the follower or the pilot left it: at LEAX the
            // plan's 2,600 ft constraint, 130 ft under the arm, and the AP's
            // altitude capture was already arresting the descent as the arm
            // came. The VS dial's -1,200 was ignored for six seconds, flaps
            // ballooned it to +300 fpm, and only the divergence watchdog's
            // re-press of VS mode freed it ("not holding the descent",
            // 2026-09-15). Parked well above and VS re-selected so no capture
            // can start; a go-around levels there at the latest, which the
            // per-tick VS re-select then takes down again.
            {
                let (dial_r, alt_r) = tokio::join!(xp.get_float(ALT_DIAL), xp.get_float(ALT_IND_FT));
                if let (Ok(dial), Ok(alt_ind)) = (dial_r, alt_r) {
                    if let Some(parked) = parked_altitude_dial_ft(dial, alt_ind) {
                        let _ = xp.set_float(ALT_DIAL, parked).await;
                        let _ = xp.run_command(CMD_VS_MODE).await;
                        crate::blackbox::record("autoland", serde_json::json!({
                            "event": "alt_dial_parked", "was": dial, "now": parked, "alt_ft": alt_ind,
                        }));
                    }
                }
            }

            *TRACKING_ERROR.lock().unwrap_or_else(|e| e.into_inner()) = None;
            let mut last_phase: Option<ApproachPhase> = None;
            let mut flare_power = FlarePower::default();
            // The heading knob, set as a pilot sets it (`bug_knob`, issue #93).
            let mut bug_hold = crate::circuit::BugHold::default();
            let knob_t0 = std::time::Instant::now();
            // The lever the approach was flown on, for a balloon to hold.
            let mut approach_thr: Option<f64> = None;
            let mut go_around = GoAroundLatch::new();
            // The stabilised-approach rule (`stable_approach`, #199): the
            // breaches held so far, the clock they are timed on, the balk
            // word and spoken reason of a go-around it ordered, and whether
            // this approach's gate crossing has been recorded.
            let mut stable_watch = crate::stable_approach::StableWatch::default();
            let stable_clock = std::time::Instant::now();
            let mut stable_balk: Option<&'static str> = None;
            let mut stable_said: Option<&'static str> = None;
            let mut gate_recorded = false;
            // Watch the tower's go-around orders from here: an order shouted
            // at somebody else's approach half an hour ago must not balk this
            // one on its first tick. A WATCH, never the main loop's `take_*`
            // flag: sharing that flag, the main loop's FSM sync consumed every
            // order before this loop's tick saw it (LEMG 31, 2026-09-28).
            // The pilot's order rides the same watch (`go_around`, #77).
            let mut orders = crate::go_around::GoAroundWatch::arm();
            // The crew's own watch on the landing clearance (#155).
            let mut uncleared = UnclearedWatch::default();
            // The final report owed on a SECOND approach. The circuit or the
            // arrival makes the first one, which is what earns the landing
            // clearance; a missed approach then flies away, and until this
            // existed nothing called again — eleven minutes of missed
            // approach at Malaga with the tower waiting on "report downwind",
            // then a landing without a clearance (2026-08-05).
            let mut owe_final_report = false;
            // The "going around" transmission: said once per balked landing,
            // with the callsign read from the aircraft the first time it's
            // needed (an HTTP read, so not on every tick).
            let mut said_going_around = false;
            let mut going_around_callsign: Option<Option<String>> = None;
            // The balk being flown until it climbs (then the call) or the
            // wheels take it (then no call: see [`go_around_call_due`]), and
            // whether the intercom said "going around" for it.
            let mut balk_watch: Option<BalkWatch> = None;
            let mut spoke_going_around = false;
            // The current missed approach's (or pattern's) progress, judged
            // by the ground it makes (`reversal::MissedWatch`), on a clock of
            // this loop's own; and the speed the pattern began at, which
            // sizes its track to go.
            let mut missed_watch: Option<crate::reversal::MissedWatch> = None;
            let watch_epoch = std::time::Instant::now();
            let mut pattern_gs_kts = 0.0_f64;
            let mut gear_down = false;
            let mut landing_config_done = false;
            // The flap the wing is flying on at the arm: the schedule's first
            // write must know what it would take away (`flap_kept_flying`).
            // A failed read is no measurement: unknown, as before.
            let mut last_flap: Option<f64> = xp
                .get_float(FLAP_RATIO)
                .await
                .ok()
                .filter(|f| f.is_finite() && (0.0..=1.0).contains(f));
            let mut vs_diverged_s: f64 = 0.0;
            let mut prev_vs_err: Option<f64> = None;
            // Warned once per approach when the VSI cannot be read.
            let mut vvi_read_failed = false;
            // The vertical speed one tick ago, for the arrest's trend term.
            let mut prev_vvi: Option<(f64, std::time::Instant)> = None;
            // The flare's elevator law. Its integrator is per-approach: a
            // standing pull must not survive a go-around into the next one.
            let mut flare_pitch =
                FlarePitch::with_gains(FlareGains::from_profile(crate::aircraft::flare_elevator()));
            // ...or, where the airframe declares its elevator's response,
            // the arrest flown on that model (`FlareModel`, the 737).
            let mut flare_model = FlareModel::from_profile(crate::aircraft::flare_elevator());
            // The lever the loop last commanded, for the tick record (the
            // flare's table had the elevator and the sink but not the power).
            let mut last_lever: Option<f64> = None;
            // Last tick was the flare: a balloon stays in it (`hold_flare`).
            let mut flaring = false;
            // This flare was committed (`commit_flare_latched`).
            let mut flare_committed = false;
            let mut pitch_yielded = false;
            // The arrest has the servos (see `arrest_hands_step`); the
            // wings-level law that holds the track while it does; the
            // pilot has taken the roll for this approach.
            let mut arrest_hands = false;
            // This tick's arrest is only `hold_roundout`'s (a balloon over
            // the band), for the blackbox's `roundout_held` edges.
            let mut roundout_held = false;
            let mut wings = arrest_wings();
            let mut short_final: Option<ShortFinal> = None;
            let mut arrest_rate = crate::autotakeoff::XtkRate::default();
            let mut arrest_prev_xtk: Option<f64> = None;
            let mut roll_yielded = false;
            let mut last_arresting = false;
            // The final's air (`FinalAir`), for how the flare holds the column.
            let mut final_air = FinalAir::default();
            // Airspeed history for the throttle governor's derivative term — the
            // anti-stall reflex needs the TREND (kts/sec), not just the error.
            let mut prev_ias: Option<f64> = None;
            let mut prev_ias_t = std::time::Instant::now();
            // Cross-track history for the intercept damping term.
            let mut prev_xtk: Option<(f64, std::time::Instant)> = None;
            let mut rollout_rate = crate::autotakeoff::XtkRate::default();
            let mut xtk_rate: f64 = 0.0;
            // Wind crab, low-passed — see [`CrabFilter`] for the hunt it ends.
            let mut crab_filter = CrabFilter::new();
            // Integral trim on the final's track error — see [`TrackTrim`]
            // for the standing 35 m the crab loop could not see.
            let mut track_trim = TrackTrim::new();
            // Rollout: latched at touchdown, then brakes down — to a full stop,
            // or to taxi pace when a vacate drive follows. The takeoff roll law
            // (P + trim, tested) holds the centreline through the plugin steer
            // channel while it brakes.
            let mut rolling_out = false;
            let mut wheels_on: Option<f64> = None;
            // When the last live tick read the wheels (the wheels-on clock).
            let mut wheels_read_t: Option<std::time::Instant> = None;
            let mut vacate_follows = false;
            let mut roll_law = crate::autotakeoff::RollSteer::default();
            // A castoring nosewheel steers the rollout on its brakes too
            // (`RolloutBrakeSteer`, issue #166).
            let rollout_brake_steer = rollout_brake_steers();
            let mut rollout_brakes = RolloutBrakeSteer::for_airframe();
            // A steered nosewheel that declares its own rollout law
            // (`RolloutSteer`, issue #197); else the takeoff roll's default.
            let mut rollout_steer = crate::aircraft::rollout_steer().map(RolloutSteer::new);
            // The wheels off again after the rollout latched (a bounce).
            let mut bounce_hold = BounceHold::default();
            // A hold-off airframe held on from its first contact
            // (`TouchHold`), and the elevator the flare last wrote.
            let holds_off = flare_model.as_ref().is_some_and(|m| m.holds_off());
            let mut touch_hold = TouchHold::default();
            let mut last_flare_elev: Option<f64> = None;
            // The landing readback last sent, so an owed one goes out once
            // (the party line keeps it owed until the tower hears it).
            let mut readback_sent: Option<String> = None;
            // The cleared ident already looked up against apt.dat, so the
            // (seconds-long) lookup of one that is not at this field runs once.
            let mut clearance_checked: Option<String> = None;
            // The course reversal in progress, if any (see [`crate::reversal`]):
            // how this loop gets round to a final it is on the wrong side of.
            let mut reversal: Option<crate::reversal::Reversal> = None;
            let mut reversals_flown: u32 = 0;
            // A published missed approach asked for once: one that cannot be
            // flown falls through to our own pattern, never to "Your aircraft"
            // with nobody there (LEMG 13 SR22 circuit 2026-09-29).
            let mut missed_procedure_tried = false;
            // The pattern's plan over the probed ground (#182), probed beside
            // the loop (the plugin answers in seconds; the aircraft flies on),
            // and when it began. Begun in the go-around's climb, so it is
            // ready when the reposition wants it.
            let mut reversal_plan: Option<(
                tokio::task::JoinHandle<crate::reversal::PatternPlan>,
                std::time::Instant,
            )> = None;
            // Straight ahead at a held altitude: while that plan is probed,
            // or because no pattern here clears the ground (#182).
            let mut hold: Option<TerrainHold> = None;
            // The live floor over the ground under and ahead on every leg
            // but the final (#182, the circuit's `live_floor_ft`): the floor
            // and when it tripped, when it was last probed, and whether the
            // spell has been said.
            let mut live_floor: Option<(f64, std::time::Instant)> = None;
            let mut live_floor_probed: Option<std::time::Instant> = None;
            let mut live_floor_said = false;
            // How far out this approach has legitimately been: the arm
            // distance, pushed out by any pattern flown since.
            let mut recede_ref_nm = rwy.dist_nm;
            // A give-up happened: the speed is handed to the flight governor
            // once the loop has ended (see below).
            let mut gave_up = false;
            // The confirmed engage's pitch authority is checked from the
            // next tick with a climb read (#181).
            let mut authority_owed = false;
            let mut ils_xcheck = crate::ils::XCheck::default();
            // The tick's writes, sent in one round (`sim::state::Writes`),
            // and the lever last written, for `lever_took`.
            let mut writes = sim::state::Writes::new();
            let mut last_thr_written: Option<f64> = None;
            let mut coupling = crate::ils::Coupling::Off;
            let mut coupling_armed_s = 0.0_f64;
            loop {
                if xp.cancelled() || GEN.load(Ordering::SeqCst) != my_gen || ABORT.load(Ordering::SeqCst) {
                    break;
                }
                // The tick runs to its period (`tick_ms`), not sleep + work.
                let pacer = sim::state::Pacer::start();
                // Frozen sim (replay or pause): a replay's positions are a
                // recording and not ours to fly; a pause is real but frozen,
                // and ticking the throttle governor against a frozen airspeed
                // walks the lever to its clamp for a surge on unpause. Freeze
                // (no writes, no state, no blackbox) and resume when it's live.
                //
                // ONE read a tick (`sim::state`): the freeze flags, position,
                // height, groundspeed and heading as ONE frame saw them (#157:
                // read separately, latitude and longitude paired two frames,
                // metres of cross-track for a tick at speed), and with them
                // everything the tick used to ask for later: the VSI, the
                // lever (the profile's own beside the frame), the servos, the
                // mode lights, the bug, the ILS needles and the yields. Every
                // round of requests is a sim frame: the freeze check, this
                // read, the VSI and lever, the ILS, the knob and yields, the
                // lever write and its read back made Final 0.56 s (`power` dt
                // median) against a 200 ms sleep and the Flare 0.39 s against
                // 100 ms on the iMac at 20 fps (2026-09-30), where a MacBook
                // ran near 0.3. An older plugin falls back to separate reads,
                // still one round.
                let thr_name = throttle_dref();
                let appr_name = crate::ils::approach_status_dref();
                let (st, extra) = if ils.is_some() {
                    sim::state::read_with(
                        &xp,
                        sim::state::Want(AUTOLAND_WANT_ILS),
                        &[thr_name.as_str(), appr_name.as_str()],
                    )
                    .await
                } else {
                    sim::state::read_with(&xp, sim::state::Want(AUTOLAND_WANT), &[thr_name.as_str()])
                        .await
                };
                if st.frozen() {
                    tokio::time::sleep(std::time::Duration::from_millis(400)).await;
                    continue;
                }
                let mut extra = extra.into_iter();
                let mut throttle_read = Some(extra.next().unwrap_or_else(|| Err(anyhow::anyhow!("unread"))));
                let mut appr_read = extra.next();
                let pos = st.position();
                let (var_r, ias_r, hpath_r, ground_r, phi_r) = (
                    st.get(F::MagVar),
                    st.get(F::Ias),
                    st.get(F::Hpath),
                    st.get(F::OnGround),
                    st.get(F::Phi),
                );
                let (lat_r, lon_r) = sim::position::lat_lon(&pos);
                let elev_r = sim::position::field(&pos, |p| p.elev_m);
                let agl_r = sim::position::field(&pos, |p| p.agl_m);
                let gs_r = sim::position::field(&pos, |p| p.gs_ms);
                let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
                let pos_source = sim::position::source_word(&pos);
                let (Ok(lat), Ok(lon), Ok(elev_m), Ok(agl_m), Ok(gs_ms), Ok(var), Ok(ias), Ok(on_ground)) =
                    (lat_r, lon_r, elev_r, agl_r, gs_r, var_r, ias_r, ground_r)
                else { tokio::time::sleep(std::time::Duration::from_millis(300)).await; continue };
                let alt = elev_m * M_TO_FT;
                let radio_ft = agl_m * M_TO_FT;
                let gs_kts = gs_ms * MS_TO_KTS;

                // THE TOWER'S WORD DURING THE APPROACH. The clearance was read
                // once, at arm time; the final report that EARNS it goes out
                // after that, so the answer was never seen: "negative, runway
                // zero niner, I say again runway zero niner ... cleared to
                // land" came back five seconds after the pick, and autoland
                // flew 27 into a 21 kt tailwind (LXGB, 2026-09-14). Now the
                // landing clearance is read back (the runway-critical echo
                // the tower waits for, the crew's job while a loop flies),
                // and a clearance for ANOTHER end of THIS field re-arms the
                // approach on it: a fresh loop, forced onto the cleared
                // runway, repositioning from wherever we are. A departure
                // state's runway is the departure's; a stale one is never
                // ours to land on.
                if on_ground < 0.5 && !rolling_out {
                    let cleared = crate::party_line::clearance_runway();
                    match crate::party_line::readback_owed() {
                        None => readback_sent = None,
                        Some(owed) if landing_readback(&owed) && readback_sent.as_deref() != Some(&owed) => {
                            readback_sent = Some(owed.clone());
                            if going_around_callsign.is_none() {
                                going_around_callsign = Some(crate::circuit::crew_callsign(&xp).await);
                            }
                            let cs = going_around_callsign.clone().flatten();
                            let rwy_words = cleared
                                .as_deref()
                                .map(|r| format!(" runway {}", crate::circuit::spell_runway(r)))
                                .unwrap_or_default();
                            crate::circuit::transmit(crate::circuit::radio_call(
                                &cs,
                                &format!("cleared to land{rwy_words}"),
                            ));
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "landing_readback", "rwy": cleared}),
                            );
                        }
                        Some(_) => {}
                    }
                    // A re-sequence the go-around order named is taken once
                    // the climb away is flown (#182).
                    let expected = if go_around.climbing() {
                        None
                    } else {
                        orders.expected_runway()
                    };
                    if let Some(c) = clearance_switch(
                        &rwy.id,
                        crate::party_line::clearance_state(),
                        cleared.as_deref(),
                        clearance_checked.as_deref(),
                        go_around.active(),
                        expected.as_deref(),
                    ) {
                        clearance_checked = Some(c.clone());
                        let resequenced = go_around.active();
                        match runway_by_id(&apt_dat, lat, lon, &c, 40.0, Some(rwy.icao.as_str())) {
                            Ok(Some(r)) if r.icao == rwy.icao => {
                                crate::blackbox::record(
                                    "autoland",
                                    serde_json::json!({"event": "runway_from_clearance",
                                        "planned": rwy.id, "cleared": r.id, "in_flight": true,
                                        "resequenced": resequenced,
                                        "radio": radio_ft, "dist": r.dist_nm}),
                                );
                                announce(
                                    if resequenced {
                                        format!(
                                            "Tower expects us on runway {}. Setting up for runway {}.",
                                            crate::circuit::spell_runway(&r.id),
                                            crate::circuit::spell_runway(&r.id)
                                        )
                                    } else {
                                        format!(
                                            "Tower cleared us for runway {}, not {}. Setting up for runway {}.",
                                            crate::circuit::spell_runway(&r.id),
                                            crate::circuit::spell_runway(&rwy.id),
                                            crate::circuit::spell_runway(&r.id)
                                        )
                                    },
                                    Priority::Caution,
                                );
                                // The mission's ONE arrival runway moves
                                // with us, before the new loop arms: its
                                // ILS tune, the crew's calls, the vacate
                                // and the card read the cleared end (#64).
                                crate::full_auto::retarget_arrival_runway(
                                    &r.id,
                                    if resequenced { "resequenced" } else { "clearance" },
                                );
                                // A new loop owns the aircraft from its first
                                // tick; this one exits on the generation bump
                                // and leaves ACTIVE to it.
                                run_autoland_scoped(
                                    base_url.clone(),
                                    apt_dat.clone(),
                                    cfg.clone(),
                                    approach_ias,
                                    voice.clone(),
                                    None,
                                    Some(PlannedRunway::at(&r.icao, &r.id)),
                                    xp.cancellation(),
                                );
                                break;
                            }
                            other => {
                                crate::blackbox::record(
                                    "autoland",
                                    serde_json::json!({"event": "clearance_runway_elsewhere",
                                        "flying": format!("{} {}", rwy.icao, rwy.id), "cleared": c,
                                        "found": other.ok().flatten().map(|r| format!("{} {}", r.icao, r.id))}),
                                );
                            }
                        }
                    }
                }

                // ROLLOUT TO A STOP. Once on the ground and slow, cut the power
                // and brake to a halt before handing off — the old touchdown
                // path just idled the throttle at 40 kt and said "you have the
                // aircraft", leaving it coasting down the runway with nobody on
                // the brakes (the circuit's landings rolled on unmanaged,
                // 2026-07-23). Latched, so the approach guidance below never
                // re-fires once we're down. The nose is held to the centreline
                // through the plugin steer channel with the takeoff roll law.
                //
                // Commit on GROUNDSPEED *or* airspeed-below-approach: the bare
                // gs < 45 gate is a 172 number, and a heavier airframe landing
                // into a TAILWIND touches down well above it — a Twin Otter
                // planted at 70 kt gs (56 KIAS, 6 kt tailwind on 27) never
                // latched, so nothing steered the centreline OR braked; the
                // approach guidance floated past the aimpoint, flipped
                // Flare->Intercept, and the go-around latch firewalled it off
                // the end into the bay (LXGB, 2026-07-26). Airspeed below the
                // approach speed can identify a committed landing only after
                // weight-on-wheels. Without that gate a normal flare below five
                // feet could latch rollout in the air, cut power, and arm the
                // brakes before touchdown.
                let wheels_t = std::time::Instant::now();
                let wheels_dt = wheels_read_t
                    .map_or(0.0, |t0| wheels_t.duration_since(t0).as_secs_f64());
                wheels_read_t = Some(wheels_t);
                wheels_on = wheels_on_time(wheels_on, on_ground > 0.5, radio_ft, wheels_dt);
                if should_start_rollout(
                    rolling_out,
                    on_ground > 0.5,
                    radio_ft,
                    gs_kts,
                    ias,
                    approach_ias,
                    wheels_on,
                ) {
                    // Consumed once, at rollout entry: a stale flag from a
                    // mission that never landed must not turn a later
                    // standalone landing into a rolling handoff to nobody.
                    if !rolling_out {
                        rolling_out = true;
                        // A hold-off airframe holds the pull it touched with
                        // (`TouchHold`); the flare's own contact, if it saw
                        // one first, already took it.
                        // Only a column the flare was flying: never a take
                        // of the pitch the arrest never had.
                        if holds_off
                            && !pitch_yielded
                            && last_flare_elev.is_some()
                            && touch_hold.note(true, last_flare_elev)
                        {
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "touch_hold", "held": touch_hold.held(),
                                    "at": "rollout", "radio": radio_ft, "gs": gs_kts, "ias": ias,
                                    "on_ground": on_ground > 0.5}),
                            );
                        }
                        // The arrest's last aileron rides the roll channel's
                        // 1 s heartbeat onto the wheels unless it is
                        // centred: full right at the handover banked a 737
                        // 9.9 degrees on the mains and slid it 6.1 m right
                        // (EGKK 08R 1790687142, 2026-09-29).
                        if arrest_hands {
                            let _ = xp.set_float(FLARE_ROLL, 0.0).await;
                        }
                        disconnect_autopilot(&xp).await;
                        vacate_follows = VACATE_FOLLOWS.swap(false, Ordering::SeqCst);
                        // Wheels on this runway: the fact the vacate needs.
                        *LANDED_RUNWAY.lock().unwrap_or_else(|e| e.into_inner()) =
                            Some(rwy.id.clone());
                        // A balk the wheels took is a LANDING (LFBD 11,
                        // 2026-09-27): the call waited for a climb that
                        // never came, so the tower still holds our landing
                        // clearance; the intercom's "going around" is
                        // corrected.
                        if go_around.active() {
                            let along_m = along_track_nm(lat, lon, &rwy) * M_PER_NM;
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "balk_landed",
                                    "why": balk_watch.and_then(|w| w.why),
                                    "balk_radio": balk_watch.map(|w| w.balk_radio_ft),
                                    "radio": radio_ft, "gs": gs_kts,
                                    "remaining_m": (rwy.length_m + along_m).max(0.0),
                                    "called": said_going_around}),
                            );
                            if spoke_going_around {
                                announce(
                                    "Correction, we're down. Landing, not going around.".into(),
                                    Priority::Caution,
                                );
                            }
                        }
                    }
                    let xtk_nm = cross_track_nm(lat, lon, &rwy);
                    let xtk_m = xtk_nm * M_PER_NM;
                    // The roll's own closure rate and its own tick time. The
                    // approach path below never runs again once we are rolling
                    // out, so `prev_xtk` is ours from here; its last value is
                    // the final's, which is the continuity we want at the
                    // wheels. The steering law used to be handed a HARDCODED
                    // 0.4 s while this loop actually ticks at 300 ms plus two
                    // round trips, so its lead term and its slew limit were
                    // both computed against a clock that did not exist.
                    let t_roll = std::time::Instant::now();
                    let roll_dt = prev_xtk
                        .map(|(_, t0)| t_roll.duration_since(t0).as_secs_f64())
                        .filter(|d| (0.05..=3.0).contains(d))
                        .unwrap_or(0.4);
                    let xtk_rate = rollout_rate.update(
                        prev_xtk
                            .filter(|_| roll_dt > 0.0)
                            .map_or(0.0, |(x0, _)| (xtk_m - x0 * M_PER_NM) / roll_dt),
                        roll_dt,
                    );
                    prev_xtk = Some((xtk_nm, t_roll));
                    let target = crate::autotakeoff::roll_target_heading(
                        rwy.course_deg,
                        xtk_m,
                        xtk_rate,
                        gs_kts,
                    );
                    let psi = psi_r.as_ref().copied().unwrap_or(rwy.course_deg);
                    let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
                    // OFF THE WHEELS after the latch: a bounce (`BounceHold`).
                    // The latch takes the first tick the wheels read on, and
                    // at LEMG 13 (the SR22, 2026-09-29) that tick was a
                    // bounce both times: 1790689843 flew 3.5 to 3.8 ft for
                    // six seconds after it, 1790694457 up to 9 ft for four
                    // and a half, the brakes split and the rudder steering an
                    // aircraft in the air with the roll channel let go.
                    // 1790694457 drifted from 4.1 m to 10.3 m right at 1.1 to
                    // 1.8 m/s and landed with 0.39 of left brake already on,
                    // which slewed the nose from 134 to 127 in a second on a
                    // 131.5 track (the scorer's 3.8 degrees of drift at the
                    // touchdown). In the air: no wheel brake, no rudder, the
                    // arrest's wings closing the track on the centreline.
                    let airborne = on_ground < 0.5;
                    let bounce = bounce_hold.tick(airborne);
                    if matches!(bounce, BounceTick::Left | BounceTick::Landed) {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "bounce",
                                "off": bounce == BounceTick::Left,
                                "spell": bounce_hold.spells(), "radio": radio_ft,
                                "gs": gs_kts, "xtk_m": xtk_m, "psi": psi}),
                        );
                    }
                    if bounce == BounceTick::Landed {
                        // Down again: the split starts from the wheels, never
                        // from what it learnt steering the air.
                        rollout_brakes = RolloutBrakeSteer::for_airframe();
                    }
                    // A declared rollout steer damps on the sim's yaw rate
                    // (`RolloutSteer`, #197): carried by the tick's one read
                    // (`sim::state`, `F::R`), no round of its own.
                    let yaw_r = match rollout_steer.as_ref() {
                        Some(_) if !airborne => st.opt(F::R),
                        _ => None,
                    };
                    let steer = if airborne {
                        if let Some(law) = rollout_steer.as_mut() {
                            law.in_air();
                        }
                        0.0
                    } else if let Some(law) = rollout_steer.as_mut() {
                        law.step(err, roll_dt, gs_kts, yaw_r)
                    } else {
                        roll_law.step(err, roll_dt, gs_kts, 0.0)
                    };
                    let track_roll = hpath_r.as_ref().ok().copied();
                    let side_roll = side_velocity_m_s(track_roll, rwy.course_deg, gs_kts)
                        .unwrap_or(xtk_rate);
                    let phi_roll = phi_r.as_ref().ok().copied();
                    let air_bank_tgt = phi_roll.map(|phi| {
                        arrest_bank_target(xtk_m, side_roll, wings.predicted(phi), radio_ft)
                    });
                    let air_ail = match (airborne && arrest_hands && !roll_yielded, phi_roll, air_bank_tgt) {
                        (true, Some(phi), Some(tgt)) => Some(wings.step(phi, tgt, roll_dt)),
                        _ => None,
                    };
                    // Queued: the rollout's writes go out in one round below.
                    if let Some(a) = air_ail {
                        // Ours, not a hand: unmarked, the scorer read this
                        // bounce's bank as a pilot's yoke (LEMG 13 SR22
                        // 1790697159, yoke_under_ap yr -0.18, 2026-09-29).
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_ROLL, a);
                    } else if bounce == BounceTick::Landed && arrest_hands {
                        // The rollout entry's centring, again at the wheels.
                        writes.set(FLARE_ROLL, 0.0);
                    }
                    // Brake by wanted deceleration, not a fixed clamp: a
                    // comfortable rate when the runway allows it, harder
                    // only when stopping in what remains demands it; on a
                    // castoring nosewheel, split between the wheels to steer
                    // (`RolloutBrakeSteer`; an unread heading steers nothing).
                    let along_m = along_track_nm(lat, lon, &rwy) * M_PER_NM;
                    let remaining_m = (rwy.length_m + along_m).max(0.0);
                    let target_kts = if vacate_follows { VACATE_HANDOFF_KTS } else { 0.0 };
                    let brake = rollout_brake_ratio(gs_kts, target_kts, remaining_m);
                    let brakes = if airborne {
                        RolloutBrakes {
                            sym: 0.0,
                            left: 0.0,
                            right: 0.0,
                        }
                    } else if rollout_brake_steer {
                        rollout_brakes.step_with(
                            if psi_r.is_ok() { err } else { f64::NAN },
                            psi_r.as_ref().ok().copied(),
                            track_roll,
                            steer,
                            roll_dt,
                            gs_kts,
                            brake,
                        )
                    } else {
                        RolloutBrakes {
                            sym: brake,
                            left: 0.0,
                            right: 0.0,
                        }
                    };
                    // A hold-off airframe keeps the pull it touched with
                    // (`TouchHold`), in the air too: the channel is a 1 s
                    // heartbeat, so it is written every tick it is held.
                    let touch_elev = if pitch_yielded {
                        None
                    } else {
                        touch_hold.elev(gs_kts, !airborne, roll_dt)
                    };
                    // The rollout kept no record of its own: the EGLF and LFPB
                    // weaves had to be reconstructed from the 1 Hz `state`
                    // samples, which is no way to read a control loop.
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({
                            "event": "rollout", "gs": gs_kts, "xtk_m": xtk_m,
                            // The held pull (#166), when one is.
                            "touch_elev": touch_elev,
                            "xtk_rate": xtk_rate, "err": err, "steer": steer,
                            "trim": roll_law.trim(), "psi": psi, "dt": roll_dt,
                            // The sim's yaw rate a declared rollout steer
                            // damps on (#197); null when not read.
                            "yaw_r": yaw_r,
                            "pos": pos_source,
                            // The brakes as written: the symmetric channel and
                            // each wheel's steering channel (issue #166).
                            "brake": brakes.sym, "bl": brakes.left, "br": brakes.right,
                            // The free nose's learnt standing yaw, deg/s^2
                            // (`ground_steer`; 0 on the shared split), and its
                            // tick: the yaw rate asked and read.
                            "btrim": rollout_brakes.trim(),
                            "gsteer": rollout_brakes.last().map(|s| s.json()),
                            // The wheels and the height: a latch on a bounce
                            // flew six seconds of "rollout" in the air, and
                            // only the 3 s `state` samples said so (#166).
                            "ground": !airborne, "radio": radio_ft,
                            "phi": phi_roll, "trk": track_roll,
                            "air_ail": air_ail,
                            "bank_tgt": airborne.then_some(air_bank_tgt).flatten(),
                        }),
                    );
                    writes.set(&thr_name, 0.0).set(TAXI_STEER, steer);
                    if let Some(e) = touch_elev {
                        // Ours, not a hand (the scorer's yoke check).
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_PITCH, e);
                    }
                    // A vacate follows: hand off ROLLING at taxi pace — the
                    // completion of a landing is clearing the runway, and a
                    // dead stop on the strip followed by a re-acceleration was
                    // neither procedure nor comfortable (EGHI, 2026-08-02).
                    // The vacate drive arms the governor from the current
                    // position, which takes a rolling aircraft like any join.
                    if vacate_follows && gs_kts <= VACATE_HANDOFF_KTS {
                        writes.flush(&xp).await;
                        let _ = xp.set_float(TAXI_BRAKE, 0.0).await;
                        let _ = xp.set_float(TAXI_STEER, 0.0).await;
                        if rollout_brake_steer {
                            let _ = tokio::join!(
                                xp.set_float(crate::autotaxi::LEFT_BRAKE, 0.0),
                                xp.set_float(crate::autotaxi::RIGHT_BRAKE, 0.0),
                            );
                        }
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({ "event": "rolling_handoff", "gs": gs_kts }),
                        );
                        break;
                    }
                    if gs_kts > 2.0 {
                        // The steering pair every tick with the stop: the
                        // plugin's channels are one-second heartbeats.
                        // With the lever, the rudder and the column: the
                        // rollout's ONE write round (it was two or three,
                        // 0.48 s a tick against a 300 ms sleep, 2026-09-30).
                        writes.set(TAXI_BRAKE, brakes.sym);
                        if rollout_brake_steer {
                            writes
                                .set(crate::autotaxi::LEFT_BRAKE, brakes.left)
                                .set(crate::autotaxi::RIGHT_BRAKE, brakes.right);
                        }
                        writes.flush(&xp).await;
                        set_panel(&format!("AUTOLAND {} {} ROLLOUT", rwy.icao, rwy.id));
                        pacer.wait(std::time::Duration::from_millis(ROLLOUT_TICK_MS)).await;
                        continue;
                    }
                    // Stopped: squeeze the brakes, free the nosewheel, then
                    // hand the hold to the PARKING brake through the
                    // valve-aware seam, and say what the read-back says. The
                    // squeeze is the plugin's one-second heartbeat: on its own
                    // it let go the moment this loop ended, and at EGSS the
                    // 737 sat on the runway after "Brakes on" with
                    // `parking_brake_ratio` at 0.03 (2026-09-22).
                    writes.flush(&xp).await;
                    let _ = xp.set_float(TAXI_BRAKE, ROLLOUT_BRAKE_HOLD).await;
                    let _ = xp.set_float(TAXI_STEER, 0.0).await;
                    if rollout_brake_steer {
                        let _ = tokio::join!(
                            xp.set_float(crate::autotaxi::LEFT_BRAKE, 0.0),
                            xp.set_float(crate::autotaxi::RIGHT_BRAKE, 0.0),
                        );
                    }
                    let valve = crate::aircraft::live_profile().and_then(|p| p.park_brake_valve_set);
                    let set = crate::tools::controls::set_park_brake_confirmed(&xp, valve, 3).await;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({
                            "event": "stopped_brakes", "held": set.held, "tries": set.tries,
                            "park": set.park, "err": set.err, "valve_set": valve,
                        }),
                    );
                    announce(stopped_line(set.held).into(), Priority::Warning);
                    break;
                }
                // Cross-track closure rate (nm/min, positive = drifting right),
                // low-pass filtered — the damping term that turns the P-only
                // centreline chase into a crisp intercept-and-rollout. P alone
                // converged asymptotically (the banana join) and S-ed under
                // wind (2026-07-24).
                let xtk_now = cross_track_nm(lat, lon, &rwy);
                let t_now = std::time::Instant::now();
                let dt_s = prev_xtk.map_or(0.0, |(_, t0)| t_now.duration_since(t0).as_secs_f64());
                let raw_rate = match prev_xtk {
                    Some((x0, _)) if (0.1..=3.0).contains(&dt_s) => (xtk_now - x0) / dt_s * 60.0,
                    _ => 0.0,
                };
                prev_xtk = Some((xtk_now, t_now));
                // Low-passed on TIME (`XTK_RATE_TAU_S`), not half per tick.
                let a = 1.0 - (-dt_s.clamp(0.05, 3.0) / XTK_RATE_TAU_S).exp();
                xtk_rate += (raw_rate.clamp(-3.0, 3.0) - xtk_rate) * a;

                // Wind crab, from the reference recipe (heading vs ground track):
                // command a heading whose TRACK aligns with the runway, not the
                // heading itself. P-only tracking left a standing 0.10 nm
                // cross-track under the LXGB crosswind — touchdown in the grass
                // (2026-07-22). LOW-PASSED: the raw read fed straight back to
                // the bug hunted the LEMG 13 final into a balk (2026-09-12,
                // see [`CrabFilter`]). Estimated every tick, applied on final.
                let crab_raw = CrabFilter::raw(psi_r.as_ref().ok().copied(), hpath_r.as_ref().ok().copied());
                let crab = crab_filter.update(crab_raw, dt_s.clamp(0.0, 3.0));

                let raw = guidance_on(
                    lat, lon, alt, radio_ft, gs_kts, ias, &rwy, approach_ias, 25.0, xtk_rate,
                    runway_width_m(&rwy),
                );
                let raw = hold_flare(raw, flaring, radio_ft, gs_kts, 25.0);
                // A missed approach called low is flown by the latch (full power,
                // straight ahead) until clear of the ground — the raw intercept
                // guidance turns back to the FAP immediately, which is a killer
                // at 50 ft (see [`GoAroundLatch`]).
                let along_nm = along_track_nm(lat, lon, &rwy);
                // The arrest keeps the servos it took (`hold_roundout`): a
                // roundout balloon over the band is still the landing, or a
                // balked one, never the autopilot's descent again.
                let held = hold_roundout(
                    raw,
                    arrest_hands,
                    radio_ft,
                    gs_kts,
                    rwy.length_m + along_nm.min(0.0) * M_PER_NM,
                    crate::aircraft::landing_distance_m(),
                );
                let held_now = held.arresting && !raw.arresting;
                let balked_now = held.balk.is_some() && raw.balk.is_none();
                if held_now != roundout_held || balked_now {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "roundout_held", "held": held_now,
                            "balk": held.balk, "radio": radio_ft,
                            "band": arrest_gates_ft(25.0, gs_kts).1}),
                    );
                }
                roundout_held = held_now;
                let raw = held;
                let committed = commit_flare_latched(
                    raw,
                    flaring,
                    flare_committed,
                    radio_ft,
                    gs_kts,
                    25.0,
                    rwy.length_m + along_nm.min(0.0) * M_PER_NM,
                    crate::aircraft::landing_distance_m(),
                    rwy.course_deg,
                );
                let committed_now = committed.phase != raw.phase;
                if committed_now && !flare_committed {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "committed_flare", "radio": radio_ft,
                                           "past_m": (-along_nm * M_PER_NM).max(0.0),
                                           "remaining_m": rwy.length_m + along_nm.min(0.0) * M_PER_NM}),
                    );
                }
                // A commitment the room no longer covers is a go-around.
                if flare_committed && raw.balk.is_some() && !committed_now {
                    let remaining_m = rwy.length_m + along_nm.min(0.0) * M_PER_NM;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "commit_released", "why": raw.balk,
                                           "radio": radio_ft, "remaining_m": remaining_m}),
                    );
                }
                let (raw, refused) =
                    wheels_hold_the_landing(committed, on_ground > 0.5, flaring, rwy.course_deg);
                if let Some(why) = refused {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "balk_refused_on_ground", "why": why,
                                           "radio": radio_ft, "ias": ias, "gs": gs_kts}),
                    );
                }
                // A controller's order beats anything the approach thinks of
                // itself. Watched here because this is the thing that can ACT
                // on it; the watch is armed with the approach above, so only
                // an order given during this approach can trip it.
                // The pilot's order is flown the same way and answered, so
                // the `go_around` tool reads back what the aircraft does.
                if let Some(by) = orders.take() {
                    let already = go_around.active();
                    let flown = go_around.order(on_ground > 0.5);
                    if flown {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "ordered_go_around", "radio": radio_ft,
                                               "dist": raw.dist_nm, "by": by.label()}),
                        );
                    }
                    // Answered for the tower's too: a pilot's order the
                    // same tick is folded into it.
                    use crate::go_around::Answer;
                    crate::go_around::answer(if flown {
                        Answer::Flying
                    } else if already {
                        Answer::Already
                    } else {
                        Answer::OnGround
                    });
                }
                // A final the tower is working but has not cleared: ask
                // again at 1,000 ft, go around at 300 (`UnclearedWatch`).
                // The tower's hold is judged by the monitor on its own; this
                // is the crew never landing uncleared (#155).
                match uncleared.step(
                    crate::party_line::clearance_state() == Some(crate::tower::State::Inbound),
                    radio_ft,
                    on_ground > 0.5,
                    go_around.active(),
                ) {
                    Some(UnclearedStep::Ask) => {
                        if going_around_callsign.is_none() {
                            going_around_callsign = Some(crate::circuit::crew_callsign(&xp).await);
                        }
                        let cs = going_around_callsign.clone().flatten();
                        crate::circuit::transmit(crate::circuit::radio_call(
                            &cs,
                            &format!(
                                "final runway {}, request landing clearance",
                                crate::circuit::spell_runway(&rwy.id)
                            ),
                        ));
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "landing_clearance_asked", "rwy": rwy.id,
                                               "radio": radio_ft, "dist": raw.dist_nm,
                                               "held": crate::party_line::landing_hold_armed()}),
                        );
                    }
                    Some(UnclearedStep::GoAround) => {
                        if go_around.order(on_ground > 0.5) {
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "uncleared_go_around", "rwy": rwy.id,
                                                   "radio": radio_ft, "dist": raw.dist_nm,
                                                   "held": crate::party_line::landing_hold_armed()}),
                            );
                            announce(
                                "No landing clearance. Going around.".into(),
                                Priority::Caution,
                            );
                        }
                    }
                    None => {}
                }
                // Unstabilised at the thousand-foot gate is a go-around, not a
                // quiet re-intercept: the latch only arms itself below the
                // terrain floor, so it is ordered.
                // So is one found unstabilised below the floor on the FIRST
                // tick (a handoff that arrives that way, LEMG 13 2026-09-28):
                // with no final before it, the latch's own transition never
                // sees it.
                if matches!(raw.balk, Some("unstabilised_at_gate" | "unstabilised_below_floor")) {
                    go_around.order(on_ground > 0.5);
                }
                // THE STABILISED APPROACH (`stable_approach`, #199): by the
                // gate the landing configuration, on speed, not sinking hard
                // and on the path, or a go-around THEN, with height in hand.
                // The lateral half is the guidance's own gate above. EGNX 09
                // (2026-09-30): 255 kt clean and 1,400 fpm down from 1,000 ft
                // to 121 ft radio, on the centreline, so nothing balked.
                let height_aal = alt - rwy.elev_ft;
                let judging = raw.phase == ApproachPhase::Final
                    && !raw.arresting
                    && !go_around.active()
                    && on_ground < 0.5
                    && along_nm > STABILISED_MIN_ALONG_NM
                    && crate::stable_approach::judged(gate, height_aal);
                let flap_limits_now = crate::aircraft::flap_limits();
                let stable_sample = crate::stable_approach::Sample {
                    height_aal_ft: height_aal,
                    dist_to_aim_nm: along_nm.max(0.0) + TOUCHDOWN_AIM_NM,
                    ias_kts: ias,
                    gs_kts,
                    // What the governor holds in the landing configuration.
                    target_kts: approach_target_kts(
                        approach_ias,
                        1.0,
                        &flap_limits_now,
                        crate::aircraft::config_floor_kts(1.0),
                    ),
                    vvi_fpm: prev_vvi.map(|(v, _)| v),
                    above_path_ft: alt - raw.glidepath_alt_ft,
                    path_ft_per_nm: gp_ft_per_nm(),
                    gear_down,
                    landing_flap: last_flap.is_some_and(|f| f >= LANDING_FLAP_RATIO),
                };
                let now_breaches = if judging {
                    crate::stable_approach::breaches(gate, &stable_sample)
                } else {
                    Vec::new()
                };
                if judging && !gate_recorded {
                    gate_recorded = true;
                    crate::blackbox::record(
                        "autoland",
                        stable_record("stable_gate", gate, &stable_sample, &now_breaches, None),
                    );
                }
                if let Some(breach) =
                    stable_watch.step(stable_clock.elapsed().as_secs_f64(), &now_breaches)
                {
                    stable_watch.reset();
                    if go_around.order(on_ground > 0.5) {
                        stable_balk = Some(breach.word());
                        stable_said = Some(breach.spoken());
                        crate::blackbox::record(
                            "autoland",
                            stable_record(
                                "unstabilised",
                                gate,
                                &stable_sample,
                                &now_breaches,
                                Some(breach),
                            ),
                        );
                    }
                }
                if go_around.active() {
                    stable_watch.reset();
                    gate_recorded = false;
                }
                let mut missed = go_around.update(
                    last_phase,
                    raw.phase,
                    radio_ft,
                    raw.dist_nm,
                    along_nm,
                    join_point_nm(alt, rwy.elev_ft),
                    xtk_now,
                );
                flaring = raw.phase == ApproachPhase::Flare && missed.is_none();
                // The latch lives as long as the flare it committed.
                flare_committed = flaring && (flare_committed || committed_now);
                // THE COURSE REVERSAL. Started by the missed approach's
                // reposition stage, or by an ordinary approach that finds
                // itself steering at a centreline point it would reach flying
                // away from the runway (or called "final" while doing so).
                let track_true = hpath_r
                    .as_ref()
                    .ok()
                    .or(psi_r.as_ref().ok())
                    .copied()
                    .unwrap_or(rwy.course_deg);
                // Set on the tick a pattern begins: its own line says what
                // happens, so the phase change says nothing more.
                let mut reversal_began = false;
                // A refused pattern is not asked for again: its hold flies.
                if reversal.is_none() && !hold.as_ref().is_some_and(|h| h.refused) {
                    // The guidance's own aim: a join point behind only when
                    // the height cannot come off on the way in (#131).
                    let aim_along_nm = if aims_at_join_point(
                        alt - raw.glidepath_alt_ft,
                        along_nm,
                        xtk_now,
                        gs_kts,
                    ) {
                        join_point_nm(alt, rwy.elev_ft)
                    } else {
                        FAP_NM
                    };
                    // A steep, visual-segment approach (LSZS) is never
                    // repositioned by a pattern of our own: that is a valley.
                    // A go-around there flies the PUBLISHED missed approach.
                    // PUBLISHED steep only: a circuit's path steepened over
                    // the ground (#182) is an ordinary field, its reversal
                    // terrain-probed (LEMG 13 SR22 2026-09-29: 3.26 degrees
                    // took the valley branch, no pattern when high on the
                    // join, then no missed approach to fly after the balk).
                    let steep = published_steep() && !missed_procedure_tried;
                    let wanted = match missed {
                        Some(MissedStage::Reposition) => true,
                        Some(MissedStage::Climb) => false,
                        None if steep => false,
                        None => {
                            let needed = raw.phase != ApproachPhase::Flare
                                && radio_ft >= TERRAIN_FLOOR_AGL_FT
                                && crate::reversal::needed(
                                    &rwy,
                                    lat,
                                    lon,
                                    track_true,
                                    aim_along_nm,
                                    raw.phase == ApproachPhase::Final,
                                );
                            // Inside the FAP and pointing down the runway, a
                            // pattern is not the answer to "too high": a crew
                            // GOES AROUND, straight ahead, and the latch's own
                            // reposition follows the climb. Armed at 2.6 nm
                            // 800 ft high, the 737 turned straight onto a
                            // downwind from 1,600 ft at LEMG 13 (2026-09-26).
                            if needed
                                && late_go_around(along_nm, heading_diff(track_true, rwy.course_deg))
                                && go_around.order(on_ground > 0.5)
                            {
                                crate::blackbox::record(
                                    "autoland",
                                    serde_json::json!({"event": "late_go_around", "dist": raw.dist_nm,
                                        "radio": radio_ft, "above_path_ft": alt - raw.glidepath_alt_ft}),
                                );
                                announce(
                                    "Too high to land from here. Going around.".to_string(),
                                    Priority::Caution,
                                );
                                false
                            } else {
                                needed
                            }
                        }
                    };
                    if wanted && steep {
                        missed_procedure_tried = true;
                        let missed_legs = crate::procedure::take_missed_legs();
                        let vy = crate::aircraft::live_profile()
                            .and_then(|p| p.reference.as_ref().and_then(|r| r.vy_kts))
                            .unwrap_or(approach_ias * 1.3);
                        let flying = match missed_legs {
                            Some(legs) => crate::procedure::start_scoped(
                                base_url.clone(),
                                voice.clone(),
                                crate::procedure::Plan {
                                    name: "the missed approach".to_string(),
                                    legs,
                                    end: crate::procedure::EndAction::Level,
                                    vy_kts: vy,
                                    cruise_ias_kts: None,
                                    transition_altitude_ft: None,
                                    climb_minimums: Vec::new(),
                                    arrival_constraints: Vec::new(),
                                    visual_deg: None,
                                    missed_legs: Vec::new(),
                                    cruise_schedule: Default::default(),
                                },
                                Vec::new(),
                            ),
                            None => false,
                        };
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "missed_procedure", "flying": flying, "radio": radio_ft}),
                        );
                        if flying {
                            gave_up = true;
                            announce(
                                "Flying the published missed approach.".to_string(),
                                Priority::Caution,
                            );
                            break;
                        }
                        // No published missed approach to fly: our own
                        // terrain-probed pattern below (`reversal::plan`,
                        // which climbs straight ahead and HOLDS a height
                        // clear of the ground where no pattern fits). The
                        // old "climbing straight ahead" hand-back left
                        // an unattended SR22 at 805 ft with nobody flying
                        // (LEMG 13 2026-09-29).
                    }
                    if wanted && reversals_flown >= crate::reversal::MAX_REVERSALS {
                        gave_up = true;
                        let thr_dref = throttle_dref();
                        let _ = tokio::join!(
                            xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                            xp.set_float(VS_DIAL, 0.0),
                        );
                        announce(
                            format!(
                                "I can't get us established for runway {}. Your aircraft. \
                                 The autopilot is holding heading and altitude{}.",
                                crate::circuit::spell_runway(&rwy.id),
                                if gear_down { ", gear is down" } else { "" }
                            ),
                            Priority::Warning,
                        );
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "give_up", "reason": "max_reversals",
                                               "flown": reversals_flown}),
                        );
                        break;
                    }
                    // The pattern is planned over the probed ground (#182,
                    // `reversal::plan`), beside the loop: begun in the
                    // go-around's climb (the reposition will want it), or the
                    // tick it is wanted; a plan over a minute old is stale.
                    let default_platform = rwy.elev_ft + REPOSITION_MIN_AGL_FT;
                    if reversal_plan
                        .as_ref()
                        .is_some_and(|(_, t)| t.elapsed().as_secs_f64() > REVERSAL_PLAN_FRESH_S)
                    {
                        reversal_plan = None;
                    }
                    if (wanted || missed == Some(MissedStage::Climb)) && reversal_plan.is_none() {
                        let task = crate::reversal::plan(
                            XPlaneClient::new(base_url.clone()).with_parents(xp.cancellation()),
                            rwy.clone(),
                            along_nm,
                            xtk_now,
                            gs_kts,
                            default_platform,
                            (lat, lon, track_true, alt),
                        );
                        reversal_plan = Some((tokio::spawn(task), std::time::Instant::now()));
                    }
                    // Waiting on the plan: straight ahead, never lower than
                    // the platform the old rule would have flown.
                    if wanted && hold.is_none() {
                        hold = Some(TerrainHold::waiting(track_true, alt.max(default_platform)));
                    }
                    if !wanted && hold.as_ref().is_some_and(|h| !h.refused) {
                        hold = None;
                    }
                    let ready = wanted
                        && reversal_plan.as_ref().is_some_and(|(h, _)| h.is_finished());
                    let planned = if ready {
                        match reversal_plan.take() {
                            Some((h, _)) => h.await.ok(),
                            None => None,
                        }
                    } else {
                        None
                    };
                    if let Some(plan) = planned {
                        let above_path_ft = alt - raw.glidepath_alt_ft;
                        let mut record = serde_json::json!({"event": "reversal", "side": plan.side,
                            "platform_ft": plan.platform_ft,
                            "xtk_m": xtk_now * M_PER_NM, "dist": raw.dist_nm,
                            "track": track_true, "missed": missed.is_some(),
                            "final_called": raw.phase == ApproachPhase::Final,
                            "why": crate::reversal::why(
                                missed.is_some(),
                                raw.phase == ApproachPhase::Final,
                                above_path_ft,
                            ),
                            "above_path_ft": above_path_ft, "aim_nm": aim_along_nm,
                            "along": along_nm, "ias": ias, "gs": gs_kts});
                        if let (Some(r), Some(p)) = (record.as_object_mut(), plan.record.as_object()) {
                            r.extend(p.clone());
                        }
                        crate::blackbox::record("autoland", record);
                        match (plan.refused.as_deref(), plan.hold_ft) {
                            // No pattern here clears the ground: climb straight
                            // ahead to a height clear of it and hold it on the
                            // autopilot, said with the numbers; the aircraft is
                            // handed over only once it is there.
                            (Some(why), hold_ft) => {
                                let to = hold_ft.unwrap_or_else(|| alt.max(default_platform));
                                hold = Some(TerrainHold::refused(track_true, to));
                                announce(
                                    format!(
                                        "No pattern for runway {} clears the ground: {why}. \
                                         Climbing straight ahead to {:.0} feet and holding it.",
                                        crate::circuit::spell_runway(&rwy.id),
                                        to
                                    ),
                                    Priority::Warning,
                                );
                            }
                            (None, _) => {
                                hold = None;
                                reversals_flown += 1;
                                reversal = Some(crate::reversal::Reversal {
                                    side: plan.side,
                                    leg: crate::reversal::Leg::Outbound,
                                    platform_ft: plan.platform_ft,
                                    high: false,
                                });
                                if missed == Some(MissedStage::Reposition) {
                                    go_around.hold_reposition();
                                }
                                reversal_began = true;
                                pattern_gs_kts = gs_kts;
                                if matches!(
                                    plan.why,
                                    "switched_side_for_terrain" | "raised_for_terrain"
                                ) {
                                    crate::blackbox::record(
                                        "autoland",
                                        serde_json::json!({"event": "reversal_terrain",
                                            "why": plan.why, "platform_ft": plan.platform_ft}),
                                    );
                                }
                                announce(
                                    format!(
                                        "Repositioning for runway {}: {} downwind at {:.0} feet{}.",
                                        crate::circuit::spell_runway(&rwy.id),
                                        if plan.side < 0.0 { "left-hand" } else { "right-hand" },
                                        (plan.platform_ft / 100.0).round() * 100.0,
                                        match plan.why {
                                            "raised_for_terrain" => ", raised for the high ground",
                                            "switched_side_for_terrain" => ", the other side is high ground",
                                            _ => "",
                                        }
                                    ),
                                    Priority::Advisory,
                                );
                            }
                        }
                    }
                }
                // Fly it: the pattern owns the heading, the platform and the
                // speed until the aircraft is inbound and the ordinary
                // guidance calls a final it is actually pointing down.
                let mut pattern: Option<(f64, crate::reversal::Leg, f64)> = None;
                // An inbound leg that went over the field is over: left
                // alone it flies the runway heading away for ever.
                if reversal.as_ref().is_some_and(|r| crate::reversal::overflown(r, along_nm)) {
                    reversal = None;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "reversal_overflown", "alt": alt,
                            "radio": radio_ft, "dist": raw.dist_nm, "flown": reversals_flown}),
                    );
                }
                // An inbound leg the glidepath has come down to is an
                // approach: the pattern is done, the path takes the descent.
                if reversal.as_ref().is_some_and(|r| {
                    crate::reversal::path_joined(
                        r,
                        along_nm,
                        xtk_now,
                        track_true,
                        rwy.course_deg,
                        alt - raw.glidepath_alt_ft,
                    )
                }) {
                    reversal = None;
                    go_around.finish_reposition();
                    missed = None;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "reversal_done", "why": "path_joined",
                            "dist": raw.dist_nm, "xtk_m": xtk_now * M_PER_NM,
                            "above_path_ft": alt - raw.glidepath_alt_ft}),
                    );
                }
                if let Some(rev) = reversal.as_mut() {
                    recede_ref_nm = recede_ref_nm.max(raw.dist_nm);
                    let join = join_point_nm(rev.platform_ft, rwy.elev_ft);
                    // Height first: a downwind begun far above the platform
                    // extends until the rest of the pattern can lose it.
                    let high = rev.leg == crate::reversal::Leg::Outbound
                        && crate::reversal::too_high_to_turn(
                            alt,
                            rev.platform_ft,
                            along_nm,
                            join,
                            gs_kts,
                            crate::procedure::descent_fpm(),
                        );
                    if high && !rev.high {
                        let above = ((alt - rev.platform_ft) / 100.0).round() * 100.0;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "downwind_extended", "above_ft": above,
                                               "along_nm": along_nm, "alt": alt}),
                        );
                        announce(
                            format!("We're {above:.0} feet above the pattern. Extending the downwind to lose it."),
                            Priority::Advisory,
                        );
                    }
                    rev.high = high;
                    match crate::reversal::step(
                        rev,
                        &rwy,
                        lat,
                        lon,
                        along_nm,
                        xtk_now,
                        track_true,
                        gs_kts,
                        join,
                        raw.phase == ApproachPhase::Final,
                    ) {
                        Some(h) => pattern = Some((h, rev.leg, rev.platform_ft)),
                        None => {
                            reversal = None;
                            go_around.finish_reposition();
                            missed = None;
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "reversal_done",
                                    "dist": raw.dist_nm, "xtk_m": xtk_now * M_PER_NM}),
                            );
                        }
                    }
                }
                let ga = missed.is_some();
                // The tower is owed this the moment it happens: it is holding a
                // landing clearance for an aircraft that is no longer landing,
                // and it works the traffic underneath us. Luton 2026-07-28: the
                // autoland balked, climbed away and repositioned over the field
                // for four minutes, and nothing was ever said — the FSM sat in
                // ClearedToLand throughout. Announced ONCE, once the go-around
                // is real: a balk low in the flare can still land (LFBD 11,
                // 2026-09-27), and then it is a landing, cleared, and nothing
                // goes out (see [`go_around_call_due`], `balk_landed`).
                if missed.is_none() {
                    balk_watch = None;
                } else if balk_watch.is_none() {
                    // The stabilised gate's word when it ordered this one.
                    balk_watch = Some(BalkWatch::new(raw.balk.or(stable_balk.take()), radio_ft));
                }
                if let Some(w) = balk_watch.as_mut() {
                    w.note(radio_ft, on_ground > 0.5);
                }
                if go_around_call_due(
                    missed.is_some(),
                    said_going_around,
                    on_ground > 0.5,
                    balk_watch.as_ref(),
                    radio_ft,
                    prev_vvi.map(|(v, _)| v),
                ) {
                    said_going_around = true;
                    // The go-around VOIDS the landing clearance, so this
                    // approach owes the tower a fresh final report before it
                    // can be cleared again — the call that earns it.
                    owe_final_report = true;
                    if going_around_callsign.is_none() {
                        going_around_callsign = Some(crate::circuit::crew_callsign(&xp).await);
                    }
                    let cs = going_around_callsign.clone().flatten();
                    crate::circuit::transmit(crate::circuit::radio_call(&cs, "going around"));
                    // `radio` and `why` are the BALK's (the debrief's
                    // "GO-AROUND at N ft radio: why"); `called_radio` is
                    // where the climb made it real.
                    let w = balk_watch.unwrap_or(BalkWatch::new(raw.balk, radio_ft));
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "going_around", "radio": w.balk_radio_ft,
                                           "why": w.why, "called_radio": radio_ft,
                                           "low_ft": w.low_ft, "touched": w.touched,
                                           "xtk_m": xtk_now * M_PER_NM,
                                           "dist": raw.dist_nm, "trim": track_trim.value()}),
                    );
                }
                // Back on final after a missed approach: report it. The first
                // approach is reported by whoever set this one up (the
                // circuit's own "final runway ...", or full auto's arrival);
                // the second is nobody's until here, and a tower that never
                // hears from us again holds nothing to clear us with. Said on
                // the way IN — established on final, inside the FAP — because
                // "final" is a position report and it has to be true.
                if owe_final_report
                    && missed.is_none()
                    && raw.phase == ApproachPhase::Final
                    && raw.dist_nm <= FAP_NM
                {
                    owe_final_report = false;
                    if going_around_callsign.is_none() {
                        going_around_callsign = Some(crate::circuit::crew_callsign(&xp).await);
                    }
                    let cs = going_around_callsign.clone().flatten();
                    crate::circuit::transmit(crate::circuit::radio_call(
                        &cs,
                        &format!("final runway {}", crate::circuit::spell_runway(&rwy.id)),
                    ));
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "final_report", "rwy": rwy.id,
                                           "dist": raw.dist_nm}),
                    );
                }
                // Cleared to try again: the next balked landing is a new call.
                if missed.is_none() {
                    said_going_around = false;
                    spoke_going_around = false;
                }
                // A missed approach (or a pattern) that stops getting anywhere
                // is its own emergency, judged by the ground it makes, never a
                // flat clock (`reversal::MissedWatch`): the stage's measure
                // (feet to climb, the pattern's track to go, an extended
                // downwind's height) stepping at a rate for the airframe's
                // speed, legs sequenced, inside an outer bound from the plan.
                // The flat 600 s gave up on an SR22 on the line, 90 ft under
                // the path it was about to join, on LEMG 13's terrain-raised
                // pattern (iMac 2026-09-30, sortie 1790775809-approach).
                let watched = if reversal.is_some() || hold.is_some() || missed.is_some() {
                    let progress = match (reversal.as_ref(), hold.as_ref()) {
                        (Some(rev), _) => {
                            let join = join_point_nm(rev.platform_ft, rwy.elev_ft);
                            let end = crate::reversal::pattern_end_nm(
                                rev.platform_ft,
                                rwy.elev_ft,
                                gp_ft_per_nm(),
                                join,
                            );
                            let to_go = crate::reversal::to_go_nm(
                                rev,
                                along_nm,
                                xtk_now,
                                pattern_gs_kts,
                                join,
                                end,
                            );
                            let above = alt - rev.platform_ft;
                            crate::reversal::Progress::Pattern {
                                n: reversals_flown,
                                leg: rev.leg,
                                high: rev.high,
                                to_go_nm: to_go,
                                above_ft: above,
                                budget_s: crate::reversal::pattern_budget_s(
                                    to_go,
                                    pattern_gs_kts,
                                    above,
                                    crate::procedure::descent_fpm(),
                                ),
                                gs_kts: pattern_gs_kts,
                            }
                        }
                        (None, Some(h)) if h.refused => {
                            crate::reversal::Progress::Hold { to_go_ft: h.alt_ft - alt }
                        }
                        _ if missed == Some(MissedStage::Climb) => crate::reversal::Progress::Climb {
                            to_go_ft: GO_AROUND_TURN_AGL_FT - radio_ft,
                        },
                        _ => crate::reversal::Progress::Waiting,
                    };
                    let t = watch_epoch.elapsed().as_secs_f64();
                    let w = missed_watch.get_or_insert_with(|| crate::reversal::MissedWatch::start(t));
                    if let Some(stage) = w.note(t, progress) {
                        let (to_go_nm, leg) = match progress {
                            crate::reversal::Progress::Pattern { to_go_nm, leg, .. } => {
                                (Some(to_go_nm), Some(leg.word()))
                            }
                            _ => (None, None),
                        };
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "missed_progress", "stage": stage,
                                "missed": missed.is_some(), "to_go_nm": to_go_nm, "leg": leg,
                                "gs": gs_kts, "pattern_gs": pattern_gs_kts,
                                "budget_s": w.budget_left_s(t), "alt": alt, "radio": radio_ft,
                                "dist": raw.dist_nm}),
                        );
                    }
                    w.verdict(t).map(|v| (v, progress, w.stalled_s(t), w.budget_left_s(t)))
                } else {
                    missed_watch = None;
                    None
                };
                if let Some((verdict, progress, stalled_s, budget_left_s)) = watched {
                    // A give-up must hand back a FLYABLE aircraft. The balked
                    // landing left gear down and landing flap out, and a flat
                    // 0.5 throttle cannot hold level flight dirty: the Baron
                    // decayed 110 to 80 kt at -900 fpm in the thirty seconds
                    // after the old handback (EGHI, 2026-08-02). Climb power,
                    // the VS dial zeroed (the AP is already holding heading),
                    // and the line says what is actually held.
                    gave_up = true;
                    let thr_dref = throttle_dref();
                    let _ = tokio::join!(
                        xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                        xp.set_float(VS_DIAL, 0.0),
                    );
                    announce(
                        format!(
                            "I can't get us back round for another approach. Your aircraft. \
                             The autopilot is holding heading and altitude{}.",
                            if gear_down { ", gear is down" } else { "" }
                        ),
                        Priority::Warning,
                    );
                    let to_go_nm = match progress {
                        crate::reversal::Progress::Pattern { to_go_nm, .. } => Some(to_go_nm),
                        _ => None,
                    };
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": verdict.reason(),
                            "stage": progress.word(), "to_go_nm": to_go_nm,
                            "stalled_s": stalled_s, "budget_left_s": budget_left_s,
                            "missed": missed.is_some(), "leg": reversal.map(|r| r.leg.word()),
                            "alt": alt, "radio": radio_ft, "dist": raw.dist_nm, "gs": gs_kts}),
                    );
                    break;
                }
                // Runaway backstop: an approach that keeps missing (arrives too
                // fast/high to ever capture) would loop go-arounds forever,
                // accelerating toward Vne. After MAX_GO_AROUNDS, stop trying:
                // level the wings, throttle to a safe climb, and give the
                // aircraft back — a real crew would divert, not keep diving at
                // a runway they can't make (LXGB Twin Otter, 2026-07-26).
                // Too many go-arounds: give up, but only once the climb-away
                // is flown. The third LFLL balk released the aircraft at 95 ft
                // with landing flaps and climb power, "holding altitude": it
                // accelerated at 300 ft AGL to 236 kt through the flap limit
                // (2026-09-17). The latch's Climb stage runs to
                // GO_AROUND_TURN_AGL_FT under full power with the go-around
                // flap; the handback happens there, flaps retracted.
                if go_around.attempts() > MAX_GO_AROUNDS && missed != Some(MissedStage::Climb) {
                    gave_up = true;
                    let thr_dref = throttle_dref();
                    let _ = tokio::join!(
                        xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                        xp.set_float(VS_DIAL, 0.0),
                        xp.set_float(FLAP_RATIO, 0.0),
                    );
                    announce(
                        format!(
                            "Unable to land, we've gone around too many times. Climbed away to \
                             {} feet. Your aircraft. The autopilot is holding heading and \
                             altitude, flaps up, gear is down.",
                            (radio_ft / 100.0).round() * 100.0
                        ),
                        Priority::Warning,
                    );
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "max_go_arounds", "attempts": go_around.attempts()}),
                    );
                    break;
                }
                let g = match missed {
                    Some(stage) => missed_approach_guidance(
                        &rwy,
                        stage,
                        lat,
                        lon,
                        alt,
                        radio_ft,
                        raw.dist_nm,
                        raw.glidepath_alt_ft,
                        ias,
                        approach_ias,
                        // The speed's trend since the last tick's power
                        // law read it (the same pair; nothing written).
                        prev_ias.map_or(0.0, |p| {
                            (ias - p) / prev_ias_t.elapsed().as_secs_f64().clamp(0.1, 2.0)
                        }),
                        crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0)),
                    ),
                    None => raw,
                };
                // The pattern's own command over whichever of those stood.
                let g = match pattern {
                    Some((hdg, leg, platform_ft)) => {
                        let slow = approach_ias - ias > SPEED_FLOOR_MARGIN_KTS;
                        let inbound = leg == crate::reversal::Leg::Inbound;
                        Guidance {
                            target_heading_deg: hdg,
                            target_vs_fpm: crate::reversal::platform_vs_fpm(
                                platform_ft,
                                alt,
                                slow,
                                max_climb_fpm(),
                                crate::procedure::descent_fpm(),
                            ),
                            target_ias_kts: if inbound {
                                scheduled_ias(g.dist_nm, approach_ias)
                            } else {
                                pattern_ias(approach_ias, &crate::aircraft::flap_limits())
                            },
                            phase: ApproachPhase::Intercept,
                            balk: None,
                            arresting: false,
                            ..g
                        }
                    }
                    None => g,
                };
                // ...or the hold's (#182): straight ahead at its altitude,
                // while the pattern is planned or because none clears the
                // ground. A refusal hands over only once it is there, the
                // autopilot holding heading and altitude and the governor the
                // speed (`gave_up`): never "your aircraft" low over the hills.
                let slow = approach_ias - ias > SPEED_FLOOR_MARGIN_KTS;
                if pattern.is_some() {
                    hold = None;
                }
                let hold_done = hold.as_mut().is_some_and(|h| h.level_now(alt));
                if hold_done {
                    let h = hold.take().expect("a hold that is done exists");
                    gave_up = true;
                    let _ = xp.set_float(VS_DIAL, 0.0).await;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "terrain_refused",
                                           "hold_ft": h.alt_ft, "alt": alt, "radio": radio_ft}),
                    );
                    announce(
                        format!(
                            "Level at {:.0} feet, straight ahead, the autopilot holding heading and \
                             altitude{}. Your aircraft.",
                            h.alt_ft,
                            if gear_down { ", gear is down" } else { "" }
                        ),
                        Priority::Warning,
                    );
                    break;
                }
                let g = match (pattern, hold) {
                    (None, Some(h)) => Guidance {
                        target_heading_deg: h.hdg_true,
                        target_vs_fpm: crate::reversal::platform_vs_fpm(
                            h.alt_ft,
                            alt,
                            slow,
                            max_climb_fpm(),
                            crate::procedure::descent_fpm(),
                        ),
                        target_ias_kts: pattern_ias(approach_ias, &crate::aircraft::flap_limits()),
                        phase: ApproachPhase::Intercept,
                        balk: None,
                        arresting: false,
                        ..g
                    },
                    _ => g,
                };
                // The live floor (#182): every leg but the final keeps
                // `circuit::live_floor_ft` over the ground under and 1.5 nm
                // ahead, probed every 2 s: the level legs climb 800 ft clear
                // of it, the base and an intercept only keep the floor. The
                // go-around's climb is its own; the final is the landing's.
                // A steep (visual segment, mountain) approach's intercept is
                // a valley's, flown on its published path: its walls ahead
                // are not ours to climb away from.
                let floor_leg = live_floor_leg(
                    g.phase,
                    missed,
                    pattern.map(|(_, leg, _)| leg),
                    hold.is_some(),
                )
                .filter(|_| {
                    pattern.is_some()
                        || hold.is_some()
                        || missed.is_some()
                        || glidepath_deg() <= 3.0 + 1e-9
                });
                let mut g = g;
                if let Some(leg) = floor_leg {
                    if live_floor_probed.is_none_or(|t| t.elapsed().as_secs_f64() >= LIVE_FLOOR_EVERY_S) {
                        live_floor_probed = Some(std::time::Instant::now());
                        let mut pts = vec![(lat, lon)];
                        pts.extend(crate::procedure::look_ahead_points(
                            lat,
                            lon,
                            track_true,
                            crate::circuit::LIVE_LOOK_NM,
                            crate::circuit::LIVE_LOOK_POINTS,
                        ));
                        if let Some(e) = crate::terrain::max_elevation_ft(&xp, &pts).await {
                            if let Some(floor) = crate::circuit::live_floor_ft(leg, alt, e) {
                                let raised = live_floor.is_none_or(|(f, _)| floor > f);
                                live_floor = Some((
                                    live_floor.map_or(floor, |(f, _)| f.max(floor)),
                                    std::time::Instant::now(),
                                ));
                                if raised {
                                    crate::blackbox::record(
                                        "autoland",
                                        serde_json::json!({"event": "terrain_floor",
                                            "leg": format!("{leg:?}"),
                                            "pattern_leg": pattern.map(|(_, l, _)| l.word()),
                                            "hold": hold.is_some(), "ground_ft": e, "alt": alt,
                                            "radio": radio_ft, "floor_ft": floor}),
                                    );
                                }
                                if !live_floor_said {
                                    live_floor_said = true;
                                    announce(
                                        format!(
                                            "Terrain ahead. Holding {:.0} feet.",
                                            (floor / 50.0).ceil() * 50.0
                                        ),
                                        Priority::Warning,
                                    );
                                }
                            }
                        }
                    }
                } else {
                    live_floor = None;
                }
                // A spell ends with its floor: the next one is said again.
                live_floor = live_floor.filter(|(_, t)| t.elapsed().as_secs_f64() < LIVE_FLOOR_HOLD_S);
                if live_floor.is_none() {
                    live_floor_said = false;
                }
                if let Some((floor, _)) = live_floor {
                    g.target_vs_fpm = g.target_vs_fpm.max(crate::reversal::platform_vs_fpm(
                        floor,
                        alt,
                        slow,
                        max_climb_fpm(),
                        crate::procedure::descent_fpm(),
                    ));
                }
                let g = g;
                let outbound = hold.is_some()
                    || pattern.is_some_and(|(_, leg, _)| leg != crate::reversal::Leg::Inbound);
                let trim = track_trim.update(
                    g.phase,
                    g.target_heading_deg,
                    hpath_r.as_ref().ok().copied(),
                    dt_s.clamp(0.0, 3.0),
                );
                let phase_word = match g.phase {
                    ApproachPhase::Intercept => "INTERCEPT",
                    ApproachPhase::Final => "FINAL",
                    ApproachPhase::Flare => "FLARE",
                };
                set_panel(&format!(
                    "AUTOLAND {} {} {:.1}nm {}",
                    rwy.icao, rwy.id, g.dist_nm, phase_word
                ));
                tracing::info!(
                    target: "xvoicelayer::autoland",
                    dist = g.dist_nm, alt, gp_alt = g.glidepath_alt_ft, vs = g.target_vs_fpm,
                    hdg = g.target_heading_deg, ias, radio = radio_ft, phase = ?g.phase,
                    "autoland"
                );
                // The vertical speed actually flown: the watchdog below reads
                // it, and so does the landing arrest, whose dial leads the
                // target through the autopilot's lag (`vs_dial_fpm`). The
                // blackbox keeps both columns: `cmd_vs` is what the law
                // wants, `vs_dial` what the autopilot was told.
                // A failed read is NO measurement, never the command in
                // disguise: the dial gets the bare target that tick, the
                // watchdog does not count it, and the blackbox keeps `null`
                // so the debrief can see the gauge was missing.
                // From the tick's one read (`st`), no round of its own.
                let vvi_read = st.get(F::Vvi);
                let throttle_read = throttle_read
                    .take()
                    .unwrap_or_else(|| Err(anyhow::anyhow!("unread")));
                let servos_read = if engage.confirmed() { None } else { st.opt(F::Servos) };
                // The engage watch (see `ApEngage`): re-press until the
                // servos read on with the sim live, each try on the tape.
                let engage_now = engage_t0.elapsed().as_secs_f64();
                // Airborne only: a servo pressed on the wheels winds the
                // yoke over (`disconnect_autopilot`).
                let es = if on_ground < 0.5 && !rolling_out {
                    engage.step(engage_now, servos_read.filter(|v| v.is_finite()), ias)
                } else {
                    EngageStep::default()
                };
                if es.fire {
                    for c in &cfg.fd_commands { let _ = xp.run_command(c).await; }
                    let ok = xp.run_command(CMD_SERVOS_ON).await.is_ok();
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "ap_engage", "try": es.try_n, "sent": ok,
                            "at_s": engage_now, "servos": servos_read, "ias": ias,
                            "radio": radio_ft}),
                    );
                }
                if es.confirmed_now {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "ap_engaged", "tries": es.try_n,
                            "at_s": engage_now, "held": engage_held, "radio": radio_ft}),
                    );
                    authority_owed = true;
                }
                if es.give_up {
                    gave_up = true;
                    let _ = xp.set_float(&throttle_dref(), GIVE_UP_THROTTLE).await;
                    announce(ENGAGE_GIVE_UP_LINE.into(), Priority::Warning);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "ap_not_engaged",
                            "tries": es.try_n, "at_s": engage_now, "radio": radio_ft}),
                    );
                    break;
                }
                let thr_now = throttle_read.ok().filter(|v| v.is_finite());
                let vvi = match vvi_read {
                    Ok(v) => Some(v),
                    Err(e) => {
                        if !vvi_read_failed {
                            vvi_read_failed = true;
                            tracing::warn!(target: "xvoicelayer::autoland", error = %e, "VSI read failed; flying the bare VS target");
                        }
                        None
                    }
                };
                // The sink's rate of change over the last tick (fpm per
                // second), the arrest's damping term on a slow autopilot.
                let tick_dt = prev_vvi
                    .map(|(_, t0)| t_now.duration_since(t0).as_secs_f64())
                    .filter(|d| (0.02..=3.0).contains(d))
                    .unwrap_or(0.2);
                let vvi_trend = match (vvi, prev_vvi) {
                    (Some(v), Some((p, t0))) => {
                        let dt = t_now.duration_since(t0).as_secs_f64();
                        if (0.1..=3.0).contains(&dt) { (v - p) / dt } else { 0.0 }
                    }
                    _ => 0.0,
                };
                prev_vvi = vvi.map(|v| (v, t_now));
                let arrest_lead = crate::aircraft::arrest_lead();
                let vs_dial = match vvi {
                    Some(v) => vs_dial_fpm(g.target_vs_fpm, v, vvi_trend, g.arresting, arrest_lead),
                    None => g.target_vs_fpm,
                };
                // The servos' pitch AUTHORITY (#181): the arrest takes the
                // servos out, so the check ends there; before it, a dial the
                // autopilot's elevator does not fly is hand flown level by
                // the recovery and the approach abandoned.
                if g.arresting || !engage.confirmed() || es.hold {
                    if g.arresting {
                        crate::climb_watch::disarm_authority();
                        authority_owed = false;
                    }
                } else if authority_owed {
                    if let Some(v) = vvi {
                        crate::climb_watch::arm_authority("autoland", v, vs_dial);
                        authority_owed = false;
                    }
                } else if let Some(crate::climb_watch::Authority::NoAuthority(why)) =
                    crate::climb_watch::feed_authority(vvi, Some(vs_dial))
                {
                    let started = crate::tools::recover::take_over(
                        base_url.clone(),
                        voice.clone(),
                        crate::tools::recover::TakeOver {
                            why: format!("autoland_{why}"),
                            min_safe_ias: crate::aircraft::config_floor_kts(
                                last_flap.unwrap_or(0.0),
                            )
                            .map(|f| f + crate::circuit::RECOVERY_MARGIN_KTS),
                            hold_kts: None,
                            ap_no_pitch: true,
                            hold_ft: None,
                        },
                    );
                    announce("Abandoning the approach.".into(), Priority::Warning);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "ap_no_pitch",
                            "why": why, "recovery_started": started, "radio": radio_ft}),
                    );
                    break;
                }
                // THE ARREST FLIES THE ELEVATOR, not the autopilot's VS dial.
                // Only while actually arresting, and only with both
                // measurements in hand: anywhere else the dial is right and
                // proven. The channel is a 1 s expiring heartbeat, so simply
                // ceasing to write it hands the pitch back by itself.
                // The final's air, for the flare (`FinalAir`): measured on the
                // way down to the gate with the autopilot flying, begun again
                // above it (a go-around's climb, a new pattern).
                if radio_ft > FINAL_AIR_FROM_FT {
                    final_air = FinalAir::default();
                } else if let (ApproachPhase::Final, false, false, Some(v)) =
                    (g.phase, g.arresting, es.hold, vvi)
                {
                    final_air.note(v, g.target_vs_fpm);
                }
                let mut model_out: Option<FlareModelOut> = None;
                // A model that carries the speed leads its bleed (`fpm_per_kt`).
                if let Some(m) = flare_model.as_mut() {
                    m.note_speed(ias, tick_dt);
                }
                // A HOLD-OFF held on from its first contact (`TouchHold`):
                // the wheels, or the radio under TOUCH_RADIO_FT, in the
                // flare. A go-around or a new pattern starts free.
                if missed.is_some() || g.phase == ApproachPhase::Intercept {
                    touch_hold.reset();
                    last_flare_elev = None;
                } else if holds_off
                    && !pitch_yielded
                    && g.phase == ApproachPhase::Flare
                    && touch_hold.note(on_ground > 0.5 || radio_ft < TOUCH_RADIO_FT, last_flare_elev)
                {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "touch_hold", "held": touch_hold.held(),
                            "at": "flare", "radio": radio_ft, "gs": gs_kts, "ias": ias,
                            "vvi": vvi, "on_ground": on_ground > 0.5}),
                    );
                }
                let touch_elev = if touch_hold.touched() && !pitch_yielded {
                    touch_hold.elev(gs_kts, on_ground > 0.5, tick_dt)
                } else {
                    None
                };
                let flare_elev = match (g.arresting, vvi, pitch_yielded) {
                    // Once the pilot has taken the pitch it stays taken for
                    // this approach — the debounce the taxi and takeoff
                    // channels learned: a latch that re-arms is a fight over
                    // a control with a person on the other end.
                    (true, Some(v), false) => match flare_model.as_mut() {
                        Some(m) => {
                            let o = m.step(
                                radio_ft,
                                gs_kts,
                                v,
                                tick_dt,
                                g.phase == ApproachPhase::Flare,
                                final_air.rough(),
                            );
                            model_out = Some(o);
                            Some(o.elev)
                        }
                        None => Some(flare_pitch.step(
                            g.target_vs_fpm,
                            v,
                            vvi_trend,
                            tick_dt,
                            g.phase == ApproachPhase::Flare,
                        )),
                    },
                    // No sink read: no decision. The model keeps time with
                    // what is on the channel (it expires by itself).
                    (true, None, false) if flare_model.is_some() => {
                        if let Some(m) = flare_model.as_mut() {
                            m.coast(tick_dt, radio_ft);
                        }
                        None
                    }
                    // NOBODY IS FLYING: the autopilot is not confirmed and
                    // its servos read off. Level on the elevator until it
                    // is (`ApEngage`, `engage_hold_gains`); the same
                    // channel, the same single write below.
                    (false, Some(v), false) if es.hold => {
                        engage_held = true;
                        flare_pitch.reset();
                        if let Some(m) = flare_model.as_mut() {
                            m.reset();
                        }
                        Some(engage_pitch.step(0.0, v, vvi_trend, tick_dt, false))
                    }
                    _ => {
                        flare_pitch.reset();
                        if let Some(m) = flare_model.as_mut() {
                            m.reset();
                        }
                        None
                    }
                };
                // Held on: the pull it touched with, whatever the flare
                // would have asked.
                let flare_elev = if touch_hold.touched() && !pitch_yielded {
                    touch_elev
                } else {
                    flare_elev
                };
                if flare_elev.is_some() {
                    last_flare_elev = flare_elev;
                }
                // THE ARREST TAKES THE SERVOS. The pitch channel writes the
                // yoke under the joystick override and the autopilot's pitch
                // servo writes it back the same frame: LFPG 09R (2026-09-22)
                // held 0.35 of elevator for three seconds and touched down at
                // 0.1 degrees of pitch and -672 fpm, the sink following the
                // VS dial's own slow shape exactly as it had before the
                // elevator law existed (every 737 landing since it shipped
                // shows the same). The channel is proven with the servos
                // OFF - the takeoff rotates on it - so the roundout gate is
                // where the autopilot comes off: pitch on the elevator
                // channel, the runway track on the roll channel (the
                // takeoff's wings-level law, banking toward the crabbed
                // heading), the power on the governor. Only a go-around
                // hands the servos back and re-engages the modes (a balloon
                // over the band stays the arrest, `hold_roundout`); the
                // rollout latch finds them already off.
                // The last 0.3 nm of the final: the centreline closed in
                // seconds, damped on the drift over the ground (see
                // `short_final_cut_deg`), where the old 80 deg/nm let 1 degree
                // of track build 9 m from 170 ft to the gate.
                let off_course = |a: f64| ((a - rwy.course_deg + 540.0).rem_euclid(360.0)) - 180.0;
                let track_off = hpath_r.as_ref().ok().copied().map(off_course);
                let in_short_final = g.phase == ApproachPhase::Final
                    && g.dist_nm <= SHORT_FINAL_NM
                    && !g.arresting;
                let crabbed_true = match (g.phase, in_short_final, track_off) {
                    (ApproachPhase::Final, true, Some(to)) => {
                        let pursuit_bug = g.target_heading_deg + crab + trim;
                        let sf = short_final
                            .get_or_insert_with(|| ShortFinal::new(off_course(pursuit_bug), to));
                        (rwy.course_deg
                            + sf.step(xtk_now * M_PER_NM, to, gs_kts, tick_dt, phi_r.as_ref().ok().copied()))
                            .rem_euclid(360.0)
                    }
                    (ApproachPhase::Final | ApproachPhase::Flare, ..) => {
                        short_final = None;
                        (g.target_heading_deg + crab + trim).rem_euclid(360.0)
                    }
                    (ApproachPhase::Intercept, ..) => {
                        short_final = None;
                        g.target_heading_deg
                    }
                };
                match arrest_hands_step(arrest_hands, g.arresting) {
                    ArrestHands::Take => {
                        let ok = xp.run_command(CMD_SERVOS_OFF).await.is_ok();
                        arrest_hands = true;
                        wings = arrest_wings();
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({
                                "event": "arrest_hands", "taken": true, "ok": ok, "radio": radio_ft,
                                // The final's air the flare will be flown for.
                                "air_rms": final_air.rms_fpm(), "rough": final_air.rough(),
                            }),
                        );
                    }
                    ArrestHands::Return => {
                        let ok = xp.run_command(CMD_SERVOS_ON).await.is_ok();
                        let _ = xp.run_command(CMD_HDG_MODE).await;
                        let _ = xp.run_command(CMD_VS_MODE).await;
                        let _ = xp.set_float(VS_DIAL, vs_dial).await;
                        arrest_hands = false;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "arrest_hands", "taken": false, "ok": ok, "radio": radio_ft}),
                        );
                    }
                    ArrestHands::Off | ArrestHands::Keep => {}
                }
                // The wings while the arrest has the servos: a bank that
                // closes the centreline by the TRACK over the runway (see
                // `arrest_bank_target`), the sim's ground track against the
                // course, the smoothed cross-track rate when no track was
                // read. No bank read is no command (the channel expires).
                // The approach's pursuit aimed far down the runway (9 m right
                // at 55 ft, 16 m left at the wheels), and the rollout's
                // heading law after it closed in nine seconds what the flare
                // has eight for (LEMG 13, 2026-09-26).
                let xtk_m_now = xtk_now * M_PER_NM;
                let arrest_rate_now = arrest_rate.update(
                    arrest_prev_xtk.map_or(0.0, |p: f64| (xtk_m_now - p) / tick_dt.max(0.05)),
                    tick_dt.max(0.05),
                );
                arrest_prev_xtk = Some(xtk_m_now);
                let track_now = hpath_r.as_ref().ok().copied();
                let side_now = side_velocity_m_s(track_now, rwy.course_deg, gs_kts)
                    .unwrap_or(arrest_rate_now);
                let phi_now = phi_r.as_ref().ok().copied().unwrap_or(0.0);
                // The bank the ailerons already written will make: on a
                // roll-rate plant the reading is a dead time late.
                let bank_tgt =
                    arrest_bank_target(xtk_m_now, side_now, wings.predicted(phi_now), radio_ft);
                let flare_ail = match (arrest_hands, phi_r.as_ref().ok().copied(), roll_yielded) {
                    (true, Some(phi), false) => Some(wings.step(phi, bank_tgt, tick_dt)),
                    _ => None,
                };
                *TRACKING_ERROR.lock().unwrap_or_else(|e| e.into_inner()) = Some(xtk_now * M_PER_NM);
                crate::blackbox::record("autoland", serde_json::json!({
                    "rwy": format!("{} {}", rwy.icao, rwy.id), "dist": g.dist_nm,
                    "alt": alt, "gp_alt": g.glidepath_alt_ft, "cmd_vs": g.target_vs_fpm,
                    "vs_dial": vs_dial, "vvi": vvi, "vvi_trend": vvi_trend, "arresting": g.arresting,
                    "cmd_hdg": g.target_heading_deg, "ias": ias, "radio": radio_ft,
                    // The wheels, so a first contact the rollout latch never
                    // saw (a bounce) is on the tape: the scorecard judges it.
                    "ground": on_ground > 0.5,
                    "phase": format!("{:?}", g.phase), "ga": ga, "crab": crab, "crab_raw": crab_raw,
                    "trim": trim, "xtk_m": xtk_now * M_PER_NM,
                    // "plugin": one frame's position; "split": an older
                    // plugin's separate reads (#157).
                    "pos": pos_source,
                    "flare_elev": flare_elev, "flare_ail": flare_ail, "hands": arrest_hands,
                    // The autopilot not yet confirmed and read off: the
                    // elevator above is the level hold's (`ApEngage`).
                    "ap_hold": es.hold,
                    // The model flare's own numbers: the sink it flew toward,
                    // the height that was taken at, and the sink the
                    // elevator did not explain.
                    "flare_target": model_out.map(|o| o.target_fpm),
                    "flare_ahead": model_out.map(|o| o.ahead_ft),
                    "flare_unexplained": model_out.map(|o| o.unexplained_fpm),
                    // The prediction's own ask before the flare's ratchet:
                    // `flare_elev` over it is the pull held.
                    "flare_ask": model_out.map(|o| o.ask),
                    // A hold-off's pull held from its first contact (#166).
                    "touch_hold": touch_hold.held(),
                    "rough_air": g.arresting.then(|| final_air.rough()),
                    "lever": last_lever,
                    "phi": phi_r.as_ref().ok().copied(),
                    "psi": psi_r.as_ref().ok().copied(), "bug_true": crabbed_true,
                    "short_final": short_final.is_some(),
                    // The arrest's lateral: the track it closes on, the
                    // sideways velocity and the bank it asked for.
                    "trk": track_now, "side_m_s": side_now,
                    "bank_tgt": arrest_hands.then_some(bank_tgt),
                    // Which leg of a reversal is flying: every pattern tick
                    // reports phase Intercept, so without this the outbound,
                    // the base and the inbound are one undifferentiated block
                    // in the log and an overshoot cannot be read back to the
                    // leg that flew it.
                    "leg": pattern.map(|(_, leg, _)| leg.word()),
                    // The hold's altitude (#182): planning, or refused.
                    "hold_ft": hold.map(|h| h.alt_ft),
                    "floor_ft": live_floor.map(|(f, _)| f),
                }));

                // Lateral: heading bug in MAGNETIC (guidance is TRUE — see
                // [`true_to_mag`] for the variation sign trap). On final and in
                // the flare, add the wind crab so the commanded heading flies a
                // runway-aligned TRACK; intercept vectors stay coarse.
                let mag_hdg = true_to_mag(crabbed_true, var);
                // Knob presses, not a raw write — registers on glass (G1000) too.
                // Slew-capped: each press is an HTTP round trip, and the LXGB go-
                // around's 178° flip stalled this loop ~5 s per tick (2026-07-14) —
                // no VS/throttle updates the whole time. 30°/tick keeps a full
                // reversal turning while the loop stays live.
                // The bug, the VS dial and BOTH mode checks in one burst. They
                // were four sequential round trips on the hot path, and at
                // 40-70 ms each that is a quarter of a second of tick time
                // spent waiting — on top of the sleep. Nothing here depends on
                // anything else here: the mode reads decide whether to re-engage
                // AFTER the fact, and a dial written while its mode is off is
                // exactly what the re-engage below is for.
                // The elevator rides in the SAME burst. Written every tick it
                // is wanted (the plugin channel is a 1 s heartbeat, so a
                // change-gate would drop it exactly where it sits still), and
                // simply not written otherwise — which hands the pitch back
                // inside a second with nothing to remember.
                // The ILS: its needles against our path, and the coupled
                // approach where the airframe allows it.
                let on_final = g.phase == ApproachPhase::Final && pattern.is_none() && !ga;
                let needles = if ils.is_some() {
                    // From the tick's one read; the airframe's own
                    // approach status rode beside the frame.
                    let (h, v, lv, gv, gs, stock_ap) = (
                        st.get(F::Hdef),
                        st.get(F::Vdef),
                        st.get(F::LocValid),
                        st.get(F::GsValid),
                        st.get(F::GsStatus),
                        st.get(F::ApprStatus),
                    );
                    let ap = appr_read
                        .take()
                        .unwrap_or_else(|| Err(anyhow::anyhow!("unread")));
                    // Both readings, while an airframe's own status is being
                    // learned (the 737's app_status, 2026-09-26).
                    LAST_APPR_READS
                        .lock()
                        .map(|mut l| *l = (ap.as_ref().ok().copied(), stock_ap.ok()))
                        .ok();
                    Some((
                        crate::ils::Needles {
                            hdef: h.ok(),
                            vdef: v.ok(),
                            loc_valid: lv.is_ok_and(|x| x > 0.5),
                            gs_valid: gv.is_ok_and(|x| x > 0.5),
                        },
                        ap.ok(),
                        gs.ok(),
                    ))
                } else {
                    None
                };
                let cpl = match needles {
                    Some((n, ap, gs)) => {
                        if let Some(line) = ils_xcheck.step(n, on_final, radio_ft, tick_dt) {
                            crate::blackbox::record(
                                "ils",
                                serde_json::json!({"event": "xcheck", "line": line, "hdef": n.hdef,
                                    "vdef": n.vdef, "xtk_m": xtk_now * M_PER_NM, "radio": radio_ft}),
                            );
                            announce(line, Priority::Caution);
                        }
                        let before = coupling;
                        let out = coupling.step(
                            crate::ils::CouplingIn {
                                allowed: coupled_ok,
                                on_final,
                                arresting: g.arresting,
                                go_around: ga,
                                dist_nm: g.dist_nm,
                                needles: n,
                                appr: ap,
                                gs,
                            },
                            &mut coupling_armed_s,
                            tick_dt,
                        );
                        if out.press_appr {
                            let _ = xp.run_command(&crate::ils::approach_command()).await;
                        }
                        if out.decouple {
                            let _ = xp.run_command(CMD_HDG_MODE).await;
                            let _ = xp.run_command(CMD_VS_MODE).await;
                            let _ = xp.set_float(VS_DIAL, vs_dial).await;
                        }
                        if coupling != before {
                            crate::blackbox::record(
                                "ils",
                                serde_json::json!({"event": "coupling",
                                    "appr_reads": *LAST_APPR_READS.lock().unwrap_or_else(|e| e.into_inner()),
                                    "from": format!("{before:?}"),
                                    "to": format!("{coupling:?}"), "hdef": n.hdef, "vdef": n.vdef,
                                    "radio": radio_ft, "dist": g.dist_nm}),
                            );
                            match coupling {
                                crate::ils::Coupling::Coupled => announce(
                                    "Localizer and glideslope captured. The autopilot has the ILS.".into(),
                                    Priority::Advisory,
                                ),
                                crate::ils::Coupling::Abandoned if out.decouple => announce(
                                    "Off the ILS coupling. I'm flying the approach myself.".into(),
                                    Priority::Caution,
                                ),
                                _ => {}
                            }
                        }
                        out
                    }
                    None => crate::ils::CouplingOut {
                        press_appr: false,
                        decouple: false,
                        own_lateral: true,
                        own_vertical: true,
                    },
                };
                // The knob: once per phase or pattern leg and corrected past a
                // deadband (`bug_knob`); the short final and the flare steer
                // it every tick, as before.
                let knob = bug_knob(
                    g.phase,
                    in_short_final || g.arresting,
                    pattern.map(|(_, l, _)| l),
                    missed,
                );
                let knob_set = match (cpl.own_lateral, knob) {
                    (false, _) => {
                        bug_hold.reset();
                        None
                    }
                    (true, Some((key, db))) => {
                        bug_hold.decide(key, mag_hdg, knob_t0.elapsed().as_secs_f64(), db)
                    }
                    (true, None) => {
                        bug_hold.reset();
                        Some(mag_hdg)
                    }
                };
                // The mode lights and the yields come from the tick's one
                // read; the VS dial, the column, the ailerons and a fine bug
                // correction are QUEUED and go out with the lever in the
                // tick's one write round below. A yield is read as it always
                // was, only on a tick that writes its channel; the latch now
                // comes from the tick's read, a frame before this tick's
                // write, and the plugin drops the override on its own frame
                // either way (the write that follows a yield is ignored).
                let hdg_st = st.get(F::HdgStatus);
                let vs_st = st.get(F::VsStatus);
                if cpl.own_vertical {
                    writes.set(VS_DIAL, vs_dial);
                }
                let yield_r = match flare_elev {
                    Some(e) => {
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_PITCH, e);
                        st.get(F::PitchYield)
                    }
                    None => Ok(0.0),
                };
                let roll_yield_r = match flare_ail {
                    Some(a) => {
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_ROLL, a);
                        st.get(F::RollYield)
                    }
                    None => Ok(0.0),
                };
                // A keyed setting (once per phase or leg) is driven and READ
                // BACK as before, beside the write round; the every-tick
                // steering of the short final and the flare is one queued
                // write from the bug the tick read (`bug_step_from`), pressed
                // only past three degrees.
                let bug_drive = match (knob, knob_set) {
                    (Some(_), Some(h)) => Some(h),
                    (None, Some(h)) => {
                        use crate::tools::autopilot::{bug_step_from, BugStep};
                        match bug_step_from(st.opt(F::Bug), h) {
                            BugStep::Leave => None,
                            BugStep::Set(v) => {
                                writes.set(HDG_BUG, v);
                                None
                            }
                            BugStep::Drive => Some(h),
                        }
                    }
                    (_, None) => None,
                };
                // The pilot pulled or pushed: the plugin has already dropped
                // the override this frame. Stop commanding and let the dial
                // fly what is left — the VS path is still running underneath.
                if yield_r.is_ok_and(|v| v > 0.5) {
                    if !pitch_yielded {
                        pitch_yielded = true;
                        crate::flight::note_pilot_takeover("autoland: yoke");
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "pitch_yield", "radio": radio_ft}),
                        );
                        announce(
                            "You have the pitch. I'm holding the wings and the power.".into(),
                            Priority::Caution,
                        );
                    }
                    flare_pitch.reset();
                    if let Some(m) = flare_model.as_mut() {
                        m.reset();
                    }
                }
                // Likewise the roll: the plugin dropped that override the
                // frame the hand moved, and the elevator keeps flying.
                if roll_yield_r.is_ok_and(|v| v > 0.5) && !roll_yielded {
                    roll_yielded = true;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "roll_yield", "radio": radio_ft}),
                    );
                    announce(
                        "You have the roll. I'm on the elevator and the power.".into(),
                        Priority::Caution,
                    );
                }
                // Mode housekeeping is the autopilot's while it flies. With
                // the arrest on the channels a mode press is at best nothing
                // and at worst (an MCP that engages the master with the
                // mode) the servo back on the yoke mid-flare.
                // A coupled ILS's captured modes are the autopilot's:
                // re-engaging HDG or VS would drop the localizer or the
                // glideslope it just captured.
                if !arrest_hands {
                    if cpl.own_lateral && hdg_st.unwrap_or(0.0) <= 0.5 {
                        let _ = xp.run_command(CMD_HDG_MODE).await;
                    }
                    // Vertical: VS mode to the glidepath rate.
                    if cpl.own_vertical && vs_st.unwrap_or(0.0) <= 0.5 {
                        let _ = xp.run_command(CMD_VS_MODE).await;
                        // The engage syncs the dial to the CURRENT rate, so the
                        // rate we want has to be written again after it.
                        let _ = xp.set_float(VS_DIAL, vs_dial).await;
                    }
                }

                // Speed: hold the approach speed with the SAME governor the copilot
                // uses for speed hold (speedhold::throttle_adjust) — a P+D law whose
                // derivative term adds power while the airspeed is still UNWINDING,
                // before the error grows, with extra authority on the slow side. The
                // old loop here was P-only (0.004/kt, no trend), so it only reacted
                // after speed was already gone and dribbled power in too slowly —
                // the nose came up holding level, speed bled to a stall, the AP quit
                // ("lose speed, lose AP", seen live at LERL/XLE000N). Profile-led flare power.
                let now = std::time::Instant::now();
                let dt_s = (now - prev_ias_t).as_secs_f64().clamp(0.1, 2.0);
                let trend_kts_s = prev_ias.map_or(0.0, |p| (ias - p) / dt_s);
                prev_ias = Some(ias);
                prev_ias_t = now;
                if g.phase != ApproachPhase::Flare || missed.is_some() {
                    flare_power.reset();
                }
                let mut ballooned = false;
                let thr = if missed == Some(MissedStage::Climb) {
                    // The balked landing itself is firewall, not a governor
                    // holding approach speed — the LXGB miss flew its last 20
                    // seconds at the governor's idea of "on speed" with the
                    // ground coming up. ONLY the climb: full power belongs to
                    // the seconds of getting away from the runway, not to the
                    // minutes of repositioning that follow. Left firewalled
                    // across the whole missed approach (the first flight of the
                    // sticky version, Luton 2026-07-28) the MD-82 accelerated to
                    // 284 KIAS on the outbound leg and was still gaining.
                    // ...and only until the climb is ESTABLISHED: then it
                    // flies the go-around speed (`missed_approach_throttle`).
                    // EGNX 09 (2026-09-30): firewalled from 150 ft to the
                    // turn at 800 ft radio, the clean 737 went 250 to 309 kt,
                    // and 355 before the lever came back (#199).
                    let limits = crate::aircraft::flap_limits();
                    let flap = last_flap.unwrap_or(0.0);
                    let target = missed_approach_speed_kts(
                        approach_ias,
                        flap,
                        &limits,
                        crate::aircraft::config_floor_kts(GO_AROUND_FLAP),
                        crate::aircraft::config_floor_kts(flap),
                        alt,
                    );
                    Some(match thr_now {
                        Some(t) => missed_approach_throttle(
                            t,
                            ias,
                            trend_kts_s,
                            target,
                            radio_ft,
                            vvi,
                            crate::aircraft::approach_thrust_min(),
                        ),
                        None => 1.0,
                    })
                } else if touch_hold.touched() {
                    // A hold-off held on (`TouchHold`): the lever closed,
                    // never the balloon's approach thrust. Each iMac SR22
                    // hop had 0.265 back on at 3 to 4 ft (LEMG 13
                    // 1790702352, 1790702623, 2026-09-29).
                    Some(0.0)
                } else if let Some(hold) = g
                    .phase
                    .eq(&ApproachPhase::Flare)
                    .then(|| {
                        balloon_power(
                            vvi,
                            approach_thr,
                            radio_ft > arrest_gates_ft(25.0, gs_kts).1,
                        )
                    })
                    .flatten()
                {
                    // A balloon at idle only bleeds the speed the settle
                    // needs: hold the approach's own thrust until it sinks.
                    ballooned = true;
                    Some(hold)
                } else if g.phase == ApproachPhase::Flare {
                    match cfg.flare_power {
                        Some(power) => match (thr_now, vvi) {
                            (Some(throttle), Some(vs)) => flare_power.step(
                                power,
                                throttle,
                                vs,
                                dt_s,
                                radio_ft,
                                Some(ias),
                                Some(approach_ias),
                            ),
                            _ => None, // No measurement: no guessed power and no ramp advance.
                        },
                        None => Some(0.0),
                    }
                } else {
                    // The target is only as safe as the flaps actually out
                    // (`config_floor_kts`), and the lever has a floor of its
                    // own on final and a ceiling-only rule low in a
                    // reposition (`approach_power_floor`).
                    let floor = crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0));
                    let target = approach_target_kts(
                        g.target_ias_kts,
                        last_flap.unwrap_or(0.0),
                        &crate::aircraft::flap_limits(),
                        floor,
                    );
                    let adj = crate::tools::speedhold::throttle_adjust(target - ias, trend_kts_s);
                    // In the roundout the lever only ever comes back. The
                    // speed law read the roundout's own deceleration as
                    // "five knots slow" and walked the 737's lever from 0.32
                    // to 0.55 at 30 ft over EGCC 23L; the flare then held
                    // that thrust (sink over `hold_sink_fpm`) and the
                    // aircraft climbed away from two feet at 155 kt
                    // (2026-09-19). The floor still stands underneath.
                    let adj = if g.arresting { adj.min(0.0) } else { adj };
                    // A roundout that FLOATS takes the thrust off, as the
                    // flare's float does: the arrest is ours, not yet the
                    // Flare, and the speed law holds the approach's lever
                    // into a level-off and a climb (`roundout_float_lever`).
                    let float_lever = match (arrest_hands && g.arresting, thr_now, vvi) {
                        (true, Some(throttle), Some(v)) => roundout_float_lever(
                            throttle,
                            v,
                            vvi_trend,
                            dt_s,
                            cfg.flare_power.map_or(DEFAULT_ROUNDOUT_RETARD_PER_S, |p| {
                                p.retard_max_per_s.unwrap_or(p.reduction_per_second)
                            }),
                        ),
                        _ => None,
                    };
                    let min_thr = approach_power_floor(
                        g.phase,
                        radio_ft,
                        target - ias,
                        crate::aircraft::approach_thrust_min(),
                        last_flap.is_some_and(|f| f >= LANDING_FLAP_RATIO),
                    )
                    .max(climb_power_floor(
                        g.phase,
                        g.target_vs_fpm,
                        crate::aircraft::approach_thrust_min(),
                    ));
                    float_lever.or_else(|| thr_now.map(|throttle| (throttle + adj).clamp(min_thr, 1.0)))
                };
                if g.phase != ApproachPhase::Flare && missed.is_none() {
                    if let Some(t) = thr {
                        approach_thr = Some(t);
                    }
                }
                if thr.is_some() {
                    last_lever = thr;
                }
                // The tick's ONE write round: the lever, the VS dial, the
                // column, the ailerons and the bug, together (they were up to
                // four rounds, 2026-09-30), a keyed bug drive beside it.
                if let Some(throttle) = thr {
                    writes.set(&thr_name, throttle);
                }
                let (done, knob_read) = tokio::join!(writes.flush(&xp), async {
                    match bug_drive {
                        Some(h) => {
                            Some(crate::tools::autopilot::drive_heading_bug_step(&xp, h, 30.0).await)
                        }
                        None => None,
                    }
                });
                if let (Some((key, _)), Some(h)) = (knob, knob_set) {
                    bug_hold.took(knob_read);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "heading_set", "key": key, "hdg": h,
                            "read": knob_read, "phase": format!("{:?}", g.phase),
                            "radio": radio_ft}),
                    );
                }
                let throttle_written = thr.map(|_| done.ok(&thr_name) == Some(true));
                // Whether the lever TOOK the last tick's write (an add-on
                // re-driving it is caught this way): this tick's read against
                // that command. It was a second round trip after the write
                // (`throttle_after`), skipped in the flare; the next tick's
                // read says the same thing a tick later, for no round.
                let lever_took = last_thr_written
                    .zip(thr_now)
                    .map(|(c, r)| (c - r).abs() < LEVER_TOOK_TOL);
                last_thr_written = if throttle_written == Some(true) { thr } else { None };
                if throttle_written == Some(false) {
                    flare_power.reset();
                }
                crate::blackbox::record("autoland", serde_json::json!({
                    "event": "power", "phase": format!("{:?}", g.phase),
                    "throttle_before": thr_now, "throttle_command": thr,
                    "lever_took": lever_took, "write_ok": throttle_written,
                    "progressive_flare": cfg.flare_power.is_some(), "dt": dt_s,
                    // Which key moved the flare's lever this tick (issue
                    // #76): entry, height, time, slow, float or timed; a
                    // climb's approach thrust is "balloon" (issue #146: the
                    // last key's word hid it).
                    "retard": (g.phase == ApproachPhase::Flare)
                        .then(|| if touch_hold.touched() {
                            Some("touched")
                        } else if ballooned {
                            Some("balloon")
                        } else {
                            flare_power.last_key.map(RetardKey::word)
                        })
                        .flatten(),
                    "vvi": vvi, "radio": radio_ft, "ias": ias,
                }));

                // Config: gear down inside 6 nm; flaps by distance AND vertical
                // state (full only once on the path — see [`flap_schedule`]).
                // Both wait for the speed the placard allows (the profile's
                // VLO and per-detent Vfe): a fast handoff slows first and
                // configures as the limits open up. Written only when the
                // target changes, so the setting isn't re-asserted against the
                // pilot every tick.
                if !gear_down
                    && !outbound
                    && g.dist_nm < 6.0
                    && gear_may_extend(ias, crate::aircraft::gear_limit_kts())
                {
                    let _ = xp.set_float(GEAR_HANDLE, 1.0).await;
                    gear_down = true;
                }
                // The landing's own checklist items, with the gear: ground
                // spoilers armed and the autobrake, where the airframe has
                // them. Asked again while the descent still has the
                // speedbrake out. (The Monitor list named both on every A330
                // flight; nothing had ever set them.)
                if !landing_config_done && g.dist_nm < 6.0 && !ga {
                    let cfg = crate::takeoff_config::set_for_landing(&xp).await;
                    landing_config_done = !cfg.deferred;
                    if landing_config_done {
                        LANDING_CONFIGURED.store(true, Ordering::SeqCst);
                    }
                    if let Some(line) = crate::takeoff_config::line("Landing config", &cfg) {
                        announce(line, Priority::Advisory);
                    }
                }
                // Go-around flap: half, whatever the schedule says — full flap
                // won't climb, and clean this low gives the sink away.
                // A downwind is flown at half flap too: the distance schedule
                // put full flap out abeam the numbers at 1,900 ft.
                // ...once the speed flies it (`go_around_flap_due`): the
                // half flap's floor, not the full flap's.
                let wanted = if ga
                    && last_flap.is_some_and(|f| f > 0.5 + 1e-6)
                    && !go_around_flap_due(ias, crate::aircraft::config_floor_kts(0.5))
                {
                    last_flap.unwrap_or(0.5)
                } else if ga || outbound {
                    0.5
                } else {
                    let sched = flap_schedule(g.dist_nm, alt - g.glidepath_alt_ft);
                    approach_flap_wanted(sched, last_flap)
                };
                let limits = crate::aircraft::flap_limits();
                let flap = flap_kept_flying(
                    flap_held_within_limits(wanted, ias, &limits, last_flap),
                    last_flap,
                    ias,
                    &limits,
                    ga,
                    retract_speed_kts,
                );
                if last_flap != Some(flap) {
                    last_flap = Some(flap);
                    let _ = xp.set_float(FLAP_RATIO, flap).await;
                }

                // Watchdog: the AP not FLYING the commanded vertical speed is how
                // the LEMI approach dove 2,300 fpm into terrain with vs_cmd=0 and
                // no callout. If actual VS diverges hard for ~5 s, re-arm the VS
                // mode and tell the pilot — this is exactly when a hand on the
                // yoke matters.
                let vs_err = vvi.map(|v| v - g.target_vs_fpm);
                // ...and never inside the arrest, where the autopilot has
                // no vertical to hold: a watchdog re-engaging the servos at
                // fifty feet would be the fight this change removes.
                // ...nor before the engage is confirmed: `ApEngage` owns the
                // press until then, and the aircraft is held level meanwhile.
                let not_held = g.phase != ApproachPhase::Flare
                    && !g.arresting
                    && engage.confirmed()
                    && vs_err.is_some_and(|e| vs_not_held(e, prev_vs_err, tick_dt));
                prev_vs_err = vs_err;
                if not_held {
                    let was_s = vs_diverged_s;
                    vs_diverged_s += tick_dt;
                    if was_s < VS_DIVERGED_WARN_S && vs_diverged_s >= VS_DIVERGED_WARN_S {
                        // The mode AND the master: an AP kicked off by the
                        // airframe's own protection (the A330's high-speed
                        // law at Vmo) answers to neither a dial nor a mode
                        // command until the servos are back on.
                        let _ = xp.run_command(CMD_SERVOS_ON).await;
                        let _ = xp.run_command(CMD_VS_MODE).await;
                        announce(vs_not_held_line(g.target_vs_fpm).into(), Priority::Warning);
                    }
                    // Said once and then flown for twenty-two minutes: the
                    // A330 climbed from 1,900 ft to FL350 on a -1,200 fpm
                    // dial with the loop still "on approach" and the aircraft
                    // 90 nm from the runway (EGPH 06, 2026-09-20). A warning
                    // that changes nothing is not a watchdog. Give up with a
                    // flyable aircraft and say what is held.
                    if vs_diverged_s >= VS_DIVERGED_GIVE_UP_S {
                        gave_up = true;
                        let thr_dref = throttle_dref();
                        let _ = tokio::join!(
                            xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                            xp.set_float(VS_DIAL, 0.0),
                        );
                        announce(
                            "Autopilot is still not holding the descent. Abandoning the approach. \
                             Your aircraft — the vertical speed dial is zero and the heading bug is set."
                                .into(),
                            Priority::Warning,
                        );
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "give_up", "reason": "vs_divergence",
                                               "diverged_s": vs_diverged_s, "vvi": vvi, "cmd_vs": g.target_vs_fpm}),
                        );
                        break;
                    }
                } else {
                    vs_diverged_s = 0.0;
                }
                // Flying AWAY from the runway is not an approach. The same
                // sortie: an arm 3.6 nm from the threshold, then 90 nm out and
                // opening, phase still Intercept. A missed approach flies
                // outbound by design; anything else past the arm distance plus
                // a margin is handed back before it becomes a diversion.
                // Measured from as far out as this approach has legitimately
                // been, never while a pattern is flying its downwind: armed
                // over the runway in a go-around (0.9 nm) the limit was 5.9 nm,
                // a 737's turn back needs 6.5, and the guard fired on the tick
                // the turn inbound began (EGTE, 2026-09-21).
                if missed.is_none()
                    && reversal.is_none()
                    && hold.is_none()
                    && g.dist_nm > recede_ref_nm + RECEDING_GIVE_UP_NM
                {
                    gave_up = true;
                    let thr_dref = throttle_dref();
                    let _ = tokio::join!(
                        xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                        xp.set_float(VS_DIAL, 0.0),
                    );
                    announce(
                        format!(
                            "We're {:.0} miles from runway {} and opening. Abandoning the approach. \
                             Your aircraft — the vertical speed dial is zero.",
                            g.dist_nm,
                            crate::circuit::spell_runway(&rwy.id)
                        ),
                        Priority::Warning,
                    );
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "receding",
                                           "dist_nm": g.dist_nm, "armed_at_nm": rwy.dist_nm,
                                           "ref_nm": recede_ref_nm}),
                    );
                    break;
                }

                if last_phase != Some(g.phase) {
                    let prev = last_phase;
                    last_phase = Some(g.phase);
                    last_arresting = g.arresting;
                    // The landing light for the landing, at the final: the
                    // exterior-lights rule lights it lined up and puts it out
                    // at the vacate, and nothing lit it for the ARRIVAL. The
                    // BEFORE LANDING cross-check read it off on the LEMG 31
                    // final (2026-09-12).
                    if g.phase == ApproachPhase::Final {
                        let _ = crate::tools::lights::landing(&xp, true).await;
                    }
                    let line = match g.phase {
                        // Dropping back to Intercept from Final/Flare is a
                        // go-around, not a fresh intercept — say so. Low it's
                        // the latch flying (straight ahead); high it's the
                        // normal turn back to re-intercept.
                        ApproachPhase::Intercept
                            if matches!(prev, Some(ApproachPhase::Final | ApproachPhase::Flare)) =>
                        {
                            if ga {
                                // Said at once: the power is coming and the
                                // pilot must know why. Corrected if the
                                // wheels take it (`balk_landed`).
                                spoke_going_around = true;
                                "Going around. Full power, climbing straight ahead."
                            } else {
                                dropped_final_line(
                                    along_nm,
                                    reversal_began,
                                    alt - g.glidepath_alt_ft,
                                )
                                .unwrap_or("")
                            }
                        }
                        ApproachPhase::Intercept => "Intercepting the final approach.",
                        ApproachPhase::Final => "On final. Gear and flaps as the speed allows.",
                        ApproachPhase::Flare if cfg.flare_power.is_some() => "Over the threshold, flaring, easing the power back.",
                        ApproachPhase::Flare => "Over the threshold, flaring, power to idle.",
                    };
                    // A go-around the stabilised gate ordered says why.
                    let line = match stable_said.take() {
                        Some(why) if ga && line.starts_with("Going around") => {
                            format!("Not stabilised, {why}. Going around.")
                        }
                        _ => line.to_string(),
                    };
                    if !line.is_empty() {
                        announce(line, Priority::Caution);
                    }
                }

                // (Touchdown + rollout-to-stop is handled at the top of the loop,
                // latched by `rolling_out`, before the approach guidance.)
                pacer.wait(std::time::Duration::from_millis(tick_ms(last_phase, last_arresting))).await;
            }
            // A newer loop (the clearance re-arm above, or any later arm) has
            // already set ACTIVE for itself; clearing it here would report
            // "landed or gone around" to whoever waits on it.
            // "YOUR AIRCRAFT" WITH A SPEED HELD. Every give-up wrote a flat
            // 0.75 lever for the pilot to find: right for a piston dragging
            // gear and flap, and on a level 737 it is an acceleration with no
            // end (152 kt at the hand-back, 240 forty seconds later, 346 and
            // ten overspeed warnings before anyone had the picture, EGTE
            // 2026-09-21). The lever write stays as the first second's power;
            // the governor then holds a speed the configuration allows, and
            // "my controls" or a spoken speed takes it from there.
            if gave_up && GEN.load(Ordering::SeqCst) == my_gen && !xp.cancelled() {
                let hold = hand_back_speed_kts(
                    approach_ias,
                    last_flap.unwrap_or(0.0),
                    &crate::aircraft::flap_limits(),
                );
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "hand_back", "speed_kts": hold, "flap": last_flap}),
                );
                announce(format!("I'm holding {hold:.0} knots for you."), Priority::Advisory);
                crate::tools::speedhold::arm_flight(base_url.clone(), voice.clone(), hold);
            }
            if GEN.load(Ordering::SeqCst) == my_gen {
                set_panel("");
                ACTIVE.store(false, Ordering::SeqCst);
                LANDING_CONFIGURED.store(false, Ordering::SeqCst);
            }
        });
    });
}

/// The runway END a clearance heard DURING the approach moves us to, if any:
/// an arrival-shaped clearance (told to join, or cleared to land) naming a
/// different end than the one being flown, not already looked up. A
/// departure state's runway belongs to the departure; a blank or matching
/// ident changes nothing. The caller still checks the end is at THIS field.
///
/// Never during a missed approach: the go-around itself moves the tower
/// from `ClearedToLand` to `Inbound`, and the ident published with that state
/// is the runway IN USE, which on a wind dead across the strip had just
/// flipped. Five seconds into the EGTE go-around, at 60 ft, "Tower cleared us
/// for runway two six" replaced the climb-away with a fresh approach to the
/// other end; nobody had cleared anything (2026-09-21). A missed approach is
/// flown to its end, and the tower's answer to the next final report is where
/// a new end comes from.
///
/// ...unless the tower's go-around ORDER itself named the end
/// (`expected`, "expect runway one three", `GoAroundWatch::expected_runway`),
/// and the published clearance agrees: that is a re-sequence, taken once the
/// climb away is flown (the caller passes `expected` only then). LEMG,
/// 2026-09-29 (sortie 1790668215-full_auto): sent round off 30 and
/// re-sequenced to 13 at t+649, the reposition flew a right-hand pattern for
/// 30 from t+661 and learnt of 13 only at t+984, 5.5 minutes later. Pure.
pub fn clearance_switch(
    flying: &str,
    state: Option<crate::tower::State>,
    cleared: Option<&str>,
    already_checked: Option<&str>,
    going_around: bool,
    expected: Option<&str>,
) -> Option<String> {
    use crate::tower::State::{ClearedToLand, Inbound};
    if !matches!(state, Some(ClearedToLand | Inbound)) {
        return None;
    }
    let c = cleared?.trim();
    if going_around && !expected.is_some_and(|e| e.trim().eq_ignore_ascii_case(c)) {
        return None;
    }
    if !clearance_differs(flying, c) {
        return None;
    }
    if already_checked.is_some_and(|a| a.trim().eq_ignore_ascii_case(c)) {
        return None;
    }
    Some(c.to_string())
}

/// Why the runway autoland arms on is (or is not) a change of the mission's
/// arrival runway, for `full_auto::retarget_arrival_runway`: the tower's
/// clearance moved it at the arm ("clearance"), the forced end was flown as
/// handed down ("arm", no change), or nothing forced it or the forced end
/// was missing and the track picked ("self_pick"). Pure.
pub fn armed_runway_why(forced: Option<&str>, flying: &str, from_clearance: bool) -> &'static str {
    if from_clearance {
        "clearance"
    } else if forced.is_some_and(|f| f.trim().eq_ignore_ascii_case(flying.trim())) {
        "arm"
    } else {
        "self_pick"
    }
}

/// Is an owed readback the LANDING one? The tower phrases what it waits for
/// (`Tower::suggested_readback`); a hold or line-up echo is never an
/// approach's to make.
pub fn landing_readback(owed: &str) -> bool {
    let o = owed.trim_start().to_ascii_lowercase();
    o.starts_with("cleared to land") || o.starts_with("cleared touch and go")
}

/// Does the standing clearance name a different runway END than the plan?
/// Idents compare trimmed and case-blind ("12" vs "12 "), a missing or blank
/// clearance never differs.
pub fn clearance_differs(planned_id: &str, cleared_id: &str) -> bool {
    let c = cleared_id.trim();
    !c.is_empty() && !c.eq_ignore_ascii_case(planned_id.trim())
}

/// Time constant of the wind-crab estimate the final's lateral law feeds
/// forward. A wind-correction angle changes over minutes (wind shear down the
/// last thousand feet); the RAW heading-minus-track read changes every tick.
/// LEMG 13, 2026-09-12: the raw value was added straight to the heading bug
/// on final, so every nose swing the autopilot made came back as a bug move
/// the same tick, unity gain, and the loop hunted. The blackbox has the
/// signature: crab standard deviation 0.5° on the intercept (where nothing
/// fed it back), then 1.2, 1.8, 2.5, 3.1, 3.7, 4.0° down the final, a ±9°
/// swing with a ~13 s period at the numbers, a standing 35 m offset the
/// short-final law could not close, and the lateral balk gate at 74 ft.
/// At 15 s the 13 s swing comes through at about a seventh of its amplitude.
pub const CRAB_TAU_S: f64 = 15.0;

/// The wind crab actually commanded: heading minus ground track, low-passed
/// with [`CRAB_TAU_S`]. Fed EVERY tick (intercept included) so it is settled
/// when the final starts. A missing read holds the last estimate rather than
/// yanking the nose by the whole angle; a filter that has never seen a read
/// says zero.
#[derive(Debug, Default, Clone, Copy)]
pub struct CrabFilter {
    est: Option<f64>,
}

impl CrabFilter {
    pub const fn new() -> Self {
        Self { est: None }
    }

    /// Raw heading-minus-track (degrees, positive = nose right of track),
    /// `None` when either read is missing. Clamped to ±15°: beyond that it
    /// is a turn or a slip, not a wind angle.
    pub fn raw(psi_true: Option<f64>, hpath_true: Option<f64>) -> Option<f64> {
        let (psi, hpath) = (psi_true?, hpath_true?);
        Some((((psi - hpath + 540.0).rem_euclid(360.0)) - 180.0).clamp(-15.0, 15.0))
    }

    /// Advance by one tick of `dt_s` seconds and return the crab to command.
    pub fn update(&mut self, raw: Option<f64>, dt_s: f64) -> f64 {
        match (raw, self.est) {
            (Some(r), None) => {
                self.est = Some(r);
                r
            }
            (Some(r), Some(e)) => {
                let a = (dt_s / CRAB_TAU_S).clamp(0.0, 1.0);
                let e = e + a * (r - e);
                self.est = Some(e);
                e
            }
            (None, e) => e.unwrap_or(0.0),
        }
    }

    /// The current estimate without advancing.
    pub fn value(&self) -> f64 {
        self.est.unwrap_or(0.0)
    }
}

/// Time constant (s) of the final's track-error trim: how long a steady
/// difference between the track the final asks for and the track flown takes
/// to be integrated out. Slow against the autopilot's heading response (a few
/// seconds) and the crab filter (15 s), so the three never chase each other.
pub const TRACK_TRIM_TAU_S: f64 = 30.0;
/// The most trim the integrator may carry, degrees. A bias bigger than this
/// is a broken autopilot, not something to trim.
pub const TRACK_TRIM_MAX_DEG: f64 = 5.0;
/// A track error beyond this is a turn in progress, not a bias: not integrated.
/// It was 10, and a handoff still converging is a turn in progress: LEMG 31
/// (sortie 1790584715-full_auto, 2026-09-28) the 737 came off the circuit's
/// intercept tracking 303 to 309 for a 311.7 runway, the trim learnt +0.78
/// degrees from those six seconds of roll-out, and with the autopilot
/// flying its bug to a tenth of a degree the final then sat 30 to 32 m right
/// of the centreline from 1.4 nm to 0.3, the trim cancelling the pursuit's
/// 0.75 degree cut; LEMG 13 (1790591256) learnt -0.57 the same way and sat
/// 50 m left. The biases it exists for were 1.1 to 1.9 degrees.
const TRACK_TRIM_ERR_CAP_DEG: f64 = 3.0;
/// A bias is learnt only from a track that has been STEADY this long, s:
/// turning under [`TRACK_TRIM_STEADY_DPS`] (smoothed over
/// [`TRACK_TRIM_RATE_TAU_S`]) for [`TRACK_TRIM_SETTLE_S`]. The 3 degree cap
/// above still let the last three degrees of every circuit's roll-out in:
/// LEMG 30 A330 (sortie 1790710769-full_auto, 2026-09-29) was handed over
/// tracking 3.6 degrees off, flew straight on for a second and a half, then
/// rolled out at 0.2 to 0.65 degrees a second; the trim learnt -0.51 in ten
/// seconds with no bias in the air (calm, the aircraft on its bug), and the
/// final that had reached 4.4 m sat 0.3 to 0.5 degrees left of the course,
/// 17 m off at 0.3 nm, and went around at 99 ft (`off_centreline_low`). The
/// other three circuits flown that evening learnt 0.37 to 0.61 the same way
/// and drifted 6 to 14 m. Settled finals turn under 0.1 degrees a second
/// (p90 0.02 to 0.09 over the four); the biases this exists for held for
/// whole finals. 0.15 and 5 s passed the replays too, and left the replayed
/// crosswind finals (`the_final_on_a_held_knob_keeps_the_line`) 0.4 m
/// further off at the gate.
pub const TRACK_TRIM_STEADY_DPS: f64 = 0.25;
/// Time constant of the smoothing on that track rate, s: a gust's single
/// tick is not a turn, eight seconds of roll-out are.
const TRACK_TRIM_RATE_TAU_S: f64 = 1.5;
/// ...and how long the track must have been steady, s: the handoff's own
/// straight second or two before autoland's first bank is not a bias.
pub const TRACK_TRIM_SETTLE_S: f64 = 3.0;

/// Integral trim on the final's TRACK error, in degrees added to the heading
/// bug alongside the wind crab.
///
/// The crab loop commands a heading whose track aligns with the runway by
/// measuring heading minus track — the wind. It cannot see a bias that sits
/// between the BUG and the heading: an autopilot that settles two degrees
/// left of its bug produces a track two degrees left of the one asked for,
/// and both proportional laws on final (the fixed-lead pursuit, then the
/// 80°/nm short-final cut) only ever ask for a cut, so they equilibrate
/// where the cut equals the bias — 0.6 nm of lead times tan 2° is 39 m,
/// 2°/80 is 46 m. LEMG 13, 2026-09-12: the 172's heading sat 1.1° left of
/// the bug through the whole intercept with no crab in the loop and 1.9°
/// left down both finals, the position 33 to 37 m left of the centreline
/// from 1.3 km out, steady, in both approaches, and the lateral gate under
/// 100 ft (37 m) balked both at 74 and 40 ft. Two go-arounds; the pilot
/// landed by hand. This integrates the difference between the track the
/// guidance wants and the track flown, so whatever the cause of the bias
/// (autopilot law, servo trim, a variation error), the final converges on
/// the centreline instead of parking beside it. Final only; held through
/// the flare; reset on anything else so a missed approach never carries a
/// stale correction into the next attempt.
#[derive(Debug, Default, Clone, Copy)]
pub struct TrackTrim {
    deg: f64,
    /// The last track read on final, the smoothed rate it turns at (`None`
    /// until two reads have measured one) and how long it has been steady.
    last_trk: Option<f64>,
    rate_dps: Option<f64>,
    steady_s: f64,
}

impl TrackTrim {
    pub const fn new() -> Self {
        Self { deg: 0.0, last_trk: None, rate_dps: None, steady_s: 0.0 }
    }

    /// Has the track been steady (under `steady_dps`) long enough to learn a
    /// bias from? Measures the rate from the read before; the first read
    /// measures nothing.
    fn settled(&mut self, h: f64, dt_s: f64, steady_dps: f64) -> bool {
        let prev = self.last_trk.replace(h);
        let Some(p) = prev.filter(|_| dt_s > 1e-3) else {
            self.steady_s = 0.0;
            return false;
        };
        let raw = ((h - p + 540.0).rem_euclid(360.0) - 180.0) / dt_s;
        let r = match self.rate_dps {
            Some(r) => r + (dt_s / TRACK_TRIM_RATE_TAU_S).clamp(0.0, 1.0) * (raw - r),
            None => raw,
        };
        self.rate_dps = Some(r);
        if r.abs() <= steady_dps {
            self.steady_s += dt_s;
        } else {
            self.steady_s = 0.0;
        }
        self.steady_s >= TRACK_TRIM_SETTLE_S
    }

    /// Advance by one tick: `wanted_track_deg` is the guidance heading on
    /// final (the runway course plus the cut, i.e. the TRACK it wants),
    /// `hpath_true` the sim's ground track. Returns the trim to add.
    pub fn update(
        &mut self,
        phase: ApproachPhase,
        wanted_track_deg: f64,
        hpath_true: Option<f64>,
        dt_s: f64,
    ) -> f64 {
        self.update_capped(
            phase,
            wanted_track_deg,
            hpath_true,
            dt_s,
            TRACK_TRIM_ERR_CAP_DEG,
        )
    }

    /// [`TrackTrim::update`] with the turn-in-progress cap given (the replay
    /// tests fly the old 10 degrees against the new).
    pub(crate) fn update_capped(
        &mut self,
        phase: ApproachPhase,
        wanted_track_deg: f64,
        hpath_true: Option<f64>,
        dt_s: f64,
        cap_deg: f64,
    ) -> f64 {
        self.update_gated(
            phase,
            wanted_track_deg,
            hpath_true,
            dt_s,
            cap_deg,
            Some(TRACK_TRIM_STEADY_DPS),
        )
    }

    /// ...and the steady-track gate given (`None`: the old rule, the cap
    /// alone, for the replays of what flew).
    pub(crate) fn update_gated(
        &mut self,
        phase: ApproachPhase,
        wanted_track_deg: f64,
        hpath_true: Option<f64>,
        dt_s: f64,
        cap_deg: f64,
        steady_dps: Option<f64>,
    ) -> f64 {
        match phase {
            ApproachPhase::Final => {
                if let Some(h) = hpath_true {
                    let err = (wanted_track_deg - h + 540.0).rem_euclid(360.0) - 180.0;
                    let steady = match steady_dps {
                        Some(r) => self.settled(h, dt_s, r),
                        None => true,
                    };
                    if err.abs() <= cap_deg && steady {
                        self.deg = (self.deg + err * dt_s / TRACK_TRIM_TAU_S)
                            .clamp(-TRACK_TRIM_MAX_DEG, TRACK_TRIM_MAX_DEG);
                    }
                }
            }
            ApproachPhase::Flare => {}
            ApproachPhase::Intercept => *self = Self::new(),
        }
        self.deg
    }

    /// The trim currently applied.
    pub fn value(&self) -> f64 {
        self.deg
    }
}

/// Feet of altitude per nm along a 3° glidepath (tan 3° × 6076 ft/nm ≈ 318).
/// Public because the circuit descends its base leg onto the same path — a
/// pattern that hands over 500 ft below it has already lost the approach.
pub const GLIDEPATH_FT_PER_NM: f64 = 318.0;

/// This approach's glidepath angle, degrees: 3 unless an approach handed a
/// steeper one down for its visual segment (`set_next_glidepath_deg`; LSZS
/// RNAV 03's MAP is 996 ft above the threshold two miles out, 4.7 degrees).
/// Set at every arm, so no approach inherits another's.
/// In a test build both live per THREAD: the LSZS visual-segment test arms a
/// 4.5 degree path, and every other test landing at the same moment flew it
/// (the balloon test failed one run in three, 2026-09-24). One landing per
/// process is the app's truth; one per test is the tests'.
/// The angle carries WHO handed it down: `true` for a published visual
/// segment (the follower at a MAP, with its missed approach stashed), `false`
/// for a circuit's path steepened over the ground (#182). Only the first is
/// a valley our own pattern must never fly into; the second is an ordinary
/// field whose go-around the terrain-probed reversal flies (LEMG 13 SR22
/// circuit 2026-09-29: a 3.26 degree circuit path was taken for LSZS, the
/// go-around found no published missed approach and said "Your aircraft"
/// at 805 ft to nobody).
#[cfg(not(test))]
static GLIDEPATH_DEG: std::sync::Mutex<(f64, bool)> = std::sync::Mutex::new((3.0, false));
/// The stabilised gate's blackbox record (`stable_gate` at the crossing,
/// `unstabilised` when it orders a go-around): every number the rule read.
fn stable_record(
    event: &str,
    gate: crate::stable_approach::GateKind,
    s: &crate::stable_approach::Sample,
    now: &[crate::stable_approach::Breach],
    tripped: Option<crate::stable_approach::Breach>,
) -> serde_json::Value {
    use crate::stable_approach as sa;
    serde_json::json!({
        "event": event, "gate": gate.word(), "gate_ft": gate.gate_ft(),
        "height_aal": s.height_aal_ft, "dist_aim_nm": s.dist_to_aim_nm,
        "ias": s.ias_kts, "target": s.target_kts, "gs": s.gs_kts,
        "vvi": s.vvi_fpm,
        "sink_limit": sa::sink_limit_fpm(s.target_kts, s.ias_kts, s.gs_kts, s.path_ft_per_nm),
        "above_path_ft": s.above_path_ft,
        "path_tol_ft": sa::path_tolerance_ft(s.dist_to_aim_nm),
        "gear": s.gear_down, "landing_flap": s.landing_flap,
        "breaches": now.iter().map(|b| b.word()).collect::<Vec<_>>(),
        "why": tripped.map(|b| b.word()),
        "verdict": if now.is_empty() { "stable" } else { "unstable" },
    })
}

/// The stabilised gate the NEXT arm judges its final at, consumed there:
/// the circuit hands down a visual one, every other caller flies the
/// instrument gate (`stable_approach`, #199).
static NEXT_GATE: std::sync::Mutex<Option<crate::stable_approach::GateKind>> =
    std::sync::Mutex::new(None);

/// Hand the next autoland arm a visual gate (a circuit's final).
pub fn set_next_gate(kind: crate::stable_approach::GateKind) {
    *NEXT_GATE.lock().unwrap_or_else(|e| e.into_inner()) = Some(kind);
}

/// Take the handed-down gate for this arm, the instrument gate without one.
fn arm_gate() -> crate::stable_approach::GateKind {
    NEXT_GATE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .take()
        .unwrap_or_default()
}

/// The angle the NEXT arm takes, consumed there.
#[cfg(not(test))]
static NEXT_GLIDEPATH_DEG: std::sync::Mutex<Option<(f64, bool)>> = std::sync::Mutex::new(None);
#[cfg(test)]
thread_local! {
    static GLIDEPATH_DEG_T: std::cell::Cell<(f64, bool)> = const { std::cell::Cell::new((3.0, false)) };
    static NEXT_GLIDEPATH_DEG_T: std::cell::Cell<Option<(f64, bool)>> = const { std::cell::Cell::new(None) };
}
/// The steepest path the landing will fly.
pub const MAX_GLIDEPATH_DEG: f64 = 5.5;

#[cfg(not(test))]
fn next_glidepath(set: Option<Option<(f64, bool)>>) -> Option<(f64, bool)> {
    let mut g = NEXT_GLIDEPATH_DEG.lock().unwrap_or_else(|e| e.into_inner());
    match set {
        Some(v) => {
            *g = v;
            v
        }
        None => g.take(),
    }
}
#[cfg(test)]
fn next_glidepath(set: Option<Option<(f64, bool)>>) -> Option<(f64, bool)> {
    NEXT_GLIDEPATH_DEG_T.with(|c| match set {
        Some(v) => {
            c.set(v);
            v
        }
        None => c.take(),
    })
}
#[cfg(not(test))]
fn live_glidepath(set: Option<(f64, bool)>) -> (f64, bool) {
    let mut g = GLIDEPATH_DEG.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(v) = set {
        *g = v;
    }
    *g
}
#[cfg(test)]
fn live_glidepath(set: Option<(f64, bool)>) -> (f64, bool) {
    GLIDEPATH_DEG_T.with(|c| {
        if let Some(v) = set {
            c.set(v);
        }
        c.get()
    })
}

/// Hand the next autoland arm a glidepath angle for a PUBLISHED visual
/// segment (the follower does, at a MAP).
pub fn set_next_glidepath_deg(deg: Option<f64>) {
    next_glidepath(Some(deg.map(|d| (d, true))));
}

/// Hand the next autoland arm a circuit's path steepened over the ground
/// (#182): flown at its angle, but not a valley ([`published_steep`]).
pub fn set_next_circuit_glidepath_deg(deg: Option<f64>) {
    next_glidepath(Some(deg.map(|d| (d, false))));
}

/// Take the handed-down angle for this arm (3 without one), clamped.
fn arm_glidepath() -> f64 {
    let (deg, published) = next_glidepath(None)
        .filter(|(d, _)| d.is_finite())
        .map_or((3.0, false), |(d, p)| (d.clamp(3.0, MAX_GLIDEPATH_DEG), p));
    live_glidepath(Some((deg, published && deg > 3.0 + 1e-9))).0
}

fn glidepath_deg() -> f64 {
    live_glidepath(None).0
}

/// Is this approach a PUBLISHED steep visual segment (LSZS), the one kind no
/// pattern of our own may reposition? A circuit's steepened path is not.
fn published_steep() -> bool {
    let (deg, published) = live_glidepath(None);
    published && deg > 3.0 + 1e-9
}

/// Feet of path per nautical mile for this approach (318 at 3 degrees, the
/// constant every law was tuned on, returned exactly).
pub fn gp_ft_per_nm() -> f64 {
    let deg = glidepath_deg();
    if (deg - 3.0).abs() < 1e-9 {
        GLIDEPATH_FT_PER_NM
    } else {
        6076.12 * deg.to_radians().tan()
    }
}

/// tan of this approach's path angle (the 3 degree constant returned exactly).
fn gp_tan() -> f64 {
    let deg = glidepath_deg();
    if (deg - 3.0).abs() < 1e-9 {
        GLIDEPATH_TAN
    } else {
        deg.to_radians().tan()
    }
}

/// The glidepath correction, fpm per foot of path error, sized to close the
/// error over a fixed DISTANCE of final rather than a fixed time. 1.2 fpm/ft
/// is a 50 s time constant: 0.9 nm on a 172 at 65 kt, but 2.4 nm on a 737 at
/// 170 kt, whose last mile is 22 s long. Captured from below at 1.6 nm, the
/// 737 levelled, started down late, and arrived over LFLL 35L's numbers 51 ft
/// high with a correction of 86 fpm to fix it (2026-09-17). Scaled by
/// groundspeed over the 172's, floored at the old gain so every slow airframe
/// flies the law it always did, capped so the gain never outruns the VS
/// mode's own lag.
pub fn glidepath_gain_fpm_per_ft(gs_kts: f64) -> f64 {
    GLIDEPATH_GAIN_FPM_PER_FT
        * (gs_kts / GLIDEPATH_GAIN_REF_KTS).clamp(1.0, GLIDEPATH_GAIN_MAX_MULT)
}
const GLIDEPATH_GAIN_FPM_PER_FT: f64 = 1.2;
const GLIDEPATH_GAIN_REF_KTS: f64 = 65.0;
const GLIDEPATH_GAIN_MAX_MULT: f64 = 3.0;
/// Aim the glidepath at a touchdown point ~300 m (≈1,000 ft) PAST the threshold,
/// so the path crosses the numbers at a standard ~50 ft threshold-crossing
/// height and touches in the touchdown zone — not planted on the leading edge.
/// The aim point was the threshold itself, so the 3° path targeted the runway
/// EDGE at zero height (2026-07-24). Lateral tracking still anchors on the
/// threshold — it's the same centreline, just a farther aim along it — and the
/// flare still triggers off actual radio height, so it simply engages a little
/// farther in, over the touchdown zone.
pub const TOUCHDOWN_AIM_NM: f64 = 300.0 / M_PER_NM;
/// Final approach point: intercept the extended centreline this far out (nm).
const FAP_NM: f64 = 5.0;
/// Above the path by this much the guidance still flies the straight-in and
/// dives onto the path; a join point behind was the only answer past it.
const CAPTURE_ABOVE_FT: f64 = 500.0;
/// The share of the path law's catch-up rate a straight-in is planned on.
const STRAIGHT_IN_CATCH_UP_SHARE: f64 = 0.8;

/// Can the height above the path come off on the way in, straight in? The
/// catch-up the path law flies over its path's own sink
/// ([`path_descent_cap_fpm`]) for the minutes to the stabilised gate, on a
/// final the pursuit can intercept from here (`circuit::interceptable`).
/// LEMG 12 (sortie 1790583663-approach, #131): re-armed from 13's final
/// 3.5 km beside 12's, the aircraft was 513 ft over the path at 7.6 nm; the
/// 500 ft rule put the join point 10.8 nm out behind it, and a ten minute
/// reversal began where a steeper descent onto an intercept was a minute
/// and a half. Pure.
pub fn straight_in_losable(above_path_ft: f64, along_nm: f64, xtk_nm: f64, gs_kts: f64) -> bool {
    let gate_nm = STABILISED_GATE_AGL_FT / gp_ft_per_nm() - TOUCHDOWN_AIM_NM;
    let run_nm = along_nm - gate_nm;
    if run_nm <= 0.0 || !crate::circuit::interceptable(xtk_nm * M_PER_NM, along_nm * M_PER_NM) {
        return false;
    }
    let gs = gs_kts.max(60.0);
    let nominal = -(gs / 60.0) * gp_ft_per_nm();
    let catch_up_fpm = nominal - path_descent_cap_fpm(nominal);
    let minutes = run_nm / gs * 60.0;
    above_path_ft <= CAPTURE_ABOVE_FT + catch_up_fpm * minutes * STRAIGHT_IN_CATCH_UP_SHARE
}

/// Does the guidance aim at the join point (out along the centreline, behind
/// an aircraft inside it) rather than fly the straight-in? Only when the
/// height above the path cannot come off on the way in
/// ([`straight_in_losable`]). The reversal's own aim asks the same, so a
/// pattern is flown only when a re-intercept is not flyable. Pure.
pub fn aims_at_join_point(above_path_ft: f64, along_nm: f64, xtk_nm: f64, gs_kts: f64) -> bool {
    above_path_ft > CAPTURE_ABOVE_FT
        && !straight_in_losable(above_path_ft, along_nm, xtk_nm, gs_kts)
}
/// Can a final still be made from `xtk_nm` off the centreline with `along_nm`
/// to run — will the pursuit's steepest cut close the offset, with margin,
/// before the threshold? The circuit asks exactly this before it hands over
/// (`circuit::interceptable`); below the terrain floor autoland asks it of
/// itself. A 370 m offset at 0.8 nm passes (the pursuit flies that out); the
/// Gibraltar handoff's 1,660 m at 1.5 nm does not. See the balk in [`guidance`].
/// The gate applies on the approach side with this much still to run: closer
/// in, the flare gate and the pursuit's own short-final law own the question
/// (and past the threshold the along-track is negative, which the intercept
/// sum would read as "can't", over a landing that is already happening).
const STABILISED_MIN_ALONG_NM: f64 = 0.3;
/// Below this on final the approach is stabilised laterally or it is a
/// go-around (see the gate in [`guidance`]).
pub const STABILISED_GATE_AGL_FT: f64 = 1000.0;

pub fn stabilised_xtk(xtk_nm: f64, along_nm: f64) -> bool {
    crate::circuit::interceptable(xtk_nm * M_PER_NM, along_nm * M_PER_NM)
}

/// Will the final's pursuit have the offset inside `limit_m` by the
/// threshold ([`crate::reversal::predicted_residual_m`], the stabilised
/// gate's own time model)? The question below the terrain floor, where
/// "interceptable" is true of any offset a landing could still be saved from
/// and of many it could not.
pub fn lands_on_the_pavement(xtk_nm: f64, along_nm: f64, gs_kts: f64, limit_m: f64) -> bool {
    crate::reversal::predicted_residual_m(xtk_nm, along_nm, gs_kts) <= limit_m
}
/// Airspeed this far below the approach speed triggers speed protection (nose
/// down to regain it), before the decay reaches a stall and disconnects the AP.
/// Approach speed is already ~1.3×Vs, so this margin still stays well above stall.
const SPEED_FLOOR_MARGIN_KTS: f64 = 7.0;

/// Clean manoeuvring speed for the intercept, as a multiple of the airframe's
/// approach speed. The two scale together across every airframe we fly, which
/// is why this is a RATIO and not another profile field to thread through five
/// callers: the SF50's 87 kt approach gives 126 (it was flying 125 when the
/// circuit handed off), the 172's 65 gives 94, and the MD-82's 140 gives 203 —
/// within a few knots of the 200 its profile already declares as the pattern
/// speed. A profile figure can always override this later.
const CLEAN_APPROACH_RATIO: f64 = 1.45;
/// Start slowing to the approach speed here (nm), reaching it at the FAP.
/// Commanding the approach speed from the moment autoland arms is what flew an
/// SF50 at 92 kt NINE MILES OUT with the gear up (Southampton, 2026-07-30) —
/// the airframe's own gear warning was right and we were wrong. Gear comes
/// down at 6 nm, inside this ramp, so it extends well below Vlo.
const DECEL_START_NM: f64 = 10.0;

/// The clean manoeuvring speed for an airframe with this approach speed — what
/// autoland flies outside the deceleration ramp, and what the procedure
/// follower holds down a straight-in so the FAF handoff arrives at a speed
/// autoland expects.
pub fn clean_approach_ias(approach_ias: f64) -> f64 {
    approach_ias * CLEAN_APPROACH_RATIO
}

/// May the gear come down at this speed? The profile's VLO/VLE gates it; a
/// profile with no figure never gates. Autoland used to lower the gear the
/// moment it was inside 6 nm whatever the speed: 195 kt on a Baron placarded
/// 152 (2026-09-11).
pub fn gear_may_extend(ias_kts: f64, vlo_kts: Option<f64>) -> bool {
    vlo_kts.is_none_or(|v| ias_kts <= v)
}

/// The most flap the schedule's `wanted` ratio allows at this speed: the
/// deepest detent at or below `wanted` whose placard Vfe the speed respects.
/// `limits` is the profile's detents as `(ratio, Vfe)`, retracted first; a
/// detent with no Vfe never gates, and an empty list (no flap profile) hands
/// `wanted` straight back. The setting is held back, not skipped: as the speed
/// bleeds off, the schedule's target becomes legal and goes out then.
pub fn flap_within_limits(wanted: f64, ias_kts: f64, limits: &[(f64, Option<f64>)]) -> f64 {
    if limits.is_empty() {
        return wanted;
    }
    limits
        .iter()
        .filter(|(ratio, vfe)| *ratio <= wanted + 1e-6 && vfe.is_none_or(|v| ias_kts <= v))
        .map(|(ratio, _)| *ratio)
        .fold(0.0, f64::max)
}

/// A detent goes out only with this much room under its placard...
const FLAP_EXTEND_MARGIN_KTS: f64 = 5.0;
/// ...and one already out comes back in only this far over it.
const FLAP_RETRACT_MARGIN_KTS: f64 = 8.0;

/// [`flap_within_limits`] with a memory. The bare rule is a comparator on a
/// noisy airspeed: the 737's flap 40 went out at 161 kt under a 162 placard,
/// came back to 10 at 162, and went out again at 900 ft on final (EGTE,
/// 2026-09-21). A crew that touches the placard slows down; it does not cycle
/// the flaps. Extending wants margin under the limit, a detent already out is
/// held until the speed is properly over it, and a schedule that asks for
/// LESS (the go-around's half) is obeyed at once.
pub fn flap_held_within_limits(
    wanted: f64,
    ias_kts: f64,
    limits: &[(f64, Option<f64>)],
    current: Option<f64>,
) -> f64 {
    let cur = current.unwrap_or(0.0);
    if wanted + 1e-6 < cur {
        return flap_within_limits(wanted, ias_kts, limits);
    }
    let extend = flap_within_limits(wanted, ias_kts + FLAP_EXTEND_MARGIN_KTS, limits);
    if extend > cur {
        return extend;
    }
    flap_within_limits(cur, ias_kts - FLAP_RETRACT_MARGIN_KTS, limits)
}

/// An APPROACH never takes away the flap the wing is flying on. A detent comes
/// in only at the speed that flies what is left (`retract_kts`: the cleaner
/// configuration's manoeuvring speed where the profile declares one, else its
/// speed floor), unless a placard forces it (over the current detent's Vfe,
/// [`flap_within_limits`]) or the go-around asks for its half (`go_around`).
/// LEMG 13, 2026-09-28: every sim-started 10 nm final handed autoland the 737
/// with flaps 2 (0.25) at idle, the distance schedule wanted it clean, and
/// the flaps came up on the first tick at a 58 kt airspeed read (the sim's
/// ASI still waking) and 146 kt a second later, under the 139 kt clean
/// stall; with the lever at idle and a climb on the dial the aircraft sank
/// at up to 4,400 fpm (sortie 1790583434-approach bottomed at 200 ft AGL,
/// 1790584266-approach turned onto 12's line over rising ground and hit).
/// `current` is `None` only when the flap position was never read. Pure.
pub fn flap_kept_flying(
    flap: f64,
    current: Option<f64>,
    ias_kts: f64,
    limits: &[(f64, Option<f64>)],
    go_around: bool,
    retract_kts: impl Fn(f64) -> Option<f64>,
) -> f64 {
    let Some(cur) = current else {
        return flap;
    };
    if go_around || flap + 1e-6 >= cur {
        return flap;
    }
    // Over the placard of what is out: the placard wins, it is the other
    // edge of the same envelope.
    if flap_within_limits(cur, ias_kts, limits) + 1e-6 < cur {
        return flap;
    }
    match retract_kts(flap) {
        Some(need) if !(ias_kts >= need) => cur,
        _ => flap,
    }
}

/// The speed a flap setting may be retracted TO for this airframe: the
/// detent's declared manoeuvring speed (`retract_above_kts`), else its speed
/// floor (1.2 x the stall in it), else nothing known.
fn retract_speed_kts(ratio: f64) -> Option<f64> {
    crate::aircraft::live_profile()
        .and_then(|p| {
            crate::aircraft::flap_schedule(p.flaps.as_ref())
                .into_iter()
                .find(|(r, _)| (r - ratio).abs() < 1e-3)
                .map(|(_, k)| k)
        })
        .or_else(|| crate::aircraft::config_floor_kts(ratio))
}

/// Speed to command at this distance: clean far out, approach speed by the FAP,
/// linear between. Pure, so the schedule is testable without a sim.
pub fn scheduled_ias(dist_nm: f64, approach_ias: f64) -> f64 {
    let clean = clean_approach_ias(approach_ias);
    if dist_nm >= DECEL_START_NM {
        clean
    } else if dist_nm <= FAP_NM {
        approach_ias
    } else {
        let frac = (dist_nm - FAP_NM) / (DECEL_START_NM - FAP_NM);
        approach_ias + (clean - approach_ias) * frac
    }
}

/// The speed a reposition pattern's downwind and base are flown at: midway
/// down the deceleration schedule, and well inside the half-flap placard it is
/// flown with. The approach speed itself belongs to the final: at it, with the
/// reposition's climb bonus, the 737 flew its downwind at 139 to 162 kt under
/// flap 40's 162 kt limit.
pub fn pattern_ias(approach_ias: f64, flap_limits: &[(f64, Option<f64>)]) -> f64 {
    let mid = scheduled_ias((FAP_NM + DECEL_START_NM) / 2.0, approach_ias);
    let half_vfe = flap_limits
        .iter()
        .filter(|(ratio, _)| *ratio <= 0.5 + 1e-6)
        .filter_map(|(_, vfe)| *vfe)
        .fold(f64::INFINITY, f64::min);
    mid.min(half_vfe - 15.0).max(approach_ias)
}

/// What the copilot says when a final drops back to an intercept without a
/// go-around, or `None` when another line already says it. "Overflew the
/// runway" only with the runway BEHIND (`along_nm` negative): it was said at
/// 5.2 nm on the tick a pattern began, over the pattern's own "Repositioning"
/// line (LEMG 13, sortie 1790622093-approach, #171). Pure.
pub fn dropped_final_line(
    along_nm: f64,
    reversal_began: bool,
    above_path_ft: f64,
) -> Option<&'static str> {
    if along_nm < 0.0 {
        Some("Overflew the runway. Going around, climbing to re-intercept.")
    } else if reversal_began {
        None
    } else if above_path_ft > 0.0 {
        Some("High on the approach. Re-intercepting the glidepath.")
    } else {
        Some("Off the final. Re-intercepting.")
    }
}

/// Room past the path's own sink the glidepath law may command to get back
/// down to it, fpm.
const PATH_CATCH_UP_FPM: f64 = 300.0;

/// The steepest descent the glidepath law commands, fpm (negative): the old
/// flat 1,200, or the path's own sink at this groundspeed plus
/// [`PATH_CATCH_UP_FPM`] when that is steeper. A flat 1,200 is the 3 degree
/// path at 226 kt over the ground: faster, the law could not even HOLD the
/// path, the error grew, and past 500 ft high the guidance aimed at a join
/// point behind the aircraft, which the reversal read as a pattern (the A330
/// at 239 kt, 491 ft high at 5.3 nm on a -1,200 dial, then a downwind from
/// 5.2 nm; LEMG 13, sortie 1790622093-approach, #171). Pure.
pub fn path_descent_cap_fpm(nominal_vs_fpm: f64) -> f64 {
    (nominal_vs_fpm - PATH_CATCH_UP_FPM).min(-1200.0)
}

/// The placard of the flap that is out: the lowest Vfe of every detent at or
/// inside `flap_ratio`. `None` clean, or with no Vfe declared. Pure.
pub fn flap_placard_kts(flap_ratio: f64, flap_limits: &[(f64, Option<f64>)]) -> Option<f64> {
    flap_limits
        .iter()
        .filter(|(ratio, _)| *ratio > 1e-6 && *ratio <= flap_ratio + 1e-6)
        .filter_map(|(_, vfe)| *vfe)
        .reduce(f64::min)
}

/// The approach's speed target sits this far under the placard of the flap out.
const PLACARD_TARGET_MARGIN_KTS: f64 = 10.0;

/// The speed the approach's governor holds: the schedule, never closer to the
/// placard of the flap actually out than [`PLACARD_TARGET_MARGIN_KTS`], and
/// never under that configuration's floor (the floor wins a conflict). The
/// schedule's clean speed is 1.45 x the approach speed, 223 kt on the A330's
/// 154: with CONF 1 out (Vfe 215) the governor added power to reach it, the
/// placard took the flap, the clean floor (1.2 x 175 = 210) then held the
/// lever up, and a 10 nm final ran at 200 to 241 kt (LEMG 13, sortie
/// 1790622093-approach, 2026-09-28, #171). Pure.
pub fn approach_target_kts(
    scheduled_kts: f64,
    flap_ratio: f64,
    flap_limits: &[(f64, Option<f64>)],
    floor_kts: Option<f64>,
) -> f64 {
    let ceiling = flap_placard_kts(flap_ratio, flap_limits)
        .map_or(f64::INFINITY, |v| v - PLACARD_TARGET_MARGIN_KTS);
    crate::aircraft::floored_target(scheduled_kts.min(ceiling), floor_kts)
}

/// The flap an approach asks for: the distance schedule, never LESS than is
/// already out. A flap out is drag and a lower floor, both what an approach
/// needs; only a placard (`flap_held_within_limits`) or the go-around's half
/// takes one away. The schedule wanted the A330 clean beyond 6 nm, CONF 1
/// came up at 211 kt 7.9 nm out (the clean retract speed, 210), and the clean
/// floor then kept the final at 210 and over (#171). `current` is the last
/// flap written or read. Pure.
pub fn approach_flap_wanted(scheduled: f64, current: Option<f64>) -> f64 {
    scheduled.max(current.unwrap_or(0.0))
}

/// The speed a give-up leaves the governor holding: the pattern speed, inside
/// the placard of whatever flap is actually out, never under the approach
/// speed. With the flaps up it is the clean approach speed.
pub fn hand_back_speed_kts(
    approach_ias: f64,
    flap_ratio: f64,
    flap_limits: &[(f64, Option<f64>)],
) -> f64 {
    if flap_ratio < 0.05 {
        return clean_approach_ias(approach_ias);
    }
    let placard = flap_placard_kts(flap_ratio, flap_limits).unwrap_or(f64::INFINITY);
    pattern_ias(approach_ias, flap_limits)
        .min(placard - 15.0)
        .max(approach_ias)
}

/// Past the threshold by more than this (nm along the centreline, departure side)
/// with no landing = an overflight. Go around: fly back to the FAP instead of
/// tracking the runway heading straight out. Seen live 2026-07-06: overflew the
/// numbers at 167 ft, then departed the far end on a fixed course to 5.4 nm at
/// 60 kt, never turning back. The margin ignores a normal float/rollout past the
/// threshold (the flare branch, checked first, still owns the touchdown).
const OVERFLIGHT_GO_AROUND_NM: f64 = 0.15;
/// The float must leave at least this much pavement to land and stop on before
/// the overflight rule lets it continue — the Baron's measured comfortable
/// rollout from 79 kt used ~550 m.
const FLOAT_MIN_REMAINING_M: f64 = 900.0;

/// How far past the threshold the aircraft may fly before "overflight" is the
/// verdict. The fixed 0.15 nm was tuned when the descent ran at path rate all
/// the way to the flare gate; the roundout legitimately floats further, and at
/// EGHI it balked a perfectly good landing at 26.3 ft radio — ONE FOOT above
/// the 25 ft flare gate — with 1,400 m of runway still ahead (2026-08-02).
/// Low over the pavement, in the landing float, the question is not "how far
/// past the numbers" but "is there still room to land and stop": allow the
/// float as long as [`FLOAT_MIN_REMAINING_M`] remains, capped at half the
/// runway (not down by halfway is a go-around on ANY runway), floored at the
/// old margin so a short strip keeps exactly the old rule. HIGH past the
/// threshold (above the roundout band) the question is geometric: where a
/// 3 degree descent from HERE reaches the band, is the float allowance still
/// ahead? The fixed 0.15 nm was a 172 figure twice over: at 60 kt a 172 that
/// crosses the numbers 50 ft high is in the band 100 m later, a 737 at 140 kt
/// is 290 m in, and EGCC 23L balked it at 57 ft and 322 m with 2,700 m of
/// runway ahead (2026-09-19). The band's own allowance less the run to the
/// band, floored at the old margin, so the 167 ft overflight over a short
/// strip is still caught and a long runway is judged by its length. The
/// pavement a landing needs is the airframe's recited landing distance when
/// it has one (the MD-82 would float Luton away on the Baron's 900 m).
pub fn overflight_allowance_nm(
    length_m: f64,
    radio_alt_ft: f64,
    band_ft: f64,
    landing_m: Option<f64>,
) -> f64 {
    let need_m = landing_m
        .filter(|v| v.is_finite() && *v > 0.0)
        .unwrap_or(FLOAT_MIN_REMAINING_M)
        .max(FLOAT_MIN_REMAINING_M);
    let low_m = (length_m - need_m).min(0.5 * length_m);
    let run_to_band_m = (radio_alt_ft - band_ft).max(0.0) * M_PER_FT / gp_tan();
    ((low_m - run_to_band_m) / M_PER_NM).max(OVERFLIGHT_GO_AROUND_NM)
}
const M_PER_FT: f64 = 0.3048;
/// tan(3 degrees): the metres a 3 degree path runs per metre of height.
const GLIDEPATH_TAN: f64 = 0.052_408;

/// A latched go-around climbs straight ahead until this height before the
/// intercept guidance may turn it back toward the FAP (see GoAroundLatch).
const GO_AROUND_TURN_AGL_FT: f64 = 800.0;
/// How near the extended centreline the reposition leg must be before it
/// hands back to the ordinary approach (see [`GoAroundLatch::update`]). Half
/// a mile leaves the pursuit ~17 m of offset after four time constants on a
/// 737's 118 s final, inside the flare gate; a 172's 280 s final never
/// needed the rule.
const REPOSITION_RELEASE_XTK_NM: f64 = 0.5;
/// Terrain floor (ft AGL): below this and outside the flare zone, low radio
/// altitude is rising terrain under the flight path, not the runway — climb away
/// from it rather than hold level into it. Seen live 2026-07-06: held level at
/// ~50 ft AGL 5 nm out over rising ground, no climb authority anywhere in the loop.
const TERRAIN_FLOOR_AGL_FT: f64 = 500.0;

/// Fast by more than this on final and the speed law may still idle a jet.
const APPROACH_FAST_KTS: f64 = 10.0;
/// The flap ratio that is the landing configuration (the schedule's full).
pub(crate) const LANDING_FLAP_RATIO: f64 = 0.999;
/// The lever's idle stop in the speed law.
const IDLE_THROTTLE: f64 = 0.1;

/// The least the speed law may pull the lever to. On FINAL, not more than
/// `APPROACH_FAST_KTS` fast, it is the airframe's approach thrust floor
/// (jets fly the approach on thrust: the 737 idled its last mile and touched
/// down 17 kt under Vref, LFLL 2026-09-17) - but only in the LANDING
/// configuration (`landing_flaps`), the one the floor was measured in (0.4
/// at Vref with flaps 40). With flaps 10 the same 0.3 is more than the
/// path needs: from the FAP to 3 nm every LEMG 13 final of 2026-09-26 sat
/// at 161 to 164 kt, the lever flipping 0.1 / 0.3 on the ten-knot edge of
/// the flaps-10 floor (152), so the landing flaps came out late and the
/// final was still 155 to 158 kt through 800 ft. In a REPOSITION below the
/// terrain floor it is full power: excess speed there is spent on the
/// climb, never on the lever ("land the aircraft" from 140 ft at 237 kt
/// pulled the throttle to idle for its 179 kt target and the aircraft
/// settled into the fields with +800 fpm on the dial, same day). Everywhere
/// else, idle.
/// The roundout's retard where the airframe declares no `flare_power`.
const DEFAULT_ROUNDOUT_RETARD_PER_S: f64 = 0.1;

/// The lever in a roundout (the arrest holding the servos, not yet the
/// Flare) that FLOATS: a sink under [`RETARD_FLOAT_FPM`], now or
/// [`RETARD_FLOAT_LEAD_S`] ahead on its trend, walks the lever toward idle
/// at `per_s`, under the approach's own floor; else `None` (the speed law's).
/// King Air LEMG 13 (iMac 2026-09-30, 1790786159 and 1790787298): the
/// column at neutral, the arrest took the sink from 557 fpm to level at 36 ft
/// with the lever held at 0.61 (the speed law "slow" by a knot, only ever
/// back in the roundout), climbed to 101 ft at 700 fpm and balked `balloon`.
/// Pure.
pub fn roundout_float_lever(throttle: f64, vvi_fpm: f64, trend_fpm_s: f64, dt_s: f64, per_s: f64) -> Option<f64> {
    let floating = vvi_fpm + trend_fpm_s.max(0.0) * RETARD_FLOAT_LEAD_S > -RETARD_FLOAT_FPM;
    floating.then(|| (throttle - per_s * dt_s.clamp(0.0, 1.0)).max(IDLE_THROTTLE))
}

pub fn approach_power_floor(
    phase: ApproachPhase,
    radio_alt_ft: f64,
    err_kts: f64,
    approach_thrust_min: Option<f64>,
    landing_flaps: bool,
) -> f64 {
    match phase {
        ApproachPhase::Intercept if radio_alt_ft < TERRAIN_FLOOR_AGL_FT => 1.0,
        ApproachPhase::Final if err_kts > -APPROACH_FAST_KTS && landing_flaps => {
            approach_thrust_min
                .unwrap_or(IDLE_THROTTLE)
                .max(IDLE_THROTTLE)
        }
        _ => IDLE_THROTTLE,
    }
}
/// A climb is flown on thrust. Repositioning (Intercept) with the vertical
/// law asking to CLIMB, the lever never sits below the airframe's approach
/// thrust floor (idle when none is declared): the speed law alone reads a
/// dive's rising airspeed as "fast" and idles. LEMG 13, 2026-09-28 (sortie
/// 1790584266-approach): +100 to +2,000 fpm on the dial under the reposition
/// floor, the lever at 0.10 from the arm until 467 ft radio, the 737 sinking
/// 1,200 to 3,700 fpm. Final and the flare keep their own rules. Pure.
pub fn climb_power_floor(
    phase: ApproachPhase,
    target_vs_fpm: f64,
    approach_thrust_min: Option<f64>,
) -> f64 {
    if phase == ApproachPhase::Intercept && target_vs_fpm > 0.0 {
        approach_thrust_min.unwrap_or(IDLE_THROTTLE).max(IDLE_THROTTLE)
    } else {
        IDLE_THROTTLE
    }
}

/// Don't enforce the terrain floor inside this range ON FINAL — here a low
/// radio altitude IS the approach (short final), not terrain to climb over.
/// The gate applies to Final only: in Intercept the aircraft is repositioning,
/// not landing, so proximity to the threshold means nothing — the KPDK 16
/// rejoin (2026-07-14) armed 0.75 nm out at 300 AGL, the gate held the floor
/// off, and the vertical law tracked the (distance-symmetric) glidepath into
/// the ground 0.35 nm short of the runway.
const FLOOR_ENFORCE_MIN_DIST_NM: f64 = 1.0;
/// ...and on Final the floor binds only this far BELOW the glidepath. A 3°
/// path crosses 500 AGL at ~1.6 nm, so at any flat field the floor used to
/// capture a perfectly on-path approach between 1.6 and 1.0 nm — held it
/// level with the climb-energy speed bonus, then released it 200 ft high
/// and 15 kt hot at the numbers for a guaranteed overflight go-around
/// (LXGB 09, 2026-07-22). Every scenario the floor exists for (dragged in
/// low over rising ground) is deeply below the path; on-path with low
/// radio near the field IS the approach.
const FLOOR_BELOW_PATH_FT: f64 = -100.0;
/// Repositioning (Intercept) wants PLATFORM height — the ~1,600 ft AGL the
/// aircraft must be back at by the FAP to fly the path down anyway. Not 500
/// ("flying around 500 feet, is that a bit too low?" — it was), and not
/// 1,000 either: the LXGB 27 go-around turned back toward a FAP east of the
/// field and flew level at ~650 ft AGL into the Rock of Gibraltar (1,400 ft;
/// radio collapsed 680 → 3 in six seconds, CFIT, 2026-07-22). Terrain-aware
/// routing is the real fix; until then, climb to the altitude the rejoin
/// needs and the highest terrain a sane approach corridor hides.
const REPOSITION_MIN_AGL_FT: f64 = FAP_NM * GLIDEPATH_FT_PER_NM;
/// A pattern plan older than this is stale: the aircraft has flown on from
/// where its downwind was probed from.
const REVERSAL_PLAN_FRESH_S: f64 = 60.0;

/// The live floor is probed this often, and a tripped floor stands this
/// long after its last trip (the circuit's 2 s and 60 s).
const LIVE_FLOOR_EVERY_S: f64 = 2.0;
const LIVE_FLOOR_HOLD_S: f64 = 60.0;

use crate::reversal::TerrainHold;

/// The circuit's leg whose live floor (`circuit::live_floor_ft`) a reversal
/// leg or a hold keeps: the level legs climb clear of ground ahead, the base
/// (and the ordinary intercept, descending to its path) only keeps the floor
/// itself. `None` on the final and in the go-around's own climb. Pure.
fn live_floor_leg(
    phase: ApproachPhase,
    missed: Option<MissedStage>,
    pattern_leg: Option<crate::reversal::Leg>,
    holding: bool,
) -> Option<crate::circuit::Leg> {
    use crate::circuit::Leg as C;
    if phase != ApproachPhase::Intercept || missed == Some(MissedStage::Climb) {
        return None;
    }
    Some(match pattern_leg {
        Some(crate::reversal::Leg::Base) => C::Base,
        Some(_) => C::Downwind,
        None if holding || missed.is_some() => C::Downwind,
        None => C::Base,
    })
}

#[cfg(test)]
mod terrain_hold_tests {
    use super::*;

    /// #182: the live floor rides every leg but the final and the go-around's
    /// own climb; the level legs climb clear of ground ahead, the base and
    /// an intercept keep the floor itself. LEMG 13 (sortie
    /// 1790668215-full_auto, t+1795): 2,565 ft over 2,043 on the downwind
    /// was 522 ft, a hair outside the 500 ft trip (the corridor plan is the
    /// fix, the floor the backstop); 25 ft lower it climbs to 2,843.
    #[test]
    fn the_live_floor_rides_every_leg_but_the_final() {
        use crate::circuit::Leg as C;
        use crate::reversal::Leg as R;
        let i = ApproachPhase::Intercept;
        assert_eq!(
            live_floor_leg(i, None, Some(R::Outbound), false),
            Some(C::Downwind)
        );
        assert_eq!(live_floor_leg(i, None, Some(R::Base), false), Some(C::Base));
        assert_eq!(
            live_floor_leg(i, None, Some(R::Inbound), false),
            Some(C::Downwind)
        );
        assert_eq!(live_floor_leg(i, None, None, true), Some(C::Downwind));
        assert_eq!(
            live_floor_leg(i, Some(MissedStage::Reposition), None, false),
            Some(C::Downwind)
        );
        assert_eq!(live_floor_leg(i, None, None, false), Some(C::Base));
        assert_eq!(
            live_floor_leg(i, Some(MissedStage::Climb), None, false),
            None
        );
        assert_eq!(
            live_floor_leg(ApproachPhase::Final, None, None, false),
            None
        );
        assert_eq!(
            live_floor_leg(ApproachPhase::Flare, None, None, false),
            None
        );
        assert_eq!(
            crate::circuit::live_floor_ft(C::Downwind, 2564.7, 2043.0),
            None
        );
        let floor = crate::circuit::live_floor_ft(C::Downwind, 2540.0, 2043.0);
        assert_eq!(floor, Some(2843.0));
    }
}

/// Only climb with this much surplus over the approach speed. Climbing slow in a
/// marginal airframe bleeds energy to a stall (the reason this loop was
/// descent-only) — so a low-and-slow aircraft holds level and lets the throttle
/// governor rebuild speed FIRST, then the climb engages once the surplus exists.
const CLIMB_SPEED_MARGIN_KTS: f64 = 5.0;
/// Ceiling on the safe-altitude climb (fpm). Was 500 — "gentle and energy-
/// preserving" read as anemic against real terrain (the Rock closed at an
/// effective 5,000+ fpm, 2026-07-22). The climb gate already requires
/// surplus airspeed, and speed protection still owns the slow case.
const MAX_CLIMB_FPM: f64 = 800.0;

/// ...and the AIRFRAME's climb when it has more, because 800 fpm is a light
/// single's answer to terrain and an airliner's idea of level flight. It was
/// raised from 500 once already for being anemic; on a 737 it is anemic
/// again — the EGPH go-around (2026-09-20) climbed away from a balked
/// approach at exactly this number. Never LESS than the GA figure, so
/// nothing that flew before gets weaker.
pub(crate) fn max_climb_fpm() -> f64 {
    crate::procedure::climb_fpm().max(MAX_CLIMB_FPM)
}
/// While below the terrain floor, command the throttle governor to this much over
/// the approach speed, so it builds the surplus the climb gate needs.
const CLIMB_ENERGY_BONUS_KTS: f64 = 12.0;

/// Flare only inside this distance to the threshold — beyond it, a low radio
/// altitude is terrain under the approach, not the runway. (25 ft on a 3° path
/// is ~0.08 nm out; 0.35 leaves margin for displaced thresholds.)
const FLARE_MAX_DIST_NM: f64 = 0.35;
/// The roundout band: below this multiple of the flare height (inside the same
/// runway gates as the flare itself), the vertical target eases to
/// [`ROUNDOUT_VS_FPM`] so the flare is entered from a gentle descent rather
/// than from the full path rate or a level float.
const ROUNDOUT_MULT: f64 = 2.0;
const ROUNDOUT_VS_FPM: f64 = -250.0;
const FLARE_VS_FPM: f64 = -100.0;

/// The arrest gates are SECONDS of sink, not feet: 25 ft is three seconds
/// of a 172's path sink and two of a 737's, and the 737 through this
/// autopilot's lag needs every one of them (EGCC 23L, 2026-09-19: the
/// roundout opened at 50 ft with the aircraft sinking 712 fpm, 4 s from the
/// pavement, and the arrest could only balloon or arrive firm). Both gates
/// are the greater of the fixed feet and the seconds at the NOMINAL path
/// sink for this groundspeed, so every airframe up to a 172's speeds keeps
/// exactly its old feet (50 and 25 at 75 kt) and a jet's open higher: at
/// 132 kt over the ground, 76 and 35.
const ROUNDOUT_LEAD_S: f64 = 6.5;
const FLARE_LEAD_S: f64 = 3.0;
/// The gates for this approach: (flare height, roundout band top), feet.
pub fn arrest_gates_ft(flare_radio_ft: f64, gs_kts: f64) -> (f64, f64) {
    let sink_fps = (gs_kts.max(0.0) / 60.0) * gp_ft_per_nm() / 60.0;
    let flare = flare_radio_ft.max(sink_fps * FLARE_LEAD_S);
    let band = (flare * ROUNDOUT_MULT).max(sink_fps * ROUNDOUT_LEAD_S);
    (flare, band)
}
/// The arrest's targets scale with the path sink the same way: the 172's
/// -250 then -100 are 60% and 25% of its 400 fpm path, and a 737 asked to
/// hold -100 from 35 ft floats a kilometre at 140 kt before it touches.
/// (roundout, flare), fpm; never shallower than the fixed figures.
pub fn arrest_targets_fpm(nominal_vs_fpm: f64) -> (f64, f64) {
    (
        ROUNDOUT_VS_FPM.min(0.6 * nominal_vs_fpm),
        FLARE_VS_FPM.min(0.25 * nominal_vs_fpm),
    )
}

/// The flare's sink target at this height: an EXPONENTIAL flare, as
/// autoland systems fly it. The touchdown sink (`flare_vs`) plus the height
/// over [`FLARE_TAU_S`], never steeper than the approach path. The flare
/// used to hold `flare_vs` flat from its gate: a 737 asked for -175 fpm from
/// 35 ft takes twelve seconds to arrive, and floated LEMG 13 from 36 to 19 ft
/// over five of them before the power came off and it dropped in at -859
/// fpm (2026-09-26). Proportional to the height it keeps descending and
/// arrives at `flare_vs` in about five. Pure.
pub fn flare_target_fpm(radio_ft: f64, flare_vs: f64, nominal_vs: f64) -> f64 {
    flare_target_with(radio_ft, flare_vs, nominal_vs, flare_tau_s())
}

/// [`flare_target_fpm`] with the time constant given: `None` is the flat
/// flare every airframe but a declared one keeps. The 172's flare is flown
/// through its VS mode's 4.5 s lag, and a target that moves with the height
/// is faster than that lag can follow (the sortie test lands it at -298
/// fpm); the 737's is flown on the elevator with the servos out. Pure.
pub fn flare_target_with(radio_ft: f64, flare_vs: f64, nominal_vs: f64, tau_s: Option<f64>) -> f64 {
    match tau_s.filter(|t| t.is_finite() && *t > 0.5) {
        Some(tau) => (flare_vs - radio_ft.max(0.0) * 60.0 / tau).max(nominal_vs.min(flare_vs)),
        None => flare_vs,
    }
}

/// The live airframe's exponential flare time constant, if it declares one
/// (`flare_elevator.tau_s`).
fn flare_tau_s() -> Option<f64> {
    crate::aircraft::live_profile()
        .and_then(|p| p.flare_elevator)
        .and_then(|f| f.tau_s)
}

/// The VS mode is a slow servo, and the landing arrest is the one place the
/// dial's lag is measured in feet of runway. LXGB 27, 2026-09-14 (the 172's
/// third firm arrival in three days, same shape each time): the roundout
/// dialled -250 at 49 ft radio and the aircraft was still sinking 386 fpm
/// four seconds later; the flare dialled -100 at 23 ft and the vertical
/// speed never moved off -260 before the wheels touched at -309 (the
/// blackbox's `cmd_vs` column changed on time, the `vsi` column never
/// followed). Fitted to that tick log the autopilot follows the dial as a
/// first-order lag of about 4.5 s with a tick of dead time, which from the
/// flare gate at approach sink is longer than the flare itself. So in the
/// roundout and the flare the DIAL leads the target by the error the
/// autopilot still owes (`vs_dial_fpm`): dial = target + gain x (target -
/// actual). Gain 2 makes the closed lag ~1.5 s, comfortably inside the
/// stability margin a 1 s dead time leaves (about 7). The dial may read a
/// CLIMB for a moment - that is the flare, the yoke coming back - capped at
/// [`VS_LEAD_MAX_CLIMB_FPM`] so a balloon can never be commanded, and the
/// glidepath above the band is untouched (its own P law tracks a ramp with
/// `path_descent_fpm`, and every approach test stays byte-identical).
/// The 172's numbers live in [`crate::aircraft::ArrestLeadProfile::default`]
/// (gain 2, cap +300, no trend term); a jet declares its own in its profile.
#[cfg(test)]
const VS_LEAD_GAIN: f64 = 2.0;
#[cfg(test)]
const VS_LEAD_MAX_CLIMB_FPM: f64 = 300.0;

/// The altitude dial must never be within reach BELOW the aircraft while
/// autoland flies: an armed capture takes the vertical away from the VS dial
/// (see the arm sequence in [`run_autoland`]). `None` = already clear.
pub const ALT_DIAL_CLEAR_FT: f64 = 500.0;
/// Where the dial is parked: this far above the arm altitude, rounded UP to
/// the hundred, so a go-around's climb captures a sensible level at worst.
pub const ALT_DIAL_PARK_ABOVE_FT: f64 = 1000.0;
pub fn parked_altitude_dial_ft(dial_ft: f64, alt_ft: f64) -> Option<f64> {
    if dial_ft >= alt_ft + ALT_DIAL_CLEAR_FT {
        return None;
    }
    Some(((alt_ft + ALT_DIAL_PARK_ABOVE_FT) / 100.0).ceil() * 100.0)
}

/// A probed threshold elevation within this of the header is the header (the
/// mesh and the file agree; no reason to move the aim by a few feet).
pub const THRESHOLD_ELEV_TRUST_FT: f64 = 20.0;
/// ...and one further than this from it is not believed (a probe off the
/// pavement, a broken mesh): the header stays, the number is still reported.
pub const THRESHOLD_ELEV_MAX_DIFF_FT: f64 = 300.0;
/// The glidepath's aim elevation: the HIGHER of the probed threshold and
/// touchdown-zone points when they disagree with the apt.dat header by more
/// than [`THRESHOLD_ELEV_TRUST_FT`] and less than [`THRESHOLD_ELEV_MAX_DIFF_FT`];
/// the header otherwise (and always without a probe). Returns (aim, probed).
pub fn aim_elevation_ft(header_ft: f64, probed_ft: Option<&[f64]>) -> (f64, Option<f64>) {
    let Some(p) = probed_ft.filter(|v| !v.is_empty()) else {
        return (header_ft, None);
    };
    let probed = p.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if !probed.is_finite() {
        return (header_ft, None);
    }
    let diff = (probed - header_ft).abs();
    if diff <= THRESHOLD_ELEV_TRUST_FT || diff > THRESHOLD_ELEV_MAX_DIFF_FT {
        return (header_ft, Some(probed));
    }
    (probed, Some(probed))
}

/// How far a tuned glideslope may move the aim from the pavement's, ft:
/// the path's threshold crossing (51.6 ft at 3 degrees aimed 300 m in)
/// stays inside a book's 35 to 80 ft.
pub const GS_AIM_BELOW_FT: f64 = 15.0;
pub const GS_AIM_ABOVE_FT: f64 = 30.0;

/// The aim elevation with the runway's ILS glideslope tuned: the path that
/// lies ON the glideslope (`ils::glideslope_aim_ft`), as far as a book's
/// threshold crossing over the pavement (`pavement_aim_ft`, from
/// [`aim_elevation_ft`]) allows; the pavement's aim without one. Pure.
///
/// EGBB 15, 2026-09-28 (sortie 1790601822-full_auto, #156): the probe put
/// the aim at 297 ft (the header 327) while the glideslope's antenna stands
/// at 348 ft, 0.34 nm in: the needle read 1.2 dots low at 497 ft on a path
/// autoland was flying as it meant to. Aimed at 327 the path lies
/// 30 ft nearer the beam with the wheels still crossing the numbers at 81 ft.
pub fn aim_with_glideslope(pavement_aim_ft: f64, gs_aim_ft: Option<f64>) -> f64 {
    match gs_aim_ft.filter(|g| g.is_finite()) {
        Some(g) => pavement_aim_ft + (g - pavement_aim_ft).clamp(-GS_AIM_BELOW_FT, GS_AIM_ABOVE_FT),
        None => pavement_aim_ft,
    }
}

/// The gentlest retard below the retard height, lever per second.
const RETARD_MIN_PER_S: f64 = 0.03;
/// The retard waits while the airspeed is this far under the approach speed
/// (kt), unless the profile says otherwise (`retard_slow_kts`).
const RETARD_SLOW_KTS: f64 = 5.0;
/// Time to the wheels is taken at no less than this sink (fpm): a float is
/// not an infinite time, it is a slow retard.
const RETARD_SINK_FLOOR_FPM: f64 = 100.0;
/// A flare sinking less than this (fpm) is FLOATING: the arrest is done and
/// the thrust is what holds it up, so the retard starts, whatever the height
/// and the speed. EGBB 15 (737, 2026-09-28, sortie 1790601822): levelled at
/// 22.7 ft on 0.30 of lever, held by the height key over 20 ft, then by the
/// time key (a float is 13 s from the wheels), then by the speed key once
/// the float had bled 7 kt; the lever never moved and the float ran 4 s.
pub const RETARD_FLOAT_FPM: f64 = 100.0;
/// ...or will be, this many seconds ahead on the sink's trend: the float is
/// met where the arrest is heading, not where it arrives. Thrust acts through
/// the speed and the column through the 737's 1.5 s dead time, so a retard
/// begun at the float itself is late: at EGBB 15 the sink came from -300 to
/// -111 in 2.1 s. Replayed through that flare's fitted plant (6,300 fpm per
/// unit, 1.7 s) and its air, a lead of 1 s left 3.6 s of float, 1.5 s none.
pub const RETARD_FLOAT_LEAD_S: f64 = 1.5;
/// The sink's trend for the float ahead is smoothed over this, seconds.
const RETARD_TREND_TAU_S: f64 = 0.5;
/// A flare is CLIMBING (a balloon) only past this, fpm. A near-level float
/// reads a few fpm either side of zero (EGBB 15: -2, +1, +2, 0 at 22.9 ft),
/// and +1 is not a climb: it eased the held column 0.28 to 0.20 and would
/// put the approach thrust back on a lever the float is taking off.
pub const BALLOON_CLIMB_FPM: f64 = 25.0;

/// Why the retard did what it did this tick (the `power` tick's `retard`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetardKey {
    /// The first flare tick: the approach's power is where it starts.
    Entry,
    /// Above the retard height.
    Height,
    /// More time to the wheels than `retard_at_s`.
    Time,
    /// Slower than the approach speed less the margin, the retard not yet
    /// begun and the aircraft still sinking: the thrust stays.
    Slow,
    /// Floating (sink under [`RETARD_FLOAT_FPM`], now or
    /// [`RETARD_FLOAT_LEAD_S`] ahead on its trend): walking off at the
    /// airframe's quickest retard, at any height and any speed.
    Float,
    /// Walking to idle, timed to the wheels.
    Timed,
}

impl RetardKey {
    pub fn word(self) -> &'static str {
        match self {
            RetardKey::Entry => "entry",
            RetardKey::Height => "height",
            RetardKey::Time => "time",
            RetardKey::Slow => "slow",
            RetardKey::Float => "float",
            RetardKey::Timed => "timed",
        }
    }
}

/// THE FLARE RETARD, keyed on what it is for: HEIGHT (nothing above the
/// retard height), TIME to the wheels (the lever reaches idle as they
/// arrive: the rate is what is left over the seconds left, radio over
/// sink), the FLOAT (an arrested sink held up by thrust, or one the trend
/// will arrest within [`RETARD_FLOAT_LEAD_S`]: off it comes at any height,
/// at the airframe's quickest) and SPEED (a flare slower than the approach speed less
/// [`RETARD_SLOW_KTS`] may DEFER the retard while it is still sinking; it
/// never holds power in a float and never stops a retard that has begun).
/// Never on the sink the arrest is removing: the old law held the lever
/// while the arrest worked and let it go in the float the arrest had made.
/// It "fired latest exactly when it should already be finished" (EGLF 06
/// 2026-09-22; LEMG 13 2026-09-26 and 2026-09-28; issue #76). The float is
/// the other half: height, time and speed each held 0.30 through a 4 s
/// float at 22.7 ft (EGBB 15, 2026-09-28; issue #146).
///
/// Per approach: a go-around resets it, so a second flare starts from its
/// own power. Never adds power or impersonates an airspeed governor in the
/// flare; a lever still up at the wheels is the rollout's to close.
#[derive(Default)]
struct FlarePower {
    last_command: Option<f64>,
    last_key: Option<RetardKey>,
    /// The last sink read and its smoothed rate of change (fpm per second,
    /// positive = easing), for the float ahead.
    prev_vvi: Option<f64>,
    trend_fpm_s: f64,
    /// The time to the wheels has come inside `retard_at_s`, or the flare
    /// has floated (latched).
    due: bool,
    /// The lever has started off (latched): the speed key no longer holds.
    begun: bool,
}

impl FlarePower {
    fn reset(&mut self) {
        *self = Self::default();
    }

    /// The lever this tick; `None` when a measurement is missing (no guessed
    /// power and no ramp advance). An unread airspeed is no speed key.
    #[allow(clippy::too_many_arguments)]
    fn step(
        &mut self,
        cfg: crate::aircraft::FlarePowerProfile,
        measured: f64,
        vvi: f64,
        dt: f64,
        radio_ft: f64,
        ias: Option<f64>,
        approach_ias: Option<f64>,
    ) -> Option<f64> {
        if !measured.is_finite()
            || !vvi.is_finite()
            || !radio_ft.is_finite()
            || !dt.is_finite()
            || dt <= 0.0
            || !cfg.reduction_per_second.is_finite()
            || cfg.reduction_per_second <= 0.0
        {
            return None;
        }
        // First flare tick preserves the observed approach power. Readback may
        // lag a write, but it must never restart the ramp or increase power.
        let current = measured
            .clamp(0.0, 1.0)
            .min(self.last_command.unwrap_or(1.0));
        let slow_kts = cfg
            .retard_slow_kts
            .filter(|k| k.is_finite() && *k >= 0.0)
            .unwrap_or(RETARD_SLOW_KTS);
        let slow = match (ias, approach_ias) {
            (Some(v), Some(a)) if v.is_finite() && a.is_finite() => v < a - slow_kts,
            _ => false,
        };
        // The sink's trend, smoothed over RETARD_TREND_TAU_S: a float the
        // arrest will reach a lever's lag from now is a float now.
        if let Some(p) = self.prev_vvi {
            let raw = (vvi - p) / dt;
            self.trend_fpm_s += (raw - self.trend_fpm_s) * (dt / RETARD_TREND_TAU_S).min(1.0);
        }
        self.prev_vvi = Some(vvi);
        let floating = vvi + self.trend_fpm_s.max(0.0) * RETARD_FLOAT_LEAD_S > -RETARD_FLOAT_FPM;
        let above = cfg
            .retard_below_ft
            .filter(|h| h.is_finite() && *h > 0.0)
            .is_some_and(|h| radio_ft > h);
        let sink = (-vvi).max(RETARD_SINK_FLOOR_FPM);
        let to_touch_s = (radio_ft.max(0.0) / sink * 60.0).max(0.5);
        let fast = match (ias, approach_ias, cfg.retard_fast_kts) {
            (Some(v), Some(a), Some(k)) if v.is_finite() && a.is_finite() && k.is_finite() => {
                v > a + k
            }
            _ => false,
        };
        self.due |= floating
            || (!above
                && (fast
                    || cfg
                        .retard_at_s
                        .filter(|s| s.is_finite() && *s > 0.0)
                        .is_none_or(|s| to_touch_s <= s)));
        let key = if self.last_command.is_none() {
            RetardKey::Entry
        } else if floating {
            RetardKey::Float
        } else if !self.due {
            if above {
                RetardKey::Height
            } else {
                RetardKey::Time
            }
        } else if slow && !self.begun {
            RetardKey::Slow
        } else {
            RetardKey::Timed
        };
        let fastest = cfg
            .retard_max_per_s
            .filter(|r| r.is_finite() && *r > 0.0)
            .unwrap_or(cfg.reduction_per_second)
            .max(RETARD_MIN_PER_S);
        let rate = match key {
            // A float has no time to the wheels: the thrust holding it up
            // is what comes off, as quickly as the airframe allows.
            RetardKey::Float => Some(fastest),
            RetardKey::Timed => Some((current / to_touch_s).clamp(RETARD_MIN_PER_S, fastest)),
            _ => None,
        };
        let next = match rate {
            Some(rate) => {
                self.begun = true;
                (current - rate * dt.min(1.0)).max(0.0)
            }
            None => current,
        };
        self.last_command = Some(next);
        self.last_key = Some(key);
        Some(next)
    }
}

/// The lever in a flare that is CLIMBING (past [`BALLOON_CLIMB_FPM`]): the
/// approach's own thrust, never more; `None` otherwise (the flare's usual
/// idle or progressive law). A near-level float is not a climb: its thrust
/// is the retard's to take off. Pure.
/// `above_band` holds it too: a committed balloon still above the roundout
/// band comes back down on approach power, not at idle.
pub fn balloon_power(
    vvi_fpm: Option<f64>,
    approach_thr: Option<f64>,
    above_band: bool,
) -> Option<f64> {
    match (vvi_fpm, approach_thr) {
        (Some(v), Some(t)) if (v > BALLOON_CLIMB_FPM || above_band) && t.is_finite() => {
            Some(t.clamp(0.0, 1.0))
        }
        _ => None,
    }
}

/// What to dial into the VS mode so the aircraft actually FLIES `target_fpm`.
/// The plugin's direct elevator channel, and the pilot's take-back latch —
/// the SAME pair the takeoff rotates on. `override_joystick_pitch` bypasses
/// the autopilot's pitch servo while leaving it the roll, so the flare can be
/// flown on the elevator with the AP still holding the bug.
const FLARE_PITCH: &str = "xcopilot/pitch";
const FLARE_PITCH_YIELD: &str = "xcopilot/pitch_yield";
/// The roll channel the takeoff's climb-out flies, and its take-back latch:
/// with the servos off for the arrest, the runway track is held here.
/// The airframe's own approach status and the stock one, last read (while
/// an add-on's encoding is being learned).
static LAST_APPR_READS: std::sync::Mutex<(Option<f64>, Option<f64>)> = std::sync::Mutex::new((None, None));

const FLARE_ROLL: &str = "xcopilot/roll";
const FLARE_ROLL_YIELD: &str = "xcopilot/roll_yield";

/// Elevator for a wanted sink rate, flown DIRECTLY instead of asked of the
/// autopilot's vertical-speed mode.
///
/// The whole `vs_dial_fpm` lead exists to compensate for a lag we chose to
/// fly through: the 737's VS mode is two 1.5 s lags with 1.5 s of dead time,
/// and the arrest is over in three seconds. Commanding the elevator removes
/// the lag rather than predicting it — which is what the TAKEOFF already
/// does, for the same reason, in its own comment: "the AP's VS mode is too
/// slow for a rotation". EGPH (2026-09-20) touched down at 977 fpm against a
/// 175 fpm target with the arrest asking the dial nicely.
///
/// P on the sink-rate error plus D on its trend, in units of elevator. Small
/// gains and a tight clamp on purpose: this is three seconds of flight a few
/// feet up, and a flare that over-rotates is a tail strike. The trim term is
/// the standing pull the aircraft needs to hold the attitude, found by the
/// integrator rather than assumed.
///
/// IN THE FLARE THE COLUMN ONLY COMES BACK, never forward (`ratchet`). A
/// tracker that has BEATEN its target relaxes, and forward column a few feet
/// up is a nosewheel-first landing. EGLF 06 (2026-09-22): the arrest pulled
/// the 737 from -511 to -102 fpm against a -197 target, the error went
/// negative and the elevator came back 0.35 -> 0.185 at thirteen feet, while
/// [`FlarePower`] - unlocked by that very same arrest, sink now inside its
/// `hold_sink_fpm` - walked the lever 0.29 to idle over the last 3.4 s. The
/// thrust left DURING the arrest instead of before it, the sink rebuilt to
/// -358 and worsening, and by then the law was saturated at
/// [`FLARE_PITCH_MAX`] with nothing left to give. The cap still bounds the
/// attitude, so the ratchet cannot buy a tail strike; a float it can buy is
/// the overflight guard's to judge, and the guard already knows the recited
/// landing distance.
///
/// The ROUNDOUT above the flare is ordinary flying and stays free both ways:
/// the ratchet is the last few feet, where the only question left is how
/// hard the wheels arrive.
#[derive(Debug, Default, Clone, Copy)]
pub struct FlarePitch {
    trim: f64,
    held: f64,
    gains: FlareGains,
    /// The gentlest sink the flare has reached (fpm, negative), since it
    /// began: the target never asks for more sink than that again.
    best_vvi: Option<f64>,
}

/// The flare law's numbers for THIS airframe (the profile's
/// `flare_elevator:`, [`crate::aircraft::FlareElevatorProfile`]). The default
/// is the law as the 172 and the 737 have flown it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlareGains {
    /// Elevator per 100 fpm of sink error.
    pub p: f64,
    /// Elevator per 100 fpm per second of sink trend.
    pub d: f64,
    /// The integrator's rate, elevator per 100 fpm of error per second.
    pub trim_rate: f64,
    /// Most pull, and most push.
    pub max: f64,
    pub min: f64,
    /// In a balloon the held pull eases toward the free law at most this
    /// fast (elevator per second); `None` lets go at once.
    pub balloon_ease_per_s: Option<f64>,
}

impl Default for FlareGains {
    fn default() -> Self {
        Self {
            p: FLARE_P_PER_100FPM,
            d: FLARE_D_PER_100FPM_S,
            trim_rate: FLARE_TRIM_PER_100FPM_S,
            max: FLARE_PITCH_MAX,
            min: FLARE_PITCH_MIN,
            balloon_ease_per_s: None,
        }
    }
}

impl FlareGains {
    /// The airframe's declared numbers over the default law.
    pub fn from_profile(p: Option<crate::aircraft::FlareElevatorProfile>) -> Self {
        let mut g = Self::default();
        if let Some(p) = p.and_then(|p| p.valid()) {
            g.p *= p.gain;
            g.d *= p.gain;
            g.trim_rate *= p.trim_gain;
            g.max = p.max;
            g.min = -p.max_push;
            g.balloon_ease_per_s = p.balloon_ease_per_s;
        }
        g
    }
}

/// Elevator per 100 fpm of sink error. A flare corrects hundreds of fpm with
/// a few percent of travel.
const FLARE_P_PER_100FPM: f64 = 0.060;
/// Damping on the sink TREND, per 100 fpm per second — the term that stops
/// the pull becoming a balloon.
const FLARE_D_PER_100FPM_S: f64 = 0.050;
/// The integrator's rate, per 100 fpm of standing error per second. Small:
/// it is there to find the standing pull, not to fly the arrest — wound
/// harder it carried the 737 through its target into a 66 fpm float.
const FLARE_TRIM_PER_100FPM_S: f64 = 0.015;
/// The flare holds the gentlest sink it has reached, but never asks for
/// less than this (fpm): a float is not a landing either.
const FLARE_HOLD_MIN_SINK_FPM: f64 = 150.0;
/// Total elevator the flare may command. Up is positive.
const FLARE_PITCH_MAX: f64 = 0.35;
/// ...and the push, which is deliberately far smaller: the ground is there.
const FLARE_PITCH_MIN: f64 = -0.10;
/// The ratchet lets go this far ahead of a balloon: the sink PREDICTED this
/// many seconds out (the trend) above zero counts as climbing.
const BALLOON_LEAD_S: f64 = 0.5;

impl FlarePitch {
    /// `vvi_trend` is fpm per second, positive = sink easing (the same sign
    /// the VS lead uses). `None` for either measurement holds what we had.
    ///
    /// `ratchet` is the flare proper (as against the roundout above it): see
    /// [`FlarePitch`] for why the column only ever comes further back there.
    pub fn step(
        &mut self,
        target_fpm: f64,
        vvi_fpm: f64,
        vvi_trend: f64,
        dt_s: f64,
        ratchet: bool,
    ) -> f64 {
        let dt = dt_s.clamp(0.02, 0.5);
        let g = self.gains;
        // In the flare the sink never grows again: the target is held to the
        // gentlest sink reached (no gentler than FLARE_HOLD_MIN_SINK). The
        // roundout had the 737 at -300 by 29 ft; the exponential target was
        // still -660 there, the law saw nothing to pull for, and the sink
        // built back to -587 at the wheels (LEMG 13, 2026-09-26).
        let target_fpm = if ratchet {
            let best = self.best_vvi.map_or(vvi_fpm, |b| b.max(vvi_fpm));
            self.best_vvi = Some(best);
            target_fpm.max(best.min(-FLARE_HOLD_MIN_SINK_FPM))
        } else {
            self.best_vvi = None;
            target_fpm
        };
        // Positive error = sinking FASTER than wanted = pull.
        let err = (target_fpm - vvi_fpm) / 100.0;
        self.trim = (self.trim + err * g.trim_rate * dt).clamp(g.min, g.max);
        let want = (err * g.p - (vvi_trend / 100.0) * g.d + self.trim).clamp(g.min, g.max);
        if !ratchet {
            // The roundout is still ordinary flying: the law is free both
            // ways. But the pull it ends on is where the flare STARTS: reset
            // to nothing, the ratchet began from the flare's first ask, and
            // the 737 went from 0.29 of elevator to 0.016 in one tick at the
            // hand-over (38 ft), the nose dropped, and it met the runway at
            // -773 fpm and bounced to 11 ft (LEMG circuit, 2026-09-26).
            // ...but a measured CLIMB (past BALLOON_CLIMB_FPM) is a
            // balloon here too, and a balloon never pushes: the SF50's
            // roundout wrote -0.05 at +62 and +214 fpm (LEMG 12,
            // 2026-09-28) and the sink it rebuilt met the runway.
            let want = if vvi_fpm > BALLOON_CLIMB_FPM {
                want.max(0.0)
            } else {
                want
            };
            self.held = want.max(0.0);
            return want;
        }
        if vvi_fpm > 0.0 || vvi_fpm + vvi_trend * BALLOON_LEAD_S > 0.0 {
            // A BALLOON (or one BALLOON_LEAD_S away: the SF50's 0.3 was held
            // while the sink vanished at +400 fpm a second, and let go only
            // once it was climbing, which was already a balloon to 52 ft;
            // EDLP 24, 2026-09-24): the pull has beaten the ground and the aircraft is
            // climbing. The ratchet exists for a sink that is being arrested,
            // and held through a climb it kept the Citation going up, 28 to
            // 58 ft at +545 fpm with the speed bleeding away (EDDV 27L,
            // 2026-09-22). The pull eases toward neutral and never becomes a
            // push, so the aircraft settles; the ratchet takes over again
            // from there as soon as it sinks.
            let eased = want.max(0.0);
            self.held = match g.balloon_ease_per_s {
                // Eased, not dropped: neutral is the TRIMMED approach
                // attitude, and at a flare's speed that sinks at once. The
                // SF50's 0.35 went to 0 in one tick at +190 fpm and the sink
                // was back at -590 two seconds later (EDLP 24, 2026-09-24).
                Some(rate) => eased.max(self.held - rate * dt),
                None => eased,
            };
            return self.held;
        }
        self.held = self.held.max(want);
        self.held
    }

    /// The law with this airframe's numbers.
    pub fn with_gains(gains: FlareGains) -> Self {
        Self {
            gains,
            ..Self::default()
        }
    }

    /// Let go — the channel expires on its own within a second, but neither
    /// the integrator nor the ratchet may carry a standing pull into the
    /// next approach. The airframe's numbers stay.
    pub fn reset(&mut self) {
        *self = Self::with_gains(self.gains);
    }
}

/// The flare flown on a MODEL of the airframe's elevator, for an airframe
/// that declares one (`flare_elevator.model`,
/// [`crate::aircraft::FlareModelProfile`]).
///
/// The P+D tracker above treats the sink as something the elevator
/// accelerates: pull and it keeps arresting. The 737's LEMG flares of
/// 2026-09-26 say a held pull HOLDS a sink, arriving after a dead time, and
/// that the aircraft pitches down by itself over the last forty feet (the
/// tail's ground effect): at LEMG 12 the sink sat at -300 from 37 to 16 ft
/// with 0.02 to 0.07 of pull and grew to -416 at the wheels while the pull
/// crept to 0.13, too late by the dead time.
///
/// So the law is a prediction, not a correction. Each tick it runs the
/// model forward `horizon_s` from now: the elevator already written arrives
/// on schedule, the ground term grows as the predicted height falls, and a
/// single new elevator is held from the moment it can arrive. It writes the
/// elevator that puts the MODEL's sink, at the horizon, on the flare's
/// target for the height the aircraft will have then. What the model does
/// not explain (the trim the autopilot left, the thrust, the speed, a gust)
/// is measured every tick as the sink less the model's own answer
/// (`unexplained`) and carried through the prediction unchanged, so a
/// wrong trim at the gate (the LEMG 12 approach arrived at -774 on a trim
/// that held -470) is learned a dead time later instead of assumed. The
/// dead time is met with LEAD, the future written into the ask, never with
/// gain. Never a push past the airframe's `max_push`, never a pull past its
/// `max`.
///
/// IN ROUGH AIR THE COLUMN ONLY COMES BACK in the flare (the Flare phase;
/// the roundout above it stays free both ways): the prediction's ask is the
/// least pull written, never less than the pull already held. Free all the
/// way down, the model chased the air mass: LEMG 13, 2026-09-28 (sortie
/// 1790589309, the final's sink straying 72 fpm rms from the one asked for
/// against 10 to 23 on the ten calm finals before it), a downdraft of about
/// 130 fpm over 30 to 20 ft had it pull 0.39 at 25 ft, the air came back,
/// the unexplained sink swung up with it, and the prediction eased the
/// column 0.39 to 0.08 by 16 ft while still sinking 415 to 246; the ease
/// arrived a dead time later in a -69 fpm FLOAT at 15 ft, the sink rebuilt
/// faster than full pull could come back, and the wheels met the runway at
/// -422 fpm. The gust and a strong elevator look the same for the seconds
/// that matter, and in SMOOTH air the free prediction is what lands a strong
/// one: through the 17:25 flare of 2026-09-26 (6,429 fpm per unit) in still
/// air, the column held from the flare gate ballooned four of the eight
/// recorded gates and dropped the worst in at 506, where the prediction
/// landed all eight under 305. So smooth air
/// keeps the prediction and holds only a near-level float
/// ([`FLARE_HOLD_MIN_SINK_FPM`]); rough air ([`FinalAir`]) holds from the
/// flare gate. Either way a float is not a balloon: only a measured CLIMB
/// (past [`BALLOON_CLIMB_FPM`], never a +1 fpm float) lets the held pull go, eased at the airframe's `balloon_ease_per_s`
/// toward the ask, never a push, and more pull comes whenever the prediction
/// of the sink at the horizon asks for it.
#[derive(Debug, Clone)]
pub struct FlareModel {
    cfg: crate::aircraft::FlareModelProfile,
    max: f64,
    min: f64,
    tau_s: f64,
    /// In a climb the held pull eases toward the ask at most this fast
    /// (elevator per second); `None` lets go at once.
    balloon_ease_per_s: Option<f64>,
    /// The pull the flare holds (the ratchet), `None` above the flare.
    held: Option<f64>,
    /// Model time, seconds, and every elevator written with its time, for
    /// the dead time.
    t: f64,
    written: std::collections::VecDeque<(f64, f64)>,
    /// The model's answer to the elevator and the ground (two lag states).
    b1: f64,
    b2: f64,
    /// The elevator the model believes is on the channel.
    el: f64,
    /// The sink the model does not explain, smoothed (a RATE plant: the
    /// change of sink it does not explain, fpm per second).
    unexplained: Option<f64>,
    /// A rate plant's last measured sink and the model's own rate over the
    /// last advance, for the rate the model does not explain.
    last_vvi: Option<f64>,
    model_rate: f64,
    /// Seconds since the last write (a failed read coasts).
    coasting_s: f64,
    /// The airspeed last read and its smoothed trend (kt/s), for a model
    /// that carries the speed (`fpm_per_kt`).
    last_ias: Option<f64>,
    ias_trend: f64,
    /// A climb past this eases the held pull ([`BALLOON_CLIMB_FPM`]; a
    /// replay of the law as flown before issue #146 sets it to zero).
    climb_fpm: f64,
}

/// One tick of [`FlareModel`], for the blackbox as much as the channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlareModelOut {
    /// The elevator to write.
    pub elev: f64,
    /// The sink the law asked for at the horizon, fpm.
    pub target_fpm: f64,
    /// The height predicted at the horizon, ft.
    pub ahead_ft: f64,
    /// The sink the model does not explain, fpm (a rate plant: fpm per
    /// second).
    pub unexplained_fpm: f64,
    /// The elevator the prediction asked for, before the flare's ratchet.
    pub ask: f64,
}

/// The model's own integration step, seconds.
const FLARE_MODEL_STEP_S: f64 = 0.02;
/// The prediction's step, seconds.
const FLARE_PREDICT_STEP_S: f64 = 0.05;
/// The airspeed trend a speed-carrying model leads with is smoothed over
/// this, seconds.
const FLARE_SPEED_TREND_TAU_S: f64 = 1.0;
/// The pitch channel expires this long after its last write.
const FLARE_CHANNEL_EXPIRY_S: f64 = 1.0;

impl FlareModel {
    /// The law for this airframe's model, pull cap, push cap (negative) and
    /// flare time constant.
    pub fn new(cfg: crate::aircraft::FlareModelProfile, max: f64, min: f64, tau_s: f64) -> Self {
        // A STANDING plant never takes the column forward of neutral in the
        // roundout or the flare: the arrest leaves the sink shallower than
        // the exponential asks, the model pushed to catch it up, and the
        // nose-down it stored arrived after its own lag, at the wheels.
        // King Air LEMG 13 (iMac 2026-09-30): no push, 93 kt and 299 fpm
        // (1790783744); pushed from 48 and 61 ft at -0.05, 103 and 109 kt,
        // 349 and 367 fpm, both skipped (1790783945, 1790784149). The
        // Evolution at LILB pushed from 52 ft at 361 fpm, the sink built to
        // 576 and it landed at 570 (1790781114, #196). A float is the
        // retard's (`FlarePower`) and the commit's, never the column's. Only
        // a RATE plant, whose neutral HOLDS a sink, pushes (`step_rate`).
        let min = if cfg.rate { min } else { min.max(0.0) };
        Self {
            cfg,
            max,
            min,
            tau_s: if tau_s.is_finite() && tau_s > 0.5 {
                tau_s
            } else {
                5.0
            },
            balloon_ease_per_s: None,
            held: None,
            t: 0.0,
            written: std::collections::VecDeque::new(),
            b1: 0.0,
            b2: 0.0,
            el: 0.0,
            unexplained: None,
            last_vvi: None,
            model_rate: 0.0,
            coasting_s: 0.0,
            last_ias: None,
            ias_trend: 0.0,
            climb_fpm: BALLOON_CLIMB_FPM,
        }
    }

    /// A climb in the flare eases the held pull at most this fast (elevator
    /// per second; the profile's `balloon_ease_per_s`).
    pub fn with_balloon_ease(mut self, per_s: Option<f64>) -> Self {
        self.balloon_ease_per_s = per_s.filter(|r| r.is_finite() && *r > 0.0);
        self
    }

    /// From the airframe's `flare_elevator:`; `None` unless it declares a
    /// model (every other airframe keeps [`FlarePitch`]).
    pub fn from_profile(p: Option<crate::aircraft::FlareElevatorProfile>) -> Option<Self> {
        let p = p.and_then(|p| p.valid())?;
        let gains = FlareGains::from_profile(Some(p));
        p.model.map(|m| {
            Self::new(m, gains.max, gains.min, p.tau_s.unwrap_or(5.0))
                .with_balloon_ease(gains.balloon_ease_per_s)
        })
    }

    /// A new approach: nothing written, nothing owed, nothing held.
    pub fn reset(&mut self) {
        let climb_fpm = self.climb_fpm;
        *self = Self::new(self.cfg, self.max, self.min, self.tau_s)
            .with_balloon_ease(self.balloon_ease_per_s);
        self.climb_fpm = climb_fpm;
    }

    /// The law as flown before issue #146: any climb, +1 fpm included,
    /// eased the held pull. Replays only.
    #[cfg(test)]
    fn easing_on_any_climb(mut self) -> Self {
        self.climb_fpm = 0.0;
        self
    }

    fn lag2(&self) -> f64 {
        self.cfg.lag2_s.unwrap_or(self.cfg.lag_s)
    }

    fn horizon(&self) -> f64 {
        self.cfg.horizon_s.unwrap_or(self.cfg.dead_s + 1.0)
    }

    /// The sink the ground adds at this height, fpm (positive).
    fn ground(&self, h: f64) -> f64 {
        self.cfg.ground_fpm * (1.0 - h.max(0.0) / self.cfg.ground_ft).max(0.0)
    }

    /// The elevator arriving at the aircraft at model time `t`: the last
    /// one written a dead time before it.
    fn arriving(&self, t: f64) -> f64 {
        let due = t - self.cfg.dead_s;
        self.written
            .iter()
            .take_while(|(tw, _)| *tw <= due + 1e-9)
            .last()
            .map_or(0.0, |(_, e)| *e)
    }

    /// Advance the model `dt` at height `h`.
    fn advance(&mut self, dt: f64, h: f64) {
        let n = (dt / FLARE_MODEL_STEP_S).round().max(1.0) as usize;
        let step = dt / n as f64;
        let (k, l1, l2) = (self.cfg.fpm_per_elevator, self.cfg.lag_s, self.lag2());
        let ground = self.ground(h);
        let rate = self.cfg.rate;
        let mut rate_sum = 0.0;
        for _ in 0..n {
            self.t += step;
            self.written.push_back((self.t, self.el));
            // Keep one entry at or before the dead-time boundary.
            let due = self.t - self.cfg.dead_s;
            while self.written.len() > 1 && self.written[1].0 <= due {
                self.written.pop_front();
            }
            let arriving = self.arriving(self.t);
            if rate {
                // The lags carry the RATE the column asks; the ground's
                // own rate is taken off it, not lagged.
                self.b1 += (k * arriving - self.b1) * step / l1;
                self.b2 += (self.b1 - self.b2) * step / l2;
                rate_sum += self.b2 - ground;
            } else {
                self.b1 += (k * arriving - ground - self.b1) * step / l1;
                self.b2 += (self.b1 - self.b2) * step / l2;
            }
        }
        if rate {
            self.model_rate = rate_sum / n as f64;
        }
    }

    /// The airspeed this tick, for a model that carries the speed
    /// (`fpm_per_kt`): its trend is smoothed over [`FLARE_SPEED_TREND_TAU_S`].
    /// Nothing else reads it.
    pub fn note_speed(&mut self, ias_kts: f64, dt_s: f64) {
        if !ias_kts.is_finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            return;
        }
        if let Some(p) = self.last_ias {
            let raw = ((ias_kts - p) / dt_s).clamp(-6.0, 3.0);
            self.ias_trend += (raw - self.ias_trend) * (dt_s / FLARE_SPEED_TREND_TAU_S).min(1.0);
        }
        self.last_ias = Some(ias_kts);
    }

    /// The sink the speed's trend adds over the next `ahead_s` (fpm,
    /// negative while it bleeds); nothing without `fpm_per_kt`.
    fn speed_sink_ahead(&self, ahead_s: f64) -> f64 {
        self.cfg
            .fpm_per_kt
            .map_or(0.0, |k| k * self.ias_trend.min(0.0) * ahead_s.max(0.0))
    }

    /// The model's sink and height at the horizon if `el_new` were held
    /// from now, the elevator already written arriving on schedule.
    fn predict(&self, el_new: f64, radio_ft: f64, vvi_fpm: f64, unexplained: f64) -> (f64, f64) {
        let horizon = self.horizon();
        let n = (horizon / FLARE_PREDICT_STEP_S).round().max(1.0) as usize;
        let dt = horizon / n as f64;
        let (k, l1, l2) = (self.cfg.fpm_per_elevator, self.cfg.lag_s, self.lag2());
        let (mut b1, mut b2, mut t, mut h, mut v) = (self.b1, self.b2, self.t, radio_ft, vvi_fpm);
        for _ in 0..n {
            t += dt;
            let arriving = if t - self.cfg.dead_s <= self.t {
                self.arriving(t)
            } else {
                el_new
            };
            if self.cfg.rate {
                // From the sink measured now: the column's rate, less the
                // ground's, plus the rate nothing explains.
                b1 += (k * arriving - b1) * dt / l1;
                b2 += (b1 - b2) * dt / l2;
                v += (b2 - self.ground(h) + unexplained) * dt;
            } else {
                b1 += (k * arriving - self.ground(h) - b1) * dt / l1;
                b2 += (b1 - b2) * dt / l2;
                // The speed still to bleed, as sink, from now to then.
                v = unexplained + b2 + self.speed_sink_ahead(t - self.t);
            }
            h = (h + v / 60.0 * dt).max(0.0);
        }
        (v, h)
    }

    /// No measurement this tick: nothing is decided, the model keeps time
    /// with what is on the channel, and the channel itself runs out after
    /// [`FLARE_CHANNEL_EXPIRY_S`].
    pub fn coast(&mut self, dt_s: f64, radio_ft: f64) {
        let dt = dt_s.clamp(0.02, 3.0);
        self.advance(dt, radio_ft);
        self.coasting_s += dt;
        // A rate plant's next change of sink spans the gap: no measurement.
        self.last_vvi = None;
        if self.coasting_s >= FLARE_CHANNEL_EXPIRY_S {
            self.el = 0.0;
        }
    }

    /// The flare's target at a height and groundspeed: the exponential
    /// flare to the touchdown sink, never steeper than the path. Pure.
    pub fn target_fpm(&self, radio_ft: f64, gs_kts: f64) -> f64 {
        let nominal = -(gs_kts.max(0.0) / 60.0) * gp_ft_per_nm();
        let (_, arrest) = arrest_targets_fpm(nominal);
        let touchdown = self.cfg.touchdown_fpm.map_or(arrest, |t| -t);
        // A HOLD-OFF levels at its height, then settles: the same
        // exponential, measured from the hold-off height, not the runway.
        let above = match self.cfg.hold_off_ft {
            Some(hold) => (radio_ft - hold).max(0.0),
            None => radio_ft,
        };
        flare_target_with(above, touchdown, nominal, Some(self.tau_s))
    }

    /// The airframe flies a light single's hold-off (`hold_off_ft`).
    pub fn holds_off(&self) -> bool {
        self.cfg.hold_off_ft.is_some()
    }

    /// One tick on a RATE plant (`FlareModelProfile::rate`, the A330's
    /// sidestick): the elevator whose rate, held from when it can arrive,
    /// puts the sink at the horizon on the flare's target for the height it
    /// will have then. The rate the model does not explain (the speed
    /// bleeding off at idle, the trim the autopilot left) is measured from
    /// the sink's own change and carried through the prediction. Neutral
    /// HOLDS a sink on this plant, so there is no ratchet and no "never
    /// push a climb": the column comes forward to end a float or a balloon,
    /// as far as the airframe's `max_push`. The model's pipe carries what
    /// was written, as on the standing plant.
    fn step_rate(&mut self, radio_ft: f64, gs_kts: f64, vvi_fpm: f64, dt: f64) -> FlareModelOut {
        let unexplained = match self.last_vvi {
            Some(prev) => {
                let now = (vvi_fpm - prev) / dt - self.model_rate;
                match self.unexplained {
                    Some(u) => u + (now - u) * (dt / self.cfg.observer_s).min(1.0),
                    None => now,
                }
            }
            None => self.unexplained.unwrap_or(0.0),
        };
        self.last_vvi = Some(vvi_fpm);
        self.unexplained = Some(unexplained);
        let (v0, ahead) = self.predict(0.0, radio_ft, vvi_fpm, unexplained);
        let (v1, _) = self.predict(1.0, radio_ft, vvi_fpm, unexplained);
        let target = self.target_fpm(ahead, gs_kts);
        let per_unit = v1 - v0;
        let ask = if per_unit > 1e-6 {
            ((target - v0) / per_unit).clamp(self.min, self.max)
        } else {
            0.0
        };
        self.held = None;
        self.el = ask;
        FlareModelOut {
            elev: ask,
            target_fpm: target,
            ahead_ft: ahead,
            unexplained_fpm: unexplained,
            ask,
        }
    }

    /// One tick: the elevator for this measured sink. `flare` is the flare
    /// proper (as against the roundout above it, which stays free both
    /// ways); `rough` is the final's air ([`FinalAir::rough`]). See
    /// [`FlareModel`] for what each holds.
    pub fn step(
        &mut self,
        radio_ft: f64,
        gs_kts: f64,
        vvi_fpm: f64,
        dt_s: f64,
        flare: bool,
        rough: bool,
    ) -> FlareModelOut {
        let dt = dt_s.clamp(0.02, 3.0);
        self.advance(dt, radio_ft);
        self.coasting_s = 0.0;
        if self.cfg.rate {
            return self.step_rate(radio_ft, gs_kts, vvi_fpm, dt);
        }
        let now = vvi_fpm - self.b2;
        let unexplained = match self.unexplained {
            Some(u) => u + (now - u) * (dt / self.cfg.observer_s).min(1.0),
            None => now,
        };
        self.unexplained = Some(unexplained);
        // Linear in the new elevator: two predictions give the answer.
        let (v0, ahead) = self.predict(0.0, radio_ft, vvi_fpm, unexplained);
        let (v1, _) = self.predict(1.0, radio_ft, vvi_fpm, unexplained);
        let target = self.target_fpm(ahead, gs_kts);
        let per_unit = v1 - v0;
        let ask = if per_unit > 1e-6 {
            ((target - v0) / per_unit).clamp(self.min, self.max)
        } else {
            0.0
        };
        let elev = if flare {
            // The roundout's last ask is where the flare starts.
            let held = self.held.unwrap_or(ask);
            let next = if vvi_fpm > self.climb_fpm {
                // A BALLOON, measured: the pull has beaten the ground and the
                // aircraft climbs. Eased toward the ask, never a push. A
                // near-level float is not one: +1 fpm at 22.9 ft eased the
                // held 0.28 to 0.20 at EGBB 15 (2026-09-28), a dead time
                // before the sink the retard would rebuild.
                let eased = ask.max(0.0);
                match self.balloon_ease_per_s {
                    Some(rate) => eased.max(held - rate * dt),
                    None => eased,
                }
            } else if (rough && self.cfg.rough_hold != Some(false))
                || vvi_fpm > -FLARE_HOLD_MIN_SINK_FPM
            {
                // Rough air, or a near-level float: the pull stays, and more
                // comes only when the prediction asks for it.
                ask.max(held)
            } else {
                // Smooth air and a real sink: the prediction flies it.
                ask
            };
            self.held = Some(next);
            next
        } else {
            self.held = None;
            // The roundout is free both ways, except that a measured climb
            // is a balloon and a balloon never pushes (LEMG 12, the SF50,
            // 2026-09-28: -0.05 at +214 fpm).
            if vvi_fpm > self.climb_fpm {
                ask.max(0.0)
            } else {
                ask
            }
        };
        // The model's pipe carries what was WRITTEN, the held pull included.
        self.el = elev;
        FlareModelOut {
            elev,
            target_fpm: target,
            ahead_ft: ahead,
            unexplained_fpm: unexplained,
            ask,
        }
    }
}

/// The final's air, as the autopilot felt it: how far the sink strays from
/// the one asked for (fpm rms about its own mean, so a steady tracking
/// offset is not turbulence), from [`FINAL_AIR_FROM_FT`] down to the arrest
/// gate, only while the autopilot is confirmed. The ten calm LEMG 13 finals
/// of 2026-09-28 strayed 10 to 23 fpm; the three gusty ones after them 72,
/// 83 and 105, and the 2026-09-26 finals in 8 to 12 kt of swinging wind 29
/// to 110. Too few samples (a close-in reposition, failed reads) is no
/// measurement, and no measurement is smooth air: the flare the calm
/// landings flew.
#[derive(Debug, Default, Clone, Copy)]
pub struct FinalAir {
    n: u32,
    sum: f64,
    sum_sq: f64,
}

/// The final's air is measured from this radio height down, ft.
pub const FINAL_AIR_FROM_FT: f64 = 400.0;
/// ...needs this many samples (about five seconds of ticks)...
const FINAL_AIR_MIN_SAMPLES: u32 = 15;
/// ...and is rough past this, fpm rms.
pub const ROUGH_AIR_FPM: f64 = 40.0;

impl FinalAir {
    /// One tick's sink against the one asked for; a read that failed is no
    /// sample.
    pub fn note(&mut self, vvi_fpm: f64, asked_fpm: f64) {
        let e = vvi_fpm - asked_fpm;
        if e.is_finite() {
            self.n += 1;
            self.sum += e;
            self.sum_sq += e * e;
        }
    }

    /// The stray, fpm rms about its mean; `None` with too few samples.
    pub fn rms_fpm(&self) -> Option<f64> {
        if self.n < FINAL_AIR_MIN_SAMPLES {
            return None;
        }
        let n = f64::from(self.n);
        let mean = self.sum / n;
        Some((self.sum_sq / n - mean * mean).max(0.0).sqrt())
    }

    pub fn rough(&self) -> bool {
        self.rms_fpm().is_some_and(|r| r > ROUGH_AIR_FPM)
    }
}

/// Who has the servos this tick. The arrest flies the channels with the
/// autopilot's servos OFF (the pitch servo writes the yoke back over the
/// channel otherwise; LFPG 09R, 2026-09-22), takes them at the roundout
/// gate and hands them back only the tick the arrest ends in a go-around
/// (a balk, the missed approach, a pattern): a balloon back over the band
/// is still the arrest ([`hold_roundout`]; LEMG 12, 2026-09-28, handed back
/// at 50.5 ft and dived to -814 fpm). The rollout latch never reaches this decision:
/// it disconnects on its own and finds the servos already off.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArrestHands {
    /// The autopilot flies; nothing to do.
    Off,
    /// This tick: servos off, the channels take over.
    Take,
    /// The arrest has the servos and keeps flying the channels.
    Keep,
    /// This tick: servos on, modes re-engaged, the channels expire.
    Return,
}

pub fn arrest_hands_step(taken: bool, arresting: bool) -> ArrestHands {
    match (taken, arresting) {
        (false, false) => ArrestHands::Off,
        (false, true) => ArrestHands::Take,
        (true, true) => ArrestHands::Keep,
        (true, false) => ArrestHands::Return,
    }
}

/// Inside this distance to the threshold the final's heading is the
/// short-final law's ([`short_final_cut_deg`]), not the pursuit's.
pub const SHORT_FINAL_NM: f64 = 0.30;
/// The short final closes the offset over this many seconds...
const SHORT_FINAL_CLOSE_S: f64 = 6.0;
/// ...judged this far ahead on the drift over the ground...
const SHORT_FINAL_LEAD_S: f64 = 3.0;
/// ...with a cut no bigger than this, degrees.
const SHORT_FINAL_MAX_CUT_DEG: f64 = 3.0;

/// The heading cut off the runway course on the last 0.3 nm, degrees (left
/// negative), from the offset (`xtk_m`, right positive) and the drift over
/// the ground (`side_m_s`, right positive, from the track).
///
/// It was 80 degrees per nm of offset and nothing else: 0.04 degrees a
/// metre, an 18 s closure with no damping, where a degree of track error
/// is 1.3 m/s at 140 kt. LEMG 13 (2026-09-26, 20:17Z): from 170 ft the 737
/// tracked about a degree right of the runway (the track trim still held
/// +0.45 degrees learnt against an earlier left drift, and the 15 s crab
/// filter was still holding the wind of half a minute before, while the
/// wind swung round), the law answered 8.9 m of offset with 0.4 degrees,
/// and the arrest gate at 79 ft found it 9 to 12 m right. Here the offset
/// closes in [`SHORT_FINAL_CLOSE_S`] and the drift is counted
/// [`SHORT_FINAL_LEAD_S`] ahead, so a track error is answered the moment it
/// shows, not once it has become metres. Pure.
pub fn short_final_cut_deg(xtk_m: f64, side_m_s: f64, gs_kts: f64) -> f64 {
    if !xtk_m.is_finite() || !side_m_s.is_finite() {
        return 0.0;
    }
    let v = (if gs_kts.is_finite() { gs_kts } else { 0.0 } * 0.514_444).max(30.0);
    let ahead = xtk_m + side_m_s * SHORT_FINAL_LEAD_S;
    (-(ahead / (v * SHORT_FINAL_CLOSE_S)).atan().to_degrees())
        .clamp(-SHORT_FINAL_MAX_CUT_DEG, SHORT_FINAL_MAX_CUT_DEG)
}

/// How quickly the short final learns what the track does against its bug,
/// seconds.
const SHORT_FINAL_BIAS_TAU_S: f64 = 2.0;

/// The short final's bug, as an offset from the runway course (degrees):
/// the [`short_final_cut_deg`] track it wants, less what the track has been
/// doing against the bug it was given (the autopilot's own heading offset,
/// a wind the 15 s crab filter has not caught up with, a stale track trim),
/// learnt over [`SHORT_FINAL_BIAS_TAU_S`]. On the replayed LEMG 13 ticks a
/// law without it still reached the gate 5 m right: the bug-to-track error
/// was a steady degree, and a P law holds a steady offset against a steady
/// error however it is damped. Seeded with the bug the pursuit was flying,
/// so the hand-over is continuous.
///
/// What the track does against the bug is only a BIAS while the aircraft is
/// flying that bug steadily. Learnt while it is still turning onto it, the
/// autopilot's own lag reads as bias: LEMG 13 (sortie 1790591256-full_auto,
/// 2026-09-28) the short final began 32 m left at 0.29 nm, the cut went to
/// its 3 degree cap, the track lagged it by 5.4 degrees a tick later, the 2 s
/// learner took that for a bias and ran to its 5 degree clamp, and the bug
/// stood 8 degrees right of the runway: 16 degrees of bank at 90 ft, across
/// the centreline at the threshold and 37 m right at the wheels. The same
/// shape at 11 degrees of bank in the circuit before it (1790590663). So a
/// tick banked past [`SHORT_FINAL_LEARN_BANK_DEG`], or one whose track is
/// more than [`SHORT_FINAL_LEARN_ERR_DEG`] from what the bias already
/// explains, teaches nothing, and the bias is bounded by what one really was
/// (a degree, 2026-09-26) with room: [`SHORT_FINAL_MAX_BIAS_DEG`].
#[derive(Debug, Clone, Copy)]
pub struct ShortFinal {
    bias_deg: f64,
    prev_bug_deg: f64,
}

/// The most bias the short final carries, degrees.
const SHORT_FINAL_MAX_BIAS_DEG: f64 = 2.5;
/// Banked past this, the track is turning and teaches no bias, degrees.
const SHORT_FINAL_LEARN_BANK_DEG: f64 = 5.0;
/// A track further than this from the bug plus the bias learnt is a turn
/// still under way, not a bias, degrees.
const SHORT_FINAL_LEARN_ERR_DEG: f64 = 3.0;

impl ShortFinal {
    /// `entry_bug_off_deg`: the bug (true) less the course on the tick the
    /// short final begins; `track_off_deg`: the track less the course then.
    pub fn new(entry_bug_off_deg: f64, track_off_deg: f64) -> Self {
        let seed = track_off_deg - entry_bug_off_deg;
        Self {
            bias_deg: if seed.is_finite() {
                seed.clamp(-SHORT_FINAL_MAX_BIAS_DEG, SHORT_FINAL_MAX_BIAS_DEG)
            } else {
                0.0
            },
            prev_bug_deg: entry_bug_off_deg,
        }
    }

    /// One tick: the bug offset from the course, degrees. `bank_deg` is the
    /// bank read this tick (`None` when unread: judged on the track alone).
    pub fn step(
        &mut self,
        xtk_m: f64,
        track_off_deg: f64,
        gs_kts: f64,
        dt_s: f64,
        bank_deg: Option<f64>,
    ) -> f64 {
        if !track_off_deg.is_finite() || !xtk_m.is_finite() {
            return self.prev_bug_deg;
        }
        let dt = dt_s.clamp(0.05, 1.0);
        let e = track_off_deg - self.prev_bug_deg;
        let turning = bank_deg.is_some_and(|b| b.abs() > SHORT_FINAL_LEARN_BANK_DEG)
            || (e - self.bias_deg).abs() > SHORT_FINAL_LEARN_ERR_DEG;
        if !turning {
            let a = (dt / SHORT_FINAL_BIAS_TAU_S).min(1.0);
            self.bias_deg = (self.bias_deg + (e - self.bias_deg) * a)
                .clamp(-SHORT_FINAL_MAX_BIAS_DEG, SHORT_FINAL_MAX_BIAS_DEG);
        }
        let v = (gs_kts.max(0.0) * 0.514_444).max(30.0);
        let side = v * track_off_deg.to_radians().sin();
        let bug = short_final_cut_deg(xtk_m, side, gs_kts) - self.bias_deg;
        self.prev_bug_deg = bug;
        bug
    }
}

/// The closure the arrest asks for: the offset over this many seconds.
const ARREST_CLOSE_S: f64 = 4.0;
/// ...and never faster than this sideways, m/s.
const ARREST_CLOSE_MAX_M_S: f64 = 2.5;
/// How quickly a sideways-velocity error is taken out by the bank, seconds.
const ARREST_VEL_TAU_S: f64 = 1.5;
/// The steepest bank over the runway, degrees, tapering toward the wheels.
const ARREST_BANK_MAX_DEG: f64 = 5.0;
const ARREST_BANK_WHEELS_DEG: f64 = 2.0;
const G_M_S2: f64 = 9.81;
/// The sideways deceleration the closure is sized to be stopped by, m/s^2:
/// well inside the 0.5 a 3-degree bank gives, since the bank takes seconds
/// to get there.
const ARREST_STOP_M_S2: f64 = 0.2;
/// How far ahead the current bank's push on the drift is counted, seconds:
/// the 737's roll lag at the arrest's aileron (LEMG 13, 2026-09-26).
const ARREST_BANK_LEAD_S: f64 = 1.0;
/// Bank error for full aileron in the arrest (see `WingsLevel::stiff`).
pub(crate) const ARREST_FULL_AIL_BANK_DEG: f64 = 6.0;

/// The roll loop the arrest flies its bank target through: the airframe's
/// declared roll-rate plant (`arrest_roll:`, the A330), else the stiff law.
pub(crate) fn arrest_wings() -> crate::autotakeoff::WingsLevel {
    match crate::aircraft::arrest_roll() {
        Some(p) => crate::autotakeoff::WingsLevel::rate_plant(p),
        None => crate::autotakeoff::WingsLevel::stiff(ARREST_FULL_AIL_BANK_DEG),
    }
}

/// The bank the arrest flies with the servos out, from the centreline offset
/// (`xtk_m`, right positive) and the SIDEWAYS VELOCITY over the runway
/// (`side_m_s`, right positive: the ground track's angle off the course
/// times the groundspeed). A velocity loop inside a position loop: the offset
/// asks for a closure rate, the bank takes out the difference.
///
/// It flew a HEADING (the rollout's centreline law plus the crab), and over
/// the runway a heading is the wrong thing to fly. The rollout law is sized
/// for a closure time of about nine seconds at 140 kt, longer than the whole
/// arrest; and the 737's nose did not follow its bank: LEMG 13 (2026-09-26,
/// 17:43Z) went into the gate 3.9 m right and crossing left at 2.3 m/s, the
/// bank built to 4.6 degrees right while the heading fell from 130.5 to
/// 129.1, the track came right only 1 degree, and it touched down 15.3 m
/// left and rolled out 16.8 m left. The same crossing flew four landings in
/// a row (100 ft to the wheels: +5.6 to -11.9, +5.9 to -14.7, +13.0 to
/// -5.6, +7.2 to -10.4). The track is what moves the aircraft over the
/// paint, so the track is what this closes.
///
/// The closure goes to ZERO at the centreline and the bank's lag is led.
/// LEMG 13 (2026-09-26, 20:17Z) went into the gate 11.9 m right: the old
/// capped rate (the offset over 4 s, at most 2.5 m/s) built the drift to
/// 2.65 m/s, the bank target reversed at 42 ft but the 737 rolls at about
/// 2 deg/s on the aileron `WingsLevel` gives it, so the bank only crossed
/// wings level 2 s later with the aircraft still 2.4 m right at 2.65 m/s;
/// the shrinking cap then allowed 3 degrees to stop that, and it touched
/// down 6.5 m left. So the wanted rate is also what a gentle deceleration
/// ([`ARREST_STOP_M_S2`]) can take off in the offset left, and the drift
/// the bank is judged against is the one the CURRENT bank will have made
/// [`ARREST_BANK_LEAD_S`] from now.
pub fn arrest_bank_target(xtk_m: f64, side_m_s: f64, phi_deg: f64, radio_ft: f64) -> f64 {
    if !xtk_m.is_finite() || !side_m_s.is_finite() {
        return 0.0;
    }
    let phi = if phi_deg.is_finite() { phi_deg.clamp(-30.0, 30.0) } else { 0.0 };
    let off = xtk_m.abs();
    let rate = (off / ARREST_CLOSE_S)
        .min((2.0 * ARREST_STOP_M_S2 * off).sqrt())
        .min(ARREST_CLOSE_MAX_M_S);
    let want = -xtk_m.signum() * rate;
    let side_ahead = side_m_s + G_M_S2 * phi.to_radians().tan() * ARREST_BANK_LEAD_S;
    let bank = ((want - side_ahead) / (G_M_S2 * ARREST_VEL_TAU_S)).atan().to_degrees();
    let cap = if radio_ft.is_finite() {
        (ARREST_BANK_WHEELS_DEG + radio_ft.max(0.0) / 10.0).min(ARREST_BANK_MAX_DEG)
    } else {
        ARREST_BANK_WHEELS_DEG
    };
    bank.clamp(-cap, cap)
}

/// The sideways velocity over the runway, m/s right positive, from the sim's
/// ground track (`hpath`, true) against the runway course (true). `None`
/// without a track read: no measurement, and the caller falls back to the
/// smoothed cross-track rate.
pub fn side_velocity_m_s(track_true: Option<f64>, course_true: f64, gs_kts: f64) -> Option<f64> {
    let trk = track_true.filter(|t| t.is_finite())?;
    if !gs_kts.is_finite() {
        return None;
    }
    let off = (((trk - course_true + 540.0).rem_euclid(360.0)) - 180.0).to_radians();
    Some(gs_kts * 0.514_444 * off.sin())
}

/// Outside a landing arrest (`arresting` false) the dial is the target, as it
/// always was. Inside one it leads the target by the autopilot's outstanding
/// error - see [`VS_LEAD_GAIN`] - against the sink PREDICTED `trend_seconds`
/// ahead (`trend_fpm_s`, the vertical speed's rate of change): on a
/// second-order autopilot the trend is the damping the bare error lacks.
/// The gain, the cap and the trend horizon are the airframe's
/// ([`crate::aircraft::ArrestLeadProfile`]; the default is the 172's).
pub fn vs_dial_fpm(
    target_fpm: f64,
    actual_fpm: f64,
    trend_fpm_s: f64,
    arresting: bool,
    lead: crate::aircraft::ArrestLeadProfile,
) -> f64 {
    if !arresting {
        return target_fpm;
    }
    let trend = if trend_fpm_s.is_finite() {
        trend_fpm_s
    } else {
        0.0
    };
    let predicted = actual_fpm + lead.trend_seconds * trend;
    (target_fpm + lead.gain * (target_fpm - predicted)).clamp(-1200.0, lead.max_climb_fpm)
}

/// May the roundout own the descent from HERE? Only where a
/// [`ROUNDOUT_VS_FPM`] descent still REACHES the pavement. At or past the
/// threshold, always — there the unsigned glidepath distance grows again and
/// alt_err stops meaning anything, which is the arithmetic the roundout
/// exists to pre-empt. Short of it, the sink from this height must cover the
/// metres to the threshold: 40 ft at 0.2 nm closes (a normal crossing), and
/// 30 ft at 0.3 nm does not — that aircraft is dragged low, and a commanded
/// descent grounds it ~190 m into the approach lights where the level-hold
/// would have let the path come down to it (audit, 2026-08-02). A failed
/// (zero) groundspeed read denies the roundout, which falls back to the
/// level-hold: short of the runway, level is always the safe reading.
fn roundout_reaches_pavement(along_nm: f64, radio_alt_ft: f64, gs_kts: f64) -> bool {
    if along_nm <= 0.0 {
        return true;
    }
    let sink_s = radio_alt_ft / (ROUNDOUT_VS_FPM.abs() / 60.0);
    let reach_m = gs_kts * 0.514_444 * sink_s;
    along_nm * M_PER_NM <= reach_m
}
/// ...and only within this much cross-track of the centreline (~37 m — about a
/// runway half-width plus margin). Beyond it the aircraft is over grass, and
/// touching down there is how the LXGB 09 rollout ended in wreckage
/// (2026-07-22). A balked landing straight ahead is always the better trade.
const FLARE_MAX_XTK_NM: f64 = 0.02;

/// How far inside the pavement's edge a landing's centreline offset must stay,
/// metres: a jet's main gear (the 737's is 5.7 m across, 2.9 m either side)
/// plus a margin for the arrest's last closure.
const LANDING_EDGE_MARGIN_M: f64 = 7.5;
/// ...and how close to the edge the flare may still be at the flare gate: the
/// gear on the pavement with a metre to spare.
const FLARE_EDGE_MARGIN_M: f64 = 4.0;

/// The most centreline offset a landing may carry to the wheels on a runway
/// `width_m` wide, metres: the stabilised gates below the terrain floor and
/// the low gate judge against it. `FLARE_MAX_XTK_NM`'s 37 m is a 60 m strip's
/// half-width plus margin, and on LEMG 13's 45 m it is 14.5 m past the edge:
/// the 737 flew the last 500 ft 30 to 50 m off, the gates never fired, and it
/// touched down 37 m right of the centreline, in the grass (sortie
/// 1790591256-full_auto, 2026-09-28). Unknown width keeps the old 37 m; a
/// narrow strip keeps at least a quarter of its width.
pub fn landing_xtk_limit_m(width_m: Option<f64>) -> f64 {
    let legacy = FLARE_MAX_XTK_NM * M_PER_NM;
    match width_m.filter(|w| w.is_finite() && *w > 1.0) {
        Some(w) => ((w / 2.0 - LANDING_EDGE_MARGIN_M).max(w / 4.0)).min(legacy),
        None => legacy,
    }
}

/// The offset the FLARE gate balks beyond, metres: the gear still on the
/// pavement ([`FLARE_EDGE_MARGIN_M`] inside the edge), never tighter than the
/// landing limit, never looser than the old 37 m.
pub fn flare_xtk_limit_m(width_m: Option<f64>) -> f64 {
    let legacy = FLARE_MAX_XTK_NM * M_PER_NM;
    match width_m.filter(|w| w.is_finite() && *w > 1.0) {
        Some(w) => (w / 2.0 - FLARE_EDGE_MARGIN_M)
            .max(landing_xtk_limit_m(Some(w)))
            .min(legacy),
        None => legacy,
    }
}

/// The pavement widths read with the runways (apt.dat's `100` row, field 1),
/// keyed by field and end. A runway's width is scenery, not flight state: it
/// is noted as `nearby_runways` reads it, so the arm that found the runway
/// knows its width without a second pass over apt.dat.
static RUNWAY_WIDTHS: std::sync::Mutex<Option<std::collections::HashMap<(String, String), f64>>> =
    std::sync::Mutex::new(None);

fn width_key(icao: &str, id: &str) -> (String, String) {
    (
        icao.trim().to_ascii_uppercase(),
        id.trim().to_ascii_uppercase(),
    )
}

/// Note a runway end's pavement width, metres.
pub fn note_runway_width(icao: &str, id: &str, width_m: f64) {
    if !(width_m.is_finite() && width_m > 1.0) {
        return;
    }
    if let Ok(mut m) = RUNWAY_WIDTHS.lock() {
        m.get_or_insert_with(Default::default)
            .insert(width_key(icao, id), width_m);
    }
}

/// The pavement width of `rwy` when apt.dat gave one; `None` otherwise.
pub fn runway_width_m(rwy: &Runway) -> Option<f64> {
    RUNWAY_WIDTHS
        .lock()
        .ok()?
        .as_ref()?
        .get(&width_key(&rwy.icao, &rwy.id))
        .copied()
}

/// Inside the FAP and within this of the runway's course, an approach that
/// needs repositioning goes around first instead of turning at once.
const LATE_GO_AROUND_TRACK_DEG: f64 = 45.0;

/// Is a reposition this late a go-around? `along_nm` is the distance still
/// to the threshold along the approach (positive short of it). Pure.
pub fn late_go_around(along_nm: f64, track_off_deg: f64) -> bool {
    along_nm > 0.0 && along_nm < FAP_NM && track_off_deg.abs() < LATE_GO_AROUND_TRACK_DEG
}

/// Flap setting for the approach: half inside 6 nm, full a mile before the
/// stabilised gate ([`landing_flap_nm`], about 4 nm on a 3 degree path), but
/// full only once established ON the glidepath. While holding level BELOW the
/// path waiting to capture (`alt_err` well negative), full flaps are an energy
/// trap: the LERL 172 approach dragged along at 50 to 57 kt against a 65-kt
/// target with the throttle governor already at its ceiling, and arrived at
/// the flare with nothing left. Half flaps keep the drag survivable until the
/// path is captured.
///
/// Full used to wait for 3 nm, which on a 3 degree path IS the 1,000 ft gate:
/// every final on file reached the gate with half flap out and the speed
/// still on the half flap's floor (the 737 at 169 kt on EGBB 33, sortie
/// 1790759458-full_auto, 2026-09-30), so no final could be configured by the
/// gate (`stable_approach`, #199).
pub fn flap_schedule(dist_nm: f64, alt_err_ft: f64) -> f64 {
    if dist_nm < landing_flap_nm() && alt_err_ft > -150.0 {
        1.0
    } else if dist_nm < 6.0 {
        0.5
    } else {
        0.0
    }
}

/// The landing flap is selected this far ahead of the stabilised gate, nm:
/// time for it to run and for the speed to come back to its target before
/// the gate judges both.
const LANDING_FLAP_LEAD_NM: f64 = 1.0;

/// Where the approach selects its landing flap, nm from the threshold: a
/// mile before the instrument gate on this approach's path (3.98 nm at 3
/// degrees). Never beyond the half-flap point.
pub fn landing_flap_nm() -> f64 {
    let gate_nm = crate::stable_approach::GateKind::Instrument.gate_ft() / gp_ft_per_nm()
        - TOUCHDOWN_AIM_NM;
    (gate_nm + LANDING_FLAP_LEAD_NM).clamp(3.0, 5.5)
}

/// TRUE heading → MAGNETIC, using X-Plane's `magnetic_variation` dataref, whose
/// sign is positive-WEST: `magpsi = psi + variation`. Verified live at KPDK
/// (2026-07-14): psi 191.3 true, magpsi 196.8, variation +5.65. Subtracting
/// instead flew every autoland heading 2*var off (11 degrees at Atlanta) — the
/// whole cross-track cut vanished and the final tracked 0.4 nm left, parallel.
pub fn true_to_mag(true_deg: f64, xp_variation_deg: f64) -> f64 {
    (true_deg + xp_variation_deg).rem_euclid(360.0)
}

/// Signed distance (nm) along the runway centreline: positive = on the approach
/// side (short of the threshold, still to fly), negative = past the threshold
/// (departure side). Detects an overflight so the guidance goes around instead of
/// tracking the runway heading straight out the far end.
pub fn along_track_nm(lat: f64, lon: f64, rwy: &Runway) -> f64 {
    let brg_from_thr = bearing_deg(rwy.thr_lat, rwy.thr_lon, lat, lon);
    let dist = haversine_nm(rwy.thr_lat, rwy.thr_lon, lat, lon);
    let recip = (rwy.course_deg + 180.0) % 360.0;
    // Project threshold→aircraft onto the INBOUND (recip) direction: on the
    // approach side the aircraft bears ~recip from the threshold (cos ≈ +1),
    // past it the aircraft bears ~course_deg (cos ≈ -1).
    let ang = ((brg_from_thr - recip + 540.0) % 360.0 - 180.0).to_radians();
    dist * ang.cos()
}

/// Signed cross-track distance (nm) from the runway's extended centreline —
/// positive = right of the inbound course. Used to steer back to the centreline.
pub fn cross_track_nm(lat: f64, lon: f64, rwy: &Runway) -> f64 {
    let brg_from_thr = bearing_deg(rwy.thr_lat, rwy.thr_lon, lat, lon);
    let dist = haversine_nm(rwy.thr_lat, rwy.thr_lon, lat, lon);
    // Perpendicular offset from the extended centreline, signed right-positive
    // relative to the INBOUND landing course (the direction of flight on final).
    let ang = ((brg_from_thr - rwy.course_deg + 540.0) % 360.0 - 180.0).to_radians();
    dist * ang.sin()
}

/// Latched low go-around: a missed approach called BELOW the terrain floor is
/// flown wings-level STRAIGHT AHEAD at full power until clear of the ground —
/// never the immediate turn-back the intercept guidance commands. Seen live
/// 2026-07-14 (LXGB 09): overflew the numbers at ~57 ft, the guidance flipped
/// the heading command 178° toward the FAP, the flare branch re-fired 0.3 nm
/// PAST the threshold (`dist_nm` is unsigned, so the flare gate re-arms on the
/// departure side) and pulled the power, then speed protection commanded
/// -590 fpm at ground level — impact 200 m south of the strip. The latch owns
/// the whole low go-around: it arms when Final/Flare degrades to Intercept
/// below [`TERRAIN_FLOOR_AGL_FT`], holds through every phase flicker the raw
/// guidance produces down there, and releases at the floor, where the normal
/// intercept guidance takes over the turn back to the FAP.
/// Where a balked landing is in the missed approach. Two stages, because a
/// missed approach is a PROCEDURE, not a moment: releasing the aircraft back to
/// the ordinary intercept guidance the instant it was clear of the ground left
/// it 800 ft over the far end with the field behind it and no plan, and the
/// stateless guidance then argued with itself — see `descent_limit_fpm` for the
/// vertical half of the Luton orbit (2026-07-28) and `Reposition` for the
/// lateral half.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissedStage {
    /// Straight ahead on runway heading, climbing away from the ground. The one
    /// direction a runway promises is flyable.
    Climb,
    /// Repositioning: outbound to the join point at platform height, held there
    /// until established OUTSIDE the FAP on the approach side — which is simply
    /// where a normal approach begins.
    ///
    /// This stage is what makes the decision STICKY, and stickiness is the
    /// whole point. The ordinary guidance picks its target from the CURRENT
    /// altitude error, so at reposition height the choice flips every time the
    /// aircraft crosses the distance where the path is 500 ft below it: turn
    /// out, fly out, the error shrinks, turn back in, the error grows, turn out
    /// again. At Luton that oscillation ran between 1.1 and 4.1 nm for three
    /// and a half minutes with a landing clearance live and the tower working
    /// arrivals underneath. An outbound leg has to run to completion.
    Reposition,
}

#[derive(Debug, Default)]
pub struct GoAroundLatch {
    stage: Option<MissedStage>,
    /// How many go-arounds this approach has flown — a runaway backstop. An
    /// approach that keeps missing (arrives too fast/high to capture, as the
    /// LXGB Twin Otter did off the tight circuit) would otherwise firewall a
    /// climb, release at 800 ft, re-intercept, miss again — forever, accelerating
    /// toward Vne. After [`MAX_GO_AROUNDS`] the loop gives the aircraft back.
    attempts: u32,
    /// The reposition is being flown as a pattern by the loop
    /// ([`crate::reversal`]) and ends when the pattern does
    /// ([`GoAroundLatch::finish_reposition`]), not at the join point: the
    /// pattern's downwind passes abeam it, outbound, inside the old release
    /// test's half mile on a light single.
    hold: bool,
}

/// Give up (level off, hand to the pilot) after this many go-arounds on one
/// approach rather than loop a missed approach until the fuel runs out.
pub const MAX_GO_AROUNDS: u32 = 2;
// The counter above cannot catch a lost reposition, because a second
// go-around needs another approach to fail first. A single missed approach
// is given up when it stops making progress (`reversal::MissedWatch`,
// 2026-09-30), never on a flat clock: 300 s expired with EGHI's Baron
// compliant mid-leg (2026-08-02), and the 600 s that replaced it gave up an
// SR22 on LEMG 13's terrain-raised pattern with the path a few seconds ahead
// (iMac 2026-09-30).

impl GoAroundLatch {
    pub fn new() -> Self {
        Self::default()
    }

    /// Go-arounds flown so far this approach.
    pub fn attempts(&self) -> u32 {
        self.attempts
    }

    /// Is a missed approach being flown right now?
    pub fn active(&self) -> bool {
        self.stage.is_some()
    }

    /// ...and still in its straight-ahead climb away from the ground?
    pub fn climbing(&self) -> bool {
        self.stage == Some(MissedStage::Climb)
    }

    /// The loop flies the reposition as a pattern: hold the stage until told.
    pub fn hold_reposition(&mut self) {
        self.hold = true;
    }

    /// The pattern is complete and the aircraft is inbound on the final: the
    /// missed approach is over and the ordinary approach has it.
    pub fn finish_reposition(&mut self) {
        self.hold = false;
        if self.stage == Some(MissedStage::Reposition) {
            self.stage = None;
        }
    }

    /// A controller ORDERED a go-around: fly it, now, from wherever we are.
    ///
    /// Every other entry to a missed approach is something the aircraft
    /// noticed about itself (off the centreline at flare height, past the
    /// numbers still airborne). This one comes from outside and outranks all
    /// of it — the tower can see the runway and we cannot. Returns whether
    /// this call started one, so the caller records it once.
    ///
    /// Malaga, 2026-08-05: "cleared to land" at 13:41:06 and "GO AROUND,
    /// traffic on the runway" at 13:41:07. The order was published on the
    /// party line and SPOKEN, and nothing consumed it — the approach flew on
    /// for another seventy seconds, past a 0.5 nm same-altitude conflict, and
    /// only balked because it happened to arrive high. The clearance the
    /// order voided was never re-earned, so the aircraft then landed without
    /// one, could not vacate for want of it, and sat on the runway with an
    /// arrival inside two miles. One unread flag cost all of that.
    ///
    /// Ignored once on the ground: a landing that has already happened is not
    /// a landing to abandon, and the rollout is the safer place to be.
    pub fn order(&mut self, on_ground: bool) -> bool {
        if on_ground || self.stage.is_some() {
            return false;
        }
        self.stage = Some(MissedStage::Climb);
        self.attempts += 1;
        true
    }

    /// Advance the latch one tick: `prev`/`now` are the raw guidance phases,
    /// `radio_ft` the radio altitude, `dist_nm` the distance to the threshold,
    /// `along_nm` the signed along-track (positive = approach side),
    /// `join_nm` the [`join_point_nm`] the reposition leg is flying out to
    /// and `xtk_nm` the cross-track from the extended centreline.
    /// Returns the stage being flown, `None` when the approach is the pilot's
    /// normal one again.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        prev: Option<ApproachPhase>,
        now: ApproachPhase,
        radio_ft: f64,
        dist_nm: f64,
        along_nm: f64,
        join_nm: f64,
        xtk_nm: f64,
    ) -> Option<MissedStage> {
        match self.stage {
            // Clear of the ground — stop climbing straight ahead and start
            // repositioning. 800 ft, not the 500 ft terrain floor: the turn
            // back is the leg that crosses whatever the runway points away
            // from, and releasing at 500 put the LXGB go-around eastbound at
            // the Rock with no height in hand (2026-07-22).
            Some(MissedStage::Climb) if radio_ft >= GO_AROUND_TURN_AGL_FT => {
                self.stage = Some(MissedStage::Reposition);
            }
            // Out at the join point on the approach side: that is where an
            // approach starts, so hand back. Releasing merely OUTSIDE the FAP
            // would hand back mid-outbound-leg, pointing away from the field,
            // and leave the ordinary guidance to fly a 180 on final.
            // AT the join point, not merely abeam it: the leg homes on the
            // point, and a fast aircraft's turn-back leaves it miles beside
            // the centreline when the distance test first passes. The 737
            // was released 2.7 nm abeam at 6.7 nm, called "final" 1.9 nm off
            // (a 19 degree cut is interceptable on paper), and the pursuit's
            // 30 s time constant had 110 s of final to close 3.5 km: it
            // balked at 95 ft still 84 m off (LFLL 35L, 2026-09-17).
            Some(MissedStage::Reposition)
                if !self.hold
                    && along_nm > 0.0
                    && dist_nm >= join_nm - 0.2
                    && xtk_nm.abs() <= REPOSITION_RELEASE_XTK_NM =>
            {
                self.stage = None;
            }
            None if now == ApproachPhase::Intercept
                && matches!(prev, Some(ApproachPhase::Final | ApproachPhase::Flare))
                && radio_ft < TERRAIN_FLOOR_AGL_FT =>
            {
                self.stage = Some(MissedStage::Climb);
                self.attempts += 1;
            }
            _ => {}
        }
        self.stage
    }
}

/// Where a descent from `cur_alt_ft` meets the extended centreline: the point
/// to be established at before flying the path down, never closer than the FAP.
/// The 1.15 is margin — arriving exactly on a 3° path with no slack leaves
/// nothing for the capture.
pub fn join_point_nm(cur_alt_ft: f64, rwy_elev_ft: f64) -> f64 {
    ((cur_alt_ft - rwy_elev_ft) / gp_ft_per_nm() * 1.15).clamp(FAP_NM, 40.0)
}

/// The command a latched go-around flies.
///
/// [`MissedStage::Climb`] is runway heading (straight ahead, no turn near the
/// ground) and a bounded climb once the speed is there — level acceleration
/// first, because climbing slow bleeds energy to a stall.
///
/// [`MissedStage::Reposition`] steers OUTBOUND to the join point down the
/// extended centreline and holds platform height there. It does not chase the
/// glidepath: the path inside the join point is the one this leg is flying away
/// from, and descending toward it is what turned the Luton missed approach into
/// an orbit. Levelling at platform height means arriving at the FAP already on
/// the path, which is the whole reason the reposition altitude is defined as
/// the path height at the FAP.
///
/// Both stages ask the throttle governor for a margin over the approach speed
/// (the loop firewalls it anyway while latched; the target keeps the governor
/// sane on the release tick).
#[allow(clippy::too_many_arguments)]
pub fn missed_approach_guidance(
    rwy: &Runway,
    stage: MissedStage,
    cur_lat: f64,
    cur_lon: f64,
    cur_alt_ft: f64,
    radio_alt_ft: f64,
    dist_nm: f64,
    glidepath_alt_ft: f64,
    ias_kts: f64,
    approach_ias: f64,
    ias_trend_kts_s: f64,
    floor_kts: Option<f64>,
) -> Guidance {
    let slow = approach_ias - ias_kts > SPEED_FLOOR_MARGIN_KTS;
    let (target_heading_deg, target_vs_fpm) = match stage {
        MissedStage::Climb => (
            rwy.course_deg,
            go_around_vs_fpm(
                ias_kts,
                ias_trend_kts_s,
                approach_ias,
                floor_kts,
                radio_alt_ft,
                max_climb_fpm(),
            ),
        ),
        MissedStage::Reposition => {
            // Same join point the ordinary guidance uses, so the handback is
            // continuous.
            let join_nm = join_point_nm(cur_alt_ft, rwy.elev_ft);
            let recip = (rwy.course_deg + 180.0) % 360.0;
            let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
            let vs = if radio_alt_ft < REPOSITION_MIN_AGL_FT - 50.0 && !slow {
                ((REPOSITION_MIN_AGL_FT - radio_alt_ft) * 2.0).clamp(100.0, max_climb_fpm())
            } else {
                0.0
            };
            (bearing_deg(cur_lat, cur_lon, jlat, jlon), vs)
        }
    };
    Guidance {
        target_heading_deg,
        target_vs_fpm,
        target_ias_kts: approach_ias + CLIMB_ENERGY_BONUS_KTS,
        dist_nm,
        glidepath_alt_ft,
        phase: ApproachPhase::Intercept,
        balk: None,
        arresting: false,
    }
}

/// The flap a go-around cleans up to (the loop's go-around flap).
const GO_AROUND_FLAP: f64 = 0.5;
/// The missed approach holds go-around thrust until this high above the
/// ground ...
const GA_THRUST_HOLD_AGL_FT: f64 = 400.0;
/// ... and climbing at least this, fpm: then it is established.
const GA_CLIMB_ESTABLISHED_FPM: f64 = 500.0;
/// Established and this far over the go-around speed, the thrust comes back
/// at once to the climb floor, not a governor step a tick: at 0.04 a tick the
/// lever takes twenty seconds from 1.0, and the clean 737 gains 3 kt/s.
const GA_FAST_KTS: f64 = 10.0;
/// ...and this far under it, go-around thrust again.
const GA_SLOW_KTS: f64 = 10.0;

/// The speed a missed approach flies, kt: the give-up's pattern speed for the
/// flap out (`hand_back_speed_kts`, the clean approach speed with the flaps
/// up), at least the approach speed plus the climb margin and fast enough for
/// the go-around flap to come up (`go_around_floor_kts`, the floor of
/// [`GO_AROUND_FLAP`], plus 5), never within 10 kt of the placard of the flap
/// out, never over 250 below 10,000 ft (`alt_ft`), never under the floor of
/// the configuration flown (`config_floor_kts`, which wins). EGNX 09
/// (2026-09-30, #199): the clean 737 flew its missed approach at full power
/// from 250 to 355 kt below 1,700 ft; this is 210. Pure.
pub fn missed_approach_speed_kts(
    approach_ias: f64,
    flap_ratio: f64,
    flap_limits: &[(f64, Option<f64>)],
    go_around_floor_kts: Option<f64>,
    config_floor_kts: Option<f64>,
    alt_ft: f64,
) -> f64 {
    let placard = flap_placard_kts(flap_ratio, flap_limits).unwrap_or(f64::INFINITY);
    let mut t = hand_back_speed_kts(approach_ias, flap_ratio, flap_limits)
        .max(approach_ias + CLIMB_ENERGY_BONUS_KTS)
        .max(go_around_floor_kts.map_or(0.0, |f| f + 5.0))
        .min(placard - PLACARD_TARGET_MARGIN_KTS);
    if alt_ft < crate::tools::speedhold::SPEED_LIMIT_ALT_FT {
        t = t.min(crate::tools::speedhold::BELOW_10K_LIMIT_KTS);
    }
    crate::aircraft::floored_target(t, config_floor_kts)
}

/// The lever a missed approach's climb flies. Go-around thrust until the
/// climb is ESTABLISHED ([`GA_THRUST_HOLD_AGL_FT`] up and climbing at
/// [`GA_CLIMB_ESTABLISHED_FPM`]; an unread climb is not established), and
/// whenever [`GA_SLOW_KTS`] slow; then the speed: more than [`GA_FAST_KTS`]
/// over `target_kts`, the airframe's climb floor at once (`climb_floor`, its
/// approach thrust, else idle: the VS mode holds the climb and the excess
/// becomes height), else the speed governor's step, inside the floor and
/// full. A crew's go-around: TOGA, positive climb, gear and flap, then the
/// thrust set for the go-around speed. Pure.
pub fn missed_approach_throttle(
    throttle_now: f64,
    ias_kts: f64,
    trend_kts_s: f64,
    target_kts: f64,
    radio_ft: f64,
    vvi_fpm: Option<f64>,
    climb_floor: Option<f64>,
) -> f64 {
    let established = radio_ft >= GA_THRUST_HOLD_AGL_FT
        && vvi_fpm.is_some_and(|v| v >= GA_CLIMB_ESTABLISHED_FPM);
    if !established || ias_kts < target_kts - GA_SLOW_KTS {
        return 1.0;
    }
    let floor = climb_floor.unwrap_or(IDLE_THROTTLE).max(IDLE_THROTTLE);
    if ias_kts > target_kts + GA_FAST_KTS {
        return floor;
    }
    let trend = if trend_kts_s.is_finite() { trend_kts_s } else { 0.0 };
    (throttle_now + crate::tools::speedhold::throttle_adjust_live(target_kts - ias_kts, trend))
        .clamp(floor, 1.0)
}

/// A go-around judges the speed it is HEADING for: the airspeed this many
/// seconds ahead on its trend.
const GA_SPEED_LEAD_S: f64 = 2.0;
/// Below the floor, the sink asked per knot short of it (fpm per kt).
const GA_FPM_PER_KT: f64 = 100.0;
/// ...and never more sink than reaches the ground in this long: height is
/// traded for speed only where there is height to trade.
const GA_GROUND_S: f64 = 10.0;

/// The vertical speed a slow go-around flies: PITCH FOR SPEED FIRST (issue
/// #77). Not slow (the projected airspeed within [`SPEED_FLOOR_MARGIN_KTS`]
/// of the approach speed): `climb_fpm`. Slow but at or over the floor (1.2
/// Vs in the configuration flown, [`crate::aircraft::config_floor_kts`]):
/// level, the thrust builds the speed. Under the floor: a descent of
/// [`GA_FPM_PER_KT`] per knot short, never steeper than the ground ten
/// seconds away ([`GA_GROUND_S`]), so near the runway it is almost level and
/// on it level. The old law asked 0 fpm whatever the speed, and a VS mode
/// holds 0 fpm by pitching up until the wing stalls: EDLP 24 (2026-09-24)
/// the SF50 balked `balloon` at 52 ft climbing 860 fpm at 81 kt on idle, was
/// given +1,500 then 0, bled to 74 kt with the go-around's half flap, and
/// stalled onto the runway at -1,568 fpm. `floor_kts` unknown: the approach
/// speed's own 1.2/1.3. Pure.
pub fn go_around_vs_fpm(
    ias_kts: f64,
    trend_kts_s: f64,
    approach_ias: f64,
    floor_kts: Option<f64>,
    radio_ft: f64,
    climb_fpm: f64,
) -> f64 {
    let trend = if trend_kts_s.is_finite() {
        trend_kts_s.clamp(-10.0, 10.0)
    } else {
        0.0
    };
    let projected = ias_kts + trend * GA_SPEED_LEAD_S;
    if !(approach_ias - projected > SPEED_FLOOR_MARGIN_KTS) {
        return climb_fpm;
    }
    let floor = floor_kts
        .filter(|f| f.is_finite() && *f > 0.0)
        .unwrap_or(approach_ias * crate::aircraft::SPEED_FLOOR_RATIO / 1.3);
    let give = -(radio_ft.max(0.0) / GA_GROUND_S * 60.0);
    (GA_FPM_PER_KT * (projected - floor)).clamp(give.min(0.0), 0.0)
}

/// The go-around flap comes up only at the speed that flies it: its floor
/// (1.2 Vs in that configuration). EDLP 24 (2026-09-24): half flap at 81 kt
/// put the SF50's stall at 74, where the go-around met it. Pure.
pub fn go_around_flap_due(ias_kts: f64, floor_kts: Option<f64>) -> bool {
    match floor_kts {
        Some(f) if f.is_finite() => ias_kts >= f,
        _ => true,
    }
}

/// Autoland's intercepts, patterns and missed approach: a two-degree
/// correction at most every five seconds.
pub const AUTOLAND_INTERCEPT_DEADBAND: crate::circuit::BugDeadband =
    crate::circuit::BugDeadband { deg: 2.0, every_s: 5.0 };
/// The final outside [`SHORT_FINAL_NM`]: half a degree at most every two
/// seconds (a degree every three left the fitted 737 9.8 m off at the
/// roundout gate from a 30 m LEMG 13 handover, against 3.9 every tick).
pub const AUTOLAND_FINAL_DEADBAND: crate::circuit::BugDeadband =
    crate::circuit::BugDeadband { deg: 0.5, every_s: 2.0 };

/// How autoland's heading knob is set this tick (issue #93): the key and
/// deadband for [`crate::circuit::BugHold`], or `None` to steer it every
/// tick as before (the short final and the flare, where the last 0.3 nm
/// closes the centreline in seconds). A pattern's legs and each missed
/// approach stage are their own keys, so a new leg is set at once. Pure.
pub fn bug_knob(
    phase: ApproachPhase,
    short_final: bool,
    pattern_leg: Option<crate::reversal::Leg>,
    missed: Option<MissedStage>,
) -> Option<(u32, crate::circuit::BugDeadband)> {
    match (phase, short_final, pattern_leg, missed) {
        (ApproachPhase::Flare, ..) | (_, true, ..) => None,
        (_, _, Some(leg), _) => Some((10 + leg as u32, AUTOLAND_INTERCEPT_DEADBAND)),
        (_, _, None, Some(stage)) => Some((20 + stage as u32, AUTOLAND_INTERCEPT_DEADBAND)),
        (ApproachPhase::Final, ..) => Some((2, AUTOLAND_FINAL_DEADBAND)),
        (ApproachPhase::Intercept, ..) => Some((1, AUTOLAND_INTERCEPT_DEADBAND)),
    }
}

/// Where the autoland is in the approach.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApproachPhase {
    /// Flying to intercept the final approach course.
    Intercept,
    /// Established on final, tracking the centreline and glidepath.
    Final,
    /// Very short — over the threshold, flaring.
    Flare,
}

/// How long to sleep between ticks, milliseconds — by what the loop is
/// actually flying.
///
/// 400 ms everywhere made the tick 0.6 s end to end (measured on the EGPH
/// landing, 2026-09-20: `dt` median 0.59 s, so 1.7 Hz), and a flare is over
/// in three seconds. The 737's own `ArrestLeadProfile` declares 1.5 s of
/// dead time, so an arrest commanded six times a flare gets about TWO
/// commands the aircraft can still act on — it touched down at 977 fpm
/// against a 175 fpm target, and the yoke visibly stepped between updates.
/// No gain fixes that; there is not enough bandwidth to fly a flare.
///
/// The Web API is served from X-Plane's flight loop, so this is sim frame
/// time and it is spent where it buys something: fast where feet matter,
/// unchanged out on the intercept where nothing moves in half a second.
///
/// Since 2026-09-30 this is the tick's PERIOD, paced from its start
/// (`sim::state::Pacer`), not a sleep after its work: the old sleeps (100,
/// 200, 400) plus the requests' rounds made the MacBook's 0.20 s Flare and
/// 0.33 s Final (bench landings, `power` dt) and the iMac's 0.39 and 0.56.
/// These are the MacBook's, the ticks the laws were tuned on, now on any
/// machine; the Intercept's 540 is its old 400 sleep plus seven 20 ms rounds
/// (no MacBook record).
fn tick_ms(phase: Option<ApproachPhase>, arresting: bool) -> u64 {
    // The roundout is hand-flown on the channels too (it is where the
    // servos come off), and it needs the flare's rate, not Final's.
    if arresting {
        return 200;
    }
    match phase {
        // Over the threshold: the flare and the arrest.
        Some(ApproachPhase::Flare) => 200,
        // Established, configuring, closing the last miles.
        Some(ApproachPhase::Final) => 330,
        _ => 540,
    }
}

/// The rollout's tick period, ms: its old 300 ms sleep plus the MacBook's
/// rounds (bench rollouts before 2026-09-29, `dt` median 0.378 s).
const ROLLOUT_TICK_MS: u64 = 380;
/// The approach's cross-track rate filter, seconds: half the raw rate a
/// tick at the MacBook's 0.33 s Final tick is a time constant of
/// 0.33 / ln 2.
const XTK_RATE_TAU_S: f64 = 0.476;

/// The command the control loop should fly this tick.
#[derive(Debug, Clone, Copy)]
pub struct Guidance {
    pub target_heading_deg: f64,
    pub target_vs_fpm: f64,
    pub target_ias_kts: f64,
    pub dist_nm: f64,
    /// Where the aircraft *should* be for the glidepath, ft MSL.
    pub glidepath_alt_ft: f64,
    pub phase: ApproachPhase,
    /// Why this tick balked the landing, when it did: the gate that flipped
    /// the phase to `Intercept` low enough for the latch to fly it as a missed
    /// approach. `None` on every ordinary tick. Blackboxed with the
    /// `going_around` event, because "climbing straight ahead" with no reason
    /// attached cost a person an hour of reconstruction from the command
    /// column (LEMG 13, 2026-09-12).
    pub balk: Option<&'static str>,
    /// The vertical target is a landing ARREST (roundout or flare): the VS
    /// dial must lead it through the autopilot's lag ([`vs_dial_fpm`]).
    /// False everywhere else, where the dial is the target.
    pub arresting: bool,
}

/// Where the final's pure pursuit aims, in nm from the threshold measured back
/// up the approach. Negative means a point PAST the threshold, on the runway.
///
/// The lead is fixed — about thirty seconds of travel — because that is what
/// makes the convergence exponential in TIME rather than linear in distance:
/// hard correction early, established miles out, then a tracked wings-level
/// final, which is what "stabilised approach" means. An aim at half the
/// remaining distance decays the cut only linearly, and the aircraft rides the
/// whole final at an angle (EGSS 22, 2026-07-28: 28 degrees at 5.4 nm, still
/// 20 at 3, aligned only at 0.13 nm, i.e. over the numbers).
///
/// The trap this closes is that the fix for that was itself undone on short
/// final. Flooring the aim AT the threshold meant that inside one lead
/// distance the aim stopped advancing and the EFFECTIVE lead shrank — 0.875 nm
/// at 1.2 nm out, 0.55 at 0.7, 0.20 at 0.35 — which is the d/2 law again,
/// reintroduced exactly where being lined up matters most. At Southampton the
/// cut sat at 16 degrees from 0.87 nm all the way in to 0.32, then snapped to
/// the runway course over the numbers with the aircraft still 6 degrees off
/// and still turning (2026-08-01).
///
/// So the aim keeps its full lead all the way in and simply walks past the
/// threshold. That is not a trick: the extended centreline and the runway
/// centreline are one line. Bounded by the pavement so the aim is always a
/// point on the runway.
pub fn pursuit_aim_nm(along_nm: f64, gs_kts: f64, rwy_len_m: f64) -> f64 {
    let lead_nm = (gs_kts / 120.0).clamp(0.6, 1.5);
    let past_nm = (rwy_len_m / M_PER_NM * 0.8).clamp(0.2, 1.5);
    (along_nm - lead_nm).max(-past_nm)
}

/// ONCE FLARING, STAY FLARING. `guidance` decides the phase from this tick's
/// radio height alone, so a flare that ballooned back above its gate became
/// "final" again, and final's law put the nose down: forward elevator and a
/// -1,200 fpm dial from 58 ft, then the flare again at 24 ft with the column
/// already at its stop, and the wheels arrived at -1,031 fpm (the Citation,
/// EDDV 27L, 2026-09-22). A pilot holds a balloon and lets it settle, or
/// goes around. So: a tick that was flaring and now reads Final stays in the
/// flare up to the top of the roundout band; above the band the landing is
/// balked (`balloon`) and the go-around latch flies it. Anything else
/// guidance decided (a balk, an overflight) stands.
pub fn hold_flare(
    g: Guidance,
    was_flaring: bool,
    radio_ft: f64,
    gs_kts: f64,
    flare_radio_ft: f64,
) -> Guidance {
    if !was_flaring || g.phase != ApproachPhase::Final {
        return g;
    }
    let (_, band_ft) = arrest_gates_ft(flare_radio_ft, gs_kts);
    if radio_ft > band_ft {
        return Guidance {
            phase: ApproachPhase::Intercept,
            balk: Some("balloon"),
            arresting: false,
            ..g
        };
    }
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (_, flare_vs) = arrest_targets_fpm(nominal_vs);
    Guidance {
        phase: ApproachPhase::Flare,
        target_vs_fpm: flare_target_fpm(radio_ft, flare_vs, nominal_vs),
        arresting: true,
        ..g
    }
}

/// A balloon out of the flare is landed, not gone around from, below this
/// height (with room to stop). EDLP 24 (2026-09-24): the SF50's flare held
/// 0.3 of elevator until it was already climbing, ballooned to 52 ft at +850
/// fpm, balked `balloon` above the 50 ft band, and the go-around from idle at
/// 81 kt and slowing was given 0 fpm and stalled onto the runway at -1,568
/// fpm. A pilot holds a balloon, adds a little power and lets it settle.
pub const BALLOON_COMMIT_FT: f64 = 100.0;

/// The height a recited landing distance is measured from: the book figure
/// (POM, AFM) runs from 50 ft over the threshold, on a stabilised 3 degree
/// approach, to a stop.
const LANDING_SCREEN_FT: f64 = 50.0;
/// The part of that figure a 3 degree path covers from the screen height to
/// the ground: 50 ft x 0.3048 / tan 3 degrees, about 291 m. The book's
/// definition, so the 3 degree constant, not this approach's path.
const SCREEN_AIR_M: f64 = LANDING_SCREEN_FT * M_PER_FT / GLIDEPATH_TAN;

/// The pavement a flare at `radio_ft` still needs to land and stop, metres.
///
/// A recited landing distance starts at 50 ft over the threshold; an
/// aircraft flaring at 8 ft has already flown the part of it between the
/// two, and asking the pavement for the whole figure counts that part twice.
/// LFBD 11, 2026-09-27 (the stock 737, POM 5,360 ft = 1,634 m, 2,418 m of
/// runway): the flare floated to 7.9 ft radio at 145 kt over the ground,
/// 796 m past the threshold with 1,622 m ahead and sinking 184 fpm, and the
/// overflight balk stood because 1,622 < 1,634, by 12 m. "Going around",
/// full power for a second, the wheels touched 1.3 s later and it stopped
/// with about 700 m to spare. The air flown is taken on the book's 3 degree
/// path from 50 ft down to this height, in proportion (a flare at 7.9 ft is
/// credited 245 m of the 291, a balloon at or above 50 ft nothing): the
/// book's own air segment is longer than that geometry, since it includes
/// the flare's float, so what is left asked for is the ground roll plus a
/// float, never less.
///
/// Floored at half the declared figure: a light single's ground roll is a
/// small part of its over-50-ft figure (the 172's book: 1,335 ft total,
/// 575 ft of roll, 43%), and the geometric 291 m would leave it asking for
/// 116 m to stop in 175. Half covers it; a jet's need never reaches the
/// floor (the 737's 1,634 less 291 is 1,343, against 817).
///
/// With NO declared figure the need stays [`FLOAT_MIN_REMAINING_M`] whole:
/// that 900 m is the Baron's measured ~550 m ROLLOUT plus a margin, a
/// ground figure already, with no air segment in it to credit. Pure.
pub fn flare_pavement_need_m(radio_ft: f64, landing_m: Option<f64>) -> f64 {
    match landing_m.filter(|v| v.is_finite() && *v > 0.0) {
        Some(book_m) => {
            let flown_ft = (LANDING_SCREEN_FT - radio_ft.max(0.0)).clamp(0.0, LANDING_SCREEN_FT);
            let air_flown_m = SCREEN_AIR_M * flown_ft / LANDING_SCREEN_FT;
            (book_m - air_flown_m).max(0.5 * book_m)
        }
        None => FLOAT_MIN_REMAINING_M,
    }
}

/// COMMITTED, ONCE FLARING LOW. A flare below the roundout band is at idle,
/// slow and nose up, and the go-around the overflight balk hands it cannot
/// climb: the vertical law climbs only with speed in hand, a flare has none,
/// and a jet's engines are a few seconds from thrust. ETND 26 (2026-09-23):
/// the SF50 ballooned to 32 ft, the float allowance (1,244 m of runway less the
/// Baron's 900 m floor) ran out at 344 m, it went around with 900 m of runway
/// ahead, the law gave it 0 fpm at 80 kt, and it met the runway at about -740
/// fpm. So below the band, an `overflew_the_runway` verdict does not stand
/// while the pavement left still covers the airframe's OWN landing distance
/// (the 900 m floor is for the float's judgement above, not for a landing
/// already in its last seconds). A BALLOON is committed the same way up to
/// [`BALLOON_COMMIT_FT`] (EDLP 24, the crash). No room to stop is still a
/// go-around; every other balk stands. The room is
/// [`flare_pavement_need_m`]: the recited distance less the air segment
/// already flown (LFBD 11). Pure.
#[allow(clippy::too_many_arguments)]
pub fn commit_flare(
    g: Guidance,
    was_flaring: bool,
    radio_ft: f64,
    gs_kts: f64,
    flare_radio_ft: f64,
    remaining_m: f64,
    landing_m: Option<f64>,
    course_deg: f64,
) -> Guidance {
    commit_flare_latched(
        g,
        was_flaring,
        false,
        radio_ft,
        gs_kts,
        flare_radio_ft,
        remaining_m,
        landing_m,
        course_deg,
    )
}

/// ONCE COMMITTED, THE FLARE STAYS COMMITTED. [`commit_flare`] judged the
/// room afresh every tick against [`flare_pavement_need_m`] at THIS height,
/// which high in the band is the overflight allowance itself less the few
/// metres of air credited from 50 ft: the A330 at LEMG 30 (sortie
/// 1790634412-approach, 2026-09-28) was committed at 46.4 ft with 1,432 m
/// ahead against a need of 1,427, floating level, and 0.3 s and 23 m later
/// the same test failed, the arrest handed the servos back and it went
/// around `overflew_the_runway` with 1,409 m of runway left. A commitment
/// that lasts one tick is no commitment. So once committed (`was_committed`,
/// the loop's latch for this flare), the landing continues while the
/// pavement ahead still covers the book's GROUND roll (the need at the
/// wheels, the whole air segment credited); past that it is still a
/// go-around. The ceiling heights stand. Pure.
#[allow(clippy::too_many_arguments)]
pub fn commit_flare_latched(
    g: Guidance,
    was_flaring: bool,
    was_committed: bool,
    radio_ft: f64,
    gs_kts: f64,
    flare_radio_ft: f64,
    remaining_m: f64,
    landing_m: Option<f64>,
    course_deg: f64,
) -> Guidance {
    let (_, band_ft) = arrest_gates_ft(flare_radio_ft, gs_kts);
    let ceiling_ft = match g.balk {
        Some("overflew_the_runway") if was_flaring => band_ft,
        Some("balloon") if was_flaring => BALLOON_COMMIT_FT,
        _ => return g,
    };
    let need_m = flare_pavement_need_m(if was_committed { 0.0 } else { radio_ft }, landing_m);
    if radio_ft > ceiling_ft || remaining_m < need_m {
        return g;
    }
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (roundout_vs, flare_vs) = arrest_targets_fpm(nominal_vs);
    Guidance {
        // The overflight turned the heading back to the FAP; a landing
        // keeps the runway's.
        target_heading_deg: course_deg,
        phase: ApproachPhase::Flare,
        // Above the band (a balloon's top) come back down at the roundout's
        // sink: the flare's -100 from 60 ft is half a minute of runway.
        target_vs_fpm: if radio_ft > band_ft {
            roundout_vs
        } else {
            flare_target_fpm(radio_ft, flare_vs, nominal_vs)
        },
        balk: None,
        arresting: true,
        ..g
    }
}

/// ONCE THE ARREST HAS THE SERVOS, IT KEEPS THEM. `guidance` calls the
/// roundout from this tick's radio height alone (`radio_alt_ft <= band_ft`),
/// so a balloon in the roundout that rose back over the band top turned the
/// arrest off, and [`arrest_hands_step`] gave the servos back to an
/// autopilot whose VS dial was still the approach's descent: LEMG 12, the
/// SF50 (sortie 1790611629, 2026-09-28), taken at 51 ft, ballooned to +263
/// fpm, handed back at 50.5 ft against a 50 ft band; the autopilot flew its
/// -445 fpm dial and the governor, no longer held to "only back", walked the
/// lever 0.16 to 0.27 (on a tail-mounted engine, nose down) for 1.2 s; taken
/// again at 48.4 ft with the sink already rebuilding, 45 to 3 ft in 1.5 s,
/// -814 fpm at the wheels. So a tick called Final and not arresting, with no
/// balk, after the arrest has taken the servos is still the arrest: flown
/// down at the roundout's sink up to [`BALLOON_COMMIT_FT`] while the pavement
/// ahead still covers the landing ([`flare_pavement_need_m`], as
/// [`commit_flare`]); higher, or with no room left, it is a balked landing
/// (`balloon`) and the go-around latch flies it. A real balk (off the
/// centreline, overflown, ordered, a pattern) is never overruled: only a
/// go-around brings the autopilot back. The flare's own balloon is
/// [`hold_flare`]'s. Pure.
pub fn hold_roundout(
    g: Guidance,
    arrest_taken: bool,
    radio_ft: f64,
    gs_kts: f64,
    remaining_m: f64,
    landing_m: Option<f64>,
) -> Guidance {
    if !arrest_taken || g.arresting || g.phase != ApproachPhase::Final || g.balk.is_some() {
        return g;
    }
    if radio_ft > BALLOON_COMMIT_FT || remaining_m < flare_pavement_need_m(radio_ft, landing_m) {
        return Guidance {
            phase: ApproachPhase::Intercept,
            balk: Some("balloon"),
            arresting: false,
            ..g
        };
    }
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (roundout_vs, _) = arrest_targets_fpm(nominal_vs);
    Guidance {
        target_vs_fpm: roundout_vs,
        arresting: true,
        ..g
    }
}

/// Compute the approach guidance: steer to the final approach point then track the
/// centreline, and descend the 3° glidepath. Pure — the control loop applies it via
/// the autopilot. `flare_radio_ft` is the height above the runway to start the flare.
/// `xtk_rate_nm_min` is the cross-track closure rate (positive = drifting right),
/// 0.0 when unknown — the damping term on the final's centreline correction.
#[allow(clippy::too_many_arguments)]
pub fn guidance(
    cur_lat: f64,
    cur_lon: f64,
    cur_alt_ft: f64,
    radio_alt_ft: f64,
    gs_kts: f64,
    ias_kts: f64,
    rwy: &Runway,
    approach_ias: f64,
    flare_radio_ft: f64,
    xtk_rate_nm_min: f64,
) -> Guidance {
    guidance_on(
        cur_lat,
        cur_lon,
        cur_alt_ft,
        radio_alt_ft,
        gs_kts,
        ias_kts,
        rwy,
        approach_ias,
        flare_radio_ft,
        xtk_rate_nm_min,
        None,
    )
}

/// [`guidance`] on a runway of known pavement width (`width_m`, apt.dat's):
/// the lateral gates below the terrain floor judge the final against the
/// pavement ([`landing_xtk_limit_m`], [`flare_xtk_limit_m`]). `None` is the
/// old 37 m everywhere.
#[allow(clippy::too_many_arguments)]
pub fn guidance_on(
    cur_lat: f64,
    cur_lon: f64,
    cur_alt_ft: f64,
    radio_alt_ft: f64,
    gs_kts: f64,
    ias_kts: f64,
    rwy: &Runway,
    approach_ias: f64,
    flare_radio_ft: f64,
    xtk_rate_nm_min: f64,
    width_m: Option<f64>,
) -> Guidance {
    let landing_xtk_nm = landing_xtk_limit_m(width_m) / M_PER_NM;
    let flare_xtk_nm = flare_xtk_limit_m(width_m) / M_PER_NM;
    let dist_nm = haversine_nm(cur_lat, cur_lon, rwy.thr_lat, rwy.thr_lon);
    let recip = (rwy.course_deg + 180.0) % 360.0;

    // 3° glidepath altitude at this distance (+ = we're above it). Aimed at the
    // touchdown zone, so it crosses the threshold at ~50 ft, not zero.
    let glidepath_alt_ft = rwy.elev_ft + (dist_nm + TOUCHDOWN_AIM_NM) * gp_ft_per_nm();
    let alt_err = cur_alt_ft - glidepath_alt_ft;

    // Above what the way in can lose, we can't just dive onto the path: fly to
    // the JOIN point instead: where a 3° descent (with 15% margin) meets our
    // CURRENT altitude, out along the extended centreline. When we're inside it
    // (armed high overhead, the failure seen live: 7,000 ft at 6 nm, VS pegged,
    // never capturing) that means flying OUTBOUND first; the point walks back
    // toward the FAP as altitude burns off (a teardrop join). See
    // [`aims_at_join_point`] for the height a straight-in keeps.
    let join_nm = join_point_nm(cur_alt_ft, rwy.elev_ft);

    // Are we past the threshold, on the departure side, still airborne — by
    // more than the landing float is allowed? (The flare branch below, checked
    // first, owns a normal float/rollout across the numbers.)
    let along_nm = along_track_nm(cur_lat, cur_lon, rwy);
    // The arrest's gates and targets for THIS groundspeed (seconds of sink,
    // never bare feet: see `arrest_gates_ft`). `flare_radio_ft` is the floor.
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (flare_ft, band_ft) = arrest_gates_ft(flare_radio_ft, gs_kts);
    let (roundout_vs, flare_vs) = arrest_targets_fpm(nominal_vs);
    let float_nm = overflight_allowance_nm(
        rwy.length_m,
        radio_alt_ft,
        band_ft,
        crate::aircraft::landing_distance_m(),
    );
    let past_threshold = along_nm < -float_nm;
    // The flare/roundout environment: short final by plain distance, or
    // anywhere over the pavement the float allowance still covers — `dist_nm`
    // is unsigned, so past the threshold it alone would end the flare zone at
    // 0.35 nm while the float is still legitimately landing.
    let in_flare_zone = dist_nm <= FLARE_MAX_DIST_NM || (along_nm < 0.0 && !past_threshold);

    let xtk = cross_track_nm(cur_lat, cur_lon, rwy);
    let mut balk: Option<&'static str> = None;
    let (phase, target_heading) = if radio_alt_ft <= flare_ft && in_flare_zone {
        // Low radio altitude alone is NOT the flare cue — terrain rises to meet
        // the approach path miles from the field (seen live: "flared" at 7 ft AGL
        // 4.1 nm out at LEMI, power cut over open ground; and again into a dip
        // 0.4 nm short at LERL). Flare only over the runway environment.
        if xtk.abs() > flare_xtk_nm {
            // ...and only over the PAVEMENT. The final's P-only tracking left a
            // standing 0.10 nm cross-track under the LXGB crosswind and the
            // flare froze it: touchdown 185 m right of the centreline, in the
            // grass, wrecked on the rollout (2026-07-22). Off-centreline at
            // flare height is a balked landing — the phase flip drops the
            // GoAroundLatch, which climbs straight ahead.
            balk = Some("off_centreline_at_flare");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else {
            // Keep steering through the flare — locking the bare runway course
            // froze whatever offset the final still carried. Gentler clamp:
            // no big heading swings a few feet off the ground. Rate-damped
            // like the final, so a closing drift isn't chased into an S.
            let corr = ((xtk + 0.2 * xtk_rate_nm_min) * 80.0).clamp(-8.0, 8.0);
            (
                ApproachPhase::Flare,
                (rwy.course_deg - corr + 360.0) % 360.0,
            )
        }
    } else if past_threshold {
        // Overflew the runway without landing (too high/fast at the numbers). Go
        // around: steer BACK to the Final Approach Point, never track the runway
        // course outbound — that departs the field forever (seen live 2026-07-06:
        // over the numbers at 167 ft, then straight out to 5.4 nm at 60 kt, never
        // turning back). The vertical law climbs to a safe altitude on the way, if
        // energy allows. Takes priority over the join/FAP/final branches so a
        // go-around always turns toward the field regardless of altitude.
        let (fap_lat, fap_lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, FAP_NM);
        balk = Some("overflew_the_runway");
        (
            ApproachPhase::Intercept,
            bearing_deg(cur_lat, cur_lon, fap_lat, fap_lon),
        )
    } else if aims_at_join_point(alt_err, along_nm, xtk, gs_kts) {
        // Too high for this distance — descend toward the join point.
        let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
        (
            ApproachPhase::Intercept,
            bearing_deg(cur_lat, cur_lon, jlat, jlon),
        )
    } else if dist_nm > FAP_NM + 0.5 {
        // Outside the FAP: steer TO the Final Approach Point — a point FAP_NM out
        // on the approach side of the threshold along the extended centreline —
        // not the bare runway course. Tracking the raw course assumes we're
        // already on the approach side flying inbound; when we're off to the side
        // or PAST the threshold it flies us away from the field, the distance
        // growing every tick (seen live: XLE000N, 12.6 -> 13.8 nm on a fixed
        // heading, never intercepting). Aiming at the FAP always closes on the
        // runway from the correct side, then the final branch tracks the centreline.
        let (fap_lat, fap_lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, FAP_NM);
        (
            ApproachPhase::Intercept,
            bearing_deg(cur_lat, cur_lon, fap_lat, fap_lon),
        )
    } else {
        // On final: PURSUE a point ON the centreline halfway to the threshold
        // instead of P-correcting the offset. Offset-proportional gains kept
        // failing by geometry: 80°/nm commanded all of 7° of cut at 0.09 nm
        // off — mathematically unable to centre in the distance remaining,
        // and the aircraft crossed 150 ft still 145 m left (2026-07-24; same
        // family as the 120 m/2026-07-14 and 55 m/2026-07-22 misses at lower
        // gains). Aiming at the centreline point makes the cut DISTANCE-
        // aware: atan(offset / distance-to-aim) — gentle far out, up to a
        // 35° intercept when close and offset, rolling out on the line as
        // the pursuit point recedes.
        // Inside ~550 m the halfway aim point would fall BEHIND the aircraft
        // (the 0.25 nm floor exceeded the distance) and the bearing flipped
        // the cut ±35° at 100 ft over the numbers (2026-07-24). Short final
        // uses small course-referenced corrections only — offsets there are
        // either small or the balk gate's problem.
        // FIXED-lead pursuit, not half-the-remaining-distance. With the aim at
        // d/2 the cut angle decays only LINEARLY with distance (offset falls
        // as d², cut ~ offset/d ~ d), so the aircraft rode the WHOLE final at
        // an angle and only rolled out at the flare — measured at EGSS 22:
        // final began 5.4 nm out with a 28° cut, still 20° at 3 nm, 9° at
        // 1.9, 5° inside a mile, aligned at 0.13 nm, i.e. over the numbers
        // ("never in line with the runway, always at an angle", 2026-07-28).
        // A fixed lead makes the convergence exponential in TIME instead:
        // hard correction early, established miles out, then a tracked,
        // wings-level final — which is what "stabilised approach" means.
        // ~30 s of travel, floored so a slow single still aims somewhere and
        // capped so the far-out cut stays flyable.
        let cut = if dist_nm > 0.30 {
            let along = along_track_nm(cur_lat, cur_lon, rwy);
            let aim_nm = pursuit_aim_nm(along, gs_kts, rwy.length_m);
            let (alat, alon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, aim_nm);
            let brg = bearing_deg(cur_lat, cur_lon, alat, alon);
            ((brg - rwy.course_deg + 540.0).rem_euclid(360.0) - 180.0).clamp(-35.0, 35.0)
        } else {
            (-xtk * 80.0).clamp(-8.0, 8.0)
        };
        if radio_alt_ft < 100.0 && dist_nm < 0.6 && xtk.abs() > landing_xtk_nm {
            // Not going to make the pavement: balk NOW, with height to work
            // with. The 22 ft flare-gate balk arrested nothing — the aircraft
            // still skimmed the ground at 91 kt before climbing (2026-07-22).
            balk = Some("off_centreline_low");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else if along_nm > STABILISED_MIN_ALONG_NM
            && radio_alt_ft >= TERRAIN_FLOOR_AGL_FT
            && !stabilised_xtk(xtk, along_nm)
        {
            // Inside the FAP but not somewhere a final can be flown from:
            // reposition to the join point out along the centreline, the
            // way the alt_err branch above does. "Land the aircraft" from
            // 1.5 nm abeam Gibraltar's 09 used to be called FINAL here — a
            // saturated 35 degree cut, the speed schedule pulling the RV-10
            // to its 65 kt "approach speed", and a stall (2026-09-11).
            let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
            (
                ApproachPhase::Intercept,
                bearing_deg(cur_lat, cur_lon, jlat, jlon),
            )
        } else if (TERRAIN_FLOOR_AGL_FT..STABILISED_GATE_AGL_FT).contains(&radio_alt_ft)
            && (STABILISED_MIN_ALONG_NM..FAP_NM).contains(&along_nm)
            && !crate::reversal::stabilisable(xtk, along_nm, gs_kts)
        {
            // THE STABILISED GATE, where a crew has one: a thousand feet.
            // Position alone called 1,880 m off at 3.3 nm "interceptable"
            // (a 35 degree cut does reach it), the 737 rode the glidepath
            // down beside the runway and the first lateral balk it met was
            // the flare gate's, at 94 ft and 207 m off; the second approach
            // got to 143 ft (EGTE, 2026-09-21). The pursuit converges in
            // TIME, so the question is the seconds of final left
            // (`reversal::predicted_residual_m`). The loop flies this one as
            // a go-around whatever the height.
            balk = Some("unstabilised_at_gate");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else if radio_alt_ft < TERRAIN_FLOOR_AGL_FT
            && (STABILISED_MIN_ALONG_NM..FAP_NM).contains(&along_nm)
            && !(stabilised_xtk(xtk, along_nm)
                && (width_m.is_none()
                    || lands_on_the_pavement(xtk, along_nm, gs_kts, landing_xtk_nm * M_PER_NM)))
        {
            // ...and "interceptable" is not stabilised either: a 35 degree
            // cut reaches 50 m from anywhere, and the 737 rode 30 to 50 m
            // beside a 45 m strip from 500 ft to the wheels (LEMG 13,
            // 2026-09-28). Below the floor the pursuit's own TIME must bring
            // the offset onto the pavement by the threshold.
            // Below the terrain floor and not laterally stabilised: a balk
            // with height in hand. That flare gate above was the ONLY lateral
            // balk, so a final that never converged (handed off 50 degrees
            // off at Gibraltar) flew to 95 ft still 306 m off the centreline,
            // went around with a 470 fpm sink and full flap, and hit the
            // North Front four seconds later (2026-09-11). Inside the FAP
            // only: further out the intercept is still legitimately wide,
            // and radio height alone means nothing over rising ground.
            balk = Some("unstabilised_below_floor");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else {
            (
                ApproachPhase::Final,
                (rwy.course_deg + cut).rem_euclid(360.0),
            )
        }
    };

    // Track the path with the nominal descent for the current groundspeed
    // (`nominal_vs`, above) plus an error correction; bounded so it never dives.
    // SPEED PROTECTION overrides everything except the flare. Holding level below
    // the glidepath to "wait for the path" bleeds airspeed just as surely as
    // climbing does when the energy is marginal — that's the failure seen live:
    // stuck below the path 13 nm out, IAS oscillating down to 53 kt in a level
    // hold until the AP stalled off. A stall is fatal; being low is not. So when
    // airspeed decays below the approach speed, lower the nose and trade a little
    // altitude back for speed, proportional to how slow it is.
    // SAFE-ALTITUDE CLIMB: below the terrain floor and outside the flare zone, the
    // low radio altitude is rising terrain under the flight path — climb away from
    // it. But ONLY with surplus airspeed: climbing slow bleeds energy to a stall
    // (why this loop was descent-only). Low + slow falls through to speed
    // protection / level hold, and the raised throttle target below rebuilds the
    // surplus so the climb can engage on a later tick.
    let below_terrain_floor = match phase {
        ApproachPhase::Flare => false,
        // Final owns the descent to the runway — the floor stands down inside
        // short-final range where low radio altitude IS the approach, and it
        // never captures an aircraft ON the path (FLOOR_BELOW_PATH_FT: the
        // LXGB 09 overflight, 2026-07-22).
        ApproachPhase::Final => {
            radio_alt_ft < TERRAIN_FLOOR_AGL_FT
                && dist_nm > FLOOR_ENFORCE_MIN_DIST_NM
                && alt_err < FLOOR_BELOW_PATH_FT
        }
        // Intercept is repositioning, not landing: the floor holds at ANY
        // distance (see FLOOR_ENFORCE_MIN_DIST_NM for the KPDK 16 accident),
        // and at CIRCUIT height, not the bare terrain floor — a go-around
        // was vectoring back at 500-600 ft AGL ("is that a bit too low?",
        // 2026-07-22; it is).
        ApproachPhase::Intercept => radio_alt_ft < REPOSITION_MIN_AGL_FT,
    };
    let has_climb_energy = ias_kts > approach_ias + CLIMB_SPEED_MARGIN_KTS;

    // A SOFT floor under the descent, not the hard edge the climb gate is.
    // Repositioning wants REPOSITION_MIN_AGL_FT (~1,590 ft AGL), the 3° path
    // inside 5 nm sits below it, and the two laws met at a step: dive at 1,200
    // toward the path, cross the floor, climb at 800 back through it, repeat.
    // The Luton go-around (2026-07-28) rode that limit cycle for three and a
    // half minutes, porpoising 1,859 to 2,050 ft while wandering between 1.1
    // and 4.1 nm from the field. Fading the permitted descent to zero AT the
    // floor lands the aircraft on it instead of through it.
    let descent_cap_fpm = path_descent_cap_fpm(nominal_vs);
    let descent_limit_fpm = if phase == ApproachPhase::Intercept {
        -((radio_alt_ft - REPOSITION_MIN_AGL_FT) * 2.0).clamp(0.0, -descent_cap_fpm)
    } else {
        descent_cap_fpm
    };

    // The roundout band's own gates, named once: the vertical law below owns
    // the descent from here, and the dial leads it (`arresting`).
    let roundout = phase == ApproachPhase::Final
        && !below_terrain_floor
        && in_flare_zone
        && xtk.abs() <= FLARE_MAX_XTK_NM
        && radio_alt_ft <= band_ft
        && roundout_reaches_pavement(along_nm, radio_alt_ft, gs_kts);

    // What the vertical path asks for, computed first so the speed-protection
    // branch below can be measured against it.
    let path_vs = match phase {
        // Arrest the descent near the ground: exponentially (`flare_target_fpm`).
        ApproachPhase::Flare => flare_target_fpm(radio_alt_ft, flare_vs, nominal_vs),
        // Below the floor without the surplus to climb: hold level while the
        // throttle bonus rebuilds it — NEVER track the glidepath down. The
        // glidepath altitude is distance-symmetric, so a rejoin on the wrong
        // side of the threshold reads "on path" and descends (KPDK 16).
        _ if below_terrain_floor => 0.0,
        // ROUNDOUT: on final over the pavement, just above the flare gate,
        // keep coming down gently — never chase the path and never hold
        // level. Both wrong answers happened from the same arithmetic: past
        // the threshold the (unsigned) glidepath distance grows again, so
        // alt_err goes negative and the level-hold branch below fired at
        // 27 ft over the runway. The aircraft floated level, bled the very
        // airspeed the flare needed, and dropped on from the mush at
        // -380 fpm one knot above the clean stall (EGHI, 2026-08-02). A
        // staged descent — this, then the flare's -100 — arrives at the
        // flare with flying speed, which is what a soft touchdown is made
        // of.
        // ...but only where its own descent still REACHES the pavement.
        // `in_flare_zone`'s first term is an unsigned distance, so this
        // branch also fires SHORT of the threshold — right for a normal
        // landing (40 ft crosses the numbers from ~0.1 nm out), and a
        // commanded descent into the approach lights for an aircraft dragged
        // low: at 30 ft radio 0.3 nm short, -250 fpm touches down ~190 m
        // BEFORE the runway, where the old level-hold below let the path
        // come down to it (audit, 2026-08-02). The test is the physical one
        // — see `roundout_reaches_pavement`.
        ApproachPhase::Final if roundout => roundout_vs,
        // OVER THE PAVEMENT above the band: keep the path's own sink. The
        // glidepath altitude is distance-symmetric, so past the threshold
        // alt_err turns negative and the level-hold below fired at 60 ft
        // over EGCC 23L with a 737 sinking 730 fpm; the dial read zero, the
        // aircraft levelled at 30 ft and floated 500 m into the balloon
        // (2026-09-19). A jet that crosses the numbers high has runway to
        // spend and no height to hold; the float rule above still bounds
        // how far. Never steeper either: the aim point 300 m in is not
        // worth a dive from 90 ft.
        ApproachPhase::Final if along_nm < 0.0 && xtk.abs() <= FLARE_MAX_XTK_NM => nominal_vs,
        // On/above speed but below the glidepath: hold level and let the path
        // descend to us — don't climb to intercept. Speed protection above has
        // already handled the case where holding level would stall us.
        _ if alt_err < -50.0 => 0.0,
        // 1.2 fpm per ft of path error: the old 0.4 recovered 77 fpm
        // against the 190 ft the LXGB 09 final carried — it arrived at
        // the numbers still 190 ft high (2026-07-22). 1.2 converges a
        // 200 ft error inside the last mile; the clamp still forbids a
        // dive, and it is never shallower than the path itself plus a
        // catch-up (`path_descent_cap_fpm`).
        _ => (nominal_vs - alt_err * glidepath_gain_fpm_per_ft(gs_kts))
            .clamp(descent_cap_fpm, 0.0)
            .max(descent_limit_fpm),
    };

    let speed_deficit = approach_ias - ias_kts;
    let target_vs = if phase != ApproachPhase::Flare && speed_deficit > SPEED_FLOOR_MARGIN_KTS {
        // Nose down to regain speed — but NEVER shallower than the path was
        // already asking for. The protection rate is a fixed 40 fpm per knot
        // capped at 700, which on a slow airframe is steeper than the path and
        // on a fast one is not: a jet on a 3° path at 140 kt needs ~740 fpm, so
        // 9 knots slow produced a 360 fpm "protection" that HALVED the descent
        // and made the aircraft slower still, the exact opposite of the intent.
        // The MD-82 drifted above the path through the last mile that way and
        // crossed the threshold 51 ft high at 152 kt, which is the overflight
        // that forced the Luton go-around (2026-07-28).
        path_vs.min(-(speed_deficit * 40.0).clamp(200.0, 700.0))
    } else if below_terrain_floor && has_climb_energy {
        // Bounded climb toward THIS PHASE'S floor, steeper the deeper below
        // it. The magnitude used to reference the 500 ft terrain floor even
        // when the repositioning gate had armed the climb — above 500 AGL it
        // clamped to a 100 fpm dribble, and the LXGB go-around crept along
        // at ~650 ft into the Rock (2026-07-22).
        let floor_ft = match phase {
            ApproachPhase::Intercept => REPOSITION_MIN_AGL_FT,
            _ => TERRAIN_FLOOR_AGL_FT,
        };
        ((floor_ft - radio_alt_ft) * 2.0).clamp(100.0, max_climb_fpm())
    } else {
        path_vs
    };

    // Below the terrain floor, ask the throttle governor for extra speed so it
    // builds the surplus the climb gate needs. Otherwise fly the DECELERATION
    // SCHEDULE: clean out at range, approach speed by the FAP. The floor case
    // still wins, and the schedule can only ever ask for MORE speed than the
    // bare approach figure, so nothing downstream sees a slower target than it
    // used to.
    let target_ias = if below_terrain_floor {
        approach_ias + CLIMB_ENERGY_BONUS_KTS
    } else {
        scheduled_ias(dist_nm, approach_ias)
    };

    Guidance {
        target_heading_deg: target_heading,
        target_vs_fpm: target_vs,
        target_ias_kts: target_ias,
        dist_nm,
        glidepath_alt_ft,
        phase,
        balk,
        arresting: phase == ApproachPhase::Flare || roundout,
    }
}

#[cfg(test)]
mod tests {

    /// EGSS 22, 2026-09-22 (issue #65): "Down and stopped. Brakes on, your
    /// aircraft." with the 737's `parking_brake_ratio` at 0.03, the rollout's
    /// squeeze expired. The stop's line is chosen from the parking brake's
    /// read-back: the brake reading 0.03 through every drive names that it did
    /// not set, and only a brake that reads set is "Brakes on".
    #[tokio::test]
    async fn the_stop_says_brakes_on_only_when_the_parking_brake_reads_set() {
        use crate::xplane::FakeXPlane;
        const PARK: &str = "sim/cockpit2/controls/parking_brake_ratio";
        const SET: &str = "sim/flight_controls/park_brake_set";
        let egss = FakeXPlane::with_reads(&[(PARK, 0.03)]);
        let r = crate::tools::controls::set_park_brake_confirmed(&egss, None, 3).await;
        let line = super::stopped_line(r.held);
        assert!(!line.contains("Brakes on"), "{line}");
        assert!(line.contains("did not set"), "{line}");

        let set = FakeXPlane::with_reads(&[(PARK, 0.03)]);
        set.step_on_command(SET, PARK, 0.97);
        let r = crate::tools::controls::set_park_brake_confirmed(&set, None, 3).await;
        assert_eq!(
            super::stopped_line(r.held),
            "Down and stopped. Brakes on, your aircraft."
        );
        assert!(
            !set.wrote(PARK),
            "the command set it; no raw ratio write needed"
        );
    }

    /// The flare flies the ELEVATOR, and it has to arrest a real sink in the
    /// three seconds it has. Flown against a plant with no autopilot in it —
    /// elevator to pitch rate to sink — because removing the autopilot's
    /// 1.5 s of dead time is the entire point of the change.
    #[test]
    fn the_flare_arrests_a_jets_sink_on_the_elevator() {
        // A 737-ish short-period response: elevator commands a pitch rate,
        // pitch changes the sink, both first order and quick. Deliberately
        // NOT the VS-mode plant, which is what we stopped flying through.
        struct Plant {
            vvi: f64,
            rate: f64,
        }
        impl Plant {
            fn step(&mut self, elev: f64, dt: f64) {
                // Elevator to vertical acceleration, with a short lag.
                let want = elev * 1400.0; // fpm/s at full commanded travel
                self.rate += (want - self.rate) * (dt / 0.35).min(1.0);
                self.vvi += self.rate * dt;
            }
        }
        let mut law = FlarePitch::default();
        let mut p = Plant {
            vvi: -900.0,
            rate: 0.0,
        };
        let target = -175.0; // the 737's flare target
        let dt = 0.1; // the flare tick rate
        let mut prev = p.vvi;
        let mut worst_up: f64 = 0.0;
        for _ in 0..30 {
            // 3 seconds
            let trend = (p.vvi - prev) / dt;
            prev = p.vvi;
            let e = law.step(target, p.vvi, trend, dt, false);
            worst_up = worst_up.max(e);
            p.step(e, dt);
        }
        assert!(
            p.vvi > target - 120.0,
            "three seconds must arrest 900 fpm to near {target}: got {:.0}",
            p.vvi
        );
        assert!(
            p.vvi < 0.0,
            "and must not balloon: the arrest settles ON the target, it does \
             not fly through it: got {:.0}",
            p.vvi
        );
        assert!(
            (p.vvi - target).abs() < 60.0,
            "settled on {target}, not merely near it: got {:.0}",
            p.vvi
        );
        assert!(
            worst_up <= FLARE_PITCH_MAX + 1e-9,
            "never past the clamp: {worst_up}"
        );
    }

    /// EGLF 06, 2026-09-22: the arrest worked and then gave the runway back.
    /// The law beat its target at thirteen feet, relaxed the column from 0.35
    /// to 0.185 because the error had gone negative, and [`FlarePower`] -
    /// unlocked by that very arrest - took the lever from 0.29 to idle over
    /// the last 3.4 s. Touchdown at about 360 fpm and still worsening.
    ///
    /// The plant is the flare plant above with the thrust leaving underneath
    /// it: a sinking bias that ramps in over three seconds, which is what
    /// losing a third of a lever's travel at 140 kt feels like. Without the
    /// ratchet the law hands it back; with it the column stays where the
    /// arrest put it and the wheels arrive softly.
    #[test]
    fn the_flare_does_not_hand_back_the_arrest_when_the_thrust_leaves() {
        struct Plant {
            vvi: f64,
            rate: f64,
            /// Sink the departing thrust adds, fpm/s at full effect.
            thrust_loss: f64,
        }
        impl Plant {
            fn step(&mut self, elev: f64, dt: f64, retard: f64) {
                let want = elev * 1400.0 - self.thrust_loss * retard;
                self.rate += (want - self.rate) * (dt / 0.35).min(1.0);
                self.vvi += self.rate * dt;
            }
        }

        /// Fly the last four seconds and return the touchdown sink.
        fn fly(ratchet: bool) -> f64 {
            let mut law = FlarePitch::default();
            let mut p = Plant {
                vvi: -510.0, // where the EGLF arrest had got to at 21 ft
                rate: 0.0,
                thrust_loss: 260.0,
            };
            let (target, dt) = (-197.0, 0.1);
            let mut prev = p.vvi;
            for tick in 0..40 {
                // The lever walks off over the last 3.4 s, as it did live.
                let retard = (tick as f64 * dt / 3.4).clamp(0.0, 1.0);
                let trend = (p.vvi - prev) / dt;
                prev = p.vvi;
                let e = law.step(target, p.vvi, trend, dt, ratchet);
                p.step(e, dt, retard);
            }
            p.vvi
        }

        let handed_back = fly(false);
        assert!(
            handed_back < -300.0,
            "the tracker must still reproduce the firm landing, or this test \
             is not about EGLF any more: got {handed_back:.0}"
        );
        let held = fly(true);
        assert!(
            held > -300.0,
            "the column stays where the arrest put it, so the wheels arrive \
             under the instructor's firm threshold: got {held:.0}"
        );
        assert!(held < 0.0, "and the aircraft still lands: got {held:.0}");
    }

    /// EDDV 27L, 2026-09-22, the Citation: the flare at 28 ft held its pull
    /// through a climb (+545 fpm, up to 58 ft), and the phase flipped back to
    /// final, which pushed. The plant needs a STANDING pull to hold the flare
    /// attitude (`E_TRIM`), so neutral elevator sinks and anything held above
    /// it climbs: that is what makes a held pull a balloon. The old ratchet
    /// is exactly the running maximum of the free law (the integrator never
    /// reads `held`), so the test flies both.
    #[test]
    fn a_balloon_eases_the_pull_and_settles_onto_the_runway() {
        const E_TRIM: f64 = 0.10;
        struct Plant {
            h: f64,
            vvi: f64,
            rate: f64,
        }
        impl Plant {
            fn step(&mut self, elev: f64, dt: f64) {
                let want = (elev - E_TRIM) * 1400.0;
                self.rate += (want - self.rate) * (dt / 0.35).min(1.0);
                self.vvi += self.rate * dt;
                self.h += self.vvi / 60.0 * dt;
            }
        }
        // (peak height, touchdown sink) from the flare gate at 140 kt.
        fn fly(old_ratchet: bool) -> (f64, f64) {
            let mut law = FlarePitch::default();
            let mut p = Plant {
                h: 37.0,
                vvi: -480.0,
                rate: 0.0,
            };
            let (_, target) = arrest_targets_fpm(-(140.0 / 60.0) * GLIDEPATH_FT_PER_NM);
            let dt = 0.1;
            let (mut prev, mut peak, mut held) = (p.vvi, p.h, f64::MIN);
            for _ in 0..300 {
                let trend = (p.vvi - prev) / dt;
                prev = p.vvi;
                let e = if old_ratchet {
                    held = held.max(law.step(target, p.vvi, trend, dt, false));
                    held
                } else {
                    law.step(target, p.vvi, trend, dt, true)
                };
                p.step(e, dt);
                peak = peak.max(p.h);
                if p.h <= 0.0 {
                    return (peak, p.vvi);
                }
            }
            (peak, p.vvi)
        }
        let (old_peak, _) = fly(true);
        assert!(
            old_peak > 55.0,
            "the old ratchet must still balloon, or this test is not about \
             EDDV any more: peaked at {old_peak:.0} ft"
        );
        let (peak, sink) = fly(false);
        assert!(
            peak < old_peak - 10.0,
            "eased: {peak:.0} ft against {old_peak:.0}"
        );
        let (_, band) = arrest_gates_ft(25.0, 140.0);
        assert!(
            peak < band,
            "never high enough to balk: {peak:.0} against {band:.0}"
        );
        assert!(
            sink > -400.0 && sink < 0.0,
            "and it lands softly: {sink:.0} fpm"
        );
    }

    /// Once flaring, a balloon is still the flare; above the roundout band it
    /// is a balked landing, never "final" again with its push.
    #[test]
    fn a_balloon_stays_in_the_flare_and_a_big_one_goes_around() {
        let rwy = rwy_north();
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.05);
        let (flare_ft, band_ft) = arrest_gates_ft(25.0, 140.0);
        let at =
            |radio: f64| guidance(lat, lon, radio, radio, 140.0, 140.0, &rwy, 132.0, 25.0, 0.0);

        let g = at(58.0);
        assert!(58.0 > flare_ft && 58.0 < band_ft, "{flare_ft} {band_ft}");
        assert_eq!(g.phase, ApproachPhase::Final, "the tick that pushed");
        let held = hold_flare(g, true, 58.0, 140.0, 25.0);
        assert_eq!(held.phase, ApproachPhase::Flare);
        assert!(held.arresting);
        let (_, flare_vs) = arrest_targets_fpm(-(140.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert_eq!(held.target_vs_fpm, flare_vs);
        // Never flared: nothing changes.
        assert_eq!(
            hold_flare(g, false, 58.0, 140.0, 25.0).phase,
            ApproachPhase::Final
        );

        let high = band_ft + 10.0;
        let balked = hold_flare(at(high), true, high, 140.0, 25.0);
        assert_eq!(balked.phase, ApproachPhase::Intercept);
        assert_eq!(balked.balk, Some("balloon"));
    }

    /// ETND 26, 2026-09-23: 1,244 m of runway, the SF50 flaring at 32 ft and
    /// 344 m past the threshold, 900 m ahead, a 525 m landing distance. The
    /// overflight balk stood and the go-around hit the runway at -740 fpm.
    #[test]
    fn a_low_flare_with_room_to_stop_is_committed() {
        let g = Guidance {
            target_heading_deg: 265.0,
            target_vs_fpm: 1500.0,
            target_ias_kts: 87.0,
            dist_nm: 0.186,
            glidepath_alt_ft: 0.0,
            phase: ApproachPhase::Intercept,
            balk: Some("overflew_the_runway"),
            arresting: false,
        };
        let sf50_m = Some(1721.0 * 0.3048);
        let c = commit_flare(g, true, 32.0, 77.0, 25.0, 900.0, sf50_m, 265.0);
        assert_eq!(c.phase, ApproachPhase::Flare);
        assert_eq!(c.balk, None);
        assert!(c.arresting && c.target_vs_fpm < 0.0);
        assert_eq!(c.target_heading_deg, 265.0, "the runway's heading, not the FAP's");
        // Not flaring (crossed the numbers high and fast): the balk stands.
        assert_eq!(commit_flare(g, false, 32.0, 77.0, 25.0, 900.0, sf50_m, 265.0).balk, g.balk);
        // Above the band: the float's own geometry judges it.
        let (_, band) = arrest_gates_ft(25.0, 77.0);
        assert_eq!(commit_flare(g, true, band + 5.0, 77.0, 25.0, 900.0, sf50_m, 265.0).balk, g.balk);
        // No room to stop: still a go-around. At 20 ft the SF50 has flown
        // 175 m of its book's 525, so it asks 350 m (400 m now commits).
        assert_eq!(commit_flare(g, true, 20.0, 77.0, 25.0, 300.0, sf50_m, 265.0).balk, g.balk);
        // No declared distance: the 900 m floor.
        assert_eq!(commit_flare(g, true, 20.0, 77.0, 25.0, 850.0, None, 265.0).balk, g.balk);
        // A balloon (EDLP 24: 52 ft, 2,180 m of runway, 1,900 ahead) is
        // landed below the commit height and balked above it.
        let balloon = Guidance {
            balk: Some("balloon"),
            ..g
        };
        let c = commit_flare(balloon, true, 52.0, 81.0, 25.0, 1900.0, sf50_m, 240.0);
        assert_eq!((c.phase, c.balk), (ApproachPhase::Flare, None));
        let (roundout_vs, flare_vs) = arrest_targets_fpm(-(81.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert_eq!(c.target_vs_fpm, roundout_vs, "down to the band at the roundout's sink");
        assert!(roundout_vs < flare_vs);
        assert_eq!(
            commit_flare(balloon, true, BALLOON_COMMIT_FT + 1.0, 81.0, 25.0, 1900.0, sf50_m, 240.0)
                .balk,
            Some("balloon")
        );
        assert_eq!(
            commit_flare(balloon, true, 52.0, 81.0, 25.0, 300.0, sf50_m, 240.0).balk,
            Some("balloon"),
            "no room to stop"
        );
        // Any other balk is not ours to overrule.
        let off = Guidance {
            balk: Some("off_centreline_at_flare"),
            ..g
        };
        assert_eq!(commit_flare(off, true, 20.0, 77.0, 25.0, 900.0, sf50_m, 265.0).balk, off.balk);
    }

    /// LFBD 11, 2026-09-27, the stock 737 (POM 5,360 ft): the flare floated
    /// to 7.9 ft radio at 145 kt over the ground, 796 m past the threshold
    /// with 1,622 m of the 2,418 ahead, and the overflight balk stood by
    /// 12 m against the whole book figure. It landed anyway, 1.3 s later,
    /// and stopped with about 700 m to spare.
    #[test]
    fn a_flare_is_credited_the_air_it_has_already_flown() {
        let b738_m = Some(5360.0 * 0.3048);
        let g = Guidance {
            target_heading_deg: 293.0,
            target_vs_fpm: 1500.0,
            target_ias_kts: 150.0,
            dist_nm: 0.43,
            glidepath_alt_ft: 0.0,
            phase: ApproachPhase::Intercept,
            balk: Some("overflew_the_runway"),
            arresting: false,
        };
        let c = commit_flare(g, true, 7.9, 145.0, 25.0, 1622.0, b738_m, 110.0);
        assert_eq!((c.phase, c.balk), (ApproachPhase::Flare, None), "the recorded landing");
        assert_eq!(c.target_heading_deg, 110.0, "the runway's heading");
        // The need at 7.9 ft: 1,634 less 42.1 ft of 3 degree path (245 m).
        let need = flare_pavement_need_m(7.9, b738_m);
        assert!((need - 1389.0).abs() < 2.0, "{need}");
        // The whole 291 m only at the ground, none at or above 50 ft.
        assert!((SCREEN_AIR_M - 290.8).abs() < 0.5, "{SCREEN_AIR_M}");
        let at_wheels = flare_pavement_need_m(0.0, b738_m);
        assert!((at_wheels - (5360.0 * 0.3048 - SCREEN_AIR_M)).abs() < 1e-9);
        assert_eq!(flare_pavement_need_m(50.0, b738_m), 5360.0 * 0.3048);
        assert_eq!(flare_pavement_need_m(80.0, b738_m), 5360.0 * 0.3048);
        // Genuinely too little runway still balks: 1,300 m at 7.9 ft.
        assert_eq!(
            commit_flare(g, true, 7.9, 145.0, 25.0, 1300.0, b738_m, 110.0).balk,
            Some("overflew_the_runway")
        );
        // A light single keeps half its book: the 172's 407 m asks 203 at
        // the ground, never the geometric 116.
        let c172_m = 1335.0 * 0.3048;
        assert_eq!(flare_pavement_need_m(0.0, Some(c172_m)), 0.5 * c172_m);
        // No declared figure: the Baron's 900 m, a rollout already, whole.
        assert_eq!(flare_pavement_need_m(5.0, None), FLOAT_MIN_REMAINING_M);
    }

    #[test]
    fn a_balloon_holds_the_approach_thrust_and_the_ratchet_lets_go_ahead_of_it() {
        assert_eq!(balloon_power(Some(300.0), Some(0.32), false), Some(0.32));
        assert_eq!(
            balloon_power(Some(-200.0), Some(0.32), false),
            None,
            "sinking in the band: the flare's law"
        );
        assert_eq!(
            balloon_power(Some(-200.0), Some(0.32), true),
            Some(0.32),
            "coming back down to the band on power"
        );
        assert_eq!(balloon_power(Some(300.0), None, false), None, "nothing measured");
        // EDLP 24: 0.3 held, vvi -192 easing at +480 fpm/s is a balloon in
        // 0.4 s. The ratchet lets go now instead of at the climb.
        let mut p = FlarePitch::default();
        let held = p.step(-107.0, -451.0, -200.0, 0.2, true);
        assert!(held > 0.2, "{held}");
        let next = p.step(-107.0, -192.0, 480.0, 0.2, true);
        assert!(next < held, "released ahead of the balloon: {held} -> {next}");
        assert!(next >= 0.0, "never a push");
    }

    /// The ratchet is the flare's, not the roundout's, and it never outlives
    /// its approach: a balk and the next arrest starts from nothing.
    #[test]
    fn the_ratchet_is_the_flares_alone_and_resets_with_the_approach() {
        let mut law = FlarePitch::default();
        // Sinking hard: the law pulls, and the ratchet remembers it.
        let pulled = law.step(-197.0, -600.0, 0.0, 0.1, true);
        assert!(pulled > 0.0, "a sink that fast is a pull: {pulled}");
        // Now beating the target by a mile. The tracker wants the column
        // forward; in the flare it does not get it.
        let held = law.step(-197.0, 0.0, 0.0, 0.1, true);
        assert!(
            held >= pulled,
            "the flare's column only comes back: {pulled} then {held}"
        );
        // The same tick in the ROUNDOUT is ordinary flying.
        let mut loose = FlarePitch::default();
        loose.step(-197.0, -600.0, 0.0, 0.1, false);
        let relaxed = loose.step(-197.0, 0.0, 0.0, 0.1, false);
        assert!(
            relaxed < held,
            "the roundout is free both ways: {relaxed} against {held}"
        );
        law.reset();
        let fresh = law.step(-197.0, 0.0, 0.0, 0.1, true);
        assert!(
            fresh < held,
            "a new approach inherits no standing pull: {fresh} against {held}"
        );
    }

    /// The old VS path could not do it, which is why the change exists: the
    /// same three seconds through the 737's declared lag and dead time.
    #[test]
    fn the_vs_dial_could_not_have_arrested_it() {
        // 1.5 s of dead time plus a 1.5 s lag, the 737's ArrestLeadProfile.
        let (dt, target) = (0.6f64, -175.0f64); // and the OLD tick rate
        let mut vvi = -900.0f64;
        let mut queue = std::collections::VecDeque::from(vec![vvi; 3]);
        for _ in 0..5 {
            // 3 seconds at the old rate is five commands
            let commanded = target;
            queue.push_back(commanded);
            let effective = queue.pop_front().unwrap_or(commanded);
            vvi += (effective - vvi) * (dt / 1.5).min(1.0);
        }
        // Three seconds of dead time and lag leave it at about -436 against a
        // -175 target: a quarter of a G of unarrested sink still to absorb in
        // the undercarriage. The elevator law above settles ON -175 in the
        // same three seconds, which is the whole argument for the change.
        assert!(
            vvi < target - 200.0,
            "the dial path is still {:.0} fpm fast at touchdown: {vvi:.0}",
            target - vvi
        );
    }

    /// The tick rate is the bandwidth the flare has to fly with.
    ///
    /// 400 ms everywhere measured 0.59 s end to end on the EGPH landing
    /// (2026-09-20) — 1.7 Hz. A flare lasts about three seconds and the
    /// 737 declares 1.5 s of dead time, so the arrest got roughly two
    /// commands the aircraft could still act on: it touched down at 977 fpm
    /// against a 175 fpm target, and the yoke stepped visibly between
    /// updates. No gain fixes a bandwidth problem.
    #[test]
    fn the_flare_ticks_fast_enough_to_fly_an_arrest() {
        // Periods since 2026-09-30 (paced, not slept): the MacBook's measured
        // ticks, which the 100/200/400 sleeps plus their rounds made.
        assert_eq!(tick_ms(None, false), 540, "nothing to fly yet");
        assert_eq!(
            tick_ms(Some(ApproachPhase::Intercept), false),
            540,
            "miles out"
        );
        assert!(tick_ms(Some(ApproachPhase::Final), false) <= 330);
        let flare = tick_ms(Some(ApproachPhase::Flare), false);
        // The roundout is the arrest's first half, flown on the same
        // channels with the servos off: it ticks at the flare's rate.
        assert_eq!(tick_ms(Some(ApproachPhase::Final), true), flare);
        assert!(flare <= 200, "the flare needs feet, not seconds: {flare}");
        // The 737's arrest lead is 1.5 s of dead time; a three second flare
        // must carry enough commands to matter through it (the MacBook's
        // 0.20 s flare tick, the one the arrest was tuned on: fifteen).
        let commands_in_flare = 3000 / flare;
        assert!(
            commands_in_flare >= 15,
            "only {commands_in_flare} commands in a flare"
        );
        // ...and it must never get SLOWER as the aircraft gets closer.
        assert!(
            tick_ms(Some(ApproachPhase::Flare), false)
                <= tick_ms(Some(ApproachPhase::Final), false)
                && tick_ms(Some(ApproachPhase::Final), false)
                    <= tick_ms(Some(ApproachPhase::Intercept), false),
            "the tick rate must rise monotonically toward the ground"
        );
    }

    /// The tick's I/O budget: ONE read round and ONE write round, on the
    /// plugin's frame and on an older plugin's split reads alike. Every round
    /// is a sim frame (iMac 2026-09-30: one GET 46 ms at 20 fps, eight
    /// concurrent 44 to 51, six sequential 312), and the Final tick waited
    /// on about seven (`power` dt median 0.56 s against its 200 ms sleep).
    /// At 20 fps two rounds make the Flare 0.2 s and Final 0.3 s; at 50 fps
    /// 0.14 and 0.24.
    #[tokio::test(start_paused = true)]
    async fn a_tick_reads_in_one_round_and_writes_in_one() {
        use sim::state::{self, Want, Writes};
        use sim::xplane::{FakeXPlane, FramedXPlane};
        let pos = sim::position::Position {
            lat: 36.66,
            lon: -4.48,
            elev_m: 80.0,
            psi_true_deg: 131.0,
            gs_ms: 70.0,
            vs_fpm: -700.0,
            agl_m: 60.0,
            frame: Some(7),
            sim_time_s: Some(1.0),
            source: sim::position::Source::Plugin,
        };
        let fake = FakeXPlane::new();
        let frame = state::State::from_parts(
            Ok(pos),
            &[(F::Paused, 0.0), (F::Replay, 0.0), (F::Ias, 130.0), (F::Bug, 131.0)],
            state::Source::Plugin,
        );
        fake.set_read_bytes(state::STATE, Some(&state::encode(&frame)));
        // The profile's own lever and approach status ride beside the frame.
        fake.set_read("test/lever", 0.45);
        fake.set_read("test/appr", 0.0);
        // The split: every field the tick wants, and the position.
        for f in AUTOLAND_WANT_ILS {
            fake.set_read(f.name(), 0.0);
        }
        for (k, v) in [
            (crate::datarefs::LATITUDE, 36.66),
            (crate::datarefs::LONGITUDE, -4.48),
            (crate::datarefs::ELEVATION_M, 80.0),
            (PSI_TRUE, 131.0),
            (GS_MS, 70.0),
            ("sim/flightmodel/position/vh_ind_fpm", -700.0),
            (crate::datarefs::AGL_METERS, 18.0),
            (crate::datarefs::PAUSED, 0.0),
            (crate::datarefs::IS_IN_REPLAY, 0.0),
        ] {
            fake.set_read(k, v);
        }
        let xp = FramedXPlane::new(fake, std::time::Duration::from_millis(50));
        for plugin in [true, false] {
            if !plugin {
                xp.inner.set_read_bytes(state::STATE, None);
            }
            let (rounds, _) = xp
                .rounds(|| async {
                    let (st, extra) =
                        state::read_with(&xp, Want(AUTOLAND_WANT_ILS), &["test/lever", "test/appr"])
                            .await;
                    assert!(!st.frozen());
                    assert!(st.position().is_ok() && extra.iter().all(|r| r.is_ok()));
                    let mut w = Writes::new();
                    w.set(VS_DIAL, -650.0)
                        .set(FLARE_PITCH, 0.12)
                        .set(FLARE_ROLL, -0.02)
                        .set(HDG_BUG, 131.2)
                        .set("test/lever", 0.4);
                    assert_eq!(w.flush(&xp).await.ok("test/lever"), Some(true));
                })
                .await;
            assert_eq!(rounds, 2, "plugin frame: {plugin}");
        }
    }
    // LEMG 13, 2026-09-12: the raw heading-minus-track fed straight back to
    // the heading bug hunted the final (crab sd 0.5° on the intercept, 4.0°
    // at the numbers, a ±9° swing with a ~13 s period) into the lateral
    // balk at 74 ft. The filter must pass the wind angle and eat the swing.
    /// LEMG 13, 2026-09-12, replayed through the real lateral law: a 172
    /// whose autopilot settles 2° left of its bug, flown from 3 nm on the
    /// centreline with the loop's own crab filter and tick. Second-order
    /// heading response, the tick's transport delay, the glidepath flown
    /// exactly so only the lateral answer is under test. Returns the
    /// cross-track (nm, right-positive) at 100 ft radio or at the balk,
    /// the guidance there and the trim carried.
    fn fly_final_with_heading_bias(bias_deg: f64, trim_on: bool) -> (f64, Guidance, f64) {
        let rwy = rwy_north();
        let gs_kts = 60.0;
        let dt = 0.6;
        // 3 nm south of the threshold, on the centreline, on the path.
        let (mut lat, mut lon) = project_latlon(0.0, 0.0, 180.0, 3.0);
        let mut psi: f64 = 0.0;
        let mut psi_dot: f64 = 0.0;
        let mut crab = super::CrabFilter::new();
        let mut trim = super::TrackTrim::new();
        let mut bug = 0.0;
        let mut trim_deg = 0.0;
        let mut g = guidance(
            lat, lon, 1000.0, 1000.0, gs_kts, 65.0, &rwy, 65.0, 25.0, 0.0,
        );
        for _ in 0..800 {
            let alt = g.glidepath_alt_ft;
            g = guidance(
                lat,
                lon,
                alt,
                alt - rwy.elev_ft,
                gs_kts,
                65.0,
                &rwy,
                65.0,
                25.0,
                0.0,
            );
            if alt - rwy.elev_ft < 100.0 || g.phase == ApproachPhase::Intercept {
                break;
            }
            let track = psi; // no wind: the bias is the only thing between bug and track
            let c = crab.update(super::CrabFilter::raw(Some(psi), Some(track)), dt);
            trim_deg = if trim_on {
                trim.update(g.phase, g.target_heading_deg, Some(track), dt)
            } else {
                0.0
            };
            bug = (g.target_heading_deg + c + trim_deg).rem_euclid(360.0);
            // The autopilot: second-order toward (bug + bias), ωn 0.5 rad/s, ζ 0.8.
            let err = (bug + bias_deg - psi + 540.0).rem_euclid(360.0) - 180.0;
            psi_dot += (0.25 * err - 2.0 * 0.8 * 0.5 * psi_dot) * dt;
            psi = (psi + psi_dot * dt).rem_euclid(360.0);
            let (nlat, nlon) = project_latlon(lat, lon, track, gs_kts * dt / 3600.0);
            lat = nlat;
            lon = nlon;
        }
        let _ = bug;
        (cross_track_nm(lat, lon, &rwy), g, trim_deg)
    }

    fn baron_flare_power() -> crate::aircraft::FlarePowerProfile {
        let profile: crate::aircraft::AircraftProfile =
            serde_yaml::from_str(include_str!("../profiles/beechcraft_baron_58.yaml")).unwrap();
        profile.flare_power.unwrap()
    }

    #[test]
    fn baron_recorded_flare_walks_the_power_off_and_never_cuts_it() {
        // LEBL 20, 2026-09-16. Replay measurements, NOT a flight-model
        // prediction: the old rule commanded zero throughout these sink rates.
        // Issue #76: the retard is timed, not held for the sink the arrest is
        // removing and let go in the float it made: the lever walks off at
        // the airframe's own rate from the flare gate, never a step.
        let mut power = FlarePower::default();
        let cfg = baron_flare_power();
        assert_eq!(cfg.retard_below_ft, None, "the Baron retards from the flare gate");
        let samples = [
            -393.65, -278.33, -148.51, -49.71, -24.33, -84.22, -211.0, -360.38, -515.28,
        ];
        let mut throttle = 0.30;
        let mut commands = Vec::new();
        for vs in samples {
            throttle = power
                .step(cfg, throttle, vs, 0.67, 10.0, Some(112.0), Some(110.0))
                .unwrap();
            commands.push(throttle);
        }
        assert_eq!(commands[0], 0.30, "no step to idle at flare entry");
        for w in commands.windows(2) {
            assert!(w[1] < w[0], "walking off whatever the sink: {commands:?}");
            assert!(
                w[0] - w[1] <= cfg.reduction_per_second * 0.67 + 1e-9,
                "never faster than the airframe's rate: {commands:?}"
            );
        }
        assert!(commands[8] > 0.0, "not cut: {commands:?}");
        // Slow (8 kt under the approach speed) and still sinking steadily:
        // the thrust stays, the sink it would cost is the wheels'.
        let mut power = FlarePower::default();
        for _ in 0..5 {
            let t = power
                .step(cfg, 0.30, -300.0, 0.67, 10.0, Some(102.0), Some(110.0))
                .unwrap();
            assert_eq!(t, 0.30);
        }
        assert_eq!(power.last_key, Some(RetardKey::Slow));
        // The recorded flare slow: it FLOATS (-393 to -278 in 0.67 s is a
        // float 1.5 s ahead, then -50 and -24), and slow never holds power
        // in a float (issue #146): the lever walks off and never comes back.
        let mut power = FlarePower::default();
        let mut throttle = 0.30;
        let mut slow = Vec::new();
        for vs in samples {
            throttle = power
                .step(cfg, throttle, vs, 0.67, 10.0, Some(102.0), Some(110.0))
                .unwrap();
            slow.push(throttle);
        }
        assert_eq!(slow[0], 0.30);
        for w in slow.windows(2) {
            assert!(w[1] < w[0], "{slow:?}");
        }
        assert!(slow[8] > 0.0, "not cut: {slow:?}");
        assert_ne!(power.last_key, Some(RetardKey::Slow));
    }

    #[test]
    fn flare_power_never_restarts_and_resets_for_a_new_approach() {
        let cfg = baron_flare_power();
        let mut power = FlarePower::default();
        let (ias, app) = (Some(110.0), Some(110.0));
        assert_eq!(power.step(cfg, 0.3, -50.0, 0.5, 10.0, ias, app), Some(0.3));
        assert_eq!(power.last_key, Some(RetardKey::Entry));
        assert!(power.step(cfg, 0.3, -150.0, 0.5, 10.0, ias, app).unwrap() < 0.3);
        // A balloon: power coming off, not held.
        assert!(power.step(cfg, 0.3, 100.0, 0.5, 10.0, ias, app).unwrap() < 0.3);
        // A lagging throttle read must not restart/increase the command.
        assert!(power.step(cfg, 0.4, -100.0, 0.5, 10.0, ias, app).unwrap() < 0.3);
        power.reset();
        assert_eq!(power.step(cfg, 0.45, -300.0, 0.5, 10.0, ias, app), Some(0.45));
        // An unread airspeed is no speed key: the retard goes on by time.
        assert!(power.step(cfg, 0.45, -300.0, 0.5, 10.0, None, app).unwrap() < 0.45);
    }

    #[test]
    fn flare_power_reaches_idle_smoothly_and_rejects_missing_measurements() {
        let cfg = baron_flare_power();
        let mut power = FlarePower::default();
        let (ias, app) = (Some(110.0), Some(110.0));
        let mut throttle = power.step(cfg, 0.3, -100.0, 0.5, 10.0, ias, app).unwrap();
        for _ in 0..30 {
            let next = power.step(cfg, throttle, -100.0, 0.5, 10.0, ias, app).unwrap();
            assert!((0.0..=throttle).contains(&next));
            assert!(throttle - next <= cfg.reduction_per_second * 0.5 + 1e-9);
            throttle = next;
        }
        assert_eq!(throttle, 0.0);
        assert_eq!(power.step(cfg, f64::NAN, -100.0, 0.5, 10.0, ias, app), None);
        assert_eq!(power.step(cfg, 0.3, f64::NAN, 0.5, 10.0, ias, app), None);
        assert_eq!(power.step(cfg, 0.3, -100.0, 0.0, 10.0, ias, app), None);
        assert_eq!(power.step(cfg, 0.3, -100.0, 0.5, f64::NAN, ias, app), None);
        let invalid = crate::aircraft::FlarePowerProfile {
            reduction_per_second: -1.0,
            ..cfg
        };
        assert_eq!(power.step(invalid, 0.3, -100.0, 0.5, 10.0, ias, app), None);
    }
    /// LEMG 13 (2026-09-26, 737): the arrest levelled at 20 ft, the sink-keyed
    /// retard took the lever to idle IN the float, and the sink came back to
    /// -859 fpm. A jet retards by height: nothing above it, and below it a
    /// ramp that reaches idle as the wheels arrive.
    /// LEMG 13, the approach test (2026-09-26): -300 by 29 ft, then the sink
    /// built back to -587 against a target still steeper than the aircraft.
    /// Held to the best sink reached, the law pulls as soon as it grows.
    #[test]
    fn the_flare_never_lets_the_sink_grow_again() {
        let mut law = FlarePitch::default();
        law.step(-450.0, -700.0, 0.0, 0.2, false);
        // Flare: reaches -300 while the target is -636.
        let a = law.step(-636.0, -300.0, 0.0, 0.2, true);
        // The sink grows to -400 with the target still -580: pull more.
        let b = law.step(-580.0, -400.0, -200.0, 0.2, true);
        assert!(b > a, "pulled for the growing sink: {a} -> {b}");
    }

    /// LEMG circuit (2026-09-26): the roundout's pull is where the flare
    /// starts. The flare's first ask (a path target the aircraft is already
    /// shallower than) must not take the column forward at the hand-over.
    #[test]
    fn the_flare_starts_from_the_roundouts_pull() {
        let mut law = FlarePitch::default();
        // Roundout: sinking 860 against 460 wanted, the pull builds.
        let mut pull = 0.0;
        for _ in 0..10 {
            pull = law.step(-460.0, -860.0, 0.0, 0.2, false);
        }
        assert!(pull > 0.2, "{pull}");
        // Flare: the target is the path (-759) and the sink already -690.
        let first = law.step(-759.0, -690.0, 200.0, 0.2, true);
        assert!(first >= pull - 1e-9, "the column held: {first} after {pull}");
    }

    /// LEMG 13 (2026-09-26): the 737 floated from 36 to 19 ft on a flat
    /// -175. Exponential with 4 s, it is the path at the gate, -475 at 20 ft,
    /// -325 at 10, and the touchdown sink at the wheels; undeclared, flat.
    #[test]
    fn the_737s_flare_is_exponential_and_the_rest_stay_flat() {
        let (flare_vs, nominal) = (-175.0, -700.0);
        let t = |h: f64| super::flare_target_with(h, flare_vs, nominal, Some(4.0));
        assert_eq!(t(35.0), -700.0, "the path at the gate, never steeper");
        assert!((t(20.0) + 475.0).abs() < 1.0, "{}", t(20.0));
        assert!((t(10.0) + 325.0).abs() < 1.0, "{}", t(10.0));
        assert!((t(0.0) + 175.0).abs() < 1.0, "{}", t(0.0));
        assert_eq!(super::flare_target_with(20.0, flare_vs, nominal, None), flare_vs);
        // Time from 35 ft: the float is gone (about 5.5 s, not 12).
        let (mut h, mut secs) = (35.0_f64, 0.0);
        while h > 0.5 {
            h += t(h) / 60.0 * 0.1;
            secs += 0.1;
        }
        assert!(secs < 7.0, "{secs:.1} s from 35 ft");
    }

    #[test]
    fn a_jet_retards_by_height_timed_to_the_touchdown() {
        let cfg = crate::aircraft::FlarePowerProfile {
            reduction_per_second: 0.08,
            retard_below_ft: Some(20.0),
            retard_max_per_s: None,
            retard_slow_kts: None,
            retard_at_s: None,
            retard_fast_kts: None,
        };
        let (ias, app) = (Some(145.0), Some(145.0));
        let mut power = FlarePower::default();
        // First tick keeps the approach power; at 36 ft in a float, still held.
        assert_eq!(power.step(cfg, 0.35, -170.0, 0.2, 36.0, ias, app), Some(0.35));
        assert_eq!(power.step(cfg, 0.35, -170.0, 0.2, 30.0, ias, app), Some(0.35));
        assert_eq!(power.last_key, Some(RetardKey::Height));
        // Below 20 ft at 170 fpm: about 7 s to the wheels, about 0.05 a second.
        let next = power.step(cfg, 0.35, -170.0, 1.0, 20.0, ias, app).unwrap();
        assert!((0.29..0.31).contains(&next), "{next}");
        assert_eq!(power.last_key, Some(RetardKey::Timed));
        // A normal sink closer in: a quicker retard, never past the rate.
        let mut quick = FlarePower::default();
        quick.step(cfg, 0.3, -300.0, 0.2, 10.0, ias, app);
        let next = quick.step(cfg, 0.3, -300.0, 1.0, 10.0, ias, app).unwrap();
        assert!((0.3 - 0.08 - 1e-9..0.3).contains(&next), "{next}");
        // Sinking 600 at 12 ft is the SAME retard as 300: the sink the arrest
        // is removing never holds the lever (issue #76), it only shortens
        // the time left.
        let mut hard = FlarePower::default();
        hard.step(cfg, 0.3, -600.0, 0.2, 12.0, ias, app);
        assert!(hard.step(cfg, 0.3, -600.0, 0.2, 12.0, ias, app).unwrap() < 0.3);
        // Slow: 6 kt under the approach speed keeps the thrust at any height.
        let mut slow = FlarePower::default();
        slow.step(cfg, 0.3, -200.0, 0.2, 10.0, Some(139.0), app);
        assert_eq!(slow.step(cfg, 0.3, -200.0, 0.2, 10.0, Some(139.0), app), Some(0.3));
        assert_eq!(slow.last_key, Some(RetardKey::Slow));
        // ...and the speed back, the retard resumes from where it stood.
        assert!(slow.step(cfg, 0.3, -200.0, 0.2, 9.0, Some(142.0), app).unwrap() < 0.3);
    }

    #[test]
    fn a_heading_bias_on_final_parks_beside_the_centreline_without_trim() {
        // The flight, in the plant: 2° left of the bug settles some 40 m
        // left and never closes, and a lateral gate balks it — the go-around
        // the pilot watched twice.
        let (xtk, g, _) = fly_final_with_heading_bias(-2.0, false);
        assert_eq!(
            g.phase,
            ApproachPhase::Intercept,
            "{:.0} m off",
            xtk * super::M_PER_NM
        );
        assert!(
            matches!(
                g.balk,
                Some("off_centreline_low" | "unstabilised_below_floor")
            ),
            "{:?}",
            g.balk
        );
        assert!(
            xtk < -0.015 && xtk > -0.035,
            "expected the standing offset the flight showed, got {:.0} m",
            xtk * super::M_PER_NM
        );
    }

    #[test]
    fn the_track_trim_takes_a_biased_final_to_the_centreline() {
        for bias in [-2.0, 2.0, -1.0] {
            let (xtk, g, trim) = fly_final_with_heading_bias(bias, true);
            assert_eq!(g.phase, ApproachPhase::Final, "bias {bias}: {:?}", g.balk);
            assert!(
                xtk.abs() < 0.005,
                "bias {bias}: still {:.0} m off at 100 ft",
                xtk * super::M_PER_NM
            );
            assert!(
                (trim + bias).abs() < 0.6,
                "bias {bias}: the trim should carry the bias, carries {trim:.2}"
            );
        }
        // No bias: nothing to trim, and the trim invents nothing.
        let (xtk, _, trim) = fly_final_with_heading_bias(0.0, true);
        assert!(
            xtk.abs() < 0.003 && trim.abs() < 0.3,
            "xtk {xtk} trim {trim}"
        );
    }

    #[test]
    fn the_track_trim_is_final_only_and_never_a_turn() {
        let mut t = super::TrackTrim::new();
        // A 30° track error is a turn in progress: not integrated.
        assert_eq!(t.update(ApproachPhase::Final, 90.0, Some(60.0), 1.0), 0.0);
        // The first read of a track measures no rate yet, and two steady
        // seconds are not yet settled: nothing learnt.
        let mut t = super::TrackTrim::new();
        assert_eq!(t.update(ApproachPhase::Final, 90.0, Some(88.0), 3.0), 0.0);
        assert_eq!(t.update(ApproachPhase::Final, 90.0, Some(88.0), 2.0), 0.0);
        // Settled, a steady 2° error integrates at 2/30 per second.
        let v = t.update(ApproachPhase::Final, 90.0, Some(88.0), 3.0);
        assert!((v - 0.2).abs() < 1e-9, "{v}");
        // Held through the flare, whatever the track does.
        assert_eq!(t.update(ApproachPhase::Flare, 90.0, Some(80.0), 5.0), v);
        // A missing track read holds the value.
        assert_eq!(t.update(ApproachPhase::Final, 90.0, None, 5.0), v);
        // Wraps: 359 wanted, 1 flown is -2, not +358.
        let mut w = super::TrackTrim::new();
        for _ in 0..2 {
            w.update(ApproachPhase::Final, 359.0, Some(1.0), 3.0);
        }
        assert!(w.update(ApproachPhase::Final, 359.0, Some(1.0), 3.0) < 0.0);
        // A 5 degree error is a roll-out still converging, not a bias.
        let before = t.value();
        assert_eq!(
            t.update(ApproachPhase::Final, 90.0, Some(85.0), 1.0),
            before
        );
        // Capped, and reset by the intercept (a go-around starts clean).
        for _ in 0..10_000 {
            t.update(ApproachPhase::Final, 90.0, Some(87.5), 1.0);
        }
        assert_eq!(t.value(), super::TRACK_TRIM_MAX_DEG);
        assert_eq!(
            t.update(ApproachPhase::Intercept, 90.0, Some(85.0), 1.0),
            0.0
        );
    }

    /// Issue #189: LEMG 30 A330, sortie 1790710769-full_auto (2026-09-29),
    /// autoland's first 20 s after the circuit's handoff (`autoland` ticks:
    /// seconds, ground track, the guidance's wanted track). Calm air, the
    /// aircraft on its bug: the roll-out off the circuit's intercept taught
    /// the old trim -0.51 degrees in ten seconds (recorded), which held the
    /// final 17 m left and went around at 99 ft. A turning track teaches it
    /// nothing now; the steady error after the roll-out is still learnt.
    #[test]
    fn the_track_trim_learns_no_roll_out() {
        let ticks = [
            (2.0, 301.33, 298.52), (3.3, 301.27, 298.36), (3.8, 301.17, 298.30),
            (4.4, 301.01, 298.24), (5.0, 300.78, 298.17), (5.5, 300.60, 298.13),
            (6.0, 300.37, 298.09), (6.4, 300.10, 298.04), (7.0, 299.80, 298.00),
            (7.6, 299.46, 297.96), (8.0, 299.20, 297.93), (8.4, 298.97, 297.91),
            (8.8, 298.71, 297.89), (9.1, 298.60, 297.89), (9.6, 298.31, 297.87),
            (10.0, 298.10, 297.86), (10.6, 297.87, 297.86), (11.2, 297.66, 297.86),
            (11.6, 297.55, 297.86), (12.1, 297.43, 297.86), (12.5, 297.36, 297.86),
            (13.0, 297.30, 297.87), (13.4, 297.27, 297.87), (13.9, 297.25, 297.88),
            (14.3, 297.25, 297.88), (14.7, 297.26, 297.89), (15.3, 297.28, 297.89),
            (15.6, 297.31, 297.90), (16.1, 297.34, 297.90), (16.5, 297.37, 297.90),
            (17.1, 297.40, 297.91), (17.5, 297.41, 297.91), (17.9, 297.42, 297.91),
            (18.4, 297.43, 297.92), (18.9, 297.42, 297.92), (19.4, 297.40, 297.92),
            (19.8, 297.38, 297.93), (20.3, 297.35, 297.93),
        ];
        let mut t = super::TrackTrim::new();
        // The old rule, the error cap alone, for the record.
        let mut old = 0.0_f64;
        let mut prev = 0.0;
        let mut at_10 = (0.0, 0.0);
        for (i, (s, trk, want)) in ticks.iter().enumerate() {
            let dt = if i == 0 { 0.45 } else { s - prev };
            prev = *s;
            let v = t.update(ApproachPhase::Final, *want, Some(*trk), dt);
            let err = ((want - trk + 540.0_f64).rem_euclid(360.0)) - 180.0;
            if err.abs() <= super::TRACK_TRIM_ERR_CAP_DEG {
                old += err * dt / super::TRACK_TRIM_TAU_S;
            }
            if (*s - 10.0).abs() < 0.05 {
                at_10 = (v, old);
            }
        }
        assert!(at_10.1 < -0.3, "the old rule learns the roll-out: {at_10:?}");
        assert!(at_10.0.abs() < 0.05, "a turning track teaches nothing: {at_10:?}");
        // Rolled out 0.5 to 0.6 degrees left of what it wants: learnt RIGHT.
        assert!(t.value() > 0.02, "{}", t.value());
    }

    #[test]
    fn crab_filter_settles_on_a_steady_crosswind() {
        let mut f = super::CrabFilter::new();
        let mut out = 0.0;
        for _ in 0..90 {
            out = f.update(Some(6.0), 0.5);
        }
        assert!((out - 6.0).abs() < 0.01, "45 s in, got {out}");
        // A fresh filter takes its first read whole — no 15 s ramp from zero
        // on the tick the final begins.
        let mut g = super::CrabFilter::new();
        assert_eq!(g.update(Some(6.0), 0.5), 6.0);
    }

    #[test]
    fn crab_filter_eats_the_lemg_13_hunt() {
        let mut f = super::CrabFilter::new();
        let mut worst = 0.0_f64;
        for i in 0..600 {
            let t = i as f64 * 0.5;
            let raw = 2.0 + 9.0 * (std::f64::consts::TAU * t / 13.0).sin();
            let out = f.update(Some(raw), 0.5);
            if t > 60.0 {
                worst = worst.max((out - 2.0).abs());
            }
        }
        assert!(worst < 2.0, "the swing came through at {worst:.1} degrees");
        assert!(
            worst > 0.5,
            "a filter that passes nothing is a constant, got {worst:.2}"
        );
    }

    #[test]
    fn crab_filter_holds_through_a_missed_read() {
        let mut f = super::CrabFilter::new();
        for _ in 0..90 {
            f.update(Some(6.0), 0.5);
        }
        assert!(
            (f.update(None, 0.5) - 6.0).abs() < 0.01,
            "a missed read is not a zero crab"
        );
        assert_eq!(
            super::CrabFilter::new().update(None, 0.5),
            0.0,
            "never read = zero"
        );
        // Wrap-safe: 359 vs 3 is a 4 degree crab, not 356.
        assert!((super::CrabFilter::raw(Some(359.0), Some(3.0)).unwrap() + 4.0).abs() < 1e-9);
        assert_eq!(super::CrabFilter::raw(Some(90.0), None), None);
    }

    #[test]
    fn a_clearance_for_another_end_differs_and_blanks_do_not() {
        assert!(super::clearance_differs("12", "13"));
        assert!(!super::clearance_differs("12", " 12 "));
        assert!(!super::clearance_differs("09R", "09r"));
        assert!(!super::clearance_differs("12", ""));
    }

    /// LXGB 2026-09-14: "final runway two seven" earned "negative, runway
    /// zero niner ... cleared to land" five seconds after the arm-time pick,
    /// and nothing looked again.
    #[test]
    fn a_landing_clearance_for_another_end_moves_the_approach_once() {
        use crate::tower::State::*;
        let sw = |f, st, c, a| super::clearance_switch(f, st, c, a, false, None);
        assert_eq!(
            sw("27", Some(ClearedToLand), Some("09"), None),
            Some("09".into())
        );
        // EGTE 2026-09-21: the go-around's own state change published the
        // flipped runway in use. A missed approach is never re-aimed.
        assert_eq!(
            super::clearance_switch("08", Some(Inbound), Some("26"), None, true, None),
            None
        );
        // Told to join the other end on the inbound call: same switch, earlier.
        assert_eq!(sw("27", Some(Inbound), Some("09"), None), Some("09".into()));
        // Looked up once: the apt.dat pass is not repeated every tick.
        assert_eq!(sw("27", Some(ClearedToLand), Some("09"), Some("09")), None);
        assert_eq!(sw("27", Some(ClearedToLand), Some("09"), Some("09 ")), None);
        // Our own end, a blank, or no clearance at all: nothing to do.
        assert_eq!(sw("27", Some(ClearedToLand), Some("27"), None), None);
        assert_eq!(sw("27", Some(ClearedToLand), Some(""), None), None);
        assert_eq!(sw("27", Some(ClearedToLand), None, None), None);
        // A departure's runway is the departure's, whatever end it names.
        assert_eq!(sw("27", Some(TakeoffCleared), Some("09"), None), None);
        assert_eq!(sw("27", Some(TaxiCleared), Some("09"), None), None);
        assert_eq!(sw("27", None, Some("09"), None), None);
    }

    /// #182, LEMG 2026-09-29 (sortie 1790668215-full_auto): at t+649.4 the
    /// tower sent the 737 round off 30 ("go around, traffic on the runway
    /// ... expect runway one three", atc `resequenced` 30 to 13, oncoming),
    /// and the FSM sync published 13 with `Inbound`. The missed approach
    /// read no switch; the reposition flew a right-hand pattern for 30 from
    /// t+661.3 and found 13 only at t+984.6, once `reversal_done` ended the
    /// missed approach. The order's own runway is a re-sequence: taken once
    /// the climb away is flown, and only when the clearance agrees with it.
    #[test]
    fn a_go_around_order_that_resequences_moves_the_approach_after_the_climb() {
        use crate::tower::State::{ClearedToLand, Inbound};
        // Past the climb (the caller passes the order's runway): 13 now.
        assert_eq!(
            super::clearance_switch("30", Some(Inbound), Some("13"), None, true, Some("13")),
            Some("13".into())
        );
        // In the climb, or with an order that named none (EGTE's runway in
        // use flipping under a go-around): the missed approach is flown.
        assert_eq!(
            super::clearance_switch("30", Some(Inbound), Some("13"), None, true, None),
            None
        );
        // The clearance must agree with the order: a runway in use that has
        // moved on again is not what the tower told us to expect.
        assert_eq!(
            super::clearance_switch("30", Some(Inbound), Some("12"), None, true, Some("13")),
            None
        );
        // Looked up once, and never onto a departure's runway.
        assert_eq!(
            super::clearance_switch(
                "30",
                Some(Inbound),
                Some("13"),
                Some("13"),
                true,
                Some("13")
            ),
            None
        );
        assert_eq!(
            super::clearance_switch(
                "30",
                Some(crate::tower::State::TakeoffCleared),
                Some("13"),
                None,
                true,
                Some("13")
            ),
            None
        );
        // Already flying the expected end: nothing to do.
        assert_eq!(
            super::clearance_switch(
                "13",
                Some(ClearedToLand),
                Some("13"),
                None,
                true,
                Some("13")
            ),
            None
        );
        // The latch says when the climb is flown: straight ahead to 800 ft
        // radio, then the reposition (the recorded 748 ft order, 856 ft at
        // the pattern's start).
        let mut latch = super::GoAroundLatch::new();
        assert!(latch.order(false));
        assert!(latch.climbing());
        latch.update(
            Some(ApproachPhase::Final),
            ApproachPhase::Intercept,
            748.0,
            1.53,
            1.53,
            5.75,
            0.008,
        );
        assert!(latch.climbing(), "still climbing away at 748 ft");
        latch.update(
            Some(ApproachPhase::Intercept),
            ApproachPhase::Intercept,
            856.0,
            1.0,
            1.0,
            5.75,
            0.009,
        );
        assert!(
            !latch.climbing() && latch.active(),
            "repositioning: the re-sequence is taken"
        );
    }

    #[test]
    fn only_the_landing_echo_is_the_approachs_to_make() {
        assert!(super::landing_readback(
            "Cleared to land runway 09, November one seven two Sierra Papa."
        ));
        assert!(super::landing_readback(
            "Cleared touch and go runway 09, N172SP."
        ));
        assert!(!super::landing_readback(
            "Cleared for takeoff runway 09, N172SP."
        ));
        assert!(!super::landing_readback("Holding position, N172SP."));
        assert!(!super::landing_readback("Lining up runway 09, N172SP."));
    }

    use super::*;

    #[test]
    fn rollout_requires_weight_on_wheels() {
        // A normal flare can be below five feet and slightly below target;
        // neither airspeed nor radio altitude is proof of touchdown.
        assert!(!should_start_rollout(false, false, 3.0, 70.0, 89.0, 90.0, None));
        assert!(!should_start_rollout(false, false, 3.0, 40.0, 89.0, 90.0, None));
        // The same committed state starts rollout once the wheels are down,
        // including a tailwind landing whose groundspeed remains high.
        assert!(should_start_rollout(false, true, 3.0, 70.0, 89.0, 90.0, None));
        assert!(should_start_rollout(false, true, 3.0, 40.0, 95.0, 90.0, None));
        // Once latched, a noisy WOW sample cannot return control to approach.
        assert!(should_start_rollout(true, false, 10.0, 70.0, 95.0, 90.0, None));
    }

    /// LEMG 12 (2026-09-26, 17:52Z, a 737, approach speed 145): the guidance
    /// ticks from the last airborne one to the balk, as recorded (t from
    /// 1790448340.0, the sim's on_ground, radio ft, IAS; 146 kt over the
    /// ground throughout). The speed rule never latched the rollout, the
    /// flare ran 2.8 s on the wheels, and at 3.3 s `overflew_the_runway`
    /// balked from 0.33 ft into a full-power go-around and a "going around"
    /// call to the tower.
    #[test]
    fn the_wheels_are_a_landing_and_nothing_balks_from_the_pavement() {
        const TICKS: &[(f64, bool, f64, f64)] = &[
            (0.0, false, 3.06, 147.42),
            (0.3, true, 0.97, 147.34),
            (0.5, true, -0.01, 147.28),
            (0.7, true, 0.33, 147.21),
            (1.0, true, 0.90, 147.08),
            (1.2, true, 0.94, 146.99),
            (1.4, true, 0.58, 146.89),
            (1.7, true, 0.28, 146.73),
            (1.9, true, 0.38, 146.60),
            (2.2, true, 0.37, 146.45),
            (2.4, true, 0.30, 146.29),
            (2.6, true, 0.32, 146.12),
            (2.8, true, 0.35, 145.95),
            (3.1, true, 0.32, 145.65),
            (3.3, true, 0.33, 145.46),
        ];
        let (gs, approach) = (146.0, 145.0);
        // (1) The old rule: not one tick latched.
        assert!(TICKS.iter().all(|&(_, g, r, ias)| !should_start_rollout(false, g, r, gs, ias, approach, None)));
        // The wheels: latched 0.4 s after first contact (the third tick on
        // the ground at this 0.2 s tick).
        let mut n = None;
        let mut last_t: Option<f64> = None;
        let latched = TICKS.iter().find(|&&(t, g, r, ias)| {
            let dt = last_t.map_or(0.0, |t0| t - t0);
            last_t = Some(t);
            n = wheels_on_time(n, g, r, dt);
            should_start_rollout(false, g, r, gs, ias, approach, n)
        });
        assert_eq!(latched.map(|t| t.0), Some(0.7), "the rollout latches at the wheels");
        // A bounce breaks the clock.
        assert_eq!(wheels_on_time(Some(0.2), false, 4.0, 0.2), None);
        assert_eq!(wheels_on_time(Some(0.2), true, 6.0, 0.2), None, "a wheel brushing at 6 ft is not down");
        // The same 0.4 s on a slower tick: 0.39 s ticks latch on the
        // second read after contact, not the third (0.78 s).
        let mut s = wheels_on_time(None, true, 0.5, 0.39);
        assert!(!should_start_rollout(false, true, 0.5, gs, 147.0, approach, s));
        s = wheels_on_time(s, true, 0.5, 0.39);
        assert!(!should_start_rollout(false, true, 0.5, gs, 147.0, approach, s));
        s = wheels_on_time(s, true, 0.5, 0.39);
        assert!(should_start_rollout(false, true, 0.5, gs, 147.0, approach, s));

        // (2) The balk tick as the guidance produced it.
        let balked = Guidance {
            target_heading_deg: 117.71,
            target_vs_fpm: 2000.0,
            target_ias_kts: 150.0,
            dist_nm: 0.61,
            glidepath_alt_ft: 292.3,
            phase: ApproachPhase::Intercept,
            balk: Some("overflew_the_runway"),
            arresting: false,
        };
        let (held, refused) = wheels_hold_the_landing(balked, true, true, 117.3);
        assert_eq!(refused, Some("overflew_the_runway"));
        assert_eq!((held.phase, held.balk), (ApproachPhase::Flare, None));
        assert!((held.target_heading_deg - 117.3).abs() < 1e-9, "down the runway's course");
        // The go-around latch never arms off it; off the raw balk it did.
        let mut latch = GoAroundLatch::new();
        assert!(latch
            .update(Some(ApproachPhase::Flare), held.phase, 0.33, 0.61, -0.61, FAP_NM, 0.0)
            .is_none());
        let mut latch = GoAroundLatch::new();
        assert_eq!(
            latch.update(Some(ApproachPhase::Flare), balked.phase, 0.33, 0.61, -0.61, FAP_NM, 0.0),
            Some(MissedStage::Climb),
            "the recorded go-around"
        );
        // Airborne, the balk stands (an overflight is still a go-around).
        let (air, none) = wheels_hold_the_landing(balked, false, true, 117.3);
        assert_eq!((air.phase, air.balk, none), (ApproachPhase::Intercept, Some("overflew_the_runway"), None));
        // A flare that stays a flare on the ground is untouched.
        let flare = Guidance { phase: ApproachPhase::Flare, balk: None, arresting: true, ..balked };
        assert_eq!(wheels_hold_the_landing(flare, true, true, 117.3).1, None);

        // (4) No "going around" call with the wheels on the runway.
        assert!(!go_around_call_due(true, false, true, None, 0.3, None));
        assert!(go_around_call_due(true, false, false, None, 300.0, None));
        assert!(!go_around_call_due(true, true, false, None, 300.0, None), "once");
        assert!(!go_around_call_due(false, false, false, None, 300.0, None));
    }

    /// LFBD 11, 2026-09-27: the 737 balked `overflew_the_runway` at 7.9 ft
    /// radio sinking 184 fpm, "going around" went out on that tick and the
    /// tower dropped the landing clearance, and the wheels touched 1.3 s
    /// later. The call now waits for the climb; a balk the wheels take never
    /// calls. The first approach's `off_centreline_low` at 96 ft still calls
    /// as soon as it climbs.
    /// EGKK-EGBB 15, sortie 1790601822-full_auto (2026-09-28, #155): the
    /// final report at +3314.2 got "continue approach, expect late landing
    /// clearance" and the tower never came back; the book stayed Inbound down
    /// the whole final (the recorded autoland ticks' radio heights below,
    /// +3313.6 to +3452.0) and the wheels touched uncleared at +3472.2. The
    /// crew now asks at 1,000 ft (the +3390.8 tick, 990 ft) and goes around at
    /// 300 (the +3435.7 tick, 296 ft), once each; cleared, or with no tower
    /// working us, it says nothing; a missed approach asks again next time.
    #[test]
    fn an_uncleared_final_asks_then_goes_around() {
        let radio = [
            2103.0, 2097.0, 1991.0, 1886.0, 1796.0, 1692.0, 1593.0, 1499.0, 1394.0, 1298.0,
            1200.0, 1096.0, 990.0, 893.0, 795.0, 694.0, 596.0, 499.0, 396.0, 296.0, 199.0, 95.0,
        ];
        let run = |working: bool| {
            let mut w = UnclearedWatch::default();
            radio
                .iter()
                .filter_map(|&r| w.step(working, r, false, false).map(|s| (r, s)))
                .collect::<Vec<_>>()
        };
        let steps = run(true);
        assert_eq!(steps.first(), Some(&(990.0, UnclearedStep::Ask)));
        assert_eq!(steps.get(1), Some(&(296.0, UnclearedStep::GoAround)));
        assert_eq!(
            steps.iter().filter(|s| s.1 == UnclearedStep::Ask).count(),
            1,
            "asked once: {steps:?}"
        );
        // Cleared (or no tower working us): silent all the way down.
        assert!(run(false).is_empty());
        // Never on the wheels.
        let mut w = UnclearedWatch::default();
        assert_eq!(w.step(true, 0.0, true, false), None);
        // A missed approach resets the question for the next approach.
        let mut w = UnclearedWatch::default();
        assert_eq!(w.step(true, 900.0, false, false), Some(UnclearedStep::Ask));
        assert_eq!(w.step(true, 900.0, false, false), None);
        assert_eq!(w.step(true, 250.0, false, true), None);
        assert_eq!(w.step(true, 900.0, false, false), Some(UnclearedStep::Ask));
        // An unread height is no measurement.
        assert_eq!(UnclearedWatch::default().step(true, f64::NAN, false, false), None);
    }

    #[test]
    fn a_low_balk_is_called_on_the_radio_only_once_it_climbs() {
        let due = |w: &BalkWatch, on_ground, radio, vvi| {
            go_around_call_due(true, false, on_ground, Some(w), radio, vvi)
        };
        // The recorded balk: 7.9 ft, sinking, spooling.
        let mut w = BalkWatch::new(Some("overflew_the_runway"), 7.9);
        assert!(!due(&w, false, 7.9, Some(-184.0)), "not on the balk tick");
        w.note(4.0, false);
        assert!(!due(&w, false, 4.0, Some(-150.0)));
        // The wheels take it: never called, whatever follows under 200 ft.
        w.note(0.4, true);
        assert!(!due(&w, true, 0.4, Some(-60.0)), "on the runway");
        w.note(15.0, false);
        assert!(!due(&w, false, 15.0, Some(300.0)), "a bounce is not a go-around");
        // A touch that really leaves is called once clear of 200 ft.
        w.note(GO_AROUND_CALL_AGL_FT, false);
        assert!(due(&w, false, GO_AROUND_CALL_AGL_FT, Some(900.0)));

        // The same balk flown away without touching: called on the climb.
        let mut w = BalkWatch::new(Some("overflew_the_runway"), 7.9);
        w.note(3.0, false);
        assert!(!due(&w, false, 3.0 + GO_AROUND_RISE_FT - 1.0, Some(200.0)), "not yet risen");
        w.note(3.0 + GO_AROUND_RISE_FT, false);
        assert!(due(&w, false, 3.0 + GO_AROUND_RISE_FT, Some(400.0)));
        assert!(!due(&w, false, 3.0 + GO_AROUND_RISE_FT, Some(-50.0)), "the VSI says sinking");
        assert!(due(&w, false, 3.0 + GO_AROUND_RISE_FT, None), "no VSI: the radio's rise stands");
        assert!(!go_around_call_due(true, true, false, Some(&w), 40.0, Some(400.0)), "once");

        // The first LFBD approach: off_centreline_low at 96 ft, settling to
        // 88 before the power bites, called as soon as it climbs.
        let mut w = BalkWatch::new(Some("off_centreline_low"), 96.0);
        w.note(88.0, false);
        assert!(!due(&w, false, 88.0, Some(-300.0)));
        w.note(99.0, false);
        assert!(due(&w, false, 99.0, Some(500.0)));
        // A balk above the call height is called on its own tick.
        let w = BalkWatch::new(Some("unstabilised_at_gate"), 1000.0);
        assert!(due(&w, false, 1000.0, Some(-700.0)));
    }

    /// The rollout brakes by wanted deceleration, not a fixed clamp. A flat
    /// 0.45 measured ~3.3 m/s² from 79 kt to a dead stop — "brakes really
    /// really hard" from the right seat (EGHI, 2026-08-02) — on a runway with
    /// over a kilometre to spare.
    #[test]
    fn rollout_brakes_for_comfort_unless_the_runway_disagrees() {
        // Plenty of pavement: the comfort rate, well under the old 0.45.
        let easy = rollout_brake_ratio(79.0, 0.0, 1300.0);
        assert!((0.2..0.3).contains(&easy), "{easy}");
        // Short remaining: physics outranks comfort.
        let tight = rollout_brake_ratio(79.0, 0.0, 300.0);
        assert!(tight > 0.5, "{tight}");
        // ...and the cap holds even when the sums say more.
        assert_eq!(rollout_brake_ratio(120.0, 0.0, 80.0), 0.8);
        // Slowing to a vacate handoff needs less than slowing to a stop.
        assert!(
            rollout_brake_ratio(40.0, VACATE_HANDOFF_KTS, 500.0)
                <= rollout_brake_ratio(40.0, 0.0, 500.0)
        );
        // Near the target the floor still holds a gentle squeeze on.
        assert!(rollout_brake_ratio(12.0, 10.0, 800.0) >= 0.2);
    }

    // Gibraltar 2026-09-11: the only lateral balk was the flare gate under
    // 100 ft, so a final that never converged flew to 95 ft still 306 m off
    // the centreline, went around with no height in hand, and hit the North
    // Front. Below the terrain floor, off the course is a go-around.
    #[test]
    fn an_unstabilised_final_balks_with_height_in_hand() {
        let rwy = rwy_north();
        // The crash geometry at the floor: 1.5 nm out, 480 ft, 0.9 nm right.
        let (clat, clon) = project_latlon(0.0, 0.0, 180.0, 1.5);
        let (lat, lon) = project_latlon(clat, clon, 90.0, 0.9);
        let g = guidance(lat, lon, 480.0, 480.0, 90.0, 100.0, &rwy, 100.0, 25.0, 0.0);
        assert_eq!(
            g.phase,
            ApproachPhase::Intercept,
            "balk at the floor, not at the flare gate"
        );
        // A normal final 40 m off at the same point: still on final.
        let (lat, lon) = project_latlon(clat, clon, 90.0, 0.02);
        let g = guidance(lat, lon, 480.0, 480.0, 90.0, 100.0, &rwy, 100.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        // Above the floor the same geometry is not a balk but a REPOSITION:
        // steer out to the join point instead of calling it final. The RV-10
        // "Land the aircraft" at 1.5 nm abeam, 660 ft (2026-09-11).
        let (clat, clon) = project_latlon(0.0, 0.0, 180.0, 1.06);
        let (lat, lon) = project_latlon(clat, clon, 90.0, 1.06);
        let g = guidance(lat, lon, 660.0, 660.0, 120.0, 120.0, &rwy, 80.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Intercept, "not a final from there");
        // The join point lies down the extended centreline, south of us and
        // west: a heading in the south-west quadrant, never the runway's.
        assert!(
            (160.0..=260.0).contains(&g.target_heading_deg),
            "{}",
            g.target_heading_deg
        );

        // The rule itself: the offset the steepest cut can still close with
        // margin in the distance left. No opinion outside the FAP (an
        // intercept is allowed to be wide out there).
        assert!(!stabilised_xtk(0.9, 1.5), "the Gibraltar handoff");
        assert!(stabilised_xtk(0.10, 1.5));
        assert!(stabilised_xtk(0.2, 0.8), "the pursuit flies this one out");
        assert!(!stabilised_xtk(0.2, 0.3));
    }

    /// The flare is entered from a roundout, never from a level float. Past
    /// the threshold the unsigned glidepath distance grows again, alt_err
    /// goes negative, and the level-hold branch commanded 0 fpm at 27 ft over
    /// the runway: the aircraft floated, bled the speed the flare needed, and
    /// dropped on at -380 fpm one knot above the clean stall (EGHI,
    /// 2026-08-02).
    #[test]
    fn the_flare_is_entered_from_a_roundout_not_a_float() {
        let mut rwy = rwy_north();
        // Short final, over the pavement environment, just above the flare
        // gate (radio 40 ft, flare 25, band 50): a gentle continued descent.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.2);
        let g = guidance(lat, lon, 40.0, 40.0, 95.0, 95.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        let (roundout, _) = arrest_targets_fpm(-(95.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert_eq!(g.target_vs_fpm, roundout);

        // The live float case: just past the threshold (inside the overflight
        // margin), 30 ft radio — alt_err is negative here and the old law
        // held LEVEL. It must keep descending.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.05);
        let g = guidance(lat, lon, 30.0, 30.0, 93.0, 93.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        // The roundout's target scales with the path sink (a 95 kt approach
        // sinks 500 fpm, so its roundout is 300, not the 172's 250).
        let (roundout, _) = arrest_targets_fpm(-(93.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert!(
            roundout < ROUNDOUT_VS_FPM && roundout > -350.0,
            "{roundout}"
        );
        assert_eq!(
            g.target_vs_fpm, roundout,
            "a level float at 30 ft is how the mush happened"
        );

        // Below the flare gate the flare still owns the touchdown.
        let g = guidance(lat, lon, 20.0, 20.0, 92.0, 92.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Flare);
        let (_, flare) = arrest_targets_fpm(-(92.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert!(flare < FLARE_VS_FPM && flare > -150.0, "{flare}");
        assert_eq!(g.target_vs_fpm, flare);

        // Dragged LOW short of the threshold the roundout must NOT own it:
        // from 30 ft at 0.3 nm its -250 grounds ~190 m before the pavement.
        // The level-hold holds, and the path comes down to us — the guard is
        // the physical reach test, not the distance band (audit, 2026-08-02).
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.3);
        let g = guidance(lat, lon, 30.0, 30.0, 100.0, 100.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        assert_eq!(
            g.target_vs_fpm, 0.0,
            "a commanded descent here lands in the approach lights"
        );

        // And a genuine overflight go-around is untouched by the roundout:
        // 150 ft over the pavement 0.3 nm in, where a 3 degree descent
        // reaches the band 550 m further on. On this 3,000 m test runway
        // that still leaves the room a landing needs (the rule is
        // geometric now), so the strip is 1,800 m here: the band would be
        // reached 1,100 m in with 700 m left, and that IS an overflight.
        rwy.length_m = 1800.0;
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.3);
        let g = guidance(lat, lon, 150.0, 150.0, 95.0, 95.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
        assert_eq!(g.balk, Some("overflew_the_runway"));
    }

    /// LXGB 27, 2026-09-14, replayed through the vertical law against the
    /// autopilot it was talking to: a VS mode that follows its dial as a
    /// first-order lag of 4.5 s behind one tick of dead time, and the flare's
    /// throttle cut as a sink disturbance of 250 fpm arriving over 1.5 s that
    /// the same lag has to reject. Both fitted to the tick log: -250 dialled
    /// at 49 ft, still -386 four seconds on; -100 dialled at 23 ft with the
    /// throttle closed, the sink eased to -257 and then GREW back to -309 at
    /// the wheels. A pure lag without the disturbance lands this at -140 and
    /// would have passed the old law. Flies from 60 ft radio just short of
    /// the numbers at the 172's approach sink, the dial either the bare
    /// target (the live sortie) or led through [`vs_dial_fpm`]. Returns the
    /// sink rate at the wheels, the shallowest rate seen (a balloon would be
    /// positive) and the seconds it took.
    fn fly_arrest_through_a_lagging_autopilot(lead: bool) -> (f64, f64, f64) {
        let rwy = rwy_north();
        let dt = 0.6;
        let tau_s = 4.5;
        let idle_sink_fpm = -250.0;
        let idle_tau_s = 1.5;
        let gs_kts = 75.0;
        let mut radio = 60.0;
        let mut vs = -400.0;
        let mut pending_dial = vs; // one tick of dead time on the dial
        let mut idle_sink = 0.0;
        let mut along_nm = -0.12; // short of the threshold, on the centreline
        let mut shallowest: f64 = vs;
        let mut t = 0.0;
        let mut saw_roundout = false;
        let mut saw_flare = false;
        loop {
            let (lat, lon) = if along_nm < 0.0 {
                project_latlon(0.0, 0.0, 180.0, -along_nm)
            } else {
                project_latlon(0.0, 0.0, 0.0, along_nm)
            };
            let g = guidance(lat, lon, radio, radio, gs_kts, 65.0, &rwy, 65.0, 25.0, 0.0);
            assert_ne!(
                g.phase,
                ApproachPhase::Intercept,
                "balked at {radio:.0} ft: {:?}",
                g.balk
            );
            saw_roundout |= g.phase == ApproachPhase::Final && g.arresting;
            saw_flare |= g.phase == ApproachPhase::Flare;
            let dial = if lead {
                vs_dial_fpm(g.target_vs_fpm, vs, 0.0, g.arresting, Default::default())
            } else {
                g.target_vs_fpm
            };
            let applied = pending_dial;
            pending_dial = dial;
            // The throttle closes at the flare (the loop's own rule) and the
            // extra sink enters the state; the autopilot only sees it as
            // vertical-speed error and works it off through its lag.
            let want_sink = if g.phase == ApproachPhase::Flare {
                idle_sink_fpm
            } else {
                0.0
            };
            let d_sink = (want_sink - idle_sink) * (dt / idle_tau_s);
            idle_sink += d_sink;
            vs += (applied - vs) * (dt / tau_s) + d_sink;
            shallowest = shallowest.max(vs);
            radio += vs / 60.0 * dt;
            along_nm += gs_kts * dt / 3600.0;
            t += dt;
            if radio <= 0.0 {
                assert!(
                    saw_roundout && saw_flare,
                    "the arrest must run roundout then flare"
                );
                return (vs, shallowest, t);
            }
            assert!(t < 60.0, "never touched down");
        }
    }

    /// The flare must arrive at the wheels, not at the dial. The bare target
    /// reproduces the live arrival (three firm 172 landings in three days,
    /// -309 fpm at LXGB); the led dial arrests it inside the same height,
    /// without a balloon and without floating the runway away.
    #[test]
    fn the_landing_arrest_leads_the_autopilots_lag() {
        let (bare_vs, _, bare_t) = fly_arrest_through_a_lagging_autopilot(false);
        assert!(
            bare_vs < -250.0,
            "the bare dial should reproduce the firm arrival, got {bare_vs:.0}"
        );

        let (led_vs, shallowest, led_t) = fly_arrest_through_a_lagging_autopilot(true);
        assert!(
            led_vs > -170.0,
            "led dial still firm: {led_vs:.0} fpm at the wheels"
        );
        assert!(
            led_vs < -40.0,
            "the arrest must still land, got {led_vs:.0}"
        );
        assert!(shallowest < 0.0, "ballooned to {shallowest:.0} fpm");
        assert!(
            led_t < bare_t + 8.0,
            "floated: {led_t:.1} s vs {bare_t:.1} s"
        );

        // Outside an arrest the dial is the target, exactly as before.
        let lead = crate::aircraft::ArrestLeadProfile::default();
        assert_eq!(
            (lead.gain, lead.max_climb_fpm, lead.trend_seconds),
            (VS_LEAD_GAIN, VS_LEAD_MAX_CLIMB_FPM, 0.0)
        );
        assert_eq!(vs_dial_fpm(-380.0, -600.0, 0.0, false, lead), -380.0);
        // Inside one it leads by the outstanding error, capped at a gentle climb.
        assert_eq!(vs_dial_fpm(-100.0, -300.0, 0.0, true, lead), 300.0);
        assert_eq!(vs_dial_fpm(-100.0, -150.0, 0.0, true, lead), 0.0);
        assert_eq!(vs_dial_fpm(-100.0, -100.0, 0.0, true, lead), -100.0);
        assert_eq!(vs_dial_fpm(-100.0, 200.0, 0.0, true, lead), -700.0);
        // The 172 has no trend term: a trend changes nothing for it.
        assert_eq!(vs_dial_fpm(-100.0, -300.0, 150.0, true, lead), 300.0);
        // A jet's lead predicts the sink `trend_seconds` ahead and caps low:
        // sinking 300 but easing at 100 fpm/s reads as 150 in 1.5 s.
        let jet = crate::aircraft::ArrestLeadProfile {
            gain: 1.5,
            max_climb_fpm: 100.0,
            trend_seconds: 1.5,
        };
        assert_eq!(vs_dial_fpm(-100.0, -300.0, 100.0, true, jet), -25.0);
        assert_eq!(vs_dial_fpm(-100.0, -700.0, 0.0, true, jet), 100.0);
        assert_eq!(
            vs_dial_fpm(-100.0, -300.0, f64::NAN, true, jet),
            200.0_f64.min(100.0)
        );
    }

    /// The gates and targets are seconds of sink: a 172 keeps its feet and
    /// fpm exactly, a jet's open higher and aim firmer.
    #[test]
    fn arrest_gates_and_targets_scale_with_the_path_sink() {
        // 172 at 65-78 kt over the ground: 25 / 50 ft, -250 / -100 fpm.
        for gs in [55.0, 65.0, 75.0] {
            assert_eq!(arrest_gates_ft(25.0, gs), (25.0, 50.0), "gs {gs}");
            let (r, f) = arrest_targets_fpm(-(gs / 60.0) * GLIDEPATH_FT_PER_NM);
            assert_eq!((r, f), (ROUNDOUT_VS_FPM, FLARE_VS_FPM), "gs {gs}");
        }
        // The 737 on EGCC 23L, 132 kt over the ground: 700 fpm on the path.
        let nominal = -(132.0 / 60.0) * GLIDEPATH_FT_PER_NM;
        let (flare, band) = arrest_gates_ft(25.0, 132.0);
        assert!((flare - 35.0).abs() < 1.0, "{flare}");
        assert!((band - 76.0).abs() < 1.0, "{band}");
        let (r, f) = arrest_targets_fpm(nominal);
        assert!((r - -420.0).abs() < 2.0, "{r}");
        assert!((f - -175.0).abs() < 2.0, "{f}");
        // A failed (zero) groundspeed read is the fixed gates, never zero.
        assert_eq!(arrest_gates_ft(25.0, 0.0), (25.0, 50.0));
    }

    /// EGCC 23L, 2026-09-19, replayed: the 737 crossed the numbers at 100 ft
    /// sinking 730 fpm, the level-hold branch dialled ZERO at 60 ft over
    /// the pavement, and the first approach balked "overflew the runway" at
    /// 57 ft and 0.174 nm with 2,700 m of runway ahead.
    #[test]
    fn a_jet_over_the_pavement_keeps_its_sink_and_is_not_an_overflight() {
        let mut rwy = rwy_north();
        rwy.length_m = 3040.0;
        let gs = 132.0;
        let nominal = -(gs / 60.0) * GLIDEPATH_FT_PER_NM;
        // 60 ft over the runway, 150 m in, above the 76 ft band? No: in it.
        // 90 ft, 100 m in: above the band, alt_err negative, the old law
        // held level. Now: the path's own sink, and still Final.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 100.0 / M_PER_NM);
        let g = guidance(lat, lon, 90.0, 90.0, gs, 145.0, &rwy, 145.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        assert_eq!(
            g.target_vs_fpm, nominal,
            "a jet over the pavement keeps descending"
        );
        assert!(!g.arresting);
        // The first balk: 57 ft, 0.174 nm past the threshold, in the band.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.174);
        let g = guidance(lat, lon, 57.0, 57.0, gs, 145.0, &rwy, 145.0, 25.0, 0.0);
        assert_eq!(
            g.phase,
            ApproachPhase::Final,
            "this was a landing: {:?}",
            g.balk
        );
        assert!(g.arresting);
        let (roundout, _) = arrest_targets_fpm(nominal);
        assert_eq!(g.target_vs_fpm, roundout);
        // The second: 11 ft, 0.83 nm in, IS past what 3,040 m allows a
        // landing that still needs 900 m (the GA figure; the profile's
        // 1,634 m is stricter still): the overflight verdict stands there.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.83);
        let g = guidance(lat, lon, 11.0, 11.0, gs, 155.0, &rwy, 145.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
        assert_eq!(g.balk, Some("overflew_the_runway"));
    }

    /// The 737's vertical-speed mode, fitted to the LFPG 09R tick log
    /// (2026-09-22, the dial stepped from -788 to +30 at the roundout gate
    /// and to +100 at the flare's, the sink -801 to -699 in six seconds):
    /// two first-order lags of 5 s in series and three seconds of dead time
    /// (four ticks). The EGCC fit before it (1.5 s lags, 2 s dead) was
    /// three times too quick, which is how the dial-path tests passed
    /// while the aircraft arrived at -672. Flown from 100 ft over the
    /// numbers at the path sink, gs 132, with the flare's thrust reduction
    /// as the same sink disturbance the 172 plant carries. Returns the sink
    /// at the wheels, the shallowest rate seen (a balloon is positive), the
    /// metres of runway used and whether the flare gate was reached.
    fn fly_jet_arrest(lead: crate::aircraft::ArrestLeadProfile) -> (f64, f64, f64, bool) {
        let mut rwy = rwy_north();
        rwy.length_m = 3040.0;
        let dt = 0.7;
        let (t1, t2) = (5.0, 5.0);
        let dead_ticks = 4;
        let gs_kts = 132.0;
        let nominal = -(gs_kts / 60.0) * GLIDEPATH_FT_PER_NM;
        let mut radio = 100.0;
        let mut vs = nominal;
        let mut vs_rate = 0.0;
        let mut dials = std::collections::VecDeque::from(vec![vs; dead_ticks]);
        let mut idle_sink = 0.0;
        let mut along_nm = 0.0;
        let mut shallowest: f64 = vs;
        let mut saw_flare = false;
        let mut prev_vs = vs;
        let mut t = 0.0;
        loop {
            let (lat, lon) = project_latlon(0.0, 0.0, 0.0, along_nm);
            let g = guidance(
                lat, lon, radio, radio, gs_kts, 145.0, &rwy, 145.0, 25.0, 0.0,
            );
            assert_ne!(
                g.phase,
                ApproachPhase::Intercept,
                "balked at {radio:.0} ft: {:?}",
                g.balk
            );
            saw_flare |= g.phase == ApproachPhase::Flare;
            let trend = (vs - prev_vs) / dt;
            prev_vs = vs;
            dials.push_back(vs_dial_fpm(g.target_vs_fpm, vs, trend, g.arresting, lead));
            let applied = dials.pop_front().unwrap();
            let want_sink = if g.phase == ApproachPhase::Flare {
                -250.0
            } else {
                0.0
            };
            let sub = 7;
            for _ in 0..sub {
                let h = dt / sub as f64;
                let acc = (applied - vs - (t1 + t2) * vs_rate) / (t1 * t2);
                vs_rate += acc * h;
                vs += vs_rate * h;
                let d_sink = (want_sink - idle_sink) * (h / 1.5);
                idle_sink += d_sink;
                vs += d_sink;
                radio += vs / 60.0 * h;
                along_nm += gs_kts * h / 3600.0;
                if radio <= 0.0 {
                    break;
                }
            }
            shallowest = shallowest.max(vs);
            t += dt;
            if radio <= 0.0 {
                return (vs, shallowest, along_nm * M_PER_NM, saw_flare);
            }
            assert!(t < 60.0, "never touched down");
        }
    }

    /// Through the servo as logged, the dial path arrives hard whatever
    /// the lead: the 172's, the 737's own, or a dial pinned at the cap from
    /// the gate. Six seconds of arrest against three of dead time and ten
    /// of lag is arithmetic, not tuning, and it is why the arrest takes the
    /// servos and flies the elevator (`arrest_hands_step`).
    #[test]
    fn the_dial_path_cannot_arrest_the_737_through_its_logged_servo() {
        let jet = crate::aircraft::ArrestLeadProfile {
            gain: 1.5,
            max_climb_fpm: 100.0,
            trend_seconds: 1.5,
        };
        let pinned = crate::aircraft::ArrestLeadProfile {
            gain: 5.0,
            max_climb_fpm: 1000.0,
            trend_seconds: 0.0,
        };
        for (name, lead) in [
            (
                "the 172's lead",
                crate::aircraft::ArrestLeadProfile::default(),
            ),
            ("the 737's lead", jet),
            ("a dial pinned at +1000", pinned),
        ] {
            let (td, _, _, flared) = fly_jet_arrest(lead);
            assert!(flared, "{name}: never reached the flare gate");
            assert!(
                td < -450.0,
                "{name}: the logged servo cannot have arrested it, yet the plant landed at {td:.0} fpm"
            );
        }
    }

    /// King Air 1790787298: the arrest's level-off at 36 ft on approach
    /// thrust. Sinking on the approach, the speed law keeps the lever; the
    /// sink trending through level retards it, under the approach floor; a
    /// climb walks it to idle and no further.
    #[test]
    fn a_floating_roundout_takes_the_thrust_off() {
        assert_eq!(roundout_float_lever(0.612, -557.0, 40.0, 0.33, 0.1), None);
        let lead = roundout_float_lever(0.612, -182.0, 140.0, 0.33, 0.1).expect("level on the trend");
        assert!((lead - 0.579).abs() < 1e-9, "{lead}");
        assert!(roundout_float_lever(0.612, 91.0, 130.0, 0.33, 0.1).is_some());
        assert_eq!(roundout_float_lever(0.11, 300.0, 0.0, 0.5, 0.1), Some(IDLE_THROTTLE));
    }

    /// The decision itself: taken at the roundout gate, kept through the
    /// flare, returned the tick the arrest ends without wheels on.
    #[test]
    fn the_arrest_takes_the_servos_once_and_returns_them_on_a_balk() {
        assert_eq!(arrest_hands_step(false, false), ArrestHands::Off);
        assert_eq!(arrest_hands_step(false, true), ArrestHands::Take);
        assert_eq!(arrest_hands_step(true, true), ArrestHands::Keep);
        assert_eq!(arrest_hands_step(true, false), ArrestHands::Return);
    }

    /// The servos-out arrest over the runway, flown from the gate of the LEMG
    /// 13 landing at 17:43Z (2026-09-26, a 737): 79 ft radio, 3.9 m right of
    /// the centreline and crossing left at 2.3 m/s, 1.4 degrees of left bank,
    /// 144 kt, 8.5 s to the wheels. It touched down 15.3 m left and the
    /// rollout scored 16.8 m.
    ///
    /// The plant is second order in roll (aileron to roll rate with a 0.5 s
    /// lag, 20 deg/s per unit of aileron: the recorded 1 deg/s at 0.035 to
    /// 0.08 of aileron), bank to sideways acceleration, and the arrest's own
    /// tick of 0.25 s with one tick of transport delay. The number asserted is
    /// the offset at the wheels, where the scorer's rollout begins, and the
    /// worst overshoot past the centreline on the way down.
    #[test]
    fn the_arrest_closes_the_centreline_by_the_track_over_the_runway() {
        use crate::autotakeoff::{climb_bank_target, roll_target_heading, WingsLevel, XtkRate};
        let fly = |track_law: bool, x0: f64, v0: f64, phi0: f64| -> (f64, f64) {
            let (gs_kts, t_end, dt) = (144.0, 8.5, 0.25);
            let v_air = gs_kts * 0.514_444;
            let (mut x, mut v, mut phi, mut p) = (x0, v0, phi0, 0.0_f64);
            let mut wings = if track_law { arrest_wings() } else { WingsLevel::default() };
            let mut rate = XtkRate::default();
            let mut prev_x: Option<f64> = None;
            let mut queued = 0.0_f64;
            let (mut t, mut worst) = (0.0, 0.0_f64);
            while t < t_end {
                let radio = 79.0 * (1.0 - t / t_end);
                let rate_now = rate.update(prev_x.map_or(0.0, |px| (x - px) / dt), dt);
                prev_x = Some(x);
                // Coordinated: the heading is the track (no wind, no crab).
                let track_deg = (v / v_air).asin().to_degrees();
                let bank_tgt = if track_law {
                    let side = side_velocity_m_s(Some(track_deg), 0.0, gs_kts).unwrap();
                    arrest_bank_target(x, side, phi, radio)
                } else {
                    let hdg = roll_target_heading(0.0, x, rate_now, gs_kts);
                    let err = ((hdg - track_deg + 540.0).rem_euclid(360.0)) - 180.0;
                    climb_bank_target(err)
                };
                let ail = wings.step(phi, bank_tgt, dt);
                let applied = std::mem::replace(&mut queued, ail);
                for _ in 0..10 {
                    let h = dt / 10.0;
                    p += (20.0 * applied - p) / 0.5 * h;
                    phi += p * h;
                    v += 9.81 * phi.to_radians().tan() * h;
                    x += v * h;
                }
                worst = worst.max(-x * x0.signum());
                t += dt;
            }
            (x, worst)
        };
        let (old_wheels, _) = fly(false, 3.9, -2.3, -1.4);
        assert!(old_wheels.abs() > 5.0, "the heading law lands {old_wheels:.1} m off");
        let (wheels, worst) = fly(true, 3.9, -2.3, -1.4);
        assert!(wheels.abs() < 2.0, "the track law lands {wheels:.1} m off");
        assert!(worst < 5.0, "and never overshoots {worst:.1} m past the line");
        // The 17:25Z landing: 5.9 m right at 100 ft, crossing at 2.5 m/s.
        let (old_wheels, _) = fly(false, 5.9, -2.5, 0.0);
        assert!(old_wheels.abs() > 5.0, "{old_wheels:.1}");
        let (wheels, worst) = fly(true, 5.9, -2.5, 0.0);
        assert!(wheels.abs() < 2.0 && worst < 5.0, "{wheels:.1} / {worst:.1}");
    }

    /// LEMG 13, 2026-09-26 (20:17Z, a 737): the arrest gate at 79 ft, 11.89 m
    /// right, drifting 0.13 m/s left, 2.08 degrees of left bank, 10.4 s to
    /// the wheels. Recorded: the drift built to 2.65 m/s, the bank target
    /// reversed at 42 ft but the bank crossed wings level only at 26 ft, it
    /// crossed the centreline at 22 ft still at 2.5 m/s and touched down
    /// 6.5 m left.
    ///
    /// The roll plant is FITTED to those ticks, not the faster one above:
    /// 13 deg/s of roll rate per unit of aileron (0.15 to 0.18 of aileron took
    /// the bank from -3.8 to -0.5 degrees in 1.7 s) through a 0.5 s lag, and
    /// a slow return toward wings level (3.1 to 2.2 degrees in 2 s at zero
    /// aileron), with the real `WingsLevel`, a 0.25 s tick and one tick of
    /// delay. Asserted: the offset at the wheels and the worst overshoot.
    #[test]
    fn the_arrest_stops_its_drift_by_the_centreline() {
        use crate::autotakeoff::WingsLevel;
        fn old_law(x: f64, side: f64, radio: f64) -> f64 {
            let want = (-x / 4.0).clamp(-2.5, 2.5);
            let bank = ((want - side) / (9.81 * 2.0)).atan().to_degrees();
            let cap = (2.0 + radio.max(0.0) / 10.0).min(5.0);
            bank.clamp(-cap, cap)
        }
        let fly_k = |new: bool, x0: f64, v0: f64, phi0: f64, t_end: f64, k: f64| -> (f64, f64) {
            let (gate_ft, dt) = (79.0, 0.25);
            let (mut x, mut v, mut phi, mut p) = (x0, v0, phi0, 0.0_f64);
            let mut wings = if new { arrest_wings() } else { WingsLevel::default() };
            let mut queued = 0.0_f64;
            let (mut t, mut worst) = (0.0, 0.0_f64);
            while t < t_end {
                let radio = gate_ft * (1.0 - t / t_end);
                let tgt = if new {
                    arrest_bank_target(x, v, phi, radio)
                } else {
                    old_law(x, v, radio)
                };
                let ail = wings.step(phi, tgt, dt);
                let applied = std::mem::replace(&mut queued, ail);
                for _ in 0..10 {
                    let h = dt / 10.0;
                    p += (k * applied - phi / 7.5 - p) / 0.5 * h;
                    phi += p * h;
                    v += 9.81 * phi.to_radians().tan() * h;
                    x += v * h;
                }
                worst = worst.max(-x * x0.signum());
                t += dt;
            }
            (x, worst)
        };
        let fly = |new: bool, x0: f64, v0: f64, phi0: f64, t_end: f64| fly_k(new, x0, v0, phi0, t_end, 13.0);
        let (old_wheels, old_worst) = fly(false, 11.89, -0.13, -2.08, 10.4);
        let (wheels, worst) = fly(true, 11.89, -0.13, -2.08, 10.4);
        let (small, small_worst) = fly(true, 3.0, 0.0, 0.0, 10.4);
        assert!(old_wheels < -4.0 && old_worst > 4.0, "the plant reproduces the overshoot: {old_wheels:.1} m");
        assert!(wheels.abs() < 2.0, "lands {wheels:.1} m off");
        assert!(worst < 2.0, "overshoots {worst:.1} m past the line");
        assert!(small.abs() < 1.0 && small_worst < 1.0, "from 3 m: {small:.1} / {small_worst:.1}");
        // Not tuned to one roll rate: the gate lands inside 2 m on a plant
        // that rolls a third slower or half again faster.
        for k in [9.0, 20.0] {
            let (w, o) = fly_k(true, 11.89, -0.13, -2.08, 10.4, k);
            assert!(w.abs() < 2.0 && o < 2.0, "at {k} deg/s per unit: {w:.1} / {o:.1}");
        }
        // Drifting AWAY at the gate still comes in, and a gate already
        // crossing fast (17:43Z's 3.9 m at 2.3 m/s) lands nearer than it did.
        let (w, o) = fly(true, 8.0, 1.0, 0.0, 10.4);
        assert!(w.abs() < 1.0 && o < 1.0, "drifting away: {w:.1} / {o:.1}");
        let (w_old, _) = fly(false, 3.9, -2.3, -1.4, 8.5);
        let (w, _) = fly(true, 3.9, -2.3, -1.4, 8.5);
        assert!(w.abs() < 2.0 && w.abs() < w_old.abs(), "crossing fast: {w:.1} against {w_old:.1}");
    }

    /// LEMG 13, 2026-09-26 (20:17Z, a 737), the last 7.5 s of the final: on
    /// the centreline at 172 ft, 8.9 m right at 88 ft and 9 to 12 m right at
    /// the 79 ft gate, with the bug within half a degree of the runway
    /// (131.68 true) all the way. Replayed from the recorded inputs: the
    /// crab plus track trim the loop added (0.59 falling to 0.27 degrees),
    /// the recorded drift of the track off the heading (the wind), and an
    /// autopilot that holds heading 0.6 degrees right of its bug through a
    /// 3 s lag (the recorded heading ran 132.1 to 132.6 against a bug of
    /// 131.6 to 132.3). Asserted: the offset at the gate.
    #[test]
    fn the_short_final_holds_the_centreline_to_the_gate() {
        let course = 131.68_f64;
        let gs = 144.0_f64;
        let v_air = gs * 0.514_444;
        let crab_trim = |t: f64| if t < 4.7 { 0.59 - (0.59 - 0.27) * t / 4.7 } else { 0.28 };
        // The recorded track minus heading, at the state samples.
        let drift_pts = [(0.0, 0.9), (0.9, 0.3), (2.2, 0.2), (3.6, 0.7), (5.0, 0.2), (6.3, -0.4), (7.6, -0.4)];
        let drift = |t: f64| {
            let mut d = drift_pts[0].1;
            for w in drift_pts.windows(2) {
                if t >= w[0].0 && t <= w[1].0 {
                    d = w[0].1 + (w[1].1 - w[0].1) * (t - w[0].0) / (w[1].0 - w[0].0);
                }
            }
            if t > drift_pts[drift_pts.len() - 1].0 {
                d = drift_pts[drift_pts.len() - 1].1;
            }
            d
        };
        let fly_with = |new: bool, ap_bias: f64, ap_tau: f64, t_end: f64| -> (f64, f64) {
            let dt = 0.33;
            let (mut x, mut hdg, mut t, mut worst) = (0.0_f64, 131.8_f64, 0.0_f64, 0.0_f64);
            let mut queued = 132.27_f64;
            let mut sf = ShortFinal::new(132.27 - course, hdg + drift(0.0) - course);
            while t < t_end {
                let track = hdg + drift(t);
                let bug = if new {
                    course + sf.step(x, track - course, gs, dt, None)
                } else {
                    course + (-x / M_PER_NM * 80.0).clamp(-8.0, 8.0) + crab_trim(t)
                };
                let applied = std::mem::replace(&mut queued, bug);
                for _ in 0..10 {
                    let h = dt / 10.0;
                    hdg += (applied + ap_bias - hdg) / ap_tau * h;
                    x += v_air * (hdg + drift(t) - course).to_radians().sin() * h;
                }
                worst = worst.max(x.abs());
                t += dt;
            }
            (x, worst)
        };
        let fly = |new: bool| fly_with(new, 0.6, 3.0, 7.5);
        let (old_gate, _) = fly(false);
        let (gate, worst) = fly(true);
        assert!(old_gate > 6.0, "the replay reproduces the drift to the gate: {old_gate:.1} m");
        assert!(gate.abs() < 2.0 && worst < 3.0, "the gate at {gate:.1} m, worst {worst:.1}");
        // Not tuned to one autopilot: no bias, a larger one, a slower and a
        // quicker heading mode.
        for (b, tau) in [(0.0, 3.0), (1.2, 3.0), (0.6, 5.0), (0.6, 1.5)] {
            let (g, w) = fly_with(true, b, tau, 7.5);
            assert!(g.abs() < 2.5 && w < 3.5, "bias {b}, lag {tau} s: gate {g:.1}, worst {w:.1}");
            // ...and a slow airframe's 20 s of short final settles, not rings.
            let (g, w) = fly_with(true, b, tau, 20.0);
            assert!(g.abs() < 1.5 && w < 4.0, "20 s, bias {b}, lag {tau} s: {g:.1} / {w:.1}");
        }
        // Small and bounded: a metre with no drift asks a tenth of a degree,
        // and nothing asks more than the cap.
        assert!(short_final_cut_deg(1.0, 0.0, 144.0).abs() < 0.2);
        assert_eq!(short_final_cut_deg(200.0, 3.0, 144.0), -SHORT_FINAL_MAX_CUT_DEG);
        assert_eq!(short_final_cut_deg(f64::NAN, 0.0, 144.0), 0.0);
        assert!(short_final_cut_deg(0.0, 1.3, 144.0) < -0.5, "a drift is answered before it is metres");
    }

    /// The bank is shallow near the wheels and nothing without a reading.
    #[test]
    fn the_arrest_bank_tapers_to_the_wheels() {
        assert_eq!(arrest_bank_target(-30.0, 0.0, 0.0, 80.0), ARREST_BANK_MAX_DEG);
        assert_eq!(arrest_bank_target(-30.0, 0.0, 0.0, 0.0), ARREST_BANK_WHEELS_DEG);
        assert!(arrest_bank_target(30.0, 0.0, 0.0, 80.0) < 0.0, "right of the line banks left");
        assert_eq!(arrest_bank_target(f64::NAN, 0.0, 0.0, 50.0), 0.0);
        assert_eq!(arrest_bank_target(0.0, 0.0, 0.0, 50.0), 0.0);
        // Crossing left at 2.3 m/s over the centreline: bank right now.
        assert!(arrest_bank_target(0.0, -2.3, 0.0, 50.0) > 3.0);
        assert!(side_velocity_m_s(None, 130.0, 140.0).is_none());
        let v = side_velocity_m_s(Some(129.0), 130.0, 140.0).unwrap();
        assert!((v + 1.26).abs() < 0.05, "1 degree left at 140 kt: {v}");
    }

    /// The overflight rule must know a landing float from an overflight. At
    /// EGHI it balked at 26.3 ft radio — ONE FOOT above the flare gate — at
    /// 0.165 nm past the threshold with 1,400 m of runway still ahead, then
    /// the missed approach timed out mid-reposition and the give-up handed
    /// the pilot a dirty aircraft at half throttle (2026-08-02).
    #[test]
    fn a_low_float_over_a_long_runway_is_a_landing_not_an_overflight() {
        // EGHI-sized runway: 1,723 m.
        let mut rwy = rwy_north();
        rwy.length_m = 1723.0;
        // Low in the float: the allowance is runway-aware — 823 m here.
        let low = overflight_allowance_nm(1723.0, 26.0, 50.0, None);
        assert!((low - 823.0 / M_PER_NM).abs() < 0.01, "{low}");
        // High past the threshold keeps the old hair-trigger: from 160 ft a
        // 3 degree descent runs 640 m to the band, more than the float has.
        assert_eq!(overflight_allowance_nm(1723.0, 160.0, 50.0, None), 0.15);
        // A short strip keeps the old rule even in the float, and a long one
        // is capped at half its length.
        assert_eq!(overflight_allowance_nm(700.0, 20.0, 50.0, None), 0.15);
        assert!(
            (overflight_allowance_nm(3900.0, 20.0, 50.0, None) - 1950.0 / M_PER_NM).abs() < 0.01
        );
        // EGCC 23L, 2026-09-19: the 737 at 57 ft radio, 322 m past the
        // numbers and sinking, balked as an overflight with 2,700 m ahead.
        // Its band is 76 ft, so it is IN the float, and the pavement a
        // landing needs is its own 1,634 m: 1,406 m of float, not 278.
        let jet = overflight_allowance_nm(3040.0, 57.0, 76.0, Some(1634.0));
        assert!(jet > 0.174, "{jet}");
        assert!((jet - 1406.0 / M_PER_NM).abs() < 0.01, "{jet}");
        // Above its band the same geometry applies: 100 ft is 24 ft above
        // the band, 140 m of run, still 1,266 m of float.
        let high = overflight_allowance_nm(3040.0, 100.0, 76.0, Some(1634.0));
        assert!((high - 1266.0 / M_PER_NM).abs() < 0.02, "{high}");
        // A recited landing distance never LOOSENS the GA figure.
        assert_eq!(
            overflight_allowance_nm(1723.0, 26.0, 50.0, Some(300.0)),
            overflight_allowance_nm(1723.0, 26.0, 50.0, None)
        );

        // The live balk, replayed: 0.165 nm past the threshold, 26 ft radio.
        // Must be a roundout on Final, not a go-around.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.165);
        let g = guidance(lat, lon, 26.0, 26.0, 94.0, 97.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final, "this was a landing");
        let (roundout, _) = arrest_targets_fpm(-(94.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert_eq!(g.target_vs_fpm, roundout);

        // Deeper down the float, below the flare gate — dist past the old
        // 0.35 nm flare-zone bound — the flare still owns the touchdown.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.40);
        let g = guidance(lat, lon, 18.0, 18.0, 92.0, 92.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Flare);

        // Past the allowance the overflight verdict returns.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.50);
        let g = guidance(lat, lon, 20.0, 20.0, 92.0, 92.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(
            g.phase,
            ApproachPhase::Intercept,
            "not down by 823 m: go around"
        );

        // And HIGH past the threshold goes around at the old margin, float or
        // no float — the 167 ft overflight stays caught.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.2);
        let g = guidance(lat, lon, 160.0, 160.0, 95.0, 95.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
    }

    fn rwy_north() -> Runway {
        // A runway at the equator, landing due north (course 0), threshold at 0,0.
        Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 3000.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 0.0,
        }
    }

    /// EGTE 08, 2026-09-21: 1,880 m off at 3.3 nm and 160 kt, on the path at
    /// 1,000 ft. Position called it interceptable; it arrived 207 m off at
    /// 94 ft. The gate balks it with the height still in hand.
    #[test]
    fn a_jet_final_that_cannot_converge_balks_at_a_thousand_feet() {
        let at = |along_nm: f64, xtk_m: f64, radio: f64, gs: f64| {
            let (lat, lon) = project_latlon(0.0, 0.0, 180.0, along_nm);
            let (lat, lon) = project_latlon(lat, lon, 90.0, xtk_m / M_PER_NM);
            guidance(
                lat,
                lon,
                radio,
                radio,
                gs,
                gs,
                &rwy_north(),
                150.0,
                25.0,
                0.0,
            )
        };
        let g = at(3.0, 1880.0, 990.0, 160.0);
        assert_eq!(g.balk, Some("unstabilised_at_gate"));
        assert_eq!(g.phase, ApproachPhase::Intercept);
        // Above the gate the same offset is still an intercept in progress.
        let g = at(3.3, 1880.0, 1100.0, 160.0);
        assert_eq!(g.balk, None);
        // An ordinary jet final through the gate, and a light single's.
        assert_eq!(at(3.0, 150.0, 990.0, 150.0).balk, None);
        assert_eq!(at(2.0, 250.0, 700.0, 70.0).balk, None);
    }

    /// Flap 40 out at 161 kt under a 162 placard, back to 10 at 162, out again
    /// at 900 ft (EGTE, 2026-09-21).
    #[test]
    fn flaps_do_not_cycle_on_a_placard() {
        let limits = [
            (0.0, None),
            (0.5, Some(210.0)),
            (0.875, Some(175.0)),
            (1.0, Some(162.0)),
        ];
        let held = |w, ias, cur| flap_held_within_limits(w, ias, &limits, cur);
        // Not extended with a knot of room...
        assert_eq!(held(1.0, 161.0, Some(0.875)), 0.875);
        // ...extended with five...
        assert_eq!(held(1.0, 156.0, Some(0.875)), 1.0);
        // ...held through a touch of the limit...
        assert_eq!(held(1.0, 163.0, Some(1.0)), 1.0);
        // ...and brought in when properly over it.
        assert_eq!(held(1.0, 172.0, Some(1.0)), 0.875);
        // The go-around's half is obeyed at once.
        assert_eq!(held(0.5, 150.0, Some(1.0)), 0.5);
        // No flap profile: the schedule stands.
        assert_eq!(flap_held_within_limits(1.0, 300.0, &[], Some(0.0)), 1.0);
    }

    /// LEMG 13, 2026-09-28 (sortie 1790584266-approach, and every LEMG setup
    /// of the run): the sim handed autoland the 737 at flaps 2 (0.25), the
    /// schedule wanted clean at 9.9 nm, and the first tick took the flaps up
    /// at a 58 kt airspeed read, the next at 146, under the 139 kt clean
    /// stall. An approach keeps the flap the wing flies on until the speed
    /// flies the cleaner configuration (the 737's "up" is declared at 210).
    #[test]
    fn the_approach_keeps_the_flap_the_wing_is_flying_on() {
        let p = crate::aircraft::load("boeing_737_800").unwrap();
        let sched = crate::aircraft::flap_schedule(p.flaps.as_ref());
        let retract = |r: f64| {
            sched
                .iter()
                .find(|(x, _)| (x - r).abs() < 1e-3)
                .map(|(_, k)| *k)
        };
        let limits = [
            (0.0, None),
            (0.25, Some(250.0)),
            (0.5, Some(210.0)),
            (0.875, Some(175.0)),
            (1.0, Some(162.0)),
        ];
        let wanted = flap_schedule(9.88, 2379.2 - 3240.9);
        assert_eq!(wanted, 0.0, "the distance schedule wants clean at 9.9 nm");
        // The old rule took them away on the recorded ticks...
        assert_eq!(flap_held_within_limits(wanted, 58.2, &limits, Some(0.25)), 0.0);
        assert_eq!(flap_held_within_limits(wanted, 146.7, &limits, Some(0.25)), 0.0);
        // ...the approach keeps them, every tick of the dive.
        for ias in [58.2, 146.7, 163.6, 166.0, 172.2, 203.5] {
            let held = flap_held_within_limits(wanted, ias, &limits, Some(0.25));
            assert_eq!(
                flap_kept_flying(held, Some(0.25), ias, &limits, false, retract),
                0.25,
                "{ias} kt"
            );
        }
        // At the manoeuvring speed they come in.
        assert_eq!(flap_kept_flying(0.0, Some(0.25), 212.2, &limits, false, retract), 0.0);
        // No declared speed: the configuration's floor stands in.
        assert_eq!(flap_kept_flying(0.0, Some(0.25), 160.0, &limits, false, |_| Some(167.0)), 0.25);
        assert_eq!(flap_kept_flying(0.0, Some(0.25), 170.0, &limits, false, |_| Some(167.0)), 0.0);
        // Nothing known: the schedule stands, as before.
        assert_eq!(flap_kept_flying(0.0, Some(0.25), 150.0, &limits, false, |_| None), 0.0);
        // A placard still wins, and so does the go-around's half.
        assert_eq!(flap_kept_flying(0.875, Some(1.0), 172.0, &limits, false, retract), 0.875);
        assert_eq!(flap_kept_flying(0.5, Some(1.0), 145.0, &limits, true, retract), 0.5);
        // Extending is never held back, and an unread flap changes nothing.
        assert_eq!(flap_kept_flying(0.5, Some(0.25), 180.0, &limits, false, retract), 0.5);
        assert_eq!(flap_kept_flying(0.0, None, 58.2, &limits, false, retract), 0.0);
    }

    /// The A330's placards and per-notch floors, straight from its profile.
    fn a330_flaps() -> (Vec<(f64, Option<f64>)>, impl Fn(f64) -> Option<f64>) {
        let p = crate::aircraft::load("airbus_a330_300").unwrap();
        let f = p.flaps.clone().unwrap();
        let limits: Vec<_> = f.detents.iter().map(|d| (d.ratio, d.vfe_kts)).collect();
        let clean = p.vs_clean_kts;
        let floors: Vec<_> = f
            .detents
            .iter()
            .map(|d| (d.ratio, if d.ratio == 0.0 { clean } else { d.vs_kts }))
            .collect();
        let floor = move |r: f64| {
            floors
                .iter()
                .find(|(x, _)| (x - r).abs() < 1e-3)
                .and_then(|(_, vs)| vs.map(|v| (1.2 * v).round()))
        };
        (limits, floor)
    }

    /// LEMG 13, sortie 1790622093-approach (2026-09-28, #171): an A330 set up
    /// on a 10 nm final at 206 kt with CONF 1 out. At 7.9 nm (211 kt) the
    /// distance schedule's "clean" took CONF 1 away at the clean retract
    /// speed, the clean floor (210) then held the lever up, and after the
    /// pattern the inbound leg chased the clean 223 kt with CONF 2 out: the
    /// placard took both flaps and the final ran at 200 to 241 kt.
    #[test]
    fn the_a330_final_keeps_its_flap_and_flies_under_the_placard() {
        let (limits, floor) = a330_flaps();
        assert_eq!(flap_placard_kts(0.0, &limits), None);
        assert_eq!(flap_placard_kts(0.25, &limits), Some(215.0));
        assert_eq!(flap_placard_kts(0.5, &limits), Some(196.0));
        assert_eq!(floor(0.0), Some(210.0), "the clean floor, 1.2 x 175");
        assert_eq!(floor(0.25), Some(176.0));

        // t 36.9: 7.88 nm, 47 ft under the path, 211.3 kt, CONF 1 out.
        let sched = flap_schedule(7.877, 2557.3 - 2604.4);
        assert_eq!(sched, 0.0, "the distance schedule wants clean");
        // The old ask: CONF 1 came up at the clean retract speed (210).
        let old = flap_held_within_limits(sched, 211.3, &limits, Some(0.25));
        let retract = |_: f64| Some(210.0);
        assert_eq!(flap_kept_flying(old, Some(0.25), 211.3, &limits, false, retract), 0.0);
        // The approach never asks for less than is out.
        let wanted = approach_flap_wanted(sched, Some(0.25));
        assert_eq!(wanted, 0.25);
        let held = flap_held_within_limits(wanted, 211.3, &limits, Some(0.25));
        assert_eq!(flap_kept_flying(held, Some(0.25), 211.3, &limits, false, retract), 0.25);
        // Inside 6 nm it still asks for more, and a placard still takes it.
        assert_eq!(approach_flap_wanted(flap_schedule(5.4, 400.0), Some(0.25)), 0.5);
        assert_eq!(flap_held_within_limits(0.25, 236.4, &limits, Some(0.25)), 0.0);
        // Nothing read is nothing kept.
        assert_eq!(approach_flap_wanted(0.0, None), 0.0);

        // The speed: t 2.1, 9.9 nm, CONF 1. The schedule's clean speed is
        // 1.45 x 154 = 223, over CONF 1's 215 placard.
        let at_arm = scheduled_ias(9.916, 154.0);
        assert!(at_arm > 215.0, "{at_arm}");
        assert_eq!(approach_target_kts(at_arm, 0.25, &limits, floor(0.25)), 205.0);
        // t 553.1, the pattern's inbound leg at 12.6 nm with CONF 2 out:
        // the governor went to full power for 223 and reached 241.
        let inbound = scheduled_ias(12.635, 154.0);
        assert!((inbound - 223.3).abs() < 0.1, "{inbound}");
        assert_eq!(approach_target_kts(inbound, 0.5, &limits, floor(0.5)), 186.0);
        // The schedule under the placard stands, and so does the floor.
        assert_eq!(approach_target_kts(154.0, 0.25, &limits, floor(0.25)), 176.0);
        assert_eq!(approach_target_kts(154.0, 0.0, &limits, floor(0.0)), 210.0);
        assert_eq!(approach_target_kts(180.0, 1.0, &limits, None), 170.0);
    }

    /// Same sortie, t 679.0: Final at 5.28 nm, 239.4 kt over the ground,
    /// 491.7 ft above the path, on a -1,200 dial. The 3 degree path at that
    /// groundspeed is 1,269 fpm: the flat cap could not even hold it, and a
    /// second later the aircraft was 505 ft high and the guidance aimed at a
    /// join point behind it.
    #[test]
    fn the_glidepath_law_can_always_hold_the_path_it_is_on() {
        let nominal = -(239.4 / 60.0) * GLIDEPATH_FT_PER_NM;
        assert!(nominal < -1200.0, "{nominal}");
        let cap = path_descent_cap_fpm(nominal);
        assert!(cap <= nominal - 299.0, "{cap}");
        let law = (nominal - 491.7 * glidepath_gain_fpm_per_ft(239.4)).clamp(cap, 0.0);
        assert!(law < nominal, "it catches up: {law}");
        // A slower final keeps the old limit.
        assert_eq!(path_descent_cap_fpm(-(150.0 / 60.0) * GLIDEPATH_FT_PER_NM), -1200.0);
        assert_eq!(path_descent_cap_fpm(-(70.0 / 60.0) * GLIDEPATH_FT_PER_NM), -1200.0);
    }

    /// Same sortie, t 680.4: the second `reversal` (missed false, final not
    /// called) at 5.2 nm on the centreline, tracking the runway, 505 ft
    /// above the path. Over 500 ft the guidance aims at the join point for
    /// its altitude, 8 nm out, behind it: `reversal::needed` read that as a
    /// pattern. The copilot then said "Overflew the runway" at 5.2 nm.
    #[test]
    fn a_pattern_begun_short_of_the_field_says_why_and_never_overflew() {
        let rwy = Runway {
            icao: "LEMG".into(),
            id: "13".into(),
            thr_lat: 36.6860,
            thr_lon: -4.5100,
            course_deg: 131.5,
            length_m: 2750.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 48.0,
        };
        let (lat, lon) = crate::nav::project_latlon(
            rwy.thr_lat,
            rwy.thr_lon,
            (rwy.course_deg + 180.0) % 360.0,
            5.2,
        );
        let aim = join_point_nm(2250.0, rwy.elev_ft);
        assert!(aim > 5.2, "the join point is behind: {aim}");
        assert!(crate::reversal::needed(&rwy, lat, lon, 132.2, aim, false));
        assert_eq!(crate::reversal::why(false, false, 505.0), "join_point_behind");
        assert_eq!(crate::reversal::why(true, false, -40.0), "missed_approach");
        assert_eq!(crate::reversal::why(false, true, 100.0), "final_pointing_away");
        assert_eq!(crate::reversal::why(false, false, 100.0), "fap_behind");
        // The phase change on that tick: the pattern's own line says it.
        assert_eq!(dropped_final_line(5.07, true, 516.0), None);
        let high = dropped_final_line(5.07, false, 516.0).unwrap();
        assert!(!high.contains("Overflew"), "{high}");
        // Only with the runway behind.
        assert!(dropped_final_line(-0.3, false, 800.0).unwrap().starts_with("Overflew"));
        assert!(dropped_final_line(-0.3, true, 800.0).unwrap().starts_with("Overflew"));
    }

    /// Same sortie: +100 to +2,000 fpm on the dial under the reposition
    /// floor while the speed law held the lever at 0.10 (the dive's rising
    /// airspeed read as "fast"). A climb is flown on thrust.
    #[test]
    fn a_commanded_climb_is_never_flown_at_idle() {
        use ApproachPhase::*;
        assert_eq!(climb_power_floor(Intercept, 100.0, Some(0.3)), 0.3);
        assert_eq!(climb_power_floor(Intercept, 2000.0, Some(0.3)), 0.3);
        assert_eq!(climb_power_floor(Intercept, 2000.0, None), 0.1);
        assert_eq!(climb_power_floor(Intercept, -700.0, Some(0.3)), 0.1);
        assert_eq!(climb_power_floor(Intercept, 0.0, Some(0.3)), 0.1);
        assert_eq!(climb_power_floor(Final, 500.0, Some(0.3)), 0.1);
        assert_eq!(climb_power_floor(Flare, 500.0, Some(0.3)), 0.1);
    }

    /// Sortie 1790584266-approach, replayed: autoland armed on 12 from the
    /// stale clearance, 3.6 km beside its centreline. The guidance never
    /// descended or flared there: every recorded tick is an Intercept, no
    /// arrest, and from 1,470 ft radio down to 16 ft the law asked to CLIMB
    /// (the reposition floor). The dive was the aircraft's: flaps taken away
    /// under the clean stall at idle (see the two tests above). Pinned so a
    /// later gate cannot flare or descend off a centreline this far away.
    #[test]
    fn the_lemg_crash_ticks_were_never_a_descent_or_a_flare() {
        let rwy = Runway {
            icao: "LEMG".into(),
            id: "12".into(),
            // apt.dat's LEMG 12 threshold and its course to 30's.
            thr_lat: 36.6910753,
            thr_lon: -4.5078270,
            course_deg: crate::nav::bearing_deg(36.6910753, -4.5078270, 36.6795648, -4.4805125),
            length_m: 2750.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 48.0,
        };
        // (dist nm, xtk m, alt ft, radio ft, ias kt), as recorded.
        let ticks = [
            (9.72, -3547.73, 2302.61, 1469.68, 166.04),
            (9.55, -3491.5, 2198.82, 1079.73, 163.1),
            (9.47, -3481.21, 2152.65, 909.03, 163.04),
            (9.37, -3482.11, 2078.31, 864.14, 170.6),
            (9.27, -3495.28, 1944.93, 728.23, 173.39),
            (9.16, -3519.9, 1821.9, 467.69, 175.85),
            (9.06, -3546.88, 1697.67, 274.78, 185.54),
            (8.96, -3575.45, 1560.96, 131.37, 192.15),
            (8.84, -3607.9, 1436.3, 72.21, 203.51),
            (8.81, -3619.45, 1401.01, 46.95, 207.8),
            (8.77, -3631.42, 1370.29, 16.36, 212.19),
        ];
        for (dist, xtk_m, alt, radio, ias) in ticks {
            let (dist, ias): (f64, f64) = (dist, ias);
            let lateral = xtk_m / M_PER_NM;
            let along = (dist * dist - lateral * lateral).sqrt();
            let (lat, lon) = crate::reversal::pattern_point(&rwy, along, lateral);
            let g = guidance(lat, lon, alt, radio, ias.max(174.0), ias, &rwy, 145.0, 25.0, 0.0);
            assert_eq!(g.phase, ApproachPhase::Intercept, "{dist} nm {radio} ft");
            assert!(!g.arresting, "{dist} nm {radio} ft");
            assert!(g.target_vs_fpm > 0.0, "{dist} nm {radio} ft: {}", g.target_vs_fpm);
        }
    }

    /// #131, sortie 1790583663-approach: re-armed from a 10 nm final for 13
    /// onto 12 (courses 131 and 117), 3.5 km beside 12's centreline, and at
    /// 7.6 nm 1,566 m left, 513 ft over the path, tracking 151 at 204 kt,
    /// a reversal began ("Repositioning for runway one two") that was still
    /// flying ten minutes later. From both recorded positions the final is
    /// re-intercepted: the guidance steers in toward the FAP, never out to a
    /// join point behind, and no pattern is needed. Too high to lose on the
    /// way in, or too far off to intercept, it is still a join point.
    #[test]
    fn a_retarget_to_the_near_parallel_reintercepts_its_final() {
        let rwy = Runway {
            icao: "LEMG".into(),
            id: "12".into(),
            thr_lat: 36.6910753,
            thr_lon: -4.5078270,
            course_deg: crate::nav::bearing_deg(36.6910753, -4.5078270, 36.6795648, -4.4805125),
            length_m: 2750.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 48.0,
        };
        let at = |dist: f64, xtk_m: f64| {
            let lateral = xtk_m / M_PER_NM;
            let along = (dist * dist - lateral * lateral).sqrt();
            let (lat, lon) = crate::reversal::pattern_point(&rwy, along, lateral);
            (lat, lon, along, lateral)
        };
        // (dist nm, xtk m, alt ft, radio ft, ias, gs, track): the retarget's
        // first tick on 12, and the tick the reversal began.
        for (dist, xtk_m, alt, radio, ias, gs, track) in [
            (9.67, -3525.0, 2274.0, 1373.0, 168.0, 177.0, 132.0),
            (7.60, -1566.0, 3030.0, 2124.0, 194.0, 204.0, 151.0),
        ] {
            let (lat, lon, along, lateral) = at(dist, xtk_m);
            let g = guidance(lat, lon, alt, radio, gs, ias, &rwy, 145.0, 25.0, 0.0);
            let above = alt - g.glidepath_alt_ft;
            assert!(!aims_at_join_point(above, along, lateral, gs), "{dist} nm {above:.0} ft over");
            let toward = super::heading_diff(g.target_heading_deg, rwy.course_deg);
            assert!(toward.abs() < 45.0, "{dist} nm: steering {toward:.0} off the course");
            let aim = FAP_NM;
            assert!(!crate::reversal::needed(&rwy, lat, lon, track, aim, false), "{dist} nm");
        }
        // The rule as it was: the join point 10.8 nm out, behind at 7.6 nm.
        let (lat, lon, _, _) = at(7.60, -1566.0);
        let behind = join_point_nm(3030.0, rwy.elev_ft);
        assert!(behind > 10.0 && crate::reversal::needed(&rwy, lat, lon, 151.0, behind, false));
        // Past what the way in can lose, or beside a final no cut reaches.
        assert!(aims_at_join_point(1_200.0, 7.5, -0.85, 204.0));
        assert!(aims_at_join_point(600.0, 3.5, 0.0, 204.0), "inside the gate: no room");
        assert!(aims_at_join_point(600.0, 7.5, -5.0, 204.0), "not interceptable");
        assert!(!aims_at_join_point(450.0, 3.5, 0.0, 204.0), "within 500 ft: as before");
    }

    /// EGTE 2026-09-21: handed back at 152 kt with half flap and a flat 0.75
    /// lever; 346 kt followed.
    /// LEMG 13 (2026-09-26): armed at 2.6 nm, 800 ft high, lined up: a
    /// go-around, not a turn onto the downwind. Far out or pointing the
    /// wrong way, the pattern stands.
    #[test]
    fn a_late_high_approach_goes_around_before_it_repositions() {
        assert!(super::late_go_around(2.6, 7.0));
        assert!(!super::late_go_around(2.6, 120.0), "pointing away: the pattern");
        assert!(!super::late_go_around(super::FAP_NM + 2.0, 5.0), "outside the FAP: the pattern");
        assert!(!super::late_go_around(-0.5, 5.0), "past the threshold");
    }

    #[test]
    fn a_give_up_hands_back_a_speed_the_configuration_allows() {
        let limits = [
            (0.0, None),
            (0.5, Some(210.0)),
            (0.875, Some(175.0)),
            (1.0, Some(162.0)),
        ];
        let half = hand_back_speed_kts(148.0, 0.5, &limits);
        assert!((160.0..=195.0).contains(&half), "{half}");
        // Full flap still out: inside ITS placard, never under the approach speed.
        let full = hand_back_speed_kts(148.0, 1.0, &limits);
        assert!((148.0..=150.0).contains(&full), "{full}");
        // Clean: the clean approach speed.
        assert_eq!(
            hand_back_speed_kts(148.0, 0.0, &limits),
            clean_approach_ias(148.0)
        );
    }

    #[test]
    fn the_downwind_is_flown_faster_than_the_final_and_inside_half_flap() {
        let limits = [(0.0, None), (0.5, Some(210.0)), (1.0, Some(162.0))];
        let v = pattern_ias(148.0, &limits);
        assert!(v > 160.0 && v <= 195.0, "{v}");
        // A low half-flap placard caps it; never under the approach speed.
        let v = pattern_ias(65.0, &[(0.0, None), (0.5, Some(85.0)), (1.0, Some(85.0))]);
        assert!((65.0..=70.0).contains(&v), "{v}");
    }

    /// The pattern's downwind passes abeam the join point outbound: the old
    /// release test would let go there on a narrow pattern.
    #[test]
    fn a_held_reposition_ends_when_the_pattern_does() {
        let mut latch = GoAroundLatch::new();
        assert!(latch.order(false));
        let stage = latch.update(None, ApproachPhase::Intercept, 900.0, 1.0, -1.0, 5.75, 0.0);
        assert_eq!(stage, Some(MissedStage::Reposition));
        latch.hold_reposition();
        let stage = latch.update(None, ApproachPhase::Intercept, 1600.0, 5.8, 5.7, 5.75, 0.4);
        assert_eq!(
            stage,
            Some(MissedStage::Reposition),
            "held abeam the join point"
        );
        latch.finish_reposition();
        assert!(!latch.active());
        // Unheld, the old rule still releases there.
        let mut latch = GoAroundLatch::new();
        latch.order(false);
        latch.update(None, ApproachPhase::Intercept, 900.0, 1.0, -1.0, 5.75, 0.0);
        let stage = latch.update(None, ApproachPhase::Intercept, 1600.0, 5.8, 5.7, 5.75, 0.4);
        assert_eq!(stage, None);
    }

    #[test]
    fn true_to_mag_adds_xplane_variation() {
        // The live KPDK final (2026-07-14): variation dataref +5.65 (positive-
        // west), psi 191.3 true read back as magpsi 196.8. The bug must be dialed
        // ABOVE the true command here; the old subtraction flew 11 degrees left
        // of every commanded heading and crossed the threshold 0.4 nm off.
        assert!((true_to_mag(191.3, 5.65) - 196.95).abs() < 0.01);
        // Runway course 20.4 true at KPDK -> 26.05 mag, never 14.75.
        assert!((true_to_mag(20.4, 5.65) - 26.05).abs() < 0.01);
        // Wraps: east-variation field (negative dataref) just below north.
        assert!((true_to_mag(2.0, -5.0) - 357.0).abs() < 0.01);
    }

    #[test]
    fn intercept_steers_to_the_fap_not_away_from_the_field() {
        // The live flee (XLE000N): outside the FAP the old law tracked the bare
        // runway course, which flies AWAY when the aircraft is past the threshold.
        // Place it 8 nm NORTH of a north-landing runway (past the field, departure
        // side): the command must steer SOUTH toward the FAP, not north on course.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 8.0); // 8 nm north, past the threshold
        let g = guidance(
            lat,
            lon,
            2000.0,
            2000.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert_eq!(g.phase, ApproachPhase::Intercept);
        // FAP is 5 nm SOUTH of the threshold; from 8 nm north that's ~due south.
        assert!(
            heading_diff(g.target_heading_deg, 180.0) < 20.0,
            "must steer toward the FAP/field, got hdg {}",
            g.target_heading_deg
        );
    }

    #[test]
    fn speed_protection_lowers_the_nose_before_a_stall() {
        // The live failure (XLE000N): stuck below the glidepath 13 nm out, holding
        // level, IAS decaying to 53 kt until the AP stalled off. Below the path AND
        // slow, the guidance must now command a DESCENT to regain speed, not a level
        // hold. 5 nm south, well below the path, 55 kt against a 90 kt approach.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 5.0);
        let g = guidance(
            lat,
            lon,
            200.0,
            200.0,
            55.0,
            55.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert!(
            g.target_vs_fpm < -100.0,
            "slow + below path must descend, got {}",
            g.target_vs_fpm
        );
        // On speed at the same spot, it holds level to wait for the path (no dive).
        let g_ok = guidance(
            lat,
            lon,
            200.0,
            200.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert_eq!(g_ok.target_vs_fpm, 0.0, "on speed below path → level hold");
        // ...and protection must never be SHALLOWER than the path the aircraft
        // is already flying. On a fast airframe the fixed 40 fpm/kt rate is
        // less than the path itself, so it used to REDUCE the descent — which
        // both loses the path and loses speed, the opposite of the intent. The
        // Luton MD-82 (2026-07-28) was 9 kt slow at 1.5 nm on a 3° path: the
        // path wanted ~740 fpm down, protection commanded 360, and it arrived
        // over the threshold 51 ft high and 12 kt fast for a go-around.
        let (jlat, jlon) = project_latlon(0.0, 0.0, 180.0, 1.5);
        let jet = guidance(
            jlat,
            jlon,
            520.0 + 1.5 * GLIDEPATH_FT_PER_NM,
            1.5 * GLIDEPATH_FT_PER_NM,
            140.0,
            131.0,
            &rwy_north(),
            140.0,
            25.0,
            0.0,
        );
        let path_vs = -(140.0 / 60.0) * GLIDEPATH_FT_PER_NM;
        assert!(
            jet.target_vs_fpm <= path_vs,
            "protection must not be shallower than the {path_vs:.0} fpm path, got {}",
            jet.target_vs_fpm
        );
        // Speed protection never fights the flare (we WANT to slow down landing).
        let g_flare = guidance(
            0.001,
            0.0,
            30.0,
            15.0,
            40.0,
            40.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert_eq!(g_flare.phase, ApproachPhase::Flare);
        assert!(
            g_flare.target_vs_fpm > -200.0,
            "flare not overridden by speed floor"
        );
    }

    #[test]
    fn overflight_past_the_threshold_goes_around_to_the_fap() {
        // The live failure (2026-07-06): overflew the numbers, then tracked the
        // runway course straight out the departure end to 5.4 nm, never turning
        // back. Place the aircraft 0.5 nm PAST a north-landing threshold (0.5 nm
        // north, departure side) at 200 ft, on speed — inside the old "Final"
        // distance so the bug applied. It must now go around: phase Intercept,
        // steering SOUTH back toward the FAP, not north on the runway course.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.5); // 0.5 nm north, past the threshold
        let g = guidance(
            lat,
            lon,
            200.0,
            200.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(
            g.phase,
            ApproachPhase::Intercept,
            "past the threshold → go around"
        );
        assert!(
            heading_diff(g.target_heading_deg, 180.0) < 25.0,
            "must turn back toward the FAP, got hdg {}",
            g.target_heading_deg
        );
        // Sanity: a mirror point 0.5 nm SHORT of the threshold (approach side) is a
        // normal final, not a go-around.
        let (slat, slon) = project_latlon(0.0, 0.0, 180.0, 0.5);
        let g_final = guidance(
            slat,
            slon,
            200.0,
            200.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_ne!(
            g_final.phase,
            ApproachPhase::Intercept,
            "short of the threshold is not a go-around"
        );
    }

    #[test]
    fn climbs_off_the_terrain_floor_only_with_energy() {
        // The live failure (2026-07-06): level at ~50 ft AGL 5 nm out over rising
        // terrain, no climb anywhere in the loop. 3 nm out, 200 ft AGL (below the
        // 500 ft floor), on the approach side.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 3.0);
        // FAST (approach + 10): surplus energy → bounded climb away from terrain.
        let g_fast = guidance(
            lat,
            lon,
            200.0,
            200.0,
            100.0,
            100.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert!(
            g_fast.target_vs_fpm > 0.0,
            "below the floor + fast → climb, got {}",
            g_fast.target_vs_fpm
        );
        assert!(
            g_fast.target_vs_fpm <= MAX_CLIMB_FPM,
            "climb is bounded, got {}",
            g_fast.target_vs_fpm
        );
        // And it asks the throttle for extra speed to sustain the climb.
        assert!(
            g_fast.target_ias_kts > 90.0,
            "climb raises the speed target, got {}",
            g_fast.target_ias_kts
        );
        // ON SPEED (no surplus): must NOT climb — hold level and let the governor
        // rebuild speed first (climbing slow would stall).
        let g_slow = guidance(
            lat,
            lon,
            200.0,
            200.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(
            g_slow.target_vs_fpm, 0.0,
            "below the floor but no surplus → level, got {}",
            g_slow.target_vs_fpm
        );
    }

    #[test]
    fn terrain_floor_is_not_enforced_on_short_final() {
        // Low AGL close in is the approach, not terrain — don't climb away from it.
        // 0.5 nm short of the threshold, 200 ft AGL, fast: no climb (inside the
        // floor-enforcement range).
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.5);
        let g = guidance(
            lat,
            lon,
            200.0,
            200.0,
            100.0,
            100.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert!(
            g.target_vs_fpm <= 0.0,
            "no climb on short final, got {}",
            g.target_vs_fpm
        );
        assert_eq!(
            g.target_ias_kts, 90.0,
            "no climb-energy bonus on short final"
        );
    }

    #[test]
    fn floor_never_captures_an_on_path_final_at_a_flat_field() {
        // The LXGB 09 overflight (2026-07-22): a 3° path crosses 500 AGL at
        // ~1.6 nm, and the floor held a perfectly on-path approach level (and
        // hot, via the climb-energy bonus) from there to 1.0 nm — 216 ft high
        // at the numbers, guaranteed go-around at every flat field. On path
        // with low radio IS the approach: keep descending, no speed bonus.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 1.3);
        // On the TDZ-aimed path at 1.3 nm (~465 ft) — still below the 500 floor.
        let on_path = (1.3 + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
        let g = guidance(
            lat,
            lon,
            on_path,
            on_path,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(g.phase, ApproachPhase::Final);
        assert!(
            g.target_vs_fpm < -300.0,
            "on path must keep descending, got {}",
            g.target_vs_fpm
        );
        assert_eq!(
            g.target_ias_kts, 90.0,
            "no climb-energy bonus on an on-path final"
        );
        // Deeply below the path at the same distance: the floor still stands.
        let g_low = guidance(
            lat,
            lon,
            200.0,
            200.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(
            g_low.target_vs_fpm, 0.0,
            "dragged-in-low keeps the floor, got {}",
            g_low.target_vs_fpm
        );
    }

    #[test]
    fn flare_balks_off_the_centreline_and_keeps_steering_on_it() {
        // The LXGB 09 crash (2026-07-22): the flare froze the bare runway
        // course with 0.10 nm of standing cross-track — touchdown 185 m right
        // of the centreline, wrecked in the grass. Off the pavement at flare
        // height must be a balked landing (Intercept → the latch climbs
        // straight ahead), never a touchdown.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.15);
        let off = lon + 0.10 / 60.0; // 0.10 nm right of the centreline
        let g = guidance(
            lat,
            off,
            20.0,
            20.0,
            80.0,
            80.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(
            g.phase,
            ApproachPhase::Intercept,
            "off the pavement at 20 ft = balk"
        );
        // ...and the balk fires EARLY, from final at 80 ft — the 22 ft balk
        // still skimmed the ground at 91 kt (2026-07-22).
        let (hlat, hlon) = project_latlon(0.0, 0.0, 180.0, 0.4);
        let hoff = hlon + 0.05 / 60.0;
        let g_early = guidance(
            hlat,
            hoff,
            80.0,
            80.0,
            85.0,
            85.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(
            g_early.phase,
            ApproachPhase::Intercept,
            "off-centreline at 80 ft = balk"
        );
        // On the centreline the flare still fires, and a small offset keeps a
        // gentle correction instead of locking the bare course.
        let near = lon + 0.01 / 60.0;
        let g_ok = guidance(
            lat,
            near,
            20.0,
            20.0,
            80.0,
            80.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(g_ok.phase, ApproachPhase::Flare);
        let cut = ((rwy_north().course_deg - g_ok.target_heading_deg + 540.0) % 360.0) - 180.0;
        assert!(
            cut > 0.1 && cut <= 8.0,
            "flare keeps a bounded correction, got {cut}"
        );
    }

    #[test]
    fn intercept_holds_the_floor_even_close_to_the_threshold() {
        // The KPDK 16 rejoin (2026-07-14): armed 0.75 nm out on the WRONG side
        // of the threshold at ~300 AGL. Distance is symmetric, so the glidepath
        // read "on path at 0.7 nm" and the vertical law descended a repositioning
        // aircraft into the ground 0.35 nm short. In Intercept the floor must
        // hold at ANY distance: climb with surplus energy, level without — never
        // a glidepath descent.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.7); // 0.7 nm PAST a north-landing threshold
        let g_fast = guidance(
            lat,
            lon,
            300.0,
            300.0,
            100.0,
            100.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(g_fast.phase, ApproachPhase::Intercept);
        assert!(
            g_fast.target_vs_fpm > 0.0,
            "repositioning low + fast → climb, got {}",
            g_fast.target_vs_fpm
        );
        let g_slow = guidance(
            lat,
            lon,
            300.0,
            300.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(g_slow.phase, ApproachPhase::Intercept);
        assert_eq!(
            g_slow.target_vs_fpm, 0.0,
            "repositioning low + no surplus → level, got {}",
            g_slow.target_vs_fpm
        );
    }

    #[test]
    fn low_go_around_latches_straight_ahead_until_the_floor() {
        // The LXGB 09 crash (2026-07-14): overflew the numbers at ~57 ft, the
        // intercept guidance flipped the heading 178° toward the FAP, the flare
        // re-fired past the threshold and pulled the power, and the aircraft
        // hit 200 m south of the strip. Low, the latch must own the go-around.
        use ApproachPhase::*;
        use MissedStage::*;
        let mut latch = GoAroundLatch::new();
        // Just past the threshold, so along-track is negative throughout.
        let up = |l: &mut GoAroundLatch, p, n, r| l.update(Some(p), n, r, 0.2, -0.2, FAP_NM, 0.0);
        // Final ticking along at 60 ft: not a go-around.
        assert_eq!(up(&mut latch, Final, Final, 60.0), None);
        // Overflight: Final degrades to Intercept at 57 ft → latch, climbing.
        assert_eq!(up(&mut latch, Final, Intercept, 57.0), Some(Climb));
        // The raw guidance flickering (Intercept → Flare past the threshold →
        // Intercept) must not shake it loose while still low.
        assert_eq!(up(&mut latch, Intercept, Flare, 22.0), Some(Climb));
        assert_eq!(up(&mut latch, Flare, Intercept, 35.0), Some(Climb));
        // Above the old 500 ft floor but below the turn height: still climbing
        // straight ahead — turning back at 520 ft is how the Rock CFIT
        // happened (2026-07-22).
        assert_eq!(up(&mut latch, Intercept, Intercept, 520.0), Some(Climb));
        // At the turn height: start repositioning, but do NOT hand back yet —
        // 800 ft over the far end with the field behind us is not an approach.
        assert_eq!(
            up(&mut latch, Intercept, Intercept, 850.0),
            Some(Reposition)
        );
        // A HIGH overflight never latches: the immediate turn back is fine.
        let mut high = GoAroundLatch::new();
        assert_eq!(up(&mut high, Final, Intercept, 800.0), None);
    }

    /// The missed approach is a procedure, not a moment: the reposition stage
    /// runs until the aircraft is back where an approach starts. Anything less
    /// and the stateless guidance re-decides every tick — the Luton orbit,
    /// 1.1 to 4.1 nm for three and a half minutes (2026-07-28).
    #[test]
    fn the_reposition_runs_until_established_outside_the_fap() {
        use ApproachPhase::*;
        use MissedStage::*;
        let mut l = GoAroundLatch::new();
        // Platform height puts the join point ~5.75 nm out.
        let join = join_point_nm(1600.0, 0.0);
        assert_eq!(
            l.update(Some(Final), Intercept, 50.0, 0.2, -0.2, join, 0.0),
            Some(Climb)
        );
        assert_eq!(
            l.update(Some(Intercept), Intercept, 900.0, 1.0, -1.0, join, 0.0),
            Some(Reposition)
        );
        // Departure side, miles out: still repositioning, however far.
        assert_eq!(
            l.update(Some(Intercept), Intercept, 1600.0, 6.0, -6.0, join, 0.0),
            Some(Reposition)
        );
        // Approach side but short of the join point: an approach can't start
        // here, and handing back mid-outbound-leg means a 180 on final.
        assert_eq!(
            l.update(Some(Intercept), Intercept, 1600.0, 5.2, 5.2, join, 0.0),
            Some(Reposition)
        );
        // Abeam the join point but 2.7 nm beside the centreline (the 737's
        // turn-back, LFLL 2026-09-17): not an approach yet, keep homing.
        assert_eq!(
            l.update(
                Some(Intercept),
                Intercept,
                1600.0,
                join + 1.3,
                join + 1.0,
                join,
                -2.7
            ),
            Some(Reposition)
        );
        // Out at the join point on the approach side: hand back.
        assert_eq!(
            l.update(Some(Intercept), Intercept, 1600.0, join, join, join, 0.3),
            None
        );
    }

    /// A controller's order starts a missed approach from wherever we are,
    /// and it counts like any other.
    ///
    /// Malaga 2026-08-05: "cleared to land" and, one second later, "GO
    /// AROUND, traffic on the runway". Nothing consumed the order — the
    /// approach flew on for another seventy seconds through a 0.5 nm
    /// same-altitude conflict and only balked because it happened to be
    /// high.
    #[test]
    fn an_ordered_go_around_is_flown_from_wherever_the_approach_is() {
        use ApproachPhase::*;
        let mut l = GoAroundLatch::new();
        // Established on final, stable, nothing wrong with the approach: the
        // aircraft would never have called this one itself.
        assert_eq!(
            l.update(Some(Final), Final, 600.0, 2.0, 2.0, FAP_NM, 0.0),
            None
        );
        assert!(l.order(false), "the order starts one");
        assert_eq!(l.stage, Some(MissedStage::Climb));
        assert_eq!(l.attempts(), 1, "an ordered go-around is still a go-around");
        // Idempotent while one is already being flown — a controller
        // repeating the order must not restart the climb or burn an attempt.
        assert!(!l.order(false));
        assert_eq!(l.attempts(), 1);
        // And it flies on as usual from there: climb clear of the ground,
        // then reposition. (The raw guidance under a climb-away reads
        // Intercept — it is already turning back toward the FAP, which is
        // exactly what the latch exists to override until we are clear.)
        assert_eq!(
            l.update(Some(Final), Intercept, 900.0, 0.5, -0.5, 6.0, 0.0),
            Some(MissedStage::Reposition),
            "clear of the ground, so reposition"
        );

        // On the ground it is too late: a landing that has already happened
        // is not one to abandon, and the rollout is the safer place to be.
        let mut rolled = GoAroundLatch::new();
        assert!(!rolled.order(true));
        assert_eq!(rolled.attempts(), 0);
    }

    #[test]
    fn go_around_latch_counts_attempts_for_the_runaway_backstop() {
        use ApproachPhase::*;
        let mut l = GoAroundLatch::new();
        assert_eq!(l.attempts(), 0);
        // Each Final->Intercept low is one go-around; the latch releases once
        // the aircraft is repositioned outside the FAP, ready to try again.
        for n in 1..=3 {
            assert!(
                l.update(Some(Final), Intercept, 50.0, 0.2, -0.2, FAP_NM, 0.0)
                    .is_some(),
                "latched low"
            );
            assert_eq!(l.attempts(), n, "counted");
            assert!(
                l.update(Some(Intercept), Intercept, 50.0, 0.3, -0.3, FAP_NM, 0.0)
                    .is_some(),
                "stays latched climbing"
            );
            assert!(
                l.update(Some(Intercept), Intercept, 900.0, 2.0, -2.0, FAP_NM, 0.0)
                    .is_some(),
                "and stays latched repositioning"
            );
            assert!(
                l.update(Some(Intercept), Intercept, 1600.0, 5.5, 5.5, FAP_NM, 0.0)
                    .is_none(),
                "released once established at the join point"
            );
        }
        // The loop gives up once attempts EXCEED the cap.
        assert!(l.attempts() > MAX_GO_AROUNDS);
    }

    #[test]
    fn go_around_guidance_climbs_straight_ahead_on_speed() {
        let rwy = rwy_north();
        let climb = |ias| {
            missed_approach_guidance(
                &rwy,
                MissedStage::Climb,
                -0.005,
                0.0,
                200.0,
                150.0,
                0.3,
                100.0,
                ias,
                90.0,
                0.0,
                Some(70.0),
            )
        };
        // On speed: full climb, straight down the runway heading.
        let g = climb(92.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
        assert!(
            heading_diff(g.target_heading_deg, rwy.course_deg) < 0.1,
            "straight ahead"
        );
        assert!(g.target_vs_fpm > 0.0 && g.target_vs_fpm <= MAX_CLIMB_FPM);
        // Slow (deficit beyond the floor margin): level acceleration first —
        // climbing slow at full flap is how a go-around stalls.
        let g_slow = climb(75.0);
        assert_eq!(g_slow.target_vs_fpm, 0.0, "build speed before climbing");
        assert!(heading_diff(g_slow.target_heading_deg, rwy.course_deg) < 0.1);
    }

    /// The tower is owed the call, so the words have to be words it knows: the
    /// FSM sat in ClearedToLand through a four-minute missed approach because
    /// nothing was ever transmitted (Luton, 2026-07-28). This pins the phrase
    /// the loop sends against the parser that has to understand it.
    #[test]
    fn the_going_around_call_reads_as_a_go_around_to_the_tower() {
        let call = crate::circuit::radio_call(
            &Some("November five five two Alpha Alpha".to_string()),
            "going around",
        );
        assert_eq!(
            crate::tower::parse_intent(&call.to_ascii_lowercase()),
            crate::tower::Intent::GoAround,
            "{call}"
        );
        // ...and without a readable tail number it still parses.
        let bare = crate::circuit::radio_call(&None, "going around");
        assert_eq!(
            crate::tower::parse_intent(&bare),
            crate::tower::Intent::GoAround
        );
    }

    /// Both calls the vectored arrival makes have to be words the tower knows,
    /// for the same reason the go-around does. Asking for vectors and then
    /// reporting established is the sequence that earns the approach clearance
    /// and, after it, the landing clearance — a phrasing the parser shrugs at
    /// leaves the automation complying with a controller who never engaged.
    #[test]
    fn the_vectored_arrival_calls_read_as_the_tower_expects() {
        let cs = Some("November five five two Alpha Alpha".to_string());

        let ask = crate::circuit::radio_call(&cs, "request vectors");
        assert_eq!(
            crate::tower::parse_intent(&ask.to_ascii_lowercase()),
            crate::tower::Intent::RequestVectors,
            "{ask}"
        );
        // ...and with no readable tail number.
        assert_eq!(
            crate::tower::parse_intent(&crate::circuit::radio_call(&None, "request vectors")),
            crate::tower::Intent::RequestVectors
        );

        // "established" is the report the ILS clearance asks for, and it must
        // reach the tower as the FINAL report that earns the landing
        // clearance — not as an unknown.
        let est = crate::circuit::radio_call(&cs, "established");
        assert_ne!(
            crate::tower::parse_intent(&est.to_ascii_lowercase()),
            crate::tower::Intent::Unknown,
            "the established report must parse: {est}"
        );

        // The guard that makes "request vectors" safe: a ground call carrying
        // the same word is NOT a vectoring request (a real 2026-07-09 call).
        assert_ne!(
            crate::tower::parse_intent("request vectors for taxi to parking"),
            crate::tower::Intent::RequestVectors
        );
    }

    /// The reposition leg flies OUTBOUND to the join point and levels at
    /// platform height — it never dives at the glidepath under it, which is
    /// the path it is flying away from.
    #[test]
    fn the_reposition_leg_flies_outbound_and_holds_platform_height() {
        let rwy = rwy_north(); // course ~000, threshold at 0,0
                               // Half a mile past the far end at 900 ft, on speed: the join point is
                               // SOUTH (approach side), so the aircraft must turn around.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.5);
        let g = missed_approach_guidance(
            &rwy,
            MissedStage::Reposition,
            lat,
            lon,
            900.0,
            900.0,
            0.5,
            200.0,
            95.0,
            90.0,
            0.0,
            Some(70.0),
        );
        assert!(
            heading_diff(g.target_heading_deg, 180.0) < 20.0,
            "outbound to the join point, got {}",
            g.target_heading_deg
        );
        assert!(g.target_vs_fpm > 0.0, "still below platform: climb");
        // At platform height it levels off rather than chasing the path down.
        let level = missed_approach_guidance(
            &rwy,
            MissedStage::Reposition,
            lat,
            lon,
            REPOSITION_MIN_AGL_FT + 100.0,
            REPOSITION_MIN_AGL_FT + 100.0,
            0.5,
            200.0,
            95.0,
            90.0,
            0.0,
            Some(70.0),
        );
        assert_eq!(level.target_vs_fpm, 0.0, "hold platform, never dive at it");
    }

    #[test]
    fn final_cuts_hard_enough_to_capture_the_centreline() {
        // The LXGB final (2026-07-14) entered 0.45 nm off the centreline and
        // the old 8°/nm gain commanded 3.6° of cut — it crossed the threshold
        // still ~120 m off ("coming in at an angle"). A meaningful offset must
        // earn a real intercept angle, decaying near the centreline.
        let rwy = rwy_north();
        // 3 nm south (on final), 0.45 nm EAST (right) of the centreline.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 3.0);
        let east = lon + 0.45 / 60.0;
        let g = guidance(lat, east, 900.0, 900.0, 90.0, 90.0, &rwy, 90.0, 20.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        // Right of course → cut LEFT (west of north): 0.45 × 40 clamps to 30.
        let cut = ((rwy.course_deg - g.target_heading_deg + 540.0) % 360.0) - 180.0;
        assert!(cut > 15.0, "0.45 nm off must earn a real cut, got {cut}");
        // Nearly on the centreline: gentle.
        let near = lon + 0.03 / 60.0;
        let g2 = guidance(lat, near, 900.0, 900.0, 90.0, 90.0, &rwy, 90.0, 20.0, 0.0);
        let cut2 = ((rwy.course_deg - g2.target_heading_deg + 540.0) % 360.0) - 180.0;
        assert!(cut2.abs() < 5.0, "small offset stays gentle, got {cut2}");
    }

    #[test]
    fn glidepath_descends_toward_the_threshold() {
        // 5 nm south of the threshold at 2000 ft, 120 kt groundspeed, tracking north.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 5.0); // 5 nm south
        let g = guidance(
            lat,
            lon,
            2000.0,
            2000.0,
            120.0,
            90.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert!((g.dist_nm - 5.0).abs() < 0.2, "5 nm out, got {}", g.dist_nm);
        // Glidepath alt at 5 nm ≈ 1590 ft; we're above it → commanded to descend.
        assert!(g.glidepath_alt_ft > 1400.0 && g.glidepath_alt_ft < 1700.0);
        assert!(
            g.target_vs_fpm < 0.0,
            "should descend, got {}",
            g.target_vs_fpm
        );
        // Roughly on the centreline → heading close to the runway course (north).
        assert!(
            heading_diff(g.target_heading_deg, 0.0) < 10.0,
            "hdg {}",
            g.target_heading_deg
        );
    }

    #[test]
    fn armed_high_overhead_flies_outbound_to_a_join_point() {
        // The live failure: 7,000 ft only 6 nm out (path altitude ~1,900) — the
        // old guidance aimed at the FAP and could never capture. Now it must fly
        // AWAY from the runway (toward the join point down the extended
        // centreline) and descend.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 6.0); // 6 nm south
        let g = guidance(
            lat,
            lon,
            7000.0,
            7000.0,
            135.0,
            110.0,
            &rwy_north(),
            110.0,
            20.0,
            0.0,
        );
        assert_eq!(g.phase, ApproachPhase::Intercept);
        // Runway is north (bearing ~0); the join point is further SOUTH — so the
        // commanded heading must be southbound, not at the runway.
        assert!(
            heading_diff(g.target_heading_deg, 180.0) < 30.0,
            "should fly outbound, got hdg {}",
            g.target_heading_deg
        );
        assert!(
            g.target_vs_fpm <= -1000.0,
            "and descend hard, got {}",
            g.target_vs_fpm
        );
    }

    #[test]
    fn flare_near_the_ground_arrests_descent_and_tracks_runway() {
        let g = guidance(
            0.001,
            0.0,
            30.0,
            15.0,
            100.0,
            90.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert_eq!(g.phase, ApproachPhase::Flare);
        assert!(
            g.target_vs_fpm > -200.0,
            "flare eases the descent, got {}",
            g.target_vs_fpm
        );
        assert!(
            heading_diff(g.target_heading_deg, 0.0) < 1.0,
            "flare tracks the runway"
        );
    }

    #[test]
    fn low_radio_far_from_the_runway_is_terrain_not_a_flare() {
        // The LEMI failure: 7 ft AGL over rising terrain 4.1 nm out — the old
        // radio-only trigger "flared" (power to idle, vs -100) into open ground.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 4.1); // 4.1 nm south
        let g = guidance(
            lat,
            lon,
            900.0,
            7.0,
            130.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_ne!(
            g.phase,
            ApproachPhase::Flare,
            "no flare 4 nm from the threshold"
        );
        // And the same low radio just short of the threshold IS the flare.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.1);
        let g = guidance(
            lat,
            lon,
            25.0,
            15.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(g.phase, ApproachPhase::Flare);
    }

    /// The user-visible property the whole final law exists for: from a real
    /// offset at the top of final, the aircraft must be ESTABLISHED — cut
    /// under 3° — well before short final, not rolling out in the flare.
    /// March the kinematics through the guidance and check where alignment
    /// actually arrives.
    #[test]
    fn the_final_is_established_miles_before_the_threshold() {
        let rwy = rwy_north();
        let gs_kts = 120.0;
        // Top of final: 5 nm out, 0.4 nm right of the centreline.
        let mut d_nm = 5.0f64;
        let mut off_nm = 0.4f64;
        let mut established_at = None;
        for _ in 0..600 {
            if d_nm < 0.4 {
                break;
            }
            let (lat, lon) = project_latlon(0.0, 0.0, 180.0, d_nm);
            let lon_off = lon + off_nm / 60.0;
            let on_path = (d_nm + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
            let g = guidance(
                lat, lon_off, on_path, on_path, gs_kts, 100.0, &rwy, 90.0, 25.0, 0.0,
            );
            let cut = ((g.target_heading_deg + 540.0).rem_euclid(360.0)) - 180.0;
            if established_at.is_none() && cut.abs() < 3.0 && off_nm.abs() < 0.05 {
                established_at = Some(d_nm);
            }
            // Fly one second along the commanded heading (no wind, perfect AP).
            let dt_h = 1.0 / 3600.0;
            let trk = g.target_heading_deg.to_radians();
            off_nm += gs_kts * dt_h * trk.sin();
            d_nm -= gs_kts * dt_h * trk.cos().abs();
        }
        let est = established_at.expect("the approach must become established");
        assert!(
            est > 2.0,
            "established by {est:.2} nm — short final is too late (the EGSS \
             approach aligned at 0.13 nm and it felt exactly as unsafe as it was)"
        );
    }

    #[test]
    fn final_pursuit_cut_scales_with_distance_remaining() {
        // 0.2 nm RIGHT of the centreline. The cut toward the line must be
        // DISTANCE-aware: modest 3 nm out, aggressive at 0.8 nm — the fixed
        // per-nm gain commanded 7° at 150 ft with 145 m to close and the
        // aircraft crossed short final diagonal (2026-07-24).
        let cut = |g: &Guidance| ((g.target_heading_deg - 0.0 + 540.0).rem_euclid(360.0)) - 180.0;
        let at = |d_nm: f64| {
            let (lat, lon) = project_latlon(0.0, 0.0, 180.0, d_nm);
            let east = (lat, lon + 0.2 / 60.0);
            let on_path = (d_nm + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
            guidance(
                east.0,
                east.1,
                on_path,
                on_path,
                90.0,
                90.0,
                &rwy_north(),
                90.0,
                25.0,
                0.0,
            )
        };
        let far = at(3.0);
        let close = at(0.8);
        // Offset RIGHT of a north course → cut LEFT (negative), and a REAL
        // intercept at BOTH distances. The fixed-lead pursuit replaced the
        // half-remaining aim precisely because the far-out cut was too shy:
        // with the aim at d/2 the offset decayed as d² and the aircraft rode
        // the whole final at an angle, aligning at 0.13 nm — over the numbers
        // (EGSS 22, 2026-07-28). Same offset now earns roughly the same cut
        // at any distance, which is what makes the convergence exponential
        // in time and the last miles wings-level.
        assert!(
            cut(&far) < -12.0,
            "far cut is a real intercept: {:.1}",
            cut(&far)
        );
        assert!(cut(&close) < -12.0, "close cut too: {:.1}", cut(&close));
        assert!(
            cut(&close) <= cut(&far) + 1.0,
            "never softer close-in: far {:.1} vs close {:.1}",
            cut(&far),
            cut(&close)
        );
        // And clamped — never a base-turn-sized swing on final.
        assert!(cut(&close) >= -35.5);
        // On the centreline: no cut either distance.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 1.5);
        let on_path = (1.5 + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
        let g = guidance(
            lat,
            lon,
            on_path,
            on_path,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert!(cut(&g).abs() < 1.5, "centred: {:.1}", cut(&g));
        // SHORT final (inside the 0.3 nm pursuit floor), small offset (a big
        // one is the balk gate's problem): tiny corrections only — the
        // halfway aim fell BEHIND the aircraft at 0.06 nm and the command
        // thrashed ±35° at 100 ft over the numbers (2026-07-24).
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.06);
        let near = (lat, lon + 0.015 / 60.0); // ~28 m right
        let on_path = (0.06 + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
        let over = guidance(
            near.0,
            near.1,
            on_path,
            on_path,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert!(
            cut(&over).abs() <= 8.5,
            "no thrash over the numbers: {:.1}",
            cut(&over)
        );
        assert!(
            cut(&over) < -0.5,
            "still corrects toward the centreline: {:.1}",
            cut(&over)
        );
    }

    #[test]
    fn glidepath_aims_at_the_touchdown_zone_not_the_threshold_edge() {
        // On the extended centreline 3 nm out, established: the commanded
        // glidepath altitude must be the 3° path aimed ~300 m PAST the threshold,
        // so it crosses the numbers at ~50 ft, not zero.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 3.0); // 3 nm south of thr
        let g = guidance(
            lat,
            lon,
            1000.0,
            1000.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        let expected = (3.0 + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
        assert!(
            (g.glidepath_alt_ft - expected).abs() < 1.0,
            "gp {} vs {expected}",
            g.glidepath_alt_ft
        );
        // Threshold-crossing height ~50 ft (the aim offset × slope), not 0.
        let tch = TOUCHDOWN_AIM_NM * GLIDEPATH_FT_PER_NM;
        assert!((45.0..=55.0).contains(&tch), "TCH ~50 ft, was {tch}");
    }

    #[test]
    fn full_flaps_wait_for_the_glidepath() {
        // Established on the path inside 3 nm: full flaps.
        assert_eq!(flap_schedule(2.0, 0.0), 1.0);
        assert_eq!(flap_schedule(2.0, 100.0), 1.0);
        // Dragging level BELOW the path (the LERL energy trap): stay at half.
        assert_eq!(flap_schedule(2.0, -300.0), 0.5);
        // Approach segment and clean segment unchanged.
        assert_eq!(flap_schedule(4.5, -300.0), 0.5);
        assert_eq!(flap_schedule(8.0, 0.0), 0.0);
    }

    #[test]
    fn the_landing_flap_is_selected_a_mile_before_the_gate() {
        // #199: at 3 nm the landing flap went out AT the 1,000 ft gate, so
        // the gate could never find the landing configuration. Now 3.98 nm
        // on a 3 degree path: EGNX 09's second approach (2026-09-30) at
        // 3.26 nm and 150 kt would have had it selected.
        assert!((landing_flap_nm() - 3.98).abs() < 0.02, "{}", landing_flap_nm());
        assert_eq!(flap_schedule(3.9, 0.0), 1.0);
        assert_eq!(flap_schedule(3.26, -10.0), 1.0);
        assert_eq!(flap_schedule(4.1, 0.0), 0.5);
        let gate_nm = 1000.0 / GLIDEPATH_FT_PER_NM - TOUCHDOWN_AIM_NM;
        assert!(landing_flap_nm() - gate_nm >= 1.0 - 1e-9);
    }

    #[test]
    fn cross_track_sign_is_right_of_course() {
        // Aircraft to the EAST of a north-landing runway's extended centreline (which
        // runs south of the threshold) → right of the inbound course → positive.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 3.0); // 3 nm south, on centreline
        assert!(
            cross_track_nm(lat, lon, &rwy_north()).abs() < 0.05,
            "on centreline ~0"
        );
        let east = (lat, lon + 0.02);
        assert!(
            cross_track_nm(east.0, east.1, &rwy_north()) > 0.0,
            "east = right = positive"
        );
    }

    #[test]
    fn bearing_cardinals() {
        // Due north / east / south / west from the equator.
        assert!((bearing_deg(0.0, 0.0, 1.0, 0.0) - 0.0).abs() < 0.5, "north");
        assert!((bearing_deg(0.0, 0.0, 0.0, 1.0) - 90.0).abs() < 0.5, "east");
        assert!(
            (bearing_deg(0.0, 0.0, -1.0, 0.0) - 180.0).abs() < 0.5,
            "south"
        );
        assert!(
            (bearing_deg(0.0, 0.0, 0.0, -1.0) - 270.0).abs() < 0.5,
            "west"
        );
    }

    #[test]
    fn heading_diff_wraps() {
        assert_eq!(heading_diff(10.0, 350.0), 20.0);
        assert_eq!(heading_diff(350.0, 10.0), 20.0);
        assert!((heading_diff(90.0, 270.0) - 180.0).abs() < 1e-9);
        assert_eq!(heading_diff(45.0, 45.0), 0.0);
    }

    #[test]
    fn parses_a_real_type_100_line() {
        // A real apt.dat land-runway record (ends 14/32).
        let line = "100 7.62 3 0 0.25 0 0 0 14 63.1361123 -142.5229183 0 0 1 0 0 0 32 63.1315349 -142.5209494 0 0 1 0 0 0";
        let (e1, e2, length_m, surface) = parse_runway_100(line).expect("parses");
        assert_eq!(e1.id, "14");
        assert_eq!(e2.id, "32");
        assert_eq!(surface, 3);
        // The two ends are the same runway → reciprocal courses (~180 apart).
        assert!((heading_diff(e1.course, e2.course) - 180.0).abs() < 2.0);
        // ~510 m strip; sanity-bound the length.
        assert!(
            length_m > 300.0 && length_m < 800.0,
            "length was {length_m}"
        );
    }

    #[test]
    fn rejects_malformed_and_non_runway_lines() {
        assert!(parse_runway_100("1 100 0 0 KJFK").is_none());
        assert!(parse_runway_100("100 7.62 3").is_none()); // too few fields
    }

    // Real-data smoke test against the actual Global Airports apt.dat. Ignored by
    // default (363 MB, slow, machine-specific path); run with:
    //   cargo test autoland::tests::real_apt_dat -- --ignored --nocapture
    #[test]
    #[ignore]
    fn real_apt_dat_finds_a_known_airport() {
        let apt = format!(
            "{}/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat",
            std::env::var("HOME").unwrap()
        );
        // Near KSEA (Seattle-Tacoma). Track roughly south to favour the 16s.
        let best = best_landing_runway(&apt, 47.449, -122.309, 160.0, 30.0, 1500.0)
            .expect("read apt.dat")
            .expect("a suitable runway near KSEA");
        println!(
            "selected: {} rwy {} @ {:.0} nm, {:.0} m, course {:.0}",
            best.icao, best.id, best.dist_nm, best.length_m, best.course_deg
        );
        assert_eq!(best.icao, "KSEA");
        assert!(best.length_m > 2000.0, "KSEA runways are long");
        assert!(best.dist_nm < 5.0, "should be right at the field");
    }

    // runway_by_id honours the arrival's already-chosen end: matches by ident,
    // skips soft surfaces, and returns None for an ident not at the field (the
    // caller then falls back to the track-based pick).
    #[test]
    fn runway_by_id_matches_the_named_hard_end() {
        let dir = std::env::temp_dir().join(format!("xcopilot_rwyid_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        // A hard runway 09/27 and a grass 05/23, both at ~test coords.
        std::fs::write(
            &path,
            "1 20 0 0 TEST Test Field\n\
             100 30 1 0 0.25 0 0 0 09 36.150 -5.360 0 0 1 0 0 0 27 36.150 -5.330 0 0 1 0 0 0\n\
             100 30 3 0 0.25 0 0 0 05 36.140 -5.350 0 0 1 0 0 0 23 36.160 -5.350 0 0 1 0 0 0\n",
        )
        .unwrap();
        let p = path.to_str().unwrap();
        // The into-wind end the arrival chose (27) resolves to the hard runway.
        let r = runway_by_id(p, 36.15, -5.34, "27", 40.0, None)
            .unwrap()
            .expect("27 is at the field");
        assert_eq!(r.id, "27");
        assert!(r.hard(), "27 is asphalt");
        // Its reciprocal is equally resolvable.
        assert_eq!(
            runway_by_id(p, 36.15, -5.34, "09", 40.0, None)
                .unwrap()
                .unwrap()
                .id,
            "09"
        );
        // The grass end is filtered out (not a landing runway for autoland).
        assert!(
            runway_by_id(p, 36.15, -5.34, "05", 40.0, None)
                .unwrap()
                .is_none(),
            "grass is skipped"
        );
        // An ident that isn't here → None (caller falls back to track pick).
        assert!(
            runway_by_id(p, 36.15, -5.34, "18", 40.0, None)
                .unwrap()
                .is_none(),
            "no 18 here"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    // The circuit handoff must FORCE its own runway, because the track-based
    // search is free to prefer a different AIRFIELD entirely.
    //
    // Southampton, 2026-07-30: the circuit turned final at EGHI and handed to
    // autoland with `forced_rwy: None`. The search chose EGHL 09 sixteen miles
    // north, and the aircraft flew away from the field it had just circled for
    // five minutes, pinned in Intercept below a glidepath it could never
    // capture. This pins the DIFFERENCE between the two picks: on downwind
    // (heading away from the landing end), the track search does not return
    // the near field, and forcing the ident does.
    /// A runway ident is NOT unique, and the old lookup resolved it by
    /// proximity alone. EGPH-bound over the Forth, the nearest "24" was EGPJ
    /// (Fife) nine miles away, and the ILS was flown to a different airport
    /// (2026-09-20). The previous test could never catch this: its two fields
    /// had different idents, so the ambiguity never arose.
    #[test]
    fn a_planned_arrival_lands_at_the_airport_the_plan_named() {
        let dir = std::env::temp_dir().join(format!("xvl_two_24s_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        // Both fields have a 24. EGPJ is NEARER the aircraft; EGPH is the
        // destination. Geometry taken from the real pair across the Forth.
        std::fs::write(
            &path,
            "1 395 0 0 EGPJ Fife\n\
             100 45 1 0 0.25 0 0 0 06 56.180 -3.420 0 0 1 0 0 0 24 56.190 -3.390 0 0 1 0 0 0\n\
             1 136 0 0 EGPH Edinburgh\n\
             100 45 1 0 0.25 0 0 0 06 55.940 -3.400 0 0 1 0 0 0 24 55.960 -3.340 0 0 1 0 0 0\n",
        )
        .unwrap();
        let p = path.to_str().unwrap();
        // North of Edinburgh, closer to Fife — where the aircraft actually was.
        let (lat, lon) = (56.166, -3.286);

        // The bug: an ident with no airport behind it goes to the near field.
        let bare = runway_by_id(p, lat, lon, "24", 40.0, None)
            .unwrap()
            .expect("some 24 is in range");
        assert_eq!(bare.icao, "EGPJ", "proximity picks the wrong field");

        // The fix: the plan names its destination, so there is nothing to pick.
        let planned = runway_by_id(p, lat, lon, "24", 40.0, Some("EGPH"))
            .unwrap()
            .expect("EGPH has a 24");
        assert_eq!(planned.icao, "EGPH");
        assert!(
            planned.dist_nm > bare.dist_nm,
            "and it is the FURTHER one, which is the whole point"
        );

        // An ident the named field does not have is None — the caller falls
        // back to the track search rather than silently landing elsewhere.
        assert!(runway_by_id(p, lat, lon, "13", 40.0, Some("EGPH"))
            .unwrap()
            .is_none());
        // Case and whitespace in the ICAO must not send us to the wrong place.
        assert_eq!(
            runway_by_id(p, lat, lon, "24", 40.0, Some(" egph "))
                .unwrap()
                .map(|r| r.icao),
            Some("EGPH".to_string())
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_forced_ident_beats_the_track_search_at_the_wrong_field() {
        let dir = std::env::temp_dir().join(format!("xvl_circuit_rwy_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        // NEAR field: runway 02/20, the one we flew the pattern around.
        // FAR field ~16 nm north: runway 09/27, aligned nothing like our track.
        std::fs::write(
            &path,
            "1 20 0 0 NEAR Near Field\n\
             100 45 1 0 0.25 0 0 0 02 50.950 -1.360 0 0 1 0 0 0 20 50.965 -1.350 0 0 1 0 0 0\n\
             1 620 0 0 FAR Far Field\n\
             100 45 1 0 0.25 0 0 0 09 51.190 -1.040 0 0 1 0 0 0 27 51.190 -1.010 0 0 1 0 0 0\n",
        )
        .unwrap();
        let p = path.to_str().unwrap();
        // Downwind for 20: abeam the near field, tracking roughly 020 — i.e.
        // the RECIPROCAL of the landing direction, which is what made the
        // track search look elsewhere in the first place.
        let (lat, lon, hdg) = (50.958, -1.330, 20.0);

        // Forcing the pattern's ident lands us at the near field, every time.
        let forced = runway_by_id(p, lat, lon, "20", 40.0, None)
            .unwrap()
            .expect("20 is the pattern runway, right here");
        assert_eq!(forced.id, "20");
        assert!(
            forced.dist_nm < 5.0,
            "forced pick must be the field we are circling, got {:.1} nm",
            forced.dist_nm
        );

        // The bare track search is NOT required to agree, and that is the whole
        // point: whatever it returns, it is not authoritative for a circuit.
        // If it ever wanders to the far field, the forced pick above is what
        // keeps the aircraft home.
        if let Ok(Some(searched)) = best_landing_runway(p, lat, lon, hdg, 40.0, 1200.0) {
            assert!(
                forced.dist_nm <= searched.dist_nm,
                "forcing the ident must never be farther than the search: \
                 forced {} at {:.1} nm vs searched {} at {:.1} nm",
                forced.id,
                forced.dist_nm,
                searched.id,
                searched.dist_nm
            );
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    // X-Plane 12 numbers paved surfaces 20-38 and 50-57, not just the legacy
    // asphalt=1 / concrete=2. Accepting only [1, 2] hid 36% of the world's
    // usable runways from the emergency autoland, Heathrow among them.
    #[test]
    fn xp12_paved_surface_codes_are_landable() {
        // Legacy pair, still valid.
        assert!(is_hard_surface(1), "asphalt");
        assert!(is_hard_surface(2), "concrete");

        // The one that started it: EGHI 20 is surface 22, and it sat 2.16 nm
        // away while the search took Lasham at 15.81.
        assert!(is_hard_surface(22), "EGHI, KCLT, LTFJ");
        // A spread of real fields across the XP12 material range.
        for (code, who) in [
            (23, "ENGM / PANC"),
            (25, "EGLL"),
            (26, "EKCH / ENBR"),
            (27, "EFHK"),
            (29, "EGLL / EBCI"),
            (30, "EGLC / BIRK"),
            (33, "EGAA / LTFM"),
            (34, "ESSA / EKCH"),
            (35, "EBBR"),
            (37, "OMDB / LTAC"),
            (50, "CYYC / UKBB"),
            (51, "KSEA"),
            (55, "ULLI / KPDX"),
            (57, "KGEG"),
        ] {
            assert!(is_hard_surface(code), "surface {code} is paved: {who}");
        }

        // The soft ones must NEVER become a runway — this is why the check
        // stays a whitelist.
        for (code, what) in [
            (3, "turf"),
            (4, "dirt"),
            (5, "gravel"),
            (12, "dry lakebed"),
            (13, "water"),
            (14, "snow"),
            (15, "transparent"),
        ] {
            assert!(
                !is_hard_surface(code),
                "surface {code} ({what}) is not landable"
            );
        }

        // Unused/unknown codes are rejected rather than assumed paved.
        assert!(!is_hard_surface(0));
        assert!(!is_hard_surface(45), "39-49 unused in the global apt.dat");
        assert!(!is_hard_surface(99));
    }

    // A forced ident must survive the surface gate. `runway_by_id` filters on
    // hard() too, so before the XP12 codes were accepted, the circuit forcing
    // its own runway at Southampton would STILL have been rejected here and
    // fallen back to the track search that chose the wrong airfield.
    #[test]
    fn a_forced_ident_resolves_on_an_xp12_paved_runway() {
        let dir = std::env::temp_dir().join(format!("xvl_surf22_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        // EGHI-shaped: runway 02/20, surface 22, 1885 m.
        std::fs::write(
            &path,
            "1 44 0 0 EGHI Southampton\n\
             100 45 22 0 0.25 0 0 0 02 50.9425 -1.3660 0 0 1 0 0 0 20 50.9585 -1.3520 0 0 1 0 0 0\n",
        )
        .unwrap();
        let p = path.to_str().unwrap();
        let r = runway_by_id(p, 50.984, -1.311, "20", 40.0, None)
            .unwrap()
            .expect("surface 22 is a paved runway and must resolve");
        assert_eq!(r.id, "20");
        assert_eq!(r.surface, 22);
        assert!(r.dist_nm < 5.0, "it is right there: {:.1} nm", r.dist_nm);
        let _ = std::fs::remove_dir_all(&dir);
    }

    // Gibraltar 2026-09-11: the straight-in handed off at 195 kt and the gear
    // and flaps went out at once, 40 kt over the Baron's placards. The config
    // now waits for the speed the profile allows and goes out as it decays.
    #[test]
    fn gear_and_flaps_wait_for_their_placard_speeds() {
        // Baron 58: VLO 152, approach flap Vfe 152, full flap Vfe 122.
        const BARON: [(f64, Option<f64>); 3] =
            [(0.0, None), (0.5, Some(152.0)), (1.0, Some(122.0))];
        assert!(
            !gear_may_extend(195.0, Some(152.0)),
            "no gear at 195 on a 152 placard"
        );
        assert!(gear_may_extend(150.0, Some(152.0)));
        assert!(
            gear_may_extend(195.0, None),
            "no figure, no gate (the old behaviour)"
        );

        assert_eq!(
            flap_within_limits(0.5, 195.0, &BARON),
            0.0,
            "held clean at 195"
        );
        assert_eq!(
            flap_within_limits(0.5, 145.0, &BARON),
            0.5,
            "approach flap once under 152"
        );
        assert_eq!(
            flap_within_limits(1.0, 130.0, &BARON),
            0.5,
            "full wanted, 130 kt: only approach"
        );
        assert_eq!(
            flap_within_limits(1.0, 100.0, &BARON),
            1.0,
            "full flap under 122"
        );
        assert_eq!(
            flap_within_limits(0.0, 195.0, &BARON),
            0.0,
            "retraction never gated"
        );
        assert_eq!(
            flap_within_limits(1.0, 195.0, &[]),
            1.0,
            "no flap profile: schedule stands"
        );
    }

    // The approach speed is what you cross the threshold at, NOT what you fly
    // the intercept at. Commanding it from the moment autoland arms flew an
    // SF50 (approach 87 kt) at 92 kt nine miles out with the gear up, and the
    // airframe's own gear warning called it (Southampton, 2026-07-30).
    #[test]
    fn the_approach_speed_is_flown_at_the_faf_not_at_range() {
        const VAPP: f64 = 87.0; // SF50, profiles/cirrus_vision_sf50.yaml

        // Far out: clean manoeuvring speed, near what it was actually flying
        // when the circuit handed off (125 kt).
        let far = scheduled_ias(16.0, VAPP);
        assert!(
            (120.0..=132.0).contains(&far),
            "clean intercept speed should be ~126 kt, got {far:.0}"
        );
        assert_eq!(far, scheduled_ias(40.0, VAPP), "flat beyond the ramp");

        // The failure datapoint: 8.8 nm out it was doing 92. It should be well
        // above the approach speed there.
        let at_bug = scheduled_ias(8.8, VAPP);
        assert!(
            at_bug > VAPP + 20.0,
            "at 8.8 nm we must still be clean-ish, got {at_bug:.0}"
        );

        // Gear comes down at 6 nm — comfortably below the SF50's 160 kt Vlo.
        assert!(scheduled_ias(6.0, VAPP) < 160.0, "gear speed must be legal");

        // Stabilised at the approach speed by the FAP and all the way in.
        assert_eq!(scheduled_ias(FAP_NM, VAPP), VAPP);
        assert_eq!(scheduled_ias(2.0, VAPP), VAPP);
        assert_eq!(scheduled_ias(0.0, VAPP), VAPP);

        // Monotonic: never speeds up as it gets closer, never below Vapp.
        let mut prev = f64::MAX;
        let mut d = 20.0;
        while d >= 0.0 {
            let v = scheduled_ias(d, VAPP);
            assert!(
                v <= prev + 1e-9,
                "speed must not rise on the way in at {d} nm"
            );
            assert!(v >= VAPP - 1e-9, "never below the approach speed at {d} nm");
            prev = v;
            d -= 0.25;
        }

        // Scales with the airframe, so a 172 is not asked to fly a jet profile.
        assert!(scheduled_ias(16.0, 65.0) < 100.0, "172 stays a 172");
    }

    /// The pursuit lead must stay FIXED all the way to the threshold. It did
    /// not: flooring the aim at the threshold let the effective lead shrink
    /// inside one lead distance, which is the d/2 law the fixed lead was
    /// introduced to replace, back on short final where being lined up
    /// matters most. Southampton: the cut sat at 16 degrees from 0.87 nm to
    /// 0.32, then snapped to the runway course over the numbers with the
    /// aircraft still 6 degrees off and still turning (2026-08-01).
    #[test]
    fn the_pursuit_lead_does_not_collapse_on_short_final() {
        let (gs, len) = (105.0, 1723.0); // the Baron at EGHI 02
        let lead = |along: f64| along - pursuit_aim_nm(along, gs, len);

        // The same lead at every distance, far out and short final alike.
        let far = lead(1.68);
        for along in [1.2, 0.9, 0.7, 0.5, 0.35] {
            assert!(
                (lead(along) - far).abs() < 1e-9,
                "lead collapsed to {:.3} at {along} nm (was {far:.3})",
                lead(along)
            );
        }

        // Which means the aim walks PAST the threshold, onto the runway. That
        // is the point: the extended centreline and the runway centreline are
        // the same line.
        assert!(pursuit_aim_nm(0.35, gs, len) < 0.0);

        // ...but never past the far end of it.
        let past = -pursuit_aim_nm(0.0, gs, len);
        assert!(
            past <= len / M_PER_NM,
            "aim {past:.2} nm past the threshold on a {:.2} nm runway",
            len / M_PER_NM
        );

        // The cut therefore keeps shrinking with the offset instead of
        // holding: half the offset at the same range is half the cut.
        let cut = |xtk: f64, along: f64| (xtk / (along - pursuit_aim_nm(along, gs, len))).atan();
        assert!(cut(0.25, 0.7) > cut(0.125, 0.7));
        // ...and closing in at a FIXED offset no longer grows it.
        assert!((cut(0.1, 1.2) - cut(0.1, 0.4)).abs() < 1e-9);
    }
    #[test]
    fn the_self_pick_knows_the_airframe_the_wind_and_the_longer_parallel() {
        // The A330 lands in 4,750 ft (1,448 m): nothing under 1,882 m. A 172,
        // or no figure on file, keeps the old 1,200.
        assert!((self_pick_min_length_m(Some(4750.0 * 0.3048)) - 1882.0).abs() < 1.0);
        assert_eq!(self_pick_min_length_m(Some(407.0)), 1200.0);
        assert_eq!(self_pick_min_length_m(None), 1200.0);
        assert_eq!(self_pick_min_length_m(Some(f64::NAN)), 1200.0);

        // Opposite ends of one runway, the downwind end nearer and dead
        // ahead: 12 kt on the tail loses to the into-wind end 3 nm further
        // on with a turn (LXGB 27 into 090 at 21 is the case on file).
        let wind = Some((240.0, 12.0));
        let downwind_end = landing_score(5.0, 0.0, 60.0, wind, 2500.0, 1200.0);
        let into_wind_end = landing_score(8.0, 40.0, 240.0, wind, 2500.0, 1200.0);
        assert!(
            into_wind_end < downwind_end,
            "{into_wind_end} vs {downwind_end}"
        );
        // With no wind known the nearer end ahead still wins, as before.
        assert!(
            landing_score(5.0, 0.0, 60.0, None, 2500.0, 1200.0)
                < landing_score(8.0, 40.0, 240.0, None, 2500.0, 1200.0)
        );
        // A crosswind is nobody's tailwind.
        assert_eq!(
            landing_score(5.0, 0.0, 60.0, Some((150.0, 20.0)), 1200.0, 1200.0),
            5.0
        );
        // Parallels: the threshold 0.2 nm nearer loses to 1,000 m more runway.
        assert!(
            landing_score(6.2, 5.0, 130.0, None, 3200.0, 1200.0)
                < landing_score(6.0, 5.0, 130.0, None, 2200.0, 1200.0)
        );
        // ...but length never outbids a field that is really nearer.
        assert!(
            landing_score(6.0, 5.0, 130.0, None, 1500.0, 1200.0)
                < landing_score(7.0, 5.0, 130.0, None, 4000.0, 1200.0)
        );
    }

    /// LIPK (2026-09-25): "land the aircraft" 6.4 nm from both Forli 30 and
    /// Cervia 11, with LIPE-LIPK on the shelf, went to Cervia. The plan's
    /// destination is now worth [`DESTINATION_BONUS_NM`].
    #[test]
    fn the_self_pick_prefers_the_plans_destination() {
        let apt = std::env::temp_dir().join(format!("xvl_selfpick_{}.dat", std::process::id()));
        std::fs::write(
            &apt,
            "1     97 0 0 LIPK Forli\n\
             100 45.00 23 1 0.25 0 3 0  12 44.2001695 12.0562203 0 0 3 3 0 1 30 44.1897225 12.0848141 147 8 3 1 0 0\n\
             1     18 0 0 LIPC Cervia Pisignano\n\
             100 45.00 1 701 0.25 0 3 1  11 44.2303638 12.2901111 119 0 3 1 0 0 29 44.2182383 12.3243840 116 0 3 7 0 0\n",
        )
        .unwrap();
        let path = apt.to_str().unwrap();
        let pick = |dest: Option<&str>| {
            best_landing_runway_in_wind(path, 44.1447, 12.2021, 8.0, 40.0, 1200.0, &|_| None, dest)
                .unwrap()
                .map(|r| format!("{} {}", r.icao.trim(), r.id))
        };
        assert_eq!(pick(None).as_deref(), Some("LIPC 11"), "the old pick, no plan");
        assert_eq!(pick(Some("LIPK")).as_deref(), Some("LIPK 30"));
        assert_eq!(pick(Some("lipk")).as_deref(), Some("LIPK 30"));
        // A destination out of range changes nothing.
        assert_eq!(pick(Some("EGLL")).as_deref(), Some("LIPC 11"));
        let _ = std::fs::remove_file(&apt);
    }

    #[test]
    fn the_vs_watchdog_excuses_a_slow_autopilot_and_names_what_is_commanded() {
        // LEMI: cmd 0, diving 2,300 and not recovering. Counts, every tick.
        // A 0.33 s tick, the MacBook's Final (the rate is per second).
        let t = 0.33;
        assert!(vs_not_held(-2300.0, None, t));
        assert!(vs_not_held(-2300.0, Some(-2300.0), t));
        assert!(vs_not_held(-2400.0, Some(-2300.0), t));
        // The A330 at the arm: a -1,200 step from level, closing at 40 fpm a
        // tick. Far from the command, and being flown toward it.
        assert!(!vs_not_held(1100.0, Some(1140.0), t));
        // Closing too slowly to be the autopilot's doing.
        assert!(vs_not_held(1100.0, Some(1105.0), t));
        // Inside the band is held, whatever the trend.
        assert!(!vs_not_held(650.0, Some(100.0), t));
        // The same closing RATE judged alike on a slower tick: 40 fpm in
        // 0.33 s is 121 fpm/s, held; at 0.56 s 40 fpm is 71 fpm/s, still
        // held; 5 fpm in 0.56 s (9 fpm/s) is not.
        assert!(!vs_not_held(1100.0, Some(1140.0), 0.56));
        assert!(vs_not_held(1100.0, Some(1105.0), 0.56));
        assert!(vs_not_held_line(-1200.0).contains("descent"));
        assert!(vs_not_held_line(150.0).contains("climb"));
        assert!(vs_not_held_line(0.0).contains("level"));
    }

    #[test]
    fn the_altitude_dial_is_parked_above_the_arm() {
        // LEAX 2026-09-15: the plan's 2,600 ft dialled, the arm at 2,733 ft
        // indicated, the capture already arresting the descent. Parked 1,000
        // up, rounded to the hundred; VS re-selected by the caller.
        assert_eq!(parked_altitude_dial_ft(2600.0, 2733.0), Some(3800.0));
        // Dial at the aircraft's own altitude (an ALT hold at the arm): parked.
        assert_eq!(parked_altitude_dial_ft(2700.0, 2700.0), Some(3700.0));
        // Cruise altitude still dialled far above: nothing to do.
        assert_eq!(parked_altitude_dial_ft(8000.0, 2733.0), None);
        // Just inside the clear band counts as in the way.
        assert_eq!(parked_altitude_dial_ft(3100.0, 2733.0), Some(3800.0));
        assert_eq!(parked_altitude_dial_ft(3300.0, 2733.0), None);
    }

    #[test]
    fn the_glidepath_aims_at_the_probed_threshold() {
        // LEAX: header 160 ft, the 30 threshold probed at 117, the aim point
        // 300 m in at 121 - the higher of the two is the aim.
        assert_eq!(
            aim_elevation_ft(160.0, Some(&[117.0, 121.0])),
            (121.0, Some(121.0))
        );
        // Mesh and header agree within the trust band: the header stays.
        assert_eq!(
            aim_elevation_ft(160.0, Some(&[150.0, 158.0])),
            (160.0, Some(158.0))
        );
        // No probe (old plugin, paused sim): the header, and nothing seen.
        assert_eq!(aim_elevation_ft(160.0, None), (160.0, None));
        assert_eq!(aim_elevation_ft(160.0, Some(&[])), (160.0, None));
        // An absurd probe (a point in the sea beside the strip) is reported,
        // never flown.
        assert_eq!(
            aim_elevation_ft(160.0, Some(&[-500.0, -480.0])),
            (160.0, Some(-480.0))
        );
        // The LEAX approach: aimed at 121 the threshold crossing is ~57 ft
        // over the pavement; aimed at 160 it was 99 ft (radio) and balked.
        let tch = |aim: f64| aim + TOUCHDOWN_AIM_NM * GLIDEPATH_FT_PER_NM - 117.0;
        assert!(tch(121.0) < 65.0 && tch(121.0) > 45.0, "{}", tch(121.0));
        assert!(tch(160.0) > 90.0);
    }
}

#[cfg(test)]
mod glidepath_gain_tests {
    use super::glidepath_gain_fpm_per_ft;

    /// The glidepath correction closes an error over a fixed distance of
    /// final: the 172's gain stands, a 737 at 170 kt gets 2.6x it, and a
    /// 50 ft-high 737 over the numbers is answered with a real descent
    /// (LFLL 35L, 2026-09-17: 86 fpm was the whole correction).
    #[test]
    fn glidepath_gain_scales_with_groundspeed() {
        assert_eq!(glidepath_gain_fpm_per_ft(65.0), 1.2);
        assert_eq!(
            glidepath_gain_fpm_per_ft(40.0),
            1.2,
            "slow airframes keep the old law"
        );
        let jet = glidepath_gain_fpm_per_ft(170.0);
        // The 172's gain scaled by the groundspeed ratio, 2.6x it.
        assert!((jet - 1.2 * 170.0 / 65.0).abs() < 0.02, "{jet}");
        assert!(
            (glidepath_gain_fpm_per_ft(400.0) - 3.6).abs() < 1e-9,
            "capped"
        );
        assert!(
            50.0 * jet > 150.0,
            "50 ft high on a jet earns more than 150 fpm"
        );
    }
}

#[cfg(test)]
mod approach_power_floor_tests {
    use super::{approach_power_floor, ApproachPhase::*};

    /// The 737 (thrust floor 0.3): on final at Vref the lever never goes
    /// under 0.3; 20 kt fast it may idle; a piston with no floor idles; a
    /// reposition under 500 ft is full power whatever the speed.
    #[test]
    fn the_lever_floor_follows_the_phase_and_the_airframe() {
        assert_eq!(
            approach_power_floor(Final, 300.0, 0.0, Some(0.3), true),
            0.3
        );
        assert_eq!(
            approach_power_floor(Final, 300.0, -6.0, Some(0.3), true),
            0.3
        );
        assert_eq!(
            approach_power_floor(Final, 300.0, -20.0, Some(0.3), true),
            0.1
        );
        assert_eq!(approach_power_floor(Final, 300.0, 0.0, None, true), 0.1);
        assert_eq!(
            approach_power_floor(Intercept, 140.0, -58.0, Some(0.3), false),
            1.0
        );
        assert_eq!(
            approach_power_floor(Intercept, 1500.0, -58.0, Some(0.3), false),
            0.1
        );
        assert_eq!(approach_power_floor(Flare, 20.0, 0.0, Some(0.3), true), 0.1);
    }

    /// LEMG 13, 2026-09-26, every final: flaps 10 out from the FAP, the
    /// target floored to that configuration's 152 kt, and the 737 sat at 161
    /// to 164 kt for a minute with the lever flipping 0.1 and 0.3 on the
    /// ten-knot edge (0.3 on a 3 degree path with flaps 10 accelerates it).
    /// Short of the landing flaps the speed law may idle; with them out the
    /// floor stands as before. An airframe with no floor never had one.
    #[test]
    fn the_thrust_floor_is_the_landing_configurations() {
        // 161 kt against the flaps-10 floor of 152: nine knots fast.
        assert_eq!(
            approach_power_floor(Final, 1200.0, -9.0, Some(0.3), false),
            0.1
        );
        assert_eq!(
            approach_power_floor(Final, 1200.0, -9.0, Some(0.3), true),
            0.3
        );
        assert_eq!(approach_power_floor(Final, 1200.0, -9.0, None, false), 0.1);
        assert_eq!(approach_power_floor(Final, 1200.0, -9.0, None, true), 0.1);
    }

    /// EGCC 23L, 2026-09-21: the servos stayed on through the rollout and the
    /// taxi, and the heading servo wound the yoke over on the ground.
    #[tokio::test]
    async fn the_autopilot_comes_off_at_the_wheels() {
        use crate::xplane::FakeXPlane;
        let xp = FakeXPlane::with_reads(&[(super::SERVOS_ON, 1.0)]);
        // The fake never moves, so the readback disagrees and it is SAID so.
        assert_eq!(super::disconnect_autopilot(&xp).await, Some(false));
        assert!(xp.ran(super::CMD_SERVOS_FDIR_OFF));
        // Servos already out by the arrest, the master and yaw damper still
        // on (King Air LEMG 13, #197): both come out.
        let xp = FakeXPlane::with_reads(&[
            (super::SERVOS_ON, 0.0),
            (super::AP_MASTER_ON, 1.0),
            (super::YAW_DAMPER_ON, 1.0),
        ]);
        assert!(super::disconnect_autopilot(&xp).await.is_some());
        assert!(xp.ran(super::CMD_SERVOS_FDIR_OFF));
        assert!(xp.ran(super::CMD_YAW_DAMPER_OFF));
        // All off (a hand-flown landing under autoland's callouts): left.
        let xp = FakeXPlane::with_reads(&[
            (super::SERVOS_ON, 0.0),
            (super::AP_MASTER_ON, 0.0),
            (super::YAW_DAMPER_ON, 0.0),
        ]);
        assert_eq!(super::disconnect_autopilot(&xp).await, None);
        assert!(!xp.ran(super::CMD_SERVOS_FDIR_OFF));
    }

    /// EDLP 24 (2026-09-24): the SF50's 0.35 went to neutral in one tick at
    /// +190 fpm, and neutral is the TRIMMED approach attitude: the sink was
    /// back at -590 two seconds later. A declared ease walks the pull down.
    #[test]
    fn a_declared_balloon_ease_walks_the_pull_down_instead_of_dropping_it() {
        let gains = super::FlareGains {
            balloon_ease_per_s: Some(0.15),
            ..Default::default()
        };
        let mut law = super::FlarePitch::with_gains(gains);
        // Sinking hard: the ratchet builds the pull to the cap.
        for _ in 0..10 {
            law.step(-100.0, -600.0, 0.0, 0.22, true);
        }
        let held = law.step(-100.0, -600.0, 0.0, 0.22, true);
        assert!(held > 0.3, "{held}");
        // Now climbing: one tick eases 0.15 x 0.22, it does not let go.
        let next = law.step(-100.0, 150.0, 400.0, 0.22, true);
        assert!((held - next - 0.033).abs() < 1e-6, "{held} -> {next}");
        // The default law (the 172, the 737, the Citation) still lets go.
        let mut old = super::FlarePitch::default();
        for _ in 0..10 {
            old.step(-100.0, -600.0, 0.0, 0.22, true);
        }
        assert_eq!(old.step(-100.0, 150.0, 400.0, 0.22, true), 0.0);
        // And a reset keeps the airframe's numbers.
        law.reset();
        for _ in 0..10 {
            law.step(-100.0, -600.0, 0.0, 0.22, true);
        }
        let h = law.step(-100.0, -600.0, 0.0, 0.22, true);
        assert!(h - law.step(-100.0, 150.0, 400.0, 0.22, true) < 0.05);
    }

    #[test]
    fn a_flare_elevator_block_scales_the_law_and_a_bad_one_is_the_default() {
        let p = crate::aircraft::FlareElevatorProfile {
            gain: 0.5,
            trim_gain: 3.0,
            max: 0.30,
            max_push: 0.05,
            balloon_ease_per_s: Some(0.15),
            tau_s: None,
            model: None,
        };
        let g = super::FlareGains::from_profile(Some(p));
        let d = super::FlareGains::default();
        assert!((g.p - 0.5 * d.p).abs() < 1e-12 && (g.d - 0.5 * d.d).abs() < 1e-12);
        assert!((g.trim_rate - 3.0 * d.trim_rate).abs() < 1e-12);
        assert_eq!((g.max, g.min, g.balloon_ease_per_s), (0.30, -0.05, Some(0.15)));
        let bad = crate::aircraft::FlareElevatorProfile { max: 2.0, ..p };
        assert_eq!(super::FlareGains::from_profile(Some(bad)), d);
        assert_eq!(super::FlareGains::from_profile(None), d);
    }

    /// LSZS RNAV 03 (2026-09-24): the MAP is 1.98 nm from the threshold,
    /// 996 ft above it, 380 m right of the extended centreline. Handed the
    /// visual segment's 4.74 degrees, the landing takes it from there on its
    /// own path; at 3 degrees the same point would be 300 ft high.
    #[test]
    fn a_visual_segment_is_flown_on_its_own_path() {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let rwy = super::Runway {
            icao: "LSZS".into(),
            id: "03".into(),
            thr_lat: 46.5270195,
            thr_lon: 9.8780890,
            course_deg: 29.0,
            length_m: 1842.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 5602.0,
        };
        let (map_lat, map_lon) = (46.5, 9.850666667);
        // 996 ft over the threshold at 1.98 nm, measured to the aim point
        // 300 m past it: what the follower hands down.
        let aim = super::TOUCHDOWN_AIM_NM;
        let deg = (996.0_f64 / ((1.98 + aim) * 6076.12)).atan().to_degrees();
        super::set_next_glidepath_deg(Some(deg));
        assert!((super::arm_glidepath() - deg).abs() < 1e-9);
        assert!(deg > 4.2 && deg < 4.6, "{deg}");
        let g = super::guidance(map_lat, map_lon, 6598.0, 996.0, 80.0, 90.0, &rwy, 87.0, 25.0, 0.0);
        let above = 6598.0 - g.glidepath_alt_ft;
        assert!(above.abs() < 40.0, "on the steep path at the MAP: {above:.0} ft");
        assert_eq!(g.balk, None, "no balk at the MAP");
        assert_eq!(g.phase, super::ApproachPhase::Final);
        assert!(g.target_vs_fpm < -450.0 && g.target_vs_fpm > -1000.0, "{}", g.target_vs_fpm);
        // The next arm without a handed angle is 3 degrees again.
        assert_eq!(super::arm_glidepath(), 3.0);
        assert_eq!(super::gp_ft_per_nm(), super::GLIDEPATH_FT_PER_NM);
        let g3 = super::guidance(map_lat, map_lon, 6598.0, 996.0, 80.0, 90.0, &rwy, 87.0, 25.0, 0.0);
        assert!(6598.0 - g3.glidepath_alt_ft > 250.0, "a 3 degree path calls it high");
    }

    /// LEMG 13 SR22 circuit, 2026-09-29 (sortie 1790692781-full_auto): the
    /// circuit steepened its path to 3.26 degrees over the ground (#182)
    /// and autoland took that for LSZS's valley: no pattern when it arrived
    /// 560 ft high, and after the balk no published missed approach, so
    /// "climbing straight ahead. Your aircraft." to nobody at 805 ft. A
    /// circuit's angle is flown but is not a published visual segment.
    #[test]
    fn a_circuit_path_is_steep_but_not_a_valley() {
        super::set_next_circuit_glidepath_deg(Some(3.2576));
        assert!((super::arm_glidepath() - 3.2576).abs() < 1e-9, "the angle is flown");
        assert!(!super::published_steep(), "a circuit's path is not a valley");
        super::set_next_glidepath_deg(Some(4.5));
        assert!((super::arm_glidepath() - 4.5).abs() < 1e-9);
        assert!(super::published_steep(), "the follower's MAP path is");
        // A handed 3 degrees (or none) is never steep, whoever handed it.
        super::set_next_glidepath_deg(Some(3.0));
        assert_eq!(super::arm_glidepath(), 3.0);
        assert!(!super::published_steep());
        assert_eq!(super::arm_glidepath(), 3.0);
        assert!(!super::published_steep());
    }

    /// The same sortie's other half: the steep branch's missed approach that
    /// cannot be flown falls through to the terrain-probed pattern, never to
    /// "Your aircraft" (source-pinned: the loop is not a plain test).
    #[test]
    fn an_unflyable_missed_procedure_is_never_your_aircraft() {
        let src = include_str!("autoland.rs");
        let body = src
            .split("if wanted && steep {")
            .nth(1)
            .expect("the steep branch")
            .split("if wanted && reversals_flown >= crate::reversal::MAX_REVERSALS")
            .next()
            .unwrap();
        assert!(!body.contains("Your aircraft"), "no hand-back to nobody");
        assert!(body.contains("missed_procedure_tried = true"), "asked once");
        assert!(src.contains("let steep = published_steep() && !missed_procedure_tried;"));
        let circuit = include_str!("circuit.rs");
        assert!(circuit.contains("set_next_circuit_glidepath_deg(Some(descent_shape"));
        assert!(!circuit.contains("autoland::set_next_glidepath_deg("));
    }
}

/// The 737's flare against a plant FITTED to its own flares: the seven LEMG
/// landings of 2026-09-26 (six on 13, then 12 with the first model law),
/// the elevator written against the sink that followed, 35 fpm rms over
/// all seven arrests from the gate to the wheels. A standing pull holds a
/// standing sink change of 4,970 fpm per unit through lags of 0.84 and
/// 3.09 s after 1.35 s of dead time; the aircraft pitches down by itself
/// over the last 43 ft (405 fpm at the wheels, the tail's ground effect);
/// the sink follows the speed at 63 fpm a knot, and the speed follows the
/// lever, the pull and the sink (-1.08 + 2.99 lever - 2.05 pull - 0.40 per
/// 1,000 fpm of sink, kt/s, fitted to the same ticks). Fitted flare by
/// flare the gain ran 4,020 to 6,430 and the dead time 1.16 to 2.17 s, so
/// every landing is flown against every flare's own plant. The first fit
/// (2,600 fpm, two 1 s lags, 1.8 s, no ground) landed every gate soft and
/// was wrong: LEMG 12 flew its law to 407.
#[cfg(test)]
mod lemg_737_flare_tests {
    use super::{arrest_targets_fpm, flare_target_with, FlareModel, FlarePower, RetardKey};
    use std::collections::VecDeque;

    /// Where each recorded arrest began: the `arrest_hands` tick.
    #[derive(Clone, Copy)]
    struct Gate {
        radio_ft: f64,
        vvi_fpm: f64,
        trend_fpm_s: f64,
        ias_kts: f64,
        throttle: f64,
        /// The sink this approach's trim held with the elevator at zero
        /// (fitted; LEMG 12 arrived at -774 on a trim that held -496).
        trimmed_sink_fpm: f64,
        /// The sink at the wheels, as flown.
        flown_fpm: f64,
    }

    /// 16:29, 17:04 (its FIRST contact; the scored 278 is after a bounce),
    /// 17:17, 17:25, 17:32, 17:43Z on 13, 18:59Z on 12, and 20:03Z on 13
    /// (the first prediction flare: 277 at the wheels, the lever never
    /// retarded).
    const GATES: [Gate; 8] = [
        Gate {
            radio_ft: 79.2,
            vvi_fpm: -727.0,
            trend_fpm_s: -4.0,
            ias_kts: 145.6,
            throttle: 0.309,
            trimmed_sink_fpm: -853.0,
            flown_fpm: -445.0,
        },
        Gate {
            radio_ft: 81.2,
            vvi_fpm: -862.0,
            trend_fpm_s: -3.0,
            ias_kts: 147.8,
            throttle: 0.300,
            trimmed_sink_fpm: -1021.0,
            flown_fpm: -794.0,
        },
        Gate {
            radio_ft: 79.5,
            vvi_fpm: -748.0,
            trend_fpm_s: -26.0,
            ias_kts: 143.6,
            throttle: 0.357,
            trimmed_sink_fpm: -903.0,
            flown_fpm: -537.0,
        },
        Gate {
            radio_ft: 82.5,
            vvi_fpm: -705.0,
            trend_fpm_s: -56.0,
            ias_kts: 146.1,
            throttle: 0.372,
            trimmed_sink_fpm: -727.0,
            flown_fpm: -769.0,
        },
        Gate {
            radio_ft: 81.7,
            vvi_fpm: -752.0,
            trend_fpm_s: -72.0,
            ias_kts: 145.3,
            throttle: 0.301,
            trimmed_sink_fpm: -950.0,
            flown_fpm: -411.0,
        },
        Gate {
            radio_ft: 78.9,
            vvi_fpm: -789.0,
            trend_fpm_s: -53.0,
            ias_kts: 145.1,
            throttle: 0.301,
            trimmed_sink_fpm: -1096.0,
            flown_fpm: -408.0,
        },
        Gate {
            radio_ft: 81.5,
            vvi_fpm: -774.0,
            trend_fpm_s: 27.0,
            ias_kts: 144.7,
            throttle: 0.441,
            trimmed_sink_fpm: -496.0,
            flown_fpm: -407.0,
        },
        Gate {
            radio_ft: 79.2,
            vvi_fpm: -797.0,
            trend_fpm_s: -2.0,
            ias_kts: 143.6,
            throttle: 0.304,
            trimmed_sink_fpm: -779.0,
            flown_fpm: -277.0,
        },
    ];
    const LEMG_12: usize = 6;
    const PREDICTION_FLOWN: usize = 7;

    #[derive(Clone, Copy, Debug)]
    struct Fit {
        fpm_per_elevator: f64,
        dead_s: f64,
    }
    const LAG1_S: f64 = 0.841;
    const LAG2_S: f64 = 3.088;
    const GROUND_FPM: f64 = 405.3;
    const GROUND_FT: f64 = 43.2;
    const FPM_PER_KT: f64 = 62.7;
    /// The 737 profile's approach speed (`reference.approach_kts`), the
    /// retard's speed key's reference.
    const APPROACH_IAS: f64 = 145.0;

    /// The retard as it flew until issue #76: held while the sink (with a
    /// second of its worsening) was over 400, nothing above 20 ft, and on
    /// 2026-09-28 not before the sink had come inside 185 once or 10 ft;
    /// then timed to the wheels at 0.03 to 0.08 a second. Kept to replay
    /// the landings it flew.
    #[derive(Clone, Copy)]
    struct SinkKeyed {
        inside_fpm: Option<f64>,
        by_ft: Option<f64>,
        max_per_s: f64,
        previous_sink: Option<f64>,
        last: Option<f64>,
        inside: bool,
    }

    impl SinkKeyed {
        /// As flown on 2026-09-28.
        fn as_flown() -> Self {
            SinkKeyed {
                inside_fpm: Some(185.0),
                by_ft: Some(10.0),
                max_per_s: 0.08,
                previous_sink: None,
                last: None,
                inside: false,
            }
        }
        /// LEMG 12's (2026-09-26): by height alone, up to 0.32 a second.
        fn height_only() -> Self {
            SinkKeyed {
                inside_fpm: None,
                by_ft: None,
                max_per_s: 0.32,
                ..Self::as_flown()
            }
        }
        fn step(&mut self, measured: f64, vvi: f64, dt: f64, radio_ft: f64) -> f64 {
            let sink = -vvi;
            let worsening = self
                .previous_sink
                .map_or(0.0, |old| ((sink - old) / dt).max(0.0));
            self.previous_sink = Some(sink);
            self.inside |= self.inside_fpm.is_some_and(|f| sink <= f);
            let floor = self.by_ft.is_some_and(|h| radio_ft <= h);
            let waiting = self.inside_fpm.is_some() && !self.inside && !floor;
            let current = measured.clamp(0.0, 1.0).min(self.last.unwrap_or(1.0));
            let rate = if waiting || radio_ft > 20.0 {
                0.0
            } else {
                let to_touch_s = radio_ft.max(0.0) / sink.max(100.0) * 60.0;
                (current / to_touch_s.max(0.5)).clamp(0.03, self.max_per_s)
            };
            let next = if self.last.is_none() || sink.max(sink + worsening) > 400.0 {
                current
            } else {
                (current - rate * dt.min(1.0)).max(0.0)
            };
            self.last = Some(next);
            next
        }
    }

    /// Which retard a replay flies.
    enum Retard {
        Keyed(FlarePower, crate::aircraft::FlarePowerProfile),
        Sink(SinkKeyed),
        /// Height, time and speed as flown on 2026-09-28 after issue #76,
        /// before the float key (issue #146).
        NoFloat(NoFloatKeyed, crate::aircraft::FlarePowerProfile),
    }

    /// The keyed retard without the float key, as flown at EGBB 15
    /// (2026-09-28): nothing above the retard height, nothing while more
    /// than `retard_at_s` from the wheels (a float counted at 100 fpm),
    /// and once due, nothing while slow. Kept to replay the landing it flew.
    #[derive(Default, Clone, Copy)]
    struct NoFloatKeyed {
        last: Option<f64>,
        due: bool,
    }

    impl NoFloatKeyed {
        fn step(
            &mut self,
            cfg: crate::aircraft::FlarePowerProfile,
            measured: f64,
            vvi: f64,
            dt: f64,
            radio_ft: f64,
            ias: f64,
        ) -> f64 {
            let current = measured.clamp(0.0, 1.0).min(self.last.unwrap_or(1.0));
            let slow = ias < APPROACH_IAS - cfg.retard_slow_kts.unwrap_or(5.0);
            let above = cfg.retard_below_ft.is_some_and(|h| radio_ft > h);
            let to_touch_s = (radio_ft.max(0.0) / (-vvi).max(100.0) * 60.0).max(0.5);
            let fast = cfg.retard_fast_kts.is_some_and(|k| ias > APPROACH_IAS + k);
            self.due |= !above && (fast || cfg.retard_at_s.is_none_or(|s| to_touch_s <= s));
            let next = if self.last.is_none() || above || !self.due || slow {
                current
            } else {
                let fastest = cfg
                    .retard_max_per_s
                    .unwrap_or(cfg.reduction_per_second)
                    .max(0.03);
                let rate = (current / to_touch_s).clamp(0.03, fastest);
                (current - rate * dt.min(1.0)).max(0.0)
            };
            self.last = Some(next);
            next
        }
    }

    impl Retard {
        /// The 737 profile's, today's law.
        fn profile() -> Self {
            Retard::Keyed(
                FlarePower::default(),
                profile_737().flare_power.expect("the 737 retards"),
            )
        }
        /// ...and the retard as it flew at EGBB 15.
        fn no_float() -> Self {
            Retard::NoFloat(
                NoFloatKeyed::default(),
                profile_737().flare_power.expect("the 737 retards"),
            )
        }
        fn step(&mut self, lever: f64, vvi: f64, dt: f64, h: f64, ias: f64) -> Option<f64> {
            match self {
                Retard::Keyed(p, cfg) => p.step(*cfg, lever, vvi, dt, h, Some(ias), Some(APPROACH_IAS)),
                Retard::Sink(s) => Some(s.step(lever, vvi, dt, h)),
                Retard::NoFloat(k, cfg) => Some(k.step(*cfg, lever, vvi, dt, h, ias)),
            }
        }
    }
    /// Each flare's own gain and dead time, in the order of [`GATES`].
    const PLANTS: [Fit; 8] = [
        Fit {
            fpm_per_elevator: 4814.0,
            dead_s: 1.37,
        },
        Fit {
            fpm_per_elevator: 5097.0,
            dead_s: 1.16,
        },
        Fit {
            fpm_per_elevator: 4990.0,
            dead_s: 1.29,
        },
        Fit {
            fpm_per_elevator: 6429.0,
            dead_s: 1.43,
        },
        Fit {
            fpm_per_elevator: 4622.0,
            dead_s: 1.50,
        },
        Fit {
            fpm_per_elevator: 4093.0,
            dead_s: 2.17,
        },
        Fit {
            fpm_per_elevator: 4021.0,
            dead_s: 1.77,
        },
        Fit {
            fpm_per_elevator: 4099.0,
            dead_s: 1.63,
        },
    ];

    struct Plant {
        fit: Fit,
        trimmed: f64,
        x1: f64,
        x2: f64,
        ias: f64,
        ias0: f64,
        t: f64,
        queue: VecDeque<(f64, f64)>,
        elev: f64,
        vvi: f64,
        /// The air: sink (fpm) and speed change (kt/s) the fit does not
        /// explain, taken from a recording ([`air_of`]); zero is still air.
        air_fpm: f64,
        air_kts_s: f64,
    }

    impl Plant {
        fn at(g: Gate, fit: Fit) -> Self {
            let x2 = g.vvi_fpm - g.trimmed_sink_fpm;
            Plant {
                fit,
                trimmed: g.trimmed_sink_fpm,
                x1: x2 + g.trend_fpm_s * LAG2_S,
                x2,
                ias: g.ias_kts,
                ias0: g.ias_kts,
                t: 0.0,
                queue: VecDeque::new(),
                elev: 0.0,
                vvi: g.vvi_fpm,
                air_fpm: 0.0,
                air_kts_s: 0.0,
            }
        }
        fn write(&mut self, elev: f64) {
            self.queue.push_back((self.t + self.fit.dead_s, elev));
        }
        /// `ias` Some = the recorded speed (a replay); None = the speed
        /// model flies it.
        fn step(&mut self, lever: f64, h: f64, ias: Option<f64>, dt: f64) {
            while self.queue.front().is_some_and(|&(t, _)| t <= self.t) {
                self.elev = self.queue.pop_front().map_or(0.0, |(_, e)| e);
            }
            let ground = GROUND_FPM * (1.0 - h.max(0.0) / GROUND_FT).max(0.0);
            self.x1 += (self.fit.fpm_per_elevator * self.elev - ground - self.x1) * dt / LAG1_S;
            self.x2 += (self.x1 - self.x2) * dt / LAG2_S;
            self.ias = match ias {
                Some(v) => v,
                None => self.ias + (speed_model(lever, self.elev, self.vvi) + self.air_kts_s) * dt,
            };
            self.vvi = self.trimmed + self.x2 + FPM_PER_KT * (self.ias - self.ias0) + self.air_fpm;
            self.t += dt;
        }
    }

    /// The fitted speed model, kt/s.
    fn speed_model(lever: f64, elev: f64, vvi: f64) -> f64 {
        -1.08 + 2.99 * lever - 2.05 * elev - 0.40 * vvi / 1000.0
    }

    /// The fitted plant IS the recorded aircraft: LEMG 12's own elevator,
    /// replayed open loop through its own flare's plant, gives back its
    /// sink, the plateau at -300 and the growth to -383 over the last 15 ft
    /// included.
    #[test]
    fn the_737_plant_is_fitted_to_its_own_flares() {
        // (seconds from the gate, radio ft, sink recorded, elevator written,
        // IAS).
        const REC: [(f64, f64, f64, f64, f64); 57] = [
            (0.00, 81.5, -774.0, 0.156, 144.7),
            (0.30, 76.7, -766.0, 0.166, 145.0),
            (0.60, 73.2, -756.0, 0.141, 145.3),
            (1.00, 69.8, -739.0, 0.139, 145.6),
            (1.30, 66.8, -718.0, 0.122, 145.9),
            (1.60, 63.8, -686.0, 0.109, 145.9),
            (2.00, 61.3, -650.0, 0.088, 145.7),
            (2.30, 58.8, -605.0, 0.079, 145.3),
            (2.70, 56.7, -561.0, 0.068, 145.0),
            (3.00, 55.1, -513.0, 0.055, 144.7),
            (3.30, 53.7, -464.0, 0.037, 144.6),
            (3.70, 52.5, -410.0, 0.012, 144.6),
            (4.00, 51.6, -362.0, -0.000, 144.8),
            (4.40, 50.8, -312.0, -0.000, 145.0),
            (4.70, 50.1, -276.0, -0.000, 145.2),
            (5.10, 49.3, -248.0, -0.000, 145.4),
            (5.40, 48.6, -233.0, -0.000, 145.5),
            (5.80, 47.7, -228.0, -0.000, 145.5),
            (6.10, 46.9, -231.0, -0.000, 145.5),
            (6.60, 45.2, -248.0, -0.000, 145.3),
            (7.00, 43.9, -263.0, -0.000, 145.3),
            (7.30, 42.5, -276.0, -0.000, 145.3),
            (7.70, 40.6, -290.0, -0.000, 145.4),
            (8.00, 39.1, -301.0, -0.000, 145.5),
            (8.30, 37.0, -310.0, 0.012, 145.7),
            (8.50, 36.0, -314.0, 0.004, 145.8),
            (8.80, 34.7, -317.0, 0.012, 146.0),
            (9.00, 33.5, -319.0, 0.017, 146.2),
            (9.20, 32.4, -319.0, 0.019, 146.4),
            (9.40, 31.4, -320.0, 0.022, 146.6),
            (9.60, 30.7, -319.0, 0.022, 146.8),
            (9.80, 30.0, -319.0, 0.020, 147.0),
            (10.00, 29.1, -318.0, 0.022, 147.1),
            (10.30, 28.2, -317.0, 0.026, 147.3),
            (10.50, 27.4, -317.0, 0.027, 147.3),
            (10.70, 26.5, -316.0, 0.031, 147.4),
            (10.90, 25.7, -315.0, 0.033, 147.4),
            (11.10, 24.9, -314.0, 0.037, 147.4),
            (11.30, 24.2, -313.0, 0.039, 147.4),
            (11.60, 23.1, -311.0, 0.048, 147.5),
            (11.80, 22.2, -308.0, 0.050, 147.6),
            (12.00, 21.1, -306.0, 0.054, 147.6),
            (12.20, 20.1, -302.0, 0.061, 147.7),
            (12.40, 19.1, -299.0, 0.062, 147.8),
            (12.60, 18.2, -297.0, 0.066, 147.8),
            (12.90, 17.1, -295.0, 0.073, 147.8),
            (13.10, 16.1, -294.0, 0.068, 147.9),
            (13.30, 15.0, -295.0, 0.064, 147.8),
            (13.50, 13.7, -299.0, 0.065, 147.8),
            (13.70, 12.6, -303.0, 0.066, 147.8),
            (13.90, 11.4, -309.0, 0.070, 147.8),
            (14.10, 10.1, -317.0, 0.074, 147.8),
            (14.30, 8.8, -327.0, 0.081, 147.7),
            (14.50, 7.6, -337.0, 0.086, 147.7),
            (14.80, 6.1, -352.0, 0.095, 147.6),
            (14.90, 4.6, -366.0, 0.103, 147.5),
            (15.10, 3.1, -383.0, 0.111, 147.4),
        ];
        let g = GATES[LEMG_12];
        let mut p = Plant::at(
            Gate {
                trimmed_sink_fpm: -468.0,
                ..g
            },
            PLANTS[LEMG_12],
        );
        let dt = 0.01;
        let (mut sq, mut i) = (0.0, 0);
        let mut at_20ft = None;
        while i < REC.len() {
            if p.t + 1e-9 >= REC[i].0 {
                sq += (p.vvi - REC[i].2).powi(2);
                if at_20ft.is_none() && REC[i].1 <= 20.5 {
                    at_20ft = Some(p.vvi);
                }
                p.write(REC[i].3);
                i += 1;
            }
            let h = REC[i.saturating_sub(1)].1;
            let ias = REC[i.saturating_sub(1)].4;
            p.step(g.throttle, h, Some(ias), dt);
        }
        let rms = (sq / REC.len() as f64).sqrt();
        assert!(
            rms < 35.0,
            "the replay follows the record: {rms:.0} fpm rms"
        );
        let at_20ft = at_20ft.unwrap();
        assert!(
            (at_20ft + 302.0).abs() < 50.0,
            "the plateau at 20 ft (-302): {at_20ft:.0}"
        );
        assert!(
            p.vvi < -350.0,
            "and the sink grows over the last feet as it did (-383): {:.0}",
            p.vvi
        );
    }

    /// The first model law (2,600 fpm, two 1 s lags, 1.8 s, the exponential
    /// target 3.5 s ahead), as flown at LEMG 12, kept here to prove the
    /// plant: flown from LEMG 12's gate through LEMG 12's plant it lands
    /// where the aircraft did.
    struct FirstModel {
        t: f64,
        hist: VecDeque<(f64, f64)>,
        b1: f64,
        b2: f64,
        el: f64,
        r1: Option<f64>,
        r2: f64,
        u: Option<f64>,
    }
    impl FirstModel {
        fn new() -> Self {
            FirstModel {
                t: 0.0,
                hist: VecDeque::new(),
                b1: 0.0,
                b2: 0.0,
                el: 0.0,
                r1: None,
                r2: 0.0,
                u: None,
            }
        }
        fn step(&mut self, h: f64, gs: f64, vvi: f64, dt: f64) -> f64 {
            let (k, lag, dead, lead, lam, obs, tau) = (2600.0, 1.0, 1.8, 3.5, 0.6, 0.3, 5.0);
            let n = (dt / 0.02).round().max(1.0) as usize;
            let step = dt / n as f64;
            for _ in 0..n {
                self.t += step;
                self.hist.push_back((self.t, self.el));
                while self.hist.len() > 1 && self.hist[1].0 <= self.t - dead {
                    self.hist.pop_front();
                }
                let d = match self.hist.front() {
                    Some(&(t0, e)) if t0 <= self.t - dead => e,
                    _ => 0.0,
                };
                self.b1 += (k * d - self.b1) * step / lag;
                self.b2 += (self.b1 - self.b2) * step / lag;
            }
            let now = vvi - self.b2;
            let u = self.u.map_or(now, |u| u + (now - u) * (dt / obs).min(1.0));
            self.u = Some(u);
            let nominal = -(gs / 60.0) * super::gp_ft_per_nm();
            let (_, flare_vs) = arrest_targets_fpm(nominal);
            let ahead = (h + vvi.min(0.0) * lead / 60.0).max(0.0);
            let target = flare_target_with(ahead, flare_vs, nominal, Some(tau));
            let r1 = match self.r1 {
                Some(r) => r,
                None => {
                    self.r2 = vvi;
                    vvi
                }
            };
            let d1 = (r1 - self.r2) / lam;
            let d2 = ((target - r1) / lam - d1) / lam;
            let a = (dt / lam).min(1.0);
            self.r1 = Some(r1 + (target - r1) * a);
            self.r2 += (r1 - self.r2) * a;
            let want = self.r2 + 2.0 * lag * d1 + lag * lag * d2;
            self.el = ((want - u) / k).clamp(0.0, 0.45);
            self.el
        }
    }

    enum Law {
        First(FirstModel),
        Model(FlareModel),
        /// The model flare as flown in rough air (the column held from the
        /// flare gate).
        Rough(FlareModel),
    }

    struct Landing {
        sink_fpm: f64,
        float_m: f64,
        climbed: bool,
        max_elev: f64,
        min_elev: f64,
        lever_at_wheels: f64,
    }

    fn profile_737() -> crate::aircraft::AircraftProfile {
        crate::aircraft::load("boeing_737_800").expect("the 737 profile loads")
    }

    fn model() -> FlareModel {
        FlareModel::from_profile(profile_737().flare_elevator).expect("the 737 declares its model")
    }

    /// Fly one recorded gate to the wheels: the loop's 0.22 s tick (LEMG
    /// 12's), the flare gate from the loop's own function, the profile's
    /// `flare_power` below it (the model's) or LEMG 12's height-only retard
    /// (the first law's), the elevator reaching the aircraft a dead time
    /// after it is written, the speed on the fitted speed model.
    fn fly(g: Gate, fit: Fit, law: Law) -> Landing {
        let power = if matches!(law, Law::First(_)) {
            Retard::Sink(SinkKeyed::height_only())
        } else {
            Retard::profile()
        };
        fly_with(g, fit, law, power)
    }

    fn fly_with(g: Gate, fit: Fit, mut law: Law, mut power: Retard) -> Landing {
        let mut p = Plant::at(g, fit);
        let (tick, dt) = (0.22, 0.01);
        let mut h = g.radio_ft;
        let mut lever = g.throttle;
        let mut next = 0.0;
        let mut out = Landing {
            sink_fpm: 0.0,
            float_m: 0.0,
            climbed: false,
            max_elev: 0.0,
            min_elev: 0.0,
            lever_at_wheels: 0.0,
        };
        while h > 0.0 && p.t < 40.0 {
            if p.t + 1e-9 >= next {
                next += tick;
                let gs = p.ias - 6.0;
                let v = p.vvi;
                let (flare_ft, _) = super::arrest_gates_ft(25.0, gs);
                let e = match &mut law {
                    Law::First(m) => m.step(h, gs, v, tick),
                    Law::Model(m) => m.step(h, gs, v, tick, h <= flare_ft, false).elev,
                    Law::Rough(m) => m.step(h, gs, v, tick, h <= flare_ft, true).elev,
                };
                out.max_elev = out.max_elev.max(e);
                out.min_elev = out.min_elev.min(e);
                p.write(e);
                if h <= flare_ft {
                    if let Some(l) = power.step(lever, v, tick, h, p.ias) {
                        lever = l;
                    }
                }
            }
            p.step(lever, h, None, dt);
            h += p.vvi / 60.0 * dt;
            out.float_m += (p.ias - 6.0) * 0.514_444 * dt;
            out.climbed |= p.vvi > 0.0;
        }
        out.sink_fpm = -p.vvi;
        out.lever_at_wheels = lever;
        out
    }

    /// LEMG 12, closed loop: the first model law from its own gate through
    /// its own flare's plant lands at the 407 the aircraft did. A plant that
    /// landed this soft would be a plant that could not be trusted.
    #[test]
    fn the_first_model_law_lands_lemg_12_where_it_did() {
        let l = fly(
            GATES[LEMG_12],
            PLANTS[LEMG_12],
            Law::First(FirstModel::new()),
        );
        assert!(
            (l.sink_fpm - 407.0).abs() < 30.0,
            "the first model law flown at LEMG 12 lands at 407: {:.0}",
            l.sink_fpm
        );
        // And across the seven flares it was never inside the limit with
        // margin: the plant fails it as the runway did.
        let worst = GATES
            .iter()
            .zip(PLANTS.iter())
            .map(|(g, f)| fly(*g, *f, Law::First(FirstModel::new())).sink_fpm)
            .fold(0.0, f64::max);
        assert!(
            worst > crate::scorecard::TOUCHDOWN_MAX_SINK_FPM,
            "{worst:.0}"
        );
    }

    /// The prediction flare, the profile as declared, from all seven
    /// recorded gates through all seven flares' own plants (49 landings):
    /// every one well under the scorecard's 400, none climbing, the column
    /// never pushed nor past its stop, no float past 1,200 m. LEMG 12's own
    /// gate and plant, the landing flown at 407: under 300.
    #[test]
    fn the_737_model_flare_lands_every_recorded_gate_under_400() {
        let limit = crate::scorecard::TOUCHDOWN_MAX_SINK_FPM;
        for (k, fit) in PLANTS.iter().enumerate() {
            for (i, g) in GATES.iter().enumerate() {
                let l = fly(*g, *fit, Law::Model(model()));
                assert!(
                    l.sink_fpm < limit - 60.0,
                    "gate {i}, plant {k} {fit:?}: {:.0} fpm",
                    l.sink_fpm
                );
                assert!(
                    l.sink_fpm > 100.0,
                    "gate {i}, plant {k}: a landing, not a float: {:.0}",
                    l.sink_fpm
                );
                assert!(!l.climbed, "gate {i}, plant {k}: ballooned");
                assert!(
                    l.min_elev >= 0.0 && l.max_elev <= 0.45 + 1e-9,
                    "gate {i}: {} {}",
                    l.min_elev,
                    l.max_elev
                );
                assert!(
                    l.float_m < 1200.0,
                    "gate {i}, plant {k}: floated {:.0} m",
                    l.float_m
                );
            }
        }
        let l = fly(GATES[LEMG_12], PLANTS[LEMG_12], Law::Model(model()));
        assert!(
            l.sink_fpm < 300.0,
            "LEMG 12 on the new law: {:.0}",
            l.sink_fpm
        );
        // The prediction flare as flown (277 at the wheels, 0.30 of lever):
        // with the height, time and speed keys the lever is coming off at the
        // wheels and the sink it rebuilds is small.
        let l = fly(
            GATES[PREDICTION_FLOWN],
            PLANTS[PREDICTION_FLOWN],
            Law::Model(model()),
        );
        assert!(l.sink_fpm < 300.0, "{:.0}", l.sink_fpm);
        assert!(
            l.lever_at_wheels < 0.30 && l.lever_at_wheels > 0.0,
            "walking off, not cut: {:.2}",
            l.lever_at_wheels
        );
    }

    /// Issue #76: the retard keyed on height, time to the wheels and speed,
    /// never on the sink. Replayed through the fitted 737 flare plant (its
    /// dead time, both lags, the ground and the speed model) and the air
    /// each recording carries: all sixteen LEMG 13 approaches of 2026-09-28
    /// (sorties 1790586843 to 1790590296), and every recorded gate of
    /// 2026-09-26 through every flare's own plant (64 landings).
    ///
    /// As flown, the sink-keyed retard started off when the ARREST brought
    /// the sink inside 185 (15.8 ft in 1790589309, mid-float) and stopped
    /// again when the sink it cost came back past 400: through the plant the
    /// sixteen land at 268 fpm mean, 300 worst. Keyed on what it is for they
    /// land softer on average and at worst, and none climbs.
    #[test]
    fn the_retard_is_keyed_on_height_time_and_speed_never_the_sink() {
        let cfg = profile_737().flare_power.expect("the 737 retards");
        assert_eq!(cfg.retard_below_ft, Some(20.0));
        assert_eq!(cfg.retard_at_s, Some(2.0));
        let (mut old_worst, mut old_sum, mut new_worst, mut new_sum) = (0.0f64, 0.0, 0.0f64, 0.0);
        for s in LEMG_13_SORTIES {
            let r = recording(s);
            let a = r.arrest();
            let rough = r.air().rough();
            let old = fly_as_flown(&a, CALM_FIT, rough, true);
            let new = fly_through_air(&a, CALM_FIT, rough, true);
            assert!(!new.climbed, "{s}: climbed");
            assert!(new.sink_fpm > 150.0, "{s}: a landing, not a float: {:.0}", new.sink_fpm);
            assert!(new.float_m < 1200.0, "{s}: floated {:.0} m", new.float_m);
            old_worst = old_worst.max(old.sink_fpm);
            new_worst = new_worst.max(new.sink_fpm);
            old_sum += old.sink_fpm;
            new_sum += new.sink_fpm;
        }
        let n = LEMG_13_SORTIES.len() as f64;
        assert!(
            (old_sum / n - 268.0).abs() < 10.0 && (old_worst - 300.0).abs() < 10.0,
            "the old retard replays as characterised: {:.0} mean, {old_worst:.0} worst",
            old_sum / n
        );
        assert!(
            new_sum / n < old_sum / n - 15.0 && new_worst < old_worst,
            "{:.0} mean, {new_worst:.0} worst against {:.0}, {old_worst:.0}",
            new_sum / n,
            old_sum / n
        );
        // The 2026-09-26 gates through every plant: under the scorecard's
        // 400 with margin, none climbing, no float past 1,200 m.
        for (k, fit) in PLANTS.iter().enumerate() {
            for (i, g) in GATES.iter().enumerate() {
                let l = fly(*g, *fit, Law::Model(model()));
                assert!(l.sink_fpm < 340.0, "gate {i}, plant {k}: {:.0}", l.sink_fpm);
                assert!(!l.climbed && l.float_m < 1200.0, "gate {i}, plant {k}");
            }
        }
    }

    /// The keys one by one on the 737's numbers: above 20 ft nothing; below
    /// it, nothing while more than 2 s from the wheels unless faster than the approach speed;
    /// then timed to the wheels at no more than 0.04 a second; and a flare
    /// 5 kt slow keeps its thrust. LEMG 12 (2026-09-26) sinking 300 at
    /// 20 ft: 4 s from the wheels, so the lever stays (it went 0.44 to 0.16
    /// by height alone and landed at 407).
    #[test]
    fn the_737_retard_keys() {
        let cfg = profile_737().flare_power.expect("the 737 retards");
        let app = Some(APPROACH_IAS);
        let on = Some(APPROACH_IAS);
        let mut p = FlarePower::default();
        assert_eq!(p.step(cfg, 0.44, -300.0, 0.22, 25.0, on, app), Some(0.44));
        assert_eq!(p.step(cfg, 0.44, -300.0, 0.22, 21.0, on, app), Some(0.44));
        assert_eq!(p.last_key, Some(RetardKey::Height));
        assert_eq!(p.step(cfg, 0.44, -300.0, 0.22, 19.0, on, app), Some(0.44));
        assert_eq!(p.last_key, Some(RetardKey::Time));
        // 10 ft at 300 fpm is 2 s: off it comes, at no more than 0.04 a second.
        let a = p.step(cfg, 0.44, -300.0, 0.22, 10.0, on, app).unwrap();
        assert!(a < 0.44 && a >= 0.44 - 0.04 * 0.22 - 1e-9, "{a}");
        assert_eq!(p.last_key, Some(RetardKey::Timed));
        // Once due it stays due: a float after it does not re-hold the lever.
        let b = p.step(cfg, 0.44, -60.0, 0.22, 9.5, on, app).unwrap();
        assert!(b < a, "{a} -> {b}");
        // Fast: 2 kt over the approach speed starts it at 19 ft.
        let mut f = FlarePower::default();
        f.step(cfg, 0.3, -300.0, 0.22, 19.0, Some(148.0), app);
        assert!(f.step(cfg, 0.3, -300.0, 0.22, 18.0, Some(148.0), app).unwrap() < 0.3);
        // Slow: 6 kt under keeps it, however close the wheels.
        let mut s = FlarePower::default();
        s.step(cfg, 0.3, -300.0, 0.22, 5.0, Some(139.0), app);
        assert_eq!(s.step(cfg, 0.3, -300.0, 0.22, 4.0, Some(139.0), app), Some(0.3));
        assert_eq!(s.last_key, Some(RetardKey::Slow));
    }

    /// Only an airframe that declares a model flies it (the 737, and the
    /// SF50 since LEMG 12, issue #163); a block without one keeps the
    /// tracker (and a bad model is dropped, never the rest of the block).
    #[test]
    fn only_a_declared_model_flies_the_model_flare() {
        assert!(FlareModel::from_profile(profile_737().flare_elevator).is_some());
        let sf50 = crate::aircraft::load("cirrus_vision_sf50").expect("sf50");
        assert!(FlareModel::from_profile(sf50.flare_elevator).is_some());
        let tracker_only = crate::aircraft::FlareElevatorProfile {
            model: None,
            ..sf50.flare_elevator.expect("the SF50's block")
        };
        assert!(FlareModel::from_profile(Some(tracker_only)).is_none());
        assert!(FlareModel::from_profile(None).is_none());
        let mut p = profile_737().flare_elevator.unwrap();
        if let Some(m) = p.model.as_mut() {
            m.dead_s = 9.0;
        }
        let v = p.valid().expect("the block stands");
        assert!(v.model.is_none(), "the bad model is dropped");
        assert_eq!(v.max, 0.45);
    }

    /// A read that fails is no measurement: the model coasts, nothing is
    /// written, and the next tick decides again.
    #[test]
    fn a_missed_read_coasts_the_model() {
        let mut m = model();
        let first = m.step(81.5, 139.0, -774.0, 0.22, false, false);
        assert!(
            first.elev > 0.1,
            "the gate kicks the column: {}",
            first.elev
        );
        m.coast(0.22, 78.0);
        let again = m.step(76.0, 139.0, -760.0, 0.22, false, false);
        assert!(again.elev >= 0.0 && again.elev <= 0.45);
        assert!(again.unexplained_fpm.is_finite() && again.ahead_ft < 76.0);
    }

    // ---- LEMG 13, 2026-09-28: the flare in rough air (issue #146) ----

    /// Three recorded LEMG 13 finals (the autoland ticks from 400 ft to the
    /// wheels, and the flight model's touchdown): 1790589309 (422 fpm) and
    /// 1790589773 (335) in gusts, 1790588609 (236) in calm air.
    const LEMG_13_FLARES: &str = include_str!("../bench/landings/lemg13_gusty_flares.jsonl");
    /// The other thirteen LEMG 13 approaches of that night's build (sorties
    /// 1790586843 to 1790590296, 225 to 315 fpm), the same fields.
    const LEMG_13_CALM_FLARES: &str = include_str!("../bench/landings/lemg13_calm_flares.jsonl");
    /// Every one of the sixteen, in order.
    const LEMG_13_SORTIES: [&str; 16] = [
        "1790586843",
        "1790587073",
        "1790587296",
        "1790587535",
        "1790587771",
        "1790588007",
        "1790588156",
        "1790588305",
        "1790588460",
        "1790588609",
        "1790589309",
        "1790589541",
        "1790589773",
        "1790590005",
        "1790590150",
        "1790590296",
    ];

    /// EGBB 15 (737, EGKK-EGBB full auto, 2026-09-28, sortie 1790601822):
    /// the autoland ticks from 400 ft to the wheels and the flight model's
    /// touchdown (288 fpm). Rough air (41 fpm rms); the column held 0.28
    /// through a float at 22.7 ft, eased to 0.20 at +1 fpm; the lever stayed
    /// 0.30 from the flare to 2.3 ft (retard keys height, time, slow).
    const EGBB_15_FLOAT: &str = include_str!("../bench/landings/egbb15_rough_float.jsonl");

    /// EGBB 15's own flare, fitted open loop to the recorded elevator, lever
    /// and speed from the arrest gate to the wheels (36 fpm rms): a strong
    /// elevator, 6,300 fpm per unit through 1.7 s of dead time, on a trim
    /// that held -1,025. The law's 3,500 is the weakest flare's on purpose,
    /// so the pull it holds in rough air is more than this aircraft needs.
    const EGBB_FIT: Fit = Fit {
        fpm_per_elevator: 6300.0,
        dead_s: 1.7,
    };
    const EGBB_TRIM_FPM: f64 = -1025.0;

    #[test]
    fn the_egbb_plant_is_fitted_to_its_own_flare() {
        let a = recording("1790601822").arrest();
        let mut p = Plant::at(gate_of(&a, Some(EGBB_TRIM_FPM)), EGBB_FIT);
        let mut sq = 0.0;
        for w in a.windows(2) {
            let (t0, t1) = (w[0].t - a[0].t, w[1].t - a[0].t);
            p.write(w[0].elev);
            while p.t + 1e-9 < t1 {
                let f = (p.t - t0) / (t1 - t0);
                p.step(
                    w[0].lever,
                    w[0].radio,
                    Some(w[0].ias + (w[1].ias - w[0].ias) * f),
                    0.01,
                );
            }
            sq += (p.vvi - w[1].vvi).powi(2);
        }
        let rms = (sq / (a.len() - 1) as f64).sqrt();
        assert!(rms < 40.0, "the fit follows the record: {rms:.0} fpm rms");
        // The law that flew is the law in the harness: fed the recorded
        // sinks in rough air, easing on any climb, it writes what was
        // written (the 0.28 hold, the ease to 0.20 at +1 fpm, the cap).
        let mut m = model().easing_on_any_climb();
        let mut worst: f64 = 0.0;
        for (n, k) in a.iter().enumerate() {
            let dt = if n == 0 { 0.22 } else { k.t - a[n - 1].t };
            let e = m.step(k.radio, k.ias - 4.0, k.vvi, dt, k.flare, true).elev;
            worst = worst.max((e - k.elev).abs());
        }
        assert!(worst < 0.04, "elevator {worst:.3} off the record");
    }

    /// Issues #76 and #146, EGBB 15 (sortie 1790601822): the column held
    /// 0.28 in rough air and levelled the 737 at 22.7 ft on 0.30 of lever;
    /// the retard, keyed on height (over 20 ft), then time (a float is 13 s
    /// from the wheels), then speed (the float bled it 7 kt under), never
    /// moved the lever, and the float ran on its thrust. Through the flare's
    /// own fit and the air it carried, as flown: 5.6 s under 100 fpm of sink,
    /// 940 m from the flare gate, the lever at 0.30 at the wheels. With the
    /// float key (a sink under 100 fpm now or 1.5 s ahead on its trend takes
    /// the lever off at the airframe's quickest, at any height or speed) and
    /// a column that eases only for a climb past 25 fpm: no float, 280 m
    /// shorter, the lever off at the wheels, a landing under 350 fpm.
    #[test]
    fn the_egbb_float_takes_the_thrust_off_and_lands_without_it() {
        let r = recording("1790601822");
        assert_eq!(r.touchdown_fpm, 288.0);
        assert!(r.air().rough(), "{:?}", r.air().rms_fpm());
        let a = r.arrest();
        let trim = Some(EGBB_TRIM_FPM);
        let flown = fly_through_air_with(&a, EGBB_FIT, trim, true, true, Retard::no_float(), true);
        assert!(
            flown.float_s > 4.0,
            "as flown it floats: {:.1} s",
            flown.float_s
        );
        assert!(
            (flown.lever_at_wheels - a[0].lever).abs() < 1e-9,
            "and the lever never moves: {:.3}",
            flown.lever_at_wheels
        );
        let f = fly_through_air_with(&a, EGBB_FIT, trim, true, true, Retard::profile(), false);
        assert!(f.float_s < 0.2, "no float: {:.1} s", f.float_s);
        assert!(
            f.float_m < flown.float_m - 200.0,
            "{:.0} m against {:.0}",
            f.float_m,
            flown.float_m
        );
        assert!(
            f.lever_at_wheels < 0.05,
            "the lever off: {:.3}",
            f.lever_at_wheels
        );
        assert!(
            f.sink_fpm > 150.0 && f.sink_fpm < 350.0,
            "a landing: {:.0}",
            f.sink_fpm
        );
        assert!(!f.climbed && f.eased < 1e-9, "never forward, never up");
        // Not one plant's luck: around the fit (elevator 5,700 to 6,900,
        // dead time 1.6 to 1.8 s) the worst landing is softer, the mean float
        // shorter, and the lever always comes off.
        let (mut old_worst, mut new_worst) = (0.0f64, 0.0f64);
        let (mut old_m, mut new_m, mut n) = (0.0, 0.0, 0.0);
        for k in [5700.0, 6300.0, 6900.0] {
            for d in [1.6, 1.7, 1.8] {
                let fit = Fit {
                    fpm_per_elevator: k,
                    dead_s: d,
                };
                let old = fly_through_air_with(&a, fit, trim, true, true, Retard::no_float(), true);
                let new = fly_through_air_with(&a, fit, trim, true, true, Retard::profile(), false);
                old_worst = old_worst.max(old.sink_fpm);
                new_worst = new_worst.max(new.sink_fpm);
                old_m += old.float_m;
                new_m += new.float_m;
                n += 1.0;
                assert!(
                    new.lever_at_wheels < 0.11,
                    "{k} {d}: {:.3}",
                    new.lever_at_wheels
                );
                assert!(new.sink_fpm < 430.0, "{k} {d}: {:.0}", new.sink_fpm);
            }
        }
        assert!(
            new_worst < old_worst - 100.0,
            "worst {new_worst:.0} against {old_worst:.0}"
        );
        assert!(
            new_m < old_m - 50.0 * n,
            "mean float {:.0} m against {:.0}",
            new_m / n,
            old_m / n
        );
    }

    /// The float key on the 737's numbers: a float at 22.7 ft (over the
    /// retard height), 5 kt slow and far from the wheels, walks the lever
    /// off at 0.04 a second; a sink easing fast enough to float within 1.5 s
    /// is one already; slow defers only a retard not yet begun and a sink
    /// still sinking; a +1 fpm float is no balloon for the lever.
    #[test]
    fn a_float_takes_the_thrust_off_at_any_height_and_speed() {
        let cfg = profile_737().flare_power.expect("the 737 retards");
        let app = Some(APPROACH_IAS);
        let slow = Some(APPROACH_IAS - 6.0);
        let mut p = FlarePower::default();
        assert_eq!(p.step(cfg, 0.30, -140.0, 0.25, 22.9, slow, app), Some(0.30));
        let a = p.step(cfg, 0.30, -86.0, 0.25, 22.7, slow, app).unwrap();
        assert_eq!(p.last_key, Some(RetardKey::Float));
        assert!((a - (0.30 - 0.04 * 0.25)).abs() < 1e-9, "{a}");
        // The sink rebuilding under it: the retard goes on, timed, and the
        // speed never holds it again.
        let b = p.step(cfg, 0.30, -250.0, 0.25, 12.0, slow, app).unwrap();
        assert!(b < a, "{a} -> {b}");
        assert_eq!(p.last_key, Some(RetardKey::Timed));
        // At 24.9 ft sinking 300 and easing 140 fpm a second (EGBB, 2.1 s
        // before the float): the float ahead takes the thrust now.
        let mut q = FlarePower::default();
        q.step(cfg, 0.30, -440.0, 0.3, 26.7, app, app);
        q.step(cfg, 0.30, -398.0, 0.3, 25.8, app, app);
        let c = q.step(cfg, 0.30, -300.0, 0.3, 24.9, app, app).unwrap();
        assert_eq!(q.last_key, Some(RetardKey::Float));
        assert!(c < 0.30);
        // Still sinking 300 steadily over 20 ft: the height holds it.
        let mut h = FlarePower::default();
        h.step(cfg, 0.30, -300.0, 0.3, 26.0, app, app);
        assert_eq!(h.step(cfg, 0.30, -300.0, 0.3, 25.0, app, app), Some(0.30));
        assert_eq!(h.last_key, Some(RetardKey::Height));
        // A near-level float is not a balloon: no approach thrust back.
        assert_eq!(super::balloon_power(Some(1.0), Some(0.30), false), None);
        assert_eq!(super::balloon_power(Some(2.0), Some(0.30), false), None);
        assert_eq!(
            super::balloon_power(Some(60.0), Some(0.30), false),
            Some(0.30)
        );
    }

    #[derive(Clone, Copy, Debug)]
    struct Tick {
        t: f64,
        radio: f64,
        vvi: f64,
        trend: f64,
        asked: f64,
        ias: f64,
        elev: f64,
        lever: f64,
        arresting: bool,
        flare: bool,
        final_leg: bool,
        ap_hold: bool,
    }

    struct Recording {
        ticks: Vec<Tick>,
        touchdown_fpm: f64,
    }

    fn recording(sortie: &str) -> Recording {
        let mut ticks = Vec::new();
        let mut touchdown_fpm = f64::NAN;
        for line in LEMG_13_FLARES
            .lines()
            .chain(LEMG_13_CALM_FLARES.lines())
            .chain(EGBB_15_FLOAT.lines())
        {
            let v: serde_json::Value = serde_json::from_str(line).expect("a bench line");
            if !v["sortie"].as_str().is_some_and(|s| s.starts_with(sortie)) {
                continue;
            }
            if v["kind"] == "flight" {
                touchdown_fpm = -v["sink"].as_f64().unwrap();
                continue;
            }
            let f = |k: &str| v[k].as_f64().unwrap_or(0.0);
            ticks.push(Tick {
                t: f("t"),
                radio: f("radio"),
                vvi: f("vvi"),
                trend: f("vvi_trend"),
                asked: f("cmd_vs"),
                ias: f("ias"),
                elev: f("flare_elev"),
                lever: f("lever"),
                arresting: v["arresting"] == true,
                flare: v["phase"] == "Flare",
                final_leg: v["phase"] == "Final",
                ap_hold: v["ap_hold"] == true,
            });
        }
        assert!(!ticks.is_empty() && touchdown_fpm.is_finite(), "{sortie}");
        Recording {
            ticks,
            touchdown_fpm,
        }
    }

    impl Recording {
        /// The final's air as the loop measures it on the way to the gate.
        fn air(&self) -> super::FinalAir {
            let mut air = super::FinalAir::default();
            for k in &self.ticks {
                if k.final_leg && !k.arresting && !k.ap_hold && k.radio <= super::FINAL_AIR_FROM_FT
                {
                    air.note(k.vvi, k.asked);
                }
            }
            air
        }
        fn arrest(&self) -> Vec<Tick> {
            self.ticks.iter().copied().filter(|k| k.arresting).collect()
        }
    }

    /// Today's calm flares through their own fits: 3,700 to 4,450 fpm per
    /// unit, 1.35 to 1.8 s of dead time (ten landings, 9 to 23 fpm rms).
    /// The gusty ones fit nothing sensible (6,450 to 9,150 fpm, 49 to 96
    /// rms): the air is not in the plant.
    const CALM_FIT: Fit = Fit {
        fpm_per_elevator: 4000.0,
        dead_s: 1.55,
    };

    /// The recorded gate; `trim` is a fitted trimmed sink (fpm), else the
    /// gate's own sink (the trim folded into the air).
    fn gate_of(a: &[Tick], trim: Option<f64>) -> Gate {
        Gate {
            radio_ft: a[0].radio,
            vvi_fpm: a[0].vvi,
            trend_fpm_s: a[0].trend,
            ias_kts: a[0].ias,
            throttle: a[0].lever,
            // What the air does is taken from the recording; the trim is
            // folded into it.
            trimmed_sink_fpm: trim.unwrap_or(a[0].vvi),
            flown_fpm: f64::NAN,
        }
    }

    /// The air of a recording through a fit: the recorded elevator, speed
    /// and lever flown open loop, and per tick the sink (fpm, at the tick)
    /// and speed change (kt/s, over the interval after it) the fit does not
    /// explain.
    fn air_of(a: &[Tick], fit: Fit, trim: Option<f64>) -> (Vec<f64>, Vec<f64>) {
        let mut p = Plant::at(gate_of(a, trim), fit);
        let dt = 0.01;
        let (mut air, mut speed) = (vec![0.0], Vec::new());
        for w in a.windows(2) {
            let (t0, t1) = (w[0].t - a[0].t, w[1].t - a[0].t);
            p.write(w[0].elev);
            let (mut acc, mut n) = (0.0f64, 0.0f64);
            while p.t + 1e-9 < t1 {
                acc += speed_model(w[0].lever, p.elev, w[0].vvi);
                n += 1.0;
                let f = (p.t - t0) / (t1 - t0);
                p.step(
                    w[0].lever,
                    w[0].radio,
                    Some(w[0].ias + (w[1].ias - w[0].ias) * f),
                    dt,
                );
            }
            speed.push((w[1].ias - w[0].ias) / (t1 - t0) - acc / n.max(1.0));
            air.push(w[1].vvi - (p.trimmed + p.x2 + FPM_PER_KT * (w[1].ias - p.ias0)));
        }
        (air, speed)
    }

    struct Flown {
        sink_fpm: f64,
        /// Most the column came forward of its flare peak while not
        /// climbing.
        eased: f64,
        climbed: bool,
        float_m: f64,
        lever_at_wheels: f64,
        ias_at_wheels: f64,
        /// Seconds of the flare sinking under [`super::RETARD_FLOAT_FPM`].
        float_s: f64,
    }

    /// Fly a recorded gate through a fit and the recorded air, the law
    /// ticking at the recorded tick times (0.22 s past them), the flare from
    /// the loop's own gate, the profile's retard below it and the approach
    /// lever held in a climb, the elevator arriving a dead time after it is
    /// written through two lags.
    fn fly_through_air(a: &[Tick], fit: Fit, rough: bool, with_air: bool) -> Flown {
        fly_through_air_with(a, fit, None, rough, with_air, Retard::profile(), false)
    }

    /// ...with the retard as it flew on 2026-09-28 (the sink-keyed law) and
    /// the column and lever taking any climb for a balloon.
    fn fly_as_flown(a: &[Tick], fit: Fit, rough: bool, with_air: bool) -> Flown {
        fly_through_air_with(
            a,
            fit,
            None,
            rough,
            with_air,
            Retard::Sink(SinkKeyed::as_flown()),
            true,
        )
    }

    /// `any_climb`: the column eases and the lever takes the approach
    /// thrust back on any climb, +1 fpm included (as flown before issue
    /// #146); else only past [`super::BALLOON_CLIMB_FPM`].
    fn fly_through_air_with(
        a: &[Tick],
        fit: Fit,
        trim: Option<f64>,
        rough: bool,
        with_air: bool,
        mut power: Retard,
        any_climb: bool,
    ) -> Flown {
        let climb_fpm = if any_climb {
            0.0
        } else {
            super::BALLOON_CLIMB_FPM
        };
        let (air, speed) = if with_air {
            air_of(a, fit, trim)
        } else {
            (vec![0.0], Vec::new())
        };
        let times: Vec<f64> = a.iter().map(|k| k.t - a[0].t).collect();
        let mut law = if any_climb {
            model().easing_on_any_climb()
        } else {
            model()
        };
        let mut p = Plant::at(gate_of(a, trim), fit);
        let (mut h, approach_lever) = (a[0].radio, a[0].lever);
        let mut lever = approach_lever;
        let (dt, mut i, mut next) = (0.01, 0usize, 0.0);
        let mut prev_t: Option<f64> = None;
        let (mut peak, mut flaring) = (0.0f64, false);
        let mut out = Flown {
            sink_fpm: 0.0,
            eased: 0.0,
            climbed: false,
            float_m: 0.0,
            lever_at_wheels: 0.0,
            ias_at_wheels: 0.0,
            float_s: 0.0,
        };
        while h > 0.0 && p.t < 40.0 {
            if p.t + 1e-9 >= next {
                let tick = times.get(i + 1).map_or(0.22, |t1| t1 - times[i]);
                let law_dt = prev_t.map_or(0.22, |t0| p.t - t0);
                prev_t = Some(p.t);
                let v = p.vvi;
                let gs = p.ias - 4.0;
                let (flare_ft, _) = super::arrest_gates_ft(25.0, gs);
                let flare = h <= flare_ft;
                let e = law.step(h, gs, v, law_dt, flare, rough).elev;
                p.write(e);
                if flare {
                    if !flaring {
                        peak = e;
                    }
                    flaring = true;
                    peak = peak.max(e);
                    if v <= climb_fpm {
                        out.eased = out.eased.max(peak - e);
                    }
                    lever = if v > climb_fpm {
                        approach_lever
                    } else {
                        power.step(lever, v, law_dt, h, p.ias).unwrap_or(lever)
                    };
                }
                p.air_kts_s = if with_air {
                    speed.get(i).copied().unwrap_or(0.0)
                } else {
                    0.0
                };
                i += 1;
                next += tick;
            }
            if with_air {
                let j = i - 1;
                p.air_fpm = match (air.get(j), air.get(j + 1), times.get(j + 1)) {
                    (Some(a0), Some(a1), Some(t1)) => {
                        let f = ((p.t - times[j]) / (t1 - times[j])).clamp(0.0, 1.0);
                        a0 + (a1 - a0) * f
                    }
                    _ => *air.last().unwrap(),
                };
            }
            p.step(lever, h, None, dt);
            h += p.vvi / 60.0 * dt;
            if flaring {
                out.float_m += (p.ias - 4.0) * 0.514_444 * dt;
                out.climbed |= p.vvi > 0.0;
                if p.vvi > -super::RETARD_FLOAT_FPM {
                    out.float_s += dt;
                }
            }
        }
        out.sink_fpm = -p.vvi;
        out.lever_at_wheels = lever;
        out.ias_at_wheels = p.ias;
        out
    }

    /// Every autoland tick's bug (true) from four LEMG 13 approaches of
    /// 2026-09-28, the arm to the wheels (two 5 nm finals, one with an
    /// intercept, and a full-auto circuit's final).
    const LEMG_13_FINAL_KNOBS: &str = include_str!("../bench/landings/lemg13_final_knobs.jsonl");

    /// Issue #93: those approaches' bugs replayed through the knob autoland
    /// now sets (`bug_knob`): the knob moved (0.2 degrees or more, the least
    /// `drive_heading_bug_step` writes) at most half as often outside the
    /// short final and the flare (which still steer every tick), and always
    /// within a turn's steering of the law's own bug: 21 moves against 76.
    #[test]
    fn the_recorded_finals_move_the_knob_less() {
        let moved = |a: Option<f64>, b: f64| {
            a.is_none_or(|a| (((b - a) + 540.0).rem_euclid(360.0) - 180.0).abs() >= 0.2)
        };
        let mut by_sortie: Vec<(String, Vec<serde_json::Value>)> = Vec::new();
        for line in LEMG_13_FINAL_KNOBS.lines() {
            let v: serde_json::Value = serde_json::from_str(line).expect("a bench line");
            let s = v["sortie"].as_str().unwrap().to_string();
            match by_sortie.last_mut() {
                Some((id, ticks)) if *id == s => ticks.push(v),
                _ => by_sortie.push((s, vec![v])),
            }
        }
        assert_eq!(by_sortie.len(), 4);
        let (mut free_all, mut held_all) = (0u32, 0u32);
        for (s, ticks) in &by_sortie {
            let mut hold = crate::circuit::BugHold::default();
            let (mut free, mut held) = (0u32, 0u32);
            let (mut free_at, mut held_at) = (None::<f64>, None::<f64>);
            let t0 = ticks[0]["t"].as_f64().unwrap();
            for k in ticks {
                let b = k["bug_true"].as_f64().unwrap();
                let phase = match k["phase"].as_str() {
                    Some("Flare") => super::ApproachPhase::Flare,
                    Some("Final") => super::ApproachPhase::Final,
                    _ => super::ApproachPhase::Intercept,
                };
                let short = k["short_final"] == true || k["arresting"] == true;
                let leg = match k["leg"].as_str() {
                    Some("outbound") => Some(crate::reversal::Leg::Outbound),
                    Some("base") => Some(crate::reversal::Leg::Base),
                    Some("inbound") => Some(crate::reversal::Leg::Inbound),
                    _ => None,
                };
                let knob = match super::bug_knob(phase, short, leg, None) {
                    Some((key, db)) => {
                        hold.decide(key, b, k["t"].as_f64().unwrap() - t0, db);
                        let h = hold.set().unwrap_or(b);
                        let off = (((h - b) + 540.0).rem_euclid(360.0) - 180.0).abs();
                        assert!(
                            off < crate::circuit::BUG_STEER_DEG,
                            "{s}: the knob {h:.1} against the law's {b:.1}"
                        );
                        h
                    }
                    None => {
                        hold.reset();
                        free_at = None;
                        held_at = None;
                        continue;
                    }
                };
                if moved(free_at, b) {
                    free += 1;
                    free_at = Some(b);
                }
                if moved(held_at, knob) {
                    held += 1;
                    held_at = Some(knob);
                }
            }
            assert!(held * 2 <= free, "{s}: {held} moves against {free}");
            free_all += free;
            held_all += held;
        }
        assert!(held_all * 3 <= free_all, "{held_all} moves against {free_all}");
    }

    /// The loop's measure of the final's air tells the three apart: the
    /// gusty finals strayed 72 and 105 fpm rms from the sink asked for, the
    /// calm one 19.
    #[test]
    fn the_final_says_whether_the_air_is_rough() {
        for (sortie, rough) in [
            ("1790589309", true),
            ("1790589773", true),
            ("1790588609", false),
        ] {
            let air = recording(sortie).air();
            let rms = air.rms_fpm().expect("a whole final measured");
            assert_eq!(air.rough(), rough, "{sortie}: {rms:.0} fpm rms");
        }
        assert!(recording("1790588609").air().rms_fpm().unwrap() < 25.0);
        assert!(recording("1790589309").air().rms_fpm().unwrap() > 60.0);
        // Too little of a final is no measurement, and no measurement is
        // smooth air.
        let mut short = super::FinalAir::default();
        for _ in 0..5 {
            short.note(-700.0, -800.0);
            short.note(-900.0, -800.0);
        }
        assert!(short.rms_fpm().is_none() && !short.rough());
        short.note(f64::NAN, -800.0);
        assert!(short.rms_fpm().is_none(), "a failed read is no sample");
    }

    /// The harness is the law that flew: today's law, fed the recorded
    /// sinks, writes the elevator that was written (the float hold changes
    /// nothing there, the model was already pulling), and the fit flown
    /// open loop through the recorded air gives back the recorded sink.
    #[test]
    fn the_recorded_flares_replay_through_todays_law() {
        for sortie in ["1790589309", "1790589773", "1790588609"] {
            let a = recording(sortie).arrest();
            let mut m = model();
            let mut worst: f64 = 0.0;
            for (n, k) in a.iter().enumerate() {
                let dt = if n == 0 { 0.22 } else { k.t - a[n - 1].t };
                let e = m.step(k.radio, k.ias - 4.0, k.vvi, dt, k.flare, false).elev;
                worst = worst.max((e - k.elev).abs());
            }
            assert!(worst < 0.07, "{sortie}: elevator {worst:.3} off the record");
        }
    }

    /// LEMG 13, 1790589309 (issue #146): through today's calm fit and the
    /// air the recording carries, the flare as flown (smooth, the prediction
    /// free) eases the column a fifth of its travel from 25 ft while still
    /// sinking, floats, and arrives near the 422 the aircraft did. Flown for
    /// the rough air the final measured, the column holds from the flare
    /// gate and the same air lands under 300 without a climb.
    #[test]
    fn a_gusty_flare_holds_the_column_through_the_float() {
        let r = recording("1790589309");
        assert_eq!(r.touchdown_fpm, 422.0);
        assert!(r.air().rough());
        let a = r.arrest();
        let flown = fly_as_flown(&a, CALM_FIT, false, true);
        assert!(
            (flown.sink_fpm - r.touchdown_fpm).abs() < 40.0,
            "the replay lands where the aircraft did: {:.0}",
            flown.sink_fpm
        );
        assert!(flown.eased > 0.2, "and eases as it did: {:.2}", flown.eased);
        let held = fly_through_air(&a, CALM_FIT, true, true);
        assert!(held.sink_fpm < 300.0, "{:.0}", held.sink_fpm);
        assert!(
            held.sink_fpm > 150.0,
            "a landing, not a float: {:.0}",
            held.sink_fpm
        );
        assert!(held.eased < 1e-9, "never forward: {:.3}", held.eased);
        assert!(!held.climbed, "no balloon");
        assert!(held.float_m < 1200.0, "floated {:.0} m", held.float_m);
        // 1790589773 (335 fpm, the same gusts): 380 as flown here, under
        // 320 held.
        let a = recording("1790589773").arrest();
        let flown = fly_as_flown(&a, CALM_FIT, false, true);
        let held = fly_through_air(&a, CALM_FIT, true, true);
        assert!(flown.sink_fpm > 350.0, "{:.0}", flown.sink_fpm);
        assert!(
            held.sink_fpm < 320.0 && !held.climbed,
            "{:.0}",
            held.sink_fpm
        );
        // Not one plant's luck: through elevators from today's weakest to
        // past the 2026-09-26 middle, and dead times 1.35 to 1.8 s, both
        // gusty flares held land under 340 and never climb.
        for sortie in ["1790589309", "1790589773"] {
            let a = recording(sortie).arrest();
            for k in [3700.0, 4400.0, 5100.0] {
                for d in [1.35, 1.55, 1.8] {
                    let fit = Fit {
                        fpm_per_elevator: k,
                        dead_s: d,
                    };
                    let held = fly_through_air(&a, fit, true, true);
                    assert!(
                        held.sink_fpm < 340.0 && !held.climbed,
                        "{sortie} {k} {d}: {:.0}",
                        held.sink_fpm
                    );
                }
            }
        }
    }

    /// The calm final keeps the prediction: 1790588609 through its air is
    /// smooth, and flown smooth lands where it did (236).
    #[test]
    fn a_calm_flare_is_flown_as_before() {
        let r = recording("1790588609");
        assert!(!r.air().rough());
        let a = r.arrest();
        let flown = fly_as_flown(&a, CALM_FIT, false, true);
        assert!(
            (flown.sink_fpm - r.touchdown_fpm).abs() < 30.0,
            "{:.0} against {:.0}",
            flown.sink_fpm,
            r.touchdown_fpm
        );
        assert!(flown.sink_fpm > 200.0 && flown.sink_fpm < 290.0);
    }

    /// Why smooth air keeps the prediction: the 17:25 flare of 2026-09-26
    /// answered 6,429 fpm per unit, and in still air the column held from
    /// the flare gate balloons it and drops it in, where the prediction
    /// eases and lands. The gust and a strong elevator look alike for the
    /// seconds that matter; only the air tells them apart.
    #[test]
    fn a_strong_elevator_in_still_air_keeps_the_prediction() {
        let strong = PLANTS[3];
        let mut worst_held: f64 = 0.0;
        let mut climbed = false;
        for g in GATES.iter() {
            let free = fly(*g, strong, Law::Model(model()));
            assert!(
                free.sink_fpm < 340.0 && !free.climbed,
                "{:.0}",
                free.sink_fpm
            );
            let held = fly(*g, strong, Law::Rough(model()));
            worst_held = worst_held.max(held.sink_fpm);
            climbed |= held.climbed;
        }
        assert!(
            climbed && worst_held > 400.0,
            "held in still air it balloons: {worst_held:.0}"
        );
    }

    /// A near-level float never takes the column forward, smooth air or
    /// rough; a climb eases it at the airframe's rate, never a push; the
    /// roundout above the flare stays free.
    #[test]
    fn a_float_keeps_its_pull_and_only_a_climb_eases_it() {
        let mut m = model();
        // Into the flare sinking hard: the prediction pulls.
        let mut pulled: f64 = 0.0;
        for n in 0..6 {
            let h = 30.0 - n as f64 * 1.5;
            pulled = m.step(h, 140.0, -420.0, 0.22, true, false).elev;
        }
        assert!(pulled > 0.1, "{pulled}");
        // Floating at -70 with that pull in the pipe: the prediction wants
        // it forward, the column stays.
        let mut e = pulled;
        for _ in 0..8 {
            let o = m.step(16.0, 140.0, -70.0, 0.22, true, false);
            assert!(o.elev >= e - 1e-9, "forward in a float: {e} -> {}", o.elev);
            assert!(o.ask <= o.elev + 1e-9);
            e = o.elev;
        }
        // Climbing: eased at the profile's 0.15 a second, never below zero.
        let o = m.step(16.5, 140.0, 60.0, 0.22, true, false);
        assert!(
            o.elev < e && o.elev >= e - 0.15 * 0.22 - 1e-9,
            "{e} -> {}",
            o.elev
        );
        assert!(o.elev >= 0.0);
        // Above the flare the model is free both ways again.
        let mut free = model();
        let hi = free.step(60.0, 140.0, -900.0, 0.22, false, true).elev;
        let lo = free.step(58.0, 140.0, -300.0, 0.22, false, true).elev;
        assert!(lo < hi, "the roundout eases: {hi} -> {lo}");
    }
}

#[cfg(test)]
mod ap_engage_tests {
    use super::*;

    /// An arm over an autopilot already flying (the circuit's, full auto's):
    /// confirmed on the second live read, never held, never re-pressed.
    #[test]
    fn an_engaged_autopilot_is_confirmed_and_never_held() {
        let mut e = ApEngage::default();
        assert!(e.arm(0.0).fire);
        let s = e.step(0.6, Some(1.0), 150.0);
        assert!(!s.fire && !s.hold && !s.confirmed_now);
        let s = e.step(1.2, Some(1.0), 150.0);
        assert!(s.confirmed_now && !s.hold);
        assert_eq!(e.tries(), 1);
        assert_eq!(
            e.step(1.8, Some(0.0), 150.0),
            EngageStep::default(),
            "confirmed is sticky"
        );
    }

    /// The lost press: re-pressed every tick while the servos read off,
    /// held level meanwhile, confirmed only on live reads; a read on while
    /// the flight is still starting (IAS 5) is not a confirmation, and the
    /// flight start dropping it is pressed again.
    #[test]
    fn a_lost_press_is_pressed_again_until_the_servos_read_on() {
        let mut e = ApEngage::default();
        e.arm(0.0);
        let s = e.step(0.05, Some(0.0), 12.0);
        assert!(!s.fire && s.hold, "too soon to press again, but held");
        let s = e.step(0.65, Some(0.0), 85.0);
        assert!(s.fire && s.hold && s.try_n == 2);
        let s = e.step(1.25, Some(1.0), 5.0);
        assert!(
            !s.fire && !s.hold && !s.confirmed_now,
            "not live: no confirmation"
        );
        let s = e.step(1.85, Some(0.0), 132.0);
        assert!(s.fire && s.hold && s.try_n == 3, "the start dropped it");
        let s = e.step(2.45, None, 150.0);
        assert!(
            !s.confirmed_now && s.hold && s.fire && s.try_n == 4,
            "a failed read is no measurement: still pressed, still held"
        );
        let s = e.step(3.05, Some(1.0), 160.0);
        assert!(!s.confirmed_now && !s.fire && !s.hold);
        let s = e.step(3.65, None, 160.0);
        assert!(!s.confirmed_now, "a failed read does not confirm");
        let s = e.step(4.25, Some(1.0), 160.0);
        assert!(s.confirmed_now && s.try_n == 4);
    }

    /// An autopilot that never comes in: said and handed over after
    /// ENGAGE_GIVE_UP_S of live flight, never before, never while starting.
    #[test]
    fn an_autopilot_that_never_engages_is_handed_over() {
        let mut e = ApEngage::default();
        e.arm(0.0);
        let mut t = 0.6;
        while t < 5.0 {
            assert!(!e.step(t, Some(0.0), 0.0).give_up, "not live yet at {t}");
            t += 0.6;
        }
        let live_from = t;
        let mut gave = None;
        while t < 30.0 {
            if e.step(t, Some(0.0), 170.0).give_up {
                gave = Some(t);
                break;
            }
            t += 0.6;
        }
        let gave = gave.expect("gave up");
        assert!(
            gave - live_from >= ENGAGE_GIVE_UP_S - 1e-9
                && gave - live_from < ENGAGE_GIVE_UP_S + 0.7,
            "{gave} from {live_from}"
        );
        assert!(
            !ENGAGE_GIVE_UP_LINE.contains('\u{2014}') && !ENGAGE_GIVE_UP_LINE.contains('\u{2013}')
        );
    }

    /// The 737's pitch plant, fitted to the seven LEMG flares of 2026-09-26
    /// (profiles/boeing_737_800.yaml `flare_elevator.model`: 4,970 fpm per
    /// unit of elevator through lags of 0.84 and 3.1 s after 1.35 s of dead
    /// time), with the new-flight start of LEMG 13 (sortie
    /// 1790587073-approach, 2026-09-28) as its disturbance: the sim starts
    /// the flight at 1.3 s descending 900 fpm with 0.54 of nose-down
    /// mistrim, read through a 1 s VSI (fitted to the recorded ticks, 112
    /// fpm rms). The servos: the flight start drops any engage made before
    /// it; a press after it lands at the next frame.
    struct NewFlight {
        t: f64,
        hist: std::collections::VecDeque<f64>,
        x1: f64,
        x2: f64,
        vsi: f64,
        alt_ft: f64,
        servos: bool,
        pressed: bool,
    }

    const DT: f64 = 0.05;
    const START_S: f64 = 1.3;
    const START_FPM: f64 = -900.0;
    const MISTRIM: f64 = -0.54;
    const FPM_PER_ELEV: f64 = 4970.0;
    const DEAD_S: f64 = 1.35;
    const ALT0_FT: f64 = 2318.2;

    impl NewFlight {
        fn new() -> Self {
            let n = (DEAD_S / DT).round() as usize;
            Self {
                t: 0.0,
                hist: std::iter::repeat_n(0.0, n).collect(),
                x1: 0.0,
                x2: 0.0,
                vsi: 0.0,
                alt_ft: ALT0_FT,
                servos: false,
                pressed: false,
            }
        }
        fn press(&mut self) {
            self.pressed = true;
        }
        /// Fly to `until` with `elev` on the channel (the loop's
        /// sample-and-hold: one write per tick).
        fn run(&mut self, until: f64, elev: f64) {
            while self.t < until - 1e-9 {
                let live = self.t >= START_S;
                if !live {
                    // The flight start wipes whatever was engaged before it.
                    self.servos = false;
                    self.pressed = false;
                } else if self.pressed {
                    self.servos = true;
                    self.pressed = false;
                }
                let e_in = if live { MISTRIM + elev } else { 0.0 };
                self.hist.push_back(e_in);
                let e = self.hist.pop_front().unwrap_or(0.0);
                self.x1 += (FPM_PER_ELEV * e - self.x1) * DT / 0.84;
                self.x2 += (self.x1 - self.x2) * DT / 3.1;
                let truth = if live { START_FPM + self.x2 } else { 0.0 };
                self.vsi += (truth - self.vsi) * DT / 1.0;
                self.alt_ft += truth / 60.0 * DT;
                self.t += DT;
            }
        }
    }

    /// The autoland ticks of 1790587073 (seconds from the setup's result,
    /// IAS kt, the VSI read fpm) until the AP came in at +10.9 s.
    const TICKS: [(f64, f64, f64); 18] = [
        (0.9, 12.4, -5.0),
        (1.5, 85.3, -207.0),
        (2.1, 132.1, -533.0),
        (2.7, 153.8, -819.0),
        (3.3, 162.4, -1017.0),
        (3.9, 165.9, -1167.0),
        (4.5, 167.7, -1311.0),
        (5.1, 168.9, -1502.0),
        (5.7, 169.6, -1720.0),
        (6.3, 170.2, -1965.0),
        (6.9, 170.9, -2208.0),
        (7.5, 171.8, -2428.0),
        (8.1, 173.1, -2642.0),
        (8.7, 174.4, -2836.0),
        (9.3, 175.6, -3027.0),
        (9.9, 176.9, -3193.0),
        (10.5, 178.1, -3337.0),
        (11.0, 179.4, -3450.0),
    ];
    /// The arm's one press, just before the first tick (threshold_elev +0.8).
    const ARM_S: f64 = 0.85;

    /// The old arm: one press, never read back. Nobody flies until the
    /// watchdog's re-press at +9.5 s. The plant reproduces the tape.
    #[test]
    fn the_old_arm_lets_the_new_flight_sink_until_the_watchdog() {
        let mut p = NewFlight::new();
        p.run(ARM_S, 0.0);
        p.press();
        let mut sq = 0.0;
        for &(t, _, vvi) in &TICKS {
            p.run(t, 0.0);
            assert!(!p.servos, "the arm's press was lost at {t}");
            sq += (p.vsi - vvi).powi(2);
        }
        let rms = (sq / TICKS.len() as f64).sqrt();
        assert!(
            rms < 150.0,
            "the plant is the recorded flight: {rms:.0} fpm rms"
        );
        assert!(
            p.vsi < -3000.0,
            "the sink handed to the late AP: {:.0}",
            p.vsi
        );
        assert!(
            ALT0_FT - p.alt_ft > 250.0,
            "lost {:.0} ft before anyone flew",
            ALT0_FT - p.alt_ft
        );
    }

    /// The new arm on the same flight: pressed until the servos READ on,
    /// and held level on the elevator until then. The AP is in within two
    /// seconds of the flight starting and inherits a sink it can hold.
    #[test]
    fn a_setup_approach_is_flown_from_the_first_tick() {
        let mut p = NewFlight::new();
        p.run(ARM_S, 0.0);
        let mut e = ApEngage::default();
        assert!(e.arm(0.0).fire);
        p.press();
        let mut law = FlarePitch::with_gains(engage_hold_gains());
        let mut prev: Option<(f64, f64)> = None;
        let mut elev = 0.0;
        let mut confirmed_at = None;
        let mut held_ticks = 0;
        for &(t, ias, _) in &TICKS {
            p.run(t, elev);
            let servos = Some(if p.servos { 1.0 } else { 0.0 });
            let s = e.step(t - ARM_S, servos, ias);
            if s.fire {
                p.press();
            }
            if s.confirmed_now {
                confirmed_at = Some((t, p.vsi, p.alt_ft));
                break;
            }
            let trend = prev.map_or(0.0, |(v, t0)| (p.vsi - v) / (t - t0));
            let dt = prev.map_or(0.6, |(_, t0)| t - t0);
            prev = Some((p.vsi, t));
            elev = if s.hold {
                held_ticks += 1;
                law.step(0.0, p.vsi, trend, dt, false)
            } else {
                0.0
            };
        }
        let (t, vsi, alt) = confirmed_at.expect("the autopilot is confirmed");
        assert!(t <= 3.5, "confirmed at +{t} s, the old arm at +10.9");
        assert!(held_ticks >= 1, "held from the first tick");
        assert!(vsi > -1000.0, "the AP inherits {vsi:.0} fpm, not -3,200");
        assert!(
            ALT0_FT - alt < 60.0,
            "lost {:.0} ft before the AP was in",
            ALT0_FT - alt
        );
    }

    /// The hold alone, with an autopilot that never comes in: the new
    /// flight's sink is arrested on the elevator and the approach handed
    /// over at the bound with the aircraft level, not at -3,450 fpm.
    #[test]
    fn with_no_autopilot_the_hold_keeps_the_new_flight_level_until_the_hand_over() {
        let mut p = NewFlight::new();
        p.run(ARM_S, 0.0);
        let mut e = ApEngage::default();
        e.arm(0.0);
        let mut law = FlarePitch::with_gains(engage_hold_gains());
        let mut prev: Option<(f64, f64)> = None;
        let mut elev = 0.0;
        let mut worst_vsi: f64 = 0.0;
        let mut lowest = p.alt_ft;
        let mut t = 0.9;
        let mut gave = false;
        while t < 30.0 {
            p.run(t, elev);
            lowest = lowest.min(p.alt_ft);
            if t > 4.0 {
                worst_vsi = worst_vsi.min(p.vsi);
            }
            let ias = if t < 1.5 { 12.0 } else { 170.0 };
            let s = e.step(t - ARM_S, Some(0.0), ias);
            if s.give_up {
                gave = true;
                break;
            }
            let trend = prev.map_or(0.0, |(v, t0)| (p.vsi - v) / (t - t0));
            let dt = prev.map_or(0.6, |(_, t0)| t - t0);
            prev = Some((p.vsi, t));
            assert!(s.hold);
            elev = law.step(0.0, p.vsi, trend, dt, false);
            assert!(elev <= engage_hold_gains().max + 1e-9);
            t += 0.6;
        }
        assert!(gave, "handed over");
        // The flight starts at -900 and the elevator arrives 1.35 s late:
        // the peak is that start, never the -3,450 nobody arrested.
        assert!(worst_vsi > -1200.0, "the sink never builds: {worst_vsi:.0}");
        assert!(p.vsi.abs() < 400.0, "level at the hand-over: {:.0}", p.vsi);
        assert!(ALT0_FT - lowest < 150.0, "lost {:.0} ft", ALT0_FT - lowest);
    }
}

/// LEMG 13 and 31 as apt.dat has them, a 737's heading response and the
/// autoland loop's lateral composition, for the replays of the 2026-09-28
/// circuits (issue #147): the circuit's final, the handoff and autoland's
/// final flown against the same plant.
#[cfg(test)]
pub(crate) mod lemg_replay {
    use super::*;

    /// LEMG's 13/31 row from the installed apt.dat (Global Airports), 45.11 m
    /// wide.
    pub const LEMG_13_31: &str = "100 45.11 1 1 0.00 1 3 0  13 36.6845573 -4.5126253 0 75 3 1 0 2 31 36.6654027 -4.4858140 0 75 3 1 0 3";
    pub const LEMG_WIDTH_M: f64 = 45.11;
    /// ...and its 12/30 row, the same width.
    pub const LEMG_12_30: &str = "100 45.11 1 1 0.00 1 3 0  12 36.6910753 -4.5078270 0 190 3 1 1 1 30 36.6795648 -4.4805125 0 489 3 0 0 1";

    /// One end of LEMG 13/31 or 12/30, parsed from the row the app reads.
    pub fn lemg(id: &str) -> Runway {
        let row = if id == "12" || id == "30" { LEMG_12_30 } else { LEMG_13_31 };
        let (e1, e2, length_m, surface) = parse_runway_100(row).expect("the LEMG row parses");
        let e = if e1.id == id { e1 } else { e2 };
        Runway {
            icao: "LEMG".into(),
            id: e.id,
            thr_lat: e.lat,
            thr_lon: e.lon,
            course_deg: e.course,
            length_m,
            surface,
            dist_nm: 0.0,
            elev_ft: 48.0,
        }
    }

    /// The point `along_m` short of the threshold (positive on the approach
    /// side) and `xtk_m` right of the inbound course.
    pub fn at(rwy: &Runway, along_m: f64, xtk_m: f64) -> (f64, f64) {
        let recip = (rwy.course_deg + 180.0).rem_euclid(360.0);
        let (lat, lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, along_m / M_PER_NM);
        let side = if xtk_m >= 0.0 {
            rwy.course_deg + 90.0
        } else {
            rwy.course_deg - 90.0
        };
        project_latlon(lat, lon, side.rem_euclid(360.0), xtk_m.abs() / M_PER_NM)
    }

    /// A 737 in heading mode, fitted to the recorded LEMG ticks: the
    /// autopilot banks 2 degrees per degree of heading error up to 25 (the
    /// base and final turns held 23 to 27), rolls toward it through a 1 s lag
    /// at no more than 7 degrees a second (0 to 16 degrees in 3.6 s on the
    /// short final of sortie 1790591256), turns coordinated, and flies the
    /// bug of the tick before (the loop's transport delay). The wind blows
    /// from 106 degrees at 6 m/s: a 2 degree crab on 131 and 3.7 on 158, as
    /// recorded.
    pub struct Jet {
        pub lat: f64,
        pub lon: f64,
        pub psi: f64,
        pub phi: f64,
        pub tas_mps: f64,
        pub wind_to_deg: f64,
        pub wind_mps: f64,
        /// How its autopilot answers the bug.
        pub fit: HeadingFit,
        queued: std::collections::VecDeque<f64>,
    }

    /// A heading mode, fitted: bank `gain` degrees per degree of heading
    /// error up to `bank_max_deg`, rolled toward through a `lag_s` lag at no
    /// more than `roll_dps`, flying the bug `delay_ticks` ticks late.
    #[derive(Debug, Clone, Copy)]
    pub struct HeadingFit {
        pub gain: f64,
        pub bank_max_deg: f64,
        pub roll_dps: f64,
        pub lag_s: f64,
        pub delay_ticks: usize,
    }

    /// The 737 above.
    pub const B737_HEADING: HeadingFit =
        HeadingFit { gain: 2.0, bank_max_deg: 25.0, roll_dps: 7.0, lag_s: 1.0, delay_ticks: 1 };
    /// The A330's, fitted to the circuit ticks from the turn onto base to
    /// the handoff of the LEMG 30 circuits of 2026-09-29 (sortie
    /// 1790710769-full_auto 1.0 degrees rms, 1790698132 1.4; the 737's fit
    /// flies them 4.0 and 5.1 wrong): the same 2 per degree to 25, but
    /// rolled at 3 degrees a second through 2 s, two ticks late. The state
    /// records agree: 8 degrees of bank 4 s after the bug moved 59, 22 at 8.
    pub const A330_HEADING: HeadingFit =
        HeadingFit { gain: 2.0, bank_max_deg: 25.0, roll_dps: 3.0, lag_s: 2.0, delay_ticks: 2 };

    impl Jet {
        pub fn new(lat: f64, lon: f64, psi: f64, tas_kts: f64) -> Self {
            Self {
                lat,
                lon,
                psi,
                phi: 0.0,
                tas_mps: tas_kts * 0.514_444,
                wind_to_deg: 286.0,
                wind_mps: 6.0,
                fit: B737_HEADING,
                queued: std::collections::VecDeque::new(),
            }
        }

        fn ground(&self) -> (f64, f64) {
            let (s, c) = self.psi.to_radians().sin_cos();
            let (ws, wc) = self.wind_to_deg.to_radians().sin_cos();
            (
                self.tas_mps * s + self.wind_mps * ws,
                self.tas_mps * c + self.wind_mps * wc,
            )
        }

        /// The ground track, degrees true.
        pub fn track(&self) -> f64 {
            let (e, n) = self.ground();
            e.atan2(n).to_degrees().rem_euclid(360.0)
        }

        pub fn gs_kts(&self) -> f64 {
            let (e, n) = self.ground();
            e.hypot(n) / 0.514_444
        }

        /// One loop tick of `dt` seconds with this tick's `bug` (true).
        pub fn step(&mut self, bug: f64, dt: f64) {
            let f = self.fit;
            self.queued.push_back(bug);
            let applied = if self.queued.len() > f.delay_ticks {
                self.queued.pop_front().unwrap_or(bug)
            } else {
                *self.queued.front().unwrap_or(&bug)
            };
            let n = 20;
            let h = dt / n as f64;
            for _ in 0..n {
                let err = ((applied - self.psi + 540.0).rem_euclid(360.0)) - 180.0;
                let want = (f.gain * err).clamp(-f.bank_max_deg, f.bank_max_deg);
                self.phi += ((want - self.phi) / f.lag_s).clamp(-f.roll_dps, f.roll_dps) * h;
                let rate = (9.81 * self.phi.to_radians().tan() / self.tas_mps).to_degrees();
                self.psi = (self.psi + rate * h).rem_euclid(360.0);
                let (e, nn) = self.ground();
                let (lat, lon) = project_latlon(
                    self.lat,
                    self.lon,
                    e.atan2(nn).to_degrees().rem_euclid(360.0),
                    e.hypot(nn) * h / M_PER_NM,
                );
                self.lat = lat;
                self.lon = lon;
            }
        }
    }

    /// The autoland loop's lateral answer, composed as the loop composes it:
    /// guidance on the runway's width, the crab filter, the track trim (its
    /// turn cap given, so the replays can fly the old one), the short final
    /// inside [`SHORT_FINAL_NM`]. The vertical is flown exactly on the path.
    pub struct FinalLoop {
        pub crab: CrabFilter,
        pub trim: TrackTrim,
        pub short_final: Option<ShortFinal>,
        pub width_m: Option<f64>,
        pub trim_cap_deg: f64,
        /// The trim's steady-track gate (`None`: the old cap alone).
        pub trim_steady_dps: Option<f64>,
        /// The knob as the loop sets it (`bug_knob`), when held; the knob
        /// the aircraft flies, how many times it moved (0.2 degrees, the
        /// least `drive_heading_bug_step` writes) and the loop's clock.
        pub knob: Option<crate::circuit::BugHold>,
        pub knob_at: Option<f64>,
        /// A deadband over the loop's own (`bug_knob`), for comparisons.
        pub knob_db: Option<crate::circuit::BugDeadband>,
        pub moves: u32,
        pub t: f64,
    }

    impl FinalLoop {
        pub fn new(width_m: Option<f64>, trim_cap_deg: f64) -> Self {
            Self {
                crab: CrabFilter::new(),
                trim: TrackTrim::new(),
                short_final: None,
                width_m,
                trim_cap_deg,
                trim_steady_dps: Some(TRACK_TRIM_STEADY_DPS),
                knob: None,
                knob_at: None,
                knob_db: None,
                moves: 0,
                t: 0.0,
            }
        }

        /// ...with the knob set as today's loop sets it.
        pub fn held(mut self) -> Self {
            self.knob = Some(crate::circuit::BugHold::default());
            self
        }

        /// One tick: the bug (true) and the guidance behind it.
        pub fn step(&mut self, jet: &Jet, rwy: &Runway, dt: f64) -> (f64, Guidance) {
            let (psi, trk, gs) = (jet.psi, jet.track(), jet.gs_kts());
            let crab = self.crab.update(CrabFilter::raw(Some(psi), Some(trk)), dt);
            let dist = haversine_nm(jet.lat, jet.lon, rwy.thr_lat, rwy.thr_lon);
            let alt = rwy.elev_ft + (dist + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
            let g = guidance_on(
                jet.lat,
                jet.lon,
                alt,
                alt - rwy.elev_ft,
                gs,
                145.0,
                rwy,
                145.0,
                25.0,
                0.0,
                self.width_m,
            );
            let trim = self.trim.update_gated(
                g.phase,
                g.target_heading_deg,
                Some(trk),
                dt,
                self.trim_cap_deg,
                self.trim_steady_dps,
            );
            let off = |a: f64| ((a - rwy.course_deg + 540.0).rem_euclid(360.0)) - 180.0;
            let in_short =
                g.phase == ApproachPhase::Final && g.dist_nm <= SHORT_FINAL_NM && !g.arresting;
            let bug = match g.phase {
                ApproachPhase::Final if in_short => {
                    let xtk_m = cross_track_nm(jet.lat, jet.lon, rwy) * M_PER_NM;
                    let pursuit = g.target_heading_deg + crab + trim;
                    let sf = self
                        .short_final
                        .get_or_insert_with(|| ShortFinal::new(off(pursuit), off(trk)));
                    rwy.course_deg + sf.step(xtk_m, off(trk), gs, dt, Some(jet.phi))
                }
                ApproachPhase::Intercept => {
                    self.short_final = None;
                    g.target_heading_deg
                }
                _ => {
                    self.short_final = None;
                    g.target_heading_deg + crab + trim
                }
            };
            self.t += dt;
            let short = in_short || g.arresting;
            let mut bug = bug.rem_euclid(360.0);
            if let Some(k) = self.knob.as_mut() {
                match bug_knob(g.phase, short, None, None) {
                    Some((key, db)) => {
                        k.decide(key, bug, self.t, self.knob_db.unwrap_or(db));
                        bug = k.set().unwrap_or(bug);
                    }
                    None => k.reset(),
                }
            }
            if self
                .knob_at
                .is_none_or(|a| (((bug - a) + 540.0).rem_euclid(360.0) - 180.0).abs() >= 0.2)
            {
                self.moves += 1;
                self.knob_at = Some(bug);
            }
            (bug, g)
        }
    }

    /// What a flown final came to: the offset at the roundout gate (the last
    /// tick above 80 ft on the path), the worst offset below 500 ft, the
    /// worst bank on the short final, and the first balk (why, radio, xtk).
    #[derive(Debug, Default)]
    pub struct Flown {
        pub gate_xtk_m: f64,
        pub worst_low_xtk_m: f64,
        pub worst_short_bank_deg: f64,
        pub balk: Option<(&'static str, f64, f64)>,
    }

    /// Fly `jet` down the final with `lp` to the roundout gate (80 ft on the
    /// path) or the first balk.
    pub fn fly_final(jet: &mut Jet, lp: &mut FinalLoop, rwy: &Runway, dt: f64) -> Flown {
        let mut out = Flown::default();
        for _ in 0..3000 {
            let (bug, g) = lp.step(jet, rwy, dt);
            let xtk_m = cross_track_nm(jet.lat, jet.lon, rwy) * M_PER_NM;
            let radio = g.glidepath_alt_ft - rwy.elev_ft;
            if let Some(why) = g.balk {
                out.balk = Some((why, radio, xtk_m));
                return out;
            }
            if radio < 80.0 {
                return out;
            }
            out.gate_xtk_m = xtk_m;
            if radio < 500.0 {
                out.worst_low_xtk_m = out.worst_low_xtk_m.max(xtk_m.abs());
            }
            if g.dist_nm <= SHORT_FINAL_NM {
                out.worst_short_bank_deg = out.worst_short_bank_deg.max(jet.phi.abs());
            }
            jet.step(bug, dt);
        }
        out
    }
}

/// Issue #147: LEMG 13 and 31, the 737's full-auto circuits of 2026-09-28,
/// replayed against the real apt.dat geometry.
#[cfg(test)]
mod issue_147_tests {
    use super::lemg_replay::*;
    use super::*;

    /// The logged cross-track is the aircraft's real offset, and the flip
    /// across the threshold was the aircraft crossing the centreline, not a
    /// reference changing: sortie 1790591256-full_auto's `state` positions
    /// against apt.dat's 13/31 row, with the offsets computed independently
    /// (east/north metres off the 13 threshold, projected on the line to 31)
    /// and the `autoland` tick's `xtk_m` nearest in time.
    #[test]
    fn the_lemg_13_cross_track_is_the_true_offset_across_the_threshold() {
        let rwy = lemg("13");
        assert!((rwy.course_deg - 131.70).abs() < 0.02, "{}", rwy.course_deg);
        // (lat, lon, true along-track m (approach side positive), true
        // cross-track m (right positive), the logged xtk_m, radio ft)
        let pts: [(f64, f64, f64, f64, f64, f64); 7] = [
            (
                36.70614964963893,
                -4.541190214471253,
                3499.0,
                -98.4,
                -91.9,
                465.0,
            ),
            (
                36.69694294373395,
                -4.529138374167257,
                2015.6,
                -48.8,
                -48.2,
                413.0,
            ),
            (
                36.68657643707785,
                -4.514858502002209,
                298.0,
                -35.2,
                -32.4,
                90.0,
            ),
            (
                36.68514500057357,
                -4.513137373226305,
                77.6,
                -18.4,
                -14.6,
                44.0,
            ),
            (
                36.68453229043331,
                -4.5124318927144085,
                -14.7,
                -9.4,
                -2.2,
                34.0,
            ),
            (
                36.683699213228394,
                -4.511474633941739,
                -140.1,
                3.0,
                7.8,
                32.0,
            ),
            (
                36.67955142537449,
                -4.506237707168569,
                -795.6,
                36.7,
                38.2,
                0.0,
            ),
        ];
        for (lat, lon, along, xtk, logged, radio) in pts {
            let x = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
            let a = along_track_nm(lat, lon, &rwy) * M_PER_NM;
            // A metre, plus the flat-earth sum's own 0.1 % over distance.
            let tol = 1.0 + along.abs() * 1e-3;
            assert!(
                (x - xtk).abs() < tol,
                "at {radio} ft: {x:.1} m against {xtk:.1} true"
            );
            assert!(
                (a - along).abs() < 2.0,
                "at {radio} ft: {a:.0} m along against {along:.0}"
            );
            // The loop's own number agrees with the position to within the
            // tick it was sampled apart (7.7 m/s sideways over the threshold).
            assert!(
                (x - logged).abs() < 8.0,
                "at {radio} ft: logged {logged:.1}, {x:.1} m"
            );
        }
        // Left of the line 300 m short, across it at the threshold, right of
        // it at the wheels: an S-turn, and 36.7 m on a 45 m strip is 14 m
        // into the grass.
        let wheels = cross_track_nm(pts[6].0, pts[6].1, &rwy) * M_PER_NM;
        assert!(wheels > LEMG_WIDTH_M / 2.0 + 10.0, "{wheels:.1}");
    }

    /// The old short-final learner, as it flew (a 2 s low-pass of the track
    /// against the last bug, clamped at 5 degrees, no gate).
    struct OldShortFinal {
        bias_deg: f64,
        prev_bug_deg: f64,
    }

    impl OldShortFinal {
        fn step(&mut self, xtk_m: f64, track_off_deg: f64, gs_kts: f64, dt: f64) -> f64 {
            let e = track_off_deg - self.prev_bug_deg;
            let a = (dt / SHORT_FINAL_BIAS_TAU_S).min(1.0);
            self.bias_deg = (self.bias_deg + (e - self.bias_deg) * a).clamp(-5.0, 5.0);
            let side = gs_kts * 0.514_444 * track_off_deg.to_radians().sin();
            let bug = short_final_cut_deg(xtk_m, side, gs_kts) - self.bias_deg;
            self.prev_bug_deg = bug;
            bug
        }
    }

    /// LEMG 13 (sortie 1790591256-full_auto): the short final began at
    /// 0.29 nm, 31.8 m left, heading 129.4 and tracking 130.3, the pursuit's
    /// bug 0.33 left of the runway. The old learner took the autopilot's lag
    /// for a bias: the bug ran to 8 degrees right in two seconds and the 737
    /// banked 16 at 90 ft (13.6 in the plant), and it was still turning onto
    /// the line at the gate. Replayed against the fitted 737, 0.4 s ticks.
    #[test]
    fn the_short_final_learns_no_bias_from_its_own_turn() {
        let rwy = lemg("13");
        let off = |a: f64| ((a - rwy.course_deg + 540.0).rem_euclid(360.0)) - 180.0;
        let fly = |old: bool| -> (f64, f64, f64) {
            let (lat, lon) = at(&rwy, 0.29 * M_PER_NM, -31.8);
            let mut jet = Jet::new(lat, lon, 129.4, 146.0);
            jet.wind_mps = 2.5;
            jet.phi = -0.4;
            let entry_bug = -0.33;
            let mut o = OldShortFinal {
                bias_deg: (off(jet.track()) - entry_bug).clamp(-5.0, 5.0),
                prev_bug_deg: entry_bug,
            };
            let mut n = ShortFinal::new(entry_bug, off(jet.track()));
            let (mut worst_bug, mut worst_bank) = (0.0_f64, 0.0_f64);
            let dt = 0.4;
            // To the roundout gate: 80 ft is about 220 m short at 146 kt.
            while along_track_nm(jet.lat, jet.lon, &rwy) * M_PER_NM > 220.0 {
                let x = cross_track_nm(jet.lat, jet.lon, &rwy) * M_PER_NM;
                let b = if old {
                    o.step(x, off(jet.track()), jet.gs_kts(), dt)
                } else {
                    n.step(x, off(jet.track()), jet.gs_kts(), dt, Some(jet.phi))
                };
                worst_bug = worst_bug.max(b.abs());
                worst_bank = worst_bank.max(jet.phi.abs());
                jet.step(rwy.course_deg + b, dt);
            }
            (worst_bug, worst_bank, n.bias_deg)
        };
        let (old_bug, old_bank, _) = fly(true);
        let (bug, bank, bias) = fly(false);
        assert!(
            old_bug > 7.5 && old_bank > 12.0,
            "the replay winds up: bug {old_bug:.1}, bank {old_bank:.1}"
        );
        assert!(
            bug <= SHORT_FINAL_MAX_CUT_DEG + SHORT_FINAL_MAX_BIAS_DEG + 1e-9,
            "the bug stays inside cut plus bias: {bug:.1}"
        );
        assert!(bias.abs() <= SHORT_FINAL_MAX_BIAS_DEG, "{bias:.2}");
        assert!(
            bank < 10.0 && bank < old_bank - 3.0,
            "bank {bank:.1} against {old_bank:.1}"
        );
    }

    /// The same final's positions, judged by the gates on LEMG's 45 m
    /// pavement: at the handoff (98 m off at 1.9 nm, 471 ft) the pursuit's
    /// own time model leaves 20 m at the threshold, 5 m past the landing
    /// limit, and the final is not stabilised below the floor; 49 m off at
    /// 1.1 nm the same; 29 m off at 78 ft is off the centreline low. On a
    /// runway of unknown width all three were FINAL, which is what flew.
    #[test]
    fn the_lemg_13_final_goes_around_below_the_floor_on_a_45_m_strip() {
        let rwy = lemg("13");
        assert!((landing_xtk_limit_m(Some(LEMG_WIDTH_M)) - 15.05).abs() < 0.01);
        // (lat, lon, alt ft MSL, radio ft, gs kt, the balk wanted)
        let pts = [
            (
                36.70614964963893,
                -4.541190214471253,
                533.4,
                471.6,
                148.6,
                "unstabilised_below_floor",
            ),
            (
                36.69694294373395,
                -4.529138374167257,
                479.5,
                432.0,
                150.1,
                "unstabilised_below_floor",
            ),
            (
                36.68578005055136,
                -4.513846406166833,
                136.2,
                77.6,
                146.5,
                "off_centreline_low",
            ),
        ];
        for (lat, lon, alt, radio, gs, want) in pts {
            let g = guidance_on(
                lat,
                lon,
                alt,
                radio,
                gs,
                gs,
                &rwy,
                145.0,
                25.0,
                0.0,
                Some(LEMG_WIDTH_M),
            );
            assert_eq!(g.balk, Some(want), "at {radio} ft");
            assert_eq!(g.phase, ApproachPhase::Intercept, "at {radio} ft");
            let old = guidance(lat, lon, alt, radio, gs, gs, &rwy, 145.0, 25.0, 0.0);
            assert_eq!(
                (old.phase, old.balk),
                (ApproachPhase::Final, None),
                "what flew, at {radio} ft"
            );
        }
        // On the line it is a final: 5 m off at 1.1 nm and at 78 ft.
        for (along, radio) in [(2015.6, 432.0), (171.7, 77.6)] {
            let (lat, lon) = at(&rwy, along, -5.0);
            let alt = rwy.elev_ft + radio;
            let g = guidance_on(
                lat,
                lon,
                alt,
                radio,
                148.0,
                148.0,
                &rwy,
                145.0,
                25.0,
                0.0,
                Some(LEMG_WIDTH_M),
            );
            assert_eq!(
                (g.phase, g.balk),
                (ApproachPhase::Final, None),
                "5 m off at {radio} ft"
            );
        }
    }

    /// LEMG 31 (sortie 1790584715-full_auto): handed over 103 m right at
    /// 1.8 nm, still rolling out of the circuit's intercept (heading 299 for
    /// a 311.7 runway, 15 degrees of bank). The trim learnt the roll-out as a
    /// bias (+0.78 recorded) and the final sat 30 to 32 m right from 1.4 nm
    /// to 0.3 with the autopilot on its bug. Replayed: the old 10 degree cap
    /// parks it there; the 3 degree cap learns a fraction and closes.
    #[test]
    fn the_track_trim_learns_no_bias_from_the_handoffs_roll_out() {
        let rwy = lemg("31");
        let fly = |cap: f64, gate: Option<f64>| -> (f64, f64) {
            let (lat, lon) = at(&rwy, 3392.9, 103.1);
            let mut jet = Jet::new(lat, lon, 299.4, 149.0);
            jet.wind_mps = 3.0;
            jet.phi = 14.8;
            let mut lp = FinalLoop::new(None, cap);
            lp.trim_steady_dps = gate;
            let mut at_half = f64::NAN;
            let mut worst_trim = 0.0_f64;
            for _ in 0..1000 {
                let (bug, g) = lp.step(&jet, &rwy, 0.35);
                if g.dist_nm < 0.5 {
                    at_half = cross_track_nm(jet.lat, jet.lon, &rwy) * M_PER_NM;
                    break;
                }
                worst_trim = worst_trim.max(lp.trim.value().abs());
                jet.step(bug, 0.35);
            }
            (at_half, worst_trim)
        };
        let (old_x, old_trim) = fly(10.0, None);
        let (x, trim) = fly(TRACK_TRIM_ERR_CAP_DEG, Some(TRACK_TRIM_STEADY_DPS));
        assert!(
            (25.0..40.0).contains(&old_x),
            "the replay parks as it flew: {old_x:.1} m at 0.5 nm"
        );
        assert!(old_trim > 0.9, "and learns the roll-out: {old_trim:.2}");
        assert!(
            x.abs() < landing_xtk_limit_m(Some(LEMG_WIDTH_M)),
            "{x:.1} m at 0.5 nm"
        );
        assert!(trim < 0.5, "{trim:.2}");
    }

    /// Issue #93: autoland's final with the knob set as a pilot sets it (half
    /// a degree at most every two seconds outside the short final) holds the
    /// line the every-tick knob held, from the LEMG 31 handover and offsets
    /// either side of 13 in four winds: no balk, the roundout gate within
    /// 2 m of it, the worst offset below 500 ft within 3 m of it or inside
    /// the landing limit (the plant has no gusts, so the knob's moves are
    /// counted on the recordings: `the_recorded_finals_move_the_knob_less`).
    #[test]
    fn the_final_on_a_held_knob_keeps_the_line() {
        let limit = landing_xtk_limit_m(Some(LEMG_WIDTH_M));
        // (runway, along m, xtk m, heading true, IAS, wind from, m/s)
        let starts = [
            ("31", 3392.9, 103.1, 299.4, 149.0, 106.0, 3.0),
            ("13", 3700.0, 30.0, 126.0, 146.0, 106.0, 6.0),
            ("13", 3700.0, -40.0, 136.0, 146.0, 200.0, 8.0),
            ("13", 5500.0, 0.0, 131.7, 150.0, 40.0, 7.0),
        ];
        for (id, along, xtk, psi, ias, wf, wm) in starts {
            let rwy = lemg(id);
            let fly = |held: bool| {
                let (lat, lon) = at(&rwy, along, xtk);
                let mut jet = Jet::new(lat, lon, psi, ias);
                jet.wind_to_deg = (wf + 180.0_f64).rem_euclid(360.0);
                jet.wind_mps = wm;
                let lp = FinalLoop::new(Some(LEMG_WIDTH_M), TRACK_TRIM_ERR_CAP_DEG);
                let mut lp = if held { lp.held() } else { lp };
                let f = fly_final(&mut jet, &mut lp, &rwy, 0.35);
                (f, lp.moves)
            };
            let (free, free_moves) = fly(false);
            let (f, moves) = fly(true);
            assert!(f.balk.is_none() && free.balk.is_none(), "{id} {along} {xtk}: {f:?}");
            // Within 2 m of the every-tick knob or 5 m of the line: the
            // trim that learns no roll-out (#189) left the free knob 0.1 m
            // at the gate from LEMG 31 where the held one keeps 3.2.
            assert!(
                f.gate_xtk_m.abs() <= (free.gate_xtk_m.abs() + 2.0).max(5.0)
                    && f.worst_low_xtk_m <= (free.worst_low_xtk_m + 3.0).max(limit),
                "{id} {along} {xtk}: {f:?} against {free:?}"
            );
            // The knob moves no more than the every-tick one would, or a
            // handful of times: with no roll-out learnt (#189) the free bug
            // barely moves in this still air (5 moves from 13 3700 30,
            // against the held knob's 8).
            assert!(moves <= free_moves.max(10), "{id}: {moves} against {free_moves}");
        }
    }

    /// The knob's keys: the short final and the flare steer every tick; a
    /// pattern leg, a missed approach stage, the final and an intercept are
    /// each set once and corrected past their deadband.
    #[test]
    fn the_knob_keys() {
        use crate::reversal::Leg;
        assert_eq!(bug_knob(ApproachPhase::Flare, false, None, None), None);
        assert_eq!(bug_knob(ApproachPhase::Final, true, None, None), None);
        assert_eq!(
            bug_knob(ApproachPhase::Final, false, None, None),
            Some((2, AUTOLAND_FINAL_DEADBAND))
        );
        let out = bug_knob(ApproachPhase::Intercept, false, Some(Leg::Outbound), None);
        let base = bug_knob(ApproachPhase::Intercept, false, Some(Leg::Base), None);
        assert!(out.is_some() && base.is_some() && out != base);
        let climb = bug_knob(ApproachPhase::Intercept, false, None, Some(MissedStage::Climb));
        let repo = bug_knob(ApproachPhase::Intercept, false, None, Some(MissedStage::Reposition));
        assert!(climb.is_some() && repo.is_some() && climb != repo);
        assert_eq!(
            bug_knob(ApproachPhase::Intercept, false, None, None),
            Some((1, AUTOLAND_INTERCEPT_DEADBAND))
        );
    }

    /// The limits follow the pavement, and a runway of unknown width keeps
    /// the old 37 m.
    #[test]
    fn the_landing_limits_follow_the_pavement() {
        let legacy = FLARE_MAX_XTK_NM * M_PER_NM;
        assert_eq!(landing_xtk_limit_m(None), legacy);
        assert_eq!(flare_xtk_limit_m(None), legacy);
        assert!((landing_xtk_limit_m(Some(45.11)) - 15.05).abs() < 0.01);
        assert!((flare_xtk_limit_m(Some(45.11)) - 18.55).abs() < 0.01);
        assert!((landing_xtk_limit_m(Some(60.0)) - 22.5).abs() < 1e-9);
        // A narrow strip keeps a quarter of its width; a wide one the old cap.
        assert!((landing_xtk_limit_m(Some(18.0)) - 4.5).abs() < 1e-9);
        assert_eq!(landing_xtk_limit_m(Some(150.0)), legacy);
        assert_eq!(landing_xtk_limit_m(Some(f64::NAN)), legacy);
        assert!(flare_xtk_limit_m(Some(18.0)) >= landing_xtk_limit_m(Some(18.0)));
    }

    /// The width is read with the runway: `nearby_runways` notes it, and the
    /// arm's runway finds it.
    #[test]
    fn the_runway_read_from_apt_dat_carries_its_width() {
        let dir = std::env::temp_dir().join(format!("xvl-147-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let apt = dir.join("apt.dat");
        std::fs::write(
            &apt,
            format!("I\n1200 test\n\n1     48 0 0 LEMG Malaga\n{LEMG_13_31}\n99\n"),
        )
        .unwrap();
        let rwy = lemg("13");
        let found = runway_by_id(
            apt.to_str().unwrap(),
            rwy.thr_lat,
            rwy.thr_lon,
            "13",
            5.0,
            Some("LEMG"),
        )
        .unwrap()
        .expect("LEMG 13");
        assert_eq!(runway_width_m(&found), Some(LEMG_WIDTH_M));
        assert_eq!(runway_width_m(&lemg("31")), Some(LEMG_WIDTH_M));
        let _ = std::fs::remove_dir_all(&dir);
    }
}

/// Issue #77: EDLP 24, the SF50's balked balloon of 2026-09-24, replayed.
#[cfg(test)]
mod edlp_go_around_tests {
    use super::*;

    /// The recorded ticks after the balk (s from it, radio ft, vvi fpm, IAS
    /// kt), as the autoland loop logged them: +1,500 then 0 on the dial,
    /// half flap from the balk, the wheels at -1,568.
    const REC: [(f64, f64, f64, f64); 10] = [
        (0.1, 52.1, 860.0, 81.0),
        (0.6, 58.1, 796.0, 79.0),
        (1.1, 60.8, 649.0, 77.3),
        (1.6, 60.6, 418.0, 75.9),
        (2.1, 58.0, 151.0, 74.8),
        (2.6, 53.2, -130.0, 74.2),
        (3.1, 44.8, -463.0, 74.1),
        (3.7, 33.2, -819.0, 74.7),
        (4.2, 18.6, -1175.0, 76.0),
        (4.7, 2.7, -1487.0, 77.8),
    ];
    /// The SF50's numbers: approach 87, Vso 67 (floor 80), the half flap's
    /// stall where the recording met it (74.5, floor 89; by the straight
    /// line to a clean 82), and the best climb the loop asked (+1,500).
    const APPROACH: f64 = 87.0;
    const CLIMB: f64 = 1500.0;
    fn stall(flap: f64) -> f64 {
        67.0 + (82.0 - 67.0) * (1.0 - flap)
    }
    fn floor(flap: f64) -> f64 {
        (1.2 * stall(flap)).round()
    }

    /// The airframe as it answered, fitted to the ticks before the stall: the
    /// speed changes at -2.06 + 0.38 t kt/s (idle spooling up after the
    /// balk; capped at +2.5) less 2.44 kt/s per 1,000 fpm of climb (0.08
    /// kt/s rms; g sin of the path at 78 kt is 2.41, so the climb's cost is
    /// physics, not a fit); the VS mode follows the dial after 0.3 s through
    /// a 0.5 s lag, down at up to 700 fpm a second and UP at only 50 (the
    /// +1,500 asked at 81 kt never showed); under the stall speed a wing the
    /// autopilot keeps pulling loses 800 fpm a second, until it is 3 kt clear
    /// and the dial lets it go. The recorded radio height fell faster than
    /// the VSI (58 to 3 ft in 2.6 s at a mean of -800), so height is not
    /// fitted.
    struct Sf50 {
        t: f64,
        h: f64,
        vvi: f64,
        ias: f64,
        stalled: bool,
        ever_stalled: bool,
        dials: std::collections::VecDeque<(f64, f64)>,
        dial: f64,
    }

    impl Sf50 {
        fn at_the_balk() -> Self {
            Sf50 {
                t: 0.0,
                h: REC[0].1,
                vvi: REC[0].2,
                ias: REC[0].3,
                stalled: false,
                ever_stalled: false,
                dials: std::collections::VecDeque::new(),
                dial: REC[0].2,
            }
        }
        fn write(&mut self, dial: f64) {
            self.dials.push_back((self.t + 0.3, dial));
        }
        fn step(&mut self, flap: f64, dt: f64) {
            while self.dials.front().is_some_and(|&(t, _)| t <= self.t) {
                self.dial = self.dials.pop_front().map_or(self.dial, |(_, d)| d);
            }
            let s = stall(flap);
            if self.ias < s && self.dial >= self.vvi - 100.0 {
                self.stalled = true;
            }
            if self.stalled && (self.ias > s + 3.0 || self.dial < self.vvi - 100.0) {
                self.stalled = false;
            }
            self.ever_stalled |= self.stalled;
            let vdot = if self.stalled {
                -800.0
            } else {
                ((self.dial - self.vvi) / 0.5).clamp(-700.0, 50.0)
            };
            self.vvi += vdot * dt;
            let a = (-2.06 + 0.38 * (self.t + REC[0].0)).min(2.5);
            self.ias += (a - 2.44 * self.vvi / 1000.0) * dt;
            self.h += self.vvi / 60.0 * dt;
            self.t += dt;
        }
    }

    struct Flown {
        wheels_fpm: f64,
        stalled: bool,
        min_ias: f64,
        min_h: f64,
        trace: Vec<(f64, f64, f64, f64)>,
    }

    /// Fly from the balk to the ground (or 20 s) at the loop's 0.5 s tick.
    /// `new`: today's law and flap; else the dial and flap as flown.
    fn fly(new: bool) -> Flown {
        let mut p = Sf50::at_the_balk();
        let (mut prev_ias, mut flap) = ((83.7, -1.1), 1.0);
        let mut out = Flown {
            wheels_fpm: 0.0,
            stalled: false,
            min_ias: p.ias,
            min_h: p.h,
            trace: Vec::new(),
        };
        let (dt, tick) = (0.01, 0.5);
        let mut next = 0.0;
        while p.h > 0.0 && p.t < 20.0 {
            if p.t + 1e-9 >= next {
                next += tick;
                let trend = (p.ias - prev_ias.0) / (p.t - prev_ias.1);
                prev_ias = (p.ias, p.t);
                let dial = if new {
                    if go_around_flap_due(p.ias, Some(floor(0.5))) {
                        flap = 0.5;
                    }
                    go_around_vs_fpm(p.ias, trend, APPROACH, Some(floor(flap)), p.h, CLIMB)
                } else {
                    flap = 0.5;
                    if APPROACH - p.ias > SPEED_FLOOR_MARGIN_KTS {
                        0.0
                    } else {
                        CLIMB
                    }
                };
                p.write(dial);
                out.trace.push((p.t, p.h, p.vvi, p.ias));
            }
            p.step(flap, dt);
            out.min_ias = out.min_ias.min(p.ias);
            out.min_h = out.min_h.min(p.h);
        }
        out.wheels_fpm = p.vvi;
        out.stalled = p.ever_stalled;
        out
    }

    /// The plant is the recording: the dial and flap as flown give back the
    /// recorded ticks and the stall onto the runway.
    #[test]
    fn the_sf50_plant_replays_the_edlp_go_around() {
        let f = fly(false);
        for &(t, h, vvi, ias) in REC.iter().skip(1) {
            let k = f
                .trace
                .iter()
                .min_by(|a, b| (a.0 - t).abs().total_cmp(&(b.0 - t).abs()))
                .unwrap();
            assert!((k.3 - ias).abs() < 1.0, "{t} s: {:.1} kt against {ias}", k.3);
            assert!((k.2 - vvi).abs() < 350.0, "{t} s: {:.0} fpm against {vvi}", k.2);
            let _ = h;
        }
        assert!(f.stalled, "the go-around as flown stalls");
        assert!(f.wheels_fpm < -1200.0, "and meets the runway hard: {:.0}", f.wheels_fpm);
    }

    /// Pitched for speed: the half flap waits for its floor, the dial asks
    /// level and then a descent the height allows while the speed is short,
    /// and the SF50 never stalls: its lowest speed 75.7 kt against the full
    /// flap's 67, its lowest height 30 ft over the runway, climbing (+426) when
    /// the thrust has come, where the go-around as flown hit at -1,729.
    #[test]
    fn a_slow_low_go_around_pitches_for_speed_and_never_stalls() {
        let f = fly(true);
        assert!(!f.stalled, "{:?}", f.trace);
        assert!(f.min_ias > stall(1.0) + 3.0, "{:.1}", f.min_ias);
        assert!(f.min_h > 20.0, "{:.0} ft", f.min_h);
        assert!(f.wheels_fpm > 200.0, "climbing away: {:.0} fpm", f.wheels_fpm);
    }

    /// The law by itself: on speed, the airframe's climb; slow over the
    /// floor, level; under it, a descent no steeper than the ground ten
    /// seconds away, level on the runway; a lost trend or floor is no
    /// measurement (the approach speed's own floor, no projection).
    #[test]
    fn the_go_around_vertical_is_speed_first() {
        assert_eq!(go_around_vs_fpm(86.0, 0.0, 87.0, Some(80.0), 50.0, 1500.0), 1500.0);
        // Decaying at 3 kt/s: 86 projects to 80, slow.
        assert_eq!(go_around_vs_fpm(86.0, -4.0, 87.0, Some(78.0), 50.0, 1500.0), 0.0);
        let v = go_around_vs_fpm(79.0, -3.0, 87.0, Some(80.0), 60.0, 1500.0);
        assert!((v + 360.0).abs() < 1e-9, "{v}");
        let v = go_around_vs_fpm(79.0, -3.0, 87.0, Some(80.0), 600.0, 1500.0);
        assert!((v + 700.0).abs() < 1e-9, "{v}");
        assert_eq!(go_around_vs_fpm(70.0, -3.0, 87.0, Some(80.0), 0.0, 1500.0), 0.0);
        assert_eq!(go_around_vs_fpm(86.0, f64::NAN, 87.0, None, 50.0, 1500.0), 1500.0);
        assert!(go_around_vs_fpm(70.0, f64::NAN, 87.0, None, 50.0, 1500.0) < 0.0);
        // The flap: half at its floor, not before.
        assert!(!go_around_flap_due(81.0, Some(88.0)));
        assert!(go_around_flap_due(88.0, Some(88.0)));
        assert!(go_around_flap_due(81.0, None));
    }
}

/// Issue #163: LEMG 12, the SF50 (sortie 1790611629, 2026-09-28), in rough
/// air (the final's stray 42 fpm rms). The arrest took the servos at 51 ft,
/// its roundout pulled 0.19 into a +263 fpm balloon and pushed -0.05 through
/// it, handed the servos back at 50.5 ft (the band's top, 50 ft, from the
/// radio height alone), the autopilot flew its -445 fpm dial for 1.2 s with
/// the lever walked 0.16 to 0.27, the arrest took them again at 48.4 ft with
/// the sink rebuilding, and the wheels met the runway at -814 fpm.
///
/// The plant is the SF50's elevator with the servos out, in the form of
/// [`FlareModel`]: a standing change of sink per unit of pull through two
/// lags after a dead time, fitted to the three clean SF50 arrests on record
/// (ETND 26 2026-09-23, EDLP 24's balked balloon 2026-09-24, and this
/// roundout), each flare's own plant and the one fitted to all three. On
/// top of it, what the model does not carry: the speed bleeds 1.71 kt/s
/// less 1.29 kt/s per 1,000 fpm of climb (fitted to the same four SF50
/// arrests' ticks), under the 67 kt Vso the wing gives the sink 800 fpm a
/// second (the sink's own answer to the speed is inside the fit: a fit
/// with 63 fpm a knot on top chose none), and rough air is a random
/// vertical gust (an Ornstein-Uhlenbeck sink, 1.5 s). The plugin's pitch
/// channel and the loop's tick (0.35 s) are inside the dead time.
#[cfg(test)]
mod lemg12_sf50_balloon_tests {
    use super::*;

    /// The recorded arrest (s from `arrest_hands` taken, radio ft, vvi
    /// fpm, the elevator written), to the tick before the servos went back.
    const REC: [(f64, f64, f64, f64); 8] = [
        (0.0, 51.15, -529.3, 0.11),
        (0.4, 46.82, -525.2, 0.16),
        (0.8, 43.87, -512.1, 0.19),
        (1.2, 41.72, -454.1, 0.17),
        (1.5, 41.55, -323.6, 0.09),
        (1.9, 43.29, -125.7, -0.03),
        (2.3, 46.07, 62.7, -0.05),
        (2.7, 48.94, 214.5, -0.05),
    ];
    /// The trend and airspeed at the gate, and when the servos went back.
    const GATE_TREND: f64 = 8.5;
    const GATE_IAS: f64 = 86.4;
    const HANDED_BACK_S: f64 = 3.1;
    const HANDED_BACK_FT: f64 = 50.48;
    const TICK_S: f64 = 0.35;
    const VSO_KTS: f64 = 67.0;

    /// (fpm per unit, lag 1, lag 2, dead time): ETND 26, EDLP 24, LEMG 12,
    /// and all three together (the profile's).
    const PLANTS: [(f64, f64, f64, f64); 4] = [
        (7000.0, 1.2, 1.2, 1.0),
        (9000.0, 1.5, 1.2, 0.8),
        (10000.0, 1.5, 0.9, 0.7),
        (8000.0, 1.2, 1.2, 0.9),
    ];
    const LEMG_FIT: usize = 2;
    const PROFILE_FIT: usize = 3;
    /// The other recorded gates (radio ft, vvi fpm, trend fpm/s, IAS kt):
    /// ETND 26, EDLP 24's balked balloon, EDLP 24's landing.
    const OTHER_GATES: [(f64, f64, f64, f64); 3] = [
        (49.0, -456.0, -13.0, 88.0),
        (49.0, -371.0, -2.0, 88.0),
        (49.0, -571.0, 43.0, 87.0),
    ];

    struct Plant {
        fit: (f64, f64, f64, f64),
        h: f64,
        v: f64,
        ias: f64,
        base: f64,
        b1: f64,
        b2: f64,
        stall: f64,
        gust: f64,
        rng: u64,
        rough_fpm: f64,
        t: f64,
        written: Vec<(f64, f64)>,
    }

    impl Plant {
        /// At a gate: sinking `v` with `trend`, the column that made the
        /// trend already in the pipe.
        fn at(fit: (f64, f64, f64, f64), h: f64, v: f64, trend: f64, ias: f64) -> Self {
            let b1 = trend * fit.2;
            Plant {
                fit,
                h,
                v,
                ias,
                base: v,
                b1,
                b2: 0.0,
                stall: 0.0,
                gust: 0.0,
                rng: 0,
                rough_fpm: 0.0,
                t: 0.0,
                written: vec![(-10.0, b1 / fit.0)],
            }
        }
        fn rough(mut self, seed: u64, rms_fpm: f64) -> Self {
            self.rng = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
            self.rough_fpm = rms_fpm;
            self
        }
        fn gauss(&mut self) -> f64 {
            let mut u = || {
                self.rng ^= self.rng << 13;
                self.rng ^= self.rng >> 7;
                self.rng ^= self.rng << 17;
                ((self.rng >> 11) as f64 + 0.5) / (1u64 << 53) as f64
            };
            let (a, b) = (u(), u());
            (-2.0 * a.ln()).sqrt() * (std::f64::consts::TAU * b).cos()
        }
        fn write(&mut self, e: f64) {
            self.written.push((self.t, e));
        }
        fn step(&mut self, dt: f64) {
            let (k, l1, l2, dead) = self.fit;
            let due = self.t - dead;
            let arriving = self
                .written
                .iter()
                .take_while(|(t, _)| *t <= due + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            self.b1 += (k * arriving - self.b1) * dt / l1;
            self.b2 += (self.b1 - self.b2) * dt / l2;
            if self.rough_fpm > 0.0 {
                const TC: f64 = 1.5;
                let n = self.gauss();
                self.gust += -self.gust / TC * dt + self.rough_fpm * (2.0 * dt / TC).sqrt() * n;
            }
            if self.ias < VSO_KTS {
                self.stall += 800.0 * dt;
            }
            self.v = self.base + self.b2 + self.gust - self.stall;
            self.ias += (-1.71 - 1.29 * self.v / 1000.0) * dt;
            self.h += self.v / 60.0 * dt;
            self.t += dt;
        }
        /// The recorded column, written as it was, to `until`.
        fn recorded(&mut self, until: f64) {
            while self.t < until - 1e-9 {
                let e = REC
                    .iter()
                    .take_while(|r| r.0 <= self.t + 1e-9)
                    .last()
                    .map_or(0.0, |r| r.3);
                if self.written.last().map(|w| w.1) != Some(e) {
                    self.write(e);
                }
                self.step(0.01);
            }
        }
    }

    /// Which elevator law flies.
    enum Law {
        Model(FlareModel),
        Tracker(FlarePitch, Option<f64>),
    }

    impl Law {
        fn sf50_elevator() -> crate::aircraft::FlareElevatorProfile {
            crate::aircraft::load("cirrus_vision_sf50")
                .and_then(|p| p.flare_elevator)
                .expect("the SF50 declares its flare")
        }
        /// Today's: the profile's model.
        fn today() -> Self {
            Law::Model(
                FlareModel::from_profile(Some(Self::sf50_elevator()))
                    .expect("the SF50 declares its model"),
            )
        }
        /// The tracker the SF50 flew at LEMG 12 (the profile's gains, no
        /// model), still with the servos latched.
        fn as_flown() -> Self {
            Law::Tracker(
                FlarePitch::with_gains(FlareGains::from_profile(Some(Self::sf50_elevator()))),
                None,
            )
        }
        fn step(&mut self, h: f64, gs: f64, v: f64, dt: f64, flare: bool, rough: bool) -> f64 {
            match self {
                Law::Model(m) => m.step(h, gs, v, dt, flare, rough).elev,
                Law::Tracker(p, prev) => {
                    let trend = prev.map_or(0.0, |p| (v - p) / dt);
                    *prev = Some(v);
                    let nominal = -(gs / 60.0) * gp_ft_per_nm();
                    let (roundout, flare_vs) = arrest_targets_fpm(nominal);
                    p.step(if flare { flare_vs } else { roundout }, v, trend, dt, flare)
                }
            }
        }
    }

    struct Landed {
        wheels_fpm: f64,
        pushed_climbing: bool,
        secs: f64,
        min_ias: f64,
    }

    /// The arrest from here to the wheels at the loop's tick, the servos
    /// kept (the latch): the flare below its gate and a flare stays a flare
    /// to the band's top (`hold_flare`), the roundout above.
    fn land(p: &mut Plant, law: &mut Law, rough: bool) -> Landed {
        let start = p.t;
        let (mut next, mut flaring) = (p.t, false);
        let mut out = Landed {
            wheels_fpm: 0.0,
            pushed_climbing: false,
            secs: 0.0,
            min_ias: p.ias,
        };
        while p.h > 0.0 && p.t - start < 40.0 {
            if p.t + 1e-9 >= next {
                next += TICK_S;
                let gs = p.ias + 3.0;
                let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
                flaring = p.h <= flare_ft || (flaring && p.h <= band_ft);
                let e = law.step(p.h, gs, p.v, TICK_S, flaring, rough);
                out.pushed_climbing |= p.v > BALLOON_CLIMB_FPM && e < -1e-9;
                p.write(e);
            }
            p.step(0.01);
            out.min_ias = out.min_ias.min(p.ias);
        }
        out.wheels_fpm = p.v;
        out.secs = p.t - start;
        out
    }

    fn at_the_gate(fit: (f64, f64, f64, f64)) -> Plant {
        Plant::at(fit, REC[0].1, REC[0].2, GATE_TREND, GATE_IAS)
    }

    /// The plant is the recording: the column as written gives back the
    /// sink as measured, the balloon included, under 12 fpm rms on its own
    /// fit and 130 on the profile's (the other flares' own plants are other
    /// flares: 200 and more here).
    #[test]
    fn the_sf50_plant_replays_the_lemg_roundout() {
        for (i, fit) in PLANTS.iter().enumerate() {
            let mut p = at_the_gate(*fit);
            let mut sq = 0.0;
            for r in REC.iter().skip(1) {
                p.recorded(r.0);
                sq += (p.v - r.2).powi(2);
            }
            let rms = (sq / (REC.len() - 1) as f64).sqrt();
            match i {
                LEMG_FIT => assert!(rms < 12.0, "its own fit: {rms:.1} fpm rms"),
                PROFILE_FIT => assert!(rms < 140.0, "the profile's: {rms:.1} fpm rms"),
                _ => {}
            }
        }
    }

    /// The tick that gave the servos back: Final at 50.48 ft, 0.48 over a
    /// band read from the radio height alone. With the servos taken it is
    /// still the arrest, down at the roundout's sink; the arrest keeps them.
    #[test]
    fn a_balloon_over_the_band_keeps_the_servos() {
        let (_, band) = arrest_gates_ft(25.0, 85.0);
        assert!(HANDED_BACK_FT > band, "{band}");
        let tick = Guidance {
            target_heading_deg: 117.0,
            target_vs_fpm: -444.6,
            target_ias_kts: 87.0,
            dist_nm: 0.05,
            glidepath_alt_ft: 30.0,
            phase: ApproachPhase::Final,
            balk: None,
            arresting: false,
        };
        // The profile's recited landing distance, 1,721 ft.
        let sf50_m = Some(1721.0 * M_PER_FT);
        let held = hold_roundout(tick, true, HANDED_BACK_FT, 85.0, 2700.0, sf50_m);
        assert!(held.arresting, "still the arrest");
        assert_eq!((held.phase, held.balk), (ApproachPhase::Final, None));
        let (roundout, _) = arrest_targets_fpm(-(85.0 / 60.0) * gp_ft_per_nm());
        assert_eq!(held.target_vs_fpm, roundout);
        assert_eq!(arrest_hands_step(true, held.arresting), ArrestHands::Keep);
        // As flown, the same tick handed them back.
        assert_eq!(arrest_hands_step(true, tick.arresting), ArrestHands::Return);
        // Above the commit height, or with no room to stop, it is a
        // balked landing: the go-around brings the autopilot back.
        let high = hold_roundout(tick, true, BALLOON_COMMIT_FT + 1.0, 85.0, 2700.0, sf50_m);
        assert_eq!(
            (high.phase, high.balk, high.arresting),
            (ApproachPhase::Intercept, Some("balloon"), false)
        );
        assert_eq!(arrest_hands_step(true, high.arresting), ArrestHands::Return);
        let short = hold_roundout(tick, true, HANDED_BACK_FT, 85.0, 100.0, sf50_m);
        assert_eq!(short.balk, Some("balloon"));
        // Servos never taken, a real balk, an arrest already: untouched.
        assert_eq!(
            hold_roundout(tick, false, HANDED_BACK_FT, 85.0, 2700.0, sf50_m).arresting,
            false
        );
        let balk = Guidance {
            phase: ApproachPhase::Intercept,
            balk: Some("off_centreline_low"),
            ..tick
        };
        assert_eq!(
            hold_roundout(balk, true, HANDED_BACK_FT, 85.0, 2700.0, sf50_m).balk,
            balk.balk
        );
        let arresting = Guidance {
            arresting: true,
            target_vs_fpm: -100.0,
            ..tick
        };
        assert_eq!(
            hold_roundout(arresting, true, 40.0, 85.0, 2700.0, sf50_m).target_vs_fpm,
            -100.0
        );
    }

    /// A balloon never pushes: the tracker's roundout wrote -0.05 at +62
    /// and +214 fpm; the model's roundout, and both flares, never below
    /// zero while climbing.
    #[test]
    fn a_roundout_balloon_never_pushes() {
        let gains = FlareGains::from_profile(Some(Law::sf50_elevator()));
        let mut law = FlarePitch::with_gains(gains);
        for &(_, _, v, _) in REC.iter().take(5) {
            law.step(-283.0, v, 0.0, TICK_S, false);
        }
        // +214 fpm, easing at 396 fpm/s against the -268 target: the
        // recorded -0.05, and now nothing.
        assert!(law.step(-268.0, 214.5, 396.0, TICK_S, false) >= 0.0);
        // Sinking, the roundout is still free both ways.
        let mut law = FlarePitch::with_gains(gains);
        assert!(law.step(-283.0, -125.7, 495.0, TICK_S, false) < 0.0);
        let mut m = FlareModel::from_profile(Some(Law::sf50_elevator())).expect("model");
        for &(_, h, v, _) in REC.iter() {
            let e = m.step(h, 86.0, v, TICK_S, false, true).elev;
            if v > BALLOON_CLIMB_FPM {
                assert!(e >= 0.0, "{v} fpm: {e}");
            }
        }
    }

    /// From the gate, on every flare's own plant and every recorded SF50
    /// gate, the model lands under 300 fpm and never pushes a climb; the
    /// tracker the SF50 flew, the servos latched and never pushing, still
    /// porpoises through the dead time to harder than 400 somewhere.
    #[test]
    fn the_sf50_flare_lands_every_gate_on_every_plant() {
        let gates = std::iter::once((REC[0].1, REC[0].2, GATE_TREND, GATE_IAS)).chain(OTHER_GATES);
        let mut tracker_worst: f64 = 0.0;
        for (h, v, trend, ias) in gates {
            for fit in PLANTS {
                let mut p = Plant::at(fit, h, v, trend, ias);
                let l = land(&mut p, &mut Law::today(), false);
                assert!(
                    l.wheels_fpm > -300.0,
                    "{fit:?} from {h} ft {v} fpm: {:.0}",
                    l.wheels_fpm
                );
                assert!(!l.pushed_climbing, "{fit:?} from {h} ft");
                assert!(l.min_ias > VSO_KTS + 5.0, "{fit:?}: {:.1} kt", l.min_ias);
                assert!(l.secs < 12.0, "{fit:?}: floated {:.1} s", l.secs);
                let mut p = Plant::at(fit, h, v, trend, ias);
                tracker_worst =
                    tracker_worst.min(land(&mut p, &mut Law::as_flown(), false).wheels_fpm);
            }
        }
        assert!(tracker_worst < -400.0, "{tracker_worst:.0}");
    }

    /// The recorded balloon, recovered: the column as written to the tick
    /// the servos went back (+263 fpm at 50.5 ft), then the arrest keeps
    /// them. The rebuilt sink is arrested: under 320 fpm on every plant in
    /// still air, under 420 in 40 fpm rms of gusts (ten seeds each), where
    /// the hand-back met the runway at -814.
    #[test]
    fn the_recorded_balloon_is_recovered_without_the_autopilot() {
        let mut worst_calm: f64 = 0.0;
        let mut worst_rough: f64 = 0.0;
        for fit in PLANTS {
            for seed in 0..=10u64 {
                let mut p = at_the_gate(fit);
                if seed > 0 {
                    p = p.rough(seed, 40.0);
                }
                p.recorded(HANDED_BACK_S);
                if seed == 0 && fit == PLANTS[LEMG_FIT] {
                    // Still climbing (recorded +263; the fit, whose column
                    // was -0.05 for a dead time already, +104).
                    assert!(p.v > BALLOON_CLIMB_FPM, "the balloon: {:.0} fpm", p.v);
                }
                let l = land(&mut p, &mut Law::today(), seed > 0);
                assert!(!l.pushed_climbing, "{fit:?} seed {seed}");
                if seed == 0 {
                    worst_calm = worst_calm.min(l.wheels_fpm);
                } else {
                    worst_rough = worst_rough.min(l.wheels_fpm);
                }
            }
        }
        assert!(worst_calm > -300.0, "{worst_calm:.0}");
        assert!(worst_rough > -400.0, "{worst_rough:.0}");
    }

    /// Rough air, every recorded gate and plant, ten gust seeds each at
    /// the 40 fpm rms the rough-air gate starts at: the SF50's prediction
    /// flown in any air lands all 160 under 400 (the worst 236). The 737's
    /// rough-air hold on the same elevator (pull only ever back) turns the
    /// pulls into floats that bleed the speed onto the runway: 57 of them
    /// past 400 fpm, the worst 1,098.
    #[test]
    fn in_rough_air_the_sf50_flies_its_prediction() {
        let held = {
            let mut p = Law::sf50_elevator();
            if let Some(m) = p.model.as_mut() {
                m.rough_hold = None;
            }
            p
        };
        let gates: Vec<_> = std::iter::once((REC[0].1, REC[0].2, GATE_TREND, GATE_IAS))
            .chain(OTHER_GATES)
            .collect();
        let (mut worst, mut n, mut held_hard, mut worst_held) = (0.0_f64, 0, 0, 0.0_f64);
        for &(h, v, trend, ias) in &gates {
            for fit in PLANTS {
                for seed in 1..=10u64 {
                    let mut p = Plant::at(fit, h, v, trend, ias).rough(seed, 40.0);
                    let l = land(&mut p, &mut Law::today(), true);
                    assert!(!l.pushed_climbing, "{fit:?} seed {seed}");
                    worst = worst.min(l.wheels_fpm);
                    n += 1;
                    let mut p = Plant::at(fit, h, v, trend, ias).rough(seed, 40.0);
                    let law = FlareModel::from_profile(Some(held)).expect("model");
                    let l = land(&mut p, &mut Law::Model(law), true);
                    held_hard += usize::from(l.wheels_fpm < -400.0);
                    worst_held = worst_held.min(l.wheels_fpm);
                }
            }
        }
        assert!(worst > -400.0, "{worst:.0} over {n}");
        assert!(
            held_hard * 5 > n,
            "held: {held_hard} of {n} past 400, worst {worst_held:.0}"
        );
    }
}

/// Issue #166: LEMG 13, the SR22 (sortie 1790619465-approach, 2026-09-28),
/// 4.7 kt of wind from 145, the final's air 20 fpm rms (smooth). The arrest
/// took the servos at 50.6 ft sinking 476 fpm and flew the 172's tracker on
/// this elevator: 0.14 of pull had the sink at +10 by 45 ft, the roundout
/// then PUSHED 0.10 and the sink rebuilt to -471 at 28 ft; again to -52;
/// the flare's full 0.35 from 22 ft met the step to idle a dead time later
/// (-528 at 12.5 ft), ballooned to +298 at 22 ft, was let go to neutral in
/// one tick, the balloon's thrust came back and went again, and the sink
/// the 0.35 met too late was -585 fpm at 1.4 ft (575 on the scorecard).
///
/// The plant is the SR22's elevator with the servos out in the form of
/// [`FlareModel`] (a standing change of sink per unit of pull through two
/// lags after a dead time), plus what the model does not carry, fitted to
/// the same 38 ticks: the thrust (fpm per unit of lever through a lag) and
/// the speed (fpm of sink per knot lost from the gate), the speed itself
/// -2.70 kt/s less 5.1 per 1,000 fpm of climb plus 2.74 per unit of lever
/// (0.32 kt/s rms). Each plant is a fit: the whole record, and the record's
/// parts (the first arrest, the arrest to the flare, the flare), so a law
/// that lands on all of them does not depend on one fit being right. Rough
/// air is a random vertical gust (an Ornstein-Uhlenbeck sink, 1.5 s). The
/// plugin's pitch channel and the loop's tick (0.35 s) are inside the dead
/// time.
///
/// What the plants say: at idle with full flap this wing bleeds 2.7 kt a
/// second and sinks about 115 fpm harder for each knot, so a flare that cuts
/// the lever at its gate has lost the lift it is flaring with before the
/// column's dead time is out. The profile declares the model (6,000 fpm per
/// unit through 1.0 s lags after 0.8 s: under every fit's gain but the
/// arrest's, so it pulls for the weakest; the fitted 10,000 / 1.45 / 0.9
/// lands the same gates at 291) and a retard held until a second from the
/// wheels (`flare_power` `retard_at_s`).
#[cfg(test)]
mod lemg13_sr22_flare_tests {
    use super::*;

    /// The recorded arrest (s from `arrest_hands` taken, radio ft, vvi fpm,
    /// the elevator written, IAS kt), to the last tick before the wheels.
    const REC: [(f64, f64, f64, f64, f64); 38] = [
        (0.0, 50.63, -475.8, 0.126, 92.23),
        (0.4, 46.72, -475.0, 0.139, 92.32),
        (0.9, 43.75, -447.2, 0.105, 92.48),
        (1.3, 43.15, -357.5, -0.028, 92.58),
        (1.8, 44.21, -179.0, -0.100, 92.47),
        (2.2, 45.21, -43.5, -0.100, 92.08),
        (2.7, 45.18, 9.7, -0.100, 91.43),
        (3.1, 43.88, -37.5, -0.100, 90.88),
        (3.6, 41.09, -152.1, 0.016, 90.42),
        (4.0, 37.69, -268.3, 0.114, 90.17),
        (4.5, 33.87, -404.7, 0.206, 90.09),
        (4.8, 30.61, -464.0, 0.202, 90.16),
        (5.2, 28.07, -471.1, 0.127, 90.32),
        (5.6, 27.52, -386.1, -0.035, 90.37),
        (6.0, 28.40, -214.6, -0.100, 90.20),
        (6.4, 28.87, -91.0, -0.100, 89.81),
        (6.9, 28.15, -52.1, -0.100, 89.17),
        (7.3, 25.98, -115.3, -0.045, 88.66),
        (7.8, 21.76, -261.7, 0.221, 88.22),
        (8.0, 19.03, -352.8, 0.312, 88.11),
        (8.4, 14.87, -475.2, 0.350, 88.14),
        (8.7, 12.51, -528.0, 0.350, 88.26),
        (9.0, 11.12, -503.4, 0.350, 88.30),
        (9.3, 11.41, -408.4, 0.350, 88.14),
        (9.6, 13.00, -236.7, 0.000, 87.65),
        (9.9, 15.08, -74.4, 0.000, 87.05),
        (10.2, 18.24, 123.9, 0.000, 86.08),
        (10.5, 20.25, 239.6, 0.000, 85.12),
        (10.8, 21.73, 298.4, 0.000, 83.79),
        (11.1, 22.04, 276.3, 0.000, 82.81),
        (11.4, 21.47, 189.7, 0.000, 81.66),
        (11.7, 20.14, 83.4, 0.053, 80.85),
        (12.1, 17.81, -74.6, 0.176, 80.11),
        (12.4, 15.00, -195.5, 0.269, 79.80),
        (12.7, 11.14, -349.1, 0.350, 79.65),
        (13.0, 7.55, -469.4, 0.350, 79.67),
        (13.3, 3.83, -558.4, 0.350, 79.71),
        (13.6, 1.37, -584.8, 0.350, 79.65),
    ];
    /// The lever as commanded (s, ratio): approach power, the arrest's
    /// "only back", idle at the flare gate, the balloon's approach thrust,
    /// idle again.
    const THR: [(f64, f64); 5] = [
        (-10.0, 0.318),
        (1.5, 0.309),
        (7.9, 0.0),
        (10.3, 0.309),
        (12.2, 0.0),
    ];
    const TICK_S: f64 = 0.35;
    /// The SR22's landing-configuration stall (POH, profile `vs0_kts`).
    const VSO_KTS: f64 = 60.0;
    /// The final's approach thrust and speed at the gate.
    const APPROACH_THR: f64 = 0.318;
    const APPROACH_IAS: f64 = 92.0;
    /// Groundspeed less airspeed (4.7 kt from 145 on runway 13).
    const HEADWIND_KTS: f64 = 3.0;

    /// One fit: (fpm per unit, lag 1, lag 2, dead time), the column the
    /// autopilot left in the pipe, fpm per unit of lever and its lag, fpm
    /// of sink per knot lost.
    #[derive(Debug, Clone, Copy, PartialEq)]
    struct Fit {
        el: (f64, f64, f64, f64),
        pre: f64,
        thr_fpm: f64,
        thr_lag: f64,
        per_kt: f64,
    }

    /// The whole record (39 fpm rms), then fitted to its parts alone: the
    /// first arrest (3 fpm rms there), the arrest to the flare gate (26),
    /// the flare (37, and 50 over the whole record).
    const PLANTS: [Fit; 4] = [
        Fit {
            el: (10000.0, 1.46, 1.43, 0.92),
            pre: -0.058,
            thr_fpm: 316.0,
            thr_lag: 2.70,
            per_kt: 115.0,
        },
        Fit {
            el: (7720.0, 0.95, 1.72, 0.77),
            pre: -0.062,
            thr_fpm: 1074.0,
            thr_lag: 1.10,
            per_kt: 34.0,
        },
        Fit {
            el: (6240.0, 1.03, 1.03, 1.17),
            pre: -0.132,
            thr_fpm: 1160.0,
            thr_lag: 2.78,
            per_kt: 183.0,
        },
        Fit {
            el: (9070.0, 1.42, 1.31, 0.86),
            pre: -0.038,
            thr_fpm: 1.0,
            thr_lag: 0.16,
            per_kt: 94.0,
        },
    ];
    const WHOLE: usize = 0;

    /// Gates the arrest must land from (radio ft, vvi fpm, trend fpm/s,
    /// IAS kt): the recorded one, and the same final a little steeper,
    /// shallower, slower and faster.
    const GATES: [(f64, f64, f64, f64); 5] = [
        (50.63, -475.8, 1.0, 92.23),
        (50.0, -560.0, -20.0, 92.0),
        (50.0, -400.0, 20.0, 92.0),
        (50.0, -480.0, 0.0, 86.0),
        (50.0, -480.0, 0.0, 97.0),
    ];

    struct Plant {
        fit: Fit,
        h: f64,
        v: f64,
        ias: f64,
        ias0: f64,
        base: f64,
        b1: f64,
        b2: f64,
        thr: f64,
        thr_cmd: f64,
        stall: f64,
        gust: f64,
        rng: u64,
        rough_fpm: f64,
        t: f64,
        written: Vec<(f64, f64)>,
    }

    impl Plant {
        /// As recorded: the autopilot's column settled in the pipe.
        fn recorded_gate(fit: Fit) -> Self {
            let (_, h, v, _, ias) = REC[0];
            let b = fit.el.0 * fit.pre;
            Plant {
                fit,
                h,
                v,
                ias,
                ias0: ias,
                base: v - b,
                b1: b,
                b2: b,
                thr: APPROACH_THR,
                thr_cmd: APPROACH_THR,
                stall: 0.0,
                gust: 0.0,
                rng: 0,
                rough_fpm: 0.0,
                t: 0.0,
                written: vec![(-10.0, fit.pre)],
            }
        }
        /// At a gate: sinking `v` with `trend`, the column that made the
        /// trend already in the pipe.
        fn at(fit: Fit, h: f64, v: f64, trend: f64, ias: f64) -> Self {
            let mut p = Self::recorded_gate(fit);
            let b = fit.el.0 * fit.pre;
            p.h = h;
            p.v = v;
            p.ias = ias;
            p.ias0 = ias;
            p.b1 = b + trend * fit.el.2;
            p.base = v - b;
            p
        }
        fn rough(mut self, seed: u64, rms_fpm: f64) -> Self {
            self.rng = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
            self.rough_fpm = rms_fpm;
            self
        }
        fn gauss(&mut self) -> f64 {
            let mut u = || {
                self.rng ^= self.rng << 13;
                self.rng ^= self.rng >> 7;
                self.rng ^= self.rng << 17;
                ((self.rng >> 11) as f64 + 0.5) / (1u64 << 53) as f64
            };
            let (a, b) = (u(), u());
            (-2.0 * a.ln()).sqrt() * (std::f64::consts::TAU * b).cos()
        }
        fn write(&mut self, e: f64) {
            self.written.push((self.t, e));
        }
        fn step(&mut self, dt: f64) {
            let (k, l1, l2, dead) = self.fit.el;
            let due = self.t - dead;
            let arriving = self
                .written
                .iter()
                .take_while(|(t, _)| *t <= due + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            self.b1 += (k * arriving - self.b1) * dt / l1;
            self.b2 += (self.b1 - self.b2) * dt / l2;
            self.thr += (self.thr_cmd - self.thr) * dt / self.fit.thr_lag;
            if self.rough_fpm > 0.0 {
                const TC: f64 = 1.5;
                let n = self.gauss();
                self.gust += -self.gust / TC * dt + self.rough_fpm * (2.0 * dt / TC).sqrt() * n;
            }
            if self.ias < VSO_KTS {
                self.stall += 800.0 * dt;
            }
            self.v = self.base
                + self.b2
                + self.fit.thr_fpm * (self.thr - APPROACH_THR)
                + self.fit.per_kt * (self.ias - self.ias0)
                + self.gust
                - self.stall;
            self.ias += (-2.70 - 5.1 * self.v / 1000.0 + 2.74 * self.thr) * dt;
            self.h += self.v / 60.0 * dt;
            self.t += dt;
        }
        /// The recorded column and lever, written as they were, to `until`.
        fn recorded(&mut self, until: f64) {
            while self.t < until - 1e-9 {
                let e = REC
                    .iter()
                    .take_while(|r| r.0 <= self.t + 1e-9)
                    .last()
                    .map_or(0.0, |r| r.3);
                if self.written.last().map(|w| w.1) != Some(e) {
                    self.write(e);
                }
                self.thr_cmd = THR
                    .iter()
                    .take_while(|r| r.0 <= self.t + 1e-9)
                    .last()
                    .map_or(APPROACH_THR, |r| r.1);
                self.step(0.01);
            }
        }
    }

    fn sr22() -> crate::aircraft::AircraftProfile {
        crate::aircraft::load("cirrus_sr22").expect("the SR22 profile loads")
    }

    /// The SR22's flare as declared from 2026-09-28 to the hold-off of
    /// 2026-09-29 (the 90 kt finals these plants were fitted to): the model
    /// 6,000 fpm through 1.0 s lags after 0.8 s, no hold-off, 0.35 of pull,
    /// the lever held to a second from the wheels. Today's hold-off is
    /// `lemg13_sr22_bounce_tests`.
    pub(super) fn flare_0928() -> crate::aircraft::FlareElevatorProfile {
        let mut fe = sr22().flare_elevator.expect("a flare_elevator");
        fe.max = 0.35;
        fe.tau_s = None;
        let mut m = fe.model.expect("a model");
        m.lag_s = 1.0;
        m.lag2_s = None;
        m.dead_s = 0.8;
        m.ground_fpm = 0.0;
        m.hold_off_ft = None;
        m.touchdown_fpm = None;
        m.fpm_per_kt = None;
        fe.model = Some(m);
        fe
    }

    pub(super) fn power_0928() -> crate::aircraft::FlarePowerProfile {
        crate::aircraft::FlarePowerProfile {
            reduction_per_second: 0.03,
            retard_below_ft: None,
            retard_max_per_s: Some(0.03),
            retard_slow_kts: None,
            retard_at_s: Some(1.0),
            retard_fast_kts: None,
        }
    }

    /// Which elevator law flies.
    enum Law {
        Model(FlareModel),
        Tracker(FlarePitch, Option<f64>),
    }

    impl Law {
        /// The 90 kt law these plants were written for ([`flare_0928`]).
        fn today() -> Self {
            Law::Model(FlareModel::from_profile(Some(flare_0928())).expect("the 09-28 model"))
        }
        /// The law the SR22 flew at LEMG 13: no `flare_elevator:`, the
        /// default tracker (the servos latched as today).
        fn as_flown() -> Self {
            Law::Tracker(FlarePitch::with_gains(FlareGains::default()), None)
        }
        fn step(&mut self, h: f64, gs: f64, v: f64, dt: f64, flare: bool, rough: bool) -> f64 {
            match self {
                Law::Model(m) => m.step(h, gs, v, dt, flare, rough).elev,
                Law::Tracker(p, prev) => {
                    let trend = prev.map_or(0.0, |p| (v - p) / dt);
                    *prev = Some(v);
                    let nominal = -(gs / 60.0) * gp_ft_per_nm();
                    let (roundout, flare_vs) = arrest_targets_fpm(nominal);
                    p.step(if flare { flare_vs } else { roundout }, v, trend, dt, flare)
                }
            }
        }
    }

    #[derive(Debug)]
    struct Landed {
        wheels_fpm: f64,
        pushed_climbing: bool,
        secs: f64,
        min_ias: f64,
    }

    /// The arrest from here to the wheels at the loop's tick, the servos
    /// kept: the flare below its gate and a flare stays a flare to the
    /// band's top (`hold_flare`), the roundout above. The lever as the loop
    /// flies it: the arrest's approach thrust, then the profile's
    /// `flare_power` (idle at the flare gate when it declares none), the
    /// approach thrust again while climbing (`balloon_power`).
    fn land(p: &mut Plant, law: &mut Law, rough: bool) -> Landed {
        land_with(p, law, rough, Some(power_0928()))
    }

    fn land_with(
        p: &mut Plant,
        law: &mut Law,
        rough: bool,
        cfg: Option<crate::aircraft::FlarePowerProfile>,
    ) -> Landed {
        let mut power = FlarePower::default();
        let start = p.t;
        let (mut next, mut flaring) = (p.t, false);
        let mut out = Landed {
            wheels_fpm: 0.0,
            pushed_climbing: false,
            secs: 0.0,
            min_ias: p.ias,
        };
        while p.h > 0.0 && p.t - start < 40.0 {
            if p.t + 1e-9 >= next {
                next += TICK_S;
                let gs = p.ias - HEADWIND_KTS;
                let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
                flaring = p.h <= flare_ft || (flaring && p.h <= band_ft);
                let e = law.step(p.h, gs, p.v, TICK_S, flaring, rough);
                out.pushed_climbing |= p.v > BALLOON_CLIMB_FPM && e < -1e-9;
                p.write(e);
                p.thr_cmd = if !flaring {
                    power.reset();
                    APPROACH_THR.min(p.thr_cmd.max(0.0))
                } else if let Some(t) = balloon_power(Some(p.v), Some(APPROACH_THR), p.h > band_ft)
                {
                    t
                } else {
                    match cfg {
                        Some(c) => power
                            .step(
                                c,
                                p.thr_cmd,
                                p.v,
                                TICK_S,
                                p.h,
                                Some(p.ias),
                                Some(APPROACH_IAS),
                            )
                            .unwrap_or(p.thr_cmd),
                        None => 0.0,
                    }
                };
            }
            p.step(0.01);
            out.min_ias = out.min_ias.min(p.ias);
        }
        out.wheels_fpm = p.v;
        out.secs = p.t - start;
        out
    }

    /// The plant is the recording: the column and lever as written give
    /// back the sink as measured, balloon included, 39 fpm rms on the whole
    /// record's fit and the wheels at -494 against the recorded -585 (the
    /// parts' fits are other flares: 50 to 160 over the whole).
    #[test]
    fn the_sr22_plant_replays_the_lemg_flare() {
        let mut p = Plant::recorded_gate(PLANTS[WHOLE]);
        let mut sq = 0.0;
        for r in REC.iter().skip(1) {
            p.recorded(r.0);
            sq += (p.v - r.2).powi(2);
        }
        let rms = (sq / (REC.len() - 1) as f64).sqrt();
        assert!(rms < 45.0, "the whole record's fit: {rms:.1} fpm rms");
        assert!((p.v - REC[REC.len() - 1].2).abs() < 100.0, "{:.0}", p.v);
    }

    /// From every gate on every plant, calm air: the SR22's declared flare
    /// (its model, the thrust held to the last second) lands under 300 fpm
    /// (the worst 273) without pushing a climb, slowing within 5 kt of Vso
    /// or floating; the same model with the lever cut at the flare gate goes
    /// past 300 somewhere; the tracker it flew goes past 400 (the worst
    /// 1,267, a float that stalled onto the runway).
    #[test]
    fn the_sr22_flare_lands_every_gate_on_every_plant() {
        let (mut idle_worst, mut tracker_worst) = (0.0_f64, 0.0_f64);
        for (h, v, trend, ias) in GATES {
            for fit in PLANTS {
                let mut p = Plant::at(fit, h, v, trend, ias);
                let l = land(&mut p, &mut Law::today(), false);
                assert!(l.wheels_fpm > -300.0, "{fit:?} from {h} ft {v} fpm: {l:?}");
                assert!(!l.pushed_climbing, "{fit:?} from {h} ft");
                assert!(l.min_ias > VSO_KTS + 5.0, "{fit:?}: {l:?}");
                // No push on a standing plant: the model floats longer than
                // flight (the King Air's 11.3 s flown against 18 modelled).
                assert!(l.secs < 16.0, "{fit:?}: floated {l:?}");
                let mut p = Plant::at(fit, h, v, trend, ias);
                idle_worst =
                    idle_worst.min(land_with(&mut p, &mut Law::today(), false, None).wheels_fpm);
                let mut p = Plant::at(fit, h, v, trend, ias);
                tracker_worst = tracker_worst
                    .min(land_with(&mut p, &mut Law::as_flown(), false, None).wheels_fpm);
            }
        }
        assert!(idle_worst < -300.0, "idle at the gate: {idle_worst:.0}");
        assert!(tracker_worst < -400.0, "as flown: {tracker_worst:.0}");
    }

    /// Rough air (40 fpm rms, where the rough-air gate starts), every gate
    /// and plant, ten gust seeds each: under 400 fpm (the worst 353), never
    /// a push while climbing.
    #[test]
    fn in_rough_air_the_sr22_flare_still_lands() {
        let (mut worst, mut n) = (0.0_f64, 0);
        for (h, v, trend, ias) in GATES {
            for fit in PLANTS {
                for seed in 1..=10u64 {
                    let mut p = Plant::at(fit, h, v, trend, ias).rough(seed, 40.0);
                    let l = land(&mut p, &mut Law::today(), true);
                    assert!(!l.pushed_climbing, "{fit:?} seed {seed}");
                    worst = worst.min(l.wheels_fpm);
                    n += 1;
                }
            }
        }
        eprintln!("worst {worst:.0} over {n}");
        assert!(worst > -400.0, "{worst:.0} over {n}");
    }
}

/// Issue #166, the BOUNCE: LEMG 13, the SR22 on the iMac (a 0.4 to 0.46 s
/// loop), three 5 nm finals at the POH's 80 kt on 2026-09-29 (sorties
/// 1790701546, 1790702352, 1790702623), the final's air 42 to 45 fpm rms.
/// Each flew the jets' firm-touchdown flare to the wheels: the target a sink
/// of 107 fpm at the runway, the lever held at approach power to a second
/// from the wheels, the IAS 80 to 81 all the way down, first contact at 207
/// to 215 fpm with 0.10 of pull on the column. Each then flew: the ratchet
/// held the 0.10 as it came back up, the balloon ease let it go to neutral,
/// the balloon's approach thrust (0.265) came back at 3 to 4 ft, and the
/// wheels met again with the pull the flare had rebuilt on the way down:
/// 4.5 to 4.9 ft, then 4.9 to 6.0, then 9.9 ft after the rollout latched on
/// a 0.115 pull at 75 kt. At neutral and idle the hops went on to 55 to 60 kt.
///
/// The plant: the standing-sink elevator of [`FlareModel`] refitted to the
/// three 80 kt flares, carried to the ground by the vertical speed through a
/// lag of its own (the lift the pull adds is an ACCELERATION: a hop needs an
/// aircraft that can move faster than its standing sink), the tail's ground
/// effect sinking it from 40 ft, the gear a soft spring and damper, and the
/// mains' contact pitching the nose up (a climb tendency in proportion to the
/// sink met up to 369 fpm, scaled by the dynamic pressure, decaying; unbound
/// it ran away), the gear bottoming at 0.4 ft. Fitted open loop, the column
/// and lever as written, to the three flares' sink before contact and their
/// first six hops after it. A fit is only as good as the model it fits: the
/// elevator's authority on the wheels and the pitch the gear makes are not
/// measured (no attitude in the autoland ticks), only their sum.
///
/// What it says: the flare as flown bounces 3.8 to 5.9 ft in every plant,
/// as it did in the sim. The hold-off (`hold_off_ft` 2 ft, `touchdown_fpm`
/// 50, `tau_s` 7, the speed's bleed led (`fpm_per_kt`), the lever to idle
/// from 20 ft, `TouchHold` from the first contact) touches once at 124 fpm
/// or less at 64 to 69 kt, the hop at most 1.2 ft. The sim decides: VERIFY
/// on the next SR22 landings `touch_hold` (held pull, at, ias), the ticks'
/// `touch_hold` and `flare_ask` below 20 ft, the `power` events' `retard`
/// (float or timed from 20 ft, then `touched`), `bounce` events (none, or one
/// spell under 1.5 ft), the landing's `sink_fpm` and `bounce_ft`.
#[cfg(test)]
mod lemg13_sr22_bounce_tests {
    use super::*;

    const FLOWN: [&str; 3] = [
        include_str!("../bench/landings/sr22_lemg13_flare_1790701546.jsonl"),
        include_str!("../bench/landings/sr22_lemg13_flare_1790702352.jsonl"),
        include_str!("../bench/landings/sr22_lemg13_flare_1790702623.jsonl"),
    ];
    /// The recorded first hops over the runway (ft): the peak between the
    /// first contact and the second, and between the second and the third.
    const FLOWN_HOPS: [(f64, f64); 3] = [(4.9, 5.6), (4.9, 6.0), (4.5, 4.9)];

    /// The loop's tick on the iMac.
    const TICK_S: f64 = 0.45;
    /// The SR22's landing-configuration stall (POH `vs0_kts`): the plant
    /// sinks past it.
    const VSO_KTS: f64 = 60.0;
    /// The SR22 at rest reads this on its radio altimeter, ft.
    const RADIO_AT_REST_FT: f64 = 0.15;
    /// On the wheels the rollout's brakes and the rolling drag, kt/s (the
    /// recorded rollouts: 3 to 5).
    const ROLL_DECEL_KTS_S: f64 = 3.0;
    /// The gear's travel, ft.
    const GEAR_TRAVEL_FT: f64 = 0.4;

    /// One recorded flare: the arrest's ticks from the servos taken (s,
    /// radio ft, vvi fpm, IAS or the rollout's groundspeed kt), the elevator
    /// as written (s, ratio) and the lever as commanded (s, ratio).
    struct Flown {
        ticks: Vec<(f64, f64, Option<f64>, f64, bool)>,
        writes: Vec<(f64, f64)>,
        levers: Vec<(f64, f64)>,
    }

    fn flown(src: &str) -> Flown {
        let rows: Vec<serde_json::Value> = src
            .lines()
            .filter(|l| !l.trim().is_empty())
            .map(|l| serde_json::from_str(l).expect("a record"))
            .collect();
        let t0 = rows
            .iter()
            .find(|r| r["event"] == "arrest_hands")
            .and_then(|r| r["t"].as_f64())
            .expect("the arrest took the servos");
        let mut out = Flown {
            ticks: vec![],
            writes: vec![],
            levers: vec![],
        };
        for r in &rows {
            let t = r["t"].as_f64().unwrap() - t0;
            match r["event"].as_str() {
                None => {
                    out.ticks.push((
                        t,
                        r["radio"].as_f64().unwrap(),
                        r["vvi"].as_f64(),
                        r["ias"].as_f64().unwrap(),
                        false,
                    ));
                    if let Some(e) = r["flare_elev"].as_f64() {
                        out.writes.push((t, e));
                    }
                }
                Some("power") => {
                    if let Some(l) = r["throttle_command"].as_f64() {
                        out.levers.push((t, l));
                    }
                }
                Some("rollout") => {
                    out.ticks.push((
                        t,
                        r["radio"].as_f64().unwrap(),
                        None,
                        r["gs"].as_f64().unwrap(),
                        true,
                    ));
                    out.levers.push((t, 0.0));
                }
                _ => {}
            }
        }
        out
    }

    /// The plant's numbers.
    #[derive(Debug, Clone, Copy)]
    struct Fit {
        /// Standing sink per unit of elevator (fpm), its two equal lags (s)
        /// and the dead time (s).
        k: f64,
        lag: f64,
        dead: f64,
        /// The tail's ground effect: sink at the runway (fpm), from 40 ft.
        tail_fpm: f64,
        /// fpm of sink per knot lost from the gate.
        per_kt: f64,
        /// The vertical speed's own lag toward the standing sink (s).
        path_s: f64,
        /// The wing's ground cushion: climb at the runway (fpm) fading by
        /// this height (ft).
        wing_fpm: f64,
        wing_ft: f64,
        /// The gear: spring (ft/s^2 per ft) and damper (per s).
        spring: f64,
        damper: f64,
        /// The mains' contact: climb tendency per fpm of sink met, and its
        /// decay (s).
        kick: f64,
        kick_s: f64,
        /// ...met up to this sink (fpm), and scaled by the dynamic pressure
        /// against 80 kt: the nose the gear lifts is worth what the wing
        /// makes of it.
        kick_cap: f64,
        /// The vertical-speed gauge the laws read lags the flight path by
        /// this (s); 0 reads the path itself.
        gauge_s: f64,
        /// The column's neutral as the plant feels it, added to what is
        /// written (the trim the servos left).
        off: f64,
        /// Sink per unit of lever (fpm) and the lever's spool lag (s).
        thr_fpm: f64,
        thr_lag: f64,
        /// The speed model, kt/s: -drag0 - drag_w x climb (per 1,000 fpm)
        /// + thrust x lever; and the stall (kt) the plant sinks past.
        drag0: f64,
        drag_w: f64,
        thrust: f64,
        vso: f64,
    }

    /// The best fit (the three flares' sink before contact, 39 to 63 fpm
    /// rms, and their first six hops): a soft gear and a strong nose-up
    /// kick. Replayed, the first hops are 5.8, 5.0 and 6.0 ft against 4.9,
    /// 4.9 and 4.5; closed loop, the law as flown bounces 5.9 ft on it.
    const FIT: Fit = Fit {
        k: 6170.0,
        lag: 1.42,
        dead: 1.19,
        tail_fpm: 380.0,
        per_kt: 122.0,
        path_s: 0.34,
        wing_fpm: 85.0,
        wing_ft: 4.9,
        spring: 9.3,
        damper: 2.7,
        kick: 4.95,
        kick_s: 1.38,
        kick_cap: 369.0,
        gauge_s: 0.0,
        off: 0.0,
        thr_fpm: 316.0,
        thr_lag: 2.7,
        drag0: 2.70,
        drag_w: 5.1,
        thrust: 2.74,
        vso: VSO_KTS,
    };
    /// The plants that replay the records (`the_plant_replays_the_80_kt_bounces`):
    /// the best fit, its dead time 0.2 s shorter (3.8, 6.5, 3.6 ft) and its
    /// kick a fifth weaker (3.9, 3.1, 4.1). A law that lands on all of them
    /// does not depend on one number being right. The elevator 15 % weaker
    /// or stronger, the tail's ground effect 40 % either way, the speed's
    /// sink 30 % either way, or another fit (a heavy vertical and a stiff
    /// gear) do not replay the three flares (first hops of 1.1 to 36 ft)
    /// and are not plants of this aircraft.
    fn plants() -> Vec<Fit> {
        vec![
            FIT,
            Fit {
                dead: FIT.dead - 0.2,
                ..FIT
            },
            Fit {
                kick: FIT.kick * 0.8,
                ..FIT
            },
        ]
    }

    pub(super) struct Plant {
        fit: Fit,
        t: f64,
        h: f64,
        /// Vertical speed, fpm.
        w: f64,
        /// The gauge's vertical speed, fpm ([`Fit::gauge_s`]).
        g: f64,
        ias: f64,
        ias0: f64,
        base: f64,
        b1: f64,
        b2: f64,
        thr: f64,
        thr0: f64,
        thr_cmd: f64,
        kick: f64,
        stall: f64,
        /// The elevator as written, with the plugin's 1 s expiry.
        written: Vec<(f64, f64)>,
        /// Recorded IAS to follow (a replay) instead of the speed model.
        ias_track: Option<Vec<(f64, f64)>>,
        on_wheels: bool,
        /// Every contact: (time, sink met fpm, IAS).
        contacts: Vec<(f64, f64, f64)>,
        /// Highest point after the first contact.
        bounce_ft: f64,
        /// The gauge at the first contact (the scorer's sink), fpm, and the
        /// ground covered from the gate to it, m.
        first_gauge: Option<f64>,
        along_m: f64,
    }

    impl Plant {
        /// At the arrest gate, sinking steadily with `pipe` on the column
        /// and the lever holding the speed.
        fn at(fit: Fit, h: f64, vvi: f64, ias: f64, pipe: f64) -> Self {
            let b = fit.k * (pipe + fit.off);
            // The lever the speed model holds this sink on.
            let thr0 = ((fit.drag0 + fit.drag_w * vvi / 1000.0) / fit.thrust).clamp(0.0, 1.0);
            let mut p = Plant {
                fit,
                t: 0.0,
                h,
                w: vvi,
                g: vvi,
                ias,
                ias0: ias,
                base: 0.0,
                b1: b,
                b2: b,
                thr: thr0,
                thr0,
                thr_cmd: thr0,
                kick: 0.0,
                stall: 0.0,
                written: vec![(-10.0, pipe)],
                ias_track: None,
                on_wheels: false,
                contacts: vec![],
                bounce_ft: 0.0,
                first_gauge: None,
                along_m: 0.0,
            };
            p.base = vvi - b - p.ground(h);
            p
        }

        fn ground(&self, h: f64) -> f64 {
            let h = h.max(0.0);
            -self.fit.tail_fpm * (1.0 - h / 40.0).max(0.0)
                + self.fit.wing_fpm * (1.0 - h / self.fit.wing_ft).max(0.0)
        }

        fn write(&mut self, e: f64) {
            self.written.push((self.t, e));
        }

        fn arriving(&self) -> f64 {
            let due = self.t - self.fit.dead;
            self.fit.off
                + match self.written.iter().take_while(|(t, _)| *t <= due + 1e-9).last() {
                    // The autopilot's column at the gate never expires.
                    Some((tw, e)) if due - tw <= 1.0 || *tw < -1.0 => *e,
                    _ => 0.0,
                }
        }

        fn standing(&self) -> f64 {
            self.base
                + self.b2
                + self.fit.thr_fpm * (self.thr - self.thr0)
                + self.fit.per_kt * (self.ias - self.ias0)
                + self.ground(self.h)
                + self.kick
                - self.stall
        }

        fn step(&mut self, dt: f64) {
            let f = self.fit;
            let e = self.arriving();
            self.b1 += (f.k * e - self.b1) * dt / f.lag;
            self.b2 += (self.b1 - self.b2) * dt / f.lag;
            self.thr += (self.thr_cmd - self.thr) * dt / f.thr_lag;
            self.kick -= self.kick * dt / f.kick_s;
            match &self.ias_track {
                Some(tr) => {
                    self.ias = tr
                        .iter()
                        .take_while(|(t, _)| *t <= self.t)
                        .last()
                        .map_or(self.ias0, |x| x.1)
                }
                None => {
                    self.ias += if self.on_wheels {
                        -ROLL_DECEL_KTS_S
                    } else {
                        -f.drag0 - f.drag_w * self.w / 1000.0 + f.thrust * self.thr
                    } * dt
                }
            }
            if self.ias < f.vso && !self.on_wheels {
                self.stall += 800.0 * dt;
            }
            let mut a = (self.standing() - self.w) / f.path_s;
            if self.h < 0.0 {
                let g = (f.spring * -self.h - f.damper * self.w / 60.0) * 60.0;
                if g > 0.0 {
                    a += g;
                }
            }
            self.w += a * dt;
            self.g = if f.gauge_s > 0.0 {
                self.g + (self.w - self.g) * dt / f.gauge_s
            } else {
                self.w
            };
            let was = self.h;
            self.h += self.w / 60.0 * dt;
            // The gear bottoms out: the soft spring the hops were fitted
            // with is only the first few inches of it.
            if self.h < -GEAR_TRAVEL_FT {
                self.h = -GEAR_TRAVEL_FT;
                self.w = self.w.max(0.0);
            }
            if was >= 0.0 && self.h < 0.0 {
                self.kick +=
                    f.kick * (-self.w).max(0.0).min(f.kick_cap) * (self.ias / 80.0).powi(2);
                self.contacts.push((self.t, self.w, self.ias));
                self.first_gauge.get_or_insert(self.g);
            }
            self.on_wheels = self.h <= 0.0;
            if !self.contacts.is_empty() {
                self.bounce_ft = self.bounce_ft.max(self.h);
            }
            if self.contacts.is_empty() {
                self.along_m += self.ias / 1.943_844 * dt;
            }
            self.t += dt;
        }

        fn radio(&self) -> f64 {
            self.h.max(0.0) + RADIO_AT_REST_FT
        }

        /// The vertical speed the laws read: the gauge.
        fn vvi(&self) -> f64 {
            self.g
        }
    }

    /// The hops of a height series: the peak between each contact and the
    /// next (a contact is the radio under 0.8 ft).
    fn hops(series: &[(f64, f64)]) -> Vec<f64> {
        let mut out = vec![];
        let mut i = 0;
        while i < series.len() {
            if series[i].1 < 0.8 {
                while i < series.len() && series[i].1 < 0.8 {
                    i += 1;
                }
                let mut peak = 0.0_f64;
                let start = i;
                while i < series.len() && series[i].1 >= 0.8 {
                    peak = peak.max(series[i].1);
                    i += 1;
                }
                if i > start && i < series.len() {
                    out.push(peak);
                }
            } else {
                i += 1;
            }
        }
        out
    }

    /// The recorded column and lever, open loop, from the recorded gate: the
    /// plant's radio at every recorded tick.
    fn replay(f: &Flown, fit: Fit) -> Vec<(f64, f64)> {
        let (_, h, v, ias, _) = f.ticks[0];
        let mut p = Plant::at(fit, h, v.unwrap(), ias, f.writes[0].1);
        p.ias_track = Some(f.ticks.iter().map(|x| (x.0, x.3)).collect());
        p.thr0 = f.levers.first().map_or(p.thr0, |l| l.1);
        p.thr = p.thr0;
        p.written = vec![(-10.0, f.writes[0].1)];
        let mut out = vec![];
        let (mut wi, mut li) = (0, 0);
        for &(t, ..) in &f.ticks {
            while p.t < t - 1e-9 {
                while wi < f.writes.len() && f.writes[wi].0 <= p.t + 1e-9 {
                    let e = f.writes[wi].1;
                    p.write(e);
                    wi += 1;
                }
                while li < f.levers.len() && f.levers[li].0 <= p.t + 1e-9 {
                    p.thr_cmd = f.levers[li].1;
                    li += 1;
                }
                p.step(0.01);
            }
            out.push((t, p.radio()));
        }
        out
    }

    /// The loop's laws over the plant, from a gate to the wheels and on to
    /// `until` s. `hold_off`: today's SR22 (its profile); else the flare as
    /// flown at 80 kt (no hold-off, the lever to a second from the wheels,
    /// the rollout letting the channel go).
    #[derive(Debug)]
    struct Landing {
        first_fpm: f64,
        first_ias: f64,
        contacts: usize,
        bounce_ft: f64,
        min_ias: f64,
        secs_to_wheels: f64,
    }

    fn sr22() -> crate::aircraft::AircraftProfile {
        crate::aircraft::load("cirrus_sr22").expect("the SR22 profile loads")
    }

    /// The profile as flown on 2026-09-29 at 80 kt: the model without the
    /// hold-off, the lever held to a second from the wheels.
    fn as_flown() -> (FlareModel, crate::aircraft::FlarePowerProfile) {
        (
            FlareModel::from_profile(Some(super::lemg13_sr22_flare_tests::flare_0928()))
                .expect("the 09-28 model"),
            super::lemg13_sr22_flare_tests::power_0928(),
        )
    }

    fn today() -> (FlareModel, crate::aircraft::FlarePowerProfile) {
        let p = sr22();
        (
            FlareModel::from_profile(p.flare_elevator).expect("the SR22 declares its model"),
            p.flare_power.expect("the SR22 declares its retard"),
        )
    }

    fn land(
        p: &mut Plant,
        model: &mut FlareModel,
        power_cfg: crate::aircraft::FlarePowerProfile,
        approach_ias: f64,
        rough: bool,
        until: f64,
    ) -> Landing {
        let holds_off = model.holds_off();
        let mut power = FlarePower::default();
        let mut touch = TouchHold::default();
        let mut last_elev: Option<f64> = None;
        let (mut next, mut flaring, mut rolling, mut wheels_on) = (0.0, false, false, None::<f64>);
        let mut min_ias = p.ias;
        let approach_thr = p.thr0;
        while p.t < until {
            if p.t + 1e-9 >= next {
                next += TICK_S;
                let gs = p.ias;
                let radio = p.radio();
                let on_ground = p.on_wheels;
                wheels_on = wheels_on_time(wheels_on, on_ground, radio, TICK_S);
                if should_start_rollout(rolling, on_ground, radio, gs, p.ias, approach_ias, wheels_on) {
                    if !rolling && holds_off {
                        touch.note(true, last_elev);
                    }
                    rolling = true;
                }
                if rolling {
                    p.thr_cmd = 0.0;
                    if let Some(e) = touch.elev(gs, on_ground, TICK_S) {
                        p.write(e);
                    }
                } else {
                    let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
                    flaring = radio <= flare_ft || (flaring && radio <= band_ft);
                    if holds_off && flaring {
                        touch.note(on_ground || radio < TOUCH_RADIO_FT, last_elev);
                    }
                    model.note_speed(p.ias, TICK_S);
                    let o = model.step(radio, gs, p.vvi(), TICK_S, flaring, rough);
                    let e = if touch.touched() {
                        touch.elev(gs, on_ground, TICK_S)
                    } else {
                        Some(o.elev)
                    };
                    if let Some(e) = e {
                        p.write(e);
                        last_elev = Some(e);
                    }
                    p.thr_cmd = if touch.touched() {
                        0.0
                    } else if !flaring {
                        power.reset();
                        approach_thr
                    } else if let Some(t) =
                        balloon_power(Some(p.vvi()), Some(approach_thr), radio > band_ft)
                    {
                        t
                    } else {
                        power
                            .step(power_cfg, p.thr_cmd, p.vvi(), TICK_S, radio, Some(p.ias), Some(approach_ias))
                            .unwrap_or(p.thr_cmd)
                    };
                }
            }
            p.step(0.01);
            if p.contacts.is_empty() {
                min_ias = min_ias.min(p.ias);
            }
        }
        let first = p.contacts.first().copied().unwrap_or((f64::NAN, f64::NAN, f64::NAN));
        Landing {
            first_fpm: -first.1,
            first_ias: first.2,
            contacts: p.contacts.len(),
            bounce_ft: p.bounce_ft,
            min_ias,
            secs_to_wheels: first.0,
        }
    }

    /// The plant is the recording: the three flares' column and lever as
    /// written, open loop, give back their first hop within 2 ft on every
    /// plant (4.9, 4.9 and 4.5 recorded).
    #[test]
    fn the_plant_replays_the_80_kt_bounces() {
        for fit in plants() {
            for (src, want) in FLOWN.iter().zip(FLOWN_HOPS) {
                let f = flown(src);
                let got = hops(&replay(&f, fit));
                assert!(!got.is_empty(), "{fit:?}: no hop");
                assert!((got[0] - want.0).abs() < 2.0, "{fit:?}: {got:?} against {want:?}");
            }
        }
    }

    /// Gates at 75 kt (the final since 6c4276f): the recorded three scaled to
    /// the slower path, a little steeper and shallower, faster and slower.
    const GATES_75: [(f64, f64, f64, f64); 5] = [
        (49.0, -356.0, 75.0, -0.027),
        (49.0, -420.0, 75.0, -0.02),
        (49.0, -300.0, 75.0, -0.03),
        (46.0, -356.0, 77.0, -0.027),
        (50.0, -356.0, 73.0, -0.027),
    ];

    /// The flare as flown (the 80 kt law, `lemg13_sr22_flare_tests::
    /// flare_0928`), closed loop from the three recorded gates, bounces on
    /// every plant as it did in the sim (3.8 to 5.9 ft). The hold-off from
    /// every 75 kt gate on every plant, calm and rough: one contact under 250
    /// fpm near the stall (57 to 70 kt), never over 1.5 ft after it, never
    /// slower than 57 kt before it (the best fit: 124 fpm at 64.5 to 68.5
    /// kt, 1.2 ft), and every plant's hold-off under a third of its flare
    /// as flown.
    #[test]
    fn the_hold_off_touches_once_where_the_flare_as_flown_bounced() {
        let mut worst = (0.0_f64, 0.0_f64);
        for fit in plants() {
            let mut flown_worst = 0.0_f64;
            for src in FLOWN {
                let f = flown(src);
                let (_, h, v, ias, _) = f.ticks[0];
                let mut p = Plant::at(fit, h, v.unwrap(), ias, f.writes[0].1);
                let (mut m, pw) = as_flown();
                flown_worst = flown_worst.max(land(&mut p, &mut m, pw, 80.0, true, 30.0).bounce_ft);
            }
            assert!(flown_worst > 3.0, "{fit:?} as flown: {flown_worst:.2} ft");
            let mut held = 0.0_f64;
            for (h, v, ias, pipe) in GATES_75 {
                for rough in [false, true] {
                    let mut p = Plant::at(fit, h, v, ias, pipe);
                    let (mut m, pw) = today();
                    let l = land(&mut p, &mut m, pw, 75.0, rough, 40.0);
                    assert!(l.first_fpm < 250.0, "{fit:?} {h} {v} {ias}: {l:?}");
                    assert!(l.bounce_ft < 1.5, "{fit:?} {h} {v} {ias}: {l:?}");
                    assert!((57.0..=70.0).contains(&l.first_ias), "{fit:?}: {l:?}");
                    assert!(l.min_ias > 57.0, "{fit:?}: {l:?}");
                    held = held.max(l.bounce_ft);
                    worst = (worst.0.max(l.first_fpm), worst.1.max(l.bounce_ft));
                }
            }
            assert!(held < flown_worst / 3.0, "{fit:?}: {held:.2} against {flown_worst:.2}");
        }
        eprintln!("held off: worst {:.0} fpm, {:.2} ft", worst.0, worst.1);
    }

    /// Held on: the pull it touched with, never more and never less, the
    /// lever closed; let go only on the wheels below the release speed.
    #[test]
    fn touch_hold_keeps_the_pull_it_touched_with() {
        let mut t = TouchHold::default();
        assert!(!t.note(false, Some(0.3)), "no contact");
        assert_eq!(t.elev(70.0, false, 0.45), None, "nothing held before a contact");
        assert!(t.note(true, Some(0.31)));
        assert!(!t.note(true, Some(0.5)), "the first contact's pull, once");
        assert_eq!(t.elev(66.0, true, 0.45), Some(0.31));
        assert_eq!(t.elev(64.0, false, 0.45), Some(0.31), "a hop keeps it");
        let e = t.elev(40.0, true, 1.0).unwrap();
        assert!((e - (0.31 - TOUCH_HOLD_EASE_PER_S)).abs() < 1e-9, "{e}");
        for _ in 0..20 {
            t.elev(30.0, true, 1.0);
        }
        assert_eq!(t.elev(30.0, true, 1.0), None, "let go at neutral");
        let mut push = TouchHold::default();
        push.note(true, Some(-0.04));
        assert_eq!(push.elev(70.0, true, 0.45), Some(0.0), "never a push");
    }

    /// Only an airframe that declares the hold-off flies it: the 737, the
    /// A330 and the SF50 keep their firm-touchdown flares.
    #[test]
    fn only_the_sr22_holds_off() {
        for id in ["boeing_737_800", "airbus_a330_300", "cirrus_vision_sf50"] {
            let p = crate::aircraft::load(id).expect("the profile loads");
            let m = FlareModel::from_profile(p.flare_elevator).expect("a model");
            assert!(!m.holds_off(), "{id}");
        }
        let (m, _) = today();
        assert!(m.holds_off());
    }

    /// The Lancair Evolution's flare at LILB 03 (iMac 2026-09-30, sortie
    /// 1790740292-full_auto, `bench/landings/evolution_lilb03_flare_1790740292.jsonl`):
    /// the only landing it has flown under the autoland, on the jets'
    /// default tracker (no `flare_elevator:`). The gauge the laws read
    /// (`vvi_fpm_pilot`) lags the flight path by about a second on this
    /// airframe: at 9 ft the height had been level for a second while the
    /// gauge still read -386 to -82, the tracker's balloon lead let the
    /// whole 0.35 of pull go at once, and the aircraft dropped 9 ft onto the
    /// runway (the gauge read 392 fpm at the contact, the height fell at
    /// about 800). The same let-go at 24 ft had ballooned it to 32 ft on the
    /// approach thrust the balloon rule gave back.
    mod lilb03_evolution {
        use super::*;

        const SRC: &str = include_str!("../bench/landings/evolution_lilb03_flare_1790740292.jsonl");

        /// Fitted to the recorded flare from the servos taken to the
        /// contact (radio 0.66 ft rms, gauge 43 fpm rms): 6,450 fpm per unit
        /// through 0.29 s lags after 0.23 s, the path's own 1.5 s, the
        /// gauge 0.97 s behind it, 152 fpm per knot, a wing cushion of 350
        /// fpm under 3.6 ft, no tail effect, the column's neutral 0.084 of
        /// pull (the trim), the lever worth nothing but its speed. The gear
        /// is the SR22's (one contact is no fit).
        pub(super) const EVO: Fit = Fit {
            k: 6453.0,
            lag: 0.29,
            dead: 0.23,
            tail_fpm: 0.0,
            per_kt: 152.0,
            path_s: 1.5,
            wing_fpm: 351.0,
            wing_ft: 3.65,
            gauge_s: 0.965,
            off: 0.084,
            thr_fpm: 1.6,
            thr_lag: 0.6,
            drag0: 2.0,
            drag_w: 1.53,
            thrust: 3.3,
            vso: 61.0,
            ..FIT
        };
        /// The column the autopilot held at the gate, less the neutral.
        const PIPE: f64 = 0.071 - 0.084;

        /// The fit and its neighbours that still replay the flare (2 ft,
        /// 60 fpm): the gauge 20 % either way, the dead time 0.1 s either
        /// way, the path 20 % either way, and without the wing's cushion,
        /// which the flare as flown never reached and so does not fix. The
        /// elevator 5 % either way (2.3, 2.8 ft) or the speed's sink 30 %
        /// (3.2, 8.4 ft) do not replay it.
        fn evo_plants() -> Vec<Fit> {
            vec![
                EVO,
                Fit { wing_fpm: 0.0, ..EVO },
                Fit { gauge_s: EVO.gauge_s * 0.8, ..EVO },
                Fit { gauge_s: EVO.gauge_s * 1.2, wing_fpm: 0.0, ..EVO },
                Fit { dead: EVO.dead + 0.1, ..EVO },
                Fit { dead: EVO.dead - 0.1, wing_fpm: 0.0, ..EVO },
                Fit { path_s: EVO.path_s * 0.8, ..EVO },
                Fit { path_s: EVO.path_s * 1.2, wing_fpm: 0.0, ..EVO },
            ]
        }

        /// The recorded column and lever, open loop: (t, radio, gauge) at
        /// every recorded tick before the contact.
        fn evo_replay(f: &Flown, fit: Fit) -> Vec<(f64, f64, f64)> {
            let (_, h, v, ias, _) = f.ticks[0];
            let mut p = Plant::at(fit, h, v.unwrap(), ias, PIPE);
            p.ias_track = Some(f.ticks.iter().map(|x| (x.0, x.3)).collect());
            p.thr0 = f.levers.first().map_or(p.thr0, |l| l.1);
            p.thr = p.thr0;
            p.thr_cmd = p.thr0;
            let mut out = vec![];
            let (mut wi, mut li) = (0, 0);
            for &(t, _, _, _, rolling) in &f.ticks {
                if rolling {
                    break;
                }
                while p.t < t - 1e-9 {
                    while wi < f.writes.len() && f.writes[wi].0 <= p.t + 1e-9 {
                        let e = f.writes[wi].1;
                        p.write(e);
                        wi += 1;
                    }
                    while li < f.levers.len() && f.levers[li].0 <= p.t + 1e-9 {
                        p.thr_cmd = f.levers[li].1;
                        li += 1;
                    }
                    p.step(0.01);
                }
                out.push((t, p.radio(), p.vvi()));
            }
            out
        }

        /// The plant is the recording: the column and lever as written,
        /// open loop, give back the radio and the gauge at every tick to
        /// the contact on every plant kept.
        #[test]
        fn the_evolution_plant_replays_its_flare() {
            let f = flown(SRC);
            let rec: Vec<_> = f.ticks.iter().filter(|x| !x.4).collect();
            for fit in evo_plants() {
                let got = evo_replay(&f, fit);
                let n = got.len().min(rec.len()) as f64;
                let (mut dh, mut dv) = (0.0, 0.0);
                for (g, r) in got.iter().zip(&rec) {
                    dh += (g.1 - r.1).powi(2);
                    dv += (g.2 - r.2.unwrap()).powi(2);
                }
                let (dh, dv) = ((dh / n).sqrt(), (dv / n).sqrt());
                eprintln!("replay {:.0} fpm/unit gauge {:.2} s dead {:.2}: {dh:.2} ft, {dv:.0} fpm rms", fit.k, fit.gauge_s, fit.dead);
                assert!(dh < 2.0 && dv < 60.0, "{fit:?}: {dh:.2} ft, {dv:.0} fpm");
            }
        }

        /// The loop's default tracker as it flew the Evolution: the arrest's
        /// target above the flare gate, the flare's below it, the ratchet in
        /// the flare, the lever idle there but the approach's own on a
        /// balloon, the channel let go on the wheels.
        pub(super) fn land_tracker(p: &mut Plant, gains: FlareGains, until: f64) -> Landing {
            let mut law = FlarePitch::with_gains(gains);
            let (mut next, mut flaring, mut rolling, mut wheels_on) = (0.0, false, false, None::<f64>);
            let mut prev: Option<f64> = None;
            let mut min_ias = p.ias;
            let approach_thr = p.thr0;
            while p.t < until {
                if p.t + 1e-9 >= next {
                    next += TICK_S;
                    let (gs, radio, vvi) = (p.ias, p.radio(), p.vvi());
                    wheels_on = wheels_on_time(wheels_on, p.on_wheels, radio, TICK_S);
                    if should_start_rollout(rolling, p.on_wheels, radio, gs, p.ias, 90.0, wheels_on) {
                        rolling = true;
                    }
                    let trend = prev.map_or(0.0, |v| (vvi - v) / TICK_S);
                    prev = Some(vvi);
                    if rolling {
                        p.thr_cmd = 0.0;
                    } else {
                        let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
                        flaring = radio <= flare_ft || (flaring && radio <= band_ft);
                        let nominal = -(gs / 60.0) * gp_ft_per_nm();
                        let (roundout, flare_vs) = arrest_targets_fpm(nominal);
                        let target = if flaring { flare_vs } else { roundout };
                        let e = law.step(target, vvi, trend, TICK_S, flaring);
                        p.write(e);
                        p.thr_cmd = if !flaring {
                            approach_thr
                        } else {
                            balloon_power(Some(vvi), Some(approach_thr), radio > band_ft).unwrap_or(0.0)
                        };
                    }
                }
                p.step(0.01);
                if p.contacts.is_empty() {
                    min_ias = min_ias.min(p.ias);
                }
            }
            let first = p.contacts.first().copied().unwrap_or((f64::NAN, f64::NAN, f64::NAN));
            Landing {
                first_fpm: -first.1,
                first_ias: first.2,
                contacts: p.contacts.len(),
                bounce_ft: p.bounce_ft,
                min_ias,
                secs_to_wheels: first.0,
            }
        }

        fn evolution() -> crate::aircraft::AircraftProfile {
            crate::aircraft::load("lancair_evolution").expect("the Evolution profile loads")
        }

        /// The gate as recorded (the servos taken at 50.2 ft, -404 fpm on
        /// the gauge, 90.1 KIAS on the POM's 90 kt final) and its
        /// neighbours: steeper and shallower, faster and slower, lower.
        const GATES: [(f64, f64, f64); 5] = [
            (50.2, -404.0, 90.1),
            (50.0, -470.0, 90.0),
            (50.0, -340.0, 90.0),
            (50.0, -404.0, 94.0),
            (46.0, -404.0, 87.0),
        ];

        /// The tracker as flown drops in on the plant from the recorded
        /// gate, as it did (the gauge's 392 at the wheels, the height
        /// falling at 800 or more); the declared model flare
        /// (`flare_elevator` model, `flare_power`) from every gate on every
        /// plant that replays, calm and rough: one contact under 250 fpm on
        /// the flight path and on the gauge, under 1.5 ft after it, and no
        /// longer a float than the tracker's as flown (the flown contact came
        /// 13.6 s and about 580 m past the gate on a 601 m strip, so a float
        /// to the stall from the POM's 90 kt final is no answer: no hold-off).
        #[test]
        fn the_evolution_flares_where_the_tracker_dropped_it_in() {
            let mut flown_worst = 0.0_f64;
            for fit in evo_plants() {
                let (h, v, ias) = GATES[0];
                let mut p = Plant::at(fit, h, v, ias, PIPE);
                let l = land_tracker(&mut p, FlareGains::default(), 30.0);
                eprintln!("tracker as flown, {:.0} fpm/unit gauge {:.2}: {l:?} {:.0} m", fit.k, fit.gauge_s, p.along_m);
                flown_worst = flown_worst.max(l.first_fpm);
            }
            assert!(flown_worst > 400.0, "the tracker as flown: {flown_worst:.0} fpm");
            let prof = evolution();
            let power = prof.flare_power.expect("the Evolution declares its retard");
            let mut worst = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
            for fit in evo_plants() {
                for (h, v, ias) in GATES {
                    for rough in [false, true] {
                        let mut p = Plant::at(fit, h, v, ias, PIPE);
                        let mut m = FlareModel::from_profile(prof.flare_elevator).expect("a model");
                        assert!(!m.holds_off());
                        let l = land(&mut p, &mut m, power, 90.0, rough, 40.0);
                        let gauge = -p.first_gauge.unwrap_or(f64::NAN);
                        let why = format!(
                            "{:.0} fpm/unit gauge {:.2} dead {:.2} path {:.1} wing {:.0}, {h} {v} {ias} rough {rough}: {l:?}, gauge {gauge:.0}, {:.0} m",
                            fit.k, fit.gauge_s, fit.dead, fit.path_s, fit.wing_fpm, p.along_m
                        );
                        assert!(l.first_fpm < 250.0 && gauge < 250.0, "{why}");
                        assert!(l.bounce_ft < 1.5, "{why}");
                        assert!(l.min_ias > fit.vso, "{why}");
                        // No push: the model's float (the King Air's note).
                        assert!(p.along_m < 700.0, "{why}");
                        worst = (worst.0.max(l.first_fpm), worst.1.max(gauge), worst.2.max(l.bounce_ft), worst.3.max(p.along_m));
                    }
                }
            }
            eprintln!(
                "Evolution flare: worst {:.0} fpm on the path, {:.0} on the gauge, {:.2} ft after, {:.0} m past the gate",
                worst.0, worst.1, worst.2, worst.3
            );
        }
    }

    /// The King Air C90B's flares at LEMG 13 (iMac 2026-09-30, issue #197,
    /// sorties 1790757702, 1790758429 and 1790758604 from 4 nm,
    /// `bench/landings/king_air_lemg13_landing_*.jsonl`), on the jets'
    /// default tracker (no `flare_elevator:`). Each arrest took the servos at
    /// 59 to 61 ft sinking 640 to 700 fpm at 110 to 114 KIAS; the tracker's
    /// 0.23 to 0.35 of pull ballooned it (to 61 ft level, then 17.5 to 39 ft
    /// on the balloon's approach thrust), let go to neutral, and the sink it
    /// rebuilt met the runway at 730 to 960 fpm on the gauge the tick before
    /// the wheels (the scorer's 256 to 518, read from 3 s samples after it).
    mod lemg13_king_air {
        use super::*;

        const SRC: [&str; 3] = [
            include_str!("../bench/landings/king_air_lemg13_landing_1790757702.jsonl"),
            include_str!("../bench/landings/king_air_lemg13_landing_1790758429.jsonl"),
            include_str!("../bench/landings/king_air_lemg13_landing_1790758604.jsonl"),
        ];

        /// The autopilot's column at each gate (fitted with the plant).
        const PIPES: [f64; 3] = [-0.106, -0.049, -0.112];

        /// Fitted to the three flares from the servos taken to the contact
        /// (radio 1.9 to 2.1 ft rms, gauge 50 to 56 fpm rms): 9,670 fpm per
        /// unit after 0.8 s of dead time, the path's own 2.6 s, the gauge
        /// 0.88 s behind it, 118 fpm of sink per knot; no tail effect and the
        /// lever worth nothing but its speed (the fit puts both at zero); the
        /// speed -2.72 kt/s, 2.22 more per 1,000 fpm of climb, 1.93 per unit
        /// of lever (0.66 kt/s rms). The gear is the SR22's (a contact is no
        /// fit), the stall the POM's 76 kt.
        pub(super) const KA: Fit = Fit {
            k: 9672.0,
            lag: 0.07,
            dead: 0.8,
            tail_fpm: 0.0,
            per_kt: 118.0,
            path_s: 2.58,
            wing_fpm: 0.0,
            wing_ft: 5.0,
            gauge_s: 0.875,
            off: 0.0,
            thr_fpm: 0.0,
            thr_lag: 2.0,
            drag0: 2.72,
            drag_w: 2.22,
            thrust: 1.93,
            vso: 76.0,
            ..FIT
        };

        /// The fit and neighbours that still replay the flares (4.5 ft, 95
        /// fpm): the dead time 0.1 s either way, the path 10 % and the gauge
        /// 20 % either way, a tail effect of 150 fpm, a wing cushion the
        /// flares never reached. The elevator 5 % either way (7 to 11 ft),
        /// the speed's sink 20 % either way (14 to 23 ft) or the tail's 320
        /// fpm (4 to 5 ft, 65 to 83 fpm) do not replay them.
        fn ka_plants() -> Vec<Fit> {
            vec![
                KA,
                Fit { dead: KA.dead + 0.1, ..KA },
                Fit { dead: KA.dead - 0.1, ..KA },
                Fit { path_s: KA.path_s * 1.1, ..KA },
                Fit { path_s: KA.path_s * 0.9, ..KA },
                Fit { gauge_s: KA.gauge_s * 1.2, ..KA },
                Fit { gauge_s: KA.gauge_s * 0.8, ..KA },
                Fit { tail_fpm: 150.0, ..KA },
                Fit { wing_fpm: 250.0, wing_ft: 6.0, ..KA },
            ]
        }

        /// The recorded column and lever, open loop, from the recorded
        /// gate: (t, radio, gauge) at every recorded tick before the wheels.
        fn ka_replay(f: &Flown, fit: Fit, pipe: f64) -> Vec<(f64, f64, f64)> {
            let (_, h, v, ias, _) = f.ticks[0];
            let mut p = Plant::at(fit, h, v.unwrap(), ias, pipe);
            p.ias_track = Some(f.ticks.iter().map(|x| (x.0, x.3)).collect());
            p.thr0 = f.levers.first().map_or(p.thr0, |l| l.1);
            p.thr = p.thr0;
            p.thr_cmd = p.thr0;
            let mut out = vec![];
            let (mut wi, mut li) = (0, 0);
            for &(t, _, _, _, rolling) in &f.ticks {
                if rolling {
                    break;
                }
                while p.t < t - 1e-9 {
                    while wi < f.writes.len() && f.writes[wi].0 <= p.t + 1e-9 {
                        let e = f.writes[wi].1;
                        p.write(e);
                        wi += 1;
                    }
                    while li < f.levers.len() && f.levers[li].0 <= p.t + 1e-9 {
                        p.thr_cmd = f.levers[li].1;
                        li += 1;
                    }
                    p.step(0.01);
                }
                out.push((t, p.radio(), p.vvi()));
            }
            out
        }

        /// The plant is the recording: each flare's column and lever as
        /// written, open loop, give back its radio within 4.5 ft and its
        /// gauge within 95 fpm rms on every plant kept (the fit: 1.9 to 2.1
        /// ft, 50 to 56 fpm).
        #[test]
        fn the_king_air_plant_replays_its_flares() {
            for fit in ka_plants() {
                for (src, pipe) in SRC.iter().zip(PIPES) {
                    let f = flown(src);
                    let rec: Vec<_> = f.ticks.iter().filter(|x| !x.4).collect();
                    let got = ka_replay(&f, fit, pipe);
                    let n = got.len().min(rec.len()) as f64;
                    let (mut dh, mut dv) = (0.0, 0.0);
                    for (g, r) in got.iter().zip(&rec) {
                        dh += (g.1 - r.1).powi(2);
                        dv += (g.2 - r.2.unwrap()).powi(2);
                    }
                    let (dh, dv) = ((dh / n).sqrt(), (dv / n).sqrt());
                    eprintln!("replay {}: {dh:.2} ft, {dv:.0} fpm rms", tag(fit));
                    assert!(dh < 4.5 && dv < 95.0, "{}: {dh:.2} ft, {dv:.0} fpm", tag(fit));
                }
            }
        }

        /// A plant's name for the record.
        fn tag(f: Fit) -> String {
            format!(
                "k {:.0} dead {:.2} path {:.2} gauge {:.2} tail {:.0} wing {:.0}",
                f.k, f.dead, f.path_s, f.gauge_s, f.tail_fpm, f.wing_fpm
            )
        }

        fn king_air() -> crate::aircraft::AircraftProfile {
            crate::aircraft::load("king_air_c90b").expect("the King Air profile loads")
        }

        /// The three recorded gates and neighbours: steeper and shallower,
        /// faster and slower, lower.
        const GATES: [(f64, f64, f64); 6] = [
            (58.9, -703.0, 111.9),
            (61.3, -642.0, 113.5),
            (59.0, -699.0, 110.5),
            (60.0, -780.0, 110.0),
            (60.0, -600.0, 114.0),
            (52.0, -700.0, 106.0),
        ];

        /// The tracker as flown meets the runway hard on the plant from the
        /// recorded gates, as it did (730 to 960 fpm on the gauge a tick
        /// before the wheels); the declared model flare (`flare_elevator`
        /// model, `flare_power`) from every gate on every plant that
        /// replays, calm and rough: one contact under 250 fpm on the flight
        /// path and on the gauge, never slower than the stall, and no longer
        /// a float than the tracker's as flown on the plant (700 to 1,000 m
        /// from the gate to the wheels; the sim's three about 830). The hop
        /// after it is held under 2 ft, not the SR22's 1.5: the gear is the
        /// SR22's, whose nose-up kick grows as the square of the speed, and
        /// at 90 to 97 kt it hops 1.6 to 1.8 ft off 115 to 135 fpm.
        #[test]
        fn the_king_air_flares_where_the_tracker_dropped_it_in() {
            let mut flown_worst = 0.0_f64;
            for fit in ka_plants() {
                for ((h, v, ias), pipe) in GATES.iter().take(3).zip(PIPES) {
                    let mut p = Plant::at(fit, *h, *v, *ias, pipe);
                    let l = super::lilb03_evolution::land_tracker(&mut p, FlareGains::default(), 40.0);
                    let gauge = -p.first_gauge.unwrap_or(f64::NAN);
                    eprintln!("tracker as flown {}: {l:?}, gauge {gauge:.0}, {:.0} m", tag(fit), p.along_m);
                    flown_worst = flown_worst.max(l.first_fpm.max(gauge));
                }
            }
            assert!(flown_worst > 400.0, "the tracker as flown: {flown_worst:.0} fpm");
            let prof = king_air();
            let power = prof.flare_power.expect("the King Air declares its retard");
            let approach = prof
                .reference
                .as_ref()
                .and_then(|r| r.approach_kts)
                .expect("an approach speed");
            let mut worst = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, f64::INFINITY);
            let mut bad: Vec<String> = vec![];
            for fit in ka_plants() {
                for (h, v, ias) in GATES {
                    for rough in [false, true] {
                        let pipe = PIPES[0];
                        let mut p = Plant::at(fit, h, v, ias, pipe);
                        let mut m = FlareModel::from_profile(prof.flare_elevator).expect("a model");
                        let l = land(&mut p, &mut m, power, approach, rough, 45.0);
                        let gauge = -p.first_gauge.unwrap_or(f64::NAN);
                        let why = format!("{}, {h} {v} {ias} rough {rough}: {l:?}, gauge {gauge:.0}, {:.0} m", tag(fit), p.along_m);
// Without the push a standing plant FLOATS in these models (18 to 20 s
                        // from 60 ft, to 1,075 m); flown, the King Air's no-push
                        // flare took 11.3 s, no longer than the pushed ones (10.8,
                        // 11.5 s, which skipped): LEMG 13 1790783744 vs 1790783945
                        // and 1790784149, iMac 2026-09-30. The float is bounded
                        // by the pavement (`commit_flare`), not by the model.
                        if !(l.first_fpm < 250.0
                            && gauge < 250.0
                            && l.bounce_ft < 2.5
                            && l.min_ias > fit.vso
                            && p.along_m < 1100.0)
                        {
                            bad.push(why);
                        }
                        worst = (
                            worst.0.max(l.first_fpm),
                            worst.1.max(gauge),
                            worst.2.max(l.bounce_ft),
                            worst.3.max(p.along_m),
                            worst.4.min(l.first_ias),
                        );
                    }
                }
            }
            eprintln!(
                "King Air flare: worst {:.0} fpm on the path, {:.0} on the gauge, {:.2} ft after, {:.0} m past the gate, slowest contact {:.1} kt",
                worst.0, worst.1, worst.2, worst.3, worst.4
            );
            assert!(bad.is_empty(), "{} landings out of limits:\n{}", bad.len(), bad.join("\n"));
        }
    }
}

/// Issue #166, the rollout: LEMG 13, the SR22 (sortie 1790619465-approach,
/// 2026-09-28), both brakes at 0.24 and the rudder alone steering. The
/// recorded heading IS the plant here: fitted, the steer explains none of it
/// (under 2 deg/s per unit), so what the SR22 did on the wheels is taken as
/// the disturbance and the brakes' yaw is added on top of it, a first-order
/// yaw rate (0.5 s) behind 0.3 s of the plugin's channel, at the loop's
/// 0.4 s tick. The brakes' authority is not in this record (nothing split
/// them); the SR22's taxi ticks give 40 to 75 deg/s per unit of difference
/// at taxi pace, with less weight on the wheels at 70 kt, so the law is
/// flown from 15 to 60.
#[cfg(test)]
mod lemg13_sr22_rollout_tests {
    use super::*;

    /// The recorded rollout ticks (s from the first, groundspeed kt, true
    /// heading, cross-track m right, the heading error, the rudder steer).
    const REC: [(f64, f64, f64, f64, f64, f64); 36] = [
        (0.0, 73.9, 131.22, 1.65, 0.14, 0.004),
        (0.4, 72.9, 132.07, 1.55, -0.67, -0.045),
        (0.8, 71.8, 133.34, 1.56, -1.96, -0.094),
        (1.2, 70.8, 133.78, 1.74, -2.52, -0.102),
        (1.6, 68.8, 136.47, 2.15, -5.44, -0.257),
        (2.0, 66.7, 137.92, 2.70, -7.19, -0.305),
        (2.5, 66.0, 136.75, 3.40, -6.34, -0.229),
        (2.8, 64.5, 134.53, 4.07, -4.43, -0.166),
        (3.3, 62.6, 132.86, 4.71, -3.03, -0.160),
        (3.6, 60.5, 132.71, 5.31, -3.15, -0.219),
        (4.1, 58.3, 133.85, 5.87, -4.51, -0.304),
        (4.4, 56.2, 135.59, 6.34, -6.47, -0.405),
        (4.8, 53.9, 136.96, 6.87, -8.11, -0.463),
        (5.2, 51.4, 137.45, 7.53, -8.98, -0.485),
        (5.6, 49.2, 136.96, 8.16, -8.89, -0.492),
        (6.1, 46.5, 135.41, 8.97, -7.89, -0.490),
        (6.5, 44.2, 134.09, 9.62, -7.04, -0.510),
        (6.9, 42.2, 133.30, 10.14, -6.59, -0.552),
        (7.3, 39.6, 132.86, 10.67, -6.61, -0.600),
        (7.7, 37.1, 132.88, 11.09, -6.67, -0.631),
        (8.1, 34.6, 133.05, 11.39, -6.84, -0.671),
        (8.5, 32.0, 133.04, 11.63, -6.80, -0.686),
        (8.9, 29.5, 132.63, 11.84, -6.36, -0.662),
        (9.3, 27.4, 132.04, 11.98, -5.72, -0.624),
        (9.7, 25.0, 131.27, 12.06, -4.85, -0.575),
        (10.1, 22.8, 130.56, 12.07, -4.03, -0.527),
        (10.6, 20.3, 129.81, 12.01, -3.12, -0.472),
        (11.0, 18.2, 129.16, 11.91, -2.32, -0.417),
        (11.4, 15.8, 128.43, 11.76, -1.41, -0.349),
        (11.8, 13.9, 127.83, 11.61, -0.69, -0.291),
        (12.2, 11.5, 127.09, 11.42, 0.21, -0.216),
        (12.6, 9.5, 126.42, 11.25, 0.99, -0.149),
        (13.0, 7.1, 125.61, 11.07, 1.90, -0.074),
        (13.4, 5.1, 124.91, 10.95, 2.65, -0.010),
        (13.9, 3.2, 124.19, 10.85, 3.39, 0.039),
        (14.2, 0.6, 123.26, 10.79, 4.32, 0.147),
    ];
    /// LEMG 13's true course: the first tick's heading plus its error plus
    /// the cut its 1.65 m asked (131.22 + 0.14 + 0.33).
    const COURSE: f64 = 131.69;
    const TICK_S: f64 = 0.4;
    const CHANNEL_DEAD_S: f64 = 0.3;
    const YAW_LAG_S: f64 = 0.5;

    /// Column `i` of the record at `t`, linear between ticks.
    fn at(t: f64, i: usize) -> f64 {
        let col = |r: &(f64, f64, f64, f64, f64, f64)| match i {
            1 => r.1,
            2 => r.2,
            _ => r.3,
        };
        if t <= REC[0].0 {
            return col(&REC[0]);
        }
        for w in REC.windows(2) {
            if (w[0].0..=w[1].0).contains(&t) {
                let f = (t - w[0].0) / (w[1].0 - w[0].0);
                return col(&w[0]) + f * (col(&w[1]) - col(&w[0]));
            }
        }
        col(&REC[REC.len() - 1])
    }

    /// The rollout from the first tick to the stop with the brakes split by
    /// the law (or not), `authority` deg/s of yaw per unit of brake
    /// difference at the wheels. The worst cross-track, metres.
    fn roll(authority: f64, split: bool) -> f64 {
        roll_with(authority, split, None)
    }

    /// [`roll`] with the free nose's one ground law on `plant` (`None` the
    /// shared split).
    pub(super) fn roll_with(
        authority: f64,
        split: bool,
        plant: Option<crate::ground_steer::GroundPlant>,
    ) -> f64 {
        let end = REC[REC.len() - 1].0;
        let dt = 0.01;
        let (mut t, mut next) = (0.0, 0.0);
        let (mut dpsi, mut yaw, mut dx) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut rate = crate::autotakeoff::XtkRate::default();
        let mut prev: Option<f64> = None;
        let mut law = plant.map_or_else(RolloutBrakeSteer::default, RolloutBrakeSteer::free);
        let mut written: Vec<(f64, f64)> = vec![(-10.0, 0.0)];
        let mut worst: f64 = 0.0;
        while t < end {
            let gs = at(t, 1);
            if t + 1e-9 >= next {
                next += TICK_S;
                let x = at(t, 3) + dx;
                let r = rate.update(prev.map_or(0.0, |p| (x - p) / TICK_S), TICK_S);
                prev = Some(x);
                let target = crate::autotakeoff::roll_target_heading(COURSE, x, r, gs);
                let psi = at(t, 2) + dpsi;
                let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
                let brake = rollout_brake_ratio(gs, 0.0, 2500.0);
                let b = if split {
                    law.step_with(err, Some(psi), None, 0.0, TICK_S, gs, brake)
                } else {
                    RolloutBrakes {
                        sym: brake,
                        left: 0.0,
                        right: 0.0,
                    }
                };
                // The plugin: each wheel the harder of the pair.
                let diff = b.sym.max(b.right) - b.sym.max(b.left);
                written.push((t, diff));
            }
            let arriving = written
                .iter()
                .take_while(|(tw, _)| *tw <= t - CHANNEL_DEAD_S + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            yaw += (authority * arriving - yaw) * dt / YAW_LAG_S;
            dpsi += yaw * dt;
            dx += gs * 0.514_444 * dpsi.to_radians().sin() * dt;
            worst = worst.max((at(t, 3) + dx).abs());
            t += dt;
        }
        worst
    }

    /// As flown the record is the plant: 12.1 m right. Split by the law the
    /// brakes hold the same rollout inside 2.6 m at every authority from 15
    /// to 60 deg/s per unit, and inside 3.5 m at 10 and 90 (3.1 and 3.3; the start is 1.65 m off).
    #[test]
    fn the_sr22_rollout_steers_on_its_brakes() {
        let flown = roll(30.0, false);
        assert!((flown - 12.07).abs() < 0.1, "as flown: {flown:.2}");
        for a in [15.0, 20.0, 30.0, 45.0, 60.0] {
            let w = roll(a, true);
            assert!(w < 2.6, "{a} deg/s per unit: {w:.2} m");
        }
        for a in [10.0, 90.0] {
            let w = roll(a, true);
            assert!(w < 3.5, "{a} deg/s per unit: {w:.2} m");
        }
    }

    /// The law: nose right of the target is the left wheel, the difference
    /// taken off the outside wheel so the stop is kept, capped, damped on
    /// the error's rate; nothing below 4 kt or on an unread heading.
    #[test]
    fn a_rollout_brake_split_keeps_the_stop() {
        let mut law = RolloutBrakeSteer::default();
        let b = law.step(-5.0, 0.4, 60.0, 0.24);
        assert!((b.left - 0.315).abs() < 1e-9, "{b:?}");
        assert!((b.sym - 0.165).abs() < 1e-9, "{b:?}");
        assert_eq!(b.right, 0.0);
        // The error closing at 5 deg/s leads it off.
        let b = law.step(-3.0, 0.4, 60.0, 0.24);
        assert!(b.right > 0.0 && b.left == 0.0, "{b:?}");
        // Capped at 0.3 of difference.
        let mut law = RolloutBrakeSteer::default();
        let b = law.step(40.0, 0.4, 60.0, 0.24);
        assert!((b.right - b.sym - 0.3).abs() < 1e-9, "{b:?}");
        // A light stop cannot go below zero on the outside wheel.
        let mut law = RolloutBrakeSteer::default();
        let b = law.step(40.0, 0.4, 60.0, 0.05);
        assert_eq!(b.sym, 0.0);
        assert!((b.right - 0.2).abs() < 1e-9, "{b:?}");
        // Slow, or no heading: the plain stop.
        let mut law = RolloutBrakeSteer::default();
        let plain = RolloutBrakes {
            sym: 0.24,
            left: 0.0,
            right: 0.0,
        };
        assert_eq!(law.step(-5.0, 0.4, 3.0, 0.24), plain);
        assert_eq!(law.step(f64::NAN, 0.4, 60.0, 0.24), plain);
    }
}

/// Issue #166 again, the split flown twice on the iMac (0.42 to 0.62 s
/// ticks), LEMG 13, the SR22, both rollouts latched on a BOUNCE.
///
/// 1790689843-approach (the shared split; 9.1 m): the latch took the first
/// contact at 88 kt and the aircraft flew 3.5 to 3.8 ft for about six seconds
/// after it (`state` ground false, the deceleration 2.5 kt/s, the flight
/// model's touchdown at 69 kt). 1790694457-approach (the split of aa5ab0f;
/// 9.4 m): three porpoises in the flare (radio 0.8, 0.4, 1.3 ft and back up
/// to 7.6, 11.4, 10.2), the third contact kicking the bank to 7 degrees and
/// the drift from 0 to 1.1 m/s right, then the latch at 61 kt and up to 9 ft
/// for four and a half seconds with 0.39 of left brake on, which slewed the
/// nose from 134 to 127 in a second as the wheels came down on a 131.5
/// track. The `state` samples ride about 2.5 s behind their stamp (their
/// groundspeed against the rollout ticks'); the wheels came down at about
/// 6.3 s and 4.4 s after the latch, where the deceleration jumps.
///
/// Fitted over the ground ticks of both (two-tick predictions of heading and
/// track): the wheel difference 45 deg/s per unit after 0.4 to 0.5 s, the
/// nose weathervaning onto the track at 2 to 6 per second at 60 kt (as the
/// square of the speed), the track following the nose at 2 to 3 per second.
/// The replay takes the recorded heading and cross-track as the plant under
/// the recorded split and adds, on the wheels, the response to the
/// DIFFERENCE a law writes through that plant; in the air (with the bounce
/// handled) no brake and the arrest's bank target flown through 0.3 s and a
/// 0.8 s roll lag, as a point mass from the recorded drift. The laws as
/// flown reproduce both rollouts.
#[cfg(test)]
mod lemg13_sr22_rollout_split_tests {
    use super::*;
    use crate::ground_steer::GroundPlant;

    /// The split 1790694457 flew (aa5ab0f), kept here to replay it: 0.025
    /// of difference per degree, 1.2 s of lead on the raw rate, a trim of
    /// 0.8 per second to 0.15. (The SR22's later declared schedule, the
    /// `rollout_brakes:` block, is retired for the one ground law.)
    #[derive(Default)]
    struct AsFlown2 {
        prev: Option<f64>,
        rate: f64,
        trim: f64,
    }

    impl AsFlown2 {
        fn step(&mut self, err: f64, dt: f64, gs: f64, brake: f64) -> RolloutBrakes {
            if let Some(p) = self.prev {
                self.rate = (err - p) / dt.clamp(0.05, 3.0);
            }
            self.prev = Some(err);
            if gs < 4.0 {
                return RolloutBrakes { sym: brake, left: 0.0, right: 0.0 };
            }
            self.trim = (self.trim + 0.025 * 0.8 * err * dt.clamp(0.05, 1.0)).clamp(-0.15, 0.15);
            let diff = (0.025 * (err + 1.2 * self.rate) + self.trim).clamp(-0.3, 0.3);
            let (sym, left, right) = crate::ground_steer::stop_wheels(brake, 0.0);
            let _ = (left, right);
            let half = diff.abs() / 2.0;
            let outside = (sym - half).max(0.0);
            let inside = (sym + half).min(1.0);
            if diff > 0.0 {
                RolloutBrakes { sym: outside, left: 0.0, right: inside }
            } else if diff < 0.0 {
                RolloutBrakes { sym: outside, left: inside, right: 0.0 }
            } else {
                RolloutBrakes { sym, left: 0.0, right: 0.0 }
            }
        }
    }

    /// The law a replay flies.
    #[derive(Clone, Copy, Debug)]
    enum Law {
        /// The shared split (1790689843 as flown; the SF50's today).
        Shared,
        /// 1790694457 as flown.
        AsFlown2,
        /// The free nose's one ground law on the identified SR22.
        Ground,
    }

    enum Running {
        Split(RolloutBrakeSteer),
        Flown(AsFlown2),
    }

    impl Running {
        fn new(law: Law) -> Self {
            match law {
                Law::Shared => Running::Split(RolloutBrakeSteer::default()),
                Law::AsFlown2 => Running::Flown(AsFlown2::default()),
                Law::Ground => Running::Split(RolloutBrakeSteer::free(GroundPlant::SR22)),
            }
        }

        fn step(&mut self, err: f64, psi: f64, trk: f64, dt: f64, gs: f64, brake: f64) -> RolloutBrakes {
            match self {
                Running::Split(l) => l.step_with(err, Some(psi), Some(trk), 0.0, dt, gs, brake),
                Running::Flown(l) => l.step(err, dt, gs, brake),
            }
        }
    }

    const FLOWN_1: &str = include_str!("../bench/landings/sr22_lemg13_rollout_1790689843.jsonl");
    const FLOWN_2: &str = include_str!("../bench/landings/sr22_lemg13_rollout_1790694457.jsonl");
    /// LEMG 13's true course, as in the 2026-09-28 record.
    const COURSE: f64 = 131.69;

    /// (s from the latch, the tick's dt, groundspeed, heading, cross-track m
    /// right, the stopping brake, the wheel difference as the plugin applied
    /// it, right positive).
    type Tick = (f64, f64, f64, f64, f64, f64, f64);

    fn ticks(text: &str) -> Vec<Tick> {
        let mut t = 0.0;
        let mut out = Vec::new();
        for (i, line) in text.lines().filter(|l| !l.trim().is_empty()).enumerate() {
            let r: serde_json::Value = serde_json::from_str(line).unwrap();
            let f = |k: &str| r[k].as_f64().unwrap();
            let dt = f("dt");
            if i > 0 {
                t += dt;
            }
            let (sym, bl, br) = (f("brake"), f("bl"), f("br"));
            let inside = bl.max(br);
            let stop = if inside > 0.0 {
                (sym + inside) / 2.0
            } else {
                sym
            };
            out.push((
                t,
                dt,
                f("gs"),
                f("psi"),
                f("xtk_m"),
                stop,
                sym.max(br) - sym.max(bl),
            ));
        }
        out
    }

    /// The wheels' plant: deg/s of yaw per unit of difference, the dead time
    /// and lag before it, the weathervane per second at 60 kt, and the rate
    /// the track follows the nose.
    #[derive(Clone, Copy)]
    struct Plant {
        a: f64,
        dead_s: f64,
        lag_s: f64,
        vane_60: f64,
        track_per_s: f64,
    }
    const FIT: Plant = Plant {
        a: 45.0,
        dead_s: 0.45,
        lag_s: 0.1,
        vane_60: 4.0,
        track_per_s: 2.5,
    };

    /// A flown rollout.
    struct Flown {
        text: &'static str,
        /// The wheels down to stay, s after the latch.
        touchdown_s: f64,
        /// The bank at the latch, degrees.
        phi0: f64,
    }
    const S1: Flown = Flown {
        text: FLOWN_1,
        touchdown_s: 6.3,
        phi0: 0.0,
    };
    const S2: Flown = Flown {
        text: FLOWN_2,
        touchdown_s: 4.4,
        phi0: -0.84,
    };

    /// The replay: `law` flown, the bounce handled or not. The worst
    /// cross-track at the ticks, and (s, heading, wheel difference) for each
    /// tick on the wheels.
    fn replay(f: &Flown, law_kind: Law, bounce_held: bool, p: Plant) -> (f64, Vec<(f64, f64, f64)>) {
        let rec = ticks(f.text);
        let n = rec.len();
        let end = rec[n - 1].0;
        // The recorded sideways velocity (central differences).
        let v_rec: Vec<f64> = (0..n)
            .map(|i| {
                let (a, b) = (i.saturating_sub(1), (i + 1).min(n - 1));
                (rec[b].4 - rec[a].4) / (rec[b].0 - rec[a].0)
            })
            .collect();
        let interp = |t: f64, col: &dyn Fn(usize) -> f64| -> f64 {
            if t <= rec[0].0 {
                return col(0);
            }
            for i in 1..n {
                if t <= rec[i].0 {
                    let fr = (t - rec[i - 1].0) / (rec[i].0 - rec[i - 1].0);
                    return col(i - 1) + fr * (col(i) - col(i - 1));
                }
            }
            col(n - 1)
        };
        let h = 0.01;
        let mut law = Running::new(law_kind);
        let mut on_wheels_seen = false;
        let (mut dpsi, mut dtrk, mut dx, mut dv, mut yaw) =
            (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
        let mut phi = f.phi0;
        let mut bank_cmds: Vec<(f64, f64)> = vec![(-10.0, f.phi0)];
        let mut written: Vec<(f64, f64)> = vec![(-10.0, 0.0)];
        let mut rate = crate::autotakeoff::XtkRate::default();
        let mut prev: Option<f64> = None;
        let (mut t, mut k) = (0.0, 0);
        let mut worst: f64 = 0.0;
        let mut ground = Vec::new();
        while t < end {
            let on = t >= f.touchdown_s;
            if k < n && t + 1e-9 >= rec[k].0 {
                let (tk, dt, gs, psi, xtk, stop, flown) = rec[k];
                let x = xtk + dx;
                let r = rate.update(prev.map_or(0.0, |q| (x - q) / dt), dt);
                prev = Some(x);
                worst = worst.max(x.abs());
                let wd = if on || !bounce_held {
                    if bounce_held && !on_wheels_seen {
                        law = Running::new(law_kind);
                        on_wheels_seen = true;
                    }
                    let target = crate::autotakeoff::roll_target_heading(COURSE, x, r, gs);
                    let err = ((target - (psi + dpsi) + 540.0).rem_euclid(360.0)) - 180.0;
                    // The track as the `hpath` read gives it: the recorded one
                    // (the course and the sideways speed) and the plant's own
                    // departure from it.
                    let trk = COURSE + (v_rec[k] / (gs * 0.514_444).max(1.0)).atan().to_degrees() + dtrk;
                    let b = law.step(err, psi + dpsi, trk, dt, gs, stop);
                    if std::env::var("RO_DEBUG").is_ok() && matches!(law_kind, Law::Ground) {
                        let (rr, w) = match &law {
                            Running::Split(l) => (l.last().map(|s| (s.r_ref, s.r_now)), l.trim()),
                            _ => (None, 0.0),
                        };
                        eprintln!("t {tk:5.2} gs {gs:5.1} x {x:6.2} r {r:5.2} psi {:7.2} trk {trk:7.2} err {err:6.2} diff {:5.2} rr {rr:?} w {w:.2}", psi + dpsi, b.sym.max(b.right) - b.sym.max(b.left));
                    }
                    if gs > 2.0 {
                        b.sym.max(b.right) - b.sym.max(b.left)
                    } else {
                        0.0
                    }
                } else {
                    let v = v_rec[k] + dv;
                    bank_cmds.push((tk, arrest_bank_target(x, v, phi, 5.0)));
                    0.0
                };
                if on {
                    ground.push((tk, psi + dpsi, wd));
                }
                written.push((tk, wd - flown));
                k += 1;
            }
            let g = interp(t, &|i| rec[i].2);
            let v_ms = g * 0.514_444;
            if on {
                let arriving = written
                    .iter()
                    .take_while(|(tw, _)| *tw <= t - p.dead_s + 1e-9)
                    .last()
                    .map_or(0.0, |w| w.1);
                yaw += (p.a * arriving - yaw) * h / p.lag_s;
                let vane = p.vane_60 * (g / 60.0).powi(2);
                dpsi += (yaw + vane * (dtrk - dpsi)) * h;
                dtrk += p.track_per_s * (dpsi - dtrk) * h;
                dx += v_ms * dtrk.to_radians().sin() * h;
            } else if bounce_held {
                let cmd = bank_cmds
                    .iter()
                    .take_while(|(tc, _)| *tc <= t - 0.3 + 1e-9)
                    .last()
                    .map_or(f.phi0, |c| c.1);
                phi += (cmd - phi) * h / 0.8;
                dv += G_M_S2 * phi.to_radians().tan() * h;
                dx += dv * h;
                let vr = interp(t, &|i| v_rec[i]);
                let base = v_ms.max(1.0);
                dtrk = ((vr + dv).atan2(base) - vr.atan2(base)).to_degrees();
                dpsi = dtrk;
            }
            t += h;
        }
        (worst, ground)
    }

    /// The fitted plant and its neighbours.
    fn plants() -> Vec<Plant> {
        let mut out = Vec::new();
        for a in [40.0, 50.0] {
            for dead_s in [0.4, 0.55] {
                for vane_60 in [2.0, 6.0] {
                    for track_per_s in [2.0, 3.0] {
                        out.push(Plant {
                            a,
                            dead_s,
                            lag_s: 0.1,
                            vane_60,
                            track_per_s,
                        });
                    }
                }
            }
        }
        out
    }

    /// The span of the heading on the wheels once past the touchdown's
    /// first two seconds, degrees.
    fn heading_span(g: &[(f64, f64, f64)]) -> f64 {
        let rest = &g[4.min(g.len())..];
        let hi = rest.iter().map(|x| x.1).fold(f64::MIN, f64::max);
        let lo = rest.iter().map(|x| x.1).fold(f64::MAX, f64::min);
        hi - lo
    }

    /// As flown, each law reproduces its rollout (9.06 m, 10.97 m). With the
    /// bounce held, the free nose's one ground law (`ground_steer`, the
    /// identified SR22, no rollout numbers of its own) on this wheel plant
    /// (the #166 fit, not the law's): 1790689843 8.2 m at the fit and 9.1
    /// over the neighbouring wheel plants, WORSE than the retired declared
    /// split (4.8 and 5.2, its numbers fitted on this replay) and the shared
    /// law (7.3): its heading loop is sized for the 0.4 s loop delay the
    /// identified plant carries and follows the lateral's target 2 to 4
    /// degrees behind for 6 s (2026-09-30; the rollout is the manoeuvre to
    /// check in flight). 1790694457 inside ROLL_2_M, the drift the bounce
    /// made before the wheels (no law can touch it), where it flew 11.0 (the
    /// retired split 7.6).
    #[test]
    fn the_sr22_rollouts_replay_with_the_bounce_held() {
        let (flown_1, _) = replay(&S1, Law::Shared, false, FIT);
        assert!((flown_1 - 9.06).abs() < 0.1, "1790689843 as flown: {flown_1:.2}");
        let (flown_2, _) = replay(&S2, Law::AsFlown2, false, FIT);
        assert!((flown_2 - 10.97).abs() < 0.1, "1790694457 as flown: {flown_2:.2}");
        let (fit_1, _) = replay(&S1, Law::Ground, true, FIT);
        let (fit_2, _) = replay(&S2, Law::Ground, true, FIT);
        let (shared_1, _) = replay(&S1, Law::Shared, true, FIT);
        let worst_1 = plants().into_iter().map(|p| replay(&S1, Law::Ground, true, p).0).fold(0.0, f64::max);
        eprintln!("1790689843 {fit_1:.2} m (shared {shared_1:.2}, neighbours {worst_1:.2}), 1790694457 {fit_2:.2} m");
        assert!(fit_1 < ROLL_1_M, "1790689843 at the fit: {fit_1:.2} m");
        assert!(fit_2 < ROLL_2_M, "1790694457 at the fit: {fit_2:.2} m");
        assert!(worst_1 < ROLL_1_NEIGHBOURS_M, "1790689843 over the neighbours: {worst_1:.2} m");
    }
    const ROLL_1_M: f64 = 8.5;
    const ROLL_1_NEIGHBOURS_M: f64 = 9.5;
    const ROLL_2_M: f64 = 7.8;

    /// Below 35 kt the numbers 1790694457 flew banged the split from wheel
    /// to wheel (0.39 each tick in the air and on it) and swung the nose
    /// +/-7 degrees; replayed on its wheels the one ground law keeps the
    /// heading's span under two thirds of theirs on a wheel plant up to 75
    /// deg/s per unit and 0.6 s (3.4, 8.3, 11.5 degrees against 10.3, 13.6,
    /// 22.2; 2026-09-30).
    #[test]
    fn the_sr22_split_does_not_bang_wheel_to_wheel_slow() {
        for (a, dead_s) in [(45.0, 0.45), (60.0, 0.6), (75.0, 0.6)] {
            let p = Plant { a, dead_s, ..FIT };
            let (_, new) = replay(&S2, Law::Ground, true, p);
            let (_, old) = replay(&S2, Law::AsFlown2, true, p);
            let (sn, so) = (heading_span(&new), heading_span(&old));
            eprintln!("{a} {dead_s}: span {sn:.1} (old numbers {so:.1})");
            assert!(sn < 0.65 * so, "{a} deg/s per unit, {dead_s} s: {sn:.1} degrees against {so:.1}");
            assert!(so > sn + 2.0, "the old numbers swung {so:.1}, the new {sn:.1}");
        }
    }

    /// The bounce watch: on the wheels, off (a spell), in the air, down.
    #[test]
    fn a_bounce_is_a_spell_off_the_wheels() {
        let mut b = BounceHold::default();
        assert_eq!(b.tick(false), BounceTick::OnWheels);
        assert_eq!(b.tick(true), BounceTick::Left);
        assert_eq!(b.tick(true), BounceTick::InAir);
        assert_eq!(b.tick(false), BounceTick::Landed);
        assert_eq!(b.tick(false), BounceTick::OnWheels);
        assert_eq!(b.tick(true), BounceTick::Left);
        assert_eq!(b.spells(), 2);
    }

    /// The free law learns the standing yaw (its observer, the declared
    /// trim it replaces: in the model's own units, the crossover-matched
    /// gain's, so a third of a standing 3 deg/s^2 reads about 1.4) and holds
    /// the heading against it, and leaves the stop plain below 4 kt; the
    /// shared split learns nothing.
    #[test]
    fn the_free_split_learns_the_standing_yaw() {
        let mut law = RolloutBrakeSteer::free(GroundPlant::SR22);
        let mut psi = 131.0;
        let mut r = 0.0_f64;
        for _ in 0..300 {
            let err = 131.0 - psi;
            let b = law.step_with(err, Some(psi), None, 0.0, 0.46, 50.0, 0.22);
            let diff = b.sym.max(b.right) - b.sym.max(b.left);
            // The SR22 at 25.7 m/s with the idle propeller's right yaw.
            let p = GroundPlant::SR22;
            let v = 50.0 * 0.514_444;
            for _ in 0..46 {
                r += (p.kb(v) * diff - p.d(v) * r + 3.0) * 0.01;
                psi += r * 0.01;
            }
        }
        assert!((psi - 131.0).abs() < 0.5, "held: {psi:.2}");
        let (kb_m, _) = GroundPlant::SR22.model(50.0 * 0.514_444, 0.46);
        let want = 3.0 * kb_m / GroundPlant::SR22.kb(50.0 * 0.514_444);
        assert!((law.trim() - want).abs() < 0.3 * want, "learnt {:.2} against {want:.2}", law.trim());
        let mut shared = RolloutBrakeSteer::default();
        for _ in 0..200 {
            shared.step(-3.0, 0.46, 60.0, 0.22);
        }
        assert_eq!(shared.trim(), 0.0);
        let mut slow = RolloutBrakeSteer::free(GroundPlant::SR22);
        let b = slow.step_with(-3.0, Some(131.0), None, 0.0, 0.46, 3.0, 0.22);
        assert_eq!(b, RolloutBrakes { sym: 0.22, left: 0.0, right: 0.0 });
    }

    /// The 2026-09-28 rollout (#166, rudder alone as flown; that replay has
    /// no weathervane and the record's first seconds may be a bounce too)
    /// under the one ground law (a wheel plant of its own, not the law's):
    /// inside 6.5 m from 10 to 90 deg/s per unit (2.5 to 3.7 to 60, 6.0 at
    /// 90; the shared law 1.7 to 3.3 there, as flown 12.1).
    #[test]
    fn the_sr22_split_on_the_first_rollout() {
        for a in [10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0] {
            let w = super::lemg13_sr22_rollout_tests::roll_with(a, true, Some(GroundPlant::SR22));
            eprintln!("{a}: {w:.2} m");
            assert!(w < 6.5, "{a} deg/s per unit: {w:.2} m");
        }
    }
}

/// Issue #180: the A330 at LEMG 30, 10 nm final (sortie
/// 1790634412-approach, 2026-09-28), first approach in rough air (the
/// final's air 119 fpm rms). The arrest took the servos at 86.8 ft and flew
/// the default tracker (the A330 declared no `flare_elevator:`): 0.35 of
/// pull held five seconds arrested -1,155 fpm into a +277 climb, the column
/// came back to neutral and the aircraft FLOATED at 30 to 48 ft for 17 s,
/// was committed at 46.4 ft with 1,432 m ahead and went around
/// `overflew_the_runway` 0.3 s later.
///
/// The plant is the A330's sidestick with the servos out: a RATE plant (the
/// pull asks a change of sink, through two lags after a dead time, and a
/// neutral stick holds the sink it has), the ground's own rate growing
/// below a height, and a sink that grows with the airspeed lost since the
/// gate (fpm per knot), the speed itself -1.0 kt/s less 1.3 per 1,000 fpm of
/// climb (the four arrests' idle bleed). Fitted per arrest to every tick of
/// the four recorded A330 arrests (LEMG 13, sortie 1790621184; LEMG 30,
/// sorties 1790633832 and 1790634412 twice): 6 to 21 fpm rms each, and the
/// four together. A standing-sink fit of the same ticks needs a 93 s lag,
/// which is the integrator. Rough air is a random vertical gust (an
/// Ornstein-Uhlenbeck sink, 1.5 s). The loop's tick is 0.2 s.
#[cfg(test)]
mod a330_lemg30_flare_tests {
    use super::*;

    /// The recorded arrest of the first approach (s from `arrest_hands`
    /// taken, radio ft, vvi fpm, the elevator written, IAS kt), every
    /// other tick, to the tick before the servos went back.
    const FLOAT: [(f64, f64, f64, f64, f64); 40] = [
        (0.0, 86.85, -1154.8, 0.350, 162.29),
        (0.7, 72.30, -1141.2, 0.350, 162.53),
        (1.4, 58.52, -1112.6, 0.350, 162.90),
        (2.0, 47.93, -1014.6, 0.350, 163.28),
        (2.7, 40.41, -855.3, 0.350, 163.62),
        (3.1, 36.77, -737.9, 0.350, 163.75),
        (3.6, 33.96, -611.5, 0.350, 163.84),
        (3.9, 32.18, -506.0, 0.350, 163.88),
        (4.4, 30.72, -389.1, 0.350, 163.89),
        (4.8, 29.95, -274.3, 0.350, 163.82),
        (5.2, 29.88, -195.1, 0.350, 163.75),
        (5.6, 30.41, -95.6, 0.203, 163.57),
        (6.0, 31.44, -8.8, 0.147, 163.33),
        (6.4, 32.90, 70.6, 0.093, 163.08),
        (6.8, 34.93, 154.5, 0.037, 162.72),
        (7.3, 37.37, 223.0, 0.000, 162.25),
        (7.6, 39.29, 258.9, 0.000, 161.77),
        (8.1, 41.32, 277.2, 0.000, 161.15),
        (8.5, 42.60, 273.1, 0.000, 160.56),
        (8.9, 43.41, 256.1, 0.000, 160.12),
        (9.3, 44.06, 229.4, 0.000, 159.70),
        (9.7, 45.12, 200.3, 0.000, 159.35),
        (10.1, 46.30, 175.9, 0.000, 159.08),
        (10.5, 46.89, 150.7, 0.000, 158.77),
        (11.0, 47.16, 130.6, 0.000, 158.51),
        (11.3, 47.38, 116.1, 0.000, 158.30),
        (11.8, 47.61, 102.8, 0.000, 158.10),
        (12.1, 47.59, 91.6, 0.000, 157.93),
        (12.5, 47.78, 81.3, 0.000, 157.78),
        (12.9, 47.87, 69.6, 0.000, 157.65),
        (13.4, 48.16, 57.1, 0.000, 157.50),
        (13.7, 48.28, 45.4, 0.000, 157.36),
        (14.1, 48.31, 32.7, 0.000, 157.23),
        (14.6, 48.36, 20.8, 0.000, 157.10),
        (14.9, 48.24, 10.4, 0.000, 157.00),
        (15.4, 48.09, -1.5, 0.000, 156.87),
        (15.8, 47.88, -13.5, 0.000, 156.74),
        (16.2, 47.48, -25.3, 0.000, 156.58),
        (16.6, 46.99, -39.3, 0.000, 156.34),
        (17.0, 46.40, -56.3, 0.000, 156.03),
    ];
    /// Every recorded A330 gate (radio ft, vvi fpm, trend fpm/s, IAS kt):
    /// LEMG 13 (1790621184), LEMG 30's circuit (1790633832), the floated
    /// approach and the one that landed (1790634412).
    const GATES: [(f64, f64, f64, f64); 4] = [
        (87.9, -913.9, 37.6, 162.3),
        (86.4, -942.8, -13.9, 161.2),
        (86.9, -1154.8, 20.0, 162.3),
        (90.8, -810.7, 26.3, 164.1),
    ];
    const FLOATED: usize = 2;
    /// (fpm/s per unit, lag 1, lag 2, dead s, ground fpm/s at the wheels,
    /// from ft, fpm per knot lost): each arrest's own fit, then all four.
    type Fit = (f64, f64, f64, f64, f64, f64, f64);
    const PLANTS: [Fit; 5] = [
        (824.0, 0.611, 0.602, 0.78, 164.6, 48.7, 42.9),
        (808.0, 0.235, 0.288, 1.633, 55.5, 107.3, 82.9),
        (764.0, 0.42, 0.438, 1.12, 10.95, 70.1, 75.6),
        (598.0, 0.151, 0.275, 1.38, 75.1, 41.7, 45.35),
        (777.0, 0.1, 0.465, 1.67, 60.0, 49.2, 94.1),
    ];
    const TICK_S: f64 = 0.2;

    struct Plant {
        fit: Fit,
        h: f64,
        v0: f64,
        v: f64,
        ias0: f64,
        ias: f64,
        integ: f64,
        b1: f64,
        b2: f64,
        gust: f64,
        rng: u64,
        rough_fpm: f64,
        t: f64,
        written: Vec<(f64, f64)>,
    }

    impl Plant {
        /// At a gate: sinking `v` with `trend` (the autopilot's own rate,
        /// standing in the pipe), at `ias`.
        fn at(fit: Fit, gate: (f64, f64, f64, f64)) -> Self {
            let (h, v, trend, ias) = gate;
            Plant {
                fit,
                h,
                v0: v,
                v,
                ias0: ias,
                ias,
                integ: 0.0,
                b1: trend,
                b2: trend,
                gust: 0.0,
                rng: 0,
                rough_fpm: 0.0,
                t: 0.0,
                written: vec![(-10.0, trend / fit.0)],
            }
        }
        fn rough(mut self, seed: u64, rms_fpm: f64) -> Self {
            self.rng = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
            self.rough_fpm = rms_fpm;
            self
        }
        fn gauss(&mut self) -> f64 {
            let mut u = || {
                self.rng ^= self.rng << 13;
                self.rng ^= self.rng >> 7;
                self.rng ^= self.rng << 17;
                ((self.rng >> 11) as f64 + 0.5) / (1u64 << 53) as f64
            };
            let (a, b) = (u(), u());
            (-2.0 * a.ln()).sqrt() * (std::f64::consts::TAU * b).cos()
        }
        fn write(&mut self, e: f64) {
            self.written.push((self.t, e));
        }
        /// One step; `ias` recorded, else the plant's own bleed.
        fn step_with(&mut self, dt: f64, ias: Option<f64>) {
            let (k, l1, l2, dead, gf, gft, spd) = self.fit;
            let due = self.t - dead;
            let arriving = self
                .written
                .iter()
                .take_while(|(t, _)| *t <= due + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            self.b1 += (k * arriving - self.b1) * dt / l1;
            self.b2 += (self.b1 - self.b2) * dt / l2;
            let ground = gf * (1.0 - self.h.max(0.0) / gft).max(0.0);
            self.integ += (self.b2 - ground) * dt;
            if self.rough_fpm > 0.0 {
                const TC: f64 = 1.5;
                let n = self.gauss();
                self.gust += -self.gust / TC * dt + self.rough_fpm * (2.0 * dt / TC).sqrt() * n;
            }
            self.v = self.v0 + self.integ - spd * (self.ias0 - self.ias) + self.gust;
            self.ias = match ias {
                Some(i) => i,
                None => self.ias + (-1.0 - 1.3 * self.v / 1000.0) * dt,
            };
            self.h += self.v / 60.0 * dt;
            self.t += dt;
        }
    }

    enum Law {
        Model(FlareModel),
        Tracker(FlarePitch, Option<f64>),
    }

    impl Law {
        fn a330_elevator() -> crate::aircraft::FlareElevatorProfile {
            crate::aircraft::load("airbus_a330_300")
                .and_then(|p| p.flare_elevator)
                .expect("the A330 declares its flare")
        }
        fn today() -> Self {
            Law::Model(
                FlareModel::from_profile(Some(Self::a330_elevator()))
                    .expect("the A330 declares its model"),
            )
        }
        /// The tracker the A330 flew: no `flare_elevator:`, the default law.
        fn as_flown() -> Self {
            Law::Tracker(FlarePitch::with_gains(FlareGains::from_profile(None)), None)
        }
        fn step(&mut self, h: f64, gs: f64, v: f64, dt: f64, flare: bool, rough: bool) -> f64 {
            match self {
                Law::Model(m) => m.step(h, gs, v, dt, flare, rough).elev,
                Law::Tracker(p, prev) => {
                    let trend = prev.map_or(0.0, |p| (v - p) / dt);
                    *prev = Some(v);
                    let nominal = -(gs / 60.0) * gp_ft_per_nm();
                    let (roundout, flare_vs) = arrest_targets_fpm(nominal);
                    p.step(if flare { flare_vs } else { roundout }, v, trend, dt, flare)
                }
            }
        }
    }

    struct Landed {
        wheels_fpm: f64,
        secs: f64,
        /// The steepest climb after the first three seconds, fpm.
        climb_fpm: f64,
    }

    /// The arrest from the gate to the wheels (or 25 s) at the loop's tick,
    /// the servos kept: the flare below its gate, and a flare stays a flare
    /// to the band's top.
    fn land(p: &mut Plant, law: &mut Law, rough: bool) -> Landed {
        let start = p.t;
        let (mut next, mut flaring) = (p.t, false);
        let mut climb: f64 = f64::MIN;
        while p.h > 0.0 && p.t - start < 25.0 {
            if p.t + 1e-9 >= next {
                next += TICK_S;
                let gs = p.ias + 3.0;
                let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
                flaring = p.h <= flare_ft || (flaring && p.h <= band_ft);
                let e = law.step(p.h, gs, p.v, TICK_S, flaring, rough);
                p.write(e);
            }
            p.step_with(0.01, None);
            if p.t - start > 3.0 {
                climb = climb.max(p.v);
            }
        }
        Landed {
            wheels_fpm: if p.h <= 0.0 { p.v } else { f64::NAN },
            secs: p.t - start,
            climb_fpm: climb,
        }
    }

    /// The plant is the recording: the floated approach's column as written
    /// gives back its sink, the climb and the float included, under 25 fpm
    /// rms on its own fit (the other arrests' fits are other arrests).
    #[test]
    fn the_a330_plant_replays_the_float() {
        for (i, fit) in PLANTS.iter().enumerate() {
            // As fitted: nothing in the pipe before the first write.
            let mut p = Plant::at(*fit, (GATES[FLOATED].0, GATES[FLOATED].1, 0.0, FLOAT[0].4));
            p.written.clear();
            let mut sq = 0.0;
            for (j, r) in FLOAT.iter().enumerate().skip(1) {
                let prev = FLOAT[j - 1];
                while p.t < r.0 - 1e-9 {
                    if p.written.last().map(|w| w.1) != Some(prev.3) {
                        p.write(prev.3);
                    }
                    // The recorded speed, as the fit was made on it.
                    let f = ((p.t - prev.0) / (r.0 - prev.0)).clamp(0.0, 1.0);
                    p.step_with(0.01, Some(prev.4 + (r.4 - prev.4) * f));
                }
                sq += (p.v - r.2).powi(2);
            }
            let rms = (sq / (FLOAT.len() - 1) as f64).sqrt();
            if i == FLOATED {
                assert!(rms < 25.0, "its own fit: {rms:.1} fpm rms");
            }
        }
    }

    /// From every recorded gate on every fit, the declared model lands in
    /// still air under 250 fpm inside 13 s and never climbs; the tracker the
    /// A330 flew floats the floated approach's own plant past 15 s.
    #[test]
    fn the_a330_flare_lands_every_gate_on_every_plant() {
        for gate in GATES {
            for fit in PLANTS {
                let mut p = Plant::at(fit, gate);
                let l = land(&mut p, &mut Law::today(), false);
                assert!(l.wheels_fpm > -250.0, "{fit:?} from {gate:?}: {:.0}", l.wheels_fpm);
                assert!(l.secs < 13.0, "{fit:?} from {gate:?}: floated {:.1} s", l.secs);
                assert!(l.climb_fpm < 50.0, "{fit:?} from {gate:?}: climbed {:.0}", l.climb_fpm);
            }
        }
        let mut p = Plant::at(PLANTS[FLOATED], GATES[FLOATED]);
        let l = land(&mut p, &mut Law::as_flown(), false);
        assert!(l.secs > 15.0, "the tracker as flown: {:.1} s", l.secs);
        assert!(l.climb_fpm > 150.0, "and its balloon: {:.0} fpm", l.climb_fpm);
    }

    /// Rough air, every gate and plant, five gust seeds each at 40 fpm rms:
    /// the model lands them all, no more than one in twenty past 400 fpm,
    /// none past 560.
    #[test]
    fn in_rough_air_the_a330_lands_its_prediction() {
        let (mut worst, mut hard, mut n) = (0.0_f64, 0, 0);
        for gate in GATES {
            for fit in PLANTS {
                for seed in 1..=5u64 {
                    let mut p = Plant::at(fit, gate).rough(seed, 40.0);
                    let l = land(&mut p, &mut Law::today(), true);
                    assert!(l.wheels_fpm.is_finite(), "{fit:?} seed {seed}: never landed");
                    worst = worst.min(l.wheels_fpm);
                    hard += usize::from(l.wheels_fpm < -400.0);
                    n += 1;
                }
            }
        }
        assert!(worst > -560.0, "{worst:.0}");
        assert!(hard * 20 <= n, "{hard} of {n} past 400");
    }

    /// Only a declared rate plant flies the rate law; the 737's standing
    /// plant is untouched, and the rate law pushes a float down.
    #[test]
    fn a_rate_plant_pushes_a_float_down() {
        let mut m = FlareModel::from_profile(Some(Law::a330_elevator())).expect("model");
        // Level at 48 ft, the float's own numbers: the column comes forward.
        let mut e = 0.0;
        for _ in 0..5 {
            e = m.step(48.0, 160.0, 20.0, TICK_S, true, true).elev;
        }
        assert!(e < -0.05, "a float is pushed: {e:.3}");
        let p = Law::a330_elevator();
        assert!(p.model.is_some_and(|m| m.rate));
        let b738 = crate::aircraft::load("boeing_737_800")
            .and_then(|p| p.flare_elevator)
            .and_then(|f| f.model)
            .expect("the 737's model");
        assert!(!b738.rate);
    }

    /// The commit that lasted one tick: 46.4 ft, 1,432 m ahead of a 4,750 ft
    /// (1,448 m) book, then 46.1 ft and about 1,409 m. Judged afresh, the
    /// second tick is a go-around; latched, the landing continues while the
    /// pavement covers the book's ground roll, and not past it.
    #[test]
    fn a_committed_flare_stays_committed() {
        let book = Some(4750.0 * M_PER_FT);
        let over = Guidance {
            target_heading_deg: 120.0,
            target_vs_fpm: 1000.0,
            target_ias_kts: 150.0,
            dist_nm: 0.7,
            glidepath_alt_ft: 0.0,
            phase: ApproachPhase::Intercept,
            balk: Some("overflew_the_runway"),
            arresting: false,
        };
        let first = commit_flare(over, true, 46.4, 150.0, 25.0, 1432.0, book, 297.7);
        assert_eq!((first.phase, first.balk), (ApproachPhase::Flare, None));
        let fresh = commit_flare(over, true, 46.1, 150.0, 25.0, 1409.0, book, 297.7);
        assert_eq!(fresh.balk, Some("overflew_the_runway"), "as flown: released");
        let held = commit_flare_latched(over, true, true, 46.1, 150.0, 25.0, 1409.0, book, 297.7);
        assert_eq!((held.phase, held.balk), (ApproachPhase::Flare, None));
        assert!(held.arresting);
        // Past the ground roll (1,448 m less the 291 m air segment): around.
        let short = commit_flare_latched(over, true, true, 30.0, 150.0, 25.0, 1100.0, book, 297.7);
        assert_eq!(short.balk, Some("overflew_the_runway"));
        // Never above the band, latched or not.
        let (_, band) = arrest_gates_ft(25.0, 150.0);
        let high =
            commit_flare_latched(over, true, true, band + 1.0, 150.0, 25.0, 2000.0, book, 297.7);
        assert_eq!(high.balk, Some("overflew_the_runway"));
    }

    /// The arrest's lateral from a gate (x m, sideways m/s, bank deg,
    /// seconds to the wheels, gate ft) on a roll-rate plant (`k` deg/s per
    /// unit after `dead` s, a 0.07 s lag; the sideways push through 0.2 s):
    /// (offset at the wheels, worst overshoot, the bank's wobble about its
    /// own 2 s average, degrees rms).
    fn fly_roll(
        wings: &mut crate::autotakeoff::WingsLevel,
        gate: (f64, f64, f64, f64, f64),
        k: f64,
        dead: f64,
    ) -> (f64, f64, f64) {
        let (x0, v0, phi0, t_end, radio0) = gate;
        let (tick, h) = (0.2, 0.01);
        let (mut x, mut v, mut phi, mut p) = (x0, v0, phi0, 0.0_f64);
        let mut acc = 9.81 * phi0.to_radians().tan();
        let mut written: Vec<(f64, f64)> = Vec::new();
        let (mut t, mut next, mut worst) = (0.0, 0.0, 0.0_f64);
        let mut phis = Vec::new();
        while t < t_end {
            if t + 1e-9 >= next {
                next += tick;
                let radio = radio0 * (1.0 - t / t_end);
                let tgt = arrest_bank_target(x, v, wings.predicted(phi), radio);
                written.push((t, wings.step(phi, tgt, tick)));
            }
            let a = written
                .iter()
                .take_while(|w| w.0 <= t - dead + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            p += (k * a - p) * h / 0.07;
            phi += p * h;
            acc += (9.81 * phi.to_radians().tan() - acc) * h / 0.2;
            v += acc * h;
            x += v * h;
            t += h;
            worst = worst.max(-x * x0.signum());
            phis.push(phi);
        }
        let w = 200;
        let (mut sq, mut n) = (0.0, 0);
        for i in (w + w / 2)..phis.len().saturating_sub(w / 2) {
            let mean: f64 = phis[i - w / 2..i + w / 2].iter().sum::<f64>() / w as f64;
            sq += (phis[i] - mean).powi(2);
            n += 1;
        }
        (x, worst, (sq / n.max(1) as f64).sqrt())
    }

    /// #180's roll: the A330's sidestick asks a roll rate (fitted 11 to 13
    /// deg/s per unit after 0.25 to 0.37 s; 9 to 20 and up to 0.6 s bound
    /// it). From every recorded gate, the stiff wings-level law rings on the
    /// stronger, later plants (the recorded +/-2 to 4 degrees at 1.5 to 2 s);
    /// the declared rate-plant law wobbles under half as much and lands
    /// within 2 m of the line. The floated approach's gate (8 m right,
    /// crossing left at 3.8 m/s, banked 3.4 the wrong way) is stopped by the
    /// 5 degree cap, not the law: no further past the line than the stiff
    /// law plus 1.5 m.
    #[test]
    fn the_a330_arrest_wings_do_not_ring() {
        let gates = [
            (0.55, 0.25, -1.47, 17.4, 87.9),
            (-7.5, 1.23, 5.19, 17.9, 86.4),
            (7.96, -3.8, -3.37, 17.3, 86.9),
            (1.07, 0.21, 0.6, 20.3, 90.8),
        ];
        let rate = crate::aircraft::load("airbus_a330_300")
            .and_then(|p| p.arrest_roll)
            .and_then(crate::aircraft::ArrestRollProfile::valid)
            .expect("the A330 declares its roll");
        let stiff = || crate::autotakeoff::WingsLevel::stiff(ARREST_FULL_AIL_BANK_DEG);
        let (mut stiff_ring, mut new_ring) = (0.0_f64, 0.0_f64);
        for (i, gate) in gates.iter().enumerate() {
            for (k, dead) in [(13.0, 0.35), (20.0, 0.35), (13.0, 0.6), (20.0, 0.6), (9.0, 0.35)] {
                let (_, s_worst, s_ring) = fly_roll(&mut stiff(), *gate, k, dead);
                let mut w = crate::autotakeoff::WingsLevel::rate_plant(rate);
                let (x, worst, ring) = fly_roll(&mut w, *gate, k, dead);
                stiff_ring = stiff_ring.max(s_ring);
                new_ring = new_ring.max(ring);
                assert!(x.abs() < 2.0, "gate {i} at {k}/{dead}: lands {x:.1} m off");
                if i == 2 {
                    assert!(worst < s_worst + 1.5, "gate {i}, {k}/{dead}: {worst:.1}/{s_worst:.1}");
                } else {
                    assert!(worst < 4.0, "gate {i} at {k}/{dead}: overshoots {worst:.1}");
                }
            }
        }
        assert!(stiff_ring > 1.5, "the stiff law rings: {stiff_ring:.2}");
        assert!(new_ring < 0.5 * stiff_ring, "{new_ring:.2} against {stiff_ring:.2}");
    }

    /// A slow tick is a slower loop: the iMac's 19 fps stretched the
    /// arrest's tick to 0.4 to 0.44 s, and the stiff wings-level law, set
    /// on 0.25 to 0.3, rang +/-5 degrees on the aileron's stops through the
    /// 737's flare (EGKK 08R 1790687142, 2026-09-29: 12 of 17 flare ticks
    /// at 0.5). On the arrest test's 737 roll plant (20 deg/s per unit,
    /// 0.3 s lag, 0.3 s dead) the gain now softens with the tick.
    #[test]
    fn a_slow_tick_does_not_ring_the_arrest_wings() {
        let fly = |tick: f64| -> (f64, usize) {
            let mut wings = arrest_wings();
            let (x0, v0, phi0, t_end, radio0): (f64, f64, f64, f64, f64) =
                (1.2, -0.3, 3.9, 9.0, 60.0);
            let (h, k, lag, dead) = (0.01, 20.0, 0.3, 0.3);
            let (mut x, mut v, mut phi, mut p) = (x0, v0, phi0, 0.0_f64);
            let mut acc = 9.81 * phi0.to_radians().tan();
            let mut written: Vec<(f64, f64)> = Vec::new();
            let (mut t, mut next) = (0.0, 0.0);
            let (mut swing, mut sat) = (0.0_f64, 0usize);
            while t < t_end {
                if t + 1e-9 >= next {
                    next += tick;
                    let radio = radio0 * (1.0 - t / t_end);
                    let tgt = arrest_bank_target(x, v, wings.predicted(phi), radio);
                    let a = wings.step(phi, tgt, tick);
                    sat += usize::from(a.abs() > 0.49);
                    written.push((t, a));
                }
                let a = written
                    .iter()
                    .take_while(|w| w.0 <= t - dead + 1e-9)
                    .last()
                    .map_or(0.0, |w| w.1);
                p += (k * a - p) * h / lag;
                phi += p * h;
                acc += (9.81 * phi.to_radians().tan() - acc) * h / 0.2;
                v += acc * h;
                x += v * h;
                t += h;
                if t > 3.0 {
                    swing = swing.max(phi.abs());
                }
            }
            (swing, sat)
        };
        let (fast, _) = fly(0.25);
        for tick in [0.4, 0.44] {
            let (swing, sat) = fly(tick);
            assert_eq!(sat, 0, "tick {tick}: {sat} ticks on the stops");
            assert!(swing < 2.0, "tick {tick}: swings {swing:.1} degrees");
        }
        assert!(fast < 2.0, "the tuned tick is the one it was: {fast:.1}");
    }
}

/// #199: the missed approach flies a speed.
#[cfg(test)]
mod missed_approach_speed_tests {
    use super::*;

    /// The stock 737's detents (`profiles/boeing_737_800.yaml`).
    const B738: [(f64, Option<f64>); 9] = [
        (0.0, None),
        (0.125, Some(250.0)),
        (0.25, Some(250.0)),
        (0.375, Some(250.0)),
        (0.5, Some(210.0)),
        (0.625, Some(200.0)),
        (0.75, Some(190.0)),
        (0.875, Some(175.0)),
        (1.0, Some(162.0)),
    ];
    /// The 737's floors: clean 1.2 x 139, landing 1.2 x 114, the half flap's
    /// between them.
    const CLEAN_FLOOR: f64 = 167.0;
    const HALF_FLOOR: f64 = 152.0;
    const LANDING_FLOOR: f64 = 137.0;

    #[test]
    fn the_clean_737s_missed_approach_flies_210_not_355() {
        // EGNX 09 (2026-09-30): clean, 1,500 ft.
        let clean = missed_approach_speed_kts(
            145.0,
            0.0,
            &B738,
            Some(HALF_FLOOR),
            Some(CLEAN_FLOOR),
            1500.0,
        );
        assert!((clean - 210.25).abs() < 0.01, "{clean}");
        // Landing flap out: under its placard, fast enough to clean up to
        // the go-around flap (the half flap's floor plus 5).
        let full = missed_approach_speed_kts(
            145.0,
            1.0,
            &B738,
            Some(HALF_FLOOR),
            Some(LANDING_FLOOR),
            1500.0,
        );
        assert_eq!(full, 152.0);
        assert!(go_around_flap_due(full, Some(HALF_FLOOR)));
        // A faster airframe's clean speed is still 250 below 10,000 ft...
        let fast = missed_approach_speed_kts(190.0, 0.0, &[], None, None, 3000.0);
        assert_eq!(fast, 250.0);
        // ...and not above it.
        let high = missed_approach_speed_kts(190.0, 0.0, &[], None, None, 11_000.0);
        assert!((high - 275.5).abs() < 0.01);
        // The floor of the configuration flown wins everything.
        assert_eq!(missed_approach_speed_kts(100.0, 0.0, &[], None, Some(180.0), 3000.0), 180.0);
    }

    #[test]
    fn go_around_thrust_until_established_then_the_speed() {
        let floor = Some(0.3);
        // The balk at 150 ft, climbing 883 fpm at 248 kt: go-around thrust.
        assert_eq!(missed_approach_throttle(0.1, 248.0, 2.0, 210.0, 150.0, Some(883.0), floor), 1.0);
        // An unread climb is not established.
        assert_eq!(missed_approach_throttle(1.0, 277.0, 3.0, 210.0, 600.0, None, floor), 1.0);
        // 426 ft, 2,476 fpm up, 277 kt: established and 67 kt fast, the
        // climb floor at once (the tick EGNX 09 was still at 1.0).
        assert_eq!(missed_approach_throttle(1.0, 277.0, 3.0, 210.0, 426.0, Some(2476.0), floor), 0.3);
        // Idle where no floor is declared.
        assert_eq!(
            missed_approach_throttle(1.0, 277.0, 3.0, 210.0, 426.0, Some(2476.0), None),
            IDLE_THROTTLE
        );
        // Near the speed, the governor's step inside the floor and full.
        let near = missed_approach_throttle(0.5, 214.0, -1.0, 210.0, 900.0, Some(1500.0), floor);
        assert!((0.3..=1.0).contains(&near) && (near - 0.5).abs() <= 0.041, "{near}");
        // Slow by more than 10: go-around thrust again.
        assert_eq!(missed_approach_throttle(0.3, 198.0, -2.0, 210.0, 900.0, Some(1500.0), floor), 1.0);
    }

    /// A crude clean 737 in a go-around climb (kt/s = 5 thr - 1.5 - 0.5 x
    /// climb per 1,000 fpm, the lever spooling at 2.5 s): TOGA gains the 3
    /// kt/s the EGNX 09 miss did at 2,400 fpm. From the balk at 150 ft and
    /// 250 kt the climb is held at the VS mode's 2,000 fpm.
    #[test]
    fn the_egnx_missed_approach_replayed_stays_under_250() {
        let (mut v, mut h, mut vs, mut thr, mut lever) =
            (250.0_f64, 150.0_f64, 0.0_f64, 1.0_f64, 1.0_f64);
        let mut prev_v = v;
        let dt = 0.5;
        let mut peak = v;
        let mut t = 0.0;
        while t < 120.0 {
            let trend = (v - prev_v) / dt;
            prev_v = v;
            let target = missed_approach_speed_kts(
                145.0,
                0.0,
                &B738,
                Some(HALF_FLOOR),
                Some(CLEAN_FLOOR),
                300.0 + h,
            );
            thr = missed_approach_throttle(thr, v, trend, target, h, Some(vs), Some(0.3));
            lever += (thr - lever) * (dt / 2.5);
            vs += (2000.0 - vs) * (dt / 4.0);
            v += (5.0 * lever - 1.5 - 0.5 * vs / 1000.0) * dt;
            h += vs / 60.0 * dt;
            peak = peak.max(v);
            t += dt;
        }
        assert!(peak < 285.0, "the peak {peak:.0} kt (355 flown)");
        assert!((v - 210.0).abs() < 15.0, "settles on the go-around speed: {v:.0}");
    }
}

/// Issue #197, the rollout: LEMG 13, the King Air C90B (iMac 2026-09-30,
/// sorties 1790757702, 1790758429 and 1790758604 from 4 nm), steered by the
/// takeoff roll's default law through the rudder and nosewheel (7 degrees of
/// nosewheel at speed, 48 slow; `_gear_castors 0`, so not a brake-steer
/// airframe). Every contact yawed the nose 8 to 15 deg/s left with the steer
/// at zero; the default law answered with its whole 0.35 at 80 kt for a
/// second and a half, the nose swung 30 to 80 degrees right, and full left
/// steer held none of it until 45 kt: 1790757702 left the runway 176 m right,
/// 1790758604 125.8 m, 1790758429 swung 24 degrees and came back at 19.7 m.
///
/// The recorded heading and track ARE the plant here: three fits over the
/// first three and a half to four and a half seconds on the wheels, each
/// with the steer as written, the contact's own yaw as its first state, and a
/// yaw the slip between nose and track feeds (A, over 4.5 s: 62 deg/s per
/// unit at 80 kt, 1.5 per degree of slip, 0.41 s, 0.29 s dead; B, over 3.5
/// s: 43, 2.3, 0.08 s, 0.30 s; C, over 3.5 s with no slip term: a near
/// integrator of 219 through 3 s after 0.32 s; 1.2 to 2.4 degrees rms).
/// Below 40 kt the nosewheel's own steering: 12 to 40 deg/s per unit after
/// 0.3 to 0.5 s.
#[cfg(test)]
mod lemg13_king_air_rollout_tests {
    use super::*;

    const SRC: [&str; 3] = [
        include_str!("../bench/landings/king_air_lemg13_landing_1790757702.jsonl"),
        include_str!("../bench/landings/king_air_lemg13_landing_1790758429.jsonl"),
        include_str!("../bench/landings/king_air_lemg13_landing_1790758604.jsonl"),
    ];
    /// The loop's tick on the iMac's rollouts.
    const TICK_S: f64 = 0.45;
    /// The rollout's braking on the recorded ticks, kt/s (86 to 60 kt in 4 s).
    const DECEL_KTS_S: f64 = 6.0;

    /// A rollout tick: (t from the first, gs, psi, track, steer written,
    /// cross-track m, cross-track rate m/s, the error the law saw).
    type Tick = (f64, f64, f64, f64, f64, f64, f64, f64);

    fn ticks(src: &str) -> Vec<Tick> {
        let mut out: Vec<Tick> = vec![];
        let mut t0 = None;
        for l in src.lines().filter(|l| !l.trim().is_empty()) {
            let r: serde_json::Value = serde_json::from_str(l).expect("a record");
            if r["event"] != "rollout" {
                continue;
            }
            let t = r["t"].as_f64().unwrap();
            let t0 = *t0.get_or_insert(t);
            let f = |k: &str| r[k].as_f64().unwrap();
            out.push((t - t0, f("gs"), f("psi"), f("trk"), f("steer"), f("xtk_m"), f("xtk_rate"), f("err")));
        }
        out
    }

    /// The course the rollout steered for: the recorded error on the first
    /// tick plus the centreline cut it held (LEMG 13's true course).
    fn course(t: &Tick) -> f64 {
        let cut = -crate::autotakeoff::roll_target_heading(0.0, t.5, t.6, t.1);
        let cut = (cut + 540.0).rem_euclid(360.0) - 180.0;
        (t.2 + t.7 + cut).rem_euclid(360.0)
    }

    /// The yaw on the wheels: the yaw rate goes toward ks x (gs/80)^2 x the
    /// steer `dead` s ago plus kb x the slip x (gs/80)^n, over `tau`; the
    /// track follows the nose at `lam` per second of slip.
    #[derive(Debug, Clone, Copy)]
    struct Yaw {
        ks: f64,
        kb: f64,
        n: f64,
        tau: f64,
        dead: f64,
        lam: f64,
        /// The contact's yaw rate, deg/s, per recorded rollout.
        r0: [f64; 3],
    }

    const A: Yaw = Yaw { ks: 62.5, kb: 1.52, n: 0.52, tau: 0.41, dead: 0.29, lam: 0.61, r0: [-16.8, -16.7, -18.8] };
    const B: Yaw = Yaw { ks: 42.6, kb: 2.26, n: 3.0, tau: 0.08, dead: 0.30, lam: 0.69, r0: [-40.0, -5.8, -24.8] };
    const C: Yaw = Yaw { ks: 219.0, kb: 0.0, n: 1.0, tau: 3.0, dead: 0.32, lam: 0.71, r0: [-6.7, -10.6, -13.3] };

    struct Sim {
        y: Yaw,
        psi: f64,
        trk: f64,
        r: f64,
        gs: f64,
        xtk: f64,
        course: f64,
        writes: Vec<(f64, f64)>,
        t: f64,
    }

    impl Sim {
        fn new(y: Yaw, first: &Tick, r0: f64, course: f64) -> Self {
            Sim {
                y,
                psi: first.2,
                trk: first.3,
                r: r0,
                gs: first.1,
                xtk: first.5,
                course,
                writes: vec![(-9.0, 0.0)],
                t: 0.0,
            }
        }

        fn step(&mut self, dt: f64, decel: Option<f64>) {
            let due = self.t - self.y.dead;
            let u = self
                .writes
                .iter()
                .take_while(|w| w.0 <= due + 1e-9)
                .last()
                .map_or(0.0, |w| w.1);
            let q = self.gs / 80.0;
            let slip = self.psi - self.trk;
            self.r += (self.y.ks * q * q * u + self.y.kb * slip * q.powf(self.y.n) - self.r) / self.y.tau
                * dt;
            self.psi += self.r * dt;
            self.trk += self.y.lam * slip * dt;
            self.xtk += self.gs / MS_TO_KTS * (self.trk - self.course).to_radians().sin() * dt;
            if let Some(a) = decel {
                self.gs -= a * dt;
            }
            self.t += dt;
        }
    }

    /// The plant is the recording: each rollout's steer as written, open
    /// loop, gives back its heading over its first three and a half seconds
    /// within 3.5 degrees rms on the slip fits and 5 on the one without
    /// (1.4 to 3.2 and 2.2 to 4.6 as fitted).
    #[test]
    fn the_king_air_yaw_plants_replay_the_rollouts() {
        for y in [A, B, C] {
            for (i, src) in SRC.iter().enumerate() {
                let tk = ticks(src);
                let mut s = Sim::new(y, &tk[0], y.r0[i], course(&tk[0]));
                let (mut se, mut n) = (0.0, 0.0);
                for w in tk.iter().filter(|w| w.0 <= 3.5) {
                    while s.t < w.0 - 1e-9 {
                        s.step(0.01, None);
                    }
                    se += (s.psi - w.2).powi(2);
                    n += 1.0;
                    s.gs = w.1;
                    s.writes.push((w.0, w.4));
                }
                let rms = (se / n).sqrt();
                eprintln!("{y:?} rollout {i}: {rms:.2} deg rms");
                let limit = if y.kb > 0.0 { 3.5 } else { 5.0 };
                assert!(rms < limit, "{y:?} rollout {i}: {rms:.2} deg rms");
            }
        }
    }

    /// One closed-loop rollout from a recorded contact down to 40 kt: the
    /// worst heading off the course and the worst cross-track.
    fn roll(y: Yaw, i: usize, mut law: impl FnMut(f64, f64, f64, f64) -> f64) -> (f64, f64) {
        let tk = ticks(SRC[i]);
        let c = course(&tk[0]);
        let mut s = Sim::new(y, &tk[0], y.r0[i], c);
        let mut rate = crate::autotakeoff::XtkRate::default();
        let mut prev = s.xtk;
        let (mut worst_h, mut worst_x) = (0.0_f64, s.xtk.abs());
        let mut next = 0.0;
        while s.gs > 40.0 {
            if s.t + 1e-9 >= next {
                let xr = if next > 0.0 {
                    rate.update((s.xtk - prev) / TICK_S, TICK_S)
                } else {
                    tk[0].6
                };
                prev = s.xtk;
                next += TICK_S;
                let target = crate::autotakeoff::roll_target_heading(c, s.xtk, xr, s.gs);
                let err = ((target - s.psi + 540.0).rem_euclid(360.0)) - 180.0;
                let u = law(err, TICK_S, s.gs, s.r);
                s.writes.push((s.t, u));
            }
            s.step(0.01, Some(DECEL_KTS_S));
            worst_h = worst_h.max((s.psi - c).abs());
            worst_x = worst_x.max(s.xtk.abs());
            if worst_h > 90.0 {
                break;
            }
        }
        (worst_h, worst_x)
    }

    fn king_air_law() -> RolloutSteer {
        let p = crate::aircraft::load("king_air_c90b").expect("the King Air profile loads");
        RolloutSteer::new(
            p.rollout_steer
                .and_then(crate::aircraft::RolloutSteerProfile::valid)
                .expect("the King Air declares its rollout steer"),
        )
    }

    /// The takeoff roll's default law as it flew the three rollouts swings
    /// the nose past 20 degrees from every contact on every fit, and on the
    /// best fit (A) loses it past 45 from two of the three, as the sim did;
    /// the declared law from every contact on every fit holds the nose
    /// within 15 degrees of the course and the wheels within 15 m of the
    /// centreline (LEMG 13 is 45 m wide).
    #[test]
    fn the_king_air_rollout_holds_the_runway_where_the_roll_law_lost_it() {
        let mut lost_on_a = 0;
        let mut flown_least = f64::INFINITY;
        let mut worst = (0.0_f64, 0.0_f64);
        for (name, y) in [("A", A), ("B", B), ("C", C)] {
            for i in 0..3 {
                let mut flown = crate::autotakeoff::RollSteer::default();
                let (fh, fx) = roll(y, i, |e, dt, gs, _| flown.step(e, dt, gs, 0.0));
                flown_least = flown_least.min(fh);
                if name == "A" && fh > 45.0 {
                    lost_on_a += 1;
                }
                let mut law = king_air_law();
                let (h, x) = roll(y, i, |e, dt, gs, r| law.step(e, dt, gs, Some(r)));
                eprintln!("{name} {i}: as flown {fh:.1} deg, {fx:.1} m; declared {h:.1} deg, {x:.1} m");
                assert!(h < 15.0 && x < 15.0, "{name} {i}: {h:.1} deg, {x:.1} m");
                worst = (worst.0.max(h), worst.1.max(x));
            }
        }
        assert!(flown_least > 20.0, "the roll law as flown held one within {flown_least:.1} degrees");
        assert!(lost_on_a >= 2, "the roll law as flown lost {lost_on_a} of 3 on the best fit");
        eprintln!(
            "King Air rollout: as flown at least {flown_least:.1} deg; declared worst {:.1} deg, {:.1} m",
            worst.0, worst.1
        );
    }

    /// Slow, the nosewheel's own steering (12 to 40 deg/s per unit after 0.3
    /// to 0.5 s): the default law's full authority per 12 degrees sawed
    /// +/-10 to 15 degrees at 10 to 30 kt on the recorded ticks, and saws
    /// past 10 on the 40 deg/s nosewheel here; the declared law settles
    /// within 4 degrees after four seconds on every one.
    #[test]
    fn the_king_air_rollout_settles_on_the_nosewheel() {
        fn fly(ks: f64, tau: f64, dead: f64, e0: f64, r0: f64, law: &mut dyn FnMut(f64, f64, f64, f64) -> f64) -> f64 {
            let (mut psi, mut r, mut gs, mut t, mut next) = (-e0, r0, 40.0_f64, 0.0, 0.0);
            let mut writes = vec![(-9.0, 0.0)];
            let mut late = 0.0_f64;
            while gs > 5.0 {
                if t + 1e-9 >= next {
                    next += TICK_S;
                    writes.push((t, law(-psi, TICK_S, gs, r)));
                }
                let u = writes
                    .iter()
                    .take_while(|w: &&(f64, f64)| w.0 <= t - dead + 1e-9)
                    .last()
                    .map_or(0.0, |w| w.1);
                r += (ks * u - r) / tau * 0.01;
                psi += r * 0.01;
                gs -= 3.0 * 0.01;
                t += 0.01;
                if t > 4.0 {
                    late = late.max(psi.abs());
                }
            }
            late
        }
        for (ks, tau, dead) in [(20.0, 0.2, 0.4), (40.0, 0.3, 0.5), (12.0, 0.15, 0.3)] {
            for (e0, r0) in [(10.0, 0.0), (0.0, 15.0), (-8.0, -10.0)] {
                let mut flown = crate::autotakeoff::RollSteer::default();
                let as_flown = fly(ks, tau, dead, e0, r0, &mut |e, dt, gs, _| flown.step(e, dt, gs, 0.0));
                let mut law = king_air_law();
                let declared = fly(ks, tau, dead, e0, r0, &mut |e, dt, gs, r| law.step(e, dt, gs, Some(r)));
                eprintln!("slow {ks} {tau} {dead} from {e0} {r0}: as flown {as_flown:.1}, declared {declared:.1}");
                assert!(declared < 4.0, "{ks} {tau} {dead} {e0} {r0}: {declared:.1}");
                if ks >= 40.0 {
                    assert!(as_flown > 10.0, "as flown {ks} {e0} {r0}: {as_flown:.1}");
                }
            }
        }
    }

    /// The law's shape: full authority slow, `max_fast` at speed, the gain
    /// rising as the speed falls to `min_kts`, damped on the yaw rate
    /// against the error, nothing carried through the air, and only the
    /// King Air declares it.
    #[test]
    fn rollout_steer_schedules_and_damps() {
        let mut law = king_air_law();
        assert!((law.cap_at(85.0) - law.prof.max_fast).abs() < 1e-9);
        assert!((law.cap_at(20.0) - ROLLOUT_STEER_MAX).abs() < 1e-9);
        assert!(law.per_deg_at(40.0) > law.per_deg_at(80.0));
        assert!((law.per_deg_at(120.0) - law.per_deg_at(80.0)).abs() < 1e-12);
        // The nose swinging left at 17 deg/s with no error yet: right steer.
        assert!(law.step(0.0, 0.45, 85.0, Some(-17.0)) > 0.0);
        law.in_air();
        assert_eq!(law.out, 0.0);
        for id in [
            "cirrus_sr22",
            "boeing_737_800",
            "cirrus_vision_sf50",
            "lancair_evolution",
            "beechcraft_baron_58",
            "airbus_a330_300",
        ] {
            let p = crate::aircraft::load(id).expect("the profile loads");
            assert!(p.rollout_steer.is_none(), "{id}");
        }
    }
}
