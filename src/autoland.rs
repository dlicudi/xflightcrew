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
    if !ACTIVE.load(std::sync::atomic::Ordering::Relaxed) { return None; }
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
                for e in [e1, e2] {
                    let dist_nm = haversine_nm(lat, lon, e.lat, e.lon);
                    if dist_nm <= max_nm {
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
    best_landing_runway_in_wind(apt_dat, lat, lon, track_deg, max_nm, min_length_m, &|_| None)
}

/// The shortest strip the self-pick will take, metres: the airframe's landing
/// distance with a 1.3 margin, never under the 1,200 m every airframe was
/// held to. The flat 1,200 offered an A330 (1,448 m before any margin) strips
/// it cannot stop on; the planned path has checked the distance at the arm
/// since 2026-09-15 and the emergency path never did.
pub const SELF_PICK_MIN_LENGTH_M: f64 = 1200.0;
pub fn self_pick_min_length_m(landing_distance_m: Option<f64>) -> f64 {
    landing_distance_m
        .filter(|d| d.is_finite() && *d > 0.0)
        .map_or(SELF_PICK_MIN_LENGTH_M, |d| (d * 1.3).max(SELF_PICK_MIN_LENGTH_M))
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
        )
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
    let picked =
        best_landing_runway_in_wind(apt_dat, lat, lon, track_deg, 40.0, min_length_m, &wind_at)?;
    crate::blackbox::record(
        "autoland",
        serde_json::json!({
            "event": "self_pick", "min_length_m": min_length_m,
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
/// The gentler brake hold once stopped.
const ROLLOUT_BRAKE_HOLD: f64 = 0.5;
/// Throttle a give-up hands back with: enough to hold level flight with gear
/// down and landing flap out, which is the state a balked landing leaves the
/// aircraft in. Half throttle was not (110 → 80 kt, EGHI 2026-08-02).
const GIVE_UP_THROTTLE: f64 = 0.75;
/// The VS watchdog: ticks of >700 fpm divergence before the warning (about
/// five seconds) and before the give-up (about a minute — long enough for a
/// re-engaged autopilot to catch the dial, short enough that a descent that
/// is really a climb does not become a cruise).
const VS_DIVERGED_WARN_TICKS: u32 = 12;
/// How far the vertical speed may sit from the command before the watchdog
/// looks at it, and how fast the gap must be CLOSING for the autopilot to
/// count as holding. The gap alone is a 172 number: the A330's VS mode takes
/// longer than the watchdog's five seconds to reach a -1,200 step from
/// level, and "not holding the descent" was said at the arm of an approach
/// the autopilot was flying correctly, just slowly (EGPF 23, 2026-09-20).
pub const VS_DIVERGED_FPM: f64 = 700.0;
pub const VS_CONVERGING_FPM_PER_TICK: f64 = 15.0;

/// Is the autopilot NOT holding the commanded vertical speed? Far from it,
/// and not closing on it. `prev_err` is last tick's error (None = first
/// sight, which counts: there is no trend to excuse it yet).
pub fn vs_not_held(err_fpm: f64, prev_err_fpm: Option<f64>) -> bool {
    if err_fpm.abs() <= VS_DIVERGED_FPM {
        return false;
    }
    prev_err_fpm.is_none_or(|p| err_fpm.abs() > p.abs() - VS_CONVERGING_FPM_PER_TICK)
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
const VS_DIVERGED_GIVE_UP_TICKS: u32 = 140;
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
    if !xp.get_float(SERVOS_ON).await.is_ok_and(|v| v > 0.5) {
        return None;
    }
    let ok = xp.run_command(CMD_SERVOS_OFF).await.is_ok()
        && xp.get_float(SERVOS_ON).await.is_ok_and(|v| v < 0.5);
    crate::blackbox::record(
        "autoland",
        serde_json::json!({"event": "ap_disconnected", "ok": ok}),
    );
    Some(ok)
}

/// Rollout starts only after weight-on-wheels. The speed terms distinguish a
/// landing from a low pass once on the surface; they must never substitute for
/// the ground signal because a normal flare can satisfy both in the air.
fn should_start_rollout(
    already_rolling: bool,
    on_ground: bool,
    radio_ft: f64,
    gs_kts: f64,
    ias_kts: f64,
    approach_ias: f64,
) -> bool {
    already_rolling || (on_ground && radio_ft < 5.0 && (gs_kts < 45.0 || ias_kts < approach_ias))
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
            announce(
                format!("Autoland to {} runway {}, {:.0} miles. Flying the approach.", rwy.icao, rwy.id, rwy.dist_nm),
                Priority::Warning,
            );

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
                let (aim, seen) = aim_elevation_ft(rwy.elev_ft, probed.as_deref());
                crate::blackbox::record("autoland", serde_json::json!({
                    "event": "threshold_elev", "rwy": format!("{} {}", rwy.icao, rwy.id),
                    "header_ft": rwy.elev_ft, "probed_ft": seen, "aim_ft": aim,
                }));
                if (aim - rwy.elev_ft).abs() > THRESHOLD_ELEV_TRUST_FT {
                    let diff = aim - rwy.elev_ft;
                    announce(
                        format!(
                            "Runway {} threshold is {:.0} feet {} the field elevation. Aiming at {:.0} feet.",
                            crate::circuit::spell_runway(&rwy.id),
                            diff.abs(),
                            if diff < 0.0 { "below" } else { "above" },
                            aim
                        ),
                        Priority::Advisory,
                    );
                    rwy.elev_ft = aim;
                }
            }

            // Engage the AP (FD up first for airframes that need it).
            for c in &cfg.fd_commands { let _ = xp.run_command(c).await; }
            let _ = xp.run_command(CMD_SERVOS_ON).await;
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
            let mut go_around = GoAroundLatch::new();
            // Drain any go-around order left on the party line before this
            // approach existed. The flag is a latch, not a queue: an order
            // shouted at somebody else's approach half an hour ago must not
            // balk this one on its first tick.
            let _ = crate::party_line::take_pending_go_around();
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
            // When the current missed approach started, for the runaway guard.
            let mut missed_since: Option<std::time::Instant> = None;
            let mut gear_down = false;
            let mut landing_config_done = false;
            let mut last_flap: Option<f64> = None;
            let mut vs_diverged: u32 = 0;
            let mut prev_vs_err: Option<f64> = None;
            // Warned once per approach when the VSI cannot be read.
            let mut vvi_read_failed = false;
            // The vertical speed one tick ago, for the arrest's trend term.
            let mut prev_vvi: Option<(f64, std::time::Instant)> = None;
            // The flare's elevator law. Its integrator is per-approach: a
            // standing pull must not survive a go-around into the next one.
            let mut flare_pitch = FlarePitch::default();
            let mut pitch_yielded = false;
            // Airspeed history for the throttle governor's derivative term — the
            // anti-stall reflex needs the TREND (kts/sec), not just the error.
            let mut prev_ias: Option<f64> = None;
            let mut prev_ias_t = std::time::Instant::now();
            // Cross-track history for the intercept damping term.
            let mut prev_xtk: Option<(f64, std::time::Instant)> = None;
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
            let mut vacate_follows = false;
            let mut roll_law = crate::autotakeoff::RollSteer::default();
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
            // How far out this approach has legitimately been: the arm
            // distance, pushed out by any pattern flown since.
            let mut recede_ref_nm = rwy.dist_nm;
            // A give-up happened: the speed is handed to the flight governor
            // once the loop has ended (see below).
            let mut gave_up = false;
            loop {
                if xp.cancelled() || GEN.load(Ordering::SeqCst) != my_gen || ABORT.load(Ordering::SeqCst) {
                    break;
                }
                // Frozen sim (replay or pause): a replay's positions are a
                // recording and not ours to fly; a pause is real but frozen,
                // and ticking the throttle governor against a frozen airspeed
                // walks the lever to its clamp for a surge on unpause. Freeze
                // (no writes, no state, no blackbox) and resume when it's live.
                if crate::xplane::sim_frozen(&xp).await {
                    tokio::time::sleep(std::time::Duration::from_millis(400)).await;
                    continue;
                }
                // One read burst — nine sequential HTTP awaits is the slow-tick
                // disease that gave the takeoff its slew weave (sortie four).
                let (lat_r, lon_r, elev_r, agl_r, gs_r, var_r, ias_r, psi_r, hpath_r, ground_r) = tokio::join!(
                    xp.get_float(LAT), xp.get_float(LON), xp.get_float(ELEV_M),
                    xp.get_float(AGL_M), xp.get_float(GS_MS), xp.get_float(MAG_VAR),
                    xp.get_float(IAS), xp.get_float(PSI_TRUE), xp.get_float(HPATH_TRUE),
                    xp.get_float(ON_GROUND),
                );
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
                    if let Some(c) = clearance_switch(
                        &rwy.id,
                        crate::party_line::clearance_state(),
                        cleared.as_deref(),
                        clearance_checked.as_deref(),
                        go_around.active(),
                    ) {
                        clearance_checked = Some(c.clone());
                        match runway_by_id(&apt_dat, lat, lon, &c, 40.0, Some(rwy.icao.as_str())) {
                            Ok(Some(r)) if r.icao == rwy.icao => {
                                crate::blackbox::record(
                                    "autoland",
                                    serde_json::json!({"event": "runway_from_clearance",
                                        "planned": rwy.id, "cleared": r.id, "in_flight": true,
                                        "radio": radio_ft, "dist": r.dist_nm}),
                                );
                                announce(
                                    format!(
                                        "Tower cleared us for runway {}, not {}. Setting up for runway {}.",
                                        crate::circuit::spell_runway(&r.id),
                                        crate::circuit::spell_runway(&rwy.id),
                                        crate::circuit::spell_runway(&r.id)
                                    ),
                                    Priority::Caution,
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
                if should_start_rollout(
                    rolling_out,
                    on_ground > 0.5,
                    radio_ft,
                    gs_kts,
                    ias,
                    approach_ias,
                ) {
                    // Consumed once, at rollout entry: a stale flag from a
                    // mission that never landed must not turn a later
                    // standalone landing into a rolling handoff to nobody.
                    if !rolling_out {
                        rolling_out = true;
                        disconnect_autopilot(&xp).await;
                        vacate_follows = VACATE_FOLLOWS.swap(false, Ordering::SeqCst);
                        // Wheels on this runway: the fact the vacate needs.
                        *LANDED_RUNWAY.lock().unwrap_or_else(|e| e.into_inner()) =
                            Some(rwy.id.clone());
                    }
                    let xtk_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
                    let target = crate::autotakeoff::roll_target_heading(rwy.course_deg, xtk_m);
                    let psi = psi_r.as_ref().copied().unwrap_or(rwy.course_deg);
                    let err = ((target - psi + 540.0).rem_euclid(360.0)) - 180.0;
                    let steer = roll_law.step(err, 0.4, gs_kts, 0.0);
                    let thr_dref = throttle_dref();
                    let (_, _) = tokio::join!(
                        xp.set_float(&thr_dref, 0.0),
                        xp.set_float(TAXI_STEER, steer),
                    );
                    // A vacate follows: hand off ROLLING at taxi pace — the
                    // completion of a landing is clearing the runway, and a
                    // dead stop on the strip followed by a re-acceleration was
                    // neither procedure nor comfortable (EGHI, 2026-08-02).
                    // The vacate drive arms the governor from the current
                    // position, which takes a rolling aircraft like any join.
                    if vacate_follows && gs_kts <= VACATE_HANDOFF_KTS {
                        let _ = xp.set_float(TAXI_BRAKE, 0.0).await;
                        let _ = xp.set_float(TAXI_STEER, 0.0).await;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({ "event": "rolling_handoff", "gs": gs_kts }),
                        );
                        break;
                    }
                    if gs_kts > 2.0 {
                        // Brake by wanted deceleration, not a fixed clamp: a
                        // comfortable rate when the runway allows it, harder
                        // only when stopping in what remains demands it.
                        let along_m = along_track_nm(lat, lon, &rwy) * M_PER_NM;
                        let remaining_m = (rwy.length_m + along_m).max(0.0);
                        let target_kts = if vacate_follows { VACATE_HANDOFF_KTS } else { 0.0 };
                        let brake = rollout_brake_ratio(gs_kts, target_kts, remaining_m);
                        let _ = xp.set_float(TAXI_BRAKE, brake).await;
                        set_panel(&format!("AUTOLAND {} {} ROLLOUT", rwy.icao, rwy.id));
                        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
                        continue;
                    }
                    // Stopped: hold the brakes, free the nosewheel, hand off.
                    let _ = xp.set_float(TAXI_BRAKE, ROLLOUT_BRAKE_HOLD).await;
                    let _ = xp.set_float(TAXI_STEER, 0.0).await;
                    announce("Down and stopped. Brakes on, your aircraft.".into(), Priority::Warning);
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
                xtk_rate = xtk_rate * 0.5 + raw_rate.clamp(-3.0, 3.0) * 0.5;

                // Wind crab, from the reference recipe (heading vs ground track):
                // command a heading whose TRACK aligns with the runway, not the
                // heading itself. P-only tracking left a standing 0.10 nm
                // cross-track under the LXGB crosswind — touchdown in the grass
                // (2026-07-22). LOW-PASSED: the raw read fed straight back to
                // the bug hunted the LEMG 13 final into a balk (2026-09-12,
                // see [`CrabFilter`]). Estimated every tick, applied on final.
                let crab_raw = CrabFilter::raw(psi_r.as_ref().ok().copied(), hpath_r.as_ref().ok().copied());
                let crab = crab_filter.update(crab_raw, dt_s.clamp(0.0, 3.0));

                let raw = guidance(lat, lon, alt, radio_ft, gs_kts, ias, &rwy, approach_ias, 25.0, xtk_rate);
                // A missed approach called low is flown by the latch (full power,
                // straight ahead) until clear of the ground — the raw intercept
                // guidance turns back to the FAP immediately, which is a killer
                // at 50 ft (see [`GoAroundLatch`]).
                let along_nm = along_track_nm(lat, lon, &rwy);
                // A controller's order beats anything the approach thinks of
                // itself. Consumed here rather than in the main loop because
                // this is the thing that can ACT on it; the flag is drained
                // at arm time above, so only an order given during this
                // approach can trip it.
                if crate::party_line::take_pending_go_around()
                    && go_around.order(on_ground > 0.5)
                {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "ordered_go_around", "radio": radio_ft,
                                           "dist": raw.dist_nm}),
                    );
                }
                // Unstabilised at the thousand-foot gate is a go-around, not a
                // quiet re-intercept: the latch only arms itself below the
                // terrain floor, so it is ordered.
                if raw.balk == Some("unstabilised_at_gate") {
                    go_around.order(on_ground > 0.5);
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
                if reversal.is_none() {
                    let aim_along_nm = if alt - raw.glidepath_alt_ft > 500.0 {
                        join_point_nm(alt, rwy.elev_ft)
                    } else {
                        FAP_NM
                    };
                    let wanted = match missed {
                        Some(MissedStage::Reposition) => true,
                        Some(MissedStage::Climb) => false,
                        None => {
                            raw.phase != ApproachPhase::Flare
                                && radio_ft >= TERRAIN_FLOOR_AGL_FT
                                && crate::reversal::needed(
                                    &rwy,
                                    lat,
                                    lon,
                                    track_true,
                                    aim_along_nm,
                                    raw.phase == ApproachPhase::Final,
                                )
                        }
                    };
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
                    if wanted {
                        reversals_flown += 1;
                        // On the centreline there is no side yet: probe the
                        // ground under both downwinds and fly the lower.
                        let r_nm = crate::reversal::sizing_radius_nm(gs_kts);
                        let default_platform = rwy.elev_ft + REPOSITION_MIN_AGL_FT;
                        let join = join_point_nm(default_platform, rwy.elev_ft);
                        let line = |side: f64| -> Vec<(f64, f64)> {
                            (0..8)
                                .map(|i| {
                                    let along = -1.0 + (join + 2.0 + r_nm) * i as f64 / 7.0;
                                    crate::reversal::pattern_point(&rwy, along, side * 3.0 * r_nm)
                                })
                                .collect()
                        };
                        let max_of = |v: Option<Vec<f64>>| {
                            v.map(|v| v.into_iter().fold(f64::NEG_INFINITY, f64::max))
                                .filter(|m| m.is_finite())
                        };
                        // One mailbox, so one after the other.
                        let (left_pts, right_pts) = (line(-1.0), line(1.0));
                        let left_ft = crate::terrain::sample_ft(&xp, &left_pts).await;
                        let right_ft = crate::terrain::sample_ft(&xp, &right_pts).await;
                        let (left_ft, right_ft) = (max_of(left_ft), max_of(right_ft));
                        let side = crate::reversal::pick_side(xtk_now, left_ft, right_ft);
                        let under = if side < 0.0 { left_ft } else { right_ft };
                        let platform_ft = under
                            .map_or(default_platform, |t| default_platform.max(t + 1000.0));
                        reversal = Some(crate::reversal::Reversal {
                            side,
                            leg: crate::reversal::Leg::Outbound,
                            platform_ft,
                        });
                        if missed == Some(MissedStage::Reposition) {
                            go_around.hold_reposition();
                        }
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "reversal", "side": side,
                                "platform_ft": platform_ft, "left_ft": left_ft, "right_ft": right_ft,
                                "xtk_m": xtk_now * M_PER_NM, "dist": raw.dist_nm,
                                "track": track_true, "missed": missed.is_some(),
                                "final_called": raw.phase == ApproachPhase::Final}),
                        );
                        announce(
                            format!(
                                "Repositioning for runway {}: {} downwind at {:.0} feet.",
                                crate::circuit::spell_runway(&rwy.id),
                                if side < 0.0 { "left-hand" } else { "right-hand" },
                                (platform_ft / 100.0).round() * 100.0
                            ),
                            Priority::Advisory,
                        );
                    }
                }
                // Fly it: the pattern owns the heading, the platform and the
                // speed until the aircraft is inbound and the ordinary
                // guidance calls a final it is actually pointing down.
                let mut pattern: Option<(f64, crate::reversal::Leg, f64)> = None;
                if let Some(rev) = reversal.as_mut() {
                    recede_ref_nm = recede_ref_nm.max(raw.dist_nm);
                    let join = join_point_nm(rev.platform_ft, rwy.elev_ft);
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
                // ClearedToLand throughout. Announced ONCE, on the arming tick.
                if missed.is_some() && !said_going_around {
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
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "going_around", "radio": radio_ft,
                                           "why": raw.balk, "xtk_m": xtk_now * M_PER_NM,
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
                    missed_since = None;
                } else if missed_since.is_none() {
                    missed_since = Some(std::time::Instant::now());
                }
                // A missed approach that never ends is its own emergency.
                if missed_since
                    .is_some_and(|t| t.elapsed().as_secs_f64() > MAX_MISSED_APPROACH_SECS)
                {
                    // A give-up must hand back a FLYABLE aircraft. The balked
                    // landing left gear down and landing flap out, and a flat
                    // 0.5 throttle cannot hold level flight dirty: the Baron
                    // decayed 110 to 80 kt at -900 fpm in the thirty seconds
                    // after the old handback (EGHI, 2026-08-02). Climb power,
                    // VS dial zeroed — the AP is already holding heading — and
                    // the line says what is actually held.
                    gave_up = true;
                    let thr_dref = throttle_dref();
                    let _ = tokio::join!(
                        xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                        xp.set_float(VS_DIAL, 0.0),
                    );
                    announce(
                        "I can't get us back round for another approach. Your aircraft — \
                         the autopilot is holding heading and altitude, gear is down."
                            .into(),
                        Priority::Warning,
                    );
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "missed_approach_timeout"}),
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
                let outbound = pattern
                    .is_some_and(|(_, leg, _)| leg != crate::reversal::Leg::Inbound);
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
                let throttle_name = throttle_dref();
                let (vvi_read, throttle_read) = tokio::join!(
                    xp.get_float(VVI_FPM), xp.get_float(&throttle_name),
                );
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
                // THE ARREST FLIES THE ELEVATOR, not the autopilot's VS dial.
                // Only while actually arresting, and only with both
                // measurements in hand: anywhere else the dial is right and
                // proven. The channel is a 1 s expiring heartbeat, so simply
                // ceasing to write it hands the pitch back by itself.
                let flare_elev = match (g.arresting, vvi, pitch_yielded) {
                    // Once the pilot has taken the pitch it stays taken for
                    // this approach — the debounce the taxi and takeoff
                    // channels learned: a latch that re-arms is a fight over
                    // a control with a person on the other end.
                    (true, Some(v), false) => {
                        Some(flare_pitch.step(g.target_vs_fpm, v, vvi_trend, tick_dt))
                    }
                    _ => {
                        flare_pitch.reset();
                        None
                    }
                };
                *TRACKING_ERROR.lock().unwrap_or_else(|e| e.into_inner()) = Some(xtk_now * M_PER_NM);
                crate::blackbox::record("autoland", serde_json::json!({
                    "rwy": format!("{} {}", rwy.icao, rwy.id), "dist": g.dist_nm,
                    "alt": alt, "gp_alt": g.glidepath_alt_ft, "cmd_vs": g.target_vs_fpm,
                    "vs_dial": vs_dial, "vvi": vvi, "vvi_trend": vvi_trend, "arresting": g.arresting,
                    "cmd_hdg": g.target_heading_deg, "ias": ias, "radio": radio_ft,
                    "phase": format!("{:?}", g.phase), "ga": ga, "crab": crab, "crab_raw": crab_raw,
                    "trim": trim, "xtk_m": xtk_now * M_PER_NM,
                    "flare_elev": flare_elev,
                }));

                // Lateral: heading bug in MAGNETIC (guidance is TRUE — see
                // [`true_to_mag`] for the variation sign trap). On final and in
                // the flare, add the wind crab so the commanded heading flies a
                // runway-aligned TRACK; intercept vectors stay coarse.
                let crabbed = match g.phase {
                    ApproachPhase::Final | ApproachPhase::Flare => {
                        (g.target_heading_deg + crab + trim).rem_euclid(360.0)
                    }
                    ApproachPhase::Intercept => g.target_heading_deg,
                };
                let mag_hdg = true_to_mag(crabbed, var);
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
                let (_, hdg_st, vs_st, _, yield_r) = tokio::join!(
                    crate::tools::autopilot::drive_heading_bug_step(&xp, mag_hdg, 30.0),
                    xp.get_float(HDG_STATUS),
                    xp.get_float(VS_STATUS),
                    xp.set_float(VS_DIAL, vs_dial),
                    async {
                        match flare_elev {
                            Some(e) => {
                                let _ = xp.set_float(FLARE_PITCH, e).await;
                                xp.get_float(FLARE_PITCH_YIELD).await
                            }
                            None => Ok(0.0),
                        }
                    },
                );
                // The pilot pulled or pushed: the plugin has already dropped
                // the override this frame. Stop commanding and let the dial
                // fly what is left — the VS path is still running underneath.
                if yield_r.is_ok_and(|v| v > 0.5) {
                    if !pitch_yielded {
                        pitch_yielded = true;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "pitch_yield", "radio": radio_ft}),
                        );
                        announce(
                            "You have the pitch. I'm still on the bug and the power.".into(),
                            Priority::Caution,
                        );
                    }
                    flare_pitch.reset();
                }
                if hdg_st.unwrap_or(0.0) <= 0.5 {
                    let _ = xp.run_command(CMD_HDG_MODE).await;
                }
                // Vertical: VS mode to the glidepath rate.
                if vs_st.unwrap_or(0.0) <= 0.5 {
                    let _ = xp.run_command(CMD_VS_MODE).await;
                    // The engage syncs the dial to the CURRENT rate, so the
                    // rate we want has to be written again after it.
                    let _ = xp.set_float(VS_DIAL, vs_dial).await;
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
                    Some(1.0)
                } else if g.phase == ApproachPhase::Flare {
                    match cfg.flare_power {
                        Some(power) => match (thr_now, vvi) {
                            (Some(throttle), Some(vs)) => flare_power.step(power, throttle, vs, dt_s),
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
                    let target = crate::aircraft::floored_target(g.target_ias_kts, floor);
                    let adj = crate::tools::speedhold::throttle_adjust(target - ias, trend_kts_s);
                    // In the roundout the lever only ever comes back. The
                    // speed law read the roundout's own deceleration as
                    // "five knots slow" and walked the 737's lever from 0.32
                    // to 0.55 at 30 ft over EGCC 23L; the flare then held
                    // that thrust (sink over `hold_sink_fpm`) and the
                    // aircraft climbed away from two feet at 155 kt
                    // (2026-09-19). The floor still stands underneath.
                    let adj = if g.arresting { adj.min(0.0) } else { adj };
                    let min_thr = approach_power_floor(
                        g.phase,
                        radio_ft,
                        target - ias,
                        crate::aircraft::approach_thrust_min(),
                    );
                    thr_now.map(|throttle| (throttle + adj).clamp(min_thr, 1.0))
                };
                let throttle_written = match thr {
                    Some(throttle) => Some(xp.set_float(&throttle_dref(), throttle).await.is_ok()),
                    None => None,
                };
                // A second round trip purely for the blackbox — whether the
                // lever TOOK the write, which is how an add-on re-driving it
                // gets caught. Worth it for the whole approach and not worth
                // it in the flare, where the tick budget is the landing: the
                // next tick's `throttle_before` says the same thing anyway,
                // one tick later.
                let throttle_after = if throttle_written == Some(true)
                    && g.phase != ApproachPhase::Flare
                {
                    xp.get_float(&throttle_dref())
                        .await
                        .ok()
                        .filter(|v| v.is_finite())
                } else {
                    None
                };
                if throttle_written == Some(false) {
                    flare_power.reset();
                }
                crate::blackbox::record("autoland", serde_json::json!({
                    "event": "power", "phase": format!("{:?}", g.phase),
                    "throttle_before": thr_now, "throttle_command": thr,
                    "throttle_after": throttle_after, "write_ok": throttle_written,
                    "progressive_flare": cfg.flare_power.is_some(), "dt": dt_s,
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
                    if let Some(line) = crate::takeoff_config::line("Landing config", &cfg) {
                        announce(line, Priority::Advisory);
                    }
                }
                // Go-around flap: half, whatever the schedule says — full flap
                // won't climb, and clean this low gives the sink away.
                // A downwind is flown at half flap too: the distance schedule
                // put full flap out abeam the numbers at 1,900 ft.
                let wanted = if ga || outbound {
                    0.5
                } else {
                    flap_schedule(g.dist_nm, alt - g.glidepath_alt_ft)
                };
                let flap = flap_held_within_limits(
                    wanted,
                    ias,
                    &crate::aircraft::flap_limits(),
                    last_flap,
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
                let not_held = g.phase != ApproachPhase::Flare
                    && vs_err.is_some_and(|e| vs_not_held(e, prev_vs_err));
                prev_vs_err = vs_err;
                if not_held {
                    vs_diverged += 1;
                    if vs_diverged == VS_DIVERGED_WARN_TICKS {
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
                    if vs_diverged >= VS_DIVERGED_GIVE_UP_TICKS {
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
                                               "ticks": vs_diverged, "vvi": vvi, "cmd_vs": g.target_vs_fpm}),
                        );
                        break;
                    }
                } else {
                    vs_diverged = 0;
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
                                "Going around. Full power, climbing straight ahead."
                            } else {
                                "Overflew the runway. Going around, climbing to re-intercept."
                            }
                        }
                        ApproachPhase::Intercept => "Intercepting the final approach.",
                        ApproachPhase::Final => "On final. Gear and flaps as the speed allows.",
                        ApproachPhase::Flare if cfg.flare_power.is_some() => "Over the threshold, flaring, easing the power back.",
                        ApproachPhase::Flare => "Over the threshold, flaring, power to idle.",
                    };
                    announce(line.into(), Priority::Caution);
                }

                // (Touchdown + rollout-to-stop is handled at the top of the loop,
                // latched by `rolling_out`, before the approach guidance.)
                tokio::time::sleep(std::time::Duration::from_millis(tick_ms(last_phase))).await;
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
pub fn clearance_switch(
    flying: &str,
    state: Option<crate::tower::State>,
    cleared: Option<&str>,
    already_checked: Option<&str>,
    going_around: bool,
) -> Option<String> {
    use crate::tower::State::{ClearedToLand, Inbound};
    if going_around || !matches!(state, Some(ClearedToLand | Inbound)) {
        return None;
    }
    let c = cleared?.trim();
    if !clearance_differs(flying, c) {
        return None;
    }
    if already_checked.is_some_and(|a| a.trim().eq_ignore_ascii_case(c)) {
        return None;
    }
    Some(c.to_string())
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
const TRACK_TRIM_ERR_CAP_DEG: f64 = 10.0;

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
}

impl TrackTrim {
    pub const fn new() -> Self {
        Self { deg: 0.0 }
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
        match phase {
            ApproachPhase::Final => {
                if let Some(h) = hpath_true {
                    let err = (wanted_track_deg - h + 540.0).rem_euclid(360.0) - 180.0;
                    if err.abs() <= TRACK_TRIM_ERR_CAP_DEG {
                        self.deg = (self.deg + err * dt_s / TRACK_TRIM_TAU_S)
                            .clamp(-TRACK_TRIM_MAX_DEG, TRACK_TRIM_MAX_DEG);
                    }
                }
            }
            ApproachPhase::Flare => {}
            ApproachPhase::Intercept => self.deg = 0.0,
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
const TOUCHDOWN_AIM_NM: f64 = 300.0 / M_PER_NM;
/// Final approach point: intercept the extended centreline this far out (nm).
const FAP_NM: f64 = 5.0;
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

/// The speed a give-up leaves the governor holding: the pattern speed, inside
/// the placard of whatever flap is actually out, never under the approach
/// speed. With the flaps up it is the clean approach speed.
pub fn hand_back_speed_kts(approach_ias: f64, flap_ratio: f64, flap_limits: &[(f64, Option<f64>)]) -> f64 {
    if flap_ratio < 0.05 {
        return clean_approach_ias(approach_ias);
    }
    let placard = flap_limits
        .iter()
        .filter(|(ratio, _)| *ratio <= flap_ratio + 1e-6)
        .filter_map(|(_, vfe)| *vfe)
        .fold(f64::INFINITY, f64::min);
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
    let run_to_band_m = (radio_alt_ft - band_ft).max(0.0) * M_PER_FT / GLIDEPATH_TAN;
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
/// The lever's idle stop in the speed law.
const IDLE_THROTTLE: f64 = 0.1;

/// The least the speed law may pull the lever to. On FINAL, not more than
/// `APPROACH_FAST_KTS` fast, it is the airframe's approach thrust floor
/// (jets fly the approach on thrust: the 737 idled its last mile and touched
/// down 17 kt under Vref, LFLL 2026-09-17). In a REPOSITION below the
/// terrain floor it is full power: excess speed there is spent on the
/// climb, never on the lever ("land the aircraft" from 140 ft at 237 kt
/// pulled the throttle to idle for its 179 kt target and the aircraft
/// settled into the fields with +800 fpm on the dial, same day). Everywhere
/// else, idle.
pub fn approach_power_floor(
    phase: ApproachPhase,
    radio_alt_ft: f64,
    err_kts: f64,
    approach_thrust_min: Option<f64>,
) -> f64 {
    match phase {
        ApproachPhase::Intercept if radio_alt_ft < TERRAIN_FLOOR_AGL_FT => 1.0,
        ApproachPhase::Final if err_kts > -APPROACH_FAST_KTS => approach_thrust_min
            .unwrap_or(IDLE_THROTTLE)
            .max(IDLE_THROTTLE),
        _ => IDLE_THROTTLE,
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
fn max_climb_fpm() -> f64 {
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
    let sink_fps = (gs_kts.max(0.0) / 60.0) * GLIDEPATH_FT_PER_NM / 60.0;
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

/// Per-approach state: a go-around resets this, so a second flare starts from
/// its own power. Never adds power or impersonates an airspeed governor in the
/// flare. The existing overflight/go-around guard still bounds runway usage.
#[derive(Default)]
struct FlarePower {
    previous_sink: Option<f64>,
    last_command: Option<f64>,
}

impl FlarePower {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn step(
        &mut self,
        cfg: crate::aircraft::FlarePowerProfile,
        measured: f64,
        vvi: f64,
        dt: f64,
    ) -> Option<f64> {
        if !measured.is_finite()
            || !vvi.is_finite()
            || !dt.is_finite()
            || dt <= 0.0
            || !cfg.reduction_per_second.is_finite()
            || cfg.reduction_per_second <= 0.0
            || !cfg.hold_sink_fpm.is_finite()
            || cfg.hold_sink_fpm <= 0.0
            || !cfg.lead_seconds.is_finite()
            || cfg.lead_seconds < 0.0
        {
            return None;
        }
        let sink = -vvi;
        let worsening = self
            .previous_sink
            .map_or(0.0, |old| ((sink - old) / dt).max(0.0));
        self.previous_sink = Some(sink);
        // First flare tick preserves the observed approach power. Readback may
        // lag a write, but it must never restart the ramp or increase power.
        let current = measured
            .clamp(0.0, 1.0)
            .min(self.last_command.unwrap_or(1.0));
        let next = if self.last_command.is_none()
            || sink.max(sink + worsening * cfg.lead_seconds) > cfg.hold_sink_fpm
        {
            current
        } else {
            (current - cfg.reduction_per_second * dt.min(1.0)).max(0.0)
        };
        self.last_command = Some(next);
        Some(next)
    }
}

/// What to dial into the VS mode so the aircraft actually FLIES `target_fpm`.
/// The plugin's direct elevator channel, and the pilot's take-back latch —
/// the SAME pair the takeoff rotates on. `override_joystick_pitch` bypasses
/// the autopilot's pitch servo while leaving it the roll, so the flare can be
/// flown on the elevator with the AP still holding the bug.
const FLARE_PITCH: &str = "xcopilot/pitch";
const FLARE_PITCH_YIELD: &str = "xcopilot/pitch_yield";

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
#[derive(Debug, Default, Clone, Copy)]
pub struct FlarePitch {
    trim: f64,
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
/// Total elevator the flare may command. Up is positive.
const FLARE_PITCH_MAX: f64 = 0.35;
/// ...and the push, which is deliberately far smaller: the ground is there.
const FLARE_PITCH_MIN: f64 = -0.10;

impl FlarePitch {
    /// `vvi_trend` is fpm per second, positive = sink easing (the same sign
    /// the VS lead uses). `None` for either measurement holds what we had.
    pub fn step(&mut self, target_fpm: f64, vvi_fpm: f64, vvi_trend: f64, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(0.02, 0.5);
        // Positive error = sinking FASTER than wanted = pull.
        let err = (target_fpm - vvi_fpm) / 100.0;
        self.trim = (self.trim + err * FLARE_TRIM_PER_100FPM_S * dt)
            .clamp(FLARE_PITCH_MIN, FLARE_PITCH_MAX);
        (err * FLARE_P_PER_100FPM - (vvi_trend / 100.0) * FLARE_D_PER_100FPM_S + self.trim)
            .clamp(FLARE_PITCH_MIN, FLARE_PITCH_MAX)
    }

    /// Let go — the channel expires on its own within a second, but the
    /// integrator must not carry a standing pull into the next approach.
    pub fn reset(&mut self) {
        self.trim = 0.0;
    }
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

/// Flap setting for the approach: half inside 6 nm, full inside 3 nm — but full
/// only once established ON the glidepath. While holding level BELOW the path
/// waiting to capture (`alt_err` well negative), full flaps are an energy trap:
/// the LERL 172 approach dragged along at 50–57 kt against a 65-kt target with
/// the throttle governor already at its ceiling, and arrived at the flare with
/// nothing left. Half flaps keep the drag survivable until the path is captured.
pub fn flap_schedule(dist_nm: f64, alt_err_ft: f64) -> f64 {
    if dist_nm < 3.0 && alt_err_ft > -150.0 {
        1.0
    } else if dist_nm < 6.0 {
        0.5
    } else {
        0.0
    }
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
/// ...and give up on a single missed approach that has run this long. The
/// counter above cannot catch a lost reposition because a second go-around
/// needs another approach to fail first. But the bound must be LONGER than
/// the flight the reposition itself commands: climb straight ahead to 800 ft,
/// turn back, overfly the field, and fly out to a join point ~6 nm on the
/// approach side — at a piston twin's 110 kt that is comfortably over five
/// minutes, and the old 300 s expired with the aircraft mid-leg at 3.8 nm,
/// compliant, pointing away from the field, which the give-up then handed to
/// the pilot as-is (EGHI, 2026-08-02). Ten minutes still bounds a genuinely
/// lost missed approach well inside fuel margins.
pub const MAX_MISSED_APPROACH_SECS: f64 = 600.0;

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
    ((cur_alt_ft - rwy_elev_ft) / GLIDEPATH_FT_PER_NM * 1.15).clamp(FAP_NM, 40.0)
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
) -> Guidance {
    let slow = approach_ias - ias_kts > SPEED_FLOOR_MARGIN_KTS;
    let (target_heading_deg, target_vs_fpm) = match stage {
        MissedStage::Climb => (rwy.course_deg, if slow { 0.0 } else { max_climb_fpm() }),
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
fn tick_ms(phase: Option<ApproachPhase>) -> u64 {
    match phase {
        // Over the threshold: the flare and the arrest.
        Some(ApproachPhase::Flare) => 100,
        // Established, configuring, closing the last miles.
        Some(ApproachPhase::Final) => 200,
        _ => 400,
    }
}

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
    let dist_nm = haversine_nm(cur_lat, cur_lon, rwy.thr_lat, rwy.thr_lon);
    let recip = (rwy.course_deg + 180.0) % 360.0;

    // 3° glidepath altitude at this distance (+ = we're above it). Aimed at the
    // touchdown zone, so it crosses the threshold at ~50 ft, not zero.
    let glidepath_alt_ft = rwy.elev_ft + (dist_nm + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
    let alt_err = cur_alt_ft - glidepath_alt_ft;

    // Above this much excess, we can't just dive onto the path — fly to the JOIN
    // point instead: where a 3° descent (with 15% margin) meets our CURRENT
    // altitude, out along the extended centreline. When we're inside it (armed
    // high overhead — the failure seen live: 7,000 ft at 6 nm, VS pegged, never
    // capturing) that means flying OUTBOUND first; the point walks back toward
    // the FAP as altitude burns off (a teardrop join).
    const CAPTURE_ABOVE_FT: f64 = 500.0;
    let join_nm = join_point_nm(cur_alt_ft, rwy.elev_ft);

    // Are we past the threshold, on the departure side, still airborne — by
    // more than the landing float is allowed? (The flare branch below, checked
    // first, owns a normal float/rollout across the numbers.)
    let along_nm = along_track_nm(cur_lat, cur_lon, rwy);
    // The arrest's gates and targets for THIS groundspeed (seconds of sink,
    // never bare feet: see `arrest_gates_ft`). `flare_radio_ft` is the floor.
    let nominal_vs = -(gs_kts / 60.0) * GLIDEPATH_FT_PER_NM;
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
        if xtk.abs() > FLARE_MAX_XTK_NM {
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
    } else if alt_err > CAPTURE_ABOVE_FT {
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
        if radio_alt_ft < 100.0 && dist_nm < 0.6 && xtk.abs() > FLARE_MAX_XTK_NM {
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
            && !stabilised_xtk(xtk, along_nm)
        {
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
    let descent_limit_fpm = if phase == ApproachPhase::Intercept {
        -((radio_alt_ft - REPOSITION_MIN_AGL_FT) * 2.0).clamp(0.0, 1200.0)
    } else {
        -1200.0
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
        ApproachPhase::Flare => flare_vs, // arrest the descent near the ground
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
        // dive.
        _ => (nominal_vs - alt_err * glidepath_gain_fpm_per_ft(gs_kts))
            .clamp(-1200.0, 0.0)
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
            let e = law.step(target, p.vvi, trend, dt);
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
        assert_eq!(tick_ms(None), 400, "nothing to fly yet: unchanged");
        assert_eq!(tick_ms(Some(ApproachPhase::Intercept)), 400, "miles out");
        assert!(tick_ms(Some(ApproachPhase::Final)) <= 200);
        let flare = tick_ms(Some(ApproachPhase::Flare));
        assert!(flare <= 100, "the flare needs feet, not seconds: {flare}");
        // The 737's arrest lead is 1.5 s of dead time; a three second flare
        // must carry enough commands to matter through it.
        let commands_in_flare = 3000 / flare;
        assert!(
            commands_in_flare >= 20,
            "only {commands_in_flare} commands in a flare"
        );
        // ...and it must never get SLOWER as the aircraft gets closer.
        assert!(
            tick_ms(Some(ApproachPhase::Flare)) <= tick_ms(Some(ApproachPhase::Final))
                && tick_ms(Some(ApproachPhase::Final)) <= tick_ms(Some(ApproachPhase::Intercept)),
            "the tick rate must rise monotonically toward the ground"
        );
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
    fn baron_recorded_flare_does_not_cut_power_into_renewed_sink() {
        // LEBL 20, 2026-09-16. Replay measurements, NOT a flight-model prediction:
        // the old rule commanded zero throughout these actual sink rates.
        let mut power = FlarePower::default();
        let cfg = baron_flare_power();
        let samples = [
            -393.65, -278.33, -148.51, -49.71, -24.33, -84.22, -211.0, -360.38, -515.28,
        ];
        let mut throttle = 0.30;
        let mut commands = Vec::new();
        for vs in samples {
            throttle = power.step(cfg, throttle, vs, 0.67).unwrap();
            commands.push(throttle);
        }
        assert_eq!(commands[0], 0.30, "no step to idle at flare entry");
        assert_eq!(commands[1], commands[0], "hold while still sinking fast");
        assert!(
            commands[4] < commands[1],
            "taper after the pitch arrests descent"
        );
        assert!(
            commands[5] < commands[4],
            "small initial trend remains below limit"
        );
        assert_eq!(commands[6], commands[5], "stop the cut when sink returns");
        assert_eq!(commands[8], commands[6]);
        assert!(
            commands[8] > 0.0,
            "retain residual power rather than force idle"
        );
    }

    #[test]
    fn flare_power_foresees_sink_and_resets_for_a_new_approach() {
        let cfg = baron_flare_power();
        let mut power = FlarePower::default();
        assert_eq!(power.step(cfg, 0.3, -50.0, 0.5), Some(0.3));
        // Still less than 200 fpm down, but deteriorating at 200 fpm per second.
        assert_eq!(power.step(cfg, 0.3, -150.0, 0.5), Some(0.3));
        // A balloon needs power coming off, not a sink-triggered hold.
        assert!(power.step(cfg, 0.3, 100.0, 0.5).unwrap() < 0.3);
        // A lagging throttle read must not restart/increase the command.
        assert!(power.step(cfg, 0.4, -100.0, 0.5).unwrap() < 0.3);
        power.reset();
        assert_eq!(power.step(cfg, 0.45, -300.0, 0.5), Some(0.45));
    }

    #[test]
    fn flare_power_reaches_idle_smoothly_and_rejects_missing_measurements() {
        let cfg = baron_flare_power();
        let mut power = FlarePower::default();
        let mut throttle = power.step(cfg, 0.3, -100.0, 0.5).unwrap();
        for _ in 0..30 {
            let next = power.step(cfg, throttle, -100.0, 0.5).unwrap();
            assert!((0.0..=throttle).contains(&next));
            assert!(throttle - next <= cfg.reduction_per_second * 0.5 + 1e-9);
            throttle = next;
        }
        assert_eq!(throttle, 0.0);
        assert_eq!(power.step(cfg, f64::NAN, -100.0, 0.5), None);
        assert_eq!(power.step(cfg, 0.3, f64::NAN, 0.5), None);
        assert_eq!(power.step(cfg, 0.3, -100.0, 0.0), None);
        let invalid = crate::aircraft::FlarePowerProfile {
            reduction_per_second: -1.0,
            ..cfg
        };
        assert_eq!(power.step(invalid, 0.3, -100.0, 0.5), None);
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
        // A 2° error integrates at 2/30 per second.
        let v = t.update(ApproachPhase::Final, 90.0, Some(88.0), 3.0);
        assert!((v - 0.2).abs() < 1e-9, "{v}");
        // Held through the flare, whatever the track does.
        assert_eq!(t.update(ApproachPhase::Flare, 90.0, Some(80.0), 5.0), v);
        // A missing track read holds the value.
        assert_eq!(t.update(ApproachPhase::Final, 90.0, None, 5.0), v);
        // Wraps: 359 wanted, 1 flown is -2, not +358.
        let mut w = super::TrackTrim::new();
        assert!(w.update(ApproachPhase::Final, 359.0, Some(1.0), 3.0) < 0.0);
        // Capped, and reset by the intercept (a go-around starts clean).
        for _ in 0..10_000 {
            t.update(ApproachPhase::Final, 90.0, Some(85.0), 1.0);
        }
        assert_eq!(t.value(), super::TRACK_TRIM_MAX_DEG);
        assert_eq!(
            t.update(ApproachPhase::Intercept, 90.0, Some(85.0), 1.0),
            0.0
        );
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
        let sw = |f, st, c, a| super::clearance_switch(f, st, c, a, false);
        assert_eq!(
            sw("27", Some(ClearedToLand), Some("09"), None),
            Some("09".into())
        );
        // EGTE 2026-09-21: the go-around's own state change published the
        // flipped runway in use. A missed approach is never re-aimed.
        assert_eq!(
            super::clearance_switch("08", Some(Inbound), Some("26"), None, true),
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
        assert!(!should_start_rollout(false, false, 3.0, 70.0, 89.0, 90.0));
        assert!(!should_start_rollout(false, false, 3.0, 40.0, 89.0, 90.0));
        // The same committed state starts rollout once the wheels are down,
        // including a tailwind landing whose groundspeed remains high.
        assert!(should_start_rollout(false, true, 3.0, 70.0, 89.0, 90.0));
        assert!(should_start_rollout(false, true, 3.0, 40.0, 95.0, 90.0));
        // Once latched, a noisy WOW sample cannot return control to approach.
        assert!(should_start_rollout(true, false, 10.0, 70.0, 95.0, 90.0));
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

    /// The 737's vertical-speed mode, fitted to the EGCC 23L tick log
    /// (2026-09-19): two first-order lags of 1.5 s in series and two
    /// seconds of dead time (three ticks). Flown from 100 ft over the
    /// numbers at the path sink, gs 132, with the flare's thrust reduction
    /// as the same sink disturbance the 172 plant carries. Returns the sink
    /// at the wheels, the shallowest rate seen (a balloon is positive), the
    /// metres of runway used and whether the flare gate was reached.
    fn fly_jet_arrest(lead: crate::aircraft::ArrestLeadProfile) -> (f64, f64, f64, bool) {
        let mut rwy = rwy_north();
        rwy.length_m = 3040.0;
        let dt = 0.7;
        let (t1, t2) = (1.5, 1.5);
        let dead_ticks = 3;
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

    /// The live failure and its fix, through the fitted plant: the 172's
    /// lead (gain 2, cap +300) pins the dial at a climb for seconds and the
    /// jet balloons and floats; its own lead (gain 1.5, cap +100, a 1.5 s
    /// trend term) arrests it without a balloon, inside the touchdown zone.
    #[test]
    fn the_jets_arrest_neither_balloons_nor_floats() {
        let (td, shallowest, along_m, flared) =
            fly_jet_arrest(crate::aircraft::ArrestLeadProfile::default());
        assert!(flared);
        assert!(
            shallowest > 0.0 || along_m > 1000.0,
            "the 172's lead should reproduce the balloon or the float: {td:.0} fpm, {shallowest:.0} shallowest, {along_m:.0} m"
        );

        let jet = crate::aircraft::ArrestLeadProfile {
            gain: 1.5,
            max_climb_fpm: 100.0,
            trend_seconds: 1.5,
        };
        let (td, shallowest, along_m, flared) = fly_jet_arrest(jet);
        assert!(flared);
        assert!(shallowest < 0.0, "ballooned to {shallowest:.0} fpm");
        assert!(td > -300.0, "still firm: {td:.0} fpm at the wheels");
        assert!(td < -60.0, "the arrest must still land, got {td:.0}");
        assert!(along_m < 1000.0, "floated {along_m:.0} m");
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
            guidance(lat, lon, radio, radio, gs, gs, &rwy_north(), 150.0, 25.0, 0.0)
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

    /// EGTE 2026-09-21: handed back at 152 kt with half flap and a flat 0.75
    /// lever; 346 kt followed.
    #[test]
    fn a_give_up_hands_back_a_speed_the_configuration_allows() {
        let limits = [(0.0, None), (0.5, Some(210.0)), (0.875, Some(175.0)), (1.0, Some(162.0))];
        let half = hand_back_speed_kts(148.0, 0.5, &limits);
        assert!((160.0..=195.0).contains(&half), "{half}");
        // Full flap still out: inside ITS placard, never under the approach speed.
        let full = hand_back_speed_kts(148.0, 1.0, &limits);
        assert!((148.0..=150.0).contains(&full), "{full}");
        // Clean: the clean approach speed.
        assert_eq!(hand_back_speed_kts(148.0, 0.0, &limits), clean_approach_ias(148.0));
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
        assert_eq!(stage, Some(MissedStage::Reposition), "held abeam the join point");
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
        assert!(into_wind_end < downwind_end, "{into_wind_end} vs {downwind_end}");
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

    #[test]
    fn the_vs_watchdog_excuses_a_slow_autopilot_and_names_what_is_commanded() {
        // LEMI: cmd 0, diving 2,300 and not recovering. Counts, every tick.
        assert!(vs_not_held(-2300.0, None));
        assert!(vs_not_held(-2300.0, Some(-2300.0)));
        assert!(vs_not_held(-2400.0, Some(-2300.0)));
        // The A330 at the arm: a -1,200 step from level, closing at 40 fpm a
        // tick. Far from the command, and being flown toward it.
        assert!(!vs_not_held(1100.0, Some(1140.0)));
        // Closing too slowly to be the autopilot's doing.
        assert!(vs_not_held(1100.0, Some(1105.0)));
        // Inside the band is held, whatever the trend.
        assert!(!vs_not_held(650.0, Some(100.0)));
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
        assert_eq!(approach_power_floor(Final, 300.0, 0.0, Some(0.3)), 0.3);
        assert_eq!(approach_power_floor(Final, 300.0, -6.0, Some(0.3)), 0.3);
        assert_eq!(approach_power_floor(Final, 300.0, -20.0, Some(0.3)), 0.1);
        assert_eq!(approach_power_floor(Final, 300.0, 0.0, None), 0.1);
        assert_eq!(
            approach_power_floor(Intercept, 140.0, -58.0, Some(0.3)),
            1.0
        );
        assert_eq!(
            approach_power_floor(Intercept, 1500.0, -58.0, Some(0.3)),
            0.1
        );
        assert_eq!(approach_power_floor(Flare, 20.0, 0.0, Some(0.3)), 0.1);
    }

    /// EGCC 23L, 2026-09-21: the servos stayed on through the rollout and the
    /// taxi, and the heading servo wound the yoke over on the ground.
    #[tokio::test]
    async fn the_autopilot_comes_off_at_the_wheels() {
        use crate::xplane::FakeXPlane;
        let xp = FakeXPlane::with_reads(&[(super::SERVOS_ON, 1.0)]);
        // The fake never moves, so the readback disagrees and it is SAID so.
        assert_eq!(super::disconnect_autopilot(&xp).await, Some(false));
        assert!(xp.ran(super::CMD_SERVOS_OFF));
        // Already off (a hand-flown landing under autoland's callouts): left.
        let xp = FakeXPlane::with_reads(&[(super::SERVOS_ON, 0.0)]);
        assert_eq!(super::disconnect_autopilot(&xp).await, None);
        assert!(!xp.ran(super::CMD_SERVOS_OFF));
    }
}
