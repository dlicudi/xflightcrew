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

const M_PER_NM: f64 = 1852.0;
/// apt.dat surface codes that are hard (asphalt=1, concrete=2) — preferred.
const HARD_SURFACES: [i32; 2] = [1, 2];

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
        HARD_SURFACES.contains(&self.surface)
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
    let mut cands = nearby_runways(apt_dat, lat, lon, max_nm)?;
    // Suitability: hard surface and long enough.
    cands.retain(|r| r.hard() && r.length_m >= min_length_m);
    // Score: distance + a turn penalty (bearing TO the threshold vs current track).
    // Lower is better — favours a runway ahead and close.
    let best = cands.into_iter().min_by(|a, b| {
        let score = |r: &Runway| {
            let brg = bearing_deg(lat, lon, r.thr_lat, r.thr_lon);
            r.dist_nm + heading_diff(track_deg, brg) / 30.0
        };
        score(a)
            .partial_cmp(&score(b))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    Ok(best)
}

/// The specific hard runway END matching `id` at the nearest field within
/// `max_nm` — used when the arrival has ALREADY chosen the runway (by wind) and
/// autoland must honour that end instead of re-picking by track. `None` if no
/// such end is in range (the caller falls back to [`best_landing_runway`]).
pub fn runway_by_id(
    apt_dat: &str,
    lat: f64,
    lon: f64,
    id: &str,
    max_nm: f64,
) -> std::io::Result<Option<Runway>> {
    let want = id.trim();
    // nearby_runways is already nearest-threshold-first.
    let cands = nearby_runways(apt_dat, lat, lon, max_nm)?;
    Ok(cands.into_iter().find(|r| r.hard() && r.id.trim() == want))
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
const VVI_FPM: &str = "sim/cockpit2/gauges/indicators/vvi_dial_fpm";
// AP + controls (mirror of autopilot.rs / recover.rs).
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
const HDG_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
const HDG_STATUS: &str = "sim/cockpit2/autopilot/heading_status";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
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
/// Brake application on the rollout, and the gentler hold once stopped.
const ROLLOUT_BRAKE: f64 = 0.45;
const ROLLOUT_BRAKE_HOLD: f64 = 0.5;
const M_TO_FT: f64 = 3.280_84;
const MS_TO_KTS: f64 = 1.943_84;

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
    // The chat mirror now rides `voice.alert`; an explicit `s.action` here too
    // double-wrote every autoland line to the transcript (2026-07-24).
    _status: Option<crate::plugin_status::StatusSink>,
    // The arrival's already-chosen landing runway (by wind). `Some` forces that
    // end; `None` self-picks by track — the emergency/manual path, unchanged.
    forced_rwy: Option<String>,
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
            let xp = XPlaneClient::new(base);
            ACTIVE.store(true, Ordering::SeqCst);
            let announce = |line: String, sev: Priority| {
                tracing::info!(target: "xvoicelayer::autoland", spoke = %line, "copilot");
                if let Some(v) = &voice { v.alert(sev, line); }
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
            // rather than bailing on the landing.
            let picked = match forced_rwy.as_deref() {
                Some(id) => match runway_by_id(&apt_dat, lat0, lon0, id, 40.0) {
                    Ok(None) => best_landing_runway(&apt_dat, lat0, lon0, hdg0, 40.0, 1200.0),
                    found => found,
                },
                None => best_landing_runway(&apt_dat, lat0, lon0, hdg0, 40.0, 1200.0),
            };
            let rwy = match picked {
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

            // Engage the AP (FD up first for airframes that need it).
            for c in &cfg.fd_commands { let _ = xp.run_command(c).await; }
            let _ = xp.run_command(CMD_SERVOS_ON).await;

            let mut last_phase: Option<ApproachPhase> = None;
            let mut go_around = GoAroundLatch::new();
            // The "going around" transmission: said once per balked landing,
            // with the callsign read from the aircraft the first time it's
            // needed (an HTTP read, so not on every tick).
            let mut said_going_around = false;
            let mut going_around_callsign: Option<Option<String>> = None;
            // When the current missed approach started, for the runaway guard.
            let mut missed_since: Option<std::time::Instant> = None;
            let mut gear_down = false;
            let mut last_flap: Option<f64> = None;
            let mut vs_diverged: u32 = 0;
            // Airspeed history for the throttle governor's derivative term — the
            // anti-stall reflex needs the TREND (kts/sec), not just the error.
            let mut prev_ias: Option<f64> = None;
            let mut prev_ias_t = std::time::Instant::now();
            // Cross-track history for the intercept damping term.
            let mut prev_xtk: Option<(f64, std::time::Instant)> = None;
            let mut xtk_rate: f64 = 0.0;
            // Rollout: latched at touchdown, then brakes to a full stop. The
            // takeoff roll law (P + trim, tested) holds the centreline through
            // the plugin steer channel while it brakes.
            let mut rolling_out = false;
            let mut roll_law = crate::autotakeoff::RollSteer::default();
            loop {
                if GEN.load(Ordering::SeqCst) != my_gen || ABORT.load(Ordering::SeqCst) {
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
                    rolling_out = true;
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
                    if gs_kts > 2.0 {
                        let _ = xp.set_float(TAXI_BRAKE, ROLLOUT_BRAKE).await;
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
                // Wind crab, from the reference recipe (heading vs ground track):
                // command a heading whose TRACK aligns with the runway, not the
                // heading itself. P-only tracking left a standing 0.10 nm
                // cross-track under the LXGB crosswind — touchdown in the grass
                // (2026-07-22). Fail to zero when either read is missing.
                let crab = match (&psi_r, &hpath_r) {
                    (Ok(psi), Ok(hpath)) => {
                        (((psi - hpath + 540.0).rem_euclid(360.0)) - 180.0).clamp(-15.0, 15.0)
                    }
                    _ => 0.0,
                };

                // Cross-track closure rate (nm/min, positive = drifting right),
                // low-pass filtered — the damping term that turns the P-only
                // centreline chase into a crisp intercept-and-rollout. P alone
                // converged asymptotically (the banana join) and S-ed under
                // wind (2026-07-24).
                let xtk_now = cross_track_nm(lat, lon, &rwy);
                let t_now = std::time::Instant::now();
                let raw_rate = prev_xtk.map_or(0.0, |(x0, t0)| {
                    let dt = t_now.duration_since(t0).as_secs_f64();
                    if (0.1..=3.0).contains(&dt) { (xtk_now - x0) / dt * 60.0 } else { 0.0 }
                });
                prev_xtk = Some((xtk_now, t_now));
                xtk_rate = xtk_rate * 0.5 + raw_rate.clamp(-3.0, 3.0) * 0.5;

                let raw = guidance(lat, lon, alt, radio_ft, gs_kts, ias, &rwy, approach_ias, 25.0, xtk_rate);
                // A missed approach called low is flown by the latch (full power,
                // straight ahead) until clear of the ground — the raw intercept
                // guidance turns back to the FAP immediately, which is a killer
                // at 50 ft (see [`GoAroundLatch`]).
                let along_nm = along_track_nm(lat, lon, &rwy);
                let missed = go_around.update(
                    last_phase,
                    raw.phase,
                    radio_ft,
                    raw.dist_nm,
                    along_nm,
                    join_point_nm(alt, rwy.elev_ft),
                );
                let ga = missed.is_some();
                // The tower is owed this the moment it happens: it is holding a
                // landing clearance for an aircraft that is no longer landing,
                // and it works the traffic underneath us. Luton 2026-07-28: the
                // autoland balked, climbed away and repositioned over the field
                // for four minutes, and nothing was ever said — the FSM sat in
                // ClearedToLand throughout. Announced ONCE, on the arming tick.
                if missed == Some(MissedStage::Climb) && !said_going_around {
                    said_going_around = true;
                    if going_around_callsign.is_none() {
                        going_around_callsign = Some(crate::circuit::crew_callsign(&xp).await);
                    }
                    let cs = going_around_callsign.clone().flatten();
                    crate::circuit::transmit(crate::circuit::radio_call(&cs, "going around"));
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "going_around", "radio": radio_ft}),
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
                    let _ = xp.set_float(&throttle_dref(), 0.5).await;
                    announce(
                        "I can't get us back round for another approach. Your aircraft.".into(),
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
                if go_around.attempts() > MAX_GO_AROUNDS {
                    let _ = xp.set_float(&throttle_dref(), 0.5).await;
                    announce(
                        "Unable to land — we've gone around too many times. Your aircraft."
                            .into(),
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
                crate::blackbox::record("autoland", serde_json::json!({
                    "rwy": format!("{} {}", rwy.icao, rwy.id), "dist": g.dist_nm,
                    "alt": alt, "gp_alt": g.glidepath_alt_ft, "cmd_vs": g.target_vs_fpm,
                    "cmd_hdg": g.target_heading_deg, "ias": ias, "radio": radio_ft,
                    "phase": format!("{:?}", g.phase), "ga": ga, "crab": crab,
                }));

                // Lateral: heading bug in MAGNETIC (guidance is TRUE — see
                // [`true_to_mag`] for the variation sign trap). On final and in
                // the flare, add the wind crab so the commanded heading flies a
                // runway-aligned TRACK; intercept vectors stay coarse.
                let crabbed = match g.phase {
                    ApproachPhase::Final | ApproachPhase::Flare => {
                        (g.target_heading_deg + crab).rem_euclid(360.0)
                    }
                    ApproachPhase::Intercept => g.target_heading_deg,
                };
                let mag_hdg = true_to_mag(crabbed, var);
                // Knob presses, not a raw write — registers on glass (G1000) too.
                // Slew-capped: each press is an HTTP round trip, and the LXGB go-
                // around's 178° flip stalled this loop ~5 s per tick (2026-07-14) —
                // no VS/throttle updates the whole time. 30°/tick keeps a full
                // reversal turning while the loop stays live.
                let _ = crate::tools::autopilot::drive_heading_bug_step(&xp, mag_hdg, 30.0).await;
                if xp.get_float(HDG_STATUS).await.unwrap_or(0.0) <= 0.5 {
                    let _ = xp.run_command(CMD_HDG_MODE).await;
                }
                // Vertical: VS mode to the glidepath rate.
                if xp.get_float(VS_STATUS).await.unwrap_or(0.0) <= 0.5 {
                    let _ = xp.run_command(CMD_VS_MODE).await;
                }
                let _ = xp.set_float(VS_DIAL, g.target_vs_fpm).await;

                // Speed: hold the approach speed with the SAME governor the copilot
                // uses for speed hold (speedhold::throttle_adjust) — a P+D law whose
                // derivative term adds power while the airspeed is still UNWINDING,
                // before the error grows, with extra authority on the slow side. The
                // old loop here was P-only (0.004/kt, no trend), so it only reacted
                // after speed was already gone and dribbled power in too slowly —
                // the nose came up holding level, speed bled to a stall, the AP quit
                // ("lose speed, lose AP", seen live at LERL/XLE000N). Idle in flare.
                let now = std::time::Instant::now();
                let dt_s = (now - prev_ias_t).as_secs_f64().clamp(0.1, 2.0);
                let trend_kts_s = prev_ias.map_or(0.0, |p| (ias - p) / dt_s);
                prev_ias = Some(ias);
                prev_ias_t = now;
                let thr_now = xp.get_float(&throttle_dref()).await.unwrap_or(0.5);
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
                    1.0
                } else if g.phase == ApproachPhase::Flare {
                    0.0
                } else {
                    let adj = crate::tools::speedhold::throttle_adjust(g.target_ias_kts - ias, trend_kts_s);
                    (thr_now + adj).clamp(0.1, 1.0)
                };
                let _ = xp.set_float(&throttle_dref(), thr).await;

                // Config: gear down inside 6 nm; flaps by distance AND vertical
                // state (full only once on the path — see [`flap_schedule`]).
                // Written only when the target changes, so the setting isn't
                // re-asserted against the pilot every tick.
                if !gear_down && g.dist_nm < 6.0 {
                    let _ = xp.set_float(GEAR_HANDLE, 1.0).await;
                    gear_down = true;
                }
                // Go-around flap: half, whatever the schedule says — full flap
                // won't climb, and clean this low gives the sink away.
                let flap = if ga { 0.5 } else { flap_schedule(g.dist_nm, alt - g.glidepath_alt_ft) };
                if last_flap != Some(flap) {
                    last_flap = Some(flap);
                    let _ = xp.set_float(FLAP_RATIO, flap).await;
                }

                // Watchdog: the AP not FLYING the commanded vertical speed is how
                // the LEMI approach dove 2,300 fpm into terrain with vs_cmd=0 and
                // no callout. If actual VS diverges hard for ~5 s, re-arm the VS
                // mode and tell the pilot — this is exactly when a hand on the
                // yoke matters.
                let vvi = xp.get_float(VVI_FPM).await.unwrap_or(g.target_vs_fpm);
                if g.phase != ApproachPhase::Flare && (vvi - g.target_vs_fpm).abs() > 700.0 {
                    vs_diverged += 1;
                    if vs_diverged == 12 {
                        let _ = xp.run_command(CMD_VS_MODE).await;
                        announce(
                            "Autopilot is not holding the descent. Watch the pitch — be ready to take it.".into(),
                            Priority::Warning,
                        );
                    }
                } else {
                    vs_diverged = 0;
                }

                if last_phase != Some(g.phase) {
                    let prev = last_phase;
                    last_phase = Some(g.phase);
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
                        ApproachPhase::Final => "Established on final, gear and flaps down.",
                        ApproachPhase::Flare => "Over the threshold, flaring, power to idle.",
                    };
                    announce(line.into(), Priority::Caution);
                }

                // (Touchdown + rollout-to-stop is handled at the top of the loop,
                // latched by `rolling_out`, before the approach guidance.)
                tokio::time::sleep(std::time::Duration::from_millis(400)).await;
            }
            set_panel("");
            ACTIVE.store(false, Ordering::SeqCst);
        });
    });
}

/// Feet of altitude per nm along a 3° glidepath (tan 3° × 6076 ft/nm ≈ 318).
/// Public because the circuit descends its base leg onto the same path — a
/// pattern that hands over 500 ft below it has already lost the approach.
pub const GLIDEPATH_FT_PER_NM: f64 = 318.0;
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
/// Airspeed this far below the approach speed triggers speed protection (nose
/// down to regain it), before the decay reaches a stall and disconnects the AP.
/// Approach speed is already ~1.3×Vs, so this margin still stays well above stall.
const SPEED_FLOOR_MARGIN_KTS: f64 = 7.0;

/// Past the threshold by more than this (nm along the centreline, departure side)
/// with no landing = an overflight. Go around: fly back to the FAP instead of
/// tracking the runway heading straight out. Seen live 2026-07-06: overflew the
/// numbers at 167 ft, then departed the far end on a fixed course to 5.4 nm at
/// 60 kt, never turning back. The margin ignores a normal float/rollout past the
/// threshold (the flare branch, checked first, still owns the touchdown).
const OVERFLIGHT_GO_AROUND_NM: f64 = 0.15;

/// A latched go-around climbs straight ahead until this height before the
/// intercept guidance may turn it back toward the FAP (see GoAroundLatch).
const GO_AROUND_TURN_AGL_FT: f64 = 800.0;
/// Terrain floor (ft AGL): below this and outside the flare zone, low radio
/// altitude is rising terrain under the flight path, not the runway — climb away
/// from it rather than hold level into it. Seen live 2026-07-06: held level at
/// ~50 ft AGL 5 nm out over rising ground, no climb authority anywhere in the loop.
const TERRAIN_FLOOR_AGL_FT: f64 = 500.0;
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
/// While below the terrain floor, command the throttle governor to this much over
/// the approach speed, so it builds the surplus the climb gate needs.
const CLIMB_ENERGY_BONUS_KTS: f64 = 12.0;

/// Flare only inside this distance to the threshold — beyond it, a low radio
/// altitude is terrain under the approach, not the runway. (25 ft on a 3° path
/// is ~0.08 nm out; 0.35 leaves margin for displaced thresholds.)
const FLARE_MAX_DIST_NM: f64 = 0.35;
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
}

/// Give up (level off, hand to the pilot) after this many go-arounds on one
/// approach rather than loop a missed approach until the fuel runs out.
pub const MAX_GO_AROUNDS: u32 = 2;
/// ...and give up on a single missed approach that has run this long. A
/// reposition leg is a couple of minutes of flying; one that is still going
/// after this is not repositioning, it is lost, and the counter above cannot
/// catch it because a second go-around needs another approach to fail first.
/// Costs nothing when everything works and bounds the failure when it doesn't.
pub const MAX_MISSED_APPROACH_SECS: f64 = 300.0;

impl GoAroundLatch {
    pub fn new() -> Self {
        Self::default()
    }

    /// Go-arounds flown so far this approach.
    pub fn attempts(&self) -> u32 {
        self.attempts
    }

    /// Advance the latch one tick: `prev`/`now` are the raw guidance phases,
    /// `radio_ft` the radio altitude, `dist_nm` the distance to the threshold,
    /// `along_nm` the signed along-track (positive = approach side) and
    /// `join_nm` the [`join_point_nm`] the reposition leg is flying out to.
    /// Returns the stage being flown, `None` when the approach is the pilot's
    /// normal one again.
    pub fn update(
        &mut self,
        prev: Option<ApproachPhase>,
        now: ApproachPhase,
        radio_ft: f64,
        dist_nm: f64,
        along_nm: f64,
        join_nm: f64,
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
            Some(MissedStage::Reposition) if along_nm > 0.0 && dist_nm >= join_nm - 0.2 => {
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
        MissedStage::Climb => (rwy.course_deg, if slow { 0.0 } else { MAX_CLIMB_FPM }),
        MissedStage::Reposition => {
            // Same join point the ordinary guidance uses, so the handback is
            // continuous.
            let join_nm = join_point_nm(cur_alt_ft, rwy.elev_ft);
            let recip = (rwy.course_deg + 180.0) % 360.0;
            let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
            let vs = if radio_alt_ft < REPOSITION_MIN_AGL_FT - 50.0 && !slow {
                ((REPOSITION_MIN_AGL_FT - radio_alt_ft) * 2.0).clamp(100.0, MAX_CLIMB_FPM)
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

    // Are we past the threshold, on the departure side, still airborne? (The flare
    // branch below, checked first, owns a normal float/rollout across the numbers.)
    let past_threshold = along_track_nm(cur_lat, cur_lon, rwy) < -OVERFLIGHT_GO_AROUND_NM;

    let xtk = cross_track_nm(cur_lat, cur_lon, rwy);
    let (phase, target_heading) = if radio_alt_ft <= flare_radio_ft && dist_nm <= FLARE_MAX_DIST_NM
    {
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
            let lead_nm = (gs_kts / 120.0).clamp(0.6, 1.5);
            let along = along_track_nm(cur_lat, cur_lon, rwy);
            // The aim must sit AHEAD of the aircraft along the approach: an
            // aim past our own along-track position flips the bearing and the
            // cut with it (the ±35° S over the numbers, 2026-07-24). Abeam
            // close-in (tiny `along`, big offset) halve what's left instead —
            // the balk gate owns that geometry anyway.
            let mut aim_nm = (along - lead_nm).max(0.15);
            if aim_nm >= along {
                aim_nm = (along * 0.5).max(0.02);
            }
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
            (ApproachPhase::Intercept, rwy.course_deg)
        } else {
            (
                ApproachPhase::Final,
                (rwy.course_deg + cut).rem_euclid(360.0),
            )
        }
    };

    // Track the path with a nominal descent for the current groundspeed plus an
    // error correction; bounded so it never dives.
    let nominal_vs = -(gs_kts / 60.0) * GLIDEPATH_FT_PER_NM;
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

    // What the vertical path asks for, computed first so the speed-protection
    // branch below can be measured against it.
    let path_vs = match phase {
        ApproachPhase::Flare => -100.0, // arrest the descent near the ground
        // Below the floor without the surplus to climb: hold level while the
        // throttle bonus rebuilds it — NEVER track the glidepath down. The
        // glidepath altitude is distance-symmetric, so a rejoin on the wrong
        // side of the threshold reads "on path" and descends (KPDK 16).
        _ if below_terrain_floor => 0.0,
        // On/above speed but below the glidepath: hold level and let the path
        // descend to us — don't climb to intercept. Speed protection above has
        // already handled the case where holding level would stall us.
        _ if alt_err < -50.0 => 0.0,
        // 1.2 fpm per ft of path error: the old 0.4 recovered 77 fpm
        // against the 190 ft the LXGB 09 final carried — it arrived at
        // the numbers still 190 ft high (2026-07-22). 1.2 converges a
        // 200 ft error inside the last mile; the clamp still forbids a
        // dive.
        _ => (nominal_vs - alt_err * 1.2)
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
        ((floor_ft - radio_alt_ft) * 2.0).clamp(100.0, MAX_CLIMB_FPM)
    } else {
        path_vs
    };

    // Below the terrain floor, ask the throttle governor for extra speed so it
    // builds the surplus the climb gate needs; otherwise hold the approach speed.
    let target_ias = if below_terrain_floor {
        approach_ias + CLIMB_ENERGY_BONUS_KTS
    } else {
        approach_ias
    };

    Guidance {
        target_heading_deg: target_heading,
        target_vs_fpm: target_vs,
        target_ias_kts: target_ias,
        dist_nm,
        glidepath_alt_ft,
        phase,
    }
}

#[cfg(test)]
mod tests {
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
        let up = |l: &mut GoAroundLatch, p, n, r| l.update(Some(p), n, r, 0.2, -0.2, FAP_NM);
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
            l.update(Some(Final), Intercept, 50.0, 0.2, -0.2, join),
            Some(Climb)
        );
        assert_eq!(
            l.update(Some(Intercept), Intercept, 900.0, 1.0, -1.0, join),
            Some(Reposition)
        );
        // Departure side, miles out: still repositioning, however far.
        assert_eq!(
            l.update(Some(Intercept), Intercept, 1600.0, 6.0, -6.0, join),
            Some(Reposition)
        );
        // Approach side but short of the join point: an approach can't start
        // here, and handing back mid-outbound-leg means a 180 on final.
        assert_eq!(
            l.update(Some(Intercept), Intercept, 1600.0, 5.2, 5.2, join),
            Some(Reposition)
        );
        // Out at the join point on the approach side: hand back.
        assert_eq!(
            l.update(Some(Intercept), Intercept, 1600.0, join, join, join),
            None
        );
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
                l.update(Some(Final), Intercept, 50.0, 0.2, -0.2, FAP_NM)
                    .is_some(),
                "latched low"
            );
            assert_eq!(l.attempts(), n, "counted");
            assert!(
                l.update(Some(Intercept), Intercept, 50.0, 0.3, -0.3, FAP_NM)
                    .is_some(),
                "stays latched climbing"
            );
            assert!(
                l.update(Some(Intercept), Intercept, 900.0, 2.0, -2.0, FAP_NM)
                    .is_some(),
                "and stays latched repositioning"
            );
            assert!(
                l.update(Some(Intercept), Intercept, 1600.0, 5.5, 5.5, FAP_NM)
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
        let r = runway_by_id(p, 36.15, -5.34, "27", 40.0)
            .unwrap()
            .expect("27 is at the field");
        assert_eq!(r.id, "27");
        assert!(r.hard(), "27 is asphalt");
        // Its reciprocal is equally resolvable.
        assert_eq!(
            runway_by_id(p, 36.15, -5.34, "09", 40.0)
                .unwrap()
                .unwrap()
                .id,
            "09"
        );
        // The grass end is filtered out (not a landing runway for autoland).
        assert!(
            runway_by_id(p, 36.15, -5.34, "05", 40.0).unwrap().is_none(),
            "grass is skipped"
        );
        // An ident that isn't here → None (caller falls back to track pick).
        assert!(
            runway_by_id(p, 36.15, -5.34, "18", 40.0).unwrap().is_none(),
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
    // five minutes. This pins the DIFFERENCE between the two picks: on downwind
    // (heading away from the landing end), the track search does not return
    // the near field, and forcing the ident does.
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
        let forced = runway_by_id(p, lat, lon, "20", 40.0)
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
}
