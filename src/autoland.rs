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
    let end1 = RunwayEnd { id: f[8].to_string(), lat: lat1, lon: lon1, course: bearing_deg(lat1, lon1, lat2, lon2) };
    let end2 = RunwayEnd { id: f[17].to_string(), lat: lat2, lon: lon2, course: bearing_deg(lat2, lon2, lat1, lon1) };
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
pub fn nearby_runways(apt_dat: &str, lat: f64, lon: f64, max_nm: f64) -> std::io::Result<Vec<Runway>> {
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
                let Some((e1, e2, length_m, surface)) = parse_runway_100(&line) else { continue };
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
    out.sort_by(|a, b| a.dist_nm.partial_cmp(&b.dist_nm).unwrap_or(std::cmp::Ordering::Equal));
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
        score(a).partial_cmp(&score(b)).unwrap_or(std::cmp::Ordering::Equal)
    });
    Ok(best)
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
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
const VVI_FPM: &str = "sim/cockpit2/gauges/indicators/vvi_dial_fpm";
// AP + controls (mirror of autopilot.rs / recover.rs).
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
const HDG_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
const HDG_STATUS: &str = "sim/cockpit2/autopilot/heading_status";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
const THROTTLE_ALL: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";
const GEAR_HANDLE: &str = "sim/cockpit2/controls/gear_handle_down";
const FLAP_RATIO: &str = "sim/cockpit2/controls/flap_ratio";
const M_TO_FT: f64 = 3.280_84;
const MS_TO_KTS: f64 = 1.943_84;

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
    status: Option<crate::plugin_status::StatusSink>,
) {
    let my_gen = GEN.fetch_add(1, Ordering::SeqCst) + 1;
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else { return };
        rt.block_on(async move {
            let xp = XPlaneClient::new(base);
            ACTIVE.store(true, Ordering::SeqCst);
            let announce = |line: String, sev: Priority| {
                if let Some(s) = &status { s.action(&line); }
                if let Some(v) = &voice { v.alert(sev, line); }
            };

            // Find the runway (a full apt.dat pass, a few seconds — that's why the
            // tool says "searching"). Needs a valid position first.
            let (Ok(lat0), Ok(lon0), Ok(track0)) = (
                xp.get_float(LAT).await, xp.get_float(LON).await, xp.get_float(MAG_VAR).await,
            ) else { ACTIVE.store(false, Ordering::SeqCst); return };
            let _ = track0;
            let hdg0 = xp.get_float("sim/flightmodel/position/psi").await.unwrap_or(0.0);
            let rwy = match best_landing_runway(&apt_dat, lat0, lon0, hdg0, 40.0, 1200.0) {
                Ok(Some(r)) => r,
                other => {
                    tracing::warn!(target: "x_copilot::autoland", ?other, from=?(lat0, lon0), "no runway found");
                    announce("No suitable runway within range. You have the aircraft.".into(), Priority::Warning);
                    ACTIVE.store(false, Ordering::SeqCst);
                    return;
                }
            };
            tracing::info!(
                target: "x_copilot::autoland",
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
            let mut gear_down = false;
            let mut last_flap: Option<f64> = None;
            let mut vs_diverged: u32 = 0;
            // Airspeed history for the throttle governor's derivative term — the
            // anti-stall reflex needs the TREND (kts/sec), not just the error.
            let mut prev_ias: Option<f64> = None;
            let mut prev_ias_t = std::time::Instant::now();
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
                let (Ok(lat), Ok(lon), Ok(elev_m), Ok(agl_m), Ok(gs_ms), Ok(var), Ok(ias)) = (
                    xp.get_float(LAT).await, xp.get_float(LON).await, xp.get_float(ELEV_M).await,
                    xp.get_float(AGL_M).await, xp.get_float(GS_MS).await, xp.get_float(MAG_VAR).await,
                    xp.get_float(IAS).await,
                ) else { tokio::time::sleep(std::time::Duration::from_millis(300)).await; continue };
                let alt = elev_m * M_TO_FT;
                let radio_ft = agl_m * M_TO_FT;
                let gs_kts = gs_ms * MS_TO_KTS;

                let raw = guidance(lat, lon, alt, radio_ft, gs_kts, ias, &rwy, approach_ias, 25.0);
                // A missed approach called low is flown by the latch (full power,
                // straight ahead) until clear of the ground — the raw intercept
                // guidance turns back to the FAP immediately, which is a killer
                // at 50 ft (see [`GoAroundLatch`]).
                let ga = go_around.update(last_phase, raw.phase, radio_ft);
                let g = if ga {
                    go_around_guidance(&rwy, raw.dist_nm, raw.glidepath_alt_ft, ias, approach_ias)
                } else {
                    raw
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
                    target: "x_copilot::autoland",
                    dist = g.dist_nm, alt, gp_alt = g.glidepath_alt_ft, vs = g.target_vs_fpm,
                    hdg = g.target_heading_deg, ias, radio = radio_ft, phase = ?g.phase,
                    "autoland"
                );
                crate::blackbox::record("autoland", serde_json::json!({
                    "rwy": format!("{} {}", rwy.icao, rwy.id), "dist": g.dist_nm,
                    "alt": alt, "gp_alt": g.glidepath_alt_ft, "cmd_vs": g.target_vs_fpm,
                    "cmd_hdg": g.target_heading_deg, "ias": ias, "radio": radio_ft,
                    "phase": format!("{:?}", g.phase), "ga": ga,
                }));

                // Lateral: heading bug in MAGNETIC (guidance is TRUE — see
                // [`true_to_mag`] for the variation sign trap).
                let mag_hdg = true_to_mag(g.target_heading_deg, var);
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
                let thr_now = xp.get_float(THROTTLE_ALL).await.unwrap_or(0.5);
                let thr = if ga {
                    // Go-around is firewall, not a governor holding approach
                    // speed — the LXGB miss flew its last 20 seconds at the
                    // governor's idea of "on speed" with the ground coming up.
                    1.0
                } else if g.phase == ApproachPhase::Flare {
                    0.0
                } else {
                    let adj = crate::tools::speedhold::throttle_adjust(g.target_ias_kts - ias, trend_kts_s);
                    (thr_now + adj).clamp(0.1, 1.0)
                };
                let _ = xp.set_float(THROTTLE_ALL, thr).await;

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

                // Touchdown: on the ground and slowing → done.
                if radio_ft < 5.0 && gs_kts < 40.0 {
                    let _ = xp.set_float(THROTTLE_ALL, 0.0).await;
                    announce("Down. You have the aircraft.".into(), Priority::Warning);
                    break;
                }
                tokio::time::sleep(std::time::Duration::from_millis(400)).await;
            }
            set_panel("");
            ACTIVE.store(false, Ordering::SeqCst);
        });
    });
}

/// Feet of altitude per nm along a 3° glidepath (tan 3° × 6076 ft/nm ≈ 318).
const GLIDEPATH_FT_PER_NM: f64 = 318.0;
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
/// Only climb with this much surplus over the approach speed. Climbing slow in a
/// marginal airframe bleeds energy to a stall (the reason this loop was
/// descent-only) — so a low-and-slow aircraft holds level and lets the throttle
/// governor rebuild speed FIRST, then the climb engages once the surplus exists.
const CLIMB_SPEED_MARGIN_KTS: f64 = 5.0;
/// Ceiling on the safe-altitude climb (fpm) — gentle and energy-preserving.
const MAX_CLIMB_FPM: f64 = 500.0;
/// While below the terrain floor, command the throttle governor to this much over
/// the approach speed, so it builds the surplus the climb gate needs.
const CLIMB_ENERGY_BONUS_KTS: f64 = 12.0;

/// Flare only inside this distance to the threshold — beyond it, a low radio
/// altitude is terrain under the approach, not the runway. (25 ft on a 3° path
/// is ~0.08 nm out; 0.35 leaves margin for displaced thresholds.)
const FLARE_MAX_DIST_NM: f64 = 0.35;

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
#[derive(Debug, Default)]
pub struct GoAroundLatch {
    active: bool,
}

impl GoAroundLatch {
    pub fn new() -> Self {
        Self::default()
    }

    /// Advance the latch one tick: `prev`/`now` are the raw guidance phases,
    /// `radio_ft` the radio altitude. Returns whether the latch is flying.
    pub fn update(&mut self, prev: Option<ApproachPhase>, now: ApproachPhase, radio_ft: f64) -> bool {
        if self.active {
            // Clear of the ground — hand the turn back to the intercept guidance.
            if radio_ft >= TERRAIN_FLOOR_AGL_FT {
                self.active = false;
            }
        } else if now == ApproachPhase::Intercept
            && matches!(prev, Some(ApproachPhase::Final | ApproachPhase::Flare))
            && radio_ft < TERRAIN_FLOOR_AGL_FT
        {
            self.active = true;
        }
        self.active
    }
}

/// The command a latched go-around flies: runway heading (straight ahead, no
/// turn near the ground), a bounded climb once the speed is there — level
/// acceleration first, because climbing slow bleeds energy to a stall — and a
/// raised speed target for the throttle (the loop firewall it anyway while
/// latched; the target keeps the governor sane on the release tick).
pub fn go_around_guidance(
    rwy: &Runway,
    dist_nm: f64,
    glidepath_alt_ft: f64,
    ias_kts: f64,
    approach_ias: f64,
) -> Guidance {
    let target_vs = if approach_ias - ias_kts > SPEED_FLOOR_MARGIN_KTS { 0.0 } else { MAX_CLIMB_FPM };
    Guidance {
        target_heading_deg: rwy.course_deg,
        target_vs_fpm: target_vs,
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
) -> Guidance {
    let dist_nm = haversine_nm(cur_lat, cur_lon, rwy.thr_lat, rwy.thr_lon);
    let recip = (rwy.course_deg + 180.0) % 360.0;

    // 3° glidepath altitude at this distance (+ = we're above it).
    let glidepath_alt_ft = rwy.elev_ft + dist_nm * GLIDEPATH_FT_PER_NM;
    let alt_err = cur_alt_ft - glidepath_alt_ft;

    // Above this much excess, we can't just dive onto the path — fly to the JOIN
    // point instead: where a 3° descent (with 15% margin) meets our CURRENT
    // altitude, out along the extended centreline. When we're inside it (armed
    // high overhead — the failure seen live: 7,000 ft at 6 nm, VS pegged, never
    // capturing) that means flying OUTBOUND first; the point walks back toward
    // the FAP as altitude burns off (a teardrop join).
    const CAPTURE_ABOVE_FT: f64 = 500.0;
    let join_nm = ((cur_alt_ft - rwy.elev_ft) / GLIDEPATH_FT_PER_NM * 1.15)
        .clamp(FAP_NM, 40.0);

    // Are we past the threshold, on the departure side, still airborne? (The flare
    // branch below, checked first, owns a normal float/rollout across the numbers.)
    let past_threshold = along_track_nm(cur_lat, cur_lon, rwy) < -OVERFLIGHT_GO_AROUND_NM;

    let (phase, target_heading) = if radio_alt_ft <= flare_radio_ft && dist_nm <= FLARE_MAX_DIST_NM {
        // Low radio altitude alone is NOT the flare cue — terrain rises to meet
        // the approach path miles from the field (seen live: "flared" at 7 ft AGL
        // 4.1 nm out at LEMI, power cut over open ground; and again into a dip
        // 0.4 nm short at LERL). Flare only over the runway environment.
        (ApproachPhase::Flare, rwy.course_deg)
    } else if past_threshold {
        // Overflew the runway without landing (too high/fast at the numbers). Go
        // around: steer BACK to the Final Approach Point, never track the runway
        // course outbound — that departs the field forever (seen live 2026-07-06:
        // over the numbers at 167 ft, then straight out to 5.4 nm at 60 kt, never
        // turning back). The vertical law climbs to a safe altitude on the way, if
        // energy allows. Takes priority over the join/FAP/final branches so a
        // go-around always turns toward the field regardless of altitude.
        let (fap_lat, fap_lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, FAP_NM);
        (ApproachPhase::Intercept, bearing_deg(cur_lat, cur_lon, fap_lat, fap_lon))
    } else if alt_err > CAPTURE_ABOVE_FT {
        // Too high for this distance — descend toward the join point.
        let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
        (ApproachPhase::Intercept, bearing_deg(cur_lat, cur_lon, jlat, jlon))
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
        (ApproachPhase::Intercept, bearing_deg(cur_lat, cur_lon, fap_lat, fap_lon))
    } else {
        // On final: track the runway course, corrected for lateral offset. The
        // gain is per-nm of cross-track: at 8 the LXGB final entered 0.45 nm
        // off and commanded all of 3.6° of cut — it crossed the threshold still
        // ~120 m off the centreline (2026-07-14). 40°/nm reaches the 30° clamp
        // at 0.75 nm out and still decays to a gentle 2-4° inside 0.1 nm.
        let xtk = cross_track_nm(cur_lat, cur_lon, rwy);
        let corr = (xtk * 40.0).clamp(-30.0, 30.0); // deg toward the centreline
        (ApproachPhase::Final, (rwy.course_deg - corr + 360.0) % 360.0)
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
        // short-final range where low radio altitude IS the approach.
        ApproachPhase::Final => {
            radio_alt_ft < TERRAIN_FLOOR_AGL_FT && dist_nm > FLOOR_ENFORCE_MIN_DIST_NM
        }
        // Intercept is repositioning, not landing: the floor holds at ANY
        // distance (see FLOOR_ENFORCE_MIN_DIST_NM for the KPDK 16 accident).
        ApproachPhase::Intercept => radio_alt_ft < TERRAIN_FLOOR_AGL_FT,
    };
    let has_climb_energy = ias_kts > approach_ias + CLIMB_SPEED_MARGIN_KTS;

    let speed_deficit = approach_ias - ias_kts;
    let target_vs = if phase != ApproachPhase::Flare && speed_deficit > SPEED_FLOOR_MARGIN_KTS {
        -(speed_deficit * 40.0).clamp(200.0, 700.0)
    } else if below_terrain_floor && has_climb_energy {
        // Bounded climb toward the floor, steeper the deeper below it we are.
        ((TERRAIN_FLOOR_AGL_FT - radio_alt_ft) * 2.0).clamp(100.0, MAX_CLIMB_FPM)
    } else {
        match phase {
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
            _ => (nominal_vs - alt_err * 0.4).clamp(-1200.0, 0.0),
        }
    };

    // Below the terrain floor, ask the throttle governor for extra speed so it
    // builds the surplus the climb gate needs; otherwise hold the approach speed.
    let target_ias = if below_terrain_floor { approach_ias + CLIMB_ENERGY_BONUS_KTS } else { approach_ias };

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

    fn rwy_north() -> Runway {
        // A runway at the equator, landing due north (course 0), threshold at 0,0.
        Runway {
            icao: "TEST".into(), id: "36".into(), thr_lat: 0.0, thr_lon: 0.0,
            course_deg: 0.0, length_m: 3000.0, surface: 1, dist_nm: 0.0, elev_ft: 0.0,
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
        let g = guidance(lat, lon, 2000.0, 2000.0, 90.0, 90.0, &rwy_north(), 90.0, 20.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
        // FAP is 5 nm SOUTH of the threshold; from 8 nm north that's ~due south.
        assert!(heading_diff(g.target_heading_deg, 180.0) < 20.0,
            "must steer toward the FAP/field, got hdg {}", g.target_heading_deg);
    }

    #[test]
    fn speed_protection_lowers_the_nose_before_a_stall() {
        // The live failure (XLE000N): stuck below the glidepath 13 nm out, holding
        // level, IAS decaying to 53 kt until the AP stalled off. Below the path AND
        // slow, the guidance must now command a DESCENT to regain speed, not a level
        // hold. 5 nm south, well below the path, 55 kt against a 90 kt approach.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 5.0);
        let g = guidance(lat, lon, 200.0, 200.0, 55.0, 55.0, &rwy_north(), 90.0, 20.0);
        assert!(g.target_vs_fpm < -100.0, "slow + below path must descend, got {}", g.target_vs_fpm);
        // On speed at the same spot, it holds level to wait for the path (no dive).
        let g_ok = guidance(lat, lon, 200.0, 200.0, 90.0, 90.0, &rwy_north(), 90.0, 20.0);
        assert_eq!(g_ok.target_vs_fpm, 0.0, "on speed below path → level hold");
        // Speed protection never fights the flare (we WANT to slow down landing).
        let g_flare = guidance(0.001, 0.0, 30.0, 15.0, 40.0, 40.0, &rwy_north(), 90.0, 20.0);
        assert_eq!(g_flare.phase, ApproachPhase::Flare);
        assert!(g_flare.target_vs_fpm > -200.0, "flare not overridden by speed floor");
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
        let g = guidance(lat, lon, 200.0, 200.0, 90.0, 90.0, &rwy_north(), 90.0, 25.0);
        assert_eq!(g.phase, ApproachPhase::Intercept, "past the threshold → go around");
        assert!(heading_diff(g.target_heading_deg, 180.0) < 25.0,
            "must turn back toward the FAP, got hdg {}", g.target_heading_deg);
        // Sanity: a mirror point 0.5 nm SHORT of the threshold (approach side) is a
        // normal final, not a go-around.
        let (slat, slon) = project_latlon(0.0, 0.0, 180.0, 0.5);
        let g_final = guidance(slat, slon, 200.0, 200.0, 90.0, 90.0, &rwy_north(), 90.0, 25.0);
        assert_ne!(g_final.phase, ApproachPhase::Intercept, "short of the threshold is not a go-around");
    }

    #[test]
    fn climbs_off_the_terrain_floor_only_with_energy() {
        // The live failure (2026-07-06): level at ~50 ft AGL 5 nm out over rising
        // terrain, no climb anywhere in the loop. 3 nm out, 200 ft AGL (below the
        // 500 ft floor), on the approach side.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 3.0);
        // FAST (approach + 10): surplus energy → bounded climb away from terrain.
        let g_fast = guidance(lat, lon, 200.0, 200.0, 100.0, 100.0, &rwy_north(), 90.0, 25.0);
        assert!(g_fast.target_vs_fpm > 0.0, "below the floor + fast → climb, got {}", g_fast.target_vs_fpm);
        assert!(g_fast.target_vs_fpm <= MAX_CLIMB_FPM, "climb is bounded, got {}", g_fast.target_vs_fpm);
        // And it asks the throttle for extra speed to sustain the climb.
        assert!(g_fast.target_ias_kts > 90.0, "climb raises the speed target, got {}", g_fast.target_ias_kts);
        // ON SPEED (no surplus): must NOT climb — hold level and let the governor
        // rebuild speed first (climbing slow would stall).
        let g_slow = guidance(lat, lon, 200.0, 200.0, 90.0, 90.0, &rwy_north(), 90.0, 25.0);
        assert_eq!(g_slow.target_vs_fpm, 0.0, "below the floor but no surplus → level, got {}", g_slow.target_vs_fpm);
    }

    #[test]
    fn terrain_floor_is_not_enforced_on_short_final() {
        // Low AGL close in is the approach, not terrain — don't climb away from it.
        // 0.5 nm short of the threshold, 200 ft AGL, fast: no climb (inside the
        // floor-enforcement range).
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.5);
        let g = guidance(lat, lon, 200.0, 200.0, 100.0, 100.0, &rwy_north(), 90.0, 25.0);
        assert!(g.target_vs_fpm <= 0.0, "no climb on short final, got {}", g.target_vs_fpm);
        assert_eq!(g.target_ias_kts, 90.0, "no climb-energy bonus on short final");
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
        let g_fast = guidance(lat, lon, 300.0, 300.0, 100.0, 100.0, &rwy_north(), 90.0, 25.0);
        assert_eq!(g_fast.phase, ApproachPhase::Intercept);
        assert!(g_fast.target_vs_fpm > 0.0, "repositioning low + fast → climb, got {}", g_fast.target_vs_fpm);
        let g_slow = guidance(lat, lon, 300.0, 300.0, 90.0, 90.0, &rwy_north(), 90.0, 25.0);
        assert_eq!(g_slow.phase, ApproachPhase::Intercept);
        assert_eq!(g_slow.target_vs_fpm, 0.0, "repositioning low + no surplus → level, got {}", g_slow.target_vs_fpm);
    }

    #[test]
    fn low_go_around_latches_straight_ahead_until_the_floor() {
        // The LXGB 09 crash (2026-07-14): overflew the numbers at ~57 ft, the
        // intercept guidance flipped the heading 178° toward the FAP, the flare
        // re-fired past the threshold and pulled the power, and the aircraft
        // hit 200 m south of the strip. Low, the latch must own the go-around.
        use ApproachPhase::*;
        let mut latch = GoAroundLatch::new();
        // Final ticking along at 60 ft: not a go-around.
        assert!(!latch.update(Some(Final), Final, 60.0));
        // Overflight: Final degrades to Intercept at 57 ft → latch.
        assert!(latch.update(Some(Final), Intercept, 57.0));
        // The raw guidance flickering (Intercept → Flare past the threshold →
        // Intercept) must not shake it loose while still low.
        assert!(latch.update(Some(Intercept), Flare, 22.0));
        assert!(latch.update(Some(Flare), Intercept, 35.0));
        // Clear of the ground: release — the normal intercept turn is safe now.
        assert!(!latch.update(Some(Intercept), Intercept, 520.0));
        // A HIGH overflight never latches: the immediate turn back is fine.
        let mut high = GoAroundLatch::new();
        assert!(!high.update(Some(Final), Intercept, 800.0));
    }

    #[test]
    fn go_around_guidance_climbs_straight_ahead_on_speed() {
        let rwy = rwy_north();
        // On speed: full climb, straight down the runway heading.
        let g = go_around_guidance(&rwy, 0.3, 100.0, 92.0, 90.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
        assert!(heading_diff(g.target_heading_deg, rwy.course_deg) < 0.1, "straight ahead");
        assert!(g.target_vs_fpm > 0.0 && g.target_vs_fpm <= MAX_CLIMB_FPM);
        // Slow (deficit beyond the floor margin): level acceleration first —
        // climbing slow at full flap is how a go-around stalls.
        let g_slow = go_around_guidance(&rwy, 0.3, 100.0, 75.0, 90.0);
        assert_eq!(g_slow.target_vs_fpm, 0.0, "build speed before climbing");
        assert!(heading_diff(g_slow.target_heading_deg, rwy.course_deg) < 0.1);
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
        let g = guidance(lat, east, 900.0, 900.0, 90.0, 90.0, &rwy, 90.0, 20.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        // Right of course → cut LEFT (west of north): 0.45 × 40 clamps to 30.
        let cut = ((rwy.course_deg - g.target_heading_deg + 540.0) % 360.0) - 180.0;
        assert!(cut > 15.0, "0.45 nm off must earn a real cut, got {cut}");
        // Nearly on the centreline: gentle.
        let near = lon + 0.03 / 60.0;
        let g2 = guidance(lat, near, 900.0, 900.0, 90.0, 90.0, &rwy, 90.0, 20.0);
        let cut2 = ((rwy.course_deg - g2.target_heading_deg + 540.0) % 360.0) - 180.0;
        assert!(cut2.abs() < 5.0, "small offset stays gentle, got {cut2}");
    }

    #[test]
    fn glidepath_descends_toward_the_threshold() {
        // 5 nm south of the threshold at 2000 ft, 120 kt groundspeed, tracking north.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 5.0); // 5 nm south
        let g = guidance(lat, lon, 2000.0, 2000.0, 120.0, 90.0, &rwy_north(), 90.0, 20.0);
        assert!((g.dist_nm - 5.0).abs() < 0.2, "5 nm out, got {}", g.dist_nm);
        // Glidepath alt at 5 nm ≈ 1590 ft; we're above it → commanded to descend.
        assert!(g.glidepath_alt_ft > 1400.0 && g.glidepath_alt_ft < 1700.0);
        assert!(g.target_vs_fpm < 0.0, "should descend, got {}", g.target_vs_fpm);
        // Roughly on the centreline → heading close to the runway course (north).
        assert!(heading_diff(g.target_heading_deg, 0.0) < 10.0, "hdg {}", g.target_heading_deg);
    }

    #[test]
    fn armed_high_overhead_flies_outbound_to_a_join_point() {
        // The live failure: 7,000 ft only 6 nm out (path altitude ~1,900) — the
        // old guidance aimed at the FAP and could never capture. Now it must fly
        // AWAY from the runway (toward the join point down the extended
        // centreline) and descend.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 6.0); // 6 nm south
        let g = guidance(lat, lon, 7000.0, 7000.0, 135.0, 110.0, &rwy_north(), 110.0, 20.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
        // Runway is north (bearing ~0); the join point is further SOUTH — so the
        // commanded heading must be southbound, not at the runway.
        assert!(heading_diff(g.target_heading_deg, 180.0) < 30.0,
            "should fly outbound, got hdg {}", g.target_heading_deg);
        assert!(g.target_vs_fpm <= -1000.0, "and descend hard, got {}", g.target_vs_fpm);
    }

    #[test]
    fn flare_near_the_ground_arrests_descent_and_tracks_runway() {
        let g = guidance(0.001, 0.0, 30.0, 15.0, 100.0, 90.0, &rwy_north(), 90.0, 20.0);
        assert_eq!(g.phase, ApproachPhase::Flare);
        assert!(g.target_vs_fpm > -200.0, "flare eases the descent, got {}", g.target_vs_fpm);
        assert!(heading_diff(g.target_heading_deg, 0.0) < 1.0, "flare tracks the runway");
    }

    #[test]
    fn low_radio_far_from_the_runway_is_terrain_not_a_flare() {
        // The LEMI failure: 7 ft AGL over rising terrain 4.1 nm out — the old
        // radio-only trigger "flared" (power to idle, vs -100) into open ground.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 4.1); // 4.1 nm south
        let g = guidance(lat, lon, 900.0, 7.0, 130.0, 90.0, &rwy_north(), 90.0, 25.0);
        assert_ne!(g.phase, ApproachPhase::Flare, "no flare 4 nm from the threshold");
        // And the same low radio just short of the threshold IS the flare.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.1);
        let g = guidance(lat, lon, 25.0, 15.0, 90.0, 90.0, &rwy_north(), 90.0, 25.0);
        assert_eq!(g.phase, ApproachPhase::Flare);
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
        assert!(cross_track_nm(lat, lon, &rwy_north()).abs() < 0.05, "on centreline ~0");
        let east = (lat, lon + 0.02);
        assert!(cross_track_nm(east.0, east.1, &rwy_north()) > 0.0, "east = right = positive");
    }

    #[test]
    fn bearing_cardinals() {
        // Due north / east / south / west from the equator.
        assert!((bearing_deg(0.0, 0.0, 1.0, 0.0) - 0.0).abs() < 0.5, "north");
        assert!((bearing_deg(0.0, 0.0, 0.0, 1.0) - 90.0).abs() < 0.5, "east");
        assert!((bearing_deg(0.0, 0.0, -1.0, 0.0) - 180.0).abs() < 0.5, "south");
        assert!((bearing_deg(0.0, 0.0, 0.0, -1.0) - 270.0).abs() < 0.5, "west");
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
        assert!(length_m > 300.0 && length_m < 800.0, "length was {length_m}");
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
        println!("selected: {} rwy {} @ {:.0} nm, {:.0} m, course {:.0}",
            best.icao, best.id, best.dist_nm, best.length_m, best.course_deg);
        assert_eq!(best.icao, "KSEA");
        assert!(best.length_m > 2000.0, "KSEA runways are long");
        assert!(best.dist_nm < 5.0, "should be right at the field");
    }
}
