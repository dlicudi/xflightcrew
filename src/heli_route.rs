//! A helicopter flies the flight plan: the hover loop for the ends, the
//! autopilot's cruise modes in between.
//!
//! What the S-76C showed on 2026-09-23 (EDDV) decides the split. The hover
//! mode follows a moving groundspeed target well up to about 40 kt and lets
//! go near 47, never to re-engage that fast; heading and altitude hold flew
//! it steadily from 54 to 81 kt on a fixed collective. So:
//!
//! - **Depart** on the hover loop ([`hover::Goal::Fly`]) toward the first
//!   fix, level until flying, then a climb to [`CRUISE_AGL_FT`] at 35 kt.
//! - **Hand over** in level flight inside the hover mode's range: the loop is
//!   released WITHOUT the autopilot going off ([`hover::release`]), the
//!   heading bug and altitude dial are set, heading and altitude hold engage
//!   ([`hover::cruise_modes`]), and the collective becomes [`SpeedLaw`]'s.
//! - **Cruise**: altitude hold works the pitch, the collective the airspeed
//!   (the helicopter's own split, the inverse of the hover's), the bug steers
//!   fix to fix, turning early by the turn's size.
//! - **Slow** on the last leg: the same law to [`HANDOVER_KTS`], level, so the
//!   hover loop takes over a steady aircraft. The morning's takeover caught
//!   one decelerating hard at a cruise collective, and the pitch rang.
//! - **Arrive** on the hover loop: a final at 6 degrees to the helipad nearest
//!   the inbound fix, the speed tapering to stop over it, then down.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use crate::hover::{self, Goal};
use crate::taxi_guide::{angle_diff, bearing, dist, Geo};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

pub static ACTIVE: AtomicBool = AtomicBool::new(false);

pub fn abort() {
    ACTIVE.store(false, Ordering::SeqCst);
}

static DETAIL: crate::auto_status::Detail = crate::auto_status::Detail::new();

pub fn status() -> Option<String> {
    ACTIVE.load(Ordering::SeqCst).then(|| DETAIL.line("route"))
}

/// Cruise height above the departure ground, and the airspeeds.
pub const CRUISE_AGL_FT: f64 = 1000.0;
pub const CRUISE_KTS: f64 = 120.0;
/// The hover mode's side of the hand-over, both ways: level, inside the
/// 40 kt the mode has been flown at.
pub const HANDOVER_KTS: f64 = 38.0;
const DEPART_MPS: f64 = 18.0;
/// The final: its path, its deceleration, the hover it ends in.
const GLIDE_DEG: f64 = 6.0;
const FINAL_DECEL_MPS2: f64 = 0.3;
const FINAL_HOVER_FT: f64 = 30.0;
const ARRIVED_M: f64 = 12.0;
/// A standard-rate turn's radius at a groundspeed, m.
fn turn_radius_m(gs_mps: f64) -> f64 {
    gs_mps / 3f64.to_radians()
}
const TICK: Duration = Duration::from_millis(500);
/// The heading bug is moved only by a course change at least this big.
const BUG_HYSTERESIS_DEG: f64 = 2.0;

/// The collective for an airspeed under altitude hold: altitude hold holds
/// the height on the pitch, so power is speed. P and a slow trim that starts
/// at the lever the hand-over left, slewed like a pilot's hand. The target
/// itself is ramped so an acceleration or a slow-down is gentle.
#[derive(Debug, Clone, Copy)]
pub struct SpeedLaw {
    cmd: f64,
    trim: f64,
    target_kts: f64,
}

const SPEED_KP_PER_KT: f64 = 0.008;
const SPEED_KI_PER_KT_S: f64 = 0.0005;
const COLLECTIVE_SLEW: f64 = 0.03;
const COLLECTIVE_MIN: f64 = 0.25;
const COLLECTIVE_MAX: f64 = 0.85;
const ACCEL_KTS_S: f64 = 1.5;
const DECEL_KTS_S: f64 = 1.0;

impl SpeedLaw {
    pub fn from(collective: f64, ias_kts: f64) -> Self {
        Self {
            cmd: collective,
            trim: collective,
            target_kts: ias_kts,
        }
    }
    pub fn target(&self) -> f64 {
        self.target_kts
    }
    pub fn step(&mut self, want_kts: f64, ias_kts: f64, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        let step = if want_kts > self.target_kts {
            ACCEL_KTS_S
        } else {
            DECEL_KTS_S
        } * dt;
        self.target_kts += (want_kts - self.target_kts).clamp(-step, step);
        let err = self.target_kts - ias_kts;
        self.trim =
            (self.trim + err * SPEED_KI_PER_KT_S * dt).clamp(COLLECTIVE_MIN, COLLECTIVE_MAX);
        let want = (self.trim + err * SPEED_KP_PER_KT).clamp(COLLECTIVE_MIN, COLLECTIVE_MAX);
        self.cmd += (want - self.cmd).clamp(-COLLECTIVE_SLEW * dt, COLLECTIVE_SLEW * dt);
        self.cmd
    }
}

/// The final's command at `d` metres from the pad, from a height and speed it
/// starts at: a speed that stops on the pad, and a height on the path led by
/// the hover loop's lag (see [`crate::heli_circuit::final_command`]).
pub fn final_command(d_m: f64, from_ft: f64, from_mps: f64) -> (f64, f64) {
    let speed = (2.0 * FINAL_DECEL_MPS2 * d_m.max(0.0)).sqrt().min(from_mps);
    let path = FINAL_HOVER_FT + d_m * GLIDE_DEG.to_radians().tan() * 3.28084;
    let sink_fpm = speed * GLIDE_DEG.to_radians().tan() * 196.85;
    let lead = sink_fpm / hover::HEIGHT_GAIN_FPM_PER_FT;
    let height = if path >= from_ft {
        from_ft
    } else {
        (path - lead).max(FINAL_HOVER_FT)
    };
    (speed, height)
}

/// The route's ground-track sampled every `step_m` (lat, lon), both ends
/// included.
pub fn sample_legs(line: &[(f64, f64)], step_m: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for w in line.windows(2) {
        let geo = Geo::new(w[0].0);
        let d = dist((0.0, 0.0), geo.to_m(w[0], w[1]));
        let n = ((d / step_m).ceil() as usize).max(1);
        for i in 0..n {
            let t = i as f64 / n as f64;
            out.push((
                w[0].0 + (w[1].0 - w[0].0) * t,
                w[0].1 + (w[1].1 - w[0].1) * t,
            ));
        }
    }
    if let Some(last) = line.last() {
        out.push(*last);
    }
    out
}

/// The cruise altitude (ft, the dial's): [`CRUISE_AGL_FT`] over the higher of
/// the departure ground and the highest ground on the route, to the hundred.
pub fn cruise_altitude_ft(ground_ft: f64, terrain_ft: Option<f64>) -> f64 {
    let floor = terrain_ft.map_or(ground_ft, |t| t.max(ground_ft));
    ((floor + CRUISE_AGL_FT) / 100.0).ceil() * 100.0
}

/// Where the slow-down starts, metres before the pad: the deceleration from
/// cruise to the hand-over speed at the law's rate, then the final's length
/// from the cruise height on the glidepath, and a margin.
pub fn slow_start_m(cruise_kts: f64, agl_ft: f64) -> f64 {
    let kt = 0.5144;
    let secs = (cruise_kts - HANDOVER_KTS).max(0.0) / DECEL_KTS_S;
    let decel_m = secs * (cruise_kts + HANDOVER_KTS) / 2.0 * kt;
    let final_m = (agl_ft - FINAL_HOVER_FT).max(0.0) / 3.28084 / GLIDE_DEG.to_radians().tan();
    decel_m + final_m + 1000.0
}

/// The helipads of a field in apt.dat (row 102), (lat, lon, name).
pub fn helipads(apt_dat: &str, icao: &str) -> Vec<(f64, f64, String)> {
    let Ok(text) = std::fs::read_to_string(apt_dat) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    let mut inside = false;
    for line in text.lines() {
        let mut f = line.split_whitespace();
        match f.next() {
            Some("1") | Some("16") | Some("17") => {
                inside = line.split_whitespace().nth(4) == Some(icao);
                if !inside && !out.is_empty() {
                    break;
                }
            }
            Some("102") if inside => {
                let name = f.next().unwrap_or("").to_string();
                let lat = f.next().and_then(|v| v.parse().ok());
                let lon = f.next().and_then(|v| v.parse().ok());
                if let (Some(lat), Some(lon)) = (lat, lon) {
                    out.push((lat, lon, name));
                }
            }
            _ => {}
        }
    }
    out
}

/// The pad nearest the inbound point (the fix before the destination).
pub fn pick_pad(pads: &[(f64, f64, String)], inbound: (f64, f64)) -> Option<(f64, f64, String)> {
    let geo = Geo::new(inbound.0);
    pads.iter()
        .min_by(|a, b| {
            let da = dist((0.0, 0.0), geo.to_m(inbound, (a.0, a.1)));
            let db = dist((0.0, 0.0), geo.to_m(inbound, (b.0, b.1)));
            da.total_cmp(&db)
        })
        .cloned()
}

/// A fix this close to the destination is a terminal fix (a field's VOR, an
/// approach point): a helicopter goes direct to its pad instead. CEL lies
/// past ETHC, and the route flew over the field to it and back, starting its
/// slow-down 1.5 km from the pad (2026-09-23).
const TERMINAL_FIX_M: f64 = 10_000.0;

/// The route with any fix just before the pad dropped when it is a terminal
/// fix (within [`TERMINAL_FIX_M`] of the pad). The origin and the pad stay.
pub fn drop_terminal_fixes(mut points: Vec<(String, f64, f64)>) -> Vec<(String, f64, f64)> {
    while points.len() >= 3 {
        let pad = points[points.len() - 1].clone();
        let fix = &points[points.len() - 2];
        let geo = Geo::new(pad.1);
        let d = dist((0.0, 0.0), geo.to_m((pad.1, pad.2), (fix.1, fix.2)));
        if d < TERMINAL_FIX_M {
            points.remove(points.len() - 2);
        } else {
            break;
        }
    }
    points
}

/// The distance still to fly: to the fix ahead, then leg by leg to the pad.
pub fn remaining_m(here: (f64, f64), pts: &[(f64, f64)], leg: usize) -> f64 {
    let Some(first) = pts.get(leg) else {
        return 0.0;
    };
    dist(here, *first) + pts[leg..].windows(2).map(|w| dist(w[0], w[1])).sum::<f64>()
}

/// The hover taxi's pace.
const PARK_MPS: f64 = 5.0;

/// Hover-taxi to a stand and set down on it: the hover loop flies it, lifted
/// first when on the ground. The final's taper brings the speed to zero over
/// the stand at the taxi height, then [`Goal::Land`].
/// Where the hover taxi goes: straight to a stand, or along the tower's
/// cleared taxi route (lat, lon nodes) to its end - the way a helicopter
/// air-taxis for real, over the taxiways, never across the buildings and
/// the parked aircraft a straight line can cross.
pub enum ParkTarget {
    Route { path: Vec<(f64, f64)>, via: String },
}

/// Along a polyline (metres): the point `ahead` past the projection of
/// `here`, and the distance still to go to its end.
pub fn pursue(path: &[(f64, f64)], here: (f64, f64), ahead: f64) -> Option<((f64, f64), f64)> {
    if path.len() < 2 {
        return path.first().map(|p| (*p, dist(here, *p)));
    }
    let mut cum = vec![0.0];
    for w in path.windows(2) {
        cum.push(cum.last().unwrap() + dist(w[0], w[1]));
    }
    let total = *cum.last().unwrap();
    // The projection: the nearest point on any segment.
    let mut best = (f64::INFINITY, 0.0);
    for (i, w) in path.windows(2).enumerate() {
        let (a, b) = (w[0], w[1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        let t = if len2 > 0.0 {
            (((here.0 - a.0) * dx + (here.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let p = (a.0 + t * dx, a.1 + t * dy);
        let d = dist(here, p);
        if d < best.0 {
            best = (d, cum[i] + t * len2.sqrt());
        }
    }
    let s = (best.1 + ahead).min(total);
    let i = cum
        .iter()
        .rposition(|c| *c <= s)
        .unwrap_or(0)
        .min(path.len() - 2);
    let seg = cum[i + 1] - cum[i];
    let t = if seg > 0.0 { (s - cum[i]) / seg } else { 0.0 };
    let (a, b) = (path[i], path[i + 1]);
    let target = (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
    Some((target, total - best.1 + best.0))
}

/// The hover taxi's pace factor for the turn ahead: the change of direction
/// between the route where we are and 25 m further along it. 1 on a
/// straight, 0.3 at a right angle or more.
pub fn corner_factor(pts: &[(f64, f64)], here: (f64, f64)) -> f64 {
    let (Some((near, _)), Some((far, _))) = (
        pursue(pts, here, 5.0),
        pursue(pts, here, ROUTE_AHEAD_M + 5.0),
    ) else {
        return 1.0;
    };
    let Some((mid, _)) = pursue(pts, here, ROUTE_AHEAD_M / 2.0) else {
        return 1.0;
    };
    let turn = angle_diff(bearing(mid, far), bearing(near, mid)).abs();
    (1.0 - turn / 90.0 * 0.7).clamp(0.3, 1.0)
}

/// The look-ahead along a cleared route, m.
const ROUTE_AHEAD_M: f64 = 25.0;
/// A route's taxi height: a hover taxi, above the ground cushion's worst.
pub const ROUTE_TAXI_FT: f64 = 12.0;

pub fn park(voice: Option<Voice>, base_url: String, collective: String, target: ParkTarget) {
    if ACTIVE.swap(true, Ordering::SeqCst) {
        return;
    }
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            ACTIVE.store(false, Ordering::SeqCst);
            return;
        };
        rt.block_on(fly_park(
            XPlaneClient::new(base_url.clone()),
            voice,
            base_url,
            collective,
            target,
        ));
        ACTIVE.store(false, Ordering::SeqCst);
    });
}

async fn fly_park(
    xp: XPlaneClient,
    voice: Option<Voice>,
    base_url: String,
    collective: String,
    target: ParkTarget,
) {
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    let (path_ll, name, height) = match &target {
        ParkTarget::Route { path, via } => {
            (path.clone(), format!("the stand via {via}"), ROUTE_TAXI_FT)
        }
    };
    if path_ll.is_empty() {
        return;
    }
    crate::blackbox::record(
        "heli_route",
        serde_json::json!({ "event": "park", "to": name, "nodes": path_ll.len() }),
    );
    // Up to the taxi height first, where we are.
    if !hover::ACTIVE.load(Ordering::SeqCst) {
        hover::request(
            base_url,
            voice.clone(),
            collective,
            Goal::Hover { height_ft: height },
        );
    } else {
        hover::retarget(Goal::Hover { height_ft: height });
    }
    let mut ready = false;
    let mut moving = false;
    while ACTIVE.load(Ordering::SeqCst) {
        tokio::time::sleep(TICK).await;
        if crate::xplane::sim_frozen(&xp).await {
            continue;
        }
        if !hover::ACTIVE.load(Ordering::SeqCst) {
            record_end(&voice, "ended_outside", "park");
            return;
        }
        let Some(s) = read(&xp).await else { continue };
        if !ready {
            // Hovering at the taxi height and steady: go.
            ready = (s.agl_ft - height).abs() < 3.0 && s.gs < 1.0;
            continue;
        }
        let geo = Geo::new(s.lat);
        let here = (0.0, 0.0);
        let pts: Vec<(f64, f64)> = path_ll
            .iter()
            .map(|p| geo.to_m((s.lat, s.lon), *p))
            .collect();
        let Some((to, d)) = pursue(&pts, here, ROUTE_AHEAD_M) else {
            continue;
        };
        // Anything but our own goal is someone else's decision.
        let ours = matches!(hover::goal(), Some(Goal::Fly { .. }))
            || (!moving && matches!(hover::goal(), Some(Goal::Hover { .. })));
        if !ours {
            record_end(&voice, "ended_outside", "park");
            return;
        }
        // Slower into a corner: the route's turn between here and the aim
        // point sets the pace, a right angle at 30 % of it.
        let (speed, _) = final_command(d, height, PARK_MPS * corner_factor(&pts, here));
        if d < ARRIVED_M || (moving && speed < 0.5) {
            hover::retarget(Goal::Land);
            crate::blackbox::record(
                "heli_route",
                serde_json::json!({ "event": "over_stand", "d_m": d }),
            );
            say(
                &voice,
                Priority::Readback,
                "Over the stand. Setting it down.",
            );
            return;
        }
        moving = true;
        hover::retarget(Goal::Fly {
            height_ft: height,
            speed_mps: speed,
            track_deg: bearing(here, to),
        });
        DETAIL.set(format!("to {name} {:.0} m", (d / 10.0).round() * 10.0));
    }
    crate::blackbox::record("heli_route", serde_json::json!({ "event": "stood_down" }));
}

pub struct Route {
    /// (ident, lat, lon), the destination's pad last.
    pub points: Vec<(String, f64, f64)>,
    pub destination: String,
    /// The destination's staffed tower (apt.dat), for the inbound call.
    pub dest_tower_khz: Option<i64>,
}

/// How long the departure call waits for a clearance before going anyway
/// (said): a tower that never answers must not strand a flight on the pad.
const DEPARTURE_WAIT_S: u64 = 60;

/// The departure call, the aeroplanes' own words: COM1 to the tower, "ready
/// for departure", then a bounded wait for the clearance. No simulated tower,
/// no call.
async fn call_departure(xp: &XPlaneClient, voice: &Option<Voice>) {
    use crate::tower::State::{LinedUp, TakeoffCleared};
    if !crate::mode::atc_enabled() {
        return;
    }
    let Some(khz) = crate::party_line::home_tower_khz() else {
        return;
    };
    let _ = xp
        .set_float(crate::datarefs::COM1_FREQ_KHZ, khz as f64)
        .await;
    let callsign = crate::circuit::crew_callsign(xp).await;
    crate::circuit::transmit(crate::circuit::radio_call(&callsign, "ready for departure"));
    DETAIL.set("waiting for the departure clearance");
    crate::blackbox::record(
        "heli_route",
        serde_json::json!({ "event": "departure_call", "khz": khz }),
    );
    for _ in 0..DEPARTURE_WAIT_S * 2 {
        tokio::time::sleep(Duration::from_millis(500)).await;
        if matches!(
            crate::party_line::clearance_state(),
            Some(LinedUp) | Some(TakeoffCleared)
        ) {
            crate::blackbox::record(
                "heli_route",
                serde_json::json!({ "event": "departure_cleared" }),
            );
            return;
        }
    }
    crate::blackbox::record(
        "heli_route",
        serde_json::json!({ "event": "departure_no_clearance" }),
    );
    say(
        voice,
        Priority::Caution,
        "No departure clearance came back. Departing.",
    );
}

/// The inbound call to the destination's tower, on its frequency, and the
/// station pinned to it for the arrival (the aeroplanes' `call_inbound`).
async fn call_inbound(xp: &XPlaneClient, khz: i64) {
    if !crate::mode::atc_enabled() {
        return;
    }
    let _ = xp
        .set_float(crate::datarefs::COM1_FREQ_KHZ, khz as f64)
        .await;
    let callsign = crate::circuit::crew_callsign(xp).await;
    crate::circuit::transmit(crate::circuit::radio_call(&callsign, "inbound for landing"));
    crate::station::pin_arrival_station(Some(khz));
    crate::blackbox::record(
        "heli_route",
        serde_json::json!({ "event": "inbound_call", "khz": khz }),
    );
}

/// The taxi-in's settings, for the parking at the destination (the same
/// [`crate::taxi_in::start`] the aeroplanes use).
#[derive(Clone)]
pub struct TaxiIn {
    pub park_brake_valve_set: Option<f64>,
    pub prop: Option<crate::aircraft::PropProfile>,
    pub pace: crate::autotaxi::Pace,
}

/// The whole flight's ends, around the route: the engine start when cold,
/// the lift to a hover from the ground, and the taxi to parking after the
/// landing - "fly the flight plan" does everything, as full auto does.
pub struct Ends {
    pub start: Option<crate::tools::heli::StartHelicopter>,
    pub lift: bool,
    pub taxi: TaxiIn,
}

pub fn start(
    voice: Option<Voice>,
    config: crate::config::Config,
    collective: String,
    route: Route,
    ends: Ends,
) {
    if ACTIVE.swap(true, Ordering::SeqCst) {
        return;
    }
    let base_url = config.xplane_base_url.clone();
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            ACTIVE.store(false, Ordering::SeqCst);
            return;
        };
        let xp = XPlaneClient::new(base_url.clone());
        let landed = rt.block_on(async {
            if !before_route(&xp, &voice, &config, &base_url, &collective, &ends).await {
                return false;
            }
            fly(
                XPlaneClient::new(base_url.clone()),
                voice.clone(),
                base_url.clone(),
                collective,
                route,
            )
            .await
        });
        ACTIVE.store(false, Ordering::SeqCst);
        // Down at the destination: the aeroplanes' own taxi-in (the tower's
        // clearance, then the hover taxi of its route).
        if landed && crate::mode::atc_enabled() && crate::party_line::home_tower_khz().is_some() {
            crate::blackbox::record("heli_route", serde_json::json!({ "event": "taxi_in" }));
            crate::taxi_in::start(
                base_url,
                config,
                voice,
                ends.taxi.park_brake_valve_set,
                ends.taxi.prop,
                ends.taxi.pace,
                None,
            );
        }
    });
}

fn say(voice: &Option<Voice>, p: Priority, text: impl Into<String>) {
    if let Some(v) = voice {
        v.alert(p, text);
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    Depart,
    Cruise,
    Slow,
    Arrive,
}

struct Sample {
    lat: f64,
    lon: f64,
    psi: f64,
    mag: f64,
    ias: f64,
    agl_ft: f64,
    gs: f64,
    theta: f64,
    phi: f64,
    ap: f64,
}

async fn read(xp: &XPlaneClient) -> Option<Sample> {
    use crate::datarefs::{LATITUDE, LONGITUDE, MAG_PSI};
    let (lat, lon, psi, mag, ias, agl, vx, vz, th, ph, ap) = tokio::join!(
        xp.get_float(LATITUDE),
        xp.get_float(LONGITUDE),
        xp.get_float("sim/flightmodel/position/psi"),
        xp.get_float(MAG_PSI),
        xp.get_float("sim/flightmodel/position/indicated_airspeed"),
        xp.get_float("sim/flightmodel/position/y_agl"),
        xp.get_float("sim/flightmodel/position/local_vx"),
        xp.get_float("sim/flightmodel/position/local_vz"),
        xp.get_float("sim/flightmodel/position/theta"),
        xp.get_float("sim/flightmodel/position/phi"),
        xp.get_float("sim/cockpit2/autopilot/autopilot_on"),
    );
    let (vx, vz) = (vx.ok()?, vz.ok()?);
    Some(Sample {
        lat: lat.ok()?,
        lon: lon.ok()?,
        psi: psi.ok()?,
        mag: mag.ok()?,
        ias: ias.ok()?,
        agl_ft: agl.ok()? * 3.28084,
        gs: vx.hypot(vz),
        theta: th.ok()?,
        phi: ph.ok()?,
        ap: ap.ok()?,
    })
}

/// The start and the lift, as the route asked for them. False = stop here
/// (said).
async fn before_route(
    xp: &XPlaneClient,
    voice: &Option<Voice>,
    config: &crate::config::Config,
    base_url: &str,
    collective: &str,
    ends: &Ends,
) -> bool {
    use crate::tools::Tool;
    if let Some(start) = &ends.start {
        DETAIL.set("starting the engines");
        let ctx = crate::tools::ToolCtx {
            xplane: xp,
            config,
            voice: voice.as_ref(),
            status: None,
            reminders: None,
        };
        let line = start
            .execute(&serde_json::json!({}), &ctx)
            .await
            .unwrap_or_else(|e| e.to_string());
        say(voice, Priority::Readback, line.clone());
        crate::blackbox::record(
            "heli_route",
            serde_json::json!({ "event": "started", "line": line }),
        );
        let running = xp
            .get_float_array("sim/flightmodel/engine/ENGN_running")
            .await
            .map(|v| v.iter().take(2).all(|r| *r > 0.5))
            .unwrap_or(false);
        if !running || line.contains("not fly") || line.contains("didn't") {
            say(
                voice,
                Priority::Caution,
                "The engines aren't right. No flight.",
            );
            return false;
        }
    }
    if ends.lift {
        call_departure(xp, voice).await;
        DETAIL.set("lifting");
        hover::request(
            base_url.to_string(),
            voice.clone(),
            collective.to_string(),
            Goal::Hover {
                height_ft: FINAL_HOVER_FT,
            },
        );
        // Hovering at the height and still: the route starts from there.
        for _ in 0..240 {
            tokio::time::sleep(TICK).await;
            if !ACTIVE.load(Ordering::SeqCst) || !hover::ACTIVE.load(Ordering::SeqCst) {
                return false;
            }
            if let Some(s) = read(xp).await {
                if (s.agl_ft - FINAL_HOVER_FT).abs() < 3.0 && s.gs < 1.0 {
                    return true;
                }
            }
        }
        say(
            voice,
            Priority::Caution,
            "The hover didn't settle. No flight.",
        );
        return false;
    }
    true
}

async fn fly(
    xp: XPlaneClient,
    voice: Option<Voice>,
    base_url: String,
    collective: String,
    route: Route,
) -> bool {
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    let Some(s0) = read(&xp).await else {
        say(
            &voice,
            Priority::Caution,
            "I can't read our position. No route.",
        );
        return false;
    };
    let origin = (s0.lat, s0.lon);
    let geo = Geo::new(s0.lat);
    let pts: Vec<(String, (f64, f64))> = route
        .points
        .iter()
        .map(|(id, la, lo)| (id.clone(), geo.to_m(origin, (*la, *lo))))
        .collect();
    let Some(pad) = pts.last().map(|p| p.1) else {
        return false;
    };
    // The departure ground's height sets the cruise level, in the
    // altimeter's feet (the dial's), measured once.
    let alt0 = xp
        .get_float("sim/cockpit2/gauges/indicators/altitude_ft_pilot")
        .await
        .unwrap_or(0.0);
    let ground_ft = alt0 - s0.agl_ft;
    // The cruise level is 1,000 ft over the HIGHEST ground on the route,
    // probed along every leg at a kilometre's spacing before departure; the
    // departure field is only the floor of it. No terrain data (an old
    // plugin) keeps the departure-ground rule and says so in the log.
    let mut line: Vec<(f64, f64)> = vec![origin];
    line.extend(route.points.iter().map(|p| (p.1, p.2)));
    let samples = sample_legs(&line, 1000.0);
    let terrain_ft = crate::terrain::max_elevation_ft(&xp, &samples).await;
    let cruise_alt_ft = cruise_altitude_ft(ground_ft, terrain_ft);
    // The departure climbs to it on the radio height over the field; altitude
    // hold then holds what it engages at.
    let depart_agl_ft = (cruise_alt_ft - ground_ft).max(CRUISE_AGL_FT);
    crate::blackbox::record(
        "heli_route",
        serde_json::json!({ "event": "start", "points": route.points.iter().map(|p| &p.0).collect::<Vec<_>>(), "cruise_alt_ft": cruise_alt_ft, "terrain_ft": terrain_ft, "samples": samples.len() }),
    );

    let mut phase = Phase::Depart;
    let mut leg = 0usize;
    let mut law: Option<SpeedLaw> = None;
    let mut final_from: Option<(f64, f64)> = None;
    let mut bug_set: Option<f64> = None;
    let mut last = std::time::Instant::now();
    let mut tick_n = 0u64;
    while ACTIVE.load(Ordering::SeqCst) {
        tokio::time::sleep(TICK).await;
        if crate::xplane::sim_frozen(&xp).await {
            last = std::time::Instant::now();
            continue;
        }
        let Some(s) = read(&xp).await else { continue };
        let dt = last.elapsed().as_secs_f64();
        last = std::time::Instant::now();
        tick_n += 1;
        let here = geo.to_m(origin, (s.lat, s.lon));
        // Sequence: the fix is passed when inside the turn's lead, or when
        // it is behind us (a wide turn that never got close).
        while leg + 1 < pts.len() {
            let to = pts[leg].1;
            let next = pts[leg + 1].1;
            let turn = angle_diff(bearing(to, next), bearing(here, to));
            let lead = (turn_radius_m(s.gs.max(10.0))
                * (turn.abs().min(120.0) / 2.0).to_radians().tan())
            .max(300.0);
            let behind =
                angle_diff(bearing(here, to), s.psi).abs() > 100.0 && dist(here, to) < 3000.0;
            if dist(here, to) < lead || behind {
                leg += 1;
                crate::blackbox::record(
                    "heli_route",
                    serde_json::json!({ "event": "fix", "passed": pts[leg - 1].0, "next": pts[leg].0 }),
                );
            } else {
                break;
            }
        }
        let (ident, to) = (&pts[leg].0, pts[leg].1);
        let track = bearing(here, to);
        let to_pad = dist(here, pad);
        let variation = angle_diff(s.psi, s.mag);

        match phase {
            Phase::Depart => {
                let running = hover::ACTIVE.load(Ordering::SeqCst);
                let ours = matches!(hover::goal(), Some(Goal::Fly { .. })) || tick_n == 1;
                if !running || !ours {
                    record_end(&voice, "ended_outside", "depart");
                    return false;
                }
                // Level until flying, then up to the cruise height.
                let height = if s.gs < hover::DEPART_CLIMB_MPS {
                    s.agl_ft.max(FINAL_HOVER_FT)
                } else {
                    depart_agl_ft
                };
                hover::retarget(Goal::Fly {
                    height_ft: height,
                    speed_mps: DEPART_MPS,
                    track_deg: track,
                });
                if s.agl_ft > depart_agl_ft - 60.0 && s.gs > DEPART_MPS - 2.0 {
                    // The hand-over: the loop let go without the autopilot
                    // going off, then the cruise modes on what we are doing.
                    hover::release();
                    for _ in 0..20 {
                        if !hover::ACTIVE.load(Ordering::SeqCst) {
                            break;
                        }
                        tokio::time::sleep(Duration::from_millis(100)).await;
                    }
                    let c = xp
                        .get_float_array(&collective)
                        .await
                        .ok()
                        .and_then(|v| v.first().copied())
                        .unwrap_or(0.45);
                    let _ = xp
                        .set_float(
                            "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot",
                            (track - variation).rem_euclid(360.0).round(),
                        )
                        .await;
                    let ok = hover::cruise_modes(&xp).await;
                    let _ = xp
                        .set_float("sim/cockpit2/autopilot/altitude_dial_ft", cruise_alt_ft)
                        .await;
                    crate::blackbox::record(
                        "heli_route",
                        serde_json::json!({ "event": "handover_cruise", "ok": ok, "collective": c, "ias": s.ias, "agl": s.agl_ft }),
                    );
                    if !ok {
                        say(
                            &voice,
                            Priority::Warning,
                            "The autopilot won't take heading and altitude. Your controls.",
                        );
                        return false;
                    }
                    law = Some(SpeedLaw::from(c, s.ias));
                    phase = Phase::Cruise;
                    say(
                        &voice,
                        Priority::Readback,
                        format!("Autopilot has it. Accelerating to {CRUISE_KTS:.0} knots, {ident} next."),
                    );
                }
            }
            Phase::Cruise | Phase::Slow => {
                // The cruise modes are the autopilot's to keep: a mode that
                // dropped is put back, an autopilot turned off is the pilot's.
                if s.ap < 0.5 {
                    record_end(&voice, "ap_off", "cruise");
                    say(
                        &voice,
                        Priority::Caution,
                        "Autopilot's off. Route stopped, your controls.",
                    );
                    return false;
                }
                if s.theta.abs() > 20.0 || s.phi.abs() > 35.0 {
                    record_end(&voice, "attitude", "cruise");
                    let _ = hover::cruise_modes(&xp).await;
                    say(
                        &voice,
                        Priority::Warning,
                        "The attitude's past the limits. Route stopped, autopilot on heading and altitude, your controls.",
                    );
                    return false;
                }
                // The bug moves only when the course has moved: rewritten
                // every tick and rounded, it flipped a degree each way as the
                // bearing crossed a half, the heading hold chased every flip
                // and the pilot saw the yaw jitter (EDDV-ETHC, 2026-09-23).
                let want_bug = (track - variation).rem_euclid(360.0);
                let bug = match bug_set {
                    Some(b) if angle_diff(want_bug, b).abs() < BUG_HYSTERESIS_DEG => b,
                    _ => {
                        let b = want_bug.round().rem_euclid(360.0);
                        let _ = xp
                            .set_float("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot", b)
                            .await;
                        bug_set = Some(b);
                        b
                    }
                };
                // Slowing is keyed on the distance still to FLY, not on the
                // last leg: the fix before the pad can lie past it.
                let pts_m: Vec<(f64, f64)> = pts.iter().map(|p| p.1).collect();
                if phase == Phase::Cruise
                    && remaining_m(here, &pts_m, leg) < slow_start_m(CRUISE_KTS, depart_agl_ft)
                {
                    phase = Phase::Slow;
                    crate::blackbox::record(
                        "heli_route",
                        serde_json::json!({ "event": "slow", "to_pad_m": to_pad, "ias": s.ias }),
                    );
                    say(
                        &voice,
                        Priority::Readback,
                        format!("Slowing for {}.", route.destination),
                    );
                    if let Some(khz) = route.dest_tower_khz {
                        call_inbound(&xp, khz).await;
                    }
                }
                let want = if phase == Phase::Slow {
                    HANDOVER_KTS
                } else {
                    CRUISE_KTS
                };
                if let Some(l) = law.as_mut() {
                    let c = l.step(want, s.ias, dt);
                    let _ = xp.set_float_index(&collective, 0, c).await;
                    if tick_n % 4 == 0 {
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({ "event": "tick", "phase": format!("{phase:?}"), "fix": ident, "d_fix_m": dist(here, to), "to_pad_m": to_pad, "ias": s.ias, "target": l.target(), "collective": c, "agl": s.agl_ft, "bug": bug, "hdg": s.mag, "theta": s.theta, "phi": s.phi }),
                        );
                    }
                }
                // Slowed and level: the hover loop takes the aircraft at the
                // velocity it has, and flies the final.
                if phase == Phase::Slow
                    && s.ias < HANDOVER_KTS + 2.0
                    && s.gs < hover::TAKEOVER_MAX_MPS
                {
                    hover::request(
                        base_url.clone(),
                        voice.clone(),
                        collective.clone(),
                        Goal::Fly {
                            height_ft: s.agl_ft,
                            speed_mps: s.gs,
                            track_deg: bearing(here, pad),
                        },
                    );
                    final_from = Some((s.agl_ft, s.gs));
                    phase = Phase::Arrive;
                    crate::blackbox::record(
                        "heli_route",
                        serde_json::json!({ "event": "handover_hover", "to_pad_m": to_pad, "ias": s.ias, "gs": s.gs, "agl": s.agl_ft }),
                    );
                    say(&voice, Priority::Readback, "Final for the pad.");
                    // The loop's own start takes a second; let it.
                    tokio::time::sleep(Duration::from_millis(1500)).await;
                }
            }
            Phase::Arrive => {
                let running = hover::ACTIVE.load(Ordering::SeqCst);
                let ours = matches!(hover::goal(), Some(Goal::Fly { .. }));
                if !running || !ours {
                    record_end(&voice, "ended_outside", "arrive");
                    return false;
                }
                let (from_ft, from_mps) = final_from.unwrap_or((depart_agl_ft, DEPART_MPS));
                let (speed, height) = final_command(to_pad, from_ft, from_mps);
                if to_pad < ARRIVED_M || speed < 1.0 {
                    hover::retarget(Goal::Land);
                    crate::blackbox::record(
                        "heli_route",
                        serde_json::json!({ "event": "over_pad", "d_m": to_pad }),
                    );
                    say(
                        &voice,
                        Priority::Readback,
                        format!("Over the pad at {}. Setting it down.", route.destination),
                    );
                    // Down means the hover loop's own landing finished.
                    for _ in 0..600 {
                        tokio::time::sleep(TICK).await;
                        if !hover::ACTIVE.load(Ordering::SeqCst) {
                            break;
                        }
                    }
                    let down = xp
                        .get_float("sim/flightmodel/failures/onground_any")
                        .await
                        .is_ok_and(|g| g > 0.5);
                    crate::blackbox::record(
                        "heli_route",
                        serde_json::json!({ "event": "arrived", "on_ground": down }),
                    );
                    return down;
                }
                hover::retarget(Goal::Fly {
                    height_ft: height,
                    speed_mps: speed,
                    track_deg: bearing(here, pad),
                });
            }
        }
        DETAIL.set(match phase {
            Phase::Arrive => format!("final {:.1} nm", to_pad / 1852.0),
            _ => format!("{} {:.1} nm", ident, dist(here, to) / 1852.0),
        });
    }
    crate::blackbox::record("heli_route", serde_json::json!({ "event": "stood_down" }));
    false
}

fn record_end(_voice: &Option<Voice>, why: &str, phase: &str) {
    crate::blackbox::record(
        "heli_route",
        serde_json::json!({ "event": "ended", "why": why, "phase": phase }),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The speed law under altitude hold: collective above the level figure
    /// is acceleration, through a lag. It reaches cruise and comes back to
    /// the hand-over speed without overshooting past a few knots, across
    /// plant gains nobody has measured, and never slews the lever fast.
    #[test]
    fn the_speed_law_accelerates_and_slows_without_ringing() {
        // A power curve past the knee: rising again toward cruise.
        let level = |kts: f64| {
            let s = kts * 0.5144;
            if s < 37.0 {
                hover::level_collective(s)
            } else {
                0.42 + (s - 37.0) * 0.006
            }
        };
        let mut bad = Vec::new();
        for gain in [8.0, 20.0, 40.0] {
            let mut law = SpeedLaw::from(0.45, 38.0);
            let (mut v, mut acc, mut last_c) = (38.0_f64, 0.0_f64, 0.45);
            let dt = 0.5;
            let mut over: f64 = 0.0;
            let mut under: f64 = 0.0;
            for i in 0..1200 {
                let want = if i < 600 { CRUISE_KTS } else { HANDOVER_KTS };
                let c = law.step(want, v, dt);
                assert!((c - last_c).abs() <= COLLECTIVE_SLEW * dt + 1e-9);
                last_c = c;
                // kt/s per unit collective above level, with drag's damping.
                let want_acc = gain * (c - level(v));
                acc += (want_acc - acc) * (dt / 2.0);
                v += acc * dt;
                if (300..600).contains(&i) {
                    over = over.max(v - CRUISE_KTS);
                }
                if i > 900 {
                    under = under.max(HANDOVER_KTS - v);
                }
            }
            let end = (v - HANDOVER_KTS).abs();
            if over > 6.0 || under > 4.0 || end > 1.5 {
                bad.push(format!(
                    "gain {gain}: over {over:.1} under {under:.1} end {end:.1}"
                ));
            }
        }
        assert!(bad.is_empty(), "{bad:#?}");
    }

    #[test]
    fn the_final_starts_at_its_height_and_stops_on_the_pad() {
        assert_eq!(final_command(10_000.0, 1000.0, 18.0), (18.0, 1000.0));
        let (s, h) = final_command(0.0, 1000.0, 18.0);
        assert_eq!((s, h), (0.0, FINAL_HOVER_FT));
        // The slow-down begins far enough out for the decel and the path.
        let d = slow_start_m(CRUISE_KTS, CRUISE_AGL_FT);
        assert!((6000.0..12000.0).contains(&d), "{d}");
    }

    /// CEL lies past ETHC: a terminal fix is dropped, a far fix kept, and
    /// the distance to fly counts every leg still ahead.
    #[test]
    fn a_fix_past_the_pad_is_dropped_and_the_slowdown_counts_the_legs() {
        let p = |n: &str, la: f64, lo: f64| (n.to_string(), la, lo);
        let route = vec![
            p("DV253", 52.50, 9.85),
            p("CEL", 52.62, 10.06),
            p("ETHC H20", 52.594, 10.028),
        ];
        let out = drop_terminal_fixes(route);
        assert_eq!(
            out.iter().map(|x| x.0.as_str()).collect::<Vec<_>>(),
            ["DV253", "ETHC H20"]
        );
        let far = vec![
            p("A", 52.0, 9.0),
            p("B", 52.3, 9.5),
            p("PAD", 52.594, 10.028),
        ];
        assert_eq!(drop_terminal_fixes(far.clone()).len(), 3);
        let pts = [(0.0, 1000.0), (1000.0, 1000.0), (1000.0, 0.0)];
        assert!((remaining_m((0.0, 0.0), &pts, 0) - 3000.0).abs() < 1e-6);
        assert!((remaining_m((1000.0, 500.0), &pts, 2) - 500.0).abs() < 1e-6);
    }

    /// The route pursuit: the aim is ahead ALONG the route (round a corner,
    /// not across it), the distance left runs to the end, and a point off
    /// the route still projects onto it.
    #[test]
    fn a_cleared_route_is_pursued_along_its_taxiways() {
        let path = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)];
        let (aim, left) = pursue(&path, (90.0, 0.0), 25.0).unwrap();
        assert!(
            (aim.0 - 100.0).abs() < 1e-6 && (aim.1 - 15.0).abs() < 1e-6,
            "{aim:?}"
        );
        assert!((left - 110.0).abs() < 1e-6, "{left}");
        let (aim, _) = pursue(&path, (50.0, 8.0), 25.0).unwrap();
        assert!((aim.0 - 75.0).abs() < 1e-6 && aim.1.abs() < 1e-6);
        let (_, left) = pursue(&path, (100.0, 100.0), 25.0).unwrap();
        assert!(left < 1e-6);
    }

    #[test]
    fn a_hover_taxi_slows_for_a_corner() {
        let path = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)];
        assert!((corner_factor(&path, (20.0, 0.0)) - 1.0).abs() < 1e-6);
        let f = corner_factor(&path, (88.0, 0.0));
        assert!(f < 0.5, "{f}");
    }

    #[test]
    fn the_cruise_clears_the_highest_ground_on_the_route() {
        assert_eq!(cruise_altitude_ft(187.0, Some(256.0)), 1300.0);
        assert_eq!(cruise_altitude_ft(187.0, Some(1500.0)), 2500.0);
        assert_eq!(cruise_altitude_ft(187.0, None), 1200.0);
        let line = [(52.0, 9.0), (52.0, 9.1)];
        let s = sample_legs(&line, 1000.0);
        assert!(s.len() >= 7 && s.len() <= 9, "{}", s.len());
        assert_eq!(*s.last().unwrap(), (52.0, 9.1));
    }

    #[test]
    fn the_pad_is_the_one_nearest_the_inbound_fix() {
        let pads = vec![
            (52.5954, 10.0358, "H1".to_string()),
            (52.5939, 10.0276, "H18".to_string()),
        ];
        // Inbound from the west.
        let p = pick_pad(&pads, (52.59, 9.9)).unwrap();
        assert_eq!(p.2, "H18");
    }

    /// The real ETHC helipads, when the install is here.
    #[test]
    #[ignore]
    fn real_install_ethc_helipads() {
        let pads = helipads(
            "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat",
            "ETHC",
        );
        assert!(pads.len() >= 18, "{}", pads.len());
    }
}
