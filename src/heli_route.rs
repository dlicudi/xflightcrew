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
/// The cruise speed when the airframe declares none (see [`cruise_kts`]).
pub const CRUISE_KTS: f64 = 120.0;

/// The cruise speed: the airframe's own (`reference.cruise_ias_kts`, 140 kt
/// on the S-76, whose Vno is 139.5), never past its speed ceiling (Vne less
/// 15), [`CRUISE_KTS`] when it declares none. A flat 120 flew the S-76 20 kt
/// under its book cruise with the collective at 0.72 of a 0.85 cap
/// (LILO-LILB, 2026-09-24). Where the power is not there the cruise settles
/// at what it makes ([`power_limited_kts`]).
pub fn cruise_kts() -> f64 {
    let own = crate::aircraft::live_profile()
        .and_then(|p| p.reference.as_ref().and_then(|r| r.cruise_ias_kts))
        .filter(|k| *k >= 60.0);
    let k = own.unwrap_or(CRUISE_KTS);
    crate::aircraft::speed_ceiling_kts().map_or(k, |c| k.min(c))
}

/// The collective at its cap this long in a level cruise is a speed the
/// power cannot make.
pub const POWER_LIMIT_S: f64 = 15.0;

/// The cruise speed once the power has run out: what is being made, down to
/// the 5 kt, never under the approach speed. Pure.
pub fn power_limited_kts(ias_kts: f64) -> f64 {
    ((ias_kts / 5.0).floor() * 5.0).max(APPROACH_KTS)
}

/// Whether the speed law's collective is at its cap.
pub fn collective_capped(c: f64) -> bool {
    c >= COLLECTIVE_MAX - 0.005
}
/// The hover mode's side of the hand-over, both ways: level, inside the
/// 40 kt the mode has been flown at.
pub const HANDOVER_KTS: f64 = 38.0;
const DEPART_MPS: f64 = 18.0;
/// The fast departure: the hover loop to [`FAST_HANDOVER_AGL_FT`], then the
/// autopilot climbs at [`CLIMB_FPM`] with the collective on [`CLIMB_KTS`]
/// (about Vy). The whole climb at the hover mode's 35 kt took 11 minutes to
/// 4,800 ft over Samedan (2026-09-24). It is SHALLOWER over the ground than
/// the slow climb (about 560 ft a mile against 860), so the arm picks it only
/// where the probed ground allows (`climb_conflict`).
pub const FAST_HANDOVER_AGL_FT: f64 = 500.0;
pub const CLIMB_KTS: f64 = 75.0;
pub const CLIMB_FPM: f64 = 800.0;
/// Below this in the fast climb the power is not there: the climb eases to
/// [`CLIMB_EASED_FPM`] rather than trade the speed away.
const CLIMB_MIN_KTS: f64 = 55.0;
const CLIMB_EASED_FPM: f64 = 400.0;
/// The final: its path, its deceleration, the hover it ends in.
const GLIDE_DEG: f64 = 6.0;
const FINAL_DECEL_MPS2: f64 = 0.3;
const FINAL_HOVER_FT: f64 = 30.0;
const ARRIVED_M: f64 = 12.0;
/// The approach: down from the cruise at speed, under the autopilot's
/// vertical-speed mode with the collective on [`APPROACH_KTS`], to a gate
/// [`GATE_FT`] over the pad, then the slow-down level at the gate and the
/// hover loop's final from there. Until 2026-09-23 the whole descent was
/// the hover loop's, at 38 kt or less from the cruise height: 5.5 minutes
/// from a 1,000 ft cruise, 8.3 into ETND from 2,000 ft, 7 of them on a final
/// that a 17 kt headwind held to 21 kt over the ground.
pub const APPROACH_KTS: f64 = 70.0;
/// The approach path, degrees: about 800 fpm at 70 kt.
const APPROACH_DEG: f64 = 6.5;
/// The gate: this high over the pad, where the final's own path begins.
pub const GATE_FT: f64 = 500.0;
/// The gate clears the highest ground still to be crossed by this much.
const APPROACH_CLEAR_FT: f64 = 400.0;
/// The steepest the approach asks of the vertical-speed mode.
const APPROACH_MAX_SINK_FPM: f64 = 1200.0;
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
    /// The collective's floor: [`COLLECTIVE_MIN`] in the cruise, higher on
    /// the approach and the slow-down ([`APPROACH_COLLECTIVE_MIN`]).
    floor: f64,
    cmd: f64,
    trim: f64,
    target_kts: f64,
}

const SPEED_KP_PER_KT: f64 = 0.008;
const SPEED_KI_PER_KT_S: f64 = 0.0005;
const COLLECTIVE_SLEW: f64 = 0.03;
const COLLECTIVE_MIN: f64 = 0.25;
const COLLECTIVE_MAX: f64 = 0.85;
/// The collective's floor on the approach and the slow-down. At 0.25 the
/// speed law cut it to nothing for a gate reached at 86 kt, the helicopter
/// sank, the vertical-speed mode held the level with 15 degrees of nose-up,
/// the speed fell to 25 kt, the law then drove the collective to 0.65 and it
/// climbed: a growing cycle that tripped the 20 degree limit over a ridge
/// (LILO-LILB, 2026-09-24). Near the power curve's 0.42 the pitch has less
/// to hold and the cycle has nothing to swing on.
const APPROACH_COLLECTIVE_MIN: f64 = 0.35;
const ACCEL_KTS_S: f64 = 1.5;
const DECEL_KTS_S: f64 = 1.0;

impl SpeedLaw {
    pub fn from(collective: f64, ias_kts: f64) -> Self {
        Self {
            floor: COLLECTIVE_MIN,
            cmd: collective,
            trim: collective,
            target_kts: ias_kts,
        }
    }
    pub fn target(&self) -> f64 {
        self.target_kts
    }
    pub fn cmd(&self) -> f64 {
        self.cmd
    }
    pub fn set_floor(&mut self, floor: f64) {
        self.floor = floor.clamp(COLLECTIVE_MIN, COLLECTIVE_MAX);
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
        self.trim = (self.trim + err * SPEED_KI_PER_KT_S * dt).clamp(self.floor, COLLECTIVE_MAX);
        let want = (self.trim + err * SPEED_KP_PER_KT).clamp(self.floor, COLLECTIVE_MAX);
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

/// Half-width of the corridor the cruise is sized over, m.
pub const CORRIDOR_HALF_M: f64 = 1000.0;

/// [`sample_legs`] plus a point `half_m` either side of the track at each
/// sample: the ground a drift or a cut corner can reach. The track alone
/// sized LSZS-LILO's cruise at 9,100 ft while the Forcola di Livigno wall
/// stood at 9,069 ft 750 m off it and 9,528 ft at a kilometre (probed
/// 2026-09-24): a valley route has its hazards beside the line, not on it.
pub fn sample_corridor(line: &[(f64, f64)], step_m: f64, half_m: f64) -> Vec<(f64, f64)> {
    let track = sample_legs(line, step_m);
    let mut out = track.clone();
    for w in track.windows(2) {
        let brg = crate::nav::bearing_deg(w[0].0, w[0].1, w[1].0, w[1].1);
        for side in [-90.0, 90.0] {
            out.push(crate::nav::project_latlon(w[0].0, w[0].1, brg + side, half_m / 1852.0));
        }
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

/// The final's length on its own path from `agl_ft`, m.
pub fn final_len_m(agl_ft: f64) -> f64 {
    (agl_ft - FINAL_HOVER_FT).max(0.0) / 3.28084 / GLIDE_DEG.to_radians().tan()
}

/// Distance flown slowing from `from_kts` to `to_kts` at the law's rate, m.
fn decel_m(from_kts: f64, to_kts: f64) -> f64 {
    let secs = (from_kts - to_kts).max(0.0) / DECEL_KTS_S;
    secs * (from_kts + to_kts) / 2.0 * 0.5144
}

/// The gate's height over the pad: [`GATE_FT`], or higher where the ground
/// still to be crossed (`floor_ft`, its highest point, same feet as `pad_ft`)
/// needs [`APPROACH_CLEAR_FT`] over it.
pub fn gate_agl_ft(pad_ft: f64, floor_ft: Option<f64>) -> f64 {
    floor_ft.map_or(GATE_FT, |f| (f + APPROACH_CLEAR_FT - pad_ft).max(GATE_FT))
}

/// Where the slow-down to the hand-over speed starts on an approach, m before
/// the pad: level at the gate, it ends where the final's path from the gate
/// begins, so the final starts ON its path, never above it (from above,
/// `final_command` dives to it: 2,000 ft into ETND, 2026-09-23).
pub fn approach_slow_m(gate_agl: f64) -> f64 {
    final_len_m(gate_agl) + decel_m(APPROACH_KTS, HANDOVER_KTS)
}

/// The approach's target altitude (ft, the dial's) at `d_m` from the pad: the
/// path down to the gate where the slow-down starts, never above the cruise.
pub fn approach_target_ft(d_m: f64, pad_ft: f64, gate_agl: f64, cruise_ft: f64) -> f64 {
    let gate = pad_ft + gate_agl;
    let path = gate
        + (d_m - approach_slow_m(gate_agl)).max(0.0) * APPROACH_DEG.to_radians().tan() * 3.28084;
    path.min(cruise_ft.max(gate))
}

/// Where the approach starts, m before the pad: the slow-down from the cruise
/// speed to the approach speed, then the path from the cruise height.
pub fn approach_start_m(cruise_ft: f64, pad_ft: f64, gate_agl: f64, cruise_kts: f64) -> f64 {
    let above = (cruise_ft - pad_ft - gate_agl).max(0.0);
    let path_m = above / 3.28084 / APPROACH_DEG.to_radians().tan();
    approach_slow_m(gate_agl) + path_m + decel_m(cruise_kts, APPROACH_KTS)
}

/// The vertical-speed dial on the approach: the target's own descent rate
/// (`target_ft` now against `ahead_ft`, the target 10 s down the track),
/// fed forward, plus 2 fpm a foot toward it. A P term alone rides a falling
/// path ~120 ft high. Capped at [`APPROACH_MAX_SINK_FPM`] down and 500 fpm
/// up: under the terrain floor it climbs.
pub fn approach_vs_fpm(alt_ft: f64, target_ft: f64, ahead_ft: f64) -> f64 {
    let rate = (ahead_ft - target_ft) * 6.0; // ft per 10 s, as fpm
    let want = rate + 2.0 * (target_ft - alt_ft);
    (want / 50.0)
        .round()
        .clamp(-APPROACH_MAX_SINK_FPM / 50.0, 10.0)
        * 50.0
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

/// The departure climb, as the hover loop flies it: level while it
/// accelerates to [`hover::DEPART_CLIMB_MPS`], then [`DEPART_CLIMB_FPM`] at
/// [`DEPART_MPS`] (the steeper climb at the slower speeds in between is
/// margin). Feet over the field `d_nm` along the track.
pub fn depart_climb_ft(d_nm: f64, fast: bool) -> f64 {
    let level_nm = hover::DEPART_CLIMB_MPS.powi(2) / (2.0 * DEPART_ACCEL_MPS2) / 1852.0;
    let slow_ft_per_nm = DEPART_CLIMB_FPM / (DEPART_MPS * 1.943_84 / 60.0);
    let d = (d_nm - level_nm).max(0.0);
    let hover_nm = FAST_HANDOVER_AGL_FT / slow_ft_per_nm;
    if !fast || d <= hover_nm {
        return d * slow_ft_per_nm;
    }
    // Planned at 15 % over the climb speed for true airspeed at height:
    // a faster true airspeed is a shallower climb.
    FAST_HANDOVER_AGL_FT + (d - hover_nm) * CLIMB_FPM / (CLIMB_KTS * 1.15 / 60.0)
}
/// The hover loop's speed ramp and departure climb rate (mirrors).
const DEPART_ACCEL_MPS2: f64 = 0.4;
const DEPART_CLIMB_FPM: f64 = 500.0;
/// How far out the departure climb is checked against the ground, nm: past
/// this it has gained more than any ridge in Europe asks.
pub const CLIMB_CHECK_NM: f64 = 15.0;
/// The clearance the departure climb must keep over probed ground, ft.
pub const CLIMB_MARGIN_FT: f64 = 500.0;

/// Probe stations along the first `max_nm` of `line`: (nm along, lat, lon),
/// on the track and `half_m` either side.
pub fn climb_stations(line: &[(f64, f64)], max_nm: f64, step_m: f64, half_m: f64) -> Vec<(f64, f64, f64)> {
    let mut out = Vec::new();
    let mut along = 0.0;
    for w in line.windows(2) {
        let seg = crate::nav::haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1);
        let brg = crate::nav::bearing_deg(w[0].0, w[0].1, w[1].0, w[1].1);
        let n = ((seg * 1852.0 / step_m).ceil() as usize).max(1);
        for i in 0..n {
            let d = seg * i as f64 / n as f64;
            if along + d > max_nm {
                return out;
            }
            let (la, lo) = crate::nav::project_latlon(w[0].0, w[0].1, brg, d);
            out.push((along + d, la, lo));
            for side in [-90.0, 90.0] {
                let (a, b) = crate::nav::project_latlon(la, lo, brg + side, half_m / 1852.0);
                out.push((along + d, a, b));
            }
        }
        along += seg;
    }
    out
}

/// The worst place the departure climb comes within [`CLIMB_MARGIN_FT`] of
/// the probed ground: (nm along, ground ft, climb ft). `ground` is (nm along,
/// ft MSL); `field_ft` where the climb starts. LILO-LILB (2026-09-24): the
/// direct line leaves the Valtellina over a 7,728 ft ridge 4.3 nm out, and
/// the climb would have been at about 4,600 there. The cruise was sized over
/// the ridge; the climb to it never was.
/// The margin is half the height the climb has gained, up to
/// [`CLIMB_MARGIN_FT`]: a flat 500 ft refused every departure over a level
/// valley floor, where 0.3 nm out the climb has only gained 220 ft (the LILO
/// valley route, probed the same evening).
pub fn climb_conflict(field_ft: f64, ground: &[(f64, f64)], fast: bool) -> Option<(f64, f64, f64)> {
    ground
        .iter()
        .filter(|(d, _)| *d > 0.3)
        .map(|&(d, g)| (d, g, field_ft + depart_climb_ft(d, fast)))
        .filter(|(_, g, a)| *a < g + (0.5 * (a - field_ft)).min(CLIMB_MARGIN_FT))
        .min_by(|x, y| (x.2 - x.1).total_cmp(&(y.2 - y.1)))
}

/// Where a helicopter lands at a field with no helipad: on the hard runway
/// end nearest the arrival, [`RUNWAY_SPOT_M`] in from the threshold (never
/// the reference point, which at LILB is 120 m off the strip in whatever is
/// there). (lat, lon, name).
pub fn runway_spot(apt: &crate::nav::AirportInfo, inbound: (f64, f64)) -> Option<(f64, f64, String)> {
    let mut best: Option<(f64, (f64, f64, String))> = None;
    for r in apt.runways.iter().filter(|r| r.is_hard()) {
        let len_nm = crate::nav::haversine_nm(r.lat_a, r.lon_a, r.lat_b, r.lon_b);
        let into = (RUNWAY_SPOT_M / 1852.0).min(len_nm / 2.0);
        for (thr, far, id) in [
            ((r.lat_a, r.lon_a), (r.lat_b, r.lon_b), &r.ident_a),
            ((r.lat_b, r.lon_b), (r.lat_a, r.lon_a), &r.ident_b),
        ] {
            let d = crate::nav::haversine_nm(inbound.0, inbound.1, thr.0, thr.1);
            if best.as_ref().is_none_or(|b| d < b.0) {
                let brg = crate::nav::bearing_deg(thr.0, thr.1, far.0, far.1);
                let (la, lo) = crate::nav::project_latlon(thr.0, thr.1, brg, into);
                best = Some((d, (la, lo, format!("runway {id}"))));
            }
        }
    }
    best.map(|b| b.1)
}
const RUNWAY_SPOT_M: f64 = 150.0;


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
        let prev = &points[points.len() - 3];
        let geo = Geo::new(pad.1);
        let d = dist((0.0, 0.0), geo.to_m((pad.1, pad.2), (fix.1, fix.2)));
        // Only a fix at or PAST the pad (farther from the leg before it than
        // the pad is): one short of it is the pilot's way in. Dropping
        // LILO-LILB's lake point sent the last leg over the hills east of
        // Como, and the approach met 3,870 ft of ridge (2026-09-24).
        let past = dist((0.0, 0.0), geo.to_m((prev.1, prev.2), (fix.1, fix.2)))
            >= dist((0.0, 0.0), geo.to_m((prev.1, prev.2), (pad.1, pad.2)));
        if d < TERMINAL_FIX_M && past {
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
/// The warning before a hover taxi lifts a helicopter off the ground, s.
const LIFT_WARNING_S: f64 = 5.0;
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
            let name = if via.trim().is_empty() {
                "the stand".to_string()
            } else {
                format!("the stand via {via}")
            };
            (path.clone(), name, ROUTE_TAXI_FT)
        }
    };
    if path_ll.is_empty() {
        return;
    }
    // A helicopter on the ground is never lifted without warning: said,
    // then [`LIFT_WARNING_S`] for "my controls" to keep it where it is.
    say(
        &voice,
        Priority::Advisory,
        format!("Hover taxi to {name}. Lifting in five seconds. Say my controls to stay here."),
    );
    let wait = std::time::Instant::now();
    while wait.elapsed().as_secs_f64() < LIFT_WARNING_S {
        tokio::time::sleep(TICK).await;
        if !ACTIVE.load(Ordering::SeqCst) {
            crate::blackbox::record("heli_route", serde_json::json!({ "event": "park_declined" }));
            return;
        }
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
    /// The last point is a published helipad: the flight ends there, shut
    /// down, and never lifts again to taxi. A runway spot is vacated.
    pub on_pad: bool,
    /// The departure climbs fast (see [`FAST_HANDOVER_AGL_FT`]); the arm
    /// sets it only where the probed ground allows.
    pub fast_climb: bool,
    pub destination: String,
    /// The destination's staffed tower (apt.dat), for the inbound call.
    pub dest_tower_khz: Option<i64>,
}

/// The FMS's copy of the route (`xcopilot/fms/route`, then a load), so the
/// GPS and the panel's Route tab show the plan the helicopter is flying: the
/// pad goes in under the destination's ICAO, which is what fills the CDU's
/// DEST. Best effort, like the follower's.
async fn publish_fms(xp: &XPlaneClient, route: &Route, cruise_alt_ft: f64) {
    let n = route.points.len();
    let wps: Vec<crate::simbrief::Waypoint> = route
        .points
        .iter()
        .enumerate()
        .map(|(i, (id, lat, lon))| crate::simbrief::Waypoint {
            ident: if i + 1 == n { route.destination.clone() } else { id.clone() },
            lat: *lat,
            lon: *lon,
            alt: if i == 0 || i + 1 == n { 0.0 } else { cruise_alt_ft },
        })
        .collect();
    let payload = crate::simbrief::fms_route_payload(&wps);
    let ok = xp.set_data("xcopilot/fms/route", payload.as_bytes()).await.is_ok()
        && xp.set_int("xcopilot/fms/load", 1).await.is_ok();
    crate::blackbox::record(
        "heli_route",
        serde_json::json!({ "event": "fms_loaded", "waypoints": wps.len(), "ok": ok }),
    );
}

/// The FMS's active entry to point `i` (the pad under the destination ICAO).
async fn fms_goto(xp: &XPlaneClient, route: &Route, i: usize) {
    let Some((id, lat, lon)) = route.points.get(i) else { return };
    let ident = if i + 1 == route.points.len() { &route.destination } else { id };
    let _ = xp
        .set_data(
            "xcopilot/fms/goto",
            crate::procedure::fms_goto_payload(ident, *lat, *lon).as_bytes(),
        )
        .await;
}

/// How long the departure call waits for a clearance before going anyway
/// (said): a tower that never answers must not strand a flight on the pad.
const DEPARTURE_WAIT_S: u64 = 60;

/// The departure call, the aeroplanes' own words: the zone clearance first
/// (`full_auto::get_zone_clearance`: squawk dialled, clearance read back),
/// then COM1 to the tower, "ready for departure", and a bounded wait for the
/// clearance. No simulated tower, no call. Until 2026-09-23 it went straight
/// to "ready for departure": cleared for takeoff, the S-76 left EDVW with no
/// zone clearance and no squawk.
async fn call_departure(xp: &XPlaneClient, voice: &Option<Voice>, config: &crate::config::Config) {
    use crate::tower::State::{LinedUp, TakeoffCleared};
    if !crate::mode::atc_enabled() {
        return;
    }
    let Some(khz) = crate::party_line::home_tower_khz() else {
        return;
    };
    let callsign = crate::circuit::crew_callsign(xp).await;
    DETAIL.set("waiting for the zone clearance");
    let ctx = crate::tools::ToolCtx {
        xplane: xp,
        config,
        voice: voice.as_ref(),
        status: None,
        reminders: None,
    };
    crate::full_auto::get_zone_clearance(xp, &ctx, voice, &callsign).await;
    crate::blackbox::record(
        "heli_route",
        serde_json::json!({
            "event": "zone_clearance",
            "squawk": crate::party_line::assigned_squawk(),
        }),
    );
    let _ = xp
        .set_float(crate::datarefs::COM1_FREQ_KHZ, khz as f64)
        .await;
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

/// The tower's after-takeoff handoff and the region's "radar services
/// terminated, contact X Tower", obeyed with the aeroplanes' own code
/// (`full_auto::obey_departure_handoff` / `obey_region_handoff`: readback on
/// the old frequency, squawk, retune, check-in). Off the control loop: the
/// readback holds the frequency for seconds and the collective law must keep
/// ticking. Until 2026-09-23 nothing took them: at EDVW-ETND the region's
/// handoff to Eimbeckhausen went unanswered, COM1 stayed on Hameln, and the
/// crew then called its own zone "without clearance" four times.
fn obey_handoffs(base_url: &str, voice: &Option<Voice>, climb_to_ft: f64) {
    let departure = crate::party_line::take_departure_handoff();
    let region = crate::party_line::take_region_handoff();
    if departure.is_none() && region.is_none() {
        return;
    }
    let (base, voice) = (base_url.to_string(), voice.clone());
    tokio::spawn(async move {
        let xp = XPlaneClient::new(base);
        let config = crate::settings::live();
        let ctx = crate::tools::ToolCtx {
            xplane: &xp,
            config: &config,
            voice: voice.as_ref(),
            status: None,
            reminders: None,
        };
        let callsign = crate::circuit::crew_callsign(&xp).await;
        if let Some((to, khz)) = departure {
            crate::blackbox::record(
                "heli_route",
                serde_json::json!({ "event": "departure_handoff", "to": &to, "khz": khz }),
            );
            crate::full_auto::obey_departure_handoff(&xp, &callsign, to, khz, climb_to_ft).await;
        }
        if let Some(h) = region {
            crate::blackbox::record(
                "heli_route",
                serde_json::json!({ "event": "region_handoff", "to": &h.to, "khz": h.khz }),
            );
            crate::full_auto::obey_region_handoff(&xp, &ctx, &voice, &callsign, h, true).await;
        }
    });
}

const ALT_HOLD_STATUS: &str = "sim/cockpit2/autopilot/altitude_hold_status";
const VVI_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";

/// The vertical-speed rate back to `level_ft` from `alt_ft`: 700 fpm each
/// way, less inside 700 ft so the capture is gentle.
pub fn recovery_vs_fpm(alt_ft: f64, level_ft: f64) -> f64 {
    (level_ft - alt_ft).clamp(-700.0, 700.0).round()
}

/// Back to the cruise level: the dial on it, vertical speed toward it,
/// altitude hold arming itself to capture (what caught the 4,200 ft climb
/// by hand, 2026-09-23).
async fn recover_level(xp: &XPlaneClient, alt_ft: f64, level_ft: f64) {
    let _ = xp
        .set_float("sim/cockpit2/autopilot/altitude_dial_ft", level_ft)
        .await;
    let vs = recovery_vs_fpm(alt_ft, level_ft);
    let _ = xp
        .set_float("sim/cockpit2/autopilot/vvi_dial_fpm", vs)
        .await;
    if xp.get_float(VVI_STATUS).await.is_ok_and(|v| v < 0.5) {
        let _ = xp.run_command("sim/autopilot/vertical_speed").await;
        tokio::time::sleep(Duration::from_millis(500)).await;
        // Engaging the mode syncs the dial to the rate of the moment.
        let _ = xp
            .set_float("sim/cockpit2/autopilot/vvi_dial_fpm", vs)
            .await;
    }
}

/// Down on the vertical-speed mode toward `dial_ft`: the dial first, the
/// rate, the mode if it is not on, the rate again (engaging the mode syncs
/// the dial to the rate of the moment). `recover_level`'s sequence.
async fn descend_on_vs(xp: &XPlaneClient, dial_ft: f64, vs_fpm: f64) {
    let _ = xp
        .set_float("sim/cockpit2/autopilot/altitude_dial_ft", dial_ft.round())
        .await;
    let _ = xp
        .set_float("sim/cockpit2/autopilot/vvi_dial_fpm", vs_fpm)
        .await;
    if xp.get_float(VVI_STATUS).await.is_ok_and(|v| v < 0.5) {
        let _ = xp.run_command("sim/autopilot/vertical_speed").await;
        tokio::time::sleep(Duration::from_millis(500)).await;
        let _ = xp
            .set_float("sim/cockpit2/autopilot/vvi_dial_fpm", vs_fpm)
            .await;
    }
}

/// How long the inbound call waits for the destination's tower to become the
/// station before going out regardless (full auto's own bound).
const INBOUND_SWAP_WAIT_S: u64 = 45;

/// The inbound call to the destination's tower, the aeroplanes' sequence:
/// COM1 on its frequency, that field ARMED as the station (you call it, you
/// get it) and pinned for the arrival, then a bounded wait for the station
/// to be it before the call goes out. Off the control loop (the wait is up
/// to 45 s). Until 2026-09-23 it transmitted at once: into ETND the call
/// went out at 19:20:45, the region made Diepholz the station at 19:21:00,
/// the call went into the void and Diepholz challenged the final twice.
fn call_inbound(base_url: &str, khz: i64) {
    if !crate::mode::atc_enabled() {
        return;
    }
    let base = base_url.to_string();
    tokio::spawn(async move {
        let xp = XPlaneClient::new(base);
        let _ = xp
            .set_float(crate::datarefs::COM1_FREQ_KHZ, khz as f64)
            .await;
        let (lat, lon, alt) = tokio::join!(
            xp.get_float(crate::datarefs::LATITUDE),
            xp.get_float(crate::datarefs::LONGITUDE),
            xp.get_float("sim/cockpit2/gauges/indicators/altitude_ft_pilot"),
        );
        if let (Ok(lat), Ok(lon), Ok(alt)) = (lat, lon, alt) {
            crate::station::arm_station_for_freq(khz, lat, lon, alt, std::time::Instant::now());
        }
        crate::station::pin_arrival_station(Some(khz));
        let mut swapped = false;
        for _ in 0..INBOUND_SWAP_WAIT_S * 2 {
            if crate::party_line::home_tower_khz() == Some(khz) {
                swapped = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
        let callsign = crate::circuit::crew_callsign(&xp).await;
        crate::circuit::transmit(crate::circuit::radio_call(&callsign, "inbound for landing"));
        crate::blackbox::record(
            "heli_route",
            serde_json::json!({ "event": "inbound_call", "khz": khz, "station_swapped": swapped }),
        );
    });
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
        let on_pad = route.on_pad;
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
        // Down on a helipad: that is where the flight ends, shut down. A
        // pad landing is never lifted again to taxi: at LILO the helicopter
        // rose off H1 unannounced for a stand and the pilot stopped it
        // (2026-09-24).
        if landed && on_pad {
            crate::blackbox::record("heli_route", serde_json::json!({ "event": "pad_shutdown" }));
            rt.block_on(crate::taxi_in::shut_down_at_stand(&xp, &config, &voice));
            return;
        }
        // Down on a runway: the aeroplanes' own taxi-in (the tower's
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
                true,
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
    Approach,
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
        call_departure(xp, voice, config).await;
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
    // probed along every leg at a kilometre's spacing, and a kilometre either
    // side of it (`sample_corridor`), before departure; the
    // departure field is only the floor of it. No terrain data (an old
    // plugin) keeps the departure-ground rule and says so in the log.
    let mut line: Vec<(f64, f64)> = vec![origin];
    line.extend(route.points.iter().map(|p| (p.1, p.2)));
    let samples = sample_corridor(&line, 1000.0, CORRIDOR_HALF_M);
    let terrain_ft = crate::terrain::max_elevation_ft(&xp, &samples).await;
    let cruise_alt_ft = cruise_altitude_ft(ground_ft, terrain_ft);
    // The departure climbs to it on the radio height over the field; altitude
    // hold then holds what it engages at.
    let depart_agl_ft = (cruise_alt_ft - ground_ft).max(CRUISE_AGL_FT);
    // The approach measures the pad and the ground ahead with the plugin's
    // terrain probe (true feet) and flies the altimeter's (the dial's):
    // the offset between them, taken once here. No probe, no approach: the
    // slow-down happens at the cruise height as it always did.
    let pad_ll = route.points.last().map(|p| (p.1, p.2));
    let (pad_true, elev_m) = tokio::join!(
        async {
            match pad_ll {
                Some(ll) => crate::terrain::max_elevation_ft(&xp, &[ll]).await,
                None => None,
            }
        },
        xp.get_float("sim/flightmodel/position/elevation"),
    );
    let ind_offset_ft = elev_m.ok().map(|m| alt0 - m * 3.28084);
    let pad_ft = match (pad_true, ind_offset_ft, terrain_ft) {
        (Some(p), Some(o), Some(_)) => Some(p + o),
        _ => None,
    };
    let mut gate: Option<(f64, f64)> = None;
    let mut level_ft = cruise_alt_ft;
    let mut vs_written: Option<f64> = None;
    crate::blackbox::record(
        "heli_route",
        serde_json::json!({ "event": "start", "points": route.points.iter().map(|p| &p.0).collect::<Vec<_>>(), "cruise_alt_ft": cruise_alt_ft, "terrain_ft": terrain_ft, "samples": samples.len() }),
    );

    publish_fms(&xp, &route, cruise_alt_ft).await;
    fms_goto(&xp, &route, 1).await;
    say(
        &voice,
        Priority::Readback,
        match terrain_ft {
            Some(_) => format!("Cruise {}.", crate::flightplan::spoken_feet(cruise_alt_ft)),
            None => format!(
                "Cruise {}. I couldn't probe the ground on the route.",
                crate::flightplan::spoken_feet(cruise_alt_ft)
            ),
        },
    );
    let mut phase = Phase::Depart;
    let mut climbing = false;
    let mut cruise_speed = cruise_kts();
    let mut capped_since: Option<std::time::Instant> = None;
    let mut climb_eased = false;
    let mut gate_leg: Option<usize> = None;
    let mut leg = 0usize;
    let mut law: Option<SpeedLaw> = None;
    let mut final_from: Option<(f64, f64)> = None;
    let mut bug_set: Option<f64> = None;
    let mut unheld_since: Option<std::time::Instant> = None;
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
                fms_goto(&xp, &route, leg).await;
            } else {
                break;
            }
        }
        let (ident, to) = (&pts[leg].0, pts[leg].1);
        let track = bearing(here, to);
        let to_pad = dist(here, pad);
        let variation = angle_diff(s.psi, s.mag);

        // The radio is worked whatever the phase: the handoffs arrive in the
        // climb (the tower's) and anywhere enroute (the region's).
        obey_handoffs(&base_url, &voice, cruise_alt_ft);
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
                let handover_agl = if route.fast_climb {
                    depart_agl_ft.min(FAST_HANDOVER_AGL_FT)
                } else {
                    depart_agl_ft
                };
                if s.agl_ft > handover_agl - 60.0 && s.gs > DEPART_MPS - 2.0 {
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
                    // Altitude hold engages at the level the climb reached (the
                    // departure climbed to the cruise level for exactly this);
                    // the dial is NOT moved afterwards: moving it turned the
                    // hold into "armed", nothing held the pitch, and the speed
                    // law's power climbed the S-76 at 1,200 fpm through 4,200
                    // ft over a 2,200 dial (ETHB-EDVW, 2026-09-23).
                    let ok = hover::cruise_modes(&xp).await;
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
                    // The fast departure goes on up under the autopilot: the
                    // cruise dial, vertical speed, altitude hold armed to
                    // capture it (`recover_level`'s sequence, flown).
                    let alt_now = xp
                        .get_float("sim/cockpit2/gauges/indicators/altitude_ft_pilot")
                        .await
                        .ok();
                    if route.fast_climb && alt_now.is_some_and(|a| a < cruise_alt_ft - 300.0) {
                        descend_on_vs(&xp, cruise_alt_ft, CLIMB_FPM).await;
                        climbing = true;
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({ "event": "climb", "to_ft": cruise_alt_ft, "vs": CLIMB_FPM, "kts": CLIMB_KTS }),
                        );
                        say(
                            &voice,
                            Priority::Readback,
                            format!(
                                "Autopilot has it. Climbing to {} at {CLIMB_KTS:.0} knots, {ident} next.",
                                crate::flightplan::spoken_feet(cruise_alt_ft)
                            ),
                        );
                    } else {
                        say(
                            &voice,
                            Priority::Readback,
                            format!("Autopilot has it. Accelerating to {cruise_speed:.0} knots, {ident} next."),
                        );
                    }
                }
            }
            Phase::Cruise | Phase::Approach | Phase::Slow => {
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
                let remaining = remaining_m(here, &pts_m, leg);
                if phase == Phase::Cruise {
                    match pad_ft {
                        Some(pad) if remaining < approach_start_m(cruise_alt_ft, pad, GATE_FT, cruise_speed) => {
                            // The gate clears the ground still to be crossed,
                            // probed now along what is left of the route; a
                            // failed probe falls back on the whole route's.
                            let mut rest = vec![(s.lat, s.lon)];
                            rest.extend(
                                route.points[leg.min(route.points.len())..]
                                    .iter()
                                    .map(|p| (p.1, p.2)),
                            );
                            let ahead =
                                crate::terrain::max_elevation_ft(&xp, &sample_corridor(&rest, 500.0, CORRIDOR_HALF_M))
                                    .await
                                    .or(terrain_ft);
                            let floor = ahead.map(|f| f + ind_offset_ft.unwrap_or(0.0));
                            let g = gate_agl_ft(pad, floor);
                            gate = Some((pad, g));
                            gate_leg = Some(leg);
                            level_ft = pad + g;
                            phase = Phase::Approach;
                            crate::blackbox::record(
                                "heli_route",
                                serde_json::json!({ "event": "approach", "to_pad_m": to_pad, "remaining_m": remaining, "pad_ft": pad, "floor_ft": floor, "gate_agl": g, "ias": s.ias }),
                            );
                            say(
                                &voice,
                                Priority::Readback,
                                format!("Starting down for {}.", route.destination),
                            );
                            if let Some(khz) = route.dest_tower_khz {
                                call_inbound(&base_url, khz);
                            }
                        }
                        None if remaining < slow_start_m(cruise_speed, depart_agl_ft) => {
                            phase = Phase::Slow;
                            crate::blackbox::record(
                                "heli_route",
                                serde_json::json!({ "event": "slow", "to_pad_m": to_pad, "ias": s.ias, "approach": false }),
                            );
                            say(
                                &voice,
                                Priority::Readback,
                                format!("Slowing for {}.", route.destination),
                            );
                            if let Some(khz) = route.dest_tower_khz {
                                call_inbound(&base_url, khz);
                            }
                        }
                        _ => {}
                    }
                }
                // Each fix passed on the approach: the ground still ahead is
                // probed again, and a gate held up for ground now behind us
                // comes down (LILO: 1,874 ft over the pad for a 2,426 ft bump
                // already passed, 5.3 km of final at hover speeds).
                if phase == Phase::Approach && gate_leg.is_some_and(|l| l != leg) {
                    gate_leg = Some(leg);
                    if let Some((pad, g)) = gate {
                        let mut rest = vec![(s.lat, s.lon)];
                        rest.extend(
                            route.points[leg.min(route.points.len())..]
                                .iter()
                                .map(|p| (p.1, p.2)),
                        );
                        if let Some(ahead) =
                            crate::terrain::max_elevation_ft(&xp, &sample_corridor(&rest, 500.0, CORRIDOR_HALF_M)).await
                        {
                            let ng = gate_agl_ft(pad, Some(ahead + ind_offset_ft.unwrap_or(0.0)));
                            if ng < g - 50.0 {
                                gate = Some((pad, ng));
                                level_ft = pad + ng;
                                crate::blackbox::record(
                                    "heli_route",
                                    serde_json::json!({ "event": "gate_lowered", "from": g, "to": ng, "leg": leg }),
                                );
                            }
                        }
                    }
                }
                if phase == Phase::Approach {
                    if let Some((_, g)) = gate {
                        if remaining < approach_slow_m(g) {
                            phase = Phase::Slow;
                            crate::blackbox::record(
                                "heli_route",
                                serde_json::json!({ "event": "slow", "to_pad_m": to_pad, "ias": s.ias, "agl": s.agl_ft, "approach": true }),
                            );
                            say(&voice, Priority::Readback, "Slowing for the pad.");
                        }
                    }
                }
                // The vertical is the autopilot's, and it must actually be
                // HELD: neither altitude hold nor vertical speed engaged for
                // 5 s is the level coming back on vertical speed to the
                // cruise dial, altitude hold armed to capture it.
                let (ah, vm, alt_now) = tokio::join!(
                    xp.get_float(ALT_HOLD_STATUS),
                    xp.get_float(VVI_STATUS),
                    xp.get_float("sim/cockpit2/gauges/indicators/altitude_ft_pilot"),
                );
                let (alt_hold, vvi_mode) = (ah.ok(), vm.ok());
                let unheld = alt_hold.is_some_and(|a| a < 1.5) && vvi_mode.is_some_and(|v| v < 1.5);
                if unheld {
                    let since = *unheld_since.get_or_insert_with(std::time::Instant::now);
                    if since.elapsed().as_secs_f64() > 5.0 {
                        if let Ok(alt) = alt_now {
                            recover_level(&xp, alt, level_ft).await;
                            crate::blackbox::record(
                                "heli_route",
                                serde_json::json!({ "event": "level_recovered", "alt": alt, "level_ft": level_ft, "alt_hold": alt_hold }),
                            );
                        }
                        unheld_since = None;
                    }
                } else {
                    unheld_since = None;
                }
                // The approach's vertical: down the path on the vertical-speed
                // mode, the dial on the gate so the autopilot's own capture
                // levels us there. The dial moves only with the mode's engage
                // (moved under altitude hold it leaves the pitch unheld).
                let mut vs_cmd: Option<f64> = None;
                let mut target_alt: Option<f64> = None;
                if let (Some((pad, g)), Ok(alt), Phase::Approach | Phase::Slow) =
                    (gate, alt_now.as_ref().copied(), phase)
                {
                    let target = approach_target_ft(remaining, pad, g, cruise_alt_ft);
                    let ahead = approach_target_ft(
                        (remaining - s.gs * 10.0).max(0.0),
                        pad,
                        g,
                        cruise_alt_ft,
                    );
                    let vs = approach_vs_fpm(alt, target, ahead);
                    let engaged = vvi_mode.is_some_and(|v| v > 1.5);
                    if !engaged && vs <= -100.0 && alt > pad + g + 60.0 {
                        descend_on_vs(&xp, pad + g, vs).await;
                        vs_written = Some(vs);
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({ "event": "descent_vs", "alt": alt, "target": target, "vs": vs, "dial": (pad + g).round() }),
                        );
                    } else if engaged && vs_written.is_none_or(|w| (w - vs).abs() >= 50.0) {
                        let _ = xp
                            .set_float("sim/cockpit2/autopilot/vvi_dial_fpm", vs)
                            .await;
                        vs_written = Some(vs);
                    }
                    vs_cmd = Some(vs);
                    target_alt = Some(target.round());
                }
                // The fast climb ends at the level (captured, or within
                // 150 ft); until then the collective holds the climb speed,
                // and a speed the power cannot keep eases the climb once.
                if climbing {
                    let captured = alt_hold.is_some_and(|a| a > 1.5)
                        || alt_now.as_ref().is_ok_and(|a| *a > cruise_alt_ft - 150.0);
                    if captured {
                        climbing = false;
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({ "event": "climb_done", "alt": alt_now.as_ref().ok() }),
                        );
                    } else if !climb_eased && s.ias < CLIMB_MIN_KTS && s.ias > 1.0 {
                        climb_eased = true;
                        let _ = xp
                            .set_float("sim/cockpit2/autopilot/vvi_dial_fpm", CLIMB_EASED_FPM)
                            .await;
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({ "event": "climb_eased", "ias": s.ias, "vs": CLIMB_EASED_FPM }),
                        );
                    }
                }
                let want = match phase {
                    Phase::Slow => HANDOVER_KTS,
                    Phase::Approach => APPROACH_KTS,
                    _ if climbing => CLIMB_KTS,
                    _ => cruise_speed,
                };
                // Level in the cruise with the collective pinned at its cap:
                // the speed the power makes becomes the cruise, said once.
                if phase == Phase::Cruise && !climbing {
                    let capped = law.as_ref().is_some_and(|l| collective_capped(l.cmd()));
                    match (capped, capped_since) {
                        (true, None) => capped_since = Some(std::time::Instant::now()),
                        (true, Some(t)) if t.elapsed().as_secs_f64() > POWER_LIMIT_S => {
                            let k = power_limited_kts(s.ias);
                            if k < cruise_speed {
                                cruise_speed = k;
                                crate::blackbox::record(
                                    "heli_route",
                                    serde_json::json!({ "event": "power_limited", "ias": s.ias, "cruise_kts": k }),
                                );
                                say(&voice, Priority::Advisory, format!("Power limited. Cruising at {k:.0} knots."));
                            }
                            capped_since = None;
                        }
                        (false, _) => capped_since = None,
                        _ => {}
                    }
                }
                if let Some(l) = law.as_mut() {
                    l.set_floor(if matches!(phase, Phase::Approach | Phase::Slow) {
                        APPROACH_COLLECTIVE_MIN
                    } else {
                        COLLECTIVE_MIN
                    });
                    let c = l.step(want, s.ias, dt);
                    let _ = xp.set_float_index(&collective, 0, c).await;
                    if tick_n % 4 == 0 {
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({ "event": "tick", "phase": format!("{phase:?}"), "fix": ident, "d_fix_m": dist(here, to), "to_pad_m": to_pad, "ias": s.ias, "target": l.target(), "collective": c, "agl": s.agl_ft, "bug": bug, "hdg": s.mag, "theta": s.theta, "phi": s.phi, "alt_hold": alt_hold, "vvi_mode": vvi_mode, "alt": alt_now.as_ref().ok(), "target_alt": target_alt, "vs_cmd": vs_cmd }),
                        );
                    }
                }
                // Slowed and level: the hover loop takes the aircraft at the
                // velocity it has, and flies the final.
                // On an approach, down at the gate as well, so the final
                // starts on its own path; a descent that never arrived still
                // hands over inside the final's length rather than fly past.
                let at_gate = match (gate, alt_now.as_ref().ok()) {
                    (Some((pad, g)), Some(alt)) => {
                        *alt <= pad + g + 80.0 || remaining < final_len_m(g) * 0.8
                    }
                    _ => true,
                };
                // Slowed is enough on its own: the hover loop is what holds
                // a helicopter at these speeds, and waiting for the gate's
                // height kept the cycle above going over the ridge.
                let slowed = s.ias < HANDOVER_KTS + 2.0;
                if phase == Phase::Slow
                    && (at_gate || slowed)
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

    /// The approach, flown on a plant: speed through the law's own ramp and a
    /// first-order lag, the vertical-speed mode as two 1.5 s lags with a
    /// second of dead time, the dial rewritten by the real law every half
    /// second. From a 1,000 ft and a 2,000 ft cruise over flat ground and over
    /// a ridge near the pad: at the gate when the slow-down starts, never
    /// under the path's floor, never past the sink cap, and minutes faster
    /// than the old slow-down at the cruise height.
    #[test]
    fn the_approach_comes_down_at_speed_to_the_gate() {
        struct Case {
            cruise_agl: f64,
            ridge_ft: Option<f64>,
        }
        for case in [
            Case {
                cruise_agl: 1000.0,
                ridge_ft: None,
            },
            Case {
                cruise_agl: 2000.0,
                ridge_ft: None,
            },
            Case {
                cruise_agl: 1500.0,
                ridge_ft: Some(400.0),
            },
        ] {
            let pad = 100.0;
            let cruise = pad + case.cruise_agl;
            let g = gate_agl_ft(pad, case.ridge_ft.map(|r| pad + r));
            let start = approach_start_m(cruise, pad, GATE_FT, CRUISE_KTS);
            let slow_at = approach_slow_m(g);
            let dt = 0.5;
            let (mut d, mut ias, mut alt) = (start, CRUISE_KTS, cruise);
            let (mut target_kts, mut v1, mut v2) = (CRUISE_KTS, 0.0_f64, 0.0_f64);
            let mut dial_hist = std::collections::VecDeque::from(vec![0.0; 2]);
            let mut t = 0.0;
            let mut worst_sink: f64 = 0.0;
            let mut alt_at_slow = None;
            while d > final_len_m(g) {
                let want = if d < slow_at {
                    HANDOVER_KTS
                } else {
                    APPROACH_KTS
                };
                target_kts += (want - target_kts).clamp(-DECEL_KTS_S * dt, DECEL_KTS_S * dt);
                ias += (target_kts - ias) * dt / 3.0;
                let target = approach_target_ft(d, pad, g, cruise);
                let ahead = approach_target_ft((d - ias * 0.5144 * 10.0).max(0.0), pad, g, cruise);
                let vs = approach_vs_fpm(alt, target, ahead);
                worst_sink = worst_sink.max(-vs);
                // Level at the gate once captured: the dial is the gate.
                let vs = if alt <= pad + g + 5.0 && vs < 0.0 {
                    0.0
                } else {
                    vs
                };
                dial_hist.push_back(vs);
                let delayed = dial_hist.pop_front().unwrap();
                v1 += (delayed - v1) * dt / 1.5;
                v2 += (v1 - v2) * dt / 1.5;
                alt += v2 / 60.0 * dt;
                assert!(
                    alt > pad + g - 60.0,
                    "{}: {alt:.0} ft under the gate {:.0} at {d:.0} m",
                    case.cruise_agl,
                    pad + g
                );
                if alt_at_slow.is_none() && d < slow_at {
                    alt_at_slow = Some(alt);
                }
                d -= ias * 0.5144 * dt;
                t += dt;
            }
            let at_slow = alt_at_slow.expect("reached the slow-down");
            assert!(
                (at_slow - (pad + g)).abs() < 80.0,
                "{}: {at_slow:.0} ft at the slow-down, gate {:.0}",
                case.cruise_agl,
                pad + g
            );
            assert!(worst_sink <= APPROACH_MAX_SINK_FPM + 1.0);
            // The old way: slow at the cruise height, the final from there.
            let old = slow_start_m(CRUISE_KTS, case.cruise_agl);
            let final_old = final_len_m(case.cruise_agl);
            let new_final = final_len_m(g);
            let old_s = (CRUISE_KTS - HANDOVER_KTS) / DECEL_KTS_S
                + (old - decel_m(CRUISE_KTS, HANDOVER_KTS) - final_old).max(0.0)
                    / (HANDOVER_KTS * 0.5144)
                + final_old / (HANDOVER_KTS * 0.5144);
            let new_s = t + new_final / (HANDOVER_KTS * 0.5144);
            eprintln!(
                "cruise {} ft AGL, gate {g:.0}: approach {:.0} s + final {:.0} s against the old {:.0} s",
                case.cruise_agl,
                t,
                new_final / (HANDOVER_KTS * 0.5144),
                old_s
            );
            assert!(
                new_s < old_s - 60.0,
                "{}: {new_s:.0} s against {old_s:.0} s",
                case.cruise_agl
            );
        }
    }

    #[test]
    fn the_approach_path_and_gate() {
        // The gate is 500 ft over the pad, or clears a ridge by 400.
        assert_eq!(gate_agl_ft(100.0, None), GATE_FT);
        assert_eq!(gate_agl_ft(100.0, Some(150.0)), GATE_FT);
        assert_eq!(gate_agl_ft(100.0, Some(600.0)), 900.0);
        // The path: level at the cruise far out, down to the gate at the
        // slow-down, level at the gate inside it.
        let (pad, cruise) = (100.0, 1100.0);
        let slow = approach_slow_m(GATE_FT);
        assert_eq!(approach_target_ft(20_000.0, pad, GATE_FT, cruise), cruise);
        assert_eq!(
            approach_target_ft(slow, pad, GATE_FT, cruise),
            pad + GATE_FT
        );
        assert_eq!(
            approach_target_ft(100.0, pad, GATE_FT, cruise),
            pad + GATE_FT
        );
        // It starts where the path from the cruise meets the level, plus the
        // slow-down to the approach speed.
        let start = approach_start_m(cruise, pad, GATE_FT, CRUISE_KTS);
        assert!(start > slow + decel_m(CRUISE_KTS, APPROACH_KTS));
        // The final from the gate begins on its own path.
        assert!((final_command(final_len_m(GATE_FT), GATE_FT, 19.5).1 - GATE_FT).abs() < 60.0);
        // The vertical law: level on the target, down toward it, and up when
        // under the floor.
        assert_eq!(approach_vs_fpm(1000.0, 1000.0, 1000.0), 0.0);
        assert!(approach_vs_fpm(1300.0, 1000.0, 1000.0) < -500.0);
        assert!(approach_vs_fpm(900.0, 1000.0, 1000.0) > 0.0);
        assert_eq!(
            approach_vs_fpm(5000.0, 1000.0, 900.0),
            -APPROACH_MAX_SINK_FPM
        );
    }

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
        // LILO-LILB: the lake point short of LILB is the way in, kept.
        let lake = vec![
            p("4553N00909E", 45.8833, 9.15),
            p("4550N00905E", 45.8333, 9.0833),
            p("LILB runway 21", 45.7706, 9.1628),
        ];
        assert_eq!(drop_terminal_fixes(lake).len(), 3);
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
    fn a_departure_climb_into_a_ridge_is_found_with_its_numbers() {
        // 500 fpm at 35 kt is about 857 ft a mile after an 80 m level run.
        assert!((depart_climb_ft(4.3, false) - 3_650.0).abs() < 60.0, "{}", depart_climb_ft(4.3, false));
        // The fast climb: the same to 500 ft, then about 560 ft a mile.
        assert_eq!(depart_climb_ft(0.5, true), depart_climb_ft(0.5, false));
        let f = depart_climb_ft(4.3, true);
        assert!(f < depart_climb_ft(4.3, false) && (f - 2_500.0).abs() < 150.0, "{f}");
        // LILO-LILB, probed 2026-09-24: 946 ft field, the ridge 7,728 at 4.3 nm.
        let ground = [(1.1, 2_360.0), (3.3, 6_619.0), (4.3, 7_728.0), (5.4, 4_730.0)];
        let (d, g, a) = climb_conflict(946.0, &ground, false).expect("into the ridge");
        assert_eq!((d, g), (4.3, 7_728.0));
        assert!(a < 5_000.0, "{a}");
        // Up a valley (LSZS, the Engadin): nothing.
        let valley = [(1.0, 5_700.0), (2.5, 6_638.0), (6.0, 6_100.0)];
        assert!(climb_conflict(5_600.0, &valley, false).is_none());
        // The LILO valley route: a level floor right off the pad is no
        // conflict (0.3 nm: 954 ft ground, the climb at about 1,170).
        let floor = [(0.35, 954.0), (1.0, 1_047.0), (3.0, 948.0)];
        assert!(climb_conflict(946.0, &floor, true).is_none());
    }

    #[test]
    fn climb_stations_stop_at_the_distance_and_carry_both_sides() {
        let line = [(46.0, 9.0), (46.0, 9.5)];
        let s = climb_stations(&line, 2.0, 500.0, 500.0);
        assert!(s.iter().all(|p| p.0 <= 2.0));
        assert_eq!(s.len() % 3, 0);
        assert!((s[1].1 - 46.0).abs() > 0.004, "a side point is off the track");
    }

    #[test]
    fn no_helipad_lands_on_the_runway_near_the_arrival() {
        let apt = crate::nav::AirportInfo {
            name: "Alzate Brianza".into(),
            elev_ft: 1_258.0,
            runways: vec![crate::nav::Runway {
                ident_a: "03".into(),
                ident_b: "21".into(),
                lat_a: 45.7671573,
                lon_a: 9.1605458,
                lat_b: 45.7720916,
                lon_b: 9.1636892,
                width_m: 15.0,
                surface: 1,
            }],
            freqs: Vec::new(),
        };
        // Arriving from the north-east (Lake Como): runway 21's end.
        let (la, lo, name) = runway_spot(&apt, (45.85, 9.25)).expect("a spot");
        assert_eq!(name, "runway 21");
        let from_thr = crate::nav::haversine_nm(45.7720916, 9.1636892, la, lo) * 1852.0;
        assert!((from_thr - 150.0).abs() < 5.0, "{from_thr}");
        let mut grass = apt.clone();
        grass.runways[0].surface = 3;
        assert!(runway_spot(&grass, (45.85, 9.25)).is_none(), "no hard runway, no spot");
    }

    #[test]
    fn the_cruise_is_the_airframes_and_settles_where_the_power_runs_out() {
        assert_eq!(power_limited_kts(131.7), 130.0);
        assert_eq!(power_limited_kts(52.0), APPROACH_KTS, "never under the approach speed");
        assert!(collective_capped(0.85) && collective_capped(0.846) && !collective_capped(0.83));
    }

    #[test]
    fn the_approach_collective_never_drops_to_nothing() {
        // The LILO-LILB gate: 86 kt against a 70 kt target.
        let mut l = SpeedLaw::from(0.40, 86.0);
        l.set_floor(APPROACH_COLLECTIVE_MIN);
        let mut c = 0.4;
        for _ in 0..60 {
            c = l.step(70.0, 86.0, 0.5);
        }
        assert!((c - APPROACH_COLLECTIVE_MIN).abs() < 1e-9, "{c}");
        let mut cruise = SpeedLaw::from(0.40, 86.0);
        for _ in 0..60 {
            c = cruise.step(70.0, 86.0, 0.5);
        }
        assert!(c < 0.3, "the cruise floor stays 0.25: {c}");
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
        // The corridor: the track plus both sides, a kilometre out.
        let c = sample_corridor(&line, 1000.0, 1000.0);
        assert_eq!(c.len(), s.len() + 2 * (s.len() - 1));
        let north = c.iter().map(|p| p.0).fold(f64::MIN, f64::max);
        let south = c.iter().map(|p| p.0).fold(f64::MAX, f64::min);
        assert!(((north - 52.0) * 60.0 * 1852.0 - 1000.0).abs() < 20.0, "{north}");
        assert!(((52.0 - south) * 60.0 * 1852.0 - 1000.0).abs() < 20.0, "{south}");
    }

    #[test]
    fn a_lost_level_comes_back_gently() {
        assert_eq!(recovery_vs_fpm(4200.0, 2200.0), -700.0);
        assert_eq!(recovery_vs_fpm(2000.0, 2200.0), 200.0);
        assert_eq!(recovery_vs_fpm(2200.0, 2200.0), 0.0);
    }

    #[test]
    fn every_mission_that_leaves_a_field_works_the_radio() {
        // One radio crew, not two: the aeroplane and the helicopter get the
        // zone clearance and take both handoffs through the same functions.
        // The S-76 once flew EDVW-ETND on a takeoff clearance alone.
        let src = |f: &str| {
            std::fs::read_to_string(format!("{}/src/{f}", env!("CARGO_MANIFEST_DIR"))).unwrap()
        };
        for (file, calls) in [
            (
                "full_auto.rs",
                &[
                    "get_zone_clearance(&xp",
                    "obey_departure_handoff(xp",
                    "obey_region_handoff(xp",
                ][..],
            ),
            (
                "heli_route.rs",
                &[
                    "full_auto::get_zone_clearance(",
                    "full_auto::obey_departure_handoff(",
                    "full_auto::obey_region_handoff(",
                    "obey_handoffs(&base_url",
                ][..],
            ),
        ] {
            let text = src(file);
            for c in calls {
                assert!(text.contains(c), "{file} no longer calls {c}");
            }
        }
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
