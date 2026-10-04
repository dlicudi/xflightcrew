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
//! - **Slow** on the last leg: level at the gate, a schedule down to
//!   [`HANDOVER_KTS`] where the final begins at a deceleration the S-76 makes
//!   ([`SLOW_DECEL_KTS_S`]), flown by [`SlowLaw`] (the lever under the level
//!   one, so the autopilot's nose-up does the slowing), so the hover loop
//!   takes over a steady aircraft. The morning's takeover caught one
//!   decelerating hard at a cruise collective, and the pitch rang. A
//!   slow-down that cannot make the pad goes around straight ahead
//!   ([`slow_check`], `Phase::Around`), never an orbit (#191).
//! - **Arrive** on the hover loop: a final at 6 degrees to the helipad nearest
//!   the inbound fix, the speed tapering to stop over it, then down.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::Duration;

use crate::hover::{self, Goal};
use crate::taxi_guide::{bearing, dist, Geo};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

pub static ACTIVE: AtomicBool = AtomicBool::new(false);
static GENERATION: AtomicU64 = AtomicU64::new(0);

pub fn abort() {
    GENERATION.fetch_add(1, Ordering::SeqCst);
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

/// The cruise speed: the airframe's own (`reference.cruise_ias_kts`, 128 kt
/// on the S-76, under its 139.5 kt overspeed caution), never past its speed
/// ceiling (`aircraft::ceiling_from_vne`), [`CRUISE_KTS`] when it declares none. A flat 120 flew the S-76 20 kt
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

/// The final's GROUNDSPEED ceiling, m/s: the hover mode's own range, the
/// hand-over speed flown over the ground. The mode is a groundspeed hold and
/// its let-go is a groundspeed: climbing out of LSZS (2026-09-24) it held 35
/// kt over the ground at 50 to 56 kt indicated in the valley wind, and the
/// one let-go it has shown came at 39 kt over the ground and 37 indicated
/// (EDDV, 2026-09-23).
pub const FINAL_MAX_GS_MPS: f64 = HANDOVER_KTS * 0.514_444;
/// The final's AIRSPEED ceiling, kt (true, from the wind at the aircraft):
/// the most the hover mode is recorded holding (56 kt indicated at LSZS). A
/// headwind stronger than the difference slows the final over the ground.
pub const FINAL_MAX_AIR_KTS: f64 = 55.0;

/// Headwind (positive into the nose of the track) and crosswind, m/s, of a
/// wind blowing FROM `from_deg` (true) at `mps`, on a true `track_deg`.
pub fn wind_components(track_deg: f64, from_deg: f64, mps: f64) -> (f64, f64) {
    let rel = (from_deg - track_deg).to_radians();
    (mps * rel.cos(), mps * rel.sin())
}

/// The final's groundspeed cap, m/s: as fast over the ground as the hover
/// mode flies ([`FINAL_MAX_GS_MPS`]), and no faster through the air than
/// [`FINAL_MAX_AIR_KTS`]; never under the groundspeed it was handed over at
/// (`from_mps`, itself inside the mode's range). `wind` is (headwind,
/// crosswind) on the final's track, m/s; `None` (no read) keeps the old
/// cap, the handed-over groundspeed. Until #82 that WAS the cap: into ETND
/// (2026-09-23) the slow-down reached 38 kt indicated with 21.6 kt over the
/// ground in a 17 kt headwind, and the final flew 4.4 km at 21.6 kt, six and
/// a half minutes. Pure.
pub fn final_cap_mps(from_mps: f64, wind: Option<(f64, f64)>) -> f64 {
    let base = from_mps.clamp(0.0, FINAL_MAX_GS_MPS);
    let Some((head, cross)) = wind.filter(|(h, x)| h.is_finite() && x.is_finite()) else {
        return base;
    };
    let air = FINAL_MAX_AIR_KTS * 0.514_444;
    // Airspeed = |ground + head, cross|: the fastest ground that keeps it
    // at the ceiling.
    let by_air = (air * air - cross * cross).max(0.0).sqrt() - head;
    by_air.min(FINAL_MAX_GS_MPS).max(base)
}

/// The slow-down's IAS target, kt: the hand-over speed, less the tailwind the
/// aircraft is flying in (groundspeed over indicated), so the GROUNDSPEED
/// arrives inside the hover mode's takeover range ([`hover::TAKEOVER_MAX_MPS`]);
/// never under [`SLOW_MIN_KTS`]. A headwind leaves it at [`HANDOVER_KTS`]:
/// slower over the ground is already inside the range. With 38 kt indicated
/// a 12 kt tailwind is 50 kt over the ground, and the takeover, which waits
/// for 39, never came: the route flew on over the pad. Pure.
pub fn slow_target_kts(ias_kts: f64, gs_kts: f64) -> f64 {
    if !(ias_kts.is_finite() && gs_kts.is_finite()) {
        return HANDOVER_KTS;
    }
    (HANDOVER_KTS - (gs_kts - ias_kts).max(0.0)).max(SLOW_MIN_KTS)
}
/// The slowest the slow-down flies under the cruise modes: altitude hold at
/// 35 kt drifted backwards and sank once (ETHC, 2026-09-23), so the tailwind
/// correction stops here.
pub const SLOW_MIN_KTS: f64 = 30.0;

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

/// The slow-down's PLANNED deceleration, kt/s (over the ground). The S-76's
/// own, fitted to 1790715215-heli_route (LEMG-LXGB, iMac 2026-09-29): the
/// slow-down began 2.16 km out at 117 KIAS with the collective on the old
/// 0.35 floor and bled 117 to 88 kt in 55 s (0.53 kt/s), toward 78 kt,
/// where 0.35 is the level lever: the floor was a speed the helicopter
/// could not go under, and it orbited the pad at 79 kt for 26 minutes. With
/// the slow-down's lever down to [`SLOW_COLLECTIVE_MIN`] the fitted power
/// curve ([`CRUISE_POWER_CURVE`], [`COLLECTIVE_POWER_WKG`]) makes 1.2 kt/s
/// level at its worst (the bucket, 65 to 80 kt) and 2.7 at 118: the plan
/// flies 60 % of the worst.
pub const SLOW_DECEL_KTS_S: f64 = 0.7;
/// A slow-down that would need more than this to reach the hand-over speed
/// at the final cannot make it: it goes around ([`slow_check`]).
pub const SLOW_DECEL_MAX_KTS_S: f64 = 1.5;
/// Room left over the planned slow-down, m: the law's lag and the level-off.
const SLOW_MARGIN_M: f64 = 300.0;

/// Distance flown slowing from `from_kts` to `to_kts` at the planned
/// deceleration, m (both groundspeeds). Pure.
pub fn slow_down_m(from_kts: f64, to_kts: f64) -> f64 {
    (from_kts * from_kts - to_kts * to_kts).max(0.0) / (2.0 * SLOW_DECEL_KTS_S) * 0.514_444
}

/// The slow-down's groundspeed schedule, kt, `to_final_m` before the final
/// begins: `to_kts` there, and at the planned deceleration before it
/// (v^2 = vh^2 + 2 a d). Pure.
pub fn slow_schedule_kts(to_final_m: f64, to_kts: f64) -> f64 {
    (to_kts * to_kts + 2.0 * SLOW_DECEL_KTS_S * to_final_m.max(0.0) / 0.514_444).sqrt()
}

/// Where the slow-down starts, metres before the pad, when there is no
/// approach (no pad elevation): at the cruise height, the planned slow-down
/// from the cruise speed, then the final's length from there, and a margin.
pub fn slow_start_m(cruise_kts: f64, agl_ft: f64) -> f64 {
    slow_down_m(cruise_kts, HANDOVER_KTS) + final_len_m(agl_ft) + 1000.0
}

/// The final's length on its own path from `agl_ft`, m.
pub fn final_len_m(agl_ft: f64) -> f64 {
    (agl_ft - FINAL_HOVER_FT).max(0.0) / 3.28084 / GLIDE_DEG.to_radians().tan()
}

/// The gate's height over the pad: [`GATE_FT`], or higher where the ground
/// still to be crossed (`floor_ft`, its highest point, same feet as `pad_ft`)
/// needs [`APPROACH_CLEAR_FT`] over it.
pub fn gate_agl_ft(pad_ft: f64, floor_ft: Option<f64>) -> f64 {
    floor_ft.map_or(GATE_FT, |f| (f + APPROACH_CLEAR_FT - pad_ft).max(GATE_FT))
}

/// The approach's geometry, fixed at its start from the gate and the
/// groundspeed it is flown at: the descent comes down at speed to the gate
/// where the slow-down begins, and the slow-down is level at the gate to
/// where the final's own path from the gate begins, so the final starts ON
/// its path, never above it (from above, `final_command` dives to it: 2,000
/// ft into ETND, 2026-09-23).
///
/// Until #191 both ends were the 70 kt book's: a path of 6.5 degrees, which
/// at the 118 kt a floored collective actually holds on the way down is
/// 1,360 fpm against the 1,200 cap (the S-76 reached the slow-down 300 ft
/// over the gate), and a slow-down sized from 70 kt at 1 kt/s (2.25 km) that
/// began at 117. Now the path is as steep as the cap allows at the speed
/// flown ([`approach_tan`]) and the level segment is the planned slow-down
/// from that speed ([`slow_down_m`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ApproachPath {
    /// The gate's height over the pad, ft.
    pub gate_agl: f64,
    /// Where the path meets the gate and the slow-down starts, m before the pad.
    pub slow_m: f64,
    /// The path's gradient (rise over run).
    pub tan: f64,
}

/// The approach path's gradient at a groundspeed: 6.5 degrees, or what 85 %
/// of the vertical-speed cap makes at that speed if shallower. Pure.
pub fn approach_tan(gs_kts: f64) -> f64 {
    let by_cap = APPROACH_MAX_SINK_FPM * 0.85 / (gs_kts.max(1.0) * 101.269);
    APPROACH_DEG.to_radians().tan().min(by_cap)
}

impl ApproachPath {
    pub fn new(gate_agl: f64, gs_kts: f64) -> Self {
        let kts = gs_kts.max(APPROACH_KTS);
        Self {
            gate_agl,
            slow_m: approach_slow_m(gate_agl, kts),
            tan: approach_tan(kts),
        }
    }
    /// The target altitude (ft, the dial's) at `d_m` from the pad: the path
    /// down to the gate where the slow-down starts, never above the cruise.
    pub fn target_ft(&self, d_m: f64, pad_ft: f64, cruise_ft: f64) -> f64 {
        let gate = pad_ft + self.gate_agl;
        let path = gate + (d_m - self.slow_m).max(0.0) * self.tan * 3.28084;
        path.min(cruise_ft.max(gate))
    }
    /// Where the approach starts, m before the pad: the slow-down, then the
    /// path from the cruise height.
    pub fn start_m(&self, cruise_ft: f64, pad_ft: f64) -> f64 {
        let above = (cruise_ft - pad_ft - self.gate_agl).max(0.0);
        self.slow_m + above / 3.28084 / self.tan
    }
}

/// Where the slow-down to the hand-over speed starts on an approach flown at
/// `gs_kts`, m before the pad: the final's length from the gate, the planned
/// slow-down before it, and a margin.
pub fn approach_slow_m(gate_agl: f64, gs_kts: f64) -> f64 {
    final_len_m(gate_agl) + slow_down_m(gs_kts, HANDOVER_KTS) + SLOW_MARGIN_M
}

/// The collective level flight needs at an indicated airspeed in the
/// cruise modes, the S-76 at the end of a 45 minute flight, fitted to
/// 1790715215-heli_route (2026-09-29): 0.675 at 125 kt (the cruise), 0.60 at
/// 117 and 0.46 at 100 (the approach at the 0.35 floor held 118 kt at 1,200
/// fpm down), 0.35 at 78 (where the 0.35 floor held it level, turning, for
/// 26 minutes), the bucket about 0.34 near 65, and under it the hover loop's
/// own curve ([`hover::level_collective`]) scaled to that loading. The slow
/// law uses its SLOPE to move the trim as the target comes down and its
/// value for a ceiling; the test plant uses the whole curve.
pub const CRUISE_POWER_CURVE: &[(f64, f64)] = &[
    (0.0, 0.55),
    (10.0, 0.51),
    (16.0, 0.46),
    (23.0, 0.41),
    (30.0, 0.38),
    (40.0, 0.36),
    (55.0, 0.345),
    (65.0, 0.34),
    (78.0, 0.35),
    (100.0, 0.46),
    (117.0, 0.60),
    (125.0, 0.675),
    (140.0, 0.80),
];

/// W/kg of excess power per unit of collective, fitted with the curve: the
/// descent at 118 kt on 0.35 (level lever 0.61) held its speed at 1,200 fpm
/// (60 W/kg), and the level slow-down's 0.75 kt/s at 117 with 690 fpm of sink
/// agrees within 5 %. The plant's `dV/dt = (K (c - level) - g vz) / V`
/// replays the recorded slow-down within a knot for 95 s.
pub const COLLECTIVE_POWER_WKG: f64 = 230.0;

fn curve_at(ias_kts: f64) -> (f64, f64) {
    let c = CRUISE_POWER_CURVE;
    let s = ias_kts.max(0.0);
    for w in c.windows(2) {
        let ((s0, c0), (s1, c1)) = (w[0], w[1]);
        if s <= s1 {
            let slope = (c1 - c0) / (s1 - s0);
            return (c0 + slope * (s - s0), slope);
        }
    }
    let n = c.len();
    let slope = (c[n - 1].1 - c[n - 2].1) / (c[n - 1].0 - c[n - 2].0);
    (c[n - 1].1 + slope * (s - c[n - 1].0), slope)
}

/// The level collective at an indicated airspeed ([`CRUISE_POWER_CURVE`]).
pub fn cruise_level_collective(ias_kts: f64) -> f64 {
    curve_at(ias_kts).0
}

/// The slow-down's lever floor: the cruise law's own. The approach's 0.35
/// ([`APPROACH_COLLECTIVE_MIN`]) is the level lever at 78 kt, so it could
/// never slow the S-76 under 78 (#191). What the 0.25 did at LILO
/// (2026-09-24, a crash) was a speed law chasing a 38 kt step: the collective
/// on the floor, the VS mode's nose up to 15 degrees, then the law's trim
/// driving 0.65 at 25 kt and the climb that rang. The slow law flies a
/// SCHEDULE, never asks the lever back up by more than [`SLOW_UP_ERR_KTS`] of
/// error, keeps it under the level lever plus [`SLOW_CEILING_OVER_LEVEL`] and
/// hands to the hover loop at the hand-over speed.
pub const SLOW_COLLECTIVE_MIN: f64 = 0.25;
/// The nose-up the slow-down is allowed before the lever comes up to ease
/// the deceleration, degrees; per degree past it, [`SLOW_PITCH_GUARD`].
pub const SLOW_PITCH_MAX_DEG: f64 = 10.0;
const SLOW_PITCH_GUARD: f64 = 0.04;
const SLOW_KP_PER_KT: f64 = 0.008;
const SLOW_KI_PER_KT_S: f64 = 0.001;
/// A speed under the schedule asks the lever up by no more than this much
/// error: the slow-down never re-accelerates hard (LILO's 0.65).
const SLOW_UP_ERR_KTS: f64 = 3.0;
/// The slow-down's lever stays under the level lever at the speed flown plus
/// this (the backside's climb at LILO).
const SLOW_CEILING_OVER_LEVEL: f64 = 0.10;

/// The collective for the slow-down: the autopilot holds the level (or the
/// descent to the gate) on the pitch, so a lever under the level one is a
/// nose-up deceleration, which is how a helicopter slows (the cyclic stays
/// the autopilot's). A feedforward of the schedule's deceleration through
/// the fitted power ([`COLLECTIVE_POWER_WKG`]), a trim that follows the
/// level curve's slope as the target comes down, P and a slow I on the speed
/// error, a pitch guard, slewed like a pilot's hand. Pure.
#[derive(Debug, Clone, Copy)]
pub struct SlowLaw {
    cmd: f64,
    trim: f64,
    want: Option<f64>,
}

impl SlowLaw {
    pub fn from(collective: f64) -> Self {
        Self {
            cmd: collective,
            trim: collective,
            want: None,
        }
    }
    /// One step: `want_kts` the indicated target (never rising), `decel_kts_s`
    /// the schedule's deceleration now (0 once at the hand-over speed).
    pub fn step(
        &mut self,
        want_kts: f64,
        decel_kts_s: f64,
        ias_kts: f64,
        theta_deg: f64,
        dt_s: f64,
    ) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        if let Some(prev) = self.want {
            self.trim += curve_at(want_kts).1 * (want_kts - prev);
        }
        self.want = Some(want_kts);
        let (level, _) = curve_at(ias_kts);
        let ceiling = (level + SLOW_CEILING_OVER_LEVEL).clamp(SLOW_COLLECTIVE_MIN, COLLECTIVE_MAX);
        let err = (want_kts - ias_kts).min(SLOW_UP_ERR_KTS);
        let ff = -decel_kts_s.max(0.0) * 0.514_444 * (ias_kts * 0.514_444).max(8.0) / COLLECTIVE_POWER_WKG;
        let guard = (theta_deg - SLOW_PITCH_MAX_DEG).max(0.0) * SLOW_PITCH_GUARD;
        self.trim = (self.trim + err * SLOW_KI_PER_KT_S * dt).clamp(SLOW_COLLECTIVE_MIN, ceiling);
        let want = ((self.trim + ff + err * SLOW_KP_PER_KT).clamp(SLOW_COLLECTIVE_MIN, ceiling) + guard)
            .min(COLLECTIVE_MAX);
        self.cmd += (want - self.cmd).clamp(-COLLECTIVE_SLEW * dt, COLLECTIVE_SLEW * dt);
        self.cmd
    }
}

/// The slow-down's indicated target, kt: a groundspeed schedule ending at
/// the hand-over's own speed ([`slow_target_kts`]) over the ground where the
/// final begins, less the wind along the track (a headwind adds), never
/// under the hand-over's, never above `prev` (the target only comes down;
/// the first is the speed the slow-down began at). Pure.
pub fn slow_want_kts(to_final_m: f64, ias_kts: f64, gs_kts: f64, prev: f64) -> f64 {
    let wind = gs_kts - ias_kts;
    let to_ias = slow_target_kts(ias_kts, gs_kts);
    let to_gs = (to_ias + wind).max(1.0);
    (slow_schedule_kts(to_final_m, to_gs) - wind).max(to_ias).min(prev)
}

/// How a slow-down stands against the pad.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SlowCheck {
    OnTrack,
    /// The pad is behind us and we are still faster than the hand-over.
    Overflown,
    /// The deceleration left to reach the hand-over speed by the final is
    /// past [`SLOW_DECEL_MAX_KTS_S`] (`need` kt/s), or there is no room left.
    TooFast { need: f64 },
}

/// Whether the slow-down can still make the pad (#191): the pad behind the
/// nose with the speed still above the hand-over is overflown; the
/// deceleration still needed (over the ground, to where half the final is
/// left, the least a hand-over can start its final from) past the most the
/// S-76 is planned to make is too fast. Either goes around; a slow enough
/// aircraft is the hover loop's wherever it is. Pure.
pub fn slow_check(gs_kts: f64, ias_kts: f64, remaining_m: f64, final_m: f64, pad_behind: bool) -> SlowCheck {
    if ias_kts < HANDOVER_KTS + 2.0 {
        return SlowCheck::OnTrack;
    }
    if pad_behind {
        return SlowCheck::Overflown;
    }
    let room = remaining_m - final_m * 0.5;
    let to = (slow_target_kts(ias_kts, gs_kts) + gs_kts - ias_kts).max(1.0);
    if room <= 0.0 {
        return SlowCheck::TooFast { need: f64::INFINITY };
    }
    let need = (gs_kts * gs_kts - to * to).max(0.0) / (2.0 * room / 0.514_444);
    if need > SLOW_DECEL_MAX_KTS_S {
        SlowCheck::TooFast { need }
    } else {
        SlowCheck::OnTrack
    }
}

/// The go-around's bounds (#191): slowing straight ahead on the course it
/// began on, a slow-down that has not reached the hand-over speed in this
/// long, or this far past the pad, is given back.
pub const AROUND_MAX_S: f64 = 180.0;
pub const AROUND_MAX_M: f64 = 6_000.0;
/// The ground ahead of a go-around is probed this far, and the level raised
/// to clear it by [`APPROACH_CLEAR_FT`].
const AROUND_PROBE_M: f64 = 5_000.0;

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

/// Every hard runway end's landing spot, [`RUNWAY_SPOT_M`] in from its
/// threshold: (ident, threshold, spot). The spot a tower's runway names
/// ([`spot_for_runway`]) and the one nearest the arrival ([`runway_spot`])
/// come from the same list.
fn runway_ends(apt: &crate::nav::AirportInfo) -> Vec<(String, (f64, f64), (f64, f64))> {
    let mut out = Vec::new();
    for r in apt.runways.iter().filter(|r| r.is_hard()) {
        let len_nm = crate::nav::haversine_nm(r.lat_a, r.lon_a, r.lat_b, r.lon_b);
        let into = (RUNWAY_SPOT_M / 1852.0).min(len_nm / 2.0);
        for (thr, far, id) in [
            ((r.lat_a, r.lon_a), (r.lat_b, r.lon_b), &r.ident_a),
            ((r.lat_b, r.lon_b), (r.lat_a, r.lon_a), &r.ident_b),
        ] {
            let brg = crate::nav::bearing_deg(thr.0, thr.1, far.0, far.1);
            let spot = crate::nav::project_latlon(thr.0, thr.1, brg, into);
            out.push((id.trim().to_string(), thr, spot));
        }
    }
    out
}

/// Where a helicopter lands at a field with no helipad: on the hard runway
/// end nearest the arrival, [`RUNWAY_SPOT_M`] in from the threshold (never
/// the reference point, which at LILB is 120 m off the strip in whatever is
/// there). (lat, lon, name). A tower that names its runway moves it
/// ([`spot_for_runway`]).
pub fn runway_spot(apt: &crate::nav::AirportInfo, inbound: (f64, f64)) -> Option<(f64, f64, String)> {
    runway_ends(apt)
        .into_iter()
        .map(|(id, thr, spot)| (crate::nav::haversine_nm(inbound.0, inbound.1, thr.0, thr.1), id, spot))
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, id, (la, lo))| (la, lo, format!("runway {id}")))
}

/// Every runway end's spot as (ident, lat, lon), for the route to move its
/// landing onto the runway the tower gives ([`spot_for_runway`]).
pub fn runway_spots(apt: &crate::nav::AirportInfo) -> Vec<(String, f64, f64)> {
    runway_ends(apt)
        .into_iter()
        .map(|(id, _, (la, lo))| (id, la, lo))
        .collect()
}

/// The spot of the runway END the tower named ("09"), when it is not the
/// one the route already lands on (`current`). Idents compare trimmed and
/// case-blind. `None`: no runway named, the same end, or an end the field
/// does not have. Pure.
///
/// LXGB (iMac 2026-09-30, sortie 1790722408-heli_route): the inbound call
/// earned "join final runway zero niner ... Report final", the route flew
/// on to its spot on 27 (the end nearest the arrival) and set down there,
/// and the tower called "you are on runway two seven without clearance".
pub fn spot_for_runway(
    spots: &[(String, f64, f64)],
    current: Option<&str>,
    named: Option<&str>,
) -> Option<(String, f64, f64)> {
    let named = named.map(str::trim).filter(|n| !n.is_empty())?;
    if current.is_some_and(|c| c.trim().eq_ignore_ascii_case(named)) {
        return None;
    }
    spots
        .iter()
        .find(|(id, ..)| id.eq_ignore_ascii_case(named))
        .cloned()
}

/// The runway end a spot's name lands on ("LXGB runway 27" is "27").
pub fn spot_runway(name: &str) -> Option<String> {
    name.rsplit_once("runway ")
        .map(|(_, id)| id.trim().to_string())
        .filter(|id| !id.is_empty())
}

/// How far short of the pad the final holds in the hover while a towered
/// field has not cleared the landing. The runway spot is 150 m in from its
/// threshold, so this is 250 m short of the pavement flown along the
/// runway, and further across it.
pub const HOLD_SHORT_M: f64 = 400.0;
/// Held: inside this much of the hold point and slower than
/// [`HELD_GS_MPS`] over the ground.
pub const HOLD_SLACK_M: f64 = 60.0;
pub const HELD_GS_MPS: f64 = 3.0;
/// Held without a clearance, the landing clearance is asked for again this
/// often, at most [`CLEARANCE_ASKS_MAX`] times.
pub const CLEARANCE_ASK_EVERY_S: f64 = 30.0;
pub const CLEARANCE_ASKS_MAX: u32 = 3;
/// Held this long without a clearance, the hover is handed to the pilot
/// where it is (the hover loop keeps holding it).
pub const CLEARANCE_HOLD_MAX_S: f64 = 120.0;

/// The final's command while the landing is not cleared: the speed of a
/// final that stops [`HOLD_SHORT_M`] short of the pad, at the height the
/// final's path has where we are, led for that speed (stopped, the path
/// itself: 168 ft at the hold point). Inside the hold point it is a hover
/// where we are. Pure.
pub fn hold_command(d_m: f64, from_ft: f64, from_mps: f64) -> (f64, f64) {
    let (speed, _) = final_command((d_m - HOLD_SHORT_M).max(0.0), from_ft, from_mps);
    let (_, height) = final_command(d_m, from_ft, speed);
    (speed, height)
}

/// What the route owes the destination's tower on its final.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClearanceStep {
    /// The final report ("final runway zero niner"), once.
    ReportFinal,
    /// Held short without a clearance: "final ..., request landing
    /// clearance" again.
    Ask,
    /// Held too long without one: the hover is the pilot's.
    GiveUp,
}

/// The helicopter's landing clearance at a towered field: the aeroplanes'
/// rule (`autoland::UnclearedWatch`), a crew never lands uncleared, flown
/// the helicopter's way: the final reports, and a final not cleared holds
/// in the hover short of the pad and asks again, bounded. `needs`: the
/// destination has a tower and we called it; `tower_ready`: our inbound
/// call went out and the station is that tower; `cleared`: its book says
/// cleared to land; `held`: stopped at the hold point. Pure (time passed in).
#[derive(Debug, Default)]
pub struct ClearanceWatch {
    reported: bool,
    asks: u32,
    last_call_s: Option<f64>,
    held_since_s: Option<f64>,
    gave_up: bool,
}

impl ClearanceWatch {
    pub fn step(
        &mut self,
        now_s: f64,
        needs: bool,
        tower_ready: bool,
        cleared: bool,
        held: bool,
    ) -> Option<ClearanceStep> {
        if !needs || cleared || self.gave_up {
            return None;
        }
        if tower_ready && !self.reported {
            self.reported = true;
            self.last_call_s = Some(now_s);
            return Some(ClearanceStep::ReportFinal);
        }
        if !held {
            return None;
        }
        let since = *self.held_since_s.get_or_insert(now_s);
        if tower_ready
            && self.asks < CLEARANCE_ASKS_MAX
            && self.last_call_s.is_none_or(|c| now_s - c >= CLEARANCE_ASK_EVERY_S)
        {
            self.asks += 1;
            self.last_call_s = Some(now_s);
            return Some(ClearanceStep::Ask);
        }
        if now_s - since >= CLEARANCE_HOLD_MAX_S {
            self.gave_up = true;
            return Some(ClearanceStep::GiveUp);
        }
        None
    }

    pub fn held_since_s(&self) -> Option<f64> {
        self.held_since_s
    }

    pub fn asks(&self) -> u32 {
        self.asks
    }
}

/// The final report's words: the runway the spot lands on, else plain.
pub fn final_words(rwy: Option<&str>, asking: bool) -> String {
    let base = match rwy {
        Some(r) => format!("final runway {}", crate::circuit::spell_runway(r)),
        None => "final".to_string(),
    };
    if asking {
        format!("{base}, request landing clearance")
    } else {
        base
    }
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
    let turn = crate::angles::abs_diff_deg(bearing(mid, far), bearing(near, mid));
    (1.0 - turn / 90.0 * 0.7).clamp(0.3, 1.0)
}

/// The look-ahead along a cleared route, m.
const ROUTE_AHEAD_M: f64 = 25.0;
/// The warning before a hover taxi lifts a helicopter off the ground, s.
const LIFT_WARNING_S: f64 = 5.0;
/// A route's taxi height: a hover taxi, above the ground cushion's worst.
pub const ROUTE_TAXI_FT: f64 = 12.0;

pub fn park(voice: Option<Voice>, base_url: String, collective: String, target: ParkTarget) {
    if ACTIVE.compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst).is_err() {
        return;
    }
    let generation = GENERATION.fetch_add(1, Ordering::SeqCst) + 1;
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            if GENERATION.load(Ordering::SeqCst) == generation {
                ACTIVE.store(false, Ordering::SeqCst);
            }
            return;
        };
        rt.block_on(fly_park(
            XPlaneClient::new(base_url.clone()),
            voice,
            base_url,
            collective,
            target,
            generation,
        ));
        if GENERATION.load(Ordering::SeqCst) == generation {
            ACTIVE.store(false, Ordering::SeqCst);
        }
    });
}

async fn fly_park(
    xp: XPlaneClient,
    voice: Option<Voice>,
    base_url: String,
    collective: String,
    target: ParkTarget,
    generation: u64,
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
        if !ACTIVE.load(Ordering::SeqCst) || GENERATION.load(Ordering::SeqCst) != generation {
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
    while ACTIVE.load(Ordering::SeqCst) && GENERATION.load(Ordering::SeqCst) == generation {
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
    /// From a hover near the pad: the final and the landing only, flown by
    /// the same final that put the S-76 1.9 and 6.4 m from its spots. The
    /// pilot's "not here, land at the airport" at LIME had nothing to call
    /// (2026-09-25).
    pub arrive_only: bool,
    /// An airborne rejoin starts in cruise modes at the present state.
    pub resume: bool,
    /// The destination's runway spots (ident, lat, lon, [`runway_spots`]),
    /// when the flight lands on a runway: the tower's runway moves the
    /// landing onto its own end ([`spot_for_runway`]). Empty on a helipad.
    pub runway_spots: Vec<(String, f64, f64)>,
}

/// Forward endpoint of the route segment the aircraft is abeam of.
pub fn resume_index(points: &[(String, f64, f64)], lat: f64, lon: f64) -> Result<usize, String> {
    if points.len() < 2 || !lat.is_finite() || !lon.is_finite() {
        return Err("The loaded route has no segment I can rejoin.".into());
    }
    let scale = lat.to_radians().cos();
    let mut best: Option<(f64, usize)> = None;
    for (i, pair) in points.windows(2).enumerate() {
        let (ax, ay) = ((pair[0].2 - lon) * 60.0 * scale, (pair[0].1 - lat) * 60.0);
        let (bx, by) = ((pair[1].2 - lon) * 60.0 * scale, (pair[1].1 - lat) * 60.0);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        if len2 < 0.0001 { continue; }
        let u = -(ax * dx + ay * dy) / len2;
        if !(0.0..=1.0).contains(&u) { continue; }
        let d = (ax + u * dx).hypot(ay + u * dy);
        if best.is_none_or(|b| d < b.0) { best = Some((d, i + 1)); }
    }
    match best {
        Some((d, i)) if d <= crate::interaction::REJOIN_CORRIDOR_NM => Ok(i),
        Some((d, _)) => Err(format!(
            "We're {:.0} miles from the route, outside the {:.0} mile rejoin corridor. I haven't taken control.",
            d, crate::interaction::REJOIN_CORRIDOR_NM
        )),
        None => Err("I can't find a forward route segment here. I haven't taken control.".into()),
    }
}

/// Our inbound call to the destination's tower has gone out (set by
/// [`call_inbound`] once transmitted, cleared at each route's start): the
/// final report waits for it, so the tower hears "inbound" before "final".
static INBOUND_SENT: AtomicBool = AtomicBool::new(false);

/// An arrival-only route starts this close to the pad, or not at all.
pub const ARRIVE_ONLY_MAX_M: f64 = 5_000.0;

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
    crate::full_auto::get_zone_clearance(xp, &ctx, voice, &callsign, None).await;
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
    crate::full_auto::transponder_before_departure(xp, voice, "heli_pad").await;
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
        INBOUND_SENT.store(true, Ordering::SeqCst);
        crate::blackbox::record(
            "heli_route",
            serde_json::json!({ "event": "inbound_call", "khz": khz, "station_swapped": swapped }),
        );
    });
}

/// What the route owes a tower that asked "say callsign and intentions"
/// (the zone watch's challenge, `party_line::take_zone_challenge`, drained
/// every tick whoever asked it).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChallengeAnswer {
    /// The destination's own tower: the inbound call is the answer, as for
    /// the aeroplanes (`full_auto::inbound_due`).
    Inbound(i64),
    /// Any other field's tower: we are crossing its zone.
    Transit,
}

/// The answers one field gets before the route stops answering it: a call
/// the tower heard is two-way contact and ends its questions, so a third
/// challenge means the frequency is not hearing us.
pub const CHALLENGE_ANSWERS_MAX: u32 = 2;

/// The answer owed a challenge heard on the station `home_khz`, with the
/// destination's tower `dest_khz` and `answered` answers already given that
/// station. `None`: no challenge, no tower to answer, or answered enough.
/// A crossing field is never answered "inbound" (the aeroplanes' rule). Pure.
pub fn challenge_answer(
    challenged: bool,
    home_khz: Option<i64>,
    dest_khz: Option<i64>,
    answered: u32,
) -> Option<ChallengeAnswer> {
    let home = home_khz.filter(|_| challenged)?;
    if answered >= CHALLENGE_ANSWERS_MAX {
        return None;
    }
    Some(if dest_khz == Some(home) {
        ChallengeAnswer::Inbound(home)
    } else {
        ChallengeAnswer::Transit
    })
}

/// The transit call on the frequency the challenge came on (COM1 is on it:
/// the watch only asks aloud when it is): who, what, how high, and the
/// request. Off the control loop.
fn call_transit(base_url: &str, level_ft: f64) {
    let base = base_url.to_string();
    tokio::spawn(async move {
        let xp = XPlaneClient::new(base);
        let callsign = crate::circuit::crew_callsign(&xp).await;
        let alt = xp
            .get_float("sim/cockpit2/gauges/indicators/altitude_ft_pilot")
            .await
            .unwrap_or(level_ft);
        crate::circuit::transmit(crate::circuit::radio_call(&callsign, &transit_words(alt)));
    });
}

/// The transit request's words, the level to the hundred: "helicopter, two
/// thousand two hundred feet, request zone transit". Pure.
pub fn transit_words(alt_ft: f64) -> String {
    format!(
        "helicopter, {} feet, request zone transit",
        crate::atis::spoken_alt(alt_ft)
    )
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
    if ACTIVE.compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst).is_err() {
        return;
    }
    let generation = GENERATION.fetch_add(1, Ordering::SeqCst) + 1;
    let base_url = config.xplane_base_url.clone();
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            if GENERATION.load(Ordering::SeqCst) == generation {
                ACTIVE.store(false, Ordering::SeqCst);
            }
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
                generation,
            )
            .await
        });
        // A retired run must not clear a newer route's flag. That race ran
        // two copies through the hover-to-cruise handoff, where one treated
        // the other's expected hover release as an external handover.
        if GENERATION.load(Ordering::SeqCst) == generation {
            ACTIVE.store(false, Ordering::SeqCst);
        }
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
    /// Too fast for the pad (#191): straight ahead on the course the
    /// slow-down was flying, level clear of the ground ahead, slowing to the
    /// hand-over speed, then the hover loop turns back and flies the final.
    /// Never a chase of the pad's bearing at speed (it orbited 26 minutes).
    Around,
    Arrive,
}

/// A go-around from a slow-down that could not make the pad.
struct Around {
    since: std::time::Instant,
    /// The course held, true and on the dial (magnetic).
    course_true: f64,
    bug: f64,
    /// The pad's distance where it began, m.
    from_pad_m: f64,
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
    mut route: Route,
    generation: u64,
) -> bool {
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    let _sortie = crate::full_auto::SortieGuard::begin(
        "heli_route",
        serde_json::json!({ "points": route.points.len() }),
    );
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
    let mut pts: Vec<(String, (f64, f64))> = route
        .points
        .iter()
        .map(|(id, la, lo)| (id.clone(), geo.to_m(origin, (*la, *lo))))
        .collect();
    let Some(mut pad) = pts.last().map(|p| p.1) else {
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
    // The approach's geometry (#191), fixed at its start from the gate and
    // the groundspeed then; re-made at the same speed when the gate comes down.
    let mut path: Option<ApproachPath> = None;
    let mut approach_kts = 0.0_f64;
    let mut slow_law: Option<SlowLaw> = None;
    let mut slow_want = f64::INFINITY;
    let mut around: Option<Around> = None;
    let mut level_ft = cruise_alt_ft;
    let mut vs_written: Option<f64> = None;
    crate::blackbox::record(
        "heli_route",
        serde_json::json!({ "event": "start", "points": route.points.iter().map(|p| &p.0).collect::<Vec<_>>(), "cruise_alt_ft": cruise_alt_ft, "terrain_ft": terrain_ft, "samples": samples.len() }),
    );

    publish_fms(&xp, &route, cruise_alt_ft).await;
    if !route.arrive_only {
        fms_goto(&xp, &route, 1).await;
    }
    if !route.arrive_only {
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
    }
    let mut phase = if route.resume { Phase::Cruise } else { Phase::Depart };
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
    let mut slow_tail_said = false;
    if route.resume {
        let c = xp.get_float_array(&collective).await.ok()
            .and_then(|v| v.first().copied()).unwrap_or(0.45);
        if !hover::cruise_modes(&xp).await {
            say(&voice, Priority::Warning, "The autopilot won't take heading and altitude. You still have control.");
            return false;
        }
        law = Some(SpeedLaw::from(c, s0.ias));
        level_ft = alt0;
        crate::blackbox::record("heli_route", serde_json::json!({
            "event": "resume", "ias": s0.ias, "agl": s0.agl_ft,
            "points": route.points.len()
        }));
    }
    // The inbound call went out (at the approach, or as the answer to the
    // destination's challenge): the approach does not make it twice.
    let mut inbound_made = false;
    // Answers given each station's zone challenge, by frequency.
    let mut answered: std::collections::HashMap<i64, u32> = std::collections::HashMap::new();
    // The landing at a towered field (#192): the runway the spot lands on
    // (moved to the tower's), the final report and clearance watch, the
    // landing readback made.
    INBOUND_SENT.store(false, Ordering::SeqCst);
    let mut pad_rwy: Option<String> = if route.on_pad {
        None
    } else {
        route.points.last().and_then(|p| spot_runway(&p.0))
    };
    let mut clearance = ClearanceWatch::default();
    let mut readback_sent: Option<String> = None;
    let mut hold_said = false;
    let t0 = std::time::Instant::now();
    let mut last = std::time::Instant::now();
    let mut tick_n = 0u64;
    if route.arrive_only {
        // The tower is called first (it arms the station and tunes COM1),
        // so the final report below has someone to clear it.
        if let Some(khz) = route.dest_tower_khz {
            inbound_made = true;
            call_inbound(&base_url, khz);
        }
        // Straight to the final, from where the aircraft is: the hover loop
        // takes (or keeps) it at its height and speed, aimed at the pad.
        let goal = Goal::Fly {
            height_ft: s0.agl_ft,
            speed_mps: s0.gs,
            track_deg: bearing((0.0, 0.0), pad),
        };
        if hover::ACTIVE.load(Ordering::SeqCst) {
            hover::retarget(goal);
        } else {
            hover::request(base_url.clone(), voice.clone(), collective.clone(), goal);
        }
        final_from = Some((s0.agl_ft, s0.gs));
        phase = Phase::Arrive;
        crate::blackbox::record(
            "heli_route",
            serde_json::json!({ "event": "arrive_only", "to_pad_m": dist((0.0, 0.0), pad), "agl": s0.agl_ft }),
        );
        say(&voice, Priority::Readback, "Final for the pad.");
        tokio::time::sleep(Duration::from_millis(1500)).await;
    }
    while ACTIVE.load(Ordering::SeqCst) && GENERATION.load(Ordering::SeqCst) == generation {
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
        // The destination's tower, once it is the station and working us
        // (the inbound call answered, or the landing cleared), names the
        // runway: a runway landing moves to that end's spot (#192).
        let tower_is_dest = crate::mode::atc_enabled()
            && route.dest_tower_khz.is_some()
            && crate::party_line::home_tower_khz() == route.dest_tower_khz;
        let book = crate::party_line::clearance_state();
        if tower_is_dest
            && !route.on_pad
            && matches!(book, Some(crate::tower::State::Inbound | crate::tower::State::ClearedToLand))
        {
            let named = crate::party_line::clearance_runway();
            if let Some((id, la, lo)) =
                spot_for_runway(&route.runway_spots, pad_rwy.as_deref(), named.as_deref())
            {
                let from = pts.last().map(|p| p.0.clone());
                let name = format!("{} runway {id}", route.destination);
                let to_m = geo.to_m(origin, (la, lo));
                if let Some(last) = pts.last_mut() {
                    *last = (name.clone(), to_m);
                }
                if let Some(last) = route.points.last_mut() {
                    *last = (name.clone(), la, lo);
                }
                pad = to_m;
                crate::blackbox::record(
                    "heli_route",
                    serde_json::json!({
                        "event": "pad_to_tower_runway", "from": from, "to": name, "rwy": id,
                        "was_rwy": pad_rwy, "phase": format!("{phase:?}"),
                        "to_pad_m": dist(here, pad), "book": format!("{book:?}"),
                    }),
                );
                say(
                    &voice,
                    Priority::Readback,
                    format!(
                        "Tower wants runway {}. Landing on it.",
                        crate::circuit::spell_runway(&id)
                    ),
                );
                pad_rwy = Some(id);
            }
        }
        // Sequence: the fix is passed when inside the turn's lead, or when
        // it is behind us (a wide turn that never got close).
        while leg + 1 < pts.len() {
            let to = pts[leg].1;
            let next = pts[leg + 1].1;
            let turn = crate::angles::signed_diff_deg(bearing(to, next), bearing(here, to));
            let lead = (turn_radius_m(s.gs.max(10.0))
                * (turn.abs().min(120.0) / 2.0).to_radians().tan())
            .max(300.0);
            let behind =
                crate::angles::abs_diff_deg(bearing(here, to), s.psi) > 100.0 && dist(here, to) < 3000.0;
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
        let variation = crate::angles::signed_diff_deg(s.psi, s.mag);

        // The radio is worked whatever the phase: the handoffs arrive in the
        // climb (the tower's) and anywhere enroute (the region's).
        obey_handoffs(&base_url, &voice, cruise_alt_ft);
        // A tower asking who we are: the destination's gets the inbound
        // call, any other field's the transit request. Drained every pass
        // whoever asked (#83: EDVW-ETND 2026-09-23 crossed zones with the
        // question unanswered).
        let challenged = crate::party_line::take_zone_challenge();
        if challenged && crate::mode::atc_enabled() {
            let home = crate::party_line::home_tower_khz();
            let n = home.map_or(0, |k| *answered.get(&k).unwrap_or(&0));
            let answer = challenge_answer(true, home, route.dest_tower_khz, n);
            crate::blackbox::record(
                "heli_route",
                serde_json::json!({
                    "event": "zone_challenge", "home_khz": home, "dest_khz": route.dest_tower_khz,
                    "answer": answer.map(|a| format!("{a:?}")), "answered_before": n,
                    "phase": format!("{phase:?}"),
                }),
            );
            if let (Some(k), Some(a)) = (home, answer) {
                *answered.entry(k).or_insert(0) += 1;
                match a {
                    ChallengeAnswer::Inbound(khz) => {
                        inbound_made = true;
                        call_inbound(&base_url, khz);
                    }
                    ChallengeAnswer::Transit => call_transit(&base_url, level_ft),
                }
            }
        }
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
            Phase::Cruise | Phase::Approach | Phase::Slow | Phase::Around => {
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
                // A go-around holds the course it began on (written once at
                // its start): chasing the pad's bearing at speed is the orbit.
                // Through the one turn-safe steer (`heading_bug::bug_toward`,
                // audit #6): a new leg's course on the far side of a turn in
                // progress waits on the nose, a big one is led from level by
                // at most 25 degrees; the 2 degree hysteresis below still
                // decides when the knob moves.
                let want_bug = crate::heading_bug::bug_toward(
                    bug_set.unwrap_or(s.mag),
                    (track - variation).rem_euclid(360.0),
                    s.mag,
                    s.phi,
                    360.0,
                );
                let bug = match bug_set {
                    _ if around.is_some() => around.as_ref().map_or(0.0, |a| a.bug),
                    Some(b) if crate::angles::abs_diff_deg(want_bug, b) < BUG_HYSTERESIS_DEG => b,
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
                let gs_kts = s.gs * 1.943_84;
                if phase == Phase::Cruise {
                    match pad_ft {
                        Some(pad)
                            if remaining
                                < ApproachPath::new(GATE_FT, gs_kts).start_m(cruise_alt_ft, pad) =>
                        {
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
                            approach_kts = gs_kts;
                            let p = ApproachPath::new(g, gs_kts);
                            path = Some(p);
                            gate_leg = Some(leg);
                            level_ft = pad + g;
                            phase = Phase::Approach;
                            crate::blackbox::record(
                                "heli_route",
                                serde_json::json!({ "event": "approach", "to_pad_m": to_pad, "remaining_m": remaining, "pad_ft": pad, "floor_ft": floor, "gate_agl": g, "ias": s.ias, "gs_kts": gs_kts, "slow_m": p.slow_m, "path_deg": p.tan.atan().to_degrees() }),
                            );
                            say(
                                &voice,
                                Priority::Readback,
                                format!("Starting down for {}.", route.destination),
                            );
                            if let Some(khz) = route.dest_tower_khz.filter(|_| !inbound_made) {
                                inbound_made = true;
                                call_inbound(&base_url, khz);
                            }
                        }
                        None if remaining < slow_start_m(gs_kts.max(cruise_speed), depart_agl_ft) => {
                            phase = Phase::Slow;
                            slow_law = Some(SlowLaw::from(law.as_ref().map_or(0.45, |l| l.cmd())));
                            slow_want = s.ias;
                            crate::blackbox::record(
                                "heli_route",
                                serde_json::json!({ "event": "slow", "to_pad_m": to_pad, "ias": s.ias, "gs_kts": gs_kts, "approach": false }),
                            );
                            say(
                                &voice,
                                Priority::Readback,
                                format!("Slowing for {}.", route.destination),
                            );
                            if let Some(khz) = route.dest_tower_khz.filter(|_| !inbound_made) {
                                inbound_made = true;
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
                                path = Some(ApproachPath::new(ng, approach_kts));
                                level_ft = pad + ng;
                                crate::blackbox::record(
                                    "heli_route",
                                    serde_json::json!({ "event": "gate_lowered", "from": g, "to": ng, "leg": leg }),
                                );
                            }
                        }
                    }
                }
                // The slow-down starts where the path meets the gate, or
                // sooner if the speed flown now needs more room than the
                // approach was planned for (#191: it began 2.16 km out at
                // 117 kt on a plan made for 70).
                if phase == Phase::Approach {
                    if let (Some((_, g)), Some(p)) = (gate, path) {
                        let need_m = final_len_m(g) + slow_down_m(gs_kts, HANDOVER_KTS) + SLOW_MARGIN_M;
                        if remaining < p.slow_m.max(need_m) {
                            phase = Phase::Slow;
                            slow_law = Some(SlowLaw::from(law.as_ref().map_or(0.45, |l| l.cmd())));
                            slow_want = s.ias;
                            crate::blackbox::record(
                                "heli_route",
                                serde_json::json!({ "event": "slow", "to_pad_m": to_pad, "remaining_m": remaining, "ias": s.ias, "gs_kts": gs_kts, "agl": s.agl_ft, "approach": true, "slow_m": p.slow_m, "need_m": need_m }),
                            );
                            say(&voice, Priority::Readback, "Slowing for the pad.");
                        }
                    }
                }
                // A slow-down that cannot make the pad goes around, once:
                // straight ahead on the course it flies, the level raised
                // clear of the ground ahead, slowing to the hand-over, and
                // the hover loop brings it back (#191: the pad's bearing
                // chased at 88 kt was a 26 minute orbit at 79).
                if phase == Phase::Slow {
                    let final_m = gate.map_or(final_len_m(depart_agl_ft), |(_, g)| final_len_m(g));
                    let behind = leg + 1 == pts.len()
                        && to_pad < 3000.0
                        && crate::angles::abs_diff_deg(bearing(here, pad), s.psi) > 90.0;
                    let check = slow_check(gs_kts, s.ias, remaining, final_m, behind);
                    if check != SlowCheck::OnTrack {
                        let course = s.psi;
                        let end = crate::nav::project_latlon(s.lat, s.lon, course, AROUND_PROBE_M / 1852.0);
                        let ground = crate::terrain::max_elevation_ft(
                            &xp,
                            &sample_corridor(&[(s.lat, s.lon), end], 500.0, 500.0),
                        )
                        .await;
                        let floor = ground.map(|f| f + ind_offset_ft.unwrap_or(0.0));
                        let from = level_ft;
                        if let Some(f) = floor {
                            level_ft = level_ft.max(((f + APPROACH_CLEAR_FT) / 100.0).ceil() * 100.0);
                        }
                        let b = (course - variation).rem_euclid(360.0).round().rem_euclid(360.0);
                        let _ = xp
                            .set_float("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot", b)
                            .await;
                        bug_set = Some(b);
                        around = Some(Around {
                            since: std::time::Instant::now(),
                            course_true: course,
                            bug: b,
                            from_pad_m: to_pad,
                        });
                        slow_want = slow_want.min(s.ias);
                        phase = Phase::Around;
                        let (why, need) = match check {
                            SlowCheck::Overflown => ("overflown", None),
                            SlowCheck::TooFast { need } => ("too_fast", need.is_finite().then_some(need)),
                            SlowCheck::OnTrack => ("", None),
                        };
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({
                                "event": "go_around", "why": why, "need_kts_s": need, "ias": s.ias,
                                "gs_kts": gs_kts, "to_pad_m": to_pad, "remaining_m": remaining,
                                "final_m": final_m, "course": course, "bug": b, "level_from": from,
                                "level_ft": level_ft, "floor_ft": floor, "agl": s.agl_ft,
                            }),
                        );
                        say(
                            &voice,
                            Priority::Caution,
                            "Too fast for the pad. Going around: straight ahead, slowing, then back to it on the hover.",
                        );
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
                // A go-around holds its level (the gate, or higher for the
                // ground ahead), on the same mode.
                let vplan = match (phase, gate, path) {
                    (Phase::Around, _, _) => Some((level_ft, level_ft, level_ft)),
                    (Phase::Approach | Phase::Slow, Some((pad, g)), Some(p)) => Some((
                        p.target_ft(remaining, pad, cruise_alt_ft),
                        p.target_ft((remaining - s.gs * 10.0).max(0.0), pad, cruise_alt_ft),
                        pad + g,
                    )),
                    _ => None,
                };
                if let (Some((target, ahead, dial)), Ok(alt)) = (vplan, alt_now.as_ref().copied()) {
                    let vs = approach_vs_fpm(alt, target, ahead);
                    let engaged = vvi_mode.is_some_and(|v| v > 1.5);
                    let off_level = (vs <= -100.0 && alt > dial + 60.0) || (vs >= 100.0 && alt < dial - 60.0);
                    if !engaged && off_level {
                        descend_on_vs(&xp, dial, vs).await;
                        vs_written = Some(vs);
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({ "event": "descent_vs", "alt": alt, "target": target, "vs": vs, "dial": dial.round() }),
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
                // The slow-down flies its schedule (#191): the groundspeed
                // that reaches the hand-over speed where the final begins at
                // the planned deceleration, as an airspeed, never rising; a
                // go-around brings it down at the planned rate from where it
                // began.
                let final_m = gate.map_or(final_len_m(depart_agl_ft), |(_, g)| final_len_m(g));
                let handover_want = slow_target_kts(s.ias, gs_kts);
                let (want, decel) = match phase {
                    Phase::Slow => {
                        slow_want = slow_want_kts(remaining - final_m, s.ias, gs_kts, slow_want);
                        let d = if slow_want > handover_want + 0.5 {
                            SLOW_DECEL_KTS_S
                        } else {
                            0.0
                        };
                        (slow_want, d)
                    }
                    Phase::Around => {
                        slow_want = (slow_want - SLOW_DECEL_KTS_S * dt.clamp(0.0, 1.0)).max(handover_want);
                        let d = if slow_want > handover_want + 0.5 {
                            SLOW_DECEL_KTS_S
                        } else {
                            0.0
                        };
                        (slow_want, d)
                    }
                    Phase::Approach => (APPROACH_KTS, 0.0),
                    _ if climbing => (CLIMB_KTS, 0.0),
                    _ => (cruise_speed, 0.0),
                };
                if phase == Phase::Slow && handover_want < HANDOVER_KTS - 0.5 && !slow_tail_said {
                    slow_tail_said = true;
                    crate::blackbox::record(
                        "heli_route",
                        serde_json::json!({ "event": "slow_tailwind", "ias": s.ias, "gs_kts": gs_kts, "target_kts": handover_want }),
                    );
                }
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
                // The slow-down's lever is the slow law's (down to the
                // cruise floor, the autopilot's nose-up doing the slowing);
                // the approach keeps the speed law on its 0.35 floor.
                let slowing = matches!(phase, Phase::Slow | Phase::Around);
                if let Some(l) = law.as_mut() {
                    l.set_floor(if phase == Phase::Approach {
                        APPROACH_COLLECTIVE_MIN
                    } else {
                        COLLECTIVE_MIN
                    });
                    let (c, target) = if slowing {
                        let sl = slow_law.get_or_insert_with(|| SlowLaw::from(l.cmd()));
                        (sl.step(want, decel, s.ias, s.theta, dt), want)
                    } else {
                        (l.step(want, s.ias, dt), l.target())
                    };
                    let _ = xp.set_float_index(&collective, 0, c).await;
                    if tick_n % 4 == 0 {
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({ "event": "tick", "phase": format!("{phase:?}"), "fix": ident, "d_fix_m": dist(here, to), "to_pad_m": to_pad, "remaining_m": remaining, "ias": s.ias, "gs_kts": gs_kts, "target": target, "decel": decel, "collective": c, "agl": s.agl_ft, "bug": bug, "hdg": s.mag, "theta": s.theta, "phi": s.phi, "alt_hold": alt_hold, "vvi_mode": vvi_mode, "alt": alt_now.as_ref().ok(), "target_alt": target_alt, "vs_cmd": vs_cmd }),
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
                if slowing
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
                    let from = format!("{phase:?}");
                    phase = Phase::Arrive;
                    crate::blackbox::record(
                        "heli_route",
                        serde_json::json!({ "event": "handover_hover", "to_pad_m": to_pad, "ias": s.ias, "gs": s.gs, "agl": s.agl_ft, "from": from, "theta": s.theta }),
                    );
                    say(&voice, Priority::Readback, "Final for the pad.");
                    // The loop's own start takes a second; let it.
                    tokio::time::sleep(Duration::from_millis(1500)).await;
                } else if let Some(a) = around.as_ref().filter(|_| phase == Phase::Around) {
                    // Bounded: not slowed in AROUND_MAX_S, or this far from
                    // the pad, the aircraft is given back level on heading
                    // and altitude hold, the lever at the level one for its
                    // speed (a steady cruise, never a chase).
                    let secs = a.since.elapsed().as_secs_f64();
                    if secs > AROUND_MAX_S || to_pad > AROUND_MAX_M {
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({ "event": "go_around_failed", "secs": secs, "to_pad_m": to_pad, "from_pad_m": a.from_pad_m, "ias": s.ias, "gs_kts": gs_kts, "course": a.course_true }),
                        );
                        record_end(&voice, "slow_failed", "around");
                        let lever = cruise_level_collective(s.ias.max(APPROACH_KTS));
                        let _ = xp.set_float_index(&collective, 0, lever).await;
                        let _ = hover::cruise_modes(&xp).await;
                        say(
                            &voice,
                            Priority::Warning,
                            "I couldn't slow for the pad. Route stopped, autopilot on heading and altitude, your controls.",
                        );
                        return false;
                    }
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
                // The final's speed cap is the hover mode's GROUNDSPEED range
                // and an airspeed ceiling through the wind at the aircraft,
                // never the groundspeed a headwind left at the hand-over. A
                // failed wind read keeps the old cap.
                let track_pad = bearing(here, pad);
                let (wdir, wspd) = tokio::join!(
                    xp.get_float(crate::datarefs::WIND_DIR_DEG_TRUE),
                    xp.get_float(crate::datarefs::WIND_SPEED_MSC),
                );
                let wind = match (wdir, wspd) {
                    (Ok(d), Ok(v)) => Some(wind_components(track_pad, d, v)),
                    _ => None,
                };
                let cap = final_cap_mps(from_mps, wind);
                // The landing at a towered field is the aeroplanes' (#192):
                // the final reported, the clearance read back, and never a
                // landing without one; a final not cleared holds in the
                // hover short of the pad and asks again, bounded (LXGB,
                // iMac 2026-09-30, 1790722408-heli_route: "Report final"
                // never answered, set down on 27 without a clearance).
                let needs = crate::mode::atc_enabled() && route.dest_tower_khz.is_some() && inbound_made;
                let tower_ready = tower_is_dest && INBOUND_SENT.load(Ordering::SeqCst);
                let cleared = tower_ready && book == Some(crate::tower::State::ClearedToLand);
                let landing_ok = !needs || cleared;
                if needs && tower_ready {
                    match crate::party_line::readback_owed() {
                        None => readback_sent = None,
                        Some(owed)
                            if crate::autoland::landing_readback(&owed)
                                && readback_sent.as_deref() != Some(owed.as_str()) =>
                        {
                            readback_sent = Some(owed.clone());
                            let cs = crate::circuit::crew_callsign(&xp).await;
                            let rwy = crate::party_line::clearance_runway();
                            let words = match rwy.as_deref() {
                                Some(r) if !route.on_pad => {
                                    format!("cleared to land runway {}", crate::circuit::spell_runway(r))
                                }
                                _ => "cleared to land".to_string(),
                            };
                            crate::circuit::transmit_readback(crate::circuit::radio_call(&cs, &words), false);
                            crate::blackbox::record(
                                "heli_route",
                                serde_json::json!({ "event": "landing_readback", "rwy": rwy, "to_pad_m": to_pad }),
                            );
                        }
                        Some(_) => {}
                    }
                }
                let held = !landing_ok && to_pad <= HOLD_SHORT_M + HOLD_SLACK_M && s.gs < HELD_GS_MPS;
                match clearance.step(t0.elapsed().as_secs_f64(), needs, tower_ready, cleared, held) {
                    Some(step @ (ClearanceStep::ReportFinal | ClearanceStep::Ask)) => {
                        let asking = step == ClearanceStep::Ask;
                        let cs = crate::circuit::crew_callsign(&xp).await;
                        crate::circuit::transmit(crate::circuit::radio_call(
                            &cs,
                            &final_words(pad_rwy.as_deref(), asking),
                        ));
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({
                                "event": if asking { "landing_clearance_asked" } else { "final_report" },
                                "rwy": pad_rwy, "to_pad_m": to_pad, "agl": s.agl_ft, "gs": s.gs,
                                "asks": clearance.asks(), "book": format!("{book:?}"),
                            }),
                        );
                    }
                    Some(ClearanceStep::GiveUp) => {
                        hover::retarget(Goal::Hover {
                            height_ft: s.agl_ft.max(FINAL_HOVER_FT),
                        });
                        crate::blackbox::record(
                            "heli_route",
                            serde_json::json!({
                                "event": "uncleared_hold_given_up", "to_pad_m": to_pad, "agl": s.agl_ft,
                                "asks": clearance.asks(), "tower_ready": tower_ready,
                                "held_s": clearance.held_since_s().map(|h| t0.elapsed().as_secs_f64() - h),
                                "book": format!("{book:?}"),
                            }),
                        );
                        record_end(&voice, "uncleared", "arrive");
                        say(
                            &voice,
                            Priority::Caution,
                            "No landing clearance from the tower. Holding in the hover short of the pad, your controls.",
                        );
                        return false;
                    }
                    None => {}
                }
                if held && !hold_said {
                    hold_said = true;
                    crate::blackbox::record(
                        "heli_route",
                        serde_json::json!({
                            "event": "hold_short", "to_pad_m": to_pad, "agl": s.agl_ft,
                            "tower_ready": tower_ready, "book": format!("{book:?}"),
                        }),
                    );
                    say(
                        &voice,
                        Priority::Readback,
                        "No landing clearance yet. Holding short of the pad.",
                    );
                }
                let (speed, height) = if landing_ok {
                    final_command(to_pad, from_ft, cap)
                } else {
                    hold_command(to_pad, from_ft, cap)
                };
                if tick_n % 4 == 0 {
                    crate::blackbox::record(
                        "heli_route",
                        serde_json::json!({
                            "event": "final_tick", "to_pad_m": to_pad, "cmd_mps": speed,
                            "cap_mps": cap, "from_mps": from_mps, "height_ft": height,
                            "agl": s.agl_ft, "ias": s.ias, "gs": s.gs,
                            "head_mps": wind.map(|w| w.0), "cross_mps": wind.map(|w| w.1), "cleared": landing_ok,
                        }),
                    );
                }
                if landing_ok && (to_pad < ARRIVED_M || speed < 1.0) {
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
                        .get_float(crate::datarefs::ON_GROUND_ANY)
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
            Phase::Around => format!("going around, pad {:.1} nm", to_pad / 1852.0),
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

    #[test]
    fn an_airborne_resume_joins_the_forward_end_of_the_leg() {
        let p = vec![
            ("A".into(), 50.0, 0.0),
            ("B".into(), 50.0, 1.0),
            ("C".into(), 50.0, 2.0),
        ];
        assert_eq!(resume_index(&p, 50.01, 0.6), Ok(1));
        assert_eq!(resume_index(&p, 50.01, 1.6), Ok(2));
        assert!(resume_index(&p, 50.2, 0.6).unwrap_err()
            .contains("outside the 3 mile rejoin corridor"));
    }

    /// The S-76 in the cruise modes, fitted to 1790715215-heli_route (LEMG-
    /// LXGB, iMac 2026-09-29): the autopilot holds the vertical speed on the
    /// pitch (the dial through a second of dead time and two 1.5 s lags), so
    /// the collective is energy: `dV/dt = (K (c - level(V)) - g vz) / V`, the
    /// level lever [`CRUISE_POWER_CURVE`] (shifted by `off`), `K`
    /// [`COLLECTIVE_POWER_WKG`] (scaled by `k`), the lever through a 0.5 s
    /// lag. The attitude is the trim's (-1.1 degrees at 125 kt, 3 at 78,
    /// as flown) plus the deceleration's tilt plus 0.4 of the path angle
    /// (-2.3 degrees in the 1,200 fpm descent at 115 kt, as flown). `wind`
    /// is along the track, kt (a tailwind positive).
    struct S76 {
        v: f64,
        alt: f64,
        c: f64,
        vz: f64,
        v1: f64,
        q: std::collections::VecDeque<f64>,
        k: f64,
        off: f64,
        acc: f64,
    }

    impl S76 {
        fn new(v: f64, alt: f64, c: f64, vz: f64, k: f64, off: f64) -> Self {
            Self {
                v,
                alt,
                c,
                vz,
                v1: vz,
                q: std::collections::VecDeque::from(vec![vz; 10]),
                k,
                off,
                acc: 0.0,
            }
        }
        /// 0.1 s of flight on a lever command and a vertical-speed dial.
        fn step(&mut self, cmd: f64, dial_fpm: f64) {
            let dt = 0.1;
            self.c += (cmd - self.c) * dt / 0.5;
            self.q.push_back(dial_fpm);
            let d = self.q.pop_front().unwrap();
            self.v1 += (d - self.v1) * dt / 1.5;
            self.vz += (self.v1 - self.vz) * dt / 1.5;
            let vm = (self.v * 0.514_444).max(8.0);
            let level = cruise_level_collective(self.v) + self.off;
            self.acc = (COLLECTIVE_POWER_WKG * self.k * (self.c - level) - 9.81 * self.vz * 0.00508) / vm;
            self.v += self.acc / 0.514_444 * dt;
            self.alt += self.vz / 60.0 * dt;
        }
        fn theta(&self) -> f64 {
            let trim = 3.0 - 4.0 * (self.v - 78.0) / 47.0;
            let gamma = (self.vz * 0.00508).atan2((self.v * 0.514_444).max(8.0)).to_degrees();
            trim + (-self.acc / 9.81).atan().to_degrees() + 0.4 * gamma
        }
    }

    /// #191, the plant against the record: the slow-down of
    /// 1790715215-heli_route replayed from its start (2,161 m from the pad,
    /// 117 KIAS, 810 ft on the altimeter over a 503 ft gate, the lever on
    /// the 0.35 floor, 600 fpm down) with the old law (the speed law on
    /// APPROACH_COLLECTIVE_MIN toward the 37 kt hand-over target) flies the
    /// recorded airspeeds within 1.5 kt for 95 s: 94 kt at the pad, then the
    /// floor's own level speed near 78, where it orbited for 26 minutes.
    /// The hand-over (under 40 kt) never comes.
    #[test]
    fn the_plant_replays_the_recorded_overshoot() {
        let recorded = [
            (5.5, 112.6),
            (11.1, 108.5),
            (22.1, 102.3),
            (33.1, 97.1),
            (38.5, 94.9),
            (52.7, 89.3),
            (72.3, 83.2),
            (94.4, 79.9),
        ];
        let mut p = S76::new(117.0, 810.0, 0.35, -600.0, 1.0, 0.0);
        let mut law = SpeedLaw::from(0.35, 67.8);
        law.set_floor(APPROACH_COLLECTIVE_MIN);
        let (mut d, mut t) = (2161.0_f64, 0.0_f64);
        let mut at_pad = None;
        let mut next = 0;
        let mut worst: f64 = 0.0;
        while t < 300.0 {
            let c = law.step(37.3, p.v, 0.5);
            let dial = approach_vs_fpm(p.alt, 503.0, 503.0);
            for _ in 0..5 {
                p.step(c, dial);
                d -= p.v * 0.514_444 * 0.1;
                t += 0.1;
                if at_pad.is_none() && d <= 0.0 {
                    at_pad = Some(p.v);
                }
                if next < recorded.len() && t >= recorded[next].0 {
                    worst = worst.max((p.v - recorded[next].1).abs());
                    next += 1;
                }
            }
            assert!(p.v > HANDOVER_KTS + 2.0, "the old law never slows to the hand-over");
        }
        let at_pad = at_pad.unwrap();
        eprintln!("replay: {at_pad:.1} kt at the pad (recorded 94.9 at 81 m), {:.1} kt at 300 s, worst {worst:.2} kt off", p.v);
        assert!(worst < 1.5, "the plant flies the record: worst {worst:.2} kt");
        assert!((90.0..98.0).contains(&at_pad), "{at_pad:.1} kt at the pad");
        assert!((76.0..81.0).contains(&p.v), "the floor's level speed: {:.1}", p.v);
    }

    struct Arrival {
        /// (distance from the pad, KIAS, altitude) at the hover loop's takeover.
        hand: Option<(f64, f64, f64)>,
        final_m: f64,
        slow_at: (f64, f64),
        worst_theta: f64,
        lever: (f64, f64),
        lowest_ft: f64,
        worst_sink: f64,
        went_around: Option<SlowCheck>,
    }

    /// The arrival as the route flies it, on the plant: the approach from its
    /// start (the path, the speed law on its 0.35 floor), the slow-down where
    /// the loop starts it, its schedule and the slow law, the go-around
    /// check, the takeover when slowed. Every half second, as the loop.
    fn fly_arrival(v0: f64, cruise_agl: f64, gate_agl: f64, k: f64, off: f64, wind: f64) -> Arrival {
        let pad = 3.0;
        let cruise = pad + cruise_agl;
        let gs0 = v0 + wind;
        let start = ApproachPath::new(GATE_FT, gs0).start_m(cruise, pad);
        let path = ApproachPath::new(gate_agl, gs0);
        let final_m = final_len_m(gate_agl);
        let c0 = cruise_level_collective(v0) + off;
        let mut p = S76::new(v0, cruise, c0, 0.0, k, off);
        let mut law = SpeedLaw::from(c0, v0);
        law.set_floor(APPROACH_COLLECTIVE_MIN);
        let mut slow: Option<SlowLaw> = None;
        let mut want = f64::INFINITY;
        let mut d = start;
        let mut out = Arrival {
            hand: None,
            final_m,
            slow_at: (0.0, 0.0),
            worst_theta: f64::MIN,
            lever: (1.0, 0.0),
            lowest_ft: f64::MAX,
            worst_sink: 0.0,
            went_around: None,
        };
        for _ in 0..4000 {
            let gs = p.v + wind;
            if slow.is_none() {
                let need = final_m + slow_down_m(gs, HANDOVER_KTS) + SLOW_MARGIN_M;
                if d < path.slow_m.max(need) {
                    slow = Some(SlowLaw::from(law.cmd()));
                    want = p.v;
                    out.slow_at = (d, p.alt);
                }
            }
            let target = path.target_ft(d, pad, cruise);
            let ahead = path.target_ft((d - gs * 0.514_444 * 10.0).max(0.0), pad, cruise);
            let dial = approach_vs_fpm(p.alt, target, ahead);
            out.worst_sink = out.worst_sink.max(-dial);
            let c = match slow.as_mut() {
                None => law.step(APPROACH_KTS, p.v, 0.5),
                Some(sl) => {
                    let check = slow_check(gs, p.v, d, final_m, d < 0.0);
                    if check != SlowCheck::OnTrack && out.went_around.is_none() {
                        out.went_around = Some(check);
                    }
                    want = slow_want_kts(d - final_m, p.v, gs, want);
                    let decel = if want > slow_target_kts(p.v, gs) + 0.5 {
                        SLOW_DECEL_KTS_S
                    } else {
                        0.0
                    };
                    if p.v < HANDOVER_KTS + 2.0 && gs * 0.514_444 < hover::TAKEOVER_MAX_MPS {
                        out.hand = Some((d, p.v, p.alt));
                        break;
                    }
                    let c = sl.step(want, decel, p.v, p.theta(), 0.5);
                    out.lever = (out.lever.0.min(c), out.lever.1.max(c));
                    c
                }
            };
            for _ in 0..5 {
                p.step(c, dial);
                d -= (p.v + wind) * 0.514_444 * 0.1;
                out.lowest_ft = out.lowest_ft.min(p.alt);
                if slow.is_some() {
                    out.worst_theta = out.worst_theta.max(p.theta());
                }
            }
        }
        out
    }

    /// #191, the fix on the fitted plant: from the recorded cruise (4,180 ft,
    /// 125 kt, the pad at sea level, the 500 ft gate) and from a 1,000 and a
    /// 2,000 ft cruise with a ridge that holds the gate at 900, the approach
    /// reaches the gate by the time the slow-down starts, the slow-down
    /// flies its schedule down to the hand-over speed where the final
    /// begins, level at the gate, the nose never past the guard, the lever
    /// never under the cruise floor, and the go-around never needed; in a
    /// 10 kt headwind and a 6 kt tailwind (the most the hand-over's 30 kt
    /// floor, [`SLOW_MIN_KTS`], brings inside the takeover's groundspeed). The old slow-down handed over
    /// nowhere (`the_plant_replays_the_recorded_overshoot`).
    #[test]
    fn the_slowdown_reaches_the_takeover_speed_where_the_final_begins() {
        let mut bad = Vec::new();
        for (v0, cruise_agl, gate) in [(125.0, 4177.0, GATE_FT), (120.0, 1000.0, GATE_FT), (120.0, 2000.0, 900.0)] {
            for wind in [-10.0, 0.0, 6.0] {
                let r = fly_arrival(v0, cruise_agl, gate, 1.0, 0.0, wind);
                let tag = format!("{cruise_agl} ft AGL, gate {gate}, wind {wind}");
                let Some((d, v, alt)) = r.hand else {
                    bad.push(format!("{tag}: no takeover"));
                    continue;
                };
                eprintln!(
                    "{tag}: slow-down at {:.0} m ({:.0} ft), takeover {d:.0} m out at {v:.1} kt ({alt:.0} ft), final {:.0} m, nose {:.1}, lever {:.2}..{:.2}",
                    r.slow_at.0, r.slow_at.1, r.final_m, r.worst_theta, r.lever.0, r.lever.1
                );
                if (d - r.final_m).abs() > 400.0 {
                    bad.push(format!("{tag}: takeover {d:.0} m out, the final is {:.0}", r.final_m));
                }
                if (r.slow_at.1 - (3.0 + gate)).abs() > 120.0 {
                    bad.push(format!("{tag}: {:.0} ft at the slow-down, gate {:.0}", r.slow_at.1, 3.0 + gate));
                }
                if r.lowest_ft < 3.0 + gate - 60.0 {
                    bad.push(format!("{tag}: {:.0} ft, under the gate", r.lowest_ft));
                }
                if r.worst_theta > SLOW_PITCH_MAX_DEG + 1.0 {
                    bad.push(format!("{tag}: nose {:.1}", r.worst_theta));
                }
                if r.lever.0 < SLOW_COLLECTIVE_MIN - 1e-9 {
                    bad.push(format!("{tag}: lever {:.3}", r.lever.0));
                }
                if r.worst_sink > APPROACH_MAX_SINK_FPM + 1.0 {
                    bad.push(format!("{tag}: sink {:.0}", r.worst_sink));
                }
                if let Some(g) = r.went_around {
                    bad.push(format!("{tag}: went around {g:?}"));
                }
            }
        }
        assert!(bad.is_empty(), "{bad:#?}");
    }

    /// The same arrival on plants nobody has measured: the power 30 % weaker
    /// or stronger, the level lever 0.04 lower or higher (another weight, a
    /// hot day), each wind. The takeover always comes before the pad, the
    /// nose stays within the guard and a degree or two; where the plan was
    /// too hopeful the go-around check says so rather than fly past.
    #[test]
    fn the_slowdown_holds_on_plants_nobody_has_measured() {
        let mut bad = Vec::new();
        for k in [0.7, 1.0, 1.3] {
            for off in [-0.04, 0.0, 0.04] {
                for wind in [-10.0, 0.0, 6.0] {
                    let r = fly_arrival(125.0, 4177.0, GATE_FT, k, off, wind);
                    let tag = format!("k {k} off {off} wind {wind}");
                    match r.hand {
                        Some((d, _, _)) if d > 0.0 => {}
                        Some((d, _, _)) if r.went_around.is_some() => {
                            eprintln!("{tag}: past the pad {d:.0} m, went around first")
                        }
                        other => bad.push(format!("{tag}: takeover {other:?}")),
                    }
                    if r.worst_theta > SLOW_PITCH_MAX_DEG + 2.0 {
                        bad.push(format!("{tag}: nose {:.1}", r.worst_theta));
                    }
                }
            }
        }
        assert!(bad.is_empty(), "{bad:#?}");
    }

    /// #191, the overshoot is never an orbit. The recorded slow-down began
    /// with 2,251 m to go at 118.9 kt over the ground: 2.1 kt/s needed to
    /// half the final's length where the plan flies 0.7, so it is TOO FAST
    /// at once; the pad behind
    /// at 88 kt is OVERFLOWN; a slowed aircraft is the hover loop's wherever
    /// it is; the new slow-down's own start is on track. Flown from the
    /// recorded overflight (94 kt at the pad, 575 ft), the go-around holds
    /// its course and level and slows at the planned rate to the takeover
    /// inside its bounds, where the old one turned after the pad at 5
    /// degrees a tick and circled for 26 minutes.
    #[test]
    fn a_slowdown_that_cannot_make_the_pad_goes_around() {
        let final_m = final_len_m(GATE_FT);
        match slow_check(118.9, 117.7, 2251.0, final_m, false) {
            SlowCheck::TooFast { need } => assert!(need > 2.0, "{need}"),
            other => panic!("{other:?}"),
        }
        assert_eq!(slow_check(88.0, 88.0, 700.0, final_m, true), SlowCheck::Overflown);
        assert_eq!(slow_check(39.0, 39.0, 50.0, final_m, true), SlowCheck::OnTrack);
        assert_eq!(slow_check(39.0, 39.0, -500.0, final_m, false), SlowCheck::OnTrack);
        let start = ApproachPath::new(GATE_FT, 126.0).slow_m;
        assert_eq!(slow_check(126.0, 125.0, start, final_m, false), SlowCheck::OnTrack);
        assert!(matches!(
            slow_check(80.0, 80.0, final_m * 0.4, final_m, false),
            SlowCheck::TooFast { .. }
        ));
        // The go-around, flown: the target down at the planned rate from the
        // speed it began at, the level held at the gate.
        for (k, off) in [(1.0, 0.0), (0.7, -0.04), (1.3, 0.04)] {
            let mut p = S76::new(94.0, 575.0, 0.35, -150.0, k, off);
            let mut sl = SlowLaw::from(0.35);
            let mut want = p.v;
            let (mut t, mut past) = (0.0_f64, 0.0_f64);
            let mut worst_theta: f64 = f64::MIN;
            let hand = loop {
                if p.v < HANDOVER_KTS + 2.0 && p.v * 0.514_444 < hover::TAKEOVER_MAX_MPS {
                    break Some((t, past, p.v));
                }
                if t > AROUND_MAX_S {
                    break None;
                }
                let floor = slow_target_kts(p.v, p.v);
                want = (want - SLOW_DECEL_KTS_S * 0.5).max(floor);
                let decel = if want > floor + 0.5 { SLOW_DECEL_KTS_S } else { 0.0 };
                let c = sl.step(want, decel, p.v, p.theta(), 0.5);
                let dial = approach_vs_fpm(p.alt, 503.0, 503.0);
                for _ in 0..5 {
                    p.step(c, dial);
                    past += p.v * 0.514_444 * 0.1;
                    worst_theta = worst_theta.max(p.theta());
                }
                t += 0.5;
            };
            let (t, past, v) = hand.unwrap_or_else(|| panic!("k {k} off {off}: not slowed in {AROUND_MAX_S} s"));
            eprintln!("go-around k {k} off {off}: takeover at {v:.1} kt after {t:.0} s, {past:.0} m past the pad, nose {worst_theta:.1}, {:.0} ft", p.alt);
            assert!(past < AROUND_MAX_M, "{past:.0} m");
            assert!(worst_theta < SLOW_PITCH_MAX_DEG + 2.0, "{worst_theta:.1}");
            assert!((p.alt - 503.0).abs() < 80.0, "{:.0} ft", p.alt);
        }
    }

    /// LILO-LILB (2026-09-24, crashed): the speed law on a 0.25 floor at 86
    /// kt drove the lever to 0.65 once the nose-up had the speed at 25 kt,
    /// and the helicopter climbed into a growing cycle. The slow law under
    /// its target never asks more than the level lever and its margin, and
    /// a nose past the guard eases the deceleration.
    #[test]
    fn the_slow_law_never_climbs_away_under_its_target() {
        let mut sl = SlowLaw::from(0.30);
        let mut c = 0.30;
        for _ in 0..200 {
            c = sl.step(HANDOVER_KTS, 0.0, 25.0, 6.0, 0.5);
        }
        assert!(c <= cruise_level_collective(25.0) + 0.1 + 1e-9, "{c}");
        assert!(c < 0.55, "never LILO's 0.65: {c}");
        // Past the guard the lever comes up by the guard's share.
        let mut calm = SlowLaw::from(0.30);
        let mut steep = SlowLaw::from(0.30);
        let (mut a, mut b) = (0.0, 0.0);
        for _ in 0..40 {
            a = calm.step(60.0, SLOW_DECEL_KTS_S, 90.0, 5.0, 0.5);
            b = steep.step(60.0, SLOW_DECEL_KTS_S, 90.0, SLOW_PITCH_MAX_DEG + 3.0, 0.5);
        }
        assert!(b > a + 0.1, "{a:.3} {b:.3}");
        // Never under the cruise floor, never slewed fast.
        let mut hard = SlowLaw::from(0.60);
        let mut last = 0.60;
        for _ in 0..100 {
            let c = hard.step(38.0, SLOW_DECEL_KTS_S, 117.0, 0.0, 0.5);
            assert!((c - last).abs() <= COLLECTIVE_SLEW * 0.5 + 1e-9);
            assert!(c >= SLOW_COLLECTIVE_MIN - 1e-9);
            last = c;
        }
    }

    #[test]
    fn the_approach_path_and_gate() {
        // The gate is 500 ft over the pad, or clears a ridge by 400.
        assert_eq!(gate_agl_ft(100.0, None), GATE_FT);
        assert_eq!(gate_agl_ft(100.0, Some(150.0)), GATE_FT);
        assert_eq!(gate_agl_ft(100.0, Some(600.0)), 900.0);
        // The path: level at the cruise far out, down to the gate where the
        // slow-down starts, level at the gate inside it.
        let (pad, cruise) = (100.0, 1100.0);
        let p = ApproachPath::new(GATE_FT, 120.0);
        assert_eq!(p.target_ft(40_000.0, pad, cruise), cruise);
        assert_eq!(p.target_ft(p.slow_m, pad, cruise), pad + GATE_FT);
        assert_eq!(p.target_ft(100.0, pad, cruise), pad + GATE_FT);
        // The level segment is the planned slow-down from the speed flown,
        // to where the final begins, and a margin.
        assert!(
            (p.slow_m - (final_len_m(GATE_FT) + slow_down_m(120.0, HANDOVER_KTS) + SLOW_MARGIN_M)).abs() < 1e-6
        );
        assert!(ApproachPath::new(GATE_FT, 125.0).slow_m > ApproachPath::new(GATE_FT, 90.0).slow_m);
        // As steep as the vertical-speed cap allows at the speed flown: 6.5
        // degrees at 70 kt, shallower at 125 (6.5 degrees would be 1,450 fpm).
        assert!((ApproachPath::new(GATE_FT, 50.0).tan - APPROACH_DEG.to_radians().tan()).abs() < 1e-12);
        let fast = ApproachPath::new(GATE_FT, 125.0);
        assert!(fast.tan * 125.0 * 101.269 <= APPROACH_MAX_SINK_FPM * 0.85 + 1e-6);
        assert!(fast.tan < APPROACH_DEG.to_radians().tan());
        // It starts where the path from the cruise meets the level.
        let start = p.start_m(cruise, pad);
        assert!((start - (p.slow_m + (cruise - pad - GATE_FT) / 3.28084 / p.tan)).abs() < 1e-6);
        // The schedule: the hand-over speed where the final begins, the
        // speed flown where the slow-down begins, and the target only down.
        assert!((slow_schedule_kts(0.0, HANDOVER_KTS) - HANDOVER_KTS).abs() < 1e-9);
        assert!((slow_schedule_kts(slow_down_m(117.0, HANDOVER_KTS), HANDOVER_KTS) - 117.0).abs() < 1e-6);
        assert_eq!(slow_want_kts(10_000.0, 100.0, 100.0, 100.0), 100.0);
        assert!(slow_want_kts(1_000.0, 100.0, 100.0, 100.0) < 70.0);
        // The schedule is over the ground: a tailwind ends it lower, at the
        // hand-over's own target; a headwind flies more airspeed on the way.
        assert!((slow_want_kts(0.0, 40.0, 50.0, 90.0) - slow_target_kts(40.0, 50.0)).abs() < 1e-9);
        assert!(slow_want_kts(3_000.0, 80.0, 70.0, 200.0) > slow_want_kts(3_000.0, 80.0, 80.0, 200.0));
        assert!((slow_want_kts(0.0, 60.0, 50.0, 200.0) - HANDOVER_KTS).abs() < 1e-9);
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

    /// The ETND final (2026-09-23, #82), replayed: handed over 4,391 m from
    /// the pad at 11.1 m/s (21.6 kt) over the ground and 39.5 indicated, a
    /// 17 kt headwind. The route re-aims every half second, the hover loop
    /// slews its command at its own ramp, and the mode's groundspeed follows
    /// through a second-order lag and 0.3 s of transport delay. The old cap
    /// (the handed-over groundspeed) flew it at 21.6 kt; the cap keyed on the
    /// mode's groundspeed range and an airspeed ceiling flies it at 38, the
    /// airspeed never past the ceiling, and still stops on the pad.
    #[test]
    fn a_headwind_final_flies_the_modes_groundspeed_not_the_handover_speed() {
        let fly = |wind: Option<(f64, f64)>| {
            let head = wind.map_or(0.0, |w| w.0);
            let (mut d, mut v, mut v1, mut cmd) = (4391.0_f64, 11.1_f64, 11.1_f64, 11.1_f64);
            let mut want = 11.1_f64;
            let mut q = std::collections::VecDeque::from(vec![11.1; 3]);
            let dt = 0.1;
            let (mut t, mut worst_air, mut worst_cmd): (f64, f64, f64) = (0.0, 0.0, 0.0);
            let mut arrived = None;
            for i in 0..20_000 {
                if i % 5 == 0 {
                    let cap = final_cap_mps(11.1, wind);
                    let (s, _) = final_command(d, 2004.0, cap);
                    if d < ARRIVED_M || s < 1.0 {
                        arrived = Some((d, v));
                        break;
                    }
                    want = s;
                }
                // The hover loop's slew (RAMP_ACCEL_MPS2), then the mode.
                cmd += (want - cmd).clamp(-0.4 * dt, 0.4 * dt);
                q.push_back(cmd);
                let c = q.pop_front().unwrap();
                v1 += (c - v1) * dt / 1.2;
                v += (v1 - v) * dt / 1.2;
                d -= v * dt;
                t += dt;
                worst_air = worst_air.max((v + head) * 1.943_84);
                worst_cmd = worst_cmd.max(cmd);
            }
            (t, worst_air, worst_cmd, arrived.expect("reached the pad"))
        };
        // Headwind 8.7 m/s (17 kt), straight down the final.
        let etnd = Some(wind_components(270.0, 270.0, 8.7));
        let (old_t, _, old_cmd, (_, old_end)) = fly(None);
        let (new_t, air, new_cmd, (d_end, v_end)) = fly(etnd);
        eprintln!("ETND final: {old_t:.0} s at the hand-over's speed, {new_t:.0} s now, {air:.1} kt through the air");
        assert!(
            old_t > 380.0,
            "the recorded final took 6.5 min: {old_t:.0} s"
        );
        assert!(
            (old_cmd - 11.1).abs() < 0.01,
            "old cap was the hand-over speed"
        );
        assert!(new_t < old_t - 120.0, "{new_t:.0} s against {old_t:.0} s");
        assert!(
            new_cmd <= FINAL_MAX_GS_MPS + 1e-9,
            "inside the mode's range: {new_cmd:.2}"
        );
        assert!(air <= FINAL_MAX_AIR_KTS + 1.0, "airspeed {air:.1} kt");
        // The taper is the same one: over the pad no faster than before.
        assert!(
            d_end <= ARRIVED_M && v_end < old_end + 0.5,
            "stops on the pad: {d_end:.0} m at {v_end:.1} m/s (old {old_end:.1})"
        );
        // A gale is held to the airspeed ceiling, and never slowed under the
        // groundspeed it came in at; calm air flies the mode's range.
        let gale = final_cap_mps(5.0, Some((25.0, 0.0)));
        assert!((gale - 5.0).abs() < 1e-9, "{gale}");
        let strong = final_cap_mps(5.0, Some((15.0, 0.0)));
        assert!(
            ((strong + 15.0) * 1.943_84 - FINAL_MAX_AIR_KTS).abs() < 0.1,
            "{strong}"
        );
        assert!((final_cap_mps(19.0, Some((0.0, 0.0))) - FINAL_MAX_GS_MPS).abs() < 1e-9);
        // A crosswind eats airspeed too; a tailwind is capped by the ground.
        assert!(final_cap_mps(5.0, Some((10.0, 12.0))) < final_cap_mps(5.0, Some((10.0, 0.0))));
        assert!((final_cap_mps(19.0, Some((-10.0, 0.0))) - FINAL_MAX_GS_MPS).abs() < 1e-9);
        // No wind read: the old cap, never a guess.
        assert_eq!(final_cap_mps(11.1, None), 11.1);
        assert_eq!(final_cap_mps(11.1, Some((f64::NAN, 0.0))), 11.1);
        // The wind's components: from the nose is a headwind, from the right
        // a positive crosswind, from behind a negative headwind.
        let (h, x) = wind_components(90.0, 90.0, 10.0);
        assert!((h - 10.0).abs() < 1e-9 && x.abs() < 1e-9);
        let (h, x) = wind_components(90.0, 180.0, 10.0);
        assert!(h.abs() < 1e-9 && (x - 10.0).abs() < 1e-9);
        assert!((wind_components(90.0, 270.0, 10.0).0 + 10.0).abs() < 1e-9);
    }

    /// The slow-down in a tailwind: 38 kt indicated with 12 kt behind is 50
    /// over the ground, and the hover loop's takeover waits for 39. The
    /// target comes down by the tailwind (to its floor), so the speed law on
    /// the collective brings the GROUNDSPEED into the takeover's range; a
    /// headwind leaves it alone.
    #[test]
    fn a_tailwind_slowdown_arrives_inside_the_takeover_range() {
        assert_eq!(
            slow_target_kts(38.0, 21.6),
            HANDOVER_KTS,
            "a headwind changes nothing"
        );
        assert_eq!(slow_target_kts(38.0, 38.0), HANDOVER_KTS);
        assert_eq!(slow_target_kts(38.0, 44.0), 32.0);
        assert_eq!(slow_target_kts(38.0, 60.0), SLOW_MIN_KTS);
        assert_eq!(slow_target_kts(f64::NAN, 40.0), HANDOVER_KTS);
        // Through the speed law and a lagged plant, a 6 kt tailwind: the old
        // target settles 44 kt over the ground, outside the takeover; the new
        // one inside it.
        let fly = |tail: f64, keyed: bool| {
            let mut law = SpeedLaw::from(0.45, 70.0);
            law.set_floor(APPROACH_COLLECTIVE_MIN);
            let (mut v, mut acc) = (70.0_f64, 0.0_f64);
            let dt = 0.5;
            for _ in 0..600 {
                let want = if keyed {
                    slow_target_kts(v, v + tail)
                } else {
                    HANDOVER_KTS
                };
                let c = law.step(want, v, dt);
                let level = hover::level_collective(v * 0.5144);
                acc += (20.0 * (c - level) - acc) * (dt / 2.0);
                v += acc * dt;
            }
            (v + tail) * 0.514_444
        };
        let old = fly(6.0, false);
        let new = fly(6.0, true);
        assert!(
            old > hover::TAKEOVER_MAX_MPS,
            "old: {:.1} kt over the ground",
            old * 1.943_84
        );
        assert!(
            new < hover::TAKEOVER_MAX_MPS,
            "new: {:.1} kt over the ground",
            new * 1.943_84
        );
    }

    /// #83: a tower's "say callsign and intentions" is answered: the
    /// destination's own tower with the inbound call, any other field's with
    /// a transit request the tower parses as one, and a station that keeps
    /// asking is not answered forever (EDVW-ETND 2026-09-23: every transit
    /// field's challenge went unanswered).
    #[test]
    fn a_zone_challenge_is_answered_inbound_at_home_and_transit_elsewhere() {
        assert_eq!(challenge_answer(false, Some(131200), Some(131200), 0), None);
        assert_eq!(challenge_answer(true, None, Some(131200), 0), None);
        assert_eq!(
            challenge_answer(true, Some(131200), Some(131200), 0),
            Some(ChallengeAnswer::Inbound(131200))
        );
        assert_eq!(
            challenge_answer(true, Some(118500), Some(131200), 0),
            Some(ChallengeAnswer::Transit),
            "a crossing field is never answered inbound"
        );
        assert_eq!(
            challenge_answer(true, Some(118500), None, 1),
            Some(ChallengeAnswer::Transit)
        );
        assert_eq!(
            challenge_answer(true, Some(118500), None, CHALLENGE_ANSWERS_MAX),
            None
        );
        let words = transit_words(2240.0);
        assert_eq!(
            words,
            "helicopter, two thousand two hundred feet, request zone transit"
        );
        let call = crate::circuit::radio_call(&Some("Delta Hotel Xray".into()), &words);
        assert_eq!(
            crate::tower::parse_intent(&call),
            crate::tower::Intent::RequestTransit,
            "{call}"
        );
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
            transition_level_ft: None,
            allows_circuits: None,
            pattern_sides: Vec::new(),
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
        let src = |f: &str| crate::test_support::module_source(f.trim_end_matches(".rs")).to_string();
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
            &crate::tools::autoland::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat"),
            "ETHC",
        );
        assert!(pads.len() >= 18, "{}", pads.len());
    }

    /// LXGB as apt.dat has it (one strip, 09/27).
    fn lxgb() -> crate::nav::AirportInfo {
        crate::nav::AirportInfo {
            transition_level_ft: None,
            allows_circuits: None,
            pattern_sides: Vec::new(),
            name: "Gibraltar".into(),
            elev_ft: 15.0,
            runways: vec![crate::nav::Runway {
                ident_a: "09".into(),
                ident_b: "27".into(),
                lat_a: 36.1508156,
                lon_a: -5.3596312,
                lat_b: 36.1514944,
                lon_b: -5.3400082,
                width_m: 45.11,
                surface: 1,
            }],
            freqs: Vec::new(),
        }
    }

    /// LXGB (iMac 2026-09-30, 1790722408-heli_route): arriving from LEMG,
    /// the nearest end is 27, and the inbound call earned "join final runway
    /// zero niner". The tower's runway moves the landing to 09's spot, 150 m
    /// in from ITS threshold; the same end, an unnamed runway or an end the
    /// field lacks moves nothing.
    #[test]
    fn the_towers_runway_moves_the_landing_to_its_end() {
        let apt = lxgb();
        let inbound = (36.20, -5.28);
        let (_, _, name) = runway_spot(&apt, inbound).expect("a spot");
        assert_eq!(name, "runway 27");
        let current = spot_runway(&format!("LXGB {name}"));
        assert_eq!(current.as_deref(), Some("27"));
        let spots = runway_spots(&apt);
        assert_eq!(spots.len(), 2);
        let (id, la, lo) = spot_for_runway(&spots, current.as_deref(), Some("09")).expect("09's spot");
        assert_eq!(id, "09");
        let from_09 = crate::nav::haversine_nm(36.1508156, -5.3596312, la, lo) * 1852.0;
        assert!((from_09 - 150.0).abs() < 5.0, "{from_09}");
        assert!(lo < -5.355, "on the west end: {lo}");
        assert!(spot_for_runway(&spots, current.as_deref(), Some("27")).is_none(), "already there");
        assert!(spot_for_runway(&spots, Some("27 "), Some(" 27")).is_none(), "trimmed");
        assert!(spot_for_runway(&spots, current.as_deref(), None).is_none());
        assert!(spot_for_runway(&spots, current.as_deref(), Some("")).is_none());
        assert!(spot_for_runway(&spots, current.as_deref(), Some("15")).is_none(), "no such end");
        // Once moved, the same runway named again is no move.
        assert!(spot_for_runway(&spots, Some("09"), Some("09")).is_none());
    }

    /// A towered landing: the final reported once the inbound call is out
    /// and the tower is the station; cleared, nothing more is owed.
    #[test]
    fn a_towered_final_reports_and_lands_once_cleared() {
        let mut w = ClearanceWatch::default();
        // Not yet the tower's (the swap still pending): nothing to say.
        assert_eq!(w.step(0.0, true, false, false, false), None);
        assert_eq!(w.step(1.0, true, true, false, false), Some(ClearanceStep::ReportFinal));
        assert_eq!(w.step(2.0, true, true, false, false), None, "reported once");
        assert_eq!(w.step(3.0, true, true, true, false), None, "cleared");
        assert_eq!(w.step(200.0, true, true, true, true), None, "cleared never holds");
        // No tower (or copilot mode): the watch is silent throughout.
        let mut none = ClearanceWatch::default();
        for t in 0..300 {
            assert_eq!(none.step(t as f64, false, false, false, true), None);
        }
    }

    /// Never land uncleared: held short, the clearance is asked again
    /// every 30 s, three times, and 120 s held without it hands the hover
    /// to the pilot, once.
    #[test]
    fn an_uncleared_final_holds_asks_again_and_hands_over_bounded() {
        let mut w = ClearanceWatch::default();
        assert_eq!(w.step(0.0, true, true, false, false), Some(ClearanceStep::ReportFinal));
        let mut asks = Vec::new();
        let mut gave_up = None;
        let mut t = 40.0;
        while t < 400.0 {
            match w.step(t, true, true, false, true) {
                Some(ClearanceStep::Ask) => asks.push(t),
                Some(ClearanceStep::GiveUp) => {
                    gave_up = Some(t);
                    break;
                }
                Some(ClearanceStep::ReportFinal) => panic!("reported twice"),
                None => {}
            }
            t += 0.5;
        }
        assert_eq!(asks, vec![40.0, 70.0, 100.0]);
        assert_eq!(gave_up, Some(40.0 + CLEARANCE_HOLD_MAX_S));
        assert_eq!(w.step(t + 1.0, true, true, false, true), None, "given up once");
        // A tower that never became the station: no calls to nobody, and
        // still bounded.
        let mut deaf = ClearanceWatch::default();
        let mut end = None;
        for i in 0..400 {
            let t = i as f64;
            match deaf.step(t, true, false, false, true) {
                Some(ClearanceStep::GiveUp) => {
                    end = Some(t);
                    break;
                }
                Some(s) => panic!("{s:?} with no tower"),
                None => {}
            }
        }
        assert_eq!(end, Some(CLEARANCE_HOLD_MAX_S));
        // Cleared while held: it goes on to land.
        let mut late = ClearanceWatch::default();
        late.step(0.0, true, true, false, false);
        assert_eq!(late.step(10.0, true, true, false, true), None, "too soon to ask");
        assert_eq!(late.step(30.0, true, true, false, true), Some(ClearanceStep::Ask));
        assert_eq!(late.step(500.0, true, true, true, true), None);
    }

    /// The hold is a hover short of the pad: its speed runs out at the
    /// hold point, its height is the final's where we are, and inside the
    /// hold point it is a hover where we are. Cleared, the final's own
    /// command lands on the pad as before.
    #[test]
    fn the_uncleared_final_stops_short_of_the_pad() {
        let (from_ft, from_mps) = (470.0, 19.5);
        let (s_far, h_far) = hold_command(1300.0, from_ft, from_mps);
        assert!(s_far > 15.0, "still flying the final: {s_far}");
        assert_eq!(h_far, final_command(1300.0, from_ft, s_far).1);
        assert!(h_far <= from_ft);
        let (s_at, h_at) = hold_command(HOLD_SHORT_M, from_ft, from_mps);
        assert!(s_at < 1e-9, "{s_at}");
        assert!(h_at > 160.0, "held on the path, clear of the ground: {h_at}");
        let (s_in, _) = hold_command(100.0, from_ft, from_mps);
        assert!(s_in < 1e-9);
        // The landing gate (`speed < 1.0`) is the final's, never the hold's.
        assert!(final_command(HOLD_SHORT_M, from_ft, from_mps).0 > 1.0);
    }

    /// The calls are the aeroplanes' words, and the tower hears a final
    /// report in each (never a landing request).
    #[test]
    fn the_final_report_and_the_ask_are_final_reports_to_the_tower() {
        assert_eq!(final_words(Some("09"), false), "final runway zero nine");
        assert_eq!(
            final_words(Some("09"), true),
            "final runway zero nine, request landing clearance"
        );
        assert_eq!(final_words(None, false), "final");
        for w in [
            final_words(Some("09"), false),
            final_words(Some("09"), true),
            final_words(None, false),
            final_words(None, true),
        ] {
            let call = crate::circuit::radio_call(&Some("November five seven six X-ray Papa".into()), &w);
            assert_eq!(
                crate::tower::parse_intent(&call.to_ascii_lowercase()),
                crate::tower::Intent::ReportFinal,
                "{call}"
            );
        }
    }
}
