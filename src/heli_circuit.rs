//! A helicopter's circuit, flown on the hover loop's velocity vector.
//!
//! Measured on the S-76C (EDDV, 2026-09-23) before this was written: the
//! autopilot's hover mode is a groundspeed HOLD that follows a moving target
//! (10 kt to 0.2 m/s of error, a departure to 30 kt and 300 ft to 0.7, a
//! rotating target through a turn to 0.4), while [`crate::hover`] holds the
//! height on the collective and, in forward flight, the nose on the track
//! with the pedals. So a circuit is a sequence of tracks, speeds and heights
//! handed to that loop ([`hover::Goal::Fly`]), and the approach is a speed
//! that tapers with the distance to go and a height on a glidepath, ending
//! in a hover over the spot the circuit left from, then the set-down.
//!
//! The pattern is laid out from the take-off spot and the heading at the
//! start: upwind along the nose, crosswind, downwind back past the spot,
//! base, and a final into the spot. No runway: a helicopter's circuit is
//! flown to the spot it lifted from.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use crate::hover::{self, Goal};
use crate::taxi_guide::{bearing, dist, Geo};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

pub static ACTIVE: AtomicBool = AtomicBool::new(false);

/// The panel's line: the leg and the metres to its corner.
static DETAIL: crate::auto_status::Detail = crate::auto_status::Detail::new();

pub fn status() -> Option<String> {
    ACTIVE
        .load(Ordering::SeqCst)
        .then(|| DETAIL.line("heli circuit"))
}

/// Stand the circuit down; the hover loop keeps whatever it was last given
/// until its own door is used (the caller's).
pub fn abort() {
    ACTIVE.store(false, Ordering::SeqCst);
}

/// The circuit's numbers. Speeds and heights the hover mode has been flown
/// at; they grow by measurement.
pub const PATTERN_FT: f64 = 300.0;
pub const PATTERN_MPS: f64 = 15.4; // 30 kt
const UPWIND_M: f64 = 700.0;
const WIDTH_M: f64 = 600.0;
/// Past the spot on the downwind, before base: the final's length.
const FINAL_M: f64 = 800.0;
/// The final's glidepath, degrees, and the hover it ends in.
const GLIDE_DEG: f64 = 6.0;
pub const FINAL_HOVER_FT: f64 = 30.0;
/// The final's deceleration: under the ramp's own 0.4 m/s^2 so the command
/// is flyable and the speed arrives at zero on the spot.
const FINAL_DECEL_MPS2: f64 = 0.3;
/// A waypoint is passed this far before it, scaled by how much the next leg
/// turns (a 3 deg/s turn at 30 kt has a 290 m radius).
const TURN_RADIUS_M: f64 = 290.0;
/// Inside this of the spot and slower than this, the final is a hover.
const ARRIVED_M: f64 = 12.0;
const ARRIVED_MPS: f64 = 1.0;
const TICK: Duration = Duration::from_millis(500);

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Leg {
    Upwind,
    Crosswind,
    Downwind,
    Base,
    Final,
}

impl Leg {
    fn name(self) -> &'static str {
        match self {
            Leg::Upwind => "upwind",
            Leg::Crosswind => "crosswind",
            Leg::Downwind => "downwind",
            Leg::Base => "base",
            Leg::Final => "final",
        }
    }
}

/// The pattern in metres (x east, y north) from the spot: the corner each
/// leg flies TO, in order, the last being the spot itself.
pub fn corners(heading_deg: f64, right: bool) -> [(Leg, (f64, f64)); 5] {
    let h = heading_deg.to_radians();
    let fwd = (h.sin(), h.cos());
    let side = if right { 1.0 } else { -1.0 };
    // Right of the nose is (cos h, -sin h) in east/north.
    let out = (side * h.cos(), -side * h.sin());
    let at = |along: f64, across: f64| {
        (
            fwd.0 * along + out.0 * across,
            fwd.1 * along + out.1 * across,
        )
    };
    [
        (Leg::Upwind, at(UPWIND_M, 0.0)),
        (Leg::Crosswind, at(UPWIND_M, WIDTH_M)),
        (Leg::Downwind, at(-FINAL_M, WIDTH_M)),
        (Leg::Base, at(-FINAL_M, 0.0)),
        (Leg::Final, (0.0, 0.0)),
    ]
}

/// How far before a corner the turn onto the next leg begins.
pub fn lead_m(turn_deg: f64) -> f64 {
    TURN_RADIUS_M * (turn_deg.abs().min(120.0) / 2.0).to_radians().tan()
}

/// The final's command at `d` metres from the spot: a speed that stops on
/// the spot at the final's deceleration, and a height on the glidepath LED
/// by the height loop's own lag. That loop asks 10 fpm a foot of error, so a
/// path descending at 300 fpm is followed 30 ft high: the first circuit's
/// final ran 30 ft above its path all the way in and reached the spot at 42
/// ft against 34 (EDDV, 2026-09-23). The lead is the path's descent rate at
/// the commanded speed over that gain.
pub fn final_command(d_m: f64) -> (f64, f64) {
    let speed = (2.0 * FINAL_DECEL_MPS2 * d_m.max(0.0))
        .sqrt()
        .min(PATTERN_MPS);
    let path = FINAL_HOVER_FT + d_m * GLIDE_DEG.to_radians().tan() * 3.28084;
    let sink_fpm = speed * GLIDE_DEG.to_radians().tan() * 196.85;
    let lead = sink_fpm / hover::HEIGHT_GAIN_FPM_PER_FT;
    let height = if path >= PATTERN_FT {
        PATTERN_FT
    } else {
        (path - lead).max(FINAL_HOVER_FT)
    };
    (speed, height)
}

/// Start a circuit from the hover the loop is holding.
pub fn start(voice: Option<Voice>, base_url: String, right: bool) {
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
        rt.block_on(fly(XPlaneClient::new(base_url), voice, right));
        ACTIVE.store(false, Ordering::SeqCst);
    });
}

fn say(voice: &Option<Voice>, p: Priority, text: impl Into<String>) {
    if let Some(v) = voice {
        v.alert(p, text);
    }
}

async fn fly(xp: XPlaneClient, voice: Option<Voice>, right: bool) {
    use crate::datarefs::{LATITUDE, LONGITUDE};
    let (lat, lon, psi) = tokio::join!(
        xp.get_float(LATITUDE),
        xp.get_float(LONGITUDE),
        xp.get_float("sim/flightmodel/position/psi"),
    );
    let (Ok(lat), Ok(lon), Ok(psi)) = (lat, lon, psi) else {
        say(
            &voice,
            Priority::Caution,
            "I can't read our position. No circuit.",
        );
        return;
    };
    let spot = (lat, lon);
    let geo = Geo::new(lat);
    let pattern = corners(psi, right);
    crate::blackbox::record(
        "heli_circuit",
        serde_json::json!({ "event": "start", "lat": lat, "lon": lon, "heading": psi, "right": right }),
    );
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    let mut leg = 0usize;
    let mut tick_n = 0u64;
    while ACTIVE.load(Ordering::SeqCst) {
        tokio::time::sleep(TICK).await;
        if crate::xplane::sim_frozen(&xp).await {
            continue;
        }
        // Something else changed what the hover loop flies (a hold, a
        // landing, a drift stop, "my controls"): the circuit is over.
        let running = hover::ACTIVE.load(Ordering::SeqCst);
        let ours = matches!(hover::goal(), Some(Goal::Fly { .. })) || tick_n == 0;
        if !running || !ours {
            crate::blackbox::record(
                "heli_circuit",
                serde_json::json!({ "event": "ended_outside", "leg": pattern[leg].0.name(), "goal": format!("{:?}", hover::goal()) }),
            );
            return;
        }
        let (la, lo, agl, vx, vz) = tokio::join!(
            xp.get_float(LATITUDE),
            xp.get_float(LONGITUDE),
            xp.get_float("sim/flightmodel/position/y_agl"),
            xp.get_float("sim/flightmodel/position/local_vx"),
            xp.get_float("sim/flightmodel/position/local_vz"),
        );
        let (Ok(la), Ok(lo)) = (la, lo) else { continue };
        let slow = match (vx, vz) {
            (Ok(x), Ok(z)) => x.hypot(z) < hover::DEPART_CLIMB_MPS,
            _ => true,
        };
        let here = geo.to_m(spot, (la, lo));
        let (this_leg, to) = pattern[leg];
        let d = dist(here, to);
        let track = bearing(here, to);
        tick_n += 1;

        // Rounded to 50 m so the panel feed is not re-sent every tick.
        DETAIL.set(format!(
            "{} {:.0} m",
            this_leg.name(),
            (d / 50.0).round() * 50.0
        ));
        if this_leg == Leg::Final {
            let (speed, height) = final_command(d);
            if d < ARRIVED_M || speed < ARRIVED_MPS {
                // Over the spot: the landing stops what speed is left, then
                // comes down (the hover loop's own set-down).
                hover::retarget(Goal::Land);
                crate::blackbox::record(
                    "heli_circuit",
                    serde_json::json!({ "event": "over_spot", "d_m": d }),
                );
                say(
                    &voice,
                    Priority::Readback,
                    "Over the spot. Setting it down.",
                );
                return;
            }
            hover::retarget(Goal::Fly {
                height_ft: height,
                speed_mps: speed,
                track_deg: track,
            });
        } else {
            // Turn onto the next leg before the corner, by the turn's size.
            let next = pattern[leg + 1].1;
            let turn = crate::angles::signed_diff_deg(bearing(to, next), bearing(here, to));
            if d < lead_m(turn).max(40.0) {
                leg += 1;
                let name = pattern[leg].0.name();
                crate::blackbox::record(
                    "heli_circuit",
                    serde_json::json!({ "event": "leg", "leg": name, "d_m": d }),
                );
                if pattern[leg].0 == Leg::Final {
                    say(&voice, Priority::Readback, "Final.");
                }
                continue;
            }
            // The upwind climbs only once flying, like a departure (the
            // height-velocity diagram); a failed height read holds nothing
            // new and climbs.
            let height = match (this_leg, agl) {
                (Leg::Upwind, Ok(m)) if slow => (m * 3.28084).max(FINAL_HOVER_FT),
                _ => PATTERN_FT,
            };
            hover::retarget(Goal::Fly {
                height_ft: height,
                speed_mps: PATTERN_MPS,
                track_deg: track,
            });
        }
        if tick_n % 4 == 0 {
            crate::blackbox::record(
                "heli_circuit",
                serde_json::json!({ "event": "tick", "leg": this_leg.name(), "d_m": d, "track": track, "x": here.0, "y": here.1 }),
            );
        }
    }
    crate::blackbox::record("heli_circuit", serde_json::json!({ "event": "stood_down" }));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A left circuit off a northbound spot: upwind north, crosswind west,
    /// downwind south past the spot, base east, final north into it.
    #[test]
    fn the_pattern_is_laid_out_from_the_spot_and_the_nose() {
        let p = corners(0.0, false);
        let near = |a: (f64, f64), b: (f64, f64)| dist(a, b) < 1e-6;
        assert!(near(p[0].1, (0.0, UPWIND_M)));
        assert!(near(p[1].1, (-WIDTH_M, UPWIND_M)), "{:?}", p[1].1);
        assert!(near(p[2].1, (-WIDTH_M, -FINAL_M)));
        assert!(near(p[3].1, (0.0, -FINAL_M)));
        assert!(near(p[4].1, (0.0, 0.0)));
        // A right circuit off an eastbound spot goes south first.
        let r = corners(90.0, true);
        assert!(r[1].1 .1 < -WIDTH_M + 1.0, "{:?}", r[1].1);
    }

    /// The final stops on the spot: a flyable taper, the glidepath down to
    /// the hover, and both at the pattern's numbers far out.
    #[test]
    fn the_final_tapers_to_a_hover_over_the_spot() {
        assert_eq!(final_command(5000.0), (PATTERN_MPS, PATTERN_FT));
        // On the path, the height asked is below it by the loop's lag.
        let (_, h) = final_command(400.0);
        let path = FINAL_HOVER_FT + 400.0 * GLIDE_DEG.to_radians().tan() * 3.28084;
        assert!(h < path - 20.0 && h > FINAL_HOVER_FT, "{h} vs {path}");
        let (s, h) = final_command(0.0);
        assert_eq!((s, h), (0.0, FINAL_HOVER_FT));
        // Fly the taper with the loop's own ramp: the speed arrives at the
        // command without the ramp ever lagging it by a knot.
        let mut d = FINAL_M;
        let mut v: f64 = PATTERN_MPS;
        let dt = 0.1;
        let mut worst: f64 = 0.0;
        while d > ARRIVED_M {
            let (want, _) = final_command(d);
            let step = 0.4 * dt;
            v += (want - v).clamp(-step, step);
            worst = worst.max(v - want);
            d -= v * dt;
            assert!(d > -20.0, "overflew the spot");
        }
        assert!(worst < 0.5, "the ramp lagged the taper by {worst:.2} m/s");
        // The glidepath's descent at pattern speed is inside the loop's.
        let fpm = PATTERN_MPS * GLIDE_DEG.to_radians().tan() * 196.85;
        assert!(fpm < FLY_DESCENT_FPM_LIMIT, "{fpm:.0} fpm");
    }

    const FLY_DESCENT_FPM_LIMIT: f64 = 500.0;

    #[test]
    fn a_turn_leads_its_corner() {
        assert!((lead_m(90.0) - TURN_RADIUS_M).abs() < 1.0);
        assert!(lead_m(0.0) < 1.0);
    }
}
