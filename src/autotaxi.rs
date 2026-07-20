//! Auto-taxi — the copilot steers the cleared taxi route to the holding point.
//!
//! Composes the two halves that already exist: the taxi-speed governor
//! ([`crate::tools::speedhold`], throttle + brakes) and the tower's published
//! route ([`crate::party_line::with_taxi_route`], the same polyline the
//! progressive taxi guide narrates). This module adds the missing piece — a
//! ground steering law — and a speed schedule that slows for bends and ramps
//! to a stop short of the holding point. The guide keeps talking over it, so
//! the pilot hears the same "next left onto Whiskey" narration whether they
//! or the copilot are steering.
//!
//! Steering goes through the plugin's `xcopilot/taxi_steer` channel (the
//! proportional-brake pattern: per-frame writes under
//! `override_joystick_heading`, expiring within a second of writes stopping),
//! because REST-rate writes to `yoke_heading_ratio` lose the re-drive fight
//! with the pilot's hardware exactly like the brake ratios did.
//!
//! Handback, every path instant:
//! - The pilot's FEET win in-plugin: real yaw-axis deflection latches
//!   `xcopilot/steer_yield` and drops the override that frame — this loop
//!   polls the latch and stands down, but the pedals were live already.
//! - "My controls" (voice or panel button) releases steering and throttle
//!   together via the main loop's release path; "my steering" releases just
//!   the steering, leaving the speed hold — the pre-existing "you steer, I
//!   have the throttle" division.
//! - The governor standing down for ANY reason (park handle, throttle push,
//!   airborne, release words) takes the steering with it: speed authority
//!   gone means nobody is stopping the aircraft, so steering alone must not
//!   keep driving it.
//! - Off the route past [`OFF_ROUTE_STOP_M`] the loop STOPS the aircraft and
//!   releases steering — never "recovers" by driving pavement it wasn't
//!   cleared on. The pilot sorts it out with the tower.
//!
//! Everything here is pure geometry over the published polyline; the laws
//! battle-test as plain `cargo test`, no sim.

use std::sync::atomic::{AtomicBool, Ordering};

use crate::taxi_guide;
use crate::tools::speedhold;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;
use crate::station::TaxiWatch;

pub(crate) const LAT: &str = "sim/flightmodel/position/latitude";
pub(crate) const LON: &str = "sim/flightmodel/position/longitude";
/// TRUE heading, matching the route bearings computed in the local
/// east/north frame — never the magnetic compass.
pub(crate) const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
const THROTTLE: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
/// The plugin's proportional steering channel (-1..1, right positive).
const TAXI_STEER: &str = "xcopilot/taxi_steer";
/// The plugin's pilot-feet latch: 1 = the yaw axis moved, override dropped.
const STEER_YIELD: &str = "xcopilot/steer_yield";
const MS_TO_KTS: f64 = 1.943_844;

/// Stop this many meters short of the route's end. The planner already ends
/// the route at the hold-short standoff (40 m back from a runway-junction
/// tail — nav.rs `plan`), so this is only the nose-ahead-of-CG margin, not
/// the hold line's position. It was 12 m back when the route ended ON the
/// runway centreline, and the aircraft still stopped past the hold line
/// (2026-07-20): the standoff belongs to the planner, not this margin.
const STOP_SHORT_M: f64 = 5.0;
/// Lateral drift at which the loop stops the aircraft and gives steering
/// back. Deliberately much tighter than the guide/compliance 60 m: the law
/// holds a couple of meters in normal work, so 25 m off means something is
/// wrong (bad fix, blocked nosewheel, a route the geometry can't follow).
pub(crate) const OFF_ROUTE_STOP_M: f64 = 25.0;
/// Full steering deflection at this much heading error.
const STEER_FULL_DEG: f64 = 30.0;
/// Route bends at/above this (smoothed like the guide's cues) slow the
/// schedule to [`TURN_KTS`]; at/above `SHARP_DEG`, to [`SHARP_KTS`].
const TURN_DEG: f64 = 30.0;
const SHARP_DEG: f64 = 100.0;
const TURN_KTS: f64 = 6.0;
const SHARP_KTS: f64 = 4.0;
/// Bearing smoothing window for bend detection, meters (the guide's value:
/// densely-noded fillet arcs must not read as hairpins).
const WINDOW_M: f64 = 15.0;

/// True while the steering loop runs, mirroring `speedhold::ACTIVE` /
/// `autoland::ACTIVE`. Cleared by the loop itself, the release path in
/// main, or the `auto_taxi` tool's off switch.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);

/// One-line status for the dashboard's "managing" panel.
pub fn status() -> Option<String> {
    ACTIVE.load(Ordering::Relaxed).then(|| "auto taxi".to_string())
}

/// The cleared route, pre-digested into the local flat-earth frame the
/// steering law works in.
pub struct Route {
    /// Lat/lon polyline, for projecting fixes (same code path as the guide).
    path: Vec<(f64, f64)>,
    /// The polyline in meters (x east, y north) from `path[0]`.
    pts: Vec<(f64, f64)>,
    /// Cumulative along-route distance per vertex.
    cum: Vec<f64>,
    total: f64,
    /// (along-route m, |bend| degrees) per real corner, for the slow-down.
    bends: Vec<(f64, f64)>,
    /// Spoken holding point ("November five Whiskey"), may be empty.
    pub holding_point: String,
}

impl Route {
    /// Digest a published route. `None` when it's too short to drive.
    pub fn build(watch: &TaxiWatch) -> Option<Route> {
        if watch.path.len() < 2 {
            return None;
        }
        let geo = taxi_guide::Geo::new(watch.path[0].0);
        let pts: Vec<(f64, f64)> =
            watch.path.iter().map(|&p| geo.to_m(watch.path[0], p)).collect();
        let mut cum = vec![0.0_f64];
        for w in pts.windows(2) {
            cum.push(cum.last().unwrap() + taxi_guide::dist(w[0], w[1]));
        }
        let total = *cum.last().unwrap();
        // Shorter than the stop-short margin plus a rolling start there is
        // nothing to drive — the tool answers "we're already there".
        if total < STOP_SHORT_M + 15.0 {
            return None;
        }
        let mut bends: Vec<(f64, f64)> = Vec::new();
        for i in 1..pts.len() - 1 {
            let b_in = taxi_guide::bearing(
                taxi_guide::point_at(&pts, &cum, cum[i] - WINDOW_M),
                pts[i],
            );
            let b_out = taxi_guide::bearing(
                pts[i],
                taxi_guide::point_at(&pts, &cum, cum[i] + WINDOW_M),
            );
            let delta = taxi_guide::angle_diff(b_out, b_in).abs();
            if delta >= TURN_DEG {
                bends.push((cum[i], delta));
            }
        }
        Some(Route {
            path: watch.path.clone(),
            pts,
            cum,
            total,
            bends,
            holding_point: crate::station::spoken_taxiway(&watch.holding_point),
        })
    }

    /// Project a lat/lon fix: (along-route progress m, lateral m).
    pub fn fix(&self, lat: f64, lon: f64) -> Option<(f64, f64)> {
        taxi_guide::project(&self.path, lat, lon)
    }

    /// Heading error (signed degrees, right positive) from the aircraft's
    /// true heading to the pure-pursuit aim point `lookahead` meters further
    /// along the route. Aiming at a point ON the route (not the tangent) is
    /// what folds the cross-track error back to the centreline.
    pub fn heading_err_deg(
        &self,
        lat: f64,
        lon: f64,
        heading_true: f64,
        progress: f64,
        lookahead: f64,
    ) -> f64 {
        let geo = taxi_guide::Geo::new(self.path[0].0);
        let here = geo.to_m(self.path[0], (lat, lon));
        let aim = taxi_guide::point_at(&self.pts, &self.cum, progress + lookahead);
        taxi_guide::angle_diff(taxi_guide::bearing(here, aim), heading_true)
    }

    /// Meters left before the stop point short of the holding point.
    pub fn remaining(&self, progress: f64) -> f64 {
        self.total - STOP_SHORT_M - progress
    }

    /// The bend-imposed speed cap near `progress`, if a corner is close
    /// enough to matter: inside a speed-scaled lead distance before the
    /// corner, and until 15 m past it (mid-turn is no place to accelerate).
    pub fn bend_cap_kts(&self, progress: f64, gs_kts: f64) -> Option<f64> {
        let lead = (gs_kts * 0.514 * 6.0).clamp(25.0, 90.0);
        self.bends
            .iter()
            .filter(|(at, _)| progress < at + 15.0 && at - progress < lead)
            .map(|&(_, deg)| if deg >= SHARP_DEG { SHARP_KTS } else { TURN_KTS })
            .fold(None, |acc: Option<f64>, cap| Some(acc.map_or(cap, |a| a.min(cap))))
    }
}

/// Steering command for a heading error: proportional, full deflection at
/// [`STEER_FULL_DEG`]. No derivative term — the nosewheel plant at taxi
/// speed is heavily damped already, and the tick is fast.
pub fn steer_cmd(err_deg: f64) -> f64 {
    (err_deg / STEER_FULL_DEG).clamp(-1.0, 1.0)
}

/// Pure-pursuit lookahead: seconds of taxi ahead, floored so creep-speed
/// taxiing still aims somewhere and capped so the aim point can't cut a
/// corner two junctions away.
pub fn lookahead_m(gs_kts: f64) -> f64 {
    (gs_kts * 0.514 * 3.5).clamp(12.0, 50.0)
}

/// The braking-distance speed cap approaching the stop point: v = √(2·a·d)
/// at a gentle 0.5 m/s², floored at a 2-knot creep so the last meters
/// actually finish. At/inside half a meter it's a stop.
pub fn stop_ramp_kts(remaining_m: f64) -> f64 {
    if remaining_m <= 0.5 {
        return 0.0;
    }
    (MS_TO_KTS * (2.0 * 0.5 * remaining_m).sqrt()).max(2.0)
}

/// The scheduled taxi target: the pilot's pace, capped by any nearby bend
/// and by the stop ramp.
pub fn schedule_kts(base: f64, route: &Route, progress: f64, gs_kts: f64) -> f64 {
    let mut v = base.min(stop_ramp_kts(route.remaining(progress)));
    if let Some(cap) = route.bend_cap_kts(progress, gs_kts) {
        v = v.min(cap);
    }
    v
}

/// True when the plugin's steering channel is live — a zero write is a
/// no-op steer, so probing costs nothing.
pub async fn steer_channel_ok(xp: &dyn crate::xplane::XPlane) -> bool {
    xp.set_float(TAXI_STEER, 0.0).await.is_ok()
}

/// Spawn the steering loop. The caller (the `auto_taxi` tool) has already
/// validated the route, armed the taxi governor, and proved the plugin's
/// steer channel writable.
pub fn start(base_url: String, voice: Option<Voice>, route: Route, base_kts: f64) {
    if ACTIVE.swap(true, Ordering::Relaxed) {
        return; // already driving — the tool retargets instead
    }
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            ACTIVE.store(false, Ordering::Relaxed);
            return;
        };
        rt.block_on(drive(XPlaneClient::new(base_url), voice, route, base_kts));
    });
}

/// Say a line without blocking the loop (queued; gated off in ATC-only mode
/// like every crew alert).
fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    if let Some(v) = voice {
        v.alert(priority, line);
    }
}

/// The steering loop. Steering writes ride the plugin's expiring channel, so
/// every exit path — including a panic or kill -9 — releases the override
/// within a second by silence alone; the explicit zero write on the way out
/// is just politeness.
async fn drive(xp: XPlaneClient, voice: Option<Voice>, route: Route, base_kts: f64) {
    tracing::info!("auto taxi started");
    crate::blackbox::record(
        "autotaxi",
        serde_json::json!({"event": "engaged", "total_m": route.total, "base_kts": base_kts}),
    );
    // The pilot's pace: adopted from a spoken "taxi at N" mid-drive.
    let mut base = base_kts;
    // What the schedule last wrote into the governor — any OTHER value read
    // back is an external retarget (spoken stop/go/speed) to fold in.
    let mut last_sched = base_kts;
    // External stop ("stop" / both pedals park the target at 0): the
    // schedule stands aside until a "go" restores a rolling target.
    let mut ext_stopped = false;
    loop {
        if !ACTIVE.load(Ordering::Relaxed) {
            break;
        }
        // Speed authority gone — steering alone must not keep driving. This
        // is NOT only the deliberate paths (park handle, spoken release):
        // the governor's throttle-override check can't tell a pilot's hand
        // from a hardware-axis bump the sim re-drives, and mid-drive that
        // left an aircraft rolling at taxi power with NOBODY on the
        // throttle — the 2026-07-20 Gatwick runaway ran 10 to 50 knots
        // through the grass in silence. On the ground the safe reading is
        // stopping: throttle to idle plus ONE expiring brake pulse (the
        // pilot's own brake key, frees itself within a second, can't
        // stick), and say it out loud — a pilot who truly wants the power
        // pushes again with everything already released. Airborne (a
        // runaway that got that far), touch nothing.
        if !speedhold::taxi_hold_active() {
            let on_ground = xp.get_float(ON_GROUND).await.unwrap_or(0.0) > 0.5;
            tracing::info!(on_ground, "auto taxi: taxi hold gone, arresting and releasing");
            if on_ground {
                let _ = xp.set_float(THROTTLE, 0.0).await;
                if xp
                    .run_command_for(crate::tools::BRAKES_REGULAR, crate::tools::BRAKE_HOLD_SECS)
                    .await
                    .is_ok()
                {
                    crate::tools::note_brake_cmd();
                }
                alert(
                    &voice,
                    Priority::Caution,
                    "I've lost the speed hold. Throttle's idle. Your aircraft.",
                );
            } else {
                alert(&voice, Priority::Caution, "I've lost the speed hold. Your aircraft.");
            }
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "hold_gone", "arrested": on_ground}),
            );
            break;
        }
        // Paused or replay: stop writing — the steer channel expires and the
        // pedals are the pilot's until the sim is live again.
        if crate::xplane::sim_frozen(&xp).await {
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        }
        // The pilot's feet moved the yaw axis: the plugin already dropped
        // the override — stand the loop down and keep the speed hold, which
        // is the pre-existing "you steer, I have the throttle" division.
        if xp.get_float(STEER_YIELD).await.is_ok_and(|v| v > 0.5) {
            tracing::info!("auto taxi: pilot on the pedals, steering released");
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "pedal_yield"}));
            alert(
                &voice,
                Priority::Advisory,
                "Your steering. I still have the throttle and brakes.",
            );
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        let (Ok(lat), Ok(lon), Ok(psi), Ok(gs_ms)) = (
            xp.get_float(LAT).await,
            xp.get_float(LON).await,
            xp.get_float(PSI_TRUE).await,
            xp.get_float(GS_MS).await,
        ) else {
            // Sim hiccup: no writes, the channel expires, retry.
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
            continue;
        };
        let gs_kts = gs_ms * MS_TO_KTS;
        let Some((progress, lateral)) = route.fix(lat, lon) else {
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
            continue;
        };
        // Too far off the pavement we were cleared on: stop, hand steering
        // back, and say so. Never drive an uncleared recovery.
        if lateral > OFF_ROUTE_STOP_M {
            tracing::warn!(lateral, "auto taxi: off route, stopping");
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "off_route", "lateral_m": lateral}),
            );
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            alert(
                &voice,
                Priority::Caution,
                "We're off the cleared route. Stopping here. Your steering.",
            );
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        let remaining = route.remaining(progress);
        // Arrived: stopped at the hold-short line. The governor stays,
        // holding position on the brakes, exactly as a spoken "stop" —
        // "go" past this point is the pilot's call (a crossing clearance).
        if remaining <= 1.0 && gs_kts < 1.5 {
            tracing::info!("auto taxi: holding point, done");
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "holding_point"}));
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            let line = if route.holding_point.is_empty() {
                "Holding point. Holding position, brakes set.".to_string()
            } else {
                format!("Holding point {}. Holding position, brakes set.", route.holding_point)
            };
            alert(&voice, Priority::Advisory, &line);
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        // Steering: aim at the route ahead, command the error down.
        let err = route.heading_err_deg(lat, lon, psi, progress, lookahead_m(gs_kts));
        if xp.set_float(TAXI_STEER, steer_cmd(err)).await.is_err() {
            // The channel vanished (plugin reload): no steering, no drive.
            tracing::warn!("auto taxi: steer channel lost, stopping");
            speedhold::set_taxi_target(0.0);
            alert(
                &voice,
                Priority::Caution,
                "I've lost the steering. Stopping. Your aircraft.",
            );
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        // Speed: fold in any external retarget, then write the schedule.
        match speedhold::taxi_target() {
            Some(t) if (t - last_sched).abs() > 0.01 => {
                // Someone else moved the target: a spoken "stop" parks the
                // schedule; a spoken pace ("go", "taxi at 12") becomes the
                // new base.
                if t < 1.0 {
                    ext_stopped = true;
                } else {
                    base = t;
                    ext_stopped = false;
                }
                last_sched = t;
            }
            _ => {}
        }
        if !ext_stopped {
            let sched = schedule_kts(base, &route, progress, gs_kts);
            if (sched - last_sched).abs() > 0.01 {
                speedhold::set_taxi_target(sched);
                last_sched = sched;
            }
        }
        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
    }
    // Politeness on the way out — expiry would do it within the second.
    let _ = xp.set_float(TAXI_STEER, 0.0).await;
    ACTIVE.store(false, Ordering::Relaxed);
    tracing::info!("auto taxi stopped");
}

#[cfg(test)]
mod tests {
    use super::*;

    const LAT0: f64 = 36.0;
    const LON0: f64 = -5.0;
    /// Lat/lon from meters east/north of the origin — the inverse of
    /// [`Geo::to_m`], so routes read like a taxi diagram.
    fn ll(x_m: f64, y_m: f64) -> (f64, f64) {
        (LAT0 + y_m / 111_320.0, LON0 + x_m / (111_320.0 * LAT0.to_radians().cos()))
    }

    /// East 200 m, left turn north, 300 m to the holding point — the guide's
    /// test diagram, minus the runway crossing.
    fn watch() -> TaxiWatch {
        TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(200.0, 0.0), ll(200.0, 300.0)],
            via: "Whiskey, November".into(),
            seg: vec![("W".into(), false), ("N".into(), false)],
            holding_point: "N1".into(),
        }
    }

    #[test]
    fn steering_is_proportional_and_bounded() {
        assert_eq!(steer_cmd(0.0), 0.0);
        assert!(steer_cmd(10.0) > 0.0 && steer_cmd(10.0) < 1.0, "right of course, steer right");
        assert!(steer_cmd(-10.0) < 0.0);
        assert_eq!(steer_cmd(90.0), 1.0, "full deflection is the cap");
        assert_eq!(steer_cmd(-170.0), -1.0);
    }

    #[test]
    fn pure_pursuit_steers_back_to_the_centreline() {
        let r = Route::build(&watch()).expect("route");
        // On the first leg (eastbound, true bearing 090), 8 m LEFT of the
        // centreline, heading exactly east: the aim point is on the route
        // ahead, so the error must steer RIGHT (positive), back toward it.
        let (lat, lon) = ll(50.0, 8.0);
        let (progress, lateral) = r.fix(lat, lon).expect("fix");
        assert!((lateral - 8.0).abs() < 1.0, "{lateral}");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err > 2.0, "must steer back right, got {err}");
        // Mirror: 8 m right of the line steers left.
        let (lat, lon) = ll(50.0, -8.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err < -2.0, "must steer back left, got {err}");
        // On the line, on heading: hands still.
        let (lat, lon) = ll(50.0, 0.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err.abs() < 1.0, "{err}");
    }

    #[test]
    fn the_turn_is_anticipated_through_the_aim_point() {
        let r = Route::build(&watch()).expect("route");
        // 10 m short of the left turn at 10 knots: the aim point is already
        // around the corner, so the command leads the turn.
        let (lat, lon) = ll(190.0, 0.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(10.0));
        assert!(err < -10.0, "aim point should pull left into the turn, got {err}");
    }

    #[test]
    fn schedule_slows_for_bends_and_ramps_to_the_stop() {
        let r = Route::build(&watch()).expect("route");
        // Mid first leg: cruise pace.
        assert_eq!(schedule_kts(10.0, &r, 50.0, 10.0), 10.0);
        // Approaching the 90° corner at 200 m: capped to the turn speed.
        let near_turn = schedule_kts(10.0, &r, 170.0, 10.0);
        assert_eq!(near_turn, TURN_KTS, "{near_turn}");
        // Mid-turn (just past the vertex): still capped.
        assert_eq!(schedule_kts(10.0, &r, 210.0, 6.0), TURN_KTS);
        // Well past it on the straight: back to pace.
        assert_eq!(schedule_kts(10.0, &r, 300.0, 10.0), 10.0);
        // 20 m from the stop point: ramping down, still moving.
        let ramp = schedule_kts(10.0, &r, r.total - STOP_SHORT_M - 20.0, 8.0);
        assert!(ramp > 2.0 && ramp < 10.0, "{ramp}");
        // At the stop point: zero.
        assert_eq!(schedule_kts(10.0, &r, r.total - STOP_SHORT_M, 3.0), 0.0);
    }

    #[test]
    fn stop_ramp_is_gentle_and_finishes() {
        assert_eq!(stop_ramp_kts(0.0), 0.0);
        assert_eq!(stop_ramp_kts(0.4), 0.0, "inside half a meter is stopped");
        assert!(stop_ramp_kts(1.0) >= 2.0, "creep floor carries the last meters");
        let (near, far) = (stop_ramp_kts(10.0), stop_ramp_kts(40.0));
        assert!(near < far, "closer is slower: {near} vs {far}");
        assert!(far < 13.0, "40 m out needs no takeoff-roll pace: {far}");
    }

    #[test]
    fn lookahead_scales_with_speed_within_bounds() {
        assert_eq!(lookahead_m(0.0), 12.0);
        assert!(lookahead_m(10.0) > 12.0 && lookahead_m(10.0) < 50.0);
        assert_eq!(lookahead_m(60.0), 50.0);
    }

    #[test]
    fn short_routes_refuse_to_build() {
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(15.0, 0.0)],
            via: "A".into(),
            seg: vec![("A".into(), false)],
            holding_point: "A1".into(),
        };
        assert!(Route::build(&w).is_none(), "nothing to drive on 15 m");
    }
}
