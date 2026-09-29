//! Spoken reminders: "wake me at top of descent", "tell me when we reach 5000
//! feet", "remind me in 10 minutes". Each arms a one-shot background watcher (the
//! same fire-and-forget pattern as the autopilot capture watcher) that calls out
//! when its condition is met, spoken and mirrored to the panel.
//!
//! Armed reminders are also recorded in a [`Reminders`] registry so the copilot
//! has *memory* of them: it can answer "do we have a reminder set?" and cancel
//! them, rather than firing watchers it then forgets about.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::plugin_status::{AlertLevel, StatusSink};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const ALT_FT: &str = crate::datarefs::ALTITUDE_FT;
const TOD_DIST: &str = "sim/cockpit2/radios/indicators/fms_distance_to_tod_pilot";
const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const GPS_NAV_ID: &str = "sim/cockpit2/radios/indicators/gps_nav_id";
/// Reminders give up after this long if their condition never arrives.
const REMINDER_DEADLINE_SECS: u64 = 4 * 60 * 60;
const POLL_SECS: u64 = 5;

/// The live record of armed reminders, shared between the tool (which adds them)
/// and the command loop (which lists/cancels them). Cheap to clone.
#[derive(Clone, Default)]
pub struct Reminders {
    inner: Arc<Mutex<Vec<Armed>>>,
}

struct Armed {
    /// Spoken description ("at top of descent", "at 5000 feet").
    desc: String,
    /// Set to cancel: the watcher checks it each poll and exits without calling out.
    cancel: Arc<AtomicBool>,
    /// Set by the watcher once it fires or gives up, so a stale entry can be pruned.
    done: Arc<AtomicBool>,
}

impl Reminders {
    /// Register a reminder and hand back the flags its watcher honours.
    fn arm(&self, desc: String) -> Life {
        let life = Life {
            cancel: Arc::new(AtomicBool::new(false)),
            done: Arc::new(AtomicBool::new(false)),
        };
        let mut v = self.inner.lock().unwrap();
        v.retain(|a| !a.done.load(Ordering::Relaxed));
        v.push(Armed {
            desc,
            cancel: life.cancel.clone(),
            done: life.done.clone(),
        });
        life
    }

    /// Descriptions of the reminders still pending (fired/cancelled ones pruned).
    pub fn active(&self) -> Vec<String> {
        let mut v = self.inner.lock().unwrap();
        v.retain(|a| !a.done.load(Ordering::Relaxed) && !a.cancel.load(Ordering::Relaxed));
        v.iter().map(|a| a.desc.clone()).collect()
    }

    /// Cancel every pending reminder; returns how many were cancelled.
    pub fn cancel_all(&self) -> usize {
        let mut v = self.inner.lock().unwrap();
        let n = v
            .iter()
            .filter(|a| !a.done.load(Ordering::Relaxed) && !a.cancel.swap(true, Ordering::Relaxed))
            .count();
        v.clear();
        n
    }
}

/// The flags a watcher honours: stop early when `cancel` is set; mark `done` on
/// exit so the registry can prune it. Standalone (unregistered) when there's no
/// registry, e.g. outside the live loop.
#[derive(Clone)]
struct Life {
    cancel: Arc<AtomicBool>,
    done: Arc<AtomicBool>,
}

impl Life {
    fn standalone() -> Self {
        Self {
            cancel: Arc::new(AtomicBool::new(false)),
            done: Arc::new(AtomicBool::new(false)),
        }
    }
    fn cancelled(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }
}

/// Marks a reminder `done` on drop, so every watcher exit path (fire, cancel,
/// deadline, panic) prunes the registry entry without threading it through each return.
struct DoneGuard(Arc<AtomicBool>);
impl Drop for DoneGuard {
    fn drop(&mut self) {
        self.0.store(true, Ordering::Relaxed);
    }
}

pub struct SetReminder;

#[derive(Deserialize)]
struct Args {
    /// "top_of_descent" | "altitude" | "minutes" | "next_waypoint".
    kind: String,
    #[serde(default)]
    value: f64,
}

#[async_trait]
impl Tool for SetReminder {
    fn name(&self) -> &str {
        "set_reminder"
    }
    fn description(&self) -> &str {
        "Set a reminder that calls out when a condition is met: at the top of \
         descent, at a target altitude, after a number of minutes, or when the next \
         waypoint is reached. Use for 'wake me at top of descent', 'tell me when we \
         reach 5000 feet', 'remind me in 10 minutes', 'let me know when we reach the \
         next waypoint', 'give me a heads up at 3000 feet'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "kind": {"type": "string", "enum": ["top_of_descent", "altitude", "minutes", "next_waypoint"]},
                "value": {"type": "number", "description": "feet for altitude, minutes for minutes; ignored for top_of_descent and next_waypoint"}
            },
            "required": ["kind"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: Args = serde_json::from_value(args.clone())?;
        let base = ctx.config.xplane_base_url.clone();
        let voice = ctx.voice.cloned();
        let status = ctx.status.cloned();
        // Register the reminder so the copilot can recall and cancel it later.
        let register = |desc: String| match ctx.reminders {
            Some(r) => r.arm(desc),
            None => Life::standalone(),
        };
        match a.kind.as_str() {
            "top_of_descent" => {
                // Resolve a TOD ring up front so the watcher fires even on avionics
                // with no VNAV (the 172's GNS530): SimBrief's planned TOD, else the
                // cruise-altitude rule of thumb. The watcher still prefers the FMS's
                // own TOD distance when the box publishes one.
                let ring = tod_ring(&ctx.config.simbrief_id).await;
                arm_tod(base, ring, register("at top of descent".to_string()), voice, status);
                Ok("Roger, I'll call out the top of descent.".to_string())
            }
            "altitude" => {
                let target = a.value;
                let current = ctx.xplane.get_float(ALT_FT).await.unwrap_or(target);
                arm_altitude(base, target, current, register(format!("at {target:.0} feet")), voice, status);
                Ok(format!("Roger, I'll call out {target:.0} feet."))
            }
            "minutes" => {
                let mins = a.value.max(0.0);
                arm_timer(mins, register(format!("in {mins:.0} minutes")), voice, status);
                Ok(format!("Roger, I'll remind you in {mins:.0} minutes."))
            }
            "next_waypoint" => {
                let current = tracked_fix(&XPlaneClient::new(base.clone())).await;
                if current.trim().is_empty() {
                    return Ok("There's no active GPS waypoint to track. Load and navigate a route first.".to_string());
                }
                let from = current.trim().to_string();
                arm_waypoint(base, from.clone(), register(format!("at {from}")), voice, status);
                Ok("Roger, I'll call out the next waypoint.".to_string())
            }
            other => Ok(format!(
                "I can remind you at top of descent, at an altitude, or after some minutes, not '{other}'."
            )),
        }
    }
}

/// Sound the chime, speak the reminder, and mirror it to the panel.
fn announce(voice: &Option<Voice>, status: &Option<StatusSink>, line: &str) {
    if let Some(s) = status {
        s.alert(AlertLevel::Advisory, line);
    }
    if let Some(v) = voice {
        v.chime(Priority::Advisory, line.to_string());
    }
}

/// Run `body` on its own thread + runtime — the fire-and-forget watcher pattern.
fn spawn<F>(body: F)
where
    F: std::future::Future<Output = ()> + Send + 'static,
{
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(body);
    });
}

fn arm_timer(mins: f64, life: Life, voice: Option<Voice>, status: Option<StatusSink>) {
    spawn(async move {
        let _guard = DoneGuard(life.done.clone());
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs_f64(mins * 60.0);
        while std::time::Instant::now() < deadline {
            if life.cancelled() {
                return;
            }
            tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
        }
        if !life.cancelled() {
            announce(
                &voice,
                &status,
                &format!("Heads up, that's {mins:.0} minutes."),
            );
        }
    });
}

fn arm_altitude(
    base: String,
    target: f64,
    current: f64,
    life: Life,
    voice: Option<Voice>,
    status: Option<StatusSink>,
) {
    let climbing = target > current;
    spawn(async move {
        let _guard = DoneGuard(life.done.clone());
        let xp = XPlaneClient::new(base);
        let deadline =
            std::time::Instant::now() + std::time::Duration::from_secs(REMINDER_DEADLINE_SECS);
        while std::time::Instant::now() < deadline {
            if life.cancelled() {
                return;
            }
            if let Ok(alt) = xp.get_float(ALT_FT).await {
                let reached = if climbing {
                    alt >= target - 50.0
                } else {
                    alt <= target + 50.0
                };
                if reached {
                    announce(
                        &voice,
                        &status,
                        &format!("Heads up, passing {target:.0} feet."),
                    );
                    return;
                }
            }
            tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
        }
    });
}

/// The fix we are flying to: the procedure follower's own leg while one runs
/// (the sim's GPS stays parked on the plan's first entry the whole time), else
/// the GPS's active waypoint. Empty when neither knows.
async fn tracked_fix(xp: &XPlaneClient) -> String {
    if let Some(leg) = crate::procedure::active_leg() {
        return leg.fix;
    }
    // get_string lives on the concrete client, not the XPlane trait.
    xp.get_string(GPS_NAV_ID)
        .await
        .map(|s| s.trim().to_string())
        .unwrap_or_default()
}

fn arm_waypoint(
    base: String,
    from: String,
    life: Life,
    voice: Option<Voice>,
    status: Option<StatusSink>,
) {
    spawn(async move {
        let _guard = DoneGuard(life.done.clone());
        let xp = XPlaneClient::new(base);
        let deadline =
            std::time::Instant::now() + std::time::Duration::from_secs(REMINDER_DEADLINE_SECS);
        // The tracked fix sequences when we reach it: the id flips to the next
        // leg. Fire on that change (or when the id empties at the end). A
        // procedure that finishes hands the GPS's stale entry back; that is the
        // end of what we were following, not a new fix.
        let from_procedure = crate::procedure::active_leg().is_some();
        while std::time::Instant::now() < deadline {
            if life.cancelled() {
                return;
            }
            {
                let id = if from_procedure && crate::procedure::active_leg().is_none() {
                    String::new()
                } else {
                    tracked_fix(&xp).await
                };
                if id != from {
                    let line = if id.is_empty() {
                        format!("Heads up, reached {from}, end of the route.")
                    } else {
                        format!("Heads up, reached {from}, now tracking {id}.")
                    };
                    announce(&voice, &status, &line);
                    return;
                }
            }
            tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
        }
    });
}

/// The plan's legs and its TOD ring (nm to go along them at the top of
/// descent, `plan::tod_ring_nm`), resolved from the SimBrief plan. `None` if
/// there's no plan to work from.
async fn tod_ring(simbrief_id: &str) -> Option<(Vec<(f64, f64)>, f64)> {
    if simbrief_id.trim().is_empty() {
        return None;
    }
    let fp = crate::simbrief::fetch(simbrief_id).await.ok()?;
    let route: Vec<(f64, f64)> = fp.waypoints.iter().map(|w| (w.lat, w.lon)).collect();
    if route.is_empty() {
        return None;
    }
    let ring_nm = crate::plan::tod_ring_nm(&fp, &route);
    Some((route, ring_nm))
}

/// Distance to TOD this instant: the FMS's own VNAV TOD if it has one, else the
/// miles still to fly along the plan's legs minus the precomputed ring (#109:
/// the straight line to the field against an along-route ring called it
/// early on any winding route). `None` if unknowable.
async fn dist_to_tod(xp: &XPlaneClient, ring: Option<&(Vec<(f64, f64)>, f64)>) -> Option<f64> {
    if let Ok(d) = xp.get_float(TOD_DIST).await {
        if d > 0.0 {
            return Some(d);
        }
    }
    let (route, ring_nm) = ring?;
    let (lat, lon) = (xp.get_float(LAT).await.ok()?, xp.get_float(LON).await.ok()?);
    let to_go = crate::monitor::route_remaining_nm(route, lat, lon).or_else(|| {
        route
            .last()
            .map(|d| crate::nav::haversine_nm(lat, lon, d.0, d.1))
    })?;
    Some(to_go - ring_nm)
}

fn arm_tod(
    base: String,
    ring: Option<(Vec<(f64, f64)>, f64)>,
    life: Life,
    voice: Option<Voice>,
    status: Option<StatusSink>,
) {
    spawn(async move {
        let _guard = DoneGuard(life.done.clone());
        let xp = XPlaneClient::new(base);
        let deadline =
            std::time::Instant::now() + std::time::Duration::from_secs(REMINDER_DEADLINE_SECS);
        // Wait until TOD is genuinely ahead (a positive distance) before arming the
        // trigger, so arming while already descending doesn't fire instantly; then
        // call out when we're within a couple of miles.
        let mut seen_ahead = false;
        while std::time::Instant::now() < deadline {
            if life.cancelled() {
                return;
            }
            if let Some(d) = dist_to_tod(&xp, ring.as_ref()).await {
                if d > 2.0 {
                    seen_ahead = true;
                } else if seen_ahead {
                    announce(&voice, &status, "Heads up, approaching top of descent.");
                    return;
                }
            }
            tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
        }
    });
}
