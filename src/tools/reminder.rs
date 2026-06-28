//! Spoken reminders: "wake me at top of descent", "tell me when we reach 5000
//! feet", "remind me in 10 minutes". Each arms a one-shot background watcher (the
//! same fire-and-forget pattern as the autopilot capture watcher) that calls out
//! when its condition is met, spoken and mirrored to the panel.

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

pub struct SetReminder;

#[derive(Deserialize)]
struct Args {
    /// "top_of_descent" | "altitude" | "minutes".
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
        match a.kind.as_str() {
            "top_of_descent" => {
                // Resolve a TOD ring up front so the watcher fires even on avionics
                // with no VNAV (the 172's GNS530): SimBrief's planned TOD, else the
                // cruise-altitude rule of thumb. The watcher still prefers the FMS's
                // own TOD distance when the box publishes one.
                let ring = tod_ring(&ctx.config.simbrief_id).await;
                arm_tod(base, ring, voice, status);
                Ok("Roger, I'll call out the top of descent.".to_string())
            }
            "altitude" => {
                let target = a.value;
                let current = ctx.xplane.get_float(ALT_FT).await.unwrap_or(target);
                arm_altitude(base, target, current, voice, status);
                Ok(format!("Roger, I'll call out {target:.0} feet."))
            }
            "minutes" => {
                let mins = a.value.max(0.0);
                arm_timer(mins, voice, status);
                Ok(format!("Roger, I'll remind you in {mins:.0} minutes."))
            }
            "next_waypoint" => {
                // get_string lives on the concrete client, not the XPlane trait.
                let current = XPlaneClient::new(base.clone())
                    .get_string(GPS_NAV_ID)
                    .await
                    .unwrap_or_default();
                if current.trim().is_empty() {
                    return Ok("There's no active GPS waypoint to track. Load and navigate a route first.".to_string());
                }
                arm_waypoint(base, current.trim().to_string(), voice, status);
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
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            return;
        };
        rt.block_on(body);
    });
}

fn arm_timer(mins: f64, voice: Option<Voice>, status: Option<StatusSink>) {
    spawn(async move {
        tokio::time::sleep(std::time::Duration::from_secs_f64(mins * 60.0)).await;
        announce(&voice, &status, &format!("Heads up, that's {mins:.0} minutes."));
    });
}

fn arm_altitude(base: String, target: f64, current: f64, voice: Option<Voice>, status: Option<StatusSink>) {
    let climbing = target > current;
    spawn(async move {
        let xp = XPlaneClient::new(base);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(REMINDER_DEADLINE_SECS);
        while std::time::Instant::now() < deadline {
            if let Ok(alt) = xp.get_float(ALT_FT).await {
                let reached = if climbing { alt >= target - 50.0 } else { alt <= target + 50.0 };
                if reached {
                    announce(&voice, &status, &format!("Heads up, passing {target:.0} feet."));
                    return;
                }
            }
            tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
        }
    });
}

fn arm_waypoint(base: String, from: String, voice: Option<Voice>, status: Option<StatusSink>) {
    spawn(async move {
        let xp = XPlaneClient::new(base);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(REMINDER_DEADLINE_SECS);
        // The active GPS waypoint sequences when we reach it: the id flips to the
        // next leg. Fire on that change (or when the id empties at the end).
        while std::time::Instant::now() < deadline {
            if let Ok(id) = xp.get_string(GPS_NAV_ID).await {
                let id = id.trim().to_string();
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

/// Destination position and the TOD ring (nm before the destination), resolved
/// from the SimBrief plan: its planned TOD if the navlog has one, else the
/// cruise-altitude rule of thumb. `None` if there's no plan to work from.
async fn tod_ring(simbrief_id: &str) -> Option<(f64, f64, f64)> {
    if simbrief_id.trim().is_empty() {
        return None;
    }
    let fp = crate::simbrief::fetch(simbrief_id).await.ok()?;
    let dest = fp.waypoints.last()?;
    let ring_nm = if fp.tod_to_dest_nm > 0.0 {
        fp.tod_to_dest_nm
    } else {
        fp.cruise_alt_ft / 1000.0 * crate::TOD_NM_PER_1000FT
    };
    Some((dest.lat, dest.lon, ring_nm))
}

/// Distance to TOD this instant: the FMS's own VNAV TOD if it has one, else our
/// distance to the destination minus the precomputed ring. `None` if unknowable.
async fn dist_to_tod(xp: &XPlaneClient, ring: Option<(f64, f64, f64)>) -> Option<f64> {
    if let Ok(d) = xp.get_float(TOD_DIST).await {
        if d > 0.0 {
            return Some(d);
        }
    }
    let (dlat, dlon, ring_nm) = ring?;
    let (lat, lon) = (xp.get_float(LAT).await.ok()?, xp.get_float(LON).await.ok()?);
    Some(crate::haversine_nm(lat, lon, dlat, dlon) - ring_nm)
}

fn arm_tod(base: String, ring: Option<(f64, f64, f64)>, voice: Option<Voice>, status: Option<StatusSink>) {
    spawn(async move {
        let xp = XPlaneClient::new(base);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(REMINDER_DEADLINE_SECS);
        // Wait until TOD is genuinely ahead (a positive distance) before arming the
        // trigger, so arming while already descending doesn't fire instantly; then
        // call out when we're within a couple of miles.
        let mut seen_ahead = false;
        while std::time::Instant::now() < deadline {
            if let Some(d) = dist_to_tod(&xp, ring).await {
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
