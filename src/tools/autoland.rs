//! `autoland` — native autoland to the nearest suitable runway (no external plugin).
//!
//! XVoiceLayer owns the *upset recovery* (`recover`); this flies a full approach and
//! landing to the nearest suitable runway, entirely in-process: it parses X-Plane's
//! `apt.dat` for the runway ([`crate::autoland`]), then an AP-first control loop
//! tracks the final approach course and 3° glidepath, lowers gear/flaps, and eases
//! to idle in the flare. EXPERIMENTAL and untuned — arm it high, over an airport,
//! hand ready; "my controls" / "cancel autoland" hands it back instantly.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::PropProfile;
use crate::tools::autopilot::ApConfig;

/// Where the Global Airports apt.dat lives — `XCOPILOT_APT_DAT` overrides the
/// standard install path.
pub fn apt_dat_path() -> String {
    if let Ok(p) = std::env::var("XCOPILOT_APT_DAT") {
        return p;
    }
    xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat")
}

/// The X-Plane install the sim is running from — `XCOPILOT_XPLANE_ROOT`
/// overrides the standard location. Where an unknown airframe's `.acf` is
/// read from (`sim/aircraft/view/acf_relative_path` is relative to it).
pub fn xplane_root() -> String {
    crate::config::xplane_root()
}

/// A path under the X-Plane install (`xplane_root`), e.g.
/// `xplane_path("Custom Data/CIFP")`.
pub fn xplane_path(rel: &str) -> String {
    crate::config::xplane_path(rel)
}

pub struct Autoland {
    pub cfg: ApConfig,
    /// Approach speed (KIAS) — the loop holds this on final. From ~1.3×Vs.
    pub approach_ias: f64,
    #[allow(dead_code)]
    pub prop: Option<PropProfile>,
    /// Full auto's recovery planner: a landing request goes to the plan's
    /// destination from anywhere (`FullAuto::arm_landing`). `None` on an
    /// airframe full auto cannot fly: the nearest runway, as before.
    pub planner: Option<std::sync::Arc<crate::tools::full_auto::FullAuto>>,
}

#[derive(Deserialize, Default)]
struct Args {
    /// true = stop/disarm the autoland; false/absent = arm it.
    #[serde(default)]
    cancel: bool,
    /// true = the nearest suitable runway, whatever the plan ("land at the
    /// nearest", "emergency landing").
    #[serde(default)]
    nearest: bool,
}

/// The nearest suitable runway, by the self-pick's own rules and with no
/// destination to prefer: the airframe's minimum length, the METAR wind at
/// each field. Blackboxed `autoland` `self_pick` with `nearest` and why.
async fn nearest_runway(ctx: &ToolCtx<'_>, why: &str) -> Option<crate::autoland::Runway> {
    let (lat, lon, trk) = tokio::join!(
        ctx.xplane.get_float(crate::datarefs::LATITUDE),
        ctx.xplane.get_float(crate::datarefs::LONGITUDE),
        ctx.xplane.get_float(crate::datarefs::TRACK_DEG_TRUE),
    );
    let (lat, lon, trk) = (lat.ok()?, lon.ok()?, trk.unwrap_or(0.0));
    let apt = apt_dat_path();
    let picked = tokio::task::spawn_blocking(move || {
        let min = crate::runway::self_pick_min_length_m(crate::aircraft::landing_distance_m());
        let root = xplane_root();
        let wind_at = |icao: &str| crate::weather::surface_wind(&root, icao);
        let r = crate::runway::best_landing_runway_in_wind(&apt, lat, lon, trk, 40.0, min, &wind_at, None)
            .ok()
            .flatten();
        (r, min)
    })
    .await
    .ok()?;
    let (picked, min) = picked;
    crate::blackbox::record(
        "autoland",
        json!({ "event": "self_pick", "nearest": true, "why": why, "min_length_m": min,
                "destination": serde_json::Value::Null,
                "rwy": picked.as_ref().map(|r| format!("{} {}", r.icao.trim(), r.id)),
                "length_m": picked.as_ref().map(|r| r.length_m) }),
    );
    picked
}

#[async_trait]
impl Tool for Autoland {
    fn name(&self) -> &str {
        "autoland"
    }
    fn description(&self) -> &str {
        "Autoland: fly a full approach and land, at the flight plan's destination \
         from anywhere (full auto's recovery planner), else the nearest suitable \
         runway (nearest=true asks for it). \
         Use for 'land the aircraft', 'get us down', 'emergency landing', \
         'autoland', 'land us'. Pass cancel=true for 'cancel autoland' / 'stop the \
         autoland'. The pilot takes back control with 'my controls'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "cancel": {"type": "boolean", "description": "true = disarm/stop the autoland."},
                "nearest": {"type": "boolean", "description": "true = the nearest suitable runway, not the plan's destination."}
            }
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: Args = serde_json::from_value(args.clone()).unwrap_or_default();
        if a.cancel {
            return Ok(if crate::autoland::abort() {
                "Landing cancelled. You have the aircraft.".to_string()
            } else {
                "I'm not landing the aircraft.".to_string()
            });
        }
        // A replay can't be flown: the position it would arm against is a
        // recording, and the runway search + engaged AP would spring on the
        // pilot the moment the sim goes live. (The running loop pauses itself
        // on the same dataref.)
        if ctx
            .xplane
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5)
        {
            return Ok(
                "We're in replay, nothing to fly. Go back to live and ask again.".to_string(),
            );
        }
        // Re-arming mid-approach is destructive: it kills the flying loop and
        // spends ~6 s re-searching apt.dat with nobody on the controls (seen
        // live at LERL: three "autoland" calls in three minutes on short final).
        // An autoland already flying keeps flying.
        if crate::autoland::ACTIVE.load(std::sync::atomic::Ordering::SeqCst) {
            return Ok(
                "I'm already landing. Say 'my controls' first if you want me to start over."
                    .to_string(),
            );
        }
        // A mission flying the plan owns the aircraft: a second loop armed
        // under it would fight it for the autopilot.
        if crate::full_auto::running() {
            return Ok(
                "I'm flying the plan. Say my controls first, then land, or land at the nearest."
                    .to_string(),
            );
        }
        // The plan's destination first, from wherever we are: the climb to
        // the way's floor, the join, its approach. The nearest runway only
        // when asked, or when the planner says why not the destination.
        let mut why_nearest = a.nearest.then(|| "You asked for the nearest.".to_string());
        if why_nearest.is_none() {
            if let Some(planner) = &self.planner {
                match planner.arm_landing(ctx).await {
                    Ok(crate::tools::full_auto::LandArm::Armed(line)) => return Ok(line),
                    Ok(crate::tools::full_auto::LandArm::Nearest(why)) => why_nearest = Some(why),
                    Err(line) => return Ok(line),
                }
            }
        }
        let forced = match why_nearest.as_deref() {
            Some(why) => nearest_runway(ctx, why)
                .await
                .map(|r| (crate::autoland::PlannedRunway::at(r.icao.trim(), &r.id), r)),
            None => None,
        };
        let line = match (&why_nearest, &forced) {
            (Some(why), Some((_, r))) => format!(
                "{why} Landing at the nearest, {} runway {}, {:.0} miles. Say 'my controls' to take it back.",
                r.icao.trim(),
                atc::atis::spoken_runway(&r.id),
                r.dist_nm
            ),
            (Some(why), None) => format!(
                "{why} Searching for the nearest runway, then I'll fly the approach. Say 'my controls' to take it back."
            ),
            (None, _) => "I will land. Searching for the nearest runway, then I'll fly the approach. \
                Say 'my controls' to take it back."
                .to_string(),
        };
        crate::autoland::run_autoland_scoped(
            ctx.config.xplane_base_url.clone(),
            apt_dat_path(),
            self.cfg.clone(),
            self.approach_ias,
            ctx.voice.cloned(),
            ctx.status.cloned(),
            forced.map(|(p, _)| p),
            ctx.xplane.cancellation(),
        );
        Ok(line)
    }
}
