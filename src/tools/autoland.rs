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
}

#[derive(Deserialize, Default)]
struct Args {
    /// true = stop/disarm the autoland; false/absent = arm it.
    #[serde(default)]
    cancel: bool,
}

#[async_trait]
impl Tool for Autoland {
    fn name(&self) -> &str {
        "autoland"
    }
    fn description(&self) -> &str {
        "Autoland: fly a full approach and land at the nearest suitable runway \
         (the Garmin-style nearest-runway mode; the planned arrival is full auto's). \
         Use for 'land the aircraft', 'get us down', 'emergency landing', \
         'autoland', 'land us'. Pass cancel=true for 'cancel autoland' / 'stop the \
         autoland'. The pilot takes back control with 'my controls'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "cancel": {"type": "boolean", "description": "true = disarm/stop the autoland."}
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
        crate::autoland::ABORT.store(false, std::sync::atomic::Ordering::SeqCst);
        crate::autoland::run_autoland(
            ctx.config.xplane_base_url.clone(),
            apt_dat_path(),
            self.cfg.clone(),
            self.approach_ias,
            ctx.voice.cloned(),
            ctx.status.cloned(),
            None,
        );
        Ok("I will land. Searching for the nearest runway, then I'll fly the approach. \
            Say 'my controls' to take it back."
            .to_string())
    }
}
