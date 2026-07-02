//! `autoland` — native emergency autoland (no external plugin).
//!
//! x-copilot owns the *upset recovery* (`recover`); this flies a full approach and
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
    let home = std::env::var("HOME").unwrap_or_default();
    format!("{home}/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat")
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
        "Emergency autoland: fly a full approach and land at the nearest suitable \
         runway. Use for 'land the aircraft', 'get us down', 'emergency landing', \
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
                "Autoland cancelled. You have the aircraft.".to_string()
            } else {
                "No autoland running.".to_string()
            });
        }
        crate::autoland::ABORT.store(false, std::sync::atomic::Ordering::SeqCst);
        crate::autoland::run_autoland(
            ctx.config.xplane_base_url.clone(),
            apt_dat_path(),
            self.cfg.clone(),
            self.approach_ias,
            ctx.voice.cloned(),
            ctx.status.cloned(),
        );
        Ok("Roger, emergency autoland. Searching for the nearest runway, then flying \
            the approach. Say 'my controls' to take it back."
            .to_string())
    }
}
