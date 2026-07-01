//! `autoland` — hand off to the Emergency Autoland plugin.
//!
//! x-copilot owns the *upset recovery* (`recover`); the actual approach-and-land
//! is a separate, mature XPPython3 plugin (`~/GitHub/xplane-emergency-autoland`).
//! This tool just arms it, by firing its toggle command over the Web API — so
//! "land the aircraft" chains recovery → autoland → touchdown, all by voice.
//!
//! The plugin's master is a *toggle* with no readable armed state, so `cancel`
//! fires the same command; the pilot drives which they mean. If the plugin isn't
//! loaded the command won't resolve — we say so rather than fail the action.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

/// The Emergency Autoland plugin's master toggle command.
const CMD_AUTOLAND_TOGGLE: &str = "duanelicudi/landingassist/toggle";

pub struct Autoland;

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
        "Arm the emergency autoland to fly an approach and land at the nearest \
         suitable runway (hands off to the Emergency Autoland plugin). Use for \
         'land the aircraft', 'get us down', 'emergency landing', 'autoland', 'land \
         us'. Pass cancel=true for 'cancel autoland' / 'stop the autoland'."
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
        match ctx.xplane.run_command(CMD_AUTOLAND_TOGGLE).await {
            Ok(()) if a.cancel => Ok("Cancelling the autoland. You have the aircraft.".to_string()),
            Ok(()) => Ok(
                "Roger, emergency autoland armed. It'll fly the approach to the nearest \
                 runway and land. Say 'cancel autoland' to stop."
                    .to_string(),
            ),
            // The command didn't resolve — the autoland plugin isn't loaded.
            Err(_) => Ok(
                "I can't reach the autoland — check the Emergency Autoland plugin is installed and loaded."
                    .to_string(),
            ),
        }
    }
}
