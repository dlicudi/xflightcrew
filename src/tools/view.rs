//! `set_view` — move the camera / change the view.
//!
//! Fires X-Plane's standard view commands (and the user's own saved quick-look
//! 3D-cockpit presets). Same command mechanism as every other action, and there's
//! nothing to gate — a view change is harmless — so it's exposed on every airframe.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

/// A named view → (the X-Plane command that selects it, the spoken confirmation).
/// Synonyms fold to the same command so both the grammar's canonical value and a
/// looser LLM guess resolve.
fn named_view(name: &str) -> Option<(&'static str, &'static str)> {
    let v = match name {
        "forward" | "front" | "cockpit" | "center" | "centre" | "panel" | "instruments" => {
            ("sim/view/3d_cockpit_cmnd_look", "forward")
        }
        "external" | "outside" | "chase" => ("sim/view/chase", "external"),
        "spot" | "flyby" => ("sim/view/still_spot", "spot"),
        "orbit" | "circle" | "circling" => ("sim/view/linear_spot", "orbit"),
        "free" | "free_camera" => ("sim/view/free_camera", "free camera"),
        _ => return None,
    };
    Some(v)
}

pub struct SetView;

#[derive(Deserialize)]
struct ViewArgs {
    /// A named view (forward/external/spot/orbit/free).
    #[serde(default)]
    view: Option<String>,
    /// A saved quick-look preset, 1-based the way the pilot says it ("quick look 3").
    #[serde(default)]
    quick_look: Option<i64>,
}

#[async_trait]
impl Tool for SetView {
    fn name(&self) -> &str {
        "set_view"
    }
    fn description(&self) -> &str {
        "Change the camera view. Named views: 'forward' (3D cockpit), 'external' \
         (chase), 'spot', 'orbit', 'free'; or one of your saved quick-look presets \
         by number. Use for 'forward view', 'look forward', 'external view', \
         'outside', 'chase view', 'spot view', 'orbit', 'free camera', 'quick look \
         3', 'view 2'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "view": {"type": "string", "enum": ["forward", "external", "spot", "orbit", "free"],
                         "description": "Named view."},
                "quick_look": {"type": "integer", "minimum": 1, "maximum": 20,
                               "description": "Saved quick-look preset number (1-based)."}
            }
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: ViewArgs = serde_json::from_value(args.clone())?;
        // A quick-look preset wins if given. Spoken 1-based → 0-based command index.
        if let Some(n) = a.quick_look {
            let idx = (n - 1).clamp(0, 19);
            ctx.xplane.run_command(&format!("sim/view/quick_look_{idx}")).await?;
            return Ok(format!("Roger, quick look {}.", idx + 1));
        }
        match a.view.as_deref().and_then(named_view) {
            Some((cmd, spoken)) => {
                ctx.xplane.run_command(cmd).await?;
                Ok(format!("Roger, {spoken} view."))
            }
            None => Ok("Which view — forward, external, spot, orbit, or a quick look number?".to_string()),
        }
    }
}
