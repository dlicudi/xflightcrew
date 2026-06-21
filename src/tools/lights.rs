//! `set_lights` — switch an exterior/interior light on or off.
//!
//! Like `set_flaps`, the set of switchable lights comes from the aircraft
//! profile, so the LLM's `light` enum only ever contains lights the airframe
//! actually has. Same three-part pattern: own the dataref + valid values, no
//! safety gate needed here, read back the switch and confirm the actual state.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::LightProfile;

pub struct SetLights {
    lights: Vec<LightProfile>,
}

#[derive(Deserialize)]
struct Args {
    /// Light key, one of the profile's lights (e.g. "landing", "strobe").
    light: String,
    /// Desired state: true = on, false = off.
    on: bool,
}

impl SetLights {
    pub fn new(lights: Vec<LightProfile>) -> Self {
        Self { lights }
    }

    fn keys(&self) -> Vec<&str> {
        self.lights.iter().map(|l| l.key.as_str()).collect()
    }

    /// Resolve a (possibly misheard) light name to one of the profile's lights.
    /// Exact key match first; if that fails, snap an out-of-enum value to the
    /// nearest key within a tight edit distance. The adversarial battery showed
    /// *every* tested model passes whisper mishearings straight through — e.g.
    /// "strobe" → "probe"/"probes" — and the LLM can't be relied on to correct
    /// them. The threshold is deliberately tight (≤2) and plurals are normalized
    /// first, so genuine unknowns ("disco lights") still fall through to the
    /// clarifying prompt rather than snapping to something arbitrary.
    fn resolve_light(&self, input: &str) -> Option<&LightProfile> {
        let want = input.trim().to_ascii_lowercase();
        if let Some(l) = self.lights.iter().find(|l| l.key.eq_ignore_ascii_case(&want)) {
            return Some(l);
        }
        let stripped = want.strip_suffix('s').unwrap_or(want.as_str());
        self.lights
            .iter()
            .map(|l| (crate::levenshtein(stripped, &l.key.to_ascii_lowercase()), l))
            .filter(|(d, _)| *d <= 2)
            .min_by_key(|(d, _)| *d)
            .map(|(_, l)| l)
    }
}

#[async_trait]
impl Tool for SetLights {
    fn name(&self) -> &str {
        "set_lights"
    }

    fn description(&self) -> &str {
        "Turn an exterior or interior light on or off. Use for commands like \
         'landing light on', 'strobes off', 'beacon on', 'taxi light off', \
         'nav lights on', 'give me the landing light', 'kill the strobes', \
         'turn off the beacon', 'position lights on'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "light": {
                    "type": "string",
                    "enum": self.keys(),
                    "description": "Which light to switch."
                },
                "on": {
                    "type": "boolean",
                    "description": "true to turn on, false to turn off."
                }
            },
            "required": ["light", "on"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;

        let light = match self.resolve_light(&args.light) {
            Some(l) => l,
            None => {
                // Don't hard-fail (e.g. whisper hears "beak" for "beacon") —
                // ask, listing what we actually have.
                let names: Vec<&str> = self.lights.iter().map(|l| l.spoken.as_str()).collect();
                return Ok(format!("I didn't catch which light. I have {}.", names.join(", ")));
            }
        };

        // Prefer the command (it animates the panel switch); fall back to a raw
        // dataref write if the profile doesn't supply one. Either way, read the
        // dataref back and confirm the *actual* state.
        let cmd = if args.on {
            light.command_on.as_deref()
        } else {
            light.command_off.as_deref()
        };
        match cmd {
            Some(c) => ctx.xplane.run_command(c).await?,
            None => ctx.xplane.set_int(&light.dataref, args.on as i64).await?,
        }
        let actual_on = ctx.xplane.get_float(&light.dataref).await? > 0.5;

        Ok(format!(
            "Roger, {} {}.",
            light.spoken,
            if actual_on { "on" } else { "off" }
        ))
    }
}

/// The two cockpit flood lights. Kept separate from [`SetLights`] because the
/// floods use two different write mechanisms on the 172: the pilot side is a
/// scalar brightness dataref, the copilot side is one element of an array. A
/// single command drives both together.
const FLOOD_PILOT: &str = "sim/cockpit/electrical/cockpit_lights";
const FLOOD_COPILOT: &str = "sim/cockpit2/switches/panel_brightness_ratio";
const FLOOD_COPILOT_IDX: usize = 1;

pub struct SetFloodLights;

#[derive(Deserialize)]
struct FloodArgs {
    /// On/off shortcut (on = full bright, off = dark).
    #[serde(default)]
    on: Option<bool>,
    /// Brightness 0-100 percent. Takes precedence over `on` when given.
    #[serde(default)]
    percent: Option<i64>,
}

#[async_trait]
impl Tool for SetFloodLights {
    fn name(&self) -> &str {
        "set_flood_lights"
    }

    fn description(&self) -> &str {
        "Set the cockpit flood lights (both pilot and copilot together) — on, off, \
         or a brightness percentage. Use for 'flood lights on', 'flood lights off', \
         'flood lights fifty percent', 'dim the floods to 30', 'flood lights full'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "on": {"type": "boolean", "description": "true = full bright, false = off. Use for plain on/off."},
                "percent": {"type": "integer", "minimum": 0, "maximum": 100,
                            "description": "Brightness 0-100%. Use when a level is named, e.g. 'floods to 40 percent'."}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: FloodArgs = serde_json::from_value(args.clone())?;
        // percent wins if given; else the on/off shortcut; else we didn't catch it.
        let level = match (args.percent, args.on) {
            (Some(p), _) => (p.clamp(0, 100) as f64) / 100.0,
            (None, Some(true)) => 1.0,
            (None, Some(false)) => 0.0,
            (None, None) => return Ok("I didn't catch a flood-light setting — say on, off, or a percentage.".to_string()),
        };

        // Pilot side: scalar brightness. Copilot side: one float array element.
        ctx.xplane.set_float(FLOOD_PILOT, level).await?;
        ctx.xplane
            .set_float_array_elem(FLOOD_COPILOT, FLOOD_COPILOT_IDX, level)
            .await?;

        // Read both back and confirm they actually moved.
        let pilot = ctx.xplane.get_float(FLOOD_PILOT).await?;
        let copilot = ctx
            .xplane
            .get_float_array(FLOOD_COPILOT)
            .await?
            .get(FLOOD_COPILOT_IDX)
            .copied()
            .unwrap_or(0.0);

        // If the two sides disagree by more than a touch, flag it.
        if (pilot - copilot).abs() > 0.1 {
            return Ok(format!(
                "Heads up — pilot flood {}, copilot flood {}.",
                describe_level(pilot),
                describe_level(copilot)
            ));
        }
        Ok(format!("Roger, flood lights {}.", describe_level(pilot)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft;

    /// The `probe`→`strobe` miss the adversarial battery surfaced on every
    /// model: confirm the tool snaps the real misheard values to `strobe`,
    /// regardless of which model produced them, while leaving exact names alone
    /// and refusing to snap a genuine unknown.
    #[test]
    fn snaps_misheard_light_names() {
        let profile = aircraft::builtin("cessna_172sp").unwrap();
        let tool = SetLights::new(profile.lights);
        let key = |s: &str| tool.resolve_light(s).map(|l| l.key.clone());

        // Values the benchmarked models actually emitted for "strobe lights".
        assert_eq!(key("probe").as_deref(), Some("strobe"));
        assert_eq!(key("probes").as_deref(), Some("strobe"));
        // Plural of the real key should resolve too.
        assert_eq!(key("strobes").as_deref(), Some("strobe"));
        // Exact names are untouched.
        assert_eq!(key("landing").as_deref(), Some("landing"));
        assert_eq!(key("beacon").as_deref(), Some("beacon"));
        // A genuine unknown stays unresolved (→ clarifying prompt, not a guess).
        assert_eq!(key("disco"), None);
    }
}

/// Spoken description of a flood brightness level (0.0–1.0).
fn describe_level(level: f64) -> String {
    let pct = (level * 100.0).round();
    if pct <= 0.0 {
        "off".to_string()
    } else if pct >= 100.0 {
        "on".to_string()
    } else {
        format!("{pct:.0} percent")
    }
}
