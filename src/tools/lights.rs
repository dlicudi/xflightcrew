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

use sim::xplane::XPlane;

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
    /// Does this airframe declare a light `key` would resolve to? The execute
    /// path answers an unknown key with an Ok QUESTION ("I didn't catch which
    /// light...") — right for a misheard pilot, unreadable for a caller
    /// judging success by Ok: full auto read it as the switch thrown and
    /// announced "Beacon on" on airframes with no beacon, blackboxing lights
    /// that were never touched (audit, 2026-08-02). A programmatic caller
    /// checks here first and stays silent about switches that don't exist.
    pub fn declares(&self, key: &str) -> bool {
        self.resolve_light(key).is_some()
    }

    fn resolve_light(&self, input: &str) -> Option<&LightProfile> {
        let want = input.trim().to_ascii_lowercase();
        if let Some(l) = self
            .lights
            .iter()
            .find(|l| l.key.eq_ignore_ascii_case(&want))
        {
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

/// Read one 0/1 switch dataref, which may be an array element (`name[i]`).
async fn read_switch(xp: &dyn XPlane, dr: &str) -> Option<bool> {
    let (base, idx) = split_index(dr);
    let v = match idx {
        Some(i) => xp.get_float_array(base).await.ok()?.get(i).copied()?,
        None => xp.get_float(base).await.ok()?,
    };
    Some(v > 0.5)
}

/// Drive one profile light to `on` and report the state it ACTUALLY reached.
///
/// The single writer for this, shared by the `set_lights` tool and the
/// copilot's own night-lights watch ([`crate::cockpit_auto`]) — the same rule
/// the transponder mode learned on the Twin Otter's KT73: two callers writing
/// one switch by two routes is how one of them silently stops working. Every
/// mechanism a profile can declare lives here (toggle-only switches read before
/// firing, a command that animates the panel, a plain write, and the extra
/// datarefs one "light" can be), and so does the readback every caller's trust
/// rests on.
pub async fn apply(xp: &dyn XPlane, light: &LightProfile, on: bool) -> Result<bool> {
    // Toggle-only lights (the Citation X: every light is a toggle, and its
    // datarefs are read-only over the API) — read each switch and fire only
    // where it differs, so "on" can never turn an already-on switch off.
    if !light.toggles.is_empty() {
        for t in &light.toggles {
            if read_switch(xp, &t.dataref).await != Some(on) {
                xp.run_command(&t.command).await?;
            }
        }
    } else {
        let cmd = if on {
            light.command_on.as_deref()
        } else {
            light.command_off.as_deref()
        };
        let (base, idx) = split_index(&light.dataref);
        match cmd {
            // Prefer the command: it animates the 3D switch, where a raw write
            // may set the value and leave the panel looking untouched.
            Some(c) => xp.run_command(c).await?,
            None => match idx {
                Some(i) => xp.set_int_array_elem(base, i, on as i64).await?,
                None => xp.set_int(base, on as i64).await?,
            },
        }
        // One panel light can be two switches (the Citation X's landing lights
        // are independent left and right; the Baron's roof floods are a left
        // and a right pod). Drive the rest to the same state, or the aircraft
        // flies half lit with a readback that confirms the half it happened to
        // look at.
        for extra in &light.also_datarefs {
            let (ebase, eidx) = split_index(extra);
            match eidx {
                Some(i) => xp.set_int_array_elem(ebase, i, on as i64).await?,
                None => xp.set_int(ebase, on as i64).await?,
            }
        }
    }
    // The readback is the point: confirm the ACTION, never the intent.
    read_switch(xp, &light.dataref)
        .await
        .ok_or_else(|| anyhow::anyhow!("readback failed for {}", light.dataref))
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
                return Ok(format!(
                    "I didn't catch which light. I have {}.",
                    names.join(", ")
                ));
            }
        };

        // One shared writer owns every mechanism and the readback (see
        // [`apply`]). The dataref may be an array element
        // (`landing_lights_switch[0]` on the SR22/Evolution) — the v1 lookup
        // 404s on the bracketed name, so it is split there (seen live: "switch
        // landing lights off" flipped the light, then the readback 404'd and
        // the whole tool reported failure).
        let actual_on = apply(ctx.xplane, light, args.on).await?;

        // The pilot reached for a switch the copilot also drives, so the
        // copilot drops its claim: daybreak must leave this where the PILOT
        // left it, not undo their decision hours later.
        if light.auto_at_night {
            crate::cockpit_auto::note_pilot_moved_lights();
        }

        // Confirm the ACTION against what was asked. The toggle path already
        // did this; the write path used to report whatever the readback said,
        // so a write that silently failed answered "Roger, landing lights off"
        // to "landing lights on" — a readback that confirms itself.
        Ok(if actual_on == args.on {
            format!(
                "Roger, {} {}.",
                light.spoken,
                if args.on { "on" } else { "off" }
            )
        } else {
            format!("The {} didn't move.", light.spoken)
        })
    }
}

/// The two cockpit flood lights. Kept separate from [`SetLights`] because the
/// floods use two different write mechanisms on the 172: the pilot side is a
/// scalar brightness dataref, the copilot side is one element of an array. A
/// single command drives both together.
const FLOOD_PILOT: &str = "sim/cockpit/electrical/cockpit_lights";
const FLOOD_COPILOT: &str = "sim/cockpit2/switches/panel_brightness_ratio";
const FLOOD_COPILOT_IDX: usize = 1;

/// Cockpit flood lights. By default drives the universal pilot+copilot flood
/// datarefs; a profile may override with a single on/off dataref (e.g. the
/// Laminar King Air's `generic_lights_switch[0]` overhead flood), which may be an
/// array element written as `name[i]`.
pub struct SetFloodLights {
    override_dataref: Option<String>,
    /// A flood switch that is a TOGGLE command with a read-only position, which
    /// no write can drive (the Baron 58). Takes precedence over
    /// `override_dataref`.
    toggle: Option<crate::aircraft::ToggleSwitch>,
}

impl SetFloodLights {
    pub fn new(
        override_dataref: Option<String>,
        toggle: Option<crate::aircraft::ToggleSwitch>,
    ) -> Self {
        Self {
            override_dataref,
            toggle,
        }
    }
}

/// Split a `name[i]` dataref into base + index; `(name, None)` if unindexed.
fn split_index(dr: &str) -> (&str, Option<usize>) {
    if let Some(open) = dr.rfind('[') {
        if let Some(inner) = dr.strip_suffix(']') {
            if let Ok(i) = inner[open + 1..].parse::<usize>() {
                return (&dr[..open], Some(i));
            }
        }
    }
    (dr, None)
}

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
            (None, None) => {
                return Ok(
                    "I didn't catch a flood-light setting — say on, off, or a percentage."
                        .to_string(),
                )
            }
        };

        // A toggle-command flood (the Baron 58: `flood_light_switch` is a
        // COMMAND, and its position dataref is read-only). Read first and fire
        // only if it differs — a toggle fired as "on" turns the light off
        // whenever it was already on, which is worse than having no tool.
        if let Some(t) = &self.toggle {
            let on = level > 0.5;
            if read_switch(ctx.xplane, &t.dataref).await != Some(on) {
                ctx.xplane.run_command(&t.command).await?;
            }
            return Ok(match read_switch(ctx.xplane, &t.dataref).await {
                Some(actual) if actual == on => {
                    format!("Roger, flood lights {}.", if on { "on" } else { "off" })
                }
                _ => "The flood lights didn't move.".to_string(),
            });
        }

        // Profile override: a single on/off switch (the King Air's overhead flood
        // is `generic_lights_switch[0]`, a switch, not a rheostat — so it's on/off,
        // not a brightness). Write the element (or scalar) and read it back.
        if let Some(dr) = &self.override_dataref {
            let on = level > 0.5;
            let (base, idx) = split_index(dr);
            match idx {
                Some(i) => ctx.xplane.set_int_array_elem(base, i, on as i64).await?,
                None => ctx.xplane.set_int(base, on as i64).await?,
            }
            let actual = match idx {
                Some(i) => ctx
                    .xplane
                    .get_float_array(base)
                    .await?
                    .get(i)
                    .copied()
                    .unwrap_or(0.0),
                None => ctx.xplane.get_float(base).await?,
            } > 0.5;
            // Confirm against the INTENT, not just report what came back: a
            // write that silently failed used to answer "Roger, flood lights
            // off" to "flood lights on" (the set_lights lesson).
            return Ok(if actual == on {
                format!("Roger, flood lights {}.", if on { "on" } else { "off" })
            } else {
                "The flood lights didn't move.".to_string()
            });
        }

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
#[allow(clippy::items_after_test_module)]
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
