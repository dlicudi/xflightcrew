//! `set_flaps` — flap control, driven by the active aircraft's flap profile.
//!
//! The detents, their ratios, and per-notch Vfe all come from the
//! [`FlapProfile`], so the same tool serves any airframe. It demonstrates the
//! pattern every tool follows:
//!   1. owns its dataref + valid values (the profile's detents — no arbitrary writes),
//!   2. a safety gate (refuse extending past a detent's Vfe),
//!   3. a readback that re-reads the dataref *after* writing and confirms it.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::{FlapDetent, FlapProfile};

/// X-Plane flap handle position, 0.0 (up) .. 1.0 (full). Writable; the actual
/// surfaces animate toward it. Reading it back gives the commanded notch.
const FLAP_HANDLE: &str = crate::datarefs::FLAP_RATIO;
/// Indicated airspeed at the pilot's ASI, in knots — used by the safety gate.
const IAS_KTS: &str = crate::datarefs::AIRSPEED_KTS;

pub struct SetFlaps {
    flaps: FlapProfile,
}

#[derive(Deserialize)]
struct Args {
    /// Detent label, one of the profile's detents (e.g. "up", "10", "full").
    notch: String,
}

impl SetFlaps {
    pub fn new(flaps: FlapProfile) -> Self {
        Self { flaps }
    }

    /// The detent labels, for the schema enum and the description.
    fn labels(&self) -> Vec<&str> {
        self.flaps
            .detents
            .iter()
            .map(|d| d.label.as_str())
            .collect()
    }
}

#[async_trait]
impl Tool for SetFlaps {
    fn name(&self) -> &str {
        "set_flaps"
    }

    fn description(&self) -> &str {
        // Static text; the concrete notches are conveyed via the enum schema.
        "Set the wing flaps to a detent. Use for commands like 'flaps up', \
         'flaps ten', 'flaps twenty', 'flaps full', 'retract the flaps', \
         'bring the flaps up', 'give me ten degrees of flap', 'approach flaps', \
         'flaps for landing', 'milk the flaps up', 'flaps down'. 'up' is \
         retracted; higher numbers are more flap; 'down' means extend."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "notch": {
                    "type": "string",
                    "enum": self.labels(),
                    "description": "The flap detent to select."
                }
            },
            "required": ["notch"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;

        // Map the requested label to a known detent. Anything else is rejected —
        // the LLM can only command real detents, never an arbitrary ratio.
        let target = match self.resolve(&args.notch) {
            Some(d) => d,
            None => {
                let names: Vec<&str> = self
                    .flaps
                    .detents
                    .iter()
                    .map(|d| d.spoken.as_str())
                    .collect();
                return Ok(format!(
                    "I didn't catch the flap setting. Options are {}.",
                    names.join(", ")
                ));
            }
        };

        let current_ratio = ctx.xplane.get_float(FLAP_HANDLE).await?;

        // Already at the commanded detent? Say so instead of re-commanding.
        if (target.ratio - current_ratio).abs() <= crate::aircraft::FLAP_DETENT_TOLERANCE {
            return Ok(format!("Flaps already {}.", target.spoken));
        }

        // Safety gate: refuse *extending* above the target's placard (the
        // one lookup, `aircraft::flap_placard_kts`: the lowest Vfe at or
        // inside the target detent).
        let extending = target.ratio > current_ratio + 0.01;
        if extending {
            let limits: Vec<(f64, Option<f64>)> =
                self.flaps.detents.iter().map(|d| (d.ratio, d.vfe_kts)).collect();
            if let Some(vfe) = crate::aircraft::flap_placard_kts(target.ratio, &limits) {
                let ias = ctx.xplane.get_float(IAS_KTS).await?;
                if ias > vfe {
                    return Ok(format!(
                        "Negative — {ias:.0} knots is above the {vfe:.0}-knot limit for flaps {}. Holding {}.",
                        target.spoken,
                        self.describe(current_ratio),
                    ));
                }
            }
        }

        // Write, then read back the actual value and confirm *that*.
        ctx.xplane.set_float(FLAP_HANDLE, target.ratio).await?;
        let actual = ctx.xplane.get_float(FLAP_HANDLE).await?;

        Ok(format!("Roger, flaps {}.", self.describe(actual)))
    }
}

impl SetFlaps {
    /// Resolve a requested notch to a detent. Exact label match first (a profile
    /// may declare its own "full" or "down"); then the semantic aliases every
    /// airframe understands — "landing"/"land"/"full"/"down" all mean the last
    /// detent, whatever it's labelled (the MD-82's is "40", the 172's "full").
    /// The per-notch Vfe gate still applies, so "flaps for landing" while fast
    /// refuses rather than dumping full flap.
    fn resolve(&self, notch: &str) -> Option<&FlapDetent> {
        if let Some(d) = self
            .flaps
            .detents
            .iter()
            .find(|d| d.label.eq_ignore_ascii_case(notch))
        {
            return Some(d);
        }
        if ["landing", "land", "full", "down"]
            .iter()
            .any(|a| notch.eq_ignore_ascii_case(a))
        {
            return self
                .flaps
                .detents
                .iter()
                .max_by(|a, b| a.ratio.total_cmp(&b.ratio));
        }
        None
    }

    /// Map an observed flap ratio to the nearest detent's spoken phrase.
    fn describe(&self, ratio: f64) -> &str {
        self.flaps
            .detents
            .iter()
            .min_by(|a, b| {
                (a.ratio - ratio)
                    .abs()
                    .partial_cmp(&(b.ratio - ratio).abs())
                    .unwrap()
            })
            .map(|d| d.spoken.as_str())
            .unwrap_or("set")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft;

    fn cessna_flaps() -> FlapProfile {
        aircraft::builtin("cessna_172sp").unwrap().flaps.unwrap()
    }

    #[test]
    fn enum_labels_track_the_profile() {
        let t = SetFlaps::new(cessna_flaps());
        assert_eq!(t.labels(), vec!["up", "10", "20", "full"]);
    }

    #[test]
    fn describe_snaps_to_nearest_detent() {
        let t = SetFlaps::new(cessna_flaps());
        assert_eq!(t.describe(0.0), "up");
        assert_eq!(t.describe(0.05), "up"); // slightly off still reads as up
        assert_eq!(t.describe(1.0 / 3.0), "ten degrees");
        assert_eq!(t.describe(0.66), "twenty degrees");
        assert_eq!(t.describe(1.0), "full");
    }

    #[test]
    fn parameters_expose_only_real_detents() {
        let schema = SetFlaps::new(cessna_flaps()).parameters();
        let enum_vals = &schema["properties"]["notch"]["enum"];
        assert_eq!(enum_vals, &serde_json::json!(["up", "10", "20", "full"]));
    }

    #[test]
    fn landing_alias_resolves_to_last_detent() {
        let t = SetFlaps::new(cessna_flaps());
        // Exact labels win; the aliases land on the highest-ratio detent.
        assert_eq!(t.resolve("10").unwrap().label, "10");
        assert_eq!(t.resolve("full").unwrap().label, "full"); // real label, not alias
        for alias in ["landing", "land", "LANDING", "down"] {
            assert_eq!(t.resolve(alias).unwrap().label, "full", "{alias}");
        }
        assert!(t.resolve("crazy").is_none());
    }
}
