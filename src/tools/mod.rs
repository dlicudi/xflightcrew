//! The curated tool surface.
//!
//! Core design rule (from the brief): the LLM never writes arbitrary datarefs.
//! It only picks one of these explicit tools. Each tool owns its dataref(s),
//! valid range, safety gates, and — crucially — a *readback* that re-reads the
//! dataref after writing and confirms the actual value. A copilot confirms the
//! action, not the intent.
//!
//! The [`Tool`] schema is backend-neutral (`name` + `description` +
//! JSON-schema `parameters`); [`crate::llm`] wraps it into whatever the local
//! LLM server expects.

pub mod approach;
pub mod autopilot;
pub mod controls;
pub mod electrical;
pub mod engine;
pub mod flaps;
pub mod instruments;
pub mod lights;
pub mod radio;

use anyhow::Result;
use async_trait::async_trait;
use serde_json::Value;

use crate::aircraft::AircraftProfile;
use crate::config::Config;
use crate::xplane::XPlaneClient;

/// Shared context handed to every tool invocation.
pub struct ToolCtx<'a> {
    pub xplane: &'a XPlaneClient,
    // Available to tools that need global config; unused by `set_flaps` (its
    // limits come from the aircraft profile).
    #[allow(dead_code)]
    pub config: &'a Config,
}

/// One curated action the copilot can take.
#[async_trait]
pub trait Tool: Send + Sync {
    /// Stable tool name the LLM selects by (e.g. `set_flaps`).
    fn name(&self) -> &str;

    /// Human/LLM-facing description of when to use this tool.
    fn description(&self) -> &str;

    /// JSON Schema for the tool's arguments (an `object` schema).
    fn parameters(&self) -> Value;

    /// If this action is safety-critical, return a spoken description of the
    /// *interpreted* command so the pilot can confirm it before anything moves
    /// ("descend seven hundred feet per minute"). `None` = execute immediately.
    /// This is the guard against a misheard command being flown blind.
    fn confirm_prompt(&self, _args: &Value) -> Option<String> {
        None
    }

    /// Execute the action and return the spoken readback line.
    ///
    /// On a refused safety gate this still returns `Ok` — the returned string is
    /// the explanation to speak back ("Negative, too fast for flaps"). Only
    /// genuine failures (X-Plane unreachable, bad dataref) return `Err`.
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String>;
}

/// The set of tools exposed to the LLM for one session.
pub struct Registry {
    tools: Vec<Box<dyn Tool>>,
}

impl Registry {
    /// Build the tool set for an aircraft. The profile shapes the surface: the
    /// 172 has no retractable gear, so no `gear` tool is added for it. Each
    /// system present on the airframe contributes its tool(s).
    pub fn for_aircraft(profile: &AircraftProfile) -> Self {
        let mut tools: Vec<Box<dyn Tool>> = Vec::new();

        if let Some(flaps) = &profile.flaps {
            tools.push(Box::new(flaps::SetFlaps::new(flaps.clone())));
        }

        if !profile.lights.is_empty() {
            tools.push(Box::new(lights::SetLights::new(profile.lights.clone())));
        }

        if profile.flood_lights {
            tools.push(Box::new(lights::SetFloodLights));
        }

        if profile.autopilot {
            tools.push(Box::new(autopilot::SetHeading));
            tools.push(Box::new(autopilot::SetAltitude));
            tools.push(Box::new(autopilot::SetVerticalSpeed));
            tools.push(Box::new(autopilot::LevelOff));
            tools.push(Box::new(autopilot::AutopilotOff));
        }

        if profile.engine {
            tools.push(Box::new(engine::SetThrottle));
            tools.push(Box::new(engine::SetMixture));
            tools.push(Box::new(engine::StartEngine));
            tools.push(Box::new(engine::StopEngine));
        }

        if profile.fuel_pump {
            tools.push(Box::new(engine::SetFuelPump));
        }

        if profile.magnetos {
            tools.push(Box::new(engine::SetMagnetos));
        }

        if profile.carb_heat {
            tools.push(Box::new(engine::SetCarbHeat));
        }

        if let Some(fs) = &profile.fuel_selector {
            tools.push(Box::new(engine::SetFuelSelector::new(fs.clone())));
        }

        if profile.parking_brake {
            tools.push(Box::new(controls::SetParkingBrake));
        }

        if profile.pitch_trim {
            tools.push(Box::new(controls::SetPitchTrim));
        }

        if profile.pitot_heat {
            tools.push(Box::new(instruments::SetPitotHeat));
        }

        if !profile.electrical.is_empty() {
            tools.push(Box::new(electrical::SetElectrical::new(profile.electrical.clone())));
        }

        if profile.barometer {
            tools.push(Box::new(instruments::SetBarometer));
        }

        if profile.directional_gyro {
            tools.push(Box::new(instruments::AlignGyro));
        }

        if profile.com_radio {
            tools.push(Box::new(radio::SetCom1));
        }

        if profile.com2_radio {
            tools.push(Box::new(radio::SetCom2));
            tools.push(Box::new(radio::SwapRadio));
        }

        if profile.nav_radio {
            tools.push(Box::new(radio::SetNav1));
            tools.push(Box::new(radio::SetObs));
        }

        if profile.transponder {
            tools.push(Box::new(radio::SetTransponder));
            tools.push(Box::new(radio::SetTransponderMode));
        }

        if profile.ils {
            tools.push(Box::new(approach::PrepareIls));
        }

        // Checklists are NOT registered as one-shot tools — the copilot runs them
        // interactively (narrate + act + verify + call out the pilot's items),
        // which needs the voice, so the main loop intercepts the pick. They're
        // interpreted from the airframe's XChecklist file (see [`crate::checklist`]).

        // Future, gated the same way:
        //   if profile.retractable_gear { tools.push(Box::new(gear::Gear::new())); }
        //   fuel pump, pitot heat, COM2/NAV radios, transponder, …

        Self { tools }
    }

    /// Look up a tool by the name the LLM returned.
    pub fn get(&self, name: &str) -> Option<&dyn Tool> {
        self.tools
            .iter()
            .find(|t| t.name() == name)
            .map(|t| t.as_ref())
    }

    /// Backend-neutral specs for every tool, for the LLM request.
    pub fn specs(&self) -> Vec<ToolSpec> {
        self.tools
            .iter()
            .map(|t| ToolSpec {
                name: t.name().to_string(),
                description: t.description().to_string(),
                parameters: t.parameters(),
            })
            .collect()
    }
}

/// A flattened, serializable view of a tool for the LLM layer to wrap.
pub struct ToolSpec {
    pub name: String,
    pub description: String,
    pub parameters: Value,
}
