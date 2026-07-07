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
pub mod autoland;
pub mod autopilot;
pub mod controls;
pub mod cycle;
pub mod electrical;
pub mod engine;
pub mod flaps;
pub mod gear;
pub mod jet_start;
pub mod ground;
pub mod instruments;
pub mod lights;
pub mod power;
pub mod prop;
pub mod radio;
pub mod recover;
pub mod speedhold;
pub mod reminder;

use anyhow::Result;
use async_trait::async_trait;
use serde_json::Value;

use crate::aircraft::AircraftProfile;
use crate::config::Config;
use crate::voice::Voice;
use crate::xplane::XPlane;

// Datarefs/commands shared by more than one tool module, defined once here so the
// two readers can't silently drift. Datarefs private to a single tool stay in
// that tool's module.

/// Parking-brake ratio dataref (0 = off, 1 = set). Written by `set_parking_brake`
/// and read by `stop_engine`'s pre-shutdown check.
pub(crate) const PARK_BRAKE: &str = "sim/cockpit2/controls/parking_brake_ratio";
/// Command: engage the autopilot servos. Pressed by the AP-mode tools
/// (`set_heading`/`set_altitude`/…) and by the ILS setup in `approach`.
pub(crate) const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";

/// Shared context handed to every tool invocation.
pub struct ToolCtx<'a> {
    pub xplane: &'a dyn XPlane,
    // Available to tools that need global config; unused by `set_flaps` (its
    // limits come from the aircraft profile).
    #[allow(dead_code)]
    pub config: &'a Config,
    /// The speech arbiter, for the rare action that announces *later* than its
    /// readback: the autopilot capture watcher calls out the level-off when it
    /// reaches the target, minutes after the command. `None` outside the live loop.
    pub voice: Option<&'a Voice>,
    /// The in-sim panel sink, so a later announcement (the level-off callout) shows
    /// on the panel too, not just spoken. `None` outside the live loop / no plugin.
    pub status: Option<&'a crate::plugin_status::StatusSink>,
    /// The armed-reminder registry, so `set_reminder` records what it arms and the
    /// copilot can recall/cancel them. `None` outside the live loop.
    pub reminders: Option<&'a reminder::Reminders>,
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

        if profile.retractable_gear {
            tools.push(Box::new(gear::Gear { vlo_kts: profile.vlo_kts }));
        }

        if !profile.lights.is_empty() {
            tools.push(Box::new(lights::SetLights::new(profile.lights.clone())));
        }

        if profile.flood_lights {
            tools.push(Box::new(lights::SetFloodLights::new(profile.flood_lights_dataref.clone())));
        }

        if profile.autopilot {
            // Per-airframe engagement prerequisites (FD switch, AC power) — the C90
            // needs them; the GA singles leave this default (plain servos-on).
            let prep = profile.autopilot_prep.clone().unwrap_or_default();
            // Altitude gate uses the airframe's real service ceiling (the C90B is
            // 28,100 ft) rather than the GA default, so a valid high climb isn't refused.
            let ceiling = profile
                .reference
                .as_ref()
                .and_then(|r| r.service_ceiling_ft)
                .unwrap_or(autopilot::ApConfig::default().service_ceiling_ft);
            let ap = autopilot::ApConfig {
                fd_commands: prep.flight_director,
                ac_powered: prep.ac_powered,
                service_ceiling_ft: ceiling,
            };
            tools.push(Box::new(autopilot::SetHeading { cfg: ap.clone() }));
            tools.push(Box::new(autopilot::Navigate { cfg: ap.clone() }));
            tools.push(Box::new(autopilot::SetAltitude { cfg: ap.clone() }));
            tools.push(Box::new(autopilot::SetVerticalSpeed { cfg: ap.clone() }));
            tools.push(Box::new(autopilot::SetClimb { cfg: ap.clone() }));
            tools.push(Box::new(autopilot::LevelOff { cfg: ap.clone() }));
            tools.push(Box::new(autopilot::AutopilotOff));
            // Emergency upset/stall recovery — takes the controls, so it needs the
            // autopilot (for the level-hold handoff) present. It also drives the
            // props full forward for max power, and holds ≈1.3×Vs.
            let vs = profile.vs_clean_kts.unwrap_or(60.0);
            tools.push(Box::new(recover::Recover {
                cfg: ap.clone(),
                prop: profile.prop.clone(),
                stall_speed_kts: vs,
            }));
            // Native emergency autoland — needs the AP for approach guidance. Prefer
            // the airframe's real approach speed; fall back to ~1.3×Vs.
            let approach_ias = profile
                .reference
                .as_ref()
                .and_then(|r| r.approach_kts)
                .unwrap_or((vs * 1.3).max(70.0));
            tools.push(Box::new(autoland::Autoland {
                cfg: ap,
                approach_ias,
                prop: profile.prop.clone(),
            }));
        }

        // Thrust levers: any piston engine, or a turbine that declares `throttle`.
        if profile.engine || profile.throttle {
            tools.push(Box::new(engine::SetThrottle));
        }
        // Constant-speed propeller levers, when the airframe declares them.
        if let Some(prop) = &profile.prop {
            tools.push(Box::new(prop::SetProp::new(prop.clone())));
        }
        // Turbine start sequence (jets) — distinct from the piston one-shot start.
        if profile.turbine_start {
            tools.push(Box::new(jet_start::StartEngines));
            tools.push(Box::new(jet_start::StopEngines));
        }
        // Procedural electrical power-up (step-by-step with callouts).
        if profile.auto_prep {
            tools.push(Box::new(power::PowerUp));
        }
        // Talk-down (spoken visual-approach guidance) — any airframe, any
        // runway; needs nothing but a voice and apt.dat.
        tools.push(Box::new(approach::VisualApproach));

        // Speed hold (copilot works the throttle) — airframes with a throttle
        // and no autothrottle; the MD-82 crowd sets the AP speed bug instead.
        if (profile.engine || profile.throttle) && !profile.autothrottle {
            let approach = profile.reference.as_ref().and_then(|r| r.approach_kts);
            tools.push(Box::new(speedhold::HoldSpeed { approach_kts: approach }));
        }

        // Airliner autoflight extras: AP speed select + autothrottle.
        if profile.autothrottle {
            let approach = profile.reference.as_ref().and_then(|r| r.approach_kts);
            tools.push(Box::new(autopilot::SetSpeed::new(approach)));
            tools.push(Box::new(autopilot::Autothrottle));
        }
        if profile.engine {
            tools.push(Box::new(engine::SetMixture));
            tools.push(Box::new(engine::LeanMixture));
            tools.push(Box::new(engine::StartEngine {
                fuel_selector: profile.fuel_selector.clone(),
                alternator_on_cmds: alternator_on_cmds(&profile.electrical),
            }));
            tools.push(Box::new(engine::StopEngine));
        }

        if profile.fuel_pump {
            tools.push(Box::new(engine::SetFuelPump { engines: profile.engines }));
        }

        if profile.magnetos {
            tools.push(Box::new(engine::SetMagnetos {
                engines: profile.engines,
                per_engine: profile.magnetos_per_engine,
            }));
        }

        if profile.carb_heat {
            tools.push(Box::new(engine::SetCarbHeat { engines: profile.engines }));
        }

        if let Some(fs) = &profile.fuel_selector {
            tools.push(Box::new(engine::SetFuelSelector::new(fs.clone())));
        }

        // Cycle-switch systems (the AWX DC-3): power switch, per-side fuel
        // selectors, and the twin piston start/shutdown. These drive xlua-
        // overridden systems via up/down cycle commands from the profile.
        if let Some(ps) = &profile.power_switch {
            tools.push(Box::new(cycle::SetPower::new(ps.clone())));
        }
        if !profile.fuel_selectors.is_empty() {
            tools.push(Box::new(cycle::SetCycleFuelSelector::new(profile.fuel_selectors.clone())));
        }
        if let Some(start) = &profile.piston_start {
            tools.push(Box::new(cycle::StartEngines::new(
                start.clone(),
                profile.fuel_selectors.clone(),
                profile.power_switch.clone(),
            )));
            tools.push(Box::new(cycle::StopEngines::new(start.clone())));
        }
        if let Some(mix) = &profile.mixture_notch {
            tools.push(Box::new(cycle::SetMixture::new(mix.clone())));
        }
        if let Some(ts) = &profile.turboprop_start {
            tools.push(Box::new(cycle::StartTurbines::new(ts.clone())));
            tools.push(Box::new(cycle::StopTurbines::new(ts.clone())));
        }

        if profile.parking_brake {
            tools.push(Box::new(controls::SetParkingBrake {
                valve_set: profile.park_brake_valve_set,
            }));
        }

        if profile.pitch_trim {
            tools.push(Box::new(controls::SetPitchTrim));
        }

        if profile.speedbrake {
            tools.push(Box::new(controls::SetSpeedbrake));
        }

        if profile.pitot_heat {
            tools.push(Box::new(instruments::SetPitotHeat {
                override_dataref: profile.pitot_heat_dataref.clone(),
            }));
        }

        if !profile.electrical.is_empty() {
            tools.push(Box::new(electrical::SetElectrical::new(profile.electrical.clone())));
        }

        if profile.barometer {
            tools.push(Box::new(instruments::SetBarometer));
            tools.push(Box::new(instruments::SetAltimeterCurrent {
                transition_altitude_ft: profile.transition_altitude_ft,
            }));
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

        // Ramp / ground-ops — sim ground-crew actions (chocks, ground power,
        // pushback, services). Not airframe systems, so available on any aircraft.
        tools.push(Box::new(ground::SetChocks));
        tools.push(Box::new(ground::SetGroundPower));
        tools.push(Box::new(ground::Pushback));
        tools.push(Box::new(ground::RequestService));

        // Reminders ("wake me at top of descent", "tell me at the next waypoint") are
        // crew memory, not an airframe system, so available on any aircraft.
        tools.push(Box::new(reminder::SetReminder));

        // Checklists are NOT registered as one-shot tools — the copilot runs them
        // interactively (narrate + act + verify + call out the pilot's items),
        // which needs the voice, so the main loop intercepts the pick. They're
        // interpreted from the airframe's XChecklist file (see [`crate::checklist`]).

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

/// ON commands for every charging source in an electrical profile (keys
/// starting "alternator"/"generator"), deduped — the pair entries ("alternators")
/// repeat the singles' commands. Feeds `StartEngine`, so a dual-alternator
/// airframe (SR22 ALT 1 + ALT 2) gets both online after a start.
pub fn alternator_on_cmds(systems: &[crate::aircraft::ElectricalSystem]) -> Vec<String> {
    let mut cmds: Vec<String> = Vec::new();
    for s in systems {
        let k = s.key.to_ascii_lowercase();
        if k.starts_with("alternator") || k.starts_with("generator") {
            for c in &s.on_commands {
                if !cmds.contains(c) {
                    cmds.push(c.clone());
                }
            }
        }
    }
    cmds
}

/// A flattened, serializable view of a tool for the LLM layer to wrap.
pub struct ToolSpec {
    pub name: String,
    pub description: String,
    pub parameters: Value,
}
