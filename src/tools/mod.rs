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
pub mod circuit;
pub mod controls;
pub mod cycle;
pub mod electrical;
pub mod engine;
pub mod flaps;
pub mod full_auto;
pub mod gear;
pub mod ground;
pub mod instruments;
pub mod jet_start;
pub mod lights;
pub mod power;
pub mod procedure;
pub mod prop;
pub mod radio;
pub mod recover;
pub mod reminder;
pub mod reverse;
pub mod speedhold;
pub mod takeoff;
pub mod taxi;

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
/// The input-layer held-brake command, the pilot's own brake key. Held by the
/// taxi governor's stop channel AND by `hold_brakes` ("brakes on"). Command,
/// not ratio writes, on purpose: the sim re-drives the toe-brake ratios from
/// the pilot's hardware every frame, and REST-rate ratio writes lost that
/// fight (flaky "stop" since 2026-07-11, pinned 2026-07-13). A command can't
/// be stomped and can't stick.
pub(crate) const BRAKES_REGULAR: &str = "sim/flight_controls/brakes_regular";
/// How long each activation holds the brakes. Comfortably longer than a
/// renewal tick, so the hold is continuous while renewed, and any end of the
/// holder (release, crash, kill -9) frees the brakes within this long, by
/// expiry. A boot that can never be left on.
pub(crate) const BRAKE_HOLD_SECS: f64 = 1.0;
/// Command: engage the autopilot servos. Pressed by the AP-mode tools
/// (`set_heading`/`set_altitude`/…) and by the ILS setup in `approach`.
pub(crate) const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";

/// When [`BRAKES_REGULAR`] was last (re)issued, by ANY crew loop - the
/// `hold_brakes` holder or the taxi governor's stop channel. One clock for
/// both, because the taxi governor reads the PARKING-BRAKE ratio to detect
/// the pilot's brake toggle, and the held command rides that very dataref
/// while active (not the toe pedals - measured live, 2026-07-14: the sim
/// ramps parkbrake to regular effort ~0.36 and decays it ~0.8 s after
/// expiry). Until this clock is quiet, an elevated parkbrake can be our own
/// braking wearing the pilot's shoes.
static LAST_BRAKE_CMD: std::sync::Mutex<Option<std::time::Instant>> = std::sync::Mutex::new(None);

/// Record a [`BRAKES_REGULAR`] activation. Call alongside every issue of the
/// held command, or `brake_cmd_quiet` tells lies.
pub(crate) fn note_brake_cmd() {
    *LAST_BRAKE_CMD.lock().unwrap_or_else(|e| e.into_inner()) = Some(std::time::Instant::now());
}

/// The held-brake command, its expiry, and the parkbrake decay tail are all
/// behind us: the parking-brake ratio now reads as the PILOT's input. The
/// 1.0 s tail margin is the measured decay (~0.8 s to noise) plus slack -
/// thinner and the governor pauses itself on its own expiring stop command.
pub(crate) fn brake_cmd_quiet() -> bool {
    !controls::brakes_held()
        && LAST_BRAKE_CMD
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_none_or(|t| t.elapsed().as_secs_f64() > BRAKE_HOLD_SECS + 1.0)
}

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
            tools.push(Box::new(gear::Gear {
                vlo_kts: profile.vlo_kts,
            }));
        }

        if !profile.lights.is_empty() {
            tools.push(Box::new(lights::SetLights::new(profile.lights.clone())));
        }

        if profile.flood_lights {
            tools.push(Box::new(lights::SetFloodLights::new(
                profile.flood_lights_dataref.clone(),
            )));
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
                cfg: ap.clone(),
                approach_ias,
                prop: profile.prop.clone(),
            }));
            // Auto takeoff — the departure sibling: the roll steers through
            // the plugin's channel, the climb-out rides this same AP. Speeds
            // from the airframe's reference figures, conservative fallbacks
            // off stall speed when a profile carries none.
            let vr = profile
                .reference
                .as_ref()
                .and_then(|r| r.vr_kts)
                .unwrap_or((vs * 1.15).max(55.0));
            let vy = profile
                .reference
                .as_ref()
                .and_then(|r| r.vy_kts)
                .unwrap_or(vr + 15.0);
            let climb = profile
                .reference
                .as_ref()
                .and_then(|r| r.best_climb_fpm)
                .map_or(500.0, |c| (c * 0.8).clamp(400.0, 1500.0));
            let takeoff_roll_m = profile
                .reference
                .as_ref()
                .and_then(|r| r.takeoff_distance_ft)
                .map(|f| f * 0.3048);
            tools.push(Box::new(takeoff::AutoTakeoff {
                cfg: ap.clone(),
                vr_kts: vr,
                vy_kts: vy,
                climb_fpm: climb,
                retract_gear: profile.retractable_gear,
                takeoff_flap_ratio: profile.flaps.as_ref().and_then(|f| f.takeoff_ratio),
                takeoff_roll_m,
                prop: profile.prop.clone(),
            }));
            // Auto circuit — one command that flies takeoff + pattern + landing
            // by orchestrating the takeoff loop, the AP, and autoland. Shares
            // the takeoff speeds and the autoland approach speed above.
            tools.push(Box::new(circuit::Circuit {
                cfg: ap.clone(),
                vr_kts: vr,
                vy_kts: vy,
                climb_fpm: climb,
                retract_gear: profile.retractable_gear,
                takeoff_flap_ratio: profile.flaps.as_ref().and_then(|f| f.takeoff_ratio),
                takeoff_roll_m,
                approach_ias,
                prop: profile.prop.clone(),
            }));
            // Fly a REAL published SID / instrument approach from the sim's CIFP —
            // the terrain-safe path where the circuit only guessed. Shares the AP
            // and the autoland approach speed; hands the final to autoland.
            tools.push(Box::new(procedure::FlyProcedure {
                cfg: ap.clone(),
                approach_ias,
                vy_kts: vy,
            }));
            // Full auto — the whole flight from cold: power up, start engine,
            // request taxi, drive to the runway, line up, and fly the circuit.
            // Only where the airframe can do all of it (an autopilot airframe
            // with an engine to start).
            if profile.engine || profile.throttle {
                tools.push(Box::new(full_auto::FullAuto {
                    fuel_selector: profile.fuel_selector.clone(),
                    alternator_on_cmds: alternator_on_cmds(&profile.electrical),
                    bus_power_on_cmds: bus_power_on_cmds(&profile.electrical),
                    park_brake_valve_set: profile.park_brake_valve_set,
                    takeoff_roll_m,
                    takeoff_flap_ratio: profile.flaps.as_ref().and_then(|f| f.takeoff_ratio),
                    prop: profile.prop.clone(),
                    turboprop_start: profile.turboprop_start.clone(),
                    all_electrical_on_cmds: all_electrical_on_cmds(&profile.electrical),
                    cfg: ap,
                    vr_kts: vr,
                    vy_kts: vy,
                    climb_fpm: climb,
                    retract_gear: profile.retractable_gear,
                    approach_ias,
                    // The APU/electrical bring-up is the MD-82 sequence (APU
                    // start, cross-ties, bleed) — it belongs ONLY to a jet with
                    // an APU (`turbine_start`). A turboprop has no APU: running
                    // it on the Twin Otter narrated "starting the APU / the APU
                    // didn't come up" and waited 75 s on an APU that will never
                    // spin (2026-07-26). Turboprops power their bus at the start
                    // instead (StartEngine's profile battery/master commands).
                    power_up_first: profile.turbine_start,
                }));
            }
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
            tools.push(Box::new(speedhold::HoldSpeed {
                approach_kts: approach,
                park_brake_valve_set: profile.park_brake_valve_set,
            }));
            // Auto taxi stands on the same governor, so it ships with it —
            // and the line-up (hold-short onto the centreline) with both.
            tools.push(Box::new(taxi::AutoTaxi {
                park_brake_valve_set: profile.park_brake_valve_set,
                prop: profile.prop.clone(),
            }));
            tools.push(Box::new(taxi::LineUp {
                park_brake_valve_set: profile.park_brake_valve_set,
                takeoff_roll_m: profile
                    .reference
                    .as_ref()
                    .and_then(|r| r.takeoff_distance_ft)
                    .map(|f| f * 0.3048),
                prop: profile.prop.clone(),
            }));
            // Taxi to PARKING after landing — requests the taxi-in from ATC and
            // drives the cleared stand route.
            tools.push(Box::new(taxi::TaxiToParking {
                park_brake_valve_set: profile.park_brake_valve_set,
                prop: profile.prop.clone(),
            }));
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
                bus_power_on_cmds: bus_power_on_cmds(&profile.electrical),
            }));
            tools.push(Box::new(engine::StopEngine));
        }

        // Reverse thrust (rollout only, self-stowing) — turboprops/jets whose
        // profile says the model has a usable beta/reverse range.
        if profile.reverse_thrust {
            tools.push(Box::new(reverse::ReverseThrust {
                engines: profile.engines,
            }));
        }

        if profile.fuel_pump || profile.fuel_pump_commands.is_some() {
            tools.push(Box::new(engine::SetFuelPump {
                engines: profile.engines,
                commands: profile.fuel_pump_commands.clone(),
            }));
        }

        if profile.magnetos {
            tools.push(Box::new(engine::SetMagnetos {
                engines: profile.engines,
                per_engine: profile.magnetos_per_engine,
            }));
        }

        if profile.carb_heat {
            tools.push(Box::new(engine::SetCarbHeat {
                engines: profile.engines,
            }));
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
            tools.push(Box::new(cycle::SetCycleFuelSelector::new(
                profile.fuel_selectors.clone(),
            )));
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

        // Regular (crew-held) wheel brakes, "brakes on" / "brakes off". Every
        // airframe has wheel brakes, so not profile-gated the way the latch
        // above is.
        tools.push(Box::new(controls::HoldBrakes));

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
            tools.push(Box::new(electrical::SetElectrical::new(
                profile.electrical.clone(),
            )));
        }

        if profile.barometer {
            tools.push(Box::new(instruments::SetBarometer));
            tools.push(Box::new(instruments::SetAltimeterCurrent {
                transition_altitude_ft: profile.transition_altitude_ft,
            }));
        }

        // Always registered, NOT gated on `profile.directional_gyro`: X-Plane
        // keeps a syncable (and drifting!) DG under airframes the profile
        // calls glass — the SR22 flew 166° off its indicated heading while
        // "auto sync the gyro" earned a miss beep (2026-07-12). Syncing a
        // zero-drift gyro is a harmless no-op with an honest readback;
        // refusing the ask while the pilot's heading reference is wrong is
        // the worst outcome. The profile flag still gates the vacuum reads
        // and the drift nag, where a glass panel genuinely has nothing.
        tools.push(Box::new(instruments::AlignGyro));

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

/// ON commands that bring the DC bus alive for an engine start — the battery and
/// (where the airframe has one) the DC master, in profile order. On airframes
/// whose stock `battery_on` dataref is frame-synced FROM a custom switch (the
/// Twin Otter: master.lua overwrites it every frame), a stock write reads back
/// 0 V and the starter never sees voltage — firing these profile commands is the
/// only way the bus comes up. Empty on airframes with no custom electrical (the
/// pistons), where the stock write `StartEngine` falls back to works.
pub fn bus_power_on_cmds(systems: &[crate::aircraft::ElectricalSystem]) -> Vec<String> {
    let mut cmds: Vec<String> = Vec::new();
    for s in systems {
        let k = s.key.to_ascii_lowercase();
        if k.starts_with("batt") || k.starts_with("master") || k.starts_with("dc_master") {
            for c in &s.on_commands {
                if !cmds.contains(c) {
                    cmds.push(c.clone());
                }
            }
        }
    }
    cmds
}

/// EVERY electrical system's ON command, in profile order, deduped — the full
/// bring-up full auto runs AFTER the engine catches so the aircraft is properly
/// powered for the flight, not just cranked. On the Twin Otter this is what
/// finally switches the generators AND the BLEED AIR on — and the bleeds feed
/// the autopilot and anti-ice, so a circuit flown without them is a circuit
/// with no servos. Empty on airframes with no electrical profile (the 172,
/// which powers its own bus at start). Battery/master re-fire harmlessly.
pub fn all_electrical_on_cmds(systems: &[crate::aircraft::ElectricalSystem]) -> Vec<String> {
    let mut cmds: Vec<String> = Vec::new();
    for s in systems {
        for c in &s.on_commands {
            if !cmds.contains(c) {
                cmds.push(c.clone());
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

#[cfg(test)]
mod power_cmd_tests {
    use super::*;
    use crate::aircraft::ElectricalSystem;

    fn sys(key: &str, on: &[&str]) -> ElectricalSystem {
        ElectricalSystem {
            key: key.into(),
            spoken: key.into(),
            on_commands: on.iter().map(|s| s.to_string()).collect(),
            off_commands: vec![],
            readback: vec![],
        }
    }

    #[test]
    fn bus_power_takes_battery_and_master_not_generators() {
        // The Twin Otter's electrical: battery + DC master bring the bus alive;
        // the generators come on AFTER the start (that's alternator_on_cmds).
        let elec = vec![
            sys("battery", &["Twin_Otter/battery/power_to_battery"]),
            sys("master", &["Twin_Otter/dc_master/master_on"]),
            sys("generator1", &["Twin_Otter/generators/left_on"]),
            sys(
                "generators",
                &[
                    "Twin_Otter/generators/left_on",
                    "Twin_Otter/generators/right_on",
                ],
            ),
            sys("bleed", &["Twin_Otter/bleed_air/left_on"]),
        ];
        assert_eq!(
            bus_power_on_cmds(&elec),
            vec![
                "Twin_Otter/battery/power_to_battery".to_string(),
                "Twin_Otter/dc_master/master_on".to_string(),
            ]
        );
        // And the generators are what alternator_on_cmds picks up, deduped.
        assert_eq!(
            alternator_on_cmds(&elec),
            vec![
                "Twin_Otter/generators/left_on".to_string(),
                "Twin_Otter/generators/right_on".to_string(),
            ]
        );
    }

    #[test]
    fn bus_power_is_empty_for_a_piston_with_no_custom_electrical() {
        // Nothing to fire → StartEngine keeps its stock battery_on write.
        assert!(bus_power_on_cmds(&[]).is_empty());
    }
}
