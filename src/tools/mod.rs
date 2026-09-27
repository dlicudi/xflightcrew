//! The curated tool surface.
//!
//! Core design rule (from the brief): the LLM never writes arbitrary datarefs.
//! It only picks one of these explicit tools. Each tool owns its dataref(s),
//! valid range, safety gates, and — crucially — a *readback* that re-reads the
//! dataref after writing and confirms the actual value. A copilot confirms the
//! action, not the intent.
//!
//! The [`Tool`] schema (`name` + `description` + JSON-schema `parameters`) is
//! what the deterministic grammar and the dialogue stack target: a
//! [`ToolCall`] names a tool and carries its arguments. No model picks tools;
//! the LLM that once did was retired on 2026-09-13 (see docs/dev-history.md).

pub mod approach;
pub mod autoland;
pub mod autopilot;
pub mod cabin_air;
pub mod circuit;
pub mod controls;
pub mod cycle;
pub mod electrical;
pub mod engine;
pub mod flaps;
pub mod fuel_pumps;
pub mod full_auto;
pub mod gear;
pub mod ground;
pub mod heli;
pub mod instruments;
pub mod jet_start;
pub mod lights;
pub mod loading;
pub mod power;
pub mod procedure;
pub mod prop;
pub mod radio;
pub mod recover;
pub mod reminder;
pub mod reverse;
pub mod setup;
pub mod signs;
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
    /// What the tool does, in the words a pilot would use to ask for it. No
    /// model reads this any more; it stays as each tool's own statement of
    /// intent beside its grammar rules, and tests may quote it.
    #[allow(dead_code)]
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

/// The tools a rotorcraft profile never gets: every loop whose law assumes a
/// wing, a runway roll or a throttle-for-speed (see
/// [`crate::aircraft::AircraftProfile::rotorcraft`]).
pub const ROTORCRAFT_WITHHELD: &[&str] = &[
    "autoland",
    "auto_takeoff",
    "circuit",
    "fly_procedure",
    "full_auto",
    "recover",
    "visual_approach",
    "prepare_ils",
    "hold_speed",
    "auto_taxi",
    "line_up",
    "pushback",
    "reverse_thrust",
    // A new flight on a runway final is landed by autoland.
    "setup_approach",
];

impl Registry {
    /// Build the tool set for an aircraft. The profile shapes the surface: the
    /// 172 has no retractable gear, so no `gear` tool is added for it. Each
    /// system present on the airframe contributes its tool(s).
    pub fn for_aircraft(profile: &AircraftProfile) -> Self {
        Self::build(profile)
    }

    fn build(profile: &AircraftProfile) -> Self {
        let mut tools: Vec<Box<dyn Tool>> = Vec::new();
        // How briskly this airframe taxis. Resolved once here so every ground
        // drive — the spoken auto taxi, the line-up, the arrival, and the ones
        // full auto builds for itself — is paced by the aircraft actually
        // loaded. A jet is not a 172: see `aircraft::TaxiProfile`.
        let pace = crate::autotaxi::Pace::from_profile(profile.taxi.as_ref());
        // The runway the wheels actually need, meters. Prefer the declared
        // GROUND ROLL; the recited takeoff distance is the fallback the GA
        // profiles have always used, but on an airliner that figure is a
        // factored field length, and the ×1.4 every consumer adds then lands
        // past the end of real runways (see `ReferenceProfile::takeoff_roll_ft`).
        // One computation, because the line-up and the takeoff must agree on
        // what "enough runway" means.
        // ...and the weight that roll is quoted AT, so it can be scaled to
        // the aircraft's actual weight rather than assumed at max.
        let (takeoff_roll_m, mtow_kg) = crate::aircraft::profile_takeoff_roll(profile);

        if let Some(flaps) = &profile.flaps {
            tools.push(Box::new(flaps::SetFlaps::new(flaps.clone())));
        }

        if profile.retractable_gear {
            tools.push(Box::new(gear::Gear {
                vlo_kts: profile.vlo_kts,
                by_command: profile.gear_by_command,
            }));
        }

        if !profile.lights.is_empty() {
            tools.push(Box::new(lights::SetLights::new(profile.lights.clone())));
        }

        if profile.flood_lights {
            tools.push(Box::new(lights::SetFloodLights::new(
                profile.flood_lights_dataref.clone(),
                profile.flood_lights_toggle.clone(),
            )));
        }

        // What autoland flies with, for the new flight on a final
        // (`setup_approach`): set with the autoland below, `None` without an
        // autopilot (the new flight is then the pilot's).
        let mut approach_land: Option<(autopilot::ApConfig, f64)> = None;
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
                flare_power: profile.flare_power,
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
            approach_land = Some((ap.clone(), approach_ias));
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
            // The TAKEOFF's climb reference is the INITIAL climb speed, which
            // is `vy` on everything that does not say otherwise (see
            // `ReferenceProfile::initial_climb_kts`). The circuit and the
            // follower keep `vy` — those legs really are flown at it.
            let initial_climb = profile
                .reference
                .as_ref()
                .and_then(|r| r.initial_climb_kts)
                .unwrap_or(vy);
            let climb = profile
                .reference
                .as_ref()
                .and_then(|r| r.best_climb_fpm)
                .map_or(500.0, |c| (c * 0.8).clamp(400.0, 1500.0));
            tools.push(Box::new(takeoff::AutoTakeoff {
                cfg: ap.clone(),
                vr_kts: vr,
                vy_kts: vy,
                initial_climb_kts: initial_climb,
                climb_fpm: climb,
                retract_gear: profile.retractable_gear,
                takeoff_flap_ratio: profile.flaps.as_ref().and_then(|f| f.takeoff_ratio),
                flap_schedule: crate::aircraft::flap_schedule(profile.flaps.as_ref()),
                takeoff_roll_m,
                mtow_kg,
                cruise_ias_kts: profile.reference.as_ref().and_then(|r| r.cruise_ias_kts),
                prop: profile.prop.clone(),
                tail_strike_deg: profile.tail_strike_deg,
                pitch_authority_down: profile.pitch_authority_down,
                vne_kts: profile.vne_kts,
                handoff_to_caller: false,
                park_brake_valve_set: profile.park_brake_valve_set,
            }));
            // Auto circuit — one command that flies takeoff + pattern + landing
            // by orchestrating the takeoff loop, the AP, and autoland. Shares
            // the takeoff speeds and the autoland approach speed above.
            tools.push(Box::new(circuit::Circuit {
                tail_strike_deg: profile.tail_strike_deg,
                pitch_authority_down: profile.pitch_authority_down,
                vne_kts: profile.vne_kts,
                cfg: ap.clone(),
                vr_kts: vr,
                vy_kts: vy,
                initial_climb_kts: initial_climb,
                climb_fpm: climb,
                retract_gear: profile.retractable_gear,
                takeoff_flap_ratio: profile.flaps.as_ref().and_then(|f| f.takeoff_ratio),
                flap_schedule: crate::aircraft::flap_schedule(profile.flaps.as_ref()),
                takeoff_roll_m,
                mtow_kg,
                approach_ias,
                prop: profile.prop.clone(),
                climb_floor_kts: circuit::climb_floor(profile),
                vs_clean_kts: profile.vs_clean_kts,
                park_brake_valve_set: profile.park_brake_valve_set,
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
                    engines: profile.engines,
                    cycle_selectors: profile.fuel_selectors.clone(),
                    fuel_selector: profile.fuel_selector.clone(),
                    alternator_on_cmds: alternator_on_cmds(&profile.electrical),
                    bus_power_on_cmds: bus_power_on_cmds(&profile.electrical),
                    park_brake_valve_set: profile.park_brake_valve_set,
                    takeoff_roll_m,
                    mtow_kg,
                    landing_distance_ft: profile
                        .reference
                        .as_ref()
                        .and_then(|r| r.landing_distance_ft),
                    takeoff_distance_ft: profile
                        .reference
                        .as_ref()
                        .and_then(|r| r.takeoff_distance_ft),
                    max_gross_lb: profile.reference.as_ref().and_then(|r| r.max_gross_lb),
                    load_sheet: profile.load_sheet,
                    airframe: profile.display_name.clone(),
                    takeoff_flap_ratio: profile.flaps.as_ref().and_then(|f| f.takeoff_ratio),
                    flap_schedule: crate::aircraft::flap_schedule(profile.flaps.as_ref()),
                    cruise_ias_kts: profile.reference.as_ref().and_then(|r| r.cruise_ias_kts),
                    transition_altitude_ft: profile.transition_altitude_ft,
                    prop: profile.prop.clone(),
                    pace,
                    tail_strike_deg: profile.tail_strike_deg,
                    pitch_authority_down: profile.pitch_authority_down,
                    vne_kts: profile.vne_kts,
                    climb_floor_kts: circuit::climb_floor(profile),
                    vs_clean_kts: profile.vs_clean_kts,
                    lights: profile.lights.clone(),
                    flood_dataref: profile.flood_lights_dataref.clone(),
                    flood_toggle: profile.flood_lights_toggle.clone(),
                    // "Can we start it at all": the generic piston group OR a
                    // custom cycle-switch piston sequence (the DC-3, whose
                    // mixture is xlua-overridden so it declares no `engine`).
                    piston_engine: profile.engine || profile.piston_start.is_some(),
                    piston_mixture: profile.engine,
                    turboprop_start: profile.turboprop_start.clone(),
                    airliner_start: profile.airliner_start.clone(),
                    all_electrical_on_cmds: all_electrical_on_cmds(&profile.electrical),
                    all_electrical_toggles: all_electrical_toggles(&profile.electrical),
                    cfg: ap,
                    vr_kts: vr,
                    vy_kts: vy,
                    initial_climb_kts: initial_climb,
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
                    jet_start: profile.turbine_start,
                    scripted_auto_start: profile.scripted_auto_start,
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
        // The AIRLINER start: an ENG MODE knob and guarded master switches.
        // Declared first because it is the specific one — an airframe with
        // this block never wants the MD-82's bleed/crank sequence as well,
        // and two tools of the same name is a registry that answers twice.
        if let Some(a) = profile.airliner_start.clone() {
            tools.push(Box::new(jet_start::AirlinerStart::new(a, profile.engines)));
            tools.push(Box::new(jet_start::StopEngines::new(profile)));
        }
        // Turbine start sequence (jets) — distinct from the piston one-shot start.
        if profile.turbine_start && profile.airliner_start.is_none() {
            tools.push(Box::new(jet_start::StartEngines));
            tools.push(Box::new(jet_start::StopEngines::new(profile)));
        }
        // An aircraft that scripts its own start: "start the engines" fires
        // that script (the stock 737-800, `scripted_auto_start`). The SHUTDOWN
        // is ours either way — the aircraft scripts the start, not the stop —
        // and leaving it out gave the 737 no way to be shut down at all: at
        // the Heathrow stand the pilot asked seven ways over forty-five
        // seconds ("let's shut down", "engines off", "turn engines off",
        // "shut down aircraft") and every one of them parsed to a tool the
        // registry had not built, until they gave up with "my aircraft"
        // (2026-09-17).
        if profile.scripted_auto_start && !profile.turbine_start && profile.airliner_start.is_none()
        {
            tools.push(Box::new(jet_start::ScriptedAutoStart));
            tools.push(Box::new(jet_start::StopEngines::new(profile)));
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
        }

        // The GROUND drives, for anything with a throttle — autothrottle or
        // not. These were gated behind `!autothrottle` along with `hold_speed`,
        // which is right for the AIRBORNE speed tool (an airliner sets its
        // speed bug) and wrong for these: on the ground the taxi governor is
        // the only speed control there is, and auto taxi arms it directly
        // rather than through the registry. The effect was that neither jet
        // had a taxi, line-up or park tool at all — "taxi for me" reached
        // nothing on the MD-82 or the Citation, and only full auto worked
        // because it builds its own copies (found 2026-07-28).
        if profile.engine || profile.throttle {
            tools.push(Box::new(taxi::AutoTaxi {
                park_brake_valve_set: profile.park_brake_valve_set,
                prop: profile.prop.clone(),
                pace,
                must_start: false,
            }));
            tools.push(Box::new(taxi::LineUp {
                park_brake_valve_set: profile.park_brake_valve_set,
                takeoff_roll_m,
                mtow_kg,
                prop: profile.prop.clone(),
                pace,
            }));
            // Taxi to PARKING after landing — requests the taxi-in from ATC and
            // drives the cleared stand route.
            tools.push(Box::new(taxi::TaxiToParking {
                park_brake_valve_set: profile.park_brake_valve_set,
                prop: profile.prop.clone(),
                pace,
            }));
        } else if profile.rotorcraft && profile.heli_start.is_some() {
            // A helicopter taxis in like any other aircraft: the same tool,
            // the same call to the tower and the same clearance; only the
            // drive differs (`taxi_in` hover-taxis the route).
            tools.push(Box::new(taxi::TaxiToParking {
                park_brake_valve_set: profile.park_brake_valve_set,
                prop: profile.prop.clone(),
                pace,
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
                engines: profile.engines,
                cycle_selectors: profile.fuel_selectors.clone(),
                fuel_selector: profile.fuel_selector.clone(),
                alternator_on_cmds: alternator_on_cmds(&profile.electrical),
                bus_power_on_cmds: bus_power_on_cmds(&profile.electrical),
            }));
            tools.push(Box::new(engine::StopEngine {
                engines: profile.engines,
                park_brake_valve_set: profile.park_brake_valve_set,
            }));
        }

        // Reverse thrust (rollout only, self-stowing) — turboprops/jets whose
        // profile says the model has a usable beta/reverse range.
        if profile.reverse_thrust {
            tools.push(Box::new(reverse::ReverseThrust {
                engines: profile.engines,
            }));
        }

        // The centre tank pumps: registered on every airframe so "centre
        // pumps off" on one without them is answered, not a silent miss.
        tools.push(Box::new(fuel_pumps::SetCentrePumps {
            declared: profile.fuel_tank_pumps_off_when_empty.clone(),
        }));

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
        if let Some(h) = &profile.heli_start {
            let battery_on = profile
                .electrical
                .iter()
                .filter(|e| e.key == "battery")
                .flat_map(|e| e.on_commands.clone())
                .collect();
            let start = heli::StartHelicopter::new(h.clone(), profile.engines, battery_on);
            tools.push(Box::new(start.clone()));
            tools.push(Box::new(heli::StopHelicopter::new(
                h.clone(),
                profile.engines,
            )));
            tools.push(Box::new(heli::Hover::new(
                h,
                start,
                crate::heli_route::TaxiIn {
                    park_brake_valve_set: profile.park_brake_valve_set,
                    prop: profile.prop.clone(),
                    pace: crate::autotaxi::Pace::from_profile(profile.taxi.as_ref()),
                },
            )));
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

        // The load sheet is the plan's, not an airframe system: every
        // aircraft has tanks and a payload.
        tools.push(Box::new(loading::LoadAircraft));
        tools.push(Box::new(setup::SetUpFlight));
        tools.push(Box::new(setup::SetUpApproach { land: approach_land }));

        // Reminders ("wake me at top of descent", "tell me at the next waypoint") are
        // crew memory, not an airframe system, so available on any aircraft.
        tools.push(Box::new(reminder::SetReminder));

        // A helicopter keeps its switches, radios and autopilot dials and
        // loses every fixed-wing law, whatever the other flags said: the
        // laws would rotate, flare and taxi a rotorcraft like a 172.
        if profile.rotorcraft {
            tools.retain(|t| !ROTORCRAFT_WITHHELD.contains(&t.name()));
        }

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
/// Every toggle-only switch across the airframe's electrical systems.
///
/// A bulk power-up that fires `on_commands` skips these entirely, because a
/// toggle-driven system HAS no on command — and skipping is silent. The
/// Citation X's avionics master is toggle-only, so full auto powered up,
/// announced "generators and bleeds on", and left the radios dead; the copilot
/// then correctly reported "we're not transmitting, check the avionics master"
/// about a switch it had just declined to touch (2026-07-28). Fired only where
/// the readback says the system is actually off.
pub fn all_electrical_toggles(
    systems: &[crate::aircraft::ElectricalSystem],
) -> Vec<crate::aircraft::ToggleSwitch> {
    systems.iter().flat_map(|s| s.toggles.clone()).collect()
}

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

/// A tool picked for an utterance (by the grammar, the dialogue stack or a
/// checklist title), plus its parsed arguments.
#[derive(Debug)]
pub struct ToolCall {
    pub name: String,
    pub arguments: Value,
}

#[cfg(test)]
mod departure_lights_tests {
    /// Full auto flew DARK — no beacon, no nav, no taxi light, no strobes, no
    /// landing lights — while the arrival explicitly switched strobes, taxi
    /// and landing off at the stand. Turning off what was never turned on is
    /// the proof it was an oversight (2026-08-01). The schedule is only as
    /// good as the profile keys it names, so check every shipped profile
    /// actually declares them.
    #[test]
    fn every_profile_declares_the_lights_the_departure_switches() {
        let dir = std::path::Path::new("profiles");
        let mut checked = 0;
        for entry in std::fs::read_dir(dir).expect("profiles dir").flatten() {
            let path = entry.path();
            if path.extension().is_none_or(|e| e != "yaml") {
                continue;
            }
            let id = path.file_stem().unwrap().to_string_lossy().to_string();
            let Some(p) = crate::aircraft::load(&id) else {
                continue;
            };
            if p.lights.is_empty() {
                continue; // an airframe with no light layer is not a failure
            }
            checked += 1;
            let keys: Vec<&str> = p.lights.iter().map(|l| l.key.as_str()).collect();
            // Nav, and at least one anti-collision light. NOT beacon
            // specifically: the Lancair Evolution's switch row really is
            // NAV / LAND / STRB with no separate beacon, and inventing a
            // switch an airframe hasn't got is the trap the profile comments
            // exist to prevent. The departure schedule no-ops any key a
            // profile doesn't declare, so this only catches a profile that
            // would depart with nothing lit at all.
            // Nav, plus at least one light the TAKEOFF half of the schedule
            // can switch. Deliberately not "beacon" specifically: the Lancair
            // Evolution's switch row really is NAV / LAND / STRB with no
            // separate beacon, and inventing a switch an airframe hasn't got
            // is the trap the profile comments exist to prevent. The schedule
            // no-ops any key a profile omits, so this catches only a profile
            // that would depart with nothing lit at all.
            assert!(keys.contains(&"nav"), "{id}: no nav lights in {keys:?}");
            assert!(
                keys.contains(&"strobe") || keys.contains(&"landing"),
                "{id}: nothing for the takeoff to light in {keys:?}"
            );
        }
        assert!(checked > 0, "no profiles with lights were checked");
    }

    /// The S-76C: a helicopter gets its switches, radios, gear and the
    /// autopilot dials, and none of the fixed-wing laws.
    #[test]
    fn a_rotorcraft_keeps_the_switches_and_loses_every_fixed_wing_law() {
        let p = crate::aircraft::load("sikorsky_s76c").expect("S-76C profile loads");
        assert!(p.rotorcraft);
        let reg = super::Registry::for_aircraft(&p);
        let names: Vec<&str> = reg.tools.iter().map(|t| t.name()).collect();
        for withheld in super::ROTORCRAFT_WITHHELD {
            assert!(
                !names.contains(withheld),
                "{withheld} registered on a helicopter"
            );
        }
        for kept in [
            "gear",
            "set_heading",
            "set_altitude",
            "set_com1",
            "set_transponder",
            "set_parking_brake",
        ] {
            assert!(
                names.contains(&kept),
                "{kept} missing on the S-76C from {names:?}"
            );
        }
        // The gate is the flag, not the other flags: a fixed-wing profile
        // with the same switches keeps its laws.
        let mut fixed = p.clone();
        fixed.rotorcraft = false;
        let fixed_reg = super::Registry::for_aircraft(&fixed);
        let names: Vec<&str> = fixed_reg.tools.iter().map(|t| t.name()).collect();
        assert!(names.contains(&"autoland") && names.contains(&"auto_takeoff"));
    }
}

#[cfg(test)]
mod power_cmd_tests {
    use super::*;
    use crate::aircraft::ElectricalSystem;

    fn sys(key: &str, on: &[&str]) -> ElectricalSystem {
        ElectricalSystem {
            write_datarefs: vec![],
            toggles: vec![],
            guards: vec![],
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
