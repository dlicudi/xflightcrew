//! Aircraft profiles.
//!
//! A profile is the per-airframe data that shapes the copilot: which systems
//! exist (and therefore which tools are exposed) and their limits.
//!
//! Profiles load from `profiles/<id>.yaml` at startup ([`load`]), falling back
//! to a built-in Rust definition ([`builtin`]) if the file is missing — so the
//! app works out of the box, and new airframes can be added as YAML without
//! recompiling. Datarefs/commands stay in the Rust tools (they're standard
//! X-Plane); the YAML carries the per-aircraft *config* (flap detents, lights,
//! capability flags).

use serde::{Deserialize, Serialize};

/// Everything the tool layer needs to know about one airframe.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AircraftProfile {
    pub id: String,
    pub display_name: String,
    /// Other ICAO types a SimBrief plan for THIS airframe may be filed as,
    /// when SimBrief has no base type of its own for it (the Lancair
    /// Evolution, acf_ICAO EVOT, is planned as a custom airframe on the
    /// Piper M500, P46T). The plan audit's type check accepts them.
    #[serde(default)]
    pub simbrief_types: Vec<String>,
    /// Which loaded aircraft this profile is FOR: its `acf_ICAO` and the
    /// model (`acf_relative_path`, optionally author and description). The
    /// stock Laminar 737 and the Zibo both say B738, and one ICAO cannot
    /// pick between them (2026-09-27). `None` is a profile detection never
    /// picks (a pinned `XCOPILOT_AIRCRAFT` still can).
    #[serde(default, rename = "match")]
    pub matcher: Option<ProfileMatch>,
    /// Flap system, if any. `None` means no `set_flaps` tool.
    #[serde(default)]
    pub flaps: Option<FlapProfile>,
    /// Fuel endurance monitoring thresholds. `None` means no fuel callouts.
    #[serde(default)]
    pub fuel: Option<FuelProfile>,
    /// Clean (flaps-up) stall speed, KIAS — drives the monitor's airspeed
    /// callouts. `None` means no stall caution.
    #[serde(default)]
    pub vs_clean_kts: Option<f64>,
    /// Never-exceed speed (Vne), KIAS — drives the monitor's overspeed callout.
    /// `None` means no overspeed caution.
    #[serde(default)]
    pub vne_kts: Option<f64>,
    /// A HIGHER Vmo above an altitude (`vmo_high: { above_ft: 8000, kts:
    /// 350 }`), for an airframe whose limit rises with height: `vne_kts` is
    /// then the LOW limit, the one below `above_ft` and the one a caller
    /// without an altitude gets. The Citation X is 270 KIAS below 8,000 ft
    /// and 350 above; one number capped its cruise at 233 and would have
    /// nagged "overspeed" over 243 at FL350. `None` = one Vne at every
    /// altitude ([`vne_at`]).
    #[serde(default)]
    pub vmo_high: Option<VmoHigh>,
    /// Landing-gear operating/extended limit speed (VLO/VLE), KIAS — the `gear`
    /// tool refuses cycling the gear above it. `None` = no gear speed gate.
    #[serde(default)]
    pub vlo_kts: Option<f64>,
    /// The airframe's own landing-gear warning envelope (the horn and the
    /// warning annunciator), from its `.acf` at detection when the profile
    /// is silent. The takeoff raises the gear only once doing so will not
    /// sound it: the SF50's fires below 125 kt with the takeoff flap set,
    /// and the gear came up at 200 ft and 122 kt (ETND, 2026-09-23).
    #[serde(default)]
    pub gear_warning: Option<GearWarning>,
    /// Nominal DC bus voltage (12 or 24). The engine-start "battery's low"
    /// gate scales with it: a healthy 12 volt RV-10 read "only 12 volts"
    /// against the 172's 24 volt threshold (2026-09-11). `None` = 24.
    #[serde(default)]
    pub bus_volts_nominal: Option<f64>,
    /// The least thrust (lever ratio) the autoland holds on a stabilised
    /// final, when not more than a few knots fast. Jets fly the approach on
    /// thrust; `None` lets the speed law idle the engines (the pistons).
    #[serde(default)]
    pub approach_thrust_min: Option<f64>,
    /// The rudder's authority on the takeoff roll, when measured.
    #[serde(default)]
    pub roll_rudder: Option<RollRudderProfile>,
    /// The rotation's elevator on the wheels, when fitted.
    #[serde(default)]
    pub rotate_elevator: Option<RotateElevatorProfile>,
    /// The yaw plant of a free castoring nose (`ground_steer`), when fitted.
    #[serde(default)]
    pub ground_plant: Option<GroundPlantProfile>,
    /// The rollout's steer on a steered nosewheel, when fitted.
    #[serde(default)]
    pub rollout_steer: Option<RolloutSteerProfile>,
    /// The speed governor's lead on this airframe's engines, when fitted.
    #[serde(default)]
    pub speed_governor: Option<SpeedGovernorProfile>,
    /// Bleed air and crew oxygen, where the airframe has them.
    #[serde(default)]
    pub cabin_air: Option<CabinAirProfile>,
    /// None = unknown; do not invent a pressurisation system in warnings.
    #[serde(default)]
    pub pressurised: Option<bool>,
    /// Static reference figures (V-speeds, weights, limits) the copilot recites
    /// when asked. Per-airframe, from the POH/POM. `None` = no reference answers.
    #[serde(default)]
    pub reference: Option<ReferenceProfile>,
    /// How briskly the copilot taxis this airframe. `None` = the built-in GA
    /// pace (see [`TaxiProfile`]).
    #[serde(default)]
    pub taxi: Option<TaxiProfile>,
    /// The input the plugin's yoke channels (`xcopilot/pitch`, `xcopilot/roll`)
    /// drive, when it is not the stock yoke ratios (`crate::yoke`). `None` =
    /// the stock path.
    #[serde(default)]
    pub yoke_input: Option<YokeInput>,
    /// The thrust-lever dataref this airframe obeys, when it is not the stock
    /// one. `None` = `throttle_ratio_all` (see [`throttle_dataref`]).
    #[serde(default)]
    pub throttle_dataref: Option<String>,
    /// The pilot's HARDWARE thrust lever as this airframe reads it, when it
    /// can snap the levers to it on its own (the Zibo's `axis/throttle1`,
    /// with its throttle noise lock OFF). A lever read below our command is
    /// then a pilot's hand only when this axis moved too
    /// ([`crate::autotakeoff::pilot_moved_throttle`]). `None` = the lever
    /// read alone.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub throttle_hardware_axis: Option<String>,
    /// The takeoff stabilizer trim set from the type's table, as a crew sets
    /// it (`crate::takeoff_trim`). `None` = the generic neutral trim.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub takeoff_trim: Option<TakeoffTrimProfile>,
    /// The takeoff and climb thrust flown to the type's N1 tables, the
    /// rating chosen for the weight (`crate::takeoff_thrust`). `None` = the
    /// levers to full, reduced to `takeoff_config.climb_thrust_ratio`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub takeoff_thrust: Option<TakeoffThrustProfile>,
    /// Reverse selected by HOLDING this command, where the stock reverse
    /// lever write is undone (`tools::reverse`). `None` = the lever write.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reverse_hold_command: Option<String>,
    /// The airframe's own reverse levers, written to 0 to STOW: on the Zibo
    /// a hold released with the lever left at reverse idle (0.06) kept the
    /// reversers deployed until the levers were written down (probed
    /// parked, 2026-10-07). Empty = the release alone.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub reverse_stow_levers: Vec<String>,
    /// The type's speeds BY WEIGHT, from its FCOM tables: Vref per landing
    /// flap and V1 / Vr / V2 for the takeoff flap ([`WeightSpeedsProfile`]).
    /// Declared, the approach and the takeoff fly the speeds for what the
    /// aircraft weighs NOW, as a crew reads them off the card; `None` = the
    /// fixed `reference:` figures.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub weight_speeds: Option<WeightSpeedsProfile>,
    /// Pitch attitude at which this airframe's tail touches the runway,
    /// degrees. The auto-takeoff rotation will not command within a margin of
    /// it. `None` on airframes short enough that elevator authority runs out
    /// long before tail clearance does — every GA single so far.
    #[serde(default)]
    pub tail_strike_deg: Option<f64>,
    /// Nose-DOWN elevator the rotation law may command, 0..1. `None` = the GA
    /// tenth, which is right for a light single and is not authority at all
    /// on a jet — see `autotakeoff::PITCH_MIN`.
    #[serde(default)]
    pub pitch_authority_down: Option<f64>,
    /// The highest pitch attitude the hand-flown initial climb may hold,
    /// degrees, once clear of the runway (see `autotakeoff::ClimbHold`). With
    /// it the rotation holds the initial climb speed by PITCH from liftoff to
    /// the autopilot's engage, the way a crew flies V2 + 15; without it the
    /// law holds its fixed rotation attitude, as every GA single always has.
    /// EGPH 2026-09-21 and LEMG 2026-09-28: at 8 degrees a 737 at takeoff
    /// thrust went 162 to 185 kt between liftoff and 300 ft.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub climb_pitch_max_deg: Option<f64>,
    /// How far ahead the circuit's final leg leads the offset it pursues,
    /// seconds (see `circuit::final_led_xtk_m`): an autopilot whose heading
    /// answers its bug slowly runs through the centreline on the bare
    /// pursuit. Silent: no lead, the pursuit every airframe has flown. The
    /// A330 ran 66 to 67 m through at LEMG 30 (2026-09-29, #189).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub circuit_final_lead_s: Option<f64>,
    /// Path to an XChecklist file to interpret (relative to the working dir),
    /// or `acf:<file>` for a file the aircraft ships in its own folder (the
    /// Zibo's `clist.txt`: not ours to copy into this repo), resolved against
    /// the loaded `.acf`'s directory when detection picks the profile
    /// ([`crate::checklist::resolve_acf_path`]). `None` means no checklists
    /// for this airframe. The file is read-only input; the copilot interprets
    /// it (see [`crate::checklist`]).
    #[serde(default)]
    pub checklist: Option<String>,
    /// Maps the checklist file's datarefs to curated actions, per-airframe. Empty
    /// means the copilot only verifies/calls out checklist items (never acts).
    #[serde(default)]
    pub checklist_map: Vec<ChecklistMapEntry>,
    /// Only the mapped tools act: every other checklist item is read and
    /// confirmed, never written raw by the verify path. For an aircraft
    /// whose stock datarefs are its plugin's OUTPUTS (the Zibo: a stock
    /// speedbrake write sticks while its own lever never moves, so the
    /// readback would lie). `false` (the default) keeps the verify path's
    /// write for settable switches.
    #[serde(default)]
    pub checklist_map_only: bool,
    /// Which file section the checklist flow ([`crate::flow`]) reads at each
    /// phase boundary, for files whose titles the section words do not find:
    /// `climb`, `cruise`, `descent`, `before_landing`, `after_landing` -> the
    /// file's own section title. Empty (the default) means the section words
    /// match the titles themselves, as they do in the 172 and SR22 files. A
    /// LIST of titles is one flow read in order (a file that splits a phase
    /// into "CLIMB PROCEDURE (1/2)" and "(2/2)"): the parts are joined into
    /// one list, titled by their shared name ([`crate::checklist::parse`]).
    #[serde(default)]
    pub checklist_flow: std::collections::BTreeMap<String, FlowTitles>,
    /// Phase-expectation advisor: switch states the airframe *should* be in per
    /// flight phase; the monitor advises on the delta ("yaw damper's still off"
    /// at climb). Declarative SOP knowledge — the deterministic version of a
    /// human copilot's "aren't you forgetting something?".
    #[serde(default)]
    pub advisor: Vec<AdvisorItem>,
    /// Exterior/interior lights this airframe has. Empty means no `set_lights` tool.
    #[serde(default)]
    pub lights: Vec<LightProfile>,
    /// Heading/altitude/VS autopilot. `false` means no autopilot tools.
    #[serde(default)]
    pub autopilot: bool,
    /// Extra prerequisites for airframes whose AP needs more than servos-on to
    /// engage (the C90 needs its flight director up and AC power from an inverter).
    /// `None` → plain servos-on (the GA singles). See [`AutopilotPrep`].
    #[serde(default)]
    pub autopilot_prep: Option<AutopilotPrep>,
    /// Piston engine controls (throttle, mixture).
    #[serde(default)]
    pub engine: bool,
    /// Move the gear lever with the stock gear COMMANDS, not a handle write
    /// (see `tools::gear::Gear::by_command`).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub gear_by_command: bool,
    /// The gear lever's own commands and handle ([`GearCommands`]): the
    /// tool and every loop press these and read the handle back. Wins over
    /// `gear_by_command`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gear_commands: Option<GearCommands>,
    /// The speedbrake lever stepped by commands ([`SpeedbrakeLever`]):
    /// every caller of the speedbrake (the tool, the takeoff and landing
    /// configuration, the reject, the descent drag) goes through it and the
    /// stock `speedbrake_ratio` is never written.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speedbrake_lever: Option<SpeedbrakeLever>,
    /// A Boeing start: starter knob and start lever per engine
    /// ([`BoeingStartProfile`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub boeing_start: Option<BoeingStartProfile>,
    /// Thrust levers (`set_throttle`) without the piston controls — for turbine
    /// aircraft that have a throttle but no mixture/magnetos/piston start. A piston
    /// `engine` already implies a throttle, so it's only set for jets/turboprops.
    #[serde(default)]
    pub throttle: bool,
    /// Constant-speed propeller lever (`set_prop`) — the blue levers that set the
    /// governed prop RPM. `None` (fixed-pitch, e.g. the 172) → no prop tool.
    #[serde(default)]
    pub prop: Option<PropProfile>,
    /// Turbine start sequence (`start_engines`) — bleed/fuel/ignition/crank with N2
    /// monitoring, as opposed to the piston `engine` group's one-shot start.
    #[serde(default)]
    pub turbine_start: bool,
    /// Procedural power-up (`power_up`) — "all electrical systems on" runs the
    /// electrical bring-up step by step (battery, APU, bleed, buses, generators),
    /// calling out each, rather than a one-shot shortcut.
    #[serde(default)]
    pub auto_prep: bool,
    /// The aircraft scripts its OWN start behind the stock `auto_start`
    /// command (an xlua `replace_command`): one press runs the whole cold and
    /// dark bring-up and the engine start with the aircraft's real switches.
    /// The stock 737-800: battery, standby power, fire tests, APU, packs, then
    /// starter 2 and 1 on N2, generators, probe heat, bleeds, APU off, 124 s
    /// from cold to both running (probed 2026-09-17). `StartKind::Scripted`:
    /// full auto fires the command and waits on the engines.
    #[serde(default)]
    pub scripted_auto_start: bool,
    /// Per-engine FUEL CUTOFF lever, when the stock `mixture_ratio` is not the
    /// control the aircraft is actually cut with. Empty = write
    /// `sim/cockpit2/engine/actuators/mixture_ratio[i]`, which is right on
    /// every airframe whose levers the sim itself owns (the MD-82).
    ///
    /// The stock 737-800 is the case that found this (2026-09-20): its
    /// `B738.systems.lua` rewrites `mixture_ratio[0..1]` EVERY FRAME from
    /// `laminar/B738/engine/slider/condition_lever{1,2}`, which is itself an
    /// animation output driven toward `..._target`. So the stock write is
    /// accepted (`is_writable: true`), overwritten on the next frame, and does
    /// nothing: the copilot said "fuel levers cut" at a Dublin stand and the
    /// engines ran on until the pilot cut them by hand. The writable control is
    /// the `_target` pair, and only a readback can tell the two apart.
    #[serde(default)]
    pub fuel_cutoff_levers: Vec<String>,
    /// Per-engine CUTOFF commands, fired instead of writing the levers when
    /// the lever is an output its plugin owns (the Zibo's
    /// `mixture1_cutoff` / `mixture2_cutoff`). The levers above are still
    /// the readback where declared.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fuel_cutoff_commands: Vec<String>,
    /// The value those levers take at CUTOFF. Defaults to 0.0, which is cutoff
    /// on every lever seen so far; a lever whose sense is inverted says so here.
    #[serde(default)]
    pub fuel_cutoff_value: f64,
    /// Airliner autoflight extras: `set_speed` (AP airspeed select) and
    /// `autothrottle` (engage/disengage). For jets with an autothrottle.
    #[serde(default)]
    pub autothrottle: bool,
    /// Speedbrakes/spoilers (`speedbrake` extend/retract/arm). For aircraft that
    /// have them (the GA singles don't).
    #[serde(default)]
    pub speedbrake: bool,
    /// Reverse thrust (`reverse_thrust`) — rollout-only, self-stowing below
    /// taxi speed. For turboprops/jets whose model has a usable beta/reverse
    /// range on `throttle_beta_rev_ratio` (the Twin Otter).
    #[serde(default)]
    pub reverse_thrust: bool,
    /// Autoland's rollout selects reverse itself (`reverse::select`) once the
    /// wheels have settled; needs `reverse_thrust`. Off: the pilot's call only.
    #[serde(default)]
    pub rollout_reverse: bool,
    /// Adjustable altimeter (`set_barometer`).
    #[serde(default)]
    pub barometer: bool,
    /// Vacuum directional gyro that needs syncing (`align_gyro`).
    #[serde(default)]
    pub directional_gyro: bool,
    /// COM1 radio (`set_com1`).
    #[serde(default)]
    pub com_radio: bool,
    /// Transponder (`set_transponder` / squawk).
    #[serde(default)]
    pub transponder: bool,
    /// Override for airframes whose transponder MODE selector is an add-on's
    /// own knob, where the stock `transponder_mode` actuator is not an input
    /// at all but an OUTPUT the add-on rewrites every frame from that knob
    /// (the RWDesigns Twin Otter's KT73 — see
    /// [`TransponderModeKnob`]). `None` = the stock actuator, the universal case.
    #[serde(default)]
    pub transponder_mode_knob: Option<TransponderModeKnob>,
    /// What this airframe's crew sets before the roll and at the gear beyond
    /// flaps, trim and the altitude selector (see [`crate::takeoff_config`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub takeoff_config: Option<TakeoffConfigProfile>,
    /// The manual's takeoff power limit, where it has one (see
    /// [`TakeoffPowerProfile`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub takeoff_power: Option<TakeoffPowerProfile>,
    /// The seat belt sign switch, where it is the add-on's own (see
    /// [`crate::tools::signs`]). `None` with `cabin_crew` is the stock switch.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub seat_belt_signs: Option<SeatBeltSigns>,
    /// The crew's preflight on the airframe's own switches, run by full
    /// auto between the battery and the ramp items ([`PreflightProfile`]).
    /// `None`: full auto goes from the battery straight to the ramp, as
    /// every airframe did before 2026-10-07.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preflight: Option<PreflightProfile>,
    /// Whether full auto's ramp items include the panel lighting through the
    /// flood tool. `false` where the panel's rheostats are the add-on's own
    /// and no switch for them is declared: the stock flood write is undone
    /// there, and "Panel lights did not come on" was said about a switch we
    /// never had (the Zibo, EGHL 2026-10-06). Defaults to true.
    #[serde(default = "yes", skip_serializing_if = "is_true")]
    pub ramp_panel_lights: bool,
    /// The circuit selects its pattern altitude on the altitude dial and lets
    /// the autopilot's capture hold the level legs, instead of flying altitude
    /// through the VS dial (`circuit::ap_holds_level`). Per airframe because
    /// it rests on the stock VS command taking the aircraft OUT of an altitude
    /// hold, which is still unproven on the 737's MCP (history, 2026-09-19).
    #[serde(default)]
    pub ap_altitude_capture: bool,
    /// ILS approach setup (`prepare_ils`) — needs a NAV radio + approach mode.
    #[serde(default)]
    pub ils: bool,
    /// Parking brake (`set_parking_brake`).
    #[serde(default)]
    pub parking_brake: bool,
    /// Cabin crew aboard (the interphone persona — see [`crate::cabin`]).
    /// Airliner-class airframes only; `XCOPILOT_CABIN_CREW` overrides for
    /// testing on anything.
    #[serde(default)]
    pub cabin_crew: bool,
    /// Whether full auto may load this airframe from the plan at the arm
    /// (`full_auto::arm_load`, issue #95). `stock` (the default) writes the
    /// stock `weight/m_fuel` and `weight/m_fixed`, read back; `unprobed` and
    /// `own_fuel_system` are named and left to the pilot, never forced. The
    /// spoken "load the aircraft" is the pilot's and ignores it.
    #[serde(default)]
    pub load_sheet: LoadSheet,
    /// Fly an ILS COUPLED on this airframe's own approach mode (see
    /// `ils::Coupling`): armed on an intercept from below the beam, the bug
    /// and VS dial handed over as the localizer and glideslope capture.
    /// Absent: coupled wherever the surveyed autopilot has an approach mode
    /// (`ils::coupled_allowed`, `systems::ApType::has_approach`): the final
    /// is the airframe's own modes' to minimums where it has them. `false`
    /// where a capture was flown and failed, or the autopilot lands it
    /// another way (the Zibo's `autopilot_lands`).
    #[serde(default)]
    pub ils_coupled: Option<bool>,
    /// Radio height, feet, below which a coupled ILS is handed back to
    /// autoland's own VS final (`ils::CouplingIn::decouple_below_ft`).
    /// `None`: coupled to the arrest.
    #[serde(default)]
    pub ils_decouple_agl_ft: Option<f64>,
    /// The autopilot lands this airframe on an ILS ([`AutopilotLands`]).
    /// `None`: autoland's own law, as before.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub autopilot_lands: Option<AutopilotLands>,
    /// The autopilot's own approach button, where the add-on has its own
    /// (the Laminar 737's APP is `laminar/B738/autopilot/app_press`: its lamp
    /// lights). `None`: the stock command. The modes it arms are read on
    /// the `autopilot_state` bitfield (`autoflight::Modes`): the 737's
    /// `app_status` and the stock `approach_status` read 0 through captures
    /// (#81).
    #[serde(default)]
    pub approach_command: Option<String>,
    /// The `park_brake_valve` value that means SET on this airframe, where the
    /// valve is modeled. The sense is PER-AIRCRAFT (C90B: 0 = set, verified live;
    /// SR22: 1 = set — the opposite, seen as an "inverted" handle). `None` = don't
    /// touch the valve (ratio only), the safe default for unknown airframes.
    #[serde(default)]
    pub park_brake_valve_set: Option<f64>,
    /// An add-on whose own script owns the parking brake: the stock
    /// `park_brake_set` nudged the Zibo's ratio to 0.54 and its script put it
    /// back to 0 seconds later, after the reject had said "parking brake set"
    /// (the aircraft rolled to 20 kt, 2026-10-06). Its brake follows only a
    /// TOGGLE command, read before firing and confirmed on its own position.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub park_brake_toggle: Option<ParkBrakeToggle>,
    /// A turbofan's spool from the lever at takeoff power to the thrust, in
    /// seconds, added to the takeoff's stagnation clock, which then starts
    /// only once the lever is up ([`crate::autotakeoff::roll_stagnant`]).
    /// `None`: the clock runs from brake release, as it always has.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub takeoff_spool_s: Option<f64>,
    /// Electrical systems the `set_electrical` tool exposes (per-airframe wiring).
    /// Empty means no electrical tool.
    #[serde(default)]
    pub electrical: Vec<ElectricalSystem>,
    /// Cockpit flood lights (`set_flood_lights`). Separate from `lights` because
    /// the floods use a mix of write mechanisms (a scalar plus an array element)
    /// that the scalar-per-light `set_lights` can't express.
    #[serde(default)]
    pub flood_lights: bool,
    /// Override dataref for the flood lights when the airframe's flood isn't the
    /// universal pilot/copilot pair — a single on/off switch, optionally an array
    /// element written `name[i]` (the King Air's overhead flood is
    /// `sim/cockpit2/switches/generic_lights_switch[0]`). `None` → universal floods.
    #[serde(default)]
    pub flood_lights_dataref: Option<String>,
    /// Override for airframes whose flood switch is a TOGGLE command with a
    /// read-only position dataref, which no write can drive. Read first, fired
    /// only when the position differs — the Citation X rule: a toggle fired as
    /// "on" turns the system off whenever it was already on.
    ///
    /// The Baron 58 is the case that found this. Its profile declared
    /// `flood_lights_dataref: laminar/b58/lighting/flood_light_switch`, and
    /// probing the live sim showed that name is a COMMAND, not a dataref — the
    /// airframe has no dataref by that name at all, so every `set_flood_lights`
    /// on the Baron was a write into nothing that then read back nothing
    /// (2026-08-02). Its real position lives at
    /// `laminar/b58/flood_light_toggle/switch_pos`, read-only.
    #[serde(default)]
    pub flood_lights_toggle: Option<ToggleSwitch>,
    /// A command fired TWICE when the aircraft comes alive, to acknowledge the
    /// GNS430's welcome/database page — the screen that otherwise sits there
    /// until somebody presses ENT. `None` on airframes with no such unit.
    /// Fired once per power-up (see [`crate::cockpit_auto`]), never repeated
    /// while the bus stays live, so a pilot navigating the unit is never
    /// interrupted by a stray ENT.
    #[serde(default)]
    pub gps_ack_command: Option<String>,
    /// Override switch for pitot heat when the stock command is a no-op — the
    /// airframe's script drives the heat from its own control and overwrites the
    /// stock datarefs every frame (MD-82: `laminar/md82/ice/heatknob`, 0 = off,
    /// 1–9 = on; the tool writes 1/0). `None` → the stock pitot-heat command.
    #[serde(default)]
    pub pitot_heat_dataref: Option<String>,
    /// Electric/auxiliary fuel pump (`set_fuel_pump`).
    #[serde(default)]
    pub fuel_pump: bool,
    /// Override for airframes whose boost pumps are custom commands rather than
    /// the universal `fuel_pump_on` array (the RWDesigns Twin Otter's FWD/AFT
    /// pumps live behind SASL commands; writing the stock array is a no-op).
    /// When set, `set_fuel_pump` fires these and confirms from `readback`
    /// (entries may be `name[i]`). Exposes the tool even when `fuel_pump` is
    /// false.
    #[serde(default)]
    pub fuel_pump_commands: Option<PumpCommands>,
    /// Tank indices of the stock `sim/cockpit2/fuel/fuel_tank_pump_on` whose
    /// pumps come OFF when that tank is empty with an engine running (see
    /// [`crate::tools::fuel_pumps`]): the 737's CTR pumps were left running
    /// dry for a whole flight after the scripted start switched them on over
    /// an empty centre (2026-09-27). Only a CENTRELINE tank with wing tanks
    /// beside it is ever honoured (`acf_tank_X`); a main tank is never a
    /// switch to tidy. Declare only where the add-on's scripts leave the
    /// stock array as the cockpit switch. Empty (silent) = off.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fuel_tank_pumps_off_when_empty: Vec<usize>,
    /// Magneto/ignition selector (`set_magnetos`) — for the run-up mag check.
    #[serde(default)]
    pub magnetos: bool,
    /// Carburetor / alternate-air heat (`set_carb_heat`).
    #[serde(default)]
    pub carb_heat: bool,
    /// Number of engines — how many array elements the per-engine accessory
    /// tools (carb heat, fuel pump, magnetos) write. Defaults to 1 (the GA
    /// singles); a twin like the DC-3 sets 2 so both engines switch together.
    #[serde(default = "default_engines")]
    pub engines: usize,
    /// Drive the magnetos via the per-engine `ignition_on` selector (off/R/L/
    /// both) instead of the single `ignition_key` — for radials like the DC-3.
    #[serde(default)]
    pub magnetos_per_engine: bool,
    /// Fuel tank selector (`set_fuel_selector`), per-airframe positions.
    /// `None` → no selector tool.
    #[serde(default)]
    pub fuel_selector: Option<FuelSelectorProfile>,
    /// DC power switch driven by up/down cycle commands (battery / off / ground
    /// power) — the AWX DC-3 pattern. Gates `set_power`. `None` → no power tool.
    #[serde(default)]
    pub power_switch: Option<PowerSwitchProfile>,
    /// Fuel selectors driven by up/down cycle commands, one per side (the DC-3
    /// has two). Empty → no cycle fuel-selector tool. Distinct from
    /// `fuel_selector` (the single direct-command selector on the GA singles).
    #[serde(default)]
    pub fuel_selectors: Vec<CycleSelectorProfile>,
    /// Twin piston start/shutdown for cycle-switch airframes (mixture/fuel are
    /// xlua-overridden). Gates `start_engines` / `stop_engines`. `None` → use the
    /// generic `engine` group instead.
    #[serde(default)]
    pub piston_start: Option<PistonStartProfile>,
    /// Notched mixture lever (xlua-overridden). Gates `set_mixture` for airframes
    /// whose mixture is a detented lever rather than a continuous 0–100%. `None`
    /// → no notched mixture tool.
    #[serde(default)]
    pub mixture_notch: Option<MixtureNotchProfile>,
    /// Switch/condition-lever turboprop start (the Laminar King Air pattern).
    /// Gates `start_engines` / `stop_engines`. Distinct from `turbine_start`
    /// (the MD-82 jet sequence) and the piston `engine`/`piston_start` groups.
    #[serde(default)]
    pub turboprop_start: Option<TurbineStartProfile>,
    /// Mode-selector + master-switch jet start: the AIRLINER pattern, and the
    /// one the existing blocks could not express.
    ///
    /// `turbine_start` is the MD-82's, hardcoded to `laminar/md82/*`;
    /// `turboprop_start` is condition levers and starter commands. An Airbus
    /// or a 737 has neither — it has an ENG MODE knob and two guarded master
    /// switches, so every airliner here but the MD-82 had `turbine_start:
    /// false` and no way to start its own engines at all (found in the A330,
    /// 2026-09-20).
    #[serde(default)]
    pub airliner_start: Option<AirlinerStartProfile>,
    /// Pitot heat anti-ice (`set_pitot_heat`).
    #[serde(default)]
    pub pitot_heat: bool,
    /// Elevator pitch trim (`set_pitch_trim`).
    #[serde(default)]
    pub pitch_trim: bool,
    /// NAV1 radio + OBS course (`set_nav1`, `set_obs`).
    #[serde(default)]
    pub nav_radio: bool,
    /// Second comm radio + active/standby swap (`set_com2`, `swap_radio`).
    #[serde(default)]
    pub com2_radio: bool,
    /// Whether the gear retracts. `false` (e.g. the 172) means no `gear` tool.
    #[serde(default)]
    pub retractable_gear: bool,
    /// A helicopter. Every flying and ground law here is fixed-wing (a
    /// rotation through the pitch channel, a flare through the VS dial, a
    /// throttle governor, a pattern sized to a bank angle), so the registry
    /// withholds them all ([`crate::tools::ROTORCRAFT_WITHHELD`]) whatever
    /// the other flags say; the crew keeps the switches, the radios and the
    /// autopilot dials. The S-76C, 2026-09-16.
    #[serde(default)]
    pub rotorcraft: bool,
    /// A helicopter's engine start and shutdown ([`crate::tools::heli`]):
    /// held starters, throttles as the fuel control (0 is OFF), the rotor
    /// read back at its governed speed.
    #[serde(default)]
    pub heli_start: Option<HeliStartProfile>,
    /// Transition altitude (ft MSL): at or above it, "set the altimeter" dials in
    /// standard pressure (29.92 inHg / 1013 hPa) instead of local QNH — that's the
    /// rule above the transition altitude. Defaults to 18,000 ft (US FL180); lower
    /// it per-profile for regions with a lower transition altitude.
    #[serde(default = "default_transition_altitude")]
    pub transition_altitude_ft: f64,
    /// How far the snap-to-panel camera moves toward the saved pose: 1.0 = all the
    /// way (the captured close-up), 0.5 = a half glance from wherever you're
    /// looking — position and angles interpolated, so the view turns toward the
    /// panel without zooming into it.
    #[serde(default = "default_view_blend")]
    pub view_blend: f64,
    /// Cockpit geography: lowercase item name → spoken location ("on the left
    /// subpanel, next to the avionics master"). Answers "where is the X?"
    /// deterministically — the one thing PDF manuals can't carry (their location
    /// diagrams don't survive text extraction). Grows per-airframe as verified.
    #[serde(default)]
    pub locations: std::collections::HashMap<String, String>,
    /// Snap-to-panel: tool name → a saved 3D-cockpit camera position to jump to
    /// just before that action runs, so you watch the switch move. Empty (default)
    /// = no snapping. Positions are captured live from the pilot's own framing (the
    /// copilot reads the head datarefs and stores them here), so no fiddly X-Plane
    /// quick-look saving. E.g. `gear: {x: -0.35, y: 0.83, z: -0.44, psi: 12, the: -20}`.
    #[serde(default)]
    pub view_map: std::collections::HashMap<String, PanelView>,
    /// The altimeter knobs' fast paths ([`AltimeterProfile`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub altimeter: Option<AltimeterProfile>,
    /// Anti-ice and de-ice systems ([`AntiIceProfile`]); `None` reads them
    /// from the `.acf`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub anti_ice: Option<AntiIceProfile>,
    /// The autopilot's own modes ([`AutoflightProfile`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub autoflight: Option<AutoflightProfile>,
}

/// A saved 3D-cockpit camera position — the pilot's-head datarefs
/// (`sim/graphics/view/pilots_head_*`): position relative to the CG in metres and
/// the view angles in degrees. Restored by writing them back after selecting the
/// 3D cockpit view. `phi` (roll) defaults to 0 — rarely needed.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PanelView {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Heading (yaw), degrees.
    pub psi: f64,
    /// Pitch, degrees.
    pub the: f64,
    #[serde(default)]
    pub phi: f64,
}

/// US transition altitude (FL180). See [`AircraftProfile::transition_altitude_ft`].
fn default_transition_altitude() -> f64 {
    18_000.0
}

/// Full snap by default — profiles opt into a softer glance. See `view_blend`.
fn default_view_blend() -> f64 {
    1.0
}

/// Engine count for the per-engine accessory tools — 1 (the GA singles) unless
/// a profile says otherwise. See [`AircraftProfile::engines`].
fn default_engines() -> usize {
    1
}

/// Autopilot engagement prerequisites beyond servos-on, for airframes that need
/// them. The C90's AP won't hold without its **flight director** switch up (its
/// `fd_monitor` drops the servos otherwise) and is **AC-powered**, so with the
/// inverter off it has no power to engage at all — which lets the tool name the
/// real cause ("no AC power, turn the inverter on") instead of a generic hint.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AutopilotPrep {
    /// Commands to raise the flight director before engaging the servos. Empty =
    /// none. C90: `laminar/c90/avionics/switch/fd_up`.
    #[serde(default)]
    pub flight_director: Vec<String>,
    /// True if the AP is AC-powered (needs an inverter). Drives the honest
    /// "no AC power" readback when it fails to engage.
    #[serde(default)]
    pub ac_powered: bool,
}

/// Constant-speed propeller control, per airframe. The blue levers command a
/// *governed RPM*, not a raw position; X-Plane's actuator dataref carries it in
/// radians/second (all engines), so the tool works in RPM (what the pilot says and
/// the tach shows) and converts. Named detents map to the airframe's book settings;
/// an explicit RPM is clamped to [`min_rpm`, `max_rpm`]. Verified live on the C90B
/// with `scripts/xp_watch.py` (the lever moves `..._rad_sec_all`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropProfile {
    /// Actuator dataref (rad/sec, all engines). C90B:
    /// `sim/cockpit2/engine/actuators/prop_rotation_speed_rad_sec_all`.
    pub dataref: String,
    /// Full-forward / takeoff governed RPM ("props full", "props up").
    pub max_rpm: f64,
    /// Cruise detent RPM ("props to cruise").
    pub cruise_rpm: f64,
    /// Aft governing limit / low RPM ("props back", "props low").
    pub min_rpm: f64,
}

// The profile's sections and the airframe's numbers in their own files,
// whole and unchanged (refactor phase 4); every path stays
// `crate::aircraft::...` through these re-exports.
mod schema_laws;
pub use schema_laws::*;
mod schema_systems;
pub use schema_systems::*;
mod schema_config;
pub use schema_config::*;
mod facts;
pub use facts::*;
mod speeds;
pub use speeds::*;
mod generic;
pub mod learned;
pub use generic::*;
mod builtin_profiles;
use builtin_profiles::*;

#[cfg(test)]
#[allow(clippy::items_after_test_module)]
mod tests;

#[cfg(test)]
mod generic_profile_tests;

/// Every hardcoded `sim/...` name in the workspace, checked against the
/// running install's own `DataRefs.txt` and `Commands.txt`. The live probe
/// walks the PROFILE's names on every airframe (`profile_references`); the
/// Rust tools' constants were never probed, and two of them were misspelt for
/// months (`vvi_dial_fpm` in autoland, `compass_heading_deg_magnetic` in the
/// stall recovery, found 2026-09-15), each read failing silently into a
/// fallback that impersonated a measurement. Needs the sim on disk:
/// `cargo test -- --ignored real_install_names`.
#[path = "aircraft/real_install_names_tests.rs"]
#[cfg(test)]
mod real_install_names;

/// Does the write LAND? The companion to `real_install_names`, which only
/// asks whether a name resolves.
///
/// A study-level add-on owns much of its own cockpit: it REPLACES stock
/// commands (our fire reaches its handler — fine), and it WRITES stock
/// datarefs every frame (our write is overwritten a frame later — a silent
/// no-op). Both resolve. Both probe clean. One of them does nothing.
///
/// Three failures in one evening on the A330 (2026-09-20) were this shape:
/// a name declared as a dataref that was really a command, an array element
/// written by subscript, and an `_on` command the aircraft leaves stock while
/// replacing only its `_toggle`. Every one passed the existence probe.
///
/// Needs the aircraft on disk:
/// `cargo test -- --ignored real_install_writes`.
#[path = "aircraft/real_install_writes_tests.rs"]
#[cfg(test)]
mod real_install_writes;

#[cfg(test)]
mod vmo_tests;

/// The rules unified by the refactor audit (2026-10-04, findings #2, #10,
/// #12, #13): one home each, pinned here.
#[cfg(test)]
mod one_rule_tests;

/// The speeds by weight (`weight_speeds:`, `reference.stall_ref_weight_kg`),
/// pinned on the Zibo's card (LEMG 30, 2026-10-07).
#[cfg(test)]
mod weight_speeds_tests;
