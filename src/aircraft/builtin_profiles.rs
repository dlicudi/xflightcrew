//! The built-in profiles: the Cessna 172SP and the Cirrus SR22.

use super::*;

/// The X-Plane stock Cessna 172SP Skyhawk.
///
/// Flap detents 0°/10°/20°/30°; Vfe 110 KIAS for the first 10° and 85 KIAS
/// beyond. Fixed gear. Full piston panel + a heading/altitude/VS autopilot.
pub(super) fn cessna_172sp() -> AircraftProfile {
    let detent = |label: &str, spoken: &str, ratio: f64, vfe: Option<f64>| FlapDetent {
        retract_above_kts: None,
        vs_kts: None,
        idle_decel_kts_s: None,
        label: label.into(),
        spoken: spoken.into(),
        ratio,
        vfe_kts: vfe,
    };
    let light =
        |key: &str, spoken: &str, dataref: &str, cmd_on: &str, cmd_off: &str| LightProfile {
            also_datarefs: Vec::new(),
            toggles: Vec::new(),
            auto_at_night: false,
            key: key.into(),
            spoken: spoken.into(),
            dataref: dataref.into(),
            command_on: Some(cmd_on.into()),
            command_off: Some(cmd_off.into()),
        };
    let fuel_pos = |key: &str, spoken: &str, command: &str, value: Option<f64>| FuelPosition {
        key: key.into(),
        spoken: spoken.into(),
        command: command.into(),
        value,
    };
    AircraftProfile {
        id: "cessna_172sp".into(),
        reverse_stow_levers: vec![],
        reverse_hold_command: None,
        takeoff_trim: None,
        takeoff_thrust: None,
        weight_speeds: None,
        throttle_hardware_axis: None,
        yoke_input: None,
        display_name: "Cessna 172SP Skyhawk".into(),
        simbrief_types: Vec::new(),
        matcher: Some(ProfileMatch {
            icao: vec!["C172".into()],
            acf: vec!["Laminar Research/Cessna 172 SP/".into()],
            ..Default::default()
        }),
        engines: 1,
        magnetos_per_engine: false,
        flood_lights_dataref: None,
        flood_lights_toggle: None,
        gps_ack_command: None,
        // Cycle-switch systems (the DC-3 pattern) — not used by the GA singles.
        power_switch: None,
        fuel_selectors: Vec::new(),
        piston_start: None,
        mixture_notch: None,
        turboprop_start: None,
        // A piston has no ENG MODE knob and no master switches.
        airliner_start: None,
        retractable_gear: false,
        gear_by_command: false,
        gear_commands: None,
        speedbrake_lever: None,
        boeing_start: None,
        fuel_cutoff_commands: Vec::new(),
        rotorcraft: false,
        heli_start: None,
        transition_altitude_ft: 18_000.0,
        view_blend: 1.0,
        locations: std::collections::HashMap::new(),
        view_map: std::collections::HashMap::new(),
        autopilot: true,
        autopilot_prep: None,
        engine: true,
        throttle: false,
        prop: None,
        turbine_start: false,
        auto_prep: false,
        scripted_auto_start: false,
        fuel_cutoff_levers: Vec::new(),
        fuel_cutoff_value: 0.0,
        autothrottle: false,
        speedbrake: false,
        barometer: true,
        directional_gyro: true,
        com_radio: true,
        transponder: true,
        // Stock actuator: both builtins model a sim-native transponder.
        transponder_mode_knob: None,
        takeoff_config: None,
        takeoff_power: None,
        seat_belt_signs: None,
        preflight: None,
        ramp_panel_lights: true,
        ap_altitude_capture: false,
        ils: true,
        parking_brake: true,
        cabin_crew: false,
        load_sheet: LoadSheet::Stock,
        ils_coupled: true,
        ils_decouple_agl_ft: Some(200.0),
        autopilot_lands: None,
        approach_command: None,
        bus_volts_nominal: None,
        approach_thrust_min: None,
        roll_rudder: None,
        rotate_elevator: None,
        ground_plant: None,
        rollout_steer: None,
        speed_governor: None,
        cabin_air: None,
        pressurised: Some(false),
        park_brake_valve_set: None,
        park_brake_toggle: None,
        takeoff_spool_s: None,
        electrical: {
            let e = |key: &str, spoken: &str, on: &[&str], off: &[&str], rb: &[&str]| {
                ElectricalSystem {
                    write_datarefs: Vec::new(),
                    toggles: Vec::new(),
                    guards: Vec::new(),
                    key: key.into(),
                    spoken: spoken.into(),
                    on_commands: on.iter().map(|s| s.to_string()).collect(),
                    off_commands: off.iter().map(|s| s.to_string()).collect(),
                    readback: rb.iter().map(|s| s.to_string()).collect(),
                }
            };
            vec![
                e(
                    "battery",
                    "battery",
                    &["sim/electrical/battery_1_on"],
                    &["sim/electrical/battery_1_off"],
                    &["sim/cockpit/electrical/battery_on"],
                ),
                e(
                    "alternator",
                    "alternator",
                    &["sim/electrical/generator_1_on"],
                    &["sim/electrical/generator_1_off"],
                    &["sim/cockpit/electrical/generator_on[0]"],
                ),
                // The two AVIONICS switches: BUS 1 (avionics master) + BUS 2 (cross-tie).
                e(
                    "avionics",
                    "avionics master",
                    &["sim/systems/avionics_on", "sim/electrical/cross_tie_on"],
                    &["sim/systems/avionics_off", "sim/electrical/cross_tie_off"],
                    &[
                        "sim/cockpit/electrical/avionics_on",
                        "sim/cockpit2/electrical/cross_tie",
                    ],
                ),
                // Split BAT/ALT rocker — battery + alternator together, NOT avionics.
                e(
                    "master",
                    "master",
                    &[
                        "sim/electrical/battery_1_on",
                        "sim/electrical/generator_1_on",
                    ],
                    &[
                        "sim/electrical/battery_1_off",
                        "sim/electrical/generator_1_off",
                    ],
                    &[
                        "sim/cockpit/electrical/battery_on",
                        "sim/cockpit/electrical/generator_on[0]",
                    ],
                ),
            ]
        },
        flood_lights: true,
        fuel_pump: true,
        fuel_pump_commands: None,
        fuel_tank_pumps_off_when_empty: Vec::new(),
        reverse_thrust: false,
        rollout_reverse: false,
        magnetos: true,
        carb_heat: true,
        // The 172 selector is LEFT/BOTH/RIGHT (OFF is a separate shutoff valve,
        // not a selector position). Commands animate the 3D knob; the logical
        // position dataref confirms (1 left, 3 right, 4 both).
        fuel_selector: Some(FuelSelectorProfile {
            readback: Some("sim/cockpit2/fuel/fuel_tank_selector".into()),
            positions: vec![
                fuel_pos("left", "left", "sim/fuel/fuel_selector_lft", Some(1.0)),
                fuel_pos("both", "both", "sim/fuel/fuel_selector_all", Some(4.0)),
                fuel_pos("right", "right", "sim/fuel/fuel_selector_rgt", Some(3.0)),
            ],
        }),
        pitot_heat: true,
        pitot_heat_dataref: None,
        advisor: vec![],
        pitch_trim: true,
        nav_radio: true,
        com2_radio: true,
        flaps: Some(FlapProfile {
            detents: vec![
                detent("up", "up", 0.0, None),
                detent("10", "ten degrees", 1.0 / 3.0, Some(110.0)),
                detent("20", "twenty degrees", 2.0 / 3.0, Some(85.0)),
                detent("full", "full", 1.0, Some(85.0)),
            ],
            takeoff_ratio: Some(0.0), // 172 takes off clean
            by_command: None,
        }),
        fuel: Some(FuelProfile {
            reserve_caution_min: 45.0,
            reserve_warning_min: 30.0,
            kg_per_gal: None,
            spoken_unit: None,
            cruise_burn_kg_hr: 24.0,
        }),
        // 172SP clean stall ~48 KIAS (power-off, flaps up).
        vs_clean_kts: Some(48.0),
        // 172SP Vne (never-exceed) 163 KIAS.
        vne_kts: Some(163.0),
        vmo_high: None,
        vlo_kts: None,
        gear_warning: None,
        // The GA pace is the built-in default — nothing to declare.
        taxi: None,
        throttle_dataref: None,
        tail_strike_deg: None,
        // A light single runs out of elevator before it runs out of tail
        // clearance, and only ever needs the rotation held BACK.
        pitch_authority_down: None,
        climb_pitch_max_deg: None,
        circuit_final_lead_s: None,
        // Reference figures from the X-Plane C172 Pilot's Operating Manual.
        reference: Some(ReferenceProfile {
            kind: Some("single-engine aircraft".into()),
            // A piston does not step-climb: it goes as high as it can and
            // stays. No optimum to advise, so nothing is said.
            optimum_alt_ft: None,
            // ...and the GA vertical rates are the shared default already.
            enroute_climb_fpm: None,
            enroute_descent_fpm: None,
            optimum_alt_at_kg: None,
            optimum_alt_ft_per_1000kg: None,
            vno_kts: Some(129.0),
            va_kts: Some(99.0),
            vx_kts: Some(62.0),
            vy_kts: Some(74.0),
            // A trainer climbs out at Vy; only airframes that do not say so.
            initial_climb_kts: None,
            acceleration_height_agl_ft: None,
            vyse_kts: None,
            vmca_kts: None,
            vr_kts: Some(55.0),
            rotate_rate_deg_s: None,
            best_glide_kts: Some(68.0),
            vs0_kts: Some(40.0),
            approach_kts: Some(65.0),
            max_crosswind_kts: Some(15.0),
            cruise_kts: Some(124.0),
            cruise_ias_kts: Some(108.0), // KIAS to hold in cruise (124 kt TAS)
            best_climb_fpm: Some(730.0),
            max_gross_lb: Some(2558.0),
            empty_weight_lb: Some(1640.0),
            useful_load_lb: Some(918.0),
            service_ceiling_ft: Some(14000.0),
            fuel_capacity_gal: Some(56.0),
            fuel_useable_gal: Some(53.0),
            endurance_hours: Some(4.0),
            takeoff_distance_ft: Some(1630.0),
            // No separate ground roll: on a GA single the over-50-ft figure is
            // already a sane "runway needed", and it is what has been flown.
            takeoff_roll_ft: None,
            landing_distance_ft: Some(1335.0),
            landing_roll_ft: None,
            max_rpm: Some(2700.0),
            oil_temp_min_f: Some(100.0),
            oil_temp_max_f: Some(245.0),
            oil_press_min_psi: Some(50.0),
            oil_press_max_psi: Some(90.0),
            stall_ref_weight_kg: None,
        }),
        checklist: Some("checklists/cessna_172sp.txt".into()),
        checklist_map: {
            let m = |s: &str, a: ChecklistAction| ChecklistMapEntry {
                match_: s.into(),
                action: a,
            };
            use ChecklistAction::*;
            vec![
                m("parkbrake", ParkingBrake),
                m("autopilot/autopilot_on", AutopilotOff),
                m("electrical/battery_on", Electrical("battery".into())),
                m("electrical/avionics_on", Electrical("avionics".into())),
                m("electrical/generator_on", Electrical("alternator".into())),
                m("pitot_heat_on", PitotHeat),
                m("fuel_pump_on", FuelPump),
                m("transponder_mode", Transponder),
                m("fuel_tank_selector", FuelSelector),
                m("flap_ratio", Flaps),
                m("mixture_ratio", Mixture),
                m("throttle_ratio", Throttle),
                m("ignition_key", Ignition),
                m("elevator_trim", PitchTrim),
                m("taxi_light", Light("taxi".into())),
                m("landing_lights", Light("landing".into())),
                m("strobe_lights", Light("strobe".into())),
                m("beacon_lights", Light("beacon".into())),
                m("navigation_lights", Light("nav".into())),
                m("nav_lights", Light("nav".into())),
            ]
        },
        checklist_flow: Default::default(),
        checklist_map_only: false,
        altimeter: None,
        anti_ice: None,
        autoflight: None,
        lights: vec![
            light(
                "nav",
                "navigation lights",
                "sim/cockpit2/switches/navigation_lights_on",
                "sim/lights/nav_lights_on",
                "sim/lights/nav_lights_off",
            ),
            light(
                "beacon",
                "beacon",
                "sim/cockpit2/switches/beacon_on",
                "sim/lights/beacon_lights_on",
                "sim/lights/beacon_lights_off",
            ),
            light(
                "strobe",
                "strobe lights",
                "sim/cockpit2/switches/strobe_lights_on",
                "sim/lights/strobe_lights_on",
                "sim/lights/strobe_lights_off",
            ),
            light(
                "landing",
                "landing light",
                "sim/cockpit2/switches/landing_lights_on",
                "sim/lights/landing_lights_on",
                "sim/lights/landing_lights_off",
            ),
            light(
                "taxi",
                "taxi light",
                "sim/cockpit2/switches/taxi_light_on",
                "sim/lights/taxi_lights_on",
                "sim/lights/taxi_lights_off",
            ),
        ],
    }
}

/// The X-Plane stock Cirrus SR22 (Laminar). Built from the GoofyJP XChecklist
/// file, the POH, and the rustdecks SR22 deck (the verified dataref/command
/// source). Enabled systems write universal X-Plane datarefs or rustdecks-
/// verified commands: the dual battery/alternator, mixture, throttle, boost
/// pump, key-start ignition, pitch trim, and the off/left/right fuel selector.
/// Still off: carb heat (fuel-injected) and the vacuum DG (G1000 glass).
pub(super) fn cirrus_sr22() -> AircraftProfile {
    let detent = |label: &str, spoken: &str, ratio: f64, vfe: Option<f64>| FlapDetent {
        retract_above_kts: None,
        vs_kts: None,
        idle_decel_kts_s: None,
        label: label.into(),
        spoken: spoken.into(),
        ratio,
        vfe_kts: vfe,
    };
    let light =
        |key: &str, spoken: &str, dataref: &str, cmd_on: &str, cmd_off: &str| LightProfile {
            also_datarefs: Vec::new(),
            toggles: Vec::new(),
            auto_at_night: false,
            key: key.into(),
            spoken: spoken.into(),
            dataref: dataref.into(),
            command_on: Some(cmd_on.into()),
            command_off: Some(cmd_off.into()),
        };
    let fuel_pos = |key: &str, spoken: &str, command: &str, value: Option<f64>| FuelPosition {
        key: key.into(),
        spoken: spoken.into(),
        command: command.into(),
        value,
    };
    AircraftProfile {
        id: "cirrus_sr22".into(),
        reverse_stow_levers: vec![],
        reverse_hold_command: None,
        takeoff_trim: None,
        takeoff_thrust: None,
        weight_speeds: None,
        throttle_hardware_axis: None,
        yoke_input: None,
        display_name: "Cirrus SR22".into(),
        simbrief_types: Vec::new(),
        matcher: Some(ProfileMatch {
            icao: vec!["SR22".into()],
            acf: vec!["Laminar Research/Cirrus SR22/".into()],
            ..Default::default()
        }),
        engines: 1,
        magnetos_per_engine: false,
        flood_lights_dataref: None,
        flood_lights_toggle: None,
        gps_ack_command: None,
        // Cycle-switch systems (the DC-3 pattern) — not used by the GA singles.
        power_switch: None,
        fuel_selectors: Vec::new(),
        piston_start: None,
        mixture_notch: None,
        turboprop_start: None,
        // A piston has no ENG MODE knob and no master switches.
        airliner_start: None,
        retractable_gear: false,
        gear_by_command: false,
        gear_commands: None,
        speedbrake_lever: None,
        boeing_start: None,
        fuel_cutoff_commands: Vec::new(),
        rotorcraft: false,
        heli_start: None,
        transition_altitude_ft: 18_000.0,
        view_blend: 1.0,
        locations: std::collections::HashMap::new(),
        view_map: std::collections::HashMap::new(),
        // Universal-dataref tools — safe across airframes.
        autopilot: true,
        autopilot_prep: None,
        barometer: true,
        com_radio: true,
        com2_radio: true,
        nav_radio: true,
        transponder: true,
        // Stock actuator: both builtins model a sim-native transponder.
        transponder_mode_knob: None,
        takeoff_config: None,
        takeoff_power: None,
        seat_belt_signs: None,
        preflight: None,
        ramp_panel_lights: true,
        ap_altitude_capture: false,
        ils: true,
        parking_brake: true,
        cabin_crew: false,
        load_sheet: LoadSheet::Stock,
        ils_coupled: false,
        ils_decouple_agl_ft: None,
        autopilot_lands: None,
        approach_command: None,
        bus_volts_nominal: None,

        approach_thrust_min: None,
        roll_rudder: None,
        rotate_elevator: None,
        ground_plant: None,
        rollout_steer: None,
        speed_governor: None,
        cabin_air: None,
        pressurised: Some(false),
        park_brake_valve_set: None,
        park_brake_toggle: None,
        takeoff_spool_s: None,
        pitot_heat: true,
        pitot_heat_dataref: None,
        advisor: vec![],
        // Throttle/mixture/start write universal datarefs (throttle_ratio_all,
        // mixture_ratio_all, the starter command) — confirmed against the
        // rustdecks SR22 deck. (start_engine sets battery 1 + alternator 1 only;
        // the SR22's second battery/alternator need the dual-electrical tool.)
        engine: true,
        throttle: false,
        prop: None,
        turbine_start: false,
        auto_prep: false,
        scripted_auto_start: false,
        fuel_cutoff_levers: Vec::new(),
        fuel_cutoff_value: 0.0,
        autothrottle: false,
        speedbrake: false,
        // Dual battery/alternator + avionics — datarefs/commands verified against
        // the rustdecks SR22 deck.
        electrical: {
            let e = |key: &str, spoken: &str, on: &[&str], off: &[&str], rb: &[&str]| {
                ElectricalSystem {
                    write_datarefs: Vec::new(),
                    toggles: Vec::new(),
                    guards: Vec::new(),
                    key: key.into(),
                    spoken: spoken.into(),
                    on_commands: on.iter().map(|s| s.to_string()).collect(),
                    off_commands: off.iter().map(|s| s.to_string()).collect(),
                    readback: rb.iter().map(|s| s.to_string()).collect(),
                }
            };
            vec![
                e(
                    "battery1",
                    "battery one",
                    &["sim/electrical/battery_1_on"],
                    &["sim/electrical/battery_1_off"],
                    &["sim/cockpit2/electrical/battery_on[0]"],
                ),
                e(
                    "battery2",
                    "battery two",
                    &["sim/electrical/battery_2_on"],
                    &["sim/electrical/battery_2_off"],
                    &["sim/cockpit2/electrical/battery_on[1]"],
                ),
                e(
                    "alternator1",
                    "alternator one",
                    &["sim/electrical/generator_1_on"],
                    &["sim/electrical/generator_1_off"],
                    &["sim/cockpit2/electrical/generator_on[0]"],
                ),
                e(
                    "alternator2",
                    "alternator two",
                    &["sim/electrical/generator_2_on"],
                    &["sim/electrical/generator_2_off"],
                    &["sim/cockpit2/electrical/generator_on[1]"],
                ),
                e(
                    "batteries",
                    "both batteries",
                    &["sim/electrical/battery_1_on", "sim/electrical/battery_2_on"],
                    &[
                        "sim/electrical/battery_1_off",
                        "sim/electrical/battery_2_off",
                    ],
                    &[
                        "sim/cockpit2/electrical/battery_on[0]",
                        "sim/cockpit2/electrical/battery_on[1]",
                    ],
                ),
                e(
                    "alternators",
                    "both alternators",
                    &[
                        "sim/electrical/generator_1_on",
                        "sim/electrical/generator_2_on",
                    ],
                    &[
                        "sim/electrical/generator_1_off",
                        "sim/electrical/generator_2_off",
                    ],
                    &[
                        "sim/cockpit2/electrical/generator_on[0]",
                        "sim/cockpit2/electrical/generator_on[1]",
                    ],
                ),
                e(
                    "avionics",
                    "avionics master",
                    &["sim/systems/avionics_on"],
                    &["sim/systems/avionics_off"],
                    &["sim/cockpit/electrical/avionics_on"],
                ),
            ]
        },
        // Key-start ignition: set_magnetos writes the universal ignition_key
        // (0 off/1 R/2 L/3 both/4 start) and start_engine engages starter 1
        // (sim/starters/engage_starter_1, the command the rustdecks deck uses).
        magnetos: true,
        carb_heat: false, // fuel-injected — no carb heat
        // Boost pump: set_fuel_pump writes sim/cockpit2/engine/actuators/
        // fuel_pump_on[0] — the dataref the rustdecks deck drives for BOOST
        // (not laminar/sr22/switch/boost_pump, which an earlier scaffold guessed).
        fuel_pump: true,
        fuel_pump_commands: None,
        fuel_tank_pumps_off_when_empty: Vec::new(),
        reverse_thrust: false,
        rollout_reverse: false,
        // OFF / LEFT / RIGHT — no "both" (low-wing twin tanks, fed one at a
        // time). Selector commands verified against the rustdecks deck. The
        // standard sim/cockpit2/fuel/fuel_tank_selector (0=off/1=left/2=right) is
        // the readback the GoofyJP checklist trusts, so it both confirms a write
        // and lets the checklist decode the file's `fuel_tank_selector:N` items.
        fuel_selector: Some(FuelSelectorProfile {
            readback: Some("sim/cockpit2/fuel/fuel_tank_selector".into()),
            positions: vec![
                fuel_pos("off", "off", "laminar/sr22/fuel_sel_left_off", Some(0.0)),
                fuel_pos(
                    "left",
                    "left",
                    "laminar/sr22/fuel_sel_left_select",
                    Some(1.0),
                ),
                fuel_pos(
                    "right",
                    "right",
                    "laminar/sr22/fuel_sel_right_select",
                    Some(2.0),
                ),
            ],
        }),
        directional_gyro: false, // G1000 glass — no vacuum DG
        flood_lights: false,
        // set_pitch_trim writes the universal elevator_trim; "takeoff" centres it
        // (within the file's T/O band).
        pitch_trim: true,
        // Flaps 0 / 50% / 100% (Vfe 150 / 110 KIAS); set_flaps writes the
        // universal flap_ratio. Detent ratios: verify.
        flaps: Some(FlapProfile {
            detents: vec![
                detent("up", "up", 0.0, None),
                detent("50", "fifty percent", 0.5, Some(150.0)),
                detent("full", "full", 1.0, Some(110.0)),
            ],
            takeoff_ratio: None,
            by_command: None,
        }),
        // SR22 ~92 gal usable, ~17 gal/hr cruise (~46 kg/hr). Verify.
        fuel: Some(FuelProfile {
            reserve_caution_min: 45.0,
            reserve_warning_min: 30.0,
            kg_per_gal: None,
            spoken_unit: None,
            cruise_burn_kg_hr: 46.0,
        }),
        // VS1 (clean) ~76 KIAS; VS0 (landing) ~61.
        // SR22 clean stall (Vs) 70 KIAS, per the X-Plane SR22 POM.
        vs_clean_kts: Some(70.0),
        // SR22 Vne (never-exceed) 204 KIAS.
        vne_kts: Some(204.0),
        vmo_high: None,
        vlo_kts: None,
        gear_warning: None,
        taxi: None,
        throttle_dataref: None,
        tail_strike_deg: None,
        // A light single runs out of elevator before it runs out of tail
        // clearance, and only ever needs the rotation held BACK.
        pitch_authority_down: None,
        climb_pitch_max_deg: None,
        circuit_final_lead_s: None,
        // Reference figures from the X-Plane Cirrus SR22 Pilot's Operating Manual
        // (the modeled aircraft is the turbo TSIO-550-K, ceiling 25,000 ft).
        reference: Some(ReferenceProfile {
            kind: Some("single-engine aircraft".into()),
            // A piston does not step-climb: it goes as high as it can and
            // stays. No optimum to advise, so nothing is said.
            optimum_alt_ft: None,
            // ...and the GA vertical rates are the shared default already.
            enroute_climb_fpm: None,
            enroute_descent_fpm: None,
            optimum_alt_at_kg: None,
            optimum_alt_ft_per_1000kg: None,
            vno_kts: Some(178.0),
            va_kts: Some(133.0),
            vx_kts: Some(88.0),
            vy_kts: Some(101.0),
            initial_climb_kts: None,
            acceleration_height_agl_ft: None,
            vyse_kts: None,
            vmca_kts: None,
            vr_kts: Some(73.0),
            rotate_rate_deg_s: None,
            best_glide_kts: Some(88.0),
            vs0_kts: Some(60.0),
            approach_kts: None,
            max_crosswind_kts: Some(21.0),
            cruise_kts: Some(180.0), // normal cruise (manual lists 213 KTAS max)
            cruise_ias_kts: None,    // no full-auto for the SR22 yet
            best_climb_fpm: Some(1200.0),
            max_gross_lb: Some(3600.0),
            empty_weight_lb: Some(2354.0),
            useful_load_lb: Some(1260.0),
            service_ceiling_ft: Some(25000.0),
            fuel_capacity_gal: Some(92.0),
            fuel_useable_gal: None,
            endurance_hours: Some(5.0), // 92 gal / ~18 gph
            takeoff_distance_ft: None,
            takeoff_roll_ft: None,
            landing_distance_ft: None,
            landing_roll_ft: None,
            max_rpm: Some(2700.0),
            oil_temp_min_f: None,
            oil_temp_max_f: None,
            oil_press_min_psi: None,
            oil_press_max_psi: None,
            stall_ref_weight_kg: None,
        }),
        checklist: Some("checklists/cirrus_sr22.txt".into()),
        // Lights — datarefs verified against the rustdecks SR22 deck; universal
        // on/off commands.
        lights: vec![
            light(
                "nav",
                "nav lights",
                "sim/cockpit2/switches/navigation_lights_on",
                "sim/lights/nav_lights_on",
                "sim/lights/nav_lights_off",
            ),
            light(
                "strobe",
                "strobe lights",
                "sim/cockpit2/switches/strobe_lights_on",
                "sim/lights/strobe_lights_on",
                "sim/lights/strobe_lights_off",
            ),
            light(
                "landing",
                "landing light",
                "sim/cockpit2/switches/landing_lights_switch[0]",
                "sim/lights/landing_lights_on",
                "sim/lights/landing_lights_off",
            ),
        ],
        // Acts on universal-dataref systems plus the now-wired boost pump,
        // key-start ignition, pitch trim and the off/left/right fuel selector
        // (decoded from the profile's position values). The dual battery/
        // alternator stay call-out — their file conditions are compound `&&`.
        checklist_map: {
            let m = |s: &str, a: ChecklistAction| ChecklistMapEntry {
                match_: s.into(),
                action: a,
            };
            use ChecklistAction::*;
            vec![
                m("parkbrake", ParkingBrake),
                m("transponder_mode", Transponder),
                m("pitot_heat_on", PitotHeat),
                m("throttle_ratio", Throttle),
                m("ENGN_mixt", Mixture),
                m("flap_handle", Flaps),
                m("avionics_on", Electrical("avionics".into())), // batteries are compound → call-out
                m("nav_lights", Light("nav".into())),
                m("strobe_lights", Light("strobe".into())),
                m("landing_lights", Light("landing".into())),
                m("boost_pump", FuelPump), // laminar/sr22/switch/boost_pump → set_fuel_pump
                m("ignition_key", Ignition), // actuators/ignition_key: 4=start, 0=off
                m("ignition_on", Ignition), // mag test: 1=R, 2=L, 3=both
                m("elevator_trim", PitchTrim), // "set T/O" → centre (not ail_trim)
                m("fuel_tank_selector", FuelSelector), // 0=off/1=left/2=right
            ]
        },
        checklist_flow: Default::default(),
        checklist_map_only: false,
        altimeter: None,
        anti_ice: None,
        autoflight: None,
    }
}
