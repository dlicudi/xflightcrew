//! Read-only X-Plane datarefs for live aircraft state.
//!
//! These are the *gauges and status* the copilot reads to ground its answers and
//! the dashboard — not the curated write targets a tool owns (those stay in the
//! tool, per the tool-surface rule in CLAUDE.md). Centralizing the read names
//! here means the state snapshot, the dashboard poller, and the autopilot status
//! reads can't drift to different strings for the same value.

// --- Primary flight gauges (pilot side) ---
pub const ALTITUDE_FT: &str = "sim/cockpit2/gauges/indicators/altitude_ft_pilot";
pub const AIRSPEED_KTS: &str = "sim/cockpit2/gauges/indicators/airspeed_kts_pilot";
pub const VVI_FPM: &str = "sim/cockpit2/gauges/indicators/vvi_fpm_pilot";
/// The flight model's own vertical speed, fpm: the gauge above lags it by
/// about a second (0.6 to 1.0 s measured in flares, #198), so a touchdown's
/// sink is judged from this one.
pub const VS_FPM_FLIGHT_MODEL: &str = "sim/flightmodel/position/vh_ind_fpm";
pub const COMPASS_HEADING_DEG: &str = "sim/cockpit2/gauges/indicators/compass_heading_deg_mag";
pub const BANK_DEG: &str = "sim/flightmodel/position/true_phi";
pub const PITCH_DEG: &str = "sim/flightmodel/position/true_theta";
/// The aircraft's actual magnetic heading (flight model, not a gauge) — what
/// "keep current heading" bugs. Steadier than the whiskey compass in a turn.
pub const MAG_PSI: &str = "sim/flightmodel/position/mag_psi";
pub const FLAP_RATIO: &str = "sim/cockpit2/controls/flap_ratio";

// --- Airframe state ---
pub const GEAR_HANDLE_DOWN: &str = "sim/cockpit2/controls/gear_handle_down";
pub const ON_GROUND_ANY: &str = "sim/flightmodel/failures/onground_any";
/// Groundspeed, METRES PER SECOND (a magnitude).
pub const GROUNDSPEED_MS: &str = "sim/flightmodel/position/groundspeed";
/// 1 while X-Plane considers the airframe crashed (the crash screen); a
/// flight reset drops it back to 0.
pub const HAS_CRASHED: &str = "sim/flightmodel2/misc/has_crashed";
/// 1 while the sim is playing a REPLAY: every flight-model read is a recording,
/// not the aircraft flying now, so nothing may react to it — no ATC watches, no
/// callouts, no crash record, no control loop writes. A replayed crash landing
/// earned "you landed on runway two seven without clearance" (2026-07-14).
pub const IS_IN_REPLAY: &str = "sim/time/is_in_replay";
/// 1 while the sim is PAUSED: the flight is real (unlike a replay) but frozen —
/// control loops must not integrate against it (a throttle governor winds to
/// its clamp against a frozen airspeed and surges on unpause), and the crew /
/// world watches have nothing to react to. See `xplane::sim_frozen`.
pub const PAUSED: &str = "sim/time/paused";
/// Seconds the sim has run (it stops with a pause): our own traffic's
/// clock (`traffic::SimClock`), so the fleet moves on the sim's time.
pub const TOTAL_RUNNING_TIME_SEC: &str = "sim/time/total_running_time_sec";
/// Height above the terrain directly below, in METERS (× 3.28084 for feet).
pub const AGL_METERS: &str = "sim/flightmodel/position/y_agl";
pub const LATITUDE: &str = "sim/flightmodel/position/latitude";
pub const LONGITUDE: &str = "sim/flightmodel/position/longitude";
/// True altitude MSL, METERS (× 3.28084 for feet) — for the VHF range check,
/// which wants height above a station's field elevation, not the baro gauge.
pub const ELEVATION_M: &str = "sim/flightmodel/position/elevation";
/// Total fuel mass. X-Plane reports this in kilograms regardless of airframe.
pub const FUEL_TOTAL_KG: &str = "sim/flightmodel/weight/m_fuel_total";
/// The payload the pilot loaded in Weight & Balance (passengers, bags, cargo),
/// kg — the sim's side of the plan's load sheet.
pub const PAYLOAD_KG: &str = "sim/flightmodel/weight/m_fixed";
/// All-up mass now (empty + payload + fuel), kg — what a max takeoff weight
/// is compared against.
pub const TOTAL_WEIGHT_KG: &str = "sim/flightmodel/weight/m_total";
/// Per-tank fuel mass (kg), one element per tank. For a 2-tank single it's
/// `[left, right]`. Read as an array (not cached); used for the balance callout.
pub const FUEL_PER_TANK_KG: &str = "sim/flightmodel/weight/m_fuel";
/// Per-tank boost pump switch, one element per tank (0/1). The stock 737's
/// and the MD-82's cockpit switch itself; the A330's is an OUTPUT of its own
/// buttons. Written only by the crew's `fuel_pumps` tool.
pub const FUEL_TANK_PUMP_ON: &str = "sim/cockpit2/fuel/fuel_tank_pump_on";
/// Each tank's lateral position, m, one element per tank: zero is a
/// centreline tank.
pub const FUEL_TANK_LATERAL_M: &str = "sim/aircraft/overflow/acf_tank_X";
/// Per-engine fuel flow, kg/sec, one element per engine. Summed × 3600 gives the
/// live burn rate (kg/hr) for an endurance answer; ~0 with engines off.
pub const FUEL_FLOW_KG_SEC: &str = "sim/cockpit2/engine/indicators/fuel_flow_kg_sec";

// --- Weather (XP12 aircraft-relative) ---
/// Wind direction at the aircraft, degrees TRUE. (The `cockpit2/gauges` wind
/// datarefs don't exist in XP12 — these aircraft-weather ones are the live values.)
pub const WIND_DIR_DEG_TRUE: &str = "sim/weather/aircraft/wind_now_direction_degt";
/// Wind speed at the aircraft, METERS PER SECOND (× 1.943844 for knots).
pub const WIND_SPEED_MSC: &str = "sim/weather/aircraft/wind_now_speed_msc";
pub const OAT_DEG_C: &str = "sim/cockpit2/temperature/outside_air_temp_degc";
/// The REGION's wind profile: thirteen layers, altitude MSL in meters with a
/// matching speed and direction per index. The `aircraft/*` pair above is the
/// wind we are IN; these are the winds we could be in at another level, which
/// is what a cruise-level choice is actually about.
pub const WINDS_ALOFT_ALT_M: &str = "sim/weather/region/wind_altitude_msl_m";
/// Layer wind speeds, METERS PER SECOND (× 1.943844 for knots).
pub const WINDS_ALOFT_SPEED_MSC: &str = "sim/weather/region/wind_speed_msc";
/// Layer wind directions, degrees TRUE (the direction the wind comes FROM).
pub const WINDS_ALOFT_DIR_DEG_TRUE: &str = "sim/weather/region/wind_direction_degt";
pub const VISIBILITY_SM: &str = "sim/weather/aircraft/visibility_reported_sm";
/// Sea-level pressure in Pascals (1 inHg = 3386.389 Pa).
pub const QNH_PASCALS: &str = "sim/weather/aircraft/qnh_pas";

// --- Time ---
/// Sim UTC, seconds since midnight zulu.
pub const ZULU_TIME_SEC: &str = "sim/time/zulu_time_sec";
/// The sim's date, days since the first of January (0 based): the traffic's
/// timetable day (`traffic::rotation::Date`, 2026-10-13).
pub const LOCAL_DATE_DAYS: &str = "sim/time/local_date_days";

// --- Radios ---
/// COM1 active frequency. Despite the `hz` in the name, the value is in kHz
/// (118325 = 118.325 MHz); supports 8.33 kHz channel spacing.
pub const COM1_FREQ_KHZ: &str = "sim/cockpit2/radios/actuators/com1_frequency_hz_833";
/// Transponder code as squawked (e.g. 4521) — read by the radar watch to
/// verify an assigned code is actually set (the write side stays in the
/// transponder tool, per the tool-surface rule).
pub const TRANSPONDER_CODE: &str = "sim/cockpit2/radios/actuators/transponder_code";
/// Transponder mode: 0 off, 1 standby, 2 on, 3 alt. Replying means >= 2.
pub const TRANSPONDER_MODE: &str = "sim/cockpit2/radios/actuators/transponder_mode";
// --- TCAS targets (XP 11.50+ TCAS system; LiveTraffic/AI populate these) ---
/// Absolute target positions, one element per target (slot 0 = own ship).
pub const TCAS_LAT: &str = "sim/cockpit2/tcas/targets/position/lat";
pub const TCAS_LON: &str = "sim/cockpit2/tcas/targets/position/lon";
/// Target elevation, METERS MSL.
pub const TCAS_ELE_M: &str = "sim/cockpit2/tcas/targets/position/ele";
/// Weight on wheels per target (nonzero = on the ground).
pub const TCAS_WOW: &str = "sim/cockpit2/tcas/targets/position/weight_on_wheels";
/// Flight IDs, 8 NUL-padded bytes per target — read via `get_bytes` so slot
/// alignment survives.
pub const TCAS_FLIGHT_ID: &str = "sim/cockpit2/tcas/targets/flight_id";
/// Per-target vertical speed, ft/min — the one signal that tells an arrival
/// descending the final apart from a departure climbing through the same
/// corridor. Optional: a feed that doesn't populate it reads as level.
pub const TCAS_VS_FPM: &str = "sim/cockpit2/tcas/targets/position/vertical_speed";

// --- The pitch channel, as it reaches the tail (#181) ---
/// The autopilot's pitch servo output, -1..1.
pub const SERVO_PITCH_RATIO: &str = "sim/joystick/servo_pitch_ratio";
/// The column the flight model gets: yoke, servo and stability summed.
pub const TOTAL_PITCH_RATIO: &str = "sim/cockpit2/controls/total_pitch_ratio";
/// ...and in roll and yaw: the ailerons and the rudder the flight model gets.
pub const TOTAL_ROLL_RATIO: &str = "sim/cockpit2/controls/total_roll_ratio";
pub const TOTAL_HEADING_RATIO: &str = "sim/cockpit2/controls/total_heading_ratio";
/// The sideslip, degrees (the nose relative to the flown path).
pub const SIDESLIP_DEG: &str = "sim/flightmodel/position/beta";
/// The elevator SURFACE per wing, degrees, trailing edge down positive
/// (reduced by [`elevator_surface_deg`]).
pub const ELEVATOR1_DEG: &str = "sim/flightmodel2/wing/elevator1_deg";
pub const ELEVATOR_TRIM: &str = "sim/cockpit2/controls/elevator_trim";
/// The autopilot's VS dial as the autopilot reads it.
pub const VVI_DIAL_FPM: &str = "sim/cockpit2/autopilot/vvi_dial_fpm";
/// The altitude the autopilot's hold is holding.
pub const ALTITUDE_HOLD_FT: &str = "sim/cockpit2/autopilot/altitude_hold_ft";
pub const OVERRIDE_JOYSTICK_PITCH: &str = "sim/operation/override/override_joystick_pitch";
pub const OVERRIDE_FD_PITCH: &str = "sim/operation/override/override_flightdir_ptch";

/// The elevator's deflection from the per-wing array: the element that
/// moved furthest (the wings without an elevator read 0). `None` for an
/// empty or non-finite read.
pub fn elevator_surface_deg(per_wing: &[f64]) -> Option<f64> {
    per_wing
        .iter()
        .copied()
        .filter(|v| v.is_finite())
        .max_by(|a, b| a.abs().total_cmp(&b.abs()))
}

/// Audio panel: COM1 monitor selected (nonzero = COM1 is audible in the
/// cockpit). The ATC broadcast respects this so it can be muted like a radio.
pub const AUDIO_SELECTION_COM1: &str = "sim/cockpit2/radios/actuators/audio_selection_com1";

// --- Identity ---
/// Tail number string ("N172SP") — the spoken callsign.
pub const TAILNUM: &str = "sim/aircraft/view/acf_tailnum";
/// The loaded model's .acf, relative to the install
/// ("Aircraft/Laminar Research/Boeing 737-800/b738.acf"): with the ICAO, the
/// key aircraft detection resolves a profile from.
pub const ACF_RELATIVE_PATH: &str = "sim/aircraft/view/acf_relative_path";
/// The .acf's `acf/_author` string, a second route to the model.
pub const ACF_AUTHOR: &str = "sim/aircraft/view/acf_author";
/// The .acf's `acf/_descrip` string.
pub const ACF_DESCRIP: &str = "sim/aircraft/view/acf_descrip";
/// The loaded aircraft's ICAO type ("B738"): a SimBrief plan's callsign is
/// ours only on an airframe of the plan's type.
pub const ACF_ICAO: &str = "sim/aircraft/view/acf_ICAO";
/// Maximum gross mass, kg. The wake turbulence category is read off it.
pub const ACF_M_MAX_KG: &str = "sim/aircraft/weight/acf_m_max";

// --- Flight model, engines, inputs: read by several loops ---
/// True heading of the nose, degrees.
pub const TRUE_HEADING_DEG: &str = "sim/flightmodel/position/psi";
/// The flight path's true track, degrees (meaningless when stopped).
pub const TRACK_DEG_TRUE: &str = "sim/flightmodel/position/hpath";
/// Magnetic variation here, degrees (magnetic = true - variation).
pub const MAGNETIC_VARIATION_DEG: &str = "sim/flightmodel/position/magnetic_variation";
/// The flight model's indicated airspeed, knots (no gauge lag).
pub const INDICATED_AIRSPEED_KTS: &str = "sim/flightmodel/position/indicated_airspeed";
/// Flight-model bank and pitch, degrees (not the `true_` attitude above).
pub const PHI_DEG: &str = "sim/flightmodel/position/phi";
pub const THETA_DEG: &str = "sim/flightmodel/position/theta";
/// Angle of attack, degrees: the nose's attitude less the flight path.
pub const ALPHA_DEG: &str = "sim/flightmodel/position/alpha";
/// Body pitch rate, degrees a second.
pub const PITCH_RATE_DEG_S: &str = "sim/flightmodel/position/Q";
/// Vertical velocity in the local frame, metres a second, positive up: the
/// physical rate, not an instrument (the VSI trails it by about a second).
pub const LOCAL_VY_MS: &str = "sim/flightmodel/position/local_vy";
/// True airspeed, metres a second.
pub const TRUE_AIRSPEED_MS: &str = "sim/flightmodel/position/true_airspeed";
/// Body yaw rate, degrees per second.
pub const YAW_RATE_DEG_S: &str = "sim/flightmodel/position/R";
/// Velocity in the local frame, m/s: east (x) and south (z).
pub const LOCAL_VX_MS: &str = "sim/flightmodel/position/local_vx";
pub const LOCAL_VZ_MS: &str = "sim/flightmodel/position/local_vz";
/// Per engine, 1 while running.
pub const ENGINE_RUNNING: &str = "sim/flightmodel/engine/ENGN_running";
/// Per engine, the FADEC's lever mode: 0 outside a detent, 1 CL, 2 FLX/MCT,
/// 3 TOGA (a lever-driven A/THR, the A330).
pub const FADEC_MODE: &str = "sim/flightmodel/engine/ENGN_fadec_pow_req";
pub const N1_PERCENT: &str = "sim/cockpit2/engine/indicators/N1_percent";
pub const N2_PERCENT: &str = "sim/cockpit2/engine/indicators/N2_percent";
pub const PROP_SPEED_RPM: &str = "sim/cockpit2/engine/indicators/prop_speed_rpm";
/// Per engine, 1 while its starter is turning it.
pub const STARTER_IS_RUNNING: &str = "sim/flightmodel2/engines/starter_is_running";
/// The APU's N1, percent.
pub const APU_N1_PERCENT: &str = "sim/cockpit2/electrical/APU_N1_percent";
/// The APU bleed valve, 1 open.
pub const APU_BLEED: &str = "sim/cockpit2/bleedair/actuators/apu_bleed";
/// Per bus, volts.
pub const BUS_VOLTS: &str = "sim/cockpit2/electrical/bus_volts";
pub const STALL_WARNING_RATIO: &str = "sim/cockpit2/annunciators/stall_warning_ratio";
/// 1 while a tug is on the bar.
pub const PUSHBACK_ATTACHED: &str = "sim/aircraft/overflow/pushback_attached";
/// The GPS's active waypoint ident.
pub const GPS_NAV_ID: &str = "sim/cockpit2/radios/indicators/gps_nav_id";
/// The GPS cross-track error from the FMS's active leg, nm, signed: what
/// LNAV captures to 0.0 (#227).
pub const GPS_XTK: &str = "sim/cockpit2/radios/indicators/gps_xtk";
pub const SUN_PITCH_DEG: &str = "sim/graphics/scenery/sun_pitch_degrees";
/// The pilot's yoke, -1..1: hands on the controls.
pub const YOKE_PITCH_RATIO: &str = "sim/joystick/yoke_pitch_ratio";
pub const YOKE_ROLL_RATIO: &str = "sim/joystick/yoke_roll_ratio";

// --- Autopilot status (read-only; > 0 = engaged/armed) ---
pub const SERVOS_ON: &str = "sim/cockpit2/autopilot/servos_on";
pub const APPROACH_STATUS: &str = "sim/cockpit2/autopilot/approach_status";
pub const NAV_STATUS: &str = "sim/cockpit2/autopilot/nav_status";
pub const HEADING_STATUS: &str = "sim/cockpit2/autopilot/heading_status";

#[cfg(test)]
mod tests {
    /// The surface is the wing that moved furthest, its sign kept; a wing
    /// without an elevator reads 0 and a failed element is no reading.
    #[test]
    fn the_elevator_is_the_wing_that_moved() {
        use super::elevator_surface_deg as e;
        assert_eq!(e(&[0.0, 0.0, -4.5, -4.5, 0.0]), Some(-4.5));
        assert_eq!(e(&[0.0, 2.0, f64::NAN]), Some(2.0));
        assert_eq!(e(&[]), None);
        assert_eq!(e(&[f64::NAN]), None);
    }
}
