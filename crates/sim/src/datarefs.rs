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
pub const COMPASS_HEADING_DEG: &str = "sim/cockpit2/gauges/indicators/compass_heading_deg_mag";
/// The aircraft's actual magnetic heading (flight model, not a gauge) — what
/// "keep current heading" bugs. Steadier than the whiskey compass in a turn.
pub const MAG_PSI: &str = "sim/flightmodel/position/mag_psi";
pub const FLAP_RATIO: &str = "sim/cockpit2/controls/flap_ratio";

// --- Airframe state ---
pub const GEAR_HANDLE_DOWN: &str = "sim/cockpit2/controls/gear_handle_down";
pub const ON_GROUND_ANY: &str = "sim/flightmodel/failures/onground_any";
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
pub const VISIBILITY_SM: &str = "sim/weather/aircraft/visibility_reported_sm";
/// Sea-level pressure in Pascals (1 inHg = 3386.389 Pa).
pub const QNH_PASCALS: &str = "sim/weather/aircraft/qnh_pas";

// --- Time ---
/// Sim UTC, seconds since midnight zulu.
#[allow(dead_code)] // consumed by the atc binary's tree, not the copilot's (yet)
pub const ZULU_TIME_SEC: &str = "sim/time/zulu_time_sec";

// --- Radios ---
/// COM1 active frequency. Despite the `hz` in the name, the value is in kHz
/// (118325 = 118.325 MHz); supports 8.33 kHz channel spacing.
#[allow(dead_code)] // consumed by the atc binary's tree, not the copilot's (yet)
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

/// Audio panel: COM1 monitor selected (nonzero = COM1 is audible in the
/// cockpit). The ATC broadcast respects this so it can be muted like a radio.
#[allow(dead_code)] // consumed by the atc binary's tree, not the copilot's (yet)
pub const AUDIO_SELECTION_COM1: &str = "sim/cockpit2/radios/actuators/audio_selection_com1";

// --- Identity ---
/// Tail number string ("N172SP") — the spoken callsign.
#[allow(dead_code)] // consumed by the atc binary's tree, not the copilot's (yet)
pub const TAILNUM: &str = "sim/aircraft/view/acf_tailnum";

// --- Autopilot status (read-only; > 0 = engaged/armed) ---
pub const SERVOS_ON: &str = "sim/cockpit2/autopilot/servos_on";
pub const APPROACH_STATUS: &str = "sim/cockpit2/autopilot/approach_status";
pub const NAV_STATUS: &str = "sim/cockpit2/autopilot/nav_status";
pub const HEADING_STATUS: &str = "sim/cockpit2/autopilot/heading_status";
