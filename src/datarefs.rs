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
pub const FLAP_RATIO: &str = "sim/cockpit2/controls/flap_ratio";

// --- Airframe state ---
pub const GEAR_HANDLE_DOWN: &str = "sim/cockpit2/controls/gear_handle_down";
pub const ON_GROUND_ANY: &str = "sim/flightmodel/failures/onground_any";
/// Height above the terrain directly below, in METERS (× 3.28084 for feet).
pub const AGL_METERS: &str = "sim/flightmodel/position/y_agl";
/// Total fuel mass. X-Plane reports this in kilograms regardless of airframe.
pub const FUEL_TOTAL_KG: &str = "sim/flightmodel/weight/m_fuel_total";
/// Per-tank fuel mass (kg), one element per tank. For a 2-tank single it's
/// `[left, right]`. Read as an array (not cached); used for the balance callout.
pub const FUEL_PER_TANK_KG: &str = "sim/flightmodel/weight/m_fuel";

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

// --- Autopilot status (read-only; > 0 = engaged/armed) ---
pub const SERVOS_ON: &str = "sim/cockpit2/autopilot/servos_on";
pub const APPROACH_STATUS: &str = "sim/cockpit2/autopilot/approach_status";
pub const NAV_STATUS: &str = "sim/cockpit2/autopilot/nav_status";
pub const HEADING_STATUS: &str = "sim/cockpit2/autopilot/heading_status";
