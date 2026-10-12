//! Native autoland — **Stage 1: runway data**.
//!
//! Finds the nearest suitable landing runway, parsed straight from X-Plane's
//! `apt.dat` (Global Airports): type-`1`/`16`/`17` airport headers carry the ICAO,
//! type-`100` land runways carry both thresholds' lat/lon (fields 9-10 and 18-19).
//! One streaming pass, proximity-filtered. Each physical runway yields two landing
//! options (one per direction). Later stages plan the approach and fly it.

pub(crate) use crate::nav::{bearing_deg, haversine_nm, project_latlon};

/// The landing's cross-track, metres, per tick: this flight's only
/// (`Scoped`), written with the generation the run was launched in.
static TRACKING_ERROR: crate::scoped::Scoped<f64> = crate::scoped::Scoped::flight();
pub fn last_tracking_error() -> Option<f64> {
    if !ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
        return None;
    }
    TRACKING_ERROR.get()
}

pub(crate) const M_PER_NM: f64 = 1852.0;

// Runway data and geometry moved to `runway.rs` (phase 2): autoland's own
// code names it unqualified, nobody else reaches it through here.
use crate::runway::*;

/// The emergency self-pick as autoland arms it: the AIRFRAME's minimum length
/// and the surface wind from each field's METAR (never the aircraft's own
/// wind datarefs: at the arm they are the winds aloft). Blackboxed with the
/// numbers it chose on.
fn self_pick(apt_dat: &str, lat: f64, lon: f64, track_deg: f64) -> std::io::Result<Option<Runway>> {
    let min_length_m = self_pick_min_length_m(crate::aircraft::landing_distance_m());
    let root = crate::config::xplane_root();
    let wind_at = |icao: &str| crate::weather::surface_wind(&root, icao);
    let destination = crate::simbrief::loaded_plan().map(|p| p.destination);
    let picked = best_landing_runway_in_wind(
        apt_dat,
        lat,
        lon,
        track_deg,
        40.0,
        min_length_m,
        &wind_at,
        destination.as_deref(),
    )?;
    crate::blackbox::record(
        "autoland",
        serde_json::json!({
            "event": "self_pick", "min_length_m": min_length_m, "destination": destination,
            "rwy": picked.as_ref().map(|r| format!("{} {}", r.icao.trim(), r.id)),
            "length_m": picked.as_ref().map(|r| r.length_m),
            "wind": picked.as_ref().and_then(|r| wind_at(r.icao.trim())),
        }),
    );
    Ok(picked)
}

// ---- Stage 3: the control loop (AP-first, like the reference plugin) ----------

use crate::autoflight::{self, Button, Dial};
use std::sync::atomic::{AtomicBool, Ordering};

use crate::aircraft::throttle_dataref;
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

/// True while an autoland is flying (so cancel can acknowledge it). Written
/// only by the run ([`RUN`]).
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// The autoland's run. Every arm takes it over (a re-arm on the cleared
/// runway owns the aircraft from its first tick, the loop it replaces is
/// cancelled and exits on its own), and nothing a superseded loop does on
/// its way out touches the flag, the panel or the landing configuration of
/// the loop that replaced it. A cancel ("cancel autoland", "my controls", a
/// stand-down) is [`abort`]: it cancels the run flying, never a latch the
/// next launch inherits, so no caller clears anything before a launch, and
/// a launcher's own cancel reaches the autoland through `parents`.
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::flagged(&ACTIVE);
/// This approach's landing configuration (spoilers, autobrake) is set: the
/// `landing_config` event. Cleared when an approach starts and ends.
static LANDING_CONFIGURED: AtomicBool = AtomicBool::new(false);

/// Has the approach flying now set its landing configuration? The Monitor
/// BEFORE LANDING list waits for it under an automation
/// (`flow::landing_config_for`): a circuit hands to autoland on the final,
/// and the list audited the A330's spoilers and autobrake 48 s before
/// autoland set them (LEMG 30, sortie 1790633832-full_auto, #169).
pub fn landing_configured() -> bool {
    LANDING_CONFIGURED.load(Ordering::SeqCst)
}

/// The approach flying now is ESTABLISHED on its final: the crew's "final"
/// report is due ([`final_report_due`]). Written every tick by the run,
/// cleared when an approach starts and ends.
static ON_FINAL: AtomicBool = AtomicBool::new(false);

/// Is the approach flying now established on its final? Full auto's "final"
/// call waits for it (`full_auto` `final_call`).
pub fn established_on_final() -> bool {
    ON_FINAL.load(Ordering::SeqCst)
}

/// A crew reports final once ESTABLISHED on it: the guidance flying its
/// Final (inside the final's range, laterally stabilised), no reposition
/// pattern in hand, not going around, and the track lined up with the
/// runway. LEGR-LXGB, 2026-10-06 (sortie 1791275171-full_auto): "final
/// runway zero nine" went out the moment autoland took the arrival, 12 nm
/// EAST of the field at 13,300 ft and about to fly a reposition round the
/// Rock, and the tower cleared it to land on it. Pure.
pub fn final_report_due(
    phase_final: bool,
    pattern_flying: bool,
    going_around: bool,
    track_deg: f64,
    course_deg: f64,
) -> bool {
    phase_final
        && !pattern_flying
        && !going_around
        && crate::angles::abs_diff_deg(track_deg, course_deg) <= FINAL_REPORT_TRACK_DEG
}

/// Lined up for the report: the track within this of the runway course
/// (the final's own 35 degree cut is an intercept, not a final).
const FINAL_REPORT_TRACK_DEG: f64 = 30.0;

/// One-line emergency status for the panel's live strip ("AUTOLAND LELC 23R
/// 12.4nm FINAL"); empty when nothing is flying. Written by the autoland and
/// recover watchers, composed into the strip by the monitor each tick.
pub static PANEL: std::sync::Mutex<String> = std::sync::Mutex::new(String::new());

/// Set/clear the emergency line (empty = clear). Swallows a poisoned lock — the
/// strip is display-only.
pub fn set_panel(text: &str) {
    if let Ok(mut p) = PANEL.lock() {
        text.clone_into(&mut p);
    }
}

// State reads.
const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
#[cfg(test)]
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
/// Weight-on-wheels for the player aircraft. Radio altitude and airspeed are
/// not touchdown evidence: a normal flare can be below five feet and a knot
/// below target while the wheels are still airborne.
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
/// TRUE heading and TRUE ground track: their difference is the wind crab the
/// lateral law feeds forward on final (heading vs track, the reference recipe).
#[cfg(test)]
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
/// Yaw rate, deg/s nose right, for a declared rollout steer (`RolloutSteer`):
/// read through the state frame, `sim::state::F::R` (`position/R`).
// The autopilot panel is `crate::autoflight`'s: every button and dial below
// goes through it (the airframe's own buttons, the window rule, the thrust).
const CMD_YAW_DAMPER_OFF: &str = "sim/systems/yaw_damper_off";
/// The autopilot master as the sim has it (power and failures included).
const AP_MASTER_ON: &str = autoflight::names::AP_ON;
const YAW_DAMPER_ON: &str = "sim/cockpit2/switches/yaw_damper_on";
const SERVOS_ON: &str = autoflight::names::SERVOS_ON;
#[cfg(test)]
const CMD_SERVOS_FDIR_OFF: &str = autoflight::cmd::SERVOS_FDIR_OFF;
#[cfg(test)]
const HDG_BUG: &str = Dial::HeadingBug.read_name();
#[cfg(test)]
const VS_DIAL: &str = Dial::Vs.read_name();
const ALT_IND_FT: &str = crate::datarefs::ALTITUDE_FT;
const FLAP_RATIO: &str = "sim/cockpit2/controls/flap_ratio";
// Rollout: nose steering + proportional wheel brakes through the plugin, the
// same channels the taxi governor drives.
const TAXI_STEER: &str = "xcopilot/taxi_steer";
const TAXI_BRAKE: &str = "xcopilot/taxi_brake";
use sim::state::F;
/// What a tick reads past the position (`sim::state`; each field names its
/// dataref): the constants above (`MAG_VAR`, `IAS`, `HPATH_TRUE`,
/// `crate::datarefs::ON_GROUND_ANY`, `PHI_TRUE`, `VVI_FPM`, `SERVOS_ON`, `HDG_STATUS`,
/// `VS_STATUS`, `HDG_BUG`) and the plugin's yields.
const AUTOLAND_WANT: &[F] = &[
    F::MagVar,
    F::Ias,
    F::Hpath,
    F::OnGround,
    F::Phi,
    F::Vvi,
    F::Servos,
    F::HdgStatus,
    F::VsStatus,
    F::Bug,
    F::PitchYield,
    F::RollYield,
    F::R,
];
/// ...and with the ILS tuned, its needles (`crate::ils`; the modes are the
/// bitfield's, `autoflight::Modes`, read beside the frame).
const AUTOLAND_WANT_ILS: &[F] = &[
    F::MagVar,
    F::Ias,
    F::Hpath,
    F::OnGround,
    F::Phi,
    F::Vvi,
    F::Servos,
    F::HdgStatus,
    F::VsStatus,
    F::Bug,
    F::PitchYield,
    F::RollYield,
    F::R,
    F::Hdef,
    F::Vdef,
    F::LocValid,
    F::GsValid,
];
/// A lever read this close to the last command took it (`lever_took`).
const LEVER_TOOK_TOL: f64 = 0.01;
/// The gentler brake hold once stopped.
const ROLLOUT_BRAKE_HOLD: f64 = 0.5;

/// The line at a full stop, from the PARKING brake's read-back: "Brakes on"
/// only when it reads set. A brake that did not take is named, because the
/// rollout's squeeze expires within a second of the hand-off and the aircraft
/// is then the pilot's to hold (EGSS 2026-09-22: said on, read 0.03).
pub(crate) fn stopped_line(parking_brake_held: bool) -> &'static str {
    if parking_brake_held {
        "Down and stopped. Brakes on, your aircraft."
    } else {
        "Down and stopped. The parking brake did not set, hold the brakes. Your aircraft."
    }
}
/// Throttle a give-up hands back with: enough to hold level flight with gear
/// down and landing flap out, which is the state a balked landing leaves the
/// aircraft in. Half throttle was not (110 → 80 kt, EGHI 2026-08-02).
const GIVE_UP_THROTTLE: f64 = 0.75;
/// The VS watchdog: ticks of >700 fpm divergence before the warning (about
/// five seconds) and before the give-up (about a minute — long enough for a
/// re-engaged autopilot to catch the dial, short enough that a descent that
/// is really a climb does not become a cruise).
///
/// In SECONDS since 2026-09-30: they were 12 and 140 ticks, which is 4 s
/// and 46 s at the MacBook's 0.33 s Final tick the watchdog was tuned on
/// (6.7 s and 78 s at the iMac's 0.56 s).
const VS_DIVERGED_WARN_S: f64 = 4.0;
/// How far the vertical speed may sit from the command before the watchdog
/// looks at it, and how fast the gap must be CLOSING for the autopilot to
/// count as holding. The gap alone is a 172 number: the A330's VS mode takes
/// longer than the watchdog's five seconds to reach a -1,200 step from
/// level, and "not holding the descent" was said at the arm of an approach
/// the autopilot was flying correctly, just slowly (EGPF 23, 2026-09-20).
pub const VS_DIVERGED_FPM: f64 = 700.0;
/// Per SECOND since 2026-09-30: it was 15 fpm a tick, 45 fpm/s at the
/// MacBook's 0.33 s Final tick (27 fpm/s at the iMac's 0.56 s).
pub const VS_CONVERGING_FPM_PER_S: f64 = 45.0;

/// Is the autopilot NOT holding the commanded vertical speed? Far from it,
/// and not closing on it. `prev_err` is last tick's error (None = first
/// sight, which counts: there is no trend to excuse it yet).
/// `dt_s` is the time since that error was read.
pub fn vs_not_held(err_fpm: f64, prev_err_fpm: Option<f64>, dt_s: f64) -> bool {
    if err_fpm.abs() <= VS_DIVERGED_FPM {
        return false;
    }
    prev_err_fpm
        .is_none_or(|p| err_fpm.abs() > p.abs() - VS_CONVERGING_FPM_PER_S * dt_s.max(0.0))
}

/// The watchdog's words name what is actually commanded. "Not holding the
/// descent" was said four times in a missed approach climbing at 970 fpm.
pub fn vs_not_held_line(target_vs_fpm: f64) -> &'static str {
    if target_vs_fpm < -50.0 {
        "Autopilot is not holding the descent. Watch the pitch, be ready to take it."
    } else if target_vs_fpm > 50.0 {
        "Autopilot is not holding the climb. Watch the pitch, be ready to take it."
    } else {
        "Autopilot is not holding level. Watch the pitch, be ready to take it."
    }
}
const VS_DIVERGED_GIVE_UP_S: f64 = 46.0;

/// The confirmed engager lives in [`crate::ap_engage`] (refactor audit
/// 2026-10-04 #5: one engage for every loop); re-exported for the arm and
/// the tests here.
pub use crate::ap_engage::{ApEngage, EngageStep};
#[cfg(test)]
pub use crate::ap_engage::ENGAGE_GIVE_UP_S;

/// The elevator law that holds the aircraft LEVEL until the autopilot is
/// confirmed ([`LevelHold`]). The recorded new-flight start at LEMG 13
/// needs about 0.7 of standing pull to hold level (fitted to 1790587073:
/// -900 fpm at the start plus 0.54 of nose-down mistrim through the 737's
/// pitch plant, 112 fpm rms). Swept against that plant at the loop's 0.6 s
/// tick: the least height lost with the least ringing.
pub fn engage_hold_gains() -> LevelHoldGains {
    LevelHoldGains {
        p: 0.04,
        d: 0.05,
        trim_rate: 0.015,
        max: 0.8,
        min: -0.2,
    }
}

/// The words when the autopilot will not come in.
pub const ENGAGE_GIVE_UP_LINE: &str =
    "The autopilot will not engage. Abandoning the approach. Your aircraft.";
/// Past the arm distance by this much, still not on a missed approach, the
/// approach is abandoned: the aircraft is leaving, not arriving.
const RECEDING_GIVE_UP_NM: f64 = 5.0;
const M_TO_FT: f64 = 3.280_84;
const MS_TO_KTS: f64 = 1.943_84;

// The loop and its laws in their own files, whole and unchanged (refactor
// phase 4); every path stays `crate::autoland::...` through these re-exports.
mod rollout;
pub use rollout::*;
mod launch;
pub use launch::*;
mod run;
use run::*;
mod approach;
pub use approach::*;
mod flare;
// A visual final's VS set once and nudged (docs/tasks/autonomy-layers).
pub(crate) mod path_nudge;
pub use flare::*;
mod arrest;
pub use arrest::*;
mod missed;
pub use missed::*;
mod guidance;
pub use guidance::*;

use crate::reversal::TerrainHold;

#[cfg(test)]
mod terrain_hold_tests;

/// The final above minimums on the fitted heading modes (2026-10-12).
#[cfg(test)]
mod final_heading_tests;

#[cfg(test)]
mod lxgb09_landing_call_tests;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod glidepath_gain_tests;

#[cfg(test)]
mod approach_power_floor_tests;

/// The 737's flare retard keys (`flare_power`) on its own numbers, and the
/// heading knob its LEMG 13 finals of 2026-09-28 set.
#[cfg(test)]
mod lemg_737_retard_tests;

#[cfg(test)]
mod ap_engage_tests;

#[cfg(test)]
mod final_report_tests;

/// LEMG 13 and 31 as apt.dat has them, a 737's heading response and the
/// autoland loop's lateral composition, for the replays of the 2026-09-28
/// circuits (issue #147): the circuit's final, the handoff and autoland's
/// final flown against the same plant.
#[path = "autoland/lemg_replay_tests.rs"]
#[cfg(test)]
pub(crate) mod lemg_replay;

/// Issue #147: LEMG 13 and 31, the 737's full-auto circuits of 2026-09-28,
/// replayed against the real apt.dat geometry.
#[cfg(test)]
mod issue_147_tests;

/// LEMG 30, the Zibo 737's finals of 2026-10-07: its heading select fitted
/// to the recorded windows, its whole-degree window, and the final flown as
/// a pilot flies it (whole degrees above minimums, hand flown below).
#[cfg(test)]
mod lemg30_zibo_final_tests;

/// Issue #77: EDLP 24, the SF50's balked balloon of 2026-09-24, replayed.
#[cfg(test)]
mod edlp_go_around_tests;


/// Issue #166, the rollout: LEMG 13, the SR22 (sortie 1790619465-approach,
/// 2026-09-28), both brakes at 0.24 and the rudder alone steering. The
/// recorded heading IS the plant here: fitted, the steer explains none of it
/// (under 2 deg/s per unit), so what the SR22 did on the wheels is taken as
/// the disturbance and the brakes' yaw is added on top of it, a first-order
/// yaw rate (0.5 s) behind 0.3 s of the plugin's channel, at the loop's
/// 0.4 s tick. The brakes' authority is not in this record (nothing split
/// them); the SR22's taxi ticks give 40 to 75 deg/s per unit of difference
/// at taxi pace, with less weight on the wheels at 70 kt, so the law is
/// flown from 15 to 60.
#[cfg(test)]
mod lemg13_sr22_rollout_tests;

/// Issue #166 again, the split flown twice on the iMac (0.42 to 0.62 s
/// ticks), LEMG 13, the SR22, both rollouts latched on a BOUNCE.
///
/// 1790689843-approach (the shared split; 9.1 m): the latch took the first
/// contact at 88 kt and the aircraft flew 3.5 to 3.8 ft for about six seconds
/// after it (`state` ground false, the deceleration 2.5 kt/s, the flight
/// model's touchdown at 69 kt). 1790694457-approach (the split of aa5ab0f;
/// 9.4 m): three porpoises in the flare (radio 0.8, 0.4, 1.3 ft and back up
/// to 7.6, 11.4, 10.2), the third contact kicking the bank to 7 degrees and
/// the drift from 0 to 1.1 m/s right, then the latch at 61 kt and up to 9 ft
/// for four and a half seconds with 0.39 of left brake on, which slewed the
/// nose from 134 to 127 in a second as the wheels came down on a 131.5
/// track. The `state` samples ride about 2.5 s behind their stamp (their
/// groundspeed against the rollout ticks'); the wheels came down at about
/// 6.3 s and 4.4 s after the latch, where the deceleration jumps.
///
/// Fitted over the ground ticks of both (two-tick predictions of heading and
/// track): the wheel difference 45 deg/s per unit after 0.4 to 0.5 s, the
/// nose weathervaning onto the track at 2 to 6 per second at 60 kt (as the
/// square of the speed), the track following the nose at 2 to 3 per second.
/// The replay takes the recorded heading and cross-track as the plant under
/// the recorded split and adds, on the wheels, the response to the
/// DIFFERENCE a law writes through that plant; in the air (with the bounce
/// handled) no brake and the arrest's bank target flown through 0.3 s and a
/// 0.8 s roll lag, as a point mass from the recorded drift. The laws as
/// flown reproduce both rollouts.
#[cfg(test)]
mod lemg13_sr22_rollout_split_tests;

/// Issue #180: the A330 at LEMG 30 (sorties 1790621184, 1790633832 and
/// 1790634412, 2026-09-28): a commit that lasted one tick (its roll gates
/// fly the hand-flown bank in `pilot::plants_tests`).
#[cfg(test)]
mod a330_lemg30_arrest_tests;

/// LEMG 30, the Zibo 737's six own-flare arrests of 2026-10-07: the
/// go/no-go at each recorded roundout gate.
#[cfg(test)]
mod zibo_lemg30_landing_call_tests;

/// #199: the missed approach flies a speed.
#[cfg(test)]
mod missed_approach_speed_tests;

/// Issue #197, the rollout: LEMG 13, the King Air C90B (iMac 2026-09-30,
/// sorties 1790757702, 1790758429 and 1790758604 from 4 nm), steered by the
/// takeoff roll's default law through the rudder and nosewheel (7 degrees of
/// nosewheel at speed, 48 slow; `_gear_castors 0`, so not a brake-steer
/// airframe). Every contact yawed the nose 8 to 15 deg/s left with the steer
/// at zero; the default law answered with its whole 0.35 at 80 kt for a
/// second and a half, the nose swung 30 to 80 degrees right, and full left
/// steer held none of it until 45 kt: 1790757702 left the runway 176 m right,
/// 1790758604 125.8 m, 1790758429 swung 24 degrees and came back at 19.7 m.
///
/// The recorded heading and track ARE the plant here: three fits over the
/// first three and a half to four and a half seconds on the wheels, each
/// with the steer as written, the contact's own yaw as its first state, and a
/// yaw the slip between nose and track feeds (A, over 4.5 s: 62 deg/s per
/// unit at 80 kt, 1.5 per degree of slip, 0.41 s, 0.29 s dead; B, over 3.5
/// s: 43, 2.3, 0.08 s, 0.30 s; C, over 3.5 s with no slip term: a near
/// integrator of 219 through 3 s after 0.32 s; 1.2 to 2.4 degrees rms).
/// Below 40 kt the nosewheel's own steering: 12 to 40 deg/s per unit after
/// 0.3 to 0.5 s.
#[cfg(test)]
mod lemg13_king_air_rollout_tests;

/// Issue #197, the roundout: LEMG 13, the King Air C90B from 5 nm (iMac
/// 2026-09-30, thirteen autolands). The recorded final from 150 ft to 6 s
/// after the arrest took the servos (`bench/landings/
/// king_air_lemg13_roundouts.jsonl`: autoland ticks, `arrest_hands`, and
/// the `state` samples' groundspeed and pitch trim).
#[cfg(test)]
mod lemg13_king_air_roundout_tests;

/// The LEMG 13 737 final of 2026-10-05 through the energy plan.
#[cfg(test)]
mod lemg13_737_energy_tests;
