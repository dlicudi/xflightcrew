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
/// confirmed: the flare's free tracker ([`FlarePitch`] outside the
/// ratchet) at two thirds of its default gain and a wider pull. Not the
/// airframe's flare numbers: those are sized for the last forty feet (the
/// 737's is pull-only to 0.45), and the recorded new-flight start at LEMG 13
/// needs about 0.7 of standing pull to hold level (fitted to 1790587073:
/// -900 fpm at the start plus 0.54 of nose-down mistrim through the 737's
/// flare plant, 112 fpm rms). Swept against that plant and the LEMG 12 and
/// fastest flare plants at the loop's 0.6 s tick: the least height lost
/// with the least ringing.
pub fn engage_hold_gains() -> FlareGains {
    FlareGains {
        p: 0.04,
        d: 0.05,
        trim_rate: 0.015,
        max: 0.8,
        min: -0.2,
        balloon_ease_per_s: None,
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
// The flare by attitude, judged by height (docs/tasks/autonomy-layers).
pub(crate) mod attitude_flare;
// The approach's power on total energy (docs/tasks/autonomy-layers).
pub(crate) mod energy_throttle;
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

#[cfg(test)]
mod lxgb09_landing_call_tests;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod glidepath_gain_tests;

#[cfg(test)]
mod approach_power_floor_tests;

/// The 737's flare against a plant FITTED to its own flares: the seven LEMG
/// landings of 2026-09-26 (six on 13, then 12 with the first model law),
/// the elevator written against the sink that followed, 35 fpm rms over
/// all seven arrests from the gate to the wheels. A standing pull holds a
/// standing sink change of 4,970 fpm per unit through lags of 0.84 and
/// 3.09 s after 1.35 s of dead time; the aircraft pitches down by itself
/// over the last 43 ft (405 fpm at the wheels, the tail's ground effect);
/// the sink follows the speed at 63 fpm a knot, and the speed follows the
/// lever, the pull and the sink (-1.08 + 2.99 lever - 2.05 pull - 0.40 per
/// 1,000 fpm of sink, kt/s, fitted to the same ticks). Fitted flare by
/// flare the gain ran 4,020 to 6,430 and the dead time 1.16 to 2.17 s, so
/// every landing is flown against every flare's own plant. The first fit
/// (2,600 fpm, two 1 s lags, 1.8 s, no ground) landed every gate soft and
/// was wrong: LEMG 12 flew its law to 407.
#[cfg(test)]
mod lemg_737_flare_tests;

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
/// to the recorded windows, its whole-degree window, and the final flown on
/// the pursuit (weaving) and on its declared track law (`final_track:`).
#[cfg(test)]
mod lemg30_zibo_final_tests;

/// Issue #77: EDLP 24, the SF50's balked balloon of 2026-09-24, replayed.
#[cfg(test)]
mod edlp_go_around_tests;

/// Issue #163: LEMG 12, the SF50 (sortie 1790611629, 2026-09-28), in rough
/// air (the final's stray 42 fpm rms). The arrest took the servos at 51 ft,
/// its roundout pulled 0.19 into a +263 fpm balloon and pushed -0.05 through
/// it, handed the servos back at 50.5 ft (the band's top, 50 ft, from the
/// radio height alone), the autopilot flew its -445 fpm dial for 1.2 s with
/// the lever walked 0.16 to 0.27, the arrest took them again at 48.4 ft with
/// the sink rebuilding, and the wheels met the runway at -814 fpm.
///
/// The plant is the SF50's elevator with the servos out, in the form of
/// [`FlareModel`]: a standing change of sink per unit of pull through two
/// lags after a dead time, fitted to the three clean SF50 arrests on record
/// (ETND 26 2026-09-23, EDLP 24's balked balloon 2026-09-24, and this
/// roundout), each flare's own plant and the one fitted to all three. On
/// top of it, what the model does not carry: the speed bleeds 1.71 kt/s
/// less 1.29 kt/s per 1,000 fpm of climb (fitted to the same four SF50
/// arrests' ticks), under the 67 kt Vso the wing gives the sink 800 fpm a
/// second (the sink's own answer to the speed is inside the fit: a fit
/// with 63 fpm a knot on top chose none), and rough air is a random
/// vertical gust (an Ornstein-Uhlenbeck sink, 1.5 s). The plugin's pitch
/// channel and the loop's tick (0.35 s) are inside the dead time.
#[cfg(test)]
mod lemg12_sf50_balloon_tests;

/// Issue #166: LEMG 13, the SR22 (sortie 1790619465-approach, 2026-09-28),
/// 4.7 kt of wind from 145, the final's air 20 fpm rms (smooth). The arrest
/// took the servos at 50.6 ft sinking 476 fpm and flew the 172's tracker on
/// this elevator: 0.14 of pull had the sink at +10 by 45 ft, the roundout
/// then PUSHED 0.10 and the sink rebuilt to -471 at 28 ft; again to -52;
/// the flare's full 0.35 from 22 ft met the step to idle a dead time later
/// (-528 at 12.5 ft), ballooned to +298 at 22 ft, was let go to neutral in
/// one tick, the balloon's thrust came back and went again, and the sink
/// the 0.35 met too late was -585 fpm at 1.4 ft (575 on the scorecard).
///
/// The plant is the SR22's elevator with the servos out in the form of
/// [`FlareModel`] (a standing change of sink per unit of pull through two
/// lags after a dead time), plus what the model does not carry, fitted to
/// the same 38 ticks: the thrust (fpm per unit of lever through a lag) and
/// the speed (fpm of sink per knot lost from the gate), the speed itself
/// -2.70 kt/s less 5.1 per 1,000 fpm of climb plus 2.74 per unit of lever
/// (0.32 kt/s rms). Each plant is a fit: the whole record, and the record's
/// parts (the first arrest, the arrest to the flare, the flare), so a law
/// that lands on all of them does not depend on one fit being right. Rough
/// air is a random vertical gust (an Ornstein-Uhlenbeck sink, 1.5 s). The
/// plugin's pitch channel and the loop's tick (0.35 s) are inside the dead
/// time.
///
/// What the plants say: at idle with full flap this wing bleeds 2.7 kt a
/// second and sinks about 115 fpm harder for each knot, so a flare that cuts
/// the lever at its gate has lost the lift it is flaring with before the
/// column's dead time is out. The profile declares the model (6,000 fpm per
/// unit through 1.0 s lags after 0.8 s: under every fit's gain but the
/// arrest's, so it pulls for the weakest; the fitted 10,000 / 1.45 / 0.9
/// lands the same gates at 291) and a retard held until a second from the
/// wheels (`flare_power` `retard_at_s`).
#[cfg(test)]
mod lemg13_sr22_flare_tests;

/// Issue #166, the BOUNCE: LEMG 13, the SR22 on the iMac (a 0.4 to 0.46 s
/// loop), three 5 nm finals at the POH's 80 kt on 2026-09-29 (sorties
/// 1790701546, 1790702352, 1790702623), the final's air 42 to 45 fpm rms.
/// Each flew the jets' firm-touchdown flare to the wheels: the target a sink
/// of 107 fpm at the runway, the lever held at approach power to a second
/// from the wheels, the IAS 80 to 81 all the way down, first contact at 207
/// to 215 fpm with 0.10 of pull on the column. Each then flew: the ratchet
/// held the 0.10 as it came back up, the balloon ease let it go to neutral,
/// the balloon's approach thrust (0.265) came back at 3 to 4 ft, and the
/// wheels met again with the pull the flare had rebuilt on the way down:
/// 4.5 to 4.9 ft, then 4.9 to 6.0, then 9.9 ft after the rollout latched on
/// a 0.115 pull at 75 kt. At neutral and idle the hops went on to 55 to 60 kt.
///
/// The plant: the standing-sink elevator of [`FlareModel`] refitted to the
/// three 80 kt flares, carried to the ground by the vertical speed through a
/// lag of its own (the lift the pull adds is an ACCELERATION: a hop needs an
/// aircraft that can move faster than its standing sink), the tail's ground
/// effect sinking it from 40 ft, the gear a soft spring and damper, and the
/// mains' contact pitching the nose up (a climb tendency in proportion to the
/// sink met up to 369 fpm, scaled by the dynamic pressure, decaying; unbound
/// it ran away), the gear bottoming at 0.4 ft. Fitted open loop, the column
/// and lever as written, to the three flares' sink before contact and their
/// first six hops after it. A fit is only as good as the model it fits: the
/// elevator's authority on the wheels and the pitch the gear makes are not
/// measured (no attitude in the autoland ticks), only their sum.
///
/// What it says: the flare as flown bounces 3.8 to 5.9 ft in every plant,
/// as it did in the sim. The hold-off (`hold_off_ft` 2 ft, `touchdown_fpm`
/// 50, `tau_s` 7, the speed's bleed led (`fpm_per_kt`), the lever to idle
/// from 20 ft, `TouchHold` from the first contact) touches once at 124 fpm
/// or less at 64 to 69 kt, the hop at most 1.2 ft. The sim decides: VERIFY
/// on the next SR22 landings `touch_hold` (held pull, at, ias), the ticks'
/// `touch_hold` and `flare_ask` below 20 ft, the `power` events' `retard`
/// (float or timed from 20 ft, then `touched`), `bounce` events (none, or one
/// spell under 1.5 ft), the landing's `sink_fpm` and `bounce_ft`.
#[cfg(test)]
mod lemg13_sr22_bounce_tests;

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

/// Issue #180: the A330 at LEMG 30, 10 nm final (sortie
/// 1790634412-approach, 2026-09-28), first approach in rough air (the
/// final's air 119 fpm rms). The arrest took the servos at 86.8 ft and flew
/// the default tracker (the A330 declared no `flare_elevator:`): 0.35 of
/// pull held five seconds arrested -1,155 fpm into a +277 climb, the column
/// came back to neutral and the aircraft FLOATED at 30 to 48 ft for 17 s,
/// was committed at 46.4 ft with 1,432 m ahead and went around
/// `overflew_the_runway` 0.3 s later.
///
/// The plant is the A330's sidestick with the servos out: a RATE plant (the
/// pull asks a change of sink, through two lags after a dead time, and a
/// neutral stick holds the sink it has), the ground's own rate growing
/// below a height, and a sink that grows with the airspeed lost since the
/// gate (fpm per knot), the speed itself -1.0 kt/s less 1.3 per 1,000 fpm of
/// climb (the four arrests' idle bleed). Fitted per arrest to every tick of
/// the four recorded A330 arrests (LEMG 13, sortie 1790621184; LEMG 30,
/// sorties 1790633832 and 1790634412 twice): 6 to 21 fpm rms each, and the
/// four together. A standing-sink fit of the same ticks needs a 93 s lag,
/// which is the integrator. Rough air is a random vertical gust (an
/// Ornstein-Uhlenbeck sink, 1.5 s). The loop's tick is 0.2 s.
#[cfg(test)]
mod a330_lemg30_flare_tests;

/// LEMG 30, the Zibo 737's six own-flare arrests of 2026-10-07: its column
/// with the servos out fitted to them (a standing plant, the lever's pitch,
/// the ground and the speed), the tracker as flown replayed through it, and
/// the declared model flare from every gate on every fit.
#[cfg(test)]
mod zibo_lemg30_flare_tests;

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
