//! Native autoland — **Stage 1: runway data**.
//!
//! Finds the nearest suitable landing runway, parsed straight from X-Plane's
//! `apt.dat` (Global Airports): type-`1`/`16`/`17` airport headers carry the ICAO,
//! type-`100` land runways carry both thresholds' lat/lon (fields 9-10 and 18-19).
//! One streaming pass, proximity-filtered. Each physical runway yields two landing
//! options (one per direction). Later stages plan the approach and fly it.

pub(crate) use crate::nav::{bearing_deg, haversine_nm, project_latlon};

static TRACKING_ERROR: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);
pub fn last_tracking_error() -> Option<f64> {
    if !ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
        return None;
    }
    *TRACKING_ERROR.lock().unwrap_or_else(|e| e.into_inner())
}

pub(crate) const M_PER_NM: f64 = 1852.0;

// Runway data and geometry moved to `runway.rs` (phase 2); re-exported so
// every `autoland::` path keeps compiling.
pub use crate::runway::*;


/// The emergency self-pick as autoland arms it: the AIRFRAME's minimum length
/// and the surface wind from each field's METAR (never the aircraft's own
/// wind datarefs: at the arm they are the winds aloft). Blackboxed with the
/// numbers it chose on.
fn self_pick(apt_dat: &str, lat: f64, lon: f64, track_deg: f64) -> std::io::Result<Option<Runway>> {
    let min_length_m = self_pick_min_length_m(crate::aircraft::landing_distance_m());
    let root = crate::tools::autoland::xplane_root();
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
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}
const GEAR_HANDLE: &str = "sim/cockpit2/controls/gear_handle_down";
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

/// The deceleration a rollout aims for when the runway leaves the choice open,
/// m/s². A flat 0.45 brake ratio measured ~3.3 m/s² on the Baron — maximum
/// autobrake territory, applied from touchdown speed to a dead stop, and the
/// pilot called it "brakes really really hard" (EGHI, 2026-08-02). Real
/// autobrake LOW is ~1.2-1.8; this errs on the firm side of comfortable
/// because the distance term below, not this figure, is what guarantees the
/// stop.
const ROLLOUT_COMFORT_DECEL: f64 = 1.6;
/// Brake ratio per m/s² of wanted deceleration — calibrated from the same
/// measurement (0.45 ratio ≈ 3.3 m/s² including aero drag, so the wheel share
/// is a little under that).
const BRAKE_RATIO_PER_MPS2: f64 = 0.15;
/// The rollout hands off to a following vacate at or below this groundspeed —
/// taxi pace, never a stop. Stopping dead on an active runway and then
/// re-accelerating to leave it was neither procedure nor comfortable.
const VACATE_HANDOFF_KTS: f64 = 10.0;

/// Brake ratio for the rollout: a comfortable deceleration, unless stopping
/// within the pavement actually remaining demands more. `target_kts` is the
/// speed the rollout is aiming for — zero for a full stop, taxi pace when a
/// vacate follows.
fn rollout_brake_ratio(gs_kts: f64, target_kts: f64, remaining_m: f64) -> f64 {
    let v = (gs_kts.max(0.0)) / MS_TO_KTS;
    let vt = (target_kts.max(0.0)) / MS_TO_KTS;
    // What physics requires to be at target speed with 30% of the remaining
    // runway still in hand — the safety term, immune to the comfort figure.
    let usable = (remaining_m * 0.7).max(30.0);
    let required = ((v * v - vt * vt) / (2.0 * usable)).max(0.0);
    let decel = required.max(ROLLOUT_COMFORT_DECEL);
    (decel * BRAKE_RATIO_PER_MPS2).clamp(0.2, 0.8)
}

/// The rollout's differential brake on an airframe that steers on its brakes
/// (a castoring nosewheel, `taxi.brake_steer`), as the real aircraft is
/// steered down the runway: the stopping brake moved from the outside wheel
/// to the inside one, the total unchanged.
///
/// LEMG 13 (issue #166, 2026-09-28, the SR22, sortie 1790619465): the
/// rollout steered on the rudder alone with both brakes at 0.24. The nose
/// swung 6 degrees right in the first two seconds on the wheels, and for ten
/// seconds from 70 kt it sat 3 to 6 degrees right of the runway while the
/// steer asked for left, growing to 0.69 and the trim to its 0.3 cap: fitted
/// over those ticks the rudder's yaw is under 2 deg/s per unit of steer, no
/// authority at all, and the aircraft rolled from 1.6 m to 12.1 m right of
/// the centreline. The takeoff's split (`autotakeoff::roll_brake_split`)
/// fades out from 20 to 40 kt because a brake on the roll costs the
/// acceleration; on the rollout the brake is wanted anyway, so it steers at
/// every speed. The heading error is the takeoff roll's target less the
/// heading (`roll_target_heading`), damped on its own rate; the SR22's taxi
/// ticks give its brakes 40 to 75 deg/s of yaw per unit of difference at
/// taxi pace, and the gain holds the recorded rollout inside 2.6 m from 15 to
/// 60 deg/s per unit.
///
/// On a FREE castoring nose (`aircraft::nose_free`: the SR22, the Evolution,
/// the RV-10) the difference is the one ground law's (`ground_steer`): the
/// identified plant inverted over the tick, the rollout's rudder a known
/// input, the standing yaw (the idle propeller's) learnt by its observer.
/// It replaces the SR22's declared split (`rollout_brakes:`, a gain
/// schedule, lead and trim fitted to LEMG 13 1790689843 and 1790694457,
/// 2026-09-29) and the shared law the Evolution flew into 8.3 m at LILB 03
/// (1790762771, 2026-09-30: 0.14 of difference at 0.75 of stop, the nose
/// 3.7 degrees right of its target for six seconds from 66 kt).
#[derive(Debug, Default, Clone)]
pub struct RolloutBrakeSteer {
    prev_err: Option<f64>,
    rate: f64,
    /// The one ground law on a free castoring nose, with the last heading
    /// (its yaw rate is the heading differenced over the tick).
    free: Option<(crate::ground_steer::GroundSteer, Option<f64>)>,
    /// The free law's last tick, for the record.
    last: Option<crate::ground_steer::Steer>,
}

/// Brake difference per degree of heading error...
const ROLLOUT_BRAKE_STEER_PER_DEG: f64 = 0.03;
/// ...led by this many seconds of the error's rate...
const ROLLOUT_BRAKE_STEER_LEAD_S: f64 = 0.8;
/// ...never more than this difference between the wheels...
const ROLLOUT_BRAKE_STEER_MAX: f64 = 0.3;
/// ...and none below this groundspeed, where the stop owns the brakes.
const ROLLOUT_BRAKE_STEER_MIN_KTS: f64 = 4.0;
/// A free nose's most wheel difference on the rollout: an inside wheel at
/// full brake takes the rest off the outside (`ground_steer::stop_wheels`).
const ROLLOUT_FREE_DIFF_MAX: f64 = 0.5;

/// One tick of [`RolloutBrakeSteer`]: the brakes, as (the symmetric channel,
/// the left wheel, the right wheel). The plugin gives each wheel the harder
/// of the symmetric channel and its own, so the outside wheel's brake is the
/// symmetric channel and the inside one's its own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RolloutBrakes {
    pub sym: f64,
    pub left: f64,
    pub right: f64,
}

impl RolloutBrakeSteer {
    /// The law for the live airframe: the one ground law on a free
    /// castoring nose, else the shared split.
    pub fn for_airframe() -> Self {
        if crate::aircraft::nose_free() {
            Self::free(crate::ground_steer::GroundPlant::for_airframe())
        } else {
            Self::default()
        }
    }

    /// The one ground law on this plant.
    pub fn free(plant: crate::ground_steer::GroundPlant) -> Self {
        Self {
            free: Some((crate::ground_steer::GroundSteer::new(plant), None)),
            ..Self::default()
        }
    }

    /// The learnt standing yaw (the free law's, deg/s^2), for the record.
    pub fn trim(&self) -> f64 {
        self.free.as_ref().map_or(0.0, |(law, _)| law.w_hat())
    }

    /// The free law's last tick, for the record.
    pub fn last(&self) -> Option<crate::ground_steer::Steer> {
        self.last
    }

    /// The heading error (degrees, target right of the nose positive), the
    /// tick, groundspeed and the stopping brake the rollout wants.
    #[cfg(test)]
    pub fn step(&mut self, err_deg: f64, dt_s: f64, gs_kts: f64, brake: f64) -> RolloutBrakes {
        self.step_with(err_deg, None, None, 0.0, dt_s, gs_kts, brake)
    }

    /// [`Self::step`] with the heading and the track (true degrees: the free
    /// law's yaw rate and the slip its tail weathervanes on) and the rudder
    /// written this tick (echoed; the roll law's).
    #[allow(clippy::too_many_arguments)]
    pub fn step_with(
        &mut self,
        err_deg: f64,
        psi_deg: Option<f64>,
        track_deg: Option<f64>,
        rudder: f64,
        dt_s: f64,
        gs_kts: f64,
        brake: f64,
    ) -> RolloutBrakes {
        let brake = if brake.is_finite() {
            brake.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let plain = RolloutBrakes {
            sym: brake,
            left: 0.0,
            right: 0.0,
        };
        if !err_deg.is_finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            // A failed read is no measurement: the plain stop.
            return plain;
        }
        if let Some((law, prev_psi)) = self.free.as_mut() {
            let mean = match (psi_deg, *prev_psi) {
                (Some(p), Some(q)) => crate::angles::signed_diff_deg(p, q) / dt_s.clamp(0.05, 3.0),
                _ => 0.0,
            };
            *prev_psi = psi_deg;
            if !gs_kts.is_finite() || gs_kts < ROLLOUT_BRAKE_STEER_MIN_KTS {
                law.release();
                return plain;
            }
            let _ = track_deg;
            let s = law.step(crate::ground_steer::Demand {
                io_delay_s: crate::ground_steer::ROLLOUT_IO_DELAY_S,
                ..crate::ground_steer::heading_demand(
                    dt_s,
                    gs_kts / MS_TO_KTS,
                    crate::ground_steer::Yaw::Mean(mean),
                    err_deg,
                    crate::ground_steer::Rudder::Given(rudder),
                    ROLLOUT_FREE_DIFF_MAX,
                    crate::aircraft::live_profile()
                        .and_then(|p| p.taxi.as_ref().and_then(|t| t.max_yaw_dps))
                        .unwrap_or(crate::ground_steer::DEFAULT_MAX_YAW_DPS),
                )
            });
            self.last = Some(s);
            let (sym, left, right) = crate::ground_steer::stop_wheels(brake, s.diff);
            return RolloutBrakes { sym, left, right };
        }
        if let Some(p) = self.prev_err {
            self.rate = (err_deg - p) / dt_s.clamp(0.05, 3.0);
        }
        self.prev_err = Some(err_deg);
        if !gs_kts.is_finite() || gs_kts < ROLLOUT_BRAKE_STEER_MIN_KTS {
            return plain;
        }
        let diff = (ROLLOUT_BRAKE_STEER_PER_DEG * (err_deg + ROLLOUT_BRAKE_STEER_LEAD_S * self.rate))
            .clamp(-ROLLOUT_BRAKE_STEER_MAX, ROLLOUT_BRAKE_STEER_MAX);
        let half = diff.abs() / 2.0;
        let outside = (brake - half).max(0.0);
        let inside = (brake + half).min(1.0);
        if diff > 0.0 {
            // Nose right: the right wheel brakes harder.
            RolloutBrakes {
                sym: outside,
                left: 0.0,
                right: inside,
            }
        } else if diff < 0.0 {
            RolloutBrakes {
                sym: outside,
                left: inside,
                right: 0.0,
            }
        } else {
            plain
        }
    }
}

/// The rollout's steer on a STEERED nosewheel that declares its own law
/// (`rollout_steer:`, [`crate::aircraft::RolloutSteerProfile`], issue #197);
/// every other airframe keeps the takeoff roll's default law. Pure.
///
/// LEMG 13 (iMac 2026-09-30, the King Air C90B): every contact yawed the
/// nose 8 to 15 deg/s left with nothing steering, and the default law, its
/// whole authority 0.35 at 80 kt, answered on a heading DIFFERENCED over
/// a 0.45 s tick: 0.35 of right steer for a second and a half, the nose
/// swung 30 to 80 degrees right, and full left steer held none of it until
/// 45 kt (1790757702 off the runway, 176 m right). Fitted over the three
/// rollouts the steer is 45 to 65 deg/s per unit at 80 kt after 0.3 s and
/// the slip between the nose and the track feeds its own yaw: the swing is
/// seen first in the yaw RATE, so the law damps on the rate the sim
/// measures (`sim/flightmodel/position/R`), holds its gain low at speed and
/// raises it as `ref_kts / gs` toward the nosewheel's own steering.
#[derive(Debug, Clone, Copy)]
pub struct RolloutSteer {
    prof: crate::aircraft::RolloutSteerProfile,
    prev_err: Option<f64>,
    out: f64,
}

/// The rollout steer's slew, units per second (the takeoff roll's).
const ROLLOUT_STEER_SLEW_PER_S: f64 = 1.5;
/// The rollout steer's authority under `min_kts` (the takeoff roll's).
const ROLLOUT_STEER_MAX: f64 = 0.85;

impl RolloutSteer {
    pub fn new(prof: crate::aircraft::RolloutSteerProfile) -> Self {
        Self {
            prof,
            prev_err: None,
            out: 0.0,
        }
    }

    /// The gain per degree at this groundspeed.
    pub fn per_deg_at(&self, gs_kts: f64) -> f64 {
        let p = self.prof;
        let v = if gs_kts.is_finite() { gs_kts } else { p.ref_kts };
        p.per_deg * p.ref_kts / v.clamp(p.min_kts, p.ref_kts)
    }

    /// The most steer at this groundspeed: the full authority at `min_kts`
    /// and under, `max_fast` from `fast_kts`.
    pub fn cap_at(&self, gs_kts: f64) -> f64 {
        let p = self.prof;
        let v = if gs_kts.is_finite() { gs_kts } else { p.fast_kts };
        let share = ((v - p.min_kts) / (p.fast_kts - p.min_kts)).clamp(0.0, 1.0);
        ROLLOUT_STEER_MAX + (p.max_fast - ROLLOUT_STEER_MAX) * share
    }

    /// Off the wheels (a bounce): nothing is written, so the next contact
    /// slews from neutral.
    pub fn in_air(&mut self) {
        self.out = 0.0;
        self.prev_err = None;
    }

    /// The heading error (degrees, target right of the nose positive), the
    /// tick, groundspeed and the sim's yaw rate (deg/s, nose right
    /// positive; `None` on a failed read, when the error's own rate stands
    /// in).
    pub fn step(&mut self, err_deg: f64, dt_s: f64, gs_kts: f64, yaw_dps: Option<f64>) -> f64 {
        if !err_deg.is_finite() {
            return self.out;
        }
        let dt = if dt_s.is_finite() { dt_s.clamp(0.05, 1.0) } else { 0.4 };
        let differenced = self.prev_err.map_or(0.0, |p| -(err_deg - p) / dt);
        self.prev_err = Some(err_deg);
        let yaw = yaw_dps.filter(|y| y.is_finite()).unwrap_or(differenced);
        let cap = self.cap_at(gs_kts);
        let want =
            (self.per_deg_at(gs_kts) * (err_deg - self.prof.damp_s * yaw)).clamp(-cap, cap);
        let step = ROLLOUT_STEER_SLEW_PER_S * dt;
        self.out += (want - self.out).clamp(-step, step);
        self.out
    }
}

/// The rollout's view of the wheels, tick by tick: on them, off them after
/// the latch (a bounce, blackboxed `bounce` once per spell with its height),
/// or down again. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct BounceHold {
    in_air: bool,
    spells: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BounceTick {
    OnWheels,
    /// The first airborne tick of a spell.
    Left,
    InAir,
    /// The first tick back on the wheels after a spell.
    Landed,
}

impl BounceHold {
    pub fn tick(&mut self, airborne: bool) -> BounceTick {
        let t = match (self.in_air, airborne) {
            (false, false) => BounceTick::OnWheels,
            (false, true) => {
                self.spells += 1;
                BounceTick::Left
            }
            (true, true) => BounceTick::InAir,
            (true, false) => BounceTick::Landed,
        };
        self.in_air = airborne;
        t
    }

    /// Spells in the air since the latch.
    pub fn spells(&self) -> u32 {
        self.spells
    }
}

/// A light single HELD ON after its first contact (a hold-off flare,
/// `flare_elevator.model.hold_off_ft`, issue #166). From the first tick the
/// wheels read on, or the radio under [`TOUCH_RADIO_FT`], the column holds
/// the pull it touched with: never more (a pull added on the wheels lifts a
/// wing that still flies back off) and never less (no push, and no let-go:
/// the flare's balloon ease and the rollout's expiring channel took the
/// SR22's 0.10 to neutral within a second, and at 75 to 80 kt neutral
/// floated it 3 to 10 ft; LEMG 13 1790702352, 1790702623, 2026-09-29), with
/// the lever closed (never the balloon's approach thrust: each of those hops
/// had it back at 0.265). It lets go only on the wheels, slower than
/// [`TOUCH_HOLD_RELEASE_KTS`], easing to neutral at
/// [`TOUCH_HOLD_EASE_PER_S`]. A go-around forgets it. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct TouchHold {
    held: Option<f64>,
    released: bool,
}

/// Under this radio height a hold-off is on the runway, ft: at rest the
/// SR22's radio reads 0.1 to 0.2, and a contact between two 0.45 s ticks
/// was read at 0.3 ft on its way back up with no wheels flag (1790702352).
pub const TOUCH_RADIO_FT: f64 = 0.5;
/// The held pull goes on the wheels below this groundspeed, kt: the hops of
/// the four iMac SR22 rollouts died out between 54 and 60 kt.
pub const TOUCH_HOLD_RELEASE_KTS: f64 = 45.0;
/// ...eased to neutral this fast, elevator per second.
pub const TOUCH_HOLD_EASE_PER_S: f64 = 0.05;

impl TouchHold {
    /// This tick's contact (`on_wheels`, or the radio under
    /// [`TOUCH_RADIO_FT`]) with the elevator last written; true on the
    /// first one, which takes that pull (a push is taken as neutral).
    pub fn note(&mut self, contact: bool, written: Option<f64>) -> bool {
        if self.held.is_some() || !contact {
            return false;
        }
        self.held = Some(written.filter(|e| e.is_finite()).unwrap_or(0.0).max(0.0));
        true
    }

    /// The wheels have touched this approach.
    pub fn touched(&self) -> bool {
        self.held.is_some()
    }

    /// The pull held, for the blackbox.
    pub fn held(&self) -> Option<f64> {
        self.held
    }

    /// The column this tick: the pull held; on the wheels below the release
    /// speed eased toward neutral; `None` once let go (the channel expires
    /// by itself) or before any contact.
    pub fn elev(&mut self, gs_kts: f64, on_wheels: bool, dt: f64) -> Option<f64> {
        let h = self.held?;
        if self.released {
            return None;
        }
        if on_wheels && gs_kts.is_finite() && gs_kts < TOUCH_HOLD_RELEASE_KTS {
            let next = h - TOUCH_HOLD_EASE_PER_S * dt.clamp(0.0, 1.0);
            if next <= 0.0 {
                self.released = true;
                self.held = Some(0.0);
                return Some(0.0);
            }
            self.held = Some(next);
            return Some(next);
        }
        Some(h)
    }

    /// A go-around: the next approach starts free.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Whether the live airframe steers on its brakes (the takeoff's own test).
fn rollout_brake_steers() -> bool {
    crate::aircraft::nose_castors()
        || crate::aircraft::live_profile()
            .and_then(|p| p.taxi.as_ref().map(|t| t.brake_steer))
            .unwrap_or(false)
}

/// Set by full auto before its landing: a vacate drive follows this rollout,
/// so end it at taxi pace instead of a dead stop. Consumed (swapped false) by
/// the rollout that honours it and cleared on abort, so a standalone "land for
/// me" — where nobody follows — always brakes to a stop as before.
pub static VACATE_FOLLOWS: AtomicBool = AtomicBool::new(false);

/// The runway this autoland actually rolled out on, set at rollout entry.
///
/// Where we ARE, as against where a clearance says we should be — and for
/// vacating, being on the strip is the whole of the question. A landing
/// clearance can be missing for reasons that have nothing to do with the
/// pavement under the wheels: at Malaga the tower ordered a go-around, which
/// voided the clearance, the second approach was never re-cleared, and the
/// vacate — gated on a STANDING clearance — refused. The aircraft braked to
/// a stop on runway 13 and sat there through four incursion warnings with an
/// arrival closing to 1.3 nm (2026-08-05). Landing without a clearance makes
/// clearing the runway more urgent, not less.
static LANDED_RUNWAY: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

/// A new flight forgets the last one's landing: the runway it rolled out on
/// (a hand-landed 31 after a reposition onto the same strip vacated as the
/// old 13) and its tracking (`flight_state::forget_flight`).
pub(crate) fn forget_flight() {
    *LANDED_RUNWAY.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *TRACKING_ERROR.lock().unwrap_or_else(|e| e.into_inner()) = None;
    drop_handed_down();
}

/// The runway the last autoland rolled out on, if it has not been consumed.
pub fn landed_runway() -> Option<String> {
    LANDED_RUNWAY
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
}

/// Wheels on: the autopilot's approach is over and its servos come off. It
/// flew the heading bug and the VS dial to the ground, and nothing ever took
/// it out: on the ground a heading servo cannot bank the aircraft, so its
/// error never closes and it winds the ailerons over (EGCC 23L, 2026-09-21:
/// the 737 taxied in with the yoke half way to the right stop, servos on,
/// the bug a quarter of a degree off the nose). Read first, read back, and
/// blackboxed `ap_disconnected`. `None` = it was already off, or no reading.
async fn disconnect_autopilot(xp: &dyn crate::xplane::XPlane) -> Option<bool> {
    // The whole autopilot, not only the servos: the arrest takes the servos
    // out at the roundout, so `servos_on` read 0 on the wheels while the
    // master (`autopilot_on`) held HDG and VS lit and the yaw damper stayed
    // on, and nothing came out (King Air LEMG 13, 1790780099-approach and
    // 1790780311-approach, iMac 2026-09-30: `ap` true down to 32 kt; one
    // swung 70 degrees right from 91 kt under full left steer, #197).
    let on = |r: Result<f64, _>| r.is_ok_and(|v: f64| v > 0.5);
    let (servos, master, damper) = tokio::join!(
        xp.get_float(SERVOS_ON),
        xp.get_float(AP_MASTER_ON),
        xp.get_float(YAW_DAMPER_ON),
    );
    let (servos, master, damper) = (on(servos), on(master), on(damper));
    if !servos && !master && !damper {
        return None;
    }
    if servos || master {
        let _ = autoflight::press(xp, Button::DisconnectFd).await;
    }
    if damper {
        let _ = xp.run_command(CMD_YAW_DAMPER_OFF).await;
    }
    let (s2, m2, d2) = tokio::join!(
        xp.get_float(SERVOS_ON),
        xp.get_float(AP_MASTER_ON),
        xp.get_float(YAW_DAMPER_ON),
    );
    let off = |r: Result<f64, _>| r.is_ok_and(|v: f64| v < 0.5);
    let ok = off(s2) && (!master || off(m2)) && (!damper || off(d2));
    crate::blackbox::record(
        "autoland",
        serde_json::json!({"event": "ap_disconnected", "ok": ok,
            "servos": servos, "master": master, "yaw_damper": damper}),
    );
    Some(ok)
}

/// Rollout starts only after weight-on-wheels. The speed terms distinguish a
/// landing from a low pass once on the surface; they must never substitute for
/// the ground signal because a normal flare can satisfy both in the air.
///
/// ...but the wheels themselves are the best evidence there is. A 737 that
/// touches down a couple of knots over its approach speed (LEMG 12,
/// 2026-09-26: IAS 147 against 145, 146 kt over the ground) satisfied
/// neither speed term, so the loop went on flying the FLARE on the runway
/// for 2.8 s, then balked `overflew_the_runway` from the pavement and
/// firewalled the throttle. `ground_ticks` consecutive guidance ticks with
/// the wheels on and the radio under [`WHEELS_ON_RADIO_FT`] is a landing at
/// any speed; a bounce breaks the count.
fn should_start_rollout(
    already_rolling: bool,
    on_ground: bool,
    radio_ft: f64,
    gs_kts: f64,
    ias_kts: f64,
    approach_ias: f64,
    wheels_on_s: Option<f64>,
) -> bool {
    already_rolling
        || (on_ground
            && radio_ft < WHEELS_ON_RADIO_FT
            && wheels_on_s.is_some_and(|s| s >= WHEELS_ON_S - 1e-9))
        || (on_ground && radio_ft < 5.0 && (gs_kts < 45.0 || ias_kts < approach_ias))
}

/// Seconds of wheels on, from the first read that saw them, that make a
/// landing whatever the speed. It was three consecutive TICKS, which is
/// 0.4 s from first contact at the MacBook's 0.2 s flare tick (the recorded
/// 1790448340 contact below latched 0.4 s after it) and 0.8 s at the
/// iMac's 0.39 s (2026-09-30): a count of ticks is a time only on the
/// machine it was tuned on.
const WHEELS_ON_S: f64 = 0.4;
/// ...with the radio altimeter under this.
const WHEELS_ON_RADIO_FT: f64 = 2.0;

/// The clock [`should_start_rollout`] reads: seconds since the first read
/// with the wheels on under [`WHEELS_ON_RADIO_FT`] (`Some(0.0)` on that
/// read, plus the tick's `dt_s` on each after), or `None` (a bounce, a
/// failed read).
fn wheels_on_time(prev: Option<f64>, on_ground: bool, radio_ft: f64, dt_s: f64) -> Option<f64> {
    if on_ground && radio_ft.is_finite() && radio_ft < WHEELS_ON_RADIO_FT {
        Some(prev.map_or(0.0, |s| s + dt_s.max(0.0)))
    } else {
        None
    }
}

/// NO BALK FROM THE PAVEMENT. With the wheels on the runway a landing has
/// happened: a go-around from there at 145 kt with the throttle closed, the
/// spoilers out and the autobrake armed is the most dangerous thing the loop
/// could ask for. LEMG 12 (2026-09-26): 2.8 s on the wheels in the flare,
/// then `overflew_the_runway` at 0.33 ft radio, full power, "going around"
/// to the tower, and a landing clearance voided for an aircraft that then
/// stopped on the runway. On the ground any balk, and a flare turned into
/// anything else, stays a flare down the runway's course, which the rollout
/// latch takes over within ticks. Pure; the balk refused is returned.
pub fn wheels_hold_the_landing(
    g: Guidance,
    on_ground: bool,
    was_flaring: bool,
    course_deg: f64,
) -> (Guidance, Option<&'static str>) {
    if !on_ground || !(g.balk.is_some() || (was_flaring && g.phase != ApproachPhase::Flare)) {
        return (g, None);
    }
    let refused = g.balk.or(Some("phase_change"));
    (
        Guidance {
            target_heading_deg: course_deg,
            phase: ApproachPhase::Flare,
            balk: None,
            arresting: true,
            ..g
        },
        refused,
    )
}

/// A balked landing called at or above this radio height is a go-around the
/// moment it is called: from here no spool-up ends on the runway (a 737's
/// idle-to-go-around spool, 6 to 8 s at the path's 800 fpm, loses about 100
/// ft), so the tower hears it at once. Below it the call waits for a climb.
const GO_AROUND_CALL_AGL_FT: f64 = 200.0;
/// The climb that makes a low balk real: the radio this far above its
/// lowest since the balk, with the VSI (one tick old) positive.
const GO_AROUND_RISE_FT: f64 = 10.0;

/// A balk being flown, watched until it is a go-around or a landing: the
/// balk's own gate and height, the lowest radio since, and whether the
/// wheels have touched.
#[derive(Debug, Clone, Copy)]
struct BalkWatch {
    why: Option<&'static str>,
    balk_radio_ft: f64,
    low_ft: f64,
    touched: bool,
}

impl BalkWatch {
    fn new(why: Option<&'static str>, radio_ft: f64) -> Self {
        Self {
            why,
            balk_radio_ft: radio_ft,
            low_ft: radio_ft,
            touched: false,
        }
    }

    fn note(&mut self, radio_ft: f64, on_ground: bool) {
        if radio_ft.is_finite() {
            self.low_ft = self.low_ft.min(radio_ft);
        }
        self.touched |= on_ground;
    }
}

/// The "going around" radio call goes out once per balk that was actually
/// FLOWN as a go-around, and never with the wheels on the runway. A balk
/// called low may still land: LFBD 11 (2026-09-27) the 737 balked
/// `overflew_the_runway` at 7.9 ft radio, the call went out on that tick,
/// the tower answered "roger, go around" and dropped the landing clearance,
/// and the wheels touched 1.3 s later with the engines still spooling; the
/// crew had called a go-around and then landed uncleared. So a low balk is
/// called once it CLIMBS (`GO_AROUND_RISE_FT` above its lowest radio, the
/// VSI positive when it could be read), and a balk whose wheels touched
/// only once it is clear of [`GO_AROUND_CALL_AGL_FT`] (a touch that then
/// really left); one that rolls out never calls. A balk at or above that
/// height calls at once; the first LFBD approach's `off_centreline_low` at
/// 96 ft calls as soon as it climbs. Pure.
fn go_around_call_due(
    missed: bool,
    said: bool,
    on_ground: bool,
    watch: Option<&BalkWatch>,
    radio_ft: f64,
    vvi_fpm: Option<f64>,
) -> bool {
    if !missed || said || on_ground {
        return false;
    }
    let Some(w) = watch else { return true };
    if radio_ft >= GO_AROUND_CALL_AGL_FT {
        return true;
    }
    !w.touched && radio_ft - w.low_ft >= GO_AROUND_RISE_FT && vvi_fpm.is_none_or(|v| v > 0.0)
}

/// Radio height at which a final the tower is working but has NOT cleared
/// asks again: "final runway one five, request landing clearance" (a final
/// report, which the tower judges afresh against the strip).
pub const CLEARANCE_QUERY_FT: f64 = 1000.0;
/// Radio height at which a final still without its landing clearance goes
/// around: a crew never lands uncleared (EGKK-EGBB 15, sortie
/// 1790601822-full_auto, 2026-09-28: "expect late landing clearance" at
/// 7 nm, then nothing, and autoland landed; "you landed on runway one five
/// without clearance"). Above the flare by far: the order is the gate path's
/// (`GoAroundLatch::order`), flown from wherever we are.
pub const UNCLEARED_GO_AROUND_FT: f64 = 300.0;

/// What the crew owes a final the tower is working and has not cleared.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnclearedStep {
    /// Ask again, once per approach.
    Ask,
    /// Go around.
    GoAround,
}

/// The crew's watch on its own landing clearance. `working` is the tower's
/// book saying it has us in sequence and has not cleared us (`Inbound`: the
/// "continue approach, expect late landing clearance" answer, or a final
/// report not yet answered); without a tower working us (an uncontrolled
/// field, no two-way contact) there is nothing to be cleared by and the
/// watch is silent. Pure.
#[derive(Debug, Default)]
pub struct UnclearedWatch {
    asked: bool,
}

impl UnclearedWatch {
    pub fn step(
        &mut self,
        working: bool,
        radio_ft: f64,
        on_ground: bool,
        missed: bool,
    ) -> Option<UnclearedStep> {
        if missed {
            // The next approach asks again.
            self.asked = false;
            return None;
        }
        if !working || on_ground || !radio_ft.is_finite() {
            return None;
        }
        if radio_ft <= UNCLEARED_GO_AROUND_FT {
            return Some(UnclearedStep::GoAround);
        }
        if radio_ft <= CLEARANCE_QUERY_FT && !self.asked {
            self.asked = true;
            return Some(UnclearedStep::Ask);
        }
        None
    }
}

/// Cancel an active autoland; returns whether one was flying.
pub fn abort() -> bool {
    VACATE_FOLLOWS.store(false, Ordering::SeqCst);
    RUN.cancel()
}

/// Fly a full emergency autoland to the nearest suitable runway. AP-first: HDG bug
/// tracks the guidance heading, VS mode flies the glidepath, throttle holds the
/// approach speed, gear/flaps come down on final, and it eases to idle in the
/// flare. EXPERIMENTAL — untuned; test high, over an airport, with a hand ready.
/// Own thread/runtime/client (like the recover watcher).
#[allow(clippy::too_many_arguments)]
pub fn run_autoland_scoped(
    base: String,
    apt_dat: String,
    cfg: ApConfig,
    approach_ias: f64,
    voice: Option<Voice>,
    // The chat mirror now rides `voice.alert`; an explicit `s.action` here too
    // double-wrote every autoland line to the transcript (2026-07-24).
    _status: Option<crate::plugin_status::StatusSink>,
    // The arrival's already-chosen landing runway (by wind) AND its airport.
    // `Some` forces that end; `None` self-picks by track — the emergency and
    // manual path, unchanged.
    forced_rwy: Option<PlannedRunway>,
    // The launcher's cancel tokens: a cancel of the loop that handed the
    // landing down (a stand-down landing before or after the launch) is the
    // autoland's too. Empty only for the pilot's own request.
    parents: Vec<std::sync::Arc<AtomicBool>>,
) {
    spawn_autoland(AutolandLaunch {
        base,
        apt_dat,
        cfg,
        approach_ias,
        voice,
        forced_rwy,
        parents,
    });
}

/// Everything an autoland worker owns from arm until hand-back. Keeping the
/// launch facts together prevents the approach thread and its runway re-arm
/// from acquiring parallel argument lists as new per-approach state is added.
struct AutolandLaunch {
    base: String,
    apt_dat: String,
    cfg: ApConfig,
    approach_ias: f64,
    voice: Option<Voice>,
    forced_rwy: Option<PlannedRunway>,
    parents: Vec<std::sync::Arc<AtomicBool>>,
}

struct ArmedRunway {
    runway: Runway,
    ils: Option<crate::nav::Ils>,
    coupled_ok: bool,
}

/// Mutable facts that describe this approach as an operation rather than a
/// flight law: clearances, stability judgment and go-around communication.
/// Keeping them under one owner makes reset-at-new-approach explicit and
/// prevents another free-standing latch being added to the control loop.
struct ApproachRuntime {
    go_around: GoAroundLatch,
    stable_watch: crate::stable_approach::StableWatch,
    stable_clock: std::time::Instant,
    stable_balk: Option<&'static str>,
    stable_said: Option<&'static str>,
    gate_recorded: bool,
    orders: crate::go_around::GoAroundWatch,
    uncleared: UnclearedWatch,
    owe_final_report: bool,
    said_going_around: bool,
    going_around_callsign: Option<Option<String>>,
    balk_watch: Option<BalkWatch>,
    spoke_going_around: bool,
}

impl ApproachRuntime {
    fn arm() -> Self {
        Self {
            go_around: GoAroundLatch::new(),
            stable_watch: crate::stable_approach::StableWatch::default(),
            stable_clock: std::time::Instant::now(),
            stable_balk: None,
            stable_said: None,
            gate_recorded: false,
            orders: crate::go_around::GoAroundWatch::arm(),
            uncleared: UnclearedWatch::default(),
            owe_final_report: false,
            said_going_around: false,
            going_around_callsign: None,
            balk_watch: None,
            spoke_going_around: false,
        }
    }
}

fn announce_autoland(voice: &Option<Voice>, line: String, sev: Priority) {
    tracing::info!(target: "xvoicelayer::autoland", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(sev, line);
    }
}

/// Resolve, announce, tune and terrain-correct the one runway this worker
/// will fly. Failure has already been spoken and logged when this returns
/// `None`; the caller only has to release the active worker.
async fn arm_runway(
    xp: &XPlaneClient,
    apt_dat: &str,
    forced_rwy: Option<&PlannedRunway>,
    gp_deg: f64,
    voice: &Option<Voice>,
) -> Option<ArmedRunway> {
    let (Ok(lat0), Ok(lon0), Ok(_mag_var)) = (
        xp.get_float(LAT).await,
        xp.get_float(LON).await,
        xp.get_float(MAG_VAR).await,
    ) else {
        return None;
    };
    let hdg0 = xp
        .get_float("sim/flightmodel/position/psi")
        .await
        .unwrap_or(0.0);
    let cleared = crate::party_line::clearance_runway();
    let mut from_clearance = false;
    let picked = match forced_rwy {
        Some(planned_rwy) => match runway_by_id(
            apt_dat,
            lat0,
            lon0,
            &planned_rwy.id,
            40.0,
            planned_rwy.icao.as_deref(),
        ) {
            Ok(None) => {
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "forced_runway_missing",
                        "id": planned_rwy.id, "icao": planned_rwy.icao}),
                );
                // A named arrival is not an emergency "land anywhere" arm.
                // Losing its exact end must never silently change airports.
                Ok(None)
            }
            Ok(Some(planned)) => {
                let switch_to = clearance_switch(
                    &planned.id,
                    crate::party_line::clearance_state(),
                    cleared.as_deref(),
                    None,
                    false,
                    None,
                );
                match switch_to.as_deref() {
                    Some(c) => match runway_by_id(
                        apt_dat,
                        lat0,
                        lon0,
                        c,
                        40.0,
                        Some(planned.icao.as_str()),
                    ) {
                        Ok(Some(r)) if r.icao == planned.icao => {
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "runway_from_clearance",
                                    "planned": planned.id, "cleared": r.id}),
                            );
                            announce_autoland(
                                voice,
                                format!(
                                    "Tower cleared us for runway {}, not the planned {}. Landing runway {}.",
                                    crate::circuit::spell_runway(&r.id),
                                    crate::circuit::spell_runway(&planned.id),
                                    crate::circuit::spell_runway(&r.id)
                                ),
                                Priority::Caution,
                            );
                            from_clearance = true;
                            Ok(Some(r))
                        }
                        _ => Ok(Some(planned)),
                    },
                    None => Ok(Some(planned)),
                }
            }
            err => err,
        },
        None => self_pick(apt_dat, lat0, lon0, hdg0),
    };
    let mut runway = match picked {
        Ok(Some(r)) => r,
        other => {
            tracing::warn!(target: "xvoicelayer::autoland", ?other, from=?(lat0, lon0), "no runway found");
            announce_autoland(
                voice,
                "No suitable runway within range. You have the aircraft.".into(),
                Priority::Warning,
            );
            return None;
        }
    };
    tracing::info!(
        target: "xvoicelayer::autoland",
        icao = %runway.icao, id = %runway.id, dist_nm = runway.dist_nm,
        len_m = runway.length_m, course = runway.course_deg, elev = runway.elev_ft,
        "runway selected"
    );
    crate::full_auto::retarget_arrival_runway(
        &runway.id,
        armed_runway_why(forced_rwy.map(|p| p.id.as_str()), &runway.id, from_clearance),
    );
    announce_autoland(
        voice,
        format!(
            "I'll land at {} runway {}, {:.0} miles. I have the approach.",
            runway.icao, runway.id, runway.dist_nm
        ),
        Priority::Warning,
    );
    let ils = crate::ils::tune(xp, &runway.icao, &runway.id, lat0, lon0).await;
    let coupled_ok = ils.is_some() && crate::ils::coupled_allowed();

    let (aim_lat, aim_lon) = project_latlon(
        runway.thr_lat,
        runway.thr_lon,
        runway.course_deg,
        TOUCHDOWN_AIM_NM,
    );
    let probed = crate::terrain::sample_ft(
        xp,
        &[(runway.thr_lat, runway.thr_lon), (aim_lat, aim_lon)],
    )
    .await;
    let (pavement_aim, seen) = aim_elevation_ft(runway.elev_ft, probed.as_deref());
    let gs_aim = if ils.is_some() {
        let (nav, icao, id) = (
            crate::settings::live().nav_data_path.clone(),
            runway.icao.clone(),
            runway.id.clone(),
        );
        tokio::task::spawn_blocking(move || crate::ils::find_glideslope(&nav, &icao, &id))
            .await
            .ok()
            .flatten()
            .and_then(|gs| {
                crate::ils::glideslope_aim_ft(
                    &gs,
                    runway.thr_lat,
                    runway.thr_lon,
                    runway.course_deg,
                    TOUCHDOWN_AIM_NM,
                    gp_deg,
                )
            })
    } else {
        None
    };
    let aim = aim_with_glideslope(pavement_aim, gs_aim);
    crate::blackbox::record("autoland", serde_json::json!({
        "event": "threshold_elev", "rwy": format!("{} {}", runway.icao, runway.id),
        "header_ft": runway.elev_ft, "probed_ft": seen, "aim_ft": aim,
        "pavement_aim_ft": pavement_aim, "gs_aim_ft": gs_aim,
        "width_m": runway_width_m(&runway),
        "xtk_limit_m": landing_xtk_limit_m(runway_width_m(&runway)),
        "flare_xtk_limit_m": flare_xtk_limit_m(runway_width_m(&runway)),
    }));
    if (aim - runway.elev_ft).abs() > THRESHOLD_ELEV_TRUST_FT {
        let diff = aim - runway.elev_ft;
        let line = if gs_aim.is_some() {
            format!(
                "Runway {} on the glideslope, aiming at {:.0} feet.",
                crate::circuit::spell_runway(&runway.id),
                aim
            )
        } else {
            format!(
                "Runway {} threshold is {:.0} feet {} the field elevation. Aiming at {:.0} feet.",
                crate::circuit::spell_runway(&runway.id),
                diff.abs(),
                if diff < 0.0 { "below" } else { "above" },
                aim
            )
        };
        announce_autoland(voice, line, Priority::Advisory);
        runway.elev_ft = aim;
    } else if (aim - pavement_aim).abs() > 0.5 {
        runway.elev_ft = aim;
    }
    Some(ArmedRunway {
        runway,
        ils,
        coupled_ok,
    })
}

fn spawn_autoland(launch: AutolandLaunch) {
    let AutolandLaunch {
        base,
        apt_dat,
        cfg,
        approach_ias,
        voice,
        forced_rwy,
        parents,
    } = launch;
    // The last approach's tracking goes before the flag rises (it is
    // unreadable once ACTIVE is set again, not at the first tick).
    *TRACKING_ERROR.lock().unwrap_or_else(|e| e.into_inner()) = None;
    // Taken on the caller's thread: a stand-down landing before the thread
    // runs cancels THIS run. The run's end clears the panel line and the
    // landing configuration, only while it still owns the slot.
    let lease = RUN.take_over().with_cleanup(&LANDING_CONFIGURED, || set_panel(""));
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        let lease = &lease;
        rt.block_on(async move {
            // Kept for a re-arm on a different runway (see the clearance
            // watch in the tick): the PARENTS only, never this run's own
            // token, which the re-arm's take-over cancels.
            let base_url = base.clone();
            let xp = XPlaneClient::new(base)
                .with_parents(parents.clone())
                .with_cancel(lease.cancel.clone());
            if xp.cancelled() {
                // A cancelled launch takes what was handed down for it with
                // it: the next arm, perhaps another flight's instrument
                // approach, must not fly a circuit's visual gate or path.
                drop_handed_down();
                return;
            }
            LANDING_CONFIGURED.store(false, Ordering::SeqCst);
            let announce = |line: String, sev: Priority| {
                tracing::info!(target: "xvoicelayer::autoland", spoke = %line, "copilot");
                if let Some(v) = &voice { v.alert(sev, line); }
            };
            // The approach speed is a number somebody handed us; the airframe's
            // stall floor is a fact. A Cessna's 65 kt on an RV-10 whose clean
            // stall is 65 flew the speed schedule into a stall (2026-09-11).
            // The LANDING configuration's floor, not the clean one: the 737's
            // Vref 145 floored to the clean 167 overflew LFLL 35L twice
            // (2026-09-17; `aircraft::approach_floor_kts`).
            let approach_ias = {
                let floored = crate::aircraft::floored_target(approach_ias, crate::aircraft::approach_floor_kts());
                if floored > approach_ias {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "approach_ias_floored", "asked": approach_ias, "floor": floored}),
                    );
                    announce(
                        format!("Approach speed {approach_ias:.0} is below this airframe's {floored:.0} knot floor. Using {floored:.0}."),
                        Priority::Caution,
                    );
                }
                floored
            };

            // This approach's path angle: a steeper one handed down for a
            // visual segment (LSZS), else 3. Taken now so no approach
            // inherits another's.
            let gp_deg = arm_glidepath();
            // The stabilised gate this final is judged at (#199): a
            // circuit's is visual, every other the instrument gate.
            let gate = arm_gate();
            if (gp_deg - 3.0).abs() > 1e-9 {
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "glidepath", "deg": gp_deg}),
                );
            }
            let ArmedRunway {
                runway: rwy,
                ils,
                coupled_ok,
            } = match arm_runway(&xp, &apt_dat, forced_rwy.as_ref(), gp_deg, &voice).await {
                Some(armed) => armed,
                // The run ends with the thread (the lease drops the flag).
                None => return,
            };
            // THE ENERGY PLAN (`approach_energy`): the airframe's own idle
            // deceleration, planned back from this final's gate. `None` on an
            // airframe that declares none: the distance schedule flies.
            let energy_model = crate::aircraft::live_profile()
                .and_then(|p| crate::approach_energy::EnergyModel::from_profile(&p, gp_deg));
            // What the gate judges the speed against (the stable sample's).
            let energy_vapp = approach_target_kts(
                approach_ias,
                1.0,
                &crate::aircraft::flap_limits(),
                crate::aircraft::config_floor_kts(1.0),
            );
            // A planned final never accelerates: the speed it asked for is a
            // ceiling for the rest of it (the curve is a MAXIMUM, and a
            // governor handed a maximum flies it as a target).
            let mut energy_cap: Option<f64> = None;
            if let Some(m) = &energy_model {
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "energy_model", "vapp": energy_vapp, "gp_deg": gp_deg,
                        "path_decel": m.detents.iter().map(|d| (d.ratio, m.path_decel(d.ratio))).collect::<Vec<_>>()}),
                );
            }

            // Engage the AP (FD up first for airframes that need it), and
            // CONFIRM it: the tick re-presses until the servos read on
            // (`ApEngage`), holding the aircraft level until then.
            let engage_t0 = std::time::Instant::now();
            let mut engage = ApEngage::default();
            let mut engage_pitch = FlarePitch::with_gains(engage_hold_gains());
            let mut engage_held = false;
            {
                let e = engage.arm(0.0);
                let ok = crate::ap_engage::press(&xp, &cfg.fd_commands).await;
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "ap_engage", "try": e.try_n, "sent": ok,
                        "at_s": 0.0, "servos": null, "ias": null}),
                );
            }
            // The altitude dial OUT OF THE WAY. Autoland flies the vertical
            // through the VS dial and never touched the altitude dial, so it
            // sat wherever the follower or the pilot left it: at LEAX the
            // plan's 2,600 ft constraint, 130 ft under the arm, and the AP's
            // altitude capture was already arresting the descent as the arm
            // came. The VS dial's -1,200 was ignored for six seconds, flaps
            // ballooned it to +300 fpm, and only the divergence watchdog's
            // re-press of VS mode freed it ("not holding the descent",
            // 2026-09-15). Parked well above and VS re-selected so no capture
            // can start; a go-around levels there at the latest, which the
            // per-tick VS re-select then takes down again.
            // The airframe's window rule (`autoflight::WindowRule`): above
            // (the LEAX rule), or on an MCP that flies a V/S only toward its
            // window (the 737) at the field, since an approach descends.
            let window_rule = autoflight::panel().window;
            {
                let (dial_r, alt_r) = tokio::join!(
                    autoflight::read_dial(&xp, Dial::Altitude),
                    xp.get_float(ALT_IND_FT)
                );
                if let (Ok(dial), Ok(alt_ind)) = (dial_r, alt_r) {
                    if let Some(parked) = parked_window_ft(window_rule, dial, alt_ind, rwy.elev_ft) {
                        let _ = autoflight::set_dial(&xp, Dial::Altitude, parked).await;
                        autoflight::press_logged(&xp, Button::VerticalSpeed, "autoland_arm").await;
                        crate::blackbox::record("autoland", serde_json::json!({
                            "event": "alt_dial_parked", "was": dial, "now": parked, "alt_ft": alt_ind,
                            "rule": window_rule,
                        }));
                    }
                }
            }

            *TRACKING_ERROR.lock().unwrap_or_else(|e| e.into_inner()) = None;
            let mut last_phase: Option<ApproachPhase> = None;
            let mut flare_power = FlarePower::default();
            // The heading knob, set as a pilot sets it (`bug_knob`, issue #93).
            let mut bug_hold = crate::heading_bug::BugHold::default();
            let knob_t0 = std::time::Instant::now();
            // The lever the approach was flown on, for a balloon to hold.
            let mut approach_thr: Option<f64> = None;
            // The stabilised-approach rule (`stable_approach`, #199): the
            // breaches held so far, the clock they are timed on, the balk
            // word and spoken reason of a go-around it ordered, and whether
            // this approach's gate crossing has been recorded.
            // Watch the tower's go-around orders from here: an order shouted
            // at somebody else's approach half an hour ago must not balk this
            // one on its first tick. A WATCH, never the main loop's `take_*`
            // flag: sharing that flag, the main loop's FSM sync consumed every
            // order before this loop's tick saw it (LEMG 31, 2026-09-28).
            // The pilot's order rides the same watch (`go_around`, #77).
            // The crew's own watch on the landing clearance (#155).
            // The final report owed on a SECOND approach. The circuit or the
            // arrival makes the first one, which is what earns the landing
            // clearance; a missed approach then flies away, and until this
            // existed nothing called again — eleven minutes of missed
            // approach at Malaga with the tower waiting on "report downwind",
            // then a landing without a clearance (2026-08-05).
            // The "going around" transmission: said once per balked landing,
            // with the callsign read from the aircraft the first time it's
            // needed (an HTTP read, so not on every tick).
            // The balk being flown until it climbs (then the call) or the
            // wheels take it (then no call: see [`go_around_call_due`]), and
            // whether the intercom said "going around" for it.
            let mut runtime = ApproachRuntime::arm();
            // The current missed approach's (or pattern's) progress, judged
            // by the ground it makes (`reversal::MissedWatch`), on a clock of
            // this loop's own; and the speed the pattern began at, which
            // sizes its track to go.
            let mut missed_watch: Option<crate::reversal::MissedWatch> = None;
            let watch_epoch = std::time::Instant::now();
            let mut pattern_gs_kts = 0.0_f64;
            let mut gear_down = false;
            let mut landing_config_done = false;
            // The flap the wing is flying on at the arm: the schedule's first
            // write must know what it would take away (`flap_kept_flying`).
            // A failed read is no measurement: unknown, as before.
            let mut last_flap: Option<f64> = xp
                .get_float(FLAP_RATIO)
                .await
                .ok()
                .filter(|f| f.is_finite() && (0.0..=1.0).contains(f));
            let mut vs_diverged_s: f64 = 0.0;
            let mut prev_vs_err: Option<f64> = None;
            // Warned once per approach when the VSI cannot be read.
            let mut vvi_read_failed = false;
            // The vertical speed one tick ago, for the arrest's trend term.
            let mut prev_vvi: Option<(f64, std::time::Instant)> = None;
            // The flare's elevator law. Its integrator is per-approach: a
            // standing pull must not survive a go-around into the next one.
            let mut flare_pitch =
                FlarePitch::with_gains(FlareGains::from_profile(crate::aircraft::flare_elevator()));
            // ...or, where the airframe declares its elevator's response,
            // the arrest flown on that model (`FlareModel`, the 737).
            let mut flare_model = FlareModel::from_profile(crate::aircraft::flare_elevator());
            // The lever the loop last commanded, for the tick record (the
            // flare's table had the elevator and the sink but not the power).
            let mut last_lever: Option<f64> = None;
            // Last tick was the flare: a balloon stays in it (`hold_flare`).
            let mut flaring = false;
            // This flare was committed (`commit_flare_latched`).
            let mut flare_committed = false;
            let mut pitch_yielded = false;
            // The arrest has the servos (see `arrest_hands_step`); the
            // wings-level law that holds the track while it does; the
            // pilot has taken the roll for this approach.
            let mut arrest_hands = false;
            // This tick's arrest is only `hold_roundout`'s (a balloon over
            // the band), for the blackbox's `roundout_held` edges.
            let mut roundout_held = false;
            let mut wings = arrest_wings();
            let mut short_final: Option<ShortFinal> = None;
            let mut arrest_rate = crate::autotakeoff::XtkRate::default();
            let mut arrest_prev_xtk: Option<f64> = None;
            let mut roll_yielded = false;
            let mut last_arresting = false;
            // The final's air (`FinalAir`), for how the flare holds the column.
            let mut final_air = FinalAir::default();
            // Airspeed history for the throttle governor's derivative term — the
            // anti-stall reflex needs the TREND (kts/sec), not just the error.
            let mut prev_ias: Option<f64> = None;
            let mut prev_ias_t = std::time::Instant::now();
            // Cross-track history for the intercept damping term.
            let mut prev_xtk: Option<(f64, std::time::Instant)> = None;
            let mut rollout_rate = crate::autotakeoff::XtkRate::default();
            let mut xtk_rate: f64 = 0.0;
            // Wind crab, low-passed — see [`CrabFilter`] for the hunt it ends.
            let mut crab_filter = CrabFilter::new();
            // Integral trim on the final's track error — see [`TrackTrim`]
            // for the standing 35 m the crab loop could not see.
            let mut track_trim = TrackTrim::new();
            // Rollout: latched at touchdown, then brakes down — to a full stop,
            // or to taxi pace when a vacate drive follows. The takeoff roll law
            // (P + trim, tested) holds the centreline through the plugin steer
            // channel while it brakes.
            let mut rolling_out = false;
            let mut wheels_on: Option<f64> = None;
            // When the last live tick read the wheels (the wheels-on clock).
            let mut wheels_read_t: Option<std::time::Instant> = None;
            let mut vacate_follows = false;
            let mut roll_law = crate::autotakeoff::RollSteer::default();
            // A castoring nosewheel steers the rollout on its brakes too
            // (`RolloutBrakeSteer`, issue #166).
            let rollout_brake_steer = rollout_brake_steers();
            let mut rollout_brakes = RolloutBrakeSteer::for_airframe();
            // A steered nosewheel that declares its own rollout law
            // (`RolloutSteer`, issue #197); else the takeoff roll's default.
            let mut rollout_steer = crate::aircraft::rollout_steer().map(RolloutSteer::new);
            // The wheels off again after the rollout latched (a bounce).
            let mut bounce_hold = BounceHold::default();
            // A hold-off airframe held on from its first contact
            // (`TouchHold`), and the elevator the flare last wrote.
            let holds_off = flare_model.as_ref().is_some_and(|m| m.holds_off());
            let mut touch_hold = TouchHold::default();
            let mut last_flare_elev: Option<f64> = None;
            // The landing readback last sent, so an owed one goes out once
            // (the party line keeps it owed until the tower hears it).
            let mut readback_sent: Option<String> = None;
            // The cleared ident already looked up against apt.dat, so the
            // (seconds-long) lookup of one that is not at this field runs once.
            let mut clearance_checked: Option<String> = None;
            // The course reversal in progress, if any (see [`crate::reversal`]):
            // how this loop gets round to a final it is on the wrong side of.
            let mut reversal: Option<crate::reversal::Reversal> = None;
            let mut reversals_flown: u32 = 0;
            // A published missed approach asked for once: one that cannot be
            // flown falls through to our own pattern, never to "Your aircraft"
            // with nobody there (LEMG 13 SR22 circuit 2026-09-29).
            let mut missed_procedure_tried = false;
            // The pattern's plan over the probed ground (#182), probed beside
            // the loop (the plugin answers in seconds; the aircraft flies on),
            // and when it began. Begun in the go-around's climb, so it is
            // ready when the reposition wants it.
            let mut reversal_plan: Option<(
                tokio::task::JoinHandle<crate::reversal::PatternPlan>,
                std::time::Instant,
            )> = None;
            // Straight ahead at a held altitude: while that plan is probed,
            // or because no pattern here clears the ground (#182).
            let mut hold: Option<TerrainHold> = None;
            // The live floor over the ground under and ahead on every leg
            // but the final (#182, the circuit's `live_floor_ft`): the floor
            // and when it tripped, when it was last probed, and whether the
            // spell has been said.
            let mut live_floor: Option<(f64, std::time::Instant)> = None;
            let mut live_floor_probed: Option<std::time::Instant> = None;
            let mut live_floor_said = false;
            // How far out this approach has legitimately been: the arm
            // distance, pushed out by any pattern flown since.
            let mut recede_ref_nm = rwy.dist_nm;
            // A give-up happened: the speed is handed to the flight governor
            // once the loop has ended (see below).
            let mut gave_up = false;
            // The confirmed engage's pitch authority is checked from the
            // next tick with a climb read (#181).
            let mut authority_owed = false;
            let mut ils_xcheck = crate::ils::XCheck::default();
            // The tick's writes, sent in one round (`sim::state::Writes`),
            // and the lever last written, for `lever_took`.
            let mut writes = sim::state::Writes::new();
            let mut last_thr_written: Option<f64> = None;
            let mut coupling = crate::ils::Coupling::Off;
            let mut coupling_armed_s = 0.0_f64;
            // The thrust is this loop's (`autoflight::ThrustWatch`): an
            // ENGAGED autothrottle is taken out to ARM, said once. The A/T
            // in SPEED 230 from a manual recovery overrode every lever
            // written from 13:02 to 13:09 (LEMG 13, 2026-10-05).
            let mut thrust_watch = autoflight::ThrustWatch::default();
            let thrust_t0 = std::time::Instant::now();
            loop {
                if xp.cancelled() || !lease.current() {
                    break;
                }
                // The tick runs to its period (`tick_ms`), not sleep + work.
                let pacer = sim::state::Pacer::start();
                // Frozen sim (replay or pause): a replay's positions are a
                // recording and not ours to fly; a pause is real but frozen,
                // and ticking the throttle governor against a frozen airspeed
                // walks the lever to its clamp for a surge on unpause. Freeze
                // (no writes, no state, no blackbox) and resume when it's live.
                //
                // ONE read a tick (`sim::state`): the freeze flags, position,
                // height, groundspeed and heading as ONE frame saw them (#157:
                // read separately, latitude and longitude paired two frames,
                // metres of cross-track for a tick at speed), and with them
                // everything the tick used to ask for later: the VSI, the
                // lever (the profile's own beside the frame), the servos, the
                // mode lights, the bug, the ILS needles and the yields. Every
                // round of requests is a sim frame: the freeze check, this
                // read, the VSI and lever, the ILS, the knob and yields, the
                // lever write and its read back made Final 0.56 s (`power` dt
                // median) against a 200 ms sleep and the Flare 0.39 s against
                // 100 ms on the iMac at 20 fps (2026-09-30), where a MacBook
                // ran near 0.3. An older plugin falls back to separate reads,
                // still one round.
                let thr_name = throttle_dref();
                // Beside the frame, in its round: the lever, the autopilot's
                // mode bitfield and the autothrottle (`crate::autoflight`:
                // the modes' one truth, and a thrust this loop owns).
                use autoflight::names as afn;
                let (st, extra) = if ils.is_some() {
                    sim::state::read_with(
                        &xp,
                        sim::state::Want(AUTOLAND_WANT_ILS),
                        &[thr_name.as_str(), afn::AP_STATE, afn::AT_ON, afn::AT_MODE],
                    )
                    .await
                } else {
                    sim::state::read_with(
                        &xp,
                        sim::state::Want(AUTOLAND_WANT),
                        &[thr_name.as_str(), afn::AP_STATE, afn::AT_ON, afn::AT_MODE],
                    )
                    .await
                };
                if st.frozen() {
                    tokio::time::sleep(std::time::Duration::from_millis(400)).await;
                    continue;
                }
                let mut extra = extra.into_iter();
                let mut unread = || Err(anyhow::anyhow!("unread"));
                let mut throttle_read = Some(extra.next().unwrap_or_else(&mut unread));
                let ap_modes = extra
                    .next()
                    .and_then(|r| r.ok())
                    .and_then(autoflight::Modes::from_read);
                let at_engaged = {
                    let (on, mode) = (extra.next(), extra.next());
                    autoflight::autothrottle_engaged(
                        on.and_then(|r| r.ok()),
                        mode.and_then(|r| r.ok()),
                    )
                };
                // Before the lever is written: an engaged autothrottle fights it.
                {
                    let step = thrust_watch.step(thrust_t0.elapsed().as_secs_f64(), at_engaged);
                    if let Some(line) = autoflight::act_thrust(&xp, step, "autoland").await {
                        announce(line.to_string(), Priority::Caution);
                    }
                }
                let pos = st.position();
                let (var_r, ias_r, hpath_r, ground_r, phi_r) = (
                    st.get(F::MagVar),
                    st.get(F::Ias),
                    st.get(F::Hpath),
                    st.get(F::OnGround),
                    st.get(F::Phi),
                );
                let (lat_r, lon_r) = sim::position::lat_lon(&pos);
                let elev_r = sim::position::field(&pos, |p| p.elev_m);
                let agl_r = sim::position::field(&pos, |p| p.agl_m);
                let gs_r = sim::position::field(&pos, |p| p.gs_ms);
                let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
                // The flight model's vertical speed, from the frame the height
                // came from: the gauge (`vvi`) lags it by about a second, so
                // the touchdown is scored on this one (#198). Logged only
                // until each airframe's flare is refitted to it.
                let vs_fm = sim::position::field(&pos, |p| p.vs_fpm).ok();
                let pos_source = sim::position::source_word(&pos);
                let (Ok(lat), Ok(lon), Ok(elev_m), Ok(agl_m), Ok(gs_ms), Ok(var), Ok(ias), Ok(on_ground)) =
                    (lat_r, lon_r, elev_r, agl_r, gs_r, var_r, ias_r, ground_r)
                else { tokio::time::sleep(std::time::Duration::from_millis(300)).await; continue };
                let alt = elev_m * M_TO_FT;
                let radio_ft = agl_m * M_TO_FT;
                let gs_kts = gs_ms * MS_TO_KTS;

                // THE TOWER'S WORD DURING THE APPROACH. The clearance was read
                // once, at arm time; the final report that EARNS it goes out
                // after that, so the answer was never seen: "negative, runway
                // zero niner, I say again runway zero niner ... cleared to
                // land" came back five seconds after the pick, and autoland
                // flew 27 into a 21 kt tailwind (LXGB, 2026-09-14). Now the
                // landing clearance is read back (the runway-critical echo
                // the tower waits for, the crew's job while a loop flies),
                // and a clearance for ANOTHER end of THIS field re-arms the
                // approach on it: a fresh loop, forced onto the cleared
                // runway, repositioning from wherever we are. A departure
                // state's runway is the departure's; a stale one is never
                // ours to land on.
                if on_ground < 0.5 && !rolling_out {
                    let cleared = crate::party_line::clearance_runway();
                    match crate::party_line::readback_owed() {
                        None => readback_sent = None,
                        Some(owed) if landing_readback(&owed) && readback_sent.as_deref() != Some(&owed) => {
                            readback_sent = Some(owed.clone());
                            if runtime.going_around_callsign.is_none() {
                                runtime.going_around_callsign = Some(crate::circuit::crew_callsign(&xp).await);
                            }
                            let cs = runtime.going_around_callsign.clone().flatten();
                            let rwy_words = cleared
                                .as_deref()
                                .map(|r| format!(" runway {}", crate::circuit::spell_runway(r)))
                                .unwrap_or_default();
                            crate::circuit::transmit(crate::circuit::radio_call(
                                &cs,
                                &format!("cleared to land{rwy_words}"),
                            ));
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "landing_readback", "rwy": cleared}),
                            );
                        }
                        Some(_) => {}
                    }
                    // A re-sequence the go-around order named is taken once
                    // the climb away is flown (#182).
                    let expected = if runtime.go_around.climbing() {
                        None
                    } else {
                        runtime.orders.expected_runway()
                    };
                    if let Some(c) = clearance_switch(
                        &rwy.id,
                        crate::party_line::clearance_state(),
                        cleared.as_deref(),
                        clearance_checked.as_deref(),
                        runtime.go_around.active(),
                        expected.as_deref(),
                    ) {
                        clearance_checked = Some(c.clone());
                        let resequenced = runtime.go_around.active();
                        match runway_by_id(&apt_dat, lat, lon, &c, 40.0, Some(rwy.icao.as_str())) {
                            Ok(Some(r)) if r.icao == rwy.icao => {
                                crate::blackbox::record(
                                    "autoland",
                                    serde_json::json!({"event": "runway_from_clearance",
                                        "planned": rwy.id, "cleared": r.id, "in_flight": true,
                                        "resequenced": resequenced,
                                        "radio": radio_ft, "dist": r.dist_nm}),
                                );
                                announce(
                                    if resequenced {
                                        format!(
                                            "Tower expects us on runway {}. Setting up for runway {}.",
                                            crate::circuit::spell_runway(&r.id),
                                            crate::circuit::spell_runway(&r.id)
                                        )
                                    } else {
                                        format!(
                                            "Tower cleared us for runway {}, not {}. Setting up for runway {}.",
                                            crate::circuit::spell_runway(&r.id),
                                            crate::circuit::spell_runway(&rwy.id),
                                            crate::circuit::spell_runway(&r.id)
                                        )
                                    },
                                    Priority::Caution,
                                );
                                // The mission's ONE arrival runway moves
                                // with us, before the new loop arms: its
                                // ILS tune, the crew's calls, the vacate
                                // and the card read the cleared end (#64).
                                crate::full_auto::retarget_arrival_runway(
                                    &r.id,
                                    if resequenced { "resequenced" } else { "clearance" },
                                );
                                // A new loop owns the aircraft from its first
                                // tick; this one is taken over (cancelled)
                                // and leaves ACTIVE to it. Our parents are
                                // its parents.
                                run_autoland_scoped(
                                    base_url.clone(),
                                    apt_dat.clone(),
                                    cfg.clone(),
                                    approach_ias,
                                    voice.clone(),
                                    None,
                                    Some(PlannedRunway::at(&r.icao, &r.id)),
                                    parents.clone(),
                                );
                                break;
                            }
                            other => {
                                crate::blackbox::record(
                                    "autoland",
                                    serde_json::json!({"event": "clearance_runway_elsewhere",
                                        "flying": format!("{} {}", rwy.icao, rwy.id), "cleared": c,
                                        "found": other.ok().flatten().map(|r| format!("{} {}", r.icao, r.id))}),
                                );
                            }
                        }
                    }
                }

                // ROLLOUT TO A STOP. Once on the ground and slow, cut the power
                // and brake to a halt before handing off — the old touchdown
                // path just idled the throttle at 40 kt and said "you have the
                // aircraft", leaving it coasting down the runway with nobody on
                // the brakes (the circuit's landings rolled on unmanaged,
                // 2026-07-23). Latched, so the approach guidance below never
                // re-fires once we're down. The nose is held to the centreline
                // through the plugin steer channel with the takeoff roll law.
                //
                // Commit on GROUNDSPEED *or* airspeed-below-approach: the bare
                // gs < 45 gate is a 172 number, and a heavier airframe landing
                // into a TAILWIND touches down well above it — a Twin Otter
                // planted at 70 kt gs (56 KIAS, 6 kt tailwind on 27) never
                // latched, so nothing steered the centreline OR braked; the
                // approach guidance floated past the aimpoint, flipped
                // Flare->Intercept, and the go-around latch firewalled it off
                // the end into the bay (LXGB, 2026-07-26). Airspeed below the
                // approach speed can identify a committed landing only after
                // weight-on-wheels. Without that gate a normal flare below five
                // feet could latch rollout in the air, cut power, and arm the
                // brakes before touchdown.
                let wheels_t = std::time::Instant::now();
                let wheels_dt = wheels_read_t
                    .map_or(0.0, |t0| wheels_t.duration_since(t0).as_secs_f64());
                wheels_read_t = Some(wheels_t);
                wheels_on = wheels_on_time(wheels_on, on_ground > 0.5, radio_ft, wheels_dt);
                if should_start_rollout(
                    rolling_out,
                    on_ground > 0.5,
                    radio_ft,
                    gs_kts,
                    ias,
                    approach_ias,
                    wheels_on,
                ) {
                    // Consumed once, at rollout entry: a stale flag from a
                    // mission that never landed must not turn a later
                    // standalone landing into a rolling handoff to nobody.
                    if !rolling_out {
                        rolling_out = true;
                        // A hold-off airframe holds the pull it touched with
                        // (`TouchHold`); the flare's own contact, if it saw
                        // one first, already took it.
                        // Only a column the flare was flying: never a take
                        // of the pitch the arrest never had.
                        if holds_off
                            && !pitch_yielded
                            && last_flare_elev.is_some()
                            && touch_hold.note(true, last_flare_elev)
                        {
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "touch_hold", "held": touch_hold.held(),
                                    "at": "rollout", "radio": radio_ft, "gs": gs_kts, "ias": ias,
                                    "on_ground": on_ground > 0.5}),
                            );
                        }
                        // The arrest's last aileron rides the roll channel's
                        // 1 s heartbeat onto the wheels unless it is
                        // centred: full right at the handover banked a 737
                        // 9.9 degrees on the mains and slid it 6.1 m right
                        // (EGKK 08R 1790687142, 2026-09-29).
                        if arrest_hands {
                            let _ = xp.set_float(FLARE_ROLL, 0.0).await;
                        }
                        disconnect_autopilot(&xp).await;
                        vacate_follows = VACATE_FOLLOWS.swap(false, Ordering::SeqCst);
                        // Wheels on this runway: the fact the vacate needs.
                        *LANDED_RUNWAY.lock().unwrap_or_else(|e| e.into_inner()) =
                            Some(rwy.id.clone());
                        // A balk the wheels took is a LANDING (LFBD 11,
                        // 2026-09-27): the call waited for a climb that
                        // never came, so the tower still holds our landing
                        // clearance; the intercom's "going around" is
                        // corrected.
                        if runtime.go_around.active() {
                            let along_m = along_track_nm(lat, lon, &rwy) * M_PER_NM;
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "balk_landed",
                                    "why": runtime.balk_watch.and_then(|w| w.why),
                                    "balk_radio": runtime.balk_watch.map(|w| w.balk_radio_ft),
                                    "radio": radio_ft, "gs": gs_kts,
                                    "remaining_m": (rwy.length_m + along_m).max(0.0),
                                    "called": runtime.said_going_around}),
                            );
                            if runtime.spoke_going_around {
                                announce(
                                    "Correction, we're down. Landing, not going around.".into(),
                                    Priority::Caution,
                                );
                            }
                        }
                    }
                    let xtk_nm = cross_track_nm(lat, lon, &rwy);
                    let xtk_m = xtk_nm * M_PER_NM;
                    // The roll's own closure rate and its own tick time. The
                    // approach path below never runs again once we are rolling
                    // out, so `prev_xtk` is ours from here; its last value is
                    // the final's, which is the continuity we want at the
                    // wheels. The steering law used to be handed a HARDCODED
                    // 0.4 s while this loop actually ticks at 300 ms plus two
                    // round trips, so its lead term and its slew limit were
                    // both computed against a clock that did not exist.
                    let t_roll = std::time::Instant::now();
                    let roll_dt = prev_xtk
                        .map(|(_, t0)| t_roll.duration_since(t0).as_secs_f64())
                        .filter(|d| (0.05..=3.0).contains(d))
                        .unwrap_or(0.4);
                    let xtk_rate = rollout_rate.update(
                        prev_xtk
                            .filter(|_| roll_dt > 0.0)
                            .map_or(0.0, |(x0, _)| (xtk_m - x0 * M_PER_NM) / roll_dt),
                        roll_dt,
                    );
                    prev_xtk = Some((xtk_nm, t_roll));
                    let target = crate::autotakeoff::roll_target_heading(
                        rwy.course_deg,
                        xtk_m,
                        xtk_rate,
                        gs_kts,
                    );
                    let psi = psi_r.as_ref().copied().unwrap_or(rwy.course_deg);
                    let err = crate::angles::signed_diff_deg(target, psi);
                    // OFF THE WHEELS after the latch: a bounce (`BounceHold`).
                    // The latch takes the first tick the wheels read on, and
                    // at LEMG 13 (the SR22, 2026-09-29) that tick was a
                    // bounce both times: 1790689843 flew 3.5 to 3.8 ft for
                    // six seconds after it, 1790694457 up to 9 ft for four
                    // and a half, the brakes split and the rudder steering an
                    // aircraft in the air with the roll channel let go.
                    // 1790694457 drifted from 4.1 m to 10.3 m right at 1.1 to
                    // 1.8 m/s and landed with 0.39 of left brake already on,
                    // which slewed the nose from 134 to 127 in a second on a
                    // 131.5 track (the scorer's 3.8 degrees of drift at the
                    // touchdown). In the air: no wheel brake, no rudder, the
                    // arrest's wings closing the track on the centreline.
                    let airborne = on_ground < 0.5;
                    let bounce = bounce_hold.tick(airborne);
                    if matches!(bounce, BounceTick::Left | BounceTick::Landed) {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "bounce",
                                "off": bounce == BounceTick::Left,
                                "spell": bounce_hold.spells(), "radio": radio_ft,
                                "gs": gs_kts, "xtk_m": xtk_m, "psi": psi}),
                        );
                    }
                    if bounce == BounceTick::Landed {
                        // Down again: the split starts from the wheels, never
                        // from what it learnt steering the air.
                        rollout_brakes = RolloutBrakeSteer::for_airframe();
                    }
                    // A declared rollout steer damps on the sim's yaw rate
                    // (`RolloutSteer`, #197): carried by the tick's one read
                    // (`sim::state`, `F::R`), no round of its own.
                    let yaw_r = match rollout_steer.as_ref() {
                        Some(_) if !airborne => st.opt(F::R),
                        _ => None,
                    };
                    let steer = if airborne {
                        if let Some(law) = rollout_steer.as_mut() {
                            law.in_air();
                        }
                        0.0
                    } else if let Some(law) = rollout_steer.as_mut() {
                        law.step(err, roll_dt, gs_kts, yaw_r)
                    } else {
                        roll_law.step(err, roll_dt, gs_kts, 0.0)
                    };
                    let track_roll = hpath_r.as_ref().ok().copied();
                    let side_roll = side_velocity_m_s(track_roll, rwy.course_deg, gs_kts)
                        .unwrap_or(xtk_rate);
                    let phi_roll = phi_r.as_ref().ok().copied();
                    let air_bank_tgt = phi_roll.map(|phi| {
                        arrest_bank_target(xtk_m, side_roll, wings.predicted(phi), radio_ft)
                    });
                    let air_ail = match (airborne && arrest_hands && !roll_yielded, phi_roll, air_bank_tgt) {
                        (true, Some(phi), Some(tgt)) => Some(wings.step(phi, tgt, roll_dt)),
                        _ => None,
                    };
                    // Queued: the rollout's writes go out in one round below.
                    if let Some(a) = air_ail {
                        // Ours, not a hand: unmarked, the scorer read this
                        // bounce's bank as a pilot's yoke (LEMG 13 SR22
                        // 1790697159, yoke_under_ap yr -0.18, 2026-09-29).
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_ROLL, a);
                    } else if bounce == BounceTick::Landed && arrest_hands {
                        // The rollout entry's centring, again at the wheels.
                        writes.set(FLARE_ROLL, 0.0);
                    }
                    // Brake by wanted deceleration, not a fixed clamp: a
                    // comfortable rate when the runway allows it, harder
                    // only when stopping in what remains demands it; on a
                    // castoring nosewheel, split between the wheels to steer
                    // (`RolloutBrakeSteer`; an unread heading steers nothing).
                    let along_m = along_track_nm(lat, lon, &rwy) * M_PER_NM;
                    let remaining_m = (rwy.length_m + along_m).max(0.0);
                    let target_kts = if vacate_follows { VACATE_HANDOFF_KTS } else { 0.0 };
                    let brake = rollout_brake_ratio(gs_kts, target_kts, remaining_m);
                    let brakes = if airborne {
                        RolloutBrakes {
                            sym: 0.0,
                            left: 0.0,
                            right: 0.0,
                        }
                    } else if rollout_brake_steer {
                        rollout_brakes.step_with(
                            if psi_r.is_ok() { err } else { f64::NAN },
                            psi_r.as_ref().ok().copied(),
                            track_roll,
                            steer,
                            roll_dt,
                            gs_kts,
                            brake,
                        )
                    } else {
                        RolloutBrakes {
                            sym: brake,
                            left: 0.0,
                            right: 0.0,
                        }
                    };
                    // A hold-off airframe keeps the pull it touched with
                    // (`TouchHold`), in the air too: the channel is a 1 s
                    // heartbeat, so it is written every tick it is held.
                    let touch_elev = if pitch_yielded {
                        None
                    } else {
                        touch_hold.elev(gs_kts, !airborne, roll_dt)
                    };
                    // The rollout kept no record of its own: the EGLF and LFPB
                    // weaves had to be reconstructed from the 1 Hz `state`
                    // samples, which is no way to read a control loop.
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({
                            "event": "rollout", "gs": gs_kts, "xtk_m": xtk_m,
                            // The held pull (#166), when one is.
                            "touch_elev": touch_elev,
                            "xtk_rate": xtk_rate, "err": err, "steer": steer,
                            "trim": roll_law.trim(), "psi": psi, "dt": roll_dt,
                            // The sim's yaw rate a declared rollout steer
                            // damps on (#197); null when not read.
                            "yaw_r": yaw_r,
                            "pos": pos_source,
                            // The brakes as written: the symmetric channel and
                            // each wheel's steering channel (issue #166).
                            "brake": brakes.sym, "bl": brakes.left, "br": brakes.right,
                            // The free nose's learnt standing yaw, deg/s^2
                            // (`ground_steer`; 0 on the shared split), and its
                            // tick: the yaw rate asked and read.
                            "btrim": rollout_brakes.trim(),
                            "gsteer": rollout_brakes.last().map(|s| s.json()),
                            // The wheels and the height: a latch on a bounce
                            // flew six seconds of "rollout" in the air, and
                            // only the 3 s `state` samples said so (#166).
                            "ground": !airborne, "radio": radio_ft,
                            "phi": phi_roll, "trk": track_roll,
                            "air_ail": air_ail,
                            "bank_tgt": airborne.then_some(air_bank_tgt).flatten(),
                        }),
                    );
                    writes.set(&thr_name, 0.0).set(TAXI_STEER, steer);
                    if let Some(e) = touch_elev {
                        // Ours, not a hand (the scorer's yoke check).
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_PITCH, e);
                    }
                    // A vacate follows: hand off ROLLING at taxi pace — the
                    // completion of a landing is clearing the runway, and a
                    // dead stop on the strip followed by a re-acceleration was
                    // neither procedure nor comfortable (EGHI, 2026-08-02).
                    // The vacate drive arms the governor from the current
                    // position, which takes a rolling aircraft like any join.
                    if vacate_follows && gs_kts <= VACATE_HANDOFF_KTS {
                        writes.flush(&xp).await;
                        let _ = xp.set_float(TAXI_BRAKE, 0.0).await;
                        let _ = xp.set_float(TAXI_STEER, 0.0).await;
                        if rollout_brake_steer {
                            let _ = tokio::join!(
                                xp.set_float(crate::autotaxi::LEFT_BRAKE, 0.0),
                                xp.set_float(crate::autotaxi::RIGHT_BRAKE, 0.0),
                            );
                        }
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({ "event": "rolling_handoff", "gs": gs_kts }),
                        );
                        break;
                    }
                    if gs_kts > 2.0 {
                        // The steering pair every tick with the stop: the
                        // plugin's channels are one-second heartbeats.
                        // With the lever, the rudder and the column: the
                        // rollout's ONE write round (it was two or three,
                        // 0.48 s a tick against a 300 ms sleep, 2026-09-30).
                        writes.set(TAXI_BRAKE, brakes.sym);
                        if rollout_brake_steer {
                            writes
                                .set(crate::autotaxi::LEFT_BRAKE, brakes.left)
                                .set(crate::autotaxi::RIGHT_BRAKE, brakes.right);
                        }
                        writes.flush(&xp).await;
                        set_panel(&format!("AUTOLAND {} {} ROLLOUT", rwy.icao, rwy.id));
                        pacer.wait(std::time::Duration::from_millis(ROLLOUT_TICK_MS)).await;
                        continue;
                    }
                    // Stopped: squeeze the brakes, free the nosewheel, then
                    // hand the hold to the PARKING brake through the
                    // valve-aware seam, and say what the read-back says. The
                    // squeeze is the plugin's one-second heartbeat: on its own
                    // it let go the moment this loop ended, and at EGSS the
                    // 737 sat on the runway after "Brakes on" with
                    // `parking_brake_ratio` at 0.03 (2026-09-22).
                    writes.flush(&xp).await;
                    let _ = xp.set_float(TAXI_BRAKE, ROLLOUT_BRAKE_HOLD).await;
                    let _ = xp.set_float(TAXI_STEER, 0.0).await;
                    if rollout_brake_steer {
                        let _ = tokio::join!(
                            xp.set_float(crate::autotaxi::LEFT_BRAKE, 0.0),
                            xp.set_float(crate::autotaxi::RIGHT_BRAKE, 0.0),
                        );
                    }
                    // The one stop-and-park (`hand_back_hold`): to under a
                    // knot on the wheel brakes, the held command's tail, then
                    // the parking brake read back. It set the parking brake
                    // at 2 kt with no tail wait (refactor audit 2026-10-04,
                    // duplicates #8). Fail closed: we are on the runway.
                    let valve = crate::aircraft::live_profile().and_then(|p| p.park_brake_valve_set);
                    let held = crate::tools::controls::hand_back_hold(
                        &xp,
                        valve,
                        crate::tools::controls::HoldFor::by("autoland").fail_closed(),
                    )
                    .await
                    .unwrap_or(false);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({
                            "event": "stopped_brakes", "held": held, "valve_set": valve,
                        }),
                    );
                    announce(stopped_line(held).into(), Priority::Warning);
                    break;
                }
                // Cross-track closure rate (nm/min, positive = drifting right),
                // low-pass filtered — the damping term that turns the P-only
                // centreline chase into a crisp intercept-and-rollout. P alone
                // converged asymptotically (the banana join) and S-ed under
                // wind (2026-07-24).
                let xtk_now = cross_track_nm(lat, lon, &rwy);
                let t_now = std::time::Instant::now();
                let dt_s = prev_xtk.map_or(0.0, |(_, t0)| t_now.duration_since(t0).as_secs_f64());
                let raw_rate = match prev_xtk {
                    Some((x0, _)) if (0.1..=3.0).contains(&dt_s) => (xtk_now - x0) / dt_s * 60.0,
                    _ => 0.0,
                };
                prev_xtk = Some((xtk_now, t_now));
                // Low-passed on TIME (`XTK_RATE_TAU_S`), not half per tick.
                let a = 1.0 - (-dt_s.clamp(0.05, 3.0) / XTK_RATE_TAU_S).exp();
                xtk_rate += (raw_rate.clamp(-3.0, 3.0) - xtk_rate) * a;

                // Wind crab, from the reference recipe (heading vs ground track):
                // command a heading whose TRACK aligns with the runway, not the
                // heading itself. P-only tracking left a standing 0.10 nm
                // cross-track under the LXGB crosswind — touchdown in the grass
                // (2026-07-22). LOW-PASSED: the raw read fed straight back to
                // the bug hunted the LEMG 13 final into a balk (2026-09-12,
                // see [`CrabFilter`]). Estimated every tick, applied on final.
                let crab_raw = CrabFilter::raw(psi_r.as_ref().ok().copied(), hpath_r.as_ref().ok().copied());
                let crab = crab_filter.update(crab_raw, dt_s.clamp(0.0, 3.0));

                let raw = guidance_on(
                    lat, lon, alt, radio_ft, gs_kts, ias, &rwy, approach_ias, 25.0, xtk_rate,
                    runway_width_m(&rwy),
                );
                let raw = hold_flare(raw, flaring, radio_ft, gs_kts, 25.0);
                // A missed approach called low is flown by the latch (full power,
                // straight ahead) until clear of the ground — the raw intercept
                // guidance turns back to the FAP immediately, which is a killer
                // at 50 ft (see [`GoAroundLatch`]).
                let along_nm = along_track_nm(lat, lon, &rwy);
                // The arrest keeps the servos it took (`hold_roundout`): a
                // roundout balloon over the band is still the landing, or a
                // balked one, never the autopilot's descent again.
                let held = hold_roundout(
                    raw,
                    arrest_hands,
                    radio_ft,
                    gs_kts,
                    rwy.length_m + along_nm.min(0.0) * M_PER_NM,
                    crate::aircraft::landing_distance_m(),
                );
                let held_now = held.arresting && !raw.arresting;
                let balked_now = held.balk.is_some() && raw.balk.is_none();
                if held_now != roundout_held || balked_now {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "roundout_held", "held": held_now,
                            "balk": held.balk, "radio": radio_ft,
                            "band": arrest_gates_ft(25.0, gs_kts).1}),
                    );
                }
                roundout_held = held_now;
                let raw = held;
                let committed = commit_flare_latched(
                    raw,
                    flaring,
                    flare_committed,
                    radio_ft,
                    gs_kts,
                    25.0,
                    rwy.length_m + along_nm.min(0.0) * M_PER_NM,
                    crate::aircraft::landing_distance_m(),
                    rwy.course_deg,
                );
                let committed_now = committed.phase != raw.phase;
                if committed_now && !flare_committed {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "committed_flare", "radio": radio_ft,
                                           "past_m": (-along_nm * M_PER_NM).max(0.0),
                                           "remaining_m": rwy.length_m + along_nm.min(0.0) * M_PER_NM}),
                    );
                }
                // A commitment the room no longer covers is a go-around.
                if flare_committed && raw.balk.is_some() && !committed_now {
                    let remaining_m = rwy.length_m + along_nm.min(0.0) * M_PER_NM;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "commit_released", "why": raw.balk,
                                           "radio": radio_ft, "remaining_m": remaining_m}),
                    );
                }
                let (raw, refused) =
                    wheels_hold_the_landing(committed, on_ground > 0.5, flaring, rwy.course_deg);
                if let Some(why) = refused {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "balk_refused_on_ground", "why": why,
                                           "radio": radio_ft, "ias": ias, "gs": gs_kts}),
                    );
                }
                // A controller's order beats anything the approach thinks of
                // itself. Watched here because this is the thing that can ACT
                // on it; the watch is armed with the approach above, so only
                // an order given during this approach can trip it.
                // The pilot's order is flown the same way and answered, so
                // the `go_around` tool reads back what the aircraft does.
                if let Some(by) = runtime.orders.take() {
                    let already = runtime.go_around.active();
                    let flown = runtime.go_around.order(on_ground > 0.5);
                    if flown {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "ordered_go_around", "radio": radio_ft,
                                               "dist": raw.dist_nm, "by": by.label()}),
                        );
                    }
                    // Answered for the tower's too: a pilot's order the
                    // same tick is folded into it.
                    use crate::go_around::Answer;
                    crate::go_around::answer(if flown {
                        Answer::Flying
                    } else if already {
                        Answer::Already
                    } else {
                        Answer::OnGround
                    });
                }
                // A final the tower is working but has not cleared: ask
                // again at 1,000 ft, go around at 300 (`UnclearedWatch`).
                // The tower's hold is judged by the monitor on its own; this
                // is the crew never landing uncleared (#155).
                match runtime.uncleared.step(
                    crate::party_line::clearance_state() == Some(crate::tower::State::Inbound),
                    radio_ft,
                    on_ground > 0.5,
                    runtime.go_around.active(),
                ) {
                    Some(UnclearedStep::Ask) => {
                        if runtime.going_around_callsign.is_none() {
                            runtime.going_around_callsign = Some(crate::circuit::crew_callsign(&xp).await);
                        }
                        let cs = runtime.going_around_callsign.clone().flatten();
                        crate::circuit::transmit(crate::circuit::radio_call(
                            &cs,
                            &format!(
                                "final runway {}, request landing clearance",
                                crate::circuit::spell_runway(&rwy.id)
                            ),
                        ));
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "landing_clearance_asked", "rwy": rwy.id,
                                               "radio": radio_ft, "dist": raw.dist_nm,
                                               "held": crate::party_line::landing_hold_armed()}),
                        );
                    }
                    Some(UnclearedStep::GoAround) => {
                        if runtime.go_around.order(on_ground > 0.5) {
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "uncleared_go_around", "rwy": rwy.id,
                                                   "radio": radio_ft, "dist": raw.dist_nm,
                                                   "held": crate::party_line::landing_hold_armed()}),
                            );
                            announce(
                                "No landing clearance. Going around.".into(),
                                Priority::Caution,
                            );
                        }
                    }
                    None => {}
                }
                // Unstabilised at the thousand-foot gate is a go-around, not a
                // quiet re-intercept: the latch only arms itself below the
                // terrain floor, so it is ordered.
                // So is one found unstabilised below the floor on the FIRST
                // tick (a handoff that arrives that way, LEMG 13 2026-09-28):
                // with no final before it, the latch's own transition never
                // sees it.
                if matches!(raw.balk, Some("unstabilised_at_gate" | "unstabilised_below_floor")) {
                    runtime.go_around.order(on_ground > 0.5);
                }
                // THE STABILISED APPROACH (`stable_approach`, #199): by the
                // gate the landing configuration, on speed, not sinking hard
                // and on the path, or a go-around THEN, with height in hand.
                // The lateral half is the guidance's own gate above. EGNX 09
                // (2026-09-30): 255 kt clean and 1,400 fpm down from 1,000 ft
                // to 121 ft radio, on the centreline, so nothing balked.
                let height_aal = alt - rwy.elev_ft;
                let judging = raw.phase == ApproachPhase::Final
                    && !raw.arresting
                    && !runtime.go_around.active()
                    && on_ground < 0.5
                    && along_nm > STABILISED_MIN_ALONG_NM
                    && crate::stable_approach::judged(gate, height_aal);
                let flap_limits_now = crate::aircraft::flap_limits();
                let stable_sample = crate::stable_approach::Sample {
                    height_aal_ft: height_aal,
                    dist_to_aim_nm: along_nm.max(0.0) + TOUCHDOWN_AIM_NM,
                    ias_kts: ias,
                    gs_kts,
                    // What the governor holds in the landing configuration.
                    target_kts: approach_target_kts(
                        approach_ias,
                        1.0,
                        &flap_limits_now,
                        crate::aircraft::config_floor_kts(1.0),
                    ),
                    vvi_fpm: prev_vvi.map(|(v, _)| v),
                    above_path_ft: alt - raw.glidepath_alt_ft,
                    path_ft_per_nm: gp_ft_per_nm(),
                    gear_down,
                    landing_flap: last_flap.is_some_and(|f| f >= LANDING_FLAP_RATIO),
                };
                let now_breaches = if judging {
                    crate::stable_approach::breaches(gate, &stable_sample)
                } else {
                    Vec::new()
                };
                if judging && !runtime.gate_recorded {
                    runtime.gate_recorded = true;
                    crate::blackbox::record(
                        "autoland",
                        stable_record("stable_gate", gate, &stable_sample, &now_breaches, None),
                    );
                }
                if let Some(breach) =
                    runtime.stable_watch.step(runtime.stable_clock.elapsed().as_secs_f64(), &now_breaches)
                {
                    runtime.stable_watch.reset();
                    if runtime.go_around.order_over_gate(on_ground > 0.5, gate.gate_ft()) {
                        runtime.stable_balk = Some(breach.word());
                        runtime.stable_said = Some(breach.spoken());
                        crate::blackbox::record(
                            "autoland",
                            stable_record(
                                "unstabilised",
                                gate,
                                &stable_sample,
                                &now_breaches,
                                Some(breach),
                            ),
                        );
                    }
                }
                if runtime.go_around.active() {
                    runtime.stable_watch.reset();
                    runtime.gate_recorded = false;
                }
                runtime.go_around.note_height_aal(alt - rwy.elev_ft);
                let mut missed = runtime.go_around.update(
                    last_phase,
                    raw.phase,
                    radio_ft,
                    raw.dist_nm,
                    along_nm,
                    join_point_nm(alt, rwy.elev_ft),
                    xtk_now,
                );
                flaring = raw.phase == ApproachPhase::Flare && missed.is_none();
                // The latch lives as long as the flare it committed.
                flare_committed = flaring && (flare_committed || committed_now);
                // THE COURSE REVERSAL. Started by the missed approach's
                // reposition stage, or by an ordinary approach that finds
                // itself steering at a centreline point it would reach flying
                // away from the runway (or called "final" while doing so).
                let track_true = hpath_r
                    .as_ref()
                    .ok()
                    .or(psi_r.as_ref().ok())
                    .copied()
                    .unwrap_or(rwy.course_deg);
                // Set on the tick a pattern begins: its own line says what
                // happens, so the phase change says nothing more.
                let mut reversal_began = false;
                // A refused pattern is not asked for again: its hold flies.
                if reversal.is_none() && !hold.as_ref().is_some_and(|h| h.refused) {
                    // The guidance's own aim: a join point behind only when
                    // the height cannot come off on the way in (#131).
                    let aim_along_nm = if aims_at_join_point(
                        alt - raw.glidepath_alt_ft,
                        along_nm,
                        xtk_now,
                        gs_kts,
                    ) {
                        join_point_nm(alt, rwy.elev_ft)
                    } else {
                        FAP_NM
                    };
                    // A steep, visual-segment approach (LSZS) is never
                    // repositioned by a pattern of our own: that is a valley.
                    // A go-around there flies the PUBLISHED missed approach.
                    // PUBLISHED steep only: a circuit's path steepened over
                    // the ground (#182) is an ordinary field, its reversal
                    // terrain-probed (LEMG 13 SR22 2026-09-29: 3.26 degrees
                    // took the valley branch, no pattern when high on the
                    // join, then no missed approach to fly after the balk).
                    let steep = published_steep() && !missed_procedure_tried;
                    let wanted = match missed {
                        Some(MissedStage::Reposition) => true,
                        Some(MissedStage::Climb) => false,
                        None if steep => false,
                        None => {
                            let needed = raw.phase != ApproachPhase::Flare
                                && radio_ft >= TERRAIN_FLOOR_AGL_FT
                                && crate::reversal::needed(
                                    &rwy,
                                    lat,
                                    lon,
                                    track_true,
                                    aim_along_nm,
                                    raw.phase == ApproachPhase::Final,
                                );
                            // Inside the FAP and pointing down the runway, a
                            // pattern is not the answer to "too high": a crew
                            // GOES AROUND, straight ahead, and the latch's own
                            // reposition follows the climb. Armed at 2.6 nm
                            // 800 ft high, the 737 turned straight onto a
                            // downwind from 1,600 ft at LEMG 13 (2026-09-26).
                            if needed
                                && late_go_around(along_nm, crate::angles::abs_diff_deg(track_true, rwy.course_deg))
                                && runtime.go_around.order(on_ground > 0.5)
                            {
                                crate::blackbox::record(
                                    "autoland",
                                    serde_json::json!({"event": "late_go_around", "dist": raw.dist_nm,
                                        "radio": radio_ft, "above_path_ft": alt - raw.glidepath_alt_ft}),
                                );
                                announce(
                                    "Too high to land from here. Going around.".to_string(),
                                    Priority::Caution,
                                );
                                false
                            } else {
                                needed
                            }
                        }
                    };
                    if wanted && steep {
                        missed_procedure_tried = true;
                        let missed_legs = crate::procedure::take_missed_legs();
                        let vy = crate::aircraft::live_profile()
                            .and_then(|p| p.reference.as_ref().and_then(|r| r.vy_kts))
                            .unwrap_or(approach_ias * 1.3);
                        let flying = match missed_legs {
                            Some(legs) => crate::procedure::start_scoped(
                                base_url.clone(),
                                voice.clone(),
                                crate::procedure::Plan {
                                    name: "the missed approach".to_string(),
                                    legs,
                                    end: crate::procedure::EndAction::Level,
                                    vy_kts: vy,
                                    cruise_ias_kts: None,
                                    transition_altitude_ft: None,
                                    climb_minimums: Vec::new(),
                                    arrival_constraints: Vec::new(),
                                    visual_deg: None,
                                    missed_legs: Vec::new(),
                                    cruise_schedule: Default::default(),
                                },
                                Vec::new(),
                            ),
                            None => false,
                        };
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "missed_procedure", "flying": flying, "radio": radio_ft}),
                        );
                        if flying {
                            gave_up = true;
                            announce(
                                "Flying the published missed approach.".to_string(),
                                Priority::Caution,
                            );
                            break;
                        }
                        // No published missed approach to fly: our own
                        // terrain-probed pattern below (`reversal::plan`,
                        // which climbs straight ahead and HOLDS a height
                        // clear of the ground where no pattern fits). The
                        // old "climbing straight ahead" hand-back left
                        // an unattended SR22 at 805 ft with nobody flying
                        // (LEMG 13 2026-09-29).
                    }
                    if wanted && reversals_flown >= crate::reversal::MAX_REVERSALS {
                        gave_up = true;
                        let thr_dref = throttle_dref();
                        let _ = tokio::join!(
                            xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                            autoflight::set_dial(&xp, Dial::Vs, 0.0),
                        );
                        announce(
                            format!(
                                "I can't get us established for runway {}. Your aircraft. \
                                 The autopilot is holding heading and altitude{}.",
                                crate::circuit::spell_runway(&rwy.id),
                                if gear_down { ", gear is down" } else { "" }
                            ),
                            Priority::Warning,
                        );
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "give_up", "reason": "max_reversals",
                                               "flown": reversals_flown}),
                        );
                        break;
                    }
                    // The pattern is planned over the probed ground (#182,
                    // `reversal::plan`), beside the loop: begun in the
                    // go-around's climb (the reposition will want it), or the
                    // tick it is wanted; a plan over a minute old is stale.
                    let default_platform = rwy.elev_ft + REPOSITION_MIN_AGL_FT;
                    if reversal_plan
                        .as_ref()
                        .is_some_and(|(_, t)| t.elapsed().as_secs_f64() > REVERSAL_PLAN_FRESH_S)
                    {
                        reversal_plan = None;
                    }
                    if (wanted || missed == Some(MissedStage::Climb)) && reversal_plan.is_none() {
                        let task = crate::reversal::plan(
                            XPlaneClient::new(base_url.clone()).with_parents(xp.cancellation()),
                            rwy.clone(),
                            along_nm,
                            xtk_now,
                            gs_kts,
                            default_platform,
                            (lat, lon, track_true, alt),
                        );
                        reversal_plan = Some((tokio::spawn(task), std::time::Instant::now()));
                    }
                    // Waiting on the plan: straight ahead, never lower than
                    // the platform the old rule would have flown.
                    if wanted && hold.is_none() {
                        hold = Some(TerrainHold::waiting(track_true, alt.max(default_platform)));
                    }
                    if !wanted && hold.as_ref().is_some_and(|h| !h.refused) {
                        hold = None;
                    }
                    let ready = wanted
                        && reversal_plan.as_ref().is_some_and(|(h, _)| h.is_finished());
                    let planned = if ready {
                        match reversal_plan.take() {
                            Some((h, _)) => h.await.ok(),
                            None => None,
                        }
                    } else {
                        None
                    };
                    if let Some(plan) = planned {
                        let above_path_ft = alt - raw.glidepath_alt_ft;
                        let mut record = serde_json::json!({"event": "reversal", "side": plan.side,
                            "platform_ft": plan.platform_ft,
                            "xtk_m": xtk_now * M_PER_NM, "dist": raw.dist_nm,
                            "track": track_true, "missed": missed.is_some(),
                            "final_called": raw.phase == ApproachPhase::Final,
                            "why": crate::reversal::why(
                                missed.is_some(),
                                raw.phase == ApproachPhase::Final,
                                above_path_ft,
                            ),
                            "above_path_ft": above_path_ft, "aim_nm": aim_along_nm,
                            "along": along_nm, "ias": ias, "gs": gs_kts});
                        if let (Some(r), Some(p)) = (record.as_object_mut(), plan.record.as_object()) {
                            r.extend(p.clone());
                        }
                        crate::blackbox::record("autoland", record);
                        match (plan.refused.as_deref(), plan.hold_ft) {
                            // No pattern here clears the ground: climb straight
                            // ahead to a height clear of it and hold it on the
                            // autopilot, said with the numbers; the aircraft is
                            // handed over only once it is there.
                            (Some(why), hold_ft) => {
                                let to = hold_ft.unwrap_or_else(|| alt.max(default_platform));
                                hold = Some(TerrainHold::refused(track_true, to));
                                announce(
                                    format!(
                                        "No pattern for runway {} clears the ground: {why}. \
                                         Climbing straight ahead to {:.0} feet and holding it.",
                                        crate::circuit::spell_runway(&rwy.id),
                                        to
                                    ),
                                    Priority::Warning,
                                );
                            }
                            (None, _) => {
                                hold = None;
                                reversals_flown += 1;
                                reversal = Some(crate::reversal::Reversal {
                                    side: plan.side,
                                    leg: crate::reversal::Leg::Outbound,
                                    platform_ft: plan.platform_ft,
                                    high: false,
                                });
                                if missed == Some(MissedStage::Reposition) {
                                    runtime.go_around.hold_reposition();
                                }
                                reversal_began = true;
                                pattern_gs_kts = gs_kts;
                                if matches!(
                                    plan.why,
                                    "switched_side_for_terrain" | "raised_for_terrain"
                                ) {
                                    crate::blackbox::record(
                                        "autoland",
                                        serde_json::json!({"event": "reversal_terrain",
                                            "why": plan.why, "platform_ft": plan.platform_ft}),
                                    );
                                }
                                announce(
                                    format!(
                                        "Repositioning for runway {}: {} downwind at {:.0} feet{}.",
                                        crate::circuit::spell_runway(&rwy.id),
                                        if plan.side < 0.0 { "left-hand" } else { "right-hand" },
                                        (plan.platform_ft / 100.0).round() * 100.0,
                                        match plan.why {
                                            "raised_for_terrain" => ", raised for the high ground",
                                            "switched_side_for_terrain" => ", the other side is high ground",
                                            _ => "",
                                        }
                                    ),
                                    Priority::Advisory,
                                );
                            }
                        }
                    }
                }
                // Fly it: the pattern owns the heading, the platform and the
                // speed until the aircraft is inbound and the ordinary
                // guidance calls a final it is actually pointing down.
                let mut pattern: Option<(f64, crate::reversal::Leg, f64)> = None;
                // An inbound leg that went over the field is over: left
                // alone it flies the runway heading away for ever.
                if reversal.as_ref().is_some_and(|r| crate::reversal::overflown(r, along_nm)) {
                    reversal = None;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "reversal_overflown", "alt": alt,
                            "radio": radio_ft, "dist": raw.dist_nm, "flown": reversals_flown}),
                    );
                }
                // An inbound leg the glidepath has come down to is an
                // approach: the pattern is done, the path takes the descent.
                if reversal.as_ref().is_some_and(|r| {
                    crate::reversal::path_joined(
                        r,
                        along_nm,
                        xtk_now,
                        track_true,
                        rwy.course_deg,
                        alt - raw.glidepath_alt_ft,
                    )
                }) {
                    reversal = None;
                    runtime.go_around.finish_reposition();
                    missed = None;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "reversal_done", "why": "path_joined",
                            "dist": raw.dist_nm, "xtk_m": xtk_now * M_PER_NM,
                            "above_path_ft": alt - raw.glidepath_alt_ft}),
                    );
                }
                if let Some(rev) = reversal.as_mut() {
                    recede_ref_nm = recede_ref_nm.max(raw.dist_nm);
                    let join = join_point_nm(rev.platform_ft, rwy.elev_ft);
                    // Height first: a downwind begun far above the platform
                    // extends until the rest of the pattern can lose it.
                    let high = rev.leg == crate::reversal::Leg::Outbound
                        && crate::reversal::too_high_to_turn(
                            alt,
                            rev.platform_ft,
                            along_nm,
                            join,
                            gs_kts,
                            crate::procedure::descent_fpm(),
                        );
                    if high && !rev.high {
                        let above = ((alt - rev.platform_ft) / 100.0).round() * 100.0;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "downwind_extended", "above_ft": above,
                                               "along_nm": along_nm, "alt": alt}),
                        );
                        announce(
                            format!("We're {above:.0} feet above the pattern. Extending the downwind to lose it."),
                            Priority::Advisory,
                        );
                    }
                    rev.high = high;
                    match crate::reversal::step(
                        rev,
                        &rwy,
                        lat,
                        lon,
                        along_nm,
                        xtk_now,
                        track_true,
                        gs_kts,
                        join,
                        raw.phase == ApproachPhase::Final,
                    ) {
                        Some(h) => pattern = Some((h, rev.leg, rev.platform_ft)),
                        None => {
                            reversal = None;
                            runtime.go_around.finish_reposition();
                            missed = None;
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "reversal_done",
                                    "dist": raw.dist_nm, "xtk_m": xtk_now * M_PER_NM}),
                            );
                        }
                    }
                }
                let ga = missed.is_some();
                // The tower is owed this the moment it happens: it is holding a
                // landing clearance for an aircraft that is no longer landing,
                // and it works the traffic underneath us. Luton 2026-07-28: the
                // autoland balked, climbed away and repositioned over the field
                // for four minutes, and nothing was ever said — the FSM sat in
                // ClearedToLand throughout. Announced ONCE, once the go-around
                // is real: a balk low in the flare can still land (LFBD 11,
                // 2026-09-27), and then it is a landing, cleared, and nothing
                // goes out (see [`go_around_call_due`], `balk_landed`).
                if missed.is_none() {
                    runtime.balk_watch = None;
                } else if runtime.balk_watch.is_none() {
                    // The stabilised gate's word when it ordered this one.
                    runtime.balk_watch = Some(BalkWatch::new(raw.balk.or(runtime.stable_balk.take()), radio_ft));
                }
                if let Some(w) = runtime.balk_watch.as_mut() {
                    w.note(radio_ft, on_ground > 0.5);
                }
                if go_around_call_due(
                    missed.is_some(),
                    runtime.said_going_around,
                    on_ground > 0.5,
                    runtime.balk_watch.as_ref(),
                    radio_ft,
                    prev_vvi.map(|(v, _)| v),
                ) {
                    runtime.said_going_around = true;
                    // The go-around VOIDS the landing clearance, so this
                    // approach owes the tower a fresh final report before it
                    // can be cleared again — the call that earns it.
                    runtime.owe_final_report = true;
                    if runtime.going_around_callsign.is_none() {
                        runtime.going_around_callsign = Some(crate::circuit::crew_callsign(&xp).await);
                    }
                    let cs = runtime.going_around_callsign.clone().flatten();
                    crate::circuit::transmit(crate::circuit::radio_call(&cs, "going around"));
                    // `radio` and `why` are the BALK's (the debrief's
                    // "GO-AROUND at N ft radio: why"); `called_radio` is
                    // where the climb made it real.
                    let w = runtime.balk_watch.unwrap_or(BalkWatch::new(raw.balk, radio_ft));
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "going_around", "radio": w.balk_radio_ft,
                                           "why": w.why, "called_radio": radio_ft,
                                           "low_ft": w.low_ft, "touched": w.touched,
                                           "xtk_m": xtk_now * M_PER_NM,
                                           "dist": raw.dist_nm, "trim": track_trim.value()}),
                    );
                }
                // Back on final after a missed approach: report it. The first
                // approach is reported by whoever set this one up (the
                // circuit's own "final runway ...", or full auto's arrival);
                // the second is nobody's until here, and a tower that never
                // hears from us again holds nothing to clear us with. Said on
                // the way IN — established on final, inside the FAP — because
                // "final" is a position report and it has to be true.
                if runtime.owe_final_report
                    && missed.is_none()
                    && raw.phase == ApproachPhase::Final
                    && raw.dist_nm <= FAP_NM
                {
                    runtime.owe_final_report = false;
                    if runtime.going_around_callsign.is_none() {
                        runtime.going_around_callsign = Some(crate::circuit::crew_callsign(&xp).await);
                    }
                    let cs = runtime.going_around_callsign.clone().flatten();
                    crate::circuit::transmit(crate::circuit::radio_call(
                        &cs,
                        &format!("final runway {}", crate::circuit::spell_runway(&rwy.id)),
                    ));
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "final_report", "rwy": rwy.id,
                                           "dist": raw.dist_nm}),
                    );
                }
                // Cleared to try again: the next balked landing is a new call.
                if missed.is_none() {
                    runtime.said_going_around = false;
                    runtime.spoke_going_around = false;
                }
                // A missed approach (or a pattern) that stops getting anywhere
                // is its own emergency, judged by the ground it makes, never a
                // flat clock (`reversal::MissedWatch`): the stage's measure
                // (feet to climb, the pattern's track to go, an extended
                // downwind's height) stepping at a rate for the airframe's
                // speed, legs sequenced, inside an outer bound from the plan.
                // The flat 600 s gave up on an SR22 on the line, 90 ft under
                // the path it was about to join, on LEMG 13's terrain-raised
                // pattern (iMac 2026-09-30, sortie 1790775809-approach).
                let watched = if reversal.is_some() || hold.is_some() || missed.is_some() {
                    let progress = match (reversal.as_ref(), hold.as_ref()) {
                        (Some(rev), _) => {
                            let join = join_point_nm(rev.platform_ft, rwy.elev_ft);
                            let end = crate::reversal::pattern_end_nm(
                                rev.platform_ft,
                                rwy.elev_ft,
                                gp_ft_per_nm(),
                                join,
                            );
                            let to_go = crate::reversal::to_go_nm(
                                rev,
                                along_nm,
                                xtk_now,
                                pattern_gs_kts,
                                join,
                                end,
                            );
                            let above = alt - rev.platform_ft;
                            crate::reversal::Progress::Pattern {
                                n: reversals_flown,
                                leg: rev.leg,
                                high: rev.high,
                                to_go_nm: to_go,
                                above_ft: above,
                                budget_s: crate::reversal::pattern_budget_s(
                                    to_go,
                                    pattern_gs_kts,
                                    above,
                                    crate::procedure::descent_fpm(),
                                ),
                                gs_kts: pattern_gs_kts,
                            }
                        }
                        (None, Some(h)) if h.refused => {
                            crate::reversal::Progress::Hold { to_go_ft: h.alt_ft - alt }
                        }
                        _ if missed == Some(MissedStage::Climb) => crate::reversal::Progress::Climb {
                            to_go_ft: GO_AROUND_TURN_AGL_FT - radio_ft,
                        },
                        _ => crate::reversal::Progress::Waiting,
                    };
                    let t = watch_epoch.elapsed().as_secs_f64();
                    let w = missed_watch.get_or_insert_with(|| crate::reversal::MissedWatch::start(t));
                    if let Some(stage) = w.note(t, progress) {
                        let (to_go_nm, leg) = match progress {
                            crate::reversal::Progress::Pattern { to_go_nm, leg, .. } => {
                                (Some(to_go_nm), Some(leg.word()))
                            }
                            _ => (None, None),
                        };
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "missed_progress", "stage": stage,
                                "missed": missed.is_some(), "to_go_nm": to_go_nm, "leg": leg,
                                "gs": gs_kts, "pattern_gs": pattern_gs_kts,
                                "budget_s": w.budget_left_s(t), "alt": alt, "radio": radio_ft,
                                "dist": raw.dist_nm}),
                        );
                    }
                    w.verdict(t).map(|v| (v, progress, w.stalled_s(t), w.budget_left_s(t)))
                } else {
                    missed_watch = None;
                    None
                };
                if let Some((verdict, progress, stalled_s, budget_left_s)) = watched {
                    // A give-up must hand back a FLYABLE aircraft. The balked
                    // landing left gear down and landing flap out, and a flat
                    // 0.5 throttle cannot hold level flight dirty: the Baron
                    // decayed 110 to 80 kt at -900 fpm in the thirty seconds
                    // after the old handback (EGHI, 2026-08-02). Climb power,
                    // the VS dial zeroed (the AP is already holding heading),
                    // and the line says what is actually held.
                    gave_up = true;
                    let thr_dref = throttle_dref();
                    let _ = tokio::join!(
                        xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                        autoflight::set_dial(&xp, Dial::Vs, 0.0),
                    );
                    announce(
                        format!(
                            "I can't get us back round for another approach. Your aircraft. \
                             The autopilot is holding heading and altitude{}.",
                            if gear_down { ", gear is down" } else { "" }
                        ),
                        Priority::Warning,
                    );
                    let to_go_nm = match progress {
                        crate::reversal::Progress::Pattern { to_go_nm, .. } => Some(to_go_nm),
                        _ => None,
                    };
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": verdict.reason(),
                            "stage": progress.word(), "to_go_nm": to_go_nm,
                            "stalled_s": stalled_s, "budget_left_s": budget_left_s,
                            "missed": missed.is_some(), "leg": reversal.map(|r| r.leg.word()),
                            "alt": alt, "radio": radio_ft, "dist": raw.dist_nm, "gs": gs_kts}),
                    );
                    break;
                }
                // Runaway backstop: an approach that keeps missing (arrives too
                // fast/high to ever capture) would loop go-arounds forever,
                // accelerating toward Vne. After MAX_GO_AROUNDS, stop trying:
                // level the wings, throttle to a safe climb, and give the
                // aircraft back — a real crew would divert, not keep diving at
                // a runway they can't make (LXGB Twin Otter, 2026-07-26).
                // Too many go-arounds: give up, but only once the climb-away
                // is flown. The third LFLL balk released the aircraft at 95 ft
                // with landing flaps and climb power, "holding altitude": it
                // accelerated at 300 ft AGL to 236 kt through the flap limit
                // (2026-09-17). The latch's Climb stage runs to
                // GO_AROUND_TURN_AGL_FT under full power with the go-around
                // flap; the handback happens there, flaps retracted.
                if runtime.go_around.attempts() > MAX_GO_AROUNDS && missed != Some(MissedStage::Climb) {
                    gave_up = true;
                    let thr_dref = throttle_dref();
                    let _ = tokio::join!(
                        xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                        autoflight::set_dial(&xp, Dial::Vs, 0.0),
                        xp.set_float(FLAP_RATIO, 0.0),
                    );
                    announce(
                        format!(
                            "Unable to land, we've gone around too many times. Climbed away to \
                             {} feet. Your aircraft. The autopilot is holding heading and \
                             altitude, flaps up, gear is down.",
                            (radio_ft / 100.0).round() * 100.0
                        ),
                        Priority::Warning,
                    );
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "max_go_arounds", "attempts": runtime.go_around.attempts()}),
                    );
                    break;
                }
                let g = match missed {
                    Some(stage) => missed_approach_guidance(
                        &rwy,
                        stage,
                        lat,
                        lon,
                        alt,
                        radio_ft,
                        raw.dist_nm,
                        raw.glidepath_alt_ft,
                        ias,
                        approach_ias,
                        // The speed's trend since the last tick's power
                        // law read it (the same pair; nothing written).
                        prev_ias.map_or(0.0, |p| {
                            (ias - p) / prev_ias_t.elapsed().as_secs_f64().clamp(0.1, 2.0)
                        }),
                        crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0)),
                    ),
                    None => raw,
                };
                // The pattern's own command over whichever of those stood.
                let g = match pattern {
                    Some((hdg, leg, platform_ft)) => {
                        let slow = approach_ias - ias > SPEED_FLOOR_MARGIN_KTS;
                        let inbound = leg == crate::reversal::Leg::Inbound;
                        Guidance {
                            target_heading_deg: hdg,
                            target_vs_fpm: crate::reversal::platform_vs_fpm(
                                platform_ft,
                                alt,
                                slow,
                                max_climb_fpm(),
                                crate::procedure::descent_fpm(),
                            ),
                            // The inbound never accelerates back past the
                            // pattern's own speed: at 14.5 nm the schedule
                            // read "clean" and took the 737 from 177 to 222 kt
                            // level before the path (LEMG 13, 2026-10-05).
                            target_ias_kts: if inbound {
                                scheduled_ias(g.dist_nm, approach_ias)
                                    .min(pattern_ias(approach_ias, &crate::aircraft::flap_limits()))
                            } else {
                                pattern_ias(approach_ias, &crate::aircraft::flap_limits())
                            },
                            phase: ApproachPhase::Intercept,
                            balk: None,
                            arresting: false,
                            ..g
                        }
                    }
                    None => g,
                };
                // ...or the hold's (#182): straight ahead at its altitude,
                // while the pattern is planned or because none clears the
                // ground. A refusal hands over only once it is there, the
                // autopilot holding heading and altitude and the governor the
                // speed (`gave_up`): never "your aircraft" low over the hills.
                let slow = approach_ias - ias > SPEED_FLOOR_MARGIN_KTS;
                if pattern.is_some() {
                    hold = None;
                }
                let hold_done = hold.as_mut().is_some_and(|h| h.level_now(alt));
                if hold_done {
                    let h = hold.take().expect("a hold that is done exists");
                    gave_up = true;
                    let _ = autoflight::set_dial(&xp, Dial::Vs, 0.0).await;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "terrain_refused",
                                           "hold_ft": h.alt_ft, "alt": alt, "radio": radio_ft}),
                    );
                    announce(
                        format!(
                            "Level at {:.0} feet, straight ahead, the autopilot holding heading and \
                             altitude{}. Your aircraft.",
                            h.alt_ft,
                            if gear_down { ", gear is down" } else { "" }
                        ),
                        Priority::Warning,
                    );
                    break;
                }
                let g = match (pattern, hold) {
                    (None, Some(h)) => Guidance {
                        target_heading_deg: h.hdg_true,
                        target_vs_fpm: crate::reversal::platform_vs_fpm(
                            h.alt_ft,
                            alt,
                            slow,
                            max_climb_fpm(),
                            crate::procedure::descent_fpm(),
                        ),
                        target_ias_kts: pattern_ias(approach_ias, &crate::aircraft::flap_limits()),
                        phase: ApproachPhase::Intercept,
                        balk: None,
                        arresting: false,
                        ..g
                    },
                    _ => g,
                };
                // The live floor (#182): every leg but the final keeps
                // `circuit::live_floor_ft` over the ground under and 1.5 nm
                // ahead, probed every 2 s: the level legs climb 800 ft clear
                // of it, the base and an intercept only keep the floor. The
                // go-around's climb is its own; the final is the landing's.
                // A steep (visual segment, mountain) approach's intercept is
                // a valley's, flown on its published path: its walls ahead
                // are not ours to climb away from.
                let floor_leg = live_floor_leg(
                    g.phase,
                    missed,
                    pattern.map(|(_, leg, _)| leg),
                    hold.is_some(),
                )
                .filter(|_| {
                    pattern.is_some()
                        || hold.is_some()
                        || missed.is_some()
                        || glidepath_deg() <= 3.0 + 1e-9
                });
                let mut g = g;
                if let Some(leg) = floor_leg {
                    if live_floor_probed.is_none_or(|t| t.elapsed().as_secs_f64() >= LIVE_FLOOR_EVERY_S) {
                        live_floor_probed = Some(std::time::Instant::now());
                        let mut pts = vec![(lat, lon)];
                        pts.extend(crate::procedure::look_ahead_points(
                            lat,
                            lon,
                            track_true,
                            crate::circuit::LIVE_LOOK_NM,
                            crate::circuit::LIVE_LOOK_POINTS,
                        ));
                        if let Some(e) = crate::terrain::max_elevation_ft(&xp, &pts).await {
                            if let Some(floor) = crate::circuit::live_floor_ft(leg, alt, e) {
                                let raised = live_floor.is_none_or(|(f, _)| floor > f);
                                live_floor = Some((
                                    live_floor.map_or(floor, |(f, _)| f.max(floor)),
                                    std::time::Instant::now(),
                                ));
                                if raised {
                                    crate::blackbox::record(
                                        "autoland",
                                        serde_json::json!({"event": "terrain_floor",
                                            "leg": format!("{leg:?}"),
                                            "pattern_leg": pattern.map(|(_, l, _)| l.word()),
                                            "hold": hold.is_some(), "ground_ft": e, "alt": alt,
                                            "radio": radio_ft, "floor_ft": floor}),
                                    );
                                }
                                if !live_floor_said {
                                    live_floor_said = true;
                                    announce(
                                        format!(
                                            "Terrain ahead. Holding {:.0} feet.",
                                            (floor / 50.0).ceil() * 50.0
                                        ),
                                        Priority::Warning,
                                    );
                                }
                            }
                        }
                    }
                } else {
                    live_floor = None;
                }
                // A spell ends with its floor: the next one is said again.
                live_floor = live_floor.filter(|(_, t)| t.elapsed().as_secs_f64() < LIVE_FLOOR_HOLD_S);
                if live_floor.is_none() {
                    live_floor_said = false;
                }
                if let Some((floor, _)) = live_floor {
                    g.target_vs_fpm = g.target_vs_fpm.max(crate::reversal::platform_vs_fpm(
                        floor,
                        alt,
                        slow,
                        max_climb_fpm(),
                        crate::procedure::descent_fpm(),
                    ));
                }
                // The plan's ask (`approach_energy::ask`): level, slow to the
                // speed the path is joined at; on the path, no faster than
                // its own curve. Descent first: the vertical law is not
                // touched. Not in a go-around, a hold or the flare.
                let gate_nm = gate.gate_ft() / gp_ft_per_nm() - TOUCHDOWN_AIM_NM;
                let energy = energy_model
                    .as_ref()
                    .filter(|_| missed.is_none() && hold.is_none() && g.phase != ApproachPhase::Flare)
                    .map(|m| {
                        let (s_nm, path_nm) = energy_geometry(
                            reversal.as_ref().filter(|_| pattern.is_some()).map(|rev| {
                                let join = join_point_nm(rev.platform_ft, rwy.elev_ft);
                                let end = crate::reversal::pattern_end_nm(
                                    rev.platform_ft,
                                    rwy.elev_ft,
                                    gp_ft_per_nm(),
                                    join,
                                );
                                let to_go = crate::reversal::to_go_nm(
                                    rev,
                                    along_nm,
                                    xtk_now,
                                    pattern_gs_kts,
                                    join,
                                    end,
                                );
                                (to_go, end)
                            }),
                            g.dist_nm,
                            alt - g.glidepath_alt_ft,
                            gp_ft_per_nm(),
                            gate_nm,
                        );
                        let ask = crate::approach_energy::ask(m, energy_vapp, s_nm, path_nm, gs_kts);
                        let gate_kts =
                            crate::approach_energy::predicted_gate_kts(m, energy_vapp, ias, s_nm, path_nm);
                        (ask, s_nm, path_nm, gate_kts)
                    });
                let mut g = g;
                // Below the terrain floor the guidance asks for climb energy
                // over the schedule (`CLIMB_ENERGY_BONUS_KTS`): never capped.
                let floor_bonus =
                    pattern.is_none() && g.target_ias_kts > scheduled_ias(g.dist_nm, approach_ias) + 0.5;
                // A new pattern is a new final: planned afresh.
                if reversal_began {
                    energy_cap = None;
                }
                match energy.filter(|_| !floor_bonus) {
                    Some((ask, ..)) => {
                        let kts = g.target_ias_kts.min(ask.kts).max(approach_ias);
                        let kts = energy_cap.map_or(kts, |c| kts.min(c));
                        energy_cap = Some(kts);
                        g.target_ias_kts = kts;
                    }
                    // A go-around, a hold, the flare: the next final plans
                    // afresh.
                    None => energy_cap = None,
                }
                let g = g;
                let outbound = hold.is_some()
                    || pattern.is_some_and(|(_, leg, _)| leg != crate::reversal::Leg::Inbound);
                let trim = track_trim.update(
                    g.phase,
                    g.target_heading_deg,
                    hpath_r.as_ref().ok().copied(),
                    dt_s.clamp(0.0, 3.0),
                );
                let phase_word = match g.phase {
                    ApproachPhase::Intercept => "INTERCEPT",
                    ApproachPhase::Final => "FINAL",
                    ApproachPhase::Flare => "FLARE",
                };
                set_panel(&format!(
                    "AUTOLAND {} {} {:.1}nm {}",
                    rwy.icao, rwy.id, g.dist_nm, phase_word
                ));
                tracing::info!(
                    target: "xvoicelayer::autoland",
                    dist = g.dist_nm, alt, gp_alt = g.glidepath_alt_ft, vs = g.target_vs_fpm,
                    hdg = g.target_heading_deg, ias, radio = radio_ft, phase = ?g.phase,
                    "autoland"
                );
                // The vertical speed actually flown: the watchdog below reads
                // it, and so does the landing arrest, whose dial leads the
                // target through the autopilot's lag (`vs_dial_fpm`). The
                // blackbox keeps both columns: `cmd_vs` is what the law
                // wants, `vs_dial` what the autopilot was told.
                // A failed read is NO measurement, never the command in
                // disguise: the dial gets the bare target that tick, the
                // watchdog does not count it, and the blackbox keeps `null`
                // so the debrief can see the gauge was missing.
                // From the tick's one read (`st`), no round of its own.
                let vvi_read = st.get(F::Vvi);
                let throttle_read = throttle_read
                    .take()
                    .unwrap_or_else(|| Err(anyhow::anyhow!("unread")));
                let servos_read = if engage.confirmed() { None } else { st.opt(F::Servos) };
                // The engage watch (see `ApEngage`): re-press until the
                // servos read on with the sim live, each try on the tape.
                let engage_now = engage_t0.elapsed().as_secs_f64();
                // Airborne only: a servo pressed on the wheels winds the
                // yoke over (`disconnect_autopilot`).
                let es = if on_ground < 0.5 && !rolling_out {
                    engage.step(engage_now, servos_read.filter(|v| v.is_finite()), ias)
                } else {
                    EngageStep::default()
                };
                if es.fire {
                    let ok = crate::ap_engage::press(&xp, &cfg.fd_commands).await;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "ap_engage", "try": es.try_n, "sent": ok,
                            "at_s": engage_now, "servos": servos_read, "ias": ias,
                            "radio": radio_ft}),
                    );
                }
                if es.confirmed_now {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "ap_engaged", "tries": es.try_n,
                            "at_s": engage_now, "held": engage_held, "radio": radio_ft}),
                    );
                    authority_owed = true;
                }
                if es.give_up {
                    gave_up = true;
                    let _ = xp.set_float(&throttle_dref(), GIVE_UP_THROTTLE).await;
                    announce(ENGAGE_GIVE_UP_LINE.into(), Priority::Warning);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "ap_not_engaged",
                            "tries": es.try_n, "at_s": engage_now, "radio": radio_ft}),
                    );
                    break;
                }
                let thr_now = throttle_read.ok().filter(|v| v.is_finite());
                let vvi = match vvi_read {
                    Ok(v) => Some(v),
                    Err(e) => {
                        if !vvi_read_failed {
                            vvi_read_failed = true;
                            tracing::warn!(target: "xvoicelayer::autoland", error = %e, "VSI read failed; flying the bare VS target");
                        }
                        None
                    }
                };
                // The sink's rate of change over the last tick (fpm per
                // second), the arrest's damping term on a slow autopilot.
                let tick_dt = prev_vvi
                    .map(|(_, t0)| t_now.duration_since(t0).as_secs_f64())
                    .filter(|d| (0.02..=3.0).contains(d))
                    .unwrap_or(0.2);
                let vvi_trend = match (vvi, prev_vvi) {
                    (Some(v), Some((p, t0))) => {
                        let dt = t_now.duration_since(t0).as_secs_f64();
                        if (0.1..=3.0).contains(&dt) { (v - p) / dt } else { 0.0 }
                    }
                    _ => 0.0,
                };
                prev_vvi = vvi.map(|v| (v, t_now));
                let arrest_lead = crate::aircraft::arrest_lead();
                let vs_dial = match vvi {
                    Some(v) => vs_dial_fpm(g.target_vs_fpm, v, vvi_trend, g.arresting, arrest_lead),
                    None => g.target_vs_fpm,
                };
                // The servos' pitch AUTHORITY (#181): the arrest takes the
                // servos out, so the check ends there; before it, a dial the
                // autopilot's elevator does not fly is hand flown level by
                // the recovery and the approach abandoned.
                if g.arresting || !engage.confirmed() || es.hold {
                    if g.arresting {
                        crate::climb_watch::disarm_authority();
                        authority_owed = false;
                    }
                } else if authority_owed {
                    if let Some(v) = vvi {
                        crate::climb_watch::arm_authority("autoland", v, vs_dial);
                        authority_owed = false;
                    }
                } else if let Some(crate::climb_watch::Authority::NoAuthority(why)) =
                    crate::climb_watch::feed_authority(vvi, Some(vs_dial))
                {
                    let started = crate::tools::recover::take_over(
                        base_url.clone(),
                        voice.clone(),
                        crate::tools::recover::TakeOver {
                            why: format!("autoland_{why}"),
                            min_safe_ias: crate::aircraft::config_floor_kts(
                                last_flap.unwrap_or(0.0),
                            )
                            .map(|f| f + crate::circuit::RECOVERY_MARGIN_KTS),
                            hold_kts: None,
                            ap_no_pitch: true,
                            hold_ft: None,
                        },
                    );
                    announce("Abandoning the approach.".into(), Priority::Warning);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "ap_no_pitch",
                            "why": why, "recovery_started": started, "radio": radio_ft}),
                    );
                    break;
                }
                // THE ARREST FLIES THE ELEVATOR, not the autopilot's VS dial.
                // Only while actually arresting, and only with both
                // measurements in hand: anywhere else the dial is right and
                // proven. The channel is a 1 s expiring heartbeat, so simply
                // ceasing to write it hands the pitch back by itself.
                // The final's air, for the flare (`FinalAir`): measured on the
                // way down to the gate with the autopilot flying, begun again
                // above it (a go-around's climb, a new pattern).
                if radio_ft > FINAL_AIR_FROM_FT {
                    final_air = FinalAir::default();
                } else if let (ApproachPhase::Final, false, false, Some(v)) =
                    (g.phase, g.arresting, es.hold, vvi)
                {
                    final_air.note(v, g.target_vs_fpm);
                }
                let mut model_out: Option<FlareModelOut> = None;
                // A model that carries the speed leads its bleed (`fpm_per_kt`).
                if let Some(m) = flare_model.as_mut() {
                    m.note_speed(ias, tick_dt);
                }
                // A HOLD-OFF held on from its first contact (`TouchHold`):
                // the wheels, or the radio under TOUCH_RADIO_FT, in the
                // flare. A go-around or a new pattern starts free.
                if missed.is_some() || g.phase == ApproachPhase::Intercept {
                    touch_hold.reset();
                    last_flare_elev = None;
                } else if holds_off
                    && !pitch_yielded
                    && g.phase == ApproachPhase::Flare
                    && touch_hold.note(on_ground > 0.5 || radio_ft < TOUCH_RADIO_FT, last_flare_elev)
                {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "touch_hold", "held": touch_hold.held(),
                            "at": "flare", "radio": radio_ft, "gs": gs_kts, "ias": ias,
                            "vvi": vvi, "on_ground": on_ground > 0.5}),
                    );
                }
                let touch_elev = if touch_hold.touched() && !pitch_yielded {
                    touch_hold.elev(gs_kts, on_ground > 0.5, tick_dt)
                } else {
                    None
                };
                let flare_elev = match (g.arresting, vvi, pitch_yielded) {
                    // Once the pilot has taken the pitch it stays taken for
                    // this approach — the debounce the taxi and takeoff
                    // channels learned: a latch that re-arms is a fight over
                    // a control with a person on the other end.
                    (true, Some(v), false) => match flare_model.as_mut() {
                        Some(m) => {
                            let o = m.step(
                                radio_ft,
                                gs_kts,
                                v,
                                tick_dt,
                                g.phase == ApproachPhase::Flare,
                                final_air.rough(),
                            );
                            model_out = Some(o);
                            Some(o.elev)
                        }
                        None => Some(flare_pitch.step(
                            g.target_vs_fpm,
                            v,
                            vvi_trend,
                            tick_dt,
                            g.phase == ApproachPhase::Flare,
                        )),
                    },
                    // No sink read: no decision. The model keeps time with
                    // what is on the channel (it expires by itself).
                    (true, None, false) if flare_model.is_some() => {
                        if let Some(m) = flare_model.as_mut() {
                            m.coast(tick_dt, radio_ft);
                        }
                        None
                    }
                    // NOBODY IS FLYING: the autopilot is not confirmed and
                    // its servos read off. Level on the elevator until it
                    // is (`ApEngage`, `engage_hold_gains`); the same
                    // channel, the same single write below.
                    (false, Some(v), false) if es.hold => {
                        engage_held = true;
                        flare_pitch.reset();
                        if let Some(m) = flare_model.as_mut() {
                            m.reset();
                        }
                        Some(engage_pitch.step(0.0, v, vvi_trend, tick_dt, false))
                    }
                    _ => {
                        flare_pitch.reset();
                        if let Some(m) = flare_model.as_mut() {
                            m.reset();
                        }
                        None
                    }
                };
                // Held on: the pull it touched with, whatever the flare
                // would have asked.
                let flare_elev = if touch_hold.touched() && !pitch_yielded {
                    touch_elev
                } else {
                    flare_elev
                };
                if flare_elev.is_some() {
                    last_flare_elev = flare_elev;
                }
                // THE ARREST TAKES THE SERVOS. The pitch channel writes the
                // yoke under the joystick override and the autopilot's pitch
                // servo writes it back the same frame: LFPG 09R (2026-09-22)
                // held 0.35 of elevator for three seconds and touched down at
                // 0.1 degrees of pitch and -672 fpm, the sink following the
                // VS dial's own slow shape exactly as it had before the
                // elevator law existed (every 737 landing since it shipped
                // shows the same). The channel is proven with the servos
                // OFF - the takeoff rotates on it - so the roundout gate is
                // where the autopilot comes off: pitch on the elevator
                // channel, the runway track on the roll channel (the
                // takeoff's wings-level law, banking toward the crabbed
                // heading), the power on the governor. Only a go-around
                // hands the servos back and re-engages the modes (a balloon
                // over the band stays the arrest, `hold_roundout`); the
                // rollout latch finds them already off.
                // The last 0.3 nm of the final: the centreline closed in
                // seconds, damped on the drift over the ground (see
                // `short_final_cut_deg`), where the old 80 deg/nm let 1 degree
                // of track build 9 m from 170 ft to the gate.
                let off_course = |a: f64| crate::angles::signed_diff_deg(a, rwy.course_deg);
                let track_off = hpath_r.as_ref().ok().copied().map(off_course);
                let in_short_final = g.phase == ApproachPhase::Final
                    && g.dist_nm <= SHORT_FINAL_NM
                    && !g.arresting;
                let crabbed_true = match (g.phase, in_short_final, track_off) {
                    (ApproachPhase::Final, true, Some(to)) => {
                        let pursuit_bug = g.target_heading_deg + crab + trim;
                        let sf = short_final
                            .get_or_insert_with(|| ShortFinal::new(off_course(pursuit_bug), to));
                        (rwy.course_deg
                            + sf.step(xtk_now * M_PER_NM, to, gs_kts, tick_dt, phi_r.as_ref().ok().copied()))
                            .rem_euclid(360.0)
                    }
                    (ApproachPhase::Final | ApproachPhase::Flare, ..) => {
                        short_final = None;
                        (g.target_heading_deg + crab + trim).rem_euclid(360.0)
                    }
                    (ApproachPhase::Intercept, ..) => {
                        short_final = None;
                        g.target_heading_deg
                    }
                };
                match arrest_hands_step(arrest_hands, g.arresting) {
                    ArrestHands::Take => {
                        let ok = autoflight::press(&xp, Button::Disconnect).await.is_ok();
                        arrest_hands = true;
                        wings = arrest_wings();
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({
                                "event": "arrest_hands", "taken": true, "ok": ok, "radio": radio_ft,
                                // The final's air the flare will be flown for.
                                "air_rms": final_air.rms_fpm(), "rough": final_air.rough(),
                            }),
                        );
                    }
                    ArrestHands::Return => {
                        // The servos back through the one engage door (FD
                        // first, `ap_engage` on the tape). Not a fresh
                        // `ApEngage` watch: its level hold would fight the
                        // balk's climb and its give-up would hand a go-around
                        // over; the VS watchdog below judges what comes back.
                        let ok = crate::ap_engage::press_logged(
                            &xp, &cfg.fd_commands, "autoland", "arrest_return", 1,
                        )
                        .await;
                        autoflight::apply(
                            &xp,
                            &[Button::Heading, Button::VerticalSpeed],
                            "autoland_arrest_return",
                        )
                        .await;
                        let _ = autoflight::set_dial(&xp, Dial::Vs, vs_dial).await;
                        arrest_hands = false;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "arrest_hands", "taken": false, "ok": ok, "radio": radio_ft}),
                        );
                    }
                    ArrestHands::Off | ArrestHands::Keep => {}
                }
                // The wings while the arrest has the servos: a bank that
                // closes the centreline by the TRACK over the runway (see
                // `arrest_bank_target`), the sim's ground track against the
                // course, the smoothed cross-track rate when no track was
                // read. No bank read is no command (the channel expires).
                // The approach's pursuit aimed far down the runway (9 m right
                // at 55 ft, 16 m left at the wheels), and the rollout's
                // heading law after it closed in nine seconds what the flare
                // has eight for (LEMG 13, 2026-09-26).
                let xtk_m_now = xtk_now * M_PER_NM;
                let arrest_rate_now = arrest_rate.update(
                    arrest_prev_xtk.map_or(0.0, |p: f64| (xtk_m_now - p) / tick_dt.max(0.05)),
                    tick_dt.max(0.05),
                );
                arrest_prev_xtk = Some(xtk_m_now);
                let track_now = hpath_r.as_ref().ok().copied();
                let side_now = side_velocity_m_s(track_now, rwy.course_deg, gs_kts)
                    .unwrap_or(arrest_rate_now);
                let phi_now = phi_r.as_ref().ok().copied().unwrap_or(0.0);
                // The bank the ailerons already written will make: on a
                // roll-rate plant the reading is a dead time late.
                let bank_tgt =
                    arrest_bank_target(xtk_m_now, side_now, wings.predicted(phi_now), radio_ft);
                let flare_ail = match (arrest_hands, phi_r.as_ref().ok().copied(), roll_yielded) {
                    (true, Some(phi), false) => Some(wings.step(phi, bank_tgt, tick_dt)),
                    _ => None,
                };
                *TRACKING_ERROR.lock().unwrap_or_else(|e| e.into_inner()) = Some(xtk_now * M_PER_NM);
                crate::blackbox::record("autoland", serde_json::json!({
                    "rwy": format!("{} {}", rwy.icao, rwy.id), "dist": g.dist_nm,
                    "alt": alt, "gp_alt": g.glidepath_alt_ft, "cmd_vs": g.target_vs_fpm,
                    // The energy plan (`approach_energy`): its speed, its
                    // flap, the join speed, the speed it predicts at the
                    // gate from here at idle, the route to the gate and the
                    // path's share of it.
                    "tgt_ias": g.target_ias_kts,
                    "e_kts": energy.map(|(a, ..)| a.kts), "e_flap": energy.and_then(|(a, ..)| a.flap),
                    "e_join": energy.map(|(a, ..)| a.join_kts), "e_gate": energy.map(|(.., k)| k),
                    "e_s": energy.map(|(_, s, ..)| s), "e_path": energy.map(|(_, _, p, _)| p),
                    "vs_dial": vs_dial, "vvi": vvi, "vs_fm": vs_fm, "vvi_trend": vvi_trend, "arresting": g.arresting,
                    "cmd_hdg": g.target_heading_deg, "ias": ias, "radio": radio_ft,
                    // The wheels, so a first contact the rollout latch never
                    // saw (a bounce) is on the tape: the scorecard judges it.
                    "ground": on_ground > 0.5,
                    "phase": format!("{:?}", g.phase), "ga": ga, "crab": crab, "crab_raw": crab_raw,
                    "trim": trim, "xtk_m": xtk_now * M_PER_NM,
                    // "plugin": one frame's position; "split": an older
                    // plugin's separate reads (#157).
                    "pos": pos_source,
                    "flare_elev": flare_elev, "flare_ail": flare_ail, "hands": arrest_hands,
                    // The autopilot not yet confirmed and read off: the
                    // elevator above is the level hold's (`ApEngage`).
                    "ap_hold": es.hold,
                    // The model flare's own numbers: the sink it flew toward,
                    // the height that was taken at, and the sink the
                    // elevator did not explain.
                    "flare_target": model_out.map(|o| o.target_fpm),
                    "flare_ahead": model_out.map(|o| o.ahead_ft),
                    "flare_unexplained": model_out.map(|o| o.unexplained_fpm),
                    // The prediction's own ask before the flare's ratchet:
                    // `flare_elev` over it is the pull held.
                    "flare_ask": model_out.map(|o| o.ask),
                    // A hold-off's pull held from its first contact (#166).
                    "touch_hold": touch_hold.held(),
                    "rough_air": g.arresting.then(|| final_air.rough()),
                    "lever": last_lever,
                    "phi": phi_r.as_ref().ok().copied(),
                    "psi": psi_r.as_ref().ok().copied(), "bug_true": crabbed_true,
                    "short_final": short_final.is_some(),
                    // The arrest's lateral: the track it closes on, the
                    // sideways velocity and the bank it asked for.
                    "trk": track_now, "side_m_s": side_now,
                    "bank_tgt": arrest_hands.then_some(bank_tgt),
                    // Which leg of a reversal is flying: every pattern tick
                    // reports phase Intercept, so without this the outbound,
                    // the base and the inbound are one undifferentiated block
                    // in the log and an overshoot cannot be read back to the
                    // leg that flew it.
                    "leg": pattern.map(|(_, leg, _)| leg.word()),
                    // The hold's altitude (#182): planning, or refused.
                    "hold_ft": hold.map(|h| h.alt_ft),
                    "floor_ft": live_floor.map(|(f, _)| f),
                }));

                // Lateral: heading bug in MAGNETIC (guidance is TRUE — see
                // [`true_to_mag`] for the variation sign trap). On final and in
                // the flare, add the wind crab so the commanded heading flies a
                // runway-aligned TRACK; intercept vectors stay coarse.
                let mag_hdg = true_to_mag(crabbed_true, var);
                // Knob presses, not a raw write — registers on glass (G1000) too.
                // Slew-capped: each press is an HTTP round trip, and the LXGB go-
                // around's 178° flip stalled this loop ~5 s per tick (2026-07-14) —
                // no VS/throttle updates the whole time. 30°/tick keeps a full
                // reversal turning while the loop stays live.
                // The bug, the VS dial and BOTH mode checks in one burst. They
                // were four sequential round trips on the hot path, and at
                // 40-70 ms each that is a quarter of a second of tick time
                // spent waiting — on top of the sleep. Nothing here depends on
                // anything else here: the mode reads decide whether to re-engage
                // AFTER the fact, and a dial written while its mode is off is
                // exactly what the re-engage below is for.
                // The elevator rides in the SAME burst. Written every tick it
                // is wanted (the plugin channel is a 1 s heartbeat, so a
                // change-gate would drop it exactly where it sits still), and
                // simply not written otherwise — which hands the pitch back
                // inside a second with nothing to remember.
                // The ILS: its needles against our path, and the coupled
                // approach where the airframe allows it.
                let on_final = g.phase == ApproachPhase::Final && pattern.is_none() && !ga;
                let needles = ils.as_ref().map(|_| {
                    // From the tick's one read.
                    let (h, v, lv, gv) = (st.get(F::Hdef), st.get(F::Vdef), st.get(F::LocValid), st.get(F::GsValid));
                    crate::ils::Needles {
                        hdef: h.ok(),
                        vdef: v.ok(),
                        loc_valid: lv.is_ok_and(|x| x > 0.5),
                        gs_valid: gv.is_ok_and(|x| x > 0.5),
                    }
                });
                let cpl = match needles {
                    Some(n) => {
                        if let Some(line) = ils_xcheck.step(n, on_final, radio_ft, tick_dt) {
                            crate::blackbox::record(
                                "ils",
                                serde_json::json!({"event": "xcheck", "line": line, "hdef": n.hdef,
                                    "vdef": n.vdef, "xtk_m": xtk_now * M_PER_NM, "radio": radio_ft}),
                            );
                            announce(line, Priority::Caution);
                        }
                        // The airframe's own capture, armed on an intercept
                        // to the localizer from below the beam: the final, a
                        // straight-in heading for it, or a pattern's inbound.
                        let inbound_leg = pattern.is_none_or(|(_, leg, _)| leg == crate::reversal::Leg::Inbound);
                        let toward_loc = ils.as_ref().zip(psi_r.as_ref().ok()).is_some_and(|(i, psi)| {
                            crate::ils::arm_verdict(lat, lon, true_to_mag(*psi, var), i)
                                == crate::ils::ArmVerdict::Now
                        });
                        let before = coupling;
                        let out = coupling.step(
                            crate::ils::CouplingIn {
                                allowed: coupled_ok,
                                inbound: !ga && hold.is_none() && inbound_leg && (on_final || toward_loc),
                                arresting: g.arresting,
                                leaving: ga || hold.is_some() || !inbound_leg,
                                dist_nm: g.dist_nm,
                                needles: n,
                                modes: ap_modes,
                                radio_ft,
                                decouple_below_ft: crate::aircraft::live_profile()
                                    .and_then(|p| p.ils_decouple_agl_ft),
                            },
                            &mut coupling_armed_s,
                            tick_dt,
                        );
                        if out.press_appr || out.disarm {
                            autoflight::press_logged(
                                &xp,
                                Button::Approach,
                                if out.disarm { "autoland_decouple" } else { "autoland_coupling" },
                            )
                            .await;
                        }
                        // A decouple is an INTENT: the mode housekeeping
                        // below presses HDG and V/S back once, from the
                        // tick's read (pressed here too, a toggle went twice).
                        if coupling != before {
                            crate::blackbox::record(
                                "ils",
                                serde_json::json!({"event": "coupling",
                                    "modes": ap_modes.map(|m| m.words()),
                                    "from": format!("{before:?}"),
                                    "to": format!("{coupling:?}"), "hdef": n.hdef, "vdef": n.vdef,
                                    "radio": radio_ft, "dist": g.dist_nm}),
                            );
                            match coupling {
                                crate::ils::Coupling::Coupled => announce(
                                    "Localizer and glideslope captured. The autopilot has the ILS.".into(),
                                    Priority::Advisory,
                                ),
                                crate::ils::Coupling::Abandoned if out.decouple => announce(
                                    "Off the ILS coupling. I'm flying the approach myself.".into(),
                                    Priority::Caution,
                                ),
                                _ => {}
                            }
                        }
                        out
                    }
                    None => crate::ils::CouplingOut {
                        press_appr: false,
                        decouple: false,
                        disarm: false,
                        own_lateral: true,
                        own_vertical: true,
                    },
                };
                // The knob: once per phase or pattern leg and corrected past a
                // deadband (`bug_knob`); the short final and the flare steer
                // it every tick, as before.
                let knob = bug_knob(
                    g.phase,
                    in_short_final || g.arresting,
                    pattern.map(|(_, l, _)| l),
                    missed,
                );
                let knob_set = match (cpl.own_lateral, knob) {
                    (false, _) => {
                        bug_hold.reset();
                        None
                    }
                    (true, Some((key, db))) => {
                        // The pattern's, the reversal's, the join's and the
                        // missed approach's turns go through the one
                        // turn-safe steer (`heading_bug::bug_toward`, audit
                        // #6): never a bug on the far side of a turn in
                        // progress, a big turn led from level by at most
                        // 25 degrees. They had no reversal guard. The knob
                        // drive below does the stepping (30 a tick), so the
                        // steer's own slew is not used.
                        let want = match psi_r.as_ref().ok().copied() {
                            Some(psi)
                                if g.phase == ApproachPhase::Intercept
                                    || pattern.is_some()
                                    || missed.is_some() =>
                            {
                                let hdg_mag = true_to_mag(psi, var);
                                crate::heading_bug::bug_toward(
                                    bug_hold.set().unwrap_or(hdg_mag),
                                    mag_hdg,
                                    hdg_mag,
                                    phi_r.as_ref().ok().copied().unwrap_or(0.0),
                                    360.0,
                                )
                            }
                            _ => mag_hdg,
                        };
                        bug_hold.decide(key, want, knob_t0.elapsed().as_secs_f64(), db)
                    }
                    (true, None) => {
                        bug_hold.reset();
                        Some(mag_hdg)
                    }
                };
                // The mode lights and the yields come from the tick's one
                // read; the VS dial, the column, the ailerons and a fine bug
                // correction are QUEUED and go out with the lever in the
                // tick's one write round below. A yield is read as it always
                // was, only on a tick that writes its channel; the latch now
                // comes from the tick's read, a frame before this tick's
                // write, and the plugin drops the override on its own frame
                // either way (the write that follows a yield is ignored).
                let hdg_st = st.get(F::HdgStatus);
                let vs_st = st.get(F::VsStatus);
                if cpl.own_vertical {
                    autoflight::stage_dial(&mut writes, Dial::Vs, vs_dial);
                }
                let yield_r = match flare_elev {
                    Some(e) => {
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_PITCH, e);
                        st.get(F::PitchYield)
                    }
                    None => Ok(0.0),
                };
                let roll_yield_r = match flare_ail {
                    Some(a) => {
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_ROLL, a);
                        st.get(F::RollYield)
                    }
                    None => Ok(0.0),
                };
                // A keyed setting (once per phase or leg) is driven and READ
                // BACK as before, beside the write round; the every-tick
                // steering of the short final and the flare is one queued
                // write from the bug the tick read (`bug_step_from`), pressed
                // only past three degrees.
                let bug_drive = match (knob, knob_set) {
                    (Some(_), Some(h)) => Some(h),
                    (None, Some(h)) => {
                        use crate::tools::autopilot::{bug_step_from, BugStep};
                        match bug_step_from(st.opt(F::Bug), h) {
                            BugStep::Leave => None,
                            BugStep::Set(v) => {
                                autoflight::stage_dial(&mut writes, Dial::HeadingBug, v);
                                None
                            }
                            BugStep::Drive => Some(h),
                        }
                    }
                    (_, None) => None,
                };
                // The pilot pulled or pushed: the plugin has already dropped
                // the override this frame. Stop commanding and let the dial
                // fly what is left — the VS path is still running underneath.
                if yield_r.is_ok_and(|v| v > 0.5) {
                    if !pitch_yielded {
                        pitch_yielded = true;
                        crate::flight::note_pilot_takeover("autoland: yoke");
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "pitch_yield", "radio": radio_ft}),
                        );
                        announce(
                            "You have the pitch. I'm holding the wings and the power.".into(),
                            Priority::Caution,
                        );
                    }
                    flare_pitch.reset();
                    if let Some(m) = flare_model.as_mut() {
                        m.reset();
                    }
                }
                // Likewise the roll: the plugin dropped that override the
                // frame the hand moved, and the elevator keeps flying.
                if roll_yield_r.is_ok_and(|v| v > 0.5) && !roll_yielded {
                    roll_yielded = true;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "roll_yield", "radio": radio_ft}),
                    );
                    announce(
                        "You have the roll. I'm on the elevator and the power.".into(),
                        Priority::Caution,
                    );
                }
                // Mode housekeeping is the autopilot's while it flies. With
                // the arrest on the channels a mode press is at best nothing
                // and at worst (an MCP that engages the master with the
                // mode) the servo back on the yoke mid-flare.
                // A coupled ILS's captured modes are the autopilot's:
                // re-engaging HDG or VS would drop the localizer or the
                // glideslope it just captured.
                // The owner turns the intent into presses (`autoflight::
                // reconcile`): HDG and V/S on a difference only, nothing on a
                // failed read, and never over what a coupled ILS hands the
                // airframe (#81: the old presses here dropped the 737's
                // captured localiser and glideslope at the next tick).
                if !arrest_hands {
                    use autoflight::{Lateral, ModeRead, Vertical, Want};
                    let coupled = matches!(
                        coupling,
                        crate::ils::Coupling::Armed
                            | crate::ils::Coupling::LocCaptured
                            | crate::ils::Coupling::Coupled
                    );
                    let want = Want {
                        lateral: Some(if coupled { Lateral::Coupled } else { Lateral::Heading }),
                        vertical: Some(if coupled { Vertical::Coupled } else { Vertical::Vs }),
                    };
                    let presses = autoflight::reconcile(
                        want,
                        ModeRead {
                            hdg_status: hdg_st.as_ref().ok().copied(),
                            vs_status: vs_st.as_ref().ok().copied(),
                            modes: ap_modes,
                        },
                    );
                    autoflight::apply(&xp, &presses, "autoland").await;
                    if presses.contains(&Button::VerticalSpeed) {
                        // The engage syncs the dial to the CURRENT rate, so the
                        // rate we want has to be written again after it.
                        let _ = autoflight::set_dial(&xp, Dial::Vs, vs_dial).await;
                    }
                }

                // Speed: hold the approach speed with the SAME governor the copilot
                // uses for speed hold (speedhold::throttle_adjust) — a P+D law whose
                // derivative term adds power while the airspeed is still UNWINDING,
                // before the error grows, with extra authority on the slow side. The
                // old loop here was P-only (0.004/kt, no trend), so it only reacted
                // after speed was already gone and dribbled power in too slowly —
                // the nose came up holding level, speed bled to a stall, the AP quit
                // ("lose speed, lose AP", seen live at LERL/XLE000N). Profile-led flare power.
                let now = std::time::Instant::now();
                let dt_s = (now - prev_ias_t).as_secs_f64().clamp(0.1, 2.0);
                let trend_kts_s = prev_ias.map_or(0.0, |p| (ias - p) / dt_s);
                prev_ias = Some(ias);
                prev_ias_t = now;
                if g.phase != ApproachPhase::Flare || missed.is_some() {
                    flare_power.reset();
                }
                let mut ballooned = false;
                let mut power_held = false;
                let thr = if missed == Some(MissedStage::Climb) {
                    // The balked landing itself is firewall, not a governor
                    // holding approach speed — the LXGB miss flew its last 20
                    // seconds at the governor's idea of "on speed" with the
                    // ground coming up. ONLY the climb: full power belongs to
                    // the seconds of getting away from the runway, not to the
                    // minutes of repositioning that follow. Left firewalled
                    // across the whole missed approach (the first flight of the
                    // sticky version, Luton 2026-07-28) the MD-82 accelerated to
                    // 284 KIAS on the outbound leg and was still gaining.
                    // ...and only until the climb is ESTABLISHED: then it
                    // flies the go-around speed (`missed_approach_throttle`).
                    // EGNX 09 (2026-09-30): firewalled from 150 ft to the
                    // turn at 800 ft radio, the clean 737 went 250 to 309 kt,
                    // and 355 before the lever came back (#199).
                    let limits = crate::aircraft::flap_limits();
                    let flap = last_flap.unwrap_or(0.0);
                    let target = missed_approach_speed_kts(
                        approach_ias,
                        flap,
                        &limits,
                        crate::aircraft::config_floor_kts(GO_AROUND_FLAP),
                        crate::aircraft::config_floor_kts(flap),
                        alt,
                    );
                    Some(match thr_now {
                        Some(t) => missed_approach_throttle(
                            t,
                            ias,
                            trend_kts_s,
                            target,
                            radio_ft,
                            vvi,
                            crate::aircraft::approach_thrust_min(),
                        ),
                        None => 1.0,
                    })
                } else if touch_hold.touched() {
                    // A hold-off held on (`TouchHold`): the lever closed,
                    // never the balloon's approach thrust. Each iMac SR22
                    // hop had 0.265 back on at 3 to 4 ft (LEMG 13
                    // 1790702352, 1790702623, 2026-09-29).
                    Some(0.0)
                } else if let Some(hold) = g
                    .phase
                    .eq(&ApproachPhase::Flare)
                    .then(|| {
                        balloon_power(
                            vvi,
                            approach_thr,
                            radio_ft > arrest_gates_ft(25.0, gs_kts).1,
                            cfg.flare_power.is_some(),
                        )
                    })
                    .flatten()
                {
                    // A balloon at idle only bleeds the speed the settle
                    // needs: hold the approach's own thrust until it sinks.
                    ballooned = true;
                    Some(hold)
                } else if g.phase == ApproachPhase::Flare {
                    match cfg.flare_power {
                        Some(power) => match (thr_now, vvi) {
                            (Some(throttle), Some(vs)) => flare_power.step(
                                power,
                                throttle,
                                vs,
                                dt_s,
                                radio_ft,
                                Some(ias),
                                Some(retard_reference_kts(
                                    approach_ias,
                                    crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0)),
                                )),
                            ),
                            _ => None, // No measurement: no guessed power and no ramp advance.
                        },
                        // An undeclared airframe retains the conservative
                        // legacy idle flare. Airframes whose energy response
                        // is known declare the continuous law above.
                        None => Some(0.0),
                    }
                } else {
                    // The target is only as safe as the flaps actually out
                    // (`config_floor_kts`), and the lever has a floor of its
                    // own on final and a ceiling-only rule low in a
                    // reposition (`approach_power_floor`).
                    let floor = crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0));
                    let target = approach_target_kts(
                        g.target_ias_kts,
                        last_flap.unwrap_or(0.0),
                        &crate::aircraft::flap_limits(),
                        floor,
                    );
                    let adj = crate::tools::speedhold::throttle_adjust_live(target - ias, trend_kts_s);
                    // In the roundout the lever only ever comes back. The
                    // speed law read the roundout's own deceleration as
                    // "five knots slow" and walked the 737's lever from 0.32
                    // to 0.55 at 30 ft over EGCC 23L; the flare then held
                    // that thrust (sink over `hold_sink_fpm`) and the
                    // aircraft climbed away from two feet at 155 kt
                    // (2026-09-19). The floor still stands underneath.
                    // ...and, where the airframe declares it, from the
                    // short final's hold height on: a lever raised in the
                    // seconds before the release spools into the roundout
                    // with no trim left against it (`short_final_power_held`).
                    let held = short_final_power_held(
                        g.phase,
                        g.dist_nm,
                        radio_ft,
                        gs_kts,
                        cfg.flare_power.and_then(|p| p.hold_from_s),
                        ias,
                        trend_kts_s,
                        retard_reference_kts(approach_ias, floor),
                    );
                    power_held = held;
                    let adj = if g.arresting || held { adj.min(0.0) } else { adj };
                    // A roundout that FLOATS takes the thrust off, as the
                    // flare's float does: the arrest is ours, not yet the
                    // Flare, and the speed law holds the approach's lever
                    // into a level-off and a climb (`roundout_float_lever`).
                    let float_lever = match (arrest_hands && g.arresting, thr_now, vvi) {
                        (true, Some(throttle), Some(v)) => roundout_float_lever(
                            throttle,
                            v,
                            vvi_trend,
                            dt_s,
                            cfg.flare_power.map_or(DEFAULT_ROUNDOUT_RETARD_PER_S, |p| {
                                p.retard_max_per_s.unwrap_or(p.reduction_per_second)
                            }),
                        ),
                        _ => None,
                    };
                    let min_thr = approach_power_floor(
                        g.phase,
                        radio_ft,
                        target - ias,
                        crate::aircraft::approach_thrust_min(),
                        last_flap.is_some_and(|f| f >= LANDING_FLAP_RATIO),
                    )
                    .max(climb_power_floor(
                        g.phase,
                        g.target_vs_fpm,
                        crate::aircraft::approach_thrust_min(),
                    ));
                    float_lever.or_else(|| thr_now.map(|throttle| (throttle + adj).clamp(min_thr, 1.0)))
                };
                if g.phase != ApproachPhase::Flare && missed.is_none() {
                    if let Some(t) = thr {
                        approach_thr = Some(t);
                    }
                }
                if thr.is_some() {
                    last_lever = thr;
                }
                // The tick's ONE write round: the lever, the VS dial, the
                // column, the ailerons and the bug, together (they were up to
                // four rounds, 2026-09-30), a keyed bug drive beside it.
                if let Some(throttle) = thr {
                    writes.set(&thr_name, throttle);
                }
                let (done, knob_read) = tokio::join!(writes.flush(&xp), async {
                    match bug_drive {
                        Some(h) => {
                            Some(crate::tools::autopilot::drive_heading_bug_step(&xp, h, 30.0).await)
                        }
                        None => None,
                    }
                });
                if let (Some((key, _)), Some(h)) = (knob, knob_set) {
                    bug_hold.took(knob_read);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "heading_set", "key": key, "hdg": h,
                            "read": knob_read, "phase": format!("{:?}", g.phase),
                            "radio": radio_ft}),
                    );
                }
                let throttle_written = thr.map(|_| done.ok(&thr_name) == Some(true));
                // Whether the lever TOOK the last tick's write (an add-on
                // re-driving it is caught this way): this tick's read against
                // that command. It was a second round trip after the write
                // (`throttle_after`), skipped in the flare; the next tick's
                // read says the same thing a tick later, for no round.
                let lever_took = last_thr_written
                    .zip(thr_now)
                    .map(|(c, r)| (c - r).abs() < LEVER_TOOK_TOL);
                last_thr_written = if throttle_written == Some(true) { thr } else { None };
                if throttle_written == Some(false) {
                    flare_power.reset();
                }
                crate::blackbox::record("autoland", serde_json::json!({
                    "event": "power", "phase": format!("{:?}", g.phase),
                    "throttle_before": thr_now, "throttle_command": thr,
                    "lever_took": lever_took, "write_ok": throttle_written,
                    "progressive_flare": cfg.flare_power.is_some(), "dt": dt_s,
                    // Which key moved the flare's lever this tick (issue
                    // #76): entry, height, time, slow, float or timed; a
                    // climb's approach thrust is "balloon" (issue #146: the
                    // last key's word hid it).
                    "retard": (g.phase == ApproachPhase::Flare)
                        .then(|| if touch_hold.touched() {
                            Some("touched")
                        } else if ballooned {
                            Some("balloon")
                        } else {
                            flare_power.last_key.map(RetardKey::word)
                        })
                        .flatten(),
                    "vvi": vvi, "radio": radio_ft, "ias": ias,
                    // The short final's hold (`short_final_power_held`).
                    "held": power_held,
                }));

                // Config: gear down inside 6 nm; flaps by distance AND vertical
                // state (full only once on the path — see [`flap_schedule`]).
                // Both wait for the speed the placard allows (the profile's
                // VLO and per-detent Vfe): a fast handoff slows first and
                // configures as the limits open up. Written only when the
                // target changes, so the setting isn't re-asserted against the
                // pilot every tick.
                // ...or with the plan's half flap: its deceleration was fitted
                // with the gear down from there (`approach_energy`).
                let plan_flap = energy.and_then(|(a, ..)| a.flap);
                if !gear_down
                    && !outbound
                    && (g.dist_nm < 6.0 || plan_flap.is_some_and(|f| f >= 0.5 - 1e-6))
                    && gear_may_extend(ias, crate::aircraft::gear_limit_kts())
                {
                    let _ = xp.set_float(GEAR_HANDLE, 1.0).await;
                    gear_down = true;
                }
                // The landing's own checklist items, with the gear: ground
                // spoilers armed and the autobrake, where the airframe has
                // them. Asked again while the descent still has the
                // speedbrake out. (The Monitor list named both on every A330
                // flight; nothing had ever set them.)
                if !landing_config_done && g.dist_nm < 6.0 && !ga {
                    let cfg = crate::takeoff_config::set_for_landing(&xp).await;
                    landing_config_done = !cfg.deferred;
                    if landing_config_done {
                        LANDING_CONFIGURED.store(true, Ordering::SeqCst);
                    }
                    if let Some(line) = crate::takeoff_config::line("Landing config", &cfg) {
                        announce(line, Priority::Advisory);
                    }
                }
                // Go-around flap: half, whatever the schedule says — full flap
                // won't climb, and clean this low gives the sink away.
                // A CLOSE downwind is flown at half flap too. `outbound` also
                // names a distant reversal/reposition, which stays clean until
                // it enters the normal six-mile configuration area (#211).
                // ...once the speed flies it (`go_around_flap_due`): the
                // half flap's floor, not the full flap's.
                let wanted = if ga
                    && last_flap.is_some_and(|f| f > 0.5 + 1e-6)
                    && !go_around_flap_due(ias, crate::aircraft::config_floor_kts(0.5))
                {
                    last_flap.unwrap_or(0.5)
                } else if ga {
                    0.5
                } else {
                    let sched = approach_leg_flap_schedule(
                        g.dist_nm,
                        alt - g.glidepath_alt_ft,
                        outbound,
                    );
                    // The plan's configuration for its speed, never less
                    // than the schedule's; the landing flap only on or above
                    // the path (the LERL energy trap, `flap_schedule`).
                    let sched = plan_flap.map_or(sched, |f| {
                        plan_flap_wanted(f, alt - g.glidepath_alt_ft).max(sched)
                    });
                    approach_flap_wanted(sched, last_flap)
                };
                let limits = crate::aircraft::flap_limits();
                let flap = flap_kept_flying(
                    flap_held_within_limits(wanted, ias, &limits, last_flap),
                    last_flap,
                    ias,
                    &limits,
                    ga,
                    retract_speed_kts,
                );
                if last_flap != Some(flap) {
                    last_flap = Some(flap);
                    let _ = xp.set_float(FLAP_RATIO, flap).await;
                }

                // Watchdog: the AP not FLYING the commanded vertical speed is how
                // the LEMI approach dove 2,300 fpm into terrain with vs_cmd=0 and
                // no callout. If actual VS diverges hard for ~5 s, re-arm the VS
                // mode and tell the pilot — this is exactly when a hand on the
                // yoke matters.
                let vs_err = vvi.map(|v| v - g.target_vs_fpm);
                // ...and never inside the arrest, where the autopilot has
                // no vertical to hold: a watchdog re-engaging the servos at
                // fifty feet would be the fight this change removes.
                // ...nor before the engage is confirmed: `ApEngage` owns the
                // press until then, and the aircraft is held level meanwhile.
                let not_held = g.phase != ApproachPhase::Flare
                    && !g.arresting
                    && engage.confirmed()
                    // A captured glideslope is the airframe's vertical, not our dial.
                    && cpl.own_vertical
                    && vs_err.is_some_and(|e| vs_not_held(e, prev_vs_err, tick_dt));
                prev_vs_err = vs_err;
                if not_held {
                    let was_s = vs_diverged_s;
                    vs_diverged_s += tick_dt;
                    if was_s < VS_DIVERGED_WARN_S && vs_diverged_s >= VS_DIVERGED_WARN_S {
                        // The mode AND the master: an AP kicked off by the
                        // airframe's own protection (the A330's high-speed
                        // law at Vmo) answers to neither a dial nor a mode
                        // command until the servos are back on.
                        // FD first (`ap_engage`): the watchdog's own clock
                        // judges whether it held.
                        crate::ap_engage::press_logged(
                            &xp, &cfg.fd_commands, "autoland", "vs_watchdog", 1,
                        )
                        .await;
                        autoflight::press_logged(&xp, Button::VerticalSpeed, "autoland_vs_watchdog").await;
                        announce(vs_not_held_line(g.target_vs_fpm).into(), Priority::Warning);
                    }
                    // Said once and then flown for twenty-two minutes: the
                    // A330 climbed from 1,900 ft to FL350 on a -1,200 fpm
                    // dial with the loop still "on approach" and the aircraft
                    // 90 nm from the runway (EGPH 06, 2026-09-20). A warning
                    // that changes nothing is not a watchdog. Give up with a
                    // flyable aircraft and say what is held.
                    if vs_diverged_s >= VS_DIVERGED_GIVE_UP_S {
                        gave_up = true;
                        let thr_dref = throttle_dref();
                        let _ = tokio::join!(
                            xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                            autoflight::set_dial(&xp, Dial::Vs, 0.0),
                        );
                        announce(
                            "Autopilot is still not holding the descent. Abandoning the approach. \
                             Your aircraft — the vertical speed dial is zero and the heading bug is set."
                                .into(),
                            Priority::Warning,
                        );
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "give_up", "reason": "vs_divergence",
                                               "diverged_s": vs_diverged_s, "vvi": vvi, "cmd_vs": g.target_vs_fpm}),
                        );
                        break;
                    }
                } else {
                    vs_diverged_s = 0.0;
                }
                // Flying AWAY from the runway is not an approach. The same
                // sortie: an arm 3.6 nm from the threshold, then 90 nm out and
                // opening, phase still Intercept. A missed approach flies
                // outbound by design; anything else past the arm distance plus
                // a margin is handed back before it becomes a diversion.
                // Measured from as far out as this approach has legitimately
                // been, never while a pattern is flying its downwind: armed
                // over the runway in a go-around (0.9 nm) the limit was 5.9 nm,
                // a 737's turn back needs 6.5, and the guard fired on the tick
                // the turn inbound began (EGTE, 2026-09-21).
                if missed.is_none()
                    && reversal.is_none()
                    && hold.is_none()
                    && g.dist_nm > recede_ref_nm + RECEDING_GIVE_UP_NM
                {
                    gave_up = true;
                    let thr_dref = throttle_dref();
                    let _ = tokio::join!(
                        xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                        autoflight::set_dial(&xp, Dial::Vs, 0.0),
                    );
                    announce(
                        format!(
                            "We're {:.0} miles from runway {} and opening. Abandoning the approach. \
                             Your aircraft — the vertical speed dial is zero.",
                            g.dist_nm,
                            crate::circuit::spell_runway(&rwy.id)
                        ),
                        Priority::Warning,
                    );
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "receding",
                                           "dist_nm": g.dist_nm, "armed_at_nm": rwy.dist_nm,
                                           "ref_nm": recede_ref_nm}),
                    );
                    break;
                }

                if last_phase != Some(g.phase) {
                    let prev = last_phase;
                    last_phase = Some(g.phase);
                    last_arresting = g.arresting;
                    // The landing light for the landing, at the final: the
                    // exterior-lights rule lights it lined up and puts it out
                    // at the vacate, and nothing lit it for the ARRIVAL. The
                    // BEFORE LANDING cross-check read it off on the LEMG 31
                    // final (2026-09-12).
                    if g.phase == ApproachPhase::Final {
                        let _ = crate::tools::lights::landing(&xp, true).await;
                    }
                    let line = match g.phase {
                        // Dropping back to Intercept from Final/Flare is a
                        // go-around, not a fresh intercept — say so. Low it's
                        // the latch flying (straight ahead); high it's the
                        // normal turn back to re-intercept.
                        ApproachPhase::Intercept
                            if matches!(prev, Some(ApproachPhase::Final | ApproachPhase::Flare)) =>
                        {
                            if ga {
                                // Said at once: the power is coming and the
                                // pilot must know why. Corrected if the
                                // wheels take it (`balk_landed`).
                                runtime.spoke_going_around = true;
                                "Going around. Full power, climbing straight ahead."
                            } else {
                                dropped_final_line(
                                    along_nm,
                                    reversal_began,
                                    alt - g.glidepath_alt_ft,
                                )
                                .unwrap_or("")
                            }
                        }
                        ApproachPhase::Intercept => "Intercepting the final approach.",
                        ApproachPhase::Final => "On final. Gear and flaps as the speed allows.",
                        ApproachPhase::Flare if cfg.flare_power.is_some() => "Over the threshold, flaring, easing the power back.",
                        ApproachPhase::Flare => "Over the threshold, flaring, power to idle.",
                    };
                    // A go-around the stabilised gate ordered says why.
                    let line = match runtime.stable_said.take() {
                        Some(why) if ga && line.starts_with("Going around") => {
                            format!("Not stabilised, {why}. Going around.")
                        }
                        _ => line.to_string(),
                    };
                    if !line.is_empty() {
                        announce(line, Priority::Caution);
                    }
                }

                // (Touchdown + rollout-to-stop is handled at the top of the loop,
                // latched by `rolling_out`, before the approach guidance.)
                pacer.wait(std::time::Duration::from_millis(tick_ms(last_phase, last_arresting))).await;
            }
            // A newer loop (the clearance re-arm above, or any later arm) has
            // already set ACTIVE for itself; clearing it here would report
            // "landed or gone around" to whoever waits on it.
            // "YOUR AIRCRAFT" WITH A SPEED HELD. Every give-up wrote a flat
            // 0.75 lever for the pilot to find: right for a piston dragging
            // gear and flap, and on a level 737 it is an acceleration with no
            // end (152 kt at the hand-back, 240 forty seconds later, 346 and
            // ten overspeed warnings before anyone had the picture, EGTE
            // 2026-09-21). The lever write stays as the first second's power;
            // the governor then holds a speed the configuration allows, and
            // "my controls" or a spoken speed takes it from there.
            if gave_up && lease.current() && !xp.cancelled() {
                let hold = hand_back_speed_kts(
                    approach_ias,
                    last_flap.unwrap_or(0.0),
                    &crate::aircraft::flap_limits(),
                );
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "hand_back", "speed_kts": hold, "flap": last_flap}),
                );
                announce(format!("I'm holding {hold:.0} knots for you."), Priority::Advisory);
                crate::tools::speedhold::arm_flight(base_url.clone(), voice.clone(), hold);
            }
            // The flag, the panel and the landing configuration go with the
            // run's lease when the thread ends (a newer loop's are its own).
        });
    });
}

/// The runway END a clearance heard DURING the approach moves us to, if any:
/// an arrival-shaped clearance (told to join, or cleared to land) naming a
/// different end than the one being flown, not already looked up. A
/// departure state's runway belongs to the departure; a blank or matching
/// ident changes nothing. The caller still checks the end is at THIS field.
///
/// Never during a missed approach: the go-around itself moves the tower
/// from `ClearedToLand` to `Inbound`, and the ident published with that state
/// is the runway IN USE, which on a wind dead across the strip had just
/// flipped. Five seconds into the EGTE go-around, at 60 ft, "Tower cleared us
/// for runway two six" replaced the climb-away with a fresh approach to the
/// other end; nobody had cleared anything (2026-09-21). A missed approach is
/// flown to its end, and the tower's answer to the next final report is where
/// a new end comes from.
///
/// ...unless the tower's go-around ORDER itself named the end
/// (`expected`, "expect runway one three", `GoAroundWatch::expected_runway`),
/// and the published clearance agrees: that is a re-sequence, taken once the
/// climb away is flown (the caller passes `expected` only then). LEMG,
/// 2026-09-29 (sortie 1790668215-full_auto): sent round off 30 and
/// re-sequenced to 13 at t+649, the reposition flew a right-hand pattern for
/// 30 from t+661 and learnt of 13 only at t+984, 5.5 minutes later. Pure.
pub fn clearance_switch(
    flying: &str,
    state: Option<crate::tower::State>,
    cleared: Option<&str>,
    already_checked: Option<&str>,
    going_around: bool,
    expected: Option<&str>,
) -> Option<String> {
    use crate::tower::State::{ClearedToLand, Inbound};
    if !matches!(state, Some(ClearedToLand | Inbound)) {
        return None;
    }
    let c = cleared?.trim();
    if going_around && !expected.is_some_and(|e| e.trim().eq_ignore_ascii_case(c)) {
        return None;
    }
    if !clearance_differs(flying, c) {
        return None;
    }
    if already_checked.is_some_and(|a| a.trim().eq_ignore_ascii_case(c)) {
        return None;
    }
    Some(c.to_string())
}

/// Why the runway autoland arms on is (or is not) a change of the mission's
/// arrival runway, for `full_auto::retarget_arrival_runway`: the tower's
/// clearance moved it at the arm ("clearance"), the forced end was flown as
/// handed down ("arm", no change), or nothing forced it and the track picked
/// ("self_pick"). Pure. A missing forced end fails the arm before this runs.
pub fn armed_runway_why(forced: Option<&str>, flying: &str, from_clearance: bool) -> &'static str {
    if from_clearance {
        "clearance"
    } else if forced.is_some_and(|f| f.trim().eq_ignore_ascii_case(flying.trim())) {
        "arm"
    } else {
        "self_pick"
    }
}

/// Is an owed readback the LANDING one? The tower phrases what it waits for
/// (`Tower::suggested_readback`); a hold or line-up echo is never an
/// approach's to make.
pub fn landing_readback(owed: &str) -> bool {
    let o = owed.trim_start().to_ascii_lowercase();
    o.starts_with("cleared to land") || o.starts_with("cleared touch and go")
}

/// Does the standing clearance name a different runway END than the plan?
/// Idents compare trimmed and case-blind ("12" vs "12 "), a missing or blank
/// clearance never differs.
pub fn clearance_differs(planned_id: &str, cleared_id: &str) -> bool {
    let c = cleared_id.trim();
    !c.is_empty() && !c.eq_ignore_ascii_case(planned_id.trim())
}

/// Time constant of the wind-crab estimate the final's lateral law feeds
/// forward. A wind-correction angle changes over minutes (wind shear down the
/// last thousand feet); the RAW heading-minus-track read changes every tick.
/// LEMG 13, 2026-09-12: the raw value was added straight to the heading bug
/// on final, so every nose swing the autopilot made came back as a bug move
/// the same tick, unity gain, and the loop hunted. The blackbox has the
/// signature: crab standard deviation 0.5° on the intercept (where nothing
/// fed it back), then 1.2, 1.8, 2.5, 3.1, 3.7, 4.0° down the final, a ±9°
/// swing with a ~13 s period at the numbers, a standing 35 m offset the
/// short-final law could not close, and the lateral balk gate at 74 ft.
/// At 15 s the 13 s swing comes through at about a seventh of its amplitude.
pub const CRAB_TAU_S: f64 = 15.0;

/// The wind crab actually commanded: heading minus ground track, low-passed
/// with [`CRAB_TAU_S`]. Fed EVERY tick (intercept included) so it is settled
/// when the final starts. A missing read holds the last estimate rather than
/// yanking the nose by the whole angle; a filter that has never seen a read
/// says zero.
#[derive(Debug, Default, Clone, Copy)]
pub struct CrabFilter {
    est: Option<f64>,
}

impl CrabFilter {
    pub const fn new() -> Self {
        Self { est: None }
    }

    /// Raw heading-minus-track (degrees, positive = nose right of track),
    /// `None` when either read is missing. Clamped to ±15°: beyond that it
    /// is a turn or a slip, not a wind angle.
    pub fn raw(psi_true: Option<f64>, hpath_true: Option<f64>) -> Option<f64> {
        let (psi, hpath) = (psi_true?, hpath_true?);
        Some(crate::angles::signed_diff_deg(psi, hpath).clamp(-15.0, 15.0))
    }

    /// Advance by one tick of `dt_s` seconds and return the crab to command.
    pub fn update(&mut self, raw: Option<f64>, dt_s: f64) -> f64 {
        match (raw, self.est) {
            (Some(r), None) => {
                self.est = Some(r);
                r
            }
            (Some(r), Some(e)) => {
                let a = (dt_s / CRAB_TAU_S).clamp(0.0, 1.0);
                let e = e + a * (r - e);
                self.est = Some(e);
                e
            }
            (None, e) => e.unwrap_or(0.0),
        }
    }
}

/// Time constant (s) of the final's track-error trim: how long a steady
/// difference between the track the final asks for and the track flown takes
/// to be integrated out. Slow against the autopilot's heading response (a few
/// seconds) and the crab filter (15 s), so the three never chase each other.
pub const TRACK_TRIM_TAU_S: f64 = 30.0;
/// The most trim the integrator may carry, degrees. A bias bigger than this
/// is a broken autopilot, not something to trim.
pub const TRACK_TRIM_MAX_DEG: f64 = 5.0;
/// A track error beyond this is a turn in progress, not a bias: not integrated.
/// It was 10, and a handoff still converging is a turn in progress: LEMG 31
/// (sortie 1790584715-full_auto, 2026-09-28) the 737 came off the circuit's
/// intercept tracking 303 to 309 for a 311.7 runway, the trim learnt +0.78
/// degrees from those six seconds of roll-out, and with the autopilot
/// flying its bug to a tenth of a degree the final then sat 30 to 32 m right
/// of the centreline from 1.4 nm to 0.3, the trim cancelling the pursuit's
/// 0.75 degree cut; LEMG 13 (1790591256) learnt -0.57 the same way and sat
/// 50 m left. The biases it exists for were 1.1 to 1.9 degrees.
const TRACK_TRIM_ERR_CAP_DEG: f64 = 3.0;
/// A bias is learnt only from a track that has been STEADY this long, s:
/// turning under [`TRACK_TRIM_STEADY_DPS`] (smoothed over
/// [`TRACK_TRIM_RATE_TAU_S`]) for [`TRACK_TRIM_SETTLE_S`]. The 3 degree cap
/// above still let the last three degrees of every circuit's roll-out in:
/// LEMG 30 A330 (sortie 1790710769-full_auto, 2026-09-29) was handed over
/// tracking 3.6 degrees off, flew straight on for a second and a half, then
/// rolled out at 0.2 to 0.65 degrees a second; the trim learnt -0.51 in ten
/// seconds with no bias in the air (calm, the aircraft on its bug), and the
/// final that had reached 4.4 m sat 0.3 to 0.5 degrees left of the course,
/// 17 m off at 0.3 nm, and went around at 99 ft (`off_centreline_low`). The
/// other three circuits flown that evening learnt 0.37 to 0.61 the same way
/// and drifted 6 to 14 m. Settled finals turn under 0.1 degrees a second
/// (p90 0.02 to 0.09 over the four); the biases this exists for held for
/// whole finals. 0.15 and 5 s passed the replays too, and left the replayed
/// crosswind finals (`the_final_on_a_held_knob_keeps_the_line`) 0.4 m
/// further off at the gate.
pub const TRACK_TRIM_STEADY_DPS: f64 = 0.25;
/// Time constant of the smoothing on that track rate, s: a gust's single
/// tick is not a turn, eight seconds of roll-out are.
const TRACK_TRIM_RATE_TAU_S: f64 = 1.5;
/// ...and how long the track must have been steady, s: the handoff's own
/// straight second or two before autoland's first bank is not a bias.
pub const TRACK_TRIM_SETTLE_S: f64 = 3.0;

/// Integral trim on the final's TRACK error, in degrees added to the heading
/// bug alongside the wind crab.
///
/// The crab loop commands a heading whose track aligns with the runway by
/// measuring heading minus track — the wind. It cannot see a bias that sits
/// between the BUG and the heading: an autopilot that settles two degrees
/// left of its bug produces a track two degrees left of the one asked for,
/// and both proportional laws on final (the fixed-lead pursuit, then the
/// 80°/nm short-final cut) only ever ask for a cut, so they equilibrate
/// where the cut equals the bias — 0.6 nm of lead times tan 2° is 39 m,
/// 2°/80 is 46 m. LEMG 13, 2026-09-12: the 172's heading sat 1.1° left of
/// the bug through the whole intercept with no crab in the loop and 1.9°
/// left down both finals, the position 33 to 37 m left of the centreline
/// from 1.3 km out, steady, in both approaches, and the lateral gate under
/// 100 ft (37 m) balked both at 74 and 40 ft. Two go-arounds; the pilot
/// landed by hand. This integrates the difference between the track the
/// guidance wants and the track flown, so whatever the cause of the bias
/// (autopilot law, servo trim, a variation error), the final converges on
/// the centreline instead of parking beside it. Final only; held through
/// the flare; reset on anything else so a missed approach never carries a
/// stale correction into the next attempt.
#[derive(Debug, Default, Clone, Copy)]
pub struct TrackTrim {
    deg: f64,
    /// The last track read on final, the smoothed rate it turns at (`None`
    /// until two reads have measured one) and how long it has been steady.
    last_trk: Option<f64>,
    rate_dps: Option<f64>,
    steady_s: f64,
}

impl TrackTrim {
    pub const fn new() -> Self {
        Self { deg: 0.0, last_trk: None, rate_dps: None, steady_s: 0.0 }
    }

    /// Has the track been steady (under `steady_dps`) long enough to learn a
    /// bias from? Measures the rate from the read before; the first read
    /// measures nothing.
    fn settled(&mut self, h: f64, dt_s: f64, steady_dps: f64) -> bool {
        let prev = self.last_trk.replace(h);
        let Some(p) = prev.filter(|_| dt_s > 1e-3) else {
            self.steady_s = 0.0;
            return false;
        };
        let raw = crate::angles::signed_diff_deg(h, p) / dt_s;
        let r = match self.rate_dps {
            Some(r) => r + (dt_s / TRACK_TRIM_RATE_TAU_S).clamp(0.0, 1.0) * (raw - r),
            None => raw,
        };
        self.rate_dps = Some(r);
        if r.abs() <= steady_dps {
            self.steady_s += dt_s;
        } else {
            self.steady_s = 0.0;
        }
        self.steady_s >= TRACK_TRIM_SETTLE_S
    }

    /// Advance by one tick: `wanted_track_deg` is the guidance heading on
    /// final (the runway course plus the cut, i.e. the TRACK it wants),
    /// `hpath_true` the sim's ground track. Returns the trim to add.
    pub fn update(
        &mut self,
        phase: ApproachPhase,
        wanted_track_deg: f64,
        hpath_true: Option<f64>,
        dt_s: f64,
    ) -> f64 {
        self.update_capped(
            phase,
            wanted_track_deg,
            hpath_true,
            dt_s,
            TRACK_TRIM_ERR_CAP_DEG,
        )
    }

    /// [`TrackTrim::update`] with the turn-in-progress cap given (the replay
    /// tests fly the old 10 degrees against the new).
    pub(crate) fn update_capped(
        &mut self,
        phase: ApproachPhase,
        wanted_track_deg: f64,
        hpath_true: Option<f64>,
        dt_s: f64,
        cap_deg: f64,
    ) -> f64 {
        self.update_gated(
            phase,
            wanted_track_deg,
            hpath_true,
            dt_s,
            cap_deg,
            Some(TRACK_TRIM_STEADY_DPS),
        )
    }

    /// ...and the steady-track gate given (`None`: the old rule, the cap
    /// alone, for the replays of what flew).
    pub(crate) fn update_gated(
        &mut self,
        phase: ApproachPhase,
        wanted_track_deg: f64,
        hpath_true: Option<f64>,
        dt_s: f64,
        cap_deg: f64,
        steady_dps: Option<f64>,
    ) -> f64 {
        match phase {
            ApproachPhase::Final => {
                if let Some(h) = hpath_true {
                    let err = crate::angles::signed_diff_deg(wanted_track_deg, h);
                    let steady = match steady_dps {
                        Some(r) => self.settled(h, dt_s, r),
                        None => true,
                    };
                    if err.abs() <= cap_deg && steady {
                        self.deg = (self.deg + err * dt_s / TRACK_TRIM_TAU_S)
                            .clamp(-TRACK_TRIM_MAX_DEG, TRACK_TRIM_MAX_DEG);
                    }
                }
            }
            ApproachPhase::Flare => {}
            ApproachPhase::Intercept => *self = Self::new(),
        }
        self.deg
    }

    /// The trim currently applied.
    pub fn value(&self) -> f64 {
        self.deg
    }
}

/// Feet of altitude per nm along a 3° glidepath (tan 3° × 6076 ft/nm ≈ 318).
/// Public because the circuit descends its base leg onto the same path — a
/// pattern that hands over 500 ft below it has already lost the approach.
pub const GLIDEPATH_FT_PER_NM: f64 = 318.0;

/// This approach's glidepath angle, degrees: 3 unless an approach handed a
/// steeper one down for its visual segment (`set_next_glidepath_deg`; LSZS
/// RNAV 03's MAP is 996 ft above the threshold two miles out, 4.7 degrees).
/// Set at every arm, so no approach inherits another's.
/// In a test build both live per THREAD: the LSZS visual-segment test arms a
/// 4.5 degree path, and every other test landing at the same moment flew it
/// (the balloon test failed one run in three, 2026-09-24). One landing per
/// process is the app's truth; one per test is the tests'.
/// The angle carries WHO handed it down: `true` for a published visual
/// segment (the follower at a MAP, with its missed approach stashed), `false`
/// for a circuit's path steepened over the ground (#182). Only the first is
/// a valley our own pattern must never fly into; the second is an ordinary
/// field whose go-around the terrain-probed reversal flies (LEMG 13 SR22
/// circuit 2026-09-29: a 3.26 degree circuit path was taken for LSZS, the
/// go-around found no published missed approach and said "Your aircraft"
/// at 805 ft to nobody).
#[cfg(not(test))]
static GLIDEPATH_DEG: std::sync::Mutex<(f64, bool)> = std::sync::Mutex::new((3.0, false));
/// The stabilised gate's blackbox record (`stable_gate` at the crossing,
/// `unstabilised` when it orders a go-around): every number the rule read.
fn stable_record(
    event: &str,
    gate: crate::stable_approach::GateKind,
    s: &crate::stable_approach::Sample,
    now: &[crate::stable_approach::Breach],
    tripped: Option<crate::stable_approach::Breach>,
) -> serde_json::Value {
    use crate::stable_approach as sa;
    serde_json::json!({
        "event": event, "gate": gate.word(), "gate_ft": gate.gate_ft(),
        "height_aal": s.height_aal_ft, "dist_aim_nm": s.dist_to_aim_nm,
        "ias": s.ias_kts, "target": s.target_kts, "gs": s.gs_kts,
        "vvi": s.vvi_fpm,
        "sink_limit": sa::sink_limit_fpm(s.target_kts, s.ias_kts, s.gs_kts, s.path_ft_per_nm),
        "above_path_ft": s.above_path_ft,
        "path_tol_ft": sa::path_tolerance_ft(s.dist_to_aim_nm),
        "gear": s.gear_down, "landing_flap": s.landing_flap,
        "breaches": now.iter().map(|b| b.word()).collect::<Vec<_>>(),
        "why": tripped.map(|b| b.word()),
        "verdict": if now.is_empty() { "stable" } else { "unstable" },
    })
}

/// The stabilised gate the NEXT arm judges its final at, consumed there:
/// the circuit hands down a visual one, every other caller flies the
/// instrument gate (`stable_approach`, #199).
static NEXT_GATE: std::sync::Mutex<Option<crate::stable_approach::GateKind>> =
    std::sync::Mutex::new(None);

/// Hand the next autoland arm a visual gate (a circuit's final).
pub fn set_next_gate(kind: crate::stable_approach::GateKind) {
    *NEXT_GATE.lock().unwrap_or_else(|e| e.into_inner()) = Some(kind);
}

/// Take the handed-down gate for this arm, the instrument gate without one.
fn arm_gate() -> crate::stable_approach::GateKind {
    NEXT_GATE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .take()
        .unwrap_or_default()
}

/// The angle the NEXT arm takes, consumed there.
#[cfg(not(test))]
static NEXT_GLIDEPATH_DEG: std::sync::Mutex<Option<(f64, bool)>> = std::sync::Mutex::new(None);
#[cfg(test)]
thread_local! {
    static GLIDEPATH_DEG_T: std::cell::Cell<(f64, bool)> = const { std::cell::Cell::new((3.0, false)) };
    static NEXT_GLIDEPATH_DEG_T: std::cell::Cell<Option<(f64, bool)>> = const { std::cell::Cell::new(None) };
}
/// The steepest path the landing will fly.
pub const MAX_GLIDEPATH_DEG: f64 = 5.5;

#[cfg(not(test))]
fn next_glidepath(set: Option<Option<(f64, bool)>>) -> Option<(f64, bool)> {
    let mut g = NEXT_GLIDEPATH_DEG.lock().unwrap_or_else(|e| e.into_inner());
    match set {
        Some(v) => {
            *g = v;
            v
        }
        None => g.take(),
    }
}
#[cfg(test)]
fn next_glidepath(set: Option<Option<(f64, bool)>>) -> Option<(f64, bool)> {
    NEXT_GLIDEPATH_DEG_T.with(|c| match set {
        Some(v) => {
            c.set(v);
            v
        }
        None => c.take(),
    })
}
#[cfg(not(test))]
fn live_glidepath(set: Option<(f64, bool)>) -> (f64, bool) {
    let mut g = GLIDEPATH_DEG.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(v) = set {
        *g = v;
    }
    *g
}
#[cfg(test)]
fn live_glidepath(set: Option<(f64, bool)>) -> (f64, bool) {
    GLIDEPATH_DEG_T.with(|c| {
        if let Some(v) = set {
            c.set(v);
        }
        c.get()
    })
}

/// Hand the next autoland arm a glidepath angle for a PUBLISHED visual
/// segment (the follower does, at a MAP).
pub fn set_next_glidepath_deg(deg: Option<f64>) {
    next_glidepath(Some(deg.map(|d| (d, true))));
}

/// Hand the next autoland arm a circuit's path steepened over the ground
/// (#182): flown at its angle, but not a valley ([`published_steep`]).
pub fn set_next_circuit_glidepath_deg(deg: Option<f64>) {
    next_glidepath(Some(deg.map(|d| (d, false))));
}

/// Drop the gate and angle handed down for an arm that never flew.
fn drop_handed_down() {
    let _ = arm_gate();
    let _ = next_glidepath(None);
}

/// Take the handed-down angle for this arm (3 without one), clamped.
fn arm_glidepath() -> f64 {
    let (deg, published) = next_glidepath(None)
        .filter(|(d, _)| d.is_finite())
        .map_or((3.0, false), |(d, p)| (d.clamp(3.0, MAX_GLIDEPATH_DEG), p));
    live_glidepath(Some((deg, published && deg > 3.0 + 1e-9))).0
}

fn glidepath_deg() -> f64 {
    live_glidepath(None).0
}

/// Is this approach a PUBLISHED steep visual segment (LSZS), the one kind no
/// pattern of our own may reposition? A circuit's steepened path is not.
fn published_steep() -> bool {
    let (deg, published) = live_glidepath(None);
    published && deg > 3.0 + 1e-9
}

/// Feet of path per nautical mile for this approach (318 at 3 degrees, the
/// constant every law was tuned on, returned exactly).
pub fn gp_ft_per_nm() -> f64 {
    let deg = glidepath_deg();
    if (deg - 3.0).abs() < 1e-9 {
        GLIDEPATH_FT_PER_NM
    } else {
        6076.12 * deg.to_radians().tan()
    }
}

/// tan of this approach's path angle (the 3 degree constant returned exactly).
fn gp_tan() -> f64 {
    let deg = glidepath_deg();
    if (deg - 3.0).abs() < 1e-9 {
        GLIDEPATH_TAN
    } else {
        deg.to_radians().tan()
    }
}

/// The glidepath correction, fpm per foot of path error, sized to close the
/// error over a fixed DISTANCE of final rather than a fixed time. 1.2 fpm/ft
/// is a 50 s time constant: 0.9 nm on a 172 at 65 kt, but 2.4 nm on a 737 at
/// 170 kt, whose last mile is 22 s long. Captured from below at 1.6 nm, the
/// 737 levelled, started down late, and arrived over LFLL 35L's numbers 51 ft
/// high with a correction of 86 fpm to fix it (2026-09-17). Scaled by
/// groundspeed over the 172's, floored at the old gain so every slow airframe
/// flies the law it always did, capped so the gain never outruns the VS
/// mode's own lag.
pub fn glidepath_gain_fpm_per_ft(gs_kts: f64) -> f64 {
    GLIDEPATH_GAIN_FPM_PER_FT
        * (gs_kts / GLIDEPATH_GAIN_REF_KTS).clamp(1.0, GLIDEPATH_GAIN_MAX_MULT)
}
const GLIDEPATH_GAIN_FPM_PER_FT: f64 = 1.2;
const GLIDEPATH_GAIN_REF_KTS: f64 = 65.0;
const GLIDEPATH_GAIN_MAX_MULT: f64 = 3.0;
/// Aim the glidepath at a touchdown point ~300 m (≈1,000 ft) PAST the threshold,
/// so the path crosses the numbers at a standard ~50 ft threshold-crossing
/// height and touches in the touchdown zone — not planted on the leading edge.
/// The aim point was the threshold itself, so the 3° path targeted the runway
/// EDGE at zero height (2026-07-24). Lateral tracking still anchors on the
/// threshold — it's the same centreline, just a farther aim along it — and the
/// flare still triggers off actual radio height, so it simply engages a little
/// farther in, over the touchdown zone.
pub const TOUCHDOWN_AIM_NM: f64 = 300.0 / M_PER_NM;
/// Final approach point: intercept the extended centreline this far out (nm).
const FAP_NM: f64 = 5.0;
/// Above the path by this much the guidance still flies the straight-in and
/// dives onto the path; a join point behind was the only answer past it.
const CAPTURE_ABOVE_FT: f64 = 500.0;
/// The share of the path law's catch-up rate a straight-in is planned on.
const STRAIGHT_IN_CATCH_UP_SHARE: f64 = 0.8;

/// Can the height above the path come off on the way in, straight in? The
/// catch-up the path law flies over its path's own sink
/// ([`path_descent_cap_fpm`]) for the minutes to the stabilised gate, on a
/// final the pursuit can intercept from here (`circuit::interceptable`).
/// LEMG 12 (sortie 1790583663-approach, #131): re-armed from 13's final
/// 3.5 km beside 12's, the aircraft was 513 ft over the path at 7.6 nm; the
/// 500 ft rule put the join point 10.8 nm out behind it, and a ten minute
/// reversal began where a steeper descent onto an intercept was a minute
/// and a half. Pure.
pub fn straight_in_losable(above_path_ft: f64, along_nm: f64, xtk_nm: f64, gs_kts: f64) -> bool {
    let gate_nm = STABILISED_GATE_AGL_FT / gp_ft_per_nm() - TOUCHDOWN_AIM_NM;
    let run_nm = along_nm - gate_nm;
    if run_nm <= 0.0 || !crate::circuit::interceptable(xtk_nm * M_PER_NM, along_nm * M_PER_NM) {
        return false;
    }
    let gs = gs_kts.max(60.0);
    let nominal = -(gs / 60.0) * gp_ft_per_nm();
    let catch_up_fpm = nominal - path_descent_cap_fpm(nominal);
    let minutes = run_nm / gs * 60.0;
    above_path_ft <= CAPTURE_ABOVE_FT + catch_up_fpm * minutes * STRAIGHT_IN_CATCH_UP_SHARE
}

/// Does the guidance aim at the join point (out along the centreline, behind
/// an aircraft inside it) rather than fly the straight-in? Only when the
/// height above the path cannot come off on the way in
/// ([`straight_in_losable`]). The reversal's own aim asks the same, so a
/// pattern is flown only when a re-intercept is not flyable. Pure.
pub fn aims_at_join_point(above_path_ft: f64, along_nm: f64, xtk_nm: f64, gs_kts: f64) -> bool {
    above_path_ft > CAPTURE_ABOVE_FT
        && !straight_in_losable(above_path_ft, along_nm, xtk_nm, gs_kts)
}
/// Can a final still be made from `xtk_nm` off the centreline with `along_nm`
/// to run — will the pursuit's steepest cut close the offset, with margin,
/// before the threshold? The circuit asks exactly this before it hands over
/// (`circuit::interceptable`); below the terrain floor autoland asks it of
/// itself. A 370 m offset at 0.8 nm passes (the pursuit flies that out); the
/// Gibraltar handoff's 1,660 m at 1.5 nm does not. See the balk in [`guidance`].
/// The gate applies on the approach side with this much still to run: closer
/// in, the flare gate and the pursuit's own short-final law own the question
/// (and past the threshold the along-track is negative, which the intercept
/// sum would read as "can't", over a landing that is already happening).
const STABILISED_MIN_ALONG_NM: f64 = 0.3;
/// Below this on final the approach is stabilised laterally or it is a
/// go-around (see the gate in [`guidance`]).
pub const STABILISED_GATE_AGL_FT: f64 = 1000.0;

pub fn stabilised_xtk(xtk_nm: f64, along_nm: f64) -> bool {
    crate::circuit::interceptable(xtk_nm * M_PER_NM, along_nm * M_PER_NM)
}

/// Will the final's pursuit have the offset inside `limit_m` by the
/// threshold ([`crate::reversal::predicted_residual_m`], the stabilised
/// gate's own time model)? The question below the terrain floor, where
/// "interceptable" is true of any offset a landing could still be saved from
/// and of many it could not.
pub fn lands_on_the_pavement(xtk_nm: f64, along_nm: f64, gs_kts: f64, limit_m: f64) -> bool {
    crate::reversal::predicted_residual_m(xtk_nm, along_nm, gs_kts) <= limit_m
}
/// Airspeed this far below the approach speed triggers speed protection (nose
/// down to regain it), before the decay reaches a stall and disconnects the AP.
/// Approach speed is already ~1.3×Vs, so this margin still stays well above stall.
const SPEED_FLOOR_MARGIN_KTS: f64 = 7.0;

/// Clean manoeuvring speed for the intercept, as a multiple of the airframe's
/// approach speed. The two scale together across every airframe we fly, which
/// is why this is a RATIO and not another profile field to thread through five
/// callers: the SF50's 87 kt approach gives 126 (it was flying 125 when the
/// circuit handed off), the 172's 65 gives 94, and the MD-82's 140 gives 203 —
/// within a few knots of the 200 its profile already declares as the pattern
/// speed. A profile figure can always override this later.
const CLEAN_APPROACH_RATIO: f64 = 1.45;
/// Start slowing to the approach speed here (nm), reaching it at the FAP.
/// Commanding the approach speed from the moment autoland arms is what flew an
/// SF50 at 92 kt NINE MILES OUT with the gear up (Southampton, 2026-07-30) —
/// the airframe's own gear warning was right and we were wrong. Gear comes
/// down at 6 nm, inside this ramp, so it extends well below Vlo.
const DECEL_START_NM: f64 = 10.0;

/// The clean manoeuvring speed for an airframe with this approach speed — what
/// autoland flies outside the deceleration ramp, and what the procedure
/// follower holds down a straight-in so the FAF handoff arrives at a speed
/// autoland expects.
pub fn clean_approach_ias(approach_ias: f64) -> f64 {
    approach_ias * CLEAN_APPROACH_RATIO
}

/// May the gear come down at this speed? The profile's VLO/VLE gates it; a
/// profile with no figure never gates. Autoland used to lower the gear the
/// moment it was inside 6 nm whatever the speed: 195 kt on a Baron placarded
/// 152 (2026-09-11).
pub fn gear_may_extend(ias_kts: f64, vlo_kts: Option<f64>) -> bool {
    vlo_kts.is_none_or(|v| ias_kts <= v)
}

/// The most flap the schedule's `wanted` ratio allows at this speed: the
/// deepest detent at or below `wanted` whose placard Vfe the speed respects.
/// `limits` is the profile's detents as `(ratio, Vfe)`, retracted first; a
/// detent with no Vfe never gates, and an empty list (no flap profile) hands
/// `wanted` straight back. The setting is held back, not skipped: as the speed
/// bleeds off, the schedule's target becomes legal and goes out then.
pub fn flap_within_limits(wanted: f64, ias_kts: f64, limits: &[(f64, Option<f64>)]) -> f64 {
    if limits.is_empty() {
        return wanted;
    }
    limits
        .iter()
        .filter(|(ratio, vfe)| *ratio <= wanted + 1e-6 && vfe.is_none_or(|v| ias_kts <= v))
        .map(|(ratio, _)| *ratio)
        .fold(0.0, f64::max)
}

/// A detent goes out only with this much room under its placard...
const FLAP_EXTEND_MARGIN_KTS: f64 = 5.0;
/// ...and one already out comes back in only this far over it.
const FLAP_RETRACT_MARGIN_KTS: f64 = 8.0;

/// [`flap_within_limits`] with a memory. The bare rule is a comparator on a
/// noisy airspeed: the 737's flap 40 went out at 161 kt under a 162 placard,
/// came back to 10 at 162, and went out again at 900 ft on final (EGTE,
/// 2026-09-21). A crew that touches the placard slows down; it does not cycle
/// the flaps. Extending wants margin under the limit, a detent already out is
/// held until the speed is properly over it, and a schedule that asks for
/// LESS (the go-around's half) is obeyed at once.
pub fn flap_held_within_limits(
    wanted: f64,
    ias_kts: f64,
    limits: &[(f64, Option<f64>)],
    current: Option<f64>,
) -> f64 {
    let cur = current.unwrap_or(0.0);
    if wanted + 1e-6 < cur {
        return flap_within_limits(wanted, ias_kts, limits);
    }
    let extend = flap_within_limits(wanted, ias_kts + FLAP_EXTEND_MARGIN_KTS, limits);
    if extend > cur {
        return extend;
    }
    flap_within_limits(cur, ias_kts - FLAP_RETRACT_MARGIN_KTS, limits)
}

/// An APPROACH never takes away the flap the wing is flying on. A detent comes
/// in only at the speed that flies what is left (`retract_kts`: the cleaner
/// configuration's manoeuvring speed where the profile declares one, else its
/// speed floor), unless a placard forces it (over the current detent's Vfe,
/// [`flap_within_limits`]) or the go-around asks for its half (`go_around`).
/// LEMG 13, 2026-09-28: every sim-started 10 nm final handed autoland the 737
/// with flaps 2 (0.25) at idle, the distance schedule wanted it clean, and
/// the flaps came up on the first tick at a 58 kt airspeed read (the sim's
/// ASI still waking) and 146 kt a second later, under the 139 kt clean
/// stall; with the lever at idle and a climb on the dial the aircraft sank
/// at up to 4,400 fpm (sortie 1790583434-approach bottomed at 200 ft AGL,
/// 1790584266-approach turned onto 12's line over rising ground and hit).
/// `current` is `None` only when the flap position was never read. Pure.
pub fn flap_kept_flying(
    flap: f64,
    current: Option<f64>,
    ias_kts: f64,
    limits: &[(f64, Option<f64>)],
    go_around: bool,
    retract_kts: impl Fn(f64) -> Option<f64>,
) -> f64 {
    let Some(cur) = current else {
        return flap;
    };
    if go_around || flap + 1e-6 >= cur {
        return flap;
    }
    // Over the placard of what is out: the placard wins, it is the other
    // edge of the same envelope.
    if flap_within_limits(cur, ias_kts, limits) + 1e-6 < cur {
        return flap;
    }
    match retract_kts(flap) {
        Some(need) if !(ias_kts >= need) => cur,
        _ => flap,
    }
}

/// The speed a flap setting may be retracted TO for this airframe: the
/// detent's declared manoeuvring speed (`retract_above_kts`), else its speed
/// floor (1.2 x the stall in it), else nothing known.
fn retract_speed_kts(ratio: f64) -> Option<f64> {
    crate::aircraft::live_profile()
        .and_then(|p| {
            crate::aircraft::flap_schedule(p.flaps.as_ref())
                .into_iter()
                .find(|(r, _)| (r - ratio).abs() < 1e-3)
                .map(|(_, k)| k)
        })
        .or_else(|| crate::aircraft::config_floor_kts(ratio))
}

/// Speed to command at this distance: clean far out, approach speed by the FAP,
/// linear between. Pure, so the schedule is testable without a sim.
pub fn scheduled_ias(dist_nm: f64, approach_ias: f64) -> f64 {
    let clean = clean_approach_ias(approach_ias);
    if dist_nm >= DECEL_START_NM {
        clean
    } else if dist_nm <= FAP_NM {
        approach_ias
    } else {
        let frac = (dist_nm - FAP_NM) / (DECEL_START_NM - FAP_NM);
        approach_ias + (clean - approach_ias) * frac
    }
}

/// The speed a reposition pattern's downwind and base are flown at: midway
/// down the deceleration schedule, and well inside the half-flap placard it is
/// flown with. The approach speed itself belongs to the final: at it, with the
/// reposition's climb bonus, the 737 flew its downwind at 139 to 162 kt under
/// flap 40's 162 kt limit.
pub fn pattern_ias(approach_ias: f64, flap_limits: &[(f64, Option<f64>)]) -> f64 {
    let mid = scheduled_ias((FAP_NM + DECEL_START_NM) / 2.0, approach_ias);
    let half_vfe = flap_limits
        .iter()
        .filter(|(ratio, _)| *ratio <= 0.5 + 1e-6)
        .filter_map(|(_, vfe)| *vfe)
        .fold(f64::INFINITY, f64::min);
    mid.min(half_vfe - 15.0).max(approach_ias)
}

/// What the copilot says when a final drops back to an intercept without a
/// go-around, or `None` when another line already says it. "Overflew the
/// runway" only with the runway BEHIND (`along_nm` negative): it was said at
/// 5.2 nm on the tick a pattern began, over the pattern's own "Repositioning"
/// line (LEMG 13, sortie 1790622093-approach, #171). Pure.
pub fn dropped_final_line(
    along_nm: f64,
    reversal_began: bool,
    above_path_ft: f64,
) -> Option<&'static str> {
    if along_nm < 0.0 {
        Some("Overflew the runway. Going around, climbing to re-intercept.")
    } else if reversal_began {
        None
    } else if above_path_ft > 0.0 {
        Some("High on the approach. Re-intercepting the glidepath.")
    } else {
        Some("Off the final. Re-intercepting.")
    }
}

/// Room past the path's own sink the glidepath law may command to get back
/// down to it, fpm.
const PATH_CATCH_UP_FPM: f64 = 300.0;

/// The steepest descent the glidepath law commands, fpm (negative): the old
/// flat 1,200, or the path's own sink at this groundspeed plus
/// [`PATH_CATCH_UP_FPM`] when that is steeper. A flat 1,200 is the 3 degree
/// path at 226 kt over the ground: faster, the law could not even HOLD the
/// path, the error grew, and past 500 ft high the guidance aimed at a join
/// point behind the aircraft, which the reversal read as a pattern (the A330
/// at 239 kt, 491 ft high at 5.3 nm on a -1,200 dial, then a downwind from
/// 5.2 nm; LEMG 13, sortie 1790622093-approach, #171). Pure.
pub fn path_descent_cap_fpm(nominal_vs_fpm: f64) -> f64 {
    (nominal_vs_fpm - PATH_CATCH_UP_FPM).min(-1200.0)
}

/// The placard of the flap that is out: the one lookup,
/// [`crate::aircraft::flap_placard_kts`]. Pure.
pub fn flap_placard_kts(flap_ratio: f64, flap_limits: &[(f64, Option<f64>)]) -> Option<f64> {
    crate::aircraft::flap_placard_kts(flap_ratio, flap_limits)
}

/// The approach's speed target sits this far under the placard of the flap out.
const PLACARD_TARGET_MARGIN_KTS: f64 = 10.0;

/// The speed the approach's governor holds: the schedule, never closer to the
/// placard of the flap actually out than [`PLACARD_TARGET_MARGIN_KTS`], and
/// never under that configuration's floor (the floor wins a conflict). The
/// schedule's clean speed is 1.45 x the approach speed, 223 kt on the A330's
/// 154: with CONF 1 out (Vfe 215) the governor added power to reach it, the
/// placard took the flap, the clean floor (1.2 x 175 = 210) then held the
/// lever up, and a 10 nm final ran at 200 to 241 kt (LEMG 13, sortie
/// 1790622093-approach, 2026-09-28, #171). Pure.
pub fn approach_target_kts(
    scheduled_kts: f64,
    flap_ratio: f64,
    flap_limits: &[(f64, Option<f64>)],
    floor_kts: Option<f64>,
) -> f64 {
    let ceiling = flap_placard_kts(flap_ratio, flap_limits)
        .map_or(f64::INFINITY, |v| v - PLACARD_TARGET_MARGIN_KTS);
    crate::aircraft::floored_target(scheduled_kts.min(ceiling), floor_kts)
}

/// The flap an approach asks for: the distance schedule, never LESS than is
/// already out. A flap out is drag and a lower floor, both what an approach
/// needs; only a placard (`flap_held_within_limits`) or the go-around's half
/// takes one away. The schedule wanted the A330 clean beyond 6 nm, CONF 1
/// came up at 211 kt 7.9 nm out (the clean retract speed, 210), and the clean
/// floor then kept the final at 210 and over (#171). `current` is the last
/// flap written or read. Pure.
pub fn approach_flap_wanted(scheduled: f64, current: Option<f64>) -> f64 {
    scheduled.max(current.unwrap_or(0.0))
}

/// The speed a give-up leaves the governor holding: the pattern speed, inside
/// the placard of whatever flap is actually out, never under the approach
/// speed. With the flaps up it is the clean approach speed.
pub fn hand_back_speed_kts(
    approach_ias: f64,
    flap_ratio: f64,
    flap_limits: &[(f64, Option<f64>)],
) -> f64 {
    if flap_ratio < 0.05 {
        return clean_approach_ias(approach_ias);
    }
    let placard = flap_placard_kts(flap_ratio, flap_limits).unwrap_or(f64::INFINITY);
    pattern_ias(approach_ias, flap_limits)
        .min(placard - 15.0)
        .max(approach_ias)
}

/// Past the threshold by more than this (nm along the centreline, departure side)
/// with no landing = an overflight. Go around: fly back to the FAP instead of
/// tracking the runway heading straight out. Seen live 2026-07-06: overflew the
/// numbers at 167 ft, then departed the far end on a fixed course to 5.4 nm at
/// 60 kt, never turning back. The margin ignores a normal float/rollout past the
/// threshold (the flare branch, checked first, still owns the touchdown).
const OVERFLIGHT_GO_AROUND_NM: f64 = 0.15;
/// The float must leave at least this much pavement to land and stop on before
/// the overflight rule lets it continue — the Baron's measured comfortable
/// rollout from 79 kt used ~550 m.
const FLOAT_MIN_REMAINING_M: f64 = 900.0;

/// How far past the threshold the aircraft may fly before "overflight" is the
/// verdict. The fixed 0.15 nm was tuned when the descent ran at path rate all
/// the way to the flare gate; the roundout legitimately floats further, and at
/// EGHI it balked a perfectly good landing at 26.3 ft radio — ONE FOOT above
/// the 25 ft flare gate — with 1,400 m of runway still ahead (2026-08-02).
/// Low over the pavement, in the landing float, the question is not "how far
/// past the numbers" but "is there still room to land and stop": allow the
/// float as long as [`FLOAT_MIN_REMAINING_M`] remains, capped at half the
/// runway (not down by halfway is a go-around on ANY runway), floored at the
/// old margin so a short strip keeps exactly the old rule. HIGH past the
/// threshold (above the roundout band) the question is geometric: where a
/// 3 degree descent from HERE reaches the band, is the float allowance still
/// ahead? The fixed 0.15 nm was a 172 figure twice over: at 60 kt a 172 that
/// crosses the numbers 50 ft high is in the band 100 m later, a 737 at 140 kt
/// is 290 m in, and EGCC 23L balked it at 57 ft and 322 m with 2,700 m of
/// runway ahead (2026-09-19). The band's own allowance less the run to the
/// band, floored at the old margin, so the 167 ft overflight over a short
/// strip is still caught and a long runway is judged by its length. The
/// pavement a landing needs is the airframe's recited landing distance when
/// it has one (the MD-82 would float Luton away on the Baron's 900 m).
pub fn overflight_allowance_nm(
    length_m: f64,
    radio_alt_ft: f64,
    band_ft: f64,
    landing_m: Option<f64>,
) -> f64 {
    let need_m = landing_m
        .filter(|v| v.is_finite() && *v > 0.0)
        .unwrap_or(FLOAT_MIN_REMAINING_M)
        .max(FLOAT_MIN_REMAINING_M);
    let low_m = (length_m - need_m).min(0.5 * length_m);
    let run_to_band_m = (radio_alt_ft - band_ft).max(0.0) * M_PER_FT / gp_tan();
    ((low_m - run_to_band_m) / M_PER_NM).max(OVERFLIGHT_GO_AROUND_NM)
}
const M_PER_FT: f64 = 0.3048;
/// tan(3 degrees): the metres a 3 degree path runs per metre of height.
const GLIDEPATH_TAN: f64 = 0.052_408;

/// A latched go-around climbs straight ahead until this height before the
/// intercept guidance may turn it back toward the FAP (see GoAroundLatch).
const GO_AROUND_TURN_AGL_FT: f64 = 800.0;
/// How near the extended centreline the reposition leg must be before it
/// hands back to the ordinary approach (see [`GoAroundLatch::update`]). Half
/// a mile leaves the pursuit ~17 m of offset after four time constants on a
/// 737's 118 s final, inside the flare gate; a 172's 280 s final never
/// needed the rule.
const REPOSITION_RELEASE_XTK_NM: f64 = 0.5;
/// Terrain floor (ft AGL): below this and outside the flare zone, low radio
/// altitude is rising terrain under the flight path, not the runway — climb away
/// from it rather than hold level into it. Seen live 2026-07-06: held level at
/// ~50 ft AGL 5 nm out over rising ground, no climb authority anywhere in the loop.
const TERRAIN_FLOOR_AGL_FT: f64 = 500.0;

/// Fast by more than this on final and the speed law may still idle a jet.
const APPROACH_FAST_KTS: f64 = 10.0;
/// The flap ratio that is the landing configuration (the schedule's full).
pub(crate) const LANDING_FLAP_RATIO: f64 = 0.999;
/// The lever's idle stop in the speed law.
const IDLE_THROTTLE: f64 = 0.1;

/// The least the speed law may pull the lever to. On FINAL, not more than
/// `APPROACH_FAST_KTS` fast, it is the airframe's approach thrust floor
/// (jets fly the approach on thrust: the 737 idled its last mile and touched
/// down 17 kt under Vref, LFLL 2026-09-17) - but only in the LANDING
/// configuration (`landing_flaps`), the one the floor was measured in (0.4
/// at Vref with flaps 40). With flaps 10 the same 0.3 is more than the
/// path needs: from the FAP to 3 nm every LEMG 13 final of 2026-09-26 sat
/// at 161 to 164 kt, the lever flipping 0.1 / 0.3 on the ten-knot edge of
/// the flaps-10 floor (152), so the landing flaps came out late and the
/// final was still 155 to 158 kt through 800 ft. In a REPOSITION below the
/// terrain floor it is full power: excess speed there is spent on the
/// climb, never on the lever ("land the aircraft" from 140 ft at 237 kt
/// pulled the throttle to idle for its 179 kt target and the aircraft
/// settled into the fields with +800 fpm on the dial, same day). Everywhere
/// else, idle.
/// The speed a flare's "slow" deferral protects: the approach speed, but
/// never above 1.3 Vs in the configuration flown (`config_floor` is 1.2 Vs).
/// A generous approach target is not a stall margin: the King Air's 110
/// held the lever at 0.53 to the wheels at 103 kt, 1.36 Vso, "slow" for 30
/// flare ticks, and it skipped at 101 kt (LEMG 13, iMac 2026-09-30,
/// 1790790995). Pure.
pub fn retard_reference_kts(approach_ias: f64, config_floor_kts: Option<f64>) -> f64 {
    match config_floor_kts.filter(|f| f.is_finite() && *f > 0.0) {
        Some(floor) => approach_ias.min(floor * 1.3 / crate::aircraft::SPEED_FLOOR_RATIO),
        None => approach_ias,
    }
}

/// The roundout's retard where the airframe declares no `flare_power`.
const DEFAULT_ROUNDOUT_RETARD_PER_S: f64 = 0.1;

/// The lever in a roundout (the arrest holding the servos, not yet the
/// Flare) that FLOATS: a sink under [`RETARD_FLOAT_FPM`], now or
/// [`RETARD_FLOAT_LEAD_S`] ahead on its trend, walks the lever toward idle
/// at `per_s`, under the approach's own floor; else `None` (the speed law's).
/// King Air LEMG 13 (iMac 2026-09-30, 1790786159 and 1790787298): the
/// column at neutral, the arrest took the sink from 557 fpm to level at 36 ft
/// with the lever held at 0.61 (the speed law "slow" by a knot, only ever
/// back in the roundout), climbed to 101 ft at 700 fpm and balked `balloon`.
/// Pure.
pub fn roundout_float_lever(throttle: f64, vvi_fpm: f64, trend_fpm_s: f64, dt_s: f64, per_s: f64) -> Option<f64> {
    let floating = vvi_fpm + trend_fpm_s.max(0.0) * RETARD_FLOAT_LEAD_S > -RETARD_FLOAT_FPM;
    floating.then(|| (throttle - per_s * dt_s.clamp(0.0, 1.0)).max(IDLE_THROTTLE))
}

/// The short final's power hold is asked only this close to the threshold:
/// a low radio altitude further out is terrain under the approach.
const POWER_HOLD_MAX_DIST_NM: f64 = 1.0;
/// How far ahead the hold's protected speed is judged, seconds (the speed
/// law's own projection).
const POWER_HOLD_PROTECT_LEAD_S: f64 = 5.0;

/// Whether the speed law may only take power off on this tick of a final:
/// under the airframe's `flare_power.hold_from_s` seconds of the path's sink
/// at this groundspeed, inside [`POWER_HOLD_MAX_DIST_NM`], while the
/// airspeed projected [`POWER_HOLD_PROTECT_LEAD_S`] ahead stays at or over
/// `protect_kts` (the flare's protected speed, `retard_reference_kts`).
///
/// King Air C90B, LEMG 13 from 5 nm (iMac 2026-09-30, thirteen autolands):
/// the speed law cycled the lever 0.45 to 0.66 every 20 s on the final
/// (107 to 113 kt), the autopilot flying the sink through the elevator TRIM
/// against it (servo 0.00, trim -0.40 at the lever's peak, -0.12 at its
/// trough). At the arrest's release (59 to 61 ft) the servos and the trim
/// stop, and what the lever did in the seconds before decides the roundout:
/// the three releases on an unraised lever (0.47 to 0.49: 1790783744,
/// 1790787102, 1790792087) came through 27 ft still sinking 460 to 470 fpm
/// and landed at 29 to 69 fpm; the ten raised in the last 6 s (0.45-0.48 to
/// 0.52-0.66) levelled at 30 to 39 ft and ballooned to 100 ft (1790786159,
/// 1790787298), or came to the flare at 240 to 380 fpm on a lever the float
/// then idled (1790795346: 0.64 at release, level at 39 ft, idled at 43,
/// 641 fpm at the wheels). The trim at release was the approach's own
/// (-0.08 to -0.27), not an arrest's. Pure.
#[allow(clippy::too_many_arguments)]
pub fn short_final_power_held(
    phase: ApproachPhase,
    dist_nm: f64,
    radio_ft: f64,
    gs_kts: f64,
    hold_from_s: Option<f64>,
    ias: f64,
    trend_kts_s: f64,
    protect_kts: f64,
) -> bool {
    let Some(hold_s) = hold_from_s.filter(|s| s.is_finite() && *s > 0.0) else {
        return false;
    };
    let sink_fps = (gs_kts.max(0.0) / 60.0) * gp_ft_per_nm() / 60.0;
    phase == ApproachPhase::Final
        && dist_nm <= POWER_HOLD_MAX_DIST_NM
        && radio_ft.is_finite()
        && radio_ft <= sink_fps * hold_s
        && ias + trend_kts_s.min(0.0) * POWER_HOLD_PROTECT_LEAD_S >= protect_kts
}

pub fn approach_power_floor(
    phase: ApproachPhase,
    radio_alt_ft: f64,
    err_kts: f64,
    approach_thrust_min: Option<f64>,
    landing_flaps: bool,
) -> f64 {
    match phase {
        ApproachPhase::Intercept if radio_alt_ft < TERRAIN_FLOOR_AGL_FT => 1.0,
        ApproachPhase::Final if err_kts > -APPROACH_FAST_KTS && landing_flaps => {
            approach_thrust_min
                .unwrap_or(IDLE_THROTTLE)
                .max(IDLE_THROTTLE)
        }
        _ => IDLE_THROTTLE,
    }
}
/// A climb is flown on thrust. Repositioning (Intercept) with the vertical
/// law asking to CLIMB, the lever never sits below the airframe's approach
/// thrust floor (idle when none is declared): the speed law alone reads a
/// dive's rising airspeed as "fast" and idles. LEMG 13, 2026-09-28 (sortie
/// 1790584266-approach): +100 to +2,000 fpm on the dial under the reposition
/// floor, the lever at 0.10 from the arm until 467 ft radio, the 737 sinking
/// 1,200 to 3,700 fpm. Final and the flare keep their own rules. Pure.
pub fn climb_power_floor(
    phase: ApproachPhase,
    target_vs_fpm: f64,
    approach_thrust_min: Option<f64>,
) -> f64 {
    if phase == ApproachPhase::Intercept && target_vs_fpm > 0.0 {
        approach_thrust_min.unwrap_or(IDLE_THROTTLE).max(IDLE_THROTTLE)
    } else {
        IDLE_THROTTLE
    }
}

/// Don't enforce the terrain floor inside this range ON FINAL — here a low
/// radio altitude IS the approach (short final), not terrain to climb over.
/// The gate applies to Final only: in Intercept the aircraft is repositioning,
/// not landing, so proximity to the threshold means nothing — the KPDK 16
/// rejoin (2026-07-14) armed 0.75 nm out at 300 AGL, the gate held the floor
/// off, and the vertical law tracked the (distance-symmetric) glidepath into
/// the ground 0.35 nm short of the runway.
const FLOOR_ENFORCE_MIN_DIST_NM: f64 = 1.0;
/// ...and on Final the floor binds only this far BELOW the glidepath. A 3°
/// path crosses 500 AGL at ~1.6 nm, so at any flat field the floor used to
/// capture a perfectly on-path approach between 1.6 and 1.0 nm — held it
/// level with the climb-energy speed bonus, then released it 200 ft high
/// and 15 kt hot at the numbers for a guaranteed overflight go-around
/// (LXGB 09, 2026-07-22). Every scenario the floor exists for (dragged in
/// low over rising ground) is deeply below the path; on-path with low
/// radio near the field IS the approach.
const FLOOR_BELOW_PATH_FT: f64 = -100.0;
/// Repositioning (Intercept) wants PLATFORM height — the ~1,600 ft AGL the
/// aircraft must be back at by the FAP to fly the path down anyway. Not 500
/// ("flying around 500 feet, is that a bit too low?" — it was), and not
/// 1,000 either: the LXGB 27 go-around turned back toward a FAP east of the
/// field and flew level at ~650 ft AGL into the Rock of Gibraltar (1,400 ft;
/// radio collapsed 680 → 3 in six seconds, CFIT, 2026-07-22). Terrain-aware
/// routing is the real fix; until then, climb to the altitude the rejoin
/// needs and the highest terrain a sane approach corridor hides.
const REPOSITION_MIN_AGL_FT: f64 = FAP_NM * GLIDEPATH_FT_PER_NM;
/// A pattern plan older than this is stale: the aircraft has flown on from
/// where its downwind was probed from.
const REVERSAL_PLAN_FRESH_S: f64 = 60.0;

/// The live floor is probed this often, and a tripped floor stands this
/// long after its last trip (the circuit's 2 s and 60 s).
const LIVE_FLOOR_EVERY_S: f64 = 2.0;
const LIVE_FLOOR_HOLD_S: f64 = 60.0;

use crate::reversal::TerrainHold;

/// The circuit's leg whose live floor (`circuit::live_floor_ft`) a reversal
/// leg or a hold keeps: the level legs climb clear of ground ahead, the base
/// (and the ordinary intercept, descending to its path) only keeps the floor
/// itself. `None` on the final and in the go-around's own climb. Pure.
fn live_floor_leg(
    phase: ApproachPhase,
    missed: Option<MissedStage>,
    pattern_leg: Option<crate::reversal::Leg>,
    holding: bool,
) -> Option<crate::circuit::Leg> {
    use crate::circuit::Leg as C;
    if phase != ApproachPhase::Intercept || missed == Some(MissedStage::Climb) {
        return None;
    }
    Some(match pattern_leg {
        Some(crate::reversal::Leg::Base) => C::Base,
        Some(_) => C::Downwind,
        None if holding || missed.is_some() => C::Downwind,
        None => C::Base,
    })
}

#[cfg(test)]
mod terrain_hold_tests;

/// Only climb with this much surplus over the approach speed. Climbing slow in a
/// marginal airframe bleeds energy to a stall (the reason this loop was
/// descent-only) — so a low-and-slow aircraft holds level and lets the throttle
/// governor rebuild speed FIRST, then the climb engages once the surplus exists.
const CLIMB_SPEED_MARGIN_KTS: f64 = 5.0;
/// Ceiling on the safe-altitude climb (fpm). Was 500 — "gentle and energy-
/// preserving" read as anemic against real terrain (the Rock closed at an
/// effective 5,000+ fpm, 2026-07-22). The climb gate already requires
/// surplus airspeed, and speed protection still owns the slow case.
const MAX_CLIMB_FPM: f64 = 800.0;

/// ...and the AIRFRAME's climb when it has more, because 800 fpm is a light
/// single's answer to terrain and an airliner's idea of level flight. It was
/// raised from 500 once already for being anemic; on a 737 it is anemic
/// again — the EGPH go-around (2026-09-20) climbed away from a balked
/// approach at exactly this number. Never LESS than the GA figure, so
/// nothing that flew before gets weaker.
pub(crate) fn max_climb_fpm() -> f64 {
    crate::procedure::climb_fpm().max(MAX_CLIMB_FPM)
}
/// While below the terrain floor, command the throttle governor to this much over
/// the approach speed, so it builds the surplus the climb gate needs.
const CLIMB_ENERGY_BONUS_KTS: f64 = 12.0;

/// Flare only inside this distance to the threshold — beyond it, a low radio
/// altitude is terrain under the approach, not the runway. (25 ft on a 3° path
/// is ~0.08 nm out; 0.35 leaves margin for displaced thresholds.)
const FLARE_MAX_DIST_NM: f64 = 0.35;
/// The roundout band: below this multiple of the flare height (inside the same
/// runway gates as the flare itself), the vertical target eases to
/// [`ROUNDOUT_VS_FPM`] so the flare is entered from a gentle descent rather
/// than from the full path rate or a level float.
const ROUNDOUT_MULT: f64 = 2.0;
const ROUNDOUT_VS_FPM: f64 = -250.0;
const FLARE_VS_FPM: f64 = -100.0;

/// The arrest gates are SECONDS of sink, not feet: 25 ft is three seconds
/// of a 172's path sink and two of a 737's, and the 737 through this
/// autopilot's lag needs every one of them (EGCC 23L, 2026-09-19: the
/// roundout opened at 50 ft with the aircraft sinking 712 fpm, 4 s from the
/// pavement, and the arrest could only balloon or arrive firm). Both gates
/// are the greater of the fixed feet and the seconds at the NOMINAL path
/// sink for this groundspeed, so every airframe up to a 172's speeds keeps
/// exactly its old feet (50 and 25 at 75 kt) and a jet's open higher: at
/// 132 kt over the ground, 76 and 35.
const ROUNDOUT_LEAD_S: f64 = 6.5;
const FLARE_LEAD_S: f64 = 3.0;
/// The gates for this approach: (flare height, roundout band top), feet.
pub fn arrest_gates_ft(flare_radio_ft: f64, gs_kts: f64) -> (f64, f64) {
    let sink_fps = (gs_kts.max(0.0) / 60.0) * gp_ft_per_nm() / 60.0;
    let flare = flare_radio_ft.max(sink_fps * FLARE_LEAD_S);
    let band = (flare * ROUNDOUT_MULT).max(sink_fps * ROUNDOUT_LEAD_S);
    (flare, band)
}
/// The arrest's targets scale with the path sink the same way: the 172's
/// -250 then -100 are 60% and 25% of its 400 fpm path, and a 737 asked to
/// hold -100 from 35 ft floats a kilometre at 140 kt before it touches.
/// (roundout, flare), fpm; never shallower than the fixed figures.
pub fn arrest_targets_fpm(nominal_vs_fpm: f64) -> (f64, f64) {
    (
        ROUNDOUT_VS_FPM.min(0.6 * nominal_vs_fpm),
        FLARE_VS_FPM.min(0.25 * nominal_vs_fpm),
    )
}

/// The flare's sink target at this height: an EXPONENTIAL flare, as
/// autoland systems fly it. The touchdown sink (`flare_vs`) plus the height
/// over [`FLARE_TAU_S`], never steeper than the approach path. The flare
/// used to hold `flare_vs` flat from its gate: a 737 asked for -175 fpm from
/// 35 ft takes twelve seconds to arrive, and floated LEMG 13 from 36 to 19 ft
/// over five of them before the power came off and it dropped in at -859
/// fpm (2026-09-26). Proportional to the height it keeps descending and
/// arrives at `flare_vs` in about five. Pure.
pub fn flare_target_fpm(radio_ft: f64, flare_vs: f64, nominal_vs: f64) -> f64 {
    flare_target_with(radio_ft, flare_vs, nominal_vs, flare_tau_s())
}

/// [`flare_target_fpm`] with the time constant given: `None` is the flat
/// flare every airframe but a declared one keeps. The 172's flare is flown
/// through its VS mode's 4.5 s lag, and a target that moves with the height
/// is faster than that lag can follow (the sortie test lands it at -298
/// fpm); the 737's is flown on the elevator with the servos out. Pure.
pub fn flare_target_with(radio_ft: f64, flare_vs: f64, nominal_vs: f64, tau_s: Option<f64>) -> f64 {
    match tau_s.filter(|t| t.is_finite() && *t > 0.5) {
        Some(tau) => (flare_vs - radio_ft.max(0.0) * 60.0 / tau).max(nominal_vs.min(flare_vs)),
        None => flare_vs,
    }
}

/// The live airframe's exponential flare time constant, if it declares one
/// (`flare_elevator.tau_s`).
fn flare_tau_s() -> Option<f64> {
    crate::aircraft::live_profile()
        .and_then(|p| p.flare_elevator)
        .and_then(|f| f.tau_s)
}

/// The VS mode is a slow servo, and the landing arrest is the one place the
/// dial's lag is measured in feet of runway. LXGB 27, 2026-09-14 (the 172's
/// third firm arrival in three days, same shape each time): the roundout
/// dialled -250 at 49 ft radio and the aircraft was still sinking 386 fpm
/// four seconds later; the flare dialled -100 at 23 ft and the vertical
/// speed never moved off -260 before the wheels touched at -309 (the
/// blackbox's `cmd_vs` column changed on time, the `vsi` column never
/// followed). Fitted to that tick log the autopilot follows the dial as a
/// first-order lag of about 4.5 s with a tick of dead time, which from the
/// flare gate at approach sink is longer than the flare itself. So in the
/// roundout and the flare the DIAL leads the target by the error the
/// autopilot still owes (`vs_dial_fpm`): dial = target + gain x (target -
/// actual). Gain 2 makes the closed lag ~1.5 s, comfortably inside the
/// stability margin a 1 s dead time leaves (about 7). The dial may read a
/// CLIMB for a moment - that is the flare, the yoke coming back - capped at
/// [`VS_LEAD_MAX_CLIMB_FPM`] so a balloon can never be commanded, and the
/// glidepath above the band is untouched (its own P law tracks a ramp with
/// `path_descent_fpm`, and every approach test stays byte-identical).
/// The 172's numbers live in [`crate::aircraft::ArrestLeadProfile::default`]
/// (gain 2, cap +300, no trend term); a jet declares its own in its profile.
#[cfg(test)]
const VS_LEAD_GAIN: f64 = 2.0;
#[cfg(test)]
const VS_LEAD_MAX_CLIMB_FPM: f64 = 300.0;

/// The altitude dial must never be within reach BELOW the aircraft while
/// autoland flies: an armed capture takes the vertical away from the VS dial
/// (see the arm sequence in [`run_autoland`]). The LEAX rule
/// ([`autoflight::WindowRule::ParkAbove`]: 1,000 ft above, rounded up to the
/// hundred, unless already 500 ft clear). Pure.
pub fn parked_altitude_dial_ft(dial_ft: f64, alt_ft: f64) -> Option<f64> {
    autoflight::window_for(autoflight::WindowRule::ParkAbove, dial_ft, alt_ft, 0.0)
}

/// The window the arm parks for the airframe's rule
/// ([`autoflight::window_for`]): above, or at the field on an MCP
/// that flies a V/S only toward its window (the 737, LEMG 13 2026-10-05).
/// Pure.
pub fn parked_window_ft(
    rule: autoflight::WindowRule,
    dial_ft: f64,
    alt_ft: f64,
    field_ft: f64,
) -> Option<f64> {
    autoflight::window_for(rule, dial_ft, alt_ft, field_ft)
}

/// A probed threshold elevation within this of the header is the header (the
/// mesh and the file agree; no reason to move the aim by a few feet).
pub const THRESHOLD_ELEV_TRUST_FT: f64 = 20.0;
/// ...and one further than this from it is not believed (a probe off the
/// pavement, a broken mesh): the header stays, the number is still reported.
pub const THRESHOLD_ELEV_MAX_DIFF_FT: f64 = 300.0;
/// The glidepath's aim elevation: the HIGHER of the probed threshold and
/// touchdown-zone points when they disagree with the apt.dat header by more
/// than [`THRESHOLD_ELEV_TRUST_FT`] and less than [`THRESHOLD_ELEV_MAX_DIFF_FT`];
/// the header otherwise (and always without a probe). Returns (aim, probed).
pub fn aim_elevation_ft(header_ft: f64, probed_ft: Option<&[f64]>) -> (f64, Option<f64>) {
    let Some(p) = probed_ft.filter(|v| !v.is_empty()) else {
        return (header_ft, None);
    };
    let probed = p.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if !probed.is_finite() {
        return (header_ft, None);
    }
    let diff = (probed - header_ft).abs();
    if diff <= THRESHOLD_ELEV_TRUST_FT || diff > THRESHOLD_ELEV_MAX_DIFF_FT {
        return (header_ft, Some(probed));
    }
    (probed, Some(probed))
}

/// How far a tuned glideslope may move the aim from the pavement's, ft:
/// the path's threshold crossing (51.6 ft at 3 degrees aimed 300 m in)
/// stays inside a book's 35 to 80 ft.
pub const GS_AIM_BELOW_FT: f64 = 15.0;
pub const GS_AIM_ABOVE_FT: f64 = 30.0;

/// The aim elevation with the runway's ILS glideslope tuned: the path that
/// lies ON the glideslope (`ils::glideslope_aim_ft`), as far as a book's
/// threshold crossing over the pavement (`pavement_aim_ft`, from
/// [`aim_elevation_ft`]) allows; the pavement's aim without one. Pure.
///
/// EGBB 15, 2026-09-28 (sortie 1790601822-full_auto, #156): the probe put
/// the aim at 297 ft (the header 327) while the glideslope's antenna stands
/// at 348 ft, 0.34 nm in: the needle read 1.2 dots low at 497 ft on a path
/// autoland was flying as it meant to. Aimed at 327 the path lies
/// 30 ft nearer the beam with the wheels still crossing the numbers at 81 ft.
pub fn aim_with_glideslope(pavement_aim_ft: f64, gs_aim_ft: Option<f64>) -> f64 {
    match gs_aim_ft.filter(|g| g.is_finite()) {
        Some(g) => pavement_aim_ft + (g - pavement_aim_ft).clamp(-GS_AIM_BELOW_FT, GS_AIM_ABOVE_FT),
        None => pavement_aim_ft,
    }
}

/// The gentlest retard below the retard height, lever per second.
const RETARD_MIN_PER_S: f64 = 0.03;
/// The retard waits while the airspeed is this far under the approach speed
/// (kt), unless the profile says otherwise (`retard_slow_kts`).
const RETARD_SLOW_KTS: f64 = 5.0;
/// Time to the wheels is taken at no less than this sink (fpm): a float is
/// not an infinite time, it is a slow retard.
const RETARD_SINK_FLOOR_FPM: f64 = 100.0;
/// A flare sinking less than this (fpm) is FLOATING: the arrest is done and
/// the thrust is what holds it up, so the retard starts, whatever the height
/// and the speed. EGBB 15 (737, 2026-09-28, sortie 1790601822): levelled at
/// 22.7 ft on 0.30 of lever, held by the height key over 20 ft, then by the
/// time key (a float is 13 s from the wheels), then by the speed key once
/// the float had bled 7 kt; the lever never moved and the float ran 4 s.
pub const RETARD_FLOAT_FPM: f64 = 100.0;
/// ...or will be, this many seconds ahead on the sink's trend: the float is
/// met where the arrest is heading, not where it arrives. Thrust acts through
/// the speed and the column through the 737's 1.5 s dead time, so a retard
/// begun at the float itself is late: at EGBB 15 the sink came from -300 to
/// -111 in 2.1 s. Replayed through that flare's fitted plant (6,300 fpm per
/// unit, 1.7 s) and its air, a lead of 1 s left 3.6 s of float, 1.5 s none.
pub const RETARD_FLOAT_LEAD_S: f64 = 1.5;
/// The sink's trend for the float ahead is smoothed over this, seconds.
const RETARD_TREND_TAU_S: f64 = 0.5;
/// A flare is CLIMBING (a balloon) only past this, fpm. A near-level float
/// reads a few fpm either side of zero (EGBB 15: -2, +1, +2, 0 at 22.9 ft),
/// and +1 is not a climb: it eased the held column 0.28 to 0.20 and would
/// put the approach thrust back on a lever the float is taking off.
pub const BALLOON_CLIMB_FPM: f64 = 25.0;

/// Why the retard did what it did this tick (the `power` tick's `retard`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetardKey {
    /// The first flare tick: the approach's power is where it starts.
    Entry,
    /// Above the retard height.
    Height,
    /// More time to the wheels than `retard_at_s`.
    Time,
    /// Slower than the approach speed less the margin, the retard not yet
    /// begun and the aircraft still sinking: the thrust stays.
    Slow,
    /// Floating (sink under [`RETARD_FLOAT_FPM`], now or
    /// [`RETARD_FLOAT_LEAD_S`] ahead on its trend): walking off at the
    /// airframe's quickest retard, at any height and any speed.
    Float,
    /// Walking to idle, timed to the wheels.
    Timed,
}

impl RetardKey {
    pub fn word(self) -> &'static str {
        match self {
            RetardKey::Entry => "entry",
            RetardKey::Height => "height",
            RetardKey::Time => "time",
            RetardKey::Slow => "slow",
            RetardKey::Float => "float",
            RetardKey::Timed => "timed",
        }
    }
}

/// THE FLARE RETARD, keyed on what it is for: HEIGHT (nothing above the
/// retard height), TIME to the wheels (the lever reaches idle as they
/// arrive: the rate is what is left over the seconds left, radio over
/// sink), the FLOAT (an arrested sink held up by thrust, or one the trend
/// will arrest within [`RETARD_FLOAT_LEAD_S`]: off it comes at any height,
/// at the airframe's quickest) and SPEED (a flare slower than the approach speed less
/// [`RETARD_SLOW_KTS`] may DEFER the retard while it is still sinking; it
/// never holds power in a float and never stops a retard that has begun).
/// Never on the sink the arrest is removing: the old law held the lever
/// while the arrest worked and let it go in the float the arrest had made.
/// It "fired latest exactly when it should already be finished" (EGLF 06
/// 2026-09-22; LEMG 13 2026-09-26 and 2026-09-28; issue #76). The float is
/// the other half: height, time and speed each held 0.30 through a 4 s
/// float at 22.7 ft (EGBB 15, 2026-09-28; issue #146).
///
/// Per approach: a go-around resets it, so a second flare starts from its
/// own power. Never adds power or impersonates an airspeed governor in the
/// flare; a lever still up at the wheels is the rollout's to close.
#[derive(Default)]
struct FlarePower {
    last_command: Option<f64>,
    last_key: Option<RetardKey>,
    /// The last sink read and its smoothed rate of change (fpm per second,
    /// positive = easing), for the float ahead.
    prev_vvi: Option<f64>,
    trend_fpm_s: f64,
    /// The time to the wheels has come inside `retard_at_s`, or the flare
    /// has floated (latched).
    due: bool,
    /// The lever has started off (latched): the speed key no longer holds.
    begun: bool,
}

impl FlarePower {
    fn reset(&mut self) {
        *self = Self::default();
    }

    /// The lever this tick; `None` when a measurement is missing (no guessed
    /// power and no ramp advance). An unread airspeed is no speed key.
    #[allow(clippy::too_many_arguments)]
    fn step(
        &mut self,
        cfg: crate::aircraft::FlarePowerProfile,
        measured: f64,
        vvi: f64,
        dt: f64,
        radio_ft: f64,
        ias: Option<f64>,
        approach_ias: Option<f64>,
    ) -> Option<f64> {
        if !measured.is_finite()
            || !vvi.is_finite()
            || !radio_ft.is_finite()
            || !dt.is_finite()
            || dt <= 0.0
            || !cfg.reduction_per_second.is_finite()
            || cfg.reduction_per_second <= 0.0
        {
            return None;
        }
        // First flare tick preserves the observed approach power. Readback may
        // lag a write, but it must never restart the ramp or increase power.
        let current = measured
            .clamp(0.0, 1.0)
            .min(self.last_command.unwrap_or(1.0));
        let slow_kts = cfg
            .retard_slow_kts
            .filter(|k| k.is_finite() && *k >= 0.0)
            .unwrap_or(RETARD_SLOW_KTS);
        let slow = match (ias, approach_ias) {
            (Some(v), Some(a)) if v.is_finite() && a.is_finite() => v < a - slow_kts,
            _ => false,
        };
        // The sink's trend, smoothed over RETARD_TREND_TAU_S: a float the
        // arrest will reach a lever's lag from now is a float now.
        if let Some(p) = self.prev_vvi {
            let raw = (vvi - p) / dt;
            self.trend_fpm_s += (raw - self.trend_fpm_s) * (dt / RETARD_TREND_TAU_S).min(1.0);
        }
        self.prev_vvi = Some(vvi);
        let floating = vvi + self.trend_fpm_s.max(0.0) * RETARD_FLOAT_LEAD_S > -RETARD_FLOAT_FPM;
        let above = cfg
            .retard_below_ft
            .filter(|h| h.is_finite() && *h > 0.0)
            .is_some_and(|h| radio_ft > h);
        let sink = (-vvi).max(RETARD_SINK_FLOOR_FPM);
        let to_touch_s = (radio_ft.max(0.0) / sink * 60.0).max(0.5);
        let fast = match (ias, approach_ias, cfg.retard_fast_kts) {
            (Some(v), Some(a), Some(k)) if v.is_finite() && a.is_finite() && k.is_finite() => {
                v > a + k
            }
            _ => false,
        };
        self.due |= floating
            || (!above
                && (fast
                    || cfg
                        .retard_at_s
                        .filter(|s| s.is_finite() && *s > 0.0)
                        .is_none_or(|s| to_touch_s <= s)));
        let key = if self.last_command.is_none() {
            RetardKey::Entry
        } else if floating {
            RetardKey::Float
        } else if !self.due {
            if above {
                RetardKey::Height
            } else {
                RetardKey::Time
            }
        } else if slow && !self.begun {
            RetardKey::Slow
        } else {
            RetardKey::Timed
        };
        let fastest = cfg
            .retard_max_per_s
            .filter(|r| r.is_finite() && *r > 0.0)
            .unwrap_or(cfg.reduction_per_second)
            .max(RETARD_MIN_PER_S);
        let rate = match key {
            // A float has no time to the wheels: the thrust holding it up
            // is what comes off, as quickly as the airframe allows.
            RetardKey::Float => Some(fastest),
            RetardKey::Timed => Some((current / to_touch_s).clamp(RETARD_MIN_PER_S, fastest)),
            _ => None,
        };
        let next = match rate {
            Some(rate) => {
                self.begun = true;
                (current - rate * dt.min(1.0)).max(0.0)
            }
            None => current,
        };
        self.last_command = Some(next);
        self.last_key = Some(key);
        Some(next)
    }
}

/// The lever in a flare that is CLIMBING (past [`BALLOON_CLIMB_FPM`]): the
/// approach's own thrust, never more; `None` otherwise (the flare's usual
/// idle or progressive law). A near-level float is not a climb: its thrust
/// is the retard's to take off. Pure.
/// `above_band` holds it too: a committed balloon still above the roundout
/// band comes back down on approach power, not at idle.
pub fn balloon_power(
    vvi_fpm: Option<f64>,
    approach_thr: Option<f64>,
    above_band: bool,
    progressive_flare: bool,
) -> Option<f64> {
    // An airframe with a measured progressive retard owns its energy through
    // the whole flare. Its FlarePower law is monotonic even in a balloon;
    // restoring approach thrust here bypassed that guarantee and floated the
    // C172 at LFOS 06 from 23 to 26 ft (2026-10-03). The legacy/no-profile
    // path still uses approach thrust to bring a committed high balloon back.
    if progressive_flare {
        return None;
    }
    match (vvi_fpm, approach_thr) {
        (Some(v), Some(t)) if (v > BALLOON_CLIMB_FPM || above_band) && t.is_finite() => {
            Some(t.clamp(0.0, 1.0))
        }
        _ => None,
    }
}

/// What to dial into the VS mode so the aircraft actually FLIES `target_fpm`.
/// The plugin's direct elevator channel, and the pilot's take-back latch —
/// the SAME pair the takeoff rotates on. `override_joystick_pitch` bypasses
/// the autopilot's pitch servo while leaving it the roll, so the flare can be
/// flown on the elevator with the AP still holding the bug.
const FLARE_PITCH: &str = "xcopilot/pitch";
/// The roll channel the takeoff's climb-out flies, and its take-back latch:
/// with the servos off for the arrest, the runway track is held here.

const FLARE_ROLL: &str = "xcopilot/roll";

/// Elevator for a wanted sink rate, flown DIRECTLY instead of asked of the
/// autopilot's vertical-speed mode.
///
/// The whole `vs_dial_fpm` lead exists to compensate for a lag we chose to
/// fly through: the 737's VS mode is two 1.5 s lags with 1.5 s of dead time,
/// and the arrest is over in three seconds. Commanding the elevator removes
/// the lag rather than predicting it — which is what the TAKEOFF already
/// does, for the same reason, in its own comment: "the AP's VS mode is too
/// slow for a rotation". EGPH (2026-09-20) touched down at 977 fpm against a
/// 175 fpm target with the arrest asking the dial nicely.
///
/// P on the sink-rate error plus D on its trend, in units of elevator. Small
/// gains and a tight clamp on purpose: this is three seconds of flight a few
/// feet up, and a flare that over-rotates is a tail strike. The trim term is
/// the standing pull the aircraft needs to hold the attitude, found by the
/// integrator rather than assumed.
///
/// IN THE FLARE THE COLUMN ONLY COMES BACK, never forward (`ratchet`). A
/// tracker that has BEATEN its target relaxes, and forward column a few feet
/// up is a nosewheel-first landing. EGLF 06 (2026-09-22): the arrest pulled
/// the 737 from -511 to -102 fpm against a -197 target, the error went
/// negative and the elevator came back 0.35 -> 0.185 at thirteen feet, while
/// [`FlarePower`] - unlocked by that very same arrest, sink now inside its
/// `hold_sink_fpm` - walked the lever 0.29 to idle over the last 3.4 s. The
/// thrust left DURING the arrest instead of before it, the sink rebuilt to
/// -358 and worsening, and by then the law was saturated at
/// [`FLARE_PITCH_MAX`] with nothing left to give. The cap still bounds the
/// attitude, so the ratchet cannot buy a tail strike; a float it can buy is
/// the overflight guard's to judge, and the guard already knows the recited
/// landing distance.
///
/// The ROUNDOUT above the flare is ordinary flying and stays free both ways:
/// the ratchet is the last few feet, where the only question left is how
/// hard the wheels arrive.
#[derive(Debug, Default, Clone, Copy)]
pub struct FlarePitch {
    trim: f64,
    held: f64,
    gains: FlareGains,
    /// The gentlest sink the flare has reached (fpm, negative), since it
    /// began: the target never asks for more sink than that again.
    best_vvi: Option<f64>,
}

/// The flare law's numbers for THIS airframe (the profile's
/// `flare_elevator:`, [`crate::aircraft::FlareElevatorProfile`]). The default
/// is the law as the 172 and the 737 have flown it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlareGains {
    /// Elevator per 100 fpm of sink error.
    pub p: f64,
    /// Elevator per 100 fpm per second of sink trend.
    pub d: f64,
    /// The integrator's rate, elevator per 100 fpm of error per second.
    pub trim_rate: f64,
    /// Most pull, and most push.
    pub max: f64,
    pub min: f64,
    /// In a balloon the held pull eases toward the free law at most this
    /// fast (elevator per second); `None` lets go at once.
    pub balloon_ease_per_s: Option<f64>,
}

impl Default for FlareGains {
    fn default() -> Self {
        Self {
            p: FLARE_P_PER_100FPM,
            d: FLARE_D_PER_100FPM_S,
            trim_rate: FLARE_TRIM_PER_100FPM_S,
            max: FLARE_PITCH_MAX,
            min: FLARE_PITCH_MIN,
            balloon_ease_per_s: None,
        }
    }
}

impl FlareGains {
    /// The airframe's declared numbers over the default law.
    pub fn from_profile(p: Option<crate::aircraft::FlareElevatorProfile>) -> Self {
        let mut g = Self::default();
        if let Some(p) = p.and_then(|p| p.valid()) {
            g.p *= p.gain;
            g.d *= p.gain;
            g.trim_rate *= p.trim_gain;
            g.max = p.max;
            g.min = -p.max_push;
            g.balloon_ease_per_s = p.balloon_ease_per_s;
        }
        g
    }
}

/// Elevator per 100 fpm of sink error. A flare corrects hundreds of fpm with
/// a few percent of travel.
const FLARE_P_PER_100FPM: f64 = 0.060;
/// Damping on the sink TREND, per 100 fpm per second — the term that stops
/// the pull becoming a balloon.
const FLARE_D_PER_100FPM_S: f64 = 0.050;
/// The integrator's rate, per 100 fpm of standing error per second. Small:
/// it is there to find the standing pull, not to fly the arrest — wound
/// harder it carried the 737 through its target into a 66 fpm float.
const FLARE_TRIM_PER_100FPM_S: f64 = 0.015;
/// The flare holds the gentlest sink it has reached, but never asks for
/// less than this (fpm): a float is not a landing either.
const FLARE_HOLD_MIN_SINK_FPM: f64 = 150.0;
/// Total elevator the flare may command. Up is positive.
const FLARE_PITCH_MAX: f64 = 0.35;
/// ...and the push, which is deliberately far smaller: the ground is there.
const FLARE_PITCH_MIN: f64 = -0.10;
/// The ratchet lets go this far ahead of a balloon: the sink PREDICTED this
/// many seconds out (the trend) above zero counts as climbing.
const BALLOON_LEAD_S: f64 = 0.5;

impl FlarePitch {
    /// `vvi_trend` is fpm per second, positive = sink easing (the same sign
    /// the VS lead uses). `None` for either measurement holds what we had.
    ///
    /// `ratchet` is the flare proper (as against the roundout above it): see
    /// [`FlarePitch`] for why the column only ever comes further back there.
    pub fn step(
        &mut self,
        target_fpm: f64,
        vvi_fpm: f64,
        vvi_trend: f64,
        dt_s: f64,
        ratchet: bool,
    ) -> f64 {
        let dt = dt_s.clamp(0.02, 0.5);
        let g = self.gains;
        // In the flare the sink never grows again: the target is held to the
        // gentlest sink reached (no gentler than FLARE_HOLD_MIN_SINK). The
        // roundout had the 737 at -300 by 29 ft; the exponential target was
        // still -660 there, the law saw nothing to pull for, and the sink
        // built back to -587 at the wheels (LEMG 13, 2026-09-26).
        let target_fpm = if ratchet {
            let best = self.best_vvi.map_or(vvi_fpm, |b| b.max(vvi_fpm));
            self.best_vvi = Some(best);
            target_fpm.max(best.min(-FLARE_HOLD_MIN_SINK_FPM))
        } else {
            self.best_vvi = None;
            target_fpm
        };
        // Positive error = sinking FASTER than wanted = pull.
        let err = (target_fpm - vvi_fpm) / 100.0;
        self.trim = (self.trim + err * g.trim_rate * dt).clamp(g.min, g.max);
        let want = (err * g.p - (vvi_trend / 100.0) * g.d + self.trim).clamp(g.min, g.max);
        if !ratchet {
            // The roundout is still ordinary flying: the law is free both
            // ways. But the pull it ends on is where the flare STARTS: reset
            // to nothing, the ratchet began from the flare's first ask, and
            // the 737 went from 0.29 of elevator to 0.016 in one tick at the
            // hand-over (38 ft), the nose dropped, and it met the runway at
            // -773 fpm and bounced to 11 ft (LEMG circuit, 2026-09-26).
            // ...but a measured CLIMB (past BALLOON_CLIMB_FPM) is a
            // balloon here too, and a balloon never pushes: the SF50's
            // roundout wrote -0.05 at +62 and +214 fpm (LEMG 12,
            // 2026-09-28) and the sink it rebuilt met the runway.
            let want = if vvi_fpm > BALLOON_CLIMB_FPM {
                want.max(0.0)
            } else {
                want
            };
            self.held = want.max(0.0);
            return want;
        }
        if vvi_fpm > 0.0 || vvi_fpm + vvi_trend * BALLOON_LEAD_S > 0.0 {
            // A BALLOON (or one BALLOON_LEAD_S away: the SF50's 0.3 was held
            // while the sink vanished at +400 fpm a second, and let go only
            // once it was climbing, which was already a balloon to 52 ft;
            // EDLP 24, 2026-09-24): the pull has beaten the ground and the aircraft is
            // climbing. The ratchet exists for a sink that is being arrested,
            // and held through a climb it kept the Citation going up, 28 to
            // 58 ft at +545 fpm with the speed bleeding away (EDDV 27L,
            // 2026-09-22). The pull eases toward neutral and never becomes a
            // push, so the aircraft settles; the ratchet takes over again
            // from there as soon as it sinks.
            let eased = want.max(0.0);
            self.held = match g.balloon_ease_per_s {
                // Eased, not dropped: neutral is the TRIMMED approach
                // attitude, and at a flare's speed that sinks at once. The
                // SF50's 0.35 went to 0 in one tick at +190 fpm and the sink
                // was back at -590 two seconds later (EDLP 24, 2026-09-24).
                Some(rate) => eased.max(self.held - rate * dt),
                None => eased,
            };
            return self.held;
        }
        self.held = self.held.max(want);
        self.held
    }

    /// The law with this airframe's numbers.
    pub fn with_gains(gains: FlareGains) -> Self {
        Self {
            gains,
            ..Self::default()
        }
    }

    /// Let go — the channel expires on its own within a second, but neither
    /// the integrator nor the ratchet may carry a standing pull into the
    /// next approach. The airframe's numbers stay.
    pub fn reset(&mut self) {
        *self = Self::with_gains(self.gains);
    }
}

/// The flare flown on a MODEL of the airframe's elevator, for an airframe
/// that declares one (`flare_elevator.model`,
/// [`crate::aircraft::FlareModelProfile`]).
///
/// The P+D tracker above treats the sink as something the elevator
/// accelerates: pull and it keeps arresting. The 737's LEMG flares of
/// 2026-09-26 say a held pull HOLDS a sink, arriving after a dead time, and
/// that the aircraft pitches down by itself over the last forty feet (the
/// tail's ground effect): at LEMG 12 the sink sat at -300 from 37 to 16 ft
/// with 0.02 to 0.07 of pull and grew to -416 at the wheels while the pull
/// crept to 0.13, too late by the dead time.
///
/// So the law is a prediction, not a correction. Each tick it runs the
/// model forward `horizon_s` from now: the elevator already written arrives
/// on schedule, the ground term grows as the predicted height falls, and a
/// single new elevator is held from the moment it can arrive. It writes the
/// elevator that puts the MODEL's sink, at the horizon, on the flare's
/// target for the height the aircraft will have then. What the model does
/// not explain (the trim the autopilot left, the thrust, the speed, a gust)
/// is measured every tick as the sink less the model's own answer
/// (`unexplained`) and carried through the prediction unchanged, so a
/// wrong trim at the gate (the LEMG 12 approach arrived at -774 on a trim
/// that held -470) is learned a dead time later instead of assumed. The
/// dead time is met with LEAD, the future written into the ask, never with
/// gain. Never a push past the airframe's `max_push`, never a pull past its
/// `max`.
///
/// IN ROUGH AIR THE COLUMN ONLY COMES BACK in the flare (the Flare phase;
/// the roundout above it stays free both ways): the prediction's ask is the
/// least pull written, never less than the pull already held. Free all the
/// way down, the model chased the air mass: LEMG 13, 2026-09-28 (sortie
/// 1790589309, the final's sink straying 72 fpm rms from the one asked for
/// against 10 to 23 on the ten calm finals before it), a downdraft of about
/// 130 fpm over 30 to 20 ft had it pull 0.39 at 25 ft, the air came back,
/// the unexplained sink swung up with it, and the prediction eased the
/// column 0.39 to 0.08 by 16 ft while still sinking 415 to 246; the ease
/// arrived a dead time later in a -69 fpm FLOAT at 15 ft, the sink rebuilt
/// faster than full pull could come back, and the wheels met the runway at
/// -422 fpm. The gust and a strong elevator look the same for the seconds
/// that matter, and in SMOOTH air the free prediction is what lands a strong
/// one: through the 17:25 flare of 2026-09-26 (6,429 fpm per unit) in still
/// air, the column held from the flare gate ballooned four of the eight
/// recorded gates and dropped the worst in at 506, where the prediction
/// landed all eight under 305. So smooth air
/// keeps the prediction and holds only a near-level float
/// ([`FLARE_HOLD_MIN_SINK_FPM`]); rough air ([`FinalAir`]) holds from the
/// flare gate. Either way a float is not a balloon: only a measured CLIMB
/// (past [`BALLOON_CLIMB_FPM`], never a +1 fpm float) lets the held pull go, eased at the airframe's `balloon_ease_per_s`
/// toward the ask, never a push, and more pull comes whenever the prediction
/// of the sink at the horizon asks for it.
#[derive(Debug, Clone)]
pub struct FlareModel {
    cfg: crate::aircraft::FlareModelProfile,
    max: f64,
    min: f64,
    tau_s: f64,
    /// In a climb the held pull eases toward the ask at most this fast
    /// (elevator per second); `None` lets go at once.
    balloon_ease_per_s: Option<f64>,
    /// The pull the flare holds (the ratchet), `None` above the flare.
    held: Option<f64>,
    /// Model time, seconds, and every elevator written with its time, for
    /// the dead time.
    t: f64,
    written: std::collections::VecDeque<(f64, f64)>,
    /// The model's answer to the elevator and the ground (two lag states).
    b1: f64,
    b2: f64,
    /// The elevator the model believes is on the channel.
    el: f64,
    /// The sink the model does not explain, smoothed (a RATE plant: the
    /// change of sink it does not explain, fpm per second).
    unexplained: Option<f64>,
    /// A rate plant's last measured sink and the model's own rate over the
    /// last advance, for the rate the model does not explain.
    last_vvi: Option<f64>,
    model_rate: f64,
    /// Seconds since the last write (a failed read coasts).
    coasting_s: f64,
    /// The airspeed last read and its smoothed trend (kt/s), for a model
    /// that carries the speed (`fpm_per_kt`).
    last_ias: Option<f64>,
    ias_trend: f64,
    /// A climb past this eases the held pull ([`BALLOON_CLIMB_FPM`]; a
    /// replay of the law as flown before issue #146 sets it to zero).
    climb_fpm: f64,
}

/// One tick of [`FlareModel`], for the blackbox as much as the channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlareModelOut {
    /// The elevator to write.
    pub elev: f64,
    /// The sink the law asked for at the horizon, fpm.
    pub target_fpm: f64,
    /// The height predicted at the horizon, ft.
    pub ahead_ft: f64,
    /// The sink the model does not explain, fpm (a rate plant: fpm per
    /// second).
    pub unexplained_fpm: f64,
    /// The elevator the prediction asked for, before the flare's ratchet.
    pub ask: f64,
}

/// The model's own integration step, seconds.
const FLARE_MODEL_STEP_S: f64 = 0.02;
/// The prediction's step, seconds.
const FLARE_PREDICT_STEP_S: f64 = 0.05;
/// The airspeed trend a speed-carrying model leads with is smoothed over
/// this, seconds.
const FLARE_SPEED_TREND_TAU_S: f64 = 1.0;
/// The pitch channel expires this long after its last write.
const FLARE_CHANNEL_EXPIRY_S: f64 = 1.0;

impl FlareModel {
    /// The law for this airframe's model, pull cap, push cap (negative) and
    /// flare time constant.
    pub fn new(cfg: crate::aircraft::FlareModelProfile, max: f64, min: f64, tau_s: f64) -> Self {
        // A STANDING plant never takes the column forward of neutral in the
        // roundout or the flare: the arrest leaves the sink shallower than
        // the exponential asks, the model pushed to catch it up, and the
        // nose-down it stored arrived after its own lag, at the wheels.
        // King Air LEMG 13 (iMac 2026-09-30): no push, 93 kt and 299 fpm
        // (1790783744); pushed from 48 and 61 ft at -0.05, 103 and 109 kt,
        // 349 and 367 fpm, both skipped (1790783945, 1790784149). The
        // Evolution at LILB pushed from 52 ft at 361 fpm, the sink built to
        // 576 and it landed at 570 (1790781114, #196). A float is the
        // retard's (`FlarePower`) and the commit's, never the column's. Only
        // a RATE plant, whose neutral HOLDS a sink, pushes (`step_rate`).
        let min = if cfg.rate { min } else { min.max(0.0) };
        Self {
            cfg,
            max,
            min,
            tau_s: if tau_s.is_finite() && tau_s > 0.5 {
                tau_s
            } else {
                5.0
            },
            balloon_ease_per_s: None,
            held: None,
            t: 0.0,
            written: std::collections::VecDeque::new(),
            b1: 0.0,
            b2: 0.0,
            el: 0.0,
            unexplained: None,
            last_vvi: None,
            model_rate: 0.0,
            coasting_s: 0.0,
            last_ias: None,
            ias_trend: 0.0,
            climb_fpm: BALLOON_CLIMB_FPM,
        }
    }

    /// A climb in the flare eases the held pull at most this fast (elevator
    /// per second; the profile's `balloon_ease_per_s`).
    pub fn with_balloon_ease(mut self, per_s: Option<f64>) -> Self {
        self.balloon_ease_per_s = per_s.filter(|r| r.is_finite() && *r > 0.0);
        self
    }

    /// From the airframe's `flare_elevator:`; `None` unless it declares a
    /// model (every other airframe keeps [`FlarePitch`]).
    pub fn from_profile(p: Option<crate::aircraft::FlareElevatorProfile>) -> Option<Self> {
        let p = p.and_then(|p| p.valid())?;
        let gains = FlareGains::from_profile(Some(p));
        p.model.map(|m| {
            Self::new(m, gains.max, gains.min, p.tau_s.unwrap_or(5.0))
                .with_balloon_ease(gains.balloon_ease_per_s)
        })
    }

    /// A new approach: nothing written, nothing owed, nothing held.
    pub fn reset(&mut self) {
        let climb_fpm = self.climb_fpm;
        *self = Self::new(self.cfg, self.max, self.min, self.tau_s)
            .with_balloon_ease(self.balloon_ease_per_s);
        self.climb_fpm = climb_fpm;
    }

    /// The law as flown before issue #146: any climb, +1 fpm included,
    /// eased the held pull. Replays only.
    #[cfg(test)]
    fn easing_on_any_climb(mut self) -> Self {
        self.climb_fpm = 0.0;
        self
    }

    fn lag2(&self) -> f64 {
        self.cfg.lag2_s.unwrap_or(self.cfg.lag_s)
    }

    fn horizon(&self) -> f64 {
        self.cfg.horizon_s.unwrap_or(self.cfg.dead_s + 1.0)
    }

    /// The sink the ground adds at this height, fpm (positive).
    fn ground(&self, h: f64) -> f64 {
        self.cfg.ground_fpm * (1.0 - h.max(0.0) / self.cfg.ground_ft).max(0.0)
    }

    /// The elevator arriving at the aircraft at model time `t`: the last
    /// one written a dead time before it.
    fn arriving(&self, t: f64) -> f64 {
        let due = t - self.cfg.dead_s;
        self.written
            .iter()
            .take_while(|(tw, _)| *tw <= due + 1e-9)
            .last()
            .map_or(0.0, |(_, e)| *e)
    }

    /// Advance the model `dt` at height `h`.
    fn advance(&mut self, dt: f64, h: f64) {
        let n = (dt / FLARE_MODEL_STEP_S).round().max(1.0) as usize;
        let step = dt / n as f64;
        let (k, l1, l2) = (self.cfg.fpm_per_elevator, self.cfg.lag_s, self.lag2());
        let ground = self.ground(h);
        let rate = self.cfg.rate;
        let mut rate_sum = 0.0;
        for _ in 0..n {
            self.t += step;
            self.written.push_back((self.t, self.el));
            // Keep one entry at or before the dead-time boundary.
            let due = self.t - self.cfg.dead_s;
            while self.written.len() > 1 && self.written[1].0 <= due {
                self.written.pop_front();
            }
            let arriving = self.arriving(self.t);
            if rate {
                // The lags carry the RATE the column asks; the ground's
                // own rate is taken off it, not lagged.
                self.b1 += (k * arriving - self.b1) * step / l1;
                self.b2 += (self.b1 - self.b2) * step / l2;
                rate_sum += self.b2 - ground;
            } else {
                self.b1 += (k * arriving - ground - self.b1) * step / l1;
                self.b2 += (self.b1 - self.b2) * step / l2;
            }
        }
        if rate {
            self.model_rate = rate_sum / n as f64;
        }
    }

    /// The airspeed this tick, for a model that carries the speed
    /// (`fpm_per_kt`): its trend is smoothed over [`FLARE_SPEED_TREND_TAU_S`].
    /// Nothing else reads it.
    pub fn note_speed(&mut self, ias_kts: f64, dt_s: f64) {
        if !ias_kts.is_finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            return;
        }
        if let Some(p) = self.last_ias {
            let raw = ((ias_kts - p) / dt_s).clamp(-6.0, 3.0);
            self.ias_trend += (raw - self.ias_trend) * (dt_s / FLARE_SPEED_TREND_TAU_S).min(1.0);
        }
        self.last_ias = Some(ias_kts);
    }

    /// The sink the speed's trend adds over the next `ahead_s` (fpm,
    /// negative while it bleeds); nothing without `fpm_per_kt`.
    fn speed_sink_ahead(&self, ahead_s: f64) -> f64 {
        self.cfg
            .fpm_per_kt
            .map_or(0.0, |k| k * self.ias_trend.min(0.0) * ahead_s.max(0.0))
    }

    /// The model's sink and height at the horizon if `el_new` were held
    /// from now, the elevator already written arriving on schedule.
    fn predict(&self, el_new: f64, radio_ft: f64, vvi_fpm: f64, unexplained: f64) -> (f64, f64) {
        let horizon = self.horizon();
        let n = (horizon / FLARE_PREDICT_STEP_S).round().max(1.0) as usize;
        let dt = horizon / n as f64;
        let (k, l1, l2) = (self.cfg.fpm_per_elevator, self.cfg.lag_s, self.lag2());
        let (mut b1, mut b2, mut t, mut h, mut v) = (self.b1, self.b2, self.t, radio_ft, vvi_fpm);
        for _ in 0..n {
            t += dt;
            let arriving = if t - self.cfg.dead_s <= self.t {
                self.arriving(t)
            } else {
                el_new
            };
            if self.cfg.rate {
                // From the sink measured now: the column's rate, less the
                // ground's, plus the rate nothing explains.
                b1 += (k * arriving - b1) * dt / l1;
                b2 += (b1 - b2) * dt / l2;
                v += (b2 - self.ground(h) + unexplained) * dt;
            } else {
                b1 += (k * arriving - self.ground(h) - b1) * dt / l1;
                b2 += (b1 - b2) * dt / l2;
                // The speed still to bleed, as sink, from now to then.
                v = unexplained + b2 + self.speed_sink_ahead(t - self.t);
            }
            h = (h + v / 60.0 * dt).max(0.0);
        }
        (v, h)
    }

    /// No measurement this tick: nothing is decided, the model keeps time
    /// with what is on the channel, and the channel itself runs out after
    /// [`FLARE_CHANNEL_EXPIRY_S`].
    pub fn coast(&mut self, dt_s: f64, radio_ft: f64) {
        let dt = dt_s.clamp(0.02, 3.0);
        self.advance(dt, radio_ft);
        self.coasting_s += dt;
        // A rate plant's next change of sink spans the gap: no measurement.
        self.last_vvi = None;
        if self.coasting_s >= FLARE_CHANNEL_EXPIRY_S {
            self.el = 0.0;
        }
    }

    /// The flare's target at a height and groundspeed: the exponential
    /// flare to the touchdown sink, never steeper than the path. Pure.
    pub fn target_fpm(&self, radio_ft: f64, gs_kts: f64) -> f64 {
        let nominal = -(gs_kts.max(0.0) / 60.0) * gp_ft_per_nm();
        let (_, arrest) = arrest_targets_fpm(nominal);
        let touchdown = self.cfg.touchdown_fpm.map_or(arrest, |t| -t);
        // A HOLD-OFF levels at its height, then settles: the same
        // exponential, measured from the hold-off height, not the runway.
        let above = match self.cfg.hold_off_ft {
            Some(hold) => (radio_ft - hold).max(0.0),
            None => radio_ft,
        };
        flare_target_with(above, touchdown, nominal, Some(self.tau_s))
    }

    /// The airframe flies a light single's hold-off (`hold_off_ft`).
    pub fn holds_off(&self) -> bool {
        self.cfg.hold_off_ft.is_some()
    }

    /// One tick on a RATE plant (`FlareModelProfile::rate`, the A330's
    /// sidestick): the elevator whose rate, held from when it can arrive,
    /// puts the sink at the horizon on the flare's target for the height it
    /// will have then. The rate the model does not explain (the speed
    /// bleeding off at idle, the trim the autopilot left) is measured from
    /// the sink's own change and carried through the prediction. Neutral
    /// HOLDS a sink on this plant, so there is no ratchet and no "never
    /// push a climb": the column comes forward to end a float or a balloon,
    /// as far as the airframe's `max_push`. The model's pipe carries what
    /// was written, as on the standing plant.
    fn step_rate(&mut self, radio_ft: f64, gs_kts: f64, vvi_fpm: f64, dt: f64) -> FlareModelOut {
        let unexplained = match self.last_vvi {
            Some(prev) => {
                let now = (vvi_fpm - prev) / dt - self.model_rate;
                match self.unexplained {
                    Some(u) => u + (now - u) * (dt / self.cfg.observer_s).min(1.0),
                    None => now,
                }
            }
            None => self.unexplained.unwrap_or(0.0),
        };
        self.last_vvi = Some(vvi_fpm);
        self.unexplained = Some(unexplained);
        let (v0, ahead) = self.predict(0.0, radio_ft, vvi_fpm, unexplained);
        let (v1, _) = self.predict(1.0, radio_ft, vvi_fpm, unexplained);
        let target = self.target_fpm(ahead, gs_kts);
        let per_unit = v1 - v0;
        let ask = if per_unit > 1e-6 {
            ((target - v0) / per_unit).clamp(self.min, self.max)
        } else {
            0.0
        };
        self.held = None;
        self.el = ask;
        FlareModelOut {
            elev: ask,
            target_fpm: target,
            ahead_ft: ahead,
            unexplained_fpm: unexplained,
            ask,
        }
    }

    /// One tick: the elevator for this measured sink. `flare` is the flare
    /// proper (as against the roundout above it, which stays free both
    /// ways); `rough` is the final's air ([`FinalAir::rough`]). See
    /// [`FlareModel`] for what each holds.
    pub fn step(
        &mut self,
        radio_ft: f64,
        gs_kts: f64,
        vvi_fpm: f64,
        dt_s: f64,
        flare: bool,
        rough: bool,
    ) -> FlareModelOut {
        let dt = dt_s.clamp(0.02, 3.0);
        self.advance(dt, radio_ft);
        self.coasting_s = 0.0;
        if self.cfg.rate {
            return self.step_rate(radio_ft, gs_kts, vvi_fpm, dt);
        }
        let now = vvi_fpm - self.b2;
        let unexplained = match self.unexplained {
            Some(u) => u + (now - u) * (dt / self.cfg.observer_s).min(1.0),
            None => now,
        };
        self.unexplained = Some(unexplained);
        // Linear in the new elevator: two predictions give the answer.
        let (v0, ahead) = self.predict(0.0, radio_ft, vvi_fpm, unexplained);
        let (v1, _) = self.predict(1.0, radio_ft, vvi_fpm, unexplained);
        let target = self.target_fpm(ahead, gs_kts);
        let per_unit = v1 - v0;
        let ask = if per_unit > 1e-6 {
            ((target - v0) / per_unit).clamp(self.min, self.max)
        } else {
            0.0
        };
        let elev = if flare {
            // The roundout's last ask is where the flare starts.
            let held = self.held.unwrap_or(ask);
            let next = if vvi_fpm > self.climb_fpm {
                // A BALLOON, measured: the pull has beaten the ground and the
                // aircraft climbs. Eased toward the ask, never a push. A
                // near-level float is not one: +1 fpm at 22.9 ft eased the
                // held 0.28 to 0.20 at EGBB 15 (2026-09-28), a dead time
                // before the sink the retard would rebuild.
                let eased = ask.max(0.0);
                match self.balloon_ease_per_s {
                    Some(rate) => eased.max(held - rate * dt),
                    None => eased,
                }
            } else if (rough && self.cfg.rough_hold != Some(false))
                || vvi_fpm > -FLARE_HOLD_MIN_SINK_FPM
            {
                // Rough air, or a near-level float: the pull stays, and more
                // comes only when the prediction asks for it.
                ask.max(held)
            } else {
                // Smooth air and a real sink: the prediction flies it.
                ask
            };
            self.held = Some(next);
            next
        } else {
            self.held = None;
            // The roundout is free both ways, except that a measured climb
            // is a balloon and a balloon never pushes (LEMG 12, the SF50,
            // 2026-09-28: -0.05 at +214 fpm).
            if vvi_fpm > self.climb_fpm {
                ask.max(0.0)
            } else {
                ask
            }
        };
        // The model's pipe carries what was WRITTEN, the held pull included.
        self.el = elev;
        FlareModelOut {
            elev,
            target_fpm: target,
            ahead_ft: ahead,
            unexplained_fpm: unexplained,
            ask,
        }
    }
}

/// The final's air, as the autopilot felt it: how far the sink strays from
/// the one asked for (fpm rms about its own mean, so a steady tracking
/// offset is not turbulence), from [`FINAL_AIR_FROM_FT`] down to the arrest
/// gate, only while the autopilot is confirmed. The ten calm LEMG 13 finals
/// of 2026-09-28 strayed 10 to 23 fpm; the three gusty ones after them 72,
/// 83 and 105, and the 2026-09-26 finals in 8 to 12 kt of swinging wind 29
/// to 110. Too few samples (a close-in reposition, failed reads) is no
/// measurement, and no measurement is smooth air: the flare the calm
/// landings flew.
#[derive(Debug, Default, Clone, Copy)]
pub struct FinalAir {
    n: u32,
    sum: f64,
    sum_sq: f64,
}

/// The final's air is measured from this radio height down, ft.
pub const FINAL_AIR_FROM_FT: f64 = 400.0;
/// ...needs this many samples (about five seconds of ticks)...
const FINAL_AIR_MIN_SAMPLES: u32 = 15;
/// ...and is rough past this, fpm rms.
pub const ROUGH_AIR_FPM: f64 = 40.0;

impl FinalAir {
    /// One tick's sink against the one asked for; a read that failed is no
    /// sample.
    pub fn note(&mut self, vvi_fpm: f64, asked_fpm: f64) {
        let e = vvi_fpm - asked_fpm;
        if e.is_finite() {
            self.n += 1;
            self.sum += e;
            self.sum_sq += e * e;
        }
    }

    /// The stray, fpm rms about its mean; `None` with too few samples.
    pub fn rms_fpm(&self) -> Option<f64> {
        if self.n < FINAL_AIR_MIN_SAMPLES {
            return None;
        }
        let n = f64::from(self.n);
        let mean = self.sum / n;
        Some((self.sum_sq / n - mean * mean).max(0.0).sqrt())
    }

    pub fn rough(&self) -> bool {
        self.rms_fpm().is_some_and(|r| r > ROUGH_AIR_FPM)
    }
}

/// Who has the servos this tick. The arrest flies the channels with the
/// autopilot's servos OFF (the pitch servo writes the yoke back over the
/// channel otherwise; LFPG 09R, 2026-09-22), takes them at the roundout
/// gate and hands them back only the tick the arrest ends in a go-around
/// (a balk, the missed approach, a pattern): a balloon back over the band
/// is still the arrest ([`hold_roundout`]; LEMG 12, 2026-09-28, handed back
/// at 50.5 ft and dived to -814 fpm). The rollout latch never reaches this decision:
/// it disconnects on its own and finds the servos already off.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArrestHands {
    /// The autopilot flies; nothing to do.
    Off,
    /// This tick: servos off, the channels take over.
    Take,
    /// The arrest has the servos and keeps flying the channels.
    Keep,
    /// This tick: servos on, modes re-engaged, the channels expire.
    Return,
}

pub fn arrest_hands_step(taken: bool, arresting: bool) -> ArrestHands {
    match (taken, arresting) {
        (false, false) => ArrestHands::Off,
        (false, true) => ArrestHands::Take,
        (true, true) => ArrestHands::Keep,
        (true, false) => ArrestHands::Return,
    }
}

/// Inside this distance to the threshold the final's heading is the
/// short-final law's ([`short_final_cut_deg`]), not the pursuit's.
pub const SHORT_FINAL_NM: f64 = 0.30;
/// The short final closes the offset over this many seconds...
const SHORT_FINAL_CLOSE_S: f64 = 6.0;
/// ...judged this far ahead on the drift over the ground...
const SHORT_FINAL_LEAD_S: f64 = 3.0;
/// ...with a cut no bigger than this, degrees.
const SHORT_FINAL_MAX_CUT_DEG: f64 = 3.0;

/// The heading cut off the runway course on the last 0.3 nm, degrees (left
/// negative), from the offset (`xtk_m`, right positive) and the drift over
/// the ground (`side_m_s`, right positive, from the track).
///
/// It was 80 degrees per nm of offset and nothing else: 0.04 degrees a
/// metre, an 18 s closure with no damping, where a degree of track error
/// is 1.3 m/s at 140 kt. LEMG 13 (2026-09-26, 20:17Z): from 170 ft the 737
/// tracked about a degree right of the runway (the track trim still held
/// +0.45 degrees learnt against an earlier left drift, and the 15 s crab
/// filter was still holding the wind of half a minute before, while the
/// wind swung round), the law answered 8.9 m of offset with 0.4 degrees,
/// and the arrest gate at 79 ft found it 9 to 12 m right. Here the offset
/// closes in [`SHORT_FINAL_CLOSE_S`] and the drift is counted
/// [`SHORT_FINAL_LEAD_S`] ahead, so a track error is answered the moment it
/// shows, not once it has become metres. Pure.
pub fn short_final_cut_deg(xtk_m: f64, side_m_s: f64, gs_kts: f64) -> f64 {
    if !xtk_m.is_finite() || !side_m_s.is_finite() {
        return 0.0;
    }
    let v = (if gs_kts.is_finite() { gs_kts } else { 0.0 } * 0.514_444).max(30.0);
    let ahead = xtk_m + side_m_s * SHORT_FINAL_LEAD_S;
    (-(ahead / (v * SHORT_FINAL_CLOSE_S)).atan().to_degrees())
        .clamp(-SHORT_FINAL_MAX_CUT_DEG, SHORT_FINAL_MAX_CUT_DEG)
}

/// How quickly the short final learns what the track does against its bug,
/// seconds.
const SHORT_FINAL_BIAS_TAU_S: f64 = 2.0;

/// The short final's bug, as an offset from the runway course (degrees):
/// the [`short_final_cut_deg`] track it wants, less what the track has been
/// doing against the bug it was given (the autopilot's own heading offset,
/// a wind the 15 s crab filter has not caught up with, a stale track trim),
/// learnt over [`SHORT_FINAL_BIAS_TAU_S`]. On the replayed LEMG 13 ticks a
/// law without it still reached the gate 5 m right: the bug-to-track error
/// was a steady degree, and a P law holds a steady offset against a steady
/// error however it is damped. Seeded with the bug the pursuit was flying,
/// so the hand-over is continuous.
///
/// What the track does against the bug is only a BIAS while the aircraft is
/// flying that bug steadily. Learnt while it is still turning onto it, the
/// autopilot's own lag reads as bias: LEMG 13 (sortie 1790591256-full_auto,
/// 2026-09-28) the short final began 32 m left at 0.29 nm, the cut went to
/// its 3 degree cap, the track lagged it by 5.4 degrees a tick later, the 2 s
/// learner took that for a bias and ran to its 5 degree clamp, and the bug
/// stood 8 degrees right of the runway: 16 degrees of bank at 90 ft, across
/// the centreline at the threshold and 37 m right at the wheels. The same
/// shape at 11 degrees of bank in the circuit before it (1790590663). So a
/// tick banked past [`SHORT_FINAL_LEARN_BANK_DEG`], or one whose track is
/// more than [`SHORT_FINAL_LEARN_ERR_DEG`] from what the bias already
/// explains, teaches nothing, and the bias is bounded by what one really was
/// (a degree, 2026-09-26) with room: [`SHORT_FINAL_MAX_BIAS_DEG`].
#[derive(Debug, Clone, Copy)]
pub struct ShortFinal {
    bias_deg: f64,
    prev_bug_deg: f64,
}

/// The most bias the short final carries, degrees.
const SHORT_FINAL_MAX_BIAS_DEG: f64 = 2.5;
/// Banked past this, the track is turning and teaches no bias, degrees.
const SHORT_FINAL_LEARN_BANK_DEG: f64 = 5.0;
/// A track further than this from the bug plus the bias learnt is a turn
/// still under way, not a bias, degrees.
const SHORT_FINAL_LEARN_ERR_DEG: f64 = 3.0;

impl ShortFinal {
    /// `entry_bug_off_deg`: the bug (true) less the course on the tick the
    /// short final begins; `track_off_deg`: the track less the course then.
    pub fn new(entry_bug_off_deg: f64, track_off_deg: f64) -> Self {
        let seed = track_off_deg - entry_bug_off_deg;
        Self {
            bias_deg: if seed.is_finite() {
                seed.clamp(-SHORT_FINAL_MAX_BIAS_DEG, SHORT_FINAL_MAX_BIAS_DEG)
            } else {
                0.0
            },
            prev_bug_deg: entry_bug_off_deg,
        }
    }

    /// One tick: the bug offset from the course, degrees. `bank_deg` is the
    /// bank read this tick (`None` when unread: judged on the track alone).
    pub fn step(
        &mut self,
        xtk_m: f64,
        track_off_deg: f64,
        gs_kts: f64,
        dt_s: f64,
        bank_deg: Option<f64>,
    ) -> f64 {
        if !track_off_deg.is_finite() || !xtk_m.is_finite() {
            return self.prev_bug_deg;
        }
        let dt = dt_s.clamp(0.05, 1.0);
        let e = track_off_deg - self.prev_bug_deg;
        let turning = bank_deg.is_some_and(|b| b.abs() > SHORT_FINAL_LEARN_BANK_DEG)
            || (e - self.bias_deg).abs() > SHORT_FINAL_LEARN_ERR_DEG;
        if !turning {
            let a = (dt / SHORT_FINAL_BIAS_TAU_S).min(1.0);
            self.bias_deg = (self.bias_deg + (e - self.bias_deg) * a)
                .clamp(-SHORT_FINAL_MAX_BIAS_DEG, SHORT_FINAL_MAX_BIAS_DEG);
        }
        let v = (gs_kts.max(0.0) * 0.514_444).max(30.0);
        let side = v * track_off_deg.to_radians().sin();
        let bug = short_final_cut_deg(xtk_m, side, gs_kts) - self.bias_deg;
        self.prev_bug_deg = bug;
        bug
    }
}

/// The closure the arrest asks for: the offset over this many seconds.
const ARREST_CLOSE_S: f64 = 4.0;
/// ...and never faster than this sideways, m/s.
const ARREST_CLOSE_MAX_M_S: f64 = 2.5;
/// How quickly a sideways-velocity error is taken out by the bank, seconds.
const ARREST_VEL_TAU_S: f64 = 1.5;
/// The steepest bank over the runway, degrees, tapering toward the wheels.
const ARREST_BANK_MAX_DEG: f64 = 5.0;
const ARREST_BANK_WHEELS_DEG: f64 = 2.0;
const G_M_S2: f64 = 9.81;
/// The sideways deceleration the closure is sized to be stopped by, m/s^2:
/// well inside the 0.5 a 3-degree bank gives, since the bank takes seconds
/// to get there.
const ARREST_STOP_M_S2: f64 = 0.2;
/// How far ahead the current bank's push on the drift is counted, seconds:
/// the 737's roll lag at the arrest's aileron (LEMG 13, 2026-09-26).
const ARREST_BANK_LEAD_S: f64 = 1.0;
/// Bank error for full aileron in the arrest (see `WingsLevel::stiff`).
pub(crate) const ARREST_FULL_AIL_BANK_DEG: f64 = 6.0;

/// The roll loop the arrest flies its bank target through: the airframe's
/// declared roll-rate plant (`arrest_roll:`, the A330), else the stiff law.
pub(crate) fn arrest_wings() -> crate::autotakeoff::WingsLevel {
    match crate::aircraft::arrest_roll() {
        Some(p) => crate::autotakeoff::WingsLevel::rate_plant(p),
        None => crate::autotakeoff::WingsLevel::stiff(ARREST_FULL_AIL_BANK_DEG),
    }
}

/// The bank the arrest flies with the servos out, from the centreline offset
/// (`xtk_m`, right positive) and the SIDEWAYS VELOCITY over the runway
/// (`side_m_s`, right positive: the ground track's angle off the course
/// times the groundspeed). A velocity loop inside a position loop: the offset
/// asks for a closure rate, the bank takes out the difference.
///
/// It flew a HEADING (the rollout's centreline law plus the crab), and over
/// the runway a heading is the wrong thing to fly. The rollout law is sized
/// for a closure time of about nine seconds at 140 kt, longer than the whole
/// arrest; and the 737's nose did not follow its bank: LEMG 13 (2026-09-26,
/// 17:43Z) went into the gate 3.9 m right and crossing left at 2.3 m/s, the
/// bank built to 4.6 degrees right while the heading fell from 130.5 to
/// 129.1, the track came right only 1 degree, and it touched down 15.3 m
/// left and rolled out 16.8 m left. The same crossing flew four landings in
/// a row (100 ft to the wheels: +5.6 to -11.9, +5.9 to -14.7, +13.0 to
/// -5.6, +7.2 to -10.4). The track is what moves the aircraft over the
/// paint, so the track is what this closes.
///
/// The closure goes to ZERO at the centreline and the bank's lag is led.
/// LEMG 13 (2026-09-26, 20:17Z) went into the gate 11.9 m right: the old
/// capped rate (the offset over 4 s, at most 2.5 m/s) built the drift to
/// 2.65 m/s, the bank target reversed at 42 ft but the 737 rolls at about
/// 2 deg/s on the aileron `WingsLevel` gives it, so the bank only crossed
/// wings level 2 s later with the aircraft still 2.4 m right at 2.65 m/s;
/// the shrinking cap then allowed 3 degrees to stop that, and it touched
/// down 6.5 m left. So the wanted rate is also what a gentle deceleration
/// ([`ARREST_STOP_M_S2`]) can take off in the offset left, and the drift
/// the bank is judged against is the one the CURRENT bank will have made
/// [`ARREST_BANK_LEAD_S`] from now.
pub fn arrest_bank_target(xtk_m: f64, side_m_s: f64, phi_deg: f64, radio_ft: f64) -> f64 {
    if !xtk_m.is_finite() || !side_m_s.is_finite() {
        return 0.0;
    }
    let phi = if phi_deg.is_finite() { phi_deg.clamp(-30.0, 30.0) } else { 0.0 };
    let off = xtk_m.abs();
    let rate = (off / ARREST_CLOSE_S)
        .min((2.0 * ARREST_STOP_M_S2 * off).sqrt())
        .min(ARREST_CLOSE_MAX_M_S);
    let want = -xtk_m.signum() * rate;
    let side_ahead = side_m_s + G_M_S2 * phi.to_radians().tan() * ARREST_BANK_LEAD_S;
    let bank = ((want - side_ahead) / (G_M_S2 * ARREST_VEL_TAU_S)).atan().to_degrees();
    let cap = if radio_ft.is_finite() {
        (ARREST_BANK_WHEELS_DEG + radio_ft.max(0.0) / 10.0).min(ARREST_BANK_MAX_DEG)
    } else {
        ARREST_BANK_WHEELS_DEG
    };
    bank.clamp(-cap, cap)
}

/// The sideways velocity over the runway, m/s right positive, from the sim's
/// ground track (`hpath`, true) against the runway course (true). `None`
/// without a track read: no measurement, and the caller falls back to the
/// smoothed cross-track rate.
pub fn side_velocity_m_s(track_true: Option<f64>, course_true: f64, gs_kts: f64) -> Option<f64> {
    let trk = track_true.filter(|t| t.is_finite())?;
    if !gs_kts.is_finite() {
        return None;
    }
    let off = crate::angles::signed_diff_deg(trk, course_true).to_radians();
    Some(gs_kts * 0.514_444 * off.sin())
}

/// Outside a landing arrest (`arresting` false) the dial is the target, as it
/// always was. Inside one it leads the target by the autopilot's outstanding
/// error - see [`VS_LEAD_GAIN`] - against the sink PREDICTED `trend_seconds`
/// ahead (`trend_fpm_s`, the vertical speed's rate of change): on a
/// second-order autopilot the trend is the damping the bare error lacks.
/// The gain, the cap and the trend horizon are the airframe's
/// ([`crate::aircraft::ArrestLeadProfile`]; the default is the 172's).
pub fn vs_dial_fpm(
    target_fpm: f64,
    actual_fpm: f64,
    trend_fpm_s: f64,
    arresting: bool,
    lead: crate::aircraft::ArrestLeadProfile,
) -> f64 {
    if !arresting {
        return target_fpm;
    }
    let trend = if trend_fpm_s.is_finite() {
        trend_fpm_s
    } else {
        0.0
    };
    let predicted = actual_fpm + lead.trend_seconds * trend;
    (target_fpm + lead.gain * (target_fpm - predicted)).clamp(-1200.0, lead.max_climb_fpm)
}

/// May the roundout own the descent from HERE? Only where a
/// [`ROUNDOUT_VS_FPM`] descent still REACHES the pavement. At or past the
/// threshold, always — there the unsigned glidepath distance grows again and
/// alt_err stops meaning anything, which is the arithmetic the roundout
/// exists to pre-empt. Short of it, the sink from this height must cover the
/// metres to the threshold: 40 ft at 0.2 nm closes (a normal crossing), and
/// 30 ft at 0.3 nm does not — that aircraft is dragged low, and a commanded
/// descent grounds it ~190 m into the approach lights where the level-hold
/// would have let the path come down to it (audit, 2026-08-02). A failed
/// (zero) groundspeed read denies the roundout, which falls back to the
/// level-hold: short of the runway, level is always the safe reading.
fn roundout_reaches_pavement(along_nm: f64, radio_alt_ft: f64, gs_kts: f64) -> bool {
    if along_nm <= 0.0 {
        return true;
    }
    let sink_s = radio_alt_ft / (ROUNDOUT_VS_FPM.abs() / 60.0);
    let reach_m = gs_kts * 0.514_444 * sink_s;
    along_nm * M_PER_NM <= reach_m
}
/// ...and only within this much cross-track of the centreline (~37 m — about a
/// runway half-width plus margin). Beyond it the aircraft is over grass, and
/// touching down there is how the LXGB 09 rollout ended in wreckage
/// (2026-07-22). A balked landing straight ahead is always the better trade.
const FLARE_MAX_XTK_NM: f64 = 0.02;

/// How far inside the pavement's edge a landing's centreline offset must stay,
/// metres: a jet's main gear (the 737's is 5.7 m across, 2.9 m either side)
/// plus a margin for the arrest's last closure.
const LANDING_EDGE_MARGIN_M: f64 = 7.5;
/// ...and how close to the edge the flare may still be at the flare gate: the
/// gear on the pavement with a metre to spare.
const FLARE_EDGE_MARGIN_M: f64 = 4.0;

/// The most centreline offset a landing may carry to the wheels on a runway
/// `width_m` wide, metres: the stabilised gates below the terrain floor and
/// the low gate judge against it. `FLARE_MAX_XTK_NM`'s 37 m is a 60 m strip's
/// half-width plus margin, and on LEMG 13's 45 m it is 14.5 m past the edge:
/// the 737 flew the last 500 ft 30 to 50 m off, the gates never fired, and it
/// touched down 37 m right of the centreline, in the grass (sortie
/// 1790591256-full_auto, 2026-09-28). Unknown width keeps the old 37 m; a
/// narrow strip keeps at least a quarter of its width.
pub fn landing_xtk_limit_m(width_m: Option<f64>) -> f64 {
    let legacy = FLARE_MAX_XTK_NM * M_PER_NM;
    match width_m.filter(|w| w.is_finite() && *w > 1.0) {
        Some(w) => ((w / 2.0 - LANDING_EDGE_MARGIN_M).max(w / 4.0)).min(legacy),
        None => legacy,
    }
}

/// The offset the FLARE gate balks beyond, metres: the gear still on the
/// pavement ([`FLARE_EDGE_MARGIN_M`] inside the edge), never tighter than the
/// landing limit, never looser than the old 37 m.
pub fn flare_xtk_limit_m(width_m: Option<f64>) -> f64 {
    let legacy = FLARE_MAX_XTK_NM * M_PER_NM;
    match width_m.filter(|w| w.is_finite() && *w > 1.0) {
        Some(w) => (w / 2.0 - FLARE_EDGE_MARGIN_M)
            .max(landing_xtk_limit_m(Some(w)))
            .min(legacy),
        None => legacy,
    }
}

/// Inside the FAP and within this of the runway's course, an approach that
/// needs repositioning goes around first instead of turning at once.
const LATE_GO_AROUND_TRACK_DEG: f64 = 45.0;

/// Is a reposition this late a go-around? `along_nm` is the distance still
/// to the threshold along the approach (positive short of it). Pure.
pub fn late_go_around(along_nm: f64, track_off_deg: f64) -> bool {
    along_nm > 0.0 && along_nm < FAP_NM && track_off_deg.abs() < LATE_GO_AROUND_TRACK_DEG
}

/// Flap setting for the approach: half inside 6 nm, full a mile before the
/// stabilised gate ([`landing_flap_nm`], about 4 nm on a 3 degree path), but
/// full only once established ON the glidepath. While holding level BELOW the
/// path waiting to capture (`alt_err` well negative), full flaps are an energy
/// trap: the LERL 172 approach dragged along at 50 to 57 kt against a 65-kt
/// target with the throttle governor already at its ceiling, and arrived at
/// the flare with nothing left. Half flaps keep the drag survivable until the
/// path is captured.
///
/// Full used to wait for 3 nm, which on a 3 degree path IS the 1,000 ft gate:
/// every final on file reached the gate with half flap out and the speed
/// still on the half flap's floor (the 737 at 169 kt on EGBB 33, sortie
/// 1790759458-full_auto, 2026-09-30), so no final could be configured by the
/// gate (`stable_approach`, #199).
pub fn flap_schedule(dist_nm: f64, alt_err_ft: f64) -> f64 {
    if dist_nm < landing_flap_nm() && alt_err_ft > -150.0 {
        1.0
    } else if dist_nm < 6.0 {
        0.5
    } else {
        0.0
    }
}

/// Flap schedule for the leg being flown. A normal downwind inside the
/// configuration area uses half flap, but a distant outbound reversal is a
/// reposition rather than a traffic-pattern leg and remains clean (#211).
pub fn approach_leg_flap_schedule(dist_nm: f64, alt_err_ft: f64, outbound: bool) -> f64 {
    if outbound {
        if dist_nm < 6.0 {
            0.5
        } else {
            0.0
        }
    } else {
        flap_schedule(dist_nm, alt_err_ft)
    }
}

/// The landing flap is selected this far ahead of the stabilised gate, nm:
/// time for it to run and for the speed to come back to its target before
/// the gate judges both.
const LANDING_FLAP_LEAD_NM: f64 = 1.0;

/// Where the approach selects its landing flap, nm from the threshold: a
/// mile before the instrument gate on this approach's path (3.98 nm at 3
/// degrees). Never beyond the half-flap point.
pub fn landing_flap_nm() -> f64 {
    let gate_nm = crate::stable_approach::GateKind::Instrument.gate_ft() / gp_ft_per_nm()
        - TOUCHDOWN_AIM_NM;
    (gate_nm + LANDING_FLAP_LEAD_NM).clamp(3.0, 5.5)
}

/// Inside this of the centreline, nm...
pub const ESTABLISHED_PATH_XTK_NM: f64 = 0.25;
/// ...closing or opening it no faster than this, nm/min (10 degrees at
/// 200 kt): established on the final path.
pub const ESTABLISHED_PATH_XTK_RATE_NM_MIN: f64 = 0.6;

/// Established on the final's path beyond the FAP (see `guidance_on`):
/// Intercept by phase only because the lateral still steers for the FAP,
/// on the approach side, on the centreline and tracking it, on or above the
/// path. Pure.
pub fn established_on_final_path(
    phase: ApproachPhase,
    balk: Option<&'static str>,
    along_nm: f64,
    xtk_nm: f64,
    xtk_rate_nm_min: f64,
    alt_err_ft: f64,
) -> bool {
    phase == ApproachPhase::Intercept
        && balk.is_none()
        && along_nm > FAP_NM
        && xtk_nm.abs() <= ESTABLISHED_PATH_XTK_NM
        && xtk_rate_nm_min.abs() <= ESTABLISHED_PATH_XTK_RATE_NM_MIN
        && alt_err_ft > FLOOR_BELOW_PATH_FT
}

/// The energy plan's geometry for one tick: the route to the gate, nm, and
/// the path's share of it. A pattern (`pattern`: its track to go and the
/// distance from the threshold it ends at) flies level to its end and the
/// path from there; a straight-in on or above the path is on it; one below
/// it flies level until the path comes down to it. Pure.
pub fn energy_geometry(
    pattern: Option<(f64, f64)>,
    dist_nm: f64,
    above_path_ft: f64,
    path_ft_per_nm: f64,
    gate_nm: f64,
) -> (f64, f64) {
    match pattern {
        Some((to_go_nm, end_nm)) => {
            let path = (end_nm - gate_nm).max(0.0);
            (to_go_nm.max(0.0) + path, path)
        }
        None => {
            let s = (dist_nm - gate_nm).max(0.0);
            if above_path_ft < -50.0 {
                // The path reaches our height (-above) / slope further in.
                let meet_nm = dist_nm - (-above_path_ft) / path_ft_per_nm.max(1.0);
                (s, (meet_nm - gate_nm).clamp(0.0, s))
            } else {
                (s, s)
            }
        }
    }
}

/// The flap the plan's configuration asks for: its own, except the landing
/// flap below the path (full flap held level under the path is the LERL
/// energy trap: [`flap_schedule`]). Pure.
pub fn plan_flap_wanted(plan_flap: f64, alt_err_ft: f64) -> f64 {
    if plan_flap >= LANDING_FLAP_RATIO - 1e-6 && alt_err_ft <= -150.0 {
        0.5f64.min(plan_flap)
    } else {
        plan_flap
    }
}

/// Latched low go-around: a missed approach called BELOW the terrain floor is
/// flown wings-level STRAIGHT AHEAD at full power until clear of the ground —
/// never the immediate turn-back the intercept guidance commands. Seen live
/// 2026-07-14 (LXGB 09): overflew the numbers at ~57 ft, the guidance flipped
/// the heading command 178° toward the FAP, the flare branch re-fired 0.3 nm
/// PAST the threshold (`dist_nm` is unsigned, so the flare gate re-arms on the
/// departure side) and pulled the power, then speed protection commanded
/// -590 fpm at ground level — impact 200 m south of the strip. The latch owns
/// the whole low go-around: it arms when Final/Flare degrades to Intercept
/// below [`TERRAIN_FLOOR_AGL_FT`], holds through every phase flicker the raw
/// guidance produces down there, and releases at the floor, where the normal
/// intercept guidance takes over the turn back to the FAP.
/// Where a balked landing is in the missed approach. Two stages, because a
/// missed approach is a PROCEDURE, not a moment: releasing the aircraft back to
/// the ordinary intercept guidance the instant it was clear of the ground left
/// it 800 ft over the far end with the field behind it and no plan, and the
/// stateless guidance then argued with itself — see `descent_limit_fpm` for the
/// vertical half of the Luton orbit (2026-07-28) and `Reposition` for the
/// lateral half.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissedStage {
    /// Straight ahead on runway heading, climbing away from the ground. The one
    /// direction a runway promises is flyable.
    Climb,
    /// Repositioning: outbound to the join point at platform height, held there
    /// until established OUTSIDE the FAP on the approach side — which is simply
    /// where a normal approach begins.
    ///
    /// This stage is what makes the decision STICKY, and stickiness is the
    /// whole point. The ordinary guidance picks its target from the CURRENT
    /// altitude error, so at reposition height the choice flips every time the
    /// aircraft crosses the distance where the path is 500 ft below it: turn
    /// out, fly out, the error shrinks, turn back in, the error grows, turn out
    /// again. At Luton that oscillation ran between 1.1 and 4.1 nm for three
    /// and a half minutes with a landing clearance live and the tower working
    /// arrivals underneath. An outbound leg has to run to completion.
    Reposition,
}

#[derive(Debug, Default)]
pub struct GoAroundLatch {
    stage: Option<MissedStage>,
    /// How many go-arounds this approach has flown — a runaway backstop. An
    /// approach that keeps missing (arrives too fast/high to capture, as the
    /// LXGB Twin Otter did off the tight circuit) would otherwise firewall a
    /// climb, release at 800 ft, re-intercept, miss again — forever, accelerating
    /// toward Vne. After [`MAX_GO_AROUNDS`] the loop gives the aircraft back.
    attempts: u32,
    /// The reposition is being flown as a pattern by the loop
    /// ([`crate::reversal`]) and ends when the pattern does
    /// ([`GoAroundLatch::finish_reposition`]), not at the join point: the
    /// pattern's downwind passes abeam it, outbound, inside the old release
    /// test's half mile on a light single.
    hold: bool,
    /// An unstabilised go-around climbs over the gate it failed before the
    /// approach may be judged again: feet above the runway, and the last
    /// height read (`note_height_aal`).
    climb_floor_aal_ft: Option<f64>,
    height_aal_ft: Option<f64>,
}

/// Over the gate a go-around failed, feet: the next approach crosses the
/// gate descending, never from below it.
pub const GATE_CLIMB_MARGIN_FT: f64 = 200.0;

/// Give up (level off, hand to the pilot) after this many go-arounds on one
/// approach rather than loop a missed approach until the fuel runs out.
pub const MAX_GO_AROUNDS: u32 = 2;
// The counter above cannot catch a lost reposition, because a second
// go-around needs another approach to fail first. A single missed approach
// is given up when it stops making progress (`reversal::MissedWatch`,
// 2026-09-30), never on a flat clock: 300 s expired with EGHI's Baron
// compliant mid-leg (2026-08-02), and the 600 s that replaced it gave up an
// SR22 on LEMG 13's terrain-raised pattern with the path a few seconds ahead
// (iMac 2026-09-30).

impl GoAroundLatch {
    pub fn new() -> Self {
        Self::default()
    }

    /// Go-arounds flown so far this approach.
    pub fn attempts(&self) -> u32 {
        self.attempts
    }

    /// Is a missed approach being flown right now?
    pub fn active(&self) -> bool {
        self.stage.is_some()
    }

    /// ...and still in its straight-ahead climb away from the ground?
    pub fn climbing(&self) -> bool {
        self.stage == Some(MissedStage::Climb)
    }

    /// The loop flies the reposition as a pattern: hold the stage until told.
    pub fn hold_reposition(&mut self) {
        self.hold = true;
    }

    /// The pattern is complete and the aircraft is inbound on the final: the
    /// missed approach is over and the ordinary approach has it.
    pub fn finish_reposition(&mut self) {
        self.hold = false;
        if self.stage == Some(MissedStage::Reposition) {
            self.stage = None;
        }
    }

    /// A controller ORDERED a go-around: fly it, now, from wherever we are.
    ///
    /// Every other entry to a missed approach is something the aircraft
    /// noticed about itself (off the centreline at flare height, past the
    /// numbers still airborne). This one comes from outside and outranks all
    /// of it — the tower can see the runway and we cannot. Returns whether
    /// this call started one, so the caller records it once.
    ///
    /// Malaga, 2026-08-05: "cleared to land" at 13:41:06 and "GO AROUND,
    /// traffic on the runway" at 13:41:07. The order was published on the
    /// party line and SPOKEN, and nothing consumed it — the approach flew on
    /// for another seventy seconds, past a 0.5 nm same-altitude conflict, and
    /// only balked because it happened to arrive high. The clearance the
    /// order voided was never re-earned, so the aircraft then landed without
    /// one, could not vacate for want of it, and sat on the runway with an
    /// arrival inside two miles. One unread flag cost all of that.
    ///
    /// Ignored once on the ground: a landing that has already happened is not
    /// a landing to abandon, and the rollout is the safer place to be.
    pub fn order(&mut self, on_ground: bool) -> bool {
        if on_ground || self.stage.is_some() {
            return false;
        }
        self.stage = Some(MissedStage::Climb);
        self.attempts += 1;
        true
    }

    /// An UNSTABILISED approach's go-around: [`Self::order`], and the climb
    /// holds until the aircraft is [`GATE_CLIMB_MARGIN_FT`] over `gate_ft`
    /// above the runway as well as clear of the ground. SR22 LEMG 13 (iMac
    /// 2026-09-30, 1790792031): placed diving, it failed the 1,000 ft gate
    /// at 4 nm below the path; over the hills the radio already read 800 ft,
    /// so the climb ended the next tick, the reposition released at once (the
    /// path's join point ahead of an aircraft under it), the gate judged the
    /// same final again, and three go-arounds in 8 s gave it up airborne.
    pub fn order_over_gate(&mut self, on_ground: bool, gate_ft: f64) -> bool {
        let started = self.order(on_ground);
        if started {
            self.climb_floor_aal_ft = Some(gate_ft + GATE_CLIMB_MARGIN_FT);
        }
        started
    }

    /// The height above the runway this tick, for the gate's climb floor.
    pub fn note_height_aal(&mut self, aal_ft: f64) {
        self.height_aal_ft = aal_ft.is_finite().then_some(aal_ft);
    }

    /// Advance the latch one tick: `prev`/`now` are the raw guidance phases,
    /// `radio_ft` the radio altitude, `dist_nm` the distance to the threshold,
    /// `along_nm` the signed along-track (positive = approach side),
    /// `join_nm` the [`join_point_nm`] the reposition leg is flying out to
    /// and `xtk_nm` the cross-track from the extended centreline.
    /// Returns the stage being flown, `None` when the approach is the pilot's
    /// normal one again.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        prev: Option<ApproachPhase>,
        now: ApproachPhase,
        radio_ft: f64,
        dist_nm: f64,
        along_nm: f64,
        join_nm: f64,
        xtk_nm: f64,
    ) -> Option<MissedStage> {
        match self.stage {
            // Clear of the ground — stop climbing straight ahead and start
            // repositioning. 800 ft, not the 500 ft terrain floor: the turn
            // back is the leg that crosses whatever the runway points away
            // from, and releasing at 500 put the LXGB go-around eastbound at
            // the Rock with no height in hand (2026-07-22).
            Some(MissedStage::Climb)
                if radio_ft >= GO_AROUND_TURN_AGL_FT
                    && self
                        .climb_floor_aal_ft
                        .is_none_or(|f| self.height_aal_ft.is_some_and(|h| h >= f)) =>
            {
                self.stage = Some(MissedStage::Reposition);
                self.climb_floor_aal_ft = None;
            }
            // Out at the join point on the approach side: that is where an
            // approach starts, so hand back. Releasing merely OUTSIDE the FAP
            // would hand back mid-outbound-leg, pointing away from the field,
            // and leave the ordinary guidance to fly a 180 on final.
            // AT the join point, not merely abeam it: the leg homes on the
            // point, and a fast aircraft's turn-back leaves it miles beside
            // the centreline when the distance test first passes. The 737
            // was released 2.7 nm abeam at 6.7 nm, called "final" 1.9 nm off
            // (a 19 degree cut is interceptable on paper), and the pursuit's
            // 30 s time constant had 110 s of final to close 3.5 km: it
            // balked at 95 ft still 84 m off (LFLL 35L, 2026-09-17).
            Some(MissedStage::Reposition)
                if !self.hold
                    && along_nm > 0.0
                    && dist_nm >= join_nm - 0.2
                    && xtk_nm.abs() <= REPOSITION_RELEASE_XTK_NM =>
            {
                self.stage = None;
            }
            None if now == ApproachPhase::Intercept
                && matches!(prev, Some(ApproachPhase::Final | ApproachPhase::Flare))
                && radio_ft < TERRAIN_FLOOR_AGL_FT =>
            {
                self.stage = Some(MissedStage::Climb);
                self.attempts += 1;
            }
            _ => {}
        }
        self.stage
    }
}

/// Where a descent from `cur_alt_ft` meets the extended centreline: the point
/// to be established at before flying the path down, never closer than the FAP.
/// The 1.15 is margin — arriving exactly on a 3° path with no slack leaves
/// nothing for the capture.
pub fn join_point_nm(cur_alt_ft: f64, rwy_elev_ft: f64) -> f64 {
    ((cur_alt_ft - rwy_elev_ft) / gp_ft_per_nm() * 1.15).clamp(FAP_NM, 40.0)
}

/// The command a latched go-around flies.
///
/// [`MissedStage::Climb`] is runway heading (straight ahead, no turn near the
/// ground) and a bounded climb once the speed is there — level acceleration
/// first, because climbing slow bleeds energy to a stall.
///
/// [`MissedStage::Reposition`] steers OUTBOUND to the join point down the
/// extended centreline and holds platform height there. It does not chase the
/// glidepath: the path inside the join point is the one this leg is flying away
/// from, and descending toward it is what turned the Luton missed approach into
/// an orbit. Levelling at platform height means arriving at the FAP already on
/// the path, which is the whole reason the reposition altitude is defined as
/// the path height at the FAP.
///
/// Both stages ask the throttle governor for a margin over the approach speed
/// (the loop firewalls it anyway while latched; the target keeps the governor
/// sane on the release tick).
#[allow(clippy::too_many_arguments)]
pub fn missed_approach_guidance(
    rwy: &Runway,
    stage: MissedStage,
    cur_lat: f64,
    cur_lon: f64,
    cur_alt_ft: f64,
    radio_alt_ft: f64,
    dist_nm: f64,
    glidepath_alt_ft: f64,
    ias_kts: f64,
    approach_ias: f64,
    ias_trend_kts_s: f64,
    floor_kts: Option<f64>,
) -> Guidance {
    let slow = approach_ias - ias_kts > SPEED_FLOOR_MARGIN_KTS;
    let (target_heading_deg, target_vs_fpm) = match stage {
        MissedStage::Climb => (
            rwy.course_deg,
            go_around_vs_fpm(
                ias_kts,
                ias_trend_kts_s,
                approach_ias,
                floor_kts,
                radio_alt_ft,
                max_climb_fpm(),
            ),
        ),
        MissedStage::Reposition => {
            // Same join point the ordinary guidance uses, so the handback is
            // continuous.
            let join_nm = join_point_nm(cur_alt_ft, rwy.elev_ft);
            let recip = (rwy.course_deg + 180.0) % 360.0;
            let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
            let vs = if radio_alt_ft < REPOSITION_MIN_AGL_FT - 50.0 && !slow {
                ((REPOSITION_MIN_AGL_FT - radio_alt_ft) * 2.0).clamp(100.0, max_climb_fpm())
            } else {
                0.0
            };
            (bearing_deg(cur_lat, cur_lon, jlat, jlon), vs)
        }
    };
    Guidance {
        target_heading_deg,
        target_vs_fpm,
        target_ias_kts: approach_ias + CLIMB_ENERGY_BONUS_KTS,
        dist_nm,
        glidepath_alt_ft,
        phase: ApproachPhase::Intercept,
        balk: None,
        arresting: false,
    }
}

/// The flap a go-around cleans up to (the loop's go-around flap).
const GO_AROUND_FLAP: f64 = 0.5;
/// The missed approach holds go-around thrust until this high above the
/// ground ...
const GA_THRUST_HOLD_AGL_FT: f64 = 400.0;
/// ... and climbing at least this, fpm: then it is established.
const GA_CLIMB_ESTABLISHED_FPM: f64 = 500.0;
/// Established and this far over the go-around speed, the thrust comes back
/// at once to the climb floor, not a governor step a tick: at 0.04 a tick the
/// lever takes twenty seconds from 1.0, and the clean 737 gains 3 kt/s.
const GA_FAST_KTS: f64 = 10.0;
/// ...and this far under it, go-around thrust again.
const GA_SLOW_KTS: f64 = 10.0;

/// The speed a missed approach flies, kt: the give-up's pattern speed for the
/// flap out (`hand_back_speed_kts`, the clean approach speed with the flaps
/// up), at least the approach speed plus the climb margin and fast enough for
/// the go-around flap to come up (`go_around_floor_kts`, the floor of
/// [`GO_AROUND_FLAP`], plus 5), never within 10 kt of the placard of the flap
/// out, never over 250 below 10,000 ft (`alt_ft`), never under the floor of
/// the configuration flown (`config_floor_kts`, which wins). EGNX 09
/// (2026-09-30, #199): the clean 737 flew its missed approach at full power
/// from 250 to 355 kt below 1,700 ft; this is 210. Pure.
pub fn missed_approach_speed_kts(
    approach_ias: f64,
    flap_ratio: f64,
    flap_limits: &[(f64, Option<f64>)],
    go_around_floor_kts: Option<f64>,
    config_floor_kts: Option<f64>,
    alt_ft: f64,
) -> f64 {
    let placard = flap_placard_kts(flap_ratio, flap_limits).unwrap_or(f64::INFINITY);
    let mut t = hand_back_speed_kts(approach_ias, flap_ratio, flap_limits)
        .max(approach_ias + CLIMB_ENERGY_BONUS_KTS)
        .max(go_around_floor_kts.map_or(0.0, |f| f + 5.0))
        .min(placard - PLACARD_TARGET_MARGIN_KTS);
    if alt_ft < crate::tools::speedhold::SPEED_LIMIT_ALT_FT {
        t = t.min(crate::tools::speedhold::BELOW_10K_LIMIT_KTS);
    }
    crate::aircraft::floored_target(t, config_floor_kts)
}

/// The lever a missed approach's climb flies. Go-around thrust until the
/// climb is ESTABLISHED ([`GA_THRUST_HOLD_AGL_FT`] up and climbing at
/// [`GA_CLIMB_ESTABLISHED_FPM`]; an unread climb is not established), and
/// whenever [`GA_SLOW_KTS`] slow; then the speed: more than [`GA_FAST_KTS`]
/// over `target_kts`, the airframe's climb floor at once (`climb_floor`, its
/// approach thrust, else idle: the VS mode holds the climb and the excess
/// becomes height), else the speed governor's step, inside the floor and
/// full. A crew's go-around: TOGA, positive climb, gear and flap, then the
/// thrust set for the go-around speed. Pure.
pub fn missed_approach_throttle(
    throttle_now: f64,
    ias_kts: f64,
    trend_kts_s: f64,
    target_kts: f64,
    radio_ft: f64,
    vvi_fpm: Option<f64>,
    climb_floor: Option<f64>,
) -> f64 {
    let established = radio_ft >= GA_THRUST_HOLD_AGL_FT
        && vvi_fpm.is_some_and(|v| v >= GA_CLIMB_ESTABLISHED_FPM);
    if !established || ias_kts < target_kts - GA_SLOW_KTS {
        return 1.0;
    }
    let floor = climb_floor.unwrap_or(IDLE_THROTTLE).max(IDLE_THROTTLE);
    if ias_kts > target_kts + GA_FAST_KTS {
        return floor;
    }
    let trend = if trend_kts_s.is_finite() { trend_kts_s } else { 0.0 };
    (throttle_now + crate::tools::speedhold::throttle_adjust_live(target_kts - ias_kts, trend))
        .clamp(floor, 1.0)
}

/// A go-around judges the speed it is HEADING for: the airspeed this many
/// seconds ahead on its trend.
const GA_SPEED_LEAD_S: f64 = 2.0;
/// Below the floor, the sink asked per knot short of it (fpm per kt).
const GA_FPM_PER_KT: f64 = 100.0;
/// ...and never more sink than reaches the ground in this long: height is
/// traded for speed only where there is height to trade.
const GA_GROUND_S: f64 = 10.0;

/// The vertical speed a slow go-around flies: PITCH FOR SPEED FIRST (issue
/// #77). Not slow (the projected airspeed within [`SPEED_FLOOR_MARGIN_KTS`]
/// of the approach speed): `climb_fpm`. Slow but at or over the floor (1.2
/// Vs in the configuration flown, [`crate::aircraft::config_floor_kts`]):
/// level, the thrust builds the speed. Under the floor: a descent of
/// [`GA_FPM_PER_KT`] per knot short, never steeper than the ground ten
/// seconds away ([`GA_GROUND_S`]), so near the runway it is almost level and
/// on it level. The old law asked 0 fpm whatever the speed, and a VS mode
/// holds 0 fpm by pitching up until the wing stalls: EDLP 24 (2026-09-24)
/// the SF50 balked `balloon` at 52 ft climbing 860 fpm at 81 kt on idle, was
/// given +1,500 then 0, bled to 74 kt with the go-around's half flap, and
/// stalled onto the runway at -1,568 fpm. `floor_kts` unknown: the approach
/// speed's own 1.2/1.3. Pure.
pub fn go_around_vs_fpm(
    ias_kts: f64,
    trend_kts_s: f64,
    approach_ias: f64,
    floor_kts: Option<f64>,
    radio_ft: f64,
    climb_fpm: f64,
) -> f64 {
    let trend = if trend_kts_s.is_finite() {
        trend_kts_s.clamp(-10.0, 10.0)
    } else {
        0.0
    };
    let projected = ias_kts + trend * GA_SPEED_LEAD_S;
    if !(approach_ias - projected > SPEED_FLOOR_MARGIN_KTS) {
        return climb_fpm;
    }
    let floor = floor_kts
        .filter(|f| f.is_finite() && *f > 0.0)
        .unwrap_or(approach_ias * crate::aircraft::SPEED_FLOOR_RATIO / 1.3);
    let give = -(radio_ft.max(0.0) / GA_GROUND_S * 60.0);
    (GA_FPM_PER_KT * (projected - floor)).clamp(give.min(0.0), 0.0)
}

/// The go-around flap comes up only at the speed that flies it: its floor
/// (1.2 Vs in that configuration). EDLP 24 (2026-09-24): half flap at 81 kt
/// put the SF50's stall at 74, where the go-around met it. Pure.
pub fn go_around_flap_due(ias_kts: f64, floor_kts: Option<f64>) -> bool {
    match floor_kts {
        Some(f) if f.is_finite() => ias_kts >= f,
        _ => true,
    }
}

/// Autoland's intercepts, patterns and missed approach: a two-degree
/// correction at most every five seconds.
pub const AUTOLAND_INTERCEPT_DEADBAND: crate::heading_bug::BugDeadband =
    crate::heading_bug::BugDeadband { deg: 2.0, every_s: 5.0 };
/// The final outside [`SHORT_FINAL_NM`]: half a degree at most every two
/// seconds (a degree every three left the fitted 737 9.8 m off at the
/// roundout gate from a 30 m LEMG 13 handover, against 3.9 every tick).
pub const AUTOLAND_FINAL_DEADBAND: crate::heading_bug::BugDeadband =
    crate::heading_bug::BugDeadband { deg: 0.5, every_s: 2.0 };

/// How autoland's heading knob is set this tick (issue #93): the key and
/// deadband for [`crate::heading_bug::BugHold`], or `None` to steer it every
/// tick as before (the short final and the flare, where the last 0.3 nm
/// closes the centreline in seconds). A pattern's legs and each missed
/// approach stage are their own keys, so a new leg is set at once. Pure.
pub fn bug_knob(
    phase: ApproachPhase,
    short_final: bool,
    pattern_leg: Option<crate::reversal::Leg>,
    missed: Option<MissedStage>,
) -> Option<(u32, crate::heading_bug::BugDeadband)> {
    match (phase, short_final, pattern_leg, missed) {
        (ApproachPhase::Flare, ..) | (_, true, ..) => None,
        (_, _, Some(leg), _) => Some((10 + leg as u32, AUTOLAND_INTERCEPT_DEADBAND)),
        (_, _, None, Some(stage)) => Some((20 + stage as u32, AUTOLAND_INTERCEPT_DEADBAND)),
        (ApproachPhase::Final, ..) => Some((2, AUTOLAND_FINAL_DEADBAND)),
        (ApproachPhase::Intercept, ..) => Some((1, AUTOLAND_INTERCEPT_DEADBAND)),
    }
}

/// Where the autoland is in the approach.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApproachPhase {
    /// Flying to intercept the final approach course.
    Intercept,
    /// Established on final, tracking the centreline and glidepath.
    Final,
    /// Very short — over the threshold, flaring.
    Flare,
}

/// How long to sleep between ticks, milliseconds — by what the loop is
/// actually flying.
///
/// 400 ms everywhere made the tick 0.6 s end to end (measured on the EGPH
/// landing, 2026-09-20: `dt` median 0.59 s, so 1.7 Hz), and a flare is over
/// in three seconds. The 737's own `ArrestLeadProfile` declares 1.5 s of
/// dead time, so an arrest commanded six times a flare gets about TWO
/// commands the aircraft can still act on — it touched down at 977 fpm
/// against a 175 fpm target, and the yoke visibly stepped between updates.
/// No gain fixes that; there is not enough bandwidth to fly a flare.
///
/// The Web API is served from X-Plane's flight loop, so this is sim frame
/// time and it is spent where it buys something: fast where feet matter,
/// unchanged out on the intercept where nothing moves in half a second.
///
/// Since 2026-09-30 this is the tick's PERIOD, paced from its start
/// (`sim::state::Pacer`), not a sleep after its work: the old sleeps (100,
/// 200, 400) plus the requests' rounds made the MacBook's 0.20 s Flare and
/// 0.33 s Final (bench landings, `power` dt) and the iMac's 0.39 and 0.56.
/// These are the MacBook's, the ticks the laws were tuned on, now on any
/// machine; the Intercept's 540 is its old 400 sleep plus seven 20 ms rounds
/// (no MacBook record).
fn tick_ms(phase: Option<ApproachPhase>, arresting: bool) -> u64 {
    // The roundout is hand-flown on the channels too (it is where the
    // servos come off), and it needs the flare's rate, not Final's.
    if arresting {
        return 200;
    }
    match phase {
        // Over the threshold: the flare and the arrest.
        Some(ApproachPhase::Flare) => 200,
        // Established, configuring, closing the last miles.
        Some(ApproachPhase::Final) => 330,
        _ => 540,
    }
}

/// The rollout's tick period, ms: its old 300 ms sleep plus the MacBook's
/// rounds (bench rollouts before 2026-09-29, `dt` median 0.378 s).
const ROLLOUT_TICK_MS: u64 = 380;
/// The approach's cross-track rate filter, seconds: half the raw rate a
/// tick at the MacBook's 0.33 s Final tick is a time constant of
/// 0.33 / ln 2.
const XTK_RATE_TAU_S: f64 = 0.476;

/// The command the control loop should fly this tick.
#[derive(Debug, Clone, Copy)]
pub struct Guidance {
    pub target_heading_deg: f64,
    pub target_vs_fpm: f64,
    pub target_ias_kts: f64,
    pub dist_nm: f64,
    /// Where the aircraft *should* be for the glidepath, ft MSL.
    pub glidepath_alt_ft: f64,
    pub phase: ApproachPhase,
    /// Why this tick balked the landing, when it did: the gate that flipped
    /// the phase to `Intercept` low enough for the latch to fly it as a missed
    /// approach. `None` on every ordinary tick. Blackboxed with the
    /// `going_around` event, because "climbing straight ahead" with no reason
    /// attached cost a person an hour of reconstruction from the command
    /// column (LEMG 13, 2026-09-12).
    pub balk: Option<&'static str>,
    /// The vertical target is a landing ARREST (roundout or flare): the VS
    /// dial must lead it through the autopilot's lag ([`vs_dial_fpm`]).
    /// False everywhere else, where the dial is the target.
    pub arresting: bool,
}

/// Where the final's pure pursuit aims, in nm from the threshold measured back
/// up the approach. Negative means a point PAST the threshold, on the runway.
///
/// The lead is fixed — about thirty seconds of travel — because that is what
/// makes the convergence exponential in TIME rather than linear in distance:
/// hard correction early, established miles out, then a tracked wings-level
/// final, which is what "stabilised approach" means. An aim at half the
/// remaining distance decays the cut only linearly, and the aircraft rides the
/// whole final at an angle (EGSS 22, 2026-07-28: 28 degrees at 5.4 nm, still
/// 20 at 3, aligned only at 0.13 nm, i.e. over the numbers).
///
/// The trap this closes is that the fix for that was itself undone on short
/// final. Flooring the aim AT the threshold meant that inside one lead
/// distance the aim stopped advancing and the EFFECTIVE lead shrank — 0.875 nm
/// at 1.2 nm out, 0.55 at 0.7, 0.20 at 0.35 — which is the d/2 law again,
/// reintroduced exactly where being lined up matters most. At Southampton the
/// cut sat at 16 degrees from 0.87 nm all the way in to 0.32, then snapped to
/// the runway course over the numbers with the aircraft still 6 degrees off
/// and still turning (2026-08-01).
///
/// So the aim keeps its full lead all the way in and simply walks past the
/// threshold. That is not a trick: the extended centreline and the runway
/// centreline are one line. Bounded by the pavement so the aim is always a
/// point on the runway.
pub fn pursuit_aim_nm(along_nm: f64, gs_kts: f64, rwy_len_m: f64) -> f64 {
    let lead_nm = (gs_kts / 120.0).clamp(0.6, 1.5);
    let past_nm = (rwy_len_m / M_PER_NM * 0.8).clamp(0.2, 1.5);
    (along_nm - lead_nm).max(-past_nm)
}

/// ONCE FLARING, STAY FLARING. `guidance` decides the phase from this tick's
/// radio height alone, so a flare that ballooned back above its gate became
/// "final" again, and final's law put the nose down: forward elevator and a
/// -1,200 fpm dial from 58 ft, then the flare again at 24 ft with the column
/// already at its stop, and the wheels arrived at -1,031 fpm (the Citation,
/// EDDV 27L, 2026-09-22). A pilot holds a balloon and lets it settle, or
/// goes around. So: a tick that was flaring and now reads Final stays in the
/// flare up to the top of the roundout band; above the band the landing is
/// balked (`balloon`) and the go-around latch flies it. Anything else
/// guidance decided (a balk, an overflight) stands.
pub fn hold_flare(
    g: Guidance,
    was_flaring: bool,
    radio_ft: f64,
    gs_kts: f64,
    flare_radio_ft: f64,
) -> Guidance {
    if !was_flaring || g.phase != ApproachPhase::Final {
        return g;
    }
    let (_, band_ft) = arrest_gates_ft(flare_radio_ft, gs_kts);
    if radio_ft > band_ft {
        return Guidance {
            phase: ApproachPhase::Intercept,
            balk: Some("balloon"),
            arresting: false,
            ..g
        };
    }
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (_, flare_vs) = arrest_targets_fpm(nominal_vs);
    Guidance {
        phase: ApproachPhase::Flare,
        target_vs_fpm: flare_target_fpm(radio_ft, flare_vs, nominal_vs),
        arresting: true,
        ..g
    }
}

/// A balloon out of the flare is landed, not gone around from, below this
/// height (with room to stop). EDLP 24 (2026-09-24): the SF50's flare held
/// 0.3 of elevator until it was already climbing, ballooned to 52 ft at +850
/// fpm, balked `balloon` above the 50 ft band, and the go-around from idle at
/// 81 kt and slowing was given 0 fpm and stalled onto the runway at -1,568
/// fpm. A pilot holds a balloon, adds a little power and lets it settle.
pub const BALLOON_COMMIT_FT: f64 = 100.0;

/// The height a recited landing distance is measured from: the book figure
/// (POM, AFM) runs from 50 ft over the threshold, on a stabilised 3 degree
/// approach, to a stop.
const LANDING_SCREEN_FT: f64 = 50.0;
/// The part of that figure a 3 degree path covers from the screen height to
/// the ground: 50 ft x 0.3048 / tan 3 degrees, about 291 m. The book's
/// definition, so the 3 degree constant, not this approach's path.
const SCREEN_AIR_M: f64 = LANDING_SCREEN_FT * M_PER_FT / GLIDEPATH_TAN;

/// The pavement a flare at `radio_ft` still needs to land and stop, metres.
///
/// A recited landing distance starts at 50 ft over the threshold; an
/// aircraft flaring at 8 ft has already flown the part of it between the
/// two, and asking the pavement for the whole figure counts that part twice.
/// LFBD 11, 2026-09-27 (the stock 737, POM 5,360 ft = 1,634 m, 2,418 m of
/// runway): the flare floated to 7.9 ft radio at 145 kt over the ground,
/// 796 m past the threshold with 1,622 m ahead and sinking 184 fpm, and the
/// overflight balk stood because 1,622 < 1,634, by 12 m. "Going around",
/// full power for a second, the wheels touched 1.3 s later and it stopped
/// with about 700 m to spare. The air flown is taken on the book's 3 degree
/// path from 50 ft down to this height, in proportion (a flare at 7.9 ft is
/// credited 245 m of the 291, a balloon at or above 50 ft nothing): the
/// book's own air segment is longer than that geometry, since it includes
/// the flare's float, so what is left asked for is the ground roll plus a
/// float, never less.
///
/// Floored at half the declared figure: a light single's ground roll is a
/// small part of its over-50-ft figure (the 172's book: 1,335 ft total,
/// 575 ft of roll, 43%), and the geometric 291 m would leave it asking for
/// 116 m to stop in 175. Half covers it; a jet's need never reaches the
/// floor (the 737's 1,634 less 291 is 1,343, against 817).
///
/// With NO declared figure the need stays [`FLOAT_MIN_REMAINING_M`] whole:
/// that 900 m is the Baron's measured ~550 m ROLLOUT plus a margin, a
/// ground figure already, with no air segment in it to credit. Pure.
pub fn flare_pavement_need_m(radio_ft: f64, landing_m: Option<f64>) -> f64 {
    match landing_m.filter(|v| v.is_finite() && *v > 0.0) {
        Some(book_m) => {
            let flown_ft = (LANDING_SCREEN_FT - radio_ft.max(0.0)).clamp(0.0, LANDING_SCREEN_FT);
            let air_flown_m = SCREEN_AIR_M * flown_ft / LANDING_SCREEN_FT;
            (book_m - air_flown_m).max(0.5 * book_m)
        }
        None => FLOAT_MIN_REMAINING_M,
    }
}

/// COMMITTED, ONCE FLARING LOW. A flare below the roundout band is at idle,
/// slow and nose up, and the go-around the overflight balk hands it cannot
/// climb: the vertical law climbs only with speed in hand, a flare has none,
/// and a jet's engines are a few seconds from thrust. ETND 26 (2026-09-23):
/// the SF50 ballooned to 32 ft, the float allowance (1,244 m of runway less the
/// Baron's 900 m floor) ran out at 344 m, it went around with 900 m of runway
/// ahead, the law gave it 0 fpm at 80 kt, and it met the runway at about -740
/// fpm. So below the band, an `overflew_the_runway` verdict does not stand
/// while the pavement left still covers the airframe's OWN landing distance
/// (the 900 m floor is for the float's judgement above, not for a landing
/// already in its last seconds). A BALLOON is committed the same way up to
/// [`BALLOON_COMMIT_FT`] (EDLP 24, the crash). No room to stop is still a
/// go-around; every other balk stands. The room is
/// [`flare_pavement_need_m`]: the recited distance less the air segment
/// already flown (LFBD 11). Pure.
#[allow(clippy::too_many_arguments)]
#[cfg(test)]
pub fn commit_flare(
    g: Guidance,
    was_flaring: bool,
    radio_ft: f64,
    gs_kts: f64,
    flare_radio_ft: f64,
    remaining_m: f64,
    landing_m: Option<f64>,
    course_deg: f64,
) -> Guidance {
    commit_flare_latched(
        g,
        was_flaring,
        false,
        radio_ft,
        gs_kts,
        flare_radio_ft,
        remaining_m,
        landing_m,
        course_deg,
    )
}

/// ONCE COMMITTED, THE FLARE STAYS COMMITTED. [`commit_flare`] judged the
/// room afresh every tick against [`flare_pavement_need_m`] at THIS height,
/// which high in the band is the overflight allowance itself less the few
/// metres of air credited from 50 ft: the A330 at LEMG 30 (sortie
/// 1790634412-approach, 2026-09-28) was committed at 46.4 ft with 1,432 m
/// ahead against a need of 1,427, floating level, and 0.3 s and 23 m later
/// the same test failed, the arrest handed the servos back and it went
/// around `overflew_the_runway` with 1,409 m of runway left. A commitment
/// that lasts one tick is no commitment. So once committed (`was_committed`,
/// the loop's latch for this flare), the landing continues while the
/// pavement ahead still covers the book's GROUND roll (the need at the
/// wheels, the whole air segment credited); past that it is still a
/// go-around. The ceiling heights stand. Pure.
#[allow(clippy::too_many_arguments)]
pub fn commit_flare_latched(
    g: Guidance,
    was_flaring: bool,
    was_committed: bool,
    radio_ft: f64,
    gs_kts: f64,
    flare_radio_ft: f64,
    remaining_m: f64,
    landing_m: Option<f64>,
    course_deg: f64,
) -> Guidance {
    let (_, band_ft) = arrest_gates_ft(flare_radio_ft, gs_kts);
    let ceiling_ft = match g.balk {
        Some("overflew_the_runway") if was_flaring => band_ft,
        Some("balloon") if was_flaring => BALLOON_COMMIT_FT,
        _ => return g,
    };
    let need_m = flare_pavement_need_m(if was_committed { 0.0 } else { radio_ft }, landing_m);
    if radio_ft > ceiling_ft || remaining_m < need_m {
        return g;
    }
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (roundout_vs, flare_vs) = arrest_targets_fpm(nominal_vs);
    Guidance {
        // The overflight turned the heading back to the FAP; a landing
        // keeps the runway's.
        target_heading_deg: course_deg,
        phase: ApproachPhase::Flare,
        // Above the band (a balloon's top) come back down at the roundout's
        // sink: the flare's -100 from 60 ft is half a minute of runway.
        target_vs_fpm: if radio_ft > band_ft {
            roundout_vs
        } else {
            flare_target_fpm(radio_ft, flare_vs, nominal_vs)
        },
        balk: None,
        arresting: true,
        ..g
    }
}

/// ONCE THE ARREST HAS THE SERVOS, IT KEEPS THEM. `guidance` calls the
/// roundout from this tick's radio height alone (`radio_alt_ft <= band_ft`),
/// so a balloon in the roundout that rose back over the band top turned the
/// arrest off, and [`arrest_hands_step`] gave the servos back to an
/// autopilot whose VS dial was still the approach's descent: LEMG 12, the
/// SF50 (sortie 1790611629, 2026-09-28), taken at 51 ft, ballooned to +263
/// fpm, handed back at 50.5 ft against a 50 ft band; the autopilot flew its
/// -445 fpm dial and the governor, no longer held to "only back", walked the
/// lever 0.16 to 0.27 (on a tail-mounted engine, nose down) for 1.2 s; taken
/// again at 48.4 ft with the sink already rebuilding, 45 to 3 ft in 1.5 s,
/// -814 fpm at the wheels. So a tick called Final and not arresting, with no
/// balk, after the arrest has taken the servos is still the arrest: flown
/// down at the roundout's sink up to [`BALLOON_COMMIT_FT`] while the pavement
/// ahead still covers the landing ([`flare_pavement_need_m`], as
/// [`commit_flare`]); higher, or with no room left, it is a balked landing
/// (`balloon`) and the go-around latch flies it. A real balk (off the
/// centreline, overflown, ordered, a pattern) is never overruled: only a
/// go-around brings the autopilot back. The flare's own balloon is
/// [`hold_flare`]'s. Pure.
pub fn hold_roundout(
    g: Guidance,
    arrest_taken: bool,
    radio_ft: f64,
    gs_kts: f64,
    remaining_m: f64,
    landing_m: Option<f64>,
) -> Guidance {
    if !arrest_taken || g.arresting || g.phase != ApproachPhase::Final || g.balk.is_some() {
        return g;
    }
    if radio_ft > BALLOON_COMMIT_FT || remaining_m < flare_pavement_need_m(radio_ft, landing_m) {
        return Guidance {
            phase: ApproachPhase::Intercept,
            balk: Some("balloon"),
            arresting: false,
            ..g
        };
    }
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (roundout_vs, _) = arrest_targets_fpm(nominal_vs);
    Guidance {
        target_vs_fpm: roundout_vs,
        arresting: true,
        ..g
    }
}

/// Compute the approach guidance: steer to the final approach point then track the
/// centreline, and descend the 3° glidepath. Pure — the control loop applies it via
/// the autopilot. `flare_radio_ft` is the height above the runway to start the flare.
/// `xtk_rate_nm_min` is the cross-track closure rate (positive = drifting right),
/// 0.0 when unknown — the damping term on the final's centreline correction.
#[cfg(test)]
#[allow(clippy::too_many_arguments)]
pub fn guidance(
    cur_lat: f64,
    cur_lon: f64,
    cur_alt_ft: f64,
    radio_alt_ft: f64,
    gs_kts: f64,
    ias_kts: f64,
    rwy: &Runway,
    approach_ias: f64,
    flare_radio_ft: f64,
    xtk_rate_nm_min: f64,
) -> Guidance {
    guidance_on(
        cur_lat,
        cur_lon,
        cur_alt_ft,
        radio_alt_ft,
        gs_kts,
        ias_kts,
        rwy,
        approach_ias,
        flare_radio_ft,
        xtk_rate_nm_min,
        None,
    )
}

/// [`guidance`] on a runway of known pavement width (`width_m`, apt.dat's):
/// the lateral gates below the terrain floor judge the final against the
/// pavement ([`landing_xtk_limit_m`], [`flare_xtk_limit_m`]). `None` is the
/// old 37 m everywhere.
#[allow(clippy::too_many_arguments)]
pub fn guidance_on(
    cur_lat: f64,
    cur_lon: f64,
    cur_alt_ft: f64,
    radio_alt_ft: f64,
    gs_kts: f64,
    ias_kts: f64,
    rwy: &Runway,
    approach_ias: f64,
    flare_radio_ft: f64,
    xtk_rate_nm_min: f64,
    width_m: Option<f64>,
) -> Guidance {
    let landing_xtk_nm = landing_xtk_limit_m(width_m) / M_PER_NM;
    let flare_xtk_nm = flare_xtk_limit_m(width_m) / M_PER_NM;
    let dist_nm = haversine_nm(cur_lat, cur_lon, rwy.thr_lat, rwy.thr_lon);
    let recip = (rwy.course_deg + 180.0) % 360.0;

    // 3° glidepath altitude at this distance (+ = we're above it). Aimed at the
    // touchdown zone, so it crosses the threshold at ~50 ft, not zero.
    let glidepath_alt_ft = rwy.elev_ft + (dist_nm + TOUCHDOWN_AIM_NM) * gp_ft_per_nm();
    let alt_err = cur_alt_ft - glidepath_alt_ft;

    // Above what the way in can lose, we can't just dive onto the path: fly to
    // the JOIN point instead: where a 3° descent (with 15% margin) meets our
    // CURRENT altitude, out along the extended centreline. When we're inside it
    // (armed high overhead, the failure seen live: 7,000 ft at 6 nm, VS pegged,
    // never capturing) that means flying OUTBOUND first; the point walks back
    // toward the FAP as altitude burns off (a teardrop join). See
    // [`aims_at_join_point`] for the height a straight-in keeps.
    let join_nm = join_point_nm(cur_alt_ft, rwy.elev_ft);

    // Are we past the threshold, on the departure side, still airborne — by
    // more than the landing float is allowed? (The flare branch below, checked
    // first, owns a normal float/rollout across the numbers.)
    let along_nm = along_track_nm(cur_lat, cur_lon, rwy);
    // The arrest's gates and targets for THIS groundspeed (seconds of sink,
    // never bare feet: see `arrest_gates_ft`). `flare_radio_ft` is the floor.
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (flare_ft, band_ft) = arrest_gates_ft(flare_radio_ft, gs_kts);
    let (roundout_vs, flare_vs) = arrest_targets_fpm(nominal_vs);
    let float_nm = overflight_allowance_nm(
        rwy.length_m,
        radio_alt_ft,
        band_ft,
        crate::aircraft::landing_distance_m(),
    );
    let past_threshold = along_nm < -float_nm;
    // The flare/roundout environment: short final by plain distance, or
    // anywhere over the pavement the float allowance still covers — `dist_nm`
    // is unsigned, so past the threshold it alone would end the flare zone at
    // 0.35 nm while the float is still legitimately landing.
    let in_flare_zone = dist_nm <= FLARE_MAX_DIST_NM || (along_nm < 0.0 && !past_threshold);

    let xtk = cross_track_nm(cur_lat, cur_lon, rwy);
    let mut balk: Option<&'static str> = None;
    let (phase, target_heading) = if radio_alt_ft <= flare_ft && in_flare_zone {
        // Low radio altitude alone is NOT the flare cue — terrain rises to meet
        // the approach path miles from the field (seen live: "flared" at 7 ft AGL
        // 4.1 nm out at LEMI, power cut over open ground; and again into a dip
        // 0.4 nm short at LERL). Flare only over the runway environment.
        if xtk.abs() > flare_xtk_nm {
            // ...and only over the PAVEMENT. The final's P-only tracking left a
            // standing 0.10 nm cross-track under the LXGB crosswind and the
            // flare froze it: touchdown 185 m right of the centreline, in the
            // grass, wrecked on the rollout (2026-07-22). Off-centreline at
            // flare height is a balked landing — the phase flip drops the
            // GoAroundLatch, which climbs straight ahead.
            balk = Some("off_centreline_at_flare");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else {
            // Keep steering through the flare — locking the bare runway course
            // froze whatever offset the final still carried. Gentler clamp:
            // no big heading swings a few feet off the ground. Rate-damped
            // like the final, so a closing drift isn't chased into an S.
            let corr = ((xtk + 0.2 * xtk_rate_nm_min) * 80.0).clamp(-8.0, 8.0);
            (
                ApproachPhase::Flare,
                (rwy.course_deg - corr + 360.0) % 360.0,
            )
        }
    } else if past_threshold {
        // Overflew the runway without landing (too high/fast at the numbers). Go
        // around: steer BACK to the Final Approach Point, never track the runway
        // course outbound — that departs the field forever (seen live 2026-07-06:
        // over the numbers at 167 ft, then straight out to 5.4 nm at 60 kt, never
        // turning back). The vertical law climbs to a safe altitude on the way, if
        // energy allows. Takes priority over the join/FAP/final branches so a
        // go-around always turns toward the field regardless of altitude.
        let (fap_lat, fap_lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, FAP_NM);
        balk = Some("overflew_the_runway");
        (
            ApproachPhase::Intercept,
            bearing_deg(cur_lat, cur_lon, fap_lat, fap_lon),
        )
    } else if aims_at_join_point(alt_err, along_nm, xtk, gs_kts) {
        // Too high for this distance — descend toward the join point.
        let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
        (
            ApproachPhase::Intercept,
            bearing_deg(cur_lat, cur_lon, jlat, jlon),
        )
    } else if dist_nm > FAP_NM + 0.5 {
        // Outside the FAP: steer TO the Final Approach Point — a point FAP_NM out
        // on the approach side of the threshold along the extended centreline —
        // not the bare runway course. Tracking the raw course assumes we're
        // already on the approach side flying inbound; when we're off to the side
        // or PAST the threshold it flies us away from the field, the distance
        // growing every tick (seen live: XLE000N, 12.6 -> 13.8 nm on a fixed
        // heading, never intercepting). Aiming at the FAP always closes on the
        // runway from the correct side, then the final branch tracks the centreline.
        let (fap_lat, fap_lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, FAP_NM);
        (
            ApproachPhase::Intercept,
            bearing_deg(cur_lat, cur_lon, fap_lat, fap_lon),
        )
    } else {
        // On final: PURSUE a point ON the centreline halfway to the threshold
        // instead of P-correcting the offset. Offset-proportional gains kept
        // failing by geometry: 80°/nm commanded all of 7° of cut at 0.09 nm
        // off — mathematically unable to centre in the distance remaining,
        // and the aircraft crossed 150 ft still 145 m left (2026-07-24; same
        // family as the 120 m/2026-07-14 and 55 m/2026-07-22 misses at lower
        // gains). Aiming at the centreline point makes the cut DISTANCE-
        // aware: atan(offset / distance-to-aim) — gentle far out, up to a
        // 35° intercept when close and offset, rolling out on the line as
        // the pursuit point recedes.
        // Inside ~550 m the halfway aim point would fall BEHIND the aircraft
        // (the 0.25 nm floor exceeded the distance) and the bearing flipped
        // the cut ±35° at 100 ft over the numbers (2026-07-24). Short final
        // uses small course-referenced corrections only — offsets there are
        // either small or the balk gate's problem.
        // FIXED-lead pursuit, not half-the-remaining-distance. With the aim at
        // d/2 the cut angle decays only LINEARLY with distance (offset falls
        // as d², cut ~ offset/d ~ d), so the aircraft rode the WHOLE final at
        // an angle and only rolled out at the flare — measured at EGSS 22:
        // final began 5.4 nm out with a 28° cut, still 20° at 3 nm, 9° at
        // 1.9, 5° inside a mile, aligned at 0.13 nm, i.e. over the numbers
        // ("never in line with the runway, always at an angle", 2026-07-28).
        // A fixed lead makes the convergence exponential in TIME instead:
        // hard correction early, established miles out, then a tracked,
        // wings-level final — which is what "stabilised approach" means.
        // ~30 s of travel, floored so a slow single still aims somewhere and
        // capped so the far-out cut stays flyable.
        let cut = if dist_nm > 0.30 {
            let along = along_track_nm(cur_lat, cur_lon, rwy);
            let aim_nm = pursuit_aim_nm(along, gs_kts, rwy.length_m);
            let (alat, alon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, aim_nm);
            let brg = bearing_deg(cur_lat, cur_lon, alat, alon);
            crate::angles::signed_diff_deg(brg, rwy.course_deg).clamp(-35.0, 35.0)
        } else {
            (-xtk * 80.0).clamp(-8.0, 8.0)
        };
        if radio_alt_ft < 100.0 && dist_nm < 0.6 && xtk.abs() > landing_xtk_nm {
            // Not going to make the pavement: balk NOW, with height to work
            // with. The 22 ft flare-gate balk arrested nothing — the aircraft
            // still skimmed the ground at 91 kt before climbing (2026-07-22).
            balk = Some("off_centreline_low");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else if along_nm > STABILISED_MIN_ALONG_NM
            && radio_alt_ft >= TERRAIN_FLOOR_AGL_FT
            && !stabilised_xtk(xtk, along_nm)
        {
            // Inside the FAP but not somewhere a final can be flown from:
            // reposition to the join point out along the centreline, the
            // way the alt_err branch above does. "Land the aircraft" from
            // 1.5 nm abeam Gibraltar's 09 used to be called FINAL here — a
            // saturated 35 degree cut, the speed schedule pulling the RV-10
            // to its 65 kt "approach speed", and a stall (2026-09-11).
            let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
            (
                ApproachPhase::Intercept,
                bearing_deg(cur_lat, cur_lon, jlat, jlon),
            )
        } else if (TERRAIN_FLOOR_AGL_FT..STABILISED_GATE_AGL_FT).contains(&radio_alt_ft)
            && (STABILISED_MIN_ALONG_NM..FAP_NM).contains(&along_nm)
            && !crate::reversal::stabilisable(xtk, along_nm, gs_kts)
        {
            // THE STABILISED GATE, where a crew has one: a thousand feet.
            // Position alone called 1,880 m off at 3.3 nm "interceptable"
            // (a 35 degree cut does reach it), the 737 rode the glidepath
            // down beside the runway and the first lateral balk it met was
            // the flare gate's, at 94 ft and 207 m off; the second approach
            // got to 143 ft (EGTE, 2026-09-21). The pursuit converges in
            // TIME, so the question is the seconds of final left
            // (`reversal::predicted_residual_m`). The loop flies this one as
            // a go-around whatever the height.
            balk = Some("unstabilised_at_gate");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else if radio_alt_ft < TERRAIN_FLOOR_AGL_FT
            && (STABILISED_MIN_ALONG_NM..FAP_NM).contains(&along_nm)
            && !(stabilised_xtk(xtk, along_nm)
                && (width_m.is_none()
                    || lands_on_the_pavement(xtk, along_nm, gs_kts, landing_xtk_nm * M_PER_NM)))
        {
            // ...and "interceptable" is not stabilised either: a 35 degree
            // cut reaches 50 m from anywhere, and the 737 rode 30 to 50 m
            // beside a 45 m strip from 500 ft to the wheels (LEMG 13,
            // 2026-09-28). Below the floor the pursuit's own TIME must bring
            // the offset onto the pavement by the threshold.
            // Below the terrain floor and not laterally stabilised: a balk
            // with height in hand. That flare gate above was the ONLY lateral
            // balk, so a final that never converged (handed off 50 degrees
            // off at Gibraltar) flew to 95 ft still 306 m off the centreline,
            // went around with a 470 fpm sink and full flap, and hit the
            // North Front four seconds later (2026-09-11). Inside the FAP
            // only: further out the intercept is still legitimately wide,
            // and radio height alone means nothing over rising ground.
            balk = Some("unstabilised_below_floor");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else {
            (
                ApproachPhase::Final,
                (rwy.course_deg + cut).rem_euclid(360.0),
            )
        }
    };

    // Track the path with the nominal descent for the current groundspeed
    // (`nominal_vs`, above) plus an error correction; bounded so it never dives.
    // SPEED PROTECTION overrides everything except the flare. Holding level below
    // the glidepath to "wait for the path" bleeds airspeed just as surely as
    // climbing does when the energy is marginal — that's the failure seen live:
    // stuck below the path 13 nm out, IAS oscillating down to 53 kt in a level
    // hold until the AP stalled off. A stall is fatal; being low is not. So when
    // airspeed decays below the approach speed, lower the nose and trade a little
    // altitude back for speed, proportional to how slow it is.
    // SAFE-ALTITUDE CLIMB: below the terrain floor and outside the flare zone, the
    // low radio altitude is rising terrain under the flight path — climb away from
    // it. But ONLY with surplus airspeed: climbing slow bleeds energy to a stall
    // (why this loop was descent-only). Low + slow falls through to speed
    // protection / level hold, and the raised throttle target below rebuilds the
    // surplus so the climb can engage on a later tick.
    // ESTABLISHED on the extended centreline beyond the FAP, inbound and on
    // or above the path: the descent is the final's, under the final's
    // floors, not the reposition's soft floor at circuit height. LEMG 13
    // (2026-10-05, 13:14:20 to 13:15:21): the pattern joined the path at
    // 10.7 nm, phase still Intercept (steering for the FAP), and the fade
    // keyed on radio height over the hills under the final cut the path's
    // -1,100 fpm to a dial jumping -210 to -1,400 that no VS mode follows:
    // on the path to 511 ft high in 60 s. The path itself was the pattern's
    // to clear (`reversal::plan`, 500 ft over its corridor).
    let established_path = established_on_final_path(phase, balk, along_nm, xtk, xtk_rate_nm_min, alt_err);
    let below_terrain_floor = match phase {
        ApproachPhase::Flare => false,
        _ if established_path => {
            radio_alt_ft < TERRAIN_FLOOR_AGL_FT && alt_err < FLOOR_BELOW_PATH_FT
        }
        // Final owns the descent to the runway — the floor stands down inside
        // short-final range where low radio altitude IS the approach, and it
        // never captures an aircraft ON the path (FLOOR_BELOW_PATH_FT: the
        // LXGB 09 overflight, 2026-07-22).
        ApproachPhase::Final => {
            radio_alt_ft < TERRAIN_FLOOR_AGL_FT
                && dist_nm > FLOOR_ENFORCE_MIN_DIST_NM
                && alt_err < FLOOR_BELOW_PATH_FT
        }
        // Intercept is repositioning, not landing: the floor holds at ANY
        // distance (see FLOOR_ENFORCE_MIN_DIST_NM for the KPDK 16 accident),
        // and at CIRCUIT height, not the bare terrain floor — a go-around
        // was vectoring back at 500-600 ft AGL ("is that a bit too low?",
        // 2026-07-22; it is).
        ApproachPhase::Intercept => radio_alt_ft < REPOSITION_MIN_AGL_FT,
    };
    let has_climb_energy = ias_kts > approach_ias + CLIMB_SPEED_MARGIN_KTS;

    // A SOFT floor under the descent, not the hard edge the climb gate is.
    // Repositioning wants REPOSITION_MIN_AGL_FT (~1,590 ft AGL), the 3° path
    // inside 5 nm sits below it, and the two laws met at a step: dive at 1,200
    // toward the path, cross the floor, climb at 800 back through it, repeat.
    // The Luton go-around (2026-07-28) rode that limit cycle for three and a
    // half minutes, porpoising 1,859 to 2,050 ft while wandering between 1.1
    // and 4.1 nm from the field. Fading the permitted descent to zero AT the
    // floor lands the aircraft on it instead of through it.
    let descent_cap_fpm = path_descent_cap_fpm(nominal_vs);
    let descent_limit_fpm = if phase == ApproachPhase::Intercept && !established_path {
        -((radio_alt_ft - REPOSITION_MIN_AGL_FT) * 2.0).clamp(0.0, -descent_cap_fpm)
    } else {
        descent_cap_fpm
    };

    // The roundout band's own gates, named once: the vertical law below owns
    // the descent from here, and the dial leads it (`arresting`).
    let roundout = phase == ApproachPhase::Final
        && !below_terrain_floor
        && in_flare_zone
        && xtk.abs() <= FLARE_MAX_XTK_NM
        && radio_alt_ft <= band_ft
        && roundout_reaches_pavement(along_nm, radio_alt_ft, gs_kts);

    // What the vertical path asks for, computed first so the speed-protection
    // branch below can be measured against it.
    let path_vs = match phase {
        // Arrest the descent near the ground: exponentially (`flare_target_fpm`).
        ApproachPhase::Flare => flare_target_fpm(radio_alt_ft, flare_vs, nominal_vs),
        // Below the floor without the surplus to climb: hold level while the
        // throttle bonus rebuilds it — NEVER track the glidepath down. The
        // glidepath altitude is distance-symmetric, so a rejoin on the wrong
        // side of the threshold reads "on path" and descends (KPDK 16).
        _ if below_terrain_floor => 0.0,
        // ROUNDOUT: on final over the pavement, just above the flare gate,
        // keep coming down gently — never chase the path and never hold
        // level. Both wrong answers happened from the same arithmetic: past
        // the threshold the (unsigned) glidepath distance grows again, so
        // alt_err goes negative and the level-hold branch below fired at
        // 27 ft over the runway. The aircraft floated level, bled the very
        // airspeed the flare needed, and dropped on from the mush at
        // -380 fpm one knot above the clean stall (EGHI, 2026-08-02). A
        // staged descent — this, then the flare's -100 — arrives at the
        // flare with flying speed, which is what a soft touchdown is made
        // of.
        // ...but only where its own descent still REACHES the pavement.
        // `in_flare_zone`'s first term is an unsigned distance, so this
        // branch also fires SHORT of the threshold — right for a normal
        // landing (40 ft crosses the numbers from ~0.1 nm out), and a
        // commanded descent into the approach lights for an aircraft dragged
        // low: at 30 ft radio 0.3 nm short, -250 fpm touches down ~190 m
        // BEFORE the runway, where the old level-hold below let the path
        // come down to it (audit, 2026-08-02). The test is the physical one
        // — see `roundout_reaches_pavement`.
        ApproachPhase::Final if roundout => roundout_vs,
        // OVER THE PAVEMENT above the band: keep the path's own sink. The
        // glidepath altitude is distance-symmetric, so past the threshold
        // alt_err turns negative and the level-hold below fired at 60 ft
        // over EGCC 23L with a 737 sinking 730 fpm; the dial read zero, the
        // aircraft levelled at 30 ft and floated 500 m into the balloon
        // (2026-09-19). A jet that crosses the numbers high has runway to
        // spend and no height to hold; the float rule above still bounds
        // how far. Never steeper either: the aim point 300 m in is not
        // worth a dive from 90 ft.
        ApproachPhase::Final if along_nm < 0.0 && xtk.abs() <= FLARE_MAX_XTK_NM => nominal_vs,
        // On/above speed but below the glidepath: hold level and let the path
        // descend to us — don't climb to intercept. Speed protection above has
        // already handled the case where holding level would stall us.
        _ if alt_err < -50.0 => 0.0,
        // 1.2 fpm per ft of path error: the old 0.4 recovered 77 fpm
        // against the 190 ft the LXGB 09 final carried — it arrived at
        // the numbers still 190 ft high (2026-07-22). 1.2 converges a
        // 200 ft error inside the last mile; the clamp still forbids a
        // dive, and it is never shallower than the path itself plus a
        // catch-up (`path_descent_cap_fpm`).
        _ => (nominal_vs - alt_err * glidepath_gain_fpm_per_ft(gs_kts))
            .clamp(descent_cap_fpm, 0.0)
            .max(descent_limit_fpm),
    };

    let speed_deficit = approach_ias - ias_kts;
    let target_vs = if phase != ApproachPhase::Flare && speed_deficit > SPEED_FLOOR_MARGIN_KTS {
        // Nose down to regain speed — but NEVER shallower than the path was
        // already asking for. The protection rate is a fixed 40 fpm per knot
        // capped at 700, which on a slow airframe is steeper than the path and
        // on a fast one is not: a jet on a 3° path at 140 kt needs ~740 fpm, so
        // 9 knots slow produced a 360 fpm "protection" that HALVED the descent
        // and made the aircraft slower still, the exact opposite of the intent.
        // The MD-82 drifted above the path through the last mile that way and
        // crossed the threshold 51 ft high at 152 kt, which is the overflight
        // that forced the Luton go-around (2026-07-28).
        path_vs.min(-(speed_deficit * 40.0).clamp(200.0, 700.0))
    } else if below_terrain_floor && has_climb_energy {
        // Bounded climb toward THIS PHASE'S floor, steeper the deeper below
        // it. The magnitude used to reference the 500 ft terrain floor even
        // when the repositioning gate had armed the climb — above 500 AGL it
        // clamped to a 100 fpm dribble, and the LXGB go-around crept along
        // at ~650 ft into the Rock (2026-07-22).
        let floor_ft = match phase {
            ApproachPhase::Intercept => REPOSITION_MIN_AGL_FT,
            _ => TERRAIN_FLOOR_AGL_FT,
        };
        ((floor_ft - radio_alt_ft) * 2.0).clamp(100.0, max_climb_fpm())
    } else {
        path_vs
    };

    // Below the terrain floor, ask the throttle governor for extra speed so it
    // builds the surplus the climb gate needs. Otherwise fly the DECELERATION
    // SCHEDULE: clean out at range, approach speed by the FAP. The floor case
    // still wins, and the schedule can only ever ask for MORE speed than the
    // bare approach figure, so nothing downstream sees a slower target than it
    // used to.
    let target_ias = if below_terrain_floor {
        approach_ias + CLIMB_ENERGY_BONUS_KTS
    } else {
        scheduled_ias(dist_nm, approach_ias)
    };

    Guidance {
        target_heading_deg: target_heading,
        target_vs_fpm: target_vs,
        target_ias_kts: target_ias,
        dist_nm,
        glidepath_alt_ft,
        phase,
        balk,
        arresting: phase == ApproachPhase::Flare || roundout,
    }
}

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
