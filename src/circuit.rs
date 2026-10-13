//! Auto circuit — the copilot flies a full traffic pattern: takeoff, the four
//! pattern legs, and a landing back at the departure runway.
//!
//! This is an ORCHESTRATOR, not a new flight law. It stitches together the
//! three proven pieces so "do a circuit" is one command:
//!
//! 1. [`crate::autotakeoff`] flies the takeoff and climb-out, handing off with
//!    the autopilot engaged (servos + HDG + VS) at a thousand feet.
//! 2. This loop then flies the pattern on the autopilot's heading bug —
//!    crosswind, downwind, base — levelling at pattern altitude and starting
//!    the descent on base.
//! 3. [`crate::autoland::run_autoland`] takes the turn to final and lands.
//!
//! Trust model matches its parts: every actuation goes through the same AP the
//! copilot tools drive, and "my controls" / "cancel circuit" hands back at any
//! point. EXPERIMENTAL and untuned — the leg timings and altitudes are coarse
//! constants meant to be tuned from `logs/blackbox.jsonl` (`"kind":"circuit"`)
//! after in-sim flights, exactly like autoland. Fly it VMC, over the field,
//! with a hand ready.
//!
//! What it deliberately does NOT do: hold precise pattern spacing (the legs are
//! time-based, not abeam-the-numbers geometry), fly a right-hand pattern with
//! terrain awareness, or guarantee autoland re-selects the departure runway
//! (it scores by position and heading; turning final onto the departure course
//! from close in is what biases it back — see the handoff below).

// The tests reach these through `use super::*` (the queue that used them
// moved to `crew_radio.rs`).
use crate::aircraft::throttle_dataref;
#[cfg(test)]
use crate::aircraft::climb_floor_kts;
use crate::aircraft::{climb_full_rate_kts, climb_limit_fpm};
use crate::autoflight::{self, Button, Dial};
#[cfg_attr(not(test), allow(unused_imports))]
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
#[cfg_attr(not(test), allow(unused_imports))]
use std::sync::Mutex;

use crate::autotakeoff;
use crate::heading_bug::{bug_toward, slew_bug, BugDeadband, BugHold};
use crate::runway::{along_track_nm, cross_track_nm, nearby_runways, true_to_mag};
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const HPATH: &str = "sim/flightmodel/position/hpath";
/// Bank angle, degrees, right wing down positive — the authored follower
/// reads it to know which way the AP is turning (see `bug_toward`).
const PHI: &str = "sim/flightmodel/position/phi";
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
// AP surface (mirror of autotakeoff.rs — the recipe PROVEN to climb and
// steer; `sim/autopilot/altitude_hold` is deliberately absent: it holds the
// CURRENT altitude, not the dial, and locked every pattern at handoff height
// (2026-07-24)).
const HDG_STATUS: &str = autoflight::names::HDG_STATUS;
const SERVOS_ON: &str = sim::datarefs::SERVOS_ON;
const VS_STATUS: &str = autoflight::names::VS_STATUS;
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
/// Gear handle — the circuit lowers it turning base (autoland's own
/// extension only fires inside 6 nm, which this hands over well inside of).
/// Flap handle: base and final extend the landing flaps by the placard
/// gates as the speed allows, the same law autoland runs inside.
/// The flap the base leg configures toward (the handle ratio): about half
/// travel, the approach setting on every airframe here; final goes to full.
const BASE_FLAP_RATIO: f64 = 0.5;
const M_TO_FT: f64 = 3.280_84;
const M_PER_NM: f64 = 1852.0;
// The pattern speed moved to `aircraft.rs` (pillars audit 2026-10-07, stage
// A): the pattern numbers are the airframe's, read by the circuit and the
// instructor's brief alike.
use crate::aircraft::pattern_speed_kts;

/// An airborne AGL below this after the takeoff loop exits means it REJECTED
/// (never flew) — the circuit is off, not into the pattern.
const AIRBORNE_MIN_AGL_FT: f64 = 300.0;
/// Terrain clearance the GENERIC rectangle must have at pattern altitude over
/// every leg it flies level (the crosswind, and the downwind until abeam the
/// threshold), over the whole corridor the autopilot really flies
/// ([`corridor_frame`]); below it the circuit switches sides, and with BOTH
/// sides fouled it raises the pattern altitude. 1,000 ft: SERA.5005(f)(1)'s
/// figure over congested areas, and the standard for an automation flying a
/// leg nobody is looking out of (#182). It was 600, checked on the drawn line
/// alone: the 737's LEMG 13 left side read 729 ft and was flown at 1,548
/// (sortie 1790599750), the downwind 1.3 km outside that line.
pub const PATTERN_TERRAIN_CLEAR_FT: f64 = 1000.0;
/// The authored LXGB route's own figure ([`raise_for_terrain`]), kept at the
/// 600 ft it was flown and checked on (2026-07-26, 2026-09-15): its legs are
/// the drawn lines themselves, joined and captured within
/// [`AUTHORED_CAPTURE_M`] by a waypoint follower rather than a rectangle whose
/// autopilot turns overshoot by a kilometre, and its climb model already
/// refuses a leg entered low. The rectangle's 1,000 would lift or refuse the
/// Rock circuit on a figure no sortie has tested (#182).
const AUTHORED_TERRAIN_CLEAR_FT: f64 = 600.0;

/// True while the circuit orchestrator runs. Written only by the run
/// ([`RUN`]).
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// The circuit's run: claimed by [`start`], cancelled by [`abort`] ("cancel
/// circuit" / "my controls", a stand-down). The circuit's client carries the
/// run's token, so a cancel is seen by every loop through
/// `xp.cancelled()` (the old shared ABORT latch was cleared by the next
/// circuit's start, under a circuit still winding down) and refuses its
/// writes; the sub-loops it launches take the token as their parent.
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::flagged(&ACTIVE);

/// The circuit run that owns `xp` is over: its flag drops now (the rest of
/// the run, a give-up's hand-back, still to come). Never a newer run's.
fn lower_flag(xp: &XPlaneClient) {
    for t in xp.cancellation() {
        RUN.lower_flag_for(&t);
    }
}

/// The circuit run that owns `xp` handed the aircraft to a loop it launched
/// with its token as the parent (autoland, the published arrival): its exit
/// must not cancel that loop.
fn handed_off(xp: &XPlaneClient) {
    for t in xp.cancellation() {
        RUN.record_for(&t, sim::run::Outcome::HandedOff);
    }
}

/// One-line status for the dashboard's "managing" panel.
/// The circuit's live line for the panel: the leg, the runway, the altitude
/// against the pattern target and the speed. Set every tick, cleared with
/// the loop.
pub static DETAIL: crate::auto_status::Detail = crate::auto_status::Detail::new();

pub fn status() -> Option<String> {
    ACTIVE
        .load(Ordering::Relaxed)
        .then(|| DETAIL.line("auto circuit"))
}

/// "downwind runway 27, 1,200 ft to 1,500, 75 kt"; the speed is left out
/// while the read has not come in.
pub fn circuit_detail(leg: Leg, rwy_id: &str, alt_ft: f64, target_alt_ft: f64, ias: f64) -> String {
    let leg = format!("{leg:?}").to_lowercase();
    let alt = crate::auto_status::commas((alt_ft / 10.0).round() as i64 * 10);
    let tgt = crate::auto_status::commas((target_alt_ft / 10.0).round() as i64 * 10);
    let mut s = format!("{leg} runway {rwy_id}, {alt} ft to {tgt}");
    if ias.is_finite() {
        s.push_str(&format!(", {} kt", (ias / 5.0).round() * 5.0));
    }
    s
}

// The crew radio queue moved to `crew_radio.rs` (phase 2): the circuit's
// own calls name it unqualified, nobody else reaches it through here.
use crate::crew_radio::*;

/// Cancel an active circuit and stand down whichever sub-loop is flying;
/// returns whether one was running.
pub fn abort() -> bool {
    let was = RUN.cancel();
    // Stand down whatever phase is live. The takeoff hands off silently (power
    // stays set); the procedure/autoland arrival aborts to the pilot.
    autotakeoff::cancel();
    crate::procedure::abort();
    crate::autoland::abort();
    was
}

/// Everything the orchestrator needs, resolved by the arming tool.
pub struct Params {
    pub apt_dat: String,
    /// Nav-data paths for the published-approach arrival (the terrain-safe
    /// alternative to a naive base-to-final): CIFP procedure dir, `earth_fix.dat`,
    /// `earth_nav.dat`. Empty-string-safe — a missing file just means the
    /// approach isn't found and the arrival falls back to autoland.
    pub cifp_dir: String,
    pub fix_data_path: String,
    pub nav_data_path: String,
    /// OpenAir airspace file — the pattern's restricted/prohibited/danger
    /// avoidance. Empty/missing file = no airspace data, no avoidance.
    pub airspace_path: String,
    pub cfg: ApConfig,
    /// A fully-built takeoff config — the circuit arms the same takeoff loop.
    pub takeoff: autotakeoff::Params,
    /// Approach speed handed to autoland (KIAS).
    pub approach_ias: f64,
    /// Pattern altitude above the field, feet.
    pub tpa_agl_ft: f64,
    /// Standard left-hand pattern unless the pilot asked for right.
    pub left_pattern: bool,
    /// Clean stall speed, KIAS — the floor under the pattern speed, since the
    /// circuit is flown clean. `None` leaves the speed as Vy alone.
    pub vs_clean_kts: Option<f64>,
    /// The slowest the pattern may climb toward, KIAS. See
    /// [`climb_limit_fpm`]; 0 disables the protection.
    pub climb_floor_kts: f64,
}

/// The profile's `ap_altitude_capture`: the pattern altitude goes on the
/// altitude dial and the autopilot's own capture holds the level legs (see
/// [`ap_holds_level`]). Off, the VS law flies altitude as it always has and
/// the dial is only lifted out of the way. Republished on every swap
/// ([`crate::flight_state::publish_airframe`]), the snapshot's
/// `ap_altitude_capture`.
fn ap_altitude_capture() -> bool {
    crate::aircraft::airframe_facts().ap_altitude_capture
}

const ALT_HOLD_STATUS: &str = autoflight::names::ALT_HOLD_STATUS;

/// The altitude the dial should read for this tick's vertical target. A LEVEL
/// target (pattern altitude, the base leg's height) is selected to the hundred
/// so the autopilot captures it; on the glidepath the dial is parked above,
/// autoland's own rule, because a capture under a descent eats the descent
/// (LEAX, 2026-09-15). `None` = leave it.
pub fn circuit_dial_ft(
    target_rate_fpm: f64,
    target_alt_ft: f64,
    dial_ft: f64,
    alt_ft: f64,
) -> Option<f64> {
    if target_rate_fpm != 0.0 {
        return crate::autoland::parked_altitude_dial_ft(dial_ft, alt_ft);
    }
    let want = (target_alt_ft / 100.0).round() * 100.0;
    ((dial_ft - want).abs() > 50.0).then_some(want)
}

/// How long VS mode may stay off under a command for it before it is said.
pub const VS_REFUSED_SECS: f64 = 8.0;
pub fn vs_stuck(off_for_s: f64) -> bool {
    off_for_s >= VS_REFUSED_SECS
}

/// Whether the autopilot's altitude hold has the level leg, so the VS law
/// stands off. CAPTURED (status 2) on a dial that IS this tick's level target:
/// re-pressing VS mode then would drop the hold it just made, every tick. The
/// A330 wandered 1,796 to 1,840 ft down the EGPF downwind on the VS law
/// (2026-09-21) with an altitude hold sitting unused behind a dial at 100 ft.
pub fn ap_holds_level(
    target_rate_fpm: f64,
    target_alt_ft: f64,
    dial_ft: f64,
    alt_hold_status: f64,
) -> bool {
    target_rate_fpm == 0.0
        && alt_hold_status.round() as i64 == 2
        && (dial_ft - (target_alt_ft / 100.0).round() * 100.0).abs() <= 50.0
}

/// The four legs after the departure runway; `Final` is the handoff to autoland.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Leg {
    Upwind,
    Crosswind,
    Downwind,
    Base,
    Final,
}

/// The heading to fly on a given leg: the runway course plus 90 degrees per leg,
/// left (subtract) for a standard pattern, right (add) otherwise. Upwind is the
/// runway course; Final comes back to it (four 90-degree turns = 360).
pub fn leg_heading(rwy_course_deg: f64, leg: Leg, left: bool) -> f64 {
    let n = match leg {
        Leg::Upwind => 0.0,
        Leg::Crosswind => 1.0,
        Leg::Downwind => 2.0,
        Leg::Base => 3.0,
        Leg::Final => 4.0,
    };
    let turn = if left { -90.0 } else { 90.0 };
    (rwy_course_deg + turn * n).rem_euclid(360.0)
}

/// Put the bug on a WHOLE degree, and only when that degree changes. The
/// laws above keep their fine value for their own arithmetic; the knob a
/// pilot sees moves in the steps a pilot's hand makes, not a tenth of a
/// degree every tick (the pilot, 2026-09-26: "a human pilot would never use
/// the heading bug like that"). Half a degree of rounding is inside what the
/// autopilot's own heading hold settles to.
async fn write_bug(xp: &dyn crate::xplane::XPlane, bug_mag: f64) {
    let whole = bug_mag.round().rem_euclid(360.0);
    let bits = whole.to_bits();
    if LAST_BUG.swap(bits, std::sync::atomic::Ordering::Relaxed) == bits {
        return;
    }
    if autoflight::set_dial(xp, Dial::HeadingBug, whole).await.is_err() {
        LAST_BUG.store(u64::MAX, std::sync::atomic::Ordering::Relaxed);
    }
}

/// The circuit's legs: a two-degree correction at most every ten seconds.
pub const CIRCUIT_LEG_DEADBAND: BugDeadband = BugDeadband { deg: 2.0, every_s: 10.0 };
/// The circuit's final, closing the centreline for the handover: a degree
/// at most every three seconds.
pub const CIRCUIT_FINAL_DEADBAND: BugDeadband = BugDeadband { deg: 1.0, every_s: 3.0 };

/// The laws' bug through the [`BugHold`]: the knob is set only when the hold
/// says so, and the laws carry on from what the knob holds.
async fn hold_bug(
    xp: &dyn crate::xplane::XPlane,
    hold: &mut BugHold,
    key: u32,
    bug_mag: &mut f64,
    now_s: f64,
    db: BugDeadband,
) {
    if let Some(b) = hold.decide(key, *bug_mag, now_s, db) {
        write_bug(xp, b).await;
    }
    if let Some(s) = hold.set() {
        *bug_mag = s;
    }
}

/// The last whole degree written, as f64 bits (`u64::MAX`: none).
static LAST_BUG: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(u64::MAX);

/// The authored follower's progress watchdog fires when the distance to the
/// current waypoint has set no new low for this long: the loop is orbiting or
/// being blown away, not flying the leg.
const STALL_S: f64 = 45.0;
/// A new low has to beat the old one by this much to count as progress
/// (an orbit's near side re-sets the same distance every lap).
const STALL_PROGRESS_M: f64 = 25.0;
/// Stalls the authored follower tolerates before handing the aircraft back.
const MAX_STALLS: u32 = 3;

/// Spawn the circuit orchestrator on its own thread + runtime + client, the
/// same shape as the takeoff and autoland loops.
pub fn start(base_url: String, voice: Option<Voice>, p: Params) {
    let Some(run) = RUN.claim() else {
        return;
    };
    *STOOD_DOWN.lock().unwrap_or_else(|e| e.into_inner()) = None;
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        // The run's token scopes the circuit's client: its cancel test, its
        // writes, and the parent of every loop it launches.
        let xp = XPlaneClient::new(base_url.clone()).with_cancel(run.cancel.clone());
        rt.block_on(fly(xp, base_url, voice, p));
        // (base_url is cloned into the client and passed on for the sub-loops.)
        drop(run);
    });
}

fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    // Log every spoken line so the auto-flight narration shows up in the app
    // log, not just through the speakers (background loops have no reporter).
    tracing::info!(target: "xflightcrew::circuit", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

/// Why the running (or last) circuit stood down, `None` while it has not:
/// full auto's circuit mission ends as the takeoff's give-up when the
/// takeoff never completed, instead of a silent "ended" (LEMG 13,
/// 2026-09-28, sortie 1790627884-full_auto: a rejected takeoff ended the
/// mission as `no_autoland`).
static STOOD_DOWN: std::sync::Mutex<Option<&'static str>> = std::sync::Mutex::new(None);

/// The reason the last circuit stood down, if it did.
pub fn stood_down() -> Option<&'static str> {
    *STOOD_DOWN.lock().unwrap_or_else(|e| e.into_inner())
}

/// The circuit's takeoff never completed: a reject, a pilot taking the
/// roll, or a takeoff that handed back on the ground.
pub const TAKEOFF_NOT_COMPLETED: &str = "takeoff_not_completed";
/// The circuit was refused on the runway: no side of it clears the ground
/// (#182). Never flown.
pub const TERRAIN_REFUSED: &str = "terrain_refused";

/// The spoken line for a descent raised over the ground (#182). No dashes.
/// Pure.
pub fn descent_shape_line(shape: DescentShape, field_elev_ft: f64) -> String {
    let mut parts = Vec::new();
    if shape.platform_agl_ft > DOWNWIND_DESCENT_AGL_FT + 1.0 {
        parts.push(format!(
            "holding {:.0} feet on the downwind",
            ((field_elev_ft + shape.platform_agl_ft) / 50.0).ceil() * 50.0
        ));
    }
    if shape.steep() {
        parts.push(format!(
            "a {:.1} degree approach path",
            shape.glidepath_deg()
        ));
    }
    format!("High ground under the approach, {}.", parts.join(", then "))
}

/// The side the circuit asks the terrain pick for (#182): the runway's
/// published pattern side (apt.dat `1101`; LEMG 13 and 31 left) over the
/// one asked for, which only a runway without one keeps. Terrain and
/// airspace still veto it in [`choose_pattern`], and say so. Pure.
pub fn pattern_side_wanted(asked_left: bool, published_left: Option<bool>) -> bool {
    published_left.unwrap_or(asked_left)
}

/// What the crew says about the field's circuit data before the takeoff
/// (#182): a published side that is not the one asked for, and a field
/// whose data says no circuits (`1302 allows_circuits 0`; LEMG, EGKK),
/// flown as asked. `None` when there is nothing to say. No dashes. Pure.
pub fn field_circuit_line(
    field: &str,
    runway: &str,
    asked_left: bool,
    published_left: Option<bool>,
    allows_circuits: Option<bool>,
) -> Option<String> {
    let mut parts = Vec::new();
    if allows_circuits == Some(false) {
        parts.push(format!(
            "The airport data says no circuits at {field}, flying this one as asked."
        ));
    }
    if let Some(left) = published_left.filter(|l| *l != asked_left) {
        parts.push(format!(
            "Runway {} has a published {} hand circuit, flying it {}.",
            spell_runway(runway),
            if left { "left" } else { "right" },
            if left { "left" } else { "right" },
        ));
    }
    (!parts.is_empty()).then(|| parts.join(" "))
}

/// Bail out cleanly: clear our flag and let whatever sub-loop is flying stand
/// down. Never leaves the aircraft with nobody watching the loop.
fn stand_down(xp: &XPlaneClient, voice: &Option<Voice>, line: &str, reason: &'static str) {
    *STOOD_DOWN.lock().unwrap_or_else(|e| e.into_inner()) = Some(reason);
    lower_flag(xp);
    crate::blackbox::record(
        "circuit",
        serde_json::json!({"event": "stand_down", "reason": reason}),
    );
    alert(voice, Priority::Caution, line);
}

// The circuit's laws and its loop in their own files, whole and unchanged
// (refactor phase 4); every path stays `crate::circuit::...` through these
// re-exports.
mod geometry;
pub use geometry::*;
mod guard;
pub use guard::*;
mod pattern;
pub use pattern::*;
mod authored;
pub use authored::*;
mod run;
use run::*;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod final_leg_kts_tests;

/// Issue #147: the LEMG 13 circuit's final and handoff, flown from the
/// recorded base leg of sortie 1790591256-full_auto (2026-09-28) against
/// the real apt.dat geometry and a 737 fitted to the recorded turns.
#[cfg(test)]
mod issue_147_circuit_tests;

/// Issue #189: the A330's LEMG 30 circuits of 2026-09-29 (sorties
/// 1790710769 and 1790698132-full_auto) ran 66 to 67 m through the
/// centreline off the final turn and were handed over coming back, 41 to
/// 43 m off. Flown from the recorded final-turn point against the real
/// 12/30 row with the A330's fitted heading mode and the knob as the loop
/// sets it.
#[cfg(test)]
mod issue_189_tests;

#[cfg(test)]
mod issue_150_climb_out_tests;

/// The SF50's LEMG 13 circuit (sortie 1790607325, 2026-09-28, issue #160):
/// the recorded ticks through the circuit's give-up, and this sortie's
/// handoff flown through a fitted autopilot plant.
#[cfg(test)]
mod pattern_guard_tests;

/// Issue #169: the A330's LEMG 13 circuit (sortie 1790621184-full_auto,
/// 2026-09-28) flew 224 kt on CONF 1 (placard 215): the throttle held the
/// clean-up's 212.5 while the climb limiter granted full rate only at 220.
#[cfg(test)]
mod issue_169_tests;

/// #172, LEMG 13 (2026-09-28, sortie 1790627884-full_auto): the circuit
/// clearance's readback, queued at +13.7 and again at +34.0, went out at
/// +36.1 (the tower's silence: heard) and again at +63.2 in the takeoff roll.
#[cfg(test)]
mod readback_tests;

/// #182 (review of 528b7d1, 2026-09-29): at LXGB the Rock route and the
/// rectangle refused in the AIR stood the circuit down with "Your aircraft"
/// at the takeoff's handoff height. The numbers are the Rock's: 1,398 ft of
/// granite, the 1,098 ft ridge under 09's downwind entered at 1,130 ft in
/// the climb (2026-09-15), the 1,650 ft pattern.
#[cfg(test)]
mod terrain_hold_tests;

/// #182 layer 1: the field's own circuit data. LEMG's apt.dat (2026-09-29)
/// says `1302 allows_circuits 0` and `1101 13 left`, `1101 31 left`.
#[cfg(test)]
mod field_circuit_tests;

/// Issue #188: the circuit handed autoland the final far above the path
/// after a pattern raised for the ground. LEMG 13, SR22, 2026-09-29.
#[cfg(test)]
mod issue_188_on_the_path_tests;
