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

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

use crate::autoland::{along_track_nm, cross_track_nm, nearby_runways, true_to_mag};
use crate::autotakeoff;
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const HPATH: &str = "sim/flightmodel/position/hpath";
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
// AP surface (mirror of autopilot.rs / autoland.rs).
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_ALT_HOLD: &str = "sim/autopilot/altitude_hold";
const HDG_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
const HDG_STATUS: &str = "sim/cockpit2/autopilot/heading_status";

const M_TO_FT: f64 = 3.280_84;
const M_PER_NM: f64 = 1852.0;

// Pattern GEOMETRY, meters, in the departure-runway frame (along = the takeoff
// direction from the departure threshold; cross = right of that course). Timed
// legs looped the aircraft — the 172's autopilot turns ~3 deg/s, so a 90-degree
// turn takes ~30 s, longer than any sane dwell; every leg turning the same way
// with no straight segment spiralled it (LXGB, 2026-07-23). Position triggers
// don't care how slow a turn is: each leg flies until the aircraft REACHES
// the geometric point, so the pattern is a rectangle whatever the turn rate.
/// Climb past the departure end by this before turning crosswind.
const UPWIND_PAST_M: f64 = 300.0;
/// Downwind spacing abeam the runway (~0.5 nm) — how far out the pattern sits.
const PATTERN_WIDTH_M: f64 = 900.0;
/// Fly the downwind until this far PAST the landing threshold (on the approach
/// side), so the base-to-final turn leaves a normal final.
const DOWNWIND_PAST_M: f64 = 1200.0;
/// Turn final once base has brought us back within this of the extended
/// centreline.
const FINAL_CAPTURE_M: f64 = 250.0;
/// Height above the field to hand to autoland on base, feet — low enough that
/// autoland intercepts a sane final, high enough to have turned final safely.
const HANDOFF_AGL_FT: f64 = 700.0;
/// An airborne AGL below this after the takeoff loop exits means it REJECTED
/// (never flew) — the circuit is off, not into the pattern.
const AIRBORNE_MIN_AGL_FT: f64 = 300.0;

/// True while the circuit orchestrator runs.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// Set true to cancel ("cancel circuit" / "my controls").
pub static ABORT: AtomicBool = AtomicBool::new(false);

/// One-line status for the dashboard's "managing" panel.
pub fn status() -> Option<String> {
    ACTIVE.load(Ordering::Relaxed).then(|| "auto circuit".to_string())
}

/// Radio calls the copilot makes on our behalf during the circuit — the ONE
/// place the crew talks to the world. The circuit thread (which doesn't own the
/// station) pushes a position report here; the main loop (which does) drains it
/// each tick, routes it through the tuned tower like a keyed pilot call, and
/// speaks both the copilot's transmission and the tower's reply. Empty in
/// copilot-only mode or with no station — the calls are simply discarded, since
/// real/external ATC owns the radio then.
static CREW_RADIO: Mutex<VecDeque<String>> = Mutex::new(VecDeque::new());

/// Drain the pending crew radio calls (main loop only).
pub fn take_crew_radio() -> Vec<String> {
    let mut q = CREW_RADIO.lock().unwrap_or_else(|e| e.into_inner());
    q.drain(..).collect()
}

/// Queue a radio call for the main loop to transmit. Public so the full-auto
/// orchestrator can request taxi through the same channel.
pub fn transmit(call: String) {
    CREW_RADIO.lock().unwrap_or_else(|e| e.into_inner()).push_back(call);
}

/// The aircraft's callsign, spoken the way the tower says it ("November one
/// seven two Sierra Papa"), so the copilot's calls sound like a real pilot's.
/// `None` if the tail number can't be read.
pub async fn crew_callsign(xp: &XPlaneClient) -> Option<String> {
    xp.get_string(crate::datarefs::TAILNUM)
        .await
        .ok()
        .map(|t| crate::tower::spoken_callsign(&t))
        .filter(|s| !s.is_empty())
}

/// Format a radio call with the callsign in front, pilot-style: "November one
/// seven two Sierra Papa, downwind runway two seven". No callsign → bare.
pub fn radio_call(callsign: &Option<String>, msg: &str) -> String {
    match callsign {
        Some(cs) => format!("{cs}, {msg}"),
        None => msg.to_string(),
    }
}

/// Spell a runway identifier for the radio: "27" -> "two seven", "09L" -> "zero
/// nine left". Digit by digit, with the L/R/C suffix spoken.
pub fn spell_runway(id: &str) -> String {
    const DIGITS: [&str; 10] =
        ["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine"];
    let mut out: Vec<String> = Vec::new();
    for c in id.chars() {
        match c {
            '0'..='9' => out.push(DIGITS[c as usize - '0' as usize].to_string()),
            'L' | 'l' => out.push("left".to_string()),
            'R' | 'r' => out.push("right".to_string()),
            'C' | 'c' => out.push("center".to_string()),
            _ => {}
        }
    }
    out.join(" ")
}

/// Cancel an active circuit and stand down whichever sub-loop is flying;
/// returns whether one was running.
pub fn abort() -> bool {
    ABORT.store(true, Ordering::SeqCst);
    // Stand down whatever phase is live. The takeoff hands off silently (power
    // stays set); the autoland aborts to the pilot.
    autotakeoff::ACTIVE.store(false, Ordering::SeqCst);
    crate::autoland::abort();
    ACTIVE.swap(false, Ordering::SeqCst)
}

/// Everything the orchestrator needs, resolved by the arming tool.
pub struct Params {
    pub apt_dat: String,
    pub cfg: ApConfig,
    /// A fully-built takeoff config — the circuit arms the same takeoff loop.
    pub takeoff: autotakeoff::Params,
    /// Approach speed handed to autoland (KIAS).
    pub approach_ias: f64,
    /// Pattern altitude above the field, feet.
    pub tpa_agl_ft: f64,
    /// Standard left-hand pattern unless the pilot asked for right.
    pub left_pattern: bool,
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

/// The next leg in sequence, or `None` past Final.
pub fn next_leg(leg: Leg) -> Option<Leg> {
    match leg {
        Leg::Upwind => Some(Leg::Crosswind),
        Leg::Crosswind => Some(Leg::Downwind),
        Leg::Downwind => Some(Leg::Base),
        Leg::Base => Some(Leg::Final),
        Leg::Final => None,
    }
}

/// The pure leg-advance decision — POSITION-based, so the sequencing is
/// testable without a sim and immune to how long the autopilot's turns take.
/// `a_m` is distance along the takeoff direction from the departure threshold
/// (negative = on the approach side, before the threshold); `x_m` is cross-
/// track, right of the takeoff course positive. `left` selects the pattern
/// side. Each leg advances when the aircraft REACHES the geometric point, not
/// after a timer.
#[allow(clippy::too_many_arguments)]
pub fn pattern_advance(
    leg: Leg,
    a_m: f64,
    x_m: f64,
    agl_ft: f64,
    tpa_agl_ft: f64,
    runway_len_m: f64,
    left: bool,
) -> bool {
    // The pattern sits on the LEFT (negative cross-track) for a standard
    // circuit; `side * x_m` is then "distance out to the pattern side", always
    // positive as we fly away from the runway on crosswind.
    let _ = (agl_ft, tpa_agl_ft);
    let side = if left { -1.0 } else { 1.0 };
    match leg {
        // Past the departure end — turn crosswind while still climbing to
        // pattern altitude (a real pattern climbs on the crosswind/downwind,
        // not four kilometres straight ahead waiting to reach TPA first).
        Leg::Upwind => a_m >= runway_len_m + UPWIND_PAST_M,
        // Reached the downwind spacing out to the pattern side.
        Leg::Crosswind => side * x_m >= PATTERN_WIDTH_M,
        // Flown back down the downwind until past the landing threshold by
        // enough to build a normal final.
        Leg::Downwind => a_m <= -DOWNWIND_PAST_M,
        // Base has brought us back to (near) the extended centreline.
        Leg::Base => side * x_m <= FINAL_CAPTURE_M,
        Leg::Final => false,
    }
}

/// Spawn the circuit orchestrator on its own thread + runtime + client, the
/// same shape as the takeoff and autoland loops.
pub fn start(base_url: String, voice: Option<Voice>, p: Params) {
    if ACTIVE.swap(true, Ordering::SeqCst) {
        return;
    }
    ABORT.store(false, Ordering::SeqCst);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            ACTIVE.store(false, Ordering::SeqCst);
            return;
        };
        rt.block_on(fly(XPlaneClient::new(base_url.clone()), base_url, voice, p));
        ACTIVE.store(false, Ordering::SeqCst);
        // (base_url is cloned into the client and passed on for the sub-loops.)
    });
}

fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    // Log every spoken line so the auto-flight narration shows up in the app
    // log, not just through the speakers (background loops have no reporter).
    tracing::info!(target: "x_copilot::circuit", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

/// Bail out cleanly: clear our flag and let whatever sub-loop is flying stand
/// down. Never leaves the aircraft with nobody watching the loop.
fn stand_down(voice: &Option<Voice>, line: &str, reason: &str) {
    ACTIVE.store(false, Ordering::SeqCst);
    crate::blackbox::record("circuit", serde_json::json!({"event": "stand_down", "reason": reason}));
    alert(voice, Priority::Caution, line);
}

async fn fly(xp: XPlaneClient, base_url: String, voice: Option<Voice>, p: Params) {
    // `base_url` is kept owned so each sub-loop gets its own client for its own
    // thread (the takeoff loop and the autoland loop each run on one).
    // The departure runway — its course drives every leg heading. Fit it from
    // where we're pointing right now (lined up before the roll), the same way
    // the takeoff loop does.
    let (Ok(lat), Ok(lon), Ok(psi)) =
        (xp.get_float(LAT).await, xp.get_float(LON).await, xp.get_float(PSI_TRUE).await)
    else {
        stand_down(&voice, "I can't read our position. The circuit is off — your aircraft.", "no_position");
        return;
    };
    let cands = nearby_runways(&p.apt_dat, lat, lon, 3.0).unwrap_or_default();
    let Some((rwy, _)) = autotakeoff::runway_fit(&cands, lat, lon, psi, p.takeoff.needed_m) else {
        stand_down(
            &voice,
            "We're not on a runway yet — I can only fly the circuit from the runway itself. \
             Taxi us over and line us up first, then say do a circuit.",
            "not_lined_up",
        );
        return;
    };
    let rwy_course = rwy.course_deg;
    let rwy_id = rwy.id.clone();
    let callsign = crew_callsign(&xp).await;
    let field_elev_ft = rwy.elev_ft;
    let tpa_msl_ft = field_elev_ft + p.tpa_agl_ft;
    crate::blackbox::record(
        "circuit",
        serde_json::json!({
            "event": "armed", "runway": rwy.id, "icao": rwy.icao,
            "course": rwy_course, "field_elev_ft": field_elev_ft,
            "tpa_msl_ft": tpa_msl_ft, "left": p.left_pattern,
        }),
    );
    alert(&voice, Priority::Advisory, "Flying a full circuit. My takeoff first.");

    // --- Phase 1: the takeoff (the proven loop). Arm it, then wait for it to
    // hand off. It clears its own ACTIVE at the thousand-foot handoff — or on a
    // reject, which we tell apart by how high we ended up.
    autotakeoff::start(base_url.clone(), voice.clone(), p.takeoff);
    loop {
        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        if ABORT.load(Ordering::SeqCst) {
            stand_down(&voice, "Circuit cancelled. Your aircraft.", "abort_takeoff_phase");
            return;
        }
        if !autotakeoff::ACTIVE.load(Ordering::SeqCst) {
            break;
        }
    }
    let agl_ft = xp.get_float(AGL_M).await.unwrap_or(0.0) * M_TO_FT;
    if agl_ft < AIRBORNE_MIN_AGL_FT {
        stand_down(&voice, "The takeoff didn't get us airborne, so the circuit is off.", "takeoff_rejected");
        return;
    }
    crate::blackbox::record("circuit", serde_json::json!({"event": "takeoff_handoff", "agl_ft": agl_ft}));
    alert(&voice, Priority::Advisory, "Airborne. Flying the pattern.");

    // --- Phase 2: the pattern on the autopilot. Take the AP the takeoff left
    // engaged, level it at pattern altitude, and walk the legs on the heading
    // bug. Upwind is the current runway heading.
    let _ = xp.run_command(CMD_SERVOS_ON).await;
    if xp.get_float(HDG_STATUS).await.unwrap_or(0.0) <= 0.5 {
        let _ = xp.run_command(CMD_HDG_MODE).await;
    }
    let mag_var = xp.get_float(MAG_VAR).await.unwrap_or(0.0);
    let _ = xp.set_float(ALT_DIAL, tpa_msl_ft).await;
    let _ = xp.run_command(CMD_ALT_HOLD).await;

    let mut leg = Leg::Upwind;
    let set_bug_for = |leg: Leg| leg_heading(rwy_course, leg, p.left_pattern);
    let _ = xp.set_float(HDG_BUG, true_to_mag(set_bug_for(Leg::Upwind), mag_var)).await;

    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if ABORT.load(Ordering::SeqCst) {
            stand_down(&voice, "Circuit cancelled. Your aircraft.", "abort_pattern_phase");
            return;
        }
        // A frozen sim (pause/replay) suspends the sequence; the AP holds.
        if crate::xplane::sim_frozen(&xp).await {
            continue;
        }

        let (lat, lon, agl_m, psi_r, hpath_r) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(AGL_M),
            xp.get_float(PSI_TRUE),
            xp.get_float(HPATH),
        );
        let (Ok(lat), Ok(lon)) = (lat, lon) else { continue };
        let agl_ft = agl_m.unwrap_or(0.0) * M_TO_FT;
        // Position in the departure-runway frame: along the takeoff direction
        // (negative on the approach side) and cross-track (right positive).
        let a_m = -along_track_nm(lat, lon, &rwy) * M_PER_NM;
        let x_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;

        // Fly each leg as a TRACK, not a bare heading: crab into the wind so the
        // AP holds the ground track. Without this a 12 kt crosswind blew the
        // downwind 900 m back into the field (the teardrop, 2026-07-23).
        // crab = heading - track; command heading = leg heading + crab.
        let crab = match (psi_r, hpath_r) {
            (Ok(psi), Ok(hp)) => (((psi - hp + 540.0).rem_euclid(360.0)) - 180.0).clamp(-25.0, 25.0),
            _ => 0.0,
        };
        let want_hdg = leg_heading(rwy_course, leg, p.left_pattern) + crab;
        let _ = xp.set_float(HDG_BUG, true_to_mag(want_hdg, mag_var)).await;

        crate::blackbox::record(
            "circuit",
            serde_json::json!({
                "event": "leg", "leg": format!("{leg:?}"), "agl_ft": agl_ft,
                "a_m": a_m, "x_m": x_m, "crab": crab,
            }),
        );

        if !pattern_advance(leg, a_m, x_m, agl_ft, p.tpa_agl_ft, rwy.length_m, p.left_pattern) {
            continue;
        }
        let Some(nxt) = next_leg(leg) else { continue };
        leg = nxt;

        // The wind-corrected bug is written every tick above; the transitions
        // just announce, report, and change altitude / hand off.
        match leg {
            Leg::Crosswind => {
                alert(&voice, Priority::Advisory, "Crosswind.");
            }
            Leg::Downwind => {
                alert(&voice, Priority::Advisory, "Downwind.");
                // The copilot reports the pattern position to the tower.
                transmit(radio_call(&callsign, &format!("downwind runway {}", spell_runway(&rwy_id))));
            }
            Leg::Base => {
                // Turn base and start down: lower the altitude bug to the
                // autoland handoff height; ALT hold flies it down.
                let _ = xp.set_float(ALT_DIAL, field_elev_ft + HANDOFF_AGL_FT).await;
                let _ = xp.run_command(CMD_ALT_HOLD).await;
                alert(&voice, Priority::Advisory, "Base. Descending.");
                transmit(radio_call(&callsign, "turning base"));
            }
            Leg::Final => {
                crate::blackbox::record("circuit", serde_json::json!({"event": "handoff_autoland", "agl_ft": agl_ft}));
                crate::blackbox::record("circuit", serde_json::json!({"event": "handoff_autoland", "agl_ft": agl_ft}));
                alert(&voice, Priority::Advisory, "Turning final. Landing us.");
                // The final report — this is what earns the landing clearance.
                transmit(radio_call(&callsign, &format!("final runway {}", spell_runway(&rwy_id))));
                crate::autoland::ABORT.store(false, Ordering::SeqCst);
                crate::autoland::run_autoland(
                    // A fresh client for autoland's own thread.
                    base_url.clone(),
                    p.apt_dat.clone(),
                    p.cfg.clone(),
                    p.approach_ias,
                    voice.clone(),
                    None,
                );
                // The circuit's job is done; autoland owns the aircraft now.
                ACTIVE.store(false, Ordering::SeqCst);
                return;
            }
            Leg::Upwind => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn left_pattern_turns_ninety_degrees_left_each_leg() {
        // Runway 27 (course 270), standard left-hand pattern.
        assert_eq!(leg_heading(270.0, Leg::Upwind, true), 270.0);
        assert_eq!(leg_heading(270.0, Leg::Crosswind, true), 180.0);
        assert_eq!(leg_heading(270.0, Leg::Downwind, true), 90.0);
        assert_eq!(leg_heading(270.0, Leg::Base, true), 0.0);
        // Final comes all the way back to the runway heading.
        assert_eq!(leg_heading(270.0, Leg::Final, true), 270.0);
    }

    #[test]
    fn right_pattern_turns_the_other_way_and_wraps() {
        // Runway 09 (course 90), right-hand pattern, checking the 360 wrap.
        assert_eq!(leg_heading(90.0, Leg::Crosswind, false), 180.0);
        assert_eq!(leg_heading(90.0, Leg::Downwind, false), 270.0);
        assert_eq!(leg_heading(90.0, Leg::Base, false), 0.0); // 90+270 = 360 -> 0
        assert_eq!(leg_heading(90.0, Leg::Final, false), 90.0);
    }

    #[test]
    fn the_downwind_is_always_the_reciprocal() {
        for course in [0.0_f64, 27.0, 90.0, 200.0, 359.0] {
            let recip = (course + 180.0).rem_euclid(360.0);
            assert!((leg_heading(course, Leg::Downwind, true) - recip).abs() < 1e-9);
            assert!((leg_heading(course, Leg::Downwind, false) - recip).abs() < 1e-9);
        }
    }

    // Position-based sequencing. A left pattern on a 1500 m runway, tpa 1000 ft.
    // In the runway frame: a_m along the takeoff direction, x_m right-positive;
    // a left pattern sits at NEGATIVE x_m.
    const L: f64 = 1500.0;

    #[test]
    fn upwind_turns_crosswind_past_the_end_while_climbing() {
        // Not yet past the departure end: hold, whatever the altitude.
        assert!(!pattern_advance(Leg::Upwind, L - 100.0, 0.0, 950.0, 1000.0, L, true));
        // Past the end + margin: turn crosswind, even still climbing (600 ft) —
        // a real pattern climbs to TPA on the crosswind/downwind, not straight
        // ahead for kilometres.
        assert!(pattern_advance(Leg::Upwind, L + UPWIND_PAST_M + 1.0, 0.0, 600.0, 1000.0, L, true));
    }

    #[test]
    fn crosswind_turns_downwind_at_the_pattern_width() {
        // Left pattern: still inside the spacing (x only slightly left) -> hold.
        assert!(!pattern_advance(Leg::Crosswind, L, -300.0, 1000.0, 1000.0, L, true));
        // Reached the downwind line out to the left -> turn downwind.
        assert!(pattern_advance(Leg::Crosswind, L, -(PATTERN_WIDTH_M + 1.0), 1000.0, 1000.0, L, true));
        // Right pattern mirrors: the spacing is to the RIGHT (positive x).
        assert!(pattern_advance(Leg::Crosswind, L, PATTERN_WIDTH_M + 1.0, 1000.0, 1000.0, L, false));
        assert!(!pattern_advance(Leg::Crosswind, L, -(PATTERN_WIDTH_M + 1.0), 1000.0, 1000.0, L, false));
    }

    #[test]
    fn downwind_turns_base_past_the_landing_threshold() {
        // Still abeam / not far enough back -> hold.
        assert!(!pattern_advance(Leg::Downwind, 200.0, -900.0, 1000.0, 1000.0, L, true));
        // Flown back past the threshold by the final margin -> turn base.
        assert!(pattern_advance(Leg::Downwind, -(DOWNWIND_PAST_M + 1.0), -900.0, 900.0, 1000.0, L, true));
    }

    #[test]
    fn base_turns_final_near_the_centreline() {
        // Left pattern, still well left of centreline -> hold.
        assert!(!pattern_advance(Leg::Base, -1200.0, -600.0, 800.0, 1000.0, L, true));
        // Crossed back to near the extended centreline -> turn final.
        assert!(pattern_advance(Leg::Base, -1200.0, -(FINAL_CAPTURE_M - 50.0), 800.0, 1000.0, L, true));
    }

    #[test]
    fn spells_runways_for_the_radio() {
        assert_eq!(spell_runway("27"), "two seven");
        assert_eq!(spell_runway("09L"), "zero nine left");
        assert_eq!(spell_runway("36R"), "three six right");
        assert_eq!(spell_runway("6"), "six");
        assert_eq!(spell_runway("13C"), "one three center");
    }

    #[test]
    fn crew_radio_queue_is_fifo_and_drains_once() {
        let _ = take_crew_radio(); // clear any residue
        transmit("Downwind runway two seven".to_string());
        transmit("Final runway two seven".to_string());
        assert_eq!(
            take_crew_radio(),
            vec![
                "Downwind runway two seven".to_string(),
                "Final runway two seven".to_string()
            ]
        );
        assert!(take_crew_radio().is_empty(), "a second drain is empty");
    }

    #[test]
    fn final_is_the_end_of_the_sequence() {
        assert_eq!(next_leg(Leg::Base), Some(Leg::Final));
        assert_eq!(next_leg(Leg::Final), None);
        // Final never self-advances (autoland owns it), whatever the position.
        assert!(!pattern_advance(Leg::Final, -800.0, 0.0, 700.0, 1000.0, L, true));
    }
}
