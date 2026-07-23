//! Full auto — cold-and-dark to a flown circuit, one command.
//!
//! An ORCHESTRATOR, not new flight logic: it drives the aircraft through the
//! whole gate-to-gate sequence by invoking the SAME tested tools the pilot
//! would, in order, waiting on each before the next:
//!
//!   power up -> start engine -> release brake -> REQUEST TAXI from the tower
//!   -> auto-taxi to the hold -> line up -> fly the circuit (takeoff, pattern,
//!   land, stop).
//!
//! The taxi route is the tower's to give — the copilot can't invent one — so
//! this transmits "request taxi" and drives whatever clearance comes back
//! (needs COM1 on the tower/ground and a station running). Every stage is
//! abortable with "my controls" / "cancel", and each blackboxes as
//! `"kind":"full_auto"`. EXPERIMENTAL: eight subsystems in a chain — fly it
//! VMC, hand ready, and mine the blackbox after.

use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
use std::time::{Duration, Instant};

use serde_json::json;

use crate::config::Config;
use crate::tools::{Tool, ToolCtx};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const ENGN_RUNNING: &str = "sim/flightmodel/engine/ENGN_running";
const PARK_BRAKE: &str = "sim/cockpit2/controls/parking_brake_ratio";
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
const COM1_FREQ_KHZ: &str = crate::datarefs::COM1_FREQ_KHZ;

/// True while the full-auto orchestrator runs.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// Set to cancel ("cancel" / "my controls").
pub static ABORT: AtomicBool = AtomicBool::new(false);

/// One-line status for the dashboard.
pub fn status() -> Option<String> {
    ACTIVE.load(SeqCst).then(|| "full auto".to_string())
}

/// Cancel the sequence and stand down every sub-loop it may have armed.
pub fn abort() -> bool {
    ABORT.store(true, SeqCst);
    crate::autotaxi::ACTIVE.store(false, SeqCst);
    crate::autotakeoff::ACTIVE.store(false, SeqCst);
    crate::circuit::abort();
    crate::autoland::abort();
    ACTIVE.swap(false, SeqCst)
}

/// The tested sub-tools the orchestrator invokes, built from the profile by the
/// arming tool (power-up is stateless, constructed inline).
pub struct SubTools {
    pub start_engine: crate::tools::engine::StartEngine,
    pub taxi: crate::tools::taxi::AutoTaxi,
    pub line_up: crate::tools::taxi::LineUp,
    pub circuit: crate::tools::circuit::Circuit,
    /// true = fly a right-hand pattern.
    pub right_pattern: bool,
    /// Run the electrical/APU power-up before the engine start. Only turbine
    /// airframes need it — a piston's `start_engine` brings its own battery up,
    /// and running PowerUp on a 172 had the copilot narrating an APU it doesn't
    /// have ("APU not coming up", 2026-07-23).
    pub power_up_first: bool,
}

/// Spawn the orchestrator on its own thread + runtime + client.
pub fn start(base_url: String, config: Config, voice: Option<Voice>, tools: SubTools) {
    if ACTIVE.swap(true, SeqCst) {
        return;
    }
    ABORT.store(false, SeqCst);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            ACTIVE.store(false, SeqCst);
            return;
        };
        rt.block_on(fly(XPlaneClient::new(base_url), config, voice, tools));
        ACTIVE.store(false, SeqCst);
    });
}

fn announce(voice: &Option<Voice>, priority: Priority, line: &str) {
    tracing::info!(target: "x_copilot::full_auto", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

fn event(stage: &str, detail: serde_json::Value) {
    crate::blackbox::record("full_auto", json!({ "stage": stage, "detail": detail }));
}

/// Give up cleanly.
fn give_up(voice: &Option<Voice>, line: &str, reason: &str) {
    ACTIVE.store(false, SeqCst);
    event("give_up", json!({ "reason": reason }));
    announce(voice, Priority::Caution, line);
}

async fn engine_running(xp: &XPlaneClient) -> bool {
    xp.get_float_array(ENGN_RUNNING)
        .await
        .ok()
        .and_then(|v| v.first().copied())
        .map(|r| r > 0.5)
        .unwrap_or(false)
}

/// Poll `cond` every `step`, up to `secs`, bailing on ABORT. Returns whether it
/// became true in time.
async fn wait_until<F, Fut>(secs: f64, step_ms: u64, mut cond: F) -> bool
where
    F: FnMut() -> Fut,
    Fut: std::future::Future<Output = bool>,
{
    let deadline = Instant::now() + Duration::from_secs_f64(secs);
    loop {
        if ABORT.load(SeqCst) {
            return false;
        }
        if cond().await {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        tokio::time::sleep(Duration::from_millis(step_ms)).await;
    }
}

async fn fly(xp: XPlaneClient, config: Config, voice: Option<Voice>, tools: SubTools) {
    macro_rules! bail_if_abort {
        () => {
            if ABORT.load(SeqCst) {
                give_up(&voice, "Cancelled. Your aircraft.", "abort");
                return;
            }
        };
    }
    // One context, reused for every tool call (all borrows are immutable).
    let ctx = ToolCtx {
        xplane: &xp,
        config: &config,
        voice: voice.as_ref(),
        status: None,
        reminders: None,
    };

    announce(&voice, Priority::Advisory, "Full auto. Starting from cold.");
    event("start", json!({}));
    // The callsign the tower knows us by, so our calls sound like a real pilot.
    let callsign = crate::circuit::crew_callsign(&xp).await;

    // --- Stage 1: electrics + engine ------------------------------------------
    if !engine_running(&xp).await {
        // Only turbine airframes need the APU/electrical bring-up; a piston's
        // start_engine powers its own bus.
        if tools.power_up_first {
            announce(&voice, Priority::Advisory, "Powering up.");
            let r = crate::tools::power::PowerUp.execute(&json!({}), &ctx).await;
            event("power_up", json!({ "reply": r.as_ref().map(|s| s.as_str()).unwrap_or("err") }));
            bail_if_abort!();
        }

        announce(&voice, Priority::Advisory, "Starting the engine.");
        match tools.start_engine.execute(&json!({}), &ctx).await {
            Ok(reply) => announce(&voice, Priority::Advisory, &reply),
            Err(_) => {}
        }
        // The starter cranks for a few seconds before it catches.
        if !wait_until(40.0, 500, || engine_running(&xp)).await {
            give_up(&voice, "The engine didn't catch. Full auto is off — your aircraft.", "no_engine");
            return;
        }
        event("engine_running", json!({}));
    }
    bail_if_abort!();

    // --- Stage 2: brakes off --------------------------------------------------
    let _ = xp.set_float(PARK_BRAKE, 0.0).await;

    // --- Stage 3: onto the runway ---------------------------------------------
    // Already lined up (the pilot put us on)? Skip straight to the circuit.
    if !lined_up(&xp, &tools, &config).await {
        // The taxi route is the tower's. Ask for it if we don't already hold one.
        if crate::party_line::with_taxi_route(|r| r.is_none()) {
            // Tune COM1 to the tower first — the request goes nowhere if we're
            // off frequency (the crew radio only reaches the tuned station).
            if let Some(khz) = crate::party_line::home_tower_khz() {
                let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
                event("tuned_tower", json!({ "khz": khz }));
                announce(&voice, Priority::Advisory, "Tuning the tower, requesting taxi.");
            } else {
                announce(&voice, Priority::Advisory, "Requesting taxi.");
            }
            crate::circuit::transmit(crate::circuit::radio_call(&callsign, "request taxi"));
            event("request_taxi", json!({}));
            let got = wait_until(25.0, 1000, || async {
                crate::party_line::with_taxi_route(|r| r.is_some())
            })
            .await;
            if !got {
                give_up(
                    &voice,
                    "No taxi clearance came back. Tune the tower and request taxi, then say full auto again.",
                    "no_taxi_clearance",
                );
                return;
            }
        }
        bail_if_abort!();

        // Drive the cleared route to the holding point.
        announce(&voice, Priority::Advisory, "Taxiing to the runway.");
        match tools.taxi.execute(&json!({}), &ctx).await {
            Ok(reply) => announce(&voice, Priority::Advisory, &reply),
            Err(_) => {}
        }
        // Wait for the drive to finish (it clears ACTIVE at the holding point).
        if crate::autotaxi::ACTIVE.load(SeqCst)
            && !wait_until(600.0, 1000, || async { !crate::autotaxi::ACTIVE.load(SeqCst) }).await
        {
            give_up(&voice, "The taxi didn't finish. Your aircraft.", "taxi_stalled");
            return;
        }
        bail_if_abort!();
        event("at_hold", json!({}));

        // ATC: get the departure clearance BEFORE lining up — the tower gates
        // both the line-up and the takeoff on it. No station running means
        // clearance stays None and the tools don't gate; the calls fall on dead
        // air, harmless. The calls are voiced + transcribed by the main loop's
        // crew-radio drain (with the tower's reply), so we only transmit here.
        let atc_running = crate::party_line::clearance_state().is_some();
        if atc_running {
            request_departure(&voice, &callsign).await;
            bail_if_abort!();
        }

        // Line up.
        announce(&voice, Priority::Advisory, "Lining up.");
        match tools.line_up.execute(&json!({}), &ctx).await {
            Ok(reply) => announce(&voice, Priority::Advisory, &reply),
            Err(_) => {}
        }
        if crate::autotaxi::ACTIVE.load(SeqCst)
            && !wait_until(120.0, 1000, || async { !crate::autotaxi::ACTIVE.load(SeqCst) }).await
        {
            give_up(&voice, "The line-up didn't finish. Your aircraft.", "line_up_stalled");
            return;
        }
        bail_if_abort!();
        event("lined_up", json!({}));

        // ATC: make sure we actually hold the takeoff clearance (the tower may
        // have said "line up and wait"), then read it back — the circuit's
        // takeoff refuses without it when a tower is running.
        if atc_running {
            ensure_takeoff_cleared(&callsign).await;
            bail_if_abort!();
        }
    }

    // --- Stage 4: the circuit (takeoff, pattern, land, stop) ------------------
    announce(&voice, Priority::Advisory, "Lined up. Flying the circuit.");
    let args = json!({ "right": tools.right_pattern });
    match tools.circuit.execute(&args, &ctx).await {
        Ok(reply) => announce(&voice, Priority::Advisory, &reply),
        Err(_) => {}
    }
    event("circuit_armed", json!({}));
    // The circuit owns the aircraft from here; our job is done.
    ACTIVE.store(false, SeqCst);
}

/// Call the tower for departure and wait for it to move us to a departure
/// clearance (line-up-and-wait or cleared for takeoff). The call itself is
/// voiced + transcribed by the crew-radio drain, along with the tower's reply.
async fn request_departure(voice: &Option<Voice>, callsign: &Option<String>) {
    use crate::tower::State::{LinedUp, TakeoffCleared};
    announce(voice, Priority::Advisory, "Calling for departure.");
    crate::circuit::transmit(crate::circuit::radio_call(callsign, "ready for departure"));
    event("request_departure", json!({}));
    wait_until(20.0, 1000, || async {
        matches!(crate::party_line::clearance_state(), Some(LinedUp) | Some(TakeoffCleared))
    })
    .await;
}

/// Ask again from line-up-and-wait until the tower clears the takeoff, then
/// read the clearance back (the tower enforces the runway-critical readback).
async fn ensure_takeoff_cleared(callsign: &Option<String>) {
    use crate::tower::State::TakeoffCleared;
    for _ in 0..3 {
        if matches!(crate::party_line::clearance_state(), Some(TakeoffCleared)) {
            break;
        }
        crate::circuit::transmit(crate::circuit::radio_call(callsign, "ready for departure"));
        wait_until(15.0, 1000, || async {
            matches!(crate::party_line::clearance_state(), Some(TakeoffCleared))
        })
        .await;
    }
    if matches!(crate::party_line::clearance_state(), Some(TakeoffCleared)) {
        if let Some(rwy) = crate::party_line::clearance_runway() {
            crate::circuit::transmit(crate::circuit::radio_call(
                callsign,
                &format!("cleared for takeoff runway {}", crate::circuit::spell_runway(&rwy)),
            ));
            event("readback_takeoff", json!({ "runway": rwy }));
        }
    }
}

/// Whether we're already sitting on a runway ready to depart — then taxi and
/// line-up are skipped. Uses the takeoff loop's own fit against apt.dat.
async fn lined_up(xp: &XPlaneClient, tools: &SubTools, _config: &Config) -> bool {
    // Airborne is not "lined up"; and a fit needs a valid position.
    if xp.get_float(ON_GROUND).await.unwrap_or(1.0) < 0.5 {
        return false;
    }
    let (Ok(lat), Ok(lon), Ok(psi)) = (
        xp.get_float("sim/flightmodel/position/latitude").await,
        xp.get_float("sim/flightmodel/position/longitude").await,
        xp.get_float("sim/flightmodel/position/psi").await,
    ) else {
        return false;
    };
    let needed_m = tools.line_up.takeoff_roll_m.map_or(500.0, |m| (m * 1.4).max(500.0));
    let apt = crate::tools::autoland::apt_dat_path();
    let cands = crate::autoland::nearby_runways(&apt, lat, lon, 3.0).unwrap_or_default();
    crate::autotakeoff::runway_fit(&cands, lat, lon, psi, needed_m).is_some()
}
