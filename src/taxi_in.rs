//! Arrival taxi-to-parking — the CREW half. The world (tower) provides the
//! route (see `crates/atc`'s `RequestTaxiIn`, which plans to the nearest stand
//! and publishes it); this asks for it over the crew radio and then drives the
//! published stand route with `AutoTaxi`.
//!
//! It mirrors full_auto's request-then-drive taxi step, scoped to the arrival:
//! request the taxi-in if we don't already hold a stand clearance, wait for the
//! tower to publish it, then hand the drive to `AutoTaxi` — which phrases the
//! destination as "stand ..." off [`crate::party_line::taxi_to_stand`]. With
//! real/external ATC (no simulated tower) the pilot owns the request; we only
//! drive what's already cleared.

use serde_json::json;

use crate::config::Config;
use crate::tools::{taxi::AutoTaxi, Tool, ToolCtx};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const COM1_FREQ_KHZ: &str = crate::datarefs::COM1_FREQ_KHZ;
const PARK_BRAKE: &str = "sim/cockpit2/controls/parking_brake_ratio";

/// Spawn the arrival taxi on its own thread + runtime + client.
pub fn start(
    base_url: String,
    config: Config,
    voice: Option<Voice>,
    park_brake_valve_set: Option<f64>,
    prop: Option<crate::aircraft::PropProfile>,
    pace: crate::autotaxi::Pace,
) {
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(fly(
            XPlaneClient::new(base_url),
            config,
            voice,
            park_brake_valve_set,
            prop,
            pace,
        ));
    });
}

fn announce(voice: &Option<Voice>, priority: Priority, line: &str) {
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

/// A stand (arrival) route is already published — the drive can start now.
fn stand_route_ready() -> bool {
    crate::party_line::taxi_to_stand() && crate::party_line::with_taxi_route(|r| r.is_some())
}

async fn fly(
    xp: XPlaneClient,
    config: Config,
    voice: Option<Voice>,
    park_brake_valve_set: Option<f64>,
    prop: Option<crate::aircraft::PropProfile>,
    pace: crate::autotaxi::Pace,
) {
    // Ask the tower for the taxi-in unless we already hold a stand clearance.
    if !stand_route_ready() {
        let Some(khz) = crate::party_line::home_tower_khz() else {
            // No simulated tower: real/external ATC owns the clearance.
            announce(
                &voice,
                Priority::Advisory,
                "Call the tower for your taxi-in, then say taxi to parking.",
            );
            return;
        };
        // The crew radio only reaches the tuned station — tune the tower first.
        let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
        let callsign = crate::circuit::crew_callsign(&xp).await;
        crate::circuit::transmit(crate::circuit::radio_call(&callsign, "taxi to parking"));
        crate::blackbox::record("taxi_in", json!({ "event": "request", "khz": khz }));
        // Wait for the tower to plan and publish the stand route (~60 s, to
        // outlast a busy frequency working other traffic).
        let mut got = false;
        for _ in 0..120 {
            if stand_route_ready() {
                got = true;
                break;
            }
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        }
        if !got {
            // Say WHICH of the two failed. The tower sets `taxi_to_stand` on
            // its reply and publishes the route separately, so a cleared
            // aircraft with no route is a different fault from a silent
            // frequency — and announcing "no clearance came back" when the
            // clearance plainly did came out as a flat contradiction of what
            // the pilot had just heard (EGHI 2026-07-30: "taxi to parking,
            // this frequency", then sixty seconds of nothing, then that).
            let cleared = crate::party_line::taxi_to_stand();
            let (line, why) = if cleared {
                (
                    "The tower cleared us to parking but sent no route. Your aircraft.",
                    "no_route",
                )
            } else {
                (
                    "No taxi-in clearance came back. Your aircraft.",
                    "no_clearance",
                )
            };
            announce(&voice, Priority::Caution, line);
            crate::blackbox::record("taxi_in", json!({ "event": why, "cleared": cleared }));
            return;
        }
    }

    // Release the brakes autoland set at the stop ("down and stopped, brakes
    // on") — otherwise the taxi governor reads the residual parking-brake
    // pressure as the pilot braking and never applies power, and the aircraft
    // sits stagnant at the numbers (2026-07-26). The same "brakes off" full auto
    // does before a departure taxi.
    let _ = xp.set_float(PARK_BRAKE, 0.0).await;

    // Drive the cleared stand route. AutoTaxi owns the gates (on ground, on the
    // route, steer channel) and speaks the readback ("My taxi. stand ...").
    let ctx = ToolCtx {
        xplane: &xp,
        config: &config,
        voice: voice.as_ref(),
        status: None,
        reminders: None,
    };
    crate::blackbox::record("taxi_in", json!({ "event": "driving" }));
    let auto_taxi = AutoTaxi {
        park_brake_valve_set,
        prop,
        pace,
    };
    match auto_taxi.execute(&json!({}), &ctx).await {
        Ok(line) => announce(&voice, Priority::Advisory, &line),
        Err(_) => {
            announce(&voice, Priority::Caution, "I couldn't start the taxi in.");
            return;
        }
    }

    // Wait for the drive to reach the stand, then SECURE — the arrival mirror of
    // the departure's start-up. `LAST_ARRIVED` distinguishes a real arrival from
    // a hand-back/abort mid-taxi (pilot took the pedals): only secure when we
    // actually parked. Give the drive a beat to arm before watching `ACTIVE`.
    use std::sync::atomic::Ordering::Relaxed;
    tokio::time::sleep(std::time::Duration::from_secs(2)).await;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20 * 60);
    while crate::autotaxi::ACTIVE.load(Relaxed) {
        if std::time::Instant::now() >= deadline {
            return;
        }
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    }
    if crate::autotaxi::LAST_ARRIVED.load(Relaxed) {
        secure(&xp, &voice).await;
    }
}

/// Arrival securing at the stand. STOCK-safe items only — they work on every
/// airframe with no profile: park brake set, transponder to standby, and the
/// exterior lights you don't want on at the gate (strobes, taxi and landing).
/// The beacon and nav lights stay for the ground crew, and ENGINE SHUTDOWN is
/// deliberately left to the pilot for now — it's airframe-specific (a piston's
/// mixture cutoff vs a turboprop's condition levers) and wants sim validation
/// before the copilot cuts an engine on its own.
async fn secure(xp: &XPlaneClient, voice: &Option<Voice>) {
    announce(
        voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "On the stand. Securing.",
            "At the gate. Tidying up.",
            "Parked. Securing the aircraft.",
        ]),
    );
    // Park brake firmly set (the taxi already set it — belt and braces).
    let _ = xp.set_float(PARK_BRAKE, 1.0).await;
    // Transponder to standby now we're off the movement area. Through the
    // shared writer so an add-on's own box (the Twin Otter's KT73) has its
    // KNOB turned — the stock dataref is that airframe's output, not its input.
    let _ = crate::tools::radio::set_mode(xp, crate::tools::radio::XpdrMode::Standby).await;
    // Exterior lights not wanted at the stand (stock commands).
    for cmd in [
        "sim/lights/strobe_lights_off",
        "sim/lights/taxi_lights_off",
        "sim/lights/landing_lights_off",
    ] {
        let _ = xp.run_command(cmd).await;
    }
    crate::blackbox::record("taxi_in", json!({ "event": "secured" }));
    announce(
        voice,
        Priority::Advisory,
        "Parking items done. Shut down when you're ready. Your aircraft.",
    );
}
