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

/// Spawn the arrival taxi on its own thread + runtime + client.
pub fn start(
    base_url: String,
    config: Config,
    voice: Option<Voice>,
    park_brake_valve_set: Option<f64>,
    prop: Option<crate::aircraft::PropProfile>,
) {
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            return;
        };
        rt.block_on(fly(
            XPlaneClient::new(base_url),
            config,
            voice,
            park_brake_valve_set,
            prop,
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
            announce(
                &voice,
                Priority::Caution,
                "No taxi-in clearance came back. Your aircraft.",
            );
            crate::blackbox::record("taxi_in", json!({ "event": "no_clearance" }));
            return;
        }
    }

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
    let auto_taxi = AutoTaxi { park_brake_valve_set, prop };
    match auto_taxi.execute(&json!({}), &ctx).await {
        Ok(line) => announce(&voice, Priority::Advisory, &line),
        Err(_) => announce(&voice, Priority::Caution, "I couldn't start the taxi in."),
    }
}
