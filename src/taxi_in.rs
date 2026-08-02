//! Arrival taxi-to-parking — the CREW half, in two phases: VACATE, then taxi.
//!
//! **Vacating comes first and asks nobody.** Clearing the runway is the
//! completion of the landing clearance we already hold, not a new instruction,
//! so it needs no controller in the loop and owes no readback. This used to be
//! the other way round: the request went out from wherever autoland stopped —
//! on the runway — and the drive waited up to sixty seconds for the tower to
//! publish a stand route, with the give-up path handing back "your aircraft"
//! still on the strip. At Southampton that held runway 02 for ninety seconds
//! after touchdown while an arrival closed from 2.6 nm to 0.5, the tower
//! telling the lander "continue approach, traffic on the runway ahead"
//! (2026-08-01). Sixteen of those seconds were spent waiting for a clearance
//! to do something that never needed one.
//!
//! Phase two is the original: the world (tower) provides the stand route (see
//! `crates/atc`'s `RequestTaxiIn`), this asks for it over the crew radio and
//! hands the drive to `AutoTaxi` — which phrases the destination as "stand ..."
//! off [`crate::party_line::taxi_to_stand`]. With real/external ATC (no
//! simulated tower) the pilot owns the request; we only drive what's already
//! cleared. Note that a copilot-only boot has no station and therefore no taxi
//! network either, so the vacate degrades to nothing there — the pilot is
//! flying with real ATC and owns the rollout.

use serde_json::json;

use crate::config::Config;
use crate::tools::{taxi::AutoTaxi, Tool, ToolCtx};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const COM1_FREQ_KHZ: &str = crate::datarefs::COM1_FREQ_KHZ;
const PARK_BRAKE: &str = "sim/cockpit2/controls/parking_brake_ratio";
/// True heading — the rollout direction, so a vacate never picks an exit
/// behind us (reaching one means taxiing back down the runway).
const PSI_TRUE: &str = "sim/flightmodel/position/psi";

/// Spawn the arrival taxi on its own thread + runtime + client.
/// `vacate_from` is the runway END we have just landed on, when we have: the
/// drive off it runs before anything is requested. `None` means we are already
/// on a taxiway (the pilot asking for a taxi-in at the holding point) and there
/// is nothing to vacate — build it with [`vacate_runway`], never from
/// `clearance_runway` alone.
#[allow(clippy::too_many_arguments)]
pub fn start(
    base_url: String,
    config: Config,
    voice: Option<Voice>,
    park_brake_valve_set: Option<f64>,
    prop: Option<crate::aircraft::PropProfile>,
    pace: crate::autotaxi::Pace,
    vacate_from: Option<String>,
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
            vacate_from,
        ));
    });
}

fn announce(voice: &Option<Voice>, priority: Priority, line: &str) {
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

/// The runway to vacate, or `None` when there is nothing to vacate.
///
/// A STANDING LANDING CLEARANCE is the signal, and it is the right one
/// precisely because of how narrow it is: the tower issues it on final and the
/// taxi-in request is what clears it, so the state holds over exactly the
/// window between touchdown and asking to go to the stand. Reading
/// `clearance_runway` on its own would hand a stale ident from an earlier
/// flight to a drive that then aims for a runway exit from the middle of a
/// taxiway — a process-wide fact outliving its moment, which is the shape of
/// half the bugs in this codebase. With no simulated tower there is no
/// clearance and no taxi network either, so this is `None` and the pilot owns
/// the rollout, as they already did.
pub fn vacate_runway() -> Option<String> {
    (crate::party_line::clearance_state() == Some(crate::tower::State::ClearedToLand))
        .then(crate::party_line::clearance_runway)
        .flatten()
}

/// A stand (arrival) route is already published — the drive can start now.
fn stand_route_ready() -> bool {
    crate::party_line::taxi_to_stand() && crate::party_line::with_taxi_route(|r| r.is_some())
}

/// How long to give the drive off the runway before pressing on regardless.
/// A vacate is a hundred metres at taxi pace; well past that, something has
/// gone wrong and continuing to the taxi-in request beats waiting silently.
const VACATE_MAX_SECS: f64 = 120.0;

/// Phase one: drive clear of `rwy`, then report it.
///
/// Best effort throughout, and every failure falls through to the taxi-in
/// exactly as it behaved before this existed — no network at the field, a
/// runway with no authored exits, a drive that refuses. The one thing this
/// must never do is BLOCK the taxi-in, because the taxi-in is the other way
/// off the runway.
async fn vacate(
    xp: &XPlaneClient,
    config: &Config,
    voice: &Option<Voice>,
    park_brake_valve_set: Option<f64>,
    prop: Option<crate::aircraft::PropProfile>,
    pace: crate::autotaxi::Pace,
    rwy: &str,
) {
    use std::sync::atomic::Ordering::Relaxed;
    let (lat_r, lon_r, psi_r) = tokio::join!(
        xp.get_float(crate::datarefs::LATITUDE),
        xp.get_float(crate::datarefs::LONGITUDE),
        xp.get_float(PSI_TRUE),
    );
    let (Ok(lat), Ok(lon), Ok(psi)) = (lat_r, lon_r, psi_r) else {
        crate::blackbox::record("taxi_in", json!({ "event": "vacate_no_position" }));
        return;
    };
    let Some(plan) = crate::party_line::plan_vacate(lat, lon, psi, rwy) else {
        // No network, or no exit ahead. The taxi-in still runs; it just starts
        // from the runway as it always did.
        crate::blackbox::record("taxi_in", json!({ "event": "vacate_no_plan", "rwy": rwy }));
        return;
    };
    let watch = crate::station::TaxiWatch {
        path: plan.path.clone(),
        via: String::new(),
        seg: plan.seg.clone(),
        holding_point: String::new(),
        lead_on: Vec::new(),
    };
    let Some(mut route) = crate::autotaxi::Route::build(&watch) else {
        // Shorter than the drive's own minimum: we are close enough to clear
        // that the taxi-in's first metres will do it.
        crate::blackbox::record(
            "taxi_in",
            json!({ "event": "vacate_too_short", "pts": plan.path.len() }),
        );
        return;
    };
    route.pace = pace;
    route.done_line = Some(if plan.exit_name.is_empty() {
        "Clear of the runway.".to_string()
    } else {
        format!(
            "Clear of the runway on {}.",
            crate::station::spoken_taxiway(&plan.exit_name)
        )
    });
    crate::blackbox::record(
        "taxi_in",
        json!({ "event": "vacating", "rwy": rwy, "exit": plan.exit_name,
                "pts": plan.path.len() }),
    );
    announce(voice, Priority::Advisory, "Clearing the runway.");
    // Brakes off — autoland set them at the stop, and the governor reads the
    // residual pressure as the pilot braking and never applies power (the
    // stagnant-at-the-numbers trap, 2026-07-26).
    let _ = xp.set_float(PARK_BRAKE, 0.0).await;
    // Governor through the SAME front door the auto taxi and the line-up use,
    // so every parking-brake and held-brake gate applies to a vacate
    // unchanged. Driving without it would be a drive with nobody on the
    // throttle, which is the one failure this module must not add.
    let ctx = ToolCtx {
        xplane: xp,
        config,
        voice: voice.as_ref(),
        status: None,
        reminders: None,
    };
    let drive_kts = crate::tools::speedhold::clamp_taxi_target(pace.turn).max(3.0);
    let armed = crate::tools::speedhold::HoldSpeed {
        approach_kts: None,
        park_brake_valve_set,
    }
    .execute(&json!({ "knots": drive_kts }), &ctx)
    .await;
    if armed.is_err() || !crate::tools::speedhold::taxi_hold_active() {
        crate::blackbox::record("taxi_in", json!({ "event": "vacate_no_governor" }));
        return;
    }
    // Props out of feather before the drive rolls (the same wall auto taxi
    // and the line-up hit): a turboprop with feathered blades makes no thrust.
    crate::tools::prop::advance_for_ground_roll(xp, &prop).await;
    crate::autotaxi::start(
        config.xplane_base_url.clone(),
        voice.clone(),
        route,
        drive_kts,
    );

    // Wait for it to finish. Bounded, and a timeout is not fatal: the taxi-in
    // request goes out regardless, which is the other way off the pavement.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs_f64(VACATE_MAX_SECS);
    tokio::time::sleep(std::time::Duration::from_secs(2)).await;
    while crate::autotaxi::driving() {
        if std::time::Instant::now() >= deadline {
            crate::blackbox::record("taxi_in", json!({ "event": "vacate_timeout" }));
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    }
    crate::blackbox::record(
        "taxi_in",
        json!({ "event": "vacated", "arrived": crate::autotaxi::LAST_ARRIVED.load(Relaxed) }),
    );
    // Report it. The tower's own taxi-in reply comes next, and a controller
    // hearing "runway vacated" before the request is exactly the order.
    let callsign = crate::circuit::crew_callsign(xp).await;
    crate::circuit::transmit(crate::circuit::radio_call(&callsign, "runway vacated"));
}

async fn fly(
    xp: XPlaneClient,
    config: Config,
    voice: Option<Voice>,
    park_brake_valve_set: Option<f64>,
    prop: Option<crate::aircraft::PropProfile>,
    pace: crate::autotaxi::Pace,
    vacate_from: Option<String>,
) {
    // --- Phase one: get off the runway. No clearance, no request, no wait. ---
    if let Some(rwy) = vacate_from {
        vacate(
            &xp,
            &config,
            &voice,
            park_brake_valve_set,
            prop.clone(),
            pace,
            &rwy,
        )
        .await;
    }
    // Autoland may have handed off ROLLING (taxi pace, brakes released) on the
    // promise that the vacate drive takes over. Any vacate failure path breaks
    // that promise, and the request wait below is minutes of nobody on the
    // brakes — the same rule as pushback: a stage that is not moving the
    // aircraft is holding it.
    if !crate::autotaxi::driving() {
        let gs = xp
            .get_float("sim/flightmodel/position/groundspeed")
            .await
            .unwrap_or(0.0)
            * 1.943_84;
        if gs > 3.0 {
            let _ = xp.set_float(PARK_BRAKE, 1.0).await;
            crate::blackbox::record("taxi_in", json!({ "event": "arrest_rolling", "gs": gs }));
        }
    }

    // --- Phase two: the taxi-in proper, from wherever phase one left us. ---
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
