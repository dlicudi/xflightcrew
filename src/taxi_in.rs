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
/// True heading — the rollout direction, so a vacate never picks an exit
/// behind us (reaching one means taxiing back down the runway).
const PSI_TRUE: &str = "sim/flightmodel/position/psi";

/// The taxi-in has ARRESTED a rolling handoff with the parking brake and is
/// holding on it while the clearance is worked. The monitor's rolling-park-
/// brake caution reads this: the copilot must not nag the pilot about a brake
/// it set itself four seconds earlier (LEMG, 2026-09-12: a manual landing,
/// "taxi to parking" at 38 kt, the arrest, then "Parking brake is set and
/// we're rolling — release it"). Dropped the moment the brake is released
/// for the drive or the taxi-in gives the aircraft back.
pub(crate) static ARREST_HOLD: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Whether the taxi-in is holding the aircraft on the parking brake it set
/// arresting a rolling handoff.
pub fn arrest_holding() -> bool {
    ARREST_HOLD.load(std::sync::atomic::Ordering::Relaxed)
}

fn arrest_release() {
    ARREST_HOLD.store(false, std::sync::atomic::Ordering::Relaxed);
}

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
    shut_down: bool,
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
            shut_down,
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
        // No standing clearance is not evidence we are not on a runway. The
        // clearance was the best PROXY available for "we just landed, and on
        // which strip" — but autoland knows the answer directly, and there
        // are ordinary ways to hold no clearance while sitting on the
        // pavement: a controller-ordered go-around voids it (Malaga
        // 2026-08-05, where the vacate then refused and the aircraft stopped
        // dead on runway 13 with an arrival at 1.3 nm), and copilot-only mode
        // never has one at all. A stale ident is harmless here in a way the
        // stop was not: `plan_vacate` refuses outright unless the aircraft is
        // actually on an authored runway edge.
        .or_else(crate::autoland::landed_runway)
}

/// A stand (arrival) route is already published — the drive can start now.
fn stand_route_ready() -> bool {
    crate::party_line::taxi_to_stand() && crate::party_line::with_taxi_route(|r| r.is_some())
}

/// How long to give the drive off the runway before pressing on regardless:
/// at least this, and more for a longer vacate (`vacate_budget_secs`).
/// Well past it something has gone wrong, and continuing to the taxi-in
/// request beats waiting silently.
const VACATE_MAX_SECS: f64 = 120.0;

/// The vacate's budget, sized on its own route: half as long again as the
/// drive at `kts`, never under [`VACATE_MAX_SECS`]. "A vacate is a hundred
/// metres" was the flat figure's premise, and EDLP's was 1,217 m at 8 kt:
/// halted on the runway with 714 m to go (2026-09-24).
fn vacate_budget_secs(route_m: f64, kts: f64) -> f64 {
    let mps = kts.max(3.0) * 0.514_444;
    (1.5 * route_m.max(0.0) / mps).max(VACATE_MAX_SECS)
}

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
    // stagnant-at-the-numbers trap, 2026-07-26). Through the valve-aware
    // release: a bare ratio write leaves the valve where the stop's
    // `park_brake_set` put it, and an armed valve traps the governor's own
    // first squeeze straight back into the parking brake (2026-08-03).
    crate::tools::controls::release_park_brake_for_roll(xp, park_brake_valve_set).await;
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
    let budget = vacate_budget_secs(route.total_m(), drive_kts);
    crate::autotaxi::start(
        config.xplane_base_url.clone(),
        voice.clone(),
        route,
        drive_kts,
    );

    // Wait for it to finish. Bounded, and a timeout is not fatal: the taxi-in
    // request goes out regardless, which is the other way off the pavement.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs_f64(budget);
    tokio::time::sleep(std::time::Duration::from_secs(2)).await;
    while crate::autotaxi::driving() {
        if std::time::Instant::now() >= deadline {
            // STOP it, don't just stop waiting for it. A drive left running
            // past its budget is still steering, still holding the arrival
            // latch, and the stand drive cannot start on top of it: at BIKF
            // (2026-09-20) this vacate ran 179 s past here with 1,082 m to
            // go, outlived the overrun guard below, and its eventual arrival
            // was taken for the stand's.
            crate::blackbox::record("taxi_in", json!({ "event": "vacate_timeout" }));
            crate::autotaxi::halt();
            let _ = crate::tools::controls::drive_park_brake(xp, true, park_brake_valve_set).await;
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    }
    let arrived = crate::autotaxi::LAST_ARRIVED.load(Relaxed);
    crate::blackbox::record("taxi_in", json!({ "event": "vacated", "arrived": arrived }));
    // Report it — but only a drive that ARRIVED has anything to report. The
    // timeout, a refused join and a pilot handback all reach this line too,
    // possibly still mid-runway, and "runway vacated" transmitted from the
    // strip is a false report to a tower still holding our landing clearance
    // (audit, 2026-08-02). The taxi-in request still goes out either way; an
    // unvacated runway keeps its clearance standing, which is the truth.
    if arrived {
        // The tower's own taxi-in reply comes next, and a controller hearing
        // "runway vacated" before the request is exactly the order.
        let callsign = crate::circuit::crew_callsign(xp).await;
        crate::circuit::transmit(crate::circuit::radio_call(&callsign, "runway vacated"));
        // Off the runway: landing lights off, taxi light on, by the shared
        // rule (an airframe with only a landing lamp keeps it after dusk).
        let dark = crate::tools::lights::dark_now(xp).await;
        let did = crate::tools::lights::after_vacate(xp, dark).await;
        crate::blackbox::record(
            "taxi_in",
            json!({ "event": "lights_vacated", "did": did, "dark": dark }),
        );
    }
}

async fn fly(
    xp: XPlaneClient,
    config: Config,
    voice: Option<Voice>,
    park_brake_valve_set: Option<f64>,
    prop: Option<crate::aircraft::PropProfile>,
    pace: crate::autotaxi::Pace,
    vacate_from: Option<String>,
    shut_down: bool,
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
        // Fail CLOSED on the speed read: the likeliest reason it fails is the
        // same faltering client that just bailed the vacate at
        // `vacate_no_position` — exactly the path most likely to have
        // inherited a rolling handoff — and a redundant brake on a stopped
        // aircraft costs nothing, where a coasting one on a runway is the
        // thing this line exists to stop. And valve-aware like every other
        // brake owner: on a valve airframe the bare ratio write is the
        // fallback `drive_park_brake` only reaches AFTER the valve, and can
        // be a no-op on its own (audit, 2026-08-02).
        let gs_kts = xp
            .get_float("sim/flightmodel/position/groundspeed")
            .await
            .map(|v| v * 1.943_84);
        if gs_kts.as_ref().map_or(true, |gs| *gs > 3.0) {
            ARREST_HOLD.store(true, std::sync::atomic::Ordering::Relaxed);
            let _ = crate::tools::controls::drive_park_brake(&xp, true, park_brake_valve_set).await;
            // Say so: a brake the pilot did not set, on a roll they may still
            // think is theirs, is announced, not discovered.
            announce(
                &voice,
                Priority::Caution,
                "Brakes on, holding for the taxi clearance.",
            );
            crate::blackbox::record(
                "taxi_in",
                json!({ "event": "arrest_rolling", "gs": gs_kts.unwrap_or(-1.0) }),
            );
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
            arrest_release();
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
            arrest_release();
            return;
        }
    }

    // Release the brakes autoland set at the stop ("down and stopped, brakes
    // on") — otherwise the taxi governor reads the residual parking-brake
    // pressure as the pilot braking and never applies power, and the aircraft
    // sits stagnant at the numbers (2026-07-26). The same "brakes off" full auto
    // does before a departure taxi, and valve-aware for the reason above: the
    // ratio-only write left the valve armed to re-trap the first squeeze.
    crate::tools::controls::release_park_brake_for_roll(&xp, park_brake_valve_set).await;
    arrest_release();

    // A helicopter flies the same cleared route as a hover taxi: everything
    // before this (the call, the clearance, the wait) is the aeroplanes'.
    if let Some(collective) = crate::hover::collective_dref() {
        let route = crate::party_line::with_taxi_route(|r| {
            r.filter(|w| w.path.len() >= 2)
                .map(|w| (w.path.clone(), crate::station::spoken_taxiway(&w.via)))
        });
        let Some((path, via)) = route else {
            announce(&voice, Priority::Caution, "No taxi route to fly. Your aircraft.");
            return;
        };
        crate::blackbox::record("taxi_in", json!({ "event": "hover_taxi", "nodes": path.len() }));
        // The park says the lift itself, with the time to stop it.
        crate::heli_route::park(
            voice.clone(),
            config.xplane_base_url.clone(),
            collective,
            crate::heli_route::ParkTarget::Route { path, via },
        );
        return;
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
    let auto_taxi = AutoTaxi {
        park_brake_valve_set,
        prop,
        pace,
        must_start: true,
    };
    // A vacate that over-ran its budget may still be driving; arming the
    // stand drive on top would be refused, and the fall-through below would
    // then read the VACATE's arrival latch as the stand's. Wait it out
    // (bounded), then clear the latch so only THIS drive's arrival can
    // secure — securing on the vacate's own arrival one state earlier is
    // what parked an aircraft "on the stand" just off the strip, transponder
    // to standby and lights off, on a live taxiway (audit, 2026-08-02).
    let overrun = std::time::Instant::now() + std::time::Duration::from_secs(120);
    while crate::autotaxi::driving() && std::time::Instant::now() < overrun {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    }
    // Waiting it out was a RACE, and the vacate won it: the budget here has
    // to be a limit on the old drive, not a hope about it. Anything still
    // driving at this point is stopped outright, which also clears the
    // arrival latch the guard below depends on.
    if crate::autotaxi::driving() {
        crate::blackbox::record("taxi_in", json!({ "event": "overrun_halt" }));
        crate::autotaxi::halt();
        let _ = crate::tools::controls::drive_park_brake(&xp, true, park_brake_valve_set).await;
        // Give the loop its tick to see the flag and let go of the channels.
        tokio::time::sleep(std::time::Duration::from_millis(600)).await;
    }
    crate::autotaxi::LAST_ARRIVED.store(false, std::sync::atomic::Ordering::Relaxed);
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
    if crate::autotaxi::LAST_ARRIVED.load(Relaxed)
        && secure(&xp, &voice, park_brake_valve_set, shut_down).await
        && shut_down
    {
        shut_down_at_stand(&xp, &config, &voice).await;
    }
}

/// Arrival securing at the stand. STOCK-safe items only — they work on every
/// airframe with no profile: park brake set, transponder to standby, and the
/// exterior lights you don't want on at the gate (strobes, taxi and landing).
/// The beacon and nav lights stay for the ground crew. The ENGINES are a
/// mission's to cut (`shut_down_at_stand`), never a spoken "taxi to parking".
/// Returns whether the aircraft is parked ON the stand.
async fn secure(
    xp: &XPlaneClient,
    voice: &Option<Voice>,
    park_brake_valve_set: Option<f64>,
    shut_down: bool,
) -> bool {
    // Are we actually ON a stand? The claim was never checked: any arrival
    // latch at all earned "At the gate", and at BIKF (2026-09-20) that was
    // said 140 m from the nearest stand, ninety degrees across it, on a
    // taxiway, after a drive that never went to the stand it was cleared to.
    //
    // The test already existed for the tug (`pushback::on_a_stand`, an
    // apt.dat 1300 row within 35 m with the nose inside 60 degrees of its
    // parking heading) — this just asks it. A field that publishes no stands
    // keeps the old behaviour, because there is nothing to check against.
    let (lat_r, lon_r, psi_r) = tokio::join!(
        xp.get_float(crate::autotaxi::LAT),
        xp.get_float(crate::autotaxi::LON),
        xp.get_float(crate::autotaxi::PSI_TRUE),
    );
    let at_stand = match (lat_r, lon_r, psi_r) {
        (Ok(lat), Ok(lon), Ok(_)) => crate::pushback::parked_at_stand(
            crate::party_line::nearest_stand(lat, lon).map(|s| (s.dist_m, s.heading)),
            crate::party_line::stands_known(),
        ),
        // A failed read is no measurement: say nothing we cannot support, and
        // do not strip the transponder and lights off an aircraft that may be
        // sitting on live pavement.
        _ => false,
    };
    crate::blackbox::record(
        "taxi_in",
        json!({ "event": "secure", "at_stand": at_stand }),
    );
    if !at_stand {
        // Hold it where it is and hand back. Securing here is what put an
        // aircraft on a taxiway with its transponder in standby.
        let _ = crate::tools::controls::drive_park_brake(xp, true, park_brake_valve_set).await;
        announce(
            voice,
            Priority::Caution,
            "We've stopped short of the stand. Brakes set, but we're not parked \
             and I've left the lights and transponder alone. Your aircraft.",
        );
        crate::full_auto::release_arrival_station();
        return false;
    }
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
    // Through the valve-aware routine, which is what makes "firmly" true on an
    // airframe that models a valve: the drive ISOLATES it so the governor's
    // squeezes cannot latch, and a raw ratio write into an isolated valve is
    // the last thing you want to be trusting at a stand.
    let _ = crate::tools::controls::drive_park_brake(xp, true, park_brake_valve_set).await;
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
    // The arrival is over: hand the station back to the region's own pick.
    crate::full_auto::release_arrival_station();
    if !shut_down {
        announce(
            voice,
            Priority::Readback,
            "Parking items done. Shut down when you're ready. Your aircraft.",
        );
    }
    true
}

/// How long the engines get to wind down after the shutdown tool returns.
const SHUTDOWN_CONFIRM_SECS: f64 = 60.0;

/// A MISSION ends shut down on the stand: the engines through the airframe's
/// own shutdown tool (whichever the registry built for its class: the piston
/// cutoff, a turboprop's condition levers, a jet's fuel levers, the
/// helicopter's), confirmed from `ENGN_running` for EVERY engine, and only
/// then every electrical system off through `set_electrical`. Engines still
/// turning leave the electrics on and say so. Asked 2026-09-24: full auto
/// ended at "Parking items done. Shut down when you're ready."
pub(crate) async fn shut_down_at_stand(xp: &XPlaneClient, config: &Config, voice: &Option<Voice>) {
    let Some(profile) = crate::aircraft::live_profile() else {
        // A hand-back and the flight's last word are never perishable: the
        // closing line was dropped stale behind the frequency at LIPR
        // (2026-09-26) and the pilot never heard the flight end.
        announce(voice, Priority::Readback, "Parked. Your aircraft for the shutdown.");
        return;
    };
    let reg = crate::tools::Registry::for_aircraft(&profile);
    let ctx = ToolCtx {
        xplane: xp,
        config,
        voice: voice.as_ref(),
        status: None,
        reminders: None,
    };
    let Some(stop) = reg.get("stop_engines").or_else(|| reg.get("stop_engine")) else {
        crate::blackbox::record("taxi_in", json!({ "event": "shutdown", "tool": null }));
        announce(
            voice,
            Priority::Readback,
            "Parked. I have no shutdown for this airframe, so the engines are yours. Your aircraft.",
        );
        return;
    };
    let line = match stop.execute(&json!({}), &ctx).await {
        Ok(l) => l,
        Err(e) => format!("The shutdown failed: {e}."),
    };
    let engines = profile.engines.max(1);
    let deadline =
        std::time::Instant::now() + std::time::Duration::from_secs_f64(SHUTDOWN_CONFIRM_SECS);
    let stopped = loop {
        // A failed read is no measurement: keep asking until the deadline.
        if let Ok(r) = xp.get_float_array("sim/flightmodel/engine/ENGN_running").await {
            if engines_stopped(&r, engines) {
                break true;
            }
        }
        if std::time::Instant::now() >= deadline {
            break false;
        }
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    };
    crate::blackbox::record(
        "taxi_in",
        json!({ "event": "shutdown", "tool": stop.name(), "line": line, "stopped": stopped }),
    );
    if !stopped {
        announce(
            voice,
            Priority::Caution,
            &format!(
                "{line} The engines are still turning, so I've left the electrics on. \
                 Your aircraft."
            ),
        );
        return;
    }
    let electrics = match reg.get("set_electrical") {
        Some(t) => Some(
            t.execute(&json!({ "system": "all", "on": false }), &ctx)
                .await
                .unwrap_or_else(|e| format!("The electrics did not switch off: {e}.")),
        ),
        None => None,
    };
    crate::blackbox::record(
        "taxi_in",
        json!({ "event": "electrics_off", "line": electrics }),
    );
    announce(voice, Priority::Readback, &shutdown_line(electrics.as_deref()));
}

/// Every engine the airframe has reads stopped. A short array is not a
/// verdict on the engines it does not cover.
fn engines_stopped(running: &[f64], engines: usize) -> bool {
    running.len() >= engines && running.iter().take(engines).all(|&v| v < 0.5)
}

/// The closing line: the electrics' own readback when one of them did not
/// move, never "all off" over a switch that stayed on.
fn shutdown_line(electrics: Option<&str>) -> String {
    match electrics {
        Some(l) if l.starts_with("Roger") => {
            "Engines shut down and the electrics are off. That's us parked.".to_string()
        }
        Some(l) => format!("Engines shut down. {l}"),
        None => "Engines shut down. The electrics are yours; I can't switch them here.".to_string(),
    }
}

#[cfg(test)]
mod shutdown_tests {
    use super::{engines_stopped, shutdown_line, vacate_budget_secs, VACATE_MAX_SECS};

    #[test]
    fn a_long_vacate_gets_the_time_its_route_needs() {
        // EDLP 24 (2026-09-24): 1,217 m at 8 kt is about 296 s of driving.
        assert!(vacate_budget_secs(1_217.0, 8.0) >= 400.0);
        assert_eq!(vacate_budget_secs(100.0, 8.0), VACATE_MAX_SECS);
        assert_eq!(vacate_budget_secs(f64::NAN.max(0.0), 0.0), VACATE_MAX_SECS);
    }

    #[test]
    fn every_engine_must_read_stopped_and_a_short_array_is_no_verdict() {
        assert!(engines_stopped(&[0.0, 0.0, 1.0], 2));
        assert!(!engines_stopped(&[0.0, 1.0], 2), "the LAST engine too");
        assert!(!engines_stopped(&[0.0], 2));
        assert!(engines_stopped(&[0.0; 8], 1));
    }

    #[test]
    fn the_closing_line_never_claims_electrics_that_did_not_move() {
        assert!(shutdown_line(Some("Roger, all electrical systems off.")).contains("electrics are off"));
        let l = shutdown_line(Some("The avionics didn't move."));
        assert!(l.contains("avionics didn't move") && !l.contains("electrics are off"), "{l}");
        assert!(shutdown_line(None).contains("electrics are yours"));
    }

    /// A mission ends shut down on the stand, so every airframe full auto
    /// can fly must have a shutdown the registry built.
    #[test]
    fn every_full_auto_airframe_has_a_shutdown_for_the_stand() {
        let mut checked = 0;
        for entry in std::fs::read_dir("profiles").unwrap() {
            let path = entry.unwrap().path();
            if path.extension().and_then(|e| e.to_str()) != Some("yaml") {
                continue;
            }
            let id = path.file_stem().unwrap().to_str().unwrap().to_string();
            let Some(p) = crate::aircraft::load(&id) else { continue };
            let reg = crate::tools::Registry::for_aircraft(&p);
            if reg.get("full_auto").is_none() {
                continue;
            }
            checked += 1;
            assert!(
                reg.get("stop_engines").is_some() || reg.get("stop_engine").is_some(),
                "{id}: full auto can fly it but nothing can shut it down at the stand"
            );
        }
        assert!(checked > 3, "only {checked} full-auto airframes found");
    }
}
