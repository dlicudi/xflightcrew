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
//! off [`crate::party_line::taxi_to_stand`]. With built-in ATC enabled but no
//! tower at the field, the crew plans its own route from the apt.dat taxi
//! network. With real/external ATC (copilot-only mode), the pilot owns the
//! request; we only drive what's already cleared. Note that a copilot-only boot
//! has no station and therefore no taxi network either, so the vacate degrades
//! to nothing there — the pilot is flying with real ATC and owns the rollout.

use serde_json::json;

use crate::config::Config;
use crate::tools::controls::HoldFor;
use crate::tools::{taxi::AutoTaxi, Tool, ToolCtx};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const COM1_FREQ_KHZ: &str = crate::datarefs::COM1_FREQ_KHZ;
/// True heading — the rollout direction, so a vacate never picks an exit
/// behind us (reaching one means taxiing back down the runway).
const PSI_TRUE: &str = "sim/flightmodel/position/psi";

/// The pilot's release: a hand-back's own gates (a tug, an unread speed).
const RELEASE_HOLD: HoldFor = HoldFor::by("taxi_in");
/// The taxi-in's own holds (the arrest, a halted drive): it knows the
/// aircraft is on the ground, so an unread speed still brakes and parks.
const TAXI_IN_HOLD: HoldFor = HoldFor::by("taxi_in").fail_closed();

/// A drive this taxi-in halted is still on the governor's throttle: hold it
/// at zero (idle and its brakes) and stop and park on the one seam
/// ([`crate::tools::controls::hand_back_hold`]), read back. It used to set
/// the parking brake unconfirmed on a rolling aircraft (refactor audit
/// 2026-10-04, duplicates #8).
async fn stop_and_park(xp: &XPlaneClient, valve_set: Option<f64>) -> Option<bool> {
    crate::tools::speedhold::set_taxi_target(0.0);
    crate::tools::controls::hand_back_hold(xp, valve_set, TAXI_IN_HOLD).await
}

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

/// A taxi-in is running: from [`start`] to the end of its thread, vacate,
/// request and drive alike (the request's wait drives nothing, so the auto
/// taxi's flag alone does not say it). Written only by the run ([`RUN`]).
pub(crate) static ACTIVE: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);
/// The taxi-in's run: a newer [`start`] takes it over, [`abort`] cancels it,
/// and the run that ends (or was superseded) can never clear a newer one's
/// flag.
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::flagged(&ACTIVE);
/// With an [`abort`]: whether the end stops and parks the aircraft (the
/// pilot's "my controls") or only lets go (a stand-down for a swap or a new
/// flight, whose setup owns the aircraft). Reset by every new run.
static CANCEL_HOLD: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Whether a taxi-in is running.
pub fn running() -> bool {
    ACTIVE.load(std::sync::atomic::Ordering::SeqCst)
}

/// End the running taxi-in where it is: its drive halted, its governor off,
/// nothing requested or driven after. `hold`: the aircraft is then braked to
/// a stop and parked (`full_auto::hand_back_hold`), the pilot's release. It
/// used to be only the auto taxi's flag: LEMG (2026-09-28, t 1790591992.8)
/// "my controls" in the vacate stopped the drive, and the taxi-in reported
/// vacated, called for taxi and drove to the stand 7 s later.
pub fn abort(hold: bool) {
    use std::sync::atomic::Ordering::SeqCst;
    if !RUN.live() {
        return;
    }
    CANCEL_HOLD.store(hold, SeqCst);
    RUN.cancel();
    crate::autotaxi::halt();
    crate::tools::speedhold::release();
}

/// A new taxi-in is running from now: its run (the flag drops with it,
/// however the taxi-in ends, unless a newer one has taken over).
fn begin() -> sim::run::Lease<'static> {
    CANCEL_HOLD.store(false, std::sync::atomic::Ordering::SeqCst);
    RUN.take_over()
}

/// This taxi-in is to end: aborted, or superseded by a newer one.
fn cancelled(run: &sim::run::Lease<'_>) -> bool {
    !run.live()
}

/// The taxi-in was told to end at `at`: blackboxed, and on the pilot's
/// release the aircraft stopped and parked, the parking brake said. Always
/// true, so a checkpoint reads `if cancelled(gen) && end_cancelled(..).await`.
async fn end_cancelled(
    xp: &XPlaneClient,
    voice: &Option<Voice>,
    park_brake_valve_set: Option<f64>,
    at: &str,
) -> bool {
    use std::sync::atomic::Ordering::SeqCst;
    let hold = CANCEL_HOLD.load(SeqCst);
    let held = match hold {
        true => {
            crate::tools::controls::hand_back_hold(xp, park_brake_valve_set, RELEASE_HOLD).await
        }
        false => None,
    };
    crate::blackbox::record(
        "taxi_in",
        json!({ "event": "cancelled", "at": at, "hold": hold, "held": held }),
    );
    match held {
        Some(true) => announce(voice, Priority::Advisory, "Parking brake set."),
        Some(false) => announce(
            voice,
            Priority::Caution,
            "The parking brake did not set. Hold the brakes.",
        ),
        None => {}
    }
    arrest_release();
    true
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
    // Running from here, not from the thread's first poll: a "my controls"
    // typed the moment the landing's verdict is in must find it.
    let gen = begin();
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
            &gen,
        ));
    });
}

/// The aircraft was moved under the taxi-in: stop here, ask nothing, drive
/// nothing. Said only when the drive's own guard has not already said it.
fn stand_down_relocated(voice: &Option<Voice>, at: &str, said: bool) {
    crate::blackbox::record(
        "taxi_in",
        json!({ "event": "relocated", "at": at, "said": said }),
    );
    if !said {
        announce(
            voice,
            Priority::Caution,
            "We've been moved. Taxi-in cancelled. Your aircraft.",
        );
    }
    arrest_release();
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

/// Who owns a taxi-in when no simulated tower reaches the aircraft.
///
/// The world mode is the authority, not an arrival marker published by one
/// orchestrator. A pilot-flown arrival never passes full auto's
/// `begin_uncontrolled_arrival` seam, but with built-in ATC running the absence
/// of a reachable tower still means there is nobody to ask and the crew must
/// plan the uncontrolled-field route itself. In copilot-only mode the absent
/// station means real/external ATC, so the pilot owns the clearance.
fn crew_owns_taxi_without_tower(atc_enabled: bool) -> bool {
    atc_enabled
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

/// How phase one ended, for the only question phase two asks of it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VacateEnd {
    /// Driven, refused, timed out or handed back: the taxi-in goes on from
    /// wherever the aircraft is, as it always did.
    Carry,
    /// The aircraft was MOVED under the vacate (a reposition, a sim reload):
    /// nothing planned from the rollout means anything any more, and a
    /// taxi-in request from the new place is a flight nobody flew. `said`:
    /// the drive's own guard already told the pilot.
    Relocated { said: bool },
}

/// The aircraft's position, or `None` on a failed read (no measurement).
/// One frame's latitude and longitude (#157), else the two stock reads.
async fn position(xp: &XPlaneClient) -> Option<(f64, f64)> {
    let (lat, lon, _) = sim::position::read_lat_lon(xp).await.ok()?;
    Some((lat, lon))
}

/// Whether a vacate route planned from the rollout still starts where the
/// aircraft is: `from` is where it was planned, `now` a fresh read `dt_s`
/// later, `lateral_m` the fresh read's distance off the route. Either a jump
/// ([`crate::autotaxi::relocation_jump_m`]) or a route further off than any
/// join ([`crate::autotaxi::MAX_JOIN_M`], the auto taxi tool's own refusal,
/// which the vacate's direct start used to skip) is a vacate that must not
/// start. LEMG 31 (2026-09-28): planned at the rollout's stop, engaged 0.3 s
/// later 1,033 m off its line after the sortie runner reloaded the aircraft.
fn vacate_still_here(from: (f64, f64), now: (f64, f64), dt_s: f64, lateral_m: f64) -> bool {
    crate::autotaxi::relocation_jump_m(from, now, dt_s).is_none()
        && lateral_m <= crate::autotaxi::MAX_JOIN_M
}

/// Phase one: drive clear of `rwy`, then report it.
///
/// Best effort throughout, and every failure falls through to the taxi-in
/// exactly as it behaved before this existed: no network at the field, a
/// runway with no authored exits, a drive that refuses. The one thing this
/// must never do is BLOCK the taxi-in, because the taxi-in is the other way
/// off the runway. The one exception is an aircraft MOVED under it
/// ([`VacateEnd::Relocated`]): there is no runway of ours to get off.
async fn vacate(
    xp: &XPlaneClient,
    config: &Config,
    voice: &Option<Voice>,
    park_brake_valve_set: Option<f64>,
    prop: Option<crate::aircraft::PropProfile>,
    pace: crate::autotaxi::Pace,
    rwy: &str,
) -> VacateEnd {
    use std::sync::atomic::Ordering::Relaxed;
    let planned_at = std::time::Instant::now();
    let (lat_r, lon_r, psi_r) = tokio::join!(
        xp.get_float(crate::datarefs::LATITUDE),
        xp.get_float(crate::datarefs::LONGITUDE),
        xp.get_float(PSI_TRUE),
    );
    let (Ok(lat), Ok(lon), Ok(psi)) = (lat_r, lon_r, psi_r) else {
        crate::blackbox::record("taxi_in", json!({ "event": "vacate_no_position" }));
        return VacateEnd::Carry;
    };
    let Some(plan) = crate::party_line::plan_vacate(lat, lon, psi, rwy) else {
        // No network, or no exit ahead. The taxi-in still runs; it just starts
        // from the runway as it always did.
        crate::blackbox::record("taxi_in", json!({ "event": "vacate_no_plan", "rwy": rwy }));
        return VacateEnd::Carry;
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
        return VacateEnd::Carry;
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
        return VacateEnd::Carry;
    }
    // Props out of feather before the drive rolls (the same wall auto taxi
    // and the line-up hit): a turboprop with feathered blades makes no thrust.
    crate::tools::prop::advance_for_ground_roll(xp, &prop).await;
    let budget = vacate_budget_secs(route.total_m(), drive_kts);
    // Is the route still where we are? Read again at the last moment: the
    // plan is from the rollout's stop, and the governor's arm and the brake
    // release took time the aircraft may have spent somewhere else.
    if let Some(now) = position(xp).await {
        let lateral = route
            .fix(now.0, now.1)
            .map_or(f64::INFINITY, |(_, l)| l.abs());
        let dt_s = planned_at.elapsed().as_secs_f64();
        if !vacate_still_here((lat, lon), now, dt_s, lateral) {
            let jump_m = crate::autoland::haversine_nm(lat, lon, now.0, now.1) * 1852.0;
            crate::blackbox::record(
                "taxi_in",
                json!({ "event": "vacate_relocated", "jump_m": jump_m,
                        "lateral_m": lateral, "dt_s": dt_s }),
            );
            // Nothing drives, so nothing may be driving: the governor just
            // armed at a rolling target holds at zero instead.
            crate::tools::speedhold::set_taxi_target(0.0);
            return VacateEnd::Relocated { said: false };
        }
    }
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
            crate::autotaxi::halt();
            let held = stop_and_park(xp, park_brake_valve_set).await;
            crate::blackbox::record(
                "taxi_in",
                json!({ "event": "vacate_timeout", "held": held }),
            );
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    }
    if crate::autotaxi::RELOCATED.load(Relaxed) {
        crate::blackbox::record(
            "taxi_in",
            json!({ "event": "vacated", "arrived": false, "relocated": true }),
        );
        return VacateEnd::Relocated { said: true };
    }
    // Arrived is the drive's word; OFF the strip is the pavement's. A route
    // end inside the strip's lateral span (thin data, a short exit) is still
    // the runway, and "runway vacated" from it is a false report.
    let drove_there = crate::autotaxi::LAST_ARRIVED.load(Relaxed);
    let on_runway = match position(xp).await {
        Some((la, lo)) => crate::party_line::on_runway(la, lo),
        None => None,
    };
    let arrived = vacated_report_due(drove_there, on_runway);
    crate::blackbox::record(
        "taxi_in",
        json!({ "event": "vacated", "arrived": arrived, "drove_there": drove_there,
                "on_runway": on_runway }),
    );
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
    VacateEnd::Carry
}

/// Whether "runway vacated" may go out: the drive ARRIVED and the pavement
/// does not say we are still on a strip. No runway data (`None`) is no
/// evidence either way and leaves the drive's word standing.
fn vacated_report_due(arrived: bool, on_runway: Option<bool>) -> bool {
    arrived && on_runway != Some(true)
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
    gen: &sim::run::Lease<'_>,
) {
    // --- Phase one: get off the runway. No clearance, no request, no wait. ---
    if let Some(rwy) = vacate_from {
        let end = vacate(
            &xp,
            &config,
            &voice,
            park_brake_valve_set,
            prop.clone(),
            pace,
            &rwy,
        )
        .await;
        if let VacateEnd::Relocated { said } = end {
            stand_down_relocated(&voice, "vacate", said);
            return;
        }
    }
    let valve = park_brake_valve_set;
    if cancelled(gen) && end_cancelled(&xp, &voice, valve, "vacate").await {
        return;
    }
    // Autoland may have handed off ROLLING (taxi pace, brakes released) on the
    // promise that the vacate drive takes over. Any vacate failure path breaks
    // that promise, and the request wait below is minutes of nobody on the
    // brakes — the same rule as pushback: a stage that is not moving the
    // aircraft is holding it.
    if !crate::autotaxi::driving() {
        // Stopped and parked on the one seam (`hand_back_hold`): the wheel
        // brakes to a stop, the held command's tail, then the valve-aware
        // parking brake, read back. It used to set the PARKING brake on the
        // rolling aircraft above 3 kt and leave one under 3 kt to roll
        // (refactor audit 2026-10-04, duplicates #8). Fail CLOSED on the
        // speed read (`TAXI_IN_HOLD`): the likeliest reason it fails is the
        // same faltering client that just bailed the vacate at
        // `vacate_no_position`, exactly the path most likely to have
        // inherited a rolling handoff (audit, 2026-08-02). Flagged first: the
        // held brake commands ride the parking-brake ratio, and the monitor's
        // rolling-park-brake caution must not nag about our own stop.
        let gs_kts = xp
            .get_float(crate::datarefs::GROUNDSPEED_MS)
            .await
            .ok()
            .map(|v| v * sim::ground::MS_TO_KTS);
        ARREST_HOLD.store(true, std::sync::atomic::Ordering::Relaxed);
        let held = crate::tools::controls::hand_back_hold(&xp, park_brake_valve_set, TAXI_IN_HOLD)
            .await;
        // Say so: a brake the pilot did not set, on a roll they may still
        // think is theirs, is announced, not discovered. And say what the
        // read-back says: "Brakes on" over a brake that did not take was
        // the EGSS rollout's untrue line (2026-09-22, issue #65).
        match held {
            Some(true) => announce(
                &voice,
                Priority::Caution,
                "Brakes on, holding for the taxi clearance.",
            ),
            Some(false) => announce(
                &voice,
                Priority::Caution,
                "The parking brake did not set. Hold the brakes, we may still be rolling.",
            ),
            // Skipped (airborne, a tug on the bar, a pilot on the toe brakes
            // at speed): nothing of ours holds it.
            None => arrest_release(),
        }
        crate::blackbox::record(
            "taxi_in",
            json!({ "event": "arrest_rolling", "gs": gs_kts.unwrap_or(-1.0), "held": held }),
        );
    }

    // --- Phase two: the taxi-in proper, from wherever phase one left us. ---
    // Ask the tower for the taxi-in unless we already hold a stand clearance.
    if !stand_route_ready() {
        if let Some(khz) = crate::party_line::home_tower_khz() {
            // The crew radio only reaches the tuned station — tune the tower first.
            let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
            let callsign = crate::circuit::crew_callsign(&xp).await;
            crate::circuit::transmit(crate::circuit::radio_call(&callsign, "taxi to parking"));
            crate::blackbox::record("taxi_in", json!({ "event": "request", "khz": khz }));
            // Wait for the tower to plan and publish the stand route (~60 s, to
            // outlast a busy frequency working other traffic). Watching where we
            // are meanwhile: a route the tower plans for an aircraft that was
            // moved while it waited is not ours to drive.
            let mut got = false;
            let mut last = position(&xp).await.map(|p| (p, std::time::Instant::now()));
            for _ in 0..120 {
                if cancelled(gen) && end_cancelled(&xp, &voice, valve, "request").await {
                    return;
                }
                if stand_route_ready() {
                    got = true;
                    break;
                }
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
                if let Some(now) = position(&xp).await {
                    let t = std::time::Instant::now();
                    if let Some((from, t0)) = last {
                        if let Some(jump_m) = crate::autotaxi::relocation_jump_m(
                            from,
                            now,
                            t.duration_since(t0).as_secs_f64(),
                        ) {
                            crate::blackbox::record(
                                "taxi_in",
                                json!({ "event": "request_relocated", "jump_m": jump_m }),
                            );
                            stand_down_relocated(&voice, "request", false);
                            return;
                        }
                    }
                    last = Some((now, t));
                }
            }
            if !got {
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
        } else {
            if crew_owns_taxi_without_tower(crate::mode::atc_enabled()) {
                // No controller exists here: plan the same stand route the
                // tower would have published, but mark it as the crew's own.
                // Do not require `uncontrolled_arrival`: full auto publishes
                // it before approach, but a manual arrival has no such seam.
                // The apt.dat scan is blocking and stays off this runtime.
                let marked_uncontrolled = crate::party_line::uncontrolled_arrival();
                let (lat, lon, psi) = match (
                    xp.get_float(crate::autotaxi::LAT).await,
                    xp.get_float(crate::autotaxi::LON).await,
                    xp.get_float(PSI_TRUE).await,
                ) {
                    (Ok(lat), Ok(lon), Ok(psi)) => (lat, lon, Some(psi)),
                    (Ok(lat), Ok(lon), Err(_)) => (lat, lon, None),
                    _ => {
                        announce(&voice, Priority::Caution, "I can't place us on the field, so the taxi is yours.");
                        arrest_release();
                        return;
                    }
                };
                let apt = config.apt_data_path.clone();
                match tokio::task::spawn_blocking(move || {
                    crate::party_line::plan_self_taxi_to_stand(&apt, lat, lon, psi)
                })
                .await
                .ok()
                .and_then(Result::ok)
                {
                    Some(route) => {
                        announce(
                            &voice,
                            Priority::Advisory,
                            &format!("Uncontrolled field. Taxiing ourselves to {}.", route.stand),
                        );
                        crate::blackbox::record(
                            "taxi_in",
                            json!({ "event": "self_taxi", "field": route.field,
                                    "stand": route.stand, "via": route.via,
                                    "marked_uncontrolled": marked_uncontrolled }),
                        );
                    }
                    None => {
                        announce(&voice, Priority::Caution, "I can't find a route to parking, so the taxi is yours.");
                        arrest_release();
                        return;
                    }
                }
            } else {
                // No simulated tower: real/external ATC owns the clearance.
                announce(
                    &voice,
                    Priority::Advisory,
                    "Call the tower for your taxi-in, then say taxi to parking.",
                );
                arrest_release();
                return;
            }
            // The uncontrolled branch published a stand route; continue to
            // the ordinary drive below without a radio request.
            if !stand_route_ready() {
                arrest_release();
                return;
            }
        }
    }

    // Release the brakes autoland set at the stop ("down and stopped, brakes
    // on") — otherwise the taxi governor reads the residual parking-brake
    // pressure as the pilot braking and never applies power, and the aircraft
    // sits stagnant at the numbers (2026-07-26). The same "brakes off" full auto
    // does before a departure taxi, and valve-aware for the reason above: the
    // ratio-only write left the valve armed to re-trap the first squeeze.
    if cancelled(gen) && end_cancelled(&xp, &voice, valve, "cleared").await {
        return;
    }
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
        crate::autotaxi::halt();
        let held = stop_and_park(&xp, park_brake_valve_set).await;
        crate::blackbox::record("taxi_in", json!({ "event": "overrun_halt", "held": held }));
        // Give the loop its tick to see the flag and let go of the channels.
        tokio::time::sleep(std::time::Duration::from_millis(600)).await;
    }
    if cancelled(gen) && end_cancelled(&xp, &voice, valve, "before_drive").await {
        return;
    }
    crate::autotaxi::forget_arrival();
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
    if cancelled(gen) && end_cancelled(&xp, &voice, valve, "driving").await {
        return;
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
    use super::{
        crew_owns_taxi_without_tower, engines_stopped, shutdown_line, vacate_budget_secs,
        VACATE_MAX_SECS,
    };

    /// EGBS (2026-10-01, issue #212): the pilot flew the arrival, so full
    /// auto never marked it uncontrolled. Built-in ATC still owns the world
    /// and the crew must self-taxi; only external ATC keeps that with the
    /// pilot.
    #[test]
    fn a_manual_untowered_arrival_self_taxis_only_with_built_in_atc() {
        assert!(crew_owns_taxi_without_tower(true));
        assert!(!crew_owns_taxi_without_tower(false));
    }

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

#[cfg(test)]
mod relocation_tests {
    use super::{vacate_still_here, vacated_report_due};

    /// LEMG 31 (2026-09-28, issue #141): the rollout's stop, where the vacate
    /// was planned (state t 1790585617.7), and where the aircraft was 0.4 s
    /// later (t 1790585618.2) after the sortie runner reloaded it; the drive
    /// engaged 1,032.6 m off the vacate's line.
    const ROLLOUT: (f64, f64) = (36.67410120039185, -4.49797079757389);
    const RELOADED: (f64, f64) = (36.68532005, -4.49417);

    #[test]
    fn a_vacate_never_starts_from_where_the_aircraft_was_moved_to() {
        assert!(!vacate_still_here(ROLLOUT, RELOADED, 0.4, 1032.6));
        // A route far off with no jump seen is still no join (the tool's own
        // 150 m refusal, which the vacate's direct start skipped).
        assert!(!vacate_still_here(ROLLOUT, ROLLOUT, 0.4, 151.0));
        // Still rolling at the recorded 16 kt for the 0.4 s: here.
        assert!(vacate_still_here(ROLLOUT, (36.674127, -4.498005), 0.4, 1.0));
    }

    #[test]
    fn runway_vacated_is_said_only_off_the_strip() {
        assert!(vacated_report_due(true, Some(false)));
        assert!(
            vacated_report_due(true, None),
            "no runway data is no evidence"
        );
        assert!(
            !vacated_report_due(true, Some(true)),
            "an arrival still on a strip"
        );
        assert!(!vacated_report_due(false, Some(false)));
        assert!(!vacated_report_due(false, None));
    }
}

#[cfg(test)]
mod abort_tests {
    use super::*;
    use std::sync::atomic::Ordering::SeqCst;

    /// LEMG (2026-09-28, t 1790591992.8): "my controls" in the vacate stopped
    /// the drive and the taxi-in drove to the stand 7 s later. The release
    /// now ends the taxi-in itself, between its drives too, and holds.
    #[test]
    fn my_controls_ends_the_taxi_in_between_its_drives() {
        let _g = crate::mode::TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        RUN.cancel();
        CANCEL_HOLD.store(false, SeqCst);
        // Nothing running: nothing to end.
        abort(true);
        assert!(!CANCEL_HOLD.load(SeqCst));
        // Running (the request's wait: no auto taxi), the pilot's release.
        let gen = begin();
        assert!(running() && !cancelled(&gen));
        crate::autotaxi::ACTIVE.store(true, SeqCst);
        abort(true);
        assert!(cancelled(&gen), "the taxi-in ends");
        assert!(CANCEL_HOLD.load(SeqCst), "and stops and parks the aircraft");
        assert!(!crate::autotaxi::driving(), "its drive halted");
        assert!(!crate::tools::speedhold::ACTIVE.load(SeqCst), "its governor off");
        // A stand-down (swap, new flight) lets go without the hold.
        let gen = begin();
        abort(false);
        assert!(cancelled(&gen) && !CANCEL_HOLD.load(SeqCst));
        drop(gen);
        // A newer taxi-in supersedes an older one, and the older one's end
        // leaves the newer running.
        let old = begin();
        let newer = begin();
        assert!(cancelled(&old) && !cancelled(&newer));
        drop(old);
        assert!(running());
        drop(newer);
        assert!(!running());
    }
}
