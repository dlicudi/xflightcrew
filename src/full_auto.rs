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
    crate::procedure::abort();
    crate::autoland::abort();
    ACTIVE.swap(false, SeqCst)
}

/// What the flight IS once we're lined up: the proving-ground circuit, or the
/// pilot's actual SimBrief flight plan flown to its destination.
pub enum Mission {
    /// Takeoff, the pattern, land back at the departure runway.
    Circuit { right: bool },
    /// The SimBrief route: takeoff, the enroute fixes at the plan's altitudes,
    /// then the destination's published approach (autoland as the fallback).
    Flight(Route),
}

/// The flyable distillation of a SimBrief OFP, built by the arming tool.
pub struct Route {
    pub dest_icao: String,
    /// The airport's name from apt.dat ("Gibraltar"), for the announcements;
    /// falls back to the ICAO when the lookup failed.
    pub dest_name: String,
    /// Enroute legs (origin and destination airports already dropped), fixes
    /// resolved from the plan's own lat/lons.
    pub legs: Vec<crate::nav::ProcLeg>,
    /// The plugin FMS-mailbox payload ("IDENT,lat,lon,alt|...") for the whole
    /// plan, so the GPS shows the route too. Best-effort — the follower flies
    /// the plan's own fixes, not the FMS.
    pub fms_payload: String,
}

/// The tested sub-tools the orchestrator invokes, built from the profile by the
/// arming tool (power-up is stateless, constructed inline).
pub struct SubTools {
    /// The engine-start tool for THIS airframe — the piston `StartEngine` or, on
    /// a turboprop, the PT6 `StartTurbines` (condition levers + fire-once
    /// starter). Piston-cranking a turboprop never catches, so the Twin Otter's
    /// full-auto start failed until this became per-airframe (2026-07-26).
    pub start_engine: Box<dyn crate::tools::Tool>,
    /// Battery + DC-master commands to power the bus BEFORE the start — the
    /// turboprop start only checks volts, it doesn't switch the battery on, and
    /// the Twin Otter's battery is a custom command a stock write can't set.
    pub bus_power_on_cmds: Vec<String>,
    /// EVERY electrical system's ON command — fired AFTER the engine catches so
    /// the generators and bleed air (which feed the Twin Otter's autopilot) come
    /// on, not just the cranking bus.
    pub all_electrical_on_cmds: Vec<String>,
    pub taxi: crate::tools::taxi::AutoTaxi,
    pub line_up: crate::tools::taxi::LineUp,
    pub circuit: crate::tools::circuit::Circuit,
    /// The takeoff tool, armed directly on a Flight mission (a circuit brings
    /// its own takeoff inside the circuit orchestrator).
    pub takeoff: crate::tools::takeoff::AutoTakeoff,
    pub mission: Mission,
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
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
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

    announce(
        &voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Full auto. Starting from cold.",
            "Full auto it is. Taking her from cold and dark.",
            "Alright, full auto. Starting cold.",
        ]),
    );
    event("start", json!({}));
    // The callsign the tower knows us by, so our calls sound like a real pilot.
    let callsign = crate::circuit::crew_callsign(&xp).await;

    // --- Stage 1: electrics + engine ------------------------------------------
    if !engine_running(&xp).await {
        // Only turbine airframes need the APU/electrical bring-up; a piston's
        // start_engine powers its own bus.
        if tools.power_up_first {
            announce(
                &voice,
                Priority::Advisory,
                crate::readback::one_of(&[
                    "Powering up.",
                    "Getting some power on.",
                    "Bringing her to life.",
                ]),
            );
            let r = crate::tools::power::PowerUp.execute(&json!({}), &ctx).await;
            event(
                "power_up",
                json!({ "reply": r.as_ref().map(|s| s.as_str()).unwrap_or("err") }),
            );
            bail_if_abort!();
        }

        // Power the DC bus the way THIS airframe does it before cranking: the
        // turboprop start only checks volts, and the Twin Otter's battery is a
        // custom command the stock write can't set — so a cold start read 0 V
        // and never turned a blade (2026-07-26). No-op for pistons (empty).
        if !tools.bus_power_on_cmds.is_empty() {
            announce(&voice, Priority::Advisory, "Battery on.");
            for cmd in &tools.bus_power_on_cmds {
                let _ = xp.run_command(cmd).await;
            }
            tokio::time::sleep(std::time::Duration::from_millis(600)).await;
            event(
                "bus_power",
                json!({ "cmds": tools.bus_power_on_cmds.len() }),
            );
            bail_if_abort!();
        }

        announce(
            &voice,
            Priority::Advisory,
            crate::readback::one_of(&[
                "Starting the engine.",
                "Spinning up the engine.",
                "Cranking her over.",
            ]),
        );
        match tools.start_engine.execute(&json!({}), &ctx).await {
            Ok(reply) => announce(&voice, Priority::Advisory, &reply),
            Err(_) => {}
        }
        // The starter cranks for a few seconds before it catches.
        if !wait_until(40.0, 500, || engine_running(&xp)).await {
            give_up(
                &voice,
                "The engine didn't catch. Full auto is off, your aircraft.",
                "no_engine",
            );
            return;
        }
        event("engine_running", json!({}));

        // Complete the electrical now the engine's turning: generators AND
        // bleed air on. On the Twin Otter the bleeds feed the autopilot and
        // anti-ice, so a circuit flown without them has no servos. Skipped when
        // PowerUp already ran the bus (jets), and a no-op on airframes with no
        // electrical profile. Battery/master re-fire harmlessly.
        if !tools.power_up_first && !tools.all_electrical_on_cmds.is_empty() {
            for cmd in &tools.all_electrical_on_cmds {
                let _ = xp.run_command(cmd).await;
            }
            announce(
                &voice,
                Priority::Advisory,
                "Electrical set. Generators and bleeds on.",
            );
            event(
                "electrical_set",
                json!({ "cmds": tools.all_electrical_on_cmds.len() }),
            );
        }
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
                announce(
                    &voice,
                    Priority::Advisory,
                    crate::readback::one_of(&[
                        "Tuning the tower, requesting taxi.",
                        "Getting the tower, asking for taxi.",
                        "On the tower now, requesting taxi.",
                    ]),
                );
            } else {
                announce(
                    &voice,
                    Priority::Advisory,
                    crate::readback::one_of(&[
                        "Requesting taxi.",
                        "Asking for taxi.",
                        "Calling for taxi.",
                    ]),
                );
            }
            crate::circuit::transmit(crate::circuit::radio_call(&callsign, "request taxi"));
            event("request_taxi", json!({}));
            // The call queues behind our own intercom lines AND anyone the
            // tower is already working — the clock must outlast a worked
            // arrival. 25 s gave up ONE SECOND before a busy Jerez frequency
            // came back with the clearance (16:45Z, 2026-07-24).
            let got = wait_until(90.0, 1000, || async {
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
        announce(
            &voice,
            Priority::Advisory,
            crate::readback::one_of(&[
                "Taxiing to the runway.",
                "Rolling out to the runway.",
                "Taking us to the runway.",
            ]),
        );
        match tools.taxi.execute(&json!({}), &ctx).await {
            Ok(reply) => announce(&voice, Priority::Advisory, &reply),
            Err(_) => {}
        }
        // Wait for the drive to finish (it clears ACTIVE at the holding point).
        if crate::autotaxi::ACTIVE.load(SeqCst)
            && !wait_until(600.0, 1000, || async {
                !crate::autotaxi::ACTIVE.load(SeqCst)
            })
            .await
        {
            give_up(
                &voice,
                "The taxi didn't finish. Your aircraft.",
                "taxi_stalled",
            );
            return;
        }
        bail_if_abort!();
        // ACTIVE clearing is NOT proof of arrival: a pedal yield, an off-route
        // stop, or the governor evaporating all clear it too. A 30-second
        // stand-down (pedals) had full auto call "ready for departure" from
        // the PARKING STAND, wait out a traffic hold, and read back a takeoff
        // clearance 1.8 km from the runway (16:48Z, 2026-07-24). The drive's
        // own verdict is `LAST_ARRIVED` (the backtrack supervisor's signal);
        // when it says no, accept a position within 60 m of the route's end
        // instead (the pilot may have taxied us there by hand) — otherwise
        // stand down honestly.
        if !crate::autotaxi::LAST_ARRIVED.load(SeqCst) {
            let hold_end =
                crate::party_line::with_taxi_route(|r| r.and_then(|w| w.path.last().copied()));
            let near_hold = match hold_end {
                Some((hl, hn)) => {
                    let (Ok(lat), Ok(lon)) = (xp.get_float(LAT).await, xp.get_float(LON).await)
                    else {
                        give_up(
                            &voice,
                            "I can't read our position. Your aircraft.",
                            "no_position",
                        );
                        return;
                    };
                    let dist_m = crate::nav::haversine_nm(lat, lon, hl, hn) * 1852.0;
                    event("taxi_end_check", json!({ "dist_m": dist_m }));
                    dist_m <= 60.0
                }
                None => false,
            };
            if !near_hold {
                give_up(
                    &voice,
                    "The taxi ended before the holding point, so I'm standing down. \
                     Say fly the flight plan or full auto again and I'll take it from here.",
                    "taxi_incomplete",
                );
                return;
            }
        }
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

        // Onto the runway, CLEARANCE-AWARE, and VERIFIED. The line-up tool
        // rightly refuses to enter the runway on a bare taxi clearance — and
        // full_auto used to sail past that refusal string, wait out the
        // traffic hold at the hold-short, read back "cleared for takeoff"
        // from the WRONG SIDE of the line, and arm a circuit that refused:
        // stranded at the hold with a valid clearance (12:44Z, 2026-07-24).
        // Now: try the line-up when the tower's state allows entering
        // (line-up-and-wait or cleared; no tower = pilot's judgment); when
        // the tower is HOLDING us, wait the clearance out FIRST, then drive
        // on; and never proceed to the circuit without the runway_fit check
        // actually confirming we're on the strip.
        use crate::tower::State::{LinedUp, TakeoffCleared};
        let can_enter = || {
            matches!(
                crate::party_line::clearance_state(),
                None | Some(LinedUp) | Some(TakeoffCleared)
            )
        };
        let mut read_back = false;
        for attempt in 0..2 {
            if atc_running && !can_enter() {
                // Held short (traffic, wake): the clearance must come first.
                if !ensure_takeoff_cleared(&voice, &callsign, &mut read_back).await {
                    give_up(
                        &voice,
                        "The tower never cleared us for takeoff. Your aircraft. \
                         Say full auto again once we're cleared.",
                        "no_takeoff_clearance",
                    );
                    return;
                }
                bail_if_abort!();
            }
            announce(
                &voice,
                Priority::Advisory,
                crate::readback::one_of(&["Lining up.", "Lining us up.", "Taking the runway."]),
            );
            match tools.line_up.execute(&json!({}), &ctx).await {
                Ok(reply) => announce(&voice, Priority::Advisory, &reply),
                Err(_) => {}
            }
            // A full-length backtrack is a long, slow drive (LXGB's 09 backtrack
            // is ~1 km) — the 120 s that fit a plain line-up timed out mid-taxi
            // (2026-07-26). Give it the taxi's budget.
            if crate::autotaxi::ACTIVE.load(SeqCst)
                && !wait_until(600.0, 1000, || async {
                    !crate::autotaxi::ACTIVE.load(SeqCst)
                })
                .await
            {
                give_up(
                    &voice,
                    "The line-up didn't finish. Your aircraft.",
                    "line_up_stalled",
                );
                return;
            }
            bail_if_abort!();
            if lined_up(&xp, &tools, &config).await {
                break;
            }
            if attempt == 1 {
                give_up(
                    &voice,
                    "I couldn't get us onto the runway. Your aircraft.",
                    "line_up_failed",
                );
                return;
            }
        }
        event("lined_up", json!({}));

        // The takeoff clearance, if the line-up went on under line-up-and-wait
        // (already satisfied when the held-short path above got it first).
        if atc_running && !ensure_takeoff_cleared(&voice, &callsign, &mut read_back).await {
            give_up(
                &voice,
                "We're lined up and ready, but the tower never cleared us for takeoff. \
                 Your aircraft. Say full auto again once we're cleared.",
                "no_takeoff_clearance",
            );
            return;
        }
        bail_if_abort!();
    }

    // --- Stage 4: the mission — a circuit, or the SimBrief flight -------------
    match &tools.mission {
        Mission::Circuit { right } => {
            announce(
                &voice,
                Priority::Advisory,
                crate::readback::one_of(&[
                    "Lined up. Flying the circuit.",
                    "Lined up and ready. Flying the circuit.",
                    "On the runway. Off we go, flying the circuit.",
                ]),
            );
            let args = json!({ "right": right });
            match tools.circuit.execute(&args, &ctx).await {
                Ok(reply) => announce(&voice, Priority::Advisory, &reply),
                Err(_) => {}
            }
            event("circuit_armed", json!({}));
            // The circuit owns the aircraft from here; our job is done.
            ACTIVE.store(false, SeqCst);
        }
        Mission::Flight(route) => {
            fly_route(&xp, &config, &voice, &ctx, &tools, route, &callsign).await;
        }
    }
}

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const ELEV_M: &str = "sim/flightmodel/position/elevation";
const M_TO_FT: f64 = 3.280_84;
const MS_TO_KTS: f64 = 1.943_844;
/// Below this surface wind the arrival picks its runway by geometry (the end
/// most aligned with our bearing to the field), not wind: a light or variable
/// wind isn't worth a big turn onto final. At or above it the wind decides —
/// landing INTO it — because a real tailwind is what put the arrival on
/// Gibraltar's 09 into 17 kt while the tower worked 27 (2026-07-24).
const WIND_DECISIVE_KTS: f64 = 4.0;

/// An airborne AGL below this after the takeoff loop exits means it REJECTED
/// (never flew) — same reading the circuit uses.
const AIRBORNE_MIN_AGL_FT: f64 = 300.0;

/// Stage 4 of a Flight mission: the takeoff, the enroute fixes at the plan's
/// altitudes, then the destination's published approach with autoland as the
/// fallback — every phase one of the proven loops, watched in sequence.
async fn fly_route(
    xp: &XPlaneClient,
    config: &Config,
    voice: &Option<Voice>,
    ctx: &ToolCtx<'_>,
    tools: &SubTools,
    route: &Route,
    callsign: &Option<String>,
) {
    use std::sync::atomic::Ordering::Relaxed;

    // Show the route on the GPS too (the plugin's FMS mailbox, the same path
    // "load the flight plan" takes). Best-effort: only the concrete client can
    // write a byte-array dataref, which is why this lives here and not in the
    // arming tool.
    if !route.fms_payload.is_empty() {
        let _ = xp
            .set_data("xcopilot/fms/route", route.fms_payload.as_bytes())
            .await;
        let _ = xp.set_int("xcopilot/fms/load", 1).await;
    }

    // --- Takeoff: the proven loop, armed through its tool so every arm-time
    // gate applies (the pitch-channel probe included).
    announce(
        voice,
        Priority::Advisory,
        &format!(
            "Cleared to go. Flying your plan to {}. My takeoff.",
            route.dest_name
        ),
    );
    match tools.takeoff.execute(&json!({}), ctx).await {
        Ok(reply) => announce(voice, Priority::Advisory, &reply),
        Err(_) => {}
    }
    if !crate::autotakeoff::ACTIVE.load(Relaxed) {
        give_up(
            voice,
            "The takeoff didn't arm, so the flight is off. Your aircraft.",
            "takeoff_refused",
        );
        return;
    }
    if !wait_until(360.0, 1000, || async {
        !crate::autotakeoff::ACTIVE.load(Relaxed)
    })
    .await
    {
        if !ABORT.load(SeqCst) {
            give_up(
                voice,
                "The takeoff never finished. Your aircraft.",
                "takeoff_stalled",
            );
        } else {
            give_up(voice, "Cancelled. Your aircraft.", "abort");
        }
        return;
    }
    let agl_ft = xp.get_float(AGL_M).await.unwrap_or(0.0) * M_TO_FT;
    if agl_ft < AIRBORNE_MIN_AGL_FT {
        give_up(
            voice,
            "The takeoff didn't get us airborne, so the flight is off.",
            "takeoff_rejected",
        );
        return;
    }
    event("airborne", json!({ "agl_ft": agl_ft }));

    // --- Enroute: the procedure follower over the plan's own fixes. It climbs
    // to cruise on the first constraint and flies the plan's descent numbers
    // after TOD; quiet completion (EndAction::Enroute) hands back here.
    crate::procedure::ABORT.store(false, SeqCst);
    crate::procedure::start(
        config.xplane_base_url.clone(),
        voice.clone(),
        crate::procedure::Plan {
            name: format!("the flight plan to {}", route.dest_name),
            legs: route.legs.clone(),
            end: crate::procedure::EndAction::Enroute,
            // The climb to cruise runs on the speed-protected law — the
            // unprotected rate stalled the first LEJR climb at 53 knots.
            vy_kts: tools.takeoff.vy_kts,
        },
    );
    // Generous ceiling — ABORT bails out of the wait immediately.
    if !wait_until(4.0 * 3600.0, 2000, || async {
        !crate::procedure::ACTIVE.load(Relaxed)
    })
    .await
    {
        if !ABORT.load(SeqCst) {
            give_up(
                voice,
                "The enroute leg never completed. Your aircraft.",
                "enroute_stalled",
            );
        } else {
            give_up(voice, "Cancelled. Your aircraft.", "abort");
        }
        return;
    }
    if ABORT.load(SeqCst) {
        give_up(voice, "Cancelled. Your aircraft.", "abort");
        return;
    }
    event("enroute_complete", json!({}));

    // --- Arrival: prefer the destination's PUBLISHED approach (the terrain-
    // safe corridor, the circuit's arrival rule); a visual autoland otherwise.
    let (Ok(lat), Ok(lon)) = (xp.get_float(LAT).await, xp.get_float(LON).await) else {
        give_up(
            voice,
            "I can't read our position for the arrival. Your aircraft.",
            "no_position",
        );
        return;
    };
    let alt_msl_ft = xp.get_float(ELEV_M).await.unwrap_or(0.0) * M_TO_FT;
    // The surface wind near the field (the aircraft-wind datarefs, same source
    // ATIS and the tower read) chooses the landing runway — see build_arrival.
    let wind = match (
        xp.get_float(crate::datarefs::WIND_DIR_DEG_TRUE).await,
        xp.get_float(crate::datarefs::WIND_SPEED_MSC).await,
    ) {
        (Ok(dir), Ok(spd_ms)) => Some((dir, spd_ms * MS_TO_KTS)),
        _ => None,
    };
    let built = {
        let (apt_dat, cifp, fixp, navp) = (
            crate::tools::autoland::apt_dat_path(),
            config.cifp_dir.clone(),
            config.fix_data_path.clone(),
            config.nav_data_path.clone(),
        );
        let dest = route.dest_icao.clone();
        let cfg = tools.circuit.cfg.clone();
        let ias = tools.circuit.approach_ias;
        tokio::task::spawn_blocking(move || {
            build_arrival(
                &apt_dat, &cifp, &fixp, &navp, &dest, lat, lon, cfg, ias, wind,
            )
        })
        .await
        .unwrap_or(Arrival {
            plan: None,
            rwy_id: None,
            tower_khz: None,
        })
    };
    // The radio: tune the DESTINATION'S tower from its own apt.dat comm row —
    // never `home_tower_khz`, which is whoever the enroute region last handed
    // us to. At LINTO that was Tangier (genuinely the nearest field,
    // mid-strait), and the inbound call earned "join final runway two eight"
    // from Ibn Battouta Tower while we set up for Gibraltar's zero nine
    // (17:32Z, 2026-07-24). The region hands the STATION over as we close;
    // with COM1 already on the destination's frequency the handoff finds us
    // listening.
    if let Some(khz) = built.tower_khz.or_else(crate::party_line::home_tower_khz) {
        let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
        crate::circuit::transmit(crate::circuit::radio_call(callsign, "inbound for landing"));
        event("inbound_call", json!({ "khz": khz }));
    }
    // The wind-chosen end (build_arrival) drives BOTH the published-approach
    // branch and the visual fallback — the visual autoland is forced onto the
    // same runway so a tailwind can't be re-picked back in by track.
    let landing_rwy = built.rwy_id.clone();
    match built.plan {
        Some(plan) if crate::procedure::approach_flyable_from(&plan, lat, lon, alt_msl_ft) => {
            event(
                "arrival_procedure",
                json!({ "name": plan.name, "runway": landing_rwy }),
            );
            announce(
                voice,
                Priority::Advisory,
                &format!("Setting up {}.", plan.name),
            );
            crate::procedure::ABORT.store(false, SeqCst);
            crate::procedure::start(config.xplane_base_url.clone(), voice.clone(), plan);
        }
        _ => {
            event("arrival_autoland", json!({ "runway": landing_rwy }));
            announce(
                voice,
                Priority::Advisory,
                "No published approach lines up from here, so I'll land us visually.",
            );
            crate::autoland::ABORT.store(false, SeqCst);
            crate::autoland::run_autoland(
                config.xplane_base_url.clone(),
                crate::tools::autoland::apt_dat_path(),
                tools.circuit.cfg.clone(),
                tools.circuit.approach_ias,
                voice.clone(),
                None,
                landing_rwy.clone(),
            );
        }
    };

    // --- The final report is what earns the landing clearance: call it the
    // moment autoland takes the approach (the FAF handoff, or straight away on
    // the visual). Nothing left to actuate here — the arrival loops own the
    // aircraft — so this watch is the orchestrator's last duty.
    let called = wait_until(1800.0, 1000, || async {
        crate::autoland::ACTIVE.load(Relaxed) || !crate::procedure::ACTIVE.load(Relaxed)
    })
    .await;
    // A procedure→autoland handoff has a brief both-inactive window while the
    // autoland thread spins up; give it a beat before concluding nobody flies.
    if called && !crate::autoland::ACTIVE.load(Relaxed) {
        tokio::time::sleep(Duration::from_secs(3)).await;
    }
    if crate::autoland::ACTIVE.load(Relaxed) && !ABORT.load(SeqCst) {
        let call = match &landing_rwy {
            Some(rwy) => format!("final runway {}", crate::circuit::spell_runway(rwy)),
            None => "final".to_string(),
        };
        crate::circuit::transmit(crate::circuit::radio_call(callsign, &call));
        event("final_call", json!({ "runway": landing_rwy }));
    }

    // Gate to gate: once autoland is down and stopped (it goes inactive after
    // "down and stopped, your aircraft"), taxi to PARKING on our own — request
    // the taxi-in and drive to the stand, the arrival mirror of the departure
    // taxi we ran at the start. Bounded so a balk/abort can't hang here, and
    // skipped unless we actually got the aircraft down on the ground (autoland
    // also goes inactive on a go-around, climbing away).
    if crate::autoland::ACTIVE.load(Relaxed) && !ABORT.load(SeqCst) {
        let landed = wait_until(1800.0, 1000, || async {
            !crate::autoland::ACTIVE.load(Relaxed)
        })
        .await;
        let on_ground = xp
            .get_float(crate::datarefs::ON_GROUND_ANY)
            .await
            .unwrap_or(0.0)
            > 0.5;
        if landed && on_ground && !ABORT.load(SeqCst) {
            announce(
                voice,
                Priority::Advisory,
                crate::readback::one_of(&[
                    "Down. I'll taxi us to parking.",
                    "Down and clear. Taxiing us to parking.",
                    "On the ground. I'll get us to the gate.",
                ]),
            );
            event("taxi_in", json!({}));
            crate::taxi_in::start(
                config.xplane_base_url.clone(),
                config.clone(),
                voice.clone(),
                tools.taxi.park_brake_valve_set,
                tools.taxi.prop.clone(),
            );
        }
    }
    event("arrival_owns", json!({}));
    ACTIVE.store(false, SeqCst);
}

/// What the blocking arrival builder found: the approach plan + runway when
/// one could be built, and the destination's own tower frequency either way.
struct Arrival {
    /// The published/reconstructed approach for `rwy_id`, when one is flyable.
    plan: Option<crate::procedure::Plan>,
    /// The chosen landing runway — set whenever the field was found, whether or
    /// not a plan came with it (the visual fallback and the final call need it).
    rwy_id: Option<String>,
    tower_khz: Option<i64>,
}

/// Blocking arrival builder: destination airport + the landing runway, then the
/// published-approach plan for it. The runway is the end that best faces INTO
/// the surface wind when the wind is decisive (`WIND_DECISIVE_KTS`); in a light
/// or variable wind it's the end whose landing direction best matches our
/// bearing to the field (no big turn onto final for a wind that doesn't matter).
/// The tower frequency comes back even when no plan does — the radio still needs
/// the right station for the visual fallback. `wind` is (from_deg true, kts).
#[allow(clippy::too_many_arguments)]
fn build_arrival(
    apt_dat: &str,
    cifp: &str,
    fixp: &str,
    navp: &str,
    dest: &str,
    lat: f64,
    lon: f64,
    cfg: crate::tools::autopilot::ApConfig,
    approach_ias: f64,
    wind: Option<(f64, f64)>,
) -> Arrival {
    let Some(airport) = crate::nav::find_airport(apt_dat, dest).ok().flatten() else {
        return Arrival {
            plan: None,
            rwy_id: None,
            tower_khz: None,
        };
    };
    let tower_khz = airport.freq(crate::nav::FREQ_TOWER).map(|f| f.khz);
    // Into the wind when it's strong enough to matter...
    let by_wind = wind
        .filter(|(_, kts)| *kts >= WIND_DECISIVE_KTS)
        .and_then(|(dir, kts)| crate::nav::best_runway_for_wind(&airport.runways, dir, kts))
        .map(|(id, _, _)| id);
    // ...otherwise the end most aligned with our arrival bearing.
    let rwy_id = by_wind.or_else(|| {
        let diff = |a: f64, b: f64| ((a - b + 540.0).rem_euclid(360.0) - 180.0).abs();
        let (mut best_id, mut best_score) = (None::<String>, f64::MAX);
        for r in &airport.runways {
            let hdg_a = crate::nav::bearing_deg(r.lat_a, r.lon_a, r.lat_b, r.lon_b);
            let hdg_b = (hdg_a + 180.0).rem_euclid(360.0);
            for (id, thr_lat, thr_lon, hdg) in [
                (&r.ident_a, r.lat_a, r.lon_a, hdg_a),
                (&r.ident_b, r.lat_b, r.lon_b, hdg_b),
            ] {
                let arrival_brg = crate::nav::bearing_deg(lat, lon, thr_lat, thr_lon);
                let score = diff(arrival_brg, hdg);
                if score < best_score {
                    best_score = score;
                    best_id = Some(id.clone());
                }
            }
        }
        best_id
    });
    let Some(rwy_id) = rwy_id else {
        return Arrival {
            plan: None,
            rwy_id: None,
            tower_khz,
        };
    };
    let end = crate::procedure::EndAction::Approach {
        apt_dat: apt_dat.to_string(),
        cfg,
        approach_ias,
    };
    let plan = crate::procedure::approach_plan_for(
        cifp, fixp, navp, &airport, dest, &rwy_id, lat, lon, end,
    )
    .ok();
    Arrival {
        plan,
        rwy_id: Some(rwy_id),
        tower_khz,
    }
}

/// Call the tower for departure and wait for it to move us to a departure
/// clearance (line-up-and-wait or cleared for takeoff). The call itself is
/// voiced + transcribed by the crew-radio drain, along with the tower's reply.
async fn request_departure(voice: &Option<Voice>, callsign: &Option<String>) {
    use crate::tower::State::{LinedUp, TakeoffCleared};
    announce(
        voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Calling for departure.",
            "Calling the tower for departure.",
            "Getting our departure clearance.",
        ]),
    );
    crate::circuit::transmit(crate::circuit::radio_call(callsign, "ready for departure"));
    event("request_departure", json!({}));
    wait_until(20.0, 1000, || async {
        matches!(
            crate::party_line::clearance_state(),
            Some(LinedUp) | Some(TakeoffCleared)
        )
    })
    .await;
}

/// Wait until the tower actually clears the takeoff, through a traffic hold if
/// that's what it takes (a final can be minutes out). Re-report "ready for
/// departure" periodically while merely waiting, but stay SILENT while
/// explicitly held for traffic: the tower said hold, and nagging a held
/// clearance is wrong. Reads the clearance back ONCE across calls
/// (`read_back`); the tower enforces the runway-critical readback. Returns
/// whether we ended up cleared; abortable throughout.
async fn ensure_takeoff_cleared(
    voice: &Option<Voice>,
    callsign: &Option<String>,
    read_back: &mut bool,
) -> bool {
    use crate::tower::State::TakeoffCleared;
    /// Wait this long overall for the clearance — long enough to sit out a
    /// traffic hold, bounded so a dead frequency eventually hands back.
    const OVERALL_SECS: f64 = 480.0;
    /// Re-report "ready" this often while lined up and NOT held for traffic.
    const NUDGE_EVERY: Duration = Duration::from_secs(40);

    let deadline = Instant::now() + Duration::from_secs_f64(OVERALL_SECS);
    let mut last_call = Instant::now(); // request_departure just called
    let mut announced_hold = false;
    loop {
        if ABORT.load(SeqCst) {
            return false;
        }
        if matches!(crate::party_line::clearance_state(), Some(TakeoffCleared)) {
            break;
        }
        if Instant::now() >= deadline {
            return false;
        }
        if crate::party_line::takeoff_hold_armed() {
            // Held for traffic: wait it out, silently. Say so once.
            if !announced_hold {
                announce(
                    voice,
                    Priority::Advisory,
                    crate::readback::one_of(&[
                        "Holding, the tower has traffic. I'll wait.",
                        "Tower's got traffic. Holding here.",
                        "We've got traffic ahead. Holding for now.",
                    ]),
                );
                announced_hold = true;
            }
        } else if last_call.elapsed() >= NUDGE_EVERY {
            crate::circuit::transmit(crate::circuit::radio_call(callsign, "ready for departure"));
            last_call = Instant::now();
        }
        tokio::time::sleep(Duration::from_millis(1000)).await;
    }
    // Cleared — read it back, once across the whole departure sequence.
    if !*read_back {
        if let Some(rwy) = crate::party_line::clearance_runway() {
            *read_back = true;
            crate::circuit::transmit(crate::circuit::radio_call(
                callsign,
                &format!(
                    "cleared for takeoff runway {}",
                    crate::circuit::spell_runway(&rwy)
                ),
            ));
            event("readback_takeoff", json!({ "runway": rwy }));
        }
    }
    true
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
    let needed_m = tools
        .line_up
        .takeoff_roll_m
        .map_or(500.0, |m| (m * 1.4).max(500.0));
    let apt = crate::tools::autoland::apt_dat_path();
    let cands = crate::autoland::nearby_runways(&apt, lat, lon, 3.0).unwrap_or_default();
    crate::autotakeoff::runway_fit(&cands, lat, lon, psi, needed_m).is_some()
}
