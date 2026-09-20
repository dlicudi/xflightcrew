//! Full auto — cold-and-dark to a flown circuit, one command.
//!
//! An ORCHESTRATOR, not new flight logic: it drives the aircraft through the
//! whole gate-to-gate sequence by invoking the SAME tested tools the pilot
//! would, in order, waiting on each before the next:
//!
//!   power up -> start engine -> altimeter -> REQUEST TAXI from the tower
//!   -> (pushback if nose-in) -> auto-taxi to the hold -> line up -> fly the
//!   circuit (takeoff, pattern, land, stop).
//!
//! No blanket brake release anywhere in the chain: the brakes stay as the
//! pilot left them until something actually drives — the taxi governor,
//! pushback and auto-takeoff each release at their own moment (the stage-2
//! release left an idle jet brakeless during the tug wait and it crept into
//! the grass, Luton 2026-07-28).
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
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
/// Throttle a mission give-up hands the aircraft back with — autoland's own
/// number, for the same reason: enough to hold level flight dirty.
const GIVE_UP_THROTTLE: f64 = 0.75;
const COM1_FREQ_KHZ: &str = crate::datarefs::COM1_FREQ_KHZ;

/// True while the full-auto orchestrator runs.
static RUN: sim::run::Slot = sim::run::Slot::new();
pub fn running() -> bool {
    RUN.running()
}

pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// Set to cancel ("cancel" / "my controls").
pub static ABORT: AtomicBool = AtomicBool::new(false);
/// The most recent stage `event()` recorded ("request taxi", "lined up"),
/// underscores spaced for reading. The panel's Flight tab shows it beside the
/// FULL AUTO headline; empty between runs.
static STAGE: std::sync::Mutex<String> = std::sync::Mutex::new(String::new());
/// The mission label ("circuit" / "flight"), set at start so the status names
/// WHICH full auto is running — the pilot pressed CIRCUIT, so the headline
/// must say circuit, not just the orchestrator's name. Empty between runs.
static MISSION: std::sync::Mutex<&'static str> = std::sync::Mutex::new("");

/// The human-readable current stage, empty when idle or not yet staged.
pub fn stage() -> String {
    STAGE.lock().map(|s| s.clone()).unwrap_or_default()
}

fn clear_stage() {
    if let Ok(mut s) = STAGE.lock() {
        s.clear();
    }
    WAIT.clear();
    if let Ok(mut m) = MISSION.lock() {
        *m = "";
    }
}

/// One-line status for the dashboard, mission and stage folded in once known:
/// "full auto circuit: request taxi" (the panel renders it as
/// "FULL AUTO CIRCUIT [request taxi]").
pub fn status() -> Option<String> {
    ACTIVE.load(SeqCst).then(|| {
        let mission = MISSION.lock().map(|m| *m).unwrap_or("");
        let name = if mission.is_empty() {
            "full auto".to_string()
        } else {
            format!("full auto {mission}")
        };
        let stage = stage();
        if stage.is_empty() {
            name
        } else {
            format!("{name}: {stage}")
        }
    })
}

/// Cancel the sequence and stand down every sub-loop it may have armed.
pub fn abort() -> bool {
    RUN.cancel();
    ABORT.store(true, SeqCst);
    clear_stage();
    crate::autotaxi::ACTIVE.store(false, SeqCst);
    crate::autotakeoff::cancel();
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
    /// Explicit checked airborne continuation; bypass every ground stage.
    Resume(Route),
}

/// The flyable distillation of a SimBrief OFP, built by the arming tool.
pub struct Route {
    /// A local route was terrain-checked for this final; changed wind needs replanning.
    pub local_runway: Option<String>,
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
    /// Toggle-only electrical switches: read, then fired only if off.
    pub all_electrical_toggles: Vec<crate::aircraft::ToggleSwitch>,
    /// The airframe's exterior lights, driven through the SAME tool a spoken
    /// "beacon on" reaches — one owner for the switch, so a light that is two
    /// switches or a dataref rather than a command behaves identically here.
    pub lights: crate::tools::lights::SetLights,
    /// Panel / cockpit flood lighting. Exterior lights are for other people to
    /// see us; this one is so the PILOT can read the panel — full auto brought
    /// a Baron to life and left the cockpit dark (2026-08-01).
    pub flood: crate::tools::lights::SetFloodLights,
    pub taxi: crate::tools::taxi::AutoTaxi,
    pub line_up: crate::tools::taxi::LineUp,
    pub circuit: crate::tools::circuit::Circuit,
    /// The takeoff tool, armed directly on a Flight mission (a circuit brings
    /// its own takeoff inside the circuit orchestrator).
    pub takeoff: crate::tools::takeoff::AutoTakeoff,
    /// Indicated cruise speed for the enroute cruise-power hold (profile's
    /// `reference.cruise_ias_kts`); `None` leaves cruise at climb power.
    pub cruise_ias_kts: Option<f64>,
    /// The airframe's transition altitude: the altimeter is set to QNH below
    /// it and standard above, at departure and at the enroute follower's arm.
    pub transition_altitude_ft: f64,
    /// The airframe has a continuous piston mixture lever (the profile's
    /// `engine` group): full auto arms the mixture manager for the enroute
    /// leg (full rich below the leaning floor, leaned to peak EGT above it,
    /// re-trimmed as altitude and power move) and sets full rich for the
    /// arrival. A turbine, or a notched xlua lever (the DC-3), gets neither:
    /// the mixture stayed wherever the start left it for the whole flight
    /// (2026-09-12).
    pub piston_mixture: bool,
    pub mission: Mission,
    /// Run the electrical/APU power-up before the engine start. Only turbine
    /// True when the airframe declares no start sequence we can fly at all.
    pub no_start_sequence: bool,
    /// airframes need it — a piston's `start_engine` brings its own battery up,
    /// and running PowerUp on a 172 had the copilot narrating an APU it doesn't
    /// have ("APU not coming up", 2026-07-23).
    pub power_up_first: bool,
    /// How long to wait for the engine to be running after the start is fired
    /// (`StartKind::engine_start_secs`). Per airframe CLASS: a jet's start is a
    /// three-minute procedure, a piston's is seconds.
    pub engine_start_secs: f64,
}

/// Spawn the orchestrator on its own thread + runtime + client.
pub fn start(base_url: String, config: Config, voice: Option<Voice>, tools: SubTools) -> bool {
    if crate::tools::recover::ACTIVE.load(SeqCst)
        || crate::procedure::running()
        || crate::autotakeoff::running()
        || crate::circuit::ACTIVE.load(SeqCst)
        || crate::autoland::ACTIVE.load(SeqCst)
    {
        return false;
    }
    let Some(lease) = RUN.begin() else {
        return false;
    };
    if ACTIVE.swap(true, SeqCst) {
        return false;
    }
    ABORT.store(false, SeqCst);
    INBOUND_MADE.store(false, SeqCst);
    if let Ok(mut m) = MISSION.lock() {
        *m = match tools.mission {
            Mission::Circuit { .. } => "circuit",
            Mission::Flight(_) | Mission::Resume(_) => "flight",
        };
    }
    std::thread::spawn(move || {
        let _lease = lease.with_cleanup(&ACTIVE, clear_stage);
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            ACTIVE.store(false, SeqCst);
            return;
        };
        rt.block_on(fly(
            XPlaneClient::new(base_url).with_cancel(_lease.cancel.clone()),
            config,
            voice,
            tools,
        ));
        ACTIVE.store(false, SeqCst);
        clear_stage();
    });
    true
}

fn announce(voice: &Option<Voice>, priority: Priority, line: &str) {
    tracing::info!(target: "xvoicelayer::full_auto", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

/// What the current stage is waiting on ("the squawk", "the landing
/// traffic to pass"), for the panel; cleared by the next stage event.
static WAIT: crate::auto_status::Detail = crate::auto_status::Detail::new();

fn waiting(what: &str) {
    WAIT.set(what);
}

/// The panel line under the stage: "waiting for the squawk", or `None`.
pub fn wait_status() -> Option<String> {
    if !ACTIVE.load(SeqCst) {
        return None;
    }
    let w = WAIT.get();
    (!w.is_empty()).then(|| format!("waiting for {w}"))
}

pub(crate) fn event(stage: &str, detail: serde_json::Value) {
    if let Ok(mut s) = STAGE.lock() {
        *s = stage.replace('_', " ");
    }
    WAIT.clear();
    crate::blackbox::record("full_auto", json!({ "stage": stage, "detail": detail }));
}

/// Set one exterior light, best effort and silent.
///
/// Full auto turned on NO lights at all: it powered the bus, started the
/// engines, taxied, lined up and took off dark, while the ARRIVAL explicitly
/// switched strobes, taxi and landing lights off at the stand — an asymmetry
/// that is the whole proof it was an oversight rather than a decision
/// (2026-08-01). A profile that doesn't declare the key simply errors and is
/// ignored, so an airframe with no beacon is not a failed departure.
async fn light(tools: &SubTools, ctx: &ToolCtx<'_>, key: &str, on: bool) -> bool {
    use crate::tools::Tool;
    // Declared first: the tool answers an unknown light with an Ok QUESTION
    // ("I didn't catch which light..."), which is right for a misheard pilot
    // and a lie under `.is_ok()` — the SR22, SF50 and Lancair have no beacon,
    // and full auto announced "Beacon on" and blackboxed switches it never
    // touched on all three (audit, 2026-08-02).
    if !tools.lights.declares(key) {
        return false;
    }
    tools
        .lights
        .execute(&json!({ "light": key, "on": on }), ctx)
        .await
        .is_ok()
}

/// Give up cleanly.
fn give_up(voice: &Option<Voice>, line: &str, reason: &str) {
    RUN.fail(reason);
    ACTIVE.store(false, SeqCst);
    // "Your aircraft" means every lever: the mixture manager stands down with
    // the rest, never quietly re-trimming under the pilot's hand.
    crate::tools::engine::MIX_MANAGED.store(false, SeqCst);
    event("give_up", json!({ "reason": reason }));
    clear_stage();
    announce(voice, Priority::Caution, line);
}

/// The squawk still to dial: the tower's assigned code when the transponder
/// does not already show it. `None` when nothing was assigned or it is set.
/// An unreadable transponder is owed the code too: the tool reads back.
pub fn squawk_owed(assigned: Option<i64>, actual: Option<f64>) -> Option<i64> {
    let code = assigned?;
    match actual {
        Some(a) if a.round() as i64 == code => None,
        _ => Some(code),
    }
}

/// Dial the assigned squawk if the transponder does not show it yet, and say
/// so. Called after the clearance wait AND at the later stages, because the
/// tower's answer can arrive after the wait gave up, and a squawk assigned
/// is a squawk owed until the wheels leave.
async fn ensure_squawk(xp: &XPlaneClient, ctx: &ToolCtx<'_>, voice: &Option<Voice>) {
    let actual = xp.get_float(crate::datarefs::TRANSPONDER_CODE).await.ok();
    let Some(code) = squawk_owed(crate::party_line::assigned_squawk(), actual) else {
        return;
    };
    let ok = crate::tools::radio::SetTransponder
        .execute(&json!({ "code": code }), ctx)
        .await
        .is_ok();
    event(
        "squawk_set",
        json!({ "code": code, "ok": ok, "was": actual }),
    );
    if ok {
        announce(
            voice,
            Priority::Advisory,
            &format!(
                "Squawk {}, set.",
                crate::atis::spoken_digits(&format!("{code:04}"))
            ),
        );
    }
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
/// How long a conditional line-up waits for the named arrival to pass before
/// full auto gives the aircraft back at the hold line.
const LINE_UP_BEHIND_SECS: f64 = 300.0;
/// How long the inbound call waits for the destination's station to take
/// over the frequency before going out regardless.
const INBOUND_SWAP_WAIT_SECS: f64 = 45.0;
/// Between the handoff readback and the retune: the readback has to be off
/// the air on the frequency it was given on before the radio leaves it.
const HANDOFF_READBACK_SECS: u64 = 4;
/// The inbound call goes out this far from the destination, once the region
/// has handed us to its own tower: twice the tower's 5 nm zone, so the crew
/// has called before the zone watch can ask who we are. Until 2026-09-15 the
/// call waited for the last enroute fix, and a plan ending at GRA (a mile
/// from Granada's runway) flew the whole zone unannounced: three "say
/// callsign and intentions" in four minutes, none answered.
const INBOUND_CALL_NM: f64 = 10.0;
/// The inbound call has been made this mission (by the enroute leg or the
/// arrival), so no later stage makes it twice. Reset at the arm.
static INBOUND_MADE: AtomicBool = AtomicBool::new(false);

/// Is the inbound call owed now? Only on the DESTINATION's tower frequency
/// (`home` is whoever the region last handed us to; at LINTO that was
/// Tangier, mid-strait), inside [`INBOUND_CALL_NM`], and not already made. A
/// zone challenge from that tower asks for it regardless of the distance.
pub fn inbound_due(
    dest_khz: Option<i64>,
    home_khz: Option<i64>,
    dist_nm: f64,
    challenged: bool,
    made: bool,
) -> bool {
    if made {
        return false;
    }
    match (dest_khz, home_khz) {
        (Some(d), Some(h)) if d == h => challenged || dist_nm <= INBOUND_CALL_NM,
        _ => false,
    }
}
/// Clear of the pattern: the landing AND taxi lights go off passing this
/// height (the taxi light was lit for the taxi and nothing else puts it out).
const LANDING_LIGHTS_OFF_AGL_FT: f64 = 1000.0;

/// The enroute wait, working the radio. The tower's after-takeoff handoff
/// ("contact Malaga Departure on ...") is read back on the frequency it came
/// on, COM1 is retuned, and the departure is checked in with altitude — the
/// exchange nobody made at Malaga, where the flight stayed on tower for
/// thirteen minutes and heard the party line all the way to the region swap
/// (12:23Z, 2026-09-11). The inbound call is made HERE when the region hands
/// us to the destination's own tower and the field is inside
/// [`INBOUND_CALL_NM`], or the moment that tower challenges us (`dest`: the
/// destination's position and tower frequency; `None` for a field with no
/// tower row, where nobody is there to call). Returns true once the enroute
/// leg completes, false on abort or the four-hour ceiling.
async fn wait_enroute_working_the_radio(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    callsign: &Option<String>,
    climb_to_ft: f64,
    dest: Option<(f64, f64, i64)>,
) -> bool {
    let deadline = Instant::now() + Duration::from_secs_f64(4.0 * 3600.0);
    // Landing and taxi lights off once clear of the pattern: one read per
    // pass until the height is made, then never again this leg.
    let mut landing_off = false;
    loop {
        if ABORT.load(SeqCst) {
            return false;
        }
        if !crate::procedure::running() {
            return crate::procedure::outcome() == Some(sim::run::Outcome::Completed);
        }
        if Instant::now() >= deadline {
            return false;
        }
        if !landing_off {
            let agl_ft = xp.get_float(AGL_M).await.unwrap_or(0.0) * M_TO_FT;
            if agl_ft >= LANDING_LIGHTS_OFF_AGL_FT {
                landing_off = true;
                let did = crate::tools::lights::after_climb(xp).await;
                event(
                    "lights_climb",
                    json!({ "off": did, "agl_ft": agl_ft.round() }),
                );
            }
        }
        if let Some((to, khz)) = crate::party_line::take_departure_handoff() {
            crate::circuit::transmit(crate::circuit::radio_call(
                callsign,
                &format!("{to} on {}", crate::atis::spoken_mhz(khz)),
            ));
            tokio::time::sleep(Duration::from_secs(HANDOFF_READBACK_SECS)).await;
            let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
            // The altitude is read by the main loop as the call goes out, not
            // here: the queue can hold it for twenty seconds behind a busy
            // frequency, and "passing one thousand" was said at 1,207 ft.
            let alt_ft = xp.get_float(ALT_IND_FT).await.unwrap_or(0.0);
            crate::circuit::transmit_check_in(callsign.clone(), to.clone(), climb_to_ft);
            event(
                "departure_checkin",
                json!({ "to": to, "khz": khz, "alt_ft_queued": alt_ft.round() }),
            );
        }
        if let Some(h) = crate::party_line::take_region_handoff() {
            obey_region_handoff(xp, ctx, voice, callsign, h, true).await;
        }
        // The tower's question is drained every pass whoever asked it: an
        // answer is owed the destination's tower (the inbound call, below);
        // a transit field's challenge has no answer yet and is blackboxed.
        let challenged = crate::party_line::take_zone_challenge();
        let home = crate::party_line::home_tower_khz();
        if let Some((dlat, dlon, dkhz)) = dest {
            let on_dest = home == Some(dkhz);
            if challenged && !on_dest {
                event(
                    "zone_challenge",
                    json!({ "answered": false, "home_khz": home, "dest_khz": dkhz }),
                );
            }
            if on_dest && !INBOUND_MADE.load(SeqCst) {
                let dist_nm = match tokio::join!(xp.get_float(LAT), xp.get_float(LON)) {
                    (Ok(lat), Ok(lon)) => crate::nav::haversine_nm(lat, lon, dlat, dlon),
                    _ => f64::INFINITY,
                };
                if inbound_due(Some(dkhz), home, dist_nm, challenged, false) {
                    event(
                        "inbound_enroute",
                        json!({
                            "khz": dkhz, "dist_nm": (dist_nm * 10.0).round() / 10.0,
                            "challenged": challenged,
                        }),
                    );
                    call_inbound(xp, ctx, voice, callsign, dkhz, true).await;
                }
            }
        } else if challenged {
            event(
                "zone_challenge",
                json!({ "answered": false, "home_khz": home }),
            );
        }
        tokio::time::sleep(Duration::from_millis(2000)).await;
    }
}

/// The crew's readback of the enroute handoff, on the frequency it came on:
/// "squawking seven thousand, Malaga Tower on one one eight decimal one".
/// Shaped as a READBACK (the tower FSM's `is_readback` words) so the
/// controller who gave it answers with silence, never "say again".
pub fn region_readback(callsign: &Option<String>, h: &crate::party_line::RegionHandoff) -> String {
    let lead = if h.squawk_vfr {
        "squawking seven thousand"
    } else {
        "roger"
    };
    crate::circuit::radio_call(
        callsign,
        &format!("{lead}, {} on {}", h.to, crate::atis::spoken_mhz(h.khz)),
    )
}

/// Obey "radar services terminated, squawk seven thousand, contact X Tower on
/// Y": read it back on the old frequency (when `readback`: the arrival stage
/// has already tuned the destination, and a readback there would land on the
/// NEW tower as a garbled call), give the discrete code back, retune COM1.
/// The spoken line had no crew consumer until 2026-09-14: the flight kept the
/// departure code and the old frequency until its own arrival stage.
async fn obey_region_handoff(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    callsign: &Option<String>,
    h: crate::party_line::RegionHandoff,
    readback: bool,
) {
    if readback {
        crate::circuit::transmit(region_readback(callsign, &h));
        tokio::time::sleep(Duration::from_secs(HANDOFF_READBACK_SECS)).await;
    }
    let squawk_ok = if h.squawk_vfr {
        Some(
            crate::tools::radio::SetTransponder
                .execute(&json!({ "code": 7000 }), ctx)
                .await
                .is_ok(),
        )
    } else {
        None
    };
    let tuned = xp.set_float(COM1_FREQ_KHZ, h.khz as f64).await.is_ok();
    event(
        "region_handoff",
        json!({
            "to": h.to, "khz": h.khz, "squawk_vfr": h.squawk_vfr,
            "squawk_set": squawk_ok, "tuned": tuned, "readback": readback,
        }),
    );
    if squawk_ok == Some(true) {
        announce(voice, Priority::Advisory, "Seven thousand set.");
    }
}

/// The first call to the departure controller: who we are, where we are,
/// where we're going ("Malaga Departure, November four five X-ray Sierra,
/// passing two thousand climbing four thousand"). Shaped so the station reads
/// it as a check-in (a readback with "passing"/"climbing") and answers "radar
/// contact". With no planned altitude the climb clause is left off rather
/// than invented.
pub fn departure_check_in(
    callsign: &Option<String>,
    to: &str,
    alt_ft: f64,
    climb_to_ft: f64,
) -> String {
    let who = callsign
        .as_ref()
        .map(|c| format!("{c}, "))
        .unwrap_or_default();
    let climb = if climb_to_ft > alt_ft + 200.0 {
        format!(" climbing {}", crate::atis::spoken_alt(climb_to_ft))
    } else {
        String::new()
    };
    format!(
        "{to}, {who}passing {}{climb}",
        crate::atis::spoken_alt(alt_ft)
    )
}

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

/// Both instruments must affirm the ground gate; an API failure is no clearance.
fn confirmed_ground(agl_m: anyhow::Result<f64>, on_ground: anyhow::Result<f64>) -> bool {
    matches!((agl_m, on_ground), (Ok(agl), Ok(ground))
        if agl.is_finite() && ground.is_finite() && agl * M_TO_FT <= 50.0 && ground > 0.5)
}

async fn fly(xp: XPlaneClient, config: Config, voice: Option<Voice>, tools: SubTools) {
    // A new mission starts owing nothing to the last one's destination: a pin
    // left by the previous arrival would keep the region from ever handing
    // this departure on (see `release_arrival_station`).
    release_arrival_station();
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

    if let Mission::Resume(route) = &tools.mission {
        let callsign = crate::circuit::crew_callsign(&xp).await;
        // The normal follower owns speed, route and arrival; no duplicate laws.
        fly_route(&xp, &config, &voice, &ctx, &tools, route, &callsign).await;
        return;
    }

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
    // Full auto is a GATE-to-gate chain: armed in the air it walks its ground
    // stages against reality. After the missed-approach give-up the pilot hit
    // FULL AUTO at 1,475 ft descending at 900 fpm, and the chain read a stale
    // `LAST_ARRIVED` from the morning's taxi as "at the hold", called ready
    // for departure, and requested a takeoff clearance while the aircraft
    // sank (EGHI, 2026-08-02). Airborne, the honest answer is the tools that
    // fly: refuse and name them.
    let (agl, ground) = tokio::join!(
        xp.get_float(AGL_M),
        xp.get_float(crate::datarefs::ON_GROUND_ANY),
    );
    if !confirmed_ground(agl, ground) {
        give_up(
            &voice,
            "Full auto needs confirmed ground contact. Say navigate to follow the plan, fly the ILS, or land for me when airborne.",
            "ground_unconfirmed",
        );
        return;
    }
    // The callsign the tower knows us by, so our calls sound like a real pilot.
    let callsign = crate::circuit::crew_callsign(&xp).await;

    // On a Flight mission the route goes into the GPS NOW, at the gate. The
    // write used to sit at the top of stage 4 — fly_route was where the
    // concrete client happened to be — so the whole start-up and taxi ran
    // with an empty flight-plan page, and the pilot read that as "it didn't
    // load" and commanded the load by hand mid-taxi (LXGB to LEMG,
    // 2026-08-02). The map during the taxi is half of what loading the plan
    // is FOR. Best effort, exactly as it was at the runway.
    if let Mission::Flight(route) = &tools.mission {
        if !route.fms_payload.is_empty() {
            let _ = xp
                .set_data("xcopilot/fms/route", route.fms_payload.as_bytes())
                .await;
            let _ = xp.set_int("xcopilot/fms/load", 1).await;
            event(
                "fms_loaded",
                json!({ "waypoints": route.fms_payload.split('|').count() }),
            );
        }
    }

    // --- Stage 1: electrics + engine ------------------------------------------
    if !engine_running(&xp).await {
        // A turbine we have no sequence for is not a piston. Say so and stop,
        // rather than crank a starter this airframe hasn't got — the failure
        // the MD-82 taught us, from the other side (see `StartKind::None`).
        if tools.no_start_sequence {
            give_up(
                &voice,
                "I don't have an engine start for this aircraft. Get the engines running and say full auto again.",
                "no_start_sequence",
            );
            return;
        }
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
            // Run the bring-up INLINE, not through the PowerUp tool: the tool
            // spawns the sequence and answers "running the electrical"
            // immediately, so full auto walked straight on to the engine start,
            // which found the APU still down and started a SECOND copy of the
            // same sequence. Two racing copies both read a cross-tie as open and
            // both fire its toggle command, which leaves it shut. Here the
            // orchestrator is already on its own thread, so waiting is free and
            // "power up, then crank" is true again.
            crate::tools::power::run_prep(&xp, &voice, &None).await;
            event("power_up", json!({ "inline": true }));
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

        // Beacon BEFORE the propeller moves — that is what it is for, and it
        // is the one light with an actual safety job on a ramp. Nav lights with
        // it, since they belong on whenever the master is.
        let beacon = light(&tools, &ctx, "beacon", true).await;
        light(&tools, &ctx, "nav", true).await;
        // Panel lighting with them. The bus is live and the pilot is about to
        // be asked to read instruments.
        {
            use crate::tools::Tool;
            let lit = tools
                .flood
                .execute(&json!({ "on": true }), &ctx)
                .await
                .is_ok();
            event("lights_panel", json!({ "on": lit }));
        }
        event("lights_beacon", json!({ "beacon": beacon }));
        // Only claim the beacon on an airframe that has one — the Lancair's
        // switch row is NAV / LAND / STRB and nothing else.
        announce(
            &voice,
            Priority::Advisory,
            if beacon {
                crate::readback::one_of(&[
                    "Beacon on. Starting the engine.",
                    "Beacon's on, spinning up the engine.",
                    "Beacon on, cranking her over.",
                ])
            } else {
                crate::readback::one_of(&[
                    "Starting the engine.",
                    "Spinning up the engine.",
                    "Cranking her over.",
                ])
            },
        );
        if let Ok(reply) = tools.start_engine.execute(&json!({}), &ctx).await {
            announce(&voice, Priority::Advisory, &reply);
        }
        // The starter cranks before it catches — for seconds on a piston, for
        // minutes on the jet's two-engine bleed-air sequence.
        waiting("the engine to catch");
        if !wait_until(tools.engine_start_secs, 500, || engine_running(&xp)).await {
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
        if !(tools.power_up_first
            || tools.all_electrical_on_cmds.is_empty() && tools.all_electrical_toggles.is_empty())
        {
            for cmd in &tools.all_electrical_on_cmds {
                let _ = xp.run_command(cmd).await;
            }
            // Toggle-only systems have no ON command, so a bulk power-up that
            // fires command lists skips them SILENTLY. That left the Citation's
            // avionics master off after "electrical set", and the radios dead.
            for t in &tools.all_electrical_toggles {
                if xp.get_float(&t.dataref).await.unwrap_or(0.0) <= 0.5 {
                    let _ = xp.run_command(&t.command).await;
                }
            }
            // What was set is the profile's list; "generators and bleeds"
            // on a 172 was an airliner's line (2026-09-15), and the start
            // line has already named the alternator and avionics.
            announce(&voice, Priority::Advisory, "Electrical set.");
            event(
                "electrical_set",
                json!({
                    "cmds": tools.all_electrical_on_cmds.len(),
                    "toggles": tools.all_electrical_toggles.len(),
                }),
            );
        }
    }
    bail_if_abort!();

    // --- Stage 2: altimeter ---------------------------------------------------
    // NO brake release here, deliberately. The brakes used to come off at this
    // stage — engine just started, taxi clearance not yet requested — which is
    // minutes before anything drives, and an idle jet on a flat apron rolls:
    // the SF50 crept from 0 to 2.8 kt with `park=0.0` while full auto was
    // still WAITING FOR THE TUG, and ended up in the grass (Luton,
    // 2026-07-28). Nobody needs the early release: the taxi governor
    // auto-releases (valve-aware) when a drive arms, the pushback stage
    // handles its own brakes-off at hook-up, and the auto-takeoff releases on
    // the runway. Brakes stay as the pilot left them until something is
    // actually driving.

    // Set the altimeter to field QNH before departure — a misset altimeter is
    // the classic level-bust. Autoland aims at the runway's true elevation so it
    // doesn't depend on this, but a real crew sets it, and everything MSL between
    // here and there reads right once it's set.
    if let Some(sync) =
        crate::tools::instruments::sync_altimeter(&xp, tools.transition_altitude_ft).await
    {
        announce(
            &voice,
            Priority::Advisory,
            &format!(
                "Altimeter set, {}.",
                crate::tools::instruments::spoken_setting(sync.now)
            ),
        );
        event(
            "altimeter_set",
            json!({ "inhg": (sync.now * 100.0).round() / 100.0, "was": (sync.was * 100.0).round() / 100.0 }),
        );
    }

    // ...and align the DG to the compass, the other half of the same two
    // items. The monitor's auto-sync is deliberately AIRBORNE-only, on the
    // rule that on the ground you align at the run-up — which is right, and
    // full auto IS the run-up. Left to the monitor, the sync lands at the
    // first moment the wheels leave the ground: at Southampton a Baron taxied,
    // lined up and started its roll with the heading indicator 159.7 degrees
    // out, and the alignment arrived twenty-six seconds after the line-up
    // (2026-08-01). Nothing the automation flies reads the DG (the AP bug is a
    // magnetic heading, and every ground loop uses true heading), so this is
    // for the pilot watching the panel — which is reason enough.
    let _ = xp.run_command("sim/instruments/DG_sync_mag").await;
    event("gyro_synced", json!({}));
    bail_if_abort!();

    // --- Stage 3: onto the runway ---------------------------------------------
    // Already lined up (the pilot put us on)? Skip straight to the circuit.
    if !lined_up(&xp, &tools, &config).await {
        // Clearance first — the real order is clearance, THEN taxi. Only with a
        // simulated tower to answer (real/external ATC and copilot-only leave the
        // clearance to the pilot), and only if we don't already hold a squawk.
        if crate::mode::atc_enabled() && crate::party_line::assigned_squawk().is_none() {
            if let Some(khz) = crate::party_line::home_tower_khz() {
                let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
            }
            announce(
                &voice,
                Priority::Advisory,
                crate::readback::one_of(&[
                    "Requesting clearance.",
                    "Getting our clearance.",
                    "Calling for our clearance.",
                ]),
            );
            crate::circuit::transmit(crate::circuit::radio_call(&callsign, "request clearance"));
            event("request_clearance", json!({}));
            // The tower answers with a discrete squawk; wait for it, then dial it
            // into the transponder and read it back — the crew's clearance copy.
            // The clock starts when the call is QUEUED, and the call rides the
            // speech queue behind our own arming brief and anyone the tower is
            // working: at Malaga 30 s gave up 13 s before the clearance came
            // back, the squawk was never set, and the tower spent the climb
            // asking us to confirm 4503 while we showed 7000 (23:06Z,
            // 2026-09-12). So the wait is longer, and the squawk is OWED, not
            // missed: `ensure_squawk` sets a late one at the hold and lined up.
            waiting("the squawk");
            wait_until(60.0, 500, || async {
                crate::party_line::assigned_squawk().is_some()
            })
            .await;
            ensure_squawk(&xp, &ctx, &voice).await;
            bail_if_abort!();
            // The clearance is read back like a real one: the zone cap, the
            // QNH and the squawk (the tower suggests the words, enforces
            // nothing, and answers a correct one with silence). Never done
            // before 2026-09-15: the squawk was set and the taxi asked for.
            if let Some(rb) = crate::party_line::readback_owed() {
                crate::circuit::transmit(rb.trim_end_matches('.').to_string());
                event("clearance_readback", json!({}));
            }
        }
        // The taxi route is the tower's. Ask for it if we don't already hold
        // one — or if the one we hold WAS PLANNED FROM SOMEWHERE ELSE. A
        // clearance outlives a reposition: the tower planned a route from the
        // aircraft's spot on Stansted's 22C, full auto's first attempt gave
        // up waiting one second before it arrived, the pilot moved to a gate
        // a kilometre away, and the SECOND attempt reused the fossil — read
        // "nose in" against a lane 1 km off, called a tug to push toward it,
        // and the taxi drive then (correctly) refused at 1090 m off route
        // (2026-07-29). A route is only OURS if it starts within reach of
        // the auto-taxi's own join limit; otherwise ask again and the tower
        // re-plans from where we actually are.
        // ...and a route to a STAND is never a departure's, however near it
        // starts. An arrival's clearance ends where we are parked, so the
        // reach test below passes it with room to spare: at Malaga full auto
        // adopted the taxi-in route it had just finished driving, skipped the
        // request entirely, drove 33 s of an already-complete 88 m route back
        // toward the stand from 66 m off it, and announced itself at the
        // holding point — the tower's answer to the departure request was
        // "taxi to holding point Hotel Whiskey one FIRST", and the line-up
        // then failed (2026-08-05). The party line already knows which kind
        // of route it published; nobody had asked it.
        let route_is_ours = !crate::party_line::taxi_to_stand() && {
            let (lat_r, lon_r) = tokio::join!(xp.get_float(LAT), xp.get_float(LON));
            match (lat_r, lon_r) {
                (Ok(lat), Ok(lon)) => crate::party_line::with_taxi_route(|r| {
                    r.map(|w| {
                        w.path
                            .iter()
                            .any(|p| crate::nav::haversine_nm(lat, lon, p.0, p.1) * 1852.0 < 200.0)
                    })
                })
                .unwrap_or(false),
                // Position unknown: trust the route rather than re-request in
                // a loop we can't verify our way out of.
                _ => crate::party_line::with_taxi_route(|r| r.is_some()),
            }
        };
        if !route_is_ours {
            if crate::party_line::with_taxi_route(|r| r.is_some()) {
                let arrival = crate::party_line::taxi_to_stand();
                announce(
                    &voice,
                    Priority::Advisory,
                    if arrival {
                        "That was our taxi in to the stand. Asking for a departure clearance."
                    } else {
                        "That taxi clearance was from the old spot. Asking again from here."
                    },
                );
                event("stale_taxi_route", json!({ "arrival": arrival }));
            }
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
            // came back with the clearance (16:45Z, 2026-07-24) — and 90 s
            // then gave up ONE SECOND before a busy Stansted did the same
            // thing (2026-07-29, the tower was working a Ryanair arrival and
            // berating us for sitting on its runway). Whatever this number
            // is, a queued frequency will someday beat it by a second; 180
            // outlasts two worked arrivals.
            waiting("a taxi route");
            let got = wait_until(180.0, 1000, || async {
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

        // Off the stand FIRST, when the cleared route's way out runs behind
        // us: at a nose-in gate the taxi drive's opening metres point into
        // the terminal. Read from the route itself, so a tie-down we can taxi
        // straight out of never calls a tug (see `pushback::plan`). Best
        // effort by design — a field or airframe with no tug falls back to
        // the taxi join's own turn-around, which is ugly on an apron but is
        // exactly the behaviour that shipped before this stage existed. The
        // push itself is narrated by the monitor's pushback watch, so the
        // only crew line here is the ask.
        {
            let (lat_r, lon_r, psi_r) = tokio::join!(
                xp.get_float(LAT),
                xp.get_float(LON),
                xp.get_float(crate::autotaxi::PSI_TRUE),
            );
            let path = crate::party_line::with_taxi_route(|r| r.map(|w| w.path.clone()))
                .unwrap_or_default();
            if let (Ok(lat), Ok(lon), Ok(psi)) = (lat_r, lon_r, psi_r) {
                let plan = crate::pushback::plan(lat, lon, psi, &path);
                let stand = crate::party_line::nearest_stand(lat, lon);
                let at_stand = crate::pushback::on_a_stand(
                    stand.as_ref().map(|s| (s.dist_m, s.heading)),
                    psi,
                    crate::party_line::stands_known(),
                );
                if plan.is_some() && !at_stand {
                    // The way out is behind us but this is not a stand: a
                    // restart from an off-route stop, mid-taxiway. The taxi
                    // join turns the aircraft round; a tug here is theatre.
                    announce(
                        &voice,
                        Priority::Advisory,
                        "Way out's behind us, but we're not on a stand, so no tug. \
                         I'll turn us round under our own steam. Watch the wingtips.",
                    );
                    event(
                        "pushback_skipped",
                        json!({
                            "why": "not_on_stand",
                            "stand": stand.as_ref().map(|s| s.name.clone()),
                            "dist_m": stand.as_ref().map(|s| s.dist_m),
                        }),
                    );
                }
                if let Some(plan) = plan.filter(|_| at_stand) {
                    announce(
                        &voice,
                        Priority::Advisory,
                        crate::readback::one_of(&[
                            "We're nose in. Calling for pushback.",
                            "Way out's behind us. Calling for a push.",
                            "Nose in here, so pushback first.",
                        ]),
                    );
                    event(
                        "pushback",
                        json!({ "back_m": plan.back_m, "target_hdg": plan.target_hdg }),
                    );
                    match crate::pushback::run(
                        &xp,
                        &ctx,
                        &voice,
                        &plan,
                        &ABORT,
                        tools.taxi.park_brake_valve_set,
                    )
                    .await
                    {
                        Ok(back_m) => event("pushback_done", json!({ "back_m": back_m })),
                        Err(why) => {
                            event("pushback_failed", json!({ "why": why }));
                            // Say what actually happened. One line covered
                            // every failure, so a tug that came, hooked on and
                            // pushed us fifty metres was reported as "no tug
                            // came" — the pilot watching it out the window has
                            // no reason to trust anything else the copilot
                            // says after that (2026-07-27). An abort is the
                            // pilot's own and is already spoken for.
                            let line = match why.as_str() {
                                "abort" => "",
                                "no_tug" => {
                                    "No tug came, so I'll turn us around under our own steam. \
                                     Watch the wingtips."
                                }
                                "tug_released" => {
                                    "The tug let go early. I'll bring us round the rest of the \
                                     way myself. Watch the wingtips."
                                }
                                "skipped" => {
                                    "Skipping the push. I'll turn us round under our own steam. \
                                     Watch the wingtips."
                                }
                                "creeping" => {
                                    "The tug never pushed us and we were rolling forward, so \
                                     I've got the brakes. I'll turn us around under our own \
                                     steam. Watch the wingtips."
                                }
                                _ => {
                                    "That's as far as the push got. I'll bring us round the rest \
                                     of the way myself. Watch the wingtips."
                                }
                            };
                            if !line.is_empty() {
                                announce(&voice, Priority::Advisory, line);
                            }
                        }
                    }
                }
            }
        }
        bail_if_abort!();

        // Drive the cleared route to the holding point. The taxi light, or
        // on an airframe that has none the landing light: the SR22 carries
        // only nav, strobe and landing, and the taxi ran dark because the
        // "taxi" key was declared nowhere (2026-09-12). That is how the real
        // aircraft is taxied at night; it goes off at the stand with the rest.
        let dark = crate::tools::lights::dark_now(&xp).await;
        let (taxi_key, taxi_lit) = match crate::tools::lights::taxi_light(dark) {
            Some(l) => {
                let ok = crate::tools::lights::apply(&xp, &l, true).await.is_ok();
                (l.key.clone(), ok)
            }
            None => ("none".to_string(), false),
        };
        event(
            "lights_taxi",
            json!({ "on": taxi_lit, "light": taxi_key, "dark": dark }),
        );
        announce(
            &voice,
            Priority::Advisory,
            crate::readback::one_of(&[
                "Taxiing to the runway.",
                "Rolling out to the runway.",
                "Taking us to the runway.",
            ]),
        );
        if let Ok(reply) = tools.taxi.execute(&json!({}), &ctx).await {
            announce(&voice, Priority::Advisory, &reply);
        }
        // Wait for the drive to finish. `driving()`, not ACTIVE: a backtrack
        // drops ACTIVE in the gap between its two legs, and polling the bare
        // flag reads that gap as "finished".
        waiting("the taxi to arrive");
        if crate::autotaxi::driving()
            && !wait_until(600.0, 1000, || async { !crate::autotaxi::driving() }).await
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
        // A squawk that came back after the clearance wait gave up is set
        // here, before we talk to the tower again.
        ensure_squawk(&xp, &ctx, &voice).await;

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
                // Held SHORT: a line-up-and-wait is a good enough answer to
                // get us moving — the clearance is asked for again from the
                // runway, below.
                if !ensure_takeoff_cleared(&voice, &callsign, &mut read_back, true).await {
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
            // A CONDITIONAL line-up ("behind the landing X, line up behind")
            // is a clearance to enter AFTER X, not now. The Baron at Malaga
            // rolled five seconds after the words with the lander at 2.4 nm
            // and reached the pavement 15 s after it touched down — the
            // margin was the 85 s crawl, not any logic (12:21Z, 2026-09-11).
            // The world publishes the condition every tick from its traffic
            // picture; wait for it to clear, or for the hold to become a
            // release (the condition goes away with it).
            if atc_running && crate::party_line::line_up_condition_clear() == Some(false) {
                announce(
                    &voice,
                    Priority::Advisory,
                    "Behind the landing traffic. Waiting for it to pass.",
                );
                event("line_up_behind_wait", json!({}));
                waiting("the landing traffic to pass");
                if !wait_until(LINE_UP_BEHIND_SECS, 500, || async {
                    crate::party_line::line_up_condition_clear() != Some(false)
                })
                .await
                {
                    give_up(
                        &voice,
                        "The landing traffic never cleared. Holding short. Your aircraft.",
                        "line_up_behind_timeout",
                    );
                    return;
                }
                bail_if_abort!();
                event("line_up_behind_clear", json!({}));
            }
            // Once: a backtrack is two executions (the drive to the far end,
            // then the turn onto the line), and the tool's own reply names
            // each leg. "Taking the runway" twice in ninety seconds, LXGB 27,
            // 2026-09-15.
            if attempt == 0 {
                announce(
                    &voice,
                    Priority::Advisory,
                    crate::readback::one_of(&["Lining up.", "Lining us up.", "Taking the runway."]),
                );
            }
            if let Ok(reply) = tools.line_up.execute(&json!({}), &ctx).await {
                announce(&voice, Priority::Advisory, &reply);
            }
            // A full-length backtrack is a long, slow drive (LXGB's 09 backtrack
            // is ~1 km) — the 120 s that fit a plain line-up timed out mid-taxi
            // (2026-07-26). Give it the taxi's budget.
            // `driving()` covers the gap between a backtrack's two legs —
            // ACTIVE alone drops there, and full auto called a running line-up
            // a failure one second after leg two engaged (2026-07-28).
            waiting("the taxi to arrive");
            if crate::autotaxi::driving()
                && !wait_until(600.0, 1000, || async { !crate::autotaxi::driving() }).await
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
        ensure_squawk(&xp, &ctx, &voice).await;
        // On the runway: strobes and landing lights. These are the two that
        // say "this aeroplane is about to move fast", which is why they go on
        // here and not at the holding point.
        let strobe = light(&tools, &ctx, "strobe", true).await;
        let landing = light(&tools, &ctx, "landing", true).await;
        event(
            "lights_takeoff",
            json!({ "strobe": strobe, "landing": landing }),
        );

        // The takeoff clearance, if the line-up went on under line-up-and-wait
        // (already satisfied when the held-short path above got it first).
        if atc_running && !ensure_takeoff_cleared(&voice, &callsign, &mut read_back, false).await {
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
            if let Ok(reply) = tools.circuit.execute(&args, &ctx).await {
                announce(&voice, Priority::Advisory, &reply);
            }
            // A refusal comes back Ok with the reason as the reply, so the
            // reply proves nothing — the FLAG is the arm, exactly as
            // fly_route checks the takeoff's. Without this, a refused
            // circuit fell through to a 900 s wait for an autoland that was
            // never coming, with every re-arm answered "I'm already running
            // the full sequence" (audit, 2026-08-02). The refusal line has
            // already been spoken; this only stands the sequence down.
            if !crate::circuit::ACTIVE.load(SeqCst) {
                give_up(
                    &voice,
                    "The circuit didn't arm, so I'm standing down. Your aircraft.",
                    "circuit_refused",
                );
                return;
            }
            event("circuit_armed", json!({}));
            // The circuit owns the FLYING from here, but not the last stage:
            // it hands the final to autoland, which stops on the runway and
            // lets go. Somebody has to vacate. See `taxi_in_after_landing` —
            // ninety seconds holding Southampton's 02 with an arrival at half
            // a mile is what "our job is done" used to mean here.
            taxi_in_after_landing(&xp, &config, &voice, &tools, true).await;
            ACTIVE.store(false, SeqCst);
        }
        Mission::Flight(route) | Mission::Resume(route) => {
            fly_route(&xp, &config, &voice, &ctx, &tools, route, &callsign).await;
        }
    }
}

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const AGL_M: &str = "sim/flightmodel/position/y_agl";
/// The altimeter, not true elevation: the check-in's "passing" and the arrival's
/// platform altitudes are what the controller and the chart mean.
const ALT_IND_FT: &str = crate::datarefs::ALTITUDE_FT;
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

/// How long the circuit's taxi-in stage waits for autoland to ARM before
/// concluding the pattern never got that far. Generous: a takeoff, a climb to
/// pattern altitude, a full rectangle and a base leg, at trainer speeds, with
/// a go-around's worth of slack on top.
const ARM_WAIT_SECS: f64 = 900.0;

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

    // (The GPS already has the route: fly() writes the FMS mailbox at arm
    // time, at the gate, so the taxi happens with the plan on the map.)

    if !matches!(tools.mission, Mission::Resume(_)) {
    // --- Takeoff: the proven loop, armed through its tool so every arm-time
    // gate applies (the pitch-channel probe included).
    announce(
        voice,
        Priority::Advisory,
        &format!(
            "Cleared to go. Flying your plan to {}.",
            route.dest_name
        ),
    );
    if let Ok(reply) = tools.takeoff.execute(&json!({}), ctx).await {
        announce(voice, Priority::Advisory, &reply);
    }
    if !crate::autotakeoff::ACTIVE.load(Relaxed) {
        give_up(
            voice,
            "The takeoff didn't arm, so the flight is off. Your aircraft.",
            "takeoff_refused",
        );
        return;
    }
    waiting("the takeoff");
    if !wait_until(360.0, 1000, || async { !crate::autotakeoff::running() }).await {
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
    if crate::autotakeoff::outcome() != Some(sim::run::Outcome::Completed) {
        give_up(
            voice,
            "The takeoff handed control back. Your aircraft.",
            "takeoff_not_completed",
        );
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

    } else {
        let (ground, agl) = tokio::join!(xp.get_float(crate::datarefs::ON_GROUND_ANY), xp.get_float(AGL_M));
        if !matches!((ground,agl), (Ok(g),Ok(h)) if g.is_finite() && h.is_finite() && g < 0.5 && h*M_TO_FT >= 1000.0) {
            give_up(voice, "Aircraft state changed before resume. I'm monitoring only; you have control.", "resume_state_changed");
            return;
        }
        event("resumed", json!({ "fix": route.legs.first().map(|l| &l.fix_ident) }));
        // Reload only after the explicit resume; fms_goto selects the joined fix.
        let _ = xp.set_data("xcopilot/fms/route", route.fms_payload.as_bytes()).await;
        let _ = xp.set_int("xcopilot/fms/load", 1).await;
    }

    // --- Enroute: the procedure follower over the plan's own fixes. It climbs
    // to cruise on the first constraint and flies the plan's descent numbers
    // after TOD; quiet completion (EndAction::Enroute) hands back here.
    if !crate::procedure::start_scoped(
        config.xplane_base_url.clone(),
        voice.clone(),
        crate::procedure::Plan {
            name: format!("the flight plan to {}", route.dest_name),
            legs: route.legs.clone(),
            end: crate::procedure::EndAction::Enroute,
            // The climb to cruise runs on the speed-protected law — the
            // unprotected rate stalled the first LEJR climb at 53 knots.
            vy_kts: tools.takeoff.vy_kts,
            cruise_ias_kts: tools.cruise_ias_kts,
            transition_altitude_ft: Some(tools.transition_altitude_ft),
        },
        xp.cancellation(),
    ) {
        give_up(
            voice,
            "The previous procedure is still stopping. Your aircraft.",
            "procedure_busy",
        );
        return;
    }
    // The CLIMB and CRUISE checklists' mixture items, flown rather than read:
    // the continuous manager keeps it full rich below the leaning floor and
    // leans to peak EGT above it, re-trimming as the climb and the power
    // settle. The pilot's hand on the lever, or "my mixture", releases it.
    // Armed silently: the manager says what it did when it first leans
    // (`lean_seek`'s own line), and this moment already carries the
    // thousand-foot callout, "flying the plan" and the handoff readback
    // (four lines in one second at LXGB, 2026-09-15).
    if tools.piston_mixture {
        crate::tools::engine::arm_mixture_manager(config.xplane_base_url.clone());
        event("mixture_managed", json!({}));
    }
    // Generous ceiling — ABORT bails out of the wait immediately. The wait
    // works the radio: the tower's after-takeoff handoff is owed a readback,
    // a retune and a check-in.
    let climb_to_ft = route
        .legs
        .iter()
        .filter_map(|l| l.alt1_ft)
        .fold(0.0, f64::max);
    // The destination's own position and tower, for the inbound call at the
    // zone approach: the apt.dat row, never `home_tower_khz`, which is
    // whoever the region last handed us to.
    let dest_tower =
        crate::nav::find_airport(&crate::tools::autoland::apt_dat_path(), &route.dest_icao)
            .ok()
            .flatten()
            .and_then(|a| {
                let (lat, lon) = a.position()?;
                let khz = a.freq(crate::nav::FREQ_TOWER)?.khz;
                Some((lat, lon, khz))
            });
    if !wait_enroute_working_the_radio(xp, ctx, voice, callsign, climb_to_ft, dest_tower).await {
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
    // The BEFORE LANDING item: full rich for the arrival, whatever the field
    // elevation (the manager's own floor would leave a leaned lever over a
    // high field), before the approach or the visual landing is built.
    if tools.piston_mixture {
        match crate::tools::engine::mixture_rich_for_landing(xp).await {
            Some(moved) => {
                event("mixture_rich", json!({ "moved": moved }));
                if moved {
                    announce(voice, Priority::Advisory, "Mixture full rich.");
                }
            }
            None => event("mixture_rich", json!({ "moved": false, "unread": true })),
        }
    }

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
    let alt_msl_ft = xp.get_float(ALT_IND_FT).await.unwrap_or(0.0);
    // The surface wind AT THE FIELD chooses the landing runway (see
    // build_arrival): the destination's METAR first, the aircraft-wind
    // datarefs only without one. The datarefs are the wind HERE, and here is
    // 30 nm out at 7,000 ft: LEJR-LXGB read a light westerly aloft, set up
    // the straight-in to 27, and the tower's "negative, runway zero niner,
    // wind zero niner zero degrees, two one knots" came with the landing
    // clearance (2026-09-14).
    let aloft = match (
        xp.get_float(crate::datarefs::WIND_DIR_DEG_TRUE).await,
        xp.get_float(crate::datarefs::WIND_SPEED_MSC).await,
    ) {
        (Ok(dir), Ok(spd_ms)) => Some((dir, spd_ms * MS_TO_KTS)),
        _ => None,
    };
    let metar =
        crate::weather::surface_wind(&crate::tools::autoland::xplane_root(), &route.dest_icao);
    let (wind, wind_source) = arrival_wind(metar, aloft);
    let wind_detail = json!({
        "source": wind_source,
        "dir": wind.map(|(d, _)| d.round()),
        "kts": wind.map(|(_, k)| k.round()),
    });
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
    if route
        .local_runway
        .as_ref()
        .is_some_and(|r| built.rwy_id.as_ref() != Some(r))
    {
        give_up(
            voice,
            "The arrival runway changed. The local route needs replanning. Your aircraft.",
            "local_runway_changed",
        );
        return;
    }
    // The radio: tune the DESTINATION'S tower from its own apt.dat comm row —
    // never `home_tower_khz`, which is whoever the enroute region last handed
    // us to. At LINTO that was Tangier (genuinely the nearest field,
    // mid-strait), and the inbound call earned "join final runway two eight"
    // from Ibn Battouta Tower while we set up for Gibraltar's zero nine
    // (17:32Z, 2026-07-24). The region hands the STATION over as we close;
    // with COM1 already on the destination's frequency the handoff finds us
    // listening.
    let mut inbound_owed: Option<i64> = None;
    if let Some(khz) = built.tower_khz.or_else(crate::party_line::home_tower_khz) {
        let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
        // You call it, you get it - for the crew's own retune as much as the
        // pilot's. The enroute region weighs a field by its published
        // services, so a tower-only strip beside a primary never wins the
        // sky over its own runway (LEAX 123.5 against Malaga's five rows:
        // both the inbound and the final went into the void on a frequency
        // no station held, 2026-09-15). Tuning the destination's tower arms
        // that field the way a keyed transmission on its frequency does; the
        // swap wait below then finds it on the next loop pass.
        crate::station::arm_station_for_freq(khz, lat, lon, alt_msl_ft, Instant::now());
        // And HOLD it there for the rest of the arrival: arming only wins the
        // next swap, where the region's tick keeps running all the way to the
        // flare and will hand a nearer field the station we are talking to.
        crate::station::pin_arrival_station(Some(khz));
        // The call is worth nothing until the station on that frequency IS
        // the destination: at PIMOS the region swapped to Gibraltar three
        // seconds AFTER "inbound for landing" went out, the call went into
        // the void, and Gibraltar's zone watch challenged the arrival as an
        // unknown aircraft (12:36Z, 2026-09-11). The swap is distance-driven
        // and due about now; wait for it, bounded, and call either way.
        // Already made by the enroute leg (the region handed us to this
        // tower inside INBOUND_CALL_NM): nothing owed, the tower knows us.
        let swapped = INBOUND_MADE.load(SeqCst) || {
            waiting("the destination's frequency");
            wait_until(INBOUND_SWAP_WAIT_SECS, 500, || async {
                crate::party_line::home_tower_khz() == Some(khz)
            })
            .await
        };
        if INBOUND_MADE.load(SeqCst) {
            event("inbound_already_made", json!({ "khz": khz }));
        } else if swapped {
            call_inbound(xp, ctx, voice, callsign, khz, true).await;
        } else {
            // Not yet the destination's frequency: the call would go into
            // the void (LEJR-LXGB, 2026-09-14: made 10 minutes before the
            // swap from Ibn Battouta, the nearest field for most of the
            // strait, so Gibraltar never heard an inbound and the final
            // report was its first word from us). Owed at the swap, made
            // below while the arrival flies; before the final report at the
            // latest.
            inbound_owed = Some(khz);
            event("inbound_deferred", json!({ "khz": khz }));
        }
    }
    // The wind-chosen end (build_arrival) drives BOTH the published-approach
    // branch and the visual fallback — the visual autoland is forced onto the
    // same runway so a tailwind can't be re-picked back in by track.
    // The runway AND the field it belongs to. A bare ident is resolved by
    // proximity, and on the EGPH arrival that found EGPJ's 24 nine miles
    // away across the Forth (2026-09-20) — a plan that names its destination
    // must never let the airport be a search result.
    let landing_rwy = built
        .rwy_id
        .as_deref()
        .map(|id| crate::autoland::PlannedRunway::at(&route.dest_icao, id));
    match built.plan {
        Some(plan)
            if route.local_runway.is_none()
                && crate::procedure::approach_flyable_from(&plan, lat, lon, alt_msl_ft) =>
        {
            event(
                "arrival_procedure",
                json!({ "name": plan.name, "runway": built.rwy_id, "wind": wind_detail }),
            );
            announce(
                voice,
                Priority::Advisory,
                &format!("Setting up {}.", plan.name),
            );
            if !crate::procedure::start_scoped(
                config.xplane_base_url.clone(),
                voice.clone(),
                plan,
                xp.cancellation(),
            ) {
                give_up(
                    voice,
                    "The previous procedure is still stopping. Your aircraft.",
                    "procedure_busy",
                );
                return;
            }
        }
        _ => {
            event(
                "arrival_autoland",
                json!({ "runway": built.rwy_id, "wind": wind_detail }),
            );
            announce(
                voice,
                Priority::Advisory,
                if route.local_runway.is_some() {
                    "On the local final. Landing us visually."
                } else {
                    "No published approach lines up from here, so I'll land us visually."
                },
            );
            crate::autoland::ABORT.store(false, SeqCst);
            crate::autoland::run_autoland_scoped(
                config.xplane_base_url.clone(),
                crate::tools::autoland::apt_dat_path(),
                tools.circuit.cfg.clone(),
                tools.circuit.approach_ias,
                voice.clone(),
                None,
                landing_rwy.clone(),
                xp.cancellation(),
            );
        }
    };

    // --- The final report is what earns the landing clearance: call it the
    // moment autoland takes the approach (the FAF handoff, or straight away on
    // the visual). Nothing left to actuate here — the arrival loops own the
    // aircraft — so this watch is the orchestrator's last duty.
    waiting("the arrival to hand to autoland");
    let started = Instant::now();
    let mut watch = ArrivalWatch::default();
    let called = loop {
        if ABORT.load(SeqCst) {
            break false;
        }
        // The deferred inbound call goes out the moment the destination's
        // station is the one on the frequency, or the moment a tower on it
        // asks who we are (the challenge is drained either way).
        let challenged = crate::party_line::take_zone_challenge();
        if let Some(khz) = inbound_owed {
            let swapped = crate::party_line::home_tower_khz() == Some(khz);
            if swapped || challenged {
                inbound_owed = None;
                call_inbound(xp, ctx, voice, callsign, khz, swapped).await;
            }
        }
        if crate::autoland::ACTIVE.load(Relaxed) || !crate::procedure::ACTIVE.load(Relaxed) {
            break true;
        }
        let elapsed = started.elapsed().as_secs_f64();
        let stalled = watch.stalled(
            crate::procedure::active_leg().map(|l| l.fix).as_deref(),
            crate::procedure::fix_distance_nm(),
            elapsed,
        );
        if stalled || elapsed >= ARRIVAL_CAP_SECS {
            event(
                "arrival_stalled",
                json!({ "elapsed_s": elapsed.round(), "stalled": stalled }),
            );
            break false;
        }
        tokio::time::sleep(Duration::from_millis(1000)).await;
    };
    // Out of patience with an approach still being flown: the follower dies
    // with this scope whatever we do here (its client refuses writes the
    // moment the lease drops), so stand it down while we can still speak and
    // still write, and hand back a FLYABLE aircraft — the rule autoland's own
    // give-ups follow. Silence here is what killed the EGPH arrival.
    if !called && !ABORT.load(SeqCst) && crate::procedure::ACTIVE.load(Relaxed) {
        crate::procedure::abort();
        // The follower's stand-down releases the speed governor; the throttle
        // write has to come after it, or the governor re-commands the power we
        // just set.
        wait_until(5.0, 250, || async {
            !crate::procedure::ACTIVE.load(Relaxed)
        })
        .await;
        let thr = crate::aircraft::throttle_dataref();
        let _ = tokio::join!(
            xp.set_float(&thr, GIVE_UP_THROTTLE),
            xp.set_float(VS_DIAL, 0.0),
        );
        give_up(
            voice,
            "The arrival isn't getting us there. Your aircraft. The autopilot is holding \
             heading and altitude, climb power set.",
            "arrival_stalled",
        );
        return;
    }
    // A procedure→autoland handoff has a brief both-inactive window while the
    // autoland thread spins up; give it a beat before concluding nobody flies.
    if called && !crate::autoland::ACTIVE.load(Relaxed) {
        tokio::time::sleep(Duration::from_secs(3)).await;
    }
    if crate::autoland::ACTIVE.load(Relaxed) && !ABORT.load(SeqCst) {
        // Never swapped (a field with no region of its own): the inbound
        // still precedes the final, on whatever tower is listening.
        if let Some(khz) = inbound_owed.take() {
            call_inbound(xp, ctx, voice, callsign, khz, false).await;
        }
        let call = match &landing_rwy {
            Some(rwy) => format!("final runway {}", crate::circuit::spell_runway(&rwy.id)),
            None => "final".to_string(),
        };
        crate::circuit::transmit(crate::circuit::radio_call(callsign, &call));
        event(
            "final_call",
            json!({ "runway": landing_rwy.as_ref().map(|r| &r.id),
                    "icao": landing_rwy.as_ref().and_then(|r| r.icao.as_ref()) }),
        );
        // The tower's answer (the clearance, and any correction of the
        // runway) is autoland's to read back and obey: it watches the party
        // line every tick and re-arms itself on the cleared end.
    }

    // Gate to gate: once autoland is down and stopped (it goes inactive after
    // "down and stopped, your aircraft"), taxi to PARKING on our own — request
    // the taxi-in and drive to the stand, the arrival mirror of the departure
    // taxi we ran at the start. Bounded so a balk/abort can't hang here, and
    // skipped unless we actually got the aircraft down on the ground (autoland
    // also goes inactive on a go-around, climbing away).
    if crate::autoland::ACTIVE.load(Relaxed) {
        taxi_in_after_landing(xp, config, voice, tools, false).await;
    }
    event("arrival_owns", json!({}));
    ACTIVE.store(false, SeqCst);
}

/// How long the arrival may stop closing on the fix the follower is steering
/// at before the mission gives up on it, seconds. Longer than the follower's
/// own per-leg stall watch (which rejoins twice before it stands itself down),
/// because this is the backstop under that, not a second opinion on it.
const ARRIVAL_STALL_SECS: f64 = 600.0;
/// Closure that restarts that clock, miles.
const ARRIVAL_PROGRESS_NM: f64 = 0.5;
/// Absolute ceiling on the arrival wait, seconds — three hours. Not a judgement
/// on the approach (the watch below is): a bound on a mission thread whose
/// follower has wedged ACTIVE without ticking.
const ARRIVAL_CAP_SECS: f64 = 3.0 * 3600.0;

/// The mission's watch on an arrival it has handed to the follower: a watchdog
/// on PROGRESS, not a clock.
///
/// It was a flat 1,800 seconds from the arm, and an approach armed far enough
/// out simply takes longer than that to fly. EGLL-EGPH, 2026-09-17: the
/// SimBrief route's last fix was RIBEL, 121 nm from the ILS 24 initial fix, so
/// the arrival armed there and the follower flew the straight line at 265 kt,
/// descending on its profile the whole way. At 4.2 nm to run — two minutes
/// from the fix — the 1,800 seconds expired, the mission returned, and its
/// scope cancelled the follower under it at 4,900 ft. Nothing flew the
/// aircraft for the next eleven minutes: the speed bled from 200 to 128 kt
/// through six minutes of airspeed warnings and the 737 sank into the ground
/// near Perth. A clock cannot tell a long arrival from a stuck one; closure
/// can, and closure is what the mission actually cares about.
#[derive(Default)]
pub(crate) struct ArrivalWatch {
    fix: Option<String>,
    best_nm: f64,
    best_at_s: f64,
}

impl ArrivalWatch {
    /// Feed the fix the follower is steering at and the miles to it. True once
    /// per stall: no new low for [`ARRIVAL_STALL_SECS`]. A new fix starts the
    /// judgement afresh (the distance jumps UP when a leg sequences), and a
    /// follower steering at no fix at all holds the clock rather than starting
    /// it — a to-altitude leg is the follower's own business.
    pub(crate) fn stalled(&mut self, fix: Option<&str>, dist_nm: Option<f64>, now_s: f64) -> bool {
        let (Some(fix), Some(d)) = (fix, dist_nm) else {
            self.fix = None;
            return false;
        };
        if self.fix.as_deref() != Some(fix) {
            self.fix = Some(fix.to_string());
            self.best_nm = d;
            self.best_at_s = now_s;
            return false;
        }
        if d < self.best_nm - ARRIVAL_PROGRESS_NM {
            self.best_nm = d;
            self.best_at_s = now_s;
            return false;
        }
        if now_s - self.best_at_s > ARRIVAL_STALL_SECS {
            self.best_nm = d;
            self.best_at_s = now_s;
            return true;
        }
        false
    }
}

/// The last stage of any mission that ends with a landing: once autoland has
/// rolled out and stopped, request the taxi-in and drive to the stand.
///
/// This is not tidiness, it is the runway. Autoland stops ON the strip and
/// hands back, and for a CIRCUIT nothing came after it — full auto armed the
/// circuit and exited, so the aircraft simply sat where it stopped. At
/// Southampton it held runway 02 for ninety seconds after touchdown while an
/// arrival closed from 2.6 nm to 0.5 nm: six `runway_incursion` records, and
/// the tower telling the lander "continue approach, traffic on the runway
/// ahead". It only moved because the pilot said "taxi to parking"
/// (2026-08-01). Vacating is part of the landing, so it belongs to whoever
/// owns the landing.
///
/// `wait_for_arm` is for the circuit, whose autoland is minutes away when the
/// mission is armed; the flight path calls this with autoland already running.
async fn taxi_in_after_landing(
    xp: &XPlaneClient,
    config: &Config,
    voice: &Option<Voice>,
    tools: &SubTools,
    wait_for_arm: bool,
) {
    use std::sync::atomic::Ordering::Relaxed;
    if wait_for_arm {
        // Wait for autoland to ARM — or for the mission to DIE first. The
        // condition used to be autoland alone, and a circuit cancelled with
        // "cancel the circuit" (which touches no full-auto flag) left this
        // wait holding full auto for its whole 900 s (audit, 2026-08-02).
        // The mission is alive while any of its flyers is: the circuit, the
        // published approach it may escalate to, or autoland itself.
        waiting("the landing to arm");
        wait_until(ARM_WAIT_SECS, 1000, || async {
            crate::autoland::ACTIVE.load(Relaxed)
                || !(crate::circuit::ACTIVE.load(Relaxed) || crate::procedure::ACTIVE.load(Relaxed))
        })
        .await;
        // The handoff clears the circuit's flag AROUND arming autoland, so a
        // "nothing flying" read gets a beat of grace before it means dead.
        let armed = crate::autoland::ACTIVE.load(Relaxed)
            || wait_until(5.0, 250, || async { crate::autoland::ACTIVE.load(Relaxed) }).await;
        if !armed {
            event("no_autoland", json!({}));
            return;
        }
    }
    if ABORT.load(SeqCst) {
        return;
    }
    // The panel's automation well reads the stage string, and full auto now
    // stays ACTIVE through the mission instead of dropping out at the arm —
    // so say where we actually are rather than leaving "circuit armed" up for
    // the whole flight.
    event("on_approach", json!({}));
    // A vacate drive follows this landing, so the rollout may hand off at taxi
    // pace instead of stopping dead on the strip — landing includes getting
    // off the runway, and it is one continuous manoeuvre, not a stop and a
    // fresh start (EGHI, 2026-08-02). Consumed by the rollout; cleared below
    // if the landing never happens, so it cannot go stale.
    crate::autoland::VACATE_FOLLOWS.store(true, Relaxed);
    waiting("the landing");
    let landed = wait_until(1800.0, 1000, || async {
        !crate::autoland::ACTIVE.load(Relaxed)
    })
    .await;
    // Autoland also goes inactive climbing away from a go-around, so being on
    // the ground is the half of "landed" that matters here.
    let on_ground = xp
        .get_float(crate::datarefs::ON_GROUND_ANY)
        .await
        .unwrap_or(0.0)
        > 0.5;
    // A wreck is "on the ground" too. The Gibraltar go-around hit the North
    // Front, the sim raised has_crashed, autoland read the stop as a rollout,
    // and this said "On the ground. I'll clear the runway", called for taxi,
    // and armed auto taxi on the wreckage (12:46Z, 2026-09-11). Fail open: a
    // failed read is not a crash.
    let crashed = xp
        .get_float(crate::datarefs::HAS_CRASHED)
        .await
        .unwrap_or(0.0)
        > 0.5;
    if !landed || !on_ground || crashed || ABORT.load(SeqCst) {
        // The landing never completed here — take the rolling-handoff offer
        // back, or a later standalone landing would hand off to nobody.
        crate::autoland::VACATE_FOLLOWS.store(false, Relaxed);
        event(
            "no_taxi_in",
            json!({ "landed": landed, "on_ground": on_ground, "crashed": crashed }),
        );
        return;
    }
    announce(
        voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Down. I'll get us off the runway and taxi to parking.",
            "Down. Vacating and taxiing us to parking.",
            "On the ground. I'll clear the runway and get us to the gate.",
        ]),
    );
    event("taxi_in", json!({}));
    crate::taxi_in::start(
        config.xplane_base_url.clone(),
        config.clone(),
        voice.clone(),
        tools.taxi.park_brake_valve_set,
        tools.taxi.prop.clone(),
        tools.taxi.pace,
        // We have just landed, so the first thing owed is the runway itself.
        crate::taxi_in::vacate_runway(),
    );
    RUN.record(sim::run::Outcome::HandedOff);
}

/// The arrival, briefed AT THE GATE: what the destination offers and what
/// full auto will therefore do there — a published approach, a straight-in
/// reconstructed from DME fixes, or a visual landing. The Gibraltar crash was
/// discovered 25 nm out, when the arrival was first built; nothing about it
/// needed the aircraft to be airborne. `Err` is a refusal: a destination the
/// airport data doesn't have, or one with no runway, is not a destination.
pub fn brief_arrival(
    apt_dat: &str,
    cifp: &str,
    fix_path: &str,
    dest: &str,
) -> Result<String, String> {
    let airport = crate::nav::find_airport(apt_dat, dest)
        .ok()
        .flatten()
        .ok_or_else(|| {
            format!("I can't find {dest} in the airport data, so I can't plan the arrival.")
        })?;
    if airport.runways.is_empty() {
        return Err(format!(
            "{} has no runways in the airport data.",
            airport.name
        ));
    }
    let (mut published, mut straight_in) = (Vec::new(), Vec::new());
    for r in &airport.runways {
        for id in [&r.ident_a, &r.ident_b] {
            let spoken = crate::circuit::spell_runway(id);
            if let Some(appch) = crate::nav::best_approach(cifp, dest, id) {
                published.push(format!(
                    "the {} on {spoken}",
                    crate::procedure::approach_kind(&appch)
                ));
            } else if !crate::nav::reconstruct_approach(fix_path, &airport, dest, id).is_empty() {
                straight_in.push(spoken);
            }
        }
    }
    let name = &airport.name;
    let mut parts = Vec::new();
    if !published.is_empty() {
        parts.push(format!("{name} has {}", published.join(", ")));
    }
    if !straight_in.is_empty() {
        parts.push(format!(
            "{}a straight-in on {}",
            if published.is_empty() {
                format!("{name} has ")
            } else {
                "and ".to_string()
            },
            straight_in.join(" and ")
        ));
    }
    Ok(if parts.is_empty() {
        format!("{name} has no published approach and no straight-in, so I'll land visually. The wind at the time picks the runway.")
    } else {
        format!(
            "{}. The wind at the time picks the runway.",
            parts.join(" ")
        )
    })
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

/// The inbound call to the destination's tower, once its station is the one
/// on the frequency (`swapped`), or regardless at the last moment before the
/// final report. The swap IS the region handoff: the frequency is already
/// ours and the readback would be a garbled call to the NEW tower, but a
/// discrete code still goes back to seven thousand.
async fn call_inbound(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    callsign: &Option<String>,
    khz: i64,
    swapped: bool,
) {
    if let Some(h) = crate::party_line::take_region_handoff() {
        obey_region_handoff(xp, ctx, voice, callsign, h, false).await;
    }
    crate::circuit::transmit(crate::circuit::radio_call(callsign, "inbound for landing"));
    INBOUND_MADE.store(true, SeqCst);
    // From here the destination's tower is OURS: the region's service-weighted
    // nearest-field pick may not hand the station to a closer field while we
    // fly its zone (London City took the station fifteen miles out and both
    // the final report and the taxi-in request went into the void, EGLL
    // 2026-09-17). Released by `release_arrival_station`.
    crate::station::pin_arrival_station(Some(khz));
    event(
        "inbound_call",
        json!({ "khz": khz, "station_swapped": swapped }),
    );
}

/// Release the arrival's hold on the station (see
/// [`crate::station::pin_arrival_station`]). Every path that ends a mission
/// calls this, so a pin can never outlive the arrival that set it: the
/// region's ordinary nearest-field handoffs resume for the next flight.
pub fn release_arrival_station() {
    crate::station::pin_arrival_station(None);
}

/// The wind the arrival picks its runway by, and where it came from: the
/// destination's METAR when there is one, else the aircraft's own wind
/// datarefs (the winds aloft where we are, the only wind without a report).
pub fn arrival_wind(
    metar: Option<(f64, f64)>,
    aircraft: Option<(f64, f64)>,
) -> (Option<(f64, f64)>, &'static str) {
    match (metar, aircraft) {
        (Some(w), _) => (Some(w), "metar"),
        (None, Some(w)) => (Some(w), "aircraft"),
        (None, None) => (None, "none"),
    }
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
        rwy_id: Some(crate::autoland::PlannedRunway::at(dest, &rwy_id)),
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
    waiting("departure clearance");
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
///
/// `accept_line_up` is the difference between the two calls this makes. Held
/// SHORT of the runway, a line-up-and-wait is the answer we're waiting for —
/// the tower has sequenced us and wants us on the strip, so we take it, read
/// it back, and drive on (the caller asks again from the runway). Already ON
/// the runway, only the takeoff clearance will do. Without the distinction a
/// sequenced departure waited out its whole budget behind the hold line while
/// the tower believed it was lined up (Heathrow 27L, 2026-07-27).
async fn ensure_takeoff_cleared(
    voice: &Option<Voice>,
    callsign: &Option<String>,
    read_back: &mut bool,
    accept_line_up: bool,
) -> bool {
    use crate::tower::State::{LinedUp, TakeoffCleared};
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
        let state = crate::party_line::clearance_state();
        if matches!(state, Some(TakeoffCleared))
            || (accept_line_up && matches!(state, Some(LinedUp)))
        {
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
    // Read back what we were actually given. A line-up-and-wait is its own
    // runway-critical item — echoing "cleared for takeoff" at it would earn a
    // "negative" from the readback enforcement and, worse, is not what we
    // were told to do. The takeoff readback stays one-shot across the whole
    // departure sequence; the line-up's is owed at the moment it's issued.
    let lined_up_only = matches!(crate::party_line::clearance_state(), Some(LinedUp));
    if let Some(rwy) = crate::party_line::clearance_runway() {
        if lined_up_only {
            crate::circuit::transmit(crate::circuit::radio_call(
                callsign,
                &format!("lining up runway {}", crate::circuit::spell_runway(&rwy)),
            ));
            event("readback_line_up", json!({ "runway": rwy }));
        } else if !*read_back {
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

/// Pavement the takeoff needs, metres: the profile's ground roll with a 1.4
/// factor (the POH figure is a test pilot's on a dry day), never under 500 m.
/// One rule for the line-up's fit test and the arm-time field check.
pub fn takeoff_needed_m(takeoff_roll_m: Option<f64>) -> f64 {
    takeoff_roll_m.map_or(500.0, |m| (m * 1.4).max(500.0))
}

/// What the arm-time preflight compares the plan against: the tanks and the
/// load as the sim has them NOW, the airframe's book figures, and the longest
/// runway at each end. `None`/0 = unknown, and an unknown is never a refusal.
#[derive(Debug, Clone, Default)]
pub struct PreflightFacts {
    pub fuel_kg: Option<f64>,
    pub payload_kg: Option<f64>,
    pub total_kg: Option<f64>,
    pub max_gross_lb: Option<f64>,
    pub takeoff_roll_m: Option<f64>,
    pub landing_distance_ft: Option<f64>,
    pub takeoff_distance_ft: Option<f64>,
    pub origin_longest_ft: f64,
    pub dest_longest_ft: f64,
}

const LB_PER_KG: f64 = 2.204_62;
/// A tank within this of the block is "as planned"; SimBrief rounds too.
const FUEL_MATCH_FRAC: f64 = 0.03;
/// Payload within this of the plan (and 20 kg) is "as planned".
const PAYLOAD_MATCH_FRAC: f64 = 0.10;
const PAYLOAD_MATCH_KG: f64 = 20.0;

/// The plan against the aircraft, BEFORE full auto starts an engine: the
/// destination's longest runway against the landing distance, this field's
/// against the takeoff roll, the all-up weight against the max takeoff weight,
/// the tanks against the plan's block and minimum takeoff fuel, and the
/// payload against the load sheet. `Err` is a refusal that names the number;
/// `Ok` is the brief (fuel and load, each said with its figures) for the
/// arming ack. Every figure the check lacks is skipped, never guessed: a plan
/// with no load sheet is briefed on fuel alone. The pilot asked "does the
/// copilot check the fuel and the runway" (2026-09-15); it did neither at the
/// arm, and found the fuel at the enroute caution and the runway at the hold.
pub fn preflight(fp: &crate::simbrief::FlightPlan, f: &PreflightFacts) -> Result<String, String> {
    use crate::flightplan::spoken_feet;
    // Fuel in the plan's own unit when it carries a load sheet, otherwise the
    // AIRFRAME's — never the app's avgas gallons, which is what a local plan
    // used to fall back to whatever it was flying; a weight in the plan's
    // unit, or pounds (the profile's `max_gross_lb`) when there is no sheet.
    let spoken_fuel = |kg: f64| match &fp.weights {
        Some(w) => w.spoken(kg),
        None => crate::aircraft::say_fuel_kg(kg),
    };
    let spoken_weight = |kg: f64| match &fp.weights {
        Some(w) => w.spoken(kg),
        None => crate::simbrief::spoken_weight(kg * LB_PER_KG, "pounds"),
    };

    // The far end first: a field we cannot land at is not a destination.
    if let Some(need) =
        crate::flightplan::runway_needed_ft(f.landing_distance_ft, f.takeoff_distance_ft)
    {
        if f.dest_longest_ft > 0.0 && f.dest_longest_ft < need {
            return Err(format!(
                "Negative. {}'s longest runway is {}, and we need {} to land. Not flying that.",
                fp.destination,
                spoken_feet(f.dest_longest_ft),
                spoken_feet(need)
            ));
        }
    }
    // This end: the pavement the roll needs, factored, or the line-up would
    // find out at the hold after the start and the taxi.
    let need_ft = takeoff_needed_m(f.takeoff_roll_m) * M_TO_FT;
    if f.takeoff_roll_m.is_some() && f.origin_longest_ft > 0.0 && f.origin_longest_ft < need_ft {
        return Err(format!(
            "Negative. {}'s longest runway is {}, and the takeoff needs {}. Not flying that.",
            fp.origin,
            spoken_feet(f.origin_longest_ft),
            spoken_feet(need_ft)
        ));
    }
    // Weight: the plan's max takeoff weight, else the profile's max gross.
    let max_tow_kg = fp
        .weights
        .as_ref()
        .map(|w| w.max_tow_kg)
        .filter(|m| *m > 0.0)
        .or(f.max_gross_lb.map(|lb| lb / LB_PER_KG));
    if let (Some(total), Some(max)) = (f.total_kg, max_tow_kg) {
        if total > max * 1.005 {
            // SimBrief's max_tow is the OPERATIONAL limit, the least of the
            // structural MTOW, the landing weight plus the trip's burn and the
            // zero-fuel weight plus fuel: on a short leg it sits far under the
            // placard (LFLL-EGLL, 2026-09-17: 153,280 lb against a 174,200 lb
            // airframe, and the pilot asked whether that was right). Name it
            // as the plan's, and when the load sheet says why, say that too:
            // the excess there was 9.4 t of payload over the OFP's, not fuel.
            let from_plan = fp.weights.as_ref().is_some_and(|w| w.max_tow_kg > 0.0);
            let mut why = format!(
                "Negative. We're at {}, over the {} of {}.",
                spoken_weight(total),
                if from_plan {
                    "plan's takeoff limit"
                } else {
                    "max takeoff weight"
                },
                spoken_weight(max)
            );
            if let (Some(w), Some(p)) = (fp.weights.as_ref(), f.payload_kg) {
                if w.payload_kg > 0.0
                    && p - w.payload_kg > PAYLOAD_MATCH_KG.max(w.payload_kg * PAYLOAD_MATCH_FRAC)
                {
                    why.push_str(&format!(
                        " The aircraft is loaded with {} of payload against the plan's {}.",
                        w.spoken(p),
                        w.spoken(w.payload_kg)
                    ));
                }
            }
            why.push_str(" Not flying that.");
            return Err(why);
        }
    }
    let mut lines = Vec::new();
    // Fuel: below the minimum takeoff fuel is a refusal; under the block but
    // above the minimum is said with both numbers; at the block is a check.
    if let Some(fuel) = f.fuel_kg.filter(|_| fp.block_fuel_kg > 0.0) {
        let min = fp.min_takeoff_fuel_kg;
        if min > 0.0 && fuel < min * (1.0 - FUEL_MATCH_FRAC) {
            return Err(format!(
                "Negative. {} of fuel aboard, and the plan needs at least {} to go. Fuel up and say it again.",
                spoken_fuel(fuel),
                spoken_fuel(min)
            ));
        }
        if fuel < fp.block_fuel_kg * (1.0 - FUEL_MATCH_FRAC) {
            lines.push(format!(
                "Fuel {} aboard against a {} block, above the minimum, so we can go.",
                spoken_fuel(fuel),
                spoken_fuel(fp.block_fuel_kg)
            ));
        } else {
            lines.push(format!(
                "Fuel {} aboard for the plan's {}.",
                spoken_fuel(fuel),
                spoken_fuel(fp.block_fuel_kg)
            ));
        }
    }
    // The load sheet: who and what is aboard, and whether the sim agrees.
    if let Some(w) = &fp.weights {
        let mut load = match (w.pax, w.cargo_kg > 0.0) {
            (0, false) => "No passengers or cargo".to_string(),
            (0, true) => format!("{} of cargo", w.spoken(w.cargo_kg)),
            (n, false) => format!("{n} passenger{}", if n == 1 { "" } else { "s" }),
            (n, true) => format!(
                "{n} passenger{} and {} of cargo",
                if n == 1 { "" } else { "s" },
                w.spoken(w.cargo_kg)
            ),
        };
        match f.payload_kg {
            Some(p) if w.payload_kg > 0.0 => {
                let off = (p - w.payload_kg).abs();
                if off > PAYLOAD_MATCH_KG && off > w.payload_kg * PAYLOAD_MATCH_FRAC {
                    load.push_str(&format!(
                        " on the plan, but the aircraft is loaded with {} against the plan's {}",
                        w.spoken(p),
                        w.spoken(w.payload_kg)
                    ));
                } else {
                    load.push_str(", loaded as planned");
                }
            }
            _ => {}
        }
        load.push('.');
        lines.push(load);
    }
    Ok(lines.join(" "))
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
    let needed_m = takeoff_needed_m(tools.line_up.takeoff_roll_m);
    let apt = crate::tools::autoland::apt_dat_path();
    let cands = crate::autoland::nearby_runways(&apt, lat, lon, 3.0).unwrap_or_default();
    crate::autotakeoff::runway_fit(&cands, lat, lon, psi, needed_m).is_some()
}

#[cfg(test)]
mod tests {
    #[test]
    fn full_auto_requires_both_instruments_to_confirm_ground_contact() {
        use super::confirmed_ground;
        assert!(confirmed_ground(Ok(0.0), Ok(1.0)));
        assert!(!confirmed_ground(Ok(500.0), Ok(0.0)));
        assert!(!confirmed_ground(Ok(0.0), Ok(0.0))); // just lifted off
        assert!(!confirmed_ground(Ok(500.0), Ok(1.0))); // conflicting readings
        assert!(!confirmed_ground(
            Err(anyhow::anyhow!("AGL failed")),
            Ok(1.0)
        ));
        assert!(!confirmed_ground(
            Ok(0.0),
            Err(anyhow::anyhow!("contact failed"))
        ));
        assert!(!confirmed_ground(
            Err(anyhow::anyhow!("AGL failed")),
            Err(anyhow::anyhow!("contact failed"))
        ));
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(!confirmed_ground(Ok(invalid), Ok(1.0)));
            assert!(!confirmed_ground(Ok(0.0), Ok(invalid)));
        }
    }

    /// A 172 plan LXGB-LEAX as SimBrief sent it (2026-09-15), in kilos.
    fn plan(weights: bool) -> crate::simbrief::FlightPlan {
        crate::simbrief::FlightPlan {
            source: crate::simbrief::PlanSource::SimBrief,
            origin: "LXGB".into(),
            destination: "LEAX".into(),
            alternate: String::new(),
            aircraft: "C172".into(),
            cruise_alt_ft: 3500.0,
            distance_nm: 45.0,
            ete_min: 35.0,
            trip_fuel_kg: 10.7 * 2.72,
            reserve_kg: 5.7 * 2.72,
            route: "DCT".into(),
            wind_comp_kts: 0.0,
            wind_dir: 0.0,
            wind_spd_kts: 0.0,
            tod_to_dest_nm: 8.0,
            waypoints: Vec::new(),
            block_fuel_kg: 109.0,
            min_takeoff_fuel_kg: 103.0,
            weights: weights.then(|| crate::simbrief::PlanWeights {
                pax: 3,
                bags: 1,
                cargo_kg: 30.4,
                payload_kg: 268.5,
                est_tow_kg: 1111.0,
                max_tow_kg: 1111.0,
                est_ldw_kg: 1082.0,
                max_ldw_kg: 1111.0,
                spoken_lbs: true,
            }),
        }
    }

    /// The 172's book and both fields, all fine: fuel at the block, loaded
    /// as planned, Gibraltar's 1,800 m and Axarquia's 1,000 m.
    fn facts() -> super::PreflightFacts {
        super::PreflightFacts {
            fuel_kg: Some(109.0),
            payload_kg: Some(270.0),
            total_kg: Some(1100.0),
            max_gross_lb: Some(2558.0),
            takeoff_roll_m: Some(1630.0 * 0.3048),
            landing_distance_ft: Some(1335.0),
            takeoff_distance_ft: Some(1630.0),
            origin_longest_ft: 5900.0,
            dest_longest_ft: 3280.0,
        }
    }

    #[test]
    fn a_plan_that_fits_is_briefed_with_its_fuel_and_its_load() {
        let brief = super::preflight(&plan(true), &facts()).expect("fits");
        assert_eq!(
            brief,
            "Fuel 240 pounds aboard for the plan's 240 pounds. \
             3 passengers and 70 pounds of cargo, loaded as planned."
        );
        // A local plan has no load sheet and speaks the app's gallons.
        let brief = super::preflight(&plan(false), &facts()).expect("fits");
        assert_eq!(brief, "Fuel 40 gallons aboard for the plan's 40 gallons.");
        // Nothing readable, nothing said, nothing refused.
        let brief =
            super::preflight(&plan(true), &super::PreflightFacts::default()).expect("no facts");
        assert_eq!(brief, "3 passengers and 70 pounds of cargo.");
    }

    #[test]
    fn a_runway_too_short_at_either_end_is_refused_at_the_gate() {
        let short_dest = super::PreflightFacts {
            dest_longest_ft: 1200.0,
            ..facts()
        };
        let why = super::preflight(&plan(true), &short_dest).expect_err("refused");
        assert_eq!(
            why,
            "Negative. LEAX's longest runway is 1,200 feet, and we need 1,335 feet to land. Not flying that."
        );
        // The takeoff needs the roll x 1.4: 1,630 ft -> 2,282 ft.
        let short_origin = super::PreflightFacts {
            origin_longest_ft: 2000.0,
            ..facts()
        };
        let why = super::preflight(&plan(true), &short_origin).expect_err("refused");
        assert!(
            why.starts_with(
                "Negative. LXGB's longest runway is 2,000 feet, and the takeoff needs 2,282 feet."
            ),
            "{why}"
        );
        // A field the data lacks (0) is not a refusal: brief_arrival owns that.
        let unknown = super::PreflightFacts {
            origin_longest_ft: 0.0,
            dest_longest_ft: 0.0,
            ..facts()
        };
        assert!(super::preflight(&plan(true), &unknown).is_ok());
    }

    #[test]
    fn overweight_and_under_fuelled_are_refused_with_the_number() {
        let heavy = super::PreflightFacts {
            total_kg: Some(1160.0),
            ..facts()
        };
        let why = super::preflight(&plan(true), &heavy).expect_err("refused");
        assert_eq!(why, "Negative. We're at 2,560 pounds, over the plan's takeoff limit of 2,450 pounds. Not flying that.");
        // Overweight because the cabin is heavier than the load sheet: the
        // refusal says so, since that is what the pilot can change.
        let heavy_cabin = super::PreflightFacts {
            total_kg: Some(1160.0),
            payload_kg: Some(400.0),
            ..facts()
        };
        let why = super::preflight(&plan(true), &heavy_cabin).expect_err("refused");
        assert!(why.contains("plan's takeoff limit of 2,450 pounds. The aircraft is loaded with 880 pounds of payload against the plan's 590 pounds."), "{why}");
        // No load sheet: the profile's max gross (2,558 lb = 1,160 kg) is the limit.
        let heavy = super::PreflightFacts {
            total_kg: Some(1180.0),
            ..facts()
        };
        let why = super::preflight(&plan(false), &heavy).expect_err("refused");
        assert_eq!(why, "Negative. We're at 2,600 pounds, over the max takeoff weight of 2,560 pounds. Not flying that.");

        // Under the minimum takeoff fuel: refused, with both figures.
        let dry = super::PreflightFacts {
            fuel_kg: Some(80.0),
            ..facts()
        };
        let why = super::preflight(&plan(true), &dry).expect_err("refused");
        assert_eq!(why, "Negative. 180 pounds of fuel aboard, and the plan needs at least 230 pounds to go. Fuel up and say it again.");
        // Between the minimum and the block: said, not refused.
        let light = super::PreflightFacts {
            fuel_kg: Some(104.0),
            ..facts()
        };
        let brief = super::preflight(&plan(true), &light).expect("goes");
        assert!(brief.starts_with("Fuel 230 pounds aboard against a 240 pounds block, above the minimum, so we can go."), "{brief}");
    }

    #[test]
    fn a_load_the_sim_disagrees_with_is_said_not_refused() {
        // Two hundred kilos aboard against the plan's 268: off by more than
        // 10 percent and 20 kg, so the pilot hears both numbers.
        let light = super::PreflightFacts {
            payload_kg: Some(200.0),
            ..facts()
        };
        let brief = super::preflight(&plan(true), &light).expect("goes");
        assert!(brief.ends_with("3 passengers and 70 pounds of cargo on the plan, but the aircraft is loaded with 440 pounds against the plan's 590 pounds."), "{brief}");
        // A lone pilot with no cargo.
        let mut solo = plan(true);
        let w = solo.weights.as_mut().unwrap();
        w.pax = 0;
        w.cargo_kg = 0.0;
        let brief = super::preflight(&solo, &facts()).expect("goes");
        assert!(
            brief.ends_with("No passengers or cargo, loaded as planned."),
            "{brief}"
        );
    }

    /// LEJR-LXGB, 2026-09-14: the aircraft's wind at 7,000 ft set up 27; the
    /// field's report said 090 at 21.
    #[test]
    fn the_field_report_outranks_the_wind_aloft_for_the_runway_pick() {
        let metar = Some((90.0, 21.0));
        let aloft = Some((260.0, 5.0));
        assert_eq!(super::arrival_wind(metar, aloft), (metar, "metar"));
        assert_eq!(super::arrival_wind(None, aloft), (aloft, "aircraft"));
        assert_eq!(super::arrival_wind(None, None), (None, "none"));
    }

    use super::departure_check_in;

    // The arrival is briefed at the gate. Gibraltar, as the data has it: no
    // coded approach, DME fixes on 27's centreline, 09's off it.
    #[test]
    fn the_gate_brief_says_what_the_destination_offers() {
        use std::io::Write;
        let dir = std::env::temp_dir().join("xvl-brief");
        std::fs::create_dir_all(&dir).unwrap();
        let apt = dir.join("apt.dat");
        let mut f = std::fs::File::create(&apt).unwrap();
        writeln!(f, "1 11 0 0 LXGB Gibraltar").unwrap();
        writeln!(f, "100 46 0 0 0 0 0 0 09 36.1509000 -5.3581000 0 0 0 0 0 0 27 36.1515000 -5.3412000 0 0 0 0 0 0").unwrap();
        f.flush().unwrap();
        let fix = dir.join("earth_fix.dat");
        let mut f = std::fs::File::create(&fix).unwrap();
        writeln!(
            f,
            "36.157361111 -5.240083333 27D05 LXGB LX 5906505 RWY27 D05"
        )
        .unwrap();
        writeln!(
            f,
            "36.104666667 -5.405333333 09D05 LXGB LX 5906505 RWY09 D05"
        )
        .unwrap();
        f.flush().unwrap();
        let cifp = dir.join("no-cifp");
        let brief = super::brief_arrival(
            apt.to_str().unwrap(),
            cifp.to_str().unwrap(),
            fix.to_str().unwrap(),
            "LXGB",
        )
        .expect("a brief");
        assert_eq!(
            brief,
            "Gibraltar has a straight-in on two seven. The wind at the time picks the runway."
        );
        // A destination the data doesn't have is a refusal, at the gate.
        let why = super::brief_arrival(
            apt.to_str().unwrap(),
            cifp.to_str().unwrap(),
            fix.to_str().unwrap(),
            "ZZZZ",
        )
        .expect_err("refused");
        assert!(why.contains("ZZZZ"), "{why}");
    }

    // The first call to Departure has to read as a CHECK-IN to the station
    // (a readback carrying "passing"/"climbing", never a request), or the
    // departure controller answers "say again" instead of "radar contact".
    #[test]
    fn the_departure_check_in_is_shaped_to_earn_radar_contact() {
        let cs = Some("November four five X-ray Sierra".to_string());
        let call = departure_check_in(&cs, "Malaga Departure", 1960.0, 4000.0);
        assert_eq!(
            call,
            "Malaga Departure, November four five X-ray Sierra, passing two thousand climbing four thousand"
        );
        assert!(crate::tower::is_readback(&call), "{call}");
        assert_eq!(
            crate::tower::parse_intent(&call),
            crate::tower::Intent::Unknown,
            "{call}"
        );
        // Already at the planned altitude: no invented climb.
        let level = departure_check_in(&None, "Malaga Departure", 3980.0, 4000.0);
        assert_eq!(level, "Malaga Departure, passing four thousand");
    }
}

#[cfg(test)]
mod region_handoff_tests {
    use super::region_readback;
    use crate::party_line::RegionHandoff;

    /// The readback carries a readback word the tower FSM recognises, the
    /// station and the frequency; the squawk only when one was ordered.
    #[test]
    fn readback_is_readback_shaped() {
        let cs = Some("November one seven two Sierra Papa".to_string());
        let h = RegionHandoff {
            to: "Malaga Tower".into(),
            khz: 118_100,
            squawk_vfr: true,
        };
        assert_eq!(
            region_readback(&cs, &h),
            "November one seven two Sierra Papa, squawking seven thousand, Malaga Tower on one one eight decimal one"
        );
        assert!(crate::tower::is_readback(&region_readback(&cs, &h)));
        let h = RegionHandoff {
            squawk_vfr: false,
            ..h
        };
        let line = region_readback(&None, &h);
        assert_eq!(line, "roger, Malaga Tower on one one eight decimal one");
        assert!(!line.contains("thousand"));
        assert!(crate::tower::is_readback(&line));
    }
}

#[cfg(test)]
mod inbound_tests {
    use super::{inbound_due, INBOUND_CALL_NM};

    /// LEJR-LEGR, 2026-09-15: the region handed the flight to Granada Tower
    /// 20 nm out and the inbound waited for the last fix, a mile from the
    /// runway. Owed on the destination's own frequency inside the call
    /// distance; never on a transit field's, never twice.
    #[test]
    fn inbound_owed_on_the_destination_tower_inside_the_call_distance() {
        let dest = Some(118_850);
        assert!(
            !inbound_due(dest, dest, INBOUND_CALL_NM + 5.0, false, false),
            "still far out"
        );
        assert!(
            inbound_due(dest, dest, INBOUND_CALL_NM, false, false),
            "at the call distance"
        );
        assert!(inbound_due(dest, dest, 3.0, false, false));
        assert!(!inbound_due(dest, dest, 3.0, false, true), "already made");
        // Tangier held the frequency mid-strait: not the tower we are landing at.
        assert!(
            !inbound_due(dest, Some(121_700), 3.0, false, false),
            "another region's tower"
        );
        assert!(
            !inbound_due(dest, None, 3.0, false, false),
            "no station on the air"
        );
        assert!(
            !inbound_due(None, dest, 3.0, false, false),
            "no tower row to call"
        );
    }

    /// The tower's "say callsign and intentions" is answered at any range,
    /// but only by the tower we are inbound to.
    #[test]
    fn a_challenge_from_the_destination_tower_earns_the_inbound_now() {
        let dest = Some(118_850);
        assert!(inbound_due(dest, dest, 30.0, true, false));
        assert!(
            !inbound_due(dest, dest, 30.0, true, true),
            "already made: no second call"
        );
        assert!(
            !inbound_due(dest, Some(121_700), 2.0, true, false),
            "a transit field's question"
        );
    }
}

#[cfg(test)]
mod squawk_tests {
    use super::squawk_owed;

    /// The Malaga clearance came back after the wait gave up: the code is
    /// owed until the transponder shows it, and not once it does.
    #[test]
    fn a_late_squawk_is_owed_until_dialed() {
        assert_eq!(squawk_owed(None, Some(7000.0)), None);
        assert_eq!(squawk_owed(Some(4503), Some(7000.0)), Some(4503));
        assert_eq!(squawk_owed(Some(4503), None), Some(4503));
        assert_eq!(squawk_owed(Some(4503), Some(4503.0)), None);
    }
}

#[cfg(test)]
mod arrival_watch_tests {
    use super::{ArrivalWatch, ARRIVAL_STALL_SECS};

    /// The EGPH arrival, flown as it was flown: armed 121 nm from CI24 and
    /// closing at the 4.0 nm a minute the blackbox measured, a leg that takes
    /// half an hour. The old flat 1,800 s killed it 4 nm short; a watch on
    /// closure never fires, because the aircraft was doing exactly what it was
    /// told the whole way.
    #[test]
    fn a_long_arrival_that_keeps_closing_never_stalls() {
        let mut w = ArrivalWatch::default();
        let mut dist = 121.2;
        let mut t = 0.0;
        while dist > 0.5 {
            assert!(
                !w.stalled(Some("CI24"), Some(dist), t),
                "closing at {dist:.1} nm after {t:.0}s"
            );
            dist -= 4.0 / 60.0;
            t += 1.0;
        }
        assert!(t > 1800.0, "the leg really is longer than the old deadline");
    }

    /// An arrival that has stopped getting anywhere IS a give-up — but only
    /// after the follower's own rejoins have had their go.
    #[test]
    fn an_arrival_that_stops_closing_stalls_once() {
        let mut w = ArrivalWatch::default();
        assert!(!w.stalled(Some("CI24"), Some(12.0), 0.0));
        assert!(!w.stalled(Some("CI24"), Some(12.0), ARRIVAL_STALL_SECS - 1.0));
        assert!(w.stalled(Some("CI24"), Some(12.0), ARRIVAL_STALL_SECS + 1.0));
        // Fires once per stall: the clock restarts behind it.
        assert!(!w.stalled(Some("CI24"), Some(12.0), ARRIVAL_STALL_SECS + 2.0));
    }

    /// A sequenced leg jumps the distance UP. That is progress, not a stall.
    #[test]
    fn a_new_fix_is_judged_afresh() {
        let mut w = ArrivalWatch::default();
        assert!(!w.stalled(Some("CI24"), Some(0.4), 0.0));
        assert!(!w.stalled(Some("FI24"), Some(6.8), 1.0));
        assert!(!w.stalled(Some("FI24"), Some(6.8), ARRIVAL_STALL_SECS - 1.0));
        assert!(w.stalled(Some("FI24"), Some(6.8), ARRIVAL_STALL_SECS + 2.0));
    }

    /// A follower steering at no fix (a to-altitude leg, or between legs) is
    /// the follower's own business: hold the clock, don't start one.
    #[test]
    fn no_fix_holds_the_clock() {
        let mut w = ArrivalWatch::default();
        assert!(!w.stalled(None, None, 0.0));
        assert!(!w.stalled(None, None, ARRIVAL_STALL_SECS * 3.0));
        assert!(!w.stalled(Some("CI24"), Some(9.0), ARRIVAL_STALL_SECS * 3.0 + 1.0));
    }
}
