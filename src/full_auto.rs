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
    /// The planned SID's minimum altitudes for the departure runway, the ones
    /// on this route (`nav::sid_minimums`). Empty for a resume or a plan with
    /// no SID.
    pub climb_minimums: Vec<crate::nav::ClimbMinimum>,
    /// The runway the planned SID departs from, asked for at the taxi request.
    pub departure_rwy: Option<String>,
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
/// The fastest a give-up may set the PARKING brake. The taxi governor only
/// runs at taxi speeds, so this is belt and braces against a future caller:
/// a park brake is a chock, not a stopping device.
const HAND_BACK_BRAKE_MAX_KTS: f64 = 25.0;

async fn give_up(
    xp: &XPlaneClient,
    valve_set: Option<f64>,
    voice: &Option<Voice>,
    line: &str,
    reason: &str,
) {
    RUN.fail(reason);
    ACTIVE.store(false, SeqCst);
    // "Your aircraft" means every lever: the mixture manager stands down with
    // the rest, never quietly re-trimming under the pilot's hand.
    crate::tools::engine::MIX_MANAGED.store(false, SeqCst);
    event("give_up", json!({ "reason": reason }));
    // Held before it is said: the line claims the pilot has the aircraft, so
    // whatever is going to hold it should already be holding it.
    hand_back_brakes(xp, valve_set).await;
    clear_stage();
    announce(voice, Priority::Caution, line);
}

/// ...and on the ground it means the PARKING BRAKE holds the aircraft, not a
/// control loop of ours.
///
/// A taxi that reaches its holding point stops there with the governor still
/// running, target zero, squeezing the wheel brakes. That is deliberate, so
/// the next "go" is instant, and the line it says is "holding position, brakes
/// set" - but nothing ever turned it into a real parking brake when the
/// MISSION ended. At Le Bourget the line-up refused, full auto said "your
/// aircraft", and the governor went on holding the 737 for twenty-seven
/// minutes: 3,736 more ticks of Web API load the sim did not get, `park` still
/// 0.0 in every state record, and a proportional brake squeeze the pilot
/// cannot find or release (the governor's own exit path says exactly that).
/// CLAUDE.md has had the rule since the first orchestrator - brakes set for
/// every wait, and every abort or give-up path sets them too - and this was
/// the path that did not.
///
/// Only while the TAXI governor is the thing holding us, so a give-up at
/// rolling speed or in the air can never slam a parking brake on: the flavour
/// check is `taxi_hold_active`, and the governor's own next tick then sees the
/// brake, closes the throttle, stands down and releases its own squeeze.
async fn hand_back_brakes(xp: &XPlaneClient, valve_set: Option<f64>) {
    hand_back_brakes_if(xp, crate::tools::speedhold::taxi_hold_active(), valve_set).await;
}

/// [`hand_back_brakes`] with the governor's flavour handed in, so the gates are
/// a plain `cargo test` against the fake sim instead of a process-wide static.
async fn hand_back_brakes_if(
    xp: &dyn crate::xplane::XPlane,
    taxi_holding: bool,
    valve_set: Option<f64>,
) {
    if !taxi_holding {
        return;
    }
    let (ground, gs) = tokio::join!(
        xp.get_float(crate::datarefs::ON_GROUND_ANY),
        xp.get_float(crate::autotaxi::GS_MS),
    );
    if !matches!(ground, Ok(g) if g > 0.5) {
        return;
    }
    // A failed read is no measurement, and a parking brake is not a brake to
    // set at a speed we could not confirm.
    if !matches!(gs, Ok(v) if v.is_finite() && v.abs() * MS_TO_KTS < HAND_BACK_BRAKE_MAX_KTS) {
        return;
    }
    let held = crate::tools::controls::drive_park_brake(xp, true, valve_set)
        .await
        .unwrap_or(false);
    // Recorded, not `event`: the mission is over, so this must not reappear as
    // a stage on the panel the moment `clear_stage` has run.
    crate::blackbox::record(
        "full_auto",
        json!({ "stage": "hand_back_brakes", "detail": { "held": held } }),
    );
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

/// Clearance first, the real order: the zone clearance from the home tower,
/// its squawk dialled and the clearance read back. Only with a simulated
/// tower to answer (real/external ATC and copilot-only leave the clearance
/// to the pilot), and only if we don't already hold a squawk. Shared by every
/// mission that leaves a towered field (full auto before the taxi, the
/// helicopter route before its departure call): the S-76 called "ready for
/// departure" with no zone clearance and flew the zone that way (EDVW,
/// 2026-09-23).
pub(crate) async fn get_zone_clearance(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    callsign: &Option<String>,
) {
    if !crate::mode::atc_enabled() || crate::party_line::assigned_squawk().is_some() {
        return;
    }
    if let Some(khz) = crate::party_line::home_tower_khz() {
        let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
    }
    announce(
        voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Requesting clearance.",
            "Getting our clearance.",
            "Calling for our clearance.",
        ]),
    );
    crate::circuit::transmit(crate::circuit::radio_call(callsign, "request clearance"));
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
    ensure_squawk(xp, ctx, voice).await;
    // The clearance is read back like a real one: the zone cap, the
    // QNH and the squawk (the tower suggests the words, enforces
    // nothing, and answers a correct one with silence). Never done
    // before 2026-09-15: the squawk was set and the taxi asked for.
    if let Some(rb) = crate::party_line::readback_owed() {
        crate::circuit::transmit(rb.trim_end_matches('.').to_string());
        event("clearance_readback", json!({}));
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
    // The signs follow the flight through ten thousand, both ways.
    let mut signs = crate::tools::signs::Watch::default();
    loop {
        if ABORT.load(SeqCst) {
            return false;
        }
        if let Ok(alt_ft) = xp.get_float(ALT_IND_FT).await {
            if let Some(on) = signs.tick(alt_ft) {
                if crate::tools::signs::set(xp, on).await == Some(true) {
                    announce(
                        voice,
                        Priority::Advisory,
                        if on {
                            "Seat belt signs on."
                        } else {
                            "Seat belt signs off."
                        },
                    );
                }
            }
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
            obey_departure_handoff(xp, callsign, to, khz, climb_to_ft).await;
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

/// Obey the tower's after-takeoff handoff: read it back on the tower's
/// frequency, retune, check in with the climb. Shared with the helicopter
/// route, which never took one (EDVW, 2026-09-23).
pub(crate) async fn obey_departure_handoff(
    xp: &XPlaneClient,
    callsign: &Option<String>,
    to: String,
    khz: i64,
    climb_to_ft: f64,
) {
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
pub(crate) async fn obey_region_handoff(
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

/// Is the published taxi route one WE can drive from where we are standing?
///
/// A departure route (not a taxi-in to a stand) whose path passes within
/// 200 m of us. The same test gates the initial check and the wait for a new
/// clearance — they used to differ, and the weaker one accepted the stale
/// route it had just rejected (Edinburgh, 2026-09-20).
async fn taxi_route_is_ours(xp: &XPlaneClient) -> bool {
    if crate::party_line::taxi_to_stand() {
        return false;
    }
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
        // Position unknown: trust the route rather than re-request in a loop
        // we cannot verify our way out of.
        _ => crate::party_line::with_taxi_route(|r| r.is_some()),
    }
}

async fn fly(xp: XPlaneClient, config: Config, voice: Option<Voice>, tools: SubTools) {
    // A new mission starts owing nothing to the last one's destination: a pin
    // left by the previous arrival would keep the region from ever handing
    // this departure on (see `release_arrival_station`).
    release_arrival_station();
    macro_rules! bail_if_abort {
        () => {
            if ABORT.load(SeqCst) {
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "Cancelled. Your aircraft.",
                    "abort",
                )
                .await;
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
            &xp,
            tools.taxi.park_brake_valve_set,
            &voice,
            "Full auto needs confirmed ground contact. Say navigate to follow the plan, fly the ILS, or land for me when airborne.",
            "ground_unconfirmed",
        )
        .await;
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
                &xp,
                tools.taxi.park_brake_valve_set,
                &voice,
                "I don't have an engine start for this aircraft. Get the engines running and say full auto again.",
                "no_start_sequence",
            )
            .await;
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
        // Seat belt signs on before anything turns (see `tools::signs`).
        if let Some(ok) = crate::tools::signs::set(&xp, true).await {
            event("seat_belt_signs", json!({ "on": true, "ok": ok }));
        }
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
                &xp,
                tools.taxi.park_brake_valve_set,
                &voice,
                "The engine didn't catch. Full auto is off, your aircraft.",
                "no_engine",
            )
            .await;
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

    // The after-start flow's last item: the takeoff configuration, set with
    // the whole taxi ahead of it, not during the roll (see `takeoff_config`).
    // The takeoff confirms it before the brakes come off and there selects
    // the level its caller hands down; on the ground and engines running
    // only, so an airborne resume is never reconfigured for a takeoff.
    if !matches!(tools.mission, Mission::Resume(_)) && engine_running(&xp).await {
        let cfg = crate::takeoff_config::set_for_takeoff(
            &xp,
            tools.takeoff.takeoff_flap_ratio,
            None,
            false,
        )
        .await;
        if let Some(line) = crate::takeoff_config::line("Takeoff config", &cfg) {
            announce(&voice, Priority::Advisory, &line);
        }
        event(
            "takeoff_config",
            json!({"set": cfg.set, "not_taken": cfg.not_taken}),
        );
    }
    bail_if_abort!();

    // The cabin's air, on EVERY mission including an airborne resume: the
    // bleed that pressurises and the crew oxygen, where the airframe declares
    // them. It used to ride the engine start alone, so a departure with the
    // engine already running climbed an unpressurised SF50 to FL260 and the
    // pilot blacked out (EDLP-LOWI, 2026-09-24). Read first, left alone in
    // state, read back.
    if engine_running(&xp).await {
        let air = crate::tools::cabin_air::ensure_on(&xp).await;
        if let Some(line) = air.line() {
            announce(&voice, Priority::Advisory, &line);
        }
        event(
            "cabin_air",
            json!({"bleed": air.bleed, "oxygen": air.oxygen, "changed": air.changed()}),
        );
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
        let priority = if sync.took() {
            Priority::Advisory
        } else {
            Priority::Warning
        };
        announce(&voice, priority, &sync.line());
        event(
            "altimeter_set",
            json!({ "inhg": (sync.now * 100.0).round() / 100.0, "was": (sync.was * 100.0).round() / 100.0, "want": (sync.want * 100.0).round() / 100.0, "took": sync.took() }),
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
        get_zone_clearance(&xp, &ctx, &voice, &callsign).await;
        bail_if_abort!();
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
        let route_is_ours = taxi_route_is_ours(&xp).await;
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
            // The planned departure's runway, asked for (the tower honours it
            // unless the wind really favours another end): a SID exists for
            // one runway, and in calm air the pick could have been the other.
            let ask = match &tools.mission {
                Mission::Flight(r) => r.departure_rwy.as_deref().map(|rwy| {
                    format!("request taxi, runway {}", crate::circuit::spell_runway(rwy))
                }),
                _ => None,
            };
            crate::circuit::transmit(crate::circuit::radio_call(
                &callsign,
                ask.as_deref().unwrap_or("request taxi"),
            ));
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
            // Wait for a route that is OURS, by the same test that just
            // rejected the old one — not merely for "a route exists".
            //
            // The stale route is still published while we ask for a new one
            // (the party line is the world's to write; the crew cannot
            // retire it, and should not be able to). So the weaker condition
            // was satisfied by the very route it had just announced as
            // stale: at Edinburgh the request went out and `at_hold`,
            // `request_departure` and the line-up all fired 0.2 s later, on a
            // route starting 180 m away that the taxi drive then refused
            // (2026-09-20). Six stages in a fifth of a second is not a taxi.
            let got = wait_until(180.0, 1000, || async { taxi_route_is_ours(&xp).await }).await;
            if !got {
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "No taxi clearance came back. Tune the tower and request taxi, then say full auto again.",
                    "no_taxi_clearance",
                )
                .await;
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

        // The cleared runway against the planned departure: a SID exists for
        // its runway, and taking off on another would fly its fixes backwards
        // (at LOWI a turn back through the valley). Said and handed back here,
        // on the ramp, with the brakes on, never discovered in the climb.
        if let Some(why) = departure_runway_mismatch(
            match &tools.mission {
                Mission::Flight(r) => r.departure_rwy.as_deref(),
                _ => None,
            },
            crate::party_line::clearance_runway().as_deref(),
        ) {
            event("departure_runway_mismatch", json!({ "why": why }));
            give_up(
                &xp,
                tools.taxi.park_brake_valve_set,
                &voice,
                &format!("{why} Your aircraft."),
                "departure_runway_mismatch",
            )
            .await;
            return;
        }

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
                &xp,
                tools.taxi.park_brake_valve_set,
                &voice,
                "The taxi didn't finish. Your aircraft.",
                "taxi_stalled",
            )
            .await;
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
                            &xp,
                            tools.taxi.park_brake_valve_set,
                            &voice,
                            "I can't read our position. Your aircraft.",
                            "no_position",
                        )
                        .await;
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
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "The taxi ended before the holding point, so I'm standing down. \
                     Say fly the flight plan or full auto again and I'll take it from here.",
                    "taxi_incomplete",
                )
                .await;
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
                        &xp,
                        tools.taxi.park_brake_valve_set,
                        &voice,
                        "The tower never cleared us for takeoff. Your aircraft. \
                         Say full auto again once we're cleared.",
                        "no_takeoff_clearance",
                    )
                    .await;
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
                        &xp,
                        tools.taxi.park_brake_valve_set,
                        &voice,
                        "The landing traffic never cleared. Holding short. Your aircraft.",
                        "line_up_behind_timeout",
                    )
                    .await;
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
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "The line-up didn't finish. Your aircraft.",
                    "line_up_stalled",
                )
                .await;
                return;
            }
            bail_if_abort!();
            let why = match runway_check(&xp, &tools).await {
                Ok(()) => break,
                Err(why) => why,
            };
            // A runway that is too short stays too short: no second line-up.
            let hopeless = matches!(why, Some(crate::autotakeoff::NoFit::RunwayTooShort { .. }));
            if attempt == 1 || hopeless {
                // Say which gate failed. "I couldn't get us onto the runway"
                // was said to an A330 on Edinburgh 24's centreline, aligned
                // within a degree, the tower logging `lined_up=true`: the fit
                // had failed on LENGTH (2026-09-20).
                event("line_up_no_fit", json!({ "why": format!("{why:?}") }));
                // ...and `NoneNear` is a gate like the rest when it knows
                // which strip it looked for: at Le Bourget that was the runway
                // the tower had cleared, sitting 169 m away, and the vague
                // line was all the pilot got (2026-09-22).
                use crate::autotakeoff::NoFit;
                let named = match &why {
                    Some(NoFit::NoneNear { id, .. }) => !id.is_empty(),
                    Some(
                        NoFit::RunwayTooShort { .. }
                        | NoFit::TooLittleLeft { .. }
                        | NoFit::NotAligned { .. },
                    ) => true,
                    _ => false,
                };
                let line = match (&why, named) {
                    (Some(w), true) => format!(
                        "{} Your aircraft.",
                        crate::autotakeoff::no_fit_line(w, false).0
                    ),
                    _ => "I couldn't get us onto the runway. Your aircraft.".to_string(),
                };
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    &line,
                    "line_up_failed",
                )
                .await;
                return;
            }
        }
        event("lined_up", json!({}));
        // A SID with minimums: the takeoff's own climb-out at best rate.
        crate::autotakeoff::DEPARTURE_MINIMUMS.store(
            matches!(&tools.mission, Mission::Flight(r) if !r.climb_minimums.is_empty()),
            SeqCst,
        );
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
                &xp,
                tools.taxi.park_brake_valve_set,
                &voice,
                "We're lined up and ready, but the tower never cleared us for takeoff. \
                 Your aircraft. Say full auto again once we're cleared.",
                "no_takeoff_clearance",
            )
            .await;
            return;
        }
        bail_if_abort!();
    } else {
        // A RUNWAY START: the sim put us on the numbers and nobody taxied.
        // "Skip straight to the circuit" used to mean skipping the tower too:
        // no call, no clearance, no strobes, and the A330 flew Glasgow's zone
        // to "you are inside controlled airspace without clearance" (EGPF,
        // 2026-09-21). The tower now takes a "ready" from the strip as a
        // lined-up aircraft (`Ctx::own_on_runway`), so the same call and the
        // same wait as the taxied departure apply.
        let strobe = light(&tools, &ctx, "strobe", true).await;
        let landing = light(&tools, &ctx, "landing", true).await;
        event(
            "lights_takeoff",
            json!({ "strobe": strobe, "landing": landing, "runway_start": true }),
        );
        if crate::mode::atc_enabled() && crate::party_line::clearance_state().is_some() {
            if let Some(khz) = crate::party_line::home_tower_khz() {
                let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
            }
            request_departure(&voice, &callsign).await;
            bail_if_abort!();
            let mut read_back = false;
            waiting("the takeoff clearance");
            if !ensure_takeoff_cleared(&voice, &callsign, &mut read_back, false).await {
                give_up(
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "We're on the runway, but the tower never cleared us for takeoff. \
                     Your aircraft. Say full auto again once we're cleared.",
                    "no_takeoff_clearance",
                )
                .await;
                return;
            }
            ensure_squawk(&xp, &ctx, &voice).await;
            bail_if_abort!();
        }
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
                    &xp,
                    tools.taxi.park_brake_valve_set,
                    &voice,
                    "The circuit didn't arm, so I'm standing down. Your aircraft.",
                    "circuit_refused",
                )
                .await;
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
            &format!("Cleared to go. Flying your plan to {}.", route.dest_name),
        );
        if let Ok(reply) = tools.takeoff.execute(&json!({}), ctx).await {
            announce(voice, Priority::Advisory, &reply);
        }
        if !crate::autotakeoff::ACTIVE.load(Relaxed) {
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "The takeoff didn't arm, so the flight is off. Your aircraft.",
                "takeoff_refused",
            )
            .await;
            return;
        }
        waiting("the takeoff");
        if !wait_until(360.0, 1000, || async { !crate::autotakeoff::running() }).await {
            if !ABORT.load(SeqCst) {
                give_up(
                    xp,
                    tools.taxi.park_brake_valve_set,
                    voice,
                    "The takeoff never finished. Your aircraft.",
                    "takeoff_stalled",
                )
                .await;
            } else {
                give_up(
                    xp,
                    tools.taxi.park_brake_valve_set,
                    voice,
                    "Cancelled. Your aircraft.",
                    "abort",
                )
                .await;
            }
            return;
        }
        if crate::autotakeoff::outcome() != Some(sim::run::Outcome::Completed) {
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "The takeoff handed control back. Your aircraft.",
                "takeoff_not_completed",
            )
            .await;
            return;
        }
        let agl_ft = xp.get_float(AGL_M).await.unwrap_or(0.0) * M_TO_FT;
        if agl_ft < AIRBORNE_MIN_AGL_FT {
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "The takeoff didn't get us airborne, so the flight is off.",
                "takeoff_rejected",
            )
            .await;
            return;
        }
        event("airborne", json!({ "agl_ft": agl_ft }));
        crate::autotakeoff::DEPARTURE_MINIMUMS.store(false, SeqCst);
    } else {
        let (ground, agl) = tokio::join!(
            xp.get_float(crate::datarefs::ON_GROUND_ANY),
            xp.get_float(AGL_M)
        );
        if !matches!((ground,agl), (Ok(g),Ok(h)) if g.is_finite() && h.is_finite() && g < 0.5 && h*M_TO_FT >= 1000.0)
        {
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "Aircraft state changed before resume. I'm monitoring only; you have control.",
                "resume_state_changed",
            )
            .await;
            return;
        }
        event(
            "resumed",
            json!({ "fix": route.legs.first().map(|l| &l.fix_ident) }),
        );
        // Reload only after the explicit resume; fms_goto selects the joined fix.
        let _ = xp
            .set_data("xcopilot/fms/route", route.fms_payload.as_bytes())
            .await;
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
            climb_minimums: route.climb_minimums.clone(),
            visual_deg: None,
            missed_legs: Vec::new(),
        },
        xp.cancellation(),
    ) {
        give_up(
            xp,
            tools.taxi.park_brake_valve_set,
            voice,
            "The previous procedure is still stopping. Your aircraft.",
            "procedure_busy",
        )
        .await;
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
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "The enroute leg never completed. Your aircraft.",
                "enroute_stalled",
            )
            .await;
        } else {
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                "Cancelled. Your aircraft.",
                "abort",
            )
            .await;
        }
        return;
    }
    if ABORT.load(SeqCst) {
        give_up(
            xp,
            tools.taxi.park_brake_valve_set,
            voice,
            "Cancelled. Your aircraft.",
            "abort",
        )
        .await;
        return;
    }
    event("enroute_complete", json!({}));
    // Whatever the cruise did with them, the signs are on for the arrival (a
    // short sector never crossed ten thousand; nothing pressed when already on).
    let _ = crate::tools::signs::set(xp, true).await;
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
            xp,
            tools.taxi.park_brake_valve_set,
            voice,
            "I can't read our position for the arrival. Your aircraft.",
            "no_position",
        )
        .await;
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
    let build = |forced: Option<String>| {
        let (apt_dat, cifp, fixp, navp) = (
            crate::tools::autoland::apt_dat_path(),
            config.cifp_dir.clone(),
            config.fix_data_path.clone(),
            config.nav_data_path.clone(),
        );
        let dest = route.dest_icao.clone();
        let cfg = tools.circuit.cfg.clone();
        let ias = tools.circuit.approach_ias;
        async move {
            tokio::task::spawn_blocking(move || {
                build_arrival(
                    &apt_dat, &cifp, &fixp, &navp, &dest, lat, lon, cfg, ias, wind,
                    forced.as_deref(),
                )
            })
            .await
            .unwrap_or(Arrival {
                plan: None,
                rwy_id: None,
                tower_khz: None,
                mountain_msa_ft: None,
            })
        }
    };
    let mut built = {
        build(None).await
    };
    if route
        .local_runway
        .as_ref()
        .is_some_and(|r| built.rwy_id.as_ref() != Some(r))
    {
        give_up(
            xp,
            tools.taxi.park_brake_valve_set,
            voice,
            "The arrival runway changed. The local route needs replanning. Your aircraft.",
            "local_runway_changed",
        )
        .await;
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
    // The TOWER's runway, once the inbound call has one. LOWI (2026-09-24):
    // "join final runway two six" came back two seconds after the crew had
    // armed the 08 approach, and nothing reconciled them. The tower's end is
    // flown when there is an approach to it we can fly; otherwise ours stands
    // and the crew says why (autoland still watches the landing clearance).
    if let Some(tower_rwy) = tower_arrival_runway(built.tower_khz).await {
        if built.rwy_id.as_deref() != Some(tower_rwy.as_str()) {
            let theirs = build(Some(tower_rwy.clone())).await;
            let spoken = crate::circuit::spell_runway(&tower_rwy);
            if theirs.plan.is_some() {
                event(
                    "arrival_runway_from_tower",
                    json!({ "was": built.rwy_id, "now": tower_rwy }),
                );
                announce(
                    voice,
                    Priority::Advisory,
                    &format!("Tower wants runway {spoken}. Setting up for it."),
                );
                built = theirs;
            } else {
                event(
                    "tower_runway_unflyable",
                    json!({ "ours": built.rwy_id, "tower": tower_rwy }),
                );
                announce(
                    voice,
                    Priority::Caution,
                    &format!(
                        "Tower wants runway {spoken}, but there's no approach I can fly to it. \
                         Staying with runway {}.",
                        built
                            .rwy_id
                            .as_deref()
                            .map(crate::circuit::spell_runway)
                            .unwrap_or_default()
                    ),
                );
            }
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
                    xp,
                    tools.taxi.park_brake_valve_set,
                    voice,
                    "The previous procedure is still stopping. Your aircraft.",
                    "procedure_busy",
                )
                .await;
                return;
            }
        }
        _ if route.local_runway.is_none() && built.mountain_msa_ft.is_some() => {
            // A mountain field with no approach we can fly: never a visual
            // autoland into the valley. The follower has finished the route;
            // hold at or above the sector altitude and hand over, saying why.
            // (LSZS, 2026-09-24: its RNAV approaches end at a missed-approach
            // point before the runway; the old fallback was a visual autoland
            // into the Engadin with 15,500 ft sectors.)
            let msa = built.mountain_msa_ft.unwrap_or(0.0);
            let hold_ft = (alt_msl_ft.max(msa + 500.0) / 500.0).ceil() * 500.0;
            let thr = crate::aircraft::throttle_dataref();
            let _ = tokio::join!(
                xp.set_float("sim/cockpit2/autopilot/altitude_dial_ft", hold_ft),
                xp.set_float(VS_DIAL, if alt_msl_ft < hold_ft { 1000.0 } else { 0.0 }),
                xp.set_float(&thr, GIVE_UP_THROTTLE),
            );
            event(
                "arrival_handover",
                json!({ "runway": built.rwy_id, "msa_ft": msa, "hold_ft": hold_ft }),
            );
            give_up(
                xp,
                tools.taxi.park_brake_valve_set,
                voice,
                &mountain_handover_line(&route.dest_name, msa, hold_ft),
                "arrival_unflyable",
            )
            .await;
            return;
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
            xp,
            tools.taxi.park_brake_valve_set,
            voice,
            "The arrival isn't getting us there. Your aircraft. The autopilot is holding \
             heading and altitude, climb power set.",
            "arrival_stalled",
        )
        .await;
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
            // `autoland_ended`, not "landed": autoland also goes inactive on
            // a give-up, and `landed: true` was logged at 1,900 ft.
            json!({ "autoland_ended": landed, "on_ground": on_ground, "crashed": crashed }),
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
        // The mission ends shut down on the stand.
        true,
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
    // (kind, spoken runway) per end with a published approach; the spoken
    // runway per end with only a DME-fix straight-in.
    let (mut published, mut straight_in, mut visual) = (Vec::new(), Vec::new(), Vec::new());
    let nav_path = std::path::Path::new(fix_path)
        .with_file_name("earth_nav.dat")
        .to_string_lossy()
        .into_owned();
    let (alat, alon) = airport.position().unwrap_or((airport.runways[0].lat_a, airport.runways[0].lon_a));
    for r in &airport.runways {
        for id in [&r.ident_a, &r.ident_b] {
            let spoken = crate::circuit::spell_runway(id);
            // Built exactly as the arrival builds it: an approach the brief
            // names is one the arrival can fly (LSZS's RNAV 21 has a 5.3
            // degree visual segment, too steep, and is not named).
            match crate::nav::best_approach(cifp, dest, id).and_then(|appch| {
                crate::procedure::approach_plan_for(
                    cifp,
                    fix_path,
                    &nav_path,
                    &airport,
                    dest,
                    id,
                    alat,
                    alon,
                    crate::procedure::EndAction::Level,
                )
                .ok()
                .map(|p| (appch, p))
            }) {
                Some((appch, plan)) if plan.visual_deg.is_some() => {
                    visual.push((crate::procedure::approach_kind(&appch), spoken))
                }
                Some((appch, _)) => published.push((crate::procedure::approach_kind(&appch), spoken)),
                None if !crate::nav::reconstruct_approach(fix_path, &airport, dest, id).is_empty() => {
                    straight_in.push(spoken)
                }
                None => {}
            }
        }
    }
    if published.is_empty() && straight_in.is_empty() && !visual.is_empty() {
        let (kind, rwy) = &visual[0];
        return Ok(format!(
            "{} has the {kind} to runway {rwy}: I'll fly it to the missed approach point and \
             land visually from there, on its own steep path.",
            airport.name
        ));
    }
    if published.is_empty() && straight_in.is_empty() {
        let charted = crate::nav::list_procedures(cifp, dest, crate::nav::ProcKind::Appch);
        let msa = crate::nav::airport_msa_ft(&msa_path_for(fix_path), dest);
        if mountain_field(msa, airport.elev_ft) {
            return Ok(format!(
                "{} is a mountain field, and {}. I'll fly the route and hand over at its end, \
                 above the sector altitude of {}.",
                airport.name,
                if charted.is_empty() {
                    "it has no published approach".to_string()
                } else {
                    "its published approaches end short of the runway, which I can't fly".to_string()
                },
                crate::flightplan::spoken_feet(msa.unwrap_or(0.0))
            ));
        }
        if !charted.is_empty() {
            return Ok(format!(
                "{}'s published approaches end short of the runway, which I can't fly, so I'll land visually.",
                airport.name
            ));
        }
    }
    Ok(arrival_brief_line(
        &airport.name,
        airport.runways.len() * 2,
        &published,
        &straight_in,
    ))
}

/// Sector altitude this far above the field makes it a mountain field: no
/// visual fallback, the arrival is an approach we can fly or a hand-over.
const MOUNTAIN_MSA_MARGIN_FT: f64 = 3_000.0;

fn mountain_field(msa_ft: Option<f64>, elev_ft: f64) -> bool {
    msa_ft.is_some_and(|m| m - elev_ft >= MOUNTAIN_MSA_MARGIN_FT)
}

/// `earth_msa.dat` beside the fix file (both in Custom Data, or both in the
/// default data).
fn msa_path_for(fix_path: &str) -> String {
    std::path::Path::new(fix_path)
        .with_file_name("earth_msa.dat")
        .to_string_lossy()
        .into_owned()
}

/// The hand-over at the end of the route into a mountain field we cannot fly
/// an approach to. Pure.
pub fn mountain_handover_line(dest: &str, msa_ft: f64, hold_ft: f64) -> String {
    format!(
        "End of the route. I can't fly an approach into {dest}. Your aircraft. The autopilot \
         is holding {}, above the sector altitude of {}.",
        crate::flightplan::spoken_feet(hold_ft),
        crate::flightplan::spoken_feet(msa_ft)
    )
}

/// One line, sized to the field: a big airport's approaches are SUMMARISED
/// ("the ILS on every runway"), a small one's are named. Eight runway ends
/// read out one by one was a paragraph before the mission was said, and the
/// wind sentence that followed said nothing the pilot did not know (EGSS-LFPG,
/// 2026-09-22). Pure, tested.
fn arrival_brief_line(
    name: &str,
    ends: usize,
    published: &[(&str, String)],
    straight_in: &[String],
) -> String {
    let kinds: std::collections::BTreeSet<&str> = published.iter().map(|(k, _)| *k).collect();
    let what = if kinds.len() == 1 {
        format!("the {}", kinds.iter().next().unwrap())
    } else {
        "a published approach".to_string()
    };
    let n = published.len();
    if n > 0 && n == ends {
        return format!("{name} has {what} on every runway.");
    }
    if n > 2 {
        return format!("{name} has {what} on {n} of {ends} runway ends.");
    }
    let mut parts: Vec<String> = published
        .iter()
        .map(|(k, rwy)| format!("the {k} on {rwy}"))
        .collect();
    if !straight_in.is_empty() {
        parts.push(format!("a straight-in on {}", straight_in.join(" and ")));
    }
    if parts.is_empty() {
        format!("{name} has no published approach, so I'll land visually.")
    } else {
        format!("{name} has {}.", parts.join(" and "))
    }
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
    /// The field's highest sector altitude when it is a MOUNTAIN field
    /// (`mountain_field`): with no approach we can fly, the arrival hands the
    /// aircraft over above it instead of landing visually.
    mountain_msa_ft: Option<f64>,
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
    // ON the tower's frequency before the call is keyed. The enroute leg
    // calls when the region has made the destination's tower the station,
    // but a region change near a field is a crew notice ("closest field is
    // Exeter"), not a handoff, and nothing retunes on a notice: EGPH-EGTE
    // made this call on Carlisle's frequency, where COM1 had sat since the
    // Scottish border, it went into the void marked as made, and Exeter
    // challenged the arrival as an unknown aircraft (2026-09-21).
    let tuned = xp
        .get_float(COM1_FREQ_KHZ)
        .await
        .ok()
        .map(|v| v.round() as i64);
    if !inbound_on_frequency(tuned, khz) {
        let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
        event("inbound_retune", json!({ "khz": khz, "was": tuned }));
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

/// Is COM1 already on the tower the inbound call is for? A failed read is
/// not "yes": the retune is one write and the void call cost a zone challenge.
pub fn inbound_on_frequency(com1_khz: Option<i64>, tower_khz: i64) -> bool {
    com1_khz == Some(tower_khz)
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
    forced_rwy: Option<&str>,
) -> Arrival {
    let Some(airport) = crate::nav::find_airport(apt_dat, dest).ok().flatten() else {
        return Arrival {
            plan: None,
            rwy_id: None,
            tower_khz: None,
            mountain_msa_ft: None,
        };
    };
    let tower_khz = airport.freq(crate::nav::FREQ_TOWER).map(|f| f.khz);
    let mountain_msa_ft = crate::nav::airport_msa_ft(&msa_path_for(fixp), dest)
        .filter(|m| mountain_field(Some(*m), airport.elev_ft));
    // The tower's runway when it has named one (no fallback to another end:
    // the caller keeps its own plan if this one cannot be flown)...
    if let Some(forced) = forced_rwy {
        let end = crate::procedure::EndAction::Approach {
            apt_dat: apt_dat.to_string(),
            cfg,
            approach_ias,
            rwy_id: Some(crate::autoland::PlannedRunway::at(dest, forced)),
        };
        let plan = crate::procedure::approach_plan_for(
            cifp, fixp, navp, &airport, dest, forced, lat, lon, end,
        )
        .ok();
        return Arrival {
            plan,
            rwy_id: Some(forced.to_string()),
            tower_khz,
            mountain_msa_ft,
        };
    }
    // Into the wind when it's strong enough to matter...
    let by_wind = wind
        .filter(|(_, kts)| *kts >= WIND_DECISIVE_KTS)
        .and_then(|(dir, kts)| crate::nav::best_runway_for_wind(&airport.runways, dir, kts))
        .map(|(id, _, _)| id);
    // ...otherwise the end most aligned with our arrival bearing.
    let rwy_id = by_wind.or_else(|| {
        let diff = |a: f64, b: f64| ((a - b + 540.0).rem_euclid(360.0) - 180.0).abs();
        let (mut best_id, mut best_score) = (None::<String>, f64::MAX);
        // Hard runways only: LOWI's alignment pick was the grass "26R".
        for r in crate::nav::assignable_runways(&airport.runways) {
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
            mountain_msa_ft,
        };
    };
    let plan_for = |id: &str| {
        let end = crate::procedure::EndAction::Approach {
            apt_dat: apt_dat.to_string(),
            cfg: cfg.clone(),
            approach_ias,
            rwy_id: Some(crate::autoland::PlannedRunway::at(dest, id)),
        };
        crate::procedure::approach_plan_for(cifp, fixp, navp, &airport, dest, id, lat, lon, end)
            .ok()
    };
    if let Some(plan) = plan_for(&rwy_id) {
        return Arrival {
            plan: Some(plan),
            rwy_id: Some(rwy_id),
            tower_khz,
            mountain_msa_ft,
        };
    }
    // No approach we can fly to that end: another end that HAS one, if its
    // tailwind is acceptable, before a visual landing. LOWI at 080/6: 08's
    // approaches are a circling one and an RF arc, 26's RNAV Z is a
    // straight-in down the valley; the visual landing at Innsbruck is not an
    // option anyone should take (2026-09-24).
    for id in approach_fallback_ends(&airport, &rwy_id, wind) {
        if let Some(plan) = plan_for(&id) {
            return Arrival {
                plan: Some(plan),
                rwy_id: Some(id),
                tower_khz,
                mountain_msa_ft,
            };
        }
    }
    Arrival {
        plan: None,
        rwy_id: Some(rwy_id),
        tower_khz,
        mountain_msa_ft,
    }
}

/// The runway the destination's tower has given an arrival (its `Inbound`
/// clearance), waited for up to [`TOWER_RUNWAY_WAIT_SECS`] after the inbound
/// call; `None` with no reply, another station on the frequency, or no call.
async fn tower_arrival_runway(dest_khz: Option<i64>) -> Option<String> {
    let dest_khz = dest_khz?;
    let deadline = Instant::now() + Duration::from_secs_f64(TOWER_RUNWAY_WAIT_SECS);
    loop {
        if crate::party_line::home_tower_khz() == Some(dest_khz)
            && matches!(
                crate::party_line::clearance_state(),
                Some(crate::tower::State::Inbound | crate::tower::State::ClearedToLand)
            )
        {
            if let Some(r) = crate::party_line::clearance_runway() {
                return Some(r);
            }
        }
        if Instant::now() >= deadline || !INBOUND_MADE.load(SeqCst) {
            return None;
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
}

/// How long the arrival waits for the tower's answer to the inbound call.
const TOWER_RUNWAY_WAIT_SECS: f64 = 10.0;

/// The line said when the tower clears a runway the planned SID does not
/// depart from; `None` when they agree or either is unknown. Pure.
pub fn departure_runway_mismatch(planned: Option<&str>, cleared: Option<&str>) -> Option<String> {
    let (p, c) = (planned?, cleared?);
    (!p.eq_ignore_ascii_case(c)).then(|| {
        format!(
            "The tower has given us runway {}, and the planned departure is from runway {}. \
             I can't fly that departure from here.",
            crate::circuit::spell_runway(c),
            crate::circuit::spell_runway(p)
        )
    })
}

/// One departure minimum judged against the climb: the gradient it needs from
/// the departure end of the runway, and what the airframe makes at Vy.
#[derive(Debug, Clone, PartialEq)]
pub struct GradientCheck {
    pub fix: String,
    pub min_ft: f64,
    pub dist_nm: f64,
    pub need_ft_per_nm: f64,
    pub have_ft_per_nm: f64,
}

/// Refused above this share of the airframe's climb gradient...
const GRADIENT_REFUSE_FRAC: f64 = 0.90;
/// ...said as a caution above this.
const GRADIENT_CAUTION_FRAC: f64 = 0.75;

/// Every departure minimum on the route, from the runway's departure end
/// `der` at `field_ft`, against `best_climb_fpm` at `vy_kts` (true airspeed
/// rising 2 % a thousand feet, no wind credit). Pure.
pub fn departure_gradients(
    der: (f64, f64),
    field_ft: f64,
    legs: &[crate::nav::ProcLeg],
    minimums: &[crate::nav::ClimbMinimum],
    vy_kts: f64,
    best_climb_fpm: f64,
) -> Vec<GradientCheck> {
    let mut out = Vec::new();
    let mut dist = 0.0;
    let mut from = der;
    for leg in legs {
        let Some(f) = &leg.fix else { continue };
        dist += crate::nav::haversine_nm(from.0, from.1, f.lat, f.lon);
        from = (f.lat, f.lon);
        if let Some(m) = minimums.iter().find(|m| m.fix.eq_ignore_ascii_case(&leg.fix_ident)) {
            let mid_kft = (field_ft + m.min_ft) / 2.0 / 1000.0;
            let tas = vy_kts * (1.0 + 0.02 * mid_kft);
            out.push(GradientCheck {
                fix: m.fix.clone(),
                min_ft: m.min_ft,
                dist_nm: dist,
                need_ft_per_nm: (m.min_ft - field_ft).max(0.0) / dist.max(0.1),
                have_ft_per_nm: best_climb_fpm / (tas / 60.0),
            });
        }
    }
    out
}

/// The departure's verdict: `Err` a refusal naming the binding minimum,
/// `Ok(Some(line))` a caution for the brief, `Ok(None)` comfortable.
pub fn gradient_verdict(sid: &str, checks: &[GradientCheck]) -> Result<Option<String>, String> {
    let Some(worst) = checks
        .iter()
        .max_by(|a, b| (a.need_ft_per_nm / a.have_ft_per_nm).total_cmp(&(b.need_ft_per_nm / b.have_ft_per_nm)))
    else {
        return Ok(None);
    };
    let share = worst.need_ft_per_nm / worst.have_ft_per_nm.max(1.0);
    let line = format!(
        "{sid} needs {} at {}, {:.0} feet a mile; we climb about {:.0} at best rate.",
        crate::flightplan::spoken_feet(worst.min_ft),
        worst.fix,
        worst.need_ft_per_nm,
        worst.have_ft_per_nm
    );
    if share > GRADIENT_REFUSE_FRAC {
        Err(format!("Negative. {line} Not flying that departure."))
    } else if share > GRADIENT_CAUTION_FRAC {
        Ok(Some(format!("{line} Tight, I'll climb at Vy until it's made.")))
    } else {
        Ok(Some(format!("{line} I'll climb at Vy until it's made.")))
    }
}

/// The most tailwind an arrival accepts to fly a published approach rather
/// than a visual landing.
const APPROACH_TAILWIND_MAX_KTS: f64 = 10.0;

/// The other hard runway ends, most headwind first, whose tailwind is within
/// [`APPROACH_TAILWIND_MAX_KTS`]. With no wind every end qualifies.
fn approach_fallback_ends(
    airport: &crate::nav::AirportInfo,
    tried: &str,
    wind: Option<(f64, f64)>,
) -> Vec<String> {
    let mut ends: Vec<(String, f64)> = crate::nav::assignable_runways(&airport.runways)
        .flat_map(|r| [r.ident_a.clone(), r.ident_b.clone()])
        .filter(|id| id != tried)
        .map(|id| {
            let hw = wind
                .and_then(|(d, k)| crate::nav::headwind_for(&airport.runways, &id, d, k))
                .unwrap_or(0.0);
            (id, hw)
        })
        .filter(|(_, hw)| *hw >= -APPROACH_TAILWIND_MAX_KTS)
        .collect();
    ends.sort_by(|a, b| b.1.total_cmp(&a.1));
    ends.into_iter().map(|(id, _)| id).collect()
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
    /// The airframe's ceiling; `None` skips the cruise check.
    pub service_ceiling_ft: Option<f64>,
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

    // A cruise the aircraft cannot reach is not a plan: EDLP-LOWI's FL290
    // was armed on an airframe declared to 28,000 ft, and the refusal came
    // in the air, at the resume after a pilot takeover (2026-09-24).
    if let Some(ceiling) = f.service_ceiling_ft.filter(|c| *c > 0.0) {
        if fp.cruise_alt_ft > ceiling {
            return Err(format!(
                "Negative. The plan cruises at {}, above this aircraft's ceiling of {}. \
                 Replan lower.",
                spoken_feet(fp.cruise_alt_ft),
                spoken_feet(ceiling)
            ));
        }
    }
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
    // The roll is quoted at the PROFILE's max gross, whatever the plan's
    // operational limit is; scaled to the weight on the wheels now.
    let need_ft = crate::aircraft::takeoff_needed_m(
        f.takeoff_roll_m,
        f.total_kg,
        f.max_gross_lb.map(|lb| lb / LB_PER_KG),
        500.0,
    ) * M_TO_FT;
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
    // The unit is said ONCE: "12,880 pounds for the plan's 12,990" (the ack
    // was a paragraph, EGSS-LFPG 2026-09-22).
    if let Some(fuel) = f.fuel_kg.filter(|_| fp.block_fuel_kg > 0.0) {
        let min = fp.min_takeoff_fuel_kg;
        if min > 0.0 && fuel < min * (1.0 - FUEL_MATCH_FRAC) {
            return Err(format!(
                "Negative. {} of fuel aboard, and the plan needs at least {} to go. Say load the aircraft and I'll fuel it from the plan, then say it again.",
                spoken_fuel(fuel),
                spoken_fuel(min)
            ));
        }
        let bare = |s: String| {
            s.split_once(' ')
                .map_or_else(|| s.clone(), |(n, _)| n.to_string())
        };
        if fuel < fp.block_fuel_kg * (1.0 - FUEL_MATCH_FRAC) {
            lines.push(format!(
                "Fuel {} against a {} block, above the minimum.",
                spoken_fuel(fuel),
                bare(spoken_fuel(fp.block_fuel_kg))
            ));
        } else {
            lines.push(format!(
                "Fuel {} for the plan's {}.",
                spoken_fuel(fuel),
                bare(spoken_fuel(fp.block_fuel_kg))
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
                    load.push_str(" as planned");
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
    runway_check(xp, tools).await.is_ok()
}

/// The takeoff loop's own fit, asked with the takeoff's own pavement rule
/// (`aircraft::takeoff_needed_now_m`), and on a miss the gate that failed.
/// `Err(None)` = airborne, or no position to fit.
async fn runway_check(
    xp: &XPlaneClient,
    tools: &SubTools,
) -> Result<(), Option<crate::autotakeoff::NoFit>> {
    // Airborne is not "lined up"; and a fit needs a valid position.
    if xp.get_float(ON_GROUND).await.unwrap_or(1.0) < 0.5 {
        return Err(None);
    }
    let (Ok(lat), Ok(lon), Ok(psi)) = (
        xp.get_float("sim/flightmodel/position/latitude").await,
        xp.get_float("sim/flightmodel/position/longitude").await,
        xp.get_float("sim/flightmodel/position/psi").await,
    ) else {
        return Err(None);
    };
    let needed_m = crate::aircraft::takeoff_needed_now_m(
        xp,
        tools.line_up.takeoff_roll_m,
        tools.line_up.mtow_kg,
        500.0,
    )
    .await;
    let apt = crate::tools::autoland::apt_dat_path();
    let cands = crate::autoland::nearby_runways(&apt, lat, lon, 3.0).unwrap_or_default();
    match crate::autotakeoff::runway_fit(&cands, lat, lon, psi, needed_m) {
        Some(_) => Ok(()),
        None => Err(Some(crate::autotakeoff::why_no_fit(
            &cands, lat, lon, psi, needed_m,
        ))),
    }
}

#[cfg(test)]
mod tests {
    /// LFPB 2026-09-22: the line-up refused at the holding point, full auto
    /// said "your aircraft", and the taxi governor went on holding the 737 on
    /// its wheel brakes for twenty-seven minutes with `park` at 0.0. A give-up
    /// on the ground hands the hold to the PARKING brake so the loop can stop.
    #[tokio::test]
    async fn a_ground_give_up_hands_the_hold_to_the_parking_brake() {
        use super::hand_back_brakes_if;
        use crate::xplane::FakeXPlane;
        const SET: &str = "sim/flight_controls/park_brake_set";
        const PARK: &str = "sim/cockpit2/controls/parking_brake_ratio";

        // Stopped at the hold with the governor holding us: the brake goes on,
        // and the valve goes with it (the Baron's latching valve is why the
        // raw ratio write is never the seam).
        let xp = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (crate::autotaxi::GS_MS, 0.0),
            (PARK, 0.0),
        ]);
        xp.step_on_command(SET, PARK, 1.0);
        hand_back_brakes_if(&xp, true, Some(1.0)).await;
        assert!(xp.ran(SET), "the parking brake is set");
        assert!(
            xp.ran("sim/flight_controls/park_brake_valve_open"),
            "valve-aware, never a raw ratio write"
        );

        // Airborne: nothing. A give-up in the air hands back a flying
        // aircraft, and a parking brake is not part of that.
        let air = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 0.0),
            (crate::autotaxi::GS_MS, 90.0),
            (PARK, 0.0),
        ]);
        hand_back_brakes_if(&air, true, Some(1.0)).await;
        assert!(!air.ran(SET), "no park brake in the air");

        // Rolling: a parking brake is a chock, not a stopping device.
        let fast = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (crate::autotaxi::GS_MS, 40.0),
            (PARK, 0.0),
        ]);
        hand_back_brakes_if(&fast, true, Some(1.0)).await;
        assert!(!fast.ran(SET), "not at 78 knots");

        // Nothing of ours holding the aircraft: leave the brakes alone. The
        // pilot may be sitting on the pedals, and a give-up is not a licence
        // to move a control nobody asked us to move.
        let idle = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (crate::autotaxi::GS_MS, 0.0),
            (PARK, 0.0),
        ]);
        hand_back_brakes_if(&idle, false, Some(1.0)).await;
        assert!(!idle.ran(SET), "no governor, no change");

        // A failed groundspeed read is NO measurement, never an assumed zero.
        let blind = FakeXPlane::with_reads(&[(crate::datarefs::ON_GROUND_ANY, 1.0), (PARK, 0.0)]);
        hand_back_brakes_if(&blind, true, Some(1.0)).await;
        assert!(!blind.ran(SET), "a speed we could not read is not a stop");
    }

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
            transition_altitude_ft: None,
            transition_level_ft: None,
            procedures: Default::default(),
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
            service_ceiling_ft: None,
        }
    }

    /// LOWI 26 MOGT3H in the SF50 (the SimBrief plan of 2026-09-24): the
    /// binding minimum is WI506, 714 ft a mile over 8.3 nm from the departure
    /// end; at Vy 120 kt and 2,000 fpm the SF50 makes about 911 (78 %). Tight,
    /// briefed, flown at Vy.
    #[test]
    fn a_sid_gradient_is_judged_against_the_climb_at_vy() {
        let leg = |id: &str, lat: f64, lon: f64| crate::nav::ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "TF".into(),
            fix_ident: id.into(),
            fix_region: String::new(),
            fix: Some(crate::nav::Fix { ident: id.into(), lat, lon }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: None,
            alt2_ft: None,
            is_faf: false,
            is_map: false,
            vpa_deg: None,
        };
        let legs = vec![
            leg("WI505", 47.252, 11.269),
            leg("WI506", 47.290, 11.139),
            leg("WI507", 47.306, 11.086),
            leg("WI802", 47.296, 10.840),
            leg("MOGTI", 47.389, 10.717),
        ];
        let m = |f: &str, a: f64| crate::nav::ClimbMinimum { fix: f.into(), min_ft: a };
        let mins = vec![m("WI505", 4000.0), m("WI506", 7850.0), m("WI507", 8400.0), m("WI802", 11350.0), m("MOGTI", 13000.0)];
        let der = (47.2587834, 11.3308736); // the 08 end: 26's departure end
        let checks = super::departure_gradients(der, 1907.0, &legs, &mins, 120.0, 2000.0);
        assert_eq!(checks.len(), 5);
        let wi506 = &checks[1];
        assert!((wi506.dist_nm - 8.3).abs() < 0.3, "{wi506:?}");
        assert!((wi506.need_ft_per_nm - 714.0).abs() < 20.0, "{wi506:?}");
        let line = super::gradient_verdict("MOGT3H", &checks).expect("flyable").expect("briefed");
        assert!(line.contains("MOGT3H needs") && line.contains("Tight"), "{line}");
        // At the cruise-speed climb flown today (1,500 fpm at 190 kt ~ 470 ft/nm
        // equivalent) the same departure is refused.
        let weak = super::departure_gradients(der, 1907.0, &legs, &mins, 190.0, 1500.0);
        assert!(super::gradient_verdict("MOGT3H", &weak).is_err());
    }

    #[test]
    fn a_cleared_runway_that_is_not_the_sids_is_said_on_the_ramp() {
        assert!(super::departure_runway_mismatch(Some("26"), Some("08")).unwrap().contains("zero eight"));
        assert_eq!(super::departure_runway_mismatch(Some("26"), Some("26")), None);
        assert_eq!(super::departure_runway_mismatch(None, Some("08")), None, "no SID, no rule");
        assert_eq!(super::departure_runway_mismatch(Some("26"), None), None, "no tower, no rule");
    }

    #[test]
    fn a_cruise_above_the_ceiling_is_refused_at_the_arm() {
        // EDLP-LOWI (2026-09-24): FL290 on an airframe declared to 28,000.
        let mut fp = plan(false);
        fp.cruise_alt_ft = 29_000.0;
        let f = super::PreflightFacts {
            service_ceiling_ft: Some(28_000.0),
            ..facts()
        };
        let why = super::preflight(&fp, &f).expect_err("refused");
        assert!(why.contains("ceiling"), "{why}");
        let ok = super::PreflightFacts {
            service_ceiling_ft: Some(31_000.0),
            ..facts()
        };
        assert!(super::preflight(&fp, &ok).map_or_else(|e| !e.contains("ceiling"), |_| true));
    }

    #[test]
    fn a_plan_that_fits_is_briefed_with_its_fuel_and_its_load() {
        let brief = super::preflight(&plan(true), &facts()).expect("fits");
        assert_eq!(
            brief,
            "Fuel 240 pounds for the plan's 240. \
             3 passengers and 70 pounds of cargo as planned."
        );
        // A local plan has no load sheet and speaks the app's gallons.
        let brief = super::preflight(&plan(false), &facts()).expect("fits");
        assert_eq!(brief, "Fuel 40 gallons for the plan's 40.");
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
        // The takeoff needs the roll x 1.4 (1,630 ft -> 2,282 ft at max
        // gross), scaled to the 1,100 kg on the wheels against 1,160: 2,163.
        let short_origin = super::PreflightFacts {
            origin_longest_ft: 2000.0,
            ..facts()
        };
        let why = super::preflight(&plan(true), &short_origin).expect_err("refused");
        assert!(
            why.starts_with(
                "Negative. LXGB's longest runway is 2,000 feet, and the takeoff needs 2,163 feet."
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
        assert_eq!(why, "Negative. 180 pounds of fuel aboard, and the plan needs at least 230 pounds to go. Say load the aircraft and I'll fuel it from the plan, then say it again.");
        // Between the minimum and the block: said, not refused.
        let light = super::PreflightFacts {
            fuel_kg: Some(104.0),
            ..facts()
        };
        let brief = super::preflight(&plan(true), &light).expect("goes");
        assert!(
            brief.starts_with("Fuel 230 pounds against a 240 block, above the minimum."),
            "{brief}"
        );
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
            brief.ends_with("No passengers or cargo as planned."),
            "{brief}"
        );
    }

    /// EGSS-LFPG, 2026-09-22: eight runway ends read out one by one before
    /// the mission was even said. A field the approaches cover is one clause.
    #[test]
    fn the_arrival_brief_is_sized_to_the_field() {
        let s = |v: &str| v.to_string();
        let ils = |rwy: &str| ("ILS", s(rwy));
        // Charles de Gaulle: four runways, the ILS on all eight ends.
        let cdg: Vec<_> = [
            "zero nine left",
            "two seven right",
            "zero nine right",
            "two seven left",
            "zero eight left",
            "two six right",
            "zero eight right",
            "two six left",
        ]
        .into_iter()
        .map(ils)
        .collect();
        assert_eq!(
            super::arrival_brief_line("Charles De Gaulle", 8, &cdg, &[]),
            "Charles De Gaulle has the ILS on every runway."
        );
        // Mixed kinds still cover the field.
        let mut mixed = cdg.clone();
        mixed[7] = ("RNAV", s("two six left"));
        assert_eq!(
            super::arrival_brief_line("Charles De Gaulle", 8, &mixed, &[]),
            "Charles De Gaulle has a published approach on every runway."
        );
        // Partial cover on a big field is a count, not a list.
        assert_eq!(
            super::arrival_brief_line("Charles De Gaulle", 8, &cdg[..5], &[s("two six left")]),
            "Charles De Gaulle has the ILS on 5 of 8 runway ends."
        );
        // A small field is named end by end.
        assert_eq!(
            super::arrival_brief_line("Exeter", 2, &[ils("two six")], &[s("zero eight")]),
            "Exeter has the ILS on two six and a straight-in on zero eight."
        );
        assert_eq!(
            super::arrival_brief_line("Axarquia", 2, &[], &[]),
            "Axarquia has no published approach, so I'll land visually."
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
        assert_eq!(brief, "Gibraltar has a straight-in on two seven.");
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

    /// What full auto will fly into Innsbruck from where the aircraft is:
    /// `cargo test -- --ignored real_install_lowi_arrival --nocapture`
    #[test]
    #[ignore]
    fn real_install_lowi_arrival() {
        let apt = "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        let cifp = "/Users/duanelicudi/X-Plane 12/Custom Data/CIFP";
        let fix = "/Users/duanelicudi/X-Plane 12/Custom Data/earth_fix.dat";
        let nav = "/Users/duanelicudi/X-Plane 12/Custom Data/earth_nav.dat";
        println!("MOGT3H 26 minimums: {:?}", crate::nav::sid_minimums(cifp, "LOWI", "MOGT3H", "26"));
        for wind in [Some((360.0, 17.0)), None] {
            let a = super::build_arrival(
                apt, cifp, fix, nav, "LSZS", 46.779, 10.259,
                crate::tools::autopilot::ApConfig::default(), 87.0, wind, None,
            );
            println!("LSZS wind {wind:?}: runway {:?} plan {:?}", a.rwy_id, a.plan.as_ref().map(|p| p.name.clone()));
            if let Some(p) = &a.plan {
                println!("   visual {:?} missed {:?}", p.visual_deg, p.missed_legs.iter().map(|l| (l.fix_ident.clone(), l.alt1_ft, l.fix.is_some())).collect::<Vec<_>>());
                for l in &p.legs {
                    println!("   {} {} {:?} map={} vpa={:?} fix={}", l.leg_type, l.fix_ident, l.alt1_ft, l.is_map, l.vpa_deg, l.fix.is_some());
                }
            }
        }
        match super::brief_arrival(apt, cifp, fix, "LSZS") {
            Ok(l) | Err(l) => println!("LSZS brief: {l}"),
        }
        for forced in ["08", "26"] {
            let a = super::build_arrival(
                apt, cifp, fix, nav, "LOWI", 47.77, 11.79,
                crate::tools::autopilot::ApConfig::default(), 87.0, Some((80.0, 6.0)), Some(forced),
            );
            println!("tower wants {forced}: plan {:?}", a.plan.as_ref().map(|p| p.name.clone()));
        }
        for wind in [Some((80.0, 6.0)), Some((110.0, 4.0)), Some((260.0, 12.0)), None] {
            let a = super::build_arrival(
                apt, cifp, fix, nav, "LOWI", 47.77, 11.79,
                crate::tools::autopilot::ApConfig::default(), 87.0, wind, None,
            );
            println!("wind {wind:?}: runway {:?}", a.rwy_id);
            match &a.plan {
                None => println!("  NO PLAN (visual landing)"),
                Some(p) => {
                    println!("  {}", p.name);
                    for l in &p.legs {
                        println!(
                            "  {} {} {:?} fix={}",
                            l.leg_type,
                            l.fix_ident,
                            l.alt1_ft,
                            l.fix.is_some()
                        );
                    }
                }
            }
        }
    }
}
