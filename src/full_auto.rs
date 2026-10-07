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

use crate::autoflight::{self, Dial};
use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
use std::time::{Duration, Instant};

use serde_json::json;

use crate::config::Config;
use crate::runway::{cleared_runway_fit, runway_request_call, short_runway_line, ClearedFit};
use crate::tools::{Tool, ToolCtx};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const ENGN_RUNNING: &str = "sim/flightmodel/engine/ENGN_running";
/// Throttle a mission give-up hands the aircraft back with — autoland's own
/// number, for the same reason: enough to hold level flight dirty.
const GIVE_UP_THROTTLE: f64 = 0.75;
/// Any bus above this reads live after the battery commands: the readback
/// that lets "Battery's on." be said (a 12 volt system's battery reads
/// about 12, a 24 volt one's about 24; a dead bus 0).
const BUS_LIVE_VOLTS: f64 = 8.0;
const COM1_FREQ_KHZ: &str = crate::datarefs::COM1_FREQ_KHZ;

/// True while the full-auto orchestrator runs.
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::new();
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

/// THE arrival runway of the mission flying: chosen at the arm (the crew's
/// own STAR pick, else the plan's `destination/plan_rwy`), and the one value
/// the inbound calls, the arrival's setup, the ILS tune and the landing read.
/// LIBP-LEMG (2026-09-26): the arm chose UNTO2A for 12 on a 120/9 wind, and
/// the arrival re-picked from the METAR 45 minutes later, tuned and flew the
/// ILS 13, so the STAR flown and the runway landed on were decided twice.
/// Written only through [`set_arrival_runway`]: the arm, the arrival's own
/// safety overrule (tailwind, no approach we can fly), the tower's runway.
/// After the arrival has set it up, a change of end goes through
/// [`retarget_arrival_runway`] and nowhere else: autoland's tower switch
/// (`autoland::clearance_switch`) today, the owed in-flight re-pick (#79)
/// tomorrow. Cleared with the mission (`stop_active`).
static ARRIVAL_RUNWAY: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

/// The mission's arrival runway ([`ARRIVAL_RUNWAY`]); `None` with no mission,
/// a circuit, or a plan that names none.
pub fn arrival_runway() -> Option<String> {
    ARRIVAL_RUNWAY.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// Set the mission's arrival runway, blackboxed `arrival_runway` with who
/// set it whenever it changes.
pub fn set_arrival_runway(rwy: Option<String>, why: &str) {
    let mut cur = ARRIVAL_RUNWAY.lock().unwrap_or_else(|e| e.into_inner());
    if *cur != rwy {
        event("arrival_runway", json!({ "was": *cur, "now": rwy, "why": why }));
        *cur = rwy;
    }
}

/// THE write point for a change of arrival runway after the arm: the tower
/// clearing us for another end inside autoland (`autoland::clearance_switch`,
/// at its arm and on the approach), and the in-flight re-pick still owed
/// (#79). It MOVES a runway the mission holds and never invents one: with no
/// mission, a circuit, or a plan that named none, there is nothing to move,
/// so a standalone autoland leaves the value `None` (a stale end must not
/// outlive its mission; `stop_active` clears it). True when the mission's
/// runway is now `rwy`. Blackboxed `arrival_runway` with `why` on a change.
///
/// LIBP-LEMG seam (issue #64): a tower switch re-armed autoland on the new
/// end while this value stayed on the old one, so every reader after the
/// switch read a runway nobody was flying.
pub fn retarget_arrival_runway(rwy: &str, why: &str) -> bool {
    let rwy = rwy.trim();
    if rwy.is_empty() {
        return false;
    }
    let mut cur = ARRIVAL_RUNWAY.lock().unwrap_or_else(|e| e.into_inner());
    let Some(was) = cur.clone() else {
        return false;
    };
    if !was.eq_ignore_ascii_case(rwy) {
        event("arrival_runway", json!({ "was": was, "now": rwy, "why": why }));
        *cur = Some(rwy.to_string());
    }
    true
}

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
    crate::autotaxi::halt();
    crate::autotakeoff::cancel();
    crate::circuit::abort();
    crate::procedure::abort();
    crate::autoland::abort();
    forget_mission();
    ACTIVE.swap(false, SeqCst)
}

/// The mission has a departure's minimum altitudes ahead: its takeoff climbs
/// at best rate (`autotakeoff`'s climb-out). Set for its takeoff, cleared
/// once airborne and with the waiver.
static DEPARTURE_MINIMUMS: AtomicBool = AtomicBool::new(false);

/// Whether the mission's takeoff has departure minimums ahead.
pub fn departure_minimums() -> bool {
    DEPARTURE_MINIMUMS.load(SeqCst)
}

/// The mission's own facts die with it: at its start, every stop, a new
/// flight (`flight_state::forget_flight`) and a new airframe. The runway
/// waiver the pilot gave at this mission's arm (`Gate::OriginRunway`) is
/// the mission's too (`aircraft::set_takeoff_margin_waived`, stored in the
/// lib for its runway rule): no other takeoff ever inherits it.
pub(crate) fn forget_mission() {
    crate::aircraft::set_takeoff_margin_waived(false);
    DEPARTURE_MINIMUMS.store(false, SeqCst);
}

/// Full auto stops: the flag, and the mission facts that lived only for it.
fn stop_active() {
    forget_mission();
    set_arrival_runway(None, "mission_ended");
    // The plan's transition was armed for this mission; the monitor's
    // altimeter check must not judge the next flight against it.
    crate::tools::instruments::publish_flight_transition(None);
    ACTIVE.store(false, SeqCst);
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
#[derive(Clone)]
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
    /// The planned STAR's restrictions on this route, already clamped into
    /// `legs` (`procedure::apply_arrival_constraints`). Empty with no STAR.
    pub arrival_constraints: Vec<crate::nav::StarConstraint>,
    /// The runway the planned SID departs from, asked for at the taxi request.
    pub departure_rwy: Option<String>,
    /// The plan's origin, and the runway the plan departs from with or
    /// without a SID: an uncontrolled departure plans its own taxi to it.
    pub origin_icao: String,
    pub origin_rwy: Option<String>,
    /// The plugin FMS-mailbox payload ("IDENT,lat,lon,alt|...") for the whole
    /// plan, so the GPS shows the route too. Best-effort — the follower flies
    /// the plan's own fixes, not the FMS.
    pub fms_payload: String,
    /// The landing runway chosen at the arm: the one the STAR flown serves
    /// (`plan_audit::choose_arrival`'s pick, else the plan's own), published
    /// as [`arrival_runway`] for the whole mission.
    pub arrival_rwy: Option<String>,
    /// The STAR flown to it (`plan_audit::choose_arrival`'s pick, else the
    /// plan's): an in-flight re-pick keeps it when it serves the new runway.
    pub arrival_star: Option<String>,
    /// The planned SID's ident, when the plan has one (`general/sid_ident`):
    /// the departure an IFR clearance's SID is measured against.
    pub sid: Option<String>,
    /// The mission flies IFR ([`plan_is_ifr`]): the tower is asked for the
    /// IFR clearance to `dest_icao`, not the zone clearance.
    pub ifr: bool,
    /// The OFP's cruise schedule (`simbrief::CruiseSchedule`): the follower
    /// flies each cruise leg at its planned speed and says each planned step
    /// (`procedure::scheduled_cruise_kts`, `step_climb_at`). The levels are
    /// already in `legs` (`procedure::apply_step_climbs` at the arm).
    pub cruise_schedule: crate::simbrief::CruiseSchedule,
    /// The FILED cruise level, feet (the plan's `initial_altitude`): the
    /// highest the departure check-in asks for. `None` when the plan has none.
    pub cruise_ft: Option<f64>,
}

/// The climb the departure check-in announces, the one departure then clears:
/// the highest level the route's legs hold, never above the FILED cruise.
/// The legs carry the STAR's windows too (`apply_arrival_constraints`), and
/// EGKK-EGBB (sortie 1790601822-full_auto, 2026-09-28, #155) filed FL180
/// checked in "climbing flight level two two zero", HEME1B's HEMEL window,
/// and was cleared to it. Pure.
pub fn check_in_climb_ft(legs: &[crate::nav::ProcLeg], filed_cruise_ft: Option<f64>) -> f64 {
    let highest = legs.iter().filter_map(|l| l.alt1_ft).fold(0.0, f64::max);
    match filed_cruise_ft.filter(|c| c.is_finite() && *c > 0.0) {
        Some(c) if highest > c => c,
        _ => highest,
    }
}

/// Does this plan fly IFR? A SimBrief plan is an airline plan and always
/// does; a local plan does when it cruises above the field's transition
/// altitude (the plan's own figure, else the airframe's): a flight planned
/// into the flight levels is not a zone departure. Pure.
pub fn plan_is_ifr(
    source: crate::simbrief::PlanSource,
    cruise_ft: f64,
    transition_ft: f64,
) -> bool {
    source == crate::simbrief::PlanSource::SimBrief || cruise_ft > transition_ft
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
    /// Whether the ramp items include the panel lighting (the profile's
    /// `ramp_panel_lights`).
    pub panel_lights: bool,
    /// The crew's preflight between the battery and the ramp items
    /// ([`crate::preflight`]), and the Boeing start whose APU steps it
    /// brings the APU up with.
    pub preflight: Option<crate::aircraft::PreflightProfile>,
    pub preflight_apu: Option<crate::aircraft::BoeingStartProfile>,
    /// The start tool answers only once the start is over
    /// (`StartKind::start_is_inline`): its reply is the verdict.
    pub start_inline: bool,
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
    /// The aircraft's own script starts it (`StartKind::Scripted`, the
    /// 737): the script resets the ramp to cold and dark as it begins, so
    /// the ramp items are a check AFTER it, never set before it.
    pub start_scripted: bool,
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
    // Never over a loop already flying (`flight_state::crew_flying`, the
    // helicopter's loops included), nor one still winding down (`running`).
    if crate::flight_state::crew_flying().is_some()
        || crate::procedure::running()
        || crate::autotakeoff::running()
    {
        return false;
    }
    let Some(lease) = RUN.begin() else {
        return false;
    };
    if ACTIVE.swap(true, SeqCst) {
        return false;
    }
    // The runway margin the pilot waived at THIS arm's preflight, if any,
    // lives exactly as long as the mission (`stop_active`).
    forget_mission();
    crate::aircraft::set_takeoff_margin_waived(RUNWAY_WAIVER_PENDING.swap(false, SeqCst));
    ABORT.store(false, SeqCst);
    INBOUND_MADE.store(false, SeqCst);
    set_arrival_runway(
        match &tools.mission {
            Mission::Flight(r) | Mission::Resume(r) => r.arrival_rwy.clone(),
            Mission::Circuit { .. } => None,
        },
        "arm",
    );
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
            stop_active();
            return;
        };
        rt.block_on(fly(
            XPlaneClient::new(base_url).with_cancel(_lease.cancel.clone()),
            config,
            voice,
            tools,
        ));
        stop_active();
        clear_stage();
    });
    true
}

/// How long a line of the mission's OWN narration may wait to be said.
/// "Battery on" heard twelve seconds after the battery, with the engine
/// already turning, is worse than silence; behind a long reply or a tower
/// transmission the old 12 s let a whole queue form and play late (LIPK
/// 2026-09-26). Anything sterner than Advisory is never perishable.
const NARRATION_STALE_S: u64 = 6;

fn announce(voice: &Option<Voice>, priority: Priority, line: &str) {
    tracing::info!(target: "xvoicelayer::full_auto", spoke = %line, "copilot");
    if let Some(v) = voice {
        if priority == Priority::Advisory {
            v.narrate(priority, line, NARRATION_STALE_S);
        } else {
            v.alert(priority, line.to_string());
        }
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
    light_state(tools, ctx, key, on).await == Some(true)
}

/// The ramp items before anything turns: beacon, nav lights, panel
/// lighting, seat belt signs, each called only once its switch READ BACK
/// on and named when it did not take. True when the beacon is on.
async fn ramp_items(
    tools: &SubTools,
    ctx: &ToolCtx<'_>,
    xp: &XPlaneClient,
    voice: &Option<Voice>,
) -> bool {
    let beacon_state = light_state(tools, ctx, "beacon", true).await;
    let beacon = beacon_state == Some(true);
    if tools.lights.declares("beacon") {
        crate::crew_pace::checked(
            voice.as_ref(),
            beacon_state,
            "Beacon on.",
            "The beacon did not come on.",
        )
        .await;
    }
    if tools.lights.declares("nav") {
        let nav = light_state(tools, ctx, "nav", true).await;
        crate::crew_pace::checked(
            voice.as_ref(),
            nav,
            "Nav lights on.",
            "Nav lights did not come on.",
        )
        .await;
    }
    // Panel lighting with them. The bus is live and the pilot is about to
    // be asked to read instruments. The tool's Ok is not a readback: the
    // switch is read back here. Left out where the airframe declares no
    // path to it (`ramp_panel_lights: false`): the Zibo's rheostats are its
    // own, the stock write is undone, and "did not come on" was said about
    // a switch we never had (EGHL 2026-10-06).
    if !tools.panel_lights {
        event("lights_panel", json!({ "skipped": "undeclared" }));
    } else {
        use crate::tools::Tool;
        let sent = tools
            .flood
            .execute(&json!({ "on": true }), ctx)
            .await
            .is_ok();
        let lit = if sent {
            tools.flood.read_on(xp).await
        } else {
            Some(false)
        };
        event("lights_panel", json!({ "on": lit }));
        crate::crew_pace::checked(
            voice.as_ref(),
            lit,
            "Panel lights on.",
            "Panel lights did not come on.",
        )
        .await;
    }
    event("lights_beacon", json!({ "beacon": beacon_state }));
    // Seat belt signs on before anything turns (see `tools::signs`).
    if let Some(ok) = crate::tools::signs::set(xp, true).await {
        event("seat_belt_signs", json!({ "on": true, "ok": ok }));
        crate::crew_pace::checked(
            voice.as_ref(),
            Some(ok),
            "Seat belt signs on.",
            "The seat belt signs did not come on.",
        )
        .await;
    }
    beacon
}

/// A light driven and READ BACK (`SetLights::switch`): `None` when the
/// airframe declares no such light or the readback failed, `Some(false)`
/// when the switch did not take. The tool's reply used to be the test, and
/// it answers an unknown light with an Ok QUESTION ("I didn't catch which
/// light...") and a switch that did not move with an Ok "didn't move", both
/// counted as done under `.is_ok()`: the SR22, SF50 and Lancair have no
/// beacon, and full auto announced "Beacon on" on all three (audit,
/// 2026-08-02).
async fn light_state(tools: &SubTools, ctx: &ToolCtx<'_>, key: &str, on: bool) -> Option<bool> {
    tools.lights.switch(ctx.xplane, key, on).await
}

/// A mission's sortie in the blackbox: begun at the mission's start, ended
/// by `give_up` ("gave_up") or, on any other exit, when this drops ("ended").
pub struct SortieGuard;

impl SortieGuard {
    pub fn begin(label: &str, detail: serde_json::Value) -> Self {
        crate::blackbox::begin_sortie(label, detail);
        SortieGuard
    }
}

impl Drop for SortieGuard {
    fn drop(&mut self) {
        crate::blackbox::end_sortie("ended", json!({}));
    }
}

/// Give up cleanly.
/// Every call full auto makes goes out through here: once the mission is
/// aborted it says nothing more. A step already past its abort check queued
/// "request IFR clearance" and the fallback "request clearance" 0.3 s after
/// a cancel, and both went out 34 and 57 s later from a mission that had
/// handed back (LXGB 2026-10-06, sortie 1791289470-full_auto).
fn radio_out(call: String) {
    if ABORT.load(SeqCst) {
        event("radio_withheld", json!({ "call": call }));
        return;
    }
    crate::crew_radio::transmit(call);
}

/// [`radio_out`] for a readback.
fn radio_readback(call: String, owed_copy: bool) {
    if ABORT.load(SeqCst) {
        event("radio_withheld", json!({ "call": call, "readback": true }));
        return;
    }
    crate::crew_radio::transmit_readback(call, owed_copy);
}

async fn give_up(
    xp: &XPlaneClient,
    valve_set: Option<f64>,
    voice: &Option<Voice>,
    line: &str,
    reason: &str,
) {
    // A wait the pilot's "my controls" cut short is a cancel, never the
    // stage's own failure: at LEMG (2026-09-28, sortie 1790586620) the taxi
    // wait saw the abort and said "The taxi didn't finish" as `taxi_stalled`.
    let (line, reason) = give_up_words(line, reason, ABORT.load(SeqCst));
    RUN.fail(reason);
    stop_active();
    // "Your aircraft" means every lever: the mixture manager stands down with
    // the rest, never quietly re-trimming under the pilot's hand.
    crate::tools::engine::release_mixture();
    event("give_up", json!({ "reason": reason }));
    // A request or readback the mission queued and the main loop has not
    // yet sent is the mission's, and the mission is over.
    let withdrawn = crate::crew_radio::withdraw_queued();
    if !withdrawn.is_empty() {
        event("radio_withdrawn", json!({ "calls": withdrawn }));
    }
    crate::blackbox::end_sortie("gave_up", json!({ "reason": reason, "line": line }));
    // Held before it is said: the line claims the pilot has the aircraft, so
    // whatever is going to hold it should already be holding it.
    let held = hand_back_brakes(xp, valve_set).await;
    clear_stage();
    announce(voice, Priority::Caution, &hand_back_line(line, held));
}

/// The circuit mission ended because its takeoff never completed, or the
/// circuit was refused on the runway for the ground under it (#182), and
/// not by the pilot's cancel, which is its own give-up. Pure.
fn circuit_takeoff_failed(stood_down: Option<&str>, aborted: bool) -> bool {
    !aborted
        && matches!(
            stood_down,
            Some(crate::circuit::TAKEOFF_NOT_COMPLETED | crate::circuit::TERRAIN_REFUSED)
        )
}

/// How long an inline start's engines are given to read RUNNING once the
/// start has answered: its stable call reads N2, the running flag can lag a
/// moment behind it.
const START_SETTLE_SECS: f64 = 10.0;

/// The hand-back owed when an inline start has answered and the engines are
/// not running: its reply as the line (it says why, and ends "Your
/// aircraft."), or a plain one when the tool failed outright. `None` when
/// the engines run. Pure.
fn inline_start_hand_back(running: bool, reply: Option<&str>) -> Option<(String, &'static str)> {
    if running {
        return None;
    }
    let line = match reply.map(str::trim).filter(|r| !r.is_empty()) {
        Some(r) if r.contains("Your aircraft") => r.to_string(),
        Some(r) => format!("{r} The engines aren't running. Your aircraft."),
        None => "The start didn't run. Your aircraft.".to_string(),
    };
    Some((line, "start_stopped"))
}

/// The preflight's closing line: only the switches that did not take, by
/// name. A clean preflight has said its milestones already. Pure.
fn preflight_line(not_taken: &[String]) -> Option<String> {
    match not_taken {
        [] => None,
        [one] => Some(format!("Preflight done, but the {one} didn't take.")),
        many => Some(format!(
            "Preflight done, but {} switches didn't take: {}.",
            many.len(),
            many.join(", ")
        )),
    }
}

/// The give-up's line and reason, relabelled as the pilot's cancel when the
/// abort flag is what ended the wait. Pure.
fn give_up_words<'a>(line: &'a str, reason: &'a str, aborted: bool) -> (&'a str, &'a str) {
    if aborted && reason != "abort" {
        ("Cancelled. Your aircraft.", "abort")
    } else {
        (line, reason)
    }
}

/// The give-up's line with what the hand-back did to the brakes: said only
/// when it was a ground hand-back (`None` leaves the line alone). Pure.
fn hand_back_line(line: &str, held: Option<bool>) -> String {
    match held {
        Some(true) => format!("{line} Parking brake set."),
        Some(false) => format!("{line} The parking brake didn't set, hold the brakes."),
        None => line.to_string(),
    }
}

/// ...and on the ground it means the aircraft is STOPPED and the PARKING
/// BRAKE holds it, not a control loop of ours and not nothing: the one
/// stop-and-park, [`crate::tools::controls::hand_back_hold`].
async fn hand_back_brakes(xp: &XPlaneClient, valve_set: Option<f64>) -> Option<bool> {
    // Unscoped: the give-up has already cancelled this mission's client.
    let free = xp.unscoped();
    hand_back_hold(&free, valve_set, HAND_BACK).await
}

// The mission and its stages in their own files, whole and unchanged
// (refactor phase 4); every path stays `crate::full_auto::...` through these
// re-exports.
mod clearances;
pub use clearances::*;
mod waits;
pub use waits::*;
mod mission;
use mission::*;
mod route;
pub(crate) use route::*;
mod arrival;
pub use arrival::*;
mod departure;
pub use departure::*;
mod gates;
pub use gates::*;

/// The one stop-and-park lives beside the parking brake's seam now; the
/// mission's name for it stays.
pub(crate) use crate::tools::controls::{hand_back_hold, HoldFor};
/// The mission's hand-back: a tug or an unread speed lets go.
const HAND_BACK: HoldFor = HoldFor::by("full_auto");
#[cfg(test)]
use crate::tools::controls::{
    pilot_on_the_brakes, HAND_BACK_STOPPED_KTS, TOE_BRAKE_L, TOE_BRAKE_R,
};

#[cfg(test)]
mod runway_start_tests;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod region_handoff_tests;

#[cfg(test)]
mod inbound_tests;

#[cfg(test)]
mod squawk_tests;

#[cfg(test)]
mod arrival_watch_tests;

#[cfg(test)]
mod arrival_repick_tests;
