//! Fly a published terminal procedure — a SID, STAR, or instrument approach —
//! read from X-Plane's CIFP instead of an invented pattern.
//!
//! This is the terrain-safe backbone the naive circuit vectoring never was: the
//! fixes of a real departure or approach define a corridor the airport authority
//! already threaded through the terrain and airspace. The follower is an
//! ORCHESTRATOR in the same mould as [`crate::circuit`] — it flies the AP's
//! heading bug and altitude dial, it does not invent a new flight law:
//!
//! 1. A leg list comes in already parsed and resolved to positions
//!    ([`crate::nav::ProcLeg`] with `fix` filled), plus an [`EndAction`].
//! 2. Each tick the loop steers the great-circle track to the current leg's fix
//!    (wind-crabbed, exactly like the circuit's downwind), descends/climbs to
//!    the leg's altitude constraint, and sequences to the next leg on capture.
//!    To-altitude legs (CA/VA/FA — a missed approach's "climb to 3000", a SID's
//!    initial climb) hold the leg course until the altitude is made.
//! 3. On an APPROACH it hands the FAF-inbound segment to [`crate::autoland`]
//!    (the proven glideslope + flare + rollout); on a DEPARTURE it levels at the
//!    last constraint and hands back to the pilot.
//!
//! Trust model matches the circuit: every actuation is the same AP the copilot
//! tools drive, and "my controls" / "cancel" hands back at any point. It only
//! flies what the data says — a field with no published procedure (Gibraltar)
//! gets an honest refusal at the tool, never a guessed path into the Rock.
//! EXPERIMENTAL and coarse (fix flyby, not RF arcs or holds); fly it VMC with a
//! hand ready and mine `logs/blackbox.jsonl` (`"kind":"procedure"`) to tune.

use std::sync::atomic::{AtomicBool, Ordering};

use crate::autoland::true_to_mag;
use crate::nav::ProcLeg;
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const HPATH: &str = "sim/flightmodel/position/hpath";
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const ELEV_M: &str = "sim/flightmodel/position/elevation";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
const MS_TO_KTS: f64 = 1.943_84;
// AP surface (mirror of autopilot.rs / autoland.rs / circuit.rs).
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
const HDG_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
const HDG_STATUS: &str = "sim/cockpit2/autopilot/heading_status";

const M_TO_FT: f64 = 3.280_84;

/// A leg's fix counts as reached inside this radius (a flyby simplification —
/// we don't fly RF arcs or exact fly-over turns yet). 0.8 nm keeps the corridor
/// tight without a light aircraft overshooting a sharp turn.
const CAPTURE_NM: f64 = 0.8;
/// A to-altitude leg (CA/VA/FA) is satisfied within this of its target.
const ALT_MARGIN_FT: f64 = 150.0;

/// True while the procedure follower runs.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// Set true to cancel ("cancel" / "my controls").
pub static ABORT: AtomicBool = AtomicBool::new(false);

/// The fix the follower is steering at right now. The sim's own GPS never
/// sequences while the follower flies (it holds heading, the FMS stays on
/// entry 1), so "next waypoint" read from `gps_nav_id` answered PIMOS for the
/// whole flight while the aircraft was five fixes further on (LXGB-LEMG,
/// 2026-09-12). Crew-side questions and the waypoint reminder read THIS.
#[derive(Clone, Debug, PartialEq)]
pub struct ActiveLeg {
    pub fix: String,
    pub lat: f64,
    pub lon: f64,
}

static LEG: std::sync::Mutex<Option<ActiveLeg>> = std::sync::Mutex::new(None);

/// The leg being flown, `None` between procedures or on a to-altitude leg.
pub fn active_leg() -> Option<ActiveLeg> {
    if !ACTIVE.load(Ordering::Relaxed) {
        return None;
    }
    LEG.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

fn publish_leg(leg: Option<ActiveLeg>) {
    *LEG.lock().unwrap_or_else(|e| e.into_inner()) = leg;
}

/// The plugin's `xcopilot/fms/goto` mailbox: sequences the LOADED plan to the
/// entry with this ident nearest this position, so the sim's GPS page, the
/// G1000 readouts and every `gps_*` dataref describe the fix the follower is
/// actually steering at. The follower flies the heading bug, never GPS mode,
/// so nothing else sequences the FMS: it sat on entry 1 from LXGB to LEMG
/// while the aircraft was five fixes on (2026-09-12). An ident the plan does
/// not hold (an approach leg) changes nothing on the plugin side.
pub const FMS_GOTO: &str = "xcopilot/fms/goto";

/// The mailbox payload: `IDENT,lat,lon`, six decimals like the route payload.
pub fn fms_goto_payload(ident: &str, lat: f64, lon: f64) -> String {
    format!("{},{:.6},{:.6}", ident.trim().to_ascii_uppercase(), lat, lon)
}

/// Sequence the sim's FMS to the leg's fix. Best effort: no plugin, no plan
/// loaded, or a fix outside it are all silent (the follower does not need
/// the FMS to fly).
async fn sequence_fms(xp: &XPlaneClient, idx: usize, leg: &ProcLeg) {
    let Some(f) = leg.fix.as_ref() else {
        return;
    };
    if leg.fix_ident.is_empty() {
        return;
    }
    let payload = fms_goto_payload(&leg.fix_ident, f.lat, f.lon);
    let ok = xp.set_data(FMS_GOTO, payload.as_bytes()).await.is_ok();
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "fms_goto", "idx": idx, "fix": leg.fix_ident, "ok": ok}),
    );
}

/// Great-circle distance and, at a usable groundspeed, minutes to the leg's fix.
pub fn leg_progress(leg: &ActiveLeg, lat: f64, lon: f64, gs_kts: f64) -> (f64, Option<f64>) {
    let dist = crate::nav::haversine_nm(lat, lon, leg.lat, leg.lon);
    let mins = (gs_kts > 20.0).then(|| dist / gs_kts * 60.0);
    (dist, mins)
}

/// The follower's live line for the panel: the fix, the miles and minutes
/// to it, and the altitude it is flying toward. Set every tick from the
/// loop's own numbers, cleared with the loop.
pub static DETAIL: crate::auto_status::Detail = crate::auto_status::Detail::new();

/// One-line status for the dashboard's "managing" panel.
pub fn status() -> Option<String> {
    ACTIVE
        .load(Ordering::Relaxed)
        .then(|| DETAIL.line("flying a procedure"))
}

/// "to PIMOS, 12.3 nm, 6 min, climbing to 5,500 ft". A to-altitude leg has
/// no fix and no range; within 100 ft of the target the aircraft is level.
/// Ranges to a tenth, minutes whole, altitude to 10 ft: rounded so the
/// change-gated panel feed is not re-sent for jitter.
pub fn leg_detail(
    fix: &str,
    dist_nm: Option<f64>,
    gs_kts: f64,
    alt_ft: f64,
    want_alt: f64,
    owner: AltOwner,
) -> String {
    let mut parts = Vec::new();
    if !fix.is_empty() {
        parts.push(format!("to {fix}"));
    }
    if let Some(d) = dist_nm {
        parts.push(format!("{d:.1} nm"));
        if gs_kts > 20.0 {
            parts.push(format!("{:.0} min", d / gs_kts * 60.0));
        }
    }
    let alt = crate::auto_status::commas((want_alt / 10.0).round() as i64 * 10);
    let diff = want_alt - alt_ft;
    parts.push(if diff.abs() < 100.0 {
        format!("level at {alt} ft")
    } else if diff > 0.0 {
        format!("climbing to {alt} ft")
    } else {
        format!("descending to {alt} ft")
    });
    // Whose number that is, so a glance at the panel says why the follower is
    // not on the plan's altitude.
    match owner {
        AltOwner::Terrain => parts.push("terrain floor".to_string()),
        AltOwner::Pilot => parts.push("your altitude".to_string()),
        AltOwner::Plan => {}
    }
    parts.join(", ")
}

/// Who set the altitude the follower is flying.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AltOwner {
    Plan,
    Pilot,
    Terrain,
}

/// Cancel an active procedure and stand down whatever sub-loop is flying;
/// returns whether one was running. Autoland (if the approach handed off) is
/// aborted too — the pilot gets the aircraft.
pub fn abort() -> bool {
    ABORT.store(true, Ordering::SeqCst);
    crate::autoland::abort();
    ACTIVE.swap(false, Ordering::SeqCst)
}

/// What to do when the last flyable leg is reached.
#[derive(Clone)]
pub enum EndAction {
    /// Instrument approach: hand the FAF-inbound segment to autoland.
    Approach {
        apt_dat: String,
        cfg: ApConfig,
        approach_ias: f64,
        /// The approach's runway END ("12"), handed to autoland as its forced
        /// runway. `None` let autoland self-pick by track, and at a field
        /// with parallels that put the ILS 12 arrival on 13 against a
        /// clearance for 12 (LEMG, 2026-09-12).
        rwy_id: Option<String>,
    },
    /// Departure/arrival that ends in the enroute structure: level at the last
    /// constraint and hand back to the pilot.
    Level,
    /// Enroute segment flown as one phase of a larger flight (the full-auto
    /// SimBrief mission): complete QUIETLY — no "your aircraft", the flight
    /// orchestrator is watching `ACTIVE` and owns what happens next.
    Enroute,
}

/// Everything the follower needs, resolved by the arming tool.
pub struct Plan {
    /// Spoken name of the procedure, e.g. "the ILS approach runway two seven".
    pub name: String,
    /// Legs, already resolved (`fix` filled where a fix exists). Unresolved fix
    /// legs are skipped in flight.
    pub legs: Vec<ProcLeg>,
    pub end: EndAction,
    /// Best-climb speed reference for the SPEED-PROTECTED climb law; 0 = no
    /// protection (a descent-only approach plan). VS mode holds RATE whatever
    /// the airspeed: an unprotected 700 fpm enroute climb bled 86 knots down
    /// to a 53-knot stall departure at 4,600 ft (LEJR, 2026-07-24).
    pub vy_kts: f64,
    /// Indicated cruise speed for the cruise-power governor. `Some` on an enroute
    /// flight arms the speed-hold (throttle finds the power to hold it) for the
    /// whole leg, released at handoff; `None` leaves the throttle alone (the
    /// climb-out setting rides through — approaches/SIDs, which manage their own).
    pub cruise_ias_kts: Option<f64>,
}

/// The true great-circle bearing to a leg's fix, or `None` for a to-altitude
/// leg (no fix to steer at). Pure — the lateral-steering decision, testable
/// without a sim.
pub fn bearing_to_fix(lat: f64, lon: f64, leg: &ProcLeg) -> Option<f64> {
    leg.fix
        .as_ref()
        .map(|f| crate::nav::bearing_deg(lat, lon, f.lat, f.lon))
}

/// Has the current leg been flown? A fix leg completes inside [`CAPTURE_NM`]; a
/// to-altitude leg completes when the altitude constraint is satisfied. Pure and
/// position-based, so sequencing is immune to how slow the AP's turns are (the
/// same lesson that drove the circuit off timers).
pub fn leg_captured(leg: &ProcLeg, dist_to_fix_nm: Option<f64>, alt_ft: f64) -> bool {
    match dist_to_fix_nm {
        Some(d) => d <= CAPTURE_NM,
        // No fix: a to-altitude leg. '+' climb to at-or-above, '-' descend to
        // at-or-below, otherwise cross at the altitude.
        None => match leg.alt1_ft {
            Some(target) => match leg.alt_desc {
                '+' => alt_ft >= target - ALT_MARGIN_FT,
                '-' => alt_ft <= target + ALT_MARGIN_FT,
                _ => (alt_ft - target).abs() <= ALT_MARGIN_FT,
            },
            // A heading leg with no altitude (rare, FM/VM manual) — nothing to
            // wait for; treat as instantly done so we don't stall the sequence.
            None => true,
        },
    }
}

/// The altitude-dial target for a leg, feet MSL: its own constraint when it has
/// one, else `carry` (hold the last commanded altitude across an unconstrained
/// fix). The AP's ALT hold flies toward it — climb or descent falls out of
/// whether the target is above or below.
pub fn commanded_alt_ft(leg: &ProcLeg, carry: f64) -> f64 {
    leg.alt1_ft.unwrap_or(carry)
}

/// Clearance the follower keeps over the highest ground AHEAD, feet. The
/// non-mountainous IFR figure, and the least an instructor would accept from
/// an automation flying a leg it cannot see.
pub const TERRAIN_CLEARANCE_FT: f64 = 1_000.0;
/// How far ahead along the track the terrain probe looks, nm. Far enough that
/// a 700 fpm climb clears what it finds at light-aircraft speeds (15 nm is
/// over six minutes at 135 kt), short enough that the answer is about the leg
/// being flown and not the next county.
const TERRAIN_LOOK_NM: f64 = 15.0;
/// Seconds between terrain probes. The ground does not move; this is a picture
/// of what is ahead, not a control input.
const TERRAIN_PROBE_SECS: f64 = 10.0;
/// Points per probe. The plugin answers at most [`crate::terrain::MAX_POINTS`].
const TERRAIN_POINTS: usize = 16;
/// Re-speak the terrain floor when it rises by more than this, feet.
const TERRAIN_RESAY_FT: f64 = 200.0;

/// The lowest altitude the follower may command: the highest ground within
/// [`TERRAIN_LOOK_NM`] ahead plus [`TERRAIN_CLEARANCE_FT`]. `None` means no
/// terrain answer (old plugin, probe timed out) and the floor is not applied:
/// the module's standing contract, degrade rather than refuse.
///
/// This exists because a plan's altitude is only as good as the ground under
/// it. The LEMG-LEJR leg to JRZ carried 2,700 ft across the Sierra de
/// Grazalema, whose ridge probes at 3,861 ft; the follower flew it down to
/// 220 ft AGL (2026-09-13). The planner floors its own CRUISE altitude over
/// this same probe, but a constraint handed down from a plan was flown as
/// given. Nothing the follower commands is trusted over the ground again.
static TERRAIN_FLOOR: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// The highest-ground floor the follower is currently holding itself to, feet
/// MSL. `None` outside a procedure or with no terrain answer. Read by the
/// autopilot tool so a PILOT's altitude is judged against the same ground the
/// follower can see ahead, not merely what is under the wheels.
pub fn terrain_floor_ahead_ft() -> Option<f64> {
    if !ACTIVE.load(Ordering::Relaxed) {
        return None;
    }
    *TERRAIN_FLOOR.lock().unwrap_or_else(|e| e.into_inner())
}

fn publish_terrain_floor(v: Option<f64>) {
    *TERRAIN_FLOOR.lock().unwrap_or_else(|e| e.into_inner()) = v;
}

/// An altitude the PILOT commanded while the follower was flying.
///
/// The follower is the ONE writer of the altitude dial and rewrites it every
/// tick. So a spoken "climb to four thousand" was obeyed by the autopilot tool
/// and undone within 500 ms, twice, while the aircraft oscillated at 220 ft
/// over rising ground (2026-09-13): the pilot's own override was the thing the
/// automation was strongest against. It is now not a fight over a dataref: the
/// tool hands the number here and the follower flies it as its target until the
/// pilot says another one, keeping every leg's lateral steering.
static PILOT_ALT: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Hand the follower an altitude the pilot asked for. `false` (and nothing
/// taken) when no procedure is flying, so the caller drives the autopilot
/// itself the way it always has.
pub fn take_altitude(ft: f64) -> bool {
    if !ACTIVE.load(Ordering::Relaxed) {
        return false;
    }
    *PILOT_ALT.lock().unwrap_or_else(|e| e.into_inner()) = Some(ft);
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "pilot_altitude", "ft": ft}),
    );
    true
}

/// The pilot's commanded altitude, if they have taken the vertical.
pub fn pilot_altitude() -> Option<f64> {
    if !ACTIVE.load(Ordering::Relaxed) {
        return None;
    }
    *PILOT_ALT.lock().unwrap_or_else(|e| e.into_inner())
}

fn clear_pilot_altitude() {
    *PILOT_ALT.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// Points along the ground track from `lat`/`lon` on `brg_true`, out to
/// `look_nm`, for the terrain probe. Flat-earth stepping: over 15 nm the error
/// is metres, and the answer is a maximum over samples either way.
pub fn look_ahead_points(
    lat: f64,
    lon: f64,
    brg_true: f64,
    look_nm: f64,
    n: usize,
) -> Vec<(f64, f64)> {
    let n = n.max(1);
    let b = brg_true.to_radians();
    let coslat = lat.to_radians().cos().abs().max(0.01);
    (1..=n)
        .map(|i| {
            let d = look_nm * i as f64 / n as f64;
            (
                lat + b.cos() * d / 60.0,
                lon + b.sin() * d / (60.0 * coslat),
            )
        })
        .collect()
}

/// The descent rate the VS law flies, fpm (its clamp).
const DESCENT_FPM: f64 = 700.0;

/// The rate the follower's vertical law will actually fly, fpm (magnitude):
/// what a readback must promise when the follower owns the altitude, whatever
/// rate the pilot asked for.
pub fn vs_clamp_fpm() -> f64 {
    DESCENT_FPM
}
/// Start down this far before the computed top of descent, nm.
const TOD_LEAD_NM: f64 = 1.0;

/// The altitude to command NOW for a leg that wants `leg_alt` at a fix
/// `dist_nm` away: the previous target (`carry`, or where we are if we never
/// reached it) until the top of descent, then the leg's own. Dialling a
/// constraint straight in began the descent the moment the leg armed — the
/// Gibraltar arrival dialled 1,397 ft with 25 nm to run and flew 18 nm level
/// at 1,400 ft across the Strait, past Europa Point (2026-09-11). Climbs are
/// untouched (the climb law owns them), so is a small step, and a leg with no
/// fix has nothing to pace against.
pub fn descent_target_ft(
    leg_alt: f64,
    carry: f64,
    alt_ft: f64,
    dist_nm: Option<f64>,
    gs_kts: f64,
) -> f64 {
    let Some(dist_nm) = dist_nm else {
        return leg_alt;
    };
    let drop_ft = alt_ft - leg_alt;
    if drop_ft <= 100.0 {
        return leg_alt;
    }
    let tod_nm = drop_ft / DESCENT_FPM * (gs_kts.max(60.0) / 60.0) + TOD_LEAD_NM;
    if dist_nm <= tod_nm {
        return leg_alt;
    }
    if carry <= alt_ft + 100.0 {
        carry
    } else {
        (alt_ft / 100.0).round() * 100.0
    }
}

/// One tick of the follower's vertical bookkeeping, pure so every rule below
/// is a plain `cargo test` and not a sortie.
///
/// `carry` is the last leg CONSTRAINT (carried across unconstrained fixes);
/// `prev_cmd` is what was actually COMMANDED last tick, which is what the
/// top-of-descent deferral paces against. Conflating the two defeated the
/// deferral after one tick per leg (2026-09-13, see [`descent_target_ft`]).
#[derive(Debug, Clone, PartialEq)]
pub struct Vertical {
    /// The altitude to command now.
    pub want_alt: f64,
    /// `carry` for the next tick.
    pub carry: f64,
    /// `prev_cmd` for the next tick.
    pub prev_cmd: f64,
    /// The terrain floor, when it is what set `want_alt`.
    pub floored: Option<f64>,
    /// True when the pilot's altitude is what set `want_alt`.
    pub pilot: bool,
}

/// Order of precedence, each one a rule learned the hard way: the plan's
/// constraint paced to its top of descent, then the PILOT's altitude if they
/// have taken it, then the ground AHEAD, which outranks everyone: no target
/// the follower commands may be into terrain, whoever asked for it.
pub fn vertical_tick(
    leg: &ProcLeg,
    carry: f64,
    prev_cmd: f64,
    alt_ft: f64,
    dist_nm: Option<f64>,
    gs_kts: f64,
    pilot: Option<f64>,
    terrain_floor: Option<f64>,
) -> Vertical {
    let leg_alt = commanded_alt_ft(leg, carry);
    let mut want_alt = descent_target_ft(leg_alt, prev_cmd, alt_ft, dist_nm, gs_kts);
    let from_pilot = pilot.is_some();
    if let Some(p) = pilot {
        want_alt = p;
    }
    let mut floored = None;
    if let Some(floor) = terrain_floor {
        if want_alt < floor {
            want_alt = floor;
            floored = Some(floor);
        }
    }
    Vertical {
        want_alt,
        carry: leg_alt,
        prev_cmd: want_alt,
        floored,
        pilot: from_pilot && floored.is_none(),
    }
}

/// The leg to hand off to autoland on an approach: the FAF if the data flags one,
/// else the last fix leg before the runway threshold, else the last leg. `None`
/// only when there are no legs. Handing off AT the FAF lets autoland own the
/// glideslope from platform to touchdown, exactly where it's proven.
pub fn handoff_index(legs: &[ProcLeg]) -> Option<usize> {
    if legs.is_empty() {
        return None;
    }
    if let Some(i) = legs.iter().position(|l| l.is_faf) {
        return Some(i);
    }
    // The leg just before the first runway-threshold ("RW..") leg.
    if let Some(rw) = legs.iter().position(|l| l.fix_ident.starts_with("RW")) {
        return Some(rw.saturating_sub(1));
    }
    Some(legs.len() - 1)
}

/// A human word for the approach type from its ident's leading letter.
pub(crate) fn approach_kind(ident: &str) -> &'static str {
    match ident.chars().next() {
        Some('I') => "ILS",
        Some('X') | Some('R') | Some('Q') => "RNAV",
        Some('L') => "localizer",
        Some('D') | Some('S') | Some('V') => "VOR",
        _ => "instrument",
    }
}

/// Build an APPROACH plan for `rwy_id` at `icao`: pick the best published
/// approach, load its final segment, resolve every fix against the nav data.
/// `Err` carries an honest refusal (no published approach / unreadable legs) for
/// the caller to speak. Shared by the `fly_procedure` tool and the circuit's
/// arrival handoff so both build the arrival the same way.
#[allow(clippy::too_many_arguments)]
pub fn approach_plan_for(
    cifp: &str,
    fix_path: &str,
    nav_path: &str,
    airport: &crate::nav::AirportInfo,
    icao: &str,
    rwy_id: &str,
    lat: f64,
    lon: f64,
    end: EndAction,
) -> Result<Plan, String> {
    let spoken = crate::circuit::spell_runway(rwy_id);
    // Prefer a full CIFP approach; fall back to a straight-in reconstructed from
    // the runway's published DME fixes (the way a pilot flies the DME approach
    // when there's no coded procedure — Gibraltar's whole story).
    let (legs, name) = match crate::nav::best_approach(cifp, icao, rwy_id) {
        Some(appch) => {
            let mut legs =
                crate::nav::load_proc_legs(cifp, icao, crate::nav::ProcKind::Appch, &appch, None);
            crate::nav::resolve_proc_fixes(fix_path, nav_path, airport, icao, &mut legs, lat, lon);
            (
                legs,
                format!("the {} approach runway {spoken}", approach_kind(&appch)),
            )
        }
        None => (Vec::new(), String::new()),
    };
    let (legs, name) = if legs.is_empty() {
        let recon = crate::nav::reconstruct_approach(fix_path, airport, icao, rwy_id);
        if recon.is_empty() {
            return Err(format!(
                "{icao} has no published approach to runway {spoken}. No coded procedure and no \
                 DME approach fixes to reconstruct one from."
            ));
        }
        (recon, format!("a straight-in approach to runway {spoken}"))
    } else {
        (legs, name)
    };
    // The FAF is where autoland takes the aircraft, so it has to be somewhere
    // autoland can land FROM. A published approach's FAF is on the course by
    // definition; a reconstructed one is only as good as its fixes.
    if let Some(why) = faf_off_course(&legs, airport, rwy_id) {
        return Err(why);
    }
    // Approaches descend; the climb protection reference stays off unless the
    // caller sets one (e.g. the enroute follower).
    Ok(Plan {
        name,
        legs,
        end,
        vy_kts: 0.0,
        cruise_ias_kts: None, // the follower holds the clean approach speed
    })
}

/// The refusal when the handoff fix is not somewhere autoland can intercept the
/// runway from: the FAF's cross-track and along-track distances in the runway
/// frame have to pass autoland's own `interceptable` test, the one the circuit
/// applies before it hands over. Gibraltar's 09D05 sits 3.6 km abeam the 09
/// centreline with 5.5 km to run (a 50 degree offset); handing off there flew a
/// saturated 35 degree cut that was still 300 m off at 95 ft, went around, and
/// crashed (2026-09-11). `None` = flyable, or nothing to judge (no FAF, no
/// runway geometry) — the follower's own checks take over from there.
fn faf_off_course(
    legs: &[ProcLeg],
    airport: &crate::nav::AirportInfo,
    rwy_id: &str,
) -> Option<String> {
    let hi = handoff_index(legs)?;
    let faf = legs[hi].fix.as_ref()?;
    let (thr_lat, thr_lon) = crate::nav::runway_threshold(airport, rwy_id)?;
    let course = crate::nav::runway_course_deg(airport, rwy_id)?;
    let (xtk_m, along_m) = crate::nav::runway_frame_m(thr_lat, thr_lon, course, faf.lat, faf.lon);
    if crate::circuit::interceptable(xtk_m, along_m) {
        return None;
    }
    Some(format!(
        "The {} fix is {:.1} miles off the runway {} centreline with {:.1} to run. \
         Autoland can't line up from there, so that approach is out.",
        faf.ident,
        xtk_m.abs() / 1852.0,
        crate::circuit::spell_runway(rwy_id),
        along_m.max(0.0) / 1852.0
    ))
}

/// The indicated speed the throttle governor holds while the follower flies
/// this plan, if any. Enroute: the profile's cruise speed. An APPROACH: the
/// clean approach speed autoland itself flies outside the deceleration ramp,
/// so the handoff at the FAF is at a speed it expects. Before this, an approach
/// plan left the throttle wherever the enroute leg had it: cruise power all the
/// way down the straight-in, 195 kt at the FAF against a 100 kt approach speed,
/// and the gear and flaps went out 40 kt over their limits (2026-09-11).
pub fn power_hold_target(end: &EndAction, cruise_ias_kts: Option<f64>) -> Option<f64> {
    match end {
        EndAction::Enroute => cruise_ias_kts,
        EndAction::Approach { approach_ias, .. } => {
            Some(crate::autoland::clean_approach_ias(*approach_ias))
        }
        EndAction::Level => None,
    }
}

/// Build the ENROUTE legs of a SimBrief plan: direct-to legs over the named
/// fixes, with the plan's own crossing altitudes as constraints. Pure, so the
/// altitude logic battle-tests without a fetch.
///
/// - The origin airport (prepended by the fetch) and the destination airport
///   (the arrival phase owns that) are dropped — this is the middle of the
///   flight only.
/// - SimBrief's per-fix altitudes carry the whole profile: rising through the
///   climb, cruise in the middle, falling after TOD. Flying the CLIMB fixes at
///   their charted crossing altitudes would stair-step the departure (level at
///   3,000 over a fix mid-climb, then climb again), so every fix before the
///   first at-cruise fix commands cruise directly — one continuous climb — and
///   from there the plan's own numbers stand, which is what flies the descent.
/// - A fix with no plausible altitude (0 in the navlog) commands cruise.
pub fn enroute_legs(
    waypoints: &[crate::simbrief::Waypoint],
    origin: &str,
    destination: &str,
    cruise_alt_ft: f64,
) -> Vec<ProcLeg> {
    let mid: Vec<&crate::simbrief::Waypoint> = waypoints
        .iter()
        .enumerate()
        .filter(|(i, w)| {
            (*i != 0 || !w.ident.eq_ignore_ascii_case(origin))
                && (*i != waypoints.len() - 1 || !w.ident.eq_ignore_ascii_case(destination))
        })
        .map(|(_, w)| w)
        .collect();
    let plausible = |alt: f64| (500.0..=60_000.0).contains(&alt);
    let raw: Vec<f64> = mid
        .iter()
        .map(|w| {
            if plausible(w.alt) {
                w.alt
            } else {
                cruise_alt_ft
            }
        })
        .collect();
    let peak = raw.iter().copied().fold(cruise_alt_ft, f64::max);
    let first_at_peak = raw.iter().position(|a| *a >= peak - 100.0).unwrap_or(0);
    // A fix charted within a few hundred feet of cruise IS cruise: SimBrief
    // put MG834 at 3,900 between two 4,000 ft fixes and the dial dipped and
    // came back for nothing (2026-09-11). Real descents are thousands.
    const CRUISE_DIP_FT: f64 = 300.0;
    mid.iter()
        .zip(raw.iter())
        .enumerate()
        .map(|(i, (w, alt))| ProcLeg {
            seq: i as u32,
            transition: String::new(),
            leg_type: "TF".to_string(),
            fix_ident: w.ident.clone(),
            fix_region: String::new(),
            fix: Some(crate::nav::Fix {
                ident: w.ident.clone(),
                lat: w.lat,
                lon: w.lon,
            }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: Some(if i < first_at_peak || (peak - *alt).abs() <= CRUISE_DIP_FT {
                peak
            } else {
                *alt
            }),
            alt2_ft: None,
            is_faf: false,
        })
        .collect()
}

/// Whether the aircraft is positioned to actually FLY this approach — outside
/// the final approach fix and at/above its crossing altitude. A tight circuit at
/// pattern height INSIDE the FAF is not (arming there would command a climb back
/// to platform); those stay with autoland. An enroute arrival at altitude is.
/// The seam that lets the circuit's arrival prefer the real approach only when
/// it makes sense.
pub fn approach_flyable_from(plan: &Plan, lat: f64, lon: f64, alt_ft: f64) -> bool {
    let Some(hi) = handoff_index(&plan.legs) else {
        return false;
    };
    let leg = &plan.legs[hi];
    let Some(fix) = &leg.fix else {
        return false;
    };
    let dist = crate::nav::haversine_nm(lat, lon, fix.lat, fix.lon);
    let faf_alt = leg.alt1_ft.unwrap_or(0.0);
    dist >= 2.0 && alt_ft >= faf_alt - 500.0
}

/// Spawn the follower on its own thread + runtime + client, the circuit's shape.
pub fn start(base_url: String, voice: Option<Voice>, plan: Plan) {
    if ACTIVE.swap(true, Ordering::SeqCst) {
        return;
    }
    ABORT.store(false, Ordering::SeqCst);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            ACTIVE.store(false, Ordering::SeqCst);
            return;
        };
        rt.block_on(fly(
            XPlaneClient::new(base_url.clone()),
            base_url,
            voice,
            plan,
        ));
        ACTIVE.store(false, Ordering::SeqCst);
        publish_leg(None);
    });
}

fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    tracing::info!(target: "xvoicelayer::procedure", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

fn stand_down(voice: &Option<Voice>, line: &str, reason: &str) {
    ACTIVE.store(false, Ordering::SeqCst);
    clear_pilot_altitude();
    publish_terrain_floor(None);
    // Drop any cruise-power hold so nothing keeps working the throttle after we
    // hand back (no-op if it was never armed).
    crate::tools::speedhold::release();
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "stand_down", "reason": reason}),
    );
    alert(voice, Priority::Caution, line);
}

async fn fly(xp: XPlaneClient, base_url: String, voice: Option<Voice>, plan: Plan) {
    let Plan {
        name,
        legs,
        end,
        vy_kts,
        cruise_ias_kts,
    } = plan;
    let handoff = handoff_index(&legs);
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    // Process-wide facts must never outlive the flight that set them.
    clear_pilot_altitude();
    publish_terrain_floor(None);
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "armed", "name": name, "legs": legs.len(), "approach": matches!(end, EndAction::Approach{..})}),
    );
    alert(&voice, Priority::Advisory, &format!("Flying {name}."));

    // Engage the AP: servos + heading; vertical is VS-mode + dial (the
    // takeoff's proven recipe — `altitude_hold` holds the CURRENT altitude in
    // X-Plane, not the dial, and never flew a constraint; the circuit's
    // pattern flew flat at handoff height for the same reason, 2026-07-24).
    let _ = xp.run_command(CMD_SERVOS_ON).await;
    if xp.get_float(HDG_STATUS).await.unwrap_or(0.0) <= 0.5 {
        let _ = xp.run_command(CMD_HDG_MODE).await;
    }
    let mag_var = xp.get_float(MAG_VAR).await.unwrap_or(0.0);
    // The heading bug is SLEWED, never jumped: a single write across the
    // 0/360 seam reads as a long-way dial spin to the sim's AP (the circuit's
    // 270-degree wrong-way turn, 2026-07-24). And the slew must START AT THE
    // AIRCRAFT'S NOSE, never at the dial's read: seeding from a failed read
    // (unwrap_or(0.0)) walked the dial clockwise from zero to a target 22°
    // to our LEFT, the AP honored the spin direction, and the aircraft flew
    // a full 340° right orbit over Jerez before converging (16:59Z,
    // 2026-07-24). Snapping the dial to psi first is safe whatever the jump
    // (the AP is already pointing there, so the inferred direction moves it
    // nowhere), and every subsequent step is unambiguous.
    let psi_seed = xp.get_float(PSI_TRUE).await.unwrap_or(0.0);
    let mut bug_mag = true_to_mag(psi_seed, mag_var);
    let _ = xp.set_float(HDG_BUG, bug_mag).await;
    // Carry starts at the current altitude so an unconstrained first leg holds.
    let mut carry_alt = xp.get_float(ELEV_M).await.unwrap_or(0.0) * M_TO_FT;
    // What we COMMANDED last tick, a different thing from the leg constraint
    // carried across unconstrained fixes, and the pacing `descent_target_ft`
    // actually needs. They were one variable until 2026-09-13, and since
    // `carry_alt` is overwritten with the new leg's own constraint one line
    // after it is read, the top-of-descent test compared the constraint with
    // itself from the second tick of every leg onward: the leg to JRZ armed at
    // 55.9 nm and began its 5,300 ft descent immediately, into the Sierra. The
    // deferral worked for exactly one tick per leg, which is why the logs
    // always showed one honest 8,000 followed by the dive.
    let mut prev_cmd_alt = carry_alt;
    let mut last_vs_cmd: Option<f64> = None;
    // The altitude PRESELECT is driven to the leg target too: a stale dial
    // with capture armed (1,122 ft on the Jerez departure, from nobody's
    // loop) would otherwise level the climb wherever it happened to sit —
    // and driven to the real target, the capture works FOR us: it owns the
    // level-off at the constraint more cleanly than the VS chase.
    let mut last_alt_dial: Option<f64> = None;

    // Cruise power: on an enroute flight, arm the flight speed-hold at the
    // planned indicated cruise speed. The throttle law then FINDS the power —
    // full while slow (the climb still commands full throttle), settling to
    // cruise power once at speed, robust to weight/altitude/temp. Pitch (the VS
    // law below) owns altitude; the governor owns speed. Released at every exit
    // so autoland/the arrival own the throttle alone.
    if let Some(kts) = power_hold_target(&end, cruise_ias_kts) {
        crate::tools::speedhold::arm_flight(base_url.clone(), voice.clone(), kts);
        let event = if matches!(end, EndAction::Enroute) { "cruise_power" } else { "approach_power" };
        crate::blackbox::record(
            "procedure",
            serde_json::json!({"event": event, "target_ias": kts}),
        );
    }

    let mut idx = 0usize;
    // The FMS is sequenced once per leg, when the follower first steers at it.
    let mut fms_leg: Option<usize> = None;
    // Terrain look-ahead: probed on its own slow cadence (the ground does not
    // move), disabled for the flight the first time the plugin has no answer so
    // an old build degrades to the previous behaviour instead of stalling the
    // tick for two seconds every ten.
    let mut last_probe = std::time::Instant::now() - std::time::Duration::from_secs(60);
    let mut terrain_available = true;
    let mut said_floor: Option<f64> = None;
    let mut said_alt_hold = false;
    while idx < legs.len() {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if ABORT.load(Ordering::SeqCst) {
            stand_down(&voice, "Cancelled. Your aircraft.", "abort");
            return;
        }
        // A frozen sim (pause/replay) suspends the sequence; the AP holds.
        if crate::xplane::sim_frozen(&xp).await {
            continue;
        }

        let (lat, lon, elev_m, psi_r, hpath_r, agl_m, vs_st, hdg_st, ias_r, gs_r) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(ELEV_M),
            xp.get_float(PSI_TRUE),
            xp.get_float(HPATH),
            xp.get_float(AGL_M),
            xp.get_float(VS_STATUS),
            xp.get_float(HDG_STATUS),
            xp.get_float(IAS),
            xp.get_float(GS_MS),
        );
        let (Ok(lat), Ok(lon)) = (lat, lon) else {
            continue;
        };
        let alt_ft = elev_m.unwrap_or(0.0) * M_TO_FT;
        let agl_ft = agl_m.unwrap_or(0.0) * M_TO_FT;

        let leg = &legs[idx];

        // Skip a leg whose fix never resolved — flying at a phantom (0,0) fix
        // would drag the aircraft off the planet. Log it and move on.
        if leg.fix.is_none() && !leg.fix_ident.is_empty() {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "skip_unresolved", "idx": idx, "fix": leg.fix_ident}),
            );
            idx += 1;
            continue;
        }
        publish_leg(leg.fix.as_ref().map(|f| ActiveLeg {
            fix: leg.fix_ident.clone(),
            lat: f.lat,
            lon: f.lon,
        }));
        if fms_leg != Some(idx) {
            fms_leg = Some(idx);
            sequence_fms(&xp, idx, leg).await;
        }

        // --- Lateral: steer the great-circle track to the fix, wind-crabbed so
        // the AP holds the GROUND track (the circuit's downwind lesson). A
        // to-altitude leg holds its own magnetic course.
        let psi_v = psi_r.ok();
        let crab = match (psi_v, hpath_r.ok()) {
            (Some(psi), Some(hp)) => {
                (((psi - hp + 540.0).rem_euclid(360.0)) - 180.0).clamp(-25.0, 25.0)
            }
            _ => 0.0,
        };
        let dist_nm = bearing_to_fix(lat, lon, leg).map(|_| {
            let f = leg.fix.as_ref().unwrap();
            crate::nav::haversine_nm(lat, lon, f.lat, f.lon)
        });
        let target_mag = if let Some(brg_true) = bearing_to_fix(lat, lon, leg) {
            Some(true_to_mag(brg_true + crab, mag_var))
        } else {
            // course is already magnetic; add crab to hold it as a track.
            leg.course_mag.map(|c| (c + crab).rem_euclid(360.0))
        };
        if let Some(t) = target_mag {
            bug_mag = crate::circuit::slew_bug(bug_mag, t, 40.0);
            let _ = xp.set_float(HDG_BUG, bug_mag).await;
        }
        // The lateral mode must actually be ON for the bug to mean anything —
        // the circuit's guard, which this follower LACKED: an AP disconnect
        // mid-climb left the servos off with only the VS re-engage watching,
        // and the aircraft spiralled at 20°/s while the loop slewed a bug
        // nobody was following (LEJR enroute, 2026-07-24).
        if hdg_st.unwrap_or(1.0) <= 0.5 {
            let _ = xp.run_command(CMD_SERVOS_ON).await;
            let _ = xp.run_command(CMD_HDG_MODE).await;
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "reengage_lateral", "idx": idx}),
            );
        }

        // --- Vertical: fly toward the leg's constraint (carried across
        // unconstrained legs) with the VS law — proportional rate, clamped,
        // mode re-engaged when dropped (the takeoff's recipe) — and the
        // preselect dialled to the same target so the sim's armed capture
        // levels off AT the constraint instead of a stale value.
        // A DESCENT waits for its top-of-descent instead of starting the
        // moment the leg arms — the constraint is at the FIX, not here.
        let gs_kts = gs_r.map(|v| v * MS_TO_KTS).unwrap_or(0.0);

        // --- Terrain: probe the ground AHEAD along the track and never command
        // an altitude into it. Skipped on an approach, whose own procedure owns
        // the descent below the enroute floor (and whose platform altitudes
        // would otherwise be lifted out of the glideslope by this).
        let on_approach = matches!(end, EndAction::Approach { .. });
        if terrain_available
            && !on_approach
            && last_probe.elapsed().as_secs_f64() >= TERRAIN_PROBE_SECS
        {
            last_probe = std::time::Instant::now();
            // Look along the track we are STEERING, out to the fix at most: near
            // a field the answer should be about the field, not the hills past it.
            let brg = bearing_to_fix(lat, lon, leg)
                .or(psi_v)
                .unwrap_or_else(|| bug_mag + mag_var);
            let look = dist_nm
                .map(|d| (d + 2.0).min(TERRAIN_LOOK_NM))
                .unwrap_or(TERRAIN_LOOK_NM);
            let pts = look_ahead_points(lat, lon, brg, look, TERRAIN_POINTS);
            match crate::terrain::max_elevation_ft(&xp, &pts).await {
                Some(ground) => publish_terrain_floor(Some(ground + TERRAIN_CLEARANCE_FT)),
                None => {
                    terrain_available = false;
                    publish_terrain_floor(None);
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "terrain_probe_unavailable"}),
                    );
                }
            }
        }
        // --- The pilot's altitude outranks the plan's, the ground outranks
        // both. The autopilot tool hands a commanded altitude to the follower
        // rather than writing the dial, so the follower stays the one writer
        // and an override cannot be undone by the next tick.
        let v = vertical_tick(
            leg,
            carry_alt,
            prev_cmd_alt,
            alt_ft,
            dist_nm,
            gs_kts,
            pilot_altitude(),
            terrain_floor_ahead_ft(),
        );
        let want_alt = v.want_alt;
        carry_alt = v.carry;
        prev_cmd_alt = v.prev_cmd;
        match v.floored {
            Some(floor) => {
                if said_floor.is_none_or(|p| floor > p + TERRAIN_RESAY_FT) {
                    said_floor = Some(floor);
                    alert(
                        &voice,
                        Priority::Caution,
                        &format!(
                            "Terrain ahead, holding {} feet.",
                            crate::auto_status::commas((floor / 100.0).ceil() as i64 * 100)
                        ),
                    );
                }
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "terrain_floor", "idx": idx, "floor": floor,
                                       "alt_ft": alt_ft, "agl_ft": agl_ft, "dist_nm": dist_nm}),
                );
            }
            // Ground fell away: a later rise is news again.
            None => {
                if said_floor.is_some_and(|p| {
                    terrain_floor_ahead_ft().is_none_or(|f| f < p - TERRAIN_RESAY_FT)
                }) {
                    said_floor = None;
                }
            }
        }
        if last_alt_dial.is_none_or(|v| (v - want_alt).abs() > 50.0) {
            let _ = xp.set_float(ALT_DIAL, want_alt).await;
            last_alt_dial = Some(want_alt);
        }
        // Speed-protected: VS mode holds RATE whatever the airspeed, so the
        // climb rate backs off as IAS sags below Vy (the takeoff's own law,
        // floored positive) — the unprotected 700 fpm stalled the LEJR climb.
        let mut vs_cmd = ((want_alt - alt_ft) * 3.0).clamp(-700.0, 700.0);
        if vs_cmd > 0.0 && vy_kts > 0.0 {
            if let Ok(ias) = ias_r {
                vs_cmd = vs_cmd.min(crate::autotakeoff::climb_vs_fpm(ias, vy_kts, vs_cmd));
            }
        }
        if vs_st.unwrap_or(1.0) <= 0.5 {
            // VS mode gone NEAR the target means the preselect CAPTURED —
            // its hold is the clean level-off, leave it flying. Far from the
            // target it's a dropped mode: re-engage and rewrite the dial.
            if (want_alt - alt_ft).abs() > 100.0 {
                let _ = xp.run_command(CMD_VS_MODE).await;
                last_vs_cmd = None; // engage synced the dial; force a rewrite
            }
        }
        if last_vs_cmd.is_none_or(|v| (v - vs_cmd).abs() > 40.0) {
            let _ = xp.set_float(VS_DIAL, vs_cmd).await;
            last_vs_cmd = Some(vs_cmd);
        }
        let owner = if v.floored.is_some() {
            AltOwner::Terrain
        } else if v.pilot {
            AltOwner::Pilot
        } else {
            AltOwner::Plan
        };
        DETAIL.set(leg_detail(
            &leg.fix_ident,
            dist_nm,
            gs_kts,
            alt_ft,
            want_alt,
            owner,
        ));
        crate::auto_status::set_ap_targets(Some(bug_mag), Some(want_alt), Some(vs_cmd));

        crate::blackbox::record(
            "procedure",
            serde_json::json!({
                "event": "leg", "idx": idx, "type": leg.leg_type, "fix": leg.fix_ident,
                "dist_nm": dist_nm, "alt_ft": alt_ft, "want_alt": want_alt, "agl_ft": agl_ft, "crab": crab,
                "psi": psi_v, "bug": bug_mag, "tgt": target_mag,
            }),
        );

        // --- Approach handoff: at the FAF leg, once captured, autoland owns it.
        if let (
            EndAction::Approach {
                apt_dat,
                cfg,
                approach_ias,
                rwy_id,
            },
            Some(hi),
        ) = (&end, handoff)
        {
            if idx >= hi && leg_captured(leg, dist_nm, alt_ft) {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "handoff_autoland", "idx": idx, "agl_ft": agl_ft}),
                );
                alert(&voice, Priority::Advisory, "At the fix. Landing us.");
                // The governor lets go BEFORE autoland's throttle law starts,
                // so no tick has two writers on the lever.
                crate::tools::speedhold::release();
                crate::autoland::ABORT.store(false, Ordering::SeqCst);
                crate::autoland::run_autoland(
                    base_url.clone(),
                    apt_dat.clone(),
                    cfg.clone(),
                    *approach_ias,
                    voice.clone(),
                    None,
                    rwy_id.clone(),
                );
                clear_pilot_altitude();
                publish_terrain_floor(None);
                ACTIVE.store(false, Ordering::SeqCst);
                return;
            }
        }

        // A to-altitude leg is satisfied by ALTITUDE, so one cannot sequence
        // while the pilot owns that altitude. Say so once rather than orbit a
        // heading in silence waiting for a climb nobody is flying.
        if dist_nm.is_none() && pilot_altitude().is_some() && !said_alt_hold {
            said_alt_hold = true;
            if let Some(t) = leg.alt1_ft {
                alert(
                    &voice,
                    Priority::Caution,
                    &format!(
                        "This leg wants {} feet and you have the altitude. Say an altitude to \
                         carry on, or cancel.",
                        crate::auto_status::commas(t.round() as i64)
                    ),
                );
            }
        }

        if leg_captured(leg, dist_nm, alt_ft) {
            idx += 1;
            if idx < legs.len() {
                let next = &legs[idx];
                let label = if next.fix_ident.is_empty() {
                    "next leg".to_string()
                } else {
                    next.fix_ident.clone()
                };
                alert(&voice, Priority::Advisory, &format!("Next, {label}."));
            }
        }
    }

    // Ran off the end of the legs (a departure, or an approach with no handoff).
    // Drop any cruise-power hold before handing back (no-op if never armed).
    crate::tools::speedhold::release();
    match end {
        EndAction::Level => {
            crate::blackbox::record("procedure", serde_json::json!({"event": "complete_level"}));
            alert(
                &voice,
                Priority::Advisory,
                "Procedure complete. Level, the autopilot is holding. Your aircraft.",
            );
        }
        EndAction::Enroute => {
            // The flight orchestrator is polling ACTIVE; it announces the
            // arrival itself. The AP keeps holding heading + altitude.
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "complete_enroute"}),
            );
        }
        EndAction::Approach { .. } => {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "complete_no_handoff"}),
            );
            alert(
                &voice,
                Priority::Caution,
                "Ran out of approach legs without a handoff. Your aircraft.",
            );
        }
    }
    clear_pilot_altitude();
    publish_terrain_floor(None);
    ACTIVE.store(false, Ordering::SeqCst);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav::Fix;

    /// The crew's "next waypoint" answer while a procedure flies: distance to
    /// the follower's OWN fix, minutes from the groundspeed, no ETA when we are
    /// not really moving (a parked aircraft is not 0 minutes from anywhere).
    #[test]
    fn leg_progress_is_distance_and_minutes_to_the_followers_fix() {
        // MG430 from the point the LXGB-LEMG flight was actually at when the
        // GPS still said "PIMOS, 48 miles".
        let leg = ActiveLeg { fix: "MG430".into(), lat: 36.9985, lon: -4.9035 };
        let (dist, mins) = leg_progress(&leg, 36.95877, -4.90277, 100.0);
        assert!((dist - 2.39).abs() < 0.1, "dist {dist}");
        let mins = mins.expect("moving: an ETA");
        assert!((mins - 1.43).abs() < 0.1, "mins {mins}");
        assert_eq!(leg_progress(&leg, 36.95877, -4.90277, 3.0).1, None);
    }

    #[test]
    fn no_procedure_means_no_active_leg() {
        // Whatever a finished follower left in the slot, an inactive
        // procedure has no leg: the GPS answer applies again.
        publish_leg(Some(ActiveLeg { fix: "X".into(), lat: 0.0, lon: 0.0 }));
        if !ACTIVE.load(Ordering::Relaxed) {
            assert_eq!(active_leg(), None);
        }
        publish_leg(None);
    }

    fn leg(fix_ident: &str, lat: f64, lon: f64) -> ProcLeg {
        ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "TF".to_string(),
            fix_ident: fix_ident.to_string(),
            fix_region: "LE".to_string(),
            fix: Some(Fix {
                ident: fix_ident.to_string(),
                lat,
                lon,
            }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: None,
            alt2_ft: None,
            is_faf: false,
        }
    }

    /// The leg to JRZ as the LEMG-LEJR plan carried it: a fix 55.9 nm out with
    /// a 2,700 ft constraint, armed at 8,000 ft in the cruise.
    fn jrz() -> ProcLeg {
        let mut l = leg("JRZ", 36.7446, -6.0601);
        l.alt1_ft = Some(2700.0);
        l
    }

    /// The loop's own tick, at the 135 kt the sortie was flown at.
    fn tick(
        leg: &ProcLeg,
        carry: f64,
        prev: f64,
        alt: f64,
        dist_nm: f64,
        pilot: Option<f64>,
        floor: Option<f64>,
    ) -> Vertical {
        vertical_tick(leg, carry, prev, alt, Some(dist_nm), 135.0, pilot, floor)
    }

    /// THE bug of 2026-09-13: the descent must WAIT for its top of descent on
    /// every tick, not only the first of the leg. Ticking the real bookkeeping
    /// forward the way the loop does is the whole test: `descent_target_ft`
    /// alone always passed.
    #[test]
    fn the_descent_waits_for_its_top_of_descent_tick_after_tick() {
        let leg = jrz();
        let (mut carry, mut prev) = (8000.0, 8000.0);
        // Cruise, 55.9 nm to run: 5,300 ft at 700 fpm and 135 kt needs ~18 nm.
        for _ in 0..20 {
            let v = tick(&leg, carry, prev, 8000.0, 55.9, None, None);
            assert_eq!(v.want_alt, 8000.0, "held the cruise to the top of descent");
            carry = v.carry;
            prev = v.prev_cmd;
        }
        // Inside it, the constraint is commanded and stays commanded.
        let v = tick(&leg, carry, prev, 8000.0, 17.0, None, None);
        assert_eq!(v.want_alt, 2700.0);
        let v = tick(&leg, v.carry, v.prev_cmd, 7000.0, 15.0, None, None);
        assert_eq!(v.want_alt, 2700.0);
    }

    /// The ground outranks the plan. Grazalema probes at 3,861 ft under the
    /// leg that wanted 2,700: the follower flies the floor, and says so.
    #[test]
    fn terrain_ahead_floors_the_plans_constraint() {
        let floor = 3861.0 + TERRAIN_CLEARANCE_FT;
        let v = tick(&jrz(), 2700.0, 2700.0, 4000.0, 38.0, None, Some(floor));
        assert_eq!(v.want_alt, floor);
        assert_eq!(v.floored, Some(floor));
        // And the floor is what carries to the next tick, so the climb holds.
        assert_eq!(v.prev_cmd, floor);
        // Low ground under a high target changes nothing.
        let v = tick(&jrz(), 8000.0, 8000.0, 8000.0, 55.9, None, Some(1300.0));
        assert_eq!(v.want_alt, 8000.0);
        assert_eq!(v.floored, None);
    }

    /// A pilot's altitude outranks the plan's constraint: the override that
    /// was silently undone every 500 ms while the aircraft sank to 220 ft AGL.
    #[test]
    fn the_pilots_altitude_outranks_the_plan() {
        let v = tick(&jrz(), 2700.0, 2700.0, 2800.0, 38.0, Some(4000.0), None);
        assert_eq!(v.want_alt, 4000.0);
        assert!(v.pilot);
        // It survives the next tick, and the one after: the plan never takes
        // the altitude back on its own.
        let v = tick(&jrz(), v.carry, v.prev_cmd, 3200.0, 36.0, Some(4000.0), None);
        assert_eq!(v.want_alt, 4000.0);
        let v = tick(&jrz(), v.carry, v.prev_cmd, 4000.0, 34.0, Some(4000.0), None);
        assert_eq!(v.want_alt, 4000.0);
    }

    /// ...but the ground outranks the PILOT too. Four thousand feet was itself
    /// into the ridge; the follower holds the floor, not the number asked for.
    #[test]
    fn terrain_outranks_the_pilot_as_well() {
        let floor = 3861.0 + TERRAIN_CLEARANCE_FT;
        let v = tick(&jrz(), 2700.0, 2700.0, 2800.0, 38.0, Some(4000.0), Some(floor));
        assert_eq!(v.want_alt, floor);
        assert_eq!(v.floored, Some(floor));
        assert!(!v.pilot, "the floor set it, not the pilot");
    }

    /// The probe looks along the TRACK, not at the aircraft: the first point is
    /// ahead of us and the last is the full look-ahead out.
    #[test]
    fn look_ahead_points_walk_down_the_track() {
        let pts = look_ahead_points(36.6354, -5.2307, 285.0, 15.0, TERRAIN_POINTS);
        assert_eq!(pts.len(), TERRAIN_POINTS);
        let d = |(a, b): (f64, f64)| crate::nav::haversine_nm(36.6354, -5.2307, a, b);
        assert!(d(pts[0]) < 1.5, "first point is close ahead: {}", d(pts[0]));
        let last = d(*pts.last().unwrap());
        assert!((last - 15.0).abs() < 0.5, "last point is the look-ahead: {last}");
        // Heading 285 is west and slightly north of the aircraft.
        assert!(pts.last().unwrap().1 < -5.2307);
        assert!(pts.last().unwrap().0 > 36.6354);
    }

    fn to_alt(desc: char, alt: f64) -> ProcLeg {
        ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "CA".to_string(),
            fix_ident: String::new(),
            fix_region: String::new(),
            fix: None,
            course_mag: Some(315.0),
            dist_nm: None,
            turn: None,
            alt_desc: desc,
            alt1_ft: Some(alt),
            alt2_ft: None,
            is_faf: false,
        }
    }

    /// Gibraltar as apt.dat has it: 09 at the west end, 27 at the east.
    fn gibraltar() -> crate::nav::AirportInfo {
        use std::io::Write;
        let apt = std::env::temp_dir().join("xvl-proc-lxgb-apt.dat");
        let mut f = std::fs::File::create(&apt).unwrap();
        writeln!(f, "1 11 0 0 LXGB Gibraltar").unwrap();
        writeln!(f, "100 46 0 0 0 0 0 0 09 36.1509000 -5.3581000 0 0 0 0 0 0 27 36.1515000 -5.3412000 0 0 0 0 0 0").unwrap();
        f.flush().unwrap();
        crate::nav::find_airport(apt.to_str().unwrap(), "LXGB")
            .unwrap()
            .unwrap()
    }

    // The crash of 2026-09-11: 09D05 sits 3.6 km abeam the 09 centreline with
    // 5.5 km to run. Handing autoland the aircraft there is not an approach.
    #[test]
    fn a_faf_autoland_cannot_intercept_from_is_refused() {
        let airport = gibraltar();
        let mut faf = leg("09D05", 36.104666667, -5.405333333);
        faf.is_faf = true;
        let legs = vec![leg("09D06", 36.088, -5.404666667), faf, leg("RW09", 36.1509, -5.3581)];
        let why = faf_off_course(&legs, &airport, "09").expect("refused");
        assert!(why.contains("09D05") && why.contains("off the runway zero nine centreline"), "{why}");

        // A FAF ON the centreline 4 nm out is fine.
        let mut faf = leg("09D04", 36.1482, -5.4406);
        faf.is_faf = true;
        let legs = vec![faf, leg("RW09", 36.1509, -5.3581)];
        assert_eq!(faf_off_course(&legs, &airport, "09"), None);

        // Nothing to judge: no legs, or a runway the field doesn't have.
        assert_eq!(faf_off_course(&[], &airport, "09"), None);
        assert_eq!(faf_off_course(&legs, &airport, "18"), None);
    }

    // An approach plan holds the clean approach speed down the straight-in;
    // before this the throttle stayed at cruise power and the FAF handoff was
    // at 195 kt on a 100 kt approach (2026-09-11).
    #[test]
    fn an_approach_plan_holds_the_clean_approach_speed() {
        let approach = EndAction::Approach {
            apt_dat: String::new(),
            cfg: ApConfig::default(),
            approach_ias: 100.0,
            rwy_id: None,
        };
        let held = power_hold_target(&approach, None).expect("an approach holds a speed");
        assert!((140.0..=150.0).contains(&held), "1.45 x Vapp, got {held}");
        assert_eq!(power_hold_target(&EndAction::Enroute, Some(180.0)), Some(180.0));
        assert_eq!(power_hold_target(&EndAction::Enroute, None), None);
        assert_eq!(power_hold_target(&EndAction::Level, Some(180.0)), None);
    }

    #[test]
    fn steers_the_great_circle_bearing_to_the_fix() {
        // A fix due east: bearing ~90.
        let l = leg("EAST", 36.0, -4.0);
        let brg = bearing_to_fix(36.0, -5.0, &l).unwrap();
        assert!((brg - 90.0).abs() < 1.0, "{brg}");
        // A to-altitude leg has no fix to steer at.
        assert!(bearing_to_fix(36.0, -5.0, &to_alt('+', 3000.0)).is_none());
    }

    #[test]
    fn a_fix_leg_captures_inside_the_radius() {
        let l = leg("F", 36.0, -5.0);
        assert!(!leg_captured(&l, Some(1.5), 2000.0), "far: not captured");
        assert!(
            leg_captured(&l, Some(CAPTURE_NM - 0.1), 2000.0),
            "inside: captured"
        );
    }

    #[test]
    fn a_climb_leg_captures_at_or_above_the_altitude() {
        let climb = to_alt('+', 3000.0);
        assert!(!leg_captured(&climb, None, 2000.0), "below: keep climbing");
        assert!(leg_captured(&climb, None, 3000.0), "made it");
        assert!(leg_captured(&climb, None, 2900.0), "within margin counts");
    }

    #[test]
    fn a_descend_leg_captures_at_or_below_the_altitude() {
        let down = to_alt('-', 3000.0);
        assert!(!leg_captured(&down, None, 5000.0), "above: keep descending");
        assert!(leg_captured(&down, None, 3000.0), "made it");
    }

    #[test]
    fn altitude_carries_across_unconstrained_legs() {
        // A fix leg with no altitude keeps the last dialled value.
        let plain = leg("F", 36.0, -5.0);
        assert_eq!(commanded_alt_ft(&plain, 4500.0), 4500.0);
        // A constrained leg dials its own.
        let mut constrained = leg("G", 36.0, -5.0);
        constrained.alt1_ft = Some(2500.0);
        assert_eq!(commanded_alt_ft(&constrained, 4500.0), 2500.0);
    }

    #[test]
    fn handoff_is_the_faf_when_flagged() {
        let mut legs = vec![
            leg("IF", 36.0, -5.0),
            leg("FAF", 36.1, -5.0),
            leg("RW27", 36.2, -5.0),
        ];
        legs[1].is_faf = true;
        assert_eq!(handoff_index(&legs), Some(1));
    }

    #[test]
    fn handoff_falls_back_to_the_leg_before_the_runway() {
        // No FAF flag: hand off the leg before the RW threshold.
        let legs = vec![
            leg("IF", 36.0, -5.0),
            leg("STEP", 36.1, -5.0),
            leg("RW27", 36.2, -5.0),
        ];
        assert_eq!(handoff_index(&legs), Some(1));
        assert_eq!(handoff_index(&[]), None);
    }

    fn wp(ident: &str, lat: f64, lon: f64, alt: f64) -> crate::simbrief::Waypoint {
        crate::simbrief::Waypoint {
            ident: ident.to_string(),
            lat,
            lon,
            alt,
        }
    }

    #[test]
    fn leg_detail_reads_like_a_progress_line() {
        assert_eq!(
            leg_detail("PIMOS", Some(12.34), 120.0, 4820.0, 5497.0, AltOwner::Plan),
            "to PIMOS, 12.3 nm, 6 min, climbing to 5,500 ft"
        );
        assert_eq!(
            leg_detail("", None, 0.0, 5480.0, 5500.0, AltOwner::Plan),
            "level at 5,500 ft"
        );
        assert_eq!(
            leg_detail("BLN", Some(3.0), 10.0, 6000.0, 3000.0, AltOwner::Plan),
            "to BLN, 3.0 nm, descending to 3,000 ft"
        );
        // Not the plan's number: the panel says whose it is.
        assert_eq!(
            leg_detail("JRZ", Some(38.0), 135.0, 2800.0, 4900.0, AltOwner::Terrain),
            "to JRZ, 38.0 nm, 17 min, climbing to 4,900 ft, terrain floor"
        );
        assert_eq!(
            leg_detail("JRZ", Some(38.0), 135.0, 4000.0, 4000.0, AltOwner::Pilot),
            "to JRZ, 38.0 nm, 17 min, level at 4,000 ft, your altitude"
        );
    }

    #[test]
    fn fms_goto_payload_is_ident_lat_lon() {
        assert_eq!(
            fms_goto_payload(" pimos ", 36.4, -5.1234567),
            "PIMOS,36.400000,-5.123457"
        );
    }

    #[test]
    fn enroute_legs_drop_the_airports_and_keep_the_middle() {
        // The Jerez–Gibraltar shape: origin + two fixes + destination.
        let wps = vec![
            wp("LEJR", 36.74, -6.06, 0.0),
            wp("VJF", 36.24, -5.98, 5000.0),
            wp("LINTO", 35.83, -5.95, 5000.0),
            wp("LXGB", 36.15, -5.35, 2500.0),
        ];
        let legs = enroute_legs(&wps, "LEJR", "LXGB", 5000.0);
        assert_eq!(legs.len(), 2);
        assert_eq!(legs[0].fix_ident, "VJF");
        assert_eq!(legs[1].fix_ident, "LINTO");
        assert!(legs.iter().all(|l| l.fix.is_some() && l.leg_type == "TF"));
        assert_eq!(legs[0].alt1_ft, Some(5000.0));
    }

    #[test]
    fn climb_fixes_command_cruise_but_descent_fixes_stand() {
        // A mid-climb fix at 3,000 must not stair-step the departure; the
        // post-TOD 4,000 must be honored (that's the descent).
        let wps = vec![
            wp("AAA", 36.0, -5.0, 3000.0),
            wp("BBB", 36.2, -5.0, 5000.0),
            wp("CCC", 36.4, -5.0, 5000.0),
            wp("DDD", 36.6, -5.0, 4000.0),
        ];
        let legs = enroute_legs(&wps, "LEJR", "LXGB", 5000.0);
        let alts: Vec<f64> = legs.iter().map(|l| l.alt1_ft.unwrap()).collect();
        assert_eq!(alts, vec![5000.0, 5000.0, 5000.0, 4000.0]);
    }

    #[test]
    fn a_fix_a_hair_under_cruise_is_still_cruise() {
        // LEMG-LXGB, 2026-09-11: MG834 charted 3,900 between 4,000 ft fixes.
        let wps = vec![
            wp("MG833", 36.6, -4.4, 4000.0),
            wp("MG834", 36.5, -4.5, 3900.0),
            wp("PIMOS", 36.2, -4.9, 4000.0),
            wp("XXX", 36.1, -5.1, 2500.0),
        ];
        let legs = enroute_legs(&wps, "LEMG", "LXGB", 4000.0);
        let alts: Vec<f64> = legs.iter().map(|l| l.alt1_ft.unwrap()).collect();
        assert_eq!(alts, vec![4000.0, 4000.0, 4000.0, 2500.0]);
    }

    // The Gibraltar arrival: 3,900 ft, first approach fix wanting 1,397 with
    // 25 nm to run at 200 kt over the ground. Level until the top of descent.
    #[test]
    fn a_descent_waits_for_its_top_of_descent() {
        assert_eq!(descent_target_ft(1397.0, 3900.0, 3888.0, Some(25.0), 200.0), 3900.0);
        // TOD: 2,491 ft at 700 fpm is 3.6 min, 11.9 nm at 200 kt, plus the lead.
        assert_eq!(descent_target_ft(1397.0, 3900.0, 3888.0, Some(13.5), 200.0), 3900.0);
        assert_eq!(descent_target_ft(1397.0, 3900.0, 3888.0, Some(12.0), 200.0), 1397.0);
        // Climbs, small steps and fix-less legs go straight to the constraint.
        assert_eq!(descent_target_ft(4000.0, 2000.0, 2000.0, Some(30.0), 200.0), 4000.0);
        assert_eq!(descent_target_ft(3900.0, 4000.0, 3990.0, Some(30.0), 200.0), 3900.0);
        assert_eq!(descent_target_ft(1000.0, 4000.0, 4000.0, None, 200.0), 1000.0);
        // Never reached the previous target: hold where we are, rounded.
        assert_eq!(descent_target_ft(1000.0, 6000.0, 3460.0, Some(40.0), 200.0), 3500.0);
    }

    #[test]
    fn a_zero_altitude_fix_commands_cruise() {
        let wps = vec![wp("AAA", 36.0, -5.0, 0.0), wp("BBB", 36.2, -5.0, 0.0)];
        let legs = enroute_legs(&wps, "LEJR", "LXGB", 4500.0);
        assert!(legs.iter().all(|l| l.alt1_ft == Some(4500.0)));
    }

    #[test]
    fn a_plan_of_just_airports_yields_no_enroute_legs() {
        let wps = vec![
            wp("LEJR", 36.74, -6.06, 0.0),
            wp("LXGB", 36.15, -5.35, 2500.0),
        ];
        assert!(enroute_legs(&wps, "LEJR", "LXGB", 5000.0).is_empty());
    }

    #[test]
    fn a_tight_circuit_is_not_flyable_but_an_arrival_is() {
        // FAF at (36.0,-5.0), crossing 2500 ft.
        let mut faf = leg("FAF", 36.0, -5.0);
        faf.is_faf = true;
        faf.alt1_ft = Some(2500.0);
        let plan = Plan {
            name: "test".into(),
            legs: vec![faf],
            end: EndAction::Level,
            vy_kts: 0.0,
            cruise_ias_kts: None,
        };
        // Tight circuit: 0.3 nm from the FAF at 700 ft — NOT flyable (autoland).
        let (near_lat, near_lon) = crate::nav::project_latlon(36.0, -5.0, 0.0, 0.3);
        assert!(!approach_flyable_from(&plan, near_lat, near_lon, 700.0));
        // Arrival: 6 nm out at 3000 ft — flyable.
        let (far_lat, far_lon) = crate::nav::project_latlon(36.0, -5.0, 90.0, 6.0);
        assert!(approach_flyable_from(&plan, far_lat, far_lon, 3000.0));
        // Far out but too low (below platform − 500): not flyable.
        assert!(!approach_flyable_from(&plan, far_lat, far_lon, 1500.0));
    }
}
