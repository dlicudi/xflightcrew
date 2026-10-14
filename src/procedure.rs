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
//!    A CF leg is flown ON its course, and an approach IF behind the aircraft
//!    is joined by a pattern ([`crate::course_leg`]). To-altitude legs (CA/VA/FA — a missed approach's "climb to 3000", a SID's
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

use crate::autoflight::{self, Button, Dial};
use std::sync::atomic::{AtomicBool, Ordering};

use crate::control::measurements;
#[cfg(test)]
use crate::control::set_confirmed;
use crate::runway::{mag_to_true, true_to_mag};
use crate::heading_bug::HeadingHold;
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
/// The altimeter the pilot, the AP's preselect, the plan and ATC all read.
/// Cruise altitudes are pressure altitudes: on a hot day true altitude runs
/// hundreds of feet above indicated, and a follower targeting true feet parks
/// the aircraft below the dial with ALTS armed forever (LINTO, 2026-09-14:
/// 7,012 true, 6,838 indicated, VS +4 fpm).
const ALT_IND_FT: &str = crate::datarefs::ALTITUDE_FT;
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
const MS_TO_KTS: f64 = 1.943_84;
// AP surface (mirror of autopilot.rs / autoland.rs / circuit.rs).
#[cfg(test)]
const VS_DIAL: &str = Dial::Vs.read_name();
const GEAR_HANDLE: &str = "sim/cockpit2/controls/gear_handle_down";
const FLAP_RATIO: &str = "sim/cockpit2/controls/flap_ratio";
const VS_STATUS: &str = autoflight::names::VS_STATUS;
/// 0 off, 1 armed, 2 holding (the capture included).
const ALT_HOLD_STATUS: &str = autoflight::names::ALT_HOLD_STATUS;
#[cfg(test)]
const ALT_DIAL: &str = Dial::Altitude.read_name();
const HDG_STATUS: &str = autoflight::names::HDG_STATUS;

const M_TO_FT: f64 = 3.280_84;

/// The FLOOR on a leg's capture radius (a flyby simplification — we don't fly
/// RF arcs or exact fly-over turns yet). 0.8 nm keeps the corridor tight
/// without a light aircraft overshooting a sharp turn, and it is still the
/// whole answer for a GA airframe, whose turn anticipation never reaches it.
///
/// It was the ONLY answer until 2026-09-20, and a jet cannot fly to it: a fix
/// inside the aircraft's own turn circle is unreachable, so the follower
/// turned, missed, and orbited. EIDW-BIKF in the 737 — departure fixes 1.1,
/// 2.9 and 3.9 nm apart against a 2.5 nm turn radius at 250 kt — banked 25°
/// for minutes on end while the distance to the fix GREW (4.5, 6.7, 7.6 nm),
/// stalled three times and stood the follower down twice, once under full
/// auto and again after the pilot resumed. See [`capture_radius_nm`].
const CAPTURE_NM: f64 = 0.8;
/// The bank an enroute turn is flown at. Measured at 25° on the 737's climb
/// turns; assuming SHALLOWER is the safe direction, because a shallower bank
/// means a bigger radius, which starts the turn earlier — and erring early
/// cuts a corner, while erring late orbits the fix forever.
const ENROUTE_BANK_DEG: f64 = 20.0;
/// A to-altitude leg (CA/VA/FA) is satisfied within this of its target.
const ALT_MARGIN_FT: f64 = 150.0;

/// True while the procedure follower runs.
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::new();
pub fn running() -> bool {
    RUN.running()
}
pub fn outcome() -> Option<sim::run::Outcome> {
    RUN.outcome()
}

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

static FIX_DIST: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Miles to the fix the follower is steering at, republished every tick from
/// the number the tick already worked out. The mission's arrival watch reads
/// THIS rather than measuring the sim itself: an orchestrator that flies its
/// own reads to judge a loop it can already hear from is Web API load for
/// nothing, and it would measure a different distance from the one the
/// follower is actually closing.
pub fn fix_distance_nm() -> Option<f64> {
    if !ACTIVE.load(Ordering::Relaxed) {
        return None;
    }
    *FIX_DIST.lock().unwrap_or_else(|e| e.into_inner())
}

fn publish_fix_dist(d: Option<f64>) {
    *FIX_DIST.lock().unwrap_or_else(|e| e.into_inner()) = d;
}

/// The follower's OWN top of descent: where its vertical law will start down
/// for the leg it is flying. The monitor's descent advisor and the spoken
/// "how far to top of descent" used to measure a different point (SimBrief's
/// ring, or 3.5 nm per 1,000 ft to the FIELD) while the follower paced its
/// 700 fpm descent to the next FIX: on the LEJR-LXGB straight-in the copilot
/// said "past the descent point, 1,100 feet a minute makes the profile" and
/// "we're at top of descent, time to start down" three minutes before its own
/// hands moved (2026-09-14). One rule for what the mouth says and the hands
/// do: while a procedure flies, everything that names the top of descent
/// reads THIS.
#[derive(Clone, Debug, PartialEq)]
pub struct PlannedTod {
    /// The fix the descent is paced against.
    pub fix: String,
    /// Miles to the fix.
    pub dist_nm: f64,
    /// Miles from here to the descent point; negative once past it.
    pub to_tod_nm: f64,
    /// The altitude the follower will fly down to at the fix, indicated feet.
    pub target_ft: f64,
    /// True once the follower has commanded the descent.
    pub started: bool,
    /// The PILOT's altitude is holding the follower above this descent
    /// (`pilot_altitude::take`, sticky by design): the point is the plan's, and the
    /// hands will not move at it. LIME-LIPE (2026-09-25): "climb to 19,000"
    /// held the follower level over four of the plan's descents while the
    /// monitor said "descent coming up" and "past the descent point and
    /// still level" at each of them, a Caution about a level the pilot chose.
    pub held_ft: Option<f64>,
}

/// The follower's legs as flown and the index of the leg flying, republished
/// when either changes: what an in-flight re-pick (#79) splices into.
static ROUTE_AHEAD: std::sync::Mutex<Option<(usize, Vec<ProcLeg>)>> = std::sync::Mutex::new(None);

/// The legs the follower flies and the leg flying; `None` between procedures.
pub fn route_ahead() -> Option<(usize, Vec<ProcLeg>)> {
    if !ACTIVE.load(Ordering::Relaxed) {
        return None;
    }
    ROUTE_AHEAD.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

fn publish_route_ahead(v: Option<(usize, Vec<ProcLeg>)>) {
    *ROUTE_AHEAD.lock().unwrap_or_else(|e| e.into_inner()) = v;
}

/// A new tail for the flying route: the arrival re-picked in flight (#79,
/// `full_auto::repick_arrival`). `legs` replace the route from `from` on,
/// where the route must still hold `entry` (the STAR's first fix, the one
/// the splice was computed against); `constraints` are the new STAR's
/// windows, replacing the old ones whole.
#[derive(Debug, Clone)]
pub struct Splice {
    pub from: usize,
    pub entry: String,
    pub legs: Vec<ProcLeg>,
    pub constraints: Vec<crate::nav::StarConstraint>,
    pub why: String,
}

static SPLICE: std::sync::Mutex<Option<Splice>> = std::sync::Mutex::new(None);

/// Offer the flying follower a new tail; it takes it on its next tick, or
/// refuses it (`splice_stale`) when the leg flying is already past the entry.
/// False with no follower flying.
pub fn offer_splice(s: Splice) -> bool {
    if !ACTIVE.load(Ordering::SeqCst) {
        return false;
    }
    *SPLICE.lock().unwrap_or_else(|e| e.into_inner()) = Some(s);
    true
}

fn take_splice() -> Option<Splice> {
    SPLICE.lock().unwrap_or_else(|e| e.into_inner()).take()
}

/// The route with the splice applied, flying leg `idx`: the legs before
/// `from` kept and the new ones after, so `idx` still names the same leg.
/// Refused when the route no longer holds the entry at `from` (another
/// splice came first) or the leg flying is past it (a turn onto the old
/// STAR's next fix is already in hand: the arrival sets up from there). Pure.
pub fn apply_splice(legs: &[ProcLeg], idx: usize, s: &Splice) -> Result<Vec<ProcLeg>, &'static str> {
    let at = legs.get(s.from).ok_or("route_changed")?;
    if !at.fix_ident.eq_ignore_ascii_case(&s.entry) {
        return Err("route_changed");
    }
    if idx > s.from {
        return Err("past_entry");
    }
    if s.legs.is_empty() || !s.legs[0].fix_ident.eq_ignore_ascii_case(&s.entry) {
        return Err("no_entry");
    }
    let mut out = legs[..s.from].to_vec();
    out.extend(s.legs.iter().cloned());
    Ok(out)
}

static TOD: std::sync::Mutex<Option<PlannedTod>> = std::sync::Mutex::new(None);

/// The follower's planned top of descent for the leg being flown; `None`
/// between procedures, on a leg with no fix, or when the leg's constraint
/// is not below us (nothing to descend to yet).
pub fn planned_tod() -> Option<PlannedTod> {
    if !ACTIVE.load(Ordering::Relaxed) {
        return None;
    }
    TOD.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

fn publish_tod(tod: Option<PlannedTod>) {
    *TOD.lock().unwrap_or_else(|e| e.into_inner()) = tod;
}

/// The point to publish when the pilot's altitude (`held`) may be holding
/// the follower above the plan's descent: the FIRST such descent is published
/// marked held (the monitor reminds the pilot once, never a "past the descent
/// point" Caution); later ones under the same held altitude publish nothing.
/// Pure.
pub fn held_tod(
    tod: Option<PlannedTod>,
    held: Option<f64>,
    held_leg: &mut Option<(f64, String)>,
) -> Option<PlannedTod> {
    let mut t = tod?;
    let Some(h) = held else {
        return Some(t);
    };
    if let Some((a, f)) = held_leg.as_ref() {
        if (*a - h).abs() < 1.0 && *f != t.fix {
            return None;
        }
    }
    if t.to_tod_nm <= 12.0 && held_leg.as_ref().is_none_or(|(a, _)| (*a - h).abs() >= 1.0) {
        *held_leg = Some((h, t.fix.clone()));
    }
    t.held_ft = Some(h);
    Some(t)
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
    format!(
        "{},{:.6},{:.6}",
        crate::simbrief::fms_ident(ident),
        lat,
        lon
    )
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

/// LNAV out through the panel's owner, the bug to the heading flown (no
/// lurch); the follower's next steer sets its own. The heading written and
/// whether HDG SEL read back in (#227).
async fn lnav_hand_back(xp: &XPlaneClient, psi_true: f64, mag_var: f64) -> (f64, Option<bool>) {
    let hdg = true_to_mag(psi_true, mag_var);
    let taken = autoflight::lnav_out(xp, hdg, "procedure_lnav").await;
    (hdg, taken)
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
    RUN.cancel();
    ABORT.store(true, Ordering::SeqCst);
    // The stashed missed approach was this procedure's landing's.
    take_missed_legs();
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
        /// The approach's runway END and the AIRPORT it belongs to, handed to
        /// autoland as its forced runway. `None` lets autoland self-pick by
        /// track, and at a field with parallels that put the ILS 12 arrival
        /// on 13 against a clearance for 12 (LEMG, 2026-09-12). The airport
        /// travels with it because an ident alone resolves by proximity, and
        /// on the EGPH arrival that found EGPJ (2026-09-20).
        rwy_id: Option<crate::runway::PlannedRunway>,
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
    /// The airframe's transition altitude, for setting the altimeter at arm
    /// time: the follower flies INDICATED altitude, so a mis-set altimeter is a
    /// mis-flown altitude. `None` (approach plans) leaves the knob alone; the
    /// monitor's cross-check keeps it right from there.
    pub transition_altitude_ft: Option<f64>,
    /// The departure procedure's minimum altitudes, by fix (`nav::sid_minimums`).
    /// While one ahead is unmet the climb is flown at Vy and the best rate, every
    /// crossing is predicted, a short one is said, and one that will not be made
    /// hands the aircraft back with the numbers (`watch_minimum`).
    pub climb_minimums: Vec<crate::nav::ClimbMinimum>,
    /// The planned STAR's crossing restrictions on this route
    /// (`nav::star_constraints`), already clamped into the legs' altitudes by
    /// [`apply_arrival_constraints`]. The loop keeps every altitude one ahead
    /// reachable at the descent rate ([`arrival_ceiling_ft`]) and slows for
    /// every speed limit in time ([`arrival_speed_cap`]).
    pub arrival_constraints: Vec<crate::nav::StarConstraint>,
    /// An approach to a missed approach point short of the runway (LSZS): the
    /// angle of its visual segment, handed to the landing at the MAP.
    pub visual_deg: Option<f64>,
    /// ...and its published missed approach, straight legs only, flown by
    /// the follower if that landing goes around (never a self-flown pattern
    /// in a valley).
    pub missed_legs: Vec<ProcLeg>,
    /// The OFP's cruise schedule (#101): on an enroute flight a CRUISE leg is
    /// flown at the speed its cost index planned ([`scheduled_cruise_kts`],
    /// [`cruise_pick`]) and a planned step is said as it begins
    /// ([`step_climb_at`]; the levels themselves are already in `legs`,
    /// [`apply_step_climbs`]). Empty: the profile's `cruise_ias_kts`.
    pub cruise_schedule: crate::simbrief::CruiseSchedule,
    /// The speed an enroute plan hands its arrival over at (#199: full
    /// auto's, autoland's clean approach speed), slowed for by the end of
    /// the route ([`arrival_handoff_cap`]); `None` on every other plan.
    pub arrival_handoff_kts: Option<f64>,
}

/// The published missed approach of the approach being landed, stashed at the
/// MAP handoff for the landing's go-around (`take_missed_legs`). This
/// flight's only (`Scoped`): a stash the abort missed never flies another
/// flight's go-around.
static MISSED_LEGS: crate::scoped::Scoped<Vec<ProcLeg>> = crate::scoped::Scoped::flight();

/// Take the stashed missed approach (autoland, at a go-around).
pub fn take_missed_legs() -> Option<Vec<ProcLeg>> {
    MISSED_LEGS.take()
}

/// Stash for the flight the follower was launched in (`flight`): a tick of a
/// stood-down run, past its check when a door began another flight, stashes
/// nothing for the new one.
fn stash_missed_legs(flight: u64, legs: Vec<ProcLeg>) {
    if legs.is_empty() {
        if flight == MISSED_LEGS.now() {
            MISSED_LEGS.clear();
        }
    } else {
        MISSED_LEGS.set_in(flight, legs);
    }
}

// The follower's laws and its loop in their own files, whole and unchanged
// (refactor phase 4); every path stays `crate::procedure::...` through these
// re-exports.
mod constraints;
pub use constraints::*;
mod vertical;
pub use vertical::*;
mod schedule;
pub use schedule::*;
mod run;
pub use run::*;

// The Mach rule lives beside the airframe limits it also serves
// (`aircraft::VmoHigh`'s Mmo).
pub use crate::aircraft::cas_for_mach;

#[cfg(test)]
mod tests;

/// #153: the follower's departure owners on the EGKK LAM5P climb
/// (2026-09-28, sortie 1790601822-full_auto): D156J's 6,000 minimum, the
/// plan's navlog 5,000 at ACORN and D156O, the IFR clearance's 5,000.
#[cfg(test)]
mod departure_minimum_tests;

/// #199: the arrival hands autoland its clean approach speed; an
/// approach crosses its FAF at the speed its final absorbs (EGKK 26L,
/// 2026-10-14).
#[cfg(test)]
mod arrival_handoff_tests;
