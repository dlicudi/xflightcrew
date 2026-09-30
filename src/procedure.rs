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

use std::sync::atomic::{AtomicBool, Ordering};

use crate::autoland::true_to_mag;
use crate::control::{measurements, set_confirmed};
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
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
const GEAR_HANDLE: &str = "sim/cockpit2/controls/gear_handle_down";
const FLAP_RATIO: &str = "sim/cockpit2/controls/flap_ratio";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
/// 0 off, 1 armed, 2 holding (the capture included).
const ALT_HOLD_STATUS: &str = "sim/cockpit2/autopilot/altitude_hold_status";
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
const HDG_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
const HDG_STATUS: &str = "sim/cockpit2/autopilot/heading_status";

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
static RUN: sim::run::Slot = sim::run::Slot::new();
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
    /// (`take_altitude`, sticky by design): the point is the plan's, and the
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
        ident.trim().to_ascii_uppercase(),
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
        rwy_id: Option<crate::autoland::PlannedRunway>,
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
}

/// The published missed approach of the approach being landed, stashed at the
/// MAP handoff for the landing's go-around (`take_missed_legs`).
static MISSED_LEGS: std::sync::Mutex<Option<Vec<ProcLeg>>> = std::sync::Mutex::new(None);

/// Take the stashed missed approach (autoland, at a go-around).
pub fn take_missed_legs() -> Option<Vec<ProcLeg>> {
    MISSED_LEGS.lock().unwrap_or_else(|e| e.into_inner()).take()
}

fn stash_missed_legs(legs: Vec<ProcLeg>) {
    *MISSED_LEGS.lock().unwrap_or_else(|e| e.into_inner()) = (!legs.is_empty()).then_some(legs);
}

/// Autoland flies the visual segment from a MAP. Off until it is proven: at
/// LSZS (2026-09-24) the steep path in 18 kt of mountain wind porpoised
/// +/-1,500 fpm, levelled at 238 ft for no reason yet found, and climbed away
/// over the runway. The MAP is where a crew goes visual; the pilot takes it
/// there, talked down (`map_handover_path`).
pub const MAP_AUTOLAND: bool = false;

/// The talk-down's path for a hand-over at a MAP: from where the aircraft is
/// to 50 ft over the threshold, so the first call is "on glidepath" and every
/// later one measures the segment actually being flown. Slope clamped to
/// 3..6 degrees. Pure.
pub fn map_handover_path(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    alt_ft: f64,
) -> crate::approach_guide::ApproachPath {
    let dist_nm = crate::nav::haversine_nm(lat, lon, rwy.thr_lat, rwy.thr_lon).max(0.2);
    let height = alt_ft - (rwy.elev_ft + crate::approach_guide::THRESHOLD_CROSSING_FT);
    let (lo, hi) = (6076.12 * 3f64.to_radians().tan(), 6076.12 * 6f64.to_radians().tan());
    crate::approach_guide::ApproachPath {
        icao: rwy.icao.clone(),
        runway_id: rwy.id.clone(),
        thr_lat: rwy.thr_lat,
        thr_lon: rwy.thr_lon,
        course_deg: rwy.course_deg,
        elev_ft: rwy.elev_ft,
        ft_per_nm: (height / dist_nm).clamp(lo, hi),
    }
}

/// A missed approach point SHORT of the runway: a visual segment follows
/// (LSZS RNAV 03). A MAP coded on the runway fix itself is the landing, and
/// autoland's: LIPE's ILS 12 codes its MAP on RW12, the aircraft (high) never
/// captured the FAF, the follower reached RW12 0.8 nm out and handed the
/// pilot a "visual segment" onto an ILS runway (2026-09-25).
pub fn map_short_of_runway(leg: &ProcLeg) -> bool {
    leg.is_map && !leg.fix_ident.to_ascii_uppercase().starts_with("RW")
}

/// On the leg to a missed approach point, the published path: the altitude it
/// wants at `dist_nm` from the MAP and the descent rate that stays on it at
/// this groundspeed (fpm, negative). A P law cannot track a ramp; the rate is
/// fed forward. `None` off such a leg. Pure.
pub fn map_path(leg: &ProcLeg, dist_nm: Option<f64>, gs_kts: f64) -> Option<(f64, f64)> {
    if !leg.is_map {
        return None;
    }
    let t = leg.vpa_deg?.to_radians().tan();
    let map_ft = leg.alt1_ft?;
    let d = dist_nm?;
    Some((map_ft + d * 6076.12 * t, -gs_kts.max(0.0) * 6076.12 / 60.0 * t))
}

/// What goes on the ALTITUDE DIAL: the level this climb or descent ends at,
/// set once, the way a pilot sets the preselect and flies the rate. The
/// follower used to write its paced target (`want_alt`) every tick: on a
/// continuous descent that is a dial walking just under the aircraft, the
/// autopilot captured it, levelled, the dial stepped 50 ft lower, and the
/// descent went down a staircase 60 to 120 ft above its own path (LIPR 30,
/// 2026-09-26, handed to autoland 120 ft high at 0.8 nm). Descending, the
/// dial is the leg's constraint (`carry`), never under a terrain floor or a
/// pilot's level; climbing, the higher of the target and the constraint;
/// level, the level held; on a published path, the bottom of it. Whole
/// hundreds, rounded UP so a floor or a minimum is never under-set. Pure.
pub fn dial_target_ft(
    want_ft: f64,
    alt_ft: f64,
    carry_ft: f64,
    held_ft: Option<f64>,
    path_bottom_ft: Option<f64>,
) -> f64 {
    const BAND_FT: f64 = 100.0;
    let up = |v: f64| (v / 100.0).ceil() * 100.0;
    if let Some(b) = path_bottom_ft {
        return up(b.max(held_ft.unwrap_or(f64::MIN)));
    }
    if want_ft < alt_ft - BAND_FT {
        up(carry_ft.max(held_ft.unwrap_or(f64::MIN)))
    } else if want_ft > alt_ft + BAND_FT {
        up(want_ft.max(carry_ft))
    } else {
        (want_ft / 100.0).round() * 100.0
    }
}

/// Split a departure's minimums into the ones this plan can fly and the ones
/// above its cruise. A minimum over the planned cruise is a level the
/// follower was never going to climb to (the plan owns the target), and
/// judging the aircraft against it handed a 172 back to its pilot at 5,000
/// ft, three miles from D121Z, for not climbing to ANC8A's 9,500
/// (LIPR-LIBP 2026-09-26). Pure.
pub fn minimums_within_cruise(
    mins: Vec<crate::nav::ClimbMinimum>,
    cruise_ft: f64,
) -> (Vec<crate::nav::ClimbMinimum>, Vec<crate::nav::ClimbMinimum>) {
    mins.into_iter().partition(|m| m.min_ft <= cruise_ft + MINIMUM_MET_TOL_FT)
}

/// Within this of the clean floor, the approach's flaps come out on speed.
const FLAPS_ON_SPEED_MARGIN_KTS: f64 = 15.0;

/// A commanded rate as the VS wheel sets it: the nearest 100 fpm. Pure.
pub fn vs_dial_fpm(vs: f64) -> f64 {
    (vs / 100.0).round() * 100.0
}

/// A departure minimum is met this close under it, feet.
pub const MINIMUM_MET_TOL_FT: f64 = 100.0;

/// The first departure minimum still ahead of the aircraft and not yet met:
/// (leg index of its fix, minimum ft). Legs before `idx` are behind us.
/// Stateless, so it flips on the tolerance's last foot: the loop uses
/// [`DepartureMinimums`], which remembers; this is the old rule the #153
/// replay is judged against.
#[cfg(test)]
pub fn unmet_minimum(
    minimums: &[crate::nav::ClimbMinimum],
    legs: &[ProcLeg],
    idx: usize,
    alt_ft: f64,
) -> Option<(usize, f64)> {
    legs.iter().enumerate().skip(idx).find_map(|(i, leg)| {
        minimums
            .iter()
            .find(|m| m.fix.eq_ignore_ascii_case(&leg.fix_ident))
            // Met within the autopilot's own hold: levelled AT the minimum
            // is meeting it. Compared exactly, a 5,000 ft minimum at a 5,000
            // ft cruise flipped met/unmet on the altimeter's last foot and
            // the target with it, 74 to 108 knots and back (LIPE-LIPK,
            // 2026-09-25).
            .filter(|m| alt_ft < m.min_ft - MINIMUM_MET_TOL_FT)
            .map(|m| (i, m.min_ft))
    })
}

/// The departure minimums with MEMORY: a minimum once reached stays met, and
/// every minimum at a fix still ahead is a FLOOR on the target, met or not,
/// until its fix is behind.
///
/// EGKK LAM5P (2026-09-28, sortie 1790601822-full_auto, #153): D156J's
/// "at or above 6,000" was a level to climb TO only while unmet, and met
/// within [`MINIMUM_MET_TOL_FT`], so at 5,900 ft the target fell back to the
/// plan's navlog 5,000 (ACORN, D156O) and the speed from Vy's 210 to the
/// cruise's 250; the autopilot dived for the 5,000 dial, the minimum came
/// unmet under 5,900 and the climb and the 210 came back. Nineteen flips in
/// 207 s between 5,742 and 5,959 ft, at -1,822 to +1,145 fpm,
/// the yoke visibly jerking. A floor does not flip: met or not, the level
/// is held until the fix is passed, and "met" (Vy released) happens once.
#[derive(Debug, Default, Clone)]
pub struct DepartureMinimums {
    met: std::collections::HashSet<String>,
}

impl DepartureMinimums {
    /// The first minimum ahead not yet met, after marking met every one
    /// ahead whose level has been reached (`alt_ft` within the tolerance).
    /// Keyed by fix, so a splice of the arrival never forgets one.
    pub fn unmet(
        &mut self,
        minimums: &[crate::nav::ClimbMinimum],
        legs: &[ProcLeg],
        idx: usize,
        alt_ft: f64,
    ) -> Option<(usize, f64)> {
        for leg in legs.iter().skip(idx) {
            if let Some(m) = minimums.iter().find(|m| m.fix.eq_ignore_ascii_case(&leg.fix_ident)) {
                if alt_ft >= m.min_ft - MINIMUM_MET_TOL_FT {
                    self.met.insert(m.fix.to_ascii_uppercase());
                }
            }
        }
        legs.iter().enumerate().skip(idx).find_map(|(i, leg)| {
            minimums
                .iter()
                .find(|m| m.fix.eq_ignore_ascii_case(&leg.fix_ident))
                .filter(|m| !self.met.contains(&m.fix.to_ascii_uppercase()))
                .map(|m| (i, m.min_ft))
        })
    }

    /// The highest minimum at a fix still ahead (leg index, feet), met or
    /// not: the level the target may not go under until that fix is flown.
    pub fn floor(
        minimums: &[crate::nav::ClimbMinimum],
        legs: &[ProcLeg],
        idx: usize,
    ) -> Option<(usize, f64)> {
        legs.iter()
            .enumerate()
            .skip(idx)
            .filter_map(|(i, leg)| {
                minimums
                    .iter()
                    .find(|m| m.fix.eq_ignore_ascii_case(&leg.fix_ident))
                    .map(|m| (i, m.min_ft))
            })
            .max_by(|a, b| a.1.total_cmp(&b.1))
    }
}

/// The follower's altitude once its departure owners have spoken, in their
/// order: the plan's (`plan_ft`, the vertical law with the pilot and the
/// ground already applied), raised to a departure minimum still ahead
/// (`minimum_ft`, [`DepartureMinimums::floor`]; never on a published path),
/// then capped at an IFR clearance's altitude until departure clears higher
/// (`cap_at_clearance`: a pilot's own altitude and the terrain floor still
/// outrank the cap). A minimum above the clearance is climbed to only once
/// the check-in has released the cap: the dial and the target are one number
/// and a controller's altitude is never climbed through on a chart's
/// account. Pure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepartureTarget {
    /// The altitude to command.
    pub want_ft: f64,
    /// A departure minimum raised the plan's target.
    pub raised_by_minimum: bool,
    /// The clearance holds the target under a minimum still ahead.
    pub minimum_capped: bool,
    /// The clearance lowered the target (plan or minimum).
    pub cap_limited: bool,
}

pub fn departure_target_ft(
    plan_ft: f64,
    minimum_ft: Option<f64>,
    ceiling_ft: Option<f64>,
    pilot_held: bool,
    terrain_floor_ft: Option<f64>,
) -> DepartureTarget {
    let raised = minimum_ft.filter(|m| *m > plan_ft);
    let uncapped = raised.unwrap_or(plan_ft);
    let want_ft = cap_at_clearance(uncapped, ceiling_ft, pilot_held, terrain_floor_ft);
    DepartureTarget {
        want_ft,
        raised_by_minimum: raised.is_some(),
        minimum_capped: raised.is_some() && want_ft < uncapped - 0.5,
        cap_limited: want_ft < uncapped - 0.5,
    }
}

/// What the follower does with the VS wheel this tick, read from the modes
/// first: [`VsAction::Write`] only while VS mode reads ON. A VS dial written
/// under ALT HOLD reads back 0 (`write_unconfirmed vertical_velocity want
/// 400 actual 0`, EGKK 2026-09-28 with `ap_state` 16386), and the failed
/// write threw the rest of the tick away.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VsAction {
    /// VS mode is on: set the wheel.
    Write,
    /// The autopilot's own capture or hold has the level we want: leave it.
    Leave,
    /// A movement the holding mode will not make: engage VS, then (mode
    /// read again) set the wheel.
    Engage,
}

/// Feet from the dial inside which an ALT HOLD (or capture) on OUR dial is
/// left to finish the level-off: the 737 began capturing 6,000 at 5,817 ft
/// and 962 fpm, 183 ft out (EGKK 2026-09-28).
pub const ALT_HOLD_LEAVE_FT: f64 = 300.0;

/// Pure: `vs_on` the VS mode's status, `alt_hold_on` the altitude hold
/// ENGAGED (`altitude_hold_status` 2, the capture included), `rate_fpm` the
/// rate being flown.
pub fn vs_action(
    vs_on: bool,
    alt_hold_on: bool,
    alt_ft: f64,
    want_ft: f64,
    dial_ft: f64,
    vs_cmd_fpm: f64,
    rate_fpm: f64,
) -> VsAction {
    if vs_on {
        return VsAction::Write;
    }
    let lead = if rate_fpm.abs() > vs_cmd_fpm.abs() { rate_fpm } else { vs_cmd_fpm };
    if vs_mode_capturing(alt_ft, want_ft, dial_ft, lead) {
        return VsAction::Leave;
    }
    // A hold on OUR dial, at it or still closing on it: the capture's own.
    // A hold parked short of the dial (not closing) is not a level-off.
    let off = dial_ft - alt_ft;
    let closing = off * rate_fpm > 0.0 && off.abs() <= ALT_HOLD_LEAVE_FT;
    if alt_hold_on && (dial_ft - want_ft).abs() <= 100.0 && (off.abs() <= 100.0 || closing) {
        return VsAction::Leave;
    }
    VsAction::Engage
}

/// Along-route distance, nm, from the aircraft to leg `target`'s fix: to the
/// current leg's fix, then fix to fix.
pub fn distance_to_leg_nm(lat: f64, lon: f64, legs: &[ProcLeg], idx: usize, target: usize) -> f64 {
    let mut d = 0.0;
    let mut from = (lat, lon);
    for leg in legs.iter().take(target + 1).skip(idx) {
        if let Some(f) = &leg.fix {
            d += crate::nav::haversine_nm(from.0, from.1, f.lat, f.lon);
            from = (f.lat, f.lon);
        }
    }
    d
}

/// What a departure minimum's watch says this tick.
#[derive(Debug, Clone, PartialEq)]
pub enum MinimumWatch {
    OnTrack,
    /// Predicted to cross short: said once, the climb is already at its best.
    Short { predicted_ft: f64 },
    /// Will not be made: the aircraft goes back to the pilot, with the numbers.
    Unable { predicted_ft: f64 },
}

/// Altitude samples (about half a second apart) before the climb rate is a
/// measurement the minimum watch may judge on.
const MIN_RATE_SAMPLES: u32 = 10;
/// Below this the aircraft is not climbing, and a minimum it will miss goes
/// back to the pilot.
const NOT_CLIMBING_FPM: f64 = 200.0;

/// Feet short of a minimum that is still only a warning...
const MIN_WARN_FT: f64 = 100.0;
/// ...and short by this, within [`MIN_UNABLE_NM`] of the fix, is unable.
const MIN_UNABLE_FT: f64 = 300.0;
const MIN_UNABLE_NM: f64 = 3.0;

/// Judge one minimum from the present climb: altitude now, rate (fpm),
/// groundspeed and the distance to the fix. Pure.
pub fn watch_minimum(alt_ft: f64, vs_fpm: f64, gs_kts: f64, dist_nm: f64, min_ft: f64) -> MinimumWatch {
    let predicted_ft = alt_ft + vs_fpm.max(0.0) * dist_nm / gs_kts.max(40.0) * 60.0;
    let short = min_ft - predicted_ft;
    if short >= MIN_UNABLE_FT && dist_nm <= MIN_UNABLE_NM {
        MinimumWatch::Unable { predicted_ft }
    } else if short >= MIN_WARN_FT {
        MinimumWatch::Short { predicted_ft }
    } else {
        MinimumWatch::OnTrack
    }
}

/// The arrival restriction published for this leg's fix, if any.
fn restriction_at<'a>(
    cons: &'a [crate::nav::StarConstraint],
    leg: &ProcLeg,
) -> Option<&'a crate::nav::StarConstraint> {
    cons.iter().find(|c| c.fix.eq_ignore_ascii_case(&leg.fix_ident))
}

/// Clamp a plan's altitudes into its STAR's restrictions, at arm time. Each
/// restricted fix gets its window ([`crate::nav::StarConstraint::clamp_ft`]);
/// past the first restricted fix the profile only goes DOWN unless a fix's
/// own minimum says otherwise, so SimBrief's 8,000 after a 7,000 "at or
/// below" is never a climb back. Returns how many legs it changed.
///
/// A window above the plan's cruise (the highest altitude planned before
/// the first restricted fix) is flown at the cruise, never climbed to
/// ([`windows_above_cruise`] says which): EGKK-EGBB (2026-09-28, sortie
/// 1790601822-full_auto) clamped HEME1B's "HEMEL at FL220" into an FL180
/// plan, the follower climbed for 22,000 and crossed HEMEL at 14,638, the
/// window unmade and unsaid (#156, #32).
pub fn apply_arrival_constraints(legs: &mut [ProcLeg], cons: &[crate::nav::StarConstraint]) -> usize {
    let cruise = plan_cruise_before_star(legs, cons);
    let mut changed = 0;
    let mut prev: Option<f64> = None;
    for leg in legs.iter_mut() {
        let Some(alt) = leg.alt1_ft else { continue };
        let own = restriction_at(cons, leg);
        let mut want = own.map_or(alt, |c| c.clamp_ft(alt));
        if let (Some(c), Some(_)) = (cruise, own) {
            want = want.min(c.max(alt));
        }
        if let Some(p) = prev {
            want = want.min(p);
            if let Some(min) = own.and_then(|c| c.min_ft) {
                want = want.max(min);
            }
        }
        if own.is_some() || prev.is_some() {
            prev = Some(want);
        }
        if (want - alt).abs() > 0.5 {
            leg.alt1_ft = Some(want);
            changed += 1;
        }
    }
    changed
}

/// The plan's cruise as the STAR meets it: the highest altitude planned on
/// the legs before the first restricted fix. `None` when the plan starts on
/// the STAR (nothing before it to call a cruise).
fn plan_cruise_before_star(legs: &[ProcLeg], cons: &[crate::nav::StarConstraint]) -> Option<f64> {
    let first = legs.iter().position(|l| restriction_at(cons, l).is_some())?;
    legs[..first].iter().filter_map(|l| l.alt1_ft).reduce(f64::max)
}

/// A STAR window this far above the cruise is one the plan does not make.
const ABOVE_CRUISE_SLACK_FT: f64 = 100.0;

/// The STAR's "at or above" windows over the plan's cruise
/// ([`plan_cruise_before_star`]): `(fix, window's minimum, cruise)`. The
/// follower flies them at the cruise ([`apply_arrival_constraints`]) and
/// says so once, as a crew briefing the arrival would. Pure.
pub fn windows_above_cruise(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    cruise_ft: f64,
) -> Vec<(String, f64, f64)> {
    let mut out = Vec::new();
    for leg in legs {
        if let Some(min) = restriction_at(cons, leg).and_then(|c| c.min_ft) {
            if min > cruise_ft + ABOVE_CRUISE_SLACK_FT && !out.iter().any(|(f, _, _): &(String, f64, f64)| *f == leg.fix_ident) {
                out.push((leg.fix_ident.clone(), min, cruise_ft));
            }
        }
    }
    out
}

/// A window missed by more than this is said (the card's slack, #32).
const STAR_MISS_SLACK_FT: f64 = 300.0;

/// A STAR altitude window the airframe cannot make from here, at its best.
#[derive(Debug, Clone, PartialEq)]
pub struct WindowMiss {
    pub fix: String,
    /// Under an "at or above" (true) or over an "at or below".
    pub low: bool,
    /// The window's edge judged (a minimum over the cruise is the cruise).
    pub limit_ft: f64,
    pub predicted_ft: f64,
    pub by_ft: f64,
    pub dist_nm: f64,
}

impl WindowMiss {
    /// What a crew says: once, with the number.
    pub fn line(&self) -> String {
        format!(
            "We'll cross {} about {} feet {} its {}.",
            self.fix,
            crate::auto_status::commas(((self.by_ft / 50.0).round() * 50.0) as i64),
            if self.low { "under" } else { "above" },
            crate::flightplan::spoken_feet(self.limit_ft),
        )
    }
}

/// The next STAR altitude window ahead (leg `idx` on), judged from here:
/// climbing at `climb_fpm`, the climb being flown ([`window_climb_fpm`]),
/// to an "at or above" (never
/// above `cruise_ft`: a window over the cruise is flown at the cruise,
/// [`windows_above_cruise`]), descending at `descent_fpm` (the speedbrake's
/// rate) to an "at or below". `Some` when even that misses it by more than
/// [`STAR_MISS_SLACK_FT`]. #32's owed rule: a restriction that cannot be
/// made is said, not silently missed (EGKK-EGBB 2026-09-28: HEMEL crossed
/// at 14,638 against FL220, unsaid). Pure.
#[allow(clippy::too_many_arguments)]
pub fn star_window_miss(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    idx: usize,
    lat: f64,
    lon: f64,
    alt_ft: f64,
    gs_kts: f64,
    climb_fpm: f64,
    descent_fpm: f64,
    cruise_ft: Option<f64>,
) -> Option<WindowMiss> {
    let (k, c) = legs.iter().enumerate().skip(idx).find_map(|(k, l)| {
        restriction_at(cons, l)
            .filter(|c| c.min_ft.is_some() || c.max_ft.is_some())
            .map(|c| (k, c))
    })?;
    let dist_nm = distance_to_leg_nm(lat, lon, legs, idx, k);
    let minutes = dist_nm / gs_kts.max(60.0) * 60.0;
    let fix = legs[k].fix_ident.clone();
    if let Some(min) = c.min_ft.map(|m| cruise_ft.map_or(m, |cr| m.min(cr))) {
        if alt_ft < min {
            let predicted_ft = alt_ft + climb_fpm.max(0.0) * minutes;
            let by_ft = min - predicted_ft;
            if by_ft > STAR_MISS_SLACK_FT {
                return Some(WindowMiss { fix, low: true, limit_ft: min, predicted_ft, by_ft, dist_nm });
            }
        }
    }
    if let Some(max) = c.max_ft {
        if alt_ft > max {
            let predicted_ft = alt_ft - descent_fpm.max(0.0) * minutes;
            let by_ft = predicted_ft - max;
            if by_ft > STAR_MISS_SLACK_FT {
                return Some(WindowMiss { fix, low: false, limit_ft: max, predicted_ft, by_ft, dist_nm });
            }
        }
    }
    None
}

/// The climb a STAR "at or above" is judged on: the rate the follower is
/// commanding this tick (`vs_cmd`, after its speed protection), never the
/// airframe's book rate; `None` while an IFR clearance holds the target
/// down (`cap_limited`: the controller's level, judged once released).
/// EGKK-EGBB (sortie 1790624015-full_auto, 2026-09-28, #173): held level
/// at 6,000, the book's 2,000 fpm said "HEMEL about 350 feet under its
/// 18,000"; it was crossed at 14,587. Pure.
pub fn window_climb_fpm(vs_cmd: f64, cap_limited: bool) -> Option<f64> {
    (!cap_limited).then_some(vs_cmd.max(0.0))
}

/// What a crew says about a window over its cruise, once.
pub fn above_cruise_line(fix: &str, min_ft: f64, cruise_ft: f64) -> String {
    format!(
        "{fix}'s {} is above our cruise. We'll cross it at {}.",
        crate::flightplan::spoken_feet(min_ft),
        crate::flightplan::spoken_feet(cruise_ft),
    )
}

/// The highest altitude leg `idx`'s fix may be crossed at so that every
/// arrival "at or below" further on is still reachable at `ft_per_nm`
/// (the descent law's rate over the groundspeed), never under that fix's
/// own minimum. `None` with no ceiling ahead. Pacing each leg only to its
/// OWN fix left a restriction two fixes on to the last leg: FL070 at TARTN
/// with 12 nm from HAVEN is not a descent that starts at HAVEN. Geometric,
/// so a descent already made is never climbed back out of.
pub fn arrival_ceiling_ft(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    idx: usize,
    ft_per_nm: f64,
) -> Option<f64> {
    let from = legs.get(idx)?.fix.as_ref()?;
    let mut ceiling: Option<f64> = None;
    for (k, leg) in legs.iter().enumerate().skip(idx) {
        let Some(max) = restriction_at(cons, leg).and_then(|c| c.max_ft) else {
            continue;
        };
        let d = distance_to_leg_nm(from.lat, from.lon, legs, idx, k);
        let c = max + path_nm(d) * ft_per_nm;
        ceiling = Some(ceiling.map_or(c, |x: f64| x.min(c)));
    }
    let floor = restriction_at(cons, &legs[idx]).and_then(|c| c.min_ft);
    ceiling.map(|c| floor.map_or(c, |f| c.max(f)))
}

/// The plan's OWN altitudes ahead as ceilings, the same way: this fix's
/// target is lowered to what still makes every later fix's planned altitude
/// at `ft_per_nm`. Only fixes after `idx`, and only a plain or "at or below"
/// altitude (an approach's "at or above" is a floor, never a ceiling).
/// LIME-LIPE (2026-09-25): each leg was paced only to its own fix, so the
/// descent to PE725's 8,000 ft could not start until PE730 was behind, with
/// four miles to lose six thousand feet: 14,200 ft at 11 nm from the field
/// and the pilot stepping the dial down by hand.
pub fn profile_ceiling_ft(legs: &[ProcLeg], idx: usize, ft_per_nm: f64) -> Option<f64> {
    let from = legs.get(idx)?.fix.as_ref()?;
    let mut ceiling: Option<f64> = None;
    for (k, leg) in legs.iter().enumerate().skip(idx + 1) {
        if !matches!(leg.alt_desc, ' ' | '@' | '-') {
            continue;
        }
        let Some(alt) = leg.alt1_ft else { continue };
        let d = distance_to_leg_nm(from.lat, from.lon, legs, idx, k);
        let c = alt + path_nm(d) * ft_per_nm;
        ceiling = Some(ceiling.map_or(c, |x: f64| x.min(c)));
    }
    ceiling
}

/// The miles of a later fix's distance a ceiling may climb on: all of it
/// but the descent's lead ([`TOD_LEAD_NM`]), which the path to that fix
/// spends level at its altitude. The ceilings and the path are then ONE
/// line: crossing a fix at its ceiling puts the aircraft exactly on the next
/// leg's path. With the whole distance counted, every leg of the LEMG UNTO2A
/// began a lead's worth of gradient above its own path (2026-09-26).
fn path_nm(d: f64) -> f64 {
    if d <= 0.0 {
        0.0
    } else {
        (d - TOD_LEAD_NM).max(0.0)
    }
}

/// Feet per nm the arrival ceiling is planned on at this groundspeed: the
/// SAME planning rate the descent path is flown at
/// ([`DESCENT_PLAN_FRACTION`]). It was 85 % against the path's 75 %, so a
/// fix crossed at its ceiling was above the next leg's path and each leg
/// started by catching up at the full rate, the margin a speed limit needs
/// already spent (FOFUS 386 ft high, LEMG 2026-09-26).
pub fn arrival_ft_per_nm(descent_fpm: f64, gs_kts: f64) -> f64 {
    DESCENT_PLAN_FRACTION * descent_fpm / (gs_kts.max(60.0) / 60.0)
}

/// The altitude leg `idx` is flown to once the ceilings ahead
/// ([`arrival_ceiling_ft`], [`profile_ceiling_ft`]) are applied: `Some((alt,
/// ceiling))` when a ceiling lowered the plan's altitude, `None` otherwise.
/// `latch` holds the lowest ceiling seen on this leg: the ceiling is a
/// gradient over the groundspeed, so it RISES as the aircraft slows, and an
/// unlatched one lifted MG539's target from 14,900 to 15,500 ft in the last
/// two miles and the 737 climbed at +260 fpm into its 230 kt slow-down
/// (LEMG 2026-09-26). A descent's target never climbs back.
pub fn ceilinged_leg_alt(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    idx: usize,
    gs_kts: f64,
    descent_fpm: f64,
    latch: &mut Option<(usize, f64)>,
) -> Option<(f64, f64)> {
    let grad = arrival_ft_per_nm(descent_fpm, gs_kts);
    let ceiling = match (
        arrival_ceiling_ft(legs, cons, idx, grad),
        profile_ceiling_ft(legs, idx, grad),
    ) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (a, b) => a.or(b),
    };
    let ceiling = match (ceiling, *latch) {
        (Some(c), Some((i, l))) if i == idx => Some(c.min(l)),
        (c, _) => c,
    };
    if let Some(c) = ceiling {
        *latch = Some((idx, c));
    }
    match (legs.get(idx)?.alt1_ft, ceiling) {
        (Some(a), Some(c)) if c < a - 50.0 => Some(((c / 100.0).floor() * 100.0, c)),
        _ => None,
    }
}

/// Knots a second the governor is planned to lose speed at in a descent.
const ARRIVAL_DECEL_KTS_S: f64 = 0.5;
/// Slowed this far before the fix, nm, on top of the deceleration itself.
const ARRIVAL_SPEED_LEAD_NM: f64 = 2.0;
/// Knots under a STAR's speed LIMIT the follower flies: a maximum is not a
/// target, and the speed held down a descent at idle wanders a few knots
/// either side of what is asked (OSKOT and GROVE's 210 crossed at 216 and
/// 218, EGKK-EGBB 2026-09-28, #156).
pub const STAR_SPEED_MARGIN_KTS: f64 = 3.0;

/// The arrival's speed limit now: the lowest of every limit already passed
/// (a limit holds from its fix on) and every one ahead whose slow-down has
/// begun, at [`ARRIVAL_DECEL_KTS_S`] from `ias_kts` plus the lead. `(knots,
/// fix)`; `None` when nothing applies yet.
///
/// A limit whose slow-down has BEGUN holds (`begun`, the legs' indices,
/// carried tick to tick): the start is judged from the speed still to lose,
/// so an aircraft slowing at the planned rate rides the boundary and the
/// limit came and went with it. EGKK-EGBB (2026-09-27, #126): OSKOT's 210
/// against 250, 8.9 nm out at 257 kt, flipped the governor's target five
/// times in 27 s, and each flip moved the speedbrake, said aloud.
pub fn arrival_speed_cap(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    idx: usize,
    lat: f64,
    lon: f64,
    ias_kts: f64,
    gs_kts: f64,
    begun: &mut std::collections::HashSet<usize>,
) -> Option<(f64, String)> {
    let mut cap: Option<(f64, String)> = None;
    for (k, leg) in legs.iter().enumerate() {
        let Some(kts) = restriction_at(cons, leg).and_then(|c| c.max_kts) else {
            continue;
        };
        let applies = k < idx || begun.contains(&k) || {
            let d = distance_to_leg_nm(lat, lon, legs, idx, k);
            let slow_nm = (ias_kts - kts).max(0.0) / ARRIVAL_DECEL_KTS_S * gs_kts.max(60.0) / 3600.0;
            d <= slow_nm + ARRIVAL_SPEED_LEAD_NM
        };
        if applies {
            begun.insert(k);
        }
        if applies && cap.as_ref().is_none_or(|(c, _)| kts < *c) {
            cap = Some((kts, leg.fix_ident.clone()));
        }
    }
    cap
}

/// The speed the NEXT enroute plan hands its arrival over at, consumed at
/// its arm (full auto's, before an arrival autoland).
static NEXT_ARRIVAL_HANDOFF_KTS: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Hand the next enroute plan the speed its arrival starts at (`None` clears
/// one left by a plan that never started).
pub fn set_next_arrival_handoff_kts(kts: Option<f64>) {
    *NEXT_ARRIVAL_HANDOFF_KTS
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = kts.filter(|k| k.is_finite() && *k > 0.0);
}

fn take_arrival_handoff_kts() -> Option<f64> {
    NEXT_ARRIVAL_HANDOFF_KTS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .take()
}

/// The arrival's hand-over speed as a limit at the END of the route, slowed
/// for in time the way a STAR limit is ([`arrival_speed_cap`]: at
/// [`ARRIVAL_DECEL_KTS_S`] from `ias_kts` plus [`ARRIVAL_SPEED_LEAD_NM`]),
/// and held once begun (`begun`). `to_end_nm` is the route still to fly.
/// `None` while the slow-down is still ahead. Pure.
///
/// EGBB-EGNX (2026-09-30, sortie 1790761044-full_auto, #199): a 40 nm local
/// plan cruised at 3,000 ft and 250 kt to its last fix, and autoland was
/// handed a clean 737 at 252 kt on the path 8.4 nm out. It flew the final at
/// 255 with the flaps held up by their placards. Autoland expects its clean
/// approach speed (`autoland::clean_approach_ias`, 210 for the 737) there.
pub fn arrival_handoff_cap(
    to_end_nm: f64,
    ias_kts: f64,
    gs_kts: f64,
    handoff_kts: f64,
    begun: &mut bool,
) -> Option<f64> {
    if !*begun {
        let slow_nm =
            (ias_kts - handoff_kts).max(0.0) / ARRIVAL_DECEL_KTS_S * gs_kts.max(60.0) / 3600.0;
        *begun = to_end_nm <= slow_nm + ARRIVAL_SPEED_LEAD_NM;
    }
    begun.then_some(handoff_kts)
}

/// The true great-circle bearing to a leg's fix, or `None` for a to-altitude
/// leg (no fix to steer at). Pure — the lateral-steering decision, testable
/// without a sim.
pub fn bearing_to_fix(lat: f64, lon: f64, leg: &ProcLeg) -> Option<f64> {
    leg.fix
        .as_ref()
        .map(|f| crate::nav::bearing_deg(lat, lon, f.lat, f.lon))
}

/// The turn radius an enroute turn is actually flown at, nautical miles:
/// v²/(g·tan φ) at [`ENROUTE_BANK_DEG`].
///
/// NOT [`crate::circuit::turn_radius_at`], whose 250..2500 m clamp is a
/// PATTERN bound — 2500 m is 1.35 nm, which is already less than this
/// airframe's real radius at climb speed and a ninth of it at cruise. A clamp
/// that exists to keep a light aircraft's circuit geometry sane must not be
/// the thing that decides whether an airliner can reach its next fix.
pub fn enroute_turn_radius_nm(gs_kts: f64) -> f64 {
    let v = gs_kts.max(40.0) * 0.514_444;
    v * v / (9.81 * ENROUTE_BANK_DEG.to_radians().tan()) / 1852.0
}

/// Turn anticipation: how far before the fix the turn onto the next leg has to
/// begin, which is therefore how far out the leg counts as flown.
///
/// `R·tan(θ/2)` — the standard fly-by geometry, the distance at which a circle
/// of radius `R` is tangent to both legs — floored at [`CAPTURE_NM`] and
/// capped at `R` so a near-reversal can't ask for an unbounded lead.
///
/// The shape is what makes it right on both airframes: a 172 at 100 knots has
/// a 0.4 nm radius, so even a 90° turn asks for less than the floor and its
/// behaviour is unchanged; a 737 at cruise has an 8 nm radius but a 10° track
/// change, which also lands under the floor. It only bites where it must —
/// a big radius AND a real turn, which is exactly the jet departure that
/// orbited.
pub fn capture_radius_nm(gs_kts: f64, turn_deg: f64) -> f64 {
    let r = enroute_turn_radius_nm(gs_kts);
    let half = (turn_deg.abs().min(175.0) / 2.0).to_radians();
    (r * half.tan()).clamp(CAPTURE_NM, r.max(CAPTURE_NM))
}

/// The track change at this leg's fix: the bearing we are flying INTO it
/// against the bearing from it on to the next leg's fix, signed right-positive.
/// `None` when either end has no fix to measure between — a to-altitude leg,
/// an unresolved fix, or the last leg of the route, none of which are turns
/// this anticipates.
/// Seconds a leg's first turn is given before the closure watchdog may judge
/// it: the turn at 2 degrees a second, slower than standard rate because the
/// bug leads the nose by only 25 degrees until the bank is established.
pub fn leg_turn_grace_s(turn_deg: f64) -> f64 {
    turn_deg.abs().min(180.0) / 2.0
}

pub fn leg_turn_deg(lat: f64, lon: f64, leg: &ProcLeg, next: Option<&ProcLeg>) -> Option<f64> {
    let here = leg.fix.as_ref()?;
    let there = next?.fix.as_ref()?;
    let inbound = crate::nav::bearing_deg(lat, lon, here.lat, here.lon);
    let outbound = crate::nav::bearing_deg(here.lat, here.lon, there.lat, there.lon);
    Some(((outbound - inbound + 540.0).rem_euclid(360.0)) - 180.0)
}

/// Has the current leg been flown? A fix leg completes inside `capture_nm`
/// (see [`capture_radius_nm`] — the turn onto the next leg has to START
/// there, so that IS the fix being reached); a to-altitude leg completes when
/// the altitude constraint is satisfied. Pure and position-based, so
/// sequencing is immune to how slow the AP's turns are (the same lesson that
/// drove the circuit off timers).
/// A DME-terminated leg (CD, VD, FD: "the 291 course to 6 DME from RIM") is
/// done when the aircraft is that far from its navaid. `None` for any other
/// leg. Its altitude is a CONSTRAINT, not the terminator: judged by altitude
/// alone, ANC8A's "to 6 DME, at or above 1,500" would have turned back at
/// 1,500 feet, two miles out. Pure.
pub fn dme_leg_done(leg: &ProcLeg, lat: f64, lon: f64) -> Option<bool> {
    if !matches!(leg.leg_type.as_str(), "CD" | "VD" | "FD") {
        return None;
    }
    let (n, d) = (leg.navaid.as_ref()?, leg.dist_nm.filter(|d| *d > 0.0)?);
    Some(crate::nav::haversine_nm(lat, lon, n.lat, n.lon) >= d - 0.1)
}

/// [`leg_captured`], DME-terminated legs first.
pub fn leg_done(leg: &ProcLeg, dist_to_fix_nm: Option<f64>, alt_ft: f64, capture_nm: f64, lat: f64, lon: f64) -> bool {
    dme_leg_done(leg, lat, lon).unwrap_or_else(|| leg_captured(leg, dist_to_fix_nm, alt_ft, capture_nm))
}

pub fn leg_captured(
    leg: &ProcLeg,
    dist_to_fix_nm: Option<f64>,
    alt_ft: f64,
    capture_nm: f64,
) -> bool {
    match dist_to_fix_nm {
        Some(d) => d <= capture_nm.max(CAPTURE_NM),
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

/// The altitude an IFR clearance holds the climb to until the departure
/// controller clears higher (`hold_at_cleared`): a CEILING on the plan's
/// climb, never a target of its own, set by the crew before the follower
/// starts and released at the departure check-in. A pilot's own altitude
/// outranks it; the ground ahead and an unmet SID minimum outrank both.
static CLEARED_CEILING: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Hold the climb at the clearance's altitude (`Some`) or let it go on to
/// the plan (`None`). Blackboxed either way.
pub fn hold_at_cleared(ft: Option<f64>, why: &str) {
    let mut c = CLEARED_CEILING.lock().unwrap_or_else(|e| e.into_inner());
    if *c == ft {
        return;
    }
    *c = ft;
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "cleared_ceiling", "ft": ft, "why": why}),
    );
}

/// The clearance's ceiling on the climb, while one holds.
pub fn cleared_ceiling() -> Option<f64> {
    *CLEARED_CEILING.lock().unwrap_or_else(|e| e.into_inner())
}

/// The follower's altitude under a cleared ceiling: capped at it unless the
/// PILOT has taken the vertical, and never below the terrain floor the
/// vertical law already applied. Pure.
pub fn cap_at_clearance(
    want_ft: f64,
    ceiling_ft: Option<f64>,
    pilot_held: bool,
    floor_ft: Option<f64>,
) -> f64 {
    match ceiling_ft {
        Some(c) if !pilot_held && want_ft > c => c.max(floor_ft.unwrap_or(f64::MIN)),
        _ => want_ft,
    }
}

fn clear_pilot_altitude() {
    *PILOT_ALT.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *PILOT_SPEED.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// A speed the PILOT asked for while the follower flies, the altitude's twin:
/// the follower owns the throttle (one writer), so "set speed 110" was refused
/// with "the approach has the throttle" in an enroute cruise the pilot could
/// see was slow, and there was no way to change it (LIPE-LIPK, 2026-09-25).
/// Flown as the plan's cruise speed, still floored, capped by the airframe's
/// ceiling, 250 below 10,000 and a STAR's limits; cleared with the altitude.
static PILOT_SPEED: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Hand the follower a speed. `false` (nothing taken) when no procedure flies.
pub fn take_speed(kts: f64) -> bool {
    if !ACTIVE.load(Ordering::Relaxed) {
        return false;
    }
    *PILOT_SPEED.lock().unwrap_or_else(|e| e.into_inner()) = Some(kts);
    crate::blackbox::record("procedure", serde_json::json!({"event": "pilot_speed", "kts": kts}));
    true
}

/// The pilot's speed, if they have given one.
pub fn pilot_speed() -> Option<f64> {
    if !ACTIVE.load(Ordering::Relaxed) {
        return None;
    }
    *PILOT_SPEED.lock().unwrap_or_else(|e| e.into_inner())
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

/// The GA vertical rate, fpm, and the default for any airframe whose profile
/// declares nothing — the figure this law flew on every airframe until
/// 2026-09-20.
const GA_VS_FPM: f64 = 700.0;

/// The enroute climb and descent rates for the LIVE airframe, republished on
/// every profile swap by [`crate::flight_state::publish_airframe`] — a
/// process-wide fact must never outlive its airframe.
static CLIMB_FPM: std::sync::Mutex<f64> = std::sync::Mutex::new(GA_VS_FPM);
static DESCENT_FPM: std::sync::Mutex<f64> = std::sync::Mutex::new(GA_VS_FPM);
static BEST_CLIMB_FPM: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Publish the airframe's enroute vertical rates. Called on EVERY swap, with
/// the GA figure when the profile is silent, so the last aircraft's rates can
/// never be pointed at the next one.
pub fn publish_vs_limits(r: Option<&crate::aircraft::ReferenceProfile>) {
    let pick = |v: Option<f64>| v.filter(|f| f.is_finite() && *f > 0.0).unwrap_or(GA_VS_FPM);
    *CLIMB_FPM.lock().unwrap_or_else(|e| e.into_inner()) =
        pick(r.and_then(|r| r.enroute_climb_fpm));
    *DESCENT_FPM.lock().unwrap_or_else(|e| e.into_inner()) =
        pick(r.and_then(|r| r.enroute_descent_fpm));
    *BEST_CLIMB_FPM.lock().unwrap_or_else(|e| e.into_inner()) =
        r.and_then(|r| r.best_climb_fpm).filter(|f| f.is_finite() && *f > 0.0);
}

/// The rate a climb to a departure minimum is flown at: the airframe's best
/// climb where it declares one, never less than the enroute rate.
pub fn best_climb_fpm() -> f64 {
    BEST_CLIMB_FPM
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .unwrap_or(0.0)
        .max(climb_fpm())
}

/// The live airframe's enroute climb rate, fpm.
pub fn climb_fpm() -> f64 {
    *CLIMB_FPM.lock().unwrap_or_else(|e| e.into_inner())
}

/// The live airframe's enroute descent rate, fpm (magnitude). This both caps
/// the commanded rate and places the top of descent, so the two can never
/// disagree: the TOD is put exactly where this rate makes the constraint.
pub fn descent_fpm() -> f64 {
    *DESCENT_FPM.lock().unwrap_or_else(|e| e.into_inner())
}

/// The rate the follower's vertical law will actually fly, fpm (magnitude):
/// what a readback must promise when the follower owns the altitude, whatever
/// rate the pilot asked for.
pub fn vs_clamp_fpm() -> f64 {
    descent_fpm()
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
#[cfg(test)]
pub fn descent_target_ft(
    leg_alt: f64,
    carry: f64,
    alt_ft: f64,
    dist_nm: Option<f64>,
    gs_kts: f64,
    descent_fpm: f64,
) -> f64 {
    descent_target(leg_alt, carry, alt_ft, dist_nm, gs_kts, descent_fpm).0
}

/// [`descent_target_ft`] and the target's own rate, fpm: the path's planned
/// descent while the target is moving down it, zero when it is a level. The
/// VS law feeds it forward ([`vs_law_fpm`]): a proportional law alone trails
/// a ramp by rate over gain, 500 ft at the 737's 1,500 fpm path, and the
/// path ends at the fix.
pub fn descent_target(
    leg_alt: f64,
    carry: f64,
    alt_ft: f64,
    dist_nm: Option<f64>,
    gs_kts: f64,
    descent_fpm: f64,
) -> (f64, f64) {
    let Some(dist_nm) = dist_nm else {
        return (leg_alt, 0.0);
    };
    if descent_point_nm(leg_alt, alt_ft, gs_kts, descent_fpm).is_none() {
        return (leg_alt, 0.0);
    }
    let held = if carry <= alt_ft + 100.0 {
        carry
    } else {
        (alt_ft / 100.0).round() * 100.0
    };
    // Down a PATH at the planning rate, not a dive at the maximum: the
    // constraint plus the distance still to run times the planned gradient,
    // never above the level held, never below the constraint. Stepping the
    // target straight to the constraint at the top of descent made the VS
    // law ask the airframe's full rate at once, and the 737 went down its
    // LEMG arrival in 2,000 fpm dives with the speedbrake out between level
    // segments (2026-09-26). A late start still catches the path at the
    // full rate; the path itself is the gentler one.
    let ft_per_nm = descent_fpm * DESCENT_PLAN_FRACTION / (gs_kts.max(60.0) / 60.0);
    let path = leg_alt + (dist_nm - TOD_LEAD_NM).max(0.0) * ft_per_nm;
    let want = path.min(held).max(leg_alt);
    // Moving only while the PATH is what is commanded (not the level held
    // before the top of descent, not the constraint after the lead).
    let rate = if path < held && path > leg_alt {
        -descent_fpm * DESCENT_PLAN_FRACTION
    } else {
        0.0
    };
    (want, rate)
}

/// The follower's vertical speed law: the target's own rate fed forward,
/// plus three fpm a foot of error, inside the airframe's rates.
pub fn vs_law_fpm(want_alt: f64, alt_ft: f64, target_fpm: f64, descent_fpm: f64, climb_cap: f64) -> f64 {
    (target_fpm + (want_alt - alt_ft) * VS_GAIN_PER_FT).clamp(-descent_fpm, climb_cap)
}

/// Fpm of commanded vertical speed per foot off the target.
const VS_GAIN_PER_FT: f64 = 3.0;

/// The share of the airframe's descent rate a descent is PLANNED at: the
/// rest is margin to catch up with, as a crew plans 3 miles per thousand
/// feet and keeps the idle-and-speedbrake rate in hand.
pub const DESCENT_PLAN_FRACTION: f64 = 0.75;

/// Where the descent to `leg_alt` begins: miles BEFORE the fix, at the rate
/// the VS law flies and the groundspeed we have, plus the lead. `None` when
/// there is no descent to pace (the constraint is not more than 100 ft below
/// us). The one formula behind [`descent_target_ft`] and [`planned_tod`].
pub fn descent_point_nm(leg_alt: f64, alt_ft: f64, gs_kts: f64, descent_fpm: f64) -> Option<f64> {
    let drop_ft = alt_ft - leg_alt;
    if drop_ft <= 100.0 {
        return None;
    }
    Some(drop_ft / (descent_fpm * DESCENT_PLAN_FRACTION).max(1.0) * (gs_kts.max(60.0) / 60.0) + TOD_LEAD_NM)
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
    /// `want_alt`'s own rate, fpm (negative down a descent path, zero for a
    /// level), for the VS law's feed-forward.
    pub target_fpm: f64,
    /// [`DescentProfile::peak_ft`] for the next tick.
    pub peak_ft: f64,
    /// A plan altitude above the level already commanded in the descent,
    /// held rather than climbed to: `(plan_ft, held_ft)`.
    pub descent_held: Option<(f64, f64)>,
    /// A plan altitude below the level already commanded in the CLIMB,
    /// held rather than descended to: `(plan_ft, held_ft)`.
    pub climb_held: Option<(f64, f64)>,
}

/// Where the flight is on the plan's profile, for [`vertical_tick`]: once it
/// has begun its descent, a later plan altitude ABOVE the level already
/// commanded is a navlog prediction, not a step climb.
///
/// LIEA-LEBL, 2026-09-27 (sortie 1790494983-full_auto): the 737 began its
/// descent from 31,000 at 08:23:46, levelled at MUREN's 28,000, then armed
/// CHELY at 29,000 and CLIMBED to 28,999 before descending again for VERSO.
/// SimBrief's navlog altitudes down a descent are where it predicts the
/// aircraft to be, and a crew that has started down never climbs back.
///
/// And the mirror: until the cruise is REACHED, a plan altitude below the
/// level already commanded is not a descent. LFBD-EGKK, 2026-09-27 (sortie
/// 1790510400-full_auto): climbing through 7,638 ft for FL340, the CNA6R's
/// "course 296, at or above 660 ft" leg dialled 660, the terrain floor raised
/// it to 705 ("holding 800 feet") and, the cruise having been COMMANDED from
/// the first leg, the old rule called that a descent from 34,000 and held
/// BD923's 34,000 at 705: 14 minutes at 1,200 ft until the pilot dialled it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DescentProfile {
    /// The plan's cruise: its highest leg altitude. `None` switches the rule
    /// off (an approach's own procedure, a plan without altitudes).
    pub cruise_ft: Option<f64>,
    /// The highest altitude FLOWN this flight, carried tick to tick. Flown,
    /// not commanded: FL340 is commanded from the runway's first leg.
    pub peak_ft: f64,
    /// The leg is past the plan's last leg at its cruise altitude: whatever
    /// follows is the descent, which covers a follower armed mid-descent.
    pub past_top: bool,
    /// The arrival's ceilings lowered this leg (`ceilinged_leg_alt`): the
    /// plan's own descent begun before the cruise was reached, not a dip.
    pub arrival_ceiling: bool,
}

impl DescentProfile {
    /// The rule off.
    #[cfg(test)]
    pub fn none() -> Self {
        Self::default()
    }

    /// Whether a flight that last commanded `prev_cmd` has begun its descent
    /// on the plan's profile: it has flown the cruise and now commands below
    /// it, or it is past the plan's top and below the cruise. A step climb
    /// BEFORE the top of descent is neither: the cruise is not reached yet.
    pub fn descending(&self, prev_cmd: f64) -> bool {
        let Some(cruise) = self.cruise_ft else {
            return false;
        };
        (self.past_top || self.cruise_reached(cruise)) && prev_cmd < cruise - STEP_FT
    }

    /// Whether the flight is still climbing to the plan's cruise: never
    /// flown it, not past the plan's top, no arrival ceiling in force.
    pub fn climbing(&self) -> bool {
        self.cruise_ft
            .is_some_and(|c| !self.past_top && !self.arrival_ceiling && !self.cruise_reached(c))
    }

    fn cruise_reached(&self, cruise: f64) -> bool {
        self.peak_ft >= cruise - STEP_FT
    }
}

/// Feet a level must differ by to count as a step (the same 100 ft the
/// descent pacing treats as no descent at all).
const STEP_FT: f64 = 100.0;

/// The index of the plan's last leg at its cruise altitude (within a step):
/// every leg after it is the descent.
pub fn plan_top_index(legs: &[ProcLeg], cruise_ft: f64) -> Option<usize> {
    legs.iter()
        .rposition(|l| l.alt1_ft.is_some_and(|a| a >= cruise_ft - STEP_FT))
}

/// Order of precedence, each one a rule learned the hard way: the plan's
/// constraint paced to its top of descent, then the PILOT's altitude if they
/// have taken it, then the ground AHEAD, which outranks everyone: no target
/// the follower commands may be into terrain, whoever asked for it.
/// The terrain floor is TRUE feet (probed ground plus clearance); the dial is
/// indicated. The live true-minus-indicated offset (temperature and QNH error
/// together) converts it, so a hot day never eats into the clearance.
pub fn floor_indicated(floor_true_ft: f64, true_alt_ft: f64, indicated_alt_ft: f64) -> f64 {
    floor_true_ft - (true_alt_ft - indicated_alt_ft)
}

const SPEEDBRAKE: &str = "sim/cockpit2/controls/speedbrake_ratio";
/// The flight detent: half. Full is a ground spoiler setting on most jets.
const SPEEDBRAKE_FLIGHT: f64 = 0.5;
/// The follower put the speedbrake out, so the follower puts it away. A
/// lever the pilot pulled is the pilot's.
static DRAG_OURS: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
/// Over the governor's target by this much in a descent: brakes out...
const DRAG_OUT_KTS: f64 = 8.0;
/// ...and back in this near it (hysteresis: a comparator on airspeed cycles).
const DRAG_IN_KTS: f64 = 3.0;
/// A commanded descent gentler than this is not one the brakes are for.
const DRAG_MIN_DESCENT_FPM: f64 = 500.0;
/// A brake already out stays out through a level-off or a shallowing path
/// (the speed is still owed); only a commanded CLIMB past this puts it in.
/// The descent gate alone put it in 5 s after it went out at 216 kt when the
/// VS law eased to +131 fpm on a level-off (EGKK-EGBB +2739, 2026-09-27).
const DRAG_KEEP_BELOW_FPM: f64 = 300.0;
/// Seconds the brake keeps a state before the speed rule may change it
/// (the configuration bar and a lost target still act at once).
const DRAG_MIN_HOLD_S: f64 = 15.0;
/// Seconds between spoken speedbrake calls: a change sooner than this is
/// flown and blackboxed, not said.
const DRAG_SPEAK_GAP_S: f64 = 120.0;
/// Feet above the level commanded now at which a descent held back for its
/// speed ([`speed_protected_vs`]) gets the speedbrake: the height is owed and
/// idle alone cannot lose it at the speed allowed...
const DRAG_HIGH_FT: f64 = 400.0;
/// ...and back within this of it, the brake goes in.
const DRAG_ON_PATH_FT: f64 = 150.0;
/// Slower than the target by this, a high descent needs no brake: the
/// protection lets the whole rate through.
const DRAG_SLOW_KTS: f64 = 4.0;

/// The airspeed trend's smoothing for the speed protection, seconds.
const IAS_TREND_TAU_S: f64 = 2.0;

/// A descent is flown at IDLE with the speed on the pitch, as a crew flies
/// one (level change, VNAV SPD): while the vertical law wants down, the
/// governor's target sits [`IDLE_DESCENT_MARGIN_KTS`] under the speed owed,
/// so its lever stays closed at the speed, and [`speed_protected_vs`] holds
/// the speed owed by giving way. Power comes back only past the margin.
///
/// EGKK-EGBB (2026-09-28, sortie 1790601822-full_auto): OSKOT's 210 was made
/// level, the governor overshot to 204 and was adding power (lever 0.3) as
/// the descent to GROVE began: 216 at OSKOT, 218 at GROVE, with the
/// speedbrake out and in three times in the join after. With the governor
/// at the speed owed, every start of a descent was a lever still coming back.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct IdleDescent {
    on: bool,
}

/// The vertical law wanting this much descent starts an idle descent...
const IDLE_DESCENT_ON_FPM: f64 = -800.0;
/// ...and wanting less than this ends it (a level-off's last feet).
const IDLE_DESCENT_OFF_FPM: f64 = -300.0;
/// Knots under the speed owed the governor holds in an idle descent.
pub const IDLE_DESCENT_MARGIN_KTS: f64 = 5.0;

impl IdleDescent {
    /// Update from the descent the vertical law wants, fpm; whether the
    /// descent is flown at idle.
    pub fn step(&mut self, vs_want_fpm: f64) -> bool {
        if self.on {
            if vs_want_fpm >= IDLE_DESCENT_OFF_FPM {
                self.on = false;
            }
        } else if vs_want_fpm <= IDLE_DESCENT_ON_FPM {
            self.on = true;
        }
        self.on
    }

    pub fn on(&self) -> bool {
        self.on
    }

    /// The governor's target for the speed owed `kts`, never under `floor`.
    pub fn governor_kts(&self, kts: f64, floor: Option<f64>) -> f64 {
        if self.on {
            crate::aircraft::floored_target(kts - IDLE_DESCENT_MARGIN_KTS, floor).min(kts)
        } else {
            kts
        }
    }
}
/// Knots over the governor's target, projected [`SPEED_PROTECT_LEAD_S`]
/// ahead on the airspeed trend, at which a descent starts giving way...
const SPEED_PROTECT_FROM_KTS: f64 = 0.0;
/// ...and this many knots more flies it level, where a jet at idle loses
/// speed (the 737 about 0.9 kt/s at 8,000 ft, fitted on sortie
/// 1790601822-full_auto).
const SPEED_PROTECT_SPAN_KTS: f64 = 6.0;
/// The airspeed trend's lead, seconds.
const SPEED_PROTECT_LEAD_S: f64 = 3.0;

/// The descent a speed over its target may keep, fpm: the commanded descent
/// given way in proportion to the projected excess, level at
/// [`SPEED_PROTECT_FROM_KTS`] + [`SPEED_PROTECT_SPAN_KTS`] over. A crew
/// descending at idle on a speed limit flies the SPEED and lets the path
/// go (the speed is owed to the fix and the approach; the height can be
/// lost with the speedbrake), where the VS law alone flew the path and let
/// the speed go. Climbs, level flight and an unknown target pass through.
///
/// EGKK-EGBB 737 (2026-09-28, sortie 1790601822-full_auto, #156, #32):
/// 2,000 fpm at idle down the HEME1B, PUFAX crossed at 254 against 250,
/// OSKOT 216 and GROVE 218 against 210, and six speedbrake extensions
/// each started at 8 kt over. The fitted plant descends at constant speed
/// at idle at about 2.9 degrees; a 2,000 fpm catch-up is four and gains
/// 0.3 kt/s. Pure.
pub fn speed_protected_vs(vs_cmd: f64, over_kts: Option<f64>, trend_kts_s: f64) -> f64 {
    let Some(over) = over_kts.filter(|o| o.is_finite()) else {
        return vs_cmd;
    };
    if vs_cmd >= 0.0 {
        return vs_cmd;
    }
    let proj = over + trend_kts_s.clamp(-3.0, 3.0) * SPEED_PROTECT_LEAD_S;
    let keep = 1.0 - ((proj - SPEED_PROTECT_FROM_KTS) / SPEED_PROTECT_SPAN_KTS).clamp(0.0, 1.0);
    vs_cmd * keep
}

/// What a descent that is running away with the speed gets: whether the
/// speedbrake should be out, and the fraction of the commanded rate to fly.
///
/// EGPH-EGTE, 2026-09-21: 2,000 fpm down with the governor's lever already at
/// idle and nothing else to slow with. 258 kt through ten thousand against a
/// 250 target, 267 at six, 278 at the last fix, where the approach armed. A
/// governor with no drag device can only watch a descent accelerate. A crew
/// pulls the speedbrake, and if that is not enough it shallows the descent:
/// the speed is owed to the approach that starts at the bottom, the height
/// can be lost later. Pure; `out` is the brake's state going in, `held_s`
/// how long it has had it (`f64::INFINITY` when it never changed).
///
/// EGKK-EGBB, 2026-09-27 (sortie 1790541304-full_auto, #126): out and in
/// around the approach's 210.25 at +2621/+2691, +2735/+2739 and +2962/+3004,
/// every change spoken. Two of the three "in"s were the descent gate, not
/// the speed: the VS law easing through -500 fpm at a level-off with the
/// aircraft still 6 and 13 kt fast. Out, the brake now stays out through a
/// level-off until the speed is in or a climb is commanded, and neither
/// state is left inside [`DRAG_MIN_HOLD_S`].
///
/// EGKK-EGBB, 2026-09-28 (sortie 1790601822-full_auto, #156, #126): six
/// extensions down the HEME1B and three more in the join, every one at
/// 8 kt over. The shallower descent is now [`speed_protected_vs`]'s, flown
/// before the speed runs away, so the brake is for what a crew uses it
/// for: HEIGHT the protected descent cannot lose. Out when the aircraft is
/// [`DRAG_HIGH_FT`] above the level commanded (`high_ft`, the target's
/// shortfall) with the speed not slow, or a runaway past
/// [`DRAG_OUT_KTS`]; in once back within [`DRAG_ON_PATH_FT`] with the speed
/// in, or [`DRAG_SLOW_KTS`] slow, or a climb. `vs_want_fpm` is the descent
/// the vertical law WANTS, before the speed protection held it back. Pure.
pub fn descent_drag(
    out: bool,
    held_s: f64,
    ias_kts: f64,
    target_kts: Option<f64>,
    vs_want_fpm: f64,
    high_ft: f64,
) -> bool {
    let Some(target) = target_kts else {
        return false;
    };
    let over = ias_kts - target;
    let settled = held_s >= DRAG_MIN_HOLD_S;
    if out {
        let done = over < -DRAG_SLOW_KTS || (over <= DRAG_IN_KTS && high_ft <= DRAG_ON_PATH_FT);
        vs_want_fpm < DRAG_KEEP_BELOW_FPM && (!done || !settled)
    } else {
        let high = high_ft > DRAG_HIGH_FT && over > -DRAG_SLOW_KTS / 2.0;
        vs_want_fpm <= -DRAG_MIN_DESCENT_FPM && (over > DRAG_OUT_KTS || high) && settled
    }
}

/// Minutes of the commanded rate inside which a VS mode that drops heading
/// for the dial is the autopilot's own altitude capture.
const CAPTURE_LEAD_MIN: f64 = 0.2;

/// Whether a VS mode found OFF is the autopilot capturing, to be left alone:
/// near the follower's target, or inside the capture band of the DIAL (the
/// level this movement ends at) while heading for it. EGKK-EGBB (2026-09-27,
/// sortie 1790541304-full_auto +2295 and +3001): the 737 began capturing
/// HON's 9,000 202 ft above it at 1,960 fpm down and FI33's 1,800 246 ft
/// above it at 1,500; the path target was 134 and 148 ft below, so the old
/// 100 ft test called it a dropped mode, re-engaged VS, and the dial write
/// read back 0 (`write_unconfirmed vertical_velocity want -1900 actual 0`),
/// one tick lost and the capture undone. Pure.
pub fn vs_mode_capturing(alt_ft: f64, want_alt: f64, dial_ft: f64, vs_cmd_fpm: f64) -> bool {
    if (want_alt - alt_ft).abs() <= 100.0 {
        return true;
    }
    let band = (vs_cmd_fpm.abs() * CAPTURE_LEAD_MIN).max(100.0);
    (dial_ft - alt_ft).abs() <= band && (dial_ft - alt_ft) * vs_cmd_fpm >= 0.0
}

/// Which speedbrake changes are SAID: an extension when nothing about the
/// brake was said for [`DRAG_SPEAK_GAP_S`], and the stow of an extension
/// that was said (never an orphan "Speedbrake in."). Every change is still
/// flown and blackboxed. EGKK-EGBB (2026-09-27, #126) said "Speedbrake." /
/// "Speedbrake in." thirteen times on one arrival, seven of them in 46 s.
/// Pure: `t` is seconds on any clock.
#[derive(Debug, Default, Clone)]
pub struct DragVoice {
    last_said: Option<f64>,
    out_said: bool,
}

impl DragVoice {
    /// Whether the change to `out` at `t` is spoken.
    pub fn say(&mut self, out: bool, t: f64) -> bool {
        let due = if out {
            let due = self.last_said.is_none_or(|l| t - l >= DRAG_SPEAK_GAP_S);
            self.out_said = due;
            due
        } else {
            std::mem::take(&mut self.out_said)
        };
        if due {
            self.last_said = Some(t);
        }
        due
    }
}

/// Why the speedbrake may not be out in this configuration, if it may not:
/// `"gear"` with the gear handle down, `"flaps"` with the flap handle past the
/// airframe's first detent (`flap_limits`, retracted first; any flap at all
/// when none is declared). A crew rule, not a speed rule: the drag is for a
/// clean descent, and the approach slows on flap and gear.
///
/// EGKK 26L, 2026-09-27 (sortie 1790510400-full_auto, stock 737): the
/// follower put the speedbrake out at 218 kt against a 210 target and
/// 2,000 fpm down, then configured for the final with it still out: gear
/// down and flaps 10, 25, 30, 40 before the speed let it in, and out again
/// for five seconds at flaps 40. The SPEEDBRAKE EXTENDED caution kept
/// tripping the master caution through the approach. `descent_drag`'s
/// shallower rate is what the descent keeps.
pub fn drag_barred(
    flap_handle: f64,
    gear_handle_down: bool,
    flap_limits: &[(f64, Option<f64>)],
) -> Option<&'static str> {
    if gear_handle_down {
        return Some("gear");
    }
    let first_detent = flap_limits
        .iter()
        .map(|(r, _)| *r)
        .filter(|r| r.is_finite() && *r > 0.01)
        .min_by(f64::total_cmp)
        .unwrap_or(0.0);
    if flap_handle > first_detent + 0.01 {
        return Some("flaps");
    }
    None
}

pub fn vertical_tick(
    leg: &ProcLeg,
    carry: f64,
    prev_cmd: f64,
    alt_ft: f64,
    dist_nm: Option<f64>,
    gs_kts: f64,
    pilot: Option<f64>,
    terrain_floor: Option<f64>,
    descent_fpm: f64,
    profile: DescentProfile,
) -> Vertical {
    let mut leg_alt = commanded_alt_ft(leg, carry);
    // In the descent a plan altitude above the level already commanded is
    // held at that level until a lower one takes over. The pilot's altitude
    // still wins (a controller may ask for a climb) and so does the ground:
    // both are applied after this, over it.
    let mut descent_held = None;
    if pilot.is_none() && profile.descending(prev_cmd) && leg_alt > prev_cmd + 50.0 {
        descent_held = Some((leg_alt, prev_cmd));
        leg_alt = prev_cmd;
    }
    // In the climb a plan altitude below the level already commanded (a
    // CA leg's "at or above", a navlog prediction) is held at that level:
    // a crew climbing to its cruise never descends for a minimum it passed.
    let mut climb_held = None;
    if pilot.is_none() && profile.climbing() && leg_alt < prev_cmd - 50.0 {
        climb_held = Some((leg_alt, prev_cmd));
        leg_alt = prev_cmd;
    }
    let (mut want_alt, mut target_fpm) =
        descent_target(leg_alt, prev_cmd, alt_ft, dist_nm, gs_kts, descent_fpm);
    // The cruise is reached by FLYING it: commanded from the first leg, it
    // made a climb through 7,600 ft look like a descent (LFBD 2026-09-27).
    let peak_ft = profile.peak_ft.max(alt_ft);
    let from_pilot = pilot.is_some();
    if let Some(p) = pilot {
        want_alt = p;
        target_fpm = 0.0;
    }
    let mut floored = None;
    if let Some(floor) = terrain_floor {
        if want_alt < floor {
            want_alt = floor;
            floored = Some(floor);
            target_fpm = 0.0;
        }
    }
    Vertical {
        want_alt,
        carry: leg_alt,
        prev_cmd: want_alt,
        floored,
        pilot: from_pilot && floored.is_none(),
        target_fpm,
        peak_ft,
        descent_held,
        climb_held,
    }
}

/// The leg to hand off to autoland on an approach: the MAP of an approach
/// that ends short of the runway, else the FAF if the data flags one,
/// else the last fix leg before the runway threshold, else the last leg. `None`
/// only when there are no legs. Handing off AT the FAF lets autoland own the
/// glideslope from platform to touchdown, exactly where it's proven.
pub fn handoff_index(legs: &[ProcLeg]) -> Option<usize> {
    if legs.is_empty() {
        return None;
    }
    // An approach to a MAP short of the runway is flown TO the MAP: its final
    // is the published valley track, and the runway's extended centreline
    // from the FAF lies across the slopes beside it (LSZS 03: two miles off,
    // toward Piz Corvatsch). A MAP coded ON the runway fix is not one: the
    // ILS finals in the CIFP carry the MAP flag on the threshold ("G  M"),
    // and handing off there gave autoland EGKK 26L at 1 nm, 244 m off the
    // centreline, and a go-around at 97 ft (2026-09-27).
    if legs.last().is_some_and(map_short_of_runway) {
        return Some(legs.len() - 1);
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
    let mut visual: Option<crate::nav::VisualSegment> = None;
    let mut missed_legs: Vec<ProcLeg> = Vec::new();
    let mut found: Option<(Vec<ProcLeg>, String)> = None;
    for appch in crate::nav::approach_candidates(cifp, icao, rwy_id) {
        let raw = crate::nav::load_proc_legs(cifp, icao, crate::nav::ProcKind::Appch, &appch, None);
        // As far as the threshold: the missed approach is never the approach
        // (`straight_in_legs`).
        if let Some(mut legs) = crate::nav::straight_in_legs(raw.clone(), rwy_id) {
            crate::nav::resolve_proc_fixes(fix_path, nav_path, airport, icao, &mut legs, lat, lon);
            found = Some((legs, format!("the {} approach runway {spoken}", approach_kind(&appch))));
            break;
        }
        // An approach to a missed approach point short of the runway (LSZS):
        // flown from its TRANSITION, the published way in over the terrain,
        // to the MAP on its own path, and landed from there on a visual
        // segment the landing can fly. Judged once the fixes are resolved.
        let Some((legs, missed)) =
            map_approach_legs(cifp, fix_path, nav_path, airport, icao, &appch, lat, lon)
        else {
            continue;
        };
        let Some(seg) = runway_end(airport, rwy_id).and_then(|(thr, course)| {
            crate::nav::visual_segment(&legs, thr, course, airport.elev_ft)
        }) else {
            continue;
        };
        crate::blackbox::record(
            "procedure",
            serde_json::json!({"event": "visual_segment", "approach": appch, "runway": rwy_id,
                "dist_nm": seg.dist_nm, "lateral_nm": seg.lateral_nm,
                "course_diff": seg.course_diff_deg, "angle": seg.angle_deg, "flyable": seg.flyable()}),
        );
        if seg.flyable() {
            visual = Some(seg);
            missed_legs = missed;
            found = Some((
                legs,
                format!("the {} approach runway {spoken}", approach_kind(&appch)),
            ));
            break;
        }
    }
    let (legs, name) = found.unwrap_or_default();
    // The angle handed to the landing is measured to ITS aim point (the
    // touchdown zone past the threshold), so the MAP sits on its path rather
    // than under it: LSZS 03, 4.74 to the threshold, 4.4 to the aim point.
    let visual_deg = visual.map(|v| {
        let height_ft = v.dist_nm * 6076.12 * v.angle_deg.to_radians().tan();
        (height_ft / ((v.dist_nm + crate::autoland::TOUCHDOWN_AIM_NM) * 6076.12))
            .atan()
            .to_degrees()
    });
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
    // definition; a reconstructed one is only as good as its fixes. A visual
    // segment is handed over at the MAP instead, judged above.
    if visual.is_none() {
        if let Some(why) = faf_off_course(&legs, airport, rwy_id) {
            return Err(why);
        }
    }
    // Approaches descend; the climb protection reference stays off unless the
    // caller sets one (e.g. the enroute follower).
    Ok(Plan {
        name,
        legs,
        end,
        vy_kts: 0.0,
        cruise_ias_kts: None, // the follower holds the clean approach speed
        transition_altitude_ft: None,
        climb_minimums: Vec::new(),
        arrival_constraints: Vec::new(),
        visual_deg,
        missed_legs,
        cruise_schedule: Default::default(),
    })
}

/// A MAP approach's legs from its best TRANSITION: the one whose first fix is
/// nearest the aircraft, straight legs only (a hold or a procedure turn is not
/// flown), joined to the final and cut at the MAP. Resolved. `None` when the
/// approach does not end at a MAP or no transition can be flown (the final's
/// IF alone could lie across the ridges the transition goes round: LSZS's
/// ZS701 is 39 nm from RONAG, over the Bernina).
#[allow(clippy::too_many_arguments)]
fn map_approach_legs(
    cifp: &str,
    fix_path: &str,
    nav_path: &str,
    airport: &crate::nav::AirportInfo,
    icao: &str,
    appch: &str,
    lat: f64,
    lon: f64,
) -> Option<(Vec<ProcLeg>, Vec<ProcLeg>)> {
    let flyable = |l: &ProcLeg| matches!(l.leg_type.as_str(), "IF" | "TF" | "CF" | "DF");
    let mut best: Option<(f64, Vec<ProcLeg>)> = None;
    for t in crate::nav::approach_transitions(cifp, icao, appch) {
        let raw = crate::nav::load_proc_legs(cifp, icao, crate::nav::ProcKind::Appch, appch, Some(&t));
        let Some(mut legs) = crate::nav::map_final_legs(raw) else { continue };
        if !legs.iter().all(flyable) {
            continue;
        }
        // The transition's last fix is the final's IF: flown once.
        legs.dedup_by(|b, a| !a.fix_ident.is_empty() && a.fix_ident == b.fix_ident);
        crate::nav::resolve_proc_fixes(fix_path, nav_path, airport, icao, &mut legs, lat, lon);
        let Some(first) = legs.iter().find_map(|l| l.fix.as_ref()) else { continue };
        let d = crate::nav::haversine_nm(lat, lon, first.lat, first.lon);
        if best.as_ref().is_none_or(|(bd, _)| d < *bd) {
            best = Some((d, legs));
        }
    }
    let (_, legs) = best?;
    // The published missed approach: the final's legs after the MAP, straight
    // legs only (the hold at its end is not flown; the follower levels at the
    // last constraint and hands over there).
    let all = crate::nav::load_proc_legs(cifp, icao, crate::nav::ProcKind::Appch, appch, None);
    let mut missed: Vec<ProcLeg> = all
        .iter()
        .position(|l| l.is_map)
        .map(|at| all[at + 1..].iter().filter(|l| flyable(l) && !l.fix_ident.is_empty()).cloned().collect())
        .unwrap_or_default();
    missed.dedup_by(|b, a| a.fix_ident == b.fix_ident);
    crate::nav::resolve_proc_fixes(fix_path, nav_path, airport, icao, &mut missed, lat, lon);
    Some((legs, missed))
}

/// The threshold of runway end `rwy_id` and its course, degrees true.
fn runway_end(airport: &crate::nav::AirportInfo, rwy_id: &str) -> Option<((f64, f64), f64)> {
    airport.runways.iter().find_map(|r| {
        if r.ident_a.eq_ignore_ascii_case(rwy_id) {
            Some(((r.lat_a, r.lon_a), r.heading_a_deg()))
        } else if r.ident_b.eq_ignore_ascii_case(rwy_id) {
            Some(((r.lat_b, r.lon_b), r.heading_b_deg()))
        } else {
            None
        }
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

/// Calibrated airspeed, kt, of `mach` at a pressure altitude (ISA,
/// troposphere and the isothermal layer above 36,089 ft). The governor holds
/// INDICATED speed, so a planned Mach is held as the airspeed it is at the
/// level flown: M0.628 at FL180 is 300 kt, M0.78 at FL360 is 260. Pure.
pub fn cas_for_mach(mach: f64, pressure_alt_ft: f64) -> f64 {
    const A0_KTS: f64 = 661.4786;
    let h = pressure_alt_ft.clamp(-2_000.0, 65_000.0);
    let delta = if h <= 36_089.0 {
        (1.0 - 6.875_59e-6 * h).powf(5.255_88)
    } else {
        0.223_36 * (-4.806_34e-5 * (h - 36_089.0)).exp()
    };
    let qc_over_p = (1.0 + 0.2 * mach * mach).powf(3.5) - 1.0;
    let qc_over_p0 = qc_over_p * delta;
    A0_KTS * (5.0 * ((qc_over_p0 + 1.0).powf(2.0 / 7.0) - 1.0)).sqrt()
}

/// A leg's planned cruise speed from the OFP: the airspeed and Mach the cost
/// index gave its fix, and what the governor should hold at `alt_ft`.
#[derive(Debug, Clone, PartialEq)]
pub struct ScheduledSpeed {
    pub fix: String,
    pub plan_ias_kts: Option<f64>,
    pub mach: Option<f64>,
    /// The lower of the planned airspeed and the planned Mach at the level
    /// flown: an airline's IAS/Mach schedule, so a level other than the
    /// plan's (a pilot's, a controller's) still stays on the Mach.
    pub target_kts: f64,
}

/// The fix a schedule entry must be within to be this leg's, nm: an ident is
/// not unique across the world, a position is.
const SCHEDULE_MATCH_NM: f64 = 2.0;

/// The OFP cruise speed for the leg flown, if the navlog planned it as a
/// CRUISE leg (its fix's `stage` is CRZ). Climb and descent legs are `None`:
/// the airframe's speed, the 250 below ten thousand and the STAR limits
/// keep them as before. Pure.
pub fn scheduled_cruise_kts(
    schedule: &crate::simbrief::CruiseSchedule,
    leg: &ProcLeg,
    alt_ft: f64,
) -> Option<ScheduledSpeed> {
    let e = schedule.fixes.iter().find(|e| {
        e.stage == crate::simbrief::Stage::Cruise
            && e.ident.eq_ignore_ascii_case(&leg.fix_ident)
            && leg
                .fix
                .as_ref()
                .is_some_and(|f| crate::nav::haversine_nm(f.lat, f.lon, e.lat, e.lon) <= SCHEDULE_MATCH_NM)
    })?;
    let by_mach = e
        .mach
        .filter(|_| alt_ft.is_finite())
        .map(|m| cas_for_mach(m, alt_ft));
    let target = match (e.ias_kts, by_mach) {
        (Some(a), Some(b)) => a.min(b),
        (a, b) => a.or(b)?,
    };
    Some(ScheduledSpeed {
        fix: e.ident.clone(),
        plan_ias_kts: e.ias_kts,
        mach: e.mach,
        target_kts: target.round(),
    })
}

/// Which cruise speed the governor holds on this leg, and why.
#[derive(Debug, Clone, PartialEq)]
pub struct CruisePick {
    pub kts: f64,
    /// "ofp" or "profile".
    pub source: &'static str,
    /// Why the OFP's speed was not taken: "below_floor", "above_ceiling".
    pub rejected: Option<&'static str>,
    /// The OFP's speed when it was taken down to [`cruise_cap_kts`].
    pub capped_from: Option<f64>,
}

/// Knots under the monitor's overspeed caution a cruise is scheduled at:
/// the governor's own wander (296 to 303 kt around a 299 target at FL180)
/// and a descent's first seconds must not reach the caution band.
pub const CRUISE_CAUTION_MARGIN_KTS: f64 = 10.0;

/// The highest cruise the follower schedules: the overspeed caution's
/// floor ([`crate::monitor::OVERSPEED_CAUTION_FACTOR`] of Vne) less
/// [`CRUISE_CAUTION_MARGIN_KTS`]. `vne_kts` is the airframe's; `None`
/// without one.
///
/// EGKK-EGBB, 2026-09-28 (sortie 1790601822-full_auto, #156): the OFP's
/// 299 and 300 KIAS at FL180 against the 737's 340 kt Vne (caution from
/// 306): the governor held 294.8 to 302.7, and the nose-down at the top of
/// descent (+2115.3, 305 kt with the trend) said "Airspeed, you're getting
/// fast." A crew flies the plan's speed, never into the caution. Pure.
pub fn cruise_cap_kts(vne_kts: Option<f64>) -> Option<f64> {
    vne_kts
        .filter(|v| *v > 0.0)
        .map(|v| (v * crate::monitor::OVERSPEED_CAUTION_FACTOR - CRUISE_CAUTION_MARGIN_KTS).floor())
}

/// The OFP's speed where the leg has one and it is inside the airframe's
/// floor and ceiling; the profile's `cruise_ias_kts` otherwise. A plan made
/// for another type, or a number outside what this airframe may fly, is
/// never flown: the airframe's own figure is (a wrong number handed down
/// from a plan is still wrong). A target over the ceiling would also be
/// clamped by the governor every tick and re-armed every tick. Pure.
pub fn cruise_pick(
    profile_kts: f64,
    scheduled: Option<&ScheduledSpeed>,
    floor_kts: Option<f64>,
    ceiling_kts: Option<f64>,
    cap_kts: Option<f64>,
) -> CruisePick {
    let profile = |rejected| CruisePick {
        kts: profile_kts,
        source: "profile",
        rejected,
        capped_from: None,
    };
    let Some(s) = scheduled else {
        return profile(None);
    };
    if floor_kts.is_some_and(|f| s.target_kts < f) {
        return profile(Some("below_floor"));
    }
    if ceiling_kts.is_some_and(|c| s.target_kts > c) {
        return profile(Some("above_ceiling"));
    }
    // Under the overspeed caution ([`cruise_cap_kts`]): the plan's speed,
    // as near it as the caution allows (never under the floor).
    match cap_kts.filter(|c| s.target_kts > *c && floor_kts.is_none_or(|f| *c >= f)) {
        Some(c) => CruisePick {
            kts: c,
            source: "ofp",
            rejected: None,
            capped_from: Some(s.target_kts),
        },
        None => CruisePick {
            kts: s.target_kts,
            source: "ofp",
            rejected: None,
            capped_from: None,
        },
    }
}

/// A planned step placed on the legs: from the leg after `idx` (the leg
/// flown FROM the step fix) the plan's level is `alt_ft`.
#[derive(Debug, Clone, PartialEq)]
pub struct PlacedStep {
    pub fix: String,
    pub idx: usize,
    pub alt_ft: f64,
}

/// The plan's step climbs, flown where the OFP planned them. `enroute_legs`
/// commands the plan's HIGHEST level from the first leg (one continuous climb
/// to cruise), which for a stepped plan is the LAST step: FL340 then FL360
/// at ERNAN was flown as FL360 from the runway. With the OFP's
/// `stepclimb_string` read, every climb and cruise leg (before the first
/// DESCENT-stage fix) commands the level in force: the initial level up to
/// and including the first step fix, the step's level after it, and so on.
/// The descent keeps the navlog's own numbers. Returns the steps placed. A
/// plan with one level, no stages, an implausible initial level or a step
/// fix not on the route (before the descent) is left exactly as built. Pure.
pub fn apply_step_climbs(
    legs: &mut [ProcLeg],
    schedule: &crate::simbrief::CruiseSchedule,
) -> Vec<PlacedStep> {
    let Some((first, rest)) = schedule.steps.split_first() else {
        return Vec::new();
    };
    if rest.is_empty() || !(1_000.0..=60_000.0).contains(&first.alt_ft) {
        return Vec::new();
    }
    let stage_of = |leg: &ProcLeg| {
        schedule
            .fixes
            .iter()
            .find(|e| {
                e.ident.eq_ignore_ascii_case(&leg.fix_ident)
                    && leg.fix.as_ref().is_some_and(|f| {
                        crate::nav::haversine_nm(f.lat, f.lon, e.lat, e.lon) <= SCHEDULE_MATCH_NM
                    })
            })
            .map(|e| e.stage)
    };
    let Some(top) = legs
        .iter()
        .position(|l| stage_of(l) == Some(crate::simbrief::Stage::Descent))
        .or_else(|| legs.iter().any(|l| stage_of(l).is_some()).then_some(legs.len()))
    else {
        return Vec::new();
    };
    let mut placed = Vec::new();
    let mut from = 0usize;
    for s in rest {
        let Some(j) = legs[..top]
            .iter()
            .enumerate()
            .skip(from)
            .find(|(_, l)| l.fix_ident.eq_ignore_ascii_case(&s.fix))
            .map(|(j, _)| j)
        else {
            // A step the route does not carry: the plan cannot be flown as
            // stepped, so none of it is (the navlog's levels stand).
            return Vec::new();
        };
        placed.push(PlacedStep { fix: s.fix.clone(), idx: j, alt_ft: s.alt_ft });
        from = j + 1;
    }
    for (i, leg) in legs[..top].iter_mut().enumerate() {
        let level = placed
            .iter()
            .rev()
            .find(|p| p.idx < i)
            .map_or(first.alt_ft, |p| p.alt_ft);
        leg.alt1_ft = Some(level);
    }
    placed
}

/// The step climb that begins on leg `idx`: the leg before it ends at a
/// planned step fix and this leg's level is a step above that leg's. Pure.
pub fn step_climb_at(
    legs: &[ProcLeg],
    idx: usize,
    schedule: &crate::simbrief::CruiseSchedule,
) -> Option<(String, f64, f64)> {
    let prev = legs.get(idx.checked_sub(1)?)?;
    let (from, to) = (prev.alt1_ft?, legs.get(idx)?.alt1_ft?);
    (to > from + STEP_FT
        && schedule
            .steps
            .iter()
            .skip(1)
            .any(|s| s.fix.eq_ignore_ascii_case(&prev.fix_ident)))
    .then(|| (prev.fix_ident.clone(), from, to))
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
            alt1_ft: Some(
                if i < first_at_peak || (peak - *alt).abs() <= CRUISE_DIP_FT {
                    peak
                } else {
                    *alt
                },
            ),
            alt2_ft: None,
            is_faf: false,
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
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
pub fn start_scoped(
    base_url: String,
    voice: Option<Voice>,
    plan: Plan,
    parents: Vec<std::sync::Arc<AtomicBool>>,
) -> bool {
    let Some(lease) = RUN.begin() else {
        return false;
    };
    if ACTIVE.swap(true, Ordering::SeqCst) {
        return false;
    }
    ABORT.store(false, Ordering::SeqCst);
    std::thread::spawn(move || {
        let _lease = lease.with_cleanup(&ACTIVE, || {
            publish_leg(None);
            publish_tod(None);
            publish_fix_dist(None);
            clear_pilot_altitude();
            publish_terrain_floor(None);
            publish_route_ahead(None);
            take_splice();
            crate::tools::speedhold::release();
            hold_at_cleared(None, "follower_ended");
        });
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            ACTIVE.store(false, Ordering::SeqCst);
            return;
        };
        rt.block_on(async {
            // A second client for the one thing owed on EVERY exit: a
            // speedbrake this follower put out goes back in, whoever ended
            // the flight and however (uncancelled on purpose).
            let stow = XPlaneClient::new(base_url.clone());
            fly(
                XPlaneClient::new(base_url.clone())
                    .with_cancel(_lease.cancel.clone())
                    .with_parents(parents),
                base_url,
                voice,
                plan,
            )
            .await;
            if DRAG_OURS.swap(false, Ordering::SeqCst) {
                let _ = stow.set_float(SPEEDBRAKE, 0.0).await;
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "speedbrake", "out": false, "why": "follower_ended"}),
                );
            }
        });
        ACTIVE.store(false, Ordering::SeqCst);
        publish_leg(None);
        publish_tod(None);
        publish_fix_dist(None);
    });
    true
}

fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    tracing::info!(target: "xvoicelayer::procedure", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

fn stand_down(voice: &Option<Voice>, line: &str, reason: &str) {
    RUN.record(if reason == "abort" {
        sim::run::Outcome::Cancelled
    } else {
        sim::run::Outcome::Failed(reason.into())
    });
    ACTIVE.store(false, Ordering::SeqCst);
    clear_pilot_altitude();
    hold_at_cleared(None, "stand_down");
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

/// How the follower handles the heading bug: the way a pilot does. It was
/// rewritten every tick (bearing to the fix plus a live crab, stepped 25
/// degrees ahead of the nose through every turn), and the pilot watched it
/// twitch for the whole flight (LIPR 2026-09-26): "if we want to go a certain
/// heading, just set the heading". So: a leg's heading is SET once, a whole
/// degree, wind correction included; it is touched again only when the
/// track has drifted enough to need it, wings level, not more than twice a
/// minute; and a new heading on the far side of a turn still in progress
/// waits for the wings to come level (the autopilot turns the short way
/// from where the nose is, and a bug set against the bank went round the
/// long way, LXGB 2026-09-14).
#[derive(Debug, Default)]
pub struct HeadingHold {
    leg: Option<usize>,
    set: Option<f64>,
    last_write_s: f64,
}

/// A correction smaller than this is not worth reaching for the knob.
const BUG_CORRECT_DEG: f64 = 5.0;
/// And not more often than this.
const BUG_CORRECT_EVERY_S: f64 = 30.0;
/// Wings this level count as level for a correction.
const BUG_LEVEL_BANK_DEG: f64 = 5.0;
/// Banked past this the other way, a new heading waits for the roll-out.
const BUG_WAIT_BANK_DEG: f64 = 10.0;

impl HeadingHold {
    /// The heading to set now, or `None` to leave the knob alone. Pure.
    pub fn decide(&mut self, leg: usize, want_mag: f64, heading_mag: f64, bank_deg: f64, now_s: f64) -> Option<f64> {
        let want = want_mag.round().rem_euclid(360.0);
        let diff = |a: f64, b: f64| ((a - b + 540.0).rem_euclid(360.0)) - 180.0;
        let turn = diff(want, heading_mag);
        let against_bank = bank_deg.abs() > BUG_WAIT_BANK_DEG
            && turn.abs() > 10.0
            && (turn > 0.0) != (bank_deg > 0.0);
        let write = if self.leg != Some(leg) || self.set.is_none() {
            !against_bank
        } else {
            let off = diff(want, self.set.unwrap_or(want)).abs();
            off >= BUG_CORRECT_DEG
                && bank_deg.abs() < BUG_LEVEL_BANK_DEG
                && now_s - self.last_write_s >= BUG_CORRECT_EVERY_S
        };
        if !write {
            return None;
        }
        self.leg = Some(leg);
        self.set = Some(want);
        self.last_write_s = now_s;
        Some(want)
    }

    /// Forget the leg, so the next tick sets its heading afresh (a rejoin).
    pub fn reset(&mut self) {
        self.leg = None;
        self.set = None;
    }
}

async fn fly(xp: XPlaneClient, base_url: String, voice: Option<Voice>, plan: Plan) {
    let Plan {
        name,
        mut legs,
        end,
        vy_kts,
        cruise_ias_kts,
        transition_altitude_ft,
        climb_minimums,
        mut arrival_constraints,
        visual_deg,
        missed_legs,
        cruise_schedule,
    } = plan;
    // Never judge the aircraft against a minimum above the level it is
    // flying to (`minimums_within_cruise`); the arm says so, and this
    // covers every other way into the follower.
    let cruise_ft = legs.iter().filter_map(|l| l.alt1_ft).fold(f64::MIN, f64::max);
    let (climb_minimums, above) = if cruise_ft > 0.0 {
        minimums_within_cruise(climb_minimums, cruise_ft)
    } else {
        (climb_minimums, Vec::new())
    };
    for m in &above {
        crate::blackbox::record(
            "procedure",
            serde_json::json!({"event": "minimum_above_cruise", "fix": m.fix, "min_ft": m.min_ft, "cruise_ft": cruise_ft}),
        );
    }
    let mut handoff = handoff_index(&legs);
    let _detail = crate::auto_status::DetailGuard::hold(&DETAIL);
    // Process-wide facts must never outlive the flight that set them.
    clear_pilot_altitude();
    publish_terrain_floor(None);
    // The altimeter first: every altitude below is read off it. Said only
    // when the knob actually moved (a correct setting earns no chatter).
    if let Some(ta) = transition_altitude_ft {
        if let Some(sync) = crate::tools::instruments::sync_altimeter(&xp, ta).await {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "altimeter_set", "was": sync.was, "now": sync.now, "want": sync.want, "took": sync.took()}),
            );
            // A set that did not take is never quiet: every altitude below
            // is wrong by the difference (EGSS 2026-09-22, 12,500 ft).
            if !sync.took() {
                alert(&voice, Priority::Warning, &sync.line());
            } else if sync.changed() {
                alert(&voice, Priority::Advisory, &sync.line());
            }
        }
    }
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "armed", "name": name, "legs": legs.len(), "approach": matches!(end, EndAction::Approach{..})}),
    );
    alert(&voice, Priority::Advisory, &format!("Flying {name}."));

    // Seed from real instruments BEFORE engaging anything. An unconstrained
    // first leg holds indicated altitude, just like subsequent legs.
    let (var, psi, alt) = tokio::join!(
        xp.get_float(MAG_VAR),
        xp.get_float(PSI_TRUE),
        xp.get_float(ALT_IND_FT),
    );
    let Some([mut mag_var, psi_seed, mut carry_alt]) = measurements("procedure", [var, psi, alt])
    else {
        stand_down(
            &voice,
            "Flight instruments unavailable. Your aircraft.",
            "seed_unread",
        );
        return;
    };
    let mut bug_mag = true_to_mag(psi_seed, mag_var);
    let mut heading_hold = HeadingHold::default();
    let started = std::time::Instant::now();
    if !set_confirmed("procedure", &xp, HDG_BUG, bug_mag, &mut None, 0.0).await {
        stand_down(
            &voice,
            "Heading selection failed. Your aircraft.",
            "seed_unconfirmed",
        );
        return;
    }
    let _ = xp.run_command(CMD_SERVOS_ON).await;
    if xp.get_float(HDG_STATUS).await.is_ok_and(|v| v <= 0.5) {
        let _ = xp.run_command(CMD_HDG_MODE).await;
    }
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
    let mut prev_ias: Option<(f64, std::time::Instant)> = None;
    let mut ias_trend_f = 0.0_f64;
    // A descent flown at idle, the speed on the pitch (`IdleDescent`).
    let mut idle_descent = IdleDescent::default();
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
        // No altitude here to cap against — the first tick below has one and
        // corrects it within a fifth of a second, which is why the cap lives
        // there and not in this arm.
        crate::tools::speedhold::arm_flight(base_url.clone(), voice.clone(), kts);
        let event = if matches!(end, EndAction::Enroute) {
            "cruise_power"
        } else {
            "approach_power"
        };
        crate::blackbox::record(
            "procedure",
            serde_json::json!({"event": event, "target_ias": kts}),
        );
    }

    let mut said_star_speed: std::collections::HashSet<String> = std::collections::HashSet::new();
    // The OFP schedule's last logged pick (leg, source, why rejected), and
    // the planned steps already said.
    let mut schedule_logged: Option<(usize, &'static str, Option<&'static str>)> = None;
    let mut said_steps: std::collections::HashSet<String> = std::collections::HashSet::new();
    // The STAR speed limits whose slow-down has begun: held from then on
    // (`arrival_speed_cap`).
    let mut star_speed_begun: std::collections::HashSet<usize> = std::collections::HashSet::new();
    // The arrival's hand-over speed, an enroute plan's only (#199), whether
    // its slow-down has begun (held from then on) and was said.
    let arrival_handoff_kts =
        take_arrival_handoff_kts().filter(|_| matches!(end, EndAction::Enroute));
    let mut handoff_begun = false;
    let mut said_handoff = false;
    // The speedbrake's last change and which changes are said (`descent_drag`,
    // `DragVoice`).
    let mut drag_changed: Option<std::time::Instant> = None;
    let mut drag_voice = DragVoice::default();
    let drag_clock = std::time::Instant::now();
    let mut star_ceiling_logged: Option<usize> = None;
    // The lowest ceiling seen on the leg flying (`ceilinged_leg_alt`).
    let mut ceiling_latch: Option<(usize, f64)> = None;
    // The one descent a pilot-held altitude is reminded about: (the pilot's
    // altitude, the fix). The plan's later descents under the same held
    // altitude are silent; a new altitude from the pilot is a new hold.
    let mut held_leg: Option<(f64, String)> = None;
    // The plan's profile, for the rule that a descent never climbs back to a
    // navlog altitude above it (`DescentProfile`). Off on an approach: its
    // own procedure owns the heights.
    let descent_cruise_ft = (cruise_ft > 0.0 && !matches!(end, EndAction::Approach { .. }))
        .then_some(cruise_ft);
    let mut plan_top = descent_cruise_ft.and_then(|c| plan_top_index(&legs, c));
    let mut plan_peak_ft = prev_cmd_alt;
    let mut descent_held_logged: Option<usize> = None;
    let mut climb_held_logged: Option<usize> = None;
    if !arrival_constraints.is_empty() {
        crate::blackbox::record(
            "procedure",
            serde_json::json!({"event": "star_constraints", "constraints": arrival_constraints.iter().map(|c|
                serde_json::json!({"fix": c.fix, "min_ft": c.min_ft, "max_ft": c.max_ft, "max_kts": c.max_kts})).collect::<Vec<_>>()}),
        );
    }
    // The STAR's windows the plan does not make, said once each: over the
    // cruise at the arm (and a splice), and any the airframe cannot reach
    // from where it is as it flies (#32, #156).
    let mut said_windows: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut said_unable_windows: std::collections::HashSet<String> = std::collections::HashSet::new();
    let star_cruise_ft = (cruise_ft > 0.0).then_some(cruise_ft);
    let say_above_cruise = |legs: &[ProcLeg], cons: &[crate::nav::StarConstraint], said: &mut std::collections::HashSet<String>| {
        let Some(cruise) = star_cruise_ft else { return };
        for (fix, min_ft, cruise_ft) in windows_above_cruise(legs, cons, cruise) {
            if said.insert(fix.to_ascii_uppercase()) {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "star_window_above_cruise", "fix": fix, "min_ft": min_ft, "cruise_ft": cruise_ft}),
                );
                alert(&voice, Priority::Advisory, &above_cruise_line(&fix, min_ft, cruise_ft));
            }
        }
    };
    say_above_cruise(&legs, &arrival_constraints, &mut said_windows);
    let mut idx = 0usize;
    // The join onto an approach's final from its IF, and how the last tick
    // steered (`crate::course_leg`, #106).
    let mut final_join = crate::course_leg::FinalJoin::default();
    // The hold being flown, by leg index, and the last hold leg seen (#101).
    let mut hold: Option<(usize, crate::course_leg::HoldFlight)> = None;
    let mut hold_seen: Option<usize> = None;
    let mut steer_was = crate::course_leg::Steer::Direct;
    let mut closure: Option<(usize, crate::circuit::Closure)> = None;
    let mut stalls = 0u32;
    let mut progress_s = 0.0;
    let mut tick_clock = std::time::Instant::now();
    // The FMS is sequenced once per leg, when the follower first steers at it.
    let mut fms_leg: Option<usize> = None;
    // The leg index last published with the route (`route_ahead`).
    let mut route_published: Option<usize> = None;
    // Terrain look-ahead: probed on its own slow cadence (the ground does not
    // move), disabled for the flight the first time the plugin has no answer so
    // an old build degrades to the previous behaviour instead of stalling the
    // tick for two seconds every ten.
    let mut last_probe = std::time::Instant::now() - std::time::Duration::from_secs(60);
    let mut terrain_available = true;
    let mut terrain_misses: u32 = 0;
    let mut said_floor: Option<f64> = None;
    let mut said_alt_hold = false;
    let mut said_short: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut gear_set = false;
    let mut last_flap: Option<f64> = None;
    let mut prev_alt_sample: Option<(f64, std::time::Instant)> = None;
    let mut climb_rate_fpm = 0.0_f64;
    // The autopilot's answer to the VS wheel, watched (#181).
    let mut climb_watch = crate::climb_watch::ClimbWatch::default();
    let mut last_verdict = crate::climb_watch::Verdict::Flying;
    let watch_t0 = std::time::Instant::now();
    let mut rate_samples = 0u32;
    let mut said_unable: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut said_warned_unable = 0usize;
    // The departure minimums met so far and the floors ahead
    // (`DepartureMinimums`), the minimums said as held under the clearance,
    // and the clearance's altitude while it was holding the target down.
    let mut minimums_state = DepartureMinimums::default();
    // The 250 below 10,000 cap, once begun held through a level near the line.
    let mut airspace = crate::tools::speedhold::AirspaceLimit::default();
    let mut said_min_capped: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut cap_holding: Option<f64> = None;
    // The VS wheel's last action logged (`vs_action`), so a change is one record.
    let mut vs_action_logged: Option<VsAction> = None;
    while idx < legs.len() {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if xp.cancelled() || ABORT.load(Ordering::SeqCst) {
            stand_down(&voice, "Cancelled. Your aircraft.", "abort");
            return;
        }
        // A frozen sim (pause/replay) suspends the sequence; the AP holds.
        if crate::xplane::sim_frozen(&xp).await {
            tick_clock = std::time::Instant::now();
            continue;
        }
        // An arrival re-picked in flight (#79): the new STAR's legs from its
        // entry, and its windows, taken whole between two ticks.
        if let Some(s) = take_splice() {
            match apply_splice(&legs, idx, &s) {
                Ok(new) => {
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "star_spliced", "idx": idx, "from": s.from, "entry": s.entry,
                            "why": s.why, "fixes": new[s.from..].iter().map(|l| l.fix_ident.clone()).collect::<Vec<_>>(),
                            "alts": new[s.from..].iter().map(|l| l.alt1_ft.map(f64::round)).collect::<Vec<_>>(),
                            "constraints": s.constraints.len()}),
                    );
                    legs = new;
                    arrival_constraints = s.constraints;
                    say_above_cruise(&legs, &arrival_constraints, &mut said_windows);
                    handoff = handoff_index(&legs);
                    plan_top = descent_cruise_ft.and_then(|c| plan_top_index(&legs, c));
                    star_speed_begun.clear();
                    said_star_speed.clear();
                    star_ceiling_logged = None;
                    ceiling_latch = None;
                    fms_leg = None;
                    route_published = None;
                }
                Err(why) => crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "splice_stale", "idx": idx, "from": s.from, "entry": s.entry, "why": why}),
                ),
            }
        }
        if route_published != Some(idx) {
            route_published = Some(idx);
            publish_route_ahead(Some((idx, legs.clone())));
        }

        let (
            lat,
            lon,
            elev_m,
            alt_ind,
            psi_r,
            hpath_r,
            agl_m,
            vs_st,
            hdg_st,
            ias_r,
            gs_r,
            phi_r,
            var_r,
        ) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(ELEV_M),
            xp.get_float(ALT_IND_FT),
            xp.get_float(PSI_TRUE),
            xp.get_float(HPATH),
            xp.get_float(AGL_M),
            xp.get_float(VS_STATUS),
            xp.get_float(HDG_STATUS),
            xp.get_float(IAS),
            xp.get_float(GS_MS),
            xp.get_float(crate::datarefs::BANK_DEG),
            xp.get_float(MAG_VAR),
        );
        let now = std::time::Instant::now();
        let dt = now.duration_since(tick_clock).as_secs_f64();
        tick_clock = now;
        let Some([lat, lon, elev_m, alt_ft, psi, agl_m, vs_st, hdg_st, ias, gs, phi]) =
            measurements(
                "procedure",
                [
                    lat, lon, elev_m, alt_ind, psi_r, agl_m, vs_st, hdg_st, ias_r, gs_r, phi_r,
                ],
            )
        else {
            continue;
        };
        progress_s += dt;
        let true_alt_ft = elev_m * M_TO_FT;
        let agl_ft = agl_m * M_TO_FT;
        // The magnetic variation HERE, not at the departure airport. It is a
        // fact of the PLACE, and this loop crosses enough of the world to
        // change it: read once at Keflavik and carried to Scotland, it was
        // 11.4 degrees stale, so every heading the follower wrote was 11.4
        // degrees off the nose. The roll-out capture that is supposed to end
        // a turn ("bug on the nose") instead commanded a standing 11 degree
        // error, the AP held 25 degrees of bank for 95 seconds and went round
        // 160 degrees, and the arrival's intercept could not close either
        // (BIKF-EGPH, 2026-09-20). A stale reading kept, rather than the
        // tick skipped: a lost read is no reason to throw away the last good
        // variation, which is right to a fraction of a degree a minute later.
        if let Ok(v) = var_r {
            if v.is_finite() {
                mag_var = v;
            }
        }
        // The 250 below 10,000 limit, applied as we CLIMB THROUGH it rather
        // than once at the arm. The cruise target is armed at a thousand feet
        // on departure and would otherwise be held all the way up — which is
        // exactly how a 255 kt cruise speed became a speed bust for the whole
        // climb (2026-09-20). Re-armed only when the cap actually changes;
        // `arm_flight` is idempotent and a running hold just takes the new
        // number, so this costs nothing per tick.
        let unmet = minimums_state.unmet(&climb_minimums, &legs, idx, alt_ft);
        // An unmet minimum above an IFR clearance's altitude is held under
        // the clearance until departure clears higher: not a climb to judge
        // (a watch that sees no climb would hand the aircraft back).
        let under_clearance = match (unmet, cleared_ceiling()) {
            (Some((_, m)), Some(c)) => pilot_altitude().is_none() && m > c + MINIMUM_MET_TOL_FT,
            _ => false,
        };
        // The climb rate, from the altimeter itself (smoothed): what the
        // minimum watch predicts with.
        if let Some((a0, t0)) = prev_alt_sample {
            let span = now.duration_since(t0).as_secs_f64();
            if (0.3..=5.0).contains(&span) {
                let inst = (alt_ft - a0) / span * 60.0;
                // Seeded from the first real sample, never from zero: the
                // watch judged its first tick at a zero climb rate, predicted
                // arriving at WI505 at the present altitude and handed the
                // aircraft back one second after the takeoff did, at 3,000 ft
                // in the valley (LOWI, 2026-09-24).
                climb_rate_fpm = if rate_samples == 0 { inst } else { 0.7 * climb_rate_fpm + 0.3 * inst };
                rate_samples += 1;
            }
        }
        prev_alt_sample = Some((alt_ft, now));
        // No judgement until the rate is a measurement (several seconds).
        let unmet_judged = unmet.filter(|_| rate_samples >= MIN_RATE_SAMPLES && !under_clearance);
        if let Some((at, min_ft)) = unmet_judged {
            let fix = legs[at].fix_ident.clone();
            let dist = distance_to_leg_nm(lat, lon, &legs, idx, at);
            match watch_minimum(alt_ft, climb_rate_fpm, gs * MS_TO_KTS, dist, min_ft) {
                MinimumWatch::OnTrack => {}
                MinimumWatch::Short { predicted_ft } => {
                    if said_short.insert(fix.clone()) {
                        crate::blackbox::record(
                            "procedure",
                            serde_json::json!({"event": "minimum_short", "fix": fix,
                                "min_ft": min_ft, "predicted_ft": predicted_ft, "dist_nm": dist}),
                        );
                        alert(
                            &voice,
                            Priority::Caution,
                            &format!(
                                "We'll cross {fix} about {:.0} feet under its {} minimum. Climbing at best rate.",
                                ((min_ft - predicted_ft) / 50.0).round() * 50.0,
                                crate::flightplan::spoken_feet(min_ft)
                            ),
                        );
                    }
                }
                MinimumWatch::Unable { predicted_ft } => {
                    // A miss predicted while CLIMBING is flown on, at best
                    // rate: that is the most anyone could do, and a hand-over
                    // near the ground mid-departure is the worst moment for
                    // it (LOWI, 2026-09-24). Said, once per fix. Only an
                    // aircraft that is not climbing at all is handed back.
                    let climbing = climb_rate_fpm >= NOT_CLIMBING_FPM;
                    if said_unable.insert(fix.clone()) || !climbing {
                        crate::blackbox::record(
                            "procedure",
                            serde_json::json!({"event": "minimum_unable", "fix": fix,
                                "min_ft": min_ft, "predicted_ft": predicted_ft, "alt_ft": alt_ft,
                                "dist_nm": dist, "rate_fpm": climb_rate_fpm, "climbing": climbing}),
                        );
                    }
                    if !climbing {
                        stand_down(
                            &voice,
                            &format!(
                                "We're not climbing and {fix} needs {}. Your aircraft. Climb now.",
                                crate::flightplan::spoken_feet(min_ft),
                            ),
                            "minimum_unable",
                        );
                        return;
                    }
                    if said_unable.len() > said_warned_unable {
                        said_warned_unable = said_unable.len();
                        alert(
                            &voice,
                            Priority::Warning,
                            &format!(
                                "We won't make {} at {fix}, about {:.0} feet under. Climbing at best rate.",
                                crate::flightplan::spoken_feet(min_ft),
                                ((min_ft - predicted_ft) / 50.0).round() * 50.0,
                            ),
                        );
                    }
                }
            }
        }
        // The speed owed this tick (the governor's target outside an idle
        // descent), for the vertical's speed protection below.
        let mut speed_owed: Option<f64> = None;
        if let Some(planned) = power_hold_target(&end, cruise_ias_kts) {
            // The OFP's cruise schedule on a cruise leg of an enroute flight:
            // the speed its cost index planned, held as IAS or Mach,
            // whichever is lower at the level flown (#101).
            let planned = if matches!(end, EndAction::Enroute) {
                let sched = scheduled_cruise_kts(&cruise_schedule, &legs[idx], alt_ft);
                let ceiling = crate::aircraft::speed_ceiling_kts();
                let pick = cruise_pick(
                    planned,
                    sched.as_ref(),
                    crate::aircraft::speed_floor_kts(),
                    ceiling,
                    // Vne from the published ceiling (Vne less its margin).
                    cruise_cap_kts(ceiling.map(|c| c + crate::aircraft::SPEED_CEILING_MARGIN_KTS)),
                );
                let key = (idx, pick.source, pick.rejected);
                if sched.is_some() && schedule_logged != Some(key) {
                    schedule_logged = Some(key);
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({
                            "event": "cruise_schedule", "idx": idx, "fix": legs[idx].fix_ident,
                            "source": pick.source, "target": pick.kts, "rejected": pick.rejected,
                            "capped_from": pick.capped_from,
                            "plan_ias": sched.as_ref().and_then(|s| s.plan_ias_kts),
                            "mach": sched.as_ref().and_then(|s| s.mach),
                            "ofp_target": sched.as_ref().map(|s| s.target_kts),
                            "profile_kts": planned, "alt_ft": alt_ft, "ias": ias,
                        }),
                    );
                }
                pick.kts
            } else {
                planned
            };
            // A departure minimum ahead is climbed at Vy, the steepest the
            // airframe goes up: at cruise speed the SF50 made 375 ft a mile
            // where MOGT3H asks 600 (LOWI, 2026-09-24).
            let planned = if unmet.is_some() && vy_kts > 0.0 { vy_kts } else { planned };
            // The pilot's speed outranks the plan's, as their altitude does.
            let planned = pilot_speed().unwrap_or(planned);
            // The STAR's speed limits: slowed in time for each, held after.
            let star_cap = arrival_speed_cap(&legs, &arrival_constraints, idx, lat, lon, ias, gs * MS_TO_KTS, &mut star_speed_begun);
            // Flown a little under the limit (`STAR_SPEED_MARGIN_KTS`): a
            // maximum is not a target.
            let planned = match &star_cap {
                Some((kts, fix)) if *kts - STAR_SPEED_MARGIN_KTS < planned => {
                    if said_star_speed.insert(fix.clone()) {
                        crate::blackbox::record(
                            "procedure",
                            serde_json::json!({"event": "star_speed", "fix": fix, "kts": kts, "ias": ias}),
                        );
                        alert(
                            &voice,
                            Priority::Advisory,
                            &format!("Slowing to {kts:.0} knots for {fix}."),
                        );
                    }
                    *kts - STAR_SPEED_MARGIN_KTS
                }
                _ => planned,
            };
            // The speed the arrival hands to autoland at the end of the
            // route, slowed for in time like a STAR limit (#199): EGBB-EGNX
            // (2026-09-30) held its 250 to the last fix and gave autoland a
            // clean 737 at 252 kt 8.4 nm out on the path.
            let planned = match arrival_handoff_kts {
                Some(kts) if kts < planned && !legs.is_empty() => {
                    let to_end = distance_to_leg_nm(lat, lon, &legs, idx, legs.len() - 1);
                    match arrival_handoff_cap(to_end, ias, gs * MS_TO_KTS, kts, &mut handoff_begun) {
                        Some(cap) => {
                            if !said_handoff {
                                said_handoff = true;
                                crate::blackbox::record(
                                    "procedure",
                                    serde_json::json!({"event": "arrival_slowdown", "kts": cap,
                                        "to_end_nm": to_end, "ias": ias, "planned": planned}),
                                );
                                alert(
                                    &voice,
                                    Priority::Advisory,
                                    &format!("Slowing to {cap:.0} knots for the approach."),
                                );
                            }
                            cap
                        }
                        None => planned,
                    }
                }
                _ => planned,
            };
            // On the final to a MAP: landing speed and configuration, so the
            // visual segment is flown stabilised (a jet cannot slow down and
            // go down 4.7 degrees in two miles).
            let on_map_final = legs[idx].is_map;
            let planned = match (&end, on_map_final) {
                (EndAction::Approach { approach_ias, .. }, true) => *approach_ias,
                _ => planned,
            };
            if on_map_final {
                if !gear_set && crate::autoland::gear_may_extend(ias, crate::aircraft::gear_limit_kts()) {
                    gear_set = xp.set_float(GEAR_HANDLE, 1.0).await.is_ok();
                }
                // A MAP ON the runway is an ordinary final: flap as the
                // approach goes (half inside 6 nm, full a mile before the 1,000 ft gate on the
                // path, `autoland::flap_schedule`), as a pilot flies an ILS.
                // Full flap from the FAF flew LIPR's 9 nm final at 65 kt for
                // eight minutes (2026-09-26). A MAP short of the runway keeps
                // the whole landing configuration for its visual segment.
                let wanted = if map_short_of_runway(&legs[idx]) {
                    1.0
                } else {
                    let d = legs[idx]
                        .fix
                        .as_ref()
                        .map(|f| crate::nav::haversine_nm(lat, lon, f.lat, f.lon));
                    let err = map_path(&legs[idx], d, gs * MS_TO_KTS).map_or(0.0, |(p, _)| alt_ft - p);
                    let by_distance = crate::autoland::flap_schedule(d.unwrap_or(9.9), err);
                    // And FLAPS ON SPEED: slowing toward the approach speed
                    // under the clean floor with the flaps up is a stall on
                    // a jet. By distance alone the 737 went from 217 to 150
                    // kt clean from 12 nm out and earned "Airspeed! Lower the
                    // nose" at LEMG 13 (2026-09-26). Near the clean floor,
                    // every detent the placards allow (`flap_within_limits`
                    // below steps them out as the speed comes down).
                    let near_clean_floor = crate::aircraft::speed_floor_kts()
                        .is_some_and(|f| ias < f + FLAPS_ON_SPEED_MARGIN_KTS);
                    if near_clean_floor { 1.0 } else { by_distance }
                };
                let flap = crate::autoland::flap_within_limits(wanted, ias, &crate::aircraft::flap_limits());
                if last_flap.is_none_or(|f: f64| (f - flap).abs() > 0.01)
                    && xp.set_float(FLAP_RATIO, flap).await.is_ok()
                {
                    last_flap = Some(flap);
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "map_config", "flap": flap, "gear": gear_set, "ias": ias}),
                    );
                }
            }
            // Floored HERE, against the floor for the configuration flown
            // (landing on the final to a MAP, clean otherwise), and compared
            // floored: comparing the raw 87 with the governor's floored 96
            // re-armed, and re-said the floor, every tick (LSZS, 2026-09-24).
            let floor = if on_map_final {
                crate::aircraft::approach_floor_kts()
            } else {
                crate::aircraft::speed_floor_kts()
            };
            // With memory (`AirspaceLimit`): a level at or near 10,000 ft
            // must not flip the cap on the altimeter's wander (#154).
            let want = crate::aircraft::floored_target(airspace.limit(alt_ft, planned), floor);
            speed_owed = Some(want);
            // In a descent the governor sits under the speed owed and the
            // pitch holds it (`IdleDescent`); on a MAP's final the speed is
            // the landing's, flown on power.
            let governed = if on_map_final { want } else { idle_descent.governor_kts(want, floor) };
            if crate::tools::speedhold::flight_target().is_some_and(|t| (t - governed).abs() > 0.5) {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({
                        "event": "speed_limit", "alt_ft": alt_ft,
                        "planned": planned, "target": governed, "owed": want,
                        "idle_descent": idle_descent.on(),
                        "airspace_capped": airspace.capped(),
                    }),
                );
                crate::tools::speedhold::arm_flight_with_floor(base_url.clone(), voice.clone(), governed, floor);
            }
        }

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
        let psi_v = Some(psi);
        let crab = match (psi_v, hpath_r.as_ref().ok().copied().filter(|v| v.is_finite())) {
            (Some(psi), Some(hp)) => {
                (((psi - hp + 540.0).rem_euclid(360.0)) - 180.0).clamp(-25.0, 25.0)
            }
            _ => 0.0,
        };
        let dist_nm = bearing_to_fix(lat, lon, leg).map(|_| {
            let f = leg.fix.as_ref().unwrap();
            crate::nav::haversine_nm(lat, lon, f.lat, f.lon)
        });
        publish_fix_dist(dist_nm);
        // How far out this fix counts as reached: the turn onto the NEXT leg
        // has to begin there. Sized to the radius this airframe actually
        // turns at, because a fix inside that circle can never be closed on
        // — the 737's whole EIDW departure was inside it (2026-09-20).
        let gs_kts = gs * MS_TO_KTS;
        let capture_nm = capture_radius_nm(
            gs_kts,
            leg_turn_deg(lat, lon, leg, legs.get(idx + 1)).unwrap_or(0.0),
        );
        // A CF leg is flown ON its course, and an approach's IF behind us is
        // joined by a pattern, never homed on: EGKK 26L from WILLO turned
        // 150 degrees at CF26L and was handed to autoland 1,965 m off the
        // course tracking 044 (2026-09-27, `crate::course_leg`). Both give a
        // TRACK; the crab is added here as for every other leg.
        use crate::course_leg::{CourseLine, Join, Steer};
        let track_true = hpath_r.as_ref().ok().copied().filter(|v| v.is_finite()).unwrap_or(psi);
        let mut steer = Steer::Direct;
        let mut steer_track: Option<f64> = None;
        // The course line this tick is judged against, for the log.
        let mut steer_line: Option<CourseLine> = None;
        let join = if matches!(end, EndAction::Approach { .. }) {
            final_join.step(&legs, idx, lat, lon, track_true, gs_kts, mag_var, alt_ft, descent_fpm())
        } else {
            None
        };
        if matches!(join, Some(Join::Skip { .. }) | Some(Join::Pattern { .. })) {
            steer_line = legs.get(idx + 1).and_then(|l| CourseLine::of_leg(l, mag_var));
        }
        let line_pos = |line: &Option<CourseLine>| {
            line.as_ref().map(|l| (l.along_nm(lat, lon), l.xtk_nm(lat, lon)))
        };
        match join {
            Some(Join::Skip { why }) => {
                let (along, xtk) = line_pos(&steer_line).unzip();
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "final_join", "how": why, "idx": idx, "fix": leg.fix_ident,
                        "along_nm": along, "xtk_nm": xtk, "track": track_true}),
                );
                alert(&voice, Priority::Advisory, "Intercepting the final approach course.");
                idx += 1;
                heading_hold.reset();
                closure = None;
                continue;
            }
            Some(Join::Pattern { track_true: t, leg: pattern_leg, started, side }) => {
                if started {
                    let (along, xtk) = line_pos(&steer_line).unzip();
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "final_join", "how": "pattern", "idx": idx, "fix": leg.fix_ident,
                            "side": side, "along_nm": along, "xtk_nm": xtk, "track": track_true}),
                    );
                    alert(
                        &voice,
                        Priority::Advisory,
                        "We're on the far side of the final. A pattern to join it.",
                    );
                }
                steer = Steer::Pattern(pattern_leg);
                steer_track = Some(t);
            }
            _ => {}
        }
        // An RF leg round its arc, a hold as a racetrack (#101).
        let arc = if steer_track.is_none() { leg.arc() } else { None };
        let mut holding = false;
        let mut hold_end: Option<crate::course_leg::HoldEnd> = None;
        if let Some(a) = arc.as_ref() {
            steer_track = Some(crate::course_leg::arc_track(a, lat, lon));
            steer = Steer::Arc;
        } else if steer_track.is_none() && leg.is_hold() {
            if hold_seen != Some(idx) {
                hold_seen = Some(idx);
                hold = crate::course_leg::HoldFlight::for_leg(leg, mag_var).map(|h| (idx, h));
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": if hold.is_some() { "hold_armed" } else { "hold_not_flown" },
                        "idx": idx, "fix": leg.fix_ident, "leg": leg.leg_type, "course": leg.course_mag,
                        "turn": leg.turn.map(String::from), "leg_min": leg.path.hold_leg_min,
                        "leg_nm": leg.dist_nm, "alt": leg.alt1_ft}),
                );
            }
            if let Some((_, h)) = hold.as_mut().filter(|(i, _)| *i == idx) {
                let s = h.step(leg, lat, lon, track_true, gs_kts, alt_ft, CAPTURE_NM, progress_s);
                if s.moved {
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "hold", "idx": idx, "fix": leg.fix_ident,
                            "phase": h.phase.word(), "circuits": h.circuits, "alt_ft": alt_ft}),
                    );
                    if h.phase == crate::course_leg::HoldPhase::TurnOut && h.circuits == 0 {
                        alert(
                            &voice,
                            Priority::Advisory,
                            &format!("Entering the hold at {}.", crate::tools::procedure::spell_ident(&leg.fix_ident)),
                        );
                    }
                }
                if let Some(e) = s.end {
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "hold_end", "idx": idx, "fix": leg.fix_ident,
                            "how": if e == crate::course_leg::HoldEnd::Flown { "flown" } else { "gave_up" },
                            "circuits": h.circuits, "alt_ft": alt_ft, "alt": leg.alt1_ft}),
                    );
                    if e == crate::course_leg::HoldEnd::GaveUp {
                        alert(
                            &voice,
                            Priority::Advisory,
                            &format!(
                                "We did not make {} feet in the hold at {}. Leaving it.",
                                leg.alt1_ft.map(|a| a.round() as i64).unwrap_or(0),
                                crate::tools::procedure::spell_ident(&leg.fix_ident)
                            ),
                        );
                    }
                }
                holding = true;
                hold_end = s.end;
                steer_track = Some(s.track_true);
                steer = s.steer;
            }
        }
        if steer_track.is_none() {
            let line = CourseLine::of_leg(leg, mag_var);
            if let Some((t, s)) = line.as_ref().and_then(|l| {
                crate::course_leg::course_track(l, lat, lon, track_true, gs_kts, steer_was == Steer::Track)
            }) {
                steer_track = Some(t);
                steer = s;
            }
            if line.is_some() {
                steer_line = line;
            }
        }
        let steer_pos = line_pos(&steer_line);
        if steer != steer_was {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "steer", "idx": idx, "fix": leg.fix_ident, "steer": steer.word(),
                    "along_nm": steer_pos.map(|p| p.0), "xtk_nm": steer_pos.map(|p| p.1), "track": track_true}),
            );
        }
        steer_was = steer;
        let target_mag = if let Some(t) = steer_track {
            Some(true_to_mag(t + crab, mag_var))
        } else if let Some(brg_true) = bearing_to_fix(lat, lon, leg) {
            Some(true_to_mag(brg_true + crab, mag_var))
        } else {
            // course is already magnetic; add crab to hold it as a track.
            leg.course_mag.map(|c| (c + crab).rem_euclid(360.0))
        };
        // A course leg flown on its course is done abeam its fix, as a
        // fly-by is: the intercept may still be closing the line.
        // An arc is done at its end radial; a hold when it says so.
        let done = if let Some(a) = arc.as_ref() {
            crate::course_leg::arc_done(a, lat, lon, CAPTURE_NM)
        } else if holding {
            hold_end.is_some()
        } else {
            leg_done(leg, dist_nm, alt_ft, capture_nm, lat, lon)
                || (matches!(steer, Steer::Intercept | Steer::Track)
                    && steer_line
                        .as_ref()
                        .is_some_and(|l| crate::course_leg::course_passed(l, lat, lon, capture_nm)))
        };
        // The closure watchdog judges a leg flown AT its fix; a pattern flies
        // away from the IF on purpose, and so does a hold's outbound leg.
        if let Some(d) = dist_nm.filter(|_| !done && !final_join.flying_pattern() && !holding) {
            if closure.as_ref().is_none_or(|(i, _)| *i != idx) {
                // A new leg is judged on its own: stalls on an earlier leg
                // that was later flown are not this one's (LIPK 2026-09-25:
                // two at the start of the 51 nm leg to ESFOK, thirty minutes
                // later a third on FI30 stood the approach down). And the
                // turn onto it is given its time before any verdict.
                if closure.is_some() {
                    stalls = 0;
                }
                let turn = bearing_to_fix(lat, lon, leg)
                    .map(|b| ((b - psi + 540.0).rem_euclid(360.0) - 180.0).abs())
                    .unwrap_or(0.0);
                let grace = leg_turn_grace_s(turn);
                closure = Some((
                    idx,
                    crate::circuit::Closure::with_grace(d * 1852.0, progress_s, grace),
                ));
            }
            if closure.as_mut().unwrap().1.stalled(d * 1852.0, progress_s) {
                stalls += 1;
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({
                        "event": "leg_stalled", "idx": idx, "dist_nm": d, "stalls": stalls,
                        "cap_nm": capture_nm,
                    }),
                );
                let heading = true_to_mag(psi, mag_var);
                if xp.set_float(HDG_BUG, heading).await.is_ok() {
                    bug_mag = heading;
                }
                heading_hold.reset();
                if stalls < 3 {
                    alert(
                        &voice,
                        Priority::Caution,
                        "Not closing on the next fix. Rejoining.",
                    );
                    continue;
                }
                let _ = xp.set_float(VS_DIAL, 0.0).await;
                stand_down(
                    &voice,
                    "Not closing on the next fix. Your aircraft.",
                    "leg_stalled",
                );
                return;
            }
        }
        if let Some(t) = target_mag {
            let now_s = started.elapsed().as_secs_f64();
            // Keyed by the phase too: a new intercept, course or pattern leg
            // is set at once, corrections inside one wait (`HeadingHold`).
            if let Some(h) = heading_hold.decide(steer.key(idx), t, true_to_mag(psi, mag_var), phi, now_s) {
                if xp.set_float(HDG_BUG, h).await.is_ok() {
                    bug_mag = h;
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "heading_set", "idx": idx, "hdg": h, "crab": crab}),
                    );
                } else {
                    heading_hold.reset();
                }
            }
        }
        // The lateral mode must actually be ON for the bug to mean anything —
        // the circuit's guard, which this follower LACKED: an AP disconnect
        // mid-climb left the servos off with only the VS re-engage watching,
        // and the aircraft spiralled at 20°/s while the loop slewed a bug
        // nobody was following (LEJR enroute, 2026-07-24).
        if hdg_st <= 0.5 {
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
                Some(ground) => {
                    terrain_misses = 0;
                    publish_terrain_floor(Some(ground + TERRAIN_CLEARANCE_FT))
                }
                None if !terrain_probe_fatal(
                    &mut terrain_misses,
                    crate::xplane::sim_frozen(&xp).await,
                ) =>
                {
                    // A probe the sim never answered because it was FROZEN is
                    // no verdict on the plugin: the pilot paused 0.6 s after
                    // the query went out, the two-second wait ran into the
                    // pause, and the floor was switched off for the whole
                    // LXGB-LEAX flight (2026-09-15). Deferred; only two live
                    // misses in a row retire it.
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "terrain_probe_deferred", "misses": terrain_misses}),
                    );
                }
                None => {
                    // Said aloud: a floor that retires in silence is a
                    // safety feature the pilot believes is still on (two
                    // days of flights without one, 2026-09-16).
                    terrain_available = false;
                    publish_terrain_floor(None);
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "terrain_probe_unavailable"}),
                    );
                    alert(
                        &voice,
                        Priority::Caution,
                        "The terrain probe isn't answering. No terrain floor for the rest of this flight; I'll hold the plan's altitudes. Check the plugin build.",
                    );
                }
            }
        }
        // --- The pilot's altitude outranks the plan's, the ground outranks
        // both. The autopilot tool hands a commanded altitude to the follower
        // rather than writing the dial, so the follower stays the one writer
        // and an override cannot be undone by the next tick.
        // The STAR's "at or below"s ahead, kept reachable: this fix's target
        // comes down to whatever still makes the tightest one further on.
        let star_leg;
        let (leg_for_vertical, arrival_ceiling) = match ceilinged_leg_alt(
            &legs,
            &arrival_constraints,
            idx,
            gs_kts,
            descent_fpm(),
            &mut ceiling_latch,
        ) {
            Some((alt, c)) => {
                if star_ceiling_logged != Some(idx) {
                    star_ceiling_logged = Some(idx);
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "star_ceiling", "idx": idx, "fix": leg.fix_ident,
                                           "plan_ft": leg.alt1_ft, "ceiling_ft": c}),
                    );
                }
                let mut l = leg.clone();
                l.alt1_ft = Some(alt);
                star_leg = l;
                (&star_leg, true)
            }
            None => (leg, false),
        };
        // Round a pattern the descent is paced on the pattern's miles, not
        // the straight line to an IF it is flying away from (#156); an IF
        // skipped for the intercept but still ahead along the final keeps
        // its altitude on the miles to it (#161).
        let pattern_nm = final_join.pattern_to_go_nm(&legs, idx, lat, lon, gs_kts, mag_var);
        let owed_if = final_join.skipped_if_ahead_nm(&legs, idx, lat, lon, mag_var);
        let leg_for_vertical = owed_if.map_or(leg_for_vertical, |(i, _)| &legs[i]);
        let v = vertical_tick(
            leg_for_vertical,
            carry_alt,
            prev_cmd_alt,
            alt_ft,
            owed_if.map(|(_, d)| d).or(pattern_nm).or(dist_nm),
            gs_kts,
            pilot_altitude(),
            terrain_floor_ahead_ft().map(|f| floor_indicated(f, true_alt_ft, alt_ft)),
            descent_fpm(),
            DescentProfile {
                cruise_ft: descent_cruise_ft,
                peak_ft: plan_peak_ft,
                past_top: plan_top.is_some_and(|t| idx > t),
                arrival_ceiling,
            },
        );
        plan_peak_ft = v.peak_ft;
        // A planned step climb begins on this leg: said once, as a crew
        // says it, and only when it is the plan's level being flown (not a
        // pilot's, not the terrain floor's).
        if let Some((fix, from_ft, to_ft)) = step_climb_at(&legs, idx, &cruise_schedule) {
            if !v.pilot && v.floored.is_none() && said_steps.insert(fix.clone()) {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "step_climb", "idx": idx, "fix": fix,
                                       "from_ft": from_ft, "to_ft": to_ft, "alt_ft": alt_ft}),
                );
                alert(
                    &voice,
                    Priority::Advisory,
                    &format!("Step climb, {}.", atc::atis::spoken_flight_level(to_ft)),
                );
            }
        }
        if let Some((plan_ft, held_ft)) = v.climb_held {
            if climb_held_logged != Some(idx) {
                climb_held_logged = Some(idx);
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "climb_no_descent", "idx": idx, "fix": leg.fix_ident,
                                       "plan_ft": plan_ft, "held_ft": held_ft, "alt_ft": alt_ft}),
                );
            }
        }
        if let Some((plan_ft, held_ft)) = v.descent_held {
            if descent_held_logged != Some(idx) {
                descent_held_logged = Some(idx);
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "descent_no_climb", "idx": idx, "fix": leg.fix_ident,
                                       "plan_ft": plan_ft, "held_ft": held_ft, "alt_ft": alt_ft}),
                );
            }
        }
        // The leg to a MAP flies the published path, rate fed forward
        // (`map_path`); every other leg the constraint law above.
        let path = map_path(leg, dist_nm, gs_kts);
        let want_alt = path.map_or(v.want_alt, |(a, _)| a);
        // A departure minimum ahead is a FLOOR on the target until its fix
        // is flown, not only a reason to climb faster: a plan predicting
        // 3,000 ft at RIM against the chart's 4,000 levelled at 3,000 and the
        // minimum watch then called "not climbing" and handed the aircraft
        // back; a minimum dropped the moment it was met hunted round it
        // (EGKK D156J, #153). Minimums above the cruise were dropped at the
        // start (`minimums_within_cruise`). Then an IFR clearance's altitude
        // caps the climb, the minimum's included, until departure clears
        // higher (`departure_target_ft`).
        let ceiling_now = cleared_ceiling();
        let minimum = DepartureMinimums::floor(&climb_minimums, &legs, idx).filter(|_| path.is_none());
        let dep = departure_target_ft(want_alt, minimum.map(|m| m.1), ceiling_now, v.pilot, v.floored);
        let want_alt = dep.want_ft;
        if dep.minimum_capped {
            if let (Some((at, min_ft)), Some(c)) = (minimum, ceiling_now) {
                let fix = legs[at].fix_ident.clone();
                if said_min_capped.insert(fix.to_ascii_uppercase()) {
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "minimum_capped", "fix": fix, "min_ft": min_ft,
                                           "ceiling_ft": c, "alt_ft": alt_ft}),
                    );
                    alert(
                        &voice,
                        Priority::Advisory,
                        &format!(
                            "{fix} needs {}, we're cleared to {}. Holding it until departure clears us higher.",
                            crate::flightplan::spoken_feet(min_ft),
                            crate::flightplan::spoken_feet(c),
                        ),
                    );
                }
            }
        }
        // The clearance let go of a target it was holding down: the climb
        // on is said, once, as the dial moves.
        if let Some(c) = cap_holding {
            if ceiling_now.is_none() && want_alt > c + STEP_FT {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "cleared_climb", "from_ft": c, "to_ft": want_alt, "alt_ft": alt_ft}),
                );
                alert(
                    &voice,
                    Priority::Advisory,
                    &format!("Cleared higher. Climbing to {}.", crate::flightplan::spoken_feet(want_alt)),
                );
            }
        }
        cap_holding = if dep.cap_limited { ceiling_now } else { None };
        carry_alt = v.carry;
        // A minimum's level is COMMANDED: the climb's hold on a plan
        // altitude below the level commanded (`climb_no_descent`) keeps it
        // after its fix, instead of the plan's navlog 5,000 pulling the
        // aircraft back down from 6,000.
        prev_cmd_alt = if dep.raised_by_minimum { v.prev_cmd.max(want_alt) } else { v.prev_cmd };
        // The follower's own top of descent, for the advisor and the spoken
        // answer: the same point the law above paces against. `carry` is the
        // leg's constraint (carried across unconstrained fixes).
        // No top of descent on an approach: its descent is the procedure's
        // platform altitudes and path, and "descent coming up, down to 200
        // feet" was said four times down the LIPE 12 ILS (2026-09-25).
        let tod = dist_nm
            .filter(|_| !on_approach && path.is_none())
            .and_then(|d| {
                descent_point_nm(v.carry, alt_ft, gs_kts, descent_fpm()).map(|tod| PlannedTod {
                    fix: leg.fix_ident.clone(),
                    dist_nm: d,
                    to_tod_nm: d - tod,
                    target_ft: v.carry,
                    started: want_alt < alt_ft - 100.0,
                    held_ft: None,
                })
            });
        let pilot_holds = v.pilot && want_alt > v.carry + 100.0;
        publish_tod(held_tod(tod, pilot_holds.then_some(want_alt), &mut held_leg));
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
        // The dial is the level this movement ends at, set once
        // (`dial_target_ft`); the path itself is flown on the VS dial below.
        // A terrain floor or a pilot's level is a level the dial must not
        // go under.
        let held = if v.pilot || v.floored.is_some() { Some(want_alt) } else { None };
        let dial = dial_target_ft(
            want_alt,
            alt_ft,
            v.carry,
            held,
            path.and(leg.alt1_ft),
        );
        if !set_confirmed(
            "procedure",
            &xp,
            ALT_DIAL,
            dial,
            &mut last_alt_dial,
            50.0,
        )
        .await
        {
            continue;
        }
        // Speed-protected: VS mode holds RATE whatever the airspeed, so the
        // climb rate backs off as IAS sags below Vy (the takeoff's own law,
        // floored positive) — the unprotected 700 fpm stalled the LEJR climb.
        // Both limbs are the AIRFRAME's now. The shared 700 was a GA figure
        // and a 1.1 degree path at jet speeds: the BIKF arrival needed about
        // 2,000 fpm on every leg inbound of BASLU and would have arrived
        // 17,000 ft high, while the same cap on the climb made FL340 a
        // 37-minute job burning low-altitude fuel (2026-09-20). The descent
        // rate is the SAME one that placed the top of descent, so the point
        // and the rate can never disagree.
        let now_ias = std::time::Instant::now();
        let ias_trend = prev_ias.map_or(0.0, |(p, t0): (f64, std::time::Instant)| {
            let dt = now_ias.duration_since(t0).as_secs_f64();
            if (0.2..=5.0).contains(&dt) {
                (ias - p) / dt
            } else {
                0.0
            }
        });
        prev_ias = Some((ias, now_ias));
        // Smoothed for the descent's speed protection (the tick-to-tick
        // difference of a half-second tick is mostly the ASI's noise).
        ias_trend_f += (ias_trend - ias_trend_f) * (dt / (IAS_TREND_TAU_S + dt)).clamp(0.0, 1.0);
        let climb_cap = if minimums_state.unmet(&climb_minimums, &legs, idx, alt_ft).is_some() {
            best_climb_fpm()
        } else {
            climb_fpm()
        };
        let mut vs_cmd = match path {
            Some((path_ft, ff_fpm)) => (ff_fpm + (path_ft - alt_ft) * 2.0)
                .clamp(-descent_fpm().max(ff_fpm.abs() * 1.4), 300.0),
            None => {
                // The feed-forward only while the plan's path IS the target
                // (a departure minimum or a MAP path is not it).
                let ff = if (want_alt - v.want_alt).abs() < 1.0 { v.target_fpm } else { 0.0 };
                vs_law_fpm(want_alt, alt_ft, ff, descent_fpm(), climb_cap)
            }
        };
        if vs_cmd > 0.0 && vy_kts > 0.0 {
            vs_cmd = vs_cmd.min(crate::autotakeoff::climb_vs_fpm(ias, vy_kts, vs_cmd));
            // ...and the circuit's own fade to NOTHING at the airframe's speed
            // floor. The takeoff law above takes 60 fpm a knot off its cap and
            // never goes under 150, so against the SF50's 1,500 fpm cap it
            // still asked about 1,000 fpm at 104 kt and FL260, 16 kt under Vy
            // and falling (EDLP-LOWI, 2026-09-24). A climb is never worth more
            // than the airspeed left.
            if let Some(floor) = crate::aircraft::speed_floor_kts() {
                vs_cmd = crate::circuit::climb_limit_fpm(vs_cmd, ias, ias_trend, vy_kts, floor);
            }
        }
        if vs_cmd < -DRAG_MIN_DESCENT_FPM {
            // Down from cruise: the destination's transition level rules.
            crate::tools::instruments::note_descending();
        }
        // The descent flies the SPEED first (`speed_protected_vs`): a descent
        // that would outrun the governor gives way before the speed does,
        // and the height it leaves behind is the speedbrake's
        // (`descent_drag`). A published MAP path keeps its own rate.
        let vs_want = vs_cmd;
        let target_kts = speed_owed.or_else(crate::tools::speedhold::flight_target);
        if path.is_none() {
            vs_cmd = speed_protected_vs(vs_cmd, target_kts.map(|t| ias - t), ias_trend_f);
            idle_descent.step(vs_want);
        } else {
            idle_descent.step(0.0);
        }
        // The next STAR window, judged from here: an "at or below" at the
        // airframe's best descent, an "at or above" on the climb this loop is
        // FLYING (`window_climb_fpm`). One that cannot be made is said once
        // (#32). Not while the pilot or a clearance holds the altitude
        // (their level, their restrictions).
        let window_climb = window_climb_fpm(vs_cmd, dep.cap_limited);
        if let (false, true, Some(climb)) =
            (arrival_constraints.is_empty(), pilot_altitude().is_none(), window_climb)
        {
            if let Some(miss) = star_window_miss(
                &legs,
                &arrival_constraints,
                idx,
                lat,
                lon,
                alt_ft,
                gs_kts,
                climb,
                descent_fpm(),
                star_cruise_ft,
            ) {
                if said_unable_windows.insert(miss.fix.to_ascii_uppercase()) {
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "star_window_unable", "fix": miss.fix,
                            "low": miss.low, "limit_ft": miss.limit_ft,
                            "predicted_ft": miss.predicted_ft, "by_ft": miss.by_ft,
                            "dist_nm": miss.dist_nm, "alt_ft": alt_ft, "climb_fpm": climb}),
                    );
                    alert(&voice, Priority::Advisory, &miss.line());
                }
            }
        }
        {
            let ours = DRAG_OURS.load(Ordering::SeqCst);
            let held_s = drag_changed.map_or(f64::INFINITY, |c: std::time::Instant| c.elapsed().as_secs_f64());
            let want_out = descent_drag(ours, held_s, ias, target_kts, vs_want, alt_ft - want_alt);
            let has_speedbrake = crate::takeoff_config::has_speedbrake();
            // Never with flap or gear (`drag_barred`), read only while the
            // drag is out or wanted. A failed read is no measurement: a brake
            // of ours stays as the speed rule says, one not out stays in.
            let barred = if has_speedbrake && (ours || want_out) {
                let (flap_r, gear_r) =
                    tokio::join!(xp.get_float(FLAP_RATIO), xp.get_float(GEAR_HANDLE));
                match (flap_r, gear_r) {
                    (Ok(f), Ok(g)) => drag_barred(f, g > 0.5, &crate::aircraft::flap_limits()),
                    _ if ours => None,
                    _ => Some("unread"),
                }
            } else {
                None
            };
            let want_out = want_out && barred.is_none();
            if want_out != ours && has_speedbrake {
                // Read first: a lever already out is the pilot's, and an
                // ARMED one (negative) is stowed-and-armed, ours to use.
                let at = xp.get_float(SPEEDBRAKE).await.ok();
                let write = match (want_out, at) {
                    (true, Some(v)) if v <= 0.05 => Some(SPEEDBRAKE_FLIGHT),
                    (false, Some(_)) => Some(0.0),
                    _ => None,
                };
                if let Some(v) = write {
                    if xp.set_float(SPEEDBRAKE, v).await.is_ok() {
                        DRAG_OURS.store(want_out, Ordering::SeqCst);
                        drag_changed = Some(std::time::Instant::now());
                        let said = drag_voice.say(want_out, drag_clock.elapsed().as_secs_f64());
                        crate::blackbox::record(
                            "procedure",
                            serde_json::json!({"event": "speedbrake", "out": want_out,
                                "ias": ias, "target": target_kts,
                                "vs_cmd": vs_cmd, "vs_want": vs_want, "high_ft": alt_ft - want_alt,
                                "held_s": held_s.is_finite().then_some(held_s),
                                "said": said,
                                "why": if want_out { "speed" } else { barred.unwrap_or("speed") }}),
                        );
                        if said {
                            alert(
                                &voice,
                                Priority::Advisory,
                                if want_out {
                                    "Speedbrake."
                                } else {
                                    "Speedbrake in."
                                },
                            );
                        }
                    }
                }
            }
        }
        // The modes are read before the VS wheel is touched (`vs_action`):
        // VS mode gone NEAR the target, or an ALT hold closing on OUR dial,
        // is the autopilot's own level-off, left flying and its wheel left
        // alone (a VS dial written under ALT HOLD reads back 0 and threw the
        // tick away, EGKK 2026-09-28). Far from the target it's a dropped
        // mode: re-engage, read the mode again, and only then the wheel.
        let mut action = if vs_st > 0.5 {
            VsAction::Write
        } else {
            // A failed read of the hold is no measurement: not holding.
            let alt_hold_on = xp.get_float(ALT_HOLD_STATUS).await.is_ok_and(|v| v > 1.5);
            vs_action(false, alt_hold_on, alt_ft, want_alt, dial, vs_cmd, climb_rate_fpm)
        };
        if action == VsAction::Engage {
            last_vs_cmd = None; // engage can sync the dial even if the response is lost
            if xp.run_command(CMD_VS_MODE).await.is_err() {
                continue;
            }
            if xp.get_float(VS_STATUS).await.is_ok_and(|v| v > 0.5) {
                action = VsAction::Write;
                // A vertical mode engaged: its pitch authority is checked
                // afresh (#181).
                if let Some(vs) = climb_watch.vs_fpm() {
                    crate::climb_watch::arm_authority("procedure", vs, vs_cmd);
                }
            }
        }
        if vs_action_logged != Some(action) {
            vs_action_logged = Some(action);
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "vs_mode", "action": format!("{action:?}").to_lowercase(),
                                   "alt_ft": alt_ft, "want_alt": want_alt, "dial": dial, "vs_cmd": vs_cmd}),
            );
        }
        // The autopilot's answer, WATCHED (#181): the climb far over the
        // wheel with the speed going, or one still on under the floor, is
        // sent down and taken if it does not come down. The flap is read
        // only under the clean floor (its configuration's floor then).
        let clean_floor = crate::aircraft::speed_floor_kts();
        let watch_floor = match clean_floor {
            Some(f) if ias < f => match xp.get_float(FLAP_RATIO).await {
                Ok(flap) => crate::aircraft::config_floor_kts(flap).or(Some(f)),
                Err(_) => Some(f),
            },
            other => other,
        };
        let verdict = climb_watch.step(
            watch_t0.elapsed().as_secs_f64(),
            alt_ft,
            ias,
            watch_floor,
            (action == VsAction::Write).then_some(vs_cmd),
        );
        if verdict != last_verdict {
            last_verdict = verdict;
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "climb_watch", "verdict": format!("{verdict:?}"),
                                   "ias": ias, "vs": climb_watch.vs_fpm(), "vs_cmd": vs_cmd,
                                   "floor": watch_floor, "alt_ft": alt_ft}),
            );
        }
        // The autopilot's pitch AUTHORITY after an engage (#181): a
        // takeoff's carried into the follower, or the follower's own.
        let authority = crate::climb_watch::feed_authority(
            climb_watch.vs_fpm(),
            (action == VsAction::Write).then_some(vs_cmd),
        );
        let take = match (verdict, authority) {
            (_, Some(crate::climb_watch::Authority::NoAuthority(_))) => {
                Some(crate::climb_watch::Why::NoAuthority)
            }
            (crate::climb_watch::Verdict::TakeOver(why), _) => Some(why),
            _ => None,
        };
        if let Some(why) = take {
            let hold_kts = target_kts;
            let no_pitch = why.ap_has_no_pitch();
            // The recovery says the missing pitch itself, first thing.
            let line = if no_pitch {
                "I have control.".to_string()
            } else {
                format!("{} I have control, recovering.", why.line())
            };
            stand_down(
                &voice,
                &line,
                if no_pitch { "no_pitch_authority" } else { "climb_not_held" },
            );
            crate::tools::recover::take_over(
                base_url.clone(),
                voice.clone(),
                crate::tools::recover::TakeOver {
                    why: format!("procedure_{}", why.word()),
                    min_safe_ias: watch_floor
                        .map(|f| f + crate::circuit::RECOVERY_MARGIN_KTS),
                    hold_kts,
                    ap_no_pitch: no_pitch,
                    hold_ft: None,
                },
            );
            return;
        }
        let vs_cmd = crate::climb_watch::dial_for(verdict, vs_cmd);
        // The VS wheel in the 100 fpm clicks an autopilot has, moved only
        // when the click changes: the law's fine value every tick was the
        // same twitching the heading bug did (2026-09-26).
        let vs_cmd = vs_dial_fpm(vs_cmd);
        if action == VsAction::Write {
            if !set_confirmed("procedure", &xp, VS_DIAL, vs_cmd, &mut last_vs_cmd, 50.0).await {
                continue;
            }
        } else {
            // Unwritten: the next VS engage writes it afresh.
            last_vs_cmd = None;
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
                "dist_nm": dist_nm, "alt_ft": alt_ft, "alt_true": true_alt_ft, "want_alt": want_alt,
                // The vertical as commanded: the law's want, the speed's
                // hold on it (`speed_protected_vs`), the wheel.
                "vs_want": vs_want, "vs_cmd": vs_cmd, "ias": ias, "spd_tgt": target_kts,
                "agl_ft": agl_ft, "crab": crab, "psi": psi_v, "bug": bug_mag, "tgt": target_mag,
                // How the lateral is flown and where on the course it is
                // (`course_leg`): the FAF's line during a final's join.
                "steer": steer.word(), "along_nm": steer_pos.map(|p| p.0), "xtk_nm": steer_pos.map(|p| p.1), "pattern_nm": pattern_nm,
                // A skipped IF still ahead, its miles (#161).
                "if_owed_nm": owed_if.map(|o| o.1),
                // The capture radius in force, so a fix that will not close
                // can be told from one the aircraft simply has not reached.
                "cap_nm": capture_nm,
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
            if idx >= hi && done {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "handoff_autoland", "idx": idx, "agl_ft": agl_ft}),
                );
                if map_short_of_runway(leg) && !MAP_AUTOLAND {
                    // The visual segment is the pilot's, talked down: the
                    // aircraft configured and on speed, the autopilot still
                    // holding the descent, the talk-down armed on THIS path
                    // (from here to 50 ft over the threshold, not 3 degrees).
                    let rwy = match rwy_id.clone() {
                        Some(r) => {
                            let apt = apt_dat.clone();
                            tokio::task::spawn_blocking(move || {
                                crate::autoland::runway_by_id(&apt, lat, lon, &r.id, 10.0, r.icao.as_deref())
                                    .ok()
                                    .flatten()
                            })
                            .await
                            .ok()
                            .flatten()
                        }
                        None => None,
                    };
                    let path = rwy.as_ref().map(|r| map_handover_path(r, lat, lon, alt_ft));
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "map_handover", "agl_ft": agl_ft,
                            "ft_per_nm": path.as_ref().map(|p| p.ft_per_nm),
                            "dist_nm": rwy.as_ref().map(|r| crate::nav::haversine_nm(lat, lon, r.thr_lat, r.thr_lon))}),
                    );
                    let line = match (&path, &rwy) {
                        (Some(p), Some(r)) => {
                            crate::approach_guide::arm(p.clone());
                            format!(
                                "Missed approach point. Runway {} ahead, {:.1} miles. Your aircraft. \
                                 The autopilot is holding the descent; I'll talk you down.",
                                crate::circuit::spell_runway(&r.id),
                                crate::nav::haversine_nm(lat, lon, r.thr_lat, r.thr_lon)
                            )
                        }
                        _ => "Missed approach point, runway ahead. Your aircraft.".to_string(),
                    };
                    stand_down(&voice, &line, "map_handover");
                    return;
                }
                if map_short_of_runway(leg) {
                    // The visual segment: its own angle to the landing, and
                    // the published missed approach for its go-around.
                    crate::autoland::set_next_glidepath_deg(visual_deg);
                    stash_missed_legs(missed_legs.clone());
                    alert(&voice, Priority::Advisory, "Missed approach point. Runway ahead, landing us.");
                } else {
                    alert(&voice, Priority::Advisory, "At the fix. Landing us.");
                }
                // The governor lets go BEFORE autoland's throttle law starts,
                // so no tick has two writers on the lever.
                crate::tools::speedhold::release();
                crate::autoland::ABORT.store(false, Ordering::SeqCst);
                crate::autoland::run_autoland_scoped(
                    base_url.clone(),
                    apt_dat.clone(),
                    cfg.clone(),
                    *approach_ias,
                    voice.clone(),
                    None,
                    rwy_id.clone(),
                    xp.cancellation(),
                );
                clear_pilot_altitude();
                publish_terrain_floor(None);
                RUN.record(sim::run::Outcome::HandedOff);
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

        if done {
            idx += 1;
            if idx < legs.len() {
                let next = &legs[idx];
                // A course leg is named by its course, the way it is flown.
                let label = match (next.fix_ident.is_empty(), next.course_mag) {
                    (false, _) => next.fix_ident.clone(),
                    (true, Some(c)) => format!("course {}", crate::flightplan::spell_code(&format!("{:03}", c.round().rem_euclid(360.0) as i64))),
                    (true, None) => "the next leg".to_string(),
                };
                alert(&voice, Priority::Advisory, &format!("Next, {label}."));
            }
        }
    }

    RUN.record(if matches!(end, EndAction::Approach { .. }) {
        sim::run::Outcome::Failed("approach ended without handoff".into())
    } else {
        sim::run::Outcome::Completed
    });
    // Ran off the end of the legs (a departure, or an approach with no handoff).
    // Drop any cruise-power hold before handing back (no-op if never armed).
    crate::tools::speedhold::release();
    match end {
        EndAction::Level => {
            crate::blackbox::record("procedure", serde_json::json!({"event": "complete_level"}));
            alert(
                &voice,
                Priority::Readback,
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

/// Live misses of the terrain probe before the floor is retired for the
/// flight. One is a race with a pause or a slow frame; two in a row is a
/// plugin that does not answer.
pub const TERRAIN_MISSES_TO_RETIRE: u32 = 2;

/// Count an unanswered terrain probe. A miss while the sim is frozen is not
/// counted at all (the plugin's flight loop was not running to answer);
/// returns true when the probe should be retired for the flight.
pub fn terrain_probe_fatal(misses: &mut u32, frozen: bool) -> bool {
    if frozen {
        return false;
    }
    *misses += 1;
    *misses >= TERRAIN_MISSES_TO_RETIRE
}

#[cfg(test)]
mod tests {
    /// An IFR clearance's altitude caps the climb, below the plan's cruise,
    /// until released; a pilot's own altitude outranks it and the ground
    /// ahead outranks both.
    #[test]
    fn the_cleared_altitude_caps_the_climb() {
        assert_eq!(cap_at_clearance(35_000.0, Some(5_000.0), false, None), 5_000.0);
        assert_eq!(cap_at_clearance(4_000.0, Some(5_000.0), false, None), 4_000.0, "below it: the plan");
        assert_eq!(cap_at_clearance(35_000.0, None, false, None), 35_000.0, "released");
        assert_eq!(cap_at_clearance(9_000.0, Some(5_000.0), true, None), 9_000.0, "the pilot's");
        assert_eq!(
            cap_at_clearance(35_000.0, Some(5_000.0), false, Some(7_200.0)),
            7_200.0,
            "the terrain floor wins"
        );
    }

    fn fix_leg(ident: &str, lat: f64, lon: f64) -> ProcLeg {
        ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "TF".into(),
            fix_ident: ident.into(),
            fix_region: String::new(),
            fix: Some(crate::nav::Fix { ident: ident.into(), lat, lon }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: Some(20_000.0),
            alt2_ft: None,
            is_faf: false,
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
        }
    }

    /// #79: an in-flight re-pick replaces the route from the STAR's entry on,
    /// the leg flying keeps its index, and a splice the aircraft has flown
    /// past (or computed against another route) is refused, never applied.
    #[test]
    fn a_splice_replaces_the_route_from_its_entry_while_the_entry_is_ahead() {
        let legs: Vec<ProcLeg> = ["A", "B", "BAMEK", "OLD1", "OLD2"]
            .iter()
            .enumerate()
            .map(|(i, f)| fix_leg(f, 45.0 + i as f64 * 0.1, 10.0))
            .collect();
        let s = Splice {
            from: 2,
            entry: "BAMEK".into(),
            legs: ["BAMEK", "NEW1", "NEW2", "NEW3"].iter().map(|f| fix_leg(f, 44.8, 11.0)).collect(),
            constraints: Vec::new(),
            why: "wind".into(),
        };
        for idx in [0, 1, 2] {
            let out = apply_splice(&legs, idx, &s).unwrap();
            let ids: Vec<&str> = out.iter().map(|l| l.fix_ident.as_str()).collect();
            assert_eq!(ids, ["A", "B", "BAMEK", "NEW1", "NEW2", "NEW3"]);
            assert_eq!(out[idx].fix_ident, legs[idx].fix_ident, "the leg flying keeps its index");
        }
        assert_eq!(apply_splice(&legs, 3, &s).unwrap_err(), "past_entry");
        let moved = Splice { from: 1, ..s.clone() };
        assert_eq!(apply_splice(&legs, 0, &moved).unwrap_err(), "route_changed");
        let off_end = Splice { from: 9, ..s.clone() };
        assert_eq!(apply_splice(&legs, 0, &off_end).unwrap_err(), "route_changed");
        let headless = Splice { legs: vec![fix_leg("NEW1", 44.8, 11.0)], ..s };
        assert_eq!(apply_splice(&legs, 0, &headless).unwrap_err(), "no_entry");
    }

    /// LSZS 03 hand-over at the MAP (1.98 nm, 6,598 ft, threshold 5,602):
    /// the talk-down's path runs from here to 50 ft over the threshold, so it
    /// starts "on glidepath" on the steep segment instead of 300 ft high on 3.
    #[test]
    fn a_map_handover_talks_down_the_path_actually_flown() {
        let rwy = crate::autoland::Runway {
            icao: "LSZS".into(),
            id: "03".into(),
            thr_lat: 46.5270195,
            thr_lon: 9.8780890,
            course_deg: 29.0,
            length_m: 1842.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 5602.0,
        };
        let (lat, lon) = (46.5, 9.850666667);
        let path = super::map_handover_path(&rwy, lat, lon, 6598.0);
        assert!((path.ft_per_nm - 478.0).abs() < 10.0, "{}", path.ft_per_nm);
        let d = crate::approach_guide::deviations(&path, lat, lon, 6598.0);
        assert!(d.vdev_ft.abs() < 60.0, "on glidepath at the MAP: {}", d.vdev_ft);
        assert!(crate::approach_guide::callout(&d).unwrap().contains("on glidepath"));
        assert!(!super::MAP_AUTOLAND, "the steep autoland stays off until proven");
    }

    #[test]
    fn a_map_on_the_runway_is_a_landing_not_a_visual_segment() {
        let mut leg = super::ProcLeg {
            seq: 30,
            transition: String::new(),
            leg_type: "CF".into(),
            fix_ident: "RW12".into(),
            fix_region: String::new(),
            fix: None,
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: None,
            alt2_ft: None,
            is_faf: false,
            is_map: true,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
        };
        assert!(!super::map_short_of_runway(&leg), "LIPE I12: MAP on RW12");
        leg.fix_ident = "MA03".into();
        assert!(super::map_short_of_runway(&leg), "LSZS: MAP short of the runway");
        leg.is_map = false;
        assert!(!super::map_short_of_runway(&leg));
    }

    /// LSZS RNAV 03: FAF ZS702 at 11,800, MAP ZS705 at 6,598 on a 4.49
    /// degree path. The final is flown on the path with its rate fed forward,
    /// and handed over at the MAP, not the FAF.
    #[test]
    fn a_map_final_is_flown_on_its_path_and_handed_over_at_the_map() {
        let mut faf = fix_leg("ZS702", 46.357, 9.688);
        faf.is_faf = true;
        let mut map = fix_leg("ZS705", 46.500, 9.851);
        map.is_map = true;
        map.vpa_deg = Some(4.49);
        map.alt1_ft = Some(6598.0);
        let legs = vec![fix_leg("ZS701", 46.327, 9.576), faf, map.clone()];
        assert_eq!(super::handoff_index(&legs), Some(2), "at the MAP, not the FAF");
        // 5 nm out: 6,598 + 5 x 477 = 8,983 ft, and at 100 kt ~795 fpm down.
        let (alt, ff) = super::map_path(&map, Some(5.0), 100.0).unwrap();
        assert!((alt - 8983.0).abs() < 15.0, "{alt}");
        assert!((ff + 795.0).abs() < 10.0, "{ff}");
        assert_eq!(super::map_path(&legs[1], Some(5.0), 100.0), None, "not the MAP leg");
    }

    /// LOWI MOGT3H (2026-09-24): 7,850 at WI506, about nine miles from the
    /// runway. At cruise speed the SF50 made 375 ft a mile (1,500 fpm at
    /// 240 kt); at Vy it makes the minimum.
    #[test]
    fn a_departure_minimum_is_watched_and_an_unmakeable_one_hands_back() {
        let mins = vec![
            crate::nav::ClimbMinimum { fix: "WI505".into(), min_ft: 4000.0 },
            crate::nav::ClimbMinimum { fix: "WI506".into(), min_ft: 7850.0 },
        ];
        let legs = vec![
            fix_leg("WI505", 47.252, 11.269),
            fix_leg("WI506", 47.290, 11.139),
            fix_leg("WI507", 47.306, 11.086),
        ];
        // Past WI505 at 4,500: WI506 is the one ahead and unmet.
        assert_eq!(super::unmet_minimum(&mins, &legs, 1, 4500.0), Some((1, 7850.0)));
        assert_eq!(super::unmet_minimum(&mins, &legs, 1, 8000.0), None, "met");
        assert_eq!(super::unmet_minimum(&mins, &legs, 1, 7849.0), None, "levelled at it is met");
        assert_eq!(super::unmet_minimum(&mins, &legs, 1, 7700.0), Some((1, 7850.0)));
        // Behind us is not ahead.
        assert_eq!(super::unmet_minimum(&mins, &legs, 2, 4500.0), None);
        let d = super::distance_to_leg_nm(47.252, 11.269, &legs, 1, 1);
        assert!((d - 5.8).abs() < 0.3, "WI505 to WI506 ~5.8 nm: {d}");
        // Cruise-speed climb, 6 nm out at 4,500: short, said.
        assert!(matches!(
            super::watch_minimum(4500.0, 1500.0, 240.0, 6.0, 7850.0),
            super::MinimumWatch::Short { .. }
        ));
        // Vy climb: 2,000 fpm at 130 kt makes it.
        assert_eq!(super::watch_minimum(4500.0, 2000.0, 130.0, 6.0, 7850.0), super::MinimumWatch::OnTrack);
        // Two miles out and 1,400 short: unable.
        assert!(matches!(
            super::watch_minimum(6000.0, 1200.0, 200.0, 2.0, 7850.0),
            super::MinimumWatch::Unable { .. }
        ));
        // A descent is not a climb that will make it.
        assert!(matches!(
            super::watch_minimum(6000.0, -500.0, 150.0, 2.0, 7850.0),
            super::MinimumWatch::Unable { .. }
        ));
    }

    /// EGPH-EGTE, 2026-09-21: 2,000 fpm down at idle, 258 kt through ten
    /// thousand against 250, 278 at the last fix. Since #156 the descent
    /// gives way to the speed first (`speed_protected_vs`) and the brake is
    /// for the height that leaves (or a runaway past 8 kt).
    #[test]
    fn a_descent_gives_way_to_its_speed_and_the_brake_is_for_height() {
        // On the target and steady: the whole descent. Three over: half.
        // Six over: level. Building at 1 kt/s from two over is five in
        // three seconds: already mostly held back.
        assert_eq!(speed_protected_vs(-2000.0, Some(0.0), 0.0), -2000.0);
        assert_eq!(speed_protected_vs(-2000.0, Some(-3.0), 0.0), -2000.0);
        assert!((speed_protected_vs(-2000.0, Some(3.0), 0.0) + 1000.0).abs() < 1.0);
        assert_eq!(speed_protected_vs(-2000.0, Some(6.0), 0.0), 0.0);
        assert!(speed_protected_vs(-2000.0, Some(2.0), 1.0) > -400.0);
        // Slowing toward the target lets the descent back early.
        assert!(speed_protected_vs(-2000.0, Some(4.0), -1.0) < -1600.0);
        // Climbs, level and no target pass through.
        assert_eq!(speed_protected_vs(1500.0, Some(20.0), 0.0), 1500.0);
        assert_eq!(speed_protected_vs(-2000.0, None, 0.0), -2000.0);
        // The brake: never for 8 kt over any more, 9 is a runaway.
        assert!(!descent_drag(false, f64::INFINITY, 258.0, Some(250.0), -2000.0, 0.0));
        assert!(descent_drag(false, f64::INFINITY, 259.0, Some(250.0), -2000.0, 0.0));
        // High on the level commanded with the speed at the target: out...
        assert!(!descent_drag(false, f64::INFINITY, 250.5, Some(250.0), -2000.0, 350.0));
        assert!(descent_drag(false, f64::INFINITY, 250.5, Some(250.0), -2000.0, 450.0));
        // ...not when slow (the protection lets the whole rate through), not
        // for a level or a climb, not inside its hold.
        assert!(!descent_drag(false, f64::INFINITY, 244.0, Some(250.0), -2000.0, 900.0));
        assert!(!descent_drag(false, f64::INFINITY, 250.5, Some(250.0), 0.0, 900.0));
        assert!(!descent_drag(false, 5.0, 250.5, Some(250.0), -2000.0, 900.0));
        // Out, it stays out until back on the level with the speed in...
        assert!(descent_drag(true, f64::INFINITY, 251.0, Some(250.0), -2000.0, 300.0));
        assert!(!descent_drag(true, f64::INFINITY, 251.0, Some(250.0), -2000.0, 100.0));
        assert!(descent_drag(true, f64::INFINITY, 255.0, Some(250.0), -2000.0, 100.0));
        // ...or slow, or a climb; and through a level-off still fast (#126).
        assert!(!descent_drag(true, f64::INFINITY, 245.0, Some(250.0), -2000.0, 600.0));
        assert!(!descent_drag(true, f64::INFINITY, 278.0, Some(250.0), 800.0, 0.0));
        assert!(descent_drag(true, f64::INFINITY, 278.0, Some(250.0), 0.0, 0.0));
        assert!(!descent_drag(false, f64::INFINITY, 278.0, None, -2000.0, 900.0));
    }

    /// EGKK 26L, 2026-09-27 (sortie 1790510400-full_auto, stock 737): out at
    /// 218 kt against 210.25 and 2,000 fpm down, kept out through gear down
    /// and flaps 10 to 40. The speed rule still asks for it; the
    /// configuration says no, and the shallower rate is what is left.
    #[test]
    fn the_speedbrake_is_never_out_with_landing_flap_or_gear() {
        // The 737's detents, as the profile declares them.
        let b738: Vec<(f64, Option<f64>)> = [0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0]
            .iter()
            .map(|r| (*r, None))
            .collect();
        // The speed rule, alone, wanted it out at the sortie's numbers...
        assert!(descent_drag(false, f64::INFINITY, 218.36, Some(210.25), -2000.0, 0.0));
        // ...clean it may be, and flaps 1 is still the clean descent...
        assert_eq!(drag_barred(0.0, false, &b738), None);
        assert_eq!(drag_barred(0.125, false, &b738), None);
        // ...flaps 10 (the sortie's 0.5), 5, 40 and the gear say in.
        assert_eq!(drag_barred(0.5, false, &b738), Some("flaps"));
        assert_eq!(drag_barred(0.375, false, &b738), Some("flaps"));
        assert_eq!(drag_barred(1.0, false, &b738), Some("flaps"));
        assert_eq!(drag_barred(0.0, true, &b738), Some("gear"));
        assert_eq!(drag_barred(1.0, true, &b738), Some("gear"));
        // No detents declared: any flap at all.
        assert_eq!(drag_barred(0.1, false, &[]), Some("flaps"));
        assert_eq!(drag_barred(0.0, false, &[]), None);
        // The 5 s re-deployment at full flap: 153.6 kt against 145 and
        // 968 fpm down is out by speed, in by configuration.
        assert!(descent_drag(false, f64::INFINITY, 153.56, Some(145.0), -967.7, 0.0));
        assert_eq!(drag_barred(1.0, true, &b738), Some("gear"));
        // A runaway still levels the descent with the brake barred.
        assert_eq!(speed_protected_vs(-2000.0, Some(25.0), 0.0), 0.0);
    }

    /// #104 replayed from the recorded sortie itself (EGKK 26L, stock 737,
    /// `logs/sorties/1790510400-full_auto.jsonl`; not in the repo, so
    /// ignored): every `state` tick while the flown speedbrake was out,
    /// judged by `drag_barred` with the 737 profile's own detents and the
    /// gear from the `map_config` records. As flown it stayed out 57 s with
    /// gear down and flap past the first detent and was put out again at
    /// flaps 40; by the rule it goes in on the first tick the gear or flaps
    /// say so and the re-deployment is refused.
    /// `XVL_SORTIE_DIR` overrides where the sortie file is looked for.
    /// `cargo test --release -- --ignored real_sortie_speedbrake_barred --nocapture`
    #[test]
    #[ignore] // needs the recorded sortie; run with --ignored
    fn real_sortie_speedbrake_barred_by_gear_and_flap() {
        const SORTIE: &str = "1790510400-full_auto.jsonl";
        let dirs = [
            std::env::var("XVL_SORTIE_DIR").unwrap_or_default(),
            format!("{}/logs/sorties", env!("CARGO_MANIFEST_DIR")),
            format!("{}/GitHub/xvoicelayer/logs/sorties", std::env::var("HOME").unwrap_or_default()),
        ];
        let Some(text) = dirs
            .iter()
            .filter(|d| !d.is_empty())
            .find_map(|d| std::fs::read_to_string(format!("{d}/{SORTIE}")).ok())
        else {
            return;
        };
        let t0 = 1_790_510_400.0;
        let recs: Vec<serde_json::Value> =
            text.lines().filter_map(|l| serde_json::from_str(l).ok()).collect();
        let limits: Vec<(f64, Option<f64>)> = crate::aircraft::load("boeing_737_800")
            .and_then(|p| p.flaps)
            .expect("the 737's flaps")
            .detents
            .iter()
            .map(|d| (d.ratio, d.vfe_kts))
            .collect();
        let t = |r: &serde_json::Value| r["t"].as_f64().unwrap_or(0.0) - t0;
        let gear_down_at = recs
            .iter()
            .filter(|r| r["event"] == "map_config" && r["gear"] == true)
            .map(t)
            .fold(f64::INFINITY, f64::min);
        // The brake as flown: (time, out).
        let brake: Vec<(f64, bool)> = recs
            .iter()
            .filter(|r| r["kind"] == "procedure" && r["event"] == "speedbrake")
            .map(|r| (t(r), r["out"] == true))
            .collect();
        let out_at = |at: f64| brake.iter().filter(|(bt, _)| *bt <= at).last().is_some_and(|b| b.1);
        let (mut barred_out_s, mut first_barred, mut last_t) = (0.0, None, None::<f64>);
        for r in recs.iter().filter(|r| r["kind"] == "state") {
            let (at, flap) = (t(r), r["flap"].as_f64().unwrap_or(0.0));
            let why = drag_barred(flap, at >= gear_down_at, &limits);
            if out_at(at) && why.is_some() {
                if let Some(prev) = last_t {
                    barred_out_s += at - prev;
                }
                first_barred.get_or_insert((at, why.unwrap(), flap));
                last_t = Some(at);
            } else {
                last_t = None;
            }
        }
        let stowed = brake.iter().find(|(bt, out)| *bt > gear_down_at && !out).map(|b| b.0);
        let redeployed: Vec<f64> =
            brake.iter().filter(|(bt, out)| *bt > gear_down_at && *out).map(|b| b.0).collect();
        println!(
            "gear down +{gear_down_at:.1}; as flown out while barred {barred_out_s:.0} s, first barred \
             tick {first_barred:?}, stowed +{stowed:?}, put out again at {redeployed:?}"
        );
        let (first_at, _, _) = first_barred.expect("the brake was out with gear or flap");
        assert!(barred_out_s > 30.0, "{barred_out_s}");
        // The rule stows it at the first barred tick, within a tick of the gear.
        assert!(first_at - gear_down_at < 2.0, "+{first_at:.1}");
        assert!(stowed.is_some_and(|s| s > first_at + 30.0), "as flown it stayed out");
        // Every re-deployment after the gear came down is refused.
        for at in &redeployed {
            let flap = recs
                .iter()
                .filter(|r| r["kind"] == "state" && t(r) <= *at)
                .last()
                .and_then(|r| r["flap"].as_f64())
                .unwrap_or(0.0);
            assert!(drag_barred(flap, true, &limits).is_some(), "+{at:.1} at flap {flap}");
        }
        assert!(!redeployed.is_empty(), "the sortie's re-deployment at flaps 40");
    }

    /// EGKK-EGBB, 2026-09-27 (sortie 1790541304-full_auto, #126), the
    /// approach at 210.25 kt: (seconds after +2600, the state records' IAS,
    /// the VS law's command rebuilt from the leg records' altitude and
    /// target). The old rule put the brake out and in three times, once in
    /// under 2 s at a level-off 8 kt fast, and said every change.
    #[rustfmt::skip]
    const EGBB_APPROACH: &[(f64, f64, f64)] = &[
            (5.9, 210.4, 10.0), (7.5, 210.1, 10.0), (9.3, 209.6, -2000.0), (11.0, 209.2, -2000.0),
            (12.7, 209.4, -2000.0), (14.4, 210.3, -2000.0), (16.1, 211.7, -2000.0), (17.9, 213.6, -2000.0),
            (19.3, 215.6, -2000.0), (20.9, 217.0, -2000.0), (22.4, 218.6, -2000.0), (24.1, 219.7, -2000.0),
            (25.8, 220.3, -2000.0), (27.5, 220.9, -2000.0), (29.2, 221.6, -2000.0), (30.9, 222.4, -2000.0),
            (32.6, 222.9, -2000.0), (34.4, 223.2, -2000.0), (36.1, 223.3, -2000.0), (37.9, 223.5, -2000.0),
            (39.6, 223.9, -2000.0), (41.3, 223.8, -2000.0), (43.0, 223.5, -2000.0), (44.8, 223.2, -2000.0),
            (46.4, 223.0, -1966.0), (48.1, 222.5, -1910.0), (49.8, 221.9, -1919.0), (51.5, 222.0, -2000.0),
            (53.2, 222.4, -2000.0), (54.9, 222.6, -2000.0), (56.5, 222.5, -2000.0), (58.1, 222.2, -2000.0),
            (59.8, 222.1, -2000.0), (61.5, 222.3, -2000.0), (63.0, 222.3, -1983.0), (64.7, 222.3, -1956.0),
            (66.4, 222.5, -1732.0), (68.1, 222.5, -1855.0), (69.3, 222.4, -1743.0), (71.0, 222.3, -1760.0),
            (72.7, 222.1, -1742.0), (74.4, 222.0, -1724.0), (76.1, 222.0, -2000.0), (77.8, 221.9, -1542.0),
            (79.6, 221.9, -1380.0), (81.3, 222.4, -1348.0), (83.0, 222.4, -1093.0), (84.6, 221.9, -847.0),
            (86.3, 220.8, -674.0), (88.0, 218.8, -584.0), (89.7, 216.4, -534.0), (91.4, 214.3, -735.0),
            (93.2, 212.1, -872.0), (94.6, 210.3, -946.0), (96.2, 209.0, -1080.0), (97.9, 207.8, -1814.0),
            (99.6, 207.8, -1969.0), (101.2, 209.1, -2000.0), (102.9, 211.3, -2000.0), (104.6, 213.1, -1895.0),
            (106.3, 214.6, -1466.0), (107.9, 215.6, -1208.0), (109.5, 215.9, -930.0), (111.2, 216.1, -536.0),
            (112.8, 215.7, -582.0), (114.5, 214.8, -323.0), (116.2, 213.1, -279.0), (117.9, 211.3, -354.0),
            (119.6, 209.2, -392.0), (121.0, 207.8, -524.0), (122.6, 207.4, -816.0), (124.3, 207.0, -1059.0),
            (125.9, 206.9, -1221.0), (127.3, 207.7, -1362.0), (129.0, 209.1, -1805.0), (130.7, 212.0, -1446.0),
            (132.5, 215.1, -1040.0), (134.1, 217.2, -982.0), (135.5, 218.3, -663.0), (137.2, 218.5, -305.0),
            (138.8, 217.7, -38.0), (140.5, 216.7, 164.0), (142.2, 215.7, 209.0), (144.2, 214.5, 213.0),
            (145.9, 213.4, 187.0), (147.3, 211.9, 145.0), (148.8, 210.6, 120.0), (150.5, 208.9, 80.0),
            (152.1, 206.9, 69.0), (153.8, 205.8, 67.0), (155.5, 205.8, 74.0), (157.2, 206.7, 76.0),
            (159.0, 208.2, 78.0), (339.6, 213.3, 28.0), (341.3, 212.7, -1131.0), (342.8, 211.9, -1326.0),
            (344.3, 211.1, -1384.0), (346.0, 210.3, -1397.0), (347.5, 209.7, -1550.0), (349.2, 209.0, -1776.0),
            (350.7, 208.9, -2000.0), (352.4, 209.2, -2000.0), (354.1, 210.2, -2000.0), (355.8, 212.4, -2000.0),
            (357.5, 214.1, -2000.0), (359.2, 215.5, -2000.0), (361.0, 216.9, -2000.0), (362.7, 218.2, -2000.0),
            (364.4, 219.7, -2000.0), (366.1, 220.8, -2000.0), (367.8, 222.1, -2000.0), (369.4, 222.9, -2000.0),
            (371.2, 223.4, -1811.0), (372.8, 223.3, -1770.0), (374.5, 223.6, -1839.0), (376.2, 223.4, -1855.0),
            (377.9, 222.9, -1822.0), (379.6, 222.7, -1740.0), (381.0, 222.2, -1961.0), (382.7, 222.3, -2000.0),
            (384.4, 222.8, -1988.0), (386.1, 223.2, -1722.0), (387.8, 223.1, -1990.0), (389.4, 223.5, -2000.0),
            (391.1, 224.0, -1852.0), (392.9, 223.9, -1864.0), (394.6, 223.6, -1773.0), (396.3, 223.6, -1889.0),
            (397.9, 223.0, -2000.0), (399.6, 222.5, -1897.0), (401.3, 222.0, -1898.0), (403.0, 222.0, -1943.0),
            (404.7, 223.1, -1109.0), (406.5, 222.4, -271.0), (408.2, 221.6, -184.0), (409.6, 221.5, -184.0),
    ];

    /// The recorded approach through the old rule and the new: every change
    /// `(t, out)`, and which are said.
    fn replay_drag(new: bool) -> (Vec<(f64, bool)>, Vec<(f64, bool)>) {
        const TARGET: f64 = 210.25;
        let (mut out, mut since) = (false, f64::NEG_INFINITY);
        let mut voice = DragVoice::default();
        let (mut changes, mut said) = (Vec::new(), Vec::new());
        for &(t, ias, vs) in EGBB_APPROACH {
            let want = if new {
                descent_drag(out, t - since, ias, Some(TARGET), vs, 0.0)
            } else {
                // The rule as flown: the descent gate, then 8 out / 3 in.
                vs <= -DRAG_MIN_DESCENT_FPM
                    && if out { ias - TARGET > DRAG_IN_KTS } else { ias - TARGET > DRAG_OUT_KTS }
            };
            if want != out {
                out = want;
                since = t;
                changes.push((t, out));
                if !new || voice.say(out, t) {
                    said.push((t, out));
                }
            }
        }
        (changes, said)
    }

    /// #126: out and in around the approach speed, every change spoken.
    #[test]
    fn the_speedbrake_holds_through_a_level_off_and_is_said_once() {
        let (old, old_said) = replay_drag(false);
        // As flown: three extensions, one stowed within 2 s by the gate at
        // a level-off still 8 kt fast, and all six said.
        assert_eq!(old.iter().filter(|c| c.1).count(), 3, "{old:?}");
        assert!(old.windows(2).any(|w| w[0].1 && !w[1].1 && w[1].0 - w[0].0 < 5.0), "{old:?}");
        assert_eq!(old_said.len(), 6);
        let (new, said) = replay_drag(true);
        // Every state is held at least the minimum.
        for w in new.windows(2) {
            assert!(w[1].0 - w[0].0 >= DRAG_MIN_HOLD_S, "{new:?}");
        }
        // Never stowed while still fast and not climbing: the level-offs at
        // +2739 and +3004 keep it out.
        for &(t, out) in &new {
            if !out {
                let &(_, ias, vs) = EGBB_APPROACH.iter().find(|r| r.0 == t).unwrap();
                assert!(ias - 210.25 <= DRAG_IN_KTS || vs >= DRAG_KEEP_BELOW_FPM, "in at {t}: {ias} {vs}");
            }
        }
        assert_eq!(new.last().map(|c| c.1), Some(true), "still out 12 kt fast at the handoff");
        // Said: an extension and its stow, then silence for the quick
        // re-extension, then the last extension 4 minutes on.
        assert!(said.len() <= 3, "{said:?}");
        assert!(said.len() < new.len(), "{said:?} of {new:?}");
        for w in said.windows(2) {
            assert!(!w[1].1 || w[1].0 - w[0].0 >= DRAG_SPEAK_GAP_S, "{said:?}");
        }
    }

    /// +2295 and +3001 on EGKK-EGBB: the VS mode dropped as the autopilot
    /// captured the dial, and the follower re-engaged VS over it.
    #[test]
    fn a_capture_of_the_dial_is_not_a_dropped_vs_mode() {
        // HON: 9,202 ft, path 9,068, dial 9,000, -1,900 commanded.
        assert!(vs_mode_capturing(9_202.0, 9_068.0, 9_000.0, -1_900.0));
        // FI33: 2,046 ft, path 1,895, dial 1,800.
        assert!(vs_mode_capturing(2_046.0, 1_895.0, 1_800.0, -1_900.0));
        // Far from the dial, or moving away from it: a dropped mode.
        assert!(!vs_mode_capturing(9_900.0, 9_700.0, 9_000.0, -1_900.0));
        assert!(!vs_mode_capturing(9_202.0, 9_400.0, 9_000.0, 1_500.0));
        // Near the target, as before.
        assert!(vs_mode_capturing(5_050.0, 5_000.0, 3_000.0, -1_000.0));
    }

    #[test]
    fn a_stowed_speedbrake_is_said_only_when_its_extension_was() {
        let mut v = DragVoice::default();
        assert!(v.say(true, 0.0));
        assert!(v.say(false, 60.0));
        assert!(!v.say(true, 100.0), "40 s after the last call");
        assert!(!v.say(false, 130.0), "its extension was not said");
        assert!(v.say(true, 300.0));
    }

    /// #126: OSKOT's 210 on EGKK-EGBB (2026-09-27, +2370 to +2440), the
    /// leg records' distance with the state records' IAS and groundspeed
    /// (every other tick). Judged afresh each tick, the limit came and went
    /// as the aircraft slowed at the planned rate; begun, it holds.
    #[test]
    fn a_star_speed_limit_once_begun_holds() {
        #[rustfmt::skip]
        let recorded: &[(f64, f64, f64)] = &[
            (10.19, 248.8, 261.0), (10.10, 248.8, 261.0), (10.03, 248.1, 260.0), (9.94, 247.6, 259.0),
            (9.86, 247.4, 259.0), (9.78, 247.8, 260.0), (9.70, 248.5, 261.0), (9.62, 249.5, 262.0),
            (9.53, 250.6, 263.0), (9.45, 251.8, 265.0), (9.37, 252.7, 266.0), (9.29, 253.8, 267.0),
            (9.21, 254.7, 268.0), (9.12, 255.6, 268.0), (9.04, 256.2, 269.0), (8.96, 256.5, 269.0),
            (8.86, 256.6, 269.0), (8.77, 256.6, 269.0), (8.68, 256.2, 268.0), (8.60, 255.7, 267.0),
            (8.52, 255.2, 267.0), (8.43, 254.7, 265.0), (8.34, 253.7, 264.0), (8.26, 252.9, 263.0),
            (8.18, 252.2, 262.0), (8.10, 251.8, 262.0), (8.01, 251.5, 261.0), (7.92, 251.4, 261.0),
            (7.84, 251.4, 261.0), (7.76, 251.0, 261.0), (7.68, 250.5, 261.0), (7.59, 249.9, 260.0),
            (7.51, 249.4, 259.0), (7.43, 248.8, 258.0), (7.34, 248.0, 257.0), (7.25, 247.0, 256.0),
            (7.17, 246.3, 255.0), (7.09, 245.8, 255.0), (7.01, 245.6, 255.0), (6.92, 245.9, 255.0),
            (6.84, 246.4, 256.0), (6.76, 247.1, 256.0), (6.68, 247.0, 256.0), (6.60, 246.3, 255.0),
            (6.51, 245.4, 254.0), (6.43, 244.5, 253.0), (6.35, 243.8, 252.0), (6.26, 242.9, 251.0),
            (6.19, 242.2, 250.0), (6.11, 241.5, 249.0), (6.03, 240.8, 248.0), (5.94, 240.0, 247.0),
            (5.86, 238.9, 246.0), (5.78, 238.1, 245.0), (5.70, 237.4, 245.0), (5.62, 237.0, 244.0),
            (5.54, 237.3, 245.0), (5.47, 237.8, 246.0), (5.39, 238.1, 246.0), (5.31, 237.9, 246.0),
            (5.23, 237.9, 246.0),
        ];
        let legs = vec![leg("HON", 1.0, 0.0), leg("OSKOT", 0.0, 0.0)];
        let cons = vec![crate::nav::StarConstraint {
            fix: "OSKOT".into(),
            min_ft: None,
            max_ft: None,
            max_kts: Some(210.0),
        }];
        let caps = |fresh: bool| {
            let mut begun = std::collections::HashSet::new();
            recorded
                .iter()
                .map(|&(d, ias, gs)| {
                    if fresh {
                        begun.clear();
                    }
                    arrival_speed_cap(&legs, &cons, 1, -d / 60.0, 0.0, ias, gs, &mut begun).map(|c| c.0)
                })
                .collect::<Vec<_>>()
        };
        let flips = |v: &[Option<f64>]| v.windows(2).filter(|w| w[0] != w[1]).count();
        let old = caps(true);
        assert!(flips(&old) >= 3, "the limit came and went: {old:?}");
        let new = caps(false);
        assert_eq!(flips(&new), 1, "{new:?}");
        assert_eq!(new.last(), Some(&Some(210.0)));
    }

    use super::*;

    #[test]
    fn missing_or_nonfinite_instruments_never_reach_the_laws() {
        for missing in 0..11 {
            let values = std::array::from_fn(|i| {
                if i == missing {
                    Err(anyhow::anyhow!("read failed"))
                } else {
                    Ok(100.0)
                }
            });
            assert!(measurements::<11>("procedure", values).is_none());
        }
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(measurements("procedure", [Ok(8000.0), Ok(invalid)]).is_none());
        }
        assert_eq!(
            measurements("procedure", [Ok(0.0), Ok(-100.0)]),
            Some([0.0, -100.0])
        );
    }

    // Inject failures at the actual write/read boundary, including a sim that
    // acknowledges PATCH but silently keeps its old dial value.
    struct DialIo {
        value: std::sync::Mutex<f64>,
        writes: std::sync::atomic::AtomicUsize,
        failure: std::sync::atomic::AtomicUsize,
    }

    #[async_trait::async_trait]
    impl crate::xplane::XPlane for DialIo {
        async fn get_float(&self, _: &str) -> anyhow::Result<f64> {
            if self.failure.load(Ordering::SeqCst) == 2 {
                anyhow::bail!("read failed");
            }
            Ok(*self.value.lock().unwrap())
        }
        async fn set_float(&self, _: &str, value: f64) -> anyhow::Result<()> {
            self.writes.fetch_add(1, Ordering::SeqCst);
            match self.failure.load(Ordering::SeqCst) {
                1 => anyhow::bail!("write failed"),
                3 => (),
                _ => *self.value.lock().unwrap() = value,
            }
            Ok(())
        }
        async fn set_int(&self, _: &str, _: i64) -> anyhow::Result<()> {
            unreachable!()
        }
        async fn run_command(&self, _: &str) -> anyhow::Result<()> {
            unreachable!()
        }
        async fn run_command_for(&self, _: &str, _: f64) -> anyhow::Result<()> {
            unreachable!()
        }
        async fn get_float_array(&self, _: &str) -> anyhow::Result<Vec<f64>> {
            unreachable!()
        }
        async fn set_int_array_elem(&self, _: &str, _: usize, _: i64) -> anyhow::Result<()> {
            unreachable!()
        }
        async fn set_float_array_elem(&self, _: &str, _: usize, _: f64) -> anyhow::Result<()> {
            unreachable!()
        }
    }

    /// LIPR 30 (2026-09-26): the dial written with the paced target every
    /// tick made a staircase; it is the level the movement ends at.
    #[test]
    fn the_altitude_dial_is_where_the_descent_ends_not_where_it_is_now() {
        // A descent from 2,100 toward a 1,500 constraint, paced: the target
        // walks down, the dial stays at 1,500 the whole way.
        for (want, alt) in [(1990.0, 2105.0), (1850.0, 2000.0), (1600.0, 1750.0)] {
            assert_eq!(super::dial_target_ft(want, alt, 1500.0, None, None), 1500.0);
        }
        // Held level before the top of descent: the level held.
        assert_eq!(super::dial_target_ft(4020.0, 4000.0, 1500.0, None, None), 4000.0);
        // A terrain floor above the constraint: never under the floor.
        assert_eq!(super::dial_target_ft(2340.0, 3000.0, 1500.0, Some(2340.0), None), 2400.0);
        // Climbing to a minimum above the plan's altitude: the minimum.
        assert_eq!(super::dial_target_ft(4000.0, 1200.0, 3000.0, None, None), 4000.0);
        // Down a published path: its bottom, once.
        assert_eq!(super::dial_target_ft(800.0, 900.0, 1500.0, None, Some(147.0)), 200.0);
    }

    #[test]
    fn a_minimum_above_the_cruise_is_not_the_followers_to_judge() {
        let m = |f: &str, a: f64| crate::nav::ClimbMinimum { fix: f.into(), min_ft: a };
        let (keep, above) = super::minimums_within_cruise(vec![m("RIM", 4000.0), m("D121Z", 9500.0)], 5000.0);
        assert_eq!(keep.len(), 1);
        assert_eq!(keep[0].fix, "RIM");
        assert_eq!(above[0].fix, "D121Z");
    }

    /// LIPR 2026-09-26: set the heading, leave it. One write per leg, a
    /// whole degree; small drift and a banked aircraft leave the knob alone.
    #[test]
    fn the_heading_is_set_once_per_leg_like_a_pilot_sets_it() {
        let mut h = HeadingHold::default();
        assert_eq!(h.decide(0, 127.4, 125.0, 0.0, 0.0), Some(127.0), "a new leg: set, whole degrees");
        for t in 1..60 {
            let wobble = 127.0 + (t as f64 * 0.7).sin() * 2.5;
            assert_eq!(h.decide(0, wobble, 127.0, 1.0, t as f64), None, "under 5 degrees: leave it");
        }
        assert_eq!(h.decide(0, 134.0, 127.0, 8.0, 70.0), None, "a correction waits for level wings");
        assert_eq!(h.decide(0, 134.0, 127.0, 1.0, 71.0), Some(134.0), "drifted 7, level: correct");
        assert_eq!(h.decide(0, 141.0, 134.0, 1.0, 80.0), None, "not twice inside 30 s");
        assert_eq!(h.decide(1, 250.0, 130.0, 0.0, 81.0), Some(250.0), "the next leg: set at once");
    }

    #[test]
    fn a_new_heading_against_the_bank_waits_for_the_roll_out() {
        let mut h = HeadingHold::default();
        // Banked 20 right, the new leg is 60 left: setting it now would send
        // the autopilot round the long way.
        assert_eq!(h.decide(3, 240.0, 300.0, 20.0, 0.0), None);
        assert_eq!(h.decide(3, 240.0, 305.0, 2.0, 5.0), Some(240.0), "level: set");
        // With the bank, it goes in at once.
        let mut h = HeadingHold::default();
        assert_eq!(h.decide(3, 340.0, 300.0, 20.0, 0.0), Some(340.0));
    }

    #[tokio::test]
    async fn failed_or_refused_dial_writes_retry_the_same_target_after_recovery() {
        for name in [ALT_DIAL, VS_DIAL] {
            for failure in [1, 2, 3] {
                let io = DialIo {
                    value: std::sync::Mutex::new(0.0),
                    writes: std::sync::atomic::AtomicUsize::new(0),
                    failure: std::sync::atomic::AtomicUsize::new(failure),
                };
                let mut last = Some(0.0);
                assert!(!set_confirmed("procedure", &io, name, 700.0, &mut last, 40.0).await);
                assert_eq!(last, None);
                io.failure.store(0, Ordering::SeqCst);
                assert!(set_confirmed("procedure", &io, name, 700.0, &mut last, 40.0).await);
                assert_eq!(*io.value.lock().unwrap(), 700.0);
                assert_eq!(io.writes.load(Ordering::SeqCst), 2);
                assert!(set_confirmed("procedure", &io, name, 700.0, &mut last, 40.0).await);
                assert_eq!(io.writes.load(Ordering::SeqCst), 2);
            }
        }
    }

    /// The one formula the follower's law, the advisor and the spoken answer
    /// all read: LEJR-LXGB, 7,000 ft to 27D10 at 3,143 ft, 113 kt over the
    /// ground, planned at 75% of the 700 fpm: 13.8 nm of descent plus the lead.
    #[test]
    fn descent_point_is_one_formula() {
        let tod = descent_point_nm(3_143.0, 7_000.0, 113.0, GA_VS_FPM).unwrap();
        assert!((tod - 14.84).abs() < 0.05, "{tod}");
        // The law starts down exactly there, not before.
        assert_eq!(
            descent_target_ft(3_143.0, 7_000.0, 7_000.0, Some(tod + 0.1), 113.0, GA_VS_FPM),
            7_000.0
        );
        // Inside it, down a PATH (2026-09-26), not straight to the
        // constraint: a little under the level, and the constraint itself
        // only at the fix's lead.
        let just_in = descent_target_ft(3_143.0, 7_000.0, 7_000.0, Some(tod - 0.1), 113.0, GA_VS_FPM);
        assert!(just_in < 7_000.0 && just_in > 6_900.0, "{just_in}");
        let halfway = descent_target_ft(3_143.0, 7_000.0, 5_200.0, Some((tod + 1.0) / 2.0), 113.0, GA_VS_FPM);
        assert!((halfway - 5_070.0).abs() < 50.0, "{halfway}");
        assert_eq!(
            descent_target_ft(3_143.0, 7_000.0, 3_300.0, Some(1.0), 113.0, GA_VS_FPM),
            3_143.0
        );
        // Nothing to descend to: no point to announce.
        assert_eq!(descent_point_nm(7_000.0, 7_050.0, 113.0, GA_VS_FPM), None);
        assert_eq!(descent_point_nm(8_000.0, 7_000.0, 113.0, GA_VS_FPM), None);
        // Between procedures the accessor is empty whatever was published.
        publish_tod(Some(PlannedTod {
            fix: "X".into(),
            dist_nm: 20.0,
            to_tod_nm: 8.0,
            target_ft: 3_000.0,
            started: false,
            held_ft: None,
        }));
        assert_eq!(planned_tod(), None);
        publish_tod(None);
    }
    #[test]
    fn a_pilot_held_altitude_is_reminded_once_and_never_called_a_missed_descent() {
        let t = |fix: &str, to: f64| PlannedTod {
            fix: fix.into(),
            dist_nm: to + 3.0,
            to_tod_nm: to,
            target_ft: 18_000.0,
            started: false,
            held_ft: None,
        };
        let mut held = None;
        // No pilot altitude: the plan's point, untouched.
        assert_eq!(held_tod(Some(t("IDREK", 5.0)), None, &mut held), Some(t("IDREK", 5.0)));
        // LIME-LIPE: 19,000 held over IDREK, ADOSA, OSTEG, BENUS.
        let a = held_tod(Some(t("IDREK", 20.0)), Some(19_000.0), &mut held).unwrap();
        assert_eq!(a.held_ft, Some(19_000.0));
        assert_eq!(held, None, "not yet in the reminder's window");
        let a = held_tod(Some(t("IDREK", 5.0)), Some(19_000.0), &mut held).unwrap();
        assert_eq!(a.held_ft, Some(19_000.0));
        assert_eq!(held_tod(Some(t("IDREK", -2.0)), Some(19_000.0), &mut held).unwrap().fix, "IDREK");
        assert_eq!(held_tod(Some(t("ADOSA", 4.0)), Some(19_000.0), &mut held), None, "said once");
        assert_eq!(held_tod(Some(t("OSTEG", 9.0)), Some(19_000.0), &mut held), None);
        // A new altitude from the pilot is a new hold.
        assert!(held_tod(Some(t("BENUS", 6.0)), Some(17_000.0), &mut held).is_some());
    }

    use crate::nav::Fix;

    /// The crew's "next waypoint" answer while a procedure flies: distance to
    /// the follower's OWN fix, minutes from the groundspeed, no ETA when we are
    /// not really moving (a parked aircraft is not 0 minutes from anywhere).
    #[test]
    fn leg_progress_is_distance_and_minutes_to_the_followers_fix() {
        // MG430 from the point the LXGB-LEMG flight was actually at when the
        // GPS still said "PIMOS, 48 miles".
        let leg = ActiveLeg {
            fix: "MG430".into(),
            lat: 36.9985,
            lon: -4.9035,
        };
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
        publish_leg(Some(ActiveLeg {
            fix: "X".into(),
            lat: 0.0,
            lon: 0.0,
        }));
        if !ACTIVE.load(Ordering::Relaxed) {
            assert_eq!(active_leg(), None);
        }
        publish_leg(None);
    }

    /// EGPH AGPE1E's shape, on a line north: HAVEN 250 kt, TARTN "FL070,
    /// 230 kt" after SimBrief's guess at 9,000, then 8,000 on the far side.
    fn star_route() -> (Vec<ProcLeg>, Vec<crate::nav::StarConstraint>) {
        let at = |id: &str, nm: f64, alt: f64| {
            let mut l = leg(id, nm / 60.0, 0.0);
            l.alt1_ft = Some(alt);
            l
        };
        let legs = vec![
            at("AGPED", 0.0, 26_000.0),
            at("HAVEN", 30.0, 20_000.0),
            at("MIDDL", 55.0, 15_000.0),
            at("TARTN", 70.0, 9_000.0),
            at("AFTER", 80.0, 8_000.0),
        ];
        let c = |fix: &str, min, max, kts| crate::nav::StarConstraint {
            fix: fix.into(),
            min_ft: min,
            max_ft: max,
            max_kts: kts,
        };
        let cons = vec![
            c("AGPED", Some(26_000.0), Some(26_000.0), None),
            c("HAVEN", None, None, Some(250.0)),
            c("TARTN", None, Some(7_000.0), Some(230.0)),
        ];
        (legs, cons)
    }

    #[test]
    fn a_stars_window_is_clamped_into_the_plan_and_never_climbed_back_out_of() {
        let (mut legs, cons) = star_route();
        assert_eq!(apply_arrival_constraints(&mut legs, &cons), 2);
        let alts: Vec<f64> = legs.iter().map(|l| l.alt1_ft.unwrap()).collect();
        assert_eq!(alts, vec![26_000.0, 20_000.0, 15_000.0, 7_000.0, 7_000.0]);
        // A minimum past the first restriction still raises its own fix.
        let mut legs2 = star_route().0;
        let mut cons2 = cons.clone();
        cons2.push(crate::nav::StarConstraint {
            fix: "AFTER".into(),
            min_ft: Some(7_500.0),
            max_ft: None,
            max_kts: None,
        });
        apply_arrival_constraints(&mut legs2, &cons2);
        assert_eq!(legs2[4].alt1_ft, Some(7_500.0));
    }

    #[test]
    fn a_restriction_two_fixes_on_brings_the_descent_forward() {
        let (mut legs, cons) = star_route();
        apply_arrival_constraints(&mut legs, &cons);
        // Planned at the path's own 75 % (2026-09-26), and on the path's
        // miles: the last one before TARTN is its lead, flown level.
        let grad = arrival_ft_per_nm(2_000.0, 300.0);
        assert!((grad - 300.0).abs() < 1.0, "{grad}");
        let c = arrival_ceiling_ft(&legs, &cons, 2, grad).unwrap();
        assert!((c - (7_000.0 + 14.0 * 300.0)).abs() < 30.0, "MIDDL {c}");
        // Further out it is the same line: 39 path miles from HAVEN, under
        // the plan's 20,000 at the planning rate (above it at the old 85 %)...
        let c = arrival_ceiling_ft(&legs, &cons, 1, grad).unwrap();
        assert!((c - (7_000.0 + 39.0 * 300.0)).abs() < 30.0, "HAVEN {c}");
        // ...and a fix's own "at" stands over it.
        assert_eq!(arrival_ceiling_ft(&legs, &cons, 0, grad), Some(26_000.0));
        assert_eq!(arrival_ceiling_ft(&legs, &cons, 4, grad), None, "nothing ahead");
    }

    #[test]
    fn the_plans_own_altitudes_ahead_bring_the_descent_forward() {
        // No STAR at all: the plan says 8,000 two fixes on.
        let (mut legs, _) = star_route();
        for (l, a) in legs.iter_mut().zip([19_000.0, 19_000.0, 16_600.0, 8_000.0, 8_000.0]) {
            l.alt1_ft = Some(a);
        }
        let grad = arrival_ft_per_nm(1_300.0, 200.0);
        let c = profile_ceiling_ft(&legs, 2, grad).unwrap();
        assert!(c < 16_600.0, "the fix before must already be coming down: {c}");
        // An approach's "at or above" is no ceiling.
        legs[3].alt_desc = '+';
        legs[4].alt_desc = '+';
        assert_eq!(profile_ceiling_ft(&legs, 2, grad), None);
        // Nothing ahead, nothing to keep.
        assert_eq!(profile_ceiling_ft(&legs, 4, grad), None);
    }

    /// Fly the loop's vertical bookkeeping down the route at 300 kt and
    /// 2,000 fpm: with the ceiling TARTN is crossed at FL070, without it
    /// (each leg paced only to its own fix) it is not.
    #[test]
    fn the_arrival_crosses_its_restriction_at_the_published_altitude() {
        fn fly(with_ceiling: bool) -> f64 {
            let (mut legs, cons) = star_route();
            apply_arrival_constraints(&mut legs, &cons);
            let (gs, rate) = (300.0, 2_000.0);
            let (mut lat, mut alt, mut idx) = (-5.0 / 60.0, 26_000.0, 0usize);
            let (mut carry, mut prev) = (alt, alt);
            loop {
                let f = legs[idx].fix.clone().unwrap();
                let d = crate::nav::haversine_nm(lat, 0.0, f.lat, f.lon);
                if d < 0.3 {
                    if legs[idx].fix_ident == "TARTN" {
                        return alt;
                    }
                    idx += 1;
                    continue;
                }
                let mut l = legs[idx].clone();
                if with_ceiling {
                    if let Some(c) = arrival_ceiling_ft(&legs, &cons, idx, arrival_ft_per_nm(rate, gs)) {
                        if c < l.alt1_ft.unwrap() - 50.0 {
                            l.alt1_ft = Some((c / 100.0).floor() * 100.0);
                        }
                    }
                }
                let v = vertical_tick(&l, carry, prev, alt, Some(d), gs, None, None, rate, DescentProfile::none());
                carry = v.carry;
                prev = v.prev_cmd;
                if v.want_alt < alt - 20.0 {
                    alt = (alt - rate / 60.0).max(v.want_alt);
                }
                lat += gs / 3600.0 / 60.0;
            }
        }
        let crossed = fly(true);
        assert!(crossed <= 7_100.0, "crossed TARTN at {crossed}");
        assert!(fly(false) > 8_000.0, "the test must bite without the ceiling");
    }

    /// LEMG's UNTO2A as the 737 flew it on 2026-09-26 (LIBP-LEMG, sortie
    /// 1790429680): each fix with the plan's altitude after the STAR clamp
    /// (the `star_ceiling` records' `plan_ft`), and the groundspeed and
    /// airspeed recorded on the leg TO it (first, last).
    #[allow(clippy::type_complexity)]
    fn lemg_unto2a() -> (Vec<ProcLeg>, Vec<crate::nav::StarConstraint>, Vec<((f64, f64), (f64, f64))>) {
        let fixes: [(&str, f64, f64, f64, (f64, f64), (f64, f64)); 17] = [
            ("TOSGA", 37.625555556, -2.307222222, 37_000.0, (422.0, 435.0), (252.0, 257.0)),
            ("ROLAS", 37.415555556, -2.854444444, 37_000.0, (435.0, 410.0), (257.0, 252.0)),
            ("UNTOS", 37.250555556, -3.277694444, 29_000.0, (410.0, 383.0), (252.0, 254.0)),
            ("NESDA", 36.821472222, -3.741722222, 28_720.0, (383.0, 363.0), (254.0, 247.0)),
            ("IBACO", 36.710527778, -3.927638889, 26_007.0, (364.0, 353.0), (248.0, 252.0)),
            ("OSENE", 36.710055556, -4.051972222, 24_551.0, (351.0, 342.0), (250.0, 250.0)),
            ("PIZCU", 36.709444444, -4.176333333, 23_094.0, (342.0, 339.0), (250.0, 253.0)),
            ("MG539", 36.839777778, -4.411083333, 19_752.0, (339.0, 295.0), (253.0, 228.0)),
            ("OBLAC", 36.839055556, -4.577166667, 14_000.0, (294.0, 291.0), (227.0, 228.0)),
            ("FOFUS", 36.932777778, -4.695500000, 10_000.0, (291.0, 282.0), (228.0, 228.0)),
            ("MG529", 36.999527778, -4.696138889, 10_000.0, (282.0, 252.0), (228.0, 212.0)),
            ("MG533", 37.066277778, -4.696777778, 10_000.0, (252.0, 247.0), (212.0, 209.0)),
            ("MG537", 37.133027778, -4.697444444, 10_000.0, (247.0, 246.0), (209.0, 208.0)),
            ("MG541", 37.199805556, -4.698083333, 10_000.0, (246.0, 247.0), (207.0, 207.0)),
            ("MG437", 37.199111111, -4.802388889, 10_000.0, (247.0, 263.0), (207.0, 212.0)),
            ("MG433", 37.132333333, -4.801666667, 10_000.0, (263.0, 253.0), (212.0, 212.0)),
            ("LENHI", 37.065583333, -4.800916667, 8_800.0, (253.0, 254.0), (212.0, 218.0)),
        ];
        let legs = fixes
            .iter()
            .map(|(id, lat, lon, alt, _, _)| {
                let mut l = leg(id, *lat, *lon);
                l.alt1_ft = Some(*alt);
                l
            })
            .collect();
        let speeds = fixes.iter().map(|f| (f.4, f.5)).collect();
        // The chart's windows as `plan_audit` `star_constraints` logged them.
        let c = |fix: &str, min: Option<f64>, max: Option<f64>, kts: Option<f64>| crate::nav::StarConstraint {
            fix: fix.into(),
            min_ft: min,
            max_ft: max,
            max_kts: kts,
        };
        let cons = vec![
            c("UNTOS", Some(14_000.0), Some(29_000.0), None),
            c("NESDA", Some(13_500.0), None, None),
            c("IBACO", Some(11_000.0), None, None),
            c("OSENE", Some(10_000.0), None, None),
            c("MG539", Some(8_500.0), None, Some(230.0)),
            c("OBLAC", Some(7_000.0), Some(14_000.0), Some(220.0)),
            c("FOFUS", Some(7_000.0), Some(10_000.0), Some(220.0)),
            c("MG529", None, None, Some(210.0)),
            c("MG533", None, None, Some(210.0)),
            c("MG537", None, None, Some(210.0)),
            c("MG541", None, None, Some(210.0)),
            c("MG437", None, None, Some(210.0)),
            c("MG433", None, None, Some(210.0)),
            c("LENHI", Some(7_000.0), None, Some(210.0)),
        ];
        (legs, cons, speeds)
    }

    /// The stock 737's airspeed in a clean descent, kt/s: fitted on sortie
    /// 1790601822-full_auto (EGKK-EGBB, +1700 to +3310, 1,022 state ticks,
    /// 0.33 kt/s rms) from the lever, the flight path angle, the speedbrake
    /// and the altitude. At idle it holds its speed at about 2.9 degrees
    /// down at 8,000 ft and loses 0.9 kt/s level.
    fn b738_accel_kts_s(p: &Plant, thr: f64, vs_fpm: f64, gs_kts: f64, brake: bool, alt_ft: f64) -> f64 {
        let down_deg = -(vs_fpm / (gs_kts.max(60.0) * 101.27)).atan().to_degrees();
        -0.554 + p.drag_kts_s + 3.028 * thr + 0.299 * p.gamma_scale * down_deg - if brake { 0.432 } else { 0.0 }
            - 0.402 * alt_ft / 10_000.0
    }

    /// The fitted plant, and how far a sweep moves it.
    #[derive(Debug, Clone, Copy)]
    struct Plant {
        /// Scales the flight path angle's effect on the airspeed.
        gamma_scale: f64,
        /// Added to the idle deceleration, kt/s (negative: more drag).
        drag_kts_s: f64,
        /// The VS mode's dead time, half-second ticks.
        dead_ticks: usize,
    }

    impl Default for Plant {
        fn default() -> Self {
            Plant { gamma_scale: 1.0, drag_kts_s: 0.0, dead_ticks: 4 }
        }
    }

    /// Which laws an [`fly_arrival`] run flies.
    #[derive(Clone, Copy, PartialEq)]
    enum ArrivalLaw {
        /// The descent as flown on 2026-09-28: no speed protection, the
        /// speedbrake out at 8 over, in at 3, the rate cut to 0.75 and 0.5
        /// past 12 and 20 over.
        AsFlown,
        /// `speed_protected_vs` and the height-keyed `descent_drag`.
        Protected,
    }

    #[derive(Debug, Default)]
    struct ArrivalRun {
        /// (fix, altitude, airspeed) as each leg sequenced.
        crossings: Vec<(String, f64, f64)>,
        max_vs: f64,
        /// The most height regained above the lowest reached, ft: a
        /// level-off's overshoot coming back, or a descent climbing.
        climb_back_ft: f64,
        /// Speedbrake extensions.
        brake_outs: usize,
    }

    /// Fly the follower's real vertical and speed (the ceilings,
    /// `vertical_tick`, the VS law, the STAR's speed limits, the 250 below
    /// ten thousand, the governor's `throttle_adjust`, the speed protection
    /// and the drag, the 100 fpm dial) down `legs` from `idx` against the
    /// fitted 737 ([`b738_accel_kts_s`]): the VS mode as the autoland
    /// sorties model it (two 1.5 s lags behind 2 s of dead time), the lever
    /// spooling on a 2 s lag, the loop ticking every half second.
    /// `planned(idx)` is the cruise or profile speed on each leg, `gs(idx,
    /// frac, tas)` the groundspeed, `capture_nm` how far out a leg sequences.
    #[allow(clippy::too_many_arguments)]
    fn fly_arrival(
        legs: &[ProcLeg],
        cons: &[crate::nav::StarConstraint],
        mut idx: usize,
        (mut lat, mut lon): (f64, f64),
        mut alt: f64,
        mut ias: f64,
        planned: impl Fn(usize) -> f64,
        gs_of: impl Fn(usize, f64, f64) -> f64,
        capture_nm: f64,
        law: ArrivalLaw,
        plant: Plant,
    ) -> ArrivalRun {
        const RATE: f64 = 2_000.0;
        let dt = 0.5;
        let (mut carry, mut prev) = (alt, alt);
        let mut latch = None;
        let (mut drag_out, mut drag_held) = (false, f64::INFINITY);
        let mut begun = std::collections::HashSet::new();
        let mut airspace = crate::tools::speedhold::AirspaceLimit::default();
        let (mut lag1, mut vs) = (0.0f64, 0.0f64);
        let mut dial_queue: std::collections::VecDeque<f64> = std::collections::VecDeque::new();
        let mut run = ArrivalRun { max_vs: f64::MIN, ..Default::default() };
        let start = legs[idx - 1].fix.clone().unwrap();
        let mut leg_len = crate::nav::haversine_nm(start.lat, start.lon, legs[idx].fix.as_ref().unwrap().lat, legs[idx].fix.as_ref().unwrap().lon);
        let tas = |ias: f64, alt: f64| ias * (1.0 + 0.02 * alt / 1_000.0);
        let (mut thr, mut thrust) = (0.3f64, 0.3f64);
        let (mut trend, mut prev_ias) = (0.0f64, ias);
        let mut idle = IdleDescent::default();
        let mut low = alt;
        for _ in 0..40_000 {
            let f = legs[idx].fix.clone().unwrap();
            let d = crate::nav::haversine_nm(lat, lon, f.lat, f.lon);
            if d < capture_nm {
                run.crossings.push((legs[idx].fix_ident.clone(), alt, ias));
                idx += 1;
                if idx == legs.len() {
                    break;
                }
                let n = legs[idx].fix.clone().unwrap();
                leg_len = crate::nav::haversine_nm(lat, lon, n.lat, n.lon);
                continue;
            }
            let frac = (1.0 - d / leg_len).clamp(0.0, 1.0);
            let gs = gs_of(idx, frac, tas(ias, alt));
            // The speed the governor holds: the leg's, the STAR's limits,
            // the 250 below ten thousand.
            let mut target = planned(idx);
            if let Some((kts, _)) = arrival_speed_cap(legs, cons, idx, lat, lon, ias, gs, &mut begun) {
                target = target.min(if law == ArrivalLaw::Protected { kts - STAR_SPEED_MARGIN_KTS } else { kts });
            }
            let target = airspace.limit(alt, target);
            // The governor's own target: the speed owed, or under it at
            // idle in a descent (`IdleDescent`, the last tick's verdict).
            let governed = if law == ArrivalLaw::Protected { idle.governor_kts(target, None) } else { target };
            // The vertical.
            let mut l = legs[idx].clone();
            if let Some((a, _)) = ceilinged_leg_alt(legs, cons, idx, gs, RATE, &mut latch) {
                l.alt1_ft = Some(a);
            }
            let v = vertical_tick(&l, carry, prev, alt, Some(d), gs, None, None, RATE, DescentProfile::none());
            carry = v.carry;
            prev = v.prev_cmd;
            let want = vs_law_fpm(v.want_alt, alt, v.target_fpm, RATE, 1_500.0);
            let (cmd, out) = match law {
                ArrivalLaw::AsFlown => {
                    let over = ias - target;
                    let settled = drag_held >= DRAG_MIN_HOLD_S;
                    let out = if drag_out {
                        want < DRAG_KEEP_BELOW_FPM && (over > DRAG_IN_KTS || !settled)
                    } else {
                        want <= -DRAG_MIN_DESCENT_FPM && over > DRAG_OUT_KTS && settled
                    };
                    let scale = if want > -DRAG_MIN_DESCENT_FPM {
                        1.0
                    } else if over > 20.0 {
                        0.5
                    } else if over > 12.0 {
                        0.75
                    } else {
                        1.0
                    };
                    (want * scale, out)
                }
                ArrivalLaw::Protected => {
                    idle.step(want);
                    (
                        speed_protected_vs(want, Some(ias - target), trend),
                        descent_drag(drag_out, drag_held, ias, Some(target), want, alt - v.want_alt),
                    )
                }
            };
            if out && !drag_out {
                run.brake_outs += 1;
            }
            drag_held = if out == drag_out { drag_held + dt } else { 0.0 };
            drag_out = out;
            dial_queue.push_back(vs_dial_fpm(cmd));
            // 2 s of dead time at the half-second tick.
            let dial = if dial_queue.len() > plant.dead_ticks { dial_queue.pop_front().unwrap() } else { 0.0 };
            // The governor's tick, on the smoothed trend.
            thr = (thr + crate::tools::speedhold::throttle_adjust(governed - ias, trend)).clamp(0.0, 1.0);
            for _ in 0..10 {
                let h = dt / 10.0;
                lag1 += (dial - lag1) * h / 1.5;
                vs += (lag1 - vs) * h / 1.5;
                alt += vs * h / 60.0;
                thrust += (thr - thrust) * h / 2.0;
                ias += b738_accel_kts_s(&plant, thrust, vs, gs, drag_out, alt) * h;
            }
            trend += ((ias - prev_ias) / dt - trend) * dt / (IAS_TREND_TAU_S + dt);
            prev_ias = ias;
            run.max_vs = run.max_vs.max(vs);
            low = low.min(alt);
            run.climb_back_ft = run.climb_back_ft.max(alt - low);
            let brg = crate::nav::bearing_deg(lat, lon, f.lat, f.lon);
            (lat, lon) = crate::nav::project_latlon(lat, lon, brg, gs * dt / 3600.0);
        }
        run
    }

    /// The recorded UNTO2A from the ROLAS leg at FL370 (LIBP-LEMG,
    /// 2026-09-26), sequenced 1.2 nm out (FOFUS's recorded capture), the
    /// recorded groundspeed on each leg (the wind), flown closed loop.
    /// Returns the altitude at each fix and the highest vertical speed.
    fn fly_unto2a() -> (Vec<(String, f64)>, f64, f64) {
        let (legs, cons, speeds) = lemg_unto2a();
        let first = legs[0].fix.clone().unwrap();
        let run = fly_arrival(
            &legs,
            &cons,
            1,
            (first.lat, first.lon),
            37_000.0,
            257.0,
            |_| 255.0,
            |i, frac, tas| {
                // The recorded groundspeed less the recorded airspeed's TAS:
                // the wind, carried at the airspeed flown here.
                let ((g0, g1), (i0, i1)) = speeds[i];
                let (g, rec) = (g0 + (g1 - g0) * frac, i0 + (i1 - i0) * frac);
                let alt = legs[i].alt1_ft.unwrap_or(20_000.0);
                tas + g - rec * (1.0 + 0.02 * alt / 1_000.0)
            },
            1.2,
            ArrivalLaw::Protected,
            Plant::default(),
        );
        (run.crossings.into_iter().map(|(f, a, _)| (f, a)).collect(), run.max_vs, run.climb_back_ft)
    }

    /// EGBB's HEME1B as the 737 flew it on 2026-09-28 (sortie
    /// 1790601822-full_auto): LAM at the FL180 cruise, then the STAR's
    /// fixes (earth_fix.dat, HON the VOR), the navlog's straight line from
    /// the cruise at HEMEL to GROVE's FL070 (`plan_audit::realtitude_star`'s
    /// rule) under the windows `procedure` `star_constraints` logged.
    fn egbb_heme1b() -> (Vec<ProcLeg>, Vec<crate::nav::StarConstraint>) {
        let fixes: [(&str, f64, f64); 8] = [
            ("LAM", 51.646, 0.151667),
            ("HEMEL", 51.805555556, -0.419383333),
            ("BUZAD", 51.942244444, -0.552280556),
            ("WELIN", 52.247291667, -0.852336111),
            ("PUFAX", 52.314447222, -1.345055556),
            ("HON", 52.356677778, -1.663725000),
            ("OSKOT", 52.341655556, -2.058408333),
            ("GROVE", 52.393822222, -1.928811111),
        ];
        let mut legs: Vec<ProcLeg> = fixes.iter().map(|(id, la, lo)| leg(id, *la, *lo)).collect();
        legs[0].alt1_ft = Some(18_000.0);
        let star = crate::plan_audit::realtitude_star(legs[1..].to_vec(), Some(18_000.0), Some(7_000.0), 1);
        legs.truncate(1);
        legs.extend(star);
        let c = |fix: &str, min: Option<f64>, max: Option<f64>, kts: Option<f64>| crate::nav::StarConstraint {
            fix: fix.into(),
            min_ft: min,
            max_ft: max,
            max_kts: kts,
        };
        let cons = vec![
            c("HEMEL", Some(22_000.0), Some(22_000.0), None),
            c("PUFAX", None, None, Some(250.0)),
            c("HON", Some(9_000.0), Some(9_000.0), None),
            c("OSKOT", None, None, Some(210.0)),
            c("GROVE", Some(7_000.0), Some(7_000.0), Some(210.0)),
        ];
        (legs, cons)
    }

    /// The HEME1B from BUZAD at FL180 and 302 kt, as recorded at +1995: the
    /// OFP's cruise on WELIN (299, or the capped 296), the profile's 255
    /// after, no wind (the recorded groundspeed at FL180 was within 10 kt of
    /// the TAS). Sequenced at the recorded 0.9 nm.
    fn fly_heme1b(law: ArrivalLaw, plant: Plant) -> ArrivalRun {
        let (mut legs, cons) = egbb_heme1b();
        apply_arrival_constraints(&mut legs, &cons);
        let welin_kts = if law == ArrivalLaw::AsFlown { 299.0 } else { 296.0 };
        let buzad = legs[2].fix.clone().unwrap();
        fly_arrival(
            &legs,
            &cons,
            3,
            (buzad.lat, buzad.lon),
            18_031.0,
            302.0,
            |i| if i == 3 { welin_kts } else { 255.0 },
            |_, _, tas| tas - 10.0,
            0.9,
            law,
            plant,
        )
    }

    /// #156 (and #32's windows): PUFAX 254 against 250, OSKOT 216 and GROVE
    /// 218 against 210, six speedbrake extensions down the HEME1B, each at
    /// 8 kt over. Flown on the fitted 737, the old laws repeat the busts;
    /// the protected descent crosses every limit within 2 kt, every window
    /// within 300 ft, and needs the brake at most twice.
    #[test]
    fn the_heme1b_is_flown_at_its_speeds_with_the_brake_for_height() {
        let old = fly_heme1b(ArrivalLaw::AsFlown, Plant::default());
        let new = fly_heme1b(ArrivalLaw::Protected, Plant::default());
        let at = |run: &ArrivalRun, fix: &str| {
            run.crossings.iter().find(|c| c.0 == fix).cloned().unwrap_or_else(|| panic!("{fix}: {:?}", run.crossings))
        };
        let busts = |run: &ArrivalRun| {
            [("PUFAX", 250.0), ("OSKOT", 210.0), ("GROVE", 210.0)]
                .iter()
                .filter(|(f, kts)| at(run, f).2 > kts + 2.0)
                .count()
        };
        assert!(busts(&old) >= 1 || old.brake_outs > 2, "the test must bite: {old:?}");
        assert_eq!(busts(&new), 0, "{new:?}");
        assert!(new.brake_outs <= 2, "{} extensions: {new:?}", new.brake_outs);
        let hon = at(&new, "HON").1;
        assert!((hon - 9_000.0).abs() <= 300.0, "HON at {hon:.0}: {new:?}");
        let grove = at(&new, "GROVE").1;
        assert!((grove - 7_000.0).abs() <= 300.0, "GROVE at {grove:.0}: {new:?}");
        assert!(new.climb_back_ft < 100.0, "{:.0} ft regained: {new:?}", new.climb_back_ft);
        // Not a fit to one plant: the flight path's effect 25 % either way,
        // the idle deceleration 0.15 kt/s either way, the VS mode's dead
        // time 1 to 3 s. Every limit within 3 kt, the brake rare.
        let (mut worst, mut outs) = ((0.0f64, String::new()), 0usize);
        for gamma_scale in [0.75, 1.0, 1.25] {
            for drag_kts_s in [-0.15, 0.0, 0.15] {
                for dead_ticks in [2, 4, 6] {
                    let plant = Plant { gamma_scale, drag_kts_s, dead_ticks };
                    let run = fly_heme1b(ArrivalLaw::Protected, plant);
                    for (fix, kts) in [("PUFAX", 250.0), ("OSKOT", 210.0), ("GROVE", 210.0)] {
                        let over = at(&run, fix).2 - kts;
                        if over > worst.0 {
                            worst = (over, format!("{fix} on {plant:?}: {run:?}"));
                        }
                    }
                    outs = outs.max(run.brake_outs);
                }
            }
        }
        assert!(worst.0 <= 3.0, "{:.1} over at {}", worst.0, worst.1);
        assert!(outs <= 4, "{outs} extensions");
    }

    /// FOFUS on UNTO2A is 7,000 to 10,000 ft and the 737 crossed it at 10,386
    /// (2026-09-26): every fix's target was held a few miles into its leg
    /// against the last one (never reached), the proportional VS law trailed
    /// the path by 500 ft, and MG539's ceiling rose 600 ft as the aircraft
    /// slowed and the descent climbed back. Replayed on the recorded legs,
    /// closed loop on the fitted 737 since #156 (the speed flown, not the
    /// speed recorded): every window within the card's 300 ft slack, FOFUS
    /// within 200 ft of its top as the leg sequences 1.2 nm out (the path
    /// reaches a constraint 1 nm before its fix), and nothing climbs. The
    /// old laws on the same plant climbed at 293 fpm.
    #[test]
    fn the_recorded_unto2a_crosses_fofus_inside_its_window() {
        let (crossings, max_vs, climb_back) = fly_unto2a();
        let (_, cons, _) = lemg_unto2a();
        let at = |fix: &str| crossings.iter().find(|(f, _)| f == fix).map(|c| c.1).unwrap();
        let fofus = at("FOFUS");
        assert!((7_000.0..=10_200.0).contains(&fofus), "FOFUS at {fofus:.0}: {crossings:?}");
        // A level-off's overshoot comes back; a descent never climbs (MG539
        // regained 600 ft).
        assert!(climb_back < 100.0, "{climb_back:.0} ft regained: {crossings:?}");
        for c in &cons {
            let alt = at(&c.fix);
            assert!(c.min_ft.is_none_or(|m| alt >= m - 300.0), "{} at {alt:.0}", c.fix);
            assert!(c.max_ft.is_none_or(|m| alt <= m + 300.0), "{} at {alt:.0}", c.fix);
        }
        assert!(max_vs < 250.0, "the descent climbed at {max_vs:.0} fpm: {crossings:?}");
    }

    /// EGBB I15 as the CIFP codes it (IF CF15, CF FF15 at 2,500 ft, CF RW15),
    /// resolved (earth_fix.dat; the threshold from apt.dat).
    fn egbb_i15() -> Vec<ProcLeg> {
        let rows = "APPCH:010,I,I15, ,CF15,EG,P,C,E  I, ,   ,IF, ,IBIR,EG,P,I,      ,3248,0106,    ,    ,J,02500,02500,     , ,   ,    ,   , , , , , ,0,D,S;\n\
                    APPCH:020,I,I15, ,FF15,EG,P,C,E  F, ,   ,CF, ,IBIR,EG,P,I,      ,3248,0083,1450,0023,H,02500,02500,     , ,   ,-300,   ,BHX,EG,D,B, ,0,D,S;\n\
                    APPCH:030,I,I15, ,RW15,EG,P,G,G  M, ,   ,CF, ,IBIR,EG,P,I,      ,3248,0016,1450,0067, ,00358,     ,     , ,   ,-300,   , , , , , ,0,D,S;\n";
        let raw = crate::nav::parse_proc_legs(rows.lines().map(str::trim), crate::nav::ProcKind::Appch, "I15", None);
        let mut legs = crate::nav::straight_in_legs(raw, "15").expect("straight in");
        let at = |ident: &str, lat: f64, lon: f64| Fix { ident: ident.into(), lat, lon };
        legs[0].fix = Some(at("CF15", 52.587216667, -1.896825));
        legs[1].fix = Some(at("FF15", 52.555586111, -1.861436111));
        legs[2].fix = Some(at("RW15", 52.4656497, -1.7611282));
        legs
    }

    #[derive(Debug, Default)]
    struct JoinRun {
        /// Seconds from GROVE to the FAF's capture (the autoland handoff).
        secs: f64,
        alt: f64,
        ias: f64,
        brake_outs: usize,
        /// The fastest after the first minute (the join's own, not GROVE's).
        max_ias: f64,
        /// The fastest while descending.
        max_descent_ias: f64,
        /// Miles to the FAF at its capture.
        faf_nm: f64,
        /// Miles before the FAF the final was first established (300 m, 10
        /// degrees).
        established_nm: Option<f64>,
        /// The steers in the order flown.
        steers: Vec<&'static str>,
    }

    /// The approach armed at GROVE (+2862, sortie 1790601822-full_auto):
    /// 7,022 ft, 217 kt, tracking 119 at 242 kt over the ground, abeam
    /// runway 15's threshold 7.6 nm to the right of the final. The follower's
    /// lateral (`FinalJoin`'s pattern, the course legs, the heading set as a
    /// pilot sets it, a 25 degree autopilot), its vertical paced on
    /// `pattern_nm` or, as flown, on the straight line to the IF, and the
    /// approach's 210.25 kt on the fitted 737 ([`fly_arrival`]'s plant and
    /// laws), to the FAF's capture.
    fn fly_egbb_join(law: ArrivalLaw) -> JoinRun {
        fly_egbb_join_from(&GROVE_1790601822, law, JoinGeometry::AsFlown)
    }

    /// Where the approach armed at GROVE: position, track, altitude, airspeed,
    /// and the wind's share of the groundspeed, kt.
    struct JoinStart {
        lat: f64,
        lon: f64,
        hdg: f64,
        alt: f64,
        ias: f64,
        wind_kts: f64,
    }

    /// Sortie 1790601822-full_auto, +2862.
    const GROVE_1790601822: JoinStart = JoinStart {
        lat: 52.403079551600634,
        lon: -1.9394949225789109,
        hdg: 118.6,
        alt: 7_021.5,
        ias: 217.1,
        wind_kts: 25.0,
    };

    /// Sortie 1790635335-full_auto, +790.0 (t 1790636125): 7,185 ft, 209 kt,
    /// tracking 116 at 226 kt over the ground; the procedure's first tick put
    /// it 5.9 nm past the FAF, 7.7 nm right of the final. As flown the FAF
    /// was captured 466 s later (+1256), established 6.7 nm before it.
    const GROVE_1790635335: JoinStart = JoinStart {
        lat: 52.40216798594475,
        lon: -1.946318684047718,
        hdg: 116.0,
        alt: 7_185.3,
        ias: 208.9,
        wind_kts: -13.0,
    };

    /// Sortie 1790624015-full_auto, +2215.1 (t 1790626230): 4,797 ft, 204 kt,
    /// tracking 104 at 209 kt; 6.1 nm past the FAF, 7.9 nm right. As flown
    /// the FAF was captured 429 s later (+2644) at 211.5 kt, 2,899 ft.
    const GROVE_1790624015: JoinStart = JoinStart {
        lat: 52.39743536948908,
        lon: -1.9473286705380015,
        hdg: 103.9,
        alt: 4_796.7,
        ias: 204.3,
        wind_kts: -15.0,
    };

    /// Which base and skip the join flies.
    #[derive(Clone, Copy, PartialEq, Debug)]
    enum JoinGeometry {
        /// 2026-09-28: the base two sizing radii beyond the established
        /// point, the skip on the old room.
        AsFlown,
        /// #161: the base sized on the offset and the radius.
        Sized,
    }

    fn fly_egbb_join_from(start: &JoinStart, law: ArrivalLaw, geometry: JoinGeometry) -> JoinRun {
        use crate::course_leg::{course_passed, course_track, CourseLine, FinalJoin, Join, Steer};
        const VAR: f64 = -0.9;
        const RATE: f64 = 2_000.0;
        let legs = egbb_i15();
        let plant = Plant::default();
        let (mut lat, mut lon) = (start.lat, start.lon);
        let (mut hdg, mut bank) = (start.hdg, 0.0f64);
        let (mut alt, mut ias) = (start.alt, start.ias);
        let target = crate::autoland::clean_approach_ias(145.0);
        let mut join = match geometry {
            JoinGeometry::AsFlown => FinalJoin::as_flown(),
            JoinGeometry::Sized => FinalJoin::default(),
        };
        let mut hold = HeadingHold::default();
        let mut bug_true = hdg;
        let (mut idx, mut was) = (0usize, Steer::Direct);
        let (mut carry, mut prev) = (alt, alt);
        let (mut drag_out, mut drag_held) = (false, f64::INFINITY);
        let (mut lag1, mut vs) = (-464.0f64, -464.0f64);
        let mut dial_queue: std::collections::VecDeque<f64> = std::collections::VecDeque::new();
        let (mut thr, mut thrust) = (0.0f64, 0.0f64);
        let (mut trend, mut prev_ias) = (0.0f64, ias);
        let mut idle = IdleDescent::default();
        let mut run = JoinRun::default();
        let dt = 0.5;
        for tick in 0..6_000 {
            let now = tick as f64 * dt;
            let tas = ias * (1.0 + 0.02 * alt / 1_000.0);
            let gs = tas + start.wind_kts;
            let leg = &legs[idx];
            let (mut steer, mut track) = (Steer::Direct, None);
            match join.step(&legs, idx, lat, lon, hdg, gs, VAR, alt, RATE) {
                Some(Join::Skip { .. }) => {
                    idx += 1;
                    continue;
                }
                Some(Join::Pattern { track_true, leg, .. }) => {
                    steer = Steer::Pattern(leg);
                    track = Some(track_true);
                }
                _ => {}
            }
            let line = CourseLine::of_leg(leg, VAR);
            if track.is_none() {
                if let Some((t, s)) = line.as_ref().and_then(|l| course_track(l, lat, lon, hdg, gs, was == Steer::Track)) {
                    track = Some(t);
                    steer = s;
                }
            }
            let f = leg.fix.clone().unwrap();
            let d = crate::nav::haversine_nm(lat, lon, f.lat, f.lon);
            let track = track.unwrap_or_else(|| crate::nav::bearing_deg(lat, lon, f.lat, f.lon));
            was = steer;
            if run.steers.last() != Some(&steer.word()) {
                run.steers.push(steer.word());
            }
            if run.established_nm.is_none() {
                let faf = CourseLine::of_leg(&legs[1], VAR).unwrap();
                let (along, xtk_m) = (faf.along_nm(lat, lon), faf.xtk_nm(lat, lon) * 1852.0);
                let off = ((hdg - faf.course_true() + 540.0).rem_euclid(360.0) - 180.0).abs();
                if along > 0.0 && xtk_m.abs() < 300.0 && off < 10.0 {
                    run.established_nm = Some(along);
                }
            }
            let mag = |t: f64| crate::autoland::true_to_mag(t, VAR);
            if let Some(h) = hold.decide(steer.key(idx), mag(track), mag(hdg), bank, now) {
                bug_true = h - VAR;
            }
            let cap = capture_radius_nm(gs, leg_turn_deg(lat, lon, leg, legs.get(idx + 1)).unwrap_or(0.0));
            let done = leg_done(leg, Some(d), alt, cap, lat, lon)
                || line.as_ref().is_some_and(|l| course_passed(l, lat, lon, cap));
            if done {
                if idx == 1 {
                    (run.secs, run.alt, run.ias, run.faf_nm) = (now, alt, ias, d);
                    return run;
                }
                idx += 1;
                continue;
            }
            // The vertical: the pattern's miles, or (as flown) the line; a
            // skipped IF still ahead keeps its own altitude and distance.
            let owed_if = join.skipped_if_ahead_nm(&legs, idx, lat, lon, VAR);
            let dist = match (law, owed_if) {
                (ArrivalLaw::Protected, Some((_, d_if))) => Some(d_if),
                (ArrivalLaw::Protected, None) => {
                    join.pattern_to_go_nm(&legs, idx, lat, lon, gs, VAR).or(Some(d))
                }
                (ArrivalLaw::AsFlown, _) => Some(d),
            };
            let vleg = owed_if.map_or(leg, |(i, _)| &legs[i]);
            let v = vertical_tick(
                vleg, carry, prev, alt, dist, gs, None, None, RATE, DescentProfile::none(),
            );
            carry = v.carry;
            prev = v.prev_cmd;
            let want = vs_law_fpm(v.want_alt, alt, v.target_fpm, RATE, 1_500.0);
            let governed = if law == ArrivalLaw::Protected { idle.governor_kts(target, None) } else { target };
            let (cmd, out) = match law {
                ArrivalLaw::AsFlown => {
                    let over = ias - target;
                    let settled = drag_held >= DRAG_MIN_HOLD_S;
                    let out = if drag_out {
                        want < DRAG_KEEP_BELOW_FPM && (over > DRAG_IN_KTS || !settled)
                    } else {
                        want <= -DRAG_MIN_DESCENT_FPM && over > DRAG_OUT_KTS && settled
                    };
                    (want, out)
                }
                ArrivalLaw::Protected => {
                    idle.step(want);
                    (
                        speed_protected_vs(want, Some(ias - target), trend),
                        descent_drag(drag_out, drag_held, ias, Some(target), want, alt - v.want_alt),
                    )
                }
            };
            if out && !drag_out {
                run.brake_outs += 1;
            }
            drag_held = if out == drag_out { drag_held + dt } else { 0.0 };
            drag_out = out;
            dial_queue.push_back(vs_dial_fpm(cmd));
            let dial = if dial_queue.len() > plant.dead_ticks { dial_queue.pop_front().unwrap() } else { vs };
            thr = (thr + crate::tools::speedhold::throttle_adjust(governed - ias, trend)).clamp(0.0, 1.0);
            for _ in 0..10 {
                let h = dt / 10.0;
                lag1 += (dial - lag1) * h / 1.5;
                vs += (lag1 - vs) * h / 1.5;
                alt += vs * h / 60.0;
                thrust += (thr - thrust) * h / 2.0;
                ias += b738_accel_kts_s(&plant, thrust, vs, gs, drag_out, alt) * h;
            }
            trend += ((ias - prev_ias) / dt - trend) * dt / (IAS_TREND_TAU_S + dt);
            prev_ias = ias;
            if now > 60.0 {
                run.max_ias = run.max_ias.max(ias);
            }
            if vs < -500.0 {
                run.max_descent_ias = run.max_descent_ias.max(ias);
            }
            // The heading mode: a rate-limited roll to 25 degrees, the short way.
            let err = ((bug_true - hdg + 540.0).rem_euclid(360.0)) - 180.0;
            let want_bank = (err * 1.5).clamp(-25.0, 25.0);
            bank += (want_bank - bank).clamp(-5.0 * dt, 5.0 * dt);
            let v_ms = gs * 0.514_444;
            hdg = (hdg + (9.81 * bank.to_radians().tan() / v_ms).to_degrees() * dt).rem_euclid(360.0);
            (lat, lon) = crate::nav::project_latlon(lat, lon, hdg, gs * dt / 3600.0);
        }
        panic!("never reached the FAF: {run:?}");
    }

    /// #156, #126: the join from GROVE held a 4,011 ft plateau round the
    /// pattern (its path to the IF paced on the straight line), came down
    /// the last 4 nm at up to 2,240 fpm with the speedbrake out three times
    /// around 210.25, and was handed to autoland at 222 kt. Flown on the
    /// pattern's miles at idle with the speed on the pitch, it reaches the
    /// 2,500 ft platform before the FAF and is handed over at the approach
    /// speed, the brake needed at most once.
    #[test]
    fn the_join_from_grove_arrives_at_the_faf_at_its_speed_and_height() {
        let old = fly_egbb_join(ArrivalLaw::AsFlown);
        let new = fly_egbb_join(ArrivalLaw::Protected);
        // As flown: 218 kt at the handoff, four extensions (the flight: 222,
        // three).
        assert!(old.ias > 215.0 && old.brake_outs >= 2, "the test must bite: {old:?}");
        assert!(new.ias <= 213.0, "{new:?}");
        assert!(new.max_descent_ias <= 215.0 && old.max_descent_ias > 218.0, "{new:?} {old:?}");
        // At the FAF's capture on its 2,500 ft platform or on the way to it,
        // under the glideslope (3 degrees, 318 ft a mile, through 2,500 ft
        // at the FAF): the ILS is joined from below.
        assert!(new.alt >= 2_400.0 && new.alt <= 2_500.0 + new.faf_nm * 318.0, "{new:?}");
        assert!(new.brake_outs <= 1, "{new:?}");
        // The pattern is the pattern: the join is no longer for it.
        assert!(new.secs <= old.secs + 30.0, "{new:?} {old:?}");
    }

    /// #161: the two recorded joins from GROVE, replayed on the geometry
    /// they flew: the FAF captured within 20 s of the recorded 466 s
    /// (1790635335) and 429 s (1790624015), established 5 nm or more before
    /// it, and handed over high (the flight: 2,899 ft at 0.78 nm).
    #[test]
    fn the_recorded_joins_from_grove_replay_their_times() {
        for (start, recorded) in [(&GROVE_1790635335, 466.0), (&GROVE_1790624015, 429.0)] {
            let r = fly_egbb_join_from(start, ArrivalLaw::Protected, JoinGeometry::AsFlown);
            assert!((r.secs - recorded).abs() <= 20.0, "{recorded} s recorded: {r:?}");
            assert!(r.established_nm.is_some_and(|e| e > 5.0), "{r:?}");
            assert!(r.alt > 2_500.0 + r.faf_nm * 318.0, "over the glideslope as flown: {r:?}");
        }
    }

    /// #161: the join sized on the downwind's offset and the turn radius,
    /// the intercept's turn counted as flown, and the skipped IF's 2,500 ft
    /// still owed while it lies ahead: from each recorded GROVE state the
    /// FAF is captured sooner, still established 2 nm before it off a
    /// base, on the 2,500 ft platform and at the approach's 210 kt.
    #[test]
    fn the_join_from_grove_is_shorter_and_hands_over_on_the_platform() {
        let mut saved = 0.0;
        for start in [&GROVE_1790601822, &GROVE_1790635335, &GROVE_1790624015] {
            let old = fly_egbb_join_from(start, ArrivalLaw::Protected, JoinGeometry::AsFlown);
            let new = fly_egbb_join_from(start, ArrivalLaw::Protected, JoinGeometry::Sized);
            assert!(new.secs <= old.secs - 30.0, "{new:?} {old:?}");
            saved += old.secs - new.secs;
            let flown = ["pattern_outbound", "pattern_base", "intercept", "track"];
            assert_eq!(new.steers, flown, "{new:?}");
            let est = new.established_nm.expect("established before the FAF");
            assert!(est >= 2.0, "{new:?}");
            assert!((2_450.0..=2_600.0).contains(&new.alt), "on the platform: {new:?}");
            assert!(new.ias <= 215.0, "{new:?}");
        }
        assert!(saved >= 150.0, "{saved:.0} s saved over three joins");
    }

    /// #156 / #32: HEMEL's "at FL220" in the FL180 plan is flown at the
    /// cruise, said once at the arm; from the HEMEL leg as recorded (+1661.6:
    /// 6,350 ft, 23.4 nm, 273 kt, held at 6,000 until LAM by the clearance)
    /// even the 2,000 fpm climb cannot make 18,000, and that is said once
    /// with the number, where the flight crossed it at 14,638 unsaid.
    #[test]
    fn a_star_window_that_cannot_be_made_is_said() {
        let (mut legs, cons) = egbb_heme1b();
        apply_arrival_constraints(&mut legs, &cons);
        assert_eq!(legs[1].fix_ident, "HEMEL");
        assert_eq!(legs[1].alt1_ft, Some(18_000.0), "never climbed to FL220");
        assert!(legs.iter().all(|l| l.alt1_ft.unwrap() <= 18_000.0));
        let cruise = legs.iter().filter_map(|l| l.alt1_ft).fold(0.0, f64::max);
        assert_eq!(windows_above_cruise(&legs, &cons, cruise), vec![("HEMEL".to_string(), 22_000.0, 18_000.0)]);
        assert_eq!(
            above_cruise_line("HEMEL", 22_000.0, 18_000.0),
            "HEMEL's 22,000 feet is above our cruise. We'll cross it at 18,000 feet."
        );
        // A plan that starts on the STAR has no cruise before it: as before.
        let (legs2, cons2) = star_route();
        let mut l2 = legs2.clone();
        apply_arrival_constraints(&mut l2, &cons2);
        assert_eq!(l2[0].alt1_ft, Some(26_000.0));
        // On the LAM leg, 29 nm out: the climb still makes it.
        let hemel = legs[1].fix.clone().unwrap();
        let (lat, lon) = crate::nav::project_latlon(hemel.lat, hemel.lon, 110.0, 23.4);
        let lam = legs[0].fix.clone().unwrap();
        assert_eq!(
            star_window_miss(&legs, &cons, 0, lam.lat + 0.09, lam.lon, 6_079.0, 280.0, 2_000.0, 2_000.0, Some(cruise)),
            None
        );
        // On the HEMEL leg as recorded: 1,300 ft low even at the best.
        let miss = star_window_miss(&legs, &cons, 1, lat, lon, 6_350.0, 273.0, 2_000.0, 2_000.0, Some(cruise))
            .expect("unmakeable");
        assert!(miss.low && miss.limit_ft == 18_000.0, "{miss:?}");
        assert!((1_000.0..1_700.0).contains(&miss.by_ft), "{miss:?}");
        assert_eq!(miss.line(), format!("We'll cross HEMEL about {} feet under its 18,000 feet.",
            crate::auto_status::commas(((miss.by_ft / 50.0).round() * 50.0) as i64)));
        // An "at or below" the best descent cannot reach: said high.
        let honl = legs[5].fix.clone().unwrap();
        let (lat, lon) = crate::nav::project_latlon(honl.lat, honl.lon, 90.0, 3.0);
        let high = star_window_miss(&legs, &cons, 5, lat, lon, 12_000.0, 250.0, 2_000.0, 2_000.0, Some(cruise))
            .expect("too high for HON");
        assert!(!high.low && high.limit_ft == 9_000.0 && high.by_ft > 1_000.0, "{high:?}");
        // Within the slack, or with room: nothing.
        assert_eq!(star_window_miss(&legs, &cons, 5, lat, lon, 10_200.0, 250.0, 2_000.0, 2_000.0, Some(cruise)), None);
    }

    /// #173 (sortie 1790624015-full_auto, +1408.6): held level at 6,000 on
    /// the LAM leg, 27.4 nm from HEMEL, the book's 2,000 fpm predicted
    /// 17,668 and "about 350 feet under"; the follower's own climb was
    /// nothing, and it crossed at 14,587. Judged on the climb flown, the
    /// line is the level being held; a clearance's hold is not judged.
    #[test]
    fn a_star_window_is_judged_on_the_climb_flown() {
        let recs: Vec<serde_json::Value> =
            include_str!("../bench/follower/egkk_lam5p_held_6000.jsonl")
                .lines()
                .filter_map(|l| serde_json::from_str(l).ok())
                .collect();
        let said = recs.iter().find(|r| r["kind"] == "star_window_unable").unwrap();
        let f = |k: &str| said[k].as_f64().unwrap();
        let (alt, dist, t) = (f("alt_ft"), f("dist_nm"), f("t"));
        // The groundspeed the prediction was made on, from its own numbers.
        let gs = dist / ((f("predicted_ft") - alt) / 2_000.0) * 60.0;
        assert!((250.0..320.0).contains(&gs), "{gs}");
        let tick = recs
            .iter()
            .filter(|r| r["kind"] == "leg")
            .min_by(|a, b| {
                let d = |r: &serde_json::Value| (r["t"].as_f64().unwrap() - t).abs();
                d(a).total_cmp(&d(b))
            })
            .unwrap();
        assert_eq!(tick["want_alt"].as_f64(), Some(6_000.0), "held at 6,000");
        let vs_cmd = tick["vs_cmd"].as_f64().unwrap();
        assert!(vs_cmd.abs() < 100.0, "{vs_cmd}");

        let (mut legs, cons) = egbb_heme1b();
        apply_arrival_constraints(&mut legs, &cons);
        let hemel = legs[1].fix.clone().unwrap();
        let (lat, lon) = crate::nav::project_latlon(hemel.lat, hemel.lon, 110.0, dist);
        let cruise = Some(18_000.0);
        let book = star_window_miss(&legs, &cons, 1, lat, lon, alt, gs, 2_000.0, 2_000.0, cruise)
            .expect("the log's line");
        assert!((book.by_ft - f("by_ft")).abs() < 50.0, "{book:?}");
        let climb = window_climb_fpm(vs_cmd, false).unwrap();
        let flown = star_window_miss(&legs, &cons, 1, lat, lon, alt, gs, climb, 2_000.0, cruise)
            .expect("held level, it is missed by the whole climb");
        assert!((flown.predicted_ft - alt).abs() < 100.0, "{flown:?}");
        assert_eq!(flown.line(), "We'll cross HEMEL about 12,000 feet under its 18,000 feet.");
        assert_eq!(window_climb_fpm(-300.0, false), Some(0.0));
        assert_eq!(window_climb_fpm(1_800.0, false), Some(1_800.0));
        assert_eq!(window_climb_fpm(1_800.0, true), None, "the clearance's level");
    }

    #[test]
    fn a_stars_speed_limits_start_in_time_and_hold_after() {
        let (legs, cons) = star_route();
        // 20 nm short of HAVEN at 290 kt: nothing yet.
        assert_eq!(arrival_speed_cap(&legs, &cons, 1, 10.0 / 60.0, 0.0, 290.0, 400.0, &mut Default::default()), None);
        // 40 kt to lose at 0.5 kt/s is 80 s, 8.9 nm at 400 kt, plus 2.
        let near = arrival_speed_cap(&legs, &cons, 1, 20.0 / 60.0, 0.0, 290.0, 400.0, &mut Default::default());
        assert_eq!(near, Some((250.0, "HAVEN".to_string())));
        // Past HAVEN the 250 holds; nearing TARTN its 230 takes over.
        assert_eq!(arrival_speed_cap(&legs, &cons, 2, 31.0 / 60.0, 0.0, 250.0, 300.0, &mut Default::default()).unwrap().0, 250.0);
        assert_eq!(arrival_speed_cap(&legs, &cons, 3, 66.0 / 60.0, 0.0, 250.0, 300.0, &mut Default::default()).unwrap().0, 230.0);
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
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
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
        vertical_tick(
            leg,
            carry,
            prev,
            alt,
            Some(dist_nm),
            135.0,
            pilot,
            floor,
            GA_VS_FPM,
            DescentProfile::none(),
        )
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
        // Inside it, down the path and staying on it: under the cruise,
        // above the constraint, lower each tick, the constraint at the fix.
        let v = tick(&leg, carry, prev, 8000.0, 17.0, None, None);
        assert!(v.want_alt < 8000.0 && v.want_alt > 2700.0, "{}", v.want_alt);
        let w = tick(&leg, v.carry, v.prev_cmd, 7000.0, 15.0, None, None);
        assert!(w.want_alt < v.want_alt && w.want_alt > 2700.0, "{}", w.want_alt);
        let x = tick(&leg, w.carry, w.prev_cmd, 2900.0, 1.0, None, None);
        assert_eq!(x.want_alt, 2700.0);
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
        let v = tick(
            &jrz(),
            v.carry,
            v.prev_cmd,
            3200.0,
            36.0,
            Some(4000.0),
            None,
        );
        assert_eq!(v.want_alt, 4000.0);
        let v = tick(
            &jrz(),
            v.carry,
            v.prev_cmd,
            4000.0,
            34.0,
            Some(4000.0),
            None,
        );
        assert_eq!(v.want_alt, 4000.0);
    }

    /// ...but the ground outranks the PILOT too. Four thousand feet was itself
    /// into the ridge; the follower holds the floor, not the number asked for.
    #[test]
    fn terrain_outranks_the_pilot_as_well() {
        let floor = 3861.0 + TERRAIN_CLEARANCE_FT;
        let v = tick(
            &jrz(),
            2700.0,
            2700.0,
            2800.0,
            38.0,
            Some(4000.0),
            Some(floor),
        );
        assert_eq!(v.want_alt, floor);
        assert_eq!(v.floored, Some(floor));
        assert!(!v.pilot, "the floor set it, not the pilot");
    }

    /// The LIEA-LEBL navlog's descent as the B738 flew it (sortie
    /// 1790494983-full_auto, 2026-09-27): each leg's plan altitude and its
    /// distance at the arm, from the `procedure` `leg` ticks.
    fn liea_lebl_descent() -> Vec<(ProcLeg, f64)> {
        [
            ("KIFMI", 31_000.0, 83.1),
            ("ELVOP", 28_000.0, 28.5),
            ("MUREN", 28_000.0, 5.7),
            ("CHELY", 29_000.0, 8.1),
            ("VERSO", 25_000.0, 34.9),
        ]
        .into_iter()
        .map(|(id, alt, d)| {
            let mut l = leg(id, 41.0, 3.0);
            l.alt1_ft = Some(alt);
            (l, d)
        })
        .collect()
    }

    /// Flies `legs` through the pure vertical the way the loop does (half a
    /// second a tick, the aircraft following the target at the airframe's
    /// rate) and returns, per leg, the highest and lowest altitude commanded
    /// and the held record, if any. `with_profile` false is the old law.
    fn fly_descent(legs: &[(ProcLeg, f64)], with_profile: bool) -> Vec<(f64, f64, Option<(f64, f64)>)> {
        const RATE: f64 = 2_000.0;
        const GS: f64 = 420.0;
        let plain: Vec<ProcLeg> = legs.iter().map(|(l, _)| l.clone()).collect();
        let cruise = plain.iter().filter_map(|l| l.alt1_ft).fold(f64::MIN, f64::max);
        let top = plan_top_index(&plain, cruise);
        let mut alt = 31_000.0;
        let (mut carry, mut prev, mut peak) = (alt, alt, alt);
        let mut out = Vec::new();
        for (idx, (l, dist)) in legs.iter().enumerate() {
            let (mut hi, mut lo, mut held) = (f64::MIN, f64::MAX, None);
            let mut d = *dist;
            while d > 0.2 {
                let profile = DescentProfile {
                    cruise_ft: with_profile.then_some(cruise),
                    peak_ft: peak,
                    past_top: top.is_some_and(|t| idx > t),
                    arrival_ceiling: false,
                };
                let v = vertical_tick(l, carry, prev, alt, Some(d), GS, None, None, RATE, profile);
                carry = v.carry;
                prev = v.prev_cmd;
                peak = v.peak_ft;
                held = held.or(v.descent_held);
                hi = hi.max(v.want_alt);
                lo = lo.min(v.want_alt);
                let step = RATE / 120.0;
                alt += (v.want_alt - alt).clamp(-step, step);
                d -= GS / 7200.0;
            }
            out.push((hi, lo, held));
        }
        out
    }

    /// THE bug of 2026-09-27: down from 31,000 and level at MUREN's 28,000,
    /// CHELY's navlog 29,000 was climbed to (28,999 by 08:28:37). It is held
    /// at 28,000 and said once; VERSO's 25,000 then descends as before.
    #[test]
    fn a_descent_never_climbs_back_to_a_navlog_altitude() {
        let legs = liea_lebl_descent();
        let old = fly_descent(&legs, false);
        assert_eq!(old[3].0, 29_000.0, "the test must bite: the old law climbed to CHELY");
        let new = fly_descent(&legs, true);
        assert_eq!(new[2].1, 28_000.0, "MUREN levelled at 28,000");
        assert_eq!(new[3].0, 28_000.0, "CHELY held at the level already commanded");
        assert_eq!(new[3].2, Some((29_000.0, 28_000.0)), "plan_ft and held_ft for the blackbox");
        assert_eq!(new[4].1, 25_000.0, "VERSO descends");
        assert!(new[4].0 <= 28_000.0, "and never above the held level: {}", new[4].0);
        assert_eq!(new[4].2, None);
    }

    /// The profile rule, one tick: past the plan's top, 28,000 commanded.
    fn in_descent(pilot: Option<f64>, floor: Option<f64>) -> Vertical {
        let mut chely = leg("CHELY", 41.0, 3.0);
        chely.alt1_ft = Some(29_000.0);
        let profile = DescentProfile { cruise_ft: Some(31_000.0), peak_ft: 31_000.0, past_top: true, ..Default::default() };
        vertical_tick(&chely, 28_000.0, 28_000.0, 28_000.0, Some(8.0), 420.0, pilot, floor, 2_000.0, profile)
    }

    /// A pilot or controller asking for a climb in the descent still gets it.
    #[test]
    fn a_pilot_handed_climb_in_the_descent_still_climbs() {
        let v = in_descent(Some(30_000.0), None);
        assert_eq!(v.want_alt, 30_000.0);
        assert!(v.pilot);
        assert_eq!(v.descent_held, None);
    }

    /// A climb for the ground is always right, descent or not.
    #[test]
    fn a_terrain_floor_above_the_held_level_still_climbs() {
        let v = in_descent(None, Some(28_600.0));
        assert_eq!(v.want_alt, 28_600.0);
        assert_eq!(v.floored, Some(28_600.0));
        assert_eq!(in_descent(None, None).want_alt, 28_000.0);
    }

    /// A step climb BEFORE the top of descent is a climb: 29,000 then 31,000.
    #[test]
    fn a_step_climb_before_the_top_of_descent_still_climbs() {
        let mut first = leg("AAAAA", 41.0, 3.0);
        first.alt1_ft = Some(29_000.0);
        let mut step = leg("BBBBB", 42.0, 3.0);
        step.alt1_ft = Some(31_000.0);
        let legs = vec![first, step.clone()];
        let top = plan_top_index(&legs, 31_000.0);
        assert_eq!(top, Some(1));
        let profile = DescentProfile {
            cruise_ft: Some(31_000.0),
            peak_ft: 29_000.0,
            past_top: top.is_some_and(|t| 1 > t),
            arrival_ceiling: false,
        };
        let v = vertical_tick(&step, 29_000.0, 29_000.0, 29_000.0, Some(60.0), 420.0, None, None, 2_000.0, profile);
        assert_eq!(v.want_alt, 31_000.0);
        assert_eq!(v.descent_held, None);
        assert_eq!(v.peak_ft, 29_000.0, "the peak is what was flown");
    }

    /// LFBD-EGKK, 2026-09-27 (sortie 1790510400-full_auto): the CNA6R from
    /// runway 29 as the follower flew it. DER29 and BD923 carry the plan's
    /// FL340, the CA leg between them its chart's "at or above 660".
    fn lfbd_cna6r() -> (ProcLeg, ProcLeg, ProcLeg) {
        let mut der = leg("DER29", 44.83, -0.72);
        der.leg_type = "DF".to_string();
        der.alt1_ft = Some(34_000.0);
        let ca = to_alt('+', 660.0);
        let mut bd923 = leg("BD923", 44.93, -0.80);
        bd923.leg_type = "DF".to_string();
        bd923.alt1_ft = Some(34_000.0);
        (der, ca, bd923)
    }

    /// THE bug of LFBD 2026-09-27: climbing through 7,638 ft on the CA leg
    /// with FL340 commanded since the runway, the terrain floor at 705 ft
    /// (the probe's ground plus 1,000). The CA's 660 dialled the aircraft
    /// down, the floor made it 705, and BD923's FL340 was then held at 705
    /// as a "descent that never climbs back". Both legs keep climbing now.
    #[test]
    fn a_climb_never_descends_to_a_minimum_it_has_passed() {
        let (der, ca, bd923) = lfbd_cna6r();
        let legs = vec![der.clone(), ca.clone(), bd923.clone()];
        let cruise = 34_000.0;
        let top = plan_top_index(&legs, cruise);
        let profile = |idx: usize, peak: f64| DescentProfile {
            cruise_ft: Some(cruise),
            peak_ft: peak,
            past_top: top.is_some_and(|t| idx > t),
            arrival_ceiling: false,
        };
        let floor = Some(705.0);
        let v0 = vertical_tick(&der, 34_000.0, 34_000.0, 7_618.0, Some(0.76), 265.0, None, floor, 2_000.0, profile(0, 7_618.0));
        assert_eq!(v0.want_alt, 34_000.0);
        let v1 = vertical_tick(&ca, v0.carry, v0.prev_cmd, 7_638.0, None, 265.0, None, floor, 2_000.0, profile(1, v0.peak_ft));
        assert_eq!(v1.want_alt, 34_000.0, "the CA's 660 is a minimum passed long ago");
        assert_eq!(v1.floored, None, "the floor is only ever a minimum");
        assert_eq!(v1.climb_held, Some((660.0, 34_000.0)), "plan_ft and held_ft for the blackbox");
        let v2 = vertical_tick(&bd923, v1.carry, v1.prev_cmd, 7_658.0, Some(7.4), 265.0, None, floor, 2_000.0, profile(2, v1.peak_ft));
        assert_eq!(v2.want_alt, 34_000.0, "BD923 climbs on to the cruise");
        assert!(v2.want_alt >= v1.prev_cmd);
        assert_eq!(v2.descent_held, None);
        assert_eq!(v2.climb_held, None);
    }

    /// The same tick without the profile: the test must bite. The CA's 660
    /// goes through and the floor is what stops the dive.
    #[test]
    fn the_lfbd_climb_bit_without_the_profile() {
        let (_, ca, _) = lfbd_cna6r();
        let v = vertical_tick(&ca, 34_000.0, 34_000.0, 7_638.0, None, 265.0, None, Some(705.0), 2_000.0, DescentProfile::none());
        assert_eq!(v.want_alt, 705.0, "without the profile the CA dials the aircraft down");
    }

    /// FL340 commanded but only 7,700 ft flown is not the cruise: not in the
    /// descent, still climbing. Flown to within a step of it, it is.
    #[test]
    fn a_cruise_commanded_but_not_flown_is_not_descending() {
        let climb = DescentProfile { cruise_ft: Some(34_000.0), peak_ft: 7_700.0, past_top: false, arrival_ceiling: false };
        assert!(!climb.descending(705.0));
        assert!(!climb.descending(34_000.0));
        assert!(climb.climbing());
        let cruised = DescentProfile { peak_ft: 33_950.0, ..climb };
        assert!(cruised.descending(31_000.0));
        assert!(!cruised.climbing());
        // Past the plan's top it is the descent whatever was flown.
        let past = DescentProfile { past_top: true, ..climb };
        assert!(past.descending(7_700.0));
        assert!(!past.climbing());
    }

    /// An arrival ceiling before the cruise was reached (a short flight, a
    /// slow climb) is the plan's descent, not a dip: it is flown.
    #[test]
    fn an_arrival_ceiling_in_the_climb_is_flown() {
        let mut fix = leg("REVTU", 51.0, 0.0);
        fix.alt1_ft = Some(26_600.0);
        let profile = DescentProfile { cruise_ft: Some(34_000.0), peak_ft: 30_000.0, past_top: false, arrival_ceiling: true };
        let v = vertical_tick(&fix, 34_000.0, 34_000.0, 30_000.0, Some(5.0), 420.0, None, None, 2_000.0, profile);
        assert_eq!(v.climb_held, None);
        assert!(v.want_alt < 30_000.0, "the ceiling is paced down to: {}", v.want_alt);
    }

    /// A follower armed mid-descent (a resume) has never commanded the
    /// cruise, but past the plan's top it is in the descent all the same.
    #[test]
    fn a_follower_armed_in_the_descent_holds_too() {
        let mut chely = leg("CHELY", 41.0, 3.0);
        chely.alt1_ft = Some(29_000.0);
        let profile = DescentProfile { cruise_ft: Some(31_000.0), peak_ft: 28_000.0, past_top: true, ..Default::default() };
        let v = vertical_tick(&chely, 28_000.0, 28_000.0, 28_000.0, Some(8.0), 420.0, None, None, 2_000.0, profile);
        assert_eq!(v.want_alt, 28_000.0);
        // Not past the top and never at the cruise: a climb.
        let early = DescentProfile { past_top: false, ..profile };
        let v = vertical_tick(&chely, 28_000.0, 28_000.0, 28_000.0, Some(8.0), 420.0, None, None, 2_000.0, early);
        assert_eq!(v.want_alt, 29_000.0);
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
        assert!(
            (last - 15.0).abs() < 0.5,
            "last point is the look-ahead: {last}"
        );
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
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
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
        let legs = vec![
            leg("09D06", 36.088, -5.404666667),
            faf,
            leg("RW09", 36.1509, -5.3581),
        ];
        let why = faf_off_course(&legs, &airport, "09").expect("refused");
        assert!(
            why.contains("09D05") && why.contains("off the runway zero nine centreline"),
            "{why}"
        );

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
        assert_eq!(
            power_hold_target(&EndAction::Enroute, Some(180.0)),
            Some(180.0)
        );
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
        assert!(
            !leg_captured(&l, Some(1.5), 2000.0, CAPTURE_NM),
            "far: not captured"
        );
        assert!(
            leg_captured(&l, Some(CAPTURE_NM - 0.1), 2000.0, CAPTURE_NM),
            "inside: captured"
        );
    }

    /// The EIDW departure that orbited (2026-09-20). The 737 climbing at
    /// 250 kt turns with a radius bigger than the gap between the SID's
    /// fixes, so the flat 0.8 nm capture was a circle the aircraft could
    /// never enter: it banked 25°, went round, and the distance to the fix
    /// GREW (4.5, 6.7, 7.6 nm) until the stall watchdog stood the follower
    /// down. The capture has to be at least the turn the fix demands.
    /// The BIKF arrival (2026-09-20), which the shared 700 fpm could not fly.
    /// Every leg inbound of BASLU wanted about 2,000 fpm; at 700 the aircraft
    /// would have reached the field some 17,000 ft high, and the approach
    /// would never have armed.
    /// A heading written with the WRONG variation is a wrong heading, and the
    /// roll-out capture is where it does the damage: "bug on the nose" with a
    /// stale variation is not a capture at all, it is a standing turn command.
    ///
    /// BIKF-EGPH (2026-09-20): departed Keflavik (about 11 degrees west),
    /// arrived over Scotland (under one). The variation was read once before
    /// the loop, so every tick wrote a bug 11.4 degrees right of the nose —
    /// the log shows `bug - psi` pinned at exactly 11.4 for the whole orbit
    /// while the crab beside it varied, which is how a constant offset tells
    /// itself apart from a control error.
    #[test]
    fn a_stale_magnetic_variation_turns_a_capture_into_a_turn() {
        // The roll-out branch commands the nose. With the right variation
        // that is zero error and the autopilot rolls level.
        let (psi_true, here) = (142.2, 0.78);
        let correct = true_to_mag(psi_true, here);
        assert!(
            (crate::circuit::bug_toward(correct, 122.2, correct, 25.0, 40.0) - correct).abs()
                < 0.01,
            "on the nose is a capture"
        );
        // With Keflavik's variation still in hand — `true_to_mag` is
        // positive-WEST, so Iceland's is the LARGER number — it is 11.4
        // degrees of standing right turn, the same way the aircraft is
        // already banked. Which is exactly what it flew.
        let stale = true_to_mag(psi_true, here + 11.4);
        let commanded = crate::circuit::bug_toward(stale, 122.2, stale, 25.0, 40.0);
        let error = ((commanded - correct + 540.0).rem_euclid(360.0)) - 180.0;
        assert!(
            (error - 11.4).abs() < 0.1,
            "a standing turn command, not a capture: {error:.1} degrees"
        );
    }

    #[test]
    fn a_jet_descent_is_paced_at_the_jets_rate_not_the_ga_one() {
        // BASLU at 27,700 to the field, 78 nm, 340 kt over the ground.
        let (drop_ft, dist_nm, gs) = (27_529.0, 78.0, 340.0);
        let reach = |fpm: f64| dist_nm / gs * 60.0 * fpm;
        assert!(
            reach(GA_VS_FPM) < drop_ft * 0.5,
            "the GA rate cannot lose half of it: {:.0} of {drop_ft:.0} ft",
            reach(GA_VS_FPM)
        );
        assert!(
            reach(2000.0) >= drop_ft,
            "the jet's rate makes it: {:.0} ft",
            reach(2000.0)
        );
        // ...and the top of descent moves with the rate, so the point and the
        // rate can never disagree: three times the rate, a third of the run.
        let ga = descent_point_nm(6_000.0, 34_000.0, 340.0, GA_VS_FPM).unwrap();
        let jet = descent_point_nm(6_000.0, 34_000.0, 340.0, 2000.0).unwrap();
        let ratio = (ga - TOD_LEAD_NM) / (jet - TOD_LEAD_NM);
        assert!(
            (ratio - 2000.0 / GA_VS_FPM).abs() < 0.01,
            "TOD scales inversely with the rate: {ga:.0} nm vs {jet:.0} nm (x{ratio:.2})"
        );
        // Concretely: the GA rate would start this jet down 228 nm out, which
        // is before the fix it is descending TO.
        // (Planned at 75% of each rate since 2026-09-26: the jet's point is
        // about 100 nm out, still well inside the GA one.)
        assert!(ga > 250.0 && jet < 110.0, "{ga:.0} nm vs {jet:.0} nm");
    }

    /// The rates are the airframe's, republished on every swap, and a profile
    /// that declares nothing keeps the GA figure it always had.
    #[test]
    fn the_vertical_rates_are_published_per_airframe() {
        let _g = crate::mode::TEST_LOCK.lock();
        let mut r = crate::aircraft::ReferenceProfile::default();
        publish_vs_limits(Some(&r));
        assert_eq!(descent_fpm(), GA_VS_FPM, "silent profile keeps the default");
        assert_eq!(climb_fpm(), GA_VS_FPM);
        r.enroute_climb_fpm = Some(2000.0);
        r.enroute_descent_fpm = Some(2000.0);
        publish_vs_limits(Some(&r));
        assert_eq!(descent_fpm(), 2000.0);
        assert_eq!(climb_fpm(), 2000.0);
        assert_eq!(vs_clamp_fpm(), 2000.0, "the spoken tool promises the truth");
        // A swap to an airframe with no figures must not inherit the jet's:
        // a process-wide fact never outlives its airframe.
        publish_vs_limits(None);
        assert_eq!(descent_fpm(), GA_VS_FPM);
        assert_eq!(climb_fpm(), GA_VS_FPM);
        // A nonsense figure is refused rather than flown.
        r.enroute_descent_fpm = Some(-50.0);
        publish_vs_limits(Some(&r));
        assert_eq!(descent_fpm(), GA_VS_FPM);
    }

    #[test]
    fn a_jet_sequences_a_fix_it_could_never_have_closed_on() {
        let gs = 280.0;
        let r = enroute_turn_radius_nm(gs);
        assert!(
            r > 2.0,
            "a jet's enroute turn is nautical miles, not the pattern clamp's 1.35: {r:.2}"
        );
        // DE28R -> DW128 -> DW991 are 2.9 and 3.9 nm apart. A 60 degree turn
        // at that radius must be started well outside the old flat radius.
        let cap = capture_radius_nm(gs, 60.0);
        assert!(
            cap > CAPTURE_NM,
            "the turn is anticipated, not chased: {cap:.2} nm"
        );
        assert!(cap <= r, "and never leads by more than the radius itself");
        let l = leg("DW128", 53.4657, -6.3523);
        // 1.5 nm out with the turn already due (the anticipation is 1.81 nm
        // here): still "not reached" under the old flat radius, which is how
        // the follower ended up chasing it round; sequenced now.
        assert!(!leg_captured(&l, Some(1.5), 6000.0, CAPTURE_NM), "the bug");
        assert!(leg_captured(&l, Some(1.5), 6000.0, cap), "the fix");
    }

    /// ...and the airframes that were already right stay byte-identical: in
    /// both cases the anticipation lands under the floor, so the floor is
    /// still the whole answer.
    #[test]
    fn turn_anticipation_never_disturbs_a_leg_that_already_worked() {
        // A 172 at 100 knots, turning 90 degrees — the sharpest thing a
        // light aircraft meets on a procedure.
        assert_eq!(capture_radius_nm(100.0, 90.0), CAPTURE_NM);
        // A jet at cruise: an 8 nm radius, but a 10 degree track change.
        assert_eq!(capture_radius_nm(450.0, 10.0), CAPTURE_NM);
        // Dead straight is always the floor, whatever the speed.
        assert_eq!(capture_radius_nm(450.0, 0.0), CAPTURE_NM);
    }

    /// The turn is measured from where we ARE into the fix, then on to the
    /// next one — so a leg with nothing after it anticipates nothing.
    #[test]
    fn the_turn_angle_comes_from_the_two_legs_it_joins() {
        let here = leg("A", 53.5, -6.0);
        let next = leg("B", 53.5, -5.0); // due east of A
                                         // Approaching A from due west: the turn onto B is straight ahead.
        let straight = leg_turn_deg(53.5, -7.0, &here, Some(&next)).unwrap();
        assert!(straight.abs() < 1.0, "no turn: {straight:.1}");
        // Approaching A from the south: a left turn onto the eastbound leg.
        let left = leg_turn_deg(52.5, -6.0, &here, Some(&next)).unwrap();
        assert!(
            (left - 90.0).abs() < 2.0,
            "a 90 degree turn, right-positive: {left:.1}"
        );
        assert_eq!(leg_turn_deg(53.5, -7.0, &here, None), None, "last leg");
        assert_eq!(
            leg_turn_deg(53.5, -7.0, &to_alt('+', 3000.0), Some(&next)),
            None,
            "a to-altitude leg has no fix to turn at"
        );
    }

    #[test]
    fn a_climb_leg_captures_at_or_above_the_altitude() {
        let climb = to_alt('+', 3000.0);
        assert!(
            !leg_captured(&climb, None, 2000.0, CAPTURE_NM),
            "below: keep climbing"
        );
        assert!(leg_captured(&climb, None, 3000.0, CAPTURE_NM), "made it");
        assert!(
            leg_captured(&climb, None, 2900.0, CAPTURE_NM),
            "within margin counts"
        );
    }

    #[test]
    fn a_descend_leg_captures_at_or_below_the_altitude() {
        let down = to_alt('-', 3000.0);
        assert!(
            !leg_captured(&down, None, 5000.0, CAPTURE_NM),
            "above: keep descending"
        );
        assert!(leg_captured(&down, None, 3000.0, CAPTURE_NM), "made it");
    }

    #[test]
    fn terrain_floor_is_converted_to_the_altimeter() {
        // Hot day: 7,012 true reads 6,838. A 6,500 true floor must be flown
        // as 6,326 indicated, not 6,500 (174 ft into the clearance).
        assert!((floor_indicated(6_500.0, 7_012.0, 6_838.0) - 6_326.0).abs() < 1e-9);
        // Cold day the other way: the indicated floor sits ABOVE the true one.
        assert!((floor_indicated(6_500.0, 6_800.0, 7_000.0) - 6_700.0).abs() < 1e-9);
        // Standard day: no change.
        assert_eq!(floor_indicated(6_500.0, 7_000.0, 7_000.0), 6_500.0);
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

    /// EGKK ILS 26L (sortie 1790510400-full_auto, stock 737, 2026-09-27):
    /// the real CIFP final codes its MAP on RW26L ("G  M"), and the MAP rule
    /// for LSZS handed off THERE: "At the fix. Landing us." at 1 nm and 310 ft
    /// radio, 550 m off the centreline, localizer and glideslope 2.5 dots, a
    /// go-around at 97 ft 244 m off. The handoff is the FAF, FF26L; on the
    /// sortie's tick that sequenced it (the aircraft 4.3 nm out, turning onto
    /// the final from the south) autoland takes it there, not a mile out.
    #[test]
    fn a_runway_map_hands_autoland_the_approach_at_the_faf() {
        let rows = "APPCH:010,I,I26L, ,CF26L,EG,P,C,E  I, ,   ,IF, ,IWW,EG,P,I,      ,0766,0082,    ,    ,J,02000,02000,     , ,   ,    ,   , , , , , ,0,D,S;\n\
                    APPCH:020,I,I26L, ,FF26L,EG,P,C,E  F, ,   ,CF, ,IWW,EG,P,I,      ,0766,0072,2560,0010,H,02000,02000,     , ,   ,-300,   ,EGKK,EG,P,A, ,0,D,S;\n\
                    APPCH:030,I,I26L, ,RW26L,EG,P,G,G  M, ,   ,CF, ,IWW,EG,P,I,      ,0766,0017,2560,0055, ,00248,     ,     , ,   ,-300,   , , , , , ,0,D,S;\n\
                    APPCH:040,I,I26L, , , , , ,  M , ,   ,CA, , , , , ,      ,    ,    ,2560,    ,+,02000,     ,     , ,   ,    ,   , , , , , ,0,D,S;\n\
                    APPCH:050,I,I26L, , , , , , E  , ,   ,VM, , , , , ,      ,    ,    ,1770,    , ,03000,     ,     , ,   ,    ,   , , , , , ,0,D,S;\n";
        let raw = crate::nav::parse_proc_legs(
            rows.lines().map(str::trim),
            crate::nav::ProcKind::Appch,
            "I26L",
            None,
        );
        let mut legs = crate::nav::straight_in_legs(raw, "26L").expect("straight in");
        let at = |ident: &str, lat: f64, lon: f64| Fix {
            ident: ident.into(),
            lat,
            lon,
        };
        legs[0].fix = Some(at("CF26L", 51.173697222, -0.003744444));
        legs[1].fix = Some(at("FF26L", 51.170172222, -0.029625));
        legs[2].fix = Some(at("RW26L", 51.150672, -0.171944));
        assert_eq!(
            legs.iter()
                .map(|l| (l.fix_ident.as_str(), l.is_faf, l.is_map))
                .collect::<Vec<_>>(),
            [
                ("CF26L", false, false),
                ("FF26L", true, false),
                ("RW26L", false, true)
            ]
        );
        assert_eq!(
            handoff_index(&legs),
            Some(1),
            "the FAF, never the runway's MAP"
        );

        // t 1790515473.3: FF26L sequenced, groundspeed 261 kt, track 042.
        let (lat, lon) = (51.141135858, -0.052537624);
        let d = crate::nav::haversine_nm(lat, lon, 51.170172222, -0.029625);
        let cap = capture_radius_nm(
            260.6,
            leg_turn_deg(lat, lon, &legs[1], legs.get(2)).unwrap(),
        );
        assert!(
            leg_done(&legs[1], Some(d), 2914.0, cap, lat, lon),
            "d {d:.2} cap {cap:.2}"
        );
        let thr = crate::nav::haversine_nm(lat, lon, 51.150672, -0.171944);
        assert!(
            thr > 4.0,
            "handed over {thr:.1} nm out, not the 1 nm of the sortie"
        );
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
        assert_eq!(
            descent_target_ft(1397.0, 3900.0, 3888.0, Some(25.0), 200.0, GA_VS_FPM),
            3900.0
        );
        // TOD: 2,491 ft planned at 525 fpm is 4.7 min, 15.8 nm at 200 kt,
        // plus the lead: level at 17.5, on the path at 12, at the constraint
        // at the fix's lead.
        assert_eq!(
            descent_target_ft(1397.0, 3900.0, 3888.0, Some(17.5), 200.0, GA_VS_FPM),
            3900.0
        );
        let on_path = descent_target_ft(1397.0, 3900.0, 3500.0, Some(12.0), 200.0, GA_VS_FPM);
        assert!((on_path - 3130.0).abs() < 20.0, "{on_path}");
        assert_eq!(
            descent_target_ft(1397.0, 3900.0, 1500.0, Some(1.0), 200.0, GA_VS_FPM),
            1397.0
        );
        // Climbs, small steps and fix-less legs go straight to the constraint.
        assert_eq!(
            descent_target_ft(4000.0, 2000.0, 2000.0, Some(30.0), 200.0, GA_VS_FPM),
            4000.0
        );
        assert_eq!(
            descent_target_ft(3900.0, 4000.0, 3990.0, Some(30.0), 200.0, GA_VS_FPM),
            3900.0
        );
        assert_eq!(
            descent_target_ft(1000.0, 4000.0, 4000.0, None, 200.0, GA_VS_FPM),
            1000.0
        );
        // Never reached the previous target: hold where we are, rounded.
        assert_eq!(
            descent_target_ft(1000.0, 6000.0, 3460.0, Some(40.0), 200.0, GA_VS_FPM),
            3500.0
        );
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
            transition_altitude_ft: None,
            climb_minimums: Vec::new(),
            arrival_constraints: Vec::new(),
            visual_deg: None,
            missed_legs: Vec::new(),
            cruise_schedule: Default::default(),
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
    #[test]
    fn a_probe_lost_to_a_pause_does_not_retire_the_terrain_floor() {
        // LXGB-LEAX 2026-09-15: the pause landed inside the probe's wait.
        let mut m = 0;
        assert!(!terrain_probe_fatal(&mut m, true));
        assert_eq!(m, 0, "a frozen miss is not a miss");
        // One live miss is deferred; the second in a row retires the floor.
        assert!(!terrain_probe_fatal(&mut m, false));
        assert!(terrain_probe_fatal(&mut m, false));
        // A frozen miss between two live ones neither counts nor resets.
        let mut m = 1;
        assert!(!terrain_probe_fatal(&mut m, true));
        assert_eq!(m, 1);
        assert!(terrain_probe_fatal(&mut m, false));
    }

    /// The trimmed OFP (EGKK-EGBB, B738, CI 77, FL180; the pilot's plan,
    /// 2026-09-28), its navlog as the fetcher builds the waypoints.
    fn egkk_egbb() -> (Vec<crate::simbrief::Waypoint>, crate::simbrief::CruiseSchedule) {
        let v: serde_json::Value =
            serde_json::from_str(include_str!("../bench/ofp/egkk-egbb-738.trimmed.json")).unwrap();
        let n = |f: &serde_json::Value, k: &str| f[k].as_str().unwrap().parse::<f64>().unwrap();
        let mut wps: Vec<_> = v["navlog"]["fix"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|f| f["ident"] != "TOC" && f["ident"] != "TOD")
            .map(|f| wp(f["ident"].as_str().unwrap(), n(f, "pos_lat"), n(f, "pos_long"), n(f, "altitude_feet")))
            .collect();
        wps.insert(0, wp("EGKK", 51.148, -0.19, 0.0));
        (wps, crate::simbrief::parse_schedule(&v))
    }

    #[test]
    fn a_planned_mach_is_held_as_the_airspeed_it_is_at_the_level() {
        // The OFP's HEMEL: M0.628 at FL180 planned at 300 KIAS.
        let fl180 = cas_for_mach(0.628, 18_000.0);
        assert!((fl180 - 300.0).abs() < 2.0, "{fl180}");
        // The 737 profile's own reasoning (dev-history 2026-09-19): M0.78
        // at FL360 is about 255 to 260.
        let fl360 = cas_for_mach(0.78, 36_000.0);
        assert!((255.0..=262.0).contains(&fl360), "{fl360}");
        // Above the tropopause too, and at sea level Mach is TAS.
        assert!(cas_for_mach(0.78, 39_000.0) < fl360);
        assert!((cas_for_mach(0.3, 0.0) - 198.4).abs() < 1.0);
    }

    /// #101: the 737's profile flies 255 at every level, chosen to stay under
    /// Mmo at FL360. The OFP planned 300 KIAS / M0.63 for the FL180 cruise of
    /// EGKK-EGBB: the cruise legs are flown at it, the climb and descent
    /// legs are the profile's, and a level other than the plan's stays on
    /// the Mach (the lower of the two).
    #[test]
    fn the_ofp_cruise_speed_is_flown_on_its_cruise_legs() {
        let (wps, sched) = egkk_egbb();
        assert_eq!((sched.cost_index, sched.profile.as_deref()), (Some(77.0), Some("CI 77")));
        let legs = enroute_legs(&wps, "EGKK", "EGBB", 18_000.0);
        let at = |id: &str| legs.iter().find(|l| l.fix_ident == id).unwrap();
        let hemel = scheduled_cruise_kts(&sched, at("HEMEL"), 18_000.0).expect("a cruise leg");
        assert_eq!((hemel.plan_ias_kts, hemel.target_kts), (Some(300.0), 300.0));
        assert_eq!(scheduled_cruise_kts(&sched, at("D156O"), 18_000.0).unwrap().target_kts, 299.0);
        // Climb and descent legs: no schedule, the profile's speed.
        for id in ["DET31", "ACORN", "PUFAX", "HON", "OSKOT"] {
            assert_eq!(scheduled_cruise_kts(&sched, at(id), 12_000.0), None, "{id}");
        }
        // Held at FL280 (a controller's level): the Mach, not the 300.
        let high = scheduled_cruise_kts(&sched, at("HEMEL"), 28_000.0).unwrap().target_kts;
        assert!((240.0..=250.0).contains(&high), "{high}");
        // The 737 (floor about 167, ceiling Vne 340 less 15): the OFP's 300,
        // where the old law held the profile's 255.
        let pick = cruise_pick(255.0, Some(&hemel), Some(167.0), Some(325.0), None);
        assert_eq!(pick, CruisePick { kts: 300.0, source: "ofp", rejected: None, capped_from: None });
        assert_eq!(cruise_pick(255.0, None, Some(167.0), Some(325.0), None).kts, 255.0);
        // A 737 plan in a 172 is never flown at 300: the airframe's number.
        let pick = cruise_pick(108.0, Some(&hemel), Some(58.0), Some(148.0), None);
        assert_eq!(
            pick,
            CruisePick { kts: 108.0, source: "profile", rejected: Some("above_ceiling"), capped_from: None }
        );
        let slow = ScheduledSpeed { target_kts: 90.0, ..hemel.clone() };
        assert_eq!(cruise_pick(255.0, Some(&slow), Some(167.0), None, None).rejected, Some("below_floor"));
        // #156: under the overspeed caution (0.9 x 340 = 306) by the margin:
        // 296, the plan's 300 kept as `capped_from`. The FL180 cruise as
        // flown held 294.2 to 304.2 around its 299 (+2010 to +2100): the same
        // wander over the cap stays out of the caution band.
        let cap = cruise_cap_kts(Some(340.0));
        assert_eq!(cap, Some(296.0));
        let pick = cruise_pick(255.0, Some(&hemel), Some(167.0), Some(325.0), cap);
        assert_eq!(pick, CruisePick { kts: 296.0, source: "ofp", rejected: None, capped_from: Some(300.0) });
        assert!(296.0 + (304.2 - 299.2) < 306.0);
        // Under the cap, or a cap under the floor: the plan's own number.
        let under = ScheduledSpeed { target_kts: 280.0, ..hemel.clone() };
        assert_eq!(
            cruise_pick(255.0, Some(&under), Some(167.0), Some(325.0), cap),
            CruisePick { kts: 280.0, source: "ofp", rejected: None, capped_from: None }
        );
        assert_eq!(cruise_pick(255.0, Some(&hemel), Some(167.0), None, Some(150.0)).kts, 300.0);
        assert_eq!(cruise_cap_kts(None), None);
        // A leg whose ident matches but whose fix is elsewhere is not it.
        let mut far = at("HEMEL").clone();
        far.fix.as_mut().unwrap().lat += 1.0;
        assert_eq!(scheduled_cruise_kts(&sched, &far, 18_000.0), None);
        // One level, no step: the legs are left as built.
        let mut same = legs.clone();
        assert!(apply_step_climbs(&mut same, &sched).is_empty());
        let alts = |l: &[ProcLeg]| l.iter().map(|l| l.alt1_ft).collect::<Vec<_>>();
        assert_eq!(alts(&same), alts(&legs));
    }

    /// An EIDW-BIKF shaped plan (the sector the step-climb advisory's FL360
    /// test flies): FL340 initial, "ERNAN/0360" a step. The navlog shows
    /// 36,000 from the fix after ERNAN, so `enroute_legs` commanded FL360
    /// from the first leg and the step was flown as the initial cruise.
    #[test]
    fn a_planned_step_climb_is_flown_at_its_fix() {
        use crate::simbrief::{CruiseSchedule, NavlogSpeed, Stage, StepClimb};
        let route = [
            ("EIDW", 53.42, -6.27, 0.0, None),
            ("DUB", 53.50, -6.50, 12_000.0, Some(Stage::Climb)),
            ("LIFFY", 53.60, -7.00, 30_000.0, Some(Stage::Climb)),
            ("BAMLI", 54.00, -8.00, 34_000.0, Some(Stage::Cruise)),
            ("ERNAN", 55.00, -10.00, 34_000.0, Some(Stage::Cruise)),
            ("5715N", 57.00, -15.00, 36_000.0, Some(Stage::Cruise)),
            ("6120N", 61.00, -20.00, 36_000.0, Some(Stage::Cruise)),
            ("KEF", 63.90, -22.60, 11_000.0, Some(Stage::Descent)),
            ("BIKF", 63.98, -22.60, 0.0, None),
        ];
        let wps: Vec<_> = route.iter().map(|(i, la, lo, a, _)| wp(i, *la, *lo, *a)).collect();
        let sched = CruiseSchedule {
            cost_index: Some(30.0),
            profile: Some("CI 30".into()),
            fixes: route
                .iter()
                .filter_map(|(i, la, lo, a, st)| {
                    st.map(|stage| NavlogSpeed {
                        ident: i.to_string(),
                        lat: *la,
                        lon: *lo,
                        stage,
                        alt_ft: *a,
                        ias_kts: Some(270.0),
                        mach: Some(0.78),
                    })
                })
                .collect(),
            steps: crate::simbrief::parse_step_climbs("EIDW/0340 ERNAN/0360"),
            tod: None,
        };
        assert_eq!(sched.steps[1], StepClimb { fix: "ERNAN".into(), alt_ft: 36_000.0 });
        let built = enroute_legs(&wps, "EIDW", "BIKF", 34_000.0);
        let alts = |l: &[ProcLeg]| l.iter().map(|l| l.alt1_ft.unwrap()).collect::<Vec<_>>();
        // As built: FL360 from the runway.
        assert_eq!(alts(&built), vec![36_000.0, 36_000.0, 36_000.0, 36_000.0, 36_000.0, 36_000.0, 11_000.0]);
        let mut legs = built.clone();
        let placed = apply_step_climbs(&mut legs, &sched);
        assert_eq!(placed, vec![PlacedStep { fix: "ERNAN".into(), idx: 3, alt_ft: 36_000.0 }]);
        // FL340 to and over ERNAN, FL360 from the leg after it, the descent
        // untouched.
        assert_eq!(alts(&legs), vec![34_000.0, 34_000.0, 34_000.0, 34_000.0, 36_000.0, 36_000.0, 11_000.0]);
        // Said as it begins, on the leg flown from ERNAN, and nowhere else.
        assert_eq!(step_climb_at(&legs, 4, &sched), Some(("ERNAN".into(), 34_000.0, 36_000.0)));
        assert!((0..legs.len()).filter(|i| *i != 4).all(|i| step_climb_at(&legs, i, &sched).is_none()));
        // The follower's vertical: level at FL340 on the leg to ERNAN (no
        // early climb), climbing to FL360 on the leg after it; the cruise is
        // the plan's highest level, so this is a climb before the top.
        let cruise = legs.iter().filter_map(|l| l.alt1_ft).fold(f64::MIN, f64::max);
        let top = plan_top_index(&legs, cruise);
        let profile = |idx: usize| DescentProfile {
            cruise_ft: Some(cruise),
            peak_ft: 34_000.0,
            past_top: top.is_some_and(|t| idx > t),
            arrival_ceiling: false,
        };
        let v = vertical_tick(&legs[3], 34_000.0, 34_000.0, 34_000.0, Some(80.0), 460.0, None, None, 2_000.0, profile(3));
        assert_eq!(v.want_alt, 34_000.0);
        let v = vertical_tick(&legs[4], 34_000.0, 34_000.0, 34_000.0, Some(200.0), 460.0, None, None, 2_000.0, profile(4));
        assert_eq!(v.want_alt, 36_000.0);
        // A step fix the route does not carry: nothing is changed.
        let mut other = built.clone();
        let lost = CruiseSchedule {
            steps: crate::simbrief::parse_step_climbs("EIDW/0340 NOSUCH/0360"),
            ..sched.clone()
        };
        assert!(apply_step_climbs(&mut other, &lost).is_empty());
        assert_eq!(alts(&other), alts(&built));
        // A plan without stages (an old OFP) is not stepped either.
        let mut bare = built.clone();
        let nostage = CruiseSchedule { fixes: Vec::new(), ..sched };
        assert!(apply_step_climbs(&mut bare, &nostage).is_empty());
        assert_eq!(alts(&bare), alts(&built));
    }
}

/// #153: the follower's departure owners on the EGKK LAM5P climb
/// (2026-09-28, sortie 1790601822-full_auto): D156J's 6,000 minimum, the
/// plan's navlog 5,000 at ACORN and D156O, the IFR clearance's 5,000.
#[cfg(test)]
mod departure_minimum_tests {
    use super::*;
    use crate::nav::ClimbMinimum;

    const HUNT: &str = include_str!("../bench/follower/egkk_lam5p_minimum_hunt.jsonl");

    fn records() -> Vec<serde_json::Value> {
        HUNT.lines().filter_map(|l| serde_json::from_str(l).ok()).collect()
    }

    fn mins() -> Vec<ClimbMinimum> {
        vec![
            ClimbMinimum { fix: "ACORN".into(), min_ft: 5000.0 },
            ClimbMinimum { fix: "D156J".into(), min_ft: 6000.0 },
        ]
    }

    /// The LAM5P as flown, on a meridian: (ident, nm from the arm, plan ft).
    const ROUTE: [(&str, f64, Option<f64>); 7] = [
        ("D075D", 0.9, Some(1210.0)),
        ("D258T", 7.5, Some(2500.0)),
        ("ACORN", 13.5, Some(5000.0)),
        ("D156O", 20.5, None),
        ("D156J", 29.0, None),
        ("LAM", 39.0, Some(8000.0)),
        ("HEMEL", 60.0, Some(22000.0)),
    ];

    fn legs() -> Vec<ProcLeg> {
        ROUTE
            .iter()
            .map(|(id, nm, alt)| ProcLeg {
                seq: 0,
                transition: String::new(),
                leg_type: "TF".into(),
                fix_ident: (*id).into(),
                fix_region: String::new(),
                fix: Some(crate::nav::Fix { ident: (*id).into(), lat: 51.0 + nm / 60.0, lon: 0.0 }),
                course_mag: None,
                dist_nm: None,
                turn: None,
                alt_desc: ' ',
                alt1_ft: *alt,
                alt2_ft: None,
                is_faf: false,
                is_map: false,
                vpa_deg: None,
                navaid_ident: String::new(),
                navaid_region: String::new(),
                navaid: None,
                path: Default::default(),
            })
            .collect()
    }

    fn changes(v: &[f64]) -> usize {
        v.windows(2).filter(|w| (w[0] - w[1]).abs() > 0.5).count()
    }

    /// The recorded ticks, replayed through the old stateless test and the
    /// new memory: the old one flips the target and the speed on the
    /// altimeter's last feet at 5,900, exactly as the sortie logged them;
    /// the new one meets D156J once and holds 6,000.
    #[test]
    fn the_recorded_climb_met_d156j_once_and_held_its_floor() {
        let recs = records();
        let ticks: Vec<&serde_json::Value> = recs.iter().filter(|r| r["kind"] == "leg").collect();
        assert!(ticks.len() > 500, "{}", ticks.len());
        // What the sortie logged: the governor's target flipping 210/250.
        let logged: Vec<f64> = recs
            .iter()
            .filter(|r| r["kind"] == "speed_limit")
            .map(|r| r["target"].as_f64().unwrap())
            .collect();
        assert!(changes(&logged) >= 18, "{logged:?}");
        let recorded_want: Vec<f64> = ticks.iter().map(|r| r["want_alt"].as_f64().unwrap()).collect();
        assert!(changes(&recorded_want) >= 18, "the target hunted 5,000/6,000: {}", changes(&recorded_want));

        let (mins, legs) = (mins(), legs());
        let idx_of = |r: &serde_json::Value| {
            let fix = r["fix"].as_str().unwrap();
            legs.iter().position(|l| l.fix_ident == fix).unwrap()
        };
        let released = recs
            .iter()
            .find(|r| r["kind"] == "cleared_ceiling" && r["ft"].is_null())
            .map(|r| r["t"].as_f64().unwrap())
            .unwrap();
        let mut memory = DepartureMinimums::default();
        let (mut old_speed, mut new_speed, mut new_want) = (Vec::new(), Vec::new(), Vec::new());
        for r in &ticks {
            let (idx, alt) = (idx_of(r), r["alt_ft"].as_f64().unwrap());
            let t = r["t"].as_f64().unwrap();
            old_speed.push(if unmet_minimum(&mins, &legs, idx, alt).is_some() { 210.0 } else { 250.0 });
            new_speed.push(if memory.unmet(&mins, &legs, idx, alt).is_some() { 210.0 } else { 250.0 });
            // The plan's own level: the logged target where no minimum
            // raised it (the navlog's 3,682 held and 5,000).
            let logged = r["want_alt"].as_f64().unwrap();
            let plan = if (logged - 6000.0).abs() < 1.0 { 5000.0 } else { logged };
            let ceiling = (t < released).then_some(5000.0);
            let floor = DepartureMinimums::floor(&mins, &legs, idx).map(|m| m.1);
            new_want.push(departure_target_ft(plan, floor, ceiling, false, None).want_ft);
        }
        assert!(changes(&old_speed) >= 18, "the old rule reproduces the log: {}", changes(&old_speed));
        assert_eq!(changes(&new_speed), 1, "Vy released once, at 5,900");
        // After the check-in: one number, 6,000, all the way to D156J (past
        // it the climb's hold on the level commanded keeps it, see the
        // plant test below).
        let after: Vec<f64> = ticks
            .iter()
            .zip(&new_want)
            .filter(|(r, _)| r["t"].as_f64().unwrap() >= released && r["fix"] != "LAM")
            .map(|(_, w)| *w)
            .collect();
        assert!(after.iter().all(|w| (*w - 6000.0).abs() < 0.5), "{after:?}");
    }

    /// The owners in order: a minimum raises the plan, the clearance caps
    /// both until it is released, the pilot and the ground outrank the cap.
    #[test]
    fn the_clearance_caps_a_minimum_until_departure_clears_higher() {
        let t = departure_target_ft(5000.0, Some(6000.0), Some(5000.0), false, None);
        assert_eq!(t.want_ft, 5000.0);
        assert!(t.raised_by_minimum && t.minimum_capped && t.cap_limited);
        let t = departure_target_ft(5000.0, Some(6000.0), None, false, None);
        assert_eq!(t.want_ft, 6000.0, "released: the minimum");
        assert!(t.raised_by_minimum && !t.minimum_capped && !t.cap_limited);
        let t = departure_target_ft(22000.0, Some(6000.0), Some(5000.0), false, None);
        assert_eq!(t.want_ft, 5000.0);
        assert!(!t.raised_by_minimum && t.cap_limited);
        assert_eq!(departure_target_ft(9000.0, Some(6000.0), Some(5000.0), true, None).want_ft, 9000.0, "the pilot's");
        assert_eq!(departure_target_ft(5000.0, Some(6000.0), Some(5000.0), false, Some(5600.0)).want_ft, 5600.0, "the ground's");
        assert_eq!(departure_target_ft(7000.0, Some(6000.0), None, false, None).want_ft, 7000.0, "under the plan: nothing");
    }

    /// A minimum met stays met, keyed by fix (a splice renumbers legs); the
    /// floor holds met minimums until their fix is behind.
    #[test]
    fn a_minimum_met_stays_met_and_floors_the_target_until_its_fix() {
        let (mins, legs) = (mins(), legs());
        let mut m = DepartureMinimums::default();
        assert_eq!(m.unmet(&mins, &legs, 1, 4000.0), Some((2, 5000.0)));
        assert_eq!(m.unmet(&mins, &legs, 1, 4900.0), Some((4, 6000.0)), "ACORN met at 4,900");
        assert_eq!(m.unmet(&mins, &legs, 2, 5900.0), None, "D156J met at 5,900");
        assert_eq!(m.unmet(&mins, &legs, 3, 5750.0), None, "and stays met");
        // Renumbered by a splice ahead: still met.
        let mut spliced = legs.clone();
        spliced.insert(0, legs[0].clone());
        assert_eq!(m.unmet(&mins, &spliced, 4, 5750.0), None);
        assert_eq!(DepartureMinimums::floor(&mins, &legs, 0), Some((4, 6000.0)), "the highest ahead");
        assert_eq!(DepartureMinimums::floor(&mins, &legs, 4), Some((4, 6000.0)), "met, not flown");
        assert_eq!(DepartureMinimums::floor(&mins, &legs, 5), None, "behind us");
    }

    /// The VS wheel only in VS mode; a hold closing on our dial is left.
    #[test]
    fn the_vs_wheel_is_written_only_in_vs_mode() {
        assert_eq!(vs_action(true, false, 5800.0, 6000.0, 6000.0, 600.0, 900.0), VsAction::Write);
        // The recorded capture: ALT HOLD 183 ft under the 6,000 dial at 962 fpm.
        assert_eq!(vs_action(false, true, 5817.0, 6000.0, 6000.0, 549.0, 962.0), VsAction::Leave);
        assert_eq!(vs_action(false, true, 5990.0, 6000.0, 6000.0, 30.0, 0.0), VsAction::Leave);
        // Holding short of the dial and not closing: not a level-off.
        assert_eq!(vs_action(false, true, 5750.0, 6000.0, 6000.0, 750.0, 0.0), VsAction::Engage);
        // A new level far off the held one: VS.
        assert_eq!(vs_action(false, true, 6000.0, 8000.0, 8000.0, 1800.0, 0.0), VsAction::Engage);
        assert_eq!(vs_action(false, false, 5000.0, 8000.0, 8000.0, 1800.0, 0.0), VsAction::Engage);
    }

    /// The 737's vertical modes: VS through a 2 s transport delay and two
    /// 1.5 s lags (#150's fit); an ALT capture that takes the dial inside
    /// 0.19 min of the rate (the recorded 183 ft at 962 fpm) and holds it
    /// through the same delay; the VS wheel reads 0 outside VS mode; a VS
    /// engage syncs the wheel to the rate flown.
    struct Ap {
        alt: f64,
        vs: f64,
        lag1: f64,
        pipe: std::collections::VecDeque<f64>,
        vs_mode: bool,
        vs_dial: f64,
        alt_dial: f64,
        hold_ft: f64,
    }

    impl Ap {
        const DT: f64 = 0.1;
        fn new(alt: f64, vs: f64) -> Self {
            Ap {
                alt,
                vs,
                lag1: vs,
                pipe: std::iter::repeat_n(vs, 20).collect(),
                vs_mode: true,
                vs_dial: vs,
                alt_dial: 5000.0,
                hold_ft: 0.0,
            }
        }
        fn step(&mut self) {
            let cmd = if self.vs_mode {
                self.vs_dial
            } else {
                ((self.hold_ft - self.alt) * 3.0).clamp(-1500.0, 1500.0)
            };
            self.pipe.push_back(cmd);
            let late = self.pipe.pop_front().unwrap();
            self.lag1 += (late - self.lag1) * Self::DT / 1.5;
            self.vs += (self.lag1 - self.vs) * Self::DT / 1.5;
            self.alt += self.vs * Self::DT / 60.0;
            let off = self.alt_dial - self.alt;
            if self.vs_mode && off * self.vs >= 0.0 && off.abs() <= (self.vs.abs() * 0.19).max(20.0) {
                self.vs_mode = false;
                self.hold_ft = self.alt_dial;
            }
        }
        fn engage_vs(&mut self) {
            self.vs_mode = true;
            self.vs_dial = (self.vs / 100.0).round() * 100.0;
        }
        /// Write then read back, as `set_confirmed`.
        fn write_vs(&mut self, v: f64) -> bool {
            if self.vs_mode {
                self.vs_dial = v;
            }
            let read = if self.vs_mode { self.vs_dial } else { 0.0 };
            (read - v).abs() <= 1.0
        }
    }

    #[derive(Default, Debug)]
    struct Flown {
        speed: Vec<f64>,
        /// (alt, vs) every tick after 5,900 ft was first reached, to D156J.
        level: Vec<(f64, f64)>,
        alt_at_d156j: Option<f64>,
        vs_writes_in_alt: u32,
        unconfirmed: u32,
        final_alt: f64,
    }

    /// The follower's vertical over the plant, old chain or new, from the
    /// arm at 3,712 ft climbing 1,500 under the clearance's 5,000 (released
    /// 4 s later by the check-in, as recorded) to 180 s past LAM.
    fn fly(new: bool) -> Flown {
        let (mins, legs) = (mins(), legs());
        let mut ap = Ap::new(3712.0, 1500.0);
        let (gs, best, climb) = (230.0, 2500.0, 1800.0);
        let (mut idx, mut pos) = (0usize, 0.0_f64);
        let (mut carry, mut prev_cmd, mut peak) = (3712.0, 3712.0, 3712.0);
        let mut memory = DepartureMinimums::default();
        let (mut last_dial, mut last_vs): (Option<f64>, Option<f64>) = (None, None);
        let mut out = Flown::default();
        let mut reached = false;
        let mut past_lam_s = 0.0;
        for tick in 0..3000 {
            for _ in 0..5 {
                ap.step();
            }
            pos += gs * 0.5 / 3600.0;
            let alt = ap.alt;
            let fix_nm = |i: usize| ROUTE[i].1;
            while idx + 1 < legs.len() && fix_nm(idx) - pos <= CAPTURE_NM {
                if legs[idx].fix_ident == "D156J" {
                    out.alt_at_d156j = Some(alt);
                }
                idx += 1;
            }
            if idx >= 6 {
                past_lam_s += 0.5;
                if past_lam_s > 180.0 {
                    break;
                }
            }
            let dist = fix_nm(idx) - pos;
            let ceiling = (tick < 8).then_some(5000.0);
            let unmet = if new {
                memory.unmet(&mins, &legs, idx, alt)
            } else {
                unmet_minimum(&mins, &legs, idx, alt)
            };
            out.speed.push(if unmet.is_some() { 210.0 } else { 250.0 });
            let profile = DescentProfile { cruise_ft: Some(22000.0), peak_ft: peak, past_top: false, arrival_ceiling: false };
            let v = vertical_tick(&legs[idx], carry, prev_cmd, alt, Some(dist), gs, None, None, 2000.0, profile);
            peak = v.peak_ft;
            let want = if new {
                let floor = DepartureMinimums::floor(&mins, &legs, idx).map(|m| m.1);
                let dep = departure_target_ft(v.want_alt, floor, ceiling, false, None);
                prev_cmd = if dep.raised_by_minimum { v.prev_cmd.max(dep.want_ft) } else { v.prev_cmd };
                dep.want_ft
            } else {
                let w = cap_at_clearance(v.want_alt, ceiling, false, None);
                prev_cmd = v.prev_cmd;
                match unmet {
                    Some((_, m)) if m > w => m,
                    _ => w,
                }
            };
            carry = v.carry;
            let dial = dial_target_ft(want, alt, v.carry, None, None);
            if last_dial.is_none_or(|d| (d - dial).abs() > 50.0) {
                ap.alt_dial = dial;
                last_dial = Some(dial);
            }
            let cap = if unmet.is_some() { best } else { climb };
            let ff = if (want - v.want_alt).abs() < 1.0 { v.target_fpm } else { 0.0 };
            let vs_cmd = vs_law_fpm(want, alt, ff, 2000.0, cap);
            let write = if new {
                let mut a = if ap.vs_mode {
                    VsAction::Write
                } else {
                    vs_action(false, true, alt, want, dial, vs_cmd, ap.vs)
                };
                if a == VsAction::Engage {
                    last_vs = None;
                    ap.engage_vs();
                    if ap.vs_mode {
                        a = VsAction::Write;
                    }
                }
                a == VsAction::Write
            } else {
                if !ap.vs_mode && !vs_mode_capturing(alt, want, dial, vs_cmd) {
                    last_vs = None;
                    ap.engage_vs();
                }
                true
            };
            let vs_cmd = vs_dial_fpm(vs_cmd);
            if write {
                if last_vs.is_none_or(|l| (l - vs_cmd).abs() > 50.0) {
                    if !ap.vs_mode {
                        out.vs_writes_in_alt += 1;
                    }
                    if ap.write_vs(vs_cmd) {
                        last_vs = Some(vs_cmd);
                    } else {
                        last_vs = None;
                        out.unconfirmed += 1;
                    }
                }
            } else {
                last_vs = None;
            }
            reached |= alt >= 5900.0;
            if reached && idx <= 4 {
                out.level.push((alt, ap.vs));
            }
        }
        out.final_alt = ap.alt;
        out
    }

    /// Replayed through the plant: the old chain hunts round 5,900 with the
    /// speed flipping and VS written under ALT HOLD; the new one climbs to
    /// 6,000 once, holds it on the autopilot's own capture, leaves Vy once,
    /// and climbs on for LAM's 8,000.
    #[test]
    fn the_follower_levels_at_the_minimum_once_on_the_737s_modes() {
        let old = fly(false);
        let old_min_vs = old.level.iter().map(|p| p.1).fold(f64::MAX, f64::min);
        eprintln!(
            "old: {} speed flips, min vs {old_min_vs:.0}, {} VS writes in ALT, {} unconfirmed",
            changes(&old.speed),
            old.vs_writes_in_alt,
            old.unconfirmed
        );
        assert!(changes(&old.speed) >= 4, "the old chain hunts: {} speed flips", changes(&old.speed));
        assert!(old_min_vs < -800.0, "and dives for the 5,000 dial: {old_min_vs:.0} fpm");

        let new = fly(true);
        let min_vs = new.level.iter().map(|p| p.1).fold(f64::MAX, f64::min);
        let min_alt = new.level.iter().map(|p| p.0).fold(f64::MAX, f64::min);
        let max_alt = new.level.iter().map(|p| p.0).fold(f64::MIN, f64::max);
        eprintln!(
            "new: {} speed flips, vs {min_vs:.0}.., alt {min_alt:.0}..{max_alt:.0}, D156J {:?}, final {:.0}",
            changes(&new.speed),
            new.alt_at_d156j,
            new.final_alt
        );
        assert_eq!(changes(&new.speed), 1, "Vy to 250 once");
        assert!(min_vs > -300.0, "no dive: {min_vs:.0} fpm");
        assert!(min_alt > 5850.0 && max_alt < 6150.0, "level at 6,000: {min_alt:.0}..{max_alt:.0}");
        assert!(new.alt_at_d156j.is_some_and(|a| a >= 5900.0), "{:?}", new.alt_at_d156j);
        assert_eq!(new.vs_writes_in_alt, 0);
        assert_eq!(new.unconfirmed, 0);
        assert!(new.final_alt > 7800.0, "on for LAM: {:.0}", new.final_alt);
    }
}

/// #199: the arrival hands autoland its clean approach speed.
#[cfg(test)]
mod arrival_handoff_tests {
    use super::*;

    #[test]
    fn the_egnx_arrival_slows_to_210_before_the_end_of_the_route() {
        // EGBB-EGNX (2026-09-30): 250 kt at 3,000 ft, about 260 over the
        // ground, the 737's clean approach speed 210. The slow-down needs
        // 40 kt at 0.5 kt/s: 5.8 nm, plus the 2 nm lead.
        let mut begun = false;
        assert_eq!(arrival_handoff_cap(12.0, 250.0, 260.0, 210.0, &mut begun), None);
        assert!(!begun);
        assert_eq!(arrival_handoff_cap(7.7, 250.0, 260.0, 210.0, &mut begun), Some(210.0));
        // Held once begun, whatever the speed and distance say next.
        assert_eq!(arrival_handoff_cap(9.0, 215.0, 220.0, 210.0, &mut begun), Some(210.0));
        // Already slow: only the lead before the end.
        let mut slow = false;
        assert_eq!(arrival_handoff_cap(3.0, 200.0, 200.0, 210.0, &mut slow), None);
        assert_eq!(arrival_handoff_cap(1.9, 200.0, 200.0, 210.0, &mut slow), Some(210.0));
    }

    #[test]
    fn the_handoff_speed_is_handed_down_once() {
        set_next_arrival_handoff_kts(Some(210.25));
        assert_eq!(take_arrival_handoff_kts(), Some(210.25));
        assert_eq!(take_arrival_handoff_kts(), None);
        set_next_arrival_handoff_kts(Some(f64::NAN));
        assert_eq!(take_arrival_handoff_kts(), None);
    }
}
