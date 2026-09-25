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

/// The first departure minimum still ahead of the aircraft and not yet met:
/// (leg index of its fix, minimum ft). Legs before `idx` are behind us.
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
            .filter(|m| alt_ft < m.min_ft)
            .map(|m| (i, m.min_ft))
    })
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
pub fn apply_arrival_constraints(legs: &mut [ProcLeg], cons: &[crate::nav::StarConstraint]) -> usize {
    let mut changed = 0;
    let mut prev: Option<f64> = None;
    for leg in legs.iter_mut() {
        let Some(alt) = leg.alt1_ft else { continue };
        let own = restriction_at(cons, leg);
        let mut want = own.map_or(alt, |c| c.clamp_ft(alt));
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
        let c = max + d * ft_per_nm;
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
        let c = alt + d * ft_per_nm;
        ceiling = Some(ceiling.map_or(c, |x: f64| x.min(c)));
    }
    ceiling
}

/// Descent gradient the ceiling is planned at, as a share of what the law
/// flies: the rest is margin for the deceleration a speed limit asks for.
const ARRIVAL_GRADIENT_SHARE: f64 = 0.85;

/// Feet per nm the arrival ceiling is planned on at this groundspeed.
pub fn arrival_ft_per_nm(descent_fpm: f64, gs_kts: f64) -> f64 {
    ARRIVAL_GRADIENT_SHARE * descent_fpm / (gs_kts.max(60.0) / 60.0)
}

/// Knots a second the governor is planned to lose speed at in a descent.
const ARRIVAL_DECEL_KTS_S: f64 = 0.5;
/// Slowed this far before the fix, nm, on top of the deceleration itself.
const ARRIVAL_SPEED_LEAD_NM: f64 = 2.0;

/// The arrival's speed limit now: the lowest of every limit already passed
/// (a limit holds from its fix on) and every one ahead whose slow-down has
/// begun, at [`ARRIVAL_DECEL_KTS_S`] from `ias_kts` plus the lead. `(knots,
/// fix)`; `None` when nothing applies yet.
pub fn arrival_speed_cap(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    idx: usize,
    lat: f64,
    lon: f64,
    ias_kts: f64,
    gs_kts: f64,
) -> Option<(f64, String)> {
    let mut cap: Option<(f64, String)> = None;
    for (k, leg) in legs.iter().enumerate() {
        let Some(kts) = restriction_at(cons, leg).and_then(|c| c.max_kts) else {
            continue;
        };
        let applies = k < idx || {
            let d = distance_to_leg_nm(lat, lon, legs, idx, k);
            let slow_nm = (ias_kts - kts).max(0.0) / ARRIVAL_DECEL_KTS_S * gs_kts.max(60.0) / 3600.0;
            d <= slow_nm + ARRIVAL_SPEED_LEAD_NM
        };
        if applies && cap.as_ref().is_none_or(|(c, _)| kts < *c) {
            cap = Some((kts, leg.fix_ident.clone()));
        }
    }
    cap
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
pub fn descent_target_ft(
    leg_alt: f64,
    carry: f64,
    alt_ft: f64,
    dist_nm: Option<f64>,
    gs_kts: f64,
    descent_fpm: f64,
) -> f64 {
    let Some(dist_nm) = dist_nm else {
        return leg_alt;
    };
    let Some(tod_nm) = descent_point_nm(leg_alt, alt_ft, gs_kts, descent_fpm) else {
        return leg_alt;
    };
    if dist_nm <= tod_nm {
        return leg_alt;
    }
    if carry <= alt_ft + 100.0 {
        carry
    } else {
        (alt_ft / 100.0).round() * 100.0
    }
}

/// Where the descent to `leg_alt` begins: miles BEFORE the fix, at the rate
/// the VS law flies and the groundspeed we have, plus the lead. `None` when
/// there is no descent to pace (the constraint is not more than 100 ft below
/// us). The one formula behind [`descent_target_ft`] and [`planned_tod`].
pub fn descent_point_nm(leg_alt: f64, alt_ft: f64, gs_kts: f64, descent_fpm: f64) -> Option<f64> {
    let drop_ft = alt_ft - leg_alt;
    if drop_ft <= 100.0 {
        return None;
    }
    Some(drop_ft / descent_fpm.max(1.0) * (gs_kts.max(60.0) / 60.0) + TOD_LEAD_NM)
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

/// What a descent that is running away with the speed gets: whether the
/// speedbrake should be out, and the fraction of the commanded rate to fly.
///
/// EGPH-EGTE, 2026-09-21: 2,000 fpm down with the governor's lever already at
/// idle and nothing else to slow with. 258 kt through ten thousand against a
/// 250 target, 267 at six, 278 at the last fix, where the approach armed. A
/// governor with no drag device can only watch a descent accelerate. A crew
/// pulls the speedbrake, and if that is not enough it shallows the descent:
/// the speed is owed to the approach that starts at the bottom, the height
/// can be lost later. Pure; `out` is the brake's state going in.
pub fn descent_drag(
    out: bool,
    ias_kts: f64,
    target_kts: Option<f64>,
    vs_cmd_fpm: f64,
) -> (bool, f64) {
    let Some(target) = target_kts else {
        return (false, 1.0);
    };
    if vs_cmd_fpm > -DRAG_MIN_DESCENT_FPM {
        return (false, 1.0);
    }
    let over = ias_kts - target;
    let want_out = if out {
        over > DRAG_IN_KTS
    } else {
        over > DRAG_OUT_KTS
    };
    let scale = if over > 20.0 {
        0.5
    } else if over > 12.0 {
        0.75
    } else {
        1.0
    };
    (want_out, scale)
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
) -> Vertical {
    let leg_alt = commanded_alt_ft(leg, carry);
    let mut want_alt = descent_target_ft(leg_alt, prev_cmd, alt_ft, dist_nm, gs_kts, descent_fpm);
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
    // An approach to a MAP short of the runway is flown TO the MAP: its final
    // is the published valley track, and the runway's extended centreline
    // from the FAF lies across the slopes beside it (LSZS 03: two miles off,
    // toward Piz Corvatsch).
    if legs.last().is_some_and(|l| l.is_map) {
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
            crate::tools::speedhold::release();
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

/// Share the circuit's turn reversal law, preserving the slew anchor on failure.
async fn steer_heading(
    xp: &dyn crate::xplane::XPlane,
    bug: &mut f64,
    target: f64,
    heading: f64,
    bank: f64,
) {
    let next = crate::circuit::bug_toward(*bug, target, heading, bank, 40.0);
    if xp.set_float(HDG_BUG, next).await.is_ok() {
        *bug = next;
    }
}

async fn fly(xp: XPlaneClient, base_url: String, voice: Option<Voice>, plan: Plan) {
    let Plan {
        name,
        legs,
        end,
        vy_kts,
        cruise_ias_kts,
        transition_altitude_ft,
        climb_minimums,
        arrival_constraints,
        visual_deg,
        missed_legs,
    } = plan;
    let handoff = handoff_index(&legs);
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
    let mut star_ceiling_logged: Option<usize> = None;
    // The one descent a pilot-held altitude is reminded about: (the pilot's
    // altitude, the fix). The plan's later descents under the same held
    // altitude are silent; a new altitude from the pilot is a new hold.
    let mut held_leg: Option<(f64, String)> = None;
    if !arrival_constraints.is_empty() {
        crate::blackbox::record(
            "procedure",
            serde_json::json!({"event": "star_constraints", "constraints": arrival_constraints.iter().map(|c|
                serde_json::json!({"fix": c.fix, "min_ft": c.min_ft, "max_ft": c.max_ft, "max_kts": c.max_kts})).collect::<Vec<_>>()}),
        );
    }
    let mut idx = 0usize;
    let mut closure: Option<(usize, crate::circuit::Closure)> = None;
    let mut stalls = 0u32;
    let mut progress_s = 0.0;
    let mut tick_clock = std::time::Instant::now();
    // The FMS is sequenced once per leg, when the follower first steers at it.
    let mut fms_leg: Option<usize> = None;
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
    let mut rate_samples = 0u32;
    let mut said_unable: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut said_warned_unable = 0usize;
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
        let unmet = unmet_minimum(&climb_minimums, &legs, idx, alt_ft);
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
        let unmet_judged = unmet.filter(|_| rate_samples >= MIN_RATE_SAMPLES);
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
        if let Some(planned) = power_hold_target(&end, cruise_ias_kts) {
            // A departure minimum ahead is climbed at Vy, the steepest the
            // airframe goes up: at cruise speed the SF50 made 375 ft a mile
            // where MOGT3H asks 600 (LOWI, 2026-09-24).
            let planned = if unmet.is_some() && vy_kts > 0.0 { vy_kts } else { planned };
            // The STAR's speed limits: slowed in time for each, held after.
            let star_cap = arrival_speed_cap(&legs, &arrival_constraints, idx, lat, lon, ias, gs * MS_TO_KTS);
            let planned = match &star_cap {
                Some((kts, fix)) if *kts < planned => {
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
                    *kts
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
                let flap = crate::autoland::flap_within_limits(1.0, ias, &crate::aircraft::flap_limits());
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
            let want = crate::aircraft::floored_target(
                crate::tools::speedhold::airspace_limited(alt_ft, planned),
                floor,
            );
            if crate::tools::speedhold::flight_target().is_some_and(|t| (t - want).abs() > 0.5) {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({
                        "event": "speed_limit", "alt_ft": alt_ft,
                        "planned": planned, "target": want,
                    }),
                );
                crate::tools::speedhold::arm_flight_with_floor(base_url.clone(), voice.clone(), want, floor);
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
        let crab = match (psi_v, hpath_r.ok().filter(|v| v.is_finite())) {
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
        let target_mag = if let Some(brg_true) = bearing_to_fix(lat, lon, leg) {
            Some(true_to_mag(brg_true + crab, mag_var))
        } else {
            // course is already magnetic; add crab to hold it as a track.
            leg.course_mag.map(|c| (c + crab).rem_euclid(360.0))
        };
        if let Some(d) = dist_nm.filter(|_| !leg_captured(leg, dist_nm, alt_ft, capture_nm)) {
            if closure.as_ref().is_none_or(|(i, _)| *i != idx) {
                closure = Some((idx, crate::circuit::Closure::new(d * 1852.0, progress_s)));
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
            steer_heading(&xp, &mut bug_mag, t, true_to_mag(psi, mag_var), phi).await;
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
        let grad = arrival_ft_per_nm(descent_fpm(), gs_kts);
        let ceiling = match (
            arrival_ceiling_ft(&legs, &arrival_constraints, idx, grad),
            profile_ceiling_ft(&legs, idx, grad),
        ) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        };
        let leg_for_vertical = match (leg.alt1_ft, ceiling) {
            (Some(a), Some(c)) if c < a - 50.0 => {
                if star_ceiling_logged != Some(idx) {
                    star_ceiling_logged = Some(idx);
                    crate::blackbox::record(
                        "procedure",
                        serde_json::json!({"event": "star_ceiling", "idx": idx, "fix": leg.fix_ident,
                                           "plan_ft": a, "ceiling_ft": c}),
                    );
                }
                let mut l = leg.clone();
                l.alt1_ft = Some((c / 100.0).floor() * 100.0);
                star_leg = l;
                &star_leg
            }
            _ => leg,
        };
        let v = vertical_tick(
            leg_for_vertical,
            carry_alt,
            prev_cmd_alt,
            alt_ft,
            dist_nm,
            gs_kts,
            pilot_altitude(),
            terrain_floor_ahead_ft().map(|f| floor_indicated(f, true_alt_ft, alt_ft)),
            descent_fpm(),
        );
        // The leg to a MAP flies the published path, rate fed forward
        // (`map_path`); every other leg the constraint law above.
        let path = map_path(leg, dist_nm, gs_kts);
        let want_alt = path.map_or(v.want_alt, |(a, _)| a);
        carry_alt = v.carry;
        prev_cmd_alt = v.prev_cmd;
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
        if !set_confirmed(
            "procedure",
            &xp,
            ALT_DIAL,
            want_alt,
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
        let climb_cap = if unmet_minimum(&climb_minimums, &legs, idx, alt_ft).is_some() {
            best_climb_fpm()
        } else {
            climb_fpm()
        };
        let mut vs_cmd = match path {
            Some((path_ft, ff_fpm)) => (ff_fpm + (path_ft - alt_ft) * 2.0)
                .clamp(-descent_fpm().max(ff_fpm.abs() * 1.4), 300.0),
            None => ((want_alt - alt_ft) * 3.0).clamp(-descent_fpm(), climb_cap),
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
        // A descent that outruns the governor (`descent_drag`): speedbrake
        // where the airframe has one, a shallower rate either way.
        {
            let ours = DRAG_OURS.load(Ordering::SeqCst);
            let (want_out, scale) =
                descent_drag(ours, ias, crate::tools::speedhold::flight_target(), vs_cmd);
            vs_cmd *= scale;
            if want_out != ours && crate::takeoff_config::has_speedbrake() {
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
                        crate::blackbox::record(
                            "procedure",
                            serde_json::json!({"event": "speedbrake", "out": want_out,
                                "ias": ias, "target": crate::tools::speedhold::flight_target(),
                                "vs_cmd": vs_cmd}),
                        );
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
        if vs_st <= 0.5 {
            // VS mode gone NEAR the target means the preselect CAPTURED —
            // its hold is the clean level-off, leave it flying. Far from the
            // target it's a dropped mode: re-engage and rewrite the dial.
            if (want_alt - alt_ft).abs() > 100.0 {
                last_vs_cmd = None; // engage can sync the dial even if the response is lost
                if xp.run_command(CMD_VS_MODE).await.is_err() {
                    continue;
                }
            }
        }
        if !set_confirmed("procedure", &xp, VS_DIAL, vs_cmd, &mut last_vs_cmd, 40.0).await {
            continue;
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
                "agl_ft": agl_ft, "crab": crab, "psi": psi_v, "bug": bug_mag, "tgt": target_mag,
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
            if idx >= hi && leg_captured(leg, dist_nm, alt_ft, capture_nm) {
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

        if leg_captured(leg, dist_nm, alt_ft, capture_nm) {
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
        }
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
    /// thousand against 250, 278 at the last fix.
    #[test]
    fn a_descent_that_outruns_the_governor_gets_drag_then_a_shallower_rate() {
        // Through ten thousand, 8 over: not yet. 9 over: brakes.
        assert_eq!(
            descent_drag(false, 258.0, Some(250.0), -2000.0),
            (false, 1.0)
        );
        assert_eq!(
            descent_drag(false, 259.0, Some(250.0), -2000.0),
            (true, 1.0)
        );
        // Out, they stay out until the speed is nearly back...
        assert_eq!(descent_drag(true, 254.0, Some(250.0), -2000.0), (true, 1.0));
        assert_eq!(
            descent_drag(true, 252.0, Some(250.0), -2000.0),
            (false, 1.0)
        );
        // ...and a runaway shallows the descent as well.
        assert_eq!(
            descent_drag(true, 267.0, Some(250.0), -2000.0),
            (true, 0.75)
        );
        assert_eq!(descent_drag(true, 278.0, Some(250.0), -2000.0), (true, 0.5));
        // Level, climbing, or with no governor target: nothing.
        assert_eq!(descent_drag(true, 278.0, Some(250.0), 0.0), (false, 1.0));
        assert_eq!(descent_drag(false, 278.0, Some(250.0), 800.0), (false, 1.0));
        assert_eq!(descent_drag(false, 278.0, None, -2000.0), (false, 1.0));
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

    #[tokio::test]
    async fn follower_rolls_out_before_reversing_and_preserves_failed_slew_anchor() {
        let io = DialIo {
            value: std::sync::Mutex::new(302.4),
            writes: std::sync::atomic::AtomicUsize::new(0),
            failure: std::sync::atomic::AtomicUsize::new(0),
        };
        let mut bug = 302.4;
        steer_heading(&io, &mut bug, 276.5, 300.5, 4.3).await;
        assert!((bug - 300.5).abs() < 0.01);
        assert!((*io.value.lock().unwrap() - 300.5).abs() < 0.01);
        io.failure.store(1, Ordering::SeqCst);
        steer_heading(&io, &mut bug, 276.5, 300.5, 2.0).await;
        assert!((bug - 300.5).abs() < 0.01);
        io.failure.store(0, Ordering::SeqCst);
        steer_heading(&io, &mut bug, 276.5, 300.5, 2.0).await;
        assert!((bug - 276.5).abs() < 0.01);
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
    /// ground, is 11.4 nm before the fix (10.4 of descent plus the lead).
    #[test]
    fn descent_point_is_one_formula() {
        let tod = descent_point_nm(3_143.0, 7_000.0, 113.0, GA_VS_FPM).unwrap();
        assert!((tod - 11.38).abs() < 0.05, "{tod}");
        // The law starts down exactly there, not before.
        assert_eq!(
            descent_target_ft(3_143.0, 7_000.0, 7_000.0, Some(tod + 0.1), 113.0, GA_VS_FPM),
            7_000.0
        );
        assert_eq!(
            descent_target_ft(3_143.0, 7_000.0, 7_000.0, Some(tod - 0.1), 113.0, GA_VS_FPM),
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
        let grad = arrival_ft_per_nm(2_000.0, 300.0);
        assert!((grad - 340.0).abs() < 1.0, "{grad}");
        let c = arrival_ceiling_ft(&legs, &cons, 2, grad).unwrap();
        assert!((c - (7_000.0 + 15.0 * 340.0)).abs() < 30.0, "MIDDL {c}");
        // Further out the ceiling is above the plan and changes nothing...
        let c = arrival_ceiling_ft(&legs, &cons, 1, grad).unwrap();
        assert!(c > 20_000.0, "HAVEN {c}");
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
                let v = vertical_tick(&l, carry, prev, alt, Some(d), gs, None, None, rate);
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

    #[test]
    fn a_stars_speed_limits_start_in_time_and_hold_after() {
        let (legs, cons) = star_route();
        // 20 nm short of HAVEN at 290 kt: nothing yet.
        assert_eq!(arrival_speed_cap(&legs, &cons, 1, 10.0 / 60.0, 0.0, 290.0, 400.0), None);
        // 40 kt to lose at 0.5 kt/s is 80 s, 8.9 nm at 400 kt, plus 2.
        let near = arrival_speed_cap(&legs, &cons, 1, 20.0 / 60.0, 0.0, 290.0, 400.0);
        assert_eq!(near, Some((250.0, "HAVEN".to_string())));
        // Past HAVEN the 250 holds; nearing TARTN its 230 takes over.
        assert_eq!(arrival_speed_cap(&legs, &cons, 2, 31.0 / 60.0, 0.0, 250.0, 300.0).unwrap().0, 250.0);
        assert_eq!(arrival_speed_cap(&legs, &cons, 3, 66.0 / 60.0, 0.0, 250.0, 300.0).unwrap().0, 230.0);
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
        assert!(ga > 200.0 && jet < 90.0, "{ga:.0} nm vs {jet:.0} nm");
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
        // TOD: 2,491 ft at 700 fpm is 3.6 min, 11.9 nm at 200 kt, plus the lead.
        assert_eq!(
            descent_target_ft(1397.0, 3900.0, 3888.0, Some(13.5), 200.0, GA_VS_FPM),
            3900.0
        );
        assert_eq!(
            descent_target_ft(1397.0, 3900.0, 3888.0, Some(12.0), 200.0, GA_VS_FPM),
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
}
