//! The party line: the one page of state the world side (ATC — station.rs,
//! tower.rs) publishes for the crew side (copilot — main.rs, the monitor's
//! crew watches, taxi_guide.rs) to read.
//!
//! Boundary rules, both directions:
//!
//! - The WORLD writes these from its reply path and watch ticks. It never
//!   reads crew state and never actuates the aircraft — its only outputs
//!   are spoken text and this page.
//! - The CREW reads through the accessor functions below, never the statics,
//!   and never writes — with two deliberate exceptions, the `take_*`
//!   consumers, where the main loop drains a queue the world armed for it
//!   (a station swap, a released takeoff clearance).
//!
//! Everything else in station.rs is private to the world side. When the
//! crew needs a new fact, it gets a new accessor here — so "what can the
//! copilot see of ATC?" stays answerable on one page. Until the sides live
//! in separate crates this is convention, not compiler-checked; keep the
//! surface small enough to audit by eye.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard};

use crate::nav;
use crate::station::{ClearanceSnap, LandingHold, TakeoffHold, TaxiWatch, VectorPlan};
use crate::tower;

// ---- published state (world-side writes live in station.rs) ----

/// The standing clearance: FSM state, callsign, runway, the readback owed.
pub(crate) static CLEARANCE: Mutex<Option<ClearanceSnap>> = Mutex::new(None);

/// A radar controller's squawk nudge, armed by the squawk watch as it speaks
/// it: the crew acts on the instruction it heard ("recycle transponder",
/// "confirm squawk ..."), which it never did (EGPH 2026-09-20: two recycles,
/// nobody answered).
pub(crate) static SQUAWK_NUDGE: Mutex<Option<SquawkNudge>> = Mutex::new(None);

/// What the controller asked of the transponder: the code it wants, and
/// whether it asked to RECYCLE (the code shows, the box does not reply) or
/// to confirm the code (the digits are wrong).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SquawkNudge {
    pub code: i64,
    pub recycle: bool,
}

/// The taxi route the pilot is CLEARED on — feeds the compliance watch
/// (world) and the progressive taxi guide (crew).
pub(crate) static TAXI_ROUTE: Mutex<Option<TaxiWatch>> = Mutex::new(None);

/// Whether the current [`TAXI_ROUTE`] is an ARRIVAL route to a parking stand
/// (vs a departure to a holding point) — set alongside the route so the crew
/// phrases "stand ..." instead of "holding point ...". Kept parallel rather
/// than a `TaxiWatch` field to avoid churning every construction of it.
pub(crate) static TAXI_TO_STAND: Mutex<bool> = Mutex::new(false);

/// A takeoff clearance withheld for landing traffic — armed by the reply
/// path, released by the monitor once the final has stayed clear.
pub(crate) static TAKEOFF_HOLD: Mutex<Option<TakeoffHold>> = Mutex::new(None);

/// A landing clearance withheld for traffic on the runway ("continue
/// approach, traffic on the runway") -- armed by the reply path to a final
/// report, released by the monitor's traffic watch once the strip is clear or
/// its occupant will be off it before we arrive
/// (`station::landing_hold_tick`). LEMG 13, 2026-09-28: the tower cleared the
/// circuit to land with a departure lined up and cancelled it 0.8 s later.
pub(crate) static LANDING_HOLD: Mutex<Option<LandingHold>> = Mutex::new(None);

/// The landing clearance the monitor released (its spoken runway and the
/// instruction), for the station to sync its FSM before it answers the next
/// call (the readback): `Station::sync_landing_release`.
pub(crate) static PENDING_LANDING_RELEASE: Mutex<Option<(String, String)>> = Mutex::new(None);

/// An armed vectors-to-final plan — the approach controller's worksheet.
pub(crate) static VECTORS: Mutex<Option<VectorPlan>> = Mutex::new(None);

/// A station swap the main loop must consume (it owns the `Station`): the
/// new field's ICAO (for the voice casting) and airport, armed by
/// `region_tick` after the handoff line is spoken.
pub(crate) static PENDING_STATION: Mutex<Option<(String, nav::AirportInfo)>> = Mutex::new(None);

/// The TOWER frequency of the field the crew has committed to arrive at.
/// While it is set the enroute region may hand the station TO that field and
/// never away from it: a controller we are already talking to, inbound, is
/// not a controller a nearer strip may take from us. Set by the crew at the
/// inbound call, cleared when the arrival is over.
pub(crate) static ARRIVAL_PIN: Mutex<Option<i64>> = Mutex::new(None);

/// The crew's provider of what the sim cannot say about our callsign: the
/// pilot's override and the loaded plan's ATC callsign
/// (`callsign::provide_inputs`, registered once at boot). A function, never a
/// value: it reads the setting and the plan shelf when asked, so a replaced
/// or cleared plan is a new or no callsign with nothing here to go stale.
pub(crate) static CALLSIGN_INPUTS: Mutex<Option<fn() -> crate::callsign::Inputs>> =
    Mutex::new(None);

/// The crew's provider of the pilot's own airline telephony
/// (`callsign::provide_telephony`, registered once at boot): the raw setting
/// ("WAT=Walker; XYZ=Zulu Air"), read at the moment a callsign is spoken so an
/// edit applies to the next transmission. A function for the same reason as
/// [`CALLSIGN_INPUTS`]: nothing here can go stale.
pub(crate) static CALLSIGN_TELEPHONY: Mutex<Option<fn() -> String>> = Mutex::new(None);

/// The crew's provider of the route it FILED (`provide_filed_route`,
/// registered once at boot): the loaded plan's origin, destination and fix
/// idents, read when an IFR clearance is made up, so the tower clears the
/// SID that joins it (`nav::sid_for_filed_route`, the crew's own rule). A
/// function for the same reason as [`CALLSIGN_INPUTS`].
type FiledRouteProvider = fn() -> Option<FiledRoute>;
pub(crate) static FILED_ROUTE: Mutex<Option<FiledRouteProvider>> = Mutex::new(None);

/// A flight plan as filed, what a controller's strip would show: where from,
/// where to, and the fixes between (SID fixes included, as the navlog lists
/// them; the fields themselves left out).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FiledRoute {
    pub origin: String,
    pub dest: String,
    pub fixes: Vec<String>,
    /// The SID the plan was filed with, when it names one.
    pub sid: Option<String>,
    /// The filed cruise level, feet: no controller clears the departure
    /// higher (`station::capped_departure_climb`). `None` when not filed.
    pub cruise_ft: Option<i64>,
}

/// Register the crew's provider of [`FiledRoute`]. Called once at boot.
pub fn provide_filed_route(f: fn() -> Option<FiledRoute>) {
    *lock(&FILED_ROUTE) = Some(f);
}

/// The filed route from `origin` to `dest` right now; `None` without a
/// provider, without a plan, or for a plan between other fields (a
/// clearance to somewhere else is not this plan's).
pub fn filed_route_for(origin: &str, dest: &str) -> Option<FiledRoute> {
    let f = (*lock(&FILED_ROUTE))?;
    f().filter(|r| {
        r.origin.eq_ignore_ascii_case(origin) && r.dest.eq_ignore_ascii_case(dest) && !r.fixes.is_empty()
    })
}

/// Set when the monitor speaks the released takeoff clearance; the main
/// loop consumes it and syncs the tower FSM to what was said on frequency.
pub(crate) static PENDING_TAKEOFF_CLEARANCE: Mutex<bool> = Mutex::new(false);

/// Set when the monitor sequences a held departure onto the runway with a
/// line-up-and-wait; the main loop consumes it and syncs the tower FSM to
/// what was said on frequency, exactly like [`PENDING_TAKEOFF_CLEARANCE`].
pub(crate) static PENDING_LINE_UP: Mutex<bool> = Mutex::new(false);

/// Set when the monitor orders a go-around (traffic on the runway); the main
/// loop consumes it and voids the landing clearance in the tower FSM, so the
/// coach can never keep suggesting the readback of a cancelled clearance.
pub(crate) static PENDING_GO_AROUND: Mutex<bool> = Mutex::new(false);

/// Every go-around order the tower has given, counted and never reset: the
/// crew's loops each watch it through a [`GoAroundOrders`], so one reader
/// can never consume another's order. The FSM sync above is a `take_*` flag
/// for the main loop alone; the loops that FLY the order used to take that
/// same flag, and the main loop (polling every ~150 ms against autoland's
/// ~400 ms tick) won it: 15 orders in the logs, not one `ordered_go_around`.
pub(crate) static GO_AROUND_ORDERS: AtomicU64 = AtomicU64::new(0);

/// The runway the tower's LATEST go-around order re-sequenced the arrival to
/// ("go around, traffic on the runway ... expect runway one three"), with
/// that order's number in [`GO_AROUND_ORDERS`]; `None` when the latest order
/// named none. Written with the order, before its count moves, so a watch
/// that sees the order sees its runway. Read through
/// [`GoAroundOrders::expected_runway`]. LEMG, 2026-09-29 (sortie
/// 1790668215-full_auto): re-sequenced from 30 to 13 at t+649, the crew
/// flew 5.5 minutes of pattern to 30, because during a missed approach the
/// runway published with `Inbound` is only the runway in use (EGTE).
pub(crate) static RESEQUENCED_ORDER: Mutex<Option<(u64, String)>> = Mutex::new(None);

/// A CONDITIONAL line-up's condition, as the world sees it: `Some(false)`
/// while the named arrival is still on final, `Some(true)` once it is down on
/// the strip or gone, `None` when no conditional line-up stands. Published by
/// the release watch every tick, so the crew waits on the world's picture of
/// the traffic and never on its own guess.
pub(crate) static LINE_UP_CONDITION: Mutex<Option<bool>> = Mutex::new(None);

/// The after-takeoff handoff the tower spoke ("contact Malaga Departure on
/// one two one decimal zero two five"): the station's spoken name and its
/// frequency, armed once when the line goes out and consumed by the crew
/// that works the radio.
pub(crate) static DEPARTURE_HANDOFF: Mutex<Option<(String, i64)>> = Mutex::new(None);

/// The enroute handoff the region watch spoke ("radar services terminated,
/// squawk seven thousand, contact Malaga Tower on ..."): whom to call, on
/// what, and whether the discrete code goes back to conspicuity. Armed once
/// when the line goes out; consumed by whoever is flying (full auto reads it
/// back, dials 7000 and retunes; with the pilot flying the monitor coaches
/// once). It survives `Station::install` on purpose: the main loop installs
/// the NEW station on its very next pass, before the crew's loop has polled,
/// and an install that wiped it would lose every handoff. `uninstall` ends it.
pub(crate) static REGION_HANDOFF: Mutex<Option<RegionHandoff>> = Mutex::new(None);

/// The zone watch challenged us on the frequency ("aircraft in the Granada
/// control zone, you are inside controlled airspace without clearance, say
/// callsign and intentions"): the tower has never heard from this aircraft
/// and is asking. Armed each time the tuned line goes out; consumed by the
/// crew that works the radio, which answers with the inbound call when the
/// challenging tower is the destination's (a full-auto arrival flew the last
/// enroute leg through Granada's zone with three challenges unanswered,
/// 2026-09-15). The crew-side "we're inside the zone" variant, spoken by the
/// copilot when the radio is NOT on the tower, never arms it.
pub(crate) static ZONE_CHALLENGE: Mutex<bool> = Mutex::new(false);

/// What the enroute controller told us at the region boundary.
#[derive(Debug, Clone, PartialEq)]
pub struct RegionHandoff {
    /// The station to call, as spoken ("Malaga Tower").
    pub to: String,
    /// Its frequency in kHz.
    pub khz: i64,
    /// "Squawk seven thousand" was ordered: a radar service assigned a
    /// discrete code that is now void.
    pub squawk_vfr: bool,
    /// The IFR flight's discrete code, KEPT across the handoff (no squawk
    /// instruction): an IFR flight is handed unit to unit with its code.
    /// `None` for a VFR service ending.
    pub ifr_code: Option<i64>,
}

/// This flight holds an accepted IFR clearance: its code and callsign, from
/// the readback the tower accepted. Unlike [`IFR_CLEARANCE`] (the departure
/// field's worksheet, retired with that field) it is the FLIGHT's, so it
/// survives `Station::install`: the region handoff that follows the flight
/// must know it is IFR. LIEA-LEBL, 2026-09-27: an IFR-cleared 737 on 4501
/// was told "radar services terminated, squawk seven thousand" at FL200 and
/// set it. Retired at the landing (`station::note_aircraft`), by a ground
/// stand-down out of reach, by a new clearance offer and by `uninstall`,
/// never at a region switch.
pub(crate) static IFR_FLIGHT: Mutex<Option<IfrFlight>> = Mutex::new(None);

/// See [`IFR_FLIGHT`].
#[derive(Debug, Clone, PartialEq)]
pub struct IfrFlight {
    /// The discrete code the clearance assigned.
    pub code: i64,
    /// Our callsign as the tower heard it, for the handoff line.
    pub callsign: String,
    /// Airborne since the clearance: the next ground contact is the landing.
    pub airborne: bool,
}

/// The runway the APPROACH controller cleared you toward and handed to the
/// tower ("cleared visual/ILS approach runway X, contact Tower"). You still
/// owe the tower a "final" report before you're cleared to LAND — an approach
/// clearance is not a landing clearance. The crew's landing-clearance coach
/// reads it; the world clears it on the landing clearance and on touchdown, so
/// it never lingers into the next circuit.
pub(crate) static LANDING_HANDOFF: Mutex<Option<String>> = Mutex::new(None);

/// The discrete transponder code the controller assigned with the clearance,
/// so the crew can dial it into the transponder — the full-auto orchestrator
/// reads it after requesting clearance. Published by the tower on assignment,
/// cleared on `Station::uninstall`.
pub(crate) static ASSIGNED_SQUAWK: Mutex<Option<i64>> = Mutex::new(None);

/// The IFR clearance the tower issued AND heard read back correctly: the
/// destination, the SID, the runway, the initial altitude and the code.
/// Set only by the accepted readback (a clearance nobody read back is not
/// one the crew may fly), retired with the squawk: by `Station::install`
/// (a new field), `Station::uninstall` and the out-of-reach stand-down.
pub(crate) static IFR_CLEARANCE: Mutex<Option<tower::IfrClearance>> = Mutex::new(None);

fn lock<T>(m: &'static Mutex<T>) -> MutexGuard<'static, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

// ---- crew-side accessors ----

/// The tower FSM state of the standing clearance, if any.
/// The active tower's frequency (kHz), so the crew can tune COM1 to it before
/// calling — the full-auto orchestrator does this before requesting taxi.
pub fn home_tower_khz() -> Option<i64> {
    // A tower out of reach is no tower to tune (`station::note_aircraft`).
    if crate::station::station_unavailable() {
        return None;
    }
    crate::station::active_tower_khz()
}

/// A station reaches the aircraft: one is running and it is not out of
/// reach on the ground. False = the crew departs as from an uncontrolled
/// field ([`plan_self_taxi`]).
pub fn station_reaches() -> bool {
    !crate::station::station_unavailable() && crate::station::active_tower_khz().is_some()
}

/// The current destination is explicitly uncontrolled, rather than merely
/// being beyond the last station's ground range.
pub fn uncontrolled_arrival() -> bool {
    crate::station::uncontrolled_arrival()
}

/// Publish a crew-owned taxi route to a stand at an uncontrolled arrival.
pub fn plan_self_taxi_to_stand(
    apt_path: &str,
    lat: f64,
    lon: f64,
    psi_true: Option<f64>,
) -> Result<crate::station::SelfTaxiStand, String> {
    crate::station::plan_self_taxi_to_stand(apt_path, lat, lon, psi_true)
}

/// The crew's own taxi at an uncontrolled field, planned from the field's
/// taxi network and published like a controller's route
/// ([`crate::station::plan_self_taxi`]). A full apt.dat pass: off the
/// realtime loop.
pub fn plan_self_taxi(
    apt_path: &str,
    icao: Option<&str>,
    lat: f64,
    lon: f64,
    runway: Option<&str>,
    wind: Option<(f64, f64)>,
) -> Result<crate::station::SelfTaxi, String> {
    crate::station::plan_self_taxi(apt_path, icao, lat, lon, runway, wind)
}

pub fn clearance_state() -> Option<tower::State> {
    lock(&CLEARANCE).as_ref().map(|c| c.state)
}

/// The tower's book puts us on the strip we stand on (cleared onto it,
/// landed on it and not yet vacated, a runway start, a moving cleared
/// crossing): the runway-incursion watch stays silent. See
/// [`crate::station::runway_occupancy_authorised`].
pub fn runway_occupancy_authorised(lat: f64, lon: f64, gs_kts: f64) -> bool {
    crate::station::runway_occupancy_authorised(lat, lon, gs_kts)
}

/// The runway END of the standing clearance ("09R"), if one is known —
/// the auto-takeoff line-up crosscheck reads it against the strip the
/// aircraft is actually pointing down.
pub fn clearance_runway() -> Option<String> {
    lock(&CLEARANCE)
        .as_ref()
        .map(|c| c.runway_ident.clone())
        .filter(|r| !r.is_empty())
        // An uncontrolled departure's own runway stands in for a cleared
        // one: the line-up and the takeoff cross-check it the same way.
        .or_else(crate::station::self_taxi_runway)
}

/// The readback the tower is still waiting for, phrased ("Cleared to land
/// runway 09, November one seven two Sierra Papa."), `None` once it has
/// been heard. The crew's own approach reads it back when an automation is
/// flying: the tower's "negative, runway zero niner ... cleared to land"
/// was neither echoed nor obeyed while autoland flew 27 into a 21 kt
/// tailwind (LXGB, 2026-09-14).
pub fn readback_owed() -> Option<String> {
    lock(&CLEARANCE)
        .as_ref()
        .and_then(|c| c.pending_readback.clone())
        .filter(|r| !r.is_empty())
}

/// Read the cleared taxi route under its lock. Closure-scoped so a guard
/// can never be held across an await.
pub fn with_taxi_route<R>(f: impl FnOnce(Option<&TaxiWatch>) -> R) -> R {
    f(lock(&TAXI_ROUTE).as_ref())
}

/// True when the cleared taxi route is an ARRIVAL to a parking stand — the crew
/// calls the destination a "stand", not a "holding point".
/// The nearest parking position (apt.dat 1300 row, any kind) to a point:
/// metres away, the heading the aircraft parks on, the painted name. `None`
/// when no station is installed, its network has not loaded, or the field
/// publishes no stands; [`stands_known`] tells those apart from "none near".
/// The crew asks this before calling a tug: a pushback belongs to a stand,
/// and the bearing-only trigger alone called one from the middle of a
/// taxiway on every restart after an off-route stop (LEMG, 2026-09-12).
#[derive(Clone, Debug, PartialEq)]
pub struct StandFix {
    pub dist_m: f64,
    pub heading: f64,
    pub name: String,
}

pub fn nearest_stand(lat: f64, lon: f64) -> Option<StandFix> {
    let net = crate::station::taxi_net()?;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    net.stands
        .iter()
        .map(|s| {
            let dx = (s.lon - lon) * m_per_deg_lon;
            let dy = (s.lat - lat) * 111_320.0;
            StandFix {
                dist_m: (dx * dx + dy * dy).sqrt(),
                heading: s.heading,
                name: s.name.clone(),
            }
        })
        .min_by(|a, b| a.dist_m.total_cmp(&b.dist_m))
}

/// Is this point on the active field's drawn pavement? `None` when no
/// field is loaded or it draws no pavement: no data is never "grass".
/// The painted way out of the stand the aircraft is on, read backwards from
/// it (see `nav::TaxiNet::push_lane`); `None` with no field or no paint.
pub fn push_lane(lat: f64, lon: f64, psi_deg: f64) -> Option<Vec<(f64, f64)>> {
    crate::station::taxi_net()?.push_lane(lat, lon, psi_deg)
}

pub fn on_pavement(lat: f64, lon: f64) -> Option<bool> {
    crate::station::taxi_net()?.on_pavement(lat, lon)
}

/// Is this point on one of the active field's runway strips? `None` when no
/// field's runways are published: no data is never "off the runway". The
/// flight model's `LinedUp` evidence, which the live monitor never fed, so
/// a lined-up aircraft read "Holding" and its roll "TaxiOut" (LIBP 04 and
/// LEMG 13, 2026-09-26: the duty engine asked for a taxi and a line-up
/// under the running takeoff).
pub fn on_runway(lat: f64, lon: f64) -> Option<bool> {
    crate::station::on_active_runway_known(lat, lon)
}

/// The active station's runways as apt.dat has them (empty with no station):
/// the crew judges the runway it was cleared for against its own airframe
/// before it taxis, and picks the one to ask for when it does not fit.
pub fn field_runways() -> Vec<nav::Runway> {
    crate::station::active_runways()
}

/// Whether the active field publishes any parking positions at all.
pub fn stands_known() -> bool {
    crate::station::taxi_net().is_some_and(|n| !n.stands.is_empty())
}

pub fn taxi_to_stand() -> bool {
    *lock(&TAXI_TO_STAND)
}

/// The drive OFF the runway we have just landed on, planned from the field's
/// taxi network. `psi_true` is the rollout direction; `runway` is the end we
/// landed on.
///
/// This is the one accessor here that is not a published CLEARANCE, and that
/// is deliberate rather than a leak. The world owns the airport's geometry —
/// the taxi network is loaded once per station and the crew has no business
/// keeping a second copy — but vacating needs no permission from the tower:
/// it is the completion of the landing clearance the crew already holds. So
/// the crew asks the world a question about pavement and gets an answer, with
/// no controller in the loop and nothing to read back. The station can still
/// never actuate; this returns geometry.
pub fn plan_vacate(
    lat: f64,
    lon: f64,
    psi_true: f64,
    runway: &str,
) -> Option<crate::nav::VacatePlan> {
    crate::station::plan_vacate_from(lat, lon, psi_true, runway)
}

/// The discrete squawk the controller assigned, if one has been — the crew dials
/// it into the transponder after the clearance.
pub fn assigned_squawk() -> Option<i64> {
    *lock(&ASSIGNED_SQUAWK)
}

/// The IFR clearance this flight holds, read back and accepted: fly the
/// SID it names (when it serves the cleared runway) and climb to its
/// altitude. `None` before the readback, after a swap, and at a field the
/// flight left.
pub fn assigned_clearance() -> Option<IfrClearance> {
    lock(&IFR_CLEARANCE).clone()
}

pub use crate::tower::IfrClearance;

/// The flight is IFR (an accepted IFR clearance, not yet landed): its
/// discrete code. Survives the region handoffs the clearance does not
/// ([`IFR_FLIGHT`]).
pub fn ifr_flight_code() -> Option<i64> {
    lock(&IFR_FLIGHT).as_ref().map(|f| f.code)
}

/// The controller's last squawk nudge, consumed: the crew recycles or
/// re-dials once per call it heard.
pub fn take_squawk_nudge() -> Option<SquawkNudge> {
    lock(&SQUAWK_NUDGE).take()
}

/// A takeoff clearance is currently being withheld for traffic.
pub fn takeoff_hold_armed() -> bool {
    lock(&TAKEOFF_HOLD).is_some()
}

/// A landing clearance is currently being withheld for traffic on the
/// runway: the monitor's traffic watch must keep reading the picture to
/// release it (or order the go-around).
pub fn landing_hold_armed() -> bool {
    lock(&LANDING_HOLD).is_some()
}

/// The condition of a CONDITIONAL line-up ("behind the landing X, line up
/// behind"): `Some(false)` while X is still on final, `Some(true)` once it
/// has passed, `None` when the standing line-up is unconditional or there is
/// none. A crew about to cross the hold line waits on `Some(false)`.
pub fn line_up_condition_clear() -> Option<bool> {
    *lock(&LINE_UP_CONDITION)
}

/// Consume the after-takeoff handoff the tower spoke: the station to call
/// ("Malaga Departure") and its frequency in kHz. The crew that works the
/// radio reads it back, retunes, and checks in.
pub fn take_departure_handoff() -> Option<(String, i64)> {
    lock(&DEPARTURE_HANDOFF).take()
}

/// Consume the enroute handoff the region watch spoke: the tower to call, its
/// frequency, and whether the transponder goes back to seven thousand. The
/// crew that works the radio reads it back on the OLD frequency, sets the
/// code and retunes; a pilot flying by hand gets one nudge if neither moved.
pub fn take_region_handoff() -> Option<RegionHandoff> {
    lock(&REGION_HANDOFF).take()
}

/// Consume the zone watch's challenge ("say callsign and intentions"), spoken
/// on the frequency to an aircraft the tower has never heard from. Drained
/// unconditionally by the crew's loop each pass: an answer is owed the tower
/// it came from, and a challenge nobody answered is not kept for a later one.
pub fn take_zone_challenge() -> bool {
    std::mem::take(&mut *lock(&ZONE_CHALLENGE))
}

/// The callsign under approach vectors, if a plan is armed.
pub fn vectored_callsign() -> Option<String> {
    lock(&VECTORS).as_ref().map(|p| p.callsign.clone())
}

/// What the controller has actually TOLD US to fly, for a crew that intends to
/// comply. `vectored_callsign` only says a plan exists; this says what was
/// assigned, which is what an autoflight arrival needs to obey rather than
/// invent its own path alongside a controller who thinks it is being vectored.
///
/// `None` until the first vector has been transmitted — before that there is
/// nothing owed, and the crew flies its own navigation.
#[derive(Debug, Clone, PartialEq)]
pub struct AssignedVector {
    /// Last assigned heading, degrees. Already rounded to the ten the
    /// controller actually spoke, so the crew flies the SPOKEN number.
    pub heading_deg: f64,
    /// Intercept (platform) altitude, ft MSL.
    pub target_alt_ft: f64,
    /// The approach clearance has been issued ("cleared ILS approach, report
    /// established"), so the vectors are done and the crew owes the report.
    pub cleared: bool,
    /// Landing runway as spoken, for the report and the crosscheck.
    pub runway_spoken: String,
}

/// The vector currently owed, if a controller is working us and has spoken at
/// least one instruction.
pub fn assigned_vector() -> Option<AssignedVector> {
    let g = lock(&VECTORS);
    let p = g.as_ref()?;
    Some(AssignedVector {
        heading_deg: p.last_heading_deg()? as f64,
        target_alt_ft: p.target_alt_ft,
        cleared: p.is_cleared(),
        runway_spoken: p.runway_spoken.clone(),
    })
}

/// The runway the approach handed you toward while you still owe the tower a
/// "final" report — i.e. cleared for the APPROACH but not to LAND. `None` once
/// the tower clears the landing or the wheels are down.
pub fn landing_handoff_runway() -> Option<String> {
    lock(&LANDING_HANDOFF).clone()
}

/// Consume the armed station swap. Main loop only — it owns the `Station`.
/// A station swap is armed and the main loop has not built it yet. The crew
/// waits on this before keying a call meant for the NEW station (a new
/// flight's final report, `station::arm_new_flight`): routed a pass early it
/// reaches the old tower and its worksheet.
pub fn station_swap_pending() -> bool {
    lock(&PENDING_STATION).is_some()
}

pub fn take_pending_station() -> Option<(String, nav::AirportInfo)> {
    lock(&PENDING_STATION).take()
}

/// A new flight exists (the sim repositioned us for a setup): everything the
/// world published for the flight that ended is retired, before any loop
/// reads it, and the old station publishes nothing until the new one is up
/// ([`crate::station::retire_for_new_flight`]). The crew's one write here,
/// like the `take_*` consumers: it ends a flight, it clears nothing a
/// controller of THIS flight said.
pub fn retire_for_new_flight() {
    crate::station::retire_for_new_flight()
}

/// Consume the released-takeoff-clearance flag. Main loop only.
pub fn take_pending_takeoff_clearance() -> bool {
    std::mem::take(&mut *lock(&PENDING_TAKEOFF_CLEARANCE))
}

/// Consume the sequenced-line-up flag. Main loop only.
pub fn take_pending_line_up() -> bool {
    std::mem::take(&mut *lock(&PENDING_LINE_UP))
}

/// Consume the go-around-ordered flag. Main loop only (the tower FSM sync):
/// a loop that FLIES the order watches it through [`GoAroundOrders`].
pub fn take_pending_go_around() -> bool {
    std::mem::take(&mut *lock(&PENDING_GO_AROUND))
}

/// One crew loop's watch on the tower's go-around orders. Armed when the
/// loop starts flying (every order given before is history: an order shouted
/// at an earlier approach must not balk this one), then asked each tick.
/// Each watch sees each order once, whoever else is watching, and reading it
/// takes nothing from the main loop's FSM sync. LEMG 31 (sortie
/// 1790584715-full_auto, 2026-09-28): the order came on the circuit's final
/// at 736 ft; nothing in the circuit read it, and the flag was gone before
/// autoland armed at 2 nm, which drained whatever was left anyway.
#[derive(Debug, Clone, Copy)]
pub struct GoAroundOrders {
    seen: u64,
    /// The count when the watch was armed: an order at or before it is
    /// history.
    armed: u64,
}

impl GoAroundOrders {
    /// Start watching: only orders given from now on count.
    pub fn arm() -> Self {
        let now = GO_AROUND_ORDERS.load(Ordering::SeqCst);
        Self {
            seen: now,
            armed: now,
        }
    }

    /// The runway the latest go-around order given since this watch was
    /// armed re-sequenced us to ("expect runway one three"), if it named
    /// one. Takes nothing: the order itself is still [`Self::take`]'s.
    pub fn expected_runway(&self) -> Option<String> {
        let now = GO_AROUND_ORDERS.load(Ordering::SeqCst);
        lock(&RESEQUENCED_ORDER)
            .as_ref()
            .filter(|(n, _)| *n > self.armed && *n <= now)
            .map(|(_, rwy)| rwy.clone())
    }

    /// Whether the tower has ordered a go-around since the last look.
    pub fn take(&mut self) -> bool {
        let now = GO_AROUND_ORDERS.load(Ordering::SeqCst);
        let fresh = now != self.seen;
        self.seen = now;
        fresh
    }
}
