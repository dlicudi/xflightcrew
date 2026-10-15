//! The ATC station living inside the copilot process — the merge of the
//! standalone `atc` binary's tower service into the one binary the pilot
//! actually flies with. One mic, one PTT, one speech arbiter; the tuned COM1
//! frequency plus the shape of the words decide who answers.
//!
//! Trust boundary: this module only ever *reads* the simulator (frequency,
//! weather, tail number) and returns text. It has no access to the tool
//! registry and can never actuate anything — ATC is the world, not the crew.
//! Compiler-enforced: the sim handle here is `SimView`, a read-only view
//! with no `set_*`/`run_command` on it at all.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::Instant;

use crate::speaker::StationSpeaker;
use crate::{atis, nav, tower};
use sim::datarefs;
use sim::xplane::SimView;

// The state this module publishes for the crew lives on the party line —
// one audited page. `pub(crate)` re-export so the world side (this module
// and its tests) writes them by their bare names; the crew side reads via
// `party_line`'s accessors, never these statics.
pub use crate::party_line::RegionHandoff;
pub(crate) use crate::party_line::{
    AircraftState, ArrivalHandoff, FlightId, FlightStrip, IfrFlight, RunwayTimes, SpokenOrder,
    StripRef, ARRIVAL_PIN,
    GO_AROUND_ORDERS,
    OWN_STRIP, PENDING_STATION, STRIPS, traffic_strip, traffic_strips,
    REGION_HANDOFF,
};

pub(crate) use crate::party_line::open_traffic_strip;

// The airline telephony and the ghost speaker live with the callsign
// resolver, one table for our flight id and everyone else's.
pub use crate::callsign::spoken_ghost;
use crate::callsign::{TELEPHONY, TELEPHONY_IATA};

/// Which of the station's frequencies COM1 is tuned to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Service {
    Tower,
    Approach,
    Departure,
    /// Ground movement (apt.dat 1053): push and start, taxi out, taxi in.
    Ground,
    /// Clearance delivery (apt.dat 1052): the departure clearance.
    Delivery,
}

impl Service {
    /// The voice this position speaks with — a different controller each.
    fn speaker(self) -> StationSpeaker {
        match self {
            Service::Tower => StationSpeaker::Tower,
            Service::Approach => StationSpeaker::Approach,
            Service::Departure => StationSpeaker::Departure,
            Service::Ground => StationSpeaker::Ground,
            Service::Delivery => StationSpeaker::Delivery,
        }
    }

    /// The position a speaker is, back again (a strip's `working` cell).
    fn of_speaker(s: StationSpeaker) -> Option<Service> {
        match s {
            StationSpeaker::Tower => Some(Service::Tower),
            StationSpeaker::Approach => Some(Service::Approach),
            StationSpeaker::Departure => Some(Service::Departure),
            StationSpeaker::Ground => Some(Service::Ground),
            StationSpeaker::Delivery => Some(Service::Delivery),
            _ => None,
        }
    }

    /// The position's name for the blackbox trace.
    fn name(self) -> &'static str {
        match self {
            Service::Tower => "tower",
            Service::Approach => "approach",
            Service::Departure => "departure",
            Service::Ground => "ground",
            Service::Delivery => "delivery",
        }
    }

    /// One of the AERODROME positions (tower, ground, delivery): views of
    /// the one desk that works the flight on the field, its tower FSM and
    /// worksheet shared, so a strip moves between them with its handoffs.
    fn aerodrome(self) -> bool {
        matches!(self, Service::Tower | Service::Ground | Service::Delivery)
    }
}

/// Live wind for one exchange, read once by the async wrappers so the routing
/// layer below them is pure (transcript in, reply out — it battle-tests dry,
/// with no simulator on the other end).
#[derive(Debug, Clone, Copy)]
pub struct Wx {
    pub wind_from_deg: f64,
    pub wind_kts: f64,
}

pub struct Station {
    apt: nav::AirportInfo,
    tower_khz: i64,
    /// The approach frequency (apt.dat row 1055) — the vectoring service.
    approach_khz: Option<i64>,
    /// The departure frequency (row 1056) — where the tower sends you after
    /// takeoff.
    departure_khz: Option<i64>,
    /// The ground movement frequency (row 1053): push and start, taxi out
    /// to the holding point, taxi in from the runway ([`Station::owner`]).
    /// `None`: the tower does it all, as before.
    ground_khz: Option<i64>,
    /// Clearance delivery (row 1052): the departure clearances. `None`:
    /// Ground gives them, or without Ground the tower.
    delivery_khz: Option<i64>,
    /// The strip on frequency: our flight's desk half (the worksheet the
    /// tower keeps for the aircraft it is talking to). A traffic flight's
    /// desk is swapped in only for the synchronous answer to its call
    /// ([`Station::respond_for`]).
    desk: StripDesk,
    /// The desks of the traffic strips this station has answered, parked
    /// while our own is on frequency. Never holds `FlightId::Own`.
    desks: std::collections::BTreeMap<FlightId, StripDesk>,
    /// The runway in use, held across calls: with the wind below 5 knots the
    /// into-wind pick is a coin toss, and the 2026-07-08 flight was cleared
    /// for takeoff on two seven, then re-cleared on zero niner two minutes
    /// later in the same calm. A runway change needs actual wind.
    runway_in_use: Option<String>,
    /// Challenge missing/wrong readbacks of runway-critical clearances
    /// (`XCOPILOT_ATC_READBACK`). The pending state still arms when this is
    /// off: the coach teaches the readback either way; only the challenges
    /// are gated.
    enforce_readback: bool,
    /// Phraseology dialect for this field's controllers (set by the owner
    /// from the ICAO prefix, like the voice casting). Defaults to ICAO.
    dialect: Dialect,
    /// The field speaks CAP 413 inside the ICAO dialect ([`speaks_cap413`]:
    /// "circuit height"), from the ICAO the station was loaded or given
    /// procedures by.
    cap413: bool,
    /// The CIFP directory and this field's ICAO, for the SID an IFR
    /// clearance names (set by the owner, like the dialect). `None` = no
    /// procedure data: IFR clearances say "fly runway heading".
    procedures: Option<(String, String)>,
    /// Every SID's exit fix, positioned, loaded off the reply path by
    /// [`Station::set_procedures`] (`None` until it lands, or with no nav
    /// files): the IFR clearance picks the SID that leaves toward the
    /// destination (`nav::sid_toward`), else the first in the file.
    sid_exits: std::sync::Arc<Mutex<Option<Vec<nav::SidExit>>>>,
    /// The pressure phrase our last call's context was given
    /// ([`Station::build_ctx`] reads the sim), for a traffic call's context,
    /// which reads nothing ([`Station::traffic_ctx`]). `None` until ours has
    /// been answered once with a pressure that read.
    last_qnh: Option<String>,
    /// The moment the call being answered was keyed, while a TRAFFIC call is
    /// answered ([`Station::answer_traffic`] sets it from the call's state
    /// and clears it after): the reply path's clocks (a takeoff hold's
    /// `armed_at`, the wake gate) read the producer's time, so a fleet on a
    /// test's clock is held and released on that clock. `None` (ours, always)
    /// is the wall clock, as before ([`Station::reply_now`]).
    reply_at: Option<Instant>,
}

/// The desk half of a flight's strip: what the station keeps, between calls,
/// about the ONE aircraft it is working (its tower FSM and its
/// worksheet). Owned by the [`Station`]; the shared half is the strip's
/// cells on the party line ([`crate::party_line::FlightStrip`]).
struct StripDesk {
    /// The strip this desk works: its replies write that strip's cells.
    strip: StripRef,
    /// The tower FSM for this flight.
    twr: tower::Tower,
    /// The strip a runway start was GIVEN (`note_caller_on_strip`): held for
    /// its later calls on the ground while no stream opposes it and its
    /// tailwind stays within [`ON_STRIP_TAILWIND_KTS`], where the wind pick's
    /// hysteresis would otherwise turn it round on the next call (#178).
    start_strip: Option<String>,
    /// The runway the tower last named to this flight on the ground (its
    /// zone, circuit or IFR clearance, a taxi, a line-up): a later reply on
    /// another runway states the change (`state_runway_change`, #178).
    told_runway: Option<String>,
    /// The departure controller has said "radar contact" to this flight.
    dep_checked_in: bool,
    /// The position the after-takeoff handoff sends the flight to
    /// (Departure, or Approach at a field without one): its first readback
    /// there is the check-in. `None` until a takeoff clearance arms it.
    handed_to: Option<Service>,
    /// This flight said it stays in the pattern ("request taxi for
    /// circuits", [`tower::wants_circuits`]): the tower keeps it, so no
    /// takeoff clearance arms a departure handoff and no radar watch runs.
    /// Set by each departure request that carries the words, cleared by a
    /// taxi, zone or IFR request that does not (a new departure says what it
    /// is again); the station is rebuilt for a new flight.
    circuits: bool,
    /// A circuit asked for at a field whose data says no circuits has been
    /// declined once ([`tower::circuit_declined_line`]): asked again, it is
    /// cleared (the "not normally" is the tower's discretion).
    circuits_declined: bool,
    /// The ATIS information letter the pilot last reported having ("with
    /// information Alpha"). When it matches the current broadcast, clearances
    /// skip the QNH — the whole point of an ATIS.
    pilot_info: Option<char>,
    /// The ATIS letter the tower told this flight is current ("information
    /// Bravo is current, QNH ..."), once, on the first reply that gave the
    /// pressure to a pilot who had not reported it.
    atis_told: Option<char>,
    /// The tower's pressure phrase as last given to this flight: the same
    /// pressure is given once per contact, not in every clearance of one
    /// exchange (the IFR clearance AND the taxi clearance both said it). A
    /// change of pressure is given again; "say QNH" is always answered.
    qnh_given: Option<String>,
    /// This flight's assigned discrete squawk, once a radar service (vectors,
    /// flight following) has issued one. Repeated by every squawk check;
    /// `None` means the VFR conspicuity 7000 is the answer.
    assigned_squawk: Option<i64>,
    /// The taxi route planned when the ctx was last built on the ground —
    /// published to `OWN_STRIP.taxi_route` iff the taxi clearance is actually issued.
    planned_route: Option<nav::TaxiRoutePlan>,
    /// The (exit, destination) bearings behind the last IFR clearance's SID,
    /// for its blackbox record when the readback is accepted.
    ifr_bearings: Option<(f64, f64)>,
    /// Why the last IFR clearance named its SID (or none), for the same
    /// record.
    ifr_sid_basis: Option<SidBasis>,
    /// The runway the last IFR clearance named, while its departure is still
    /// to fly: the runway in use is FROZEN on it, exactly as a taxi
    /// clearance freezes it. EGKK 2026-09-27: the clearance named 26R, the
    /// crew's "request taxi, runway two six left" then moved the pick, and
    /// ground and tower worked 26L under a SID for 26R. Retired airborne
    /// and by an arrival's calls.
    ifr_runway: Option<String>,
    /// A re-sequence the go-around watch made (the end given up, the end
    /// offered), said once in the answer to the crew's "going around" call
    /// (issue #162).
    resequenced: Option<(String, String)>,
    /// What the transmission being answered said, consumed by the pick that
    /// answers it.
    call: CallScratch,
}

/// The final the runway in use offers an arrival (`Station::final_geometry`):
/// the runway end, its threshold and course (degrees true), whether it has
/// an ILS, the intercept platform (ft MSL) and the join distance (nm).
struct FinalGeometry {
    ident: String,
    thr_lat: f64,
    thr_lon: f64,
    course_deg: f64,
    ils: bool,
    target_alt_ft: f64,
    join_nm: f64,
}

/// Approach's words to an instrument arrival (`instrument_approach_words`):
/// the transmission and the crew's readback of it, without the callsign.
pub(super) struct ApproachWords {
    line: String,
    readback: String,
}

/// Beyond this from the field an INSTRUMENT arrival calling the tower is
/// approach's, not yet the tower's: approach hands an established arrival
/// over at the join, eight to eleven miles out on an ILS (`ils_join_nm` plus
/// two), so twelve is past any handoff approach would have made.
pub(super) const TOWER_TAKES_ARRIVAL_NM: f64 = 12.0;

/// Per transmission: what the call being answered stated, consumed by the
/// pick that answers it.
#[derive(Default)]
struct CallScratch {
    /// How far the caller is from the field, nm, as it keys (`ctx_for_call`;
    /// `None` when its position did not read). The tower sends an instrument
    /// arrival that calls it from beyond [`TOWER_TAKES_ARRIVAL_NM`] to
    /// approach.
    field_nm: Option<f64>,
    /// The runway the arrival now calling named for itself, consumed by the
    /// pick that answers the call (`note_stated_runway`).
    stated_runway: Option<String>,
    /// The caller stating [`CallScratch::stated_runway`] stands ON that strip,
    /// on the ground: a runway start asking for the runway it is lined up on
    /// (`note_caller_on_strip`), consumed by the same pick.
    stated_on_strip: bool,
    /// The caller being answered is a DEPARTURE on the ground (`build_ctx`):
    /// the wind pick then skips an end with no way on at its threshold's
    /// half when another end fits the wind (LEMD 18R, 2026-10-05).
    departing: bool,
    /// The runway a "request runway ..." call named, canonical ("8L"), for
    /// the ctx that answers it (`note_stated_runway`, `apply_runway_request`).
    runway_request: Option<String>,
    /// That request resolved to a strip end the tower can hand out ("08L"),
    /// committed as the runway in use when the tower grants it.
    requested_ident: Option<String>,
    /// The runway a crossing request named, canonical ("13"), for the ctx
    /// that answers it (`note_stated_runway`, `judge_crossing`). Never the
    /// stated runway: the strip crossed is not the one departed from.
    crossing_request: Option<String>,
    /// That request resolved to a strip end of this field ("13") and
    /// judged: the reply notes it on the strip, cleared
    /// ([`crate::party_line::FlightStrip::crossing`]) or refused
    /// ([`crate::party_line::FlightStrip::crossing_waiting`]).
    crossing_ident: Option<String>,
    /// A push request asked to start too ("request push and start").
    push_start: bool,
    /// The way a push request's aircraft will face once pushed, degrees
    /// true, read off its stand's lane and its route on (`ctx_for_call`,
    /// [`push_facing_deg`]); `None` with no lane drawn.
    push_facing: Option<f64>,
    /// Our own `acf_m_max`, kg, read by `build_ctx` for a ground call of
    /// ours: whether its departure route must start ahead of the nose
    /// ([`facing_for_route`]). `None` unread, or a traffic call.
    own_mtow_kg: Option<f64>,
}

impl StripDesk {
    /// A fresh desk for our own flight: nothing heard from it yet.
    fn new() -> StripDesk {
        StripDesk::for_strip(StripRef::Own)
    }

    /// A fresh desk working this strip.
    fn for_strip(strip: StripRef) -> StripDesk {
        StripDesk {
            strip,
            twr: tower::Tower::new(),
            start_strip: None,
            told_runway: None,
            dep_checked_in: false,
            handed_to: None,
            circuits: false,
            circuits_declined: false,
            pilot_info: None,
            atis_told: None,
            qnh_given: None,
            assigned_squawk: None,
            planned_route: None,
            ifr_bearings: None,
            ifr_sid_basis: None,
            ifr_runway: None,
            resequenced: None,
            call: CallScratch::default(),
        }
    }
}

// The dialect lives with the spoken-number helpers (`atis`), so the pure
// phrasing layers (`tower`, `atis`) take it without reaching into the station.
pub use crate::atis::Dialect;

/// The dialect for a field, by ICAO prefix: K (CONUS), P (US Pacific/Alaska)
/// and C (Canada) speak FAA-style; everywhere else speaks ICAO. Coarse on
/// purpose — refine per-region as fields get flown.
pub fn dialect_for(icao: &str) -> Dialect {
    match icao.chars().next() {
        Some('K') | Some('P') | Some('C') => Dialect::Faa,
        _ => Dialect::Icao,
    }
}

/// The clearance's pressure phrase in the field's own money: "QNH one zero
/// two zero" (hectopascals) everywhere ICAO, "altimeter two niner niner two"
/// (inches, hundredths) at FAA fields.
pub fn spoken_pressure(pascals: f64, dialect: Dialect) -> String {
    match dialect {
        Dialect::Icao => {
            format!(
                "QNH {}",
                atis::spoken_digits(&format!("{:.0}", pascals / 100.0))
            )
        }
        Dialect::Faa => {
            let inhg_hundredths = (pascals / 3386.389 * 100.0).round();
            format!(
                "altimeter {}",
                atis::spoken_digits(&format!("{inhg_hundredths:.0}"))
            )
        }
    }
}

impl Station {
    /// The runway the tower is using (None until something picks one): read
    /// by the crew's traffic link so traffic spawns for the tower's runway.
    pub fn runway_in_use(&self) -> Option<&str> {
        self.runway_in_use.as_deref()
    }

    /// Load the station for `icao` from apt.dat. `None` (with a log line) when
    /// the field is missing or has no tower frequency — the copilot then runs
    /// exactly as before the merge.
    pub fn load(apt_path: &str, icao: &str) -> Option<Station> {
        let apt = match nav::find_airport(apt_path, icao) {
            Ok(Some(a)) => a,
            Ok(None) => {
                tracing::warn!(%icao, "ATC station: airport not in apt.dat; ATC off");
                return None;
            }
            Err(e) => {
                tracing::warn!(%icao, "ATC station: apt.dat scan failed ({e:#}); ATC off");
                return None;
            }
        };
        if apt.freq(nav::FREQ_TOWER).is_none() {
            tracing::warn!(%icao, "ATC station: no tower frequency in apt.dat; ATC off");
            return None;
        }
        Self::from_airport(apt).map(|mut st| {
            st.cap413 = speaks_cap413(icao);
            st
        })
    }

    /// Build the station from an already-parsed airport. `None` without a
    /// tower frequency. Split from [`Station::load`] so offline sweeps (the
    /// cross-airport battle test) can drive one apt.dat pass through every
    /// field instead of rescanning the 400 MB file per airport.
    pub fn from_airport(apt: nav::AirportInfo) -> Option<Station> {
        let twr_row = apt.freq(nav::FREQ_TOWER)?;
        tracing::info!(
            airport = %apt.name,
            mhz = twr_row.khz as f64 / 1000.0,
            "ATC station up: tower answers radio calls on this frequency"
        );
        let approach_khz = apt.freq(nav::FREQ_APPROACH).map(|f| f.khz);
        if let Some(k) = approach_khz {
            tracing::info!(mhz = k as f64 / 1000.0, "approach on frequency (vectors)");
        }
        let departure_khz = apt.freq(nav::FREQ_DEPARTURE).map(|f| f.khz);
        if let Some(k) = departure_khz {
            tracing::info!(
                mhz = k as f64 / 1000.0,
                "departure on frequency (handoff after takeoff)"
            );
        }
        // The ground positions, where the field publishes them, and only
        // on a frequency of their own: a row sharing the tower's channel
        // is the tower working them (the tower is resolved first anyway).
        let own = |k: i64| (k - twr_row.khz).abs() > 5;
        let ground_khz = apt.freq(nav::FREQ_GROUND).map(|f| f.khz).filter(|k| own(*k));
        let delivery_khz = apt
            .freq(nav::FREQ_CLEARANCE)
            .map(|f| f.khz)
            .filter(|k| own(*k) && ground_khz.is_none_or(|g| (k - g).abs() > 5));
        if let Some(k) = ground_khz {
            tracing::info!(mhz = k as f64 / 1000.0, "ground on frequency (push, taxi)");
        }
        if let Some(k) = delivery_khz {
            tracing::info!(mhz = k as f64 / 1000.0, "delivery on frequency (clearances)");
        }
        Some(Station {
            tower_khz: twr_row.khz,
            approach_khz,
            departure_khz,
            ground_khz,
            delivery_khz,
            apt,
            desk: StripDesk::new(),
            desks: std::collections::BTreeMap::new(),
            runway_in_use: None,
            enforce_readback: true,
            dialect: Dialect::Icao,
            cap413: false,
            procedures: None,
            sid_exits: Default::default(),
            last_qnh: None,
            reply_at: None,
        })
    }

    /// Gate the readback challenges on config (`XCOPILOT_ATC_READBACK`).
    pub fn set_enforce_readback(&mut self, on: bool) {
        self.enforce_readback = on;
    }

    /// Set the phraseology dialect (the owner resolves it from the ICAO
    /// prefix via [`dialect_for`] — the station never sees its own ICAO).
    pub fn set_dialect(&mut self, d: Dialect) {
        self.dialect = d;
    }

    /// Where the field's SIDs are: the CIFP directory and the field's ICAO
    /// (the station never sees its own identifier otherwise).
    /// The SIDs' exit fixes are positioned on a background thread (one scan
    /// of `earth_fix.dat` and `earth_nav.dat`), so the SID toward the
    /// destination is a lookup by the time anyone asks.
    pub fn set_procedures(&mut self, cifp_dir: &str, fix_path: &str, nav_path: &str, icao: &str) {
        self.cap413 = speaks_cap413(icao);
        self.procedures = Some((cifp_dir.to_string(), icao.to_uppercase()));
        let (cifp, fix, nav_p, icao) = (
            cifp_dir.to_string(),
            fix_path.to_string(),
            nav_path.to_string(),
            icao.to_uppercase(),
        );
        let apt = self.apt.clone();
        let cache = std::sync::Arc::clone(&self.sid_exits);
        *cache.lock().unwrap_or_else(|e| e.into_inner()) = None;
        std::thread::Builder::new()
            .name("sid-exits".into())
            .spawn(move || {
                let exits = nav::sid_exits(&cifp, &fix, &nav_p, &apt, &icao);
                tracing::info!(%icao, sids = exits.len(), "SID exits positioned for IFR clearances");
                *cache.lock().unwrap_or_else(|e| e.into_inner()) = Some(exits);
            })
            .ok();
    }

    /// [`Station::set_procedures`] done in this thread, for tests.
    #[cfg(test)]
    fn set_procedures_now(&mut self, cifp_dir: &str, fix_path: &str, nav_path: &str, icao: &str) {
        self.procedures = Some((cifp_dir.to_string(), icao.to_uppercase()));
        let exits = nav::sid_exits(cifp_dir, fix_path, nav_path, &self.apt, &icao.to_uppercase());
        *self.sid_exits.lock().unwrap_or_else(|e| e.into_inner()) = Some(exits);
    }

    /// The active field's spoken name ("Gibraltar") — the per-airport voice
    /// seed and the home-field check key off it.
    pub fn airport_name(&self) -> &str {
        &self.apt.name
    }

    /// The name the controller keys ("Gatwick" for "London Gatwick"):
    /// [`nav::AirportInfo::radio_name`].
    pub fn radio_name(&self) -> String {
        self.apt.radio_name()
    }
}

mod atis_watch;
mod briefing;
mod coach;
mod compliance;
mod ctx;
mod ifr;
mod new_flight;
mod party_chatter;
mod positions;
mod radar;
mod region;
mod respond;
mod routing;
mod runway_pick;
mod runway_book;
mod runways;
mod self_taxi;
mod sequence;
mod state;
mod strip;
mod takeoff_hold;
mod taxi;
mod tcas;
mod traffic;
mod traffic_desk;
mod wake;

pub use atis_watch::*;
pub use briefing::*;
pub use coach::*;
pub use compliance::*;
pub(crate) use ifr::*;
pub(crate) use respond::instrument_arrival;
pub use new_flight::*;
pub use party_chatter::*;
pub use positions::*;
pub use radar::*;
pub use region::*;
pub use routing::*;
pub use runway_book::*;
pub use runways::*;
pub use self_taxi::*;
pub use sequence::*;
pub use state::*;
pub use strip::*;
pub use takeoff_hold::*;
pub use taxi::*;
pub use tcas::*;
pub use traffic::*;
pub use traffic_desk::*;
pub use wake::*;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod book_tests;

#[cfg(test)]
mod strip_tests;

#[cfg(test)]
mod tick_tests;

#[cfg(test)]
mod traffic_tests;

#[cfg(test)]
mod arrival_tests;

#[cfg(test)]
mod positions_tests;

#[cfg(test)]
mod sequence_tests;
