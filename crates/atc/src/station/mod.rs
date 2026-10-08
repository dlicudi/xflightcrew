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
    AircraftState, FlightId, FlightStrip, IfrFlight, StripRef, ARRIVAL_PIN, GO_AROUND_ORDERS,
    OWN_STRIP, PENDING_STATION, STRIPS, traffic_strip, traffic_strips,
    REGION_HANDOFF,
};

// The airline telephony and the ghost speaker live with the callsign
// resolver, one table for our flight id and everyone else's.
#[cfg(test)]
pub(crate) use crate::party_line::open_traffic_strip;
pub use crate::callsign::spoken_ghost;
use crate::callsign::{TELEPHONY, TELEPHONY_IATA};

/// Which of the station's frequencies COM1 is tuned to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Service {
    Tower,
    Approach,
    Departure,
}

impl Service {
    /// The voice this position speaks with — a different controller each.
    fn speaker(self) -> StationSpeaker {
        match self {
            Service::Tower => StationSpeaker::Tower,
            Service::Approach => StationSpeaker::Approach,
            Service::Departure => StationSpeaker::Departure,
        }
    }

    /// The position's name for the blackbox trace.
    fn name(self) -> &'static str {
        match self {
            Service::Tower => "tower",
            Service::Approach => "approach",
            Service::Departure => "departure",
        }
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
    /// The CIFP directory and this field's ICAO, for the SID an IFR
    /// clearance names (set by the owner, like the dialect). `None` = no
    /// procedure data: IFR clearances say "fly runway heading".
    procedures: Option<(String, String)>,
    /// Every SID's exit fix, positioned, loaded off the reply path by
    /// [`Station::set_procedures`] (`None` until it lands, or with no nav
    /// files): the IFR clearance picks the SID that leaves toward the
    /// destination (`nav::sid_toward`), else the first in the file.
    sid_exits: std::sync::Arc<Mutex<Option<Vec<nav::SidExit>>>>,
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
    /// The ATIS information letter the pilot last reported having ("with
    /// information Alpha"). When it matches the current broadcast, clearances
    /// skip the QNH — the whole point of an ATIS.
    pilot_info: Option<char>,
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

/// Per transmission: what the call being answered stated, consumed by the
/// pick that answers it.
#[derive(Default)]
struct CallScratch {
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
            pilot_info: None,
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

/// Radio phraseology dialect. ICAO/CAP 413 style ("descend to altitude two
/// thousand five hundred feet, QNH one zero two zero") serves most of the
/// world; FAA style ("descend and maintain two thousand five hundred") for
/// North America. Same lever as geography-accurate voice casting: a Heathrow
/// controller must not sound like Chicago Center, in accent OR in grammar.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dialect {
    Icao,
    Faa,
}

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
        Self::from_airport(apt)
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
        Some(Station {
            tower_khz: twr_row.khz,
            approach_khz,
            departure_khz,
            apt,
            desk: StripDesk::new(),
            desks: std::collections::BTreeMap::new(),
            runway_in_use: None,
            enforce_readback: true,
            dialect: Dialect::Icao,
            procedures: None,
            sid_exits: Default::default(),
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
}

mod atis_watch;
mod briefing;
mod coach;
mod compliance;
mod ctx;
mod ifr;
mod new_flight;
mod party_chatter;
mod radar;
mod region;
mod respond;
mod routing;
mod runway_pick;
mod runway_book;
mod runways;
mod self_taxi;
mod state;
mod strip;
mod takeoff_hold;
mod taxi;
mod tcas;
mod traffic;
mod wake;

pub use atis_watch::*;
pub use briefing::*;
pub use coach::*;
pub use compliance::*;
pub(crate) use ifr::*;
pub use new_flight::*;
pub use party_chatter::*;
pub use radar::*;
pub use region::*;
pub use routing::*;
pub use runway_book::*;
pub use runways::*;
pub use self_taxi::*;
pub use state::*;
pub use strip::*;
pub use takeoff_hold::*;
pub use taxi::*;
pub use tcas::*;
pub use traffic::*;
pub use wake::*;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod book_tests;

#[cfg(test)]
mod strip_tests;
