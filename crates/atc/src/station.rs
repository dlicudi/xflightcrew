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
    IfrFlight, ARRIVAL_PIN, ASSIGNED_SQUAWK, CLEARANCE, DEPARTURE_HANDOFF, GO_AROUND_ORDERS,
    IFR_CLEARANCE, IFR_FLIGHT, LANDING_HANDOFF, LANDING_HOLD, LINE_UP_CONDITION, PENDING_GO_AROUND,
    PENDING_LANDING_RELEASE, PENDING_LINE_UP, PENDING_STATION, PENDING_TAKEOFF_CLEARANCE,
    REGION_HANDOFF, RESEQUENCED_ORDER, SQUAWK_NUDGE, TAKEOFF_HOLD, TAXI_ROUTE, TAXI_TO_STAND,
    VECTORS, ZONE_CHALLENGE,
};

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
    twr: tower::Tower,
    /// The runway in use, held across calls: with the wind below 5 knots the
    /// into-wind pick is a coin toss, and the 2026-07-08 flight was cleared
    /// for takeoff on two seven, then re-cleared on zero niner two minutes
    /// later in the same calm. A runway change needs actual wind.
    runway_in_use: Option<String>,
    /// The runway the arrival now calling named for itself, consumed by the
    /// pick that answers the call (`note_stated_runway`).
    stated_runway: Option<String>,
    /// The caller stating [`Station::stated_runway`] stands ON that strip,
    /// on the ground: a runway start asking for the runway it is lined up on
    /// (`note_caller_on_strip`), consumed by the same pick.
    stated_on_strip: bool,
    /// The strip a runway start was GIVEN (`note_caller_on_strip`): held for
    /// its later calls on the ground while no stream opposes it and its
    /// tailwind stays within [`ON_STRIP_TAILWIND_KTS`], where the wind pick's
    /// hysteresis would otherwise turn it round on the next call (#178).
    start_strip: Option<String>,
    /// The runway the tower last named to this flight on the ground (its
    /// zone, circuit or IFR clearance, a taxi, a line-up): a later reply on
    /// another runway states the change (`state_runway_change`, #178).
    told_runway: Option<String>,
    /// The runway a "request runway ..." call named, canonical ("8L"), for
    /// the ctx that answers it (`note_stated_runway`, `apply_runway_request`).
    runway_request: Option<String>,
    /// That request resolved to a strip end the tower can hand out ("08L"),
    /// committed as the runway in use when the tower grants it.
    requested_ident: Option<String>,
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
    /// published to [`TAXI_ROUTE`] iff the taxi clearance is actually issued.
    planned_route: Option<nav::TaxiRoutePlan>,
    /// Challenge missing/wrong readbacks of runway-critical clearances
    /// (`XCOPILOT_ATC_READBACK`). The pending state still arms when this is
    /// off — the coach teaches the readback either way; only the challenges
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
            twr: tower::Tower::new(),
            runway_in_use: None,
            stated_runway: None,
            stated_on_strip: false,
            start_strip: None,
            told_runway: None,
            runway_request: None,
            requested_ident: None,
            dep_checked_in: false,
            handed_to: None,
            circuits: false,
            pilot_info: None,
            assigned_squawk: None,
            planned_route: None,
            enforce_readback: true,
            dialect: Dialect::Icao,
            procedures: None,
            sid_exits: Default::default(),
            ifr_bearings: None,
            ifr_sid_basis: None,
            ifr_runway: None,
            resequenced: None,
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

    /// Publish this station's process-wide side state — the ATIS broadcast
    /// and the enroute switch identity the monitor's ticks read. Called by
    /// the OWNER once a station goes live (startup and each region handoff),
    /// never by `from_airport` itself, so offline sweeps and tests stay
    /// side-effect free.
    pub fn install(&self) {
        // A new station starts in reach; the next monitor tick judges it.
        OUT_OF_REACH.store(false, std::sync::atomic::Ordering::Relaxed);
        UNCONTROLLED_ARRIVAL.store(false, Ordering::SeqCst);
        // ...and is the new flight's station: its replies publish again.
        NEW_FLIGHT_PENDING.store(false, std::sync::atomic::Ordering::SeqCst);
        *ACTIVE_FIELD.lock().unwrap_or_else(|e| e.into_inner()) = self
            .apt
            .position()
            .map(|(lat, lon)| (self.apt.name.clone(), lat, lon));
        // Controller frequencies, for the monitor's traffic advisories: a
        // controller narrates traffic whenever the pilot is on frequency,
        // not only while being vectored.
        let mut freqs = vec![(self.tower_khz, StationSpeaker::Tower)];
        if let Some(k) = self.approach_khz {
            freqs.push((k, StationSpeaker::Approach));
        }
        if let Some(k) = self.departure_khz {
            freqs.push((k, StationSpeaker::Departure));
        }
        *CONTROLLER_FREQS.lock().unwrap_or_else(|e| e.into_inner()) = freqs;
        // A new station means the old radar service (and its code) is void,
        // and the new tower has never talked to you — or to its ghosts.
        *SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *SQUAWK_NUDGE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ASSIGNED_SQUAWK.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *IFR_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        // Zone-watch grace on a fresh station: a region handoff says
        // "contact <name> Tower" and the pilot deserves the cooldown window
        // to actually call — being challenged as an intruder two seconds
        // after the handoff (2026-07-10 tape) reads as a tower at war with
        // its own radar room.
        *COMPLIANCE.lock().unwrap_or_else(|e| e.into_inner()) = ComplianceState {
            zone_last: Some(Instant::now()),
            ..ComplianceState::NEW
        };
        *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *PENDING_TAKEOFF_CLEARANCE
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = false;
        *PENDING_LINE_UP.lock().unwrap_or_else(|e| e.into_inner()) = false;
        *PENDING_GO_AROUND.lock().unwrap_or_else(|e| e.into_inner()) = false;
        *PENDING_RESEQUENCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *RESEQUENCED_ORDER.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *GO_AROUND_LAST.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *LANDING_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *PENDING_LANDING_RELEASE
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = None;
        *STRIP_SEEN.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *GHOSTS.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *WAKE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *WAKE_PREV.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *DEPARTURE_STREAM.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ARRIVAL_STREAM.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        // The last field's wind is not this field's (a re-sequence fell back
        // to it), and a challenge nobody answered is not kept for this tower.
        *TOWER_WIND.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ZONE_CHALLENGE.lock().unwrap_or_else(|e| e.into_inner()) = false;
        // The previous field's worksheets die with it: a region handoff calls
        // install() WITHOUT uninstall(), and anything left armed here keeps
        // acting for a station that no longer exists. An armed vector plan
        // kept steering toward the old field's threshold; a takeoff handoff
        // ("contact <old field> Departure on ...") could fire on a later
        // climb at the new field; a stale arrival taxi-in flag exempted
        // runway occupancy from the incursion watch. Same trap family as the
        // radio-power latch: a process-wide fact must not outlive its field.
        *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = false;
        *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *LINE_UP_CONDITION.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *DEPARTURE_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ACTIVE_PANEL.lock().unwrap_or_else(|e| e.into_inner()) =
            Some((self.apt.name.clone(), self.apt.elev_ft, services(&self.apt)));
        *ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner()) = self.apt.runways.clone();
        *ATIS.lock().unwrap_or_else(|e| e.into_inner()) = self.apt.freq(nav::FREQ_ATIS).map(|f| {
            tracing::info!(
                mhz = f.khz as f64 / 1000.0,
                "ATIS broadcasting on frequency"
            );
            AtisState {
                khz: f.khz,
                airport: spoken_airport(&self.apt.name).to_string(),
                runways: self.apt.runways.clone(),
                letter: 'A',
                edition: None,
                last_spoken: None,
            }
        });
    }

    /// The inverse of [`Station::install`]: retire the world entirely. Clears
    /// every published static — the party line AND the monitor-tick side
    /// state — so the crew's coaches, panel, and watches see "no ATC" the
    /// same instant the owner drops the station. Called by the OWNER when
    /// the app leaves an ATC-capable mode; the region index and background
    /// caches (taxi net, ILS facts) survive so re-enabling is cheap.
    pub fn uninstall() {
        NEW_FLIGHT_PENDING.store(false, std::sync::atomic::Ordering::SeqCst);
        *SURFACE_WIND.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ACTIVE_FIELD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *CONTROLLER_FREQS.lock().unwrap_or_else(|e| e.into_inner()) = Vec::new();
        OUT_OF_REACH.store(false, std::sync::atomic::Ordering::Relaxed);
        UNCONTROLLED_ARRIVAL.store(false, Ordering::SeqCst);
        *SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ACTIVE_PANEL.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner()) = Vec::new();
        *ATIS.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *SQUAWK_NUDGE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *COMPLIANCE.lock().unwrap_or_else(|e| e.into_inner()) = ComplianceState::NEW;
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *GHOSTS.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *WAKE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *WAKE_PREV.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *DEPARTURE_STREAM.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ARRIVAL_STREAM.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *GO_AROUND_LAST.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *LANDING_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *PENDING_LANDING_RELEASE
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = None;
        *STRIP_SEEN.lock().unwrap_or_else(|e| e.into_inner()) = None;
        // The party line: no clearance, no route, no vectors, no pending
        // anything — a re-enable starts from a cold contact, like a fresh
        // boot.
        *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = false;
        *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *LINE_UP_CONDITION.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *DEPARTURE_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
        // The doc on the party line promises this one dies here.
        *ASSIGNED_SQUAWK.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *IFR_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *IFR_FLIGHT.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner()) = None;
        // NOT cleared by install(): the new station is installed on the main
        // loop's next pass, before the crew has consumed the handoff.
        *REGION_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *PENDING_TAKEOFF_CLEARANCE
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = false;
        *PENDING_LINE_UP.lock().unwrap_or_else(|e| e.into_inner()) = false;
        *PENDING_GO_AROUND.lock().unwrap_or_else(|e| e.into_inner()) = false;
        *PENDING_RESEQUENCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *RESEQUENCED_ORDER.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ZONE_CHALLENGE.lock().unwrap_or_else(|e| e.into_inner()) = false;
        *TOWER_WIND.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    /// Content routing (shared PTT): if the utterance is a radio call AND COM1
    /// is on one of this station's frequencies, answer it; `None` falls
    /// through to the copilot untouched. Words are checked before any dataref
    /// read, so the common case (a cockpit command) costs nothing.
    ///
    /// Unlike a keyed transmission, an unkeyed utterance must SOUND like a
    /// radio call — addressed to a station, or classic call vocabulary — not
    /// merely parse to an intent. "Anything else we need to do before
    /// takeoff?" (a question for the crew) earned a takeoff clearance on the
    /// 2026-07-08 flight because "takeoff" alone parsed as a request; the
    /// panel's "land the aircraft" button was hijacked into a join clearance
    /// the same way.
    pub async fn try_handle(
        &mut self,
        transcript: &str,
        xp: SimView<'_>,
    ) -> Option<(String, StationSpeaker)> {
        // Cheap word gates before any dataref read — the common case is a
        // cockpit command, and it must cost nothing. route_unkeyed re-applies
        // the same gates; only the trace differs per exit. A call that names
        // our own airline callsign is addressed ("Speedbird three zero six,
        // ready to taxi").
        if !is_radio_call(transcript, &spoken_airport(&self.apt.name).to_lowercase())
            && !crate::callsign::addressed_to_own(transcript)
        {
            return None;
        }
        let intent = tower::parse_intent(transcript);
        if intent == tower::Intent::Unknown {
            // Radio-shaped words that didn't parse — exactly the utterances
            // corpus mining wants to see after a flight.
            sim::blackbox::record(
                "atc",
                serde_json::json!({
                    "route": "unkeyed", "outcome": "fallthrough",
                    "transcript": transcript,
                }),
            );
            return None;
        }
        let Some(service) = self.tuned_service(xp).await else {
            sim::blackbox::record(
                "atc",
                serde_json::json!({
                    "route": "unkeyed", "outcome": "off_frequency",
                    "transcript": transcript, "intent": format!("{intent:?}"),
                }),
            );
            return None;
        };
        self.note_stated_runway(transcript, intent.clone());
        let (ctx, wx) = self.build_ctx(xp, intent.clone()).await;
        let before = self.twr.state;
        let reply = self.route_unkeyed(transcript, service, &ctx, wx)?;
        sim::blackbox::record(
            "atc",
            serde_json::json!({
                "route": "unkeyed", "outcome": "reply", "service": service.name(),
                "transcript": transcript, "intent": format!("{intent:?}"),
                "state": format!("{before:?}>{:?}", self.twr.state), "reply": reply,
            }),
        );
        Some((reply, service.speaker()))
    }

    /// The pure half of [`Station::try_handle`]: the unkeyed gates plus the
    /// tuned service's answer, no simulator anywhere. `None` falls through to
    /// the copilot.
    fn route_unkeyed(
        &mut self,
        transcript: &str,
        service: Service,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> Option<String> {
        self.sync_landing_release();
        if !is_radio_call(transcript, &spoken_airport(&self.apt.name).to_lowercase())
            && !crate::callsign::addressed(transcript, &ctx.callsign)
        {
            return None;
        }
        self.note_information(transcript);
        let intent = tower::parse_intent(transcript);
        if intent == tower::Intent::Unknown {
            return None;
        }
        // Unkeyed "change frequency to the tower" is the pilot telling the
        // COPILOT to retune (the tune_request path) — only a KEYED
        // transmission makes it a request to the controller.
        if intent == tower::Intent::RequestFrequencyChange {
            return None;
        }
        Some(self.respond(transcript, service, intent, ctx, wx))
    }

    /// Remember "with information X" from any call — the next clearance skips
    /// the QNH when the letter matches the current broadcast.
    fn note_information(&mut self, transcript: &str) {
        if let Some(l) = heard_information(&transcript.to_lowercase()) {
            tracing::info!(letter = %l, "pilot reports having the information");
            self.pilot_info = Some(l);
        }
    }

    /// Physical routing (the radio PTT was keyed): the utterance was
    /// TRANSMITTED, so it goes to the tuned station or nowhere — never to the
    /// copilot. On frequency, even an unparsed call gets the controller's "say
    /// again" (a real controller answers garble; your crew doesn't). `None`
    /// means nobody was listening on that frequency.
    pub async fn handle_transmission(
        &mut self,
        transcript: &str,
        xp: SimView<'_>,
    ) -> Option<(String, StationSpeaker)> {
        let Some(service) = self.tuned_service(xp).await else {
            sim::blackbox::record(
                "atc",
                serde_json::json!({
                    "route": "keyed", "outcome": void_outcome(transcript, Instant::now()),
                    "transcript": transcript,
                }),
            );
            return None;
        };
        self.note_stated_runway(transcript, tower::parse_intent(transcript));
        let (ctx, wx) = self.build_ctx(xp, tower::parse_intent(transcript)).await;
        let before = self.twr.state;
        let reply = self.route_keyed(transcript, service, &ctx, wx);
        sim::blackbox::record(
            "atc",
            serde_json::json!({
                "route": "keyed",
                "outcome": if reply.is_empty() { "silence" } else { "reply" },
                // A readback has no intent of its own ("Unknown"): its
                // silence is the controller accepting it (#176).
                "readback": reply.is_empty() && tower::is_readback(transcript),
                "service": service.name(), "transcript": transcript,
                "intent": format!("{:?}", tower::parse_intent(transcript)),
                "state": format!("{before:?}>{:?}", self.twr.state), "reply": reply,
            }),
        );
        Some((reply, service.speaker()))
    }

    /// The pure half of [`Station::handle_transmission`]: everything after
    /// "somebody is listening on this frequency". Empty string = a readback
    /// the controller accepted (the frequency stays quiet).
    fn route_keyed(
        &mut self,
        transcript: &str,
        service: Service,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> String {
        self.sync_landing_release();
        self.note_information(transcript);
        let intent = tower::parse_intent(transcript);
        // The readback gate first: a runway-critical clearance is waiting
        // for its echo. Correct → silence; wrong runway → correction;
        // anything else → the challenge (which repeats the instruction).
        // Runs before the generic readback-silence path so a WRONG runway is
        // caught, not waved through as "a readback".
        if service == Service::Tower {
            // Judged BEFORE the gate spends it: with enforcement off, an IFR
            // readback that said any of the clearance stands as read.
            let ifr_read = self.twr.pending().is_some_and(|p| {
                matches!(p.awaited, tower::Awaited::IfrClearance { .. })
                    && tower::check_readback(transcript, &p.awaited)
                        != tower::ReadbackCheck::NotAReadback
            });
            if let Some(reply) = self.twr.gate(transcript, intent.clone(), ctx) {
                if reply.is_empty() {
                    // Correct readback: the debt is paid; tell the coach.
                    if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
                        c.pending_readback = None;
                    }
                    if let Some(ifr) = self.twr.take_accepted_ifr() {
                        publish_ifr_clearance(ifr, &ctx.callsign, self.ifr_bearings, self.ifr_sid_basis.as_ref());
                    }
                    return String::new();
                }
                if self.enforce_readback {
                    return reply;
                }
                if ifr_read {
                    self.twr.accept_ifr_unenforced();
                    if let Some(ifr) = self.twr.take_accepted_ifr() {
                        publish_ifr_clearance(ifr, &ctx.callsign, self.ifr_bearings, self.ifr_sid_basis.as_ref());
                    }
                }
                // Enforcement off (XCOPILOT_ATC_READBACK=false): no
                // challenge — route the call normally; the coach still
                // shows the owed line.
            }
        }
        // A pilot readback earns SILENCE, like a real frequency: the
        // controller only speaks up to correct a wrong one.
        // Exception: the first call to DEPARTURE is a check-in ("passing one
        // thousand five hundred climbing"), and a check-in gets radar contact.
        //
        // "Cleared ..." can only be an echo of a clearance — a pilot ASKING
        // says "request clearance", never "cleared". Without this, reading
        // back "cleared to land runway two seven" re-parsed as a landing
        // REQUEST: the tower answered "join final" again and the sequence
        // walked backwards from ClearedToLand to Inbound.
        let echoes_clearance = transcript
            .to_lowercase()
            .split(|c: char| !c.is_alphanumeric())
            .any(|w| w == "cleared");
        if echoes_clearance || (intent == tower::Intent::Unknown && tower::is_readback(transcript))
        {
            // The position the tower handed us to takes the check-in: a
            // field without a departure frequency hands a departure to
            // APPROACH, and until EGBB (2026-09-27) approach heard "passing
            // three thousand one hundred climbing one five thousand" as a
            // readback and said nothing, the tower still holding the flight
            // cleared for takeoff.
            let checks_in = service == Service::Departure || self.handed_to == Some(service);
            if checks_in && !self.dep_checked_in {
                self.dep_checked_in = true;
                self.transfer_departure();
                // This reply IS the radar contact: the watch has nothing
                // left to say.
                radar_contact_at_check_in();
                let position = match service {
                    Service::Approach => "Approach",
                    _ => "Departure",
                };
                // The climb the pilot announced is the climb they get: the
                // zone clearance capped them at two thousand, and "radar
                // contact" alone left "climbing five thousand" unanswered
                // (LXGB, 2026-09-15).
                let cap = self.filed_cruise_ft();
                return match climb_announced(transcript)
                    .map(|alt| capped_departure_climb(&alt, cap, ctx.transition_level_ft))
                {
                    Some(alt) => format!(
                        "{}, {} {position}, radar contact, climb {alt}.",
                        ctx.callsign, ctx.airport
                    ),
                    None => format!(
                        "{}, {} {position}, radar contact.",
                        ctx.callsign, ctx.airport
                    ),
                };
            }
            // The departure clearance's suggested readback is paid.
            self.twr.clear_clearance_readback();
            if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
                c.pending_readback = None;
            }
            return String::new();
        }
        self.respond(transcript, service, intent, ctx, wx)
    }

    /// The voice of whichever of this station's positions COM1 is tuned to.
    pub async fn tuned_speaker(&self, xp: SimView<'_>) -> Option<StationSpeaker> {
        self.tuned_service(xp).await.map(Service::speaker)
    }

    /// Does this frequency (kHz) belong to one of the station's services?
    /// Same 5 kHz tolerance as [`Station::tuned_service`], no dataref read —
    /// callers with a live COM1 value (the STT prompt switch) check for free.
    pub fn covers_khz(&self, khz: i64) -> bool {
        (khz - self.tower_khz).abs() <= 5
            || self.approach_khz.is_some_and(|k| (khz - k).abs() <= 5)
            || self.departure_khz.is_some_and(|k| (khz - k).abs() <= 5)
    }

    /// Which service COM1 is tuned to. 8.33 channel names can sit 5 kHz off
    /// the carrier; adjacent channels are 8 kHz apart, so 5 can't cross.
    async fn tuned_service(&self, xp: SimView<'_>) -> Option<Service> {
        // A field beyond reach hears nothing, whatever is tuned: Gibraltar
        // cleared a 172 parked at LILB, 1,000 nm away, and handed it a taxi
        // route to its own runway 09 (2026-09-25).
        if station_unavailable() {
            return None;
        }
        let com1 = xp.get_float(datarefs::COM1_FREQ_KHZ).await.ok()?.round() as i64;
        if (com1 - self.tower_khz).abs() <= 5 {
            return Some(Service::Tower);
        }
        if self.approach_khz.is_some_and(|k| (com1 - k).abs() <= 5) {
            return Some(Service::Approach);
        }
        if self.departure_khz.is_some_and(|k| (com1 - k).abs() <= 5) {
            return Some(Service::Departure);
        }
        None
    }

    /// Answer one transmission on whichever service is tuned. Pure: the
    /// context and wind were read before the words reached the controller.
    fn respond(
        &mut self,
        transcript: &str,
        service: Service,
        intent: tower::Intent,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> String {
        self.sync_landing_release();
        let was = self.twr.state;
        if service == Service::Tower {
            self.note_departure_intent(&intent, transcript);
        }
        // A zone request from circuit traffic (noted just above, from THIS
        // call's words) is answered with the circuit clearance: a circuit
        // never leaves the zone (LEMG 13, sorties 1790590663 and 1790591256,
        // 2026-09-28: "for circuits" got "cleared to leave the Malaga zone").
        let circuit_ctx;
        let ctx = if service == Service::Tower
            && intent == tower::Intent::RequestClearance
            && self.circuits
        {
            circuit_ctx = tower::Ctx {
                circuit: Some(self.circuit_offer(&ctx.runway)),
                ..ctx.clone()
            };
            &circuit_ctx
        } else {
            ctx
        };
        // A departure clearance in a RADAR environment carries a discrete
        // code — the zone clearance is where a real controller assigns it
        // (Heathrow flew a whole day on 7000, 2026-07-10). Tower-only strips
        // keep the VFR conspicuity. The watch's "radar contact" belongs to
        // whoever has the scope.
        let assigned_ctx;
        let ctx = if service == Service::Tower
            && intent == tower::Intent::RequestClearance
            && self.assigned_squawk.is_none()
            && (self.approach_khz.is_some() || self.departure_khz.is_some())
        {
            let scope = if self.departure_khz.is_some() {
                StationSpeaker::Departure
            } else {
                StationSpeaker::Approach
            };
            let spoken = self.assign_squawk(&ctx.callsign, scope);
            assigned_ctx = tower::Ctx {
                squawk: spoken,
                ..ctx.clone()
            };
            &assigned_ctx
        } else {
            ctx
        };
        // An IFR request to a destination the index knows: the station
        // makes up the clearance (SID, altitude, code) and the tower
        // phrases it. A destination it cannot find leaves `ifr` empty and
        // the tower asks again.
        let ifr_ctx;
        let ctx = match (&intent, service) {
            (tower::Intent::RequestIfrClearance { dest: Some(d) }, Service::Tower) => {
                match self.ifr_offer(d, ctx) {
                    Some(offer) => {
                        ifr_ctx = tower::Ctx {
                            squawk: atis::spoken_digits(&format!("{:04}", offer.clearance.squawk)),
                            ifr: Some(offer),
                            ..ctx.clone()
                        };
                        &ifr_ctx
                    }
                    None => ctx,
                }
            }
            _ => ctx,
        };
        let reply = self.respond_inner(transcript, service, intent.clone(), ctx, wx);
        let reply = if service == Service::Tower {
            self.state_runway_change(&intent, ctx, reply)
        } else {
            reply
        };
        // A granted runway request makes that runway the one in use; the
        // taxi (or the join) it bought freezes the pick on it from here.
        if service == Service::Tower && intent == tower::Intent::RequestRunway {
            if let Some(ident) = self.requested_ident.take() {
                if ctx.runway_refusal.is_none() {
                    tracing::info!(runway = %ident, "runway request granted");
                    self.runway_in_use = Some(ident);
                }
            }
        }
        // Publish the clearance picture for the monitor's compliance watch:
        // two-way contact has happened, and this is what the pilot is (and
        // isn't) cleared to do. Same reply-path-owns-its-static pattern as
        // HANDOFF and SQUAWK_WATCH.
        // ...unless a NEW flight has been set up and this station is the
        // old one, still answering until the main loop installs the new
        // (`retire_for_new_flight`): its worksheet is the ended flight's.
        if !new_flight_pending() {
            publish_clearance(ClearanceSnap {
                state: self.twr.state,
                callsign: ctx.callsign.clone(),
                runway_ident: self.runway_in_use.clone().unwrap_or_default(),
                pending_readback: self.twr.suggested_readback(&ctx.callsign),
                holds_runway: self.twr.holds_runway(),
            });
        }
        // A withheld landing clearance lives only while the flight is still
        // Inbound for it: cleared (a new final report), gone around, landed
        // or elsewhere retires it. A final report onto an occupied runway
        // ("continue approach", tower.rs) arms it, for the monitor's watch
        // to release once the strip clears (`landing_hold_tick`).
        if service == Service::Tower {
            if self.twr.state != tower::State::Inbound || intent == tower::Intent::GoAround {
                *LANDING_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
            }
            if intent == tower::Intent::ReportFinal
                && self.twr.state == tower::State::Inbound
                && !ctx.on_ground
                && ctx.runway_occupied
            {
                let ident = self.runway_in_use.clone().unwrap_or_default();
                sim::blackbox::record(
                    "atc",
                    serde_json::json!({"event": "landing_withheld", "runway": ident,
                        "why": "runway_occupied"}),
                );
                *LANDING_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = Some(LandingHold {
                    callsign: ctx.callsign.clone(),
                    runway: ctx.runway.clone(),
                    runway_ident: ident,
                    line: self.twr.landing_clearance_line(ctx),
                });
            }
        }
        // A fresh taxi clearance publishes its route for the off-route
        // compliance watch (and clears any stale one when route-less).
        // ...and so does a ready call answered "taxi to holding point ...
        // first": it is a taxi clearance with a route like any other (#178).
        let taxi_first =
            intent == tower::Intent::RequestTakeoff && was != tower::State::TaxiCleared;
        if service == Service::Tower
            && (matches!(intent, tower::Intent::RequestTaxi | tower::Intent::RequestRunway)
                || taxi_first)
            && self.twr.state == tower::State::TaxiCleared
        {
            *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = self
                .planned_route
                .as_ref()
                .filter(|r| r.path.len() >= 2)
                .map(|r| TaxiWatch {
                    path: r.path.clone(),
                    via: spoken_via(&r.via).join(", "),
                    seg: r.seg.clone(),
                    holding_point: r.holding_point.clone(),
                    lead_on: r.lead_on.clone(),
                });
            *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = false;
            *SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
        // The arrival taxi-in publishes its stand route the same way, so the
        // crew's auto_taxi drives it to parking. `was` is the pre-transition
        // state (a "request taxi" right after landing is an arrival too).
        if service == Service::Tower
            && (intent == tower::Intent::RequestTaxiIn
                || (intent == tower::Intent::RequestTaxi && was == tower::State::ClearedToLand))
        {
            *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = self
                .planned_route
                .as_ref()
                .filter(|r| r.path.len() >= 2)
                .map(|r| TaxiWatch {
                    path: r.path.clone(),
                    via: spoken_via(&r.via).join(", "),
                    seg: r.seg.clone(),
                    holding_point: r.holding_point.clone(),
                    lead_on: r.lead_on.clone(),
                });
            *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = true;
        }
        // A takeoff request answered with anything SHORT of the clearance —
        // hold position (traffic, wake, occupied strip), a conditional or
        // plain line-up, a cancellation — arms the release watch, so the
        // tower comes back unprompted the moment the runway is truly free.
        // A RUNWAY START is held from Parked: its first ready call puts it
        // LinedUp and holds it in one reply, and the watch never armed (LEMG
        // 13, sortie 1790623420-full_auto, 2026-09-28, issue #172: "hold
        // position, traffic on the runway" at +50 with nothing to release it).
        let runway_start_held =
            was == tower::State::Parked && self.twr.state == tower::State::LinedUp;
        if service == Service::Tower
            && intent == tower::Intent::RequestTakeoff
            && (matches!(
                was,
                tower::State::TaxiCleared | tower::State::LinedUp | tower::State::TakeoffCleared
            ) || runway_start_held)
            && self.twr.state != tower::State::TakeoffCleared
            && (ctx.final_traffic_nm.is_some()
                || ctx.runway_occupied
                || ctx.wake_secs_left.is_some())
        {
            let mut held = TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner());
            // Asking again does not restart the separation you have already
            // waited out. The wake gate is capped at one full interval AT THE
            // HOLD (`takeoff_hold_tick`), measured from `armed_at` — and
            // re-stamping it on every request handed that cap back: at a field
            // whose departure interval is shorter than the separation, a pilot
            // who asks a second time resets their own clock, hears "wake
            // turbulence, departing traffic ahead of you" again, and can never
            // take off. The clock belongs to the departure ahead of us in
            // sequence, not to our last transmission — so a hold already
            // standing on this runway keeps the moment it was armed.
            let armed_at = hold_armed_at(
                held.as_ref(),
                &self.runway_in_use.clone().unwrap_or_default(),
                Instant::now(),
            );
            // So is the traffic it was held for: asking again names nobody new.
            let held_for = held
                .as_ref()
                .filter(|h| h.runway_ident == self.runway_in_use.clone().unwrap_or_default())
                .and_then(|h| h.held_for.clone());
            *held = Some(TakeoffHold {
                callsign: ctx.callsign.clone(),
                runway: ctx.runway.clone(),
                runway_ident: self.runway_in_use.clone().unwrap_or_default(),
                clear_since: None,
                armed_at,
                logged_at: Instant::now(),
                // A hold issued FROM the runway is already sequenced: the
                // conditional/plain line-up put us there, so the watch owes a
                // release, not another line-up.
                lined_up: self.twr.state == tower::State::LinedUp,
                behind: self.twr.pending().is_some_and(|p| {
                    matches!(p.awaited, tower::Awaited::LineUp { behind: true, .. })
                }),
                held_for,
                cut_in_logged: Default::default(),
            });
            // Name the runway: at a parallel-runway field "held for traffic"
            // is only diagnosable if you can see WHICH strip the watch is
            // looking at (2026-07-27).
            tracing::info!(
                runway = %self.runway_in_use.clone().unwrap_or_default(),
                final_nm = ?ctx.final_traffic_nm,
                occupied = ctx.runway_occupied,
                wake_secs = ?ctx.wake_secs_left,
                "takeoff clearance withheld; release watch armed"
            );
        }
        // A departure that STOPPED on the runway spent everything its
        // clearance armed: the held release (EGKK 2026-09-27: armed while
        // the 737 rolled, never released, it replayed "traffic clear,
        // cleared for takeoff" 1.2 s into the resume), a release the main
        // loop has not synced yet, the conditional line-up's condition and
        // the after-departure handoff.
        if service == Service::Tower
            && intent == tower::Intent::RejectingTakeoff
            && ctx.on_ground
            && matches!(was, tower::State::TakeoffCleared | tower::State::LinedUp)
        {
            *PENDING_TAKEOFF_CLEARANCE
                .lock()
                .unwrap_or_else(|e| e.into_inner()) = false;
            *PENDING_LINE_UP.lock().unwrap_or_else(|e| e.into_inner()) = false;
            *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
            tracing::info!(runway = ?self.runway_in_use, "takeoff rejected; clearance cancelled");
        }
        // A held release lives only while the tower still owes one: the
        // flight holding short or on the runway WITHOUT a clearance. Any
        // other state (cleared, parked, inbound) or a reject retires it, so
        // it can never replay a clearance nobody is waiting for.
        if service == Service::Tower
            && (intent == tower::Intent::RejectingTakeoff
                || !matches!(
                    self.twr.state,
                    tower::State::TaxiCleared | tower::State::LinedUp
                ))
        {
            *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *LINE_UP_CONDITION.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
        reply
    }

    /// Sync the tower FSM with the held takeoff clearance the MONITOR spoke
    /// once the final cleared (see [`takeoff_hold_tick`]): the state moves to
    /// TakeoffCleared and the departure handoff arms, exactly as if the
    /// clearance had been issued on the reply path.
    pub async fn apply_monitor_takeoff_clearance(&mut self, xp: SimView<'_>) {
        // A departure context (the monitor's release watch) — never an arrival.
        let (ctx, _wx) = self.build_ctx(xp, tower::Intent::RequestTakeoff).await;
        self.twr.state = tower::State::TakeoffCleared;
        // The released clearance owes its readback exactly like one issued
        // on the reply path.
        self.twr.arm_takeoff_readback(
            ctx.runway.clone(),
            format!("runway {}, cleared for takeoff.", ctx.runway),
        );
        if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
            c.pending_readback = self.twr.suggested_readback(&ctx.callsign);
        }
        self.arm_departure_handoff(&ctx);
    }

    /// Sync the tower FSM with the line-up-and-wait the MONITOR spoke when it
    /// sequenced a held departure onto the runway (see [`takeoff_hold_tick`]):
    /// the state moves to LinedUp and the line-up readback is owed, exactly as
    /// if the instruction had been issued on the reply path. The hold stays
    /// armed — the release is still to come.
    pub async fn apply_monitor_line_up(&mut self, xp: SimView<'_>) {
        let (ctx, _wx) = self.build_ctx(xp, tower::Intent::RequestTakeoff).await;
        self.twr.state = tower::State::LinedUp;
        self.twr.arm_line_up_readback(
            ctx.runway.clone(),
            format!("line up and wait runway {}.", ctx.runway),
        );
        if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
            c.pending_readback = self.twr.suggested_readback(&ctx.callsign);
        }
    }

    /// Sync the tower FSM with the go-around order the MONITOR spoke (see
    /// [`go_around_tick`]): the landing clearance is void and the flight is
    /// back in the pattern, exactly as if the pilot had called it. The
    /// coach's next line becomes the acknowledgment the order deserves,
    /// never the readback of the clearance that just died.
    /// Sync the tower FSM with a withheld landing clearance the MONITOR
    /// released (see [`landing_hold_tick`]): ClearedToLand with its readback
    /// owed, exactly as if it had been the reply to the final report. Run
    /// before the station answers anything, so the crew's readback of the
    /// released clearance meets a tower that knows it gave it.
    fn sync_landing_release(&mut self) {
        let released = PENDING_LANDING_RELEASE
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        if let Some((runway, line)) = released {
            self.twr.release_landing(runway, line);
        }
    }

    pub fn apply_monitor_go_around(&mut self) {
        self.twr.apply_go_around();
        // The re-sequence the order announced ("expect runway one two"):
        // the runway in use moves with it, and the published clearance
        // names it, so the crew's approach follows once its missed approach
        // is flown (`autoland::clearance_switch`). Issue #162.
        let reseq = PENDING_RESEQUENCE
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        if let Some((from, to)) = &reseq {
            tracing::info!(%from, %to, "arrival re-sequenced after a go-around");
            self.runway_in_use = Some(to.clone());
            self.stated_runway = None;
        }
        self.resequenced = reseq.clone();
        if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
            c.state = tower::State::Inbound;
            c.pending_readback = Some(format!("Going around, {}.", c.callsign));
            if let Some((_, to)) = &reseq {
                c.runway_ident = to.clone();
            }
        }
    }

    fn respond_inner(
        &mut self,
        transcript: &str,
        service: Service,
        intent: tower::Intent,
        ctx: &tower::Ctx,
        wx: Wx,
    ) -> String {
        // "Cancel IFR" is the IFR book's, whichever position hears it.
        if intent == tower::Intent::CancelIfr {
            return self.cancel_ifr(ctx);
        }
        // A SID amendment is the IFR book's too, and the tower's to give.
        if intent == tower::Intent::RequestSid && service == Service::Tower {
            return self.amend_sid(transcript, ctx);
        }
        match service {
            Service::Tower => {
                let was = self.twr.state;
                let going_around = intent == tower::Intent::GoAround;
                let reply = self.twr.handle(intent.clone(), ctx);
                // The crew's "going around" after a re-sequenced order is
                // answered with the runway it is re-sequenced to, once.
                let reply = match (going_around, self.resequenced.take()) {
                    (true, Some((from, to))) => tower::go_around_resequenced_line(
                        &ctx.callsign,
                        &atis::spoken_runway(&from),
                        &atis::spoken_runway(&to),
                    ),
                    _ => reply,
                };
                // A final report naming the wrong runway earns its correction
                // in the same breath as the clearance.
                let reply = tower::Tower::correct_final_runway(transcript, intent, ctx, reply);
                // A fresh takeoff clearance arms the after-departure handoff:
                // once airborne and climbing, the tower sends the flight to
                // departure (or approach) — the frequency chain, not a tower
                // that owns you forever.
                if self.twr.state == tower::State::TakeoffCleared
                    && was != tower::State::TakeoffCleared
                {
                    self.arm_departure_handoff(ctx);
                }
                reply
            }
            Service::Approach => self.respond_approach(intent, ctx, wx),
            Service::Departure => self.respond_departure(intent, ctx, wx),
        }
    }

    /// The departure controller: takes the handoff after takeoff, then sends
    /// arrivals to approach (vectors) and everything runway-shaped back to the
    /// tower.
    fn respond_departure(&mut self, intent: tower::Intent, ctx: &tower::Ctx, wx: Wx) -> String {
        match intent {
            tower::Intent::RadioCheck => {
                format!(
                    "{}, {} Departure, reading you five.",
                    ctx.callsign, ctx.airport
                )
            }
            tower::Intent::WindCheck => {
                format!(
                    "{}, wind {}{}.",
                    ctx.callsign,
                    ctx.wind,
                    tower::qnh_clause(ctx)
                )
            }
            tower::Intent::SquawkCheck => {
                format!("{}, squawk seven zero zero zero.", ctx.callsign)
            }
            // Coming back in: vectors are approach's job when the field has
            // one; otherwise departure does what approach would.
            tower::Intent::RequestVectors => match &ctx.approach {
                Some(freq) => format!(
                    "{}, contact {} Approach on {} for vectors.",
                    ctx.callsign, ctx.airport, freq
                ),
                None => self.arm_vectors(ctx, wx),
            },
            tower::Intent::RequestLanding { touch_and_go } => match &ctx.approach {
                Some(freq) => {
                    self.twr.expect_touch_and_go(touch_and_go);
                    format!(
                        "{}, contact {} Approach on {} for vectors.",
                        ctx.callsign, ctx.airport, freq
                    )
                }
                None => {
                    self.twr.expect_touch_and_go(touch_and_go);
                    self.arm_vectors(ctx, wx)
                }
            },
            // A mayday on the departure frequency: departure keeps the flight
            // (an emergency is never told to go retune) and vectors it home.
            // On the ground there is nothing to vector — send the trucks.
            tower::Intent::Mayday if ctx.on_ground => format!(
                "{}, roger mayday. Emergency services on their way to you — hold position if able.",
                ctx.callsign
            ),
            tower::Intent::Mayday => {
                self.twr.expect_touch_and_go(false);
                let vectors = self.arm_vectors(ctx, wx);
                format!("Roger mayday. {vectors}")
            }
            // Runway-shaped business belongs to the tower.
            tower::Intent::ReportFinal
            | tower::Intent::ReportDownwind
            | tower::Intent::ReportBase
            | tower::Intent::ReportCrosswind
            | tower::Intent::GoAround
            | tower::Intent::RejectingTakeoff
            | tower::Intent::RequestTakeoff
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestTaxiIn
            | tower::Intent::RunwayVacated
            | tower::Intent::RequestRunway
            | tower::Intent::RequestBacktrack
            | tower::Intent::CancelIfr
            | tower::Intent::RequestSid
            | tower::Intent::RequestStartup
            | tower::Intent::RequestShutdown
            | tower::Intent::RequestIfrClearance { .. }
            | tower::Intent::RequestTransit
            | tower::Intent::RequestClearance => format!(
                "{}, contact {} Tower on {}.",
                ctx.callsign,
                ctx.airport,
                atis::spoken_mhz(self.tower_khz)
            ),
            tower::Intent::Greeting => {
                format!(
                    "{}, {} Departure, pass your message.",
                    ctx.callsign, ctx.airport
                )
            }
            tower::Intent::RequestFrequencyChange => {
                format!("{}, frequency change approved.", ctx.callsign)
            }
            // Departure has the scope: assign the code; "radar contact" comes
            // from the watch tick once the code actually squawks.
            tower::Intent::RequestFlightFollowing => {
                let squawk = self.assign_squawk(&ctx.callsign, StationSpeaker::Departure);
                format!(
                    "{}, squawk {}. Maintain VFR, flight following.",
                    ctx.callsign, squawk
                )
            }
            tower::Intent::Unknown => {
                format!("{}, {} Departure, say again.", ctx.callsign, ctx.airport)
            }
        }
    }

    /// "Cancel IFR" (issue #96): the flight's IFR standing ends. The
    /// clearance (owed, accepted or published) and the flight's IFR code
    /// are retired, and with them the runway frozen for the SID, so the next
    /// region handoff is a VFR one (the service terminated, seven thousand)
    /// and a later request is a fresh clearance. A flight with nothing to
    /// cancel is told so rather than acknowledged.
    fn cancel_ifr(&mut self, ctx: &tower::Ctx) -> String {
        let owed = self.twr.cancel_ifr();
        let published = IFR_CLEARANCE
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take()
            .is_some();
        let flight = IFR_FLIGHT
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take()
            .is_some();
        if !(owed || published || flight) {
            tracing::info!("cancel IFR with no IFR flight");
            return format!("{}, roger, you are not on an IFR clearance.", ctx.callsign);
        }
        self.ifr_runway = None;
        self.ifr_bearings = None;
        self.ifr_sid_basis = None;
        tracing::info!(owed, published, flight, "IFR cancelled");
        match self.dialect {
            Dialect::Faa => format!("{}, IFR cancellation received.", ctx.callsign),
            Dialect::Icao => format!("{}, IFR flight cancelled.", ctx.callsign),
        }
    }

    /// "Request the Lam five Papa departure" (issue #152): an IFR flight
    /// asking to fly another SID off the runway it was cleared on. Granted
    /// when the words name a SID this field publishes for that runway: the
    /// held clearance is amended (the crew sees it in
    /// `party_line::assigned_clearance`) and blackboxed `sid_amended`;
    /// otherwise the cleared departure stands, said, and blackboxed
    /// `sid_request_refused`. A flight with no IFR clearance is told to ask
    /// for one.
    fn amend_sid(&mut self, transcript: &str, ctx: &tower::Ctx) -> String {
        let cs = &ctx.callsign;
        let held = IFR_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).clone();
        let Some(mut c) = held else {
            return format!("{cs}, request your IFR clearance first.");
        };
        let refused = |why: &str, asked: Option<&str>| {
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "sid_request_refused", "asked": asked,
                    "cleared": c.sid, "runway": c.runway, "why": why}),
            );
        };
        let Some(words) = tower::requested_departure(transcript) else {
            refused("unheard", None);
            return format!("{cs}, say again the departure.");
        };
        let idents: Vec<String> = self
            .procedures
            .as_ref()
            .map(|(dir, apt)| nav::sid_transitions(dir, apt, &c.runway))
            .unwrap_or_default()
            .into_iter()
            .map(|(i, _)| i)
            .collect();
        let rwy = atis::spoken_runway(&c.runway);
        let stands = match &c.sid {
            Some(s) => format!("The {} departure stands.", tower::spoken_sid(s)),
            None => "Fly runway heading.".to_string(),
        };
        let Some(sid) = tower::match_sid(&words, &idents).map(str::to_string) else {
            refused("not_this_runway", Some(&words));
            return format!("{cs}, negative, no such departure from runway {rwy}. {stands}");
        };
        if c.sid.as_deref().is_some_and(|s| s.eq_ignore_ascii_case(&sid)) {
            return format!("{cs}, affirm, the {} departure.", tower::spoken_sid(&sid));
        }
        tracing::info!(from = ?c.sid, to = %sid, runway = %c.runway, "IFR clearance: SID amended");
        sim::blackbox::record(
            "atc",
            serde_json::json!({"event": "sid_amended", "side": "tower", "callsign": cs,
                "from": c.sid, "to": sid, "runway": c.runway}),
        );
        c.sid = Some(sid.clone());
        *IFR_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = Some(c);
        format!("{cs}, {} departure approved, runway {rwy}.", tower::spoken_sid(&sid))
    }

    /// The IFR clearance for `dest` (as heard), made up from what this
    /// station knows: the destination from the index, the runway in use,
    /// its SID from the CIFP (none = runway heading, never an invented
    /// name), the initial altitude from the field elevation, and the code:
    /// the discrete one at a radar field (the same one the zone clearance
    /// would assign), the code already held otherwise. A reissued clearance
    /// supersedes the published one until it is read back. `None` = the
    /// destination is unknown to the index.
    /// The cruise level this flight FILED, feet: the accepted IFR
    /// clearance's destination and this field's filed route to it. `None`
    /// without a clearance, a filed route or a filed level.
    fn filed_cruise_ft(&self) -> Option<i64> {
        let dest = IFR_CLEARANCE
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()?
            .dest
            .clone();
        let (_, apt) = self.procedures.as_ref()?;
        crate::party_line::filed_route_for(apt, &dest)?.cruise_ft
    }

    fn ifr_offer(&mut self, dest: &str, ctx: &tower::Ctx) -> Option<tower::IfrOffer> {
        let (icao, name, dest_pos) = resolve_destination(dest)?;
        let runway = self.runway_in_use.clone().unwrap_or_default();
        self.ifr_runway = Some(runway.clone()).filter(|r| !r.is_empty());
        // The flight's FILED route, when the crew has one to this
        // destination from this field: the SID is the one that serves the
        // runway and joins it, by the crew's own rule
        // (`nav::sid_for_filed_route`), so the crew can fly what it is
        // cleared. None joins: runway heading, never a SID that leads
        // elsewhere (EGKK 08R, 2026-09-28: KENE3P toward Birmingham's
        // bearing for a route filed via LAM; the crew flew LAM5P).
        let filed = self.procedures.as_ref().filter(|_| !runway.is_empty()).and_then(|(dir, apt)| {
            let route = crate::party_line::filed_route_for(apt, &icao)?;
            Some((
                nav::sid_for_filed_route(dir, apt, &runway, &route.fixes, route.sid.as_deref()),
                !nav::sid_transitions(dir, apt, &runway).is_empty(),
            ))
        });
        let (sid, basis) = match filed {
            Some((Some(choice), _)) => {
                self.ifr_bearings = None;
                (Some(choice.ident), SidBasis::Route { join: choice.join })
            }
            // The runway has SIDs and none meets the route.
            Some((None, true)) => {
                self.ifr_bearings = None;
                (None, SidBasis::NoneJoins)
            }
            _ => {
                // The SID that leaves toward the destination; with no
                // positioned exits (still loading, no nav data) the first in
                // the file.
                let toward = match (self.apt.position(), runway.is_empty()) {
                    (Some(field), false) => self
                        .sid_exits
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .as_deref()
                        .and_then(|x| nav::sid_toward(x, &runway, field, dest_pos)),
                    _ => None,
                };
                self.ifr_bearings = toward.as_ref().map(|t| (t.1, t.2));
                match toward {
                    Some((ident, _, _)) => (Some(ident), SidBasis::Bearing),
                    None => (
                        self.procedures
                            .as_ref()
                            .filter(|_| !runway.is_empty())
                            .and_then(|(dir, apt)| nav::sid_for_runway(dir, apt, &runway)),
                        SidBasis::FirstInFile,
                    ),
                }
            }
        };
        self.ifr_sid_basis = Some(basis);
        let radar = self.approach_khz.is_some() || self.departure_khz.is_some();
        let squawk = if radar {
            let scope = if self.departure_khz.is_some() {
                StationSpeaker::Departure
            } else {
                StationSpeaker::Approach
            };
            self.assign_squawk(&ctx.callsign, scope);
            self.assigned_squawk.unwrap_or(7000)
        } else {
            self.assigned_squawk.unwrap_or(7000)
        };
        *IFR_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *IFR_FLIGHT.lock().unwrap_or_else(|e| e.into_inner()) = None;
        Some(tower::IfrOffer {
            clearance: tower::IfrClearance {
                dest: icao,
                sid,
                runway,
                initial_alt_ft: tower::ifr_initial_altitude_ft(self.apt.elev_ft),
                squawk,
            },
            dest_spoken: spoken_airport(&name).to_string(),
            faa: self.dialect == Dialect::Faa,
            departure_freq: self.departure_khz.or(self.approach_khz).map(atis::spoken_mhz),
        })
    }

    /// The circuit clearance's facts (#182): the height, the dialect, the
    /// runway's published side (apt.dat `1101`, found by its spoken
    /// designator) and whether the field's data allows circuits (`1302
    /// allows_circuits`; LEMG says 0, blackbox `atc` `circuits_not_allowed`).
    fn circuit_offer(&self, spoken_runway: &str) -> tower::CircuitOffer {
        let left = self
            .apt
            .runways
            .iter()
            .flat_map(|r| [&r.ident_a, &r.ident_b])
            .find(|id| atis::spoken_runway(id) == spoken_runway)
            .and_then(|id| self.apt.pattern_left(id));
        let not_available = self.apt.allows_circuits == Some(false);
        if not_available {
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "circuits_not_allowed", "field": self.apt.name,
                                   "runway": spoken_runway}),
            );
        }
        tower::CircuitOffer {
            height_ft: tower::circuit_height_ft(self.apt.elev_ft),
            faa: self.dialect == Dialect::Faa,
            left,
            not_available,
        }
    }

    /// Issue this flight its discrete code (idempotent — the same flight
    /// keeps the same code) and arm the radar watch. Returns the spoken code
    /// for the reply. The watch side effect mirrors [`Station::arm_departure_handoff`]'s
    /// `HANDOFF` write: reply paths own their monitor-tick state.
    fn assign_squawk(&mut self, callsign: &str, speaker: StationSpeaker) -> String {
        let code = *self
            .assigned_squawk
            .get_or_insert_with(|| discrete_squawk(callsign));
        // Publish it for the crew (the transponder gets dialled to this).
        *ASSIGNED_SQUAWK.lock().unwrap_or_else(|e| e.into_inner()) = Some(code);
        // Circuit traffic gets no radar service: the code, but no watch
        // (no "radar contact" from a tower working the pattern by eye).
        if self.circuits {
            return atis::spoken_digits(&code.to_string());
        }
        let mut w = SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner());
        // Same code from a new controller: the watch keeps its history (no
        // second grace period, no second "radar contact") but the VOICE is
        // whoever issued it last, the one we are actually talking to.
        if let Some(w) = w.as_mut().filter(|w| w.code == code) {
            w.speaker = speaker;
        }
        if w.as_ref().map(|w| w.code) != Some(code) {
            *w = Some(SquawkWatch {
                code,
                callsign: callsign.to_string(),
                speaker,
                observed: false,
                check_in_owed: false,
                first_seen: None,
                last_nudge: None,
                nudges: 0,
            });
        }
        atis::spoken_digits(&code.to_string())
    }

    /// What a departure request says about where the flight is going: the
    /// pattern ("request taxi for circuits") or away. A taxi, zone or IFR
    /// request states it afresh (said or not); a "ready for departure" can
    /// only add it, since pilots do not repeat "for circuits" on every call.
    /// Declaring circuits retires any radar watch and any armed handoff: the
    /// tower keeps pattern traffic. LEMG 31 (sortie 1790584715, 2026-09-28):
    /// a zone clearance from the mission before left a Departure watch
    /// armed, and the tower's voice said "radar contact" to the circuit
    /// 0.4 s after rotation.
    fn note_departure_intent(&mut self, intent: &tower::Intent, transcript: &str) {
        let said = tower::wants_circuits(transcript);
        let now = match intent {
            tower::Intent::RequestTaxi | tower::Intent::RequestClearance => said,
            tower::Intent::RequestIfrClearance { .. } => false,
            tower::Intent::RequestTakeoff => self.circuits || said,
            _ => return,
        };
        if now == self.circuits {
            return;
        }
        self.circuits = now;
        sim::blackbox::record(
            "atc",
            serde_json::json!({"event": "circuit_traffic", "circuits": now,
                "intent": format!("{intent:?}")}),
        );
        tracing::info!(circuits = now, "departure intent noted");
        if now {
            *SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *SQUAWK_NUDGE.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *DEPARTURE_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
    }

    /// Arm the after-takeoff handoff. Spoken by [`handoff_tick`] once the
    /// flight is airborne and climbing through ~700 ft above the field —
    /// a tower doesn't say "contact departure" to an aircraft still rolling.
    fn arm_departure_handoff(&mut self, ctx: &tower::Ctx) {
        // Circuit traffic is the tower's own: the pattern comes back to this
        // runway, and "contact Departure" on the crosswind leg sends it to a
        // controller who does not work it (LEMG 31, 2026-09-28).
        if self.circuits {
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "departure_handoff_withheld", "why": "circuits"}),
            );
            tracing::info!("circuit traffic: the tower keeps it, no departure handoff");
            return;
        }
        // Prefer departure; fall back to approach; a tower-only field keeps
        // its traffic and no handoff arms.
        let (name, khz) = match (self.departure_khz, self.approach_khz) {
            (Some(k), _) => ("Departure", k),
            (None, Some(k)) => ("Approach", k),
            (None, None) => return,
        };
        self.dep_checked_in = false;
        // "Radar contact" is now the check-in's answer on the new frequency.
        if let Some(w) = SQUAWK_WATCH
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_mut()
        {
            w.check_in_owed = true;
        }
        self.handed_to = Some(if name == "Departure" {
            Service::Departure
        } else {
            Service::Approach
        });
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = Some(Handoff {
            line: format!(
                "{}, contact {} {} on {}.",
                ctx.callsign,
                ctx.airport,
                name,
                atis::spoken_mhz(khz)
            ),
            min_alt_ft: self.apt.elev_ft + 700.0,
            to: format!("{} {}", ctx.airport, name),
            khz,
        });
        tracing::info!(to = name, "departure handoff armed");
    }

    /// The departure has checked in with the position it was handed to: the
    /// tower's book on it is closed. The takeoff clearance is spent (the
    /// flight is airborne and someone else's), so the tower state goes back
    /// to Parked, the state every arrival starts from, and the published
    /// picture says so: nothing held, nothing owed.
    fn transfer_departure(&mut self) {
        if self.twr.state == tower::State::TakeoffCleared {
            self.twr.state = tower::State::Parked;
        }
        self.twr.clear_clearance_readback();
        if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
            c.state = self.twr.state;
            c.holds_runway = self.twr.holds_runway();
            c.pending_readback = None;
        }
    }

    /// The approach controller: vectors to final, or a redirect to the tower
    /// for anything that belongs there.
    fn respond_approach(&mut self, intent: tower::Intent, ctx: &tower::Ctx, wx: Wx) -> String {
        match intent {
            tower::Intent::RadioCheck => {
                format!(
                    "{}, {} Approach, reading you five.",
                    ctx.callsign, ctx.airport
                )
            }
            tower::Intent::WindCheck => {
                format!(
                    "{}, wind {}{}.",
                    ctx.callsign,
                    ctx.wind,
                    tower::qnh_clause(ctx)
                )
            }
            tower::Intent::SquawkCheck => {
                format!("{}, squawk seven zero zero zero.", ctx.callsign)
            }
            tower::Intent::RequestVectors => self.arm_vectors(ctx, wx),
            tower::Intent::RequestLanding { touch_and_go } => {
                // The option request rides through the vectors: the tower's
                // eventual clearance must say "touch and go", not "land".
                self.twr.expect_touch_and_go(touch_and_go);
                self.arm_vectors(ctx, wx)
            }
            // A mayday to approach: keep the flight, vector it home with
            // priority — an emergency is never told to go retune. On the
            // ground there is nothing to vector — send the trucks.
            tower::Intent::Mayday if ctx.on_ground => format!(
                "{}, roger mayday. Emergency services on their way to you — hold position if able.",
                ctx.callsign
            ),
            tower::Intent::Mayday => {
                self.twr.expect_touch_and_go(false);
                let vectors = self.arm_vectors(ctx, wx);
                format!("Roger mayday. {vectors}")
            }
            // "Established ILS two seven right" (parses as the final report)
            // while being vectored: approach's job is done — acknowledge,
            // hand the flight to the tower, drop the plan so the tick stops
            // steering a pilot who's already on the localizer.
            tower::Intent::ReportFinal
                if VECTORS.lock().unwrap_or_else(|e| e.into_inner()).is_some() =>
            {
                *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = None;
                format!(
                    "{}, roger. Contact {} Tower on {}.",
                    ctx.callsign,
                    ctx.airport,
                    atis::spoken_mhz(self.tower_khz)
                )
            }
            // Final reports and everything runway-shaped belong to the tower.
            tower::Intent::ReportFinal
            | tower::Intent::ReportDownwind
            | tower::Intent::ReportBase
            | tower::Intent::ReportCrosswind
            | tower::Intent::GoAround
            | tower::Intent::RejectingTakeoff
            | tower::Intent::RequestTakeoff
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestTaxiIn
            | tower::Intent::RunwayVacated
            | tower::Intent::RequestRunway
            | tower::Intent::RequestBacktrack
            | tower::Intent::CancelIfr
            | tower::Intent::RequestSid
            | tower::Intent::RequestStartup
            | tower::Intent::RequestShutdown
            | tower::Intent::RequestIfrClearance { .. }
            | tower::Intent::RequestTransit
            | tower::Intent::RequestClearance => format!(
                "{}, contact {} Tower on {}.",
                ctx.callsign,
                ctx.airport,
                atis::spoken_mhz(self.tower_khz)
            ),
            tower::Intent::Greeting => {
                format!(
                    "{}, {} Approach, pass your message.",
                    ctx.callsign, ctx.airport
                )
            }
            tower::Intent::RequestFrequencyChange => {
                format!("{}, frequency change approved.", ctx.callsign)
            }
            // Approach has the scope: assign the code; "radar contact" comes
            // from the watch tick once the code actually squawks.
            tower::Intent::RequestFlightFollowing => {
                let squawk = self.assign_squawk(&ctx.callsign, StationSpeaker::Approach);
                format!(
                    "{}, squawk {}. Maintain VFR, flight following.",
                    ctx.callsign, squawk
                )
            }
            tower::Intent::Unknown => {
                format!("{}, {} Approach, say again.", ctx.callsign, ctx.airport)
            }
        }
    }

    /// Arm the vectors-to-final plan and speak the opening instruction. The
    /// monitor's tick ([`vector_tick`]) takes it from here.
    fn arm_vectors(&mut self, ctx: &tower::Ctx, wx: Wx) -> String {
        // The runway END in use: its threshold is the aim point, its heading
        // the final approach course. Same held pick as every clearance, so
        // vectors and the tower always agree on the runway.
        let ident = self.pick_runway(wx.wind_from_deg, wx.wind_kts);
        if ident.is_empty() {
            return format!("{}, unable, no runway data.", ctx.callsign);
        }
        let Some((thr_lat, thr_lon, course_deg)) = self.apt.runways.iter().find_map(|r| {
            if r.ident_a == ident {
                Some((r.lat_a, r.lon_a, r.heading_a_deg()))
            } else if r.ident_b == ident {
                Some((r.lat_b, r.lon_b, r.heading_b_deg()))
            } else {
                None
            }
        }) else {
            return format!("{}, unable, no runway data.", ctx.callsign);
        };
        // ILS when the runway has one (background load may still be running:
        // then it's a visual, exactly as before — honest about what's known).
        let facts = ILS_FACTS.lock().unwrap_or_else(|e| e.into_inner()).clone();
        let ils_rec = facts
            .as_ref()
            .and_then(|f| f.ils.iter().find(|i| i.runway == ident).cloned());
        let ils = ils_rec.is_some();
        // Intercept altitude: the CIFP's real platform (glideslope-intercept)
        // when the procedure is on disk; otherwise a geometric one — 2,500 ft
        // above the field for an ILS (≈3° from 8 nm), 1,500 for the visual.
        let target_alt_ft = facts
            .as_ref()
            .filter(|_| ils)
            .and_then(|f| f.platform_ft.get(&ident).copied())
            .unwrap_or_else(|| {
                let aal = if ils { 2500.0 } else { 1500.0 };
                ((self.apt.elev_ft + aal) / 100.0).round() * 100.0
            });
        // The ILS join sits where the real slope (charted angle when we have
        // it) is still above the platform — intercept from below, always.
        let join_nm = match &ils_rec {
            Some(rec) => ils_join_nm(target_alt_ft - self.apt.elev_ft, rec.gs_deg.unwrap_or(3.0)),
            None => JOIN_FINAL_NM,
        };
        let runway_spoken = atis::spoken_runway(&ident);
        *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = Some(VectorPlan {
            callsign: ctx.callsign.clone(),
            thr_lat,
            thr_lon,
            course_deg,
            runway_spoken: runway_spoken.clone(),
            tower_spoken: Some(atis::spoken_mhz(self.tower_khz)),
            target_alt_ft,
            ils,
            join_nm,
            cleared: false,
            dialect: self.dialect,
            // The arm REPLY (squawk + expect vectors + descend) is about to be
            // spoken — start the vector clock now so the first heading follows
            // after a breath (8 s for a big cut, 25 s otherwise), not stacked
            // onto it. Three transmissions inside two seconds (reply, radar
            // contact, first vector) read as spam, taped 2026-07-12; a real
            // controller consolidates and then lets the pilot fly.
            last_tx: Some(Instant::now()),
            last_heading: None,
        });
        // A radar service starts with a discrete code — "radar contact" is
        // spoken by the watch tick once the code actually shows on the
        // scope, not on faith (it used to be instant theatre).
        let squawk = self.assign_squawk(&ctx.callsign, StationSpeaker::Approach);
        tracing::info!(runway = %ident, alt = target_alt_ft, ils, "vectors armed");
        // At or above the transition level the descent is to a flight level
        // and carries no QNH (the pressure goes with the first altitude).
        let level = atis::is_flight_level(target_alt_ft, ctx.transition_level_ft);
        let (icao_alt, faa_alt, qnh) = if level {
            let fl = atis::spoken_flight_level(target_alt_ft);
            (fl.clone(), fl, String::new())
        } else {
            (
                format!("to altitude {} feet", spoken_alt(target_alt_ft)),
                spoken_alt(target_alt_ft),
                tower::qnh_clause(ctx),
            )
        };
        let plan_phrase = match (ils, self.dialect) {
            // CAP 413 / ICAO: "vectoring ILS approach", altitudes are "to
            // altitude ... feet" with the QNH attached.
            (true, Dialect::Icao) => format!(
                "Vectoring ILS approach runway {}. Descend {icao_alt}{qnh}.",
                runway_spoken,
            ),
            // FAA: "expect vectors ILS runway ... approach", "descend and
            // maintain", and the pressure clause speaks altimeter in inches
            // (ctx.qnh carries the dialect-correct phrase).
            (true, Dialect::Faa) => format!(
                "Expect vectors ILS runway {} approach. Descend and maintain {faa_alt}{qnh}.",
                runway_spoken,
            ),
            (false, _) => format!(
                "Expect vectors, visual approach runway {}. Descend and maintain {faa_alt}{qnh}.",
                runway_spoken,
            ),
        };
        format!(
            "{}, {} Approach, squawk {}. {}",
            ctx.callsign, ctx.airport, squawk, plan_phrase
        )
    }

    /// Remember the runway a call names for itself, for the pick this call is
    /// about to make (see `pick_runway_with`): an arrival's, and a departure's
    /// asked for AT THE TAXI REQUEST ("request taxi, runway two six"), where a
    /// pilot with a planned departure asks for it. The ready-for-departure
    /// call is told its runway. LOWI (2026-09-24): calm, the pick could have
    /// been 08, and the planned MOGT3H exists only from 26. The wind still
    /// wins when it really favours the other end.
    fn note_stated_runway(&mut self, transcript: &str, intent: tower::Intent) {
        self.stated_runway = match intent {
            tower::Intent::ReportFinal
            | tower::Intent::ReportDownwind
            | tower::Intent::ReportBase
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestLanding { .. }
            // A departure's clearance request may name its runway, like the
            // taxi request; a runway start names the strip it is lined up on
            // ("request clearance, runway one three", `note_caller_on_strip`).
            | tower::Intent::RequestClearance => tower::stated_runway(transcript),
            _ => None,
        };
        self.stated_on_strip = false;
        self.runway_request = match intent {
            tower::Intent::RequestRunway => tower::requested_runway(transcript),
            _ => None,
        };
        self.requested_ident = None;
    }

    /// The caller of the stated runway stands ON that strip, on the ground:
    /// a runway start (the sim put it there, lined up). Its runway stands
    /// against the calm-wind arrival stream as long as the wind does not
    /// argue (`pick_runway_with`). Pure but for the flag.
    fn note_caller_on_strip(&mut self, on_ground: bool, lat: f64, lon: f64) {
        self.stated_on_strip = on_ground
            && self.stated_ident().is_some_and(|ident| {
                strip(&self.apt.runways, &ident).is_some_and(|r| on_runway(r, lat, lon).is_some())
            });
    }

    /// A RUNWAY CHANGE IS SAID. The runway in use may turn after a flight's
    /// clearance named another (the calm-wind stream, `pick_runway_with`);
    /// the next ground reply that sends it to the new runway opens "runway
    /// change, runway one three in use". LEMG (sortie 1790631369-full_auto,
    /// 2026-09-28, #178): a runway start on 30 was cleared for the circuit
    /// on 30, a lander on 12 turned the field round to 13, and "ready for
    /// departure" earned "taxi to holding point Hotel November one runway
    /// one three first" with nothing to say the runway had moved. Airborne
    /// forgets it: an arrival's runway is its own business.
    fn state_runway_change(
        &mut self,
        intent: &tower::Intent,
        ctx: &tower::Ctx,
        reply: String,
    ) -> String {
        if !ctx.on_ground {
            self.told_runway = None;
            self.start_strip = None;
            return reply;
        }
        if reply.is_empty() {
            return reply;
        }
        let now = self.runway_in_use.clone();
        match intent {
            tower::Intent::RequestClearance
            | tower::Intent::RequestIfrClearance { .. }
            | tower::Intent::RequestRunway => {
                self.told_runway = now;
                reply
            }
            tower::Intent::RequestTakeoff
            | tower::Intent::RequestTaxi
            | tower::Intent::RequestBacktrack => {
                let from = std::mem::replace(&mut self.told_runway, now.clone());
                match (from, now) {
                    (Some(from), Some(to)) if from != to => {
                        sim::blackbox::record(
                            "atc",
                            serde_json::json!({"event": "runway_change_stated",
                                "from": from, "to": to}),
                        );
                        tower::runway_change_line(&ctx.callsign, &ctx.runway, &reply)
                    }
                    _ => reply,
                }
            }
            _ => reply,
        }
    }

    /// The strip end the held stated runway names at this field.
    fn stated_ident(&self) -> Option<String> {
        let stated = self.stated_runway.as_deref()?;
        self.apt
            .runways
            .iter()
            .flat_map(|r| [r.ident_a.clone(), r.ident_b.clone()])
            .find(|id| tower::runway_matches(id, stated))
    }

    /// A flight the tower is sequencing owns the runway in use (see
    /// `pick_runway_with`).
    fn runway_committed(&self) -> bool {
        matches!(
            self.twr.state,
            tower::State::TaxiCleared
                | tower::State::LinedUp
                | tower::State::TakeoffCleared
                | tower::State::Inbound
        ) || (self.twr.state == tower::State::ClearedToLand && !self.twr.runway_vacated())
            || (self.ifr_runway.is_some() && self.ifr_runway == self.runway_in_use)
    }

    /// Answer a "request runway ..." call's context: the runway it named,
    /// resolved against this field (`resolve_runway_request`). Granted, the
    /// ctx names THAT runway and, on the ground, the taxi route is re-planned
    /// to it from `pos` (as `RequestTaxi` plans one to the runway in use);
    /// refused, the ctx carries the reason. Pure but for the route planner.
    fn apply_runway_request(&mut self, ctx: &mut tower::Ctx, pos: Option<(f64, f64)>) {
        match resolve_runway_request(&self.apt.runways, self.runway_request.as_deref()) {
            Err(refusal) => {
                self.requested_ident = None;
                ctx.runway_refusal = Some(refusal);
            }
            Ok(ident) => {
                ctx.runway = atis::spoken_runway(&ident);
                ctx.runway_refusal = None;
                if ctx.on_ground {
                    self.planned_route = pos.and_then(|(lat, lon)| {
                        plan_taxi_route_to(&self.apt.runways, &ident, lat, lon)
                    });
                    let route = self.planned_route.as_ref();
                    ctx.holding_point = route
                        .map(|r| spoken_taxiway(&r.holding_point))
                        .filter(|h| !h.is_empty())
                        .unwrap_or_else(|| "Alpha".to_string());
                    ctx.taxi_via = route.map(|r| spoken_via(&r.via)).unwrap_or_default();
                    ctx.taxi_crossings = route
                        .map(|r| {
                            r.crossings
                                .iter()
                                .map(|n| atis::spoken_runway(first_designator(n)))
                                .collect()
                        })
                        .unwrap_or_default();
                }
                self.requested_ident = Some(ident);
            }
        }
    }

    /// The runway in use. Re-picked from the wind only when there IS wind
    /// (≥ 5 kt): in calm the into-wind pick flip-flops on noise, and a
    /// controller doesn't swap the runway between two transmissions.
    fn pick_runway(&mut self, wind_from: f64, wind_kts: f64) -> String {
        self.pick_runway_with(wind_from, wind_kts, None, None)
    }

    /// [`pick_runway`] with the live arrival stream: in calm wind, with no
    /// runway in use yet, the end the traffic is landing on is the runway
    /// in use. LXGB at 07001KT: the tower picked 27 while LiveTraffic was
    /// landing 09 (the real field's runway), and cleared a departure into
    /// the reciprocal stream (2026-09-15).
    ///
    /// `arrival_end` is the live lander on final, else the remembered
    /// touchdowns, and `departure_end` the remembered liftoffs, both from
    /// [`field_streams`] (issue #164).
    ///
    /// `departure_end` is the live DEPARTURE stream ([`departure_end`]): with
    /// no arrival stream it is the runway in use in calm the same way, and a
    /// stated runway on its reciprocal is refused. ONE STRIP, ONE DIRECTION:
    /// in calm an uncommitted runway in use that points against the live
    /// stream is turned round to it. LEMG (sortie 1790584715-full_auto,
    /// 2026-09-28): 31 stood in use from an earlier into-wind pick, the
    /// replay departed 13 off the same pavement and landed 12, and the
    /// tower cleared our circuit to land 31 while Vueling and Easy rolled
    /// head-on toward it.
    fn pick_runway_with(
        &mut self,
        wind_from: f64,
        wind_kts: f64,
        arrival_end: Option<&str>,
        departure_end: Option<&str>,
    ) -> String {
        // The stream the field is working: arrivals first (where the traffic
        // is landing is what the field IS), else the last departures.
        let stream_end = arrival_end
            .filter(|e| strip(&self.apt.runways, e).is_some())
            .or(departure_end.filter(|e| strip(&self.apt.runways, e).is_some()));
        // A RUNWAY START IS GIVEN THE RUNWAY IT IS LINED UP ON when the wind
        // does not argue: an arrival stream on another end sequences around
        // a departure already on its runway, it does not send it off to taxi.
        // LEMG (sortie 1790584715-full_auto, 2026-09-28): the 737 set up on
        // 13's threshold in 342 at 3 kt was cleared off 31 and taxied 400 s
        // round the field to line up there.
        // ...but never INTO the live traffic: a start whose end points against
        // the arrival or departure stream (13 departures, a start on 31) is
        // the head-on of issue #140, refused and left to the stream below.
        // ...and a start's own strip carries a pilot's tailwind, not the
        // runway-in-use's: LEMG 13 (sortie 1790646817-full_auto, 2026-09-29,
        // #178), lined up on 13 with no traffic, 293 at 7.3 was 6.9 kt down
        // 13, over the 5 kt of STATED_RUNWAY_TAILWIND_KTS, and the circuit
        // was cleared on 30 across the field. Once given, the strip is HELD
        // for the start's later calls (`start_strip`): 30's 14 kt better
        // headwind would win the wind pick's hysteresis on the ready call.
        let start_ok = |st: &Self, ident: &str| {
            let into_stream = [arrival_end, departure_end]
                .into_iter()
                .flatten()
                .any(|end| ends_oppose(&st.apt.runways, end, ident));
            let hw = nav::headwind_for(&st.apt.runways, ident, wind_from, wind_kts);
            let best = nav::best_runway_for_wind(&st.apt.runways, wind_from, wind_kts);
            !into_stream
                && runway_stands_within(
                    &st.apt.runways,
                    ident,
                    hw,
                    best,
                    wind_from,
                    wind_kts,
                    ON_STRIP_TAILWIND_KTS,
                )
        };
        if std::mem::take(&mut self.stated_on_strip) && !self.runway_committed() {
            if let Some(ident) = self.stated_ident() {
                if start_ok(self, &ident) {
                    self.stated_runway = None;
                    self.runway_in_use = Some(ident.clone());
                    self.start_strip = Some(ident.clone());
                    return ident;
                }
            }
        }
        if let Some(held) = self.start_strip.take() {
            let restated = self
                .stated_runway
                .as_deref()
                .is_some_and(|s| !tower::runway_matches(&held, s));
            if !restated
                && self.runway_in_use.as_deref() == Some(held.as_str())
                && start_ok(self, &held)
            {
                self.stated_runway = None;
                self.start_strip = Some(held.clone());
                return held;
            }
        }
        if wind_kts < CALM_WIND_KTS && self.runway_in_use.is_none() {
            if let Some(end) = stream_end {
                self.runway_in_use = Some(end.to_string());
                return end.to_string();
            }
        }
        // A departure being SEQUENCED owns its runway. Once a taxi clearance
        // names one, the pick is frozen until the flight leaves or the
        // clearance dies — a controller does not move you to the other
        // parallel between your taxi clearance and your takeoff clearance.
        // ...and so does an ARRIVAL told to join it or cleared to land on it.
        // A go-around takes the state from ClearedToLand back to Inbound; on
        // a wind dead across the strip the pick flipped in that moment, the
        // party line published the other end, and the crew's approach was
        // re-aimed at it from 60 ft (EGTE 08 to 26, 2026-09-21).
        // A landed arrival that has REPORTED the runway vacated no longer
        // owns it: the pick is free to follow the wind and the stream again.
        // ...and so does an IFR clearance: it named the runway and the SID
        // off it, and the taxi that follows goes to THAT runway
        // (`runway_committed`).
        let committed = self.runway_committed();
        if committed && self.runway_in_use.is_some() {
            return self.runway_in_use.clone().unwrap_or_default();
        }
        // AN ARRIVAL'S OWN RUNWAY STANDS WHEN THE WIND DOES NOT ARGUE. "Final
        // runway two six" into 350 at 8, dead across both ends, earned
        // "negative, runway zero eight": a headwind of a fraction of a knot
        // sent a 737 round the field from a five mile final (same sortie). A
        // controller with nothing else on the runway clears the end the
        // traffic is lined up on. Only uncommitted, only without an arrival
        // stream on another end.
        // "Does not argue" is the pick's own calm (under CALM_WIND_KTS any
        // end stands), a tailwind within STATED_RUNWAY_TAILWIND_KTS on no
        // worse a crosswind than the best end's, or a headwind inside the
        // swap margin. LEMG (sortie 1790580676-approach, 2026-09-28): "10
        // mile final runway one three" into 350 at 3 earned "negative, runway
        // three one" because 13's -2.3 kt trailed 31's +2.3 kt by 4.7, over
        // the 3 kt margin, in a wind the same function calls calm.
        if let Some(stated) = self.stated_runway.take() {
            let ident = self
                .apt
                .runways
                .iter()
                .flat_map(|r| [r.ident_a.clone(), r.ident_b.clone()])
                .find(|id| tower::runway_matches(id, &stated));
            if let Some(ident) = ident {
                // The departures' reciprocal is refused too: landing into
                // them is the head-on the stream rule exists to prevent.
                let stream_elsewhere = arrival_end.is_some_and(|end| end != ident)
                    || departure_end
                        .is_some_and(|end| ends_oppose(&self.apt.runways, end, &ident));
                let hw = nav::headwind_for(&self.apt.runways, &ident, wind_from, wind_kts);
                let best = nav::best_runway_for_wind(&self.apt.runways, wind_from, wind_kts);
                let wind_indifferent = stated_runway_stands(
                    &self.apt.runways,
                    &ident,
                    hw,
                    best.clone(),
                    wind_from,
                    wind_kts,
                );
                if !stream_elsewhere && wind_indifferent {
                    self.runway_in_use = Some(ident.clone());
                    return ident;
                }
                // A refused stated runway names its reason on file: the
                // stream's end or the wind's numbers. LEMG 13 into 060 at 6
                // (sortie 1790583663-approach) was refused with the wind
                // arithmetic saying it stands (headwind 2.0 against 12's 3.3,
                // crosswind 5.7 against 5.0), and nothing recorded why.
                sim::blackbox::record(
                    "atc",
                    serde_json::json!({"event": "stated_runway_refused", "stated": ident,
                        "stream_end": arrival_end, "departure_end": departure_end,
                        "headwind_kts": hw,
                        "best": best.as_ref().map(|b| b.0.clone()),
                        "best_headwind_kts": best.as_ref().map(|b| b.2),
                        "wind_from": wind_from, "wind_kts": wind_kts,
                        "wind_indifferent": wind_indifferent}),
                );
            }
        }
        // One strip, one direction: in calm the stream's direction is the
        // runway in use. A pick that points against it (the reciprocal, or a
        // near-parallel's opposite end) turns round to the stream's end; the
        // wind below never moves a calm pick, so nothing else would.
        if wind_kts < CALM_WIND_KTS {
            if let (Some(cur), Some(end)) = (self.runway_in_use.clone(), stream_end) {
                if ends_oppose(&self.apt.runways, &cur, end) {
                    tracing::info!(from = %cur, to = %end, "runway in use turned to the live stream");
                    sim::blackbox::record(
                        "atc",
                        serde_json::json!({"event": "runway_follows_stream", "from": cur,
                            "to": end, "arrival_end": arrival_end,
                            "departure_end": departure_end, "wind_kts": wind_kts}),
                    );
                    self.runway_in_use = Some(end.to_string());
                }
            }
        }
        if wind_kts >= CALM_WIND_KTS || self.runway_in_use.is_none() {
            if let Some((id, _, hw)) =
                nav::best_runway_for_wind(&self.apt.runways, wind_from, wind_kts)
            {
                // Hysteresis: the runway in use keeps its job unless another
                // beats it by a REAL margin. The >= 5 kt guard was meant to
                // stop the flip-flop, but between two parallels the headwinds
                // differ by rounding error at any wind speed — EGLL's strips
                // are both 270 degrees to within a hundredth of a degree, so
                // which one won was decided by noise in the wind direction,
                // and nothing in the phraseology gives it away (both are "two
                // seven"). A silent flip from the 27L taxi clearance to 27R
                // pointed the takeoff hold at Heathrow's ARRIVAL stream, which
                // never has a 15 s gap: held at November one for the full
                // eight minutes and handed back, "traffic on short final" for
                // a departure runway that was empty (2026-07-27).
                let current = self
                    .runway_in_use
                    .as_deref()
                    .and_then(|cur| nav::headwind_for(&self.apt.runways, cur, wind_from, wind_kts));
                if current.is_none_or(|cur_hw| hw > cur_hw + RUNWAY_SWAP_MARGIN_KTS) {
                    self.runway_in_use = Some(id);
                }
            }
        }
        // An uncommitted pick sitting on a CLOSE parallel's standby moves to
        // the main runway: the hysteresis above can never make that swap
        // (their headwinds agree to rounding), so a 26R picked on noise
        // before the fix, or by a calm-wind stream, would stand for good.
        if let Some(cur) = self.runway_in_use.as_deref() {
            let main = nav::close_parallel_main(&self.apt.runways, cur);
            if main != cur {
                tracing::info!(from = %cur, to = %main, "runway in use moved to the main runway");
                self.runway_in_use = Some(main);
            }
        }
        self.runway_in_use.clone().unwrap_or_default()
    }

    /// Read the sim once and build the live context for a reply. Best-effort:
    /// a failed read falls back to silence on that item rather than a wrong
    /// number.
    async fn build_ctx(&mut self, xp: SimView<'_>, intent: tower::Intent) -> (tower::Ctx, Wx) {
        let mut wx = Wx {
            wind_from_deg: xp
                .get_float(datarefs::WIND_DIR_DEG_TRUE)
                .await
                .unwrap_or(0.0),
            wind_kts: xp.get_float(datarefs::WIND_SPEED_MSC).await.unwrap_or(0.0) * 1.943844,
        };
        let qnh = match xp.get_float(datarefs::QNH_PASCALS).await {
            Ok(pa) => spoken_pressure(pa, self.dialect),
            Err(_) => String::new(),
        };
        // One resolver with the crew (override > SimBrief > tail). The wake
        // category rides the sim's own MTOW, so the tower says "heavy" to an
        // A330 without the crew's profile in its dep tree.
        let callsign = crate::callsign::live(xp)
            .await
            .map(|r| r.spoken)
            .unwrap_or_else(|| "aircraft calling".to_string());
        let on_ground = xp.get_float(datarefs::ON_GROUND_ANY).await.unwrap_or(0.0) > 0.5;
        // An IFR clearance's runway is held for its departure only: airborne
        // it has been flown (or abandoned), and the pick is free again.
        // So is a runway start's strip (`start_strip`).
        if !on_ground {
            self.ifr_runway = None;
            self.start_strip = None;
        }
        // The wind a tower reads out and picks its runway by is the wind ON
        // THE FIELD. The sim's wind datarefs are the air the aircraft is in:
        // for a caller on the ground the same thing, for an arrival at
        // 2,700 ft the winds aloft (350 at 8 against the field's 190 at 3,
        // EGTE 2026-09-21: crew and tower picked opposite ends).
        if !on_ground {
            if let Some((from, kts)) = surface_wind_at(self.apt.position()) {
                wx.wind_from_deg = from;
                wx.wind_kts = kts;
            }
        }
        // The controller's own traffic picture, read once per call: an
        // arrival in OUR runway's corridor gates takeoff clearances and
        // sequences downwind joins; a ghost ON the strip blocks line-ups.
        // Restricted to the strip in use — the runway-blind version held a
        // Heathrow 09R departure for the 09L stream, indefinitely
        // (2026-07-10).
        let targets = read_tcas_targets(xp).await;
        let now = Instant::now();
        note_ground_traffic(&targets, now);
        *TOWER_WIND.lock().unwrap_or_else(|e| e.into_inner()) =
            Some((wx.wind_from_deg, wx.wind_kts));
        // The live lander on final, else the remembered touchdowns (#164).
        let (arrival_end, departure_end) = field_streams(
            &self.apt.runways,
            arrival_end(&targets, &self.apt.runways, self.apt.elev_ft),
            now,
        );
        if on_ground && self.stated_runway.is_some() {
            if let (Ok(lat), Ok(lon)) = (
                xp.get_float(datarefs::LATITUDE).await,
                xp.get_float(datarefs::LONGITUDE).await,
            ) {
                self.note_caller_on_strip(on_ground, lat, lon);
            }
        }
        let ident = self.pick_runway_with(
            wx.wind_from_deg,
            wx.wind_kts,
            arrival_end.as_deref(),
            departure_end.as_deref(),
        );
        // Our own position, for a takeoff request made ON the strip: only
        // the pavement ahead of us is in the way of the roll.
        let own = if on_ground && intent == tower::Intent::RequestTakeoff {
            match (
                xp.get_float(datarefs::LATITUDE).await,
                xp.get_float(datarefs::LONGITUDE).await,
            ) {
                (Ok(lat), Ok(lon)) => Some((lat, lon)),
                _ => None,
            }
        } else {
            None
        };
        let (final_traffic, runway_occupied) = match strip(&self.apt.runways, &ident) {
            Some(r) => {
                let r = r.clone();
                // The landing judgement's own test: on the pavement and not
                // stale data (`occupies`, issue #162); a departure standing
                // on the strip is not held by a ghost behind its tail (#172).
                let occupied = departure_strip_occupied(&r, &ident, &targets, own, now);
                (final_traffic_for(&targets, &r, &self.apt.runways, self.apt.elev_ft), occupied)
            }
            None => (None, false),
        };
        let wake_secs_left = if on_ground {
            wake_remaining(&ident, Instant::now())
        } else {
            None
        };
        // The taxi route from where the aircraft actually is, planned fresh
        // per ground call so the eventual clearance matches the stand you
        // are on, not the one you asked from an hour ago.
        // An ARRIVAL taxi-in (the pilot said "taxi to parking" — RequestTaxiIn,
        // or a bare "request taxi" while still holding a landing clearance)
        // routes to the nearest STAND; everything else is a departure to the
        // runway. Keyed on the INTENT, not the FSM state — the state falls back
        // to Parked after landing (region handoff, a prior taxi-in), which used
        // to leave a real "taxi to parking" answered flat with no route
        // (2026-07-26).
        let arrival = matches!(intent, tower::Intent::RequestTaxiIn)
            || (matches!(intent, tower::Intent::RequestTaxi)
                && self.twr.state == tower::State::ClearedToLand);
        self.planned_route = None;
        let mut own_on_runway = false;
        let mut pos = None;
        if on_ground {
            if let (Ok(lat), Ok(lon)) = (
                xp.get_float(datarefs::LATITUDE).await,
                xp.get_float(datarefs::LONGITUDE).await,
            ) {
                pos = Some((lat, lon));
                own_on_runway = strip(&self.apt.runways, &ident)
                    .is_some_and(|r| on_runway(r, lat, lon).is_some());
                self.planned_route = if arrival {
                    // The heading says which way is forward off a runway we
                    // may still be on; a failed read plans as before.
                    let psi = xp.get_float(TRUE_PSI).await.ok();
                    plan_taxi_route_to_stand(&self.apt.runways, lat, lon, psi)
                } else {
                    plan_taxi_route_to(&self.apt.runways, &ident, lat, lon)
                };
            }
        }
        let mut ctx = self.make_ctx(
            wx,
            qnh,
            callsign,
            on_ground,
            arrival,
            final_traffic,
            runway_occupied,
            wake_secs_left,
        );
        ctx.own_on_runway = own_on_runway;
        if intent == tower::Intent::RequestRunway {
            self.apply_runway_request(&mut ctx, pos);
        }
        ctx.helicopter = xp
            .get_float("sim/aircraft2/metadata/is_helicopter")
            .await
            .is_ok_and(|v| v > 0.5);
        (ctx, wx)
    }

    /// The pure half of [`Station::build_ctx`] — everything after the sim
    /// reads, so tests phrase clearances from scripted weather.
    #[allow(clippy::too_many_arguments)]
    fn make_ctx(
        &mut self,
        wx: Wx,
        qnh: String,
        callsign: String,
        on_ground: bool,
        arrival: bool,
        final_traffic: Option<FinalTraffic>,
        runway_occupied: bool,
        wake_secs_left: Option<u64>,
    ) -> tower::Ctx {
        let runway = self.pick_runway(wx.wind_from_deg, wx.wind_kts);
        // "With information X" matching the current broadcast: the clearance
        // skips the QNH — having the numbers is the whole point of an ATIS.
        // A stale or absent letter keeps them (safety over brevity).
        let qnh = if self.pilot_info.is_some() && self.pilot_info == atis_current_letter() {
            String::new()
        } else {
            qnh
        };
        let route = self.planned_route.as_ref();
        tower::Ctx {
            callsign,
            airport: spoken_airport(&self.apt.name).to_string(),
            runway: atis::spoken_runway(&runway),
            qnh,
            wind: spoken_wind(wx.wind_from_deg, wx.wind_kts),
            holding_point: route
                .map(|r| spoken_taxiway(&r.holding_point))
                .filter(|h| !h.is_empty())
                .unwrap_or_else(|| "Alpha".to_string()),
            // Arrival taxi-in (route was planned to a stand): the spoken stand
            // name for "taxi to stand ...". Empty on a departure.
            stand: if arrival {
                route
                    .map(|r| spoken_stand(&r.holding_point))
                    .unwrap_or_default()
            } else {
                String::new()
            },
            approach: self.approach_khz.map(atis::spoken_mhz),
            squawk: self
                .assigned_squawk
                .map(|c| atis::spoken_digits(&c.to_string()))
                .unwrap_or_else(|| "seven zero zero zero".to_string()),
            final_traffic_nm: final_traffic.as_ref().map(|f| f.nm),
            final_traffic_callsign: final_traffic.as_ref().and_then(|f| f.callsign.clone()),
            final_traffic_count: final_traffic.as_ref().map(|f| f.count).unwrap_or(0),
            runway_occupied,
            wake_secs_left,
            taxi_via: route.map(|r| spoken_via(&r.via)).unwrap_or_default(),
            taxi_crossings: route
                .map(|r| {
                    r.crossings
                        .iter()
                        .map(|n| atis::spoken_runway(first_designator(n)))
                        .collect()
                })
                .unwrap_or_default(),
            on_ground,
            // The sim read is `build_ctx`'s; scripted contexts are off it.
            own_on_runway: false,
            helicopter: false,
            ifr: None,
            circuit: None,
            runway_refusal: None,
            transition_level_ft: self.apt.transition_level_ft,
        }
    }
}

/// Does an UNKEYED utterance sound like a radio call? Addressed to a station
/// by name, or built from call vocabulary a pilot only uses on the radio.
/// This is the content-routing gate: with the radio PTT keyed, keying itself
/// is the addressing and this is never consulted.
/// The field's SPOKEN name. apt.dat names arrive as gazetteer entries —
/// "Badajoz- Talavera La Real", "Merida - Royanejos" — but a controller keys
/// "Badajoz Tower", not the full hyphenated row (heard live 2026-07-10, and
/// it's also what the pilot actually says back, so addressing works too).
/// First dash-separated segment. Accepted tradeoff: "Paris - Charles de
/// Gaulle" shortens to "Paris"; for the GA fields this copilot flies, the
/// first segment is the radio name.
pub fn spoken_airport(name: &str) -> &str {
    name.split('-')
        .map(str::trim)
        .find(|s| !s.is_empty())
        .unwrap_or(name)
}

pub fn is_radio_call(t: &str, airport_lc: &str) -> bool {
    let word = |w: &str| t.split(|c: char| !c.is_alphanumeric()).any(|tok| tok == w);
    t.contains("tower")
        || t.contains("approach")
        || t.contains("departure")
        || (!airport_lc.is_empty() && t.contains(airport_lc))
        || t.contains("request")
        || t.contains("inbound")
        || t.contains("radio check")
        || word("final")
        // "Established (ILS/localizer)" is the vectored arrival's final
        // report — radio speech by nature, like the pattern calls below.
        || t.contains("establish")
        // Distress and pattern calls are radio speech by nature — nobody
        // tells their copilot "mayday" or reports "downwind" to the cockpit.
        || word("mayday")
        || t.contains("may day")
        || t.contains("pan pan")
        || word("declaring")
        || word("declare")
        || t.contains("going around")
        || t.contains("missed approach")
        || word("downwind")
        || word("crosswind")
        || word("upwind")
        || t.contains("turning base")
        || t.contains("left base")
        || t.contains("right base")
        || t.contains("base leg")
        // "Runway vacated" is radio speech for the same reason a pattern call
        // is: nobody tells their copilot they have left the runway, and the
        // tower is the one party that needs to know. It earns an
        // acknowledgement and nothing else, so routing it here cannot buy a
        // clearance. "Clear of the runway" needs the runway word with it —
        // "clear left", "all clear" and "clear to taxi" are not this call.
        || word("vacated")
        || word("vacating")
        || t.contains("clear of the runway")
        || t.contains("clear of runway")
        // The IFR pilot's whole request is "ready to copy": nobody says it
        // to their copilot. "IFR" alone is not radio speech ("we're IFR
        // today" is crew talk); the IFR request carries "request" anyway.
        || t.contains("ready to copy")
        // "Cancel IFR" is said to a controller, never to the copilot.
        || tower::parse_intent(t) == tower::Intent::CancelIfr
        // "Request the Lam five Papa departure" names a procedure only a
        // controller issues.
        || tower::requested_departure(t).is_some()
}

/// Why an IFR clearance names the SID it does (blackbox `ifr_clearance`
/// `sid_basis`, `sid_join`).
#[derive(Debug, Clone, PartialEq)]
enum SidBasis {
    /// The SID off the runway that joins the filed route, at `join`.
    Route { join: String },
    /// A filed route no SID off the runway joins: runway heading.
    NoneJoins,
    /// No filed route: the exit nearest the destination's bearing.
    Bearing,
    /// No positioned exits: the first SID in the file for the runway.
    FirstInFile,
}

impl SidBasis {
    fn label(&self) -> &'static str {
        match self {
            SidBasis::Route { .. } => "route",
            SidBasis::NoneJoins => "none_joins",
            SidBasis::Bearing => "bearing",
            SidBasis::FirstInFile => "first_in_file",
        }
    }

    fn join(&self) -> Option<&str> {
        match self {
            SidBasis::Route { join } => Some(join),
            _ => None,
        }
    }
}

/// Publish an IFR clearance whose readback the tower accepted, for the
/// crew (`party_line::assigned_clearance`), and blackbox it item by item.
fn publish_ifr_clearance(
    c: tower::IfrClearance,
    callsign: &str,
    bearings: Option<(f64, f64)>,
    basis: Option<&SidBasis>,
) {
    tracing::info!(dest = %c.dest, sid = ?c.sid, runway = %c.runway, alt = c.initial_alt_ft,
                   squawk = c.squawk, "IFR clearance read back and accepted");
    sim::blackbox::record(
        "atc",
        serde_json::json!({
            "event": "ifr_clearance", "side": "tower", "callsign": callsign,
            "dest": c.dest, "sid": c.sid, "runway": c.runway,
            "initial_alt_ft": c.initial_alt_ft, "squawk": c.squawk,
            "sid_exit_brg": bearings.map(|b| b.0.round()),
            "dest_brg": bearings.map(|b| b.1.round()),
            "sid_basis": basis.map(SidBasis::label),
            "sid_join": basis.and_then(SidBasis::join),
        }),
    );
    *IFR_FLIGHT.lock().unwrap_or_else(|e| e.into_inner()) = Some(IfrFlight {
        code: c.squawk,
        callsign: callsign.to_string(),
        airborne: false,
    });
    *IFR_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = Some(c);
}

/// An IFR destination as heard, resolved against the region index: a code
/// first (three letters also with the K a US pilot drops), then the name
/// words, every one a whole word of the field's name. Several fields by one
/// name: the one whose radio name IS the words, else none (a guess would
/// clear the flight somewhere it never filed). `(icao, apt.dat name)`;
/// `None` = unknown to the index (or the index not loaded).
pub(crate) fn resolve_destination(dest: &str) -> Option<(String, String, (f64, f64))> {
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    let dest = dest.trim();
    let single = !dest.contains(' ')
        && (3..=4).contains(&dest.len())
        && dest.chars().all(|c| c.is_ascii_alphanumeric());
    if single {
        let up = dest.to_ascii_uppercase();
        let mut codes = vec![up.clone()];
        if up.len() == 3 {
            codes.push(format!("K{up}"));
        }
        for code in &codes {
            if let Some(a) = region.iter().find(|a| a.icao.eq_ignore_ascii_case(code)) {
                return Some((a.icao.clone(), a.apt.name.clone(), (a.lat, a.lon)));
            }
        }
        // Beyond the region index: any land field by its code.
        let dir = FIELD_DIR.lock().unwrap_or_else(|e| e.into_inner());
        for code in &codes {
            if let Some((icao, name, pos)) = dir.iter().find(|(i, _, _)| i.eq_ignore_ascii_case(code)) {
                return Some((icao.clone(), name.clone(), *pos));
            }
        }
    }
    let words_of = |s: &str| -> Vec<String> {
        s.to_lowercase()
            .split(|c: char| !c.is_alphanumeric())
            .filter(|w| !w.is_empty())
            .map(str::to_string)
            .collect()
    };
    let query = words_of(dest);
    if query.is_empty() {
        return None;
    }
    let matches: Vec<&RegionApt> = region
        .iter()
        .filter(|a| {
            let name = words_of(&a.apt.name);
            query.iter().all(|q| name.contains(q))
        })
        .collect();
    let pick = match matches.as_slice() {
        [one] => Some(*one),
        [] => None,
        many => {
            let exact: Vec<&&RegionApt> = many
                .iter()
                .filter(|a| words_of(spoken_airport(&a.apt.name)) == query)
                .collect();
            match exact.as_slice() {
                [one] => Some(**one),
                _ => None,
            }
        }
    }?;
    Some((pick.icao.clone(), pick.apt.name.clone(), (pick.lat, pick.lon)))
}

/// A requested runway resolved against the field: `Ok(ident)` for a strip
/// end the tower hands out (`nav::assignable_runways`), else the spoken
/// (runway, reason) the refusal says. No runway in the words is refused
/// with an empty runway, which the tower answers with "say again": a
/// mishear never buys a runway.
fn resolve_runway_request(
    runways: &[nav::Runway],
    heard: Option<&str>,
) -> Result<String, (String, String)> {
    let Some(canon) = heard else {
        return Err((String::new(), String::new()));
    };
    let spoken_canon = || {
        let digits: String = canon.chars().take_while(|c| c.is_ascii_digit()).collect();
        let padded = if digits.len() == 1 {
            format!("0{canon}")
        } else {
            canon.to_string()
        };
        atis::spoken_runway(&padded)
    };
    let Some(ident) = runways
        .iter()
        .flat_map(|r| [&r.ident_a, &r.ident_b])
        .find(|id| tower::runway_matches(id, canon))
        .cloned()
    else {
        return Err((spoken_canon(), "no such runway here".to_string()));
    };
    let assignable = nav::assignable_runways(runways)
        .any(|r| r.ident_a == ident || r.ident_b == ident);
    if !assignable {
        return Err((
            atis::spoken_runway(&ident),
            "that runway is not available".to_string(),
        ));
    }
    Ok(ident)
}

/// This flight's discrete squawk, deterministic from the callsign so tests
/// and a restarted session reproduce it. Codes live in the 45xx block with
/// octal last digits — every code valid, and the block is clear of all
/// reserved codes (0000, 1200, 2000, 7000, 7500, 7600, 7700).
pub fn discrete_squawk(callsign: &str) -> i64 {
    let seed: u32 = callsign.bytes().map(u32::from).sum();
    4500 + ((seed / 8) % 8) as i64 * 10 + (seed % 8) as i64
}

/// The radar watch on an assigned squawk: armed when a controller issues a
/// discrete code, ticked by the monitor. "Radar contact" is spoken ONCE the
/// sim actually shows the code squawking — until then it's an instruction,
/// not theatre — and while it doesn't, the controller nudges ("confirm
/// squawk ...") a couple of times, then gives up chasing like a real one.
pub struct SquawkWatch {
    pub code: i64,
    callsign: String,
    speaker: StationSpeaker,
    /// "Radar contact" has been spoken; the watch is satisfied.
    observed: bool,
    /// A departure handoff is armed and the flight has not checked in: the
    /// check-in's reply says "radar contact", so the watch holds it.
    check_in_owed: bool,
    /// First tick after assignment — starts the grace period.
    first_seen: Option<Instant>,
    last_nudge: Option<Instant>,
    nudges: u32,
}

static SQUAWK_WATCH: Mutex<Option<SquawkWatch>> = Mutex::new(None);

/// Knob-spinning time before the first nudge.
const SQUAWK_GRACE_SECS: f64 = 20.0;
/// Quiet time between nudges.
const SQUAWK_NUDGE_SECS: f64 = 45.0;
/// Then the controller stops chasing.
const SQUAWK_MAX_NUDGES: u32 = 2;

/// Cheap gate for the monitor: only read the transponder datarefs while a
/// watch is armed.
pub fn squawk_watch_active() -> bool {
    SQUAWK_WATCH
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .is_some_and(|w| !w.observed)
}

/// Can this position say "radar contact"? Only a radar unit: a tower works
/// by eye, and never relays another unit's contact.
fn radar_unit(s: StationSpeaker) -> bool {
    matches!(s, StationSpeaker::Approach | StationSpeaker::Departure)
}

/// The departure check-in's reply said "radar contact": the watch is
/// satisfied, and says nothing more.
fn radar_contact_at_check_in() {
    if let Some(w) = SQUAWK_WATCH
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_mut()
    {
        w.observed = true;
        w.check_in_owed = false;
    }
}

/// One tick of the radar watch: the actual transponder code, whether the
/// mode is replying (ON/ALT), and who COM1 is tuned to. Returns what to
/// speak and who says it.
///
/// "Radar contact" is the RADAR unit's, on its own frequency: tuned to the
/// tower the watch holds it (a tower does not relay it), and with a
/// departure handoff armed it is the check-in's reply, never the watch's.
/// LEMG 31 (sortie 1790584715, 2026-09-28): a departure heard "radar
/// contact" in the tower's voice 4.3 s after rotation (t 1790585265.4,
/// route squawk_watch), and again from Departure at the check-in. The
/// nudges ("confirm squawk") are still relayed: they are an instruction.
pub fn squawk_tick(
    actual: Option<i64>,
    mode_on: bool,
    tuned: StationSpeaker,
    now: Instant,
) -> Option<(String, StationSpeaker)> {
    let mut guard = SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner());
    let w = guard.as_mut()?;
    let first = *w.first_seen.get_or_insert(now);
    if actual == Some(w.code) && mode_on {
        if w.observed || !radar_unit(tuned) || w.check_in_owed {
            return None;
        }
        w.observed = true;
        return Some((format!("{}, radar contact.", w.callsign), w.speaker));
    }
    // Once contact was called, a later mode change isn't chased — the real
    // frequency stays quiet about it unless it matters (future: radar
    // contact lost on a long mismatch).
    if w.observed {
        return None;
    }
    if now.duration_since(first).as_secs_f64() < SQUAWK_GRACE_SECS
        || w.nudges >= SQUAWK_MAX_NUDGES
        || w.last_nudge
            .is_some_and(|t| now.duration_since(t).as_secs_f64() < SQUAWK_NUDGE_SECS)
    {
        return None;
    }
    w.last_nudge = Some(now);
    w.nudges += 1;
    // Two different problems, two different nudges: the code dialed right
    // with the transponder in standby got "confirm squawk 4503" (Gatwick,
    // 2026-07-13) — a nag that reads as "your digits are wrong" when the
    // digits are the one thing that's right.
    let recycle = actual == Some(w.code);
    *SQUAWK_NUDGE.lock().unwrap_or_else(|e| e.into_inner()) = Some(crate::party_line::SquawkNudge {
        code: w.code,
        recycle,
    });
    let line = if recycle {
        format!(
            "{}, negative radar contact, recycle transponder.",
            w.callsign
        )
    } else {
        format!(
            "{}, confirm squawk {}.",
            w.callsign,
            atis::spoken_digits(&w.code.to_string())
        )
    };
    Some((line, w.speaker))
}

/// Two-way contact plus the clearance picture, published by the reply path
/// after every answered call, for the monitor's compliance watch. `None`
/// until the pilot's first call to this station (and again after a swap) —
/// no contact means the tower doesn't know you, and the zone watch takes
/// over instead.
pub struct ClearanceSnap {
    pub state: tower::State,
    pub callsign: String,
    /// The runway END in use when this was published (e.g. "09R") — conflict
    /// watches restrict themselves to that strip; a parallel runway's
    /// traffic is somebody else's problem. Empty when no runway is known.
    pub runway_ident: String,
    /// The exact readback the pilot owes right now, phrased — the coach's
    /// first suggestion. `None` = no readback outstanding.
    pub pending_readback: Option<String>,
    /// The tower's book says OUR aircraft holds `runway_ident`: cleared onto
    /// it, or landed on it and not yet reported off ([`tower::Tower::holds_runway`]).
    /// The "runway vacated" report clears it, before the taxi request.
    pub holds_runway: bool,
}

/// A compliance or zone call: what to say, and whether the COPILOT says it
/// (not tuned to the station — a controller can't berate a radio that isn't
/// listening; the crew flags it instead, like a real FO would).
pub struct ComplianceCall {
    pub line: String,
    pub crew: bool,
}

/// The compliance watch's memory between ticks.
struct ComplianceState {
    prev_on_ground: Option<bool>,
    /// Continuously airborne since — a landing accusation needs a FLIGHT
    /// first. A one-sample unstick during the takeoff roll (0.3 ft AGL for
    /// one tick at 83 knots, Jerez 2026-07-11) read as air-then-ground and
    /// earned "you landed without clearance" mid-departure.
    airborne_since: Option<Instant>,
    /// Last violation call of any kind — one thing at a time on frequency.
    last_call: Option<Instant>,
    /// When the tower state was last seen at ClearedToLand — grace for the
    /// rollout and vacating after a legal landing.
    cleared_to_land_seen: Option<Instant>,
    /// Each transition violation is called once, not every tick.
    takeoff_berated: bool,
    landing_berated: bool,
    zone_last: Option<Instant>,
    /// Last off-route taxi challenge — its own cooldown; wandering is a
    /// nudge, not a violation.
    off_route_last: Option<Instant>,
    /// The strip (its `ident_a`) the aircraft stood on at the last ground
    /// tick, `None` off every runway, airborne or before the first tick.
    prev_strip: Option<String>,
    /// A RUNWAY START: the strip the aircraft first appeared on, stopped,
    /// without being seen to taxi onto it (the sim put it there). It is a
    /// lined-up aircraft awaiting its clearance, never an incursion, for as
    /// long as it stays on that strip (EGKK 26L, 2026-09-27).
    runway_start: Option<String>,
    /// The strip (its `ident_a`) the aircraft LANDED on, cleared or not,
    /// for as long as it stays on it: the rollout and the vacate are the only
    /// way off, never an incursion (EGKK-EGBB 15, sortie
    /// 1790601822-full_auto, 2026-09-28: landed uncleared, then "exit the
    /// runway" four times and "you are on runway one five without clearance"
    /// during the vacate the tower had just ordered with "taxi to parking").
    landed_strip: Option<String>,
    /// Our last LIFTOFF: the strip (its `ident_a`) we rolled on and when.
    /// The party line clears no departure off that strip behind us inside
    /// [`WAKE_SECS`] (EGKK 08R, 2026-09-28: "Easy eight six five three,
    /// cleared for takeoff" 1.6 s after our liftoff).
    liftoff: Option<(String, Instant)>,
}

impl ComplianceState {
    const NEW: ComplianceState = ComplianceState {
        prev_on_ground: None,
        airborne_since: None,
        last_call: None,
        cleared_to_land_seen: None,
        takeoff_berated: false,
        landing_berated: false,
        zone_last: None,
        off_route_last: None,
        prev_strip: None,
        runway_start: None,
        landed_strip: None,
        liftoff: None,
    };
}

static COMPLIANCE: Mutex<ComplianceState> = Mutex::new(ComplianceState::NEW);

/// Between repeat violation calls.
const VIOLATION_COOLDOWN_SECS: f64 = 60.0;
/// Rollout/vacating time after a landing clearance before "on the runway
/// without clearance" can fire.
const VACATE_GRACE_SECS: f64 = 120.0;
/// Below this a first sight of the aircraft on a strip is a standing start
/// (the sim's runway start reads 0.0 kt), never a taxi onto it.
const RUNWAY_START_MAX_KTS: f64 = 1.0;
/// The pseudo control zone around the active towered field: apt.dat has no
/// airspace geometry, so a cylinder stands in for the CTR.
const ZONE_RADIUS_NM: f64 = 5.0;
const ZONE_HEIGHT_FT: f64 = 3000.0;
const ZONE_COOLDOWN_SECS: f64 = 120.0;
/// Rolling out or vacating the strip it landed on, an aircraft at this pace or
/// more is on its way off (the recorded EGBB 15 vacate: 3.8 to 12 kt).
/// Stopped on it, it is squatting, cleared landing or not.
const LANDED_VACATING_KTS: f64 = 3.0;
/// Continuous air time before ground contact can be called a LANDING —
/// shorter is a skip on the takeoff roll or a bounce, not an arrival.
const MIN_FLIGHT_SECS: f64 = 30.0;

/// One tick of the compliance watch: the tower holds the pilot to the
/// clearances it actually issued, and challenges zone traffic it has never
/// talked to. Everything is derived from the published [`CLEARANCE`] state —
/// a tower that never saw you on its ground can't accuse you of an uncleared
/// takeoff.
pub fn compliance_tick(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    on_ground: bool,
    gs_kts: f64,
    tuned: bool,
    now: Instant,
) -> Option<ComplianceCall> {
    compliance_tick_facing(lat, lon, alt_ft, on_ground, gs_kts, tuned, now, None)
}

/// [`compliance_tick`] with the aircraft's true heading, so a runway it is
/// named on is the end it is pointing down ([`on_active_runway_facing`]).
#[allow(clippy::too_many_arguments)]
pub fn compliance_tick_facing(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    on_ground: bool,
    gs_kts: f64,
    tuned: bool,
    now: Instant,
    heading_true: Option<f64>,
) -> Option<ComplianceCall> {
    if station_unavailable() {
        return None;
    }
    let (field_lat, field_lon) = {
        let f = ACTIVE_FIELD.lock().unwrap_or_else(|e| e.into_inner());
        let (_, la, lo) = f.as_ref()?;
        (*la, *lo)
    };
    let dist_nm = crate::nav::haversine_nm(lat, lon, field_lat, field_lon);
    let clr: Option<(tower::State, String)> = CLEARANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.callsign.clone()));

    let mut w = COMPLIANCE.lock().unwrap_or_else(|e| e.into_inner());
    let prev_ground = w.prev_on_ground.replace(on_ground);
    // How long the flight before THIS ground contact lasted (0.0 while the
    // wheels never left) — read before the touchdown resets the clock.
    let airborne_secs = w
        .airborne_since
        .map_or(0.0, |t| now.duration_since(t).as_secs_f64());
    if on_ground {
        w.airborne_since = None;
        // Down: the landing-clearance coach is moot, and a stale handoff must
        // not nag on the next takeoff.
        *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    } else if w.airborne_since.is_none() {
        w.airborne_since = Some(now);
    }
    // A runway start: first seen on this strip, and stopped. A taxi onto a
    // runway arrives moving, a landing arrives airborne; only the sim's own
    // placement (boot, a reposition, the first tick of a fresh station)
    // shows a stopped aircraft on a strip it was not on a tick ago.
    let here = if on_ground { strip_under(lat, lon) } else { None };
    if here.is_none() || w.runway_start != here {
        w.runway_start = None;
    }
    if here.is_some() && here != w.prev_strip && gs_kts < RUNWAY_START_MAX_KTS {
        w.runway_start = here.clone();
    }
    // Our liftoff, off the strip we rolled on (see `liftoff`).
    if prev_ground == Some(true) && !on_ground {
        if let Some(strip) = w.prev_strip.clone() {
            w.liftoff = Some((strip, now));
        }
    }
    // A landing's strip is ours until we leave it (see `landed_strip`).
    if !on_ground {
        w.landed_strip = None;
    } else if prev_ground == Some(false) && airborne_secs >= MIN_FLIGHT_SECS {
        w.landed_strip = here.clone();
    } else if w.landed_strip.is_some() && here != w.landed_strip {
        w.landed_strip = None;
    }
    let landed_here =
        here.is_some() && w.landed_strip == here && gs_kts >= LANDED_VACATING_KTS;
    w.prev_strip = here;
    let runway_start = w.runway_start.is_some();
    if clr
        .as_ref()
        .is_some_and(|(s, _)| *s == tower::State::ClearedToLand)
    {
        w.cleared_to_land_seen = Some(now);
        // Cleared to land — the coach got what it was waiting for.
        *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }
    let vacating = w
        .cleared_to_land_seen
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < VACATE_GRACE_SECS);
    let quiet = w
        .last_call
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < VIOLATION_COOLDOWN_SECS);

    if let Some((state, callsign)) = &clr {
        // Landing without clearance: airborne last tick, on the active runway
        // now, and the tower never said "cleared to land". The classic ends
        // with a phone number, exactly once.
        if prev_ground == Some(false)
            && on_ground
            && airborne_secs >= MIN_FLIGHT_SECS
            && !w.landing_berated
            && *state != tower::State::ClearedToLand
        {
            if let Some(rwy) = on_active_runway_facing(lat, lon, heading_true) {
                w.landing_berated = true;
                w.last_call = Some(now);
                let rwy = atis::spoken_runway(&rwy);
                return Some(if tuned {
                    ComplianceCall {
                        line: format!(
                            "{callsign}, you landed on runway {rwy} without clearance. \
                             Taxi to parking. I have a number for you to copy when ready."
                        ),
                        crew: false,
                    }
                } else {
                    ComplianceCall {
                        line: "That was a landing without clearance. Expect a phone number."
                            .to_string(),
                        crew: true,
                    }
                });
            }
        }
        // Takeoff without clearance: ground last tick, airborne now, near the
        // field, and the tower never cleared it. Once.
        if prev_ground == Some(true)
            && !on_ground
            && dist_nm < 5.0
            && !w.takeoff_berated
            && !matches!(
                state,
                tower::State::TakeoffCleared | tower::State::ClearedToLand
            )
        {
            w.takeoff_berated = true;
            w.last_call = Some(now);
            return Some(if tuned {
                ComplianceCall {
                    line: format!("{callsign}, you were not cleared for takeoff. Say intentions."),
                    crew: false,
                }
            } else {
                ComplianceCall {
                    line: "We just took off without a clearance.".to_string(),
                    crew: true,
                }
            });
        }
        // Off the cleared taxi route: taxiing (moving, on the ground, taxi
        // clearance current), a route was issued, and the aircraft has
        // wandered off its polyline. A nudge with the via line, on its own
        // cooldown — controllers redirect strays, they don't phone them in.
        if on_ground
            && (5.0..40.0).contains(&gs_kts)
            && *state == tower::State::TaxiCleared
            && !w
                .off_route_last
                .is_some_and(|t| now.duration_since(t).as_secs_f64() < 45.0)
            && on_active_runway(lat, lon).is_none()
        {
            let stray = TAXI_ROUTE
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .as_ref()
                .and_then(|r| {
                    let d = dist_to_polyline_m(&r.path, lat, lon)?;
                    (d > 60.0).then(|| (r.via.clone(), r.path.last().copied()))
                });
            if let Some((via, goal)) = stray {
                w.off_route_last = Some(now);
                if !tuned {
                    return Some(ComplianceCall {
                        line: "We're off the cleared taxi route.".to_string(),
                        crew: true,
                    });
                }
                // A stray gets redirected from PRESENT POSITION: replan to
                // the same holding point and re-clear, instead of repeating
                // a via line for pavement the pilot left behind (the same
                // stale via spoken seven times across one Malaga taxi while
                // the pilot was 680 m off it, 2026-07-13). The republished
                // route also re-aims the taxi guide.
                let fresh = goal.and_then(|(glat, glon)| {
                    let net = TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
                    // The goal is the standing route's (already-trimmed)
                    // endpoint, never a runway junction — the standoff is
                    // moot, so the default rides.
                    net.plan(lat, lon, glat, glon, nav::HOLD_SHORT_STANDOFF_M, &[])
                        .filter(|r| r.path.len() >= 2)
                });
                let line = if let Some(r) = fresh {
                    let via = spoken_via(&r.via).join(", ");
                    let hp = spoken_taxiway(&r.holding_point);
                    *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = Some(TaxiWatch {
                        path: r.path.clone(),
                        via: via.clone(),
                        seg: r.seg.clone(),
                        holding_point: r.holding_point.clone(),
                        lead_on: r.lead_on.clone(),
                    });
                    format!(
                        "{callsign}, you appear to be off the cleared taxi route. \
                         Taxi to holding point {hp} via {via}."
                    )
                } else if via.is_empty() {
                    format!(
                        "{callsign}, you appear to be off the cleared taxi route. \
                         Continue with caution to the holding point."
                    )
                } else {
                    format!(
                        "{callsign}, you appear to be off the cleared taxi route. \
                         Taxi via {via}."
                    )
                };
                return Some(ComplianceCall { line, crew: false });
            }
        }
        // A cleared ARRIVAL taxi-in to a stand authorises being on the runway
        // just landed on and following the published route to parking — the
        // taxi-in sets the FSM to `Parked` (not `TaxiCleared`) and ends the
        // vacate grace early, so without this the berate fired ~2 min after
        // "taxi to stand" at a crew still clearing the landing runway on its own
        // cleared route. It's the arrival mirror of the departure `TaxiCleared`
        // exemption below; off-route deviations are still caught by the
        // route-deviation check above.
        // The flag only counts WITH the route it was published alongside,
        // matching the crew-side consumers: the exemption below silences the
        // incursion berate outright, and a bare flag that survived a swap or
        // a route-less clearance would switch that safety net off with
        // nothing to show the occupancy is part of a cleared drive.
        let arrival_taxi = *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner())
            && TAXI_ROUTE
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .is_some();
        // On the active runway without any clearance to be there: not rolling
        // (a takeoff/landing roll is covered above), not vacating after a
        // legal landing, and the state says holding-point at best. Repeats on
        // the violation cooldown while it persists — runway occupancy is the
        // one the tower keeps shouting about.
        if on_ground
            && gs_kts < 30.0
            && !vacating
            && !quiet
            && !runway_start
            && !landed_here
            && !matches!(
                state,
                // LinedUp is the tower's OWN instruction to be on the runway.
                tower::State::LinedUp | tower::State::TakeoffCleared | tower::State::ClearedToLand
            )
        {
            if let Some(rwy) = on_active_runway_facing(lat, lon, heading_true) {
                // The tower's OWN taxi route may cross (or backtrack) a
                // runway — apt.dat networks do it all the time, and the
                // clearance speaks the crossing ("cross runway two seven").
                // On a runway-flagged leg of the cleared route the occupancy
                // is LEGAL: silence while it keeps moving, and a stopped
                // crossing earns "expedite" — never the berate (the tower
                // shouted "you are on runway two seven without clearance" at
                // a pilot following its own route, 2026-07-14). Holds for a
                // DEPARTURE (state TaxiCleared) and an ARRIVAL taxi-in alike.
                let cleared_taxi = *state == tower::State::TaxiCleared || arrival_taxi;
                if cleared_taxi && on_cleared_runway_crossing(lat, lon) {
                    if gs_kts < 3.0 {
                        w.last_call = Some(now);
                        // Crossing, the heading is square to the strip and
                        // names neither end: the nearer one it is.
                        let rwy = atis::spoken_runway(&on_active_runway(lat, lon).unwrap_or(rwy));
                        return Some(if tuned {
                            ComplianceCall {
                                line: format!("{callsign}, expedite crossing runway {rwy}."),
                                crew: false,
                            }
                        } else {
                            ComplianceCall {
                                line: format!(
                                    "We've stopped on runway {rwy}. Keep it moving across."
                                ),
                                crew: true,
                            }
                        });
                    }
                    return None;
                }
                // An arrival vacating the runway it just landed on is NOT on a
                // route "crossing" (the runway is the start of the taxi-in, not a
                // leg to cross), so the check above misses it — but vacating under
                // a taxi-in clearance can never be an incursion. Stay silent.
                if arrival_taxi {
                    return None;
                }
                w.last_call = Some(now);
                let rwy = atis::spoken_runway(&rwy);
                return Some(if tuned {
                    ComplianceCall {
                        line: format!(
                            "{callsign}, you are on runway {rwy} without clearance. \
                             Exit the runway immediately."
                        ),
                        crew: false,
                    }
                } else {
                    ComplianceCall {
                        line: format!("We're on runway {rwy} without clearance. We should exit."),
                        crew: true,
                    }
                });
            }
        }
        return None;
    }

    // No two-way contact at all: the zone watch. Inside the pseudo-CTR
    // (cylinder: 5 nm, surface to 3,000 ft above the field), airborne, and
    // the tower has never heard from you — a real controller challenges,
    // a real FO squirms.
    if !on_ground && dist_nm < ZONE_RADIUS_NM {
        let (zone_name, elev_ft, tower_khz) = {
            let p = ACTIVE_PANEL.lock().unwrap_or_else(|e| e.into_inner());
            let (name, elev, svcs) = p.as_ref()?;
            let khz = svcs.iter().find(|(w, _)| w == "tower").map(|(_, k)| *k);
            (spoken_airport(name).to_string(), *elev, khz)
        };
        if alt_ft < elev_ft + ZONE_HEIGHT_FT
            && !w
                .zone_last
                .is_some_and(|t| now.duration_since(t).as_secs_f64() < ZONE_COOLDOWN_SECS)
        {
            w.zone_last = Some(now);
            return Some(if tuned {
                // The crew hears the question as data too: an inbound call is
                // the answer when this is the destination's tower.
                *ZONE_CHALLENGE.lock().unwrap_or_else(|e| e.into_inner()) = true;
                ComplianceCall {
                    line: format!(
                        "Aircraft in the {zone_name} control zone, you are inside \
                         controlled airspace without clearance. Say callsign and intentions."
                    ),
                    crew: false,
                }
            } else {
                ComplianceCall {
                    line: match tower_khz {
                        Some(k) => format!(
                            "We're inside the {zone_name} zone without clearance. \
                             Tower is on {}.",
                            atis::spoken_mhz(k)
                        ),
                        None => format!("We're inside the {zone_name} zone without clearance."),
                    },
                    crew: true,
                }
            });
        }
    }
    None
}

/// The tower's after-takeoff handoff, armed by a takeoff clearance and spoken
/// once airborne — see [`Station::arm_departure_handoff`] and [`handoff_tick`].
pub struct Handoff {
    /// The full transmission, prebuilt ("… contact Gibraltar Departure on …").
    pub line: String,
    /// Speak once climbing through this altitude (ft MSL, field + 700).
    pub min_alt_ft: f64,
    /// The station handed to, as spoken ("Gibraltar Departure"), and its
    /// frequency — published for the crew the moment the line goes out.
    pub to: String,
    pub khz: i64,
}

static HANDOFF: Mutex<Option<Handoff>> = Mutex::new(None);

/// One tick of the after-takeoff handoff, ~monitor rate: silence until the
/// flight is airborne and through the arming altitude, then the handoff
/// exactly once.
pub fn handoff_tick(alt_ft: f64, on_ground: bool) -> Option<String> {
    let mut guard = HANDOFF.lock().unwrap_or_else(|e| e.into_inner());
    let plan = guard.as_ref()?;
    if on_ground || alt_ft < plan.min_alt_ft {
        return None;
    }
    let line = plan.line.clone();
    // The crew that works the radio gets the instruction as data, not just
    // as audio: a readback, a retune and a check-in are owed, and until this
    // was published nothing on the crew side could do them — the Malaga
    // departure stayed on tower for thirteen minutes (12:23Z, 2026-09-11).
    *DEPARTURE_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) =
        Some((plan.to.clone(), plan.khz));
    *guard = None;
    Some(line)
}

/// The in-process ATIS: one broadcast state per station, driven by the
/// monitor's tick while COM1 sits on the ATIS frequency. Editions cut and the
/// letter advances exactly as the standalone binary did (`atis::new_edition`
/// hysteresis included); the report always reads the FROZEN edition, so the
/// numbers can't drift mid-cycle.
pub struct AtisState {
    pub khz: i64,
    airport: String,
    runways: Vec<nav::Runway>,
    letter: char,
    edition: Option<atis::Snapshot>,
    last_spoken: Option<Instant>,
}

static ATIS: Mutex<Option<AtisState>> = Mutex::new(None);

/// Re-broadcast cadence while the pilot stays tuned — a real ATIS loops.
const ATIS_REPEAT_SECS: f64 = 45.0;

/// The live weather one ATIS cycle is cut from (read by the monitor).
#[derive(Debug, Clone, Copy)]
pub struct AtisWx {
    pub zulu_sec: f64,
    pub wind_from_deg: f64,
    pub wind_kts: f64,
    pub visibility_sm: f64,
    pub qnh_hpa: f64,
    pub oat_c: f64,
}

/// The monitor's cheap gate: true when an ATIS exists, COM1 is on it, AND a
/// broadcast is due — only then is the full weather set worth reading (six
/// REST GETs; reading them every tick while parked on the frequency was
/// measurable main-thread load on the sim). Off frequency it also rewinds the
/// loop so a retune starts the broadcast from the top.
pub fn atis_due(com1_khz: i64, now: Instant) -> bool {
    let mut guard = ATIS.lock().unwrap_or_else(|e| e.into_inner());
    let Some(st) = guard.as_mut() else {
        return false;
    };
    if (com1_khz - st.khz).abs() > 5 {
        st.last_spoken = None;
        return false;
    }
    st.last_spoken
        .is_none_or(|t| now.duration_since(t).as_secs_f64() >= ATIS_REPEAT_SECS)
}

/// Follow COM1 to any in-range field's ATIS. The ATC tab lets the pilot tune
/// every listed frequency, but the broadcast only existed for the ACTIVE
/// station — tuning another field's ATIS was dead air (2026-07-10). Swapping
/// fields restarts that field's loop at information Alpha.
pub fn atis_retune(com1_khz: i64, lat: f64, lon: f64, alt_msl_ft: f64) {
    {
        let cur = ATIS.lock().unwrap_or_else(|e| e.into_inner());
        if cur.as_ref().is_some_and(|a| (a.khz - com1_khz).abs() <= 5) {
            return; // already broadcasting on this frequency
        }
    }
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    let hit = region.iter().find(|a| {
        a.apt
            .freq(nav::FREQ_ATIS)
            .is_some_and(|f| (f.khz - com1_khz).abs() <= 5)
            && crate::nav::haversine_nm(lat, lon, a.lat, a.lon)
                <= vhf_range_nm(alt_msl_ft - a.apt.elev_ft)
    });
    if let Some(a) = hit {
        let khz = a
            .apt
            .freq(nav::FREQ_ATIS)
            .map(|f| f.khz)
            .unwrap_or(com1_khz);
        tracing::info!(
            airport = %a.apt.name,
            mhz = khz as f64 / 1000.0,
            "ATIS following COM1 to another field"
        );
        *ATIS.lock().unwrap_or_else(|e| e.into_inner()) = Some(AtisState {
            khz,
            airport: spoken_airport(&a.apt.name).to_string(),
            runways: a.apt.runways.clone(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
    }
}

/// One tick of the ATIS broadcast: the full report when tuned and due (first
/// tune-in speaks immediately, then the loop repeats), silence otherwise.
/// Cuts a new edition — advancing the letter — when the weather has genuinely
/// moved (see `atis::new_edition`).
pub fn atis_tick(com1_khz: i64, wx: &AtisWx, now: Instant) -> Option<String> {
    if station_unavailable() {
        return None;
    }
    let mut guard = ATIS.lock().unwrap_or_else(|e| e.into_inner());
    let st = guard.as_mut()?;
    if (com1_khz - st.khz).abs() > 5 {
        st.last_spoken = None; // retuning restarts the loop from the top
        return None;
    }
    if st
        .last_spoken
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < ATIS_REPEAT_SECS)
    {
        return None;
    }
    // Runway from the wind — but in calm keep the edition's pick, or the
    // coin-toss flip would cut a fresh edition per broadcast.
    let runway = if wx.wind_kts < CALM_WIND_KTS {
        st.edition.as_ref().map(|e| e.runway.clone())
    } else {
        None
    }
    .or_else(|| {
        nav::best_runway_for_wind(&st.runways, wx.wind_from_deg, wx.wind_kts).map(|(id, _, _)| id)
    })
    .unwrap_or_default();
    let snap = atis::Snapshot {
        airport: st.airport.clone(),
        zulu_sec: wx.zulu_sec,
        runway,
        wind_from_deg: wx.wind_from_deg,
        wind_kts: wx.wind_kts,
        visibility_sm: wx.visibility_sm,
        qnh_hpa: wx.qnh_hpa,
        oat_c: wx.oat_c,
    };
    match &st.edition {
        None => st.edition = Some(snap),
        Some(e) if atis::new_edition(e, &snap) => {
            st.letter = atis::next_letter(st.letter);
            st.edition = Some(snap);
        }
        // Same edition: time still moves in the report (a frozen clock reads
        // as a dead station), everything else stays the edition's numbers.
        Some(_) => {
            if let Some(e) = st.edition.as_mut() {
                e.zulu_sec = wx.zulu_sec;
            }
        }
    }
    st.last_spoken = Some(now);
    Some(atis::report(st.edition.as_ref()?, st.letter))
}

/// The information letter currently on the broadcast — `None` until the first
/// edition is cut (a pilot can't "have information" that never aired).
pub fn atis_current_letter() -> Option<char> {
    ATIS.lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|a| a.edition.is_some())
        .map(|a| a.letter)
}

/// The enroute region: every towered field from apt.dat, indexed once on a
/// background thread at startup, so the flight can outgrow its departure
/// airport. When another tower becomes decisively nearer, the current
/// controller hands the flight off by name and frequency, and the station
/// rebuilds itself around the new field — Gibraltar stops following you to
/// Malaga.
pub struct RegionApt {
    pub icao: String,
    pub apt: nav::AirportInfo,
    lat: f64,
    lon: f64,
}

static REGION: Mutex<Vec<RegionApt>> = Mutex::new(Vec::new());

/// Every land field in apt.dat, towered or not, as (ICAO, name, position):
/// the IFR clearance's destination directory beyond the region index
/// (issue #96). An IFR flight files to any field with a runway, and the
/// region index holds only staffed towers, so "IFR clearance to" a
/// tower-less field (or one whose tower row is a Gateway placeholder) was
/// "say again destination". Built by the same scan as [`REGION`]; codes
/// only (a name across forty thousand fields is a guess).
type FieldDirectoryEntry = (String, String, (f64, f64));
static FIELD_DIR: Mutex<Vec<FieldDirectoryEntry>> = Mutex::new(Vec::new());

/// The active station's identity + position, for the switch decision. Set by
/// [`Station::from_airport`].
static ACTIVE_FIELD: Mutex<Option<(String, f64, f64)>> = Mutex::new(None);

static LAST_REGION_SWITCH: Mutex<Option<Instant>> = Mutex::new(None);

/// The new field must be this much nearer than the active one before a
/// handoff — hysteresis, so the midpoint between two fields can't flap.
const SWITCH_MARGIN_NM: f64 = 5.0;
/// The ground relocation's margin: the field whose pavement is nearer, by a
/// fraction of a mile, is the field you are at.
const GROUND_MARGIN_NM: f64 = 0.2;
/// No handoff to a field further than this — enroute over nowhere keeps the
/// last controller (a lightweight "radar" continuity).
const SWITCH_MAX_NM: f64 = 60.0;
/// Minimum quiet time between handoffs.
const SWITCH_COOLDOWN_SECS: f64 = 180.0;

/// Index every towered field once (a full apt.dat pass, seconds — run it on a
/// background thread). Without it [`region_tick`] never fires and the station
/// stays single-airport, exactly the pre-region behavior.
/// Whether the enroute region index has been loaded — the owner checks this
/// before spawning another full apt.dat scan (a live mode re-enable must not
/// pay the boot cost twice).
pub fn region_loaded() -> bool {
    !REGION.lock().unwrap_or_else(|e| e.into_inner()).is_empty()
}

pub fn load_region(apt_path: &str) {
    let mut v: Vec<RegionApt> = Vec::new();
    let mut dir: Vec<(String, String, (f64, f64))> = Vec::new();
    let res = nav::scan_airports(apt_path, |icao, apt| {
        if let Some(pos) = apt.position() {
            dir.push((icao.to_string(), apt.name.clone(), pos));
        }
        // Staffed towers only: a Gateway "unnamed entity" TWR row (an
        // ultralight strip's air-to-air frequency, mistyped) must not become
        // a handoff target or "the nearest station" (LEMY, 2026-07-10).
        if apt.staffed_tower().is_some() {
            if let Some((lat, lon)) = apt.position() {
                v.push(RegionApt {
                    icao: icao.to_string(),
                    apt,
                    lat,
                    lon,
                });
            }
        }
    });
    match res {
        Ok(()) => {
            tracing::info!(towered_fields = v.len(), fields = dir.len(), "enroute region indexed");
            *REGION.lock().unwrap_or_else(|e| e.into_inner()) = v;
            *FIELD_DIR.lock().unwrap_or_else(|e| e.into_inner()) = dir;
        }
        Err(e) => tracing::warn!("region index failed ({e:#}); staying single-airport"),
    }
}

/// A due station switch: what to say and who says it. Airborne it's a real
/// controller handoff; on the ground it's the CREW noting the local tower
/// (you didn't fly here under radar — you loaded the flight at a new field,
/// and "radar services terminated" from a parked aircraft would be theatre).
pub struct RegionSwitch {
    pub line: String,
    /// True = the copilot's voice (ground relocation); false = the controller.
    pub crew: bool,
}

/// You're AT a field (parked/taxiing) when it's within this — the ground
/// relocation case ("I started at LERL and Gibraltar answered", 2026-07-09).
const AT_FIELD_NM: f64 = 4.0;

/// One tick of the enroute watch: when another towered field is decisively
/// nearer than the active one, arm the station swap and return what to speak.
/// Airborne: the handoff ("radar services terminated, contact Malaga Tower on
/// ..."). On the ground: a silent relocation with a crew note, but only when
/// physically AT the new field. The main loop consumes [`PENDING_STATION`] on
/// its next pass; a cooldown keeps back-to-back fields from ping-ponging.
/// How much STATION a field is, for the enroute handoff choice: its distinct
/// published services beyond the tower itself. A primary airport scores 3-5;
/// a strip whose apt.dat rows just point at the primary's controllers scores
/// 0-1, so it never wins the sky it doesn't control.
fn region_services(apt: &nav::AirportInfo) -> usize {
    [
        nav::FREQ_ATIS,
        nav::FREQ_CLEARANCE,
        nav::FREQ_GROUND,
        nav::FREQ_APPROACH,
        nav::FREQ_DEPARTURE,
    ]
    .iter()
    .filter(|k| apt.freq(**k).is_some())
    .count()
}

/// Nautical miles of pull each published service adds to a field in the
/// airborne handoff choice (see [`region_services`]).
const SERVICE_BIAS_NM: f64 = 3.0;

pub fn region_tick(lat: f64, lon: f64, on_ground: bool, now: Instant) -> Option<RegionSwitch> {
    if uncontrolled_arrival() {
        return None;
    }
    if PENDING_STATION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some()
    {
        return None; // a switch is already waiting to be consumed
    }
    if LAST_REGION_SWITCH
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < SWITCH_COOLDOWN_SECS)
    {
        return None;
    }
    let (active_name, a_lat, a_lon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    // Airborne, "nearest" is service-weighted: a real airport (ATIS, ground,
    // its own approach) outranks a strip a few miles nearer that merely
    // BORROWED the big field's frequencies — Trebujena's apt.dat rows name
    // "Jerez Tower"/"Seville Approach", and picking it by raw distance had
    // approach faithfully vectoring the Jerez arrival onto Trebujena's 800 m
    // grass runway two six (flown 2026-07-11). On the ground, raw nearest by
    // FOOTPRINT (the nearest runway centreline, not the first threshold): a
    // relocation goes to the field whose pavement you are physically on,
    // however small. Measured from the first runway row, Anchorage's GA ramp
    // was "nearer" Lake Hood Strip than Anchorage itself, and the whole
    // sortie (2026-09-10) taxied 1.4 km across Anchorage's aprons on a
    // beeline to Lake Hood's network before the drawn route began.
    let distance = |a: &RegionApt| -> f64 {
        if on_ground {
            a.apt
                .ground_distance_nm(lat, lon)
                .unwrap_or_else(|| crate::nav::haversine_nm(lat, lon, a.lat, a.lon))
        } else {
            crate::nav::haversine_nm(lat, lon, a.lat, a.lon)
        }
    };
    let score = |d: f64, apt: &nav::AirportInfo| {
        if on_ground {
            d
        } else {
            d - SERVICE_BIAS_NM * region_services(apt) as f64
        }
    };
    // An IFR flight in the air is worked by radar units, never handed to a
    // tower that only owns its zone: EGBB-EGKK (2026-09-27) was told
    // "contact Oxford Kidlington Tower" at FL150. It goes to a field's
    // approach (or departure) position, and a tower-only field is passed
    // over, save the active one and the pinned arrival (the flight is
    // landing there; its tower is where the inbound call goes).
    let ifr = IFR_FLIGHT.lock().unwrap_or_else(|e| e.into_inner()).clone();
    let ifr_airborne = !on_ground && ifr.is_some();
    let pin = arrival_pin_khz();
    let eligible = |a: &RegionApt| {
        !ifr_airborne
            || a.apt.name == active_name
            || radar_position(&a.apt).is_some()
            || a.apt.freq(nav::FREQ_TOWER).is_some_and(|f| Some(f.khz) == pin)
    };
    let (d_best, s_best, best) = region
        .iter()
        .filter(|a| eligible(a))
        .map(|a| {
            let d = distance(a);
            (d, score(d, &a.apt), a)
        })
        .min_by(|x, y| x.1.total_cmp(&y.1))?;
    if best.apt.name == active_name {
        return None;
    }
    // Ground: only a relocation TO a field you're actually at. Airborne: the
    // enroute handoff, bounded so cruising over nowhere keeps the last
    // controller.
    if (on_ground && d_best > AT_FIELD_NM) || (!on_ground && d_best > SWITCH_MAX_NM) {
        return None;
    }
    // Hysteresis on the same scale the pick was made on. Adjacent fields
    // (Anchorage and Lake Hood share a fence) are decided by a fraction of a
    // mile on the ground, so the enroute margin would pin the pilot to
    // whichever answered first; the cooldown and a parked aircraft's fixed
    // geometry keep it from ping-ponging.
    let s_active = region
        .iter()
        .find(|a| a.apt.name == active_name)
        .map(|a| score(distance(a), &a.apt))
        .unwrap_or_else(|| crate::nav::haversine_nm(lat, lon, a_lat, a_lon));
    let margin = if on_ground {
        GROUND_MARGIN_NM
    } else {
        SWITCH_MARGIN_NM
    };
    if s_best + margin > s_active {
        return None;
    }
    // A committed arrival owns the station (see [`pin_arrival_station`]): the
    // only swap the region may still make is the one TO the pinned field.
    // On the GROUND the pavement under the wheels outranks the pin — we
    // landed somewhere else, or the arrival is over and nobody released it —
    // so the relocation is taken and the pin dropped, never stranding the
    // station at a field we have left.
    if let Some(pin) = arrival_pin_khz() {
        if !best.apt.freq(nav::FREQ_TOWER).is_some_and(|f| f.khz == pin) {
            if !on_ground {
                return None;
            }
            *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
    }
    let khz = best.apt.freq(nav::FREQ_TOWER)?.khz;
    // Whether ANY controller of the active station has actually worked this
    // flight in this session: two-way contact publishes a clearance state,
    // a radar service arms the squawk watch. Without either there is no
    // service to hand off — a fresh mid-flight boot must not open with a
    // controller terminating a service it never provided (heard live
    // 2026-07-11: app start near Sevilla answered by "radar services
    // terminated" from a departure position never called).
    let had_contact = CLEARANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some()
        || SQUAWK_WATCH
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_some();
    let sw = if on_ground {
        RegionSwitch {
            line: format!(
                "We're at {}, tower here is on {}.",
                spoken_airport(&best.apt.name),
                atis::spoken_mhz(khz)
            ),
            crew: true,
        }
    } else if !had_contact {
        // First fix of the session: the copilot orients, no radar theatre.
        // Also every switch AFTER an airborne handoff: `Station::install`
        // clears the clearance and the squawk watch, so the new station has
        // "never worked" us (EGBB-EGKK 2026-09-27: four of these after the
        // Oxford handoff). The crew speaks the note only when it is worth
        // volunteering (`monitor::region_note_said`).
        RegionSwitch {
            line: format!(
                "Closest field is {}, tower is on {}.",
                spoken_airport(&best.apt.name),
                atis::spoken_mhz(khz)
            ),
            crew: true,
        }
    } else {
        // An IFR flight is handed unit to unit with its code: no service
        // ends and no squawk is ordered (LIEA-LEBL, 2026-09-27, an IFR 737
        // on 4501 was told "squawk seven thousand" at FL200).
        // A discrete code in effect goes back to conspicuity with the service
        // that assigned it.
        let squawk_vfr = ifr.is_none()
            && SQUAWK_WATCH
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .is_some();
        // Enroute, the IFR flight goes to the field's radar position (see
        // `eligible` above); the pinned arrival keeps its tower, where the
        // crew's inbound call goes.
        let pinned = best.apt.freq(nav::FREQ_TOWER).is_some_and(|f| Some(f.khz) == pin);
        let (position, khz) = match radar_position(&best.apt) {
            Some(r) if ifr_airborne && !pinned => r,
            _ => ("Tower", khz),
        };
        let to = format!("{} {position}", spoken_airport(&best.apt.name));
        // The instruction as data for the crew that has to obey it: the
        // spoken line alone had no consumer, so the flight kept the discrete
        // code and the old frequency until its own arrival stage retuned.
        *REGION_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = Some(RegionHandoff {
            to: to.clone(),
            khz,
            squawk_vfr,
            ifr_code: ifr.as_ref().map(|f| f.code),
        });
        RegionSwitch {
            line: region_handoff_line(ifr.as_ref().map(|f| f.callsign.as_str()), squawk_vfr, &to, khz),
            crew: false,
        }
    };
    tracing::info!(from = %active_name, to = %best.apt.name, ground = on_ground, "region switch");
    *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) =
        Some((best.icao.clone(), best.apt.clone()));
    *LAST_REGION_SWITCH.lock().unwrap_or_else(|e| e.into_inner()) = Some(now);
    Some(sw)
}

/// A field's radar position for an IFR handoff: its approach, else its
/// departure frequency, as spoken ("Approach") and in kHz. `None` = the field
/// has only a tower.
fn radar_position(apt: &nav::AirportInfo) -> Option<(&'static str, i64)> {
    apt.freq(nav::FREQ_APPROACH)
        .map(|f| ("Approach", f.khz))
        .or_else(|| apt.freq(nav::FREQ_DEPARTURE).map(|f| ("Departure", f.khz)))
}

/// The enroute handoff as spoken. VFR: the radar service ends ("Radar
/// services terminated", seven thousand when a code was assigned). IFR
/// (`ifr_callsign`): a plain handoff to the next unit, the code kept, the
/// same "contact X on Y" the station's other handoffs use in both dialects.
pub(crate) fn region_handoff_line(
    ifr_callsign: Option<&str>,
    squawk_vfr: bool,
    to: &str,
    khz: i64,
) -> String {
    let mhz = atis::spoken_mhz(khz);
    match ifr_callsign {
        Some(cs) if !cs.is_empty() => format!("{cs}, contact {to} on {mhz}."),
        Some(_) => format!("Contact {to} on {mhz}."),
        None => {
            let squawk = if squawk_vfr {
                " Squawk seven thousand."
            } else {
                ""
            };
            format!("Radar services terminated.{squawk} Contact {to} on {mhz}.")
        }
    }
}

/// The active station's controller frequencies (kHz → speaking position),
/// published by [`Station::install`] for the monitor's traffic advisories.
static CONTROLLER_FREQS: Mutex<Vec<(i64, StationSpeaker)>> = Mutex::new(Vec::new());

/// The active tower's frequency (kHz), for the crew side — the full-auto
/// orchestrator tunes COM1 to it before requesting taxi. `None` when no station
/// is installed.
pub(crate) fn active_tower_khz() -> Option<i64> {
    CONTROLLER_FREQS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|(_, s)| matches!(s, StationSpeaker::Tower))
        .map(|(k, _)| *k)
}

/// The active station's panel identity — name, field elevation, published
/// services — for the radio row and the ATC tab. Published by
/// [`Station::install`], like the rest of the side state.
type ActivePanel = (String, f64, Vec<(String, i64)>);
static ACTIVE_PANEL: Mutex<Option<ActivePanel>> = Mutex::new(None);

/// A ground station's nominal antenna height. VHF range is line of sight, so
/// even on the ramp the mast buys a few miles.
const ANTENNA_FT: f64 = 50.0;

/// VHF radio range: the line-of-sight horizon for a receiver `h` feet above
/// the station's elevation, plus the station's own mast. Terrain masking is
/// ignored — this drives what the panel *shows*, a planning aid, not a
/// propagation model.
pub fn vhf_range_nm(height_above_field_ft: f64) -> f64 {
    1.23 * (height_above_field_ft.max(0.0).sqrt() + ANTENNA_FT.sqrt())
}

/// One field inside VHF range, for the panel's ATC tab and the crew's
/// "nearest station" note.
pub struct RangedField {
    pub name: String,
    pub dist_nm: f64,
    /// This is the active station — the one that answers radio calls.
    pub active: bool,
    /// Published services in briefing order, from [`services`].
    pub services: Vec<(String, i64)>,
}

/// Every towered field within VHF line of sight of the aircraft, nearest
/// first, capped at `limit`. Reads the region index; with the region off (or
/// not indexed yet) it falls back to the active station alone, so the list
/// never knows less than the station does.
pub fn fields_in_range(lat: f64, lon: f64, alt_msl_ft: f64, limit: usize) -> Vec<RangedField> {
    let active_name = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(name, _, _)| name.clone());
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    let mut v: Vec<RangedField> = region
        .iter()
        .filter_map(|a| {
            let d = crate::nav::haversine_nm(lat, lon, a.lat, a.lon);
            (d <= vhf_range_nm(alt_msl_ft - a.apt.elev_ft)).then(|| RangedField {
                name: a.apt.name.clone(),
                dist_nm: d,
                active: active_name.as_deref() == Some(a.apt.name.as_str()),
                services: services(&a.apt),
            })
        })
        .collect();
    drop(region);
    if v.is_empty() {
        if let Some((name, svcs)) = active_station_in_range(lat, lon, alt_msl_ft) {
            let (_, alat, alon) = ACTIVE_FIELD
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
                .unwrap_or_default();
            v.push(RangedField {
                name,
                dist_nm: crate::nav::haversine_nm(lat, lon, alat, alon),
                active: true,
                services: svcs,
            });
        }
    }
    v.sort_by(|x, y| x.dist_nm.total_cmp(&y.dist_nm));
    v.truncate(limit);
    v
}

/// You call it, you get it: when a keyed transmission goes out on a frequency
/// the active station does NOT cover, but which belongs to another in-range
/// field, arm the station swap so THAT field answers the call about to be
/// routed. The ATC tab invites the pilot to tune any in-range station;
/// without this, calling Badajoz on its real tower frequency was answered by
/// dead air and a crew note about Gibraltar (2026-07-10). Unicom is excluded
/// (a shared party line, not a facility), and the nearest match wins when
/// fields share a frequency. Stamps the region cooldown so the automatic
/// handoff can't immediately yank the station away from an explicit tune.
/// Commit the station to the field we are arriving at, by its TOWER
/// frequency: while this is set the enroute region may hand the station TO
/// that field and never away from it, and no other field may take it.
///
/// The region pick is service-weighted DISTANCE, which is the right rule for
/// crossing a country and the wrong one for the last fifteen miles. Inbound
/// to Heathrow with the inbound call already made and answered, the tick
/// found London City eight miles off the nose against Heathrow's fifteen and
/// took the station; the final report and then the taxi-in request were
/// transmitted to a tower that was no longer listening, both `void`, and the
/// 737 landed on 27L with no clearance and sat on it for three minutes with
/// traffic inside a mile (LFLL-EGLL, 2026-09-17). `None` releases it.
pub fn pin_arrival_station(tower_khz: Option<i64>) {
    *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner()) = tower_khz;
}

/// The filed destination has no staffed tower in the installed airport data.
/// The last radar unit remains installed for the region, but it no longer
/// works this flight: it must not answer the destination's inbound/final calls
/// or make its clearances look applicable at another field.
static UNCONTROLLED_ARRIVAL: AtomicBool = AtomicBool::new(false);

/// End radar service for an arrival to a field with no staffed tower.
/// Idempotent: the enroute/arrival seam may be retried, but the retirement is
/// one transition and never an invented station for the destination.
pub fn begin_uncontrolled_arrival(destination: &str) {
    if UNCONTROLLED_ARRIVAL.swap(true, Ordering::SeqCst) {
        return;
    }
    *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *REGION_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *DEPARTURE_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *ASSIGNED_SQUAWK.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *IFR_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *IFR_FLIGHT.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *SQUAWK_NUDGE.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *LANDING_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *PENDING_LANDING_RELEASE
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = None;
    *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = false;
    *SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner()) = None;
    tracing::info!(%destination, "radar service ended for uncontrolled arrival");
    sim::blackbox::record(
        "atc",
        serde_json::json!({
            "event": "uncontrolled_arrival",
            "destination": destination,
            "why": "no_staffed_tower",
        }),
    );
}

pub(crate) fn uncontrolled_arrival() -> bool {
    UNCONTROLLED_ARRIVAL.load(Ordering::SeqCst)
}

/// Release the arrival-only service suppression. The next region tick may use
/// the installed station normally again; a new station install also clears it.
pub fn end_arrival() {
    UNCONTROLLED_ARRIVAL.store(false, Ordering::SeqCst);
}

/// The pinned arrival field's tower frequency, if the crew has committed to
/// one. Crate-internal: the crew sets the pin, the region reads it.
fn arrival_pin_khz() -> Option<i64> {
    *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner())
}

pub fn arm_station_for_freq(com1_khz: i64, lat: f64, lon: f64, alt_msl_ft: f64, now: Instant) {
    if PENDING_STATION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some()
    {
        return; // a switch is already waiting to be consumed
    }
    let active_name = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(name, _, _)| name.clone());
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    let hit = region
        .iter()
        .filter(|a| active_name.as_deref() != Some(a.apt.name.as_str()))
        .filter(|a| {
            services(&a.apt)
                .iter()
                .any(|(word, khz)| word != "unicom" && (khz - com1_khz).abs() <= 5)
        })
        .map(|a| (crate::nav::haversine_nm(lat, lon, a.lat, a.lon), a))
        .filter(|(d, a)| *d <= vhf_range_nm(alt_msl_ft - a.apt.elev_ft))
        .min_by(|x, y| x.0.total_cmp(&y.0));
    if let Some((d, a)) = hit {
        tracing::info!(
            to = %a.apt.name,
            khz = com1_khz,
            nm = format!("{d:.1}"),
            "station swap: transmission on another in-range field's frequency"
        );
        *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) =
            Some((a.icao.clone(), a.apt.clone()));
        *LAST_REGION_SWITCH.lock().unwrap_or_else(|e| e.into_inner()) = Some(now);
    }
}

/// A new flight has been set up and the station that knew the old one has
/// not yet been replaced by the main loop ([`retire_for_new_flight`] sets it,
/// `Station::install` and `Station::uninstall` clear it). While it stands the
/// old station's replies publish no clearance picture.
static NEW_FLIGHT_PENDING: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

pub(crate) fn new_flight_pending() -> bool {
    NEW_FLIGHT_PENDING.load(std::sync::atomic::Ordering::SeqCst)
}

/// The reply path's one write of the clearance picture.
fn publish_clearance(snap: ClearanceSnap) {
    *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = Some(snap);
}

/// THE FLIGHT THAT ENDED TAKES ITS CLEARANCES WITH IT. Called by the crew the
/// moment a new flight exists (the sim has repositioned us for "put us on a
/// ten mile final"), BEFORE anything flies it: the landing clearance, the
/// approach handoff, the vectors, the held release, the taxi route, the IFR
/// clearance and code and every pending flag on the party line belonged to the
/// last flight. LEMG, 2026-09-28 (sortie 1790584266-approach): the previous
/// case had been cleared to land on 12; the reposition to a 10 nm final for
/// 13 armed autoland 1.3 s later, it read "cleared to land 12" off the party
/// line before the crew's new final call went out, and turned the 737 toward
/// a centreline 3.6 km away. The station for the field is rebuilt by
/// [`arm_new_flight`] a few seconds later; until the main loop installs it the
/// old station, still answering, publishes nothing.
pub fn retire_for_new_flight() {
    NEW_FLIGHT_PENDING.store(true, std::sync::atomic::Ordering::SeqCst);
    UNCONTROLLED_ARRIVAL.store(false, Ordering::SeqCst);
    *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = false;
    *LINE_UP_CONDITION.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *DEPARTURE_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *ASSIGNED_SQUAWK.lock().unwrap_or_else(|e| e.into_inner()) = None;
    // The radar watch on that code, and its nudge, were the old flight's
    // too: a watch left armed says "radar contact" at the next liftoff.
    *SQUAWK_WATCH.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *SQUAWK_NUDGE.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *IFR_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *IFR_FLIGHT.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *PENDING_TAKEOFF_CLEARANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = false;
    *PENDING_LINE_UP.lock().unwrap_or_else(|e| e.into_inner()) = false;
    *PENDING_GO_AROUND.lock().unwrap_or_else(|e| e.into_inner()) = false;
    *PENDING_RESEQUENCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *RESEQUENCED_ORDER.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *LANDING_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *PENDING_LANDING_RELEASE
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = None;
    // A challenge raised for the old flight, the wind the tower worked it
    // with, and our track before the jump: a groundspeed derived across a
    // reposition judged the new final's go-around.
    *ZONE_CHALLENGE.lock().unwrap_or_else(|e| e.into_inner()) = false;
    *TOWER_WIND.lock().unwrap_or_else(|e| e.into_inner()) = None;
    OWN_FIXES.lock().unwrap_or_else(|e| e.into_inner()).clear();
}

/// A NEW FLIGHT at `icao` (the crew's "put us on a ten mile final", which
/// starts a flight through the sim's own initialisation): the station for that
/// field is rebuilt from nothing on the main loop's next pass, even when it is
/// the one already up. The tower's worksheet for us belongs to the flight that
/// ended: at LEMG (2026-09-26) the station that had watched the 737 sit on
/// runway 30 went on as if the arrival 10 nm out on 13 were the same aircraft,
/// and the landing earned "you landed without clearance". Overrides any swap
/// already waiting (the new flight wins), starts the region cooldown so the
/// nearest-field pick cannot move the station straight back, and drops the
/// old flight's arrival pin and region handoff (install() keeps both on
/// purpose, for a handoff in flight). Returns the tower's frequency, `None`
/// when the field has no staffed tower in the region index (nothing armed).
pub fn arm_new_flight(icao: &str, now: Instant) -> Option<i64> {
    let (code, apt, khz) = {
        let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
        let a = region.iter().find(|a| a.icao.eq_ignore_ascii_case(icao))?;
        let khz = a.apt.freq(nav::FREQ_TOWER)?.khz;
        (a.icao.clone(), a.apt.clone(), khz)
    };
    tracing::info!(%code, khz, "station rebuilt for a new flight");
    *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) = Some((code, apt));
    *LAST_REGION_SWITCH.lock().unwrap_or_else(|e| e.into_inner()) = Some(now);
    *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *REGION_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    Some(khz)
}

/// A new flight airborne is worked by the nearest towered field within this
/// (a setup's final is at most 30 nm out).
const NEW_FLIGHT_AIR_NM: f64 = 30.0;

/// The towered field a new flight at this position belongs to. On the
/// ground: the field whose pavement is under the wheels (the region's own
/// ground rule, nearest runway footprint within [`AT_FIELD_NM`]). Airborne:
/// the nearest within [`NEW_FLIGHT_AIR_NM`]. `None`: no staffed tower here
/// (an uncontrolled field, or nowhere). Pure over the index.
fn field_under_in(region: &[RegionApt], lat: f64, lon: f64, on_ground: bool) -> Option<&RegionApt> {
    let reach = if on_ground { AT_FIELD_NM } else { NEW_FLIGHT_AIR_NM };
    let dist = |a: &RegionApt| {
        let direct = crate::nav::haversine_nm(lat, lon, a.lat, a.lon);
        if on_ground {
            a.apt.ground_distance_nm(lat, lon).unwrap_or(direct)
        } else {
            direct
        }
    };
    region
        .iter()
        .filter(|a| a.apt.freq(nav::FREQ_TOWER).is_some())
        .map(|a| (dist(a), a))
        .filter(|(d, _)| *d <= reach)
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, a)| a)
}

/// The towered field under the aircraft, as (ICAO, name); see
/// [`field_under_in`].
pub fn field_under(lat: f64, lon: f64, on_ground: bool) -> Option<(String, String)> {
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    field_under_in(&region, lat, lon, on_ground).map(|a| (a.icao.clone(), a.apt.name.clone()))
}

/// A NEW FLIGHT wherever the sim put us: the station for the field under
/// the aircraft's NEW position is rebuilt ([`arm_new_flight`]), never the
/// field the station happened to be working. EGKK, 2026-09-28: the runner
/// moved the 737 from LEMG to Gatwick stand 71, the new-flight watch re-armed
/// the ACTIVE field (Malaga) and its cooldown held the region's relocation
/// off for 180 s, so full auto departed Gatwick as an uncontrolled field and
/// looked its stand up at LEMG (#151). Returns the field and its tower;
/// `None` when no staffed tower is under the position (nothing armed: the
/// old station, retired and out of reach, stays silent).
pub fn arm_new_flight_at(lat: f64, lon: f64, on_ground: bool, now: Instant) -> Option<(String, i64)> {
    let (icao, _) = field_under(lat, lon, on_ground)?;
    let khz = arm_new_flight(&icao, now)?;
    Some((icao, khz))
}

/// A NEW FLIGHT started AIRBORNE on its plan (the sortie runner's enroute
/// setup): an IFR flight mid-route, already identified, never a departure
/// and never a field's circuit traffic.
#[derive(Debug, Clone, PartialEq)]
pub struct EnrouteStation {
    /// The field whose radar unit works the flight, and its tower (the
    /// station built for it).
    pub icao: String,
    pub tower_khz: i64,
    /// "Approach" or "Departure", and its frequency: where COM1 goes.
    pub position: &'static str,
    pub khz: i64,
    /// The flight's discrete code, kept unit to unit ([`IFR_FLIGHT`]).
    pub code: i64,
}

/// The radar unit an airborne IFR flight at this position is worked by: the
/// region's fields that HAVE one (approach or departure; a tower-only field
/// is passed over, as the enroute handoff passes it), service-weighted like
/// the airborne region pick, within [`SWITCH_MAX_NM`]. Pure over the index.
fn enroute_field_in(region: &[RegionApt], lat: f64, lon: f64) -> Option<&RegionApt> {
    region
        .iter()
        .filter(|a| radar_position(&a.apt).is_some() && a.apt.freq(nav::FREQ_TOWER).is_some())
        .map(|a| (crate::nav::haversine_nm(lat, lon, a.lat, a.lon), a))
        .filter(|(d, _)| *d <= SWITCH_MAX_NM)
        .map(|(d, a)| (d - SERVICE_BIAS_NM * region_services(&a.apt) as f64, a))
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, a)| a)
}

/// A NEW FLIGHT put in the air on its plan, mid-route: the world holds it
/// as an IFR flight already under radar service. Its code
/// ([`discrete_squawk`] of the callsign) is published as [`IFR_FLIGHT`],
/// airborne, so every region handoff after this keeps it and goes to a
/// radar position; the station for the nearest field with a radar unit is
/// rebuilt ([`arm_new_flight`]), and its approach (else departure)
/// frequency is the one to tune. No clearance, no departure handoff, no
/// radar watch: the flight is picked up identified, as a real one handed
/// over mid-route would be. Called after [`retire_for_new_flight`]. `None`:
/// no radar unit within reach (nothing armed, the code still published).
pub fn arm_new_flight_enroute(
    lat: f64,
    lon: f64,
    callsign: &str,
    now: Instant,
) -> Option<EnrouteStation> {
    let code = discrete_squawk(callsign);
    *IFR_FLIGHT.lock().unwrap_or_else(|e| e.into_inner()) = Some(IfrFlight {
        code,
        callsign: callsign.to_string(),
        airborne: true,
    });
    let (icao, position, khz) = {
        let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
        let a = enroute_field_in(&region, lat, lon)?;
        let (position, khz) = radar_position(&a.apt)?;
        (a.icao.clone(), position, khz)
    };
    let tower_khz = arm_new_flight(&icao, now)?;
    Some(EnrouteStation { icao, tower_khz, position, khz, code })
}

/// Whether the installed station is the departure field's, judged at a
/// mission's arm.
#[derive(Debug, Clone, PartialEq)]
pub enum DepartureStation {
    /// The station up is the field's (or no towered field is under us: an
    /// uncontrolled departure).
    Ready,
    /// The field's station is armed and the main loop has not installed it.
    Waiting { want: String },
    /// The station up is another field's (or the last flight's worksheet)
    /// and nothing is armed for this one.
    Arm { want: String },
}

/// Judge the station against the field under the aircraft. `active`: the
/// installed station's field; `under`: the towered field under the wheels;
/// `pending`: a swap armed and not yet installed; `new_flight_pending`: the
/// station up still holds the last flight's worksheet. Pure.
pub fn judge_departure_station(
    active: Option<&str>,
    under: Option<&str>,
    pending: Option<&str>,
    new_flight_pending: bool,
) -> DepartureStation {
    let Some(want) = under else {
        return DepartureStation::Ready;
    };
    let same = |x: Option<&str>| x.is_some_and(|x| x.eq_ignore_ascii_case(want));
    if same(pending) {
        return DepartureStation::Waiting { want: want.to_string() };
    }
    if same(active) && !new_flight_pending && pending.is_none() {
        return DepartureStation::Ready;
    }
    DepartureStation::Arm { want: want.to_string() }
}

/// The field of the swap armed and not yet installed.
fn pending_station_icao() -> Option<String> {
    PENDING_STATION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(icao, _)| icao.clone())
}

/// The departure station at the aircraft's position, on the ground: judged
/// by [`judge_departure_station`], and the field's station armed when `arm`
/// and the verdict asks for it. Returns the verdict and the installed
/// station's field.
pub fn departure_station(
    lat: f64,
    lon: f64,
    arm: bool,
    now: Instant,
) -> (DepartureStation, Option<String>) {
    let under = field_under(lat, lon, true).map(|(icao, _)| icao);
    let active = active_icao().map(|(icao, _, _)| icao);
    let pending = pending_station_icao();
    let verdict = judge_departure_station(
        active.as_deref(),
        under.as_deref(),
        pending.as_deref(),
        new_flight_pending(),
    );
    if let (DepartureStation::Arm { want }, true) = (&verdict, arm) {
        let _ = arm_new_flight(want, now);
    }
    (verdict, active)
}

/// The radio row's content: the ACTIVE station's name and services — but only
/// while it's inside VHF range. A tower you can't physically raise isn't a
/// radio row; parked in Extremadura the panel showed Gibraltar's comm plan as
/// if it could answer (2026-07-10). `None` = no station, or out of range.
pub fn active_station_in_range(
    lat: f64,
    lon: f64,
    alt_msl_ft: f64,
) -> Option<(String, Vec<(String, i64)>)> {
    let (_, alat, alon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    let (name, elev_ft, svcs) = ACTIVE_PANEL
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    let d = crate::nav::haversine_nm(lat, lon, alat, alon);
    (d <= vhf_range_nm(alt_msl_ft - elev_ft)).then_some((name, svcs))
}

/// Range to the ACTIVE station's field, for the radio's signal-quality model
/// (`radiofx::signal_quality` in the crew crate) — published to the speech arbiter by
/// the monitor each tick. `None` = no active station installed; ground
/// distance, not slant (the altitude term is noise at the ranges where
/// quality changes).
/// On the ground this far from the station's field, the station is out of
/// reach: a tower clears the aircraft on its own field, not one ten miles
/// away (the ground's line of sight is 8.7 nm, `vhf_range_nm(0.0)`).
/// Airborne the station keeps the flight ("enroute over nowhere keeps the
/// last controller", [`SWITCH_MAX_NM`]).
pub const GROUND_REACH_NM: f64 = 10.0;

/// The active station cannot hear or be heard: set per monitor tick by
/// [`note_aircraft`] from the aircraft's position.
static OUT_OF_REACH: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub(crate) fn out_of_reach() -> bool {
    OUT_OF_REACH.load(std::sync::atomic::Ordering::Relaxed)
}

pub(crate) fn station_unavailable() -> bool {
    out_of_reach() || uncontrolled_arrival()
}

/// The world's view of where the aircraft is, once per monitor tick. On the
/// ground beyond [`GROUND_REACH_NM`] of the station's field the station goes
/// silent, and what it had issued is retired: a clearance from a field we
/// are not at is nobody's (the crew then departs as from an uncontrolled
/// field). A route the crew planned for itself is left alone.
pub fn note_aircraft(lat: f64, lon: f64, on_ground: bool) {
    use std::sync::atomic::Ordering::Relaxed;
    // Where we are, for the go-around judgement's time to the threshold.
    note_own_fix(lat, lon, on_ground, Instant::now());
    // The crew's own route and runway end at the liftoff: a departure's
    // runway must never read as a cleared one at the destination (autoland
    // takes the cleared runway).
    if !on_ground {
        let mut own = SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner());
        if own.take().is_some() {
            *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
    }
    // An IFR flight ends at its landing: airborne since the clearance, the
    // next ground contact retires it (never a region switch).
    {
        let mut ifr = IFR_FLIGHT.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(f) = ifr.as_mut() {
            if !on_ground {
                f.airborne = true;
            } else if f.airborne {
                sim::blackbox::record(
                    "atc",
                    serde_json::json!({ "event": "ifr_flight_ended", "code": f.code, "why": "landed" }),
                );
                *ifr = None;
            }
        }
    }
    let far = on_ground
        && active_station_range_nm(lat, lon).is_some_and(|d| d > GROUND_REACH_NM);
    let was = OUT_OF_REACH.swap(far, Relaxed);
    if far && !was {
        let station = ACTIVE_FIELD
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map(|f| f.0.clone());
        tracing::info!(?station, "ATC station out of reach on the ground: standing it down here");
        sim::blackbox::record("atc", serde_json::json!({ "route": "reach", "station": station, "out": true }));
        *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ASSIGNED_SQUAWK.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *IFR_CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *IFR_FLIGHT.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *LINE_UP_CONDITION.lock().unwrap_or_else(|e| e.into_inner()) = None;
        if SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner()).is_none() {
            *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = false;
        }
    } else if was && !far {
        sim::blackbox::record("atc", serde_json::json!({ "route": "reach", "out": false }));
    }
}

/// A planned runway is kept for an uncontrolled departure unless the wind
/// puts more than this on its tail.
pub const SELF_TAXI_TAILWIND_KTS: f64 = 5.0;

/// The field the aircraft is on: the nearest by pavement within
/// [`AT_FIELD_NM`], any field (not only towered ones). A full apt.dat pass.
fn field_at(apt_path: &str, lat: f64, lon: f64) -> Option<(String, nav::AirportInfo)> {
    let mut best: Option<(f64, String, nav::AirportInfo)> = None;
    let _ = nav::scan_airports(apt_path, |icao, apt| {
        let Some(d) = apt.ground_distance_nm(lat, lon) else {
            return;
        };
        if d <= AT_FIELD_NM && best.as_ref().is_none_or(|(b, ..)| d < *b) {
            best = Some((d, icao.to_string(), apt));
        }
    });
    best.map(|(_, icao, apt)| (icao, apt))
}

/// The published [`TAXI_ROUTE`] is the crew's own (an uncontrolled field),
/// not a controller's, to this runway end. Cleared by any controller's
/// route. It stands where a controller's cleared runway would for the
/// line-up: with none, the line-up took the nearest end and lined the
/// Evolution up on LILB 21 for a route planned to 03 (2026-09-25).
static SELF_TAXI: Mutex<Option<String>> = Mutex::new(None);

/// The runway of the crew's own route, when there is one.
pub(crate) fn self_taxi_runway() -> Option<String> {
    SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// A name a pilot could say: a taxiway designator ("A", "B2", "TL").
/// An uncontrolled field's apt.dat names are whatever its author typed
/// ("Taxy", "21_hold" at LILB), and no controller turns them into words.
fn designator(name: &str) -> bool {
    let n = name.trim();
    let letters = n.chars().take_while(|c| c.is_ascii_alphabetic()).count();
    let digits = n.chars().skip(letters).take_while(|c| c.is_ascii_digit()).count();
    (1..=2).contains(&letters) && digits <= 2 && letters + digits == n.len()
}

/// An uncontrolled departure's route, as the crew announces it.
#[derive(Debug, Clone, PartialEq)]
pub struct SelfTaxi {
    pub field: String,
    pub runway: String,
    /// Spoken taxiway names, in order ("Alpha, Bravo").
    pub via: String,
    /// Runways crossed on the way, spoken idents.
    pub crossings: Vec<String>,
}

/// The crew's own route to parking at an uncontrolled arrival field.
#[derive(Debug, Clone, PartialEq)]
pub struct SelfTaxiStand {
    pub field: String,
    pub stand: String,
    pub via: String,
}

/// Plan and publish a stand route without a controller. This is the arrival
/// mirror of [`plan_self_taxi`]: the field is resolved from the pavement under
/// the aircraft, and the route is published as taxi-to-stand so the ordinary
/// taxi-in drive and securing checks consume it unchanged.
pub fn plan_self_taxi_to_stand(
    apt_path: &str,
    lat: f64,
    lon: f64,
    psi_true: Option<f64>,
) -> Result<SelfTaxiStand, String> {
    let (icao, apt) = field_at(apt_path, lat, lon).ok_or_else(|| {
        "I can't tell which field we're at, so the taxi is yours.".to_string()
    })?;
    let net = nav::find_taxi_net(apt_path, &icao)
        .ok()
        .flatten()
        .ok_or_else(|| format!("{icao} has no taxiways drawn, so the taxi is yours."))?;
    let mut plan = stand_route_on(&net, &apt.runways, lat, lon, psi_true)
        .filter(|p| p.path.len() >= 2)
        .ok_or_else(|| format!("I can't find a taxi route to parking at {icao}."))?;
    let stand = plan.holding_point.clone();
    let raw_via = plan.via.join(", ");
    let named: Vec<String> = plan.via.iter().filter(|n| designator(n)).cloned().collect();
    let via = spoken_via(&named).join(", ");
    plan.seg = plan
        .seg
        .into_iter()
        .map(|(n, rwy)| (if rwy || designator(&n) { n } else { String::new() }, rwy))
        .collect();
    *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = Some(TaxiWatch {
        path: plan.path,
        via: via.clone(),
        seg: plan.seg,
        holding_point: stand.clone(),
        lead_on: plan.lead_on,
    });
    *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = true;
    *SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner()) = None;
    sim::blackbox::record(
        "atc",
        serde_json::json!({
            "route": "self_taxi_in", "field": icao, "stand": stand, "via": raw_via,
        }),
    );
    Ok(SelfTaxiStand { field: apt.name, stand, via })
}

/// Plan the crew's own taxi at an uncontrolled field (no tower, or none in
/// reach): `icao`'s taxi network from `apt_path` (no ICAO = the field we
/// are at, found by a scan), to the holding point of `runway` (the plan's
/// departure runway) unless the wind puts more than [`SELF_TAXI_TAILWIND_KTS`]
/// on its tail, else the end the wind favours, else the longest main runway. Published to the party line like a tower's
/// route, so the taxi drive, the line-up's lead-on and the guide read one
/// route. `Err` says why, in words the crew can say. A full apt.dat pass:
/// call it off the realtime loop.
pub fn plan_self_taxi(
    apt_path: &str,
    icao: Option<&str>,
    lat: f64,
    lon: f64,
    runway: Option<&str>,
    wind: Option<(f64, f64)>,
) -> Result<SelfTaxi, String> {
    let found = match icao {
        Some(icao) => nav::find_airport(apt_path, icao)
            .ok()
            .flatten()
            .map(|a| (icao.to_string(), a))
            .ok_or_else(|| format!("I can't find {icao} in the scenery data."))?,
        None => field_at(apt_path, lat, lon).ok_or_else(|| {
            "I can't tell which field we're at, so the taxi is yours.".to_string()
        })?,
    };
    let (icao, apt) = (found.0.as_str(), found.1);
    let dist = apt
        .ground_distance_nm(lat, lon)
        .or_else(|| apt.position().map(|(a, b)| crate::nav::haversine_nm(lat, lon, a, b)))
        .unwrap_or(f64::MAX);
    if dist > AT_FIELD_NM {
        return Err(format!("We're not at {icao}, so I can't plan its taxi."));
    }
    let net = nav::find_taxi_net(apt_path, icao)
        .ok()
        .flatten()
        .ok_or_else(|| format!("{icao} has no taxiways drawn, so the taxi is yours."))?;
    let tail_ok = |r: &str| {
        wind.and_then(|(from, kts)| nav::headwind_for(&apt.runways, r, from, kts))
            .is_none_or(|hw| hw >= -SELF_TAXI_TAILWIND_KTS)
    };
    let asked = runway
        .filter(|r| strip(&apt.runways, r).is_some() && tail_ok(r))
        .map(str::to_string);
    let ident = asked
        .or_else(|| {
            wind.filter(|(_, kts)| *kts >= 1.0)
                .and_then(|(from, kts)| nav::best_runway_for_wind(&apt.runways, from, kts))
                .map(|(id, _, _)| id)
        })
        .or_else(|| {
            nav::assignable_runways(&apt.runways)
                .max_by(|a, b| a.length_m().total_cmp(&b.length_m()))
                .map(|r| r.ident_a.clone())
        })
        .ok_or_else(|| format!("{icao} has no runway I can pick."))?;
    let r = strip(&apt.runways, &ident).ok_or_else(|| format!("No runway {ident} at {icao}."))?;
    let (tlat, tlon) = if r.ident_a == ident {
        (r.lat_a, r.lon_a)
    } else {
        (r.lat_b, r.lon_b)
    };
    let standoff = (r.width_m / 2.0 + 30.0).max(nav::HOLD_SHORT_STANDOFF_M);
    let zone = [r.ident_a.clone(), r.ident_b.clone()];
    let mut plan = net
        .plan_to_runway(lat, lon, tlat, tlon, standoff, &zone, Some(r))
        .filter(|p| p.path.len() >= 2)
        .ok_or_else(|| {
            format!(
                "I can't find a taxi route to runway {} from here.",
                atis::spoken_runway(&ident)
            )
        })?;
    mark_runway_crossings(&mut plan, &apt.runways);
    // Only names a pilot could say ride the route: the drive, the guide
    // and the announcements read them.
    let raw_via = plan.via.join(", ");
    let named: Vec<String> = plan.via.iter().filter(|n| designator(n)).cloned().collect();
    let via = spoken_via(&named).join(", ");
    let seg = plan
        .seg
        .iter()
        .map(|(n, rwy)| (if *rwy || designator(n) { n.clone() } else { String::new() }, *rwy))
        .collect();
    let holding_point = if designator(&plan.holding_point) {
        plan.holding_point.clone()
    } else {
        String::new()
    };
    *TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner()) = Some(TaxiWatch {
        path: plan.path.clone(),
        via: via.clone(),
        seg,
        holding_point,
        lead_on: plan.lead_on.clone(),
    });
    *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner()) = false;
    *SELF_TAXI.lock().unwrap_or_else(|e| e.into_inner()) = Some(ident.clone());
    sim::blackbox::record(
        "atc",
        serde_json::json!({ "route": "self_taxi", "field": icao, "runway": ident, "via": raw_via }),
    );
    Ok(SelfTaxi {
        field: apt.name.clone(),
        runway: ident,
        via,
        crossings: plan.crossings.iter().map(|c| atis::spoken_runway(c)).collect(),
    })
}

pub fn active_station_range_nm(lat: f64, lon: f64) -> Option<f64> {
    let (_, alat, alon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    Some(crate::nav::haversine_nm(lat, lon, alat, alon))
}

/// The active field's runway geometry, published by [`Station::install`] for
/// the runway-incursion watch.
static ACTIVE_RUNWAYS: Mutex<Vec<nav::Runway>> = Mutex::new(Vec::new());

/// The active field's runways, for the party line (`field_runways`).
pub(crate) fn active_runways() -> Vec<nav::Runway> {
    ACTIVE_RUNWAYS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
}

/// Lateral capture around a runway centerline when the 100-row carried no
/// width: 40 m covers everything up to a wide-body runway (60 m ÷ 2 + slack)
/// without capturing a parallel taxiway.
const RUNWAY_HALF_WIDTH_M: f64 = 40.0;
/// Slack past the real pavement edge: edge lines, a mainwheel over the
/// threshold stripe — not the holding point. Jerez's E1 sits 40 m off the
/// 45 m runway's centerline; the blanket 40 m capture accused a pilot
/// holding SHORT of runway 20 of being ON it (2026-07-11).
const RUNWAY_EDGE_SLACK_M: f64 = 10.0;

/// Is this position ON one of the active field's runways? Point-to-segment in
/// a local flat-earth projection (exact enough over a 3 km runway). Returns
/// the spoken ident of the nearer end, for the blackbox and the call.
pub fn on_active_runway(lat: f64, lon: f64) -> Option<String> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    runways.iter().find_map(|r| on_runway(r, lat, lon))
}

/// [`on_active_runway`], naming the END the aircraft is pointing down when
/// its true heading is known, else the nearer end. The nearer end is the
/// wrong name for an aircraft that has rolled past the middle: a 737 landed
/// LEMG 12, stopped heading 117 in the far half, and was told "you landed on
/// runway three zero" (2026-09-26).
pub fn on_active_runway_facing(lat: f64, lon: f64, heading_true: Option<f64>) -> Option<String> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    runways.iter().find_map(|r| {
        let nearer = on_runway(r, lat, lon)?;
        Some(match heading_true.filter(|h| h.is_finite()) {
            Some(h) => {
                let off_a = sim::angles::abs_diff_deg(h, r.heading_a_deg());
                let off_b = sim::angles::abs_diff_deg(h, r.heading_b_deg());
                if off_a <= off_b { r.ident_a.clone() } else { r.ident_b.clone() }
            }
            None => nearer,
        })
    })
}

/// [`on_active_runway`] with "unknown" kept apart from "no": `None` when no
/// field's runways are published (no station yet), never a guessed `false`.
pub fn on_active_runway_known(lat: f64, lon: f64) -> Option<bool> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    if runways.is_empty() {
        return None;
    }
    Some(runways.iter().any(|r| on_runway(r, lat, lon).is_some()))
}

/// The strip under this position, keyed by its `ident_a` (both ends of one
/// pavement are one strip), `None` off every active runway.
fn strip_under(lat: f64, lon: f64) -> Option<String> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    runways
        .iter()
        .find(|r| on_runway(r, lat, lon).is_some())
        .map(|r| r.ident_a.clone())
}

/// Does the tower's own book put us on the runway we stand on? The runway
/// incursion watch ("traffic on final, exit the runway") asks this before it
/// shouts: a standing takeoff clearance or line-up; a landing clearance on
/// THIS strip not yet reported vacated (the rollout and the vacate, EGKK 26L
/// 2026-09-27: "exit the runway immediately" from 20 s after touchdown to
/// the vacate, eight times); a runway start; the strip we LANDED on, cleared
/// or not, until we leave it (EGKK-EGBB 15, 2026-09-28: four "exit the
/// runway" during the vacate of an uncleared landing); a crossing on the
/// cleared route that keeps moving. Anything else on a strip with traffic on
/// final is the incursion it is.
pub(crate) fn runway_occupancy_authorised(lat: f64, lon: f64, gs_kts: f64) -> bool {
    let Some(here) = strip_under(lat, lon) else {
        return false;
    };
    let clr = CLEARANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.runway_ident.clone(), c.holds_runway));
    if let Some((state, ident, holds)) = &clr {
        if matches!(state, tower::State::LinedUp | tower::State::TakeoffCleared) {
            return true;
        }
        if *state == tower::State::ClearedToLand && *holds {
            let cleared = {
                let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
                strip(&runways, ident).map(|r| r.ident_a.clone())
            };
            if ident.is_empty() || cleared.as_deref() == Some(here.as_str()) {
                return true;
            }
        }
    }
    let (started_here, landed_here) = {
        let w = COMPLIANCE.lock().unwrap_or_else(|e| e.into_inner());
        (
            w.runway_start.as_deref() == Some(here.as_str()),
            w.landed_strip.as_deref() == Some(here.as_str()) && gs_kts >= LANDED_VACATING_KTS,
        )
    };
    // A runway start, or the strip we landed on and are still rolling off
    // (cleared or not: an aircraft vacating is not an incursion).
    if started_here || landed_here {
        return true;
    }
    let arrival_taxi = *TAXI_TO_STAND.lock().unwrap_or_else(|e| e.into_inner());
    let cleared_taxi = clr
        .as_ref()
        .is_some_and(|(s, _, _)| *s == tower::State::TaxiCleared)
        || arrival_taxi;
    cleared_taxi && gs_kts >= 3.0 && on_cleared_runway_crossing(lat, lon)
}

/// Is this position ON one strip? The shared segment test behind
/// [`on_active_runway`] and the per-runway traffic watches. Returns the
/// spoken-able ident of the nearer end.
fn on_runway(r: &nav::Runway, lat: f64, lon: f64) -> Option<String> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    let (px, py) = (
        (lon - r.lon_a) * m_per_deg_lon,
        (lat - r.lat_a) * m_per_deg_lat,
    );
    let (bx, by) = (
        (r.lon_b - r.lon_a) * m_per_deg_lon,
        (r.lat_b - r.lat_a) * m_per_deg_lat,
    );
    let len2 = bx * bx + by * by;
    if len2 < 1.0 {
        return None; // degenerate row
    }
    let t = (px * bx + py * by) / len2;
    let cross_m = (px * by - py * bx).abs() / len2.sqrt();
    let half_width = if r.width_m > 1.0 {
        r.width_m / 2.0 + RUNWAY_EDGE_SLACK_M
    } else {
        RUNWAY_HALF_WIDTH_M
    };
    // Slightly past the thresholds still counts (displaced/overrun).
    if (-0.02..=1.02).contains(&t) && cross_m <= half_width {
        let ident = if t < 0.5 { &r.ident_a } else { &r.ident_b };
        return Some(ident.clone());
    }
    None
}

/// The strip a runway END belongs to: landing 27L and departing 09R share
/// the same pavement, so conflict checks compare STRIPS, not end names.
fn strip<'a>(runways: &'a [nav::Runway], ident: &str) -> Option<&'a nav::Runway> {
    runways
        .iter()
        .find(|r| r.ident_a == ident || r.ident_b == ident)
}

/// The heading of one runway END, from its strip's geometry.
fn end_heading(runways: &[nav::Runway], ident: &str) -> Option<f64> {
    let r = strip(runways, ident)?;
    Some(if r.ident_a == ident {
        r.heading_a_deg()
    } else {
        r.heading_b_deg()
    })
}

/// Two runway ends point against each other: the reciprocal of one strip,
/// or a near-parallel's opposite end (LEMG 31 against 12). More than 120
/// degrees apart; a crossing strip is not opposed. Unknown ends never are.
fn ends_oppose(runways: &[nav::Runway], a: &str, b: &str) -> bool {
    match (end_heading(runways, a), end_heading(runways, b)) {
        (Some(ha), Some(hb)) => sim::angles::abs_diff_deg(ha, hb) > 120.0,
        _ => false,
    }
}

/// The arrivals in one strip's final corridor: nearest distance, its spoken
/// callsign (names the condition of a conditional line-up), and how many are
/// in the stream (a conditional is only legal against exactly one).
pub struct FinalTraffic {
    pub nm: f64,
    pub callsign: Option<String>,
    pub count: usize,
    /// Every arrival in the corridor, by its raw callsign: the traffic a
    /// held departure was held FOR ([`TakeoffHold::held_for`]).
    pub ghosts: Vec<String>,
}

/// Lateral capture around the extended centerline: half of the tightest
/// real parallel spacing (Heathrow's pair is ~1,400 m apart) with room
/// for a wobbly final.
const CORRIDOR_HALF_M: f64 = 500.0;
/// How far out an arrival still owns the runway.
const FINAL_NM: f64 = 4.0;
/// A target climbing faster than this is a DEPARTURE leaving, not an
/// arrival: a real climb-out shows +500 fpm or more, an arrival on final
/// descends, and a level-off or an unfed VS reads ~0 and stays counted
/// (fail toward caution). A ghost lifting off Malaga runway 30 read as
/// "traffic on short final" to the runway 12 incursion watch, 2026-07-12.
const CLIMBOUT_FPM: f64 = 300.0;

/// The climb departure gives a check-in: the one it announced, never above
/// the FILED cruise (`cap`, feet). EGKK-EGBB (sortie 1790601822-full_auto,
/// 2026-09-28, #155): filed FL180, the crew announced "climbing flight level
/// two two zero" (HEME1B's HEMEL window on its legs) and departure cleared
/// FL220. A level departure cannot hear, or no cap, stands as announced. The
/// cap is said as the announcement said its level: a flight level at or
/// above the field's transition level, or when the field has none and the
/// crew said one.
pub fn capped_departure_climb(
    announced: &str,
    cap: Option<i64>,
    transition_level_ft: Option<f64>,
) -> String {
    let (Some(heard), Some(cap)) = (
        tower::heard_altitude_ft(announced),
        cap.filter(|c| *c > 0),
    ) else {
        return announced.to_string();
    };
    if heard <= cap {
        return announced.to_string();
    }
    let level = atis::is_flight_level(cap as f64, transition_level_ft)
        || (transition_level_ft.is_none() && announced.to_lowercase().contains("flight level"));
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "climb_capped", "announced_ft": heard, "cleared_ft": cap}),
    );
    if level {
        atis::spoken_flight_level(cap as f64)
    } else {
        atis::spoken_alt(cap as f64)
    }
}

/// The altitude a departure check-in announces ("passing one thousand
/// climbing five thousand" gives "five thousand"), for the controller to
/// clear. Whatever follows the last "climbing", trimmed of punctuation.
pub fn climb_announced(transcript: &str) -> Option<String> {
    let t = transcript.trim_end_matches(['.', ',', ' ']);
    let i = t.to_lowercase().rfind("climbing")?;
    let rest = t[i + "climbing".len()..].trim();
    let rest = rest.trim_start_matches("to ").trim();
    (!rest.is_empty()).then(|| rest.to_string())
}

/// The runway END the nearest arrival on final is landing on: on the
/// extended centreline beyond threshold A it is landing on A, beyond B on
/// B. Same corridor and altitude band as [`final_traffic_for`].
pub fn arrival_end(targets: &[TcasTarget], runways: &[nav::Runway], field_elev_ft: f64) -> Option<String> {
    let mut best: Option<(f64, String)> = None;
    for rwy in runways {
        let m_per_deg_lat = 111_320.0;
        let m_per_deg_lon = 111_320.0 * rwy.lat_a.to_radians().cos();
        let (bx, by) = (
            (rwy.lon_b - rwy.lon_a) * m_per_deg_lon,
            (rwy.lat_b - rwy.lat_a) * m_per_deg_lat,
        );
        let len2 = bx * bx + by * by;
        if len2 < 1.0 {
            continue;
        }
        let len_m = len2.sqrt();
        for t in targets
            .iter()
            .filter(|t| !t.on_ground)
            .filter(|t| (30.0..1500.0).contains(&(t.ele_ft - field_elev_ft)))
            .filter(|t| t.vs_fpm <= CLIMBOUT_FPM)
            .filter(|t| !on_independent_final(t, rwy, runways))
        {
            let (px, py) = (
                (t.lon - rwy.lon_a) * m_per_deg_lon,
                (t.lat - rwy.lat_a) * m_per_deg_lat,
            );
            let along = (px * bx + py * by) / len2;
            let cross_m = (px * by - py * bx).abs() / len_m;
            let beyond = FINAL_NM * 1852.0 / len_m;
            if cross_m > CORRIDOR_HALF_M {
                continue;
            }
            let (end, dist_m) = if along < 0.0 && along >= -beyond {
                (&rwy.ident_a, (px * px + py * py).sqrt())
            } else if along > 1.0 && along <= 1.0 + beyond {
                let (qx, qy) = (px - bx, py - by);
                (&rwy.ident_b, (qx * qx + qy * qy).sqrt())
            } else {
                continue;
            };
            if best.as_ref().is_none_or(|(d, _)| dist_m < *d) {
                best = Some((dist_m, end.clone()));
            }
        }
    }
    // A close parallel's corridor is the main runway's too: the stream
    // lands on the runway the field works (`nav::close_parallel_main`).
    best.map(|(_, e)| nav::close_parallel_main(runways, &e))
}

/// Live traffic on final for THIS strip: airborne, low over the field,
/// inside the runway's extended-centerline corridor (either direction), and
/// not climbing away — a departure climbing through the corridor is leaving,
/// not arriving. Distances are to the nearer threshold, in nm. The
/// runway-blind version of this check held a Heathrow 09R departure for the
/// endless 09L arrivals — indefinitely (2026-07-10); a parallel runway's
/// stream is not your traffic.
///
/// `runways` is the field's (the strip's own included): an arrival on an
/// INDEPENDENT runway's final ([`on_independent_final`]) is that runway's
/// traffic, even where its final crosses ours (LEMG, issue #172).
pub fn final_traffic_for(
    targets: &[TcasTarget],
    rwy: &nav::Runway,
    runways: &[nav::Runway],
    field_elev_ft: f64,
) -> Option<FinalTraffic> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * rwy.lat_a.to_radians().cos();
    let (bx, by) = (
        (rwy.lon_b - rwy.lon_a) * m_per_deg_lon,
        (rwy.lat_b - rwy.lat_a) * m_per_deg_lat,
    );
    let len2 = bx * bx + by * by;
    if len2 < 1.0 {
        return None;
    }
    let len_m = len2.sqrt();
    let mut hits: Vec<(f64, &TcasTarget)> = targets
        .iter()
        .filter(|t| !t.on_ground)
        .filter(|t| (30.0..1500.0).contains(&(t.ele_ft - field_elev_ft)))
        .filter(|t| t.vs_fpm <= CLIMBOUT_FPM)
        .filter(|t| !on_independent_final(t, rwy, runways))
        .filter_map(|t| {
            let (px, py) = (
                (t.lon - rwy.lon_a) * m_per_deg_lon,
                (t.lat - rwy.lat_a) * m_per_deg_lat,
            );
            let along = (px * bx + py * by) / len2;
            let cross_m = (px * by - py * bx).abs() / len_m;
            let beyond = FINAL_NM * 1852.0 / len_m;
            if cross_m > CORRIDOR_HALF_M || along < -beyond || along > 1.0 + beyond {
                return None;
            }
            let da = (px * px + py * py).sqrt();
            let (qx, qy) = (px - bx, py - by);
            let db = (qx * qx + qy * qy).sqrt();
            Some((da.min(db) / 1852.0, t))
        })
        .collect();
    hits.sort_by(|x, y| x.0.total_cmp(&y.0));
    let (nm, nearest) = hits.first()?;
    Some(FinalTraffic {
        nm: *nm,
        callsign: (!nearest.callsign.is_empty()).then(|| spoken_ghost(&nearest.callsign)),
        count: hits.len(),
        ghosts: hits.iter().map(|(_, t)| t.callsign.clone()).collect(),
    })
}

/// A position against one strip's centreline in flat metres: (along as a
/// share of the length from `ident_a`, metres off the centreline, length).
fn centreline_place(r: &nav::Runway, lat: f64, lon: f64) -> Option<(f64, f64, f64)> {
    let m_lon = 111_320.0 * r.lat_a.to_radians().cos();
    let (bx, by) = ((r.lon_b - r.lon_a) * m_lon, (r.lat_b - r.lat_a) * 111_320.0);
    let len2 = bx * bx + by * by;
    if len2 < 1.0 {
        return None;
    }
    let (px, py) = ((lon - r.lon_a) * m_lon, (lat - r.lat_a) * 111_320.0);
    Some((
        (px * bx + py * by) / len2,
        (px * by - py * bx).abs() / len2.sqrt(),
        len2.sqrt(),
    ))
}

/// The nearest two strips come to each other, metres (0 when they cross):
/// the least of the four end-to-other-strip distances.
fn strips_apart_m(a: &nav::Runway, b: &nav::Runway) -> f64 {
    let m_lon = 111_320.0 * a.lat_a.to_radians().cos();
    let xy = |lat: f64, lon: f64| ((lon - a.lon_a) * m_lon, (lat - a.lat_a) * 111_320.0);
    let (p1, p2) = (xy(a.lat_a, a.lon_a), xy(a.lat_b, a.lon_b));
    let (q1, q2) = (xy(b.lat_a, b.lon_a), xy(b.lat_b, b.lon_b));
    let cross = |o: (f64, f64), p: (f64, f64), q: (f64, f64)| {
        (p.0 - o.0) * (q.1 - o.1) - (p.1 - o.1) * (q.0 - o.0)
    };
    if cross(p1, p2, q1) * cross(p1, p2, q2) < 0.0 && cross(q1, q2, p1) * cross(q1, q2, p2) < 0.0 {
        return 0.0;
    }
    let to_seg = |p: (f64, f64), s0: (f64, f64), s1: (f64, f64)| {
        let (dx, dy) = (s1.0 - s0.0, s1.1 - s0.1);
        let l2 = dx * dx + dy * dy;
        let t = if l2 < 1.0 {
            0.0
        } else {
            (((p.0 - s0.0) * dx + (p.1 - s0.1) * dy) / l2).clamp(0.0, 1.0)
        };
        ((p.0 - s0.0 - t * dx).powi(2) + (p.1 - s0.1 - t * dy).powi(2)).sqrt()
    };
    to_seg(p1, q1, q2)
        .min(to_seg(p2, q1, q2))
        .min(to_seg(q1, p1, p2))
        .min(to_seg(q2, p1, p2))
}

/// Is this arrival on ANOTHER runway's final, one far enough from `rwy`
/// ([`nav::CLOSE_PARALLEL_M`], ICAO's segregated-parallel minimum) to be
/// worked on its own, and nearer that final's centreline than ours? Then it
/// is that runway's traffic. LEMG's 12 and 13 are 14 degrees apart and 826 m
/// at the thresholds (1,490 m at the far end); 12's final crosses 13's
/// extended centreline about 2 nm out, and a Ryanair landing 12 held a 737
/// lined up on 13 as "traffic on 2 mile final" (sortie 1790623420-full_auto,
/// 2026-09-28, issue #172). A close parallel or a crossing strip is one
/// runway's traffic.
fn on_independent_final(t: &TcasTarget, rwy: &nav::Runway, runways: &[nav::Runway]) -> bool {
    let Some((_, own_cross, _)) = centreline_place(rwy, t.lat, t.lon) else {
        return false;
    };
    runways
        .iter()
        .filter(|r| !(r.ident_a == rwy.ident_a && r.ident_b == rwy.ident_b))
        .filter(|r| strips_apart_m(r, rwy) >= nav::CLOSE_PARALLEL_M)
        .any(|r| {
            let Some((along, cross, len)) = centreline_place(r, t.lat, t.lon) else {
                return false;
            };
            let beyond = FINAL_NM * 1852.0 / len;
            let on_final = (-beyond..0.0).contains(&along) || (1.0..=1.0 + beyond).contains(&along);
            on_final && cross <= CORRIDOR_HALF_M && cross < own_cross
        })
}

/// The nearest airborne traffic on final for the strip we're standing ON:
/// the runway-incursion watch's traffic check, in nm to the threshold.
/// Strip-aware on purpose — the earlier version ranged EVERY low TCAS target
/// inside 3 nm, and a cleared Malaga runway 12 departure got "exit the
/// runway, expedite" for the parallel runway 31 arrival stream (2026-07-12).
/// A parallel runway's stream is not your traffic — the same Heathrow lesson
/// `final_traffic_for` already encodes for the release watch.
pub async fn inbound_traffic_for_strip(xp: SimView<'_>, ident: &str) -> Option<f64> {
    let targets = read_tcas_targets(xp).await;
    inbound_final_for_strip(&targets, ident)
}

/// The sync core of [`inbound_traffic_for_strip`], testable without a sim.
fn inbound_final_for_strip(targets: &[TcasTarget], ident: &str) -> Option<f64> {
    let (rwy, runways) = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        (strip(&runways, ident)?.clone(), runways.clone())
    };
    let elev_ft = ACTIVE_PANEL
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()?
        .1;
    final_traffic_for(targets, &rwy, &runways, elev_ft).map(|t| t.nm)
}

// --- Live-traffic watches (party line, go-around, held clearance) -----------
//
// LiveTraffic aircraft are replays of real flights — nothing can command
// them. The design stance (2026-07-10): the tower controls the ONE aircraft
// with a pilot in it and tells the truth about everything else. The party
// line NARRATES the clearance a ghost is about to comply with; the go-around
// order protects the player when a ghost takes the runway; the held takeoff
// clearance releases itself when the final actually clears.

/// One live TCAS target with an identity and an absolute position.
#[derive(Clone)]
pub struct TcasTarget {
    pub callsign: String,
    pub lat: f64,
    pub lon: f64,
    pub ele_ft: f64,
    pub on_ground: bool,
    /// Vertical speed, ft/min; 0.0 when the feed doesn't populate the array
    /// (reads as level, which every watch treats conservatively).
    pub vs_fpm: f64,
}

/// Read the TCAS target table (slot 0 = own ship, skipped). Targets without
/// a flight id are dropped — a tower can't address an aircraft it can't
/// name. Any missing array (older sim, no traffic feed) = empty.
pub async fn read_tcas_targets(xp: SimView<'_>) -> Vec<TcasTarget> {
    let (Ok(lat), Ok(lon), Ok(ele), Ok(wow), Ok(ids)) = (
        xp.get_float_array(datarefs::TCAS_LAT).await,
        xp.get_float_array(datarefs::TCAS_LON).await,
        xp.get_float_array(datarefs::TCAS_ELE_M).await,
        xp.get_float_array(datarefs::TCAS_WOW).await,
        xp.get_bytes(datarefs::TCAS_FLIGHT_ID).await,
    ) else {
        return Vec::new();
    };
    let ids: Vec<String> = ids
        .chunks(8)
        .map(|c| {
            String::from_utf8_lossy(c)
                .trim_matches('\0')
                .trim()
                .to_string()
        })
        .collect();
    // Vertical speed is OPTIONAL — an older sim or a feed that skips the
    // array must not blank the whole traffic picture, so a failed read pads
    // with level flight rather than joining the all-or-nothing gate above.
    let vs = xp
        .get_float_array(datarefs::TCAS_VS_FPM)
        .await
        .unwrap_or_default();
    lat.iter()
        .zip(&lon)
        .zip(&ele)
        .zip(&wow)
        .zip(&ids)
        .zip(vs.into_iter().chain(std::iter::repeat(0.0)))
        .skip(1) // slot 0 is own ship
        .filter(|(((((la, lo), _), _), id), _)| !id.is_empty() && (**la != 0.0 || **lo != 0.0))
        .map(|(((((la, lo), el), ww), id), vs)| TcasTarget {
            callsign: id.clone(),
            lat: *la,
            lon: *lo,
            ele_ft: *el * 3.28084,
            on_ground: *ww > 0.5,
            vs_fpm: vs,
        })
        .collect()
}

/// The runway end a DEPARTING ghost rolled from: nearest strip, end chosen
/// by the roll direction (bearing from last tick's position to this one).
/// `None` when the ghost hasn't moved enough for a bearing — the caller
/// falls back to the nearest end.
fn departure_runway_ident(lat: f64, lon: f64, prev_lat: f64, prev_lon: f64) -> Option<String> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    let (dx, dy) = (
        (lon - prev_lon) * m_per_deg_lon,
        (lat - prev_lat) * m_per_deg_lat,
    );
    if (dx * dx + dy * dy).sqrt() < 10.0 {
        return None; // no meaningful track over one tick
    }
    let track = dx.atan2(dy).to_degrees().rem_euclid(360.0);
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    let r = runways.iter().min_by(|a, b| {
        let da = crate::nav::haversine_nm(lat, lon, a.lat_a, a.lon_a)
            .min(crate::nav::haversine_nm(lat, lon, a.lat_b, a.lon_b));
        let db = crate::nav::haversine_nm(lat, lon, b.lat_a, b.lon_a)
            .min(crate::nav::haversine_nm(lat, lon, b.lat_b, b.lon_b));
        da.total_cmp(&db)
    })?;
    let da = sim::angles::abs_diff_deg(track, r.heading_a_deg());
    let db = sim::angles::abs_diff_deg(track, r.heading_b_deg());
    Some(if da <= db {
        r.ident_a.clone()
    } else {
        r.ident_b.clone()
    })
}

/// The runway a ghost's end is narrated as: a CLOSE parallel's is the main
/// runway's (`nav::close_parallel_main`), since replayed positions wander
/// between centrelines 200 m apart and the field works one of them.
fn main_runway_of(ident: &str) -> String {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    nav::close_parallel_main(&runways, ident)
}

/// The active field's runway end nearest a position — the runway a landing
/// ghost is aiming at, for the narrated clearance's flavor.
fn nearest_runway_ident(lat: f64, lon: f64) -> Option<String> {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    runways
        .iter()
        .flat_map(|r| {
            [
                (r.ident_a.clone(), r.lat_a, r.lon_a),
                (r.ident_b.clone(), r.lat_b, r.lon_b),
            ]
        })
        .map(|(id, la, lo)| (crate::nav::haversine_nm(lat, lon, la, lo), id))
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, id)| id)
}

/// Is this the pilot asking the CREW what to say on the radio? ("what do I
/// say", "how do I call the tower", "what's the call") — the copilot as
/// radio coach. Checked before the LLM; the answer comes from
/// [`suggest_call`], never improvised.
pub fn coach_query(t: &str) -> bool {
    t.contains("what do i say")
        || t.contains("what should i say")
        || t.contains("what do i tell")
        || t.contains("what should i tell")
        || t.contains("what's the call")
        || t.contains("what is the call")
        || t.contains("phraseology")
        || (t.contains("how do i")
            && (t.contains("call")
                || t.contains("ask")
                || t.contains("request")
                || t.contains("talk")))
        || (t.contains("help") && t.contains("radio"))
        || coach_yes_no(t)
}

/// "Should we check in with ATC?", "do we need to call the tower?", "is it
/// time to contact departure?": a yes/no about whether a call is owed. The
/// coach answers it (yes with the call, or no) rather than the copilot
/// staying silent (missed live, 2026-09-12).
pub fn coach_yes_no(t: &str) -> bool {
    let asks = t.starts_with("should we")
        || t.starts_with("should i")
        || t.starts_with("shall we")
        || t.starts_with("shall i")
        || t.contains("do we need to")
        || t.contains("do i need to")
        || t.contains("do we have to")
        || t.contains("is it time to")
        || t.contains("time to check")
        || t.contains("time we ")
        || t.contains("we owe")
        || t.contains("owe them")
        || t.contains("owe a call");
    let radio = t.contains("check in")
        || t.contains("checkin")
        || t.contains("call ")
        || t.ends_with("call")
        || t.contains("contact")
        || t.contains("talk to")
        || t.contains("report")
        || t.contains("radio")
        || t.contains("atc")
        || t.contains("tower")
        || t.contains("ground")
        || t.contains("departure")
        || t.contains("approach")
        || t.contains("center")
        || t.contains("centre")
        || t.contains("frequency");
    asks && radio
}

/// One coached hint from [`suggest_call`]: an optional prerequisite/status
/// line and the transmission itself. At least one is always present.
pub struct Suggestion {
    /// Spoken/shown BEFORE the call — a prerequisite ("First tune tower, one
    /// three one decimal two.", "First copy the ATIS on ...") or a status
    /// ("Under radar vectors. No call owed."). Never itself a transmission.
    pub first: Option<String>,
    /// The transmission to key — always a line the intent parser routes.
    pub call: Option<String>,
    /// The call is an OWED readback (runway-critical): say it now.
    pub urgent: bool,
    /// The flight is on the arrival side — drives the panel's phrase-group
    /// highlight (false = departure side).
    pub arrival: bool,
}

/// The call the pilot SHOULD make next, derived from the tower's own state
/// machine — the copilot as radio coach, and the panel's Calls tab. The
/// suggestions are deliberately the parser's happy path: following the
/// coach always routes. `com1_khz` is the live radio: a suggestion the
/// station can't hear gets a "first tune ..." line ahead of it. `None` =
/// no station to talk to.
pub fn suggest_call(
    callsign: &str,
    on_ground: bool,
    dist_nm: Option<f64>,
    com1_khz: Option<i64>,
) -> Option<Suggestion> {
    let (station, services) = {
        let p = ACTIVE_PANEL.lock().unwrap_or_else(|e| e.into_inner());
        let (name, _, svcs) = p.as_ref()?;
        (spoken_airport(name).to_string(), svcs.clone())
    };
    let clr = CLEARANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.runway_ident.clone(), c.pending_readback.clone()));
    let state = clr.as_ref().map(|(s, _, _)| *s);
    let arrival = !on_ground
        || matches!(
            state,
            Some(tower::State::Inbound | tower::State::ClearedToLand)
        );
    // A readback owed outranks everything: it IS the next call — and it was
    // just received, so the radio is by definition on the right frequency.
    if let Some(rb) = clr.as_ref().and_then(|(_, _, rb)| rb.clone()) {
        return Some(Suggestion {
            first: None,
            call: Some(rb),
            urgent: true,
            arrival,
        });
    }
    // "First tune <service> ..." when the suggested call goes to a
    // frequency COM1 isn't on — a perfect call the station can't hear is
    // the coach's own failure mode. Skipped when another `first` already
    // leads (ATIS, vectors status): one prerequisite at a time.
    let tune_first = |svc: &str| -> Option<String> {
        let com1 = com1_khz?;
        let (_, khz) = services.iter().find(|(w, _)| w == svc)?;
        ((com1 - khz).abs() > 5).then(|| format!("First tune {svc}, {}.", atis::spoken_mhz(*khz)))
    };
    // Approach's side of the picture, airborne only (a plan on the ground is
    // stale — vector_tick clears it on touchdown).
    if !on_ground {
        // Handed to the tower with an APPROACH clearance: the "final" report
        // is owed before any landing clearance — the same fact the monitor's
        // voice coach reads.
        if let Some(rwy) = LANDING_HANDOFF
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
        {
            return Some(Suggestion {
                first: tune_first("tower"),
                call: Some(format!("{station} Tower, {callsign}, final runway {rwy}.")),
                urgent: false,
                arrival: true,
            });
        }
        if let Some((ils_cleared, rwy)) = VECTORS
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map(|p| (p.ils && p.cleared, p.runway_spoken.clone()))
        {
            // Cleared for the ILS on the intercept: the one call approach
            // waits on is "established". Before that, radar vectors are a
            // listening game — coach the silence, not an invented call.
            return Some(if ils_cleared {
                Suggestion {
                    first: Some("Report when established on the localizer.".to_string()),
                    call: Some(format!("{callsign}, established, runway {rwy}.")),
                    urgent: false,
                    arrival: true,
                }
            } else {
                Suggestion {
                    first: Some(
                        "Under radar vectors. No call owed; follow the headings.".to_string(),
                    ),
                    call: None,
                    urgent: false,
                    arrival: true,
                }
            });
        }
    }
    let info = atis_current_letter()
        .map(|l| format!(", with information {}", atis::nato(l)))
        .unwrap_or_default();
    // An ATIS that has never cut an edition is an ATIS nobody has listened
    // to (editions cut while COM1 sits on its frequency) — before the first
    // call to a fresh station, the better first move is to go copy it.
    let atis_first = || {
        let a = ATIS.lock().unwrap_or_else(|e| e.into_inner());
        a.as_ref()
            .filter(|a| a.edition.is_none())
            .map(|a| format!("First copy the ATIS on {}.", atis::spoken_mhz(a.khz)))
    };
    let rwy = clr
        .as_ref()
        .map(|(_, id, _)| id.as_str())
        .filter(|s| !s.is_empty())
        .map(atis::spoken_runway);
    let pos = dist_nm
        .filter(|d| *d >= 2.0)
        .map(|d| format!("{:.0} miles out, ", d))
        .unwrap_or_default();
    let call = match (state, on_ground) {
        // Never talked to this station: the full initial call.
        (None, true) => {
            format!("{station} Tower, {callsign}{info}, request departure clearance.")
        }
        (None, false) | (Some(tower::State::Parked), false) => {
            format!("{station} Tower, {callsign}, {pos}inbound{info}, request join.")
        }
        (Some(tower::State::Parked), true) => format!("{callsign}, request taxi."),
        // Taxi clearance in hand: report ready at the REAL holding point —
        // the cleared route's destination, when the taxi network knows it.
        (Some(tower::State::TaxiCleared), true) => {
            let hp = TAXI_ROUTE
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .as_ref()
                .map(|r| spoken_taxiway(&r.holding_point));
            match (hp, &rwy) {
                (Some(h), Some(r)) => {
                    format!("{callsign}, holding point {h}, runway {r}, ready for departure.")
                }
                (Some(h), None) => format!("{callsign}, holding point {h}, ready for departure."),
                _ => format!("{callsign}, ready for departure."),
            }
        }
        // Lined up (readback already paid): the clearance comes to you —
        // a nudge routes as a fresh ask if you want it.
        (Some(tower::State::LinedUp), true) => {
            format!("{callsign}, ready for immediate departure.")
        }
        // A held/issued clearance wants its readback.
        (Some(tower::State::TakeoffCleared), true) => match &rwy {
            Some(r) => format!("Cleared for takeoff runway {r}, {callsign}."),
            None => format!("Cleared for takeoff, {callsign}."),
        },
        // Airborne after departure: leave, or work the circuit.
        (Some(tower::State::TakeoffCleared), false)
        | (Some(tower::State::TaxiCleared), false)
        | (Some(tower::State::LinedUp), false) => {
            format!("{callsign}, request frequency change. Or report: downwind.")
        }
        (Some(tower::State::Inbound), false) => match &rwy {
            Some(r) => format!("{callsign}, final runway {r}."),
            None => format!("{callsign}, final."),
        },
        (Some(tower::State::ClearedToLand), false) => match &rwy {
            Some(r) => format!("Cleared to land runway {r}, {callsign}."),
            None => format!("Cleared to land, {callsign}."),
        },
        // On the ground as an arrival: get off the runway and home.
        (Some(tower::State::ClearedToLand), true) | (Some(tower::State::Inbound), true) => {
            format!("{callsign}, runway vacated, request taxi to parking.")
        }
    };
    // One prerequisite at a time: the first-ever call prefers the ATIS (the
    // chronological next action — once in contact the tower's own QNH
    // covers you), everything else the frequency check.
    let first = match state {
        None => atis_first().or_else(|| tune_first("tower")),
        _ => tune_first("tower"),
    };
    Some(Suggestion {
        first,
        call: Some(call),
        urgent: false,
        arrival,
    })
}

/// The active field for the traffic watches: spoken name, lat, lon, field
/// elevation ft. `None` = no station.
pub fn active_field() -> Option<(String, f64, f64, f64)> {
    let (name, lat, lon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    let elev = ACTIVE_PANEL
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(_, e, _)| *e)
        .unwrap_or(0.0);
    Some((spoken_airport(&name).to_string(), lat, lon, elev))
}

/// A readback of the region's handoff, made on the frequency it came on, is
/// heard by the controller who gave it even though that station has already
/// been swapped for the next: it is not a call into the void. Six flights out
/// of six logged "radio call into the VOID" for "squawking seven thousand,
/// Carlisle Tower on ..." and the debrief listed each as a lost call.
const HANDOFF_READBACK_WINDOW_SECS: u64 = 45;

fn void_outcome(transcript: &str, now: Instant) -> &'static str {
    let recent = LAST_REGION_SWITCH
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some_and(|t| now.saturating_duration_since(t).as_secs() <= HANDOFF_READBACK_WINDOW_SECS);
    if recent && tower::is_readback(transcript) {
        "handoff_readback"
    } else {
        "void"
    }
}

/// The active field's SURFACE wind as the app last read it from the sim's
/// real-weather METAR file: (field lat, lon, from deg true, kts). The station
/// cannot read weather files (its sim handle is the read-only dataref view),
/// so the app hands the report down, the way it hands the ATIS its weather.
/// Keyed by POSITION: a station knows where it is, not what it is called.
static SURFACE_WIND: Mutex<Option<(f64, f64, f64, f64)>> = Mutex::new(None);

/// A published wind is this field's when it was read for a point this near.
const SURFACE_WIND_FIELD_NM: f64 = 3.0;

/// Publish (or retire, with `None`) the surface wind of the field at
/// `lat`/`lon`.
pub fn set_surface_wind(lat: f64, lon: f64, wind: Option<(f64, f64)>) {
    *SURFACE_WIND.lock().unwrap_or_else(|e| e.into_inner()) =
        wind.map(|(from, kts)| (lat, lon, from, kts));
}

/// The published surface wind, only if it is THIS field's.
fn surface_wind_at(pos: Option<(f64, f64)>) -> Option<(f64, f64)> {
    let (lat, lon) = pos?;
    SURFACE_WIND
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(wlat, wlon, _, _)| nav::haversine_nm(lat, lon, *wlat, *wlon) <= SURFACE_WIND_FIELD_NM)
        .map(|(_, _, from, kts)| (*from, *kts))
}

/// The active station's ICAO and position, from the region index (the
/// station itself carries a name, not an identifier): for the app's METAR
/// lookup. `None` before the index has loaded or with no station up.
pub fn active_icao() -> Option<(String, f64, f64)> {
    let (_, lat, lon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    REGION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|a| nav::haversine_nm(lat, lon, a.lat, a.lon) < 0.5)
        .map(|a| (a.icao.clone(), lat, lon))
}

/// What the party line remembers about a ghost between passes.
struct GhostMemory {
    last_seen: Instant,
    prev_ele_ft: f64,
    prev_ground: bool,
    /// Last tick's position — the roll direction at rotation, so a narrated
    /// takeoff names the runway END the ghost is departing FROM, not the
    /// threshold it happens to be nearest (liftoff is past midfield: every
    /// 09R departure read "two seven left" on the 2026-07-10 tape).
    prev_lat: f64,
    prev_lon: f64,
    announced_landing: bool,
    announced_takeoff: bool,
    /// "Continue approach" spoken (its aim runway was blocked by the player).
    announced_continue: bool,
}

static GHOSTS: Mutex<Option<std::collections::HashMap<String, GhostMemory>>> = Mutex::new(None);

/// One line of a party-line exchange: who says it, and what.
pub struct PartyLine {
    /// Spoken by the GHOST pilot (its own pooled voice); false = the tower.
    pub ghost: bool,
    pub text: String,
}

/// A narrated exchange — the ghost pilot's call, the tower's reply, the
/// readback — queued in speaking order through the one arbiter.
pub struct PartyExchange {
    pub lines: Vec<PartyLine>,
    /// Stable voice seed for this ghost (hash of its callsign): Speedbird
    /// 306 is the same person on every call.
    pub ghost_seed: usize,
}

/// The strip the PLAYER holds for the party line: the one it stands on, or,
/// on the ground, the one the tower's book still gives it (cleared onto it,
/// or landed and not yet reported off). A ghost landing or departing that
/// strip is not narrated a clearance over our head. The "runway vacated"
/// report frees it, as it frees a real tower's runway, before the taxi
/// request that used to be the only thing that did (Liege 2026-09-22).
pub fn player_blocked_strip(on_ground: bool, lat: f64, lon: f64) -> Option<String> {
    if !on_ground {
        return None;
    }
    on_active_runway(lat, lon).or_else(|| {
        CLEARANCE
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .filter(|c| c.holds_runway && !c.runway_ident.is_empty())
            .map(|c| c.runway_ident.clone())
    })
}

/// What OUR clearance claims of a strip, for the party line, on the ground
/// or in the air: `(end, whole)`. Cleared to land and not yet reported off,
/// the WHOLE strip is ours (nobody else is cleared onto it, either end);
/// sequenced for it in any other way (taxi, line-up, takeoff, the circuit's
/// Inbound, a go-around's), its DIRECTION is (nobody is cleared off or onto
/// its reciprocal). `None` without a clearance or a runway.
fn own_strip_claim() -> Option<(String, bool)> {
    let clr = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner());
    let c = clr.as_ref().filter(|c| !c.runway_ident.is_empty())?;
    match c.state {
        tower::State::ClearedToLand if c.holds_runway => Some((c.runway_ident.clone(), true)),
        // Lined up or cleared for takeoff, the runway is ours alone: one
        // departure on it at a time, either direction (EGKK 08R, sortie
        // 1790601822-full_auto, 2026-09-28: "Easy eight six eight zero,
        // runway zero eight right, cleared for takeoff" 1.6 s after our own
        // takeoff clearance, while we lined up).
        tower::State::LinedUp | tower::State::TakeoffCleared => {
            Some((c.runway_ident.clone(), true))
        }
        tower::State::TaxiCleared | tower::State::Inbound => {
            Some((c.runway_ident.clone(), false))
        }
        _ => None,
    }
}

/// Our own departure is still spacing the strip `ghost_end` is on: we lifted
/// off it less than [`WAKE_SECS`] ago, so the tower has cleared nobody off it
/// behind us (the wake and departure spacing a real tower gives the next
/// departure). Read from the compliance watch's memory of our liftoff.
fn own_departure_spacing(ghost_end: &str, now: Instant) -> bool {
    let Some((ours, at)) = COMPLIANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .liftoff
        .clone()
    else {
        return false;
    };
    if now.saturating_duration_since(at).as_secs() >= WAKE_SECS {
        return false;
    }
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    same_strip_or_twin(&runways, ghost_end, &ours)
}

/// Two runway ends on one strip, or on a strip and its close-parallel main
/// twin (`nav::close_parallel_main`): the replay puts a real 08R departure on
/// EGKK's 08L pavement 190 m away, and a wake clock kept on "26R" never
/// matched the held 08R (sortie 1790601822-full_auto, 2026-09-28).
fn same_strip_or_twin(runways: &[nav::Runway], a: &str, b: &str) -> bool {
    let a = nav::close_parallel_main(runways, a);
    let b = nav::close_parallel_main(runways, b);
    matches!(
        (strip(runways, &a), strip(runways, &b)),
        (Some(x), Some(y)) if std::ptr::eq(x, y)
    )
}

/// How recently the party line must have seen a ghost for a change in it to
/// be narrated as happening now. The chatter is not ticked while a release is
/// held or for 45 s after an exchange, and a liftoff remembered across that
/// gap was narrated 17 s late as a fresh clearance ("Easy eight six eight
/// zero, cleared for takeoff" at 13:40:48 for U28680 airborne at 13:40:31,
/// EGKK 2026-09-28).
const PARTY_LINE_FRESH_S: f64 = 5.0;

/// Whether a ghost's movement on `ghost_end` conflicts with our claim: the
/// same strip, and either the whole strip is ours or the ghost uses the
/// other direction. One strip, one direction (LEMG 2026-09-28).
fn conflicts_with_claim(ghost_end: &str, claim: &(String, bool)) -> bool {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    let same = matches!(
        (strip(&runways, ghost_end), strip(&runways, &claim.0)),
        (Some(x), Some(y)) if std::ptr::eq(x, y)
    );
    same && (claim.1 || ghost_end != claim.0)
}

/// One pass of the party line: narrate at most ONE exchange for the ghost
/// traffic — an arrival's final call answered with the landing clearance
/// while the ghost is still descending (spoken BEFORE reality lands it), a
/// takeoff clearance and readback as one rotates. Each ghost gets each
/// exchange once. The tower is never wrong: the "instruction" is derived
/// from what the replay is about to do anyway.
pub fn chatter_tick(
    targets: &[TcasTarget],
    station_name: &str,
    field_lat: f64,
    field_lon: f64,
    field_elev_ft: f64,
    // The runway END the PLAYER currently occupies (or holds a clearance
    // for): a ghost landing that strip gets "continue approach", never a
    // narrated "cleared to land" over the player's head — the 2026-07-10
    // tape cleared a takeoff and a landing on 09L eight seconds apart.
    blocked_ident: Option<&str>,
    now: Instant,
) -> Option<PartyExchange> {
    if station_unavailable() {
        return None;
    }
    // Read before the ghost memory's lock: our own clearance's claim.
    let claim = own_strip_claim();
    let mut guard = GHOSTS.lock().unwrap_or_else(|e| e.into_inner());
    let mem = guard.get_or_insert_with(std::collections::HashMap::new);
    mem.retain(|_, m| now.duration_since(m.last_seen).as_secs() < 180);
    let mut exchange = None;
    for t in targets {
        let dist = crate::nav::haversine_nm(t.lat, t.lon, field_lat, field_lon);
        if dist > 8.0 {
            continue; // beyond this tower's world
        }
        let m = mem.entry(t.callsign.clone()).or_insert(GhostMemory {
            last_seen: now,
            prev_ele_ft: t.ele_ft,
            prev_ground: t.on_ground,
            prev_lat: t.lat,
            prev_lon: t.lon,
            announced_landing: false,
            announced_takeoff: false,
            announced_continue: false,
        });
        let descending = t.ele_ft < m.prev_ele_ft - 1.0;
        // A change seen across a gap in the watch is history, not news.
        let fresh = now.saturating_duration_since(m.last_seen).as_secs_f64() <= PARTY_LINE_FRESH_S;
        let was_ground = m.prev_ground;
        let agl = t.ele_ft - field_elev_ft;
        if exchange.is_none()
            && !t.on_ground
            && !m.announced_landing
            && dist < 5.0
            && (150.0..1200.0).contains(&agl)
            && descending
        {
            let cs = spoken_ghost(&t.callsign);
            let aim = nearest_runway_ident(t.lat, t.lon).map(|id| main_runway_of(&id));
            // The player is on (or cleared onto) this ghost's strip: no
            // narrated landing clearance over their head — "continue" is
            // what a real tower says, and it stays true when the replay
            // lands anyway. The clearance may still come once the player
            // vacates and the ghost is re-seen clear.
            let blocked = match (&aim, blocked_ident) {
                (Some(a), Some(b)) => {
                    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
                    matches!(
                        (strip(&runways, a), strip(&runways, b)),
                        (Some(x), Some(y)) if std::ptr::eq(x, y)
                    )
                }
                _ => false,
            };
            if blocked {
                if !m.announced_continue {
                    m.announced_continue = true;
                    exchange = Some(PartyExchange {
                        lines: vec![
                            PartyLine {
                                ghost: true,
                                text: format!(
                                    "{station_name} Tower, {cs}, final runway {}.",
                                    aim.as_deref().map(atis::spoken_runway).unwrap_or_default()
                                ),
                            },
                            PartyLine {
                                ghost: false,
                                text: format!(
                                    "{cs}, continue approach, traffic on the runway ahead."
                                ),
                            },
                        ],
                        ghost_seed: crate::speaker::station_voice_seed(&t.callsign),
                    });
                }
                m.last_seen = now;
                m.prev_ele_ft = t.ele_ft;
                m.prev_ground = t.on_ground;
                m.prev_lat = t.lat;
                m.prev_lon = t.lon;
                continue;
            }
            // Our clearance claims this strip (cleared to land on it, or
            // sequenced the other way): the tower narrates no clearance it
            // could not have given. Silent, not "continue": the ghost may
            // be the one flying against us, and the truth about it is the
            // go-around watch's, not flavour. The clearance may still come
            // once the claim is gone and the ghost is re-seen in the band.
            if let (Some(a), Some(c)) = (&aim, &claim) {
                if conflicts_with_claim(a, c) {
                    sim::blackbox::record(
                        "atc",
                        serde_json::json!({"event": "party_line_withheld", "ghost": t.callsign,
                            "call": "landing", "runway": a, "ours": c.0, "whole": c.1}),
                    );
                    m.last_seen = now;
                    m.prev_ele_ft = t.ele_ft;
                    m.prev_ground = t.on_ground;
                    m.prev_lat = t.lat;
                    m.prev_lon = t.lon;
                    continue;
                }
            }
            m.announced_landing = true;
            let rwy = aim.as_deref().map(atis::spoken_runway);
            let lines = match &rwy {
                Some(r) => vec![
                    PartyLine {
                        ghost: true,
                        text: format!("{station_name} Tower, {cs}, final runway {r}."),
                    },
                    PartyLine {
                        ghost: false,
                        text: format!("{cs}, runway {r}, cleared to land."),
                    },
                    PartyLine {
                        ghost: true,
                        text: format!("Cleared to land runway {r}, {cs}."),
                    },
                ],
                None => vec![
                    PartyLine {
                        ghost: true,
                        text: format!("{station_name} Tower, {cs}, final."),
                    },
                    PartyLine {
                        ghost: false,
                        text: format!("{cs}, cleared to land."),
                    },
                ],
            };
            exchange = Some(PartyExchange {
                lines,
                ghost_seed: crate::speaker::station_voice_seed(&t.callsign),
            });
        }
        if exchange.is_none() && was_ground && !t.on_ground && !m.announced_takeoff && dist < 3.0
            && !fresh
        {
            // Lifted off while nobody was watching: spent, never narrated late.
            m.announced_takeoff = true;
        }
        if exchange.is_none() && was_ground && !t.on_ground && !m.announced_takeoff && dist < 3.0 {
            m.announced_takeoff = true;
            let cs = spoken_ghost(&t.callsign);
            // Name the end the ghost DEPARTED, by its roll direction —
            // liftoff is past midfield, so the nearest threshold is the
            // reciprocal (every 09R departure read "two seven left" on the
            // 2026-07-10 tape).
            let ident = departure_runway_ident(t.lat, t.lon, m.prev_lat, m.prev_lon)
                .or_else(|| nearest_runway_ident(t.lat, t.lon))
                .map(|id| main_runway_of(&id));
            // The player holds this strip: the tower gave nobody else a
            // takeoff on it, so the replay's departure goes un-narrated.
            let held = match (&ident, blocked_ident) {
                (Some(a), Some(b)) => {
                    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
                    matches!(
                        (strip(&runways, a), strip(&runways, b)),
                        (Some(x), Some(y)) if std::ptr::eq(x, y)
                    )
                }
                _ => false,
            };
            // In the air too, and by direction: nobody is cleared off the
            // strip we are cleared to land on, nor off its reciprocal while
            // we are sequenced for it (LEMG, sortie 1790584715-full_auto:
            // "Vueling eight one seven six, runway one three, cleared for
            // takeoff" with our circuit on downwind for 31, "Easy two seven
            // one six" after our go-around from the 31 final).
            let claimed = match (&ident, &claim) {
                (Some(a), Some(c)) => conflicts_with_claim(a, c),
                _ => false,
            } || ident.as_deref().is_some_and(|a| own_departure_spacing(a, now));
            if claimed {
                sim::blackbox::record(
                    "atc",
                    serde_json::json!({"event": "party_line_withheld", "ghost": t.callsign,
                        "call": "takeoff", "runway": ident, "ours": claim.as_ref().map(|c| c.0.clone()),
                        "whole": claim.as_ref().map(|c| c.1)}),
                );
            }
            if held || claimed {
                m.last_seen = now;
                m.prev_ele_ft = t.ele_ft;
                m.prev_ground = t.on_ground;
                m.prev_lat = t.lat;
                m.prev_lon = t.lon;
                continue;
            }
            let clearance = match ident {
                Some(id) => format!(
                    "{cs}, runway {}, cleared for takeoff.",
                    atis::spoken_runway(&id)
                ),
                None => format!("{cs}, cleared for takeoff."),
            };
            exchange = Some(PartyExchange {
                lines: vec![
                    PartyLine {
                        ghost: false,
                        text: clearance,
                    },
                    PartyLine {
                        ghost: true,
                        text: format!("Cleared for takeoff, {cs}."),
                    },
                ],
                ghost_seed: crate::speaker::station_voice_seed(&t.callsign),
            });
        }
        m.last_seen = now;
        m.prev_ele_ft = t.ele_ft;
        m.prev_ground = t.on_ground;
        m.prev_lat = t.lat;
        m.prev_lon = t.lon;
    }
    exchange
}

/// The go-around order was last spoken at — once per half minute is urgency,
/// not stutter.
static GO_AROUND_LAST: Mutex<Option<Instant>> = Mutex::new(None);

/// Our recent airborne positions (lat, lon, when), from the monitor's
/// [`note_aircraft`]: the go-around judgement's distance and time to the
/// threshold. Cleared on the ground.
static OWN_FIXES: Mutex<std::collections::VecDeque<(f64, f64, Instant)>> =
    Mutex::new(std::collections::VecDeque::new());

/// Recent positions of ghosts on the ground, by callsign: how a strip's
/// occupant is MOVING (rolling, vacating, standing) is what says whether it
/// will still be there when we arrive.
type GroundTracks =
    std::collections::HashMap<String, std::collections::VecDeque<(f64, f64, Instant)>>;
static STRIP_SEEN: Mutex<Option<GroundTracks>> = Mutex::new(None);

/// How much track history the judgements keep.
const TRACK_KEEP_S: f64 = 8.0;
/// A velocity is measured over at least this long (a tick-to-tick
/// difference is noise).
const TRACK_MIN_DT_S: f64 = 1.0;

/// Record our own position for the go-around judgement. Airborne only: on
/// the ground there is no threshold to reach.
pub(crate) fn note_own_fix(lat: f64, lon: f64, on_ground: bool, now: Instant) {
    let mut f = OWN_FIXES.lock().unwrap_or_else(|e| e.into_inner());
    if on_ground {
        f.clear();
        return;
    }
    if f.back()
        .is_some_and(|b| now.saturating_duration_since(b.2).as_secs_f64() < 0.05)
    {
        return;
    }
    f.push_back((lat, lon, now));
    while f
        .front()
        .is_some_and(|a| now.saturating_duration_since(a.2).as_secs_f64() > TRACK_KEEP_S)
    {
        f.pop_front();
    }
}

/// The tower decides a go-around only inside this time to the threshold: a
/// runway occupant further out than that is a sequencing matter ("continue
/// approach"), never a cancellation. About two miles at approach speed.
const GO_AROUND_DECIDE_S: f64 = 60.0;
/// The runway must be clear this long before we cross the threshold.
const RUNWAY_CLEAR_MARGIN_S: f64 = 10.0;
/// Groundspeed assumed on final when ours is not measured.
const DEFAULT_FINAL_KTS: f64 = 140.0;
/// A departure's roll: it lifts off near this speed, accelerating at this
/// rate (a jet from brake release lifts in about 35 s).
const LIFTOFF_KTS: f64 = 150.0;
const ROLL_ACCEL_KTS_S: f64 = 4.0;
/// A lander ahead rolling out (fast, not accelerating) is off at a turn-off
/// within this.
const ROLLOUT_CLEAR_S: f64 = 40.0;
/// A ghost standing still this close to the threshold we land on is a
/// departure lined up: it is off the runway a roll after it starts.
const LINED_UP_ZONE_M: f64 = 800.0;
/// Under this a ghost is standing, not moving.
const MOVING_KTS: f64 = 3.0;
/// An occupant is ON the runway when its reported position is within the
/// paved half-width plus this: a mainwheel on the edge line, never the
/// holding point (90 m out on a code C strip). The runway judgement asks the
/// PAVEMENT, not the incursion watch's wider capture (`on_runway`, 10 m of
/// slack and 2 % of the length past each end). Issue #162.
const OCCUPANT_EDGE_M: f64 = 5.0;
/// Past either END of the pavement an occupant still counts this far: a
/// displaced threshold or an overrun, not a blast pad or a taxiway on the
/// extended centreline (2 % of LEMG's 3,200 m was 64 m).
const OCCUPANT_END_M: f64 = 30.0;
/// A ghost within this of where it stopped is still standing (the feed's
/// position noise, a nudge forward in the line-up).
const STANDING_RADIUS_M: f64 = 15.0;
/// A ghost standing on a runway longer than this is stale data, not an
/// occupant: no real aircraft holds a runway for five minutes, a frozen
/// LiveTraffic target does. Logged `ghost_stale` once per standing spell.
/// Issue #162.
const GHOST_STALE_S: f64 = 300.0;
/// A standing clock survives this long without a sighting (the traffic read
/// is only made while a watch needs it); a ghost seen again where it stood
/// has stood all along.
const GHOST_MEMORY_S: f64 = 900.0;
/// Another runway offered after a go-around must be at least this share of
/// the blocked strip's length: the tower knows the runway the flight chose,
/// not the aircraft's numbers, so it never offers a much shorter one.
const RESEQUENCE_MIN_LENGTH_RATIO: f64 = 0.75;

/// How a runway occupant will leave the strip, judged from its track.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum StripExit {
    /// Moving off it: a departure rolling our way, the lander ahead rolling
    /// out, an aircraft crossing or vacating. Off in about this many seconds.
    Clearing(f64),
    /// Standing at the threshold we land on: a departure lined up, off one
    /// roll (this many seconds) after it starts.
    LinedUp(f64),
    /// Standing elsewhere, or unknown: no telling when.
    Stays,
    /// Moving TOWARD us along the strip: traffic using the other direction
    /// (a lander rolling out, a departure rolling, a backtrack). No telling
    /// when it is off, and the runway's direction is contested: the
    /// re-sequence after the go-around offers the other end
    /// ([`resequence_runway`]).
    Oncoming,
}

impl StripExit {
    fn secs(&self) -> Option<f64> {
        match self {
            StripExit::Clearing(s) | StripExit::LinedUp(s) => Some(*s),
            StripExit::Stays | StripExit::Oncoming => None,
        }
    }
}

/// Seconds for a departure rolling at `kts` to lift off.
fn roll_to_liftoff_s(kts: f64) -> f64 {
    (LIFTOFF_KTS - kts).max(0.0) / ROLL_ACCEL_KTS_S
}

/// Half the strip's width for occupancy: the pavement plus
/// [`OCCUPANT_EDGE_M`]. A row without a width keeps the blanket capture.
fn strip_half_width_m(r: &nav::Runway) -> f64 {
    if r.width_m > 1.0 {
        r.width_m / 2.0 + OCCUPANT_EDGE_M
    } else {
        RUNWAY_HALF_WIDTH_M
    }
}

/// Is this position ON the runway's pavement, for the landing judgement?
/// Laterally within [`strip_half_width_m`] of the centreline, lengthwise
/// between the ends with [`OCCUPANT_END_M`] past each. Returns (metres from
/// the `ident_a` end, metres off the centreline). Issue #162: "on the
/// runway" is the pavement, never the strip's area or its hold line.
pub(crate) fn on_runway_pavement(r: &nav::Runway, lat: f64, lon: f64) -> Option<(f64, f64)> {
    let m_lat = 111_320.0;
    let m_lon = 111_320.0 * r.lat_a.to_radians().cos();
    let (px, py) = ((lon - r.lon_a) * m_lon, (lat - r.lat_a) * m_lat);
    let (bx, by) = ((r.lon_b - r.lon_a) * m_lon, (r.lat_b - r.lat_a) * m_lat);
    let len = (bx * bx + by * by).sqrt();
    if len < 1.0 {
        return None;
    }
    let along = (px * bx + py * by) / len;
    let cross = (px * by - py * bx).abs() / len;
    ((-OCCUPANT_END_M..=len + OCCUPANT_END_M).contains(&along) && cross <= strip_half_width_m(r))
        .then_some((along, cross))
}

/// Where a ghost on the ground stopped, and since when: the stale-ghost
/// clock ([`GHOST_STALE_S`]).
struct Standing {
    lat: f64,
    lon: f64,
    since: Instant,
    seen: Instant,
    /// `ghost_stale` was recorded for this standing spell.
    stale_logged: bool,
}

static STANDING: Mutex<Option<std::collections::HashMap<String, Standing>>> = Mutex::new(None);

/// Flat-earth metres between two nearby positions.
fn metres_between(lat0: f64, lon0: f64, lat1: f64, lon1: f64) -> f64 {
    let m_lon = 111_320.0 * lat0.to_radians().cos();
    let (dx, dy) = ((lon1 - lon0) * m_lon, (lat1 - lat0) * 111_320.0);
    (dx * dx + dy * dy).sqrt()
}

/// Note every ghost's position: the short tracks the runway judgement reads
/// its motion from (airborne too, so a lander's first sample on the wheels
/// already has a velocity: at first sight a touchdown was "standing"), and
/// each ground ghost's standing clock. Idempotent within a tick; fed by
/// every watch that reads the traffic.
fn note_ground_traffic(targets: &[TcasTarget], now: Instant) {
    {
        let mut guard = STRIP_SEEN.lock().unwrap_or_else(|e| e.into_inner());
        let seen = guard.get_or_insert_with(std::collections::HashMap::new);
        for t in targets {
            let tr = seen.entry(t.callsign.clone()).or_default();
            if tr
                .back()
                .is_none_or(|b| now.saturating_duration_since(b.2).as_secs_f64() >= 0.05)
            {
                tr.push_back((t.lat, t.lon, now));
            }
            while tr
                .front()
                .is_some_and(|a| now.saturating_duration_since(a.2).as_secs_f64() > TRACK_KEEP_S)
            {
                tr.pop_front();
            }
        }
        seen.retain(|_, tr| {
            tr.back()
                .is_some_and(|b| now.saturating_duration_since(b.2).as_secs_f64() <= TRACK_KEEP_S)
        });
    }
    let mut guard = STANDING.lock().unwrap_or_else(|e| e.into_inner());
    let standing = guard.get_or_insert_with(std::collections::HashMap::new);
    for t in targets {
        if !t.on_ground {
            standing.remove(&t.callsign);
            continue;
        }
        let fresh = Standing {
            lat: t.lat,
            lon: t.lon,
            since: now,
            seen: now,
            stale_logged: false,
        };
        match standing.get_mut(&t.callsign) {
            Some(s)
                if metres_between(s.lat, s.lon, t.lat, t.lon) <= STANDING_RADIUS_M
                    && now.saturating_duration_since(s.seen).as_secs_f64() <= GHOST_MEMORY_S =>
            {
                s.seen = s.seen.max(now);
            }
            _ => {
                standing.insert(t.callsign.clone(), fresh);
            }
        }
    }
    standing.retain(|_, s| now.saturating_duration_since(s.seen).as_secs_f64() <= GHOST_MEMORY_S);
}

/// How long this ghost has stood where it is, seconds (0 when moving or
/// unknown).
fn standing_secs(callsign: &str, now: Instant) -> f64 {
    STANDING
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .and_then(|m| m.get(callsign))
        .map(|s| now.saturating_duration_since(s.since).as_secs_f64())
        .unwrap_or(0.0)
}

/// Is this ghost stale data: standing on a runway past [`GHOST_STALE_S`]?
/// The first time per standing spell it is recorded (`ghost_stale`).
fn ghost_stale(t: &TcasTarget, runway: &str, place: (f64, f64), now: Instant) -> bool {
    let mut guard = STANDING.lock().unwrap_or_else(|e| e.into_inner());
    let Some(s) = guard.as_mut().and_then(|m| m.get_mut(&t.callsign)) else {
        return false;
    };
    let secs = now.saturating_duration_since(s.since).as_secs_f64();
    if secs < GHOST_STALE_S {
        return false;
    }
    if !s.stale_logged {
        s.stale_logged = true;
        sim::blackbox::record(
            "atc",
            serde_json::json!({"event": "ghost_stale", "ghost": t.callsign, "runway": runway,
                "along_m": place.0.round(), "cross_m": place.1.round(),
                "standing_s": secs.round(), "lat": t.lat, "lon": t.lon}),
        );
    }
    true
}

/// A ghost that OCCUPIES this runway for the landing judgement: on the
/// ground, on the pavement ([`on_runway_pavement`]), and not stale data
/// ([`ghost_stale`]). Returns its place on the pavement.
fn occupies(r: &nav::Runway, t: &TcasTarget, now: Instant) -> Option<(f64, f64)> {
    if !t.on_ground {
        return None;
    }
    let place = on_runway_pavement(r, t.lat, t.lon)?;
    (!ghost_stale(t, &r.ident_a, place, now)).then_some(place)
}

/// A frame on the landing END: origin its threshold, `u` along the landing
/// direction; positions in metres, along and to the left.
struct LandingFrame {
    lat0: f64,
    lon0: f64,
    m_lat: f64,
    m_lon: f64,
    u: (f64, f64),
}

impl LandingFrame {
    fn new(r: &nav::Runway, landing_end: &str) -> Option<Self> {
        let (lat0, lon0, lat1, lon1) = if r.ident_a == landing_end {
            (r.lat_a, r.lon_a, r.lat_b, r.lon_b)
        } else if r.ident_b == landing_end {
            (r.lat_b, r.lon_b, r.lat_a, r.lon_a)
        } else {
            return None;
        };
        let m_lat = 111_320.0;
        let m_lon = 111_320.0 * lat0.to_radians().cos();
        let (bx, by) = ((lon1 - lon0) * m_lon, (lat1 - lat0) * m_lat);
        let len = (bx * bx + by * by).sqrt();
        if len < 1.0 {
            return None;
        }
        Some(Self {
            lat0,
            lon0,
            m_lat,
            m_lon,
            u: (bx / len, by / len),
        })
    }

    /// Metres east and north of the threshold.
    fn xy(&self, lat: f64, lon: f64) -> (f64, f64) {
        (
            (lon - self.lon0) * self.m_lon,
            (lat - self.lat0) * self.m_lat,
        )
    }

    /// (along, cross) of a position, metres.
    fn place(&self, lat: f64, lon: f64) -> (f64, f64) {
        let (x, y) = self.xy(lat, lon);
        (x * self.u.0 + y * self.u.1, -x * self.u.1 + y * self.u.0)
    }
}

/// The velocity (along, cross; m/s) between sample `newest` and the latest
/// one at least [`TRACK_MIN_DT_S`] older, with that older sample's index.
fn track_velocity(
    f: &LandingFrame,
    track: &[(f64, f64, f64)],
    newest: usize,
) -> Option<((f64, f64), usize)> {
    let (lat, lon, t) = track[newest];
    let i = (0..newest)
        .rev()
        .find(|&i| t - track[i].2 >= TRACK_MIN_DT_S)?;
    let (a1, c1) = f.place(lat, lon);
    let (a0, c0) = f.place(track[i].0, track[i].1);
    let dt = t - track[i].2;
    Some((((a1 - a0) / dt, (c1 - c0) / dt), i))
}

/// How a ghost on our strip will leave it, from its track (lat, lon, seconds;
/// oldest first, the last is now). Pure: the go-around judgement's and the
/// landing release's one question.
pub(crate) fn strip_exit(
    r: &nav::Runway,
    landing_end: &str,
    track: &[(f64, f64, f64)],
) -> StripExit {
    let Some(f) = LandingFrame::new(r, landing_end) else {
        return StripExit::Stays;
    };
    let Some(&(lat, lon, _)) = track.last() else {
        return StripExit::Stays;
    };
    let (along, cross) = f.place(lat, lon);
    let standing = || {
        if along < LINED_UP_ZONE_M {
            StripExit::LinedUp(roll_to_liftoff_s(0.0))
        } else {
            StripExit::Stays
        }
    };
    let Some(((va, vc), older)) = track_velocity(&f, track, track.len() - 1) else {
        // First sight: standing until it is seen to move.
        return standing();
    };
    let kts = (va * va + vc * vc).sqrt() * 1.943_844;
    if kts < MOVING_KTS {
        return standing();
    }
    let angle = vc.abs().atan2(va.abs()).to_degrees();
    if angle <= 30.0 {
        if va <= 0.0 {
            // Toward us along the strip: a backtrack, or the other direction.
            return StripExit::Oncoming;
        }
        // Our way along the strip: a departure rolling, or the lander ahead
        // rolling out. Slow or accelerating is a roll; fast and slowing is a
        // rollout, off at a turn-off.
        let accel = track_velocity(&f, track, older).map(|((pa, pc), _)| {
            let prev_kts = (pa * pa + pc * pc).sqrt() * 1.943_844;
            (kts - prev_kts) / (track[track.len() - 1].2 - track[older].2)
        });
        return if kts < 40.0 || accel.is_some_and(|a| a >= 0.5) {
            StripExit::Clearing(roll_to_liftoff_s(kts))
        } else {
            StripExit::Clearing(ROLLOUT_CLEAR_S)
        };
    }
    // Across the strip: crossing or vacating, off at the edge it heads for.
    let half = strip_half_width_m(r);
    let dist = if vc.signum() == cross.signum() {
        half - cross.abs()
    } else {
        half + cross.abs()
    };
    StripExit::Clearing(dist.max(0.0) / vc.abs())
}

/// Our distance (nm) and time (s) to the landing threshold from the fixes the
/// monitor noted. `None` without a fresh fix (a test, the first tick).
fn own_time_to_threshold(r: &nav::Runway, landing_end: &str, now: Instant) -> Option<(f64, f64)> {
    let f = LandingFrame::new(r, landing_end)?;
    let fixes = OWN_FIXES.lock().unwrap_or_else(|e| e.into_inner());
    let &(lat, lon, at) = fixes.back()?;
    if now.saturating_duration_since(at).as_secs_f64() > 3.0 {
        return None;
    }
    let (x, y) = f.xy(lat, lon);
    let dist_m = (x * x + y * y).sqrt();
    let gs_kts = fixes
        .iter()
        .rev()
        .find(|p| at.saturating_duration_since(p.2).as_secs_f64() >= TRACK_MIN_DT_S)
        .map(|p| {
            let dt = at.saturating_duration_since(p.2).as_secs_f64();
            crate::nav::haversine_nm(p.0, p.1, lat, lon) / dt * 3600.0
        })
        .filter(|g| *g >= 40.0)
        .unwrap_or(DEFAULT_FINAL_KTS);
    Some((dist_m / 1852.0, dist_m / (gs_kts / 1.943_844)))
}

/// The go-around is due: we are inside [`GO_AROUND_DECIDE_S`] of the
/// threshold and some occupant will not be off the runway
/// [`RUNWAY_CLEAR_MARGIN_S`] before we cross it. Further out it is only
/// sequencing: a departure lined up gets its roll, a vacating arrival its
/// turn-off. An unmeasured time decides as before: the order.
pub(crate) fn go_around_due(own_s: Option<f64>, exits: &[StripExit]) -> bool {
    if exits.is_empty() {
        return false;
    }
    let Some(t) = own_s else {
        return true;
    };
    t <= GO_AROUND_DECIDE_S
        && exits
            .iter()
            .any(|e| e.secs().is_none_or(|s| s + RUNWAY_CLEAR_MARGIN_S > t))
}

/// A withheld landing clearance may go out: the strip is clear, or every
/// occupant is MOVING off it and will be gone [`RUNWAY_CLEAR_MARGIN_S`]
/// before we arrive (a departure rolling, an arrival vacating). A lined-up
/// or standing aircraft never earns it.
pub(crate) fn landing_release_due(own_s: Option<f64>, exits: &[StripExit]) -> bool {
    exits.iter().all(|e| match (e, own_s) {
        (StripExit::Clearing(s), Some(t)) => s + RUNWAY_CLEAR_MARGIN_S <= t,
        _ => false,
    })
}

/// One runway occupant as the landing judgement saw it: who, how it will
/// leave, and WHERE it was (the blackbox's evidence: issue #162's ghosts
/// were judged with no position on file).
#[derive(Debug, Clone)]
struct Occupant {
    callsign: String,
    exit: StripExit,
    /// Metres past our landing threshold along the landing direction, and
    /// off the centreline (left positive). `None` without our strip.
    along_m: Option<f64>,
    cross_m: Option<f64>,
    /// How long it has stood where it is.
    standing_s: f64,
}

/// The ghosts OCCUPYING our strip (on its pavement, not stale:
/// [`occupies`]) and how each will leave it; notes every ghost's position
/// for the tracks and the standing clocks first.
fn strip_exits(
    targets: &[TcasTarget],
    ours: Option<&nav::Runway>,
    landing_end: &str,
    now: Instant,
) -> Vec<Occupant> {
    note_ground_traffic(targets, now);
    let runways: Vec<nav::Runway> = match ours {
        Some(r) => vec![r.clone()],
        None => ACTIVE_RUNWAYS
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone(),
    };
    let frame = ours.and_then(|r| LandingFrame::new(r, landing_end));
    let guard = STRIP_SEEN.lock().unwrap_or_else(|e| e.into_inner());
    targets
        .iter()
        .filter(|t| runways.iter().any(|r| occupies(r, t, now).is_some()))
        .map(|t| {
            let exit = match (ours, guard.as_ref().and_then(|g| g.get(&t.callsign))) {
                (Some(r), Some(tr)) => {
                    let track: Vec<(f64, f64, f64)> = tr
                        .iter()
                        .map(|p| (p.0, p.1, -now.saturating_duration_since(p.2).as_secs_f64()))
                        .collect();
                    strip_exit(r, landing_end, &track)
                }
                _ => StripExit::Stays,
            };
            let place = frame.as_ref().map(|f| f.place(t.lat, t.lon));
            Occupant {
                callsign: t.callsign.clone(),
                exit,
                along_m: place.map(|p| p.0),
                cross_m: place.map(|p| p.1),
                standing_s: standing_secs(&t.callsign, now),
            }
        })
        .collect()
}

/// The last "go-around deferred" record: the judgement is logged at a
/// cadence, not every tick.
static GO_AROUND_DEFER_LOGGED: Mutex<Option<Instant>> = Mutex::new(None);

/// The occupants a runway judgement used, for the blackbox: how each leaves
/// and where it stood.
fn exits_json(exits: &[Occupant]) -> serde_json::Value {
    serde_json::Value::Array(
        exits
            .iter()
            .map(|o| {
                let (kind, s) = match o.exit {
                    StripExit::Clearing(s) => ("clearing", Some(s)),
                    StripExit::LinedUp(s) => ("lined_up", Some(s)),
                    StripExit::Stays => ("stays", None),
                    StripExit::Oncoming => ("oncoming", None),
                };
                serde_json::json!({"ghost": o.callsign, "exit": kind,
                    "clear_s": s.map(|s| (s * 10.0).round() / 10.0),
                    "along_m": o.along_m.map(f64::round),
                    "cross_m": o.cross_m.map(f64::round),
                    "standing_s": o.standing_s.round()})
            })
            .collect(),
    )
}

/// Our numbers for the blackbox: distance (nm) and time (s) to the threshold.
fn own_json(own: Option<(f64, f64)>) -> (serde_json::Value, serde_json::Value) {
    (
        serde_json::json!(own.map(|o| (o.0 * 100.0).round() / 100.0)),
        serde_json::json!(own.map(|o| o.1.round())),
    )
}

/// The tower protects the one aircraft it controls: cleared to land (or held
/// for the landing clearance), low, and a ghost on the runway that will still
/// be there when we arrive: the clearance is cancelled out loud instead of
/// letting the world contradict it. Crew phrasing when the pilot isn't tuned
/// (a safety net must not depend on the radio). Judged on TIME: inside
/// [`GO_AROUND_DECIDE_S`] of the threshold, against how the occupant is
/// leaving ([`strip_exit`]). LEMG 13 (sortie 1790594471-full_auto,
/// 2026-09-28): ordered at 3.8 nm, about 80 s out, for the departure TO4611
/// that lifted off 81 s later.
pub fn go_around_tick(
    targets: &[TcasTarget],
    own_agl_ft: f64,
    on_ground: bool,
    tuned: bool,
    now: Instant,
) -> Option<ComplianceCall> {
    // The standing clocks run through the whole final, not only its last
    // 800 ft ([`GHOST_STALE_S`]).
    note_ground_traffic(targets, now);
    if on_ground || own_agl_ft > 800.0 {
        return None;
    }
    let cleared = CLEARANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|c| c.state == tower::State::ClearedToLand)
        .map(|c| (c.callsign.clone(), c.runway_ident.clone()));
    let held = || {
        LANDING_HOLD
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map(|h| (h.callsign.clone(), h.runway_ident.clone()))
    };
    let (callsign, runway_ident) = cleared.or_else(held)?;
    // Only a ghost on OUR strip cancels the clearance — a parallel runway's
    // occupant is not our conflict (Heathrow, 2026-07-10).
    let ours = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        strip(&runways, &runway_ident).cloned()
    };
    let exits = strip_exits(targets, ours.as_ref(), &runway_ident, now);
    if exits.is_empty() {
        return None;
    }
    let own = ours
        .as_ref()
        .and_then(|r| own_time_to_threshold(r, &runway_ident, now));
    let plain: Vec<StripExit> = exits.iter().map(|e| e.exit).collect();
    let (own_nm, own_s) = own_json(own);
    if !go_around_due(own.map(|o| o.1), &plain) {
        let mut logged = GO_AROUND_DEFER_LOGGED
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if logged.is_none_or(|t| now.saturating_duration_since(t).as_secs_f64() >= 10.0) {
            *logged = Some(now);
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "go_around_deferred", "runway": runway_ident,
                    "own_nm": own_nm, "own_s": own_s, "occupants": exits_json(&exits)}),
            );
        }
        return None;
    }
    let mut last = GO_AROUND_LAST.lock().unwrap_or_else(|e| e.into_inner());
    if last.is_some_and(|t| now.duration_since(t).as_secs_f64() < 30.0) {
        return None;
    }
    *last = Some(now);
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "go_around_judged", "runway": runway_ident,
            "own_nm": own_nm, "own_s": own_s, "occupants": exits_json(&exits)}),
    );
    // A withheld landing is over: the order is the answer it was waiting for.
    *LANDING_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
    // The order voids the clearance in the FSM too, or the coach keeps
    // suggesting "cleared to land ..." as the readback owed while the tower
    // is shouting go around (seen live, 2026-07-14). The sync is handed to
    // the main loop, which owns the `Station`, same as the hold release.
    *PENDING_GO_AROUND.lock().unwrap_or_else(|e| e.into_inner()) = true;
    // An occupant that will not leave on its own clock (standing, or using
    // the other direction) would send the next approach round the same way:
    // the tower re-sequences now (issue #162). Only a flight on the
    // frequency can be told.
    let expect = if tuned {
        resequence_after_go_around(targets, &runway_ident, &exits, now)
    } else {
        None
    };
    // The runway the order names rides with it, published BEFORE its count
    // moves, so a crew watch that sees the order sees its runway (#182).
    *RESEQUENCED_ORDER.lock().unwrap_or_else(|e| e.into_inner()) = expect.clone().map(|to| {
        (
            GO_AROUND_ORDERS.load(std::sync::atomic::Ordering::SeqCst) + 1,
            to,
        )
    });
    // ...and the order itself, for every crew loop that flies it.
    GO_AROUND_ORDERS.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    Some(if tuned {
        let expect = expect
            .map(|to| format!("expect runway {}, ", atis::spoken_runway(&to)))
            .unwrap_or_default();
        ComplianceCall {
            line: format!(
                "{callsign}, go around, traffic on the runway. Climb to circuit \
                 altitude, {expect}report downwind."
            ),
            crew: false,
        }
    } else {
        ComplianceCall {
            line: "Traffic on the runway ahead. Go around.".to_string(),
            crew: true,
        }
    })
}

/// The wind the tower last worked with (the field's surface wind when it
/// had one), for the re-sequence the go-around watch makes between calls.
static TOWER_WIND: Mutex<Option<(f64, f64)>> = Mutex::new(None);

/// A re-sequence decided with a go-around order, for the main loop's FSM
/// sync ([`Station::apply_monitor_go_around`]): (the end given up, the end
/// offered). Station-private.
static PENDING_RESEQUENCE: Mutex<Option<(String, String)>> = Mutex::new(None);

/// The runway the tower offers an arrival sent round by an occupant that
/// will not leave on its own clock, or why none. Pure. `oncoming` (traffic
/// using the other direction on our strip) offers the RECIPROCAL first,
/// landing with that stream; a standing occupant blocks its whole strip, so
/// only another strip will do. Every candidate: not the blocked end, not a
/// strip with a fresh occupant (`occupied`, by `ident_a`), not against a
/// live stream (`streams`, and the oncoming traffic's own end), at least
/// [`RESEQUENCE_MIN_LENGTH_RATIO`] of the blocked strip's length, and a
/// wind that does not argue ([`stated_runway_stands`]). Ranked: a stream's
/// own end, then the smallest turn from the direction we were landing, then
/// the headwind.
pub(crate) fn resequence_runway(
    runways: &[nav::Runway],
    blocked_end: &str,
    oncoming: bool,
    occupied: &[String],
    streams: &[String],
    wind: Option<(f64, f64)>,
) -> Result<String, &'static str> {
    let Some(blocked) = strip(runways, blocked_end) else {
        return Err("unknown_runway");
    };
    let Some(landing_hdg) = end_heading(runways, blocked_end) else {
        return Err("unknown_runway");
    };
    let Some((wind_from, wind_kts)) = wind else {
        return Err("no_wind");
    };
    let len = |r: &nav::Runway| {
        let m_lon = 111_320.0 * r.lat_a.to_radians().cos();
        let (dx, dy) = ((r.lon_b - r.lon_a) * m_lon, (r.lat_b - r.lat_a) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    let blocked_len = len(blocked);
    let reciprocal = if blocked.ident_a == blocked_end {
        blocked.ident_b.clone()
    } else {
        blocked.ident_a.clone()
    };
    let mut streams: Vec<String> = streams.to_vec();
    if oncoming {
        streams.push(reciprocal);
    }
    let best = nav::best_runway_for_wind(runways, wind_from, wind_kts);
    let mut picks: Vec<(u8, f64, f64, String)> = Vec::new();
    for r in runways {
        for end in [&r.ident_a, &r.ident_b] {
            if end == blocked_end {
                continue;
            }
            let same_strip = r.ident_a == blocked.ident_a;
            if same_strip && !oncoming {
                continue;
            }
            if !same_strip && occupied.contains(&r.ident_a) {
                continue;
            }
            if len(r) < RESEQUENCE_MIN_LENGTH_RATIO * blocked_len {
                continue;
            }
            if streams.iter().any(|s| ends_oppose(runways, s, end)) {
                continue;
            }
            let hw = nav::headwind_for(runways, end, wind_from, wind_kts);
            if !stated_runway_stands(runways, end, hw, best.clone(), wind_from, wind_kts) {
                continue;
            }
            let Some(hdg) = end_heading(runways, end) else {
                continue;
            };
            let stream_rank = u8::from(!streams.iter().any(|s| s == end));
            picks.push((
                stream_rank,
                sim::angles::abs_diff_deg(landing_hdg, hdg),
                -hw.unwrap_or(0.0),
                end.clone(),
            ));
        }
    }
    picks.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then(a.1.total_cmp(&b.1))
            .then(a.2.total_cmp(&b.2))
    });
    picks
        .into_iter()
        .next()
        .map(|p| p.3)
        .ok_or("no_other_runway")
}

/// The go-around watch's re-sequence: when an occupant of our strip will not
/// leave on its own clock (standing, or oncoming), pick the runway to offer
/// ([`resequence_runway`]) and hand it to the FSM sync; else the arrival is
/// held on its runway (the next final report is withheld while the strip is
/// occupied and released when it clears). Recorded `resequenced` either way.
fn resequence_after_go_around(
    targets: &[TcasTarget],
    runway_ident: &str,
    exits: &[Occupant],
    now: Instant,
) -> Option<String> {
    let oncoming = exits.iter().any(|o| o.exit == StripExit::Oncoming);
    let standing = exits.iter().any(|o| o.exit == StripExit::Stays);
    if !oncoming && !standing {
        return None;
    }
    let runways = ACTIVE_RUNWAYS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone();
    let occupied: Vec<String> = runways
        .iter()
        .filter(|r| targets.iter().any(|t| occupies(r, t, now).is_some()))
        .map(|r| r.ident_a.clone())
        .collect();
    let field = active_field();
    let (arrival, departure) = field_streams(
        &runways,
        arrival_end(
            targets,
            &runways,
            field.as_ref().map(|f| f.3).unwrap_or(0.0),
        ),
        now,
    );
    let streams: Vec<String> = arrival.iter().chain(departure.iter()).cloned().collect();
    let wind = surface_wind_at(field.as_ref().map(|f| (f.1, f.2)))
        .or(*TOWER_WIND.lock().unwrap_or_else(|e| e.into_inner()));
    let pick = resequence_runway(&runways, runway_ident, oncoming, &occupied, &streams, wind);
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "resequenced", "from": runway_ident,
            "to": pick.as_ref().ok(), "hold": pick.is_err(),
            "why": if oncoming { "oncoming" } else { "standing" },
            "refused": pick.as_ref().err(), "occupied": occupied,
            "arrival_end": arrival, "departure_end": departure,
            "wind_from": wind.map(|w| w.0.round()),
            "wind_kts": wind.map(|w| (w.1 * 10.0).round() / 10.0),
            "occupants": exits_json(exits)}),
    );
    let to = pick.ok()?;
    *PENDING_RESEQUENCE.lock().unwrap_or_else(|e| e.into_inner()) =
        Some((runway_ident.to_string(), to.clone()));
    Some(to)
}

/// A landing clearance withheld for traffic on the runway: armed by the reply
/// to a final report ("continue approach, traffic on the runway"), released
/// by [`landing_hold_tick`].
pub struct LandingHold {
    callsign: String,
    /// Spoken, for the readback ("one three").
    runway: String,
    /// The END we land on ("13").
    runway_ident: String,
    /// The clearance as it will be spoken (the wind of the final report).
    line: String,
}

/// One pass of the withheld-landing watch: once our strip is clear, or its
/// occupant is moving off it in time ([`landing_release_due`]), the tower
/// comes back unprompted with the landing clearance. The published clearance
/// moves to ClearedToLand at once (the crew reads the readback owed); the
/// station's FSM syncs before it answers the next call
/// ([`Station::sync_landing_release`]). Landed, the hold is moot: dropped.
/// The go-around order, when the traffic stays, is [`go_around_tick`]'s.
pub fn landing_hold_tick(targets: &[TcasTarget], on_ground: bool, now: Instant) -> Option<String> {
    let mut guard = LANDING_HOLD.lock().unwrap_or_else(|e| e.into_inner());
    let hold = guard.as_ref()?;
    if on_ground || station_unavailable() {
        *guard = None;
        return None;
    }
    let ours = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        strip(&runways, &hold.runway_ident).cloned()
    };
    let exits = strip_exits(targets, ours.as_ref(), &hold.runway_ident, now);
    let own = ours
        .as_ref()
        .and_then(|r| own_time_to_threshold(r, &hold.runway_ident, now));
    let plain: Vec<StripExit> = exits.iter().map(|e| e.exit).collect();
    if !landing_release_due(own.map(|o| o.1), &plain) {
        return None;
    }
    let hold = guard.take()?;
    let (own_nm, own_s) = own_json(own);
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "landing_released", "runway": hold.runway_ident,
            "own_nm": own_nm, "own_s": own_s, "occupants": exits_json(&exits)}),
    );
    if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
        c.state = tower::State::ClearedToLand;
        c.holds_runway = true;
        c.runway_ident = hold.runway_ident.clone();
        c.pending_readback = Some(format!(
            "Cleared to land runway {}, {}.",
            hold.runway, hold.callsign
        ));
    }
    *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *PENDING_LANDING_RELEASE
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = Some((hold.runway.clone(), hold.line.clone()));
    Some(hold.line)
}

/// A takeoff clearance withheld for landing traffic — armed by the reply
/// path, released by the monitor once the final has stayed clear.
pub struct TakeoffHold {
    callsign: String,
    /// Spoken, for the release line ("zero niner right").
    runway: String,
    /// The end ident ("09R"), for the strip-restricted traffic check.
    runway_ident: String,
    clear_since: Option<Instant>,
    /// When the hold went on — the wake block is measured against this, so a
    /// busy field's departure stream can't extend it forever (see
    /// [`takeoff_hold_tick`]).
    armed_at: Instant,
    /// When the "still held" diagnostic last went out.
    logged_at: Instant,
    /// Whether the watch has already sequenced this departure onto the
    /// runway with a line-up-and-wait. One shot: the aircraft can only take
    /// the runway once.
    lined_up: bool,
    /// The line-up was CONDITIONAL ("behind the landing X, line up behind"):
    /// the pavement is the pilot's only once X has passed. The watch
    /// publishes that condition for the crew every tick.
    behind: bool,
    /// The traffic this hold is FOR, by raw callsign: the strip's occupants
    /// at the watch's first tick and every arrival seen on our final since.
    /// Seeded by [`takeoff_hold_tick_from`]; kept across a re-request on
    /// the same runway. `None` until seeded.
    held_for: Option<std::collections::HashSet<String>>,
    /// Cut-ins already blackboxed (`hold_cut_in`), once per ghost.
    cut_in_logged: std::collections::HashSet<String>,
}

/// How much better another runway's headwind must be before the tower swaps
/// the runway in use. Big enough that two parallels (identical headings,
/// headwinds equal to rounding error) can never trade places on noise.
const RUNWAY_SWAP_MARGIN_KTS: f64 = 3.0;

/// Under this the wind does not pick the runway: the pick keeps the runway in
/// use (the into-wind choice flip-flops on noise), the ATIS keeps its
/// edition's runway, and an arrival's stated runway stands.
const CALM_WIND_KTS: f64 = 5.0;

/// The tailwind an arrival's (or a taxi request's) stated runway may carry and
/// still stand: ICAO PANS-ATM 7.2.3 accepts a preferential runway with up to
/// 5 kt of tailwind on a dry runway.
const STATED_RUNWAY_TAILWIND_KTS: f64 = 5.0;

/// Whether the wind argues with the runway the pilot named. `hw` is its
/// headwind (negative = tailwind), `best` the into-wind pick. It stands in
/// calm; within STATED_RUNWAY_TAILWIND_KTS of tailwind when its crosswind is
/// no worse than the best end's by more than the swap margin (the reciprocal
/// of the best end has the same crosswind; a crossing strip may not); or with
/// a headwind inside the swap margin of the best.
fn stated_runway_stands(
    runways: &[nav::Runway],
    ident: &str,
    hw: Option<f64>,
    best: Option<(String, f64, f64)>,
    wind_from: f64,
    wind_kts: f64,
) -> bool {
    runway_stands_within(
        runways,
        ident,
        hw,
        best,
        wind_from,
        wind_kts,
        STATED_RUNWAY_TAILWIND_KTS,
    )
}

/// The tailwind a RUNWAY START's own strip may carry and still be given it:
/// a pilot lined up and asking for the runway under him takes a tailwind
/// within the common transport limit of 10 kt, as a controller with nothing
/// else on the field approves a pilot's request (#178). A stream on the
/// other end still wins.
const ON_STRIP_TAILWIND_KTS: f64 = 10.0;

/// [`stated_runway_stands`] with its tailwind limit.
fn runway_stands_within(
    runways: &[nav::Runway],
    ident: &str,
    hw: Option<f64>,
    best: Option<(String, f64, f64)>,
    wind_from: f64,
    wind_kts: f64,
    tailwind_kts: f64,
) -> bool {
    let (Some(hw), Some((_, best_hdg, best_hw))) = (hw, best) else {
        return false;
    };
    if wind_kts < CALM_WIND_KTS {
        return true;
    }
    if best_hw - hw <= RUNWAY_SWAP_MARGIN_KTS {
        return true;
    }
    let crosswind =
        |hdg: f64| (wind_kts * sim::angles::signed_diff_deg(wind_from, hdg).to_radians().sin()).abs();
    let stated_hdg = runways.iter().find_map(|r| {
        if r.ident_a == ident {
            Some(r.heading_a_deg())
        } else if r.ident_b == ident {
            Some(r.heading_b_deg())
        } else {
            None
        }
    });
    let Some(stated_hdg) = stated_hdg else {
        return false;
    };
    hw >= -tailwind_kts
        && crosswind(stated_hdg) <= crosswind(best_hdg) + RUNWAY_SWAP_MARGIN_KTS
}

/// The final must stay clear this long before the held clearance releases —
/// an arrival crossing the fence vanishes from the airborne band while still
/// rolling out, and "cleared for takeoff" over a lander is a contradiction.
const HOLD_CLEAR_SECS: f64 = 15.0;

/// How far out the nearest arrival may be and still have a departure released
/// ahead of it.
///
/// A real tower THREADS departures into the arrival stream; it does not wait
/// for an empty four-mile final, because at a single-runway field working a
/// landing every couple of minutes that final is never empty. Requiring
/// [`final_traffic_for`]'s whole 4 nm corridor to be clear made the release
/// unreachable at Gatwick: held at holding point Mike for runway 26L with an
/// arrival at half a mile, then eight minutes of stream behind it, and full
/// auto handed the aircraft back (2026-07-27). Three miles is about eighty
/// seconds at approach speed — a light twin is airborne and turning out well
/// inside that, which is the gap controllers actually use. The runway itself
/// being clear is a SEPARATE and absolute condition below.
const DEPART_GAP_NM: f64 = 3.0;

/// While a departure is held, log what is holding it at this cadence. "Held
/// for traffic" is undiagnosable after the fact otherwise — the Heathrow hold
/// had to be reconstructed by probing the live sim (2026-07-27).
const HOLD_LOG_SECS: f64 = 15.0;

/// One pass of the held-clearance watch: with OUR runway's final clear (no
/// arrival in its corridor, no ghost on the strip — a parallel runway's
/// stream doesn't count) long enough, the tower comes back unprompted —
/// like a real one — and the FSM sync is handed to the main loop.
/// When a takeoff hold's clock started: `now` for a new hold, and the
/// ORIGINAL moment for one already standing on the same runway.
///
/// Asking again does not restart the separation you have already waited out.
/// The wake gate in [`takeoff_hold_tick`] is capped at one full interval at
/// the hold, measured from this stamp — and re-stamping it on every request
/// handed that cap straight back: at a field whose departure interval is
/// shorter than the separation itself, a pilot who asks a second time resets
/// their own clock, hears "wake turbulence, departing traffic ahead of you"
/// again, and never takes off. The clock belongs to the departure ahead of us
/// in sequence, not to our last transmission. A hold on a DIFFERENT runway is
/// a different queue and starts fresh.
fn hold_armed_at(existing: Option<&TakeoffHold>, runway_ident: &str, now: Instant) -> Instant {
    existing
        .filter(|h| h.runway_ident == runway_ident)
        .map_or(now, |h| h.armed_at)
}

/// Groundspeed past which our aircraft on the ground is ROLLING for takeoff,
/// never taxiing (taxi paces top out near 30 kt).
pub const HOLD_ROLL_KTS: f64 = 40.0;

/// Retire a held release once our aircraft rolls on the ground past
/// [`HOLD_ROLL_KTS`]: the takeoff it was holding has happened (through the
/// cancel, or before it was held). The reject call retires it too; this is
/// the tower's own eyes for a reject it never heard. `true` when a hold was
/// voided.
pub fn void_takeoff_hold_by_roll(gs_kts: f64) -> bool {
    if gs_kts < HOLD_ROLL_KTS {
        return false;
    }
    let voided = TAKEOFF_HOLD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .take()
        .is_some();
    if voided {
        *LINE_UP_CONDITION.lock().unwrap_or_else(|e| e.into_inner()) = None;
        tracing::info!(gs_kts, "held takeoff release voided: the aircraft rolled");
    }
    voided
}

/// [`takeoff_hold_tick_from`] without our own position: every occupant of
/// the pavement holds the departure.
pub fn takeoff_hold_tick(
    targets: &[TcasTarget],
    field_elev_ft: f64,
    player_on_ground: bool,
    now: Instant,
) -> Option<String> {
    takeoff_hold_tick_from(targets, field_elev_ft, player_on_ground, None, now)
}

/// A ghost more than this behind our reference point, on the departure
/// end's heading, is behind our tail: it is not in the way of our roll.
const OWN_TAIL_M: f64 = 40.0;

/// A ghost on our strip standing this long is standing: a lined-up or
/// parked replay, not a roll or a rollout.
const CUT_IN_STANDING_S: f64 = 10.0;

/// What a departure sees on its own strip.
#[derive(Debug, Default, PartialEq)]
struct StripPicture {
    /// Something on the pavement holds the departure.
    blocked: bool,
    /// The occupants ahead of our tail, raw callsigns (the watch's seed).
    occupants: Vec<String>,
    /// Ghosts standing on the strip that the hold was not held for: a
    /// replay's departure queue lining up in front of an aircraft the tower
    /// gave the runway to. The party line withholds their clearance
    /// (`party_line_withheld`); the hold is not held for them.
    cut_in: Vec<String>,
}

/// A departure's traffic on its own strip: a ghost that OCCUPIES the
/// pavement ([`occupies`]: on it, and not stale data) and, when we stand on
/// that pavement ourselves (`own`), is not behind our tail on the departure
/// end's heading. With `held_for` (a hold issued to an aircraft ON the
/// strip), a ghost the hold was not held for holds it only while it moves
/// (a roll leaving, a rollout); standing, it is a cut-in. The same pavement
/// and stale-ghost rules as the landing judgement (#162): the incursion
/// watch's wider capture, and a replay queue of 13 departures lining up in
/// front of it, held a 737 lined up on LEMG 13 for five minutes after the
/// traffic it was promised a clearance behind had landed (sortie
/// 1790623420-full_auto, 2026-09-28, issue #172).
fn departure_strip_picture(
    r: &nav::Runway,
    departure_end: &str,
    targets: &[TcasTarget],
    own: Option<(f64, f64)>,
    held_for: Option<&std::collections::HashSet<String>>,
    now: Instant,
) -> StripPicture {
    let frame = LandingFrame::new(r, departure_end);
    let own_along = match (&frame, own) {
        (Some(f), Some((lat, lon))) if on_runway_pavement(r, lat, lon).is_some() => {
            Some(f.place(lat, lon).0)
        }
        _ => None,
    };
    let mut out = StripPicture::default();
    for t in targets {
        if occupies(r, t, now).is_none() {
            continue;
        }
        if let (Some(f), Some(ours)) = (&frame, own_along) {
            if f.place(t.lat, t.lon).0 < ours - OWN_TAIL_M {
                continue;
            }
        }
        out.occupants.push(t.callsign.clone());
        let cut_in = held_for.is_some_and(|set| {
            !set.contains(&t.callsign) && standing_secs(&t.callsign, now) >= CUT_IN_STANDING_S
        });
        if cut_in {
            out.cut_in.push(t.callsign.clone());
        } else {
            out.blocked = true;
        }
    }
    out
}

/// [`departure_strip_picture`]'s verdict for the tower's reply: the hold
/// standing on this runway (if any) says who it was held for.
fn departure_strip_occupied(
    r: &nav::Runway,
    departure_end: &str,
    targets: &[TcasTarget],
    own: Option<(f64, f64)>,
    now: Instant,
) -> bool {
    let held_for = TAKEOFF_HOLD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|h| h.lined_up && h.runway_ident == departure_end)
        .and_then(|h| h.held_for.clone());
    departure_strip_picture(r, departure_end, targets, own, held_for.as_ref(), now).blocked
}

/// One pass of the held-clearance watch with our own position (`own`, for
/// the occupant behind our tail; see [`departure_strip_occupied`]).
pub fn takeoff_hold_tick_from(
    targets: &[TcasTarget],
    field_elev_ft: f64,
    player_on_ground: bool,
    own: Option<(f64, f64)>,
    now: Instant,
) -> Option<String> {
    if station_unavailable() {
        return None;
    }
    // The standing clocks the stale-ghost rule reads (idempotent per tick).
    note_ground_traffic(targets, now);
    let mut guard = TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner());
    let hold = guard.as_mut()?;
    // Airborne, the hold is moot: either the pilot departed through the
    // cancel (the compliance watch owns that conversation) or the whole
    // thing is stale. "Traffic clear, cleared for takeoff" spoken to an
    // aircraft two minutes into its climb (Gatwick, 2026-07-13) helps
    // nobody — void it, silently.
    if !player_on_ground {
        *guard = None;
        *LINE_UP_CONDITION.lock().unwrap_or_else(|e| e.into_inner()) = None;
        return None;
    }
    // Our strip, cloned out so the runway lock isn't held across the checks.
    let (rwy, runways) = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        (strip(&runways, &hold.runway_ident).cloned(), runways.clone())
    };
    // The two conditions are DIFFERENT in kind, and conflating them is what
    // made the release unreachable under injected traffic: the runway being
    // occupied is absolute, while an arrival on final is a question of GAP.
    let inbound = rwy
        .as_ref()
        .and_then(|r| final_traffic_for(targets, r, &runways, field_elev_ft));
    // The traffic this hold is for: the strip's occupants at its first
    // tick, and every arrival on our final since.
    if hold.held_for.is_none() {
        let seed = rwy.as_ref().map(|r| {
            departure_strip_picture(r, &hold.runway_ident, targets, own, None, now).occupants
        });
        hold.held_for = Some(seed.unwrap_or_default().into_iter().collect());
    }
    if let (Some(set), Some(f)) = (hold.held_for.as_mut(), inbound.as_ref()) {
        set.extend(f.ghosts.iter().cloned());
    }
    // Held ON the strip, the runway is ours: a ghost lining up in front of
    // us is the replay's queue, not the traffic we were held for.
    let held_for = if hold.lined_up {
        hold.held_for.clone()
    } else {
        None
    };
    let picture = rwy.as_ref().map(|r| {
        departure_strip_picture(r, &hold.runway_ident, targets, own, held_for.as_ref(), now)
    });
    for g in picture.iter().flat_map(|p| p.cut_in.iter()) {
        if hold.cut_in_logged.insert(g.clone()) {
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "hold_cut_in", "ghost": g,
                    "runway": hold.runway_ident}),
            );
        }
    }
    let on_strip = match &picture {
        Some(p) => p.blocked,
        None => targets
            .iter()
            .any(|t| t.on_ground && on_active_runway(t.lat, t.lon).is_some()),
    };
    // A CONDITIONAL line-up's condition, for the crew: the named arrival has
    // passed once it is off the final (down on the strip, or gone). Published
    // every tick, cleared with the hold. The Baron at Malaga crossed the hold
    // line five seconds after "behind the landing Bee-line, line up behind"
    // with the lander at 2.4 nm, and only its 85 s crawl kept it off the
    // pavement until 15 s after the touchdown (12:21Z, 2026-09-11).
    *LINE_UP_CONDITION.lock().unwrap_or_else(|e| e.into_inner()) =
        hold.behind.then(|| inbound.is_none());
    let traffic = match &rwy {
        Some(_) => on_strip || inbound.as_ref().is_some_and(|f| f.nm < DEPART_GAP_NM),
        // No geometry for the held runway: fall back to the blunt any-runway
        // checks rather than clearing blind.
        None => {
            on_strip
                || targets
                    .iter()
                    .any(|t| !t.on_ground && (30.0..1500.0).contains(&(t.ele_ft - field_elev_ft)))
        }
    };
    // The wake clock blocks the release exactly like traffic does: a held
    // clearance behind a departed heavy waits out the two minutes. But it is
    // measured from when the HOLD went on, not from the last liftoff —
    // separation is owed behind the departure ahead of you IN SEQUENCE, and
    // a ghost that took your slot while you sat at the taxiway restarts
    // nothing you owe. Un-capped, a field whose departure interval is
    // shorter than the separation itself (Heathrow's 27L: a liftoff about
    // every two minutes) re-armed the clock faster than it could expire, the
    // release condition became unreachable, and full auto sat out its entire
    // budget at holding point Sierra before handing the aircraft back
    // (2026-07-27). One full interval at the hold IS the separation.
    let wake = now.duration_since(hold.armed_at).as_secs() < WAKE_SECS
        && wake_remaining(&hold.runway_ident, now).is_some();
    if traffic || wake {
        hold.clear_since = None;
        // What is holding us, at a cadence — a held departure that says
        // nothing for eight minutes can only be diagnosed by probing the
        // live sim, which is how the Heathrow hold had to be found.
        // On the injected clock like the rest of the tick, so tests that
        // drive `now` see the same cadence the sim does (duration_since
        // saturates to zero if `now` sits behind the stamp).
        if now.duration_since(hold.logged_at).as_secs_f64() >= HOLD_LOG_SECS {
            hold.logged_at = now;
            tracing::info!(
                runway = %hold.runway_ident,
                final_nm = ?inbound.as_ref().map(|f| (f.nm * 10.0).round() / 10.0),
                arrivals = inbound.as_ref().map(|f| f.count).unwrap_or(0),
                on_strip,
                wake,
                lined_up = hold.lined_up,
                "departure still held"
            );
        }
        // The strip is clear and the blocker is only a CLOCK or a gap that
        // hasn't opened yet: sequence the departure ONTO the runway instead
        // of leaving it at the taxiway while the stream rolls past. Line up
        // and wait is what a real tower gives here, and it is what makes the
        // release meaningful — an aircraft still behind the hold line is not
        // next for anything. Never onto an OCCUPIED runway, which is why
        // every branch sits under `!on_strip`.
        //
        // Behind an ARRIVAL the same rule the tower's own conditional uses
        // applies: one aircraft, far enough out (`CONDITIONAL_MIN_NM`). A
        // lander at half a mile earns the hold, not the pavement.
        let staging = if !hold.lined_up && !on_strip {
            if wake {
                Some("wake turbulence, departing traffic ahead of you")
            } else if inbound
                .as_ref()
                .is_some_and(|f| f.count <= 1 && f.nm >= tower::CONDITIONAL_MIN_NM)
            {
                Some("landing traffic ahead of you")
            } else {
                None
            }
        } else {
            None
        };
        if let Some(reason) = staging {
            hold.lined_up = true;
            let line = format!(
                "{}, line up and wait runway {}, {reason}.",
                hold.callsign, hold.runway
            );
            if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
                c.state = tower::State::LinedUp;
            }
            *PENDING_LINE_UP.lock().unwrap_or_else(|e| e.into_inner()) = true;
            return Some(line);
        }
        return None;
    }
    let since = *hold.clear_since.get_or_insert(now);
    if now.duration_since(since).as_secs_f64() < HOLD_CLEAR_SECS {
        return None;
    }
    let line = format!(
        "{}, traffic clear. Runway {}, cleared for takeoff.",
        hold.callsign, hold.runway
    );
    if let Some(c) = CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
        c.state = tower::State::TakeoffCleared;
    }
    *PENDING_TAKEOFF_CLEARANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = true;
    *guard = None;
    // Released: the clearance stands on its own, no condition left to wait on.
    *LINE_UP_CONDITION.lock().unwrap_or_else(|e| e.into_inner()) = None;
    Some(line)
}

// --- Wake turbulence -------------------------------------------------------

/// The last airline-sized departure off a strip: wake separation runs from
/// its liftoff. TCAS carries no aircraft type, so "an airline callsign" is
/// the deterministic stand-in for "medium or heavier" — at the fields where
/// wake matters (Heathrow's replay stream) it's right essentially always,
/// and a GA ghost departing ahead of you correctly starts no clock.
pub struct Wake {
    pub since: Instant,
    /// The strip it departed from (end ident, e.g. "27R").
    pub runway_ident: String,
}

static WAKE: Mutex<Option<Wake>> = Mutex::new(None);

/// ICAO's two minutes for a light departing behind a medium/heavy from the
/// same position.
pub const WAKE_SECS: u64 = 120;

/// Ground/air memory per ghost for liftoff detection, independent of the
/// party line (which is config-gated and must not be a safety dependency).
static WAKE_PREV: Mutex<Option<std::collections::HashMap<String, WakePrev>>> =
    Mutex::new(None);

/// One ghost's last sample for the liftoff watch: on the ground, when seen,
/// and where (the roll direction names the END it departed).
#[derive(Clone, Copy)]
struct WakePrev {
    on_ground: bool,
    seen: Instant,
    lat: f64,
    lon: f64,
}

/// The live DEPARTURE stream: the runway end the last ghost lifted off, by
/// its roll direction, and when. A departure is an event, so it is
/// remembered here, for [`DEPARTURE_STREAM_SECS`], by the same watch that
/// starts the wake clock; landings likewise ([`ARRIVAL_STREAM`]).
static DEPARTURE_STREAM: Mutex<Option<(String, Instant)>> = Mutex::new(None);

/// How long a ghost's liftoff still says which way the field departs.
pub const DEPARTURE_STREAM_SECS: u64 = 600;

/// The end the field is departing from right now: the last ghost liftoff
/// within [`DEPARTURE_STREAM_SECS`], else `None`.
pub fn departure_end(now: Instant) -> Option<String> {
    DEPARTURE_STREAM
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(_, at)| now.saturating_duration_since(*at).as_secs() < DEPARTURE_STREAM_SECS)
        .map(|(end, _)| end.clone())
}

/// The remembered ARRIVAL stream: the runway end the last ghost touched down
/// on, by its track from the air to the pavement, and when. [`arrival_end`]
/// sees a lander only while it is on final; between landers nothing knew the
/// field was landing 12, and the SR22's stated 30 was cleared head-on to it in
/// calm (sortie 1790610218-approach, LEMG 2026-09-28: Ryanair 431 and 590
/// landed 12 at t 1790610003 and 1790610134, "cleared to land runway three
/// zero" at t 1790610223, sent round twice, issue #164). Kept, like the
/// departure stream, for [`ARRIVAL_STREAM_SECS`] by the wake watch.
static ARRIVAL_STREAM: Mutex<Option<(String, Instant)>> = Mutex::new(None);

/// How long a ghost's touchdown still says which way the field lands.
pub const ARRIVAL_STREAM_SECS: u64 = 600;

/// The end the field has been landing on: the last ghost touchdown within
/// [`ARRIVAL_STREAM_SECS`], else `None`. The memory only: the live sighting
/// on final ([`arrival_end`]) outranks it through [`field_streams`].
pub fn arrival_stream_end(now: Instant) -> Option<String> {
    ARRIVAL_STREAM
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(_, at)| now.saturating_duration_since(*at).as_secs() < ARRIVAL_STREAM_SECS)
        .map(|(end, _)| end.clone())
}

/// The streams the field is working, `(arrival, departure)`, as the runway
/// pick, the stated-runway refusal and the re-sequence read them. The
/// arrival is the live lander on final ([`arrival_end`], passed as `live`),
/// else the remembered touchdown ([`arrival_stream_end`]); the departure is
/// the remembered liftoff ([`departure_end`]). When the two point against
/// each other the field has turned round: the OLDER memory is dropped (a
/// live lander is now).
pub fn field_streams(
    runways: &[nav::Runway],
    live: Option<String>,
    now: Instant,
) -> (Option<String>, Option<String>) {
    let fresh = |at: &Instant| now.saturating_duration_since(*at).as_secs();
    let landed = ARRIVAL_STREAM
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
        .filter(|(_, at)| fresh(at) < ARRIVAL_STREAM_SECS);
    let departed = DEPARTURE_STREAM
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
        .filter(|(_, at)| fresh(at) < DEPARTURE_STREAM_SECS);
    let arrival = match live {
        Some(end) => Some((end, now)),
        None => landed,
    };
    match (arrival, departed) {
        (Some((a, at_a)), Some((d, at_d))) if ends_oppose(runways, &a, &d) => {
            if at_a >= at_d {
                (Some(a), None)
            } else {
                (None, Some(d))
            }
        }
        (a, d) => (a.map(|x| x.0), d.map(|x| x.0)),
    }
}

/// Does this flight id carry an airline prefix (ICAO 3-letter or IATA
/// 2-letter + digit)? The wake heuristic's "medium or heavier".
fn is_airline(id: &str) -> bool {
    // `get`, never a byte-index slice: the id comes off raw TCAS bytes via
    // from_utf8_lossy, and slicing a replacement character mid-way panics.
    let id = id.trim();
    (id.len() > 3
        && id
            .get(..3)
            .is_some_and(|pfx| TELEPHONY.iter().any(|(p, _)| *p == pfx)))
        || (id.len() > 2
            && id.as_bytes()[2].is_ascii_digit()
            && id
                .get(..2)
                .is_some_and(|pfx| TELEPHONY_IATA.iter().any(|(p, _)| *p == pfx)))
}

/// One pass of the wake watch: a ghost transitioning ground→air near the
/// field starts the two-minute clock on its strip. Called by the monitor on
/// the same TCAS read as the other traffic watches.
pub fn wake_note_targets(targets: &[TcasTarget], now: Instant) {
    let Some((_, flat, flon, _)) = active_field() else {
        return;
    };
    // The standing clocks ride every traffic read (`GHOST_STALE_S`).
    note_ground_traffic(targets, now);
    let mut guard = WAKE_PREV.lock().unwrap_or_else(|e| e.into_inner());
    let mem = guard.get_or_insert_with(std::collections::HashMap::new);
    mem.retain(|_, p| now.duration_since(p.seen).as_secs() < 300);
    for t in targets {
        if crate::nav::haversine_nm(t.lat, t.lon, flat, flon) > 3.0 {
            continue;
        }
        let here = WakePrev {
            on_ground: t.on_ground,
            seen: now,
            lat: t.lat,
            lon: t.lon,
        };
        let prev = *mem.entry(t.callsign.clone()).or_insert(here);
        mem.insert(t.callsign.clone(), here);
        let was_ground = prev.on_ground;
        // Any liftoff says which way the field departs, airline or not: the
        // end by the roll direction, as the party line names it.
        if was_ground && !t.on_ground {
            if let Some(end) = departure_runway_ident(t.lat, t.lon, prev.lat, prev.lon) {
                let end = main_runway_of(&end);
                tracing::info!(ghost = %t.callsign, runway = %end, "departure stream");
                *DEPARTURE_STREAM.lock().unwrap_or_else(|e| e.into_inner()) = Some((end, now));
            }
        }
        // Any touchdown ON a runway says which way the field lands: the end
        // by its track from the last airborne sample (issue #164).
        if !was_ground && t.on_ground {
            let on_pavement = ACTIVE_RUNWAYS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .iter()
                .any(|r| on_runway(r, t.lat, t.lon).is_some());
            if on_pavement {
                if let Some(end) = departure_runway_ident(t.lat, t.lon, prev.lat, prev.lon) {
                    let end = main_runway_of(&end);
                    tracing::info!(ghost = %t.callsign, runway = %end, "arrival stream");
                    let changed = ARRIVAL_STREAM
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .as_ref()
                        .is_none_or(|(e, at)| {
                            *e != end
                                || now.saturating_duration_since(*at).as_secs()
                                    >= ARRIVAL_STREAM_SECS
                        });
                    if changed {
                        sim::blackbox::record(
                            "atc",
                            serde_json::json!({"event": "arrival_stream", "ghost": t.callsign,
                                "runway": end}),
                        );
                    }
                    *ARRIVAL_STREAM.lock().unwrap_or_else(|e| e.into_inner()) = Some((end, now));
                }
            }
        }
        if was_ground && !t.on_ground && is_airline(&t.callsign) {
            // The END it departed, by the roll, on the main strip of a close
            // parallel pair: the nearest threshold of the nearest runway was
            // EGKK's 26R (the 08L pavement) for an 08R departure, and the
            // held 08R release ignored the clock (2026-09-28, #155).
            if let Some(ident) = departure_runway_ident(t.lat, t.lon, prev.lat, prev.lon)
                .or_else(|| nearest_runway_ident(t.lat, t.lon))
                .map(|id| main_runway_of(&id))
            {
                tracing::info!(ghost = %t.callsign, runway = %ident, "wake clock started");
                *WAKE.lock().unwrap_or_else(|e| e.into_inner()) = Some(Wake {
                    since: now,
                    runway_ident: ident,
                });
            }
        }
    }
}

/// Seconds of wake separation still to run behind the last departure from
/// THIS strip (end ident). `None` = clock expired, other strip, or no
/// departure seen.
pub fn wake_remaining(runway_ident: &str, now: Instant) -> Option<u64> {
    let guard = WAKE.lock().unwrap_or_else(|e| e.into_inner());
    let w = guard.as_ref()?;
    let elapsed = now.duration_since(w.since).as_secs();
    if elapsed >= WAKE_SECS {
        return None;
    }
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    same_strip_or_twin(&runways, &w.runway_ident, runway_ident).then_some(WAKE_SECS - elapsed)
}

// --- Taxi routes -------------------------------------------------------------

/// The active field's taxi network, loaded in the background (a 400 MB
/// apt.dat scan must never stall the reply path). `None` while loading or
/// when the field has none — taxi clearances then phrase route-less,
/// exactly as before.
static TAXI_NET: Mutex<Option<std::sync::Arc<nav::TaxiNet>>> = Mutex::new(None);

/// The loaded taxi network, for the party line's stand accessor. `None`
/// until the background load lands or when the field has none.
pub(crate) fn taxi_net() -> Option<std::sync::Arc<nav::TaxiNet>> {
    TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// The taxi route the pilot is CLEARED on — published by the reply path
/// when a taxi clearance is issued, consumed by the compliance watch (ATC's
/// off-route challenge) and the copilot's progressive taxi guide (the crew
/// side: "next left onto Whiskey").
#[derive(Clone)]
pub struct TaxiWatch {
    /// Node polyline of the cleared route.
    pub path: Vec<(f64, f64)>,
    /// The spoken via line, for the challenge ("taxi via Alpha, Bravo").
    pub via: String,
    /// Per-segment RAW (name, is_runway) aligned with `path` segments
    /// (`seg[i]` = the leg `path[i] -> path[i+1]`); phrase with
    /// [`spoken_taxiway`] / [`atis::spoken_runway`] at speak time.
    pub seg: Vec<(String, bool)>,
    /// RAW holding-point taxiway name ("A13"), the route's destination.
    pub holding_point: String,
    /// The authored lead-on the hold-short trim removed: the painted line from
    /// the holding point onto the runway centreline. The crew's line-up drives
    /// it instead of a synthetic diagonal. Empty when the route ends at a
    /// genuine off-runway node.
    pub lead_on: Vec<(f64, f64)>,
}

/// The active field's ILS facilities (earth_nav.dat) and per-runway platform
/// altitudes (CIFP approach legs), background-loaded per station — both are
/// full-file scans that must never stall the reply path. `None` while loading
/// or when the field has no ILS; vectors then fly the visual profile.
pub struct IlsFacts {
    pub ils: Vec<nav::AirportIls>,
    /// Glideslope-intercept ("descend to X until established") altitude per
    /// runway ident, ft MSL, where the CIFP has the approach.
    pub platform_ft: std::collections::HashMap<String, f64>,
}

static ILS_FACTS: Mutex<Option<std::sync::Arc<IlsFacts>>> = Mutex::new(None);

/// Kick off the background ILS + procedures load for the active airport.
/// Called by the station's owner at install time, like the taxi network.
pub fn spawn_ils_load(nav_path: String, cifp_dir: String, icao: String) {
    *ILS_FACTS.lock().unwrap_or_else(|e| e.into_inner()) = None;
    std::thread::spawn(move || match nav::airport_ils(&nav_path, &icao) {
        Ok(list) if !list.is_empty() => {
            let platform_ft: std::collections::HashMap<String, f64> = list
                .iter()
                .filter_map(|i| {
                    nav::ils_platform_alt_ft(&cifp_dir, &icao, &i.runway)
                        .map(|a| (i.runway.clone(), a))
                })
                .collect();
            tracing::info!(
                %icao,
                ils = list.len(),
                platforms = platform_ft.len(),
                "ILS data loaded: vectors can offer the ILS approach"
            );
            *ILS_FACTS.lock().unwrap_or_else(|e| e.into_inner()) =
                Some(std::sync::Arc::new(IlsFacts {
                    ils: list,
                    platform_ft,
                }));
        }
        Ok(_) => tracing::info!(%icao, "no ILS at this field; visual vectors only"),
        Err(e) => tracing::warn!(%icao, "ILS scan failed: {e:#}"),
    });
}

/// Kick off the background taxi-network load for the active airport.
/// Called by the station's owner at install time.
pub fn spawn_taxi_net_load(apt_path: String, icao: String) {
    *TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()) = None;
    std::thread::spawn(move || match nav::find_taxi_net(&apt_path, &icao) {
        Ok(Some(net)) => {
            tracing::info!(
                %icao,
                nodes = net.nodes.len(),
                edges = net.edges.len(),
                "taxi network loaded: clearances will carry named routes"
            );
            *TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()) = Some(std::sync::Arc::new(net));
        }
        Ok(None) => tracing::info!(%icao, "no taxi network in apt.dat; route-less taxi clearances"),
        Err(e) => tracing::warn!(%icao, "taxi network scan failed: {e:#}"),
    });
}

/// Plan the taxi route from the aircraft to the holding point of `ident`'s
/// runway end. `None` when there's no network (yet), no snap, or no path —
/// the clearance then falls back to the route-less phrasing.
/// Route an ARRIVAL from where it is to the NEAREST suitable parking stand —
/// the mirror of [`plan_taxi_route_to`] for taxi-in. Same generic Dijkstra; the
/// destination is a stand, so there's no runway hold-short trim (a stand is a
/// genuine off-runway end node), and the plan's `holding_point` carries the raw
/// stand name so the reply and the crew call it by name. Runway crossings on
/// the way in are still named. `None` when the field has no stands or neither
/// end snaps to the network.
fn plan_taxi_route_to_stand(
    runways: &[nav::Runway],
    lat: f64,
    lon: f64,
    psi_true: Option<f64>,
) -> Option<nav::TaxiRoutePlan> {
    let net = TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
    stand_route_on(&net, runways, lat, lon, psi_true)
}

/// True heading, for which way is forward off a runway.
const TRUE_PSI: &str = "sim/flightmodel/position/psi";

/// [`plan_taxi_route_to_stand`] on a given network, so the real apt.dat can
/// test it. ON a runway with a heading, the route starts with the drive off
/// it, FORWARD ([`nav::TaxiNet::plan_vacate`]), and the stand is the one
/// nearest where that drive ends: planned from the strip itself it snapped
/// to the nearest node whatever the heading and backtracked (EDLP 24,
/// 2026-09-24, see [`nav::after_vacate`]). Off a runway, no heading, or no
/// way off ahead: the plain route from here, as before.
fn stand_route_on(
    net: &nav::TaxiNet,
    runways: &[nav::Runway],
    lat: f64,
    lon: f64,
    psi_true: Option<f64>,
) -> Option<nav::TaxiRoutePlan> {
    let on = runways.iter().find(|r| on_runway(r, lat, lon).is_some());
    let vacate = match (on, psi_true.filter(|p| p.is_finite())) {
        (Some(r), Some(psi)) => {
            let zone = vec![r.ident_a.clone(), r.ident_b.clone()];
            net.plan_vacate(lat, lon, psi, &zone, nav::VACATE_CLEAR_M)
                .filter(|v| v.path.len() >= 2)
                .map(|v| (v, format!("{}/{}", r.ident_a, r.ident_b)))
        }
        _ => None,
    };
    let (slat, slon) = vacate
        .as_ref()
        .and_then(|(v, _)| v.path.last().copied())
        .unwrap_or((lat, lon));
    let stand = net.nearest_stand(slat, slon)?.clone();
    // To the STAND, not to the taxiway node nearest it — see `plan_to_stand`.
    let mut plan = net.plan_to_stand(slat, slon, &stand)?;
    plan.holding_point = stand.name.clone();
    // Crossings on the stand route only: the vacate's own runway is the
    // landing clearance's pavement, not a crossing.
    mark_runway_crossings(&mut plan, runways);
    if let Some((v, strip)) = vacate {
        sim::blackbox::record(
            "atc",
            serde_json::json!({
                "event": "stand_route_from_runway", "runway": strip,
                "exit": v.exit_name, "vacate_pts": v.path.len(),
                "stand": stand.name,
            }),
        );
        plan = nav::after_vacate(v, plan);
    }
    Some(plan)
}

/// Plan the drive off `runway` from where the rollout stopped. Reached only
/// through [`crate::party_line::plan_vacate`] — `TAXI_NET` stays station-private.
///
/// The active zone is asked for by BOTH ends of the strip, not just the one we
/// landed on: apt.dat 1204 rows are per runway END (an arrival zone for 09 and
/// another for 27 covering much the same pavement), and an edge marked only
/// for the opposite direction is still the pavement we are trying to get off.
pub(crate) fn plan_vacate_from(
    lat: f64,
    lon: f64,
    psi_true: f64,
    runway: &str,
) -> Option<nav::VacatePlan> {
    let net = TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
    let mut zone = vec![runway.to_string()];
    if let Some(r) = reciprocal_end(runway) {
        zone.push(r);
    }
    net.plan_vacate(lat, lon, psi_true, &zone, nav::VACATE_CLEAR_M)
}

/// The other end of the same strip: "09L" -> "27R", "36" -> "18". `None` when
/// the ident isn't a runway designator we can turn around.
pub fn reciprocal_end(id: &str) -> Option<String> {
    let digits: String = id.chars().take_while(|c| c.is_ascii_digit()).collect();
    let side: String = id[digits.len()..].to_string();
    let n: u32 = digits.parse().ok()?;
    if n == 0 || n > 36 {
        return None;
    }
    let opp = if n > 18 { n - 18 } else { n + 18 };
    let side = match side.as_str() {
        "L" => "R",
        "R" => "L",
        s => s,
    };
    Some(format!("{opp:02}{side}"))
}

/// Speak a parking-stand name: spell digit runs ("Stand 12" -> "stand one
/// two"), lowercase the rest, and prefix "stand" for a bare designator (LXGB
/// names its gates "1".."5", so this reads "stand five", not just "five") —
/// unless the name already says what kind of parking it is. `pub` so the crew's
/// auto-taxi calls the destination by the same name.
pub fn spoken_stand(name: &str) -> String {
    let spoken = name
        .split_whitespace()
        .map(|w| {
            if !w.is_empty() && w.chars().all(|c| c.is_ascii_digit()) {
                atis::spoken_digits(w)
            } else {
                w.to_ascii_lowercase()
            }
        })
        .collect::<Vec<_>>()
        .join(" ");
    if spoken.is_empty() {
        return spoken;
    }
    let names_the_kind = name.split_whitespace().any(|w| {
        matches!(
            w.to_ascii_lowercase().as_str(),
            "stand" | "gate" | "ramp" | "apron" | "bay" | "parking" | "tie-down" | "tiedown"
        )
    });
    if names_the_kind {
        spoken
    } else {
        format!("stand {spoken}")
    }
}

fn plan_taxi_route_to(
    runways: &[nav::Runway],
    ident: &str,
    lat: f64,
    lon: f64,
) -> Option<nav::TaxiRoutePlan> {
    let net = TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
    let r = strip(runways, ident)?;
    let (tlat, tlon) = if r.ident_a == ident {
        (r.lat_a, r.lon_a)
    } else {
        (r.lat_b, r.lon_b)
    };
    // The hold-short standoff scales with the runway's class: instrument
    // runways paint their holding positions much further out than the GA
    // default (Heathrow's CAT holds sit ~100 m from the centreline — the
    // 40 m stop parked a taxi on the 09L entry stub past the line,
    // visually ON the runway, 2026-07-20). ILS on either end of the strip
    // = instrument holds; otherwise width-scaled, floored at the default.
    // ILS facts still loading falls back to the GA figure — early over
    // absent.
    let instrument = ILS_FACTS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .is_some_and(|f| {
            f.ils
                .iter()
                .any(|i| i.runway == r.ident_a || i.runway == r.ident_b)
        });
    let standoff = if instrument {
        100.0
    } else {
        (r.width_m / 2.0 + 30.0).max(nav::HOLD_SHORT_STANDOFF_M)
    };
    let zone = [r.ident_a.clone(), r.ident_b.clone()];
    let mut plan = net.plan_to_runway(lat, lon, tlat, tlon, standoff, &zone, Some(r))?;
    mark_runway_crossings(&mut plan, runways);
    Some(plan)
}

/// Name the crossings the planner couldn't — geometry is the authority,
/// never the network flags, because apt.dat authors get crossings wrong two
/// ways: runway edges left UNNAMED (the planner's crossings list is named
/// edges only — the KPDK clearance never said "cross runway ...",
/// 2026-07-14), and crossing links drawn as plain TAXIWAY edges straight
/// across the strip (EGLL's N7: the route crossed an active 09R with no
/// crossing clause, no guide call, and no hold, 2026-07-20). Any leg whose
/// sampled points sit on a physical strip is renamed to the strip's
/// designator pair and runway-flagged — which feeds the clearance's "cross
/// runway ..." clause, the guide's "check it's clear, then cross" cue, AND
/// the cleared-crossing exemption in the incursion watch, all keyed on that
/// flag. The route's TAIL is exempt when taxiway-flagged: post-trim it ends
/// at the hold-short point NEAR the destination strip, which must not read as
/// crossing it.
///
/// How much tail that is. One network edge used to be the unit of a leg, so
/// exempting the last leg exempted the whole approach to the holding point;
/// legs are metres long now that routes are resampled onto the paint. This is
/// the standoff the trim itself works in — anything nearer the end than this
/// is the same final approach to the same hold.
const TAIL_EXEMPT_M: f64 = nav::HOLD_SHORT_STANDOFF_M;

fn mark_runway_crossings(plan: &mut nav::TaxiRoutePlan, runways: &[nav::Runway]) {
    let last = plan.seg.len().saturating_sub(1);
    // Distance from each leg's start to the end of the route, so the "we stop
    // short of the destination runway, we don't cross it" exemption covers
    // the whole approach to the holding point. It used to be `i == last`,
    // which was the same thing while a leg meant a network edge — since the
    // route is resampled onto the paint a leg is a few metres, and the last
    // one alone is a sliver that exempts nothing.
    let mut to_end: Vec<f64> = vec![0.0; plan.path.len()];
    for i in (0..plan.path.len().saturating_sub(1)).rev() {
        let (a, b) = (plan.path[i], plan.path[i + 1]);
        to_end[i] = to_end[i + 1] + crate::nav::haversine_nm(a.0, a.1, b.0, b.1) * 1852.0;
    }
    for i in 0..plan.seg.len() {
        let (name, is_runway) = plan.seg[i].clone();
        if is_runway && !name.is_empty() {
            continue; // the planner already counted this one
        }
        if !is_runway && (i == last || to_end.get(i).is_some_and(|d| *d < TAIL_EXEMPT_M)) {
            continue; // ends short of the destination runway, not across it
        }
        // Sample the leg every ~20 m (its midpoint alone can miss a strip
        // the leg only clips): first sampled point on a runway names it.
        let (a, b) = (plan.path[i], plan.path[i + 1]);
        let len_m = crate::nav::haversine_nm(a.0, a.1, b.0, b.1) * 1852.0;
        let steps = ((len_m / 20.0).ceil() as usize).max(1);
        let hit = (0..=steps).find_map(|k| {
            let t = k as f64 / steps as f64;
            let (plat, plon) = (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
            runways.iter().find(|r| on_runway(r, plat, plon).is_some())
        });
        if let Some(r) = hit {
            let full = format!("{}/{}", r.ident_a, r.ident_b);
            plan.seg[i] = (full.clone(), true);
            if !plan.crossings.contains(&full) {
                plan.crossings.push(full);
            }
        }
    }
}

/// A taxiway name spoken like a controller says it: designator letters go
/// NATO, digits are read out, real words stay words — "B2" → "Bravo two",
/// "N4W" → "November four Whiskey", "LINK27" → "Link two seven",
/// "Inner" stays "Inner". Per token: letter runs of 1-2 are designators
/// (NATO), 3+ are words; digit runs are always spoken.
pub fn spoken_taxiway(name: &str) -> String {
    let mut out: Vec<String> = Vec::new();
    for tok in name.split_whitespace() {
        let mut run = String::new();
        let flush = |run: &mut String, out: &mut Vec<String>| {
            if run.is_empty() {
                return;
            }
            if run.chars().all(|c| c.is_ascii_digit()) {
                out.push(atis::spoken_digits(run));
            } else if run.chars().count() <= 2 {
                out.push(tower::spoken_callsign(run));
            } else {
                // A real word: keep it, title-cased so ALL-CAPS apt.dat
                // names ("LINK") don't get spelled out by the TTS.
                let mut w = run.to_lowercase();
                if let Some(c) = w.get_mut(0..1) {
                    c.make_ascii_uppercase();
                }
                out.push(w);
            }
            run.clear();
        };
        let mut prev_digit: Option<bool> = None;
        for c in tok.chars() {
            let digit = c.is_ascii_digit();
            if prev_digit.is_some_and(|p| p != digit) {
                flush(&mut run, &mut out);
            }
            run.push(c);
            prev_digit = Some(digit);
        }
        flush(&mut run, &mut out);
    }
    out.join(" ")
}

/// The designator-and-number split of a taxiway name ("M3" → ("M", 3)) —
/// `None` when the name doesn't end in digits or doesn't start with them.
fn stem_num(name: &str) -> Option<(&str, i64)> {
    let idx = name.find(|c: char| c.is_ascii_digit())?;
    let (stem, num) = name.split_at(idx);
    if stem.is_empty() {
        return None;
    }
    Some((stem, num.parse().ok()?))
}

/// A via list spoken like a controller reads it: a run of three or more
/// same-designator, consecutively numbered segments collapses to its ends —
/// "C1, M1, M2, M3, M4, M5" becomes "Charlie one, Mike one to Mike five".
/// The apt.dat network names every stretch of pavement; a clearance reads
/// like a controller, not like the database (Madrid, 2026-07-13).
pub fn spoken_via(names: &[String]) -> Vec<String> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < names.len() {
        let mut j = i + 1;
        if let Some((stem, first)) = stem_num(&names[i]) {
            let mut prev = first;
            let mut step = 0i64;
            while j < names.len() {
                let Some((s, n)) = stem_num(&names[j]) else {
                    break;
                };
                let d = n - prev;
                if s != stem || d.abs() != 1 || (step != 0 && d != step) {
                    break;
                }
                step = d;
                prev = n;
                j += 1;
            }
            if j - i >= 3 {
                out.push(format!(
                    "{} to {}",
                    spoken_taxiway(&names[i]),
                    spoken_taxiway(&names[j - 1])
                ));
                i = j;
                continue;
            }
        }
        out.push(spoken_taxiway(&names[i]));
        i += 1;
    }
    out
}

/// The first designator of a runway-edge name ("09L/27R" → "09L") — the
/// end a crossing instruction names. `pub` for the copilot's taxi guide,
/// which phrases the same runway-edge names.
pub fn first_designator(name: &str) -> &str {
    name.split('/').next().unwrap_or(name).trim()
}

/// Distance from a point to a polyline, meters — the off-route check.
/// Whether the pilot stands where the cleared taxi route itself runs over a
/// runway — a crossing or backtrack the tower put in the clearance ("cross
/// runway two seven"). Geometry, not the apt.dat runway flag: authors leave
/// crossing edges unnamed and sometimes unflagged, and the flag adds nothing
/// the pavement test doesn't. The pilot must be within the off-route corridor
/// of the route AND the route's nearest point must itself be ON a runway —
/// so nosing past a hold-short point onto a runway the route only ends
/// NEAR still reads as an incursion (the route's nearest point there is the
/// holding point, off the pavement).
fn on_cleared_runway_crossing(lat: f64, lon: f64) -> bool {
    const CORRIDOR_M: f64 = 60.0;
    let nearest = {
        let route = TAXI_ROUTE.lock().unwrap_or_else(|e| e.into_inner());
        let Some(r) = route.as_ref() else {
            return false;
        };
        nearest_on_polyline(&r.path, lat, lon)
    };
    // Route lock dropped before taking the runway lock — no nesting.
    nearest.is_some_and(|(p, d)| d <= CORRIDOR_M && on_active_runway(p.0, p.1).is_some())
}

/// The closest point of a polyline to (lat, lon), with its distance in
/// meters. Flat-earth at taxi scales, like [`dist_to_polyline_m`].
fn nearest_on_polyline(path: &[(f64, f64)], lat: f64, lon: f64) -> Option<((f64, f64), f64)> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    path.windows(2)
        .map(|w| {
            let (ax, ay) = (
                (w[0].1 - lon) * m_per_deg_lon,
                (w[0].0 - lat) * m_per_deg_lat,
            );
            let (bx, by) = (
                (w[1].1 - lon) * m_per_deg_lon,
                (w[1].0 - lat) * m_per_deg_lat,
            );
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            let t = if len2 < 1.0 {
                0.0
            } else {
                (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0)
            };
            let (cx, cy) = (ax + t * dx, ay + t * dy);
            let plat = w[0].0 + t * (w[1].0 - w[0].0);
            let plon = w[0].1 + t * (w[1].1 - w[0].1);
            ((plat, plon), (cx * cx + cy * cy).sqrt())
        })
        .min_by(|x, y| x.1.total_cmp(&y.1))
}

fn dist_to_polyline_m(path: &[(f64, f64)], lat: f64, lon: f64) -> Option<f64> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    path.windows(2)
        .map(|w| {
            let (ax, ay) = (
                (w[0].1 - lon) * m_per_deg_lon,
                (w[0].0 - lat) * m_per_deg_lat,
            );
            let (bx, by) = (
                (w[1].1 - lon) * m_per_deg_lon,
                (w[1].0 - lat) * m_per_deg_lat,
            );
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            let t = if len2 < 1.0 {
                0.0
            } else {
                (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0)
            };
            let (cx, cy) = (ax + t * dx, ay + t * dy);
            (cx * cx + cy * cy).sqrt()
        })
        .min_by(|x, y| x.total_cmp(y))
}

/// Which controller COM1 is on, if any — the voice that narrates traffic.
pub fn tuned_controller(com1_khz: i64) -> Option<StationSpeaker> {
    CONTROLLER_FREQS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|(k, _)| (com1_khz - k).abs() <= 5)
        .map(|(_, s)| *s)
}

/// Which station COM1 hears — controllers OR the ATIS loop. Unlike
/// [`tuned_controller`] this resolves the ATIS frequency too, so the speech
/// arbiter can tell "on the ATIS broadcast" from "off frequency" and cut
/// live station audio the instant the pilot tunes away.
pub fn tuned_station(com1_khz: i64) -> Option<StationSpeaker> {
    tuned_controller(com1_khz).or_else(|| {
        ATIS.lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .filter(|a| (com1_khz - a.khz).abs() <= 5)
            .map(|_| StationSpeaker::Atis)
    })
}

/// The information letter in a call ("with information Alpha", "we have
/// information bravo", or just "information B").
pub fn heard_information(t: &str) -> Option<char> {
    let mut toks = t
        .split(|c: char| !c.is_alphanumeric())
        .filter(|w| !w.is_empty());
    while let Some(tok) = toks.next() {
        if tok != "information" {
            continue;
        }
        let next = toks.next()?;
        // A NATO word ("alpha") or a bare letter ("b").
        if next.len() == 1 {
            let c = next.chars().next()?.to_ascii_uppercase();
            if c.is_ascii_uppercase() {
                return Some(c);
            }
        }
        for letter in 'A'..='Z' {
            if atis::nato(letter).to_ascii_lowercase() == next {
                return Some(letter);
            }
        }
        return None;
    }
    None
}

/// An armed vectors-to-final plan — the approach controller's worksheet. Set
/// by [`Station::arm_vectors`], driven by [`vector_tick`] from the monitor's
/// poll loop (the same shape as the talk-down's `approach_guide::GUIDE`).
pub struct VectorPlan {
    pub callsign: String,
    pub thr_lat: f64,
    pub thr_lon: f64,
    /// Final approach course, degrees true.
    pub course_deg: f64,
    pub runway_spoken: String,
    /// Tower frequency, spoken — the handoff at the end of the vectors.
    pub tower_spoken: Option<String>,
    /// Intercept altitude, ft MSL.
    pub target_alt_ft: f64,
    /// ILS mode: the runway has a localizer, so the vectors end in "cleared
    /// ILS approach, report established" instead of the visual clearance.
    pub ils: bool,
    /// Where the vectors aim the final join, nm from the threshold. Visual:
    /// the stock 6. ILS: far enough out that the glideslope is still ABOVE
    /// the platform altitude there — intercepting from below is the whole
    /// discipline; a fixed 6 nm join under Heathrow's 2,500 ft platform
    /// delivered the aircraft on top of the slope.
    join_nm: f64,
    /// The ILS clearance has been spoken (it rides the intercept turn, once).
    cleared: bool,
    /// Phraseology dialect, carried from the station that armed the plan.
    dialect: Dialect,
    last_tx: Option<Instant>,
    last_heading: Option<i64>,
}

impl VectorPlan {
    /// The approach clearance has been spoken, so the vectors are finished and
    /// the crew owes the "established" report.
    pub(crate) fn is_cleared(&self) -> bool {
        self.cleared
    }
    /// The heading last actually TRANSMITTED, not an internal target — a crew
    /// complying with vectors must fly the number it was given. `None` before
    /// the first instruction goes out.
    pub(crate) fn last_heading_deg(&self) -> Option<i64> {
        self.last_heading
    }
}

/// How far out the vectors aim you onto final (the join point) for a VISUAL
/// approach; an ILS join is computed from its platform instead.
const JOIN_FINAL_NM: f64 = 6.0;

/// The ILS join distance: where a `gs_deg` slope reaches `platform_agl_ft`
/// above the field, plus a mile of level flight to capture from below —
/// never inside the visual join. Pure, so the geometry unit-tests.
fn ils_join_nm(platform_agl_ft: f64, gs_deg: f64) -> f64 {
    let slope_nm = platform_agl_ft.max(0.0) / (6076.12 * gs_deg.max(1.0).to_radians().tan());
    (slope_nm + 1.0).max(JOIN_FINAL_NM)
}

/// One tick of the approach controller, ~monitor rate. Returns an instruction
/// when one is due: a fresh heading when the required course has moved, the
/// approach clearance + tower handoff once established, silence otherwise.
/// Pure narration — the pilot flies. `track_deg` is the ground track (degrees
/// true, `None` = read failed); `now` is injected so the cadence unit-tests.
pub fn vector_tick(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    track_deg: Option<f64>,
    on_ground: bool,
    now: Instant,
) -> Option<String> {
    let mut guard = VECTORS.lock().unwrap_or_else(|e| e.into_inner());
    let plan = guard.as_mut()?;
    if on_ground {
        *guard = None; // landed (or armed on the ramp by mistake): vectors over
        return None;
    }

    let dist_thr = crate::nav::haversine_nm(lat, lon, plan.thr_lat, plan.thr_lon);
    let brg_thr = crate::nav::bearing_deg(lat, lon, plan.thr_lat, plan.thr_lon);
    // Established: on the approach side, on the extended centerline, inside the
    // join distance, AND actually flying the final course — the position wedge
    // alone would clear an aircraft crossing final at ninety degrees. A failed
    // track read doesn't block the clearance (best-effort, like every read).
    let off_course = sim::angles::abs_diff_deg(brg_thr, plan.course_deg);
    let aligned = track_deg.is_none_or(|t| sim::angles::abs_diff_deg(t, plan.course_deg) <= 30.0);
    // Established, two regimes. Beyond a mile: the position wedge — on the
    // approach side, near the extended centerline, inside the join, flying
    // the course. Inside a mile the threshold bearing is too twitchy for a
    // 15-degree wedge, and a tight base that rolls out on final at 0.9 nm
    // never satisfied the old 1.0 nm floor — the plan vectored a short-final
    // Cessna AWAY ("fly heading zero niner zero" at 272 feet, toward the
    // join point behind it) and touchdown then discarded the plan with no
    // clearance ever spoken, handing a compliant pilot the landing berate
    // (Gibraltar, 2026-07-13). Inside the mile the test is landing intent:
    // below the vector altitude, pointed down the runway, threshold still
    // broadly ahead.
    let established = if dist_thr >= 1.0 {
        dist_thr <= plan.join_nm + 2.0 && off_course <= 15.0 && aligned
    } else {
        alt_ft < plan.target_alt_ft && off_course <= 45.0 && aligned
    };
    if established {
        let handoff = plan
            .tower_spoken
            .as_ref()
            .map(|f| format!(" Contact Tower {f}."))
            .unwrap_or_default();
        // A short-final clearance rounds to "1 mile", never "0 miles".
        let miles = (dist_thr.round() as i64).max(1);
        let miles = format!("{miles} mile{}", if miles == 1 { "" } else { "s" });
        let msg = if plan.ils && plan.cleared {
            // Cleared on the intercept, now seen established: hand off. (If
            // the pilot reports established first, respond_approach does
            // this and the plan is already gone.)
            format!("{}, {miles} from touchdown.{handoff}", plan.callsign)
        } else if plan.ils {
            // Straight-in that never needed an intercept turn: clearance and
            // handoff together.
            format!(
                "{}, {miles} from touchdown. Cleared ILS approach runway {}.{handoff}",
                plan.callsign, plan.runway_spoken
            )
        } else {
            format!(
                "{}, {miles} from touchdown. Cleared visual approach runway {}.{handoff}",
                plan.callsign, plan.runway_spoken
            )
        };
        // Handed to the tower with an APPROACH clearance — the pilot still owes
        // a "final" report before the tower's landing clearance. Arm the crew
        // coach so a missed report doesn't become a landing-without-clearance
        // (the compliance watch clears this on the landing clearance and on
        // touchdown). Only when there's a tower to report to.
        if plan.tower_spoken.is_some() {
            *LANDING_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) =
                Some(plan.runway_spoken.clone());
        }
        *guard = None; // approach's job is done
        return Some(msg);
    }

    // Inside a mile and not (yet) landing: no heading. Any vector from here
    // aims at a join point BEHIND the aircraft — a reversal at circuit
    // height. Stay quiet and let the geometry develop; if they blow through
    // final, steering resumes beyond the mile.
    if dist_thr < 1.0 {
        return None;
    }

    // Steering, two regimes. Near the extended centerline on the approach
    // side: ONE honest intercept — a 30° cut toward the final course,
    // relaxing to the course itself as the centerline arrives. This used to
    // be a pursuit curve (correction ∝ cross-track); against a real pilot's
    // turn lag and the 25 s call cadence the gain wound up, marched the
    // headings 200→250, blew through the localizer, and then re-vectored a
    // reversal to the fallen-behind join point — an orbit, flown live at
    // Jerez 2026-07-12. Controllers give a cut, not a curve. Anywhere else:
    // direct to the join point, which is the downwind/base leg for free.
    let along_nm = dist_thr * off_course.to_radians().cos(); // + = approach side
    let xtk_nm = dist_thr * sim::angles::signed_diff_deg(brg_thr, plan.course_deg).to_radians().sin();
    let capturing = along_nm >= 1.0 && xtk_nm.abs() <= 2.5;
    // On the intercept after the ILS clearance, TRACKING the course: the
    // frequency goes quiet. Capturing the localizer is the pilot's job now;
    // a controller who keeps issuing headings against a coupling autopilot
    // fights it ("cleared ILS" followed by "fly heading 360", taped
    // 2026-07-11). But quiet needs BOTH near the centerline AND actually
    // flying its direction — a pilot sailing through the localizer on the
    // cut heading still gets the correction.
    if plan.ils && plan.cleared && capturing && xtk_nm.abs() <= 1.5 && aligned {
        return None;
    }
    let desired = if capturing {
        if xtk_nm.abs() <= 0.4 {
            plan.course_deg
        } else {
            (plan.course_deg + 35.0_f64.copysign(xtk_nm)).rem_euclid(360.0)
        }
    } else {
        // Aim BEYOND the join, not at it: the last leg then converges onto
        // the centerline's axis with room for the 35° cut to finish. Aiming
        // AT the join arrives perpendicular with no room, drops out of the
        // capture band at the bottom, and the re-vector back to a join now
        // BEHIND the aircraft is the other half of the Jerez orbit.
        let recip = (plan.course_deg + 180.0).rem_euclid(360.0);
        let (join_lat, join_lon) =
            crate::nav::project_latlon(plan.thr_lat, plan.thr_lon, recip, plan.join_nm + 2.0);
        crate::nav::bearing_deg(lat, lon, join_lat, join_lon)
    };
    let heading10 = {
        let h = ((desired / 10.0).round() as i64 * 10).rem_euclid(360);
        if h == 0 {
            360
        } else {
            h
        }
    };

    // Cadence: speak immediately the first time, then only when the required
    // heading has genuinely moved — a real drift after 25 s, a big cut after
    // 8 s. Holding the assigned heading stays silent, like a real frequency.
    // The drift floor is 20°: headings are rounded to 10°, so a 10° "move" is
    // one quantization step — re-vectoring it every 25 s trimmed a stable
    // downwind 070→080→070 (taped 2026-07-12). A controller who keeps issuing
    // 10-degree corrections is chasing the aircraft; a real one picks a
    // heading that works and lets the geometry develop (the 35° intercept cut
    // always clears the 30° fast rule, so the turn onto final is never held).
    let elapsed = plan
        .last_tx
        .map(|t| now.duration_since(t).as_secs_f64())
        .unwrap_or(f64::MAX);
    let moved = plan
        .last_heading
        .map(|h| sim::angles::abs_diff_deg(h as f64, heading10 as f64))
        .unwrap_or(360.0);
    let due = plan.last_tx.is_none()
        || (elapsed >= 25.0 && moved >= 20.0)
        || (elapsed >= 8.0 && moved >= 30.0);
    if !due {
        return None;
    }
    plan.last_tx = Some(now);
    plan.last_heading = Some(heading10);

    let mut msg = format!(
        "{}, fly heading {}{}",
        plan.callsign,
        atis::spoken_digits(&format!("{heading10:03}")),
        // CAP 413 headings carry the unit; FAA's don't.
        if plan.dialect == Dialect::Icao {
            " degrees"
        } else {
            ""
        }
    );
    if alt_ft > plan.target_alt_ft + 400.0 {
        msg.push_str(&match plan.dialect {
            Dialect::Icao => {
                format!(
                    ", descend to altitude {} feet",
                    spoken_alt(plan.target_alt_ft)
                )
            }
            Dialect::Faa => format!(", descend and maintain {}", spoken_alt(plan.target_alt_ft)),
        });
    }
    msg.push('.');
    // The ILS clearance rides the intercept turn, once — the heading that
    // captures the localizer is the one that comes with "cleared ILS".
    if plan.ils && !plan.cleared && capturing {
        plan.cleared = true;
        msg.push_str(&match plan.dialect {
            Dialect::Icao => format!(
                " Cleared ILS approach runway {}, report established.",
                plan.runway_spoken
            ),
            // The FAA PTAC: position was the vector, this is turn-altitude-
            // clearance in one breath.
            Dialect::Faa => format!(
                " Maintain {} until established, cleared ILS runway {} approach.",
                spoken_alt(plan.target_alt_ft),
                plan.runway_spoken
            ),
        });
    }
    Some(msg)
}

/// The nearest airborne factor from the TCAS picture: range, clock position,
/// and relative altitude (positive = traffic above us), rounded to the
/// hundred feet a pilot scans in.
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficFactor {
    pub nm: f64,
    pub clock: i32,
    pub ft: f64,
}

/// The nearest TCAS target inside 6 nm and ±1,500 ft; `None` = no factor.
/// Reads the sim's TCAS arrays, which any traffic source (LiveTraffic,
/// default AI aircraft) populates — ATC narrates traffic, never controls it.
pub async fn nearest_traffic_factor(xp: SimView<'_>) -> Option<TrafficFactor> {
    let bear = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_bearing_degs")
        .await
        .ok()?;
    let dist = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_distance_mtrs")
        .await
        .ok()?;
    let alt = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_altitude_mtrs")
        .await
        .ok()?;
    // Weight-on-wheels, same slot indexing as the indicator arrays: used to drop
    // ground traffic. Optional — an empty read fails open (announce), since the
    // ±1,500 ft gate already excludes ground targets when we're up at altitude.
    let wow = xp
        .get_float_array(datarefs::TCAS_WOW)
        .await
        .unwrap_or_default();
    select_traffic_factor(&dist, &bear, &alt, &wow)
}

/// Pick the nearest airborne factor from the parallel TCAS indicator arrays.
/// Slot 0 is own ship. A target with weight on wheels is SKIPPED — a parked or
/// taxiing aircraft at the field sits within 6 nm and (from pattern height)
/// ±1,500 ft, and was being called "traffic, three o'clock, same altitude" on
/// the ramp (2026-07-24). Pure, so the ground filter is unit-tested.
fn select_traffic_factor(
    dist: &[f64],
    bear: &[f64],
    alt: &[f64],
    wow: &[f64],
) -> Option<TrafficFactor> {
    let (d, b, a) = (1..dist.len())
        .filter(|&i| dist[i] > 1.0)
        // On the ground = not an airborne factor. Fail open when unfed.
        .filter(|&i| wow.get(i).is_none_or(|w| *w < 0.5))
        .map(|i| {
            (
                dist[i],
                *bear.get(i).unwrap_or(&0.0),
                *alt.get(i).unwrap_or(&0.0),
            )
        })
        .filter(|(d, _, a)| *d < 6.0 * 1852.0 && a.abs() < 1500.0 / 3.28084)
        .min_by(|x, y| x.0.total_cmp(&y.0))?;
    let clock = match (((b + 15.0).rem_euclid(360.0)) / 30.0) as i32 {
        0 => 12,
        c => c,
    };
    Some(TrafficFactor {
        nm: d / 1852.0,
        clock,
        ft: ((a * 3.28084) / 100.0).round() * 100.0,
    })
}

/// The relative-altitude clause both traffic calls share.
fn vert_phrase(ft: f64) -> String {
    if ft.abs() < 300.0 {
        "same altitude".to_string()
    } else if ft > 0.0 {
        format!("{ft:.0} feet above")
    } else {
        format!("{:.0} feet below", ft.abs())
    }
}

/// Factor traffic near the vectored arrival, spoken as a controller advisory
/// ("traffic, two o'clock, three miles, five hundred feet above").
/// `None` = no factor, frequency stays quiet.
pub async fn traffic_advisory(xp: SimView<'_>) -> Option<String> {
    let f = nearest_traffic_factor(xp).await?;
    Some(advisory_line(&f))
}

/// Is `now` worth calling again after `called`? A controller calls a
/// target once and again only when it has become MORE of a factor: two
/// miles closer, or 500 ft less between us. At the old 45 s cadence one
/// diverging target was called ten times across a cruise ("3 o'clock, 6
/// miles, 900 feet below", then 700, 1,000, 1,400 below; LIPK-LIPR
/// 2026-09-26). Pure.
pub fn traffic_worth_recalling(called: &TrafficFactor, now: &TrafficFactor) -> bool {
    now.nm <= called.nm - 2.0 || called.ft.abs() - now.ft.abs() >= 500.0
}

/// The advisory's words for a factor.
pub fn advisory_line(f: &TrafficFactor) -> String {
    // A mile is "a mile": 1.0 to 1.5 nm rounds to one, and "1 miles" was
    // said twice on one approach (EGBB 15, sortie 1790601822-full_auto,
    // +3194.7 and +3240.1, 2026-09-28, #155).
    let range = match f.nm {
        nm if nm < 1.0 => "under a mile".to_string(),
        nm if nm < 1.5 => "a mile".to_string(),
        nm => format!("{nm:.0} miles"),
    };
    format!(
        "traffic, {} o'clock, {range}, {}.",
        f.clock,
        vert_phrase(f.ft)
    )
}

/// Is this factor a NEAR MISS — unsafe proximity, not just a factor to scan
/// for? Half a mile and 500 ft is the NMAC line; the advisory tier has been
/// narrating the target since 6 nm, so by here the calm call has already
/// failed to separate us.
pub fn is_near_miss(f: &TrafficFactor) -> bool {
    f.nm < 0.5 && f.ft.abs() < 500.0
}

/// Which way OUT of a near miss: vertical when the intruder is clearly above
/// or below (go the other way), lateral when co-altitude — away from the
/// traffic's side, right for head-on (the right-of-way rule every pilot
/// already knows), and vertical again for one directly behind, where a turn
/// keeps it in the blind spot.
fn avoidance(clock: i32, ft: f64) -> &'static str {
    if ft >= 100.0 {
        "descend"
    } else if ft <= -100.0 {
        "climb"
    } else {
        match clock {
            12 => "turn right",
            1..=5 => "turn left",
            7..=11 => "turn right",
            _ => "climb",
        }
    }
}

/// The near-miss call: the controller's safety alert (FAA 7110.65 shape,
/// "traffic alert ... advise you ... immediately") when a frequency is tuned,
/// the crew's shout when nobody is. Advisory words only, both ways — the
/// copilot never maneuvers uncommanded.
pub fn near_miss_line(f: &TrafficFactor, crew: bool) -> String {
    let act = avoidance(f.clock, f.ft);
    if crew {
        format!(
            "Traffic! {} o'clock, half a mile, {}. {act}, now.",
            f.clock,
            vert_phrase(f.ft)
        )
    } else {
        format!(
            "traffic alert, {} o'clock, less than a mile, {}, advise you {act} immediately.",
            f.clock,
            vert_phrase(f.ft)
        )
    }
}

/// An altitude spoken controller-style: 2000 → "two thousand", 1500 → "one
/// thousand five hundred".
fn spoken_alt(ft: f64) -> String {
    atis::spoken_alt(ft)
}

/// Outcome of a named-station tuning request ("tune the tower").
pub enum Tune {
    /// Set COM1 to this kHz; `label` is spoken in the readback.
    Found { label: String, khz: i64 },
    /// The airport has no such frequency — decline by name, don't guess.
    Missing(String),
}

/// Every comm service a field can publish, in briefing order (the order a
/// pilot works them: information first, then the ground-to-air chain).
const SERVICES: &[(&str, u32)] = &[
    ("atis", nav::FREQ_ATIS),
    ("clearance", nav::FREQ_CLEARANCE),
    ("ground", nav::FREQ_GROUND),
    ("tower", nav::FREQ_TOWER),
    ("approach", nav::FREQ_APPROACH),
    ("departure", nav::FREQ_DEPARTURE),
    ("unicom", nav::FREQ_UNICOM),
];

/// The field's published services in briefing order, for the dashboard's
/// radio panel: (service name, frequency in kHz). The same rows the spoken
/// briefing reads, so screen and voice can never disagree.
pub fn services(apt: &nav::AirportInfo) -> Vec<(String, i64)> {
    SERVICES
        .iter()
        .filter_map(|(word, kind)| apt.freq(*kind).map(|f| (word.to_string(), f.khz)))
        .collect()
}

/// Is this a "what frequencies are here" / "who can I talk to" question?
/// Crew knowledge, not a radio call — checked after the tuning verbs so
/// "tune the tower" still tunes. Deterministic on purpose: the answer is a
/// list read from apt.dat, exactly the kind of question the LLM must never
/// improvise.
pub fn freq_query(t: &str) -> bool {
    let interrogative = t.contains("what")
        || t.contains("which")
        || t.contains("list")
        || t.contains("any")
        || t.contains("available");
    let subject = t.contains("frequenc") || t.contains("station");
    let talk =
        t.contains("who") && (t.contains("talk") || t.contains("call") || t.contains("contact"));
    (interrogative && subject) || talk
}

impl Station {
    /// The spoken frequency briefing: every service this field publishes, plus
    /// what COM1 is on right now (named when it matches a service). Pure —
    /// the caller reads COM1 (see [`Station::freq_briefing`]) so this tests
    /// without a simulator.
    pub fn briefing_text(&self, com1_khz: Option<i64>) -> String {
        let parts: Vec<String> = SERVICES
            .iter()
            .filter_map(|(word, kind)| {
                self.apt
                    .freq(*kind)
                    .map(|f| format!("{word} {}", atis::spoken_mhz(f.khz)))
            })
            .collect();
        let mut line = if parts.is_empty() {
            format!(
                "{} has no published frequencies.",
                spoken_airport(&self.apt.name)
            )
        } else {
            format!("{}: {}.", spoken_airport(&self.apt.name), parts.join(", "))
        };
        if let Some(com1) = com1_khz {
            // Same 5 kHz tolerance as tuned_service: 8.33 channel names can
            // sit 5 kHz off the carrier. Some fields publish ground and tower
            // on the SAME frequency (8 in the 2026-07 apt.dat, military
            // mostly) — name the service that would actually answer, in
            // tuned_service's precedence, not the first row in briefing order.
            let matches: Vec<&str> = SERVICES
                .iter()
                .filter_map(|(word, kind)| {
                    self.apt
                        .freq(*kind)
                        .filter(|f| (f.khz - com1).abs() <= 5)
                        .map(|_| *word)
                })
                .collect();
            let tuned = matches
                .iter()
                .find(|w| matches!(**w, "tower" | "approach" | "departure"))
                .or_else(|| matches.first())
                .copied();
            line.push_str(&match tuned {
                Some(w) => format!(" Com one is on the {w}."),
                None => format!(" Com one is on {}.", atis::spoken_mhz(com1)),
            });
        }
        line
    }

    /// [`Station::briefing_text`] with COM1 read live. A failed read just
    /// drops the "com one is on" sentence — the list still answers.
    pub async fn freq_briefing(&self, xp: SimView<'_>) -> String {
        let com1 = xp
            .get_float(datarefs::COM1_FREQ_KHZ)
            .await
            .ok()
            .map(|v| v.round() as i64);
        self.briefing_text(com1)
    }

    /// The crew's note after a keyed call nobody answered while this station
    /// is in range. The full directory ([`Station::briefing_text`]) answers
    /// the QUESTION "what frequencies are there" — after dead air the only
    /// useful fact is who WILL answer (all six services recited after one
    /// void call on ground read as noise, 2026-07-20). Only tower, approach
    /// and departure are ever manned here; a COM1 sitting on a published but
    /// unmanned row (ground, clearance, ATIS, unicom) gets told so by name.
    /// Pure — the caller reads COM1 and the air/ground state.
    pub fn void_note(&self, com1_khz: Option<i64>, airborne: bool) -> String {
        let (word, khz) = match self.approach_khz.filter(|_| airborne) {
            Some(k) => ("approach", k),
            None => ("tower", self.tower_khz),
        };
        let apt = spoken_airport(&self.apt.name);
        let tuned = com1_khz.and_then(|c| {
            SERVICES.iter().find_map(|(w, kind)| {
                self.apt
                    .freq(*kind)
                    .filter(|f| (f.khz - c).abs() <= 5)
                    .map(|_| *w)
            })
        });
        match tuned {
            Some("atis") => format!(
                "That's the ATIS, it only broadcasts. Call {apt} {word}, {}.",
                atis::spoken_mhz(khz)
            ),
            Some(w) if w != word => format!(
                "Nobody's manning {w} here. Call {apt} {word}, {}.",
                atis::spoken_mhz(khz)
            ),
            _ => format!("Try {apt} {word} on {}.", atis::spoken_mhz(khz)),
        }
    }

    /// Deterministic named-station tuning: "tune/select/switch to/contact the
    /// tower|atis|ground|approach|departure" (or generic "atc"/"station" =
    /// tower) resolved against this airport's apt.dat frequencies. Requires a
    /// tuning verb, so radio chatter that merely mentions a station never
    /// re-tunes the radio. `None` = not a tuning request at all.
    pub fn tune_request(&self, t: &str) -> Option<Tune> {
        // STT warts for station names (all live 2026-07-20): ATIS spelled
        // letter-by-letter or heard as "atheist", tower heard as "tau".
        // Nobody asks for any of these otherwise.
        let t = &t
            .replace("a t i s", "atis")
            .replace("atheist", "atis")
            .replace("tau frequency", "tower frequency");
        // Flight-parameter talk that happens to name a station ("change to
        // approach speed", "select approach flaps") is never a tuning request.
        if t.contains("speed")
            || t.contains("knots")
            || t.contains("flaps")
            || t.contains("heading")
            || t.contains("altitude")
        {
            return None;
        }
        let verb = t.contains("tune")
            || t.contains("switch to")
            || t.contains("select")
            || t.contains("contact")
            || t.contains("change to")
            || (t.contains("set") && t.contains("com"))
            // "change/switch frequency to the tower" — but only when the
            // sentence is about the radio: bare "change"/"switch" are too
            // promiscuous ("switch the landing light").
            || ((t.contains("change") || t.contains("switch"))
                && (t.contains("freq") || t.contains("com") || t.contains("radio")));
        // A short bare "<station> frequency" is a tuning request even with
        // the verb clipped by STT ("To tower frequency." lost its "change",
        // live 2026-07-20). Short only: a real readback quoting a frequency
        // carries digits and runs longer.
        let bare = t.contains("frequency") && t.split_whitespace().count() <= 4;
        if !verb && !bare {
            return None;
        }
        let named = SERVICES.iter().find(|(w, _)| t.contains(w));
        // Generic "tune atc" / "tune the station" means the tower.
        let (word, kind) = match named {
            Some((w, k)) => (*w, *k),
            None if t.contains("atc") || t.contains("station") => ("tower", nav::FREQ_TOWER),
            None => return None,
        };
        Some(match self.apt.freq(kind) {
            Some(f) => Tune::Found {
                label: format!("{} {}", spoken_airport(&self.apt.name), word),
                khz: f.khz,
            },
            None => Tune::Missing(format!(
                "{} has no {} frequency.",
                spoken_airport(&self.apt.name),
                word
            )),
        })
    }
}

/// Wind spoken tower-style ("zero niner zero degrees, three knots" / "calm").
fn spoken_wind(from_deg: f64, kts: f64) -> String {
    if kts < 3.0 {
        return "calm".to_string();
    }
    let dir = ((from_deg / 10.0).round() as i64 * 10).rem_euclid(360);
    let dir = if dir == 0 { 360 } else { dir };
    format!(
        "{} degrees, {} knots",
        atis::spoken_digits(&format!("{dir:03}")),
        atis::spoken_digits(&format!("{kts:.0}"))
    )
}

#[cfg(test)]
#[path = "station_tests.rs"]
mod tests;
