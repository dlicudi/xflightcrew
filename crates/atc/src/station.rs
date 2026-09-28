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
    REGION_HANDOFF, SQUAWK_NUDGE, TAKEOFF_HOLD, TAXI_ROUTE, TAXI_TO_STAND, VECTORS, ZONE_CHALLENGE,
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
        *ZONE_CHALLENGE.lock().unwrap_or_else(|e| e.into_inner()) = false;
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
        if out_of_reach() {
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
                circuit: Some(tower::CircuitOffer {
                    height_ft: tower::circuit_height_ft(self.apt.elev_ft),
                    faa: self.dialect == Dialect::Faa,
                }),
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
        if service == Service::Tower
            && matches!(intent, tower::Intent::RequestTaxi | tower::Intent::RequestRunway)
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
        if std::mem::take(&mut self.stated_on_strip) && !self.runway_committed() {
            if let Some(ident) = self.stated_ident() {
                let into_stream = [arrival_end, departure_end]
                    .into_iter()
                    .flatten()
                    .any(|end| ends_oppose(&self.apt.runways, end, &ident));
                let hw = nav::headwind_for(&self.apt.runways, &ident, wind_from, wind_kts);
                let best = nav::best_runway_for_wind(&self.apt.runways, wind_from, wind_kts);
                if !into_stream
                    && stated_runway_stands(&self.apt.runways, &ident, hw, best, wind_from, wind_kts)
                {
                    self.stated_runway = None;
                    self.runway_in_use = Some(ident.clone());
                    return ident;
                }
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
        if !on_ground {
            self.ifr_runway = None;
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
    if out_of_reach() {
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
    if out_of_reach() {
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
static FIELD_DIR: Mutex<Vec<(String, String, (f64, f64))>> = Mutex::new(Vec::new());

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
    *LANDING_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *PENDING_LANDING_RELEASE
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = None;
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
                let off_a = crate::nav::angle_diff_deg(h, r.heading_a_deg()).abs();
                let off_b = crate::nav::angle_diff_deg(h, r.heading_b_deg()).abs();
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
        (Some(ha), Some(hb)) => angle_diff_deg(ha, hb).abs() > 120.0,
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
    let da = angle_diff_deg(track, r.heading_a_deg()).abs();
    let db = angle_diff_deg(track, r.heading_b_deg()).abs();
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
    if out_of_reach() {
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
    // ...and the order itself, for every crew loop that flies it.
    GO_AROUND_ORDERS.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    // An occupant that will not leave on its own clock (standing, or using
    // the other direction) would send the next approach round the same way:
    // the tower re-sequences now (issue #162). Only a flight on the
    // frequency can be told.
    let expect = if tuned {
        resequence_after_go_around(targets, &runway_ident, &exits, now)
    } else {
        None
    };
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
            if !same_strip && occupied.iter().any(|o| *o == r.ident_a) {
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
                angle_diff_deg(landing_hdg, hdg).abs(),
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
    if on_ground || out_of_reach() {
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
        |hdg: f64| (wind_kts * nav::angle_diff_deg(wind_from, hdg).to_radians().sin()).abs();
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
    hw >= -STATED_RUNWAY_TAILWIND_KTS
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
    if out_of_reach() {
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
fn reciprocal_end(id: &str) -> Option<String> {
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
    let off_course = angle_diff_deg(brg_thr, plan.course_deg).abs();
    let aligned = track_deg.is_none_or(|t| angle_diff_deg(t, plan.course_deg).abs() <= 30.0);
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
    let xtk_nm = dist_thr * angle_diff_deg(brg_thr, plan.course_deg).to_radians().sin();
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
        .map(|h| angle_diff_deg(h as f64, heading10 as f64).abs())
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

/// Smallest signed angle a−b, degrees, in −180..180.
fn angle_diff_deg(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
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
mod tests {
    use super::*;

    #[test]
    fn traffic_is_called_again_only_when_it_is_more_of_a_factor() {
        let f = |nm: f64, ft: f64| TrafficFactor { nm, clock: 3, ft };
        let called = f(6.0, -900.0);
        assert!(!traffic_worth_recalling(&called, &f(6.0, -700.0)), "200 ft closer is the same call");
        assert!(!traffic_worth_recalling(&called, &f(6.0, -1400.0)), "diverging");
        assert!(traffic_worth_recalling(&called, &f(4.0, -900.0)), "two miles closer");
        assert!(traffic_worth_recalling(&called, &f(5.5, -300.0)), "600 ft less between us");
    }

    #[test]
    fn traffic_advisory_skips_ground_targets() {
        // Slot 0 = own ship. Two targets both 2 nm off at same altitude: slot 1
        // is on the ground (weight on wheels), slot 2 airborne and slightly
        // farther. The advisory must factor the AIRBORNE one, never the parked
        // aircraft on the ramp (the 2026-07-24 bug).
        let two_nm = 2.0 * 1852.0;
        let dist = vec![0.0, two_nm, two_nm + 200.0];
        let bear = vec![0.0, 90.0, 180.0];
        let alt = vec![0.0, 0.0, 0.0];
        let wow = vec![0.0, 1.0, 0.0]; // slot 1 on the ground
        let f = select_traffic_factor(&dist, &bear, &alt, &wow).expect("an airborne factor");
        assert_eq!(
            f.clock, 6,
            "the airborne target at 180°, not the ground one at 90°"
        );

        // ONLY a ground target in range → no advisory at all.
        let f = select_traffic_factor(&[0.0, two_nm], &[0.0, 90.0], &[0.0, 0.0], &[0.0, 1.0]);
        assert!(f.is_none(), "a lone ground target is not a factor");

        // No WOW feed → fail open (announce), so traffic isn't silently lost.
        let f = select_traffic_factor(&[0.0, two_nm], &[0.0, 90.0], &[0.0, 0.0], &[]);
        assert!(f.is_some(), "unfed WOW announces rather than suppressing");
    }

    #[test]
    fn garbled_flight_ids_never_panic_the_telephony() {
        // Flight ids come off raw TCAS bytes through from_utf8_lossy, so a
        // garbage feed hands us replacement characters whose byte boundaries
        // sit exactly where the airline-prefix slices used to cut. A bad id
        // must degrade to NATO spelling, never panic the monitor thread.
        for id in [
            "A\u{FFFD}12",
            "B\u{FFFD}306",
            "\u{FFFD}\u{FFFD}",
            "é1",
            "BÉ306",
        ] {
            let spoken = spoken_ghost(id);
            assert!(!spoken.contains('\u{FFFD}'), "{id:?}: {spoken:?}");
            let _ = is_airline(id);
        }
        // The honest ids still resolve, both spellings.
        assert_eq!(spoken_ghost("BAW306"), "Speedbird three zero six");
        assert_eq!(spoken_ghost("BA306"), "Speedbird three zero six");
        assert!(is_airline("BAW306") && is_airline("BA306"));
        assert!(!is_airline("N172SP"));
    }

    #[test]
    fn discrete_squawks_are_valid_and_deterministic() {
        for cs in ["November one seven two Sierra Papa", "G-ABCD", "x"] {
            let code = discrete_squawk(cs);
            assert_eq!(code, discrete_squawk(cs), "stable per callsign");
            assert!((4500..=4577).contains(&code), "{code} outside the block");
            // Every digit octal (transponders have no 8s or 9s).
            assert!(
                code.to_string().chars().all(|c| ('0'..='7').contains(&c)),
                "{code}"
            );
            // Clear of every reserved code by construction.
            assert!(![0, 1200, 2000, 7000, 7500, 7600, 7700].contains(&code));
        }
    }

    #[test]
    fn squawk_watch_calls_radar_contact_only_when_the_code_shows() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // The code is derived from the callsign, and a watch is only re-armed
        // for a DIFFERENT code: a watch left by an earlier test with this
        // callsign (armed by Departure, already nudged) was this test's watch
        // about one run in four, by test order.
        *SQUAWK_WATCH.lock().unwrap() = None;
        *SQUAWK_NUDGE.lock().unwrap() = None;
        let mut st = lxgb();
        let spoken = st.assign_squawk(
            "November one seven two Sierra Papa",
            StationSpeaker::Approach,
        );
        let code = st.assigned_squawk.expect("code issued");
        assert_eq!(spoken, atis::spoken_digits(&code.to_string()));
        // Same flight asks again: same code, watch not reset.
        assert_eq!(
            st.assign_squawk(
                "November one seven two Sierra Papa",
                StationSpeaker::Approach
            ),
            spoken
        );

        let t0 = Instant::now();
        // Still on the old code, inside the grace period: silence.
        assert!(squawk_tick(Some(7000), true, StationSpeaker::Approach, t0).is_none());
        // Grace expired, still wrong: the nudge, once, from the assigner.
        let late = t0 + std::time::Duration::from_secs(25);
        let (line, who) =
            squawk_tick(Some(7000), true, StationSpeaker::Approach, late).expect("nudge");
        assert!(line.contains("confirm squawk"), "{line}");
        assert!(matches!(who, StationSpeaker::Approach));
        // The crew hears it as data: the code to dial, no recycle.
        assert_eq!(
            crate::party_line::take_squawk_nudge(),
            Some(crate::party_line::SquawkNudge { code, recycle: false })
        );
        assert_eq!(crate::party_line::take_squawk_nudge(), None, "drained once");
        // Immediately again: quiet (nudge cooldown).
        assert!(squawk_tick(Some(7000), true, StationSpeaker::Approach, late).is_none());
        // Right code but transponder on standby: not replying, no contact —
        // and the nudge names the REAL problem, never "confirm squawk" (the
        // digits are the one thing that's right; Gatwick 2026-07-13).
        let later = late + std::time::Duration::from_secs(50);
        let (line, _) =
            squawk_tick(Some(code), false, StationSpeaker::Approach, later).expect("standby nudge");
        assert!(
            line.contains("negative radar contact, recycle transponder"),
            "{line}"
        );
        assert!(!line.contains("confirm squawk"), "{line}");
        assert_eq!(
            crate::party_line::take_squawk_nudge(),
            Some(crate::party_line::SquawkNudge { code, recycle: true }),
            "the crew is told to recycle"
        );
        // Code set and replying: "radar contact", exactly once.
        let (line, _) = squawk_tick(
            Some(code),
            true,
            StationSpeaker::Approach,
            later + std::time::Duration::from_secs(50),
        )
        .expect("radar contact");
        assert!(line.contains("radar contact"), "{line}");
        assert!(squawk_tick(
            Some(code),
            true,
            StationSpeaker::Approach,
            later + std::time::Duration::from_secs(60)
        )
        .is_none());
        assert!(
            !squawk_watch_active(),
            "satisfied watch stops the dataref reads"
        );
        *SQUAWK_WATCH.lock().unwrap() = None;
    }

    #[test]
    fn departure_clearance_assigns_a_discrete_code_at_radar_fields() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *SQUAWK_WATCH.lock().unwrap() = None;
        *ASSIGNED_SQUAWK.lock().unwrap() = None;
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 8.0,
        };
        // LXGB has an approach service: the zone clearance carries the code.
        let mut st = lxgb();
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            true,
            false,
            None,
            false,
            None,
        );
        let r = st.respond(
            "",
            Service::Tower,
            tower::Intent::RequestClearance,
            &ctx,
            wx,
        );
        let code = st.assigned_squawk.expect("discrete code issued");
        let spoken = atis::spoken_digits(&code.to_string());
        assert!(r.contains(&spoken), "{r}");
        assert!(!r.contains("seven zero zero zero"), "{r}");
        assert!(SQUAWK_WATCH.lock().unwrap().is_some(), "radar watch armed");
        // ...and published for the crew to dial into the transponder.
        assert_eq!(
            *ASSIGNED_SQUAWK.lock().unwrap(),
            Some(code),
            "assigned code published to the party line"
        );
        // A tower-only strip stays on the VFR conspicuity and publishes nothing.
        *SQUAWK_WATCH.lock().unwrap() = None;
        *ASSIGNED_SQUAWK.lock().unwrap() = None;
        let mut st = lxgb();
        st.approach_khz = None;
        st.departure_khz = None;
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            true,
            false,
            None,
            false,
            None,
        );
        let r = st.respond(
            "",
            Service::Tower,
            tower::Intent::RequestClearance,
            &ctx,
            wx,
        );
        assert!(r.contains("seven zero zero zero"), "{r}");
        assert!(st.assigned_squawk.is_none());
        assert!(SQUAWK_WATCH.lock().unwrap().is_none());
        assert!(
            ASSIGNED_SQUAWK.lock().unwrap().is_none(),
            "no discrete code, nothing published"
        );
        *CLEARANCE.lock().unwrap() = None;
        *ASSIGNED_SQUAWK.lock().unwrap() = None;
    }

    #[test]
    fn tuned_station_resolves_controllers_and_the_atis() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CONTROLLER_FREQS.lock().unwrap() = vec![
            (118105, StationSpeaker::Tower),
            (128500, StationSpeaker::Approach),
        ];
        *ATIS.lock().unwrap() = Some(AtisState {
            khz: 125655,
            airport: "Jerez".into(),
            runways: Vec::new(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
        // Controllers resolve (within the 5 kHz tolerance).
        assert_eq!(tuned_station(118105), Some(StationSpeaker::Tower));
        assert_eq!(tuned_station(128500), Some(StationSpeaker::Approach));
        // The ATIS loop resolves too — this is what tuned_controller misses,
        // so live ATIS audio isn't wrongly cut while you're ON its frequency.
        assert_eq!(tuned_station(125655), Some(StationSpeaker::Atis));
        assert_eq!(
            tuned_controller(125655),
            None,
            "controllers-only misses ATIS"
        );
        // A real frequency that's none of them → off all stations (→ cut).
        assert_eq!(tuned_station(122800), None);
        *CONTROLLER_FREQS.lock().unwrap() = Vec::new();
        *ATIS.lock().unwrap() = None;
    }

    #[test]
    fn squawk_check_repeats_the_assigned_code() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 8.0,
        };
        let mut st = lxgb();
        // Before any radar service: the VFR conspicuity.
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            false,
            false,
            None,
            false,
            None,
        );
        let r = st.twr.handle(tower::Intent::SquawkCheck, &ctx);
        assert!(r.contains("seven zero zero zero"), "{r}");
        // Flight following assigns the discrete code...
        let r = st.respond_approach(tower::Intent::RequestFlightFollowing, &ctx, wx);
        let code = st.assigned_squawk.expect("code issued");
        let spoken = atis::spoken_digits(&code.to_string());
        assert!(r.contains(&spoken), "{r}");
        assert!(
            !r.contains("radar contact"),
            "contact is the watch's call, not theatre: {r}"
        );
        // ...and every later squawk check repeats THAT code, tower included.
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            false,
            false,
            None,
            false,
            None,
        );
        let r = st.twr.handle(tower::Intent::SquawkCheck, &ctx);
        assert!(r.contains(&spoken), "{r}");
        assert!(!r.contains("seven zero zero zero"), "{r}");
        *SQUAWK_WATCH.lock().unwrap() = None;
    }

    #[test]
    fn compliance_watch_holds_the_pilot_to_clearances() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Gibraltar-ish: field reference + the real-ish 09/27 geometry.
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_PANEL.lock().unwrap() = Some((
            "Gibraltar".into(),
            11.0,
            vec![("tower".to_string(), 131200_i64)],
        ));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        // No arrival taxi-in in play — this test's occupancy berate depends on it
        // (a prior test could leave the shared flag set).
        *TAXI_TO_STAND.lock().unwrap() = false;
        *TAXI_ROUTE.lock().unwrap() = None;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;

        // ZONE: airborne 3 nm out at 1,500 ft, never called anyone, not
        // tuned: the CREW flags it, with the tower frequency.
        let call = compliance_tick(36.20, -5.35, 1500.0, false, 90.0, false, t0).expect("zone");
        assert!(call.crew);
        assert!(
            call.line.contains("zone without clearance"),
            "{}",
            call.line
        );
        assert!(
            call.line.contains("decimal two"),
            "no tower freq: {}",
            call.line
        );
        // Cooldown holds.
        assert!(compliance_tick(36.20, -5.35, 1500.0, false, 90.0, false, t0).is_none());
        // Tuned after the cooldown: the CONTROLLER challenges.
        let call = compliance_tick(36.20, -5.35, 1500.0, false, 90.0, true, t0 + sec(130))
            .expect("challenge");
        assert!(!call.crew);
        assert!(call.line.contains("Say callsign"), "{}", call.line);
        // Above the zone cap: overflying is not an incursion.
        assert!(compliance_tick(36.20, -5.35, 4500.0, false, 90.0, true, t0 + sec(260)).is_none());

        // Contact made (any answered call publishes the snapshot): the zone
        // watch stands down and the clearance watch takes over.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Parked,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        });
        // On the ground at the parking first (settles the air/ground edge —
        // and a stand is nobody's runway), then taxi onto the runway
        // uncleared: ordered off. It arrives MOVING: a stopped first sight
        // on a strip is a runway start, which is no incursion.
        let t2 = t0 + sec(400);
        assert!(compliance_tick(36.16, -5.35, 15.0, true, 0.0, true, t2 - sec(10)).is_none());
        let call = compliance_tick(36.15155, -5.342, 20.0, true, 5.0, true, t2).expect("occupancy");
        assert!(!call.crew);
        // Past midfield the position names the 27 end (same rule as the
        // occupancy geometry test).
        assert!(
            call.line.contains("on runway two seven without clearance"),
            "{}",
            call.line
        );
        assert!(call.line.contains("Exit the runway"), "{}", call.line);

        // Airborne next tick with only a taxi clearance: uncleared takeoff,
        // called once.
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::TaxiCleared;
        let t3 = t2 + sec(70);
        let call = compliance_tick(36.152, -5.34, 300.0, false, 70.0, true, t3).expect("takeoff");
        assert!(
            call.line.contains("not cleared for takeoff"),
            "{}",
            call.line
        );

        // Back onto the runway with no landing clearance: the phone number.
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::Inbound;
        let t4 = t3 + sec(70);
        let call = compliance_tick(36.15155, -5.342, 12.0, true, 25.0, true, t4).expect("landing");
        assert!(
            call.line.contains("number for you to copy"),
            "{}",
            call.line
        );
        // The landing berate is once, but still squatting (stopped) on the
        // runway a minute later earns the occupancy order (no clearance to be
        // there). Rolling off it is the vacate (#155).
        assert!(compliance_tick(36.15155, -5.342, 12.0, true, 10.0, true, t4 + sec(70)).is_none());
        let call = compliance_tick(36.15155, -5.342, 12.0, true, 0.0, true, t4 + sec(71))
            .expect("occupancy after uncleared landing");
        assert!(call.line.contains("Exit the runway"), "{}", call.line);

        // A LEGAL landing: ClearedToLand grants the vacate grace — sitting on
        // the runway right after touchdown is not an occupancy violation.
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::ClearedToLand;
        assert!(compliance_tick(36.15155, -5.342, 12.0, true, 15.0, true, t4 + sec(200)).is_none());

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_PANEL.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    /// Issue #63, 2026-09-26: an autoland with no landing clearance touched
    /// down at t 1790448341.9 and the tower's phone number came at
    /// 1790448368.5, 0.2 s after the autoland handed over at a standstill:
    /// the monitor skipped the watch while `autoland::ACTIVE`, so the
    /// air/ground edge it needs was the last one before the arm. Ticked
    /// through the approach, the call lands ON the touchdown tick, the same
    /// as hand-flown; with a landing clearance it stays silent through the
    /// touchdown and the rollout. The skipped-tick replay pins the old
    /// delay so the regression is visible.
    #[test]
    fn an_uncleared_landing_is_called_at_touchdown_whoever_flies() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_PANEL.lock().unwrap() = Some((
            "Gibraltar".into(),
            11.0,
            vec![("tower".to_string(), 131200_i64)],
        ));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        *TAXI_TO_STAND.lock().unwrap() = false;
        *TAXI_ROUTE.lock().unwrap() = None;
        let t0 = Instant::now();
        let at = |s: f64| t0 + std::time::Duration::from_secs_f64(s);
        // The recorded shape at 0.4 s ticks: airborne on the final, the
        // touchdown at 60 s at 146 kt, stopped 26 s later, and the autoland
        // handoff 26.4 s after the touchdown (1790448341.9 to 1790448368.3).
        let touchdown = 60.0;
        let handoff = touchdown + 26.4;
        let tick = |s: f64| {
            let on_ground = s >= touchdown;
            let gs = if on_ground {
                (146.0 - (s - touchdown) * 5.6).max(0.0)
            } else {
                146.0
            };
            let (lat, lon, alt) = if on_ground {
                (36.15145, -5.3450, 12.0)
            } else {
                (
                    36.1510,
                    -5.3554 - (touchdown - s) * 0.0012,
                    12.0 + (touchdown - s) * 8.0,
                )
            };
            compliance_tick(lat, lon, alt, on_ground, gs, true, at(s))
        };
        let reset = |state: tower::State| {
            *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
            *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
                state,
                callsign: "November eight one six November November".into(),
                runway_ident: "09".into(),
                pending_readback: None,
                holds_runway: false,
            });
        };
        let steps: Vec<f64> = (0..=250).map(|i| i as f64 * 0.4).collect();

        // Ticked every tick (hand-flown, and now under an autoland): the
        // phone number on the touchdown tick, once.
        reset(tower::State::Inbound);
        let calls: Vec<(f64, String)> = steps
            .iter()
            .filter_map(|&s| tick(s).map(|c| (s, c.line)))
            .collect();
        assert!(!calls.is_empty(), "no call at all");
        assert!(
            (calls[0].0 - touchdown).abs() < 0.41,
            "called at {} against touchdown {touchdown}",
            calls[0].0
        );
        assert!(calls[0].1.contains("without clearance"), "{}", calls[0].1);
        assert!(
            calls[0].1.contains("number for you to copy"),
            "{}",
            calls[0].1
        );
        assert_eq!(
            calls.iter().filter(|(_, l)| l.contains("number")).count(),
            1,
            "{calls:?}"
        );

        // The old gate: skipped from the arm (20 s before the touchdown) to
        // the handoff. The call came on the first tick after it.
        reset(tower::State::Inbound);
        let late = steps
            .iter()
            .filter(|&&s| !(touchdown - 20.0..handoff).contains(&s))
            .find_map(|&s| tick(s).map(|_| s))
            .expect("the skipped watch still called it, late");
        assert!(late - touchdown > 26.0, "late by {}", late - touchdown);

        // Cleared to land: silent through the touchdown and the rollout.
        reset(tower::State::ClearedToLand);
        for &s in &steps {
            assert!(tick(s).is_none(), "cleared landing berated at {s}");
        }

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_PANEL.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    #[test]
    fn arrival_taxi_in_is_not_a_runway_incursion() {
        // The live fault (2026-07-27): after landing and getting a taxi-in
        // clearance to the stand, the tower shouted "on runway without clearance"
        // a couple of minutes later. The taxi-in sets the FSM to `Parked` (not
        // `TaxiCleared`) and ends the vacate grace early, so a crew still clearing
        // the landing runway on its own cleared route got berated. A live arrival
        // taxi-in authorises vacating the runway.
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *TAXI_ROUTE.lock().unwrap() = None;
        // Down, cleared to taxi to the stand: FSM Parked, still physically on 27.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Parked,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "27".into(),
            pending_readback: None,
            holds_runway: false,
        });
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;

        // WITH the arrival taxi-in flag AND its published route: vacating the
        // landing runway is legal, well past the 120 s vacate grace — no
        // berate. The route lives on the taxiway (off the strip), the way a
        // stand route does once the vacate is under way.
        *TAXI_TO_STAND.lock().unwrap() = true;
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![(36.1470, -5.3420), (36.1460, -5.3420)],
            via: "Alpha".into(),
            seg: vec![("A".into(), false)],
            holding_point: "Stand 1".into(),
            lead_on: vec![],
        });
        assert!(
            compliance_tick(36.15155, -5.342, 20.0, true, 0.0, true, t0 + sec(200)).is_none(),
            "a cleared arrival taxi-in must not be berated for vacating the runway"
        );

        // The bare flag WITHOUT a route is not a clearance: a stale flag that
        // survived a station swap (or a route-less reply) must not switch the
        // incursion watch off. Same position, route gone: the berate stands.
        // (Rolling, as a landed aircraft is: a stopped first sight on a strip
        // is a runway start.)
        *TAXI_ROUTE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        let call = compliance_tick(36.15155, -5.342, 20.0, true, 5.0, true, t0 + sec(300))
            .expect("occupancy berate with the flag but no published route");
        assert!(call.line.contains("without clearance"), "{}", call.line);

        // Control: drop the flag (a bare Parked state) and the SAME position IS
        // an incursion — proving the exemption, not the geometry, silenced it.
        *TAXI_TO_STAND.lock().unwrap() = false;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        let call = compliance_tick(36.15155, -5.342, 20.0, true, 5.0, true, t0 + sec(400))
            .expect("occupancy berate without the taxi-in flag");
        assert!(call.line.contains("without clearance"), "{}", call.line);

        // Clean up the shared world for the next test.
        *TAXI_TO_STAND.lock().unwrap() = false;
        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    #[test]
    fn a_station_swap_retires_the_previous_fields_worksheets() {
        // A region handoff calls install() WITHOUT uninstall() (main loop,
        // take_pending_station). Anything install() forgets to clear keeps
        // acting for the field that no longer exists: an armed vector plan
        // kept steering toward the old threshold, the takeoff handoff line
        // named the old field, and a stale arrival taxi-in flag exempted
        // runway occupancy from the incursion watch at the new one.
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let st_a = lxgb();
        st_a.install();
        // Field A's leftovers.
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: "November one seven two Sierra Papa".into(),
            thr_lat: 36.1508,
            thr_lon: -5.3596,
            course_deg: 88.0,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some("one three one decimal two".into()),
            target_alt_ft: 1500.0,
            ils: false,
            join_nm: 6.0,
            cleared: false,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });
        *HANDOFF.lock().unwrap() = Some(Handoff {
            line: "November one seven two Sierra Papa, contact Gibraltar Departure on one three \
                   zero decimal four."
                .into(),
            min_alt_ft: 711.0,
            to: "Gibraltar Departure".into(),
            khz: 130_400,
        });
        *LANDING_HANDOFF.lock().unwrap() = Some("zero niner".into());
        *TAXI_TO_STAND.lock().unwrap() = true;
        // The swap: field B installs over field A.
        let st_b = lxgb();
        st_b.install();
        assert!(
            VECTORS.lock().unwrap().is_none(),
            "field A's vector plan must die with its station"
        );
        assert!(
            HANDOFF.lock().unwrap().is_none(),
            "field A's departure handoff must die with its station"
        );
        assert!(
            LANDING_HANDOFF.lock().unwrap().is_none(),
            "field A's approach handoff must die with its station"
        );
        assert!(
            !*TAXI_TO_STAND.lock().unwrap(),
            "field A's arrival taxi-in flag must die with its station"
        );
        Station::uninstall();
        // And uninstall retires the same set plus the published squawk, as
        // the party-line doc promises.
        *ASSIGNED_SQUAWK.lock().unwrap() = Some(4512);
        *TAXI_TO_STAND.lock().unwrap() = true;
        Station::uninstall();
        assert!(ASSIGNED_SQUAWK.lock().unwrap().is_none());
        assert!(!*TAXI_TO_STAND.lock().unwrap());
    }

    #[test]
    fn a_cleared_route_crossing_is_not_an_incursion() {
        // The live fault (2026-07-14): the tower's own taxi route crossed a
        // runway, then the compliance watch berated the pilot for being "on
        // the runway without clearance" mid-crossing. A runway-flagged leg of
        // the cleared route makes the occupancy legal: silence while moving,
        // "expedite" when stopped — and the berate still fires when the route
        // has no business on that runway.
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        // The cleared route: south to north straight across the runway at
        // lon -5.342, the crossing leg flagged as runway pavement.
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![(36.1490, -5.3420), (36.1530, -5.3420), (36.1540, -5.3420)],
            via: "Alpha".into(),
            seg: vec![("09/27".into(), true), ("A".into(), false)],
            holding_point: "A1".into(),
            lead_on: vec![],
        });
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // Settle the air/ground edge off the runway first.
        assert!(compliance_tick(36.1490, -5.3420, 15.0, true, 10.0, true, t0).is_none());
        // Mid-crossing at taxi speed, on the cleared runway leg: silence.
        assert!(
            compliance_tick(36.15155, -5.342, 15.0, true, 10.0, true, t0 + sec(2)).is_none(),
            "a cleared crossing must not be berated"
        );
        // Stopped dead mid-crossing: "expedite", never "without clearance".
        let call = compliance_tick(36.15155, -5.342, 15.0, true, 0.0, true, t0 + sec(4))
            .expect("a stopped crossing earns a call");
        assert!(
            call.line.contains("expedite crossing runway"),
            "{}",
            call.line
        );
        assert!(!call.line.contains("without clearance"), "{}", call.line);
        // Same spot, but the route STOPS at the hold-short point (it only
        // ends NEAR the runway, never crosses): nosing onto the pavement is
        // still an incursion — the berate stands.
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        TAXI_ROUTE.lock().unwrap().as_mut().unwrap().path =
            vec![(36.1490, -5.3420), (36.1505, -5.3420)];
        assert!(compliance_tick(36.1490, -5.3420, 15.0, true, 10.0, true, t0 + sec(70)).is_none());
        let call = compliance_tick(36.15155, -5.342, 15.0, true, 10.0, true, t0 + sec(72))
            .expect("an uncleared runway entry is still an incursion");
        assert!(call.line.contains("without clearance"), "{}", call.line);

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *TAXI_ROUTE.lock().unwrap() = None;
    }

    #[test]
    fn unnamed_runway_edges_still_name_the_crossing() {
        // apt.dat authors often leave runway network edges unnamed: the
        // planner's crossings list (named edges only) came out empty and the
        // clearance never said "cross runway ..." (the KPDK taxi, 2026-07-14).
        // Geometry backfills the designator from the physical strip.
        let runways = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        let mut plan = nav::TaxiRoutePlan {
            via: vec!["A".into()],
            crossings: Vec::new(),
            holding_point: "A1".into(),
            path: vec![(36.1490, -5.3420), (36.1530, -5.3420), (36.1540, -5.3420)],
            seg: vec![(String::new(), true), ("A".into(), false)],
            lead_on: vec![],
        };
        mark_runway_crossings(&mut plan, &runways);
        assert_eq!(plan.crossings, vec!["09/27".to_string()]);
        // The leg is renamed and flagged too — the guide's crossing cue and
        // the incursion watch's cleared-crossing exemption key on the flag.
        assert_eq!(plan.seg[0], ("09/27".to_string(), true));
        // Idempotent: a second pass (or an already-named edge) doesn't dupe.
        mark_runway_crossings(&mut plan, &runways);
        assert_eq!(plan.crossings.len(), 1);
        // A crossing link authored as a plain TAXIWAY edge across the strip
        // (EGLL's N7, 2026-07-20: no crossing clause, no guide call): the
        // geometry names and flags it all the same.
        let mut taxiway_authored = nav::TaxiRoutePlan {
            via: vec!["N7".into()],
            crossings: Vec::new(),
            holding_point: "A1".into(),
            path: vec![(36.1490, -5.3420), (36.1530, -5.3420), (36.1540, -5.3420)],
            seg: vec![("N7".into(), false), ("A".into(), false)],
            lead_on: vec![],
        };
        mark_runway_crossings(&mut taxiway_authored, &runways);
        assert_eq!(taxiway_authored.crossings, vec!["09/27".to_string()]);
        assert_eq!(taxiway_authored.seg[0], ("09/27".to_string(), true));
        // The FINAL leg ends at the hold-short point NEAR the destination
        // strip — that proximity is never a crossing.
        let mut final_leg = nav::TaxiRoutePlan {
            via: vec!["A".into()],
            crossings: Vec::new(),
            holding_point: "A1".into(),
            path: vec![(36.1490, -5.3420), (36.1512, -5.3420)],
            seg: vec![("A".into(), false)],
            lead_on: vec![],
        };
        mark_runway_crossings(&mut final_leg, &runways);
        assert!(final_leg.crossings.is_empty(), "{:?}", final_leg.crossings);
        // Same approach to the same hold, resampled onto the paint: the legs
        // are metres long now, so the exemption has to be the route's TAIL
        // and not merely its last leg. Exempting one 5 m sliver would have
        // the route announce a crossing of the runway it is holding short of.
        // 170 m of approach to a hold 25 m short of the strip (an authored
        // zone can leave it that close), in 5 m samples: several of the last
        // legs sit inside the runway's own half-width, and each one of them
        // would name a crossing if only the very last were exempt.
        let (from, to) = (36.14980, 36.151325);
        let mut resampled = nav::TaxiRoutePlan {
            via: vec!["A".into()],
            crossings: Vec::new(),
            holding_point: "A1".into(),
            path: (0..=34)
                .map(|k| (from + (to - from) * k as f64 / 34.0, -5.3420))
                .collect(),
            seg: vec![("A".to_string(), false); 34],
            lead_on: vec![],
        };
        mark_runway_crossings(&mut resampled, &runways);
        assert!(resampled.crossings.is_empty(), "{:?}", resampled.crossings);
        // A route that never touches the pavement names nothing.
        let mut clear = nav::TaxiRoutePlan {
            via: vec!["A".into()],
            crossings: Vec::new(),
            holding_point: "A1".into(),
            path: vec![(36.1490, -5.3420), (36.1505, -5.3420)],
            seg: vec![(String::new(), false)],
            lead_on: vec![],
        };
        mark_runway_crossings(&mut clear, &runways);
        assert!(clear.crossings.is_empty());
    }

    #[test]
    fn off_route_stray_gets_a_fresh_route_from_present_position() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Testfield".into(), 36.0, -5.0));
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        });
        // A tiny network: taxiway A east along 36.0000, then H1 north to the
        // holding point. The cleared route rides all of it.
        let mut nodes = std::collections::HashMap::new();
        nodes.insert(1, (36.0000, -5.0000));
        nodes.insert(2, (36.0000, -4.9990));
        nodes.insert(3, (36.0000, -4.9980));
        nodes.insert(4, (36.0010, -4.9980));
        let edge = |a: i64, b: i64, name: &str| nav::TaxiEdge {
            a,
            b,
            oneway: false,
            runway: false,
            name: name.into(),
            zone_of: Vec::new(),
        };
        *TAXI_NET.lock().unwrap() = Some(std::sync::Arc::new(nav::TaxiNet {
            nodes,
            edges: vec![edge(1, 2, "A"), edge(2, 3, "A"), edge(3, 4, "H1")],
            stands: vec![],
            paint: vec![],
            pavement: vec![],
            stand_lines: vec![],
        }));
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![
                (36.0000, -5.0000),
                (36.0000, -4.9990),
                (36.0000, -4.9980),
                (36.0010, -4.9980),
            ],
            via: "Alpha, Hotel one".into(),
            seg: vec![
                ("A".into(), false),
                ("A".into(), false),
                ("H1".into(), false),
            ],
            holding_point: "H1".into(),
            lead_on: vec![],
        });
        let t0 = Instant::now();

        // 100 m north of the route at taxi speed: the challenge carries a
        // FRESH route from where the aircraft IS, not the stale original via
        // (the same via line was spoken seven times at Malaga, 2026-07-13).
        let call = compliance_tick(36.0009, -5.0000, 15.0, true, 15.0, true, t0).expect("stray");
        assert!(!call.crew);
        assert!(
            call.line.contains("off the cleared taxi route"),
            "{}",
            call.line
        );
        assert!(
            call.line
                .contains("Taxi to holding point Hotel one via Alpha"),
            "{}",
            call.line
        );
        // The fresh route is republished (the guide rebuilds from it), and
        // it starts AT the aircraft — the walk back to the network is now
        // on-route, so the berate cannot repeat while the pilot complies.
        {
            let g = TAXI_ROUTE.lock().unwrap();
            let r = g.as_ref().expect("route republished");
            assert_eq!(r.path[0], (36.0009, -5.0000));
            assert_eq!(r.holding_point, "H1");
        }
        assert!(
            compliance_tick(
                36.0009,
                -5.0000,
                15.0,
                true,
                15.0,
                true,
                t0 + std::time::Duration::from_secs(50)
            )
            .is_none(),
            "same spot is ON the fresh route"
        );

        *TAXI_NET.lock().unwrap() = None;
        *TAXI_ROUTE.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    #[test]
    fn landing_handoff_clears_on_clearance_and_on_touchdown() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Inbound,
            callsign: "November five five zero Charlie Romeo".into(),
            runway_ident: "27".into(),
            pending_readback: None,
            holds_runway: false,
        });
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;

        // Approach handed us off: the coach is armed, still owed while airborne
        // and not yet cleared to land.
        *LANDING_HANDOFF.lock().unwrap() = Some("two seven".into());
        assert!(compliance_tick(36.30, -5.35, 1500.0, false, 90.0, true, t0).is_none());
        assert_eq!(
            crate::party_line::landing_handoff_runway().as_deref(),
            Some("two seven"),
            "still owe the tower a final report"
        );

        // Tower clears the landing → the coach's job is done.
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::ClearedToLand;
        compliance_tick(36.20, -5.35, 1000.0, false, 90.0, true, t0 + sec(20));
        assert!(
            crate::party_line::landing_handoff_runway().is_none(),
            "landing clearance clears the handoff"
        );

        // Re-armed but this time we touch down without it: on-ground clears it
        // so the next takeoff isn't nagged by a stale handoff.
        *LANDING_HANDOFF.lock().unwrap() = Some("two seven".into());
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        compliance_tick(36.1512, -5.3554, 12.0, true, 5.0, true, t0 + sec(40));
        assert!(
            crate::party_line::landing_handoff_runway().is_none(),
            "touchdown clears the handoff"
        );

        *ACTIVE_FIELD.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *LANDING_HANDOFF.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    /// LEMG 12 (2026-09-26, a 737): landed 12, stopped in the far half of
    /// the strip heading 117, and was told "you landed on runway three zero"
    /// because the name was the NEARER end. The end is the one the aircraft
    /// points down. Shown on the Jerez strip (02 runs 021 true).
    #[test]
    fn a_runway_is_named_by_the_end_the_aircraft_points_down() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Jerez".into(), 36.7446, -6.0601));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 45.0,
            surface: 0,
            ident_a: "02".into(),
            ident_b: "20".into(),
            lat_a: 36.7349761,
            lon_a: -6.0648168,
            lat_b: 36.7542941,
            lon_b: -6.0553835,
        }];
        // Three quarters of the way up 02, rolled out heading 021.
        let (lat, lon) = (36.7494646, -6.0577402);
        assert_eq!(on_active_runway(lat, lon).as_deref(), Some("20"), "the nearer end");
        assert_eq!(on_active_runway_facing(lat, lon, Some(21.0)).as_deref(), Some("02"));
        assert_eq!(on_active_runway_facing(lat, lon, Some(201.0)).as_deref(), Some("20"));
        assert_eq!(on_active_runway_facing(lat, lon, None).as_deref(), Some("20"));
        // And the uncleared-landing line says it.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Inbound,
            callsign: "November one two three".into(),
            runway_ident: "02".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        let t0 = Instant::now();
        assert!(compliance_tick_facing(lat, lon, 400.0, false, 140.0, true, t0, Some(21.0)).is_none());
        let call = compliance_tick_facing(
            lat, lon, 12.0, true, 20.0, true, t0 + std::time::Duration::from_secs(60), Some(21.0),
        )
        .expect("an uncleared landing");
        assert!(call.line.contains("runway zero two"), "{}", call.line);

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    /// Issue #108, the recorded case itself: LEMG 12/30 from apt.dat, the
    /// 737 (N816NN, 2026-09-26) stopped at 36.682790, -4.488170 heading
    /// 117.9 true, 227 m from the 30 threshold, and at t 1790448368.5 was
    /// told "you landed on runway three zero". That build predated 3b6ff11
    /// (committed 20:58:36 local, the call 20:46:08), which named the
    /// NEARER end; the monitor passes `psi` (true), the frame
    /// `heading_a_deg` is in. With the fix the same numbers say "one two".
    #[test]
    fn the_lemg_landing_is_named_one_two_from_its_recorded_numbers() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Malaga".into(), 36.6749, -4.4991));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![
            nav::Runway {
                width_m: 45.11,
                surface: 1,
                ident_a: "12".into(),
                ident_b: "30".into(),
                lat_a: 36.6910753,
                lon_a: -4.5078270,
                lat_b: 36.6795648,
                lon_b: -4.4805125,
            },
            nav::Runway {
                width_m: 45.11,
                surface: 1,
                ident_a: "13".into(),
                ident_b: "31".into(),
                lat_a: 36.6845573,
                lon_a: -4.5126253,
                lat_b: 36.6654027,
                lon_b: -4.4858140,
            },
        ];
        let (lat, lon, hdg) = (36.682790230055794, -4.488169765732041, 117.9);
        assert_eq!(on_active_runway(lat, lon).as_deref(), Some("30"), "the old name");
        assert_eq!(on_active_runway_facing(lat, lon, Some(hdg)).as_deref(), Some("12"));
        // The tower was at Parked (no landing clearance): a flight, then the
        // wheels, heading down 12.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Parked,
            callsign: "November eight one six November November".into(),
            runway_ident: "12".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *TAXI_TO_STAND.lock().unwrap() = false;
        *TAXI_ROUTE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        assert!(compliance_tick_facing(36.70, -4.53, 900.0, false, 146.0, true, t0, Some(hdg)).is_none());
        let call = compliance_tick_facing(lat, lon, 38.2, true, 0.0, true, t0 + sec(60), Some(hdg))
            .expect("an uncleared landing");
        assert!(call.line.contains("landed on runway one two"), "{}", call.line);
        assert!(!call.line.contains("three zero"), "{}", call.line);

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    /// EGKK's two strips from apt.dat: 08R/26L and the close parallel
    /// 08L/26R.
    fn egkk_runways() -> Vec<nav::Runway> {
        vec![
            nav::Runway {
                width_m: 46.0,
                surface: 1,
                ident_a: "08R".into(),
                ident_b: "26L".into(),
                lat_a: 51.1450993,
                lon_a: -0.2123748,
                lat_b: 51.1514994,
                lon_b: -0.1659296,
            },
            nav::Runway {
                width_m: 46.0,
                surface: 1,
                ident_a: "08L".into(),
                ident_b: "26R".into(),
                lat_a: 51.1468901,
                lon_a: -0.2126029,
                lat_b: 51.1518321,
                lon_b: -0.1767408,
            },
        ]
    }

    /// Issue #117, sortie 1790527340-full_auto (EGKK-EGBB, 2026-09-27): the
    /// 737 began lined up on 26L at 51.1476808, -0.1935461 (0.0 kt, heading
    /// 256.3), a runway start. The IFR request made contact (tower Parked)
    /// and the compliance watch said "you are on runway two six left without
    /// clearance. Exit the runway immediately." at +17.3 s. A stopped first
    /// sight on a strip is the sim's placement, a lined-up aircraft awaiting
    /// its clearance: silent at the recorded ticks, and the incursion watch
    /// holds it authorised too. One that TAXIES onto a strip (first seen
    /// moving) is still ordered off, and the start is spent once it leaves.
    #[test]
    fn a_runway_start_is_lined_up_not_an_incursion() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gatwick".into(), 51.1481, -0.1903));
        *ACTIVE_RUNWAYS.lock().unwrap() = egkk_runways();
        *CLEARANCE.lock().unwrap() = None;
        *TAXI_TO_STAND.lock().unwrap() = false;
        *TAXI_ROUTE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        let t0 = Instant::now();
        let at = |s: f64| t0 + std::time::Duration::from_secs_f64(s);
        let (lat, lon, hdg) = (51.14768083914999, -0.19354614043732044, 256.3);
        let tick = |s: f64| compliance_tick_facing(lat, lon, 203.0, true, 0.0, true, at(s), Some(hdg));
        // No contact yet: the recorded ticks from +1.5 s.
        for s in [1.5, 4.0, 6.4, 9.0, 11.6, 14.5] {
            assert!(tick(s).is_none(), "{s}");
        }
        // The IFR request's answer publishes the snapshot at +17.2 s.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Parked,
            callsign: "Whiskey Alpha Tango one seven three".into(),
            runway_ident: "26R".into(),
            pending_readback: None,
            holds_runway: false,
        });
        for s in [17.3, 17.8, 20.2, 33.1, 47.9, 62.7, 90.0, 200.0] {
            assert!(tick(s).is_none(), "the runway start berated at +{s}");
        }
        assert!(runway_occupancy_authorised(lat, lon, 0.0), "the incursion watch");

        // Off the strip, then back on it: not a start any more.
        assert!(compliance_tick_facing(51.1440, -0.1935, 203.0, true, 10.0, true, at(210.0), Some(0.0)).is_none());
        assert!(!runway_occupancy_authorised(lat, lon, 0.0));
        let call = compliance_tick_facing(lat, lon, 203.0, true, 5.0, true, at(220.0), Some(hdg))
            .expect("taxied onto the runway uncleared");
        assert!(call.line.contains("runway two six left without clearance"), "{}", call.line);

        // A fresh watch whose first sight is a taxi onto the strip: ordered off.
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        let call = compliance_tick_facing(lat, lon, 203.0, true, 8.0, true, at(400.0), Some(hdg))
            .expect("moving onto the strip is no runway start");
        assert!(call.line.contains("Exit the runway"), "{}", call.line);

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    /// Issue #124, sortie 1790534902-full_auto (EGBB-EGKK, 2026-09-27):
    /// cleared to land 26L, down at +2974 s, then taxiing down the strip at
    /// 8 to 12 kt to exit ER and "runway vacated" only at +3149. The monitor's
    /// incursion watch said "exit the runway immediately" / "expedite" eight
    /// times from +2994 (26L, then 08R past midfield) because only a takeoff
    /// clearance or line-up counted as the tower putting us there. A landing
    /// holds its strip until the vacated report; the arrival's cleared
    /// crossing of 08L (+3191, 12.7 kt) is legal while it moves.
    #[test]
    fn a_cleared_landing_holds_its_runway_until_vacated() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gatwick".into(), 51.1481, -0.1903));
        *ACTIVE_RUNWAYS.lock().unwrap() = egkk_runways();
        *TAXI_TO_STAND.lock().unwrap() = false;
        *TAXI_ROUTE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        let landed = |holds: bool, ident: &str| {
            *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
                state: tower::State::ClearedToLand,
                callsign: "Whiskey Alpha Tango four two three six".into(),
                runway_ident: ident.into(),
                pending_readback: None,
                holds_runway: holds,
            });
        };
        // The recorded vacate: +3000.5 (26L half), +3050.6 (08R half).
        let vacate = [
            (51.14847323089843, -0.1879083802267358, 8.4),
            (51.14796049673604, -0.1916164076685698, 9.1),
        ];
        landed(true, "26L");
        for &(lat, lon, gs) in &vacate {
            assert!(on_active_runway(lat, lon).is_some(), "on the strip");
            assert!(runway_occupancy_authorised(lat, lon, gs), "{lat},{lon}");
        }
        // Reported vacated (or a landing clearance for the other strip):
        // the strip is not ours, the watch speaks again.
        landed(false, "26L");
        assert!(!runway_occupancy_authorised(vacate[0].0, vacate[0].1, 8.4));
        landed(true, "26R");
        assert!(!runway_occupancy_authorised(vacate[0].0, vacate[0].1, 8.4));
        // Taxi-in cleared (Parked) across 08L on its route: legal moving,
        // not stopped on it.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Parked,
            callsign: "Whiskey Alpha Tango four two three six".into(),
            runway_ident: "26L".into(),
            pending_readback: None,
            holds_runway: false,
        });
        let (clat, clon) = (51.14836698215888, -0.20092418920350943);
        assert!(on_active_runway(clat, clon).is_some(), "on 08L");
        *TAXI_TO_STAND.lock().unwrap() = true;
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![(51.1478, -0.1990), (51.1484, -0.2009), (51.1492, -0.2020)],
            via: "Echo Romeo, Echo".into(),
            seg: vec![("ER".into(), false), ("E".into(), false)],
            holding_point: "Stand 230R".into(),
            lead_on: vec![],
        });
        assert!(runway_occupancy_authorised(clat, clon, 12.7));
        assert!(!runway_occupancy_authorised(clat, clon, 0.0), "stopped on a crossing");
        // A plain Parked aircraft on the strip with no route: an incursion.
        *TAXI_TO_STAND.lock().unwrap() = false;
        *TAXI_ROUTE.lock().unwrap() = None;
        assert!(!runway_occupancy_authorised(vacate[0].0, vacate[0].1, 8.4));

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    /// EGBB 15/33 as apt.dat has it (46 m wide).
    fn egbb_runways() -> Vec<nav::Runway> {
        vec![nav::Runway {
            width_m: 46.02,
            surface: 1,
            ident_a: "15".into(),
            ident_b: "33".into(),
            lat_a: 52.4656497,
            lon_a: -1.7611282,
            lat_b: 52.4428974,
            lon_b: -1.7358498,
        }]
    }

    /// Issue #155, sortie 1790601822-full_auto (EGKK-EGBB, 2026-09-28): an
    /// autoland with no landing clearance touched down on 15 at +3472.2 and
    /// the tower said "you landed on runway one five without clearance. Taxi
    /// to parking". The incursion watch then said "exit the runway" at
    /// +3491.5, +3507.8, +3529.4 and +3544.8 naming runway 33 (the nearer end,
    /// past midfield) and the compliance watch "you are on runway one five
    /// without clearance" at +3532.6, all through the vacate the tower had
    /// just ordered. The strip we landed on is ours until we leave it; off it
    /// and back on, it is an incursion again. The end named is the one we
    /// point down.
    #[test]
    fn an_uncleared_landing_still_vacates_its_own_runway() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Birmingham".into(), 52.4539, -1.7480));
        *ACTIVE_RUNWAYS.lock().unwrap() = egbb_runways();
        *TAXI_TO_STAND.lock().unwrap() = false;
        *TAXI_ROUTE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        // The book at the touchdown: Inbound (held, never released).
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Inbound,
            callsign: "Whiskey Alpha Tango six two six".into(),
            runway_ident: "15".into(),
            pending_readback: None,
            holds_runway: false,
        });
        let t0 = Instant::now();
        let at = |s: f64| t0 + std::time::Duration::from_secs_f64(s - 3400.0);
        // The recorded final, airborne since well before.
        for s in [3400.0, 3440.0, 3470.9] {
            assert!(compliance_tick_facing(52.4596, -1.7544, 330.0, false, 136.0, true, at(s), Some(144.5))
                .is_none());
        }
        // The touchdown tick: the uncleared landing is called, on 15.
        let call = compliance_tick_facing(52.45892702734815, -1.753640601511448, 312.8, true, 133.1, true,
            at(3472.3), Some(145.3))
            .expect("the uncleared landing");
        assert!(call.line.contains("landed on runway one five without clearance"), "{}", call.line);
        // The recorded rollout and vacate down the strip (state records).
        let rollout = [
            (3491.7, 52.45386543479656, -1.748043000849525, 12.1, 145.9),
            (3508.0, 52.45329194494851, -1.7474048138516027, 11.0, 145.4),
            (3520.8, 52.45279749067367, -1.7468509083893378, 8.5, 145.3),
            (3527.8, 52.45259466624598, -1.7466236640010893, 7.0, 145.4),
            (3534.4, 52.452416156695165, -1.74651448352706, 5.5, 177.6),
            (3541.5, 52.45225506252149, -1.746556227323538, 3.8, 194.1),
        ];
        for &(s, lat, lon, gs, hdg) in &rollout {
            assert!(on_active_runway(lat, lon).is_some(), "+{s} on the strip");
            assert!(runway_occupancy_authorised(lat, lon, gs), "+{s}: the incursion watch");
            let c = compliance_tick_facing(lat, lon, 325.0, true, gs, true, at(s), Some(hdg));
            assert!(c.is_none(), "+{s}: {}", c.map(|c| c.line).unwrap_or_default());
        }
        // Stopped on it, the strip is not ours: squatting is an incursion.
        assert!(!runway_occupancy_authorised(rollout[1].1, rollout[1].2, 0.0));
        // Named by the end we point down, never the nearer 33.
        let (_, lat, lon, _, hdg) = rollout[0];
        assert_eq!(on_active_runway(lat, lon).as_deref(), Some("33"));
        assert_eq!(on_active_runway_facing(lat, lon, Some(hdg)).as_deref(), Some("15"));
        // Off the strip (the recorded taxi-in at +3595.4, 1.3 kt), then back
        // on: an incursion like any other.
        let (olat, olon) = (52.45043554899071, -1.7476458779956332);
        assert!(on_active_runway(olat, olon).is_none(), "off the strip");
        assert!(compliance_tick_facing(olat, olon, 325.0, true, 1.3, true, at(3595.4), Some(193.2)).is_none());
        assert!(!runway_occupancy_authorised(rollout[1].1, rollout[1].2, 8.0));

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    /// Issue #155, sortie 1790601822-full_auto (EGKK 08R, 2026-09-28), from
    /// the xvl log: U28680 lifted off at 13:40:31 ("departure stream ...
    /// 08R", "wake clock started ... 26R"), our held release said "traffic
    /// clear ... cleared for takeoff" at 13:40:47 (the clock was on the 08L
    /// pavement's 26R, never the held 08R), and the party line, not ticked
    /// while the release was held, narrated "Easy eight six eight zero,
    /// runway zero eight right, cleared for takeoff" at 13:40:48, 17 s after
    /// that liftoff and 1.6 s after our clearance. Then U28653 lifted off
    /// 13:42:27, seconds after us, and was "cleared for takeoff" too. Now:
    /// the wake clock sits on the departure's own end (main strip of a close
    /// pair), a liftoff the party line did not see live is never narrated,
    /// the runway is ours alone while lined up or cleared for takeoff, and
    /// nobody departs our strip within the wake time of our own liftoff.
    #[test]
    fn one_departure_on_the_runway_at_a_time() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gatwick".into(), 51.1481, -0.1903));
        *ACTIVE_PANEL.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = egkk_runways();
        *GHOSTS.lock().unwrap() = None;
        *WAKE.lock().unwrap() = None;
        *WAKE_PREV.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *TAXI_TO_STAND.lock().unwrap() = false;
        *TAXI_ROUTE.lock().unwrap() = None;
        let t0 = Instant::now();
        let at = |s: f64| t0 + std::time::Duration::from_secs_f64(s);
        // A point along a strip, 08 end to 26 end.
        let along = |r: usize, f: f64| {
            let rw = &egkk_runways()[r];
            (rw.lat_a + f * (rw.lat_b - rw.lat_a), rw.lon_a + f * (rw.lon_b - rw.lon_a))
        };
        let ghost = |cs: &str, (lat, lon): (f64, f64), ground: bool| TcasTarget {
            callsign: cs.into(),
            lat,
            lon,
            ele_ft: if ground { 203.0 } else { 260.0 },
            on_ground: ground,
            vs_fpm: if ground { 0.0 } else { 1500.0 },
        };
        let chatter = |targets: &[TcasTarget], s: f64| {
            chatter_tick(targets, "Gatwick", 51.1481, -0.1903, 203.0, None, at(s))
        };
        let book = |state: tower::State| {
            *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
                state,
                callsign: "Whiskey Alpha Tango six two six".into(),
                runway_ident: "08R".into(),
                pending_readback: None,
                holds_runway: false,
            });
        };

        // The wake clock: U28680 rolling east on the 08L pavement lifts off.
        book(tower::State::TaxiCleared);
        wake_note_targets(&[ghost("U28680", along(1, 0.55), true)], at(0.0));
        wake_note_targets(&[ghost("U28680", along(1, 0.57), false)], at(1.0));
        assert!(wake_remaining("08R", at(17.0)).is_some(), "the held 08R waits out the wake");

        // The party line saw it on the ground, then not again for 17 s: its
        // liftoff is history, never narrated as a clearance.
        assert!(chatter(&[ghost("U28680", along(1, 0.55), true)], 0.0).is_none());
        assert!(
            chatter(&[ghost("U28680", along(1, 0.57), false)], 17.0).is_none(),
            "a liftoff 17 s old narrated"
        );

        // Cleared for takeoff (and lined up): nobody else departs our strip.
        book(tower::State::TakeoffCleared);
        assert!(chatter(&[ghost("U28653", along(0, 0.40), true)], 30.0).is_none());
        assert!(
            chatter(&[ghost("U28653", along(0, 0.42), false)], 31.0).is_none(),
            "cleared for takeoff behind our takeoff clearance"
        );
        book(tower::State::LinedUp);
        assert!(chatter(&[ghost("EZY1", along(0, 0.40), true)], 32.0).is_none());
        assert!(chatter(&[ghost("EZY1", along(0, 0.42), false)], 33.0).is_none());

        // Our own liftoff off 08R, then the book moves on (the check-in):
        // a departure off the same strip inside the wake time is withheld.
        book(tower::State::Parked);
        let (glat, glon) = along(0, 0.5);
        assert!(compliance_tick(glat, glon, 203.0, true, 140.0, true, at(100.0)).is_none());
        let _ = compliance_tick(glat + 0.001, glon + 0.004, 260.0, false, 150.0, true, at(101.0));
        assert!(chatter(&[ghost("U2865", along(0, 0.40), true)], 101.5).is_none());
        assert!(
            chatter(&[ghost("U2865", along(0, 0.42), false)], 102.6).is_none(),
            "cleared for takeoff 1.6 s after our liftoff"
        );
        // Past the wake time, a departure is narrated again.
        assert!(chatter(&[ghost("U2866", along(0, 0.40), true)], 230.0).is_none());
        let ex = chatter(&[ghost("U2866", along(0, 0.42), false)], 231.0).expect("narrated");
        assert!(ex.lines[0].text.contains("cleared for takeoff"), "{}", ex.lines[0].text);

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *GHOSTS.lock().unwrap() = None;
        *WAKE.lock().unwrap() = None;
        *WAKE_PREV.lock().unwrap() = None;
    }

    /// The Jerez sortie, 2026-07-11: holding at E1 (40 m off the 45 m
    /// runway's centerline) earned "exit the runway immediately", and a
    /// one-tick unstick on the takeoff roll earned "you landed without
    /// clearance" mid-departure. Real width + a real flight fix both.
    #[test]
    fn a_holding_point_is_not_the_runway_and_a_skip_is_not_a_landing() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Jerez".into(), 36.7446, -6.0601));
        // The real LEJR 02/20 100-row: 45 m wide.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 45.0,
            surface: 0,
            ident_a: "02".into(),
            ident_b: "20".into(),
            lat_a: 36.7349761,
            lon_a: -6.0648168,
            lat_b: 36.7542941,
            lon_b: -6.0553835,
        }];
        // Geometry first: E1 is beside the pavement, the centerline is on it.
        assert_eq!(
            on_active_runway(36.753995, -6.056011),
            None,
            "holding point captured"
        );
        assert_eq!(
            on_active_runway(36.7523623, -6.0563268).as_deref(),
            Some("20")
        );

        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November five five zero Charlie Romeo".into(),
            runway_ident: "20".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // Holding short at E1 with only a taxi clearance: no occupancy call.
        assert!(
            compliance_tick(36.753995, -6.056011, 40.0, true, 1.0, true, t0).is_none(),
            "holding SHORT is not ON the runway"
        );

        // Cleared for takeoff, rolling; one-sample unstick, wheels back on.
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::TakeoffCleared;
        assert!(
            compliance_tick(36.7504305, -6.0572701, 40.0, true, 60.0, true, t0 + sec(10)).is_none()
        );
        assert!(compliance_tick(36.7513, -6.0568, 40.0, false, 83.0, true, t0 + sec(11)).is_none());
        assert!(
            compliance_tick(36.7523623, -6.0563268, 40.0, true, 86.0, true, t0 + sec(12)).is_none(),
            "a takeoff-roll skip is not a landing"
        );

        // Airborne for real, then back on the strip a minute later with no
        // landing clearance: THAT is the phone number.
        assert!(
            compliance_tick(36.7450, -6.0601, 500.0, false, 100.0, true, t0 + sec(15)).is_none()
        );
        let call = compliance_tick(36.7523623, -6.0563268, 40.0, true, 40.0, true, t0 + sec(75))
            .expect("a real uncleared landing still fires");
        assert!(
            call.line.contains("number for you to copy"),
            "{}",
            call.line
        );

        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    }

    #[test]
    fn ghosts_speak_airline_telephony_or_nato() {
        assert_eq!(spoken_ghost("IBE3421"), "Iberia three four two one");
        assert_eq!(spoken_ghost("RYR12QT"), "Ryanair one two Quebec Tango");
        // LiveTraffic hands over IATA ids ("BA306", live 2026-07-10).
        assert_eq!(spoken_ghost("BA306"), "Speedbird three zero six");
        assert_eq!(spoken_ghost("U2841"), "Easy eight four one");
        // The Heathrow session's NATO fallbacks, now cast properly.
        assert_eq!(spoken_ghost("EW9460"), "Eurowings niner four six zero");
        assert_eq!(spoken_ghost("ME202"), "Cedar Jet two zero two");
        assert_eq!(spoken_ghost("AC869"), "Air Canada eight six niner");
        assert_eq!(spoken_ghost("VS26"), "Virgin two six");
        // A tail number: full NATO, like any callsign — "BAC12" style tails
        // must not become an airline.
        assert!(spoken_ghost("N172SP").starts_with("November one seven two"));
        // Unknown prefix: NATO fallback, never a guessed airline.
        assert!(spoken_ghost("XYZ123").starts_with("X-ray Yankee Zulu"));
    }

    #[test]
    fn party_line_narrates_each_ghost_once() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *GHOSTS.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        let (flat, flon, felev) = (36.1512, -5.3554, 11.0);
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // An arrival descending on final, 2 nm out at 700 ft.
        let approach = |ele: f64, dist_deg: f64| TcasTarget {
            callsign: "IBE3421".into(),
            lat: 36.1512,
            lon: -5.3554 - dist_deg,
            ele_ft: ele,
            on_ground: false,
            vs_fpm: -600.0,
        };
        // First sight seeds the memory — no call without a trend.
        assert!(chatter_tick(
            &[approach(900.0, 0.04)],
            "Gibraltar",
            flat,
            flon,
            felev,
            None,
            t0
        )
        .is_none());
        // Descending on the next pass: the full exchange — the ghost's final
        // call, the clearance, the readback — aimed at 09.
        let ex = chatter_tick(
            &[approach(700.0, 0.03)],
            "Gibraltar",
            flat,
            flon,
            felev,
            None,
            t0 + sec(5),
        )
        .expect("landing exchange");
        assert_eq!(ex.lines.len(), 3);
        assert!(ex.lines[0].ghost);
        assert!(
            ex.lines[0]
                .text
                .contains("Gibraltar Tower, Iberia three four two one, final runway zero niner"),
            "{}",
            ex.lines[0].text
        );
        assert!(!ex.lines[1].ghost);
        assert!(
            ex.lines[1]
                .text
                .contains("runway zero niner, cleared to land"),
            "{}",
            ex.lines[1].text
        );
        assert!(ex.lines[2].ghost);
        assert!(
            ex.lines[2].text.starts_with("Cleared to land"),
            "{}",
            ex.lines[2].text
        );
        assert_eq!(ex.ghost_seed, crate::speaker::station_voice_seed("IBE3421"));
        // Once per ghost — the rest of the approach stays quiet.
        assert!(chatter_tick(
            &[approach(500.0, 0.02)],
            "Gibraltar",
            flat,
            flon,
            felev,
            None,
            t0 + sec(10)
        )
        .is_none());

        // A second arrival aiming at a strip the PLAYER occupies: "continue
        // approach", never a narrated clearance over the player's head —
        // and only once.
        let inbound2 = |ele: f64, dist_deg: f64| TcasTarget {
            callsign: "BA435".into(),
            lat: 36.1512,
            lon: -5.3554 - dist_deg,
            ele_ft: ele,
            on_ground: false,
            vs_fpm: -600.0,
        };
        assert!(chatter_tick(
            &[inbound2(900.0, 0.05)],
            "Gibraltar",
            flat,
            flon,
            felev,
            Some("27"),
            t0 + sec(12)
        )
        .is_none());
        let ex = chatter_tick(
            &[inbound2(700.0, 0.04)],
            "Gibraltar",
            flat,
            flon,
            felev,
            Some("27"),
            t0 + sec(14),
        )
        .expect("continue exchange");
        assert_eq!(ex.lines.len(), 2);
        assert!(
            ex.lines[1].text.contains("continue approach"),
            "{}",
            ex.lines[1].text
        );
        assert!(
            !ex.lines[1].text.contains("cleared to land"),
            "{}",
            ex.lines[1].text
        );
        assert!(chatter_tick(
            &[inbound2(600.0, 0.03)],
            "Gibraltar",
            flat,
            flon,
            felev,
            Some("27"),
            t0 + sec(16)
        )
        .is_none());
        // Player vacates: the clearance can now come.
        let ex = chatter_tick(
            &[inbound2(500.0, 0.02)],
            "Gibraltar",
            flat,
            flon,
            felev,
            None,
            t0 + sec(18),
        )
        .expect("clearance after vacating");
        assert!(
            ex.lines[1].text.contains("cleared to land"),
            "{}",
            ex.lines[1].text
        );

        // A departure: ground last pass, airborne now → cleared for takeoff.
        let ground = TcasTarget {
            callsign: "VLG88AB".into(),
            lat: 36.1514,
            lon: -5.345,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        let rotated = TcasTarget {
            callsign: "VLG88AB".into(),
            lat: 36.1514,
            lon: -5.345,
            ele_ft: 100.0,
            on_ground: false,
            vs_fpm: 0.0,
        };
        assert!(chatter_tick(
            &[ground],
            "Gibraltar",
            flat,
            flon,
            felev,
            None,
            t0 + sec(15)
        )
        .is_none());
        let ex = chatter_tick(
            &[rotated],
            "Gibraltar",
            flat,
            flon,
            felev,
            None,
            t0 + sec(20),
        )
        .expect("takeoff exchange");
        assert_eq!(ex.lines.len(), 2);
        assert!(!ex.lines[0].ghost);
        assert!(ex.lines[0].text.contains("Vueling"), "{}", ex.lines[0].text);
        assert!(
            ex.lines[0].text.contains("cleared for takeoff"),
            "{}",
            ex.lines[0].text
        );
        assert!(ex.lines[1].ghost);
        assert!(
            ex.lines[1].text.starts_with("Cleared for takeoff"),
            "{}",
            ex.lines[1].text
        );

        *GHOSTS.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    /// ETND's two strips as apt.dat has them: 08L/26R, 446 m of grass, and
    /// 08/26, 1,240 m of asphalt beside it.
    fn etnd_runways() -> Vec<nav::Runway> {
        vec![
            nav::Runway {
                ident_a: "08L".into(),
                ident_b: "26R".into(),
                lat_a: 52.5862935,
                lon_a: 8.3393953,
                lat_b: 52.5866670,
                lon_b: 8.3459906,
                width_m: 50.0,
                surface: 3,
            },
            nav::Runway {
                ident_a: "08".into(),
                ident_b: "26".into(),
                lat_a: 52.5850000,
                lon_a: 8.3318780,
                lat_b: 52.5859870,
                lon_b: 8.3501930,
                width_m: 45.11,
                surface: 1,
            },
        ]
    }

    /// "Request runway ..." in the tower matrix (ETND 2026-09-24): an
    /// assignable runway is granted as the taxi to it and becomes the runway
    /// in use (published on the party line for the crew); the grass strip
    /// the field does not hand out, a runway the field lacks and words with
    /// no runway in them are refused and move nothing.
    #[test]
    fn a_runway_request_is_granted_or_refused_by_the_field() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CLEARANCE.lock().unwrap() = None;
        let wx = calm();
        let mut st = lxgb();
        st.apt.name = "Diepholz".into();
        st.apt.runways = etnd_runways();
        let ask = |st: &mut Station, words: &str| {
            let intent = tower::parse_intent(words);
            assert_eq!(intent, tower::Intent::RequestRunway, "{words}");
            st.note_stated_runway(words, intent);
            let mut ctx = scripted_ctx(st, wx);
            ctx.on_ground = true;
            st.apply_runway_request(&mut ctx, None);
            st.route_keyed(words, Service::Tower, &ctx, wx)
        };

        // The grass strip is refused: a tower does not hand it out.
        let r = ask(&mut st, "request runway two six right");
        assert!(r.contains("unable runway two six right, that runway is not available"), "{r}");
        assert_eq!(st.twr.state, tower::State::Parked);
        // A runway the field lacks.
        let r = ask(&mut st, "request runway three three");
        assert!(r.contains("unable runway three three, no such runway here"), "{r}");
        let r = ask(&mut st, "request runway zero niner");
        assert!(r.contains("unable runway zero niner"), "{r}");
        assert_eq!(st.twr.state, tower::State::Parked);

        // The paved runway, asked for by length: granted, and it is the
        // runway in use from here, on the party line too.
        let r = ask(&mut st, "unable two six right, request runway zero eight for length");
        assert!(
            r.contains("runway zero eight approved, taxi to holding point"),
            "{r}"
        );
        assert!(r.contains("runway zero eight"), "{r}");
        assert_eq!(st.twr.state, tower::State::TaxiCleared);
        assert_eq!(st.runway_in_use.as_deref(), Some("08"));
        assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("08"));
        // The pick holds it through the taxi, whatever the calm air says.
        assert_eq!(st.pick_runway(260.0, 3.0), "08");

        *CLEARANCE.lock().unwrap() = None;
        *TAXI_ROUTE.lock().unwrap() = None;
    }

    /// Liege 2026-09-22 (history "Still owed"): the tower learned its runway
    /// was free from our taxi request, not from the "runway vacated" report
    /// before it. Landed and off the pavement, before the report a ghost's
    /// departure off our strip goes un-narrated (nobody else is given a
    /// takeoff on a runway the book still gives us); after it the next
    /// departure is released, the pick is free again, and the bare
    /// "request taxi" is still the taxi IN.
    #[test]
    fn the_vacated_report_frees_the_strip_before_the_taxi_request() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *GHOSTS.lock().unwrap() = None;
        let path = lxgb_apt_file();
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        *ACTIVE_RUNWAYS.lock().unwrap() = st.apt.runways.clone();
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.on_ground = true;
        // Landed on 27, rolled off onto the apron south of the strip.
        let r = st.route_keyed("final runway two seven", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared to land"), "{r}");
        st.twr.gate("cleared to land runway two seven", tower::Intent::Unknown, &ctx);
        assert_eq!(st.twr.state, tower::State::ClearedToLand);
        let (off_lat, off_lon) = (36.1440, -5.3500);
        assert!(on_active_runway(off_lat, off_lon).is_none(), "the apron is off the strip");
        let held = player_blocked_strip(true, off_lat, off_lon);
        assert!(held.is_some(), "landed and not reported off: the book holds the strip");
        assert!(player_blocked_strip(false, off_lat, off_lon).is_none(), "airborne holds nothing");

        let (flat, flon, felev) = (36.1512, -5.3554, 11.0);
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        let ghost = |cs: &str, ele: f64, ground: bool| TcasTarget {
            callsign: cs.into(),
            lat: 36.1514,
            lon: -5.345,
            ele_ft: ele,
            on_ground: ground,
            vs_fpm: 0.0,
        };
        let tick = |t: TcasTarget, held: Option<&str>, at: Instant| {
            chatter_tick(&[t], "Gibraltar", flat, flon, felev, held, at)
        };
        assert!(tick(ghost("VLG88AB", 11.0, true), held.as_deref(), t0).is_none());
        assert!(
            tick(ghost("VLG88AB", 100.0, false), held.as_deref(), t0 + sec(5)).is_none(),
            "no takeoff narrated off a strip the book still gives us"
        );

        // The report: acknowledged, the state stands, the strip is free.
        let r = st.route_keyed("runway vacated", Service::Tower, &ctx, wx);
        assert!(r.contains("roger, thank you"), "{r}");
        assert_eq!(st.twr.state, tower::State::ClearedToLand);
        assert!(st.twr.runway_vacated());
        let snap_holds = CLEARANCE.lock().unwrap().as_ref().map(|c| c.holds_runway);
        assert_eq!(snap_holds, Some(false), "published for the watches");
        let held = player_blocked_strip(true, off_lat, off_lon);
        assert!(held.is_none(), "{held:?}");
        assert!(tick(ghost("RYR12", 11.0, true), held.as_deref(), t0 + sec(10)).is_none());
        let ex = tick(ghost("RYR12", 100.0, false), held.as_deref(), t0 + sec(15))
            .expect("the next departure is released");
        assert!(ex.lines[0].text.contains("cleared for takeoff"), "{}", ex.lines[0].text);
        // The pick no longer freezes on a landing that is over.
        assert_eq!(st.pick_runway(90.0, 20.0), "09", "the wind turns a freed field round");

        // Still the taxi IN, never a departure clearance.
        let r = st.route_keyed("request taxi", Service::Tower, &ctx, wx);
        assert!(!r.contains("holding point"), "{r}");
        assert_eq!(st.twr.state, tower::State::Parked);

        *GHOSTS.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
    }

    /// The order reaches every loop that flies it, whoever reads first. LEMG
    /// 31 (sortie 1790584715-full_auto, 2026-09-28): the order at t
    /// 1790585525.5 was taken by the main loop's FSM sync (the clearance went
    /// ClearedToLand to Inbound at 1527.1) and the circuit flying the final
    /// never saw it; autoland, armed at 1555.6, drained the flag at its arm.
    /// The one flag had two consumers and the main loop's ~150 ms poll beat
    /// autoland's ~400 ms tick every time: no `ordered_go_around` in the logs.
    #[test]
    fn a_go_around_order_reaches_every_loop_that_flies_it() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // No own fix: the judgement falls back to the order (unmeasured time).
        OWN_FIXES.lock().unwrap().clear();
        *STRIP_SEEN.lock().unwrap() = None;
        // LEMG 13/31, the recorded clearance: runway 31, cleared to land.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 45.0,
            surface: 1,
            ident_a: "13".into(),
            ident_b: "31".into(),
            lat_a: 36.6853,
            lon_a: -4.5093,
            lat_b: 36.6653,
            lon_b: -4.4833,
        }];
        *GO_AROUND_LAST.lock().unwrap() = None;
        *PENDING_GO_AROUND.lock().unwrap() = false;
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::ClearedToLand,
            callsign: "November eight one six November November".into(),
            runway_ident: "31".into(),
            pending_readback: None,
            holds_runway: false,
        });
        // A departure on the far end of the same strip.
        let on_the_strip = TcasTarget {
            callsign: "VLG8176".into(),
            lat: 36.6803,
            lon: -4.5028,
            ele_ft: 50.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        // Armed before the order: the circuit's watch and autoland's.
        let mut circuit = crate::party_line::GoAroundOrders::arm();
        let mut autoland = crate::party_line::GoAroundOrders::arm();
        let call = go_around_tick(
            std::slice::from_ref(&on_the_strip),
            736.0,
            false,
            true,
            Instant::now(),
        )
        .expect("the recorded order");
        assert!(call.line.contains("report downwind"), "{}", call.line);
        // The main loop syncs the FSM first, as it did live...
        assert!(crate::party_line::take_pending_go_around());
        // ...and both loops still see the order, once each.
        assert!(circuit.take(), "the circuit flying the final sees it");
        assert!(!circuit.take(), "once");
        assert!(autoland.take(), "a second reader is not robbed");
        // A loop armed after the order does not fly an old one.
        let mut later = crate::party_line::GoAroundOrders::arm();
        assert!(!later.take());
        *GO_AROUND_LAST.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
    }

    #[test]
    fn go_around_ordered_when_a_ghost_takes_the_runway() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // No own fix: the judgement falls back to the order (unmeasured time).
        OWN_FIXES.lock().unwrap().clear();
        *STRIP_SEEN.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        *GO_AROUND_LAST.lock().unwrap() = None;
        *PENDING_GO_AROUND.lock().unwrap() = false;
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::ClearedToLand,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        });
        let ghost_on_runway = TcasTarget {
            callsign: "IBE3421".into(),
            lat: 36.15155,
            lon: -5.342,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        let t0 = Instant::now();
        // Cleared to land, short final, runway occupied: the order, by name.
        let call = go_around_tick(
            std::slice::from_ref(&ghost_on_runway),
            400.0,
            false,
            true,
            t0,
        )
        .expect("go-around order");
        assert!(!call.crew);
        assert!(
            call.line.contains("go around, traffic on the runway"),
            "{}",
            call.line
        );
        // The order hands the FSM sync to the main loop, like the hold release.
        assert!(
            *PENDING_GO_AROUND.lock().unwrap(),
            "order must arm the FSM sync"
        );
        *PENDING_GO_AROUND.lock().unwrap() = false;
        // Cooldown: not machine-gunned.
        assert!(go_around_tick(
            std::slice::from_ref(&ghost_on_runway),
            350.0,
            false,
            true,
            t0
        )
        .is_none());
        assert!(
            !*PENDING_GO_AROUND.lock().unwrap(),
            "a swallowed repeat must not re-arm"
        );
        // Not tuned: the CREW calls it — and the tower's cancel is just as real.
        *GO_AROUND_LAST.lock().unwrap() = None;
        let call = go_around_tick(
            std::slice::from_ref(&ghost_on_runway),
            400.0,
            false,
            false,
            t0,
        )
        .expect("crew go-around");
        assert!(call.crew);
        assert!(
            *PENDING_GO_AROUND.lock().unwrap(),
            "crew-voiced order still voids the clearance"
        );
        *PENDING_GO_AROUND.lock().unwrap() = false;
        // No landing clearance in effect: not the tower's call to cancel.
        *GO_AROUND_LAST.lock().unwrap() = None;
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::Inbound;
        assert!(go_around_tick(
            std::slice::from_ref(&ghost_on_runway),
            400.0,
            false,
            true,
            t0
        )
        .is_none());
        // Too high: the conflict is not yet real.
        CLEARANCE.lock().unwrap().as_mut().unwrap().state = tower::State::ClearedToLand;
        assert!(go_around_tick(
            std::slice::from_ref(&ghost_on_runway),
            2000.0,
            false,
            true,
            t0
        )
        .is_none());
        // A ghost on the PARALLEL strip is not our conflict: no cancel.
        ACTIVE_RUNWAYS.lock().unwrap().push(nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09L".into(),
            ident_b: "27R".into(),
            lat_a: 36.1692,
            lon_a: -5.3554,
            lat_b: 36.1697,
            lon_b: -5.3366,
        });
        let ghost_on_parallel = TcasTarget {
            callsign: "IBE9".into(),
            lat: 36.16955,
            lon: -5.342,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        *GO_AROUND_LAST.lock().unwrap() = None;
        assert!(
            go_around_tick(
                std::slice::from_ref(&ghost_on_parallel),
                400.0,
                false,
                true,
                t0
            )
            .is_none(),
            "parallel-runway squatter must not cancel our clearance"
        );

        *CLEARANCE.lock().unwrap() = None;
        *GO_AROUND_LAST.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    /// A point on LEMG runway 13's frame: `along` metres past the 13
    /// threshold (negative = on the final), `cross` to the left.
    fn lemg13_point(along: f64, cross: f64) -> (f64, f64) {
        let r = &lemg_runways()[1];
        let f = LandingFrame::new(r, "13").unwrap();
        let x = along * f.u.0 - cross * f.u.1;
        let y = along * f.u.1 + cross * f.u.0;
        (f.lat0 + y / f.m_lat, f.lon0 + x / f.m_lon)
    }

    fn ghost_at(cs: &str, (lat, lon): (f64, f64)) -> TcasTarget {
        TcasTarget {
            callsign: cs.into(),
            lat,
            lon,
            ele_ft: 50.0,
            on_ground: true,
            vs_fpm: 0.0,
        }
    }

    /// How a runway occupant leaves, from its track: the question the
    /// go-around order and the landing release both ask (LEMG 13).
    #[test]
    fn a_runway_occupant_is_judged_by_how_it_moves() {
        let r = &lemg_runways()[1];
        let at = |along: f64, cross: f64, t: f64| {
            let (lat, lon) = lemg13_point(along, cross);
            (lat, lon, t)
        };
        // Standing at the threshold we land on: a departure lined up, off
        // one roll after it starts.
        let lined = strip_exit(r, "13", &[at(60.0, 0.0, -2.0), at(60.0, 0.0, 0.0)]);
        assert_eq!(lined, StripExit::LinedUp(37.5));
        // First sight there: the same.
        assert_eq!(
            strip_exit(r, "13", &[at(60.0, 0.0, 0.0)]),
            StripExit::LinedUp(37.5)
        );
        // Standing mid-runway: nobody can say when it goes.
        assert_eq!(
            strip_exit(r, "13", &[at(1600.0, 0.0, -2.0), at(1600.0, 0.0, 0.0)]),
            StripExit::Stays
        );
        // Rolling our way and accelerating: a departure, off at its liftoff.
        let roll = strip_exit(
            r,
            "13",
            &[
                at(100.0, 0.0, -2.0),
                at(105.0, 0.0, -1.0),
                at(112.0, 0.0, 0.0),
            ],
        );
        match roll {
            StripExit::Clearing(s) => assert!((30.0..36.0).contains(&s), "{s}"),
            other => panic!("{other:?}"),
        }
        // Fast and slowing: the lander ahead rolling out, off at a turn-off.
        let rollout = strip_exit(
            r,
            "13",
            &[
                at(900.0, 0.0, -2.0),
                at(935.0, 0.0, -1.0),
                at(965.0, 0.0, 0.0),
            ],
        );
        assert_eq!(rollout, StripExit::Clearing(ROLLOUT_CLEAR_S));
        // Backtracking toward us: oncoming, no telling when it is off.
        assert_eq!(
            strip_exit(r, "13", &[at(500.0, 0.0, -2.0), at(490.0, 0.0, 0.0)]),
            StripExit::Oncoming
        );
        // Vacating to the side at 5 m/s from 10 m left of the centreline:
        // off at the left edge (22.6 + 5 margin - 10) in about 3.5 s.
        match strip_exit(r, "13", &[at(2000.0, 0.0, -2.0), at(2000.0, 10.0, 0.0)]) {
            StripExit::Clearing(s) => assert!((3.0..4.0).contains(&s), "{s}"),
            other => panic!("{other:?}"),
        }
        // The judgement: nothing is due outside a minute of the threshold.
        assert!(!go_around_due(Some(81.0), &[StripExit::Stays]));
        assert!(go_around_due(Some(55.0), &[StripExit::Stays]));
        // A lined-up departure is given its roll until the roll no longer fits.
        assert!(!go_around_due(Some(50.0), &[StripExit::LinedUp(37.5)]));
        assert!(go_around_due(Some(45.0), &[StripExit::LinedUp(37.5)]));
        // Unmeasured: the order, as before.
        assert!(go_around_due(None, &[StripExit::LinedUp(37.5)]));
        assert!(!go_around_due(Some(10.0), &[]));
        // The release: clear, or everyone moving off in time; never a
        // standing aircraft.
        assert!(landing_release_due(Some(80.0), &[]));
        assert!(landing_release_due(None, &[]));
        assert!(landing_release_due(
            Some(80.0),
            &[StripExit::Clearing(34.0)]
        ));
        assert!(!landing_release_due(
            Some(40.0),
            &[StripExit::Clearing(34.0)]
        ));
        assert!(!landing_release_due(
            Some(80.0),
            &[StripExit::LinedUp(37.5)]
        ));
        assert!(!landing_release_due(None, &[StripExit::Clearing(5.0)]));
    }

    /// Issue #149, the recorded sequence: LEMG 13, sortie
    /// 1790594471-full_auto (2026-09-28). The circuit reported final at
    /// 3.8 nm and 722 ft (t 1790595127.0); the tower cleared it to land and,
    /// 0.8 s later (t 1790595128.1), ordered the go-around for traffic on the
    /// runway: the departure TO4611, which lifted off 13 at t 1790595209.2,
    /// 81 s after the order, about as long again as we had to fly. The final
    /// is continued, the order comes only when the traffic will still be
    /// there when we arrive, and the clearance once the strip clears.
    #[test]
    fn lemg13_the_tower_continues_the_final_and_clears_it_when_the_runway_clears() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let reset = || {
            *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
            *CLEARANCE.lock().unwrap() = None;
            *LANDING_HOLD.lock().unwrap() = None;
            *PENDING_LANDING_RELEASE.lock().unwrap() = None;
            *PENDING_GO_AROUND.lock().unwrap() = false;
            *GO_AROUND_LAST.lock().unwrap() = None;
            *STRIP_SEEN.lock().unwrap() = None;
            OWN_FIXES.lock().unwrap().clear();
            *LANDING_HANDOFF.lock().unwrap() = None;
        };
        reset();
        *ACTIVE_RUNWAYS.lock().unwrap() = lemg_runways();
        let mut st = lxgb();
        st.apt.name = "Malaga".into();
        st.apt.elev_ft = 52.0;
        st.apt.runways = lemg_runways();
        st.runway_in_use = Some("13".into());
        st.circuits = true;
        st.twr.state = tower::State::Inbound;
        let wx = Wx {
            wind_from_deg: 130.0,
            wind_kts: 5.0,
        };
        let cs = "November eight one six November November";
        let mut ctx = st.make_ctx(
            wx,
            "QNH one zero one niner".into(),
            cs.into(),
            false,
            false,
            None,
            true, // TO4611 on runway 13
            None,
        );
        assert_eq!(ctx.runway, "one three");
        // The crew's final report, as transmitted.
        let r = st.route_keyed(
            "november eight one six november november, final runway one three",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("continue approach runway one three"), "{r}");
        assert!(!r.contains("cleared to land"), "{r}");
        assert!(crate::party_line::landing_hold_armed());
        assert_eq!(
            crate::party_line::clearance_state(),
            Some(tower::State::Inbound)
        );
        let base = Instant::now();
        let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
        // Our recorded fixes: t 1790595122.3 and 1128.2 (base on the turn
        // to final, 7.3 km from the threshold, 174 kt).
        note_own_fix(36.7447615, -4.5512901, false, at(0.0));
        note_own_fix(36.7408962, -4.5555152, false, at(5.9));
        let own = own_time_to_threshold(&lemg_runways()[1], "13", at(5.9)).unwrap();
        assert!((3.8..4.1).contains(&own.0), "{own:?}");
        assert!((75.0..90.0).contains(&own.1), "{own:?}");
        // TO4611 standing on the runway at the 13 end, two looks.
        let to4611 = ghost_at("TO4611", lemg13_point(60.0, 0.0));
        assert!(
            go_around_tick(std::slice::from_ref(&to4611), 736.0, false, true, at(4.9)).is_none()
        );
        assert!(
            go_around_tick(std::slice::from_ref(&to4611), 722.8, false, true, at(5.9)).is_none(),
            "no order 80 s out: the recorded go-around at 3.8 nm is gone"
        );
        assert!(!*PENDING_GO_AROUND.lock().unwrap());
        // Nor the clearance: a lined-up departure is not a clear runway.
        assert!(landing_hold_tick(std::slice::from_ref(&to4611), false, at(5.9)).is_none());
        assert!(crate::party_line::landing_hold_armed());

        // Case A: it rolls in time. Brake release, then 5, 7 m/s and
        // accelerating while we are 3 nm out at 150 kt.
        let own_at = |d_m: f64, t: f64| {
            let (lat, lon) = lemg13_point(-d_m, 0.0);
            note_own_fix(lat, lon, false, at(t));
        };
        let mut released = None;
        for (along, t) in [(60.0, 18.0), (60.0, 19.0), (60.0, 20.0), (66.0, 21.0)] {
            own_at(3.0 * 1852.0 + (21.0 - t) * 77.0, t);
            let g = ghost_at("TO4611", lemg13_point(along, 0.0));
            assert!(go_around_tick(std::slice::from_ref(&g), 700.0, false, true, at(t)).is_none());
            if let Some(line) = landing_hold_tick(std::slice::from_ref(&g), false, at(t)) {
                released.get_or_insert((t, line));
            }
        }
        let (t, line) = released.expect("cleared once the departure rolls and will be off in time");
        assert_eq!(
            t, 21.0,
            "at the first look that sees it rolling, not while it stood"
        );
        assert_eq!(
            line,
            format!(
                "{cs}, runway one three, wind one three zero degrees, five knots, cleared to land."
            )
        );
        assert!(!crate::party_line::landing_hold_armed());
        assert_eq!(
            crate::party_line::clearance_state(),
            Some(tower::State::ClearedToLand)
        );
        let owed = crate::party_line::readback_owed().expect("the readback is owed");
        assert_eq!(owed, format!("Cleared to land runway one three, {cs}."));
        // The crew reads it back: the tower knows it gave it.
        ctx.runway_occupied = false;
        let r = st.route_keyed(owed.trim_end_matches('.'), Service::Tower, &ctx, wx);
        assert_eq!(r, "", "the readback is accepted");
        assert_eq!(st.twr.state, tower::State::ClearedToLand);
        assert_eq!(crate::party_line::readback_owed(), None);

        // Case B: it never rolls. Cleared, a lined-up aircraft is given its
        // roll until the roll no longer fits before we arrive; then the order.
        *STRIP_SEEN.lock().unwrap() = None;
        OWN_FIXES.lock().unwrap().clear();
        let standing = ghost_at("TO4611", lemg13_point(60.0, 0.0));
        own_at(2.6 * 1852.0 + 77.0, 40.0);
        own_at(2.6 * 1852.0, 41.0);
        assert!(
            go_around_tick(
                std::slice::from_ref(&standing),
                650.0,
                false,
                true,
                at(41.0)
            )
            .is_none(),
            "2.6 nm, 62 s: sequencing, not a go-around"
        );
        own_at(2.0 * 1852.0 + 77.0, 48.0);
        own_at(2.0 * 1852.0, 49.0);
        assert!(
            go_around_tick(
                std::slice::from_ref(&standing),
                600.0,
                false,
                true,
                at(49.0)
            )
            .is_none(),
            "2.0 nm, 48 s: a roll starting now still clears"
        );
        own_at(1.8 * 1852.0 + 77.0, 50.5);
        own_at(1.8 * 1852.0, 51.5);
        let call = go_around_tick(
            std::slice::from_ref(&standing),
            560.0,
            false,
            true,
            at(51.5),
        )
        .expect("1.8 nm, 43 s, still standing: the order");
        assert!(
            call.line.contains("go around, traffic on the runway"),
            "{}",
            call.line
        );
        assert!(*PENDING_GO_AROUND.lock().unwrap());
        reset();
    }

    /// A point on LEMG runway 30's frame (apt.dat geometry): `along` metres
    /// past the 30 threshold (negative = on the final), `cross` to the left.
    fn lemg30_point(along: f64, cross: f64) -> (f64, f64) {
        let r = &lemg_runways()[0];
        let f = LandingFrame::new(r, "30").unwrap();
        let x = along * f.u.0 - cross * f.u.1;
        let y = along * f.u.1 + cross * f.u.0;
        (f.lat0 + y / f.m_lat, f.lon0 + x / f.m_lon)
    }

    /// Every static the issue #162 tests touch, cleared.
    fn reset_runway_watch() {
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CLEARANCE.lock().unwrap() = None;
        *LANDING_HOLD.lock().unwrap() = None;
        *PENDING_LANDING_RELEASE.lock().unwrap() = None;
        *PENDING_GO_AROUND.lock().unwrap() = false;
        *PENDING_RESEQUENCE.lock().unwrap() = None;
        *GO_AROUND_LAST.lock().unwrap() = None;
        *STRIP_SEEN.lock().unwrap() = None;
        *STANDING.lock().unwrap() = None;
        OWN_FIXES.lock().unwrap().clear();
        *LANDING_HANDOFF.lock().unwrap() = None;
        *TOWER_WIND.lock().unwrap() = None;
        *SURFACE_WIND.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = None;
        *DEPARTURE_STREAM.lock().unwrap() = None;
        *ARRIVAL_STREAM.lock().unwrap() = None;
    }

    const SR22: &str = "November five five zero Charlie Romeo";

    /// The SR22 of sortie 1790610218-approach cleared to land LEMG 30 in
    /// the recorded wind (300 at 4), at the order's 0.95 nm (34 s at the
    /// recorded 100 kt) when `t` is given.
    fn lemg30_cleared(runways: Vec<nav::Runway>, wind: (f64, f64)) {
        reset_runway_watch();
        *ACTIVE_RUNWAYS.lock().unwrap() = runways;
        *ACTIVE_FIELD.lock().unwrap() = Some(("Malaga".into(), 36.6749, -4.4991));
        set_surface_wind(36.6749, -4.4991, Some(wind));
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::ClearedToLand,
            callsign: SR22.into(),
            runway_ident: "30".into(),
            pending_readback: None,
            holds_runway: true,
        });
    }

    fn own_on_lemg30_final(nm: f64, at: Instant) {
        let (lat, lon) = lemg30_point(-nm * 1852.0 - 51.4, 0.0);
        note_own_fix(lat, lon, false, at - std::time::Duration::from_secs(1));
        let (lat, lon) = lemg30_point(-nm * 1852.0, 0.0);
        note_own_fix(lat, lon, false, at);
    }

    /// Issue #162, part 1: "on the runway" is the pavement. LEMG 12/30 is
    /// 45.1 m wide and 2,755 m long; the incursion watch's capture
    /// (`on_runway`: half-width + 10 m, 2 % past each end) still takes a
    /// ghost 30 m off the centreline or 50 m past the 12 end, the landing
    /// judgement does not.
    #[test]
    fn a_runway_occupant_is_on_the_pavement_not_the_strip_area() {
        let r = &lemg_runways()[0];
        let on = |along: f64, cross: f64| {
            let (lat, lon) = lemg30_point(along, cross);
            (
                on_runway(r, lat, lon).is_some(),
                on_runway_pavement(r, lat, lon).is_some(),
            )
        };
        assert_eq!(on(1400.0, 0.0), (true, true), "on the centreline");
        assert_eq!(on(1400.0, 25.0), (true, true), "a mainwheel on the edge");
        assert_eq!(
            on(1400.0, 30.0),
            (true, false),
            "30 m out is off the pavement"
        );
        assert_eq!(on(1400.0, 90.0), (false, false), "the holding point");
        assert_eq!(
            on(2755.0 + 50.0, 0.0),
            (true, false),
            "50 m past the 12 end"
        );
        assert_eq!(on(-20.0, 0.0), (true, true), "the 30 threshold's overrun");
    }

    /// Issue #162, the recorded order: SR22 on LEMG 30 final, 0.95 nm, 34 s
    /// (t 1790611040.7), wind 300 at 4. No ghost position was on file; the
    /// timing says what VL1834 was: not on the strip at the final report
    /// (t 1790610897.9, cleared to land) and no `go_around_deferred` while
    /// we were under 800 ft and more than 60 s out, so it reached the
    /// pavement within the last half minute, while the live stream was
    /// landing 12 (the party line's Ryanair arrivals). A 12 lander touching
    /// down head-on. Its track now says so (the airborne samples before the
    /// wheels give it a velocity at first contact: `oncoming`), the order
    /// still goes (it cannot be off in time), and it names the runway the
    /// stream is using: "expect runway one two". The FSM sync moves the
    /// runway in use and the published clearance, and the crew's "going
    /// around" is answered with it.
    #[test]
    fn lemg30_a_head_on_lander_sends_us_round_and_we_are_resequenced_to_twelve() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        lemg30_cleared(lemg_runways(), (300.0, 4.0));
        let base = Instant::now();
        let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
        // VL1834 on the 12 final at 140 kt, then on the wheels near the 12
        // threshold, rolling toward us.
        let lander = |along: f64, on_ground: bool| {
            let (lat, lon) = lemg30_point(along, 0.0);
            TcasTarget {
                callsign: "VL1834".into(),
                lat,
                lon,
                ele_ft: if on_ground { 50.0 } else { 120.0 },
                on_ground,
                vs_fpm: if on_ground { 0.0 } else { -700.0 },
            }
        };
        let len = 2755.0;
        for (t, along, gnd) in [
            (0.0, len + 206.0, false),
            (1.0, len + 134.0, false),
            (2.0, len + 62.0, false),
        ] {
            own_on_lemg30_final(0.95 + (3.0 - t) * 0.028, at(t));
            assert!(go_around_tick(&[lander(along, gnd)], 380.0, false, true, at(t)).is_none());
        }
        own_on_lemg30_final(0.95, at(3.0));
        let touchdown = lander(len - 10.0, true);
        let exits = strip_exits(
            std::slice::from_ref(&touchdown),
            Some(&lemg_runways()[0]),
            "30",
            at(3.0),
        );
        assert_eq!(exits.len(), 1);
        assert_eq!(
            exits[0].exit,
            StripExit::Oncoming,
            "judged from its track at first contact"
        );
        *STRIP_SEEN.lock().unwrap() = None;
        for (t, along, gnd) in [
            (0.0, len + 206.0, false),
            (1.0, len + 134.0, false),
            (2.0, len + 62.0, false),
        ] {
            assert!(go_around_tick(&[lander(along, gnd)], 380.0, false, true, at(t)).is_none());
        }
        let call = go_around_tick(
            std::slice::from_ref(&touchdown),
            359.0,
            false,
            true,
            at(3.0),
        )
        .expect("34 s out with a lander rolling at us: the order");
        assert_eq!(
            call.line,
            format!(
                "{SR22}, go around, traffic on the runway. Climb to circuit altitude, \
                 expect runway one two, report downwind."
            )
        );
        assert_eq!(
            *PENDING_RESEQUENCE.lock().unwrap(),
            Some(("30".to_string(), "12".to_string()))
        );
        // The FSM sync.
        let mut st = lxgb();
        st.apt.name = "Malaga".into();
        st.apt.elev_ft = 52.0;
        st.apt.runways = lemg_runways();
        st.runway_in_use = Some("30".into());
        st.twr.state = tower::State::ClearedToLand;
        st.apply_monitor_go_around();
        assert_eq!(st.runway_in_use.as_deref(), Some("12"));
        assert_eq!(
            CLEARANCE.lock().unwrap().as_ref().unwrap().runway_ident,
            "12"
        );
        assert_eq!(
            CLEARANCE.lock().unwrap().as_ref().unwrap().state,
            tower::State::Inbound
        );
        // The crew's call, as transmitted.
        let wx = Wx {
            wind_from_deg: 300.0,
            wind_kts: 4.0,
        };
        let ctx = st.make_ctx(
            wx,
            String::new(),
            SR22.into(),
            false,
            false,
            None,
            true,
            None,
        );
        assert_eq!(
            ctx.runway, "one two",
            "the runway in use moved with the order"
        );
        let r = st.route_keyed(
            "november five five zero charlie romeo, going around",
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(
            r,
            format!(
                "{SR22}, roger, go around. Climb to circuit altitude, runway heading. \
                 Runway three zero is blocked, expect runway one two, report downwind."
            )
        );
        // Said once: the next go-around call is the plain answer.
        assert!(st.resequenced.is_none());
        reset_runway_watch();
    }

    /// Issue #162, part 3: a STANDING occupant blocks its whole strip, so the
    /// reciprocal is no answer; the parallel is (LEMG 31 in 310 at 8, 3,206 m
    /// against 30's 2,755). With no other runway the tower holds: the order
    /// names none, the downwind is number two behind the traffic, and the
    /// final report is withheld until the strip clears.
    #[test]
    fn a_standing_occupant_resequences_to_another_strip_or_holds() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let base = Instant::now();
        let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
        let squatter = ghost_at("EZY7038", lemg30_point(1400.0, 3.0));
        // Pure: the pick and its refusals.
        let rw = lemg_runways();
        assert_eq!(
            resequence_runway(&rw, "30", false, &["12".into()], &[], Some((310.0, 8.0))),
            Ok("31".to_string())
        );
        assert_eq!(
            resequence_runway(
                &rw,
                "30",
                false,
                &["12".into(), "13".into()],
                &[],
                Some((310.0, 8.0))
            ),
            Err("no_other_runway")
        );
        assert_eq!(
            resequence_runway(&rw, "30", false, &["12".into()], &[], None),
            Err("no_wind")
        );
        // 31 against a live 12 stream is refused; 13 in calm is not.
        assert_eq!(
            resequence_runway(
                &rw,
                "30",
                false,
                &["12".into()],
                &["12".into()],
                Some((300.0, 3.0))
            ),
            Ok("13".to_string())
        );
        // Oncoming: the reciprocal, with the stream.
        assert_eq!(
            resequence_runway(&rw, "30", true, &["12".into()], &[], Some((300.0, 4.0))),
            Ok("12".to_string())
        );
        // ...but not into a 20 kt tailwind, and never the parallel against the
        // oncoming direction (LEMG 31 against 12): no runway, the hold.
        assert_eq!(
            resequence_runway(&rw, "30", true, &["12".into()], &[], Some((300.0, 20.0))),
            Err("no_other_runway")
        );

        // Live: two strips, the squatter on 12/30.
        lemg30_cleared(lemg_runways(), (310.0, 8.0));
        own_on_lemg30_final(0.95, at(1.0));
        let call = go_around_tick(std::slice::from_ref(&squatter), 380.0, false, true, at(1.0))
            .expect("the order");
        assert!(
            call.line.contains("expect runway three one"),
            "{}",
            call.line
        );
        assert_eq!(
            *PENDING_RESEQUENCE.lock().unwrap(),
            Some(("30".to_string(), "31".to_string()))
        );

        // One strip only: the order stands alone, nothing re-sequenced.
        lemg30_cleared(vec![lemg_runways()[0].clone()], (310.0, 8.0));
        own_on_lemg30_final(0.95, at(1.0));
        let call = go_around_tick(std::slice::from_ref(&squatter), 380.0, false, true, at(1.0))
            .expect("the order");
        assert_eq!(
            call.line,
            format!("{SR22}, go around, traffic on the runway. Climb to circuit altitude, report downwind.")
        );
        assert!(PENDING_RESEQUENCE.lock().unwrap().is_none());
        // The hold: downwind number two, the final continued, not cleared.
        let mut st = lxgb();
        st.apt.name = "Malaga".into();
        st.apt.runways = vec![lemg_runways()[0].clone()];
        st.runway_in_use = Some("30".into());
        st.twr.state = tower::State::ClearedToLand;
        st.apply_monitor_go_around();
        assert_eq!(st.runway_in_use.as_deref(), Some("30"));
        let wx = Wx {
            wind_from_deg: 310.0,
            wind_kts: 8.0,
        };
        let ctx = st.make_ctx(
            wx,
            String::new(),
            SR22.into(),
            false,
            false,
            None,
            true,
            None,
        );
        let r = st.route_keyed(
            "november five five zero charlie romeo, downwind runway three zero",
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(
            r,
            format!("{SR22}, number two, traffic on the runway, report final.")
        );
        let r = st.route_keyed(
            "november five five zero charlie romeo, final runway three zero",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("continue approach runway three zero"), "{r}");
        assert!(crate::party_line::landing_hold_armed());
        reset_runway_watch();
    }

    /// Issue #162, part 2: a ghost standing on the runway past
    /// GHOST_STALE_S is stale data. At 4 minutes it is still traffic (the
    /// order); at 5 it is ignored (no order, and a withheld landing is
    /// released); moved 20 m, its clock starts again.
    #[test]
    fn a_ghost_standing_on_the_runway_for_five_minutes_is_stale() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let base = Instant::now();
        let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
        let frozen = ghost_at("VY3149", lemg30_point(1400.0, 0.0));
        lemg30_cleared(vec![lemg_runways()[0].clone()], (300.0, 4.0));
        // Seen through the approach, high: only the clocks run.
        for t in [0.0, 60.0, 120.0, 180.0] {
            assert!(
                go_around_tick(std::slice::from_ref(&frozen), 2000.0, false, true, at(t)).is_none()
            );
        }
        own_on_lemg30_final(0.95, at(240.0));
        assert!(
            go_around_tick(std::slice::from_ref(&frozen), 380.0, false, true, at(240.0)).is_some(),
            "four minutes standing: still an occupant"
        );
        assert!((standing_secs("VY3149", at(240.0)) - 240.0).abs() < 1e-6);

        // Five minutes: stale, no order; and the withheld landing goes.
        *GO_AROUND_LAST.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Inbound,
            callsign: SR22.into(),
            runway_ident: "30".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *LANDING_HOLD.lock().unwrap() = Some(LandingHold {
            callsign: SR22.into(),
            runway: "three zero".into(),
            runway_ident: "30".into(),
            line: format!("{SR22}, runway three zero, cleared to land."),
        });
        own_on_lemg30_final(0.95, at(301.0));
        assert!(
            go_around_tick(std::slice::from_ref(&frozen), 380.0, false, true, at(301.0)).is_none()
        );
        assert!(
            landing_hold_tick(std::slice::from_ref(&frozen), false, at(301.0)).is_some(),
            "a stale ghost does not hold the landing"
        );
        // Moved 20 m: a live aircraft again.
        let moved = ghost_at("VY3149", lemg30_point(1420.0, 0.0));
        let exits = strip_exits(
            std::slice::from_ref(&moved),
            Some(&lemg_runways()[0]),
            "30",
            at(302.0),
        );
        assert_eq!(exits.len(), 1, "moved, its clock starts again");
        assert!(standing_secs("VY3149", at(302.0)) < 1.0);
        reset_runway_watch();
    }

    /// A withheld landing is the go-around watch's too, and it retires with
    /// the landing or a new answer.
    #[test]
    fn a_withheld_landing_is_ordered_around_or_retired() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_RUNWAYS.lock().unwrap() = lemg_runways();
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Inbound,
            callsign: "November eight one six November November".into(),
            runway_ident: "13".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *GO_AROUND_LAST.lock().unwrap() = None;
        *PENDING_GO_AROUND.lock().unwrap() = false;
        *STRIP_SEEN.lock().unwrap() = None;
        OWN_FIXES.lock().unwrap().clear();
        let hold = || LandingHold {
            callsign: "November eight one six November November".into(),
            runway: "one three".into(),
            runway_ident: "13".into(),
            line: "cleared to land.".into(),
        };
        *LANDING_HOLD.lock().unwrap() = Some(hold());
        let base = Instant::now();
        let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
        // A ghost standing mid-runway, us at a mile: the order, and the hold
        // dies with it.
        let squatter = ghost_at("EZY7038", lemg13_point(1600.0, 0.0));
        let (lat, lon) = lemg13_point(-1852.0 - 77.0, 0.0);
        note_own_fix(lat, lon, false, at(0.0));
        let (lat, lon) = lemg13_point(-1852.0, 0.0);
        note_own_fix(lat, lon, false, at(1.0));
        assert!(
            go_around_tick(std::slice::from_ref(&squatter), 400.0, false, true, at(1.0)).is_some()
        );
        assert!(!crate::party_line::landing_hold_armed());
        // Landed with the hold standing: dropped, nothing said.
        *LANDING_HOLD.lock().unwrap() = Some(hold());
        assert!(landing_hold_tick(&[], true, at(2.0)).is_none());
        assert!(!crate::party_line::landing_hold_armed());
        // A new flight retires it.
        *LANDING_HOLD.lock().unwrap() = Some(hold());
        retire_for_new_flight();
        assert!(!crate::party_line::landing_hold_armed());
        NEW_FLIGHT_PENDING.store(false, std::sync::atomic::Ordering::SeqCst);
        *CLEARANCE.lock().unwrap() = None;
        *PENDING_GO_AROUND.lock().unwrap() = false;
        *GO_AROUND_LAST.lock().unwrap() = None;
        *STRIP_SEEN.lock().unwrap() = None;
        OWN_FIXES.lock().unwrap().clear();
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn a_monitor_go_around_voids_the_clearance_and_flips_the_coach() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() = Some((
            "Gibraltar".into(),
            11.0,
            vec![("tower".to_string(), 131200_i64)],
        ));
        *ATIS.lock().unwrap() = None;
        *TAXI_ROUTE.lock().unwrap() = None;
        *VECTORS.lock().unwrap() = None;
        *LANDING_HANDOFF.lock().unwrap() = None;
        let cs = "November one seven two Sierra Papa";
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::ClearedToLand,
            callsign: cs.into(),
            runway_ident: "27".into(),
            pending_readback: Some(format!("Cleared to land runway two seven, {cs}.")),
            holds_runway: false,
        });
        let mut st = lxgb();
        st.twr.state = tower::State::ClearedToLand;
        // Before the sync: the coach suggests the landing readback — the
        // exact contradiction from the 2026-07-14 tape.
        let s = suggest_call(cs, false, Some(1.0), None).unwrap();
        assert!(
            s.call.as_deref().unwrap().contains("Cleared to land"),
            "{:?}",
            s.call
        );

        st.apply_monitor_go_around();

        assert_eq!(
            st.twr.state,
            tower::State::Inbound,
            "clearance void in the FSM"
        );
        assert!(st.twr.pending().is_none(), "the owed readback dies with it");
        assert_eq!(
            CLEARANCE.lock().unwrap().as_ref().unwrap().state,
            tower::State::Inbound
        );
        // The coach's next line is the acknowledgment, urgent, and it parses.
        let s = suggest_call(cs, false, Some(1.0), None).unwrap();
        let call = s.call.expect("a call");
        assert!(call.contains("Going around"), "{call}");
        assert!(s.urgent, "the go-around ack is the say-now line");
        assert_eq!(tower::parse_intent(&call), tower::Intent::GoAround);

        *CLEARANCE.lock().unwrap() = None;
        *ACTIVE_PANEL.lock().unwrap() = None;
    }

    #[test]
    fn held_takeoff_clearance_releases_when_the_final_stays_clear() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // No wake clock: this test is about the TRAFFIC gate alone, and a
        // stale one left by another test would block every tick.
        *WAKE.lock().unwrap() = None;
        *TAKEOFF_HOLD.lock().unwrap() = Some(TakeoffHold {
            callsign: "November one seven two Sierra Papa".into(),
            runway: "zero niner".into(),
            runway_ident: "09".into(),
            clear_since: None,
            armed_at: Instant::now(),
            logged_at: Instant::now(),
            lined_up: false,
            behind: false,
            held_for: None,
            cut_in_logged: Default::default(),
        });
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        });
        // Our strip: 09/27. The watches only care about THIS pavement.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        let elev = 11.0;
        // An arrival in OUR corridor, 2 nm out at ~400 ft; a PARALLEL
        // arrival the same distance out but 2 km abeam; a ghost parked on
        // our strip.
        let ours = TcasTarget {
            callsign: "BA1".into(),
            lat: 36.1510,
            lon: -5.3554 - 0.037,
            ele_ft: 411.0,
            on_ground: false,
            vs_fpm: -600.0,
        };
        let parallel = TcasTarget {
            callsign: "BA2".into(),
            lat: 36.1690,
            ..ours.clone()
        };
        let squatter = TcasTarget {
            callsign: "BA3".into(),
            lat: 36.15155,
            lon: -5.342,
            ele_ft: elev,
            on_ground: true,
            vs_fpm: 0.0,
        };
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // Traffic on OUR final inside the departure gap: no release. The
        // strip itself is clear and the lander is far enough out, so the
        // tower STAGES us instead of parking us behind the line — once.
        let staged = takeoff_hold_tick(std::slice::from_ref(&ours), elev, true, t0)
            .expect("line up and wait");
        assert!(
            staged.contains("line up and wait runway zero niner"),
            "{staged}"
        );
        assert!(staged.contains("landing traffic"), "{staged}");
        assert!(*PENDING_LINE_UP.lock().unwrap());
        *PENDING_LINE_UP.lock().unwrap() = false;
        assert!(
            takeoff_hold_tick(std::slice::from_ref(&ours), elev, true, t0).is_none(),
            "one shot: the aircraft can only take the runway once"
        );
        // Only the PARALLEL runway's arrival: not our traffic — the timer
        // runs (the runway-blind check held a Heathrow 09R departure forever).
        assert!(
            takeoff_hold_tick(std::slice::from_ref(&parallel), elev, true, t0 + sec(1)).is_none()
        );
        // A ghost sitting on our strip resets it again.
        assert!(
            takeoff_hold_tick(std::slice::from_ref(&squatter), elev, true, t0 + sec(5)).is_none()
        );
        assert!(
            takeoff_hold_tick(std::slice::from_ref(&parallel), elev, true, t0 + sec(6)).is_none()
        );
        // Our runway clear and stayed clear (parallel stream still landing):
        // the tower comes back unprompted.
        let line = takeoff_hold_tick(std::slice::from_ref(&parallel), elev, true, t0 + sec(25))
            .expect("release");
        assert!(line.contains("traffic clear"), "{line}");
        assert!(
            line.contains("Runway zero niner, cleared for takeoff"),
            "{line}"
        );
        // The compliance picture moved with it, and the FSM sync is armed.
        assert_eq!(
            CLEARANCE.lock().unwrap().as_ref().unwrap().state,
            tower::State::TakeoffCleared
        );
        assert!(*PENDING_TAKEOFF_CLEARANCE.lock().unwrap());
        // One shot: the hold is consumed.
        assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(60)).is_none());

        // Airborne, a held clearance is VOID, not released: the pilot who
        // departed through the cancel got "traffic clear, cleared for
        // takeoff" two minutes into the climb (Gatwick, 2026-07-13).
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *TAKEOFF_HOLD.lock().unwrap() = Some(TakeoffHold {
            callsign: "November one seven two Sierra Papa".into(),
            runway: "zero niner".into(),
            runway_ident: "09".into(),
            clear_since: Some(t0),
            armed_at: t0,
            logged_at: t0,
            lined_up: false,
            behind: false,
            held_for: None,
            cut_in_logged: Default::default(),
        });
        assert!(takeoff_hold_tick(&[], elev, false, t0 + sec(120)).is_none());
        assert!(
            TAKEOFF_HOLD.lock().unwrap().is_none(),
            "airborne must void the hold"
        );
        assert!(
            takeoff_hold_tick(&[], elev, true, t0 + sec(180)).is_none(),
            "a voided hold never releases, even back on the ground"
        );
        assert!(!*PENDING_TAKEOFF_CLEARANCE.lock().unwrap());

        *CLEARANCE.lock().unwrap() = None;
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    /// Gatwick 26L, 2026-07-27, with injected traffic: held at holding point
    /// Mike behind an arrival at half a mile, and the release never came —
    /// eight minutes, then a hand-back. Every plane landed except ours. A
    /// single-runway field working a landing every couple of minutes NEVER
    /// has an empty four-mile final, so requiring the whole corridor to be
    /// clear made the release unreachable; a real tower threads the departure
    /// into the gap instead. The runway itself stays absolute.
    #[test]
    fn a_busy_arrival_stream_still_gets_a_departure_away() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        let elev = 202.0;
        *WAKE.lock().unwrap() = None;
        // Gatwick's main strip.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 45.0,
            surface: 0,
            ident_a: "08R".into(),
            ident_b: "26L".into(),
            lat_a: 51.1512,
            lon_a: -0.1997,
            lat_b: 51.1560,
            lon_b: -0.1424,
        }];
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November two four four Kilo Romeo".into(),
            runway_ident: "26L".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *PENDING_LINE_UP.lock().unwrap() = false;
        *TAKEOFF_HOLD.lock().unwrap() = Some(TakeoffHold {
            callsign: "November two four four Kilo Romeo".into(),
            runway: "two six left".into(),
            runway_ident: "26L".into(),
            clear_since: None,
            armed_at: t0,
            logged_at: t0,
            lined_up: false,
            behind: false,
            held_for: None,
            cut_in_logged: Default::default(),
        });
        // An arrival on the 26L final: `nm` beyond the 26L threshold along the
        // extended centreline (landing westbound, so the approach lies east of
        // the field), on a 3-degree slope.
        let m_lon = 111_320.0 * 51.154_f64.to_radians().cos();
        let (ux, uy) = {
            let (dx, dy) = ((-0.1424 + 0.1997) * m_lon, (51.1560 - 51.1512) * 111_320.0);
            let len = (dx * dx + dy * dy).sqrt();
            (dx / len, dy / len)
        };
        let arrival = |nm: f64| TcasTarget {
            callsign: "EZY83".into(),
            lat: 51.1560 + nm * 1852.0 * uy / 111_320.0,
            lon: -0.1424 + nm * 1852.0 * ux / m_lon,
            ele_ft: elev + 300.0 * nm,
            on_ground: false,
            vs_fpm: -700.0,
        };
        // Half a mile out: too close to stage, too close to release. This is
        // exactly where the live hold began.
        assert!(takeoff_hold_tick(std::slice::from_ref(&arrival(0.5)), elev, true, t0).is_none());
        assert!(
            !*PENDING_LINE_UP.lock().unwrap(),
            "never onto the runway under a lander"
        );

        // Two miles out, the strip momentarily clear: too close to depart
        // ahead of, far enough to STAGE. The tower puts us on the pavement
        // rather than parking us behind the line for a stream that never ends.
        let line = takeoff_hold_tick(
            std::slice::from_ref(&arrival(2.0)),
            elev,
            true,
            t0 + sec(60),
        )
        .expect("line up and wait");
        assert!(
            line.contains("line up and wait runway two six left"),
            "{line}"
        );
        assert!(line.contains("landing traffic"), "{line}");
        assert!(*PENDING_LINE_UP.lock().unwrap());
        assert!(
            takeoff_hold_tick(
                std::slice::from_ref(&arrival(1.8)),
                elev,
                true,
                t0 + sec(70)
            )
            .is_none(),
            "one shot: the aircraft can only take the runway once"
        );
        // ...and while it rolls out ON our strip, no release at all.
        let rollout = TcasTarget {
            callsign: "EZY83".into(),
            lat: 51.1540,
            lon: -0.1700,
            ele_ft: elev,
            on_ground: true,
            vs_fpm: 0.0,
        };
        assert!(
            takeoff_hold_tick(std::slice::from_ref(&rollout), elev, true, t0 + sec(120)).is_none()
        );

        // Vacated, with the next arrival 3.5 nm out and closing: THIS is the
        // gap a real tower departs into. Under the old all-or-nothing gate
        // that aircraft was still "traffic on final" — inside the 4 nm
        // corridor — and the departure sat here until it gave up.
        assert!(
            takeoff_hold_tick(
                std::slice::from_ref(&arrival(3.5)),
                elev,
                true,
                t0 + sec(130)
            )
            .is_none(),
            "the clear window has to stabilise first"
        );
        let line = takeoff_hold_tick(
            std::slice::from_ref(&arrival(3.2)),
            elev,
            true,
            t0 + sec(146),
        )
        .expect("release");
        assert!(line.contains("traffic clear"), "{line}");
        assert!(
            line.contains("Runway two six left, cleared for takeoff"),
            "{line}"
        );
        assert!(*PENDING_TAKEOFF_CLEARANCE.lock().unwrap());

        *CLEARANCE.lock().unwrap() = None;
        *PENDING_LINE_UP.lock().unwrap() = false;
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    /// Heathrow, 2026-07-27: held at holding point Sierra for 27L, full auto
    /// sat out its entire 480 s budget and handed the aircraft back. The
    /// runway had clear gaps and no arrival was in our corridor — but a
    /// liftoff off 27L about every two minutes re-armed a 120 s wake clock
    /// faster than it could expire, so the release condition was unreachable.
    /// The hold must now sequence the departure onto the runway and, once one
    /// full wake interval has been served at the hold, release it.
    #[test]
    fn a_busy_departure_field_sequences_the_hold_then_releases_it() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        let elev = 83.0;
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 50.0,
            surface: 0,
            ident_a: "09R".into(),
            ident_b: "27L".into(),
            lat_a: 51.4647701,
            lon_a: -0.4867489,
            lat_b: 51.4649460,
            lon_b: -0.4340638,
        }];
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November two four four Kilo Romeo".into(),
            runway_ident: "27L".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *PENDING_LINE_UP.lock().unwrap() = false;
        *TAKEOFF_HOLD.lock().unwrap() = Some(TakeoffHold {
            callsign: "November two four four Kilo Romeo".into(),
            runway: "two seven left".into(),
            runway_ident: "27L".into(),
            clear_since: None,
            armed_at: t0,
            logged_at: t0,
            lined_up: false,
            behind: false,
            held_for: None,
            cut_in_logged: Default::default(),
        });
        // A heavy just rotated off our strip.
        let departed = |at: Instant| {
            *WAKE.lock().unwrap() = Some(Wake {
                since: at,
                runway_ident: "27L".into(),
            });
        };
        // A ghost holding on the runway, and the queue's next departure
        // sitting on the threshold — both are ON our strip.
        let squatter = TcasTarget {
            callsign: "BAW1346".into(),
            lat: 51.46494,
            lon: -0.4345,
            ele_ft: elev,
            on_ground: true,
            vs_fpm: 0.0,
        };

        departed(t0);
        // Strip occupied AND the wake clock running: no line-up onto an
        // occupied runway, ever.
        assert!(
            takeoff_hold_tick(std::slice::from_ref(&squatter), elev, true, t0 + sec(1)).is_none()
        );
        assert!(!*PENDING_LINE_UP.lock().unwrap(), "never onto a busy strip");

        // Strip clear, wake still running: sequenced onto the runway rather
        // than left at the taxiway while the queue rolls past.
        let line = takeoff_hold_tick(&[], elev, true, t0 + sec(30)).expect("line up and wait");
        assert!(
            line.contains("line up and wait runway two seven left"),
            "{line}"
        );
        assert!(*PENDING_LINE_UP.lock().unwrap(), "the FSM sync is armed");
        assert_eq!(
            CLEARANCE.lock().unwrap().as_ref().unwrap().state,
            tower::State::LinedUp
        );
        // One shot: the aircraft can only take the runway once.
        assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(31)).is_none());

        // The queue keeps departing ahead of us. Each liftoff used to restart
        // our clock; it no longer does — but inside the first interval the
        // separation we genuinely owe still holds us.
        departed(t0 + sec(60));
        assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(61)).is_none());
        assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(119)).is_none());

        // One full wake interval served at the hold: later departures are the
        // queue's separation, not ours. The clear timer starts...
        departed(t0 + sec(130));
        assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(140)).is_none());
        // ...but a ghost back on the strip still resets it. Safety is kept.
        assert!(
            takeoff_hold_tick(std::slice::from_ref(&squatter), elev, true, t0 + sec(150)).is_none()
        );
        assert!(
            takeoff_hold_tick(&[], elev, true, t0 + sec(160)).is_none(),
            "the clear window restarts behind the squatter"
        );
        let line = takeoff_hold_tick(&[], elev, true, t0 + sec(176)).expect("release");
        assert!(line.contains("traffic clear"), "{line}");
        assert!(
            line.contains("Runway two seven left, cleared for takeoff"),
            "{line}"
        );
        assert!(*PENDING_TAKEOFF_CLEARANCE.lock().unwrap());
        assert_eq!(
            CLEARANCE.lock().unwrap().as_ref().unwrap().state,
            tower::State::TakeoffCleared
        );

        *WAKE.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *PENDING_LINE_UP.lock().unwrap() = false;
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn incursion_traffic_check_ignores_the_other_runways_stream() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() = Some(("Malaga".into(), 11.0, Vec::new()));
        // Two strips 2 km apart, Malaga-style: we hold 12, arrivals land 31.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![
            nav::Runway {
                width_m: 0.0,
                surface: 0,
                ident_a: "12".into(),
                ident_b: "30".into(),
                lat_a: 36.1512,
                lon_a: -5.3554,
                lat_b: 36.1517,
                lon_b: -5.3366,
            },
            nav::Runway {
                width_m: 0.0,
                surface: 0,
                ident_a: "13".into(),
                ident_b: "31".into(),
                lat_a: 36.1692,
                lon_a: -5.3554,
                lat_b: 36.1697,
                lon_b: -5.3366,
            },
        ];
        // An arrival ~2 nm out at ~400 ft in the OTHER strip's corridor: the
        // runway-blind check shouted "exit the runway, expedite" at a cleared
        // runway 12 departure for exactly this picture (Malaga, 2026-07-12).
        let other = TcasTarget {
            callsign: "EZY5701".into(),
            lat: 36.1690,
            lon: -5.3554 - 0.037,
            ele_ft: 411.0,
            on_ground: false,
            vs_fpm: -600.0,
        };
        assert!(
            inbound_final_for_strip(std::slice::from_ref(&other), "12").is_none(),
            "the other runway's stream is not our traffic"
        );
        // The same arrival in OUR corridor earns the shout.
        let ours = TcasTarget {
            lat: 36.1510,
            ..other.clone()
        };
        let nm = inbound_final_for_strip(std::slice::from_ref(&ours), "12").expect("our final");
        assert!((1.0..3.5).contains(&nm), "{nm}");
        // The same position CLIMBING is a departure leaving, not an arrival:
        // the ghost that lifted off runway 30 read as "traffic on short
        // final" to the runway 12 watch (Malaga, 2026-07-12).
        let climber = TcasTarget {
            vs_fpm: 1200.0,
            ..ours.clone()
        };
        assert!(
            inbound_final_for_strip(std::slice::from_ref(&climber), "12").is_none(),
            "a climbing departure is not final traffic"
        );
        // Unknown vertical speed (unfed array) stays counted — fail toward
        // caution.
        let level = TcasTarget {
            vs_fpm: 0.0,
            ..ours.clone()
        };
        assert!(inbound_final_for_strip(std::slice::from_ref(&level), "12").is_some());
        // A strip the field doesn't have: never fires.
        assert!(inbound_final_for_strip(std::slice::from_ref(&ours), "99").is_none());

        *ACTIVE_PANEL.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn the_coach_suggests_the_parsers_happy_path() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() = Some((
            "Gibraltar".into(),
            11.0,
            vec![("tower".to_string(), 131200_i64)],
        ));
        *CLEARANCE.lock().unwrap() = None;
        *ATIS.lock().unwrap() = None;
        *TAXI_ROUTE.lock().unwrap() = None;
        *VECTORS.lock().unwrap() = None;
        *LANDING_HANDOFF.lock().unwrap() = None;
        let cs = "November one seven two Sierra Papa";
        // The transmission itself, for the parse checks below.
        let call = |s: Suggestion| s.call.expect("a call");

        // Cold on the ground: the full initial call.
        let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
        assert!(s.starts_with("Gibraltar Tower"), "{s}");
        assert!(s.contains("request departure clearance"), "{s}");
        // Cold in the air: the inbound call, with position.
        let s = call(suggest_call(cs, false, Some(8.0), None).unwrap());
        assert!(s.contains("8 miles out"), "{s}");
        assert!(s.contains("inbound"), "{s}");
        // Every suggestion must parse to a routable intent — the coach IS
        // the parser's happy path.
        assert_eq!(
            tower::parse_intent(&s),
            tower::Intent::RequestLanding {
                touch_and_go: false
            }
        );

        // Walk the departure states.
        let set = |state: tower::State| {
            *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
                state,
                callsign: cs.into(),
                runway_ident: "09".into(),
                pending_readback: None,
                holds_runway: false,
            });
        };
        set(tower::State::Parked);
        let sug = suggest_call(cs, true, Some(0.5), None).unwrap();
        assert!(!sug.arrival, "parked = departure side");
        let s = call(sug);
        assert!(s.contains("request taxi"), "{s}");
        assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTaxi);
        set(tower::State::TaxiCleared);
        // No taxi network: no invented holding point name.
        let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
        assert!(s.contains("ready for departure"), "{s}");
        assert!(
            !s.contains("holding point"),
            "no route, no invented name: {s}"
        );
        assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTakeoff);
        // With the cleared route published: the REAL holding point + runway.
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![(36.16, -5.36), (36.16, -5.35)],
            via: "Alpha".into(),
            seg: vec![("A".into(), false)],
            holding_point: "A1".into(),
            lead_on: vec![],
        });
        let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
        assert!(s.contains("holding point Alpha one"), "{s}");
        assert!(s.contains("runway zero niner"), "{s}");
        assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTakeoff);
        *TAXI_ROUTE.lock().unwrap() = None;
        set(tower::State::TakeoffCleared);
        let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
        assert!(s.contains("Cleared for takeoff runway zero niner"), "{s}");
        assert!(tower::is_readback(&s.to_lowercase()), "readback: {s}");
        // Arrival side.
        set(tower::State::Inbound);
        let sug = suggest_call(cs, false, Some(2.0), None).unwrap();
        assert!(sug.arrival);
        let s = call(sug);
        assert!(s.contains("final runway zero niner"), "{s}");
        assert_eq!(tower::parse_intent(&s), tower::Intent::ReportFinal);
        set(tower::State::ClearedToLand);
        let s = call(suggest_call(cs, true, Some(0.2), None).unwrap());
        assert!(s.contains("taxi to parking"), "{s}");
        assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTaxiIn);

        // No station: nothing to suggest.
        *ACTIVE_PANEL.lock().unwrap() = None;
        assert!(suggest_call(cs, true, None, None).is_none());
        *CLEARANCE.lock().unwrap() = None;

        // The question forms that summon the coach.
        for q in [
            "what do i say",
            "what should i say to the tower",
            "how do i ask for taxi",
            "what's the call",
            "help me with the radio",
        ] {
            assert!(coach_query(q), "{q:?} should reach the coach");
        }
        // Yes/no shapes about whether a call is owed reach the coach too.
        for q in [
            "should we check in with atc",
            "do we need to call the tower",
            "is it time to contact departure",
            "shall we report to ground",
        ] {
            assert!(coach_query(q), "{q:?} should reach the coach");
            assert!(coach_yes_no(q), "{q:?} is a yes/no");
        }
        assert!(!coach_query("flaps ten"));
        assert!(!coach_query("what frequencies are here"));
        assert!(!coach_query("should we lower the gear"));
        assert!(!coach_yes_no("what do i say"));
    }

    #[test]
    fn the_coach_leads_with_the_prerequisite() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() = Some((
            "Gibraltar".into(),
            11.0,
            vec![("tower".to_string(), 131200_i64)],
        ));
        *CLEARANCE.lock().unwrap() = None;
        *ATIS.lock().unwrap() = None;
        *VECTORS.lock().unwrap() = None;
        *LANDING_HANDOFF.lock().unwrap() = None;
        let cs = "November one seven two Sierra Papa";

        // COM1 parked on another frequency: the call can't be heard — tune
        // first. On frequency (or unknown): no nag.
        let s = suggest_call(cs, true, Some(0.5), Some(122800)).unwrap();
        assert_eq!(
            s.first.as_deref(),
            Some("First tune tower, one three one decimal two."),
            "{:?}",
            s.first
        );
        assert!(s.call.is_some());
        let s = suggest_call(cs, true, Some(0.5), Some(131200)).unwrap();
        assert!(s.first.is_none(), "{:?}", s.first);

        // An ATIS that never cut an edition = nobody listened: copy it
        // before the first call, and it outranks the tune nag (it IS the
        // next tuning action).
        *ATIS.lock().unwrap() = Some(AtisState {
            khz: 118500,
            airport: "Gibraltar".into(),
            runways: Vec::new(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
        let s = suggest_call(cs, true, Some(0.5), Some(122800)).unwrap();
        assert_eq!(
            s.first.as_deref(),
            Some("First copy the ATIS on one one eight decimal five."),
            "{:?}",
            s.first
        );
        // Once in contact the tower's QNH covers you: no more ATIS nag.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::Parked,
            callsign: cs.into(),
            runway_ident: String::new(),
            pending_readback: None,
            holds_runway: false,
        });
        let s = suggest_call(cs, true, Some(0.5), Some(131200)).unwrap();
        assert!(s.first.is_none(), "{:?}", s.first);

        *ATIS.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *ACTIVE_PANEL.lock().unwrap() = None;
    }

    #[test]
    fn the_coach_covers_the_approach_phase() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() = Some((
            "Gibraltar".into(),
            11.0,
            vec![("tower".to_string(), 131200_i64)],
        ));
        *CLEARANCE.lock().unwrap() = None;
        *ATIS.lock().unwrap() = None;
        *LANDING_HANDOFF.lock().unwrap() = None;
        let cs = "November one seven two Sierra Papa";
        let plan = |ils: bool, cleared: bool| VectorPlan {
            callsign: cs.into(),
            thr_lat: 36.15,
            thr_lon: -5.35,
            course_deg: 90.0,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some("one three one decimal two".into()),
            target_alt_ft: 2000.0,
            ils,
            join_nm: 6.0,
            cleared,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        };

        // Under vectors, nothing owed: coach the silence, not a fake call.
        *VECTORS.lock().unwrap() = Some(plan(true, false));
        let s = suggest_call(cs, false, Some(12.0), None).unwrap();
        assert!(
            s.first.as_deref().unwrap().contains("No call owed"),
            "{:?}",
            s.first
        );
        assert!(s.call.is_none());
        assert!(s.arrival);
        // Cleared for the ILS on the intercept: the "established" report is
        // the one call approach waits on — and it parses as the final report.
        *VECTORS.lock().unwrap() = Some(plan(true, true));
        let s = suggest_call(cs, false, Some(10.0), None).unwrap();
        assert!(
            s.first.as_deref().unwrap().contains("established"),
            "{:?}",
            s.first
        );
        let c = s.call.expect("a call");
        assert!(c.contains("established"), "{c}");
        assert_eq!(tower::parse_intent(&c), tower::Intent::ReportFinal);
        *VECTORS.lock().unwrap() = None;

        // Handed to the tower with an approach clearance: the "final" report
        // is owed — same fact the monitor's voice coach reads.
        *LANDING_HANDOFF.lock().unwrap() = Some("zero niner".into());
        let s = suggest_call(cs, false, Some(6.0), None).unwrap();
        let c = s.call.expect("a call");
        assert!(c.contains("final runway zero niner"), "{c}");
        assert_eq!(tower::parse_intent(&c), tower::Intent::ReportFinal);
        *LANDING_HANDOFF.lock().unwrap() = None;
        *ACTIVE_PANEL.lock().unwrap() = None;
    }

    #[test]
    fn spoken_airport_uses_the_radio_name() {
        // Gazetteer entries shorten to what a controller actually keys.
        assert_eq!(spoken_airport("Badajoz- Talavera La Real"), "Badajoz");
        assert_eq!(spoken_airport("Merida - Royanejos"), "Merida");
        // Plain names pass through untouched.
        assert_eq!(spoken_airport("Gibraltar"), "Gibraltar");
        assert_eq!(spoken_airport("Ciudad Real Intl"), "Ciudad Real Intl");
        // Degenerate input survives.
        assert_eq!(spoken_airport("- Somewhere"), "Somewhere");
        assert_eq!(spoken_airport(""), "");
    }

    #[test]
    fn vhf_range_is_line_of_sight() {
        // On the ramp the station's own mast still buys a few miles — enough
        // to hear the field you're parked at, not one 20 nm out.
        let ground = vhf_range_nm(0.0);
        assert!((5.0..12.0).contains(&ground), "{ground}");
        // Range grows with height: at 5,000 ft above the field, most of a
        // sector is reachable; 200 nm (Extremadura → Gibraltar) is not.
        let cruise = vhf_range_nm(5000.0);
        assert!((80.0..110.0).contains(&cruise), "{cruise}");
        assert!(vhf_range_nm(20000.0) < 200.0);
        // Below the field's elevation nothing goes negative.
        assert!(vhf_range_nm(-500.0) >= ground - 1e-9);
    }

    fn lxgb() -> Station {
        Station {
            apt: nav::AirportInfo {
                transition_level_ft: None,
                name: "Gibraltar".into(),
                elev_ft: 11.0,
                runways: Vec::new(),
                freqs: vec![
                    nav::AtcFreq {
                        kind: nav::FREQ_TOWER,
                        khz: 131200,
                        name: "Tower Freq".into(),
                    },
                    nav::AtcFreq {
                        kind: nav::FREQ_APPROACH,
                        khz: 122800,
                        name: "Approach".into(),
                    },
                ],
            },
            tower_khz: 131200,
            approach_khz: Some(122800),
            departure_khz: None,
            twr: tower::Tower::new(),
            runway_in_use: None,
            stated_runway: None,
            stated_on_strip: false,
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
        }
    }

    /// `ATIS` is a process-wide static, same discipline as `SIDE_STATE_LOCK`.
    /// Station side state (`CLEARANCE`/`HANDOFF`/`ATIS`/`REGION`/
    /// `TAKEOFF_HOLD`/`TAXI_ROUTE`/...) is process statics, and the reply
    /// path writes them on EVERY call — so every test that drives a station
    /// or asserts on the statics holds this ONE lock. Three finer locks
    /// (ATIS/REGION/HANDOFF) raced each other through `CLEARANCE`: the coach
    /// test nulled it mid-flight under the readback test (2026-07-11).
    static SIDE_STATE_LOCK: Mutex<()> = Mutex::new(());

    fn wx(wind_deg: f64, wind_kts: f64, qnh: f64) -> AtisWx {
        AtisWx {
            zulu_sec: 12.0 * 3600.0,
            wind_from_deg: wind_deg,
            wind_kts,
            visibility_sm: 10.0,
            qnh_hpa: qnh,
            oat_c: 22.0,
        }
    }

    #[test]
    fn atis_loops_while_tuned_and_cuts_editions_on_real_change() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ATIS.lock().unwrap() = Some(AtisState {
            khz: 118500,
            airport: "Gibraltar".into(),
            runways: Vec::new(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
        let t0 = Instant::now();
        // Off frequency: silence, no edition.
        assert!(atis_tick(131200, &wx(250.0, 12.0, 1018.0), t0).is_none());
        assert_eq!(atis_current_letter(), None, "nothing aired yet");
        // Tuned: the broadcast, information Alpha.
        let r = atis_tick(118500, &wx(250.0, 12.0, 1018.0), t0).expect("broadcast");
        assert!(r.contains("information Alpha"), "{r}");
        assert!(r.contains("QNH one zero one eight"), "{r}");
        assert_eq!(atis_current_letter(), Some('A'));
        // Still tuned, seconds later: the loop is between repeats — silence.
        assert!(atis_tick(
            118500,
            &wx(250.0, 12.0, 1018.0),
            t0 + std::time::Duration::from_secs(10)
        )
        .is_none());
        // Repeat due, weather unchanged: SAME letter.
        let r = atis_tick(
            118500,
            &wx(251.0, 13.0, 1018.2),
            t0 + std::time::Duration::from_secs(50),
        )
        .expect("repeat");
        assert!(
            r.contains("information Alpha"),
            "wobble must not burn a letter: {r}"
        );
        // A real pressure change cuts information Bravo.
        let r = atis_tick(
            118500,
            &wx(250.0, 12.0, 1021.0),
            t0 + std::time::Duration::from_secs(100),
        )
        .expect("new edition");
        assert!(r.contains("information Bravo"), "{r}");
        assert_eq!(atis_current_letter(), Some('B'));
        *ATIS.lock().unwrap() = None;
    }

    /// Anchorage's GA ramp north of 15/33 with Lake Hood Strip next door
    /// (2026-09-10): Anchorage's FIRST runway row is 7R/25L, 2.7 nm away,
    /// so by threshold Lake Hood (0.9 nm) is "nearer" and the whole sortie
    /// was cleared to taxi 1.4 km across Anchorage's aprons to Lake Hood's
    /// network. By footprint the ramp is 0.56 nm from 15/33: Anchorage takes
    /// over, and it is the crew's note, not a handoff.
    fn run_adjacent_fields_case() {
        let rwy = |a: &str, b: &str, la: f64, lo: f64, lb: f64, lob: f64| nav::Runway {
            ident_a: a.into(),
            ident_b: b.into(),
            lat_a: la,
            lon_a: lo,
            lat_b: lb,
            lon_b: lob,
            width_m: 45.0,
            surface: 0,
        };
        let mut anc = region_apt("Ted Stevens Anchorage Intl", 61.1678, -150.0429);
        anc.apt.runways = vec![
            rwy(
                "7R",
                "25L",
                61.1678122,
                -150.0428651,
                61.1678817,
                -149.9726428,
            ),
            rwy(
                "15",
                "33",
                61.1997218,
                -150.0145266,
                61.1710187,
                -149.9984751,
            ),
        ];
        let mut lhd = region_apt("Lake Hood Strip", 61.1895, -149.9685);
        lhd.apt.runways = vec![rwy(
            "14",
            "32",
            61.1894983,
            -149.9684894,
            61.1836374,
            -149.9622678,
        )];
        *REGION.lock().unwrap() = vec![anc, lhd];
        *ACTIVE_FIELD.lock().unwrap() = Some(("Lake Hood Strip".into(), 61.1895, -149.9685));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        let sw =
            region_tick(61.19636, -149.9965, true, Instant::now()).expect("ramp is Anchorage's");
        assert!(sw.crew, "{}", sw.line);
        assert!(sw.line.contains("Ted Stevens Anchorage"), "{}", sw.line);
        // And from Lake Hood's own parallel taxiway (60 m west of 14/32),
        // Lake Hood stays: that pavement is theirs.
        *ACTIVE_FIELD.lock().unwrap() = Some(("Lake Hood Strip".into(), 61.1895, -149.9685));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        assert!(region_tick(61.18743, -149.96737, true, Instant::now()).is_none());
    }

    fn region_apt(name: &str, lat: f64, lon: f64) -> RegionApt {
        RegionApt {
            icao: name.get(0..4).unwrap_or("XXXX").to_uppercase(),
            apt: nav::AirportInfo {
                transition_level_ft: None,
                name: name.into(),
                elev_ft: 0.0,
                runways: Vec::new(),
                freqs: vec![nav::AtcFreq {
                    kind: nav::FREQ_TOWER,
                    khz: 118100,
                    name: "Tower".into(),
                }],
            },
            lat,
            lon,
        }
    }

    #[test]
    fn a_tower_only_strip_loses_the_region_but_wins_its_own_frequency() {
        // LEAX 2026-09-15: Leoni Benabu publishes one row (a tower on 123.5)
        // four miles ahead; Malaga, 17 nm off with ATIS, ground, approach and
        // departure, keeps the sky on the service-weighted pick however
        // close the arrival gets. The crew's retune to 123.5 arms the strip
        // anyway - the same "you call it, you get it" as a keyed transmission.
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut malaga = region_apt("Malaga", 36.675, -4.499);
        malaga.apt.freqs[0].khz = 118155;
        for (kind, khz) in [
            (nav::FREQ_ATIS, 118450),
            (nav::FREQ_GROUND, 121800),
            (nav::FREQ_APPROACH, 118455),
            (nav::FREQ_DEPARTURE, 121025),
        ] {
            malaga.apt.freqs.push(nav::AtcFreq { kind, khz, name: "Malaga".into() });
        }
        let mut strip = region_apt("Leoni Benabu", 36.80, -4.136);
        strip.apt.freqs[0].khz = 123500;
        *REGION.lock().unwrap() = vec![malaga, strip];
        *ACTIVE_FIELD.lock().unwrap() = Some(("Malaga".into(), 36.675, -4.499));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        let now = Instant::now();
        // Airborne 4 nm south of the strip at 2,700 ft: the region keeps Malaga.
        let (lat, lon) = (36.735, -4.15);
        assert!(region_tick(lat, lon, false, now).is_none());
        assert!(PENDING_STATION.lock().unwrap().is_none(), "the service bias holds");
        // The crew tunes the strip's tower: armed.
        arm_station_for_freq(123500, lat, lon, 2700.0, now);
        let armed = PENDING_STATION.lock().unwrap().take().expect("swap armed for the strip");
        assert_eq!(armed.1.name, "Leoni Benabu");
        *REGION.lock().unwrap() = Vec::new();
        *ACTIVE_FIELD.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
    }

    #[test]
    fn a_new_flight_rebuilds_the_station_even_when_it_is_the_one_up() {
        // LEMG 2026-09-26: the 737 was put on a 10 nm final for 13 from a
        // seat on runway 30, and the station that had been watching it on the
        // runway carried on as if nothing had happened. A new flight arms a
        // fresh station for its field, over any swap already waiting, and the
        // old flight's pin and handoff go with it.
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut malaga = region_apt("LEMG Malaga", 36.675, -4.499);
        malaga.apt.freqs[0].khz = 118155;
        let other = region_apt("LXGB Gibraltar", 36.15, -5.35);
        *REGION.lock().unwrap() = vec![malaga, other];
        *ACTIVE_FIELD.lock().unwrap() = Some(("LEMG Malaga".into(), 36.675, -4.499));
        *PENDING_STATION.lock().unwrap() =
            Some(("LXGB".into(), region_apt("LXGB Gibraltar", 36.15, -5.35).apt));
        *ARRIVAL_PIN.lock().unwrap() = Some(131200);
        let now = Instant::now();
        assert_eq!(arm_new_flight("lemg", now), Some(118155));
        let armed = PENDING_STATION.lock().unwrap().take().expect("a fresh station armed");
        assert_eq!(armed.0, "LEMG");
        assert!(ARRIVAL_PIN.lock().unwrap().is_none());
        assert_eq!(*LAST_REGION_SWITCH.lock().unwrap(), Some(now));
        // A field the index lacks arms nothing.
        assert_eq!(arm_new_flight("ZZZZ", now), None);
        assert!(PENDING_STATION.lock().unwrap().is_none());
        *REGION.lock().unwrap() = Vec::new();
        *ACTIVE_FIELD.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
    }

    /// EGKK, 2026-09-28 (#151): after a day at LEMG the runner put the 737 on
    /// Gatwick stand 71 (51.15540 N 0.16537 W). The new-flight watch re-armed
    /// the ACTIVE field (LEMG, tower 118155) and its cooldown held the
    /// region's relocation off for 180 s: full auto departed Gatwick as an
    /// uncontrolled field and looked stand 71 up at LEMG, 1,639 km away. A
    /// new flight's station is the field under its NEW position.
    fn lemg_and_egkk() -> Vec<RegionApt> {
        let mut malaga = region_apt("LEMG Malaga", 36.675, -4.499);
        malaga.apt.freqs[0].khz = 118155;
        let mut gatwick = region_apt("EGKK London Gatwick", 51.1481, -0.1903);
        gatwick.apt.freqs[0].khz = 124230;
        gatwick.apt.runways = vec![nav::Runway {
            ident_a: "08R".into(),
            ident_b: "26L".into(),
            lat_a: 51.14495,
            lon_a: -0.21262,
            lat_b: 51.15132,
            lon_b: -0.16813,
            width_m: 45.0,
            surface: 1,
        }];
        vec![malaga, gatwick]
    }

    #[test]
    fn a_new_flight_takes_the_station_of_the_field_it_was_put_at() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let region = lemg_and_egkk();
        // Stand 71: Gatwick's, whatever station was up.
        let stand_71 = (51.155_395_5, -0.165_369_4);
        let at = field_under_in(&region, stand_71.0, stand_71.1, true).expect("Gatwick");
        assert_eq!(at.icao, "EGKK");
        // Malaga's ramp is Malaga's.
        assert_eq!(field_under_in(&region, 36.6716, -4.4890, true).unwrap().icao, "LEMG");
        // A field with no tower in the index (on the ground 20 nm from both):
        // nobody's, nothing armed.
        assert!(field_under_in(&region, 51.4, -0.6, true).is_none());
        // Airborne on a 10 nm final to Gatwick: Gatwick.
        assert_eq!(field_under_in(&region, 51.12, -0.45, false).unwrap().icao, "EGKK");

        // Live: the station up is Malaga's, a relocation was switched 20 s ago
        // (the cooldown that held the region back), and the new flight at
        // stand 71 arms Gatwick all the same.
        *REGION.lock().unwrap() = region;
        *ACTIVE_FIELD.lock().unwrap() = Some(("LEMG Malaga".into(), 36.675, -4.499));
        *PENDING_STATION.lock().unwrap() = None;
        let now = Instant::now();
        *LAST_REGION_SWITCH.lock().unwrap() = Some(now);
        assert_eq!(
            arm_new_flight_at(stand_71.0, stand_71.1, true, now),
            Some(("EGKK".to_string(), 124230))
        );
        let armed = PENDING_STATION.lock().unwrap().take().expect("Gatwick armed");
        assert_eq!(armed.1.name, "EGKK London Gatwick");
        // Nowhere towered: nothing armed.
        assert_eq!(arm_new_flight_at(51.4, -0.6, true, now), None);
        assert!(PENDING_STATION.lock().unwrap().is_none());
        *REGION.lock().unwrap() = Vec::new();
        *ACTIVE_FIELD.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
    }

    /// Full auto's arm waits for the departure field's station: the one up
    /// must be the field under the wheels, and not the last flight's
    /// worksheet (#151).
    #[test]
    fn the_departure_station_is_the_field_under_the_wheels() {
        use DepartureStation::*;
        let want = || "EGKK".to_string();
        // The EGKK case at the arm: Malaga up, nothing armed.
        assert_eq!(judge_departure_station(Some("LEMG"), Some("EGKK"), None, true), Arm { want: want() });
        assert_eq!(judge_departure_station(Some("LEMG"), Some("EGKK"), None, false), Arm { want: want() });
        // Armed, not yet installed.
        assert_eq!(judge_departure_station(Some("LEMG"), Some("EGKK"), Some("EGKK"), true), Waiting { want: want() });
        // Another field armed (a stale swap): this one is armed over it.
        assert_eq!(judge_departure_station(Some("LEMG"), Some("EGKK"), Some("LEMG"), true), Arm { want: want() });
        // Installed.
        assert_eq!(judge_departure_station(Some("EGKK"), Some("EGKK"), None, false), Ready);
        // The right field, still holding the last flight's worksheet.
        assert_eq!(judge_departure_station(Some("EGKK"), Some("EGKK"), None, true), Arm { want: want() });
        // No station at all yet.
        assert_eq!(judge_departure_station(None, Some("EGKK"), None, false), Arm { want: want() });
        // No towered field under us: an uncontrolled departure is ready as is.
        assert_eq!(judge_departure_station(Some("LEMG"), None, None, true), Ready);
    }

    /// LEMG, 2026-09-28 (sortie 1790584266-approach): the previous case was
    /// cleared to land on 12; the reposition to a 10 nm final for 13 armed
    /// autoland 1.3 s later and it read "cleared to land 12" off the party
    /// line before the crew's new final call went out (event
    /// `runway_from_clearance`, planned 13, cleared 12, xtk -3,638 m on 12's
    /// line). A new flight retires the old one's clearances before anything
    /// reads them, and the old station, still answering until the new one is
    /// installed, publishes nothing.
    #[test]
    fn a_new_flight_retires_the_last_flights_landing_clearance() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::ClearedToLand,
            callsign: "November eight one six November November".into(),
            runway_ident: "12".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *LANDING_HANDOFF.lock().unwrap() = Some("12".into());
        *PENDING_GO_AROUND.lock().unwrap() = true;
        // The last flight's radar watch too (its "radar contact" fired at
        // the next liftoff, LEMG sortie 1790584715).
        let _ = lxgb().assign_squawk(
            "November eight one six November November",
            StationSpeaker::Departure,
        );
        assert!(squawk_watch_active());
        assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("12"));
        crate::party_line::retire_for_new_flight();
        assert!(
            !squawk_watch_active(),
            "the radar watch was the old flight's"
        );
        assert_eq!(crate::party_line::clearance_state(), None);
        assert_eq!(crate::party_line::clearance_runway(), None);
        assert_eq!(crate::party_line::readback_owed(), None);
        assert_eq!(crate::party_line::landing_handoff_runway(), None);
        assert!(!crate::party_line::take_pending_go_around());
        // The old station answers a call before the swap: nothing published.
        let wx = Wx { wind_from_deg: 90.0, wind_kts: 8.0 };
        let mut old = lxgb();
        let ctx = old.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            false,
            false,
            None,
            false,
            None,
        );
        old.respond("", Service::Tower, tower::Intent::ReportFinal, &ctx, wx);
        assert!(
            CLEARANCE.lock().unwrap().is_none(),
            "the old station's word is not this flight's"
        );
        // The new flight's station is up: its replies are the clearance.
        let mut new = lxgb();
        new.install();
        new.respond("", Service::Tower, tower::Intent::ReportFinal, &ctx, wx);
        assert_eq!(
            crate::party_line::clearance_state(),
            Some(tower::State::ClearedToLand)
        );
        Station::uninstall();
    }

    /// LFLL-EGLL, 2026-09-17. Inbound to Heathrow with the inbound call made
    /// and answered, the region's tick found London City eight miles off the
    /// nose against Heathrow's fifteen — nearer AND service-weighted — and
    /// took the station. The final report and the taxi-in request both came
    /// back `void`, the 737 landed on 27L with no clearance and sat on it for
    /// three minutes. A committed arrival owns the station until it is over.
    #[test]
    fn a_committed_arrival_keeps_its_tower_from_a_nearer_field() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut heathrow = region_apt("London Heathrow", 51.4775, -0.4614);
        heathrow.apt.freqs[0].khz = 118505;
        let mut city = region_apt("London City", 51.5050, 0.0554);
        city.apt.freqs[0].khz = 118075;
        for (kind, khz) in [
            (nav::FREQ_ATIS, 136350),
            (nav::FREQ_GROUND, 121825),
            (nav::FREQ_APPROACH, 132700),
        ] {
            city.apt.freqs.push(nav::AtcFreq {
                kind,
                khz,
                name: "City".into(),
            });
        }
        *REGION.lock().unwrap() = vec![heathrow, city];
        *ACTIVE_FIELD.lock().unwrap() = Some(("London Heathrow".into(), 51.4775, -0.4614));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *ARRIVAL_PIN.lock().unwrap() = None;
        // The position the swap actually happened from, 13:06:42Z.
        let (lat, lon) = (51.4, -0.1);
        let now = Instant::now();
        // Unpinned, the generic rule still prefers London City — this is the
        // bug, reproduced, not a rule we are changing for everyone.
        assert!(
            region_tick(lat, lon, false, now).is_some(),
            "the nearer field wins by itself"
        );
        assert_eq!(
            PENDING_STATION.lock().unwrap().take().map(|(_, a)| a.name),
            Some("London City".to_string())
        );
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        // Inbound to Heathrow: the pin holds the station through the zone.
        pin_arrival_station(Some(118505));
        assert!(
            region_tick(lat, lon, false, now).is_none(),
            "the arrival keeps its tower"
        );
        assert!(PENDING_STATION.lock().unwrap().is_none());
        // And the pin is a hold, not a freeze: on the ground the pavement
        // under the wheels still relocates the station, and drops the pin.
        assert!(
            region_tick(51.5050, 0.0554, true, now).is_some(),
            "landed at City after all"
        );
        assert!(
            ARRIVAL_PIN.lock().unwrap().is_none(),
            "a stale pin never strands the station"
        );
        *REGION.lock().unwrap() = Vec::new();
        *ACTIVE_FIELD.lock().unwrap() = None;
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
    }

    #[test]
    fn transmission_on_another_fields_freq_arms_the_swap() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut badajoz = region_apt("Badajoz", 38.89, -6.82);
        badajoz.apt.freqs[0].khz = 122100;
        *REGION.lock().unwrap() = vec![region_apt("Gibraltar", 36.15, -5.35), badajoz];
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        let now = Instant::now();
        // Parked in Extremadura at ~1,000 ft MSL, COM1 on Badajoz tower
        // (22 nm, inside VHF range): the swap arms for Badajoz.
        arm_station_for_freq(122100, 38.74, -6.40, 1075.0, now);
        let armed = PENDING_STATION.lock().unwrap().take().expect("swap armed");
        assert_eq!(armed.1.name, "Badajoz");
        assert!(
            LAST_REGION_SWITCH.lock().unwrap().is_some(),
            "region cooldown stamped"
        );
        // A frequency nobody in range publishes: nothing arms.
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        arm_station_for_freq(999000, 38.74, -6.40, 1075.0, now);
        assert!(PENDING_STATION.lock().unwrap().is_none());
        // The ACTIVE station's own frequency never re-arms a swap to itself.
        *ACTIVE_FIELD.lock().unwrap() = Some(("Badajoz".into(), 38.89, -6.82));
        arm_station_for_freq(122100, 38.74, -6.40, 1075.0, now);
        assert!(PENDING_STATION.lock().unwrap().is_none());
        // Same tune from the ramp at Gibraltar (200+ nm away): out of VHF
        // range, no swap — you can't raise a tower you can't reach.
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        arm_station_for_freq(122100, 36.15, -5.35, 15.0, now);
        assert!(PENDING_STATION.lock().unwrap().is_none());
        *REGION.lock().unwrap() = Vec::new();
        *ACTIVE_FIELD.lock().unwrap() = None;
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
    }

    #[test]
    fn unnamed_gateway_tower_rows_are_not_staffed() {
        // LEMY: an ultralight strip whose air-to-air frequency the Gateway
        // typed as "1054 130125 unnamed entity" — a frequency, not a tower.
        let apt = nav::AirportInfo {
            transition_level_ft: None,
            name: "Merida - Royanejos".into(),
            elev_ft: 883.0,
            runways: Vec::new(),
            freqs: vec![nav::AtcFreq {
                kind: nav::FREQ_TOWER,
                khz: 130125,
                name: "unnamed entity".into(),
            }],
        };
        assert!(apt.staffed_tower().is_none());
        // A real tower row (Badajoz: "TWR CIVIL") stays staffed.
        let apt = nav::AirportInfo {
            transition_level_ft: None,
            name: "Badajoz- Talavera La Real".into(),
            elev_ft: 607.0,
            runways: Vec::new(),
            freqs: vec![nav::AtcFreq {
                kind: nav::FREQ_TOWER,
                khz: 122100,
                name: "TWR CIVIL".into(),
            }],
        };
        assert!(apt.staffed_tower().is_some());
    }

    /// LILB 2026-09-25: parked in Italy with the Gibraltar station active,
    /// the station answered, cleared us and handed over a taxi to its own
    /// runway. On the ground out of reach the station is silent and what it
    /// issued is retired; the crew's own route is left alone; airborne the
    /// last controller keeps the flight.
    #[test]
    fn a_station_out_of_reach_on_the_ground_is_silent_and_retires_its_clearance() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cleared = || ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "N172SP".into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        };
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        *CLEARANCE.lock().unwrap() = Some(cleared());
        *ASSIGNED_SQUAWK.lock().unwrap() = Some(4503);
        *SELF_TAXI.lock().unwrap() = None;
        OUT_OF_REACH.store(false, std::sync::atomic::Ordering::Relaxed);
        // At the field: in reach, nothing retired.
        note_aircraft(36.15, -5.34, true);
        assert!(!out_of_reach());
        assert!(CLEARANCE.lock().unwrap().is_some());
        // Parked at LILB: out of reach, the clearance and squawk are gone.
        note_aircraft(45.77, 9.16, true);
        assert!(out_of_reach());
        assert!(CLEARANCE.lock().unwrap().is_none());
        assert!(ASSIGNED_SQUAWK.lock().unwrap().is_none());
        assert_eq!(crate::party_line::home_tower_khz(), None);
        assert!(!crate::party_line::station_reaches());
        // Airborne the station keeps the flight, and the crew's own route
        // and runway are over.
        *SELF_TAXI.lock().unwrap() = Some("03".into());
        assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("03"));
        note_aircraft(45.77, 9.16, false);
        assert!(!out_of_reach());
        assert_eq!(crate::party_line::clearance_runway(), None);
        // The crew's own route survives the station going out of reach.
        *SELF_TAXI.lock().unwrap() = Some("03".into());
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![(45.77, 9.16), (45.771, 9.161)],
            via: "Alpha".into(),
            seg: Vec::new(),
            holding_point: "A".into(),
            lead_on: Vec::new(),
        });
        note_aircraft(45.77, 9.16, true);
        assert!(TAXI_ROUTE.lock().unwrap().is_some());
        *TAXI_ROUTE.lock().unwrap() = None;
        *SELF_TAXI.lock().unwrap() = None;
        OUT_OF_REACH.store(false, std::sync::atomic::Ordering::Relaxed);
        *ACTIVE_FIELD.lock().unwrap() = None;
    }

    #[test]
    fn only_a_designator_is_a_taxiway_name() {
        for yes in ["A", "B2", "TL", "A13", "k"] {
            assert!(designator(yes), "{yes}");
        }
        for no in ["Taxy", "21_hold", "", "A123", "Main apron", "12"] {
            assert!(!designator(no), "{no}");
        }
    }

    /// The real scenery: LILB's own taxi to runway 03, planned and published.
    /// `cargo test -p atc -- --ignored real_install_self_taxi_lilb --nocapture`
    #[test]
    #[ignore]
    fn real_install_self_taxi_lilb() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let apt = std::env::var("XPLANE_APT_DATA").unwrap_or_else(|_| {
            crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat")
        });
        // Where the 172 sat on 2026-09-25.
        let (lat, lon) = (45.771_275, 9.164_289);
        let t = plan_self_taxi(&apt, Some("LILB"), lat, lon, Some("03"), Some((20.0, 4.0)));
        println!("{t:?}");
        let t = t.expect("planned");
        assert_eq!(t.runway, "03");
        assert!(TAXI_ROUTE.lock().unwrap().as_ref().is_some_and(|w| w.path.len() >= 2));
        // No ICAO: the field found by the scan is the same one.
        let found = plan_self_taxi(&apt, None, lat, lon, None, None).expect("found");
        println!("{found:?}");
        *TAXI_ROUTE.lock().unwrap() = None;
        *SELF_TAXI.lock().unwrap() = None;
    }

    /// The line the LIEA-LEBL 737 heard at FL200 (2026-09-27), and what an
    /// IFR flight hears instead.
    #[test]
    fn the_region_handoff_line_by_flight_rules() {
        assert_eq!(
            region_handoff_line(None, true, "Barcelona Tower", 118_105),
            "Radar services terminated. Squawk seven thousand. Contact Barcelona Tower on one one eight decimal one zero five."
        );
        assert_eq!(
            region_handoff_line(Some("November eight one six November November"), false, "Barcelona Tower", 118_105),
            "November eight one six November November, contact Barcelona Tower on one one eight decimal one zero five."
        );
        assert_eq!(
            region_handoff_line(None, false, "Malaga Tower", 118_100),
            "Radar services terminated. Contact Malaga Tower on one one eight decimal one."
        );
    }

    /// The real region at the EGBB-EGKK switch (sortie 1790534902, +1319 s,
    /// FL150 IFR at 52.1 N 1.4 W): a VFR flight there is still handed to
    /// Oxford's tower, the IFR flight never to a tower-only unit and to a
    /// radar position when it is handed at all.
    /// `cargo test -p atc -- --ignored real_install_region_ifr --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn real_install_region_ifr_handoff_goes_to_radar() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let apt = crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        if !std::path::Path::new(&apt).exists() {
            return;
        }
        load_region(&apt);
        let egbb = REGION
            .lock()
            .unwrap()
            .iter()
            .find(|a| a.icao == "EGBB")
            .map(|a| (a.apt.name.clone(), a.lat, a.lon))
            .expect("EGBB indexed");
        let arm = || {
            *PENDING_STATION.lock().unwrap() = None;
            *LAST_REGION_SWITCH.lock().unwrap() = None;
            *ARRIVAL_PIN.lock().unwrap() = None;
            *ACTIVE_FIELD.lock().unwrap() = Some(egbb.clone());
            *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
                state: tower::State::TakeoffCleared,
                callsign: "Whiskey Alpha Tango four two three six".into(),
                runway_ident: "33".into(),
                pending_readback: None,
                holds_runway: false,
            });
        };
        let t0 = Instant::now();
        // VFR: the old tower handoff stands.
        *IFR_FLIGHT.lock().unwrap() = None;
        arm();
        let vfr = region_tick(52.1, -1.4, false, t0).map(|s| s.line);
        let _ = crate::party_line::take_region_handoff();
        eprintln!("VFR: {vfr:?}");
        assert!(vfr.as_deref().is_some_and(|l| l.contains("Oxford Kidlington Tower")), "{vfr:?}");
        // IFR on 4505, as flown.
        arm();
        publish_ifr_clearance(
            tower::IfrClearance {
                dest: "EGKK".into(),
                sid: Some("ADME1M".into()),
                runway: "33".into(),
                initial_alt_ft: 5000,
                squawk: 4505,
            },
            "Whiskey Alpha Tango four two three six",
            None,
            None,
        );
        let ifr = region_tick(52.1, -1.4, false, t0).map(|s| s.line);
        let h = crate::party_line::take_region_handoff();
        eprintln!("IFR: {ifr:?} {h:?}");
        // Oxford publishes approach 125.09 (and "Oxford Radar"): the flight
        // goes there on radar, its code kept.
        let h = h.expect("handed to a radar position");
        assert_eq!((h.to.as_str(), h.khz, h.ifr_code), ("Oxford Kidlington Approach", 125_090, Some(4505)));
        assert!(!ifr.unwrap_or_default().contains("Tower"));
        *IFR_FLIGHT.lock().unwrap() = None;
        *IFR_CLEARANCE.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = None;
        REGION.lock().unwrap().clear();
    }

    #[test]
    fn region_hands_off_to_the_decisively_nearer_tower() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Gibraltar-ish and Malaga-ish, ~50 nm apart along a degree of longitude.
        *REGION.lock().unwrap() = vec![
            region_apt("Gibraltar", 36.15, -5.35),
            region_apt("Malaga", 36.67, -4.50),
        ];
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        // The fresh-boot branch reads the contact state: start from none,
        // whatever an earlier test left behind.
        *CLEARANCE.lock().unwrap() = None;
        *SQUAWK_WATCH.lock().unwrap() = None;
        let t0 = Instant::now();
        // On the ground at Gibraltar (the active field): never.
        assert!(region_tick(36.15, -5.35, true, t0).is_none());
        // Just airborne, Gibraltar still much nearer: quiet.
        assert!(region_tick(36.2, -5.3, false, t0).is_none());
        // Near the midpoint, Malaga slightly nearer but inside the margin:
        // still quiet — hysteresis, no flapping between two fields.
        assert!(region_tick(36.42, -4.92, false, t0).is_none());
        // On the GROUND near the midpoint (not AT any field): also quiet —
        // a ground relocation needs you physically at the new field.
        assert!(region_tick(36.42, -4.92, true, t0).is_none());
        // Decisively closer to Malaga, airborne — but NOBODY here has talked
        // to Gibraltar this session (fresh boot mid-flight): the crew
        // orients, no controller terminating a service it never gave.
        let sw = region_tick(36.6, -4.6, false, t0).expect("fresh-boot fix");
        assert!(sw.crew, "no contact yet: the crew speaks, not a controller");
        assert!(sw.line.contains("Closest field is Malaga"), "{}", sw.line);
        assert!(!sw.line.contains("Radar services"), "{}", sw.line);
        assert!(
            PENDING_STATION.lock().unwrap().is_some(),
            "swap armed for the main loop"
        );
        // While the swap waits to be consumed: silence, no re-arm.
        assert!(region_tick(36.6, -4.6, false, t0).is_none());
        // Same geometry WITH two-way contact on record: the controller's
        // proper handoff.
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        });
        let _ = crate::party_line::take_region_handoff();
        let sw = region_tick(36.6, -4.6, false, t0).expect("handoff");
        assert!(
            !sw.crew,
            "with a service on record the controller hands off"
        );
        assert!(sw.line.contains("Contact Malaga Tower"), "{}", sw.line);
        assert!(sw.line.contains("one one eight decimal one"), "{}", sw.line);
        // Tower contact only, no radar service: no code to give back, and
        // the crew gets the instruction as data, once.
        assert!(!sw.line.contains("seven thousand"), "{}", sw.line);
        let h = crate::party_line::take_region_handoff().expect("published for the crew");
        assert_eq!(
            h,
            RegionHandoff {
                to: "Malaga Tower".into(),
                khz: 118_100,
                squawk_vfr: false,
                ifr_code: None,
            }
        );
        assert!(
            crate::party_line::take_region_handoff().is_none(),
            "consumed once"
        );
        // Same again with a radar service on record: the discrete code goes
        // back to conspicuity, spoken AND handed down.
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        *SQUAWK_WATCH.lock().unwrap() = Some(SquawkWatch {
            code: 4503,
            callsign: "November one seven two Sierra Papa".into(),
            speaker: StationSpeaker::Departure,
            observed: true,
            check_in_owed: false,
            first_seen: None,
            last_nudge: None,
            nudges: 0,
        });
        let sw = region_tick(36.6, -4.6, false, t0).expect("handoff");
        assert!(sw.line.contains("Squawk seven thousand"), "{}", sw.line);
        let h = crate::party_line::take_region_handoff().expect("published for the crew");
        assert!(h.squawk_vfr, "{h:?}");
        assert_eq!(h.khz, 118_100);
        assert_eq!(h.ifr_code, None, "a VFR service ending");
        // The same radar service, but the flight holds an accepted IFR
        // clearance (LIEA-LEBL, 2026-09-27): handed on with its code, no
        // service terminated, no seven thousand.
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        publish_ifr_clearance(
            tower::IfrClearance {
                dest: "LEMG".into(),
                sid: None,
                runway: "09".into(),
                initial_alt_ft: 5000,
                squawk: 4503,
            },
            "November eight one six November November",
            None,
            None,
        );
        // Malaga here publishes only a tower: an IFR flight in the air is
        // never handed to one (EGBB-EGKK 2026-09-27, "contact Oxford
        // Kidlington Tower" at FL150). The last controller keeps it.
        assert!(region_tick(36.6, -4.6, false, t0).is_none(), "no tower-only handoff for IFR");
        assert!(crate::party_line::take_region_handoff().is_none());
        // With an approach position the field takes the flight on radar: a
        // plain handoff to Malaga APPROACH, the code kept.
        REGION.lock().unwrap()[1].apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_APPROACH,
            khz: 123_950,
            name: "Malaga Approach".into(),
        });
        let sw = region_tick(36.6, -4.6, false, t0).expect("IFR handoff");
        assert!(!sw.crew);
        assert_eq!(
            sw.line,
            "November eight one six November November, contact Malaga Approach on one two three decimal niner five."
        );
        let h = crate::party_line::take_region_handoff().expect("published for the crew");
        assert_eq!(
            h,
            RegionHandoff {
                to: "Malaga Approach".into(),
                khz: 123_950,
                squawk_vfr: false,
                ifr_code: Some(4503),
            }
        );
        // The pinned arrival keeps its tower (the inbound call goes there),
        // tower-only or not.
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
        REGION.lock().unwrap()[1].apt.freqs.pop();
        *ARRIVAL_PIN.lock().unwrap() = Some(118_100);
        let sw = region_tick(36.6, -4.6, false, t0).expect("pinned arrival handoff");
        assert!(sw.line.contains("contact Malaga Tower on one one eight decimal one"), "{}", sw.line);
        let h = crate::party_line::take_region_handoff().expect("published for the crew");
        assert_eq!((h.to.as_str(), h.khz), ("Malaga Tower", 118_100));
        *ARRIVAL_PIN.lock().unwrap() = None;
        // The IFR flight ends at the landing, not before: on the ground at
        // the clearance it stands, airborne it stands, the next ground
        // contact retires it.
        note_aircraft(36.15, -5.35, true);
        assert_eq!(crate::party_line::ifr_flight_code(), Some(4503));
        note_aircraft(36.4, -5.0, false);
        assert_eq!(crate::party_line::ifr_flight_code(), Some(4503));
        note_aircraft(36.15, -5.35, true);
        assert_eq!(crate::party_line::ifr_flight_code(), None, "landed");
        OUT_OF_REACH.store(false, std::sync::atomic::Ordering::Relaxed);
        *IFR_CLEARANCE.lock().unwrap() = None;
        *SQUAWK_WATCH.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        // Ground relocation: parked AT Malaga with Gibraltar still active
        // (the "started at LERL, Gibraltar answered" case) — the CREW notes
        // the local tower, no radar theatre.
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        let sw = region_tick(36.67, -4.50, true, t0).expect("ground relocation");
        assert!(sw.crew, "ground switch is the crew's note");
        assert!(sw.line.contains("We're at Malaga"), "{}", sw.line);
        run_borrowing_strip_case(); // same SIDE_STATE_LOCK scope, statics owned here
        run_adjacent_fields_case();
        *REGION.lock().unwrap() = Vec::new();
        *ACTIVE_FIELD.lock().unwrap() = None;
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
    }

    /// The Trebujena trap (flown 2026-07-11), run inside the region test's
    /// lock scope: a tower-only grass strip 12 nm off the nose whose apt.dat
    /// rows borrow "Jerez Tower" / "Seville Approach", with the actual
    /// destination (ATIS, ground, its own approach) 16 nm out. Raw-distance
    /// nearest picked the strip, and its controller then vectored the Jerez
    /// arrival onto Trebujena's 800 m runway two six.
    fn run_borrowing_strip_case() {
        let mut jerez = region_apt("Jerez", 36.7446, -6.0601);
        jerez.apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_ATIS,
            khz: 125655,
            name: "ATIS".into(),
        });
        jerez.apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_GROUND,
            khz: 133280,
            name: "GND".into(),
        });
        jerez.apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_APPROACH,
            khz: 128500,
            name: "Seville Approach".into(),
        });
        *REGION.lock().unwrap() = vec![region_apt("Trebujena", 36.8587, -6.1413), jerez];
        *ACTIVE_FIELD.lock().unwrap() = Some(("Sevilla San Pablo".into(), 37.418, -5.893));
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *SQUAWK_WATCH.lock().unwrap() = None;
        // Airborne between the two, at the flight's real 13:09 position:
        // the REAL airport wins the handoff despite being farther.
        let sw = region_tick(36.98726, -5.94432, false, Instant::now()).expect("switch");
        assert!(sw.line.contains("Jerez"), "{}", sw.line);
        let pending = PENDING_STATION.lock().unwrap().take().expect("swap armed");
        assert_eq!(
            pending.1.name, "Jerez",
            "vector the arrival to the destination"
        );
        // Parked AT the strip, though: you're at the field you're at — the
        // ground relocation stays raw-nearest, however small the strip.
        *ACTIVE_FIELD.lock().unwrap() = Some(("Jerez".into(), 36.7446, -6.0601));
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        let sw = region_tick(36.8587, -6.1413, true, Instant::now()).expect("ground relocation");
        assert!(sw.line.contains("Trebujena"), "{}", sw.line);
    }

    #[test]
    fn runway_occupancy_is_a_tight_rectangle() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // LXGB 09/27, real-ish geometry: ~1.7 km east-west at 36.151N.
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        // On the centerline past midfield: on the runway, named by the nearer end.
        assert_eq!(on_active_runway(36.15155, -5.342).as_deref(), Some("27"));
        // Near the 09 threshold: still on, named 09.
        assert_eq!(on_active_runway(36.1512, -5.3550).as_deref(), Some("09"));
        // 100 m abeam the centerline (a parallel taxiway): NOT on the runway.
        assert!(on_active_runway(36.1503, -5.346).is_none());
        // A mile away at the parking: no.
        assert!(on_active_runway(36.16, -5.35).is_none());
        // Past the far threshold by more than the overrun margin: no.
        assert!(on_active_runway(36.152, -5.330).is_none());
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn information_letters_parse_from_calls() {
        assert_eq!(heard_information("with information alpha"), Some('A'));
        assert_eq!(
            heard_information("we have information bravo, request taxi"),
            Some('B')
        );
        // "x-ray" splits at the hyphen; the bare "x" carries the letter anyway.
        assert_eq!(heard_information("information x-ray"), Some('X'));
        assert_eq!(heard_information("information b"), Some('B'));
        assert_eq!(heard_information("request taxi"), None);
        assert_eq!(
            heard_information("for your information the flaps are stuck"),
            None
        );
    }

    #[test]
    fn having_the_current_information_skips_the_qnh() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ATIS.lock().unwrap() = Some(AtisState {
            khz: 118500,
            airport: "Gibraltar".into(),
            runways: Vec::new(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
        let _ = atis_tick(118500, &wx(250.0, 12.0, 1018.0), Instant::now()); // airs Alpha
        let w = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        // With the current letter: the taxi clearance drops the QNH clause.
        let mut st = lxgb();
        let ctx = {
            st.note_information("gibraltar tower, with information alpha, request taxi");
            st.make_ctx(
                w,
                "QNH one zero one eight".into(),
                "November".into(),
                true,
                false,
                None,
                false,
                None,
            )
        };
        let reply = st.twr.handle(tower::parse_intent("request taxi"), &ctx);
        assert!(!reply.contains("QNH"), "{reply}");
        // A STALE letter keeps the numbers — safety over brevity.
        let mut st = lxgb();
        st.note_information("with information zulu, request taxi");
        let ctx = st.make_ctx(
            w,
            "QNH one zero one eight".into(),
            "November".into(),
            true,
            false,
            None,
            false,
            None,
        );
        let reply = st.twr.handle(tower::parse_intent("request taxi"), &ctx);
        assert!(reply.contains("QNH one zero one eight"), "{reply}");
        // No letter at all: numbers as always.
        let mut st = lxgb();
        let ctx = st.make_ctx(
            w,
            "QNH one zero one eight".into(),
            "November".into(),
            true,
            false,
            None,
            false,
            None,
        );
        let reply = st.twr.handle(tower::parse_intent("request taxi"), &ctx);
        assert!(reply.contains("QNH one zero one eight"), "{reply}");
        *ATIS.lock().unwrap() = None;
    }

    #[test]
    fn unkeyed_routing_needs_a_radio_shaped_call() {
        // Crew talk that merely contains intent keywords must fall through —
        // "anything else we need to do before takeoff?" was answered with a
        // takeoff clearance on 2026-07-08, and the panel's "land the aircraft"
        // button got a join clearance.
        for crew in [
            "anything else we need to do before takeoff",
            "land the aircraft",
            "flaps set for takeoff",
            "after landing checklist",
        ] {
            assert!(!is_radio_call(crew, "gibraltar"), "{crew}");
        }
        // Addressed or call-vocabulary utterances still route.
        for call in [
            "hello tower, i would like clearance for takeoff",
            "gibraltar, request taxi",
            "request touch and go",
            "inbound for landing",
            "november one seven two, final",
            "radio check",
        ] {
            assert!(is_radio_call(call, "gibraltar"), "{call}");
        }
    }

    /// LXGB at 07001KT (2026-09-15): the tower picked 27 while LiveTraffic
    /// was landing 09, and cleared a departure into the reciprocal stream.
    /// In calm wind the end the arrivals are landing on is the runway in
    /// use; a real wind still outranks it, and a held pick is not moved.
    #[test]
    fn calm_wind_follows_the_arrival_stream() {
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        assert_eq!(st.pick_runway_with(70.0, 1.0, Some("09"), None), "09");
        // Held once we are sequenced: a taxi clearance owns its runway.
        st.twr.state = tower::State::TaxiCleared;
        assert_eq!(st.pick_runway_with(70.0, 1.0, Some("27"), None), "09");
        // Uncommitted, a stream that turned round turns the calm pick with
        // it: one strip, one direction (LEMG 2026-09-28, issue #140).
        st.twr.state = tower::State::Parked;
        assert_eq!(st.pick_runway_with(70.0, 1.0, Some("27"), None), "27");
        // A real wind is not overruled by traffic.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        assert_eq!(st.pick_runway_with(250.0, 12.0, Some("09"), None), "27");
        // An end this field does not have is ignored.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        assert_eq!(st.pick_runway_with(70.0, 1.0, Some("18"), None), st.pick_runway(70.0, 1.0));
    }

    #[test]
    fn the_arrival_end_is_the_threshold_the_traffic_is_beyond() {
        let runways = vec![nav::Runway {
            width_m: 45.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        }];
        let target = |lon: f64, vs: f64, ground: bool| TcasTarget {
            callsign: "HTY103".into(),
            lat: 36.1512,
            lon,
            ele_ft: 700.0,
            on_ground: ground,
            vs_fpm: vs,
        };
        // West of the 09 threshold, descending: landing on 09.
        assert_eq!(arrival_end(&[target(-5.39, -600.0, false)], &runways, 11.0).as_deref(), Some("09"));
        // East of the 27 threshold: landing on 27.
        assert_eq!(arrival_end(&[target(-5.30, -600.0, false)], &runways, 11.0).as_deref(), Some("27"));
        // Climbing away, or on the ground: not an arrival.
        assert_eq!(arrival_end(&[target(-5.39, 800.0, false)], &runways, 11.0), None);
        assert_eq!(arrival_end(&[target(-5.39, -600.0, true)], &runways, 11.0), None);
        // The nearer of two wins.
        let both = [target(-5.39, -600.0, false), target(-5.33, -500.0, false)];
        assert_eq!(arrival_end(&both, &runways, 11.0).as_deref(), Some("27"));
    }

    /// Issue #155, sortie 1790601822-full_auto (EGKK-EGBB, 2026-09-28): the
    /// check-in at +1209.7 "passing four thousand seven hundred climbing
    /// flight level two two zero" earned "radar contact, climb flight level
    /// two two zero" on a plan filed at FL180 (`initial_altitude` 18000).
    /// Departure clears no higher than the filed cruise; at or under it, or
    /// with nothing filed, the announced climb stands.
    #[test]
    fn departure_clears_no_higher_than_the_filed_cruise() {
        let heard = "london gatwick departure, whiskey alpha tango six two six, passing four \
                     thousand seven hundred climbing flight level two two zero";
        let announced = climb_announced(heard).expect("a climb");
        assert_eq!(announced, "flight level two two zero");
        // EGKK's transition level unknown to the test: the crew's own form.
        assert_eq!(
            capped_departure_climb(&announced, Some(18_000), None),
            "flight level one eight zero"
        );
        assert_eq!(
            capped_departure_climb(&announced, Some(18_000), Some(7_000.0)),
            "flight level one eight zero"
        );
        assert_eq!(capped_departure_climb(&announced, None, None), announced);
        assert_eq!(capped_departure_climb(&announced, Some(24_000), None), announced);
        assert_eq!(capped_departure_climb("five thousand", Some(18_000), None), "five thousand");
        // Below the transition level a capped climb is feet.
        assert_eq!(
            capped_departure_climb("seven thousand", Some(4_000), Some(6_000.0)),
            crate::atis::spoken_alt(4_000.0)
        );
    }

    #[test]
    fn the_climb_announced_is_what_departure_clears() {
        assert_eq!(
            climb_announced("gibraltar departure, november one seven two sierra papa, passing one thousand climbing five thousand"),
            Some("five thousand".into())
        );
        assert_eq!(climb_announced("passing two thousand climbing to 5000."), Some("5000".into()));
        assert_eq!(climb_announced("passing one thousand"), None);
        assert_eq!(climb_announced("climbing"), None);
    }

    #[test]
    fn calm_wind_holds_the_runway_in_use() {
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        // A real westerly picks 27.
        assert_eq!(st.pick_runway(250.0, 12.0), "27");
        // Calm air with the direction flopping east must NOT flip the runway —
        // the 2026-07-08 flight was cleared on 27, then re-cleared on 09 two
        // minutes later in the same calm.
        assert_eq!(st.pick_runway(90.0, 1.0), "27");
        assert_eq!(st.pick_runway(170.0, 0.0), "27");
        // Actual wind changes the runway like a real controller would.
        assert_eq!(st.pick_runway(90.0, 10.0), "09");
    }

    #[test]
    fn a_handoff_readback_after_the_swap_is_not_a_void_call() {
        let _g = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let now = Instant::now();
        let readback = "november eight one six november november, squawking seven thousand, carlisle tower on one two three decimal six zero five";
        *LAST_REGION_SWITCH.lock().unwrap() = Some(now);
        assert_eq!(void_outcome(readback, now + std::time::Duration::from_secs(4)), "handoff_readback");
        // A real call to nobody is still a void call...
        assert_eq!(void_outcome("exeter tower, inbound for landing", now), "void");
        // ...and so is a readback long after any handoff.
        assert_eq!(void_outcome(readback, now + std::time::Duration::from_secs(300)), "void");
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        assert_eq!(void_outcome(readback, now), "void");
    }

    /// EGTE, 2026-09-21. "Final runway two six" into a wind dead across the
    /// strip earned "negative, runway zero eight" and a trip round the field;
    /// then the go-around (ClearedToLand back to Inbound) flipped the pick to
    /// 26 and the crew's missed approach was re-aimed from 60 ft.
    #[test]
    fn an_arrival_keeps_its_runway_on_a_wind_that_favours_neither_end() {
        let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
        // Nothing in use yet, wind across (LXGB is 09/27): the end the
        // arrival is lined up on stands.
        st.note_stated_runway("november one two three, final runway two seven", tower::Intent::ReportFinal);
        assert_eq!(st.pick_runway(0.0, 8.0), "27");
        // A wind that really favours the other end still wins.
        let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.note_stated_runway("final runway two seven", tower::Intent::ReportFinal);
        assert_eq!(st.pick_runway(90.0, 12.0), "09");
        // A departure ready to go does not choose its runway...
        let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.note_stated_runway("ready for departure runway two seven", tower::Intent::RequestTakeoff);
        assert_eq!(st.stated_runway, None);
        // ...but asks for one at the taxi request, honoured when the wind
        // does not argue (LOWI, calm, MOGT3H only from 26).
        assert_eq!(
            tower::parse_intent("november two five one charlie victor, request taxi, runway two six"),
            tower::Intent::RequestTaxi
        );
        let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.note_stated_runway("request taxi, runway two seven", tower::Intent::RequestTaxi);
        assert_eq!(st.pick_runway(0.0, 2.0), "27");
        let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.note_stated_runway("request taxi, runway two seven", tower::Intent::RequestTaxi);
        assert_eq!(st.pick_runway(90.0, 12.0), "09", "a real headwind on the other end wins");
        // The crew's inbound call names the runway it set up for (LIPK
        // 2026-09-25: "inbound for landing" alone, then "join final runway
        // one two" to a crew flying the ILS 30).
        let inbound = "november one seven two sierra papa, inbound for landing, runway two seven";
        assert!(matches!(tower::parse_intent(inbound), tower::Intent::RequestLanding { .. }));
        let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.note_stated_runway(inbound, tower::parse_intent(inbound));
        assert_eq!(st.pick_runway(0.0, 4.0), "27");
        // And once an arrival is on a runway the pick is frozen, through the
        // go-around's Inbound, whatever the wind does.
        let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.runway_in_use = Some("09".into());
        for state in [tower::State::ClearedToLand, tower::State::Inbound] {
            st.twr.state = state;
            assert_eq!(st.pick_runway(270.0, 15.0), "09", "{state:?}");
        }
    }

    /// LEMG 12/30 and 13/31 from apt.dat.
    fn lemg_runways() -> Vec<nav::Runway> {
        vec![
            nav::Runway {
                width_m: 45.11,
                surface: 1,
                ident_a: "12".into(),
                ident_b: "30".into(),
                lat_a: 36.6910753,
                lon_a: -4.5078270,
                lat_b: 36.6795648,
                lon_b: -4.4805125,
            },
            nav::Runway {
                width_m: 45.11,
                surface: 1,
                ident_a: "13".into(),
                ident_b: "31".into(),
                lat_a: 36.6845573,
                lon_a: -4.5126253,
                lat_b: 36.6654027,
                lon_b: -4.4858140,
            },
        ]
    }

    /// Issue #140, the recorded runway in use: LEMG (sortie
    /// 1790584715-full_auto, 2026-09-28), wind calm. 31 stood in use from an
    /// earlier into-wind pick while the replay landed 12 (Ryanair 8168,
    /// t 1790585296.6) and departed 13 (Vueling 8176 t 1790585447.7, Easy
    /// 2716 t 1790585551.4). A calm, uncommitted pick turns to the live
    /// stream; a committed one holds; a stated runway into the departures is
    /// refused; a real wind still wins.
    #[test]
    fn the_calm_runway_in_use_turns_to_the_live_stream() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CLEARANCE.lock().unwrap() = None;
        let runways = lemg_runways();
        assert!(ends_oppose(&runways, "31", "13"), "the reciprocal");
        assert!(ends_oppose(&runways, "31", "12"), "the near-parallel's other end");
        assert!(!ends_oppose(&runways, "13", "12"), "one direction");
        let fresh = |in_use: &str| {
            let mut st = lxgb();
            st.apt.name = "Malaga".into();
            st.apt.runways = runways.clone();
            st.runway_in_use = Some(in_use.into());
            st
        };
        // The departures alone turn 31 round to 13.
        let mut st = fresh("31");
        assert_eq!(st.pick_runway_with(350.0, 3.0, None, Some("13")), "13");
        // The arrivals first: 12, the field's landing runway, same direction.
        let mut st = fresh("31");
        assert_eq!(st.pick_runway_with(350.0, 3.0, Some("12"), Some("13")), "12");
        // A pick already with the stream is left alone (13 is not against 12).
        let mut st = fresh("13");
        assert_eq!(st.pick_runway_with(350.0, 3.0, Some("12"), Some("13")), "13");
        // Committed: the circuit's Inbound and a taxi clearance hold 31.
        for state in [tower::State::Inbound, tower::State::TaxiCleared] {
            let mut st = fresh("31");
            st.twr.state = state;
            assert_eq!(st.pick_runway_with(350.0, 3.0, None, Some("13")), "31", "{state:?}");
        }
        // "Downwind runway three one" into the 13 departures, uncommitted:
        // refused, the stream's end instead.
        let mut st = fresh("31");
        st.note_stated_runway("downwind runway three one", tower::Intent::ReportDownwind);
        assert_eq!(st.pick_runway_with(350.0, 3.0, None, Some("13")), "13");
        // With the departures on 31 it stands.
        let mut st = fresh("13");
        st.note_stated_runway("downwind runway three one", tower::Intent::ReportDownwind);
        assert_eq!(st.pick_runway_with(350.0, 3.0, None, Some("31")), "31");
        // A real wind is not overruled by the traffic (310 at 15: 31).
        let mut st = fresh("31");
        assert_eq!(st.pick_runway_with(310.0, 15.0, None, Some("13")), "31");
        // No stream known: a calm pick stands as before.
        let mut st = fresh("31");
        assert_eq!(st.pick_runway_with(350.0, 3.0, None, None), "31");
    }

    /// Issue #140: the departure stream is the END a ghost rolled from, by
    /// its roll direction, remembered for DEPARTURE_STREAM_SECS by the wake
    /// watch (which runs whether or not the party line is on).
    #[test]
    fn a_ghost_liftoff_names_the_departure_stream() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let runways = lemg_runways();
        let (a, b) = ((runways[1].lat_a, runways[1].lon_a), (runways[1].lat_b, runways[1].lon_b));
        let along = |f: f64| (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f);
        *ACTIVE_RUNWAYS.lock().unwrap() = runways.clone();
        let field = along(0.5);
        *ACTIVE_FIELD.lock().unwrap() = Some(("Malaga".into(), field.0, field.1));
        *WAKE.lock().unwrap() = None;
        *WAKE_PREV.lock().unwrap() = None;
        *DEPARTURE_STREAM.lock().unwrap() = None;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        let at = |f: f64, ground: bool| {
            let (lat, lon) = along(f);
            TcasTarget {
                callsign: "VLG8176".into(),
                lat,
                lon,
                ele_ft: if ground { 52.0 } else { 160.0 },
                on_ground: ground,
                vs_fpm: if ground { 0.0 } else { 1500.0 },
            }
        };
        wake_note_targets(&[at(0.70, true)], t0);
        assert!(departure_end(t0).is_none(), "rolling: no stream yet");
        wake_note_targets(&[at(0.78, false)], t0 + sec(3));
        // Lifted off past midfield, nearer the 31 threshold: rolled on 13.
        assert_eq!(departure_end(t0 + sec(4)).as_deref(), Some("13"));
        assert!(departure_end(t0 + sec(3 + DEPARTURE_STREAM_SECS)).is_none(), "stale");

        *WAKE.lock().unwrap() = None;
        *WAKE_PREV.lock().unwrap() = None;
        *DEPARTURE_STREAM.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    /// Issue #164, the recorded LEMG 30 final (sortie 1790610218-approach,
    /// 2026-09-28): the replay landed 12 (Ryanair 431 t 1790610003, Ryanair
    /// 590 t 1790610134) and, with no lander on final at that instant, "final
    /// runway three zero" in 300 at 4 was cleared to land 30 (t 1790610223),
    /// head-on to the stream, and sent round twice. A ghost's touchdown is
    /// remembered for ARRIVAL_STREAM_SECS: the stated 30 is refused for 12
    /// while it is, and stands once the memory lapses. The departure memory
    /// keeps working alongside, and a newer stream that points against the
    /// older one means the field has turned round.
    #[test]
    fn a_ghost_touchdown_remembers_the_arrival_stream() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CLEARANCE.lock().unwrap() = None;
        let runways = lemg_runways();
        let lerp = |p: (f64, f64), q: (f64, f64), f: f64| {
            (p.0 + (q.0 - p.0) * f, p.1 + (q.1 - p.1) * f)
        };
        let (t12, t30) = ((runways[0].lat_a, runways[0].lon_a), (runways[0].lat_b, runways[0].lon_b));
        let (t13, t31) = ((runways[1].lat_a, runways[1].lon_a), (runways[1].lat_b, runways[1].lon_b));
        *ACTIVE_RUNWAYS.lock().unwrap() = runways.clone();
        let field = lerp(t12, t30, 0.5);
        *ACTIVE_FIELD.lock().unwrap() = Some(("Malaga".into(), field.0, field.1));
        *WAKE.lock().unwrap() = None;
        *WAKE_PREV.lock().unwrap() = None;
        *DEPARTURE_STREAM.lock().unwrap() = None;
        *ARRIVAL_STREAM.lock().unwrap() = None;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        let ghost = |id: &str, p: (f64, f64), ground: bool, vs: f64| TcasTarget {
            callsign: id.into(),
            lat: p.0,
            lon: p.1,
            ele_ft: if ground { 52.0 } else { 90.0 },
            on_ground: ground,
            vs_fpm: vs,
        };
        // Ryanair 590 over the 12 threshold, then on the pavement: landed 12.
        wake_note_targets(&[ghost("RYR590", lerp(t12, t30, 0.0), false, -600.0)], t0);
        assert!(arrival_stream_end(t0).is_none(), "airborne: no stream yet");
        wake_note_targets(&[ghost("RYR590", lerp(t12, t30, 0.08), true, 0.0)], t0 + sec(2));
        assert_eq!(arrival_stream_end(t0 + sec(3)).as_deref(), Some("12"));
        // A ghost settling off every strip (a taxiway, a pad) says nothing.
        let off = (field.0 + 0.01, field.1);
        wake_note_targets(&[ghost("GA1", off, false, -300.0)], t0 + sec(4));
        wake_note_targets(&[ghost("GA1", (off.0 + 0.0005, off.1), true, 0.0)], t0 + sec(5));
        assert_eq!(arrival_stream_end(t0 + sec(6)).as_deref(), Some("12"));

        let fresh = || {
            let mut st = lxgb();
            st.apt.name = "Malaga".into();
            st.apt.runways = runways.clone();
            st.runway_in_use = None;
            st
        };
        // 89 s later, no lander on final: the stream is still 12, and the
        // SR22's stated 30 in calm is refused for it.
        let now = t0 + sec(89);
        let (arr, dep) = field_streams(&runways, None, now);
        assert_eq!((arr.as_deref(), dep.as_deref()), (Some("12"), None));
        let mut st = fresh();
        st.note_stated_runway("final runway three zero", tower::Intent::ReportFinal);
        assert_eq!(st.pick_runway_with(300.0, 4.0, arr.as_deref(), dep.as_deref()), "12");
        // The memory lapsed: no stream, the stated 30 stands in calm.
        let later = t0 + sec(2 + ARRIVAL_STREAM_SECS);
        assert!(arrival_stream_end(later).is_none(), "stale");
        let (arr, dep) = field_streams(&runways, None, later);
        assert_eq!((arr.as_deref(), dep.as_deref()), (None, None));
        let mut st = fresh();
        st.note_stated_runway("final runway three zero", tower::Intent::ReportFinal);
        assert_eq!(st.pick_runway_with(300.0, 4.0, arr.as_deref(), dep.as_deref()), "30");

        // The departure memory keeps working beside it: a 13 liftoff (same
        // direction as 12) is kept with the landings.
        let dep_t = t0 + sec(20);
        wake_note_targets(&[ghost("VLG8176", lerp(t13, t31, 0.70), true, 0.0)], dep_t);
        wake_note_targets(&[ghost("VLG8176", lerp(t13, t31, 0.78), false, 1500.0)], dep_t + sec(3));
        assert_eq!(departure_end(dep_t + sec(4)).as_deref(), Some("13"));
        let (arr, dep) = field_streams(&runways, None, dep_t + sec(4));
        assert_eq!((arr.as_deref(), dep.as_deref()), (Some("12"), Some("13")));
        // A newer departure off 31 (against the 12 landings): the field has
        // turned round, the older landing memory is dropped...
        let turn_t = t0 + sec(60);
        wake_note_targets(&[ghost("EZY2716", lerp(t31, t13, 0.70), true, 0.0)], turn_t);
        wake_note_targets(&[ghost("EZY2716", lerp(t31, t13, 0.78), false, 1500.0)], turn_t + sec(3));
        let (arr, dep) = field_streams(&runways, None, turn_t + sec(4));
        assert_eq!((arr.as_deref(), dep.as_deref()), (None, Some("31")));
        // ...and a live lander on 12 outranks that older departure.
        let (arr, dep) = field_streams(&runways, Some("12".into()), turn_t + sec(10));
        assert_eq!((arr.as_deref(), dep.as_deref()), (Some("12"), None));

        *WAKE.lock().unwrap() = None;
        *WAKE_PREV.lock().unwrap() = None;
        *DEPARTURE_STREAM.lock().unwrap() = None;
        *ARRIVAL_STREAM.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    /// Issue #140, the recorded party line: LEMG (sortie
    /// 1790584715-full_auto). Our circuit on downwind for 31 (Inbound) heard
    /// "Vueling eight one seven six, runway one three, cleared for takeoff"
    /// (t 1790585447.7); cleared to land 31 (t 1790585501.7), the tower
    /// ordered the go-around for traffic on the runway (t 1790585525.5) and
    /// then narrated "Easy two seven one six, runway one three, cleared for
    /// takeoff" (t 1790585551.4). One strip, one direction: nobody is
    /// cleared onto or off the reciprocal of our runway, and nobody onto
    /// either end of the strip we are cleared to land on. The other strip's
    /// traffic and our own direction's are still narrated.
    #[test]
    fn the_party_line_clears_nobody_against_our_runway() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let runways = lemg_runways();
        *ACTIVE_RUNWAYS.lock().unwrap() = runways.clone();
        *GHOSTS.lock().unwrap() = None;
        let lerp = |p: (f64, f64), q: (f64, f64), f: f64| {
            (p.0 + (q.0 - p.0) * f, p.1 + (q.1 - p.1) * f)
        };
        let (t13, t31) = ((runways[1].lat_a, runways[1].lon_a), (runways[1].lat_b, runways[1].lon_b));
        let (t12, t30) = ((runways[0].lat_a, runways[0].lon_a), (runways[0].lat_b, runways[0].lon_b));
        let field = lerp(t13, t31, 0.5);
        let felev = 52.0;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        let set_clearance = |state: tower::State, holds: bool| {
            *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
                state,
                callsign: "November eight one six November November".into(),
                runway_ident: "31".into(),
                pending_readback: None,
                holds_runway: holds,
            });
        };
        let ghost = |cs: &str, pos: (f64, f64), ele: f64, ground: bool, vs: f64| TcasTarget {
            callsign: cs.into(),
            lat: pos.0,
            lon: pos.1,
            ele_ft: ele,
            on_ground: ground,
            vs_fpm: vs,
        };
        // Airborne: the ground-only blocked strip is None, as in the sortie.
        let tick = |t: TcasTarget, at: Instant| {
            chatter_tick(&[t], "Malaga", field.0, field.1, felev, None, at)
        };
        // A departure: rolling at 70 % of the strip, airborne at 78 %.
        let depart = |cs: &str, from: (f64, f64), to: (f64, f64), at: Instant| {
            assert!(tick(ghost(cs, lerp(from, to, 0.70), felev, true, 0.0), at).is_none());
            tick(ghost(cs, lerp(from, to, 0.78), felev + 120.0, false, 1500.0), at + sec(3))
        };

        // Downwind for 31 (Inbound): Vueling off 13 is not cleared.
        set_clearance(tower::State::Inbound, false);
        assert!(
            depart("VLG8176", t13, t31, t0).is_none(),
            "no takeoff narrated off the reciprocal of our runway"
        );
        // Cleared to land 31: Easy off 13 is not cleared either.
        set_clearance(tower::State::ClearedToLand, true);
        assert!(depart("EZY2716", t13, t31, t0 + sec(10)).is_none());
        // After the go-around order (the FSM back to Inbound): still not.
        set_clearance(tower::State::Inbound, false);
        assert!(depart("EZS2716", t13, t31, t0 + sec(20)).is_none());
        // Cleared to land 31, a departure off 31 ahead of us: the whole
        // strip is ours, not narrated.
        set_clearance(tower::State::ClearedToLand, true);
        assert!(depart("IBE3001", t31, t13, t0 + sec(30)).is_none());
        // Inbound for 31, a departure off 31: our direction, narrated.
        set_clearance(tower::State::Inbound, false);
        let ex = depart("IBE3002", t31, t13, t0 + sec(40)).expect("our direction is narrated");
        assert!(ex.lines[0].text.contains("runway three one, cleared for takeoff"), "{}", ex.lines[0].text);
        // The other strip is not ours: a departure off 12 is narrated.
        let ex = depart("IBE3003", t12, t30, t0 + sec(50)).expect("the other strip is narrated");
        assert!(ex.lines[0].text.contains("runway one two, cleared for takeoff"), "{}", ex.lines[0].text);

        // Landings. Cleared to land 31: Ryanair on final 12 is narrated,
        // a ghost on final 31 behind us is not given the same clearance.
        set_clearance(tower::State::ClearedToLand, true);
        let final12 = |f: f64| lerp(t12, t30, f);
        assert!(tick(ghost("RYR8168", final12(-0.8), 900.0, false, -700.0), t0 + sec(60)).is_none());
        let ex = tick(ghost("RYR8168", final12(-0.7), 750.0, false, -700.0), t0 + sec(63))
            .expect("the other strip's landing is narrated");
        assert!(ex.lines[1].text.contains("runway one two, cleared to land"), "{}", ex.lines[1].text);
        let final31 = |f: f64| lerp(t31, t13, f);
        assert!(tick(ghost("RYR4458", final31(-0.6), 900.0, false, -700.0), t0 + sec(70)).is_none());
        assert!(
            tick(ghost("RYR4458", final31(-0.5), 750.0, false, -700.0), t0 + sec(73)).is_none(),
            "no second landing clearance on the strip we are cleared to land on"
        );
        // Our landing done and reported off: the claim is gone and the
        // ghost, still descending, gets its clearance.
        set_clearance(tower::State::ClearedToLand, false);
        let ex = tick(ghost("RYR4458", final31(-0.4), 600.0, false, -700.0), t0 + sec(76))
            .expect("cleared once the strip is free");
        assert!(ex.lines[1].text.contains("runway three one, cleared to land"), "{}", ex.lines[1].text);

        *GHOSTS.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    /// Issue #129, the recorded case: LEMG (sortie 1790580676-approach,
    /// 2026-09-28), the 737 on a 10 nm final for 13 in 350 at 3 kt, said
    /// "ten mile final runway one three" and at t 1790580681.3 was told
    /// "negative, runway three one ... cleared to land": 13's -2.3 kt
    /// trailed 31's +2.3 kt by 4.7, over the 3 kt swap margin, in a wind the
    /// pick itself calls calm. LEMG 12/30 and 13/31 from apt.dat.
    #[test]
    fn the_lemg_final_for_one_three_keeps_its_runway_in_a_calm() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CLEARANCE.lock().unwrap() = None;
        let runways = vec![
            nav::Runway {
                width_m: 45.11,
                surface: 1,
                ident_a: "12".into(),
                ident_b: "30".into(),
                lat_a: 36.6910753,
                lon_a: -4.5078270,
                lat_b: 36.6795648,
                lon_b: -4.4805125,
            },
            nav::Runway {
                width_m: 45.11,
                surface: 1,
                ident_a: "13".into(),
                ident_b: "31".into(),
                lat_a: 36.6845573,
                lon_a: -4.5126253,
                lat_b: 36.6654027,
                lon_b: -4.4858140,
            },
        ];
        let wx = Wx {
            wind_from_deg: 350.0,
            wind_kts: 3.0,
        };
        // The recorded numbers: 13 on a tailwind, 31 the into-wind pick,
        // 4.7 kt apart.
        let hw13 = nav::headwind_for(&runways, "13", 350.0, 3.0).unwrap();
        let (best, _, best_hw) = nav::best_runway_for_wind(&runways, 350.0, 3.0).unwrap();
        assert!(best == "31" || best == "30", "{best}");
        assert!(
            best_hw - hw13 > RUNWAY_SWAP_MARGIN_KTS,
            "{best_hw} vs {hw13}"
        );

        let mut st = lxgb();
        st.apt.name = "Malaga".into();
        st.apt.runways = runways.clone();
        let words = "november eight one six november november, ten mile final runway one three";
        let intent = tower::parse_intent(words);
        assert_eq!(intent, tower::Intent::ReportFinal);
        st.note_stated_runway(words, intent);
        let ctx = scripted_ctx(&mut st, wx);
        assert_eq!(st.runway_in_use.as_deref(), Some("13"));
        let r = st.route_keyed(words, Service::Tower, &ctx, wx);
        assert!(!r.contains("negative"), "{r}");
        assert!(r.contains("runway one three"), "{r}");
        assert!(r.contains("cleared to land"), "{r}");

        // A tailwind within 5 kt stands in a real wind too (350 at 6: 13
        // carries 4.6 kt of tailwind on the same crosswind as 31)...
        let mut st = lxgb();
        st.apt.runways = runways.clone();
        st.note_stated_runway("final runway one three", tower::Intent::ReportFinal);
        assert_eq!(st.pick_runway(350.0, 6.0), "13");
        // ...but not past it (350 at 10: 7.7 kt of tailwind).
        let mut st = lxgb();
        st.apt.runways = runways.clone();
        st.note_stated_runway("final runway one three", tower::Intent::ReportFinal);
        assert_ne!(st.pick_runway(350.0, 10.0), "13");
        // Sortie 1790583663-approach: "10 mile final runway one three" into
        // 060 at 6 earned "negative, runway one two". The wind does not argue
        // (13's headwind 2.0 against 12's 3.3, inside the margin), so 13
        // stands on the wind alone; the only way to that "negative" is an
        // arrival stream the traffic picture put on another end.
        let hw13 = nav::headwind_for(&runways, "13", 60.0, 6.0).unwrap();
        let (best, _, best_hw) = nav::best_runway_for_wind(&runways, 60.0, 6.0).unwrap();
        assert_eq!(best, "12");
        assert!(best_hw - hw13 <= RUNWAY_SWAP_MARGIN_KTS, "{best_hw} vs {hw13}");
        let mut st = lxgb();
        st.apt.runways = runways.clone();
        st.note_stated_runway("final runway one three", tower::Intent::ReportFinal);
        assert_eq!(st.pick_runway_with(60.0, 6.0, None, None), "13");
        let mut st = lxgb();
        st.apt.runways = runways.clone();
        st.note_stated_runway("final runway one three", tower::Intent::ReportFinal);
        assert_eq!(st.pick_runway_with(60.0, 6.0, Some("12"), None), "12");
        // A crossing strip with a tailwind in limits but a far worse
        // crosswind does not stand (LXGB-like 09/27 plus a synthetic 18/36,
        // wind 360 at 12: 09 has no tailwind but a 12 kt crosswind).
        let mut st = lxgb();
        st.apt.runways = vec![
            nav::Runway {
                width_m: 45.0,
                surface: 1,
                ident_a: "09".into(),
                ident_b: "27".into(),
                lat_a: 36.0,
                lon_a: -5.02,
                lat_b: 36.0,
                lon_b: -4.98,
            },
            nav::Runway {
                width_m: 45.0,
                surface: 1,
                ident_a: "18".into(),
                ident_b: "36".into(),
                lat_a: 36.02,
                lon_a: -5.0,
                lat_b: 35.98,
                lon_b: -5.0,
            },
        ];
        st.note_stated_runway("final runway zero nine", tower::Intent::ReportFinal);
        assert_eq!(st.pick_runway(0.0, 12.0), "36");
        *CLEARANCE.lock().unwrap() = None;
    }

    /// Issue #134, the recorded case: LEMG (sortie 1790584715-full_auto,
    /// 2026-09-28), the runner's runway start put the 737 on 13's threshold
    /// (36.684289 N 4.512250 W, 131.4 true, fit "13"), wind 342.6 at 2.9 kt,
    /// arrivals landing 12 on the party line. The clearance named 31 and full
    /// auto taxied 400 s round the field to line up there. A runway start
    /// asks for the strip it is on and is given it while the wind does not
    /// argue: the calm-wind stream on another end does not move it.
    #[test]
    fn a_runway_start_is_given_the_runway_it_is_lined_up_on() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CLEARANCE.lock().unwrap() = None;
        let runways = vec![
            nav::Runway {
                width_m: 45.11,
                surface: 1,
                ident_a: "12".into(),
                ident_b: "30".into(),
                lat_a: 36.6910753,
                lon_a: -4.5078270,
                lat_b: 36.6795648,
                lon_b: -4.4805125,
            },
            nav::Runway {
                width_m: 45.11,
                surface: 1,
                ident_a: "13".into(),
                ident_b: "31".into(),
                lat_a: 36.6845573,
                lon_a: -4.5126253,
                lat_b: 36.6654027,
                lon_b: -4.4858140,
            },
        ];
        let (lat, lon) = (36.684288506540085, -4.51224958630406);
        let words = "november eight one six november november, request clearance, runway one three";
        let intent = tower::parse_intent(words);
        assert_eq!(intent, tower::Intent::RequestClearance);
        let station = || {
            let mut st = lxgb();
            st.apt.name = "Malaga".into();
            st.apt.runways = runways.clone();
            st
        };

        // Lined up on 13, the stream on 12: 13.
        let mut st = station();
        st.note_stated_runway(words, intent.clone());
        st.note_caller_on_strip(true, lat, lon);
        assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "13");
        assert_eq!(st.runway_in_use.as_deref(), Some("13"));
        // ...and it holds for the takeoff call that follows (no stated
        // runway, nothing committed, calm).
        assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "13");
        // The same words from an aircraft NOT on the strip: the stream's end,
        // as before.
        let mut st = station();
        st.note_stated_runway(words, intent.clone());
        st.note_caller_on_strip(true, 36.6950, -4.4900);
        assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "12");
        // Airborne is never a runway start.
        let mut st = station();
        st.note_stated_runway(words, intent.clone());
        st.note_caller_on_strip(false, lat, lon);
        assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "12");
        // A wind that really argues still wins: 310 at 12 is 12 kt of tailwind
        // on 13.
        let mut st = station();
        st.note_stated_runway(words, intent.clone());
        st.note_caller_on_strip(true, lat, lon);
        assert_ne!(st.pick_runway_with(310.0, 12.0, None, None), "13");
        // A flight the tower is sequencing keeps its runway.
        let mut st = station();
        st.runway_in_use = Some("31".into());
        st.twr.state = tower::State::TaxiCleared;
        st.note_stated_runway(words, intent.clone());
        st.note_caller_on_strip(true, lat, lon);
        assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "31");

        // The clearance the crew reads names 13.
        let mut st = station();
        st.note_stated_runway(words, intent);
        st.note_caller_on_strip(true, lat, lon);
        assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "13");
        let wx = Wx {
            wind_from_deg: 342.6,
            wind_kts: 2.9,
        };
        let ctx = scripted_ctx(&mut st, wx);
        let r = st.route_keyed(words, Service::Tower, &ctx, wx);
        assert!(r.contains("cleared to leave"), "{r}");
        assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("13"));
        *CLEARANCE.lock().unwrap() = None;
    }

    /// Issues #134 and #140 together (LEMG 2026-09-28, calm): a runway start
    /// keeps the runway it is lined up on unless that runway points INTO the
    /// live traffic. Lined up on 13 with ghosts departing 13 and landing 12:
    /// 13. Lined up on 31 with ghosts departing 13 (or landing 13): refused,
    /// the stream's end. On 31 with the departures on 31: 31.
    #[test]
    fn a_runway_start_keeps_its_strip_unless_it_faces_the_stream() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CLEARANCE.lock().unwrap() = None;
        let runways = lemg_runways();
        let (t13, t31) = (
            (runways[1].lat_a, runways[1].lon_a),
            (runways[1].lat_b, runways[1].lon_b),
        );
        let along = |f: f64| (t13.0 + (t31.0 - t13.0) * f, t13.1 + (t31.1 - t13.1) * f);
        let start = |words: &str, pos: (f64, f64)| {
            let mut st = lxgb();
            st.apt.name = "Malaga".into();
            st.apt.runways = runways.clone();
            st.note_stated_runway(words, tower::parse_intent(words));
            st.note_caller_on_strip(true, pos.0, pos.1);
            st
        };
        let on13 = "november eight one six november november, request clearance, runway one three";
        let on31 = "november eight one six november november, request clearance, runway three one";
        // On 13's threshold, the world departing 13 and landing 12: 13.
        let mut st = start(on13, along(0.01));
        assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), Some("13")), "13");
        // On 31's threshold facing the 13 departures: refused, 13.
        let mut st = start(on31, along(0.99));
        assert_eq!(st.pick_runway_with(342.6, 2.9, None, Some("13")), "13");
        // Facing an arrival stream on 13 too.
        let mut st = start(on31, along(0.99));
        assert_eq!(st.pick_runway_with(342.6, 2.9, Some("13"), None), "13");
        // With the world on 31 the start on 31 stands.
        let mut st = start(on31, along(0.99));
        assert_eq!(st.pick_runway_with(342.6, 2.9, None, Some("31")), "31");
        // No live traffic: #134's rule alone, the start stands.
        let mut st = start(on31, along(0.99));
        assert_eq!(st.pick_runway_with(342.6, 2.9, None, None), "31");
    }

    /// Two parallels have the SAME heading to within rounding error, so which
    /// one "wins" the wind was decided by noise — and nothing in the
    /// phraseology gives it away, both are "two seven". A silent flip from the
    /// 27L taxi clearance to 27R pointed the takeoff hold at Heathrow's
    /// ARRIVAL stream, which never has a 15 s gap: held at November one for
    /// the full eight minutes and handed back, "traffic on short final" for a
    /// departure runway that was empty (2026-07-27).
    #[test]
    fn parallel_runways_never_trade_places_under_a_departure() {
        let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
        // Heathrow's real pair: both 09/27, headings a hundredth of a degree
        // apart. 09L/27R is first in apt.dat, so it wins any tie.
        st.apt.runways = vec![
            nav::Runway {
                width_m: 50.0,
                surface: 0,
                ident_a: "09L".into(),
                ident_b: "27R".into(),
                lat_a: 51.4774781,
                lon_a: -0.4894423,
                lat_b: 51.4776753,
                lon_b: -0.4332600,
            },
            nav::Runway {
                width_m: 50.0,
                surface: 0,
                ident_a: "09R".into(),
                ident_b: "27L".into(),
                lat_a: 51.4647701,
                lon_a: -0.4867489,
                lat_b: 51.4649460,
                lon_b: -0.4340638,
            },
        ];
        // The tower is working 27L — the taxi clearance named it.
        st.runway_in_use = Some("27L".into());
        st.twr.state = tower::State::TaxiCleared;
        // Every subsequent transmission, in any wind, keeps 27L: the departure
        // is committed and a controller does not move it to the other
        // parallel mid-sequence.
        for (from, kts) in [(250.0, 12.0), (270.0, 25.0), (269.0, 8.0), (271.0, 30.0)] {
            assert_eq!(st.pick_runway(from, kts), "27L", "wind {from}/{kts}");
        }
        st.twr.state = tower::State::LinedUp;
        assert_eq!(st.pick_runway(280.0, 18.0), "27L");
        st.twr.state = tower::State::TakeoffCleared;
        assert_eq!(st.pick_runway(260.0, 18.0), "27L");

        // Uncommitted, the hysteresis still holds the pick against a parallel
        // that is "better" by rounding error...
        st.twr.state = tower::State::Parked;
        for (from, kts) in [(250.0, 12.0), (270.0, 20.0), (271.0, 9.0)] {
            assert_eq!(st.pick_runway(from, kts), "27L", "wind {from}/{kts}");
        }
        // ...but a genuine reversal still turns the field around, as it must,
        // onto the main (longest) strip facing the new way: 09L/27R is
        // 3,902 m against 09R/27L's 3,660 m (`nav::best_runway_for_wind`).
        assert_eq!(st.pick_runway(90.0, 20.0), "09L");
    }

    /// Issue #155, sortie 1790601822-full_auto (EGBB 15, 2026-09-28):
    /// "traffic, 5 o'clock, 1 miles, 900 feet above" (+3194.7) and "4
    /// o'clock, 1 miles, 300 feet above" (+3240.1). One mile is "a mile".
    #[test]
    fn a_traffic_advisory_says_a_mile() {
        let f = |nm: f64, clock: i32, ft: f64| TrafficFactor { nm, clock, ft };
        assert_eq!(
            advisory_line(&f(1.2, 5, 900.0)),
            "traffic, 5 o'clock, a mile, 900 feet above."
        );
        assert!(advisory_line(&f(1.49, 4, 300.0)).contains(", a mile, "));
        assert!(advisory_line(&f(0.4, 12, 0.0)).contains("under a mile"));
        assert!(advisory_line(&f(1.5, 12, 0.0)).contains(", 2 miles, "));
        assert!(advisory_line(&f(6.0, 12, 0.0)).contains(", 6 miles, "));
        for nm in [0.2, 0.99, 1.0, 1.3, 1.49, 1.5, 2.4, 5.9] {
            assert!(!advisory_line(&f(nm, 3, 0.0)).contains(" 1 miles"), "{nm}");
        }
    }

    #[test]
    fn near_miss_alert_names_the_way_out() {
        let f = |nm: f64, clock: i32, ft: f64| TrafficFactor { nm, clock, ft };
        // The line: half a mile, 500 ft. Outside it, only the advisory tier.
        assert!(is_near_miss(&f(0.4, 3, 0.0)));
        assert!(!is_near_miss(&f(0.6, 3, 0.0)), "outside half a mile");
        assert!(
            !is_near_miss(&f(0.4, 3, 600.0)),
            "vertical separation holds"
        );
        // Vertical escape when the intruder is clearly above or below.
        assert!(near_miss_line(&f(0.4, 12, 300.0), false).contains("advise you descend"));
        assert!(near_miss_line(&f(0.4, 12, -300.0), false).contains("advise you climb"));
        // Co-altitude: turn AWAY from the traffic's side...
        assert!(near_miss_line(&f(0.4, 2, 0.0), false).contains("turn left"));
        assert!(near_miss_line(&f(0.4, 10, 0.0), false).contains("turn right"));
        // ...right for head-on (the right-of-way rule)...
        assert!(near_miss_line(&f(0.4, 12, 0.0), false).contains("turn right"));
        // ...and vertical for one dead astern, where a turn hides it.
        assert!(near_miss_line(&f(0.4, 6, 0.0), false).contains("climb"));
        // Controller shape vs crew shape: same geometry, right register.
        let atc = near_miss_line(&f(0.4, 2, 0.0), false);
        assert!(atc.starts_with("traffic alert"), "{atc}");
        assert!(atc.ends_with("immediately."), "{atc}");
        let crew = near_miss_line(&f(0.4, 2, 0.0), true);
        assert!(crew.starts_with("Traffic!"), "{crew}");
        assert!(crew.contains("now."), "{crew}");
    }

    /// The adversarial corpus (`bench/atc_corpus.yaml`): real whisper output
    /// mined from the flight logs plus hand-written paraphrases, homophone
    /// mangles, and crew-talk near-misses. Every case judges up to three
    /// parsers — intent, the unkeyed radio-call gate, the readback detector —
    /// and failures collect so one run reports every drift at once.
    #[test]
    fn transcript_corpus_parses_safely() {
        #[derive(serde::Deserialize)]
        struct Case {
            text: String,
            intent: String,
            #[serde(default)]
            radio: Option<bool>,
            #[serde(default)]
            readback: Option<bool>,
            /// The destination an IFR request must carry, as parsed
            /// (`request_ifr_clearance` only; absent = none named).
            #[serde(default)]
            dest: Option<String>,
            /// Our spoken callsign for the case: `radio` is then judged as
            /// the station's unkeyed gate judges it, the radio words OR the
            /// callsign named (`callsign::addressed`).
            #[serde(default)]
            callsign: Option<String>,
        }
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/bench/atc_corpus.yaml");
        let cases: Vec<Case> =
            serde_yaml::from_str(&std::fs::read_to_string(path).expect("corpus file"))
                .expect("corpus yaml");
        assert!(cases.len() >= 100, "corpus shrank to {} cases", cases.len());
        let mut fails = Vec::new();
        for c in &cases {
            let t = c.text.to_lowercase();
            let Some(mut want) = tower::intent_from_label(&c.intent) else {
                panic!("{:?}: unknown intent label {:?}", c.text, c.intent);
            };
            if let tower::Intent::RequestIfrClearance { dest } = &mut want {
                *dest = c.dest.clone();
            }
            let got = tower::parse_intent(&t);
            if got != want {
                fails.push(format!("{:?}: parsed {got:?}, wanted {want:?}", c.text));
            }
            if let Some(radio) = c.radio {
                let ours = c
                    .callsign
                    .as_deref()
                    .is_some_and(|cs| crate::callsign::addressed(&t, cs));
                if (is_radio_call(&t, "gibraltar") || ours) != radio {
                    fails.push(format!("{:?}: is_radio_call should be {radio}", c.text));
                }
            }
            if let Some(rb) = c.readback {
                if tower::is_readback(&t) != rb {
                    fails.push(format!("{:?}: is_readback should be {rb}", c.text));
                }
            }
        }
        assert!(
            fails.is_empty(),
            "{} of {} corpus cases drifted:\n{}",
            fails.len(),
            cases.len(),
            fails.join("\n")
        );
    }

    fn calm() -> Wx {
        Wx {
            wind_from_deg: 0.0,
            wind_kts: 0.0,
        }
    }

    /// The live context exactly as `build_ctx` would assemble it, minus the
    /// simulator: scripted wind, fixed QNH and tail number.
    fn scripted_ctx(st: &mut Station, wx: Wx) -> tower::Ctx {
        st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            false, // airborne default; only the mayday answer reads it
            false, // not an arrival taxi-in
            None,  // final clear; traffic-aware cases build their own ctx
            false, // runway unoccupied
            None,  // no wake clock
        )
    }

    /// EGBB 33, 2026-09-27 (sortie 1790534902): Birmingham has no departure
    /// frequency, so the tower handed the climbing flight to APPROACH. The
    /// crew read the handoff back on the tower (silence, correct), retuned,
    /// and checked in: "passing three thousand one hundred climbing one five
    /// thousand" was heard by approach as a plain readback and answered with
    /// nothing, the tower still holding the flight cleared for takeoff. The
    /// position a departure is handed to takes its check-in, and the tower's
    /// book on the flight closes there.
    #[test]
    fn approach_takes_the_departure_check_in_when_it_is_the_handoff() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.departure_khz = None;
        let ctx = scripted_ctx(&mut st, wx);
        // An arrival's readback to approach, no departure handed over: still
        // silence, never "radar contact".
        let r = st.route_keyed("descending two thousand", Service::Approach, &ctx, wx);
        assert_eq!(r, "", "{r}");

        st.twr.state = tower::State::TaxiCleared;
        let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared for takeoff"), "{r}");
        let _ = crate::party_line::take_departure_handoff();
        let line = handoff_tick(900.0, false).expect("handoff due");
        assert!(line.contains("Gibraltar Approach on one two two decimal eight"), "{line}");
        let _ = crate::party_line::take_departure_handoff();

        // The readback, on the TOWER before the retune: a frequency readback
        // earns silence and moves nothing.
        let r = st.route_keyed(
            "whiskey alpha tango four two three six, gibraltar approach on one two two decimal eight",
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(r, "", "{r}");
        assert_eq!(st.twr.state, tower::State::TakeoffCleared);

        // The check-in on approach, as the crew said it.
        let checkin = "birmingham approach, whiskey alpha tango four two three six, passing three thousand one hundred climbing one five thousand";
        assert_eq!(tower::parse_intent(checkin), tower::Intent::Unknown);
        assert!(tower::is_readback(checkin));
        let r = st.route_keyed(checkin, Service::Approach, &ctx, wx);
        assert_eq!(
            r,
            "November one seven two Sierra Papa, Gibraltar Approach, radar contact, climb one five thousand."
        );
        assert_eq!(st.twr.state, tower::State::Parked, "the tower's book is closed");
        let snap = crate::party_line::clearance_state();
        assert_eq!(snap, Some(tower::State::Parked));
        assert!(!st.twr.holds_runway());
        // Later readbacks on approach stay quiet.
        let r = st.route_keyed("maintaining one five thousand", Service::Approach, &ctx, wx);
        assert_eq!(r, "", "{r}");
        // The same flight coming back is an arrival from Parked.
        let r = st.route_keyed("november one seven two, final", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared to land"), "{r}");
        *CLEARANCE.lock().unwrap() = None;
    }

    /// EGKK 26L, 2026-09-27 (sortie 1790534902, +2735 s): the crew's landing
    /// readback parsed as a landing REQUEST in ClearedToLand. The route kept
    /// quiet only because the word "cleared" silenced it; the intent now says
    /// what it is, a readback, and the unkeyed path (where no such guard
    /// stood) can no longer walk the landing back to "join final".
    #[test]
    fn a_landing_readback_is_a_readback_not_a_landing_request() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        let readback = "whiskey alpha tango four two three six, cleared to land runway two seven";
        assert_eq!(tower::parse_intent(readback), tower::Intent::Unknown);
        assert!(tower::is_readback(readback));

        // Keyed: the gate takes the echo, the frequency stays quiet, the
        // clearance stands and nothing is owed.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let ctx = scripted_ctx(&mut st, wx);
        let r = st.route_keyed("november one seven two, final", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared to land"), "{r}");
        assert!(st.twr.pending().is_some(), "the landing readback is owed");
        let r = st.route_keyed(readback, Service::Tower, &ctx, wx);
        assert_eq!(r, "", "{r}");
        assert_eq!(st.twr.state, tower::State::ClearedToLand);
        assert!(st.twr.pending().is_none(), "paid");
        // Said again with nothing owed: still silence, still cleared.
        let r = st.route_keyed(readback, Service::Tower, &ctx, wx);
        assert_eq!(r, "", "{r}");
        assert_eq!(st.twr.state, tower::State::ClearedToLand);

        // Unkeyed, addressed by our callsign: not the tower's business as a
        // request any more; it falls through and the landing stands.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.callsign = "Whiskey Alpha Tango four two three six".into();
        st.route_keyed("whiskey alpha tango four two three six, final", Service::Tower, &ctx, wx);
        assert_eq!(st.twr.state, tower::State::ClearedToLand);
        assert_eq!(st.route_unkeyed(readback, Service::Tower, &ctx, wx), None);
        assert_eq!(st.twr.state, tower::State::ClearedToLand, "never walked back to Inbound");
        *CLEARANCE.lock().unwrap() = None;
    }

    /// Whole sorties driven through the pure routing layer — the dialogue
    /// sequences that live between the tower FSM (unit-tested above it) and
    /// the mic (untestable): out-of-order calls, questions on a keyed radio,
    /// readbacks, wrong-service redirects, the departure check-in. Every
    /// scene asserts the reply AND that the clearance sequence only moves
    /// when it should.
    #[test]
    fn sortie_dialogues_battle_test_the_routing() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };

        // Scene 1: the departure happy path, keyed on the tower frequency.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let ctx = scripted_ctx(&mut st, wx);
        let r = st.route_keyed("gibraltar tower, radio check", Service::Tower, &ctx, wx);
        assert!(r.contains("reading you five"), "{r}");
        assert_eq!(
            st.twr.state,
            tower::State::Parked,
            "a radio check moves nothing"
        );

        // The zone clearance suggests its readback (cap, QNH, squawk) and
        // enforces nothing; the crew's echo pays it and earns silence.
        let r = st.route_keyed("request clearance", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared to leave the Gibraltar zone"), "{r}");
        let owed = st.twr.suggested_readback(&ctx.callsign).expect("a suggested readback");
        assert!(owed.starts_with("Not above two thousand feet, QNH one zero one seven, squawk"), "{owed}");
        assert!(owed.ends_with(&format!("{}.", ctx.callsign)), "{owed}");
        let r = st.route_keyed(&owed.to_lowercase(), Service::Tower, &ctx, wx);
        assert_eq!(r, "", "a clearance readback keeps the frequency quiet: {r}");
        assert!(st.twr.suggested_readback(&ctx.callsign).is_none(), "paid");

        let r = st.route_keyed("request clearance for taxi", Service::Tower, &ctx, wx);
        assert!(
            r.contains("taxi to holding point Alpha runway two seven"),
            "{r}"
        );
        assert_eq!(st.twr.state, tower::State::TaxiCleared);

        // A readback earns silence; garble earns "say again"; neither moves
        // the sequence.
        let r = st.route_keyed("roger, holding point alpha", Service::Tower, &ctx, wx);
        assert_eq!(r, "", "a readback keeps the frequency quiet: {r}");
        let r = st.route_keyed(
            "uhh gibraltar something something",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("say again"), "{r}");
        assert_eq!(st.twr.state, tower::State::TaxiCleared);

        // A question keyed by mistake gets "say again", never a clearance —
        // the 2026-07-08 regression, now at the routing layer.
        let r = st.route_keyed(
            "what runway shall i take off from",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("say again"), "{r}");
        let r = st.route_keyed("takeoff checklist", Service::Tower, &ctx, wx);
        assert!(r.contains("say again"), "{r}");
        assert_eq!(
            st.twr.state,
            tower::State::TaxiCleared,
            "questions moved the sequence"
        );

        let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared for takeoff"), "{r}");
        assert_eq!(st.twr.state, tower::State::TakeoffCleared);

        // The takeoff clearance armed the handoff; airborne through 700 ft it
        // fires exactly once, to the departure frequency.
        assert!(handoff_tick(300.0, false).is_none(), "still low");
        // Drain whatever another scene left on the party line: the handoff
        // is published for the crew the moment the line goes out, below.
        let _ = crate::party_line::take_departure_handoff();
        let line = handoff_tick(900.0, false).expect("handoff due");
        assert!(
            line.contains("Departure on one three zero decimal four"),
            "{line}"
        );
        assert!(handoff_tick(1500.0, false).is_none(), "spoken once");
        // The crew gets the instruction as data: whom to call, on what.
        let (to, khz) =
            crate::party_line::take_departure_handoff().expect("published for the crew");
        assert_eq!((to.as_str(), khz), ("Gibraltar Departure", 130_400));
        assert!(
            crate::party_line::take_departure_handoff().is_none(),
            "consumed once"
        );

        // Scene 2: the departure controller. First readback is the check-in
        // and gets radar contact; later readbacks earn silence.
        let r = st.route_keyed(
            "passing one thousand five hundred climbing",
            Service::Departure,
            &ctx,
            wx,
        );
        assert!(r.contains("Departure, radar contact"), "{r}");
        let r = st.route_keyed("maintaining two thousand", Service::Departure, &ctx, wx);
        assert_eq!(r, "", "post-check-in readbacks stay quiet: {r}");

        // Runway-shaped business on the wrong frequency redirects to the
        // tower and must not advance the tower's sequence.
        let was = st.twr.state;
        let r = st.route_keyed("request taxi", Service::Approach, &ctx, wx);
        assert!(
            r.contains("contact Gibraltar Tower on one three one decimal two"),
            "{r}"
        );
        let r = st.route_keyed("ready for departure", Service::Departure, &ctx, wx);
        assert!(r.contains("contact Gibraltar Tower"), "{r}");
        assert_eq!(st.twr.state, was, "a redirect must not move the sequence");

        // Coming back in: departure sends arrivals to approach when the field
        // has one, and the touch-and-go option survives the redirect.
        let r = st.route_keyed("request touch and go", Service::Departure, &ctx, wx);
        assert!(
            r.contains("contact Gibraltar Approach on one two two decimal eight"),
            "{r}"
        );
        let clear = st.route_keyed("november one seven two, final", Service::Tower, &ctx, wx);
        assert!(clear.contains("cleared touch and go"), "{clear}");

        // Scene 3: the arrival, keyed on the tower — inbound, final, then off
        // the runway. Calm wind must hold the westerly runway pick.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let ctx = scripted_ctx(&mut st, wx);
        let r = st.route_keyed(
            "gibraltar tower, inbound for landing",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("join final runway two seven"), "{r}");
        let calm_ctx = scripted_ctx(&mut st, calm());
        assert!(
            calm_ctx.runway.contains("two seven"),
            "calm flipped the runway"
        );
        let r = st.route_keyed("final", Service::Tower, &calm_ctx, calm());
        assert!(r.contains("cleared to land"), "{r}");
        // Reading the clearance back must earn silence and hold the state —
        // it used to re-parse as a landing request and walk the sequence
        // backwards to Inbound.
        let r = st.route_keyed(
            "cleared to land runway two seven, november one seven two sierra papa",
            Service::Tower,
            &calm_ctx,
            calm(),
        );
        assert_eq!(r, "", "a clearance echo keeps the frequency quiet: {r}");
        assert_eq!(
            st.twr.state,
            tower::State::ClearedToLand,
            "echo moved the sequence"
        );
        let r = st.route_keyed("request taxi", Service::Tower, &calm_ctx, calm());
        assert!(r.contains("taxi to parking"), "{r}");
        assert_eq!(st.twr.state, tower::State::Parked);
        let r = st.route_keyed("i've just landed", Service::Tower, &calm_ctx, calm());
        assert!(r.contains("taxi to parking"), "{r}");

        // Scene 4: content routing (no PTT). Crew talk falls through even
        // when it contains clearance words; radio-shaped calls route.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let ctx = scripted_ctx(&mut st, wx);
        for crew in [
            "anything else we need to do before takeoff",
            "land the aircraft",
            "flaps set for takeoff",
            "after landing checklist",
            "gibraltar is somewhere ahead",
        ] {
            assert!(
                st.route_unkeyed(crew, Service::Tower, &ctx, wx).is_none(),
                "crew talk routed to ATC: {crew}"
            );
        }
        // "hello tower" IS addressed to the tower — since 2026-07-10 a hail
        // on frequency earns the invitation, and moves nothing.
        let r = st
            .route_unkeyed("hello tower", Service::Tower, &ctx, wx)
            .expect("a hail addressed to the tower is answered");
        assert!(r.contains("pass your message"), "{r}");
        assert_eq!(
            st.twr.state,
            tower::State::Parked,
            "crew talk moved the sequence"
        );
        let r = st
            .route_unkeyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx)
            .expect("a proper call routes");
        assert!(r.contains("holding point"), "{r}");

        // Any leftover handoff from this sortie must not leak into the next
        // test holding the lock.
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    /// An airline flight is worked by its airline callsign: the tower
    /// answers "Speedbird three zero six", and a call that opens with it is
    /// addressed even without a station name or a "request" (a 737 on an
    /// OFP as BAW306 was called by its registration, 2026-09-27).
    #[test]
    fn an_airline_callsign_is_addressed_and_answered() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.callsign = crate::callsign::resolve(
            &crate::callsign::Inputs {
                override_id: None,
                plan: Some(crate::callsign::PlanCallsign {
                    id: "BAW306".into(),
                    plan_type: "B738".into(),
                    also_types: Vec::new(),
                }),
            },
            Some("G-EUPT"),
            Some("B738"),
            Some(79_000.0),
        )
        .expect("a callsign")
        .spoken;
        assert_eq!(ctx.callsign, "Speedbird three zero six");

        // "Ready to taxi" alone is crew talk; with our callsign in front it
        // is a call to the tower.
        assert!(!is_radio_call("brakes set, ready to taxi", "gibraltar"));
        assert!(
            st.route_unkeyed("brakes set, ready to taxi", Service::Tower, &ctx, wx)
                .is_none(),
            "crew talk stays crew talk"
        );
        let r = st
            .route_unkeyed("Speedbird 306, ready to taxi", Service::Tower, &ctx, wx)
            .expect("our callsign addresses the tower");
        assert!(r.starts_with("Speedbird three zero six"), "{r}");
        assert!(r.contains("holding point"), "{r}");
        assert_eq!(st.twr.state, tower::State::TaxiCleared);

        // Keyed, the whisper spelling of the same callsign.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let r = st.route_keyed(
            "speed bird three zero six, request taxi",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.starts_with("Speedbird three zero six"), "{r}");
        assert!(r.contains("holding point"), "{r}");
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    /// A virtual airline the tables do not know is spoken by the pilot's own
    /// telephony (the setting `airline_telephony`, "WAT=Walker", reaching the
    /// crate through its provider): SimBrief's WAT2516 is "Walker two five one
    /// six" to the tower, and "Walker 2516, request taxi" is ours keyed and
    /// unkeyed (Walker Air was spelled "Whiskey Alpha Tango", 2026-09-27).
    #[test]
    fn a_pilots_telephony_is_spoken_and_addressed() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::callsign::provide_telephony(|| "wat=Walker; junk".to_string());
        let path = lxgb_apt_file();
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let mut ctx = scripted_ctx(&mut st, wx);
        let inputs = crate::callsign::Inputs {
            override_id: None,
            plan: Some(crate::callsign::PlanCallsign {
                id: "WAT2516".into(),
                plan_type: "B738".into(),
                also_types: Vec::new(),
            }),
        };
        let r = crate::callsign::resolve(&inputs, Some("G-EUPT"), Some("B738"), Some(79_000.0))
            .expect("a callsign");
        assert_eq!(r.source, crate::callsign::Source::Simbrief);
        ctx.callsign = r.spoken;
        assert_eq!(ctx.callsign, "Walker two five one six");

        let r = st
            .route_unkeyed("Walker 2516, request taxi", Service::Tower, &ctx, wx)
            .expect("our callsign addresses the tower");
        assert!(r.starts_with("Walker two five one six"), "{r}");
        assert!(r.contains("holding point"), "{r}");
        assert!(
            st.route_unkeyed("Walker 2517, ready to taxi", Service::Tower, &ctx, wx)
                .is_none(),
            "another flight's number is not ours"
        );

        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let r = st.route_keyed(
            "walker two five one six, request taxi",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.starts_with("Walker two five one six"), "{r}");
        assert!(r.contains("holding point"), "{r}");

        *crate::party_line::CALLSIGN_TELEPHONY
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = None;
        assert_eq!(
            crate::callsign::spoken_id("WAT2516"),
            "Whiskey Alpha Tango two five one six",
            "without the setting, spelled as before"
        );
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    #[test]
    fn readback_discipline_runs_through_the_keyed_route() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 8.0,
        }; // runway 09
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.on_ground = true;

        let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx);
        assert!(r.contains("holding point"), "{r}");
        let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared for takeoff"), "{r}");
        // The coach owes exactly the readback now.
        let owed = CLEARANCE
            .lock()
            .unwrap()
            .as_ref()
            .and_then(|c| c.pending_readback.clone())
            .expect("readback owed");
        assert!(
            owed.contains("Cleared for takeoff runway zero niner"),
            "{owed}"
        );
        // A different call is challenged, and the challenge repeats the
        // instruction; nothing moves. (A TAXI request is not challenged: it
        // cancels the takeoff, see `a_taxi_request_cancels_a_takeoff_clearance`.)
        let r = st.route_keyed(
            "gibraltar tower, request start up",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("read back"), "{r}");
        assert!(r.contains("cleared for takeoff"), "{r}");
        assert_eq!(
            st.twr.state,
            tower::State::TakeoffCleared,
            "challenge moves nothing"
        );
        // The WRONG runway is corrected, never accepted.
        let r = st.route_keyed(
            "cleared for takeoff runway two seven, november one seven two sierra papa",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("negative"), "{r}");
        // The right echo earns silence and pays the debt.
        let r = st.route_keyed(
            "cleared for takeoff runway zero niner, november one seven two sierra papa",
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(r, "", "correct readback = a quiet frequency");
        assert!(
            CLEARANCE
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .pending_readback
                .is_none(),
            "debt paid, coach released"
        );

        // Enforcement off: the challenge disappears, the call routes.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").unwrap();
        st.set_enforce_readback(false);
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.on_ground = true;
        st.route_keyed("request taxi", Service::Tower, &ctx, wx);
        let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared for takeoff"), "{r}");
        let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx);
        assert!(
            r.contains("holding point"),
            "no challenge with enforcement off: {r}"
        );

        *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    /// EGKK 2026-09-27 (sorties 1790525500 and 1790525553) replayed through
    /// the keyed route. The resumed 737 was cleared for takeoff; its queued
    /// "ready for departure" went out as it rolled, traffic stood on the
    /// strip, and the tower answered "cancel takeoff clearance, hold
    /// position" and armed the release watch. The crew rejected and stopped.
    /// Then: the watch replayed "traffic clear, cleared for takeoff" 1.2 s
    /// into the resume, and the resume's taxi request was answered "read
    /// back: runway two six left, cleared for takeoff". Now the crew's reject
    /// call cancels it all, and the taxi request gets its taxi.
    #[test]
    fn a_rejected_takeoff_cancels_the_clearance_and_the_held_release() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *WAKE.lock().unwrap() = None;
        *TAKEOFF_HOLD.lock().unwrap() = None;
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 8.0,
        };
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.on_ground = true;
        let cs = "november one seven two sierra papa";

        st.route_keyed(&format!("{cs}, request taxi"), Service::Tower, &ctx, wx);
        let r = st.route_keyed(
            &format!("{cs}, ready for departure"),
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("cleared for takeoff"), "{r}");
        let r = st.route_keyed(
            &format!("cleared for takeoff runway zero niner, {cs}"),
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(r, "", "read back");
        // The queued re-report lands with traffic on the strip: cancelled,
        // held, the release watch armed.
        let mut busy = ctx.clone();
        busy.runway_occupied = true;
        let r = st.route_keyed(
            &format!("{cs}, ready for departure"),
            Service::Tower,
            &busy,
            wx,
        );
        assert!(r.contains("cancel takeoff clearance"), "{r}");
        assert!(
            TAKEOFF_HOLD.lock().unwrap().is_some(),
            "release watch armed"
        );
        // A released clearance the main loop has not synced yet.
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = true;

        // The crew's reject call, as the drain keys it: never challenged for
        // the readback it owes, answered, and everything it armed retired.
        let r = st.route_keyed(
            &format!("{cs}, rejecting takeoff"),
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("roger") && r.contains("Hold position"), "{r}");
        assert!(!r.contains("read back"), "{r}");
        assert_eq!(
            st.twr.state,
            tower::State::LinedUp,
            "on the runway, no clearance"
        );
        assert!(st.twr.pending().is_none(), "nothing owed");
        assert!(
            TAKEOFF_HOLD.lock().unwrap().is_none(),
            "the held release is void"
        );
        assert!(
            !*PENDING_TAKEOFF_CLEARANCE.lock().unwrap(),
            "the unsynced release is void"
        );
        let (state, holds, owed) = {
            let c = CLEARANCE.lock().unwrap();
            let c = c.as_ref().unwrap();
            (c.state, c.holds_runway, c.pending_readback.clone())
        };
        assert_eq!(state, tower::State::LinedUp);
        assert!(holds, "the runway is still ours");
        assert!(owed.is_none(), "the coach owes nothing");
        // Nothing replays, however long the resume takes.
        let t0 = Instant::now();
        for s in [0, 15, 30, 120] {
            assert!(
                takeoff_hold_tick(&[], 11.0, true, t0 + std::time::Duration::from_secs(s))
                    .is_none(),
                "no replay at +{s} s"
            );
        }

        // The resume, stopped mid-runway: its taxi request gets the taxi.
        let r = st.route_keyed(
            &format!("{cs}, request taxi, runway zero niner"),
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("holding point"), "{r}");
        assert!(!r.contains("read back"), "{r}");
        assert_eq!(st.twr.state, tower::State::TaxiCleared);

        // From the start of the runway instead, it asks for takeoff again.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").unwrap();
        st.route_keyed(&format!("{cs}, request taxi"), Service::Tower, &ctx, wx);
        st.route_keyed(
            &format!("{cs}, ready for departure"),
            Service::Tower,
            &ctx,
            wx,
        );
        st.route_keyed(&format!("{cs}, stopping"), Service::Tower, &ctx, wx);
        assert_eq!(st.twr.state, tower::State::LinedUp);
        let r = st.route_keyed(
            &format!("{cs}, ready for departure"),
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("cleared for takeoff"), "a fresh clearance: {r}");

        *CLEARANCE.lock().unwrap() = None;
        *HANDOFF.lock().unwrap() = None;
        *TAKEOFF_HOLD.lock().unwrap() = None;
        *TAXI_ROUTE.lock().unwrap() = None;
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
    }

    /// The tower's own eyes, for a reject call it never heard: the hold armed
    /// at +14 of sortie 1790525500 rode a roll to 94 kt and was still armed at
    /// the resume. Rolling past 40 kt spends it; a taxi at 25 kt does not.
    #[test]
    fn a_held_release_is_spent_by_the_roll() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *TAKEOFF_HOLD.lock().unwrap() = Some(TakeoffHold {
            callsign: "Whiskey Alpha Tango one seven three".into(),
            runway: "two six left".into(),
            runway_ident: "26L".into(),
            clear_since: None,
            armed_at: Instant::now(),
            logged_at: Instant::now(),
            lined_up: true,
            behind: false,
            held_for: None,
            cut_in_logged: Default::default(),
        });
        assert!(
            !void_takeoff_hold_by_roll(25.0),
            "a taxi pace spends nothing"
        );
        assert!(TAKEOFF_HOLD.lock().unwrap().is_some());
        assert!(void_takeoff_hold_by_roll(94.0), "the roll spends it");
        assert!(TAKEOFF_HOLD.lock().unwrap().is_none());
        assert!(!void_takeoff_hold_by_roll(94.0), "once");
        assert!(takeoff_hold_tick(&[], 200.0, true, Instant::now()).is_none());
    }

    /// A flight cleared for takeoff that asks to taxi is not taking the
    /// takeoff: the tower cancels it out loud and gives the taxi, never "read
    /// back: cleared for takeoff" (the EGKK resume's answer when the crew's
    /// reject call never reached the tower).
    #[test]
    fn a_taxi_request_cancels_a_takeoff_clearance() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *TAKEOFF_HOLD.lock().unwrap() = None;
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 8.0,
        };
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.on_ground = true;
        st.route_keyed("request taxi", Service::Tower, &ctx, wx);
        let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared for takeoff"), "{r}");
        assert!(st.twr.pending().is_some(), "the takeoff readback is owed");
        let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx);
        assert!(r.contains("takeoff clearance cancelled"), "{r}");
        assert!(r.contains("holding point"), "{r}");
        assert!(!r.contains("read back"), "{r}");
        assert_eq!(st.twr.state, tower::State::TaxiCleared);
        assert!(st.twr.pending().is_none());
        *CLEARANCE.lock().unwrap() = None;
        *HANDOFF.lock().unwrap() = None;
        *TAXI_ROUTE.lock().unwrap() = None;
    }

    #[test]
    fn conditional_line_up_arms_the_release_watch() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 8.0,
        };
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.on_ground = true;
        ctx.final_traffic_nm = Some(2.5);
        ctx.final_traffic_count = 1;
        ctx.final_traffic_callsign = Some("Speedbird three zero six".into());
        *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *WAKE.lock().unwrap_or_else(|e| e.into_inner()) = None;

        st.respond("", Service::Tower, tower::Intent::RequestTaxi, &ctx, wx);
        let r = st.respond("", Service::Tower, tower::Intent::RequestTakeoff, &ctx, wx);
        assert!(
            r.contains("behind the landing Speedbird three zero six"),
            "{r}"
        );
        assert!(r.contains("line up runway zero niner behind"), "{r}");
        assert_eq!(st.twr.state, tower::State::LinedUp);
        assert!(
            TAKEOFF_HOLD.lock().unwrap().is_some(),
            "a conditional line-up owes the clearance: release watch armed"
        );
        assert!(
            TAKEOFF_HOLD
                .lock()
                .unwrap()
                .as_ref()
                .is_some_and(|h| h.behind),
            "the watch knows the line-up was conditional"
        );

        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1508156,
            lon_a: -5.3596312,
            lat_b: 36.1514944,
            lon_b: -5.3400082,
        }];
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // The lander still on final: the crew's condition reads NOT clear —
        // "behind" means after it, and the Baron that rolled five seconds
        // after the words (Malaga, 2026-09-11) had nothing to read.
        let lander = TcasTarget {
            callsign: "BAW306".into(),
            lat: 36.1497,
            lon: -5.4000,
            ele_ft: 650.0,
            on_ground: false,
            vs_fpm: -600.0,
        };
        assert!(
            takeoff_hold_tick(std::slice::from_ref(&lander), 11.0, true, t0).is_none(),
            "held behind the lander"
        );
        assert_eq!(crate::party_line::line_up_condition_clear(), Some(false));

        // The lander is down and gone: the condition clears, then the watch
        // releases the clearance after the clear window, and the FSM sync is
        // queued for the main loop exactly like a plain hold.
        assert!(
            takeoff_hold_tick(&[], 11.0, true, t0).is_none(),
            "clear window starts"
        );
        assert_eq!(
            crate::party_line::line_up_condition_clear(),
            Some(true),
            "lander off the final: the pavement is ours"
        );
        let line = takeoff_hold_tick(&[], 11.0, true, t0 + sec(16)).expect("release");
        assert!(line.contains("cleared for takeoff"), "{line}");
        assert!(*PENDING_TAKEOFF_CLEARANCE.lock().unwrap());
        assert_eq!(
            crate::party_line::line_up_condition_clear(),
            None,
            "released: no condition left to wait on"
        );

        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *TAKEOFF_HOLD.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *CLEARANCE.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner()) = Vec::new();
    }

    /// "Wake turbulence — I can never take off." The wake gate is capped at
    /// one full separation interval AT THE HOLD, so a field whose departure
    /// stream refreshes the clock faster than it expires still lets you go
    /// (2026-07-27). But the cap is measured from when the hold was armed,
    /// and every new takeoff request re-armed it: ask a second time at
    /// Heathrow and you hand your own clock back to zero, for ever.
    #[test]
    fn asking_again_does_not_restart_the_wake_clock() {
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        let hold = |ident: &str, armed_at: Instant| TakeoffHold {
            callsign: "November one two three".into(),
            runway: "27R".into(),
            runway_ident: ident.into(),
            clear_since: None,
            armed_at,
            logged_at: armed_at,
            lined_up: false,
            behind: false,
            held_for: None,
            cut_in_logged: Default::default(),
        };
        // Nothing standing: the clock starts now.
        assert_eq!(hold_armed_at(None, "27R", t0), t0);
        // Ninety seconds in, the pilot asks again. The hold keeps its stamp,
        // so thirty seconds later the cap has run and the release can come.
        let standing = hold("27R", t0);
        let again = hold_armed_at(Some(&standing), "27R", t0 + sec(90));
        assert_eq!(again, t0, "re-asking must not rewind the separation");
        assert!(
            (t0 + sec(150)).duration_since(again).as_secs() >= WAKE_SECS,
            "the cap must be reachable however often we ask"
        );
        // A hold on the OTHER strip is a different queue: its own clock.
        assert_eq!(
            hold_armed_at(Some(&standing), "09L", t0 + sec(90)),
            t0 + sec(90)
        );
    }

    #[test]
    fn wake_clock_starts_on_an_airline_liftoff_and_blocks_the_release() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_PANEL.lock().unwrap() = Some(("Gibraltar".into(), 11.0, Vec::new()));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1508156,
            lon_a: -5.3596312,
            lat_b: 36.1514944,
            lon_b: -5.3400082,
        }];
        *WAKE.lock().unwrap() = None;
        *WAKE_PREV.lock().unwrap() = None;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        let rolling = TcasTarget {
            callsign: "BA306".into(),
            lat: 36.1510,
            lon: -5.3590,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        wake_note_targets(std::slice::from_ref(&rolling), t0);
        assert!(WAKE.lock().unwrap().is_none(), "still rolling: no clock");
        let airborne = TcasTarget {
            ele_ft: 150.0,
            on_ground: false,
            ..rolling
        };
        wake_note_targets(&[airborne], t0 + sec(5));
        let rem = wake_remaining("09", t0 + sec(10)).expect("clock running");
        assert!(rem > 100, "{rem}");
        // Both ends of the same pavement share the wake.
        assert!(wake_remaining("27", t0 + sec(10)).is_some());
        // A strip this field doesn't have: not our wake.
        assert!(wake_remaining("14", t0 + sec(10)).is_none());
        // Two minutes later it has died out.
        assert!(wake_remaining("09", t0 + sec(125)).is_none());

        // A GA tail lifting off starts NO clock — no airline prefix, no
        // assumed weight class.
        *WAKE.lock().unwrap() = None;
        *WAKE_PREV.lock().unwrap() = None;
        let ga = TcasTarget {
            callsign: "N123AB".into(),
            lat: 36.1510,
            lon: -5.3590,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        wake_note_targets(std::slice::from_ref(&ga), t0);
        wake_note_targets(
            &[TcasTarget {
                ele_ft: 150.0,
                on_ground: false,
                ..ga
            }],
            t0 + sec(5),
        );
        assert!(
            WAKE.lock().unwrap().is_none(),
            "GA departure, no wake clock"
        );

        *WAKE_PREV.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn off_route_taxiing_gets_a_nudge_with_the_via_line() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
        *ACTIVE_PANEL.lock().unwrap() = Some((
            "Gibraltar".into(),
            11.0,
            vec![("tower".to_string(), 131200_i64)],
        ));
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1508156,
            lon_a: -5.3596312,
            lat_b: 36.1514944,
            lon_b: -5.3400082,
        }];
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TaxiCleared,
            callsign: "November one seven two Sierra Papa".into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        // The cleared route: a straight east-west line 1 km north of the
        // runway.
        *TAXI_ROUTE.lock().unwrap() = Some(TaxiWatch {
            path: vec![(36.1600, -5.3600), (36.1600, -5.3500)],
            via: "Alpha, Bravo".into(),
            seg: vec![("A".into(), false)],
            holding_point: "A1".into(),
            lead_on: vec![],
        });
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // Parked on the route: settles the air/ground edge, no call.
        assert!(compliance_tick(36.1600, -5.3600, 15.0, true, 0.0, true, t0).is_none());
        // Taxiing ALONG the route: quiet.
        assert!(compliance_tick(36.1600, -5.3550, 15.0, true, 12.0, true, t0 + sec(1)).is_none());
        // 200 m off it: the nudge, with the via line, from the controller.
        let call = compliance_tick(36.1580, -5.3550, 15.0, true, 12.0, true, t0 + sec(2))
            .expect("off-route nudge");
        assert!(!call.crew);
        assert!(
            call.line.contains("off the cleared taxi route"),
            "{}",
            call.line
        );
        assert!(call.line.contains("Taxi via Alpha, Bravo"), "{}", call.line);
        // Its own cooldown holds — a nudge, not a siren.
        assert!(compliance_tick(36.1580, -5.3550, 15.0, true, 12.0, true, t0 + sec(10)).is_none());

        *TAXI_ROUTE.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn coach_owes_the_readback_first_and_teaches_the_line_up() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_PANEL.lock().unwrap() = Some(("Gibraltar".into(), 11.0, Vec::new()));
        let cs = "November one seven two Sierra Papa";
        // A readback owed IS the next call — it outranks the state grid.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TakeoffCleared,
            callsign: cs.into(),
            runway_ident: "09".into(),
            pending_readback: Some(format!("Cleared for takeoff runway zero niner, {cs}.")),
            holds_runway: false,
        });
        let s = suggest_call(cs, true, None, None).expect("a suggestion");
        assert_eq!(
            s.call.as_deref(),
            Some(format!("Cleared for takeoff runway zero niner, {cs}.").as_str())
        );
        assert!(s.urgent, "an owed readback is urgent");
        assert!(s.first.is_none(), "the readback needs no prerequisite");
        // Lined up with the readback paid: the nudge parses as a fresh ask.
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::LinedUp,
            callsign: cs.into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        });
        let s = suggest_call(cs, true, None, None).expect("a suggestion");
        assert!(!s.urgent);
        let c = s.call.expect("a call");
        assert_eq!(
            tower::parse_intent(&c),
            tower::Intent::RequestTakeoff,
            "{c}"
        );
        *CLEARANCE.lock().unwrap() = None;
    }

    #[test]
    fn narrated_takeoff_names_the_departure_end_not_the_nearest() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.1508156,
            lon_a: -5.3596312,
            lat_b: 36.1514944,
            lon_b: -5.3400082,
        }];
        *GHOSTS.lock().unwrap() = None;
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        // Rolling east on 09, still on the ground: seeds the track memory.
        let rolling = TcasTarget {
            callsign: "BAW123".into(),
            lat: 36.1512,
            lon: -5.3500,
            ele_ft: 11.0,
            on_ground: true,
            vs_fpm: 0.0,
        };
        assert!(chatter_tick(
            std::slice::from_ref(&rolling),
            "Gibraltar",
            36.1512,
            -5.3554,
            11.0,
            None,
            t0
        )
        .is_none());
        // Liftoff PAST midfield, near the 27 threshold: the nearest end is
        // 27, but the roll direction says 09 — the tape read every 09R
        // departure as "two seven left" until this.
        let lifted = TcasTarget {
            lat: 36.1514,
            lon: -5.3410,
            ele_ft: 60.0,
            on_ground: false,
            ..rolling
        };
        let ex = chatter_tick(
            &[lifted],
            "Gibraltar",
            36.1512,
            -5.3554,
            11.0,
            None,
            t0 + sec(1),
        )
        .expect("takeoff narrated");
        let line = &ex.lines[0].text;
        assert!(line.contains("runway zero niner"), "{line}");
        assert!(!line.contains("two seven"), "{line}");
        *GHOSTS.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn fresh_station_install_arms_the_zone_grace() {
        // A region handoff says "contact <name> Tower" — the new station's
        // zone watch must give the pilot the cooldown window to actually
        // call, not challenge them as an intruder two seconds later
        // (2026-07-10 tape).
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        lxgb().install();
        assert!(
            COMPLIANCE.lock().unwrap().zone_last.is_some(),
            "check-in grace armed on install"
        );
        *ATIS.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *ACTIVE_PANEL.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = None;
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *CONTROLLER_FREQS.lock().unwrap() = Vec::new();
    }

    #[test]
    fn zone_challenge_on_frequency_is_published_for_the_crew() {
        // Granada's zone watch challenged a full-auto arrival three times
        // while it flew the last enroute leg through the zone, and the crew
        // had no way to hear the question (2026-09-15). The tuned challenge
        // arms the party line; the copilot's own "we're inside the zone"
        // (radio not on the tower) does not, and the flag drains on read.
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (flat, flon) = (36.1512, -5.3554);
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), flat, flon));
        *ACTIVE_PANEL.lock().unwrap() = Some((
            "Gibraltar".into(),
            11.0,
            vec![("tower".to_string(), 131200_i64)],
        ));
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *ZONE_CHALLENGE.lock().unwrap() = false;
        let t0 = Instant::now();
        // Airborne, 2 nm out, 1,500 ft, tuned to the tower: the controller asks.
        let t = t0 + std::time::Duration::from_secs_f64(ZONE_COOLDOWN_SECS + 1.0);
        let call = compliance_tick(flat + 0.03, flon, 1500.0, false, 100.0, true, t)
            .expect("the zone watch challenges an unknown aircraft");
        assert!(!call.crew, "{}", call.line);
        assert!(
            call.line.contains("Say callsign and intentions"),
            "{}",
            call.line
        );
        assert!(
            crate::party_line::take_zone_challenge(),
            "published for the crew"
        );
        assert!(!crate::party_line::take_zone_challenge(), "drained on read");
        // Radio not on the tower: the copilot's line, no question on the air.
        let t = t + std::time::Duration::from_secs_f64(ZONE_COOLDOWN_SECS + 1.0);
        let call = compliance_tick(flat + 0.03, flon, 1500.0, false, 100.0, false, t)
            .expect("the copilot flags the zone");
        assert!(call.crew, "{}", call.line);
        assert!(
            !crate::party_line::take_zone_challenge(),
            "no challenge was spoken"
        );
        Station::uninstall();
    }

    /// #83: the answer to a transit field's challenge is a transit request,
    /// and it is two-way contact: the tower clears it through and its zone
    /// watch asks no more (EDVW-ETND 2026-09-23: challenged on every
    /// cooldown, nobody answered).
    #[test]
    fn a_transit_answers_the_zone_challenge_and_the_watch_stands_down() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (flat, flon) = (36.1512, -5.3554);
        *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), flat, flon));
        *ACTIVE_PANEL.lock().unwrap() = Some((
            "Gibraltar".into(),
            11.0,
            vec![("tower".to_string(), 131200_i64)],
        ));
        *CLEARANCE.lock().unwrap() = None;
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *ZONE_CHALLENGE.lock().unwrap() = false;
        let t0 = Instant::now();
        let t = t0 + std::time::Duration::from_secs_f64(ZONE_COOLDOWN_SECS + 1.0);
        compliance_tick(flat + 0.03, flon, 1500.0, false, 100.0, true, t)
            .expect("challenged");
        assert!(crate::party_line::take_zone_challenge());
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 8.0,
        };
        let mut st = lxgb();
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "Delta Hotel Xray Alpha Bravo".into(),
            false,
            false,
            None,
            false,
            None,
        );
        let call = "Gibraltar Tower, Delta Hotel Xray Alpha Bravo, helicopter, one thousand five hundred feet, request zone transit";
        let intent = tower::parse_intent(call);
        assert_eq!(intent, tower::Intent::RequestTransit);
        let r = st.respond(call, Service::Tower, intent, &ctx, wx);
        assert!(r.contains("cleared to transit the Gibraltar zone"), "{r}");
        assert!(CLEARANCE.lock().unwrap().is_some(), "two-way contact");
        // Past the cooldown, still in the zone: no second challenge.
        let t = t + std::time::Duration::from_secs_f64(ZONE_COOLDOWN_SECS + 1.0);
        assert!(
            compliance_tick(flat + 0.03, flon, 1500.0, false, 100.0, true, t).is_none(),
            "the watch stands down once the transit has called"
        );
        assert!(!crate::party_line::take_zone_challenge());
        *CLEARANCE.lock().unwrap() = None;
        Station::uninstall();
    }

    #[test]
    fn uninstall_retires_the_whole_world() {
        // Leaving ATC mode mid-flight must clear everything the crew can
        // see — a stale clearance or taxi route would keep coaching the
        // pilot toward a tower that no longer exists.
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        lxgb().install();
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TakeoffCleared,
            callsign: "November one two three alpha bravo".into(),
            runway_ident: "27".into(),
            pending_readback: None,
            holds_runway: false,
        });
        *LANDING_HANDOFF.lock().unwrap() = Some("27".into());
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = true;
        Station::uninstall();
        assert!(
            crate::party_line::clearance_state().is_none(),
            "clearance cleared"
        );
        assert!(
            crate::party_line::landing_handoff_runway().is_none(),
            "handoff cleared"
        );
        assert!(
            !crate::party_line::take_pending_takeoff_clearance(),
            "pending drained"
        );
        assert!(ACTIVE_FIELD.lock().unwrap().is_none(), "field retired");
        assert!(ACTIVE_PANEL.lock().unwrap().is_none(), "panel retired");
        assert!(ATIS.lock().unwrap().is_none(), "ATIS off the air");
        assert!(
            CONTROLLER_FREQS.lock().unwrap().is_empty(),
            "no controllers"
        );
    }

    #[test]
    fn taxiways_speak_like_a_controller() {
        assert_eq!(spoken_taxiway("A"), "Alpha");
        assert_eq!(spoken_taxiway("B2"), "Bravo two");
        assert_eq!(spoken_taxiway("A1"), "Alpha one");
        assert_eq!(spoken_taxiway("N4W"), "November four Whiskey");
        assert_eq!(spoken_taxiway("LINK27"), "Link two seven");
        assert_eq!(spoken_taxiway("Link 41"), "Link four one");
        assert_eq!(spoken_taxiway("Inner"), "Inner");
        assert_eq!(first_designator("09L/27R"), "09L");
        assert_eq!(first_designator("14"), "14");
    }

    #[test]
    fn via_lists_collapse_numbered_runs() {
        let names = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        // Madrid's real 2026-07-13 clearance: "Charlie one, Mike one, Mike
        // two, Mike three, Mike four, Mike five" reads like the database.
        assert_eq!(
            spoken_via(&names(&["C1", "M1", "M2", "M3", "M4", "M5"])),
            vec!["Charlie one", "Mike one to Mike five"]
        );
        // Descending runs collapse the same way (taxiing the other way).
        assert_eq!(
            spoken_via(&names(&["M5", "M4", "M3", "B"])),
            vec!["Mike five to Mike three", "Bravo"]
        );
        // Two in a row is not a run — say both.
        assert_eq!(
            spoken_via(&names(&["M1", "M2", "A"])),
            vec!["Mike one", "Mike two", "Alpha"]
        );
        // A gap breaks the run; a different designator breaks the run.
        assert_eq!(
            spoken_via(&names(&["M1", "M3", "M4", "N1"])),
            vec!["Mike one", "Mike three", "Mike four", "November one"]
        );
        // Unnumbered names pass straight through.
        assert_eq!(
            spoken_via(&names(&["L", "B", "G11", "A", "HN1"])),
            vec![
                "Lima",
                "Bravo",
                "Golf one one",
                "Alpha",
                "Hotel November one"
            ]
        );
    }

    /// The world sweep: EVERY towered airport in the real Global Airports
    /// file stands up a station, briefs its frequencies, and flies the
    /// departure happy path — the shakeout for apt.dat shapes LXGB never
    /// showed us. Opt-in because the file ships with X-Plane, not the repo:
    ///
    ///   cargo test --release -p atc -- --ignored atc_world_sweep
    ///
    /// Override the file with XCOPILOT_APT_DAT.
    #[test]
    #[ignore = "needs X-Plane's Global Airports apt.dat; run with -- --ignored"]
    fn atc_world_sweep_every_towered_field_stands_up() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = std::env::var("XCOPILOT_APT_DAT").unwrap_or_else(|_| {
            crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat")
        });
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        let (mut total, mut towered, mut no_runways) = (0u32, 0u32, 0u32);
        let mut fails: Vec<String> = Vec::new();
        nav::scan_airports(&path, |icao, apt| {
            total += 1;
            let Some(mut st) = Station::from_airport(apt) else {
                return;
            };
            towered += 1;
            let mut check = |cond: bool, what: &str| {
                if !cond && fails.len() < 40 {
                    fails.push(format!("{icao}: {what}"));
                }
            };
            check(st.covers_khz(st.tower_khz), "tower frequency not covered");

            // The briefing must read every published service, and name the
            // tower when COM1 sits on it.
            let brief = st.briefing_text(Some(st.tower_khz));
            check(
                brief.contains("tower"),
                &format!("no tower in briefing: {brief}"),
            );
            check(
                brief.ends_with("Com one is on the tower."),
                &format!("tuned service unnamed: {brief}"),
            );

            // The departure happy path, spoken with live-shaped context.
            let ctx = scripted_ctx(&mut st, wx);
            if ctx.runway.is_empty() {
                no_runways += 1; // towered heliports/seaplane bases: no 100 rows
            }
            let r = st.route_keyed("radio check", Service::Tower, &ctx, wx);
            check(r.contains("reading you five"), &format!("radio check: {r}"));
            let r = st.route_keyed("request taxi", Service::Tower, &ctx, wx);
            check(r.contains("taxi to holding point"), &format!("taxi: {r}"));
            let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
            check(r.contains("cleared for takeoff"), &format!("takeoff: {r}"));
            if !ctx.runway.is_empty() {
                check(
                    r.contains(&ctx.runway),
                    &format!("clearance names the wrong runway: {r}"),
                );
            }

            // Vectors exercise the runway geometry math per field.
            let r = st.route_keyed("request vectors", Service::Approach, &ctx, wx);
            check(!r.is_empty(), "vectors: empty reply");

            // Statics must not leak between airports (or into other tests).
            *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
            *VECTORS.lock().unwrap_or_else(|e| e.into_inner()) = None;
        })
        .expect("apt.dat scan");
        println!("swept {total} airports, {towered} towered ({no_runways} with no runway rows)");
        assert!(towered > 100, "suspiciously few towered fields: {towered}");
        assert!(
            fails.is_empty(),
            "{} fields failed (first {}):\n{}",
            fails.len(),
            fails.len().min(40),
            fails.join("\n")
        );
    }

    /// LEMG 31 (sortie 1790584715-full_auto, 2026-09-28): a right-hand
    /// circuit heard "contact Malaga Departure on one two one decimal zero
    /// two five" on the crosswind leg at 772 ft, and "radar contact" in the
    /// tower's voice 0.4 s after rotation from a Departure watch a zone
    /// clearance had armed. A flight that says it is a circuit stays with
    /// the tower: no handoff, no radar watch. The next departure that does
    /// not say so is handed off as before.
    #[test]
    fn radar_contact_is_the_radar_units_never_the_towers() {
        // LEMG 31 (sortie 1790584715, 2026-09-28, t 1790585265.4, route
        // squawk_watch): the zone clearance's Departure watch said "radar
        // contact" in the TOWER's voice 4.3 s after rotation, and Departure
        // said it again at the check-in.
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let ctx = scripted_ctx(&mut st, wx);
        *HANDOFF.lock().unwrap() = None;
        *SQUAWK_WATCH.lock().unwrap() = None;
        *SQUAWK_NUDGE.lock().unwrap() = None;
        let _ = crate::party_line::take_departure_handoff();

        let r = st.route_keyed("request clearance", Service::Tower, &ctx, wx);
        assert!(r.contains("squawk"), "{r}");
        let code = st.assigned_squawk.expect("code issued");
        let t0 = Instant::now();
        // Airborne on the tower, the code squawking: the tower says nothing,
        // and the watch still waits for the radar unit.
        assert!(squawk_tick(Some(code), true, StationSpeaker::Tower, t0).is_none());
        assert!(squawk_watch_active(), "held, not spent");
        // A wrong code is still relayed: the nudge is an instruction.
        let late = t0 + std::time::Duration::from_secs(25);
        let (line, who) =
            squawk_tick(Some(7000), true, StationSpeaker::Tower, late).expect("relayed nudge");
        assert!(line.contains("confirm squawk"), "{line}");
        assert!(matches!(who, StationSpeaker::Departure), "the assigner's");
        let _ = crate::party_line::take_squawk_nudge();

        // The takeoff clearance arms the handoff: "radar contact" is now the
        // check-in's, even tuned to Departure before the call goes out.
        let r = st.route_keyed("request taxi", Service::Tower, &ctx, wx);
        assert!(r.contains("taxi"), "{r}");
        let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared for takeoff"), "{r}");
        let later = late + std::time::Duration::from_secs(60);
        assert!(squawk_tick(Some(code), true, StationSpeaker::Tower, later).is_none());
        assert!(squawk_tick(Some(code), true, StationSpeaker::Departure, later).is_none());
        let r = st.route_keyed(
            "passing one thousand five hundred climbing",
            Service::Departure,
            &ctx,
            wx,
        );
        assert!(r.contains("Departure, radar contact"), "{r}");
        // Said once: the watch is satisfied by the check-in.
        assert!(!squawk_watch_active(), "the check-in satisfied the watch");
        assert!(squawk_tick(Some(code), true, StationSpeaker::Departure, later).is_none());

        // A code issued by the radar unit on its own frequency (flight
        // following, no handoff): the watch says it there.
        *SQUAWK_WATCH.lock().unwrap() = None;
        st.assigned_squawk = None;
        st.assign_squawk("November one two three", StationSpeaker::Departure);
        let code = st.assigned_squawk.expect("code issued");
        let (line, who) = squawk_tick(Some(code), true, StationSpeaker::Departure, t0)
            .expect("radar contact on Departure");
        assert!(line.contains("radar contact"), "{line}");
        assert!(matches!(who, StationSpeaker::Departure));
        *SQUAWK_WATCH.lock().unwrap() = None;
        *HANDOFF.lock().unwrap() = None;
        let _ = crate::party_line::take_departure_handoff();
    }

    /// LEMG 13 (sorties 1790590663 and 1790591256, 2026-09-28): the runway
    /// start's "request clearance, runway one three, for circuits" was
    /// answered "cleared to leave the Malaga zone, not above two thousand
    /// feet". Circuit traffic gets the circuit clearance in the field's
    /// dialect (runway, height, pressure, code, report ready), and the crew's
    /// readback of it, the owed line as the crew transmits it, is silence.
    #[test]
    fn a_circuit_zone_request_is_cleared_for_the_circuit() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        for faa in [false, true] {
            *SQUAWK_WATCH.lock().unwrap() = None;
            *CLEARANCE.lock().unwrap() = None;
            let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
            if faa {
                st.set_dialect(Dialect::Faa);
            }
            let ctx = scripted_ctx(&mut st, wx);
            let r = st.route_keyed(
                "november one seven two sierra papa, request clearance, runway two seven, \
                 for circuits",
                Service::Tower,
                &ctx,
                wx,
            );
            assert!(st.circuits, "circuits noted from the zone request");
            assert!(!r.contains("leave the"), "{r}");
            assert!(r.contains("runway two seven"), "{r}");
            assert!(r.contains("squawk"), "{r}");
            assert!(r.ends_with("report ready for departure."), "{r}");
            if faa {
                assert!(
                    r.contains("closed traffic approved, pattern altitude one thousand"),
                    "{r}"
                );
            } else {
                assert!(r.contains("circuit height one thousand feet"), "{r}");
            }
            assert!(!squawk_watch_active(), "no radar watch for pattern traffic");
            let owed = crate::party_line::readback_owed().expect("readback suggested");
            assert!(!owed.contains("above"), "{owed}");
            // The crew transmits the owed line, stop trimmed: silence, paid.
            let r = st.route_keyed(owed.trim_end_matches('.'), Service::Tower, &ctx, wx);
            assert_eq!(r, "", "the readback is accepted");
            assert_eq!(crate::party_line::readback_owed(), None);
            assert_eq!(st.twr.state, tower::State::Parked);
            // A plain zone request states the intent afresh: a departure.
            let r = st.route_keyed("request clearance", Service::Tower, &ctx, wx);
            assert!(!st.circuits);
            assert!(r.contains("cleared to leave the Gibraltar zone"), "{r}");
        }
        // From the ramp no runway is named (full auto's zone_clearance_call,
        // LEMG sorties 1790605551 and 1790618416): still the circuit.
        *SQUAWK_WATCH.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let ctx = scripted_ctx(&mut st, wx);
        let r = st.route_keyed(
            "november one seven two sierra papa, request clearance, for circuits",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(st.circuits, "circuits noted from the ramp's zone request");
        assert!(!r.contains("leave the"), "{r}");
        assert!(r.contains("circuit height"), "{r}");
        assert!(r.ends_with("report ready for departure."), "{r}");
        *SQUAWK_WATCH.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *HANDOFF.lock().unwrap() = None;
    }

    #[test]
    fn circuit_traffic_stays_with_the_tower() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let wx = Wx {
            wind_from_deg: 250.0,
            wind_kts: 12.0,
        };
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        assert_eq!(st.departure_khz, Some(130400), "a field with a Departure");
        let ctx = scripted_ctx(&mut st, wx);
        *HANDOFF.lock().unwrap() = None;
        *SQUAWK_WATCH.lock().unwrap() = None;
        let _ = crate::party_line::take_departure_handoff();

        // The zone clearance arms the radar watch (the LEMG leftover).
        let r = st.route_keyed("request clearance", Service::Tower, &ctx, wx);
        assert!(r.contains("squawk"), "{r}");
        assert!(squawk_watch_active(), "zone clearance armed the watch");

        // The circuit says so on the taxi request: the watch goes.
        let r = st.route_keyed(
            "november one seven two sierra papa, request taxi for circuits",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("taxi to holding point"), "{r}");
        assert!(st.circuits, "circuits noted");
        assert!(!squawk_watch_active(), "no radar watch for pattern traffic");
        // A plain "ready" does not undo it.
        let r = st.route_keyed(
            "november one seven two sierra papa, ready for departure",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("cleared for takeoff"), "{r}");
        assert_eq!(st.twr.state, tower::State::TakeoffCleared);
        assert!(
            handoff_tick(900.0, false).is_none(),
            "the tower keeps a circuit"
        );
        assert!(handoff_tick(2500.0, false).is_none(), "at any height");
        assert!(
            crate::party_line::take_departure_handoff().is_none(),
            "nothing published for the crew"
        );
        // The hold-release path arms through the same door.
        st.arm_departure_handoff(&ctx);
        assert!(
            handoff_tick(900.0, false).is_none(),
            "released circuit kept too"
        );

        // The pattern calls still work the tower.
        let r = st.route_keyed(
            "november one seven two sierra papa, downwind runway two seven",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("report base"), "{r}");

        // A new departure that says nothing of circuits is a departure.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let ctx = scripted_ctx(&mut st, wx);
        st.route_keyed("request taxi for circuits", Service::Tower, &ctx, wx);
        assert!(st.circuits);
        st.route_keyed("request taxi", Service::Tower, &ctx, wx);
        assert!(!st.circuits, "a taxi request states the intent afresh");
        let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared for takeoff"), "{r}");
        let line = handoff_tick(900.0, false).expect("a departure is handed off");
        assert!(
            line.contains("Departure on one three zero decimal four"),
            "{line}"
        );
        let _ = crate::party_line::take_departure_handoff();
        *SQUAWK_WATCH.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
    }

    #[test]
    fn takeoff_clearance_arms_the_departure_handoff() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        assert_eq!(st.departure_khz, Some(130400), "departure row wired");
        let ctx = tower::Ctx {
            callsign: "November one seven two Sierra Papa".into(),
            airport: "Gibraltar".into(),
            runway: "two seven".into(),
            qnh: "QNH one zero one five".into(),
            wind: "calm".into(),
            holding_point: "Alpha".into(),
            stand: String::new(),
            approach: Some(atis::spoken_mhz(122800)),
            squawk: "seven zero zero zero".into(),
            final_traffic_nm: None,
            final_traffic_callsign: None,
            final_traffic_count: 0,
            runway_occupied: false,
            own_on_runway: false,
            helicopter: false,
            ifr: None,
            circuit: None,
            runway_refusal: None,
            transition_level_ft: None,
            wake_secs_left: None,
            taxi_via: Vec::new(),
            taxi_crossings: Vec::new(),
            on_ground: false,
        };
        // Walk the tower to a takeoff clearance and arm, exactly as respond()
        // does around twr.handle().
        st.twr.handle(tower::parse_intent("request taxi"), &ctx);
        let was = st.twr.state;
        st.twr
            .handle(tower::parse_intent("ready for departure"), &ctx);
        assert_ne!(was, st.twr.state);
        st.arm_departure_handoff(&ctx);

        // On the roll: silence. Climbing through field + 700: the handoff,
        // exactly once.
        assert!(handoff_tick(11.0, true).is_none());
        assert!(handoff_tick(300.0, false).is_none());
        let line = handoff_tick(900.0, false).expect("handoff due");
        assert!(
            line.contains("contact Gibraltar Departure on one three zero decimal four"),
            "{line}"
        );
        assert!(handoff_tick(1500.0, false).is_none(), "spoken once");
    }

    #[test]
    fn named_station_tuning_resolves_from_apt_dat() {
        let st = lxgb();
        match st.tune_request("tune the tower") {
            Some(Tune::Found { label, khz }) => {
                assert_eq!(khz, 131200);
                assert_eq!(label, "Gibraltar tower");
            }
            other => panic!("expected tower: {:?}", other.is_some()),
        }
        // Generic "atc"/"station" means the tower.
        assert!(matches!(
            st.tune_request("tune to atc"),
            Some(Tune::Found { khz: 131200, .. })
        ));
        assert!(matches!(
            st.tune_request("contact approach"),
            Some(Tune::Found { khz: 122800, .. })
        ));
        // A named station the field doesn't have declines by name, no guessing.
        match st.tune_request("tune the atis") {
            Some(Tune::Missing(msg)) => assert!(msg.contains("no atis"), "{msg}"),
            other => panic!("expected Missing: {:?}", other.is_some()),
        }
        // No tuning verb → not a tuning request, even if a station is named
        // ("gibraltar tower request taxi" must reach the tower, not the tuner).
        assert!(st.tune_request("gibraltar tower request taxi").is_none());
        // A verb without any station name is someone else's command.
        assert!(st
            .tune_request("tune com one to one one eight decimal five")
            .is_none());
        // "change frequency to <station>" — how the 2026-07-08 pilot actually
        // asked, four phrasings in a row, all missed.
        assert!(matches!(
            st.tune_request("let's change frequency to gibraltar tower"),
            Some(Tune::Found { khz: 131200, .. })
        ));
        assert!(matches!(
            st.tune_request("change frequency to tower"),
            Some(Tune::Found { khz: 131200, .. })
        ));
        // "switch frequency to tower." (2026-07-09 flight, missed: "switch"
        // only counted when followed directly by "to").
        assert!(matches!(
            st.tune_request("switch frequency to tower."),
            Some(Tune::Found { khz: 131200, .. })
        ));
        // "change"/"switch" without radio context must not retune.
        assert!(st
            .tune_request("change approach speed to seventy")
            .is_none());
        assert!(st.tune_request("switch the landing light on").is_none());
        // "change to approach" tunes (2026-07-08 night flight: answered with
        // "approach around 65 knots" instead) — but flight-parameter talk
        // naming a station never does.
        assert!(matches!(
            st.tune_request("change to approach"),
            Some(Tune::Found { khz: 122800, .. })
        ));
        assert!(st.tune_request("change to approach speed").is_none());
        assert!(st.tune_request("select approach flaps").is_none());
        // STT warts, all live 2026-07-20: ATIS spelled out or heard as
        // "atheist", tower as "tau", and the verb clipped to a short bare
        // "<station> frequency".
        match st.tune_request("change to a t i s frequency.") {
            Some(Tune::Missing(msg)) => assert!(msg.contains("no atis"), "{msg}"),
            other => panic!("spelled-out atis should resolve: {:?}", other.is_some()),
        }
        match st.tune_request("can the vibrator change to atheist frequency") {
            Some(Tune::Missing(msg)) => assert!(msg.contains("no atis"), "{msg}"),
            other => panic!("'atheist' should resolve to atis: {:?}", other.is_some()),
        }
        assert!(matches!(
            st.tune_request("change to tau frequency."),
            Some(Tune::Found { khz: 131200, .. })
        ));
        assert!(matches!(
            st.tune_request("to tower frequency."),
            Some(Tune::Found { khz: 131200, .. })
        ));
        // Long frequency-quoting readbacks still never retune bare.
        assert!(st
            .tune_request("holding point echo one runway two seven frequency copied")
            .is_none());
    }

    #[test]
    fn freq_query_matches_questions_not_commands() {
        assert!(freq_query("what frequencies are here"));
        assert!(freq_query("which stations are in range"));
        assert!(freq_query("list the frequencies"));
        assert!(freq_query("who can i talk to"));
        assert!(freq_query("who can i contact"));
        // Commands and radio calls must fall through to their own paths.
        assert!(!freq_query("set frequency to one one eight decimal five"));
        assert!(!freq_query("tune the tower"));
        assert!(!freq_query("request taxi"));
        assert!(!freq_query("what's the qnh"));
    }

    #[test]
    fn briefing_reads_lxgb_services_and_com1() {
        let path = lxgb_apt_file();
        let st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        // Everything the field publishes, in briefing order, one sentence.
        let b = st.briefing_text(Some(131200));
        assert!(
            b.starts_with(
                "Gibraltar: tower one three one decimal two, \
                 approach one two two decimal eight, \
                 departure one three zero decimal four."
            ),
            "{b}"
        );
        assert!(b.ends_with("Com one is on the tower."), "{b}");
        // Tuned off every service: the raw frequency is read back instead.
        let b = st.briefing_text(Some(118000));
        assert!(
            b.ends_with("Com one is on one one eight decimal zero."),
            "{b}"
        );
        // A failed COM1 read drops that sentence, never a wrong number.
        let b = st.briefing_text(None);
        assert!(b.ends_with("decimal four."), "{b}");
        // Ground and tower on the SAME frequency (8 real fields in the
        // 2026-07 apt.dat): the briefing names the service that answers —
        // the tower — not the first row in briefing order.
        let mut st = lxgb();
        st.apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_GROUND,
            khz: 131200,
            name: "Ground".into(),
        });
        let b = st.briefing_text(Some(131200));
        assert!(b.ends_with("Com one is on the tower."), "{b}");
    }

    #[test]
    fn void_note_points_at_who_answers_not_the_directory() {
        let mut st = lxgb();
        st.apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_GROUND,
            khz: 121800,
            name: "Ground".into(),
        });
        st.apt.freqs.push(nav::AtcFreq {
            kind: nav::FREQ_ATIS,
            khz: 136525,
            name: "ATIS".into(),
        });
        // On the ground tuned to the published but unmanned ground row:
        // name it and point at the tower — never the whole directory.
        let n = st.void_note(Some(121800), false);
        assert!(n.contains("Nobody's manning ground"), "{n}");
        assert!(n.contains("tower, one three one decimal two"), "{n}");
        assert!(!n.contains("departure") && !n.contains("clearance"), "{n}");
        // Transmitting on the ATIS: it's a broadcast, say so.
        let n = st.void_note(Some(136525), false);
        assert!(n.contains("only broadcasts"), "{n}");
        // Off every published row: just the pointer.
        let n = st.void_note(Some(118000), false);
        assert!(n.starts_with("Try Gibraltar tower on"), "{n}");
        // Airborne with an approach published, approach is the one to call.
        let n = st.void_note(Some(118000), true);
        assert!(n.contains("approach"), "{n}");
        // A failed COM1 read still points somewhere useful.
        let n = st.void_note(None, false);
        assert!(n.contains("tower"), "{n}");
    }

    /// A fixture apt.dat with LXGB's rows verbatim from the Global Airports
    /// file (2026-07 cycle), plus a trailing airport to prove the scan stops.
    /// Written exactly once: several tests read it concurrently, and a
    /// per-test `File::create` truncates it under a parallel reader.
    fn lxgb_apt_file() -> std::path::PathBuf {
        use std::io::Write;
        use std::sync::OnceLock;
        static PATH: OnceLock<std::path::PathBuf> = OnceLock::new();
        PATH.get_or_init(|| {
            let path = std::env::temp_dir()
                .join(format!("xc-station-lxgb-{}.dat", std::process::id()));
            let mut f = std::fs::File::create(&path).unwrap();
            writeln!(f, "1     11 0 0 LXGB Gibraltar").unwrap();
            writeln!(f, "100 45.11 1 0 0.20 0 3 0  09 36.1508156 -5.3596312 136 0 5 1 0 3 27 36.1514944 -5.3400082 98 0 5 7 0 3").unwrap();
            writeln!(f, "1054 131200 Tower Freq").unwrap();
            writeln!(f, "1055 122800 Approach Freq").unwrap();
            writeln!(f, "1056 130400 Departure Freq").unwrap();
            writeln!(f, "1   1027 0 0 0MU1 Sunderland").unwrap();
            f.flush().unwrap();
            path
        })
        .clone()
    }

    /// EGKK as the real install's apt.dat has it (2026-09-27): the header,
    /// both 100-rows verbatim (08R/26L 3,319 m with approach and centreline
    /// lights; 08L/26R 2,563 m, the standby) and the tower.
    fn egkk_apt_file() -> std::path::PathBuf {
        use std::io::Write;
        use std::sync::OnceLock;
        static PATH: OnceLock<std::path::PathBuf> = OnceLock::new();
        PATH.get_or_init(|| {
            let path = std::env::temp_dir()
                .join(format!("xc-station-egkk-{}.dat", std::process::id()));
            let mut f = std::fs::File::create(&path).unwrap();
            writeln!(f, "1    203 0 0 EGKK London Gatwick").unwrap();
            writeln!(f, "100 46.00 26 1527 0.25 1 3 0 08R 51.1450993 -0.2123748 395 0 7 4 1 3 26L 51.1514994 -0.1659296 425 30 7 4 1 3").unwrap();
            writeln!(f, "100 46.00 23 853 0.25 0 1 0 08L 51.1468901 -0.2126029 321 0 7 0 0 3 26R 51.1518321 -0.1767408 415 0 7 0 0 3").unwrap();
            writeln!(f, "1052 121955 Gatwick Delivery").unwrap();
            writeln!(f, "1054 124230 Gatwick Tower").unwrap();
            writeln!(f, "1055 126825 Gatwick Director").unwrap();
            writeln!(f, "1   1027 0 0 0MU1 Sunderland").unwrap();
            f.flush().unwrap();
            path
        })
        .clone()
    }

    /// EGKK 2026-09-27 (sortie 1790524628): the IFR clearance at +154 named
    /// 26R with LAM6V, and the taxi at +170 and the takeoff went to 26L. The
    /// wind was 175 at 7.7: across both parallels, whose headwinds differ by
    /// rounding, so the pick was noise. Then the crew's "request taxi, runway
    /// two six left" moved the runway in use under the clearance just read
    /// back. Now: one answer. The main runway (26L) is picked, a stale 26R
    /// moves to it, the SID is 26L's, and the clearance freezes the runway
    /// the taxi goes to.
    #[test]
    fn the_ifr_clearance_and_the_taxi_name_one_runway_at_egkk() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        Station::uninstall();
        *SQUAWK_WATCH.lock().unwrap() = None;
        let dir = std::env::temp_dir().join(format!("xc-station-egkk-sids-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        // Two SIDs to Lambourne as the CIFP has them: M off 26L, V off 26R.
        let leg = |sid: &str, rwy: &str| {
            format!(
                "SID:010,5,{sid},{rwy},LAMXX,EG,E,A,E   , ,   ,TF, ,   ,  , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   ;\n"
            )
        };
        std::fs::write(
            dir.join("EGKK.dat"),
            [leg("LAM6V", "RW26R"), leg("LAM6M", "RW26L")].concat(),
        )
        .unwrap();
        let fixes = dir.join("earth_fix.dat");
        std::fs::write(
            &fixes,
            "I\n1200 Version\n\n 51.646 0.152 LAMXX ENRT EG 2\n99\n",
        )
        .unwrap();
        let mut egbb = region_apt("Birmingham", 52.4539, -1.748);
        egbb.icao = "EGBB".into();
        *REGION.lock().unwrap() = vec![egbb];
        let wx = Wx {
            wind_from_deg: 174.7,
            wind_kts: 7.7,
        };
        let path = egkk_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "EGKK").expect("EGKK loads");
        st.set_procedures_now(dir.to_str().unwrap(), fixes.to_str().unwrap(), "", "EGKK");
        let cs = "Whiskey Alpha Tango one seven three";
        let ctx_now = |st: &mut Station| {
            st.make_ctx(
                wx,
                "QNH one zero one three".into(),
                cs.into(),
                true,
                false,
                None,
                false,
                None,
            )
        };

        // The pick: the main runway of the pair, whichever parallel the
        // wind favours by a hundredth of a knot.
        assert_eq!(st.pick_runway(174.7, 7.7), "26L");
        assert_eq!(
            st.pick_runway(354.7, 7.7),
            "26L",
            "hysteresis: no reversal in 7.7 kt of crosswind"
        );
        // A pick left on the standby (by the old rule, or a calm stream)
        // moves to the main runway while nothing is committed.
        st.runway_in_use = Some("26R".into());
        assert_eq!(st.pick_runway(174.7, 7.7), "26L");

        // The clearance names 26L and 26L's SID.
        let ctx = ctx_now(&mut st);
        let r = st.route_keyed("request ifr clearance to egbb", Service::Tower, &ctx, wx);
        assert!(r.contains("runway two six left"), "{r}");
        assert!(r.contains("Mike departure"), "LAM6M off 26L: {r}");
        assert!(!r.contains("Victor"), "{r}");
        let r = st.route_keyed(
            &format!("cleared to birmingham via the lam six mike departure, runway two six left, climb five thousand feet, squawk {}, {cs}", atis::spoken_digits(&format!("{:04}", st.assigned_squawk.unwrap()))),
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(r, "", "read back");

        // The crew states another runway with its taxi request: the
        // clearance holds the runway the SID was chosen for.
        let taxi = "whiskey alpha tango one seven three, request taxi, runway two six right";
        st.note_stated_runway(taxi, tower::parse_intent(taxi));
        let ctx = ctx_now(&mut st);
        assert_eq!(ctx.runway, "two six left", "the clearance's runway stands");
        let r = st.route_keyed(taxi, Service::Tower, &ctx, wx);
        assert!(r.contains("runway two six left"), "{r}");

        // Airborne, the freeze is spent.
        st.ifr_runway = None;
        *REGION.lock().unwrap() = Vec::new();
        *IFR_CLEARANCE.lock().unwrap() = None;
        *IFR_FLIGHT.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *TAXI_ROUTE.lock().unwrap() = None;
        *SQUAWK_WATCH.lock().unwrap() = None;
        Station::uninstall();
    }

    /// The overheard traffic at +99 of the same sortie was "cleared for
    /// takeoff runway two six right": a replayed ghost 200 m off 26L's
    /// centreline is on 26L. A close parallel names its main runway; a wide
    /// one (Heathrow's 1,400 m) is itself.
    #[test]
    fn a_close_parallel_is_its_main_runway() {
        let st = Station::load(egkk_apt_file().to_str().unwrap(), "EGKK").expect("EGKK loads");
        let rw = &st.apt.runways;
        assert_eq!(nav::close_parallel_main(rw, "26R"), "26L");
        assert_eq!(nav::close_parallel_main(rw, "08L"), "08R");
        assert_eq!(nav::close_parallel_main(rw, "26L"), "26L");
        assert_eq!(
            nav::close_parallel_main(rw, "33"),
            "33",
            "unknown end: itself"
        );
        let egll = vec![
            nav::Runway {
                width_m: 50.0,
                surface: 0,
                ident_a: "09L".into(),
                ident_b: "27R".into(),
                lat_a: 51.4774781,
                lon_a: -0.4894423,
                lat_b: 51.4776753,
                lon_b: -0.4332600,
            },
            nav::Runway {
                width_m: 50.0,
                surface: 0,
                ident_a: "09R".into(),
                ident_b: "27L".into(),
                lat_a: 51.4647701,
                lon_a: -0.4867489,
                lat_b: 51.4649460,
                lon_b: -0.4340638,
            },
        ];
        assert_eq!(nav::close_parallel_main(&egll, "27L"), "27L");
        assert_eq!(nav::close_parallel_main(&egll, "09R"), "09R");
    }

    /// A CIFP directory with one SID from Gibraltar's runway 09.
    fn lxgb_cifp_dir() -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("xc-station-cifp-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            dir.join("LXGB.dat"),
            "SID:010,5,GOSA1C,RW09,GOS01,LX,P,C,E   , ,   ,CF, ,   ,  , , ,      ,    ,    ,0900,0050, ,     ,     ,     , ,   ,    ,   ;\n",
        )
        .unwrap();
        dir
    }

    fn edinburgh() -> RegionApt {
        RegionApt {
            icao: "EGPH".into(),
            apt: nav::AirportInfo {
                transition_level_ft: None,
                name: "Edinburgh".into(),
                elev_ft: 136.0,
                runways: Vec::new(),
                freqs: Vec::new(),
            },
            lat: 55.95,
            lon: -3.37,
        }
    }

    #[test]
    fn an_ifr_clearance_is_published_only_after_its_readback() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        Station::uninstall();
        *SQUAWK_WATCH.lock().unwrap() = None;
        *REGION.lock().unwrap() = vec![edinburgh()];
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 10.0,
        };
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.set_procedures_now(lxgb_cifp_dir().to_str().unwrap(), "", "", "LXGB");
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            true,
            false,
            None,
            false,
            None,
        );
        // A destination the index does not know: asked again, nothing issued.
        let r = st.route_keyed("gibraltar tower, request ifr clearance to atlantis", Service::Tower, &ctx, wx);
        assert_eq!(r, "November one seven two Sierra Papa, say again destination.");
        assert!(st.assigned_squawk.is_none(), "no clearance, no code");
        // Issued: the SID from the CIFP for the runway in use, 5,000 ft, the
        // radar field's discrete code. Not yet published.
        let r = st.route_keyed(
            "gibraltar tower, november one seven two sierra papa, request ifr clearance to edinburgh",
            Service::Tower,
            &ctx,
            wx,
        );
        let code = st.assigned_squawk.expect("a discrete code at a radar field");
        let spoken = atis::spoken_digits(&format!("{code:04}"));
        assert_eq!(
            r,
            format!(
                "November one seven two Sierra Papa, cleared to Edinburgh via the Gosa one Charlie \
                 departure, runway zero niner, climb five thousand feet, squawk {spoken}, QNH one \
                 zero one seven."
            )
        );
        assert_eq!(crate::party_line::assigned_clearance(), None, "issued is not read back");
        assert_eq!(crate::party_line::assigned_squawk(), Some(code));
        // A wrong altitude is corrected, and still nothing is published.
        let r = st.route_keyed(
            &format!("cleared to edinburgh via the gosa one charlie departure, climb six thousand feet, squawk {spoken}"),
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(r, "November one seven two Sierra Papa, negative, climb five thousand feet.");
        assert_eq!(crate::party_line::assigned_clearance(), None);
        // The coach's line, read back: silence, and the clearance is the crew's.
        let rb = crate::party_line::readback_owed().expect("the coach owes the readback");
        assert!(rb.starts_with("Cleared to Edinburgh via the Gosa one Charlie departure"), "{rb}");
        let r = st.route_keyed(&rb.to_lowercase(), Service::Tower, &ctx, wx);
        assert_eq!(r, "", "a correct readback earns silence");
        assert_eq!(
            crate::party_line::assigned_clearance(),
            Some(tower::IfrClearance {
                dest: "EGPH".into(),
                sid: Some("GOSA1C".into()),
                runway: "09".into(),
                initial_alt_ft: 5000,
                squawk: code,
            })
        );
        assert_eq!(crate::party_line::readback_owed(), None, "the debt is paid");
        assert_eq!(crate::party_line::ifr_flight_code(), Some(code), "the flight is IFR");
        // Retired with the squawk: a new field, and the world going away.
        st.install();
        assert_eq!(crate::party_line::assigned_clearance(), None, "install retires it");
        // The FLIGHT stays IFR across a region's install: the handoff that
        // follows it must keep its code.
        assert_eq!(crate::party_line::ifr_flight_code(), Some(code), "install keeps the IFR flight");
        *IFR_CLEARANCE.lock().unwrap() = Some(tower::IfrClearance {
            dest: "EGPH".into(),
            sid: None,
            runway: "09".into(),
            initial_alt_ft: 5000,
            squawk: code,
        });
        Station::uninstall();
        assert_eq!(crate::party_line::assigned_clearance(), None, "uninstall retires it");
        assert_eq!(crate::party_line::ifr_flight_code(), None, "uninstall ends the IFR flight");
        *REGION.lock().unwrap() = Vec::new();
        *SQUAWK_WATCH.lock().unwrap() = None;
    }

    /// Issue #96: a destination beyond the region index (no staffed tower)
    /// is cleared by its ICAO code from the whole-file directory; "cancel
    /// IFR" retires the clearance and the flight's IFR standing, and a VFR
    /// flight asking is told it has none.
    #[test]
    fn an_ifr_clearance_reaches_any_field_and_cancel_ifr_ends_it() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        Station::uninstall();
        *SQUAWK_WATCH.lock().unwrap() = None;
        *REGION.lock().unwrap() = vec![edinburgh()];
        // Glenforsa, a Mull grass strip with no tower row: not in the region
        // index, in the directory by its code only.
        *FIELD_DIR.lock().unwrap() = vec![("EGHM".into(), "Glenforsa".into(), (56.52, -5.91))];
        assert_eq!(resolve_destination("EGHM").map(|r| r.0).as_deref(), Some("EGHM"));
        assert_eq!(resolve_destination("glenforsa"), None, "names only from the region index");
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 10.0,
        };
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.set_procedures_now(lxgb_cifp_dir().to_str().unwrap(), "", "", "LXGB");
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            true,
            false,
            None,
            false,
            None,
        );
        let cs = "November one seven two Sierra Papa";
        // Nothing to cancel yet.
        let r = st.route_keyed(
            "gibraltar tower, november one seven two sierra papa, cancel ifr",
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(r, format!("{cs}, roger, you are not on an IFR clearance."));
        let r = st.route_keyed(
            "gibraltar tower, november one seven two sierra papa, request ifr clearance to echo golf hotel mike",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.starts_with(&format!("{cs}, cleared to Glenforsa")), "{r}");
        let rb = crate::party_line::readback_owed().expect("owed");
        assert_eq!(st.route_keyed(&rb.to_lowercase(), Service::Tower, &ctx, wx), "");
        assert_eq!(
            crate::party_line::assigned_clearance().map(|c| c.dest).as_deref(),
            Some("EGHM")
        );
        assert!(crate::party_line::ifr_flight_code().is_some());
        assert!(st.ifr_runway.is_some(), "the SID's runway is frozen");
        // Cancelled: the clearance, the flight's IFR code and the frozen
        // runway all go.
        let r = st.route_keyed(
            "gibraltar tower, november one seven two sierra papa, cancelling i f r",
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(r, format!("{cs}, IFR flight cancelled."));
        assert_eq!(crate::party_line::assigned_clearance(), None);
        assert_eq!(crate::party_line::ifr_flight_code(), None);
        assert!(st.ifr_runway.is_none());
        // An owed readback is cancelled too, with nothing published after.
        st.route_keyed(
            "gibraltar tower, november one seven two sierra papa, request ifr clearance to edinburgh",
            Service::Tower,
            &ctx,
            wx,
        );
        let r = st.route_keyed(
            "november one seven two sierra papa, cancel ifr",
            Service::Tower,
            &ctx,
            wx,
        );
        assert_eq!(r, format!("{cs}, IFR flight cancelled."));
        assert!(crate::party_line::readback_owed().is_none_or(|r| !r.starts_with("Cleared to")));
        assert_eq!(crate::party_line::assigned_clearance(), None);
        Station::uninstall();
        *REGION.lock().unwrap() = Vec::new();
        *FIELD_DIR.lock().unwrap() = Vec::new();
        *SQUAWK_WATCH.lock().unwrap() = None;
    }

    #[test]
    fn an_ifr_clearance_with_enforcement_off_stands_on_any_readback() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        Station::uninstall();
        *REGION.lock().unwrap() = vec![edinburgh()];
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 10.0,
        };
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.set_enforce_readback(false);
        st.set_dialect(Dialect::Faa);
        let ctx = st.make_ctx(
            wx,
            "altimeter two niner niner two".into(),
            "November one seven two Sierra Papa".into(),
            true,
            false,
            None,
            false,
            None,
        );
        // No procedures set: FAA runway heading, the departure frequency.
        let r = st.route_keyed("request ifr clearance to egph", Service::Tower, &ctx, wx);
        assert!(r.contains("cleared to Edinburgh airport as filed, after departure fly runway heading, climb and maintain five thousand, departure frequency one three zero decimal four"), "{r}");
        // A mishear is still no readback: nothing stands.
        st.route_keyed("gibraltar tower, blah", Service::Tower, &ctx, wx);
        assert_eq!(crate::party_line::assigned_clearance(), None);
        // A readback with the wrong altitude is coached, not challenged, and stands.
        let r = st.route_keyed("cleared to edinburgh, climb and maintain six thousand, squawk 4521", Service::Tower, &ctx, wx);
        assert_eq!(r, "");
        assert!(crate::party_line::assigned_clearance().is_some());
        Station::uninstall();
        *REGION.lock().unwrap() = Vec::new();
        *SQUAWK_WATCH.lock().unwrap() = None;
    }

    /// Two SIDs off Gibraltar's 09 with opposite exits: NORTH1A to NRTHX
    /// (up the peninsula) and SOUTH1A to STHXX (across the strait). Each
    /// destination is cleared via the one that leaves toward it, whichever
    /// the file lists first, and the bearings behind the pick are kept for
    /// the blackbox.
    #[test]
    fn the_ifr_sid_is_the_one_that_leaves_toward_the_destination() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        Station::uninstall();
        *SQUAWK_WATCH.lock().unwrap() = None;
        let dir = std::env::temp_dir().join(format!("xc-station-sids-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let leg = |sid: &str, seq: u32, fix: &str| {
            format!(
                "SID:{seq:03},5,{sid},RW09,{fix},LX,E,A,E   , ,   ,TF, ,   ,  , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   ;\n"
            )
        };
        // The southbound SID first in the file: the old rule's pick.
        let cifp = [
            leg("SOUTH1A", 10, "SMIDX"),
            leg("SOUTH1A", 20, "STHXX"),
            leg("NORTH1A", 10, "NMIDX"),
            leg("NORTH1A", 20, "NRTHX"),
        ]
        .concat();
        std::fs::write(dir.join("LXGB.dat"), cifp).unwrap();
        let fixes = dir.join("earth_fix.dat");
        std::fs::write(
            &fixes,
            "I\n1200 Version\n\n 36.300 -5.300 NMIDX ENRT LX 2\n 37.500 -5.200 NRTHX ENRT LX 2\n \
             35.900 -5.400 SMIDX ENRT LX 2\n 34.800 -5.500 STHXX ENRT LX 2\n99\n",
        )
        .unwrap();
        let mut madrid = region_apt("Madrid Barajas", 40.47, -3.56);
        madrid.icao = "LEMD".into();
        let mut marrakech = region_apt("Marrakech Menara", 31.61, -8.04);
        marrakech.icao = "GMMX".into();
        *REGION.lock().unwrap() = vec![madrid, marrakech];
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 10.0,
        };
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.set_procedures_now(
            dir.to_str().unwrap(),
            fixes.to_str().unwrap(),
            "",
            "LXGB",
        );
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            true,
            false,
            None,
            false,
            None,
        );
        let r = st.route_keyed("request ifr clearance to lemd", Service::Tower, &ctx, wx);
        assert!(r.contains("via the North one Alpha departure, runway zero niner"), "{r}");
        let (exit, dest) = st.ifr_bearings.expect("bearings kept");
        assert!(exit < 10.0 || exit > 350.0, "north exit: {exit}");
        assert!((dest - 22.0).abs() < 10.0, "Madrid bearing: {dest}");
        // Another flight (a fresh station: this one is owed a readback).
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.set_procedures_now(
            dir.to_str().unwrap(),
            fixes.to_str().unwrap(),
            "",
            "LXGB",
        );
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            true,
            false,
            None,
            false,
            None,
        );
        let r = st.route_keyed("request ifr clearance to gmmx", Service::Tower, &ctx, wx);
        assert!(r.contains("via the South one Alpha departure"), "{r}");
        let (exit, dest) = st.ifr_bearings.expect("bearings kept");
        assert!((exit - 190.0).abs() < 15.0, "south exit: {exit}");
        assert!((dest - 212.0).abs() < 10.0, "Marrakech bearing: {dest}");
        // No positioned fixes: the first in the file, as before.
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.set_procedures_now(dir.to_str().unwrap(), "", "", "LXGB");
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            true,
            false,
            None,
            false,
            None,
        );
        let r = st.route_keyed("request ifr clearance to lemd", Service::Tower, &ctx, wx);
        assert!(r.contains("via the South one Alpha departure"), "{r}");
        assert_eq!(st.ifr_bearings, None);
        Station::uninstall();
        *REGION.lock().unwrap() = Vec::new();
        *SQUAWK_WATCH.lock().unwrap() = None;
    }

    fn filed_lxgb_lemd_via_south() -> Option<crate::party_line::FiledRoute> {
        Some(crate::party_line::FiledRoute {
            origin: "LXGB".into(),
            dest: "LEMD".into(),
            fixes: vec!["SMIDX".into(), "STHXX".into(), "ENRTE".into()],
            sid: None,
            cruise_ft: None,
        })
    }

    fn filed_lxgb_lemd_via_nowhere() -> Option<crate::party_line::FiledRoute> {
        Some(crate::party_line::FiledRoute {
            origin: "LXGB".into(),
            dest: "LEMD".into(),
            fixes: vec!["ENRTE".into()],
            sid: None,
            cruise_ft: None,
        })
    }

    /// #152 (EGKK 08R, sortie 1790601822): the tower cleared the SID whose
    /// exit lay toward the destination, KENE3P, for a route filed via LAM,
    /// and the crew flew LAM5P. With the crew's FILED route the IFR
    /// clearance names the SID that joins it (`nav::sid_for_filed_route`,
    /// the crew's rule): Madrid lies north, NORTH1A leaves north, and a
    /// route filed via STHXX is cleared via SOUTH1A. A filed route no SID
    /// joins is cleared runway heading; a plan to another destination is
    /// not this clearance's, and the bearing rule stands.
    #[test]
    fn the_ifr_sid_is_the_one_that_joins_the_filed_route() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        Station::uninstall();
        *SQUAWK_WATCH.lock().unwrap() = None;
        let dir = std::env::temp_dir().join(format!("xc-station-filed-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let leg = |sid: &str, seq: u32, fix: &str| {
            format!(
                "SID:{seq:03},5,{sid},RW09,{fix},LX,E,A,E   , ,   ,TF, ,   ,  , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   ;\n"
            )
        };
        let cifp = [
            leg("SOUTH1A", 10, "SMIDX"),
            leg("SOUTH1A", 20, "STHXX"),
            leg("NORTH1A", 10, "NMIDX"),
            leg("NORTH1A", 20, "NRTHX"),
        ]
        .concat();
        std::fs::write(dir.join("LXGB.dat"), cifp).unwrap();
        let fixes = dir.join("earth_fix.dat");
        std::fs::write(
            &fixes,
            "I\n1200 Version\n\n 36.300 -5.300 NMIDX ENRT LX 2\n 37.500 -5.200 NRTHX ENRT LX 2\n \
             35.900 -5.400 SMIDX ENRT LX 2\n 34.800 -5.500 STHXX ENRT LX 2\n99\n",
        )
        .unwrap();
        let mut madrid = region_apt("Madrid Barajas", 40.47, -3.56);
        madrid.icao = "LEMD".into();
        let mut marrakech = region_apt("Marrakech Menara", 31.61, -8.04);
        marrakech.icao = "GMMX".into();
        *REGION.lock().unwrap() = vec![madrid, marrakech];
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 10.0,
        };
        let path = lxgb_apt_file();
        let ask = |dest: &str| {
            let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
            st.set_procedures_now(dir.to_str().unwrap(), fixes.to_str().unwrap(), "", "LXGB");
            let ctx = st.make_ctx(
                wx,
                "QNH one zero one seven".into(),
                "November one seven two Sierra Papa".into(),
                true,
                false,
                None,
                false,
                None,
            );
            let r = st.route_keyed(&format!("request ifr clearance to {dest}"), Service::Tower, &ctx, wx);
            (r, st.ifr_sid_basis.clone())
        };
        *crate::party_line::FILED_ROUTE.lock().unwrap() = Some(filed_lxgb_lemd_via_south);
        let (r, basis) = ask("lemd");
        assert!(r.contains("via the South one Alpha departure"), "{r}");
        assert_eq!(basis, Some(SidBasis::Route { join: "STHXX".into() }));
        // A plan to Madrid says nothing about a clearance to Marrakech.
        let (r, basis) = ask("gmmx");
        assert!(r.contains("via the South one Alpha departure"), "{r}");
        assert_eq!(basis, Some(SidBasis::Bearing));
        *crate::party_line::FILED_ROUTE.lock().unwrap() = Some(filed_lxgb_lemd_via_nowhere);
        let (r, basis) = ask("lemd");
        assert!(r.contains("runway heading"), "{r}");
        assert!(!r.contains("One Alpha") && !r.contains("one Alpha"), "{r}");
        assert_eq!(basis, Some(SidBasis::NoneJoins));
        // No provider: the bearing rule, as before.
        *crate::party_line::FILED_ROUTE.lock().unwrap() = None;
        let (r, basis) = ask("lemd");
        assert!(r.contains("via the North one Alpha departure"), "{r}");
        assert_eq!(basis, Some(SidBasis::Bearing));
        Station::uninstall();
        *REGION.lock().unwrap() = Vec::new();
        *SQUAWK_WATCH.lock().unwrap() = None;
    }

    /// #152: a crew that cannot fly the cleared SID asks for another by
    /// name. One off the cleared runway is granted and the held clearance
    /// names it (the crew waits on `assigned_clearance`); one the field does
    /// not publish off that runway is refused and the cleared one stands; a
    /// flight with no IFR clearance is told to ask for one.
    #[test]
    fn a_sid_asked_for_by_name_amends_the_clearance_or_the_cleared_one_stands() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        Station::uninstall();
        *SQUAWK_WATCH.lock().unwrap() = None;
        let dir = std::env::temp_dir().join(format!("xc-station-amend-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let leg = |sid: &str, seq: u32, fix: &str| {
            format!(
                "SID:{seq:03},5,{sid},RW09,{fix},LX,E,A,E   , ,   ,TF, ,   ,  , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   ;\n"
            )
        };
        let cifp = [leg("SOUTH1A", 10, "STHXX"), leg("NORTH1A", 10, "NRTHX")].concat();
        std::fs::write(dir.join("LXGB.dat"), cifp).unwrap();
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 10.0,
        };
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.set_procedures_now(dir.to_str().unwrap(), "", "", "LXGB");
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            true,
            false,
            None,
            false,
            None,
        );
        let ask = "November one seven two Sierra Papa, unable North one Alpha, request the South one Alpha departure";
        *IFR_CLEARANCE.lock().unwrap() = None;
        let r = st.route_keyed(ask, Service::Tower, &ctx, wx);
        assert!(r.contains("request your IFR clearance first"), "{r}");
        *IFR_CLEARANCE.lock().unwrap() = Some(tower::IfrClearance {
            dest: "LEMD".into(),
            sid: Some("NORTH1A".into()),
            runway: "09".into(),
            initial_alt_ft: 5000,
            squawk: 4521,
        });
        let r = st.route_keyed(
            "November one seven two Sierra Papa, request the Kene three Papa departure",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("negative") && r.contains("North one Alpha departure stands"), "{r}");
        assert_eq!(
            crate::party_line::assigned_clearance().and_then(|c| c.sid).as_deref(),
            Some("NORTH1A")
        );
        let r = st.route_keyed(ask, Service::Tower, &ctx, wx);
        assert!(r.contains("South one Alpha departure approved, runway"), "{r}");
        assert_eq!(
            crate::party_line::assigned_clearance().and_then(|c| c.sid).as_deref(),
            Some("SOUTH1A")
        );
        Station::uninstall();
        *SQUAWK_WATCH.lock().unwrap() = None;
    }

    #[test]
    fn ifr_destinations_resolve_by_code_or_name_and_never_by_guess() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut ord = region_apt("Chicago O'Hare Intl", 41.97, -87.9);
        ord.icao = "KORD".into();
        let mut mdw = region_apt("Chicago Midway Intl", 41.78, -87.75);
        mdw.icao = "KMDW".into();
        *REGION.lock().unwrap() = vec![edinburgh(), ord, mdw];
        assert_eq!(resolve_destination("EGPH").map(|r| r.0).as_deref(), Some("EGPH"));
        assert_eq!(resolve_destination("egph").map(|r| r.0).as_deref(), Some("EGPH"));
        assert_eq!(resolve_destination("ORD").map(|r| r.0).as_deref(), Some("KORD"), "the K a US pilot drops");
        assert_eq!(resolve_destination("edinburgh").map(|r| r.0).as_deref(), Some("EGPH"));
        assert_eq!(resolve_destination("midway").map(|r| r.0).as_deref(), Some("KMDW"));
        assert_eq!(resolve_destination("chicago"), None, "two Chicagos: no guess");
        assert_eq!(resolve_destination("atlantis"), None);
        *REGION.lock().unwrap() = Vec::new();
        assert_eq!(resolve_destination("EGPH"), None, "no index, nothing known");
    }

    #[test]
    fn lxgb_station_loads_from_real_apt_rows() {
        let path = lxgb_apt_file();
        let st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        assert_eq!(st.apt.name, "Gibraltar");
        assert_eq!(st.tower_khz, 131200);
        assert_eq!(
            st.approach_khz,
            Some(122800),
            "approach row wired for vectors"
        );
        // One physical runway, both ends named.
        assert_eq!(st.apt.runways.len(), 1);
        let rw = &st.apt.runways[0];
        assert_eq!((rw.ident_a.as_str(), rw.ident_b.as_str()), ("09", "27"));
        // The 09 end points roughly east — the vectors' final approach course.
        assert!(
            (rw.heading_a_deg() - 88.0).abs() < 5.0,
            "{}",
            rw.heading_a_deg()
        );
        // Departures: the into-wind pick from the real geometry. Gibraltar's
        // prevailing westerly means 27; an easterly Levanter flips it to 09.
        let (id, _, hw) = nav::best_runway_for_wind(&st.apt.runways, 250.0, 12.0).unwrap();
        assert_eq!((id.as_str(), hw > 0.0), ("27", true));
        assert_eq!(
            nav::best_runway_for_wind(&st.apt.runways, 90.0, 10.0)
                .unwrap()
                .0,
            "09"
        );
        // Every service the field actually has resolves by name; ATIS (which
        // LXGB's apt.dat doesn't list) declines instead of guessing.
        assert!(matches!(
            st.tune_request("contact departure"),
            Some(Tune::Found { khz: 130400, .. })
        ));
        assert!(matches!(
            st.tune_request("tune the atis"),
            Some(Tune::Missing(_))
        ));
    }

    /// The whole LXGB arrival, flown: vectors from north of the Rock onto
    /// final for 09, the visual-approach clearance + tower handoff once
    /// established, then the tower's landing clearance off the "final" report.
    /// One test on purpose — `VECTORS` is a process-wide static and parallel
    /// tests would clobber each other's plan.
    #[test]
    fn lxgb_vectored_arrival_flies_to_a_landing_clearance() {
        use std::time::Duration;

        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let rw = st.apt.runways[0].clone();
        let (thr_lat, thr_lon, course) = (rw.lat_a, rw.lon_a, rw.heading_a_deg());

        // Arm the plan exactly as arm_vectors would for runway 09.
        let target_alt_ft = ((st.apt.elev_ft + 1500.0) / 100.0).round() * 100.0;
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: "November one seven two Sierra Papa".into(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
            target_alt_ft,
            ils: false,
            join_nm: 6.0,
            cleared: false,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });

        // Regression: crossing final perpendicular inside the wedge must NOT
        // be "established" — position said cleared, the track says crossing.
        let t0 = std::time::Instant::now();
        let (x_lat, x_lon) = crate::nav::project_latlon(thr_lat, thr_lon, course + 180.0, 4.0);
        let crossing = vector_tick(x_lat, x_lon, 1600.0, Some(0.0), false, t0);
        if let Some(msg) = &crossing {
            assert!(
                !msg.contains("Cleared"),
                "cleared while crossing final: {msg}"
            );
        }

        // Now the real arrival: 10 nm north of the field at 3,000 ft. The sim
        // obeys each assigned heading at 100 kts in 10 s ticks, descending to
        // the assigned altitude — a pilot doing exactly as told, nothing more.
        VECTORS.lock().unwrap().as_mut().unwrap().last_tx = None;
        VECTORS.lock().unwrap().as_mut().unwrap().last_heading = None;
        let (mut lat, mut lon) = crate::nav::project_latlon(thr_lat, thr_lon, 0.0, 10.0);
        let mut alt = 3000.0_f64;
        let mut hdg = 180.0_f64;
        let mut first: Option<String> = None;
        let mut cleared: Option<String> = None;
        for step in 0..240u64 {
            let now = t0 + Duration::from_secs(step * 10);
            if let Some(msg) = vector_tick(lat, lon, alt, Some(hdg), false, now) {
                if msg.contains("Cleared visual approach") {
                    cleared = Some(msg);
                    break;
                }
                first.get_or_insert(msg);
            }
            if let Some(h) = VECTORS
                .lock()
                .unwrap()
                .as_ref()
                .and_then(|p| p.last_heading)
            {
                hdg = h as f64; // instant turn: worst case for the capture curve
            }
            let (nlat, nlon) = crate::nav::project_latlon(lat, lon, hdg, 100.0 * 10.0 / 3600.0);
            (lat, lon) = (nlat, nlon);
            alt = (alt - 80.0).max(target_alt_ft);
        }

        let first = first.expect("an opening vector");
        assert!(first.contains("fly heading"), "{first}");
        // Gibraltar speaks ICAO/CAP 413: altitudes are "descend to altitude
        // ... feet", and headings carry "degrees".
        assert!(
            first.contains("descend to altitude one thousand five hundred feet"),
            "{first}"
        );
        assert!(first.contains("degrees"), "{first}");
        let cleared = cleared.expect("never established on final in 40 sim-minutes");
        assert!(cleared.contains("runway zero niner"), "{cleared}");
        assert!(
            cleared.contains("Contact Tower one three one decimal two"),
            "{cleared}"
        );
        assert!(
            VECTORS.lock().unwrap().is_none(),
            "plan must be wiped once approach's job is done"
        );

        // The handoff lands on the tower: "final" alone earns the clearance.
        let clear = st.twr.handle(
            tower::parse_intent("november one seven two sierra papa, final"),
            &tower::Ctx {
                callsign: "November one seven two Sierra Papa".into(),
                airport: "Gibraltar".into(),
                runway: "zero niner".into(),
                qnh: "QNH one zero one seven".into(),
                wind: "calm".into(),
                holding_point: "Alpha".into(),
                stand: String::new(),
                approach: Some(atis::spoken_mhz(122800)),
                squawk: "seven zero zero zero".into(),
                final_traffic_nm: None,
                final_traffic_callsign: None,
                final_traffic_count: 0,
                runway_occupied: false,
                own_on_runway: false,
                helicopter: false,
                ifr: None,
                circuit: None,
                runway_refusal: None,
                transition_level_ft: None,
                wake_secs_left: None,
                taxi_via: Vec::new(),
                taxi_crossings: Vec::new(),
                on_ground: false,
            },
        );
        assert!(clear.contains("cleared to land"), "{clear}");

        // Touchdown: the on-ground tick retires any leftover plan quietly.
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: String::new(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: String::new(),
            tower_spoken: None,
            target_alt_ft,
            ils: false,
            join_nm: 6.0,
            cleared: false,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });
        assert!(vector_tick(thr_lat, thr_lon, 15.0, None, true, t0).is_none());
        assert!(
            VECTORS.lock().unwrap().is_none(),
            "on-ground tick clears the plan"
        );

        // --- The same arrival flown as an ILS. The clearance rides the
        // intercept turn ("cleared ILS approach..., report established"),
        // headings carry "degrees" (ICAO dialect), and the established
        // wedge hands off WITHOUT re-clearing.
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: "November one seven two Sierra Papa".into(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
            target_alt_ft,
            ils: true,
            join_nm: 6.0,
            cleared: false,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });
        let (mut lat, mut lon) = crate::nav::project_latlon(thr_lat, thr_lon, 0.0, 10.0);
        let mut alt = 3000.0_f64;
        let mut hdg = 180.0_f64;
        let mut cleared_ils: Option<String> = None;
        let mut handoff: Option<String> = None;
        // Orbit detector: consecutive vectors must never reverse the
        // aircraft (the pursuit-curve wind-up commanded 250 then 010 at
        // Jerez, 2026-07-12 — an orbit, not an intercept).
        let mut prev_cmd: Option<i64> = None;
        for step in 0..240u64 {
            let now = t0 + Duration::from_secs(7200 + step * 10);
            if let Some(msg) = vector_tick(lat, lon, alt, Some(hdg), false, now) {
                if msg.contains("fly heading") {
                    assert!(
                        msg.contains("degrees"),
                        "ICAO headings carry the unit: {msg}"
                    );
                }
                if msg.contains("Cleared ILS approach") {
                    cleared_ils.get_or_insert(msg.clone());
                }
                if msg.contains("Contact Tower") {
                    handoff = Some(msg);
                    break;
                }
            }
            if let Some(h) = VECTORS
                .lock()
                .unwrap()
                .as_ref()
                .and_then(|p| p.last_heading)
            {
                if let Some(p) = prev_cmd {
                    assert!(
                        angle_diff_deg(h as f64, p as f64).abs() <= 120.0,
                        "orbit: vector {p} then {h}"
                    );
                }
                prev_cmd = Some(h);
                hdg = h as f64;
            }
            let (nlat, nlon) = crate::nav::project_latlon(lat, lon, hdg, 100.0 * 10.0 / 3600.0);
            (lat, lon) = (nlat, nlon);
            alt = (alt - 80.0).max(target_alt_ft);
        }
        let cleared_ils = cleared_ils.expect("ILS clearance never issued");
        assert!(
            cleared_ils.contains("report established"),
            "the intercept clearance asks for the report: {cleared_ils}"
        );
        let handoff = handoff.expect("no handoff after establishing");
        assert!(
            !handoff.contains("Cleared"),
            "already cleared on the intercept, don't re-clear: {handoff}"
        );
        assert!(
            VECTORS.lock().unwrap().is_none(),
            "ILS plan wiped after handoff"
        );

        // --- Close-in arrival (the Jerez 2026-07-12 geometry): already at
        // platform altitude only 4 nm from the threshold, a mile east of the
        // centerline. The old pursuit curve orbited here; the fixed cut must
        // walk it onto final and hand off without ever reversing a vector.
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: "November one seven two Sierra Papa".into(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
            target_alt_ft,
            ils: true,
            join_nm: 8.6,
            cleared: false,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });
        let recip = (course + 180.0).rem_euclid(360.0);
        let (mut lat, mut lon) = {
            let (a, b) = crate::nav::project_latlon(thr_lat, thr_lon, recip, 4.0);
            crate::nav::project_latlon(a, b, (recip + 90.0).rem_euclid(360.0), 1.0)
        };
        let mut hdg = course;
        let mut alt = target_alt_ft;
        let mut prev_cmd: Option<i64> = None;
        let mut done: Option<String> = None;
        for step in 0..240u64 {
            let now = t0 + Duration::from_secs(14_400 + step * 10);
            if let Some(msg) = vector_tick(lat, lon, alt, Some(hdg), false, now) {
                if msg.contains("Contact Tower") {
                    done = Some(msg);
                    break;
                }
            }
            if let Some(h) = VECTORS
                .lock()
                .unwrap()
                .as_ref()
                .and_then(|p| p.last_heading)
            {
                if let Some(p) = prev_cmd {
                    assert!(
                        angle_diff_deg(h as f64, p as f64).abs() <= 120.0,
                        "close-in orbit: vector {p} then {h}"
                    );
                }
                prev_cmd = Some(h);
                hdg = h as f64;
            }
            let (nlat, nlon) = crate::nav::project_latlon(lat, lon, hdg, 100.0 * 10.0 / 3600.0);
            (lat, lon) = (nlat, nlon);
            alt = (alt - 80.0).max(target_alt_ft);
        }
        done.expect("close-in arrival never handed off");
        assert!(VECTORS.lock().unwrap().is_none());

        // --- The tight base (Gibraltar 2026-07-13): the pilot rolls out on
        // final at 0.86 nm, below the vector altitude, pointed down the
        // runway. The old 1.0 nm wedge floor never saw it — the plan
        // vectored the short-final aircraft away and touchdown discarded it
        // with no clearance ever spoken, earning a compliant pilot the
        // landing berate. Inside a mile, landing intent IS established.
        let arm_short_final = || {
            *VECTORS.lock().unwrap() = Some(VectorPlan {
                callsign: "November one seven two Sierra Papa".into(),
                thr_lat,
                thr_lon,
                course_deg: course,
                runway_spoken: "zero niner".into(),
                tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
                target_alt_ft,
                ils: false,
                join_nm: 6.0,
                cleared: false,
                dialect: Dialect::Icao,
                last_tx: None,
                last_heading: None,
            });
        };
        arm_short_final();
        let (sf_lat, sf_lon) = crate::nav::project_latlon(thr_lat, thr_lon, recip + 3.0, 0.86);
        let short_final = vector_tick(
            sf_lat,
            sf_lon,
            target_alt_ft - 950.0,
            Some(course + 12.0),
            false,
            t0,
        )
        .expect("short final earns the clearance, not a vector away");
        assert!(
            short_final.contains("Cleared visual approach runway zero niner"),
            "{short_final}"
        );
        assert!(
            short_final.contains("1 mile from touchdown"),
            "{short_final}"
        );
        assert!(short_final.contains("Contact Tower"), "{short_final}");
        assert!(
            VECTORS.lock().unwrap().is_none(),
            "plan done after the clearance"
        );

        // Inside a mile but NOT landing — crossing final perpendicular, and a
        // flyover at altitude — gets silence, never a reversal heading, and
        // the plan survives for when the geometry recovers.
        arm_short_final();
        let (x_lat, x_lon) = crate::nav::project_latlon(thr_lat, thr_lon, recip, 0.5);
        let crossing_track = (course + 90.0).rem_euclid(360.0);
        assert!(
            vector_tick(
                x_lat,
                x_lon,
                target_alt_ft - 950.0,
                Some(crossing_track),
                false,
                t0
            )
            .is_none(),
            "no vector inside a mile"
        );
        assert!(
            vector_tick(x_lat, x_lon, target_alt_ft + 500.0, Some(course), false, t0).is_none(),
            "a flyover at altitude is not established"
        );
        assert!(
            VECTORS.lock().unwrap().is_some(),
            "plan survives the blow-through"
        );
        *VECTORS.lock().unwrap() = None;

        // --- The pilot who reports "established" earns the handoff from the
        // approach controller directly, and the plan dies with it.
        let ctx2 = tower::Ctx {
            callsign: "November one seven two Sierra Papa".into(),
            airport: "Gibraltar".into(),
            runway: "zero niner".into(),
            qnh: "QNH one zero one seven".into(),
            wind: "calm".into(),
            holding_point: "Alpha".into(),
            stand: String::new(),
            approach: Some(atis::spoken_mhz(122800)),
            squawk: "seven zero zero zero".into(),
            final_traffic_nm: None,
            final_traffic_callsign: None,
            final_traffic_count: 0,
            runway_occupied: false,
            own_on_runway: false,
            helicopter: false,
            ifr: None,
            circuit: None,
            runway_refusal: None,
            transition_level_ft: None,
            wake_secs_left: None,
            taxi_via: Vec::new(),
            taxi_crossings: Vec::new(),
            on_ground: false,
        };
        let wx2 = Wx {
            wind_from_deg: 90.0,
            wind_kts: 8.0,
        };
        *VECTORS.lock().unwrap() = Some(VectorPlan {
            callsign: "November one seven two Sierra Papa".into(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
            target_alt_ft,
            ils: true,
            join_nm: 6.0,
            cleared: true,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });
        let r = st.route_keyed(
            "gibraltar approach, established i l s runway zero niner",
            Service::Approach,
            &ctx2,
            wx2,
        );
        assert!(
            r.contains("Contact Gibraltar Tower"),
            "established → handoff: {r}"
        );
        assert!(VECTORS.lock().unwrap().is_none(), "report kills the plan");

        // --- Arm phrasing per dialect, with real chart data on file: the
        // CIFP platform (2,000 ft here) outranks the geometric default.
        *ILS_FACTS.lock().unwrap() = Some(std::sync::Arc::new(IlsFacts {
            ils: vec![nav::AirportIls {
                runway: "09".into(),
                freq_10khz: 11030,
                course_mag: 90.0,
                gs_deg: Some(3.0),
            }],
            platform_ft: [("09".to_string(), 2000.0)].into_iter().collect(),
        }));
        let r = st.route_keyed("request vectors", Service::Approach, &ctx2, wx2);
        assert!(
            r.contains("Vectoring ILS approach runway zero niner"),
            "{r}"
        );
        assert!(
            r.contains("Descend to altitude two thousand feet"),
            "CIFP platform: {r}"
        );
        st.set_dialect(Dialect::Faa);
        let r = st.route_keyed("request vectors", Service::Approach, &ctx2, wx2);
        assert!(
            r.contains("Expect vectors ILS runway zero niner approach"),
            "{r}"
        );
        assert!(r.contains("Descend and maintain two thousand"), "{r}");

        // Statics must not leak into other tests.
        *VECTORS.lock().unwrap() = None;
        *ILS_FACTS.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
    }

    #[test]
    fn ils_join_sits_under_the_glideslope() {
        // Heathrow: 2,500 ft platform over an 83 ft field, 3° slope — the
        // slope reaches ~2,417 AAL at ~7.6 nm, so the join must be past it.
        let join = ils_join_nm(2417.0, 3.0);
        assert!((8.0..10.0).contains(&join), "{join}");
        // At the join, the slope is ABOVE the platform: intercept from below.
        let slope_at_join_ft = (join - 1.0) * 6076.12 * 3.0_f64.to_radians().tan();
        assert!(slope_at_join_ft >= 2417.0 - 1.0, "{slope_at_join_ft}");
        // A low platform never pulls the join inside the visual's 6 nm.
        assert_eq!(ils_join_nm(800.0, 3.0), 6.0);
        // A degenerate slope angle can't divide toward infinity.
        assert!(ils_join_nm(2000.0, 0.0).is_finite());
    }

    #[test]
    fn phraseology_follows_geography() {
        assert_eq!(dialect_for("EGLL"), Dialect::Icao);
        assert_eq!(dialect_for("LXGB"), Dialect::Icao);
        assert_eq!(dialect_for("LEMD"), Dialect::Icao);
        assert_eq!(dialect_for("KORD"), Dialect::Faa);
        assert_eq!(dialect_for("PHNL"), Dialect::Faa);
        assert_eq!(dialect_for("CYYZ"), Dialect::Faa);
        // The pressure phrase speaks the field's own money: 1013.25 hPa is
        // "QNH one zero one three" in ICAO land and exactly 29.92 inches —
        // "altimeter two niner niner two" — at an FAA field.
        assert_eq!(
            spoken_pressure(101325.0, Dialect::Icao),
            "QNH one zero one three"
        );
        assert_eq!(
            spoken_pressure(101325.0, Dialect::Faa),
            "altimeter two niner niner two"
        );
        assert_eq!(
            spoken_pressure(102000.0, Dialect::Icao),
            "QNH one zero two zero"
        );
        // 1020 hPa ≈ 30.12 inHg.
        assert_eq!(
            spoken_pressure(102000.0, Dialect::Faa),
            "altimeter three zero one two"
        );
    }

    /// EDLP 24 (SF50, 2026-09-24): the vacate timed out 714 m short of exit
    /// C and the stand route was planned from the runway, 51.613882 N
    /// 8.615681 E heading 236 (state record at t 1790239644.0). It started
    /// back up the strip (the drive joined 179 degrees off) and the
    /// clearance said "cross runway zero six" for the runway we were on.
    /// Planned from the vacate it runs forward and crosses nothing of ours.
    /// `cargo test -p atc -- --ignored live_edlp_stand_route --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn live_edlp_stand_route_from_the_runway_runs_forward() {
        let apt = crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let Ok(Some(net)) = nav::find_taxi_net(&apt, "EDLP") else {
            return; // no install, no verdict
        };
        let runways = nav::find_runways(&apt, "EDLP").unwrap();
        let (lat, lon) = (51.613882, 8.615681);
        let strip = runways
            .iter()
            .find(|r| on_runway(r, lat, lon).is_some())
            .expect("the stop is on a runway");
        // The rollout direction: the end heading within 30 degrees of 236.
        let psi = [strip.heading_a_deg(), strip.heading_b_deg()]
            .into_iter()
            .find(|h| nav::angle_diff_deg(*h, 236.0).abs() < 30.0)
            .expect("the landing end");
        let full = format!("{}/{}", strip.ident_a, strip.ident_b);
        // Which way the route first goes: the bearing to its point 40 m on.
        let first_brg = |p: &nav::TaxiRoutePlan| {
            let mut run = 0.0;
            for w in p.path.windows(2) {
                run += nav::haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0;
                if run >= 40.0 {
                    return nav::bearing_deg(lat, lon, w[1].0, w[1].1);
                }
            }
            f64::NAN
        };
        // Without the heading: the old plan, from the nearest node.
        let old = stand_route_on(&net, &runways, lat, lon, None).expect("a route");
        println!(
            "old: via {:?} crossings {:?}, first leg {:.0} deg",
            old.via,
            old.crossings,
            first_brg(&old)
        );
        let new = stand_route_on(&net, &runways, lat, lon, Some(psi)).expect("a route");
        println!(
            "new: via {:?} crossings {:?}, first leg {:.0} deg, heading {psi:.0}",
            new.via,
            new.crossings,
            first_brg(&new)
        );
        assert_eq!(new.seg.len() + 1, new.path.len(), "seg stays aligned");
        assert!(
            nav::angle_diff_deg(first_brg(&new), psi).abs() < 45.0,
            "the stand route leaves the runway forward: {:.0} against {psi:.0}",
            first_brg(&new)
        );
        assert!(
            !new.crossings.contains(&full),
            "the strip we are on is not a crossing: {:?}",
            new.crossings
        );
    }

    /// LEMG 31 (737, 2026-09-28, issue #141): the rollout stopped at
    /// 36.674101 N 4.497971 W heading 311 true (state at t 1790585617.7;
    /// sortie 1790585651 stopped at 36.674500 N 4.498537 W, t 1790586584.6),
    /// 1.1 m off the 13/31 centreline and 1,262 m off 12/30. The strip under
    /// the aircraft is 13/31, the vacate starts AT the aircraft and runs
    /// forward, and the stand route from there leaves 13/31 forward without
    /// calling it a crossing. The "12/30" of the recorded
    /// `stand_route_from_runway` was the RIGHT answer for where the aircraft
    /// then was: the sortie runner had reloaded it onto 12/30's centreline
    /// (36.685320 N 4.494170 W, 0.0 m off 12/30, 1,158 m off 13/31) 0.3 s
    /// after the landing's verdict, so it is pinned here too.
    /// `cargo test -p atc -- --ignored live_lemg_31 --nocapture`
    #[test]
    #[ignore] // needs the real X-Plane install; run with --ignored
    fn live_lemg_31_rollout_vacates_forward_off_the_strip_under_it() {
        let apt =
            crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let Ok(Some(net)) = nav::find_taxi_net(&apt, "LEMG") else {
            return; // no install, no verdict
        };
        let runways = nav::find_runways(&apt, "LEMG").unwrap();
        let strip_under = |lat: f64, lon: f64| -> Vec<String> {
            runways
                .iter()
                .filter(|r| on_runway(r, lat, lon).is_some())
                .map(|r| format!("{}/{}", r.ident_a, r.ident_b))
                .collect()
        };
        // The reload spot: 12/30 and nothing else.
        assert_eq!(
            strip_under(36.68532005, -4.49417),
            vec!["12/30".to_string()]
        );
        let psi = 311.7; // 31's true course; the rollout read 310.9 to 311.4
        for (lat, lon) in [
            (36.67410120039185, -4.49797079757389),
            (36.674499892604075, -4.498536580519237),
        ] {
            assert_eq!(
                strip_under(lat, lon),
                vec!["13/31".to_string()],
                "the rollout is on 13/31"
            );
            let zone = vec!["31".to_string(), "13".to_string()];
            let v = net
                .plan_vacate(lat, lon, psi, &zone, nav::VACATE_CLEAR_M)
                .expect("a way off 31 ahead");
            let start_m = nav::haversine_nm(lat, lon, v.path[0].0, v.path[0].1) * 1852.0;
            println!(
                "vacate via {:?}: {} pts, starts {start_m:.1} m from the aircraft",
                v.exit_name,
                v.path.len()
            );
            assert!(
                start_m < 5.0,
                "the vacate starts at the aircraft: {start_m:.1} m"
            );
            // Forward: the first point 40 m on is ahead of abeam.
            let mut run = 0.0;
            let mut ahead = None;
            for w in v.path.windows(2) {
                run += nav::haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0;
                if run >= 40.0 {
                    ahead = Some(nav::bearing_deg(lat, lon, w[1].0, w[1].1));
                    break;
                }
            }
            let brg = ahead.expect("a vacate longer than 40 m");
            assert!(
                nav::angle_diff_deg(brg, psi).abs() < 90.0,
                "the vacate runs forward: {brg:.0} against {psi:.0}"
            );
            // Ends OFF 13/31 (and off every strip).
            let end = *v.path.last().unwrap();
            assert!(
                strip_under(end.0, end.1).is_empty(),
                "the vacate ends off the strips: {end:?}"
            );
            // The stand route asked from here starts with that vacate, off
            // 13/31, forward, and never calls 13/31 a crossing.
            let plan = stand_route_on(&net, &runways, lat, lon, Some(psi)).expect("a stand route");
            let first_m = nav::haversine_nm(lat, lon, plan.path[0].0, plan.path[0].1) * 1852.0;
            println!(
                "stand {:?} via {:?} crossings {:?}, starts {first_m:.1} m off",
                plan.holding_point, plan.via, plan.crossings
            );
            assert!(
                first_m < 5.0,
                "the stand route starts at the aircraft: {first_m:.1} m"
            );
            assert!(
                !plan.crossings.contains(&"13/31".to_string()),
                "the strip we are on is not a crossing: {:?}",
                plan.crossings
            );
        }
    }

    /// A point `along` metres down a runway end's heading from its
    /// threshold (negative: out on its final), `right` metres to its right.
    fn lemg_point(r: &nav::Runway, end_a: bool, along: f64, right: f64) -> (f64, f64) {
        let (lat0, lon0, lat1, lon1) = if end_a {
            (r.lat_a, r.lon_a, r.lat_b, r.lon_b)
        } else {
            (r.lat_b, r.lon_b, r.lat_a, r.lon_a)
        };
        let m_lon = 111_320.0 * lat0.to_radians().cos();
        let (bx, by) = ((lon1 - lon0) * m_lon, (lat1 - lat0) * 111_320.0);
        let len = (bx * bx + by * by).sqrt();
        let (ux, uy) = (bx / len, by / len);
        let (x, y) = (ux * along + uy * right, uy * along - ux * right);
        (lat0 + y / 111_320.0, lon0 + x / m_lon)
    }

    fn lemg_ghost(callsign: &str, at: (f64, f64), ele_ft: f64, on_ground: bool) -> TcasTarget {
        TcasTarget {
            callsign: callsign.into(),
            lat: at.0,
            lon: at.1,
            ele_ft,
            on_ground,
            vs_fpm: if on_ground { 0.0 } else { -700.0 },
        }
    }

    /// Issue #172, question 3: LEMG 12/30 and 13/31 are 14 degrees apart,
    /// 826 m at the 12 and 13 thresholds and 1,490 m at the far ends, so
    /// neither is the other's close parallel. 12's final crosses 13's
    /// extended centreline about 2 nm out: FR655 on 12's final at 2.08 nm
    /// (sortie 1790623420-full_auto, +157 s) sat 70 m off 13's line, inside
    /// its 500 m corridor, and was "traffic on 2 mile final" to a 737 lined
    /// up on 13. It is 12's traffic.
    #[test]
    fn a_lemg_12_arrival_is_not_traffic_for_13() {
        let rw = lemg_runways();
        let apart = strips_apart_m(&rw[0], &rw[1]);
        assert!((800.0..860.0).contains(&apart), "{apart}");
        assert!(apart >= nav::CLOSE_PARALLEL_M);
        assert_eq!(nav::close_parallel_main(&rw, "12"), "12");
        assert_eq!(nav::close_parallel_main(&rw, "13"), "13");
        let elev = 52.0;
        let fr655 =
            lemg_ghost("FR655", lemg_point(&rw[0], true, -3852.0, 0.0), elev + 660.0, false);
        let (_, cross, _) = centreline_place(&rw[1], fr655.lat, fr655.lon).unwrap();
        assert!(cross < CORRIDOR_HALF_M, "inside 13's corridor: {cross}");
        assert!(
            final_traffic_for(std::slice::from_ref(&fr655), &rw[1], &rw, elev).is_none(),
            "12's lander is not 13's traffic"
        );
        let on_12 = final_traffic_for(std::slice::from_ref(&fr655), &rw[0], &rw, elev)
            .expect("12's own traffic");
        assert!((on_12.nm - 2.08).abs() < 0.05, "{}", on_12.nm);
        // A lander on 13's own final at the same range stays 13's.
        let own = lemg_ghost("VY1", lemg_point(&rw[1], true, -3852.0, 0.0), elev + 660.0, false);
        assert!(final_traffic_for(std::slice::from_ref(&own), &rw[1], &rw, elev).is_some());
        // The calm-wind stream reads it as 12's too.
        assert_eq!(arrival_end(std::slice::from_ref(&fr655), &rw, elev).as_deref(), Some("12"));
        // A close parallel's stream is still one runway's: a strip 200 m off
        // 13 hands its lander to 13's corridor.
        let mut close = rw[1].clone();
        let (a, b) = (
            lemg_point(&rw[1], true, 0.0, -200.0),
            lemg_point(&rw[1], false, 0.0, 200.0),
        );
        close.ident_a = "13L".into();
        close.ident_b = "31R".into();
        (close.lat_a, close.lon_a, close.lat_b, close.lon_b) = (a.0, a.1, b.0, b.1);
        let pair = vec![rw[1].clone(), close.clone()];
        let on_close =
            lemg_ghost("VY2", lemg_point(&close, true, -3000.0, 0.0), elev + 500.0, false);
        assert!(final_traffic_for(std::slice::from_ref(&on_close), &rw[1], &pair, elev).is_some());
    }

    /// Issue #172, question 1, from the sortie's records: the 737 lined up
    /// on LEMG 13 45 m from the threshold was held (U25702 lined up ahead,
    /// FR655 on 12's final), promised a clearance after the landing traffic,
    /// and then held five minutes by the replay's 13 departures lining up in
    /// front of it. The hold is kept for what it was held FOR: the ghost
    /// ahead at the first tick leaves, a later ghost standing in front is a
    /// cut-in (blackbox `hold_cut_in`), 12's lander is never ours, and the
    /// promise is kept.
    #[test]
    fn a_held_runway_start_is_released_once_its_traffic_has_gone() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *WAKE.lock().unwrap() = None;
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        let rw = lemg_runways();
        *ACTIVE_RUNWAYS.lock().unwrap() = rw.clone();
        *CLEARANCE.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::LinedUp,
            callsign: "November eight one six November November".into(),
            runway_ident: "13".into(),
            pending_readback: None,
            holds_runway: true,
        });
        let t0 = Instant::now();
        let sec = std::time::Duration::from_secs;
        *TAKEOFF_HOLD.lock().unwrap() = Some(TakeoffHold {
            callsign: "November eight one six November November".into(),
            runway: "one three".into(),
            runway_ident: "13".into(),
            clear_since: None,
            armed_at: t0,
            logged_at: t0,
            lined_up: true,
            behind: false,
            held_for: None,
            cut_in_logged: Default::default(),
        });
        let elev = 52.0;
        let own = Some(lemg_point(&rw[1], true, 45.0, 0.0));
        let fr655_final =
            lemg_ghost("FR655", lemg_point(&rw[0], true, -3852.0, 0.0), elev + 660.0, false);
        let fr655_down = lemg_ghost("FR655", lemg_point(&rw[0], true, 900.0, 0.0), elev, true);
        let u25702 = lemg_ghost("U25702", lemg_point(&rw[1], true, 136.0, 0.0), elev, true);
        let ei887 = lemg_ghost("EI887", lemg_point(&rw[1], true, 140.0, 2.0), elev, true);
        let tick = |targets: &[TcasTarget], s: u64| {
            takeoff_hold_tick_from(targets, elev, true, own, t0 + sec(s))
        };
        // U25702 lined up ahead of us: held, whatever 12 is doing.
        assert!(tick(&[fr655_final.clone(), u25702.clone()], 0).is_none());
        assert!(tick(&[fr655_final.clone(), u25702.clone()], 30).is_none());
        // It rolls and goes; FR655 lands on 12; EI887 lines up in front.
        assert!(tick(&[fr655_down.clone(), ei887.clone()], 60).is_none(), "EI887 moving on");
        assert!(tick(&[fr655_down.clone(), ei887.clone()], 65).is_none());
        // Standing, EI887 is the replay's queue, not our traffic: the strip
        // is clear for the hold from here, and 15 s on the tower keeps its
        // word.
        assert!(tick(&[fr655_down.clone(), ei887.clone()], 71).is_none());
        let line = tick(&[fr655_down.clone(), ei887.clone()], 87).expect("the promised release");
        assert!(line.contains("cleared for takeoff"), "{line}");
        assert!(line.contains("one three"), "{line}");
        assert!(*PENDING_TAKEOFF_CLEARANCE.lock().unwrap());

        // Held from the taxiway (not lined up), a ghost on the strip is
        // traffic whatever its seed: the queue ahead of the hold line is
        // the stream we join.
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        *TAKEOFF_HOLD.lock().unwrap() = Some(TakeoffHold {
            callsign: "November eight one six November November".into(),
            runway: "one three".into(),
            runway_ident: "13".into(),
            clear_since: None,
            armed_at: t0,
            logged_at: t0,
            lined_up: false,
            behind: false,
            held_for: Some(Default::default()),
            cut_in_logged: Default::default(),
        });
        for s in [200, 230, 260] {
            assert!(
                takeoff_hold_tick_from(std::slice::from_ref(&ei887), elev, true, None, t0 + sec(s))
                    .is_none(),
                "held at +{s}"
            );
        }

        // Only the pavement AHEAD of our tail is in the way of the roll: from
        // an intersection 900 m down 13, a ghost standing 200 m behind us is
        // not traffic, one 200 m ahead is.
        let mid = Some(lemg_point(&rw[1], true, 900.0, 0.0));
        let behind = lemg_ghost("VY3", lemg_point(&rw[1], true, 700.0, 0.0), elev, true);
        let ahead = lemg_ghost("VY4", lemg_point(&rw[1], true, 1100.0, 0.0), elev, true);
        let now = t0 + sec(300);
        assert!(!departure_strip_picture(&rw[1], "13", &[behind.clone()], mid, None, now).blocked);
        assert!(departure_strip_picture(&rw[1], "13", &[ahead], mid, None, now).blocked);
        assert!(
            departure_strip_picture(&rw[1], "13", &[behind], None, None, now).blocked,
            "without our position every occupant counts"
        );
        // A holding point 90 m off the centreline is not the pavement.
        let hold_line = lemg_ghost("VY5", lemg_point(&rw[1], true, 60.0, 90.0), elev, true);
        assert!(!departure_strip_picture(&rw[1], "13", &[hold_line], own, None, now).blocked);

        *TAKEOFF_HOLD.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *PENDING_TAKEOFF_CLEARANCE.lock().unwrap() = false;
        ACTIVE_RUNWAYS.lock().unwrap().clear();
    }

    /// Issue #172: a RUNWAY START's first ready call is answered "hold
    /// position, traffic on the runway" from Parked (the tower moves it
    /// LinedUp in the same reply), and the release watch never armed: the
    /// crew nudged "ready for departure" every 40 s and was told "read
    /// back" three times. Now the watch arms, and "holding position" is the
    /// readback the tower takes.
    #[test]
    fn a_held_runway_start_arms_the_release_and_takes_the_hold_readback() {
        let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *WAKE.lock().unwrap() = None;
        *TAKEOFF_HOLD.lock().unwrap() = None;
        let path = lxgb_apt_file();
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        let wx = Wx {
            wind_from_deg: 90.0,
            wind_kts: 8.0,
        };
        let mut ctx = scripted_ctx(&mut st, wx);
        ctx.on_ground = true;
        ctx.own_on_runway = true;
        ctx.runway_occupied = true;
        let cs = "november one seven two sierra papa";
        assert_eq!(st.twr.state, tower::State::Parked);
        let r = st.route_keyed(
            &format!("{cs}, ready for departure, for circuits"),
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(r.contains("hold position, traffic on the runway"), "{r}");
        assert_eq!(st.twr.state, tower::State::LinedUp);
        assert!(
            TAKEOFF_HOLD.lock().unwrap().as_ref().is_some_and(|h| h.lined_up),
            "the release watch is armed for the runway start"
        );
        let owed = CLEARANCE
            .lock()
            .unwrap()
            .as_ref()
            .and_then(|c| c.pending_readback.clone())
            .expect("the hold is owed its readback");
        assert!(owed.starts_with("Holding position"), "{owed}");
        // The crew's readback, as full auto transmits it.
        let r = st.route_keyed(owed.trim_end_matches('.'), Service::Tower, &ctx, wx);
        assert_eq!(r, "", "read back: a quiet frequency");
        assert!(
            CLEARANCE
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .pending_readback
                .is_none()
        );
        // Asking again is answered with the hold, never "read back".
        let r = st.route_keyed(
            &format!("{cs}, ready for departure, for circuits"),
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(!r.contains("read back"), "{r}");
        assert!(r.contains("hold position"), "{r}");

        *TAKEOFF_HOLD.lock().unwrap() = None;
        *CLEARANCE.lock().unwrap() = None;
        *HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }
}
