//! Who owns the world's state, and when it dies.
//!
//! Every `static` in `crates/atc/src` is classified here by LIFETIME, and a
//! source scan (`every_atc_static_has_a_lifetime` in this module's tests)
//! fails the build of the test suite when one appears that is not. The
//! groups are the reset: a static added to a group is reset by every door
//! that resets the group, with nothing else to remember. A process-wide fact
//! must not outlive its field or its flight (TOWER_WIND, ZONE_CHALLENGE,
//! OWN_FIXES and SELF_TAXI all did, each found by a different sortie).
//!
//! One aircraft's cells live on its strip ([`FlightStrip`](crate::party_line::FlightStrip), `OWN_STRIP` for
//! ours), each declared inside its lifetime group; the strip's groups are
//! reset alongside the statics' by the same doors ([`Group`]).
//!
//! | Lifetime | Group | Dies at |
//! |---|---|---|
//! | flight AND field: the party line's clearance picture | strip `facts` | `install`, `uninstall`, `retire_for_new_flight` |
//! | flight AND field: the station's own worksheets | strip `worksheet`, [`StationFlight`] | `install`, `uninstall`, `retire_for_new_flight` |
//! | flight, carried across a region handoff | strip `carried`, [`FlightCarried`] | `uninstall`, `retire_for_new_flight` (never `install`) |
//! | field: the installed station's watches | strip `watch`, [`StationField`] | `install`, `uninstall` |
//! | field: the traffic strips themselves | [`TrafficStrips`] | `install`, `uninstall` (never `retire_for_new_flight`) |
//! | field: facts built FROM the station | [`FieldFacts`] | `install` writes `for_station`, `uninstall` writes `empty` |
//! | world: survives install, ends with ATC | [`WorldOnly`] | `uninstall` |
//! | transition | `NEW_FLIGHT_PENDING` | set FIRST by `retire_for_new_flight`, cleared LAST by `install` / `uninstall` |
//! | self-expiring world memory | [`SELF_EXPIRING`] | never reset; pruned by age |
//! | session | [`SESSION`] | never reset: memos of files on disk, crew providers, counters |
//!
//! Ordering that matters, and only this:
//! - `retire_for_new_flight` raises `NEW_FLIGHT_PENDING` BEFORE it clears
//!   anything, so the old station (still answering on the main loop) cannot
//!   publish a clearance into the new flight between the clear and the flag.
//! - `install` and `uninstall` drop `NEW_FLIGHT_PENDING` LAST, so a crew
//!   thread judging the departure station (`judge_departure_station`) never
//!   sees the flag down while the last flight's worksheet is still up.
//! - [`FieldFacts`] are OVERWRITTEN by `install`, never cleared first: the
//!   monitor's region watch must never see "no active field" mid-install.

use super::*;

/// One cell of world state that can be put back to its boot value.
pub(crate) trait ResetCell {
    fn reset(&self);
}

impl<T: Default> ResetCell for Mutex<T> {
    fn reset(&self) {
        *self.lock().unwrap_or_else(|e| e.into_inner()) = T::default();
    }
}

impl ResetCell for AtomicBool {
    fn reset(&self) {
        self.store(false, Ordering::SeqCst);
    }
}

/// A lifetime group: the statics named are reset together, to their
/// `Default` (every one is declared with its default as its initialiser).
macro_rules! lifetime_group {
    ($(#[$m:meta])* $vis:vis struct $group:ident { $($name:ident),* $(,)? }) => {
        $(#[$m])*
        $vis struct $group;
        impl $group {
            /// The statics of this group, by name, for the classification scan.
            #[allow(dead_code)]
            pub(crate) const NAMES: &'static [&'static str] = &[$(stringify!($name)),*];
            /// Every static of this group back to its boot value.
            pub(crate) fn reset() {
                $( ResetCell::reset(&$name); )*
            }
        }
    };
}

lifetime_group! {
    /// The station's own worksheet for this flight at this field that is not
    /// a strip cell: the wind the tower worked it with (a field fact).
    pub(crate) struct StationFlight {
        TOWER_WIND,
    }
}

lifetime_group! {
    /// The FLIGHT's, carried across a region handoff, that are the crew's
    /// projection rather than a strip cell: `install` keeps them because the
    /// main loop installs the next station before the crew has consumed the
    /// handoff. A new flight and the end of ATC retire them. (`arm_new_flight`
    /// also drops the pin and the handoff, but only for a field with a tower.)
    pub(crate) struct FlightCarried {
        REGION_HANDOFF,
        ARRIVAL_PIN,
    }
}

lifetime_group! {
    /// The installed station's watches over its field and its traffic.
    pub(crate) struct StationField {
        OUT_OF_REACH,
        STRIP_SEEN,
        GHOSTS,
        WAKE,
        WAKE_PREV,
        DEPARTURE_STREAM,
        ARRIVAL_STREAM,
    }
}

lifetime_group! {
    /// The field's traffic strips (`STRIPS`): a new station is a new field's
    /// traffic, and a new flight of OURS is not the traffic's, so `install`
    /// and `uninstall` clear them and `retire_for_new_flight` never does.
    /// The calls they posted and the station has not answered die with them.
    pub(crate) struct TrafficStrips {
        STRIPS,
        TRAFFIC_CALLS,
        TRAFFIC_PICTURE,
    }
}

lifetime_group! {
    /// Survives `install`, ends with ATC. `PENDING_STATION` is the swap the
    /// main loop is about to install (and `arm_new_flight` writes it AFTER
    /// `retire_for_new_flight`, so a retire must not touch it);
    /// `SURFACE_WIND` is keyed by the field's position at every read, and
    /// the app may hand it down before the install.
    pub(crate) struct WorldOnly {
        PENDING_STATION,
        SURFACE_WIND,
    }
}

/// The strips: each cell's lifetime is the strip group it is declared in
/// ([`FlightStrip`](crate::party_line::FlightStrip)), reset through [`Group`].
#[allow(dead_code)]
pub(crate) const STRIP: &[&str] = &["OWN_STRIP"];

/// One lifetime group as a door names it: a group of our strip's cells, or
/// a group of statics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Group {
    StripFacts,
    StripWorksheet,
    StripCarried,
    StripWatch,
    StationFlight,
    FlightCarried,
    StationField,
    TrafficStrips,
    WorldOnly,
}

impl Group {
    /// Every cell of this group back to its boot value.
    fn reset(self) {
        match self {
            Group::StripFacts => OWN_STRIP.reset_facts(),
            Group::StripWorksheet => OWN_STRIP.reset_worksheet(),
            Group::StripCarried => OWN_STRIP.reset_carried(),
            Group::StripWatch => OWN_STRIP.reset_watch(),
            Group::StationFlight => StationFlight::reset(),
            Group::FlightCarried => FlightCarried::reset(),
            Group::StationField => StationField::reset(),
            Group::TrafficStrips => TrafficStrips::reset(),
            Group::WorldOnly => WorldOnly::reset(),
        }
    }

    /// The cells of this group by name: a strip cell as `OWN_STRIP.<cell>`.
    #[cfg(test)]
    fn cells(self) -> Vec<String> {
        let strip = |names: &[&str]| -> Vec<String> {
            names.iter().map(|n| format!("OWN_STRIP.{n}")).collect()
        };
        let statics = |names: &[&str]| -> Vec<String> {
            names.iter().map(|n| n.to_string()).collect()
        };
        match self {
            Group::StripFacts => strip(crate::party_line::FlightStrip::FACTS),
            Group::StripWorksheet => strip(crate::party_line::FlightStrip::WORKSHEET),
            Group::StripCarried => strip(crate::party_line::FlightStrip::CARRIED),
            Group::StripWatch => strip(crate::party_line::FlightStrip::WATCH),
            Group::StationFlight => statics(StationFlight::NAMES),
            Group::FlightCarried => statics(FlightCarried::NAMES),
            Group::StationField => statics(StationField::NAMES),
            Group::TrafficStrips => statics(TrafficStrips::NAMES),
            Group::WorldOnly => statics(WorldOnly::NAMES),
        }
    }
}

/// What [`Station::install`] resets, in order.
pub(crate) const INSTALL_RESETS: &[Group] = &[
    Group::StripFacts,
    Group::StripWorksheet,
    Group::StationFlight,
    Group::StripWatch,
    Group::StationField,
    Group::TrafficStrips,
];

/// What [`Station::uninstall`] resets, in order.
pub(crate) const UNINSTALL_RESETS: &[Group] = &[
    Group::StripFacts,
    Group::StripWorksheet,
    Group::StationFlight,
    Group::StripCarried,
    Group::FlightCarried,
    Group::StripWatch,
    Group::StationField,
    Group::TrafficStrips,
    Group::WorldOnly,
];

/// What [`retire_for_new_flight`] resets, in order.
pub(crate) const RETIRE_RESETS: &[Group] = &[
    Group::StripFacts,
    Group::StripWorksheet,
    Group::StationFlight,
    Group::StripCarried,
    Group::FlightCarried,
];

fn reset_groups(groups: &[Group]) {
    for g in groups {
        g.reset();
    }
}

/// World memory that ages out by itself and is TRUE across stations: a
/// ghost that has stood on a runway for ten minutes is still stale after a
/// reposition (`GHOST_MEMORY_S`), and a log cadence stamp.
#[allow(dead_code)]
pub(crate) const SELF_EXPIRING: &[&str] = &["STANDING", "GO_AROUND_DEFER_LOGGED"];

/// Never reset, and why: (name, reason).
#[allow(dead_code)]
pub(crate) const SESSION: &[(&str, &str)] = &[
    ("REGION", "region index, loaded once from apt.dat"),
    ("FIELD_DIR", "field directory, loaded once from apt.dat"),
    (
        "TAXI_NET",
        "the field's taxi network, a background-loaded memo",
    ),
    (
        "ILS_FACTS",
        "the field's ILS facts, a background-loaded memo",
    ),
    (
        "LAST_REGION_SWITCH",
        "a cooldown stamp, judged by its window",
    ),
    ("CALLSIGN_INPUTS", "crew provider, registered once at boot"),
    (
        "CALLSIGN_TELEPHONY",
        "crew provider, registered once at boot",
    ),
    ("FILED_ROUTE", "crew provider, registered once at boot"),
    (
        "GO_AROUND_ORDERS",
        "a counter; watchers snapshot it, never reset",
    ),
    ("LAST", "callsign: the last callsign blackboxed, a dedupe"),
];

/// The one transition flag, handled by name for its ordering (module docs).
#[allow(dead_code)]
pub(crate) const TRANSITION: &[&str] = &["NEW_FLIGHT_PENDING"];

/// The facts a station publishes about its field, built FROM it: `install`
/// writes [`FieldFacts::for_station`], `uninstall` writes
/// [`FieldFacts::empty`]. A new field here is a compile error in both
/// constructors and in [`FieldFacts::publish`] until it is handled.
pub(super) struct FieldFacts {
    active_field: Option<(String, f64, f64)>,
    active_icao: Option<String>,
    controller_freqs: Vec<(i64, StationSpeaker)>,
    active_panel: Option<ActivePanel>,
    active_runways: Vec<nav::Runway>,
    atis: Option<AtisState>,
    compliance: ComplianceState,
}

impl FieldFacts {
    /// The statics [`FieldFacts::publish`] writes, for the classification scan.
    #[allow(dead_code)]
    pub(crate) const NAMES: &'static [&'static str] = &[
        "ACTIVE_FIELD",
        "ACTIVE_ICAO",
        "CONTROLLER_FREQS",
        "ACTIVE_PANEL",
        "ACTIVE_RUNWAYS",
        "ATIS",
        "COMPLIANCE",
    ];

    /// No station: the crew's coaches, panel and watches see "no ATC".
    pub(super) fn empty() -> FieldFacts {
        FieldFacts {
            active_field: None,
            active_icao: None,
            controller_freqs: Vec::new(),
            active_panel: None,
            active_runways: Vec::new(),
            atis: None,
            compliance: ComplianceState::NEW,
        }
    }

    /// What `st` publishes when it goes live.
    pub(super) fn for_station(st: &Station) -> FieldFacts {
        // Controller frequencies, for the monitor's traffic advisories: a
        // controller narrates traffic whenever the pilot is on frequency,
        // not only while being vectored.
        let mut controller_freqs = vec![(st.tower_khz, StationSpeaker::Tower)];
        if let Some(k) = st.approach_khz {
            controller_freqs.push((k, StationSpeaker::Approach));
        }
        if let Some(k) = st.departure_khz {
            controller_freqs.push((k, StationSpeaker::Departure));
        }
        let atis = st.apt.freq(nav::FREQ_ATIS).map(|f| {
            tracing::info!(
                mhz = f.khz as f64 / 1000.0,
                "ATIS broadcasting on frequency"
            );
            AtisState {
                khz: f.khz,
                airport: spoken_airport(&st.apt.name).to_string(),
                runways: st.apt.runways.clone(),
                letter: 'A',
                edition: None,
                last_spoken: None,
            }
        });
        FieldFacts {
            active_field: st
                .apt
                .position()
                .map(|(lat, lon)| (st.apt.name.clone(), lat, lon)),
            active_icao: st.procedures.as_ref().map(|(_, icao)| icao.clone()),
            controller_freqs,
            active_panel: Some((st.apt.name.clone(), st.apt.elev_ft, services(&st.apt))),
            active_runways: st.apt.runways.clone(),
            atis,
            // Zone-watch grace on a fresh station: a region handoff says
            // "contact <name> Tower" and the pilot deserves the cooldown
            // window to actually call. Being challenged as an intruder two
            // seconds after the handoff (2026-07-10 tape) reads as a tower at
            // war with its own radar room.
            compliance: ComplianceState {
                zone_last: Some(Instant::now()),
                ..ComplianceState::NEW
            },
        }
    }

    /// Write every fact, each static overwritten in one store.
    pub(super) fn publish(self) {
        let FieldFacts {
            active_field,
            active_icao,
            controller_freqs,
            active_panel,
            active_runways,
            atis,
            compliance,
        } = self;
        *ACTIVE_FIELD.lock().unwrap_or_else(|e| e.into_inner()) = active_field;
        *ACTIVE_ICAO.lock().unwrap_or_else(|e| e.into_inner()) = active_icao;
        *CONTROLLER_FREQS.lock().unwrap_or_else(|e| e.into_inner()) = controller_freqs;
        *ACTIVE_PANEL.lock().unwrap_or_else(|e| e.into_inner()) = active_panel;
        *ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner()) = active_runways;
        *ATIS.lock().unwrap_or_else(|e| e.into_inner()) = atis;
        *COMPLIANCE.lock().unwrap_or_else(|e| e.into_inner()) = compliance;
    }
}

impl Station {
    /// Publish this station's process-wide side state: the ATIS broadcast
    /// and the enroute switch identity the monitor's ticks read. Called by
    /// the OWNER once a station goes live (startup and each region handoff),
    /// never by `from_airport` itself, so offline sweeps and tests stay
    /// side-effect free.
    ///
    /// The previous field's worksheets die with it: a region handoff calls
    /// install() WITHOUT uninstall(), and anything left armed here keeps
    /// acting for a station that no longer exists. An armed vector plan
    /// kept steering toward the old field's threshold; a takeoff handoff
    /// ("contact <old field> Departure on ...") could fire on a later climb
    /// at the new field; a stale arrival taxi-in flag exempted runway
    /// occupancy from the incursion watch. [`FlightCarried`] and
    /// [`WorldOnly`] are kept on purpose (their docs say why).
    pub fn install(&self) {
        reset_groups(INSTALL_RESETS);
        FieldFacts::for_station(self).publish();
        // ...and is the new flight's station: its replies publish again.
        NEW_FLIGHT_PENDING.store(false, Ordering::SeqCst);
    }

    /// The inverse of [`Station::install`]: retire the world entirely. Clears
    /// every published static, the party line AND the monitor-tick side
    /// state, so the crew's coaches, panel, and watches see "no ATC" the
    /// same instant the owner drops the station. Called by the OWNER when
    /// the app leaves an ATC-capable mode; the session memos (region index,
    /// taxi net, ILS facts) survive so re-enabling is cheap.
    pub fn uninstall() {
        reset_groups(UNINSTALL_RESETS);
        FieldFacts::empty().publish();
        NEW_FLIGHT_PENDING.store(false, Ordering::SeqCst);
    }
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
///
/// Also our track before the jump ([`FlightCarried`]: a groundspeed derived
/// across a reposition judged the new final's go-around), the challenge
/// raised for the old flight and the wind the tower worked it with.
pub fn retire_for_new_flight() {
    // FIRST: the old station publishes nothing from here (module docs).
    NEW_FLIGHT_PENDING.store(true, Ordering::SeqCst);
    reset_groups(RETIRE_RESETS);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::party_line::FlightStrip;
    use std::collections::{BTreeMap, BTreeSet};
    use std::path::{Path, PathBuf};

    fn rust_files(dir: &Path, out: &mut Vec<PathBuf>) {
        for entry in std::fs::read_dir(dir).expect("read src dir") {
            let path = entry.expect("dir entry").path();
            if path.is_dir() {
                rust_files(&path, out);
            } else if path.extension().is_some_and(|e| e == "rs")
                && !path.to_string_lossy().ends_with("tests.rs")
            {
                out.push(path);
            }
        }
    }

    /// The name of a `static` declared on this line, if any.
    fn static_name(line: &str) -> Option<String> {
        let mut rest = line.trim_start();
        if rest.starts_with("//") {
            return None;
        }
        if let Some(r) = rest.strip_prefix("pub") {
            rest = r.trim_start();
            if rest.starts_with('(') {
                rest = &rest[rest.find(')')? + 1..];
            }
            rest = rest.trim_start();
        }
        let rest = rest.strip_prefix("static ")?.trim_start();
        let rest = rest.strip_prefix("mut ").unwrap_or(rest);
        let name: String = rest
            .chars()
            .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
            .collect();
        (!name.is_empty() && rest[name.len()..].trim_start().starts_with(':')).then_some(name)
    }

    fn classified() -> BTreeMap<&'static str, &'static str> {
        let mut all = BTreeMap::new();
        let mut add = |names: &[&'static str], class: &'static str| {
            for n in names {
                if let Some(prev) = all.insert(*n, class) {
                    panic!("{n} is classified twice: {prev} and {class}");
                }
            }
        };
        add(STRIP, "FlightStrip");
        add(StationFlight::NAMES, "StationFlight");
        add(FlightCarried::NAMES, "FlightCarried");
        add(StationField::NAMES, "StationField");
        add(TrafficStrips::NAMES, "TrafficStrips");
        add(WorldOnly::NAMES, "WorldOnly");
        add(FieldFacts::NAMES, "FieldFacts");
        add(TRANSITION, "TRANSITION");
        add(SELF_EXPIRING, "SELF_EXPIRING");
        let session: Vec<&'static str> = SESSION.iter().map(|(n, _)| *n).collect();
        add(&session, "SESSION");
        all
    }

    /// A static in the world crate without a lifetime is a fact nobody
    /// retires: classify it in `station/state.rs` (a group resets it with
    /// the others; SESSION says why it never is).
    #[test]
    fn every_atc_static_has_a_lifetime() {
        let src = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut files = Vec::new();
        rust_files(&src, &mut files);
        let mut found: BTreeMap<String, String> = BTreeMap::new();
        for f in &files {
            let text = std::fs::read_to_string(f).expect("read source");
            for line in text.lines() {
                if let Some(name) = static_name(line) {
                    let at = f.strip_prefix(&src).unwrap_or(f).display().to_string();
                    if let Some(prev) = found.insert(name.clone(), at.clone()) {
                        panic!("static {name} declared twice ({prev}, {at}): names must be unique to classify");
                    }
                }
            }
        }
        let strip_cells = [
            FlightStrip::FACTS,
            FlightStrip::WORKSHEET,
            FlightStrip::CARRIED,
            FlightStrip::WATCH,
        ]
        .iter()
        .map(|g| g.len())
        .sum::<usize>();
        assert!(
            found.len() + strip_cells > 40,
            "the scan found only {} statics and {strip_cells} strip cells",
            found.len()
        );
        let classes = classified();
        let unclassified: Vec<String> = found
            .iter()
            .filter(|(n, _)| !classes.contains_key(n.as_str()))
            .map(|(n, at)| format!("{n} ({at})"))
            .collect();
        assert!(
            unclassified.is_empty(),
            "statics with no lifetime, classify them in station/state.rs: {unclassified:?}"
        );
        let found_names: BTreeSet<&str> = found.keys().map(|s| s.as_str()).collect();
        let stale: Vec<&&str> = classes
            .keys()
            .filter(|n| !found_names.contains(**n))
            .collect();
        assert!(
            stale.is_empty(),
            "classified but not declared anywhere: {stale:?}"
        );
    }

    /// The cells a door resets, with a strip cell named by the static it was
    /// before the strip (`OWN_STRIP.clearance` is `CLEARANCE`).
    fn door_cells(groups: &[Group]) -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        for g in groups {
            for c in g.cells() {
                let name = match c.strip_prefix("OWN_STRIP.") {
                    Some(cell) => cell.to_uppercase(),
                    None => c,
                };
                assert!(out.insert(name.clone()), "{name} is reset twice by one door");
            }
        }
        out
    }

    fn names(list: &[&str]) -> BTreeSet<String> {
        list.iter().map(|n| n.to_string()).collect()
    }

    /// Moving the statics onto [`FlightStrip`](crate::party_line::FlightStrip) moved no lifetime: each door
    /// resets exactly the cells it reset when they were statics (the lists
    /// below are the groups as they stood at 8215677).
    #[test]
    fn each_door_resets_exactly_what_it_reset_before_the_strip() {
        let party_line_facts = [
            "CLEARANCE",
            "SQUAWK_NUDGE",
            "TAXI_ROUTE",
            "TAXI_TO_STAND",
            "TAKEOFF_HOLD",
            "LANDING_HOLD",
            "PENDING_LANDING_RELEASE",
            "VECTORS",
            "PENDING_TAKEOFF_CLEARANCE",
            "PENDING_LINE_UP",
            "PENDING_GO_AROUND",
            "RESEQUENCED_ORDER",
            "LINE_UP_CONDITION",
            "DEPARTURE_HANDOFF",
            "ZONE_CHALLENGE",
            "LANDING_HANDOFF",
            "ASSIGNED_SQUAWK",
            "IFR_CLEARANCE",
            // New with our own traffic (stage D, increment 6): the flight's
            // place in the landing sequence, a strip `facts` cell.
            "SEQUENCE",
            // ...and a runway crossing the tower cleared (the 2026-10-08
            // traffic sweep), a strip `facts` cell.
            "CROSSING",
        ];
        let station_flight = [
            "SQUAWK_WATCH",
            "HANDOFF",
            "PENDING_RESEQUENCE",
            "TOWER_WIND",
            "UNCONTROLLED_ARRIVAL",
            "SELF_TAXI",
        ];
        let flight_carried = ["IFR_FLIGHT", "REGION_HANDOFF", "ARRIVAL_PIN", "OWN_FIXES"];
        let station_field = [
            "OUT_OF_REACH",
            "GO_AROUND_LAST",
            "STRIP_SEEN",
            "GHOSTS",
            "WAKE",
            "WAKE_PREV",
            "DEPARTURE_STREAM",
            "ARRIVAL_STREAM",
        ];
        let world_only = ["PENDING_STATION", "SURFACE_WIND"];

        let mut install = names(&party_line_facts);
        install.extend(names(&station_flight));
        install.extend(names(&station_field));
        // New with the traffic strips (increment 5): the field's traffic.
        install.extend(names(&["STRIPS"]));
        // Split from COMPLIANCE (increment 7), which `install` and
        // `uninstall` overwrite: the flight part, a strip `watch` cell.
        install.extend(names(&["COMPLIANCE_FLIGHT"]));
        // New with our own traffic (stage D, increment 4): where a traffic
        // flight was last seen, a strip `watch` cell (never written on ours).
        install.extend(names(&["SEEN"]));
        // ...and the traffic's calls not yet answered (increment 5).
        install.extend(names(&["TRAFFIC_CALLS"]));
        // ...and the picture of our own traffic every TCAS read takes while
        // the fleet flies (stage D, increment 8): republished every tick.
        install.extend(names(&["TRAFFIC_PICTURE"]));
        assert_eq!(door_cells(INSTALL_RESETS), install, "install");

        let mut uninstall = install.clone();
        uninstall.extend(names(&flight_carried));
        uninstall.extend(names(&world_only));
        assert_eq!(door_cells(UNINSTALL_RESETS), uninstall, "uninstall");

        let mut retire = names(&party_line_facts);
        retire.extend(names(&station_flight));
        retire.extend(names(&flight_carried));
        assert_eq!(door_cells(RETIRE_RESETS), retire, "retire_for_new_flight");
    }

    #[test]
    fn the_scan_reads_every_declaration_shape() {
        assert_eq!(
            static_name("static LAST: Mutex<()> = x;").as_deref(),
            Some("LAST")
        );
        assert_eq!(
            static_name("    pub(super) static A_B: AtomicBool =").as_deref(),
            Some("A_B")
        );
        assert_eq!(
            static_name("pub(crate) static X: u8 = 0;").as_deref(),
            Some("X")
        );
        assert_eq!(
            static_name("pub static mut Y: u8 = 0;").as_deref(),
            Some("Y")
        );
        assert_eq!(static_name("/// static FAKE: not a declaration"), None);
        assert_eq!(static_name("fn f(x: &'static str) {}"), None);
    }
}
