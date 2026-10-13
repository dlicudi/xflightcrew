//! The EFB's page state: ONE pure function ([`build`]) from a snapshot of
//! facts the app already holds ([`Facts`], gathered by `efb::gather`) to the
//! JSON the page renders ([`EfbState`]). A fact nobody holds is `None` here
//! and a dash on the page, never a number made up to fill the box.
//!
//! The buttons are pilot English ([`PHRASES`]): each goes onto the command
//! channel exactly like a typed line, and `every_button_routes_where_it_says`
//! pins each one to the branch or tool the command loop takes it to.

use serde::Serialize;

use crate::efb::feed::{Alert, Line, Tick, Verdict};
use crate::flight::{ClearanceFact, Control, Phase, Status};
use crate::flow::{Flavour, ItemState, Sheet};

// ---------------------------------------------------------------- phrases

/// How a button's words reach the crew, pinned by the routing test.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Route {
    /// A tool through the grammar (`sortie_runner::route_of`).
    Tool(&'static str),
    /// A branch the loop takes before the grammar (`main::pre_grammar_capture`).
    Capture(&'static str),
    /// A radio call: a tuned tower takes it (`main::state_capture_note`).
    Radio,
    /// A checklist's closed reply (`flow::take_reply`) or an answer to its
    /// proposal (`flow::proposal_reply`), taken at the loop's flow door:
    /// "proposal run", "reply next" and so on.
    Checklist(&'static str),
}

/// A button: its id on the page, its label, the words it sends, and where
/// those words go.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Phrase {
    pub(crate) id: &'static str,
    pub(crate) label: &'static str,
    pub(crate) words: &'static str,
    /// Read by the routing test only: the table is the pin.
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) route: Route,
}

const fn p(id: &'static str, label: &'static str, words: &'static str, route: Route) -> Phrase {
    Phrase {
        id,
        label,
        words,
        route,
    }
}

/// Every fixed button the EFB can show. The two built from a fact (the
/// weather at a field, the station's "say again") are [`weather_words`]
/// and [`station_say_again`], pinned by the same test.
pub(crate) const PHRASES: &[Phrase] = &[
    p(
        "my_controls",
        "My controls",
        "my controls",
        Route::Tool("autopilot_off"),
    ),
    p(
        "load_plan",
        "Load the flight plan",
        "load the flight plan",
        Route::Capture("load the route"),
    ),
    p(
        "load_aircraft",
        "Load the aircraft",
        "load the aircraft",
        Route::Tool("load_aircraft"),
    ),
    p(
        "fly_plan",
        "Fly the flight plan",
        "fly the flight plan",
        Route::Tool("full_auto"),
    ),
    p("request_taxi", "Request taxi", "request taxi", Route::Radio),
    p(
        "check_plan",
        "Check the flight plan",
        "check the flight plan",
        Route::Capture("plan check"),
    ),
    p(
        "brief_arrival",
        "Brief the arrival",
        "brief the arrival",
        Route::Capture("arrival brief"),
    ),
    p(
        "tod",
        "Top of descent?",
        "how long to top of descent",
        Route::Capture("top of descent question"),
    ),
    p(
        "land",
        "Land the aircraft",
        "land the aircraft",
        Route::Tool("autoland"),
    ),
    p(
        "status",
        "How are we doing?",
        "how are we doing",
        Route::Capture("conversation"),
    ),
    p(
        "controller_said",
        "What did the controller say?",
        "what did the controller say",
        Route::Capture("recall"),
    ),
    p(
        "recall",
        "What were you saying?",
        "what were you saying",
        Route::Capture("recall"),
    ),
    p(
        "fetch_plan",
        "Fetch the flight plan",
        "fetch the flight plan",
        Route::Capture("flight plan brief"),
    ),
    // The approach's one suggested action (the pilot's go-around, flown
    // like the tower's: `go_around::GoAroundWatch`).
    p(
        "go_around",
        "Go around",
        "go around",
        Route::Tool("go_around"),
    ),
    // The checklist's closed vocabulary: shown only while a list waits for
    // that answer (or a proposal stands), so each lands at the flow's door.
    p(
        "cl_go",
        "Go ahead",
        "go ahead",
        Route::Checklist("proposal run"),
    ),
    p(
        "cl_teach",
        "Talk me through it",
        "talk me through it",
        Route::Checklist("proposal instruct"),
    ),
    p(
        "cl_later",
        "Not now",
        "not now",
        Route::Checklist("proposal decline"),
    ),
    p("cl_yes", "Yes", "yes", Route::Checklist("reply yes")),
    p("cl_no", "No", "no", Route::Checklist("reply no")),
    p("cl_next", "Next", "next", Route::Checklist("reply next")),
    p("cl_skip", "Skip", "skip", Route::Checklist("reply skip")),
    p("cl_wait", "Wait", "wait", Route::Checklist("reply hold")),
    p(
        "cl_help",
        "Do it for me",
        "help me with that",
        Route::Checklist("reply do it"),
    ),
    p(
        "cl_stop",
        "Stop the checklist",
        "stop the checklist",
        Route::Checklist("reply abort"),
    ),
];

/// A fixed phrase by id.
pub(crate) fn phrase(id: &str) -> &'static Phrase {
    PHRASES
        .iter()
        .find(|p| p.id == id)
        .unwrap_or_else(|| panic!("no EFB phrase {id}"))
}

/// "what's the weather at EGKK" (the loop's remote weather question).
pub(crate) fn weather_words(icao: &str) -> String {
    format!("what's the weather at {}", icao.trim().to_ascii_uppercase())
}

/// "Gatwick Tower, ready for departure": the taxi phase's suggested call,
/// addressed to the station by name so the tower takes it unkeyed. `None`
/// without a tower tuned.
pub(crate) fn station_ready_for_departure(field: &str, tag: &str) -> Option<String> {
    let position = match tag {
        "TWR" => "Tower",
        _ => return None,
    };
    let name = crate::party_line::spoken_airport(field).trim();
    if name.is_empty() {
        return None;
    }
    Some(format!("{name} {position}, ready for departure"))
}

/// "Gatwick Tower, say again": the frequency's say again, addressed to the
/// station by name so the CONTROLLER repeats (a bare "say again" is the
/// copilot's recall). `None` without a station tuned.
pub(crate) fn station_say_again(field: &str, tag: &str) -> Option<String> {
    let position = match tag {
        "TWR" => "Tower",
        "APP" => "Approach",
        "DEP" => "Departure",
        "GND" => "Ground",
        "DEL" => "Delivery",
        _ => return None,
    };
    let name = crate::party_line::spoken_airport(field).trim();
    if name.is_empty() {
        return None;
    }
    Some(format!("{name} {position}, say again"))
}

// ---------------------------------------------------------------- facts

/// The loaded plan, reduced to what the pages show.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct PlanFacts {
    /// "SimBrief" or "Local".
    pub(crate) source: &'static str,
    pub(crate) origin: String,
    pub(crate) destination: String,
    pub(crate) alternate: String,
    pub(crate) aircraft: String,
    pub(crate) route: String,
    pub(crate) cruise_alt_ft: f64,
    pub(crate) distance_nm: f64,
    pub(crate) ete_min: f64,
    pub(crate) tod_to_dest_nm: f64,
    pub(crate) block_fuel_kg: f64,
    pub(crate) trip_fuel_kg: f64,
    pub(crate) reserve_kg: f64,
    pub(crate) min_takeoff_fuel_kg: f64,
    pub(crate) sid: Option<String>,
    pub(crate) star: Option<String>,
    pub(crate) origin_rwy: Option<String>,
    pub(crate) dest_rwy: Option<String>,
    /// (ident, lat, lon, planned altitude ft) in order.
    pub(crate) waypoints: Vec<(String, f64, f64, f64)>,
    /// The navlog's stage per fix ident, when the OFP carried a schedule.
    pub(crate) stages: Vec<(String, &'static str)>,
    pub(crate) weights: Option<WeightFacts>,
    /// When it was put on the shelf (unix s).
    pub(crate) shelved_t: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct WeightFacts {
    pub(crate) pax: u32,
    pub(crate) cargo_kg: f64,
    pub(crate) payload_kg: f64,
    pub(crate) est_tow_kg: f64,
    pub(crate) max_tow_kg: f64,
    pub(crate) est_ldw_kg: f64,
    pub(crate) max_ldw_kg: f64,
}

/// The follower's top of descent (`procedure::planned_tod`).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct TodFacts {
    pub(crate) fix: String,
    pub(crate) to_tod_nm: f64,
    pub(crate) target_ft: f64,
    pub(crate) started: bool,
    pub(crate) held_ft: Option<f64>,
}

/// The station the radio row would show (`crate::party_line::active_station_in_range`).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct StationFacts {
    pub(crate) field: String,
    /// (service word, kHz): "tower", 118700.
    pub(crate) services: Vec<(String, i64)>,
}

/// The accepted IFR clearance (`party_line::assigned_clearance`).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct IfrFacts {
    pub(crate) dest: String,
    pub(crate) sid: Option<String>,
    pub(crate) runway: String,
    pub(crate) initial_alt_ft: i64,
    pub(crate) squawk: i64,
}

/// The controller's last line to us (`Voice::last_controller_line`).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ControllerFacts {
    pub(crate) text: String,
    pub(crate) tag: &'static str,
    pub(crate) khz: Option<i64>,
    pub(crate) age_s: f64,
}

/// One METAR slot: its role and field, and the raw line if the file holds one.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MetarFacts {
    pub(crate) role: &'static str,
    pub(crate) icao: String,
    pub(crate) raw: Option<String>,
    /// A runway the crew picked or was cleared for at this field.
    pub(crate) runway: Option<(String, &'static str)>,
}

/// Everything [`build`] reads, gathered from the app's accessors in one go.
#[derive(Clone, Debug, Default)]
pub(crate) struct Facts {
    pub(crate) now_t: f64,
    pub(crate) flight: Option<Status>,
    /// The innermost loop holding the aircraft (`crew_has_aircraft`).
    pub(crate) owner: Option<&'static str>,
    /// The outermost mission armed (`crew_mission`).
    pub(crate) mission: Option<&'static str>,
    /// The panel's automation feed (`auto_status::compose`).
    pub(crate) auto: String,
    pub(crate) tick: Option<Tick>,
    pub(crate) callsign: Option<String>,
    pub(crate) aircraft_type: Option<String>,
    pub(crate) aircraft_name: Option<String>,
    pub(crate) plan: Option<PlanFacts>,
    /// The follower's active fix and miles to it.
    pub(crate) active_fix: Option<(String, Option<f64>)>,
    pub(crate) tod: Option<TodFacts>,
    pub(crate) com1_khz: Option<i64>,
    /// The tuned position's tag ("TWR"), when COM1 is on our station.
    pub(crate) com1_tag: Option<&'static str>,
    pub(crate) station: Option<StationFacts>,
    pub(crate) ifr: Option<IfrFacts>,
    pub(crate) readback_owed: Option<String>,
    pub(crate) controller: Option<ControllerFacts>,
    pub(crate) lines: Vec<Line>,
    pub(crate) metars: Vec<MetarFacts>,
    /// Age of the METAR file, seconds.
    pub(crate) metar_age_s: Option<f64>,
    pub(crate) audit: Option<Verdict>,
    pub(crate) preflight: Option<Verdict>,
    pub(crate) last_sent: Option<(f64, String)>,
    /// The settings registry, row by row (`settings::ALL`).
    pub(crate) settings: Vec<SettingFacts>,
    /// This flight's crew alerts, oldest first.
    pub(crate) alerts: Vec<Alert>,
    /// The panel's activity word ("listening"), `None` before the first.
    pub(crate) activity: Option<&'static str>,
    /// This run captures on push to talk (the EFB's button can open it).
    pub(crate) ptt_available: bool,
    pub(crate) ptt_held: bool,
    /// This flight's current or last checklist.
    pub(crate) checklist: Option<Sheet>,
    /// The standing proposal's list title.
    pub(crate) proposal: Option<String>,
    /// What the running list waits on: "confirm" or "advance".
    pub(crate) flow_waiting: Option<&'static str>,
    /// The Airport page's facts (`efb::airport`).
    pub(crate) airport: crate::efb::airport::AirportFacts,
    /// Our callsign as the labels write it ("WAT519",
    /// `transcript::written_callsign`).
    pub(crate) callsign_written: Option<String>,
    /// Who has the lateral while the follower flies (`feed::lateral`):
    /// `Some(true)` LNAV, `Some(false)` the follower's bug.
    pub(crate) lateral: Option<bool>,
    /// The one takeoff rule for the plan's takeoff weight
    /// (`aircraft::takeoff_needed_m`), metres; `None` without a book roll.
    pub(crate) takeoff_need_m: Option<f64>,
    /// The one landing rule, the book figure (`aircraft::planned_landing_needed_ft`),
    /// metres; `None` when the profile recites none.
    pub(crate) landing_need_m: Option<f64>,
    /// The plan's fields where the field directory puts them (or the plan's
    /// own waypoint): ("origin" | "destination" | "alternate", ICAO, lat,
    /// lon). The route map labels them always.
    pub(crate) fields: Vec<(&'static str, String, f64, f64)>,
    /// Every field the traffic world simulates, its board payload
    /// (`feed::boards`); `None` with no world running.
    pub(crate) boards: Option<Vec<(String, String)>>,
}

/// What a setting accepts, as the registry declares it (`settings::Kind`).
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum SettingKind {
    Bool,
    Num {
        min: f64,
        max: f64,
        step: f64,
        unit: &'static str,
    },
    Choice(&'static [&'static str]),
    Text,
}

/// One registry row as it stands (`settings::Setting`).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct SettingFacts {
    pub(crate) key: &'static str,
    pub(crate) label: &'static str,
    pub(crate) help: &'static str,
    pub(crate) kind: SettingKind,
    /// The value as the registry parses it back ("on", "0.85", "both").
    pub(crate) value: String,
    /// As the terminal shows it ("0.85", "1200 ms", "(none)").
    pub(crate) shown: String,
    /// The variable that pins it, when one does.
    pub(crate) pinned_env: Option<&'static str>,
}

// ---------------------------------------------------------------- state

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Button {
    pub(crate) id: String,
    pub(crate) label: String,
    pub(crate) say: String,
}

impl Button {
    pub(crate) fn of(p: &Phrase) -> Button {
        Button {
            id: p.id.to_string(),
            label: p.label.to_string(),
            say: p.words.to_string(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Header {
    pub(crate) callsign: Option<String>,
    pub(crate) aircraft: Option<String>,
    pub(crate) origin: Option<String>,
    pub(crate) destination: Option<String>,
    /// "LEMG 13 to EGKK 26L", for a reader without the arrow.
    pub(crate) route: Option<String>,
    /// The flight model's phase, as a word ("Cruise").
    pub(crate) phase: Option<String>,
    /// The altitude beside it in the air ("35,000 ft").
    pub(crate) alt: Option<String>,
    /// Who flies, as the chip says it ("Full auto", "You").
    pub(crate) who: Option<String>,
    /// An automation of ours holds the aircraft.
    pub(crate) crew: bool,
    pub(crate) com1: Option<String>,
    /// The station COM1 is on ("Brest Approach"), when it is ours.
    pub(crate) com1_name: Option<String>,
    /// "idle", "listening", "thinking" or "speaking".
    pub(crate) activity: Option<&'static str>,
}

/// One transcript line, labelled FROM -> TO (`efb::transcript`).
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct TLine {
    pub(crate) t: f64,
    pub(crate) text: String,
    /// COM1 when it went out, for the Radio page's frequency headers.
    pub(crate) freq: Option<String>,
    /// Who that frequency is ("Brest Approach", "Tower").
    pub(crate) station: Option<String>,
    #[serde(flatten)]
    pub(crate) label: crate::efb::transcript::Labelled,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Service {
    pub(crate) name: String,
    pub(crate) freq: String,
    pub(crate) tuned: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct LastCall {
    pub(crate) text: String,
    pub(crate) who: String,
    pub(crate) freq: Option<String>,
    pub(crate) age_s: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Radio {
    pub(crate) com1: Option<String>,
    pub(crate) com1_who: Option<String>,
    pub(crate) station: Option<String>,
    pub(crate) services: Vec<Service>,
    /// The accepted IFR clearance, in a controller's order.
    pub(crate) clearance: Option<String>,
    /// The tower's standing state for us ("TaxiCleared runway 26L, squawk 4521").
    pub(crate) tower_state: Option<String>,
    pub(crate) readback_owed: Option<String>,
    pub(crate) last_call: Option<LastCall>,
    pub(crate) buttons: Vec<Button>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Leg {
    pub(crate) fix: String,
    pub(crate) alt: Option<String>,
    pub(crate) stage: Option<&'static str>,
    pub(crate) active: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Figure {
    pub(crate) k: &'static str,
    pub(crate) v: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct VerdictOut {
    pub(crate) title: &'static str,
    pub(crate) level: &'static str,
    pub(crate) lines: Vec<String>,
    pub(crate) unchecked: Vec<String>,
    pub(crate) t: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct PlanOut {
    pub(crate) source: &'static str,
    pub(crate) shelved_t: Option<f64>,
    pub(crate) origin: String,
    pub(crate) destination: String,
    pub(crate) alternate: Option<String>,
    pub(crate) route: Option<String>,
    pub(crate) procedures: Vec<Figure>,
    pub(crate) legs: Vec<Leg>,
    pub(crate) fuel: Vec<Figure>,
    pub(crate) load: Vec<Figure>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Plan {
    pub(crate) plan: Option<PlanOut>,
    pub(crate) verdicts: Vec<VerdictOut>,
    pub(crate) buttons: Vec<Button>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct MetarOut {
    pub(crate) icao: String,
    pub(crate) role: &'static str,
    pub(crate) raw: Option<String>,
    pub(crate) plain: Option<String>,
    pub(crate) runway: Option<String>,
    pub(crate) button: Button,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Weather {
    pub(crate) metars: Vec<MetarOut>,
    pub(crate) file_age_s: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Sent {
    pub(crate) t: f64,
    pub(crate) text: String,
}

/// A question the page asks before it sends a change.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Confirm {
    /// The value that needs it; `None`: any change does.
    pub(crate) when: Option<String>,
    pub(crate) ask: String,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct SettingRow {
    pub(crate) key: String,
    pub(crate) label: String,
    pub(crate) help: String,
    /// "bool", "num", "choice" or "text".
    pub(crate) kind: &'static str,
    pub(crate) options: Vec<String>,
    pub(crate) min: Option<f64>,
    pub(crate) max: Option<f64>,
    pub(crate) step: Option<f64>,
    pub(crate) unit: String,
    pub(crate) value: String,
    pub(crate) shown: String,
    /// Why the page cannot change it, when it cannot.
    pub(crate) locked: Option<String>,
    pub(crate) confirm: Option<Confirm>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct AlertOut {
    pub(crate) id: u64,
    pub(crate) t: f64,
    pub(crate) level: &'static str,
    pub(crate) text: String,
}

/// The crew's alerts the page shows outside the conversation.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Alerts {
    /// Every page's banner until the page acknowledges it (locally).
    pub(crate) warnings: Vec<AlertOut>,
    /// The header's chip, newest first.
    pub(crate) cautions: Vec<AlertOut>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Ptt {
    /// This run captures on push to talk: the button can open the mic.
    pub(crate) available: bool,
    pub(crate) held: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct ItemOut {
    pub(crate) line: String,
    pub(crate) state: &'static str,
    pub(crate) by: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct ListOut {
    pub(crate) title: String,
    /// "crew", "monitor" or "instruct".
    pub(crate) flavour: &'static str,
    /// What the flavour means, said plainly.
    pub(crate) flavour_note: &'static str,
    pub(crate) running: bool,
    pub(crate) items: Vec<ItemOut>,
    /// Items past (any state but pending and now) of all.
    pub(crate) done: usize,
    pub(crate) total: usize,
    pub(crate) summary: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct ChecklistOut {
    pub(crate) list: Option<ListOut>,
    /// "Climb checklist?" while the copilot's proposal stands.
    pub(crate) proposal: Option<String>,
    pub(crate) waiting: Option<&'static str>,
    pub(crate) buttons: Vec<Button>,
}

/// The whole page, one JSON document.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct EfbState {
    pub(crate) t: f64,
    pub(crate) header: Header,
    /// The Flight tab, laid out for the phase (`efb::flight_tab`).
    pub(crate) flight: crate::efb::flight_tab::FlightOut,
    /// Every line said, oldest first, labelled: the Flight tab's intercom
    /// log and the Radio tab's frequency log are filters of it.
    pub(crate) transcript: Vec<TLine>,
    pub(crate) radio: Radio,
    pub(crate) plan: Plan,
    pub(crate) weather: Weather,
    pub(crate) checklist: ChecklistOut,
    pub(crate) settings: Vec<SettingRow>,
    pub(crate) alerts: Alerts,
    pub(crate) ptt: Ptt,
    pub(crate) my_controls: Button,
    /// The one suggested action for the phase, beside MY CONTROLS.
    pub(crate) action: Option<Button>,
    pub(crate) last_sent: Option<Sent>,
    pub(crate) airport: crate::efb::airport::AirportOut,
    /// The Map tab's live picture in lat/lon (`efb::map`).
    pub(crate) map: crate::efb::map::MapOut,
    /// The Plan tab's flight information board (`efb::board`).
    pub(crate) board: crate::efb::board::BoardOut,
}

// ---------------------------------------------------------------- build

pub(crate) fn phase_label(p: Phase) -> &'static str {
    match p {
        Phase::Parked => "PARKED",
        Phase::EngineStart => "ENGINE START",
        Phase::Ready => "READY",
        Phase::Pushback => "PUSHBACK",
        Phase::TaxiOut => "TAXI",
        Phase::Holding => "HOLDING",
        Phase::LinedUp => "LINED UP",
        Phase::TakeoffRoll => "TAKEOFF",
        Phase::RejectedTakeoff => "REJECT",
        Phase::InitialClimb => "INITIAL CLIMB",
        Phase::Climb => "CLIMB",
        Phase::Cruise => "CRUISE",
        Phase::Descent => "DESCENT",
        Phase::Approach => "APPROACH",
        Phase::Landing => "LANDING",
        Phase::GoAround => "GO AROUND",
        Phase::Hover => "HOVER",
        Phase::Rollout => "ROLLOUT",
        Phase::TaxiIn => "TAXI IN",
    }
}

/// 12345.6 -> "12,346".
pub(crate) fn thousands(v: f64) -> String {
    let n = v.round() as i64;
    let s = n.abs().to_string();
    let mut out = String::new();
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(c);
    }
    if n < 0 {
        format!("-{out}")
    } else {
        out
    }
}

pub(crate) fn feet(v: f64) -> String {
    format!("{} ft", thousands((v / 10.0).round() * 10.0))
}

pub(crate) fn kg(v: f64) -> Option<String> {
    (v.is_finite() && v > 0.0).then(|| format!("{} kg", thousands(v)))
}

pub(crate) fn freq(khz: i64) -> String {
    format!("{:.3}", khz as f64 / 1000.0)
}

pub(crate) fn service_label(word: &str) -> String {
    match word {
        "atis" => "ATIS".into(),
        "clearance" => "Delivery".into(),
        "ground" => "Ground".into(),
        "tower" => "Tower".into(),
        "approach" => "Approach".into(),
        "departure" => "Departure".into(),
        "unicom" => "Unicom".into(),
        other => {
            let mut c = other.chars();
            c.next()
                .map(|f| f.to_uppercase().collect::<String>() + c.as_str())
                .unwrap_or_default()
        }
    }
}

/// Who a frequency is: our station's service on it ("Brest Approach").
pub(crate) fn station_name(f: &Facts, khz: i64) -> Option<String> {
    let st = f.station.as_ref()?;
    let (w, _) = st.services.iter().find(|(_, k)| (khz - k).abs() <= 5)?;
    Some(format!(
        "{} {}",
        crate::party_line::spoken_airport(&st.field).trim(),
        service_label(w)
    ))
}

pub(crate) fn position_name(tag: &str) -> &'static str {
    match tag {
        "TWR" => "Tower",
        "APP" => "Approach",
        "DEP" => "Departure",
        "GND" => "Ground",
        "DEL" => "Delivery",
        "ATIS" => "ATIS",
        _ => "Traffic",
    }
}

/// Who has the aircraft, as the header chip says it.
pub(crate) fn who_flies(f: &Facts) -> (Option<String>, bool) {
    if let Some(o) = f.owner {
        let m = f.mission.filter(|m| *m != o);
        let s = match m {
            Some(m) => format!("{} ({o})", capitalise(m)),
            None => capitalise(o),
        };
        return (Some(s), true);
    }
    match f.flight.as_ref().map(|s| s.control) {
        Some(Control::Automation(n)) => (Some(capitalise(n)), true),
        Some(Control::Autopilot) => (Some("You, autopilot".into()), false),
        Some(Control::Pilot) => (Some("You".into()), false),
        Some(Control::Unknown) | None => (None, false),
    }
}

/// The plan's route as flown from here: (lat, lon) in order.
pub(crate) fn route_points(p: &PlanFacts) -> Vec<(f64, f64)> {
    p.waypoints.iter().map(|w| (w.1, w.2)).collect()
}

/// Miles still to fly along the plan, when the aircraft is on it.
pub(crate) fn remaining_nm(f: &Facts) -> Option<f64> {
    let (p, t) = (f.plan.as_ref()?, f.tick.as_ref()?);
    crate::monitor::route_remaining_nm(&route_points(p), t.lat, t.lon)
}

/// "HH:MMZ" for a unix time.
pub(crate) fn zulu(t: f64) -> String {
    let s = (t.max(0.0) as i64) % 86_400;
    format!("{:02}:{:02}Z", s / 3600, (s % 3600) / 60)
}

pub(crate) fn minutes_words(min: f64) -> String {
    let m = min.round().max(0.0) as i64;
    if m < 1 {
        "under a minute".into()
    } else if m == 1 {
        "about a minute".into()
    } else if m < 60 {
        format!("about {m} minutes")
    } else {
        format!("about {} h {:02}", m / 60, m % 60)
    }
}

pub(crate) fn capitalise(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// The automation feed: "1NAME|stage\nline..." (marker, headline, extras).
pub(crate) fn automation(auto: &str) -> (Option<String>, Vec<String>) {
    let mut chars = auto.chars();
    if chars.next() != Some('1') {
        return (None, Vec::new());
    }
    let rest = chars.as_str();
    let mut lines = rest.lines();
    let head = lines.next().map(|h| match h.split_once('|') {
        Some((n, s)) if !s.trim().is_empty() => format!("{n}: {s}"),
        Some((n, _)) => n.to_string(),
        None => h.to_string(),
    });
    (
        head,
        lines
            .map(str::to_string)
            .filter(|l| !l.is_empty())
            .collect(),
    )
}

pub(crate) fn clearance_line(c: &IfrFacts) -> String {
    let after = match &c.sid {
        Some(s) => format!("{s} departure"),
        None => "runway heading".to_string(),
    };
    format!(
        "Cleared to {}, runway {}, {after}, climb {}, squawk {:04}",
        c.dest,
        c.runway,
        feet(c.initial_alt_ft as f64),
        c.squawk
    )
}

pub(crate) fn tower_state(c: &ClearanceFact) -> String {
    let mut parts = vec![c.state.clone()];
    if let Some(r) = &c.runway {
        parts.push(format!("runway {r}"));
    }
    if let Some(s) = c.squawk {
        parts.push(format!("squawk {s:04}"));
    }
    parts.join(", ")
}

fn radio(f: &Facts) -> Radio {
    let com1 = f.com1_khz.map(freq);
    let com1_who = f.com1_khz.and_then(|k| station_name(f, k));
    let services = f
        .station
        .as_ref()
        .map(|s| {
            s.services
                .iter()
                .map(|(w, k)| Service {
                    name: service_label(w),
                    freq: freq(*k),
                    tuned: f.com1_khz.is_some_and(|c| (c - k).abs() <= 5),
                })
                .collect()
        })
        .unwrap_or_default();
    let last_call = f.controller.as_ref().map(|c| LastCall {
        text: c.text.clone(),
        who: position_name(c.tag).to_string(),
        freq: c.khz.map(freq),
        age_s: c.age_s,
    });
    let mut buttons = Vec::new();
    if let (Some(st), Some(tag)) = (f.station.as_ref(), f.com1_tag) {
        if let Some(words) = station_say_again(&st.field, tag) {
            buttons.push(Button {
                id: "say_again".into(),
                label: "Say again".into(),
                say: words,
            });
        }
    }
    buttons.push(Button::of(phrase("controller_said")));
    buttons.push(Button::of(phrase("recall")));
    Radio {
        com1,
        com1_who,
        station: f.station.as_ref().map(|s| s.field.clone()),
        services,
        clearance: f.ifr.as_ref().map(clearance_line),
        tower_state: f
            .flight
            .as_ref()
            .and_then(|s| s.situation.clearance.as_ref())
            .map(tower_state),
        readback_owed: f.readback_owed.clone(),
        last_call,
        buttons,
    }
}

fn plan(f: &Facts) -> Plan {
    let mut verdicts = Vec::new();
    if let Some(v) = &f.preflight {
        verdicts.push(verdict_out("Preflight at the arm", v));
    }
    if let Some(v) = &f.audit {
        verdicts.push(verdict_out("Plan check", v));
    }
    let mut buttons = vec![
        Button::of(phrase("fetch_plan")),
        Button::of(phrase("check_plan")),
    ];
    if f.plan.is_some() {
        buttons.push(Button::of(phrase("load_plan")));
    }
    Plan {
        plan: f.plan.as_ref().map(|p| plan_out(f, p)),
        verdicts,
        buttons,
    }
}

fn verdict_out(title: &'static str, v: &Verdict) -> VerdictOut {
    VerdictOut {
        title,
        level: v.level,
        lines: v.lines.clone(),
        unchecked: v.unchecked.clone(),
        t: v.t,
    }
}

fn plan_out(f: &Facts, p: &PlanFacts) -> PlanOut {
    let active = f.active_fix.as_ref().map(|(fix, _)| fix.as_str());
    // The active fix is the FIRST leg with its ident at or after the leg
    // flown; without a follower nothing is highlighted.
    let mut seen_active = false;
    let legs = p
        .waypoints
        .iter()
        .map(|(id, _, _, alt)| {
            let is = !seen_active && active.is_some_and(|a| a.eq_ignore_ascii_case(id));
            seen_active |= is;
            Leg {
                fix: id.clone(),
                alt: (*alt > 0.0).then(|| feet(*alt)),
                stage: p
                    .stages
                    .iter()
                    .find(|(i, _)| i.eq_ignore_ascii_case(id))
                    .map(|(_, s)| *s),
                active: is,
            }
        })
        .collect();
    let some = |s: &Option<String>| s.clone().filter(|v| !v.trim().is_empty());
    let mut procedures = vec![
        Figure {
            k: "Departure runway",
            v: some(&p.origin_rwy),
        },
        Figure {
            k: "SID",
            v: some(&p.sid),
        },
        Figure {
            k: "STAR",
            v: some(&p.star),
        },
        Figure {
            k: "Arrival runway",
            v: some(&p.dest_rwy),
        },
        Figure {
            k: "Cruise",
            v: (p.cruise_alt_ft > 0.0).then(|| feet(p.cruise_alt_ft)),
        },
        Figure {
            k: "Distance",
            v: (p.distance_nm > 0.0).then(|| format!("{} nm", thousands(p.distance_nm))),
        },
        Figure {
            k: "Time enroute",
            v: (p.ete_min > 0.0).then(|| {
                format!(
                    "{} h {:02}",
                    (p.ete_min as i64) / 60,
                    (p.ete_min as i64) % 60
                )
            }),
        },
    ];
    if !p.aircraft.is_empty() {
        procedures.push(Figure {
            k: "Type",
            v: Some(p.aircraft.clone()),
        });
    }
    let fuel = vec![
        Figure {
            k: "Block",
            v: kg(p.block_fuel_kg),
        },
        Figure {
            k: "Trip",
            v: kg(p.trip_fuel_kg),
        },
        Figure {
            k: "Reserve",
            v: kg(p.reserve_kg),
        },
        Figure {
            k: "Minimum takeoff",
            v: kg(p.min_takeoff_fuel_kg),
        },
    ];
    let load = match &p.weights {
        Some(w) => vec![
            Figure {
                k: "Passengers",
                v: Some(w.pax.to_string()),
            },
            Figure {
                k: "Cargo",
                v: kg(w.cargo_kg),
            },
            Figure {
                k: "Payload",
                v: kg(w.payload_kg),
            },
            Figure {
                k: "Takeoff weight",
                v: kg(w.est_tow_kg),
            },
            Figure {
                k: "Max takeoff",
                v: kg(w.max_tow_kg),
            },
            Figure {
                k: "Landing weight",
                v: kg(w.est_ldw_kg),
            },
            Figure {
                k: "Max landing",
                v: kg(w.max_ldw_kg),
            },
        ],
        None => Vec::new(),
    };
    PlanOut {
        source: p.source,
        shelved_t: p.shelved_t,
        origin: p.origin.clone(),
        destination: p.destination.clone(),
        alternate: Some(p.alternate.clone()).filter(|a| !a.trim().is_empty()),
        route: Some(p.route.clone()).filter(|r| !r.trim().is_empty()),
        procedures,
        legs,
        fuel,
        load,
    }
}

/// A METAR's short plain line: wind, visibility, weather, cloud, QNH.
pub(crate) fn metar_plain(m: &crate::weather::Metar) -> String {
    let mut parts = Vec::new();
    parts.push(match (m.wind_dir, m.wind_kts) {
        (_, 0) => "Wind calm".to_string(),
        (None, k) => format!("Wind variable at {k} kt"),
        (Some(d), k) => format!("Wind {d:03} at {k} kt"),
    });
    if let Some(g) = m.gust_kts {
        let w = parts.pop().unwrap_or_default();
        parts.push(format!("{w}, gusting {g}"));
    }
    if m.cavok {
        parts.push("CAVOK".into());
    } else {
        parts.push(match m.vis_m {
            None => "visibility 10 km or more".into(),
            Some(v) if v >= 5000 => format!("visibility {} km", v / 1000),
            Some(v) => format!("visibility {v} m"),
        });
        parts.extend(m.wx.iter().cloned());
        if m.clouds.is_empty() {
            parts.push("no significant cloud".into());
        } else {
            parts.extend(m.clouds.iter().take(2).cloned());
        }
    }
    if let Some(q) = m.qnh_hpa {
        parts.push(format!("QNH {q}"));
    }
    format!("{}.", parts.join(", "))
}

fn weather(f: &Facts) -> Weather {
    let metars = f
        .metars
        .iter()
        .map(|m| {
            let parsed = m.raw.as_deref().and_then(crate::weather::parse);
            MetarOut {
                icao: m.icao.clone(),
                role: m.role,
                raw: m.raw.clone(),
                plain: parsed.as_ref().map(metar_plain),
                runway: m
                    .runway
                    .as_ref()
                    .map(|(r, why)| format!("Runway {r} ({why})")),
                button: Button {
                    id: format!("wx_{}", m.icao.to_ascii_lowercase()),
                    label: format!("Ask about {}", m.icao),
                    say: weather_words(&m.icao),
                },
            }
        })
        .collect();
    Weather {
        metars,
        file_age_s: f.metar_age_s,
    }
}

// ---------------------------------------------------------------- settings

/// Why the page may not change a setting, when it may not: a pinned one as
/// the registry refuses it, and the EFB's own link (its port and token are
/// changed where the link is shown, never from a page that would lose it).
pub(crate) fn setting_locked(key: &str, pinned_env: Option<&str>) -> Option<String> {
    if let Some(env) = pinned_env {
        return Some(format!("Pinned by {env}, edit your .env"));
    }
    match key {
        "efb_port" | "efb_token" => Some("Set in the terminal or the panel".into()),
        _ => None,
    }
}

/// The question a change must be confirmed with, when it must: the mode
/// (instructor stands every loop down) and turning the EFB itself off.
pub(crate) fn setting_confirm(key: &str) -> Option<Confirm> {
    match key {
        "mode" => Some(Confirm {
            when: None,
            ask: "Change the app mode? Instructor stands every automation down and the crew keeps its hands off.".into(),
        }),
        "efb" => Some(Confirm {
            when: Some("off".into()),
            ask: "Turn the EFB off? This page stops answering until it is turned on again.".into(),
        }),
        _ => None,
    }
}

/// Does setting `key` to `value` need the page's confirm? Pure: the page
/// and the server ask the same question.
pub(crate) fn needs_confirm(key: &str, value: &str) -> bool {
    setting_confirm(key).is_some_and(|c| match c.when {
        None => true,
        Some(w) => w.eq_ignore_ascii_case(value.trim()),
    })
}

/// The token as the page shows it: enough to recognise, not to copy.
fn masked(token: &str) -> String {
    if token.is_empty() {
        "(none)".into()
    } else {
        format!("{}...", token.chars().take(4).collect::<String>())
    }
}

fn setting_row(s: &SettingFacts) -> SettingRow {
    let (kind, options, min, max, step, unit) = match &s.kind {
        SettingKind::Bool => ("bool", Vec::new(), None, None, None, ""),
        SettingKind::Num {
            min,
            max,
            step,
            unit,
        } => (
            "num",
            Vec::new(),
            Some(*min),
            Some(*max),
            Some(*step),
            *unit,
        ),
        SettingKind::Choice(o) => (
            "choice",
            o.iter().map(|x| x.to_string()).collect(),
            None,
            None,
            None,
            "",
        ),
        SettingKind::Text => ("text", Vec::new(), None, None, None, ""),
    };
    let secret = s.key == "efb_token";
    SettingRow {
        key: s.key.to_string(),
        label: capitalise(s.label),
        help: s.help.to_string(),
        kind,
        options,
        min,
        max,
        step,
        unit: unit.to_string(),
        value: if secret {
            masked(&s.value)
        } else {
            s.value.clone()
        },
        shown: if secret {
            masked(&s.value)
        } else {
            s.shown.clone()
        },
        locked: setting_locked(s.key, s.pinned_env),
        confirm: setting_confirm(s.key),
    }
}

// ---------------------------------------------------------------- alerts

/// A warning stays on the banner this long unless acknowledged.
pub(crate) const WARNING_KEEP_S: f64 = 30.0 * 60.0;
/// A caution stays in the header chip this long.
pub(crate) const CAUTION_KEEP_S: f64 = 5.0 * 60.0;

fn alerts(f: &Facts) -> Alerts {
    let out = |a: &Alert| AlertOut {
        id: a.id,
        t: a.t,
        level: a.level,
        text: a.text.clone(),
    };
    let age = |a: &Alert| f.now_t - a.t;
    Alerts {
        warnings: f
            .alerts
            .iter()
            .filter(|a| a.level == "warning" && age(a) < WARNING_KEEP_S)
            .map(out)
            .collect(),
        cautions: f
            .alerts
            .iter()
            .rev()
            .filter(|a| a.level == "caution" && age(a) < CAUTION_KEEP_S)
            .take(3)
            .map(out)
            .collect(),
    }
}

// ---------------------------------------------------------------- checklist

fn flavour_words(f: Flavour) -> (&'static str, &'static str) {
    match f {
        Flavour::Crew => (
            "crew",
            "The copilot runs it: reads each item, sets what is not in state.",
        ),
        Flavour::Monitor => (
            "monitor",
            "An automation is flying: a silent cross-check, one line at the end.",
        ),
        Flavour::Instruct => (
            "instruct",
            "The instructor calls each item; you set it and say next.",
        ),
    }
}

fn checklist(f: &Facts) -> ChecklistOut {
    let list = f.checklist.as_ref().map(|s| {
        let (flavour, flavour_note) = flavour_words(s.flavour);
        let items: Vec<ItemOut> = s
            .items
            .iter()
            .map(|i| ItemOut {
                line: i.line.clone(),
                state: i.state.word(),
                by: i.by.clone(),
            })
            .collect();
        ListOut {
            title: s.title.clone(),
            flavour,
            flavour_note,
            running: s.running,
            done: s
                .items
                .iter()
                .filter(|i| !matches!(i.state, ItemState::Pending | ItemState::Now))
                .count(),
            total: items.len(),
            items,
            summary: s.summary.clone(),
        }
    });
    let running = list.as_ref().is_some_and(|l| l.running);
    let mut ids: Vec<&str> = Vec::new();
    if !running && f.proposal.is_some() {
        ids.extend(["cl_go", "cl_teach", "cl_later"]);
    }
    if running {
        match f.flow_waiting {
            Some("confirm") => ids.extend(["cl_yes", "cl_no"]),
            Some("advance") => ids.extend(["cl_next", "cl_skip", "cl_wait", "cl_help"]),
            _ => {}
        }
        ids.push("cl_stop");
    }
    ChecklistOut {
        list,
        proposal: f
            .proposal
            .as_ref()
            .filter(|_| !running)
            .map(|t| format!("{} checklist?", capitalise(&t.to_lowercase()))),
        waiting: f.flow_waiting.filter(|_| running),
        buttons: ids.into_iter().map(|id| Button::of(phrase(id))).collect(),
    }
}

/// The transcript, labelled: every line the taps kept, FROM -> TO, the
/// frequency it went out on, and "readback owed" on the controller's last
/// call to us while the party line holds the debt.
pub(crate) fn transcript(f: &Facts) -> Vec<TLine> {
    let ctx = crate::efb::transcript::Ctx {
        spoken: f.callsign.as_deref(),
        written: f.callsign_written.as_deref(),
    };
    let mut out: Vec<TLine> = f
        .lines
        .iter()
        .map(|l| {
            let label = crate::efb::transcript::label(l, &ctx);
            let khz = l.khz.filter(|_| l.radio);
            TLine {
                t: l.t,
                text: l.text.clone(),
                freq: khz.map(freq),
                station: khz.and_then(|k| station_name(f, k)).or_else(|| {
                    l.tag
                        .filter(|_| l.radio)
                        .map(|t| position_name(t).to_string())
                }),
                label,
            }
        })
        .collect();
    if f.readback_owed.is_some() {
        if let Some(last) = out
            .iter_mut()
            .rev()
            .find(|x| x.label.cls == "atc" && !x.label.dim)
        {
            last.label.tags.push("readback owed");
        }
    }
    out
}

/// The page, from the facts. Pure.
pub(crate) fn build(f: &Facts) -> EfbState {
    let (who, crew) = who_flies(f);
    let phase = f
        .flight
        .as_ref()
        .map(|s| capitalise(&phase_label(s.phase).to_lowercase()));
    let alt = f
        .flight
        .as_ref()
        .filter(|s| s.phase.airborne() && s.alt_ft > 0.0)
        .map(|s| feet(s.alt_ft));
    let end = |icao: &str, rwy: &Option<String>| match rwy {
        Some(r) if !r.is_empty() => format!("{icao} {r}"),
        _ => icao.to_string(),
    };
    let route = f.plan.as_ref().map(|p| {
        format!(
            "{} to {}",
            end(&p.origin, &p.origin_rwy),
            end(&p.destination, &p.dest_rwy)
        )
    });
    let radio = radio(f);
    let header = Header {
        callsign: f.callsign_written.clone().or(f.callsign.clone()),
        aircraft: f.aircraft_type.clone().or(f.aircraft_name.clone()),
        origin: f
            .plan
            .as_ref()
            .map(|p| p.origin.clone())
            .filter(|s| !s.is_empty()),
        destination: f
            .plan
            .as_ref()
            .map(|p| p.destination.clone())
            .filter(|s| !s.is_empty()),
        route,
        phase,
        alt,
        who,
        crew,
        com1: radio.com1.clone(),
        com1_name: radio.com1_who.clone(),
        activity: f.activity,
    };
    let airport =
        crate::efb::airport::page(&f.airport, f.tick.as_ref(), f.plan.as_ref(), airborne(f));
    let flight = crate::efb::flight_tab::build(f, &airport);
    let map = crate::efb::map::build(f, &airport);
    EfbState {
        t: f.now_t,
        header,
        flight,
        transcript: transcript(f),
        radio,
        plan: plan(f),
        weather: weather(f),
        checklist: checklist(f),
        settings: f.settings.iter().map(setting_row).collect(),
        alerts: alerts(f),
        ptt: Ptt {
            available: f.ptt_available,
            held: f.ptt_held,
        },
        my_controls: Button::of(phrase("my_controls")),
        action: crate::efb::flight_tab::action(f),
        last_sent: f.last_sent.as_ref().map(|(t, s)| Sent {
            t: *t,
            text: s.clone(),
        }),
        airport,
        map,
        board: crate::efb::board::build(f.boards.as_deref()),
    }
}

/// In the air: the tick says so, else the flight model's phase.
pub(crate) fn airborne(f: &Facts) -> bool {
    match &f.tick {
        Some(t) => !t.on_ground,
        None => f.flight.as_ref().is_some_and(|s| s.phase.airborne()),
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::efb::feed::Who;
    use crate::flight::{Abnormal, PlanFact, Situation};

    /// A 737 in the cruise from Malaga to Gatwick, the follower flying.
    pub(crate) fn cruise() -> Facts {
        let line = |t: f64, who: Who, text: &str, radio: bool, tag: Option<&'static str>| Line {
            t,
            who,
            text: text.into(),
            radio,
            tag,
            khz: radio.then_some(124_650),
            overheard: false,
            readback: false,
        };
        Facts {
            now_t: 1_791_280_800.0, // 2026-10-06 10:00Z
            flight: Some(Status {
                phase: Phase::Cruise,
                control: Control::Automation("follower"),
                abnormal: Abnormal::Normal,
                situation: Situation {
                    mission: Some("full auto"),
                    plan: Some(PlanFact {
                        source: "simbrief",
                        origin: "LEMG".into(),
                        destination: "EGKK".into(),
                        cruise_alt_ft: 35_000.0,
                    }),
                    active_fix: Some("NORMI".into()),
                    clearance: Some(ClearanceFact {
                        state: "Airborne".into(),
                        runway: None,
                        squawk: Some(4521),
                        readback_owed: false,
                    }),
                    ..Default::default()
                },
                alt_ft: 35_004.0,
                ias_kts: 274.4,
                gs_kts: 461.0,
                peak_alt_ft: 35_010.0,
            }),
            owner: Some("follower"),
            mission: Some("full auto"),
            auto: "1FULL AUTO|cruise\nfollower: NORMI\nap: hdg 012, alt 35,000".into(),
            tick: Some(Tick {
                lat: 45.0,
                lon: -1.0,
                alt_ft: 35_004.0,
                agl_ft: 34_900.0,
                ias_kts: 274.4,
                gs_kts: 461.0,
                on_ground: false,
                track_deg: Some(11.6),
                heading_deg: 9.8,
                fuel_kg: Some(8_412.0),
                endurance_min: Some(210.0),
            }),
            callsign: Some("X-ray Victor Lima two one four".into()),
            aircraft_type: Some("B738".into()),
            aircraft_name: Some("Boeing 737-800".into()),
            plan: Some(PlanFacts {
                source: "SimBrief",
                origin: "LEMG".into(),
                destination: "EGKK".into(),
                alternate: "EGLL".into(),
                aircraft: "B738".into(),
                route: "BELEN1C BELEN UN864 LOTOS UN866 NORMI BOGNA1G".into(),
                cruise_alt_ft: 35_000.0,
                distance_nm: 1_012.0,
                ete_min: 134.0,
                tod_to_dest_nm: 110.0,
                block_fuel_kg: 14_200.0,
                trip_fuel_kg: 9_300.0,
                reserve_kg: 2_600.0,
                min_takeoff_fuel_kg: 12_100.0,
                sid: Some("BELEN1C".into()),
                star: Some("BOGNA1G".into()),
                origin_rwy: Some("13".into()),
                dest_rwy: Some("26L".into()),
                waypoints: vec![
                    ("BELEN".into(), 37.5, -4.5, 24_000.0),
                    ("NORMI".into(), 46.0, -0.8, 35_000.0),
                    ("BOGNA".into(), 50.6, -0.5, 11_000.0),
                    ("EGKK".into(), 51.15, -0.19, 0.0),
                ],
                stages: vec![("NORMI".into(), "cruise"), ("BOGNA".into(), "descent")],
                weights: Some(WeightFacts {
                    pax: 162,
                    cargo_kg: 1_100.0,
                    payload_kg: 15_000.0,
                    est_tow_kg: 66_100.0,
                    max_tow_kg: 79_000.0,
                    est_ldw_kg: 61_200.0,
                    max_ldw_kg: 66_300.0,
                }),
                shelved_t: Some(1_791_270_000.0),
            }),
            active_fix: Some(("NORMI".into(), Some(64.0))),
            tod: Some(TodFacts {
                fix: "BOGNA".into(),
                to_tod_nm: 42.0,
                target_ft: 11_000.0,
                started: false,
                held_ft: None,
            }),
            com1_khz: Some(124_650),
            com1_tag: Some("APP"),
            station: Some(StationFacts {
                field: "Brest - Bretagne".into(),
                services: vec![("tower".into(), 118_500), ("approach".into(), 124_650)],
            }),
            ifr: Some(IfrFacts {
                dest: "EGKK".into(),
                sid: Some("BELEN1C".into()),
                runway: "13".into(),
                initial_alt_ft: 5_000,
                squawk: 4521,
            }),
            readback_owed: None,
            controller: Some(ControllerFacts {
                text: "XVL 214, Brest, contact London Control 132.860".into(),
                tag: "APP",
                khz: Some(124_650),
                age_s: 12.0,
            }),
            lines: vec![
                line(1_791_280_600.0, Who::Pilot, "How's the fuel?", false, None),
                line(
                    1_791_280_605.0,
                    Who::Crew,
                    "Eight point four tonnes.",
                    false,
                    None,
                ),
                line(
                    1_791_280_760.0,
                    Who::Atc,
                    "X-ray Victor Lima two one four, Brest, contact London Control 132.860",
                    true,
                    Some("APP"),
                ),
                line(
                    1_791_280_765.0,
                    Who::Crew,
                    "London 132.860, X-ray Victor Lima two one four.",
                    true,
                    Some("APP"),
                ),
                line(
                    1_791_280_770.0,
                    Who::Cabin,
                    "Coffee for the flight deck?",
                    false,
                    None,
                ),
            ],
            metars: vec![
                MetarFacts {
                    role: "Destination",
                    icao: "EGKK".into(),
                    raw: Some("EGKK 061020Z 21019KT 9999 BKN014 14/11 Q1009".into()),
                    runway: Some(("26L".into(), "full auto's arrival")),
                },
                MetarFacts {
                    role: "Origin",
                    icao: "LEMG".into(),
                    raw: Some("LEMG 061030Z 13008KT CAVOK 24/14 Q1018".into()),
                    runway: None,
                },
                MetarFacts {
                    role: "Alternate",
                    icao: "EGLL".into(),
                    raw: None,
                    runway: None,
                },
            ],
            metar_age_s: Some(600.0),
            audit: Some(Verdict {
                level: "caution",
                lines: vec!["Caution: crosswind 17 knots at EGKK.".into()],
                unchecked: vec!["alternate terrain".into()],
                t: 1_791_270_100.0,
            }),
            preflight: None,
            last_sent: Some((1_791_280_790.0, "how long to top of descent".into())),
            settings: vec![
                SettingFacts {
                    key: "mode",
                    label: "app mode",
                    help: "both, copilot only, ATC only, instructor",
                    kind: SettingKind::Choice(&["both", "copilot", "atc", "instructor"]),
                    value: "both".into(),
                    shown: "both".into(),
                    pinned_env: None,
                },
                SettingFacts {
                    key: "voice_volume",
                    label: "intercom volume",
                    help: "the copilot and cabin crew, 1.0 = unity",
                    kind: SettingKind::Num {
                        min: 0.0,
                        max: 2.0,
                        step: 0.05,
                        unit: "",
                    },
                    value: "0.85".into(),
                    shown: "0.85".into(),
                    pinned_env: None,
                },
                SettingFacts {
                    key: "taxi_guide",
                    label: "taxi guide",
                    help: "turn and crossing calls",
                    kind: SettingKind::Bool,
                    value: "on".into(),
                    shown: "on".into(),
                    pinned_env: Some("XCOPILOT_TAXI_GUIDE"),
                },
                SettingFacts {
                    key: "callsign",
                    label: "callsign",
                    help: "blank = the plan's",
                    kind: SettingKind::Text,
                    value: "XVL214".into(),
                    shown: "XVL214".into(),
                    pinned_env: None,
                },
                SettingFacts {
                    key: "efb",
                    label: "EFB",
                    help: "serve the EFB page on the LAN",
                    kind: SettingKind::Bool,
                    value: "on".into(),
                    shown: "on".into(),
                    pinned_env: None,
                },
                SettingFacts {
                    key: "efb_port",
                    label: "EFB port",
                    help: "the TCP port",
                    kind: SettingKind::Num {
                        min: 1024.0,
                        max: 65535.0,
                        step: 1.0,
                        unit: "",
                    },
                    value: "8787".into(),
                    shown: "8787".into(),
                    pinned_env: None,
                },
                SettingFacts {
                    key: "efb_token",
                    label: "EFB token",
                    help: "the link's key",
                    kind: SettingKind::Text,
                    value: "3f9a0c1d2b7e4a6f".into(),
                    shown: "3f9a0c1d2b7e4a6f".into(),
                    pinned_env: None,
                },
            ],
            alerts: vec![
                Alert {
                    id: 7,
                    t: 1_791_278_700.0,
                    level: "warning",
                    text: "Old warning, half an hour ago.".into(),
                    flight: 0,
                },
                Alert {
                    id: 8,
                    t: 1_791_280_700.0,
                    level: "caution",
                    text: "Centre tank is dry, pumps off.".into(),
                    flight: 0,
                },
                Alert {
                    id: 9,
                    t: 1_791_280_780.0,
                    level: "warning",
                    text: "Cabin altitude, masks on.".into(),
                    flight: 0,
                },
                Alert {
                    id: 10,
                    t: 1_791_280_790.0,
                    level: "advisory",
                    text: "Top of descent in five minutes.".into(),
                    flight: 0,
                },
            ],
            activity: Some("listening"),
            ptt_available: true,
            ptt_held: false,
            checklist: Some(Sheet {
                title: "Cruise".into(),
                flavour: Flavour::Monitor,
                items: vec![
                    crate::flow::SheetItem {
                        line: "Cruise power, set.".into(),
                        state: ItemState::Held,
                        by: Some("the speed hold".into()),
                    },
                    crate::flow::SheetItem {
                        line: "Altimeters, standard.".into(),
                        state: ItemState::Done,
                        by: None,
                    },
                    crate::flow::SheetItem {
                        line: "Fuel balance, checked.".into(),
                        state: ItemState::Discrepancy,
                        by: None,
                    },
                    crate::flow::SheetItem {
                        line: "Lights, as required.".into(),
                        state: ItemState::Now,
                        by: None,
                    },
                    crate::flow::SheetItem {
                        line: "Seat belt signs, as required.".into(),
                        state: ItemState::Pending,
                        by: None,
                    },
                ],
                running: true,
                summary: None,
            }),
            proposal: None,
            flow_waiting: None,
            airport: Default::default(),
            callsign_written: Some("XVL214".into()),
            lateral: None,
            takeoff_need_m: None,
            landing_need_m: None,
            fields: vec![
                ("origin", "LEMG".into(), 36.675, -4.499),
                ("destination", "EGKK".into(), 51.148, -0.190),
                ("alternate", "EGLL".into(), 51.471, -0.461),
            ],
            boards: None,
        }
    }

    /// The Plan tab's Board card: a dash with no traffic world, every
    /// simulated field's rows with one (`efb::board`).
    #[test]
    fn the_board_card_is_the_worlds_boards_or_a_dash() {
        let s = build(&cruise());
        assert_eq!(s.board.fields, None);
        let json = serde_json::to_value(&s).unwrap();
        assert!(json["board"]["fields"].is_null(), "a dash on the page");
        let mut f = cruise();
        f.boards = Some(vec![(
            "LEMG".into(),
            "H|LEMG|10:42\nD|10:30|10:30|U2 8062|easyJet|London Gatwick|A320|42|Taxiing|0".into(),
        )]);
        let s = build(&f);
        let fields = s.board.fields.as_ref().unwrap();
        assert_eq!(fields[0].icao, "LEMG");
        assert_eq!(fields[0].departures[0].exp, "On time");
        let json = serde_json::to_value(&s).unwrap();
        assert_eq!(
            json["board"]["fields"][0]["departures"][0]["status"],
            "Taxiing"
        );
    }

    #[test]
    fn the_cruise_page_is_built_from_the_facts() {
        let s = build(&cruise());
        assert_eq!(s.header.callsign.as_deref(), Some("XVL214"));
        assert_eq!(s.header.aircraft.as_deref(), Some("B738"));
        assert_eq!(s.header.route.as_deref(), Some("LEMG 13 to EGKK 26L"));
        assert_eq!(s.header.origin.as_deref(), Some("LEMG"));
        assert_eq!(s.header.phase.as_deref(), Some("Cruise"));
        assert_eq!(s.header.alt.as_deref(), Some("35,000 ft"));
        assert_eq!(s.header.who.as_deref(), Some("Full auto (follower)"));
        assert!(s.header.crew);
        assert_eq!(s.header.com1.as_deref(), Some("124.650"));
        assert_eq!(s.header.com1_name.as_deref(), Some("Brest Approach"));
        assert_eq!(s.flight.layout, "cruise");
        assert_eq!(s.my_controls.say, "my controls");
        assert_eq!(s.action.as_ref().map(|b| b.id.as_str()), Some("tod"));
        // The transcript, labelled.
        let t = &s.transcript;
        assert_eq!(t.len(), 5);
        let ft = |i: usize| format!("{} -> {}", t[i].label.from, t[i].label.to);
        assert_eq!(ft(0), "YOU -> COPILOT");
        assert_eq!(ft(1), "COPILOT -> YOU");
        assert_eq!(ft(2), "APPROACH -> XVL214");
        assert_eq!(t[2].freq.as_deref(), Some("124.650"));
        assert_eq!(t[2].station.as_deref(), Some("Brest Approach"));
        assert_eq!(ft(3), "XVL214 -> APPROACH");
        assert_eq!(t[3].label.tags, ["by copilot"]);
        assert_eq!(ft(4), "CABIN -> FLIGHT DECK");
        assert_eq!(t[0].label.chan, "intercom");
        assert_eq!(t[3].label.chan, "radio");
        // Radio.
        assert_eq!(
            s.radio.clearance.as_deref(),
            Some("Cleared to EGKK, runway 13, BELEN1C departure, climb 5,000 ft, squawk 4521")
        );
        assert_eq!(
            s.radio.tower_state.as_deref(),
            Some("Airborne, squawk 4521")
        );
        assert!(s
            .radio
            .services
            .iter()
            .any(|v| v.tuned && v.name == "Approach"));
        assert_eq!(s.radio.buttons[0].say, "Brest Approach, say again");
        // Plan.
        let p = s.plan.plan.as_ref().expect("plan");
        assert!(p.legs.iter().find(|l| l.fix == "NORMI").unwrap().active);
        assert_eq!(p.legs.iter().filter(|l| l.active).count(), 1);
        assert_eq!(p.legs[3].alt, None, "the field has no planned altitude");
        assert_eq!(s.plan.verdicts[0].level, "caution");
        // Weather.
        assert!(s.weather.metars[0]
            .plain
            .as_ref()
            .unwrap()
            .starts_with("Wind 210 at 19 kt"));
        assert!(s.weather.metars[0]
            .plain
            .as_ref()
            .unwrap()
            .ends_with("QNH 1009."));
        assert_eq!(
            s.weather.metars[1].plain.as_deref(),
            Some("Wind 130 at 8 kt, CAVOK, QNH 1018.")
        );
        assert_eq!(
            s.weather.metars[2].plain, None,
            "no line in the file, no weather"
        );
        assert_eq!(
            s.weather.metars[0].runway.as_deref(),
            Some("Runway 26L (full auto's arrival)")
        );
        // The map: the route ahead in the cruise, the aircraft on it.
        assert_eq!(s.map.view.as_ref().map(|v| v.kind), Some("ahead"));
        assert!(s.map.own.is_some());
        // The whole thing is JSON, the label flattened into each line.
        let j = serde_json::to_value(&s).unwrap();
        assert_eq!(j["transcript"][0]["from"], "YOU");
        assert_eq!(j["transcript"][0]["cls"], "you");
        assert_eq!(j["flight"]["layout"], "cruise");
        assert_eq!(s.header.activity, Some("listening"));
        assert!(s.ptt.available && !s.ptt.held);
    }

    /// A readback owed rides the controller's last call to us.
    #[test]
    fn a_readback_owed_is_tagged_on_the_call_that_owes_it() {
        let mut f = cruise();
        f.readback_owed = Some("Contact London 132.860".into());
        let s = build(&f);
        let owed: Vec<&TLine> = s
            .transcript
            .iter()
            .filter(|l| l.label.tags.contains(&"readback owed"))
            .collect();
        assert_eq!(owed.len(), 1);
        assert_eq!(owed[0].label.from, "APPROACH");
    }

    /// The Settings page is the registry: every row, its control, the
    /// pinned one locked with its variable named as the registry refuses
    /// it, the EFB's own link locked and its token masked, the mode and
    /// the EFB's off asking first.
    #[test]
    fn the_settings_page_is_the_registry_with_its_locks() {
        let s = build(&cruise());
        let row = |k: &str| s.settings.iter().find(|r| r.key == k).unwrap().clone();
        assert_eq!(s.settings.len(), 7);
        let mode = row("mode");
        assert_eq!(mode.kind, "choice");
        assert_eq!(mode.options.len(), 4);
        assert_eq!(mode.label, "App mode");
        assert!(mode.locked.is_none());
        assert!(mode.confirm.as_ref().is_some_and(|c| c.when.is_none()));
        let vol = row("voice_volume");
        assert_eq!((vol.kind, vol.min, vol.max, vol.step), ("num", Some(0.0), Some(2.0), Some(0.05)));
        let pinned = row("taxi_guide");
        assert_eq!(pinned.locked.as_deref(), Some("Pinned by XCOPILOT_TAXI_GUIDE, edit your .env"));
        assert_eq!(row("callsign").kind, "text");
        let efb = row("efb");
        assert!(efb.locked.is_none());
        assert_eq!(efb.confirm.as_ref().and_then(|c| c.when.as_deref()), Some("off"));
        assert!(row("efb_port").locked.is_some());
        let tok = row("efb_token");
        assert!(tok.locked.is_some());
        assert_eq!(tok.value, "3f9a...");
        assert!(!serde_json::to_string(&s).unwrap().contains("3f9a0c1d2b7e4a6f"), "the token never rides the state");
        assert!(needs_confirm("efb", "OFF") && !needs_confirm("efb", "on"));
        assert!(needs_confirm("mode", "both") && !needs_confirm("taxi_guide", "off"));
    }

    /// A warning is a banner for half an hour, a caution a chip for five
    /// minutes, an advisory only the conversation's.
    #[test]
    fn alerts_are_a_banner_a_chip_or_nothing() {
        let s = build(&cruise());
        let w: Vec<u64> = s.alerts.warnings.iter().map(|a| a.id).collect();
        assert_eq!(w, [9], "the old warning has aged off");
        let c: Vec<u64> = s.alerts.cautions.iter().map(|a| a.id).collect();
        assert_eq!(c, [8]);
        let mut f = cruise();
        f.now_t += CAUTION_KEEP_S;
        assert!(build(&f).alerts.cautions.is_empty());
    }

    /// The checklist page: the list item by item, and only the replies the
    /// list is waiting for.
    #[test]
    fn the_checklist_page_offers_only_the_answers_it_waits_for() {
        let ids = |s: &EfbState| -> Vec<String> {
            s.checklist.buttons.iter().map(|b| b.id.clone()).collect()
        };
        let s = build(&cruise());
        let l = s.checklist.list.as_ref().expect("a list");
        assert_eq!((l.flavour, l.running, l.done, l.total), ("monitor", true, 3, 5));
        assert_eq!(l.items[0].state, "held");
        assert_eq!(l.items[0].by.as_deref(), Some("the speed hold"));
        assert_eq!(l.items[3].state, "now");
        assert_eq!(ids(&s), ["cl_stop"]);
        let mut f = cruise();
        f.flow_waiting = Some("confirm");
        assert_eq!(ids(&build(&f)), ["cl_yes", "cl_no", "cl_stop"]);
        f.flow_waiting = Some("advance");
        assert_eq!(ids(&build(&f)), ["cl_next", "cl_skip", "cl_wait", "cl_help", "cl_stop"]);
        // Finished, and the copilot proposes the next.
        let mut f = cruise();
        let sheet = f.checklist.as_mut().unwrap();
        sheet.running = false;
        sheet.summary = Some("Cruise checklist complete.".into());
        f.proposal = Some("Descent".into());
        f.flow_waiting = Some("confirm");
        let s = build(&f);
        assert_eq!(s.checklist.proposal.as_deref(), Some("Descent checklist?"));
        assert_eq!(s.checklist.waiting, None, "a finished list waits for nothing");
        assert_eq!(ids(&s), ["cl_go", "cl_teach", "cl_later"]);
        assert_eq!(
            s.checklist.list.unwrap().summary.as_deref(),
            Some("Cruise checklist complete.")
        );
        // Nothing this flight: no list, no buttons.
        let s = build(&Facts::default());
        assert!(s.checklist.list.is_none() && s.checklist.buttons.is_empty());
    }

    /// Nothing known: every box is None (a dash on the page), no number
    /// invented, the ramp's layout and its one action.
    #[test]
    fn a_page_with_no_facts_shows_dashes() {
        let s = build(&Facts::default());
        assert_eq!(s.header.callsign, None);
        assert_eq!(s.header.phase, None);
        assert_eq!(s.header.who, None);
        assert_eq!(s.flight.layout, "parked");
        assert!(
            s.flight
                .tiles
                .iter()
                .all(|t| t.value.is_none() && t.unit.is_none()),
            "{:?}",
            s.flight.tiles
        );
        assert!(s.plan.plan.is_none());
        assert!(s.radio.com1.is_none() && s.radio.last_call.is_none());
        assert!(s.transcript.is_empty());
        assert!(s.map.own.is_none() && s.map.fixes.is_empty());
        assert_eq!(s.action.map(|b| b.id), Some("fetch_plan".into()));
    }

    #[test]
    fn a_pilot_holding_the_level_is_said_and_the_ground_offers_the_ground() {
        let mut f = cruise();
        f.tod.as_mut().unwrap().held_ft = Some(19_000.0);
        let s = build(&f);
        assert!(s
            .flight
            .sub
            .as_deref()
            .is_some_and(|x| x.contains("Holding 19,000 ft above the plan's descent")));
        let mut f = cruise();
        f.flight.as_mut().unwrap().phase = Phase::Ready;
        f.owner = None;
        f.mission = None;
        f.flight.as_mut().unwrap().control = Control::Pilot;
        f.tod = None;
        f.active_fix = None;
        let s = build(&f);
        assert_eq!(s.header.who.as_deref(), Some("You"));
        assert!(!s.header.crew);
        assert_eq!(s.flight.layout, "parked");
        assert_eq!(s.action.map(|b| b.id), Some("fly_plan".into()));
        // Holding short with a tower tuned and the pilot flying: the call.
        let mut f = cruise();
        f.flight.as_mut().unwrap().phase = Phase::Holding;
        f.owner = None;
        f.mission = None;
        f.com1_tag = Some("TWR");
        f.station = Some(StationFacts {
            field: "London Gatwick".into(),
            services: vec![("tower".into(), 124_225)],
        });
        let s = build(&f);
        assert_eq!(s.flight.layout, "taxi");
        assert_eq!(
            s.action.map(|b| b.say),
            Some("London Gatwick Tower, ready for departure".into())
        );
    }

    /// The fixture's page states as JSON, one per Flight tab layout, for a
    /// look at the page without a sim: `EFB_FIXTURE_OUT=target/efb cargo
    /// test -- --ignored efb_fixture_json` writes `cruise.json`,
    /// `parked.json` and `approach.json` there.
    #[test]
    #[ignore]
    fn efb_fixture_json() {
        let dir = std::env::var("EFB_FIXTURE_OUT").expect("EFB_FIXTURE_OUT");
        std::fs::create_dir_all(&dir).unwrap();
        let write = |name: &str, f: &Facts| {
            let json = serde_json::to_string_pretty(&build(f)).unwrap();
            std::fs::write(format!("{dir}/{name}.json"), json).unwrap();
        };
        write("cruise", &cruise());
        let mut f = cruise();
        f.flight.as_mut().unwrap().phase = Phase::Parked;
        f.flight.as_mut().unwrap().situation.clearance = None;
        f.owner = None;
        f.mission = None;
        f.auto = String::new();
        f.flight.as_mut().unwrap().control = Control::Pilot;
        f.checklist = None;
        f.tick.as_mut().unwrap().on_ground = true;
        f.takeoff_need_m = Some(1_421.0);
        write("parked", &f);
        write("approach", &approach());
    }

    /// The pilot's screenshot, 2026-10-06: the 737 8 nm out of EGKK at
    /// 2,040 ft, on the approach, only the destination left to fly.
    pub(crate) fn approach() -> Facts {
        let mut f = cruise();
        f.flight.as_mut().unwrap().phase = Phase::Approach;
        f.flight.as_mut().unwrap().alt_ft = 2_040.0;
        f.landing_need_m = Some(1_650.0);
        f.readback_owed = Some("Cleared ILS 26L".into());
        f.active_fix = Some(("EGKK".into(), Some(8.0)));
        f.tod = None;
        let t = f.tick.as_mut().unwrap();
        t.lat = 51.12;
        t.lon = -0.40;
        t.alt_ft = 2_040.0;
        t.agl_ft = 1_840.0;
        t.ias_kts = 160.0;
        t.gs_kts = 150.0;
        t.track_deg = Some(78.0);
        t.heading_deg = 78.0;
        f.com1_khz = Some(124_230);
        f.airport.diagram = Some(crate::efb::airport::DiagramRef {
            icao: "EGKK".into(),
            name: "Gatwick".into(),
            version: "v".into(),
            lat0: 51.148,
            lon0: -0.19,
            runways: vec![crate::efb::airport::RunwayEnd {
                ident: "26L".into(),
                length_m: 3_159.0,
                heading_true: 257.8,
            }],
            stands: Vec::new(),
        });
        f
    }

    #[test]
    fn numbers_read_like_a_panel() {
        assert_eq!(thousands(1234567.4), "1,234,567");
        assert_eq!(thousands(999.0), "999");
        assert_eq!(thousands(-1500.0), "-1,500");
        assert_eq!(zulu(1_791_280_800.0 + 3.0 * 60.0), "10:03Z");
        assert_eq!(automation("0MONITORING|You have control"), (None, vec![]));
    }

    /// Every button's words reach the branch or tool its table names, the
    /// way the command loop routes a typed line (`xfc sortie --check`'s
    /// router): a button can never do what the voice cannot, and a phrase
    /// that drifts out of the grammar fails here, not in the cockpit.
    #[test]
    fn every_button_routes_where_it_says() {
        let prof = crate::aircraft::load("boeing_737_800").expect("profile");
        let reg = crate::tools::Registry::for_aircraft(&prof);
        let routed = |words: &str| -> Route {
            if let Some(c) = crate::pre_grammar_capture(words, &reg, &prof) {
                let c: &'static str = Box::leak(c.to_string().into_boxed_str());
                return Route::Capture(c);
            }
            if let Some(t) = crate::sortie_runner::route_of(words, &reg, &prof) {
                let t: &'static str = Box::leak(t.into_boxed_str());
                return Route::Tool(t);
            }
            let tl = words.to_ascii_lowercase();
            assert!(
                crate::party_line::is_radio_call(&tl, "")
                    && crate::tower::parse_intent(&tl) != crate::tower::Intent::Unknown,
                "{words:?} reaches nothing"
            );
            Route::Radio
        };
        for p in PHRASES {
            if let Route::Checklist(door) = p.route {
                // Taken at the flow's door, which only a running list or a
                // standing proposal opens; nothing ahead of it may take the
                // words first (the undo, the release guard, a tuned tower).
                use crate::flow::{ProposalReply as P, Reply as R};
                let got = match door.split_once(' ') {
                    Some(("proposal", _)) => crate::flow::proposal_reply(p.words).map(|a| match a {
                        P::Run => "proposal run",
                        P::Instruct => "proposal instruct",
                        P::Decline => "proposal decline",
                    }),
                    _ => crate::flow::classify(p.words).map(|r| match r {
                        R::Yes => "reply yes",
                        R::No => "reply no",
                        R::Next => "reply next",
                        R::Skip => "reply skip",
                        R::Hold => "reply hold",
                        R::DoIt => "reply do it",
                        R::Abort => "reply abort",
                    }),
                };
                assert_eq!(got, Some(door), "{} ({:?})", p.id, p.words);
                assert!(!crate::is_undo(p.words), "{}: the undo takes it", p.id);
                assert_eq!(crate::state_capture_note(p.words), None, "{}", p.id);
                continue;
            }
            assert_eq!(routed(p.words), p.route, "{} ({:?})", p.id, p.words);
        }
        for icao in ["EGKK", "LEMG", "KSFO"] {
            assert_eq!(
                routed(&weather_words(icao)),
                Route::Capture("weather question"),
                "{icao}"
            );
        }
        // The frequency's say again: addressed to the station by name, it
        // is the controller's repeat at the loop's content-routing door
        // (`try_route_content_atc`), ahead of the copilot's recall.
        for (field, tag) in [
            ("Brest - Bretagne", "APP"),
            ("London Gatwick", "TWR"),
            ("Malaga", "DEP"),
        ] {
            let words = station_say_again(field, tag).expect("a controller");
            let tl = words.to_ascii_lowercase();
            let name = crate::party_line::spoken_airport(field).to_lowercase();
            assert!(crate::tower::is_say_again(&tl), "{words}");
            assert!(crate::party_line::is_radio_call(&tl, &name), "{words}");
        }
        assert_eq!(station_say_again("Malaga", "ATIS"), None);
        // The taxi phase's suggested call: addressed to the tower by name,
        // a radio call the tower parses (`try_route_content_atc`).
        for field in ["London Gatwick", "Gibraltar", "Malaga"] {
            let words = station_ready_for_departure(field, "TWR").expect("a tower");
            let tl = words.to_ascii_lowercase();
            let name = crate::party_line::spoken_airport(field).to_lowercase();
            assert!(crate::party_line::is_radio_call(&tl, &name), "{words}");
            assert_ne!(
                crate::tower::parse_intent(&tl),
                crate::tower::Intent::Unknown,
                "{words}"
            );
        }
        assert_eq!(station_ready_for_departure("Malaga", "APP"), None);
        // "my controls" is the release guard's while a loop holds a
        // control, the autopilot_off tool otherwise: either way the pilot's.
        assert_eq!(
            crate::state_capture_note("my controls"),
            Some("the release guard takes it while a loop holds a control")
        );
    }
}
