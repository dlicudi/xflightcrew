//! The EFB's page state: ONE pure function ([`build`]) from a snapshot of
//! facts the app already holds ([`Facts`], gathered by `efb::gather`) to the
//! JSON the page renders ([`EfbState`]). A fact nobody holds is `None` here
//! and a dash on the page, never a number made up to fill the box.
//!
//! The buttons are pilot English ([`PHRASES`]): each goes onto the command
//! channel exactly like a typed line, and `every_button_routes_where_it_says`
//! pins each one to the branch or tool the command loop takes it to.

use serde::Serialize;

use crate::efb::feed::{Line, Tick, Verdict, Who};
use crate::flight::{ClearanceFact, Control, Phase, Status};

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

/// "Gatwick Tower, say again": the frequency's say again, addressed to the
/// station by name so the CONTROLLER repeats (a bare "say again" is the
/// copilot's recall). `None` without a station tuned.
pub(crate) fn station_say_again(field: &str, tag: &str) -> Option<String> {
    let position = match tag {
        "TWR" => "Tower",
        "APP" => "Approach",
        "DEP" => "Departure",
        _ => return None,
    };
    let name = crate::station::spoken_airport(field).trim();
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

/// The station the radio row would show (`station::active_station_in_range`).
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
}

// ---------------------------------------------------------------- state

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Button {
    pub(crate) id: String,
    pub(crate) label: String,
    pub(crate) say: String,
}

impl Button {
    fn of(p: &Phrase) -> Button {
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
    pub(crate) route: Option<String>,
    pub(crate) phase: Option<String>,
    /// Who flies, as the chip says it ("CREW: FULL AUTO", "YOU").
    pub(crate) who: Option<String>,
    /// An automation of ours holds the aircraft.
    pub(crate) crew: bool,
    pub(crate) com1: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct StripCell {
    pub(crate) label: &'static str,
    /// "past", "now" or "next".
    pub(crate) state: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Next {
    pub(crate) title: String,
    pub(crate) detail: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Gauge {
    pub(crate) k: &'static str,
    pub(crate) v: Option<String>,
    pub(crate) sub: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Talk {
    pub(crate) t: f64,
    pub(crate) who: String,
    pub(crate) kind: Who,
    pub(crate) text: String,
    pub(crate) radio: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Now {
    pub(crate) strip: Vec<StripCell>,
    pub(crate) next: Option<Next>,
    /// The automation feed's headline and its lines, as the panel shows them.
    pub(crate) automation: Option<String>,
    pub(crate) automation_lines: Vec<String>,
    pub(crate) gauges: Vec<Gauge>,
    pub(crate) talk: Vec<Talk>,
    pub(crate) buttons: Vec<Button>,
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
    pub(crate) log: Vec<Talk>,
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

/// The whole page, one JSON document.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct EfbState {
    pub(crate) t: f64,
    pub(crate) header: Header,
    pub(crate) now: Now,
    pub(crate) radio: Radio,
    pub(crate) plan: Plan,
    pub(crate) weather: Weather,
    pub(crate) my_controls: Button,
    pub(crate) last_sent: Option<Sent>,
}

// ---------------------------------------------------------------- build

/// The phase strip's cells, in a flight's order.
const STRIP: [&str; 9] = [
    "RAMP", "TAXI", "TAKEOFF", "CLIMB", "CRUISE", "DESCENT", "APPROACH", "LANDING", "TAXI IN",
];

/// Where a phase sits on the strip; `None` for a hover (no cell is honest).
pub(crate) fn strip_index(p: Phase) -> Option<usize> {
    Some(match p {
        Phase::Parked | Phase::EngineStart | Phase::Ready | Phase::Pushback => 0,
        Phase::TaxiOut | Phase::Holding | Phase::LinedUp => 1,
        Phase::TakeoffRoll | Phase::RejectedTakeoff | Phase::InitialClimb => 2,
        Phase::Climb => 3,
        Phase::Cruise => 4,
        Phase::Descent => 5,
        Phase::Approach | Phase::GoAround => 6,
        Phase::Landing | Phase::Rollout => 7,
        Phase::TaxiIn => 8,
        Phase::Hover => return None,
    })
}

fn phase_label(p: Phase) -> &'static str {
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

fn feet(v: f64) -> String {
    format!("{} ft", thousands((v / 10.0).round() * 10.0))
}

fn kg(v: f64) -> Option<String> {
    (v.is_finite() && v > 0.0).then(|| format!("{} kg", thousands(v)))
}

fn freq(khz: i64) -> String {
    format!("{:.3}", khz as f64 / 1000.0)
}

fn service_label(word: &str) -> String {
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

fn position_name(tag: &str) -> &'static str {
    match tag {
        "TWR" => "Tower",
        "APP" => "Approach",
        "DEP" => "Departure",
        "ATIS" => "ATIS",
        _ => "Traffic",
    }
}

fn who_label(l: &Line) -> String {
    match l.who {
        Who::Pilot => "You".into(),
        Who::Crew => "Crew".into(),
        Who::Cabin => "Cabin".into(),
        Who::Atc => l.tag.map(position_name).unwrap_or("ATC").into(),
        Who::Atis => "ATIS".into(),
        Who::Traffic => "Traffic".into(),
    }
}

fn talk(l: &Line) -> Talk {
    Talk {
        t: l.t,
        who: who_label(l),
        kind: l.who,
        text: l.text.clone(),
        radio: l.radio,
    }
}

/// Who has the aircraft, as the header chip says it.
fn who_flies(f: &Facts) -> (Option<String>, bool) {
    if let Some(o) = f.owner {
        let m = f.mission.filter(|m| *m != o);
        let s = match m {
            Some(m) => format!("CREW: {} ({})", m.to_uppercase(), o.to_uppercase()),
            None => format!("CREW: {}", o.to_uppercase()),
        };
        return (Some(s), true);
    }
    match f.flight.as_ref().map(|s| s.control) {
        Some(Control::Automation(n)) => (Some(format!("CREW: {}", n.to_uppercase())), true),
        Some(Control::Autopilot) => (Some("YOU, AUTOPILOT".into()), false),
        Some(Control::Pilot) => (Some("YOU".into()), false),
        Some(Control::Unknown) | None => (None, false),
    }
}

/// The plan's route as flown from here: (lat, lon) in order.
fn route_points(p: &PlanFacts) -> Vec<(f64, f64)> {
    p.waypoints.iter().map(|w| (w.1, w.2)).collect()
}

/// Miles still to fly along the plan, when the aircraft is on it.
fn remaining_nm(f: &Facts) -> Option<f64> {
    let (p, t) = (f.plan.as_ref()?, f.tick.as_ref()?);
    crate::monitor::route_remaining_nm(&route_points(p), t.lat, t.lon)
}

/// "HH:MMZ" for a unix time.
pub(crate) fn zulu(t: f64) -> String {
    let s = (t.max(0.0) as i64) % 86_400;
    format!("{:02}:{:02}Z", s / 3600, (s % 3600) / 60)
}

fn minutes_words(min: f64) -> String {
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

/// The NEXT card: the follower's own top of descent first, then the fix it
/// steers at, then the plan's ring against the route still to fly.
fn next_card(f: &Facts) -> Option<Next> {
    let gs = f.tick.as_ref().map(|t| t.gs_kts).filter(|g| *g > 40.0);
    let time = |nm: f64| gs.map(|g| minutes_words(nm / g * 60.0));
    if let Some(t) = &f.tod {
        if !t.started && t.held_ft.is_none() && t.to_tod_nm > 0.0 {
            let mut detail = vec![format!("Descent to {} for {}.", feet(t.target_ft), t.fix)];
            if let Some(w) = time(t.to_tod_nm) {
                detail.insert(0, format!("{}.", capitalise(&w)));
            }
            return Some(Next {
                title: format!("Top of descent in {:.0} nm", t.to_tod_nm),
                detail: Some(detail.join(" ")),
            });
        }
        if let Some(h) = t.held_ft {
            return Some(Next {
                title: format!("Holding {} above the plan's descent", feet(h)),
                detail: Some(format!(
                    "The plan descends to {} for {}.",
                    feet(t.target_ft),
                    t.fix
                )),
            });
        }
    }
    if let Some((fix, dist)) = &f.active_fix {
        let title = match dist {
            Some(d) => format!("{fix} in {d:.0} nm"),
            None => format!("Next fix {fix}"),
        };
        return Some(Next {
            title,
            detail: dist.and_then(time).map(|w| format!("{}.", capitalise(&w))),
        });
    }
    let airborne = f.flight.as_ref().is_some_and(|s| s.phase.airborne());
    if let (true, Some(p), Some(rem)) = (airborne, f.plan.as_ref(), remaining_nm(f)) {
        let in_cruise = f
            .flight
            .as_ref()
            .is_some_and(|s| matches!(s.phase, Phase::Cruise | Phase::Climb));
        if in_cruise && p.tod_to_dest_nm > 0.0 && rem > p.tod_to_dest_nm {
            let to = rem - p.tod_to_dest_nm;
            return Some(Next {
                title: format!("Plan's top of descent in {to:.0} nm"),
                detail: time(to)
                    .map(|w| format!("{}. SimBrief's point, along the route.", capitalise(&w))),
            });
        }
        return Some(Next {
            title: format!("{} in {rem:.0} nm", p.destination),
            detail: time(rem).map(|w| format!("{}.", capitalise(&w))),
        });
    }
    None
}

fn capitalise(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

fn gauges(f: &Facts) -> Vec<Gauge> {
    let t = f.tick.as_ref();
    let s = f.flight.as_ref();
    let ias = t.map(|t| t.ias_kts).or(s.map(|s| s.ias_kts));
    let alt = t.map(|t| t.alt_ft).or(s.map(|s| s.alt_ft));
    let gs = t.map(|t| t.gs_kts).or(s.map(|s| s.gs_kts));
    let rem = remaining_nm(f);
    let eta = match (rem, gs) {
        (Some(r), Some(g)) if g > 40.0 && s.is_some_and(|s| s.phase.airborne()) => {
            Some(f.now_t + r / g * 3600.0)
        }
        _ => None,
    };
    let fuel = t.and_then(|t| t.fuel_kg);
    vec![
        Gauge {
            k: "IAS",
            v: ias.map(|v| format!("{v:.0}")),
            sub: Some("kt".into()).filter(|_| ias.is_some()),
        },
        Gauge {
            k: "ALT",
            v: alt.map(|v| thousands((v / 10.0).round() * 10.0)),
            sub: t.map(|t| format!("{} ft AGL", thousands((t.agl_ft / 10.0).round() * 10.0))),
        },
        Gauge {
            k: "GS",
            v: gs.map(|v| format!("{v:.0}")),
            sub: t
                .and_then(|t| t.track_deg)
                .map(|d| format!("track {:03.0}", d.rem_euclid(360.0))),
        },
        Gauge {
            k: "ETA",
            v: eta.map(zulu),
            sub: match (f.plan.as_ref(), rem) {
                (Some(p), Some(r)) => Some(format!("{} in {r:.0} nm", p.destination)),
                _ => None,
            },
        },
        Gauge {
            k: "FUEL",
            v: fuel.map(|v| thousands(v)),
            sub: fuel.map(|_| match t.and_then(|t| t.endurance_min) {
                Some(m) if m < 6000.0 => format!(
                    "kg, {} h {:02} at this burn",
                    (m as i64) / 60,
                    (m as i64) % 60
                ),
                _ => "kg".into(),
            }),
        },
    ]
}

/// The Now page's buttons for the phase: the ground's, the air's.
fn now_buttons(f: &Facts) -> Vec<Button> {
    let airborne = f.flight.as_ref().is_some_and(|s| s.phase.airborne());
    let mut out = Vec::new();
    if airborne {
        out.push(Button::of(phrase("check_plan")));
        out.push(Button::of(phrase("tod")));
        if f.plan.is_some() {
            out.push(Button::of(phrase("brief_arrival")));
        }
        if let Some(p) = f.plan.as_ref().filter(|p| !p.destination.is_empty()) {
            out.push(Button {
                id: "wx_dest".into(),
                label: format!("{} weather", p.destination),
                say: weather_words(&p.destination),
            });
        }
        out.push(Button::of(phrase("land")));
    } else {
        out.push(Button::of(phrase("load_plan")));
        if f.plan.is_some() {
            out.push(Button::of(phrase("load_aircraft")));
            out.push(Button::of(phrase("check_plan")));
            out.push(Button::of(phrase("fly_plan")));
        }
        if f.com1_tag.is_some() {
            out.push(Button::of(phrase("request_taxi")));
        }
    }
    out.push(Button::of(phrase("status")));
    out
}

/// The automation feed: "1NAME|stage\nline..." (marker, headline, extras).
fn automation(auto: &str) -> (Option<String>, Vec<String>) {
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

fn clearance_line(c: &IfrFacts) -> String {
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

fn tower_state(c: &ClearanceFact) -> String {
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
    let tuned_service = |khz: i64| {
        f.station
            .as_ref()
            .and_then(|s| s.services.iter().find(|(_, k)| (khz - k).abs() <= 5))
            .map(|(w, _)| service_label(w))
    };
    let com1_who = match (f.com1_khz, f.station.as_ref()) {
        (Some(k), Some(st)) => tuned_service(k)
            .map(|w| format!("{} {w}", crate::station::spoken_airport(&st.field).trim())),
        _ => None,
    };
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
    let log: Vec<Talk> = f.lines.iter().filter(|l| l.radio).map(talk).collect();
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
        log: log.into_iter().rev().take(40).rev().collect(),
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

/// The page, from the facts. Pure.
pub(crate) fn build(f: &Facts) -> EfbState {
    let phase = f.flight.as_ref().map(|s| s.phase);
    let idx = phase.and_then(strip_index);
    let strip = STRIP
        .iter()
        .enumerate()
        .map(|(i, label)| StripCell {
            label,
            state: match idx {
                Some(n) if i < n => "past",
                Some(n) if i == n => "now",
                _ => "next",
            },
        })
        .collect();
    let (who, crew) = who_flies(f);
    let phase_text = f.flight.as_ref().map(|s| {
        let p = phase_label(s.phase);
        if s.phase.airborne() && s.alt_ft > 0.0 {
            format!("{p} {}", feet(s.alt_ft))
        } else {
            p.to_string()
        }
    });
    let route = f.plan.as_ref().map(|p| {
        let o = match &p.origin_rwy {
            Some(r) if !r.is_empty() => format!("{} {r}", p.origin),
            _ => p.origin.clone(),
        };
        let d = match &p.dest_rwy {
            Some(r) if !r.is_empty() => format!("{} {r}", p.destination),
            _ => p.destination.clone(),
        };
        format!("{o} to {d}")
    });
    let radio = radio(f);
    let header = Header {
        callsign: f.callsign.clone(),
        aircraft: f.aircraft_type.clone().or(f.aircraft_name.clone()),
        route,
        phase: phase_text,
        who,
        crew,
        com1: radio.com1.as_ref().map(|c| match &radio.com1_who {
            Some(w) => format!("{c} {w}"),
            None => c.clone(),
        }),
    };
    let (automation, automation_lines) = automation(&f.auto);
    let talk: Vec<Talk> = f.lines.iter().rev().take(8).rev().map(talk).collect();
    EfbState {
        t: f.now_t,
        header,
        now: Now {
            strip,
            next: next_card(f),
            automation,
            automation_lines,
            gauges: gauges(f),
            talk,
            buttons: now_buttons(f),
        },
        radio,
        plan: plan(f),
        weather: weather(f),
        my_controls: Button::of(phrase("my_controls")),
        last_sent: f.last_sent.as_ref().map(|(t, s)| Sent {
            t: *t,
            text: s.clone(),
        }),
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::flight::{Abnormal, PlanFact, Situation};

    /// A 737 in the cruise from Malaga to Gatwick, the follower flying.
    pub(crate) fn cruise() -> Facts {
        let line = |t: f64, who: Who, text: &str, radio: bool, tag: Option<&'static str>| Line {
            t,
            who,
            text: text.into(),
            radio,
            tag,
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
                fuel_kg: Some(8_412.0),
                endurance_min: Some(210.0),
            }),
            callsign: Some("XVL 214".into()),
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
                    "XVL 214, Brest, contact London Control 132.860",
                    true,
                    Some("APP"),
                ),
                line(
                    1_791_280_765.0,
                    Who::Crew,
                    "London 132.860, XVL 214.",
                    true,
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
        }
    }

    #[test]
    fn the_cruise_page_is_built_from_the_facts() {
        let s = build(&cruise());
        assert_eq!(s.header.callsign.as_deref(), Some("XVL 214"));
        assert_eq!(s.header.aircraft.as_deref(), Some("B738"));
        assert_eq!(s.header.route.as_deref(), Some("LEMG 13 to EGKK 26L"));
        assert_eq!(s.header.phase.as_deref(), Some("CRUISE 35,000 ft"));
        assert_eq!(s.header.who.as_deref(), Some("CREW: FULL AUTO (FOLLOWER)"));
        assert!(s.header.crew);
        assert_eq!(s.header.com1.as_deref(), Some("124.650 Brest Approach"));
        let now: Vec<&str> = s.now.strip.iter().map(|c| c.state).collect();
        assert_eq!(now[4], "now");
        assert_eq!(now[3], "past");
        assert_eq!(now[5], "next");
        let next = s.now.next.as_ref().expect("next");
        assert_eq!(next.title, "Top of descent in 42 nm");
        assert_eq!(
            next.detail.as_deref(),
            Some("About 5 minutes. Descent to 11,000 ft for BOGNA.")
        );
        assert_eq!(s.now.automation.as_deref(), Some("FULL AUTO: cruise"));
        assert_eq!(s.now.automation_lines.len(), 2);
        let g = |k: &str| s.now.gauges.iter().find(|g| g.k == k).unwrap().clone();
        assert_eq!(g("IAS").v.as_deref(), Some("274"));
        assert_eq!(g("ALT").v.as_deref(), Some("35,000"));
        assert_eq!(g("GS").v.as_deref(), Some("461"));
        assert_eq!(g("FUEL").v.as_deref(), Some("8,412"));
        assert!(
            g("ETA").v.is_some(),
            "the route still to fly at the groundspeed"
        );
        assert_eq!(s.now.talk.len(), 4);
        assert_eq!(s.now.talk[2].who, "Approach");
        let ids: Vec<&str> = s.now.buttons.iter().map(|b| b.id.as_str()).collect();
        assert_eq!(
            ids,
            [
                "check_plan",
                "tod",
                "brief_arrival",
                "wx_dest",
                "land",
                "status"
            ]
        );
        assert_eq!(s.now.buttons[3].say, "what's the weather at EGKK");
        assert_eq!(s.my_controls.say, "my controls");
        // Radio.
        assert_eq!(
            s.radio.clearance.as_deref(),
            Some("Cleared to EGKK, runway 13, BELEN1C departure, climb 5,000 ft, squawk 4521")
        );
        assert_eq!(
            s.radio.tower_state.as_deref(),
            Some("Airborne, squawk 4521")
        );
        assert_eq!(s.radio.log.len(), 2, "radio lines only");
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
        // The whole thing is JSON.
        let j = serde_json::to_value(&s).unwrap();
        assert_eq!(j["now"]["talk"][0]["kind"], "pilot");
    }

    /// Nothing known: every box is None (a dash on the page), no number
    /// invented, and the ground's buttons only.
    #[test]
    fn a_page_with_no_facts_shows_dashes() {
        let s = build(&Facts::default());
        assert_eq!(s.header.callsign, None);
        assert_eq!(s.header.phase, None);
        assert_eq!(s.header.who, None);
        assert!(s.now.next.is_none());
        assert!(s
            .now
            .gauges
            .iter()
            .all(|g| g.v.is_none() && g.sub.is_none()));
        assert!(s.now.strip.iter().all(|c| c.state == "next"));
        assert!(s.plan.plan.is_none());
        assert!(s.radio.com1.is_none() && s.radio.last_call.is_none());
        let ids: Vec<&str> = s.now.buttons.iter().map(|b| b.id.as_str()).collect();
        assert_eq!(ids, ["load_plan", "status"]);
    }

    #[test]
    fn a_pilot_holding_the_level_is_said_and_the_ground_offers_the_ground() {
        let mut f = cruise();
        f.tod.as_mut().unwrap().held_ft = Some(19_000.0);
        let s = build(&f);
        assert_eq!(
            s.now.next.unwrap().title,
            "Holding 19,000 ft above the plan's descent"
        );
        let mut f = cruise();
        f.flight.as_mut().unwrap().phase = Phase::Ready;
        f.owner = None;
        f.mission = None;
        f.flight.as_mut().unwrap().control = Control::Pilot;
        f.tod = None;
        f.active_fix = None;
        let s = build(&f);
        assert_eq!(s.header.who.as_deref(), Some("YOU"));
        assert!(!s.header.crew);
        let ids: Vec<&str> = s.now.buttons.iter().map(|b| b.id.as_str()).collect();
        assert_eq!(
            ids,
            [
                "load_plan",
                "load_aircraft",
                "check_plan",
                "fly_plan",
                "request_taxi",
                "status"
            ]
        );
        assert!(
            s.now.next.is_none(),
            "on the ground no route ETA is invented"
        );
        assert!(s
            .now
            .gauges
            .iter()
            .find(|g| g.k == "ETA")
            .unwrap()
            .v
            .is_none());
    }

    /// The fixture's page state as JSON, for a look at the page without a
    /// sim: `EFB_FIXTURE_OUT=/tmp/efb.json cargo test -- --ignored efb_fixture_json`.
    #[test]
    #[ignore]
    fn efb_fixture_json() {
        let out = std::env::var("EFB_FIXTURE_OUT").expect("EFB_FIXTURE_OUT");
        let json = serde_json::to_string_pretty(&build(&cruise())).unwrap();
        std::fs::write(out, json).unwrap();
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
    /// way the command loop routes a typed line (`xvl sortie --check`'s
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
                crate::station::is_radio_call(&tl, "")
                    && crate::tower::parse_intent(&tl) != crate::tower::Intent::Unknown,
                "{words:?} reaches nothing"
            );
            Route::Radio
        };
        for p in PHRASES {
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
            let name = crate::station::spoken_airport(field).to_lowercase();
            assert!(crate::tower::is_say_again(&tl), "{words}");
            assert!(crate::station::is_radio_call(&tl, &name), "{words}");
        }
        assert_eq!(station_say_again("Malaga", "ATIS"), None);
        // "my controls" is the release guard's while a loop holds a
        // control, the autopilot_off tool otherwise: either way the pilot's.
        assert_eq!(
            crate::state_capture_note("my controls"),
            Some("the release guard takes it while a loop holds a control")
        );
    }
}
