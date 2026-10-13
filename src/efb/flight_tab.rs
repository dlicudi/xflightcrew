//! The Flight tab: one layout per phase of the flight model
//! (`flight::published()`), each a headline, four readouts and the cards a
//! pilot reaches for in that phase. Pure, over the same [`Facts`] the rest
//! of the page reads: a fact nobody holds is a dash, and a readout with no
//! source is dropped rather than filled.

use serde::Serialize;

use crate::efb::airport::{runway_end, AirportOut, RunwayEnd};
use crate::efb::state::{
    automation, capitalise, feet, phase_label, phrase, remaining_nm, station_ready_for_departure,
    thousands, tower_state, who_flies, zulu, Button, Facts, PlanFacts,
};
use crate::flight::Phase;

/// A run of headline text: plain, a fix (magenta) or a number (mono).
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Seg {
    pub(crate) t: String,
    /// "", "fix" or "num".
    pub(crate) k: &'static str,
}

fn plain(t: impl Into<String>) -> Seg {
    Seg { t: t.into(), k: "" }
}
fn fix(t: impl Into<String>) -> Seg {
    Seg {
        t: t.into(),
        k: "fix",
    }
}
fn num(t: impl Into<String>) -> Seg {
    Seg {
        t: t.into(),
        k: "num",
    }
}

/// One of the four large readouts.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Tile {
    pub(crate) label: String,
    pub(crate) value: Option<String>,
    pub(crate) unit: Option<String>,
    pub(crate) note: Option<String>,
    /// "", "mag", "ok" or "warn".
    pub(crate) tone: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Row {
    pub(crate) k: String,
    pub(crate) v: Option<String>,
    /// "", "mag", "cy", "ok" or "warn".
    pub(crate) tone: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct LegRow {
    pub(crate) id: String,
    pub(crate) meta: Option<String>,
    pub(crate) eta: Option<String>,
    /// "done", "active" or "next".
    pub(crate) state: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Bar {
    /// Needed as a share of available, 0..100.
    pub(crate) pct: f64,
    pub(crate) label: String,
    pub(crate) ok: bool,
}

/// A supporting card.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Card {
    pub(crate) title: String,
    /// "rows", "legs", "bar" or "checklist" (the page draws the running
    /// list, `state.checklist`, in its place).
    pub(crate) kind: &'static str,
    pub(crate) rows: Vec<Row>,
    pub(crate) legs: Vec<LegRow>,
    pub(crate) bar: Option<Bar>,
    /// What the card shows when it has nothing.
    pub(crate) empty: Option<String>,
}

impl Card {
    fn rows(title: impl Into<String>, rows: Vec<Row>) -> Card {
        Card {
            title: title.into(),
            kind: "rows",
            rows,
            legs: Vec::new(),
            bar: None,
            empty: None,
        }
    }
    fn legs(title: impl Into<String>, legs: Vec<LegRow>, empty: &str) -> Card {
        Card {
            title: title.into(),
            kind: "legs",
            rows: Vec::new(),
            legs,
            bar: None,
            empty: Some(empty.into()),
        }
    }
    fn checklist(title: impl Into<String>) -> Card {
        Card {
            title: title.into(),
            kind: "checklist",
            rows: Vec::new(),
            legs: Vec::new(),
            bar: None,
            empty: None,
        }
    }
}

/// The Flight tab.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct FlightOut {
    /// "parked", "taxi", "cruise" or "approach".
    pub(crate) layout: &'static str,
    /// "Now · cruise 35,000 ft, 274 kt".
    pub(crate) lead: String,
    /// Who flies and how, beside the lead.
    pub(crate) who: Option<String>,
    pub(crate) headline: Vec<Seg>,
    pub(crate) sub: Option<String>,
    pub(crate) tiles: Vec<Tile>,
    pub(crate) cards: Vec<Card>,
}

/// Which layout a flight phase reads as. `None` (no flight model yet) is
/// the ramp's. Pure.
pub(crate) fn layout_for(p: Option<Phase>) -> &'static str {
    match p {
        None
        | Some(Phase::Parked)
        | Some(Phase::EngineStart)
        | Some(Phase::Ready)
        | Some(Phase::Pushback) => "parked",
        Some(Phase::TaxiOut)
        | Some(Phase::Holding)
        | Some(Phase::LinedUp)
        | Some(Phase::TakeoffRoll)
        | Some(Phase::RejectedTakeoff)
        | Some(Phase::TaxiIn) => "taxi",
        Some(Phase::InitialClimb)
        | Some(Phase::Climb)
        | Some(Phase::Cruise)
        | Some(Phase::Hover) => "cruise",
        Some(Phase::Descent)
        | Some(Phase::Approach)
        | Some(Phase::GoAround)
        | Some(Phase::Landing)
        | Some(Phase::Rollout) => "approach",
    }
}

fn some(s: &Option<String>) -> Option<String> {
    s.clone().filter(|v| !v.trim().is_empty())
}

fn metres(m: f64) -> String {
    format!("{} m", thousands((m / 10.0).round() * 10.0))
}

/// The METAR of a field the plan names, parsed.
fn metar_of(f: &Facts, icao: &str) -> Option<crate::weather::Metar> {
    f.metars
        .iter()
        .find(|m| m.icao.eq_ignore_ascii_case(icao))
        .and_then(|m| m.raw.as_deref())
        .and_then(crate::weather::parse)
}

/// "090 / 11 kt", "calm", "variable 4 kt".
fn wind_words(m: &crate::weather::Metar) -> String {
    let base = match (m.wind_dir, m.wind_kts) {
        (_, 0) => return "Calm".into(),
        (None, k) => format!("Variable {k} kt"),
        (Some(d), k) => format!("{d:03} / {k} kt"),
    };
    match m.gust_kts {
        Some(g) => format!("{base}, gust {g}"),
        None => base,
    }
}

/// Head (positive) and cross components of a wind on a runway, knots.
/// Pure.
pub(crate) fn wind_components(wind_from_true: f64, kts: f64, rwy_true: f64) -> (f64, f64) {
    let a = (wind_from_true - rwy_true).to_radians();
    (kts * a.cos(), kts * a.sin())
}

/// "Headwind 12, cross 2" (or a tailwind), from the METAR on a runway end.
fn components_words(m: &crate::weather::Metar, rwy: &RunwayEnd) -> Option<String> {
    let d = m.wind_dir? as f64;
    if m.wind_kts == 0 {
        return None;
    }
    let (head, cross) = wind_components(d, m.wind_kts as f64, rwy.heading_true);
    let along = if head >= -0.5 {
        format!("Headwind {:.0}", head.max(0.0))
    } else {
        format!("Tailwind {:.0}", -head)
    };
    Some(format!("{along}, cross {:.0}", cross.abs()))
}

/// The diagram's ends, when the drawn field is this one.
fn ends_at<'a>(f: &'a Facts, icao: &str) -> &'a [RunwayEnd] {
    match f.airport.diagram.as_ref() {
        Some(d) if d.icao.eq_ignore_ascii_case(icao) => &d.runways,
        _ => &[],
    }
}

/// The departure runway: the tower's clearance, else the plan's.
fn departure_runway(f: &Facts) -> Option<String> {
    f.airport
        .cleared_runway
        .clone()
        .or_else(|| f.ifr.as_ref().map(|c| c.runway.clone()))
        .or_else(|| f.plan.as_ref().and_then(|p| some(&p.origin_rwy)))
        .filter(|r| !r.trim().is_empty())
}

/// The arrival runway: full auto's, else the clearance's in the air, else
/// the plan's.
fn arrival_runway(f: &Facts) -> Option<String> {
    f.airport
        .arrival_runway
        .clone()
        .or_else(|| f.plan.as_ref().and_then(|p| some(&p.dest_rwy)))
        .filter(|r| !r.trim().is_empty())
}

fn gs(f: &Facts) -> Option<f64> {
    f.tick
        .as_ref()
        .map(|t| t.gs_kts)
        .or(f.flight.as_ref().map(|s| s.gs_kts))
}

fn ias(f: &Facts) -> Option<f64> {
    f.tick
        .as_ref()
        .map(|t| t.ias_kts)
        .or(f.flight.as_ref().map(|s| s.ias_kts))
}

/// A time to fly `nm` at the groundspeed, as an ETA, when the aircraft is
/// moving like an aeroplane.
fn eta(f: &Facts, nm: f64) -> Option<f64> {
    gs(f)
        .filter(|g| *g > 40.0)
        .map(|g| f.now_t + nm / g * 3600.0)
}

fn who_line(f: &Facts) -> Option<String> {
    let (head, _) = automation(&f.auto);
    head.or_else(|| who_flies(f).0)
}

fn checklist_title(f: &Facts) -> Option<String> {
    f.checklist
        .as_ref()
        .map(|s| format!("{} checklist", capitalise(&s.title.to_lowercase())))
        .or_else(|| {
            f.proposal
                .as_ref()
                .map(|t| format!("{} checklist?", capitalise(&t.to_lowercase())))
        })
}

/// The card for the running (or last, or proposed) checklist, when any.
fn checklist_card(f: &Facts) -> Option<Card> {
    checklist_title(f).map(Card::checklist)
}

// ---------------------------------------------------------------- parked

fn parked(f: &Facts) -> FlightOut {
    let phase = f.flight.as_ref().map(|s| s.phase);
    let lead = format!(
        "Now · {}",
        phase
            .map(|p| phase_label(p).to_lowercase())
            .unwrap_or_else(|| "on the ground".into())
    );
    let plan = f.plan.as_ref();
    let (headline, sub) = if let Some(c) = &f.ifr {
        let after = match &c.sid {
            Some(s) => format!("{s} departure"),
            None => "Runway heading".to_string(),
        };
        (
            vec![
                plain(format!(
                    "IFR clearance to {} held, runway {}, squawk ",
                    c.dest, c.runway
                )),
                num(format!("{:04}", c.squawk)),
            ],
            Some(format!(
                "{after}, climb {}. Read back and accepted.",
                feet(c.initial_alt_ft as f64)
            )),
        )
    } else if let Some(c) = f
        .flight
        .as_ref()
        .and_then(|s| s.situation.clearance.as_ref())
    {
        (vec![plain(tower_state(c))], None)
    } else if let Some(p) = plan {
        (
            vec![
                fix(p.origin.clone()),
                plain(" to "),
                fix(p.destination.clone()),
                plain(", no clearance yet"),
            ],
            Some(if f.com1_tag.is_some() {
                "Ask the tower for your clearance, or say \"fly the flight plan\" and the crew will."
                    .to_string()
            } else {
                "Say \"fly the flight plan\" and the crew flies it, or ask for your clearance once a tower is tuned."
                    .to_string()
            }),
        )
    } else {
        (
            vec![plain("No flight plan on the shelf")],
            Some("Fetch one from SimBrief, or say \"plan a flight to\" a field.".into()),
        )
    };
    let fuel = f.tick.as_ref().and_then(|t| t.fuel_kg);
    let block = plan.map(|p| p.block_fuel_kg).filter(|b| *b > 0.0);
    let min_to = plan.map(|p| p.min_takeoff_fuel_kg).filter(|b| *b > 0.0);
    let fuel_tone = match (fuel, min_to) {
        (Some(v), Some(m)) if v + 1.0 < m => "warn",
        (Some(_), Some(_)) => "ok",
        _ => "",
    };
    let w = plan.and_then(|p| p.weights.as_ref());
    let tow = w.map(|w| w.est_tow_kg).filter(|v| *v > 0.0);
    let mtow = w.map(|w| w.max_tow_kg).filter(|v| *v > 0.0);
    let squawk = f.ifr.as_ref().map(|c| c.squawk).or(f
        .flight
        .as_ref()
        .and_then(|s| s.situation.clearance.as_ref())
        .and_then(|c| c.squawk));
    let tiles = vec![
        Tile {
            label: "Fuel on board".into(),
            value: fuel.map(thousands),
            unit: fuel.map(|_| "kg".into()),
            note: block.map(|b| format!("Plan {} kg", thousands(b))),
            tone: fuel_tone,
        },
        Tile {
            label: "Takeoff weight".into(),
            value: tow.map(thousands),
            unit: tow.map(|_| "kg".into()),
            note: mtow.map(|m| format!("Plan's figure, limit {}", thousands(m))),
            tone: match (tow, mtow) {
                (Some(t), Some(m)) if t > m => "warn",
                (Some(_), Some(_)) => "ok",
                _ => "",
            },
        },
        Tile {
            label: "Flight time".into(),
            value: plan
                .map(|p| p.ete_min)
                .filter(|m| *m > 0.0)
                .map(|m| format!("{}:{:02}", (m as i64) / 60, (m as i64) % 60)),
            unit: None,
            note: plan.filter(|p| p.distance_nm > 0.0).map(|p| {
                format!(
                    "{} nm, cruise {}",
                    thousands(p.distance_nm),
                    feet(p.cruise_alt_ft)
                )
            }),
            tone: "",
        },
        Tile {
            label: "Squawk".into(),
            value: squawk.map(|s| format!("{s:04}")),
            unit: None,
            note: Some(if f.ifr.is_some() {
                "IFR clearance accepted".into()
            } else if squawk.is_some() {
                "From the tower".into()
            } else {
                "Not cleared yet".into()
            }),
            tone: if squawk.is_some() { "ok" } else { "" },
        },
    ];
    let mut cards = Vec::new();
    match checklist_card(f) {
        Some(c) => cards.push(c),
        None => cards.push(departure_card(f)),
    }
    cards.push(runway_card(f));
    FlightOut {
        layout: "parked",
        lead,
        who: who_line(f),
        headline,
        sub,
        tiles,
        cards,
    }
}

fn departure_card(f: &Facts) -> Card {
    let plan = f.plan.as_ref();
    Card::rows(
        "Departure",
        vec![
            Row {
                k: "SID".into(),
                v: f.ifr
                    .as_ref()
                    .and_then(|c| c.sid.clone())
                    .or_else(|| plan.and_then(|p| some(&p.sid))),
                tone: "mag",
            },
            Row {
                k: "Initial climb".into(),
                v: f.ifr.as_ref().map(|c| feet(c.initial_alt_ft as f64)),
                tone: "",
            },
            Row {
                k: "Cruise".into(),
                v: plan.map(|p| p.cruise_alt_ft).filter(|a| *a > 0.0).map(feet),
                tone: "",
            },
            Row {
                k: "COM1".into(),
                v: f.com1_khz.map(|k| format!("{:.3}", k as f64 / 1000.0)),
                tone: "cy",
            },
        ],
    )
}

/// The departure runway with what it offers against what we need, the
/// wind on it and the QNH.
fn runway_card(f: &Facts) -> Card {
    let rwy = departure_runway(f);
    let origin = f
        .plan
        .as_ref()
        .map(|p| p.origin.clone())
        .unwrap_or_default();
    let end = rwy
        .as_deref()
        .and_then(|r| runway_end(ends_at(f, &origin), r));
    let metar = metar_of(f, &origin);
    let need = f.takeoff_need_m;
    let fit = match (end, need) {
        (Some(e), Some(n)) if n <= e.length_m => Some((format!("{}, fits", metres(n)), "ok")),
        (Some(e), Some(n)) => Some((
            format!("{}, short by {}", metres(n), metres(n - e.length_m)),
            "warn",
        )),
        (None, Some(n)) => Some((metres(n), "")),
        _ => None,
    };
    Card::rows(
        match &rwy {
            Some(r) => format!("Runway {r}"),
            None => "Departure runway".into(),
        },
        vec![
            Row {
                k: "Available".into(),
                v: end.map(|e| metres(e.length_m)),
                tone: "",
            },
            Row {
                k: "Needed".into(),
                v: fit.as_ref().map(|x| x.0.clone()),
                tone: fit.map(|x| x.1).unwrap_or(""),
            },
            Row {
                k: "Wind".into(),
                v: metar
                    .as_ref()
                    .map(|m| match end.and_then(|e| components_words(m, e)) {
                        Some(c) => format!("{}, {}", wind_words(m), c.to_lowercase()),
                        None => wind_words(m),
                    }),
                tone: "cy",
            },
            Row {
                k: "QNH".into(),
                v: metar
                    .as_ref()
                    .and_then(|m| m.qnh_hpa)
                    .map(|q| q.to_string()),
                tone: "",
            },
        ],
    )
}

// ---------------------------------------------------------------- taxi

/// The taxi route's legs as a pilot reads them: each taxiway or runway
/// name once, its length, done or to come, then the hold.
fn taxi_legs(f: &Facts) -> Vec<LegRow> {
    let Some(r) = f.airport.route.as_ref() else {
        return Vec::new();
    };
    if r.path.len() < 2 {
        return Vec::new();
    }
    // Where the aircraft is along the path: the nearest segment.
    let at = f.tick.as_ref().map(|t| (t.lat, t.lon));
    let seg_len = |i: usize| {
        crate::nav::haversine_nm(r.path[i].0, r.path[i].1, r.path[i + 1].0, r.path[i + 1].1)
            * 1852.0
    };
    let nearest = at.and_then(|p| crate::efb::route_map::nearest_segment(&r.path, p));
    let mut rows: Vec<(String, bool, f64, usize)> = Vec::new(); // name, runway, length, last seg
    for i in 0..r.path.len() - 1 {
        let (name, rwy) = r
            .seg
            .get(i)
            .cloned()
            .unwrap_or_else(|| (String::new(), false));
        match rows.last_mut() {
            Some(last) if last.0 == name && last.1 == rwy => {
                last.2 += seg_len(i);
                last.3 = i;
            }
            _ => rows.push((name, rwy, seg_len(i), i)),
        }
    }
    let (here_seg, here_t) = nearest.unwrap_or((0, 0.0));
    // Metres from the aircraft to the end of segment `i`.
    let to_end = |i: usize| -> f64 {
        if i < here_seg {
            return 0.0;
        }
        let mut d = seg_len(here_seg) * (1.0 - here_t);
        for k in here_seg + 1..=i {
            d += seg_len(k);
        }
        d
    };
    let first_open = rows.iter().position(|x| x.3 >= here_seg);
    let mut out: Vec<LegRow> = rows
        .iter()
        .enumerate()
        .map(|(j, (name, rwy, len, last))| {
            let done = nearest.is_some() && *last < here_seg;
            let active = nearest.is_some() && first_open == Some(j);
            let id = if name.trim().is_empty() {
                "Link".to_string()
            } else if *rwy {
                format!("RWY {name}")
            } else {
                name.clone()
            };
            LegRow {
                id,
                meta: Some(if *rwy {
                    format!("Runway, {}", metres(*len))
                } else {
                    metres(*len)
                }),
                eta: if done {
                    Some("done".into())
                } else if nearest.is_some() {
                    Some(metres(to_end(*last)))
                } else {
                    None
                },
                state: if done {
                    "done"
                } else if active {
                    "active"
                } else {
                    "next"
                },
            }
        })
        .collect();
    if !r.holding_point.trim().is_empty() {
        out.push(LegRow {
            id: if r.to_stand {
                r.holding_point.clone()
            } else {
                format!("HOLD {}", r.holding_point)
            },
            meta: Some(if r.to_stand {
                "Stand".into()
            } else {
                match &f.airport.cleared_runway {
                    Some(rw) => format!("Hold short runway {rw}"),
                    None => "Holding point".into(),
                }
            }),
            eta: None,
            state: "next",
        });
    }
    out
}

fn taxi(f: &Facts, a: &AirportOut) -> FlightOut {
    let phase = f.flight.as_ref().map(|s| s.phase);
    let g = gs(f).map(|v| v.abs());
    let lead = match (phase, g) {
        (Some(Phase::TaxiOut) | Some(Phase::TaxiIn), Some(v)) => {
            format!("Now · taxiing, {v:.0} kt")
        }
        (Some(p), _) => format!("Now · {}", phase_label(p).to_lowercase()),
        (None, _) => "Now · on the ground".into(),
    };
    let rwy = if phase == Some(Phase::TaxiIn) {
        None
    } else {
        departure_runway(f)
    };
    let route = f.airport.route.as_ref();
    let card = a.card.as_ref();
    let headline = match (card, phase) {
        (Some(c), _) if c.kind == "X" => vec![plain("Off the route, stop and ask")],
        (Some(c), _) => {
            let words = c.text.clone();
            match words
                .rfind(c.label.as_str())
                .filter(|_| !c.label.is_empty())
            {
                Some(i) => {
                    let (a1, rest) = words.split_at(i);
                    let (lbl, b) = rest.split_at(c.label.len());
                    vec![
                        plain(format!("Next: {}", a1.to_lowercase().trim_start())),
                        fix(lbl),
                        plain(b),
                    ]
                }
                None => vec![plain(format!("Next: {words}"))],
            }
        }
        (None, Some(Phase::Holding)) => match &rwy {
            Some(r) => vec![plain("Holding short of runway "), fix(r.clone())],
            None => vec![plain("Holding")],
        },
        (None, Some(Phase::LinedUp)) => match &rwy {
            Some(r) => vec![plain("Lined up on runway "), fix(r.clone())],
            None => vec![plain("Lined up")],
        },
        (None, Some(Phase::TakeoffRoll)) => vec![plain("Takeoff roll")],
        (None, Some(Phase::RejectedTakeoff)) => vec![plain("Rejected takeoff, stopping")],
        _ => match route {
            Some(r) if !r.holding_point.is_empty() => {
                vec![plain("Taxi to "), fix(r.holding_point.clone())]
            }
            _ => vec![plain("No taxi route held")],
        },
    };
    let sub = route.map(|r| {
        let mut s = Vec::new();
        if !r.via.trim().is_empty() {
            s.push(format!("Route {}.", r.via.trim()));
        }
        if !r.to_stand {
            if let (Some(rw), false) = (&rwy, r.holding_point.is_empty()) {
                s.push(format!("Hold short of runway {rw} at {}.", r.holding_point));
            }
        } else {
            s.push(format!("To the stand {}.", r.holding_point));
        }
        s.join(" ")
    });
    let origin = f
        .plan
        .as_ref()
        .map(|p| p.origin.clone())
        .unwrap_or_default();
    let end = rwy
        .as_deref()
        .and_then(|r| runway_end(ends_at(f, &origin), r));
    let metar = metar_of(f, &origin);
    let remaining = a.route.as_ref().map(|r| r.remaining_m);
    let tiles = vec![
        Tile {
            label: "Next turn".into(),
            value: card.map(|c| {
                // "L B": the way and the name, as a strip reads it.
                let name = c.label.split_whitespace().last().unwrap_or("").to_string();
                let way = if c.turn_deg <= -20.0 {
                    "L "
                } else if c.turn_deg >= 20.0 {
                    "R "
                } else {
                    ""
                };
                match (c.kind.as_str(), name.is_empty()) {
                    ("X", _) => "Off route".to_string(),
                    (_, true) => c.text.clone(),
                    _ => format!("{way}{name}"),
                }
            }),
            unit: None,
            note: card.map(|c| crate::efb::airport::distance_words(c.dist_m)),
            tone: "mag",
        },
        Tile {
            label: if route.is_some_and(|r| r.to_stand) {
                "To the stand".into()
            } else {
                "To the hold".into()
            },
            value: remaining.map(|m| thousands((m / 10.0).round() * 10.0)),
            unit: remaining.map(|_| "m".into()),
            note: match (remaining, g) {
                (Some(m), Some(v)) if v > 3.0 => Some(format!(
                    "{}",
                    capitalise(&crate::efb::state::minutes_words(m / 1852.0 / v * 60.0))
                )),
                _ => None,
            },
            tone: "",
        },
        Tile {
            label: "Groundspeed".into(),
            value: g.map(|v| format!("{v:.0}")),
            unit: g.map(|_| "kt".into()),
            note: None,
            tone: "",
        },
        Tile {
            label: if phase == Some(Phase::TaxiIn) {
                "Stand".into()
            } else {
                "Departure runway".into()
            },
            value: if phase == Some(Phase::TaxiIn) {
                route
                    .map(|r| r.holding_point.clone())
                    .filter(|s| !s.is_empty())
            } else {
                rwy.clone()
            },
            unit: None,
            note: match (end, metar.as_ref()) {
                (Some(e), Some(m)) => {
                    Some(format!("{}, wind {}", metres(e.length_m), wind_words(m)))
                }
                (Some(e), None) => Some(metres(e.length_m)),
                (None, Some(m)) => Some(format!("Wind {}", wind_words(m))),
                _ => None,
            },
            tone: if rwy.is_some() { "ok" } else { "" },
        },
    ];
    let mut cards = vec![Card::legs(
        "Taxi route",
        taxi_legs(f),
        "No taxi route held: the tower's or the crew's appears here.",
    )];
    match checklist_card(f) {
        Some(c) => cards.push(c),
        None if phase != Some(Phase::TaxiIn) => cards.push(runway_card(f)),
        None => {}
    }
    FlightOut {
        layout: "taxi",
        lead,
        who: who_line(f),
        headline,
        sub,
        tiles,
        cards,
    }
}

// ---------------------------------------------------------------- cruise

/// The index of the follower's active fix in the plan's legs: the first leg
/// with its ident (the Plan page's rule).
pub(crate) fn active_leg_index(f: &Facts) -> Option<usize> {
    let (fixid, _) = f.active_fix.as_ref()?;
    f.plan
        .as_ref()?
        .waypoints
        .iter()
        .position(|w| w.0.eq_ignore_ascii_case(fixid))
}

/// Miles to each leg from the aircraft, from the active fix on.
fn leg_distances(f: &Facts, p: &PlanFacts, from: usize) -> Vec<f64> {
    let t = f.tick.as_ref();
    let first = f.active_fix.as_ref().and_then(|(_, d)| *d).or_else(|| {
        let w = p.waypoints.get(from)?;
        t.map(|t| crate::nav::haversine_nm(t.lat, t.lon, w.1, w.2))
    });
    let Some(mut d) = first else {
        return Vec::new();
    };
    let mut out = vec![d];
    for i in from + 1..p.waypoints.len() {
        let (a, b) = (&p.waypoints[i - 1], &p.waypoints[i]);
        d += crate::nav::haversine_nm(a.1, a.2, b.1, b.2);
        out.push(d);
    }
    out
}

fn cruise(f: &Facts) -> FlightOut {
    let s = f.flight.as_ref();
    let phase = s.map(|s| s.phase);
    let lead = format!(
        "Now · {}{}{}",
        phase
            .map(|p| phase_label(p).to_lowercase())
            .unwrap_or_default(),
        s.filter(|s| s.alt_ft > 0.0)
            .map(|s| format!(" {}", feet(s.alt_ft)))
            .unwrap_or_default(),
        ias(f).map(|v| format!(", {v:.0} kt")).unwrap_or_default()
    );
    let lateral = match f.lateral {
        Some(true) => Some("LNAV flies the route".to_string()),
        Some(false) => Some("The follower steers the heading bug".to_string()),
        None => None,
    };
    let who = match (lateral.clone(), who_line(f)) {
        (Some(l), Some(w)) => Some(format!("{w}, {}", l.to_lowercase())),
        (l, w) => w.or(l),
    };
    let plan = f.plan.as_ref();
    let active = active_leg_index(f);
    let dists = match (plan, active) {
        (Some(p), Some(i)) => leg_distances(f, p, i),
        _ => Vec::new(),
    };
    let rem = remaining_nm(f);
    let next_fix = f.active_fix.as_ref().map(|(n, _)| n.clone());
    let next_d = dists.first().copied();
    let headline = match (&next_fix, next_d) {
        (Some(n), Some(d)) => {
            let mut h = vec![
                plain("Next "),
                fix(n.clone()),
                plain(" in "),
                num(format!("{d:.0} nm")),
            ];
            if let Some(t) = eta(f, d) {
                h.push(plain(", at "));
                h.push(num(zulu(t)));
            }
            h
        }
        (Some(n), None) => vec![plain("Next "), fix(n.clone())],
        _ => match (plan, rem) {
            (Some(p), Some(r)) => vec![
                fix(p.destination.clone()),
                plain(" in "),
                num(format!("{r:.0} nm")),
            ],
            (Some(p), None) => vec![plain("Bound for "), fix(p.destination.clone())],
            _ => vec![plain("No plan: the follower has no route")],
        },
    };
    let mut sub = Vec::new();
    if let (Some(p), Some(i)) = (plan, active) {
        if let Some(w) = p.waypoints.get(i + 1) {
            sub.push(format!("Then {}.", w.0));
        }
    }
    let tod = f.tod.as_ref();
    match tod {
        Some(t) if t.held_ft.is_some() => sub.push(format!(
            "Holding {} above the plan's descent.",
            feet(t.held_ft.unwrap_or(0.0))
        )),
        Some(t) if !t.started && t.to_tod_nm > 0.0 => sub.push(format!(
            "Top of descent in {:.0} nm{}.",
            t.to_tod_nm,
            gs(f)
                .filter(|g| *g > 40.0)
                .map(|g| format!(
                    ", {}",
                    crate::efb::state::minutes_words(t.to_tod_nm / g * 60.0)
                ))
                .unwrap_or_default()
        )),
        Some(t) if t.started => sub.push(format!(
            "Descending to {} for {}.",
            feet(t.target_ft),
            t.fix
        )),
        _ => {}
    }
    // Fuel at the destination at this burn: what the tick holds now, less
    // the endurance it has used by then.
    let tk = f.tick.as_ref();
    let dest_eta_min = match (rem, gs(f)) {
        (Some(r), Some(g)) if g > 40.0 => Some(r / g * 60.0),
        _ => None,
    };
    let fuel_at_dest = match (
        tk.and_then(|t| t.fuel_kg),
        tk.and_then(|t| t.endurance_min),
        dest_eta_min,
    ) {
        (Some(fuel), Some(end), Some(m)) if end > 0.0 && end < 6000.0 => {
            Some(fuel * (1.0 - m / end))
        }
        _ => None,
    };
    let plan_arrival = plan
        .filter(|p| p.block_fuel_kg > 0.0 && p.trip_fuel_kg > 0.0)
        .map(|p| p.block_fuel_kg - p.trip_fuel_kg);
    let reserve = plan.map(|p| p.reserve_kg).filter(|r| *r > 0.0);
    let tiles = vec![
        Tile {
            label: next_fix.clone().unwrap_or_else(|| "Next fix".into()),
            value: next_d.map(|d| format!("{d:.0}")),
            unit: next_d.map(|_| "nm".into()),
            note: next_d
                .and_then(|d| eta(f, d))
                .map(|t| format!("ETA {}", zulu(t))),
            tone: "mag",
        },
        Tile {
            label: "Top of descent".into(),
            value: tod
                .filter(|t| !t.started && t.held_ft.is_none() && t.to_tod_nm > 0.0)
                .and_then(|t| {
                    gs(f)
                        .filter(|g| *g > 40.0)
                        .map(|g| format!("{:.0}", t.to_tod_nm / g * 60.0))
                }),
            unit: tod
                .filter(|t| !t.started && t.held_ft.is_none())
                .and_then(|_| gs(f).filter(|g| *g > 40.0))
                .map(|_| "min".into()),
            note: tod.map(|t| {
                if t.started {
                    format!("Descending for {}", t.fix)
                } else {
                    format!(
                        "{:.0} nm, for {} at {}",
                        t.to_tod_nm,
                        t.fix,
                        feet(t.target_ft)
                    )
                }
            }),
            tone: "",
        },
        Tile {
            label: plan
                .map(|p| p.destination.clone())
                .unwrap_or_else(|| "Destination".into()),
            value: dest_eta_min.map(|m| zulu(f.now_t + m * 60.0)),
            unit: None,
            note: rem.map(|r| format!("{} nm to go", thousands(r))),
            tone: "",
        },
        Tile {
            label: format!(
                "Fuel at {}",
                plan.map(|p| p.destination.clone())
                    .unwrap_or_else(|| "destination".into())
            ),
            value: fuel_at_dest.map(thousands),
            unit: fuel_at_dest.map(|_| "kg".into()),
            note: match (plan_arrival, reserve) {
                (Some(a), Some(r)) => {
                    Some(format!("Plan {}, reserve {}", thousands(a), thousands(r)))
                }
                (Some(a), None) => Some(format!("Plan {}", thousands(a))),
                _ => None,
            },
            tone: match (fuel_at_dest, reserve) {
                (Some(v), Some(r)) if v < r => "warn",
                (Some(_), Some(_)) => "ok",
                _ => "",
            },
        },
    ];
    // The route ahead: the leg just flown, the active one, four to come.
    let mut legs = Vec::new();
    if let Some(p) = plan {
        let from = active.unwrap_or(0);
        let start = from.saturating_sub(1);
        for (i, w) in p.waypoints.iter().enumerate().skip(start).take(6) {
            let stage = p
                .stages
                .iter()
                .find(|(id, _)| id.eq_ignore_ascii_case(&w.0))
                .map(|(_, s)| *s);
            let mut meta = Vec::new();
            if w.3 > 0.0 {
                meta.push(feet(w.3));
            }
            if let Some(st) = stage {
                meta.push(st.to_string());
            }
            if tod.is_some_and(|t| t.fix.eq_ignore_ascii_case(&w.0)) {
                meta.push("descent target".into());
            }
            let d = (active.is_some() && i >= from)
                .then(|| dists.get(i - from).copied())
                .flatten();
            legs.push(LegRow {
                id: w.0.clone(),
                meta: (!meta.is_empty()).then(|| meta.join(" · ")),
                eta: match d {
                    Some(d) => eta(f, d).map(zulu).or(Some(format!("{d:.0} nm"))),
                    None => None,
                },
                state: match active {
                    Some(a) if i < a => "done",
                    Some(a) if i == a => "active",
                    _ => "next",
                },
            });
        }
    }
    let dest = plan.map(|p| p.destination.clone()).unwrap_or_default();
    let dest_metar = f.metars.iter().find(|m| m.icao.eq_ignore_ascii_case(&dest));
    let mut rows = vec![
        Row {
            k: "Speed".into(),
            v: match (ias(f), gs(f)) {
                (Some(i), Some(g)) => Some(format!("{i:.0} kt IAS · {g:.0} GS")),
                (Some(i), None) => Some(format!("{i:.0} kt IAS")),
                _ => None,
            },
            tone: "",
        },
        Row {
            k: "Lateral".into(),
            v: lateral,
            tone: "ok",
        },
        Row {
            k: format!(
                "{} weather",
                if dest.is_empty() {
                    "Destination"
                } else {
                    &dest
                }
            ),
            v: dest_metar.and_then(|m| m.raw.as_deref()).map(short_metar),
            tone: "",
        },
        Row {
            k: "Arrival runway".into(),
            v: arrival_runway(f).map(|r| match plan.and_then(|p| some(&p.star)) {
                Some(s) => format!("{r}, {s}"),
                None => r,
            }),
            tone: "mag",
        },
    ];
    if s.is_some_and(|s| s.phase == Phase::Hover) {
        rows.retain(|r| r.k != "Lateral");
    }
    let mut cards = vec![
        Card::legs("Route ahead", legs, "No plan legs: load a flight plan."),
        Card::rows("Flight", rows),
    ];
    if let Some(c) = checklist_card(f) {
        cards.push(c);
    }
    FlightOut {
        layout: "cruise",
        lead,
        who,
        headline,
        sub: (!sub.is_empty()).then(|| sub.join(" ")),
        tiles,
        cards,
    }
}

/// A METAR without its station and time: "080/12 9999 BKN035" style.
fn short_metar(raw: &str) -> String {
    let toks: Vec<&str> = raw.split_whitespace().collect();
    let skip = toks
        .iter()
        .take(3)
        .filter(|t| {
            t.len() == 4 && t.chars().all(|c| c.is_ascii_uppercase())
                || t.ends_with('Z') && t.len() == 7
                || **t == "METAR"
                || **t == "SPECI"
        })
        .count();
    toks[skip..]
        .iter()
        .take(6)
        .copied()
        .collect::<Vec<_>>()
        .join(" ")
}

// ---------------------------------------------------------------- approach

fn approach(f: &Facts) -> FlightOut {
    let s = f.flight.as_ref();
    let phase = s.map(|s| s.phase);
    let lead = format!(
        "Now · {}{}",
        phase
            .map(|p| phase_label(p).to_lowercase())
            .unwrap_or_default(),
        s.filter(|s| s.alt_ft > 0.0 && s.phase.airborne())
            .map(|s| format!(", {}", feet(s.alt_ft)))
            .unwrap_or_default()
    );
    let tower = s
        .and_then(|s| s.situation.clearance.as_ref())
        .map(tower_state);
    let plan = f.plan.as_ref();
    let dest = plan.map(|p| p.destination.clone()).unwrap_or_default();
    let rwy = arrival_runway(f);
    let end = rwy
        .as_deref()
        .and_then(|r| runway_end(ends_at(f, &dest), r));
    let metar = metar_of(f, &dest);
    let headline = match (&rwy, dest.is_empty()) {
        (Some(r), false) => vec![
            plain("Runway "),
            fix(r.clone()),
            plain(format!(" at {dest}")),
        ],
        (Some(r), true) => vec![plain("Runway "), fix(r.clone())],
        (None, false) => vec![plain("Arriving "), fix(dest.clone())],
        (None, true) => vec![plain("Descending")],
    };
    let mut sub = Vec::new();
    if let Some(t) = &tower {
        sub.push(format!("{t}."));
    }
    if let Some(star) = plan.and_then(|p| some(&p.star)) {
        sub.push(format!("Arrival {star}."));
    }
    if let Some(m) = &metar {
        sub.push(crate::efb::state::metar_plain(m));
    }
    let need = f.landing_need_m;
    let tiles = vec![
        Tile {
            label: "Runway".into(),
            value: rwy.clone(),
            unit: None,
            note: end.map(|e| format!("{} long", metres(e.length_m))),
            tone: "mag",
        },
        Tile {
            label: "Wind".into(),
            value: metar.as_ref().map(|m| match (m.wind_dir, m.wind_kts) {
                (_, 0) => "Calm".into(),
                (None, k) => format!("VRB/{k}"),
                (Some(d), k) => format!("{d:03}/{k}"),
            }),
            unit: None,
            note: metar
                .as_ref()
                .and_then(|m| end.and_then(|e| components_words(m, e))),
            tone: "",
        },
        Tile {
            label: "QNH".into(),
            value: metar
                .as_ref()
                .and_then(|m| m.qnh_hpa)
                .map(|q| q.to_string()),
            unit: None,
            note: metar.as_ref().map(|_| format!("{dest} METAR")),
            tone: "",
        },
        Tile {
            label: "Landing distance".into(),
            value: need.map(|n| thousands((n / 10.0).round() * 10.0)),
            unit: need.map(|_| "m".into()),
            note: end.map(|e| format!("Of {} available", metres(e.length_m))),
            tone: match (need, end) {
                (Some(n), Some(e)) if n > e.length_m => "warn",
                (Some(_), Some(_)) => "ok",
                _ => "",
            },
        },
    ];
    let mut cards = Vec::new();
    if let (Some(n), Some(e)) = (need, end) {
        let ok = n <= e.length_m;
        cards.push(Card {
            title: "Stopping margin".into(),
            kind: "bar",
            rows: vec![
                Row {
                    k: "Needed, the book figure with margin".into(),
                    v: Some(metres(n)),
                    tone: "",
                },
                Row {
                    k: "Margin".into(),
                    v: Some(metres(e.length_m - n)),
                    tone: if ok { "ok" } else { "warn" },
                },
            ],
            legs: Vec::new(),
            bar: Some(Bar {
                pct: (n / e.length_m * 100.0).clamp(0.0, 100.0),
                label: format!("{} needed of {}", metres(n), metres(e.length_m)),
                ok,
            }),
            empty: None,
        });
    } else {
        cards.push(Card::rows(
            "Arrival",
            vec![
                Row {
                    k: "STAR".into(),
                    v: plan.and_then(|p| some(&p.star)),
                    tone: "mag",
                },
                Row {
                    k: "Tower".into(),
                    v: tower.clone(),
                    tone: "",
                },
                Row {
                    k: "Landing distance".into(),
                    v: need.map(metres),
                    tone: "",
                },
                Row {
                    k: "Runway length".into(),
                    v: end.map(|e| metres(e.length_m)),
                    tone: "",
                },
            ],
        ));
    }
    if let Some(c) = checklist_card(f) {
        cards.push(c);
    }
    FlightOut {
        layout: "approach",
        lead,
        who: who_line(f),
        headline,
        sub: (!sub.is_empty()).then(|| sub.join(" ")),
        tiles,
        cards,
    }
}

/// The Flight tab for the phase. Pure.
pub(crate) fn build(f: &Facts, a: &AirportOut) -> FlightOut {
    match layout_for(f.flight.as_ref().map(|s| s.phase)) {
        "taxi" => taxi(f, a),
        "cruise" => cruise(f),
        "approach" => approach(f),
        _ => parked(f),
    }
}

/// The one suggested action for the phase, beside MY CONTROLS: pilot
/// English through `/say` like every button. Pure.
pub(crate) fn action(f: &Facts) -> Option<Button> {
    let layout = layout_for(f.flight.as_ref().map(|s| s.phase));
    let crew = f.owner.is_some() || f.mission.is_some();
    let b = |id: &str| Some(Button::of(phrase(id)));
    match layout {
        "parked" => {
            if f.plan.is_none() {
                b("fetch_plan")
            } else if !crew {
                b("fly_plan")
            } else {
                b("status")
            }
        }
        "taxi" => {
            let ready = matches!(
                f.flight.as_ref().map(|s| s.phase),
                Some(Phase::Holding) | Some(Phase::TaxiOut)
            );
            match (ready && !crew, f.station.as_ref(), f.com1_tag) {
                (true, Some(st), Some(tag)) => {
                    station_ready_for_departure(&st.field, tag).map(|w| Button {
                        id: "ready_dep".into(),
                        label: "Ready for departure".into(),
                        say: w,
                    })
                }
                _ => b("status"),
            }
        }
        "cruise" => b("tod"),
        _ => b("go_around"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::efb::state::tests::cruise as cruise_facts;

    /// Every flight phase reads as one of four layouts, in a flight's order.
    #[test]
    fn each_phase_has_its_layout() {
        use Phase::*;
        let cases = [
            (None, "parked"),
            (Some(Parked), "parked"),
            (Some(EngineStart), "parked"),
            (Some(Ready), "parked"),
            (Some(Pushback), "parked"),
            (Some(TaxiOut), "taxi"),
            (Some(Holding), "taxi"),
            (Some(LinedUp), "taxi"),
            (Some(TakeoffRoll), "taxi"),
            (Some(RejectedTakeoff), "taxi"),
            (Some(InitialClimb), "cruise"),
            (Some(Climb), "cruise"),
            (Some(Cruise), "cruise"),
            (Some(Hover), "cruise"),
            (Some(Descent), "approach"),
            (Some(Approach), "approach"),
            (Some(GoAround), "approach"),
            (Some(Landing), "approach"),
            (Some(Rollout), "approach"),
            (Some(TaxiIn), "taxi"),
        ];
        for (p, want) in cases {
            assert_eq!(layout_for(p), want, "{p:?}");
        }
    }

    #[test]
    fn the_wind_splits_on_the_runway() {
        let (h, c) = wind_components(80.0, 12.0, 78.0);
        assert!((h - 12.0).abs() < 0.1 && c.abs() < 0.5);
        let (h, c) = wind_components(170.0, 10.0, 80.0);
        assert!(h.abs() < 0.1 && (c - 10.0).abs() < 0.1, "a right crosswind");
        let (h, _) = wind_components(260.0, 8.0, 80.0);
        assert!(h < -7.9, "a tailwind");
        assert_eq!(
            short_metar("EGKK 061020Z 21019KT 9999 BKN014 14/11 Q1009"),
            "21019KT 9999 BKN014 14/11 Q1009"
        );
    }

    /// The cruise: the next fix with its distance and ETA, the top of
    /// descent, the destination's ETA, fuel there; the route ahead with the
    /// active fix; LNAV named when it flies.
    #[test]
    fn the_cruise_layout_reads_the_follower_and_the_plan() {
        let mut f = cruise_facts();
        f.lateral = Some(true);
        let a = crate::efb::airport::page(&f.airport, f.tick.as_ref(), f.plan.as_ref(), true);
        let out = build(&f, &a);
        assert_eq!(out.layout, "cruise");
        let head: String = out.headline.iter().map(|s| s.t.as_str()).collect();
        assert!(head.starts_with("Next NORMI in 64 nm, at "), "{head}");
        assert_eq!(out.headline[1].k, "fix");
        assert_eq!(out.tiles[0].label, "NORMI");
        assert_eq!(out.tiles[0].value.as_deref(), Some("64"));
        assert_eq!(out.tiles[1].value.as_deref(), Some("5"), "42 nm at 461 kt");
        assert!(out.tiles[2]
            .value
            .as_deref()
            .is_some_and(|v| v.ends_with('Z')));
        assert!(
            out.tiles[3].value.is_some(),
            "fuel at the destination at this burn"
        );
        assert_eq!(
            out.tiles[3].note.as_deref(),
            Some("Plan 4,900, reserve 2,600")
        );
        let legs = &out.cards[0].legs;
        assert_eq!(legs[0].id, "BELEN");
        assert_eq!(legs[0].state, "done");
        assert_eq!(legs[1].state, "active");
        assert_eq!(legs[1].id, "NORMI");
        let rows = &out.cards[1].rows;
        assert_eq!(
            rows.iter()
                .find(|r| r.k == "Lateral")
                .and_then(|r| r.v.as_deref()),
            Some("LNAV flies the route")
        );
        assert_eq!(
            rows.iter()
                .find(|r| r.k == "Arrival runway")
                .and_then(|r| r.v.as_deref()),
            Some("26L, BOGNA1G")
        );
        assert_eq!(
            out.cards[2].kind, "checklist",
            "the running list rides the Flight tab"
        );
        assert_eq!(action(&f).map(|b| b.id), Some("tod".into()));
        f.lateral = Some(false);
        let out = build(&f, &a);
        assert!(out
            .who
            .as_deref()
            .is_some_and(|w| w.contains("heading bug")));
    }

    /// Parked with a plan and no crew: the clearance to ask for, the plan's
    /// numbers, "fly the flight plan" offered; no plan, "fetch" offered.
    #[test]
    fn the_ramp_layout_offers_what_comes_next() {
        let mut f = cruise_facts();
        f.flight.as_mut().unwrap().phase = Phase::Parked;
        f.flight.as_mut().unwrap().situation.clearance = None;
        f.ifr = None;
        f.owner = None;
        f.mission = None;
        f.auto = String::new();
        f.flight.as_mut().unwrap().control = crate::flight::Control::Pilot;
        f.checklist = None;
        f.tick.as_mut().unwrap().on_ground = true;
        f.takeoff_need_m = Some(1_421.0);
        let a = crate::efb::airport::page(&f.airport, f.tick.as_ref(), f.plan.as_ref(), false);
        let out = build(&f, &a);
        assert_eq!(out.layout, "parked");
        let head: String = out.headline.iter().map(|s| s.t.as_str()).collect();
        assert_eq!(head, "LEMG to EGKK, no clearance yet");
        assert_eq!(out.tiles[0].note.as_deref(), Some("Plan 14,200 kg"));
        assert_eq!(out.tiles[1].value.as_deref(), Some("66,100"));
        assert_eq!(out.tiles[3].value, None, "no squawk until cleared");
        assert_eq!(out.cards[0].title, "Departure");
        assert_eq!(out.cards[1].title, "Runway 13");
        let need = out.cards[1].rows.iter().find(|r| r.k == "Needed").unwrap();
        assert_eq!(
            need.v.as_deref(),
            Some("1,420 m"),
            "no diagram: no fit claimed"
        );
        assert_eq!(
            out.cards[1]
                .rows
                .iter()
                .find(|r| r.k == "QNH")
                .and_then(|r| r.v.as_deref()),
            Some("1018")
        );
        assert_eq!(action(&f).map(|b| b.id), Some("fly_plan".into()));
        f.plan = None;
        assert_eq!(action(&f).map(|b| b.id), Some("fetch_plan".into()));
        // An IFR clearance in hand is the headline.
        let mut f = cruise_facts();
        f.flight.as_mut().unwrap().phase = Phase::Ready;
        let out = build(&f, &a);
        let head: String = out.headline.iter().map(|s| s.t.as_str()).collect();
        assert_eq!(head, "IFR clearance to EGKK held, runway 13, squawk 4521");
        assert_eq!(out.headline[1].k, "num");
    }

    /// The approach: the runway, the wind on it, the QNH, the landing
    /// distance against the strip as a margin bar.
    #[test]
    fn the_approach_layout_weighs_the_runway() {
        let mut f = cruise_facts();
        f.flight.as_mut().unwrap().phase = Phase::Approach;
        f.landing_need_m = Some(1_650.0);
        f.airport.diagram = Some(crate::efb::airport::DiagramRef {
            icao: "EGKK".into(),
            name: "Gatwick".into(),
            version: "v".into(),
            lat0: 51.15,
            lon0: -0.19,
            runways: vec![RunwayEnd {
                ident: "26L".into(),
                length_m: 3_159.0,
                heading_true: 257.8,
            }],
            stands: Vec::new(),
        });
        let a = crate::efb::airport::page(&f.airport, f.tick.as_ref(), f.plan.as_ref(), true);
        let out = build(&f, &a);
        assert_eq!(out.layout, "approach");
        let head: String = out.headline.iter().map(|s| s.t.as_str()).collect();
        assert_eq!(head, "Runway 26L at EGKK");
        assert_eq!(out.tiles[0].note.as_deref(), Some("3,160 m long"));
        assert_eq!(out.tiles[1].value.as_deref(), Some("210/19"));
        assert_eq!(out.tiles[1].note.as_deref(), Some("Headwind 13, cross 14"));
        assert_eq!(out.tiles[2].value.as_deref(), Some("1009"));
        assert_eq!(out.tiles[3].tone, "ok");
        let bar = out.cards[0].bar.as_ref().expect("a margin bar");
        assert!(bar.ok && (bar.pct - 52.2).abs() < 0.5);
        assert_eq!(out.cards[1].kind, "checklist");
        assert_eq!(action(&f).map(|b| b.id), Some("go_around".into()));
    }
}
