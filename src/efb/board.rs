//! The Plan tab's flight information board (2026-10-13: the pilot wanted
//! the panel's Board tab on the EFB too): every field our traffic world
//! simulates, Departures and Arrivals, read-only.
//!
//! No new source: the traffic world taps each simulated field's board
//! payload (`traffic::board::payload`, the same rows the plugin gets on
//! `xcopilot/status/board`) into `feed::note_boards` as it ticks, and this
//! is the pure reading of it. The page picks the field itself (the fields
//! are all in the state), so the panel's own pick is left alone. No world
//! running: `fields` is `None` and the page shows a dash, never an empty
//! table.

use serde::Serialize;

/// One flight as the board draws it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct BoardRow {
    /// The timetable's time, "HH:MM" local.
    pub(crate) sched: String,
    /// The expected (or actual) time, "On time" where it equals the
    /// timetable's (as an airport board shows it), empty where unknown.
    pub(crate) exp: String,
    pub(crate) on_time: bool,
    pub(crate) number: String,
    pub(crate) airline: String,
    /// The other end: To on a departure, From on an arrival.
    pub(crate) city: String,
    pub(crate) aircraft: String,
    pub(crate) stand: String,
    pub(crate) status: String,
    /// The status's colour on the page, as the panel's Board tab has it:
    /// "warn" Delayed, "active" Boarding or Approaching, "plain" Scheduled,
    /// "done" At gate, "ok" the rest.
    pub(crate) tone: &'static str,
    /// Our own flight.
    pub(crate) own: bool,
}

/// One field's board.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct FieldBoard {
    pub(crate) icao: String,
    /// The field's local time as the board stamped it, "HH:MM".
    pub(crate) clock: Option<String>,
    pub(crate) departures: Vec<BoardRow>,
    pub(crate) arrivals: Vec<BoardRow>,
}

/// The board card: `None` with no traffic world running (a dash).
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct BoardOut {
    pub(crate) fields: Option<Vec<FieldBoard>>,
}

fn tone(status: &str) -> &'static str {
    match status {
        "Delayed" => "warn",
        "Boarding" | "Approaching" => "active",
        "Scheduled" => "plain",
        "At gate" => "done",
        _ => "ok",
    }
}

/// One field's board from its payload ("H|field|HH:MM[|choices]" then
/// "D" or "A"|sched|time|number|airline|city|type|stand|status|own rows).
/// An empty payload (nothing to show) is a field with no rows.
pub(crate) fn parse(icao: &str, payload: &str) -> FieldBoard {
    let mut out = FieldBoard {
        icao: icao.to_string(),
        clock: None,
        departures: Vec::new(),
        arrivals: Vec::new(),
    };
    for line in payload.lines() {
        let f: Vec<&str> = line.split('|').collect();
        match f.first() {
            Some(&"H") if f.len() >= 3 => {
                out.clock = Some(f[2].to_string()).filter(|c| !c.is_empty());
            }
            Some(&tag @ ("D" | "A")) if f.len() >= 10 => {
                let on_time = !f[1].is_empty() && f[2] == f[1];
                let row = BoardRow {
                    sched: f[1].to_string(),
                    exp: if on_time {
                        "On time".to_string()
                    } else {
                        f[2].to_string()
                    },
                    on_time,
                    number: f[3].to_string(),
                    airline: f[4].to_string(),
                    city: f[5].to_string(),
                    aircraft: f[6].to_string(),
                    stand: f[7].to_string(),
                    status: f[8].to_string(),
                    tone: tone(f[8]),
                    own: f[9] == "1",
                };
                if tag == "D" {
                    out.departures.push(row);
                } else {
                    out.arrivals.push(row);
                }
            }
            _ => {}
        }
    }
    out
}

/// The card from the world's boards, by field (`None`: no world running).
pub(crate) fn build(boards: Option<&[(String, String)]>) -> BoardOut {
    BoardOut {
        fields: boards.map(|b| b.iter().map(|(icao, p)| parse(icao, p)).collect()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// LEMG's board as the traffic world publishes it.
    const LEMG: &str = "H|LEMG|10:42\n\
        D|10:30|10:30|U2 8062|easyJet|London Gatwick|A320|42|Taxiing|0\n\
        D|11:05|11:25|FR 1234|Ryanair|Dublin|B738||Delayed|0\n\
        D|11:20||WAT 519|Watchtower|Gibraltar|C172|G3|Scheduled|1\n\
        A|10:55|10:51|BA 492|British Airways|London Heathrow|A321||Approaching|0";

    #[test]
    fn no_traffic_world_is_a_dash_not_an_empty_board() {
        assert_eq!(build(None).fields, None);
        // A world with a field and nothing on its board is a field, empty.
        let b = build(Some(&[("LXGB".to_string(), String::new())]));
        let f = b.fields.unwrap();
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].icao, "LXGB");
        assert!(f[0].departures.is_empty() && f[0].arrivals.is_empty());
    }

    #[test]
    fn the_board_reads_the_panels_rows() {
        let f = parse("LEMG", LEMG);
        assert_eq!(f.clock.as_deref(), Some("10:42"));
        assert_eq!(f.departures.len(), 3);
        assert_eq!(f.arrivals.len(), 1);
        let ezy = &f.departures[0];
        assert_eq!(
            (ezy.number.as_str(), ezy.city.as_str(), ezy.stand.as_str()),
            ("U2 8062", "London Gatwick", "42")
        );
        // An expected time equal to the timetable's reads "On time".
        assert_eq!(ezy.exp, "On time");
        assert!(ezy.on_time);
        assert_eq!(ezy.tone, "ok");
        let fr = &f.departures[1];
        assert_eq!(
            (fr.exp.as_str(), fr.on_time, fr.tone),
            ("11:25", false, "warn")
        );
        // No estimate is a blank, never "On time"; our own flight is marked.
        let own = &f.departures[2];
        assert_eq!((own.exp.as_str(), own.on_time, own.own), ("", false, true));
        assert_eq!(f.arrivals[0].tone, "active");
    }

    #[test]
    fn every_simulated_field_is_on_the_card_in_order() {
        let b = build(Some(&[
            ("LEMG".to_string(), LEMG.to_string()),
            (
                "LXGB".to_string(),
                "H|LXGB|10:42\nA|11:00|11:00|BA 490|British Airways|London Heathrow|A320||Scheduled|0"
                    .to_string(),
            ),
        ]));
        let icaos: Vec<String> = b.fields.unwrap().into_iter().map(|f| f.icao).collect();
        assert_eq!(icaos, ["LEMG", "LXGB"]);
    }
}
