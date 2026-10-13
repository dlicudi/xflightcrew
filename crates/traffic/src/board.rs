//! The flight information board (2026-10-13): Departures and Arrivals at
//! the field as an airport's FIDS shows them, each flight's live status
//! derived from where its aircraft is ([`Status::of`]), packed into the one
//! text payload the plugin's Board tab draws ([`payload`]). Pure: the bin
//! looks the aircraft up and publishes the text.

use crate::phase::Phase;
use crate::rotation::hhmm;
use crate::schedule::FlightInfo;

/// Which half of the board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    Departure,
    Arrival,
}

/// One flight on the board: its number, its aircraft type and the board's
/// line ([`FlightInfo`]), and the callsign its aircraft flies under.
#[derive(Debug, Clone, PartialEq)]
pub struct BoardFlight {
    pub dir: Dir,
    pub callsign: String,
    pub icao_type: String,
    pub info: FlightInfo,
    /// Our own flight (from its plan).
    pub own: bool,
}

/// Where a flight's aircraft is, as the board needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seen {
    pub phase: Phase,
    /// Parked and going nowhere yet: boarding, or turned round.
    pub dormant: bool,
}

/// A flight's status, as a board says it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Scheduled,
    Boarding,
    Taxiing,
    Departed,
    Approaching,
    Landed,
    AtGate,
    Delayed,
}

/// A departure is boarding this long before it is off the blocks.
pub const BOARDING_S: f64 = 40.0 * 60.0;

/// A flight this late on its timetable is Delayed until it moves.
pub const DELAYED_S: f64 = 15.0 * 60.0;

/// A departure that left this long ago leaves the board, as a FIDS drops
/// it once its gate has closed behind it.
pub const DEPARTED_KEPT_S: f64 = 30.0 * 60.0;

/// An arrival that landed this long ago leaves the board: kept longer, for
/// the people meeting it.
pub const LANDED_KEPT_S: f64 = 60.0 * 60.0;

/// The board shows the flights of the next twelve hours and no further, so
/// every "HH:MM" on it names one time: today's or the next morning's
/// (2026-10-13, LXGB at 17:48: the timetable's next 24 h put tomorrow's
/// whole day on the board, read as today's flights still pending).
pub const AHEAD_S: f64 = 12.0 * 3600.0;

impl Status {
    pub fn word(self) -> &'static str {
        match self {
            Status::Scheduled => "Scheduled",
            Status::Boarding => "Boarding",
            Status::Taxiing => "Taxiing",
            Status::Departed => "Departed",
            Status::Approaching => "Approaching",
            Status::Landed => "Landed",
            Status::AtGate => "At gate",
            Status::Delayed => "Delayed",
        }
    }

    /// The status of `f` at local second `now_s`, its aircraft `seen`
    /// (under the flight's callsign, or, for a departure not yet off, its
    /// inbound's), else nothing: a departure dormant on its stand is
    /// Boarding inside [`BOARDING_S`] of its estimate, moving on the ground
    /// Taxiing, airborne Departed; an arrival in the air Approaching, on the
    /// runway or taxiing in Landed, on its stand At gate. With no aircraft
    /// the clock decides: past its estimate it has gone (Departed, Landed),
    /// else Scheduled, or Delayed while [`DELAYED_S`] behind its timetable.
    pub fn of(f: &BoardFlight, seen: Option<Seen>, now_s: f64) -> Status {
        let est = f.info.est_s.or(f.info.sched_s);
        let late = match (f.info.sched_s, f.info.est_s) {
            (Some(s), Some(e)) => e - s >= DELAYED_S,
            _ => false,
        };
        let waiting = || {
            if late {
                Status::Delayed
            } else {
                Status::Scheduled
            }
        };
        match (f.dir, seen) {
            (Dir::Departure, Some(s)) if s.dormant => {
                if est.is_none_or(|e| e - now_s <= BOARDING_S) {
                    Status::Boarding
                } else {
                    waiting()
                }
            }
            (Dir::Departure, Some(s)) if s.phase.airborne() || s.phase == Phase::Gone => {
                Status::Departed
            }
            (Dir::Departure, Some(_)) => Status::Taxiing,
            (Dir::Departure, None) => match est {
                Some(e) if e < now_s => Status::Departed,
                _ => waiting(),
            },
            (Dir::Arrival, Some(s)) if s.dormant || s.phase == Phase::Parked => Status::AtGate,
            (Dir::Arrival, Some(s)) if s.phase.airborne() => Status::Approaching,
            (Dir::Arrival, Some(_)) => Status::Landed,
            (Dir::Arrival, None) => match est {
                Some(e) if e < now_s => Status::Landed,
                _ => waiting(),
            },
        }
    }

    /// The status of `f` at a field whose aircraft are not simulated (the
    /// Board tab's other fields, 2026-10-13): the clock alone. A departure
    /// past its estimate has gone, inside [`BOARDING_S`] of it is Boarding
    /// (at its planned stand); an arrival past its estimate has Landed;
    /// else Scheduled, or Delayed while [`DELAYED_S`] behind its timetable.
    pub fn by_clock(f: &BoardFlight, now_s: f64) -> Status {
        let est = f.info.est_s.or(f.info.sched_s);
        let late = match (f.info.sched_s, f.info.est_s) {
            (Some(s), Some(e)) => e - s >= DELAYED_S,
            _ => false,
        };
        match (f.dir, est) {
            (Dir::Departure, Some(e)) if e < now_s => Status::Departed,
            (Dir::Arrival, Some(e)) if e < now_s => Status::Landed,
            (Dir::Departure, Some(e)) if e - now_s <= BOARDING_S => Status::Boarding,
            _ if late => Status::Delayed,
            _ => Status::Scheduled,
        }
    }

    /// Finished with: off the board once [`DEPARTED_KEPT_S`] or
    /// [`LANDED_KEPT_S`] past.
    pub fn done(self) -> bool {
        matches!(self, Status::Departed | Status::Landed | Status::AtGate)
    }
}

/// One board row as it is drawn.
#[derive(Debug, Clone, PartialEq)]
pub struct Row {
    pub flight: BoardFlight,
    pub status: Status,
    /// The time shown beside the timetable's: the estimate, the live ETA
    /// of an arrival in the air, or the actual time once it happened.
    pub time_s: Option<f64>,
    pub stand: Option<String>,
}

/// The rows each half shows.
pub const ROWS_EACH: usize = 10;

/// The board's payload for `xcopilot/status/board`: a header line
/// "H|<field>|<HH:MM local>", with "|<ICAO>,<ICAO>..." on it when there are
/// `choices` (the fields the Board tab may pick, the active one first,
/// 2026-10-13; a plugin before the selector reads the first three fields
/// only), then up to [`ROWS_EACH`] departures and as
/// many arrivals, each "D" or "A"|sched|time|number|airline|city|type|
/// stand|status|own, '\n'-separated, in timetable order; a flight finished
/// more than [`DEPARTED_KEPT_S`] (a departure) or [`LANDED_KEPT_S`] (an
/// arrival) ago is off, and one more than [`AHEAD_S`] ahead not yet on.
/// Times "HH:MM", a blank where unknown; a '|' in a name becomes '/'. Empty with no rows and no other field to
/// pick: the tab says so.
pub fn payload(field: &str, now_s: f64, rows: &[Row], choices: &[String]) -> String {
    let clean = |s: &str| s.replace(['|', '\n', ','], "/");
    let t = |v: Option<f64>| v.map(hhmm).unwrap_or_default();
    let mut head = format!("H|{}|{}", clean(field), hhmm(now_s));
    if !choices.is_empty() {
        let list: Vec<String> = choices.iter().map(|c| clean(c)).collect();
        head.push('|');
        head.push_str(&list.join(","));
    }
    let mut out = vec![head];
    for (dir, tag) in [(Dir::Departure, 'D'), (Dir::Arrival, 'A')] {
        let mut half: Vec<&Row> = rows
            .iter()
            .filter(|r| r.flight.dir == dir)
            .filter(|r| {
                let when = r.time_s.or(r.flight.info.est_s).or(r.flight.info.sched_s);
                let kept = match dir {
                    Dir::Departure => DEPARTED_KEPT_S,
                    Dir::Arrival => LANDED_KEPT_S,
                };
                !(r.status.done() && when.is_some_and(|w| now_s - w > kept))
                    && !when.is_some_and(|w| w - now_s > AHEAD_S)
            })
            .collect();
        half.sort_by(|a, b| {
            let k = |r: &Row| r.flight.info.sched_s.or(r.time_s).unwrap_or(f64::MAX);
            k(a).total_cmp(&k(b))
        });
        // The flights still to come first: a full board keeps the next
        // ones, with the last few gone at its head.
        let gone = half.iter().filter(|r| r.status.done()).count();
        let skip = gone
            .saturating_sub(3)
            .min(half.len().saturating_sub(ROWS_EACH));
        for r in half.into_iter().skip(skip).take(ROWS_EACH) {
            let i = &r.flight.info;
            out.push(format!(
                "{tag}|{}|{}|{}|{}|{}|{}|{}|{}|{}",
                t(i.sched_s),
                t(r.time_s),
                clean(&i.number),
                clean(&i.airline),
                clean(if i.city.is_empty() { &i.other } else { &i.city }),
                clean(&r.flight.icao_type),
                clean(r.stand.as_deref().unwrap_or("")),
                r.status.word(),
                u8::from(r.flight.own)
            ));
        }
    }
    if out.len() == 1 && choices.len() < 2 {
        return String::new();
    }
    out.join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flight(dir: Dir, sched: f64, est: f64) -> BoardFlight {
        BoardFlight {
            dir,
            callsign: "EZY8062".into(),
            icao_type: "A320".into(),
            info: FlightInfo {
                number: "U2 8062".into(),
                airline: "easyJet".into(),
                other: "EGKK".into(),
                city: "London Gatwick".into(),
                sched_s: Some(sched),
                est_s: Some(est),
                inbound: Some("EZY8061".into()),
                stand: None,
            },
            own: false,
        }
    }

    fn seen(phase: Phase, dormant: bool) -> Option<Seen> {
        Some(Seen { phase, dormant })
    }

    /// Each status from where the aircraft is.
    #[test]
    fn statuses_follow_the_phases() {
        let now = 36_000.0;
        let dep = flight(Dir::Departure, now + 1800.0, now + 1800.0);
        assert_eq!(
            Status::of(&dep, seen(Phase::Parked, true), now),
            Status::Boarding
        );
        let later = flight(Dir::Departure, now + 7200.0, now + 7200.0);
        assert_eq!(
            Status::of(&later, seen(Phase::Parked, true), now),
            Status::Scheduled
        );
        assert_eq!(Status::of(&later, None, now), Status::Scheduled);
        assert_eq!(
            Status::of(&dep, seen(Phase::Parked, false), now),
            Status::Taxiing
        );
        assert_eq!(
            Status::of(&dep, seen(Phase::TaxiOut, false), now),
            Status::Taxiing
        );
        assert_eq!(
            Status::of(&dep, seen(Phase::Roll, false), now),
            Status::Taxiing
        );
        assert_eq!(
            Status::of(&dep, seen(Phase::Climb, false), now),
            Status::Departed
        );
        assert_eq!(
            Status::of(&dep, seen(Phase::Leave, false), now),
            Status::Departed
        );
        let gone = flight(Dir::Departure, now - 900.0, now - 900.0);
        assert_eq!(Status::of(&gone, None, now), Status::Departed);
        let late = flight(Dir::Departure, now + 3000.0, now + 6000.0);
        assert_eq!(Status::of(&late, None, now), Status::Delayed);

        let arr = flight(Dir::Arrival, now + 600.0, now + 600.0);
        assert_eq!(Status::of(&arr, None, now), Status::Scheduled);
        assert_eq!(
            Status::of(&arr, seen(Phase::Arrive, false), now),
            Status::Approaching
        );
        assert_eq!(
            Status::of(&arr, seen(Phase::Final, false), now),
            Status::Approaching
        );
        assert_eq!(
            Status::of(&arr, seen(Phase::GoAround, false), now),
            Status::Approaching
        );
        assert_eq!(
            Status::of(&arr, seen(Phase::Landing, false), now),
            Status::Landed
        );
        assert_eq!(
            Status::of(&arr, seen(Phase::TaxiIn, false), now),
            Status::Landed
        );
        assert_eq!(
            Status::of(&arr, seen(Phase::Parked, false), now),
            Status::AtGate
        );
        assert_eq!(
            Status::of(&arr, seen(Phase::Parked, true), now),
            Status::AtGate
        );
        let landed = flight(Dir::Arrival, now - 300.0, now - 300.0);
        assert_eq!(Status::of(&landed, None, now), Status::Landed);
        let held = flight(Dir::Arrival, now + 600.0, now + 600.0 + DELAYED_S);
        assert_eq!(Status::of(&held, None, now), Status::Delayed);
    }

    /// The payload's header, its rows in timetable order with their
    /// columns, the finished flights dropped after a while, and nothing
    /// for an empty board.
    #[test]
    fn the_board_text_has_its_columns() {
        let now = 36_000.0;
        let row = |f: BoardFlight, status, time_s, stand: Option<&str>| Row {
            flight: f,
            status,
            time_s,
            stand: stand.map(str::to_string),
        };
        let rows = vec![
            row(
                flight(Dir::Departure, now + 1800.0, now + 2400.0),
                Status::Boarding,
                Some(now + 2400.0),
                Some("42"),
            ),
            row(
                flight(Dir::Departure, now - 3.0 * 3600.0, now - 3.0 * 3600.0),
                Status::Departed,
                Some(now - 3.0 * 3600.0),
                None,
            ),
            row(
                flight(Dir::Arrival, now + 300.0, now + 360.0),
                Status::Approaching,
                Some(now + 360.0),
                None,
            ),
        ];
        let p = payload("LEMG", now, &rows, &[]);
        let lines: Vec<&str> = p.lines().collect();
        assert_eq!(lines[0], "H|LEMG|10:00");
        assert_eq!(
            lines[1],
            "D|10:30|10:40|U2 8062|easyJet|London Gatwick|A320|42|Boarding|0"
        );
        assert_eq!(
            lines[2],
            "A|10:05|10:06|U2 8062|easyJet|London Gatwick|A320||Approaching|0"
        );
        assert_eq!(lines.len(), 3, "the 07:00 departure is long gone: {p}");
        assert_eq!(payload("LEMG", now, &[], &[]), "");
        assert!(p.len() < 2000);
        // With fields to pick, the header lists them, the active first;
        // the selector's header shows even with no rows.
        let picks = ["LEMG".to_string(), "LXGB".to_string()];
        let p = payload("LEMG", now, &rows, &picks);
        assert_eq!(p.lines().next(), Some("H|LEMG|10:00|LEMG,LXGB"));
        assert_eq!(p.lines().count(), 3);
        assert_eq!(payload("LXGB", now, &[], &picks), "H|LXGB|10:00|LEMG,LXGB");
        assert_eq!(payload("LEMG", now, &[], &picks[..1]), "");
    }

    /// A FIDS's window (LXGB at 17:48, 2026-10-13): a departure gone 20
    /// minutes still reads Departed, one gone 40 is off; an arrival landed
    /// 50 minutes still reads Landed; tomorrow's 11:05, 17 hours ahead, is
    /// not on the board to be read as today's.
    #[test]
    fn the_board_shows_the_next_twelve_hours_and_drops_the_gone() {
        let now = 17.0 * 3600.0 + 48.0 * 60.0;
        let row = |dir, at: f64, status| Row {
            flight: flight(dir, at, at),
            status,
            time_s: Some(at),
            stand: None,
        };
        let rows = vec![
            row(Dir::Departure, now - 20.0 * 60.0, Status::Departed),
            row(Dir::Departure, now - 40.0 * 60.0, Status::Departed),
            row(Dir::Departure, now + 3.0 * 3600.0, Status::Scheduled),
            row(
                Dir::Departure,
                86_400.0 + 11.0 * 3600.0 + 300.0,
                Status::Scheduled,
            ),
            row(Dir::Arrival, now - 50.0 * 60.0, Status::Landed),
            row(Dir::Arrival, now - 70.0 * 60.0, Status::Landed),
            row(
                Dir::Arrival,
                86_400.0 + 10.0 * 3600.0 + 1200.0,
                Status::Scheduled,
            ),
        ];
        let p = payload("LXGB", now, &rows, &[]);
        let lines: Vec<&str> = p.lines().skip(1).collect();
        assert_eq!(lines.len(), 3, "{p}");
        assert!(lines[0].starts_with("D|17:28|17:28|") && lines[0].contains("|Departed|"));
        assert!(lines[1].starts_with("D|20:48|"), "{p}");
        assert!(lines[2].starts_with("A|16:58|16:58|") && lines[2].contains("|Landed|"));
        assert!(!p.contains("11:05") && !p.contains("10:20"), "{p}");
    }

    /// A field nobody simulates goes by the clock: a departure inside 40
    /// minutes Boarding, gone past its estimate; an arrival Landed past
    /// its estimate; a late one Delayed until then.
    #[test]
    fn a_field_by_the_clock() {
        let now = 36_000.0;
        let st = |dir, sched: f64, est: f64| Status::by_clock(&flight(dir, sched, est), now);
        assert_eq!(
            st(Dir::Departure, now + 1800.0, now + 1800.0),
            Status::Boarding
        );
        assert_eq!(
            st(Dir::Departure, now + 3600.0, now + 3600.0),
            Status::Scheduled
        );
        assert_eq!(st(Dir::Departure, now - 60.0, now - 60.0), Status::Departed);
        assert_eq!(
            st(Dir::Departure, now + 600.0, now + 3600.0),
            Status::Delayed
        );
        assert_eq!(st(Dir::Arrival, now - 60.0, now - 60.0), Status::Landed);
        assert_eq!(
            st(Dir::Arrival, now + 600.0, now + 600.0),
            Status::Scheduled
        );
        assert_eq!(st(Dir::Arrival, now + 600.0, now + 2400.0), Status::Delayed);
    }
}
