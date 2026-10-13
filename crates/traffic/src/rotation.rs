//! The day's ROTATIONS at a field with a route table ([`crate::routes`]):
//! an aircraft arrives as flight N from its origin, turns round on a stand
//! for its type's time, and leaves as flight N+1 to its destination; a
//! based aircraft night-stops (in the evening before, out in the morning
//! wave). Deterministic from (seed, field, date): the timetable's times are
//! the route's own every week, the seed draws the types and the day's
//! delays (a late inbound makes a late outbound).
//!
//! Until 2026-10-13 every field drew independent departures and arrivals an
//! hour at a time; an arrival turned round as any departure that came due.
//! Times here are LOCAL seconds after midnight of the reference date, which
//! may run below 0 (the evening before) or past 86,400 (tomorrow).

use crate::airlines::{self, Carrier};
use crate::perf::{class_for_type, JET_HEAVY, TURBOPROP};
use crate::rng::XorShift64;
use crate::routes::{FieldRoutes, RouteRow};
use crate::schedule::Density;

/// A calendar day: the year and the day of the year, 0 for the first of
/// January (X-Plane's `sim/time/local_date_days`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Date {
    pub year: i32,
    pub ordinal: u32,
}

/// Days from 1970-01-01 to `y`-`m`-`d` (Howard Hinnant's civil algorithm).
fn days_from_civil(y: i32, m: u32, d: u32) -> i64 {
    let y = if m <= 2 { y - 1 } else { y } as i64;
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let m = m as i64;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d as i64 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

fn is_leap(y: i32) -> bool {
    (y % 4 == 0 && y % 100 != 0) || y % 400 == 0
}

impl Date {
    /// `y`-`m`-`d`.
    pub fn ymd(y: i32, m: u32, d: u32) -> Date {
        let ordinal = (days_from_civil(y, m, d) - days_from_civil(y, 1, 1)) as u32;
        Date { year: y, ordinal }
    }

    /// The day `days` after 1970-01-01.
    pub fn from_epoch_days(days: i64) -> Date {
        Date::ymd(1970, 1, 1).plus(days)
    }

    /// Day `ordinal` (0 based, X-Plane's `local_date_days`, which carries
    /// no year) of `year`, clamped to the year's length.
    pub fn of_year(year: i32, ordinal: u32) -> Date {
        let len = if is_leap(year) { 366 } else { 365 };
        Date {
            year,
            ordinal: ordinal.min(len - 1),
        }
    }

    /// Days since 1970-01-01.
    pub fn epoch_days(self) -> i64 {
        days_from_civil(self.year, 1, 1) + i64::from(self.ordinal)
    }

    /// 0 Monday .. 6 Sunday.
    pub fn weekday(self) -> u32 {
        // 1970-01-01 was a Thursday (3).
        ((self.epoch_days() + 3).rem_euclid(7)) as u32
    }

    /// The day `n` days on (or back).
    pub fn plus(self, n: i64) -> Date {
        let mut y = self.year;
        let mut o = i64::from(self.ordinal) + n;
        loop {
            let len = if is_leap(y) { 366 } else { 365 };
            if o < 0 {
                y -= 1;
                o += if is_leap(y) { 366 } else { 365 };
            } else if o >= len {
                o -= len;
                y += 1;
            } else {
                return Date {
                    year: y,
                    ordinal: o as u32,
                };
            }
        }
    }

    /// The last Sunday of month `m` in this year.
    fn last_sunday(year: i32, m: u32) -> Date {
        let next = if m == 12 {
            Date::ymd(year + 1, 1, 1)
        } else {
            Date::ymd(year, m + 1, 1)
        };
        let last = next.plus(-1);
        last.plus(-(i64::from((last.weekday() + 1) % 7)))
    }

    /// The IATA summer season (and the EU's summer time, near enough): from
    /// the last Sunday of March to the Saturday before the last Sunday of
    /// October.
    pub fn summer(self) -> bool {
        let start = Date::last_sunday(self.year, 3).epoch_days();
        let end = Date::last_sunday(self.year, 10).epoch_days();
        (start..end).contains(&self.epoch_days())
    }
}

/// One flight of a rotation: its number, the other end, and its times
/// (local seconds after the reference midnight).
#[derive(Debug, Clone, PartialEq)]
pub struct FlightLeg {
    pub callsign: String,
    pub spoken: String,
    pub other: &'static str,
    pub city: &'static str,
    /// The timetable's time: on the blocks for an arrival, off them for a
    /// departure.
    pub sched_s: f64,
    /// When it will happen today, delays and all.
    pub est_s: f64,
}

/// One aircraft's visit: in as `arr`, round, out as `dep`.
#[derive(Debug, Clone, PartialEq)]
pub struct Rotation {
    pub carrier: Carrier,
    pub icao_type: &'static str,
    pub arr: FlightLeg,
    pub dep: FlightLeg,
    /// It came in the evening before and leaves in the morning wave.
    pub night_stop: bool,
    /// A seed of its own (the aircraft's draws).
    pub seed: u64,
}

/// The least a type turns round in, seconds: the airlines' minimum ground
/// times (a low-cost narrowbody 25 minutes, an ATR 20, a widebody 75). A
/// late inbound leaves no earlier than this after it is on the blocks.
pub fn min_turn_s(icao_type: &str) -> f64 {
    let p = class_for_type(icao_type).0;
    60.0 * if p.name == JET_HEAVY.name {
        75.0
    } else if p.name == TURBOPROP.name {
        20.0
    } else {
        25.0
    }
}

/// The planned turnaround of `icao_type`, seconds: a regional turboprop
/// about 30 minutes, a narrowbody about 45, a widebody 90 and more (the
/// timetable's, never the minimum: there is slack in it).
pub fn sched_turn_s(icao_type: &str, rng: &mut XorShift64) -> f64 {
    let p = class_for_type(icao_type).0;
    let (lo, hi) = if p.name == JET_HEAVY.name {
        (90.0, 120.0)
    } else if p.name == TURBOPROP.name {
        (25.0, 35.0)
    } else {
        (40.0, 55.0)
    };
    rng.range_f64(lo, hi) * 60.0
}

/// A stable hash of a route row and a day: the timetable's own draws.
fn row_hash(row: &RouteRow, salt: u64) -> u64 {
    let mut h = 0xCBF2_9CE4_8422_2325u64 ^ salt;
    for b in row
        .carrier
        .code
        .bytes()
        .chain(row.number.to_le_bytes())
        .chain(row.other.bytes())
    {
        h = (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01B3);
    }
    h
}

/// Rotations `row` flies on `date`: its season's weekly frequency, the
/// whole part every day, the rest on the same weekdays every week.
pub fn daily_count(row: &RouteRow, date: Date) -> u32 {
    let f = u32::from(if date.summer() {
        row.summer
    } else {
        row.winter
    });
    let extra = f % 7;
    let off = (row_hash(row, 7) % 7) as u32;
    f / 7 + u32::from((date.weekday() + off) % 7 < extra)
}

/// The local second of the day of rotation `k` of `n`: evenly through the
/// row's windows, each row offset by its own stable share of a slot. With
/// the offset a fixed half slot every once-a-day route landed mid-way
/// through its windows, at 15:00, and 10:00 at Malaga had six rotations in
/// the hour round it (the mid-flow harness, 2026-10-13).
fn slot_s(row: &RouteRow, k: u32, n: u32) -> f64 {
    let total: f64 = row.windows.iter().map(|(a, b)| b - a).sum();
    let offset = (row_hash(row, 0x51_07) % 1000) as f64 / 1000.0;
    let mut at = total * ((f64::from(k) + offset) / f64::from(n)).clamp(0.0, 1.0);
    for (a, b) in row.windows {
        if at <= b - a {
            return (a + at) * 3600.0;
        }
        at -= b - a;
    }
    row.windows.last().map_or(43_200.0, |w| w.1 * 3600.0)
}

/// A day's delay draw, seconds: three in four on time (a few minutes either
/// way), one in five 10 to 30 minutes late, one in twenty 30 to 90.
fn delay_s(rng: &mut XorShift64) -> f64 {
    let x = rng.next_f64();
    60.0 * if x < 0.75 {
        rng.range_f64(-4.0, 6.0)
    } else if x < 0.95 {
        rng.range_f64(10.0, 30.0)
    } else {
        rng.range_f64(30.0, 90.0)
    }
}

fn leg(carrier: &Carrier, number: u32, row: &RouteRow, sched_s: f64, est_s: f64) -> FlightLeg {
    let words = number
        .to_string()
        .chars()
        .map(crate::schedule::digit_word)
        .collect::<Vec<_>>()
        .join(" ");
    FlightLeg {
        callsign: format!("{}{number}", carrier.code),
        spoken: format!("{} {words}", carrier.telephony),
        other: row.other,
        city: row.city,
        sched_s,
        est_s,
    }
}

/// A base's morning wave: its based aircraft leave between these, local
/// hours (Malaga's first departures from 06:00, a base of ten or more
/// aircraft off by 08:30).
const WAVE_FIRST_H: f64 = 6.0;
const WAVE_SPAN_H: f64 = 3.5;

/// A visitor's line reaches us in its first wave or after a sector
/// elsewhere: its first departure from its base is up to this share of a
/// cycle after the wave opens (at 0 every UK route's first arrival fell
/// 09:40 to 11:25, 34 arrivals in the 10:00 hour and 4 at noon).
const VISITOR_PHASE: f64 = 0.75;
const VISITOR_FIRST_H: f64 = 5.5;

/// The latest a line's arrival is timetabled, local hours: the night is
/// quiet (Malaga has no scheduled movement 01:00 to 06:00).
const LINE_LAST_ARR_H: f64 = 23.75;

/// Block-time cruise, knots, and the minutes on top for the taxi, climb
/// and descent: Malaga to Gatwick (892 nm) 2 h 40 against easyJet's 2 h 45,
/// to Madrid (233 nm) 1 h 08 against Iberia's 1 h 10.
const JET_BLOCK_KTS: f64 = 430.0;
const PROP_BLOCK_KTS: f64 = 270.0;
const BLOCK_EXTRA_S: f64 = 36.0 * 60.0;

/// The most aircraft one route's lines take.
const LINE_AIRCRAFT_MAX: u32 = 12;

/// A timetable time is on five minutes, as published.
fn on_five(s: f64) -> f64 {
    (s / 300.0).round() * 300.0
}

/// The type a route is timetabled for: its heaviest weight.
fn main_type(types: &'static [(&'static str, u32)]) -> &'static str {
    types.iter().max_by_key(|t| t.1).map_or("A320", |t| t.0)
}

/// Gate to gate from `from` to `to` in `icao_type`, seconds, where both
/// are places we know ([`crate::places`]).
pub fn block_s(from: &str, to: &str, icao_type: &str) -> Option<f64> {
    let nm = crate::places::dist_nm(crate::places::position(from)?, crate::places::position(to)?);
    let kts = if class_for_type(icao_type).0.name == TURBOPROP.name {
        PROP_BLOCK_KTS
    } else {
        JET_BLOCK_KTS
    };
    Some(nm / kts * 3600.0 + BLOCK_EXTRA_S)
}

/// The middle of the type's planned turnaround ([`sched_turn_s`]): the
/// one the timetable plans its lines on.
fn plan_turn_s(icao_type: &str) -> f64 {
    let p = class_for_type(icao_type).0;
    60.0 * if p.name == JET_HEAVY.name {
        105.0
    } else if p.name == TURBOPROP.name {
        30.0
    } else {
        47.5
    }
}

/// One timetabled visit before the day's draws: on the blocks here, and,
/// for a night-stop, off them in the morning wave.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Visit {
    sta: f64,
    std: Option<f64>,
    night_stop: bool,
}

/// `n` picks spread evenly over `slots` (sorted), each route its own
/// stable offset `off` (0 to 1) into the spacing.
fn spread(slots: &[f64], n: usize, off: f64) -> Vec<f64> {
    let s = slots.len();
    if s == 0 {
        return Vec::new();
    }
    (0..n)
        .map(|k| {
            let i = ((k as f64 + off) * s as f64 / n as f64).floor() as usize;
            slots[i.min(s - 1)]
        })
        .collect()
}

/// The visits of a row flown on its carrier's LINES OF FLYING
/// ([`crate::routes::LINES`]), `n` of them on `date`, local seconds; `None`
/// where either end has no place. From first principles: an aircraft's
/// day starts at its BASE in the morning wave (06:00 to 07:45 there, each
/// route its own stable minute) and goes on one rotation every CYCLE (a
/// block time each way and a turnaround at each end).
///
/// - A visitor (based at the other end) arrives first a block time after
///   its morning departure, on our clock; its later slots are a cycle
///   apart until the last arrival of the evening. More rotations than one
///   aircraft's slots take more aircraft, staggered through the cycle; a
///   route flown less often takes its picks spread evenly over the slots
///   (the aircraft flies elsewhere in between).
/// - A BASED row's aircraft night-stop here: each leaves in the morning
///   wave and is back a cycle less a turnaround later, and again every
///   cycle on; its last return of the day night-stops (tomorrow's first
///   rotation), its first rotation is the one that came in last night.
fn line_visits(table: &FieldRoutes, row: &RouteRow, date: Date, n: u32) -> Option<Vec<Visit>> {
    let types = if row.types.is_empty() {
        row.carrier.types
    } else {
        row.types
    };
    let t = main_type(types);
    let block = block_s(row.other, table.icao, t)?;
    let turn = plan_turn_s(t);
    let cycle = 2.0 * (block + turn);
    let last = LINE_LAST_ARR_H * 3600.0;
    let frac = |salt: u64| (row_hash(row, salt) % 10_000) as f64 / 10_000.0;
    let n = n as usize;
    if n == 0 {
        return Some(Vec::new());
    }
    if !row.based {
        let summer = date.summer();
        let here = table.utc_offset_h + if summer { 1.0 } else { 0.0 };
        let tz = (here - crate::places::utc_offset_h(row.other, summer)) * 3600.0;
        let phase = (WAVE_SPAN_H * 3600.0).max(VISITOR_PHASE * cycle);
        let first = VISITOR_FIRST_H * 3600.0 + phase * frac(0x11) + tz + block;
        let mut slots = Vec::new();
        for a in 1..=LINE_AIRCRAFT_MAX {
            slots.clear();
            for i in 0..a {
                let mut at = first + cycle * f64::from(i) / f64::from(a);
                while at <= last {
                    slots.push(at);
                    at += cycle;
                }
            }
            if slots.len() >= n {
                break;
            }
        }
        if slots.is_empty() {
            slots.push(first.min(last));
        }
        slots.sort_by(f64::total_cmp);
        let visits = spread(&slots, n, frac(0x22))
            .into_iter()
            .map(|sta| Visit {
                sta,
                std: None,
                night_stop: false,
            })
            .collect();
        return Some(visits);
    }
    // Based: each aircraft's departure in the wave and its returns.
    let u = frac(0x33);
    let mut lines: Vec<(f64, Vec<f64>)> = Vec::new();
    for a in 1..=LINE_AIRCRAFT_MAX {
        lines = (0..a)
            .map(|i| {
                let d = (WAVE_FIRST_H + WAVE_SPAN_H * (f64::from(i) + u) / f64::from(a)) * 3600.0;
                let mut returns = Vec::new();
                let mut r = d + cycle - turn;
                while r <= last {
                    returns.push(r);
                    r += cycle;
                }
                (d, returns)
            })
            .collect();
        let capacity: usize = lines.iter().map(|(_, r)| r.len().max(1)).sum();
        if capacity >= n {
            break;
        }
    }
    // The rotations dealt round the aircraft, as many each as it flies.
    let mut q = vec![0usize; lines.len()];
    let mut dealt = 0;
    loop {
        let before = dealt;
        for (i, (_, r)) in lines.iter().enumerate() {
            if dealt < n && q[i] < r.len().max(1) {
                q[i] += 1;
                dealt += 1;
            }
        }
        if dealt == before || dealt >= n {
            break;
        }
    }
    let off = frac(0x44);
    let mut out = Vec::new();
    for ((d, returns), q) in lines.iter().zip(q) {
        if q == 0 {
            continue;
        }
        // In last night as the day's last return, out in the wave.
        let night = returns.last().copied().unwrap_or(last - 3600.0) - 86_400.0;
        out.push(Visit {
            sta: night,
            std: Some(*d),
            night_stop: true,
        });
        let day = &returns[..returns.len().saturating_sub(1)];
        out.extend(spread(day, q - 1, off).into_iter().map(|sta| Visit {
            sta,
            std: None,
            night_stop: false,
        }));
    }
    Some(out)
}

/// The visits of a row with WINDOWS: for a based row its night-stop in the
/// evening before (22:30 to 23:45) and out in the morning wave (06:15 to
/// 07:45); the rest spread evenly through the windows ([`slot_s`]).
fn window_visits(row: &RouteRow, n: u32) -> Vec<Visit> {
    let through = if row.based { n.saturating_sub(1) } else { n };
    (0..n)
        .map(|k| {
            if row.based && k == 0 {
                let stable = (row_hash(row, 99) % 1000) as f64 / 1000.0;
                Visit {
                    sta: -3600.0 * (1.5 - 1.25 * stable),
                    std: Some(3600.0 * (6.25 + 1.5 * stable)),
                    night_stop: true,
                }
            } else {
                let j = if row.based { k - 1 } else { k };
                Visit {
                    sta: slot_s(row, j, through.max(1)),
                    std: None,
                    night_stop: false,
                }
            }
        })
        .collect()
}

/// The rotations of `table` on `date`, times local to `date`'s midnight,
/// on five minutes: for each row its day's count, on its lines of flying
/// ([`line_visits`]) or in its windows ([`window_visits`]); a night-stop
/// leaves in the morning wave, the rest a planned turnaround after they
/// arrive. A late inbound pushes its outbound back to the type's minimum
/// turnaround. At [`Density::Light`] half of them (deterministically), at
/// `Off` none.
pub fn day_rotations(
    table: &FieldRoutes,
    date: Date,
    seed: u64,
    density: Density,
) -> Vec<Rotation> {
    let mut out = Vec::new();
    if density == Density::Off {
        return out;
    }
    let day_salt = (date.epoch_days() as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    for row in table.rows {
        let n = daily_count(row, date);
        let types = if row.types.is_empty() {
            row.carrier.types
        } else {
            row.types
        };
        let lines = if row.windows.is_empty() {
            line_visits(table, row, date, n)
        } else {
            None
        };
        let visits = lines.unwrap_or_else(|| window_visits(row, n));
        for (k, v) in (0u32..).zip(visits) {
            let seed_k = row_hash(
                row,
                day_salt ^ seed.wrapping_mul(0x2545_F491_4F6C_DD1D) ^ u64::from(k),
            );
            if density == Density::Light && seed_k % 2 == 1 {
                continue;
            }
            let mut rng = XorShift64::new(seed_k);
            let icao_type = pick_type(types, &mut rng);
            let in_no = row.number + 2 * k;
            let out_no = in_no + 1;
            let sta = on_five(v.sta);
            let std = on_five(
                v.std
                    .unwrap_or_else(|| sta + sched_turn_s(icao_type, &mut rng)),
            );
            let eta = sta + delay_s(&mut rng);
            let etd = (std + delay_s(&mut rng).max(0.0)).max(eta + min_turn_s(icao_type));
            out.push(Rotation {
                carrier: row.carrier,
                icao_type,
                arr: leg(&row.carrier, in_no, row, sta, eta),
                dep: leg(&row.carrier, out_no, row, std, etd),
                night_stop: v.night_stop,
                seed: rng.next_u64(),
            });
        }
    }
    out.sort_by(|a, b| a.arr.est_s.total_cmp(&b.arr.est_s));
    out
}

fn pick_type(types: &'static [(&'static str, u32)], rng: &mut XorShift64) -> &'static str {
    let total: u32 = types.iter().map(|t| t.1).sum();
    let mut x = rng.below(u64::from(total.max(1))) as u32;
    for (t, w) in types {
        if x < *w {
            return t;
        }
        x -= w;
    }
    types.first().map_or("A320", |t| t.0)
}

/// The rotations that touch the window `from_s..to_s` (local seconds after
/// `date`'s midnight): yesterday's (its night-stops leave this morning),
/// today's and tomorrow's, each shifted onto `date`'s clock. A rotation is
/// kept when its departure is still to come after `from_s` or its arrival
/// falls before `to_s`.
pub fn window(
    table: &FieldRoutes,
    date: Date,
    seed: u64,
    density: Density,
    from_s: f64,
    to_s: f64,
) -> Vec<Rotation> {
    let mut out = Vec::new();
    for d in -1..=1i64 {
        let shift = 86_400.0 * d as f64;
        for mut r in day_rotations(table, date.plus(d), seed, density) {
            for l in [&mut r.arr, &mut r.dep] {
                l.sched_s += shift;
                l.est_s += shift;
            }
            if r.dep.est_s >= from_s && r.arr.est_s < to_s {
                out.push(r);
            }
        }
    }
    out.sort_by(|a, b| a.arr.est_s.total_cmp(&b.arr.est_s));
    out
}

/// The local time of day `zulu_s` seconds after midnight zulu is at a field
/// whose standard time is `utc_offset_h`, on `date` (summer time on top in
/// the summer season), seconds after the local midnight.
pub fn local_s(zulu_s: f64, utc_offset_h: f64, date: Date) -> f64 {
    let dst = if date.summer() { 1.0 } else { 0.0 };
    (zulu_s + (utc_offset_h + dst) * 3600.0).rem_euclid(86_400.0)
}

/// "HH:MM" of a local second (any day: the clock wraps).
pub fn hhmm(local_s: f64) -> String {
    let m = (local_s / 60.0).round().rem_euclid(1440.0) as u32;
    format!("{:02}:{:02}", m / 60, m % 60)
}

/// The carrier a rotation callsign belongs to at a table field.
pub fn carrier_of(table: &FieldRoutes, callsign: &str) -> Option<Carrier> {
    let code = callsign.get(..3)?;
    table
        .rows
        .iter()
        .map(|r| r.carrier)
        .find(|c| c.code == code)
        .or_else(|| airlines::carrier_of(table.icao, callsign))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::routes::field_routes;

    #[test]
    fn the_calendar_knows_its_weekdays_and_seasons() {
        assert_eq!(Date::ymd(2026, 10, 13).weekday(), 1, "a Tuesday");
        assert_eq!(Date::ymd(2024, 2, 29).plus(1), Date::ymd(2024, 3, 1));
        assert_eq!(Date::ymd(2026, 1, 1).plus(-1), Date::ymd(2025, 12, 31));
        assert_eq!(Date::last_sunday(2026, 3), Date::ymd(2026, 3, 29));
        assert_eq!(Date::last_sunday(2026, 10), Date::ymd(2026, 10, 25));
        assert!(Date::ymd(2026, 7, 1).summer());
        assert!(Date::ymd(2026, 3, 29).summer());
        assert!(!Date::ymd(2026, 10, 25).summer());
        assert!(!Date::ymd(2026, 1, 15).summer());
        // Malaga at 08:00 zulu in July is 10:00 local; in January 09:00.
        assert_eq!(
            hhmm(local_s(8.0 * 3600.0, 1.0, Date::ymd(2026, 7, 1))),
            "10:00"
        );
        assert_eq!(
            hhmm(local_s(8.0 * 3600.0, 1.0, Date::ymd(2026, 1, 1))),
            "09:00"
        );
    }

    fn day(icao: &str, date: Date, seed: u64) -> Vec<Rotation> {
        day_rotations(field_routes(icao).unwrap(), date, seed, Density::Normal)
    }

    /// Every rotation pairs its flights: one carrier, N in and N+1 out, to
    /// and from one city, out after in by at least its type's minimum.
    #[test]
    fn rotations_pair_up() {
        for r in day("LEMG", Date::ymd(2026, 7, 14), 3) {
            let n_in: u32 = r.arr.callsign[3..].parse().unwrap();
            let n_out: u32 = r.dep.callsign[3..].parse().unwrap();
            assert_eq!(n_out, n_in + 1, "{}", r.arr.callsign);
            assert_eq!(&r.arr.callsign[..3], &r.dep.callsign[..3]);
            assert_eq!(r.arr.other, r.dep.other);
            assert!(r.dep.est_s - r.arr.est_s >= min_turn_s(r.icao_type) - 1e-6);
            assert!(r.dep.sched_s > r.arr.sched_s);
            assert!(r.arr.spoken.starts_with(r.carrier.telephony));
        }
    }

    /// The planned turnaround is the type's: an ATR about half an hour, a
    /// narrowbody about 45 minutes, a widebody an hour and a half or more.
    #[test]
    fn turnarounds_are_the_types() {
        let rs = day("LEMG", Date::ymd(2026, 7, 14), 3);
        let turn = |pred: &dyn Fn(&Rotation) -> bool| -> Vec<f64> {
            rs.iter()
                .filter(|r| !r.night_stop && pred(r))
                .map(|r| (r.dep.sched_s - r.arr.sched_s) / 60.0)
                .collect()
        };
        let atr = turn(&|r| r.icao_type == "AT76");
        let narrow = turn(&|r| ["A320", "B738", "A20N", "A321"].contains(&r.icao_type));
        let wide = turn(&|r| ["B763", "B788", "A359"].contains(&r.icao_type));
        assert!(
            !atr.is_empty() && atr.iter().all(|m| (25.0..=35.0).contains(m)),
            "{atr:?}"
        );
        assert!(narrow.iter().all(|m| (40.0..=55.0).contains(m)));
        assert!(
            !wide.is_empty() && wide.iter().all(|m| *m >= 90.0),
            "{wide:?}"
        );
    }

    fn movements_by_hour(rs: &[Rotation]) -> [u32; 24] {
        let mut h = [0u32; 24];
        for r in rs {
            for t in [r.arr.sched_s, r.dep.sched_s] {
                if (0.0..86_400.0).contains(&t) {
                    h[(t / 3600.0) as usize] += 1;
                }
            }
        }
        h
    }

    /// A summer day at Malaga is about 450 scheduled movements (Aena's 2025
    /// count was 512 a day over the year, a summer day near 600 with the
    /// charters, business aviation and cargo no table holds), quiet before
    /// six and after midnight, with a morning departure wave of the
    /// aircraft that night-stopped and busy banks through the day; a
    /// winter day well under the summer.
    #[test]
    fn a_day_at_malaga_has_its_count_and_its_waves() {
        let summer = day("LEMG", Date::ymd(2026, 7, 14), 3);
        let moves = 2 * summer.len();
        assert!((440..=650).contains(&moves), "{moves}");
        let h = movements_by_hour(&summer);
        assert!(h[3] == 0 && h[4] == 0, "{h:?}");
        let morning_deps = summer
            .iter()
            .filter(|r| r.night_stop && (6.0 * 3600.0..8.0 * 3600.0).contains(&r.dep.sched_s))
            .count();
        assert!(morning_deps >= 10, "{morning_deps}");
        let peak = *h.iter().max().unwrap();
        assert!((35..=60).contains(&peak), "{h:?}");
        assert!(h[10] > 2 * h[6].max(1) / 2 && h[19] > 20, "{h:?}");
        let winter = day("LEMG", Date::ymd(2026, 1, 20), 3);
        assert!(
            winter.len() * 10 < summer.len() * 9,
            "{} {}",
            winter.len(),
            summer.len()
        );
    }

    /// Gibraltar is its handful: four to six rotations a summer day, BA and
    /// easyJet only, by day; about 3,000 scheduled movements a year (its
    /// 3,628 in 2024 count the military and general aviation too).
    #[test]
    fn a_day_at_gibraltar_is_a_handful() {
        for d in 0..7 {
            let rs = day("LXGB", Date::ymd(2026, 7, 13).plus(d), 3);
            assert!((3..=7).contains(&rs.len()), "{}", rs.len());
            for r in &rs {
                assert!(r.arr.sched_s > 9.0 * 3600.0 && r.dep.sched_s < 21.0 * 3600.0);
                assert_eq!(r.icao_type.get(..1), Some("A"), "an A320 family");
            }
        }
        let year: usize = (0..365)
            .map(|d| day("LXGB", Date::ymd(2026, 1, 1).plus(d), 3).len() * 2)
            .sum();
        assert!((2600..=3600).contains(&year), "{year}");
    }

    /// The same seed, field and date is the same day; another date or seed
    /// changes the draws but never the timetable.
    #[test]
    fn a_day_is_deterministic() {
        let d = Date::ymd(2026, 8, 3);
        assert_eq!(day("LEMG", d, 5), day("LEMG", d, 5));
        let other = day("LEMG", d, 6);
        assert_ne!(day("LEMG", d, 5), other);
        let sched = |rs: &[Rotation]| -> Vec<(String, i64)> {
            let mut v: Vec<_> = rs
                .iter()
                .map(|r| (r.arr.callsign.clone(), r.arr.sched_s as i64))
                .collect();
            v.sort();
            v
        };
        assert_eq!(sched(&day("LEMG", d, 5)), sched(&other));
        let light = day_rotations(field_routes("LEMG").unwrap(), d, 5, Density::Light);
        assert!(light.len() * 3 < day("LEMG", d, 5).len() * 2);
    }

    /// A night-stop's arrival is the evening before; the window takes it
    /// from yesterday's day.
    #[test]
    fn the_window_carries_yesterdays_night_stops() {
        let t = field_routes("LEMG").unwrap();
        let w = window(
            t,
            Date::ymd(2026, 7, 14),
            3,
            Density::Normal,
            5.0 * 3600.0,
            9.0 * 3600.0,
        );
        let ns: Vec<&Rotation> = w.iter().filter(|r| r.night_stop).collect();
        assert!(!ns.is_empty());
        assert!(ns
            .iter()
            .all(|r| r.arr.est_s < 0.0 && r.dep.sched_s > 6.0 * 3600.0));
    }

    /// Scheduled arrivals and departures by local hour, and by five
    /// minutes past the hour.
    fn by_hour(rs: &[Rotation]) -> ([u32; 24], [u32; 24], [u32; 12]) {
        let (mut arr, mut dep, mut five) = ([0u32; 24], [0u32; 24], [0u32; 12]);
        for r in rs {
            for (t, h) in [(r.arr.sched_s, &mut arr), (r.dep.sched_s, &mut dep)] {
                if (0.0..86_400.0).contains(&t) {
                    h[(t / 3600.0) as usize] += 1;
                    five[((t / 60.0) as usize % 60) / 5] += 1;
                }
            }
        }
        (arr, dep, five)
    }

    /// The longest wait between two scheduled times inside `from_h..to_h`,
    /// minutes, and the local hour it starts at.
    fn longest_gap(times: impl Iterator<Item = f64>, from_h: f64, to_h: f64) -> (f64, f64) {
        let (a, b) = (from_h * 3600.0, to_h * 3600.0);
        let mut v: Vec<f64> = times.filter(|t| (a..b).contains(t)).collect();
        v.extend([a, b]);
        v.sort_by(f64::total_cmp);
        v.windows(2)
            .map(|w| ((w[1] - w[0]) / 60.0, w[0] / 3600.0))
            .fold((0.0, 0.0), |m, g| if g.0 > m.0 { g } else { m })
    }

    /// A summer week at Malaga is CONTINUOUS from the morning wave to the
    /// evening, with banks, and never bunched on the hour: the pilot's
    /// board at 08:00 local (2026-10-13) had no arrival from 08:06 to 09:03
    /// and no departure from 08:47 to 09:45, and the 09:00 arrivals all
    /// within eight minutes, where the windows began. Every hour from 08:00
    /// to 22:00 has its arrivals and its departures, no wait between two
    /// arrivals from 08:30 nor two departures from 06:30 is over 40
    /// minutes up to 22:00, no five minutes past the hour holds more than
    /// half again its share, and on the hour no more than any other. Run
    /// with --nocapture for the counts.
    #[test]
    fn a_week_at_malaga_is_continuous_and_never_on_the_hour() {
        for d in 0..7 {
            let date = Date::ymd(2026, 7, 13).plus(d);
            let rs = day("LEMG", date, 3);
            let (arr, dep, five) = by_hour(&rs);
            let ga = longest_gap(rs.iter().map(|r| r.arr.sched_s), 8.5, 22.0);
            let gd = longest_gap(rs.iter().map(|r| r.dep.sched_s), 6.5, 22.0);
            println!(
                "LEMG {date:?}\n  arr {arr:?}\n  dep {dep:?}\n  five {five:?} gaps arr {ga:?} dep {gd:?}"
            );
            for h in 8..22 {
                assert!(arr[h] >= 3 && dep[h] >= 3, "{h}:00 arr {arr:?} dep {dep:?}");
            }
            assert!(ga.0 <= 40.0 && gd.0 <= 40.0, "{ga:?} {gd:?}");
            let mean = f64::from(five.iter().sum::<u32>()) / 12.0;
            assert!(five.iter().all(|c| f64::from(*c) <= 1.5 * mean), "{five:?}");
            assert!(five[0] <= *five[1..].iter().max().unwrap(), "{five:?}");
        }
        let winter = day("LEMG", Date::ymd(2026, 1, 20), 3);
        let (arr, dep, _) = by_hour(&winter);
        assert!((9..21).all(|h| arr[h] > 0 && dep[h] > 0), "{arr:?} {dep:?}");
        // Gibraltar keeps its published slots.
        let (arr, dep, _) = by_hour(&day("LXGB", Date::ymd(2026, 7, 14), 3));
        println!("LXGB\n  arr {arr:?}\n  dep {dep:?}");
    }

    /// A line's times are the route's own: a block time from its place
    /// (Malaga to Gatwick 2 h 40, to Madrid about 1 h 10), the based
    /// aircraft out in the morning wave having come in last night, every
    /// time on five minutes.
    #[test]
    fn a_line_flies_its_block_times() {
        let gatwick = block_s("LEMG", "EGKK", "A320").unwrap() / 60.0;
        assert!((150.0..175.0).contains(&gatwick), "{gatwick}");
        let madrid = block_s("LEMG", "LEMD", "A320").unwrap() / 60.0;
        assert!((60.0..80.0).contains(&madrid), "{madrid}");
        assert!(block_s("LEMG", "XXXX", "A320").is_none());
        let rs = day("LEMG", Date::ymd(2026, 7, 14), 3);
        let wave = WAVE_FIRST_H * 3600.0..=(WAVE_FIRST_H + WAVE_SPAN_H) * 3600.0 + 300.0;
        let mut night_stops = 0;
        for r in rs.iter().filter(|r| r.night_stop) {
            night_stops += 1;
            assert!(
                wave.contains(&r.dep.sched_s),
                "{} out at {}",
                r.dep.callsign,
                hhmm(r.dep.sched_s)
            );
            assert!(r.arr.sched_s < 0.0, "{} came in last night", r.arr.callsign);
        }
        assert!(night_stops >= 20, "{night_stops}");
        assert!(rs.iter().all(|r| {
            (r.arr.sched_s / 300.0).fract().abs() < 1e-9
                && (r.dep.sched_s / 300.0).fract().abs() < 1e-9
        }));
    }
}
