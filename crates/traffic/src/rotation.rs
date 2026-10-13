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
/// row's windows, nudged by a stable fraction of a slot so two rows do not
/// land together.
fn slot_s(row: &RouteRow, k: u32, n: u32) -> f64 {
    let total: f64 = row.windows.iter().map(|(a, b)| b - a).sum();
    let nudge = (row_hash(row, u64::from(k)) % 1000) as f64 / 1000.0 - 0.5;
    let mut at = total * ((f64::from(k) + 0.5 + 0.6 * nudge) / f64::from(n)).clamp(0.0, 1.0);
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

/// The rotations of `table` on `date`, times local to `date`'s midnight:
/// for each row its day's count; with a based row its FIRST rotation
/// night-stops (in the evening before, 22:30 to 23:45, out in the morning
/// wave, 06:15 to 07:45); the rest arrive in the row's windows and leave a
/// turnaround later. A late inbound pushes its outbound back to the
/// type's minimum turnaround. At [`Density::Light`] half of them
/// (deterministically), at `Off` none.
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
        let through = if row.based { n.saturating_sub(1) } else { n };
        for k in 0..n {
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
            let night_stop = row.based && k == 0;
            let (sta, std) = if night_stop {
                let stable = (row_hash(row, 99 + u64::from(k)) % 1000) as f64 / 1000.0;
                let sta = -3600.0 * (1.5 - 1.25 * stable);
                let std = 3600.0 * (6.25 + 1.5 * stable);
                (sta, std)
            } else {
                let j = if row.based { k - 1 } else { k };
                let sta = slot_s(row, j, through.max(1));
                (sta, sta + sched_turn_s(icao_type, &mut rng))
            };
            let eta = sta + delay_s(&mut rng);
            let etd = (std + delay_s(&mut rng).max(0.0)).max(eta + min_turn_s(icao_type));
            out.push(Rotation {
                carrier: row.carrier,
                icao_type,
                arr: leg(&row.carrier, in_no, row, sta, eta),
                dep: leg(&row.carrier, out_no, row, std, etd),
                night_stop,
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
}
