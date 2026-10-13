//! THE WHOLE FLIGHT (2026-10-13): where a timetabled flight is while it is
//! NOT in the full simulation, by time. The pilot at LEMG saw an aircraft
//! "pop up out of nowhere" on the EFB map: an arrival appeared 25 to 40 nm
//! out ([`crate::arrival::ENTRY_NM`]) and a departure vanished at 12 nm
//! ([`crate::aircraft::DEPART_GONE_NM`]). Now every flight to or from a
//! simulated field exists from its off-block at the origin to its on-block
//! at the destination: the full simulation flies it near the field, this
//! cheap pure model flies the rest.
//!
//! The model is kinematics by time, no physics, no randomness: taxi at the
//! origin, a climb at the type's rate ([`Perf::climb_fpm`]), a cruise at a
//! level by distance ([`cruise_level_ft`]) and the type's cruise speed, a
//! descent on a 3 degree path, along the great circle. One end of the
//! modelled part is the SIMULATION's: an arrival's track ends exactly where
//! and when the simulation will spawn it ([`Track::ending_at`]), a
//! departure's starts exactly where and when the simulation let it go
//! ([`Track::starting_at`]). So the handover is continuous by construction,
//! in position, altitude and speed.
//!
//! Speeds are ground speeds (no wind, as [`crate::perf`]); each of the three
//! segments varies its speed linearly with distance, from the anchor's own
//! speed to the cruise speed and back, so time and distance invert in
//! closed form.

use crate::kin::{LatLon, KT_TO_MS};
use crate::perf::{Perf, JET_HEAVY, TURBOPROP};

const NM: f64 = 1852.0;

/// The earth's radius the great circle is drawn on, metres (the one
/// [`crate::places::dist_nm`] uses).
const EARTH_R_M: f64 = 3440.065 * NM;

/// Off the blocks to wheels up, and wheels on to the blocks, seconds: a
/// typical taxi either way (the timetable's block time carries 36 minutes
/// on top of the cruise for both and the climb and descent,
/// [`crate::rotation::block_s`]).
pub const TAXI_OUT_S: f64 = 10.0 * 60.0;
pub const TAXI_IN_S: f64 = 5.0 * 60.0;

/// The descent's path: 3 degrees, feet a nautical mile.
pub const DESCENT_FT_PER_NM: f64 = 318.0;

/// The climb's average rate as a share of the type's initial rate: an
/// airliner climbs at 2,500 fpm low and a few hundred near its level.
const CLIMB_SHARE: f64 = 0.6;

/// The lowest speed a modelled end flies at, knots (an anchor read from a
/// stopped aircraft must not freeze the clock).
const MIN_KTS: f64 = 60.0;

/// The highest level the class flies, feet.
fn service_level_ft(p: &Perf) -> f64 {
    if p.name == JET_HEAVY.name {
        39_000.0
    } else if p.cruise_kts >= 400.0 {
        37_000.0
    } else if p.name == TURBOPROP.name {
        25_000.0
    } else {
        9_000.0
    }
}

/// The cruise level of a flight `from` to `to`, feet: higher the further
/// it goes (about 100 ft a mile over 5,000: Gibraltar to Malaga's 54 nm
/// at FL100, Madrid's 233 nm at FL270, Gatwick's 892 nm at the class's
/// ceiling), never above the class's; on the semicircular rule, an odd
/// thousand eastbound and an even one westbound.
pub fn cruise_level_ft(from: LatLon, to: LatLon, p: &Perf) -> f64 {
    let nm = gc_dist_m(from, to) / NM;
    let want = (nm * 100.0 + 5_000.0).min(service_level_ft(p)).max(3_000.0);
    let mut k = (want / 1000.0).floor() as i64;
    let east = gc_bearing(from, to) < 180.0;
    if (k % 2 == 1) != east {
        k -= 1;
    }
    k.max(2) as f64 * 1000.0
}

/// Great circle distance, metres.
pub fn gc_dist_m(a: LatLon, b: LatLon) -> f64 {
    let (la, lb) = (a.0.to_radians(), b.0.to_radians());
    let dlat = lb - la;
    let dlon = (b.1 - a.1).to_radians();
    let h = (dlat / 2.0).sin().powi(2) + la.cos() * lb.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * h.sqrt().min(1.0).asin() * EARTH_R_M
}

/// The initial great circle bearing from `a` to `b`, degrees true 0..360.
pub fn gc_bearing(a: LatLon, b: LatLon) -> f64 {
    let (la, lb) = (a.0.to_radians(), b.0.to_radians());
    let dlon = (b.1 - a.1).to_radians();
    let y = dlon.sin() * lb.cos();
    let x = la.cos() * lb.sin() - la.sin() * lb.cos() * dlon.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

/// The point `f` (0 to 1) of the way along the great circle from `a` to
/// `b`; exactly `a` at 0 and `b` at 1.
pub fn gc_point(a: LatLon, b: LatLon, f: f64) -> LatLon {
    if f <= 0.0 {
        return a;
    }
    if f >= 1.0 {
        return b;
    }
    let d = gc_dist_m(a, b) / EARTH_R_M;
    if d < 1e-9 {
        return a;
    }
    let (la, oa) = (a.0.to_radians(), a.1.to_radians());
    let (lb, ob) = (b.0.to_radians(), b.1.to_radians());
    let (ka, kb) = (((1.0 - f) * d).sin() / d.sin(), (f * d).sin() / d.sin());
    let x = ka * la.cos() * oa.cos() + kb * lb.cos() * ob.cos();
    let y = ka * la.cos() * oa.sin() + kb * lb.cos() * ob.sin();
    let z = ka * la.sin() + kb * lb.sin();
    (z.atan2(x.hypot(y)).to_degrees(), y.atan2(x).to_degrees())
}

/// How far along from `origin` to `dest` a flight at `pos` is, 0 to 1: the
/// distance flown over the distance flown and to go.
pub fn progress(origin: LatLon, dest: LatLon, pos: LatLon) -> f64 {
    let (done, left) = (gc_dist_m(origin, pos), gc_dist_m(pos, dest));
    if done + left <= 0.0 {
        return 1.0;
    }
    (done / (done + left)).clamp(0.0, 1.0)
}

/// One end of the modelled part: a field (on the ground there, off the
/// blocks or onto them) or the simulation's handover point in the air.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct End {
    pub pos: LatLon,
    pub alt_ft: f64,
    pub gs_kts: f64,
    /// A field: the flight taxis there before (after) the airborne part.
    pub on_field: bool,
}

impl End {
    /// A field at `pos`, `elev_ft`: left at the class's climb speed,
    /// reached at its approach speed.
    pub fn field(pos: LatLon, elev_ft: f64, p: &Perf, leaving: bool) -> End {
        End {
            pos,
            alt_ft: elev_ft,
            gs_kts: if leaving { p.climb_kts } else { p.app_kts },
            on_field: true,
        }
    }

    /// The simulation's own state at the handover.
    pub fn air(k: &crate::kin::Kin) -> End {
        End {
            pos: k.pos(),
            alt_ft: k.alt_ft,
            gs_kts: k.gs_kts,
            on_field: false,
        }
    }
}

/// What the model says at one time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stage {
    /// On the ground at a field, taxiing out or in.
    Taxi,
    /// In the air on the track.
    Flying,
    /// Past its handover point and not yet taken by the simulation (a
    /// spawn waiting for the runway in use, or for us to be clear): held
    /// there, never jumping.
    Holding,
}

/// Where the model puts a flight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fix {
    pub pos: LatLon,
    pub alt_ft: f64,
    pub gs_kts: f64,
    pub hdg_deg: f64,
    pub vs_fpm: f64,
    pub on_ground: bool,
    pub stage: Stage,
}

/// A stretch flown at a speed varying linearly with distance, `v0` to
/// `v1` metres a second over `len` metres.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Seg {
    len: f64,
    v0: f64,
    v1: f64,
}

impl Seg {
    fn k(&self) -> f64 {
        (self.v1 - self.v0) / self.len
    }

    fn time(&self) -> f64 {
        if self.len <= 0.0 {
            return 0.0;
        }
        if (self.v1 - self.v0).abs() < 1e-6 {
            return self.len / self.v0;
        }
        (self.v1 / self.v0).ln() / self.k()
    }

    /// The distance flown `t` seconds in (dv/dt = k v: v = v0 e^(kt)).
    fn dist_at(&self, t: f64) -> f64 {
        if self.len <= 0.0 {
            return 0.0;
        }
        let s = if (self.v1 - self.v0).abs() < 1e-6 {
            self.v0 * t
        } else {
            let k = self.k();
            self.v0 * ((k * t).exp() - 1.0) / k
        };
        s.clamp(0.0, self.len)
    }

    fn speed_at(&self, s: f64) -> f64 {
        if self.len <= 0.0 {
            return self.v0;
        }
        self.v0 + (self.v1 - self.v0) * (s / self.len).clamp(0.0, 1.0)
    }
}

/// The modelled part of a flight: from `a` at `t_a` to `b` at `t_b` along
/// the great circle (times in the caller's clock, seconds).
#[derive(Debug, Clone, PartialEq)]
pub struct Track {
    pub a: End,
    pub b: End,
    pub t_a: f64,
    pub t_b: f64,
    /// The level it cruises at (the top of a short hop's climb).
    pub peak_ft: f64,
    len_m: f64,
    toc_m: f64,
    tod_m: f64,
    segs: [Seg; 3],
    taxi_kts: f64,
    /// The course arriving at `b`, degrees true.
    final_deg: f64,
}

impl Track {
    /// The track from `a` at `t_a` to `b`, cruising at `cruise_ft` where
    /// the distance allows, at the class's speeds.
    pub fn starting_at(a: End, b: End, t_a: f64, cruise_ft: f64, p: &Perf) -> Track {
        let len = gc_dist_m(a.pos, b.pos);
        let ms = |kts: f64| kts.max(MIN_KTS) * KT_TO_MS;
        let (va, vc, vb) = (ms(a.gs_kts), ms(p.cruise_kts), ms(b.gs_kts));
        // Feet a metre: the climb at its average rate over its average
        // speed, the descent on its path.
        let gc = CLIMB_SHARE * p.climb_fpm / (0.5 * (va + vc) * 60.0);
        let gd = DESCENT_FT_PER_NM / NM;
        let mut peak = cruise_ft.max(a.alt_ft).max(b.alt_ft);
        let mut toc = (peak - a.alt_ft) / gc;
        let mut tod = len - (peak - b.alt_ft) / gd;
        if toc > tod {
            // A short hop: the climb meets the descent below the level.
            let x = (b.alt_ft - a.alt_ft + gd * len) / (gc + gd);
            if x >= len {
                (toc, tod, peak) = (len, len, b.alt_ft);
            } else if x <= 0.0 {
                (toc, tod, peak) = (0.0, 0.0, a.alt_ft);
            } else {
                (toc, tod, peak) = (x, x, a.alt_ft + gc * x);
            }
        }
        let segs = [
            Seg {
                len: toc,
                v0: va,
                v1: vc,
            },
            Seg {
                len: tod - toc,
                v0: vc,
                v1: vc,
            },
            Seg {
                len: len - tod,
                v0: vc,
                v1: vb,
            },
        ];
        let dur: f64 = segs.iter().map(Seg::time).sum();
        Track {
            a,
            b,
            t_a,
            t_b: t_a + dur,
            peak_ft: peak,
            len_m: len,
            toc_m: toc,
            tod_m: tod,
            segs,
            taxi_kts: p.taxi_kts,
            final_deg: (gc_bearing(b.pos, a.pos) + 180.0).rem_euclid(360.0),
        }
    }

    /// The same track timed to reach `b` at `t_b`.
    pub fn ending_at(a: End, b: End, t_b: f64, cruise_ft: f64, p: &Perf) -> Track {
        let mut t = Track::starting_at(a, b, 0.0, cruise_ft, p);
        t.t_a = t_b - t.t_b;
        t.t_b = t_b;
        t
    }

    /// Seconds from `a` to `b`.
    pub fn duration_s(&self) -> f64 {
        self.t_b - self.t_a
    }

    /// The great circle's length, metres.
    pub fn len_m(&self) -> f64 {
        self.len_m
    }

    /// The altitude `s` metres along, feet: linear between the anchors and
    /// the top of climb and of descent, so both ends are exact.
    fn alt_at(&self, s: f64) -> f64 {
        if s < self.toc_m && self.toc_m > 0.0 {
            self.a.alt_ft + (self.peak_ft - self.a.alt_ft) * s / self.toc_m
        } else if s > self.tod_m && self.len_m > self.tod_m {
            self.peak_ft
                + (self.b.alt_ft - self.peak_ft) * (s - self.tod_m) / (self.len_m - self.tod_m)
        } else {
            self.peak_ft
        }
    }

    /// Distance along and speed (m/s), `t` seconds after `a`.
    fn along(&self, t: f64) -> (f64, f64) {
        let mut left = t.max(0.0);
        let mut base = 0.0;
        for (i, seg) in self.segs.iter().enumerate() {
            let dt = seg.time();
            if left <= dt || i == self.segs.len() - 1 {
                let s = seg.dist_at(left);
                return (base + s, seg.speed_at(s));
            }
            left -= dt;
            base += seg.len;
        }
        (self.len_m, self.segs[2].v1)
    }

    /// Where the flight is at `t`, or `None` before its off-block (or, from
    /// the simulation's handover, before the handover) and after its
    /// on-block.
    pub fn at(&self, t: f64) -> Option<Fix> {
        let ground = |e: &End, hdg: f64, kts: f64| Fix {
            pos: e.pos,
            alt_ft: e.alt_ft,
            gs_kts: kts,
            hdg_deg: hdg,
            vs_fpm: 0.0,
            on_ground: true,
            stage: Stage::Taxi,
        };
        if t < self.t_a {
            return (self.a.on_field && t >= self.t_a - TAXI_OUT_S)
                .then(|| ground(&self.a, gc_bearing(self.a.pos, self.b.pos), self.taxi_kts));
        }
        if t > self.t_b {
            if !self.b.on_field {
                return Some(Fix {
                    pos: self.b.pos,
                    alt_ft: self.b.alt_ft,
                    gs_kts: self.b.gs_kts,
                    hdg_deg: self.final_deg,
                    vs_fpm: 0.0,
                    on_ground: false,
                    stage: Stage::Holding,
                });
            }
            return (t <= self.t_b + TAXI_IN_S)
                .then(|| ground(&self.b, self.final_deg, self.taxi_kts));
        }
        let (s, v) = self.along(t - self.t_a);
        let f = if self.len_m > 0.0 {
            s / self.len_m
        } else {
            1.0
        };
        let pos = gc_point(self.a.pos, self.b.pos, f);
        let hdg = if self.len_m - s < 50.0 {
            self.final_deg
        } else {
            gc_bearing(pos, self.b.pos)
        };
        let ds = 30.0;
        let grad = (self.alt_at((s + ds).min(self.len_m)) - self.alt_at((s - ds).max(0.0)))
            / ((s + ds).min(self.len_m) - (s - ds).max(0.0)).max(1.0);
        Some(Fix {
            pos,
            alt_ft: self.alt_at(s),
            gs_kts: v / KT_TO_MS,
            hdg_deg: hdg,
            vs_fpm: grad * v * 60.0,
            on_ground: false,
            stage: Stage::Flying,
        })
    }
}

/// A whole flight between two fields, the modelled part and its times
/// (the caller's clock): off the blocks at the origin, on them at the
/// destination.
#[derive(Debug, Clone, PartialEq)]
pub struct Whole {
    pub origin: String,
    pub origin_pos: LatLon,
    pub dest: String,
    pub dest_pos: LatLon,
    pub off_block_s: f64,
    pub on_block_s: f64,
    /// The part the model flies.
    pub track: Track,
}

impl Whole {
    /// An arrival at `field` from `origin` (a place the timetable knows,
    /// [`crate::places`]; `None` for one it does not: never invented),
    /// modelled until it reaches the simulation's entry `anchor` at
    /// `t_anchor`; on the blocks here at `on_block_s`.
    pub fn inbound(
        origin: &str,
        field: &str,
        field_pos: LatLon,
        anchor: End,
        t_anchor: f64,
        on_block_s: f64,
        p: &Perf,
    ) -> Option<Whole> {
        let o = crate::places::position(origin)?;
        let a = End::field(o, 0.0, p, true);
        let cruise = cruise_level_ft(o, field_pos, p);
        let track = Track::ending_at(a, anchor, t_anchor, cruise, p);
        Some(Whole {
            origin: origin.to_string(),
            origin_pos: o,
            dest: field.to_string(),
            dest_pos: field_pos,
            off_block_s: track.t_a - TAXI_OUT_S,
            on_block_s: on_block_s.max(t_anchor),
            track,
        })
    }

    /// A departure from `field` to `dest` modelled from the simulation's
    /// handover `anchor` at `t_anchor` (or from the field itself, an
    /// [`End::field`] anchor at its wheels up); off the blocks here at
    /// `off_block_s`. `None` for a destination the timetable does not
    /// place.
    pub fn outbound(
        field: &str,
        field_pos: LatLon,
        dest: &str,
        anchor: End,
        t_anchor: f64,
        off_block_s: f64,
        p: &Perf,
    ) -> Option<Whole> {
        let d = crate::places::position(dest)?;
        let b = End::field(d, 0.0, p, false);
        let cruise = cruise_level_ft(field_pos, d, p);
        let track = Track::starting_at(anchor, b, t_anchor, cruise, p);
        Some(Whole {
            origin: field.to_string(),
            origin_pos: field_pos,
            dest: dest.to_string(),
            dest_pos: d,
            off_block_s: off_block_s.min(t_anchor),
            on_block_s: track.t_b + TAXI_IN_S,
            track,
        })
    }

    /// Where the model puts it at `t` ([`Track::at`]).
    pub fn at(&self, t: f64) -> Option<Fix> {
        self.track.at(t)
    }

    /// Done: on the blocks at the destination.
    pub fn over(&self, t: f64) -> bool {
        t > self.on_block_s.max(
            self.track.t_b
                + if self.track.b.on_field {
                    TAXI_IN_S
                } else {
                    0.0
                },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::perf::{JET_MED, PISTON_SINGLE};

    const LEMG: LatLon = (36.67, -4.50);
    const EGKK: LatLon = (51.15, -0.19);

    /// Malaga to Gatwick in an A320: taxi, climb, cruise at the class's
    /// level, descent on 3 degrees, on the great circle (its middle is the
    /// slerp's, west of the rhumb line's), about 2 h of airborne time.
    #[test]
    fn a_long_flight_climbs_cruises_and_descends_on_the_great_circle() {
        let p = &JET_MED;
        let cruise = cruise_level_ft(LEMG, EGKK, p);
        assert_eq!(cruise, 37_000.0, "northbound-east, the class's ceiling");
        let a = End::field(LEMG, 53.0, p, true);
        let b = End::field(EGKK, 202.0, p, false);
        let t = Track::starting_at(a, b, 1000.0, cruise, p);
        let hrs = t.duration_s() / 3600.0;
        assert!((1.8..2.4).contains(&hrs), "{hrs} h airborne");
        // Taxiing before the wheels up, nothing before the off-block.
        let taxi = t.at(1000.0 - 60.0).unwrap();
        assert!(taxi.on_ground && taxi.stage == Stage::Taxi && taxi.pos == LEMG);
        assert!(t.at(1000.0 - TAXI_OUT_S - 1.0).is_none());
        // Ten minutes up: climbing, below the level.
        let c = t.at(1000.0 + 600.0).unwrap();
        assert!(
            c.vs_fpm > 500.0 && c.alt_ft > 10_000.0 && c.alt_ft < cruise,
            "{c:?}"
        );
        // The middle: at the level, at the cruise speed, on the great circle.
        let mid_t = 0.5 * (t.t_a + t.t_b);
        let m = t.at(mid_t).unwrap();
        assert!((m.alt_ft - cruise).abs() < 1.0 && m.vs_fpm.abs() < 1.0);
        assert!((m.gs_kts - p.cruise_kts).abs() < 1.0);
        let off_gc = gc_dist_m(m.pos, gc_point(LEMG, EGKK, progress(LEMG, EGKK, m.pos)));
        assert!(off_gc < 50.0, "on the great circle, {off_gc} m off");
        // Ten minutes out: descending on about 3 degrees.
        let d = t.at(t.t_b - 600.0).unwrap();
        assert!(d.vs_fpm < -500.0 && d.alt_ft < cruise, "{d:?}");
        let path_deg = (d.vs_fpm / (d.gs_kts * KT_TO_MS * 60.0 * 3.28084))
            .atan()
            .to_degrees();
        assert!((-3.3..-2.7).contains(&path_deg), "{path_deg} degrees");
        // Down: taxiing in, then gone.
        let land = t.at(t.t_b).unwrap();
        assert!((land.alt_ft - 202.0).abs() < 1.0 && gc_dist_m(land.pos, EGKK) < 1.0);
        assert!(t.at(t.t_b + 60.0).unwrap().on_ground);
        assert!(t.at(t.t_b + TAXI_IN_S + 1.0).is_none());
    }

    /// The level by distance, on the semicircular rule.
    #[test]
    fn the_level_follows_the_distance_and_the_direction() {
        let lxgb = (36.15, -5.35);
        // 54 nm north-east: FL100 would be even; eastbound takes FL090.
        assert_eq!(cruise_level_ft(lxgb, LEMG, &JET_MED), 9_000.0);
        assert_eq!(cruise_level_ft(LEMG, lxgb, &JET_MED), 10_000.0);
        let lemd = (40.47, -3.56);
        let up = cruise_level_ft(LEMG, lemd, &JET_MED);
        assert!((25_000.0..=29_000.0).contains(&up), "{up}");
        assert_eq!(cruise_level_ft(LEMG, lemd, &PISTON_SINGLE), 9_000.0);
        assert_eq!(
            cruise_level_ft(lemd, LEMG, &crate::perf::TURBOPROP),
            24_000.0
        );
    }

    /// A hop too short for the level: the climb meets the 3 degree
    /// descent, the ends exact, the time monotonic.
    #[test]
    fn a_short_hop_peaks_where_the_climb_meets_the_descent() {
        let p = &JET_MED;
        let a = End::field((36.15, -5.35), 15.0, p, true);
        let b = End::field(LEMG, 53.0, p, false);
        let t = Track::starting_at(a, b, 0.0, 37_000.0, p);
        assert!(t.peak_ft < 37_000.0 && t.peak_ft > 5_000.0, "{}", t.peak_ft);
        let mut last = 0.0;
        let mut top: f64 = 0.0;
        let mut s = 0.0;
        while s <= t.t_b {
            let f = t.at(s).unwrap();
            let d = gc_dist_m(a.pos, f.pos);
            assert!(d + 1.0 >= last, "moves forward");
            last = d;
            top = top.max(f.alt_ft);
            s += 5.0;
        }
        assert!((top - t.peak_ft).abs() < 200.0);
        assert!((t.at(t.t_b).unwrap().alt_ft - 53.0).abs() < 1.0);
    }

    /// An anchor in the air: the model reaches it exactly at its time, and
    /// past it holds there (never jumps) until the simulation takes it.
    #[test]
    fn an_air_anchor_is_met_exactly_and_held() {
        let p = &JET_MED;
        let a = End::field(EGKK, 202.0, p, true);
        let entry = End {
            pos: (37.0, -4.6),
            alt_ft: 11_053.0,
            gs_kts: 250.0,
            on_field: false,
        };
        let t = Track::ending_at(a, entry, 5_000.0, 37_000.0, p);
        let f = t.at(5_000.0).unwrap();
        assert!(gc_dist_m(f.pos, entry.pos) < 0.01);
        assert!((f.alt_ft - 11_053.0).abs() < 1e-6 && (f.gs_kts - 250.0).abs() < 1e-6);
        let held = t.at(5_090.0).unwrap();
        assert_eq!(held.stage, Stage::Holding);
        assert_eq!(held.pos, entry.pos);
        // A second before: a second's flight short of it.
        let b4 = t.at(4_999.0).unwrap();
        let gap = gc_dist_m(b4.pos, entry.pos);
        assert!((gap - 250.0 * KT_TO_MS).abs() < 5.0, "{gap} m");
        assert!(
            t.t_a < 5_000.0 - 1.8 * 3600.0,
            "left Gatwick about 2 h before"
        );
    }

    /// The model's cost, measured (`cargo test --release -p traffic --
    /// --ignored the_model_is_cheap --nocapture`): a track built and a
    /// position by time, each per flight; the link evaluates one position
    /// per shown flight a second and builds a track once per flight.
    #[test]
    #[ignore]
    fn the_model_is_cheap() {
        let p = &JET_MED;
        let a = End::field(LEMG, 53.0, p, true);
        let b = End::field(EGKK, 202.0, p, false);
        let n = 200_000;
        let t0 = std::time::Instant::now();
        let mut tracks = Vec::with_capacity(n);
        for i in 0..n {
            tracks.push(Track::starting_at(a, b, i as f64, 37_000.0, p));
        }
        let build = t0.elapsed().as_secs_f64() / n as f64;
        let t1 = std::time::Instant::now();
        let mut sum = 0.0;
        for (i, t) in tracks.iter().enumerate() {
            sum += t.at(3_000.0 + i as f64 * 0.01).map_or(0.0, |f| f.alt_ft);
        }
        let at = t1.elapsed().as_secs_f64() / n as f64;
        println!(
            "a track built in {:.2} us, a position in {:.2} us ({sum:.0})",
            build * 1e6,
            at * 1e6
        );
    }
}
