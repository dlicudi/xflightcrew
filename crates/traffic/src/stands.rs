//! THE STAND PLAN (2026-10-13): a field allocates each rotation its stand
//! when the day is planned, as an airport's stand planning does, not when
//! the aircraft turns up: the departure's gate is on the board hours before
//! it boards, and the arrival parks there. The pilot's board at LEMG
//! showed departures Boarding with an empty Stand column: a stand was only
//! known once an aircraft stood on one (the inbound parked, or the start
//! put it there), and a departure the start could not stand (the stands
//! kept free for the arrivals) had none until it appeared at its time.
//!
//! Pure and deterministic from the field and the rotations: each takes a
//! stand its type fits ([`FieldModel::takes`]), a contact gate before a
//! remote stand, clear of the lanes where
//! the apron has one (as a parked aircraft is, [`parked_lane_clear_m`]), of
//! the tightest size free, from a quarter of an hour before its arrival to
//! a quarter of an hour after its departure, never on a stand drawn over
//! one already planned for then ([`STAND_TAKEN_M`]). The aircraft in the
//! sim take their planned stand where it is free when they get there; the
//! board shows the stand the aircraft actually holds once it holds one (a
//! gate change, as a real board makes).
//!
//! Also the board's NAME of a stand ([`label`]): the apt.dat name with a
//! redundant "Gate"/"Stand" word dropped, the field's own number kept.

use std::collections::HashMap;

use crate::field::{stand_width, FieldModel, STAND_TAKEN_M};
use crate::kin::dist_m;
use crate::rotation::Rotation;
use crate::schedule::parked_lane_clear_m;

/// A stand is held this long before an arrival's estimate and after a
/// departure's, seconds: the early arrival, the late push.
pub const STAND_BUFFER_S: f64 = 15.0 * 60.0;

/// The stand of every flight of `rots` (both callsigns of a rotation name
/// its stand) at `field`; a rotation no stand takes has none.
pub fn plan(field: &FieldModel, rots: &[Rotation]) -> HashMap<String, String> {
    let stands = &field.net.stands;
    // Each stand's set: itself and the stands drawn over it.
    let near: Vec<Vec<usize>> = stands
        .iter()
        .map(|s| {
            stands
                .iter()
                .enumerate()
                .filter(|(_, o)| dist_m((o.lat, o.lon), (s.lat, s.lon)) < STAND_TAKEN_M)
                .map(|(j, _)| j)
                .collect()
        })
        .collect();
    let lane: Vec<f64> = stands
        .iter()
        .map(|s| field.lane_clearance_m((s.lat, s.lon)))
        .collect();
    let mut busy: Vec<Vec<(f64, f64)>> = vec![Vec::new(); stands.len()];
    let mut order: Vec<&Rotation> = rots.iter().collect();
    order.sort_by(|a, b| {
        a.arr
            .est_s
            .total_cmp(&b.arr.est_s)
            .then_with(|| a.arr.callsign.cmp(&b.arr.callsign))
    });
    let mut out = HashMap::new();
    for r in order {
        let from = r.arr.est_s.min(r.arr.sched_s) - STAND_BUFFER_S;
        let to = r.dep.est_s.max(r.dep.sched_s) + STAND_BUFFER_S;
        let free = |i: usize| {
            near[i]
                .iter()
                .all(|&j| busy[j].iter().all(|(a, b)| to <= *a || from >= *b))
        };
        let fit: Vec<usize> = (0..stands.len())
            .filter(|&i| field.takes(&stands[i], r.icao_type) && free(i))
            .collect();
        // A contact gate first (apt.dat's "gate" kind: LEMG's 31 with
        // their bridges, where its 94 tie-downs are the remote aprons),
        // then a remote stand; each clear of the lanes where one is.
        let gate = |i: usize| stands[i].kind.eq_ignore_ascii_case("gate");
        let clear = |i: usize| lane[i] >= parked_lane_clear_m(r.icao_type);
        let pool = [(true, true), (true, false), (false, true), (false, false)]
            .into_iter()
            .map(|(g, c)| {
                fit.iter()
                    .copied()
                    .filter(|&i| (!g || gate(i)) && (!c || clear(i)))
                    .collect::<Vec<_>>()
            })
            .find(|p| !p.is_empty())
            .unwrap_or_default();
        let Some(tight) = pool.iter().map(|&i| stand_width(&stands[i])).min() else {
            continue;
        };
        let pick = pool
            .into_iter()
            .filter(|&i| stand_width(&stands[i]) == tight)
            .min_by_key(|&i| spread_key(&r.arr.callsign, &stands[i].name));
        let Some(i) = pick else {
            continue;
        };
        busy[i].push((from, to));
        out.insert(r.arr.callsign.clone(), stands[i].name.clone());
        out.insert(r.dep.callsign.clone(), stands[i].name.clone());
    }
    out
}

/// A stable order of stands for one flight, so the day's flights spread
/// over the apron rather than all queueing for the first stand in the file.
fn spread_key(callsign: &str, stand: &str) -> u64 {
    callsign
        .bytes()
        .chain([b'|'])
        .chain(stand.bytes())
        .fold(0xCBF2_9CE4_8422_2325u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01B3)
        })
}

/// The words an apt.dat stand name may open with that only say what it is
/// ("Gate 21", "Stand 12", "Parking 3"): an airport's board names the
/// stand by its number alone, under a column headed Stand.
const KIND_WORDS: &[&str] = &[
    "gate",
    "stand",
    "parking",
    "parking stand",
    "position",
    "pos",
];

/// The board's name for stand `name` among the field's `all` stand names:
/// the apt.dat name with a leading "Gate", "Stand", "Parking" or
/// "Position" (any case, then a space, '-', '.' or '#') dropped, the
/// field's own number kept as written ("02", "90F", "W3"), so one column
/// reads alike: LEMG's file names its contact stands "Gate 14" to "Gate 68"
/// and the remote ones bare ("02" to "131"), and the board showed "10",
/// "77", "Gate 21" (2026-10-13). A name that would then read as another
/// stand's ("Gate 5" beside a "5") keeps its word; one that is nothing but
/// the word stays as it is.
pub fn label(name: &str, all: &[&str]) -> String {
    let name = name.split_whitespace().collect::<Vec<_>>().join(" ");
    let lower = name.to_ascii_lowercase();
    let mut words: Vec<&&str> = KIND_WORDS.iter().collect();
    words.sort_by_key(|w| std::cmp::Reverse(w.len()));
    for w in words {
        let Some(rest) = lower.strip_prefix(*w) else {
            continue;
        };
        let rest_cut = rest.trim_start_matches([' ', '-', '.', '#']);
        if rest_cut.len() == rest.len() || rest_cut.is_empty() {
            continue;
        }
        let short = name[name.len() - rest_cut.len()..].to_string();
        let clash = all.iter().any(|o| o.eq_ignore_ascii_case(&short));
        return if clash { name } else { short };
    }
    name
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field::fixtures;
    use crate::rotation::FlightLeg;

    #[test]
    fn a_stand_is_named_by_its_number() {
        let all = ["02", "77", "Gate 21", "Gate 5", "5", "Stand 12", "GA North"];
        assert_eq!(label("Gate 21", &all), "21");
        assert_eq!(label("gate  14", &all), "14");
        assert_eq!(label("Stand 12", &all), "12");
        assert_eq!(label("STAND-W3", &all), "W3");
        assert_eq!(label("Parking Stand 7", &all), "7");
        assert_eq!(label("77", &all), "77");
        assert_eq!(label("02", &all), "02");
        assert_eq!(label("Gate 5", &all), "Gate 5", "a bare 5 is another stand");
        assert_eq!(label("GA North", &all), "GA North");
        assert_eq!(label("Gate", &all), "Gate");
        assert_eq!(label("Gatehouse 2", &all), "Gatehouse 2");
    }

    fn rot(n: u32, arr_s: f64, dep_s: f64) -> Rotation {
        let leg = |cs: String, t: f64| FlightLeg {
            callsign: cs,
            spoken: String::new(),
            other: "EGKK",
            city: "London Gatwick",
            sched_s: t,
            est_s: t,
        };
        Rotation {
            carrier: crate::airlines::EZY,
            icao_type: "A320",
            arr: leg(format!("EZY{n}"), arr_s),
            dep: leg(format!("EZY{}", n + 1), dep_s),
            night_stop: false,
            seed: u64::from(n),
        }
    }

    /// Every rotation has its stand, both its flights the same one, and two
    /// rotations on the ground at once never share a stand; the plan is the
    /// same every time.
    #[test]
    fn every_rotation_has_its_own_stand_while_it_is_there() {
        let field = FieldModel::load(&fixtures::path("ulmm.dat"), "ULMM")
            .unwrap()
            .unwrap();
        // Three at a time on ULMM's five jet stands, all day.
        let rots: Vec<Rotation> = (0..30)
            .map(|k| {
                let t = 3600.0 * 8.0 + 1200.0 * f64::from(k);
                rot(1000 + 2 * k, t, t + 2700.0)
            })
            .collect();
        let p = plan(&field, &rots);
        assert_eq!(p, plan(&field, &rots));
        for r in &rots {
            let s = p.get(&r.arr.callsign).expect("a stand");
            assert_eq!(p.get(&r.dep.callsign), Some(s));
        }
        for (i, a) in rots.iter().enumerate() {
            for b in &rots[i + 1..] {
                let overlap = a.arr.est_s - STAND_BUFFER_S < b.dep.est_s + STAND_BUFFER_S
                    && b.arr.est_s - STAND_BUFFER_S < a.dep.est_s + STAND_BUFFER_S;
                if overlap {
                    assert_ne!(p[&a.arr.callsign], p[&b.arr.callsign]);
                }
            }
        }
    }
}
