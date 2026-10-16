//! The landing sequence (2026-10-14): computed from the strips' runway
//! times, never stored; the order spoken to us holds through a jitter.

use super::strip_tests::ClearStrips;
use super::tests::SIDE_STATE_LOCK;
use super::*;
use std::time::Duration;

/// Clears our own strip's sequence cells however the test ends.
pub(super) struct ClearOwnSequence;

impl Drop for ClearOwnSequence {
    fn drop(&mut self) {
        *OWN_STRIP
            .runway_times
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = None;
        *OWN_STRIP
            .arrival_order
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = None;
        retire_approach_speed(&OWN_STRIP);
    }
}

pub(super) fn times(runway: &str, to_thr_s: f64, callsign: &str, at: Instant) -> RunwayTimes {
    RunwayTimes {
        runway: runway.into(),
        to_thr_s,
        clear_s: to_thr_s + 60.0,
        needs_clear_s: 40.0,
        dist_nm: to_thr_s * 140.0 / 3600.0,
        callsign: callsign.into(),
        icao_type: Some("B738".into()),
        depart_gap_s: None,
        at,
    }
}

/// Every flight landing on the runway, nearest the threshold first: a
/// flight for another runway, over its threshold, or whose producer went
/// quiet has no place in it.
#[test]
fn the_landing_sequence_is_ordered_by_time_to_the_threshold() {
    let _g = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _s = ClearStrips;
    let _o = ClearOwnSequence;
    let now = Instant::now();
    note_traffic_state(1, AircraftState::at_position(0.0, 0.0, false, 140.0));
    note_traffic_state(2, AircraftState::at_position(0.0, 0.0, false, 140.0));
    note_traffic_state(3, AircraftState::at_position(0.0, 0.0, false, 140.0));
    note_traffic_state(4, AircraftState::at_position(0.0, 0.0, false, 140.0));
    note_traffic_times(1, Some(times("13", 300.0, "AAA1", now)));
    note_traffic_times(2, Some(times("13", 100.0, "BBB2", now)));
    note_traffic_times(3, Some(times("31", 50.0, "CCC3", now)));
    let stale = now - Duration::from_secs_f64(TIMES_FRESH_S + 1.0);
    note_traffic_times(4, Some(times("13", 80.0, "DDD4", stale)));
    note_own_times(Some(times("13", 200.0, "US", now)));
    assert_eq!(
        landing_sequence("13", now),
        vec![
            (FlightId::Traffic(2), 100.0),
            (FlightId::Own, 200.0),
            (FlightId::Traffic(1), 300.0),
        ]
    );
    assert_eq!(own_number_at(now), Some(2));
    let l = own_leader_at(now).unwrap();
    assert_eq!(
        (l.id, l.callsign.as_str(), l.gap_s),
        (FlightId::Traffic(2), "BBB2", 100.0)
    );
}

/// An order once SPOKEN to us holds unless the gap reverses by more than
/// [`SEQUENCE_REVERSE_S`]: a leader that drifts a few seconds behind us is
/// still the one we follow, and a flight we were told is behind us goes
/// ahead only once it is clearly in front. Nothing spoken, the times
/// decide every tick.
#[test]
fn an_order_once_spoken_holds_until_the_gap_reverses() {
    let _g = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _s = ClearStrips;
    let _o = ClearOwnSequence;
    let now = Instant::now();
    note_traffic_state(5, AircraftState::at_position(0.0, 0.0, false, 140.0));
    note_traffic_state(6, AircraftState::at_position(0.0, 0.0, false, 140.0));
    note_own_times(Some(times("13", 200.0, "US", now)));
    note_traffic_times(5, Some(times("13", 190.0, "LEAD5", now)));
    note_traffic_times(6, Some(times("13", 215.0, "BEHIND6", now)));
    assert_eq!(own_number_at(now), Some(2));
    note_order_spoken(now);
    // The leader drifts 20 s behind us, the one behind 20 s ahead: the
    // spoken order stands.
    note_traffic_times(5, Some(times("13", 220.0, "LEAD5", now)));
    note_traffic_times(6, Some(times("13", 180.0, "BEHIND6", now)));
    assert_eq!(own_number_at(now), Some(2));
    assert_eq!(own_leader_at(now).unwrap().id, FlightId::Traffic(5));
    // Reversed by more than 30 s each way: the order turns.
    note_traffic_times(5, Some(times("13", 240.0, "LEAD5", now)));
    note_traffic_times(6, Some(times("13", 160.0, "BEHIND6", now)));
    assert_eq!(own_number_at(now), Some(2));
    assert_eq!(own_leader_at(now).unwrap().id, FlightId::Traffic(6));
    // Nothing spoken: the times alone.
    *OWN_STRIP.arrival_order.lock().unwrap() = None;
    note_traffic_times(6, Some(times("13", 210.0, "BEHIND6", now)));
    assert_eq!(own_number_at(now), Some(1));
    assert!(own_leader_at(now).is_none());
}

/// A MAYDAY is number one (Doc 4444 15.1.2): once declared airborne, no
/// flight still flying its final is ahead of us, and our landing clearance
/// is never withheld behind one. EGPH (sortie 1792158539-full_auto,
/// 2026-10-16): fuel down to about 800 kg and a landing held behind traffic.
#[test]
fn a_mayday_is_number_one_and_its_landing_is_never_held_for_a_lander() {
    let _g = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _s = ClearStrips;
    let _o = ClearOwnSequence;
    struct ClearDistress;
    impl Drop for ClearDistress {
        fn drop(&mut self) {
            OWN_STRIP.distress.store(false, Ordering::SeqCst);
        }
    }
    let _d = ClearDistress;
    OWN_STRIP.distress.store(false, Ordering::SeqCst);
    let now = Instant::now();
    note_traffic_state(8, AircraftState::at_position(0.0, 0.0, false, 140.0));
    note_traffic_times(8, Some(times("26L", 100.0, "LEAD8", now)));
    note_own_times(Some(times("26L", 200.0, "US", now)));
    assert_eq!(own_number_at(now), Some(2));
    assert!(landing_leader_blocking(&OWN_STRIP, "26L", &[], now).is_some());
    // The mayday, airborne, through the station's own reply path.
    let mut st = super::strip_tests::gatwick();
    let ctx = st.make_ctx(
        Wx {
            wind_from_deg: 250.0,
            wind_kts: 8.0,
        },
        "QNH one zero one three".into(),
        "Speedbird one two three".into(),
        false,
        false,
        None,
        false,
        None,
    );
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 8.0,
    };
    let r = st.respond(
        "mayday mayday mayday fuel",
        Service::Tower,
        tower::Intent::Mayday,
        &ctx,
        wx,
    );
    assert!(r.contains("roger mayday"), "{r}");
    assert!(OWN_STRIP.distress.load(Ordering::SeqCst));
    assert_eq!(own_number_at(now), Some(1));
    assert!(own_leader_at(now).is_none());
    assert!(landing_leader_blocking(&OWN_STRIP, "26L", &[], now).is_none());
}

/// A 737's approach speeds: its 145 kt approach plus ten, no faster than 250.
fn b738_speeds() -> Option<crate::party_line::ApproachSpeeds> {
    Some(crate::party_line::ApproachSpeeds {
        min_kts: 155.0,
        max_kts: Some(250.0),
    })
}

/// Puts the crew's speed provider back however the test ends.
struct ClearSpeeds;

impl Drop for ClearSpeeds {
    fn drop(&mut self) {
        *crate::party_line::APPROACH_SPEEDS
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = None;
    }
}

/// The schedule's band where the type can fly it: 250 beyond 20 nm only
/// below FL100, 210, 180 to the join, 160 to 4 nm, then nothing (the 160
/// simply ends); never below the type's minimum nor above its ceiling.
/// Behind a leader we are short of, the minimum approach speed instead,
/// standing until we are 15 s ahead of our place.
#[test]
fn approach_assigns_the_traffics_bands_where_the_type_can_fly_them() {
    let jet = b738_speeds().unwrap();
    let w = |nm: f64, alt: f64| wanted_speed(nm, alt, jet, f64::NEG_INFINITY, false);
    assert_eq!(w(25.0, 8000.0), Some((250.0, "band")));
    assert_eq!(w(25.0, 12_000.0), None, "the 250 is below FL100 only");
    assert_eq!(w(18.0, 6000.0), Some((210.0, "band")));
    assert_eq!(w(12.0, 4000.0), Some((180.0, "band")));
    assert_eq!(w(8.0, 2500.0), Some((160.0, "band")));
    assert_eq!(w(3.5, 1100.0), None, "inside 4 nm the 160 ends");
    // A Cessna (65 kt approach) is given none of them.
    let cessna = crate::party_line::ApproachSpeeds {
        min_kts: 75.0,
        max_kts: Some(150.0),
    };
    assert_eq!(
        wanted_speed(8.0, 2500.0, cessna, f64::NEG_INFINITY, false),
        None
    );
    // A turboprop that cannot fly 160 is not asked to.
    let slow = crate::party_line::ApproachSpeeds {
        min_kts: 170.0,
        max_kts: None,
    };
    assert_eq!(
        wanted_speed(8.0, 2500.0, slow, f64::NEG_INFINITY, false),
        None
    );
    // Behind a leader, short of our place: the minimum approach speed.
    assert_eq!(
        wanted_speed(12.0, 4000.0, jet, 20.0, false),
        Some((155.0, "spacing"))
    );
    assert_eq!(
        wanted_speed(12.0, 4000.0, jet, -10.0, true),
        Some((155.0, "spacing"))
    );
    assert_eq!(
        wanted_speed(12.0, 4000.0, jet, -20.0, true),
        Some((180.0, "band"))
    );
}

/// The words in each dialect: CAP 413 "number two, following a Boeing 737
/// on 8 mile final", FAA "number two, follow a Boeing 737, 8 mile final";
/// "maintain one six zero knots to four DME" where the 160 ends.
#[test]
fn approach_says_the_sequence_and_the_speed_in_the_dialect() {
    assert_eq!(
        sequence_words(Dialect::Icao, 2, Some("B738"), 8.2),
        "number two, following a Boeing 737 on 8 mile final"
    );
    assert_eq!(
        sequence_words(Dialect::Faa, 3, Some("A320"), 5.0),
        "number three, follow an A320, 5 mile final"
    );
    assert_eq!(
        sequence_words(Dialect::Icao, 2, Some("ZZZZ"), 1.2),
        "number two, following traffic on short final"
    );
    assert_eq!(
        speed_words(Dialect::Icao, 160.0, true),
        "maintain one six zero knots to four DME"
    );
    assert_eq!(
        speed_words(Dialect::Faa, 160.0, false),
        "maintain one six zero knots until four mile final"
    );
    assert_eq!(
        speed_words(Dialect::Icao, 210.0, true),
        "reduce speed to two one zero knots"
    );
    assert_eq!(
        tower::heard_speed_kts("one six zero knots, Speedbird one"),
        Some(160)
    );
    assert_eq!(tower::heard_speed_kts("speed one eight zero"), Some(180));
    assert_eq!(tower::heard_speed_kts("descending three thousand"), None);
}

/// A vectors plan for our 737 to runway 13 at a field with its threshold at
/// (36.0, -4.0), course 130.
fn plan_13(join_nm: f64) -> VectorPlan {
    VectorPlan {
        callsign: "Speedbird one".into(),
        thr_lat: 36.0,
        thr_lon: -4.0,
        course_deg: 130.0,
        runway_spoken: "one three".into(),
        tower_spoken: Some("one one eight decimal one five".into()),
        target_alt_ft: 3000.0,
        ils: false,
        join_nm,
        cleared: false,
        dialect: Dialect::Icao,
        last_tx: Some(Instant::now()),
        last_heading: Some(130),
    }
}

/// Clears our vectors however the test ends.
struct ClearVectors;

impl Drop for ClearVectors {
    fn drop(&mut self) {
        *OWN_STRIP.vectors.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *OWN_STRIP
            .approach_readback
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = None;
    }
}

/// Under vectors with a 737 ahead on our runway: approach says our number
/// and the traffic ONCE, assigns the band's speed (a readback item, read
/// back for the crew and published), says nothing more while nothing
/// changes, takes the right speed read back in silence and corrects a
/// wrong one.
#[test]
fn approach_speaks_our_number_once_and_assigns_the_band() {
    let _g = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _s = ClearStrips;
    let _o = ClearOwnSequence;
    let _v = ClearVectors;
    let _p = ClearSpeeds;
    crate::party_line::provide_approach_speeds(b738_speeds);
    let now = Instant::now();
    // Us 13 nm out to the north west of 13's threshold, at 5,000 ft.
    let (lat, lon) = crate::nav::project_latlon(36.0, -4.0, 310.0, 13.0);
    let state = AircraftState {
        alt_ft: 5000.0,
        ..AircraftState::at_position(lat, lon, false, 230.0)
    };
    note_own_times(Some(RunwayTimes {
        dist_nm: 13.0,
        ..times("13", 260.0, "US", now)
    }));
    note_traffic_state(9, AircraftState::at_position(0.0, 0.0, false, 160.0));
    note_traffic_times(
        9,
        Some(RunwayTimes {
            dist_nm: 6.0,
            clear_s: 110.0,
            ..times("13", 90.0, "RYR9", now)
        }),
    );
    *OWN_STRIP.vectors.lock().unwrap() = Some(plan_13(8.0));
    let line = vector_tick_for(&OWN_STRIP, &state, Some(130.0), now).expect("a line");
    assert_eq!(
        line,
        "Speedbird one, number two, following a Boeing 737 on 6 mile final, \
         reduce speed to one eight zero knots."
    );
    assert_eq!(crate::party_line::assigned_speed(), Some(180.0));
    assert_eq!(
        OWN_STRIP.approach_readback.lock().unwrap().as_deref(),
        Some("one eight zero knots")
    );
    // Nothing changed: nothing said.
    assert_eq!(vector_tick_for(&OWN_STRIP, &state, Some(130.0), now), None);
    // A wrong readback is corrected, the right one taken in silence.
    assert_eq!(
        judge_speed_readback("one six zero knots, Speedbird one", "Speedbird one").as_deref(),
        Some("Speedbird one, negative, reduce speed to one eight zero knots.")
    );
    assert_eq!(
        judge_speed_readback("one eight zero knots, Speedbird one", "Speedbird one").as_deref(),
        Some("")
    );
    assert_eq!(
        judge_speed_readback("one eight zero knots", "Speedbird one"),
        None
    );
    // A go-around voids the speed.
    retire_sequence(&OWN_STRIP);
    assert_eq!(crate::party_line::assigned_speed(), None);
}

/// Short of our place behind the leader and beyond the join with the speed
/// already at our minimum: the base is delayed, the join moved out along
/// the final, "extend downwind" on a downwind-shaped leg.
#[test]
fn approach_slows_us_then_extends_the_downwind_behind_a_leader_we_cannot_follow() {
    let _g = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _s = ClearStrips;
    let _o = ClearOwnSequence;
    let _v = ClearVectors;
    let _p = ClearSpeeds;
    crate::party_line::provide_approach_speeds(b738_speeds);
    let now = Instant::now();
    // Us abeam on a downwind 12 nm from the threshold, flying 310.
    let (lat, lon) = crate::nav::project_latlon(36.0, -4.0, 270.0, 12.0);
    let state = AircraftState {
        alt_ft: 4000.0,
        ..AircraftState::at_position(lat, lon, false, 200.0)
    };
    // 230 s from the threshold along our vectors; the leader on a 4 mile
    // final, backtracking off the runway 260 s from now: we need 260 + 20
    // + 40, 90 s more than we have, and slowing to 155 kt gives 49.
    note_own_times(Some(RunwayTimes {
        dist_nm: 12.0,
        ..times("13", 230.0, "US", now)
    }));
    note_traffic_state(9, AircraftState::at_position(0.0, 0.0, false, 160.0));
    note_traffic_times(
        9,
        Some(RunwayTimes {
            dist_nm: 4.0,
            clear_s: 260.0,
            ..times("13", 90.0, "RYR9", now)
        }),
    );
    *OWN_STRIP.vectors.lock().unwrap() = Some(plan_13(8.0));
    let line = vector_tick_for(&OWN_STRIP, &state, Some(310.0), now).expect("a line");
    assert!(line.contains("number two"), "{line}");
    assert!(
        line.contains("reduce speed to one five five knots"),
        "{line}"
    );
    assert!(line.contains("extend downwind"), "{line}");
    let join = OWN_STRIP.vectors.lock().unwrap().as_ref().unwrap().join_nm;
    assert!(join > 8.5, "the join moved out to {join:.1} nm");
    assert_eq!(crate::party_line::assigned_speed(), Some(155.0));
}
