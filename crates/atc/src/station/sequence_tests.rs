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
    }
}

pub(super) fn times(runway: &str, to_thr_s: f64, callsign: &str, at: Instant) -> RunwayTimes {
    RunwayTimes {
        runway: runway.into(),
        to_thr_s,
        clear_s: to_thr_s + 60.0,
        dist_nm: to_thr_s * 140.0 / 3600.0,
        callsign: callsign.into(),
        icao_type: Some("B738".into()),
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
