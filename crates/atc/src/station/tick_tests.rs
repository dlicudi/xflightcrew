//! The monitor's ticks per strip: a traffic strip's tick moves that strip's
//! cells and nothing of ours.

use super::book_tests::{along, book_runways};
use super::strip_tests::{gatwick, ClearStrips};
use super::tests::SIDE_STATE_LOCK;
use super::*;

/// Every cell of OUR strip that is not at its boot value, by name.
pub(super) fn own_strip_moved() -> Vec<&'static str> {
    let s = &OWN_STRIP;
    let mut moved = Vec::new();
    macro_rules! some {
        ($($c:ident),*) => {
            $( if s.$c.lock().unwrap().is_some() { moved.push(stringify!($c)); } )*
        };
    }
    macro_rules! set {
        ($($c:ident),*) => {
            $( if *s.$c.lock().unwrap() { moved.push(stringify!($c)); } )*
        };
    }
    some!(
        clearance,
        squawk_nudge,
        taxi_route,
        takeoff_hold,
        landing_hold,
        pending_landing_release,
        vectors,
        resequenced_order,
        line_up_condition,
        departure_handoff,
        landing_handoff,
        assigned_squawk,
        ifr_clearance,
        squawk_watch,
        handoff,
        pending_resequence,
        self_taxi,
        ifr_flight,
        go_around_last
    );
    set!(
        taxi_to_stand,
        pending_takeoff_clearance,
        pending_line_up,
        pending_go_around,
        zone_challenge
    );
    if s.uncontrolled_arrival.load(Ordering::SeqCst) {
        moved.push("uncontrolled_arrival");
    }
    if !s.own_fixes.lock().unwrap().is_empty() {
        moved.push("own_fixes");
    }
    moved
}

/// A fresh field with every strip at rest and one traffic strip open.
fn traffic_on_gatwick(n: u32) -> std::sync::Arc<FlightStrip> {
    Station::uninstall();
    *STANDING.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *STRIP_SEEN.lock().unwrap_or_else(|e| e.into_inner()) = None;
    gatwick().install();
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    open_traffic_strip(n)
}

fn airborne(alt_ft: f64) -> AircraftState {
    AircraftState {
        alt_ft,
        ..AircraftState::at_position(f64::NAN, f64::NAN, false, f64::NAN)
    }
}

fn cleared_to_land(runway: &str) -> ClearanceSnap {
    ClearanceSnap {
        state: tower::State::ClearedToLand,
        callsign: "Easy four five six".into(),
        runway_ident: runway.into(),
        pending_readback: None,
        holds_runway: true,
    }
}

#[test]
fn a_traffic_go_around_moves_only_its_strip() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let t = traffic_on_gatwick(1);
    *t.clearance.lock().unwrap() = Some(cleared_to_land("26L"));
    let (lat, lon) = along(&book_runways()[0], 0.5);
    let ghost = TcasTarget {
        callsign: "BAW12".into(),
        lat,
        lon,
        ele_ft: 200.0,
        on_ground: true,
        vs_fpm: 0.0,
    };
    let orders = GO_AROUND_ORDERS.load(Ordering::SeqCst);
    let call = go_around_tick_for(&t, &airborne(600.0), &[ghost], 400.0, true, Instant::now())
        .expect("the traffic's go-around");
    assert!(
        call.line.starts_with("Easy four five six, go around"),
        "{}",
        call.line
    );
    assert!(
        *t.pending_go_around.lock().unwrap(),
        "its FSM sync is armed"
    );
    assert!(t.go_around_last.lock().unwrap().is_some(), "its pacing");
    assert_eq!(
        GO_AROUND_ORDERS.load(Ordering::SeqCst),
        orders,
        "the crew flies no traffic order"
    );
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

#[test]
fn a_traffic_landing_hold_moves_only_its_strip() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let t = traffic_on_gatwick(2);
    *t.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Inbound,
        ..cleared_to_land("26L")
    });
    *t.landing_hold.lock().unwrap() = Some(LandingHold {
        callsign: "Easy four five six".into(),
        runway: "two six left".into(),
        runway_ident: "26L".into(),
        line: "Easy four five six, runway two six left, cleared to land.".into(),
    });
    // Ours cleared to land on 26L: the book holds the traffic's release.
    *OWN_STRIP.clearance.lock().unwrap() = Some(cleared_to_land("26L"));
    assert!(landing_hold_tick_for(&t, &airborne(1500.0), &[], Instant::now()).is_none());
    assert!(
        t.landing_hold.lock().unwrap().is_some(),
        "held for our landing"
    );
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let line = landing_hold_tick_for(&t, &airborne(1500.0), &[], Instant::now())
        .expect("released on a clear strip");
    assert!(line.contains("cleared to land"), "{line}");
    let c = t.clearance.lock().unwrap();
    assert_eq!(
        c.as_ref().map(|c| c.state),
        Some(tower::State::ClearedToLand)
    );
    assert!(t.pending_landing_release.lock().unwrap().is_some());
    drop(c);
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

#[test]
fn a_traffic_takeoff_hold_moves_only_its_strip() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let t = traffic_on_gatwick(3);
    let t0 = Instant::now();
    *t.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "Easy four five six".into(),
        runway_ident: "26L".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *t.takeoff_hold.lock().unwrap() = Some(TakeoffHold {
        callsign: "Easy four five six".into(),
        runway: "two six left".into(),
        runway_ident: "26L".into(),
        clear_since: None,
        armed_at: t0,
        logged_at: t0,
        lined_up: false,
        behind: false,
        held_for: None,
        cut_in_logged: Default::default(),
    });
    let on_ground = AircraftState::at_position(f64::NAN, f64::NAN, true, 0.0);
    let mut released = None;
    for s in [0, 10, 20] {
        let at = t0 + std::time::Duration::from_secs(s);
        released = released.or(takeoff_hold_tick_for(&t, &on_ground, &[], 200.0, at));
    }
    let line = released.expect("released on a clear final");
    assert!(line.contains("cleared for takeoff"), "{line}");
    assert!(*t.pending_takeoff_clearance.lock().unwrap(), "its FSM sync");
    assert_eq!(
        t.clearance.lock().unwrap().as_ref().map(|c| c.state),
        Some(tower::State::TakeoffCleared)
    );
    assert!(t.takeoff_hold.lock().unwrap().is_none());
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

#[test]
fn a_traffic_handoff_moves_only_its_strip() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let t = traffic_on_gatwick(4);
    *t.handoff.lock().unwrap() = Some(Handoff {
        line: "Easy four five six, contact Gatwick Director on one two six decimal eight two five."
            .into(),
        min_alt_ft: 900.0,
        to: "Gatwick Director".into(),
        khz: 126825,
    });
    assert!(
        handoff_tick_for(&t, &airborne(500.0)).is_none(),
        "below the arming height"
    );
    let line = handoff_tick_for(&t, &airborne(1500.0)).expect("the handoff");
    assert!(line.contains("Director"), "{line}");
    assert_eq!(
        t.departure_handoff.lock().unwrap().clone(),
        Some(("Gatwick Director".to_string(), 126825))
    );
    assert!(t.handoff.lock().unwrap().is_none(), "once");
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

#[test]
fn a_traffic_squawk_watch_moves_only_its_strip() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let t = traffic_on_gatwick(5);
    *t.squawk_watch.lock().unwrap() = Some(SquawkWatch {
        code: 4501,
        callsign: "Easy four five six".into(),
        speaker: StationSpeaker::Approach,
        observed: false,
        check_in_owed: false,
        first_seen: None,
        last_nudge: None,
        nudges: 0,
    });
    let t0 = Instant::now();
    let wrong = Some(1200);
    assert!(
        squawk_tick_for(&t, wrong, true, StationSpeaker::Approach, t0).is_none(),
        "grace"
    );
    let (line, _) = squawk_tick_for(
        &t,
        wrong,
        true,
        StationSpeaker::Approach,
        t0 + std::time::Duration::from_secs(25),
    )
    .expect("the nudge");
    assert!(line.contains("confirm squawk"), "{line}");
    assert_eq!(t.squawk_nudge.lock().unwrap().map(|n| n.code), Some(4501));
    let (line, _) = squawk_tick_for(
        &t,
        Some(4501),
        true,
        StationSpeaker::Approach,
        t0 + std::time::Duration::from_secs(30),
    )
    .expect("radar contact");
    assert!(line.contains("radar contact"), "{line}");
    assert!(t.squawk_watch.lock().unwrap().as_ref().unwrap().observed);
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

#[test]
fn a_traffic_vector_plan_moves_only_its_strip() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let t = traffic_on_gatwick(6);
    // 26L's threshold and its final course, three miles out on the line.
    let r = &book_runways()[0];
    let (thr_lat, thr_lon) = (r.lat_b, r.lon_b);
    let course = r.heading_b_deg();
    let (lat, lon) = crate::nav::project_latlon(thr_lat, thr_lon, (course + 180.0) % 360.0, 3.0);
    *t.vectors.lock().unwrap() = Some(VectorPlan {
        callsign: "Easy four five six".into(),
        thr_lat,
        thr_lon,
        course_deg: course,
        runway_spoken: "two six left".into(),
        tower_spoken: Some("one two four decimal two two five".into()),
        target_alt_ft: 2500.0,
        ils: false,
        join_nm: 6.0,
        cleared: false,
        dialect: Dialect::Icao,
        last_tx: None,
        last_heading: None,
    });
    let state = AircraftState {
        alt_ft: 1200.0,
        ..AircraftState::at_position(lat, lon, false, 140.0)
    };
    let line = vector_tick_for(&t, &state, Some(course), Instant::now()).expect("established");
    assert!(
        line.contains("Cleared visual approach runway two six left"),
        "{line}"
    );
    assert_eq!(
        t.landing_handoff.lock().unwrap().as_deref(),
        Some("two six left")
    );
    assert!(
        t.vectors.lock().unwrap().is_none(),
        "approach's job is done"
    );
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

#[test]
fn a_traffic_position_note_moves_only_its_strip() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let t = traffic_on_gatwick(7);
    *t.self_taxi.lock().unwrap() = Some("26L".into());
    let (lat, lon) = along(&book_runways()[0], 0.5);
    let state = AircraftState {
        alt_ft: 800.0,
        ..AircraftState::at_position(lat, lon, false, 150.0)
    };
    note_aircraft_state_for(&t, &state);
    assert_eq!(t.own_fixes.lock().unwrap().len(), 1, "its fix");
    assert!(
        t.self_taxi.lock().unwrap().is_none(),
        "its own route ends at the liftoff"
    );
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}
