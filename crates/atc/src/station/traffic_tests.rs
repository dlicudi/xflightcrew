//! Our own AI traffic at the station: a traffic flight's call is answered on
//! its own desk, through the same keyed path as ours, and moves none of the
//! field's facts.

use super::book_tests::{along, book_runways};
use super::strip_tests::{gatwick, ClearStrips};
use super::tests::SIDE_STATE_LOCK;
use super::tick_tests::own_strip_moved;
use super::*;

const CS: &str = "Cessna three four five";

/// Gatwick with an ATIS broadcasting, 26L in use, the tower's wind calm,
/// the ghost memory empty.
fn gatwick_live() -> Station {
    Station::uninstall();
    *STANDING.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *STRIP_SEEN.lock().unwrap_or_else(|e| e.into_inner()) = None;
    let mut st = gatwick();
    st.apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_ATIS,
        khz: 136525,
        name: "ATIS".into(),
    });
    st.install();
    *TOWER_WIND.lock().unwrap() = Some((250.0, 2.0));
    st.last_qnh = Some("QNH one zero one three".into());
    st
}

/// Everything of the FIELD's a traffic call must not move.
fn field_facts(st: &Station) -> String {
    format!(
        "{:?}",
        (
            st.runway_in_use.clone(),
            *TOWER_WIND.lock().unwrap(),
            ATIS.lock()
                .unwrap()
                .as_ref()
                .map(|a| (a.letter, a.edition.is_some())),
            STANDING.lock().unwrap().as_ref().map(|m| m.len()),
            STRIP_SEEN.lock().unwrap().as_ref().map(|m| m.len()),
        )
    )
}

/// A traffic flight on the ground, west of 26L's threshold, off the strip.
fn at_the_hold() -> AircraftState {
    let r = &book_runways()[0];
    let (lat, lon) = along(r, 0.95);
    AircraftState::at_position(lat - 150.0 / 111_320.0, lon, true, 0.0)
}

/// Our aircraft on a two mile final for 26L, as a traffic flight's picture
/// shows it.
fn us_on_final() -> TrafficTarget {
    let r = &book_runways()[0];
    let (lat, lon) = crate::nav::project_latlon(r.lat_b, r.lon_b, r.heading_a_deg(), 2.0);
    TrafficTarget {
        who: FlightId::Own,
        target: TcasTarget {
            callsign: "BAW123".into(),
            lat,
            lon,
            ele_ft: 850.0,
            on_ground: false,
            vs_fpm: -600.0,
        },
    }
}

/// A ghost standing on the twin: a ghost memory note would record it.
fn ghost_on_the_twin() -> TrafficTarget {
    let (lat, lon) = along(&book_runways()[1], 0.4);
    TrafficTarget {
        who: FlightId::Traffic(9),
        target: TcasTarget {
            callsign: "EZY9".into(),
            lat,
            lon,
            ele_ft: 200.0,
            on_ground: true,
            vs_fpm: 0.0,
        },
    }
}

fn call(id: u32, text: &str, state: AircraftState, picture: Vec<TrafficTarget>) -> TrafficCall {
    TrafficCall {
        id,
        spoken: CS.into(),
        text: text.into(),
        state,
        picture,
    }
}

/// A TRAFFIC "READY" MOVES NO FIELD FACT: its taxi request and its ready
/// call are answered (the conditional or hold for our final, never a
/// takeoff clearance under it), on its own strip, and the runway in use,
/// the tower's wind, the ATIS letter and the ghost memory are as they were.
/// Its stated runway is not the field's either: a final for 08R in calm is
/// told 26L, where ours would have moved the runway in use.
#[test]
fn a_traffic_ready_call_moves_no_field_fact() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let before = field_facts(&st);
    let picture = vec![us_on_final(), ghost_on_the_twin()];

    let r = st.answer_traffic(&call(
        1,
        "Gatwick Tower, Cessna three four five, request taxi",
        at_the_hold(),
        picture.clone(),
    ));
    assert_eq!(r.id, 1);
    assert_eq!(r.speaker, StationSpeaker::Tower);
    assert!(r.line.starts_with(CS), "{}", r.line);
    assert!(r.line.contains("two six left"), "{}", r.line);
    let r = st.answer_traffic(&call(
        1,
        "Gatwick Tower, Cessna three four five, ready for departure",
        at_the_hold(),
        picture.clone(),
    ));
    assert!(!r.line.contains("cleared for takeoff"), "{}", r.line);
    assert!(r.line.starts_with(CS), "{}", r.line);
    let t = traffic_strip(1).expect("the call opened its strip");
    assert!(
        t.clearance.lock().unwrap().is_some(),
        "its clearance is on its strip"
    );
    assert!(
        t.takeoff_hold.lock().unwrap().is_some(),
        "its release watch"
    );
    assert_eq!(field_facts(&st), before, "the field's facts");
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    assert!(!st.desk.strip.is_traffic(), "our desk is back on frequency");

    // A stated runway the calm wind would grant us is not the traffic's to
    // take: it is told the runway in use.
    let r = &book_runways()[0];
    let (lat, lon) = crate::nav::project_latlon(r.lat_a, r.lon_a, r.heading_b_deg(), 3.0);
    let airborne = AircraftState {
        alt_ft: 1200.0,
        ..AircraftState::at_position(lat, lon, false, 90.0)
    };
    let final_08r = "Gatwick Tower, Cessna three four five, final runway zero eight right";
    assert_eq!(tower::parse_intent(final_08r), tower::Intent::ReportFinal);
    assert_eq!(tower::stated_runway(final_08r).as_deref(), Some("8R"));
    let r = st.answer_traffic(&call(2, final_08r, airborne, Vec::new()));
    assert!(r.line.contains("two six left"), "{}", r.line);
    assert_eq!(field_facts(&st), before, "a stated runway moved nothing");

    // Ours, the same call in the same calm, does move it: the guard is
    // what kept it for the traffic.
    let mut ours = gatwick_live();
    let calm = Wx {
        wind_from_deg: 250.0,
        wind_kts: 2.0,
    };
    let text = "Gatwick Tower, Easy one two three, final runway zero eight right";
    let intent = tower::parse_intent(text);
    assert_eq!(intent, tower::Intent::ReportFinal);
    ours.note_stated_runway(text, intent.clone());
    let (ctx, wx) = ours.ctx_for_call(
        &airborne,
        calm,
        "QNH one zero one three".into(),
        "Easy one two three".into(),
        intent,
        &[],
        Instant::now(),
    );
    ours.route_keyed(text, Service::Tower, &ctx, wx);
    assert_eq!(ours.runway_in_use.as_deref(), Some("08R"), "ours moves it");
    Station::uninstall();
}

/// THE ONE EXCEPTION, PINNED: a field with no runway in use yet takes the
/// traffic call's pick as its first (logged); from then on the traffic
/// moves nothing.
#[test]
fn a_traffic_call_initialises_the_runway_in_use_only_when_none() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    st.runway_in_use = None;
    *TOWER_WIND.lock().unwrap() = Some((250.0, 12.0));
    st.answer_traffic(&call(
        1,
        "Gatwick Tower, Cessna three four five, request taxi",
        at_the_hold(),
        Vec::new(),
    ));
    let first = st.runway_in_use.clone();
    assert!(
        first.as_deref().is_some_and(|r| r.starts_with("26")),
        "{first:?}"
    );
    // The wind turns round: the traffic's next call moves nothing.
    *TOWER_WIND.lock().unwrap() = Some((70.0, 15.0));
    st.answer_traffic(&call(
        3,
        "Gatwick Tower, Cessna three four five, request taxi",
        at_the_hold(),
        Vec::new(),
    ));
    assert_eq!(st.runway_in_use, first, "only ours re-picks it");
    Station::uninstall();
}

/// A TRAFFIC READBACK MEETS ITS OWN DESK'S GATE: a wrong runway is
/// challenged and the readback stays owed on its strip (the fleet sees it
/// and answers again); the right one is silence and pays it.
#[test]
fn a_traffic_readback_is_gated_like_ours() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let r = &book_runways()[0];
    let (lat, lon) = crate::nav::project_latlon(r.lat_b, r.lon_b, r.heading_a_deg(), 3.0);
    let on_final = AircraftState {
        alt_ft: 1100.0,
        ..AircraftState::at_position(lat, lon, false, 80.0)
    };
    let r = st.answer_traffic(&call(
        4,
        "Gatwick Tower, Cessna three four five, final runway two six left",
        on_final,
        Vec::new(),
    ));
    assert!(r.line.contains("cleared to land"), "{}", r.line);
    let owed = || {
        traffic_strip(4)
            .unwrap()
            .clearance
            .lock()
            .unwrap()
            .as_ref()
            .and_then(|c| c.pending_readback.clone())
    };
    assert!(owed().is_some(), "its readback is owed");
    let wrong = st.answer_traffic(&call(
        4,
        "Cleared to land runway two six right, Cessna three four five",
        on_final,
        Vec::new(),
    ));
    assert!(!wrong.line.is_empty(), "a wrong runway is challenged");
    assert!(owed().is_some(), "still owed after the challenge");
    let right = st.answer_traffic(&call(
        4,
        "Cleared to land runway two six left, Cessna three four five",
        on_final,
        Vec::new(),
    ));
    assert!(right.line.is_empty(), "silence: {}", right.line);
    assert!(owed().is_none(), "paid on its strip");
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

/// THE INBOX: calls come out in the order posted, once, and a new station
/// drops what nobody answered.
#[test]
fn the_traffic_inbox_is_drained_in_order_and_dies_with_the_station() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let st = gatwick_live();
    take_traffic_calls();
    post_traffic_call(call(1, "one", at_the_hold(), Vec::new()));
    post_traffic_call(call(2, "two", at_the_hold(), Vec::new()));
    let ids: Vec<u32> = take_traffic_calls().iter().map(|c| c.id).collect();
    assert_eq!(ids, vec![1, 2]);
    assert!(take_traffic_calls().is_empty(), "taken once");
    post_traffic_call(call(3, "three", at_the_hold(), Vec::new()));
    st.install();
    assert!(
        take_traffic_calls().is_empty(),
        "a new station, a new field"
    );
    Station::uninstall();
}
