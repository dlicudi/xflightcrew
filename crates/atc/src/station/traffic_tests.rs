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
        approach: false,
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

/// THE FIELD HAS A RUNWAY FROM THE INSTALL ON, FOR THE TRAFFIC: with none
/// in use, [`Station::runway_for_now`] is the pick the first traffic ground
/// call then makes (into a wind, and in calm), and it writes nothing (the
/// runway in use stays `None` for our own first call to pick as before).
/// With one in use it is that one.
#[test]
fn the_runway_for_now_is_the_first_traffic_calls_pick_and_writes_nothing() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    for wind in [(250.0, 12.0), (70.0, 15.0), (250.0, 2.0)] {
        let mut st = gatwick_live();
        st.runway_in_use = None;
        *TOWER_WIND.lock().unwrap() = Some(wind);
        let before = field_facts(&st);
        let now = st.runway_for_now();
        assert!(now.is_some(), "{wind:?}: a runway for the traffic");
        assert_eq!(field_facts(&st), before, "{wind:?}: nothing written");
        assert_eq!(st.runway_for_now(), now, "{wind:?}: the same answer twice");
        st.answer_traffic(&call(
            1,
            "Gatwick Tower, Cessna three four five, request taxi",
            at_the_hold(),
            Vec::new(),
        ));
        assert_eq!(st.runway_in_use, now, "{wind:?}: the first call's pick");
        assert_eq!(st.runway_for_now(), now, "{wind:?}: in use now");
        Station::uninstall();
    }
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

/// A traffic flight on 26L's final, `nm` out at `agl_ft`.
fn traffic_on_final(nm: f64, agl_ft: f64) -> AircraftState {
    let r = &book_runways()[0];
    let (lat, lon) = crate::nav::project_latlon(r.lat_b, r.lon_b, r.heading_a_deg(), nm);
    AircraftState {
        alt_ft: 200.0 + agl_ft,
        psi_deg: Some(r.heading_b_deg()),
        ..AircraftState::at_position(lat, lon, false, 90.0)
    }
}

/// Our aircraft standing on 26L (`on`), or clear of it on the taxiway.
fn us_on_the_ground(on: bool) -> TrafficTarget {
    let (lat, lon) = along(&book_runways()[0], 0.5);
    let lat = if on { lat } else { lat - 300.0 / 111_320.0 };
    TrafficTarget {
        who: FlightId::Own,
        target: TcasTarget {
            callsign: "BAW123".into(),
            lat,
            lon,
            ele_ft: 200.0,
            on_ground: true,
            vs_fpm: 0.0,
        },
    }
}

const FINAL_26L: &str = "Gatwick Tower, Cessna three four five, final runway two six left";
const LANDING_READBACK: &str = "Cleared to land runway two six left, Cessna three four five";

/// THE LANDING HOLD RELEASES A TRAFFIC AIRCRAFT ONCE OUR RUNWAY IS CLEAR:
/// its final report with us standing on 26L is answered "continue
/// approach" and the order says it is held; its tick holds it while we are
/// there and releases the clearance the moment we are off; the desk sync
/// puts its FSM where the tick's words did, so its readback is silence.
#[test]
fn the_landing_hold_releases_traffic_once_our_runway_is_clear() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let on_final = traffic_on_final(3.0, 900.0);
    note_traffic_state(5, on_final);
    let r = st.answer_traffic(&call(5, FINAL_26L, on_final, vec![us_on_the_ground(true)]));
    assert!(r.line.contains("continue approach"), "{}", r.line);
    let order = traffic_order(5).expect("its strip is open");
    assert!(order.hold, "its landing clearance is withheld");
    assert_eq!(order.state, tower::State::Inbound);
    assert_eq!(order.runway, "26L");

    let lines = traffic_tick(5, &[us_on_the_ground(true)], Instant::now());
    assert!(lines.is_empty(), "we are still on the runway: {lines:?}");
    assert!(traffic_order(5).unwrap().hold);

    let lines = traffic_tick(5, &[us_on_the_ground(false)], Instant::now());
    assert_eq!(lines.len(), 1, "{lines:?}");
    assert!(lines[0].contains("cleared to land"), "{}", lines[0]);
    let order = traffic_order(5).unwrap();
    assert!(!order.hold);
    assert_eq!(order.state, tower::State::ClearedToLand);
    assert_eq!(
        order.readback_owed.as_deref(),
        Some("Cleared to land runway two six left, Cessna three four five.")
    );

    st.sync_traffic_desks();
    let r = st.answer_traffic(&call(5, LANDING_READBACK, on_final, Vec::new()));
    assert!(
        r.line.is_empty(),
        "the released clearance is read back: {}",
        r.line
    );
    assert!(traffic_order(5).unwrap().readback_owed.is_none());
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

/// A TOWER GO-AROUND ORDER REACHES THE ORDER VIEW and stays there until
/// the flight says it: cleared to land, then us on the runway with the
/// traffic a mile out, its tick orders the go-around; the desk sync voids
/// the clearance (Inbound) and the order still stands; "going around" ends
/// it. The crew's go-around count never moves for traffic.
#[test]
fn a_tower_go_around_order_reaches_the_traffic_order() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let far = traffic_on_final(3.0, 900.0);
    note_traffic_state(6, far);
    let r = st.answer_traffic(&call(6, FINAL_26L, far, Vec::new()));
    assert!(r.line.contains("cleared to land"), "{}", r.line);
    let r = st.answer_traffic(&call(6, LANDING_READBACK, far, Vec::new()));
    assert!(r.line.is_empty(), "{}", r.line);
    assert!(!traffic_order(6).unwrap().go_around);

    let near = traffic_on_final(1.0, 300.0);
    note_traffic_state(6, near);
    let orders = GO_AROUND_ORDERS.load(Ordering::SeqCst);
    let lines = traffic_tick(6, &[us_on_the_ground(true)], Instant::now());
    assert_eq!(lines.len(), 1, "{lines:?}");
    assert!(
        lines[0].starts_with("Cessna three four five, go around"),
        "{}",
        lines[0]
    );
    assert!(
        !lines[0].contains("expect runway"),
        "traffic is never re-sequenced: {}",
        lines[0]
    );
    assert!(traffic_order(6).unwrap().go_around, "ordered");
    assert_eq!(GO_AROUND_ORDERS.load(Ordering::SeqCst), orders);

    st.sync_traffic_desks();
    let order = traffic_order(6).unwrap();
    assert!(order.go_around, "still owed after the sync");
    assert_eq!(order.state, tower::State::Inbound, "the clearance is void");
    let r = st.answer_traffic(&call(
        6,
        "Going around, Cessna three four five",
        near,
        vec![us_on_the_ground(true)],
    ));
    assert!(!r.line.is_empty(), "the tower acknowledges");
    assert!(
        !traffic_order(6).unwrap().go_around,
        "said: the order is done"
    );
    assert_eq!(st.runway_in_use.as_deref(), Some("26L"));
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

/// "NUMBER TWO" NAMES THE LEADER: a traffic downwind report behind us on
/// final follows us; behind another traffic flight, that flight; with the
/// final clear it is number one and follows nobody.
#[test]
fn number_two_names_the_leaders_id() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let r = &book_runways()[0];
    let (lat, lon) = crate::nav::project_latlon(along(r, 0.5).0, along(r, 0.5).1, 0.0, 1.0);
    let downwind = AircraftState {
        alt_ft: 1200.0,
        ..AircraftState::at_position(lat, lon, false, 90.0)
    };
    let words = "Gatwick Tower, Cessna three four five, downwind runway two six left";
    assert_eq!(tower::parse_intent(words), tower::Intent::ReportDownwind);

    let reply = st.answer_traffic(&call(7, words, downwind, vec![us_on_final()]));
    assert!(reply.line.contains("number two"), "{}", reply.line);
    let order = traffic_order(7).unwrap();
    assert_eq!(order.sequence, Some(2));
    assert_eq!(order.follow, Some(FlightId::Own), "it follows us");

    let mut other = us_on_final();
    other.who = FlightId::Traffic(8);
    other.target.callsign = "EZY8".into();
    let reply = st.answer_traffic(&call(7, words, downwind, vec![other]));
    assert!(reply.line.contains("number two"), "{}", reply.line);
    assert_eq!(traffic_order(7).unwrap().follow, Some(FlightId::Traffic(8)));

    let reply = st.answer_traffic(&call(7, words, downwind, Vec::new()));
    assert!(reply.line.contains("number one"), "{}", reply.line);
    let order = traffic_order(7).unwrap();
    assert_eq!((order.sequence, order.follow), (Some(1), None));
    // Cleared to land, it has no place in the sequence any more.
    st.answer_traffic(&call(7, FINAL_26L, downwind, Vec::new()));
    assert_eq!(traffic_order(7).unwrap().sequence, None);
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

/// A TRAFFIC READBACK FAILING THE GATE IS RE-CHALLENGED, observably: each
/// wrong readback is answered and the order keeps the readback owed (the
/// cap lives in the fleet).
#[test]
fn a_failed_traffic_readback_is_rechallenged() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let far = traffic_on_final(3.0, 900.0);
    st.answer_traffic(&call(9, FINAL_26L, far, Vec::new()));
    let owed = traffic_order(9).unwrap().readback_owed;
    assert!(owed.is_some());
    for n in 1..=2 {
        let r = st.answer_traffic(&call(
            9,
            "Cleared to land runway two six right, Cessna three four five",
            far,
            Vec::new(),
        ));
        assert!(!r.line.is_empty(), "challenge {n}");
        assert_eq!(
            traffic_order(9).unwrap().readback_owed,
            owed,
            "challenge {n}"
        );
    }
    let r = st.answer_traffic(&call(9, LANDING_READBACK, far, Vec::new()));
    assert!(r.line.is_empty(), "{}", r.line);
    assert!(traffic_order(9).unwrap().readback_owed.is_none());
    Station::uninstall();
}

/// A HELD TRAFFIC DEPARTURE IS RELEASED BY ITS TICK and its desk follows:
/// lined up behind us on final, released once the final is clear, the
/// desk synced to TakeoffCleared so its readback is silence.
#[test]
fn a_held_traffic_departure_is_released_and_synced() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let hold = at_the_hold();
    note_traffic_state(10, hold);
    st.answer_traffic(&call(
        10,
        "Gatwick Tower, Cessna three four five, request taxi",
        hold,
        Vec::new(),
    ));
    let r = st.answer_traffic(&call(
        10,
        "Gatwick Tower, Cessna three four five, ready for departure",
        hold,
        vec![us_on_final()],
    ));
    assert!(!r.line.contains("cleared for takeoff"), "{}", r.line);
    assert!(traffic_order(10).unwrap().hold, "held for us");
    let t0 = Instant::now();
    let mut released = Vec::new();
    for s in [0, 20] {
        released.extend(traffic_tick(
            10,
            &[],
            t0 + std::time::Duration::from_secs(s),
        ));
    }
    assert!(
        released.iter().any(|l| l.contains("cleared for takeoff")),
        "{released:?}"
    );
    let order = traffic_order(10).unwrap();
    assert_eq!(order.state, tower::State::TakeoffCleared);
    assert!(!order.hold);
    st.sync_traffic_desks();
    assert!(
        traffic_order(10).unwrap().readback_owed.is_some(),
        "the released clearance owes its readback"
    );
    let r = st.answer_traffic(&call(
        10,
        "Cleared for takeoff runway two six left, Cessna three four five",
        hold,
        Vec::new(),
    ));
    assert!(r.line.is_empty(), "{}", r.line);
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

/// A HOLD READ BACK AND A "SAY AGAIN" KEEP THEIR WORD (#300, EGKK 26L
/// 2026-10-15): a traffic departure held for a traffic lander on a short
/// final (inside the conditional line-up's range) reads the hold back (silence, nothing owed), asks "say
/// again" (answered, the hold still standing), and is released once the
/// lander has landed and reported off, before the next arrival is close.
#[test]
fn a_held_departure_reads_back_and_is_released_after_the_lander() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let lander = |state: AircraftState| TrafficTarget {
        who: FlightId::Traffic(11),
        target: TcasTarget {
            callsign: "EZY11".into(),
            lat: state.lat,
            lon: state.lon,
            ele_ft: state.alt_ft,
            on_ground: state.on_ground,
            vs_fpm: if state.on_ground { 0.0 } else { -700.0 },
        },
    };
    let on_final = traffic_on_final(1.2, 400.0);
    note_traffic_state(11, on_final);
    let r = st.answer_traffic(&call(11, FINAL_26L, on_final, Vec::new()));
    assert!(r.line.contains("cleared to land"), "{}", r.line);
    st.answer_traffic(&call(11, LANDING_READBACK, on_final, Vec::new()));

    let hold = at_the_hold();
    note_traffic_state(10, hold);
    st.answer_traffic(&call(
        10,
        "Gatwick Tower, Cessna three four five, request taxi",
        hold,
        Vec::new(),
    ));
    let r = st.answer_traffic(&call(
        10,
        "Gatwick Tower, Cessna three four five, ready for departure runway two six left",
        hold,
        vec![lander(on_final)],
    ));
    assert!(r.line.contains("hold position"), "{}", r.line);
    assert!(traffic_order(10).unwrap().hold, "held for the lander");
    assert!(traffic_order(10).unwrap().readback_owed.is_some());
    let r = st.answer_traffic(&call(
        10,
        "Holding position, Cessna three four five",
        hold,
        Vec::new(),
    ));
    assert!(r.line.is_empty(), "the hold read back: {}", r.line);
    assert!(
        traffic_order(10).unwrap().readback_owed.is_none(),
        "nothing owed"
    );
    st.answer_traffic(&call(
        10,
        "Gatwick Tower, Cessna three four five, say again",
        hold,
        Vec::new(),
    ));
    let order = traffic_order(10).unwrap();
    assert!(order.hold, "a say again never loses the hold");
    assert!(order.readback_owed.is_none(), "nor owes a readback again");

    let t0 = Instant::now();
    let at = |s: u64| t0 + std::time::Duration::from_secs(s);
    let held = traffic_tick(10, &[lander(on_final)], at(0));
    assert!(held.is_empty(), "the lander on final holds it: {held:?}");
    let (lat, lon) = along(&book_runways()[0], 0.6);
    let rolling = AircraftState::at_position(lat, lon, true, 60.0);
    note_traffic_state(11, rolling);
    let held = traffic_tick(10, &[lander(rolling)], at(30));
    assert!(
        held.is_empty(),
        "the lander on the runway holds it: {held:?}"
    );
    let off = AircraftState::at_position(lat - 300.0 / 111_320.0, lon, true, 15.0);
    note_traffic_state(11, off);
    st.answer_traffic(&call(
        11,
        "Gatwick Tower, Cessna three four five, runway vacated",
        off,
        Vec::new(),
    ));
    let mut released = Vec::new();
    for s in [60, 80] {
        released.extend(traffic_tick(10, &[lander(off)], at(s)));
    }
    assert!(
        released.iter().any(|l| l.contains("cleared for takeoff")),
        "released after the lander: {released:?}"
    );
    assert_eq!(
        traffic_order(10).unwrap().state,
        tower::State::TakeoffCleared
    );
    Station::uninstall();
}

/// A DESPAWN CLOSES THE STRIP: its claims leave the book, its order and
/// tick are gone, and the next desk sync drops its desk.
#[test]
fn a_retired_traffic_flight_leaves_the_station() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let (lat, lon) = along(&book_runways()[0], 0.5);
    note_traffic_state(11, AircraftState::at_position(lat, lon, true, 0.0));
    st.answer_traffic(&call(
        11,
        "Gatwick Tower, Cessna three four five, request taxi",
        at_the_hold(),
        Vec::new(),
    ));
    assert!(st.desks.contains_key(&FlightId::Traffic(11)));
    assert!(strip_held_by_traffic(&book_runways(), "26L"));
    retire_traffic(11);
    assert!(!strip_held_by_traffic(&book_runways(), "26L"));
    assert!(traffic_order(11).is_none());
    assert!(traffic_tick(11, &[], Instant::now()).is_empty());
    st.sync_traffic_desks();
    assert!(!st.desks.contains_key(&FlightId::Traffic(11)));
    Station::uninstall();
}

/// While our fleet flies, its picture IS the traffic: published, read back
/// whole, cleared by an install; the advisory's factor is its nearest
/// airborne aircraft, by our heading (a parked one is no factor).
#[test]
fn the_fleet_picture_is_the_one_traffic_picture() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let st = gatwick_live();
    let target = |cs: &str, lat: f64, lon: f64, ele_ft: f64, on_ground: bool| TcasTarget {
        callsign: cs.into(),
        lat,
        lon,
        ele_ft,
        on_ground,
        vs_fpm: 0.0,
    };
    assert!(traffic_picture().is_none(), "no fleet, the sim's TCAS");
    publish_traffic_picture(Some(vec![
        target("N1", 51.20, -0.20, 2000.0, false),
        target("N2", 51.10, -0.15, 200.0, true),
    ]));
    assert_eq!(traffic_picture().map(|p| p.len()), Some(2));
    // Two miles east of us, level, us heading north: three o'clock.
    let f = factor_from_picture(
        51.20,
        -0.20 - 2.0 / 37.6,
        2100.0,
        0.0,
        &traffic_picture().unwrap(),
    )
    .expect("the airborne one is a factor");
    assert_eq!(f.clock, 3, "{f:?}");
    assert!((f.nm - 2.0).abs() < 0.1, "{f:?}");
    assert_eq!(f.ft, -100.0, "{f:?}");
    st.install();
    assert!(traffic_picture().is_none(), "an install clears the picture");
}

/// A point `off_m` north of the twin strip (08L/26R), halfway along it.
fn by_the_twin(off_m: f64) -> AircraftState {
    let (lat, lon) = along(&book_runways()[1], 0.5);
    AircraftState::at_position(lat + off_m / 111_320.0, lon, true, 0.0)
}

/// A TRAFFIC CROSSING IS ASKED FOR AND JUDGED ON THE BOOK: through the same
/// keyed path as any call, held short while somebody stands on the strip,
/// cleared once it is free (the readback owed, the crossing on its strip
/// and its order), and spent once the flight has been on the strip and is
/// off it again. The world sweep's ULMM, EFKU, UEMH (2026-10-08): the
/// crossing was given in the taxi clearance and flown at once.
#[test]
fn a_traffic_crossing_is_asked_for_and_judged_on_the_book() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let ask = "Gatwick Tower, Cessna three four five, holding short runway two six right, \
               request crossing";
    let short = by_the_twin(-80.0);
    note_traffic_state(12, short);
    let held = st.answer_traffic(&call(12, ask, short, vec![ghost_on_the_twin()]));
    assert_eq!(
        held.line,
        format!("{CS}, hold short runway two six right, traffic on the runway.")
    );
    let order = || traffic_order(12).expect("its strip");
    assert_eq!(order().crossing, None, "nothing cleared");
    let cleared = st.answer_traffic(&call(12, ask, short, Vec::new()));
    assert_eq!(cleared.line, format!("{CS}, cross runway two six right."));
    assert_eq!(order().crossing.as_deref(), Some("26R"));
    assert_eq!(
        order().readback_owed.as_deref(),
        Some("Crossing runway two six right, Cessna three four five.")
    );
    let rb = st.answer_traffic(&call(
        12,
        "Crossing runway two six right, Cessna three four five",
        short,
        Vec::new(),
    ));
    assert!(rb.line.is_empty(), "the readback is accepted: {}", rb.line);
    assert_eq!(order().readback_owed, None);
    // On the strip, then off the far side: the crossing is spent. On the
    // pavement under its crossing the compliance watch is silent, at the
    // edge too (EFKU 15: a corner cut there read as an incursion).
    for (off, still) in [(-80.0, true), (-30.0, true), (0.0, true), (80.0, false)] {
        let moving = AircraftState {
            gs_kts: 8.0,
            ..by_the_twin(off)
        };
        note_traffic_state(12, moving);
        assert_eq!(
            traffic_compliance_oracle(12, Instant::now()),
            None,
            "{off} m"
        );
        traffic_tick(12, &[], Instant::now());
        assert_eq!(order().crossing.is_some(), still, "{off} m");
    }
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

/// A CLEARED CROSSING HOLDS THE STRIP, WHOLE, FROM THE CLEARANCE: the
/// flight is still short of the pavement, and no takeoff is cleared or
/// released onto the strip until it is seen off the far side. The world
/// sweep's ULMM (2026-10-08, a field's three hours): two crossings of 13
/// were cleared, and 6 s later the takeoff hold released a departure on 13
/// ("traffic clear"), because nobody was on the pavement YET.
#[test]
fn a_cleared_crossing_holds_the_strip_against_a_takeoff() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let by_26l = |off_m: f64| {
        let (lat, lon) = along(&book_runways()[0], 0.5);
        AircraftState::at_position(lat + off_m / 111_320.0, lon, true, 0.0)
    };
    let hold = at_the_hold();
    note_traffic_state(13, hold);
    st.answer_traffic(&call(
        13,
        "Gatwick Tower, Cessna three four five, request taxi",
        hold,
        Vec::new(),
    ));
    // The crossing is cleared first, 80 m short of 26L's pavement.
    note_traffic_state(12, by_26l(-80.0));
    let c = st.answer_traffic(&call(
        12,
        "Gatwick Tower, Cessna three four five, holding short runway two six left, \
         request crossing",
        by_26l(-80.0),
        Vec::new(),
    ));
    assert_eq!(c.line, format!("{CS}, cross runway two six left."));
    assert!(
        strip_held_by_traffic(&book_runways(), "26L"),
        "held for the crossing, nobody on the pavement yet"
    );
    // Ready for departure: held, never cleared onto the crossing.
    let ready = st.answer_traffic(&call(
        13,
        "Gatwick Tower, Cessna three four five, ready for departure",
        hold,
        Vec::new(),
    ));
    assert!(
        !ready.line.contains("cleared for takeoff"),
        "{}",
        ready.line
    );
    assert!(traffic_order(13).unwrap().hold, "its release is held");
    let t0 = Instant::now();
    let at = |s: u64| t0 + std::time::Duration::from_secs(s);
    for s in [0, 20, 40] {
        let lines = traffic_tick(13, &[], at(s));
        assert!(
            !lines.iter().any(|l| l.contains("cleared for takeoff")),
            "released over a cleared crossing at {s} s: {lines:?}"
        );
        assert!(
            runway_conflicts(None).is_empty(),
            "{:?}",
            runway_conflicts(None)
        );
    }
    // Across and off the far side: the strip is free, the departure goes.
    for off in [0.0, 80.0] {
        note_traffic_state(12, by_26l(off));
        traffic_tick(12, &[], at(45));
    }
    assert!(!strip_held_by_traffic(&book_runways(), "26L"));
    let mut released = Vec::new();
    for s in [50, 70, 90] {
        released.extend(traffic_tick(13, &[], at(s)));
    }
    assert!(
        released.iter().any(|l| l.contains("cleared for takeoff")),
        "{released:?}"
    );
    assert!(own_strip_moved().is_empty(), "{:?}", own_strip_moved());
    Station::uninstall();
}

/// The book's audit names a takeoff over a cleared crossing (and the
/// crossing's own claim reads by the rules ours would: the whole strip).
#[test]
fn the_book_audits_a_takeoff_over_a_cleared_crossing() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let st = gatwick_live();
    let crossing = open_traffic_strip(20);
    *crossing.crossing.lock().unwrap() = Some(("26L".into(), false));
    assert!(strip_held_by_traffic(&book_runways(), "08R"), "either end");
    assert!(
        !strip_held_by_traffic(&book_runways(), "26R"),
        "not the twin"
    );
    assert!(
        runway_conflicts(None).is_empty(),
        "a crossing alone is clean"
    );
    let departure = open_traffic_strip(21);
    *departure.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TakeoffCleared,
        callsign: "N21".into(),
        runway_ident: "26L".into(),
        pending_readback: None,
        holds_runway: true,
    });
    let found = runway_conflicts(None);
    assert!(
        found
            .iter()
            .any(|c| c.contains("CrossingCleared") && c.contains("Takeoff")),
        "{found:?}"
    );
    drop(st);
    Station::uninstall();
}

/// LEMG, 2026-10-08 (bench/traffic/lemg-quick.yaml gives a callsign and no
/// `spoken`): our own traffic keyed its RAW flight id and the tower
/// answered "RYR4521, taxi to holding point ...". The tower addresses a
/// flight by its telephony, whatever the call carried.
#[test]
fn a_raw_traffic_callsign_is_answered_by_its_telephony() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    let r = st.answer_traffic(&TrafficCall {
        id: 4,
        spoken: "RYR4521".into(),
        text: "Gatwick Tower, RYR4521, request taxi".into(),
        state: at_the_hold(),
        picture: Vec::new(),
        approach: false,
    });
    assert!(
        r.line.starts_with("Ryanair four five two one, "),
        "{}",
        r.line
    );
    assert!(!r.line.contains("RYR"), "{}", r.line);
    Station::uninstall();
}

/// An instrument arrival 9 nm out on 26L's final, 3,000 ft.
fn on_a_long_final() -> AircraftState {
    let r = &book_runways()[0];
    let (lat, lon) = crate::nav::project_latlon(r.lat_b, r.lon_b, r.heading_a_deg(), 9.0);
    AircraftState::at_position(lat, lon, false, 3000.0)
}

/// THE ARRIVAL'S FREQUENCIES (CAP 413 ch. 6; the realism review,
/// 2026-10-10: airliners called the tower ten miles out and were told
/// "join final"): an instrument arrival checks in with APPROACH and is
/// cleared for the approach, reports established and is handed to the
/// tower, then calls the tower established and is cleared to land. Each
/// answered in its own unit's voice.
#[test]
fn an_instrument_arrival_works_approach_then_the_tower() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick_live();
    st.approach_khz = Some(126_825);
    let cs = "Speedbird one two three";
    let say = |st: &mut Station, text: &str, approach: bool| {
        st.answer_traffic(&TrafficCall {
            id: 7,
            spoken: cs.into(),
            text: text.into(),
            state: on_a_long_final(),
            picture: Vec::new(),
            approach,
        })
    };
    let r = say(
        &mut st,
        "Gatwick Approach, Speedbird one two three, descending altitude nine thousand feet, inbound",
        true,
    );
    assert_eq!(r.speaker, StationSpeaker::Approach);
    assert!(
        r.line
            .starts_with("Speedbird one two three, Gatwick Approach, descend altitude ")
            && r.line
                .contains("approach runway two six left, report established."),
        "{}",
        r.line
    );
    let r = say(
        &mut st,
        "Gatwick Approach, Speedbird one two three, established ILS runway two six left",
        true,
    );
    assert_eq!(r.speaker, StationSpeaker::Approach);
    assert!(r.line.contains("contact Gatwick Tower on"), "{}", r.line);
    let r = say(
        &mut st,
        "Gatwick Tower, Speedbird one two three, established ILS runway two six left, nine miles",
        false,
    );
    assert_eq!(r.speaker, StationSpeaker::Tower);
    assert!(r.line.contains("cleared to land"), "{}", r.line);
    // Off a missed approach: back to approach, vectored for another.
    let r = say(
        &mut st,
        "Gatwick Approach, Speedbird one two three, missed approach, climbing altitude three thousand feet",
        true,
    );
    assert!(
        r.line.contains("expect vectors for a further") && !r.line.contains("contact"),
        "{}",
        r.line
    );
}
