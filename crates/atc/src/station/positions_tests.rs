//! The aerodrome's positions (2026-10-10): Delivery, Ground and the Tower
//! as views of one station, each answering its own calls, sending the
//! rest on, and handing the flight between them.

use super::book_tests::{along, book_runways};
use super::strip_tests::{gatwick, ClearStrips};
use super::tests::SIDE_STATE_LOCK;
use super::*;

const CS: &str = "Speedbird one two three";

fn wx() -> Wx {
    Wx {
        wind_from_deg: 250.0,
        wind_kts: 8.0,
    }
}

/// EGKK's comm rows as the install's apt.dat has them: Delivery, Ground
/// and the Tower each on a frequency of its own.
fn egkk_positions_file() -> std::path::PathBuf {
    use std::io::Write;
    use std::sync::OnceLock;
    static PATH: OnceLock<std::path::PathBuf> = OnceLock::new();
    PATH.get_or_init(|| {
        let path = std::env::temp_dir()
            .join(format!("xc-station-egkk-positions-{}.dat", std::process::id()));
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "1    203 0 0 EGKK London Gatwick").unwrap();
        writeln!(f, "100 46.00 26 1527 0.25 1 3 0 08R 51.1450993 -0.2123748 395 0 7 4 1 3 26L 51.1514994 -0.1659296 425 30 7 4 1 3").unwrap();
        writeln!(f, "1052 121955 Gatwick Delivery").unwrap();
        writeln!(f, "1053 121805 Gatwick Ground").unwrap();
        writeln!(f, "1054 124230 Gatwick Tower").unwrap();
        writeln!(f, "1055 126825 Gatwick Director").unwrap();
        writeln!(f, "1   1027 0 0 0MU1 Sunderland").unwrap();
        f.flush().unwrap();
        path
    })
    .clone()
}

fn egkk() -> Station {
    Station::uninstall();
    let st = Station::load(egkk_positions_file().to_str().unwrap(), "EGKK").expect("EGKK loads");
    st.install();
    st
}

fn ctx(st: &mut Station, on_ground: bool) -> tower::Ctx {
    st.make_ctx(
        wx(),
        "QNH one zero one three".into(),
        CS.into(),
        on_ground,
        false,
        None,
        false,
        None,
    )
}

/// EACH POSITION ANSWERS ITS OWN CALLS, AND A CALL ON THE WRONG ONE IS
/// SENT ON: the clearance is Delivery's (ending "when ready, contact
/// Gatwick Ground"), the push and the taxi Ground's, the takeoff the
/// Tower's. The Tower hearing a push request says "contact Gatwick Ground
/// on one two one decimal eight zero five" and clears nothing; Ground
/// hearing the ready call at the hold says "contact Gatwick Tower". A
/// radio check is answered where it is heard, in that position's name.
#[test]
fn each_position_answers_its_own_calls_and_sends_the_rest_on() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut st = egkk();
    let c = ctx(&mut st, true);
    let w = wx();

    let r = st.route_keyed("request clearance", Service::Tower, &c, w);
    assert_eq!(
        r,
        format!("{CS}, contact Gatwick Delivery on one two one decimal niner five five.")
    );
    assert_eq!(
        crate::party_line::take_position_handoff(),
        Some(("Gatwick Delivery".into(), 121955)),
        "the crew is told where to call"
    );
    assert!(
        st.desk.assigned_squawk.is_none(),
        "nothing cleared on the wrong frequency"
    );

    let r = st.route_keyed("request clearance", Service::Delivery, &c, w);
    assert!(r.contains("cleared to leave the Gatwick zone"), "{r}");
    assert!(
        r.ends_with("When ready, contact Gatwick Ground on one two one decimal eight zero five."),
        "{r}"
    );
    // The clearance's readback is Delivery's to hear.
    let r = st.route_keyed("roger, squawk four five zero one", Service::Delivery, &c, w);
    assert_eq!(r, "", "read back");

    let r = st.route_keyed("radio check", Service::Ground, &c, w);
    assert_eq!(r, format!("{CS}, Gatwick Ground, reading you five."));

    let r = st.route_keyed("request push and start", Service::Tower, &c, w);
    assert_eq!(
        r,
        format!("{CS}, contact Gatwick Ground on one two one decimal eight zero five.")
    );
    assert!(
        crate::party_line::push_approved().is_none(),
        "the Tower approved nothing"
    );
    let _ = crate::party_line::take_position_handoff();

    let r = st.route_keyed(
        "stand five two, request push and start",
        Service::Ground,
        &c,
        w,
    );
    assert_eq!(r, format!("{CS}, push and start approved."));
    assert_eq!(
        crate::party_line::push_approved(),
        Some(None),
        "approved, no lane to face by"
    );

    let r = st.route_keyed("request taxi", Service::Delivery, &c, w);
    assert!(r.contains("contact Gatwick Ground"), "{r}");
    let r = st.route_keyed("request taxi", Service::Ground, &c, w);
    assert!(
        r.contains("holding point") && r.contains("runway two six left"),
        "{r}"
    );
    assert_eq!(st.desk.twr.state, tower::State::TaxiCleared);

    let r = st.route_keyed("ready for departure", Service::Ground, &c, w);
    assert_eq!(
        r,
        format!("{CS}, contact Gatwick Tower on one two four decimal two three.")
    );
    assert_eq!(
        st.desk.twr.state,
        tower::State::TaxiCleared,
        "Ground clears no takeoff"
    );
    let r = st.route_keyed("ready for departure", Service::Tower, &c, w);
    assert!(r.contains("cleared for takeoff"), "{r}");

    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.handoff.lock().unwrap() = None;
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    Station::uninstall();
}

/// GROUND HANDS THE DEPARTURE TO THE TOWER AT THE HOLD, ONCE: taxiing on
/// Ground's clearance, within 150 m of the holding point, "hold short
/// runway two six left, contact Gatwick Tower on one two four decimal two
/// three". The crew is handed the Tower; asked again, Ground says
/// nothing more (the flight works the Tower now).
#[test]
fn ground_hands_the_departure_to_the_tower_at_the_hold_once() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut st = egkk();
    let c = ctx(&mut st, true);
    let r = st.route_keyed("request taxi", Service::Ground, &c, wx());
    assert!(r.contains("holding point"), "{r}");
    let r26 = &st.apt.runways[0];
    let hold = (r26.lat_b - 0.0009, r26.lon_b + 0.0010);
    *OWN_STRIP.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(hold.0 - 0.01, hold.1), hold],
        via: String::new(),
        seg: Vec::new(),
        holding_point: "A1".into(),
        lead_on: Vec::new(),
    });
    let _ = crate::party_line::take_position_handoff();
    assert_eq!(
        ground_handoff_tick(hold.0 - 0.01, hold.1, true),
        None,
        "a kilometre out"
    );
    assert_eq!(ground_handoff_tick(hold.0, hold.1, false), None, "airborne");
    let line = ground_handoff_tick(hold.0 - 0.0005, hold.1, true).expect("at the hold");
    assert_eq!(
        line,
        format!(
            "{CS}, hold short runway two six left, contact Gatwick Tower on one two four decimal two three."
        )
    );
    assert_eq!(
        crate::party_line::take_position_handoff(),
        Some(("Gatwick Tower".into(), 124230))
    );
    assert_eq!(ground_handoff_tick(hold.0, hold.1, true), None, "once");
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    Station::uninstall();
}

/// THE TOWER SENDS A VACATED ARRIVAL TO GROUND, and Ground taxis it in:
/// "runway vacated" to the Tower is answered "contact Gatwick Ground on
/// ...", the taxi to parking is Ground's, and the Tower hearing it sends
/// it on.
#[test]
fn the_tower_sends_a_vacated_arrival_to_ground() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut st = egkk();
    let air = ctx(&mut st, false);
    let r = st.route_keyed("inbound for landing", Service::Tower, &air, wx());
    assert!(r.contains("join final"), "{r}");
    let r = st.route_keyed("final runway two six left", Service::Tower, &air, wx());
    assert!(r.contains("cleared to land"), "{r}");
    let _ = st.route_keyed(
        "cleared to land runway two six left",
        Service::Tower,
        &air,
        wx(),
    );
    let gnd = ctx(&mut st, true);
    let r = st.route_keyed("runway vacated", Service::Tower, &gnd, wx());
    assert_eq!(
        r,
        format!("{CS}, contact Gatwick Ground on one two one decimal eight zero five.")
    );
    assert_eq!(
        crate::party_line::take_position_handoff(),
        Some(("Gatwick Ground".into(), 121805))
    );
    let r = st.route_keyed("request taxi to parking", Service::Tower, &gnd, wx());
    assert!(r.contains("contact Gatwick Ground"), "{r}");
    let r = st.route_keyed("request taxi to parking", Service::Ground, &gnd, wx());
    assert!(r.contains("taxi to"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::Parked);
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.landing_handoff.lock().unwrap() = None;
    Station::uninstall();
}

/// A FIELD WITH NO GROUND IS THE TOWER'S, AS BEFORE: Gibraltar's rows are
/// the tower and the radar units, and the tower clears, taxis and
/// launches on its own frequency without sending anybody anywhere.
#[test]
fn a_field_without_ground_is_worked_by_the_tower_as_before() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut st = gatwick();
    let c = ctx(&mut st, true);
    let r = st.route_keyed("request push and start", Service::Tower, &c, wx());
    assert!(r.contains("push and start approved"), "{r}");
    let r = st.route_keyed("request taxi", Service::Tower, &c, wx());
    assert!(r.contains("holding point"), "{r}");
    assert!(!r.contains("contact"), "{r}");
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.push_approved.lock().unwrap() = None;
    Station::uninstall();
}

/// THE PUSH IN EACH DIALECT, with the way the aircraft will face: ICAO
/// "push and start approved, facing west"; CAP 413 "push and start
/// approved, tail east"; FAA "push back approved, tail east" (FAA ground
/// approves no start); a bare "request push back" is a push alone.
#[test]
fn the_push_is_phrased_in_each_dialect() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut st = gatwick();
    let mut c = ctx(&mut st, true);
    let push = |st: &mut Station, c: &tower::Ctx, start: bool| {
        st.desk.call.push_facing = Some(271.0);
        st.desk.call.push_start = start;
        st.push_line(c)
    };
    c.uk = false;
    c.dialect = Dialect::Icao;
    assert_eq!(
        push(&mut st, &c, true),
        format!("{CS}, push and start approved, facing west.")
    );
    assert_eq!(
        push(&mut st, &c, false),
        format!("{CS}, push back approved, facing west.")
    );
    c.uk = true;
    assert_eq!(
        push(&mut st, &c, true),
        format!("{CS}, push and start approved, tail east.")
    );
    c.uk = false;
    c.dialect = Dialect::Faa;
    assert_eq!(
        push(&mut st, &c, true),
        format!("{CS}, push back approved, tail east.")
    );
    assert_eq!(*OWN_STRIP.push_approved.lock().unwrap(), Some(Some(271.0)));
    *OWN_STRIP.push_approved.lock().unwrap() = None;
    assert_eq!(compass_word(0.0), "north");
    assert_eq!(compass_word(44.0), "north east");
    assert_eq!(compass_word(181.0), "south");
    assert_eq!(compass_word(337.6), "north");
    assert_eq!(
        tower::parse_intent(
            "Gatwick Ground, Speedbird one two three, stand five two, request push and start"
        ),
        tower::Intent::RequestPushback
    );
    assert_eq!(
        tower::parse_intent("ready for push"),
        tower::Intent::RequestPushback
    );
    assert_eq!(
        tower::parse_intent("pushing back, tail east"),
        tower::Intent::Unknown
    );
    assert!(tower::is_readback(
        "pushing back, tail east, Speedbird one two three"
    ));
    assert!(tower::is_readback(
        "Gatwick Ground on one two one decimal eight zero five"
    ));
}

/// OUR TRAFFIC CALLS THE RIGHT POSITION, AND IS HANDED BETWEEN THEM: its
/// taxi request is heard by Ground (re-addressed "Gatwick Ground, ..."),
/// Ground hands it to the Tower at the hold (its readback then goes to the
/// Tower), its ready call is the Tower's.
#[test]
fn traffic_calls_are_heard_by_the_position_that_owns_them() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    Station::uninstall();
    let mut st = gatwick();
    st.ground_khz = Some(121805);
    st.apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_GROUND,
        khz: 121805,
        name: "Gatwick Ground".into(),
    });
    st.install();
    *TOWER_WIND.lock().unwrap() = Some((250.0, 2.0));
    let r = &book_runways()[0];
    let (lat, lon) = along(r, 0.95);
    let apron = AircraftState::at_position(lat - 400.0 / 111_320.0, lon, true, 0.0);
    let call = |text: &str, state: AircraftState| TrafficCall {
        id: 4,
        spoken: "Cessna three four five".into(),
        text: text.into(),
        state,
        picture: Vec::new(),
        approach: false,
    };
    let reply = st.answer_traffic(&call(
        "Gatwick Tower, Cessna three four five, request taxi",
        apron.clone(),
    ));
    assert_eq!(reply.speaker, StationSpeaker::Ground);
    assert_eq!(
        reply.call,
        "Gatwick Ground, Cessna three four five, request taxi"
    );
    assert!(reply.line.contains("holding point"), "{}", reply.line);
    assert_eq!(traffic_working(4), Some(StationSpeaker::Ground));
    // At the hold, Ground hands it on.
    let strip = traffic_strip(4).expect("open");
    let hold = (lat - 150.0 / 111_320.0, lon);
    *strip.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(hold.0 - 0.01, hold.1), hold],
        via: String::new(),
        seg: Vec::new(),
        holding_point: "A1".into(),
        lead_on: Vec::new(),
    });
    note_traffic_state(4, AircraftState::at_position(hold.0, hold.1, true, 0.0));
    let (line, who) = traffic_position_tick(4).expect("the handoff");
    assert_eq!(who, StationSpeaker::Ground);
    assert!(
        line.contains("contact Gatwick Tower on one two four decimal two two five"),
        "{line}"
    );
    assert_eq!(traffic_working(4), Some(StationSpeaker::Tower));
    assert!(traffic_position_tick(4).is_none(), "once");
    let reply = st.answer_traffic(&call(
        "Gatwick Tower, Cessna three four five, ready for departure",
        AircraftState::at_position(hold.0, hold.1, true, 0.0),
    ));
    assert_eq!(reply.speaker, StationSpeaker::Tower);
    Station::uninstall();
}

/// THE COACH ADDRESSES THE POSITION THE CALL IS FOR, and has it tuned
/// first: the first call from the ramp to Gatwick Delivery, the taxi to
/// Gatwick Ground, the ready call to the Tower.
#[test]
fn the_coach_sends_each_call_to_its_position() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut st = egkk();
    let tower_khz = Some(124_230);
    let s = suggest_call(CS, true, None, tower_khz).expect("a call");
    assert_eq!(
        s.call.as_deref(),
        Some("Gatwick Delivery, Speedbird one two three, request departure clearance.")
    );
    assert_eq!(
        s.first.as_deref(),
        Some("First tune clearance, one two one decimal niner five five.")
    );
    let c = ctx(&mut st, true);
    let _ = st.route_keyed("request clearance", Service::Delivery, &c, wx());
    let _ = st.route_keyed(
        "roger, squawk four five zero one",
        Service::Delivery,
        &c,
        wx(),
    );
    let s = suggest_call(CS, true, None, tower_khz).expect("a call");
    assert_eq!(
        s.call.as_deref(),
        Some("Speedbird one two three, request taxi.")
    );
    assert_eq!(
        s.first.as_deref(),
        Some("First tune ground, one two one decimal eight zero five.")
    );
    let _ = st.route_keyed("request taxi", Service::Ground, &c, wx());
    let s = suggest_call(CS, true, None, tower_khz).expect("a call");
    assert!(s
        .call
        .as_deref()
        .is_some_and(|c| c.ends_with("ready for departure.")));
    assert_eq!(s.first, None, "on the tower already");
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    Station::uninstall();
}
