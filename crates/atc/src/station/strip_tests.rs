//! Traffic strips: a traffic flight's claim is the book's, its answers write
//! its own strip, and an empty map changes nothing for ours.

use super::book_tests::{along, book_runways};
use super::tests::SIDE_STATE_LOCK;
use super::*;

/// Clears the traffic strips however the test ends.
pub(super) struct ClearStrips;

impl Drop for ClearStrips {
    fn drop(&mut self) {
        STRIPS.lock().unwrap_or_else(|e| e.into_inner()).clear();
    }
}

/// Gatwick's close pair, 26L in use.
pub(super) fn gatwick() -> Station {
    Station {
        apt: nav::AirportInfo {
            transition_level_ft: None,
            allows_circuits: None,
            pattern_sides: Vec::new(),
            name: "Gatwick".into(),
            elev_ft: 200.0,
            runways: book_runways(),
            freqs: vec![nav::AtcFreq {
                kind: nav::FREQ_TOWER,
                khz: 124225,
                name: "Tower".into(),
            }],
        },
        tower_khz: 124225,
        approach_khz: None,
        departure_khz: None,
        desk: StripDesk::new(),
        desks: std::collections::BTreeMap::new(),
        runway_in_use: Some("26L".into()),
        enforce_readback: true,
        dialect: Dialect::Icao,
        cap413: false,
        procedures: None,
        sid_exits: Default::default(),
        last_qnh: None,
        reply_at: None,
    }
}

fn westerly() -> Wx {
    Wx {
        wind_from_deg: 250.0,
        wind_kts: 8.0,
    }
}

fn ctx_for(st: &mut Station, callsign: &str, on_ground: bool) -> tower::Ctx {
    st.make_ctx(
        westerly(),
        "QNH one zero one three".into(),
        callsign.into(),
        on_ground,
        false,
        None,
        false,
        None,
    )
}

fn snap(state: tower::State, runway: &str, holds: bool) -> ClearanceSnap {
    ClearanceSnap {
        state,
        callsign: "Easy four five six".into(),
        runway_ident: runway.into(),
        pending_readback: None,
        holds_runway: holds,
    }
}

/// Everything the crew can read of our strip, as one comparable picture.
fn crew_picture() -> String {
    use crate::party_line as pl;
    format!(
        "{:?}",
        (
            pl::clearance_state(),
            pl::clearance_runway(),
            pl::readback_owed(),
            pl::taxi_to_stand(),
            pl::assigned_squawk(),
            pl::assigned_clearance().map(|c| format!("{c:?}")),
            pl::ifr_flight_code(),
            pl::takeoff_hold_armed(),
            pl::landing_hold_armed(),
            pl::line_up_condition_clear(),
            pl::vectored_callsign(),
            pl::landing_handoff_runway(),
        )
    )
}

/// A TRAFFIC STRIP CLEARED TO LAND BLOCKS OUR LINE-UP: the book reads its
/// clearance, no ghost on the pavement needed, and the release we are held
/// with waits for it the same way. A traffic strip on the twin, or only
/// sequenced for ours, blocks nothing.
#[test]
fn a_traffic_strip_cleared_to_land_blocks_our_line_up() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let ready = "Gatwick Tower, Speedbird one two three, ready for departure";

    // No traffic strip: cleared for takeoff.
    let mut st = gatwick();
    st.install();
    st.desk.twr.state = tower::State::TaxiCleared;
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    assert!(!ctx.runway_occupied);
    let r = st.route_keyed(ready, Service::Tower, &ctx, westerly());
    assert!(r.contains("cleared for takeoff"), "{r}");

    // Traffic on the twin, and traffic only sequenced for ours: still clear.
    let mut st = gatwick();
    st.install();
    let twin = open_traffic_strip(7);
    *twin.clearance.lock().unwrap() = Some(snap(tower::State::ClearedToLand, "26R", true));
    let inbound = open_traffic_strip(8);
    *inbound.clearance.lock().unwrap() = Some(snap(tower::State::Inbound, "26L", false));
    st.desk.twr.state = tower::State::TaxiCleared;
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    assert!(
        !ctx.runway_occupied,
        "the twin and a sequenced arrival hold nothing whole"
    );

    // Traffic cleared to land on 26L: hold position, and the release waits.
    *inbound.clearance.lock().unwrap() = Some(snap(tower::State::ClearedToLand, "26L", true));
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    assert!(ctx.runway_occupied, "the book holds 26L for the traffic");
    let r = st.route_keyed(ready, Service::Tower, &ctx, westerly());
    assert!(r.contains("hold position, traffic on the runway"), "{r}");
    assert!(
        crate::party_line::takeoff_hold_armed(),
        "the release watch armed"
    );
    let t0 = Instant::now();
    for s in [0, 20, 40] {
        let at = t0 + std::time::Duration::from_secs(s);
        assert!(
            takeoff_hold_tick(&[], 200.0, true, at).is_none(),
            "+{s}: nothing on the pavement, but the book holds the runway"
        );
    }
    // The traffic reports vacated: the runway is free in the book.
    *inbound.clearance.lock().unwrap() = Some(snap(tower::State::ClearedToLand, "26L", false));
    let mut released = None;
    for s in [60, 80] {
        released = released.or(takeoff_hold_tick(
            &[],
            200.0,
            true,
            t0 + std::time::Duration::from_secs(s),
        ));
    }
    let line = released.expect("released once the book frees the runway");
    assert!(line.contains("cleared for takeoff"), "{line}");
    Station::uninstall();
}

/// A TRAFFIC AIRCRAFT STANDING ON OUR RUNWAY BLOCKS OUR LINE-UP: its
/// producer's report puts it on the pavement in the book, our ready call is
/// held, and the release waits until it has left the strip. One standing on
/// the twin, or flying over ours, blocks nothing.
#[test]
fn a_traffic_aircraft_on_our_runway_blocks_our_line_up() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let ready = "Gatwick Tower, Speedbird one two three, ready for departure";
    let runways = book_runways();
    let (lat, lon) = along(&runways[0], 0.5);

    let mut st = gatwick();
    st.install();
    note_traffic_state(4, AircraftState::at_position(lat, lon, true, 0.0));
    st.desk.twr.state = tower::State::TaxiCleared;
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    assert!(ctx.runway_occupied, "the traffic stands on 26L");
    let r = st.route_keyed(ready, Service::Tower, &ctx, westerly());
    assert!(r.contains("hold position, traffic on the runway"), "{r}");
    assert!(crate::party_line::takeoff_hold_armed());
    let t0 = Instant::now();
    for s in [0, 20, 40] {
        let at = t0 + std::time::Duration::from_secs(s);
        assert!(
            takeoff_hold_tick(&[], 200.0, true, at).is_none(),
            "+{s}: the traffic is still on the strip"
        );
    }
    // It vacates onto the taxiway, 300 m off the centreline.
    note_traffic_state(
        4,
        AircraftState::at_position(lat - 300.0 / 111_320.0, lon, true, 10.0),
    );
    let mut released = None;
    for s in [60, 80] {
        released = released.or(takeoff_hold_tick(
            &[],
            200.0,
            true,
            t0 + std::time::Duration::from_secs(s),
        ));
    }
    let line = released.expect("released once the traffic has left the strip");
    assert!(line.contains("cleared for takeoff"), "{line}");
    Station::uninstall();

    for (what, (lat, lon), on_ground) in [
        ("on the twin", along(&runways[1], 0.5), true),
        ("over our runway", (lat, lon), false),
    ] {
        let mut st = gatwick();
        st.install();
        note_traffic_state(4, AircraftState::at_position(lat, lon, on_ground, 0.0));
        st.desk.twr.state = tower::State::TaxiCleared;
        let ctx = ctx_for(&mut st, "Speedbird one two three", true);
        assert!(!ctx.runway_occupied, "traffic {what}");
        let r = st.route_keyed(ready, Service::Tower, &ctx, westerly());
        assert!(r.contains("cleared for takeoff"), "traffic {what}: {r}");
        Station::uninstall();
    }
}

/// A TRAFFIC STRIP'S ANSWERS ARE ITS OWN: its final call and its readback
/// go through its own desk and write its own strip; not one thing the crew
/// reads of ours moves, and our desk is back on frequency after each call.
#[test]
fn a_traffic_readback_moves_no_party_line_accessor() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let mut st = gatwick();
    st.install();
    let before = crew_picture();
    let own_state = st.desk.twr.state;
    let traffic = open_traffic_strip(3);
    let id = FlightId::Traffic(3);

    let ctx = ctx_for(&mut st, "Easy four five six", false);
    let r = st
        .respond_for(
            id,
            "Gatwick Tower, Easy four five six, final runway two six left",
            Service::Tower,
            &ctx,
            westerly(),
        )
        .expect("an open strip answers");
    assert!(r.contains("cleared to land"), "{r}");
    assert!(!st.desk.strip.is_traffic(), "our desk is back on frequency");
    assert_eq!(st.desk.twr.state, own_state);
    {
        let c = traffic.clearance.lock().unwrap();
        let c = c.as_ref().expect("the traffic's clearance is on its strip");
        assert_eq!(c.state, tower::State::ClearedToLand);
        assert!(c.holds_runway);
        assert!(c.pending_readback.is_some(), "its readback is owed");
    }
    assert_eq!(
        crew_picture(),
        before,
        "the final call moved nothing of ours"
    );

    let r = st
        .respond_for(
            id,
            "Cleared to land runway two six left, Easy four five six",
            Service::Tower,
            &ctx,
            westerly(),
        )
        .expect("an open strip answers");
    assert!(r.is_empty(), "a correct readback is silence: {r}");
    assert!(
        traffic
            .clearance
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .pending_readback
            .is_none(),
        "the readback is paid on the traffic's strip"
    );
    assert_eq!(crew_picture(), before, "the readback moved nothing of ours");
    assert!(!crate::party_line::take_pending_takeoff_clearance());
    assert!(!crate::party_line::take_pending_line_up());
    assert!(!crate::party_line::take_pending_go_around());
    assert!(crate::party_line::take_departure_handoff().is_none());
    assert!(!st.desk.strip.is_traffic());
    assert!(st.desks.contains_key(&id), "the traffic's desk is parked");
    // Its claim is the book's: our departure would wait for it.
    assert!(strip_held_by_traffic(&st.apt.runways, "08R"));
    // A number with no strip open is not answered.
    assert!(st
        .respond_for(
            FlightId::Traffic(99),
            "Gatwick Tower, final",
            Service::Tower,
            &ctx,
            westerly()
        )
        .is_none());
    Station::uninstall();
}

/// AN EMPTY MAP IS IDENTICAL: with no traffic strip open the book is ours
/// alone, nothing holds a runway for traffic, and our flight's answers
/// through `respond_for` are word for word those of `route_keyed`.
#[test]
fn an_empty_traffic_map_changes_nothing() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    STRIPS.lock().unwrap().clear();
    let mut a = gatwick();
    let mut b = gatwick();
    a.install();
    for (state, holds) in [
        (tower::State::ClearedToLand, true),
        (tower::State::LinedUp, true),
        (tower::State::Inbound, false),
    ] {
        *OWN_STRIP.clearance.lock().unwrap() = Some(snap(state, "26L", holds));
        let book = RunwayBook::now(None);
        let ours_only = RunwayBook::compute(&[BookEntry {
            id: FlightId::Own,
            strip: &OWN_STRIP,
            state: None,
        }]);
        assert_eq!(book.claims(), ours_only.claims(), "{state:?}");
        for end in ["08R", "26L", "08L", "26R"] {
            assert!(!strip_held_by_traffic(&a.apt.runways, end));
            assert!(book.held_by_another(FlightId::Own, end).is_none());
        }
    }
    *OWN_STRIP.clearance.lock().unwrap() = None;
    for (call, on_ground) in [
        ("Gatwick Tower, Speedbird one two three, request taxi", true),
        ("Taxi to holding point Alpha, Speedbird one two three", true),
        (
            "Gatwick Tower, Speedbird one two three, ready for departure",
            true,
        ),
        (
            "Cleared for takeoff runway two six left, Speedbird one two three",
            true,
        ),
    ] {
        let ctx_a = ctx_for(&mut a, "Speedbird one two three", on_ground);
        let ctx_b = ctx_for(&mut b, "Speedbird one two three", on_ground);
        let ra = a.respond_for(FlightId::Own, call, Service::Tower, &ctx_a, westerly());
        let rb = b.route_keyed(call, Service::Tower, &ctx_b, westerly());
        assert_eq!(ra.as_deref(), Some(rb.as_str()), "{call}");
        assert_eq!(a.desk.twr.state, b.desk.twr.state, "{call}");
    }
    assert!(a.desks.is_empty(), "our flight never parks a desk");
    Station::uninstall();
}

/// The traffic strips are the FIELD's: a new station and the end of ATC
/// clear them, a new flight of ours never does.
#[test]
fn traffic_strips_die_with_the_station_not_with_our_flight() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let st = gatwick();
    open_traffic_strip(5);
    retire_for_new_flight();
    assert!(
        traffic_strip(5).is_some(),
        "our new flight is not the traffic's"
    );
    st.install();
    assert!(
        traffic_strips().is_empty(),
        "a new station is a new field's traffic"
    );
    open_traffic_strip(5);
    Station::uninstall();
    assert!(traffic_strips().is_empty(), "the end of ATC");
}
