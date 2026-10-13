//! Traffic strips: a traffic flight's claim is the book's, its answers write
//! its own strip, and an empty map changes nothing for ours.

use super::book_tests::{along, book_runways};
use super::ctx::landing_sequence_tail;
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
            papis: Vec::new(),
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
        ground_khz: None,
        delivery_khz: None,
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

/// A TRAFFIC LANDER STILL FLYING IS AN ARRIVAL, NEVER AN OCCUPANT: judged
/// by its distance like any arrival on final. Far out, our departure is
/// cleared; at two miles it is held for the LANDING traffic and the release
/// waits for the gap; once down on the pavement it is traffic on the runway.
/// LEMG 12 (2026-10-13): Air Nostrum cleared to land well out held our 737
/// at HW1 "hold position, traffic on the runway" with nothing on it.
#[test]
fn a_traffic_lander_still_flying_is_an_arrival() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let ready = "Gatwick Tower, Speedbird one two three, ready for departure";
    let r = book_runways()[0].clone();
    let on_final = |nm: f64| {
        let (lat, lon) = crate::nav::project_latlon(r.lat_b, r.lon_b, r.heading_a_deg(), nm);
        AircraftState::at_position(lat, lon, false, 140.0)
    };
    let mut st = gatwick();
    st.install();
    let lander = open_traffic_strip(8);
    *lander.clearance.lock().unwrap() = Some(snap(tower::State::ClearedToLand, "26L", true));

    // Eight miles out: no traffic for the departure at all.
    note_traffic_state(8, on_final(8.0));
    st.desk.twr.state = tower::State::TaxiCleared;
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    assert!(
        !ctx.runway_occupied,
        "a lander eight miles out occupies nothing"
    );
    assert_eq!(ctx.final_traffic_nm, None);
    let line = st.route_keyed(ready, Service::Tower, &ctx, westerly());
    assert!(line.contains("cleared for takeoff"), "{line}");
    // Two miles out: held for the landing traffic, by the gap.
    st.desk = StripDesk::new();
    st.desk.twr.state = tower::State::TaxiCleared;
    note_traffic_state(8, on_final(2.0));
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    assert!(!ctx.runway_occupied, "flying, it is an arrival");
    assert!(
        ctx.final_traffic_nm
            .is_some_and(|nm| (nm - 2.0).abs() < 0.1),
        "{:?}",
        ctx.final_traffic_nm
    );
    let line = st.route_keyed(ready, Service::Tower, &ctx, westerly());
    assert!(
        line.contains("behind the landing traffic on 2 mile final, line up"),
        "{line}"
    );
    assert!(!line.contains("traffic on the runway"), "{line}");
    assert!(crate::party_line::takeoff_hold_armed());
    let t0 = Instant::now();
    for s in [0, 20, 40] {
        let at = t0 + std::time::Duration::from_secs(s);
        assert!(
            takeoff_hold_tick(&[], 200.0, true, at).is_none(),
            "+{s}: the lander is inside the gap"
        );
        assert_eq!(
            crate::party_line::line_up_condition_clear(),
            Some(false),
            "+{s}: the lander has not passed"
        );
    }
    // Read back, then asked again lined up: held for the landing traffic.
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    let rb = st.route_keyed(
        "Behind the landing traffic, lining up runway two six left behind, Speedbird one two three",
        Service::Tower,
        &ctx,
        westerly(),
    );
    assert!(!rb.contains("read back"), "{rb}");
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    let line = st.route_keyed(ready, Service::Tower, &ctx, westerly());
    assert!(
        line.contains("hold position, traffic on 2 mile final")
            && line.contains("after the landing traffic"),
        "{line}"
    );
    // Down on the pavement: traffic on the runway, and still held.
    let (lat, lon) = along(&r, 0.7);
    note_traffic_state(8, AircraftState::at_position(lat, lon, true, 60.0));
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    assert!(ctx.runway_occupied, "landed, it occupies the strip");
    for s in [60, 80] {
        let at = t0 + std::time::Duration::from_secs(s);
        assert!(
            takeoff_hold_tick(&[], 200.0, true, at).is_none(),
            "+{s}: on the runway"
        );
    }
    // Reported vacated: released.
    *lander.clearance.lock().unwrap() = Some(snap(tower::State::ClearedToLand, "26L", false));
    note_traffic_state(
        8,
        AircraftState::at_position(lat - 300.0 / 111_320.0, lon, true, 10.0),
    );
    let mut released = None;
    for s in [100, 120] {
        released = released.or(takeoff_hold_tick(
            &[],
            200.0,
            true,
            t0 + std::time::Duration::from_secs(s),
        ));
    }
    let line = released.expect("released once the lander is off");
    assert!(line.contains("cleared for takeoff"), "{line}");
    Station::uninstall();
}

/// A held departure is RELEASED while a traffic lander is still outside the
/// gap ([`DEPART_GAP_NM`]): the landing clearance stands, judged on time by
/// the lander's go-around watch, as a real tower's anticipated separation.
#[test]
fn a_held_departure_is_released_ahead_of_a_distant_lander() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let ready = "Gatwick Tower, Speedbird one two three, ready for departure";
    let r = book_runways()[0].clone();
    let on_final = |nm: f64| {
        let (lat, lon) = crate::nav::project_latlon(r.lat_b, r.lon_b, r.heading_a_deg(), nm);
        AircraftState::at_position(lat, lon, false, 140.0)
    };
    let mut st = gatwick();
    st.install();
    let lander = open_traffic_strip(8);
    *lander.clearance.lock().unwrap() = Some(snap(tower::State::ClearedToLand, "26L", true));
    // Unknown position: an occupant, as before.
    st.desk.twr.state = tower::State::TaxiCleared;
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    assert!(
        ctx.runway_occupied,
        "a lander nobody can place holds the strip"
    );
    let line = st.route_keyed(ready, Service::Tower, &ctx, westerly());
    assert!(
        line.contains("hold position, traffic on the runway"),
        "{line}"
    );
    assert!(crate::party_line::takeoff_hold_armed());
    // Placed eight miles out: the release goes out once the strip has stayed
    // clear its while.
    note_traffic_state(8, on_final(8.0));
    let t0 = Instant::now();
    let mut released = None;
    for s in [0, 20] {
        released = released.or(takeoff_hold_tick(
            &[],
            200.0,
            true,
            t0 + std::time::Duration::from_secs(s),
        ));
    }
    let line = released.expect("released ahead of the distant lander");
    assert!(line.contains("cleared for takeoff"), "{line}");
    Station::uninstall();
}

/// OUR TAKEOFF CLEARANCE ENDS WITH OUR TAKEOFF: airborne two miles beyond
/// the far end and still `TakeoffCleared` (flown by hand, no next call), we
/// claim the strip no more; the traffic departure held behind us waits out
/// our liftoff's wake and is released. LEMG 12 (2026-10-13): held "traffic
/// on the runway" from 14:08Z, five minutes after our "contact Departure".
#[test]
fn our_takeoff_clearance_ends_with_our_takeoff() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let r = book_runways()[0].clone();
    let st = gatwick();
    st.install();
    *OWN_STRIP.clearance.lock().unwrap() = Some(snap(tower::State::TakeoffCleared, "26L", true));
    let t0 = Instant::now();
    OWN_STRIP.compliance_flight.lock().unwrap().liftoff = Some(("08R".into(), t0));
    // Departing 26L (west): two miles past the 08R end, climbing.
    let (lat, lon) = crate::nav::project_latlon(r.lat_a, r.lon_a, r.heading_b_deg(), 2.0);
    note_own_state(AircraftState::at_position(lat, lon, false, 180.0));
    assert!(
        RunwayBook::now(None).strip_claim(FlightId::Own).is_none(),
        "a takeoff clearance flown claims nothing"
    );
    // The traffic departure at the hold, its release armed as we lifted off.
    let (hlat, hlon) = along(&r, 0.95);
    let at_hold = AircraftState::at_position(hlat - 150.0 / 111_320.0, hlon, true, 0.0);
    note_traffic_state(5, at_hold);
    let t = traffic_strip(5).unwrap();
    assert!(!strip_held_by_others(&book_runways(), "26L", &t));
    *t.clearance.lock().unwrap() = Some(snap(tower::State::TaxiCleared, "26L", false));
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
    let tick = |s: u64| {
        takeoff_hold_tick_for(
            &t,
            &at_hold,
            &[],
            200.0,
            t0 + std::time::Duration::from_secs(s),
        )
    };
    let first = tick(30).expect("lined up behind our wake");
    assert!(
        first.contains("line up and wait") && first.contains("wake"),
        "{first}"
    );
    for s in [60, 100] {
        assert!(tick(s).is_none(), "+{s}: our wake");
    }
    // Our position as the traffic link keeps noting it.
    note_own_state(AircraftState::at_position(lat, lon, false, 180.0));
    let mut released = None;
    for s in [125, 145] {
        released = released.or(tick(s));
    }
    let line = released.expect("released after our wake");
    assert!(line.contains("cleared for takeoff"), "{line}");
    *OWN_STRIP.seen.lock().unwrap() = None;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
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

/// The landing sequence is every strip's place while its flight is still
/// Inbound for it (BIEG 03, 2026-10-08): counted, the last of them the
/// leader, the asking flight itself left out; a flight cleared to land is
/// out of it.
#[test]
fn the_landing_sequence_counts_the_inbound_places_and_names_the_last() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    *OWN_STRIP.sequence.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let place = |s: &FlightStrip, n: u8, state: tower::State| {
        *s.sequence.lock().unwrap() = Some((n, None));
        *s.clearance.lock().unwrap() = Some(ClearanceSnap {
            state,
            callsign: "x".into(),
            runway_ident: "03".into(),
            pending_readback: None,
            holds_runway: false,
        });
    };
    let me = open_traffic_strip(9);
    assert_eq!(landing_sequence_tail(&me), None, "nobody sequenced");
    place(&open_traffic_strip(1), 1, tower::State::Inbound);
    place(&open_traffic_strip(2), 2, tower::State::Inbound);
    place(&open_traffic_strip(3), 7, tower::State::ClearedToLand);
    place(&me, 4, tower::State::Inbound);
    assert_eq!(
        landing_sequence_tail(&me),
        Some((2, FlightId::Traffic(2))),
        "two inbound ahead, the cleared one and the asker left out"
    );
}

/// A RELEASE AHEAD OF A LANDER IS THE GO-AROUND WATCH'S OWN JUDGEMENT, IN
/// TIME: the departure's predicted exits fed through [`go_around_due`].
/// LEMG 13 (2026-10-13): RYR5024 cleared for takeoff at the holding point
/// with us cleared to land 4.02 nm out at 147 kt (99 s), just outside the
/// landing claim's 4 nm and so not in its book at all; it took 43 s to line
/// up, and we were sent round. From the holding point the lander must be
/// [`LINE_UP_S`] + the roll + [`RUNWAY_CLEAR_MARGIN_S`] out (107.5 s).
#[test]
fn a_release_ahead_of_a_lander_is_judged_as_the_go_around_watch_judges() {
    let roll = roll_to_liftoff_s(0.0);
    let trace = lander_secs(4.02, 146.7);
    assert!((trace - 98.7).abs() < 0.5, "{trace}");
    assert!(
        !departure_clears_lander(Some(trace), LINE_UP_S),
        "the LEMG 13 release from the holding point"
    );
    let need = LINE_UP_S + roll + RUNWAY_CLEAR_MARGIN_S;
    assert!(!departure_clears_lander(Some(need - 1.0), LINE_UP_S));
    assert!(departure_clears_lander(Some(need + 1.0), LINE_UP_S));
    assert!(
        !departure_clears_lander(None, LINE_UP_S),
        "an unknown time holds"
    );
    assert!(!departure_clears_lander(Some(0.0), LINED_UP_ROLL_S));
    // Lined up with a lander six miles out at 140 kt: released, and the
    // lander's watch on the predicted state never fires, second by second.
    let t = lander_secs(6.0, 140.0);
    assert!(departure_clears_lander(Some(t), LINED_UP_ROLL_S));
    let mut tau = 0.0;
    while tau < t {
        let exits: Vec<StripExit> = if tau < LINED_UP_ROLL_S {
            vec![StripExit::LinedUp(roll)]
        } else if tau < LINED_UP_ROLL_S + roll {
            vec![StripExit::Clearing(roll - (tau - LINED_UP_ROLL_S))]
        } else {
            Vec::new()
        };
        assert!(!go_around_due(Some(t - tau), &exits), "+{tau} s");
        tau += 0.5;
    }
    // Two miles out (51 s), even lined up: held.
    assert!(!departure_clears_lander(
        Some(lander_secs(2.0, 140.0)),
        LINED_UP_ROLL_S
    ));
    // A groundspeed that did not read is the fast side.
    assert_eq!(
        lander_secs(3.0, f64::NAN),
        lander_secs(3.0, DEFAULT_FINAL_KTS)
    );
}

/// OUR LANDING CLEARANCE, A TRAFFIC DEPARTURE AT THE HOLDING POINT: the
/// LEMG 13 case. Held with the landing traffic named, never cleared, never
/// "traffic on the runway"; its release watch holds it too.
#[test]
fn a_traffic_departure_at_the_hold_is_not_released_ahead_of_our_landing() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let r = book_runways()[0].clone();
    let mut st = gatwick();
    st.install();
    *OWN_STRIP.clearance.lock().unwrap() = Some(snap(tower::State::ClearedToLand, "26L", true));
    let (lat, lon) = crate::nav::project_latlon(r.lat_b, r.lon_b, r.heading_a_deg(), 4.02);
    note_own_state(AircraftState::at_position(lat, lon, false, 146.7));
    let (hlat, hlon) = along(&r, 0.95);
    let at_hold = AircraftState::at_position(hlat - 150.0 / 111_320.0, hlon, true, 0.0);
    let dep = open_traffic_strip(2);
    note_traffic_state(2, at_hold);
    let line = st
        .with_desk(FlightId::Traffic(2), |st| {
            st.desk.twr.state = tower::State::TaxiCleared;
            let ctx = ctx_for(st, "Ryanair five zero two four", true);
            assert!(!ctx.runway_occupied, "flying, we are an arrival");
            assert!(
                ctx.final_traffic_nm
                    .is_some_and(|nm| (nm - 4.0).abs() < 0.1),
                "{:?}",
                ctx.final_traffic_nm
            );
            st.route_keyed(
                "Gatwick Tower, Ryanair five zero two four, ready for departure",
                Service::Tower,
                &ctx,
                westerly(),
            )
        })
        .unwrap();
    assert!(!line.contains("cleared for takeoff"), "{line}");
    assert!(line.contains("landing traffic"), "{line}");
    assert!(!line.contains("traffic on the runway"), "{line}");
    // Its release watch, at the hold, with us 3 nm out: still held.
    *dep.takeoff_hold.lock().unwrap() = Some(TakeoffHold {
        callsign: "Ryanair five zero two four".into(),
        runway: "two six left".into(),
        runway_ident: "26L".into(),
        clear_since: None,
        armed_at: Instant::now(),
        logged_at: Instant::now(),
        lined_up: false,
        behind: false,
        held_for: None,
        cut_in_logged: Default::default(),
    });
    let (lat, lon) = crate::nav::project_latlon(r.lat_b, r.lon_b, r.heading_a_deg(), 3.0);
    note_own_state(AircraftState::at_position(lat, lon, false, 140.0));
    let t0 = Instant::now();
    for s in [0, 20, 40] {
        let at = t0 + std::time::Duration::from_secs(s);
        assert_eq!(
            takeoff_hold_tick_for(&dep, &at_hold, &[], 200.0, at),
            None,
            "+{s}"
        );
    }
    *OWN_STRIP.seen.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    Station::uninstall();
}

/// A TRAFFIC LANDER AND OUR DEPARTURE: at the holding point with the lander
/// 99 s out, held for the landing traffic; lined up with it six miles out,
/// cleared, by the reply and by the release watch alike; lined up with it
/// two miles out, held.
#[test]
fn our_departure_ahead_of_a_traffic_lander_is_judged_in_time() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = ClearStrips;
    let ready = "Gatwick Tower, Speedbird one two three, ready for departure";
    let r = book_runways()[0].clone();
    let on_final = |nm: f64, kts: f64| {
        let (lat, lon) = crate::nav::project_latlon(r.lat_b, r.lon_b, r.heading_a_deg(), nm);
        AircraftState::at_position(lat, lon, false, kts)
    };
    let mut st = gatwick();
    st.install();
    let lander = open_traffic_strip(8);
    *lander.clearance.lock().unwrap() = Some(snap(tower::State::ClearedToLand, "26L", true));
    note_traffic_state(8, on_final(4.02, 146.7));
    st.desk.twr.state = tower::State::TaxiCleared;
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    let line = st.route_keyed(ready, Service::Tower, &ctx, westerly());
    assert!(!line.contains("cleared for takeoff"), "{line}");
    assert!(line.contains("landing traffic on 4 mile final"), "{line}");
    // Lined up, the lander six miles out: cleared.
    st.desk = StripDesk::new();
    st.desk.twr.state = tower::State::LinedUp;
    note_traffic_state(8, on_final(6.0, 140.0));
    let ctx = ctx_for(&mut st, "Speedbird one two three", true);
    assert_eq!(ctx.final_traffic_nm, None);
    let line = st.route_keyed(ready, Service::Tower, &ctx, westerly());
    assert!(line.contains("cleared for takeoff"), "{line}");
    // The release watch, lined up on the pavement.
    let lined = along(&r, 0.97);
    let arm = |t0: Instant| {
        *OWN_STRIP.takeoff_hold.lock().unwrap() = Some(TakeoffHold {
            callsign: "Speedbird one two three".into(),
            runway: "two six left".into(),
            runway_ident: "26L".into(),
            clear_since: None,
            armed_at: t0,
            logged_at: t0,
            lined_up: true,
            behind: false,
            held_for: None,
            cut_in_logged: Default::default(),
        });
    };
    let t0 = Instant::now();
    let tick = |s: u64| {
        takeoff_hold_tick_from(
            &[],
            200.0,
            true,
            Some(lined),
            t0 + std::time::Duration::from_secs(s),
        )
    };
    arm(t0);
    let released = tick(0).or_else(|| tick(20));
    assert!(
        released
            .as_ref()
            .is_some_and(|l| l.contains("cleared for takeoff")),
        "{released:?}"
    );
    // Two miles out, lined up: held.
    note_traffic_state(8, on_final(2.0, 140.0));
    arm(t0);
    for s in [0, 20, 40] {
        assert_eq!(tick(s), None, "+{s}");
    }
    *OWN_STRIP.takeoff_hold.lock().unwrap() = None;
    Station::uninstall();
}
