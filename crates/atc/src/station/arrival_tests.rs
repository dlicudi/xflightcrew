//! OUR instrument arrival is worked as the airliners round it are (the
//! realism review, 2026-10-10): approach first, cleared on its own
//! navigation, handed to the tower once established, the tower clearing the
//! landing by the runway book; an early call to the tower is sent to
//! approach, and a go-around goes back to approach for another.

use super::book_tests::book_runways;
use super::strip_tests::gatwick;
use super::tests::SIDE_STATE_LOCK;
use super::*;

const CS: &str = "Speedbird one two three";
const APPROACH_KHZ: i64 = 126825;

/// Gatwick with an approach unit and an ILS on 26L (no CIFP platform, so
/// the geometric 2,500 ft above the 200 ft field), installed.
fn gatwick_with_approach() -> Station {
    Station::uninstall();
    let mut st = gatwick();
    st.apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_APPROACH,
        khz: APPROACH_KHZ,
        name: "Approach".into(),
    });
    st.approach_khz = Some(APPROACH_KHZ);
    st.install();
    *ILS_FACTS.lock().unwrap() = Some(std::sync::Arc::new(IlsFacts {
        ils: vec![nav::AirportIls {
            runway: "26L".into(),
            freq_10khz: 11090,
            course_mag: 257.0,
            gs_deg: Some(3.0),
        }],
        platform_ft: Default::default(),
    }));
    st
}

/// Retires the station and the ILS facts however the test ends.
struct Retire;

impl Drop for Retire {
    fn drop(&mut self) {
        *ILS_FACTS.lock().unwrap_or_else(|e| e.into_inner()) = None;
        Station::uninstall();
    }
}

fn westerly() -> Wx {
    Wx {
        wind_from_deg: 250.0,
        wind_kts: 8.0,
    }
}

/// On 26L's extended centreline, `nm` from its threshold, at `alt_ft`.
fn on_the_centreline(nm: f64, alt_ft: f64) -> AircraftState {
    let r = &book_runways()[0];
    let (lat, lon) = crate::nav::project_latlon(r.lat_b, r.lon_b, r.heading_a_deg(), nm);
    AircraftState::at_position(lat, lon, false, alt_ft)
}

/// 26L's final course, degrees true (landing westbound).
fn course() -> f64 {
    book_runways()[0].heading_b_deg()
}

/// Our call, keyed on `service`, from `state`: the context built as the
/// keyed path builds it (no sim), then the keyed path's answer.
fn say(st: &mut Station, state: &AircraftState, service: Service, text: &str) -> String {
    let intent = tower::parse_intent(text);
    st.note_stated_runway(text, intent.clone());
    let (ctx, wx) = st.ctx_for_call(
        state,
        westerly(),
        "QNH one zero one three".into(),
        CS.into(),
        intent,
        &[],
        &[],
        Instant::now(),
    );
    st.route_keyed(text, service, &ctx, wx)
}

fn tower_freq(d: Dialect) -> String {
    atis::spoken_mhz_in(124225, d)
}

/// THE WHOLE EXCHANGE, CAP 413: the check-in to approach is cleared for the
/// ILS on our own navigation (descend to the platform, QNH, report
/// established), the crew's readback is silence, the established report is
/// handed to the tower with its frequency (published for the crew, the
/// landing coach armed), the frequency readback is silence, and the tower
/// clears the landing on the established call.
#[test]
fn our_instrument_arrival_works_approach_then_the_tower() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _retire = Retire;
    let mut st = gatwick_with_approach();
    let far = on_the_centreline(25.0, 6000.0);
    let r = say(
        &mut st,
        &far,
        Service::Approach,
        "Gatwick Approach, Speedbird one two three, information Bravo, descending altitude six \
         thousand feet, inbound",
    );
    assert_eq!(
        r,
        "Speedbird one two three, Gatwick Approach, descend altitude two thousand seven hundred \
         feet, QNH one zero one three, cleared ILS approach runway two six left, report \
         established.",
    );
    assert!(
        OWN_STRIP.vectors.lock().unwrap().is_none(),
        "no vectors: own navigation"
    );
    assert!(crate::party_line::approach_watch_armed());
    assert_eq!(
        crate::party_line::clearance_state(),
        Some(tower::State::Inbound),
        "inbound on approach's runway, for the crew's arrival to set up"
    );
    assert_eq!(
        crate::party_line::clearance_runway().as_deref(),
        Some("26L")
    );
    let readback = crate::party_line::take_approach_readback().expect("readback words");
    assert_eq!(
        readback,
        "descending altitude two thousand seven hundred feet, cleared ILS approach runway two \
         six left"
    );
    let r = say(
        &mut st,
        &far,
        Service::Approach,
        &format!("{readback}, {CS}"),
    );
    assert_eq!(r, "", "the clearance's readback is silence");

    let nine = on_the_centreline(9.0, 2900.0);
    let r = say(
        &mut st,
        &nine,
        Service::Approach,
        "Gatwick Approach, Speedbird one two three, established ILS runway two six left",
    );
    assert_eq!(
        r,
        format!(
            "{CS}, contact Gatwick Tower on {}.",
            tower_freq(Dialect::Icao)
        )
    );
    assert_eq!(
        crate::party_line::take_arrival_handoff(),
        Some(ArrivalHandoff {
            to: "Gatwick Tower".into(),
            khz: 124225,
            to_tower: true,
            missed: false,
            runway_spoken: "two six left".into(),
            ils: true,
        })
    );
    assert!(
        !crate::party_line::approach_watch_armed(),
        "approach's job is done"
    );
    assert_eq!(
        crate::party_line::landing_handoff_runway().as_deref(),
        Some("two six left"),
        "the landing coach waits for the tower's clearance"
    );
    let r = say(
        &mut st,
        &nine,
        Service::Approach,
        &format!("Gatwick Tower {}, {CS}", tower_freq(Dialect::Icao)),
    );
    assert_eq!(r, "", "the frequency readback is silence");

    let r = say(
        &mut st,
        &on_the_centreline(8.0, 2600.0),
        Service::Tower,
        "Gatwick Tower, Speedbird one two three, established ILS runway two six left",
    );
    assert!(r.contains("cleared to land"), "{r}");
    assert!(r.contains("two six left"), "{r}");
}

/// The FAA's words for the same exchange: "descend and maintain", the
/// altimeter, "cleared ILS runway ... approach", and the tower's frequency
/// with "point".
#[test]
fn the_faa_arrival_is_cleared_in_the_faa_words() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _retire = Retire;
    let mut st = gatwick_with_approach();
    st.set_dialect(Dialect::Faa);
    let far = on_the_centreline(25.0, 6000.0);
    let r = say(
        &mut st,
        &far,
        Service::Approach,
        "Gatwick Approach, Speedbird one two three, descending through six thousand, inbound",
    );
    assert!(
        r.starts_with(
            "Speedbird one two three, Gatwick Approach, descend and maintain two thousand seven \
             hundred"
        ) && r.ends_with("cleared ILS runway two six left approach, report established."),
        "{r}"
    );
    assert_eq!(
        crate::party_line::take_approach_readback().as_deref(),
        Some("descend and maintain two thousand seven hundred, cleared ILS runway two six left approach")
    );
    let r = say(
        &mut st,
        &on_the_centreline(9.0, 2900.0),
        Service::Approach,
        "Gatwick Approach, Speedbird one two three, established ILS runway two six left",
    );
    assert!(r.contains(&tower_freq(Dialect::Faa)), "{r}");
    assert!(r.contains("point"), "{r}");
}

/// A pilot who never reports established is still handed to the tower:
/// approach's watch sees us established on the localiser inside the join
/// (about eleven miles on a 3 degree ILS from 2,500 ft) and says "contact
/// Tower" once. Twenty miles out, or crossing the final at ninety degrees,
/// is not established and stays silent.
#[test]
fn approach_hands_over_an_arrival_it_sees_established() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _retire = Retire;
    let mut st = gatwick_with_approach();
    say(
        &mut st,
        &on_the_centreline(25.0, 6000.0),
        Service::Approach,
        "Gatwick Approach, Speedbird one two three, inbound",
    );
    assert!(crate::party_line::approach_watch_armed());
    let tick = |s: &AircraftState, track: f64| approach_watch_tick_for(&OWN_STRIP, s, Some(track));
    assert_eq!(
        tick(&on_the_centreline(20.0, 4000.0), course()),
        None,
        "too far"
    );
    assert_eq!(
        tick(&on_the_centreline(9.0, 2900.0), course() + 90.0),
        None,
        "crossing the final is not established"
    );
    let line = tick(&on_the_centreline(9.0, 2900.0), course()).expect("handed over");
    assert_eq!(
        line,
        format!("{CS}, contact Gatwick Tower {}.", tower_freq(Dialect::Icao))
    );
    let h = crate::party_line::take_arrival_handoff().expect("published for the crew");
    assert!(h.to_tower && h.khz == 124225 && h.ils, "{h:?}");
    assert!(!crate::party_line::approach_watch_armed());
    assert_eq!(
        tick(&on_the_centreline(8.0, 2600.0), course()),
        None,
        "said once"
    );
}

/// THE WRONG ORDER: our instrument arrival calling the TOWER twenty miles
/// out is sent to approach ("join final" is the VFR answer), published for
/// the crew. Inside twelve miles the tower takes the call as before; a VFR
/// arrival is answered as before wherever it calls from; and a field with
/// no approach unit is the tower's at any range.
#[test]
fn the_tower_sends_an_early_instrument_arrival_to_approach() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _retire = Retire;
    let mut st = gatwick_with_approach();
    let twenty = on_the_centreline(20.0, 5000.0);
    let r = say(
        &mut st,
        &twenty,
        Service::Tower,
        "Gatwick Tower, Speedbird one two three, inbound for landing",
    );
    assert_eq!(
        r,
        format!(
            "{CS}, Gatwick Tower, contact Gatwick Approach on {}.",
            atis::spoken_mhz_in(APPROACH_KHZ, Dialect::Icao)
        )
    );
    let h = crate::party_line::take_arrival_handoff().expect("published");
    assert!(!h.to_tower && !h.missed && h.khz == APPROACH_KHZ, "{h:?}");

    let r = say(
        &mut st,
        &on_the_centreline(8.0, 2600.0),
        Service::Tower,
        "Gatwick Tower, Speedbird one two three, inbound for landing",
    );
    assert!(
        r.contains("join final"),
        "inside twelve miles, the tower's: {r}"
    );

    // A VFR arrival (no IFR clearance, no airline) is the tower's at twenty.
    let text = "Gatwick Tower, November one seven two Sierra Papa, inbound for landing";
    let intent = tower::parse_intent(text);
    let (ctx, wx) = st.ctx_for_call(
        &twenty,
        westerly(),
        "QNH one zero one three".into(),
        "November one seven two Sierra Papa".into(),
        intent,
        &[],
        &[],
        Instant::now(),
    );
    let r = st.route_keyed(text, Service::Tower, &ctx, wx);
    assert!(r.contains("join final"), "VFR: {r}");
    assert!(crate::party_line::take_arrival_handoff().is_none());

    // No approach unit: the tower's at any range.
    st.approach_khz = None;
    let r = say(
        &mut st,
        &twenty,
        Service::Tower,
        "Gatwick Tower, Speedbird one two three, inbound for landing",
    );
    assert!(r.contains("join final"), "no approach unit: {r}");
}

/// THE PILOT'S CHECK-IN: "Gatwick Director, Speedbird one two three,
/// information Bravo, descending flight level eight zero" has no intent of
/// its own and its "descending" read as a readback (silence). It is the
/// arrival's check-in, cleared for the approach; once cleared, the same
/// words are a readback again.
#[test]
fn the_pilots_check_in_words_earn_the_approach_clearance() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _retire = Retire;
    let mut st = gatwick_with_approach();
    let far = on_the_centreline(30.0, 8000.0);
    let text = "Gatwick Director, Speedbird one two three, information Bravo, descending flight \
                level eight zero";
    assert_eq!(tower::parse_intent(text), tower::Intent::Unknown);
    let r = say(&mut st, &far, Service::Approach, text);
    assert!(
        r.contains("cleared ILS approach runway two six left, report established."),
        "{r}"
    );
    assert_eq!(
        say(&mut st, &far, Service::Approach, text),
        "",
        "now a readback"
    );
}

/// A VFR arrival checking in with approach is vectored as before, with a
/// code; nothing of the instrument path is armed.
#[test]
fn a_vfr_arrival_on_approach_is_still_vectored() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _retire = Retire;
    let mut st = gatwick_with_approach();
    let text = "Gatwick Approach, November one seven two Sierra Papa, inbound for landing";
    let intent = tower::parse_intent(text);
    let (ctx, wx) = st.ctx_for_call(
        &on_the_centreline(20.0, 4000.0),
        westerly(),
        "QNH one zero one three".into(),
        "November one seven two Sierra Papa".into(),
        intent,
        &[],
        &[],
        Instant::now(),
    );
    let r = st.route_keyed(text, Service::Approach, &ctx, wx);
    assert!(
        r.contains("squawk") && r.contains("Vectoring ILS approach"),
        "{r}"
    );
    assert!(OWN_STRIP.vectors.lock().unwrap().is_some());
    assert!(!crate::party_line::approach_watch_armed());
    assert!(crate::party_line::take_approach_readback().is_none());
}

/// THE MISSED APPROACH: our instrument arrival going around is told the
/// missed approach and "contact Approach" by the tower (published for the
/// crew, missed), and its check-in with approach is climbed and
/// re-sequenced for another ILS, approach's watch armed again for the next
/// "established".
#[test]
fn a_go_around_goes_back_to_approach_for_another() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _retire = Retire;
    let mut st = gatwick_with_approach();
    let short = on_the_centreline(2.0, 800.0);
    let r = say(
        &mut st,
        &short,
        Service::Tower,
        "Gatwick Tower, Speedbird one two three, established ILS runway two six left",
    );
    assert!(r.contains("cleared to land"), "{r}");
    let r = say(
        &mut st,
        &short,
        Service::Tower,
        "Gatwick Tower, Speedbird one two three, going around",
    );
    assert!(
        r.contains(&format!(
            "contact Gatwick Approach on {}",
            atis::spoken_mhz_in(APPROACH_KHZ, Dialect::Icao)
        )),
        "{r}"
    );
    let h = crate::party_line::take_arrival_handoff().expect("published");
    assert!(!h.to_tower && h.missed && h.khz == APPROACH_KHZ, "{h:?}");

    let r = say(
        &mut st,
        &on_the_centreline(0.0, 2500.0),
        Service::Approach,
        "Gatwick Approach, Speedbird one two three, missed approach, climbing altitude three \
         thousand feet",
    );
    assert_eq!(
        r,
        "Speedbird one two three, Gatwick Approach, roger, climb altitude three thousand feet, \
         expect vectors for a further ILS approach runway two six left, report established."
    );
    assert!(
        crate::party_line::approach_watch_armed(),
        "watching the next approach"
    );
}
