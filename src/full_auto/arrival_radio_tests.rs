//! The crew's half of an instrument arrival's frequencies
//! (`full_auto::arrival_radio`): approach first, the handoff read back once,
//! COM1 retuned, the next unit called, every call shaped as the station
//! parses it.

use super::*;
use crate::party_line::ArrivalHandoff;
use crate::tower::{parse_intent, Intent};

fn cs() -> Option<String> {
    Some("Speedbird one two three".to_string())
}

fn to_tower(ils: bool) -> ArrivalHandoff {
    ArrivalHandoff {
        to: "Gatwick Tower".into(),
        khz: 124_225,
        to_tower: true,
        missed: false,
        runway_spoken: "two six left".into(),
        ils,
    }
}

/// The inbound goes to approach only for an instrument arrival, at the
/// destination's own station, where the field has an approach unit.
#[test]
fn the_inbound_goes_to_approach_only_for_an_instrument_arrival_with_one() {
    assert_eq!(approach_first_khz(true, Some(126_825), true), Some(126_825));
    assert_eq!(
        approach_first_khz(false, Some(126_825), true),
        None,
        "not yet swapped"
    );
    assert_eq!(
        approach_first_khz(true, Some(126_825), false),
        None,
        "VFR: the tower"
    );
    assert_eq!(
        approach_first_khz(true, None, true),
        None,
        "no approach row: the tower"
    );
}

/// Every word the crew keys parses as the station needs it: the handoff's
/// readback is a readback (silence), the tower call the final report (the
/// landing clearance), the missed approach a go-around (another approach),
/// the early caller's retry an inbound (the approach clearance), and the
/// arrival's own report approach's "established".
#[test]
fn every_arrival_call_parses_as_the_station_needs() {
    let rb = handoff_readback(&cs(), &to_tower(true));
    assert_eq!(
        rb,
        "Speedbird one two three, Gatwick Tower one two four decimal two two five"
    );
    assert!(crate::tower::is_readback(&rb), "{rb}");
    let call = handoff_call(&cs(), &to_tower(true));
    assert_eq!(
        call,
        "Speedbird one two three, established ILS runway two six left"
    );
    assert_eq!(parse_intent(&call), Intent::ReportFinal);
    assert_eq!(
        parse_intent(&handoff_call(&cs(), &to_tower(false))),
        Intent::ReportFinal,
        "final runway ..."
    );
    let missed = ArrivalHandoff {
        to: "Gatwick Approach".into(),
        khz: 126_825,
        to_tower: false,
        missed: true,
        ..to_tower(true)
    };
    assert_eq!(
        parse_intent(&handoff_call(&cs(), &missed)),
        Intent::GoAround
    );
    let early = ArrivalHandoff {
        missed: false,
        ..missed
    };
    assert_eq!(
        parse_intent(&handoff_call(&cs(), &early)),
        Intent::RequestLanding {
            touch_and_go: false
        }
    );
    for watching in [true, false] {
        let w = final_call_words(Some("26L"), watching);
        assert_eq!(parse_intent(&w), Intent::ReportFinal, "{w}");
    }
    assert_eq!(
        final_call_words(Some("26L"), true),
        "established, runway two six left"
    );
}

/// A queued "established" report dies with its final, as "final" does.
#[test]
fn an_established_report_holds_only_for_its_final() {
    let h = crate::crew_radio::holds_for(&crate::crew_radio::CrewCall::Text(
        "Speedbird one two three, established ILS runway two six left".into(),
    ));
    assert!(
        matches!(
            h,
            crate::crew_radio::Holds::Airborne {
                before_landing: true,
                ..
            }
        ),
        "{h:?}"
    );
}

/// THE HANDOFF WORKED: approach's "contact Gatwick Tower" is read back ONCE
/// on the old frequency, COM1 is tuned to the tower, and the tower is
/// called established; the arrival's own final report is owed no more.
#[tokio::test]
async fn the_handoff_is_read_back_once_then_the_tower_is_tuned_and_called() {
    let _q = crate::crew_radio::QUEUE_LOCK
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    crate::crew_radio::forget_flight_calls();
    reset_arrival_radio();
    let xp = crate::xplane::FakeXPlane::with_reads(&[]);
    obey_arrival_handoff(&xp, &cs(), &to_tower(true), 0.05, 0.05, || true).await;
    assert_eq!(
        xp.writes_to(COM1_FREQ_KHZ),
        vec![124_225.0],
        "tuned once, to the tower"
    );
    let sent: Vec<String> = crate::crew_radio::take_crew_radio()
        .into_iter()
        .map(|q| crate::crew_radio::call_words(&q.call))
        .collect();
    assert_eq!(
        sent,
        vec![
            "Speedbird one two three, Gatwick Tower one two four decimal two two five".to_string(),
            "Speedbird one two three, established ILS runway two six left".to_string(),
        ],
        "the readback once, then the tower call"
    );
    assert!(!final_call_owed(), "the tower has been called");
    reset_arrival_radio();
}

/// Off a missed approach the crew goes back to approach and checks in
/// "missed approach"; no tower call is marked made.
#[tokio::test]
async fn a_missed_approach_goes_back_to_approach() {
    let _q = crate::crew_radio::QUEUE_LOCK
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    crate::crew_radio::forget_flight_calls();
    reset_arrival_radio();
    let xp = crate::xplane::FakeXPlane::with_reads(&[]);
    let h = ArrivalHandoff {
        to: "Gatwick Approach".into(),
        khz: 126_825,
        to_tower: false,
        missed: true,
        ..to_tower(true)
    };
    obey_arrival_handoff(&xp, &cs(), &h, 0.05, 0.05, || false).await;
    assert_eq!(xp.writes_to(COM1_FREQ_KHZ), vec![126_825.0]);
    let sent: Vec<String> = crate::crew_radio::take_crew_radio()
        .into_iter()
        .map(|q| crate::crew_radio::call_words(&q.call))
        .collect();
    assert_eq!(sent.len(), 2, "{sent:?}");
    assert_eq!(sent[1], "Speedbird one two three, missed approach");
    assert!(final_call_owed());
}
