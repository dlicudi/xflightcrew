//! The wheels' first contact, witnessed at autoland's tick (`FirstContact`)
//! and read by the flight model. EGBB 15, 2026-10-15: the 737 met the
//! runway at +207.1 (autoland: wheels on, radio -0.4, 1,899 fpm), bounced to
//! 23 ft and came down again at +213.4; the monitor's state sample at +207.6
//! read the wheels off at 3.7 ft, so the model, rapport and the landing
//! summary all scored the second contact's 655 fpm.

use super::*;
use crate::flight::{Event, Replay};

const EGBB: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/bench/flight/egbb15_first_contact_bounce.jsonl"
));
const FIRST_CONTACT_T: f64 = 1792085207.1;

#[test]
fn the_witness_notes_the_first_contact_once_with_the_flares_sink() {
    let mut w = FirstContact::default();
    // On the ground before any airborne tick: no contact (armed on the
    // ground, or a rollout already under way).
    assert_eq!(w.tick(0.0, true, Some(-50.0)), None);
    assert_eq!(w.tick(1.0, false, Some(-1898.6)), None);
    assert_eq!(w.tick(1.5, true, Some(-1422.4)), Some((1.5, -1898.6)));
    // The bounce and the second contact: still the first.
    assert_eq!(w.tick(2.0, false, Some(200.0)), None);
    assert_eq!(w.tick(6.0, true, Some(-655.0)), None);
    // A flare tick too old is not the contact's.
    let mut w = FirstContact::default();
    w.tick(0.0, false, Some(-900.0));
    assert_eq!(w.tick(2.0, true, Some(-300.0)), Some((2.0, -300.0)));
    // No vertical speed read at all: no measurement.
    let mut w = FirstContact::default();
    w.tick(0.0, false, None);
    assert_eq!(w.tick(0.5, true, None), None);
}

/// The recorded approach through the model, with the witness fed from the
/// recorded autoland ticks: the touchdown is the first contact's, and the
/// second contact is the bounce.
#[test]
fn a_bounce_between_two_samples_is_scored_at_its_first_contact() {
    let mut r = Replay::default();
    let mut w = FirstContact::default();
    let mut contact = None;
    let mut events = Vec::new();
    let mut summary = crate::interaction::Landing::default();
    let mut said = None;
    let t0 = std::time::Instant::now();
    for v in EGBB
        .lines()
        .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
    {
        let t = v["t"].as_f64().unwrap();
        if v["kind"] == "autoland" {
            let ground = v["ground"].as_bool().unwrap();
            if let Some((t, sink)) = w.tick(t, ground, v["vs_fm"].as_f64()) {
                contact = Some(crate::flight::Contact { t, sink_fpm: sink });
            }
            continue;
        }
        let mut s = crate::flight::Sample::from_state_record(&v).unwrap();
        s.contact = contact;
        let ev = r.flight.tick(&s);
        let now = t0 + std::time::Duration::from_secs_f64(t - 1792085040.0);
        summary.note_flight(&ev);
        let line = summary.tick(now, s.on_ground, s.agl_ft, s.vsi_fpm, s.gs_kts, true, None);
        said = said.or(line);
        events.extend(ev);
    }
    assert_eq!(contact.map(|c| c.t), Some(FIRST_CONTACT_T));
    let td = events
        .iter()
        .find_map(|e| match e {
            Event::Touchdown(td) | Event::Impact(td) => Some(*td),
            _ => None,
        })
        .expect("a touchdown");
    assert_eq!(td.t, FIRST_CONTACT_T, "{td:?}");
    assert_eq!(td.sink_fpm.round(), -1899.0, "{td:?}");
    // 1,899 fpm is past the model's impact line: an impact, not a landing,
    // and the wreck holds the phase (no bounce counted) until at rest.
    assert!(
        events.iter().any(|e| matches!(e, Event::Impact(_))),
        "{events:?}"
    );
    let said = said.expect("the landing summary");
    assert!(
        said.starts_with("Landing summary: touchdown about 1900 feet"),
        "{said}"
    );
    assert!(r.flight.refused().is_empty(), "{:?}", r.flight.refused());
}
