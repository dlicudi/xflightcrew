//! The crew works an instrument arrival's frequencies: approach first, then
//! the tower once approach hands it over, back to approach off a missed
//! approach. The station decides who we talk to (`party_line::
//! take_arrival_handoff`, `take_approach_readback`); this side reads back,
//! retunes COM1 and makes the next call, as the departure and region
//! handoffs are worked. Radio only: nothing here flies the aircraft.
//!
//! Until 2026-10-10 our own arrival's inbound call went to the destination's
//! TOWER, which answered "join final runway two six left, report final" as
//! to a VFR circuit, while the airliners round it checked in with approach,
//! were cleared for the ILS and handed to the tower established (the realism
//! review; the traffic's half is `crates/traffic/src/arrival.rs`).

use super::*;

/// The inbound call went to APPROACH this mission: the "final" report is
/// approach's "established", and the tower call is the handoff's.
pub(super) static APPROACH_FIRST: AtomicBool = AtomicBool::new(false);
/// The tower has been called on the arrival handoff this mission: the
/// arrival's own final report is owed no more.
pub(super) static TOWER_CALLED: AtomicBool = AtomicBool::new(false);
/// An arrival handoff is being worked (read back, retuned, the next call
/// waiting): the arrival's own final report must not go out on the old
/// frequency meanwhile.
pub(super) static HANDOFF_BUSY: AtomicBool = AtomicBool::new(false);

/// How long the handoff's readback may wait in the crew radio queue before
/// the radio is retuned regardless (the queue drains between the main
/// loop's speech; a readback behind a long line is late, never lost).
const HANDOFF_READBACK_WAIT_SECS: f64 = 20.0;
/// The tower call waits for the final, as the arrival's own report does
/// (`autoland::final_report_due`), at most this long: approach hands over at
/// the join, and the final is ours a few miles later. Past it the tower is
/// called anyway: it must hear us before the landing.
const TOWER_CALL_WAIT_SECS: f64 = 240.0;

/// A new mission's arrival: nothing called yet.
pub(super) fn reset_arrival_radio() {
    APPROACH_FIRST.store(false, SeqCst);
    TOWER_CALLED.store(false, SeqCst);
    HANDOFF_BUSY.store(false, SeqCst);
}

/// The inbound call goes to APPROACH: the destination's station is the one
/// on the air (`swapped`), it has an approach unit, and the station works us
/// as an instrument arrival (`party_line::own_instrument_arrival`: IFR, an
/// airline, or a published approach declared by the arrival). Its
/// frequency, else `None` and the tower takes the call as before. Pure over
/// the party line's answers.
pub fn approach_first_khz(
    swapped: bool,
    approach_khz: Option<i64>,
    instrument: bool,
) -> Option<i64> {
    approach_khz.filter(|_| swapped && instrument)
}

/// The readback of a handoff: the unit and its frequency, as the departure
/// handoff's ("Gatwick Tower one two four decimal two two five").
pub fn handoff_readback(
    callsign: &Option<String>,
    h: &crate::party_line::ArrivalHandoff,
) -> String {
    crate::crew_radio::radio_call(
        callsign,
        &format!("{} {}", h.to, crate::atis::spoken_mhz(h.khz)),
    )
}

/// What the crew says once tuned to the unit it was handed to: the tower
/// hears "established ILS runway ..." (or "final runway ..."), approach off
/// a missed approach "missed approach", approach after an early call to the
/// tower the inbound again.
pub fn handoff_call(callsign: &Option<String>, h: &crate::party_line::ArrivalHandoff) -> String {
    let words = if h.to_tower {
        if h.ils {
            format!("established ILS runway {}", h.runway_spoken)
        } else {
            format!("final runway {}", h.runway_spoken)
        }
    } else if h.missed {
        "missed approach".to_string()
    } else {
        format!("inbound for landing, runway {}", h.runway_spoken)
    };
    crate::crew_radio::radio_call(callsign, &words)
}

/// The arrival's own final report is still owed: no handoff is being worked
/// and the tower has not been called on one.
pub fn final_call_owed() -> bool {
    !TOWER_CALLED.load(SeqCst) && !HANDOFF_BUSY.load(SeqCst)
}

/// The arrival's final report, approach first: "established, runway ..."
/// while approach still watches for it (its answer is the tower handoff),
/// else the tower's "final runway ...".
pub fn final_call_words(runway: Option<&str>, approach_watching: bool) -> String {
    match (runway, approach_watching) {
        (Some(r), true) => format!("established, runway {}", crate::crew_radio::spell_runway(r)),
        (None, true) => "established".to_string(),
        (Some(r), false) => format!("final runway {}", crate::crew_radio::spell_runway(r)),
        (None, false) => "final".to_string(),
    }
}

/// The monitor's pass: approach's clearance and any arrival handoff are
/// drained EVERY pass (a handoff nobody works is not kept for a later
/// mission), and worked only by a crew that works the radio
/// (`crew_works_radio`: full auto flying, hands allowed). A pilot flying
/// answers for himself; the landing-clearance coach still nudges him.
pub fn arrival_radio_tick(base_url: &str, crew_works_radio: bool) {
    let readback = crate::party_line::take_approach_readback();
    let handoff = crate::party_line::take_arrival_handoff();
    if !crew_works_radio || (readback.is_none() && handoff.is_none()) {
        if let Some(h) = &handoff {
            crate::blackbox::record(
                "atc",
                json!({ "route": "arrival_handoff", "worked": false, "to": h.to, "khz": h.khz }),
            );
        }
        return;
    }
    if handoff.is_some() {
        // Set before the task starts: the arrival's final report reads it.
        HANDOFF_BUSY.store(true, SeqCst);
    }
    let base = base_url.to_string();
    tokio::spawn(async move {
        let xp = crate::xplane::XPlaneClient::new(base);
        let callsign = crate::crew_radio::crew_callsign(&xp).await;
        if let Some(words) = readback {
            radio_readback(crate::crew_radio::radio_call(&callsign, &words), false);
            event("approach_readback", json!({ "words": words }));
        }
        if let Some(h) = handoff {
            obey_arrival_handoff(
                &xp,
                &callsign,
                &h,
                HANDOFF_READBACK_WAIT_SECS,
                TOWER_CALL_WAIT_SECS,
                crate::autoland::established_on_final,
            )
            .await;
            HANDOFF_BUSY.store(false, SeqCst);
        }
    });
}

/// Work one arrival handoff: read it back on the frequency it came on (once:
/// `transmit_readback` records it), wait until it has gone out (at most
/// `readback_wait_secs`), retune COM1, then make the next call. The tower
/// call waits for the final (`established`, at most `tower_wait_secs`):
/// approach lets an established arrival go at the join, and the final
/// report is made on the final (LEGR-LXGB, 2026-10-06, sortie
/// 1791275171-full_auto: a "final" 12 nm out was cleared to land).
pub(crate) async fn obey_arrival_handoff(
    xp: &dyn crate::xplane::XPlane,
    callsign: &Option<String>,
    h: &crate::party_line::ArrivalHandoff,
    readback_wait_secs: f64,
    tower_wait_secs: f64,
    established: fn() -> bool,
) {
    let readback = handoff_readback(callsign, h);
    radio_readback(readback.clone(), false);
    let out = wait_until(readback_wait_secs, 250, || {
        let r = readback.clone();
        async move { crate::crew_radio::readback_out(&r) }
    })
    .await;
    if out {
        // A beat for the readback to be off the air before the radio leaves
        // the frequency it was given on.
        tokio::time::sleep(Duration::from_secs(2)).await;
    }
    let tuned = xp.set_float(COM1_FREQ_KHZ, h.khz as f64).await.is_ok();
    event(
        "arrival_handoff",
        json!({
            "to": h.to, "khz": h.khz, "to_tower": h.to_tower, "missed": h.missed,
            "readback_out": out, "tuned": tuned,
        }),
    );
    if !h.to_tower {
        // Sent to approach (early, or off a missed approach): approach is
        // ours now, and the tower call comes with its handoff.
        APPROACH_FIRST.store(true, SeqCst);
    }
    if h.to_tower {
        let on_final = wait_until(tower_wait_secs, 500, || async move { established() }).await;
        if ABORT.load(SeqCst) {
            return;
        }
        TOWER_CALLED.store(true, SeqCst);
        event(
            "tower_call",
            json!({ "established": on_final, "runway": h.runway_spoken }),
        );
    }
    radio_out(handoff_call(callsign, h));
}
