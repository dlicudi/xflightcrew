//! The clearances before the roll: the squawk, the cleared runway, the zone
//! and IFR clearances, the readback wait.

use super::*;

/// The squawk still to dial: the tower's assigned code when the transponder
/// does not already show it. `None` when nothing was assigned or it is set.
/// An unreadable transponder is owed the code too: the tool reads back.
pub fn squawk_owed(assigned: Option<i64>, actual: Option<f64>) -> Option<i64> {
    let code = assigned?;
    match actual {
        Some(a) if a.round() as i64 == code => None,
        _ => Some(code),
    }
}

/// Dial the assigned squawk if the transponder does not show it yet, and say
/// so. Called after the clearance wait AND at the later stages, because the
/// tower's answer can arrive after the wait gave up, and a squawk assigned
/// is a squawk owed until the wheels leave.
pub(super) async fn ensure_squawk(xp: &XPlaneClient, ctx: &ToolCtx<'_>, voice: &Option<Voice>) {
    let actual = xp.get_float(crate::datarefs::TRANSPONDER_CODE).await.ok();
    let Some(code) = squawk_owed(crate::party_line::assigned_squawk(), actual) else {
        return;
    };
    let ok = crate::tools::radio::SetTransponder
        .execute(&json!({ "code": code }), ctx)
        .await
        .is_ok();
    event(
        "squawk_set",
        json!({ "code": code, "ok": ok, "was": actual }),
    );
    if ok {
        announce(
            voice,
            Priority::Advisory,
            &format!(
                "Squawk {}, set.",
                crate::atis::spoken_digits(&format!("{code:04}"))
            ),
        );
    }
}

/// At the taxi clearance: the cleared runway judged against this airframe
/// (blackbox `cleared_runway_fit`); a short one refused aloud with both
/// numbers and the longest one that fits asked for, waited for and taken
/// (`runway_granted`). `Err` is the give-up line.
pub(super) async fn judge_cleared_runway(
    xp: &XPlaneClient,
    tools: &SubTools,
    voice: &Option<Voice>,
    callsign: &Option<String>,
) -> Result<(), String> {
    let Some(cleared) = crate::party_line::clearance_runway() else {
        return Ok(());
    };
    let needed_m = crate::aircraft::takeoff_needed_now_m(
        xp,
        tools.line_up.takeoff_roll_m,
        tools.line_up.mtow_kg,
        500.0,
    )
    .await;
    let fit = cleared_runway_fit(&crate::party_line::field_runways(), &cleared, needed_m);
    event(
        "cleared_runway_fit",
        json!({ "runway": cleared, "needed_m": needed_m.round(), "fit": format!("{fit:?}") }),
    );
    let ClearedFit::Short {
        cleared,
        length_m,
        needed_m,
        ask,
    } = fit
    else {
        return Ok(());
    };
    let line = short_runway_line(length_m, needed_m, &cleared, ask.as_deref());
    let Some(ask) = ask else {
        return Err(format!("{line} Your aircraft."));
    };
    announce(voice, Priority::Advisory, &line);
    radio_out(crate::crew_radio::radio_call(
        callsign,
        &runway_request_call(&cleared, &ask),
    ));
    waiting("the longer runway");
    let granted = wait_until(90.0, 1000, || async {
        crate::party_line::clearance_runway().is_some_and(|r| r.eq_ignore_ascii_case(&ask))
            && taxi_route_is_ours(xp).await
    })
    .await;
    event("runway_granted", json!({ "asked": ask, "granted": granted }));
    if granted {
        Ok(())
    } else {
        Err(format!(
            "{line} The tower didn't give us runway {}. Your aircraft.",
            crate::crew_radio::spell_runway(&ask)
        ))
    }
}

/// The ONE transponder check before a "ready for departure" call
/// (`radio::departure_check`): the assigned code dialled and read back, ALT
/// through the airframe's selector. Said when it changed the code, and when
/// it did not take; blackboxed `transponder_check` either way. Shared by the
/// taxied departure, the runway start and the helicopter's pad departure.
pub(crate) async fn transponder_before_departure(
    xp: &XPlaneClient,
    voice: &Option<Voice>,
    at: &str,
) -> crate::tools::radio::XpdrCheck {
    let check =
        crate::tools::radio::departure_check(xp, crate::party_line::assigned_squawk()).await;
    let mut rec = check.json();
    rec["at"] = json!(at);
    event("transponder_check", rec);
    if let Some(line) = check.failure_line() {
        announce(voice, Priority::Caution, &line);
    } else if let (true, Some(code)) = (check.code_dialled, check.code) {
        announce(
            voice,
            Priority::Advisory,
            &format!(
                "Squawk {}, set.",
                crate::atis::spoken_digits(&format!("{code:04}"))
            ),
        );
    }
    check
}

/// Clearance first, the real order: the zone clearance from the home tower,
/// its squawk dialled and the clearance read back. Only with a simulated
/// tower to answer (real/external ATC and copilot-only leave the clearance
/// to the pilot), and only if we don't already hold a squawk. Shared by every
/// mission that leaves a towered field (full auto before the taxi, the
/// helicopter route before its departure call): the S-76 called "ready for
/// departure" with no zone clearance and flew the zone that way (EDVW,
/// 2026-09-23).
pub(crate) async fn get_zone_clearance(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    callsign: &Option<String>,
    ifr_dest: Option<&str>,
) {
    // No station in reach (an uncontrolled field): nobody to ask, and the
    // squawk wait would only burn a minute (every mission that leaves a
    // field calls this, the helicopter route included). An IFR flight asks
    // for its route even with a code already held (a zone clearance earlier
    // gave the code, not the route); one holding its clearance asks nothing.
    if !crate::mode::atc_enabled()
        || crate::party_line::assigned_clearance().is_some()
        || (ifr_dest.is_none() && crate::party_line::assigned_squawk().is_some())
        || !crate::party_line::station_reaches()
    {
        return;
    }
    if let Some(khz) = crate::party_line::home_tower_khz() {
        let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
    }
    if let Some(dest) = ifr_dest {
        if get_ifr_clearance(xp, ctx, callsign, dest).await {
            return;
        }
        // No IFR clearance came back (a destination the tower's index lacks,
        // a garbled exchange): the zone clearance, as before, so the
        // departure still has a code and a clearance to leave.
        if crate::party_line::assigned_squawk().is_some() {
            ensure_squawk(xp, ctx, voice).await;
            return;
        }
    }
    // The call itself is heard; a line announcing it first is a crew narrating
    // its own hands (LIPK 2026-09-26: said, queued behind the tower, and
    // dropped stale, or heard after the call it announced).
    let circuits = flying_circuits();
    radio_out(crate::crew_radio::radio_call(
        callsign,
        zone_clearance_call(circuits),
    ));
    event("request_clearance", json!({ "circuits": circuits }));
    // The tower answers with a discrete squawk; wait for it, then dial it
    // into the transponder and read it back — the crew's clearance copy.
    // The clock starts when the call is QUEUED, and the call rides the
    // speech queue behind our own arming brief and anyone the tower is
    // working: at Malaga 30 s gave up 13 s before the clearance came
    // back, the squawk was never set, and the tower spent the climb
    // asking us to confirm 4503 while we showed 7000 (23:06Z,
    // 2026-09-12). So the wait is longer, and the squawk is OWED, not
    // missed: `ensure_squawk` sets a late one at the hold and lined up.
    waiting("the squawk");
    wait_until(60.0, 500, || async {
        crate::party_line::assigned_squawk().is_some()
    })
    .await;
    ensure_squawk(xp, ctx, voice).await;
    // The clearance is read back like a real one: the zone cap, the
    // QNH and the squawk (the tower suggests the words, enforces
    // nothing, and answers a correct one with silence). Never done
    // before 2026-09-15: the squawk was set and the taxi asked for.
    if let Some(rb) = crate::party_line::readback_owed() {
        radio_readback(rb.trim_end_matches('.').to_string(), true);
        event("clearance_readback", json!({}));
    }
}

/// The zone clearance call from the ramp: "for circuits" on a circuit, so
/// the tower clears the circuit (`tower::circuit_clearance_lines`), never out
/// of the zone. LEMG (sorties 1790605551 and 1790618416, 2026-09-28): a
/// circuit from the ramp asked a plain "request clearance" and was "cleared
/// to leave the Malaga zone". Pure.
pub(crate) fn zone_clearance_call(circuits: bool) -> &'static str {
    if circuits {
        "request clearance, for circuits"
    } else {
        "request clearance"
    }
}

/// A runway start's zone clearance call: the runway it is lined up on,
/// named, so the tower can give it that runway (`station`'s stated-runway
/// pick for a caller on the strip), and "for circuits" on a circuit, since
/// the zone request states the flight's intent afresh. Pure.
pub(crate) fn strip_clearance_call(runway: &str, circuits: bool) -> String {
    format!(
        "request clearance, runway {}{}",
        crate::crew_radio::spell_runway(runway),
        if circuits { ", for circuits" } else { "" }
    )
}

/// Does a held zone clearance already serve a runway start on `runway`?
/// Only one naming that runway, with its code held. Pure.
pub(crate) fn strip_clearance_held(
    runway: &str,
    cleared: Option<&str>,
    squawk: Option<i64>,
) -> bool {
    squawk.is_some() && cleared.is_some_and(|c| crate::autotakeoff::runway_matches(runway, c))
}

/// [`get_zone_clearance`] for a runway start: asked FOR the strip we are
/// lined up on ([`strip_clearance_call`]), and asked even with a code held
/// when the held clearance names another runway. At LEMG (sortie
/// 1790584715-full_auto, 2026-09-28) the last case's zone clearance (31,
/// 4501) was still held when the runner put the 737 on 13's threshold: the
/// arm asked nothing, read "cleared off 31" and taxied 400 s round the
/// field. The tower still decides: a wind that argues, or a flight it is
/// sequencing, keeps its runway, and the taxied departure goes there.
pub(super) async fn zone_clearance_from_the_strip(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    callsign: &Option<String>,
    runway: &str,
) {
    if !crate::mode::atc_enabled()
        || !crate::party_line::station_reaches()
        || crate::party_line::assigned_clearance().is_some()
        || strip_clearance_held(
            runway,
            crate::party_line::clearance_runway().as_deref(),
            crate::party_line::assigned_squawk(),
        )
    {
        return;
    }
    if let Some(khz) = crate::party_line::home_tower_khz() {
        let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
    }
    let owed_before = crate::party_line::readback_owed();
    radio_out(crate::crew_radio::radio_call(
        callsign,
        &strip_clearance_call(runway, flying_circuits()),
    ));
    event("request_clearance", json!({ "runway": runway }));
    // The answer is a NEW readback owed (a held code answers nothing: it may
    // be the last flight's), or the clearance naming our runway with a code.
    waiting("the clearance");
    let answered = wait_until(60.0, 500, || async {
        let owed = crate::party_line::readback_owed();
        (owed.is_some() && owed != owed_before)
            || strip_clearance_held(
                runway,
                crate::party_line::clearance_runway().as_deref(),
                crate::party_line::assigned_squawk(),
            )
    })
    .await;
    event(
        "strip_clearance",
        json!({ "runway": runway, "answered": answered,
                "cleared": crate::party_line::clearance_runway() }),
    );
    ensure_squawk(xp, ctx, voice).await;
    if let Some(rb) = crate::party_line::readback_owed() {
        radio_readback(rb.trim_end_matches('.').to_string(), true);
        event("clearance_readback", json!({}));
    }
}

/// The IFR clearance the tower owes a readback for is out: its coach line
/// is the IFR readback ("Cleared to ... squawk ..."), not the zone
/// clearance's ("Not above ...") nor a stale landing one ("Cleared to land
/// runway ...", no code in it).
pub(super) fn ifr_readback_line(owed: Option<&str>) -> Option<&str> {
    owed.filter(|r| r.starts_with("Cleared to ") && r.contains("squawk"))
}

/// The IFR call: the destination's ICAO spelled out the way it is said
/// ("request IFR clearance to Echo Golf Papa Hotel"), which the tower
/// parses back to the code.
pub fn ifr_request_call(callsign: &Option<String>, dest_icao: &str) -> String {
    let spelled: Vec<&str> = dest_icao
        .chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .map(|c| crate::atis::nato(c.to_ascii_uppercase()))
        .collect();
    crate::crew_radio::radio_call(
        callsign,
        &format!("request IFR clearance to {}", spelled.join(" ")),
    )
}

/// Ask the home tower for the IFR clearance to `dest`, read it back the
/// way the tower phrases it, and dial its code once the readback is
/// accepted. True when the clearance is held (`assigned_clearance`); false
/// when none came back in time (the caller falls back to the zone call).
pub(super) async fn get_ifr_clearance(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    callsign: &Option<String>,
    dest: &str,
) -> bool {
    radio_out(ifr_request_call(callsign, dest));
    event("request_clearance", json!({ "ifr": true, "dest": dest }));
    // The same queue arithmetic as the zone call's wait: the call rides the
    // speech queue behind anyone the tower is working.
    waiting("the IFR clearance");
    let issued = wait_until(60.0, 500, || async {
        ifr_readback_line(crate::party_line::readback_owed().as_deref()).is_some()
    })
    .await;
    if !issued {
        event("ifr_clearance_missing", json!({ "dest": dest }));
        return false;
    }
    // The readback is ENFORCED: the SID, the altitude and the code, in the
    // tower's own words (the coach line), so it is right first time.
    if let Some(rb) = ifr_readback_line(crate::party_line::readback_owed().as_deref()) {
        radio_out(rb.trim_end_matches('.').to_string());
        event("clearance_readback", json!({ "ifr": true }));
    }
    // The acceptance clock starts when the readback GOES OUT, not when it
    // is queued: the tower's answer is published the moment it is routed,
    // before it is spoken, so the readback is queued at once and waits in
    // the crew queue behind the clearance itself (and anything ahead of it
    // on the frequency). EGKK 2026-09-27: queued +17.4, the clearance
    // spoken +33.2, the readback out +49.6 and accepted, 2 s after a 30 s
    // clock from the queue gave up at +47.5.
    waiting("the readback to be accepted");
    let mut wait = ReadbackWait::new(0.0);
    let t0 = Instant::now();
    loop {
        if ABORT.load(SeqCst) {
            break;
        }
        let pending = crate::crew_radio::crew_radio_pending();
        let accepted = crate::party_line::assigned_clearance().is_some();
        if wait.tick(t0.elapsed().as_secs_f64(), pending, accepted).is_some() {
            break;
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
    let Some(c) = crate::party_line::assigned_clearance() else {
        event("ifr_clearance_not_accepted", json!({ "dest": dest }));
        return false;
    };
    crate::blackbox::record(
        "atc",
        json!({
            "event": "ifr_clearance", "side": "crew", "dest": c.dest, "sid": c.sid,
            "runway": c.runway, "initial_alt_ft": c.initial_alt_ft, "squawk": c.squawk,
        }),
    );
    // The clearance's code, dialled (at a tower-only field it is the code
    // already held, which `ensure_squawk` never sees).
    let actual = xp.get_float(crate::datarefs::TRANSPONDER_CODE).await.ok();
    if let Some(code) = squawk_owed(Some(c.squawk), actual) {
        let ok = crate::tools::radio::SetTransponder
            .execute(&json!({ "code": code }), ctx)
            .await
            .is_ok();
        event(
            "squawk_set",
            json!({ "code": code, "ok": ok, "was": actual }),
        );
    }
    true
}

/// The wait for an IFR readback to be accepted: [`READBACK_ACCEPT_S`] from
/// the moment the readback left the crew queue (went out on the frequency),
/// never from when it was queued; a readback that never leaves the queue is
/// given up after [`READBACK_QUEUE_MAX_S`].
pub(super) struct ReadbackWait {
    pub(super) start_s: f64,
    pub(super) out_s: Option<f64>,
}

/// From the readback going out to the tower's acceptance (it is judged the
/// moment it is routed, so this is slack for a busy frequency).
pub(super) const READBACK_ACCEPT_S: f64 = 30.0;
/// The longest a readback may sit in the crew queue behind the clearance
/// and the frequency's traffic before full auto stops waiting.
pub(super) const READBACK_QUEUE_MAX_S: f64 = 120.0;

impl ReadbackWait {
    pub(super) fn new(start_s: f64) -> Self {
        ReadbackWait {
            start_s,
            out_s: None,
        }
    }

    /// One poll: `Some(true)` accepted, `Some(false)` given up, `None` wait on.
    pub(super) fn tick(&mut self, now_s: f64, queued: bool, accepted: bool) -> Option<bool> {
        if accepted {
            return Some(true);
        }
        if !queued && self.out_s.is_none() {
            self.out_s = Some(now_s);
        }
        match self.out_s {
            Some(out) if now_s - out >= READBACK_ACCEPT_S => Some(false),
            None if now_s - self.start_s >= READBACK_QUEUE_MAX_S => Some(false),
            _ => None,
        }
    }
}
