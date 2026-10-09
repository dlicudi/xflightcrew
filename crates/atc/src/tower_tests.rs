use super::*;

fn ctx() -> Ctx {
    Ctx {
        callsign: "November one seven two Sierra Papa".into(),
        airport: "Gibraltar".into(),
        runway: "zero niner".into(),
        qnh: "QNH one zero one seven".into(),
        wind: "zero niner zero degrees, three knots".into(),
        holding_point: "Alpha".into(),
        stand: String::new(),
        approach: Some("one two two decimal eight".into()),
        squawk: "seven zero zero zero".into(),
        final_traffic_nm: None,
        final_traffic_callsign: None,
        final_traffic_count: 0,
        final_traffic_id: None,
        sequence_ahead: None,
        runway_occupied: false,
        own_on_runway: false,
        helicopter: false,
        runway_refusal: None,
        transition_level_ft: None,
        wake_secs_left: None,
        taxi_via: Vec::new(),
        taxi_crossings: Vec::new(),
        hold_short_crossings: false,
        dialect: crate::atis::Dialect::Icao,
        uk: false,
        missed_approach: None,
        final_traffic_type: None,
        crossing: None,
        on_ground: false,
        ifr: None,
        circuit: None,
    }
}

/// Issue #178: a runway change opens the reply, after the callsign; a
/// reply not addressed to the caller is left alone.
#[test]
fn a_runway_change_opens_the_reply() {
    let cs = "November eight one six November November";
    let taxi = format!(
        "{cs}, taxi to holding point Hotel November one runway one three first via \
             Echo one one. Report ready for departure."
    );
    assert_eq!(
        runway_change_line(cs, "one three", &taxi),
        format!(
            "{cs}, runway change, runway one three in use. Taxi to holding point Hotel \
                 November one runway one three first via Echo one one. Report ready for \
                 departure."
        )
    );
    assert_eq!(runway_change_line(cs, "one three", "say again"), "say again");
}

/// ctx() on the ground as circuit traffic, code 4501, in the ICAO or the
/// FAA dialect (the FAA pressure is an altimeter setting).
fn circuit_ctx(faa: bool) -> Ctx {
    Ctx {
        on_ground: true,
        squawk: "four five zero one".into(),
        qnh: if faa {
            "altimeter three zero zero one".into()
        } else {
            "QNH one zero one niner".into()
        },
        circuit: Some(CircuitOffer {
            height_ft: circuit_height_ft(52.0),
            faa,
            left: None,
            not_available: false,
        }),
        dialect: if faa {
            crate::atis::Dialect::Faa
        } else {
            crate::atis::Dialect::Icao
        },
        // Gibraltar's tower is the RAF's: CAP 413 words.
        uk: !faa,
        ..ctx()
    }
}

/// LEMG 13 (sorties 1790590663 and 1790591256, 2026-09-28): "request
/// clearance, runway one three, for circuits" was "cleared to leave the
/// Malaga zone, not above two thousand feet". A circuit gets the
/// circuit's clearance in its dialect, and the readback the tower
/// suggests (the crew reads it verbatim) is a readback, not a request.
#[test]
fn a_circuit_is_cleared_for_the_circuit_not_out_of_the_zone() {
    assert_eq!(circuit_height_ft(52.0), 1100.0, "LEMG");
    assert_eq!(circuit_height_ft(598.0), 1600.0);
    assert_eq!(circuit_height_ft(-10.0), 1000.0, "below sea level floors");
    let request = "november eight one six november november, \
                       request clearance, runway one three, for circuits";
    assert_eq!(parse_intent(request), Intent::RequestClearance);
    assert!(wants_circuits(request));
    for (faa, line, rb) in [
        (
            false,
            "November one seven two Sierra Papa, runway zero niner, circuit height one \
                 thousand one hundred feet, QNH one zero one niner, squawk four five zero one, \
                 report ready for departure.",
            "Runway zero niner, circuit height one thousand one hundred feet, QNH one \
                 zero one niner, squawk four five zero one, November one seven two Sierra Papa.",
        ),
        (
            true,
            "November one seven two Sierra Papa, runway zero niner, closed traffic \
                 approved, pattern altitude one thousand one hundred, altimeter three zero \
                 zero one, squawk four five zero one, report ready for departure.",
            "Closed traffic runway zero niner, pattern altitude one thousand one hundred, \
                 altimeter three zero zero one, squawk four five zero one, November one seven \
                 two Sierra Papa.",
        ),
    ] {
        let c = circuit_ctx(faa);
        let mut twr = Tower::new();
        let reply = twr.handle(Intent::RequestClearance, &c);
        assert_eq!(reply, line);
        assert_eq!(twr.state, State::Parked, "a clearance moves no sequence");
        let owed = twr
            .suggested_readback(&c.callsign)
            .expect("readback suggested");
        assert_eq!(owed, rb);
        // What the crew transmits (the owed line, stop trimmed) is heard
        // as a readback: silence, and the debt is paid.
        let sent = owed.trim_end_matches('.');
        assert_eq!(parse_intent(sent), Intent::Unknown, "{sent}");
        assert!(is_readback(sent), "{sent}");
        // The next call ("ready for departure, for circuits") gets its
        // takeoff answer, never the zone.
        let r = twr.handle(parse_intent("ready for departure, for circuits"), &c);
        assert!(!r.contains("leave the"), "{r}");
    }
    // Without circuits the zone clearance is as it was.
    let mut twr = Tower::new();
    let c = Ctx {
        circuit: None,
        ..circuit_ctx(false)
    };
    let r = twr.handle(Intent::RequestClearance, &c);
    assert!(
        r.contains("cleared to leave the Gibraltar zone, not above two thousand"),
        "{r}"
    );
}

/// #182: apt.dat publishes the pattern side per runway (`1101 13 left`
/// at LEMG) and whether the field allows circuits at all (`1302
/// allows_circuits 0` at LEMG and EGKK). The clearance names the side and
/// its readback is still a readback, never a request. A field without
/// circuits DECLINES the first ask and clears nothing in that breath (the
/// tower said "circuits are not normally available at Malaga" and then
/// gave the height, the code and "report ready" anyway, the realism
/// review, 2026-10-09).
#[test]
fn a_circuit_clearance_names_the_published_side_and_a_field_without_circuits() {
    for (faa, left, line, rb) in [
        (
            false,
            Some(true),
            "November one seven two Sierra Papa, runway zero niner, left hand circuit, \
                 circuit height one thousand one hundred feet, QNH one zero one niner, squawk \
                 four five zero one, report ready for departure.",
            "Runway zero niner, left hand circuit, circuit height one thousand one hundred \
                 feet, QNH one zero one niner, squawk four five zero one, November one seven \
                 two Sierra Papa.",
        ),
        (
            true,
            Some(false),
            "November one seven two Sierra Papa, runway zero niner, right closed traffic \
                 approved, pattern altitude one thousand one hundred, altimeter three zero zero \
                 one, squawk four five zero one, report ready for departure.",
            "Right closed traffic runway zero niner, pattern altitude one thousand one \
                 hundred, altimeter three zero zero one, squawk four five zero one, November \
                 one seven two Sierra Papa.",
        ),
    ] {
        let offer = CircuitOffer {
            height_ft: circuit_height_ft(52.0),
            faa,
            left,
            not_available: false,
        };
        let c = Ctx {
            circuit: Some(offer.clone()),
            ..circuit_ctx(faa)
        };
        let mut twr = Tower::new();
        assert_eq!(twr.handle(Intent::RequestClearance, &c), line);
        let owed = twr.suggested_readback(&c.callsign).expect("readback");
        assert_eq!(owed, rb);
        let sent = owed.trim_end_matches('.');
        assert_eq!(parse_intent(sent), Intent::Unknown, "{sent}");
        assert!(is_readback(sent), "{sent}");
        assert!(!line.contains('-') && !rb.contains('-'));
        // The same ask at a field without circuits: declined, the field's
        // offer named, nothing cleared, no code, no readback owed.
        let c = Ctx {
            circuit: Some(CircuitOffer {
                not_available: true,
                ..offer
            }),
            ..circuit_ctx(faa)
        };
        let mut twr = Tower::new();
        let r = twr.handle(Intent::RequestClearance, &c);
        let want = if faa {
            "November one seven two Sierra Papa, unable closed traffic, Gibraltar is \
                 departures and arrivals only. Advise a departure or a VFR route."
        } else {
            "November one seven two Sierra Papa, circuits are not available at Gibraltar. \
                 Departure from the zone or a VFR route available, say intentions."
        };
        assert_eq!(r, want);
        for never in ["squawk", "circuit height", "pattern altitude", "report ready", "cleared"] {
            assert!(!r.contains(never), "{never}: {r}");
        }
        assert_eq!(twr.suggested_readback(&c.callsign), None);
        assert_eq!(twr.state, State::Parked);
    }
}

/// ICAO outside the UK says "circuit altitude"; CAP 413 "circuit height".
#[test]
fn circuit_height_is_a_uk_word() {
    let c = Ctx {
        uk: false,
        ..circuit_ctx(false)
    };
    let (line, rb) = circuit_clearance_lines(&c, c.circuit.as_ref().unwrap());
    assert!(line.contains("circuit altitude one thousand one hundred feet"), "{line}");
    assert!(rb.contains("circuit altitude"), "{rb}");
    assert!(!line.contains("circuit height"), "{line}");
}

/// ctx() on the ground at an ICAO field, offered an IFR clearance to
/// Edinburgh from runway 09 (with or without a SID).
fn ifr_ctx(sid: Option<&str>, faa: bool) -> Ctx {
    Ctx {
        on_ground: true,
        squawk: "four five two one".into(),
        ifr: Some(IfrOffer {
            clearance: IfrClearance {
                dest: "EGPH".into(),
                sid: sid.map(str::to_string),
                runway: "09".into(),
                initial_alt_ft: 5000,
                squawk: 4521,
            },
            dest_spoken: "Edinburgh".into(),
            faa,
            departure_freq: Some("one two zero decimal four".into()),
        }),
        ..ctx()
    }
}

fn ifr_request() -> Intent {
    Intent::RequestIfrClearance {
        dest: Some("EGPH".into()),
    }
}

#[test]
fn the_ifr_initial_altitude_is_five_thousand_or_field_plus_three_rounded_up() {
    assert_eq!(ifr_initial_altitude_ft(136.0), 5000);
    assert_eq!(ifr_initial_altitude_ft(-11.0), 5000);
    assert_eq!(ifr_initial_altitude_ft(1906.0), 5000);
    assert_eq!(ifr_initial_altitude_ft(2001.0), 6000);
    assert_eq!(ifr_initial_altitude_ft(5582.0), 9000);
    assert_eq!(ifr_initial_altitude_ft(7000.0), 10000);
}

#[test]
fn a_sid_ident_is_spoken_the_radio_way() {
    assert_eq!(spoken_sid("GOSA1C"), "Gosa one Charlie");
    assert_eq!(spoken_sid("LEGL1P"), "Legl one Papa");
    assert_eq!(spoken_sid("BIG2F"), "Big two Foxtrot");
    assert_eq!(spoken_sid("RADAR"), "Radar");
}

#[test]
fn the_ifr_clearance_is_phrased_in_the_fields_dialect() {
    // ICAO / CAP 413 with a SID.
    let mut twr = Tower::new();
    let r = twr.handle(ifr_request(), &ifr_ctx(Some("GOSA1C"), false));
    assert_eq!(
        r,
        "November one seven two Sierra Papa, cleared to Edinburgh via the Gosa one \
             Charlie departure, runway zero niner, climb five thousand feet, squawk four \
             five two one, QNH one zero one seven."
    );
    assert!(twr.ifr_readback_owed(), "the readback is enforced");
    assert_eq!(
        twr.suggested_readback("November one seven two Sierra Papa").as_deref(),
        Some(
            "Cleared to Edinburgh via the Gosa one Charlie departure, runway zero niner, \
                 climb five thousand feet, squawk four five two one, November one seven two \
                 Sierra Papa."
        )
    );
    assert_eq!(twr.state, State::Parked, "a route clearance moves no runway state");
    // ICAO with no SID from this runway: runway heading, no invented name.
    let mut twr = Tower::new();
    let r = twr.handle(ifr_request(), &ifr_ctx(None, false));
    assert!(r.contains("cleared to Edinburgh, runway zero niner, after departure fly runway heading, climb five thousand feet, squawk four five two one"), "{r}");
    assert!(!r.contains("departure,") && !r.contains(" via "), "{r}");
    // FAA with a SID: "then as filed", "climb and maintain", the frequency.
    let mut twr = Tower::new();
    let r = twr.handle(ifr_request(), &ifr_ctx(Some("BIG2F"), true));
    assert_eq!(
        r,
        "November one seven two Sierra Papa, cleared to Edinburgh airport via the Big \
             two Foxtrot departure, then as filed, climb and maintain five thousand, departure \
             frequency one two zero decimal four, squawk four five two one."
    );
    // FAA with no SID.
    let mut twr = Tower::new();
    let r = twr.handle(ifr_request(), &ifr_ctx(None, true));
    assert!(
        r.contains("cleared to Edinburgh airport as filed, after departure fly runway heading, climb and maintain five thousand"),
        "{r}"
    );
    // A destination the station could not find, or none: asked, never cleared.
    let mut twr = Tower::new();
    let r = twr.handle(ifr_request(), &Ctx { on_ground: true, ..ctx() });
    assert_eq!(r, "November one seven two Sierra Papa, say again destination.");
    assert!(!twr.ifr_readback_owed());
    let r = twr.handle(Intent::RequestIfrClearance { dest: None }, &ctx());
    assert_eq!(r, "November one seven two Sierra Papa, say destination.");
    assert!(twr.pending().is_none());
}

#[test]
fn the_ifr_readback_is_enforced_item_by_item() {
    let cs = "November one seven two Sierra Papa";
    let c = ifr_ctx(Some("GOSA1C"), false);
    let issue = || {
        let mut twr = Tower::new();
        twr.handle(ifr_request(), &c);
        twr
    };
    // Correct: silence, and the clearance is there to publish.
    let mut twr = issue();
    let rb = twr.suggested_readback(cs).unwrap();
    let heard = parse_intent(&rb.to_lowercase());
    assert_eq!(twr.gate(&rb.to_lowercase(), heard, &c), Some(String::new()));
    assert_eq!(
        twr.take_accepted_ifr(),
        Some(IfrClearance {
            dest: "EGPH".into(),
            sid: Some("GOSA1C".into()),
            runway: "09".into(),
            initial_alt_ft: 5000,
            squawk: 4521,
        })
    );
    assert!(twr.pending().is_none());
    // Whisper's spellings count: a written altitude and code, the
    // designator without the name.
    let mut twr = issue();
    let t = "cleared to edinburgh, 1c departure, climb 5,000 feet, squawk 4521";
    assert_eq!(twr.gate(t, parse_intent(t), &c), Some(String::new()));
    assert!(twr.take_accepted_ifr().is_some());
    // Wrong altitude: "negative" and the right one; still owed, no
    // clearance, and wrong items spend no challenge.
    let mut twr = issue();
    let t = "cleared to edinburgh via the gosa one charlie departure, climb six thousand feet, squawk four five two one";
    for _ in 0..4 {
        let r = twr.gate(t, parse_intent(t), &c).expect("corrected");
        assert_eq!(r, format!("{cs}, negative, climb five thousand feet."));
    }
    assert!(twr.take_accepted_ifr().is_none());
    assert!(twr.ifr_readback_owed());
    // Missing squawk: asked for, still owed.
    let mut twr = issue();
    let t = "cleared to edinburgh via the gosa one charlie departure, climb five thousand feet";
    let r = twr.gate(t, parse_intent(t), &c).expect("asked");
    assert_eq!(r, format!("{cs}, read back squawk four five two one."));
    assert!(twr.take_accepted_ifr().is_none());
    assert!(twr.ifr_readback_owed());
    // ...then read back right: accepted.
    let t = "squawk four five two one, climb five thousand, gosa one charlie";
    assert_eq!(twr.gate(t, parse_intent(t), &c), Some(String::new()));
    assert!(twr.take_accepted_ifr().is_some());
    // Wrong squawk AND a missing SID: both named.
    let mut twr = issue();
    let t = "cleared to edinburgh, climb five thousand feet, squawk four five two two";
    let r = twr.gate(t, parse_intent(t), &c).unwrap();
    assert_eq!(
        r,
        format!("{cs}, negative, squawk four five two one. Read back the Gosa one Charlie departure.")
    );
    // A wrong runway in the echo is caught too.
    let mut twr = issue();
    let t = "cleared to edinburgh via gosa one charlie runway two seven, climb five thousand, squawk 4521";
    let r = twr.gate(t, parse_intent(t), &c).unwrap();
    assert!(r.contains("negative, runway zero niner"), "{r}");
    // A mishear is never the readback: challenged, twice, then let go
    // with NOTHING published. It never earns the clearance.
    let mut twr = issue();
    let r = twr
        .gate("gibraltar tower, say again", Intent::Unknown, &c)
        .expect("challenged");
    assert!(r.starts_with(&format!("{cs}, I say again, cleared to Edinburgh")), "{r}");
    let t = "blah blah weather";
    let r = twr.gate(t, parse_intent(t), &c).expect("challenged");
    assert_eq!(r, format!("{cs}, read back."));
    let t = "mumble";
    assert_eq!(twr.gate(t, parse_intent(t), &c), None, "let through after two");
    assert!(twr.take_accepted_ifr().is_none(), "a mishear earned a clearance");
    assert!(!twr.ifr_readback_owed());
    // Without a SID only the altitude and the squawk are owed.
    let c = ifr_ctx(None, false);
    let mut twr = Tower::new();
    twr.handle(ifr_request(), &c);
    let t = "cleared to edinburgh, runway heading, climb five thousand feet, squawk four five two one";
    assert_eq!(twr.gate(t, parse_intent(t), &c), Some(String::new()));
    assert_eq!(twr.take_accepted_ifr().unwrap().sid, None);
    // Enforcement off: any readback of the clearance stands.
    let c = ifr_ctx(Some("GOSA1C"), true);
    let mut twr = Tower::new();
    twr.handle(ifr_request(), &c);
    twr.accept_ifr_unenforced();
    assert!(twr.take_accepted_ifr().is_some());
}

/// Issue #97: an initial altitude at or above the field's transition
/// LEVEL is a flight level, in the clearance, the coach's readback and
/// the correction; below it, feet with the QNH. The readback checker
/// takes either form for the same number.
#[test]
fn a_level_at_or_above_the_transition_level_is_a_flight_level() {
    let cs = "November one seven two Sierra Papa";
    // A field whose transition level is 5,000 ft (apt.dat says FL050):
    // the 5,000 ft initial altitude is flight level five zero.
    let c = Ctx {
        transition_level_ft: Some(5000.0),
        ..ifr_ctx(Some("GOSA1C"), false)
    };
    let mut twr = Tower::new();
    let r = twr.handle(ifr_request(), &c);
    assert!(r.contains("runway zero niner, climb flight level five zero, squawk"), "{r}");
    assert!(!r.contains("feet"), "{r}");
    let rb = twr.suggested_readback(cs).unwrap();
    assert!(rb.contains("climb flight level five zero, squawk four five two one"), "{rb}");
    assert_eq!(twr.gate(&rb.to_lowercase(), parse_intent(&rb.to_lowercase()), &c), Some(String::new()));
    // A wrong level is corrected in flight-level words.
    let mut twr = Tower::new();
    twr.handle(ifr_request(), &c);
    let t = "cleared to edinburgh via the gosa one charlie departure, flight level six zero, squawk four five two one";
    assert_eq!(
        twr.gate(t, parse_intent(t), &c).unwrap(),
        format!("{cs}, negative, climb flight level five zero.")
    );
    // The same number said in feet is the same clearance.
    let t = "gosa one charlie, climb five thousand feet, squawk four five two one";
    assert_eq!(twr.gate(t, parse_intent(t), &c), Some(String::new()));
    // FAA: "climb and maintain flight level".
    let c = Ctx {
        transition_level_ft: Some(5000.0),
        ..ifr_ctx(Some("BIG2F"), true)
    };
    let r = Tower::new().handle(ifr_request(), &c);
    assert!(r.contains("climb and maintain flight level five zero, departure frequency"), "{r}");
    // Below the transition level (EGKK's is 7,000): feet, as before.
    let c = Ctx {
        transition_level_ft: Some(7000.0),
        ..ifr_ctx(Some("GOSA1C"), false)
    };
    let r = Tower::new().handle(ifr_request(), &c);
    assert!(r.contains("climb five thousand feet, squawk"), "{r}");
}

/// Issue #128: a runway start with too little runway ahead asks to
/// backtrack; the tower lines it up down the strip when the strip and
/// its final are clear, holds it when not, and a parked aircraft asking
/// gets its taxi first.
#[test]
fn a_backtrack_is_a_line_up_from_the_runway() {
    let cs = "November one seven two Sierra Papa";
    for t in [
        "gatwick tower, whiskey alpha tango one seven three, request backtrack runway two six left",
        "requesting backtrack for full length",
        "request back track runway zero niner",
    ] {
        assert_eq!(parse_intent(t), Intent::RequestBacktrack, "{t}");
    }
    // The approval's readback, and plain chatter, are not requests.
    assert_eq!(
        parse_intent("backtracking runway two six left, line up and wait, whiskey alpha tango one seven three"),
        Intent::Unknown
    );
    assert_ne!(parse_intent("we could backtrack"), Intent::RequestBacktrack);
    let on_strip = Ctx {
        on_ground: true,
        own_on_runway: true,
        ..ctx()
    };
    let mut twr = Tower::new();
    let r = twr.handle(Intent::RequestBacktrack, &on_strip);
    assert_eq!(r, format!("{cs}, backtrack approved runway zero niner, line up and wait."));
    assert_eq!(twr.state, State::LinedUp);
    assert!(twr.holds_runway());
    let rb = twr.suggested_readback(cs).unwrap();
    let t = rb.to_lowercase();
    assert_eq!(twr.gate(&t, parse_intent(&t), &on_strip), Some(String::new()));
    // Then "ready for departure" from the threshold is a lined-up call.
    let r = twr.handle(Intent::RequestTakeoff, &on_strip);
    assert!(r.contains("cleared for takeoff"), "{r}");
    // Traffic on final or on the strip: hold, still Parked.
    let mut twr = Tower::new();
    let r = twr.handle(
        Intent::RequestBacktrack,
        &Ctx {
            final_traffic_nm: Some(3.0),
            ..on_strip.clone()
        },
    );
    assert!(r.contains("hold position, traffic on 3 mile final"), "{r}");
    assert_eq!(twr.state, State::Parked);
    let r = Tower::new().handle(
        Intent::RequestBacktrack,
        &Ctx {
            runway_occupied: true,
            ..on_strip.clone()
        },
    );
    assert!(r.contains("hold position, traffic on the runway"), "{r}");
    // At the hold on a taxi clearance: the same approval.
    let mut twr = Tower::new();
    twr.state = State::TaxiCleared;
    let r = twr.handle(Intent::RequestBacktrack, &Ctx { on_ground: true, ..ctx() });
    assert!(r.contains("backtrack approved"), "{r}");
    // From the stand: the taxi first, no runway.
    let mut twr = Tower::new();
    let r = twr.handle(Intent::RequestBacktrack, &Ctx { on_ground: true, ..ctx() });
    assert!(r.contains("taxi to holding point Alpha"), "{r}");
    assert_eq!(twr.state, State::TaxiCleared);
    // Airborne: a garble.
    let r = Tower::new().handle(Intent::RequestBacktrack, &ctx());
    assert_eq!(r, format!("{cs}, say again."));
}

#[test]
fn a_flight_level_is_heard_in_every_spelling() {
    assert_eq!(heard_altitude_ft("climb flight level one two zero"), Some(12000));
    assert_eq!(heard_altitude_ft("climbing flight level 120"), Some(12000));
    assert_eq!(heard_altitude_ft("climb fl120"), Some(12000));
    assert_eq!(heard_altitude_ft("climb FL 90"), Some(9000));
    assert_eq!(heard_altitude_ft("level seven zero"), Some(7000));
    assert_eq!(
        heard_altitude_ft("flight level one two zero, squawk four five two one"),
        Some(12000)
    );
    assert_eq!(heard_altitude_ft("squawk four five two one"), None);
    assert_eq!(heard_altitude_ft("level off at five thousand"), Some(5000));
    assert_eq!(crate::atis::spoken_flight_level(7000.0), "flight level seven zero");
    assert_eq!(crate::atis::spoken_flight_level(35000.0), "flight level three five zero");
    assert!(crate::atis::is_flight_level(7000.0, Some(7000.0)));
    assert!(!crate::atis::is_flight_level(6000.0, Some(7000.0)));
    assert!(!crate::atis::is_flight_level(20000.0, None), "unknown: feet");
    assert_eq!(crate::nav::parse_transition_level("7000"), Some(7000.0));
    assert_eq!(crate::nav::parse_transition_level("FL070"), Some(7000.0));
    assert_eq!(crate::nav::parse_transition_level("180"), Some(18000.0));
    assert_eq!(crate::nav::parse_transition_level("By"), None);
    assert_eq!(crate::nav::parse_transition_level("0"), None);
}

#[test]
fn readback_numbers_parse_the_way_whisper_writes_them() {
    assert_eq!(heard_altitude_ft("climb five thousand feet"), Some(5000));
    assert_eq!(heard_altitude_ft("climb to five thousand"), Some(5000));
    assert_eq!(heard_altitude_ft("climb and maintain one zero thousand"), Some(10000));
    assert_eq!(heard_altitude_ft("climb 5,000 feet"), Some(5000));
    assert_eq!(heard_altitude_ft("climb 6000"), Some(6000));
    assert_eq!(heard_altitude_ft("twelve thousand"), Some(12000));
    assert_eq!(heard_altitude_ft("four thousand five hundred"), Some(4500));
    assert_eq!(heard_altitude_ft("squawk four five two one"), None, "a code is no altitude");
    assert_eq!(heard_altitude_ft("squawk 4521"), None);
    assert_eq!(heard_squawk("squawk four five two one"), Some(4521));
    assert_eq!(heard_squawk("squawk 4521"), Some(4521));
    assert_eq!(heard_squawk("squawk niner seven one zero"), Some(9710));
    assert_eq!(heard_squawk("squawk four five"), None);
    assert!(heard_sid("via the gosam one charlie", "GOSA1C"));
    assert!(heard_sid("gosa 1c departure", "GOSA1C"));
    assert!(heard_sid("the one c departure", "GOSA1C"));
    assert!(!heard_sid("the one delta departure", "GOSA1C"));
    assert!(!heard_sid("climb five thousand", "GOSA1C"));
}

#[test]
fn ifr_requests_parse_and_the_zone_clearance_stands() {
    let ifr = |d: Option<&str>| Intent::RequestIfrClearance {
        dest: d.map(str::to_string),
    };
    assert_eq!(parse_intent("request ifr clearance to egph"), ifr(Some("EGPH")));
    assert_eq!(parse_intent("request clearance to edinburgh"), ifr(Some("edinburgh")));
    assert_eq!(parse_intent("ifr to edinburgh, ready to copy"), ifr(Some("edinburgh")));
    assert_eq!(parse_intent("request clearance as filed"), ifr(None));
    assert_eq!(parse_intent("ready to copy"), ifr(None));
    assert_eq!(
        parse_intent("request ifr clearance to kilo oscar romeo delta"),
        ifr(Some("KORD"))
    );
    assert_eq!(parse_intent("request ifr clearance to ord"), ifr(Some("ORD")));
    // Nothing existing changes its answer.
    assert_eq!(parse_intent("request clearance"), Intent::RequestClearance);
    assert_eq!(parse_intent("request departure clearance"), Intent::RequestClearance);
    assert_eq!(parse_intent("request clearance to leave the zone"), Intent::RequestClearance);
    assert_eq!(parse_intent("request clearance to start engines"), Intent::RequestStartup);
    assert_eq!(parse_intent("request clearance to parking"), Intent::RequestTaxiIn);
    assert_eq!(parse_intent("request clearance for taxi"), Intent::RequestTaxi);
}

/// #83: a transit field's "say callsign and intentions" is answered with
/// a transit request, and the tower clears it through the zone without
/// touching the sequence. A runway crossing, the readback and every
/// existing request keep their answers.
#[test]
fn a_zone_transit_is_cleared_through_and_changes_nothing() {
    for call in [
        "Hameln Tower, Delta Hotel Xray, S-76 helicopter, two thousand two hundred feet, request zone transit",
        "request transit of your zone northbound",
        "transiting your control zone to Diepholz",
        "request clearance to transit the zone",
        "request to cross your zone",
        "passing through the zone, two thousand feet",
    ] {
        assert_eq!(parse_intent(call), Intent::RequestTransit, "{call}");
    }
    assert_ne!(parse_intent("request to cross runway two seven"), Intent::RequestTransit);
    assert_eq!(
        parse_intent("cleared to transit the zone, report leaving"),
        Intent::Unknown,
        "the readback is not a request"
    );
    assert_eq!(parse_intent("can we transit your zone"), Intent::Unknown);
    assert_eq!(parse_intent("request clearance to leave the zone"), Intent::RequestClearance);
    assert_eq!(parse_intent("inbound for landing"), Intent::RequestLanding { touch_and_go: false });
    assert_eq!(intent_from_label("request_transit"), Some(Intent::RequestTransit));
    for state in [State::Parked, State::Inbound, State::TakeoffCleared] {
        let mut twr = Tower::new();
        twr.state = state;
        let r = twr.handle(Intent::RequestTransit, &ctx());
        assert_eq!(twr.state, state, "{state:?}: {r}");
        assert!(r.starts_with("November one seven two Sierra Papa, cleared to transit the Gibraltar zone"), "{r}");
        assert!(r.contains("QNH one zero one seven") && r.contains("report leaving the zone"), "{r}");
    }
    let faa = Ctx {
        qnh: "altimeter three zero zero one".into(),
        dialect: crate::atis::Dialect::Faa,
        ..ctx()
    };
    let r = Tower::new().handle(Intent::RequestTransit, &faa);
    assert!(r.contains("transition through the Gibraltar airspace approved"), "{r}");
    // The words are the dialect's even when the pressure is blank (the
    // pilot holds the ATIS): they rode the pressure phrase until
    // 2026-10-09, and an FAA field then spoke ICAO.
    let r = Tower::new().handle(
        Intent::RequestTransit,
        &Ctx {
            qnh: String::new(),
            ..faa.clone()
        },
    );
    assert!(r.contains("transition through the Gibraltar airspace approved"), "{r}");
    assert!(!r.contains("altimeter") && !r.contains("zone"), "{r}");
}

/// ctx() with one arrival on final at `nm`.
fn busy(nm: f64) -> Ctx {
    Ctx {
        final_traffic_nm: Some(nm),
        final_traffic_callsign: Some("Speedbird three zero six".into()),
        final_traffic_count: 1,
        ..ctx()
    }
}

#[test]
fn intents_parse_from_natural_calls() {
    assert_eq!(
        parse_intent("Gibraltar Tower, November 172SP, request taxi"),
        Intent::RequestTaxi
    );
    assert_eq!(
        parse_intent("ready for departure runway 09"),
        Intent::RequestTakeoff
    );
    assert_eq!(parse_intent("request takeoff"), Intent::RequestTakeoff);
    assert_eq!(
        parse_intent("request departure clearance"),
        Intent::RequestClearance
    );
    assert_eq!(parse_intent("radio check"), Intent::RadioCheck);
    // A bare hail is an initial call, answered "pass your message" —
    // not "say again" (2026-07-10: "Good evening, Merida").
    assert_eq!(parse_intent("good morning"), Intent::Greeting);
    assert_eq!(parse_intent("good evening, merida"), Intent::Greeting);
    assert_eq!(parse_intent("hello, banahos"), Intent::Greeting);
    // Greeting words never outrank a real request in the same breath.
    assert_eq!(
        parse_intent("good morning, request taxi"),
        Intent::RequestTaxi
    );
    // Flight following, straight and through whisper's mangle.
    assert_eq!(
        parse_intent("request flight following"),
        Intent::RequestFlightFollowing
    );
    assert_eq!(
        parse_intent("but a host tower requesting flight following"),
        Intent::RequestFlightFollowing
    );
    // "taxi" inside a takeoff call must not read as a taxi request.
    assert_eq!(
        parse_intent("holding point taxi complete, ready for departure"),
        Intent::RequestTakeoff
    );
    // Cockpit commands containing "taxi" are NOT radio calls — in the
    // merged binary this parser sees every utterance the pilot speaks.
    assert_eq!(parse_intent("taxi light on"), Intent::Unknown);
    assert_eq!(parse_intent("release the taxi hold"), Intent::Unknown);
    // Arrival calls.
    assert_eq!(
        parse_intent("inbound for landing"),
        Intent::RequestLanding {
            touch_and_go: false
        }
    );
    assert_eq!(
        parse_intent("request clearance for landing"),
        Intent::RequestLanding {
            touch_and_go: false
        },
        "landing beats the departure-clearance keyword"
    );
    assert_eq!(
        parse_intent("request touch and go"),
        Intent::RequestLanding { touch_and_go: true }
    );
    assert_eq!(
        parse_intent("november one seven two, final"),
        Intent::ReportFinal
    );
    // "Established" is the final report by other words — the ILS
    // arrival's version of it, to tower (clearance) and approach
    // (handoff) alike. Whisper often splits the acronym.
    assert_eq!(
        parse_intent("november one seven two, established i l s runway zero niner"),
        Intent::ReportFinal
    );
    assert_eq!(parse_intent("localizer established"), Intent::ReportFinal);
    // The crew's question on an uncleared final (autoland's
    // `UnclearedWatch`, #155) is a final report, judged afresh against
    // the strip, never a landing request that answers "join final".
    assert_eq!(
        parse_intent(
            "whiskey alpha tango six two six, final runway one five, request landing clearance"
        ),
        Intent::ReportFinal
    );
    // But a cockpit question about it stays crew talk.
    assert_eq!(
        parse_intent("how do I know when we're established"),
        Intent::Unknown
    );
    assert_eq!(
        parse_intent("request vectors for the approach"),
        Intent::RequestVectors
    );
    assert_eq!(parse_intent("say wind"), Intent::WindCheck);
    assert_eq!(parse_intent("wind check"), Intent::WindCheck);
    // Cockpit vocabulary that must NOT read as arrival calls.
    assert_eq!(parse_intent("landing light on"), Intent::Unknown);
    assert_eq!(parse_intent("landing gear down"), Intent::Unknown);
    assert_eq!(
        parse_intent("what is our final approach speed"),
        Intent::Unknown
    );
    // A callsign holding a cockpit word is still a final report (EGKK).
    assert_eq!(
        parse_intent("Speedbird two zero eight five, final runway two six left"),
        Intent::ReportFinal
    );
    // Questions are crew talk, never clearance requests — asking "what
    // runway shall I take off from" once earned a real takeoff clearance.
    assert_eq!(
        parse_intent("What runway shall I take off from?"),
        Intent::Unknown
    );
    assert_eq!(
        parse_intent("should we request takeoff now"),
        Intent::Unknown
    );
    assert_eq!(parse_intent("can we land at gibraltar"), Intent::Unknown);
    // The question-shaped calls a tower genuinely answers survive.
    assert_eq!(parse_intent("how do you read"), Intent::RadioCheck);
    assert_eq!(parse_intent("what's the wind"), Intent::WindCheck);
    // Squawk questions get the code repeated, not "say again" — asked four
    // times on the 2026-07-08 flights, stonewalled every time.
    assert_eq!(
        parse_intent("What about my transponder code?"),
        Intent::SquawkCheck
    );
    assert_eq!(parse_intent("say squawk"), Intent::SquawkCheck);
    assert_eq!(parse_intent("what's my squawk"), Intent::SquawkCheck);
    // A readback of the code is still a readback, not a question.
    assert_eq!(parse_intent("squawk seven zero zero zero"), Intent::Unknown);
    assert!(is_readback("squawk seven zero zero zero"));
    // Arrivals taxi IN — never a fresh departure taxi.
    assert_eq!(
        parse_intent("request taxi to parking"),
        Intent::RequestTaxiIn
    );
    assert_eq!(parse_intent("taxi to the ramp"), Intent::RequestTaxiIn);
    // "I've just landed" is an arrival on the runway, not "say again".
    assert_eq!(parse_intent("i've just landed"), Intent::RequestTaxiIn);
    assert_eq!(parse_intent("we have landed"), Intent::RequestTaxiIn);
    // Off the strip, reported. The crew's own call, twice answered "say
    // again" at Liege (2026-09-22).
    assert_eq!(parse_intent("runway vacated"), Intent::RunwayVacated);
    assert_eq!(
        parse_intent("november eight one six november november, runway vacated"),
        Intent::RunwayVacated
    );
    assert_eq!(
        parse_intent("vacated runway zero four right"),
        Intent::RunwayVacated
    );
    assert_eq!(
        parse_intent("we're clear of the runway"),
        Intent::RunwayVacated
    );
    assert_eq!(
        parse_intent("vacating via sierra three"),
        Intent::RunwayVacated
    );
    // ...but a call that also ASKS is that request. The station's own
    // suggested wording is the combined one, and it must still earn the
    // taxi, not an acknowledgement.
    assert_eq!(
        parse_intent("runway vacated, request taxi to parking"),
        Intent::RequestTaxiIn
    );
    assert_eq!(
        parse_intent("runway vacated, request taxi"),
        Intent::RequestTaxi,
        "a bare taxi request is still a taxi request; the FSM infers the \
             taxi IN from the landing clearance"
    );
    // A landing clearance read back is not a vacate report: "cleared"
    // contains "clear" and the two calls are seconds apart on the tape.
    assert_ne!(
        parse_intent("cleared to land runway zero four right"),
        Intent::RunwayVacated
    );
    // "request clearance for taxi" wants the taxi, not the zone clearance
    // (asked three times on 2026-07-08; each got "cleared to leave the
    // zone" while parked at the stand).
    assert_eq!(
        parse_intent("request clearance for taxi"),
        Intent::RequestTaxi
    );
    assert_eq!(
        parse_intent("requesting clearance for taxi"),
        Intent::RequestTaxi
    );
    assert_eq!(
        parse_intent("request departure clearance"),
        Intent::RequestClearance,
        "clearance without taxi words is still the zone clearance"
    );
    // The word "clearance" must not swallow an engine-start request
    // (real 2026-07-12 call, answered with the zone clearance while
    // parked, engines off).
    assert_eq!(
        parse_intent("request clearance to start engines."),
        Intent::RequestStartup
    );
    assert_eq!(parse_intent("requesting startup"), Intent::RequestStartup);
    assert_eq!(parse_intent("request start-up"), Intent::RequestStartup);
    assert_eq!(
        parse_intent("malaga tower, november five five zero charlie romeo, request start up"),
        Intent::RequestStartup
    );
    // ...and it must not swallow a PARKED arrival either — all four are
    // real 2026-07-13 Gibraltar calls; the first two were answered
    // "cleared to leave the zone" plus a squawk, ending in "no, no, no,
    // no" from the pilot.
    assert_eq!(
        parse_intent("request clearance to parking."),
        Intent::RequestTaxiIn
    );
    assert_eq!(
        parse_intent("requesting clearance to switch engines off."),
        Intent::RequestShutdown
    );
    assert_eq!(
        parse_intent("where shall i park at?"),
        Intent::RequestTaxiIn,
        "a parking question is one the tower genuinely answers"
    );
    assert_eq!(
        parse_intent("no, i said i want to park the aircraft."),
        Intent::RequestTaxiIn
    );
    assert_eq!(parse_intent("request shutdown"), Intent::RequestShutdown);
    // The parking-brake checklist item stays a cockpit command.
    assert_eq!(parse_intent("set the parking brake"), Intent::Unknown);
    // "do you copy/read/hear me" / "are you there" are radio checks — five
    // real calls across the 2026-07 flights, all stonewalled.
    assert_eq!(
        parse_intent("gibraltar tower, do you copy"),
        Intent::RadioCheck
    );
    assert_eq!(
        parse_intent("malaga tower, do you hear me?"),
        Intent::RadioCheck
    );
    assert_eq!(
        parse_intent("chipotle tower, are you there?"),
        Intent::RadioCheck
    );
    assert_eq!(parse_intent("how's the wind looking"), Intent::WindCheck);
    // Checklist call-outs are cockpit ritual, never radio calls.
    assert_eq!(parse_intent("takeoff checklist"), Intent::Unknown);
    assert_eq!(parse_intent("after landing checklist"), Intent::Unknown);
    assert_eq!(parse_intent("flaps set for landing"), Intent::Unknown);
    // "holding point" survives the taxi-hold guard; the governor commands
    // still don't.
    assert_eq!(
        parse_intent("request taxi to holding point alpha"),
        Intent::RequestTaxi
    );
    assert_eq!(parse_intent("release the taxi hold"), Intent::Unknown);
    // "transponder code" alone deserves the code (real 2026-07-08 call).
    assert_eq!(parse_intent("transponder code"), Intent::SquawkCheck);
}

/// Every (state, intent) cell of the FSM, exhaustively: the exact end
/// state, plus the invariants a tower must never break anywhere in the
/// grid. The named tests above cover the flown paths; this covers the
/// cells nobody has flown yet.
#[test]
fn state_intent_matrix_holds_the_invariants() {
    use Intent::*;
    use State::*;
    let states = [
        Parked,
        TaxiCleared,
        LinedUp,
        TakeoffCleared,
        Inbound,
        ClearedToLand,
    ];
    let intents = [
        RadioCheck,
        RequestTaxi,
        RequestClearance,
        RequestIfrClearance {
            dest: Some("EGPH".into()),
        },
        RequestIfrClearance { dest: None },
        RequestStartup,
        RequestShutdown,
        RequestTakeoff,
        RequestLanding {
            touch_and_go: false,
        },
        RequestLanding { touch_and_go: true },
        ReportFinal,
        ReportDownwind,
        ReportBase,
        ReportCrosswind,
        GoAround,
        RejectingTakeoff,
        Mayday,
        WindCheck,
        SquawkCheck,
        RequestTaxiIn,
        RunwayVacated,
        RequestRunway,
        RequestBacktrack,
        RequestCrossing,
        CancelIfr,
        RequestSid,
        RequestVectors,
        Greeting,
        RequestFlightFollowing,
        RequestFrequencyChange,
        RequestTransit,
        Unknown,
    ];
    // The plain flight, and circuit traffic in both dialects (#148),
    // airborne like ctx() so the table below holds for all three.
    let airborne = |faa| Ctx {
        on_ground: false,
        ..circuit_ctx(faa)
    };
    for c in [ctx(), airborne(false), airborne(true)] {
        for &start in &states {
            for intent in intents.iter().cloned() {
                let mut twr = Tower::new();
                twr.state = start;
                let reply = twr.handle(intent.clone(), &c);
                let end = twr.state;
                let cell = format!("{start:?} + {intent:?} (circuit {:?})", c.circuit);

                // Circuit traffic is never sent out of the zone, and its
                // clearance names the runway and the code.
                if c.circuit.is_some() {
                    assert!(!reply.contains("leave the"), "{cell}: {reply}");
                    if intent == RequestClearance {
                        assert!(reply.contains(&c.runway), "{cell}: {reply}");
                        assert!(reply.contains(&c.squawk), "{cell}: {reply}");
                    }
                }

                // A controller always answers, and always by callsign.
                assert!(!reply.is_empty(), "{cell}: silent");
                assert!(
                    reply.contains(&c.callsign),
                    "{cell}: no callsign in {reply:?}"
                );

                // A takeoff clearance is only ever spoken to an aircraft that
                // asked for one FROM the holding-point states — never handed
                // out by any other cell of the grid.
                if reply.contains("cleared for takeoff") {
                    assert_eq!(intent, RequestTakeoff, "{cell}: unasked takeoff clearance");
                    assert!(
                        matches!(start, TaxiCleared | LinedUp | TakeoffCleared),
                        "{cell}: takeoff clearance from the wrong state"
                    );
                    assert!(
                        reply.contains(&c.runway),
                        "{cell}: clearance without runway"
                    );
                    assert!(reply.contains(&c.wind), "{cell}: clearance without wind");
                }
                // A landing clearance only ever answers a final report — or a
                // MAYDAY, the one deliberate exception: an emergency outranks
                // the sequence (priority handling is the point of the call).
                if reply.contains("cleared to land") || reply.contains("cleared touch and go") {
                    assert!(
                        matches!(intent, ReportFinal | Mayday),
                        "{cell}: unasked landing clearance"
                    );
                    assert!(
                        reply.contains(&c.runway),
                        "{cell}: clearance without runway"
                    );
                    assert!(reply.contains(&c.wind), "{cell}: clearance without wind");
                }

                // The full transition table — any change here must be a
                // deliberate edit to this test, not a surprise.
                let want = match intent {
                    RadioCheck | WindCheck | SquawkCheck | Unknown | RequestClearance
                    | RequestStartup | RequestShutdown => start,
                    // A reject is a ground report: ctx() is airborne, so it
                    // is a garble there and moves nothing (the ground half
                    // has its own test).
                    RejectingTakeoff => start,
                    // A backtrack is a ground request (ctx() is airborne: a
                    // garble) and a cancellation is the station's book.
                    RequestBacktrack | CancelIfr | RequestSid => start,
                    // A crossing is a ground request (a garble airborne), and
                    // on the ground it clears a crossing, not a state.
                    RequestCrossing => start,
                    // The IFR clearance clears a route, not a runway: the
                    // sequence stands (and with no offer it is "say again").
                    RequestIfrClearance { .. } => start,
                    // A hail, flight following, and radio housekeeping are
                    // answered without touching the sequence.
                    Greeting | RequestFlightFollowing | RequestFrequencyChange => start,
                    // A transit passes through: it neither departs nor
                    // lands here, and the sequence stands.
                    RequestTransit => start,
                    // A position report off the runway is acknowledged and
                    // changes nothing — see the handler.
                    RunwayVacated => start,
                    // A grantable request (ctx() carries no refusal): the
                    // taxi to it, or the join from the air.
                    RequestRunway => {
                        if c.on_ground {
                            TaxiCleared
                        } else {
                            Inbound
                        }
                    }
                    // ctx() has an approach frequency, so vectors redirect
                    // without touching the sequence.
                    RequestVectors => start,
                    RequestTaxiIn => Parked,
                    RequestTaxi => {
                        if start == ClearedToLand {
                            Parked
                        } else {
                            TaxiCleared
                        }
                    }
                    RequestTakeoff => {
                        if matches!(start, TaxiCleared | LinedUp | TakeoffCleared) {
                            TakeoffCleared
                        } else {
                            TaxiCleared
                        }
                    }
                    RequestLanding { .. } => Inbound,
                    ReportFinal => ClearedToLand,
                    // Pattern legs and a go-around all land in "in the
                    // pattern, awaiting final"; a mayday is cleared at once.
                    ReportDownwind | ReportBase | ReportCrosswind | GoAround => Inbound,
                    Mayday => ClearedToLand,
                };
                assert_eq!(end, want, "{cell}: wrong transition");
            }
        }
    }

    // With no approach service, a vectors request joins the pattern
    // instead — from every state.
    let mut no_app = ctx();
    no_app.approach = None;
    for &start in &states {
        let mut twr = Tower::new();
        twr.state = start;
        twr.handle(RequestVectors, &no_app);
        assert_eq!(
            twr.state, Inbound,
            "{start:?}: no-approach vectors must join"
        );
    }

    // A ghost ON the strip: a re-ask never repeats the takeoff clearance
    // from ANY of the committed states. Holding-point states hold where
    // they are; an already-cleared aircraft has its clearance CANCELLED
    // and lands in LinedUp (it may be on the pavement), with the release
    // watch owing the re-clearance.
    let mut occupied = ctx();
    occupied.runway_occupied = true;
    for &start in &[TaxiCleared, LinedUp, TakeoffCleared] {
        let mut twr = Tower::new();
        twr.state = start;
        let r = twr.handle(RequestTakeoff, &occupied);
        assert!(
            !r.contains("cleared for takeoff"),
            "{start:?}: cleared onto an occupied runway: {r}"
        );
        assert!(r.contains("hold position"), "{start:?}: {r}");
        if start == TakeoffCleared {
            assert!(r.contains("cancel takeoff clearance"), "{r}");
            assert_eq!(
                twr.state, LinedUp,
                "clearance withdrawn, still on the pavement"
            );
        } else {
            assert_eq!(twr.state, start, "{start:?}: hold moves nobody");
        }
    }
}

/// The reject is its own call, never a takeoff request: "rejecting
/// takeoff" carries the word "takeoff", and parsed as `RequestTakeoff` it
/// would ask for the clearance it is giving up. The cockpit's own "abort"
/// or "stop" is not it (unkeyed, those never reach the tower anyway).
#[test]
fn a_reject_call_parses_as_the_reject() {
    for call in [
        "whiskey alpha tango one seven three, rejecting takeoff",
        "Whiskey Alpha Tango one seven three, stopping.",
        "gatwick tower, easy two five five, stopping",
        "aborting takeoff, november one seven two sierra papa",
        "rejecting take off",
        "we rejected the take-off",
    ] {
        assert_eq!(parse_intent(call), Intent::RejectingTakeoff, "{call}");
    }
    for call in [
        "ready for departure",
        "cleared for takeoff runway two six left",
        "abort",
        "stop",
        "full stop landing",
        "why are we stopping",
    ] {
        assert_ne!(parse_intent(call), Intent::RejectingTakeoff, "{call}");
    }
    assert_eq!(
        intent_from_label("rejecting_takeoff"),
        Some(Intent::RejectingTakeoff)
    );
}

/// On the ground a reject ends the takeoff clearance: from TakeoffCleared
/// or LinedUp the flight is on the runway with none (LinedUp), nothing is
/// owed, and the answer is an acknowledgement, not an instruction to read
/// back. The owed takeoff readback never gates it (safety outranks
/// discipline, as for a go-around). Elsewhere on the ground it moves
/// nothing; in the air it is a garble.
#[test]
fn a_reject_cancels_the_takeoff_clearance() {
    let mut ground = ctx();
    ground.on_ground = true;
    for start in [State::TakeoffCleared, State::LinedUp] {
        let mut twr = Tower::new();
        twr.state = start;
        twr.arm_takeoff_readback(
            "zero niner".into(),
            "runway zero niner, cleared for takeoff.".into(),
        );
        assert_eq!(
            twr.gate(
                "november one seven two sierra papa, rejecting takeoff",
                Intent::RejectingTakeoff,
                &ground
            ),
            None,
            "{start:?}: never challenged"
        );
        let r = twr.handle(Intent::RejectingTakeoff, &ground);
        assert!(r.starts_with(&ground.callsign), "{r}");
        assert!(r.contains("roger") && r.contains("Hold position"), "{r}");
        assert!(!r.contains("cleared"), "{r}");
        assert_eq!(twr.state, State::LinedUp, "{start:?}");
        assert!(twr.pending().is_none(), "{start:?}: nothing owed");
        assert!(twr.holds_runway(), "{start:?}: still on the runway");
        // The resume's next call is answered on its own terms.
        let mut t2 = Tower::new();
        t2.state = twr.state;
        assert!(t2
            .handle(Intent::RequestTaxi, &ground)
            .contains("holding point"));
        assert!(twr
            .handle(Intent::RequestTakeoff, &ground)
            .contains("cleared for takeoff"));
    }
    for start in [State::Parked, State::TaxiCleared] {
        let mut twr = Tower::new();
        twr.state = start;
        let r = twr.handle(Intent::RejectingTakeoff, &ground);
        assert!(r.contains("roger"), "{r}");
        assert_eq!(twr.state, start);
    }
    let mut twr = Tower::new();
    twr.state = State::TakeoffCleared;
    assert!(twr
        .handle(Intent::RejectingTakeoff, &ctx())
        .contains("say again"));
    assert_eq!(twr.state, State::TakeoffCleared, "airborne: a garble");
}

#[test]
fn mayday_gets_priority_from_any_state() {
    // The real 2026-07-09 sequence: "mayday, mayday" got "say again",
    // twice, and only "requesting emergency landing" happened to carry it
    // via the word "landing". Never again: a mayday is answered with a
    // clearance and emergency services, from anywhere in the sequence.
    for call in [
        "mayday, mayday, mayday",
        "mayday, engine failure",
        "pan pan, pan pan",
        "declaring an emergency",
        "requesting emergency landing",
        "emergency descent",
        "mayday, can you hear me", // the question guard must not eat it
    ] {
        assert_eq!(parse_intent(call), Intent::Mayday, "{call}");
    }
    use State::*;
    for start in [
        Parked,
        TaxiCleared,
        LinedUp,
        TakeoffCleared,
        Inbound,
        ClearedToLand,
    ] {
        let mut twr = Tower::new();
        twr.state = start;
        let r = twr.handle(Intent::Mayday, &ctx());
        assert!(r.contains("roger mayday"), "{start:?}: {r}");
        assert!(r.contains("cleared to land"), "{start:?}: {r}");
        assert!(r.contains("Emergency services"), "{start:?}: {r}");
        assert_eq!(twr.state, State::ClearedToLand);
    }
    // On the GROUND there is nothing to clear: the trucks come to you,
    // the sequence doesn't move, and no landing clearance is spoken
    // (a parked mayday was "cleared to land" live, 2026-07-09).
    let mut ground = ctx();
    ground.on_ground = true;
    let mut twr = Tower::new();
    let r = twr.handle(Intent::Mayday, &ground);
    assert!(r.contains("roger mayday"), "{r}");
    assert!(r.contains("Emergency services"), "{r}");
    assert!(!r.contains("cleared to land"), "{r}");
    assert_eq!(twr.state, State::Parked);
    // But a lone "emergency" without land/descend/declare context is NOT
    // a mayday — "emergency fuel pump on" is a cockpit command.
    assert_eq!(parse_intent("emergency fuel pump on"), Intent::Unknown);
    // And the emergency CHECKLIST is cockpit ritual, not a distress call.
    assert_eq!(parse_intent("emergency landing checklist"), Intent::Unknown);
}

#[test]
fn go_around_voids_the_clearance_and_rejoins() {
    let mut twr = Tower::new();
    let c = ctx();
    twr.handle(parse_intent("inbound for landing"), &c);
    twr.handle(parse_intent("final"), &c);
    assert_eq!(twr.state, State::ClearedToLand);
    let r = twr.handle(parse_intent("november 172SP, going around"), &c);
    assert!(r.contains("go around"), "{r}");
    assert!(r.contains("report downwind"), "{r}");
    assert_eq!(twr.state, State::Inbound, "back in the pattern");
    // The circuit then re-earns the clearance leg by leg.
    let r = twr.handle(parse_intent("downwind"), &c);
    assert!(r.contains("report base"), "{r}");
    let r = twr.handle(parse_intent("turning base"), &c);
    assert!(r.contains("report final"), "{r}");
    let r = twr.handle(parse_intent("final"), &c);
    assert!(r.contains("cleared to land"), "{r}");
    // "missed approach" is the same call in IFR words.
    assert_eq!(parse_intent("missed approach"), Intent::GoAround);
}

/// A circuit is said on the request, and the words beside it never
/// change what is being asked for.
#[test]
fn a_circuit_departure_says_so_and_stays_the_same_request() {
    let cs = "november eight one six november november";
    for (call, intent) in [
        (
            format!("{cs}, request taxi for circuits"),
            Intent::RequestTaxi,
        ),
        (
            format!("{cs}, ready for departure, for circuits"),
            Intent::RequestTakeoff,
        ),
        (
            format!("{cs}, request taxi for closed traffic"),
            Intent::RequestTaxi,
        ),
        (
            format!("{cs}, ready for departure, remain in the pattern"),
            Intent::RequestTakeoff,
        ),
    ] {
        assert_eq!(parse_intent(&call), intent, "{call}");
        assert!(wants_circuits(&call), "{call}");
    }
    for call in [
        format!("{cs}, request taxi"),
        format!("{cs}, ready for departure"),
        format!("{cs}, request taxi, runway three one"),
        format!("{cs}, downwind runway three one"),
    ] {
        assert!(!wants_circuits(&call), "{call}");
    }
}

#[test]
fn pattern_reports_sequence_the_circuit() {
    assert_eq!(parse_intent("downwind runway 09"), Intent::ReportDownwind);
    assert_eq!(parse_intent("downwind for landing"), Intent::ReportDownwind);
    assert_eq!(parse_intent("turning base"), Intent::ReportBase);
    assert_eq!(
        parse_intent("left base runway two seven"),
        Intent::ReportBase
    );
    assert_eq!(parse_intent("crosswind"), Intent::ReportCrosswind);
    assert_eq!(
        parse_intent("upwind, november 172SP"),
        Intent::ReportCrosswind
    );
    // "base" without pattern context is not a report.
    assert_eq!(parse_intent("base"), Intent::Unknown);
    // A crosswind QUESTION is crew talk, not a position report.
    assert_eq!(parse_intent("what's the max crosswind"), Intent::Unknown);
    let mut twr = Tower::new();
    let r = twr.handle(Intent::ReportCrosswind, &ctx());
    assert!(r.contains("report downwind"), "{r}");
}

#[test]
fn say_again_is_a_repeat_request_unless_it_also_asks() {
    assert!(is_say_again("say again"));
    assert!(is_say_again("gibraltar tower, say again"));
    assert!(is_say_again("say again all after runway"));
    assert!(is_say_again("tower, repeat your last"));
    assert!(!is_say_again("say again, request taxi"));
    assert!(!is_say_again("ready for departure"));
    assert!(!is_say_again("roger, holding point alpha"));
}

#[test]
fn squawk_question_repeats_the_code() {
    let mut twr = Tower::new();
    let r = twr.handle(parse_intent("what about my transponder code?"), &ctx());
    assert!(r.contains("squawk seven zero zero zero"), "{r}");
}

/// EGPF, 2026-09-21: the sim starts the aircraft ON the runway, so the
/// FSM is still Parked. "Ready for departure" from there is a lined-up
/// aircraft, cleared like one and held like one; from a stand it is still
/// "taxi first", and airborne the flag means nothing.
#[test]
fn a_runway_start_is_a_lined_up_aircraft() {
    let mut on_strip = ctx();
    on_strip.own_on_runway = true;
    on_strip.on_ground = true;
    let mut twr = Tower::new();
    let r = twr.handle(Intent::RequestTakeoff, &on_strip);
    assert!(r.contains("cleared for takeoff"), "{r}");
    assert_eq!(twr.state, State::TakeoffCleared);

    // Traffic on final holds it, exactly as it holds a taxied departure.
    let mut held = busy(1.8);
    held.own_on_runway = true;
    held.on_ground = true;
    let mut twr = Tower::new();
    let r = twr.handle(Intent::RequestTakeoff, &held);
    assert!(r.contains("hold position"), "{r}");
    assert_eq!(twr.state, State::LinedUp);

    // A stand is still a stand.
    let mut twr = Tower::new();
    let r = twr.handle(Intent::RequestTakeoff, &ctx());
    assert!(r.contains("first"), "{r}");
}

/// A helicopter on its pad is cleared from where it is (ETHB, 2026-09-23:
/// the S-76 got "taxi to holding point Foxtrot runway two five" and its
/// route waited out a clearance that was never coming). In the air it is
/// no different from anyone else.
#[test]
fn a_helicopter_departs_from_present_position() {
    let mut heli = ctx();
    heli.helicopter = true;
    heli.on_ground = true;
    let mut twr = Tower::new();
    let r = twr.handle(Intent::RequestTakeoff, &heli);
    assert!(r.contains("cleared for takeoff from present position"), "{r}");
    assert!(!r.contains("runway"), "{r}");
    assert_eq!(twr.state, State::TakeoffCleared);
    let mut airborne = heli.clone();
    airborne.on_ground = false;
    let mut twr = Tower::new();
    let r = twr.handle(Intent::RequestTakeoff, &airborne);
    assert!(!r.contains("present position"), "{r}");
}

/// LEMG 13 (sortie 1790594471-full_auto, 2026-09-28): "final runway one
/// three" at 3.8 nm with the departure TO4611 on the runway was answered
/// "cleared to land", and the go-around order followed 0.8 s later. A
/// final report onto an occupied runway continues; the clearance comes
/// when the strip clears (the station's release watch), with its
/// readback owed as if it had been the reply.
#[test]
fn a_final_onto_an_occupied_runway_continues_and_is_cleared_later() {
    let mut occupied = ctx();
    occupied.runway_occupied = true;
    let mut twr = Tower::new();
    twr.state = State::Inbound;
    let r = twr.handle(Intent::ReportFinal, &occupied);
    assert!(r.contains("continue approach runway zero niner"), "{r}");
    assert!(r.contains("traffic on the runway"), "{r}");
    assert!(!r.contains("cleared to land"), "{r}");
    assert_eq!(twr.state, State::Inbound);
    assert!(twr.pending().is_none(), "nothing to read back yet");
    assert!(!twr.holds_runway());
    // The release, as the monitor speaks it.
    let line = twr.landing_clearance_line(&ctx());
    assert!(
        line.ends_with("wind zero niner zero degrees, three knots, cleared to land."),
        "{line}"
    );
    twr.release_landing("zero niner".into(), line);
    assert_eq!(twr.state, State::ClearedToLand);
    assert_eq!(
        twr.suggested_readback("November one seven two Sierra Papa")
            .as_deref(),
        Some("Cleared to land runway zero niner, November one seven two Sierra Papa.")
    );
    assert!(twr.holds_runway());
    // A clear runway is cleared at once, as before.
    let mut twr = Tower::new();
    twr.state = State::Inbound;
    let r = twr.handle(Intent::ReportFinal, &ctx());
    assert!(r.ends_with("cleared to land."), "{r}");
    // On the ground the occupancy is our own business, not a final's.
    let mut twr = Tower::new();
    let mut down = occupied.clone();
    down.on_ground = true;
    let r = twr.handle(Intent::ReportFinal, &down);
    assert!(r.contains("cleared to land"), "{r}");
}

#[test]
fn takeoff_clearance_is_cancelled_when_traffic_appears_on_final() {
    // Cleared with a clear final; traffic appears; the re-ask CANCELS
    // (the 2026-07-10 tape had a takeoff and a landing clearance on the
    // same runway eight seconds apart). The aircraft may already be on
    // the pavement, so cancellation lands in LinedUp — the release
    // watch owes it the re-clearance.
    let mut twr = Tower::new();
    twr.state = State::TaxiCleared;
    let r = twr.handle(Intent::RequestTakeoff, &ctx());
    assert!(r.contains("cleared for takeoff"), "{r}");
    let r = twr.handle(Intent::RequestTakeoff, &busy(1.8));
    assert!(r.contains("cancel takeoff clearance"), "{r}");
    assert!(r.contains("hold position"), "{r}");
    assert_eq!(
        twr.state,
        State::LinedUp,
        "clearance withdrawn, still on the pavement"
    );
}

#[test]
fn final_traffic_gates_takeoff_and_sequences_downwind() {
    // Traffic on SHORT final: too close for a conditional — hold, and the
    // state doesn't move.
    let mut twr = Tower::new();
    twr.state = State::TaxiCleared;
    let r = twr.handle(Intent::RequestTakeoff, &busy(0.9));
    assert!(r.contains("hold position"), "{r}");
    assert!(r.contains("short final"), "{r}");
    assert_eq!(twr.state, State::TaxiCleared, "clearance withheld");
    // Final clears on the next ask: the clearance comes.
    let r = twr.handle(Intent::RequestTakeoff, &ctx());
    assert!(r.contains("cleared for takeoff"), "{r}");
    assert_eq!(twr.state, State::TakeoffCleared);

    // Downwind with traffic on short final: number two, follow it.
    let mut twr = Tower::new();
    twr.state = State::Inbound;
    let r = twr.handle(Intent::ReportDownwind, &busy(0.9));
    assert!(r.contains("number two"), "{r}");
    assert!(r.contains("short final"), "{r}");
    // Empty final: number one, as before.
    let r = twr.handle(Intent::ReportDownwind, &ctx());
    assert!(r.contains("number one"), "{r}");
}

/// EFLP 06 (world sweep, 2026-10-08): one arrival far out would earn a
/// conditional line-up, but a departure cleared for takeoff holds the strip:
/// hold position, never "line up behind" in front of it.
#[test]
fn no_conditional_line_up_onto_a_held_strip() {
    let mut twr = Tower::new();
    twr.state = State::TaxiCleared;
    let mut c = busy(2.4);
    c.runway_occupied = true;
    let r = twr.handle(Intent::RequestTakeoff, &c);
    assert!(!r.contains("line up"), "{r}");
    assert!(r.contains("hold position"), "{r}");
    assert_eq!(twr.state, State::TaxiCleared);
}

/// BIEG 03 (2026-10-08): a downwind behind the sequence already given out
/// is told its own number and whom it follows, even with the strip held,
/// never number two behind a leader somebody else follows.
#[test]
fn a_downwind_behind_the_sequence_is_its_next_number() {
    let mut twr = Tower::new();
    twr.state = State::Inbound;
    let mut c = ctx();
    c.sequence_ahead = Some((3, "on downwind".into()));
    c.runway_occupied = true;
    let r = twr.handle(Intent::ReportDownwind, &c);
    assert_eq!(
        r,
        "November one seven two Sierra Papa, number three, follow the traffic on downwind, \
         report base."
    );
    assert_eq!(twr.state, State::Inbound);
}

#[test]
fn one_arrival_far_enough_earns_a_conditional_line_up() {
    // Exactly one arrival, far enough out: "behind the landing traffic,
    // line up behind" — the departure is staged, not parked.
    let mut twr = Tower::new();
    twr.state = State::TaxiCleared;
    // The traffic by TYPE and position (Doc 4444 12.3.4, CAP 413), never
    // by a callsign the departure is not working: LEMG 2026-10-08 said
    // "behind the landing Vueling one three one one". No type known:
    // "the landing traffic".
    let r = twr.handle(Intent::RequestTakeoff, &busy(2.4));
    assert_eq!(
        r,
        "November one seven two Sierra Papa, behind the landing traffic on 2 mile final, \
             line up runway zero niner behind."
    );
    assert!(!r.contains("Speedbird"), "{r}");
    let mut typed = Tower::new();
    typed.state = State::TaxiCleared;
    let r = typed.handle(
        Intent::RequestTakeoff,
        &Ctx {
            final_traffic_type: Some("A320".into()),
            ..busy(1.6)
        },
    );
    assert!(r.contains("behind the landing A320 on 2 mile final, line up"), "{r}");
    assert!(r.contains("line up runway zero niner behind"), "{r}");
    assert!(!r.contains("cleared for takeoff"), "{r}");
    assert_eq!(twr.state, State::LinedUp);
    // The condition must be read back — echoing the line-up without
    // "behind" is challenged, with the condition echoed it's silence.
    let p = twr.pending().expect("conditional awaits its readback");
    assert_eq!(
        check_readback("lining up runway zero niner, november 172SP", &p.awaited),
        ReadbackCheck::NotAReadback,
        "the condition is part of the clearance"
    );
    assert_eq!(
        check_readback(
            "behind the landing traffic, lining up runway zero niner behind, november 172SP",
            &p.awaited
        ),
        ReadbackCheck::Correct
    );
    // A STREAM on final never gets a conditional — ambiguous condition.
    let mut stream = busy(3.0);
    stream.final_traffic_count = 2;
    let mut twr = Tower::new();
    twr.state = State::TaxiCleared;
    let r = twr.handle(Intent::RequestTakeoff, &stream);
    assert!(r.contains("hold position"), "{r}");
    assert_eq!(twr.state, State::TaxiCleared);
    // Re-asking while lined up with traffic still on final: hold, no
    // takeoff clearance, still lined up.
    let mut twr = Tower::new();
    twr.state = State::LinedUp;
    let r = twr.handle(Intent::RequestTakeoff, &busy(2.0));
    assert!(r.contains("hold position"), "{r}");
    assert!(!r.contains("line up"), "{r}");
    assert_eq!(twr.state, State::LinedUp);
    // Final clear: the lined-up aircraft gets the clearance.
    let r = twr.handle(Intent::RequestTakeoff, &ctx());
    assert!(r.contains("cleared for takeoff"), "{r}");
    assert_eq!(twr.state, State::TakeoffCleared);
}

#[test]
fn wake_turbulence_lines_up_and_waits() {
    // A heavier departure just rolled: line up and wait while the two
    // minutes run — the classic use of LUAW.
    let mut wake = ctx();
    wake.wake_secs_left = Some(95);
    let mut twr = Tower::new();
    twr.state = State::TaxiCleared;
    let r = twr.handle(Intent::RequestTakeoff, &wake);
    assert!(r.contains("line up and wait runway zero niner"), "{r}");
    assert!(r.contains("wake turbulence"), "{r}");
    assert!(!r.contains("cleared for takeoff"), "{r}");
    assert_eq!(twr.state, State::LinedUp);
    // Impatience while the clock runs: hold, still no clearance.
    let r = twr.handle(Intent::RequestTakeoff, &wake);
    assert!(r.contains("hold position"), "{r}");
    assert!(r.contains("wake turbulence"), "{r}");
    assert_eq!(twr.state, State::LinedUp);
    // Clock expired: cleared.
    let r = twr.handle(Intent::RequestTakeoff, &ctx());
    assert!(r.contains("cleared for takeoff"), "{r}");
    // An occupied runway blocks the line-up — nobody lines up onto a
    // runway with traffic on it, wake clock or not.
    let mut occupied = ctx();
    occupied.wake_secs_left = Some(60);
    occupied.runway_occupied = true;
    let mut twr = Tower::new();
    twr.state = State::TaxiCleared;
    let r = twr.handle(Intent::RequestTakeoff, &occupied);
    assert!(r.contains("hold position"), "{r}");
    assert_eq!(twr.state, State::TaxiCleared);
}

#[test]
fn readback_gate_holds_the_runway_items() {
    // Takeoff scenarios play out ON the ground — the gate voids a
    // takeoff readback the moment physics says airborne.
    let mut c = ctx();
    c.on_ground = true;
    let mut twr = Tower::new();
    twr.state = State::TaxiCleared;
    let clearance = twr.handle(Intent::RequestTakeoff, &c);
    assert!(clearance.contains("cleared for takeoff"), "{clearance}");
    // A correct readback earns silence and clears the debt.
    let mut t2 = Tower::new();
    t2.state = State::TaxiCleared;
    t2.handle(Intent::RequestTakeoff, &c);
    let g = t2.gate(
        "cleared for takeoff runway zero niner, november 172SP",
        parse_intent("cleared for takeoff runway zero niner, november 172SP"),
        &c,
    );
    assert_eq!(g, Some(String::new()), "correct readback = silence");
    assert!(t2.pending().is_none());
    // The WRONG runway is corrected, never waved through.
    let mut t3 = Tower::new();
    t3.state = State::TaxiCleared;
    t3.handle(Intent::RequestTakeoff, &c);
    let g = t3
        .gate(
            "cleared for takeoff runway two seven, november 172SP",
            Intent::Unknown,
            &c,
        )
        .expect("gate speaks");
    assert!(g.contains("negative"), "{g}");
    assert!(g.contains("I say again"), "{g}");
    assert!(t3.pending().is_some(), "debt stands until the right echo");
    // The callsign's own digits never read as a runway.
    let p = t3.pending().unwrap();
    assert_eq!(
        check_readback(
            "november one seven two sierra papa, cleared for takeoff",
            &p.awaited
        ),
        ReadbackCheck::Correct,
        "no runway named at all is accepted; callsign digits are not a runway"
    );
    // Anything that isn't the echo gets ONE instruction repeat...
    let mut t4 = Tower::new();
    t4.state = State::TaxiCleared;
    t4.handle(Intent::RequestTakeoff, &c);
    // The challenge names the flight ONCE: it was "<callsign>, read back:
    // <callsign>, runway ..., cleared for takeoff" (the realism review,
    // 2026-10-09). A garble earns "read back"; a question the
    // instruction, "I say again"; an echo missing its item "readback
    // incomplete, confirm ...".
    let g = t4
        .gate("uhh what was that", Intent::Unknown, &c)
        .expect("challenge");
    assert_eq!(g, "November one seven two Sierra Papa, read back.");
    let mut t5 = Tower::new();
    t5.state = State::TaxiCleared;
    t5.handle(Intent::RequestTakeoff, &c);
    let g = t5.gate("say again", Intent::Unknown, &c).expect("challenge");
    assert_eq!(
        g,
        "November one seven two Sierra Papa, I say again, runway zero niner, wind zero niner \
             zero degrees, three knots, cleared for takeoff."
    );
    assert_eq!(g.matches("November one seven two Sierra Papa").count(), 1, "{g}");
    let g = t5
        .gate("runway zero niner, november one seven two sierra papa", Intent::Unknown, &c)
        .expect("challenge");
    assert_eq!(
        g,
        "November one seven two Sierra Papa, readback incomplete, confirm cleared for \
             takeoff runway zero niner."
    );
    // ...the question-shaped calls a tower answers pass straight through...
    assert_eq!(t4.gate("say wind", Intent::WindCheck, &c), None);
    assert!(
        t4.pending().is_some(),
        "an answered question keeps the debt"
    );
    // ...and two spent challenges release the gate — a garbled radio must
    // never deadlock the FSM.
    t4.gate("garble garble", Intent::Unknown, &c);
    assert_eq!(
        t4.gate("garble again", Intent::Unknown, &c),
        None,
        "gate lets go"
    );
    assert!(t4.pending().is_none());
    // A mayday is NEVER answered with "read back".
    let mut t5 = Tower::new();
    t5.state = State::TaxiCleared;
    t5.handle(Intent::RequestTakeoff, &c);
    assert_eq!(t5.gate("mayday, mayday", Intent::Mayday, &c), None);
    assert!(t5.pending().is_none());
    // Physics consumes the debt: airborne, the takeoff echo is moot.
    let mut t6 = Tower::new();
    t6.state = State::TaxiCleared;
    let mut ground = c.clone();
    ground.on_ground = true;
    t6.handle(Intent::RequestTakeoff, &ground);
    assert!(t6.pending().is_some());
    let flying = ctx(); // on_ground: false
    assert_eq!(
        t6.gate("some new call downwind", Intent::ReportDownwind, &flying),
        None
    );
    assert!(
        t6.pending().is_none(),
        "airborne voids the takeoff readback"
    );
}

#[test]
fn landing_clearance_awaits_its_readback() {
    let c = ctx();
    let mut twr = Tower::new();
    twr.handle(parse_intent("inbound for landing"), &c);
    twr.handle(parse_intent("final"), &c);
    let p = twr.pending().expect("landing clearance awaits the echo");
    assert_eq!(
        p.awaited,
        Awaited::Landing {
            runway: "zero niner".into(),
            touch_and_go: false
        }
    );
    assert_eq!(
        check_readback(
            "cleared to land runway zero niner, november 172SP",
            &p.awaited
        ),
        ReadbackCheck::Correct
    );
    assert_eq!(
        check_readback("cleared to land runway two seven right", &p.awaited),
        ReadbackCheck::WrongRunway {
            heard: "27R".into()
        }
    );
    // The coach knows the exact line owed.
    let s = twr
        .suggested_readback("November 172SP")
        .expect("a suggestion");
    assert!(s.contains("Cleared to land runway zero niner"), "{s}");
    // A go-around clears the debt along with the clearance.
    twr.handle(Intent::GoAround, &c);
    assert!(twr.pending().is_none());
}

#[test]
fn a_monitor_go_around_voids_the_landing_clearance() {
    // The tower cancelled the clearance itself (traffic on the runway):
    // same outcome as the pilot calling it — Inbound, no readback owed.
    let c = ctx();
    let mut twr = Tower::new();
    twr.handle(parse_intent("inbound for landing"), &c);
    twr.handle(parse_intent("final"), &c);
    assert_eq!(twr.state, State::ClearedToLand);
    assert!(twr.pending().is_some());
    twr.apply_go_around();
    assert_eq!(twr.state, State::Inbound);
    assert!(
        twr.pending().is_none(),
        "the dead clearance's echo is not owed"
    );
}

// LEMG 2026-09-12: "final runway one three" answered with "runway one
// two, cleared to land" and nobody heard the difference.
#[test]
fn a_final_report_on_the_wrong_runway_is_corrected_aloud() {
    let c = ctx();
    let mut t = Tower { state: State::Inbound, ..Default::default() };
    let reply = t.handle(Intent::ReportFinal, &c);
    let fixed = Tower::correct_final_runway(
        "november one seven two sierra papa, final runway one three",
        Intent::ReportFinal,
        &c,
        reply.clone(),
    );
    assert!(
        fixed.starts_with("November one seven two Sierra Papa, negative, runway zero niner, I say again runway zero niner. runway zero niner, wind"),
        "{fixed}"
    );
    assert!(fixed.ends_with("cleared to land."), "{fixed}");
    // The right runway, or none spoken: the reply as it was.
    for t in ["final runway zero niner", "november one seven two, final"] {
        assert_eq!(Tower::correct_final_runway(t, Intent::ReportFinal, &c, reply.clone()), reply, "{t}");
    }
    // Only final reports: a taxi request naming another runway is the
    // parser's business, not this one's.
    assert_eq!(
        Tower::correct_final_runway("request taxi runway one three", Intent::RequestTaxi, &c, reply.clone()),
        reply
    );
}

#[test]
fn canon_runway_normalizes_all_spellings() {
    for (raw, want) in [
        ("zero niner", "9"),
        ("two seven right", "27R"),
        ("27R", "27R"),
        ("27 right", "27R"),
        ("09L", "9L"),
        ("three one", "31"),
    ] {
        assert_eq!(canon_runway(raw).as_deref(), Some(want), "{raw}");
    }
    assert_eq!(canon_runway("alpha"), None);
    // Transcript extraction only trusts digits after the word "runway".
    assert_eq!(
        runway_after_keyword("november one seven two, cleared for takeoff runway two seven"),
        Some("27".into())
    );
    assert_eq!(
        runway_after_keyword("november one seven two, cleared for takeoff"),
        None,
        "callsign digits are not a runway"
    );
}

#[test]
fn taxi_clearance_speaks_the_route() {
    let mut c = ctx();
    c.taxi_via = vec!["Alpha".into(), "Bravo two".into()];
    c.taxi_crossings = vec!["two three".into()];
    let mut twr = Tower::new();
    let r = twr.handle(Intent::RequestTaxi, &c);
    assert!(r.contains("via Alpha, Bravo two"), "{r}");
    assert!(r.contains("cross runway two three"), "{r}");
    // No network: the clearance phrases exactly as before.
    let mut twr = Tower::new();
    let r = twr.handle(Intent::RequestTaxi, &ctx());
    assert!(!r.contains(" via "), "{r}");
    assert!(!r.contains("cross runway"), "{r}");
}

#[test]
fn arrivals_taxi_to_parking_not_the_holding_point() {
    // By wording, from any state.
    let mut twr = Tower::new();
    let r = twr.handle(parse_intent("request taxi to parking"), &ctx());
    assert!(r.contains("taxi to parking"), "{r}");
    assert!(!r.contains("holding point"), "{r}");
    assert_eq!(twr.state, State::Parked);
    // By state: a generic taxi request right after a landing clearance is
    // the arrival leaving the runway.
    let mut twr = Tower::new();
    twr.handle(parse_intent("inbound for landing"), &ctx());
    twr.handle(parse_intent("final"), &ctx());
    let r = twr.handle(parse_intent("request taxi"), &ctx());
    assert!(r.contains("taxi to parking"), "{r}");
    assert_eq!(twr.state, State::Parked);
    // A departure from the stand still gets the holding point.
    let mut twr = Tower::new();
    let r = twr.handle(parse_intent("request taxi"), &ctx());
    assert!(r.contains("holding point"), "{r}");
}

#[test]
fn readbacks_earn_silence_not_say_again() {
    // Instruction echoes: no intent, but recognizably a readback.
    for rb in [
        "descending to one thousand five hundred feet",
        "heading two five zero, november one seven two",
        "roger, wilco",
        "squawking seven thousand",
    ] {
        assert_eq!(parse_intent(rb), Intent::Unknown, "{rb}");
        assert!(is_readback(rb), "{rb}");
    }
    // Genuine garble is NOT a readback — it still gets "say again".
    assert!(!is_readback("uhh gibraltar something something"));
    assert!(!is_readback("requesting flight following"));
}

#[test]
fn arrival_flow_inbound_final_cleared() {
    let mut twr = Tower::new();
    let c = ctx();
    let join = twr.handle(parse_intent("gibraltar tower, inbound for landing"), &c);
    assert!(join.contains("join final runway zero niner"), "{join}");
    assert!(join.contains("Report final"), "{join}");
    assert_eq!(twr.state, State::Inbound);
    let clear = twr.handle(parse_intent("final"), &c);
    assert!(clear.contains("cleared to land"), "{clear}");
    assert_eq!(twr.state, State::ClearedToLand);
    // Touch and go shapes the clearance.
    let mut twr = Tower::new();
    twr.handle(parse_intent("request touch and go"), &c);
    let clear = twr.handle(parse_intent("final"), &c);
    assert!(clear.contains("cleared touch and go"), "{clear}");
}

#[test]
fn touch_and_go_survives_the_approach_handoff() {
    // The option was requested on the approach frequency; the tower only
    // ever hears "final" — the clearance must still say touch and go.
    let mut twr = Tower::new();
    twr.expect_touch_and_go(true);
    let clear = twr.handle(parse_intent("final"), &ctx());
    assert!(clear.contains("cleared touch and go"), "{clear}");
}

#[test]
fn vectored_arrival_gets_cleared_straight_from_final() {
    // A vectored arrival never made an inbound call to the tower; the
    // "final" report alone earns the clearance.
    let mut twr = Tower::new();
    let clear = twr.handle(parse_intent("final"), &ctx());
    assert!(clear.contains("cleared to land"), "{clear}");
}

#[test]
fn vectors_redirect_to_approach_or_join_final() {
    let mut twr = Tower::new();
    let redirect = twr.handle(parse_intent("request vectors"), &ctx());
    assert!(
        redirect.contains("contact Gibraltar Approach on one two two decimal eight"),
        "{redirect}"
    );
    assert_eq!(twr.state, State::Parked, "redirect changes nothing");
    // No approach service → straight-in join instead.
    let mut no_app = ctx();
    no_app.approach = None;
    let join = twr.handle(parse_intent("request vectors"), &no_app);
    assert!(join.contains("Join final"), "{join}");
    assert_eq!(twr.state, State::Inbound);
}

#[test]
fn wind_check_reads_wind_and_qnh() {
    let mut twr = Tower::new();
    let r = twr.handle(parse_intent("say wind"), &ctx());
    assert!(
        r.contains("wind zero niner zero degrees, three knots"),
        "{r}"
    );
    assert!(r.contains("QNH one zero one seven"), "{r}");
}

#[test]
fn taxi_then_takeoff_is_the_happy_path() {
    let mut twr = Tower::new();
    let c = ctx();
    let taxi = twr.handle(parse_intent("request taxi"), &c);
    assert!(
        taxi.contains("taxi to holding point Alpha runway zero niner"),
        "{taxi}"
    );
    assert!(taxi.contains("QNH one zero one seven"), "{taxi}");
    assert_eq!(twr.state, State::TaxiCleared);

    let takeoff = twr.handle(parse_intent("ready for departure"), &c);
    assert!(takeoff.contains("cleared for takeoff"), "{takeoff}");
    assert!(
        takeoff.contains("wind zero niner zero degrees, three knots"),
        "{takeoff}"
    );
    assert_eq!(twr.state, State::TakeoffCleared);
}

#[test]
fn takeoff_from_the_apron_gets_taxi_first() {
    let mut twr = Tower::new();
    let reply = twr.handle(parse_intent("request takeoff"), &ctx());
    assert!(!reply.contains("cleared for takeoff"), "{reply}");
    assert!(reply.contains("taxi to holding point"), "{reply}");
    assert_eq!(twr.state, State::TaxiCleared);
}

#[test]
fn unknown_gets_say_again_and_no_state_change() {
    let mut twr = Tower::new();
    let reply = twr.handle(parse_intent("uhh what's the weather like"), &ctx());
    assert!(reply.contains("say again"), "{reply}");
    assert_eq!(twr.state, State::Parked);
}

#[test]
fn callsigns_speak_nato() {
    assert_eq!(
        spoken_callsign("N172SP"),
        "November one seven two Sierra Papa"
    );
    assert_eq!(spoken_callsign("G-ABCD"), "Golf Alpha Bravo Charlie Delta");
    assert_eq!(spoken_callsign(""), "");
}

#[test]
fn a_heavy_says_so_and_a_medium_does_not() {
    assert_eq!(
        spoken_callsign_wake("D-AIKN", Some(242_000.0)),
        "Delta Alpha India Kilo November heavy",
        "the A330 is a heavy"
    );
    assert_eq!(
        spoken_callsign_wake("G-ABCD", Some(79_000.0)),
        "Golf Alpha Bravo Charlie Delta",
        "the 737 is a medium"
    );
    assert_eq!(
        spoken_callsign_wake("N172SP", None),
        "November one seven two Sierra Papa",
        "unknown mass is no suffix"
    );
    assert_eq!(spoken_callsign_wake("", Some(300_000.0)), "");
}

/// Liege 2026-09-22: the crew reported "runway vacated" from the taxiway,
/// as it does on every arrival, and the tower said "say again" - twice.
/// The report is acknowledged now, and acknowledging it must not spend the
/// landing clearance the taxi-in inference rests on.
#[test]
fn a_vacate_report_is_acknowledged_and_spends_nothing() {
    let mut c = ctx();
    c.stand = "stand one two".into();
    let mut twr = Tower::new();
    twr.state = State::ClearedToLand;
    let reply = twr.handle(Intent::RunwayVacated, &c);
    assert!(!reply.contains("say again"), "{reply:?}");
    assert!(reply.contains(&c.callsign), "{reply:?}");
    assert!(
        !reply.contains("cleared") && !reply.contains("taxi"),
        "a report earns words, never a clearance: {reply:?}"
    );
    assert_eq!(
        twr.state,
        State::ClearedToLand,
        "the state stands, so the bare taxi request below is still the taxi IN"
    );
    assert!(twr
        .handle(Intent::RequestTaxi, &c)
        .contains("taxi to stand one two"));
}

/// The same report frees the strip in the tower's book: on the ground,
/// after a landing, and only then. Without it the old behaviour stands
/// (the landing holds the runway until the taxi request parks it).
#[test]
fn the_vacated_report_frees_the_runway_in_the_book() {
    let mut c = ctx();
    c.on_ground = true;
    let mut twr = Tower::new();
    twr.state = State::ClearedToLand;
    assert!(twr.holds_runway(), "landed, not reported off");
    twr.handle(Intent::RunwayVacated, &c);
    assert!(!twr.holds_runway() && twr.runway_vacated());
    assert_eq!(twr.state, State::ClearedToLand);
    // The next landing starts with the strip held again.
    twr.handle(Intent::RequestTaxi, &c);
    assert_eq!(twr.state, State::Parked);
    twr.handle(Intent::ReportFinal, &c);
    assert!(twr.holds_runway() && !twr.runway_vacated());

    // From the air the words are a garble, not a report.
    let mut air = ctx();
    air.on_ground = false;
    let mut twr = Tower::new();
    twr.state = State::ClearedToLand;
    twr.handle(Intent::RunwayVacated, &air);
    assert!(twr.holds_runway());

    // Without the report, the taxi request is what frees it (as before).
    let mut twr = Tower::new();
    twr.state = State::ClearedToLand;
    twr.handle(Intent::RequestTaxi, &c);
    assert!(!twr.holds_runway());
}

/// The runway request (ETND 2026-09-24): granted on the ground as the
/// taxi to the runway the station resolved, refused with its reason,
/// "say again" when the words named no runway. Never a takeoff or a
/// landing clearance.
#[test]
fn a_named_departure_is_a_sid_request_and_nothing_else_is() {
    // #152: the crew's words for another SID parse as the request and
    // name it back; the calls around it keep their own readings.
    let call = "Whiskey Alpha Tango six two six, unable Kene three Papa, request the Lam five Papa departure";
    assert_eq!(parse_intent(call), Intent::RequestSid);
    assert_eq!(requested_departure(call).as_deref(), Some("lam five papa"));
    let idents: Vec<String> =
        ["KENE3P", "LAM5P", "LAM1Z", "LAM5W"].iter().map(|s| s.to_string()).collect();
    assert_eq!(match_sid("lam five papa", &idents), Some("LAM5P"));
    assert_eq!(match_sid("lambourne five papa", &idents), Some("LAM5P"));
    assert_eq!(match_sid("lam1z", &idents), Some("LAM1Z"));
    assert_eq!(match_sid("kene three papa", &idents), Some("KENE3P"));
    assert_eq!(match_sid("lam five", &idents), None, "no suffix names none");
    assert_eq!(match_sid("dover two papa", &idents), None);
    assert_eq!(parse_intent("request LAM5P departure"), Intent::RequestSid);
    for other in [
        "request runway two six left for departure",
        "ready for departure",
        "request departure",
        "request taxi for departure",
        "request IFR clearance to Echo Golf Bravo Bravo",
        "cleared to Birmingham via the Kene three Papa departure, runway zero eight right, squawk four five four five",
    ] {
        assert_ne!(parse_intent(other), Intent::RequestSid, "{other}");
    }
    assert_eq!(intent_from_label("request_sid"), Some(Intent::RequestSid));
}

#[test]
fn a_runway_request_is_granted_as_a_taxi_or_refused_with_the_reason() {
    assert_eq!(requested_runway("request runway zero eight left").as_deref(), Some("8L"));
    assert_eq!(
        requested_runway("unable two six, request zero eight for length").as_deref(),
        Some("8")
    );
    assert_eq!(requested_runway("request taxi, runway two seven"), None);
    assert_eq!(requested_runway("november one two three, request taxi"), None);

    let mut c = ctx();
    c.on_ground = true;
    c.runway = "zero eight".into();
    c.holding_point = "Hotel two".into();
    let mut twr = Tower::new();
    let reply = twr.handle(Intent::RequestRunway, &c);
    assert!(
        reply.contains("runway zero eight approved, taxi to holding point Hotel two runway zero eight"),
        "{reply:?}"
    );
    assert!(!reply.contains("cleared"), "{reply:?}");
    assert_eq!(twr.state, State::TaxiCleared);

    let mut no = c.clone();
    no.runway_refusal = Some(("two six right".into(), "that runway is not available".into()));
    let mut twr = Tower::new();
    let reply = twr.handle(Intent::RequestRunway, &no);
    assert!(
        reply.contains("unable runway two six right, that runway is not available"),
        "{reply:?}"
    );
    assert_eq!(twr.state, State::Parked, "a refusal moves nothing");

    let mut garbled = c.clone();
    garbled.runway_refusal = Some((String::new(), String::new()));
    let mut twr = Tower::new();
    assert!(twr.handle(Intent::RequestRunway, &garbled).contains("say again"));
    assert_eq!(twr.state, State::Parked);
}

#[test]
fn arrival_taxi_in_clears_to_the_planned_stand() {
    // Fresh off landing, "taxi to parking" earns a real taxi-in clearance
    // to the planned stand — with the via and any runway crossing — and
    // drops the FSM back to Parked.
    let mut c = ctx();
    c.stand = "stand one two".into();
    c.taxi_via = vec!["Alpha".into(), "Bravo".into()];
    c.taxi_crossings = vec!["zero niner".into()];
    let mut twr = Tower::new();
    twr.state = State::ClearedToLand;
    let reply = twr.handle(Intent::RequestTaxiIn, &c);
    assert!(reply.contains("taxi to stand one two"), "{reply:?}");
    assert!(reply.contains("via Alpha, Bravo"), "{reply:?}");
    assert!(reply.contains("cross runway zero niner"), "{reply:?}");
    assert!(reply.contains("this frequency"), "{reply:?}");
    assert_eq!(twr.state, State::Parked);

    // A generic "request taxi" right after landing is an arrival too.
    let mut twr2 = Tower::new();
    twr2.state = State::ClearedToLand;
    assert!(twr2
        .handle(Intent::RequestTaxi, &c)
        .contains("taxi to stand one two"));

    // No stand planned (field with no parking, or off the network) → the
    // honest flat fallback, never a fabricated stand.
    let mut c3 = ctx();
    c3.stand = String::new();
    let mut twr3 = Tower::new();
    twr3.state = State::ClearedToLand;
    assert!(twr3
        .handle(Intent::RequestTaxiIn, &c3)
        .contains("taxi to parking"));
}

/// A crossing request: asked in words the pilot (or our traffic) uses, its
/// readback never a request; answered from the station's judgement, cleared
/// with a readback owed, held short with the reason, and never from the air.
#[test]
fn a_runway_crossing_is_asked_answered_and_read_back() {
    for words in [
        "Gatwick Tower, Cessna three four five, holding short runway two six right, \
         request crossing",
        "tower, november one two three, request to cross runway two six right",
        "requesting crossing runway zero eight left",
    ] {
        assert_eq!(parse_intent(words), Intent::RequestCrossing, "{words}");
    }
    assert_ne!(
        parse_intent("crossing runway two six right, Cessna three four five"),
        Intent::RequestCrossing,
        "the readback"
    );
    assert_eq!(
        parse_intent("request runway two six right"),
        Intent::RequestRunway,
        "a runway request stays one"
    );
    let ground = Ctx {
        on_ground: true,
        crossing: Some(("two six right".into(), None)),
        ..ctx()
    };
    let mut twr = Tower::new();
    twr.state = State::TaxiCleared;
    let r = twr.handle(Intent::RequestCrossing, &ground);
    assert_eq!(
        r,
        format!("{}, cross runway two six right.", ground.callsign)
    );
    assert_eq!(twr.state, State::TaxiCleared, "still on its taxi");
    assert_eq!(
        twr.suggested_readback("N123").as_deref(),
        Some("Crossing runway two six right, N123.")
    );
    assert_eq!(
        twr.gate(
            "crossing runway two six right, N123",
            Intent::Unknown,
            &ground
        ),
        Some(String::new()),
        "read back"
    );
    let busy = Ctx {
        crossing: Some(("two six right".into(), Some("traffic on final".into()))),
        ..ground.clone()
    };
    let r = twr.handle(Intent::RequestCrossing, &busy);
    assert_eq!(
        r,
        format!(
            "{}, hold short runway two six right, traffic on final.",
            busy.callsign
        )
    );
    assert!(twr.pending().is_none(), "nothing to read back");
    let unnamed = Ctx {
        crossing: None,
        ..ground.clone()
    };
    let r = twr.handle(Intent::RequestCrossing, &unnamed);
    assert!(r.contains("say again"), "{r}");
    let airborne = Ctx {
        on_ground: false,
        ..ground.clone()
    };
    let r = twr.handle(Intent::RequestCrossing, &airborne);
    assert!(r.contains("say again"), "{r}");
    // The taxi clearance of a flight that asks at the runway holds it short;
    // ours is cleared across in it, as before.
    let taxi = Ctx {
        taxi_crossings: vec!["two six right".into()],
        ..ground.clone()
    };
    let ours = Tower::new().handle(Intent::RequestTaxi, &taxi);
    assert!(ours.contains(", cross runway two six right"), "{ours}");
    let asks = Ctx {
        hold_short_crossings: true,
        ..taxi
    };
    let held = Tower::new().handle(Intent::RequestTaxi, &asks);
    assert!(held.contains(", hold short runway two six right"), "{held}");
}

/// The realism review (2026-10-09): a go-around is told what THIS arrival
/// flies. Circuit traffic climbs to the circuit in the field's words
/// (CAP 413 "circuit height", ICAO "circuit altitude", FAA "pattern
/// altitude"); an instrument arrival (an IFR flight, an airliner off the
/// ILS) flies the published missed approach and goes back to the radar
/// unit, or with no procedure climbs on runway heading. It was "climb to
/// circuit altitude, report downwind" to every arrival.
#[test]
fn a_go_around_is_told_what_this_arrival_flies() {
    use crate::atis::Dialect;
    let cs = "November one seven two Sierra Papa";
    let reply = |c: &Ctx| Tower::new().handle(Intent::GoAround, c);
    assert_eq!(
        reply(&Ctx { uk: true, ..ctx() }),
        format!("{cs}, roger, go around. Climb to circuit height, runway heading, report downwind.")
    );
    assert_eq!(
        reply(&ctx()),
        format!("{cs}, roger, go around. Climb to circuit altitude, runway heading, report downwind.")
    );
    assert_eq!(
        reply(&Ctx {
            dialect: Dialect::Faa,
            ..ctx()
        }),
        format!("{cs}, roger, go around. Fly runway heading, climb to pattern altitude, report downwind.")
    );
    let published = MissedApproach {
        published: true,
        alt_ft: 3000.0,
        contact: Some(("Malaga Approach".into(), "one two three decimal four five".into())),
    };
    let r = reply(&Ctx {
        missed_approach: Some(published.clone()),
        ..ctx()
    });
    assert_eq!(
        r,
        format!(
            "{cs}, roger, go around. Fly the published missed approach, contact Malaga \
             Approach on one two three decimal four five."
        )
    );
    assert!(!r.contains("circuit") && !r.contains("downwind"), "{r}");
    let none = MissedApproach {
        published: false,
        contact: None,
        ..published
    };
    assert_eq!(
        reply(&Ctx {
            missed_approach: Some(none.clone()),
            ..ctx()
        }),
        format!("{cs}, roger, go around. Climb altitude three thousand feet, runway heading.")
    );
    assert_eq!(
        reply(&Ctx {
            missed_approach: Some(none),
            dialect: Dialect::Faa,
            ..ctx()
        }),
        format!("{cs}, roger, go around. Fly runway heading, climb and maintain three thousand.")
    );
    assert_eq!(missed_approach_alt_ft(52.0), 3000.0, "LEMG");
    assert_eq!(missed_approach_alt_ft(5600.0), 8000.0, "LSZS");
    // The re-sequenced answer speaks the same words.
    let r = go_around_resequenced_line(cs, "three zero", "one two", Dialect::Icao, true, None);
    assert!(
        r.contains("Climb to circuit height, runway heading. Runway three zero is blocked"),
        "{r}"
    );
}

/// The realism review (2026-10-09): an FAA field's taxi clearance is the
/// FAA's (7110.65 3-7-2: "runway two eight left, taxi via Alpha, Bravo,
/// hold short of runway two eight left"), not ICAO's holding point by name;
/// and an ICAO clearance's via list no longer ends with the holding point it
/// names (LEMG, 2026-10-08: "taxi to holding point Hotel November one runway
/// one three via Delta, Quebec, Alpha, Hotel November one").
#[test]
fn a_taxi_clearance_is_phrased_in_the_fields_dialect() {
    let ground = Ctx {
        on_ground: true,
        runway: "one three".into(),
        holding_point: "Hotel November one".into(),
        taxi_via: vec![
            "Delta".into(),
            "Quebec".into(),
            "Alpha".into(),
            "Hotel November one".into(),
        ],
        ..ctx()
    };
    let r = Tower::new().handle(Intent::RequestTaxi, &ground);
    assert_eq!(
        r,
        "November one seven two Sierra Papa, taxi to holding point Hotel November one runway \
         one three via Delta, Quebec, Alpha, QNH one zero one seven. Report ready for departure."
    );
    let faa = Ctx {
        dialect: crate::atis::Dialect::Faa,
        runway: "two eight left".into(),
        qnh: "altimeter three zero zero one".into(),
        taxi_via: vec!["Alpha".into(), "Bravo".into()],
        taxi_crossings: vec!["one niner".into()],
        hold_short_crossings: true,
        ..ground.clone()
    };
    let r = Tower::new().handle(Intent::RequestTaxi, &faa);
    assert_eq!(
        r,
        "November one seven two Sierra Papa, runway two eight left, taxi via Alpha, Bravo, \
         hold short of runway one niner, hold short of runway two eight left, altimeter three \
         zero zero one. Report ready for departure."
    );
    assert!(!r.contains("holding point"), "{r}");
    // A ready call from the stand: the taxi first, same words.
    let r = Tower::new().handle(Intent::RequestTakeoff, &faa);
    assert!(r.contains("runway two eight left, taxi via Alpha, Bravo"), "{r}");
    let r = Tower::new().handle(Intent::RequestTakeoff, &ground);
    assert!(
        r.contains("Hotel November one runway one three first via Delta, Quebec, Alpha,"),
        "{r}"
    );
}
