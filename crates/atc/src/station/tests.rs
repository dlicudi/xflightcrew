use super::*;

#[test]
fn traffic_is_called_again_only_when_it_is_more_of_a_factor() {
    let f = |nm: f64, ft: f64| TrafficFactor::new(nm, 3, ft);
    let called = f(6.0, -900.0);
    assert!(!traffic_worth_recalling(&called, &f(6.0, -700.0)), "200 ft closer is the same call");
    assert!(!traffic_worth_recalling(&called, &f(6.0, -1400.0)), "diverging");
    assert!(traffic_worth_recalling(&called, &f(4.0, -900.0)), "two miles closer");
    assert!(traffic_worth_recalling(&called, &f(5.5, -300.0)), "600 ft less between us");
}

#[test]
fn traffic_advisory_skips_ground_targets() {
    // Slot 0 = own ship. Two targets both 2 nm off at same altitude: slot 1
    // is on the ground (weight on wheels), slot 2 airborne and slightly
    // farther. The advisory must factor the AIRBORNE one, never the parked
    // aircraft on the ramp (the 2026-07-24 bug).
    let two_nm = 2.0 * 1852.0;
    let dist = vec![0.0, two_nm, two_nm + 200.0];
    let bear = vec![0.0, 90.0, 180.0];
    let alt = vec![0.0, 0.0, 0.0];
    let wow = vec![0.0, 1.0, 0.0]; // slot 1 on the ground
    let f = select_traffic_factor(&dist, &bear, &alt, &wow).expect("an airborne factor");
    assert_eq!(
        f.clock, 6,
        "the airborne target at 180°, not the ground one at 90°"
    );

    // ONLY a ground target in range → no advisory at all.
    let f = select_traffic_factor(&[0.0, two_nm], &[0.0, 90.0], &[0.0, 0.0], &[0.0, 1.0]);
    assert!(f.is_none(), "a lone ground target is not a factor");

    // No WOW feed → fail open (announce), so traffic isn't silently lost.
    let f = select_traffic_factor(&[0.0, two_nm], &[0.0, 90.0], &[0.0, 0.0], &[]);
    assert!(f.is_some(), "unfed WOW announces rather than suppressing");
}

#[test]
fn garbled_flight_ids_never_panic_the_telephony() {
    // Flight ids come off raw TCAS bytes through from_utf8_lossy, so a
    // garbage feed hands us replacement characters whose byte boundaries
    // sit exactly where the airline-prefix slices used to cut. A bad id
    // must degrade to NATO spelling, never panic the monitor thread.
    for id in [
        "A\u{FFFD}12",
        "B\u{FFFD}306",
        "\u{FFFD}\u{FFFD}",
        "é1",
        "BÉ306",
    ] {
        let spoken = spoken_ghost(id);
        assert!(!spoken.contains('\u{FFFD}'), "{id:?}: {spoken:?}");
        let _ = is_airline(id);
    }
    // The honest ids still resolve, both spellings.
    assert_eq!(spoken_ghost("BAW306"), "Speedbird three zero six");
    assert_eq!(spoken_ghost("BA306"), "Speedbird three zero six");
    assert!(is_airline("BAW306") && is_airline("BA306"));
    assert!(!is_airline("N172SP"));
}

#[test]
fn discrete_squawks_are_valid_and_deterministic() {
    for cs in ["November one seven two Sierra Papa", "G-ABCD", "x"] {
        let code = discrete_squawk(cs);
        assert_eq!(code, discrete_squawk(cs), "stable per callsign");
        assert!((4500..=4577).contains(&code), "{code} outside the block");
        // Every digit octal (transponders have no 8s or 9s).
        assert!(
            code.to_string().chars().all(|c| ('0'..='7').contains(&c)),
            "{code}"
        );
        // Clear of every reserved code by construction.
        assert!(![0, 1200, 2000, 7000, 7500, 7600, 7700].contains(&code));
    }
}

#[test]
fn squawk_watch_calls_radar_contact_only_when_the_code_shows() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // The code is derived from the callsign, and a watch is only re-armed
    // for a DIFFERENT code: a watch left by an earlier test with this
    // callsign (armed by Departure, already nudged) was this test's watch
    // about one run in four, by test order.
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.squawk_nudge.lock().unwrap() = None;
    let mut st = lxgb();
    let spoken = st.assign_squawk(
        "November one seven two Sierra Papa",
        StationSpeaker::Approach,
    );
    let code = st.desk.assigned_squawk.expect("code issued");
    assert_eq!(spoken, atis::spoken_digits(&code.to_string()));
    // Same flight asks again: same code, watch not reset.
    assert_eq!(
        st.assign_squawk(
            "November one seven two Sierra Papa",
            StationSpeaker::Approach
        ),
        spoken
    );

    let t0 = Instant::now();
    // Still on the old code, inside the grace period: silence.
    assert!(squawk_tick(Some(7000), true, StationSpeaker::Approach, t0).is_none());
    // Grace expired, still wrong: the nudge, once, from the assigner.
    let late = t0 + std::time::Duration::from_secs(25);
    let (line, who) =
        squawk_tick(Some(7000), true, StationSpeaker::Approach, late).expect("nudge");
    assert!(line.contains("confirm squawk"), "{line}");
    assert!(matches!(who, StationSpeaker::Approach));
    // The crew hears it as data: the code to dial, no recycle.
    assert_eq!(
        crate::party_line::take_squawk_nudge(),
        Some(crate::party_line::SquawkNudge { code, recycle: false })
    );
    assert_eq!(crate::party_line::take_squawk_nudge(), None, "drained once");
    // Immediately again: quiet (nudge cooldown).
    assert!(squawk_tick(Some(7000), true, StationSpeaker::Approach, late).is_none());
    // Right code but transponder on standby: not replying, no contact —
    // and the nudge names the REAL problem, never "confirm squawk" (the
    // digits are the one thing that's right; Gatwick 2026-07-13).
    let later = late + std::time::Duration::from_secs(50);
    let (line, _) =
        squawk_tick(Some(code), false, StationSpeaker::Approach, later).expect("standby nudge");
    assert!(
        line.contains("negative radar contact, recycle transponder"),
        "{line}"
    );
    assert!(!line.contains("confirm squawk"), "{line}");
    assert_eq!(
        crate::party_line::take_squawk_nudge(),
        Some(crate::party_line::SquawkNudge { code, recycle: true }),
        "the crew is told to recycle"
    );
    // Code set and replying: "radar contact", exactly once.
    let (line, _) = squawk_tick(
        Some(code),
        true,
        StationSpeaker::Approach,
        later + std::time::Duration::from_secs(50),
    )
    .expect("radar contact");
    assert!(line.contains("radar contact"), "{line}");
    assert!(squawk_tick(
        Some(code),
        true,
        StationSpeaker::Approach,
        later + std::time::Duration::from_secs(60)
    )
    .is_none());
    assert!(
        !squawk_watch_active(),
        "satisfied watch stops the dataref reads"
    );
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
}

#[test]
fn departure_clearance_assigns_a_discrete_code_at_radar_fields() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.assigned_squawk.lock().unwrap() = None;
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 8.0,
    };
    // LXGB has an approach service: the zone clearance carries the code.
    let mut st = lxgb();
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        true,
        false,
        None,
        false,
        None,
    );
    let r = st.respond(
        "",
        Service::Tower,
        tower::Intent::RequestClearance,
        &ctx,
        wx,
    );
    let code = st.desk.assigned_squawk.expect("discrete code issued");
    let spoken = atis::spoken_digits(&code.to_string());
    assert!(r.contains(&spoken), "{r}");
    assert!(!r.contains("seven zero zero zero"), "{r}");
    assert!(OWN_STRIP.squawk_watch.lock().unwrap().is_some(), "radar watch armed");
    // ...and published for the crew to dial into the transponder.
    assert_eq!(
        *OWN_STRIP.assigned_squawk.lock().unwrap(),
        Some(code),
        "assigned code published to the party line"
    );
    // A tower-only strip stays on the VFR conspicuity and publishes nothing.
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.assigned_squawk.lock().unwrap() = None;
    let mut st = lxgb();
    st.approach_khz = None;
    st.departure_khz = None;
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        true,
        false,
        None,
        false,
        None,
    );
    let r = st.respond(
        "",
        Service::Tower,
        tower::Intent::RequestClearance,
        &ctx,
        wx,
    );
    assert!(r.contains("seven zero zero zero"), "{r}");
    assert!(st.desk.assigned_squawk.is_none());
    assert!(OWN_STRIP.squawk_watch.lock().unwrap().is_none());
    assert!(
        OWN_STRIP.assigned_squawk.lock().unwrap().is_none(),
        "no discrete code, nothing published"
    );
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.assigned_squawk.lock().unwrap() = None;
}

#[test]
fn tuned_station_resolves_controllers_and_the_atis() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *CONTROLLER_FREQS.lock().unwrap() = vec![
        (118105, StationSpeaker::Tower),
        (128500, StationSpeaker::Approach),
    ];
    *ATIS.lock().unwrap() = Some(AtisState {
        khz: 125655,
        airport: "Jerez".into(),
        runways: Vec::new(),
        letter: 'A',
        dialect: Dialect::Icao,
        edition: None,
        last_spoken: None,
    });
    // Controllers resolve (within the 5 kHz tolerance).
    assert_eq!(tuned_station(118105), Some(StationSpeaker::Tower));
    assert_eq!(tuned_station(128500), Some(StationSpeaker::Approach));
    // The ATIS loop resolves too — this is what tuned_controller misses,
    // so live ATIS audio isn't wrongly cut while you're ON its frequency.
    assert_eq!(tuned_station(125655), Some(StationSpeaker::Atis));
    assert_eq!(
        tuned_controller(125655),
        None,
        "controllers-only misses ATIS"
    );
    // A real frequency that's none of them → off all stations (→ cut).
    assert_eq!(tuned_station(122800), None);
    *CONTROLLER_FREQS.lock().unwrap() = Vec::new();
    *ATIS.lock().unwrap() = None;
}

#[test]
fn squawk_check_repeats_the_assigned_code() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 8.0,
    };
    let mut st = lxgb();
    // Before any radar service: the VFR conspicuity.
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        false,
        false,
        None,
        false,
        None,
    );
    let r = st.desk.twr.handle(tower::Intent::SquawkCheck, &ctx);
    assert!(r.contains("seven zero zero zero"), "{r}");
    // Flight following assigns the discrete code...
    let r = st.respond_approach(tower::Intent::RequestFlightFollowing, &ctx, wx);
    let code = st.desk.assigned_squawk.expect("code issued");
    let spoken = atis::spoken_digits(&code.to_string());
    assert!(r.contains(&spoken), "{r}");
    assert!(
        !r.contains("radar contact"),
        "contact is the watch's call, not theatre: {r}"
    );
    // ...and every later squawk check repeats THAT code, tower included.
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        false,
        false,
        None,
        false,
        None,
    );
    let r = st.desk.twr.handle(tower::Intent::SquawkCheck, &ctx);
    assert!(r.contains(&spoken), "{r}");
    assert!(!r.contains("seven zero zero zero"), "{r}");
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
}

#[test]
fn compliance_watch_holds_the_pilot_to_clearances() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // Gibraltar-ish: field reference + the real-ish 09/27 geometry.
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
    *ACTIVE_PANEL.lock().unwrap() = Some((
        "Gibraltar".into(),
        11.0,
        vec![("tower".to_string(), 131200_i64)],
    ));
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1512,
        lon_a: -5.3554,
        lat_b: 36.1517,
        lon_b: -5.3366,
    }];
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    // No arrival taxi-in in play — this test's occupancy berate depends on it
    // (a prior test could leave the shared flag set).
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;

    // ZONE: airborne 3 nm out at 1,500 ft, never called anyone, not
    // tuned: the CREW flags it, with the tower frequency.
    let call = compliance_tick(36.20, -5.35, 1500.0, false, 90.0, false, t0).expect("zone");
    assert!(call.crew);
    assert!(
        call.line.contains("zone without clearance"),
        "{}",
        call.line
    );
    assert!(
        call.line.contains("decimal two"),
        "no tower freq: {}",
        call.line
    );
    // Cooldown holds.
    assert!(compliance_tick(36.20, -5.35, 1500.0, false, 90.0, false, t0).is_none());
    // Tuned after the cooldown: the CONTROLLER challenges.
    let call = compliance_tick(36.20, -5.35, 1500.0, false, 90.0, true, t0 + sec(130))
        .expect("challenge");
    assert!(!call.crew);
    assert!(call.line.contains("Say callsign"), "{}", call.line);
    // Above the zone cap: overflying is not an incursion.
    assert!(compliance_tick(36.20, -5.35, 4500.0, false, 90.0, true, t0 + sec(260)).is_none());

    // Contact made (any answered call publishes the snapshot): the zone
    // watch stands down and the clearance watch takes over.
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Parked,
        callsign: "November one seven two Sierra Papa".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: false,
    });
    // On the ground at the parking first (settles the air/ground edge —
    // and a stand is nobody's runway), then taxi onto the runway
    // uncleared: ordered off. It arrives MOVING: a stopped first sight
    // on a strip is a runway start, which is no incursion.
    let t2 = t0 + sec(400);
    assert!(compliance_tick(36.16, -5.35, 15.0, true, 0.0, true, t2 - sec(10)).is_none());
    let call = compliance_tick(36.15155, -5.342, 20.0, true, 5.0, true, t2).expect("occupancy");
    assert!(!call.crew);
    // Past midfield the position names the 27 end (same rule as the
    // occupancy geometry test).
    assert!(
        call.line.contains("on runway two seven without clearance"),
        "{}",
        call.line
    );
    assert!(call.line.contains("Exit the runway"), "{}", call.line);

    // Airborne next tick with only a taxi clearance: uncleared takeoff,
    // called once.
    OWN_STRIP.clearance.lock().unwrap().as_mut().unwrap().state = tower::State::TaxiCleared;
    let t3 = t2 + sec(70);
    let call = compliance_tick(36.152, -5.34, 300.0, false, 70.0, true, t3).expect("takeoff");
    assert!(
        call.line.contains("not cleared for takeoff"),
        "{}",
        call.line
    );

    // Back onto the runway with no landing clearance: the phone number.
    OWN_STRIP.clearance.lock().unwrap().as_mut().unwrap().state = tower::State::Inbound;
    let t4 = t3 + sec(70);
    let call = compliance_tick(36.15155, -5.342, 12.0, true, 25.0, true, t4).expect("landing");
    assert!(
        call.line.contains("number for you to copy"),
        "{}",
        call.line
    );
    // The landing berate is once, but still squatting (stopped) on the
    // runway a minute later earns the occupancy order (no clearance to be
    // there). Rolling off it is the vacate (#155).
    assert!(compliance_tick(36.15155, -5.342, 12.0, true, 10.0, true, t4 + sec(70)).is_none());
    let call = compliance_tick(36.15155, -5.342, 12.0, true, 0.0, true, t4 + sec(71))
        .expect("occupancy after uncleared landing");
    assert!(call.line.contains("Exit the runway"), "{}", call.line);

    // A LEGAL landing: ClearedToLand grants the vacate grace — sitting on
    // the runway right after touchdown is not an occupancy violation.
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    OWN_STRIP.clearance.lock().unwrap().as_mut().unwrap().state = tower::State::ClearedToLand;
    assert!(compliance_tick(36.15155, -5.342, 12.0, true, 15.0, true, t4 + sec(200)).is_none());

    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_PANEL.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

/// Issue #63, 2026-09-26: an autoland with no landing clearance touched
/// down at t 1790448341.9 and the tower's phone number came at
/// 1790448368.5, 0.2 s after the autoland handed over at a standstill:
/// the monitor skipped the watch while `autoland::ACTIVE`, so the
/// air/ground edge it needs was the last one before the arm. Ticked
/// through the approach, the call lands ON the touchdown tick, the same
/// as hand-flown; with a landing clearance it stays silent through the
/// touchdown and the rollout. The skipped-tick replay pins the old
/// delay so the regression is visible.
#[test]
fn an_uncleared_landing_is_called_at_touchdown_whoever_flies() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
    *ACTIVE_PANEL.lock().unwrap() = Some((
        "Gibraltar".into(),
        11.0,
        vec![("tower".to_string(), 131200_i64)],
    ));
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1512,
        lon_a: -5.3554,
        lat_b: 36.1517,
        lon_b: -5.3366,
    }];
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    let t0 = Instant::now();
    let at = |s: f64| t0 + std::time::Duration::from_secs_f64(s);
    // The recorded shape at 0.4 s ticks: airborne on the final, the
    // touchdown at 60 s at 146 kt, stopped 26 s later, and the autoland
    // handoff 26.4 s after the touchdown (1790448341.9 to 1790448368.3).
    let touchdown = 60.0;
    let handoff = touchdown + 26.4;
    let tick = |s: f64| {
        let on_ground = s >= touchdown;
        let gs = if on_ground {
            (146.0 - (s - touchdown) * 5.6).max(0.0)
        } else {
            146.0
        };
        let (lat, lon, alt) = if on_ground {
            (36.15145, -5.3450, 12.0)
        } else {
            (
                36.1510,
                -5.3554 - (touchdown - s) * 0.0012,
                12.0 + (touchdown - s) * 8.0,
            )
        };
        compliance_tick(lat, lon, alt, on_ground, gs, true, at(s))
    };
    let reset = |state: tower::State| {
        *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
        *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
        *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
            state,
            callsign: "November eight one six November November".into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        });
    };
    let steps: Vec<f64> = (0..=250).map(|i| i as f64 * 0.4).collect();

    // Ticked every tick (hand-flown, and now under an autoland): the
    // phone number on the touchdown tick, once.
    reset(tower::State::Inbound);
    let calls: Vec<(f64, String)> = steps
        .iter()
        .filter_map(|&s| tick(s).map(|c| (s, c.line)))
        .collect();
    assert!(!calls.is_empty(), "no call at all");
    assert!(
        (calls[0].0 - touchdown).abs() < 0.41,
        "called at {} against touchdown {touchdown}",
        calls[0].0
    );
    assert!(calls[0].1.contains("without clearance"), "{}", calls[0].1);
    assert!(
        calls[0].1.contains("number for you to copy"),
        "{}",
        calls[0].1
    );
    assert_eq!(
        calls.iter().filter(|(_, l)| l.contains("number")).count(),
        1,
        "{calls:?}"
    );

    // The old gate: skipped from the arm (20 s before the touchdown) to
    // the handoff. The call came on the first tick after it.
    reset(tower::State::Inbound);
    let late = steps
        .iter()
        .filter(|&&s| !(touchdown - 20.0..handoff).contains(&s))
        .find_map(|&s| tick(s).map(|_| s))
        .expect("the skipped watch still called it, late");
    assert!(late - touchdown > 26.0, "late by {}", late - touchdown);

    // Cleared to land: silent through the touchdown and the rollout.
    reset(tower::State::ClearedToLand);
    for &s in &steps {
        assert!(tick(s).is_none(), "cleared landing berated at {s}");
    }

    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_PANEL.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

#[test]
fn arrival_taxi_in_is_not_a_runway_incursion() {
    // The live fault (2026-07-27): after landing and getting a taxi-in
    // clearance to the stand, the tower shouted "on runway without clearance"
    // a couple of minutes later. The taxi-in sets the FSM to `Parked` (not
    // `TaxiCleared`) and ends the vacate grace early, so a crew still clearing
    // the landing runway on its own cleared route got berated. A live arrival
    // taxi-in authorises vacating the runway.
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1512,
        lon_a: -5.3554,
        lat_b: 36.1517,
        lon_b: -5.3366,
    }];
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    // Down, cleared to taxi to the stand: FSM Parked, still physically on 27.
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Parked,
        callsign: "November one seven two Sierra Papa".into(),
        runway_ident: "27".into(),
        pending_readback: None,
        holds_runway: false,
    });
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;

    // WITH the arrival taxi-in flag AND its published route: vacating the
    // landing runway is legal, well past the 120 s vacate grace — no
    // berate. The route lives on the taxiway (off the strip), the way a
    // stand route does once the vacate is under way.
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = true;
    *OWN_STRIP.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(36.1470, -5.3420), (36.1460, -5.3420)],
        via: "Alpha".into(),
        seg: vec![("A".into(), false)],
        holding_point: "Stand 1".into(),
        lead_on: vec![],
    });
    assert!(
        compliance_tick(36.15155, -5.342, 20.0, true, 0.0, true, t0 + sec(200)).is_none(),
        "a cleared arrival taxi-in must not be berated for vacating the runway"
    );

    // The bare flag WITHOUT a route is not a clearance: a stale flag that
    // survived a station swap (or a route-less reply) must not switch the
    // incursion watch off. Same position, route gone: the berate stands.
    // (Rolling, as a landed aircraft is: a stopped first sight on a strip
    // is a runway start.)
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    let call = compliance_tick(36.15155, -5.342, 20.0, true, 5.0, true, t0 + sec(300))
        .expect("occupancy berate with the flag but no published route");
    assert!(call.line.contains("without clearance"), "{}", call.line);

    // Control: drop the flag (a bare Parked state) and the SAME position IS
    // an incursion — proving the exemption, not the geometry, silenced it.
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    let call = compliance_tick(36.15155, -5.342, 20.0, true, 5.0, true, t0 + sec(400))
        .expect("occupancy berate without the taxi-in flag");
    assert!(call.line.contains("without clearance"), "{}", call.line);

    // Clean up the shared world for the next test.
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

#[test]
fn a_station_swap_retires_the_previous_fields_worksheets() {
    // A region handoff calls install() WITHOUT uninstall() (main loop,
    // take_pending_station). Anything install() forgets to clear keeps
    // acting for the field that no longer exists: an armed vector plan
    // kept steering toward the old threshold, the takeoff handoff line
    // named the old field, and a stale arrival taxi-in flag exempted
    // runway occupancy from the incursion watch at the new one.
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let st_a = lxgb();
    st_a.install();
    // Field A's leftovers.
    *OWN_STRIP.vectors.lock().unwrap() = Some(VectorPlan {
        callsign: "November one seven two Sierra Papa".into(),
        thr_lat: 36.1508,
        thr_lon: -5.3596,
        course_deg: 88.0,
        runway_spoken: "zero niner".into(),
        tower_spoken: Some("one three one decimal two".into()),
        target_alt_ft: 1500.0,
        ils: false,
        join_nm: 6.0,
        cleared: false,
        dialect: Dialect::Icao,
        last_tx: None,
        last_heading: None,
    });
    *OWN_STRIP.handoff.lock().unwrap() = Some(Handoff {
        line: "November one seven two Sierra Papa, contact Gibraltar Departure on one three \
                   zero decimal four."
            .into(),
        min_alt_ft: 711.0,
        to: "Gibraltar Departure".into(),
        khz: 130_400,
    });
    *OWN_STRIP.landing_handoff.lock().unwrap() = Some("zero niner".into());
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = true;
    // The swap: field B installs over field A.
    let st_b = lxgb();
    st_b.install();
    assert!(
        OWN_STRIP.vectors.lock().unwrap().is_none(),
        "field A's vector plan must die with its station"
    );
    assert!(
        OWN_STRIP.handoff.lock().unwrap().is_none(),
        "field A's departure handoff must die with its station"
    );
    assert!(
        OWN_STRIP.landing_handoff.lock().unwrap().is_none(),
        "field A's approach handoff must die with its station"
    );
    assert!(
        !*OWN_STRIP.taxi_to_stand.lock().unwrap(),
        "field A's arrival taxi-in flag must die with its station"
    );
    Station::uninstall();
    // And uninstall retires the same set plus the published squawk, as
    // the party-line doc promises.
    *OWN_STRIP.assigned_squawk.lock().unwrap() = Some(4512);
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = true;
    Station::uninstall();
    assert!(OWN_STRIP.assigned_squawk.lock().unwrap().is_none());
    assert!(!*OWN_STRIP.taxi_to_stand.lock().unwrap());
}

#[test]
fn a_cleared_route_crossing_is_not_an_incursion() {
    // The live fault (2026-07-14): the tower's own taxi route crossed a
    // runway, then the compliance watch berated the pilot for being "on
    // the runway without clearance" mid-crossing. A runway-flagged leg of
    // the cleared route makes the occupancy legal: silence while moving,
    // "expedite" when stopped — and the berate still fires when the route
    // has no business on that runway.
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1512,
        lon_a: -5.3554,
        lat_b: 36.1517,
        lon_b: -5.3366,
    }];
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "November one seven two Sierra Papa".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    // The cleared route: south to north straight across the runway at
    // lon -5.342, the crossing leg flagged as runway pavement.
    *OWN_STRIP.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(36.1490, -5.3420), (36.1530, -5.3420), (36.1540, -5.3420)],
        via: "Alpha".into(),
        seg: vec![("09/27".into(), true), ("A".into(), false)],
        holding_point: "A1".into(),
        lead_on: vec![],
    });
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    // Settle the air/ground edge off the runway first.
    assert!(compliance_tick(36.1490, -5.3420, 15.0, true, 10.0, true, t0).is_none());
    // Mid-crossing at taxi speed, on the cleared runway leg: silence.
    assert!(
        compliance_tick(36.15155, -5.342, 15.0, true, 10.0, true, t0 + sec(2)).is_none(),
        "a cleared crossing must not be berated"
    );
    // Stopped dead mid-crossing: "expedite", never "without clearance".
    let call = compliance_tick(36.15155, -5.342, 15.0, true, 0.0, true, t0 + sec(4))
        .expect("a stopped crossing earns a call");
    assert!(
        call.line.contains("expedite crossing runway"),
        "{}",
        call.line
    );
    assert!(!call.line.contains("without clearance"), "{}", call.line);
    // Same spot, but the route STOPS at the hold-short point (it only
    // ends NEAR the runway, never crosses): nosing onto the pavement is
    // still an incursion — the berate stands.
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    OWN_STRIP.taxi_route.lock().unwrap().as_mut().unwrap().path =
        vec![(36.1490, -5.3420), (36.1505, -5.3420)];
    assert!(compliance_tick(36.1490, -5.3420, 15.0, true, 10.0, true, t0 + sec(70)).is_none());
    let call = compliance_tick(36.15155, -5.342, 15.0, true, 10.0, true, t0 + sec(72))
        .expect("an uncleared runway entry is still an incursion");
    assert!(call.line.contains("without clearance"), "{}", call.line);

    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
}

#[test]
fn unnamed_runway_edges_still_name_the_crossing() {
    // apt.dat authors often leave runway network edges unnamed: the
    // planner's crossings list (named edges only) came out empty and the
    // clearance never said "cross runway ..." (the KPDK taxi, 2026-07-14).
    // Geometry backfills the designator from the physical strip.
    let runways = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1512,
        lon_a: -5.3554,
        lat_b: 36.1517,
        lon_b: -5.3366,
    }];
    let mut plan = nav::TaxiRoutePlan {
        via: vec!["A".into()],
        crossings: Vec::new(),
        holding_point: "A1".into(),
        path: vec![(36.1490, -5.3420), (36.1530, -5.3420), (36.1540, -5.3420)],
        seg: vec![(String::new(), true), ("A".into(), false)],
        lead_on: vec![],
    };
    mark_runway_crossings(&mut plan, &runways);
    assert_eq!(plan.crossings, vec!["09/27".to_string()]);
    // The leg is renamed and flagged too — the guide's crossing cue and
    // the incursion watch's cleared-crossing exemption key on the flag.
    assert_eq!(plan.seg[0], ("09/27".to_string(), true));
    // Idempotent: a second pass (or an already-named edge) doesn't dupe.
    mark_runway_crossings(&mut plan, &runways);
    assert_eq!(plan.crossings.len(), 1);
    // A crossing link authored as a plain TAXIWAY edge across the strip
    // (EGLL's N7, 2026-07-20: no crossing clause, no guide call): the
    // geometry names and flags it all the same.
    let mut taxiway_authored = nav::TaxiRoutePlan {
        via: vec!["N7".into()],
        crossings: Vec::new(),
        holding_point: "A1".into(),
        path: vec![(36.1490, -5.3420), (36.1530, -5.3420), (36.1540, -5.3420)],
        seg: vec![("N7".into(), false), ("A".into(), false)],
        lead_on: vec![],
    };
    mark_runway_crossings(&mut taxiway_authored, &runways);
    assert_eq!(taxiway_authored.crossings, vec!["09/27".to_string()]);
    assert_eq!(taxiway_authored.seg[0], ("09/27".to_string(), true));
    // The FINAL leg ends at the hold-short point NEAR the destination
    // strip — that proximity is never a crossing.
    let mut final_leg = nav::TaxiRoutePlan {
        via: vec!["A".into()],
        crossings: Vec::new(),
        holding_point: "A1".into(),
        path: vec![(36.1490, -5.3420), (36.1512, -5.3420)],
        seg: vec![("A".into(), false)],
        lead_on: vec![],
    };
    mark_runway_crossings(&mut final_leg, &runways);
    assert!(final_leg.crossings.is_empty(), "{:?}", final_leg.crossings);
    // Same approach to the same hold, resampled onto the paint: the legs
    // are metres long now, so the exemption has to be the route's TAIL
    // and not merely its last leg. Exempting one 5 m sliver would have
    // the route announce a crossing of the runway it is holding short of.
    // 170 m of approach to a hold 25 m short of the strip (an authored
    // zone can leave it that close), in 5 m samples: several of the last
    // legs sit inside the runway's own half-width, and each one of them
    // would name a crossing if only the very last were exempt.
    let (from, to) = (36.14980, 36.151325);
    let mut resampled = nav::TaxiRoutePlan {
        via: vec!["A".into()],
        crossings: Vec::new(),
        holding_point: "A1".into(),
        path: (0..=34)
            .map(|k| (from + (to - from) * k as f64 / 34.0, -5.3420))
            .collect(),
        seg: vec![("A".to_string(), false); 34],
        lead_on: vec![],
    };
    mark_runway_crossings(&mut resampled, &runways);
    assert!(resampled.crossings.is_empty(), "{:?}", resampled.crossings);
    // A route that never touches the pavement names nothing.
    let mut clear = nav::TaxiRoutePlan {
        via: vec!["A".into()],
        crossings: Vec::new(),
        holding_point: "A1".into(),
        path: vec![(36.1490, -5.3420), (36.1505, -5.3420)],
        seg: vec![(String::new(), false)],
        lead_on: vec![],
    };
    mark_runway_crossings(&mut clear, &runways);
    assert!(clear.crossings.is_empty());
}

/// A traffic arrival's stand route passes over a stand somebody stands on
/// (the parked aircraft its picture shows) for the next nearest; with
/// nobody given (ours) the nearest stand, as always.
#[test]
fn a_stand_route_avoiding_the_parked_goes_to_the_next_free_stand() {
    let mut nodes = std::collections::HashMap::new();
    nodes.insert(1, (36.0000, -5.0000));
    nodes.insert(2, (36.0000, -4.9980));
    let stand = |name: &str, lon: f64| nav::Stand {
        lat: 36.0003,
        lon,
        heading: 180.0,
        name: name.into(),
        kind: "gate".into(),
        accepts: vec![],
        width: None,
    };
    let net = nav::TaxiNet {
        nodes,
        edges: vec![nav::TaxiEdge {
            a: 1,
            b: 2,
            oneway: false,
            runway: false,
            name: "A".into(),
            zone_of: Vec::new(),
        }],
        stands: vec![stand("S1", -5.0000), stand("S2", -4.9980)],
        paint: vec![],
        pavement: vec![],
        stand_lines: vec![],
    };
    let (lat, lon) = (36.0000, -5.0004);
    let ours = stand_route_on(&net, &[], lat, lon, None).expect("a route");
    assert_eq!(ours.holding_point, "S1", "the nearest");
    let none = stand_route_avoiding(&net, &[], lat, lon, None, &[]).expect("a route");
    assert_eq!(none.holding_point, "S1");
    let parked = [(36.0003, -5.0000)];
    let traffic = stand_route_avoiding(&net, &[], lat, lon, None, &parked).expect("a route");
    assert_eq!(traffic.holding_point, "S2", "past the parked aircraft");
    let full = [(36.0003, -5.0000), (36.0003, -4.9980)];
    assert!(stand_route_avoiding(&net, &[], lat, lon, None, &full).is_none());
}

/// A traffic arrival stopped off the strip but nearest a network node on
/// it is routed from the nearest node clear of the strip, not back through
/// it (LFPG 09R, the traffic hour sweep 2026-10-10: an arrival 66 m off the
/// runway was routed back up its exit through the strip and held short of
/// the runway it had just left for the rest of the hour).
#[test]
fn a_traffic_stand_route_starts_clear_of_the_strip() {
    let mut nodes = std::collections::HashMap::new();
    // Node 1 on the runway's strip, node 2 clear of it.
    nodes.insert(1, (36.0000, -5.00005));
    nodes.insert(2, (36.0000, -4.9985));
    let net = nav::TaxiNet {
        nodes,
        edges: vec![nav::TaxiEdge {
            a: 1,
            b: 2,
            oneway: false,
            runway: false,
            name: "A".into(),
            zone_of: Vec::new(),
        }],
        stands: vec![nav::Stand {
            lat: 36.0003,
            lon: -4.9985,
            heading: 180.0,
            name: "S1".into(),
            kind: "gate".into(),
            accepts: vec![],
            width: None,
        }],
        paint: vec![],
        pavement: vec![],
        stand_lines: vec![],
    };
    let runway = nav::Runway {
        ident_a: "18".into(),
        ident_b: "36".into(),
        lat_a: 36.0100,
        lon_a: -5.0000,
        lat_b: 35.9900,
        lon_b: -5.0000,
        width_m: 30.0,
        surface: 1,
    };
    let rw = [runway];
    // 61 m off the centreline, nearer node 1 (on the strip) than node 2.
    let (lat, lon) = (36.0000, -4.99935);
    assert!(on_runway(&rw[0], lat, lon).is_none(), "off the strip");
    let far = [(0.0, 0.0)];
    let traffic = stand_route_avoiding(&net, &rw, lat, lon, None, &far).expect("a route");
    assert_eq!(traffic.holding_point, "S1");
    assert!(traffic.crossings.is_empty(), "{:?}", traffic.crossings);
    assert_eq!(traffic.path.first(), Some(&(lat, lon)), "from where it stands");
    // Ours plans as it always did.
    let ours = stand_route_avoiding(&net, &rw, lat, lon, None, &[]).expect("a route");
    assert_eq!(ours.holding_point, "S1");
}

/// Our own arrival passes over a stand our own fleet's parked aircraft
/// stands on (#243): the link publishes where they stand, our stand route
/// (the arrival desk's and the self taxi's) reads it; with no fleet the
/// nearest stand, as always.
#[test]
fn our_stand_route_passes_over_a_stand_our_parked_traffic_stands_on() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut nodes = std::collections::HashMap::new();
    nodes.insert(1, (36.0000, -5.0000));
    nodes.insert(2, (36.0000, -4.9980));
    let stand = |name: &str, lon: f64| nav::Stand {
        lat: 36.0003,
        lon,
        heading: 180.0,
        name: name.into(),
        kind: "gate".into(),
        accepts: vec![],
        width: None,
    };
    *TAXI_NET.lock().unwrap() = Some(std::sync::Arc::new(nav::TaxiNet {
        nodes,
        edges: vec![nav::TaxiEdge {
            a: 1,
            b: 2,
            oneway: false,
            runway: false,
            name: "A".into(),
            zone_of: Vec::new(),
        }],
        stands: vec![stand("S1", -5.0000), stand("S2", -4.9980)],
        paint: vec![],
        pavement: vec![],
        stand_lines: vec![],
    }));
    let (lat, lon) = (36.0000, -5.0004);
    publish_stands_taken(Vec::new());
    let free = plan_taxi_route_to_stand(&[], lat, lon, None, &stands_taken()).expect("a route");
    assert_eq!(free.holding_point, "S1", "no fleet: the nearest");
    publish_stands_taken(vec![(36.0003, -5.0000)]);
    let ours = plan_taxi_route_to_stand(&[], lat, lon, None, &stands_taken()).expect("a route");
    assert_eq!(ours.holding_point, "S2", "past our parked traffic");
    publish_stands_taken(Vec::new());
    *TAXI_NET.lock().unwrap() = None;
}

#[test]
fn off_route_stray_gets_a_fresh_route_from_present_position() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Testfield".into(), 36.0, -5.0));
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "November one seven two Sierra Papa".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: false,
    });
    // A tiny network: taxiway A east along 36.0000, then H1 north to the
    // holding point. The cleared route rides all of it.
    let mut nodes = std::collections::HashMap::new();
    nodes.insert(1, (36.0000, -5.0000));
    nodes.insert(2, (36.0000, -4.9990));
    nodes.insert(3, (36.0000, -4.9980));
    nodes.insert(4, (36.0010, -4.9980));
    let edge = |a: i64, b: i64, name: &str| nav::TaxiEdge {
        a,
        b,
        oneway: false,
        runway: false,
        name: name.into(),
        zone_of: Vec::new(),
    };
    *TAXI_NET.lock().unwrap() = Some(std::sync::Arc::new(nav::TaxiNet {
        nodes,
        edges: vec![edge(1, 2, "A"), edge(2, 3, "A"), edge(3, 4, "H1")],
        stands: vec![],
        paint: vec![],
        pavement: vec![],
        stand_lines: vec![],
    }));
    *OWN_STRIP.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![
            (36.0000, -5.0000),
            (36.0000, -4.9990),
            (36.0000, -4.9980),
            (36.0010, -4.9980),
        ],
        via: "Alpha, Hotel one".into(),
        seg: vec![
            ("A".into(), false),
            ("A".into(), false),
            ("H1".into(), false),
        ],
        holding_point: "H1".into(),
        lead_on: vec![],
    });
    let t0 = Instant::now();

    // 100 m north of the route at taxi speed: the challenge carries a
    // FRESH route from where the aircraft IS, not the stale original via
    // (the same via line was spoken seven times at Malaga, 2026-07-13).
    let call = compliance_tick(36.0009, -5.0000, 15.0, true, 15.0, true, t0).expect("stray");
    assert!(!call.crew);
    assert!(
        call.line.contains("off the cleared taxi route"),
        "{}",
        call.line
    );
    assert!(
        call.line
            .contains("Taxi to holding point Hotel one via Alpha"),
        "{}",
        call.line
    );
    // The fresh route is republished (the guide rebuilds from it), and
    // it starts AT the aircraft — the walk back to the network is now
    // on-route, so the berate cannot repeat while the pilot complies.
    {
        let g = OWN_STRIP.taxi_route.lock().unwrap();
        let r = g.as_ref().expect("route republished");
        assert_eq!(r.path[0], (36.0009, -5.0000));
        assert_eq!(r.holding_point, "H1");
    }
    assert!(
        compliance_tick(
            36.0009,
            -5.0000,
            15.0,
            true,
            15.0,
            true,
            t0 + std::time::Duration::from_secs(50)
        )
        .is_none(),
        "same spot is ON the fresh route"
    );

    *TAXI_NET.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

#[test]
fn landing_handoff_clears_on_clearance_and_on_touchdown() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Inbound,
        callsign: "November five five zero Charlie Romeo".into(),
        runway_ident: "27".into(),
        pending_readback: None,
        holds_runway: false,
    });
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;

    // Approach handed us off: the coach is armed, still owed while airborne
    // and not yet cleared to land.
    *OWN_STRIP.landing_handoff.lock().unwrap() = Some("two seven".into());
    assert!(compliance_tick(36.30, -5.35, 1500.0, false, 90.0, true, t0).is_none());
    assert_eq!(
        crate::party_line::landing_handoff_runway().as_deref(),
        Some("two seven"),
        "still owe the tower a final report"
    );

    // Tower clears the landing → the coach's job is done.
    OWN_STRIP.clearance.lock().unwrap().as_mut().unwrap().state = tower::State::ClearedToLand;
    compliance_tick(36.20, -5.35, 1000.0, false, 90.0, true, t0 + sec(20));
    assert!(
        crate::party_line::landing_handoff_runway().is_none(),
        "landing clearance clears the handoff"
    );

    // Re-armed but this time we touch down without it: on-ground clears it
    // so the next takeoff isn't nagged by a stale handoff.
    *OWN_STRIP.landing_handoff.lock().unwrap() = Some("two seven".into());
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    compliance_tick(36.1512, -5.3554, 12.0, true, 5.0, true, t0 + sec(40));
    assert!(
        crate::party_line::landing_handoff_runway().is_none(),
        "touchdown clears the handoff"
    );

    *ACTIVE_FIELD.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.landing_handoff.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

/// LEMG 12 (2026-09-26, a 737): landed 12, stopped in the far half of
/// the strip heading 117, and was told "you landed on runway three zero"
/// because the name was the NEARER end. The end is the one the aircraft
/// points down. Shown on the Jerez strip (02 runs 021 true).
#[test]
fn a_runway_is_named_by_the_end_the_aircraft_points_down() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Jerez".into(), 36.7446, -6.0601));
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 45.0,
        surface: 0,
        ident_a: "02".into(),
        ident_b: "20".into(),
        lat_a: 36.7349761,
        lon_a: -6.0648168,
        lat_b: 36.7542941,
        lon_b: -6.0553835,
    }];
    // Three quarters of the way up 02, rolled out heading 021.
    let (lat, lon) = (36.7494646, -6.0577402);
    assert_eq!(on_active_runway(lat, lon).as_deref(), Some("20"), "the nearer end");
    assert_eq!(on_active_runway_facing(lat, lon, Some(21.0)).as_deref(), Some("02"));
    assert_eq!(on_active_runway_facing(lat, lon, Some(201.0)).as_deref(), Some("20"));
    assert_eq!(on_active_runway_facing(lat, lon, None).as_deref(), Some("20"));
    // And the uncleared-landing line says it.
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Inbound,
        callsign: "November one two three".into(),
        runway_ident: "02".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    let t0 = Instant::now();
    assert!(compliance_tick_facing(lat, lon, 400.0, false, 140.0, true, t0, Some(21.0)).is_none());
    let call = compliance_tick_facing(
        lat, lon, 12.0, true, 20.0, true, t0 + std::time::Duration::from_secs(60), Some(21.0),
    )
    .expect("an uncleared landing");
    assert!(call.line.contains("runway zero two"), "{}", call.line);

    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

/// Issue #108, the recorded case itself: LEMG 12/30 from apt.dat, the
/// 737 (N816NN, 2026-09-26) stopped at 36.682790, -4.488170 heading
/// 117.9 true, 227 m from the 30 threshold, and at t 1790448368.5 was
/// told "you landed on runway three zero". That build predated 3b6ff11
/// (committed 20:58:36 local, the call 20:46:08), which named the
/// NEARER end; the monitor passes `psi` (true), the frame
/// `heading_a_deg` is in. With the fix the same numbers say "one two".
#[test]
fn the_lemg_landing_is_named_one_two_from_its_recorded_numbers() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Malaga".into(), 36.6749, -4.4991));
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![
        nav::Runway {
            width_m: 45.11,
            surface: 1,
            ident_a: "12".into(),
            ident_b: "30".into(),
            lat_a: 36.6910753,
            lon_a: -4.5078270,
            lat_b: 36.6795648,
            lon_b: -4.4805125,
        },
        nav::Runway {
            width_m: 45.11,
            surface: 1,
            ident_a: "13".into(),
            ident_b: "31".into(),
            lat_a: 36.6845573,
            lon_a: -4.5126253,
            lat_b: 36.6654027,
            lon_b: -4.4858140,
        },
    ];
    let (lat, lon, hdg) = (36.682790230055794, -4.488169765732041, 117.9);
    assert_eq!(on_active_runway(lat, lon).as_deref(), Some("30"), "the old name");
    assert_eq!(on_active_runway_facing(lat, lon, Some(hdg)).as_deref(), Some("12"));
    // The tower was at Parked (no landing clearance): a flight, then the
    // wheels, heading down 12.
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Parked,
        callsign: "November eight one six November November".into(),
        runway_ident: "12".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    assert!(compliance_tick_facing(36.70, -4.53, 900.0, false, 146.0, true, t0, Some(hdg)).is_none());
    let call = compliance_tick_facing(lat, lon, 38.2, true, 0.0, true, t0 + sec(60), Some(hdg))
        .expect("an uncleared landing");
    assert!(call.line.contains("landed on runway one two"), "{}", call.line);
    assert!(!call.line.contains("three zero"), "{}", call.line);

    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

/// EGKK's two strips from apt.dat: 08R/26L and the close parallel
/// 08L/26R.
fn egkk_runways() -> Vec<nav::Runway> {
    vec![
        nav::Runway {
            width_m: 46.0,
            surface: 1,
            ident_a: "08R".into(),
            ident_b: "26L".into(),
            lat_a: 51.1450993,
            lon_a: -0.2123748,
            lat_b: 51.1514994,
            lon_b: -0.1659296,
        },
        nav::Runway {
            width_m: 46.0,
            surface: 1,
            ident_a: "08L".into(),
            ident_b: "26R".into(),
            lat_a: 51.1468901,
            lon_a: -0.2126029,
            lat_b: 51.1518321,
            lon_b: -0.1767408,
        },
    ]
}

/// Issue #117, sortie 1790527340-full_auto (EGKK-EGBB, 2026-09-27): the
/// 737 began lined up on 26L at 51.1476808, -0.1935461 (0.0 kt, heading
/// 256.3), a runway start. The IFR request made contact (tower Parked)
/// and the compliance watch said "you are on runway two six left without
/// clearance. Exit the runway immediately." at +17.3 s. A stopped first
/// sight on a strip is the sim's placement, a lined-up aircraft awaiting
/// its clearance: silent at the recorded ticks, and the incursion watch
/// holds it authorised too. One that TAXIES onto a strip (first seen
/// moving) is still ordered off, and the start is spent once it leaves.
#[test]
fn a_runway_start_is_lined_up_not_an_incursion() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gatwick".into(), 51.1481, -0.1903));
    *ACTIVE_RUNWAYS.lock().unwrap() = egkk_runways();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    let t0 = Instant::now();
    let at = |s: f64| t0 + std::time::Duration::from_secs_f64(s);
    let (lat, lon, hdg) = (51.14768083914999, -0.19354614043732044, 256.3);
    let tick = |s: f64| compliance_tick_facing(lat, lon, 203.0, true, 0.0, true, at(s), Some(hdg));
    // No contact yet: the recorded ticks from +1.5 s.
    for s in [1.5, 4.0, 6.4, 9.0, 11.6, 14.5] {
        assert!(tick(s).is_none(), "{s}");
    }
    // The IFR request's answer publishes the snapshot at +17.2 s.
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Parked,
        callsign: "Whiskey Alpha Tango one seven three".into(),
        runway_ident: "26R".into(),
        pending_readback: None,
        holds_runway: false,
    });
    for s in [17.3, 17.8, 20.2, 33.1, 47.9, 62.7, 90.0, 200.0] {
        assert!(tick(s).is_none(), "the runway start berated at +{s}");
    }
    assert!(runway_occupancy_authorised(lat, lon, 0.0), "the incursion watch");

    // Off the strip, then back on it: not a start any more.
    assert!(compliance_tick_facing(51.1440, -0.1935, 203.0, true, 10.0, true, at(210.0), Some(0.0)).is_none());
    assert!(!runway_occupancy_authorised(lat, lon, 0.0));
    let call = compliance_tick_facing(lat, lon, 203.0, true, 5.0, true, at(220.0), Some(hdg))
        .expect("taxied onto the runway uncleared");
    assert!(call.line.contains("runway two six left without clearance"), "{}", call.line);

    // A fresh watch whose first sight is a taxi onto the strip: ordered off.
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    let call = compliance_tick_facing(lat, lon, 203.0, true, 8.0, true, at(400.0), Some(hdg))
        .expect("moving onto the strip is no runway start");
    assert!(call.line.contains("Exit the runway"), "{}", call.line);

    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

/// Issue #124, sortie 1790534902-full_auto (EGBB-EGKK, 2026-09-27):
/// cleared to land 26L, down at +2974 s, then taxiing down the strip at
/// 8 to 12 kt to exit ER and "runway vacated" only at +3149. The monitor's
/// incursion watch said "exit the runway immediately" / "expedite" eight
/// times from +2994 (26L, then 08R past midfield) because only a takeoff
/// clearance or line-up counted as the tower putting us there. A landing
/// holds its strip until the vacated report; the arrival's cleared
/// crossing of 08L (+3191, 12.7 kt) is legal while it moves.
#[test]
fn a_cleared_landing_holds_its_runway_until_vacated() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gatwick".into(), 51.1481, -0.1903));
    *ACTIVE_RUNWAYS.lock().unwrap() = egkk_runways();
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    let landed = |holds: bool, ident: &str| {
        *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::ClearedToLand,
            callsign: "Whiskey Alpha Tango four two three six".into(),
            runway_ident: ident.into(),
            pending_readback: None,
            holds_runway: holds,
        });
    };
    // The recorded vacate: +3000.5 (26L half), +3050.6 (08R half).
    let vacate = [
        (51.14847323089843, -0.1879083802267358, 8.4),
        (51.14796049673604, -0.1916164076685698, 9.1),
    ];
    landed(true, "26L");
    for &(lat, lon, gs) in &vacate {
        assert!(on_active_runway(lat, lon).is_some(), "on the strip");
        assert!(runway_occupancy_authorised(lat, lon, gs), "{lat},{lon}");
    }
    // Reported vacated (or a landing clearance for the other strip):
    // the strip is not ours, the watch speaks again.
    landed(false, "26L");
    assert!(!runway_occupancy_authorised(vacate[0].0, vacate[0].1, 8.4));
    landed(true, "26R");
    assert!(!runway_occupancy_authorised(vacate[0].0, vacate[0].1, 8.4));
    // Taxi-in cleared (Parked) across 08L on its route: legal moving,
    // not stopped on it.
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Parked,
        callsign: "Whiskey Alpha Tango four two three six".into(),
        runway_ident: "26L".into(),
        pending_readback: None,
        holds_runway: false,
    });
    let (clat, clon) = (51.14836698215888, -0.20092418920350943);
    assert!(on_active_runway(clat, clon).is_some(), "on 08L");
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = true;
    *OWN_STRIP.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(51.1478, -0.1990), (51.1484, -0.2009), (51.1492, -0.2020)],
        via: "Echo Romeo, Echo".into(),
        seg: vec![("ER".into(), false), ("E".into(), false)],
        holding_point: "Stand 230R".into(),
        lead_on: vec![],
    });
    assert!(runway_occupancy_authorised(clat, clon, 12.7));
    assert!(!runway_occupancy_authorised(clat, clon, 0.0), "stopped on a crossing");
    // A plain Parked aircraft on the strip with no route: an incursion.
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    assert!(!runway_occupancy_authorised(vacate[0].0, vacate[0].1, 8.4));

    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

/// EGBB 15/33 as apt.dat has it (46 m wide).
fn egbb_runways() -> Vec<nav::Runway> {
    vec![nav::Runway {
        width_m: 46.02,
        surface: 1,
        ident_a: "15".into(),
        ident_b: "33".into(),
        lat_a: 52.4656497,
        lon_a: -1.7611282,
        lat_b: 52.4428974,
        lon_b: -1.7358498,
    }]
}

/// Issue #155, sortie 1790601822-full_auto (EGKK-EGBB, 2026-09-28): an
/// autoland with no landing clearance touched down on 15 at +3472.2 and
/// the tower said "you landed on runway one five without clearance. Taxi
/// to parking". The incursion watch then said "exit the runway" at
/// +3491.5, +3507.8, +3529.4 and +3544.8 naming runway 33 (the nearer end,
/// past midfield) and the compliance watch "you are on runway one five
/// without clearance" at +3532.6, all through the vacate the tower had
/// just ordered. The strip we landed on is ours until we leave it; off it
/// and back on, it is an incursion again. The end named is the one we
/// point down.
#[test]
fn an_uncleared_landing_still_vacates_its_own_runway() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Birmingham".into(), 52.4539, -1.7480));
    *ACTIVE_RUNWAYS.lock().unwrap() = egbb_runways();
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    // The book at the touchdown: Inbound (held, never released).
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Inbound,
        callsign: "Whiskey Alpha Tango six two six".into(),
        runway_ident: "15".into(),
        pending_readback: None,
        holds_runway: false,
    });
    let t0 = Instant::now();
    let at = |s: f64| t0 + std::time::Duration::from_secs_f64(s - 3400.0);
    // The recorded final, airborne since well before.
    for s in [3400.0, 3440.0, 3470.9] {
        assert!(compliance_tick_facing(52.4596, -1.7544, 330.0, false, 136.0, true, at(s), Some(144.5))
            .is_none());
    }
    // The touchdown tick: the uncleared landing is called, on 15.
    let call = compliance_tick_facing(52.45892702734815, -1.753640601511448, 312.8, true, 133.1, true,
        at(3472.3), Some(145.3))
        .expect("the uncleared landing");
    assert!(call.line.contains("landed on runway one five without clearance"), "{}", call.line);
    // The recorded rollout and vacate down the strip (state records).
    let rollout = [
        (3491.7, 52.45386543479656, -1.748043000849525, 12.1, 145.9),
        (3508.0, 52.45329194494851, -1.7474048138516027, 11.0, 145.4),
        (3520.8, 52.45279749067367, -1.7468509083893378, 8.5, 145.3),
        (3527.8, 52.45259466624598, -1.7466236640010893, 7.0, 145.4),
        (3534.4, 52.452416156695165, -1.74651448352706, 5.5, 177.6),
        (3541.5, 52.45225506252149, -1.746556227323538, 3.8, 194.1),
    ];
    for &(s, lat, lon, gs, hdg) in &rollout {
        assert!(on_active_runway(lat, lon).is_some(), "+{s} on the strip");
        assert!(runway_occupancy_authorised(lat, lon, gs), "+{s}: the incursion watch");
        let c = compliance_tick_facing(lat, lon, 325.0, true, gs, true, at(s), Some(hdg));
        assert!(c.is_none(), "+{s}: {}", c.map(|c| c.line).unwrap_or_default());
    }
    // Stopped on it, the strip is not ours: squatting is an incursion.
    assert!(!runway_occupancy_authorised(rollout[1].1, rollout[1].2, 0.0));
    // Named by the end we point down, never the nearer 33.
    let (_, lat, lon, _, hdg) = rollout[0];
    assert_eq!(on_active_runway(lat, lon).as_deref(), Some("33"));
    assert_eq!(on_active_runway_facing(lat, lon, Some(hdg)).as_deref(), Some("15"));
    // Off the strip (the recorded taxi-in at +3595.4, 1.3 kt), then back
    // on: an incursion like any other.
    let (olat, olon) = (52.45043554899071, -1.7476458779956332);
    assert!(on_active_runway(olat, olon).is_none(), "off the strip");
    assert!(compliance_tick_facing(olat, olon, 325.0, true, 1.3, true, at(3595.4), Some(193.2)).is_none());
    assert!(!runway_occupancy_authorised(rollout[1].1, rollout[1].2, 8.0));

    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

/// Issue #155, sortie 1790601822-full_auto (EGKK 08R, 2026-09-28), from
/// the xfc log: U28680 lifted off at 13:40:31 ("departure stream ...
/// 08R", "wake clock started ... 26R"), our held release said "traffic
/// clear ... cleared for takeoff" at 13:40:47 (the clock was on the 08L
/// pavement's 26R, never the held 08R), and the party line, not ticked
/// while the release was held, narrated "Easy eight six eight zero,
/// runway zero eight right, cleared for takeoff" at 13:40:48, 17 s after
/// that liftoff and 1.6 s after our clearance. Then U28653 lifted off
/// 13:42:27, seconds after us, and was "cleared for takeoff" too. Now:
/// the wake clock sits on the departure's own end (main strip of a close
/// pair), a liftoff the party line did not see live is never narrated,
/// the runway is ours alone while lined up or cleared for takeoff, and
/// nobody departs our strip within the wake time of our own liftoff.
#[test]
fn one_departure_on_the_runway_at_a_time() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gatwick".into(), 51.1481, -0.1903));
    *ACTIVE_PANEL.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = egkk_runways();
    *GHOSTS.lock().unwrap() = None;
    *WAKE.lock().unwrap() = None;
    *WAKE_PREV.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    let t0 = Instant::now();
    let at = |s: f64| t0 + std::time::Duration::from_secs_f64(s);
    // A point along a strip, 08 end to 26 end.
    let along = |r: usize, f: f64| {
        let rw = &egkk_runways()[r];
        (rw.lat_a + f * (rw.lat_b - rw.lat_a), rw.lon_a + f * (rw.lon_b - rw.lon_a))
    };
    let ghost = |cs: &str, (lat, lon): (f64, f64), ground: bool| TcasTarget {
        callsign: cs.into(),
        lat,
        lon,
        ele_ft: if ground { 203.0 } else { 260.0 },
        on_ground: ground,
        vs_fpm: if ground { 0.0 } else { 1500.0 },
    };
    let chatter = |targets: &[TcasTarget], s: f64| {
        chatter_tick(targets, "Gatwick", 51.1481, -0.1903, 203.0, None, at(s))
    };
    let book = |state: tower::State| {
        *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
            state,
            callsign: "Whiskey Alpha Tango six two six".into(),
            runway_ident: "08R".into(),
            pending_readback: None,
            holds_runway: false,
        });
    };

    // The wake clock: U28680 rolling east on the 08L pavement lifts off.
    book(tower::State::TaxiCleared);
    wake_note_targets(&[ghost("U28680", along(1, 0.55), true)], at(0.0));
    wake_note_targets(&[ghost("U28680", along(1, 0.57), false)], at(1.0));
    assert!(wake_remaining("08R", at(17.0)).is_some(), "the held 08R waits out the wake");

    // The party line saw it on the ground, then not again for 17 s: its
    // liftoff is history, never narrated as a clearance.
    assert!(chatter(&[ghost("U28680", along(1, 0.55), true)], 0.0).is_none());
    assert!(
        chatter(&[ghost("U28680", along(1, 0.57), false)], 17.0).is_none(),
        "a liftoff 17 s old narrated"
    );

    // Cleared for takeoff (and lined up): nobody else departs our strip.
    book(tower::State::TakeoffCleared);
    assert!(chatter(&[ghost("U28653", along(0, 0.40), true)], 30.0).is_none());
    assert!(
        chatter(&[ghost("U28653", along(0, 0.42), false)], 31.0).is_none(),
        "cleared for takeoff behind our takeoff clearance"
    );
    book(tower::State::LinedUp);
    assert!(chatter(&[ghost("EZY1", along(0, 0.40), true)], 32.0).is_none());
    assert!(chatter(&[ghost("EZY1", along(0, 0.42), false)], 33.0).is_none());

    // Our own liftoff off 08R, then the book moves on (the check-in):
    // a departure off the same strip inside the wake time is withheld.
    book(tower::State::Parked);
    let (glat, glon) = along(0, 0.5);
    assert!(compliance_tick(glat, glon, 203.0, true, 140.0, true, at(100.0)).is_none());
    let _ = compliance_tick(glat + 0.001, glon + 0.004, 260.0, false, 150.0, true, at(101.0));
    assert!(chatter(&[ghost("U2865", along(0, 0.40), true)], 101.5).is_none());
    assert!(
        chatter(&[ghost("U2865", along(0, 0.42), false)], 102.6).is_none(),
        "cleared for takeoff 1.6 s after our liftoff"
    );
    // Past the wake time, a departure is narrated again.
    assert!(chatter(&[ghost("U2866", along(0, 0.40), true)], 230.0).is_none());
    let ex = chatter(&[ghost("U2866", along(0, 0.42), false)], 231.0).expect("narrated");
    assert!(ex.lines[0].text.contains("cleared for takeoff"), "{}", ex.lines[0].text);

    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *GHOSTS.lock().unwrap() = None;
    *WAKE.lock().unwrap() = None;
    *WAKE_PREV.lock().unwrap() = None;
}

/// The Jerez sortie, 2026-07-11: holding at E1 (40 m off the 45 m
/// runway's centerline) earned "exit the runway immediately", and a
/// one-tick unstick on the takeoff roll earned "you landed without
/// clearance" mid-departure. Real width + a real flight fix both.
#[test]
fn a_holding_point_is_not_the_runway_and_a_skip_is_not_a_landing() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Jerez".into(), 36.7446, -6.0601));
    // The real LEJR 02/20 100-row: 45 m wide.
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 45.0,
        surface: 0,
        ident_a: "02".into(),
        ident_b: "20".into(),
        lat_a: 36.7349761,
        lon_a: -6.0648168,
        lat_b: 36.7542941,
        lon_b: -6.0553835,
    }];
    // Geometry first: E1 is beside the pavement, the centerline is on it.
    assert_eq!(
        on_active_runway(36.753995, -6.056011),
        None,
        "holding point captured"
    );
    assert_eq!(
        on_active_runway(36.7523623, -6.0563268).as_deref(),
        Some("20")
    );

    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "November five five zero Charlie Romeo".into(),
        runway_ident: "20".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    // Holding short at E1 with only a taxi clearance: no occupancy call.
    assert!(
        compliance_tick(36.753995, -6.056011, 40.0, true, 1.0, true, t0).is_none(),
        "holding SHORT is not ON the runway"
    );

    // Cleared for takeoff, rolling; one-sample unstick, wheels back on.
    OWN_STRIP.clearance.lock().unwrap().as_mut().unwrap().state = tower::State::TakeoffCleared;
    assert!(
        compliance_tick(36.7504305, -6.0572701, 40.0, true, 60.0, true, t0 + sec(10)).is_none()
    );
    assert!(compliance_tick(36.7513, -6.0568, 40.0, false, 83.0, true, t0 + sec(11)).is_none());
    assert!(
        compliance_tick(36.7523623, -6.0563268, 40.0, true, 86.0, true, t0 + sec(12)).is_none(),
        "a takeoff-roll skip is not a landing"
    );

    // Airborne for real, then back on the strip a minute later with no
    // landing clearance: THAT is the phone number.
    assert!(
        compliance_tick(36.7450, -6.0601, 500.0, false, 100.0, true, t0 + sec(15)).is_none()
    );
    let call = compliance_tick(36.7523623, -6.0563268, 40.0, true, 40.0, true, t0 + sec(75))
        .expect("a real uncleared landing still fires");
    assert!(
        call.line.contains("number for you to copy"),
        "{}",
        call.line
    );

    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
}

#[test]
fn ghosts_speak_airline_telephony_or_nato() {
    assert_eq!(spoken_ghost("IBE3421"), "Iberia three four two one");
    assert_eq!(spoken_ghost("RYR12QT"), "Ryanair one two Quebec Tango");
    // LiveTraffic hands over IATA ids ("BA306", live 2026-07-10).
    assert_eq!(spoken_ghost("BA306"), "Speedbird three zero six");
    assert_eq!(spoken_ghost("U2841"), "Easy eight four one");
    // The Heathrow session's NATO fallbacks, now cast properly.
    assert_eq!(spoken_ghost("EW9460"), "Eurowings niner four six zero");
    assert_eq!(spoken_ghost("ME202"), "Cedar Jet two zero two");
    assert_eq!(spoken_ghost("AC869"), "Air Canada eight six niner");
    assert_eq!(spoken_ghost("VS26"), "Virgin two six");
    // A tail number: full NATO, like any callsign — "BAC12" style tails
    // must not become an airline.
    assert!(spoken_ghost("N172SP").starts_with("November one seven two"));
    // Unknown prefix: NATO fallback, never a guessed airline.
    assert!(spoken_ghost("XYZ123").starts_with("X-ray Yankee Zulu"));
}

#[test]
fn party_line_narrates_each_ghost_once() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *GHOSTS.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1512,
        lon_a: -5.3554,
        lat_b: 36.1517,
        lon_b: -5.3366,
    }];
    let (flat, flon, felev) = (36.1512, -5.3554, 11.0);
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    // An arrival descending on final, 2 nm out at 700 ft.
    let approach = |ele: f64, dist_deg: f64| TcasTarget {
        callsign: "IBE3421".into(),
        lat: 36.1512,
        lon: -5.3554 - dist_deg,
        ele_ft: ele,
        on_ground: false,
        vs_fpm: -600.0,
    };
    // First sight seeds the memory — no call without a trend.
    assert!(chatter_tick(
        &[approach(900.0, 0.04)],
        "Gibraltar",
        flat,
        flon,
        felev,
        None,
        t0
    )
    .is_none());
    // Descending on the next pass: the full exchange — the ghost's final
    // call, the clearance, the readback — aimed at 09.
    let ex = chatter_tick(
        &[approach(700.0, 0.03)],
        "Gibraltar",
        flat,
        flon,
        felev,
        None,
        t0 + sec(5),
    )
    .expect("landing exchange");
    assert_eq!(ex.lines.len(), 3);
    assert!(ex.lines[0].ghost);
    assert!(
        ex.lines[0]
            .text
            .contains("Gibraltar Tower, Iberia three four two one, final runway zero niner"),
        "{}",
        ex.lines[0].text
    );
    assert!(!ex.lines[1].ghost);
    assert!(
        ex.lines[1]
            .text
            .contains("runway zero niner, cleared to land"),
        "{}",
        ex.lines[1].text
    );
    assert!(ex.lines[2].ghost);
    assert!(
        ex.lines[2].text.starts_with("Cleared to land"),
        "{}",
        ex.lines[2].text
    );
    assert_eq!(ex.ghost_seed, crate::speaker::station_voice_seed("IBE3421"));
    // Once per ghost — the rest of the approach stays quiet.
    assert!(chatter_tick(
        &[approach(500.0, 0.02)],
        "Gibraltar",
        flat,
        flon,
        felev,
        None,
        t0 + sec(10)
    )
    .is_none());

    // A second arrival aiming at a strip the PLAYER occupies: "continue
    // approach", never a narrated clearance over the player's head —
    // and only once.
    let inbound2 = |ele: f64, dist_deg: f64| TcasTarget {
        callsign: "BA435".into(),
        lat: 36.1512,
        lon: -5.3554 - dist_deg,
        ele_ft: ele,
        on_ground: false,
        vs_fpm: -600.0,
    };
    assert!(chatter_tick(
        &[inbound2(900.0, 0.05)],
        "Gibraltar",
        flat,
        flon,
        felev,
        Some("27"),
        t0 + sec(12)
    )
    .is_none());
    let ex = chatter_tick(
        &[inbound2(700.0, 0.04)],
        "Gibraltar",
        flat,
        flon,
        felev,
        Some("27"),
        t0 + sec(14),
    )
    .expect("continue exchange");
    assert_eq!(ex.lines.len(), 2);
    assert!(
        ex.lines[1].text.contains("continue approach"),
        "{}",
        ex.lines[1].text
    );
    assert!(
        !ex.lines[1].text.contains("cleared to land"),
        "{}",
        ex.lines[1].text
    );
    assert!(chatter_tick(
        &[inbound2(600.0, 0.03)],
        "Gibraltar",
        flat,
        flon,
        felev,
        Some("27"),
        t0 + sec(16)
    )
    .is_none());
    // Player vacates: the clearance can now come.
    let ex = chatter_tick(
        &[inbound2(500.0, 0.02)],
        "Gibraltar",
        flat,
        flon,
        felev,
        None,
        t0 + sec(18),
    )
    .expect("clearance after vacating");
    assert!(
        ex.lines[1].text.contains("cleared to land"),
        "{}",
        ex.lines[1].text
    );

    // A departure: ground last pass, airborne now → cleared for takeoff.
    let ground = TcasTarget {
        callsign: "VLG88AB".into(),
        lat: 36.1514,
        lon: -5.345,
        ele_ft: 11.0,
        on_ground: true,
        vs_fpm: 0.0,
    };
    let rotated = TcasTarget {
        callsign: "VLG88AB".into(),
        lat: 36.1514,
        lon: -5.345,
        ele_ft: 100.0,
        on_ground: false,
        vs_fpm: 0.0,
    };
    assert!(chatter_tick(
        &[ground],
        "Gibraltar",
        flat,
        flon,
        felev,
        None,
        t0 + sec(15)
    )
    .is_none());
    let ex = chatter_tick(
        &[rotated],
        "Gibraltar",
        flat,
        flon,
        felev,
        None,
        t0 + sec(20),
    )
    .expect("takeoff exchange");
    assert_eq!(ex.lines.len(), 2);
    assert!(!ex.lines[0].ghost);
    assert!(ex.lines[0].text.contains("Vueling"), "{}", ex.lines[0].text);
    assert!(
        ex.lines[0].text.contains("cleared for takeoff"),
        "{}",
        ex.lines[0].text
    );
    assert!(ex.lines[1].ghost);
    assert!(
        ex.lines[1].text.starts_with("Cleared for takeoff"),
        "{}",
        ex.lines[1].text
    );

    *GHOSTS.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

/// ETND's two strips as apt.dat has them: 08L/26R, 446 m of grass, and
/// 08/26, 1,240 m of asphalt beside it.
fn etnd_runways() -> Vec<nav::Runway> {
    vec![
        nav::Runway {
            ident_a: "08L".into(),
            ident_b: "26R".into(),
            lat_a: 52.5862935,
            lon_a: 8.3393953,
            lat_b: 52.5866670,
            lon_b: 8.3459906,
            width_m: 50.0,
            surface: 3,
        },
        nav::Runway {
            ident_a: "08".into(),
            ident_b: "26".into(),
            lat_a: 52.5850000,
            lon_a: 8.3318780,
            lat_b: 52.5859870,
            lon_b: 8.3501930,
            width_m: 45.11,
            surface: 1,
        },
    ]
}

/// "Request runway ..." in the tower matrix (ETND 2026-09-24): an
/// assignable runway is granted as the taxi to it and becomes the runway
/// in use (published on the party line for the crew); the grass strip
/// the field does not hand out, a runway the field lacks and words with
/// no runway in them are refused and move nothing.
#[test]
fn a_runway_request_is_granted_or_refused_by_the_field() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let wx = calm();
    let mut st = lxgb();
    st.apt.name = "Diepholz".into();
    st.apt.runways = etnd_runways();
    let ask = |st: &mut Station, words: &str| {
        let intent = tower::parse_intent(words);
        assert_eq!(intent, tower::Intent::RequestRunway, "{words}");
        st.note_stated_runway(words, intent);
        let mut ctx = scripted_ctx(st, wx);
        ctx.on_ground = true;
        st.apply_runway_request(&mut ctx, None);
        st.route_keyed(words, Service::Tower, &ctx, wx)
    };

    // The grass strip is refused: a tower does not hand it out.
    let r = ask(&mut st, "request runway two six right");
    assert!(r.contains("unable runway two six right, that runway is not available"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::Parked);
    // A runway the field lacks.
    let r = ask(&mut st, "request runway three three");
    assert!(r.contains("unable runway three three, no such runway here"), "{r}");
    let r = ask(&mut st, "request runway zero niner");
    assert!(r.contains("unable runway zero niner"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::Parked);

    // The paved runway, asked for by length: granted, and it is the
    // runway in use from here, on the party line too.
    let r = ask(&mut st, "unable two six right, request runway zero eight for length");
    assert!(
        r.contains("runway zero eight approved, taxi to holding point"),
        "{r}"
    );
    assert!(r.contains("runway zero eight"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::TaxiCleared);
    assert_eq!(st.runway_in_use.as_deref(), Some("08"));
    assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("08"));
    // The pick holds it through the taxi, whatever the calm air says.
    assert_eq!(st.pick_runway(260.0, 3.0), "08");

    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
}

/// Liege 2026-09-22 (history "Still owed"): the tower learned its runway
/// was free from our taxi request, not from the "runway vacated" report
/// before it. Landed and off the pavement, before the report a ghost's
/// departure off our strip goes un-narrated (nobody else is given a
/// takeoff on a runway the book still gives us); after it the next
/// departure is released, the pick is free again, and the bare
/// "request taxi" is still the taxi IN.
#[test]
fn the_vacated_report_frees_the_strip_before_the_taxi_request() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *GHOSTS.lock().unwrap() = None;
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    *ACTIVE_RUNWAYS.lock().unwrap() = st.apt.runways.clone();
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    // Landed on 27, rolled off onto the apron south of the strip.
    let r = st.route_keyed("final runway two seven", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared to land"), "{r}");
    st.desk.twr.gate("cleared to land runway two seven", tower::Intent::Unknown, &ctx);
    assert_eq!(st.desk.twr.state, tower::State::ClearedToLand);
    let (off_lat, off_lon) = (36.1440, -5.3500);
    assert!(on_active_runway(off_lat, off_lon).is_none(), "the apron is off the strip");
    let held = player_blocked_strip(true, off_lat, off_lon);
    assert!(held.is_some(), "landed and not reported off: the book holds the strip");
    assert!(player_blocked_strip(false, off_lat, off_lon).is_none(), "airborne holds nothing");

    let (flat, flon, felev) = (36.1512, -5.3554, 11.0);
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    let ghost = |cs: &str, ele: f64, ground: bool| TcasTarget {
        callsign: cs.into(),
        lat: 36.1514,
        lon: -5.345,
        ele_ft: ele,
        on_ground: ground,
        vs_fpm: 0.0,
    };
    let tick = |t: TcasTarget, held: Option<&str>, at: Instant| {
        chatter_tick(&[t], "Gibraltar", flat, flon, felev, held, at)
    };
    assert!(tick(ghost("VLG88AB", 11.0, true), held.as_deref(), t0).is_none());
    assert!(
        tick(ghost("VLG88AB", 100.0, false), held.as_deref(), t0 + sec(5)).is_none(),
        "no takeoff narrated off a strip the book still gives us"
    );

    // The report: acknowledged, the state stands, the strip is free.
    let r = st.route_keyed("runway vacated", Service::Tower, &ctx, wx);
    assert!(r.contains("roger, thank you"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::ClearedToLand);
    assert!(st.desk.twr.runway_vacated());
    let snap_holds = OWN_STRIP.clearance.lock().unwrap().as_ref().map(|c| c.holds_runway);
    assert_eq!(snap_holds, Some(false), "published for the watches");
    let held = player_blocked_strip(true, off_lat, off_lon);
    assert!(held.is_none(), "{held:?}");
    assert!(tick(ghost("RYR12", 11.0, true), held.as_deref(), t0 + sec(10)).is_none());
    let ex = tick(ghost("RYR12", 100.0, false), held.as_deref(), t0 + sec(15))
        .expect("the next departure is released");
    assert!(ex.lines[0].text.contains("cleared for takeoff"), "{}", ex.lines[0].text);
    // The pick no longer freezes on a landing that is over.
    assert_eq!(st.pick_runway(90.0, 20.0), "09", "the wind turns a freed field round");

    // Still the taxi IN, never a departure clearance.
    let r = st.route_keyed("request taxi", Service::Tower, &ctx, wx);
    assert!(!r.contains("holding point"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::Parked);

    *GHOSTS.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
}

/// The order reaches every loop that flies it, whoever reads first. LEMG
/// 31 (sortie 1790584715-full_auto, 2026-09-28): the order at t
/// 1790585525.5 was taken by the main loop's FSM sync (the clearance went
/// ClearedToLand to Inbound at 1527.1) and the circuit flying the final
/// never saw it; autoland, armed at 1555.6, drained the flag at its arm.
/// The one flag had two consumers and the main loop's ~150 ms poll beat
/// autoland's ~400 ms tick every time: no `ordered_go_around` in the logs.
#[test]
fn a_go_around_order_reaches_every_loop_that_flies_it() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // No own fix: the judgement falls back to the order (unmeasured time).
    OWN_STRIP.own_fixes.lock().unwrap().clear();
    *STRIP_SEEN.lock().unwrap() = None;
    // LEMG 13/31, the recorded clearance: runway 31, cleared to land.
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 45.0,
        surface: 1,
        ident_a: "13".into(),
        ident_b: "31".into(),
        lat_a: 36.6853,
        lon_a: -4.5093,
        lat_b: 36.6653,
        lon_b: -4.4833,
    }];
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    *OWN_STRIP.pending_go_around.lock().unwrap() = false;
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::ClearedToLand,
        callsign: "November eight one six November November".into(),
        runway_ident: "31".into(),
        pending_readback: None,
        holds_runway: false,
    });
    // A departure on the far end of the same strip.
    let on_the_strip = TcasTarget {
        callsign: "VLG8176".into(),
        lat: 36.6803,
        lon: -4.5028,
        ele_ft: 50.0,
        on_ground: true,
        vs_fpm: 0.0,
    };
    // Armed before the order: the circuit's watch and autoland's.
    let mut circuit = crate::party_line::GoAroundOrders::arm();
    let mut autoland = crate::party_line::GoAroundOrders::arm();
    let call = go_around_tick(
        std::slice::from_ref(&on_the_strip),
        736.0,
        false,
        true,
        Instant::now(),
    )
    .expect("the recorded order");
    assert!(call.line.contains("report downwind"), "{}", call.line);
    // The main loop syncs the FSM first, as it did live...
    assert!(crate::party_line::take_pending_go_around());
    // ...and both loops still see the order, once each.
    assert!(circuit.take(), "the circuit flying the final sees it");
    assert!(!circuit.take(), "once");
    assert!(autoland.take(), "a second reader is not robbed");
    // A loop armed after the order does not fly an old one.
    let mut later = crate::party_line::GoAroundOrders::arm();
    assert!(!later.take());
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
}

#[test]
fn go_around_ordered_when_a_ghost_takes_the_runway() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // No own fix: the judgement falls back to the order (unmeasured time).
    OWN_STRIP.own_fixes.lock().unwrap().clear();
    *STRIP_SEEN.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1512,
        lon_a: -5.3554,
        lat_b: 36.1517,
        lon_b: -5.3366,
    }];
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    *OWN_STRIP.pending_go_around.lock().unwrap() = false;
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::ClearedToLand,
        callsign: "November one seven two Sierra Papa".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: false,
    });
    let ghost_on_runway = TcasTarget {
        callsign: "IBE3421".into(),
        lat: 36.15155,
        lon: -5.342,
        ele_ft: 11.0,
        on_ground: true,
        vs_fpm: 0.0,
    };
    let t0 = Instant::now();
    // Cleared to land, short final, runway occupied: the order, by name.
    let call = go_around_tick(
        std::slice::from_ref(&ghost_on_runway),
        400.0,
        false,
        true,
        t0,
    )
    .expect("go-around order");
    assert!(!call.crew);
    assert!(
        call.line.contains("go around, traffic on the runway"),
        "{}",
        call.line
    );
    // The order hands the FSM sync to the main loop, like the hold release.
    assert!(
        *OWN_STRIP.pending_go_around.lock().unwrap(),
        "order must arm the FSM sync"
    );
    *OWN_STRIP.pending_go_around.lock().unwrap() = false;
    // Cooldown: not machine-gunned.
    assert!(go_around_tick(
        std::slice::from_ref(&ghost_on_runway),
        350.0,
        false,
        true,
        t0
    )
    .is_none());
    assert!(
        !*OWN_STRIP.pending_go_around.lock().unwrap(),
        "a swallowed repeat must not re-arm"
    );
    // Not tuned: the CREW calls it — and the tower's cancel is just as real.
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    let call = go_around_tick(
        std::slice::from_ref(&ghost_on_runway),
        400.0,
        false,
        false,
        t0,
    )
    .expect("crew go-around");
    assert!(call.crew);
    assert!(
        *OWN_STRIP.pending_go_around.lock().unwrap(),
        "crew-voiced order still voids the clearance"
    );
    *OWN_STRIP.pending_go_around.lock().unwrap() = false;
    // No landing clearance in effect: not the tower's call to cancel.
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    OWN_STRIP.clearance.lock().unwrap().as_mut().unwrap().state = tower::State::Inbound;
    assert!(go_around_tick(
        std::slice::from_ref(&ghost_on_runway),
        400.0,
        false,
        true,
        t0
    )
    .is_none());
    // Too high: the conflict is not yet real.
    OWN_STRIP.clearance.lock().unwrap().as_mut().unwrap().state = tower::State::ClearedToLand;
    assert!(go_around_tick(
        std::slice::from_ref(&ghost_on_runway),
        2000.0,
        false,
        true,
        t0
    )
    .is_none());
    // A ghost on the PARALLEL strip is not our conflict: no cancel.
    ACTIVE_RUNWAYS.lock().unwrap().push(nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09L".into(),
        ident_b: "27R".into(),
        lat_a: 36.1692,
        lon_a: -5.3554,
        lat_b: 36.1697,
        lon_b: -5.3366,
    });
    let ghost_on_parallel = TcasTarget {
        callsign: "IBE9".into(),
        lat: 36.16955,
        lon: -5.342,
        ele_ft: 11.0,
        on_ground: true,
        vs_fpm: 0.0,
    };
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    assert!(
        go_around_tick(
            std::slice::from_ref(&ghost_on_parallel),
            400.0,
            false,
            true,
            t0
        )
        .is_none(),
        "parallel-runway squatter must not cancel our clearance"
    );

    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

/// A point on LEMG runway 13's frame: `along` metres past the 13
/// threshold (negative = on the final), `cross` to the left.
fn lemg13_point(along: f64, cross: f64) -> (f64, f64) {
    let r = &lemg_runways()[1];
    let f = LandingFrame::new(r, "13").unwrap();
    let x = along * f.u.0 - cross * f.u.1;
    let y = along * f.u.1 + cross * f.u.0;
    (f.lat0 + y / f.m_lat, f.lon0 + x / f.m_lon)
}

fn ghost_at(cs: &str, (lat, lon): (f64, f64)) -> TcasTarget {
    TcasTarget {
        callsign: cs.into(),
        lat,
        lon,
        ele_ft: 50.0,
        on_ground: true,
        vs_fpm: 0.0,
    }
}

/// How a runway occupant leaves, from its track: the question the
/// go-around order and the landing release both ask (LEMG 13).
#[test]
fn a_runway_occupant_is_judged_by_how_it_moves() {
    let r = &lemg_runways()[1];
    let at = |along: f64, cross: f64, t: f64| {
        let (lat, lon) = lemg13_point(along, cross);
        (lat, lon, t)
    };
    // Standing at the threshold we land on: a departure lined up, off
    // one roll after it starts.
    let lined = strip_exit(r, "13", &[at(60.0, 0.0, -2.0), at(60.0, 0.0, 0.0)]);
    assert_eq!(lined, StripExit::LinedUp(37.5));
    // First sight there: the same.
    assert_eq!(
        strip_exit(r, "13", &[at(60.0, 0.0, 0.0)]),
        StripExit::LinedUp(37.5)
    );
    // Standing mid-runway: nobody can say when it goes.
    assert_eq!(
        strip_exit(r, "13", &[at(1600.0, 0.0, -2.0), at(1600.0, 0.0, 0.0)]),
        StripExit::Stays
    );
    // Rolling our way and accelerating: a departure, off at its liftoff.
    let roll = strip_exit(
        r,
        "13",
        &[
            at(100.0, 0.0, -2.0),
            at(105.0, 0.0, -1.0),
            at(112.0, 0.0, 0.0),
        ],
    );
    match roll {
        StripExit::Clearing(s) => assert!((30.0..36.0).contains(&s), "{s}"),
        other => panic!("{other:?}"),
    }
    // Fast and slowing: the lander ahead rolling out, off at a turn-off.
    let rollout = strip_exit(
        r,
        "13",
        &[
            at(900.0, 0.0, -2.0),
            at(935.0, 0.0, -1.0),
            at(965.0, 0.0, 0.0),
        ],
    );
    assert_eq!(rollout, StripExit::Clearing(ROLLOUT_CLEAR_S));
    // Backtracking toward us: oncoming, no telling when it is off.
    assert_eq!(
        strip_exit(r, "13", &[at(500.0, 0.0, -2.0), at(490.0, 0.0, 0.0)]),
        StripExit::Oncoming
    );
    // Vacating to the side at 5 m/s from 10 m left of the centreline:
    // off at the left edge (22.6 + 5 margin - 10) in about 3.5 s.
    match strip_exit(r, "13", &[at(2000.0, 0.0, -2.0), at(2000.0, 10.0, 0.0)]) {
        StripExit::Clearing(s) => assert!((3.0..4.0).contains(&s), "{s}"),
        other => panic!("{other:?}"),
    }
    // The judgement: nothing is due outside a minute of the threshold.
    assert!(!go_around_due(Some(81.0), &[StripExit::Stays]));
    assert!(go_around_due(Some(55.0), &[StripExit::Stays]));
    // A lined-up departure is given its roll until the roll no longer fits.
    assert!(!go_around_due(Some(50.0), &[StripExit::LinedUp(37.5)]));
    assert!(go_around_due(Some(45.0), &[StripExit::LinedUp(37.5)]));
    // Unmeasured: the order, as before.
    assert!(go_around_due(None, &[StripExit::LinedUp(37.5)]));
    assert!(!go_around_due(Some(10.0), &[]));
    // The release: clear, or everyone moving off in time; never a
    // standing aircraft.
    assert!(landing_release_due(Some(80.0), &[]));
    assert!(landing_release_due(None, &[]));
    assert!(landing_release_due(
        Some(80.0),
        &[StripExit::Clearing(34.0)]
    ));
    assert!(!landing_release_due(
        Some(40.0),
        &[StripExit::Clearing(34.0)]
    ));
    assert!(!landing_release_due(
        Some(80.0),
        &[StripExit::LinedUp(37.5)]
    ));
    assert!(!landing_release_due(None, &[StripExit::Clearing(5.0)]));
}

/// Issue #149, the recorded sequence: LEMG 13, sortie
/// 1790594471-full_auto (2026-09-28). The circuit reported final at
/// 3.8 nm and 722 ft (t 1790595127.0); the tower cleared it to land and,
/// 0.8 s later (t 1790595128.1), ordered the go-around for traffic on the
/// runway: the departure TO4611, which lifted off 13 at t 1790595209.2,
/// 81 s after the order, about as long again as we had to fly. The final
/// is continued, the order comes only when the traffic will still be
/// there when we arrive, and the clearance once the strip clears.
#[test]
fn lemg13_the_tower_continues_the_final_and_clears_it_when_the_runway_clears() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let reset = || {
        *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
        *OWN_STRIP.clearance.lock().unwrap() = None;
        *OWN_STRIP.landing_hold.lock().unwrap() = None;
        *OWN_STRIP.pending_landing_release.lock().unwrap() = None;
        *OWN_STRIP.pending_go_around.lock().unwrap() = false;
        *OWN_STRIP.go_around_last.lock().unwrap() = None;
        *STRIP_SEEN.lock().unwrap() = None;
        OWN_STRIP.own_fixes.lock().unwrap().clear();
        *OWN_STRIP.landing_handoff.lock().unwrap() = None;
    };
    reset();
    *ACTIVE_RUNWAYS.lock().unwrap() = lemg_runways();
    let mut st = lxgb();
    st.apt.name = "Malaga".into();
    st.apt.elev_ft = 52.0;
    st.apt.runways = lemg_runways();
    st.runway_in_use = Some("13".into());
    st.desk.circuits = true;
    st.desk.twr.state = tower::State::Inbound;
    let wx = Wx {
        wind_from_deg: 130.0,
        wind_kts: 5.0,
    };
    let cs = "November eight one six November November";
    let mut ctx = st.make_ctx(
        wx,
        "QNH one zero one niner".into(),
        cs.into(),
        false,
        false,
        None,
        true, // TO4611 on runway 13
        None,
    );
    assert_eq!(ctx.runway, "one three");
    // The crew's final report, as transmitted.
    let r = st.route_keyed(
        "november eight one six november november, final runway one three",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("continue approach runway one three"), "{r}");
    assert!(!r.contains("cleared to land"), "{r}");
    assert!(crate::party_line::landing_hold_armed());
    assert_eq!(
        crate::party_line::clearance_state(),
        Some(tower::State::Inbound)
    );
    let base = Instant::now();
    let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
    // Our recorded fixes: t 1790595122.3 and 1128.2 (base on the turn
    // to final, 7.3 km from the threshold, 174 kt).
    note_own_fix(36.7447615, -4.5512901, false, at(0.0));
    note_own_fix(36.7408962, -4.5555152, false, at(5.9));
    let own = own_time_to_threshold(&lemg_runways()[1], "13", at(5.9)).unwrap();
    assert!((3.8..4.1).contains(&own.0), "{own:?}");
    assert!((75.0..90.0).contains(&own.1), "{own:?}");
    // TO4611 standing on the runway at the 13 end, two looks.
    let to4611 = ghost_at("TO4611", lemg13_point(60.0, 0.0));
    assert!(
        go_around_tick(std::slice::from_ref(&to4611), 736.0, false, true, at(4.9)).is_none()
    );
    assert!(
        go_around_tick(std::slice::from_ref(&to4611), 722.8, false, true, at(5.9)).is_none(),
        "no order 80 s out: the recorded go-around at 3.8 nm is gone"
    );
    assert!(!*OWN_STRIP.pending_go_around.lock().unwrap());
    // Nor the clearance: a lined-up departure is not a clear runway.
    assert!(landing_hold_tick(std::slice::from_ref(&to4611), false, at(5.9)).is_none());
    assert!(crate::party_line::landing_hold_armed());

    // Case A: it rolls in time. Brake release, then 5, 7 m/s and
    // accelerating while we are 3 nm out at 150 kt.
    let own_at = |d_m: f64, t: f64| {
        let (lat, lon) = lemg13_point(-d_m, 0.0);
        note_own_fix(lat, lon, false, at(t));
    };
    let mut released = None;
    for (along, t) in [(60.0, 18.0), (60.0, 19.0), (60.0, 20.0), (66.0, 21.0)] {
        own_at(3.0 * 1852.0 + (21.0 - t) * 77.0, t);
        let g = ghost_at("TO4611", lemg13_point(along, 0.0));
        assert!(go_around_tick(std::slice::from_ref(&g), 700.0, false, true, at(t)).is_none());
        if let Some(line) = landing_hold_tick(std::slice::from_ref(&g), false, at(t)) {
            released.get_or_insert((t, line));
        }
    }
    let (t, line) = released.expect("cleared once the departure rolls and will be off in time");
    assert_eq!(
        t, 21.0,
        "at the first look that sees it rolling, not while it stood"
    );
    assert_eq!(
        line,
        format!(
            "{cs}, runway one three, wind one three zero degrees, five knots, cleared to land."
        )
    );
    assert!(!crate::party_line::landing_hold_armed());
    assert_eq!(
        crate::party_line::clearance_state(),
        Some(tower::State::ClearedToLand)
    );
    let owed = crate::party_line::readback_owed().expect("the readback is owed");
    assert_eq!(owed, format!("Cleared to land runway one three, {cs}."));
    // The crew reads it back: the tower knows it gave it.
    ctx.runway_occupied = false;
    let r = st.route_keyed(owed.trim_end_matches('.'), Service::Tower, &ctx, wx);
    assert_eq!(r, "", "the readback is accepted");
    assert_eq!(st.desk.twr.state, tower::State::ClearedToLand);
    assert_eq!(crate::party_line::readback_owed(), None);

    // Case B: it never rolls. Cleared, a lined-up aircraft is given its
    // roll until the roll no longer fits before we arrive; then the order.
    *STRIP_SEEN.lock().unwrap() = None;
    OWN_STRIP.own_fixes.lock().unwrap().clear();
    let standing = ghost_at("TO4611", lemg13_point(60.0, 0.0));
    own_at(2.6 * 1852.0 + 77.0, 40.0);
    own_at(2.6 * 1852.0, 41.0);
    assert!(
        go_around_tick(
            std::slice::from_ref(&standing),
            650.0,
            false,
            true,
            at(41.0)
        )
        .is_none(),
        "2.6 nm, 62 s: sequencing, not a go-around"
    );
    own_at(2.0 * 1852.0 + 77.0, 48.0);
    own_at(2.0 * 1852.0, 49.0);
    assert!(
        go_around_tick(
            std::slice::from_ref(&standing),
            600.0,
            false,
            true,
            at(49.0)
        )
        .is_none(),
        "2.0 nm, 48 s: a roll starting now still clears"
    );
    own_at(1.8 * 1852.0 + 77.0, 50.5);
    own_at(1.8 * 1852.0, 51.5);
    let call = go_around_tick(
        std::slice::from_ref(&standing),
        560.0,
        false,
        true,
        at(51.5),
    )
    .expect("1.8 nm, 43 s, still standing: the order");
    assert!(
        call.line.contains("go around, traffic on the runway"),
        "{}",
        call.line
    );
    assert!(*OWN_STRIP.pending_go_around.lock().unwrap());
    reset();
}

/// A point on LEMG runway 30's frame (apt.dat geometry): `along` metres
/// past the 30 threshold (negative = on the final), `cross` to the left.
fn lemg30_point(along: f64, cross: f64) -> (f64, f64) {
    let r = &lemg_runways()[0];
    let f = LandingFrame::new(r, "30").unwrap();
    let x = along * f.u.0 - cross * f.u.1;
    let y = along * f.u.1 + cross * f.u.0;
    (f.lat0 + y / f.m_lat, f.lon0 + x / f.m_lon)
}

/// Every static the issue #162 tests touch, cleared.
fn reset_runway_watch() {
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.landing_hold.lock().unwrap() = None;
    *OWN_STRIP.pending_landing_release.lock().unwrap() = None;
    *OWN_STRIP.pending_go_around.lock().unwrap() = false;
    *OWN_STRIP.pending_resequence.lock().unwrap() = None;
    *OWN_STRIP.resequenced_order.lock().unwrap() = None;
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    *STRIP_SEEN.lock().unwrap() = None;
    *STANDING.lock().unwrap() = None;
    OWN_STRIP.own_fixes.lock().unwrap().clear();
    *OWN_STRIP.landing_handoff.lock().unwrap() = None;
    *TOWER_WIND.lock().unwrap() = None;
    *SURFACE_WIND.lock().unwrap() = None;
    *ACTIVE_FIELD.lock().unwrap() = None;
    *DEPARTURE_STREAM.lock().unwrap() = None;
    *ARRIVAL_STREAM.lock().unwrap() = None;
}

const SR22: &str = "November five five zero Charlie Romeo";

/// The SR22 of sortie 1790610218-approach cleared to land LEMG 30 in
/// the recorded wind (300 at 4), at the order's 0.95 nm (34 s at the
/// recorded 100 kt) when `t` is given.
fn lemg30_cleared(runways: Vec<nav::Runway>, wind: (f64, f64)) {
    reset_runway_watch();
    *ACTIVE_RUNWAYS.lock().unwrap() = runways;
    *ACTIVE_FIELD.lock().unwrap() = Some(("Malaga".into(), 36.6749, -4.4991));
    set_surface_wind(36.6749, -4.4991, Some(wind));
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::ClearedToLand,
        callsign: SR22.into(),
        runway_ident: "30".into(),
        pending_readback: None,
        holds_runway: true,
    });
}

fn own_on_lemg30_final(nm: f64, at: Instant) {
    let (lat, lon) = lemg30_point(-nm * 1852.0 - 51.4, 0.0);
    note_own_fix(lat, lon, false, at - std::time::Duration::from_secs(1));
    let (lat, lon) = lemg30_point(-nm * 1852.0, 0.0);
    note_own_fix(lat, lon, false, at);
}

/// Issue #162, part 1: "on the runway" is the pavement. LEMG 12/30 is
/// 45.1 m wide and 2,755 m long; the incursion watch's capture
/// (`on_runway`: half-width + 10 m, 2 % past each end) still takes a
/// ghost 30 m off the centreline or 50 m past the 12 end, the landing
/// judgement does not.
#[test]
fn a_runway_occupant_is_on_the_pavement_not_the_strip_area() {
    let r = &lemg_runways()[0];
    let on = |along: f64, cross: f64| {
        let (lat, lon) = lemg30_point(along, cross);
        (
            on_runway(r, lat, lon).is_some(),
            on_runway_pavement(r, lat, lon).is_some(),
        )
    };
    assert_eq!(on(1400.0, 0.0), (true, true), "on the centreline");
    assert_eq!(on(1400.0, 25.0), (true, true), "a mainwheel on the edge");
    assert_eq!(
        on(1400.0, 30.0),
        (true, false),
        "30 m out is off the pavement"
    );
    assert_eq!(on(1400.0, 90.0), (false, false), "the holding point");
    assert_eq!(
        on(2755.0 + 50.0, 0.0),
        (true, false),
        "50 m past the 12 end"
    );
    assert_eq!(on(-20.0, 0.0), (true, true), "the 30 threshold's overrun");
}

/// Issue #162, the recorded order: SR22 on LEMG 30 final, 0.95 nm, 34 s
/// (t 1790611040.7), wind 300 at 4. No ghost position was on file; the
/// timing says what VL1834 was: not on the strip at the final report
/// (t 1790610897.9, cleared to land) and no `go_around_deferred` while
/// we were under 800 ft and more than 60 s out, so it reached the
/// pavement within the last half minute, while the live stream was
/// landing 12 (the party line's Ryanair arrivals). A 12 lander touching
/// down head-on. Its track now says so (the airborne samples before the
/// wheels give it a velocity at first contact: `oncoming`), the order
/// still goes (it cannot be off in time), and it names the runway the
/// stream is using: "expect runway one two". The FSM sync moves the
/// runway in use and the published clearance, and the crew's "going
/// around" is answered with it.
#[test]
fn lemg30_a_head_on_lander_sends_us_round_and_we_are_resequenced_to_twelve() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    lemg30_cleared(lemg_runways(), (300.0, 4.0));
    let base = Instant::now();
    let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
    // VL1834 on the 12 final at 140 kt, then on the wheels near the 12
    // threshold, rolling toward us.
    let lander = |along: f64, on_ground: bool| {
        let (lat, lon) = lemg30_point(along, 0.0);
        TcasTarget {
            callsign: "VL1834".into(),
            lat,
            lon,
            ele_ft: if on_ground { 50.0 } else { 120.0 },
            on_ground,
            vs_fpm: if on_ground { 0.0 } else { -700.0 },
        }
    };
    let len = 2755.0;
    for (t, along, gnd) in [
        (0.0, len + 206.0, false),
        (1.0, len + 134.0, false),
        (2.0, len + 62.0, false),
    ] {
        own_on_lemg30_final(0.95 + (3.0 - t) * 0.028, at(t));
        assert!(go_around_tick(&[lander(along, gnd)], 380.0, false, true, at(t)).is_none());
    }
    own_on_lemg30_final(0.95, at(3.0));
    let touchdown = lander(len - 10.0, true);
    let exits = strip_exits(
        std::slice::from_ref(&touchdown),
        Some(&lemg_runways()[0]),
        "30",
        at(3.0),
    );
    assert_eq!(exits.len(), 1);
    assert_eq!(
        exits[0].exit,
        StripExit::Oncoming,
        "judged from its track at first contact"
    );
    *STRIP_SEEN.lock().unwrap() = None;
    for (t, along, gnd) in [
        (0.0, len + 206.0, false),
        (1.0, len + 134.0, false),
        (2.0, len + 62.0, false),
    ] {
        assert!(go_around_tick(&[lander(along, gnd)], 380.0, false, true, at(t)).is_none());
    }
    // The crew's watch, armed on the approach (#182).
    let watch = crate::party_line::GoAroundOrders::arm();
    assert_eq!(watch.expected_runway(), None);
    let call = go_around_tick(
        std::slice::from_ref(&touchdown),
        359.0,
        false,
        true,
        at(3.0),
    )
    .expect("34 s out with a lander rolling at us: the order");
    assert_eq!(
        call.line,
        format!(
            "{SR22}, go around, I say again, go around, traffic on the runway. Climb to \
                 circuit altitude, runway heading, expect runway one two, report downwind."
        )
    );
    assert_eq!(
        *OWN_STRIP.pending_resequence.lock().unwrap(),
        Some(("30".to_string(), "12".to_string()))
    );
    // The order's runway rides with the order, for the crew that flies
    // it (#182): a watch armed before it sees "12", one armed after
    // (a later approach) sees nothing.
    assert_eq!(watch.expected_runway().as_deref(), Some("12"));
    assert_eq!(
        crate::party_line::GoAroundOrders::arm().expected_runway(),
        None
    );
    // The FSM sync.
    let mut st = lxgb();
    st.apt.name = "Malaga".into();
    st.apt.elev_ft = 52.0;
    st.apt.runways = lemg_runways();
    st.runway_in_use = Some("30".into());
    st.desk.twr.state = tower::State::ClearedToLand;
    st.apply_monitor_go_around();
    assert_eq!(st.runway_in_use.as_deref(), Some("12"));
    assert_eq!(
        OWN_STRIP.clearance.lock().unwrap().as_ref().unwrap().runway_ident,
        "12"
    );
    assert_eq!(
        OWN_STRIP.clearance.lock().unwrap().as_ref().unwrap().state,
        tower::State::Inbound
    );
    // The crew's call, as transmitted.
    let wx = Wx {
        wind_from_deg: 300.0,
        wind_kts: 4.0,
    };
    let ctx = st.make_ctx(
        wx,
        String::new(),
        SR22.into(),
        false,
        false,
        None,
        true,
        None,
    );
    assert_eq!(
        ctx.runway, "one two",
        "the runway in use moved with the order"
    );
    let r = st.route_keyed(
        "november five five zero charlie romeo, going around",
        Service::Tower,
        &ctx,
        wx,
    );
    assert_eq!(
        r,
        format!(
            "{SR22}, roger, go around. Climb to circuit altitude, runway heading. \
                 Runway three zero is blocked, expect runway one two, report downwind."
        )
    );
    // Said once: the next go-around call is the plain answer.
    assert!(st.desk.resequenced.is_none());
    reset_runway_watch();
}

/// #182: the runway a go-around order names is the LATEST order's. An
/// order that names none leaves nothing for the crew to re-aim on (a
/// new flight, install and uninstall retire it with `OWN_STRIP.pending_resequence`).
#[test]
fn an_order_without_a_runway_retires_the_last_orders_runway() {
    use std::sync::atomic::Ordering::SeqCst;
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let watch = crate::party_line::GoAroundOrders::arm();
    *OWN_STRIP.resequenced_order.lock().unwrap() = Some((GO_AROUND_ORDERS.load(SeqCst) + 1, "13".into()));
    GO_AROUND_ORDERS.fetch_add(1, SeqCst);
    assert_eq!(watch.expected_runway().as_deref(), Some("13"));
    // Published before the count moves: never seen early by a watch armed
    // between the two writes.
    *OWN_STRIP.resequenced_order.lock().unwrap() = Some((GO_AROUND_ORDERS.load(SeqCst) + 1, "12".into()));
    assert_eq!(watch.expected_runway(), None, "its order is not given yet");
    GO_AROUND_ORDERS.fetch_add(1, SeqCst);
    assert_eq!(watch.expected_runway().as_deref(), Some("12"));
    *OWN_STRIP.resequenced_order.lock().unwrap() = None;
    GO_AROUND_ORDERS.fetch_add(1, SeqCst);
    assert_eq!(watch.expected_runway(), None);
    reset_runway_watch();
}

/// Issue #162, part 3: a STANDING occupant blocks its whole strip, so the
/// reciprocal is no answer; the parallel is (LEMG 31 in 310 at 8, 3,206 m
/// against 30's 2,755). With no other runway the tower holds: the order
/// names none, the downwind is number two behind the traffic, and the
/// final report is withheld until the strip clears.
#[test]
fn a_standing_occupant_resequences_to_another_strip_or_holds() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let base = Instant::now();
    let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
    let squatter = ghost_at("EZY7038", lemg30_point(1400.0, 3.0));
    // Pure: the pick and its refusals.
    let rw = lemg_runways();
    assert_eq!(
        resequence_runway(&rw, "30", false, &["12".into()], &[], Some((310.0, 8.0))),
        Ok("31".to_string())
    );
    assert_eq!(
        resequence_runway(
            &rw,
            "30",
            false,
            &["12".into(), "13".into()],
            &[],
            Some((310.0, 8.0))
        ),
        Err("no_other_runway")
    );
    assert_eq!(
        resequence_runway(&rw, "30", false, &["12".into()], &[], None),
        Err("no_wind")
    );
    // 31 against a live 12 stream is refused; 13 in calm is not.
    assert_eq!(
        resequence_runway(
            &rw,
            "30",
            false,
            &["12".into()],
            &["12".into()],
            Some((300.0, 3.0))
        ),
        Ok("13".to_string())
    );
    // Oncoming: the reciprocal, with the stream.
    assert_eq!(
        resequence_runway(&rw, "30", true, &["12".into()], &[], Some((300.0, 4.0))),
        Ok("12".to_string())
    );
    // ...but not into a 20 kt tailwind, and never the parallel against the
    // oncoming direction (LEMG 31 against 12): no runway, the hold.
    assert_eq!(
        resequence_runway(&rw, "30", true, &["12".into()], &[], Some((300.0, 20.0))),
        Err("no_other_runway")
    );

    // Live: two strips, the squatter on 12/30.
    lemg30_cleared(lemg_runways(), (310.0, 8.0));
    own_on_lemg30_final(0.95, at(1.0));
    let call = go_around_tick(std::slice::from_ref(&squatter), 380.0, false, true, at(1.0))
        .expect("the order");
    assert!(
        call.line.contains("expect runway three one"),
        "{}",
        call.line
    );
    assert_eq!(
        *OWN_STRIP.pending_resequence.lock().unwrap(),
        Some(("30".to_string(), "31".to_string()))
    );

    // One strip only: the order stands alone, nothing re-sequenced.
    lemg30_cleared(vec![lemg_runways()[0].clone()], (310.0, 8.0));
    own_on_lemg30_final(0.95, at(1.0));
    let call = go_around_tick(std::slice::from_ref(&squatter), 380.0, false, true, at(1.0))
        .expect("the order");
    assert_eq!(
        call.line,
        format!("{SR22}, go around, I say again, go around, traffic on the runway. Climb to circuit altitude, runway heading, report downwind.")
    );
    assert!(OWN_STRIP.pending_resequence.lock().unwrap().is_none());
    // The hold: downwind number two, the final continued, not cleared.
    let mut st = lxgb();
    st.apt.name = "Malaga".into();
    st.apt.runways = vec![lemg_runways()[0].clone()];
    st.runway_in_use = Some("30".into());
    st.desk.twr.state = tower::State::ClearedToLand;
    st.apply_monitor_go_around();
    assert_eq!(st.runway_in_use.as_deref(), Some("30"));
    let wx = Wx {
        wind_from_deg: 310.0,
        wind_kts: 8.0,
    };
    let ctx = st.make_ctx(
        wx,
        String::new(),
        SR22.into(),
        false,
        false,
        None,
        true,
        None,
    );
    let r = st.route_keyed(
        "november five five zero charlie romeo, downwind runway three zero",
        Service::Tower,
        &ctx,
        wx,
    );
    assert_eq!(
        r,
        format!("{SR22}, number two, traffic on the runway, report final.")
    );
    let r = st.route_keyed(
        "november five five zero charlie romeo, final runway three zero",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("continue approach runway three zero"), "{r}");
    assert!(crate::party_line::landing_hold_armed());
    reset_runway_watch();
}

/// Issue #162, part 2: a ghost standing on the runway past
/// GHOST_STALE_S is stale data. At 4 minutes it is still traffic (the
/// order); at 5 it is ignored (no order, and a withheld landing is
/// released); moved 20 m, its clock starts again.
#[test]
fn a_ghost_standing_on_the_runway_for_five_minutes_is_stale() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let base = Instant::now();
    let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
    let frozen = ghost_at("VY3149", lemg30_point(1400.0, 0.0));
    lemg30_cleared(vec![lemg_runways()[0].clone()], (300.0, 4.0));
    // Seen through the approach, high: only the clocks run.
    for t in [0.0, 60.0, 120.0, 180.0] {
        assert!(
            go_around_tick(std::slice::from_ref(&frozen), 2000.0, false, true, at(t)).is_none()
        );
    }
    own_on_lemg30_final(0.95, at(240.0));
    assert!(
        go_around_tick(std::slice::from_ref(&frozen), 380.0, false, true, at(240.0)).is_some(),
        "four minutes standing: still an occupant"
    );
    assert!((standing_secs("VY3149", at(240.0)) - 240.0).abs() < 1e-6);

    // Five minutes: stale, no order; and the withheld landing goes.
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Inbound,
        callsign: SR22.into(),
        runway_ident: "30".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *OWN_STRIP.landing_hold.lock().unwrap() = Some(LandingHold {
        callsign: SR22.into(),
        runway: "three zero".into(),
        runway_ident: "30".into(),
        line: format!("{SR22}, runway three zero, cleared to land."),
    });
    own_on_lemg30_final(0.95, at(301.0));
    assert!(
        go_around_tick(std::slice::from_ref(&frozen), 380.0, false, true, at(301.0)).is_none()
    );
    assert!(
        landing_hold_tick(std::slice::from_ref(&frozen), false, at(301.0)).is_some(),
        "a stale ghost does not hold the landing"
    );
    // Moved 20 m: a live aircraft again.
    let moved = ghost_at("VY3149", lemg30_point(1420.0, 0.0));
    let exits = strip_exits(
        std::slice::from_ref(&moved),
        Some(&lemg_runways()[0]),
        "30",
        at(302.0),
    );
    assert_eq!(exits.len(), 1, "moved, its clock starts again");
    assert!(standing_secs("VY3149", at(302.0)) < 1.0);
    reset_runway_watch();
}

/// A withheld landing is the go-around watch's too, and it retires with
/// the landing or a new answer.
#[test]
fn a_withheld_landing_is_ordered_around_or_retired() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_RUNWAYS.lock().unwrap() = lemg_runways();
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Inbound,
        callsign: "November eight one six November November".into(),
        runway_ident: "13".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    *OWN_STRIP.pending_go_around.lock().unwrap() = false;
    *STRIP_SEEN.lock().unwrap() = None;
    OWN_STRIP.own_fixes.lock().unwrap().clear();
    let hold = || LandingHold {
        callsign: "November eight one six November November".into(),
        runway: "one three".into(),
        runway_ident: "13".into(),
        line: "cleared to land.".into(),
    };
    *OWN_STRIP.landing_hold.lock().unwrap() = Some(hold());
    let base = Instant::now();
    let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
    // A ghost standing mid-runway, us at a mile: the order, and the hold
    // dies with it.
    let squatter = ghost_at("EZY7038", lemg13_point(1600.0, 0.0));
    let (lat, lon) = lemg13_point(-1852.0 - 77.0, 0.0);
    note_own_fix(lat, lon, false, at(0.0));
    let (lat, lon) = lemg13_point(-1852.0, 0.0);
    note_own_fix(lat, lon, false, at(1.0));
    assert!(
        go_around_tick(std::slice::from_ref(&squatter), 400.0, false, true, at(1.0)).is_some()
    );
    assert!(!crate::party_line::landing_hold_armed());
    // Landed with the hold standing: dropped, nothing said.
    *OWN_STRIP.landing_hold.lock().unwrap() = Some(hold());
    assert!(landing_hold_tick(&[], true, at(2.0)).is_none());
    assert!(!crate::party_line::landing_hold_armed());
    // A new flight retires it.
    *OWN_STRIP.landing_hold.lock().unwrap() = Some(hold());
    retire_for_new_flight();
    assert!(!crate::party_line::landing_hold_armed());
    NEW_FLIGHT_PENDING.store(false, std::sync::atomic::Ordering::SeqCst);
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.pending_go_around.lock().unwrap() = false;
    *OWN_STRIP.go_around_last.lock().unwrap() = None;
    *STRIP_SEEN.lock().unwrap() = None;
    OWN_STRIP.own_fixes.lock().unwrap().clear();
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

#[test]
fn a_monitor_go_around_voids_the_clearance_and_flips_the_coach() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_PANEL.lock().unwrap() = Some((
        "Gibraltar".into(),
        11.0,
        vec![("tower".to_string(), 131200_i64)],
    ));
    *ATIS.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.vectors.lock().unwrap() = None;
    *OWN_STRIP.landing_handoff.lock().unwrap() = None;
    let cs = "November one seven two Sierra Papa";
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::ClearedToLand,
        callsign: cs.into(),
        runway_ident: "27".into(),
        pending_readback: Some(format!("Cleared to land runway two seven, {cs}.")),
        holds_runway: false,
    });
    let mut st = lxgb();
    st.desk.twr.state = tower::State::ClearedToLand;
    // Before the sync: the coach suggests the landing readback — the
    // exact contradiction from the 2026-07-14 tape.
    let s = suggest_call(cs, false, Some(1.0), None).unwrap();
    assert!(
        s.call.as_deref().unwrap().contains("Cleared to land"),
        "{:?}",
        s.call
    );

    st.apply_monitor_go_around();

    assert_eq!(
        st.desk.twr.state,
        tower::State::Inbound,
        "clearance void in the FSM"
    );
    assert!(st.desk.twr.pending().is_none(), "the owed readback dies with it");
    assert_eq!(
        OWN_STRIP.clearance.lock().unwrap().as_ref().unwrap().state,
        tower::State::Inbound
    );
    // The coach's next line is the acknowledgment, urgent, and it parses.
    let s = suggest_call(cs, false, Some(1.0), None).unwrap();
    let call = s.call.expect("a call");
    assert!(call.contains("Going around"), "{call}");
    assert!(s.urgent, "the go-around ack is the say-now line");
    assert_eq!(tower::parse_intent(&call), tower::Intent::GoAround);

    *OWN_STRIP.clearance.lock().unwrap() = None;
    *ACTIVE_PANEL.lock().unwrap() = None;
}

#[test]
fn held_takeoff_clearance_releases_when_the_final_stays_clear() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // No wake clock: this test is about the TRAFFIC gate alone, and a
    // stale one left by another test would block every tick.
    *WAKE.lock().unwrap() = None;
    *OWN_STRIP.takeoff_hold.lock().unwrap() = Some(TakeoffHold {
        callsign: "November one seven two Sierra Papa".into(),
        runway: "zero niner".into(),
        runway_ident: "09".into(),
        clear_since: None,
        armed_at: Instant::now(),
        logged_at: Instant::now(),
        lined_up: false,
        behind: false,
        held_for: None,
        cut_in_logged: Default::default(),
    });
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "November one seven two Sierra Papa".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: false,
    });
    // Our strip: 09/27. The watches only care about THIS pavement.
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1512,
        lon_a: -5.3554,
        lat_b: 36.1517,
        lon_b: -5.3366,
    }];
    let elev = 11.0;
    // An arrival in OUR corridor, 2 nm out at ~400 ft; a PARALLEL
    // arrival the same distance out but 2 km abeam; a ghost parked on
    // our strip.
    let ours = TcasTarget {
        callsign: "BA1".into(),
        lat: 36.1510,
        lon: -5.3554 - 0.037,
        ele_ft: 411.0,
        on_ground: false,
        vs_fpm: -600.0,
    };
    let parallel = TcasTarget {
        callsign: "BA2".into(),
        lat: 36.1690,
        ..ours.clone()
    };
    let squatter = TcasTarget {
        callsign: "BA3".into(),
        lat: 36.15155,
        lon: -5.342,
        ele_ft: elev,
        on_ground: true,
        vs_fpm: 0.0,
    };
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    // Traffic on OUR final inside the departure gap: no release. The
    // strip itself is clear and the lander is far enough out, so the
    // tower STAGES us instead of parking us behind the line — once.
    let staged = takeoff_hold_tick(std::slice::from_ref(&ours), elev, true, t0)
        .expect("line up and wait");
    assert!(
        staged.contains("line up and wait runway zero niner"),
        "{staged}"
    );
    assert!(staged.contains("landing traffic"), "{staged}");
    assert!(*OWN_STRIP.pending_line_up.lock().unwrap());
    *OWN_STRIP.pending_line_up.lock().unwrap() = false;
    assert!(
        takeoff_hold_tick(std::slice::from_ref(&ours), elev, true, t0).is_none(),
        "one shot: the aircraft can only take the runway once"
    );
    // Only the PARALLEL runway's arrival: not our traffic — the timer
    // runs (the runway-blind check held a Heathrow 09R departure forever).
    assert!(
        takeoff_hold_tick(std::slice::from_ref(&parallel), elev, true, t0 + sec(1)).is_none()
    );
    // A ghost sitting on our strip resets it again.
    assert!(
        takeoff_hold_tick(std::slice::from_ref(&squatter), elev, true, t0 + sec(5)).is_none()
    );
    assert!(
        takeoff_hold_tick(std::slice::from_ref(&parallel), elev, true, t0 + sec(6)).is_none()
    );
    // Our runway clear and stayed clear (parallel stream still landing):
    // the tower comes back unprompted.
    let line = takeoff_hold_tick(std::slice::from_ref(&parallel), elev, true, t0 + sec(25))
        .expect("release");
    assert!(line.contains("traffic clear"), "{line}");
    assert!(
        line.contains("Runway zero niner, cleared for takeoff"),
        "{line}"
    );
    // The compliance picture moved with it, and the FSM sync is armed.
    assert_eq!(
        OWN_STRIP.clearance.lock().unwrap().as_ref().unwrap().state,
        tower::State::TakeoffCleared
    );
    assert!(*OWN_STRIP.pending_takeoff_clearance.lock().unwrap());
    // One shot: the hold is consumed.
    assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(60)).is_none());

    // Airborne, a held clearance is VOID, not released: the pilot who
    // departed through the cancel got "traffic clear, cleared for
    // takeoff" two minutes into the climb (Gatwick, 2026-07-13).
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    *OWN_STRIP.takeoff_hold.lock().unwrap() = Some(TakeoffHold {
        callsign: "November one seven two Sierra Papa".into(),
        runway: "zero niner".into(),
        runway_ident: "09".into(),
        clear_since: Some(t0),
        armed_at: t0,
        logged_at: t0,
        lined_up: false,
        behind: false,
        held_for: None,
        cut_in_logged: Default::default(),
    });
    assert!(takeoff_hold_tick(&[], elev, false, t0 + sec(120)).is_none());
    assert!(
        OWN_STRIP.takeoff_hold.lock().unwrap().is_none(),
        "airborne must void the hold"
    );
    assert!(
        takeoff_hold_tick(&[], elev, true, t0 + sec(180)).is_none(),
        "a voided hold never releases, even back on the ground"
    );
    assert!(!*OWN_STRIP.pending_takeoff_clearance.lock().unwrap());

    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

/// Gatwick 26L, 2026-07-27, with injected traffic: held at holding point
/// Mike behind an arrival at half a mile, and the release never came —
/// eight minutes, then a hand-back. Every plane landed except ours. A
/// single-runway field working a landing every couple of minutes NEVER
/// has an empty four-mile final, so requiring the whole corridor to be
/// clear made the release unreachable; a real tower threads the departure
/// into the gap instead. The runway itself stays absolute.
#[test]
fn a_busy_arrival_stream_still_gets_a_departure_away() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    let elev = 202.0;
    *WAKE.lock().unwrap() = None;
    // Gatwick's main strip.
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 45.0,
        surface: 0,
        ident_a: "08R".into(),
        ident_b: "26L".into(),
        lat_a: 51.1512,
        lon_a: -0.1997,
        lat_b: 51.1560,
        lon_b: -0.1424,
    }];
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "November two four four Kilo Romeo".into(),
        runway_ident: "26L".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    *OWN_STRIP.pending_line_up.lock().unwrap() = false;
    *OWN_STRIP.takeoff_hold.lock().unwrap() = Some(TakeoffHold {
        callsign: "November two four four Kilo Romeo".into(),
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
    // An arrival on the 26L final: `nm` beyond the 26L threshold along the
    // extended centreline (landing westbound, so the approach lies east of
    // the field), on a 3-degree slope.
    let m_lon = 111_320.0 * 51.154_f64.to_radians().cos();
    let (ux, uy) = {
        let (dx, dy) = ((-0.1424 + 0.1997) * m_lon, (51.1560 - 51.1512) * 111_320.0);
        let len = (dx * dx + dy * dy).sqrt();
        (dx / len, dy / len)
    };
    let arrival = |nm: f64| TcasTarget {
        callsign: "EZY83".into(),
        lat: 51.1560 + nm * 1852.0 * uy / 111_320.0,
        lon: -0.1424 + nm * 1852.0 * ux / m_lon,
        ele_ft: elev + 300.0 * nm,
        on_ground: false,
        vs_fpm: -700.0,
    };
    // Half a mile out: too close to stage, too close to release. This is
    // exactly where the live hold began.
    assert!(takeoff_hold_tick(std::slice::from_ref(&arrival(0.5)), elev, true, t0).is_none());
    assert!(
        !*OWN_STRIP.pending_line_up.lock().unwrap(),
        "never onto the runway under a lander"
    );

    // Two miles out, the strip momentarily clear: too close to depart
    // ahead of, far enough to STAGE. The tower puts us on the pavement
    // rather than parking us behind the line for a stream that never ends.
    let line = takeoff_hold_tick(
        std::slice::from_ref(&arrival(2.0)),
        elev,
        true,
        t0 + sec(60),
    )
    .expect("line up and wait");
    assert!(
        line.contains("line up and wait runway two six left"),
        "{line}"
    );
    assert!(line.contains("landing traffic"), "{line}");
    assert!(*OWN_STRIP.pending_line_up.lock().unwrap());
    assert!(
        takeoff_hold_tick(
            std::slice::from_ref(&arrival(1.8)),
            elev,
            true,
            t0 + sec(70)
        )
        .is_none(),
        "one shot: the aircraft can only take the runway once"
    );
    // ...and while it rolls out ON our strip, no release at all.
    let rollout = TcasTarget {
        callsign: "EZY83".into(),
        lat: 51.1540,
        lon: -0.1700,
        ele_ft: elev,
        on_ground: true,
        vs_fpm: 0.0,
    };
    assert!(
        takeoff_hold_tick(std::slice::from_ref(&rollout), elev, true, t0 + sec(120)).is_none()
    );

    // Vacated, with the next arrival 3.5 nm out and closing: THIS is the
    // gap a real tower departs into. Under the old all-or-nothing gate
    // that aircraft was still "traffic on final" — inside the 4 nm
    // corridor — and the departure sat here until it gave up.
    assert!(
        takeoff_hold_tick(
            std::slice::from_ref(&arrival(3.5)),
            elev,
            true,
            t0 + sec(130)
        )
        .is_none(),
        "the clear window has to stabilise first"
    );
    let line = takeoff_hold_tick(
        std::slice::from_ref(&arrival(3.2)),
        elev,
        true,
        t0 + sec(146),
    )
    .expect("release");
    assert!(line.contains("traffic clear"), "{line}");
    assert!(
        line.contains("Runway two six left, cleared for takeoff"),
        "{line}"
    );
    assert!(*OWN_STRIP.pending_takeoff_clearance.lock().unwrap());

    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.pending_line_up.lock().unwrap() = false;
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

/// Heathrow, 2026-07-27: held at holding point Sierra for 27L, full auto
/// sat out its entire 480 s budget and handed the aircraft back. The
/// runway had clear gaps and no arrival was in our corridor — but a
/// liftoff off 27L about every two minutes re-armed a 120 s wake clock
/// faster than it could expire, so the release condition was unreachable.
/// The hold must now sequence the departure onto the runway and, once one
/// full wake interval has been served at the hold, release it.
#[test]
fn a_busy_departure_field_sequences_the_hold_then_releases_it() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    let elev = 83.0;
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 50.0,
        surface: 0,
        ident_a: "09R".into(),
        ident_b: "27L".into(),
        lat_a: 51.4647701,
        lon_a: -0.4867489,
        lat_b: 51.4649460,
        lon_b: -0.4340638,
    }];
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "November two four four Kilo Romeo".into(),
        runway_ident: "27L".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    *OWN_STRIP.pending_line_up.lock().unwrap() = false;
    *OWN_STRIP.takeoff_hold.lock().unwrap() = Some(TakeoffHold {
        callsign: "November two four four Kilo Romeo".into(),
        runway: "two seven left".into(),
        runway_ident: "27L".into(),
        clear_since: None,
        armed_at: t0,
        logged_at: t0,
        lined_up: false,
        behind: false,
        held_for: None,
        cut_in_logged: Default::default(),
    });
    // A heavy just rotated off our strip.
    let departed = |at: Instant| {
        *WAKE.lock().unwrap() = Some(Wake {
            since: at,
            runway_ident: "27L".into(),
        });
    };
    // A ghost holding on the runway, and the queue's next departure
    // sitting on the threshold — both are ON our strip.
    let squatter = TcasTarget {
        callsign: "BAW1346".into(),
        lat: 51.46494,
        lon: -0.4345,
        ele_ft: elev,
        on_ground: true,
        vs_fpm: 0.0,
    };

    departed(t0);
    // Strip occupied AND the wake clock running: no line-up onto an
    // occupied runway, ever.
    assert!(
        takeoff_hold_tick(std::slice::from_ref(&squatter), elev, true, t0 + sec(1)).is_none()
    );
    assert!(!*OWN_STRIP.pending_line_up.lock().unwrap(), "never onto a busy strip");

    // Strip clear, wake still running: sequenced onto the runway rather
    // than left at the taxiway while the queue rolls past.
    let line = takeoff_hold_tick(&[], elev, true, t0 + sec(30)).expect("line up and wait");
    assert!(
        line.contains("line up and wait runway two seven left"),
        "{line}"
    );
    assert!(*OWN_STRIP.pending_line_up.lock().unwrap(), "the FSM sync is armed");
    assert_eq!(
        OWN_STRIP.clearance.lock().unwrap().as_ref().unwrap().state,
        tower::State::LinedUp
    );
    // One shot: the aircraft can only take the runway once.
    assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(31)).is_none());

    // The queue keeps departing ahead of us. Each liftoff used to restart
    // our clock; it no longer does — but inside the first interval the
    // separation we genuinely owe still holds us.
    departed(t0 + sec(60));
    assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(61)).is_none());
    assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(119)).is_none());

    // One full wake interval served at the hold: later departures are the
    // queue's separation, not ours. The clear timer starts...
    departed(t0 + sec(130));
    assert!(takeoff_hold_tick(&[], elev, true, t0 + sec(140)).is_none());
    // ...but a ghost back on the strip still resets it. Safety is kept.
    assert!(
        takeoff_hold_tick(std::slice::from_ref(&squatter), elev, true, t0 + sec(150)).is_none()
    );
    assert!(
        takeoff_hold_tick(&[], elev, true, t0 + sec(160)).is_none(),
        "the clear window restarts behind the squatter"
    );
    let line = takeoff_hold_tick(&[], elev, true, t0 + sec(176)).expect("release");
    assert!(line.contains("traffic clear"), "{line}");
    assert!(
        line.contains("Runway two seven left, cleared for takeoff"),
        "{line}"
    );
    assert!(*OWN_STRIP.pending_takeoff_clearance.lock().unwrap());
    assert_eq!(
        OWN_STRIP.clearance.lock().unwrap().as_ref().unwrap().state,
        tower::State::TakeoffCleared
    );

    *WAKE.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.pending_line_up.lock().unwrap() = false;
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

#[test]
fn incursion_traffic_check_ignores_the_other_runways_stream() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_PANEL.lock().unwrap() = Some(("Malaga".into(), 11.0, Vec::new()));
    // Two strips 2 km apart, Malaga-style: we hold 12, arrivals land 31.
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![
        nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "12".into(),
            ident_b: "30".into(),
            lat_a: 36.1512,
            lon_a: -5.3554,
            lat_b: 36.1517,
            lon_b: -5.3366,
        },
        nav::Runway {
            width_m: 0.0,
            surface: 0,
            ident_a: "13".into(),
            ident_b: "31".into(),
            lat_a: 36.1692,
            lon_a: -5.3554,
            lat_b: 36.1697,
            lon_b: -5.3366,
        },
    ];
    // An arrival ~2 nm out at ~400 ft in the OTHER strip's corridor: the
    // runway-blind check shouted "exit the runway, expedite" at a cleared
    // runway 12 departure for exactly this picture (Malaga, 2026-07-12).
    let other = TcasTarget {
        callsign: "EZY5701".into(),
        lat: 36.1690,
        lon: -5.3554 - 0.037,
        ele_ft: 411.0,
        on_ground: false,
        vs_fpm: -600.0,
    };
    assert!(
        inbound_final_for_strip(std::slice::from_ref(&other), "12").is_none(),
        "the other runway's stream is not our traffic"
    );
    // The same arrival in OUR corridor earns the shout.
    let ours = TcasTarget {
        lat: 36.1510,
        ..other.clone()
    };
    let nm = inbound_final_for_strip(std::slice::from_ref(&ours), "12").expect("our final");
    assert!((1.0..3.5).contains(&nm), "{nm}");
    // The same position CLIMBING is a departure leaving, not an arrival:
    // the ghost that lifted off runway 30 read as "traffic on short
    // final" to the runway 12 watch (Malaga, 2026-07-12).
    let climber = TcasTarget {
        vs_fpm: 1200.0,
        ..ours.clone()
    };
    assert!(
        inbound_final_for_strip(std::slice::from_ref(&climber), "12").is_none(),
        "a climbing departure is not final traffic"
    );
    // Unknown vertical speed (unfed array) stays counted — fail toward
    // caution.
    let level = TcasTarget {
        vs_fpm: 0.0,
        ..ours.clone()
    };
    assert!(inbound_final_for_strip(std::slice::from_ref(&level), "12").is_some());
    // A strip the field doesn't have: never fires.
    assert!(inbound_final_for_strip(std::slice::from_ref(&ours), "99").is_none());

    *ACTIVE_PANEL.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

#[test]
fn the_coach_suggests_the_parsers_happy_path() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_PANEL.lock().unwrap() = Some((
        "Gibraltar".into(),
        11.0,
        vec![("tower".to_string(), 131200_i64)],
    ));
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *ATIS.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.vectors.lock().unwrap() = None;
    *OWN_STRIP.landing_handoff.lock().unwrap() = None;
    let cs = "November one seven two Sierra Papa";
    // The transmission itself, for the parse checks below.
    let call = |s: Suggestion| s.call.expect("a call");

    // Cold on the ground: the full initial call.
    let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
    assert!(s.starts_with("Gibraltar Tower"), "{s}");
    assert!(s.contains("request departure clearance"), "{s}");
    // Cold in the air: the inbound call, with position.
    let s = call(suggest_call(cs, false, Some(8.0), None).unwrap());
    assert!(s.contains("8 miles out"), "{s}");
    assert!(s.contains("inbound"), "{s}");
    // Every suggestion must parse to a routable intent — the coach IS
    // the parser's happy path.
    assert_eq!(
        tower::parse_intent(&s),
        tower::Intent::RequestLanding {
            touch_and_go: false
        }
    );

    // Walk the departure states.
    let set = |state: tower::State| {
        *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
            state,
            callsign: cs.into(),
            runway_ident: "09".into(),
            pending_readback: None,
            holds_runway: false,
        });
    };
    set(tower::State::Parked);
    let sug = suggest_call(cs, true, Some(0.5), None).unwrap();
    assert!(!sug.arrival, "parked = departure side");
    let s = call(sug);
    assert!(s.contains("request taxi"), "{s}");
    assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTaxi);
    set(tower::State::TaxiCleared);
    // No taxi network: no invented holding point name.
    let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
    assert!(s.contains("ready for departure"), "{s}");
    assert!(
        !s.contains("holding point"),
        "no route, no invented name: {s}"
    );
    assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTakeoff);
    // With the cleared route published: the REAL holding point + runway.
    *OWN_STRIP.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(36.16, -5.36), (36.16, -5.35)],
        via: "Alpha".into(),
        seg: vec![("A".into(), false)],
        holding_point: "A1".into(),
        lead_on: vec![],
    });
    let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
    assert!(s.contains("holding point Alpha one"), "{s}");
    assert!(s.contains("runway zero niner"), "{s}");
    assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTakeoff);
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    set(tower::State::TakeoffCleared);
    let s = call(suggest_call(cs, true, Some(0.5), None).unwrap());
    assert!(s.contains("Cleared for takeoff runway zero niner"), "{s}");
    assert!(tower::is_readback(&s.to_lowercase()), "readback: {s}");
    // Arrival side.
    set(tower::State::Inbound);
    let sug = suggest_call(cs, false, Some(2.0), None).unwrap();
    assert!(sug.arrival);
    let s = call(sug);
    assert!(s.contains("final runway zero niner"), "{s}");
    assert_eq!(tower::parse_intent(&s), tower::Intent::ReportFinal);
    set(tower::State::ClearedToLand);
    let s = call(suggest_call(cs, true, Some(0.2), None).unwrap());
    assert!(s.contains("taxi to parking"), "{s}");
    assert_eq!(tower::parse_intent(&s), tower::Intent::RequestTaxiIn);

    // No station: nothing to suggest.
    *ACTIVE_PANEL.lock().unwrap() = None;
    assert!(suggest_call(cs, true, None, None).is_none());
    *OWN_STRIP.clearance.lock().unwrap() = None;

    // The question forms that summon the coach.
    for q in [
        "what do i say",
        "what should i say to the tower",
        "how do i ask for taxi",
        "what's the call",
        "help me with the radio",
    ] {
        assert!(coach_query(q), "{q:?} should reach the coach");
    }
    // Yes/no shapes about whether a call is owed reach the coach too.
    for q in [
        "should we check in with atc",
        "do we need to call the tower",
        "is it time to contact departure",
        "shall we report to ground",
    ] {
        assert!(coach_query(q), "{q:?} should reach the coach");
        assert!(coach_yes_no(q), "{q:?} is a yes/no");
    }
    assert!(!coach_query("flaps ten"));
    assert!(!coach_query("what frequencies are here"));
    assert!(!coach_query("should we lower the gear"));
    assert!(!coach_yes_no("what do i say"));
}

#[test]
fn the_coach_leads_with_the_prerequisite() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_PANEL.lock().unwrap() = Some((
        "Gibraltar".into(),
        11.0,
        vec![("tower".to_string(), 131200_i64)],
    ));
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *ATIS.lock().unwrap() = None;
    *OWN_STRIP.vectors.lock().unwrap() = None;
    *OWN_STRIP.landing_handoff.lock().unwrap() = None;
    let cs = "November one seven two Sierra Papa";

    // COM1 parked on another frequency: the call can't be heard — tune
    // first. On frequency (or unknown): no nag.
    let s = suggest_call(cs, true, Some(0.5), Some(122800)).unwrap();
    assert_eq!(
        s.first.as_deref(),
        Some("First tune tower, one three one decimal two."),
        "{:?}",
        s.first
    );
    assert!(s.call.is_some());
    let s = suggest_call(cs, true, Some(0.5), Some(131200)).unwrap();
    assert!(s.first.is_none(), "{:?}", s.first);

    // An ATIS that never cut an edition = nobody listened: copy it
    // before the first call, and it outranks the tune nag (it IS the
    // next tuning action).
    *ATIS.lock().unwrap() = Some(AtisState {
        khz: 118500,
        airport: "Gibraltar".into(),
        runways: Vec::new(),
        letter: 'A',
        dialect: Dialect::Icao,
        edition: None,
        last_spoken: None,
    });
    let s = suggest_call(cs, true, Some(0.5), Some(122800)).unwrap();
    assert_eq!(
        s.first.as_deref(),
        Some("First copy the ATIS on one one eight decimal five."),
        "{:?}",
        s.first
    );
    // Once in contact the tower's QNH covers you: no more ATIS nag.
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Parked,
        callsign: cs.into(),
        runway_ident: String::new(),
        pending_readback: None,
        holds_runway: false,
    });
    let s = suggest_call(cs, true, Some(0.5), Some(131200)).unwrap();
    assert!(s.first.is_none(), "{:?}", s.first);

    *ATIS.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *ACTIVE_PANEL.lock().unwrap() = None;
}

#[test]
fn the_coach_covers_the_approach_phase() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_PANEL.lock().unwrap() = Some((
        "Gibraltar".into(),
        11.0,
        vec![("tower".to_string(), 131200_i64)],
    ));
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *ATIS.lock().unwrap() = None;
    *OWN_STRIP.landing_handoff.lock().unwrap() = None;
    let cs = "November one seven two Sierra Papa";
    let plan = |ils: bool, cleared: bool| VectorPlan {
        callsign: cs.into(),
        thr_lat: 36.15,
        thr_lon: -5.35,
        course_deg: 90.0,
        runway_spoken: "zero niner".into(),
        tower_spoken: Some("one three one decimal two".into()),
        target_alt_ft: 2000.0,
        ils,
        join_nm: 6.0,
        cleared,
        dialect: Dialect::Icao,
        last_tx: None,
        last_heading: None,
    };

    // Under vectors, nothing owed: coach the silence, not a fake call.
    *OWN_STRIP.vectors.lock().unwrap() = Some(plan(true, false));
    let s = suggest_call(cs, false, Some(12.0), None).unwrap();
    assert!(
        s.first.as_deref().unwrap().contains("No call owed"),
        "{:?}",
        s.first
    );
    assert!(s.call.is_none());
    assert!(s.arrival);
    // Cleared for the ILS on the intercept: the "established" report is
    // the one call approach waits on — and it parses as the final report.
    *OWN_STRIP.vectors.lock().unwrap() = Some(plan(true, true));
    let s = suggest_call(cs, false, Some(10.0), None).unwrap();
    assert!(
        s.first.as_deref().unwrap().contains("established"),
        "{:?}",
        s.first
    );
    let c = s.call.expect("a call");
    assert!(c.contains("established"), "{c}");
    assert_eq!(tower::parse_intent(&c), tower::Intent::ReportFinal);
    *OWN_STRIP.vectors.lock().unwrap() = None;

    // Handed to the tower with an approach clearance: the "final" report
    // is owed — same fact the monitor's voice coach reads.
    *OWN_STRIP.landing_handoff.lock().unwrap() = Some("zero niner".into());
    let s = suggest_call(cs, false, Some(6.0), None).unwrap();
    let c = s.call.expect("a call");
    assert!(c.contains("final runway zero niner"), "{c}");
    assert_eq!(tower::parse_intent(&c), tower::Intent::ReportFinal);
    *OWN_STRIP.landing_handoff.lock().unwrap() = None;
    *ACTIVE_PANEL.lock().unwrap() = None;
}

#[test]
fn spoken_airport_uses_the_radio_name() {
    // Gazetteer entries shorten to what a controller actually keys.
    assert_eq!(spoken_airport("Badajoz- Talavera La Real"), "Badajoz");
    assert_eq!(spoken_airport("Merida - Royanejos"), "Merida");
    // Plain names pass through untouched.
    assert_eq!(spoken_airport("Gibraltar"), "Gibraltar");
    assert_eq!(spoken_airport("Ciudad Real Intl"), "Ciudad Real Intl");
    // Degenerate input survives.
    assert_eq!(spoken_airport("- Somewhere"), "Somewhere");
    assert_eq!(spoken_airport(""), "");
}

#[test]
fn vhf_range_is_line_of_sight() {
    // On the ramp the station's own mast still buys a few miles — enough
    // to hear the field you're parked at, not one 20 nm out.
    let ground = vhf_range_nm(0.0);
    assert!((5.0..12.0).contains(&ground), "{ground}");
    // Range grows with height: at 5,000 ft above the field, most of a
    // sector is reachable; 200 nm (Extremadura → Gibraltar) is not.
    let cruise = vhf_range_nm(5000.0);
    assert!((80.0..110.0).contains(&cruise), "{cruise}");
    assert!(vhf_range_nm(20000.0) < 200.0);
    // Below the field's elevation nothing goes negative.
    assert!(vhf_range_nm(-500.0) >= ground - 1e-9);
}

fn lxgb() -> Station {
    Station {
        apt: nav::AirportInfo {
            transition_level_ft: None,
            allows_circuits: None,
            pattern_sides: Vec::new(),
            papis: Vec::new(),
            name: "Gibraltar".into(),
            elev_ft: 11.0,
            runways: Vec::new(),
            freqs: vec![
                nav::AtcFreq {
                    kind: nav::FREQ_TOWER,
                    khz: 131200,
                    name: "Tower Freq".into(),
                },
                nav::AtcFreq {
                    kind: nav::FREQ_APPROACH,
                    khz: 122800,
                    name: "Approach".into(),
                },
            ],
        },
        tower_khz: 131200,
        approach_khz: Some(122800),
        departure_khz: None,
        ground_khz: None,
        delivery_khz: None,
        desk: StripDesk::new(),
        desks: std::collections::BTreeMap::new(),
        runway_in_use: None,
        enforce_readback: true,
        dialect: Dialect::Icao,
        cap413: false,
        procedures: None,
        sid_exits: Default::default(),
        last_qnh: None,
        reply_at: None,
    }
}

/// `ATIS` is a process-wide static, same discipline as `SIDE_STATE_LOCK`.
/// Station side state (`OWN_STRIP.clearance`/`OWN_STRIP.handoff`/`ATIS`/`REGION`/
/// `OWN_STRIP.takeoff_hold`/`OWN_STRIP.taxi_route`/...) is process statics, and the reply
/// path writes them on EVERY call — so every test that drives a station
/// or asserts on the statics holds this ONE lock. Three finer locks
/// (ATIS/REGION/HANDOFF) raced each other through `OWN_STRIP.clearance`: the coach
/// test nulled it mid-flight under the readback test (2026-07-11).
pub(super) static SIDE_STATE_LOCK: Mutex<()> = Mutex::new(());

fn wx(wind_deg: f64, wind_kts: f64, qnh: f64) -> AtisWx {
    AtisWx {
        zulu_sec: 12.0 * 3600.0,
        wind_from_deg: wind_deg,
        wind_kts,
        visibility_sm: 10.0,
        qnh_hpa: qnh,
        oat_c: 22.0,
        dewpoint_c: None,
    }
}

#[test]
fn atis_loops_while_tuned_and_cuts_editions_on_real_change() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ATIS.lock().unwrap() = Some(AtisState {
        khz: 118500,
        airport: "Gibraltar".into(),
        runways: Vec::new(),
        letter: 'A',
        dialect: Dialect::Icao,
        edition: None,
        last_spoken: None,
    });
    let t0 = Instant::now();
    // Off frequency: silence, no edition.
    assert!(atis_tick(131200, &wx(250.0, 12.0, 1018.0), t0).is_none());
    assert_eq!(atis_current_letter(), None, "nothing aired yet");
    // Tuned: the broadcast, information Alpha.
    let r = atis_tick(118500, &wx(250.0, 12.0, 1018.0), t0).expect("broadcast");
    assert!(r.contains("information Alpha"), "{r}");
    assert!(r.contains("QNH one zero one eight"), "{r}");
    assert_eq!(atis_current_letter(), Some('A'));
    // Still tuned, seconds later: the loop is between repeats — silence.
    assert!(atis_tick(
        118500,
        &wx(250.0, 12.0, 1018.0),
        t0 + std::time::Duration::from_secs(10)
    )
    .is_none());
    // Repeat due, weather unchanged: SAME letter.
    let r = atis_tick(
        118500,
        &wx(251.0, 13.0, 1018.2),
        t0 + std::time::Duration::from_secs(50),
    )
    .expect("repeat");
    assert!(
        r.contains("information Alpha"),
        "wobble must not burn a letter: {r}"
    );
    // A real pressure change cuts information Bravo.
    let r = atis_tick(
        118500,
        &wx(250.0, 12.0, 1021.0),
        t0 + std::time::Duration::from_secs(100),
    )
    .expect("new edition");
    assert!(r.contains("information Bravo"), "{r}");
    assert_eq!(atis_current_letter(), Some('B'));
    *ATIS.lock().unwrap() = None;
}

/// Anchorage's GA ramp north of 15/33 with Lake Hood Strip next door
/// (2026-09-10): Anchorage's FIRST runway row is 7R/25L, 2.7 nm away,
/// so by threshold Lake Hood (0.9 nm) is "nearer" and the whole sortie
/// was cleared to taxi 1.4 km across Anchorage's aprons to Lake Hood's
/// network. By footprint the ramp is 0.56 nm from 15/33: Anchorage takes
/// over, and it is the crew's note, not a handoff.
fn run_adjacent_fields_case() {
    let rwy = |a: &str, b: &str, la: f64, lo: f64, lb: f64, lob: f64| nav::Runway {
        ident_a: a.into(),
        ident_b: b.into(),
        lat_a: la,
        lon_a: lo,
        lat_b: lb,
        lon_b: lob,
        width_m: 45.0,
        surface: 0,
    };
    let mut anc = region_apt("Ted Stevens Anchorage Intl", 61.1678, -150.0429);
    anc.apt.runways = vec![
        rwy(
            "7R",
            "25L",
            61.1678122,
            -150.0428651,
            61.1678817,
            -149.9726428,
        ),
        rwy(
            "15",
            "33",
            61.1997218,
            -150.0145266,
            61.1710187,
            -149.9984751,
        ),
    ];
    let mut lhd = region_apt("Lake Hood Strip", 61.1895, -149.9685);
    lhd.apt.runways = vec![rwy(
        "14",
        "32",
        61.1894983,
        -149.9684894,
        61.1836374,
        -149.9622678,
    )];
    *REGION.lock().unwrap() = vec![anc, lhd];
    *ACTIVE_FIELD.lock().unwrap() = Some(("Lake Hood Strip".into(), 61.1895, -149.9685));
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    let sw =
        region_tick(61.19636, -149.9965, true, Instant::now()).expect("ramp is Anchorage's");
    assert!(sw.crew, "{}", sw.line);
    assert!(sw.line.contains("Ted Stevens Anchorage"), "{}", sw.line);
    // And from Lake Hood's own parallel taxiway (60 m west of 14/32),
    // Lake Hood stays: that pavement is theirs.
    *ACTIVE_FIELD.lock().unwrap() = Some(("Lake Hood Strip".into(), 61.1895, -149.9685));
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    assert!(region_tick(61.18743, -149.96737, true, Instant::now()).is_none());
}

fn region_apt(name: &str, lat: f64, lon: f64) -> RegionApt {
    RegionApt {
        icao: name.get(0..4).unwrap_or("XXXX").to_uppercase(),
        apt: nav::AirportInfo {
            transition_level_ft: None,
            allows_circuits: None,
            pattern_sides: Vec::new(),
            papis: Vec::new(),
            name: name.into(),
            elev_ft: 0.0,
            runways: Vec::new(),
            freqs: vec![nav::AtcFreq {
                kind: nav::FREQ_TOWER,
                khz: 118100,
                name: "Tower".into(),
            }],
        },
        lat,
        lon,
    }
}

#[test]
fn a_tower_only_strip_loses_the_region_but_wins_its_own_frequency() {
    // LEAX 2026-09-15: Leoni Benabu publishes one row (a tower on 123.5)
    // four miles ahead; Malaga, 17 nm off with ATIS, ground, approach and
    // departure, keeps the sky on the service-weighted pick however
    // close the arrival gets. The crew's retune to 123.5 arms the strip
    // anyway - the same "you call it, you get it" as a keyed transmission.
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut malaga = region_apt("Malaga", 36.675, -4.499);
    malaga.apt.freqs[0].khz = 118155;
    for (kind, khz) in [
        (nav::FREQ_ATIS, 118450),
        (nav::FREQ_GROUND, 121800),
        (nav::FREQ_APPROACH, 118455),
        (nav::FREQ_DEPARTURE, 121025),
    ] {
        malaga.apt.freqs.push(nav::AtcFreq { kind, khz, name: "Malaga".into() });
    }
    let mut strip = region_apt("Leoni Benabu", 36.80, -4.136);
    strip.apt.freqs[0].khz = 123500;
    *REGION.lock().unwrap() = vec![malaga, strip];
    *ACTIVE_FIELD.lock().unwrap() = Some(("Malaga".into(), 36.675, -4.499));
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    let now = Instant::now();
    // Airborne 4 nm south of the strip at 2,700 ft: the region keeps Malaga.
    let (lat, lon) = (36.735, -4.15);
    assert!(region_tick(lat, lon, false, now).is_none());
    assert!(PENDING_STATION.lock().unwrap().is_none(), "the service bias holds");
    // The crew tunes the strip's tower: armed.
    arm_station_for_freq(123500, lat, lon, 2700.0, now);
    let armed = PENDING_STATION.lock().unwrap().take().expect("swap armed for the strip");
    assert_eq!(armed.1.name, "Leoni Benabu");
    *REGION.lock().unwrap() = Vec::new();
    *ACTIVE_FIELD.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
}

#[test]
fn a_new_flight_arms_its_field_even_when_it_is_the_one_up() {
    // LEMG 2026-09-26: the 737 was put on a 10 nm final for 13 from a
    // seat on runway 30, and the station that had been watching it on the
    // runway carried on as if nothing had happened. A new flight arms its
    // field, over any swap already waiting, and the old flight's pin and
    // handoff go with it. The field already up is served in place by the
    // main loop (`Station::new_flight_here`, 2026-10-13), never rebuilt.
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut malaga = region_apt("LEMG Malaga", 36.675, -4.499);
    malaga.apt.freqs[0].khz = 118155;
    let other = region_apt("LXGB Gibraltar", 36.15, -5.35);
    *REGION.lock().unwrap() = vec![malaga, other];
    *ACTIVE_FIELD.lock().unwrap() = Some(("LEMG Malaga".into(), 36.675, -4.499));
    *PENDING_STATION.lock().unwrap() =
        Some(("LXGB".into(), region_apt("LXGB Gibraltar", 36.15, -5.35).apt));
    *ARRIVAL_PIN.lock().unwrap() = Some(131200);
    let now = Instant::now();
    assert_eq!(arm_new_flight("lemg", now), Some(118155));
    let armed = PENDING_STATION.lock().unwrap().take().expect("a fresh station armed");
    assert_eq!(armed.0, "LEMG");
    assert!(ARRIVAL_PIN.lock().unwrap().is_none());
    assert_eq!(*LAST_REGION_SWITCH.lock().unwrap(), Some(now));
    // A field the index lacks arms nothing.
    assert_eq!(arm_new_flight("ZZZZ", now), None);
    assert!(PENDING_STATION.lock().unwrap().is_none());
    *REGION.lock().unwrap() = Vec::new();
    *ACTIVE_FIELD.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
}

/// A NEW FLIGHT OF OURS AT THE FIELD ALREADY UP resets our part of the
/// station and nothing else. LEMG 2026-09-26: the desk that had watched the
/// 737 lined up on 30 earned the arrival on 13 "you landed without
/// clearance", so our desk, clearance, runway claim and compliance memory
/// all go. LEMG 2026-10-13: an EFB "Move here" rebuilt the whole station,
/// cleared every traffic strip, and the traffic link sent thirteen moving
/// aircraft off the frequency, so the traffic's strips and desks, and the
/// runway in use, stay.
#[test]
fn a_new_flight_at_the_field_up_resets_our_desk_and_keeps_the_traffic() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = super::strip_tests::ClearStrips;
    Station::uninstall();
    let mut st = lxgb();
    st.install();
    st.runway_in_use = Some("09".into());
    // The flight that ended: lined up on 09, the tower's word on our strip,
    // the compliance watch's memory of a runway start.
    st.desk.twr.state = tower::State::LinedUp;
    st.desk.told_runway = Some("09".into());
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::LinedUp,
        callsign: "November one seven two Sierra Papa".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: true,
    });
    {
        let mut f = OWN_STRIP.compliance_flight.lock().unwrap();
        f.prev_on_ground = Some(true);
        f.runway_start = Some("09".into());
    }
    // The traffic: a strip cleared to land, and the desk working it.
    let traffic = open_traffic_strip(7);
    *traffic.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::ClearedToLand,
        callsign: "Ryanair four three one".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: true,
    });
    st.desks.insert(
        FlightId::Traffic(7),
        StripDesk::for_strip(StripRef::Traffic(traffic.clone())),
    );

    retire_for_new_flight();
    assert!(new_flight_pending());
    assert!(st.works(&lxgb().apt), "the swap armed is this station's field");
    st.new_flight_here();

    // Ours: nothing of the last flight.
    assert!(!new_flight_pending(), "the station answers for the new flight");
    assert_eq!(st.desk.twr.state, tower::Tower::new().state);
    assert_eq!(st.desk.told_runway, None);
    assert_eq!(crate::party_line::clearance_state(), None);
    assert_eq!(crate::party_line::clearance_runway(), None);
    {
        let f = OWN_STRIP.compliance_flight.lock().unwrap();
        assert_eq!(f.prev_on_ground, None);
        assert_eq!(f.runway_start, None);
    }
    assert!(COMPLIANCE.lock().unwrap().zone_last.is_some(), "zone-watch grace");
    // The traffic's: its strip, its clearance, its desk; the runway in use.
    let kept = traffic_strip(7).expect("the traffic strip stands");
    assert!(std::sync::Arc::ptr_eq(&kept, &traffic));
    assert_eq!(
        kept.clearance.lock().unwrap().as_ref().map(|c| c.state),
        Some(tower::State::ClearedToLand)
    );
    assert!(st.desks.contains_key(&FlightId::Traffic(7)), "its desk stands");
    assert_eq!(st.runway_in_use(), Some("09"));
    // The new flight's call is answered by a fresh desk, and published.
    let wx = Wx { wind_from_deg: 90.0, wind_kts: 8.0 };
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        false,
        false,
        None,
        false,
        None,
    );
    st.respond("", Service::Tower, tower::Intent::ReportFinal, &ctx, wx);
    assert!(
        crate::party_line::clearance_state().is_some(),
        "the new flight's word is published"
    );
    Station::uninstall();
}

/// A station works the field it was built for and no other.
#[test]
fn a_station_works_its_own_field_only() {
    let st = lxgb();
    assert!(st.works(&lxgb().apt));
    let mut elsewhere = lxgb().apt;
    elsewhere.name = "Malaga".into();
    assert!(!st.works(&elsewhere));
}

/// EGKK, 2026-09-28 (#151): after a day at LEMG the runner put the 737 on
/// Gatwick stand 71 (51.15540 N 0.16537 W). The new-flight watch re-armed
/// the ACTIVE field (LEMG, tower 118155) and its cooldown held the
/// region's relocation off for 180 s: full auto departed Gatwick as an
/// uncontrolled field and looked stand 71 up at LEMG, 1,639 km away. A
/// new flight's station is the field under its NEW position.
fn lemg_and_egkk() -> Vec<RegionApt> {
    let mut malaga = region_apt("LEMG Malaga", 36.675, -4.499);
    malaga.apt.freqs[0].khz = 118155;
    let mut gatwick = region_apt("EGKK London Gatwick", 51.1481, -0.1903);
    gatwick.apt.freqs[0].khz = 124230;
    gatwick.apt.runways = vec![nav::Runway {
        ident_a: "08R".into(),
        ident_b: "26L".into(),
        lat_a: 51.14495,
        lon_a: -0.21262,
        lat_b: 51.15132,
        lon_b: -0.16813,
        width_m: 45.0,
        surface: 1,
    }];
    vec![malaga, gatwick]
}

/// The enroute setup's world: an IFR flight mid-route is worked by the
/// nearest field with a RADAR unit (a tower-only field nearer is passed
/// over), keeps a discrete code, and is airborne from the start.
#[test]
fn a_new_flight_in_the_air_is_an_ifr_flight_under_a_radar_unit() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut region = lemg_and_egkk();
    let mut bham = region_apt("EGBB Birmingham", 52.4539, -1.7481);
    bham.apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_APPROACH,
        khz: 123980,
        name: "Approach".into(),
    });
    let tower_only = region_apt("EGBW Wellesbourne", 52.192, -1.614);
    region.push(bham);
    region.push(tower_only);
    // 30 nm before the top of descent on EGKK-EGBB (between BUZAD and
    // WELIN): Wellesbourne is nearer and has only a tower.
    let (lat, lon) = (52.05, -0.66);
    assert_eq!(enroute_field_in(&region, lat, lon).unwrap().icao, "EGBB");
    // Nothing with radar within reach: nothing.
    assert!(enroute_field_in(&region, 45.0, 10.0).is_none());
    *REGION.lock().unwrap() = region;
    *OWN_STRIP.ifr_flight.lock().unwrap() = None;
    let st = arm_new_flight_enroute(lat, lon, "WAT626", Instant::now()).unwrap();
    assert_eq!((st.icao.as_str(), st.position, st.khz), ("EGBB", "Approach", 123980));
    assert_eq!(st.tower_khz, 118100);
    assert_eq!(st.code, discrete_squawk("WAT626"));
    let f = OWN_STRIP.ifr_flight.lock().unwrap().clone().unwrap();
    assert!(f.airborne && f.code == st.code && f.callsign == "WAT626");
    *OWN_STRIP.ifr_flight.lock().unwrap() = None;
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    REGION.lock().unwrap().clear();
}

#[test]
fn a_new_flight_takes_the_station_of_the_field_it_was_put_at() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let region = lemg_and_egkk();
    // Stand 71: Gatwick's, whatever station was up.
    let stand_71 = (51.155_395_5, -0.165_369_4);
    let at = field_under_in(&region, stand_71.0, stand_71.1, true).expect("Gatwick");
    assert_eq!(at.icao, "EGKK");
    // Malaga's ramp is Malaga's.
    assert_eq!(field_under_in(&region, 36.6716, -4.4890, true).unwrap().icao, "LEMG");
    // A field with no tower in the index (on the ground 20 nm from both):
    // nobody's, nothing armed.
    assert!(field_under_in(&region, 51.4, -0.6, true).is_none());
    // Airborne on a 10 nm final to Gatwick: Gatwick.
    assert_eq!(field_under_in(&region, 51.12, -0.45, false).unwrap().icao, "EGKK");

    // Live: the station up is Malaga's, a relocation was switched 20 s ago
    // (the cooldown that held the region back), and the new flight at
    // stand 71 arms Gatwick all the same.
    *REGION.lock().unwrap() = region;
    *ACTIVE_FIELD.lock().unwrap() = Some(("LEMG Malaga".into(), 36.675, -4.499));
    *PENDING_STATION.lock().unwrap() = None;
    let now = Instant::now();
    *LAST_REGION_SWITCH.lock().unwrap() = Some(now);
    assert_eq!(
        arm_new_flight_at(stand_71.0, stand_71.1, true, now),
        Some(("EGKK".to_string(), 124230))
    );
    let armed = PENDING_STATION.lock().unwrap().take().expect("Gatwick armed");
    assert_eq!(armed.1.name, "EGKK London Gatwick");
    // Nowhere towered: nothing armed.
    assert_eq!(arm_new_flight_at(51.4, -0.6, true, now), None);
    assert!(PENDING_STATION.lock().unwrap().is_none());
    *REGION.lock().unwrap() = Vec::new();
    *ACTIVE_FIELD.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
}

/// Full auto's arm waits for the departure field's station: the one up
/// must be the field under the wheels, and not the last flight's
/// worksheet (#151).
#[test]
fn the_departure_station_is_the_field_under_the_wheels() {
    use DepartureStation::*;
    let want = || "EGKK".to_string();
    // The EGKK case at the arm: Malaga up, nothing armed.
    assert_eq!(judge_departure_station(Some("LEMG"), Some("EGKK"), None, true), Arm { want: want() });
    assert_eq!(judge_departure_station(Some("LEMG"), Some("EGKK"), None, false), Arm { want: want() });
    // Armed, not yet installed.
    assert_eq!(judge_departure_station(Some("LEMG"), Some("EGKK"), Some("EGKK"), true), Waiting { want: want() });
    // Another field armed (a stale swap): this one is armed over it.
    assert_eq!(judge_departure_station(Some("LEMG"), Some("EGKK"), Some("LEMG"), true), Arm { want: want() });
    // Installed.
    assert_eq!(judge_departure_station(Some("EGKK"), Some("EGKK"), None, false), Ready);
    // The right field, still holding the last flight's worksheet.
    assert_eq!(judge_departure_station(Some("EGKK"), Some("EGKK"), None, true), Arm { want: want() });
    // No station at all yet.
    assert_eq!(judge_departure_station(None, Some("EGKK"), None, false), Arm { want: want() });
    // No towered field under us: an uncontrolled departure is ready as is.
    assert_eq!(judge_departure_station(Some("LEMG"), None, None, true), Ready);
}

/// LEMG, 2026-09-28 (sortie 1790584266-approach): the previous case was
/// cleared to land on 12; the reposition to a 10 nm final for 13 armed
/// autoland 1.3 s later and it read "cleared to land 12" off the party
/// line before the crew's new final call went out (event
/// `runway_from_clearance`, planned 13, cleared 12, xtk -3,638 m on 12's
/// line). A new flight retires the old one's clearances before anything
/// reads them, and the old station, still answering until the new one is
/// installed, publishes nothing.
#[test]
fn a_new_flight_retires_the_last_flights_landing_clearance() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::ClearedToLand,
        callsign: "November eight one six November November".into(),
        runway_ident: "12".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *OWN_STRIP.landing_handoff.lock().unwrap() = Some("12".into());
    *OWN_STRIP.pending_go_around.lock().unwrap() = true;
    // The last flight's radar watch too (its "radar contact" fired at
    // the next liftoff, LEMG sortie 1790584715).
    let _ = lxgb().assign_squawk(
        "November eight one six November November",
        StationSpeaker::Departure,
    );
    assert!(squawk_watch_active());
    assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("12"));
    crate::party_line::retire_for_new_flight();
    assert!(
        !squawk_watch_active(),
        "the radar watch was the old flight's"
    );
    assert_eq!(crate::party_line::clearance_state(), None);
    assert_eq!(crate::party_line::clearance_runway(), None);
    assert_eq!(crate::party_line::readback_owed(), None);
    assert_eq!(crate::party_line::landing_handoff_runway(), None);
    assert!(!crate::party_line::take_pending_go_around());
    // The old station answers a call before the swap: nothing published.
    let wx = Wx { wind_from_deg: 90.0, wind_kts: 8.0 };
    let mut old = lxgb();
    let ctx = old.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        false,
        false,
        None,
        false,
        None,
    );
    old.respond("", Service::Tower, tower::Intent::ReportFinal, &ctx, wx);
    assert!(
        OWN_STRIP.clearance.lock().unwrap().is_none(),
        "the old station's word is not this flight's"
    );
    // The new flight's station is up: its replies are the clearance.
    let mut new = lxgb();
    new.install();
    new.respond("", Service::Tower, tower::Intent::ReportFinal, &ctx, wx);
    assert_eq!(
        crate::party_line::clearance_state(),
        Some(tower::State::ClearedToLand)
    );
    Station::uninstall();
}

/// The tower's wind, a zone challenge nobody answered and our track
/// before a jump outlived the flight and the field they belonged to: a
/// re-sequence fell back to the last field's wind, the next mission's
/// first enroute pass drained a challenge raised on the last flight, and
/// a groundspeed was derived across a reposition. Retired with the
/// flight; the wind and the challenge with the field too.
#[test]
fn a_new_flight_or_field_forgets_wind_challenge_and_track() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let arm = || {
        *TOWER_WIND.lock().unwrap() = Some((130.0, 12.0));
        *OWN_STRIP.zone_challenge.lock().unwrap() = true;
        note_own_fix(36.67, -4.49, false, Instant::now());
    };
    arm();
    retire_for_new_flight();
    assert_eq!(*TOWER_WIND.lock().unwrap(), None);
    assert!(!crate::party_line::take_zone_challenge());
    assert!(OWN_STRIP.own_fixes.lock().unwrap().is_empty());
    NEW_FLIGHT_PENDING.store(false, std::sync::atomic::Ordering::SeqCst);

    arm();
    lxgb().install();
    assert_eq!(*TOWER_WIND.lock().unwrap(), None);
    assert!(!crate::party_line::take_zone_challenge());

    arm();
    Station::uninstall();
    assert_eq!(*TOWER_WIND.lock().unwrap(), None);
    assert!(!crate::party_line::take_zone_challenge());
    OWN_STRIP.own_fixes.lock().unwrap().clear();
}

/// The crew's own runway at an uncontrolled field stands in for a cleared
/// one (`party_line::clearance_runway`), but outlived the route it came
/// with: `retire_for_new_flight` and `install` dropped `OWN_STRIP.taxi_route` and
/// kept `OWN_STRIP.self_taxi`, so the next flight's line-up and takeoff cross-check
/// read the last flight's runway end. It dies with the route now.
#[test]
fn a_new_flight_or_field_forgets_the_crews_own_runway() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    Station::uninstall();
    let arm = || {
        *OWN_STRIP.self_taxi.lock().unwrap() = Some("21".into());
        assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("21"));
    };
    arm();
    retire_for_new_flight();
    assert_eq!(crate::party_line::clearance_runway(), None, "a new flight");
    arm();
    lxgb().install();
    assert_eq!(crate::party_line::clearance_runway(), None, "a new field");
    arm();
    Station::uninstall();
    assert_eq!(crate::party_line::clearance_runway(), None, "ATC off");
}

/// `arm_new_flight` drops the old flight's region handoff and arrival pin,
/// but only for a field with a tower (it returns before it clears them):
/// a new flight at an untowered field kept a handoff the crew's first
/// enroute pass would read back and retune for. A new flight drops them
/// itself now; a region handoff's `install` still carries them (the main
/// loop installs the next station before the crew consumes the handoff).
#[test]
fn a_new_flight_drops_the_old_handoff_and_pin_and_an_install_carries_them() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let arm = || {
        *REGION_HANDOFF.lock().unwrap() = Some(RegionHandoff {
            to: "Malaga Tower".into(),
            khz: 118_150,
            squawk_vfr: true,
            ifr_code: None,
        });
        *ARRIVAL_PIN.lock().unwrap() = Some(118_150);
    };
    arm();
    lxgb().install();
    assert!(REGION_HANDOFF.lock().unwrap().is_some(), "install carries the handoff");
    assert_eq!(*ARRIVAL_PIN.lock().unwrap(), Some(118_150), "and the pin");
    retire_for_new_flight();
    assert_eq!(crate::party_line::take_region_handoff(), None);
    assert_eq!(*ARRIVAL_PIN.lock().unwrap(), None);
    arm();
    Station::uninstall();
    assert_eq!(crate::party_line::take_region_handoff(), None);
    assert_eq!(*ARRIVAL_PIN.lock().unwrap(), None);
}

/// The doors' order: a new flight's flag goes up with the clearances gone,
/// and only a whole install takes it down, with the station's own facts
/// published (never an empty field in between for the region watch).
#[test]
fn install_ends_the_new_flight_with_the_field_published() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    retire_for_new_flight();
    assert!(new_flight_pending());
    let st = lxgb();
    st.install();
    assert!(!new_flight_pending());
    let panel = ACTIVE_PANEL.lock().unwrap().as_ref().map(|p| p.0.clone());
    let freqs = !CONTROLLER_FREQS.lock().unwrap().is_empty();
    let grace = COMPLIANCE.lock().unwrap().zone_last.is_some();
    Station::uninstall();
    assert_eq!(panel, Some(st.airport_name().to_string()));
    assert!(freqs);
    assert!(grace, "zone-watch grace");
    assert!(ACTIVE_PANEL.lock().unwrap().is_none());
    assert!(CONTROLLER_FREQS.lock().unwrap().is_empty());
    assert!(COMPLIANCE.lock().unwrap().zone_last.is_none());
}

/// LFLL-EGLL, 2026-09-17. Inbound to Heathrow with the inbound call made
/// and answered, the region's tick found London City eight miles off the
/// nose against Heathrow's fifteen — nearer AND service-weighted — and
/// took the station. The final report and the taxi-in request both came
/// back `void`, the 737 landed on 27L with no clearance and sat on it for
/// three minutes. A committed arrival owns the station until it is over.
#[test]
fn a_committed_arrival_keeps_its_tower_from_a_nearer_field() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut heathrow = region_apt("London Heathrow", 51.4775, -0.4614);
    heathrow.apt.freqs[0].khz = 118505;
    let mut city = region_apt("London City", 51.5050, 0.0554);
    city.apt.freqs[0].khz = 118075;
    for (kind, khz) in [
        (nav::FREQ_ATIS, 136350),
        (nav::FREQ_GROUND, 121825),
        (nav::FREQ_APPROACH, 132700),
    ] {
        city.apt.freqs.push(nav::AtcFreq {
            kind,
            khz,
            name: "City".into(),
        });
    }
    *REGION.lock().unwrap() = vec![heathrow, city];
    *ACTIVE_FIELD.lock().unwrap() = Some(("London Heathrow".into(), 51.4775, -0.4614));
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *ARRIVAL_PIN.lock().unwrap() = None;
    // The position the swap actually happened from, 13:06:42Z.
    let (lat, lon) = (51.4, -0.1);
    let now = Instant::now();
    // Unpinned, the generic rule still prefers London City — this is the
    // bug, reproduced, not a rule we are changing for everyone.
    assert!(
        region_tick(lat, lon, false, now).is_some(),
        "the nearer field wins by itself"
    );
    assert_eq!(
        PENDING_STATION.lock().unwrap().take().map(|(_, a)| a.name),
        Some("London City".to_string())
    );
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    // Inbound to Heathrow: the pin holds the station through the zone.
    pin_arrival_station(Some(118505));
    assert!(
        region_tick(lat, lon, false, now).is_none(),
        "the arrival keeps its tower"
    );
    assert!(PENDING_STATION.lock().unwrap().is_none());
    // And the pin is a hold, not a freeze: on the ground the pavement
    // under the wheels still relocates the station, and drops the pin.
    assert!(
        region_tick(51.5050, 0.0554, true, now).is_some(),
        "landed at City after all"
    );
    assert!(
        ARRIVAL_PIN.lock().unwrap().is_none(),
        "a stale pin never strands the station"
    );
    *REGION.lock().unwrap() = Vec::new();
    *ACTIVE_FIELD.lock().unwrap() = None;
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
}

#[test]
fn the_exact_channel_takes_the_swap_over_a_nearer_field_five_khz_off() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // LXGB-LEJR 2026-10-09: Jerez Tower on 118.555, Trebujena on 118.550
    // and nearer.
    let mut jerez = region_apt("Jerez", 36.74, -6.06);
    jerez.apt.freqs[0].khz = 118555;
    let mut trebujena = region_apt("Trebujena", 36.87, -6.20);
    trebujena.apt.freqs[0].khz = 118550;
    *REGION.lock().unwrap() = vec![region_apt("Gibraltar", 36.15, -5.35), jerez, trebujena];
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
    let now = Instant::now();
    for (khz, want) in [(118555, "Jerez"), (118550, "Trebujena")] {
        *PENDING_STATION.lock().unwrap() = None;
        arm_station_for_freq(khz, 37.0, -6.25, 3000.0, now);
        let armed = PENDING_STATION.lock().unwrap().take().expect("swap armed");
        assert_eq!(armed.1.name, want, "{khz}");
    }
    *REGION.lock().unwrap() = Vec::new();
    *ACTIVE_FIELD.lock().unwrap() = None;
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
}

#[test]
fn transmission_on_another_fields_freq_arms_the_swap() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut badajoz = region_apt("Badajoz", 38.89, -6.82);
    badajoz.apt.freqs[0].khz = 122100;
    *REGION.lock().unwrap() = vec![region_apt("Gibraltar", 36.15, -5.35), badajoz];
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    let now = Instant::now();
    // Parked in Extremadura at ~1,000 ft MSL, COM1 on Badajoz tower
    // (22 nm, inside VHF range): the swap arms for Badajoz.
    arm_station_for_freq(122100, 38.74, -6.40, 1075.0, now);
    let armed = PENDING_STATION.lock().unwrap().take().expect("swap armed");
    assert_eq!(armed.1.name, "Badajoz");
    assert!(
        LAST_REGION_SWITCH.lock().unwrap().is_some(),
        "region cooldown stamped"
    );
    // A frequency nobody in range publishes: nothing arms.
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    arm_station_for_freq(999000, 38.74, -6.40, 1075.0, now);
    assert!(PENDING_STATION.lock().unwrap().is_none());
    // The ACTIVE station's own frequency never re-arms a swap to itself.
    *ACTIVE_FIELD.lock().unwrap() = Some(("Badajoz".into(), 38.89, -6.82));
    arm_station_for_freq(122100, 38.74, -6.40, 1075.0, now);
    assert!(PENDING_STATION.lock().unwrap().is_none());
    // Same tune from the ramp at Gibraltar (200+ nm away): out of VHF
    // range, no swap — you can't raise a tower you can't reach.
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
    arm_station_for_freq(122100, 36.15, -5.35, 15.0, now);
    assert!(PENDING_STATION.lock().unwrap().is_none());
    *REGION.lock().unwrap() = Vec::new();
    *ACTIVE_FIELD.lock().unwrap() = None;
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
}

#[test]
fn unnamed_gateway_tower_rows_are_not_staffed() {
    // LEMY: an ultralight strip whose air-to-air frequency the Gateway
    // typed as "1054 130125 unnamed entity" — a frequency, not a tower.
    let apt = nav::AirportInfo {
        transition_level_ft: None,
        allows_circuits: None,
        pattern_sides: Vec::new(),
        papis: Vec::new(),
        name: "Merida - Royanejos".into(),
        elev_ft: 883.0,
        runways: Vec::new(),
        freqs: vec![nav::AtcFreq {
            kind: nav::FREQ_TOWER,
            khz: 130125,
            name: "unnamed entity".into(),
        }],
    };
    assert!(apt.staffed_tower().is_none());
    // A real tower row (Badajoz: "TWR CIVIL") stays staffed.
    let apt = nav::AirportInfo {
        transition_level_ft: None,
        allows_circuits: None,
        pattern_sides: Vec::new(),
        papis: Vec::new(),
        name: "Badajoz- Talavera La Real".into(),
        elev_ft: 607.0,
        runways: Vec::new(),
        freqs: vec![nav::AtcFreq {
            kind: nav::FREQ_TOWER,
            khz: 122100,
            name: "TWR CIVIL".into(),
        }],
    };
    assert!(apt.staffed_tower().is_some());
}

/// EGFH 2026-10-01: Cardiff remained the active regional station and
/// answered an inbound call for an untowered destination, clearing its
/// own runway 30 while the aircraft landed EGFH 22. The destination seam
/// must retire that authority before any arrival call can be routed.
#[test]
fn uncontrolled_arrival_retires_the_previous_controller() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *CONTROLLER_FREQS.lock().unwrap() = vec![(133100, StationSpeaker::Tower)];
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Inbound,
        callsign: "N172SP".into(),
        runway_ident: "30".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *OWN_STRIP.assigned_squawk.lock().unwrap() = Some(4503);
    OUT_OF_REACH.store(false, std::sync::atomic::Ordering::Relaxed);

    assert!(crate::party_line::station_reaches());
    begin_uncontrolled_arrival("EGFH");
    assert!(uncontrolled_arrival());
    assert!(!crate::party_line::station_reaches());
    assert_eq!(crate::party_line::home_tower_khz(), None);
    assert!(OWN_STRIP.clearance.lock().unwrap().is_none());
    assert!(OWN_STRIP.assigned_squawk.lock().unwrap().is_none());

    end_arrival();
    assert!(crate::party_line::station_reaches());
    *CONTROLLER_FREQS.lock().unwrap() = Vec::new();
}

/// LILB 2026-09-25: parked in Italy with the Gibraltar station active,
/// the station answered, cleared us and handed over a taxi to its own
/// runway. On the ground out of reach the station is silent and what it
/// issued is retired; the crew's own route is left alone; airborne the
/// last controller keeps the flight.
#[test]
fn a_station_out_of_reach_on_the_ground_is_silent_and_retires_its_clearance() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let cleared = || ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "N172SP".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: false,
    };
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
    *OWN_STRIP.clearance.lock().unwrap() = Some(cleared());
    *OWN_STRIP.assigned_squawk.lock().unwrap() = Some(4503);
    *OWN_STRIP.self_taxi.lock().unwrap() = None;
    OUT_OF_REACH.store(false, std::sync::atomic::Ordering::Relaxed);
    // At the field: in reach, nothing retired.
    note_aircraft(36.15, -5.34, true);
    assert!(!out_of_reach());
    assert!(OWN_STRIP.clearance.lock().unwrap().is_some());
    // Parked at LILB: out of reach, the clearance and squawk are gone.
    note_aircraft(45.77, 9.16, true);
    assert!(out_of_reach());
    assert!(OWN_STRIP.clearance.lock().unwrap().is_none());
    assert!(OWN_STRIP.assigned_squawk.lock().unwrap().is_none());
    assert_eq!(crate::party_line::home_tower_khz(), None);
    assert!(!crate::party_line::station_reaches());
    // Airborne the station keeps the flight, and the crew's own route
    // and runway are over.
    *OWN_STRIP.self_taxi.lock().unwrap() = Some("03".into());
    assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("03"));
    note_aircraft(45.77, 9.16, false);
    assert!(!out_of_reach());
    assert_eq!(crate::party_line::clearance_runway(), None);
    // The crew's own route survives the station going out of reach.
    *OWN_STRIP.self_taxi.lock().unwrap() = Some("03".into());
    *OWN_STRIP.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(45.77, 9.16), (45.771, 9.161)],
        via: "Alpha".into(),
        seg: Vec::new(),
        holding_point: "A".into(),
        lead_on: Vec::new(),
    });
    note_aircraft(45.77, 9.16, true);
    assert!(OWN_STRIP.taxi_route.lock().unwrap().is_some());
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.self_taxi.lock().unwrap() = None;
    OUT_OF_REACH.store(false, std::sync::atomic::Ordering::Relaxed);
    *ACTIVE_FIELD.lock().unwrap() = None;
}

#[test]
fn only_a_designator_is_a_taxiway_name() {
    for yes in ["A", "B2", "TL", "A13", "k"] {
        assert!(designator(yes), "{yes}");
    }
    for no in ["Taxy", "21_hold", "", "A123", "Main apron", "12"] {
        assert!(!designator(no), "{no}");
    }
}

/// #24's short-runway branch at a real field (LFPB, the real apt.dat and
/// taxi network): the crew's call for the longer runway, in the words
/// `full_auto::runway_request_call` makes for a 737 cleared to 27 (1,847
/// m against 2,347 needed; `real_install_cleared_runway_fit` pins them),
/// from the stand the 2026-09-22 sortie pushed off. The tower grants 25
/// as the taxi to it, 25 is the runway in use and on the party line (what
/// `judge_cleared_runway` waits for), and the published route ends on
/// 07/25's own centreline.
/// `cargo test --release -p atc -- --ignored real_install_short_cleared_runway --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_short_cleared_runway_is_granted_the_longer_one() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let apt = crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    if !std::path::Path::new(&apt).exists() {
        return;
    }
    let mut st = Station::load(&apt, "LFPB").expect("LFPB has a tower");
    let net = nav::find_taxi_net(&apt, "LFPB").unwrap().expect("LFPB's network");
    *TAXI_NET.lock().unwrap() = Some(std::sync::Arc::new(net));
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    let wx = calm();
    let words = "unable runway two seven, request runway two five for length";
    let intent = tower::parse_intent(words);
    assert_eq!(intent, tower::Intent::RequestRunway);
    st.note_stated_runway(words, intent);
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    st.apply_runway_request(&mut ctx, Some((48.95620, 2.43880)));
    let r = st.route_keyed(words, Service::Tower, &ctx, wx);
    println!("{r}");
    assert!(r.contains("runway two five approved, taxi to holding point"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::TaxiCleared);
    assert_eq!(st.runway_in_use.as_deref(), Some("25"));
    assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("25"));
    let route = OWN_STRIP.taxi_route.lock().unwrap().clone().expect("the taxi route published");
    let strip = st.apt.runways.iter().find(|r| r.ident_b == "25").expect("07/25");
    let last = *route.lead_on.last().or(route.path.last()).unwrap();
    let d = nav::haversine_nm(strip.lat_a, strip.lon_a, last.0, last.1) * 1852.0;
    let off = sim::angles::signed_diff_deg(
        nav::bearing_deg(strip.lat_a, strip.lon_a, last.0, last.1),
        strip.heading_a_deg(),
    );
    let xt = (d * off.to_radians().sin()).abs();
    println!(
        "holding point {} via {}, {} lead-on points, ends {xt:.1} m off 07/25",
        route.holding_point,
        route.via,
        route.lead_on.len()
    );
    assert!(xt < 20.0, "the route ends {xt:.1} m off 07/25's centreline");
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *TAXI_NET.lock().unwrap() = None;
}

/// The real scenery: LILB's own taxi to runway 03, planned and published.
/// `cargo test -p atc -- --ignored real_install_self_taxi_lilb --nocapture`
#[test]
#[ignore]
fn real_install_self_taxi_lilb() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let apt = std::env::var("XPLANE_APT_DATA").unwrap_or_else(|_| {
        crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat")
    });
    // Where the 172 sat on 2026-09-25.
    let (lat, lon) = (45.771_275, 9.164_289);
    let t = plan_self_taxi(&apt, Some("LILB"), lat, lon, Some("03"), Some((20.0, 4.0)));
    println!("{t:?}");
    let t = t.expect("planned");
    assert_eq!(t.runway, "03");
    assert!(OWN_STRIP.taxi_route.lock().unwrap().as_ref().is_some_and(|w| w.path.len() >= 2));
    // The Evolution's taxis from here (#45, 2026-09-29 and 30) failed
    // 14.2 to 16.3 m off a route that ran back to node 5 and hairpinned
    // 124 degrees: no turn on the published route past 90 now.
    let path = OWN_STRIP.taxi_route.lock().unwrap().as_ref().map(|w| w.path.clone()).unwrap();
    println!("{path:?}");
    for w in path.windows(3) {
        let brg = |a: (f64, f64), b: (f64, f64)| crate::nav::bearing_deg(a.0, a.1, b.0, b.1);
        let d = brg(w[1], w[2]) - brg(w[0], w[1]);
        let t = sim::angles::wrap180(d).abs();
        assert!(t <= 90.0, "a {t:.0} degree turn at {:?}", w[1]);
    }
    // No ICAO: the field found by the scan is the same one.
    let found = plan_self_taxi(&apt, None, lat, lon, None, None).expect("found");
    println!("{found:?}");
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.self_taxi.lock().unwrap() = None;
}

/// The line the LIEA-LEBL 737 heard at FL200 (2026-09-27), and what an
/// IFR flight hears instead.
#[test]
fn the_region_handoff_line_by_flight_rules() {
    assert_eq!(
        region_handoff_line(None, true, "Barcelona Tower", 118_105, Dialect::Icao),
        "Radar services terminated. Squawk seven thousand. Contact Barcelona Tower on one one eight decimal one zero five."
    );
    assert_eq!(
        region_handoff_line(Some("November eight one six November November"), false, "Barcelona Tower", 118_105, Dialect::Icao),
        "November eight one six November November, contact Barcelona Tower on one one eight decimal one zero five."
    );
    assert_eq!(
        region_handoff_line(None, false, "Malaga Tower", 118_100, Dialect::Icao),
        "Radar services terminated. Contact Malaga Tower on one one eight decimal one."
    );
}

/// The real region at the EGBB-EGKK switch (sortie 1790534902, +1319 s,
/// FL150 IFR at 52.1 N 1.4 W): a VFR flight there is still handed to
/// Oxford's tower, the IFR flight never to a tower-only unit and to a
/// radar position when it is handed at all.
/// `cargo test -p atc -- --ignored real_install_region_ifr --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_region_ifr_handoff_goes_to_radar() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let apt = crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    if !std::path::Path::new(&apt).exists() {
        return;
    }
    load_region(&apt);
    let egbb = REGION
        .lock()
        .unwrap()
        .iter()
        .find(|a| a.icao == "EGBB")
        .map(|a| (a.apt.name.clone(), a.lat, a.lon))
        .expect("EGBB indexed");
    let arm = || {
        *PENDING_STATION.lock().unwrap() = None;
        *LAST_REGION_SWITCH.lock().unwrap() = None;
        *ARRIVAL_PIN.lock().unwrap() = None;
        *ACTIVE_FIELD.lock().unwrap() = Some(egbb.clone());
        *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
            state: tower::State::TakeoffCleared,
            callsign: "Whiskey Alpha Tango four two three six".into(),
            runway_ident: "33".into(),
            pending_readback: None,
            holds_runway: false,
        });
    };
    let t0 = Instant::now();
    // VFR: the old tower handoff stands.
    *OWN_STRIP.ifr_flight.lock().unwrap() = None;
    arm();
    let vfr = region_tick(52.1, -1.4, false, t0).map(|s| s.line);
    let _ = crate::party_line::take_region_handoff();
    eprintln!("VFR: {vfr:?}");
    assert!(vfr.as_deref().is_some_and(|l| l.contains("Oxford Kidlington Tower")), "{vfr:?}");
    // IFR on 4505, as flown.
    arm();
    publish_ifr_clearance(
        &OWN_STRIP,
        tower::IfrClearance {
            dest: "EGKK".into(),
            sid: Some("ADME1M".into()),
            runway: "33".into(),
            initial_alt_ft: 5000,
            squawk: 4505,
        },
        "Whiskey Alpha Tango four two three six",
        None,
        None,
    );
    let ifr = region_tick(52.1, -1.4, false, t0).map(|s| s.line);
    let h = crate::party_line::take_region_handoff();
    eprintln!("IFR: {ifr:?} {h:?}");
    // Oxford publishes approach 125.09 (and "Oxford Radar"): the flight
    // goes there on radar, its code kept.
    let h = h.expect("handed to a radar position");
    assert_eq!((h.to.as_str(), h.khz, h.ifr_code), ("Oxford Kidlington Approach", 125_090, Some(4505)));
    assert!(!ifr.unwrap_or_default().contains("Tower"));
    *OWN_STRIP.ifr_flight.lock().unwrap() = None;
    *OWN_STRIP.ifr_clearance.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    *ACTIVE_FIELD.lock().unwrap() = None;
    REGION.lock().unwrap().clear();
}

#[test]
fn region_hands_off_to_the_decisively_nearer_tower() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // Gibraltar-ish and Malaga-ish, ~50 nm apart along a degree of longitude.
    *REGION.lock().unwrap() = vec![
        region_apt("Gibraltar", 36.15, -5.35),
        region_apt("Malaga", 36.67, -4.50),
    ];
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    // The fresh-boot branch reads the contact state: start from none,
    // whatever an earlier test left behind.
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    let t0 = Instant::now();
    // On the ground at Gibraltar (the active field): never.
    assert!(region_tick(36.15, -5.35, true, t0).is_none());
    // Just airborne, Gibraltar still much nearer: quiet.
    assert!(region_tick(36.2, -5.3, false, t0).is_none());
    // Near the midpoint, Malaga slightly nearer but inside the margin:
    // still quiet — hysteresis, no flapping between two fields.
    assert!(region_tick(36.42, -4.92, false, t0).is_none());
    // On the GROUND near the midpoint (not AT any field): also quiet —
    // a ground relocation needs you physically at the new field.
    assert!(region_tick(36.42, -4.92, true, t0).is_none());
    // Decisively closer to Malaga, airborne — but NOBODY here has talked
    // to Gibraltar this session (fresh boot mid-flight): the crew
    // orients, no controller terminating a service it never gave.
    let sw = region_tick(36.6, -4.6, false, t0).expect("fresh-boot fix");
    assert!(sw.crew, "no contact yet: the crew speaks, not a controller");
    assert!(sw.line.contains("Closest field is Malaga"), "{}", sw.line);
    assert!(!sw.line.contains("Radar services"), "{}", sw.line);
    assert!(
        PENDING_STATION.lock().unwrap().is_some(),
        "swap armed for the main loop"
    );
    // While the swap waits to be consumed: silence, no re-arm.
    assert!(region_tick(36.6, -4.6, false, t0).is_none());
    // Same geometry WITH two-way contact on record: the controller's
    // proper handoff.
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "November one seven two Sierra Papa".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: false,
    });
    let _ = crate::party_line::take_region_handoff();
    let sw = region_tick(36.6, -4.6, false, t0).expect("handoff");
    assert!(
        !sw.crew,
        "with a service on record the controller hands off"
    );
    assert!(sw.line.contains("Contact Malaga Tower"), "{}", sw.line);
    assert!(sw.line.contains("one one eight decimal one"), "{}", sw.line);
    // Tower contact only, no radar service: no code to give back, and
    // the crew gets the instruction as data, once.
    assert!(!sw.line.contains("seven thousand"), "{}", sw.line);
    let h = crate::party_line::take_region_handoff().expect("published for the crew");
    assert_eq!(
        h,
        RegionHandoff {
            to: "Malaga Tower".into(),
            khz: 118_100,
            squawk_vfr: false,
            ifr_code: None,
        }
    );
    assert!(
        crate::party_line::take_region_handoff().is_none(),
        "consumed once"
    );
    // Same again with a radar service on record: the discrete code goes
    // back to conspicuity, spoken AND handed down.
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
    *OWN_STRIP.squawk_watch.lock().unwrap() = Some(SquawkWatch {
        code: 4503,
        callsign: "November one seven two Sierra Papa".into(),
        speaker: StationSpeaker::Departure,
        observed: true,
        check_in_owed: false,
        first_seen: None,
        last_nudge: None,
        nudges: 0,
    });
    let sw = region_tick(36.6, -4.6, false, t0).expect("handoff");
    assert!(sw.line.contains("Squawk seven thousand"), "{}", sw.line);
    let h = crate::party_line::take_region_handoff().expect("published for the crew");
    assert!(h.squawk_vfr, "{h:?}");
    assert_eq!(h.khz, 118_100);
    assert_eq!(h.ifr_code, None, "a VFR service ending");
    // The same radar service, but the flight holds an accepted IFR
    // clearance (LIEA-LEBL, 2026-09-27): handed on with its code, no
    // service terminated, no seven thousand.
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
    publish_ifr_clearance(
        &OWN_STRIP,
        tower::IfrClearance {
            dest: "LEMG".into(),
            sid: None,
            runway: "09".into(),
            initial_alt_ft: 5000,
            squawk: 4503,
        },
        "November eight one six November November",
        None,
        None,
    );
    // Malaga here publishes only a tower: an IFR flight in the air is
    // never handed to one (EGBB-EGKK 2026-09-27, "contact Oxford
    // Kidlington Tower" at FL150). The last controller keeps it.
    assert!(region_tick(36.6, -4.6, false, t0).is_none(), "no tower-only handoff for IFR");
    assert!(crate::party_line::take_region_handoff().is_none());
    // With an approach position the field takes the flight on radar: a
    // plain handoff to Malaga APPROACH, the code kept.
    REGION.lock().unwrap()[1].apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_APPROACH,
        khz: 123_950,
        name: "Malaga Approach".into(),
    });
    let sw = region_tick(36.6, -4.6, false, t0).expect("IFR handoff");
    assert!(!sw.crew);
    assert_eq!(
        sw.line,
        "November eight one six November November, contact Malaga Approach on one two three decimal niner five."
    );
    let h = crate::party_line::take_region_handoff().expect("published for the crew");
    assert_eq!(
        h,
        RegionHandoff {
            to: "Malaga Approach".into(),
            khz: 123_950,
            squawk_vfr: false,
            ifr_code: Some(4503),
        }
    );
    // The pinned arrival keeps its tower (the inbound call goes there),
    // tower-only or not.
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.15, -5.35));
    REGION.lock().unwrap()[1].apt.freqs.pop();
    *ARRIVAL_PIN.lock().unwrap() = Some(118_100);
    let sw = region_tick(36.6, -4.6, false, t0).expect("pinned arrival handoff");
    assert!(sw.line.contains("contact Malaga Tower on one one eight decimal one"), "{}", sw.line);
    let h = crate::party_line::take_region_handoff().expect("published for the crew");
    assert_eq!((h.to.as_str(), h.khz), ("Malaga Tower", 118_100));
    *ARRIVAL_PIN.lock().unwrap() = None;
    // The IFR flight ends at the landing, not before: on the ground at
    // the clearance it stands, airborne it stands, the next ground
    // contact retires it.
    note_aircraft(36.15, -5.35, true);
    assert_eq!(crate::party_line::ifr_flight_code(), Some(4503));
    note_aircraft(36.4, -5.0, false);
    assert_eq!(crate::party_line::ifr_flight_code(), Some(4503));
    note_aircraft(36.15, -5.35, true);
    assert_eq!(crate::party_line::ifr_flight_code(), None, "landed");
    OUT_OF_REACH.store(false, std::sync::atomic::Ordering::Relaxed);
    *OWN_STRIP.ifr_clearance.lock().unwrap() = None;
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    // Ground relocation: parked AT Malaga with Gibraltar still active
    // (the "started at LERL, Gibraltar answered" case) — the CREW notes
    // the local tower, no radar theatre.
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    let sw = region_tick(36.67, -4.50, true, t0).expect("ground relocation");
    assert!(sw.crew, "ground switch is the crew's note");
    assert!(sw.line.contains("We're at Malaga"), "{}", sw.line);
    run_borrowing_strip_case(); // same SIDE_STATE_LOCK scope, statics owned here
    run_adjacent_fields_case();
    *REGION.lock().unwrap() = Vec::new();
    *ACTIVE_FIELD.lock().unwrap() = None;
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
}

/// The Trebujena trap (flown 2026-07-11), run inside the region test's
/// lock scope: a tower-only grass strip 12 nm off the nose whose apt.dat
/// rows borrow "Jerez Tower" / "Seville Approach", with the actual
/// destination (ATIS, ground, its own approach) 16 nm out. Raw-distance
/// nearest picked the strip, and its controller then vectored the Jerez
/// arrival onto Trebujena's 800 m runway two six.
fn run_borrowing_strip_case() {
    let mut jerez = region_apt("Jerez", 36.7446, -6.0601);
    jerez.apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_ATIS,
        khz: 125655,
        name: "ATIS".into(),
    });
    jerez.apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_GROUND,
        khz: 133280,
        name: "GND".into(),
    });
    jerez.apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_APPROACH,
        khz: 128500,
        name: "Seville Approach".into(),
    });
    *REGION.lock().unwrap() = vec![region_apt("Trebujena", 36.8587, -6.1413), jerez];
    *ACTIVE_FIELD.lock().unwrap() = Some(("Sevilla San Pablo".into(), 37.418, -5.893));
    *PENDING_STATION.lock().unwrap() = None;
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    // Airborne between the two, at the flight's real 13:09 position:
    // the REAL airport wins the handoff despite being farther.
    let sw = region_tick(36.98726, -5.94432, false, Instant::now()).expect("switch");
    assert!(sw.line.contains("Jerez"), "{}", sw.line);
    let pending = PENDING_STATION.lock().unwrap().take().expect("swap armed");
    assert_eq!(
        pending.1.name, "Jerez",
        "vector the arrival to the destination"
    );
    // Parked AT the strip, though: you're at the field you're at — the
    // ground relocation stays raw-nearest, however small the strip.
    *ACTIVE_FIELD.lock().unwrap() = Some(("Jerez".into(), 36.7446, -6.0601));
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    let sw = region_tick(36.8587, -6.1413, true, Instant::now()).expect("ground relocation");
    assert!(sw.line.contains("Trebujena"), "{}", sw.line);
}

#[test]
fn runway_occupancy_is_a_tight_rectangle() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // LXGB 09/27, real-ish geometry: ~1.7 km east-west at 36.151N.
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1512,
        lon_a: -5.3554,
        lat_b: 36.1517,
        lon_b: -5.3366,
    }];
    // On the centerline past midfield: on the runway, named by the nearer end.
    assert_eq!(on_active_runway(36.15155, -5.342).as_deref(), Some("27"));
    // Near the 09 threshold: still on, named 09.
    assert_eq!(on_active_runway(36.1512, -5.3550).as_deref(), Some("09"));
    // 100 m abeam the centerline (a parallel taxiway): NOT on the runway.
    assert!(on_active_runway(36.1503, -5.346).is_none());
    // A mile away at the parking: no.
    assert!(on_active_runway(36.16, -5.35).is_none());
    // Past the far threshold by more than the overrun margin: no.
    assert!(on_active_runway(36.152, -5.330).is_none());
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

#[test]
fn information_letters_parse_from_calls() {
    assert_eq!(heard_information("with information alpha"), Some('A'));
    assert_eq!(
        heard_information("we have information bravo, request taxi"),
        Some('B')
    );
    // "x-ray" splits at the hyphen; the bare "x" carries the letter anyway.
    assert_eq!(heard_information("information x-ray"), Some('X'));
    assert_eq!(heard_information("information b"), Some('B'));
    assert_eq!(heard_information("request taxi"), None);
    assert_eq!(
        heard_information("for your information the flaps are stuck"),
        None
    );
}

#[test]
fn having_the_current_information_skips_the_qnh() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ATIS.lock().unwrap() = Some(AtisState {
        khz: 118500,
        airport: "Gibraltar".into(),
        runways: Vec::new(),
        letter: 'A',
        dialect: Dialect::Icao,
        edition: None,
        last_spoken: None,
    });
    let _ = atis_tick(118500, &wx(250.0, 12.0, 1018.0), Instant::now()); // airs Alpha
    let w = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    // With the current letter: the taxi clearance drops the QNH clause.
    let mut st = lxgb();
    let ctx = {
        st.note_information("gibraltar tower, with information alpha, request taxi");
        st.make_ctx(
            w,
            "QNH one zero one eight".into(),
            "November".into(),
            true,
            false,
            None,
            false,
            None,
        )
    };
    let reply = st.desk.twr.handle(tower::parse_intent("request taxi"), &ctx);
    assert!(!reply.contains("QNH"), "{reply}");
    // A STALE letter keeps the numbers — safety over brevity.
    let mut st = lxgb();
    st.note_information("with information zulu, request taxi");
    let ctx = st.make_ctx(
        w,
        "QNH one zero one eight".into(),
        "November".into(),
        true,
        false,
        None,
        false,
        None,
    );
    let reply = st.desk.twr.handle(tower::parse_intent("request taxi"), &ctx);
    assert!(reply.contains("QNH one zero one eight"), "{reply}");
    // No letter at all: numbers as always.
    let mut st = lxgb();
    let ctx = st.make_ctx(
        w,
        "QNH one zero one eight".into(),
        "November".into(),
        true,
        false,
        None,
        false,
        None,
    );
    let reply = st.desk.twr.handle(tower::parse_intent("request taxi"), &ctx);
    assert!(reply.contains("QNH one zero one eight"), "{reply}");
    *ATIS.lock().unwrap() = None;
}

#[test]
fn unkeyed_routing_needs_a_radio_shaped_call() {
    // Crew talk that merely contains intent keywords must fall through —
    // "anything else we need to do before takeoff?" was answered with a
    // takeoff clearance on 2026-07-08, and the panel's "land the aircraft"
    // button got a join clearance.
    for crew in [
        "anything else we need to do before takeoff",
        "land the aircraft",
        "flaps set for takeoff",
        "after landing checklist",
    ] {
        assert!(!is_radio_call(crew, "gibraltar"), "{crew}");
    }
    // Addressed or call-vocabulary utterances still route.
    for call in [
        "hello tower, i would like clearance for takeoff",
        "gibraltar, request taxi",
        "request touch and go",
        "inbound for landing",
        "november one seven two, final",
        "radio check",
    ] {
        assert!(is_radio_call(call, "gibraltar"), "{call}");
    }
}

/// LXGB at 07001KT (2026-09-15): the tower picked 27 while LiveTraffic
/// was landing 09, and cleared a departure into the reciprocal stream.
/// In calm wind the end the arrivals are landing on is the runway in
/// use; a real wind still outranks it, and a held pick is not moved.
#[test]
fn calm_wind_follows_the_arrival_stream() {
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    assert_eq!(st.pick_runway_with(70.0, 1.0, Some("09"), None), "09");
    // Held once we are sequenced: a taxi clearance owns its runway.
    st.desk.twr.state = tower::State::TaxiCleared;
    assert_eq!(st.pick_runway_with(70.0, 1.0, Some("27"), None), "09");
    // Uncommitted, a stream that turned round turns the calm pick with
    // it: one strip, one direction (LEMG 2026-09-28, issue #140).
    st.desk.twr.state = tower::State::Parked;
    assert_eq!(st.pick_runway_with(70.0, 1.0, Some("27"), None), "27");
    // A real wind is not overruled by traffic.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    assert_eq!(st.pick_runway_with(250.0, 12.0, Some("09"), None), "27");
    // An end this field does not have is ignored.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    assert_eq!(st.pick_runway_with(70.0, 1.0, Some("18"), None), st.pick_runway(70.0, 1.0));
}

#[test]
fn the_arrival_end_is_the_threshold_the_traffic_is_beyond() {
    let runways = vec![nav::Runway {
        width_m: 45.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1512,
        lon_a: -5.3554,
        lat_b: 36.1517,
        lon_b: -5.3366,
    }];
    let target = |lon: f64, vs: f64, ground: bool| TcasTarget {
        callsign: "HTY103".into(),
        lat: 36.1512,
        lon,
        ele_ft: 700.0,
        on_ground: ground,
        vs_fpm: vs,
    };
    // West of the 09 threshold, descending: landing on 09.
    assert_eq!(arrival_end(&[target(-5.39, -600.0, false)], &runways, 11.0).as_deref(), Some("09"));
    // East of the 27 threshold: landing on 27.
    assert_eq!(arrival_end(&[target(-5.30, -600.0, false)], &runways, 11.0).as_deref(), Some("27"));
    // Climbing away, or on the ground: not an arrival.
    assert_eq!(arrival_end(&[target(-5.39, 800.0, false)], &runways, 11.0), None);
    assert_eq!(arrival_end(&[target(-5.39, -600.0, true)], &runways, 11.0), None);
    // The nearer of two wins.
    let both = [target(-5.39, -600.0, false), target(-5.33, -500.0, false)];
    assert_eq!(arrival_end(&both, &runways, 11.0).as_deref(), Some("27"));
}

/// Issue #155, sortie 1790601822-full_auto (EGKK-EGBB, 2026-09-28): the
/// check-in at +1209.7 "passing four thousand seven hundred climbing
/// flight level two two zero" earned "radar contact, climb flight level
/// two two zero" on a plan filed at FL180 (`initial_altitude` 18000).
/// Departure clears no higher than the filed cruise; at or under it, or
/// with nothing filed, the announced climb stands.
#[test]
fn departure_clears_no_higher_than_the_filed_cruise() {
    let heard = "london gatwick departure, whiskey alpha tango six two six, passing four \
                     thousand seven hundred climbing flight level two two zero";
    let announced = climb_announced(heard).expect("a climb");
    assert_eq!(announced, "flight level two two zero");
    // EGKK's transition level unknown to the test: the crew's own form.
    assert_eq!(
        capped_departure_climb(&announced, Some(18_000), None),
        "flight level one eight zero"
    );
    assert_eq!(
        capped_departure_climb(&announced, Some(18_000), Some(7_000.0)),
        "flight level one eight zero"
    );
    assert_eq!(capped_departure_climb(&announced, None, None), announced);
    assert_eq!(capped_departure_climb(&announced, Some(24_000), None), announced);
    assert_eq!(capped_departure_climb("five thousand", Some(18_000), None), "five thousand");
    // Below the transition level a capped climb is feet.
    assert_eq!(
        capped_departure_climb("seven thousand", Some(4_000), Some(6_000.0)),
        crate::atis::spoken_alt(4_000.0)
    );
}

#[test]
fn the_climb_announced_is_what_departure_clears() {
    assert_eq!(
        climb_announced("gibraltar departure, november one seven two sierra papa, passing one thousand climbing five thousand"),
        Some("five thousand".into())
    );
    assert_eq!(climb_announced("passing two thousand climbing to 5000."), Some("5000".into()));
    assert_eq!(climb_announced("passing one thousand"), None);
    assert_eq!(climb_announced("climbing"), None);
}

#[test]
fn calm_wind_holds_the_runway_in_use() {
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    // A real westerly picks 27.
    assert_eq!(st.pick_runway(250.0, 12.0), "27");
    // Calm air with the direction flopping east must NOT flip the runway —
    // the 2026-07-08 flight was cleared on 27, then re-cleared on 09 two
    // minutes later in the same calm.
    assert_eq!(st.pick_runway(90.0, 1.0), "27");
    assert_eq!(st.pick_runway(170.0, 0.0), "27");
    // Actual wind changes the runway like a real controller would.
    assert_eq!(st.pick_runway(90.0, 10.0), "09");
}

#[test]
fn a_handoff_readback_after_the_swap_is_not_a_void_call() {
    let _g = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let now = Instant::now();
    let readback = "november eight one six november november, squawking seven thousand, carlisle tower on one two three decimal six zero five";
    *LAST_REGION_SWITCH.lock().unwrap() = Some(now);
    assert_eq!(void_outcome(readback, now + std::time::Duration::from_secs(4)), "handoff_readback");
    // A real call to nobody is still a void call...
    assert_eq!(void_outcome("exeter tower, inbound for landing", now), "void");
    // ...and so is a readback long after any handoff.
    assert_eq!(void_outcome(readback, now + std::time::Duration::from_secs(300)), "void");
    *LAST_REGION_SWITCH.lock().unwrap() = None;
    assert_eq!(void_outcome(readback, now), "void");
}

/// EGTE, 2026-09-21. "Final runway two six" into a wind dead across the
/// strip earned "negative, runway zero eight" and a trip round the field;
/// then the go-around (ClearedToLand back to Inbound) flipped the pick to
/// 26 and the crew's missed approach was re-aimed from 60 ft.
#[test]
fn an_arrival_keeps_its_runway_on_a_wind_that_favours_neither_end() {
    let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
    // Nothing in use yet, wind across (LXGB is 09/27): the end the
    // arrival is lined up on stands.
    st.note_stated_runway("november one two three, final runway two seven", tower::Intent::ReportFinal);
    assert_eq!(st.pick_runway(0.0, 8.0), "27");
    // A wind that really favours the other end still wins.
    let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.note_stated_runway("final runway two seven", tower::Intent::ReportFinal);
    assert_eq!(st.pick_runway(90.0, 12.0), "09");
    // A departure ready to go does not choose its runway...
    let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.note_stated_runway("ready for departure runway two seven", tower::Intent::RequestTakeoff);
    assert_eq!(st.desk.call.stated_runway, None);
    // ...but asks for one at the taxi request, honoured when the wind
    // does not argue (LOWI, calm, MOGT3H only from 26).
    assert_eq!(
        tower::parse_intent("november two five one charlie victor, request taxi, runway two six"),
        tower::Intent::RequestTaxi
    );
    let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.note_stated_runway("request taxi, runway two seven", tower::Intent::RequestTaxi);
    assert_eq!(st.pick_runway(0.0, 2.0), "27");
    let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.note_stated_runway("request taxi, runway two seven", tower::Intent::RequestTaxi);
    assert_eq!(st.pick_runway(90.0, 12.0), "09", "a real headwind on the other end wins");
    // The crew's inbound call names the runway it set up for (LIPK
    // 2026-09-25: "inbound for landing" alone, then "join final runway
    // one two" to a crew flying the ILS 30).
    let inbound = "november one seven two sierra papa, inbound for landing, runway two seven";
    assert!(matches!(tower::parse_intent(inbound), tower::Intent::RequestLanding { .. }));
    let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.note_stated_runway(inbound, tower::parse_intent(inbound));
    assert_eq!(st.pick_runway(0.0, 4.0), "27");
    // And once an arrival is on a runway the pick is frozen, through the
    // go-around's Inbound, whatever the wind does.
    let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.runway_in_use = Some("09".into());
    for state in [tower::State::ClearedToLand, tower::State::Inbound] {
        st.desk.twr.state = state;
        assert_eq!(st.pick_runway(270.0, 15.0), "09", "{state:?}");
    }
}

/// LEMG 12/30 and 13/31 from apt.dat.
fn lemg_runways() -> Vec<nav::Runway> {
    vec![
        nav::Runway {
            width_m: 45.11,
            surface: 1,
            ident_a: "12".into(),
            ident_b: "30".into(),
            lat_a: 36.6910753,
            lon_a: -4.5078270,
            lat_b: 36.6795648,
            lon_b: -4.4805125,
        },
        nav::Runway {
            width_m: 45.11,
            surface: 1,
            ident_a: "13".into(),
            ident_b: "31".into(),
            lat_a: 36.6845573,
            lon_a: -4.5126253,
            lat_b: 36.6654027,
            lon_b: -4.4858140,
        },
    ]
}

/// Issue #140, the recorded runway in use: LEMG (sortie
/// 1790584715-full_auto, 2026-09-28), wind calm. 31 stood in use from an
/// earlier into-wind pick while the replay landed 12 (Ryanair 8168,
/// t 1790585296.6) and departed 13 (Vueling 8176 t 1790585447.7, Easy
/// 2716 t 1790585551.4). A calm, uncommitted pick turns to the live
/// stream; a committed one holds; a stated runway into the departures is
/// refused; a real wind still wins.
#[test]
fn the_calm_runway_in_use_turns_to_the_live_stream() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let runways = lemg_runways();
    assert!(ends_oppose(&runways, "31", "13"), "the reciprocal");
    assert!(ends_oppose(&runways, "31", "12"), "the near-parallel's other end");
    assert!(!ends_oppose(&runways, "13", "12"), "one direction");
    let fresh = |in_use: &str| {
        let mut st = lxgb();
        st.apt.name = "Malaga".into();
        st.apt.runways = runways.clone();
        st.runway_in_use = Some(in_use.into());
        st
    };
    // The departures alone turn 31 round to 13.
    let mut st = fresh("31");
    assert_eq!(st.pick_runway_with(350.0, 3.0, None, Some("13")), "13");
    // The arrivals first: 12, the field's landing runway, same direction.
    let mut st = fresh("31");
    assert_eq!(st.pick_runway_with(350.0, 3.0, Some("12"), Some("13")), "12");
    // A pick already with the stream is left alone (13 is not against 12).
    let mut st = fresh("13");
    assert_eq!(st.pick_runway_with(350.0, 3.0, Some("12"), Some("13")), "13");
    // Committed: the circuit's Inbound and a taxi clearance hold 31.
    for state in [tower::State::Inbound, tower::State::TaxiCleared] {
        let mut st = fresh("31");
        st.desk.twr.state = state;
        assert_eq!(st.pick_runway_with(350.0, 3.0, None, Some("13")), "31", "{state:?}");
    }
    // "Downwind runway three one" into the 13 departures, uncommitted:
    // refused, the stream's end instead.
    let mut st = fresh("31");
    st.note_stated_runway("downwind runway three one", tower::Intent::ReportDownwind);
    assert_eq!(st.pick_runway_with(350.0, 3.0, None, Some("13")), "13");
    // With the departures on 31 it stands.
    let mut st = fresh("13");
    st.note_stated_runway("downwind runway three one", tower::Intent::ReportDownwind);
    assert_eq!(st.pick_runway_with(350.0, 3.0, None, Some("31")), "31");
    // A real wind is not overruled by the traffic (310 at 15: 31).
    let mut st = fresh("31");
    assert_eq!(st.pick_runway_with(310.0, 15.0, None, Some("13")), "31");
    // No stream known: a calm pick stands as before.
    let mut st = fresh("31");
    assert_eq!(st.pick_runway_with(350.0, 3.0, None, None), "31");
}

/// Issue #140: the departure stream is the END a ghost rolled from, by
/// its roll direction, remembered for DEPARTURE_STREAM_SECS by the wake
/// watch (which runs whether or not the party line is on).
#[test]
fn a_ghost_liftoff_names_the_departure_stream() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let runways = lemg_runways();
    let (a, b) = ((runways[1].lat_a, runways[1].lon_a), (runways[1].lat_b, runways[1].lon_b));
    let along = |f: f64| (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f);
    *ACTIVE_RUNWAYS.lock().unwrap() = runways.clone();
    let field = along(0.5);
    *ACTIVE_FIELD.lock().unwrap() = Some(("Malaga".into(), field.0, field.1));
    *WAKE.lock().unwrap() = None;
    *WAKE_PREV.lock().unwrap() = None;
    *DEPARTURE_STREAM.lock().unwrap() = None;
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    let at = |f: f64, ground: bool| {
        let (lat, lon) = along(f);
        TcasTarget {
            callsign: "VLG8176".into(),
            lat,
            lon,
            ele_ft: if ground { 52.0 } else { 160.0 },
            on_ground: ground,
            vs_fpm: if ground { 0.0 } else { 1500.0 },
        }
    };
    wake_note_targets(&[at(0.70, true)], t0);
    assert!(departure_end(t0).is_none(), "rolling: no stream yet");
    wake_note_targets(&[at(0.78, false)], t0 + sec(3));
    // Lifted off past midfield, nearer the 31 threshold: rolled on 13.
    assert_eq!(departure_end(t0 + sec(4)).as_deref(), Some("13"));
    assert!(departure_end(t0 + sec(3 + DEPARTURE_STREAM_SECS)).is_none(), "stale");

    *WAKE.lock().unwrap() = None;
    *WAKE_PREV.lock().unwrap() = None;
    *DEPARTURE_STREAM.lock().unwrap() = None;
    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

/// Issue #164, the recorded LEMG 30 final (sortie 1790610218-approach,
/// 2026-09-28): the replay landed 12 (Ryanair 431 t 1790610003, Ryanair
/// 590 t 1790610134) and, with no lander on final at that instant, "final
/// runway three zero" in 300 at 4 was cleared to land 30 (t 1790610223),
/// head-on to the stream, and sent round twice. A ghost's touchdown is
/// remembered for ARRIVAL_STREAM_SECS: the stated 30 is refused for 12
/// while it is, and stands once the memory lapses. The departure memory
/// keeps working alongside, and a newer stream that points against the
/// older one means the field has turned round.
#[test]
fn a_ghost_touchdown_remembers_the_arrival_stream() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let runways = lemg_runways();
    let lerp = |p: (f64, f64), q: (f64, f64), f: f64| {
        (p.0 + (q.0 - p.0) * f, p.1 + (q.1 - p.1) * f)
    };
    let (t12, t30) = ((runways[0].lat_a, runways[0].lon_a), (runways[0].lat_b, runways[0].lon_b));
    let (t13, t31) = ((runways[1].lat_a, runways[1].lon_a), (runways[1].lat_b, runways[1].lon_b));
    *ACTIVE_RUNWAYS.lock().unwrap() = runways.clone();
    let field = lerp(t12, t30, 0.5);
    *ACTIVE_FIELD.lock().unwrap() = Some(("Malaga".into(), field.0, field.1));
    *WAKE.lock().unwrap() = None;
    *WAKE_PREV.lock().unwrap() = None;
    *DEPARTURE_STREAM.lock().unwrap() = None;
    *ARRIVAL_STREAM.lock().unwrap() = None;
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    let ghost = |id: &str, p: (f64, f64), ground: bool, vs: f64| TcasTarget {
        callsign: id.into(),
        lat: p.0,
        lon: p.1,
        ele_ft: if ground { 52.0 } else { 90.0 },
        on_ground: ground,
        vs_fpm: vs,
    };
    // Ryanair 590 over the 12 threshold, then on the pavement: landed 12.
    wake_note_targets(&[ghost("RYR590", lerp(t12, t30, 0.0), false, -600.0)], t0);
    assert!(arrival_stream_end(t0).is_none(), "airborne: no stream yet");
    wake_note_targets(&[ghost("RYR590", lerp(t12, t30, 0.08), true, 0.0)], t0 + sec(2));
    assert_eq!(arrival_stream_end(t0 + sec(3)).as_deref(), Some("12"));
    // A ghost settling off every strip (a taxiway, a pad) says nothing.
    let off = (field.0 + 0.01, field.1);
    wake_note_targets(&[ghost("GA1", off, false, -300.0)], t0 + sec(4));
    wake_note_targets(&[ghost("GA1", (off.0 + 0.0005, off.1), true, 0.0)], t0 + sec(5));
    assert_eq!(arrival_stream_end(t0 + sec(6)).as_deref(), Some("12"));

    let fresh = || {
        let mut st = lxgb();
        st.apt.name = "Malaga".into();
        st.apt.runways = runways.clone();
        st.runway_in_use = None;
        st
    };
    // 89 s later, no lander on final: the stream is still 12, and the
    // SR22's stated 30 in calm is refused for it.
    let now = t0 + sec(89);
    let (arr, dep) = field_streams(&runways, None, now);
    assert_eq!((arr.as_deref(), dep.as_deref()), (Some("12"), None));
    let mut st = fresh();
    st.note_stated_runway("final runway three zero", tower::Intent::ReportFinal);
    assert_eq!(st.pick_runway_with(300.0, 4.0, arr.as_deref(), dep.as_deref()), "12");
    // The memory lapsed: no stream, the stated 30 stands in calm.
    let later = t0 + sec(2 + ARRIVAL_STREAM_SECS);
    assert!(arrival_stream_end(later).is_none(), "stale");
    let (arr, dep) = field_streams(&runways, None, later);
    assert_eq!((arr.as_deref(), dep.as_deref()), (None, None));
    let mut st = fresh();
    st.note_stated_runway("final runway three zero", tower::Intent::ReportFinal);
    assert_eq!(st.pick_runway_with(300.0, 4.0, arr.as_deref(), dep.as_deref()), "30");

    // The departure memory keeps working beside it: a 13 liftoff (same
    // direction as 12) is kept with the landings.
    let dep_t = t0 + sec(20);
    wake_note_targets(&[ghost("VLG8176", lerp(t13, t31, 0.70), true, 0.0)], dep_t);
    wake_note_targets(&[ghost("VLG8176", lerp(t13, t31, 0.78), false, 1500.0)], dep_t + sec(3));
    assert_eq!(departure_end(dep_t + sec(4)).as_deref(), Some("13"));
    let (arr, dep) = field_streams(&runways, None, dep_t + sec(4));
    assert_eq!((arr.as_deref(), dep.as_deref()), (Some("12"), Some("13")));
    // A newer departure off 31 (against the 12 landings): the field has
    // turned round, the older landing memory is dropped...
    let turn_t = t0 + sec(60);
    wake_note_targets(&[ghost("EZY2716", lerp(t31, t13, 0.70), true, 0.0)], turn_t);
    wake_note_targets(&[ghost("EZY2716", lerp(t31, t13, 0.78), false, 1500.0)], turn_t + sec(3));
    let (arr, dep) = field_streams(&runways, None, turn_t + sec(4));
    assert_eq!((arr.as_deref(), dep.as_deref()), (None, Some("31")));
    // ...and a live lander on 12 outranks that older departure.
    let (arr, dep) = field_streams(&runways, Some("12".into()), turn_t + sec(10));
    assert_eq!((arr.as_deref(), dep.as_deref()), (Some("12"), None));

    *WAKE.lock().unwrap() = None;
    *WAKE_PREV.lock().unwrap() = None;
    *DEPARTURE_STREAM.lock().unwrap() = None;
    *ARRIVAL_STREAM.lock().unwrap() = None;
    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

/// Issue #140, the recorded party line: LEMG (sortie
/// 1790584715-full_auto). Our circuit on downwind for 31 (Inbound) heard
/// "Vueling eight one seven six, runway one three, cleared for takeoff"
/// (t 1790585447.7); cleared to land 31 (t 1790585501.7), the tower
/// ordered the go-around for traffic on the runway (t 1790585525.5) and
/// then narrated "Easy two seven one six, runway one three, cleared for
/// takeoff" (t 1790585551.4). One strip, one direction: nobody is
/// cleared onto or off the reciprocal of our runway, and nobody onto
/// either end of the strip we are cleared to land on. The other strip's
/// traffic and our own direction's are still narrated.
#[test]
fn the_party_line_clears_nobody_against_our_runway() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let runways = lemg_runways();
    *ACTIVE_RUNWAYS.lock().unwrap() = runways.clone();
    *GHOSTS.lock().unwrap() = None;
    let lerp = |p: (f64, f64), q: (f64, f64), f: f64| {
        (p.0 + (q.0 - p.0) * f, p.1 + (q.1 - p.1) * f)
    };
    let (t13, t31) = ((runways[1].lat_a, runways[1].lon_a), (runways[1].lat_b, runways[1].lon_b));
    let (t12, t30) = ((runways[0].lat_a, runways[0].lon_a), (runways[0].lat_b, runways[0].lon_b));
    let field = lerp(t13, t31, 0.5);
    let felev = 52.0;
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    let set_clearance = |state: tower::State, holds: bool| {
        *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
            state,
            callsign: "November eight one six November November".into(),
            runway_ident: "31".into(),
            pending_readback: None,
            holds_runway: holds,
        });
    };
    let ghost = |cs: &str, pos: (f64, f64), ele: f64, ground: bool, vs: f64| TcasTarget {
        callsign: cs.into(),
        lat: pos.0,
        lon: pos.1,
        ele_ft: ele,
        on_ground: ground,
        vs_fpm: vs,
    };
    // Airborne: the ground-only blocked strip is None, as in the sortie.
    let tick = |t: TcasTarget, at: Instant| {
        chatter_tick(&[t], "Malaga", field.0, field.1, felev, None, at)
    };
    // A departure: rolling at 70 % of the strip, airborne at 78 %.
    let depart = |cs: &str, from: (f64, f64), to: (f64, f64), at: Instant| {
        assert!(tick(ghost(cs, lerp(from, to, 0.70), felev, true, 0.0), at).is_none());
        tick(ghost(cs, lerp(from, to, 0.78), felev + 120.0, false, 1500.0), at + sec(3))
    };

    // Downwind for 31 (Inbound): Vueling off 13 is not cleared.
    set_clearance(tower::State::Inbound, false);
    assert!(
        depart("VLG8176", t13, t31, t0).is_none(),
        "no takeoff narrated off the reciprocal of our runway"
    );
    // Cleared to land 31: Easy off 13 is not cleared either.
    set_clearance(tower::State::ClearedToLand, true);
    assert!(depart("EZY2716", t13, t31, t0 + sec(10)).is_none());
    // After the go-around order (the FSM back to Inbound): still not.
    set_clearance(tower::State::Inbound, false);
    assert!(depart("EZS2716", t13, t31, t0 + sec(20)).is_none());
    // Cleared to land 31, a departure off 31 ahead of us: the whole
    // strip is ours, not narrated.
    set_clearance(tower::State::ClearedToLand, true);
    assert!(depart("IBE3001", t31, t13, t0 + sec(30)).is_none());
    // Inbound for 31, a departure off 31: our direction, narrated.
    set_clearance(tower::State::Inbound, false);
    let ex = depart("IBE3002", t31, t13, t0 + sec(40)).expect("our direction is narrated");
    assert!(ex.lines[0].text.contains("runway three one, cleared for takeoff"), "{}", ex.lines[0].text);
    // The other strip is not ours: a departure off 12 is narrated.
    let ex = depart("IBE3003", t12, t30, t0 + sec(50)).expect("the other strip is narrated");
    assert!(ex.lines[0].text.contains("runway one two, cleared for takeoff"), "{}", ex.lines[0].text);

    // Landings. Cleared to land 31: Ryanair on final 12 is narrated,
    // a ghost on final 31 behind us is not given the same clearance.
    set_clearance(tower::State::ClearedToLand, true);
    let final12 = |f: f64| lerp(t12, t30, f);
    assert!(tick(ghost("RYR8168", final12(-0.8), 900.0, false, -700.0), t0 + sec(60)).is_none());
    let ex = tick(ghost("RYR8168", final12(-0.7), 750.0, false, -700.0), t0 + sec(63))
        .expect("the other strip's landing is narrated");
    assert!(ex.lines[1].text.contains("runway one two, cleared to land"), "{}", ex.lines[1].text);
    let final31 = |f: f64| lerp(t31, t13, f);
    assert!(tick(ghost("RYR4458", final31(-0.6), 900.0, false, -700.0), t0 + sec(70)).is_none());
    assert!(
        tick(ghost("RYR4458", final31(-0.5), 750.0, false, -700.0), t0 + sec(73)).is_none(),
        "no second landing clearance on the strip we are cleared to land on"
    );
    // Our landing done and reported off: the claim is gone and the
    // ghost, still descending, gets its clearance.
    set_clearance(tower::State::ClearedToLand, false);
    let ex = tick(ghost("RYR4458", final31(-0.4), 600.0, false, -700.0), t0 + sec(76))
        .expect("cleared once the strip is free");
    assert!(ex.lines[1].text.contains("runway three one, cleared to land"), "{}", ex.lines[1].text);

    *GHOSTS.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

/// Issue #129, the recorded case: LEMG (sortie 1790580676-approach,
/// 2026-09-28), the 737 on a 10 nm final for 13 in 350 at 3 kt, said
/// "ten mile final runway one three" and at t 1790580681.3 was told
/// "negative, runway three one ... cleared to land": 13's -2.3 kt
/// trailed 31's +2.3 kt by 4.7, over the 3 kt swap margin, in a wind the
/// pick itself calls calm. LEMG 12/30 and 13/31 from apt.dat.
#[test]
fn the_lemg_final_for_one_three_keeps_its_runway_in_a_calm() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let runways = vec![
        nav::Runway {
            width_m: 45.11,
            surface: 1,
            ident_a: "12".into(),
            ident_b: "30".into(),
            lat_a: 36.6910753,
            lon_a: -4.5078270,
            lat_b: 36.6795648,
            lon_b: -4.4805125,
        },
        nav::Runway {
            width_m: 45.11,
            surface: 1,
            ident_a: "13".into(),
            ident_b: "31".into(),
            lat_a: 36.6845573,
            lon_a: -4.5126253,
            lat_b: 36.6654027,
            lon_b: -4.4858140,
        },
    ];
    let wx = Wx {
        wind_from_deg: 350.0,
        wind_kts: 3.0,
    };
    // The recorded numbers: 13 on a tailwind, 31 the into-wind pick,
    // 4.7 kt apart.
    let hw13 = nav::headwind_for(&runways, "13", 350.0, 3.0).unwrap();
    let (best, _, best_hw) = nav::best_runway_for_wind(&runways, 350.0, 3.0).unwrap();
    assert!(best == "31" || best == "30", "{best}");
    assert!(
        best_hw - hw13 > RUNWAY_SWAP_MARGIN_KTS,
        "{best_hw} vs {hw13}"
    );

    let mut st = lxgb();
    st.apt.name = "Malaga".into();
    st.apt.runways = runways.clone();
    let words = "november eight one six november november, ten mile final runway one three";
    let intent = tower::parse_intent(words);
    assert_eq!(intent, tower::Intent::ReportFinal);
    st.note_stated_runway(words, intent);
    let ctx = scripted_ctx(&mut st, wx);
    assert_eq!(st.runway_in_use.as_deref(), Some("13"));
    let r = st.route_keyed(words, Service::Tower, &ctx, wx);
    assert!(!r.contains("negative"), "{r}");
    assert!(r.contains("runway one three"), "{r}");
    assert!(r.contains("cleared to land"), "{r}");

    // A tailwind within 5 kt stands in a real wind too (350 at 6: 13
    // carries 4.6 kt of tailwind on the same crosswind as 31)...
    let mut st = lxgb();
    st.apt.runways = runways.clone();
    st.note_stated_runway("final runway one three", tower::Intent::ReportFinal);
    assert_eq!(st.pick_runway(350.0, 6.0), "13");
    // ...but not past it (350 at 10: 7.7 kt of tailwind).
    let mut st = lxgb();
    st.apt.runways = runways.clone();
    st.note_stated_runway("final runway one three", tower::Intent::ReportFinal);
    assert_ne!(st.pick_runway(350.0, 10.0), "13");
    // Sortie 1790583663-approach: "10 mile final runway one three" into
    // 060 at 6 earned "negative, runway one two". The wind does not argue
    // (13's headwind 2.0 against 12's 3.3, inside the margin), so 13
    // stands on the wind alone; the only way to that "negative" is an
    // arrival stream the traffic picture put on another end.
    let hw13 = nav::headwind_for(&runways, "13", 60.0, 6.0).unwrap();
    let (best, _, best_hw) = nav::best_runway_for_wind(&runways, 60.0, 6.0).unwrap();
    assert_eq!(best, "12");
    assert!(best_hw - hw13 <= RUNWAY_SWAP_MARGIN_KTS, "{best_hw} vs {hw13}");
    let mut st = lxgb();
    st.apt.runways = runways.clone();
    st.note_stated_runway("final runway one three", tower::Intent::ReportFinal);
    assert_eq!(st.pick_runway_with(60.0, 6.0, None, None), "13");
    let mut st = lxgb();
    st.apt.runways = runways.clone();
    st.note_stated_runway("final runway one three", tower::Intent::ReportFinal);
    assert_eq!(st.pick_runway_with(60.0, 6.0, Some("12"), None), "12");
    // A crossing strip with a tailwind in limits but a far worse
    // crosswind does not stand (LXGB-like 09/27 plus a synthetic 18/36,
    // wind 360 at 12: 09 has no tailwind but a 12 kt crosswind).
    let mut st = lxgb();
    st.apt.runways = vec![
        nav::Runway {
            width_m: 45.0,
            surface: 1,
            ident_a: "09".into(),
            ident_b: "27".into(),
            lat_a: 36.0,
            lon_a: -5.02,
            lat_b: 36.0,
            lon_b: -4.98,
        },
        nav::Runway {
            width_m: 45.0,
            surface: 1,
            ident_a: "18".into(),
            ident_b: "36".into(),
            lat_a: 36.02,
            lon_a: -5.0,
            lat_b: 35.98,
            lon_b: -5.0,
        },
    ];
    st.note_stated_runway("final runway zero nine", tower::Intent::ReportFinal);
    assert_eq!(st.pick_runway(0.0, 12.0), "36");
    *OWN_STRIP.clearance.lock().unwrap() = None;
}

/// Issue #134, the recorded case: LEMG (sortie 1790584715-full_auto,
/// 2026-09-28), the runner's runway start put the 737 on 13's threshold
/// (36.684289 N 4.512250 W, 131.4 true, fit "13"), wind 342.6 at 2.9 kt,
/// arrivals landing 12 on the party line. The clearance named 31 and full
/// auto taxied 400 s round the field to line up there. A runway start
/// asks for the strip it is on and is given it while the wind does not
/// argue: the calm-wind stream on another end does not move it.
#[test]
fn a_runway_start_is_given_the_runway_it_is_lined_up_on() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let runways = vec![
        nav::Runway {
            width_m: 45.11,
            surface: 1,
            ident_a: "12".into(),
            ident_b: "30".into(),
            lat_a: 36.6910753,
            lon_a: -4.5078270,
            lat_b: 36.6795648,
            lon_b: -4.4805125,
        },
        nav::Runway {
            width_m: 45.11,
            surface: 1,
            ident_a: "13".into(),
            ident_b: "31".into(),
            lat_a: 36.6845573,
            lon_a: -4.5126253,
            lat_b: 36.6654027,
            lon_b: -4.4858140,
        },
    ];
    let (lat, lon) = (36.684288506540085, -4.51224958630406);
    let words = "november eight one six november november, request clearance, runway one three";
    let intent = tower::parse_intent(words);
    assert_eq!(intent, tower::Intent::RequestClearance);
    let station = || {
        let mut st = lxgb();
        st.apt.name = "Malaga".into();
        st.apt.runways = runways.clone();
        st
    };

    // Lined up on 13, the stream on 12: 13.
    let mut st = station();
    st.note_stated_runway(words, intent.clone());
    st.note_caller_on_strip(true, lat, lon);
    assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "13");
    assert_eq!(st.runway_in_use.as_deref(), Some("13"));
    // ...and it holds for the takeoff call that follows (no stated
    // runway, nothing committed, calm).
    assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "13");
    // The same words from an aircraft NOT on the strip: the stream's end,
    // as before.
    let mut st = station();
    st.note_stated_runway(words, intent.clone());
    st.note_caller_on_strip(true, 36.6950, -4.4900);
    assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "12");
    // Airborne is never a runway start.
    let mut st = station();
    st.note_stated_runway(words, intent.clone());
    st.note_caller_on_strip(false, lat, lon);
    assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "12");
    // A wind that really argues still wins: 310 at 12 is 12 kt of tailwind
    // on 13.
    let mut st = station();
    st.note_stated_runway(words, intent.clone());
    st.note_caller_on_strip(true, lat, lon);
    assert_ne!(st.pick_runway_with(310.0, 12.0, None, None), "13");
    // A flight the tower is sequencing keeps its runway.
    let mut st = station();
    st.runway_in_use = Some("31".into());
    st.desk.twr.state = tower::State::TaxiCleared;
    st.note_stated_runway(words, intent.clone());
    st.note_caller_on_strip(true, lat, lon);
    assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "31");

    // The clearance the crew reads names 13.
    let mut st = station();
    st.note_stated_runway(words, intent);
    st.note_caller_on_strip(true, lat, lon);
    assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), None), "13");
    let wx = Wx {
        wind_from_deg: 342.6,
        wind_kts: 2.9,
    };
    let ctx = scripted_ctx(&mut st, wx);
    let r = st.route_keyed(words, Service::Tower, &ctx, wx);
    assert!(r.contains("cleared to leave"), "{r}");
    assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("13"));
    *OWN_STRIP.clearance.lock().unwrap() = None;
}

/// Issues #134 and #140 together (LEMG 2026-09-28, calm): a runway start
/// keeps the runway it is lined up on unless that runway points INTO the
/// live traffic. Lined up on 13 with ghosts departing 13 and landing 12:
/// 13. Lined up on 31 with ghosts departing 13 (or landing 13): refused,
/// the stream's end. On 31 with the departures on 31: 31.
#[test]
fn a_runway_start_keeps_its_strip_unless_it_faces_the_stream() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let runways = lemg_runways();
    let (t13, t31) = (
        (runways[1].lat_a, runways[1].lon_a),
        (runways[1].lat_b, runways[1].lon_b),
    );
    let along = |f: f64| (t13.0 + (t31.0 - t13.0) * f, t13.1 + (t31.1 - t13.1) * f);
    let start = |words: &str, pos: (f64, f64)| {
        let mut st = lxgb();
        st.apt.name = "Malaga".into();
        st.apt.runways = runways.clone();
        st.note_stated_runway(words, tower::parse_intent(words));
        st.note_caller_on_strip(true, pos.0, pos.1);
        st
    };
    let on13 = "november eight one six november november, request clearance, runway one three";
    let on31 = "november eight one six november november, request clearance, runway three one";
    // On 13's threshold, the world departing 13 and landing 12: 13.
    let mut st = start(on13, along(0.01));
    assert_eq!(st.pick_runway_with(342.6, 2.9, Some("12"), Some("13")), "13");
    // On 31's threshold facing the 13 departures: refused, 13.
    let mut st = start(on31, along(0.99));
    assert_eq!(st.pick_runway_with(342.6, 2.9, None, Some("13")), "13");
    // Facing an arrival stream on 13 too.
    let mut st = start(on31, along(0.99));
    assert_eq!(st.pick_runway_with(342.6, 2.9, Some("13"), None), "13");
    // With the world on 31 the start on 31 stands.
    let mut st = start(on31, along(0.99));
    assert_eq!(st.pick_runway_with(342.6, 2.9, None, Some("31")), "31");
    // No live traffic: #134's rule alone, the start stands.
    let mut st = start(on31, along(0.99));
    assert_eq!(st.pick_runway_with(342.6, 2.9, None, None), "31");
}

/// Issue #178, sortie 1790646817-full_auto (LEMG, 2026-09-29): an A330
/// lined up on 13 at (36.68418, -4.51210), no traffic, the tower's wind
/// 293 at 7.3 (6.9 kt down 13), asked "request clearance, runway one
/// three, for circuits" and was cleared on 30 (`stated_runway_refused`
/// best 30). A start's own strip carries a tailwind to 10 kt, and holds
/// for the start's later calls against the wind pick's hysteresis.
#[test]
fn a_runway_start_keeps_its_strip_in_a_light_tailwind() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let (lat, lon) = (36.684182150183844, -4.512099772280839);
    let words = "delta alpha india kilo november heavy, request clearance, runway one \
                     three, for circuits";
    let station = || {
        let mut st = lxgb();
        st.apt.name = "Malaga".into();
        st.apt.runways = lemg_runways();
        st.note_stated_runway(words, tower::parse_intent(words));
        st.note_caller_on_strip(true, lat, lon);
        st
    };
    let (from, kts) = (292.9542, 7.2747811592147995);
    let mut st = station();
    assert_eq!(st.pick_runway_with(from, kts, None, None), "13");
    // The ready call names no runway: the start keeps 13, where the
    // hysteresis alone would take 30's 14 kt better headwind.
    assert_eq!(st.pick_runway_with(from, kts, None, None), "13");
    assert_eq!(st.pick_runway_with(from, kts, None, None), "13");
    // Airborne, the hold is gone and the wind picks again.
    let wx = Wx {
        wind_from_deg: from,
        wind_kts: kts,
    };
    let c = scripted_ctx(&mut st, wx);
    assert!(!c.on_ground);
    st.state_runway_change(&tower::Intent::ReportDownwind, &c, "x".into());
    assert_eq!(st.pick_runway_with(from, kts, None, None), "30");
    // Not on the strip: the arrival's 5 kt rule, refused as before.
    let mut st = station();
    st.desk.call.stated_on_strip = false;
    assert_eq!(st.pick_runway_with(from, kts, None, None), "30");
    // A real tailwind still wins: 310 at 12 down 13.
    let mut st = station();
    assert_ne!(st.pick_runway_with(310.0, 12.0, None, None), "13");
    // A stream on the reciprocal ends the hold (the wind picks 30).
    let mut st = station();
    assert_eq!(st.pick_runway_with(from, kts, None, None), "13");
    assert_ne!(st.pick_runway_with(from, kts, Some("31"), None), "13");
    assert_eq!(st.desk.start_strip, None);
    // The clearance the crew reads names 13.
    let mut st = station();
    assert_eq!(st.pick_runway_with(from, kts, None, None), "13");
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    ctx.own_on_runway = true;
    let r = st.route_keyed(words, Service::Tower, &ctx, wx);
    assert!(r.contains("runway one three, circuit altitude"), "{r}");
    *OWN_STRIP.clearance.lock().unwrap() = None;
}

/// Issue #178, the recorded sortie: LEMG (1790631369-full_auto,
/// 2026-09-28), a 737 lined up on 30, the tower's wind 2 kt. "Request
/// clearance, runway three zero, for circuits" was cleared for the
/// circuit on 30; then a lander on 12 made 13 the stream's end and the
/// calm pick turned round to it (one strip, one direction: the stream
/// wins against an uncommitted clearance). The ready call's "taxi to
/// holding point ... runway one three first" must SAY the runway changed,
/// once, and publish its route for the crew to drive.
#[test]
fn a_runway_turned_under_a_cleared_start_is_said() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    let runways = lemg_runways();
    let (t12, t30) = (
        (runways[0].lat_a, runways[0].lon_a),
        (runways[0].lat_b, runways[0].lon_b),
    );
    let on30 = (t12.0 + (t30.0 - t12.0) * 0.98, t12.1 + (t30.1 - t12.1) * 0.98);
    let mut st = lxgb();
    st.apt.name = "Malaga".into();
    st.apt.runways = runways;
    let wx = Wx {
        wind_from_deg: 300.0,
        wind_kts: 2.06,
    };
    let clearance = "november one seven two sierra papa, request clearance, runway three \
                         zero, for circuits";
    st.note_stated_runway(clearance, tower::parse_intent(clearance));
    st.note_caller_on_strip(true, on30.0, on30.1);
    assert_eq!(st.pick_runway_with(wx.wind_from_deg, wx.wind_kts, None, None), "30");
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    ctx.own_on_runway = true;
    let r = st.route_keyed(clearance, Service::Tower, &ctx, wx);
    assert!(r.contains("runway three zero, circuit altitude"), "{r}");
    // +46.1: D85023's touchdown makes 13 the arrival stream's end.
    assert_eq!(st.pick_runway_with(wx.wind_from_deg, wx.wind_kts, Some("13"), None), "13");
    // +58.1: the ready call from 30, off the strip in use.
    let ready = "november one seven two sierra papa, ready for departure, for circuits";
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    ctx.holding_point = "Hotel November one".into();
    st.desk.planned_route = Some(nav::TaxiRoutePlan {
        via: Vec::new(),
        crossings: Vec::new(),
        holding_point: "HN1".into(),
        path: vec![on30, t12],
        seg: vec![("E11".into(), false)],
        lead_on: Vec::new(),
    });
    let r = st.route_keyed(ready, Service::Tower, &ctx, wx);
    assert!(
        r.starts_with(
            "November one seven two Sierra Papa, runway change, runway one three in use. \
                 Taxi to holding point Hotel November one runway one three first"
        ),
        "{r}"
    );
    assert_eq!(st.desk.twr.state, tower::State::TaxiCleared);
    assert_eq!(crate::party_line::clearance_runway().as_deref(), Some("13"));
    // The route rides the party line like a "request taxi" answer.
    assert!(OWN_STRIP.taxi_route.lock().unwrap().is_some());
    // Said once: the next ground call on 13 carries no change.
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    let r = st.route_keyed(ready, Service::Tower, &ctx, wx);
    assert!(!r.contains("runway change"), "{r}");
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
}

/// The change is only a change: a clearance and a taxi on the same
/// runway say nothing; a flight airborne forgets what it was told.
#[test]
fn a_runway_change_is_said_only_when_the_runway_moved() {
    let mut st = lxgb();
    let wx = Wx {
        wind_from_deg: 300.0,
        wind_kts: 2.0,
    };
    st.runway_in_use = Some("13".into());
    let mut c = scripted_ctx(&mut st, wx);
    c.on_ground = true;
    let mut air = c.clone();
    air.on_ground = false;
    let taxi = format!("{}, taxi to holding point Alpha runway one three.", c.callsign);
    st.state_runway_change(&tower::Intent::RequestClearance, &c, "cleared".into());
    let r = st.state_runway_change(&tower::Intent::RequestTaxi, &c, taxi.clone());
    assert_eq!(r, taxi);
    st.runway_in_use = Some("31".into());
    let r = st.state_runway_change(&tower::Intent::RequestTakeoff, &air, taxi.clone());
    assert_eq!(r, taxi);
    let r = st.state_runway_change(&tower::Intent::RequestTaxi, &c, taxi.clone());
    assert_eq!(r, taxi, "airborne forgot the runway it was told");
    // Told 31 on the ground, then moved to 13: said.
    st.state_runway_change(&tower::Intent::RequestClearance, &c, "cleared".into());
    st.runway_in_use = Some("13".into());
    let r = st.state_runway_change(&tower::Intent::RequestTaxi, &c, taxi.clone());
    assert!(r.contains(", runway change, runway "), "{r}");
}

/// Two parallels have the SAME heading to within rounding error, so which
/// one "wins" the wind was decided by noise — and nothing in the
/// phraseology gives it away, both are "two seven". A silent flip from the
/// 27L taxi clearance to 27R pointed the takeoff hold at Heathrow's
/// ARRIVAL stream, which never has a 15 s gap: held at November one for
/// the full eight minutes and handed back, "traffic on short final" for a
/// departure runway that was empty (2026-07-27).
#[test]
fn parallel_runways_never_trade_places_under_a_departure() {
    let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
    // Heathrow's real pair: both 09/27, headings a hundredth of a degree
    // apart. 09L/27R is first in apt.dat, so it wins any tie.
    st.apt.runways = vec![
        nav::Runway {
            width_m: 50.0,
            surface: 0,
            ident_a: "09L".into(),
            ident_b: "27R".into(),
            lat_a: 51.4774781,
            lon_a: -0.4894423,
            lat_b: 51.4776753,
            lon_b: -0.4332600,
        },
        nav::Runway {
            width_m: 50.0,
            surface: 0,
            ident_a: "09R".into(),
            ident_b: "27L".into(),
            lat_a: 51.4647701,
            lon_a: -0.4867489,
            lat_b: 51.4649460,
            lon_b: -0.4340638,
        },
    ];
    // The tower is working 27L — the taxi clearance named it.
    st.runway_in_use = Some("27L".into());
    st.desk.twr.state = tower::State::TaxiCleared;
    // Every subsequent transmission, in any wind, keeps 27L: the departure
    // is committed and a controller does not move it to the other
    // parallel mid-sequence.
    for (from, kts) in [(250.0, 12.0), (270.0, 25.0), (269.0, 8.0), (271.0, 30.0)] {
        assert_eq!(st.pick_runway(from, kts), "27L", "wind {from}/{kts}");
    }
    st.desk.twr.state = tower::State::LinedUp;
    assert_eq!(st.pick_runway(280.0, 18.0), "27L");
    st.desk.twr.state = tower::State::TakeoffCleared;
    assert_eq!(st.pick_runway(260.0, 18.0), "27L");

    // Uncommitted, the hysteresis still holds the pick against a parallel
    // that is "better" by rounding error...
    st.desk.twr.state = tower::State::Parked;
    for (from, kts) in [(250.0, 12.0), (270.0, 20.0), (271.0, 9.0)] {
        assert_eq!(st.pick_runway(from, kts), "27L", "wind {from}/{kts}");
    }
    // ...but a genuine reversal still turns the field around, as it must,
    // onto the main (longest) strip facing the new way: 09L/27R is
    // 3,902 m against 09R/27L's 3,660 m (`nav::best_runway_for_wind`).
    assert_eq!(st.pick_runway(90.0, 20.0), "09L");
}

/// Issue #155, sortie 1790601822-full_auto (EGBB 15, 2026-09-28):
/// "traffic, 5 o'clock, 1 miles, 900 feet above" (+3194.7) and "4
/// o'clock, 1 miles, 300 feet above" (+3240.1). One mile is "a mile".
#[test]
fn a_traffic_advisory_says_a_mile() {
    let f = |nm: f64, clock: i32, ft: f64| TrafficFactor::new(nm, clock, ft);
    assert_eq!(
        advisory_line(&f(1.2, 5, 900.0)),
        "traffic, 5 o'clock, a mile, 900 feet above."
    );
    assert!(advisory_line(&f(1.49, 4, 300.0)).contains(", a mile, "));
    assert!(advisory_line(&f(0.4, 12, 0.0)).contains("under a mile"));
    assert!(advisory_line(&f(1.5, 12, 0.0)).contains(", 2 miles, "));
    assert!(advisory_line(&f(6.0, 12, 0.0)).contains(", 6 miles, "));
    for nm in [0.2, 0.99, 1.0, 1.3, 1.49, 1.5, 2.4, 5.9] {
        assert!(!advisory_line(&f(nm, 3, 0.0)).contains(" 1 miles"), "{nm}");
    }
}

#[test]
fn near_miss_alert_names_the_way_out() {
    // The FAA's words (7110.65 2-1-6) at an FAA field.
    let f = |nm: f64, clock: i32, ft: f64| TrafficFactor {
        dialect: Dialect::Faa,
        ..TrafficFactor::new(nm, clock, ft)
    };
    // The line: half a mile, 500 ft. Outside it, only the advisory tier.
    assert!(is_near_miss(&f(0.4, 3, 0.0)));
    assert!(!is_near_miss(&f(0.6, 3, 0.0)), "outside half a mile");
    assert!(
        !is_near_miss(&f(0.4, 3, 600.0)),
        "vertical separation holds"
    );
    // Vertical escape when the intruder is clearly above or below.
    assert!(near_miss_line(&f(0.4, 12, 300.0), false).contains("advise you descend"));
    assert!(near_miss_line(&f(0.4, 12, -300.0), false).contains("advise you climb"));
    // Co-altitude: turn AWAY from the traffic's side...
    assert!(near_miss_line(&f(0.4, 2, 0.0), false).contains("turn left"));
    assert!(near_miss_line(&f(0.4, 10, 0.0), false).contains("turn right"));
    // ...right for head-on (the right-of-way rule)...
    assert!(near_miss_line(&f(0.4, 12, 0.0), false).contains("turn right"));
    // ...and vertical for one dead astern, where a turn hides it.
    assert!(near_miss_line(&f(0.4, 6, 0.0), false).contains("climb"));
    // Controller shape vs crew shape: same geometry, right register.
    let atc = near_miss_line(&f(0.4, 2, 0.0), false);
    assert!(atc.starts_with("traffic alert"), "{atc}");
    assert!(atc.ends_with("immediately."), "{atc}");
    // Europe never hears the FAA's words (the realism review, 2026-10-09):
    // ICAO Doc 4444's avoiding action, and CAP 413's own words in the UK.
    let icao = near_miss_line(&TrafficFactor::new(0.4, 2, 0.0), false);
    assert_eq!(
        icao,
        "turn left immediately to avoid traffic, 2 o'clock, less than a mile, same altitude."
    );
    let uk = TrafficFactor {
        uk: true,
        ..TrafficFactor::new(0.4, 12, -300.0)
    };
    assert_eq!(
        near_miss_line(&uk, false),
        "avoiding action, climb immediately, traffic 12 o'clock, less than a mile, 300 feet \
         below."
    );
    for line in [icao, near_miss_line(&uk, false)] {
        assert!(!line.contains("traffic alert") && !line.contains("advise you"), "{line}");
    }
    let crew = near_miss_line(&f(0.4, 2, 0.0), true);
    assert!(crew.starts_with("Traffic!"), "{crew}");
    assert!(crew.contains("now."), "{crew}");
}

/// The adversarial corpus (`bench/atc_corpus.yaml`): real whisper output
/// mined from the flight logs plus hand-written paraphrases, homophone
/// mangles, and crew-talk near-misses. Every case judges up to three
/// parsers — intent, the unkeyed radio-call gate, the readback detector —
/// and failures collect so one run reports every drift at once.
#[test]
fn transcript_corpus_parses_safely() {
    #[derive(serde::Deserialize)]
    struct Case {
        text: String,
        intent: String,
        #[serde(default)]
        radio: Option<bool>,
        #[serde(default)]
        readback: Option<bool>,
        /// The destination an IFR request must carry, as parsed
        /// (`request_ifr_clearance` only; absent = none named).
        #[serde(default)]
        dest: Option<String>,
        /// Our spoken callsign for the case: `radio` is then judged as
        /// the station's unkeyed gate judges it, the radio words OR the
        /// callsign named (`callsign::addressed`).
        #[serde(default)]
        callsign: Option<String>,
    }
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/bench/atc_corpus.yaml");
    let cases: Vec<Case> =
        serde_yaml::from_str(&std::fs::read_to_string(path).expect("corpus file"))
            .expect("corpus yaml");
    assert!(cases.len() >= 100, "corpus shrank to {} cases", cases.len());
    let mut fails = Vec::new();
    for c in &cases {
        let t = c.text.to_lowercase();
        let Some(mut want) = tower::intent_from_label(&c.intent) else {
            panic!("{:?}: unknown intent label {:?}", c.text, c.intent);
        };
        if let tower::Intent::RequestIfrClearance { dest } = &mut want {
            *dest = c.dest.clone();
        }
        let got = tower::parse_intent(&t);
        if got != want {
            fails.push(format!("{:?}: parsed {got:?}, wanted {want:?}", c.text));
        }
        if let Some(radio) = c.radio {
            let ours = c
                .callsign
                .as_deref()
                .is_some_and(|cs| crate::callsign::addressed(&t, cs));
            if (is_radio_call(&t, "gibraltar") || ours) != radio {
                fails.push(format!("{:?}: is_radio_call should be {radio}", c.text));
            }
        }
        if let Some(rb) = c.readback {
            if tower::is_readback(&t) != rb {
                fails.push(format!("{:?}: is_readback should be {rb}", c.text));
            }
        }
    }
    assert!(
        fails.is_empty(),
        "{} of {} corpus cases drifted:\n{}",
        fails.len(),
        cases.len(),
        fails.join("\n")
    );
}

fn calm() -> Wx {
    Wx {
        wind_from_deg: 0.0,
        wind_kts: 0.0,
    }
}

/// The live context exactly as `build_ctx` would assemble it, minus the
/// simulator: scripted wind, fixed QNH and tail number.
fn scripted_ctx(st: &mut Station, wx: Wx) -> tower::Ctx {
    st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        false, // airborne default; only the mayday answer reads it
        false, // not an arrival taxi-in
        None,  // final clear; traffic-aware cases build their own ctx
        false, // runway unoccupied
        None,  // no wake clock
    )
}

/// EGBB 33, 2026-09-27 (sortie 1790534902): Birmingham has no departure
/// frequency, so the tower handed the climbing flight to APPROACH. The
/// crew read the handoff back on the tower (silence, correct), retuned,
/// and checked in: "passing three thousand one hundred climbing one five
/// thousand" was heard by approach as a plain readback and answered with
/// nothing, the tower still holding the flight cleared for takeoff. The
/// position a departure is handed to takes its check-in, and the tower's
/// book on the flight closes there.
#[test]
fn approach_takes_the_departure_check_in_when_it_is_the_handoff() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.departure_khz = None;
    let ctx = scripted_ctx(&mut st, wx);
    // An arrival's readback to approach, no departure handed over: still
    // silence, never "radar contact".
    let r = st.route_keyed("descending two thousand", Service::Approach, &ctx, wx);
    assert_eq!(r, "", "{r}");

    st.desk.twr.state = tower::State::TaxiCleared;
    let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared for takeoff"), "{r}");
    let _ = crate::party_line::take_departure_handoff();
    let line = handoff_tick(900.0, false).expect("handoff due");
    assert!(line.contains("Gibraltar Approach on one two two decimal eight"), "{line}");
    let _ = crate::party_line::take_departure_handoff();

    // The readback, on the TOWER before the retune: a frequency readback
    // earns silence and moves nothing.
    let r = st.route_keyed(
        "whiskey alpha tango four two three six, gibraltar approach on one two two decimal eight",
        Service::Tower,
        &ctx,
        wx,
    );
    assert_eq!(r, "", "{r}");
    assert_eq!(st.desk.twr.state, tower::State::TakeoffCleared);

    // The check-in on approach, as the crew said it.
    let checkin = "birmingham approach, whiskey alpha tango four two three six, passing three thousand one hundred climbing one five thousand";
    assert_eq!(tower::parse_intent(checkin), tower::Intent::Unknown);
    assert!(tower::is_readback(checkin));
    let r = st.route_keyed(checkin, Service::Approach, &ctx, wx);
    assert_eq!(
        r,
        "November one seven two Sierra Papa, Gibraltar Approach, radar contact, climb altitude \
         one five thousand feet."
    );
    assert_eq!(st.desk.twr.state, tower::State::Parked, "the tower's book is closed");
    let snap = crate::party_line::clearance_state();
    assert_eq!(snap, Some(tower::State::Parked));
    assert!(!st.desk.twr.holds_runway());
    // Later readbacks on approach stay quiet.
    let r = st.route_keyed("maintaining one five thousand", Service::Approach, &ctx, wx);
    assert_eq!(r, "", "{r}");
    // The same flight coming back is an arrival from Parked.
    let r = st.route_keyed("november one seven two, final", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared to land"), "{r}");
    *OWN_STRIP.clearance.lock().unwrap() = None;
}

/// EGKK 26L, 2026-09-27 (sortie 1790534902, +2735 s): the crew's landing
/// readback parsed as a landing REQUEST in ClearedToLand. The route kept
/// quiet only because the word "cleared" silenced it; the intent now says
/// what it is, a readback, and the unkeyed path (where no such guard
/// stood) can no longer walk the landing back to "join final".
#[test]
fn a_landing_readback_is_a_readback_not_a_landing_request() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    let readback = "whiskey alpha tango four two three six, cleared to land runway two seven";
    assert_eq!(tower::parse_intent(readback), tower::Intent::Unknown);
    assert!(tower::is_readback(readback));

    // Keyed: the gate takes the echo, the frequency stays quiet, the
    // clearance stands and nothing is owed.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let ctx = scripted_ctx(&mut st, wx);
    let r = st.route_keyed("november one seven two, final", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared to land"), "{r}");
    assert!(st.desk.twr.pending().is_some(), "the landing readback is owed");
    let r = st.route_keyed(readback, Service::Tower, &ctx, wx);
    assert_eq!(r, "", "{r}");
    assert_eq!(st.desk.twr.state, tower::State::ClearedToLand);
    assert!(st.desk.twr.pending().is_none(), "paid");
    // Said again with nothing owed: still silence, still cleared.
    let r = st.route_keyed(readback, Service::Tower, &ctx, wx);
    assert_eq!(r, "", "{r}");
    assert_eq!(st.desk.twr.state, tower::State::ClearedToLand);

    // Unkeyed, addressed by our callsign: not the tower's business as a
    // request any more; it falls through and the landing stands.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.callsign = "Whiskey Alpha Tango four two three six".into();
    st.route_keyed("whiskey alpha tango four two three six, final", Service::Tower, &ctx, wx);
    assert_eq!(st.desk.twr.state, tower::State::ClearedToLand);
    assert_eq!(st.route_unkeyed(readback, Service::Tower, &ctx, wx), None);
    assert_eq!(st.desk.twr.state, tower::State::ClearedToLand, "never walked back to Inbound");
    *OWN_STRIP.clearance.lock().unwrap() = None;
}

/// Whole sorties driven through the pure routing layer — the dialogue
/// sequences that live between the tower FSM (unit-tested above it) and
/// the mic (untestable): out-of-order calls, questions on a keyed radio,
/// readbacks, wrong-service redirects, the departure check-in. Every
/// scene asserts the reply AND that the clearance sequence only moves
/// when it should.
#[test]
fn sortie_dialogues_battle_test_the_routing() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };

    // Scene 1: the departure happy path, keyed on the tower frequency.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let ctx = scripted_ctx(&mut st, wx);
    let r = st.route_keyed("gibraltar tower, radio check", Service::Tower, &ctx, wx);
    assert!(r.contains("reading you five"), "{r}");
    assert_eq!(
        st.desk.twr.state,
        tower::State::Parked,
        "a radio check moves nothing"
    );

    // The zone clearance suggests its readback (cap, QNH, squawk) and
    // enforces nothing; the crew's echo pays it and earns silence.
    let r = st.route_keyed("request clearance", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared to leave the Gibraltar zone"), "{r}");
    let owed = st.desk.twr.suggested_readback(&ctx.callsign).expect("a suggested readback");
    assert!(owed.starts_with("Not above two thousand feet, QNH one zero one seven, squawk"), "{owed}");
    assert!(owed.ends_with(&format!("{}.", ctx.callsign)), "{owed}");
    let r = st.route_keyed(&owed.to_lowercase(), Service::Tower, &ctx, wx);
    assert_eq!(r, "", "a clearance readback keeps the frequency quiet: {r}");
    assert!(st.desk.twr.suggested_readback(&ctx.callsign).is_none(), "paid");

    let r = st.route_keyed("request clearance for taxi", Service::Tower, &ctx, wx);
    assert!(
        r.contains("taxi to holding point Alpha runway two seven"),
        "{r}"
    );
    assert_eq!(st.desk.twr.state, tower::State::TaxiCleared);

    // A readback earns silence; garble earns "say again"; neither moves
    // the sequence.
    let r = st.route_keyed("roger, holding point alpha", Service::Tower, &ctx, wx);
    assert_eq!(r, "", "a readback keeps the frequency quiet: {r}");
    let r = st.route_keyed(
        "uhh gibraltar something something",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("say again"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::TaxiCleared);

    // A question keyed by mistake gets "say again", never a clearance —
    // the 2026-07-08 regression, now at the routing layer.
    let r = st.route_keyed(
        "what runway shall i take off from",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("say again"), "{r}");
    let r = st.route_keyed("takeoff checklist", Service::Tower, &ctx, wx);
    assert!(r.contains("say again"), "{r}");
    assert_eq!(
        st.desk.twr.state,
        tower::State::TaxiCleared,
        "questions moved the sequence"
    );

    let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared for takeoff"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::TakeoffCleared);

    // The takeoff clearance armed the handoff; airborne through 700 ft it
    // fires exactly once, to the departure frequency.
    assert!(handoff_tick(300.0, false).is_none(), "still low");
    // Drain whatever another scene left on the party line: the handoff
    // is published for the crew the moment the line goes out, below.
    let _ = crate::party_line::take_departure_handoff();
    let line = handoff_tick(900.0, false).expect("handoff due");
    assert!(
        line.contains("Departure on one three zero decimal four"),
        "{line}"
    );
    assert!(handoff_tick(1500.0, false).is_none(), "spoken once");
    // The crew gets the instruction as data: whom to call, on what.
    let (to, khz) =
        crate::party_line::take_departure_handoff().expect("published for the crew");
    assert_eq!((to.as_str(), khz), ("Gibraltar Departure", 130_400));
    assert!(
        crate::party_line::take_departure_handoff().is_none(),
        "consumed once"
    );

    // Scene 2: the departure controller. First readback is the check-in
    // and gets radar contact; later readbacks earn silence.
    let r = st.route_keyed(
        "passing one thousand five hundred climbing",
        Service::Departure,
        &ctx,
        wx,
    );
    assert!(r.contains("Departure, radar contact"), "{r}");
    let r = st.route_keyed("maintaining two thousand", Service::Departure, &ctx, wx);
    assert_eq!(r, "", "post-check-in readbacks stay quiet: {r}");

    // Runway-shaped business on the wrong frequency redirects to the
    // tower and must not advance the tower's sequence.
    let was = st.desk.twr.state;
    let r = st.route_keyed("request taxi", Service::Approach, &ctx, wx);
    assert!(
        r.contains("contact Gibraltar Tower on one three one decimal two"),
        "{r}"
    );
    let r = st.route_keyed("ready for departure", Service::Departure, &ctx, wx);
    assert!(r.contains("contact Gibraltar Tower"), "{r}");
    assert_eq!(st.desk.twr.state, was, "a redirect must not move the sequence");

    // Coming back in: departure sends arrivals to approach when the field
    // has one, and the touch-and-go option survives the redirect.
    let r = st.route_keyed("request touch and go", Service::Departure, &ctx, wx);
    assert!(
        r.contains("contact Gibraltar Approach on one two two decimal eight"),
        "{r}"
    );
    let clear = st.route_keyed("november one seven two, final", Service::Tower, &ctx, wx);
    assert!(clear.contains("cleared touch and go"), "{clear}");

    // Scene 3: the arrival, keyed on the tower — inbound, final, then off
    // the runway. Calm wind must hold the westerly runway pick.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let ctx = scripted_ctx(&mut st, wx);
    let r = st.route_keyed(
        "gibraltar tower, inbound for landing",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("join final runway two seven"), "{r}");
    let calm_ctx = scripted_ctx(&mut st, calm());
    assert!(
        calm_ctx.runway.contains("two seven"),
        "calm flipped the runway"
    );
    let r = st.route_keyed("final", Service::Tower, &calm_ctx, calm());
    assert!(r.contains("cleared to land"), "{r}");
    // Reading the clearance back must earn silence and hold the state —
    // it used to re-parse as a landing request and walk the sequence
    // backwards to Inbound.
    let r = st.route_keyed(
        "cleared to land runway two seven, november one seven two sierra papa",
        Service::Tower,
        &calm_ctx,
        calm(),
    );
    assert_eq!(r, "", "a clearance echo keeps the frequency quiet: {r}");
    assert_eq!(
        st.desk.twr.state,
        tower::State::ClearedToLand,
        "echo moved the sequence"
    );
    let r = st.route_keyed("request taxi", Service::Tower, &calm_ctx, calm());
    assert!(r.contains("taxi to parking"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::Parked);
    let r = st.route_keyed("i've just landed", Service::Tower, &calm_ctx, calm());
    assert!(r.contains("taxi to parking"), "{r}");

    // Scene 4: content routing (no PTT). Crew talk falls through even
    // when it contains clearance words; radio-shaped calls route.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let ctx = scripted_ctx(&mut st, wx);
    for crew in [
        "anything else we need to do before takeoff",
        "land the aircraft",
        "flaps set for takeoff",
        "after landing checklist",
        "gibraltar is somewhere ahead",
    ] {
        assert!(
            st.route_unkeyed(crew, Service::Tower, &ctx, wx).is_none(),
            "crew talk routed to ATC: {crew}"
        );
    }
    // "hello tower" IS addressed to the tower — since 2026-07-10 a hail
    // on frequency earns the invitation, and moves nothing.
    let r = st
        .route_unkeyed("hello tower", Service::Tower, &ctx, wx)
        .expect("a hail addressed to the tower is answered");
    assert!(r.contains("pass your message"), "{r}");
    assert_eq!(
        st.desk.twr.state,
        tower::State::Parked,
        "crew talk moved the sequence"
    );
    let r = st
        .route_unkeyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx)
        .expect("a proper call routes");
    assert!(r.contains("holding point"), "{r}");

    // Any leftover handoff from this sortie must not leak into the next
    // test holding the lock.
    *OWN_STRIP.handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// An airline flight is worked by its airline callsign: the tower
/// answers "Speedbird three zero six", and a call that opens with it is
/// addressed even without a station name or a "request" (a 737 on an
/// OFP as BAW306 was called by its registration, 2026-09-27).
#[test]
fn an_airline_callsign_is_addressed_and_answered() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.callsign = crate::callsign::resolve(
        &crate::callsign::Inputs {
            override_id: None,
            plan: Some(crate::callsign::PlanCallsign {
                id: "BAW306".into(),
                plan_type: "B738".into(),
                also_types: Vec::new(),
            }),
        },
        Some("G-EUPT"),
        Some("B738"),
        Some(79_000.0),
    )
    .expect("a callsign")
    .spoken;
    assert_eq!(ctx.callsign, "Speedbird three zero six");

    // "Ready to taxi" alone is crew talk; with our callsign in front it
    // is a call to the tower.
    assert!(!is_radio_call("brakes set, ready to taxi", "gibraltar"));
    assert!(
        st.route_unkeyed("brakes set, ready to taxi", Service::Tower, &ctx, wx)
            .is_none(),
        "crew talk stays crew talk"
    );
    let r = st
        .route_unkeyed("Speedbird 306, ready to taxi", Service::Tower, &ctx, wx)
        .expect("our callsign addresses the tower");
    assert!(r.starts_with("Speedbird three zero six"), "{r}");
    assert!(r.contains("holding point"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::TaxiCleared);

    // Keyed, the whisper spelling of the same callsign.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let r = st.route_keyed(
        "speed bird three zero six, request taxi",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.starts_with("Speedbird three zero six"), "{r}");
    assert!(r.contains("holding point"), "{r}");
    *OWN_STRIP.handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// A virtual airline the tables do not know is spoken by the pilot's own
/// telephony (the setting `airline_telephony`, "WAT=Walker", reaching the
/// crate through its provider): SimBrief's WAT2516 is "Walker two five one
/// six" to the tower, and "Walker 2516, request taxi" is ours keyed and
/// unkeyed (Walker Air was spelled "Whiskey Alpha Tango", 2026-09-27).
#[test]
fn a_pilots_telephony_is_spoken_and_addressed() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    crate::callsign::provide_telephony(|| "wat=Walker; junk".to_string());
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let mut ctx = scripted_ctx(&mut st, wx);
    let inputs = crate::callsign::Inputs {
        override_id: None,
        plan: Some(crate::callsign::PlanCallsign {
            id: "WAT2516".into(),
            plan_type: "B738".into(),
            also_types: Vec::new(),
        }),
    };
    let r = crate::callsign::resolve(&inputs, Some("G-EUPT"), Some("B738"), Some(79_000.0))
        .expect("a callsign");
    assert_eq!(r.source, crate::callsign::Source::Simbrief);
    ctx.callsign = r.spoken;
    assert_eq!(ctx.callsign, "Walker two five one six");

    let r = st
        .route_unkeyed("Walker 2516, request taxi", Service::Tower, &ctx, wx)
        .expect("our callsign addresses the tower");
    assert!(r.starts_with("Walker two five one six"), "{r}");
    assert!(r.contains("holding point"), "{r}");
    assert!(
        st.route_unkeyed("Walker 2517, ready to taxi", Service::Tower, &ctx, wx)
            .is_none(),
        "another flight's number is not ours"
    );

    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let r = st.route_keyed(
        "walker two five one six, request taxi",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.starts_with("Walker two five one six"), "{r}");
    assert!(r.contains("holding point"), "{r}");

    *crate::party_line::CALLSIGN_TELEPHONY
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = None;
    assert_eq!(
        crate::callsign::spoken_id("WAT2516"),
        "Whiskey Alpha Tango two five one six",
        "without the setting, spelled as before"
    );
    *OWN_STRIP.handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

#[test]
fn readback_discipline_runs_through_the_keyed_route() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 8.0,
    }; // runway 09
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;

    let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx);
    assert!(r.contains("holding point"), "{r}");
    let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared for takeoff"), "{r}");
    // The coach owes exactly the readback now.
    let owed = OWN_STRIP.clearance
        .lock()
        .unwrap()
        .as_ref()
        .and_then(|c| c.pending_readback.clone())
        .expect("readback owed");
    assert!(
        owed.contains("Cleared for takeoff runway zero niner"),
        "{owed}"
    );
    // A different call is challenged ("<callsign>, read back", the flight
    // named once); nothing moves. (A TAXI request is not challenged: it
    // cancels the takeoff, see `a_taxi_request_cancels_a_takeoff_clearance`.)
    let r = st.route_keyed(
        "gibraltar tower, request start up",
        Service::Tower,
        &ctx,
        wx,
    );
    assert_eq!(r, "November one seven two Sierra Papa, read back.");
    assert_eq!(
        st.desk.twr.state,
        tower::State::TakeoffCleared,
        "challenge moves nothing"
    );
    // The WRONG runway is corrected, never accepted.
    let r = st.route_keyed(
        "cleared for takeoff runway two seven, november one seven two sierra papa",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("negative"), "{r}");
    // The right echo earns silence and pays the debt.
    let r = st.route_keyed(
        "cleared for takeoff runway zero niner, november one seven two sierra papa",
        Service::Tower,
        &ctx,
        wx,
    );
    assert_eq!(r, "", "correct readback = a quiet frequency");
    assert!(
        OWN_STRIP.clearance
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .pending_readback
            .is_none(),
        "debt paid, coach released"
    );

    // Enforcement off: the challenge disappears, the call routes.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").unwrap();
    st.set_enforce_readback(false);
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    st.route_keyed("request taxi", Service::Tower, &ctx, wx);
    let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared for takeoff"), "{r}");
    let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx);
    assert!(
        r.contains("holding point"),
        "no challenge with enforcement off: {r}"
    );

    *OWN_STRIP.clearance.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.takeoff_hold.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// EGKK 2026-09-27 (sorties 1790525500 and 1790525553) replayed through
/// the keyed route. The resumed 737 was cleared for takeoff; its queued
/// "ready for departure" went out as it rolled, traffic stood on the
/// strip, and the tower answered "cancel takeoff clearance, hold
/// position" and armed the release watch. The crew rejected and stopped.
/// Then: the watch replayed "traffic clear, cleared for takeoff" 1.2 s
/// into the resume, and the resume's taxi request was answered "read
/// back: runway two six left, cleared for takeoff". Now the crew's reject
/// call cancels it all, and the taxi request gets its taxi.
#[test]
fn a_rejected_takeoff_cancels_the_clearance_and_the_held_release() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *WAKE.lock().unwrap() = None;
    *OWN_STRIP.takeoff_hold.lock().unwrap() = None;
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 8.0,
    };
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    let cs = "november one seven two sierra papa";

    st.route_keyed(&format!("{cs}, request taxi"), Service::Tower, &ctx, wx);
    let r = st.route_keyed(
        &format!("{cs}, ready for departure"),
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("cleared for takeoff"), "{r}");
    let r = st.route_keyed(
        &format!("cleared for takeoff runway zero niner, {cs}"),
        Service::Tower,
        &ctx,
        wx,
    );
    assert_eq!(r, "", "read back");
    // The queued re-report lands with traffic on the strip: cancelled,
    // held, the release watch armed.
    let mut busy = ctx.clone();
    busy.runway_occupied = true;
    let r = st.route_keyed(
        &format!("{cs}, ready for departure"),
        Service::Tower,
        &busy,
        wx,
    );
    assert!(r.contains("cancel takeoff clearance"), "{r}");
    assert!(
        OWN_STRIP.takeoff_hold.lock().unwrap().is_some(),
        "release watch armed"
    );
    // A released clearance the main loop has not synced yet.
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = true;

    // The crew's reject call, as the drain keys it: never challenged for
    // the readback it owes, answered, and everything it armed retired.
    let r = st.route_keyed(
        &format!("{cs}, rejecting takeoff"),
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("roger") && r.contains("Hold position"), "{r}");
    assert!(!r.contains("read back"), "{r}");
    assert_eq!(
        st.desk.twr.state,
        tower::State::LinedUp,
        "on the runway, no clearance"
    );
    assert!(st.desk.twr.pending().is_none(), "nothing owed");
    assert!(
        OWN_STRIP.takeoff_hold.lock().unwrap().is_none(),
        "the held release is void"
    );
    assert!(
        !*OWN_STRIP.pending_takeoff_clearance.lock().unwrap(),
        "the unsynced release is void"
    );
    let (state, holds, owed) = {
        let c = OWN_STRIP.clearance.lock().unwrap();
        let c = c.as_ref().unwrap();
        (c.state, c.holds_runway, c.pending_readback.clone())
    };
    assert_eq!(state, tower::State::LinedUp);
    assert!(holds, "the runway is still ours");
    assert!(owed.is_none(), "the coach owes nothing");
    // Nothing replays, however long the resume takes.
    let t0 = Instant::now();
    for s in [0, 15, 30, 120] {
        assert!(
            takeoff_hold_tick(&[], 11.0, true, t0 + std::time::Duration::from_secs(s))
                .is_none(),
            "no replay at +{s} s"
        );
    }

    // The resume, stopped mid-runway: its taxi request gets the taxi.
    let r = st.route_keyed(
        &format!("{cs}, request taxi, runway zero niner"),
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("holding point"), "{r}");
    assert!(!r.contains("read back"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::TaxiCleared);

    // From the start of the runway instead, it asks for takeoff again.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").unwrap();
    st.route_keyed(&format!("{cs}, request taxi"), Service::Tower, &ctx, wx);
    st.route_keyed(
        &format!("{cs}, ready for departure"),
        Service::Tower,
        &ctx,
        wx,
    );
    st.route_keyed(&format!("{cs}, stopping"), Service::Tower, &ctx, wx);
    assert_eq!(st.desk.twr.state, tower::State::LinedUp);
    let r = st.route_keyed(
        &format!("{cs}, ready for departure"),
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("cleared for takeoff"), "a fresh clearance: {r}");

    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.handoff.lock().unwrap() = None;
    *OWN_STRIP.takeoff_hold.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
}

/// The tower's own eyes, for a reject call it never heard: the hold armed
/// at +14 of sortie 1790525500 rode a roll to 94 kt and was still armed at
/// the resume. Rolling past 40 kt spends it; a taxi at 25 kt does not.
#[test]
fn a_held_release_is_spent_by_the_roll() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.takeoff_hold.lock().unwrap() = Some(TakeoffHold {
        callsign: "Whiskey Alpha Tango one seven three".into(),
        runway: "two six left".into(),
        runway_ident: "26L".into(),
        clear_since: None,
        armed_at: Instant::now(),
        logged_at: Instant::now(),
        lined_up: true,
        behind: false,
        held_for: None,
        cut_in_logged: Default::default(),
    });
    assert!(
        !void_takeoff_hold_by_roll(25.0),
        "a taxi pace spends nothing"
    );
    assert!(OWN_STRIP.takeoff_hold.lock().unwrap().is_some());
    assert!(void_takeoff_hold_by_roll(94.0), "the roll spends it");
    assert!(OWN_STRIP.takeoff_hold.lock().unwrap().is_none());
    assert!(!void_takeoff_hold_by_roll(94.0), "once");
    assert!(takeoff_hold_tick(&[], 200.0, true, Instant::now()).is_none());
}

/// A flight cleared for takeoff that asks to taxi is not taking the
/// takeoff: the tower cancels it out loud and gives the taxi, never "read
/// back: cleared for takeoff" (the EGKK resume's answer when the crew's
/// reject call never reached the tower).
#[test]
fn a_taxi_request_cancels_a_takeoff_clearance() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *OWN_STRIP.takeoff_hold.lock().unwrap() = None;
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 8.0,
    };
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    st.route_keyed("request taxi", Service::Tower, &ctx, wx);
    let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared for takeoff"), "{r}");
    assert!(st.desk.twr.pending().is_some(), "the takeoff readback is owed");
    let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx);
    assert!(r.contains("takeoff clearance cancelled"), "{r}");
    assert!(r.contains("holding point"), "{r}");
    assert!(!r.contains("read back"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::TaxiCleared);
    assert!(st.desk.twr.pending().is_none());
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.handoff.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
}

#[test]
fn conditional_line_up_arms_the_release_watch() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 8.0,
    };
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    ctx.final_traffic_nm = Some(2.5);
    ctx.final_traffic_count = 1;
    ctx.final_traffic_callsign = Some("Speedbird three zero six".into());
    *OWN_STRIP.takeoff_hold.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *WAKE.lock().unwrap_or_else(|e| e.into_inner()) = None;

    st.respond("", Service::Tower, tower::Intent::RequestTaxi, &ctx, wx);
    let r = st.respond("", Service::Tower, tower::Intent::RequestTakeoff, &ctx, wx);
    assert!(
        r.contains("behind the landing traffic on 2 mile final"),
        "{r}"
    );
    assert!(r.contains("line up runway zero niner behind"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::LinedUp);
    assert!(
        OWN_STRIP.takeoff_hold.lock().unwrap().is_some(),
        "a conditional line-up owes the clearance: release watch armed"
    );
    assert!(
        OWN_STRIP.takeoff_hold
            .lock()
            .unwrap()
            .as_ref()
            .is_some_and(|h| h.behind),
        "the watch knows the line-up was conditional"
    );

    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1508156,
        lon_a: -5.3596312,
        lat_b: 36.1514944,
        lon_b: -5.3400082,
    }];
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    // The lander still on final: the crew's condition reads NOT clear —
    // "behind" means after it, and the Baron that rolled five seconds
    // after the words (Malaga, 2026-09-11) had nothing to read.
    let lander = TcasTarget {
        callsign: "BAW306".into(),
        lat: 36.1497,
        lon: -5.4000,
        ele_ft: 650.0,
        on_ground: false,
        vs_fpm: -600.0,
    };
    assert!(
        takeoff_hold_tick(std::slice::from_ref(&lander), 11.0, true, t0).is_none(),
        "held behind the lander"
    );
    assert_eq!(crate::party_line::line_up_condition_clear(), Some(false));

    // The lander is down and gone: the condition clears, then the watch
    // releases the clearance after the clear window, and the FSM sync is
    // queued for the main loop exactly like a plain hold.
    assert!(
        takeoff_hold_tick(&[], 11.0, true, t0).is_none(),
        "clear window starts"
    );
    assert_eq!(
        crate::party_line::line_up_condition_clear(),
        Some(true),
        "lander off the final: the pavement is ours"
    );
    let line = takeoff_hold_tick(&[], 11.0, true, t0 + sec(16)).expect("release");
    assert!(line.contains("cleared for takeoff"), "{line}");
    assert!(*OWN_STRIP.pending_takeoff_clearance.lock().unwrap());
    assert_eq!(
        crate::party_line::line_up_condition_clear(),
        None,
        "released: no condition left to wait on"
    );

    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    *OWN_STRIP.takeoff_hold.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.clearance.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner()) = Vec::new();
}

/// "Wake turbulence — I can never take off." The wake gate is capped at
/// one full separation interval AT THE HOLD, so a field whose departure
/// stream refreshes the clock faster than it expires still lets you go
/// (2026-07-27). But the cap is measured from when the hold was armed,
/// and every new takeoff request re-armed it: ask a second time at
/// Heathrow and you hand your own clock back to zero, for ever.
#[test]
fn asking_again_does_not_restart_the_wake_clock() {
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    let hold = |ident: &str, armed_at: Instant| TakeoffHold {
        callsign: "November one two three".into(),
        runway: "27R".into(),
        runway_ident: ident.into(),
        clear_since: None,
        armed_at,
        logged_at: armed_at,
        lined_up: false,
        behind: false,
        held_for: None,
        cut_in_logged: Default::default(),
    };
    // Nothing standing: the clock starts now.
    assert_eq!(hold_armed_at(None, "27R", t0), t0);
    // Ninety seconds in, the pilot asks again. The hold keeps its stamp,
    // so thirty seconds later the cap has run and the release can come.
    let standing = hold("27R", t0);
    let again = hold_armed_at(Some(&standing), "27R", t0 + sec(90));
    assert_eq!(again, t0, "re-asking must not rewind the separation");
    assert!(
        (t0 + sec(150)).duration_since(again).as_secs() >= WAKE_SECS,
        "the cap must be reachable however often we ask"
    );
    // A hold on the OTHER strip is a different queue: its own clock.
    assert_eq!(
        hold_armed_at(Some(&standing), "09L", t0 + sec(90)),
        t0 + sec(90)
    );
}

#[test]
fn wake_clock_starts_on_an_airline_liftoff_and_blocks_the_release() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
    *ACTIVE_PANEL.lock().unwrap() = Some(("Gibraltar".into(), 11.0, Vec::new()));
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1508156,
        lon_a: -5.3596312,
        lat_b: 36.1514944,
        lon_b: -5.3400082,
    }];
    *WAKE.lock().unwrap() = None;
    *WAKE_PREV.lock().unwrap() = None;
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    let rolling = TcasTarget {
        callsign: "BA306".into(),
        lat: 36.1510,
        lon: -5.3590,
        ele_ft: 11.0,
        on_ground: true,
        vs_fpm: 0.0,
    };
    wake_note_targets(std::slice::from_ref(&rolling), t0);
    assert!(WAKE.lock().unwrap().is_none(), "still rolling: no clock");
    let airborne = TcasTarget {
        ele_ft: 150.0,
        on_ground: false,
        ..rolling
    };
    wake_note_targets(&[airborne], t0 + sec(5));
    let rem = wake_remaining("09", t0 + sec(10)).expect("clock running");
    assert!(rem > 100, "{rem}");
    // Both ends of the same pavement share the wake.
    assert!(wake_remaining("27", t0 + sec(10)).is_some());
    // A strip this field doesn't have: not our wake.
    assert!(wake_remaining("14", t0 + sec(10)).is_none());
    // Two minutes later it has died out.
    assert!(wake_remaining("09", t0 + sec(125)).is_none());

    // A GA tail lifting off starts NO clock — no airline prefix, no
    // assumed weight class.
    *WAKE.lock().unwrap() = None;
    *WAKE_PREV.lock().unwrap() = None;
    let ga = TcasTarget {
        callsign: "N123AB".into(),
        lat: 36.1510,
        lon: -5.3590,
        ele_ft: 11.0,
        on_ground: true,
        vs_fpm: 0.0,
    };
    wake_note_targets(std::slice::from_ref(&ga), t0);
    wake_note_targets(
        &[TcasTarget {
            ele_ft: 150.0,
            on_ground: false,
            ..ga
        }],
        t0 + sec(5),
    );
    assert!(
        WAKE.lock().unwrap().is_none(),
        "GA departure, no wake clock"
    );

    *WAKE_PREV.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

#[test]
fn off_route_taxiing_gets_a_nudge_with_the_via_line() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
    *ACTIVE_PANEL.lock().unwrap() = Some((
        "Gibraltar".into(),
        11.0,
        vec![("tower".to_string(), 131200_i64)],
    ));
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1508156,
        lon_a: -5.3596312,
        lat_b: 36.1514944,
        lon_b: -5.3400082,
    }];
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "November one seven two Sierra Papa".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    // The cleared route: a straight east-west line 1 km north of the
    // runway.
    *OWN_STRIP.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(36.1600, -5.3600), (36.1600, -5.3500)],
        via: "Alpha, Bravo".into(),
        seg: vec![("A".into(), false)],
        holding_point: "A1".into(),
        lead_on: vec![],
    });
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    // Parked on the route: settles the air/ground edge, no call.
    assert!(compliance_tick(36.1600, -5.3600, 15.0, true, 0.0, true, t0).is_none());
    // Taxiing ALONG the route: quiet.
    assert!(compliance_tick(36.1600, -5.3550, 15.0, true, 12.0, true, t0 + sec(1)).is_none());
    // 200 m off it: the nudge, with the via line, from the controller.
    let call = compliance_tick(36.1580, -5.3550, 15.0, true, 12.0, true, t0 + sec(2))
        .expect("off-route nudge");
    assert!(!call.crew);
    assert!(
        call.line.contains("off the cleared taxi route"),
        "{}",
        call.line
    );
    assert!(call.line.contains("Taxi via Alpha, Bravo"), "{}", call.line);
    // Its own cooldown holds — a nudge, not a siren.
    assert!(compliance_tick(36.1580, -5.3550, 15.0, true, 12.0, true, t0 + sec(10)).is_none());

    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

#[test]
fn coach_owes_the_readback_first_and_teaches_the_line_up() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_PANEL.lock().unwrap() = Some(("Gibraltar".into(), 11.0, Vec::new()));
    let cs = "November one seven two Sierra Papa";
    // A readback owed IS the next call — it outranks the state grid.
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TakeoffCleared,
        callsign: cs.into(),
        runway_ident: "09".into(),
        pending_readback: Some(format!("Cleared for takeoff runway zero niner, {cs}.")),
        holds_runway: false,
    });
    let s = suggest_call(cs, true, None, None).expect("a suggestion");
    assert_eq!(
        s.call.as_deref(),
        Some(format!("Cleared for takeoff runway zero niner, {cs}.").as_str())
    );
    assert!(s.urgent, "an owed readback is urgent");
    assert!(s.first.is_none(), "the readback needs no prerequisite");
    // Lined up with the readback paid: the nudge parses as a fresh ask.
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::LinedUp,
        callsign: cs.into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: false,
    });
    let s = suggest_call(cs, true, None, None).expect("a suggestion");
    assert!(!s.urgent);
    let c = s.call.expect("a call");
    assert_eq!(
        tower::parse_intent(&c),
        tower::Intent::RequestTakeoff,
        "{c}"
    );
    *OWN_STRIP.clearance.lock().unwrap() = None;
}

#[test]
fn narrated_takeoff_names_the_departure_end_not_the_nearest() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_RUNWAYS.lock().unwrap() = vec![nav::Runway {
        width_m: 0.0,
        surface: 0,
        ident_a: "09".into(),
        ident_b: "27".into(),
        lat_a: 36.1508156,
        lon_a: -5.3596312,
        lat_b: 36.1514944,
        lon_b: -5.3400082,
    }];
    *GHOSTS.lock().unwrap() = None;
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    // Rolling east on 09, still on the ground: seeds the track memory.
    let rolling = TcasTarget {
        callsign: "BAW123".into(),
        lat: 36.1512,
        lon: -5.3500,
        ele_ft: 11.0,
        on_ground: true,
        vs_fpm: 0.0,
    };
    assert!(chatter_tick(
        std::slice::from_ref(&rolling),
        "Gibraltar",
        36.1512,
        -5.3554,
        11.0,
        None,
        t0
    )
    .is_none());
    // Liftoff PAST midfield, near the 27 threshold: the nearest end is
    // 27, but the roll direction says 09 — the tape read every 09R
    // departure as "two seven left" until this.
    let lifted = TcasTarget {
        lat: 36.1514,
        lon: -5.3410,
        ele_ft: 60.0,
        on_ground: false,
        ..rolling
    };
    let ex = chatter_tick(
        &[lifted],
        "Gibraltar",
        36.1512,
        -5.3554,
        11.0,
        None,
        t0 + sec(1),
    )
    .expect("takeoff narrated");
    let line = &ex.lines[0].text;
    assert!(line.contains("runway zero niner"), "{line}");
    assert!(!line.contains("two seven"), "{line}");
    *GHOSTS.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}

#[test]
fn fresh_station_install_arms_the_zone_grace() {
    // A region handoff says "contact <name> Tower" — the new station's
    // zone watch must give the pilot the cooldown window to actually
    // call, not challenge them as an intruder two seconds later
    // (2026-07-10 tape).
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    lxgb().install();
    assert!(
        COMPLIANCE.lock().unwrap().zone_last.is_some(),
        "check-in grace armed on install"
    );
    *ATIS.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *ACTIVE_PANEL.lock().unwrap() = None;
    *ACTIVE_FIELD.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *CONTROLLER_FREQS.lock().unwrap() = Vec::new();
}

#[test]
fn zone_challenge_on_frequency_is_published_for_the_crew() {
    // Granada's zone watch challenged a full-auto arrival three times
    // while it flew the last enroute leg through the zone, and the crew
    // had no way to hear the question (2026-09-15). The tuned challenge
    // arms the party line; the copilot's own "we're inside the zone"
    // (radio not on the tower) does not, and the flag drains on read.
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let (flat, flon) = (36.1512, -5.3554);
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), flat, flon));
    *ACTIVE_PANEL.lock().unwrap() = Some((
        "Gibraltar".into(),
        11.0,
        vec![("tower".to_string(), 131200_i64)],
    ));
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *OWN_STRIP.zone_challenge.lock().unwrap() = false;
    let t0 = Instant::now();
    // Airborne, 2 nm out, 1,500 ft, tuned to the tower: the controller asks.
    let t = t0 + std::time::Duration::from_secs_f64(ZONE_COOLDOWN_SECS + 1.0);
    let call = compliance_tick(flat + 0.03, flon, 1500.0, false, 100.0, true, t)
        .expect("the zone watch challenges an unknown aircraft");
    assert!(!call.crew, "{}", call.line);
    assert!(
        call.line.contains("Say callsign and intentions"),
        "{}",
        call.line
    );
    assert!(
        crate::party_line::take_zone_challenge(),
        "published for the crew"
    );
    assert!(!crate::party_line::take_zone_challenge(), "drained on read");
    // Radio not on the tower: the copilot's line, no question on the air.
    let t = t + std::time::Duration::from_secs_f64(ZONE_COOLDOWN_SECS + 1.0);
    let call = compliance_tick(flat + 0.03, flon, 1500.0, false, 100.0, false, t)
        .expect("the copilot flags the zone");
    assert!(call.crew, "{}", call.line);
    assert!(
        !crate::party_line::take_zone_challenge(),
        "no challenge was spoken"
    );
    Station::uninstall();
}

/// #83: the answer to a transit field's challenge is a transit request,
/// and it is two-way contact: the tower clears it through and its zone
/// watch asks no more (EDVW-ETND 2026-09-23: challenged on every
/// cooldown, nobody answered).
#[test]
fn a_transit_answers_the_zone_challenge_and_the_watch_stands_down() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let (flat, flon) = (36.1512, -5.3554);
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), flat, flon));
    *ACTIVE_PANEL.lock().unwrap() = Some((
        "Gibraltar".into(),
        11.0,
        vec![("tower".to_string(), 131200_i64)],
    ));
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *OWN_STRIP.zone_challenge.lock().unwrap() = false;
    let t0 = Instant::now();
    let t = t0 + std::time::Duration::from_secs_f64(ZONE_COOLDOWN_SECS + 1.0);
    compliance_tick(flat + 0.03, flon, 1500.0, false, 100.0, true, t)
        .expect("challenged");
    assert!(crate::party_line::take_zone_challenge());
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 8.0,
    };
    let mut st = lxgb();
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "Delta Hotel Xray Alpha Bravo".into(),
        false,
        false,
        None,
        false,
        None,
    );
    let call = "Gibraltar Tower, Delta Hotel Xray Alpha Bravo, helicopter, one thousand five hundred feet, request zone transit";
    let intent = tower::parse_intent(call);
    assert_eq!(intent, tower::Intent::RequestTransit);
    let r = st.respond(call, Service::Tower, intent, &ctx, wx);
    assert!(r.contains("cleared to transit the Gibraltar zone"), "{r}");
    assert!(OWN_STRIP.clearance.lock().unwrap().is_some(), "two-way contact");
    // Past the cooldown, still in the zone: no second challenge.
    let t = t + std::time::Duration::from_secs_f64(ZONE_COOLDOWN_SECS + 1.0);
    assert!(
        compliance_tick(flat + 0.03, flon, 1500.0, false, 100.0, true, t).is_none(),
        "the watch stands down once the transit has called"
    );
    assert!(!crate::party_line::take_zone_challenge());
    *OWN_STRIP.clearance.lock().unwrap() = None;
    Station::uninstall();
}

#[test]
fn uninstall_retires_the_whole_world() {
    // Leaving ATC mode mid-flight must clear everything the crew can
    // see — a stale clearance or taxi route would keep coaching the
    // pilot toward a tower that no longer exists.
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    lxgb().install();
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TakeoffCleared,
        callsign: "November one two three alpha bravo".into(),
        runway_ident: "27".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *OWN_STRIP.landing_handoff.lock().unwrap() = Some("27".into());
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = true;
    Station::uninstall();
    assert!(
        crate::party_line::clearance_state().is_none(),
        "clearance cleared"
    );
    assert!(
        crate::party_line::landing_handoff_runway().is_none(),
        "handoff cleared"
    );
    assert!(
        !crate::party_line::take_pending_takeoff_clearance(),
        "pending drained"
    );
    assert!(ACTIVE_FIELD.lock().unwrap().is_none(), "field retired");
    assert!(ACTIVE_PANEL.lock().unwrap().is_none(), "panel retired");
    assert!(ATIS.lock().unwrap().is_none(), "ATIS off the air");
    assert!(
        CONTROLLER_FREQS.lock().unwrap().is_empty(),
        "no controllers"
    );
}

#[test]
fn taxiways_speak_like_a_controller() {
    assert_eq!(spoken_taxiway("A"), "Alpha");
    assert_eq!(spoken_taxiway("B2"), "Bravo two");
    assert_eq!(spoken_taxiway("A1"), "Alpha one");
    assert_eq!(spoken_taxiway("N4W"), "November four Whiskey");
    assert_eq!(spoken_taxiway("LINK27"), "Link two seven");
    assert_eq!(spoken_taxiway("Link 41"), "Link four one");
    assert_eq!(spoken_taxiway("Inner"), "Inner");
    assert_eq!(first_designator("09L/27R"), "09L");
    assert_eq!(first_designator("14"), "14");
}

#[test]
fn via_lists_collapse_numbered_runs() {
    let names = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
    // Madrid's real 2026-07-13 clearance: "Charlie one, Mike one, Mike
    // two, Mike three, Mike four, Mike five" reads like the database.
    assert_eq!(
        spoken_via(&names(&["C1", "M1", "M2", "M3", "M4", "M5"])),
        vec!["Charlie one", "Mike one to Mike five"]
    );
    // Descending runs collapse the same way (taxiing the other way).
    assert_eq!(
        spoken_via(&names(&["M5", "M4", "M3", "B"])),
        vec!["Mike five to Mike three", "Bravo"]
    );
    // Two in a row is not a run — say both.
    assert_eq!(
        spoken_via(&names(&["M1", "M2", "A"])),
        vec!["Mike one", "Mike two", "Alpha"]
    );
    // A gap breaks the run; a different designator breaks the run.
    assert_eq!(
        spoken_via(&names(&["M1", "M3", "M4", "N1"])),
        vec!["Mike one", "Mike three", "Mike four", "November one"]
    );
    // Unnumbered names pass straight through.
    assert_eq!(
        spoken_via(&names(&["L", "B", "G11", "A", "HN1"])),
        vec![
            "Lima",
            "Bravo",
            "Golf one one",
            "Alpha",
            "Hotel November one"
        ]
    );
}

/// The world sweep: EVERY towered airport in the real Global Airports
/// file stands up a station, briefs its frequencies, and flies the
/// departure happy path — the shakeout for apt.dat shapes LXGB never
/// showed us. Opt-in because the file ships with X-Plane, not the repo:
///
///   cargo test --release -p atc -- --ignored atc_world_sweep
///
/// Override the file with XCOPILOT_APT_DAT.
#[test]
#[ignore = "needs X-Plane's Global Airports apt.dat; run with -- --ignored"]
fn atc_world_sweep_every_towered_field_stands_up() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = std::env::var("XCOPILOT_APT_DAT").unwrap_or_else(|_| {
        crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat")
    });
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    let (mut total, mut towered, mut no_runways) = (0u32, 0u32, 0u32);
    let mut fails: Vec<String> = Vec::new();
    nav::scan_airports(&path, |icao, apt| {
        total += 1;
        let Some(mut st) = Station::from_airport(apt) else {
            return;
        };
        towered += 1;
        let mut check = |cond: bool, what: &str| {
            if !cond && fails.len() < 40 {
                fails.push(format!("{icao}: {what}"));
            }
        };
        check(st.covers_khz(st.tower_khz), "tower frequency not covered");

        // The briefing must read every published service, and name the
        // tower when COM1 sits on it.
        let brief = st.briefing_text(Some(st.tower_khz));
        check(
            brief.contains("tower"),
            &format!("no tower in briefing: {brief}"),
        );
        check(
            brief.ends_with("Com one is on the tower."),
            &format!("tuned service unnamed: {brief}"),
        );

        // The departure happy path, spoken with live-shaped context.
        let ctx = scripted_ctx(&mut st, wx);
        if ctx.runway.is_empty() {
            no_runways += 1; // towered heliports/seaplane bases: no 100 rows
        }
        let r = st.route_keyed("radio check", Service::Tower, &ctx, wx);
        check(r.contains("reading you five"), &format!("radio check: {r}"));
        let gctx = tower::Ctx {
            on_ground: true,
            ..ctx.clone()
        };
        // The ground positions where the field publishes them (apt.dat
        // 1052/1053): each answers in its own name, a call on the wrong
        // one is sent on by name and frequency, and the taxi is Ground's.
        let gnd = st.ground_service();
        if gnd == Service::Ground {
            let name = st.service_name(Service::Ground);
            let r = st.route_keyed("radio check", Service::Ground, &gctx, wx);
            check(r.contains(&format!("{name}, reading you five")), &format!("ground radio check: {r}"));
            let r = st.route_keyed("request taxi", Service::Tower, &gctx, wx);
            check(
                r.contains(&format!("contact {name} on ")) && !r.contains("holding point"),
                &format!("taxi on the tower: {r}"),
            );
        }
        if st.delivery_khz.is_some() {
            let r = st.route_keyed("request taxi", Service::Delivery, &gctx, wx);
            check(r.contains("contact "), &format!("taxi on delivery: {r}"));
        }
        let r = st.route_keyed("request taxi", gnd, &gctx, wx);
        check(r.contains("taxi to holding point"), &format!("taxi: {r}"));
        if gnd == Service::Ground {
            let r = st.route_keyed("ready for departure", Service::Ground, &gctx, wx);
            check(
                r.contains(&format!("contact {} on ", st.service_name(Service::Tower))),
                &format!("ready on ground: {r}"),
            );
        }
        let r = st.route_keyed("ready for departure", Service::Tower, &gctx, wx);
        check(r.contains("cleared for takeoff"), &format!("takeoff: {r}"));
        if !ctx.runway.is_empty() {
            check(
                r.contains(&ctx.runway),
                &format!("clearance names the wrong runway: {r}"),
            );
        }

        // Vectors exercise the runway geometry math per field.
        let r = st.route_keyed("request vectors", Service::Approach, &ctx, wx);
        check(!r.is_empty(), "vectors: empty reply");

        // Statics must not leak between airports (or into other tests).
        *OWN_STRIP.position_handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *OWN_STRIP.working.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *OWN_STRIP.handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
        *OWN_STRIP.vectors.lock().unwrap_or_else(|e| e.into_inner()) = None;
    })
    .expect("apt.dat scan");
    println!("swept {total} airports, {towered} towered ({no_runways} with no runway rows)");
    assert!(towered > 100, "suspiciously few towered fields: {towered}");
    assert!(
        fails.is_empty(),
        "{} fields failed (first {}):\n{}",
        fails.len(),
        fails.len().min(40),
        fails.join("\n")
    );
}

/// LEMG 31 (sortie 1790584715-full_auto, 2026-09-28): a right-hand
/// circuit heard "contact Malaga Departure on one two one decimal zero
/// two five" on the crosswind leg at 772 ft, and "radar contact" in the
/// tower's voice 0.4 s after rotation from a Departure watch a zone
/// clearance had armed. A flight that says it is a circuit stays with
/// the tower: no handoff, no radar watch. The next departure that does
/// not say so is handed off as before.
#[test]
fn radar_contact_is_the_radar_units_never_the_towers() {
    // LEMG 31 (sortie 1790584715, 2026-09-28, t 1790585265.4, route
    // squawk_watch): the zone clearance's Departure watch said "radar
    // contact" in the TOWER's voice 4.3 s after rotation, and Departure
    // said it again at the check-in.
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let ctx = scripted_ctx(&mut st, wx);
    *OWN_STRIP.handoff.lock().unwrap() = None;
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.squawk_nudge.lock().unwrap() = None;
    let _ = crate::party_line::take_departure_handoff();

    let r = st.route_keyed("request clearance", Service::Tower, &ctx, wx);
    assert!(r.contains("squawk"), "{r}");
    let code = st.desk.assigned_squawk.expect("code issued");
    let t0 = Instant::now();
    // Airborne on the tower, the code squawking: the tower says nothing,
    // and the watch still waits for the radar unit.
    assert!(squawk_tick(Some(code), true, StationSpeaker::Tower, t0).is_none());
    assert!(squawk_watch_active(), "held, not spent");
    // A wrong code is still relayed: the nudge is an instruction.
    let late = t0 + std::time::Duration::from_secs(25);
    let (line, who) =
        squawk_tick(Some(7000), true, StationSpeaker::Tower, late).expect("relayed nudge");
    assert!(line.contains("confirm squawk"), "{line}");
    assert!(matches!(who, StationSpeaker::Departure), "the assigner's");
    let _ = crate::party_line::take_squawk_nudge();

    // The takeoff clearance arms the handoff: "radar contact" is now the
    // check-in's, even tuned to Departure before the call goes out.
    let r = st.route_keyed("request taxi", Service::Tower, &ctx, wx);
    assert!(r.contains("taxi"), "{r}");
    let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared for takeoff"), "{r}");
    let later = late + std::time::Duration::from_secs(60);
    assert!(squawk_tick(Some(code), true, StationSpeaker::Tower, later).is_none());
    assert!(squawk_tick(Some(code), true, StationSpeaker::Departure, later).is_none());
    let r = st.route_keyed(
        "passing one thousand five hundred climbing",
        Service::Departure,
        &ctx,
        wx,
    );
    assert!(r.contains("Departure, radar contact"), "{r}");
    // Said once: the watch is satisfied by the check-in.
    assert!(!squawk_watch_active(), "the check-in satisfied the watch");
    assert!(squawk_tick(Some(code), true, StationSpeaker::Departure, later).is_none());

    // A code issued by the radar unit on its own frequency (flight
    // following, no handoff): the watch says it there.
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    st.desk.assigned_squawk = None;
    st.assign_squawk("November one two three", StationSpeaker::Departure);
    let code = st.desk.assigned_squawk.expect("code issued");
    let (line, who) = squawk_tick(Some(code), true, StationSpeaker::Departure, t0)
        .expect("radar contact on Departure");
    assert!(line.contains("radar contact"), "{line}");
    assert!(matches!(who, StationSpeaker::Departure));
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.handoff.lock().unwrap() = None;
    let _ = crate::party_line::take_departure_handoff();
}

/// LEMG 13 (sorties 1790590663 and 1790591256, 2026-09-28): the runway
/// start's "request clearance, runway one three, for circuits" was
/// answered "cleared to leave the Malaga zone, not above two thousand
/// feet". Circuit traffic gets the circuit clearance in the field's
/// dialect (runway, height, pressure, code, report ready), and the crew's
/// readback of it, the owed line as the crew transmits it, is silence.
/// #182: the field's own data (apt.dat `1302 allows_circuits 0`, the
/// runway's `1101` side) reaches the circuit clearance: the tower says a
/// field without circuits first, names the published side, and the
/// crew's readback of it is still silence.
#[test]
fn a_circuit_clearance_carries_the_fields_circuit_data() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.apt.allows_circuits = Some(false);
    st.apt.pattern_sides = vec![("27".into(), Some(false))];
    let ctx = scripted_ctx(&mut st, wx);
    let ask = "november one seven two sierra papa, request clearance, runway two seven, \
             for circuits";
    // Declined the first time, with nothing cleared and no code given
    // (it was "circuits are not normally available" and the height, the
    // code and "report ready" in the same breath, 2026-10-09).
    let offer = st.circuit_offer("two seven");
    assert!(offer.not_available && offer.left == Some(false));
    let r = st.route_keyed(ask, Service::Tower, &ctx, wx);
    assert_eq!(
        r,
        "November one seven two Sierra Papa, circuits are not available at Gibraltar. \
         Departure from the zone or a VFR route available, say intentions."
    );
    assert_eq!(st.desk.assigned_squawk, None, "a declined circuit gets no code");
    assert_eq!(crate::party_line::readback_owed(), None);
    // Asked again, the tower uses its discretion: cleared, plainly.
    let r = st.route_keyed(ask, Service::Tower, &ctx, wx);
    assert!(
        r.contains("runway two seven, right hand circuit, circuit height"),
        "{r}"
    );
    assert!(!r.contains("not available"), "{r}");
    let owed = crate::party_line::readback_owed().expect("readback suggested");
    assert!(owed.contains("right hand circuit"), "{owed}");
    let r = st.route_keyed(owed.trim_end_matches('.'), Service::Tower, &ctx, wx);
    assert_eq!(r, "", "the readback is accepted");
    // Silent data: the clearance as before.
    st.apt.allows_circuits = None;
    st.apt.pattern_sides.clear();
    let offer = st.circuit_offer("two seven");
    assert!(!offer.not_available && offer.left.is_none());
}

#[test]
fn a_circuit_zone_request_is_cleared_for_the_circuit() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    for faa in [false, true] {
        *OWN_STRIP.squawk_watch.lock().unwrap() = None;
        *OWN_STRIP.clearance.lock().unwrap() = None;
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        if faa {
            st.set_dialect(Dialect::Faa);
        }
        let ctx = scripted_ctx(&mut st, wx);
        let r = st.route_keyed(
            "november one seven two sierra papa, request clearance, runway two seven, \
                 for circuits",
            Service::Tower,
            &ctx,
            wx,
        );
        assert!(st.desk.circuits, "circuits noted from the zone request");
        assert!(!r.contains("leave the"), "{r}");
        assert!(r.contains("runway two seven"), "{r}");
        assert!(r.contains("squawk"), "{r}");
        assert!(r.ends_with("report ready for departure."), "{r}");
        if faa {
            assert!(
                r.contains("closed traffic approved, pattern altitude one thousand"),
                "{r}"
            );
        } else {
            assert!(r.contains("circuit height one thousand feet"), "{r}");
        }
        assert!(!squawk_watch_active(), "no radar watch for pattern traffic");
        let owed = crate::party_line::readback_owed().expect("readback suggested");
        assert!(!owed.contains("above"), "{owed}");
        // The crew transmits the owed line, stop trimmed: silence, paid.
        let r = st.route_keyed(owed.trim_end_matches('.'), Service::Tower, &ctx, wx);
        assert_eq!(r, "", "the readback is accepted");
        assert_eq!(crate::party_line::readback_owed(), None);
        assert_eq!(st.desk.twr.state, tower::State::Parked);
        // A plain zone request states the intent afresh: a departure.
        let r = st.route_keyed("request clearance", Service::Tower, &ctx, wx);
        assert!(!st.desk.circuits);
        assert!(r.contains("cleared to leave the Gibraltar zone"), "{r}");
    }
    // From the ramp no runway is named (full auto's zone_clearance_call,
    // LEMG sorties 1790605551 and 1790618416): still the circuit.
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let ctx = scripted_ctx(&mut st, wx);
    let r = st.route_keyed(
        "november one seven two sierra papa, request clearance, for circuits",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(st.desk.circuits, "circuits noted from the ramp's zone request");
    assert!(!r.contains("leave the"), "{r}");
    assert!(r.contains("circuit height"), "{r}");
    assert!(r.ends_with("report ready for departure."), "{r}");
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.handoff.lock().unwrap() = None;
}

#[test]
fn circuit_traffic_stays_with_the_tower() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    assert_eq!(st.departure_khz, Some(130400), "a field with a Departure");
    let ctx = scripted_ctx(&mut st, wx);
    *OWN_STRIP.handoff.lock().unwrap() = None;
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    let _ = crate::party_line::take_departure_handoff();

    // The zone clearance arms the radar watch (the LEMG leftover).
    let r = st.route_keyed("request clearance", Service::Tower, &ctx, wx);
    assert!(r.contains("squawk"), "{r}");
    assert!(squawk_watch_active(), "zone clearance armed the watch");

    // The circuit says so on the taxi request: the watch goes.
    let r = st.route_keyed(
        "november one seven two sierra papa, request taxi for circuits",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("taxi to holding point"), "{r}");
    assert!(st.desk.circuits, "circuits noted");
    assert!(!squawk_watch_active(), "no radar watch for pattern traffic");
    // A plain "ready" does not undo it.
    let r = st.route_keyed(
        "november one seven two sierra papa, ready for departure",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("cleared for takeoff"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::TakeoffCleared);
    assert!(
        handoff_tick(900.0, false).is_none(),
        "the tower keeps a circuit"
    );
    assert!(handoff_tick(2500.0, false).is_none(), "at any height");
    assert!(
        crate::party_line::take_departure_handoff().is_none(),
        "nothing published for the crew"
    );
    // The hold-release path arms through the same door.
    st.arm_departure_handoff(&ctx);
    assert!(
        handoff_tick(900.0, false).is_none(),
        "released circuit kept too"
    );

    // The pattern calls still work the tower.
    let r = st.route_keyed(
        "november one seven two sierra papa, downwind runway two seven",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("report base"), "{r}");

    // A new departure that says nothing of circuits is a departure.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let ctx = scripted_ctx(&mut st, wx);
    st.route_keyed("request taxi for circuits", Service::Tower, &ctx, wx);
    assert!(st.desk.circuits);
    st.route_keyed("request taxi", Service::Tower, &ctx, wx);
    assert!(!st.desk.circuits, "a taxi request states the intent afresh");
    let r = st.route_keyed("ready for departure", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared for takeoff"), "{r}");
    let line = handoff_tick(900.0, false).expect("a departure is handed off");
    assert!(
        line.contains("Departure on one three zero decimal four"),
        "{line}"
    );
    let _ = crate::party_line::take_departure_handoff();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
}

#[test]
fn takeoff_clearance_arms_the_departure_handoff() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    assert_eq!(st.departure_khz, Some(130400), "departure row wired");
    let ctx = tower::Ctx {
        callsign: "November one seven two Sierra Papa".into(),
        airport: "Gibraltar".into(),
        runway: "two seven".into(),
        qnh: "QNH one zero one five".into(),
        wind: "calm".into(),
        holding_point: "Alpha".into(),
        stand: String::new(),
        approach: Some(atis::spoken_mhz(122800)),
        squawk: "seven zero zero zero".into(),
        final_traffic_nm: None,
        final_traffic_callsign: None,
        final_traffic_count: 0,
        final_traffic_id: None,
        sequence_ahead: None,
        runway_occupied: false,
        own_on_runway: false,
        helicopter: false,
        ifr: None,
        circuit: None,
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
    };
    // Walk the tower to a takeoff clearance and arm, exactly as respond()
    // does around twr.handle().
    st.desk.twr.handle(tower::parse_intent("request taxi"), &ctx);
    let was = st.desk.twr.state;
    st.desk.twr
        .handle(tower::parse_intent("ready for departure"), &ctx);
    assert_ne!(was, st.desk.twr.state);
    st.arm_departure_handoff(&ctx);

    // On the roll: silence. Climbing through field + 700: the handoff,
    // exactly once.
    assert!(handoff_tick(11.0, true).is_none());
    assert!(handoff_tick(300.0, false).is_none());
    let line = handoff_tick(900.0, false).expect("handoff due");
    assert!(
        line.contains("contact Gibraltar Departure on one three zero decimal four"),
        "{line}"
    );
    assert!(handoff_tick(1500.0, false).is_none(), "spoken once");
}

#[test]
fn named_station_tuning_resolves_from_apt_dat() {
    let st = lxgb();
    match st.tune_request("tune the tower") {
        Some(Tune::Found { label, khz }) => {
            assert_eq!(khz, 131200);
            assert_eq!(label, "Gibraltar tower");
        }
        other => panic!("expected tower: {:?}", other.is_some()),
    }
    // Generic "atc"/"station" means the tower.
    assert!(matches!(
        st.tune_request("tune to atc"),
        Some(Tune::Found { khz: 131200, .. })
    ));
    assert!(matches!(
        st.tune_request("contact approach"),
        Some(Tune::Found { khz: 122800, .. })
    ));
    // A named station the field doesn't have declines by name, no guessing.
    match st.tune_request("tune the atis") {
        Some(Tune::Missing(msg)) => assert!(msg.contains("no atis"), "{msg}"),
        other => panic!("expected Missing: {:?}", other.is_some()),
    }
    // No tuning verb → not a tuning request, even if a station is named
    // ("gibraltar tower request taxi" must reach the tower, not the tuner).
    assert!(st.tune_request("gibraltar tower request taxi").is_none());
    // A verb without any station name is someone else's command.
    assert!(st
        .tune_request("tune com one to one one eight decimal five")
        .is_none());
    // "change frequency to <station>" — how the 2026-07-08 pilot actually
    // asked, four phrasings in a row, all missed.
    assert!(matches!(
        st.tune_request("let's change frequency to gibraltar tower"),
        Some(Tune::Found { khz: 131200, .. })
    ));
    assert!(matches!(
        st.tune_request("change frequency to tower"),
        Some(Tune::Found { khz: 131200, .. })
    ));
    // "switch frequency to tower." (2026-07-09 flight, missed: "switch"
    // only counted when followed directly by "to").
    assert!(matches!(
        st.tune_request("switch frequency to tower."),
        Some(Tune::Found { khz: 131200, .. })
    ));
    // "change"/"switch" without radio context must not retune.
    assert!(st
        .tune_request("change approach speed to seventy")
        .is_none());
    assert!(st.tune_request("switch the landing light on").is_none());
    // "change to approach" tunes (2026-07-08 night flight: answered with
    // "approach around 65 knots" instead) — but flight-parameter talk
    // naming a station never does.
    assert!(matches!(
        st.tune_request("change to approach"),
        Some(Tune::Found { khz: 122800, .. })
    ));
    assert!(st.tune_request("change to approach speed").is_none());
    assert!(st.tune_request("select approach flaps").is_none());
    // STT warts, all live 2026-07-20: ATIS spelled out or heard as
    // "atheist", tower as "tau", and the verb clipped to a short bare
    // "<station> frequency".
    match st.tune_request("change to a t i s frequency.") {
        Some(Tune::Missing(msg)) => assert!(msg.contains("no atis"), "{msg}"),
        other => panic!("spelled-out atis should resolve: {:?}", other.is_some()),
    }
    match st.tune_request("can the vibrator change to atheist frequency") {
        Some(Tune::Missing(msg)) => assert!(msg.contains("no atis"), "{msg}"),
        other => panic!("'atheist' should resolve to atis: {:?}", other.is_some()),
    }
    assert!(matches!(
        st.tune_request("change to tau frequency."),
        Some(Tune::Found { khz: 131200, .. })
    ));
    assert!(matches!(
        st.tune_request("to tower frequency."),
        Some(Tune::Found { khz: 131200, .. })
    ));
    // Long frequency-quoting readbacks still never retune bare.
    assert!(st
        .tune_request("holding point echo one runway two seven frequency copied")
        .is_none());
}

#[test]
fn freq_query_matches_questions_not_commands() {
    assert!(freq_query("what frequencies are here"));
    assert!(freq_query("which stations are in range"));
    assert!(freq_query("list the frequencies"));
    assert!(freq_query("who can i talk to"));
    assert!(freq_query("who can i contact"));
    // Commands and radio calls must fall through to their own paths.
    assert!(!freq_query("set frequency to one one eight decimal five"));
    assert!(!freq_query("tune the tower"));
    assert!(!freq_query("request taxi"));
    assert!(!freq_query("what's the qnh"));
}

#[test]
fn briefing_reads_lxgb_services_and_com1() {
    let path = lxgb_apt_file();
    let st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    // Everything the field publishes, in briefing order, one sentence.
    let b = st.briefing_text(Some(131200));
    assert!(
        b.starts_with(
            "Gibraltar: tower one three one decimal two, \
                 approach one two two decimal eight, \
                 departure one three zero decimal four."
        ),
        "{b}"
    );
    assert!(b.ends_with("Com one is on the tower."), "{b}");
    // Tuned off every service: the raw frequency is read back instead.
    let b = st.briefing_text(Some(118000));
    assert!(
        b.ends_with("Com one is on one one eight decimal zero."),
        "{b}"
    );
    // A failed COM1 read drops that sentence, never a wrong number.
    let b = st.briefing_text(None);
    assert!(b.ends_with("decimal four."), "{b}");
    // Ground and tower on the SAME frequency (8 real fields in the
    // 2026-07 apt.dat): the briefing names the service that answers —
    // the tower — not the first row in briefing order.
    let mut st = lxgb();
    st.apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_GROUND,
        khz: 131200,
        name: "Ground".into(),
    });
    let b = st.briefing_text(Some(131200));
    assert!(b.ends_with("Com one is on the tower."), "{b}");
}

#[test]
fn void_note_points_at_who_answers_not_the_directory() {
    let mut st = lxgb();
    st.apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_GROUND,
        khz: 121800,
        name: "Ground".into(),
    });
    st.apt.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_ATIS,
        khz: 136525,
        name: "ATIS".into(),
    });
    // On the ground tuned to the published but unmanned ground row:
    // name it and point at the tower — never the whole directory.
    let n = st.void_note(Some(121800), false);
    assert!(n.contains("Nobody's manning ground"), "{n}");
    assert!(n.contains("tower, one three one decimal two"), "{n}");
    assert!(!n.contains("departure") && !n.contains("clearance"), "{n}");
    // Transmitting on the ATIS: it's a broadcast, say so.
    let n = st.void_note(Some(136525), false);
    assert!(n.contains("only broadcasts"), "{n}");
    // Off every published row: just the pointer.
    let n = st.void_note(Some(118000), false);
    assert!(n.starts_with("Try Gibraltar tower on"), "{n}");
    // Airborne with an approach published, approach is the one to call.
    let n = st.void_note(Some(118000), true);
    assert!(n.contains("approach"), "{n}");
    // A failed COM1 read still points somewhere useful.
    let n = st.void_note(None, false);
    assert!(n.contains("tower"), "{n}");
}

/// A fixture apt.dat with LXGB's rows verbatim from the Global Airports
/// file (2026-07 cycle), plus a trailing airport to prove the scan stops.
/// Written exactly once: several tests read it concurrently, and a
/// per-test `File::create` truncates it under a parallel reader.
fn lxgb_apt_file() -> std::path::PathBuf {
    use std::io::Write;
    use std::sync::OnceLock;
    static PATH: OnceLock<std::path::PathBuf> = OnceLock::new();
    PATH.get_or_init(|| {
        let path = std::env::temp_dir()
            .join(format!("xc-station-lxgb-{}.dat", std::process::id()));
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "1     11 0 0 LXGB Gibraltar").unwrap();
        writeln!(f, "100 45.11 1 0 0.20 0 3 0  09 36.1508156 -5.3596312 136 0 5 1 0 3 27 36.1514944 -5.3400082 98 0 5 7 0 3").unwrap();
        writeln!(f, "1054 131200 Tower Freq").unwrap();
        writeln!(f, "1055 122800 Approach Freq").unwrap();
        writeln!(f, "1056 130400 Departure Freq").unwrap();
        writeln!(f, "1   1027 0 0 0MU1 Sunderland").unwrap();
        f.flush().unwrap();
        path
    })
    .clone()
}

/// EGKK as the real install's apt.dat has it (2026-09-27): the header,
/// both 100-rows verbatim (08R/26L 3,319 m with approach and centreline
/// lights; 08L/26R 2,563 m, the standby) and the tower.
fn egkk_apt_file() -> std::path::PathBuf {
    use std::io::Write;
    use std::sync::OnceLock;
    static PATH: OnceLock<std::path::PathBuf> = OnceLock::new();
    PATH.get_or_init(|| {
        let path = std::env::temp_dir()
            .join(format!("xc-station-egkk-{}.dat", std::process::id()));
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "1    203 0 0 EGKK London Gatwick").unwrap();
        writeln!(f, "100 46.00 26 1527 0.25 1 3 0 08R 51.1450993 -0.2123748 395 0 7 4 1 3 26L 51.1514994 -0.1659296 425 30 7 4 1 3").unwrap();
        writeln!(f, "100 46.00 23 853 0.25 0 1 0 08L 51.1468901 -0.2126029 321 0 7 0 0 3 26R 51.1518321 -0.1767408 415 0 7 0 0 3").unwrap();
        writeln!(f, "1052 121955 Gatwick Delivery").unwrap();
        writeln!(f, "1054 124230 Gatwick Tower").unwrap();
        writeln!(f, "1055 126825 Gatwick Director").unwrap();
        writeln!(f, "1   1027 0 0 0MU1 Sunderland").unwrap();
        f.flush().unwrap();
        path
    })
    .clone()
}

/// EGKK 2026-09-27 (sortie 1790524628): the IFR clearance at +154 named
/// 26R with LAM6V, and the taxi at +170 and the takeoff went to 26L. The
/// wind was 175 at 7.7: across both parallels, whose headwinds differ by
/// rounding, so the pick was noise. Then the crew's "request taxi, runway
/// two six left" moved the runway in use under the clearance just read
/// back. Now: one answer. The main runway (26L) is picked, a stale 26R
/// moves to it, the SID is 26L's, and the clearance freezes the runway
/// the taxi goes to.
#[test]
fn the_ifr_clearance_and_the_taxi_name_one_runway_at_egkk() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    Station::uninstall();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    let dir = std::env::temp_dir().join(format!("xc-station-egkk-sids-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    // Two SIDs to Lambourne as the CIFP has them: M off 26L, V off 26R.
    let leg = |sid: &str, rwy: &str| {
        format!(
            "SID:010,5,{sid},{rwy},LAMXX,EG,E,A,E   , ,   ,TF, ,   ,  , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   ;\n"
        )
    };
    std::fs::write(
        dir.join("EGKK.dat"),
        [leg("LAM6V", "RW26R"), leg("LAM6M", "RW26L")].concat(),
    )
    .unwrap();
    let fixes = dir.join("earth_fix.dat");
    std::fs::write(
        &fixes,
        "I\n1200 Version\n\n 51.646 0.152 LAMXX ENRT EG 2\n99\n",
    )
    .unwrap();
    let mut egbb = region_apt("Birmingham", 52.4539, -1.748);
    egbb.icao = "EGBB".into();
    *REGION.lock().unwrap() = vec![egbb];
    let wx = Wx {
        wind_from_deg: 174.7,
        wind_kts: 7.7,
    };
    let path = egkk_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "EGKK").expect("EGKK loads");
    st.set_procedures_now(dir.to_str().unwrap(), fixes.to_str().unwrap(), "", "EGKK");
    let cs = "Whiskey Alpha Tango one seven three";
    let ctx_now = |st: &mut Station| {
        st.make_ctx(
            wx,
            "QNH one zero one three".into(),
            cs.into(),
            true,
            false,
            None,
            false,
            None,
        )
    };

    // The pick: the main runway of the pair, whichever parallel the
    // wind favours by a hundredth of a knot.
    assert_eq!(st.pick_runway(174.7, 7.7), "26L");
    assert_eq!(
        st.pick_runway(354.7, 7.7),
        "26L",
        "hysteresis: no reversal in 7.7 kt of crosswind"
    );
    // A pick left on the standby (by the old rule, or a calm stream)
    // moves to the main runway while nothing is committed.
    st.runway_in_use = Some("26R".into());
    assert_eq!(st.pick_runway(174.7, 7.7), "26L");

    // The clearance names 26L and 26L's SID.
    let ctx = ctx_now(&mut st);
    // EGKK's clearance is Gatwick Delivery's (apt.dat 1052): asked of the
    // Tower, the pilot is sent there.
    let r = st.route_keyed("request ifr clearance to egbb", Service::Tower, &ctx, wx);
    assert!(r.contains("contact Gatwick Delivery on one two one decimal niner five five"), "{r}");
    let r = st.route_keyed("request ifr clearance to egbb", Service::Delivery, &ctx, wx);
    assert!(r.contains("runway two six left"), "{r}");
    assert!(r.contains("Mike departure"), "LAM6M off 26L: {r}");
    assert!(!r.contains("Victor"), "{r}");
    let r = st.route_keyed(
        &format!("cleared to birmingham via the lam six mike departure, runway two six left, climb five thousand feet, squawk {}, {cs}", atis::spoken_digits(&format!("{:04}", st.desk.assigned_squawk.unwrap()))),
        Service::Delivery,
        &ctx,
        wx,
    );
    assert_eq!(r, "", "read back");

    // The crew states another runway with its taxi request: the
    // clearance holds the runway the SID was chosen for.
    let taxi = "whiskey alpha tango one seven three, request taxi, runway two six right";
    st.note_stated_runway(taxi, tower::parse_intent(taxi));
    let ctx = ctx_now(&mut st);
    assert_eq!(ctx.runway, "two six left", "the clearance's runway stands");
    let r = st.route_keyed(taxi, Service::Tower, &ctx, wx);
    assert!(r.contains("runway two six left"), "{r}");

    // Airborne, the freeze is spent.
    st.desk.ifr_runway = None;
    *REGION.lock().unwrap() = Vec::new();
    *OWN_STRIP.ifr_clearance.lock().unwrap() = None;
    *OWN_STRIP.ifr_flight.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    Station::uninstall();
}

/// The overheard traffic at +99 of the same sortie was "cleared for
/// takeoff runway two six right": a replayed ghost 200 m off 26L's
/// centreline is on 26L. A close parallel names its main runway; a wide
/// one (Heathrow's 1,400 m) is itself.
#[test]
fn a_close_parallel_is_its_main_runway() {
    let st = Station::load(egkk_apt_file().to_str().unwrap(), "EGKK").expect("EGKK loads");
    let rw = &st.apt.runways;
    assert_eq!(nav::close_parallel_main(rw, "26R"), "26L");
    assert_eq!(nav::close_parallel_main(rw, "08L"), "08R");
    assert_eq!(nav::close_parallel_main(rw, "26L"), "26L");
    assert_eq!(
        nav::close_parallel_main(rw, "33"),
        "33",
        "unknown end: itself"
    );
    let egll = vec![
        nav::Runway {
            width_m: 50.0,
            surface: 0,
            ident_a: "09L".into(),
            ident_b: "27R".into(),
            lat_a: 51.4774781,
            lon_a: -0.4894423,
            lat_b: 51.4776753,
            lon_b: -0.4332600,
        },
        nav::Runway {
            width_m: 50.0,
            surface: 0,
            ident_a: "09R".into(),
            ident_b: "27L".into(),
            lat_a: 51.4647701,
            lon_a: -0.4867489,
            lat_b: 51.4649460,
            lon_b: -0.4340638,
        },
    ];
    assert_eq!(nav::close_parallel_main(&egll, "27L"), "27L");
    assert_eq!(nav::close_parallel_main(&egll, "09R"), "09R");
}

/// A CIFP directory with one SID from Gibraltar's runway 09.
fn lxgb_cifp_dir() -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("xc-station-cifp-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(
        dir.join("LXGB.dat"),
        "SID:010,5,GOSA1C,RW09,GOS01,LX,P,C,E   , ,   ,CF, ,   ,  , , ,      ,    ,    ,0900,0050, ,     ,     ,     , ,   ,    ,   ;\n",
    )
    .unwrap();
    dir
}

fn edinburgh() -> RegionApt {
    RegionApt {
        icao: "EGPH".into(),
        apt: nav::AirportInfo {
            transition_level_ft: None,
            allows_circuits: None,
            pattern_sides: Vec::new(),
            papis: Vec::new(),
            name: "Edinburgh".into(),
            elev_ft: 136.0,
            runways: Vec::new(),
            freqs: Vec::new(),
        },
        lat: 55.95,
        lon: -3.37,
    }
}

#[test]
fn an_ifr_clearance_is_published_only_after_its_readback() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    Station::uninstall();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *REGION.lock().unwrap() = vec![edinburgh()];
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 10.0,
    };
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.set_procedures_now(lxgb_cifp_dir().to_str().unwrap(), "", "", "LXGB");
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        true,
        false,
        None,
        false,
        None,
    );
    // A destination the index does not know: asked again, nothing issued.
    let r = st.route_keyed("gibraltar tower, request ifr clearance to atlantis", Service::Tower, &ctx, wx);
    assert_eq!(r, "November one seven two Sierra Papa, say again destination.");
    assert!(st.desk.assigned_squawk.is_none(), "no clearance, no code");
    // Issued: the SID from the CIFP for the runway in use, 5,000 ft, the
    // radar field's discrete code. Not yet published.
    let r = st.route_keyed(
        "gibraltar tower, november one seven two sierra papa, request ifr clearance to edinburgh",
        Service::Tower,
        &ctx,
        wx,
    );
    let code = st.desk.assigned_squawk.expect("a discrete code at a radar field");
    let spoken = atis::spoken_digits(&format!("{code:04}"));
    assert_eq!(
        r,
        format!(
            "November one seven two Sierra Papa, cleared to Edinburgh via the Gosa one Charlie \
                 departure, runway zero niner, climb five thousand feet, squawk {spoken}, QNH one \
                 zero one seven."
        )
    );
    assert_eq!(crate::party_line::assigned_clearance(), None, "issued is not read back");
    assert_eq!(crate::party_line::assigned_squawk(), Some(code));
    // A wrong altitude is corrected, and still nothing is published.
    let r = st.route_keyed(
        &format!("cleared to edinburgh via the gosa one charlie departure, climb six thousand feet, squawk {spoken}"),
        Service::Tower,
        &ctx,
        wx,
    );
    assert_eq!(r, "November one seven two Sierra Papa, negative, climb five thousand feet.");
    assert_eq!(crate::party_line::assigned_clearance(), None);
    // The coach's line, read back: silence, and the clearance is the crew's.
    let rb = crate::party_line::readback_owed().expect("the coach owes the readback");
    assert!(rb.starts_with("Cleared to Edinburgh via the Gosa one Charlie departure"), "{rb}");
    let r = st.route_keyed(&rb.to_lowercase(), Service::Tower, &ctx, wx);
    assert_eq!(r, "", "a correct readback earns silence");
    assert_eq!(
        crate::party_line::assigned_clearance(),
        Some(tower::IfrClearance {
            dest: "EGPH".into(),
            sid: Some("GOSA1C".into()),
            runway: "09".into(),
            initial_alt_ft: 5000,
            squawk: code,
        })
    );
    assert_eq!(crate::party_line::readback_owed(), None, "the debt is paid");
    assert_eq!(crate::party_line::ifr_flight_code(), Some(code), "the flight is IFR");
    // Retired with the squawk: a new field, and the world going away.
    st.install();
    assert_eq!(crate::party_line::assigned_clearance(), None, "install retires it");
    // The FLIGHT stays IFR across a region's install: the handoff that
    // follows it must keep its code.
    assert_eq!(crate::party_line::ifr_flight_code(), Some(code), "install keeps the IFR flight");
    *OWN_STRIP.ifr_clearance.lock().unwrap() = Some(tower::IfrClearance {
        dest: "EGPH".into(),
        sid: None,
        runway: "09".into(),
        initial_alt_ft: 5000,
        squawk: code,
    });
    Station::uninstall();
    assert_eq!(crate::party_line::assigned_clearance(), None, "uninstall retires it");
    assert_eq!(crate::party_line::ifr_flight_code(), None, "uninstall ends the IFR flight");
    *REGION.lock().unwrap() = Vec::new();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
}

/// Issue #96: a destination beyond the region index (no staffed tower)
/// is cleared by its ICAO code from the whole-file directory; "cancel
/// IFR" retires the clearance and the flight's IFR standing, and a VFR
/// flight asking is told it has none.
#[test]
fn an_ifr_clearance_reaches_any_field_and_cancel_ifr_ends_it() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    Station::uninstall();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *REGION.lock().unwrap() = vec![edinburgh()];
    // Glenforsa, a Mull grass strip with no tower row: not in the region
    // index, in the directory by its code only.
    *FIELD_DIR.lock().unwrap() = vec![("EGHM".into(), "Glenforsa".into(), (56.52, -5.91))];
    assert_eq!(resolve_destination("EGHM").map(|r| r.0).as_deref(), Some("EGHM"));
    assert_eq!(resolve_destination("glenforsa"), None, "names only from the region index");
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 10.0,
    };
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.set_procedures_now(lxgb_cifp_dir().to_str().unwrap(), "", "", "LXGB");
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        true,
        false,
        None,
        false,
        None,
    );
    let cs = "November one seven two Sierra Papa";
    // Nothing to cancel yet.
    let r = st.route_keyed(
        "gibraltar tower, november one seven two sierra papa, cancel ifr",
        Service::Tower,
        &ctx,
        wx,
    );
    assert_eq!(r, format!("{cs}, roger, you are not on an IFR clearance."));
    let r = st.route_keyed(
        "gibraltar tower, november one seven two sierra papa, request ifr clearance to echo golf hotel mike",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.starts_with(&format!("{cs}, cleared to Glenforsa")), "{r}");
    let rb = crate::party_line::readback_owed().expect("owed");
    assert_eq!(st.route_keyed(&rb.to_lowercase(), Service::Tower, &ctx, wx), "");
    assert_eq!(
        crate::party_line::assigned_clearance().map(|c| c.dest).as_deref(),
        Some("EGHM")
    );
    assert!(crate::party_line::ifr_flight_code().is_some());
    assert!(st.desk.ifr_runway.is_some(), "the SID's runway is frozen");
    // Cancelled: the clearance, the flight's IFR code and the frozen
    // runway all go.
    let r = st.route_keyed(
        "gibraltar tower, november one seven two sierra papa, cancelling i f r",
        Service::Tower,
        &ctx,
        wx,
    );
    assert_eq!(r, format!("{cs}, IFR flight cancelled."));
    assert_eq!(crate::party_line::assigned_clearance(), None);
    assert_eq!(crate::party_line::ifr_flight_code(), None);
    assert!(st.desk.ifr_runway.is_none());
    // An owed readback is cancelled too, with nothing published after.
    st.route_keyed(
        "gibraltar tower, november one seven two sierra papa, request ifr clearance to edinburgh",
        Service::Tower,
        &ctx,
        wx,
    );
    let r = st.route_keyed(
        "november one seven two sierra papa, cancel ifr",
        Service::Tower,
        &ctx,
        wx,
    );
    assert_eq!(r, format!("{cs}, IFR flight cancelled."));
    assert!(crate::party_line::readback_owed().is_none_or(|r| !r.starts_with("Cleared to")));
    assert_eq!(crate::party_line::assigned_clearance(), None);
    Station::uninstall();
    *REGION.lock().unwrap() = Vec::new();
    *FIELD_DIR.lock().unwrap() = Vec::new();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
}

#[test]
fn an_ifr_clearance_with_enforcement_off_stands_on_any_readback() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    Station::uninstall();
    *REGION.lock().unwrap() = vec![edinburgh()];
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 10.0,
    };
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.set_enforce_readback(false);
    st.set_dialect(Dialect::Faa);
    let ctx = st.make_ctx(
        wx,
        "altimeter two niner niner two".into(),
        "November one seven two Sierra Papa".into(),
        true,
        false,
        None,
        false,
        None,
    );
    // No procedures set: FAA runway heading, the departure frequency.
    let r = st.route_keyed("request ifr clearance to egph", Service::Tower, &ctx, wx);
    assert!(r.contains("cleared to Edinburgh airport as filed, after departure fly runway heading, climb and maintain five thousand, departure frequency one three zero point four"), "{r}");
    // A mishear is still no readback: nothing stands.
    st.route_keyed("gibraltar tower, blah", Service::Tower, &ctx, wx);
    assert_eq!(crate::party_line::assigned_clearance(), None);
    // A readback with the wrong altitude is coached, not challenged, and stands.
    let r = st.route_keyed("cleared to edinburgh, climb and maintain six thousand, squawk 4521", Service::Tower, &ctx, wx);
    assert_eq!(r, "");
    assert!(crate::party_line::assigned_clearance().is_some());
    Station::uninstall();
    *REGION.lock().unwrap() = Vec::new();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
}

/// Two SIDs off Gibraltar's 09 with opposite exits: NORTH1A to NRTHX
/// (up the peninsula) and SOUTH1A to STHXX (across the strait). Each
/// destination is cleared via the one that leaves toward it, whichever
/// the file lists first, and the bearings behind the pick are kept for
/// the blackbox.
#[test]
fn the_ifr_sid_is_the_one_that_leaves_toward_the_destination() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    Station::uninstall();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    let dir = std::env::temp_dir().join(format!("xc-station-sids-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let leg = |sid: &str, seq: u32, fix: &str| {
        format!(
            "SID:{seq:03},5,{sid},RW09,{fix},LX,E,A,E   , ,   ,TF, ,   ,  , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   ;\n"
        )
    };
    // The southbound SID first in the file: the old rule's pick.
    let cifp = [
        leg("SOUTH1A", 10, "SMIDX"),
        leg("SOUTH1A", 20, "STHXX"),
        leg("NORTH1A", 10, "NMIDX"),
        leg("NORTH1A", 20, "NRTHX"),
    ]
    .concat();
    std::fs::write(dir.join("LXGB.dat"), cifp).unwrap();
    let fixes = dir.join("earth_fix.dat");
    std::fs::write(
        &fixes,
        "I\n1200 Version\n\n 36.300 -5.300 NMIDX ENRT LX 2\n 37.500 -5.200 NRTHX ENRT LX 2\n \
             35.900 -5.400 SMIDX ENRT LX 2\n 34.800 -5.500 STHXX ENRT LX 2\n99\n",
    )
    .unwrap();
    let mut madrid = region_apt("Madrid Barajas", 40.47, -3.56);
    madrid.icao = "LEMD".into();
    let mut marrakech = region_apt("Marrakech Menara", 31.61, -8.04);
    marrakech.icao = "GMMX".into();
    *REGION.lock().unwrap() = vec![madrid, marrakech];
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 10.0,
    };
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.set_procedures_now(
        dir.to_str().unwrap(),
        fixes.to_str().unwrap(),
        "",
        "LXGB",
    );
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        true,
        false,
        None,
        false,
        None,
    );
    let r = st.route_keyed("request ifr clearance to lemd", Service::Tower, &ctx, wx);
    assert!(r.contains("via the North one Alpha departure, runway zero niner"), "{r}");
    let (exit, dest) = st.desk.ifr_bearings.expect("bearings kept");
    assert!(exit < 10.0 || exit > 350.0, "north exit: {exit}");
    assert!((dest - 22.0).abs() < 10.0, "Madrid bearing: {dest}");
    // Another flight (a fresh station: this one is owed a readback).
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.set_procedures_now(
        dir.to_str().unwrap(),
        fixes.to_str().unwrap(),
        "",
        "LXGB",
    );
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        true,
        false,
        None,
        false,
        None,
    );
    let r = st.route_keyed("request ifr clearance to gmmx", Service::Tower, &ctx, wx);
    assert!(r.contains("via the South one Alpha departure"), "{r}");
    let (exit, dest) = st.desk.ifr_bearings.expect("bearings kept");
    assert!((exit - 190.0).abs() < 15.0, "south exit: {exit}");
    assert!((dest - 212.0).abs() < 10.0, "Marrakech bearing: {dest}");
    // No positioned fixes: the first in the file, as before.
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.set_procedures_now(dir.to_str().unwrap(), "", "", "LXGB");
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        true,
        false,
        None,
        false,
        None,
    );
    let r = st.route_keyed("request ifr clearance to lemd", Service::Tower, &ctx, wx);
    assert!(r.contains("via the South one Alpha departure"), "{r}");
    assert_eq!(st.desk.ifr_bearings, None);
    Station::uninstall();
    *REGION.lock().unwrap() = Vec::new();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
}

fn filed_lxgb_lemd_via_south() -> Option<crate::party_line::FiledRoute> {
    Some(crate::party_line::FiledRoute {
        origin: "LXGB".into(),
        dest: "LEMD".into(),
        fixes: vec!["SMIDX".into(), "STHXX".into(), "ENRTE".into()],
        sid: None,
        cruise_ft: None,
    })
}

fn filed_lxgb_lemd_via_nowhere() -> Option<crate::party_line::FiledRoute> {
    Some(crate::party_line::FiledRoute {
        origin: "LXGB".into(),
        dest: "LEMD".into(),
        fixes: vec!["ENRTE".into()],
        sid: None,
        cruise_ft: None,
    })
}

/// #152 (EGKK 08R, sortie 1790601822): the tower cleared the SID whose
/// exit lay toward the destination, KENE3P, for a route filed via LAM,
/// and the crew flew LAM5P. With the crew's FILED route the IFR
/// clearance names the SID that joins it (`nav::sid_for_filed_route`,
/// the crew's rule): Madrid lies north, NORTH1A leaves north, and a
/// route filed via STHXX is cleared via SOUTH1A. A filed route no SID
/// joins is cleared runway heading; a plan to another destination is
/// not this clearance's, and the bearing rule stands.
#[test]
fn the_ifr_sid_is_the_one_that_joins_the_filed_route() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    Station::uninstall();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    let dir = std::env::temp_dir().join(format!("xc-station-filed-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let leg = |sid: &str, seq: u32, fix: &str| {
        format!(
            "SID:{seq:03},5,{sid},RW09,{fix},LX,E,A,E   , ,   ,TF, ,   ,  , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   ;\n"
        )
    };
    let cifp = [
        leg("SOUTH1A", 10, "SMIDX"),
        leg("SOUTH1A", 20, "STHXX"),
        leg("NORTH1A", 10, "NMIDX"),
        leg("NORTH1A", 20, "NRTHX"),
    ]
    .concat();
    std::fs::write(dir.join("LXGB.dat"), cifp).unwrap();
    let fixes = dir.join("earth_fix.dat");
    std::fs::write(
        &fixes,
        "I\n1200 Version\n\n 36.300 -5.300 NMIDX ENRT LX 2\n 37.500 -5.200 NRTHX ENRT LX 2\n \
             35.900 -5.400 SMIDX ENRT LX 2\n 34.800 -5.500 STHXX ENRT LX 2\n99\n",
    )
    .unwrap();
    let mut madrid = region_apt("Madrid Barajas", 40.47, -3.56);
    madrid.icao = "LEMD".into();
    let mut marrakech = region_apt("Marrakech Menara", 31.61, -8.04);
    marrakech.icao = "GMMX".into();
    *REGION.lock().unwrap() = vec![madrid, marrakech];
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 10.0,
    };
    let path = lxgb_apt_file();
    let ask = |dest: &str| {
        let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
        st.set_procedures_now(dir.to_str().unwrap(), fixes.to_str().unwrap(), "", "LXGB");
        let ctx = st.make_ctx(
            wx,
            "QNH one zero one seven".into(),
            "November one seven two Sierra Papa".into(),
            true,
            false,
            None,
            false,
            None,
        );
        let r = st.route_keyed(&format!("request ifr clearance to {dest}"), Service::Tower, &ctx, wx);
        (r, st.desk.ifr_sid_basis.clone())
    };
    *crate::party_line::FILED_ROUTE.lock().unwrap() = Some(filed_lxgb_lemd_via_south);
    let (r, basis) = ask("lemd");
    assert!(r.contains("via the South one Alpha departure"), "{r}");
    assert_eq!(basis, Some(SidBasis::Route { join: "STHXX".into() }));
    // A plan to Madrid says nothing about a clearance to Marrakech.
    let (r, basis) = ask("gmmx");
    assert!(r.contains("via the South one Alpha departure"), "{r}");
    assert_eq!(basis, Some(SidBasis::Bearing));
    *crate::party_line::FILED_ROUTE.lock().unwrap() = Some(filed_lxgb_lemd_via_nowhere);
    let (r, basis) = ask("lemd");
    assert!(r.contains("runway heading"), "{r}");
    assert!(!r.contains("One Alpha") && !r.contains("one Alpha"), "{r}");
    assert_eq!(basis, Some(SidBasis::NoneJoins));
    // No provider: the bearing rule, as before.
    *crate::party_line::FILED_ROUTE.lock().unwrap() = None;
    let (r, basis) = ask("lemd");
    assert!(r.contains("via the North one Alpha departure"), "{r}");
    assert_eq!(basis, Some(SidBasis::Bearing));
    Station::uninstall();
    *REGION.lock().unwrap() = Vec::new();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
}

/// #152: a crew that cannot fly the cleared SID asks for another by
/// name. One off the cleared runway is granted and the held clearance
/// names it (the crew waits on `assigned_clearance`); one the field does
/// not publish off that runway is refused and the cleared one stands; a
/// flight with no IFR clearance is told to ask for one.
#[test]
fn a_sid_asked_for_by_name_amends_the_clearance_or_the_cleared_one_stands() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    Station::uninstall();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    let dir = std::env::temp_dir().join(format!("xc-station-amend-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let leg = |sid: &str, seq: u32, fix: &str| {
        format!(
            "SID:{seq:03},5,{sid},RW09,{fix},LX,E,A,E   , ,   ,TF, ,   ,  , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   ;\n"
        )
    };
    let cifp = [leg("SOUTH1A", 10, "STHXX"), leg("NORTH1A", 10, "NRTHX")].concat();
    std::fs::write(dir.join("LXGB.dat"), cifp).unwrap();
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 10.0,
    };
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    st.set_procedures_now(dir.to_str().unwrap(), "", "", "LXGB");
    let ctx = st.make_ctx(
        wx,
        "QNH one zero one seven".into(),
        "November one seven two Sierra Papa".into(),
        true,
        false,
        None,
        false,
        None,
    );
    let ask = "November one seven two Sierra Papa, unable North one Alpha, request the South one Alpha departure";
    *OWN_STRIP.ifr_clearance.lock().unwrap() = None;
    let r = st.route_keyed(ask, Service::Tower, &ctx, wx);
    assert!(r.contains("request your IFR clearance first"), "{r}");
    *OWN_STRIP.ifr_clearance.lock().unwrap() = Some(tower::IfrClearance {
        dest: "LEMD".into(),
        sid: Some("NORTH1A".into()),
        runway: "09".into(),
        initial_alt_ft: 5000,
        squawk: 4521,
    });
    let r = st.route_keyed(
        "November one seven two Sierra Papa, request the Kene three Papa departure",
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("negative") && r.contains("North one Alpha departure stands"), "{r}");
    assert_eq!(
        crate::party_line::assigned_clearance().and_then(|c| c.sid).as_deref(),
        Some("NORTH1A")
    );
    let r = st.route_keyed(ask, Service::Tower, &ctx, wx);
    assert!(r.contains("South one Alpha departure approved, runway"), "{r}");
    assert_eq!(
        crate::party_line::assigned_clearance().and_then(|c| c.sid).as_deref(),
        Some("SOUTH1A")
    );
    Station::uninstall();
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
}

#[test]
fn ifr_destinations_resolve_by_code_or_name_and_never_by_guess() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut ord = region_apt("Chicago O'Hare Intl", 41.97, -87.9);
    ord.icao = "KORD".into();
    let mut mdw = region_apt("Chicago Midway Intl", 41.78, -87.75);
    mdw.icao = "KMDW".into();
    *REGION.lock().unwrap() = vec![edinburgh(), ord, mdw];
    assert_eq!(resolve_destination("EGPH").map(|r| r.0).as_deref(), Some("EGPH"));
    assert_eq!(resolve_destination("egph").map(|r| r.0).as_deref(), Some("EGPH"));
    assert_eq!(resolve_destination("ORD").map(|r| r.0).as_deref(), Some("KORD"), "the K a US pilot drops");
    assert_eq!(resolve_destination("edinburgh").map(|r| r.0).as_deref(), Some("EGPH"));
    assert_eq!(resolve_destination("midway").map(|r| r.0).as_deref(), Some("KMDW"));
    assert_eq!(resolve_destination("chicago"), None, "two Chicagos: no guess");
    assert_eq!(resolve_destination("atlantis"), None);
    *REGION.lock().unwrap() = Vec::new();
    assert_eq!(resolve_destination("EGPH"), None, "no index, nothing known");
}

#[test]
fn lxgb_station_loads_from_real_apt_rows() {
    let path = lxgb_apt_file();
    let st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    assert_eq!(st.apt.name, "Gibraltar");
    assert_eq!(st.tower_khz, 131200);
    assert_eq!(
        st.approach_khz,
        Some(122800),
        "approach row wired for vectors"
    );
    // One physical runway, both ends named.
    assert_eq!(st.apt.runways.len(), 1);
    let rw = &st.apt.runways[0];
    assert_eq!((rw.ident_a.as_str(), rw.ident_b.as_str()), ("09", "27"));
    // The 09 end points roughly east — the vectors' final approach course.
    assert!(
        (rw.heading_a_deg() - 88.0).abs() < 5.0,
        "{}",
        rw.heading_a_deg()
    );
    // Departures: the into-wind pick from the real geometry. Gibraltar's
    // prevailing westerly means 27; an easterly Levanter flips it to 09.
    let (id, _, hw) = nav::best_runway_for_wind(&st.apt.runways, 250.0, 12.0).unwrap();
    assert_eq!((id.as_str(), hw > 0.0), ("27", true));
    assert_eq!(
        nav::best_runway_for_wind(&st.apt.runways, 90.0, 10.0)
            .unwrap()
            .0,
        "09"
    );
    // Every service the field actually has resolves by name; ATIS (which
    // LXGB's apt.dat doesn't list) declines instead of guessing.
    assert!(matches!(
        st.tune_request("contact departure"),
        Some(Tune::Found { khz: 130400, .. })
    ));
    assert!(matches!(
        st.tune_request("tune the atis"),
        Some(Tune::Missing(_))
    ));
}

/// The whole LXGB arrival, flown: vectors from north of the Rock onto
/// final for 09, the visual-approach clearance + tower handoff once
/// established, then the tower's landing clearance off the "final" report.
/// One test on purpose: `OWN_STRIP.vectors` is a process-wide static and parallel
/// tests would clobber each other's plan.
#[test]
fn lxgb_vectored_arrival_flies_to_a_landing_clearance() {
    use std::time::Duration;

    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let rw = st.apt.runways[0].clone();
    let (thr_lat, thr_lon, course) = (rw.lat_a, rw.lon_a, rw.heading_a_deg());

    // Arm the plan exactly as arm_vectors would for runway 09.
    let target_alt_ft = ((st.apt.elev_ft + 1500.0) / 100.0).round() * 100.0;
    *OWN_STRIP.vectors.lock().unwrap() = Some(VectorPlan {
        callsign: "November one seven two Sierra Papa".into(),
        thr_lat,
        thr_lon,
        course_deg: course,
        runway_spoken: "zero niner".into(),
        tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
        target_alt_ft,
        ils: false,
        join_nm: 6.0,
        cleared: false,
        dialect: Dialect::Icao,
        last_tx: None,
        last_heading: None,
    });

    // Regression: crossing final perpendicular inside the wedge must NOT
    // be "established" — position said cleared, the track says crossing.
    let t0 = std::time::Instant::now();
    let (x_lat, x_lon) = crate::nav::project_latlon(thr_lat, thr_lon, course + 180.0, 4.0);
    let crossing = vector_tick(x_lat, x_lon, 1600.0, Some(0.0), false, t0);
    if let Some(msg) = &crossing {
        assert!(
            !msg.contains("Cleared"),
            "cleared while crossing final: {msg}"
        );
    }

    // Now the real arrival: 10 nm north of the field at 3,000 ft. The sim
    // obeys each assigned heading at 100 kts in 10 s ticks, descending to
    // the assigned altitude — a pilot doing exactly as told, nothing more.
    OWN_STRIP.vectors.lock().unwrap().as_mut().unwrap().last_tx = None;
    OWN_STRIP.vectors.lock().unwrap().as_mut().unwrap().last_heading = None;
    let (mut lat, mut lon) = crate::nav::project_latlon(thr_lat, thr_lon, 0.0, 10.0);
    let mut alt = 3000.0_f64;
    let mut hdg = 180.0_f64;
    let mut first: Option<String> = None;
    let mut cleared: Option<String> = None;
    for step in 0..240u64 {
        let now = t0 + Duration::from_secs(step * 10);
        if let Some(msg) = vector_tick(lat, lon, alt, Some(hdg), false, now) {
            if msg.contains("Cleared visual approach") {
                cleared = Some(msg);
                break;
            }
            first.get_or_insert(msg);
        }
        if let Some(h) = OWN_STRIP.vectors
            .lock()
            .unwrap()
            .as_ref()
            .and_then(|p| p.last_heading)
        {
            hdg = h as f64; // instant turn: worst case for the capture curve
        }
        let (nlat, nlon) = crate::nav::project_latlon(lat, lon, hdg, 100.0 * 10.0 / 3600.0);
        (lat, lon) = (nlat, nlon);
        alt = (alt - 80.0).max(target_alt_ft);
    }

    let first = first.expect("an opening vector");
    assert!(first.contains("fly heading"), "{first}");
    // Gibraltar speaks ICAO/CAP 413: altitudes are "descend to altitude
    // ... feet", and headings carry "degrees".
    assert!(
        first.contains("descend to altitude one thousand five hundred feet"),
        "{first}"
    );
    assert!(first.contains("degrees"), "{first}");
    let cleared = cleared.expect("never established on final in 40 sim-minutes");
    assert!(cleared.contains("runway zero niner"), "{cleared}");
    assert!(
        cleared.contains("Contact Tower one three one decimal two"),
        "{cleared}"
    );
    assert!(
        OWN_STRIP.vectors.lock().unwrap().is_none(),
        "plan must be wiped once approach's job is done"
    );

    // The handoff lands on the tower: "final" alone earns the clearance.
    let clear = st.desk.twr.handle(
        tower::parse_intent("november one seven two sierra papa, final"),
        &tower::Ctx {
            callsign: "November one seven two Sierra Papa".into(),
            airport: "Gibraltar".into(),
            runway: "zero niner".into(),
            qnh: "QNH one zero one seven".into(),
            wind: "calm".into(),
            holding_point: "Alpha".into(),
            stand: String::new(),
            approach: Some(atis::spoken_mhz(122800)),
            squawk: "seven zero zero zero".into(),
            final_traffic_nm: None,
            final_traffic_callsign: None,
            final_traffic_count: 0,
            final_traffic_id: None,
            sequence_ahead: None,
            runway_occupied: false,
            own_on_runway: false,
            helicopter: false,
            ifr: None,
            circuit: None,
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
        },
    );
    assert!(clear.contains("cleared to land"), "{clear}");

    // Touchdown: the on-ground tick retires any leftover plan quietly.
    *OWN_STRIP.vectors.lock().unwrap() = Some(VectorPlan {
        callsign: String::new(),
        thr_lat,
        thr_lon,
        course_deg: course,
        runway_spoken: String::new(),
        tower_spoken: None,
        target_alt_ft,
        ils: false,
        join_nm: 6.0,
        cleared: false,
        dialect: Dialect::Icao,
        last_tx: None,
        last_heading: None,
    });
    assert!(vector_tick(thr_lat, thr_lon, 15.0, None, true, t0).is_none());
    assert!(
        OWN_STRIP.vectors.lock().unwrap().is_none(),
        "on-ground tick clears the plan"
    );

    // --- The same arrival flown as an ILS. The clearance rides the
    // intercept turn ("cleared ILS approach..., report established"),
    // headings carry "degrees" (ICAO dialect), and the established
    // wedge hands off WITHOUT re-clearing.
    *OWN_STRIP.vectors.lock().unwrap() = Some(VectorPlan {
        callsign: "November one seven two Sierra Papa".into(),
        thr_lat,
        thr_lon,
        course_deg: course,
        runway_spoken: "zero niner".into(),
        tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
        target_alt_ft,
        ils: true,
        join_nm: 6.0,
        cleared: false,
        dialect: Dialect::Icao,
        last_tx: None,
        last_heading: None,
    });
    let (mut lat, mut lon) = crate::nav::project_latlon(thr_lat, thr_lon, 0.0, 10.0);
    let mut alt = 3000.0_f64;
    let mut hdg = 180.0_f64;
    let mut cleared_ils: Option<String> = None;
    let mut handoff: Option<String> = None;
    // Orbit detector: consecutive vectors must never reverse the
    // aircraft (the pursuit-curve wind-up commanded 250 then 010 at
    // Jerez, 2026-07-12 — an orbit, not an intercept).
    let mut prev_cmd: Option<i64> = None;
    for step in 0..240u64 {
        let now = t0 + Duration::from_secs(7200 + step * 10);
        if let Some(msg) = vector_tick(lat, lon, alt, Some(hdg), false, now) {
            if msg.contains("fly heading") {
                assert!(
                    msg.contains("degrees"),
                    "ICAO headings carry the unit: {msg}"
                );
            }
            if msg.contains("Cleared ILS approach") {
                cleared_ils.get_or_insert(msg.clone());
            }
            if msg.contains("Contact Tower") {
                handoff = Some(msg);
                break;
            }
        }
        if let Some(h) = OWN_STRIP.vectors
            .lock()
            .unwrap()
            .as_ref()
            .and_then(|p| p.last_heading)
        {
            if let Some(p) = prev_cmd {
                assert!(
                    sim::angles::abs_diff_deg(h as f64, p as f64) <= 120.0,
                    "orbit: vector {p} then {h}"
                );
            }
            prev_cmd = Some(h);
            hdg = h as f64;
        }
        let (nlat, nlon) = crate::nav::project_latlon(lat, lon, hdg, 100.0 * 10.0 / 3600.0);
        (lat, lon) = (nlat, nlon);
        alt = (alt - 80.0).max(target_alt_ft);
    }
    let cleared_ils = cleared_ils.expect("ILS clearance never issued");
    assert!(
        cleared_ils.contains("report established"),
        "the intercept clearance asks for the report: {cleared_ils}"
    );
    let handoff = handoff.expect("no handoff after establishing");
    assert!(
        !handoff.contains("Cleared"),
        "already cleared on the intercept, don't re-clear: {handoff}"
    );
    assert!(
        OWN_STRIP.vectors.lock().unwrap().is_none(),
        "ILS plan wiped after handoff"
    );

    // --- Close-in arrival (the Jerez 2026-07-12 geometry): already at
    // platform altitude only 4 nm from the threshold, a mile east of the
    // centerline. The old pursuit curve orbited here; the fixed cut must
    // walk it onto final and hand off without ever reversing a vector.
    *OWN_STRIP.vectors.lock().unwrap() = Some(VectorPlan {
        callsign: "November one seven two Sierra Papa".into(),
        thr_lat,
        thr_lon,
        course_deg: course,
        runway_spoken: "zero niner".into(),
        tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
        target_alt_ft,
        ils: true,
        join_nm: 8.6,
        cleared: false,
        dialect: Dialect::Icao,
        last_tx: None,
        last_heading: None,
    });
    let recip = (course + 180.0).rem_euclid(360.0);
    let (mut lat, mut lon) = {
        let (a, b) = crate::nav::project_latlon(thr_lat, thr_lon, recip, 4.0);
        crate::nav::project_latlon(a, b, (recip + 90.0).rem_euclid(360.0), 1.0)
    };
    let mut hdg = course;
    let mut alt = target_alt_ft;
    let mut prev_cmd: Option<i64> = None;
    let mut done: Option<String> = None;
    for step in 0..240u64 {
        let now = t0 + Duration::from_secs(14_400 + step * 10);
        if let Some(msg) = vector_tick(lat, lon, alt, Some(hdg), false, now) {
            if msg.contains("Contact Tower") {
                done = Some(msg);
                break;
            }
        }
        if let Some(h) = OWN_STRIP.vectors
            .lock()
            .unwrap()
            .as_ref()
            .and_then(|p| p.last_heading)
        {
            if let Some(p) = prev_cmd {
                assert!(
                    sim::angles::abs_diff_deg(h as f64, p as f64) <= 120.0,
                    "close-in orbit: vector {p} then {h}"
                );
            }
            prev_cmd = Some(h);
            hdg = h as f64;
        }
        let (nlat, nlon) = crate::nav::project_latlon(lat, lon, hdg, 100.0 * 10.0 / 3600.0);
        (lat, lon) = (nlat, nlon);
        alt = (alt - 80.0).max(target_alt_ft);
    }
    done.expect("close-in arrival never handed off");
    assert!(OWN_STRIP.vectors.lock().unwrap().is_none());

    // --- The tight base (Gibraltar 2026-07-13): the pilot rolls out on
    // final at 0.86 nm, below the vector altitude, pointed down the
    // runway. The old 1.0 nm wedge floor never saw it — the plan
    // vectored the short-final aircraft away and touchdown discarded it
    // with no clearance ever spoken, earning a compliant pilot the
    // landing berate. Inside a mile, landing intent IS established.
    let arm_short_final = || {
        *OWN_STRIP.vectors.lock().unwrap() = Some(VectorPlan {
            callsign: "November one seven two Sierra Papa".into(),
            thr_lat,
            thr_lon,
            course_deg: course,
            runway_spoken: "zero niner".into(),
            tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
            target_alt_ft,
            ils: false,
            join_nm: 6.0,
            cleared: false,
            dialect: Dialect::Icao,
            last_tx: None,
            last_heading: None,
        });
    };
    arm_short_final();
    let (sf_lat, sf_lon) = crate::nav::project_latlon(thr_lat, thr_lon, recip + 3.0, 0.86);
    let short_final = vector_tick(
        sf_lat,
        sf_lon,
        target_alt_ft - 950.0,
        Some(course + 12.0),
        false,
        t0,
    )
    .expect("short final earns the clearance, not a vector away");
    assert!(
        short_final.contains("Cleared visual approach runway zero niner"),
        "{short_final}"
    );
    assert!(
        short_final.contains("1 mile from touchdown"),
        "{short_final}"
    );
    assert!(short_final.contains("Contact Tower"), "{short_final}");
    assert!(
        OWN_STRIP.vectors.lock().unwrap().is_none(),
        "plan done after the clearance"
    );

    // Inside a mile but NOT landing — crossing final perpendicular, and a
    // flyover at altitude — gets silence, never a reversal heading, and
    // the plan survives for when the geometry recovers.
    arm_short_final();
    let (x_lat, x_lon) = crate::nav::project_latlon(thr_lat, thr_lon, recip, 0.5);
    let crossing_track = (course + 90.0).rem_euclid(360.0);
    assert!(
        vector_tick(
            x_lat,
            x_lon,
            target_alt_ft - 950.0,
            Some(crossing_track),
            false,
            t0
        )
        .is_none(),
        "no vector inside a mile"
    );
    assert!(
        vector_tick(x_lat, x_lon, target_alt_ft + 500.0, Some(course), false, t0).is_none(),
        "a flyover at altitude is not established"
    );
    assert!(
        OWN_STRIP.vectors.lock().unwrap().is_some(),
        "plan survives the blow-through"
    );
    *OWN_STRIP.vectors.lock().unwrap() = None;

    // --- The pilot who reports "established" earns the handoff from the
    // approach controller directly, and the plan dies with it.
    let ctx2 = tower::Ctx {
        callsign: "November one seven two Sierra Papa".into(),
        airport: "Gibraltar".into(),
        runway: "zero niner".into(),
        qnh: "QNH one zero one seven".into(),
        wind: "calm".into(),
        holding_point: "Alpha".into(),
        stand: String::new(),
        approach: Some(atis::spoken_mhz(122800)),
        squawk: "seven zero zero zero".into(),
        final_traffic_nm: None,
        final_traffic_callsign: None,
        final_traffic_count: 0,
        final_traffic_id: None,
        sequence_ahead: None,
        runway_occupied: false,
        own_on_runway: false,
        helicopter: false,
        ifr: None,
        circuit: None,
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
    };
    let wx2 = Wx {
        wind_from_deg: 90.0,
        wind_kts: 8.0,
    };
    *OWN_STRIP.vectors.lock().unwrap() = Some(VectorPlan {
        callsign: "November one seven two Sierra Papa".into(),
        thr_lat,
        thr_lon,
        course_deg: course,
        runway_spoken: "zero niner".into(),
        tower_spoken: Some(atis::spoken_mhz(st.tower_khz)),
        target_alt_ft,
        ils: true,
        join_nm: 6.0,
        cleared: true,
        dialect: Dialect::Icao,
        last_tx: None,
        last_heading: None,
    });
    let r = st.route_keyed(
        "gibraltar approach, established i l s runway zero niner",
        Service::Approach,
        &ctx2,
        wx2,
    );
    assert!(
        r.contains("Contact Gibraltar Tower"),
        "established → handoff: {r}"
    );
    assert!(OWN_STRIP.vectors.lock().unwrap().is_none(), "report kills the plan");

    // --- Arm phrasing per dialect, with real chart data on file: the
    // CIFP platform (2,000 ft here) outranks the geometric default.
    *ILS_FACTS.lock().unwrap() = Some(std::sync::Arc::new(IlsFacts {
        ils: vec![nav::AirportIls {
            runway: "09".into(),
            freq_10khz: 11030,
            course_mag: 90.0,
            gs_deg: Some(3.0),
        }],
        platform_ft: [("09".to_string(), 2000.0)].into_iter().collect(),
    }));
    let r = st.route_keyed("request vectors", Service::Approach, &ctx2, wx2);
    assert!(
        r.contains("Vectoring ILS approach runway zero niner"),
        "{r}"
    );
    assert!(
        r.contains("Descend to altitude two thousand feet"),
        "CIFP platform: {r}"
    );
    st.set_dialect(Dialect::Faa);
    let r = st.route_keyed("request vectors", Service::Approach, &ctx2, wx2);
    assert!(
        r.contains("Expect vectors ILS runway zero niner approach"),
        "{r}"
    );
    assert!(r.contains("Descend and maintain two thousand"), "{r}");

    // Statics must not leak into other tests.
    *OWN_STRIP.vectors.lock().unwrap() = None;
    *ILS_FACTS.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
}

#[test]
fn ils_join_sits_under_the_glideslope() {
    // Heathrow: 2,500 ft platform over an 83 ft field, 3° slope — the
    // slope reaches ~2,417 AAL at ~7.6 nm, so the join must be past it.
    let join = ils_join_nm(2417.0, 3.0);
    assert!((8.0..10.0).contains(&join), "{join}");
    // At the join, the slope is ABOVE the platform: intercept from below.
    let slope_at_join_ft = (join - 1.0) * 6076.12 * 3.0_f64.to_radians().tan();
    assert!(slope_at_join_ft >= 2417.0 - 1.0, "{slope_at_join_ft}");
    // A low platform never pulls the join inside the visual's 6 nm.
    assert_eq!(ils_join_nm(800.0, 3.0), 6.0);
    // A degenerate slope angle can't divide toward infinity.
    assert!(ils_join_nm(2000.0, 0.0).is_finite());
}

#[test]
fn phraseology_follows_geography() {
    assert_eq!(dialect_for("EGLL"), Dialect::Icao);
    assert_eq!(dialect_for("LXGB"), Dialect::Icao);
    assert_eq!(dialect_for("LEMD"), Dialect::Icao);
    assert_eq!(dialect_for("KORD"), Dialect::Faa);
    assert_eq!(dialect_for("PHNL"), Dialect::Faa);
    assert_eq!(dialect_for("CYYZ"), Dialect::Faa);
    // The pressure phrase speaks the field's own money: 1013.25 hPa is
    // "QNH one zero one three" in ICAO land and exactly 29.92 inches —
    // "altimeter two niner niner two" — at an FAA field.
    assert_eq!(
        spoken_pressure(101325.0, Dialect::Icao),
        "QNH one zero one three"
    );
    assert_eq!(
        spoken_pressure(101325.0, Dialect::Faa),
        "altimeter two niner niner two"
    );
    assert_eq!(
        spoken_pressure(102000.0, Dialect::Icao),
        "QNH one zero two zero"
    );
    // 1020 hPa ≈ 30.12 inHg.
    assert_eq!(
        spoken_pressure(102000.0, Dialect::Faa),
        "altimeter three zero one two"
    );
}

/// EDLP 24 (SF50, 2026-09-24): the vacate timed out 714 m short of exit
/// C and the stand route was planned from the runway, 51.613882 N
/// 8.615681 E heading 236 (state record at t 1790239644.0). It started
/// back up the strip (the drive joined 179 degrees off) and the
/// clearance said "cross runway zero six" for the runway we were on.
/// Planned from the vacate it runs forward and crosses nothing of ours.
/// `cargo test -p atc -- --ignored live_edlp_stand_route --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_edlp_stand_route_from_the_runway_runs_forward() {
    let apt = crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = nav::find_taxi_net(&apt, "EDLP") else {
        return; // no install, no verdict
    };
    let runways = nav::find_runways(&apt, "EDLP").unwrap();
    let (lat, lon) = (51.613882, 8.615681);
    let strip = runways
        .iter()
        .find(|r| on_runway(r, lat, lon).is_some())
        .expect("the stop is on a runway");
    // The rollout direction: the end heading within 30 degrees of 236.
    let psi = [strip.heading_a_deg(), strip.heading_b_deg()]
        .into_iter()
        .find(|h| sim::angles::abs_diff_deg(*h, 236.0) < 30.0)
        .expect("the landing end");
    let full = format!("{}/{}", strip.ident_a, strip.ident_b);
    // Which way the route first goes: the bearing to its point 40 m on.
    let first_brg = |p: &nav::TaxiRoutePlan| {
        let mut run = 0.0;
        for w in p.path.windows(2) {
            run += nav::haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0;
            if run >= 40.0 {
                return nav::bearing_deg(lat, lon, w[1].0, w[1].1);
            }
        }
        f64::NAN
    };
    // Without the heading: the old plan, from the nearest node.
    let old = stand_route_on(&net, &runways, lat, lon, None).expect("a route");
    println!(
        "old: via {:?} crossings {:?}, first leg {:.0} deg",
        old.via,
        old.crossings,
        first_brg(&old)
    );
    let new = stand_route_on(&net, &runways, lat, lon, Some(psi)).expect("a route");
    println!(
        "new: via {:?} crossings {:?}, first leg {:.0} deg, heading {psi:.0}",
        new.via,
        new.crossings,
        first_brg(&new)
    );
    assert_eq!(new.seg.len() + 1, new.path.len(), "seg stays aligned");
    assert!(
        sim::angles::abs_diff_deg(first_brg(&new), psi) < 45.0,
        "the stand route leaves the runway forward: {:.0} against {psi:.0}",
        first_brg(&new)
    );
    assert!(
        !new.crossings.contains(&full),
        "the strip we are on is not a crossing: {:?}",
        new.crossings
    );
}

/// LEMG 31 (737, 2026-09-28, issue #141): the rollout stopped at
/// 36.674101 N 4.497971 W heading 311 true (state at t 1790585617.7;
/// sortie 1790585651 stopped at 36.674500 N 4.498537 W, t 1790586584.6),
/// 1.1 m off the 13/31 centreline and 1,262 m off 12/30. The strip under
/// the aircraft is 13/31, the vacate starts AT the aircraft and runs
/// forward, and the stand route from there leaves 13/31 forward without
/// calling it a crossing. The "12/30" of the recorded
/// `stand_route_from_runway` was the RIGHT answer for where the aircraft
/// then was: the sortie runner had reloaded it onto 12/30's centreline
/// (36.685320 N 4.494170 W, 0.0 m off 12/30, 1,158 m off 13/31) 0.3 s
/// after the landing's verdict, so it is pinned here too.
/// `cargo test -p atc -- --ignored live_lemg_31 --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_lemg_31_rollout_vacates_forward_off_the_strip_under_it() {
    let apt =
        crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = nav::find_taxi_net(&apt, "LEMG") else {
        return; // no install, no verdict
    };
    let runways = nav::find_runways(&apt, "LEMG").unwrap();
    let strip_under = |lat: f64, lon: f64| -> Vec<String> {
        runways
            .iter()
            .filter(|r| on_runway(r, lat, lon).is_some())
            .map(|r| format!("{}/{}", r.ident_a, r.ident_b))
            .collect()
    };
    // The reload spot: 12/30 and nothing else.
    assert_eq!(
        strip_under(36.68532005, -4.49417),
        vec!["12/30".to_string()]
    );
    let psi = 311.7; // 31's true course; the rollout read 310.9 to 311.4
    for (lat, lon) in [
        (36.67410120039185, -4.49797079757389),
        (36.674499892604075, -4.498536580519237),
    ] {
        assert_eq!(
            strip_under(lat, lon),
            vec!["13/31".to_string()],
            "the rollout is on 13/31"
        );
        let zone = vec!["31".to_string(), "13".to_string()];
        let v = net
            .plan_vacate(lat, lon, psi, &zone, nav::VACATE_CLEAR_M)
            .expect("a way off 31 ahead");
        let start_m = nav::haversine_nm(lat, lon, v.path[0].0, v.path[0].1) * 1852.0;
        println!(
            "vacate via {:?}: {} pts, starts {start_m:.1} m from the aircraft",
            v.exit_name,
            v.path.len()
        );
        assert!(
            start_m < 5.0,
            "the vacate starts at the aircraft: {start_m:.1} m"
        );
        // Forward: the first point 40 m on is ahead of abeam.
        let mut run = 0.0;
        let mut ahead = None;
        for w in v.path.windows(2) {
            run += nav::haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0;
            if run >= 40.0 {
                ahead = Some(nav::bearing_deg(lat, lon, w[1].0, w[1].1));
                break;
            }
        }
        let brg = ahead.expect("a vacate longer than 40 m");
        assert!(
            sim::angles::abs_diff_deg(brg, psi) < 90.0,
            "the vacate runs forward: {brg:.0} against {psi:.0}"
        );
        // Ends OFF 13/31 (and off every strip).
        let end = *v.path.last().unwrap();
        assert!(
            strip_under(end.0, end.1).is_empty(),
            "the vacate ends off the strips: {end:?}"
        );
        // The stand route asked from here starts with that vacate, off
        // 13/31, forward, and never calls 13/31 a crossing.
        let plan = stand_route_on(&net, &runways, lat, lon, Some(psi)).expect("a stand route");
        let first_m = nav::haversine_nm(lat, lon, plan.path[0].0, plan.path[0].1) * 1852.0;
        println!(
            "stand {:?} via {:?} crossings {:?}, starts {first_m:.1} m off",
            plan.holding_point, plan.via, plan.crossings
        );
        assert!(
            first_m < 5.0,
            "the stand route starts at the aircraft: {first_m:.1} m"
        );
        assert!(
            !plan.crossings.contains(&"13/31".to_string()),
            "the strip we are on is not a crossing: {:?}",
            plan.crossings
        );
    }
}

/// A point `along` metres down a runway end's heading from its
/// threshold (negative: out on its final), `right` metres to its right.
fn lemg_point(r: &nav::Runway, end_a: bool, along: f64, right: f64) -> (f64, f64) {
    let (lat0, lon0, lat1, lon1) = if end_a {
        (r.lat_a, r.lon_a, r.lat_b, r.lon_b)
    } else {
        (r.lat_b, r.lon_b, r.lat_a, r.lon_a)
    };
    let m_lon = 111_320.0 * lat0.to_radians().cos();
    let (bx, by) = ((lon1 - lon0) * m_lon, (lat1 - lat0) * 111_320.0);
    let len = (bx * bx + by * by).sqrt();
    let (ux, uy) = (bx / len, by / len);
    let (x, y) = (ux * along + uy * right, uy * along - ux * right);
    (lat0 + y / 111_320.0, lon0 + x / m_lon)
}

fn lemg_ghost(callsign: &str, at: (f64, f64), ele_ft: f64, on_ground: bool) -> TcasTarget {
    TcasTarget {
        callsign: callsign.into(),
        lat: at.0,
        lon: at.1,
        ele_ft,
        on_ground,
        vs_fpm: if on_ground { 0.0 } else { -700.0 },
    }
}

/// Issue #172, question 3: LEMG 12/30 and 13/31 are 14 degrees apart,
/// 826 m at the 12 and 13 thresholds and 1,490 m at the far ends, so
/// neither is the other's close parallel. 12's final crosses 13's
/// extended centreline about 2 nm out: FR655 on 12's final at 2.08 nm
/// (sortie 1790623420-full_auto, +157 s) sat 70 m off 13's line, inside
/// its 500 m corridor, and was "traffic on 2 mile final" to a 737 lined
/// up on 13. It is 12's traffic.
#[test]
fn a_lemg_12_arrival_is_not_traffic_for_13() {
    let rw = lemg_runways();
    let apart = strips_apart_m(&rw[0], &rw[1]);
    assert!((800.0..860.0).contains(&apart), "{apart}");
    assert!(apart >= nav::CLOSE_PARALLEL_M);
    assert_eq!(nav::close_parallel_main(&rw, "12"), "12");
    assert_eq!(nav::close_parallel_main(&rw, "13"), "13");
    let elev = 52.0;
    let fr655 =
        lemg_ghost("FR655", lemg_point(&rw[0], true, -3852.0, 0.0), elev + 660.0, false);
    let (_, cross, _) = centreline_place(&rw[1], fr655.lat, fr655.lon).unwrap();
    assert!(cross < CORRIDOR_HALF_M, "inside 13's corridor: {cross}");
    assert!(
        final_traffic_for(std::slice::from_ref(&fr655), &rw[1], &rw, elev).is_none(),
        "12's lander is not 13's traffic"
    );
    let on_12 = final_traffic_for(std::slice::from_ref(&fr655), &rw[0], &rw, elev)
        .expect("12's own traffic");
    assert!((on_12.nm - 2.08).abs() < 0.05, "{}", on_12.nm);
    // A lander on 13's own final at the same range stays 13's.
    let own = lemg_ghost("VY1", lemg_point(&rw[1], true, -3852.0, 0.0), elev + 660.0, false);
    assert!(final_traffic_for(std::slice::from_ref(&own), &rw[1], &rw, elev).is_some());
    // The calm-wind stream reads it as 12's too.
    assert_eq!(arrival_end(std::slice::from_ref(&fr655), &rw, elev).as_deref(), Some("12"));
    // A close parallel's stream is still one runway's: a strip 200 m off
    // 13 hands its lander to 13's corridor.
    let mut close = rw[1].clone();
    let (a, b) = (
        lemg_point(&rw[1], true, 0.0, -200.0),
        lemg_point(&rw[1], false, 0.0, 200.0),
    );
    close.ident_a = "13L".into();
    close.ident_b = "31R".into();
    (close.lat_a, close.lon_a, close.lat_b, close.lon_b) = (a.0, a.1, b.0, b.1);
    let pair = vec![rw[1].clone(), close.clone()];
    let on_close =
        lemg_ghost("VY2", lemg_point(&close, true, -3000.0, 0.0), elev + 500.0, false);
    assert!(final_traffic_for(std::slice::from_ref(&on_close), &rw[1], &pair, elev).is_some());
}

/// Issue #172, question 1, from the sortie's records: the 737 lined up
/// on LEMG 13 45 m from the threshold was held (U25702 lined up ahead,
/// FR655 on 12's final), promised a clearance after the landing traffic,
/// and then held five minutes by the replay's 13 departures lining up in
/// front of it. The hold is kept for what it was held FOR: the ghost
/// ahead at the first tick leaves, a later ghost standing in front is a
/// cut-in (blackbox `hold_cut_in`), 12's lander is never ours, and the
/// promise is kept.
#[test]
fn a_held_runway_start_is_released_once_its_traffic_has_gone() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *WAKE.lock().unwrap() = None;
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    let rw = lemg_runways();
    *ACTIVE_RUNWAYS.lock().unwrap() = rw.clone();
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::LinedUp,
        callsign: "November eight one six November November".into(),
        runway_ident: "13".into(),
        pending_readback: None,
        holds_runway: true,
    });
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    *OWN_STRIP.takeoff_hold.lock().unwrap() = Some(TakeoffHold {
        callsign: "November eight one six November November".into(),
        runway: "one three".into(),
        runway_ident: "13".into(),
        clear_since: None,
        armed_at: t0,
        logged_at: t0,
        lined_up: true,
        behind: false,
        held_for: None,
        cut_in_logged: Default::default(),
    });
    let elev = 52.0;
    let own = Some(lemg_point(&rw[1], true, 45.0, 0.0));
    let fr655_final =
        lemg_ghost("FR655", lemg_point(&rw[0], true, -3852.0, 0.0), elev + 660.0, false);
    let fr655_down = lemg_ghost("FR655", lemg_point(&rw[0], true, 900.0, 0.0), elev, true);
    let u25702 = lemg_ghost("U25702", lemg_point(&rw[1], true, 136.0, 0.0), elev, true);
    let ei887 = lemg_ghost("EI887", lemg_point(&rw[1], true, 140.0, 2.0), elev, true);
    let tick = |targets: &[TcasTarget], s: u64| {
        takeoff_hold_tick_from(targets, elev, true, own, t0 + sec(s))
    };
    // U25702 lined up ahead of us: held, whatever 12 is doing.
    assert!(tick(&[fr655_final.clone(), u25702.clone()], 0).is_none());
    assert!(tick(&[fr655_final.clone(), u25702.clone()], 30).is_none());
    // It rolls and goes; FR655 lands on 12; EI887 lines up in front.
    assert!(tick(&[fr655_down.clone(), ei887.clone()], 60).is_none(), "EI887 moving on");
    assert!(tick(&[fr655_down.clone(), ei887.clone()], 65).is_none());
    // Standing, EI887 is the replay's queue, not our traffic: the strip
    // is clear for the hold from here, and 15 s on the tower keeps its
    // word.
    assert!(tick(&[fr655_down.clone(), ei887.clone()], 71).is_none());
    let line = tick(&[fr655_down.clone(), ei887.clone()], 87).expect("the promised release");
    assert!(line.contains("cleared for takeoff"), "{line}");
    assert!(line.contains("one three"), "{line}");
    assert!(*OWN_STRIP.pending_takeoff_clearance.lock().unwrap());

    // Held from the taxiway (not lined up), a ghost on the strip is
    // traffic whatever its seed: the queue ahead of the hold line is
    // the stream we join.
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    *OWN_STRIP.takeoff_hold.lock().unwrap() = Some(TakeoffHold {
        callsign: "November eight one six November November".into(),
        runway: "one three".into(),
        runway_ident: "13".into(),
        clear_since: None,
        armed_at: t0,
        logged_at: t0,
        lined_up: false,
        behind: false,
        held_for: Some(Default::default()),
        cut_in_logged: Default::default(),
    });
    for s in [200, 230, 260] {
        assert!(
            takeoff_hold_tick_from(std::slice::from_ref(&ei887), elev, true, None, t0 + sec(s))
                .is_none(),
            "held at +{s}"
        );
    }

    // Only the pavement AHEAD of our tail is in the way of the roll: from
    // an intersection 900 m down 13, a ghost standing 200 m behind us is
    // not traffic, one 200 m ahead is.
    let mid = Some(lemg_point(&rw[1], true, 900.0, 0.0));
    let behind = lemg_ghost("VY3", lemg_point(&rw[1], true, 700.0, 0.0), elev, true);
    let ahead = lemg_ghost("VY4", lemg_point(&rw[1], true, 1100.0, 0.0), elev, true);
    let now = t0 + sec(300);
    assert!(!departure_strip_picture(&rw[1], "13", std::slice::from_ref(&behind), mid, None, now).blocked);
    assert!(departure_strip_picture(&rw[1], "13", &[ahead], mid, None, now).blocked);
    assert!(
        departure_strip_picture(&rw[1], "13", &[behind], None, None, now).blocked,
        "without our position every occupant counts"
    );
    // A holding point 90 m off the centreline is not the pavement.
    let hold_line = lemg_ghost("VY5", lemg_point(&rw[1], true, 60.0, 90.0), elev, true);
    assert!(!departure_strip_picture(&rw[1], "13", &[hold_line], own, None, now).blocked);

    *OWN_STRIP.takeoff_hold.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.pending_takeoff_clearance.lock().unwrap() = false;
    ACTIVE_RUNWAYS.lock().unwrap().clear();
}

/// Issue #172: a RUNWAY START's first ready call is answered "hold
/// position, traffic on the runway" from Parked (the tower moves it
/// LinedUp in the same reply), and the release watch never armed: the
/// crew nudged "ready for departure" every 40 s and was told "read
/// back" three times. Now the watch arms, and "holding position" is the
/// readback the tower takes.
#[test]
fn a_held_runway_start_arms_the_release_and_takes_the_hold_readback() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *WAKE.lock().unwrap() = None;
    *OWN_STRIP.takeoff_hold.lock().unwrap() = None;
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let wx = Wx {
        wind_from_deg: 90.0,
        wind_kts: 8.0,
    };
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    ctx.own_on_runway = true;
    ctx.runway_occupied = true;
    let cs = "november one seven two sierra papa";
    assert_eq!(st.desk.twr.state, tower::State::Parked);
    let r = st.route_keyed(
        &format!("{cs}, ready for departure, for circuits"),
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(r.contains("hold position, traffic on the runway"), "{r}");
    assert_eq!(st.desk.twr.state, tower::State::LinedUp);
    assert!(
        OWN_STRIP.takeoff_hold.lock().unwrap().as_ref().is_some_and(|h| h.lined_up),
        "the release watch is armed for the runway start"
    );
    let owed = OWN_STRIP.clearance
        .lock()
        .unwrap()
        .as_ref()
        .and_then(|c| c.pending_readback.clone())
        .expect("the hold is owed its readback");
    assert!(owed.starts_with("Holding position"), "{owed}");
    // The crew's readback, as full auto transmits it.
    let r = st.route_keyed(owed.trim_end_matches('.'), Service::Tower, &ctx, wx);
    assert_eq!(r, "", "read back: a quiet frequency");
    assert!(
        OWN_STRIP.clearance
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .pending_readback
            .is_none()
    );
    // Asking again is answered with the hold, never "read back".
    let r = st.route_keyed(
        &format!("{cs}, ready for departure, for circuits"),
        Service::Tower,
        &ctx,
        wx,
    );
    assert!(!r.contains("read back"), "{r}");
    assert!(r.contains("hold position"), "{r}");

    *OWN_STRIP.takeoff_hold.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// LEMD at 240/5 (2026-10-05): the wind faces the 18s, the longest is 18R,
/// and 18R's only ways on are at the 36L end. A departure on the ground is
/// given a runway it can enter at its numbers; an arrival keeps the wind's
/// pick.
/// `cargo test --release -p atc -- --ignored real_install_lemd_departure_pick --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_lemd_departure_pick_skips_an_exit_only_end() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let apt = crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    if !std::path::Path::new(&apt).exists() {
        return;
    }
    let net = nav::find_taxi_net(&apt, "LEMD").unwrap().expect("LEMD's network");
    *TAXI_NET.lock().unwrap() = Some(std::sync::Arc::new(net));
    let mut arr = Station::load(&apt, "LEMD").expect("LEMD has a tower");
    assert_eq!(arr.pick_runway(240.0, 5.0), "18R", "an arrival keeps the wind's pick");
    let mut dep = Station::load(&apt, "LEMD").expect("LEMD has a tower");
    dep.desk.call.departing = true;
    let id = dep.pick_runway(240.0, 5.0);
    println!("departure runway {id}");
    assert!(!["18R", "32L", "32R"].contains(&id.as_str()), "{id}");
    // A departure already given 18R is moved off it too.
    arr.desk.call.departing = true;
    let id = arr.pick_runway(240.0, 5.0);
    assert!(!["18R", "32L", "32R"].contains(&id.as_str()), "{id}");
    *TAXI_NET.lock().unwrap() = None;
}

/// The doors reset our strip BY GROUP, as the cells' lifetimes were when
/// they were statics: a new flight keeps the go-around pacing (`watch`) and
/// drops the flight's carried facts; a region handoff's `install` keeps the
/// carried facts and drops the pacing; `uninstall` drops all four groups.
#[test]
fn the_doors_reset_our_strip_by_group() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    Station::uninstall();
    let dirty = || {
        *OWN_STRIP.taxi_to_stand.lock().unwrap() = true;
        OWN_STRIP
            .uncontrolled_arrival
            .store(true, std::sync::atomic::Ordering::SeqCst);
        *OWN_STRIP.ifr_flight.lock().unwrap() = Some(IfrFlight {
            code: 4501,
            callsign: "Speedbird 1".into(),
            airborne: false,
        });
        *OWN_STRIP.go_around_last.lock().unwrap() = Some(Instant::now());
    };
    let state = || {
        (
            *OWN_STRIP.taxi_to_stand.lock().unwrap(),
            OWN_STRIP
                .uncontrolled_arrival
                .load(std::sync::atomic::Ordering::SeqCst),
            OWN_STRIP.ifr_flight.lock().unwrap().is_some(),
            OWN_STRIP.go_around_last.lock().unwrap().is_some(),
        )
    };

    dirty();
    retire_for_new_flight();
    assert_eq!(state(), (false, false, false, true), "retire_for_new_flight");

    dirty();
    lxgb().install();
    assert_eq!(state(), (false, false, true, false), "install");

    dirty();
    Station::uninstall();
    assert_eq!(state(), (false, false, false, false), "uninstall");
}

/// The ctx is built from a HANDED-IN state with no sim behind the view
/// (every read fails): our ground flag, position and type are the state's,
/// never a read. A runway start on LXGB's strip asking for takeoff is on the
/// runway; the same state airborne is not on the ground.
#[test]
fn a_ctx_is_built_from_a_handed_in_state_with_no_sim() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    Station::uninstall();
    let path = lxgb_apt_file();
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let rw = st.apt.runways[0].clone();
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    let xp = sim::xplane::XPlaneClient::new("http://127.0.0.1:1".into());
    let on_strip = AircraftState {
        lat: (rw.lat_a + rw.lat_b) / 2.0,
        lon: (rw.lon_a + rw.lon_b) / 2.0,
        alt_ft: 11.0,
        on_ground: true,
        gs_kts: 0.0,
        psi_deg: Some(rw.heading_a_deg()),
        helicopter: true,
        at: Instant::now(),
    };
    let (ctx, _) = rt.block_on(st.build_ctx(
        xp.view(),
        Some(&on_strip),
        tower::Intent::RequestTakeoff,
    ));
    assert!(ctx.on_ground, "the state's ground flag, not a failed read's");
    assert!(ctx.own_on_runway, "the state's position is on the strip");
    assert!(ctx.helicopter, "the state's type");

    let airborne = AircraftState {
        on_ground: false,
        helicopter: false,
        ..on_strip
    };
    let (ctx, _) = rt.block_on(st.build_ctx(
        xp.view(),
        Some(&airborne),
        tower::Intent::RequestTakeoff,
    ));
    assert!(!ctx.on_ground);
    assert!(!ctx.own_on_runway);
    assert!(!ctx.helicopter);
    Station::uninstall();
}

/// A crossing route along the strip is a backtrack and is judged on the
/// time it holds the strip (BIEG 03, 2026-10-08): the legs still to drive
/// on the pavement count, the ones behind do not; a 45 m crossing keeps
/// the crossing's 3 nm, a 1,850 m backtrack wants an arrival over 13 nm
/// out, and circuit traffic level at 1,000 ft blocks it.
#[test]
fn a_backtrack_is_judged_on_the_time_it_holds_the_strip() {
    use super::ctx::{arrival_within, backtrack_clear_nm, route_on_strip_m, BACKTRACK_M, CROSSING_FINAL_NM};
    let rwy = nav::Runway {
        ident_a: "03".into(),
        ident_b: "21".into(),
        lat_a: 65.27595,
        lon_a: -14.40972,
        lat_b: 65.29098,
        lon_b: -14.39287,
        width_m: 45.0,
        surface: 1,
    };
    let n = 12_u32;
    let along: Vec<(f64, f64)> = (0..=n)
        .rev()
        .map(|i| {
            let f = f64::from(i) / f64::from(n);
            (rwy.lat_a + f * (rwy.lat_b - rwy.lat_a), rwy.lon_a + f * (rwy.lon_b - rwy.lon_a))
        })
        .collect();
    let whole = route_on_strip_m(&along, None, &rwy);
    assert!(whole > 1500.0, "the backtrack is on the strip: {whole:.0}");
    let half = route_on_strip_m(&along, Some(along[6]), &rwy);
    assert!((half - whole / 2.0).abs() < 100.0, "only what is still owed: {half:.0}");
    let mid = (65.2835, -14.4013);
    let across = [(mid.0 + 0.0004, mid.1 - 0.0010), (mid.0 - 0.0004, mid.1 + 0.0010)];
    assert!(route_on_strip_m(&across, None, &rwy) < BACKTRACK_M, "a crossing");
    assert_eq!(backtrack_clear_nm(40.0), CROSSING_FINAL_NM);
    assert!(backtrack_clear_nm(whole) > 12.0);
    let target = |lat: f64, lon: f64, ele_ft: f64, vs_fpm: f64| TcasTarget {
        callsign: "TEST1".into(),
        lat,
        lon,
        ele_ft,
        on_ground: false,
        vs_fpm,
    };
    let downwind = [target(65.31, -14.3965, 1100.0, 0.0)];
    assert!(arrival_within(&downwind, &rwy, &[rwy.clone()], 75.0, backtrack_clear_nm(whole)));
    let climbing = [target(65.31, -14.3965, 1100.0, 900.0)];
    assert!(!arrival_within(&climbing, &rwy, &[rwy.clone()], 75.0, backtrack_clear_nm(whole)));
    let far = [target(65.0, -15.2, 3000.0, -700.0)];
    assert!(!arrival_within(&far, &rwy, &[rwy.clone()], 75.0, backtrack_clear_nm(whole)));
}

/// The realism review (2026-10-09), the spoken pieces in the field's
/// dialect: a check-in's climb (ICAO "climb altitude six thousand feet",
/// FAA "climb and maintain six thousand"; it was "climb six thousand"),
/// the wind (FAA "two seven zero at five"), frequencies (FAA "point"),
/// where to park (FAA "gate"), the traffic's type, and who is an airline.
#[test]
fn the_spoken_pieces_are_the_fields_dialect() {
    assert_eq!(check_in_climb("six thousand", Dialect::Icao), "climb altitude six thousand feet");
    assert_eq!(check_in_climb("six thousand feet", Dialect::Icao), "climb altitude six thousand feet");
    assert_eq!(check_in_climb("flight level eight zero", Dialect::Icao), "climb flight level eight zero");
    assert_eq!(check_in_climb("six thousand", Dialect::Faa), "climb and maintain six thousand");
    assert_eq!(
        check_in_climb("flight level one eight zero", Dialect::Faa),
        "climb and maintain flight level one eight zero"
    );
    assert_eq!(spoken_wind(270.0, 5.0, Dialect::Icao), "two seven zero degrees, five knots");
    assert_eq!(spoken_wind(270.0, 5.0, Dialect::Faa), "two seven zero at five");
    assert_eq!(spoken_wind(270.0, 2.0, Dialect::Faa), "calm");
    assert_eq!(atis::spoken_mhz_in(121800, Dialect::Faa), "one two one point eight");
    assert_eq!(atis::spoken_mhz_in(121800, Dialect::Icao), "one two one decimal eight");
    assert_eq!(spoken_stand_in("12", Dialect::Faa), "gate one two");
    assert_eq!(spoken_stand_in("12", Dialect::Icao), "stand one two");
    assert_eq!(spoken_stand_in("Ramp 3", Dialect::Faa), "ramp three");
    assert_eq!(spoken_type("A320").as_deref(), Some("A320"));
    assert_eq!(spoken_type("A20N").as_deref(), Some("A320"));
    assert_eq!(spoken_type("B738").as_deref(), Some("Boeing 737"));
    assert_eq!(spoken_type("AT76").as_deref(), Some("ATR"));
    assert_eq!(spoken_type("C172").as_deref(), Some("Cessna"));
    assert_eq!(spoken_type("ZZZZ"), None, "an unknown type is never guessed");
    let none: &[(String, String)] = &[];
    assert!(crate::callsign::is_airline_telephony_with("Ryanair four five two one", none));
    assert!(crate::callsign::is_airline_telephony_with("Speedbird two zero eight five heavy", none));
    assert!(!crate::callsign::is_airline_telephony_with("November one seven two Sierra Papa", none));
    assert!(!crate::callsign::is_airline_telephony_with("Cessna three four five", none));
    assert!(!crate::callsign::is_airline_telephony_with("Ryanair", none));
    // LEMG, 2026-10-08: a raw flight id reached speech ("RYR4521, taxi to
    // holding point ..."); it is expanded, a spoken one kept.
    assert_eq!(crate::callsign::spoken_if_raw("RYR4521"), "Ryanair four five two one");
    assert_eq!(crate::callsign::spoken_if_raw("IBE3302"), "Iberia three three zero two");
    assert_eq!(
        crate::callsign::spoken_if_raw("Cessna three four five"),
        "Cessna three four five"
    );
}

/// The realism review (2026-10-09): the pressure is given ONCE per contact
/// (an IFR clearance and then the taxi clearance both said it), again when
/// it changes or is asked for; and the first reply that gives it to a
/// pilot who did not report the ATIS names the current letter ("information
/// Bravo is current, QNH ..."), once.
#[test]
fn the_pressure_is_given_once_and_the_atis_named_once() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let path = lxgb_apt_file();
    let wx = Wx {
        wind_from_deg: 250.0,
        wind_kts: 12.0,
    };
    *OWN_STRIP.squawk_watch.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *ATIS.lock().unwrap() = Some(AtisState {
        khz: 118950,
        airport: "Gibraltar".into(),
        runways: Vec::new(),
        letter: 'B',
        dialect: Dialect::Icao,
        edition: Some(atis::Snapshot {
            airport: "Gibraltar".into(),
            zulu_sec: 0.0,
            runway: "27".into(),
            wind_from_deg: 250.0,
            wind_kts: 12.0,
            visibility_sm: 10.0,
            qnh_hpa: 1017.0,
            oat_c: 20.0,
            dewpoint_c: None,
        }),
        last_spoken: None,
    });
    let mut st = Station::load(path.to_str().unwrap(), "LXGB").expect("LXGB loads");
    let mut ctx = scripted_ctx(&mut st, wx);
    ctx.on_ground = true;
    let cs = "November one seven two Sierra Papa";
    let r = st.route_keyed("gibraltar tower, request start up", Service::Tower, &ctx, wx);
    assert_eq!(
        r,
        format!("{cs}, start-up approved, information Bravo is current, QNH one zero one seven.")
    );
    let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &ctx, wx);
    assert!(r.contains("taxi to holding point"), "{r}");
    assert!(!r.contains("QNH") && !r.contains("information"), "said once: {r}");
    // Asked for, it is said.
    let r = st.route_keyed("gibraltar tower, say QNH", Service::Tower, &ctx, wx);
    assert!(r.contains("QNH one zero one seven"), "{r}");
    assert!(!r.contains("information"), "the letter is named once: {r}");
    // A new pressure is given again.
    let changed = tower::Ctx {
        qnh: "QNH one zero one eight".into(),
        ..ctx.clone()
    };
    let r = st.route_keyed("gibraltar tower, request taxi", Service::Tower, &changed, wx);
    assert!(r.contains("QNH one zero one eight"), "{r}");
    *ATIS.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
}

/// The realism review (2026-10-09): the tower's go-around ORDER is said
/// twice ("go around, I say again, go around", CAP 413 / Doc 4444) and
/// tells the arrival what it flies: an airliner the missed approach and
/// the radar unit (published where the field's nav data has the ILS),
/// circuit traffic the circuit in the field's words. It was "climb to
/// circuit altitude, report downwind" to every arrival.
#[test]
fn the_go_around_order_tells_the_arrival_what_it_flies() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let saved = (
        ACTIVE_ICAO.lock().unwrap().clone(),
        ACTIVE_PANEL.lock().unwrap().clone(),
        CONTROLLER_FREQS.lock().unwrap().clone(),
        ILS_FACTS.lock().unwrap().clone(),
    );
    *ACTIVE_ICAO.lock().unwrap() = Some("LEMG".into());
    *ACTIVE_PANEL.lock().unwrap() = Some(("Malaga".into(), 52.0, Vec::new()));
    *CONTROLLER_FREQS.lock().unwrap() = vec![
        (118150, StationSpeaker::Tower),
        (123450, StationSpeaker::Approach),
    ];
    *ILS_FACTS.lock().unwrap() = None;
    let strip = FlightStrip::new();
    let airline = "Ryanair four five two one";
    assert_eq!(
        go_around_order_line(&strip, airline, "13", None),
        format!(
            "{airline}, go around, I say again, go around, traffic on the runway. Climb \
             altitude three thousand feet, runway heading, contact Malaga Approach on one two \
             three decimal four five."
        )
    );
    *ILS_FACTS.lock().unwrap() = Some(std::sync::Arc::new(IlsFacts {
        ils: vec![nav::AirportIls {
            runway: "13".into(),
            freq_10khz: 10990,
            course_mag: 130.0,
            gs_deg: Some(3.0),
        }],
        platform_ft: Default::default(),
    }));
    let r = go_around_order_line(&strip, airline, "13", None);
    assert!(
        r.ends_with("Fly the published missed approach, contact Malaga Approach on one two three \
                     decimal four five."),
        "{r}"
    );
    assert!(!r.contains("circuit") && !r.contains("downwind"), "{r}");
    // The same airline working the circuit, and a light aircraft: the
    // circuit's climb; "circuit height" in the UK; FAA "pattern altitude".
    strip.circuits.store(true, Ordering::SeqCst);
    let r = go_around_order_line(&strip, airline, "13", Some("12"));
    assert!(
        r.ends_with("Climb to circuit altitude, runway heading, expect runway one two, report \
                     downwind."),
        "{r}"
    );
    let ga = FlightStrip::new();
    *ACTIVE_ICAO.lock().unwrap() = Some("EGKK".into());
    let r = go_around_order_line(&ga, "Golf Alpha Bravo Charlie Delta", "26L", None);
    assert!(r.contains("go around, I say again, go around"), "{r}");
    assert!(r.contains("Climb to circuit height, runway heading, report downwind."), "{r}");
    *ACTIVE_ICAO.lock().unwrap() = Some("KSFO".into());
    let r = go_around_order_line(&ga, "November one two three", "28L", None);
    assert_eq!(
        r,
        "November one two three, go around, traffic on the runway. Fly runway heading, climb \
         to pattern altitude, report downwind."
    );
    *ACTIVE_ICAO.lock().unwrap() = saved.0;
    *ACTIVE_PANEL.lock().unwrap() = saved.1;
    *CONTROLLER_FREQS.lock().unwrap() = saved.2;
    *ILS_FACTS.lock().unwrap() = saved.3;
}

/// Doc 4444 12.3.1 traffic information: the direction of flight and the
/// type where known ("traffic, two o'clock, four miles, westbound, A320,
/// 1000 feet above"), and a target BEHIND us and moving away is never
/// called: the next one is.
#[test]
fn traffic_information_names_direction_and_type_and_skips_a_diverging_tail() {
    // Slot 0 is us (heading 090); 1 is half a mile behind flying away
    // (track 270); 2 is four miles at two o'clock, westbound, an A320.
    let dist = [0.0, 0.5 * 1852.0, 4.0 * 1852.0];
    let bear = [0.0, 180.0, 60.0];
    let alt = [0.0, 0.0, 1000.0 / 3.28084];
    let wow = [0.0, 0.0, 0.0];
    let tracks = [None, Some(270.0), Some(270.0)];
    let types = [None, None, Some("A320".to_string())];
    let f = select_traffic_factor_with(&dist, &bear, &alt, &wow, &tracks, Some(90.0), &types)
        .expect("the one ahead is a factor");
    assert_eq!(f.clock, 2);
    assert_eq!(f.bound, Some("westbound"));
    assert_eq!(
        advisory_line(&f),
        "traffic, 2 o'clock, 4 miles, westbound, A320, 1000 feet above."
    );
    // The same tail flying TOWARD us (track 090, overtaking) is called.
    let tracks = [None, Some(90.0), Some(270.0)];
    let f = select_traffic_factor_with(&dist, &bear, &alt, &wow, &tracks, Some(90.0), &types)
        .expect("a factor");
    assert_eq!(f.clock, 6, "an overtaking tail is a factor");
    assert_eq!(f.bound, Some("eastbound"));
    // No track known: judged as before, no direction said.
    let f = select_traffic_factor(&dist, &bear, &alt, &wow).expect("a factor");
    assert_eq!(f.clock, 6);
    assert_eq!(advisory_line(&f), "traffic, 6 o'clock, under a mile, same altitude.");
    assert_eq!(bound_word(0.0), "northbound");
    assert_eq!(bound_word(135.0), "south eastbound");
    assert_eq!(bound_word(350.0), "northbound");
}

/// Our own fleet's TCAS targets carry no heading: the track is the
/// target's movement since it was last seen, and its type the type table's.
#[test]
fn a_picture_target_s_track_comes_from_its_last_sighting() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *TRACK_MEMORY.lock().unwrap() = None;
    *TRAFFIC_TYPES.lock().unwrap() = None;
    note_traffic_type("VLG1311", "A320");
    let t0 = Instant::now();
    let at = |lon: f64| TcasTarget {
        callsign: "VLG1311".into(),
        lat: 36.70,
        lon,
        ele_ft: 3000.0,
        on_ground: false,
        vs_fpm: 0.0,
    };
    // We are at 36.68 N 4.50 W heading north; the target is NE of us,
    // flying west.
    let first = factor_from_picture_at(36.68, -4.50, 3000.0, 0.0, &[at(-4.46)], t0)
        .expect("a factor");
    assert_eq!(first.bound, None, "one sighting has no track");
    assert_eq!(first.type_spoken.as_deref(), Some("A320"));
    let later = t0 + std::time::Duration::from_secs(10);
    let f = factor_from_picture_at(36.68, -4.50, 3000.0, 0.0, &[at(-4.465)], later)
        .expect("a factor");
    assert_eq!(f.bound, Some("westbound"));
    assert!(advisory_line(&f).contains("westbound, A320, same altitude"), "{}", advisory_line(&f));
    *TRACK_MEMORY.lock().unwrap() = None;
    *TRAFFIC_TYPES.lock().unwrap() = None;
}

/// LFPG 09R (the traffic hour sweep, 2026-10-10): an arrival stopped 66 m
/// south of the runway, nearest a node of exit Y3 inside the runway's strip,
/// was routed back up Y3 through the strip and held short of the runway it
/// had just left for the rest of the hour. Its stand route now starts from
/// the nearest node clear of the strip, with no crossing.
/// `cargo test -p atc --release -- --ignored real_install_lfpg_off_strip --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_lfpg_off_strip_stand_route_needs_no_crossing() {
    let apt = crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    if !std::path::Path::new(&apt).exists() {
        return;
    }
    let st = Station::load(&apt, "LFPG").expect("LFPG");
    let net = nav::find_taxi_net(&apt, "LFPG").unwrap().expect("its network");
    let (lat, lon) = (49.02153743889608, 2.542026196488463);
    let far = [(0.0, 0.0)];
    let plan = stand_route_avoiding(&net, &st.apt.runways, lat, lon, None, &far).expect("a route");
    println!("{} {:?} via {:?}", plan.holding_point, plan.crossings, plan.via);
    assert!(plan.crossings.is_empty(), "{:?}", plan.crossings);
}

/// #263: the COACH speaks the field's dialect, and every FAA suggestion
/// parses as the ICAO one it replaces (a coached call the tower cannot
/// hear is the coach's own failure mode): "holding short runway ... at
/// ...", "inbound for landing", "clear of runway ...", "point" frequencies.
#[test]
fn the_coach_speaks_the_fields_dialect() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_PANEL.lock().unwrap() = Some((
        "Boston".into(),
        20.0,
        vec![("tower".to_string(), 128_800_i64)],
    ));
    *ATIS.lock().unwrap() = None;
    let cs = "November one seven two Sierra Papa";
    let snap = |state| ClearanceSnap {
        state,
        callsign: cs.into(),
        runway_ident: "27".into(),
        pending_readback: None,
        holds_runway: false,
    };
    *OWN_STRIP.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(42.36, -71.0), (42.36, -71.01)],
        via: "Alpha".into(),
        seg: vec![("A".into(), false)],
        holding_point: "A1".into(),
        lead_on: vec![],
    });
    let both = |state: Option<tower::State>, on_ground: bool| {
        *OWN_STRIP.clearance.lock().unwrap() = state.map(snap);
        let mut out = Vec::new();
        for d in [Dialect::Icao, Dialect::Faa] {
            *ACTIVE_DIALECT.lock().unwrap() = d;
            let s = suggest_call(cs, on_ground, Some(6.0), Some(121_500)).expect("a suggestion");
            out.push((s.call.expect("a call"), s.first.unwrap_or_default()));
        }
        out
    };
    let ready = both(Some(tower::State::TaxiCleared), true);
    assert!(ready[0].0.contains("holding point Alpha one"), "{:?}", ready[0]);
    assert_eq!(
        ready[1].0,
        format!("{cs}, holding short runway two seven at Alpha one, ready for departure.")
    );
    assert!(ready[1].1.contains("one two eight point eight"), "{:?}", ready[1]);
    assert!(ready[0].1.contains("one two eight decimal eight"), "{:?}", ready[0]);
    let inbound = both(None, false);
    assert!(inbound[1].0.contains("inbound for landing"), "{:?}", inbound[1]);
    assert!(!inbound[1].0.contains("join"), "{:?}", inbound[1]);
    let vacated = both(Some(tower::State::ClearedToLand), true);
    assert_eq!(
        vacated[1].0,
        format!("{cs}, clear of runway two seven, request taxi to parking.")
    );
    for pair in [&ready, &vacated, &inbound] {
        assert_eq!(
            tower::parse_intent(&pair[0].0),
            tower::parse_intent(&pair[1].0),
            "{pair:?}"
        );
    }
    *ACTIVE_DIALECT.lock().unwrap() = Dialect::Icao;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
}

/// #263: the off-route correction at an FAA field holds short of the
/// runway, never "holding point".
#[test]
fn the_off_route_correction_holds_short_at_an_faa_field() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *ACTIVE_FIELD.lock().unwrap() = Some(("Gibraltar".into(), 36.1512, -5.3554));
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
    *ACTIVE_DIALECT.lock().unwrap() = Dialect::Faa;
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "November one seven two Sierra Papa".into(),
        runway_ident: "09".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *OWN_STRIP.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(36.1600, -5.3600), (36.1600, -5.3500)],
        via: "Alpha, Bravo".into(),
        seg: vec![("A".into(), false)],
        holding_point: "A1".into(),
        lead_on: vec![],
    });
    let t0 = Instant::now();
    let sec = std::time::Duration::from_secs;
    assert!(compliance_tick(36.1600, -5.3600, 15.0, true, 0.0, true, t0).is_none());
    let call = compliance_tick(36.1580, -5.3550, 15.0, true, 12.0, true, t0 + sec(2))
        .expect("off-route nudge");
    assert!(
        call.line.ends_with("Taxi via Alpha, Bravo, hold short of runway zero niner."),
        "{}",
        call.line
    );
    assert!(!call.line.contains("holding point"), "{}", call.line);
    *ACTIVE_DIALECT.lock().unwrap() = Dialect::Icao;
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *COMPLIANCE.lock().unwrap() = ComplianceState::NEW;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *ACTIVE_FIELD.lock().unwrap() = None;
}

/// #263: the frequency briefing reads "point" at an FAA field.
#[test]
fn the_frequency_briefing_reads_the_fields_dialect() {
    // The dialect is process-wide: other tests read "decimal" while this
    // one holds FAA (four coach and taxi tests read "point", 2026-10-13).
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut st = Station::load(lxgb_apt_file().to_str().unwrap(), "LXGB").expect("LXGB loads");
    assert!(st.briefing_text(None).contains("decimal"), "{}", st.briefing_text(None));
    st.set_dialect(Dialect::Faa);
    let b = st.briefing_text(None);
    st.set_dialect(Dialect::Icao);
    assert!(b.contains("point") && !b.contains("decimal"), "{b}");
}

/// A DEPARTURE CLEARED FOR TAKEOFF AND ROLLING IS JUDGED ON ITS ROLL, never
/// as a lander rolling out. LEMG 13 (2026-10-13) `go_around_judged` at t
/// 1791902325.5: RYR5024 (cleared for takeoff) 824 m down at about 90 kt,
/// cross 0, read `clear_s` 40.0 (a rollout: fast, not seen accelerating)
/// against our `own_s` 35.0, and we were sent round; it lifted off at t
/// 2335.3, 25 s before our threshold. As a departure it is about 15 s from
/// liftoff: no go-around. An occupant nobody cleared keeps the rollout
/// reading.
#[test]
fn lemg13_a_departure_rolling_is_judged_on_its_roll_not_as_a_rollout() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = super::strip_tests::ClearStrips;
    let reset = || {
        *OWN_STRIP.go_around_last.lock().unwrap() = None;
        *OWN_STRIP.pending_go_around.lock().unwrap() = false;
        *OWN_STRIP.landing_hold.lock().unwrap() = None;
        *STRIP_SEEN.lock().unwrap() = None;
        OWN_STRIP.own_fixes.lock().unwrap().clear();
    };
    reset();
    *ACTIVE_RUNWAYS.lock().unwrap() = lemg_runways();
    let cleared = |state, callsign: &str| ClearanceSnap {
        state,
        callsign: callsign.into(),
        runway_ident: "13".into(),
        pending_readback: None,
        holds_runway: true,
    };
    *OWN_STRIP.clearance.lock().unwrap() = Some(cleared(
        tower::State::ClearedToLand,
        "November eight one six November November",
    ));
    let base = Instant::now();
    let at = |s: f64| base + std::time::Duration::from_secs_f64(s);
    // Rolling steadily at 90 kt (46.3 m/s): 731, 777, 824 m down.
    let ryr = |along: f64| ghost_at("RYR5024", lemg13_point(along, 0.0));
    // Us 1.3 nm out at 134 kt: 35 s.
    let own_at = |t: f64| {
        let d = 1.3 * 1852.0 + (2.0 - t) * 68.9;
        let (lat, lon) = lemg13_point(-d, 0.0);
        note_own_fix(lat, lon, false, at(t));
    };
    let judge = || {
        note_ground_traffic(&[ryr(731.4)], at(0.0));
        note_ground_traffic(&[ryr(777.7)], at(1.0));
        own_at(1.0);
        own_at(2.0);
        let exits = strip_exits(&[ryr(824.0)], Some(&lemg_runways()[1]), "13", at(2.0));
        let own = own_time_to_threshold(&lemg_runways()[1], "13", at(2.0)).unwrap();
        assert!((34.0..36.0).contains(&own.1), "{own:?}");
        let call = go_around_tick(&[ryr(824.0)], 300.0, false, true, at(2.0));
        (exits[0].exit, call)
    };
    // Nobody cleared it: the recorded reading, and the recorded order.
    let (exit, call) = judge();
    assert_eq!(exit, StripExit::Clearing(ROLLOUT_CLEAR_S));
    assert!(call.is_some_and(|c| c.line.contains("go around")));
    reset();
    // Its strip cleared for takeoff, seen where it rolls: its roll.
    let t = open_traffic_strip(2);
    *t.clearance.lock().unwrap() = Some(cleared(
        tower::State::TakeoffCleared,
        "Ryanair five zero two four",
    ));
    let (lat, lon) = lemg13_point(824.0, 0.0);
    note_traffic_state(2, AircraftState::at_position(lat, lon, true, 90.0));
    let (exit, call) = judge();
    match exit {
        StripExit::Clearing(s) => assert!((13.0..17.0).contains(&s), "{s}"),
        other => panic!("{other:?}"),
    }
    assert!(call.is_none(), "{:?}", call.map(|c| c.line));
    // Standing anywhere, a departure cleared for takeoff is lined up.
    let r = &lemg_runways()[1];
    let p = |along: f64, s: f64| {
        let (lat, lon) = lemg13_point(along, 0.0);
        (lat, lon, s)
    };
    assert_eq!(
        strip_exit_as(r, "13", &[p(1600.0, -2.0), p(1600.0, 0.0)], true),
        StripExit::LinedUp(roll_to_liftoff_s(0.0))
    );
    assert_eq!(
        strip_exit(r, "13", &[p(1600.0, -2.0), p(1600.0, 0.0)]),
        StripExit::Stays
    );
    reset();
    *OWN_STRIP.clearance.lock().unwrap() = None;
    *ACTIVE_RUNWAYS.lock().unwrap() = Vec::new();
}
