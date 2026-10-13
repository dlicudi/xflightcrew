//! The runway book against the functions it replaced: the four queries as
//! they stood before the book (kept verbatim below as oracles) answer every
//! cell of the matrix exactly as the book does.

use super::tests::SIDE_STATE_LOCK;
use super::*;

/// The four queries as they stood at 1738653, verbatim: the oracles.
mod oracle {
    use super::super::*;

/// The strip the PLAYER holds for the party line: the one it stands on, or,
/// on the ground, the one the tower's book still gives it (cleared onto it,
/// or landed and not yet reported off). A ghost landing or departing that
/// strip is not narrated a clearance over our head. The "runway vacated"
/// report frees it, as it frees a real tower's runway, before the taxi
/// request that used to be the only thing that did (Liege 2026-09-22).
pub fn player_blocked_strip(on_ground: bool, lat: f64, lon: f64) -> Option<String> {
    if !on_ground {
        return None;
    }
    on_active_runway(lat, lon).or_else(|| {
        OWN_STRIP.clearance
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .filter(|c| c.holds_runway && !c.runway_ident.is_empty())
            .map(|c| c.runway_ident.clone())
    })
}

/// What OUR clearance claims of a strip, for the party line, on the ground
/// or in the air: `(end, whole)`. Cleared to land and not yet reported off,
/// the WHOLE strip is ours (nobody else is cleared onto it, either end);
/// sequenced for it in any other way (taxi, line-up, takeoff, the circuit's
/// Inbound, a go-around's), its DIRECTION is (nobody is cleared off or onto
/// its reciprocal). `None` without a clearance or a runway.
pub(super) fn own_strip_claim() -> Option<(String, bool)> {
    let clr = OWN_STRIP.clearance.lock().unwrap_or_else(|e| e.into_inner());
    let c = clr.as_ref().filter(|c| !c.runway_ident.is_empty())?;
    match c.state {
        tower::State::ClearedToLand if c.holds_runway => Some((c.runway_ident.clone(), true)),
        // Lined up or cleared for takeoff, the runway is ours alone: one
        // departure on it at a time, either direction (EGKK 08R, sortie
        // 1790601822-full_auto, 2026-09-28: "Easy eight six eight zero,
        // runway zero eight right, cleared for takeoff" 1.6 s after our own
        // takeoff clearance, while we lined up).
        tower::State::LinedUp | tower::State::TakeoffCleared => {
            Some((c.runway_ident.clone(), true))
        }
        tower::State::TaxiCleared | tower::State::Inbound => {
            Some((c.runway_ident.clone(), false))
        }
        _ => None,
    }
}


/// Whether a ghost's movement on `ghost_end` conflicts with our claim: the
/// same strip, and either the whole strip is ours or the ghost uses the
/// other direction. One strip, one direction (LEMG 2026-09-28).
pub(super) fn conflicts_with_claim(ghost_end: &str, claim: &(String, bool)) -> bool {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    let same = matches!(
        (strip(&runways, ghost_end), strip(&runways, &claim.0)),
        (Some(x), Some(y)) if std::ptr::eq(x, y)
    );
    same && (claim.1 || ghost_end != claim.0)
}



/// Does the tower's own book put us on the runway we stand on? The runway
/// incursion watch ("traffic on final, exit the runway") asks this before it
/// shouts: a standing takeoff clearance or line-up; a landing clearance on
/// THIS strip not yet reported vacated (the rollout and the vacate, EGKK 26L
/// 2026-09-27: "exit the runway immediately" from 20 s after touchdown to
/// the vacate, eight times); a runway start; the strip we LANDED on, cleared
/// or not, until we leave it (EGKK-EGBB 15, 2026-09-28: four "exit the
/// runway" during the vacate of an uncleared landing); a crossing on the
/// cleared route that keeps moving. Anything else on a strip with traffic on
/// final is the incursion it is.
pub(crate) fn runway_occupancy_authorised(lat: f64, lon: f64, gs_kts: f64) -> bool {
    let Some(here) = strip_under(lat, lon) else {
        return false;
    };
    let clr = OWN_STRIP.clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.runway_ident.clone(), c.holds_runway));
    if let Some((state, ident, holds)) = &clr {
        if matches!(state, tower::State::LinedUp | tower::State::TakeoffCleared) {
            return true;
        }
        if *state == tower::State::ClearedToLand && *holds {
            let cleared = {
                let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
                strip(&runways, ident).map(|r| r.ident_a.clone())
            };
            if ident.is_empty() || cleared.as_deref() == Some(here.as_str()) {
                return true;
            }
        }
    }
    let (started_here, landed_here) = {
        let w = OWN_STRIP.compliance_flight.lock().unwrap_or_else(|e| e.into_inner());
        (
            w.runway_start.as_deref() == Some(here.as_str()),
            w.landed_strip.as_deref() == Some(here.as_str()) && gs_kts >= LANDED_VACATING_KTS,
        )
    };
    // A runway start, or the strip we landed on and are still rolling off
    // (cleared or not: an aircraft vacating is not an incursion).
    if started_here || landed_here {
        return true;
    }
    let arrival_taxi = *OWN_STRIP.taxi_to_stand.lock().unwrap_or_else(|e| e.into_inner());
    let cleared_taxi = clr
        .as_ref()
        .is_some_and(|(s, _, _)| *s == tower::State::TaxiCleared)
        || arrival_taxi;
    cleared_taxi && gs_kts >= 3.0 && on_cleared_runway_crossing(lat, lon)
}
}

/// Our strip and its close-parallel twin (EGKK's pair, 190 m apart).
pub(super) fn book_runways() -> Vec<nav::Runway> {
    vec![
        nav::Runway {
            ident_a: "08R".into(),
            ident_b: "26L".into(),
            lat_a: 51.1450993,
            lon_a: -0.2123748,
            lat_b: 51.1514994,
            lon_b: -0.1659296,
            width_m: 46.0,
            surface: 1,
        },
        nav::Runway {
            ident_a: "08L".into(),
            ident_b: "26R".into(),
            lat_a: 51.1468901,
            lon_a: -0.2126029,
            lat_b: 51.1518321,
            lon_b: -0.1767408,
            width_m: 46.0,
            surface: 1,
        },
    ]
}

/// The point `f` of the way along a strip from its A end.
pub(super) fn along(r: &nav::Runway, f: f64) -> (f64, f64) {
    (r.lat_a + f * (r.lat_b - r.lat_a), r.lon_a + f * (r.lon_b - r.lon_a))
}

fn standing_ghost_at(cs: &str, (lat, lon): (f64, f64), now: Instant) {
    note_ground_traffic(
        &[TcasTarget {
            callsign: cs.into(),
            lat,
            lon,
            ele_ft: 200.0,
            on_ground: true,
            vs_fpm: 0.0,
        }],
        now,
    );
}

fn clear_ghost_memory() {
    *STANDING.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *STRIP_SEEN.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// THE BOOK ANSWERS AS THE FUNCTIONS IT REPLACED, cell for cell: every
/// tower state, with and without `holds_runway`, on five runway names (our
/// two ends, the twin's, none, an unknown one) or no clearance; on our
/// strip at either end, on the twin, off the pavement, on the ground or not,
/// stopped or moving; an arrival taxi or not, a cleared route crossing our
/// strip or none; a runway start or a landed strip on ours or the twin's;
/// a ghost standing on our end, the other end, the twin or nowhere; and a
/// ghost's movement on each of the five names.
#[test]
fn the_runway_book_answers_as_the_functions_it_replaced() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let runways = book_runways();
    *ACTIVE_RUNWAYS.lock().unwrap() = runways.clone();
    let (ours, twin) = (&runways[0], &runways[1]);
    let ours_a = along(ours, 0.2);
    let ours_b = along(ours, 0.8);
    let on_twin = along(twin, 0.3);
    let mid = along(ours, 0.5);
    let off = (mid.0 - 300.0 / 111_320.0, mid.1);
    let positions = [("ours_a", ours_a), ("ours_b", ours_b), ("twin", on_twin), ("off", off)];
    let ends = ["08R", "26L", "08L", "26R", ""];
    let names = ["08R", "26L", "08L", "", "33"];
    let states = [
        tower::State::Parked,
        tower::State::TaxiCleared,
        tower::State::LinedUp,
        tower::State::TakeoffCleared,
        tower::State::Inbound,
        tower::State::ClearedToLand,
    ];
    let mut clearances = vec![None];
    for s in states {
        for holds in [false, true] {
            for n in names {
                clearances.push(Some((s, holds, n)));
            }
        }
    }
    // A cleared route crossing our strip where `ours_a` stands.
    let crossing = TaxiWatch {
        path: vec![(ours_a.0 + 0.0013, ours_a.1), (ours_a.0 - 0.0013, ours_a.1)],
        via: "Alpha".into(),
        seg: vec![("A".into(), false)],
        holding_point: "A1".into(),
        lead_on: Vec::new(),
    };
    let compliance = [
        (None, None),
        (Some("08R"), None),
        (Some("08L"), None),
        (None, Some("08R")),
        (None, Some("08L")),
    ];
    let ghosts = [None, Some(ours_a), Some(ours_b), Some(on_twin)];
    let now = Instant::now();
    let mut cells = 0usize;
    for ghost in ghosts {
        clear_ghost_memory();
        if let Some(at) = ghost {
            standing_ghost_at("GHOST1", at, now);
        }
        for clr in &clearances {
            *OWN_STRIP.clearance.lock().unwrap() = clr.map(|(state, holds, n)| ClearanceSnap {
                state,
                callsign: "Test".into(),
                runway_ident: n.into(),
                pending_readback: None,
                holds_runway: holds,
            });
            for arrival_taxi in [false, true] {
                *OWN_STRIP.taxi_to_stand.lock().unwrap() = arrival_taxi;
                for route in [None, Some(crossing.clone())] {
                    let routed = route.is_some();
                    *OWN_STRIP.taxi_route.lock().unwrap() = route;
                    for (start, landed) in compliance {
                        {
                            let mut w = OWN_STRIP.compliance_flight.lock().unwrap();
                            w.runway_start = start.map(String::from);
                            w.landed_strip = landed.map(String::from);
                        }
                        let old_claim = oracle::own_strip_claim();
                        assert_eq!(own_strip_claim(), old_claim, "{clr:?}");
                        for (pname, (lat, lon)) in positions {
                            for on_ground in [true, false] {
                                for gs in [0.0, 10.0] {
                                    cells += 1;
                                    let cell = || {
                                        format!(
                                            "clr {clr:?} taxi_in {arrival_taxi} route {routed} \
                                             start {start:?} landed {landed:?} ghost {ghost:?} \
                                             at {pname} ground {on_ground} gs {gs}"
                                        )
                                    };
                                    let blocked = oracle::player_blocked_strip(on_ground, lat, lon);
                                    let authorised =
                                        oracle::runway_occupancy_authorised(lat, lon, gs);
                                    assert_eq!(
                                        player_blocked_strip(on_ground, lat, lon),
                                        blocked,
                                        "blocked: {}",
                                        cell()
                                    );
                                    assert_eq!(
                                        runway_occupancy_authorised(lat, lon, gs),
                                        authorised,
                                        "authorised: {}",
                                        cell()
                                    );
                                    // One book over the whole state, as a tick holds it.
                                    let book = RunwayBook::now(Some(AircraftState {
                                        lat,
                                        lon,
                                        alt_ft: f64::NAN,
                                        on_ground,
                                        gs_kts: gs,
                                        psi_deg: None,
                                        helicopter: false,
                                        at: now,
                                    }));
                                    assert_eq!(
                                        book.blocked_strip(FlightId::Own),
                                        blocked,
                                        "book blocked: {}",
                                        cell()
                                    );
                                    assert_eq!(
                                        book.occupancy_authorised(FlightId::Own),
                                        authorised,
                                        "book authorised: {}",
                                        cell()
                                    );
                                    assert_eq!(
                                        book.strip_claim(FlightId::Own)
                                            .map(|c| (c.end.clone(), c.whole_strip)),
                                        old_claim,
                                        "book claim: {}",
                                        cell()
                                    );
                                    for end in ends {
                                        let old = old_claim
                                            .as_ref()
                                            .is_some_and(|c| oracle::conflicts_with_claim(end, c));
                                        assert_eq!(
                                            book.conflicts(FlightId::Own, end),
                                            old,
                                            "conflict on {end}: {}",
                                            cell()
                                        );
                                    }
                                    // The ghost is in the book, and it is nobody's strip.
                                    assert_eq!(
                                        book.claims()
                                            .iter()
                                            .any(|c| c.kind == ClaimKind::Observed),
                                        ghost.is_some(),
                                        "observed: {}",
                                        cell()
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    assert_eq!(cells, 61 * 2 * 2 * 5 * 4 * 4 * 2 * 2, "the matrix ran whole");
    // Spot checks that the matrix exercised what it names.
    *OWN_STRIP.taxi_route.lock().unwrap() = None;
    *OWN_STRIP.taxi_to_stand.lock().unwrap() = false;
    *OWN_STRIP.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::ClearedToLand,
        callsign: "Test".into(),
        runway_ident: "08R".into(),
        pending_readback: None,
        holds_runway: true,
    });
    let book = RunwayBook::now(None);
    assert!(book.conflicts(FlightId::Own, "26L"), "the whole strip is ours");
    assert!(!book.conflicts(FlightId::Own, "08L"), "the twin is another strip");
    assert!(!book.conflicts(FlightId::Own, "26R"));
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::Inbound,
        callsign: "Test".into(),
        runway_ident: "08R".into(),
        pending_readback: None,
        holds_runway: false,
    });
    let book = RunwayBook::now(None);
    assert!(book.conflicts(FlightId::Own, "26L"), "sequenced: its reciprocal");
    assert!(!book.conflicts(FlightId::Own, "08R"), "sequenced: its own direction is free");
    clear_ghost_memory();
    Station::uninstall();
}

/// A TRAFFIC FLIGHT'S POSITION IS READ BY THE RULES OURS IS: the state its
/// producer reported ([`note_traffic_state`]) makes the same claims as the
/// state handed in for a flight, cell for cell, on our strip at either end,
/// on the twin, off the pavement, on the ground or not, stopped or moving;
/// and only a flight standing on our strip blocks it, never one on the twin
/// or one in the air over it. A runway start, a vacate and a moving cleared
/// crossing are its claims too.
#[test]
fn a_traffic_position_claims_by_the_rules_ours_does() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = super::strip_tests::ClearStrips;
    Station::uninstall();
    clear_ghost_memory();
    let runways = book_runways();
    *ACTIVE_RUNWAYS.lock().unwrap() = runways.clone();
    let (ours, twin) = (&runways[0], &runways[1]);
    let ours_a = along(ours, 0.2);
    let mid = along(ours, 0.5);
    let positions = [
        ("ours_a", ours_a),
        ("ours_b", along(ours, 0.8)),
        ("twin", along(twin, 0.3)),
        ("off", (mid.0 - 300.0 / 111_320.0, mid.1)),
    ];
    let mut cells = 0;
    for (pname, (lat, lon)) in positions {
        for on_ground in [true, false] {
            for gs in [0.0, 10.0] {
                cells += 1;
                STRIPS.lock().unwrap().clear();
                let state = AircraftState::at_position(lat, lon, on_ground, gs);
                note_traffic_state(1, state);
                let t = traffic_strip(1).expect("the report opened its strip");
                let book = RunwayBook::now(None);
                let handed = RunwayBook::compute(&[
                    BookEntry {
                        id: FlightId::Own,
                        strip: &OWN_STRIP,
                        state: None,
                    },
                    BookEntry {
                        id: FlightId::Traffic(1),
                        strip: &t,
                        state: Some(state),
                    },
                ]);
                let cell = format!("at {pname} ground {on_ground} gs {gs}");
                assert_eq!(book.claims(), handed.claims(), "{cell}");
                let occupying = book.claims().iter().any(|c| {
                    c.who == Claimant::Strip(FlightId::Traffic(1)) && c.kind == ClaimKind::Occupying
                });
                assert_eq!(occupying, on_ground && pname != "off", "{cell}");
                let on_ours = on_ground && pname.starts_with("ours");
                let on_twin = on_ground && pname == "twin";
                for end in ["08R", "26L"] {
                    assert_eq!(strip_held_by_traffic(&runways, end), on_ours, "{end} {cell}");
                }
                for end in ["08L", "26R"] {
                    assert_eq!(strip_held_by_traffic(&runways, end), on_twin, "{end} {cell}");
                }
            }
        }
    }
    assert_eq!(cells, 16, "the matrix ran whole");

    // The strip's own memory and route make the claims they make for ours.
    STRIPS.lock().unwrap().clear();
    note_traffic_state(2, AircraftState::at_position(ours_a.0, ours_a.1, true, 0.0));
    let t = traffic_strip(2).unwrap();
    let kinds = || -> Vec<ClaimKind> {
        RunwayBook::now(None)
            .claims()
            .iter()
            .filter(|c| c.who == Claimant::Strip(FlightId::Traffic(2)))
            .map(|c| c.kind)
            .collect()
    };
    assert_eq!(kinds(), vec![ClaimKind::Occupying]);
    assert!(!RunwayBook::now(None).occupancy_authorised(FlightId::Traffic(2)));
    t.compliance_flight.lock().unwrap().runway_start = Some("08R".into());
    assert!(kinds().contains(&ClaimKind::RunwayStart));
    assert!(RunwayBook::now(None).occupancy_authorised(FlightId::Traffic(2)));
    *t.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    t.compliance_flight.lock().unwrap().landed_strip = Some("08R".into());
    note_traffic_state(2, AircraftState::at_position(ours_a.0, ours_a.1, true, 10.0));
    assert!(kinds().contains(&ClaimKind::Vacating));
    *t.compliance_flight.lock().unwrap() = ComplianceFlight::NEW;
    *t.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::TaxiCleared,
        callsign: "Easy four five six".into(),
        runway_ident: "26R".into(),
        pending_readback: None,
        holds_runway: false,
    });
    *t.taxi_route.lock().unwrap() = Some(TaxiWatch {
        path: vec![(ours_a.0 + 0.0013, ours_a.1), (ours_a.0 - 0.0013, ours_a.1)],
        via: "Alpha".into(),
        seg: vec![("A".into(), false)],
        holding_point: "A1".into(),
        lead_on: Vec::new(),
    });
    assert!(kinds().contains(&ClaimKind::Crossing));
    assert!(RunwayBook::now(None).occupancy_authorised(FlightId::Traffic(2)));
    // A crossing traffic flight still stands on our strip: it blocks it.
    assert!(strip_held_by_traffic(&runways, "26L"));
    // None of it is ours.
    assert!(RunwayBook::now(None).strip_claim(FlightId::Own).is_none());
    Station::uninstall();
}

#[test]
fn a_landing_clearance_far_out_holds_only_its_direction() {
    let _guard = SIDE_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let _strips = super::strip_tests::ClearStrips;
    Station::uninstall();
    clear_ghost_memory();
    let runways = book_runways();
    *ACTIVE_RUNWAYS.lock().unwrap() = runways.clone();
    let ours = &runways[0];
    *OWN_STRIP.clearance.lock().unwrap() = Some(ClearanceSnap {
        state: tower::State::ClearedToLand,
        callsign: "Whiskey Alpha Tango five two four".into(),
        runway_ident: "26L".into(),
        pending_readback: None,
        holds_runway: true,
    });
    note_traffic_state(1, AircraftState::at_position(ours.lat_a, ours.lon_a - 0.01, true, 0.0));
    let t = traffic_strip(1).unwrap();
    let held = || strip_held_by_others(&runways, "26L", &t);
    let out = |nm: f64, at: Instant| AircraftState {
        at,
        ..AircraftState::at_position(ours.lat_b + nm / 60.0, ours.lon_b, false, 90.0)
    };
    // Unknown, as cleared: the whole strip.
    assert!(held());
    // LEJR 2026-10-09: cleared at 7 nm, then a reversal pattern. Far out,
    // the traffic lands and departs on the strip meanwhile.
    note_own_state(out(7.0, Instant::now()));
    assert!(!held(), "7 nm out holds only the direction");
    assert!(RunwayBook::now(None).conflicts(FlightId::Own, "08R"), "the reciprocal stays ours");
    note_own_state(out(2.0, Instant::now()));
    assert!(held(), "2 nm out holds the whole strip");
    // A position the link stopped noting is no position.
    note_own_state(out(7.0, Instant::now() - OWN_SEEN_FRESH * 2));
    assert!(held(), "a stale position is unknown");
    *OWN_STRIP.seen.lock().unwrap() = None;
    *OWN_STRIP.clearance.lock().unwrap() = None;
    Station::uninstall();
}
