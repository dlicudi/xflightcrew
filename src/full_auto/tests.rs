    /// EGKK 08R, sortie 1790624015-full_auto (#176): the tower's "contact
    /// London Gatwick Departure" reached the party line at +1082.9 (960 ft)
    /// while the takeoff flew until its handoff at +1119.9; only the enroute
    /// wait took it, so the readback went at +1120.2. The takeoff's wait now
    /// obeys it on the next poll, on the tower, long before the handoff.
    #[tokio::test(start_paused = true)]
    async fn the_takeoff_reads_back_the_departure_handoff_at_once() {
        use super::{wait_takeoff_working_the_radio, TakeoffWait, TAKEOFF_POLL_MS};
        use std::cell::Cell;
        let t0 = tokio::time::Instant::now();
        // Seconds since the sortie began; the takeoff wait opened at +1040.
        let now = move || 1040.0 + t0.elapsed().as_secs_f64();
        let (given, handed_over) = (1082.9, 1119.9);
        let obeyed = Cell::new(None::<f64>);
        let wait = wait_takeoff_working_the_radio(
            360.0,
            TAKEOFF_POLL_MS,
            || async { now() < handed_over },
            || async {
                // The party line holds the handoff from the moment it is
                // spoken until someone takes it.
                if obeyed.get().is_none() && now() >= given {
                    obeyed.set(Some(now()));
                    // The readback holds the frequency before the retune.
                    tokio::time::sleep(std::time::Duration::from_secs(4)).await;
                    return true;
                }
                false
            },
        )
        .await;
        assert_eq!(wait, TakeoffWait { finished: true, checked_in: true });
        let at = obeyed.get().expect("the handoff was obeyed in the takeoff");
        assert!(at - given <= TAKEOFF_POLL_MS as f64 / 1000.0 + 1e-6, "read back at +{at}");
        assert!(at < handed_over - 30.0, "not held for the takeoff's handoff: +{at}");
        // No handoff spoken (a circuit, an untowered field): the wait still
        // ends with the takeoff and nothing is owed.
        let t1 = tokio::time::Instant::now();
        let wait = wait_takeoff_working_the_radio(
            360.0,
            TAKEOFF_POLL_MS,
            || async { t1.elapsed().as_secs_f64() < 80.0 },
            || async { false },
        )
        .await;
        assert_eq!(wait, TakeoffWait { finished: true, checked_in: false });
    }

    /// The fly_route takeoff wait is the one that works the radio, and the
    /// clearance's ceiling is not put back over a check-in made inside it.
    #[test]
    fn the_takeoff_wait_takes_the_departure_handoff() {
        let src = crate::test_support::module_source("full_auto");
        let body = src.split("async fn fly_route(").nth(1).expect("fly_route");
        let takeoff = body.split("event(\"airborne\"").next().unwrap();
        assert!(takeoff.contains("wait_takeoff_working_the_radio("));
        assert!(takeoff.contains("take_departure_handoff()"));
        assert!(body.contains(".filter(|_| !checked_in)"));
    }

    /// EGKK, 2026-09-28 (#151): the arm found Malaga's station up at Gatwick
    /// stand 71. It arms the field's station once, waits for the main loop
    /// to install it, and refuses in words when it never comes.
    #[tokio::test(start_paused = true)]
    async fn the_arm_waits_for_the_departure_fields_station() {
        use super::{wait_departure_station, DepartureWait};
        use crate::party_line::DepartureStation as D;
        let want = || "EGKK".to_string();
        let poll = std::time::Duration::from_millis(250);
        let wait = std::time::Duration::from_secs(20);
        // Malaga up, Gatwick armed on the first look, installed on the fourth.
        let mut looks = 0;
        let mut arms = 0;
        let out = wait_departure_station(
            |arm| {
                looks += 1;
                arms += usize::from(arm);
                let v = match looks {
                    1 => D::Arm { want: want() },
                    2..=3 => D::Waiting { want: want() },
                    _ => D::Ready,
                };
                (v, Some("LEMG".to_string()))
            },
            wait,
            poll,
        )
        .await;
        assert_eq!(out, DepartureWait::Ready { armed: true });
        assert_eq!(looks, 4);
        assert_eq!(arms, 1, "armed once; the main loop installs it");
        // Up already: no wait, nothing armed.
        let out = wait_departure_station(|_| (D::Ready, Some("EGKK".into())), wait, poll).await;
        assert_eq!(out, DepartureWait::Ready { armed: false });
        // Never installed (a station that would not build): refused at the
        // deadline, naming both fields, asked to arm only once.
        let mut arms = 0;
        let t0 = tokio::time::Instant::now();
        let out = wait_departure_station(
            |arm| {
                arms += usize::from(arm);
                (D::Arm { want: want() }, Some("LEMG".into()))
            },
            wait,
            poll,
        )
        .await;
        assert_eq!(out, DepartureWait::Refused { want: want(), active: Some("LEMG".into()) });
        assert_eq!(arms, 1);
        assert!(t0.elapsed() >= wait && t0.elapsed() < wait + poll * 2, "{:?}", t0.elapsed());
        let line = super::departure_station_refusal("EGKK", Some("LEMG"));
        assert!(line.contains("EGKK") && line.contains("LEMG"), "{line}");
        assert!(!line.contains('\u{2014}') && !line.contains('\u{2013}'));
        // The check comes before the mission asks the tower for anything.
        let src = crate::test_support::module_source("full_auto");
        let check = src.find("if let Err(line) = departure_station(&xp).await").unwrap();
        let towered = src.find("let towered = crate::mode::atc_enabled()").unwrap();
        let callsign = src.find("let callsign = crate::crew_radio::crew_callsign(&xp).await;\n\n    // On a Flight").unwrap();
        assert!(check < callsign && check < towered);
    }

    fn sid(ident: &str, join_index: usize) -> crate::nav::SidChoice {
        crate::nav::SidChoice {
            ident: ident.into(),
            transition: "RW24".into(),
            join: "GOSAM".into(),
            join_index,
            fixes: vec!["GOSAM".into()],
        }
    }

    /// The IFR clearance as the station hands it down (`party_line::
    /// assigned_clearance`): GOSAM 1A off runway 24.
    fn gosam_1a() -> crate::party_line::IfrClearance {
        crate::party_line::IfrClearance {
            dest: "EGLL".into(),
            sid: Some("GOSAM1A".into()),
            runway: "24".into(),
            initial_alt_ft: 5000,
            squawk: 4521,
        }
    }

    #[test]
    fn a_circuit_tells_the_tower_it_is_one() {
        use super::{ready_call, taxi_call, Mission};
        use crate::tower::{parse_intent, wants_circuits, Intent};
        // LEMG 31 (sortie 1790584715-full_auto, 2026-09-28): the circuit
        // asked "request taxi" and "ready for departure", and the tower sent
        // it to Malaga Departure on the crosswind leg.
        let cs = Some("November eight one six November November".to_string());
        let circuit = Mission::Circuit { right: true };
        let taxi = crate::crew_radio::radio_call(&cs, &taxi_call(&circuit, Some("31")));
        assert!(!taxi.contains("three one"), "a circuit asks for circuits: {taxi}");
        assert_eq!(parse_intent(&taxi), Intent::RequestTaxi, "{taxi}");
        assert!(wants_circuits(&taxi), "{taxi}");
        let ready = crate::crew_radio::radio_call(&cs, ready_call(true));
        assert_eq!(parse_intent(&ready), Intent::RequestTakeoff, "{ready}");
        assert!(wants_circuits(&ready), "{ready}");
        // A departure never says it.
        let ready = crate::crew_radio::radio_call(&cs, ready_call(false));
        assert_eq!(parse_intent(&ready), Intent::RequestTakeoff);
        assert!(!wants_circuits(&ready), "{ready}");
    }

    /// LEMG 13 (sortie 1790623420-full_auto, 2026-09-28, #172): told "hold
    /// position, traffic on the runway", the crew repeated "ready for
    /// departure" at +76 and +116 and was told "read back" each time. An
    /// owed readback goes out first, once per instruction, and again only
    /// if the tower still waits for it 30 s after it went OUT.
    #[test]
    fn a_held_crew_reads_back_the_hold_before_asking_again() {
        use super::OwedReadback;
        use crate::crew_radio::{readback_words, ReadbackRec};
        use crate::tower::{parse_intent, Intent};
        use std::time::{Duration, Instant};
        let hold = "Holding position, November eight one six November November.";
        let t0 = Instant::now();
        let rec = |words: &str, out: Option<u64>| ReadbackRec {
            words: readback_words(words),
            queued_at: t0,
            out_at: out.map(|s| t0 + Duration::from_secs(s)),
        };
        assert_eq!(OwedReadback::due(None, None, t0), None, "nothing owed, nothing said");
        assert_eq!(OwedReadback::due(Some(hold), None, t0).as_deref(), Some(hold));
        let queued = rec(hold, None);
        let at = |s| t0 + Duration::from_secs(s);
        assert_eq!(OwedReadback::due(Some(hold), Some(&queued), at(5)), None, "queued once");
        let out = rec(hold, Some(2));
        assert_eq!(OwedReadback::due(Some(hold), Some(&out), at(5)), None, "said once");
        assert_eq!(
            OwedReadback::due(Some(hold), Some(&out), at(33)).as_deref(),
            Some(hold),
            "still owed 30 s after it went out: said again"
        );
        let other = "Lining up runway one three, November eight one six November November.";
        assert_eq!(
            OwedReadback::due(Some(other), Some(&out), at(34)).as_deref(),
            Some(other),
            "a new instruction is read back at once"
        );
        // What goes out is a readback, not another request.
        let sent = hold.trim_end_matches('.');
        assert_ne!(parse_intent(sent), Intent::RequestTakeoff, "{sent}");
    }

    #[test]
    fn a_circuit_from_the_ramp_asks_its_clearance_for_circuits() {
        use super::zone_clearance_call;
        use crate::tower::{parse_intent, wants_circuits, Intent};
        // LEMG (sorties 1790605551 and 1790618416, 2026-09-28): the ramp's
        // "request clearance" on a circuit earned "cleared to leave the
        // Malaga zone".
        let cs = Some("November two five one Charlie Victor".to_string());
        let call = crate::crew_radio::radio_call(&cs, zone_clearance_call(true));
        assert_eq!(parse_intent(&call), Intent::RequestClearance, "{call}");
        assert!(wants_circuits(&call), "{call}");
        // A departure never says it.
        let call = crate::crew_radio::radio_call(&cs, zone_clearance_call(false));
        assert_eq!(parse_intent(&call), Intent::RequestClearance, "{call}");
        assert!(!wants_circuits(&call), "{call}");
    }

    fn egkk_egbb_route() -> super::Route {
        // The plan's leg altitudes as the arm's audit logged them (t
        // 1790601822.7): the SID climb, FL180 cruise, HEME1B's HEMEL window
        // at 22,000, the descent.
        let alts = [
            ("D257B", 1885.0), ("DET31", 4370.0), ("DET29", 5418.0), ("ACORN", 12411.0),
            ("D258K", 14645.0), ("D156O", 18000.0), ("D156J", 18000.0), ("LAM", 18000.0),
            ("HEMEL", 22000.0), ("BUZAD", 18872.0), ("WELIN", 16500.0), ("PUFAX", 11900.0),
            ("HON", 9000.0), ("OSKOT", 5700.0), ("GROVE", 7000.0),
        ];
        super::Route {
            local_runway: None,
            dest_icao: "EGBB".into(),
            dest_name: "Birmingham".into(),
            legs: alts
                .iter()
                .map(|(id, a)| crate::nav::ProcLeg {
                    fix_ident: (*id).into(),
                    alt1_ft: Some(*a),
                    ..Default::default()
                })
                .collect(),
            climb_minimums: Vec::new(),
            arrival_constraints: Vec::new(),
            departure_rwy: Some("26L".into()),
            origin_icao: "EGKK".into(),
            origin_rwy: Some("26L".into()),
            fms_payload: String::new(),
            arrival_rwy: Some("33".into()),
            arrival_star: Some("HEME1B".into()),
            sid: Some("LAM5P".into()),
            ifr: true,
            cruise_schedule: Default::default(),
            cruise_ft: Some(18_000.0),
        }
    }

    /// Sortie 1790601822-full_auto (EGKK-EGBB, 2026-09-28, #155): the IFR
    /// clearance named "runway zero eight right" at +142.8 and the taxi
    /// request at +159.0 still asked for "runway two six left", the plan's.
    /// The cleared runway is the one asked for; without a clearance the
    /// plan's stands.
    #[test]
    fn the_taxi_request_names_the_cleared_runway() {
        use super::{taxi_call, Mission};
        use crate::tower::{parse_intent, Intent};
        let flight = Mission::Flight(egkk_egbb_route());
        let cs = Some("Whiskey Alpha Tango six two six".to_string());
        let call = crate::crew_radio::radio_call(&cs, &taxi_call(&flight, Some("08R")));
        assert!(call.ends_with("request taxi, runway zero eight right"), "{call}");
        assert_eq!(parse_intent(&call), Intent::RequestTaxi, "{call}");
        assert_eq!(
            taxi_call(&flight, None),
            "request taxi, runway two six left",
            "no clearance: the plan's runway"
        );
        assert_eq!(taxi_call(&flight, Some("")), "request taxi, runway two six left");
    }

    /// Same sortie: the check-in at +1209.4 said "climbing flight level two
    /// two zero" (the legs' highest, HEMEL's STAR window) against a plan
    /// filed at FL180, and departure cleared it. The check-in climb is capped
    /// at the filed cruise; a plan with no cruise keeps the legs' highest.
    #[test]
    fn the_departure_check_in_climbs_no_higher_than_the_filed_cruise() {
        use super::check_in_climb_ft;
        let route = egkk_egbb_route();
        assert_eq!(check_in_climb_ft(&route.legs, route.cruise_ft), 18_000.0);
        assert_eq!(check_in_climb_ft(&route.legs, None), 22_000.0);
        assert_eq!(check_in_climb_ft(&route.legs, Some(0.0)), 22_000.0);
        // A cruise above every leg: the legs' highest (the old number).
        assert_eq!(check_in_climb_ft(&route.legs[..8], Some(24_000.0)), 18_000.0);
        let call = super::departure_check_in_above(
            &Some("Whiskey Alpha Tango six two six".into()),
            "London Gatwick Departure",
            4700.0,
            check_in_climb_ft(&route.legs, route.cruise_ft),
            6000.0,
        );
        assert!(call.ends_with("climbing flight level one eight zero"), "{call}");
    }

    /// The crew flies the departure it was cleared on: with the station's
    /// GOSAM 1A in hand the departure is GOSAM 1A and the event says
    /// "cleared", ahead of the crew's own best pick (GOSAM 1B, which joins
    /// further along). With no clearance the old pick stands, "chosen".
    #[test]
    fn a_cleared_sid_is_flown_and_the_old_pick_stands_without_one() {
        use super::{cleared_sid_for, departure_source, prefer_cleared_sid};
        // The crew's own order, best first (`nav::sids_for_route`).
        let own = || vec![sid("GOSAM1B", 3), sid("GOSAM1A", 2)];
        let c = gosam_1a();
        let wanted = cleared_sid_for(Some(&c), Some("24"));
        assert_eq!(wanted.as_deref(), Some("GOSAM1A"));
        let order = prefer_cleared_sid(own(), wanted.as_deref(), false);
        assert_eq!(order[0].ident, "GOSAM1A", "the tower's SID first");
        assert_eq!(order.len(), 2, "the crew's pick stays as the fallback");
        assert_eq!(departure_source(Some(&order[0].ident), wanted.as_deref()), "cleared");
        // No clearance: the crew's order untouched, its pick "chosen".
        let wanted = cleared_sid_for(None, Some("24"));
        assert_eq!(wanted, None);
        let order = prefer_cleared_sid(own(), wanted.as_deref(), false);
        assert_eq!(order[0].ident, "GOSAM1B");
        assert_eq!(departure_source(Some(&order[0].ident), wanted.as_deref()), "chosen");
        // A clearance whose SID does not serve the runway now cleared (the
        // tower swapped ends since): not the tower's SID any more.
        assert_eq!(cleared_sid_for(Some(&c), Some("06")), None);
        // No runway clearance yet: the clearance's own runway.
        assert_eq!(cleared_sid_for(Some(&c), None).as_deref(), Some("GOSAM1A"));
        // Runway heading (no SID in the clearance): nothing to fly but the plan.
        let heading = crate::party_line::IfrClearance { sid: None, ..gosam_1a() };
        assert_eq!(cleared_sid_for(Some(&heading), Some("24")), None);
        // The planned runway stands and only the SID changed: the tower's
        // alone, or nothing (the plan's departure is kept, never a third).
        let only = prefer_cleared_sid(own(), Some("GOSAM1A"), true);
        assert_eq!(only.iter().map(|c| c.ident.as_str()).collect::<Vec<_>>(), ["GOSAM1A"]);
        assert!(prefer_cleared_sid(own(), Some("GOSAM1C"), true).is_empty());
        assert_eq!(prefer_cleared_sid(own(), Some("GOSAM1C"), false)[0].ident, "GOSAM1B");
    }

    /// The crew's IFR call carries the destination's ICAO spelled out, and
    /// the tower's parser reads it back as that code.
    #[test]
    fn the_ifr_call_carries_the_destination_the_tower_reads() {
        let cs = Some("November one seven two Sierra Papa".to_string());
        let call = super::ifr_request_call(&cs, "EGPH");
        assert_eq!(
            call,
            "November one seven two Sierra Papa, request IFR clearance to Echo Golf Papa Hotel"
        );
        assert_eq!(
            crate::tower::parse_intent(&call.to_ascii_lowercase()),
            crate::tower::Intent::RequestIfrClearance {
                dest: Some("EGPH".into())
            }
        );
        // The IFR coach line, not the zone one nor a stale landing one.
        let ifr = "Cleared to Edinburgh via the Gosa one Charlie departure, runway zero niner, climb five thousand feet, squawk four five two one, November.";
        assert_eq!(super::ifr_readback_line(Some(ifr)), Some(ifr));
        assert_eq!(super::ifr_readback_line(Some("Not above two thousand feet, squawk four five two one, N.")), None);
        assert_eq!(super::ifr_readback_line(Some("Cleared to land runway zero niner, N.")), None);
        assert_eq!(super::ifr_readback_line(None), None);
    }

    /// Issue #116, sortie 1790527340-full_auto (EGKK, 2026-09-27; seconds
    /// after 1790527340): the readback was queued at +17.4, the tower's
    /// clearance spoken at +33.2, the readback went out and was accepted at
    /// +49.6 (atc `ifr_clearance` side tower), but a 30 s clock from the
    /// queue gave up at +47.5 (`ifr_clearance_not_accepted`). The clock now
    /// starts when the readback leaves the crew queue.
    #[test]
    fn the_ifr_readback_wait_runs_from_the_readback_going_out() {
        use super::{ReadbackWait, READBACK_ACCEPT_S, READBACK_QUEUE_MAX_S};
        let queued_at = 17.4;
        let out_at = 49.6;
        let accepted_at = 49.6;
        // The recorded polls, 0.5 s apart, on the wait's own clock.
        let run = |mut w: ReadbackWait, out_at: f64, accepted_at: f64| {
            let mut t = queued_at;
            loop {
                let now = t - queued_at;
                if let Some(v) = w.tick(now, t < out_at, t >= accepted_at) {
                    return (v, t);
                }
                t += 0.5;
                assert!(t < 1000.0, "never decided");
            }
        };
        let (ok, at) = run(ReadbackWait::new(0.0), out_at, accepted_at);
        assert!(ok, "gave up at +{at}");
        assert!((at - accepted_at).abs() < 0.51, "accepted at +{at}");
        // The old clock, 30 s from the queue, decided before the readback
        // was even heard.
        assert!(queued_at + READBACK_ACCEPT_S < out_at);
        // Out but never accepted: given up 30 s after it went out.
        let (ok, at) = run(ReadbackWait::new(0.0), out_at, f64::INFINITY);
        assert!(!ok);
        assert!((at - (out_at + READBACK_ACCEPT_S)).abs() < 0.51, "{at}");
        // Never left the queue (the radio dead): the queue's own bound.
        let (ok, at) = run(ReadbackWait::new(0.0), f64::INFINITY, f64::INFINITY);
        assert!(!ok);
        assert!((at - (queued_at + READBACK_QUEUE_MAX_S)).abs() < 0.51, "{at}");
    }

    #[test]
    fn a_plan_flies_ifr_from_simbrief_or_above_the_transition() {
        use crate::simbrief::PlanSource;
        assert!(super::plan_is_ifr(PlanSource::SimBrief, 4500.0, 6000.0));
        assert!(super::plan_is_ifr(PlanSource::Local, 9500.0, 6000.0));
        assert!(!super::plan_is_ifr(PlanSource::Local, 4500.0, 6000.0));
        // A hold at the clearance's altitude lets go after a level spell.
        assert!(!super::cleared_hold_expired(30.0));
        assert!(super::cleared_hold_expired(90.0));
    }

    /// A 737 on the taxiway after a give-up: idle thrust creeps it faster
    /// (LEMG 2026-09-28: 4.9 kt at the give-up, 5.8 kt 13 s later, 0.035
    /// m/s/s), and each held regular-brake pulse slows it at 1.5 m/s/s for
    /// the 0.4 s tick. Records the groundspeed when the parking brake is SET.
    struct TaxiingJet {
        sim: crate::xplane::FakeXPlane,
        gs_ms: std::sync::Mutex<f64>,
        braking: std::sync::atomic::AtomicBool,
        gs_at_park: std::sync::Mutex<Option<f64>>,
    }

    impl TaxiingJet {
        const SET: &'static str = "sim/flight_controls/park_brake_set";
        const PARK: &'static str = "sim/cockpit2/controls/parking_brake_ratio";

        fn rolling_at_kts(kts: f64) -> Self {
            let sim = crate::xplane::FakeXPlane::with_reads(&[
                (crate::datarefs::ON_GROUND_ANY, 1.0),
                (crate::autotaxi::TUG_ATTACHED, 0.0),
                (Self::PARK, 0.0),
            ]);
            sim.step_on_command(Self::SET, Self::PARK, 1.0);
            TaxiingJet {
                sim,
                gs_ms: std::sync::Mutex::new(kts / super::MS_TO_KTS),
                braking: std::sync::atomic::AtomicBool::new(false),
                gs_at_park: std::sync::Mutex::new(None),
            }
        }
    }

    #[async_trait::async_trait]
    impl crate::xplane::XPlane for TaxiingJet {
        async fn get_float(&self, name: &str) -> anyhow::Result<f64> {
            if name != crate::autotaxi::GS_MS {
                return self.sim.get_float(name).await;
            }
            let mut gs = self.gs_ms.lock().unwrap();
            if self.braking.swap(false, std::sync::atomic::Ordering::SeqCst) {
                *gs = (*gs - 1.5 * 0.4).max(0.0);
            } else if *gs > 0.0 {
                *gs += 0.035 * 0.4;
            }
            Ok(*gs)
        }
        async fn set_float(&self, name: &str, value: f64) -> anyhow::Result<()> {
            self.sim.set_float(name, value).await
        }
        async fn set_int(&self, name: &str, value: i64) -> anyhow::Result<()> {
            self.sim.set_int(name, value).await
        }
        async fn run_command(&self, name: &str) -> anyhow::Result<()> {
            if name == Self::SET {
                *self.gs_at_park.lock().unwrap() = Some(*self.gs_ms.lock().unwrap());
            }
            self.sim.run_command(name).await
        }
        async fn run_command_for(&self, name: &str, secs: f64) -> anyhow::Result<()> {
            if name == crate::tools::BRAKES_REGULAR || name == crate::tools::BRAKES_MAX {
                self.braking.store(true, std::sync::atomic::Ordering::SeqCst);
            }
            self.sim.run_command_for(name, secs).await
        }
        async fn get_float_array(&self, name: &str) -> anyhow::Result<Vec<f64>> {
            self.sim.get_float_array(name).await
        }
        async fn set_int_array_elem(&self, name: &str, i: usize, v: i64) -> anyhow::Result<()> {
            self.sim.set_int_array_elem(name, i, v).await
        }
        async fn set_float_array_elem(&self, name: &str, i: usize, v: f64) -> anyhow::Result<()> {
            self.sim.set_float_array_elem(name, i, v).await
        }
    }

    /// LEMG 2026-09-28 (sortie 1790586620, #135): the pilot's "my controls"
    /// stopped the taxi governor, and the give-up 0.6 s later found nothing
    /// of ours holding the 737: the old gate (`taxi_hold_active`) skipped the
    /// hand-back, "The taxi didn't finish. Your aircraft." was said over an
    /// aircraft at 4.9 kt, 5.8 kt and rising 13 s on, `parking_brake_ratio`
    /// 0.0. A ground mission's end STOPS it on the wheel brakes, then sets
    /// the parking brake, read back.
    #[tokio::test(start_paused = true)]
    async fn a_give_up_rolling_on_the_taxiway_stops_and_parks_the_aircraft() {
        let jet = TaxiingJet::rolling_at_kts(4.9);
        let held = super::hand_back_hold(&jet, Some(1.0), super::HAND_BACK).await;
        assert_eq!(held, Some(true), "the parking brake reads set");
        assert!(
            jet.sim.ran(crate::tools::BRAKES_REGULAR),
            "stopped on the wheel brakes first"
        );
        let at_park = jet.gs_at_park.lock().unwrap().expect("park brake set");
        assert!(
            at_park * super::MS_TO_KTS < super::HAND_BACK_STOPPED_KTS,
            "the parking brake went on at {:.1} kt, not stopped",
            at_park * super::MS_TO_KTS
        );
        assert_eq!(jet.sim.last_write(TaxiingJet::PARK), None, "never a raw ratio write");
        assert!(
            jet.sim.ran("sim/flight_controls/park_brake_valve_open"),
            "valve-aware, the Baron's latching valve"
        );
    }

    /// LEMG 13 (2026-09-28, sortie 1790627884-full_auto): the pilot's reject
    /// handed back at 70 kt, the hand-back skipped anything over 25 kt ("the
    /// loop that owns the speed does the stopping"), no loop owned it, and
    /// the 737 rolled 70 to 52 kt for two minutes with nobody braking. A
    /// fast hand-back on the ground brakes to a stop, at maximum effort
    /// while fast, then parks.
    #[tokio::test(start_paused = true)]
    async fn a_fast_hand_back_on_the_ground_is_braked_to_a_stop() {
        let jet = TaxiingJet::rolling_at_kts(70.0);
        let held = super::hand_back_hold(&jet, Some(1.0), super::HAND_BACK).await;
        assert_eq!(held, Some(true), "the parking brake reads set");
        assert!(jet.sim.ran(crate::tools::BRAKES_MAX), "maximum effort while fast");
        assert!(jet.sim.ran(crate::tools::BRAKES_REGULAR), "the regular effort once slow");
        let at_park = jet.gs_at_park.lock().unwrap().expect("park brake set");
        assert!(
            at_park * super::MS_TO_KTS < super::HAND_BACK_STOPPED_KTS,
            "the parking brake went on at {:.1} kt",
            at_park * super::MS_TO_KTS
        );
        assert!(!super::pilot_on_the_brakes(Some(0.1), None), "a resting foot");
        assert!(super::pilot_on_the_brakes(None, Some(0.9)), "one pedal is a stop");
    }

    /// LEMG 13 SR22 circuit, 2026-09-29 (sortie 1790692781-full_auto):
    /// autoland let go at 805 ft climbing and the mission ended airborne.
    /// Only an autopilot READ holding on heading and a vertical mode, near
    /// level by the end of the watch, is handed over; everything else is
    /// the recovery's.
    #[test]
    fn an_airborne_end_hands_over_only_a_held_autopilot() {
        use super::{airborne_end_verdict, AirborneLook};
        let held = AirborneLook {
            servos: Some(1.0),
            hdg_status: Some(2.0),
            vs_status: Some(2.0),
            alt_hold_status: Some(0.0),
            vvi_fpm: Some(-150.0),
            pitch_deg: Some(2.0),
            bank_deg: Some(1.0),
            ias_kts: Some(110.0),
        };
        assert_eq!(airborne_end_verdict(&held, 75.0, true), Ok(()));
        let alt = AirborneLook { vs_status: Some(0.0), alt_hold_status: Some(2.0), ..held };
        assert_eq!(airborne_end_verdict(&alt, 75.0, true), Ok(()));
        let off = AirborneLook { servos: Some(0.0), ..held };
        assert_eq!(airborne_end_verdict(&off, 75.0, false), Err("servos_off"));
        let unread = AirborneLook { servos: None, ..held };
        assert_eq!(airborne_end_verdict(&unread, 75.0, false), Err("servos_off"));
        let no_hdg = AirborneLook { hdg_status: Some(0.0), ..held };
        assert_eq!(airborne_end_verdict(&no_hdg, 75.0, false), Err("no_heading_mode"));
        let no_vert = AirborneLook { vs_status: Some(0.0), ..held };
        assert_eq!(airborne_end_verdict(&no_vert, 75.0, false), Err("no_vertical_mode"));
        let slow = AirborneLook { ias_kts: Some(60.0), ..held };
        assert_eq!(airborne_end_verdict(&slow, 75.0, false), Err("not_holding"));
        // The sortie's own hand-back: climbing 850 fpm on a 1,000 dial. Still
        // settling mid-watch; at the end it must have come to level.
        let climbing = AirborneLook { vvi_fpm: Some(1_400.0), ..held };
        assert_eq!(airborne_end_verdict(&climbing, 75.0, false), Ok(()));
        assert_eq!(airborne_end_verdict(&climbing, 75.0, true), Err("not_level"));
        // A failed attitude read is no evidence against.
        let blind = AirborneLook { vvi_fpm: None, pitch_deg: None, ..held };
        assert_eq!(airborne_end_verdict(&blind, 75.0, true), Ok(()));
    }

    #[test]
    fn the_airborne_end_line_says_what_is_held() {
        let l = super::airborne_end_line(Some(131.6), Some(2_048.0));
        assert_eq!(
            l,
            "I've stopped flying. The autopilot is holding heading 132 at 2000 feet, level. \
             Your aircraft."
        );
        assert!(super::airborne_end_line(Some(0.2), None).contains("heading 360,"));
        assert!(!l.contains('\u{2014}') && !l.contains('\u{2013}'));
    }

    /// The landing that let go airborne is judged, never the mission's end
    /// (source-pinned: the wait is not a plain test).
    #[test]
    fn an_airborne_autoland_end_is_held_not_ended() {
        let src = crate::test_support::module_source("full_auto");
        let body = src
            .split("async fn taxi_in_after_landing(")
            .nth(1)
            .unwrap()
            .split("\"no_taxi_in\",")
            .nth(1)
            .unwrap()
            .split("return;")
            .next()
            .unwrap();
        assert!(body.contains("if landed && !on_ground && !crashed && !ABORT.load(SeqCst)"));
        assert!(body.contains("hold_airborne_end(xp, &config.xplane_base_url, voice).await;"));
    }

    /// The circuit mission ends as the takeoff's give-up when its takeoff
    /// never completed (the runner's `expect_give_up`), never as "ended";
    /// the pilot's cancel stays the cancel.
    #[test]
    fn a_circuit_whose_takeoff_was_rejected_gives_up() {
        use super::circuit_takeoff_failed;
        let rejected = Some(crate::circuit::TAKEOFF_NOT_COMPLETED);
        assert!(circuit_takeoff_failed(rejected, false));
        assert!(!circuit_takeoff_failed(rejected, true), "the pilot's cancel");
        assert!(!circuit_takeoff_failed(None, false), "flew and landed");
        assert!(!circuit_takeoff_failed(Some("abort_takeoff_phase"), false));
        // Refused on the runway for the ground under it (#182): a give-up too.
        assert!(circuit_takeoff_failed(
            Some(crate::circuit::TERRAIN_REFUSED),
            false
        ));
    }

    /// LEMG 13 (2026-09-28, sortie 1790627884-full_auto): the roll began at
    /// +52.8, the tower's "cleared for takeoff" was spoken at +53.5 and the
    /// readback went out at +71.2 in the roll. The crew waits for its
    /// readback to go out and the frequency to go quiet, bounded.
    #[tokio::test(start_paused = true)]
    async fn the_roll_waits_for_the_takeoff_readback() {
        use super::readback_before_moving;
        let t0 = tokio::time::Instant::now();
        let since = move || t0.elapsed().as_secs_f64();
        // Out at 18 s (the tower's clearance spoken first), on the air 3 s.
        let went = readback_before_moving(45.0, 250, || since() >= 18.0, || since() < 21.0).await;
        assert!(went);
        let waited = since();
        assert!((21.0..22.0).contains(&waited), "moved at {waited:.1} s");
        // A frequency that never takes it: the crew moves at the bound.
        let t1 = tokio::time::Instant::now();
        let went = readback_before_moving(45.0, 250, || false, || false).await;
        assert!(!went);
        assert!(t1.elapsed().as_secs_f64() >= 45.0);
    }

    /// LFPB 2026-09-22: the line-up refused at the holding point, full auto
    /// said "your aircraft", and the taxi governor went on holding the 737 on
    /// its wheel brakes for twenty-seven minutes with `park` at 0.0. Stopped,
    /// the parking brake goes straight on; the gates still hold elsewhere.
    #[tokio::test(start_paused = true)]
    async fn a_ground_give_up_hands_the_hold_to_the_parking_brake() {
        use super::hand_back_hold;
        use crate::xplane::FakeXPlane;
        const SET: &str = TaxiingJet::SET;
        const PARK: &str = TaxiingJet::PARK;

        // Stopped at the hold, governor or not: the brake goes on, no pulse.
        let xp = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (crate::autotaxi::GS_MS, 0.0),
            (PARK, 0.0),
        ]);
        xp.step_on_command(SET, PARK, 1.0);
        assert_eq!(hand_back_hold(&xp, Some(1.0), super::HAND_BACK).await, Some(true));
        assert!(xp.ran(SET), "the parking brake is set");
        assert!(!xp.ran(crate::tools::BRAKES_REGULAR), "already stopped");

        // Airborne: nothing. A give-up in the air hands back a flying
        // aircraft, and a parking brake is not part of that.
        let air = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 0.0),
            (crate::autotaxi::GS_MS, 90.0),
            (PARK, 0.0),
        ]);
        assert_eq!(hand_back_hold(&air, Some(1.0), super::HAND_BACK).await, None);
        assert!(!air.ran(SET), "no park brake in the air");
        assert!(!air.ran(crate::tools::BRAKES_REGULAR), "no brakes in the air");

        // Rolling fast with the pilot standing on the toe brakes: theirs.
        let braking = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (crate::autotaxi::GS_MS, 40.0),
            (super::TOE_BRAKE_L, 0.8),
            (super::TOE_BRAKE_R, 0.8),
            (PARK, 0.0),
        ]);
        assert_eq!(hand_back_hold(&braking, Some(1.0), super::HAND_BACK).await, None);
        assert!(!braking.ran(SET), "the pilot is stopping it");
        assert!(!braking.ran(crate::tools::BRAKES_MAX), "no fight on the pedals");

        // A tug on the bar owns the aircraft.
        let tug = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (crate::autotaxi::GS_MS, 0.5),
            (crate::autotaxi::TUG_ATTACHED, 1.0),
            (PARK, 0.0),
        ]);
        assert_eq!(hand_back_hold(&tug, Some(1.0), super::HAND_BACK).await, None);
        assert!(!tug.ran(SET) && !tug.ran(crate::tools::BRAKES_REGULAR));

        // A failed groundspeed read is NO measurement, never an assumed zero.
        let blind = FakeXPlane::with_reads(&[(crate::datarefs::ON_GROUND_ANY, 1.0), (PARK, 0.0)]);
        assert_eq!(hand_back_hold(&blind, Some(1.0), super::HAND_BACK).await, None);
        assert!(!blind.ran(SET), "a speed we could not read is not a stop");

        // A brake that never reads set is said, never claimed.
        let stuck = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (crate::autotaxi::GS_MS, 0.0),
            (PARK, 0.0),
        ]);
        assert_eq!(hand_back_hold(&stuck, Some(1.0), super::HAND_BACK).await, Some(false));
    }

    /// The callers that know more (refactor audit 2026-10-04, duplicates
    /// #8): the push's stop brakes with the tug on the bar it has just told
    /// to stop, and the taxi-in, the rollout and the push fail closed on an
    /// unread speed or ground flag, braking at maximum effort and parking,
    /// where a hand-back lets go. Airborne is never a park, for anyone.
    #[tokio::test(start_paused = true)]
    async fn a_caller_that_knows_it_is_on_the_ground_fails_closed() {
        use super::{hand_back_hold, HoldFor};
        use crate::xplane::FakeXPlane;
        const SET: &str = TaxiingJet::SET;
        const PARK: &str = TaxiingJet::PARK;
        let push = HoldFor::by("pushback").on_the_bar().fail_closed();
        let taxi_in = HoldFor::by("taxi_in").fail_closed();

        let tug = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 1.0),
            (crate::datarefs::GROUNDSPEED_MS, 0.0),
            (crate::autotaxi::TUG_ATTACHED, 1.0),
            (PARK, 0.0),
        ]);
        tug.step_on_command(SET, PARK, 1.0);
        assert_eq!(hand_back_hold(&tug, None, push).await, Some(true), "the push's stop");

        let blind = FakeXPlane::with_reads(&[(crate::datarefs::ON_GROUND_ANY, 1.0), (PARK, 0.0)]);
        blind.step_on_command(SET, PARK, 1.0);
        assert_eq!(hand_back_hold(&blind, None, taxi_in).await, Some(true));
        assert!(blind.ran(crate::tools::BRAKES_MAX), "an unread speed is braked hard");
        assert!(blind.ran(SET), "and parked");

        let no_ground =
            FakeXPlane::with_reads(&[(crate::datarefs::GROUNDSPEED_MS, 0.0), (PARK, 0.0)]);
        no_ground.step_on_command(SET, PARK, 1.0);
        assert_eq!(hand_back_hold(&no_ground, None, taxi_in).await, Some(true));
        assert_eq!(hand_back_hold(&no_ground, None, super::HAND_BACK).await, None);

        let air = FakeXPlane::with_reads(&[
            (crate::datarefs::ON_GROUND_ANY, 0.0),
            (crate::datarefs::GROUNDSPEED_MS, 60.0),
            (PARK, 0.0),
        ]);
        assert_eq!(hand_back_hold(&air, None, push).await, None);
        assert!(!air.ran(SET), "never a parking brake in the air");
    }

    /// The SR22 at LEMG as the sim behaves: the held regular-brake command
    /// rides `parking_brake_ratio` at the regular effort (~0.36) while
    /// active and decays it over ~0.8 s after its 1 s expiry, writing over
    /// any parking brake set inside that window; a set after it sticks.
    /// Each pulse slows the aircraft 1.5 m/s/s for the 0.4 s tick.
    struct RidingBrake {
        sim: crate::xplane::FakeXPlane,
        gs_ms: std::sync::Mutex<f64>,
        braking: std::sync::atomic::AtomicBool,
        last_pulse: std::sync::Mutex<Option<tokio::time::Instant>>,
        set_at: std::sync::Mutex<Option<tokio::time::Instant>>,
    }

    impl RidingBrake {
        const PARK: &'static str = TaxiingJet::PARK;
        const EFFORT: f64 = 0.36;
        const DECAY_S: f64 = 0.8;

        fn rolling_at_kts(kts: f64) -> Self {
            RidingBrake {
                sim: crate::xplane::FakeXPlane::with_reads(&[
                    (crate::datarefs::ON_GROUND_ANY, 1.0),
                    (crate::autotaxi::TUG_ATTACHED, 0.0),
                ]),
                gs_ms: std::sync::Mutex::new(kts / super::MS_TO_KTS),
                braking: std::sync::atomic::AtomicBool::new(false),
                last_pulse: std::sync::Mutex::new(None),
                set_at: std::sync::Mutex::new(None),
            }
        }

        /// The ratio the sim shows now: the command's ride, else a set that
        /// came after the tail, else nothing (a set inside it was lost).
        fn park(&self) -> f64 {
            let now = tokio::time::Instant::now();
            let ride = self.last_pulse.lock().unwrap().map(|p| {
                let since = (now - p).as_secs_f64() - crate::tools::BRAKE_HOLD_SECS;
                (since, p + std::time::Duration::from_secs_f64(1.0 + Self::DECAY_S))
            });
            let set = *self.set_at.lock().unwrap();
            match ride {
                Some((since, _)) if since < 0.0 => Self::EFFORT,
                Some((since, _)) if since < Self::DECAY_S => {
                    Self::EFFORT * (1.0 - since / Self::DECAY_S)
                }
                Some((_, free)) => match set {
                    Some(s) if s >= free => 1.0,
                    _ => 0.0,
                },
                None => set.map_or(0.0, |_| 1.0),
            }
        }

        fn set_now(&self) {
            *self.set_at.lock().unwrap() = Some(tokio::time::Instant::now());
        }
    }

    #[async_trait::async_trait]
    impl crate::xplane::XPlane for RidingBrake {
        async fn get_float(&self, name: &str) -> anyhow::Result<f64> {
            if name == Self::PARK {
                return Ok(self.park());
            }
            if name != crate::autotaxi::GS_MS {
                return self.sim.get_float(name).await;
            }
            let mut gs = self.gs_ms.lock().unwrap();
            if self.braking.swap(false, std::sync::atomic::Ordering::SeqCst) {
                *gs = (*gs - 1.5 * 0.4).max(0.0);
            }
            Ok(*gs)
        }
        async fn set_float(&self, name: &str, value: f64) -> anyhow::Result<()> {
            if name == Self::PARK && value > 0.5 {
                self.set_now();
            }
            self.sim.set_float(name, value).await
        }
        async fn set_int(&self, name: &str, value: i64) -> anyhow::Result<()> {
            self.sim.set_int(name, value).await
        }
        async fn run_command(&self, name: &str) -> anyhow::Result<()> {
            if name == TaxiingJet::SET {
                self.set_now();
            }
            self.sim.run_command(name).await
        }
        async fn run_command_for(&self, name: &str, secs: f64) -> anyhow::Result<()> {
            if name == crate::tools::BRAKES_REGULAR {
                self.braking.store(true, std::sync::atomic::Ordering::SeqCst);
                *self.last_pulse.lock().unwrap() = Some(tokio::time::Instant::now());
            }
            self.sim.run_command_for(name, secs).await
        }
        async fn get_float_array(&self, name: &str) -> anyhow::Result<Vec<f64>> {
            self.sim.get_float_array(name).await
        }
        async fn set_int_array_elem(&self, name: &str, i: usize, v: i64) -> anyhow::Result<()> {
            self.sim.set_int_array_elem(name, i, v).await
        }
        async fn set_float_array_elem(&self, name: &str, i: usize, v: f64) -> anyhow::Result<()> {
            self.sim.set_float_array_elem(name, i, v).await
        }
    }

    /// #35, LEMG 2026-09-28 (sortie 1790605551, SR22, t 1790606569.6):
    /// `hand_back_brakes` from 4.2 kt, three pulses, stopped, then three
    /// parking-brake tries inside 1.6 s of the last pulse, all written over
    /// by the held command's ride on the ratio: `held: false`, `park` 0.294,
    /// "the parking brake didn't set". The 737 (t 1790601686.9) and the later
    /// SR22 (t 1790619440.1) held on try two, after the tail. Stopped, the
    /// hand-back waits the tail out and the first try sets.
    #[tokio::test(start_paused = true)]
    async fn the_hand_back_parks_after_the_brake_command_lets_go_of_the_ratio() {
        let sr22 = RidingBrake::rolling_at_kts(4.2);
        let held = super::hand_back_hold(&sr22, Some(1.0), super::HAND_BACK).await;
        let pulses = sr22.sim.ran_count(crate::tools::BRAKES_REGULAR);
        assert_eq!(pulses, 3, "the record's three pulses");
        assert_eq!(held, Some(true), "the parking brake reads set");
        assert_eq!(sr22.park(), 1.0);
        assert_eq!(sr22.sim.ran_count(TaxiingJet::SET), 1, "set on the first try");
    }

    /// #135: the pilot's cancel ends the taxi wait, and the give-up says so
    /// instead of blaming the taxi; the line then says what holds the aircraft.
    #[test]
    fn a_cancelled_wait_is_a_cancel_and_the_line_names_the_brake() {
        use super::{give_up_words, hand_back_line};
        let taxi = "The taxi didn't finish. Your aircraft.";
        assert_eq!(
            give_up_words(taxi, "taxi_stalled", true),
            ("Cancelled. Your aircraft.", "abort")
        );
        assert_eq!(give_up_words(taxi, "taxi_stalled", false), (taxi, "taxi_stalled"));
        assert_eq!(
            hand_back_line("Cancelled. Your aircraft.", Some(true)),
            "Cancelled. Your aircraft. Parking brake set."
        );
        assert!(hand_back_line(taxi, Some(false)).ends_with("hold the brakes."));
        assert_eq!(hand_back_line(taxi, None), taxi);
    }

    #[test]
    fn full_auto_requires_both_instruments_to_confirm_ground_contact() {
        use sim::ground::on_ground_confirmed as confirmed_ground;
        assert!(confirmed_ground(Some(0.0), Some(1.0)));
        assert!(!confirmed_ground(Some(500.0), Some(0.0)));
        assert!(!confirmed_ground(Some(0.0), Some(0.0))); // just lifted off
        assert!(!confirmed_ground(Some(500.0), Some(1.0))); // conflicting readings
        assert!(!confirmed_ground(None, Some(1.0)));
        assert!(!confirmed_ground(Some(0.0), None));
        assert!(!confirmed_ground(None, None));
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(!confirmed_ground(Some(invalid), Some(1.0)));
            assert!(!confirmed_ground(Some(0.0), Some(invalid)));
        }
    }

    /// #95: the arm loads only what the aircraft does not already carry, on
    /// the ground and stopped, and names an airframe it will not load.
    #[test]
    fn the_arm_loads_only_the_halves_that_are_not_aboard() {
        use super::{arm_load, ArmLoad};
        use crate::aircraft::LoadSheet;
        use crate::tools::loading::What;
        let fp = plan(true);
        let arm = |fuel: Option<f64>, payload: Option<f64>| {
            arm_load(&fp, true, fuel, payload, LoadSheet::Stock, "Cessna 172SP")
        };
        // The LXGB plan: 109 kg block, 268.5 kg payload.
        assert_eq!(arm(Some(109.0), Some(270.0)), ArmLoad::Matches);
        assert_eq!(arm(Some(105.0), Some(250.0)), ArmLoad::Matches, "inside 5 kg and one bag");
        assert_eq!(arm(Some(40.0), Some(270.0)), ArmLoad::Load(What::Fuel));
        assert_eq!(arm(Some(160.0), Some(270.0)), ArmLoad::Load(What::Fuel), "over the block too");
        assert_eq!(arm(Some(109.0), Some(80.0)), ArmLoad::Load(What::Payload));
        assert_eq!(arm(Some(40.0), Some(80.0)), ArmLoad::Load(What::Both));
        // A failed read is no measurement: the loader reads again itself.
        assert_eq!(arm(None, Some(270.0)), ArmLoad::Load(What::Fuel));
        // An absolute 100 kg would call a dry 172 "at the block".
        assert_eq!(arm(Some(20.0), Some(270.0)), ArmLoad::Load(What::Fuel));
        // A 737's 7,200 kg block: two percent is 144 kg of taxi and rounding.
        let mut big = plan(true);
        big.block_fuel_kg = 7_200.0;
        let fuel = |kg: f64| arm_load(&big, true, Some(kg), Some(268.5), LoadSheet::Stock, "737");
        assert_eq!(fuel(7_100.0), ArmLoad::Matches);
        assert_eq!(fuel(6_900.0), ArmLoad::Load(What::Fuel));
    }

    #[test]
    fn the_arm_never_loads_a_rolling_aircraft_or_forces_an_unprobed_one() {
        use super::{arm_load, ArmLoad};
        use crate::aircraft::LoadSheet;
        let fp = plan(true);
        assert_eq!(
            arm_load(&fp, false, Some(20.0), Some(0.0), LoadSheet::Stock, "Cessna 172SP"),
            ArmLoad::NotStopped
        );
        let ArmLoad::Declined(line) =
            arm_load(&fp, true, Some(20.0), Some(0.0), LoadSheet::Unprobed, "Airbus A330-300")
        else {
            panic!("an unprobed airframe is never loaded");
        };
        assert_eq!(
            line,
            "Loading the Airbus A330-300 from the plan is unprobed, so the fuel and payload are yours to set."
        );
        let ArmLoad::Declined(line) =
            arm_load(&fp, true, Some(20.0), None, LoadSheet::OwnFuelSystem, "Boeing 747-400")
        else {
            panic!("an add-on's own fuel is never loaded");
        };
        assert!(line.starts_with("The Boeing 747-400 manages its own fuel"), "{line}");
        // Already aboard is nothing to say, whoever owns the tanks.
        assert_eq!(
            arm_load(&fp, true, Some(109.0), Some(268.5), LoadSheet::Unprobed, "Airbus A330-300"),
            ArmLoad::Matches
        );
        // A local plan carries no sheet: the payload is never asked for.
        assert_eq!(
            arm_load(&plan(false), true, Some(109.0), Some(0.0), LoadSheet::Stock, "172"),
            ArmLoad::Matches
        );
        assert_eq!(super::arm_load_outcome("Loaded for LXGB to LEAX. Fuel 240 pounds."), "loaded");
        assert_eq!(
            super::arm_load_outcome("Loading for LXGB to LEAX. The fuel did not take: ..."),
            "not_taken"
        );
        assert_eq!(super::arm_load_outcome("Negative. The plan wants ..."), "refused");
        for line in [
            super::ArmFetch::Changed.line(&fp),
            super::ArmFetch::Fallback.line(&fp),
        ] {
            let line = line.expect("said");
            assert!(!line.contains('\u{2014}') && !line.contains('\u{2013}'), "{line}");
        }
    }

    /// #95: the arm fetches fresh and says only a change or a stale plan.
    #[test]
    fn the_arm_says_a_new_plan_and_a_stale_one_and_nothing_else() {
        use super::ArmFetch;
        let old = plan(true);
        let mut new = plan(true);
        new.destination = "LEMG".into();
        assert_eq!(ArmFetch::judge(false, true, Some(&old), &new), ArmFetch::Changed);
        assert_eq!(ArmFetch::judge(false, true, Some(&old), &old), ArmFetch::Fetched);
        assert_eq!(ArmFetch::judge(false, true, None, &new), ArmFetch::Fetched);
        assert_eq!(ArmFetch::judge(false, false, Some(&old), &old), ArmFetch::Fallback);
        assert_eq!(ArmFetch::judge(true, false, Some(&old), &old), ArmFetch::Local);
        assert_eq!(
            ArmFetch::Changed.line(&new).as_deref(),
            Some("New plan from SimBrief, LXGB to LEMG.")
        );
        assert_eq!(
            ArmFetch::Fallback.line(&old).as_deref(),
            Some("Couldn't reach SimBrief, flying the plan I already had, LXGB to LEAX.")
        );
        assert_eq!(ArmFetch::Fetched.line(&old), None);
        assert_eq!(ArmFetch::Local.line(&old), None);
    }

    /// A 172 plan LXGB-LEAX as SimBrief sent it (2026-09-15), in kilos.
    fn plan(weights: bool) -> crate::simbrief::FlightPlan {
        crate::simbrief::FlightPlan {
            transition_altitude_ft: None,
            transition_level_ft: None,
            procedures: Default::default(),
            callsign: None,
            schedule: Default::default(),
            source: crate::simbrief::PlanSource::SimBrief,
            origin: "LXGB".into(),
            destination: "LEAX".into(),
            alternate: String::new(),
            aircraft: "C172".into(),
            cruise_alt_ft: 3500.0,
            distance_nm: 45.0,
            ete_min: 35.0,
            trip_fuel_kg: 10.7 * 2.72,
            reserve_kg: 5.7 * 2.72,
            route: "DCT".into(),
            wind_comp_kts: 0.0,
            wind_dir: 0.0,
            wind_spd_kts: 0.0,
            tod_to_dest_nm: 8.0,
            waypoints: Vec::new(),
            block_fuel_kg: 109.0,
            min_takeoff_fuel_kg: 103.0,
            weights: weights.then(|| crate::simbrief::PlanWeights {
                pax: 3,
                bags: 1,
                cargo_kg: 30.4,
                payload_kg: 268.5,
                est_tow_kg: 1111.0,
                max_tow_kg: 1111.0,
                est_ldw_kg: 1082.0,
                max_ldw_kg: 1111.0,
                spoken_lbs: true,
            }),
        }
    }

    /// #152, the EGKK-EGBB plan of sortie 1790601822 (the pilot's OFP,
    /// trimmed): its filed route, as the tower is given it, picks LAM5P off
    /// 08R on the real EGKK SIDs, the same SID the crew's own order puts
    /// first; with the tower's SID the crew's is the cleared one, and the
    /// old clearance (KENE3P, off the route) is asked away on the radio in
    /// words the tower reads back to LAM5P.
    #[test]
    fn the_filed_route_clears_the_sid_the_crew_flies_at_egkk() {
        let v: serde_json::Value =
            serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/ofp/egkk-egbb-738.trimmed.json"))).unwrap();
        let mut p = plan(false);
        p.origin = "EGKK".into();
        p.destination = "EGBB".into();
        p.procedures.sid = Some("LAM6M".into());
        p.waypoints = std::iter::once("EGKK")
            .chain(v["navlog"]["fix"].as_array().unwrap().iter().map(|f| f["ident"].as_str().unwrap()))
            .map(|i| crate::simbrief::Waypoint { ident: i.into(), lat: 0.0, lon: 0.0, alt: 0.0 })
            .collect();
        let filed = crate::simbrief::filed_route_of(Some(&p)).unwrap();
        assert_eq!(filed.fixes.first().map(String::as_str), Some("D257B"));
        assert_eq!(filed.fixes.last().map(String::as_str), Some("GROVE"), "the fields left out");
        let cifp = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/crates/nav/bench/cifp/EGKK_sids.dat"));
        let tower = crate::nav::sid_for_filed_route_in(cifp.lines(), "08R", &filed.fixes, filed.sid.as_deref())
            .unwrap();
        assert_eq!(tower.ident, "LAM5P");
        let crew = super::prefer_cleared_sid(
            crate::nav::sids_for_route_in(cifp.lines(), "08R", &filed.fixes),
            Some(&tower.ident),
            false,
        );
        assert_eq!(crew[0].ident, "LAM5P");
        assert_eq!(super::departure_source(Some(&crew[0].ident), Some("LAM5P")), "cleared");
        // The sortie's clearance: KENE3P, never among the crew's.
        assert_eq!(
            super::cleared_sid_plan(Some(&crew[0].ident), Some("KENE3P")),
            super::ClearedSidPlan::Request("LAM5P".into())
        );
        let call = super::sid_request_call(
            &Some("Whiskey Alpha Tango six two six".into()),
            Some("KENE3P"),
            "LAM5P",
        );
        assert_eq!(crate::tower::parse_intent(&call), crate::tower::Intent::RequestSid, "{call}");
        let words = crate::tower::requested_departure(&call).unwrap();
        let idents: Vec<String> = crate::nav::sid_transitions_in(cifp.lines(), "08R")
            .into_iter()
            .map(|(i, _)| i)
            .collect();
        assert_eq!(crate::tower::match_sid(&words, &idents), Some("LAM5P"));
    }

    /// #173, sortie 1790624015-full_auto (EGKK 08R to EGBB, 2026-09-28): the
    /// cleared LAM5P spliced in carried its chart's "at 5,000" at ACORN and
    /// D156O as the plan's level (the recorded `climb_no_descent` plan_ft),
    /// LAM's 6,000 minimum floored it, and after departure cleared FL180
    /// the follower held 6,000 to LAM. Re-derived from the filed plan (the
    /// pilot's OFP), every SID leg is the plan's 18,000, and the recorded
    /// ticks past the check-in, replayed through the departure owners, want
    /// 18,000 where the sortie wanted 6,000.
    #[test]
    fn a_rechosen_sid_is_flown_at_the_plans_levels() {
        let ofp: serde_json::Value =
            serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/ofp/egkk-egbb-738.trimmed.json"))).unwrap();
        let num = |f: &serde_json::Value, k: &str| f[k].as_str().unwrap().parse::<f64>().unwrap();
        let wps: Vec<crate::simbrief::Waypoint> = ofp["navlog"]["fix"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|f| f["type"] != "ltlg")
            .map(|f| crate::simbrief::Waypoint {
                ident: f["ident"].as_str().unwrap().into(),
                lat: num(f, "pos_lat"),
                lon: num(f, "pos_long"),
                alt: num(f, "altitude_feet"),
            })
            .collect();
        let plan = crate::procedure::enroute_legs(&wps, "EGKK", "EGBB", 18_000.0);
        let cifp = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/crates/nav/bench/cifp/EGKK_sids.dat"));
        let mut chart = crate::nav::parse_proc_legs(
            cifp.lines(),
            crate::nav::ProcKind::Sid,
            "LAM5P",
            Some("RW08R"),
        );
        for (i, l) in chart.iter_mut().enumerate() {
            if !l.fix_ident.is_empty() {
                let lat = 51.2 + i as f64 * 0.05;
                l.fix = Some(crate::nav::Fix { ident: l.fix_ident.clone(), lat, lon: 0.1 });
            }
        }
        let raw = super::flown_sid_legs(chart, "LAM").expect("LAM5P reaches LAM");
        let ids: Vec<&str> = raw.iter().map(|l| l.fix_ident.as_str()).collect();
        assert_eq!(ids[1..], ["D258T", "ACORN", "D156O", "D156J", "LAM"]);
        let recs: Vec<serde_json::Value> =
            include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/follower/egkk_lam5p_held_6000.jsonl"))
                .lines()
                .filter_map(|l| serde_json::from_str(l).ok())
                .collect();
        // The old splice: the chart's altitudes, as the sortie logged them.
        let held = |r: &&serde_json::Value| {
            r["kind"] == "climb_no_descent" && r["idx"].as_u64() > Some(0)
        };
        for r in recs.iter().filter(held) {
            let i = r["idx"].as_u64().unwrap() as usize;
            assert_eq!(raw[i].alt1_ft, r["plan_ft"].as_f64(), "{r}");
        }
        let mut sid = raw.clone();
        super::rederive_sid_altitudes(&mut sid, &plan, "LAM");
        assert!(sid.iter().all(|l| l.alt1_ft == Some(18_000.0)), "{:?}",
            sid.iter().map(|l| (&l.fix_ident, l.alt1_ft)).collect::<Vec<_>>());

        let mins = crate::nav::sid_minimums_in(cifp.lines(), "LAM5P", "08R");
        assert!(mins.iter().any(|m| m.fix == "LAM" && m.min_ft == 6_000.0), "{mins:?}");
        let released = recs
            .iter()
            .find(|r| r["kind"] == "cleared_ceiling" && r["ft"].is_null())
            .and_then(|r| r["t"].as_f64())
            .unwrap();
        let ticks: Vec<&serde_json::Value> = recs
            .iter()
            .filter(|r| r["kind"] == "leg" && r["t"].as_f64().unwrap() > released + 5.0)
            .filter(|r| r["idx"].as_u64().unwrap() < sid.len() as u64)
            .collect();
        assert!(ticks.len() > 100, "{}", ticks.len());
        let (mut was_6000, mut carry) = (0, 18_000.0);
        for r in &ticks {
            let i = r["idx"].as_u64().unwrap() as usize;
            if i > 0 {
                assert_eq!(r["fix"].as_str(), Some(sid[i].fix_ident.as_str()));
            }
            if (r["want_alt"].as_f64().unwrap() - 6_000.0).abs() < 1.0 {
                was_6000 += 1;
            }
            let plan_ft = crate::procedure::commanded_alt_ft(&sid[i], carry);
            carry = plan_ft;
            let floor = crate::procedure::DepartureMinimums::floor(&mins, &sid, i).map(|m| m.1);
            let t = crate::procedure::departure_target_ft(plan_ft, floor, None, false, None);
            assert_eq!(t.want_ft, 18_000.0, "{r}");
            assert!(!t.raised_by_minimum);
        }
        let n = ticks.len();
        assert!(was_6000 * 10 >= n * 8, "the sortie held 6,000: {was_6000}/{n}");
    }

    /// A SID's "at or below" still bounds the re-derived climb, up to its
    /// fix; "at" and "at or above" never do (they are the minimums).
    #[test]
    fn a_sids_at_or_below_stays_a_ceiling() {
        let leg = |id: &str, desc: char, alt: Option<f64>| {
            let w = crate::simbrief::Waypoint { ident: id.into(), lat: 51.0, lon: 0.0, alt: 0.0 };
            crate::nav::ProcLeg {
                alt_desc: desc,
                alt1_ft: alt,
                ..crate::procedure::enroute_legs(&[w], "XXXX", "YYYY", 18_000.0)[0].clone()
            }
        };
        let plan = vec![leg("JOIN", ' ', Some(18_000.0)), leg("C", ' ', Some(12_000.0))];
        let mut sid = vec![
            leg("A", '+', Some(3_000.0)),
            leg("B", ' ', Some(5_000.0)),
            leg("C", '-', Some(7_000.0)),
            leg("D", 'B', Some(9_000.0)),
            leg("JOIN", '+', Some(6_000.0)),
        ];
        super::rederive_sid_altitudes(&mut sid, &plan, "JOIN");
        let alts: Vec<f64> = sid.iter().map(|l| l.alt1_ft.unwrap()).collect();
        assert_eq!(alts, vec![7_000.0, 7_000.0, 7_000.0, 9_000.0, 18_000.0]);
        assert_eq!(sid[0].alt_desc, '-');
        assert_eq!(sid[4].alt_desc, ' ');
        // No join in the plan: the chart as it was.
        let mut same = sid.clone();
        super::rederive_sid_altitudes(&mut same, &plan[1..], "JOIN");
        assert_eq!(same.iter().map(|l| l.alt1_ft).collect::<Vec<_>>(),
            sid.iter().map(|l| l.alt1_ft).collect::<Vec<_>>());
    }

    #[test]
    fn a_cleared_sid_is_flown_asked_away_or_handed_back() {
        use super::{cleared_sid_plan, ClearedSidPlan};
        assert_eq!(cleared_sid_plan(Some("LAM5P"), None), ClearedSidPlan::Fly);
        assert_eq!(cleared_sid_plan(None, None), ClearedSidPlan::Fly);
        assert_eq!(cleared_sid_plan(Some("lam5p"), Some("LAM5P")), ClearedSidPlan::Fly);
        assert_eq!(
            cleared_sid_plan(Some("LAM5P"), Some("KENE3P")),
            ClearedSidPlan::Request("LAM5P".into())
        );
        // Direct to the planned exit is no SID to ask for.
        assert_eq!(cleared_sid_plan(None, Some("KENE3P")), ClearedSidPlan::Cannot);
    }

    /// The 172's book and both fields, all fine: fuel at the block, loaded
    /// as planned, Gibraltar's 1,800 m and Axarquia's 1,000 m.
    fn facts() -> super::PreflightFacts {
        super::PreflightFacts {
            fuel_kg: Some(109.0),
            payload_kg: Some(270.0),
            total_kg: Some(1100.0),
            max_gross_lb: Some(2558.0),
            takeoff_roll_m: Some(1630.0 * 0.3048),
            landing_distance_ft: Some(1335.0),
            takeoff_distance_ft: Some(1630.0),
            origin_longest_ft: 5900.0,
            dest_longest_ft: 3280.0,
            service_ceiling_ft: None,
        }
    }

    /// LIME (2026-09-25), the real install: a route flying LEGL1P off 28 to
    /// LEGLO and on; the tower gives 10. The crew picks a runway-10 SID to
    /// LEGLO, its fixes resolved, and the route goes on from LEGLO.
    /// `cargo test -- --ignored real_install_lime_departure_rechosen --nocapture`
    #[tokio::test]
    #[ignore]
    async fn real_install_lime_departure_rechosen() {
        use crate::nav::ProcKind;
        let config = crate::config::Config::from_env().expect("config");
        let mut legs: Vec<crate::nav::ProcLeg> =
            crate::nav::load_proc_legs(&config.cifp_dir, "LIME", ProcKind::Sid, "LEGL1P", Some("RW28"))
                .into_iter()
                .filter(|l| !l.fix_ident.is_empty() && !l.fix_ident.starts_with("RW"))
                .collect();
        legs.extend(
            crate::nav::load_proc_legs(&config.cifp_dir, "LIME", ProcKind::Sid, "LEGL1P", Some("ARL1D"))
                .into_iter()
                .filter(|l| l.transition == "ARL1D" && l.fix_ident != "LEGLO"),
        );
        println!("before: {:?}", legs.iter().map(|l| l.fix_ident.clone()).collect::<Vec<_>>());
        let mut route = super::Route {
            local_runway: None,
            dest_icao: "LIPE".into(),
            dest_name: "Bologna".into(),
            legs,
            climb_minimums: Vec::new(),
            arrival_constraints: Vec::new(),
            departure_rwy: Some("28".into()),
            origin_icao: "LIME".into(),
            origin_rwy: Some("28".into()),
            fms_payload: "LIME,45.6689,9.7003,0|ME609,45.7,9.8,5000|LIPE,44.5354,11.2887,0".into(),
            arrival_rwy: None,
            arrival_star: None,
            sid: Some("LEGL1P".into()),
            ifr: true,
            cruise_schedule: Default::default(),
            cruise_ft: Some(12_000.0),
        };
        let fallback_route = route.clone();
        // The Evolution's own climb (profile best_climb_fpm); the GA default
        // of 700 fpm cannot make LEGLO's FL120 from either runway.
        let evo = crate::aircraft::ReferenceProfile { best_climb_fpm: Some(5000.0), ..Default::default() };
        crate::procedure::publish_vs_limits(Some(&evo));
        // The tower cleared LEGL1W off 10: that one is flown, "cleared".
        let mut cleared_route = route.clone();
        let d = super::choose_departure(&config, &mut cleared_route, "10", 105.0, Some("LEGL1W".into()), false)
            .await
            .expect("the cleared SID from 10");
        println!("{d:?}");
        assert_eq!(d.source, "cleared", "{d:?}");
        assert_eq!(cleared_route.sid.as_deref(), Some("LEGL1W"));
        let line = super::choose_departure(&config, &mut route, "10", 105.0, None, false)
            .await
            .map(|d| d.line);
        println!("{line:?}");
        let after: Vec<String> = route.legs.iter().map(|l| l.fix_ident.clone()).collect();
        println!("after: {after:?}");
        let line = line.expect("a departure from 10");
        assert!(line.contains("LEGLO"), "{line}");
        assert_eq!(route.departure_rwy.as_deref(), Some("10"));
        let join = after.iter().position(|f| f == "LEGLO").expect("joins LEGLO");
        assert!(!after[..join].iter().any(|f| f == "ME609" || f == "ME611"), "28's fixes are gone");
        assert!(route.legs[..=join].iter().all(|l| l.fix.is_some()), "every new fix resolved");
        assert!(after.contains(&"ARLES".to_string()), "the route goes on");
        assert!(route.fms_payload.starts_with("LIME,") && route.fms_payload.contains("|ME684,"), "{}", route.fms_payload);
        assert!(!route.fms_payload.contains("ME609"), "the GPS drops 28's SID");
        // A climb that makes no runway-10 SID: direct to the PLANNED SID's
        // exit (LEGL1P's LEGLO), never another SID's further join (SRN).
        // 400 fpm: the GA default of 700 makes LEGL1W's 318 feet a mile.
        let slow = crate::aircraft::ReferenceProfile { enroute_climb_fpm: Some(400.0), ..Default::default() };
        crate::procedure::publish_vs_limits(Some(&slow));
        let mut route = fallback_route;
        let line = super::choose_departure(&config, &mut route, "10", 105.0, None, false)
            .await
            .expect("fallback")
            .line;
        println!("{line}");
        assert!(line.contains("direct to LEGLO"), "{line}");
        assert_eq!(route.legs.first().map(|l| l.fix_ident.as_str()), Some("LEGLO"));
        crate::procedure::publish_vs_limits(None);
    }

    /// LILB 03 (2026-09-25): lined up with 508 m ahead, 595 needed with the
    /// margin, 500 without: the pilot's call. 398 ahead is short either way.
    #[test]
    fn a_line_up_short_only_by_the_margin_is_the_pilots_call() {
        assert_eq!(super::margin_only_short(508.0, 594.8, false), Some(500.0));
        assert_eq!(super::margin_only_short(398.0, 594.8, false), None);
        assert_eq!(super::margin_only_short(508.0, 594.8, true), None, "already waived");
        assert_eq!(super::margin_only_short(450.0, 500.0, false), None, "the floor is no margin");
    }

    /// LILB 2026-09-25: a runway the book roll fits but the factored one does
    /// not is the pilot's to waive; the book roll itself never is.
    #[test]
    fn the_runway_margin_is_waivable_and_the_book_roll_is_not() {
        use super::Gate;
        let short = super::PreflightFacts {
            origin_longest_ft: 1970.0,
            ..facts()
        };
        let (gate, _) = super::preflight_gated(&plan(true), &short, &[]).expect_err("refused");
        assert_eq!(gate, Gate::OriginRunway);
        assert!(gate.waivable());
        let brief = super::preflight_gated(&plan(true), &short, &[Gate::OriginRunway]).expect("waived");
        assert!(
            brief.starts_with("No runway margin, your call: 1,970 feet of runway, 1,640 feet needed without it."),
            "{brief}"
        );
        let shorter = super::PreflightFacts {
            origin_longest_ft: 1400.0,
            ..facts()
        };
        let (gate, why) =
            super::preflight_gated(&plan(true), &shorter, &[Gate::OriginRunway]).expect_err("refused");
        assert_eq!(gate, Gate::BookRoll);
        assert!(!gate.waivable());
        assert!(why.contains("Even with no margin"), "{why}");
    }

    /// The plan's operational limit is waivable; the airframe's max gross
    /// holds whatever was waived.
    #[test]
    fn the_plans_weight_limit_is_waivable_and_the_airframes_is_not() {
        use super::Gate;
        // 2,550 lb against the plan's 2,450 and the 172SP's 2,558.
        let heavy = super::PreflightFacts {
            total_kg: Some(1156.7),
            ..facts()
        };
        let (gate, _) = super::preflight_gated(&plan(true), &heavy, &[]).expect_err("refused");
        assert_eq!(gate, Gate::PlanWeight);
        let brief = super::preflight_gated(&plan(true), &heavy, &[Gate::PlanWeight]).expect("waived");
        assert!(brief.contains("Over the plan's weight limit, your call"), "{brief}");
        let over_gross = super::PreflightFacts {
            total_kg: Some(1180.0),
            ..facts()
        };
        let (gate, why) =
            super::preflight_gated(&plan(true), &over_gross, &[Gate::PlanWeight]).expect_err("refused");
        assert_eq!(gate, Gate::MaxWeight);
        assert!(why.contains("max takeoff weight of 2,560 pounds"), "{why}");
        // Over the plan AND the airframe, unwaived: the airframe's gate.
        let (gate, _) = super::preflight_gated(&plan(true), &over_gross, &[]).expect_err("refused");
        assert_eq!(gate, Gate::MaxWeight);
        assert!(!Gate::Fuel.waivable() && !Gate::Ceiling.waivable() && !Gate::DestRunway.waivable());
    }

    /// "Fly it anyway" waives the refusal just said, once, inside the window,
    /// for that plan only.
    #[test]
    fn a_waiver_answers_the_refusal_just_said_for_that_plan_only() {
        use super::Gate;
        let _g = WAIVER_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *super::lock(&super::WAIVER) = None;
        *super::lock(&super::REFUSED) = None;
        assert_eq!(super::take_waiver(), None, "nothing on offer");
        super::note_refusal(Gate::OriginRunway, &plan(true));
        assert_eq!(super::take_waiver(), Some(Gate::OriginRunway));
        assert_eq!(super::take_waiver(), None, "once");
        assert_eq!(super::waived_for(&plan(true)), vec![Gate::OriginRunway]);
        let mut other = plan(true);
        other.destination = "LEMG".into();
        assert!(super::waived_for(&other).is_empty(), "never another plan's");
        assert!(super::waived_for(&plan(true)).is_empty(), "and it is gone");
        // Outside the window: stale.
        *super::lock(&super::REFUSED) = Some((
            Gate::PlanWeight,
            "LXGB".into(),
            "LEAX".into(),
            std::time::Instant::now() - super::WAIVE_WINDOW - std::time::Duration::from_secs(1),
        ));
        assert_eq!(super::take_waiver(), None);
    }

    static WAIVER_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// LOWI 26 MOGT3H in the SF50 (the SimBrief plan of 2026-09-24): the
    /// binding minimum is WI506, 714 ft a mile over 8.3 nm from the departure
    /// end; at Vy 120 kt and 2,000 fpm the SF50 makes about 911 (78 %). Tight,
    /// briefed, flown at Vy.
    #[test]
    fn a_sid_gradient_is_judged_against_the_climb_at_vy() {
        let leg = |id: &str, lat: f64, lon: f64| crate::nav::ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "TF".into(),
            fix_ident: id.into(),
            fix_region: String::new(),
            fix: Some(crate::nav::Fix { ident: id.into(), lat, lon }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: None,
            alt2_ft: None,
            is_faf: false,
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
        };
        let legs = vec![
            leg("WI505", 47.252, 11.269),
            leg("WI506", 47.290, 11.139),
            leg("WI507", 47.306, 11.086),
            leg("WI802", 47.296, 10.840),
            leg("MOGTI", 47.389, 10.717),
        ];
        let m = |f: &str, a: f64| crate::nav::ClimbMinimum { fix: f.into(), min_ft: a };
        let mins = vec![m("WI505", 4000.0), m("WI506", 7850.0), m("WI507", 8400.0), m("WI802", 11350.0), m("MOGTI", 13000.0)];
        let der = (47.2587834, 11.3308736); // the 08 end: 26's departure end
        let checks = super::departure_gradients(der, 1907.0, &legs, &mins, 120.0, 2000.0);
        assert_eq!(checks.len(), 5);
        let wi506 = &checks[1];
        assert!((wi506.dist_nm - 8.3).abs() < 0.3, "{wi506:?}");
        assert!((wi506.need_ft_per_nm - 714.0).abs() < 20.0, "{wi506:?}");
        let line = super::gradient_verdict("MOGT3H", &checks).expect("flyable").expect("briefed");
        assert!(line.contains("MOGT3H needs") && line.contains("Tight"), "{line}");
        // At the cruise-speed climb flown today (1,500 fpm at 190 kt ~ 470 ft/nm
        // equivalent) the same departure is refused.
        let weak = super::departure_gradients(der, 1907.0, &legs, &mins, 190.0, 1500.0);
        assert!(super::gradient_verdict("MOGT3H", &weak).is_err());
    }

    /// LIPR 30 ANC8A (2026-09-26): climb to 450 on 304, the 291 course to 6
    /// DME, back 6 nm to RIM at 4,000 or above. RIM is a VOR on the field, so
    /// measured fix to fix it was 1.2 nm from the departure end, 3,388 feet a
    /// mile, and a 172 was refused a departure it climbs easily.
    #[test]
    fn a_sid_gradient_is_measured_along_the_procedure_not_fix_to_fix() {
        let rows = [
            "SID:010,2,ANC8A,RW30, , , , ,    , ,   ,CA, , , , , ,      ,    ,    ,3040,    ,+,00450,     ,06000, ,   ,    ,   ,RIM,LI,D, , , , , ;",
            "SID:020,2,ANC8A,RW30, , , , ,    , ,   ,CD, ,RIM,LI,D, ,      ,    ,    ,2910,0060,+,01500,     ,     ,-,210,    ,   , , , , , , , , ;",
            "SID:030,2,ANC8A,RW30,RIM,LI,D, ,V   ,R,   ,CF,Y,RIM,LI,D, ,      ,0000,0000,1610,0060,+,04000,     ,     ,-,210,    ,   , , , , , , , , ;",
        ];
        let mut legs = crate::nav::parse_proc_legs(rows.into_iter(), crate::nav::ProcKind::Sid, "ANC8A", Some("RW30"));
        assert_eq!(legs.len(), 3);
        for l in legs.iter_mut().filter(|l| l.fix_ident == "RIM") {
            l.fix = Some(crate::nav::Fix { ident: "RIM".into(), lat: 44.015167, lon: 12.616639 });
        }
        let mins = vec![crate::nav::ClimbMinimum { fix: "RIM".into(), min_ft: 4000.0 }];
        let der = (44.0285304, 12.5969644); // the 12 end: 30's departure end
        let checks = super::departure_gradients(der, 40.0, &legs, &mins, 74.0, 700.0);
        assert_eq!(checks.len(), 1);
        let rim = &checks[0];
        assert!(rim.dist_nm > 12.0, "{rim:?}");
        assert!(rim.need_ft_per_nm < 350.0, "{rim:?}");
        assert!(super::gradient_verdict("ANC8A", &checks).is_ok(), "a 172 flies it");
    }

    /// LIPR 30 ANC8A: the climb out, the out-and-back to 6 DME, RIM, then
    /// the plan. The chart's altitudes are minimums: the plan's stand.
    #[test]
    fn a_departure_is_flown_as_charted_up_to_where_it_joins_the_plan() {
        let rows = [
            "SID:010,2,ANC8A,RW30, , , , ,    , ,   ,CA, , , , , ,      ,    ,    ,3040,    ,+,00450,     ,06000, ,   ,    ,   ,RIM,LI,D, , , , , ;",
            "SID:020,2,ANC8A,RW30, , , , ,    , ,   ,CD, ,RIM,LI,D, ,      ,    ,    ,2910,0060,+,01500,     ,     ,-,210,    ,   , , , , , , , , ;",
            "SID:030,2,ANC8A,RW30,RIM,LI,D, ,V   ,R,   ,CF,Y,RIM,LI,D, ,      ,0000,0000,1610,0060,+,04000,     ,     ,-,210,    ,   , , , , , , , , ;",
            "SID:040,2,ANC8A,RW30,D121Z,LI,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,FL095,     ,     , ,   ,    ,   , , , , , , , , ;",
        ];
        let mut chart = crate::nav::parse_proc_legs(rows.into_iter(), crate::nav::ProcKind::Sid, "ANC8A", Some("RW30"));
        let rim = crate::nav::Fix { ident: "RIM".into(), lat: 44.015167, lon: 12.616639 };
        for l in chart.iter_mut() {
            if l.fix_ident == "RIM" {
                l.fix = Some(rim.clone());
            }
            if l.navaid_ident == "RIM" {
                l.navaid = Some(rim.clone());
            }
            if l.fix_ident == "D121Z" {
                l.fix = Some(crate::nav::Fix { ident: "D121Z".into(), lat: 43.8, lon: 12.95 });
            }
        }
        assert_eq!(chart[1].navaid_ident, "RIM");
        let plan_leg = |id: &str, alt: f64| {
            let mut l = chart[2].clone();
            l.fix_ident = id.into();
            l.leg_type = "TF".into();
            l.alt1_ft = Some(alt);
            l.alt_desc = ' ';
            l
        };
        let plan = vec![plan_leg("RIM", 3000.0), plan_leg("D121Z", 5000.0)];
        let dep = super::charted_departure_legs(&chart, &plan, "D121Z").expect("flyable");
        let kinds: Vec<&str> = dep.iter().map(|l| l.leg_type.as_str()).collect();
        assert_eq!(kinds, ["CA", "CD", "CF", "TF"]);
        assert_eq!(dep[0].alt1_ft, Some(450.0), "a climb-to-altitude leg ends at its altitude");
        assert_eq!(dep[1].alt1_ft, Some(5000.0), "the out-leg climbs on toward the join");
        assert_eq!(dep[2].alt1_ft, Some(3000.0), "RIM at the plan's altitude, not the chart's minimum");
        assert_eq!(dep[3].alt1_ft, Some(5000.0), "never FL095 over a 5,000 ft plan");
        // The CD leg ends at 6 DME from RIM, not at its 1,500 ft minimum.
        let out = crate::nav::project_latlon(rim.lat, rim.lon, 291.0, 5.0);
        assert_eq!(crate::procedure::dme_leg_done(&dep[1], out.0, out.1), Some(false));
        let out = crate::nav::project_latlon(rim.lat, rim.lon, 291.0, 6.0);
        assert_eq!(crate::procedure::dme_leg_done(&dep[1], out.0, out.1), Some(true));
        // An arc on the chart: not flown from the chart at all.
        let mut arc = chart.clone();
        arc[1].leg_type = "AF".into();
        assert!(super::charted_departure_legs(&arc, &plan, "D121Z").is_none());
    }

    /// LIPR 30 ANC8A from the installed data: every leg resolves, the DME
    /// leg knows RIM, and the chart comes out flyable.
    /// `cargo test -- --ignored real_install_lipr_anc8a --nocapture`
    #[test]
    #[ignore]
    fn real_install_lipr_anc8a_is_flyable_as_charted() {
        let Ok(cfg) = crate::config::Config::from_env() else {
            return;
        };
        let Ok(Some(apt)) = crate::nav::find_airport(&cfg.apt_data_path, "LIPR") else {
            return;
        };
        let chart = super::sid_chart_legs(&cfg.cifp_dir, &cfg.fix_data_path, &cfg.nav_data_path, &apt, "LIPR", "ANC8A", "30");
        assert!(!chart.is_empty(), "the chart loads");
        for l in &chart {
            println!("{} {} fix={:?} navaid={} {:?} dist={:?} alt={:?}", l.leg_type, l.fix_ident, l.fix.as_ref().map(|f| (f.lat, f.lon)), l.navaid_ident, l.navaid.as_ref().map(|f| (f.lat, f.lon)), l.dist_nm, l.alt1_ft);
        }
        let cd = chart.iter().find(|l| l.leg_type == "CD").expect("the out-leg");
        assert!(cd.navaid.is_some(), "RIM resolved for the DME leg");
        let mut plan_rim = chart.iter().find(|l| l.fix_ident == "RIM").unwrap().clone();
        plan_rim.alt1_ft = Some(3000.0);
        let dep = super::charted_departure_legs(&chart, &[plan_rim], "RIM").expect("flyable");
        assert_eq!(dep.iter().map(|l| l.leg_type.as_str()).collect::<Vec<_>>(), ["CA", "CD", "CF"]);
    }

    #[test]
    fn a_cleared_runway_that_is_not_the_sids_is_said_on_the_ramp() {
        assert!(super::departure_runway_mismatch(Some("26"), Some("08")).unwrap().contains("zero eight"));
        assert_eq!(super::departure_runway_mismatch(Some("26"), Some("26")), None);
        assert_eq!(super::departure_runway_mismatch(None, Some("08")), None, "no SID, no rule");
        assert_eq!(super::departure_runway_mismatch(Some("26"), None), None, "no tower, no rule");
    }

    #[test]
    fn a_cruise_above_the_ceiling_is_refused_at_the_arm() {
        // EDLP-LOWI (2026-09-24): FL290 on an airframe declared to 28,000.
        let mut fp = plan(false);
        fp.cruise_alt_ft = 29_000.0;
        let f = super::PreflightFacts {
            service_ceiling_ft: Some(28_000.0),
            ..facts()
        };
        let why = super::preflight(&fp, &f).expect_err("refused");
        assert!(why.contains("ceiling"), "{why}");
        let ok = super::PreflightFacts {
            service_ceiling_ft: Some(31_000.0),
            ..facts()
        };
        assert!(super::preflight(&fp, &ok).map_or_else(|e| !e.contains("ceiling"), |_| true));
    }

    #[test]
    fn a_plan_that_fits_is_briefed_with_its_fuel_and_its_load() {
        let brief = super::preflight(&plan(true), &facts()).expect("fits");
        assert_eq!(
            brief,
            "Fuel 240 pounds for the plan's 240. \
             3 passengers and 70 pounds of cargo as planned."
        );
        // A local plan has no load sheet and speaks the app's gallons.
        let brief = super::preflight(&plan(false), &facts()).expect("fits");
        assert_eq!(brief, "Fuel 40 gallons for the plan's 40.");
        // Nothing readable, nothing said, nothing refused.
        let brief =
            super::preflight(&plan(true), &super::PreflightFacts::default()).expect("no facts");
        assert_eq!(brief, "3 passengers and 70 pounds of cargo.");
    }

    #[test]
    fn a_runway_too_short_at_either_end_is_refused_at_the_gate() {
        let short_dest = super::PreflightFacts {
            dest_longest_ft: 1200.0,
            ..facts()
        };
        let why = super::preflight(&plan(true), &short_dest).expect_err("refused");
        assert_eq!(
            why,
            // The book 1,335 ft (`aircraft::landing_needed_m`, the
            // self-pick's rule too; refactor audit #10).
            "Negative. LEAX's longest runway is 1,200 feet, and we need 1,335 feet to land. Not flying that."
        );
        // The takeoff needs the roll x 1.4 (1,630 ft -> 2,282 ft at max
        // gross), scaled to the 1,100 kg on the wheels against 1,160: 2,163.
        let short_origin = super::PreflightFacts {
            origin_longest_ft: 2000.0,
            ..facts()
        };
        let why = super::preflight(&plan(true), &short_origin).expect_err("refused");
        assert!(
            why.starts_with(
                "Negative. LXGB's longest runway is 2,000 feet, and the takeoff needs 2,163 feet."
            ),
            "{why}"
        );
        // A field the data lacks (0) is not a refusal: brief_arrival owns that.
        let unknown = super::PreflightFacts {
            origin_longest_ft: 0.0,
            dest_longest_ft: 0.0,
            ..facts()
        };
        assert!(super::preflight(&plan(true), &unknown).is_ok());
    }

    #[test]
    fn overweight_and_under_fuelled_are_refused_with_the_number() {
        let heavy = super::PreflightFacts {
            total_kg: Some(1160.0),
            ..facts()
        };
        let why = super::preflight(&plan(true), &heavy).expect_err("refused");
        assert_eq!(why, "Negative. We're at 2,560 pounds, over the plan's takeoff limit of 2,450 pounds. Not flying that.");
        // Overweight because the cabin is heavier than the load sheet: the
        // refusal says so, since that is what the pilot can change.
        let heavy_cabin = super::PreflightFacts {
            total_kg: Some(1160.0),
            payload_kg: Some(400.0),
            ..facts()
        };
        let why = super::preflight(&plan(true), &heavy_cabin).expect_err("refused");
        assert!(why.contains("plan's takeoff limit of 2,450 pounds. The aircraft is loaded with 880 pounds of payload against the plan's 590 pounds."), "{why}");
        // No load sheet: the profile's max gross (2,558 lb = 1,160 kg) is the limit.
        let heavy = super::PreflightFacts {
            total_kg: Some(1180.0),
            ..facts()
        };
        let why = super::preflight(&plan(false), &heavy).expect_err("refused");
        assert_eq!(why, "Negative. We're at 2,600 pounds, over the max takeoff weight of 2,560 pounds. Not flying that.");

        // Under the minimum takeoff fuel: refused, with both figures.
        let dry = super::PreflightFacts {
            fuel_kg: Some(80.0),
            ..facts()
        };
        let why = super::preflight(&plan(true), &dry).expect_err("refused");
        assert_eq!(why, "Negative. 180 pounds of fuel aboard, and the plan needs at least 230 pounds to go. Say load the aircraft and I'll fuel it from the plan, then say it again.");
        // Between the minimum and the block: said, not refused.
        let light = super::PreflightFacts {
            fuel_kg: Some(104.0),
            ..facts()
        };
        let brief = super::preflight(&plan(true), &light).expect("goes");
        assert!(
            brief.starts_with("Fuel 230 pounds against a 240 block, above the minimum."),
            "{brief}"
        );
    }

    #[test]
    fn a_load_the_sim_disagrees_with_is_said_not_refused() {
        // Two hundred kilos aboard against the plan's 268: off by more than
        // 10 percent and 20 kg, so the pilot hears both numbers.
        let light = super::PreflightFacts {
            payload_kg: Some(200.0),
            ..facts()
        };
        let brief = super::preflight(&plan(true), &light).expect("goes");
        assert!(brief.ends_with("3 passengers and 70 pounds of cargo on the plan, but the aircraft is loaded with 440 pounds against the plan's 590 pounds."), "{brief}");
        // A lone pilot with no cargo.
        let mut solo = plan(true);
        let w = solo.weights.as_mut().unwrap();
        w.pax = 0;
        w.cargo_kg = 0.0;
        let brief = super::preflight(&solo, &facts()).expect("goes");
        assert!(
            brief.ends_with("No passengers or cargo as planned."),
            "{brief}"
        );
    }

    /// EGSS-LFPG, 2026-09-22: eight runway ends read out one by one before
    /// the mission was even said. A field the approaches cover is one clause.
    #[test]
    fn the_arrival_brief_is_sized_to_the_field() {
        let s = |v: &str| v.to_string();
        let ils = |rwy: &str| ("ILS", s(rwy));
        // Charles de Gaulle: four runways, the ILS on all eight ends.
        let cdg: Vec<_> = [
            "zero nine left",
            "two seven right",
            "zero nine right",
            "two seven left",
            "zero eight left",
            "two six right",
            "zero eight right",
            "two six left",
        ]
        .into_iter()
        .map(ils)
        .collect();
        assert_eq!(
            super::arrival_brief_line("Charles De Gaulle", 8, &cdg, &[]),
            "Charles De Gaulle has the ILS on every runway."
        );
        // Mixed kinds still cover the field.
        let mut mixed = cdg.clone();
        mixed[7] = ("RNAV", s("two six left"));
        assert_eq!(
            super::arrival_brief_line("Charles De Gaulle", 8, &mixed, &[]),
            "Charles De Gaulle has a published approach on every runway."
        );
        // Partial cover on a big field is a count, not a list.
        assert_eq!(
            super::arrival_brief_line("Charles De Gaulle", 8, &cdg[..5], &[s("two six left")]),
            "Charles De Gaulle has the ILS on 5 of 8 runway ends."
        );
        // A small field is named end by end.
        assert_eq!(
            super::arrival_brief_line("Exeter", 2, &[ils("two six")], &[s("zero eight")]),
            "Exeter has the ILS on two six and a straight-in on zero eight."
        );
        assert_eq!(
            super::arrival_brief_line("Axarquia", 2, &[], &[]),
            "Axarquia has no published approach, so I'll land visually."
        );
    }

    /// LEJR-LXGB, 2026-09-14: the aircraft's wind at 7,000 ft set up 27; the
    /// field's report said 090 at 21.
    #[test]
    fn the_field_report_outranks_the_wind_aloft_for_the_runway_pick() {
        let metar = Some((90.0, 21.0));
        let aloft = Some((260.0, 5.0));
        assert_eq!(super::arrival_wind(metar, aloft), (metar, "metar"));
        assert_eq!(super::arrival_wind(None, aloft), (aloft, "aircraft"));
        assert_eq!(super::arrival_wind(None, None), (None, "none"));
    }

    use super::{departure_check_in, departure_check_in_above};

    /// The runway chosen at the arm is the runway the arrival sets up for
    /// (LIBP-LEMG 2026-09-26: UNTO2A chosen for 12, the ILS 13 flown after a
    /// re-pick from a METAR that had moved 20 degrees), unless landing there
    /// breaks the arrival's own limits: more tailwind than an approach is
    /// flown with, or no approach we can fly. Gibraltar as the brief test
    /// has it: a straight-in on 27 only.
    #[test]
    fn the_arrival_flies_the_runway_the_arm_chose() {
        use std::io::Write;
        let dir = std::env::temp_dir().join("xfc-arrival-planned");
        std::fs::create_dir_all(&dir).unwrap();
        let apt = dir.join("apt.dat");
        let mut f = std::fs::File::create(&apt).unwrap();
        writeln!(f, "1 11 0 0 LXGB Gibraltar").unwrap();
        writeln!(f, "100 46 0 0 0 0 0 0 09 36.1509000 -5.3581000 0 0 0 0 0 0 27 36.1515000 -5.3412000 0 0 0 0 0 0").unwrap();
        // A nominal/unnamed tower row does not make this a staffed field.
        writeln!(f, "1054 130125 unnamed entity").unwrap();
        f.flush().unwrap();
        let fix = dir.join("earth_fix.dat");
        let mut f = std::fs::File::create(&fix).unwrap();
        writeln!(f, "36.157361111 -5.240083333 27D05 LXGB LX 5906505 RWY27 D05").unwrap();
        writeln!(f, "36.104666667 -5.405333333 09D05 LXGB LX 5906505 RWY09 D05").unwrap();
        f.flush().unwrap();
        let (apt, fix) = (apt.to_str().unwrap().to_string(), fix.to_str().unwrap().to_string());
        let cifp = dir.join("no-cifp").to_str().unwrap().to_string();
        let nav = dir.join("earth_nav.dat").to_str().unwrap().to_string();
        let build = |wind: Option<(f64, f64)>, planned: Option<&str>| {
            super::build_arrival(
                &apt, &cifp, &fix, &nav, "LXGB", 36.16, -5.05,
                crate::tools::autopilot::ApConfig::default(), 87.0, wind, None, planned,
            )
        };
        // Eight knots of tailwind: inside an approach's limit, the arm's 27
        // stands.
        let a = build(Some((90.0, 8.0)), Some("27"));
        assert_eq!((a.rwy_id.as_deref(), a.overruled), (Some("27"), None));
        assert!(a.plan.is_some(), "the straight-in to 27");
        assert_eq!(a.tower_khz, None, "no invented controller");
        // Fifteen knots of it is past an approach's limit: overruled, said.
        let a = build(Some((90.0, 15.0)), Some("27"));
        assert_eq!((a.rwy_id.as_deref(), a.overruled), (Some("09"), Some("tailwind")));
        assert_eq!(
            super::overruled_line("27", a.rwy_id.as_deref(), "tailwind").unwrap(),
            "The wind has gone round: too much tailwind for runway two seven. Setting up for runway zero nine."
        );
        // An end with no approach we can fly hands over to one that has.
        let a = build(None, Some("09"));
        assert_eq!((a.rwy_id.as_deref(), a.overruled), (Some("27"), Some("no_approach")));
        // A runway the field does not have is no runway.
        let a = build(Some((270.0, 10.0)), Some("13"));
        assert_eq!((a.rwy_id.as_deref(), a.overruled), (Some("27"), Some("not_at_field")));
        // The tower's runway outranks the arm's.
        let a = super::build_arrival(
            &apt, &cifp, &fix, &nav, "LXGB", 36.16, -5.05,
            crate::tools::autopilot::ApConfig::default(), 87.0, None, Some("09"), Some("27"),
        );
        assert_eq!((a.rwy_id.as_deref(), a.overruled), (Some("09"), None));
        // Nothing is said when nothing changed.
        assert_eq!(super::overruled_line("27", Some("27"), "tailwind"), None);
    }

    /// The seam every reader of the arrival runway shares: set at the arm,
    /// overwritten only through `set_arrival_runway`, cleared with the
    /// mission.
    /// The arrival-runway tests share one process-wide value.
    static ARRIVAL_RUNWAY_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn the_arrival_runway_is_one_value() {
        let _g = ARRIVAL_RUNWAY_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        super::set_arrival_runway(Some("12".into()), "arm");
        assert_eq!(super::arrival_runway().as_deref(), Some("12"));
        super::set_arrival_runway(Some("13".into()), "tower");
        assert_eq!(super::arrival_runway().as_deref(), Some("13"));
        super::stop_active();
        assert_eq!(super::arrival_runway(), None);
    }

    /// Issue #64: the tower clears us for the other end on the approach.
    /// Autoland's watch finds the switch (`clearance_switch`), moves the
    /// mission's ONE arrival runway, and the re-armed loop tunes NAV1 from
    /// that value: the ILS 30's frequency and course, not the 12's. A
    /// go-around reads no switch and moves nothing; with no mission there is
    /// nothing to move; the mission's end clears it.
    #[tokio::test]
    async fn a_clearance_switch_moves_the_arrival_runway_and_the_ils() {
        use crate::tower::State::{ClearedToLand, Inbound};
        let _g = ARRIVAL_RUNWAY_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // No mission: a standalone autoland invents no arrival runway.
        super::stop_active();
        assert!(!super::retarget_arrival_runway("30", "clearance"));
        assert_eq!(super::arrival_runway(), None);

        // Armed for 12; autoland's arm flies the forced end: no change.
        super::set_arrival_runway(Some("12".into()), "arm");
        assert_eq!(crate::autoland::armed_runway_why(Some("12"), "12", false), "arm");
        assert!(super::retarget_arrival_runway("12", "arm"));
        assert_eq!(super::arrival_runway().as_deref(), Some("12"));

        // A go-around: the ident published with Inbound is the runway in
        // use, never a clearance. Nothing moves.
        assert_eq!(
            crate::autoland::clearance_switch("12", Some(Inbound), Some("30"), None, true, None),
            None
        );
        assert_eq!(super::arrival_runway().as_deref(), Some("12"));

        // "Cleared to land runway three zero" on the approach.
        let to = crate::autoland::clearance_switch(
            "12",
            Some(ClearedToLand),
            Some("30"),
            None,
            false,
            None,
        )
        .expect("an arrival clearance to the other end is a switch");
        assert!(super::retarget_arrival_runway(&to, "clearance"));
        assert_eq!(super::arrival_runway().as_deref(), Some("30"));
        // The new loop's arm flies the end handed down: no second event.
        assert_eq!(crate::autoland::armed_runway_why(Some("30"), "30", false), "arm");
        // A switch found at the arm itself is the tower's too.
        assert_eq!(crate::autoland::armed_runway_why(Some("12"), "30", true), "clearance");
        assert_eq!(crate::autoland::armed_runway_why(None, "30", false), "self_pick");

        // The re-armed loop's ILS tune reads the one value: NAV1 goes to
        // the 30's localiser (LEMG-shaped fixture), course 300 magnetic.
        let dir = std::env::temp_dir().join(format!("xvl_issue64_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let nav = dir.join("earth_nav.dat");
        std::fs::write(
            &nav,
            "I\n1200 Version\n\n\
             4  36.66000000  -4.47000000     52 10990  18  43318.000 IMLG LEMG LE 12 ILS-cat-I\n\
             4  36.69000000  -4.51000000     52 11150  18 108298.000 IMGA LEMG LE 30 ILS-cat-I\n\
             99\n",
        )
        .unwrap();
        let xp = crate::xplane::FakeXPlane::with_reads(&[
            (crate::ils::NAV1_FREQ, 10990.0),
            (crate::ils::NAV1_OBS, 120.0),
        ]);
        let rwy = super::arrival_runway().unwrap();
        let _ = crate::ils::tune_from(&xp, nav.to_str().unwrap(), "LEMG", &rwy, 36.60, -4.40).await;
        assert_eq!(xp.last_write(crate::ils::NAV1_FREQ), Some(11150.0));
        assert!((xp.last_write(crate::ils::NAV1_OBS).unwrap() - 300.0).abs() < 1.0);
        let _ = std::fs::remove_dir_all(&dir);

        // The mission's end clears it; a later switch has nothing to move.
        super::stop_active();
        assert_eq!(super::arrival_runway(), None);
        assert!(!super::retarget_arrival_runway("12", "clearance"));
    }

    // The arrival is briefed at the gate. Gibraltar, as the data has it: no
    // coded approach, DME fixes on 27's centreline, 09's off it.
    #[test]
    fn the_gate_brief_says_what_the_destination_offers() {
        use std::io::Write;
        let dir = std::env::temp_dir().join("xfc-brief");
        std::fs::create_dir_all(&dir).unwrap();
        let apt = dir.join("apt.dat");
        let mut f = std::fs::File::create(&apt).unwrap();
        writeln!(f, "1 11 0 0 LXGB Gibraltar").unwrap();
        writeln!(f, "100 46 0 0 0 0 0 0 09 36.1509000 -5.3581000 0 0 0 0 0 0 27 36.1515000 -5.3412000 0 0 0 0 0 0").unwrap();
        f.flush().unwrap();
        let fix = dir.join("earth_fix.dat");
        let mut f = std::fs::File::create(&fix).unwrap();
        writeln!(
            f,
            "36.157361111 -5.240083333 27D05 LXGB LX 5906505 RWY27 D05"
        )
        .unwrap();
        writeln!(
            f,
            "36.104666667 -5.405333333 09D05 LXGB LX 5906505 RWY09 D05"
        )
        .unwrap();
        f.flush().unwrap();
        let cifp = dir.join("no-cifp");
        let brief = super::brief_arrival(
            apt.to_str().unwrap(),
            cifp.to_str().unwrap(),
            fix.to_str().unwrap(),
            "LXGB",
            None,
        )
        .expect("a brief");
        assert_eq!(brief, "Gibraltar has a straight-in on two seven.");
        // The runway the arm chose is briefed with what it offers.
        let planned = |rwy| {
            super::brief_arrival(
                apt.to_str().unwrap(),
                cifp.to_str().unwrap(),
                fix.to_str().unwrap(),
                "LXGB",
                Some(rwy),
            )
            .unwrap()
        };
        assert_eq!(
            planned("27"),
            "Gibraltar has a straight-in on two seven. Planned runway two seven, the straight-in."
        );
        assert_eq!(
            planned("09"),
            "Gibraltar has a straight-in on two seven. Planned runway zero nine."
        );
        // A destination the data doesn't have is a refusal, at the gate.
        let why = super::brief_arrival(
            apt.to_str().unwrap(),
            cifp.to_str().unwrap(),
            fix.to_str().unwrap(),
            "ZZZZ",
            None,
        )
        .expect_err("refused");
        assert!(why.contains("ZZZZ"), "{why}");
    }

    // The first call to Departure has to read as a CHECK-IN to the station
    // (a readback carrying "passing"/"climbing", never a request), or the
    // departure controller answers "say again" instead of "radar contact".
    #[test]
    fn a_climb_above_the_transition_altitude_checks_in_with_a_flight_level() {
        let cs = Some("Speedbird three zero six".to_string());
        let call = departure_check_in_above(&cs, "London Departure", 3000.0, 15000.0, 6000.0);
        assert_eq!(
            call,
            "London Departure, Speedbird three zero six, passing three thousand climbing flight level one five zero"
        );
        let call = departure_check_in_above(&cs, "London Departure", 7020.0, 15000.0, 6000.0);
        assert!(call.contains("passing flight level seven zero climbing flight level one five zero"), "{call}");
        // At or below the transition altitude it stays feet.
        let call = departure_check_in_above(&cs, "London Departure", 2000.0, 6000.0, 6000.0);
        assert!(call.ends_with("passing two thousand climbing six thousand"), "{call}");
        // The departure controller clears the climb the check-in announced,
        // in its own words.
        assert_eq!(
            crate::party_line::climb_announced(
                &call.replace("six thousand", "flight level one two zero")
            ),
            Some("flight level one two zero".into())
        );
    }

    #[test]
    fn the_departure_check_in_is_shaped_to_earn_radar_contact() {
        let cs = Some("November four five X-ray Sierra".to_string());
        let call = departure_check_in(&cs, "Malaga Departure", 1960.0, 4000.0);
        assert_eq!(
            call,
            "Malaga Departure, November four five X-ray Sierra, passing two thousand climbing four thousand"
        );
        assert!(crate::tower::is_readback(&call), "{call}");
        assert_eq!(
            crate::tower::parse_intent(&call),
            crate::tower::Intent::Unknown,
            "{call}"
        );
        // Already at the planned altitude: no invented climb.
        let level = departure_check_in(&None, "Malaga Departure", 3980.0, 4000.0);
        assert_eq!(level, "Malaga Departure, passing four thousand");
    }

    /// The runway waiver and the departure minimums are the mission's: set
    /// only by its arm and takeoff, cleared at its start, every stop, a new
    /// flight and a new airframe (never set true here: the runway gates of
    /// other tests read the waiver).
    #[test]
    fn the_mission_facts_die_with_the_mission() {
        super::forget_mission();
        assert!(!crate::aircraft::takeoff_margin_waived());
        assert!(!super::departure_minimums());
        let src = crate::test_support::module_source("full_auto");
        let body = |head: &str| {
            let rest = src.split(head).nth(1).expect(head);
            rest[..rest.find("\n}\n").unwrap()].to_string()
        };
        assert!(body("pub fn abort() -> bool {").contains("forget_mission();"));
        assert!(body("fn stop_active() {").contains("forget_mission();"));
        let state = crate::test_support::module_source("flight_state");
        assert_eq!(state.matches("crate::full_auto::forget_mission();").count(), 2);
        // The waiver's one writer is the mission.
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        for f in crate::test_support::rust_files(&root) {
            let rel = f.strip_prefix(&root).unwrap().display().to_string();
            let ours = |m: &str| rel == format!("{m}.rs") || rel.starts_with(&format!("{m}/"));
            if crate::test_support::is_test_file(&f) || ours("full_auto") || ours("aircraft") {
                continue;
            }
            let text = std::fs::read_to_string(&f).unwrap();
            assert!(!text.contains("set_takeoff_margin_waived("), "{rel} writes the waiver");
        }
    }

/// Full auto's calls go out through `radio_out` / `radio_readback`, which
/// hold every call once the mission is aborted: a raw transmit sent "request
/// IFR clearance" 34 s after a cancel (LXGB 2026-10-06).
#[test]
fn every_mission_call_goes_through_the_abort_door() {
    let src = include_str!("../full_auto.rs");
    let raw: Vec<&str> = src
        .lines()
        .filter(|l| {
            (l.contains("crew_radio::transmit(") || l.contains("crew_radio::transmit_readback("))
                && !l.trim_start().starts_with("//")
        })
        .collect();
    assert_eq!(raw.len(), 2, "only the door transmits: {raw:#?}");
}

/// A give-up withdraws what the mission queued and the main loop had not
/// yet sent.
#[test]
fn a_withdraw_empties_the_crew_queue() {
    let _q = crate::crew_radio::QUEUE_LOCK
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    crate::crew_radio::forget_flight_calls();
    crate::crew_radio::transmit("Gibraltar Tower, request IFR clearance".into());
    crate::crew_radio::transmit("Gibraltar Tower, request clearance".into());
    assert!(crate::crew_radio::crew_radio_pending());
    let gone = crate::crew_radio::withdraw_queued();
    assert_eq!(gone.len(), 2, "{gone:?}");
    assert!(!crate::crew_radio::crew_radio_pending());
}

/// EGHL 2026-10-06: the Zibo's Boeing start refused on 0 psi of duct
/// pressure and said "Your aircraft.", and full auto then held the
/// aircraft for the five-minute class deadline. An inline start's reply is
/// its verdict: engines not running once it has answered ends the mission
/// with the start's own words, said once.
#[test]
fn an_inline_start_that_stopped_ends_the_mission_with_its_own_words() {
    use super::inline_start_hand_back;
    let duct = "Duct pressure 0 P.S.I., not enough to start number two. Check the A.P.U. and its bleed. Your aircraft.";
    assert_eq!(
        inline_start_hand_back(false, Some(duct)),
        Some((duct.to_string(), "start_stopped"))
    );
    assert_eq!(
        inline_start_hand_back(true, Some("Both engines running. After-start flow complete.")),
        None
    );
    // A reply that does not hand back is still a hand-back without engines.
    assert_eq!(
        inline_start_hand_back(false, Some("Number two, fuel in.")).map(|(l, _)| l),
        Some("Number two, fuel in. The engines aren't running. Your aircraft.".to_string())
    );
    assert_eq!(
        inline_start_hand_back(false, None).map(|(l, _)| l),
        Some("The start didn't run. Your aircraft.".to_string())
    );
    // Only the airliner class answers inline; the others fire and return.
    use crate::tools::full_auto::StartKind;
    assert!(StartKind::Airliner.start_is_inline());
    for k in [
        StartKind::Piston,
        StartKind::Turboprop,
        StartKind::Jet,
        StartKind::Scripted,
        StartKind::None,
    ] {
        assert!(!k.start_is_inline(), "{k:?}");
    }
}

#[test]
fn the_preflight_names_only_what_did_not_take() {
    use super::preflight_line;
    assert_eq!(preflight_line(&[]), None);
    assert_eq!(
        preflight_line(&["yaw dumper".into()]).as_deref(),
        Some("Preflight done, but the yaw dumper didn't take.")
    );
    assert_eq!(
        preflight_line(&["yaw dumper".into(), "trim air".into()]).as_deref(),
        Some("Preflight done, but 2 switches didn't take: yaw dumper, trim air.")
    );
}

/// The preflight is the Zibo's alone: every other airframe goes from the
/// battery to the ramp exactly as before, its panel lights included.
#[test]
fn only_the_zibo_declares_a_preflight_and_leaves_out_the_panel_lights() {
    let mut seen = 0;
    for entry in std::fs::read_dir("profiles").expect("profiles dir") {
        let path = entry.expect("entry").path();
        if path.extension().and_then(|e| e.to_str()) != Some("yaml") {
            continue;
        }
        let id = path.file_stem().and_then(|s| s.to_str()).expect("id").to_string();
        let text = std::fs::read_to_string(&path).expect("read");
        let p: crate::aircraft::AircraftProfile =
            serde_yaml::from_str(&text).unwrap_or_else(|e| panic!("{id}: {e}"));
        seen += 1;
        if id == "zibo_738" {
            let pf = p.preflight.as_ref().expect("zibo preflight");
            assert!(pf.apu_first);
            assert!(p.boeing_start.as_ref().is_some_and(|b| !b.apu_steps.is_empty()));
            let align = pf.align.as_ref().expect("irs alignment");
            assert!(align.position_entry.is_some());
            // The IRS goes first: the alignment is the long pole.
            assert!(pf.steps[0].command.contains("irs_L"), "{:?}", pf.steps[0]);
            assert!(!p.ramp_panel_lights);
        } else {
            assert!(p.preflight.is_none(), "{id}");
            assert!(p.ramp_panel_lights, "{id}");
        }
    }
    assert!(seen > 5);
}
