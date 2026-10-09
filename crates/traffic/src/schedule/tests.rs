use super::*;
use crate::field::fixtures;

fn kinds(ms: &[Movement]) -> (usize, usize, usize) {
    let n = |k| ms.iter().filter(|m| m.kind == k).count();
    (
        n(MovementKind::Circuit),
        n(MovementKind::Departure),
        n(MovementKind::Arrival),
    )
}

#[test]
fn a_circuit_field_with_procedures_gets_all_three_kinds() {
    let f = fixtures::xtst_ifr();
    let ms = generate(&f, Density::Light, 1, &SpawnCtx::default());
    let (c, d, a) = kinds(&ms);
    assert_eq!(c, 1, "light: one circuit aircraft");
    assert!(
        d >= 2 && a >= 2,
        "every 20 minutes for an hour: {d} out, {a} in"
    );
    assert!(ms.windows(2).all(|w| w[0].t_s <= w[1].t_s), "in time order");
    assert!(ms.iter().all(|m| m.t_s < HORIZON_S));
    // 1,500 m: turboprops, never a jet.
    assert!(ms
        .iter()
        .filter(|m| m.kind != MovementKind::Circuit)
        .all(|m| m.icao_type == "AT76"));
    // 27 has the approach, so it is the runway and arrivals come straight in.
    assert!(ms.iter().all(|m| m.runway == "27"));
    assert!(ms
        .iter()
        .filter(|m| m.kind == MovementKind::Arrival)
        .all(|m| m.straight_in));
    let normal = generate(&f, Density::Normal, 1, &SpawnCtx::default());
    let (c, d, a) = kinds(&normal);
    assert!((1..=3).contains(&c), "normal: one to three circuits, {c}");
    // Four stands: the circuits and the departures share them.
    assert!(c + d <= 4, "never more ground starts than stands");
    assert!(a >= 4, "every 10 minutes: {a} in");
}

#[test]
fn a_field_that_says_no_circuits_gets_none() {
    let f = fixtures::xnoc();
    assert_eq!(f.airport.allows_circuits, Some(false));
    let ms = generate(&f, Density::Normal, 7, &SpawnCtx::default());
    let (c, d, a) = kinds(&ms);
    assert_eq!(c, 0);
    assert!(d > 0 && a > 0, "{d} out, {a} in");
    assert!(ms.iter().all(|m| m.runway == "12"), "the end with the ILS");
    assert!(ms
        .iter()
        .filter(|m| m.kind == MovementKind::Departure)
        .all(|m| m.sid.as_deref().is_some_and(|s| s.starts_with("NOC"))));
    assert!(ms
        .iter()
        .filter(|m| m.kind == MovementKind::Arrival)
        .all(|m| m.star.as_deref() == Some("NOC1S") && m.straight_in));
    // A departure only on a stand that takes its class.
    for m in ms.iter().filter(|m| m.kind == MovementKind::Departure) {
        let s = f.stand(m.stand.as_deref().unwrap()).unwrap();
        let cat = if m.icao_type == "AT76" {
            "turboprops"
        } else {
            "jets"
        };
        assert!(
            s.accepts.iter().any(|a| a == cat),
            "{} on {}",
            m.icao_type,
            s.name
        );
    }
}

#[test]
fn a_bare_strip_gets_nothing_to_fly() {
    let f = fixtures::xbar();
    assert!(f.net.stands.is_empty() && f.procs == Default::default());
    assert!(generate(&f, Density::Normal, 3, &SpawnCtx::default()).is_empty());
}

#[test]
fn off_is_nothing_and_the_words_parse() {
    let f = fixtures::xtst_ifr();
    assert!(generate(&f, Density::Off, 1, &SpawnCtx::default()).is_empty());
    assert_eq!(Density::parse(" Normal "), Some(Density::Normal));
    assert_eq!(Density::parse("light"), Some(Density::Light));
    assert_eq!(Density::parse("off"), Some(Density::Off));
    assert_eq!(Density::parse("busy"), None);
}

/// THE FIELD IS NOT EMPTY FOR MINUTES (LEMG, 2026-10-08: the first
/// movement came four to six minutes after the fleet started): the first
/// movement is within [`FIRST_MOVE_S`], a ground start where there is one,
/// its stream brought forward whole (the spacing kept, so the hour's count
/// is the hour's), the other streams untouched.
#[test]
fn the_first_movement_comes_within_a_minute() {
    for f in [fixtures::xtst_ifr(), fixtures::xnoc()] {
        for density in [Density::Light, Density::Normal] {
            for seed in 0..30 {
                let ms = generate(&f, density, seed, &SpawnCtx::default());
                let first = &ms[0];
                assert!(
                    (FIRST_MOVE_S.0..=FIRST_MOVE_S.1).contains(&first.t_s),
                    "{} {density:?} seed {seed}: first at {:.0} s",
                    f.icao,
                    first.t_s
                );
                assert_ne!(first.kind, MovementKind::Arrival, "a ground start first");
                assert!(ms.iter().all(|m| m.t_s >= first.t_s && m.t_s < HORIZON_S));
                // The departures keep their spacing (0.8 to 1.2 intervals).
                let every = if density == Density::Light {
                    1200.0
                } else {
                    600.0
                };
                let deps: Vec<f64> = ms
                    .iter()
                    .filter(|m| m.kind == MovementKind::Departure)
                    .map(|m| m.t_s)
                    .collect();
                assert!(
                    deps.windows(2)
                        .all(|w| (0.8 * every - 1e-6..=1.2 * every + 1e-6).contains(&(w[1] - w[0]))),
                    "{deps:?}"
                );
            }
        }
    }
}

/// ULMM's real layout (12 stands) with SIDs and approaches named, so it
/// schedules departures and arrivals.
fn ulmm() -> FieldModel {
    let mut f = FieldModel::load(&fixtures::path("ulmm.dat"), "ULMM")
        .unwrap()
        .unwrap();
    f.procs.sids = vec!["ASGO2E".into(), "EKLE2G".into()];
    f.procs.approaches = vec!["I13-Y".into(), "I31-Y".into()];
    f
}

/// THE FIELD IS NOT EMPTY AT THE START (LEMG, 2026-10-08: no parked
/// aircraft at all): a share of the suitable stands is taken from the
/// start, the departures first (each waits parked on its own stand), the
/// rest parked aircraft of the class the stand takes, never on a stand the
/// schedule or we hold, with stands kept free for the arrivals; the same
/// seed, the same field; nothing at `Off`.
#[test]
fn a_share_of_the_stands_is_parked_at_the_start() {
    let f = ulmm();
    let suitable = f.net.stands.iter().filter(|s| s.is_suitable()).count();
    let own = f.stand("Stand 9").map(|s| (s.lat, s.lon)).unwrap();
    let ctx = SpawnCtx {
        own: Some(own),
        ..Default::default()
    };
    for (density, share) in [
        (Density::Light, PARKED_SHARE_LIGHT),
        (Density::Normal, PARKED_SHARE_NORMAL),
    ] {
        for seed in 0..10 {
            let mut ms = generate(&f, density, seed, &ctx);
            let parked = park_at_start(&f, density, seed, &ctx, &mut ms);
            let waiting = ms.iter().filter(|m| m.parked_from_start).count();
            let budget = (share * suitable as f64).floor() as usize;
            assert!(waiting + parked.len() <= budget, "{density:?} {seed}");
            assert!(
                ms.iter()
                    .filter(|m| m.parked_from_start)
                    .all(|m| m.kind == MovementKind::Departure),
                "only departures wait on their stands"
            );
            // The earliest departures wait.
            let deps: Vec<&Movement> = ms
                .iter()
                .filter(|m| m.kind == MovementKind::Departure)
                .collect();
            assert!(deps
                .windows(2)
                .all(|w| w[0].parked_from_start || !w[1].parked_from_start));
            let held: Vec<&str> = ms.iter().filter_map(|m| m.stand.as_deref()).collect();
            let mut names: Vec<&str> = parked.iter().filter_map(|m| m.stand.as_deref()).collect();
            for p in &parked {
                assert_eq!(p.kind, MovementKind::Parked);
                assert_eq!(p.t_s, 0.0);
                let name = p.stand.as_deref().unwrap();
                assert!(!held.contains(&name), "{name}: the schedule's stand");
                assert_ne!(name, "Stand 9", "our stand");
                let s = f.stand(name).unwrap();
                // The stand's class: ULMM's stands take jets.
                let cat = match p.icao_type.as_str() {
                    "AT76" => "turboprops",
                    "C172" | "C152" | "PA28" | "SR22" => "props",
                    _ => "jets",
                };
                assert!(
                    s.accepts.is_empty() || s.accepts.iter().any(|a| a == cat),
                    "{} on {name} {:?}",
                    p.icao_type,
                    s.accepts
                );
                if cat != "props" {
                    assert!(fleet_of(&p.callsign[..3]).contains(&p.icao_type.as_str()));
                }
                assert!(p.spawn(1, &f, &ctx).is_ok_and(|a| a.dormant()));
            }
            names.sort();
            names.dedup();
            assert_eq!(names.len(), parked.len(), "one aircraft a stand");
            let free = suitable - held.len() - parked.len() - 1;
            assert!(free + 1 >= STANDS_KEPT_FREE, "stands left for the arrivals");
            let mut again = generate(&f, density, seed, &ctx);
            assert_eq!(park_at_start(&f, density, seed, &ctx, &mut again), parked);
            assert_eq!(again, ms);
        }
    }
    let mut ms = generate(&f, Density::Normal, 1, &ctx);
    let parked = park_at_start(&f, Density::Normal, 1, &ctx, &mut ms);
    assert!(
        !parked.is_empty() || ms.iter().any(|m| m.parked_from_start),
        "normal parks something at ULMM"
    );
    assert!(park_at_start(&f, Density::Off, 1, &ctx, &mut ms).is_empty());
}

/// A parked aircraft is never put on our stand or within 60 m of us, and a
/// departure waiting on its stand starts up there when it is released.
#[test]
fn a_parked_departure_starts_up_on_its_stand_when_released() {
    let f = ulmm();
    let mut ms = generate(&f, Density::Normal, 1, &SpawnCtx::default());
    park_at_start(&f, Density::Normal, 1, &SpawnCtx::default(), &mut ms);
    let m = ms
        .iter()
        .find(|m| m.parked_from_start)
        .expect("a departure waits")
        .clone();
    let stand = f.stand(m.stand.as_deref().unwrap()).unwrap();
    let near = SpawnCtx {
        own: Some(project_m((stand.lat, stand.lon), 90.0, 40.0)),
        ..Default::default()
    };
    assert_eq!(m.park(1, &f, &near).unwrap_err(), Refusal::NearOwn);
    let ac = m.park(1, &f, &SpawnCtx::default()).unwrap();
    assert!(ac.dormant() && ac.phase == crate::phase::Phase::Parked);
    assert_eq!(ac.stand.as_deref(), Some(stand.name.as_str()));
    assert!(!ac.kin.lights.beacon, "dark");
    let mut fleet = crate::Fleet::new(f.clone());
    fleet.spawn(ac);
    let events = fleet.step(30.0, None, &|_| None);
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, crate::phase::Event::Call { .. })),
        "a parked aircraft says nothing"
    );
    assert!(fleet.release(1, m.plan(&f).unwrap()));
    assert!(!fleet.release(1, m.plan(&f).unwrap()), "once");
    let events = fleet.step(1.0, None, &|_| None);
    assert!(events.iter().any(|e| matches!(
        e,
        crate::phase::Event::Call {
            call: crate::phase::Call::Taxi,
            ..
        }
    )));
    let ac = fleet.get(1).unwrap();
    assert!(
        !ac.dormant() && ac.kin.lights.beacon,
        "started up where it stood"
    );
    assert_eq!(ac.kin.pos(), (stand.lat, stand.lon));
}

#[test]
fn the_same_seed_is_the_same_schedule() {
    let f = fixtures::xtst_ifr();
    let a = generate(&f, Density::Normal, 42, &SpawnCtx::default());
    assert_eq!(a, generate(&f, Density::Normal, 42, &SpawnCtx::default()));
    assert_ne!(a, generate(&f, Density::Normal, 43, &SpawnCtx::default()));
}

#[test]
fn nothing_is_put_on_an_occupied_stand() {
    let f = fixtures::xtst_ifr();
    let stand = |n: &str| {
        let s = f.stand(n).unwrap();
        (s.lat, s.lon)
    };
    // We are on GA1, somebody is on GA2 and GA3: only GA4 is free.
    let ctx = SpawnCtx {
        own: Some(stand("GA1")),
        occupied: vec![stand("GA2"), stand("GA3")],
        ..Default::default()
    };
    for seed in 0..20 {
        let ms = generate(&f, Density::Normal, seed, &ctx);
        let used: Vec<&str> = ms.iter().filter_map(|m| m.stand.as_deref()).collect();
        assert!(used.iter().all(|s| *s == "GA4"), "seed {seed}: {used:?}");
        assert!(used.len() <= 1, "GA4 handed out once: {used:?}");
    }
    // At spawn: a named stand taken now moves to a free one, and none free
    // is a refusal.
    let m = generate(&f, Density::Light, 5, &SpawnCtx::default())
        .into_iter()
        .find(|m| m.kind == MovementKind::Circuit)
        .unwrap();
    let mut named = m.clone();
    named.stand = Some("GA2".into());
    let ac = named.spawn(1, &f, &ctx).unwrap();
    assert_eq!(ac.stand.as_deref(), Some("GA4"));
    let full = SpawnCtx {
        occupied: vec![stand("GA2"), stand("GA3"), stand("GA4")],
        ..ctx.clone()
    };
    assert_eq!(named.spawn(1, &f, &full).unwrap_err(), Refusal::NoStand);
}

#[test]
fn nothing_spawns_in_the_air_near_us() {
    let f = fixtures::xtst_ifr();
    let end = f.end("27").unwrap();
    // Sitting on the default entry, ten miles out on the centreline.
    let own = default_entry(&end);
    let ctx = SpawnCtx {
        own: Some(own),
        ..Default::default()
    };
    for seed in 0..20 {
        for m in generate(&f, Density::Normal, seed, &ctx)
            .iter()
            .filter(|m| m.kind == MovementKind::Arrival)
        {
            let d = dist_m(own, m.entry.unwrap()) / 1852.0;
            assert!(d >= OWN_CLEAR_NM, "seed {seed}: entry {d:.1} nm from us");
        }
    }
    let m = generate(&f, Density::Light, 1, &SpawnCtx::default())
        .into_iter()
        .find(|m| m.kind == MovementKind::Arrival)
        .unwrap();
    let on_it = SpawnCtx {
        own: m.entry,
        ..Default::default()
    };
    assert_eq!(m.spawn(9, &f, &on_it).unwrap_err(), Refusal::NearOwn);
    assert!(m.spawn(9, &f, &SpawnCtx::default()).is_ok());
}

const FILE: &str = "
- t_s: 0
  callsign: N345SP
  type: C172
  kind: circuit
  stand: GA2
  laps: 2
  spoken: Cessna three four five
- t_s: 300
  callsign: RYR12
  type: b738
  kind: arrival
- t_s: 120
  callsign: EZY7
  type: A320
  kind: departure
  runway: \"09\"
";

#[test]
fn the_traffic_file_parses_and_resolves_at_the_field() {
    let rows = parse_file(FILE).unwrap();
    assert_eq!(rows.len(), 3);
    assert_eq!(rows[0].kind, MovementKind::Circuit);
    assert_eq!(rows[0].laps, Some(2));
    assert_eq!(rows[1].runway, None);
    let f = fixtures::xtst_ifr();
    let ms = from_file(&rows, &f, 1, &SpawnCtx::default());
    assert_eq!(ms.len(), 3);
    assert_eq!(
        ms.iter().map(|m| m.callsign.as_str()).collect::<Vec<_>>(),
        ["N345SP", "EZY7", "RYR12"],
        "time order"
    );
    assert_eq!(ms[0].spoken, "Cessna three four five");
    assert_eq!(
        ms[0].runway, "27",
        "the field's runway when the row names none"
    );
    assert_eq!(ms[1].runway, "09");
    assert_eq!(ms[2].icao_type, "B738");
    assert!(ms[2].entry.is_some() && ms[2].straight_in);
    let ac = ms[0].spawn(1, &f, &SpawnCtx::default()).unwrap();
    assert_eq!(ac.stand.as_deref(), Some("GA2"));
}

#[test]
fn a_bad_traffic_file_is_refused_whole() {
    assert!(parse_file("- t_s: 0\n  callsign: X\n  type: C172\n  kind: hover\n").is_err());
    assert!(parse_file("- t_s: -5\n  callsign: X\n  type: C172\n  kind: circuit\n").is_err());
    assert!(
        parse_file("- t_s: 0\n  callsign: X\n  type: C172\n  kind: circuit\n  colour: red\n")
            .is_err()
    );
    assert!(parse_file("- t_s: 0\n  callsign: ' '\n  type: C172\n  kind: circuit\n").is_err());
    assert!(parse_file("[]").unwrap().is_empty());
    // A runway the field lacks drops the row, not the file.
    let rows =
        parse_file("- t_s: 0\n  callsign: X\n  type: C172\n  kind: circuit\n  runway: '18'\n")
            .unwrap();
    assert!(from_file(&rows, &fixtures::xtst(), 1, &SpawnCtx::default()).is_empty());
}

#[test]
fn callsigns_are_spoken_the_way_the_region_writes_them() {
    let mut r = XorShift64::new(3);
    let (cs, spoken) = registration("G", "C172", &mut r);
    assert!(cs.starts_with('G') && cs.len() == 5, "{cs}");
    assert!(spoken.starts_with("Cessna Golf "), "{spoken}");
    let (cs, spoken) = registration("N", "PA28", &mut r);
    assert!(cs.starts_with('N') && cs.len() == 6, "{cs}");
    assert!(spoken.starts_with("Piper "), "{spoken}");
    let (cs, spoken, _) = airline("LEMG", "A320", &mut r);
    assert!(
        cs.len() >= 6 && spoken.split(' ').count() >= 4,
        "{cs}: {spoken}"
    );
}

/// The type is the airline's: a Ryanair is a 737, a turboprop a regional
/// carrier's (a Ryanair A320 drew a Wizz Air livery at LEMG, 2026-10-08).
#[test]
fn every_airline_flies_its_own_type() {
    let mut r = XorShift64::new(7);
    for _ in 0..200 {
        for t in ["A320", "B738", "A319", "AT76"] {
            let (cs, _, ty) = airline("LEMG", t, &mut r);
            assert!(fleet_of(&cs[..3]).contains(&ty), "{cs} flies {ty}");
            if t == "AT76" {
                assert_eq!(ty, "AT76", "{cs}");
            }
        }
    }
}

/// The real Malaga with its CIFP: no circuits (it says 0), IFR both ways.
/// `cargo test -p traffic -- --ignored real_install_schedule --nocapture`
#[test]
#[ignore] // needs the real X-Plane install
fn real_install_schedule_lemg() {
    let apt = nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(f)) = FieldModel::load(&apt, "LEMG") else {
        return;
    };
    let f = f.with_cifp(&nav::test_xplane_path("Custom Data/CIFP"));
    let ms = generate(&f, Density::Normal, 1, &SpawnCtx::default());
    for m in &ms {
        println!(
            "{:>5.0} s {:?} {} {} rwy {} stand {:?} sid {:?} star {:?}",
            m.t_s, m.kind, m.callsign, m.icao_type, m.runway, m.stand, m.sid, m.star
        );
    }
    let (c, d, a) = kinds(&ms);
    assert_eq!(c, 0, "LEMG publishes no circuits");
    assert!(d > 0 && a > 0, "{d} out, {a} in");
    let mut parked_ms = ms.clone();
    let parked = park_at_start(&f, Density::Normal, 1, &SpawnCtx::default(), &mut parked_ms);
    let waiting = parked_ms.iter().filter(|m| m.parked_from_start).count();
    let suitable = f.net.stands.iter().filter(|s| s.is_suitable()).count();
    println!(
        "parked at the start: {} aircraft and {waiting} departures waiting, of {suitable} stands",
        parked.len()
    );
    for m in &parked {
        println!("  {} {} on {:?}", m.callsign, m.icao_type, m.stand);
    }
    assert!(!parked.is_empty() && waiting > 0);
    for (i, m) in ms.iter().enumerate() {
        assert!(
            m.spawn(i as u32 + 1, &f, &SpawnCtx::default()).is_ok(),
            "{} spawns",
            m.callsign
        );
    }
}

/// LXGB (2026-10-09): a field that publishes no procedures still gets its
/// airline traffic, flown visually: departures with no SID, arrivals with
/// no STAR joining the circuit; and no stand on or against a runway is
/// ever given out.
#[test]
fn a_field_without_procedures_gets_visual_airline_traffic() {
    let f = fixtures::xtst();
    assert!(f.procs.sids.is_empty() && f.procs.approaches.is_empty());
    let ms = generate(&f, Density::Normal, 3, &SpawnCtx::default());
    let (_, dep, arr) = kinds(&ms);
    assert!(dep > 0 && arr > 0, "departures {dep}, arrivals {arr}");
    for m in &ms {
        if m.kind == MovementKind::Departure {
            assert!(m.sid.is_none(), "{m:?}");
        }
        if m.kind == MovementKind::Arrival {
            assert!(m.star.is_none() && !m.straight_in, "{m:?}");
        }
        if let Some(s) = m.stand.as_deref().and_then(|n| f.stand(n)) {
            assert!(!f.beside_runway((s.lat, s.lon)), "{m:?}");
        }
    }
}
