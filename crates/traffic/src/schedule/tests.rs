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
    // Four stands at a towered field: about five airline movements an hour
    // at its peak ([`crate::rate::peak_per_h`]), half that at light.
    assert!(d >= 1 && a >= 1, "light, its own rate: {d} out, {a} in");
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
    // About its peak rate ([`crate::rate::peak_per_h`]), the departures
    // limited by the stands left.
    let peak = crate::rate::peak_per_h(&f);
    assert!(
        a >= 1 && (d + a) as f64 <= peak + 2.0,
        "normal, the field's peak {peak:.1}: {d} out, {a} in"
    );
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
                // A ground start first, where the hour has one (a quiet
                // field's hour at its own rate may send nobody out).
                let ground = ms
                    .iter()
                    .any(|m| matches!(m.kind, MovementKind::Circuit | MovementKind::Departure));
                assert!(
                    !ground || first.kind != MovementKind::Arrival,
                    "a ground start first"
                );
                assert!(ms.iter().all(|m| m.t_s >= first.t_s && m.t_s < HORIZON_S));
                // The departures keep their spacing (0.8 to 1.2 intervals
                // at the field's rate, to the stream's 5 s step).
                let every = 3600.0 / crate::rate::rates_at(&f, density.scale(), None, 0.0).0;
                let deps: Vec<f64> = ms
                    .iter()
                    .filter(|m| m.kind == MovementKind::Departure)
                    .map(|m| m.t_s)
                    .collect();
                assert!(
                    deps.windows(2)
                        .all(|w| (0.8 * every - 5.0..=1.2 * every + 5.0).contains(&(w[1] - w[0]))),
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
                    assert!(flies_its_own(&f.icao, &p.callsign, &p.icao_type), "{p:?}");
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
    let ryr = crate::airlines::carrier_of("LEMG", "RYR1").unwrap();
    let (cs, spoken) = crate::airlines::flight(&ryr, &mut r);
    assert!(
        cs.len() == 7 && spoken.starts_with("Ryanair ") && spoken.split(' ').count() == 5,
        "{cs}: {spoken}"
    );
}

/// The callsign's carrier flies the type, at that field.
fn flies_its_own(icao: &str, callsign: &str, icao_type: &str) -> bool {
    crate::airlines::carrier_of(icao, callsign)
        .is_some_and(|c| c.types.iter().any(|(t, _)| *t == icao_type))
}

/// The type is the airline's: a Ryanair is a 737, a turboprop a regional
/// carrier's (a Ryanair A320 drew a Wizz Air livery at LEMG, 2026-10-08),
/// and no two aircraft of an hour share a callsign.
#[test]
fn every_airline_flies_its_own_type() {
    let f = ulmm();
    for seed in 0..10 {
        let mut ms = generate(&f, Density::Normal, seed, &SpawnCtx::default());
        let parked = park_at_start(&f, Density::Normal, seed, &SpawnCtx::default(), &mut ms);
        let mut seen = std::collections::HashSet::new();
        for m in ms.iter().chain(&parked) {
            assert!(
                seen.insert(m.callsign.clone()),
                "{seed}: {} twice",
                m.callsign
            );
            if m.kind != MovementKind::Circuit
                && class_for_type(&m.icao_type).0.name != PISTON_SINGLE.name
            {
                assert!(
                    flies_its_own(&f.icao, &m.callsign, &m.icao_type),
                    "{seed}: {m:?}"
                );
            }
        }
    }
}

/// Gibraltar's 1,680 m takes the A320s easyJet and BA fly there (the one
/// 1,800 m floor for every jet barred them, and nine of nine were Air
/// Nostrum ATRs); a 1,300 m strip takes no jet, and the A380 only 3,000 m.
#[test]
fn a_type_flies_off_its_own_length_of_runway() {
    for t in ["A320", "A319", "B738"] {
        assert!(runway_needed_m(t) <= 1680.0, "{t}: {}", runway_needed_m(t));
    }
    for t in ["A320", "B738", "E190", "CRJ9"] {
        assert!(runway_needed_m(t) > 1300.0, "{t}");
    }
    assert!(runway_needed_m("AT76") <= 1220.0);
    assert!(runway_needed_m("A388") >= 3000.0 && runway_needed_m("B77W") > runway_needed_m("A321"));
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

/// XNOC with every stand sized `width`.
fn xnoc_sized(width: char) -> FieldModel {
    let mut f = fixtures::xnoc();
    for s in &mut f.net.stands {
        s.width = Some(width);
    }
    f
}

/// The hour sweep (2026-10-09): airliners were scheduled onto light
/// aircraft's stands. A type is flown only where a stand fits it: at a
/// field of code B stands the regional is the Saab, at one of code A
/// stands no airliner comes or goes at all; every ground start's stand
/// fits its type.
#[test]
fn an_airliner_is_never_scheduled_where_no_stand_fits_it() {
    for seed in 0..10 {
        let wide = fixtures::xnoc();
        let ms = generate(&wide, Density::Normal, seed, &SpawnCtx::default());
        assert!(
            ms.iter().any(|m| m.icao_type != SMALL_REGIONAL),
            "{seed}: jets where they fit"
        );
        let b = xnoc_sized('B');
        let ms = generate(&b, Density::Normal, seed, &SpawnCtx::default());
        assert!(
            !ms.is_empty(),
            "{seed}: still traffic at a field of B stands"
        );
        for m in &ms {
            assert_eq!(m.icao_type, SMALL_REGIONAL, "{seed}: {m:?}");
            if let Some(s) = m.stand.as_deref().and_then(|n| b.stand(n)) {
                assert!(
                    b.takes(s, &m.icao_type),
                    "{seed}: {} on {}",
                    m.icao_type,
                    s.name
                );
            }
        }
        let a = xnoc_sized('A');
        let ms = generate(&a, Density::Normal, seed, &SpawnCtx::default());
        assert!(
            ms.iter().all(|m| m.kind == MovementKind::Circuit),
            "{seed}: no airliner at a field of A stands: {ms:?}"
        );
    }
}

/// A parked aircraft is never wider than its stand (ULMM's 1301 rows: code
/// A tie-downs, C and D gates), and while the apron has room none is
/// parked beside a stand the schedule's flights leave from.
#[test]
fn a_parked_aircraft_fits_its_stand_and_leaves_the_movers_room() {
    let f = ulmm();
    assert!(
        f.net.stands.iter().all(|s| s.width.is_some()),
        "ULMM sizes every stand"
    );
    for seed in 0..10 {
        let mut ms = generate(&f, Density::Normal, seed, &SpawnCtx::default());
        let parked = park_at_start(&f, Density::Normal, seed, &SpawnCtx::default(), &mut ms);
        let movers: Vec<LatLon> = ms
            .iter()
            .filter_map(|m| m.stand.as_deref().and_then(|n| f.stand(n)))
            .map(|s| (s.lat, s.lon))
            .collect();
        let longest = f
            .airport
            .runways
            .iter()
            .map(|r| r.length_m())
            .fold(0.0, f64::max);
        let lonely_left = |taken: &[LatLon]| {
            f.net.stands.iter().any(|s| {
                s.is_suitable()
                    && !stand_taken(s, taken)
                    && !neighbour_taken(s, &movers)
                    && f.lane_clearance_m((s.lat, s.lon)) >= PARKED_LANE_CLEAR_M
                    && taken
                        .iter()
                        .all(|p| dist_m(*p, (s.lat, s.lon)) >= PARK_SET_CLEAR_M)
                    && parks_something(&f, s, longest)
            })
        };
        let mut taken = movers.clone();
        for p in &parked {
            let s = f.stand(p.stand.as_deref().unwrap()).unwrap();
            assert!(
                stand_width(s) >= wingspan_code(&p.icao_type),
                "{seed}: {} on {} ({:?})",
                p.icao_type,
                s.name,
                s.width
            );
            if neighbour_taken(s, &movers) {
                assert!(
                    !lonely_left(&taken),
                    "{seed}: {} parked beside a mover",
                    s.name
                );
            }
            taken.push((s.lat, s.lon));
        }
    }
}

/// PABV 2L (the hour sweep, 2026-10-09): two ATRs reached the flare 540 and
/// 890 m down a 1,220 m strip and rolled off its end into the parking. A
/// good landing of each class stops inside the shortest runway the
/// schedule gives it; the long ones that went off the end are long flares
/// that would not have.
#[test]
fn the_runway_minima_hold_a_good_landing_and_not_a_long_one() {
    use crate::aircraft::{flare_along_m, landing_run_m, FLARE_AGL_FT, LONG_FLARE_M};
    for t in [
        "AT76", "SF34", "A320", "B738", "A319", "E190", "B772", "A388",
    ] {
        let p = class_for_type(t).0;
        let good = landing_distance_m(p);
        assert!(good <= runway_needed_m(t), "{t}: {good:.0} m");
    }
    let atr = landing_distance_m(&TURBOPROP);
    assert!(atr < 1220.0, "PABV held a good one, {atr:.0} m");
    assert!(runway_needed_m("AT76") > 1100.0, "not 1,100 m");
    let f = fixtures::xtst_ifr();
    let needed = runway_needed_m("AT76");
    assert!(
        f.end("27").unwrap().length_m >= needed,
        "XTST's 1,500 m takes the ATR"
    );
    for along in [540.0, 890.0] {
        assert!(along > flare_along_m() + LONG_FLARE_M, "a long flare");
        let stop = along + landing_run_m(&TURBOPROP, TURBOPROP.vref_kts, FLARE_AGL_FT);
        assert!(stop > 1220.0, "{along} m down: stops at {stop:.0} m");
    }
}

/// THE FIELD'S OWN RATE THROUGH ITS DAY (2026-10-09): every field flew 12
/// movements an hour, Gatwick and Malaga alike, and 03:00 looked like noon.
/// The rate grows with the airline stands, half at light; the night is
/// nearly empty and has no circuits; the morning sends more out than it
/// brings in and the late evening brings more home.
#[test]
fn the_field_flies_its_own_rate_through_its_own_day() {
    let f = fixtures::xnoc();
    let lon = crate::rate::field_lon(&f);
    let at = |h: f64, d: Density, seed: u64| {
        let ctx = SpawnCtx {
            zulu_s: Some(crate::rate::zulu_for_local(h, lon)),
            ..SpawnCtx::default()
        };
        generate(&f, d, seed, &ctx)
    };
    let count = |h: f64, d: Density| -> (usize, usize, usize) {
        (0..20).fold((0, 0, 0), |acc, seed| {
            let (c, dd, a) = kinds(&at(h, d, seed));
            (acc.0 + c, acc.1 + dd, acc.2 + a)
        })
    };
    // The arrivals the hour's rate owes (the departures also need a free
    // stand each, and this field has few).
    let owed = |h: f64, d: Density| {
        let z = Some(crate::rate::zulu_for_local(h, lon));
        (0..360)
            .map(|k| crate::rate::rates_at(&f, d.scale(), z, k as f64 * 10.0).1)
            .sum::<f64>()
            / 360.0
    };
    let (_, d8, a8) = count(8.0, Density::Normal);
    let (c3, d3, a3) = count(3.0, Density::Normal);
    let (_, d22, a22) = count(22.0, Density::Normal);
    let (_, _, a13) = count(13.0, Density::Normal);
    let (_, _, al) = count(13.0, Density::Light);
    let hourly = |n: usize| n as f64 / 20.0;
    for (what, got, want) in [
        ("13:00", hourly(a13), owed(13.0, Density::Normal)),
        ("13:00 light", hourly(al), owed(13.0, Density::Light)),
        ("22:00", hourly(a22), owed(22.0, Density::Normal)),
    ] {
        assert!(
            (got - want).abs() < 0.6,
            "{what}: {got} arrivals an hour, the rate owes {want:.2}"
        );
    }
    assert!(owed(13.0, Density::Light) < owed(13.0, Density::Normal) * 0.6);
    assert!(d8 > a8, "the morning goes out: {d8} out, {a8} in");
    assert!(a22 > d22, "the evening comes home: {d22} out, {a22} in");
    assert!(hourly(d3 + a3) < 0.5, "03:00 is quiet: {}", hourly(d3 + a3));
    assert_eq!(c3, 0, "no circuits at night");
}

/// THE RUNWAY CAPS IT: however many stands, one runway never gets more
/// than one arrival in [`crate::rate::ARRIVAL_GAP_S`] on average nor two
/// of a kind within [`crate::rate::MIN_GAP_S`].
#[test]
fn no_stream_outruns_the_runway() {
    let ts = crate::rate::stream_times(HORIZON_S, 0.0, 0.5, |_| 500.0, || 1.0);
    assert!(
        ts.len() as f64 <= crate::rate::arrivals_cap_per_h().max(3600.0 / crate::rate::MIN_GAP_S)
    );
    let f = fixtures::xnoc();
    let (d, a) = crate::rate::rates_at(&f, 100.0, None, 0.0);
    assert_eq!(d, crate::rate::departures_cap_per_h());
    assert_eq!(a, crate::rate::arrivals_cap_per_h());
}

/// A widebody never stands on a stand drawn over another (apt.dat's
/// overlapping sets: a 777 and an A320 parked 21 m apart at EGKK, the hour
/// sweep 2026-10-10), and a widebody stands further from the lanes than a
/// narrowbody.
#[test]
fn a_widebody_needs_a_stand_of_its_own() {
    let mut f = xnoc_sized('E');
    let a = f.net.stands[0].clone();
    assert!(f.room_for(&a, "A320"));
    let mut over = a.clone();
    over.name = "over".into();
    over.lon += 0.0002; // some 14 m east
    f.net.stands.push(over);
    let a = f.stand(&a.name).unwrap().clone();
    assert!(f.room_for(&a, "A320"), "a narrowbody shares a set");
    assert!(!f.room_for(&a, "B772") && !f.takes(&a, "B772"));
    assert!(parked_lane_clear_m("B77W") > parked_lane_clear_m("A320"));
    assert_eq!(parked_lane_clear_m("A320"), PARKED_LANE_CLEAR_M);
}

/// A turnaround is the type's: a narrowbody 35 to 60 minutes, a turboprop
/// 25 to 40, a widebody 90 to 150, the A380 two to three hours.
#[test]
fn a_turnaround_takes_the_types_own_time() {
    let mut r = XorShift64::new(2);
    for _ in 0..200 {
        let m = |t: &str, r: &mut XorShift64| turnaround_s(t, r) / 60.0;
        assert!((35.0..=60.0).contains(&m("A320", &mut r)));
        assert!((25.0..=40.0).contains(&m("AT76", &mut r)));
        assert!((90.0..=150.0).contains(&m("B772", &mut r)));
        assert!((120.0..=180.0).contains(&m("A388", &mut r)));
    }
    let (cs, spoken) = next_flight("EGKK", "EZY8123", &mut r).unwrap();
    assert!(cs.starts_with("EZY8") && cs != "EZY8123" && spoken.starts_with("Easy "));
    assert_eq!(
        next_flight("EGKK", "GABCD", &mut r),
        None,
        "a registration stays"
    );
}

/// No two aircraft stand on stands of one overlapping set at the start
/// (EGKK, the hour sweep 2026-10-10: departures left through a jet parked
/// 21 m away on the next stand of the set): every parked aircraft is at
/// least [`PARK_SET_CLEAR_M`] from every other and from every stand the
/// hour's flights stand on.
#[test]
fn nobody_parks_on_a_stand_of_an_occupied_set() {
    let f = ulmm();
    for seed in 0..10 {
        let mut ms = generate(&f, Density::Normal, seed, &SpawnCtx::default());
        let parked = park_at_start(&f, Density::Normal, seed, &SpawnCtx::default(), &mut ms);
        let at = |m: &Movement| {
            m.stand
                .as_deref()
                .and_then(|n| f.stand(n))
                .map(|s| (s.lat, s.lon))
        };
        let movers: Vec<LatLon> = ms.iter().filter_map(at).collect();
        let mine: Vec<LatLon> = parked.iter().filter_map(at).collect();
        assert!(!mine.is_empty() || seed > 0, "ULMM parks something");
        for (k, p) in mine.iter().enumerate() {
            for q in movers.iter().chain(&mine[..k]) {
                let d = dist_m(*p, *q);
                assert!(d >= PARK_SET_CLEAR_M, "{seed}: {d:.0} m");
            }
        }
    }
}
