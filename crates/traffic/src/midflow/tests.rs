use super::*;
use crate::airlines::EZY;
use crate::field::fixtures;
use crate::routes::RouteRow;

/// ULMM's five jet stands and its 2,500 m runway, with a timetable dense
/// around ten in the morning: a rotation every few minutes either side.
fn ulmm() -> FieldModel {
    FieldModel::load(&fixtures::path("ulmm.dat"), "ULMM")
        .unwrap()
        .expect("ULMM in the fixture")
}

const BUSY: &[RouteRow] = &[RouteRow {
    carrier: EZY,
    number: 1001,
    other: "EGKK",
    city: "London Gatwick",
    summer: 49,
    winter: 49,
    windows: &[(9.6, 10.4)],
    based: false,
    types: &[("A320", 100)],
}];

const BUSIER: &[RouteRow] = &[
    BUSY[0],
    RouteRow {
        number: 2001,
        other: "EGCC",
        city: "Manchester",
        windows: &[(9.4, 10.5)],
        ..BUSY[0]
    },
    RouteRow {
        number: 3001,
        other: "EGPH",
        city: "Edinburgh",
        windows: &[(9.75, 10.25)],
        ..BUSY[0]
    },
];

fn table(rows: &'static [RouteRow]) -> FieldRoutes {
    FieldRoutes {
        icao: "ULMM",
        utc_offset_h: 3.0,
        rows,
    }
}

const TEN: f64 = 10.0 * 3600.0;

fn start_at(rows: &'static [RouteRow], seed: u64) -> (FieldModel, Plan0) {
    let f = ulmm();
    let t = table(rows);
    let ctx = SpawnCtx {
        runway: Some("31".into()),
        ..SpawnCtx::default()
    };
    let p = plan(
        &f,
        &t,
        Density::Normal,
        seed,
        Date::ymd(2026, 7, 14),
        TEN,
        6.0 * 3600.0,
        &ctx,
    );
    (f, p)
}

/// Every flight is put where its rotation is at 10:00: the arrivals due
/// in the next minutes on the approach, nearest no closer than five miles
/// and each a wake spacing and a runway's time behind the one ahead; the
/// later ones at their entries later on; nothing on the runway, one
/// holding short at most, one climbing out at most.
#[test]
fn a_start_at_ten_places_each_flight_by_its_rotation() {
    let mut on_approach = 0;
    for seed in 1..6 {
        let (f, p) = start_at(BUSIER, seed);
        let mut approach: Vec<f64> = p
            .movements
            .iter()
            .filter_map(|m| match m.start {
                Some(Start::Approach { to_thr_m }) => Some(to_thr_m),
                _ => None,
            })
            .collect();
        on_approach += approach.len();
        approach.sort_by(f64::total_cmp);
        assert!(approach
            .first()
            .is_none_or(|d| *d >= MIN_FINAL_NM * NM - 1.0));
        for w in approach.windows(2) {
            assert!(w[1] - w[0] >= FINAL_GAP_NM * NM - 1.0, "{approach:?}");
        }
        assert!(approach.iter().all(|d| *d <= APPROACH_LEAD_NM * NM));
        let count = |pred: &dyn Fn(&Start) -> bool| {
            p.movements
                .iter()
                .filter(|m| m.start.as_ref().is_some_and(pred))
                .count()
        };
        assert!(count(&|s| matches!(s, Start::HoldShort)) <= 1);
        assert!(count(&|s| matches!(s, Start::Climb { .. })) <= 1);
        // The placed ones place, nobody on the runway, nobody inside
        // anybody on the ground.
        let mut occupied: Vec<LatLon> = Vec::new();
        let mut placed = 0;
        for (i, m) in p.movements.iter().enumerate() {
            // Gone before the start: the whole-flight model's, never placed.
            if (m.start.is_none() && !m.parked_from_start)
                || matches!(m.start, Some(Start::Away { .. }))
            {
                continue;
            }
            let ctx = SpawnCtx {
                occupied: occupied.clone(),
                ..SpawnCtx::default()
            };
            let ac = if m.start.is_some() {
                m.spawn(i as u32 + 1, &f, &ctx)
            } else {
                m.park(i as u32 + 1, &f, &ctx)
            };
            let Ok(ac) = ac else {
                panic!("{} {:?} refused: {ac:?}", m.callsign, m.start);
            };
            placed += 1;
            if ac.kin.on_ground {
                assert!(!f.on_runway(ac.kin.pos()), "{} on the runway", m.callsign);
                occupied.push(ac.kin.pos());
            }
            let want = match &m.start {
                Some(Start::Approach { .. }) => Phase::Arrive,
                Some(Start::TaxiIn { .. }) => Phase::TaxiIn,
                Some(Start::TaxiOut { .. }) => Phase::TaxiOut,
                Some(Start::HoldShort) => Phase::HoldShort,
                Some(Start::Climb { .. }) | Some(Start::Away { .. }) => ac.phase,
                None => Phase::Parked,
            };
            assert_eq!(ac.phase, want, "{}", m.callsign);
        }
        assert!(placed > 0);
        // The board has both halves of every rotation.
        let deps = p.board.iter().filter(|b| b.dir == Dir::Departure).count();
        let arrs = p.board.iter().filter(|b| b.dir == Dir::Arrival).count();
        assert_eq!(deps, arrs);
    }
    assert!(
        on_approach >= 3,
        "{on_approach} on the approach over five starts"
    );
}

/// An arrival placed on its approach is on its route at that distance from
/// the threshold, on the 3 degree path, at the radar unit's speed for
/// there; inside the tower's call distance it calls the tower, further out
/// it checks in with approach first.
#[test]
fn an_arrival_on_the_approach_is_on_its_path() {
    let f = ulmm();
    let (_, p) = start_at(BUSY, 3);
    let m = p
        .movements
        .iter()
        .find(|m| matches!(m.start, Some(Start::Approach { .. })))
        .expect("an arrival on the approach");
    for d_nm in [6.0, 15.0] {
        let mut m = m.clone();
        m.start = Some(Start::Approach {
            to_thr_m: d_nm * NM,
        });
        let ac = m.spawn(1, &f, &SpawnCtx::default()).unwrap();
        let thr = ac.route.thr_idx.unwrap();
        let along = ac.route.dist_to(ac.kin.pos(), thr) / NM;
        assert!((along - d_nm).abs() < 0.05, "{along}");
        let end = f.end("31").unwrap();
        let glide = end.elev_ft + TCH_FT + glide_ft(d_nm * NM);
        assert!((ac.kin.alt_ft - glide).abs() < 1.0);
        assert!(!ac.kin.on_ground && ac.kin.gs_kts > 120.0);
        let tower = d_nm <= crate::arrival::TOWER_CALL_NM;
        assert_eq!(ac.ifr_radio == IfrRadio::WithTower, tower, "{d_nm}");
    }
}

/// The same field, seed, date and moment start the same.
#[test]
fn the_start_is_deterministic() {
    let (_, a) = start_at(BUSIER, 4);
    let (_, b) = start_at(BUSIER, 4);
    assert_eq!(a.movements, b.movements);
    assert_eq!(a.board, b.board);
}

/// A departure whose arrival is still to come is flown by that arrival's
/// airframe: it names it as its inbound, and leaves at least the type's
/// minimum turnaround after it is on the blocks.
#[test]
fn a_departure_names_the_airframe_it_waits_for() {
    let (_, p) = start_at(BUSIER, 2);
    let arrivals: Vec<&Movement> = p
        .movements
        .iter()
        .filter(|m| m.kind == MovementKind::Arrival)
        .collect();
    let mut paired = 0;
    for d in p
        .movements
        .iter()
        .filter(|m| m.kind == MovementKind::Departure)
    {
        // With none it stands on its stand already, or (no stand free at
        // the start) appears on one at its time.
        let Some(inbound) = d.flight.as_ref().and_then(|f| f.inbound.clone()) else {
            continue;
        };
        let a = arrivals
            .iter()
            .find(|a| a.callsign == inbound)
            .unwrap_or_else(|| panic!("{} waits for {inbound}, not flying", d.callsign));
        let n_in: u32 = a.callsign[3..].parse().unwrap();
        let n_out: u32 = d.callsign[3..].parse().unwrap();
        assert_eq!(n_out, n_in + 1);
        assert!(d.t_s > a.t_s);
        paired += 1;
    }
    assert!(paired > 0);
}

const FEW: &[RouteRow] = &[RouteRow {
    summer: 21,
    winter: 21,
    windows: &[(9.0, 11.0)],
    ..BUSY[0]
}];

/// Every flight on the board has the stand the field planned for it from
/// the start, both halves of a rotation the same one, and a departure that
/// stands on its stand from the start stands on its planned one where that
/// was free: the pilot's board at LEMG (2026-10-13) had departures
/// Boarding with an empty Stand column.
#[test]
fn every_flight_on_the_board_has_its_planned_stand() {
    for seed in 1..4 {
        let (f, p) = start_at(FEW, seed);
        assert!(!p.board.is_empty());
        for b in &p.board {
            let s = b.info.stand.as_deref().expect("a planned stand");
            assert!(f.stand(s).is_some(), "{s}");
        }
        for b in p.board.iter().filter(|b| b.dir == Dir::Departure) {
            let inbound = b.info.inbound.as_deref().unwrap();
            let arr = p.board.iter().find(|a| a.callsign == inbound).unwrap();
            assert_eq!(arr.info.stand, b.info.stand, "{}", b.callsign);
        }
        for m in p.movements.iter().filter(|m| m.parked_from_start) {
            assert_eq!(
                m.stand,
                m.flight.as_ref().and_then(|f| f.stand.clone()),
                "{}",
                m.callsign
            );
        }
        // A departure still to appear starts on its planned stand.
        for m in p
            .movements
            .iter()
            .filter(|m| m.kind == MovementKind::Departure && m.start.is_none())
        {
            assert!(m.stand.is_some(), "{}", m.callsign);
        }
    }
}

/// A world built in the evening (LXGB, 2026-10-13: the traffic rebuilt at
/// 17:48 local) holds the day's past as history. Gibraltar's own table on
/// ULMM's apron, 18:22 local on that Tuesday, its 24 h of timetable:
/// nothing parked for a departure whose inbound is still to come (it parked
/// tomorrow's 11:05 on stand 4 seventeen hours early); no departure put on
/// the ground past the moment its own taxi would have flown it, the rest
/// the model's from their wheels up after their off-block; and the board by
/// the clock reads every past flight Departed or Landed, keeps it a while,
/// and shows nothing of tomorrow it could be read as today's.
#[test]
fn an_evening_start_holds_the_days_past_as_history() {
    use crate::board::{payload, Row, Status};
    let f = ulmm();
    let t = crate::routes::field_routes("LXGB").expect("Gibraltar's table");
    let now = 18.0 * 3600.0 + 22.0 * 60.0;
    let ctx = SpawnCtx {
        runway: Some("31".into()),
        ..SpawnCtx::default()
    };
    let end = f.end("31").unwrap();
    let mut away = 0;
    for seed in 1..6 {
        let p = plan(
            &f,
            t,
            Density::Normal,
            seed,
            Date::ymd(2026, 10, 13),
            now,
            24.0 * 3600.0,
            &ctx,
        );
        let mut schedule = p.movements.clone();
        let _ = crate::schedule::park_at_start(&f, Density::Light, seed, &ctx, &mut schedule);
        for m in schedule.iter().filter(|m| m.parked_from_start) {
            let info = m.flight.as_ref().unwrap();
            assert!(
                info.inbound.is_none(),
                "{} waits for its inbound",
                m.callsign
            );
            assert!(
                m.t_s < 12.0 * 3600.0,
                "{} parked {} s early",
                m.callsign,
                m.t_s
            );
        }
        for m in schedule
            .iter()
            .filter(|m| m.kind == MovementKind::Departure)
        {
            let est = m.flight.as_ref().and_then(|i| i.est_s).unwrap();
            let gone = now - est;
            let way = m
                .stand
                .as_deref()
                .and_then(|n| f.stand(n))
                .and_then(|s| taxi_out_way(&f, &end, s));
            let (_, up, climb_end) =
                out_times(way.as_ref().map(|(w, _)| w.as_slice()), &m.icao_type);
            match &m.start {
                Some(Start::Away { off_s, up_s }) => {
                    away += 1;
                    assert!(*off_s < 0.0 && up_s > off_s, "{}", m.callsign);
                    assert!(m.stand.is_none(), "{} gone, on no stand", m.callsign);
                }
                Some(Start::Climb { .. }) => assert!(gone <= climb_end, "{}", m.callsign),
                Some(_) => assert!(gone < up, "{} on the ground past its wheels up", m.callsign),
                None => assert!(
                    gone < up,
                    "{} spawns {gone:.0} s past its off-block",
                    m.callsign
                ),
            }
        }
        let rows: Vec<Row> = p
            .board
            .iter()
            .map(|b| Row {
                flight: b.clone(),
                status: Status::by_clock(b, now),
                time_s: b.info.est_s,
                stand: None,
            })
            .collect();
        let text = payload("LXGB", now, &rows, &[]);
        let mins = |hhmm: &str| {
            let (h, m) = hhmm.split_once(':').unwrap();
            h.parse::<i64>().unwrap() * 60 + m.parse::<i64>().unwrap()
        };
        let now_min = (now / 60.0) as i64;
        for l in text.lines().skip(1) {
            let c: Vec<&str> = l.split('|').collect();
            let at = mins(c[2]);
            match c[8] {
                "Departed" => assert!((now_min - at).rem_euclid(1440) <= 30, "{l}"),
                "Landed" => assert!((now_min - at).rem_euclid(1440) <= 60, "{l}"),
                _ => assert!((at - now_min).rem_euclid(1440) <= 12 * 60, "{l}"),
            }
        }
    }
    assert!(away > 0, "the day's departures flown by the model");
}

/// A departure's moments are its own way's: a short taxi is off the
/// ground minutes before the old flat clocks said, a long one capped by
/// them.
#[test]
fn a_departure_is_off_the_ground_when_its_own_taxi_says() {
    let short = [(36.15, -5.35), (36.15, -5.343)];
    let (taxi, up, climb) = out_times(Some(&short), "A320");
    assert!(
        taxi < 5.0 * 60.0 && up < 8.0 * 60.0 && climb < 10.0 * 60.0,
        "{taxi} {up} {climb}"
    );
    let long = [(36.15, -5.35), (36.15, -5.25)];
    assert_eq!(out_times(Some(&long), "A320"), out_times(None, "A320"));
    assert_eq!(out_times(None, "A320").2, CLIMB_S);
}
