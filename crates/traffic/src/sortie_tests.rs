//! Traffic "sorties": whole flights through the phase machine against a
//! scripted tower (no ATC), scored like our own: cross-track on the taxi
//! and the final, the touchdown point, every phase in order, and the same
//! trace twice.

use std::cell::RefCell;
use std::collections::HashMap;

use crate::field::{fixtures, FieldModel};
use crate::kin::{cross_track_m, dist_m, LatLon};
use crate::phase::{Call, Event, Leg, Order, Phase, Plan, TowerState};
use crate::{Fleet, TrafficAircraft};

/// A tower that answers every call the obvious way, as data.
#[derive(Default)]
struct ScriptedTower {
    orders: HashMap<u32, Order>,
    /// Withhold the landing clearance for this many finals.
    refuse_landings: u32,
    /// Answer "ready" with line up and wait first.
    line_up_and_wait: bool,
    follow: HashMap<u32, u32>,
}

impl ScriptedTower {
    fn hear(&mut self, ev: &Event) {
        let Event::Call { id, call } = ev else { return };
        let o = self.orders.entry(*id).or_default();
        o.follow = self.follow.get(id).copied();
        match call {
            Call::Taxi | Call::Vacated => o.state = TowerState::TaxiCleared,
            Call::Ready => {
                o.state = if self.line_up_and_wait && o.state != TowerState::LinedUp {
                    TowerState::LinedUp
                } else {
                    TowerState::TakeoffCleared
                }
            }
            Call::Inbound { .. } | Call::Downwind => o.state = TowerState::Inbound,
            Call::Final => {
                if self.refuse_landings > 0 {
                    self.refuse_landings -= 1;
                } else {
                    o.state = TowerState::ClearedToLand;
                }
            }
            Call::GoingAround => {
                o.state = TowerState::Inbound;
                o.go_around = false;
            }
            Call::RequestCrossing { runway } => o.cross_cleared = Some(runway.clone()),
            Call::Base | Call::Readback(_) | Call::SayAgain => {}
        }
    }
}

/// What a sortie measured.
#[derive(Debug, Default)]
struct Score {
    phases: Vec<Phase>,
    calls: Vec<Call>,
    taxi_xt_max_m: f64,
    final_xt_max_m: f64,
    touchdowns: Vec<f64>,
    runway_len_m: f64,
    trace: Vec<u64>,
    gone: bool,
    secs: f64,
    /// Substeps flown on the ground or on a final or landing with the gear up.
    gear_up_low: u32,
}

/// Fly `ac` at `field` until it is gone or `max_s` passes. `tower` may
/// line up and wait once on the first "ready".
fn fly(
    field: FieldModel,
    acs: Vec<TrafficAircraft>,
    tower: ScriptedTower,
    max_s: f64,
    watch: u32,
) -> Score {
    let rwy = acs[0].plan.runway().unwrap().to_string();
    let end = field.end(&rwy).unwrap();
    let first = acs.iter().find(|a| a.id == watch).unwrap().phase;
    let mut fleet = Fleet::new(field);
    for ac in acs {
        assert!(fleet.spawn(ac));
    }
    let tower = RefCell::new(tower);
    let mut s = Score {
        runway_len_m: end.length_m,
        phases: vec![first],
        ..Default::default()
    };
    let mut t = 0.0;
    while t < max_s {
        let events = {
            let tw = &tower;
            fleet.step(0.1, None, &|id| tw.borrow().orders.get(&id).cloned())
        };
        t += 0.1;
        for ev in &events {
            tower.borrow_mut().hear(ev);
            match ev {
                Event::Phase { id, to, .. } if *id == watch => s.phases.push(*to),
                Event::Call { id, call } if *id == watch => s.calls.push(call.clone()),
                Event::Touchdown { id, along_m, .. } if *id == watch => s.touchdowns.push(*along_m),
                _ => {}
            }
        }
        let Some(ac) = fleet.get(watch) else {
            s.gone = true;
            break;
        };
        let k = &ac.kin;
        if (k.on_ground || matches!(ac.phase, Phase::Final | Phase::Landing)) && k.gear < 1.0 {
            s.gear_up_low += 1;
        }
        s.trace.extend([
            k.lat.to_bits(),
            k.lon.to_bits(),
            k.alt_ft.to_bits(),
            k.hdg_deg.to_bits(),
        ]);
        match ac.phase {
            Phase::TaxiOut | Phase::TaxiIn | Phase::Vacate => {
                s.taxi_xt_max_m = s.taxi_xt_max_m.max(cross_track_m(&ac.route.path, k.pos()));
            }
            Phase::Final | Phase::Landing if !k.on_ground && k.alt_ft - end.elev_ft < 300.0 => {
                s.final_xt_max_m = s.final_xt_max_m.max(end.frame(k.pos()).1.abs());
            }
            _ => {}
        }
    }
    s.secs = t;
    s
}

fn cessna(field: &FieldModel, id: u32, stand: &str, runway: &str, laps: u32) -> TrafficAircraft {
    let left = field.end(runway).unwrap().left;
    TrafficAircraft::at_stand(
        id,
        &format!("N{id}SP"),
        "Cessna three four five",
        "C172",
        Plan::Circuit {
            runway: runway.into(),
            laps,
            left,
        },
        field,
        stand,
        id as u64,
    )
    .expect("the stand")
}

const CIRCUIT_PHASES: [Phase; 15] = [
    Phase::Parked,
    Phase::TaxiOut,
    Phase::HoldShort,
    Phase::LineUp,
    Phase::Roll,
    Phase::Climb,
    Phase::Pattern {
        leg: Leg::Crosswind,
    },
    Phase::Pattern { leg: Leg::Downwind },
    Phase::Pattern { leg: Leg::Base },
    Phase::Final,
    Phase::Landing,
    Phase::Vacate,
    Phase::TaxiIn,
    Phase::Parked,
    Phase::Gone,
];

fn assert_flown(s: &Score, what: &str) {
    println!(
        "{what}: {:.0} s, taxi cross-track max {:.2} m, final (below 300 ft) max {:.2} m, \
         touchdown {:?} m into {:.0} m",
        s.secs, s.taxi_xt_max_m, s.final_xt_max_m, s.touchdowns, s.runway_len_m
    );
    assert!(s.gone, "{what}: never finished ({:?})", s.phases);
    assert!(
        s.taxi_xt_max_m < 4.0,
        "{what}: taxi cross-track {:.2} m",
        s.taxi_xt_max_m
    );
    assert!(
        s.final_xt_max_m < 5.0,
        "{what}: final cross-track {:.2} m",
        s.final_xt_max_m
    );
    for td in &s.touchdowns {
        assert!(
            *td > 0.0 && *td < s.runway_len_m / 3.0,
            "{what}: touchdown {td:.0} m into {:.0} m",
            s.runway_len_m
        );
    }
}

/// A taxi route across a runway stops short of its pavement (the strip's
/// geometry, never a network flag), asks to cross, holds (asking again
/// every [`CROSS_ASK_RETRY_S`]) and goes on only once the tower has
/// cleared THAT strip and heard the readback. The world sweep (ULMM, EFKU,
/// UEMH, 2026-10-08): traffic drove straight across under its taxi
/// clearance.
#[test]
fn a_taxi_holds_short_of_a_runway_it_crosses_until_cleared() {
    use crate::aircraft::CROSS_ASK_RETRY_S;
    let field = fixtures::xtst();
    let e09 = field.end("09").unwrap();
    let mut ac = cessna(&field, 1, "GA2", "09", 1);
    // From the stand south across 09/27 to beyond it.
    let route = vec![(36.0022, -5.0), (36.0022, -5.004), (35.9985, -5.004)];
    let mut o = Order {
        state: TowerState::TaxiCleared,
        taxi_path: Some(route.clone()),
        ..Default::default()
    };
    let mut events = Vec::new();
    let run = |ac: &mut TrafficAircraft, o: &Order, secs: f64, events: &mut Vec<Event>| {
        let mut nearest = f64::INFINITY;
        for _ in 0..(secs * 10.0) as usize {
            ac.substep(0.1, Some(o), &field, &[], events);
            nearest = nearest.min(e09.frame(ac.kin.pos()).1.abs());
        }
        nearest
    };
    let asks = |events: &[Event]| {
        events
            .iter()
            .filter(|e| {
                matches!(e, Event::Call { call: Call::RequestCrossing { runway }, .. }
                    if runway == "09")
            })
            .count()
    };
    // Uncleared for four minutes: stopped short, asking every half minute.
    let nearest = run(&mut ac, &o, 240.0, &mut events);
    assert_eq!(ac.phase, Phase::TaxiOut);
    assert!(ac.kin.gs_kts < 0.5, "held, {:.1} kt", ac.kin.gs_kts);
    assert!(
        nearest > 40.0,
        "never nearer the centreline than {nearest:.1} m"
    );
    let n = asks(&events);
    assert!(
        (2..=(240.0 / CROSS_ASK_RETRY_S) as usize).contains(&n),
        "{n} asks"
    );
    // Cleared across the strip (by its other end), the readback still owed:
    // it holds; heard, it crosses and reaches the end of its route.
    o.cross_cleared = Some("27".into());
    o.readback_owed = Some("Crossing runway two seven, Cessna three four five.".into());
    let nearest = run(&mut ac, &o, 10.0, &mut events);
    assert!(nearest > 40.0, "no move before the readback");
    o.readback_owed = None;
    run(&mut ac, &o, 120.0, &mut events);
    assert_eq!(ac.phase, Phase::HoldShort, "across and at the route's end");
    assert!(e09.frame(ac.kin.pos()).1 > 40.0, "south of the runway now");
    // A crossing cleared for ANOTHER strip is no clearance for this one:
    // a fresh aircraft with the same order but a foreign runway holds.
    let mut other = cessna(&field, 2, "GA2", "09", 1);
    let foreign = Order {
        cross_cleared: Some("18".into()),
        ..o.clone()
    };
    let nearest = run(&mut other, &foreign, 120.0, &mut Vec::new());
    assert!(nearest > 40.0 && other.phase == Phase::TaxiOut);
}

#[test]
fn a_172_flies_a_circuit_stand_to_stand() {
    let field = fixtures::xtst();
    let ac = cessna(&field, 1, "GA2", "09", 1);
    let s = fly(field, vec![ac], ScriptedTower::default(), 3600.0, 1);
    assert_flown(&s, "XTST 09");
    assert_eq!(s.phases, CIRCUIT_PHASES, "every phase, none skipped");
    assert_eq!(s.touchdowns.len(), 1);
    assert_eq!(
        s.calls,
        vec![
            Call::Taxi,
            Call::Ready,
            Call::Downwind,
            Call::Base,
            Call::Final,
            Call::Vacated
        ]
    );
}

#[test]
fn a_right_hand_circuit_flies_its_published_side() {
    let field = fixtures::xtst();
    let end = field.end("27").unwrap();
    assert!(!end.left);
    let mut ac = cessna(&field, 2, "GA1", "27", 1);
    ac.seed = 99;
    let mut fleet = Fleet::new(field);
    fleet.spawn(ac);
    let tower = RefCell::new(ScriptedTower::default());
    let mut downwind_sides = Vec::new();
    for _ in 0..30_000 {
        let events = fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned());
        events.iter().for_each(|e| tower.borrow_mut().hear(e));
        let Some(a) = fleet.get(2) else { break };
        if a.phase == (Phase::Pattern { leg: Leg::Downwind }) {
            downwind_sides.push(end.frame(a.kin.pos()).1);
        }
    }
    assert!(!downwind_sides.is_empty());
    assert!(
        downwind_sides.iter().all(|r| *r > 500.0),
        "downwind to the right of 27"
    );
}

#[test]
fn the_same_seed_flies_the_same_sortie() {
    let a = fly(
        fixtures::xtst(),
        vec![cessna(&fixtures::xtst(), 1, "GA2", "09", 1)],
        ScriptedTower::default(),
        3600.0,
        1,
    );
    let b = fly(
        fixtures::xtst(),
        vec![cessna(&fixtures::xtst(), 1, "GA2", "09", 1)],
        ScriptedTower::default(),
        3600.0,
        1,
    );
    assert_eq!(a.trace, b.trace, "bit-identical");
    assert!(!a.trace.is_empty());
}

#[test]
fn touch_and_goes_fly_every_lap_then_park() {
    let field = fixtures::xtst();
    let ac = cessna(&field, 3, "GA3", "09", 3);
    let s = fly(field, vec![ac], ScriptedTower::default(), 7200.0, 3);
    assert_flown(&s, "XTST 09 three laps");
    assert_eq!(s.touchdowns.len(), 3, "{:?}", s.phases);
    assert_eq!(s.phases.iter().filter(|p| **p == Phase::Roll).count(), 3);
}

#[test]
fn no_landing_clearance_by_300_ft_is_a_go_around() {
    let field = fixtures::xtst();
    let ac = cessna(&field, 4, "GA2", "09", 1);
    let tower = ScriptedTower {
        refuse_landings: 1,
        ..Default::default()
    };
    let s = fly(field, vec![ac], tower, 5400.0, 4);
    assert_flown(&s, "XTST 09 go-around");
    assert!(s.calls.contains(&Call::GoingAround), "{:?}", s.calls);
    let ga = s
        .phases
        .iter()
        .position(|p| *p == Phase::GoAround)
        .expect("a go-around");
    assert_eq!(s.phases[ga - 1], Phase::Final);
    assert_eq!(
        s.phases[ga + 1],
        Phase::Pattern {
            leg: Leg::Crosswind
        }
    );
    assert_eq!(s.touchdowns.len(), 1, "then lands");
}

#[test]
fn a_tower_go_around_is_flown_at_once() {
    let field = fixtures::xtst();
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(cessna(&field, 5, "GA2", "09", 1));
    let tower = RefCell::new(ScriptedTower::default());
    let mut ordered = None;
    let mut went = None;
    for _ in 0..40_000 {
        let events = fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned());
        for e in &events {
            tower.borrow_mut().hear(e);
            if let Event::Phase {
                to: Phase::GoAround,
                ..
            } = e
            {
                went.get_or_insert(fleet.get(5).unwrap().agl_ft(&field));
            }
        }
        let Some(a) = fleet.get(5) else { break };
        if a.phase == Phase::Final && a.agl_ft(&field) < 400.0 && ordered.is_none() {
            ordered = Some(a.agl_ft(&field));
            tower.borrow_mut().orders.get_mut(&5).unwrap().go_around = true;
        }
    }
    let (at, agl) = (ordered.expect("ordered"), went.expect("went around"));
    assert!(
        (at - agl).abs() < 10.0,
        "at once: ordered at {at:.0} ft, went at {agl:.0} ft"
    );
}

#[test]
fn line_up_and_wait_stops_aligned_on_the_centreline() {
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let mut fleet = Fleet::new(field);
    fleet.spawn(cessna(&fleet.field.clone(), 6, "GA2", "09", 1));
    let tower = RefCell::new(ScriptedTower {
        line_up_and_wait: true,
        ..Default::default()
    });
    let mut lined_up = None;
    for _ in 0..20_000 {
        let events = fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned());
        events.iter().for_each(|e| tower.borrow_mut().hear(e));
        let a = fleet.get(6).unwrap();
        if a.phase == Phase::LineUp && a.route.follow.done && a.kin.gs_kts == 0.0 {
            lined_up = Some(a.kin.clone());
            break;
        }
    }
    let k = lined_up.expect("lined up and waiting");
    let (_, right) = end.frame(k.pos());
    assert!(right.abs() < 1.5, "on the centreline, {right:.2} m off");
    let off = crate::kin::signed_diff_deg(k.hdg_deg, end.course_deg).abs();
    assert!(off < 5.0, "aligned, {off:.1} degrees off");
}

#[test]
fn number_two_extends_its_downwind_behind_its_leader() {
    let field = fixtures::xtst();
    let lead = cessna(&field, 10, "GA1", "09", 1);
    let two = cessna(&field, 11, "GA3", "09", 1);
    let mut tower = ScriptedTower::default();
    tower.follow.insert(11, 10);
    let end = field.end("09").unwrap();
    let mut fleet = Fleet::new(field);
    fleet.spawn(lead);
    fleet.spawn(two);
    let tower = RefCell::new(tower);
    let mut checked = false;
    let mut lead_landed = false;
    for _ in 0..60_000 {
        let events = fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned());
        for e in &events {
            tower.borrow_mut().hear(e);
            if let Event::Touchdown { id: 10, .. } = e {
                lead_landed = true;
            }
            // When number two turns base, the leader is on short final or down.
            if let Event::Phase {
                id: 11,
                to: Phase::Pattern { leg: Leg::Base },
                ..
            } = e
            {
                checked = true;
                if let Some(l) = fleet.get(10) {
                    let close = dist_m(l.kin.pos(), end.thr) <= 1.5 * 1852.0 + 1.0;
                    assert!(
                        l.kin.on_ground || (l.phase == Phase::Final && close) || lead_landed,
                        "leader {:?} {:.0} m out",
                        l.phase,
                        dist_m(l.kin.pos(), end.thr)
                    );
                }
            }
        }
        if fleet.aircraft().is_empty() {
            break;
        }
    }
    assert!(checked, "number two flew a base");
    assert!(fleet.aircraft().is_empty(), "both finished");
}

#[test]
fn a_departure_leaves_on_its_bearing_and_is_gone_at_twelve_miles() {
    let field = fixtures::xtst();
    let ac = TrafficAircraft::at_stand(
        20,
        "N20",
        "Cessna two zero",
        "C172",
        Plan::Departure {
            runway: "27".into(),
            sid: None,
            exit_bearing: 200.0,
        },
        &field,
        "GA4",
        20,
    )
    .unwrap();
    let s = fly(field, vec![ac], ScriptedTower::default(), 3600.0, 20);
    assert!(s.gone, "{:?}", s.phases);
    assert_eq!(
        s.phases,
        [
            Phase::Parked,
            Phase::TaxiOut,
            Phase::HoldShort,
            Phase::LineUp,
            Phase::Roll,
            Phase::Climb,
            Phase::Leave,
            Phase::Gone
        ]
    );
    assert_eq!(s.calls, vec![Call::Taxi, Call::Ready]);
}

fn arrival(field: &FieldModel, id: u32, icao_type: &str, straight_in: bool) -> TrafficAircraft {
    let end = field.end("27").unwrap();
    // Ten miles out, off to the south east of the field.
    let entry = crate::kin::project_m(end.thr, 120.0, 10.0 * 1852.0);
    TrafficAircraft::at_entry(
        id,
        &format!("T{id}"),
        "Traffic",
        icao_type,
        Plan::Arrival {
            entry,
            star: None,
            runway: "27".into(),
            straight_in,
        },
        field,
        id as u64,
    )
    .unwrap()
}

#[test]
fn a_straight_in_arrival_lands_and_parks() {
    let field = fixtures::xtst();
    let ac = arrival(&field, 30, "C172", true);
    let s = fly(field, vec![ac], ScriptedTower::default(), 3600.0, 30);
    assert_flown(&s, "XTST 27 straight in, 172");
    assert_eq!(
        s.phases,
        [
            Phase::Arrive,
            Phase::Final,
            Phase::Landing,
            Phase::Vacate,
            Phase::TaxiIn,
            Phase::Parked,
            Phase::Gone
        ]
    );
    assert!(
        matches!(s.calls[0], Call::Inbound { miles: 10..=11, .. }),
        "{:?}",
        s.calls
    );
}

/// A retractable straight-in arrival has its gear down from the final to
/// the stand (LEMG, 2026-10-08: it landed and taxied in with it up).
#[test]
fn a_straight_in_arrival_lands_with_its_gear_down() {
    let field = fixtures::xtst();
    let ac = arrival(&field, 31, "AT72", true);
    let s = fly(field, vec![ac], ScriptedTower::default(), 3600.0, 31);
    assert!(s.phases.contains(&Phase::Parked), "{:?}", s.phases);
    assert_eq!(s.gear_up_low, 0, "substeps with the gear up low or on the ground");
}

#[test]
fn a_joining_arrival_flies_the_downwind_of_its_runway() {
    let field = fixtures::xtst();
    let ac = arrival(&field, 31, "C172", false);
    let s = fly(field, vec![ac], ScriptedTower::default(), 3600.0, 31);
    assert_flown(&s, "XTST 27 join, 172");
    assert_eq!(
        &s.phases[..5],
        [
            Phase::Arrive,
            Phase::Pattern { leg: Leg::Downwind },
            Phase::Pattern { leg: Leg::Base },
            Phase::Final,
            Phase::Landing,
        ]
    );
}

/// The 172 circuit at the real Malaga, stand to stand, from the install.
/// `cargo test -p traffic -- --ignored real_install_lemg --nocapture`
#[test]
#[ignore] // needs the real X-Plane install
fn real_install_lemg_172_circuit_stand_to_stand() {
    let apt = nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(field)) = FieldModel::load(&apt, "LEMG") else {
        return; // no install, no verdict
    };
    let stand = field
        .net
        .nearest_stand(36.682214, -4.502304)
        .expect("LEMG has stands")
        .name
        .clone();
    let ac = cessna(&field, 1, &stand, "13", 1);
    let s = fly(field, vec![ac], ScriptedTower::default(), 5400.0, 1);
    assert_flown(&s, &format!("LEMG 13 from {stand}"));
    assert_eq!(s.phases, CIRCUIT_PHASES, "every phase, none skipped");
}

/// How one scheduled aircraft finished: its kind, why it went, and how
/// long it stood parked first.
type Finished = (
    crate::schedule::MovementKind,
    crate::phase::Despawn,
    Option<f64>,
);

/// A generated hour at `field`: every movement spawned at its time, flown
/// against the scripted tower to the end. Returns (kind, despawn reason,
/// seconds parked before going) per aircraft, and every touchdown.
fn fly_schedule(
    field: FieldModel,
    density: crate::schedule::Density,
    seed: u64,
) -> (Vec<Finished>, Vec<f64>) {
    use crate::schedule::{generate, SpawnCtx};
    let plan = generate(&field, density, seed, &SpawnCtx::default());
    assert!(!plan.is_empty());
    let mut fleet = Fleet::new(field);
    let tower = RefCell::new(ScriptedTower::default());
    let mut waiting: Vec<(u32, crate::schedule::Movement)> = plan
        .into_iter()
        .enumerate()
        .map(|(i, m)| (i as u32 + 1, m))
        .collect();
    let mut kinds = HashMap::new();
    let mut parked_at: HashMap<u32, f64> = HashMap::new();
    let mut done = Vec::new();
    let mut touchdowns = Vec::new();
    let mut t = 0.0;
    while t < 3.0 * 3600.0 && (!waiting.is_empty() || !fleet.aircraft().is_empty()) {
        let ctx = SpawnCtx {
            occupied: fleet
                .aircraft()
                .iter()
                .filter(|a| a.kin.on_ground)
                .map(|a| a.kin.pos())
                .collect(),
            ..Default::default()
        };
        waiting.retain(|(id, m)| {
            if m.t_s > t {
                return true;
            }
            match m.spawn(*id, &fleet.field, &ctx) {
                Ok(ac) => {
                    kinds.insert(*id, m.kind);
                    fleet.spawn(ac);
                    false
                }
                Err(_) => true,
            }
        });
        let events = fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned());
        t += 0.1;
        for e in &events {
            tower.borrow_mut().hear(e);
            match e {
                Event::Phase {
                    id,
                    to: Phase::Parked,
                    ..
                } => {
                    parked_at.insert(*id, t);
                }
                Event::Touchdown { along_m, .. } => touchdowns.push(*along_m),
                Event::Despawn { id, reason } => {
                    done.push((kinds[id], *reason, parked_at.get(id).map(|p| t - p)));
                }
                _ => {}
            }
        }
    }
    assert!(
        waiting.is_empty(),
        "never spawned: {:?}",
        waiting.iter().map(|w| &w.1.callsign).collect::<Vec<_>>()
    );
    assert!(
        fleet.aircraft().is_empty(),
        "still flying: {:?}",
        fleet
            .aircraft()
            .iter()
            .map(|a| (a.id, a.phase))
            .collect::<Vec<_>>()
    );
    (done, touchdowns)
}

fn assert_despawned_by_the_rules(done: &[Finished]) {
    use crate::phase::Despawn;
    use crate::schedule::MovementKind;
    for (kind, reason, parked) in done {
        match kind {
            MovementKind::Departure => assert_eq!(*reason, Despawn::LeftTheArea),
            MovementKind::Parked => panic!("a parked aircraft never leaves"),
            MovementKind::Arrival | MovementKind::Circuit => {
                assert_eq!(*reason, Despawn::Parked, "{kind:?}");
                let p = parked.expect("parked first");
                assert!(
                    (p - crate::aircraft::PARKED_LINGER_S).abs() < 0.5,
                    "{kind:?} gone {p:.1} s after parking"
                );
            }
        }
    }
}

#[test]
fn a_generated_hour_at_a_circuit_field_flies_to_the_end() {
    let (done, tds) = fly_schedule(fixtures::xtst_ifr(), crate::schedule::Density::Normal, 11);
    assert_despawned_by_the_rules(&done);
    assert!(done.len() >= 6, "{} movements", done.len());
    assert!(
        tds.iter().all(|d| *d > 0.0 && *d < 500.0),
        "touchdowns {tds:?}"
    );
}

#[test]
fn a_generated_hour_of_jets_flies_to_the_end() {
    let f = fixtures::xnoc();
    let len = f.end("12").unwrap().length_m;
    let (done, tds) = fly_schedule(f, crate::schedule::Density::Normal, 5);
    assert_despawned_by_the_rules(&done);
    println!("XNOC touchdowns {tds:.0?} m into {len:.0} m");
    assert!(
        tds.iter().all(|d| *d > 0.0 && *d < len / 3.0),
        "touchdowns {tds:?}"
    );
}

/// THE READBACK CAP (the spec's risk "a traffic readback failing the gate
/// forever"): an owed readback is given once per owing; still owed
/// [`READBACK_RETRY_S`] later (not taken) it is given again, twice in all,
/// then "say again" once, then the flight leaves the frequency
/// ([`Despawn::ReadbackFailed`]). A readback taken and owed again later (the
/// next lap's identical landing clearance) is given afresh: the old
/// once-ever dedupe never answered it, and the owed text stood for good.
#[test]
fn a_readback_not_taken_twice_says_again_once_then_leaves() {
    use crate::aircraft::{READBACK_RETRY_S, READBACK_TRIES};
    use crate::phase::Despawn;
    let field = fixtures::xtst();
    let mut ac = cessna(&field, 1, "GA1", "09", 1);
    let rb = "Cleared to land runway zero niner, Cessna three four five.".to_string();
    let owed = Order {
        readback_owed: Some(rb.clone()),
        ..Default::default()
    };
    let none = Order::default();
    let mut events = Vec::new();
    let t = std::cell::Cell::new(0.0);
    let gone_at = std::cell::Cell::new(None);
    let step = |ac: &mut TrafficAircraft, o: &Order, secs: f64, events: &mut Vec<Event>| {
        for _ in 0..(secs * 10.0).round() as usize {
            if ac.phase == Phase::Gone {
                return;
            }
            ac.substep(0.1, Some(o), &field, &[], events);
            t.set(t.get() + 0.1);
            if ac.phase == Phase::Gone {
                gone_at.set(Some(t.get()));
            }
        }
    };
    let calls = |events: &[Event]| -> Vec<Call> {
        events
            .iter()
            .filter_map(|e| match e {
                Event::Call { call, .. } if *call != Call::Taxi => Some(call.clone()),
                _ => None,
            })
            .collect()
    };
    // Taken at once, then owed again: read back again.
    step(&mut ac, &owed, 1.0, &mut events);
    step(&mut ac, &none, 1.0, &mut events);
    step(&mut ac, &owed, 1.0, &mut events);
    assert_eq!(
        calls(&events),
        vec![Call::Readback(rb.clone()), Call::Readback(rb.clone())]
    );
    events.clear();
    // Never taken: again, then "say again", then gone.
    let started = t.get();
    step(&mut ac, &owed, 120.0, &mut events);
    assert_eq!(
        calls(&events),
        vec![Call::Readback(rb.clone()), Call::SayAgain],
        "{READBACK_TRIES} readbacks in all, then one say again"
    );
    assert!(events.iter().any(|e| matches!(
        e,
        Event::Despawn {
            reason: Despawn::ReadbackFailed,
            ..
        }
    )));
    let gone = gone_at.get().expect("it left") - started;
    let expect = 3.0 * READBACK_RETRY_S;
    assert!(
        (expect - 2.0..expect + 0.5).contains(&gone),
        "gone {gone:.1} s after the last owing"
    );
    assert_eq!(
        Call::SayAgain.text("Cessna one", None),
        "Tower, Cessna one, say again"
    );
}

/// #242 (BIEG 03): a landing short rolled 1.2 km to the far exit at taxi
/// pace, and the wait for a stand, timed from the START of the vacate, ran
/// out the moment the drive ended: gone `NoStand` with "runway vacated"
/// never said. The wait runs from the vacate's END: the call is made as it
/// stops clear, and only `2 * PARKED_LINGER_S` later, still with nowhere to
/// park, does it leave.
#[test]
fn a_long_vacate_calls_vacated_before_its_wait_for_a_stand_begins() {
    use crate::aircraft::{Route, PARKED_LINGER_S};
    use crate::kin::{project_m, Kin};
    use crate::phase::Despawn;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    // 1.4 km along the strip and off it: about 270 s at the 172's 10 kt.
    let start = end.at(100.0, 0.0);
    let exit = project_m(start, end.course_deg, 1400.0);
    let off = project_m(exit, end.course_deg + 90.0, 60.0);
    let mut kin = Kin::parked(start, end.elev_ft, end.course_deg);
    kin.gs_kts = 10.0;
    let mut ac = TrafficAircraft::new(
        1,
        "N1SP",
        "Cessna one",
        "C172",
        Plan::Arrival {
            entry: start,
            star: None,
            runway: "09".into(),
            straight_in: true,
        },
        kin,
        Phase::Vacate,
        1,
    );
    ac.route = Route::taxi(vec![start, exit, off]);
    // The tower never clears it to a stand.
    let none = Order::default();
    let mut events = Vec::new();
    let (mut t, mut vacated_at, mut gone_at) = (0.0, None, None);
    while t < 900.0 && gone_at.is_none() {
        ac.substep(0.1, Some(&none), &field, &[], &mut events);
        t += 0.1;
        for e in events.drain(..) {
            match e {
                Event::Call {
                    call: Call::Vacated,
                    ..
                } => vacated_at = Some(t),
                Event::Despawn {
                    reason: Despawn::NoStand,
                    ..
                } => gone_at = Some(t),
                _ => {}
            }
        }
    }
    let vacated_at = vacated_at.expect("runway vacated was called");
    let gone_at = gone_at.expect("it left with nowhere to park");
    assert!(
        vacated_at > 2.0 * PARKED_LINGER_S,
        "the drive itself outlasts the old wait: called at {vacated_at:.0} s"
    );
    let waited = gone_at - vacated_at;
    assert!(
        (waited - 2.0 * PARKED_LINGER_S).abs() < 0.5,
        "waited {waited:.1} s after the call"
    );
}

/// Number two turns base on TIME (BIEG 03, 2026-10-08): only once its
/// leader on final will have reached the runway and left it before we get
/// there; a 737 that turned with a 172 at 1.5 nm caught it and went around.
/// A leader still on the runway holds the base; one off it releases it.
#[test]
fn number_two_turns_base_once_its_leader_will_be_off_the_runway() {
    use crate::aircraft::{leader_ready, Seen};
    use crate::kin::project_m;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let seen = |phase: Phase, nm: f64, gs_kts: f64| Seen {
        id: 1,
        phase,
        pos: project_m(end.thr, end.course_deg + 180.0, nm * 1852.0),
        gs_kts,
    };
    // A 172 at 1.5 nm, 70 kt: 79 s to the threshold, plus its runway and the gap.
    let slow = [seen(Phase::Final, 1.5, 70.0)];
    assert!(!leader_ready(1, &slow, &end, 100.0), "a jet 100 s out would catch it");
    assert!(leader_ready(1, &slow, &end, 160.0), "a 172 160 s out follows it");
    assert!(!leader_ready(1, &[seen(Phase::Vacate, 0.0, 10.0)], &end, 300.0), "still on the runway");
    assert!(leader_ready(1, &[seen(Phase::TaxiIn, 0.0, 10.0)], &end, 30.0), "off it");
    assert!(leader_ready(1, &[], &end, 30.0), "a leader gone is no leader");
}

/// A departure of `icao_type` already taxiing on `path` toward runway 09's
/// side of XTST (the spacing tests: no network, the route given).
fn taxiing(field: &FieldModel, id: u32, icao_type: &str, path: Vec<LatLon>) -> TrafficAircraft {
    use crate::aircraft::Route;
    use crate::kin::{bearing, Kin};
    let mut kin = Kin::parked(path[0], field.elev_ft(), bearing(path[0], path[1]));
    kin.lights.taxi = true;
    let mut ac = TrafficAircraft::new(
        id,
        &format!("N{id}"),
        &format!("Traffic {id}"),
        icao_type,
        Plan::Departure {
            runway: "09".into(),
            sid: None,
            exit_bearing: 90.0,
        },
        kin,
        Phase::TaxiOut,
        id as u64,
    );
    ac.route = Route::taxi(path);
    ac
}

/// THE BIEG HOLD (world sweep, 2026-10-08): a Cessna and a Ryanair 737
/// both taxied to 03's Alpha and stopped on the same spot, one inside the
/// other. Two departures for one hold now queue: the second stops behind
/// the first by at least the spacing sized from both, never within it,
/// moves up only once the first has lined up, calls "ready" only from the
/// hold itself, and both depart.
#[test]
fn two_departures_for_one_hold_queue_behind_each_other() {
    use crate::spacing::spacing_m;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let hold = end.at(200.0, -80.0);
    let line = |from: f64| vec![end.at(200.0, -from), hold];
    let first = taxiing(&field, 1, "C172", line(250.0));
    let second = taxiing(&field, 2, "B738", line(450.0));
    let gap = spacing_m(second.perf, first.perf);
    let mut fleet = Fleet::new(field);
    fleet.spawn(first);
    fleet.spawn(second);
    // The tower holds each at the hold for a minute after its "ready",
    // then clears it for takeoff.
    let mut ready_at: HashMap<u32, f64> = HashMap::new();
    let mut lined_up_at: HashMap<u32, f64> = HashMap::new();
    let mut gone = Vec::new();
    let mut closest = f64::INFINITY;
    let mut waited_s = 0.0;
    let mut t = 0.0;
    while t < 1800.0 && gone.len() < 2 {
        let orders: HashMap<u32, Order> = [1, 2]
            .into_iter()
            .map(|id| {
                let cleared = ready_at.get(&id).is_some_and(|r| t - r >= 60.0);
                let state = if cleared {
                    TowerState::TakeoffCleared
                } else {
                    TowerState::TaxiCleared
                };
                (
                    id,
                    Order {
                        state,
                        ..Default::default()
                    },
                )
            })
            .collect();
        let events = fleet.step(0.1, None, &|id| orders.get(&id).cloned());
        t += 0.1;
        for e in &events {
            match e {
                Event::Call {
                    id,
                    call: Call::Ready,
                } => {
                    let a = fleet.get(*id).unwrap();
                    assert!(
                        dist_m(a.kin.pos(), hold) < 2.0,
                        "[{id}] ready {:.1} m from the hold",
                        dist_m(a.kin.pos(), hold)
                    );
                    ready_at.insert(*id, t);
                }
                Event::Phase {
                    id,
                    to: Phase::LineUp,
                    ..
                } => {
                    lined_up_at.insert(*id, t);
                }
                Event::Despawn { id, .. } => gone.push(*id),
                Event::SpacingWaived { .. } => panic!("a queue is never waived: {e:?}"),
                _ => {}
            }
        }
        if let (Some(a), Some(b)) = (fleet.get(1), fleet.get(2)) {
            if matches!(a.phase, Phase::TaxiOut | Phase::HoldShort) {
                closest = closest.min(dist_m(a.kin.pos(), b.kin.pos()));
                if b.kin.gs_kts < 0.1 && b.phase == Phase::TaxiOut {
                    waited_s += 0.1;
                }
            }
        }
    }
    println!("queue: closest {closest:.1} m (spacing {gap:.1}), the second waited {waited_s:.0} s");
    assert!(closest >= gap - 0.5, "{closest:.1} m inside {gap:.1}");
    assert!(closest < gap + 10.0, "it moved up behind the first");
    assert!(waited_s > 30.0, "the second stood in the queue");
    assert!(
        ready_at[&2] > lined_up_at[&1],
        "number two calls ready only after number one has lined up"
    );
    gone.sort();
    assert_eq!(gone, vec![1, 2], "both departed");
}

/// Two movers each with the other ahead (head on, one taxiway): the one
/// with the right of way goes on, the other waits, and both reach their
/// ends; never a standoff.
#[test]
fn a_head_on_meeting_on_one_taxiway_resolves() {
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let (a_end, b_end) = (end.at(100.0, -150.0), end.at(500.0, -150.0));
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(taxiing(&field, 1, "C172", vec![a_end, b_end]));
    fleet.spawn(taxiing(&field, 2, "C172", vec![b_end, a_end]));
    let taxi = Order {
        state: TowerState::TaxiCleared,
        ..Default::default()
    };
    let mut arrived = HashMap::new();
    let mut t = 0.0;
    let mut stood = 0.0;
    while t < 600.0 && arrived.len() < 2 {
        // No "ready" answered: each stays at its end once there.
        let events = fleet.step(0.1, None, &|_| Some(taxi.clone()));
        t += 0.1;
        for e in &events {
            if let Event::Phase {
                id,
                to: Phase::HoldShort,
                ..
            } = e
            {
                arrived.insert(*id, t);
            }
        }
        if fleet
            .get(2)
            .is_some_and(|b| b.phase == Phase::TaxiOut && b.kin.gs_kts < 0.1)
        {
            stood += 0.1;
        }
    }
    println!("head on: arrived {arrived:?}, number two gave way {stood:.0} s");
    assert_eq!(arrived.len(), 2, "both reached their ends");
    assert!(stood > 1.0, "the lower right of way gave way");
    assert!(
        fleet.aircraft().iter().all(|a| a.spacing.waived.is_empty()),
        "resolved by right of way, not by the guard"
    );
}

/// "Ahead" is along the route, not a cone: an aircraft on a parallel
/// taxiway (even a jet coming the other way) or parked beside the line
/// stops nobody. Each flies bit for bit as it does alone.
#[test]
fn traffic_beside_the_path_stops_nobody() {
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let light = vec![end.at(100.0, -150.0), end.at(600.0, -150.0)];
    // The parallel line 45 m off, the other way.
    let jet = vec![end.at(600.0, -195.0), end.at(100.0, -195.0)];
    let trace = |acs: Vec<TrafficAircraft>, watch: u32| -> Vec<u64> {
        let mut fleet = Fleet::new(field.clone());
        for a in acs {
            fleet.spawn(a);
        }
        let mut out = Vec::new();
        for _ in 0..3000 {
            fleet.step(0.1, None, &|_| None);
            if let Some(a) = fleet.get(watch) {
                out.extend([a.kin.lat.to_bits(), a.kin.lon.to_bits()]);
            }
        }
        out
    };
    // Parked on a stand 8 m beside the line, inside the light one's own
    // corridor: a stand link past a neighbour's stand point (LEMG).
    let parked = || {
        let mut p = taxiing(
            &field,
            3,
            "C172",
            vec![end.at(350.0, -142.0), end.at(351.0, -142.0)],
        );
        p.plan = Plan::Parked;
        p.phase = Phase::Parked;
        p
    };
    let alone_light = trace(vec![taxiing(&field, 1, "C172", light.clone())], 1);
    let alone_jet = trace(vec![taxiing(&field, 2, "B738", jet.clone())], 2);
    let both = || {
        vec![
            taxiing(&field, 1, "C172", light.clone()),
            taxiing(&field, 2, "B738", jet.clone()),
            parked(),
        ]
    };
    assert!(alone_light == trace(both(), 1), "the light one was slowed");
    assert!(alone_jet == trace(both(), 2), "the jet was slowed");
}

/// Our own aircraft standing on a traffic aircraft's route stops it short
/// by the spacing (sized as an airliner: our type is not known here); once
/// we move off its line it goes on at its pace and reaches its end.
#[test]
fn our_own_aircraft_on_the_path_stops_a_taxiing_aircraft() {
    use crate::phase::OwnTarget;
    use crate::spacing::spacing_m;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let path = vec![end.at(100.0, -150.0), end.at(600.0, -150.0)];
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(taxiing(&field, 1, "C172", path));
    let own_at = end.at(350.0, -152.0);
    let us = OwnTarget {
        pos: own_at,
        alt_ft: field.elev_ft(),
        on_ground: true,
    };
    let gap = spacing_m(&crate::perf::PISTON_SINGLE, &crate::perf::JET_MED);
    let mut closest = f64::INFINITY;
    let mut t = 0.0;
    // Standing there 80 s: less than the gridlock guard once it has stopped.
    while t < 80.0 {
        let ev = fleet.step(0.1, Some(us), &|_| None);
        assert!(!ev.iter().any(|e| matches!(e, Event::SpacingWaived { .. })));
        t += 0.1;
        closest = closest.min(dist_m(fleet.get(1).unwrap().kin.pos(), own_at));
    }
    let a = fleet.get(1).unwrap();
    println!("behind us: {closest:.1} m (spacing {gap:.1})");
    assert_eq!(a.phase, Phase::TaxiOut);
    assert!(a.kin.gs_kts < 0.1, "stopped behind us");
    assert!(
        closest >= gap - 0.5 && closest < gap + 10.0,
        "{closest:.1} m"
    );
    // We taxi away off its line: it goes on to its end.
    let moved = OwnTarget {
        pos: end.at(350.0, -300.0),
        ..us
    };
    let mut t = 0.0;
    while t < 200.0 && fleet.get(1).unwrap().phase == Phase::TaxiOut {
        fleet.step(0.1, Some(moved), &|_| None);
        t += 0.1;
    }
    assert_eq!(fleet.get(1).unwrap().phase, Phase::HoldShort, "went on");
}

/// The gridlock guard: an aircraft that will not move (our own here)
/// standing on the route longer than [`crate::spacing::STUCK_S`] is passed,
/// and said ([`Event::SpacingWaived`]), rather than the field stopping for
/// good.
#[test]
fn a_blocker_that_never_moves_is_passed_after_the_guard() {
    use crate::phase::OwnTarget;
    use crate::spacing::STUCK_S;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let path = vec![end.at(100.0, -150.0), end.at(600.0, -150.0)];
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(taxiing(&field, 1, "C172", path));
    let us = OwnTarget {
        pos: end.at(350.0, -150.0),
        alt_ft: field.elev_ft(),
        on_ground: true,
    };
    let (mut waived, mut arrived, mut t) = (None, false, 0.0);
    while t < 600.0 && !arrived {
        for e in fleet.step(0.1, Some(us), &|_| None) {
            match e {
                Event::SpacingWaived { id: 1, past: None } => waived = Some(t),
                Event::Phase {
                    to: Phase::HoldShort,
                    ..
                } => arrived = true,
                _ => {}
            }
        }
        t += 0.1;
    }
    let w = waived.expect("the guard passed us");
    assert!(w > STUCK_S, "only after the guard: {w:.0} s");
    assert!(arrived, "and reached its end");
}

/// THE BIEG GRIDLOCK (world sweep, 2026-10-08): a landing's vacate stopped
/// behind the queue for a crossing of the runway it had just left, never
/// reached its drawn end, never said "runway vacated", and its landing
/// held the runway whole, so the crossing ahead was never cleared. Stopped
/// clear of the strip behind traffic, a vacate has vacated: it says so
/// there, where it stands.
#[test]
fn a_vacate_stopped_clear_of_the_runway_behind_traffic_reports_vacated() {
    use crate::aircraft::Route;
    use crate::kin::Kin;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let start = end.at(300.0, 0.0);
    let mut kin = Kin::parked(start, end.elev_ft, end.course_deg - 90.0);
    kin.gs_kts = 5.0;
    let mut vacate = TrafficAircraft::new(
        1,
        "N1",
        "Traffic one",
        "C172",
        Plan::Arrival {
            entry: start,
            star: None,
            runway: "09".into(),
            straight_in: true,
        },
        kin,
        Phase::Vacate,
        1,
    );
    vacate.route = Route::taxi(vec![start, end.at(300.0, -200.0)]);
    // Holding in the exit, facing the same way, 150 m off the centreline.
    let mut ahead = taxiing(
        &field,
        2,
        "C172",
        vec![end.at(300.0, -150.0), end.at(300.0, -151.0)],
    );
    ahead.phase = Phase::HoldShort;
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(vacate);
    fleet.spawn(ahead);
    let mut vacated = None;
    let mut t = 0.0;
    while t < 120.0 && vacated.is_none() {
        for e in fleet.step(0.1, None, &|_| None) {
            if let Event::Call {
                id: 1,
                call: Call::Vacated,
            } = e
            {
                vacated = Some(fleet.get(1).unwrap().kin.pos());
            }
        }
        t += 0.1;
    }
    let at = vacated.expect("runway vacated, behind the traffic");
    assert!(!field.on_runway(at), "said clear of the strip");
    let gap = dist_m(at, end.at(300.0, -150.0));
    assert!(
        gap >= crate::spacing::spacing_m(&crate::perf::PISTON_SINGLE, &crate::perf::PISTON_SINGLE)
            - 0.5,
        "behind the traffic: {gap:.1} m"
    );
}

/// Nose to nose with one standing still (a departure held at its hold, the
/// arrival taxiing in past it): it will not come on, so waiting for it only
/// holds the mover. Passed, never waited for (UUYY's hour, 2026-10-08:
/// six arrivals stood 90 s each in front of a held departure).
#[test]
fn one_standing_head_on_is_passed_not_waited_for() {
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let path = vec![end.at(100.0, -150.0), end.at(600.0, -150.0)];
    // Held at 350 m along, facing back down the mover's line.
    let mut held = taxiing(
        &field,
        2,
        "B738",
        vec![end.at(350.0, -150.0), end.at(349.0, -150.0)],
    );
    held.phase = Phase::HoldShort;
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(taxiing(&field, 1, "C172", path));
    fleet.spawn(held);
    let mut slowest = f64::INFINITY;
    let mut t = 0.0;
    while t < 300.0 && fleet.get(1).unwrap().phase == Phase::TaxiOut {
        fleet.step(0.1, None, &|_| None);
        t += 0.1;
        let a = fleet.get(1).unwrap();
        if t > 20.0 && a.phase == Phase::TaxiOut && a.route.dist_to(a.kin.pos(), 1) > 50.0 {
            slowest = slowest.min(a.kin.gs_kts);
        }
    }
    assert_eq!(
        fleet.get(1).unwrap().phase,
        Phase::HoldShort,
        "reached its end"
    );
    assert!(slowest > 9.0, "never slowed for it: {slowest:.1} kt");
}
