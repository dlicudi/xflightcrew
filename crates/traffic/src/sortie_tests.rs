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
    // The leg off the strip is long too: down the runway is now flown at
    // the brisker backtrack pace.
    let off = project_m(exit, end.course_deg + 90.0, 600.0);
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
                Event::SpacingRerouted { .. }
                | Event::SpacingCleared { .. }
                | Event::SpacingHeld { .. } => panic!("a queue is waited in: {e:?}"),
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
        assert!(!ev.iter().any(|e| matches!(
            e,
            Event::SpacingRerouted { .. }
                | Event::SpacingCleared { .. }
                | Event::SpacingHeld { .. }
        )));
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

/// The closest two taxiing aircraft (one of them moving) stand in `fleet`,
/// as a share of the spacing sized from both: the check every new spacing
/// test makes each substep (nothing driven through anybody, 2026-10-09).
fn closest_taxi_share(fleet: &Fleet) -> f64 {
    use crate::spacing::spacing_m;
    let taxiing = |p: Phase| {
        matches!(
            p,
            Phase::TaxiOut | Phase::TaxiIn | Phase::Vacate | Phase::LineUp | Phase::HoldShort
        )
    };
    let ground: Vec<&TrafficAircraft> = fleet
        .aircraft()
        .iter()
        .filter(|a| a.kin.on_ground && taxiing(a.phase))
        .collect();
    let mut closest = f64::INFINITY;
    for (k, a) in ground.iter().enumerate() {
        for b in &ground[k + 1..] {
            if a.kin.gs_kts < 0.5 && b.kin.gs_kts < 0.5 {
                continue;
            }
            let d = dist_m(a.kin.pos(), b.kin.pos());
            closest = closest.min(d / spacing_m(a.perf, b.perf));
        }
    }
    closest
}

/// The guard's events in `events`: (rerouted, towed, held).
fn guard_events(events: &[Event]) -> (usize, usize, usize) {
    let n = |f: fn(&Event) -> bool| events.iter().filter(|e| f(e)).count();
    (
        n(|e| matches!(e, Event::SpacingRerouted { .. })),
        n(|e| matches!(e, Event::SpacingCleared { .. })),
        n(|e| matches!(e, Event::SpacingHeld { .. })),
    )
}

/// HEAD ON AT A JUNCTION (the pass-through, 2026-10-09: the one with the
/// right of way drove THROUGH the other). [2] taxis east along the line;
/// [1] comes up a side branch and would turn west down the same line,
/// straight at it. [2] is already on the stretch they share, so [1] (the
/// lower id, but not yet on it) holds short of the junction until [2] is
/// past it, then goes; the two never come within the spacing of each
/// other, and nobody needs the guard.
#[test]
fn head_on_at_a_junction_the_one_giving_way_holds_short_of_it() {
    use crate::spacing::spacing_m;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let junction = end.at(400.0, -150.0);
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(taxiing(
        &field,
        1,
        "C172",
        vec![end.at(400.0, -300.0), junction, end.at(100.0, -150.0)],
    ));
    fleet.spawn(taxiing(
        &field,
        2,
        "C172",
        vec![end.at(100.0, -150.0), end.at(700.0, -150.0)],
    ));
    let gap = spacing_m(fleet.get(1).unwrap().perf, fleet.get(2).unwrap().perf);
    let taxi = Order {
        state: TowerState::TaxiCleared,
        ..Default::default()
    };
    let (mut arrived, mut all, mut closest, mut stood, mut t) =
        (HashMap::new(), Vec::new(), f64::INFINITY, 0.0, 0.0);
    let mut held_at: Option<LatLon> = None;
    while t < 600.0 && arrived.len() < 2 {
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
        all.extend(events);
        closest = closest.min(closest_taxi_share(&fleet));
        let one = fleet.get(1).unwrap();
        if one.phase == Phase::TaxiOut && one.kin.gs_kts < 0.1 && t > 5.0 {
            stood += 0.1;
            held_at.get_or_insert(one.kin.pos());
        }
    }
    let held_at = held_at.expect("number one held");
    println!(
        "junction: arrived {arrived:?}, number one held {stood:.0} s {:.0} m short of the \
         junction, closest {closest:.2} of the spacing ({gap:.0} m)",
        dist_m(held_at, junction)
    );
    assert_eq!(arrived.len(), 2, "both reached their ends");
    assert!(
        arrived[&2] < arrived[&1],
        "the one on the stretch went first"
    );
    assert!(stood > 10.0, "number one gave way: {stood:.0} s");
    assert!(
        end.frame(held_at).1 < -160.0,
        "held on the branch, off the line: {:.0} m",
        end.frame(held_at).1
    );
    assert!(closest >= 0.9, "never inside the spacing: {closest:.2}");
    assert_eq!(guard_events(&all), (0, 0, 0), "no guard needed");
}

/// HEAD ON ON A DEAD END: a 172 taxiing up a line with a 737 PARKED on it,
/// facing it, and nothing else to the way. It stops short by the spacing
/// and waits; after the guard it is planned another way round on the
/// network, or (with none) the 737 is towed clear, and then it goes on to
/// its end. Never driven through it (2026-10-09).
#[test]
fn a_parked_blocker_on_the_line_is_gone_round_or_towed_never_driven_through() {
    use crate::spacing::{spacing_m, STUCK_S};
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let blocker_at = end.at(350.0, -150.0);
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(taxiing(
        &field,
        1,
        "C172",
        vec![end.at(100.0, -150.0), end.at(600.0, -150.0)],
    ));
    let mut parked = taxiing(&field, 2, "B738", vec![blocker_at, end.at(349.0, -150.0)]);
    parked.plan = Plan::Parked;
    parked.phase = Phase::Parked;
    fleet.spawn(parked);
    let gap = spacing_m(&crate::perf::PISTON_SINGLE, &crate::perf::JET_MED);
    let (mut closest, mut t, mut acted, mut all) = (f64::INFINITY, 0.0, None, Vec::new());
    while t < 900.0 && fleet.get(1).is_some_and(|a| a.phase == Phase::TaxiOut) {
        let events = fleet.step(0.1, None, &|_| None);
        t += 0.1;
        if acted.is_none() && guard_events(&events) != (0, 0, 0) {
            acted = Some(t);
        }
        all.extend(events);
        if let (Some(a), Some(b)) = (fleet.get(1), fleet.get(2)) {
            closest = closest.min(dist_m(a.kin.pos(), b.kin.pos()));
        }
    }
    let (rerouted, towed, held) = guard_events(&all);
    println!(
        "dead end: guard at {acted:?} s, rerouted {rerouted} towed {towed} held {held}, \
         closest {closest:.1} m (spacing {gap:.1})"
    );
    let at = acted.expect("the guard acted");
    assert!(at >= STUCK_S, "only after the guard: {at:.0} s");
    assert!(rerouted + towed == 1, "gone round, or the blocker towed");
    assert!(closest >= 0.7 * gap, "{closest:.1} m inside {gap:.1}");
    assert_eq!(
        fleet.get(1).map(|a| a.phase),
        Some(Phase::HoldShort),
        "and reached its end"
    );
}

/// OUR OWN AIRCRAFT IS NEVER TOWED AND NEVER DRIVEN THROUGH: a taxiing
/// aircraft held behind us is planned another way round, or waits as long
/// as we stand there, saying so once and nothing more.
#[test]
fn our_own_aircraft_is_gone_round_or_waited_for_never_passed() {
    use crate::phase::OwnTarget;
    use crate::spacing::spacing_m;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let path = vec![end.at(100.0, -150.0), end.at(600.0, -150.0)];
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(taxiing(&field, 1, "C172", path));
    let own_at = end.at(350.0, -150.0);
    let us = OwnTarget {
        pos: own_at,
        alt_ft: field.elev_ft(),
        on_ground: true,
    };
    let gap = spacing_m(&crate::perf::PISTON_SINGLE, &crate::perf::JET_MED);
    let (mut closest, mut t, mut all) = (f64::INFINITY, 0.0, Vec::new());
    while t < 900.0 && fleet.get(1).unwrap().phase == Phase::TaxiOut {
        all.extend(fleet.step(0.1, Some(us), &|_| None));
        t += 0.1;
        closest = closest.min(dist_m(fleet.get(1).unwrap().kin.pos(), own_at));
    }
    let (rerouted, towed, held) = guard_events(&all);
    println!(
        "behind us: rerouted {rerouted} held {held}, closest {closest:.1} m (spacing {gap:.1})"
    );
    assert_eq!(towed, 0, "never towed");
    assert!(
        all.iter().all(|e| match e {
            Event::SpacingRerouted { around, .. } => around.is_none(),
            Event::SpacingHeld { behind, .. } => behind.is_none(),
            _ => true,
        }),
        "the guard was about us"
    );
    assert!(held <= 1, "said once: {held}");
    // Stopped by the spacing, then a turn away (its first metres swing on).
    assert!(closest >= 0.9 * gap, "{closest:.1} m inside {gap:.1}");
    let a = fleet.get(1).unwrap();
    if rerouted == 1 {
        assert_eq!(a.phase, Phase::HoldShort, "gone round to its end");
    } else {
        assert_eq!(held, 1);
        assert!(a.kin.gs_kts < 0.1, "still waiting behind us");
    }
}

/// Nose to nose with one standing still (a departure held at its hold, the
/// arrival taxiing in past it, UUYY's hour, 2026-10-08): it stops short by
/// the spacing, never through it, and after the guard goes another way
/// round or has the one that will not move towed clear.
#[test]
fn one_standing_head_on_is_never_driven_through() {
    use crate::spacing::spacing_m;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let path = vec![end.at(100.0, -150.0), end.at(600.0, -150.0)];
    let held_at = end.at(350.0, -150.0);
    let mut held = taxiing(&field, 2, "B738", vec![held_at, end.at(349.0, -150.0)]);
    held.phase = Phase::HoldShort;
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(taxiing(&field, 1, "C172", path));
    fleet.spawn(held);
    let gap = spacing_m(&crate::perf::PISTON_SINGLE, &crate::perf::JET_MED);
    let (mut closest, mut t, mut all) = (f64::INFINITY, 0.0, Vec::new());
    while t < 900.0 && fleet.get(1).unwrap().phase == Phase::TaxiOut {
        all.extend(fleet.step(0.1, None, &|_| None));
        t += 0.1;
        if let (Some(a), Some(b)) = (fleet.get(1), fleet.get(2)) {
            closest = closest.min(dist_m(a.kin.pos(), b.kin.pos()));
        }
    }
    let (rerouted, towed, _) = guard_events(&all);
    println!("held head on: rerouted {rerouted} towed {towed}, closest {closest:.1} m");
    assert!(closest >= 0.9 * gap, "{closest:.1} m inside {gap:.1}");
    assert!(rerouted + towed == 1, "gone round, or towed");
    assert_eq!(
        fleet.get(1).unwrap().phase,
        Phase::HoldShort,
        "reached its end"
    );
}

/// Met too late to hold short (both already on the one stretch, nose to
/// nose, no junction between them): both stop by the spacing, neither
/// drives through, and the guard turns ONE of them away round the network
/// (the lesser right of way is asked first; here its only way to its hold
/// runs back through the other, so the other turns); both reach their
/// holds.
#[test]
fn met_head_on_too_late_both_stop_and_one_is_turned_away() {
    use crate::spacing::STUCK_S;
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
    let (mut closest, mut t, mut all) = (f64::INFINITY, 0.0, Vec::new());
    let mut first_guard = None;
    while t < 1200.0 && fleet.aircraft().iter().any(|a| a.phase == Phase::TaxiOut) {
        let ev = fleet.step(0.1, None, &|_| Some(taxi.clone()));
        t += 0.1;
        if first_guard.is_none() {
            first_guard = ev.iter().find_map(|e| match e {
                Event::SpacingRerouted { id, .. } | Event::SpacingCleared { id, .. } => {
                    Some((*id, t))
                }
                _ => None,
            });
        }
        all.extend(ev);
        closest = closest.min(closest_taxi_share(&fleet));
    }
    let (rerouted, towed, held) = guard_events(&all);
    println!(
        "too late: first guard {first_guard:?}, rerouted {rerouted} towed {towed} held {held}, \
         closest {closest:.2} of the spacing"
    );
    let (_, at) = first_guard.expect("the guard sorted it");
    assert!(at >= STUCK_S);
    assert_eq!(rerouted + towed, 1, "one turned away");
    assert!(closest >= 0.7, "never inside the spacing: {closest:.2}");
    assert!(
        fleet.aircraft().iter().all(|a| a.phase == Phase::HoldShort),
        "both reached their ends: {:?}",
        fleet
            .aircraft()
            .iter()
            .map(|a| (a.id, a.phase))
            .collect::<Vec<_>>()
    );
}

/// ONE STAND, ONE AIRCRAFT: `field`'s schedule for two hours (the second
/// generated round the fleet at the hour, as the link does), its parked
/// aircraft and its departures waiting on their stands put down at the
/// start, everything flown against the scripted tower. At every substep no
/// stand holds two aircraft (parked on it, or taxiing in to it), and no
/// two parked aircraft stand inside each other (CYJP and EFMI in the hour
/// sweep, 2026-10-08: stands of one name resolved to the first). Returns
/// the movements flown and the findings.
fn fly_hours_one_per_stand(field: FieldModel, seed: u64) -> (usize, Vec<String>) {
    use crate::field::STAND_TAKEN_M;
    use crate::schedule::{generate, park_at_start, Density, Movement, SpawnCtx};
    let mut schedule = generate(&field, Density::Normal, seed, &SpawnCtx::default());
    let parked = park_at_start(
        &field,
        Density::Normal,
        seed,
        &SpawnCtx::default(),
        &mut schedule,
    );
    let mut fleet = Fleet::new(field);
    let tower = RefCell::new(ScriptedTower::default());
    let mut next = 1u32;
    let ctx = |fleet: &Fleet| SpawnCtx {
        occupied: fleet.stands_held(),
        ..Default::default()
    };
    // (time, the dormant aircraft standing for it, the movement)
    let mut due: Vec<(f64, Option<u32>, Movement)> = Vec::new();
    for m in schedule {
        let held = if m.parked_from_start {
            m.park(next, &fleet.field, &ctx(&fleet)).ok().map(|ac| {
                let id = ac.id;
                fleet.spawn(ac);
                next += 1;
                id
            })
        } else {
            None
        };
        due.push((m.t_s, held, m));
    }
    for m in parked {
        if let Ok(ac) = m.park(next, &fleet.field, &ctx(&fleet)) {
            fleet.spawn(ac);
            next += 1;
        }
    }
    let mut findings = Vec::new();
    let mut flown = 0;
    let mut regenerated = false;
    let mut t = 0.0;
    while t < 2.0 * 3600.0 {
        if !regenerated && t >= 3600.0 {
            regenerated = true;
            for m in generate(&fleet.field, Density::Normal, seed + 1, &ctx(&fleet)) {
                due.push((t + m.t_s, None, m));
            }
        }
        let mut later = Vec::new();
        for (at, held, m) in std::mem::take(&mut due) {
            if at > t {
                later.push((at, held, m));
                continue;
            }
            if let Some(id) = held {
                if let Ok(plan) = m.plan(&fleet.field) {
                    if fleet.release(id, plan) {
                        flown += 1;
                        continue;
                    }
                }
            }
            match m.spawn(next, &fleet.field, &ctx(&fleet)) {
                Ok(ac) => {
                    fleet.spawn(ac);
                    next += 1;
                    flown += 1;
                }
                Err(crate::schedule::Refusal::NoRunway) => {}
                Err(_) => later.push((t + 30.0, None, m)),
            }
        }
        due = later;
        let events = fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned());
        t += 0.1;
        for e in &events {
            tower.borrow_mut().hear(e);
        }
        // Who is on, or bound for, each stand.
        let stands = &fleet.field.net.stands;
        let mut on: HashMap<usize, Vec<u32>> = HashMap::new();
        for a in fleet.aircraft() {
            let k = match a.phase {
                Phase::Parked => stands
                    .iter()
                    .enumerate()
                    .map(|(k, s)| (k, dist_m(a.kin.pos(), (s.lat, s.lon))))
                    .filter(|(_, d)| *d < STAND_TAKEN_M)
                    .min_by(|x, y| x.1.total_cmp(&y.1))
                    .map(|(k, _)| k),
                Phase::TaxiIn => a
                    .stand
                    .as_deref()
                    .and_then(|n| stands.iter().position(|s| s.name == n)),
                _ => None,
            };
            if let Some(k) = k {
                on.entry(k).or_default().push(a.id);
            }
        }
        for (k, ids) in on.iter().filter(|(_, ids)| ids.len() > 1) {
            if findings.len() < 10 {
                findings.push(format!("{t:.0} s stand {:?}: {ids:?}", stands[*k].name));
            }
        }
        let parked: Vec<&TrafficAircraft> = fleet
            .aircraft()
            .iter()
            .filter(|a| a.phase == Phase::Parked)
            .collect();
        for (k, a) in parked.iter().enumerate() {
            for b in &parked[k + 1..] {
                let d = dist_m(a.kin.pos(), b.kin.pos());
                if d < 5.0 && findings.len() < 10 {
                    findings.push(format!("{t:.0} s [{}] and [{}] {d:.1} m apart", a.id, b.id));
                }
            }
        }
    }
    (flown, findings)
}

/// EFMI's real layout: eight of its ten stands are called "Apron Stand".
fn efmi() -> FieldModel {
    let mut f = FieldModel::load(&fixtures::path("efmi.dat"), "EFMI")
        .unwrap()
        .expect("EFMI in the fixture");
    f.procs.sids = vec!["ELBO1A".into(), "ELBO1B".into()];
    f.procs.approaches = vec!["R11".into(), "R29".into()];
    f
}

/// CYJP's real layout: four tie-downs all called "GA Tie-Down".
fn cyjp() -> FieldModel {
    FieldModel::load(&fixtures::path("cyjp.dat"), "CYJP")
        .unwrap()
        .expect("CYJP in the fixture")
}

/// A field that names its stands alike has them named apart: each one
/// found by its own name.
#[test]
fn every_stand_has_a_name_of_its_own() {
    for f in [efmi(), cyjp()] {
        let names: std::collections::HashSet<&str> =
            f.net.stands.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names.len(), f.net.stands.len(), "{}", f.icao);
        for s in &f.net.stands {
            let found = f.stand(&s.name).unwrap();
            assert!(
                (found.lat, found.lon) == (s.lat, s.lon),
                "{} {}",
                f.icao,
                s.name
            );
        }
    }
    assert!(efmi().stand("Apron Stand 8").is_some());
}

/// TWO AIRCRAFT ON ONE STAND (CYJP, EFMI, the hour sweep, 2026-10-08):
/// never, at spawn, parked at the start, or taxiing in, over two generated
/// hours at fields with many stands, several of them named alike.
#[test]
fn no_stand_ever_holds_two_aircraft() {
    let fields = [
        efmi(),
        cyjp(),
        fixtures::xtst_ifr(),
        fixtures::xnoc(),
        FieldModel::load(&fixtures::path("ulmm.dat"), "ULMM")
            .unwrap()
            .map(|mut f| {
                f.procs.sids = vec!["ASGO2E".into(), "EKLE2G".into()];
                f.procs.approaches = vec!["I13-Y".into(), "I31-Y".into()];
                f
            })
            .unwrap(),
    ];
    for f in fields {
        for seed in [3, 7] {
            let icao = f.icao.clone();
            let (flown, findings) = fly_hours_one_per_stand(f.clone(), seed);
            println!(
                "{icao} seed {seed}: {flown} flown, {} findings",
                findings.len()
            );
            assert!(flown > 0, "{icao}: nothing flew");
            assert!(findings.is_empty(), "{icao} seed {seed}: {findings:#?}");
        }
    }
}

/// The same over real fields from the install, the sweep's two among them.
/// `cargo test -p traffic --release -- --ignored real_install_no_stand --nocapture`
#[test]
#[ignore] // needs the real X-Plane install
fn real_install_no_stand_ever_holds_two_aircraft() {
    let apt = nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let cifp = nav::test_xplane_path("Custom Data/CIFP");
    for icao in [
        "CYJP", "EFMI", "LEMG", "BIEG", "EFUT", "UUYY", "ENGM", "BIKF",
    ] {
        let Ok(Some(f)) = FieldModel::load(&apt, icao) else {
            println!("{icao}: not in the install");
            continue;
        };
        let f = f.with_cifp(&cifp);
        let (flown, findings) = fly_hours_one_per_stand(f, 3);
        println!("{icao}: {flown} flown, {} findings", findings.len());
        assert!(findings.is_empty(), "{icao}: {findings:#?}");
    }
}

/// A generated hour is flown with nobody ever inside anybody's spacing on
/// the ground: every substep, every pair of taxiing aircraft.
#[test]
fn a_generated_hour_keeps_every_taxiing_pair_apart() {
    for f in [fixtures::xtst_ifr(), fixtures::xnoc()] {
        let icao = f.icao.clone();
        let mut fleet = Fleet::new(f);
        let plan = crate::schedule::generate(
            &fleet.field,
            crate::schedule::Density::Normal,
            11,
            &crate::schedule::SpawnCtx::default(),
        );
        let tower = RefCell::new(ScriptedTower::default());
        let mut waiting: Vec<(u32, crate::schedule::Movement)> = plan
            .into_iter()
            .enumerate()
            .map(|(i, m)| (i as u32 + 1, m))
            .collect();
        let mut closest = f64::INFINITY;
        let mut t = 0.0;
        while t < 2.0 * 3600.0 && (!waiting.is_empty() || !fleet.aircraft().is_empty()) {
            let ctx = crate::schedule::SpawnCtx {
                occupied: fleet.stands_held(),
                ..Default::default()
            };
            waiting.retain(|(id, m)| {
                m.t_s > t
                    || match m.spawn(*id, &fleet.field, &ctx) {
                        Ok(ac) => {
                            fleet.spawn(ac);
                            false
                        }
                        Err(_) => true,
                    }
            });
            for e in fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned()) {
                tower.borrow_mut().hear(&e);
            }
            t += 0.1;
            closest = closest.min(closest_taxi_share(&fleet));
        }
        println!("{icao}: closest taxiing pair {closest:.2} of the spacing");
        assert!(closest >= 0.7, "{icao}: {closest:.2} of the spacing");
    }
}
