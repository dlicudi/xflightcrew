//! Traffic "sorties": whole flights through the phase machine against a
//! scripted tower (no ATC), scored like our own: cross-track on the taxi
//! and the final, the touchdown point, every phase in order, and the same
//! trace twice.

use std::cell::RefCell;
use std::collections::HashMap;

use crate::field::{fixtures, FieldModel};
use crate::kin::{cross_track_m, dist_m};
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
            Call::Base | Call::Readback(_) => {}
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
