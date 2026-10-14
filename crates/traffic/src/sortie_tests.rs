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
            Call::PushStart { .. } => o.push_approved = true,
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
            Call::Established {
                approach: false, ..
            } => {
                if self.refuse_landings > 0 {
                    self.refuse_landings -= 1;
                } else {
                    o.state = TowerState::ClearedToLand;
                }
            }
            Call::Base
            | Call::Readback(_)
            | Call::SayAgain
            | Call::ArrivalCheckIn { .. }
            | Call::MissedCheckIn { .. }
            | Call::Established { .. } => {}
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

/// The runway slot a landing publishes at touchdown is the rollout it then
/// flies (2026-10-10): its way off is planned at speed at the wheels
/// ([`TrafficAircraft::plan_exit`]), and the slot's "off the runway" is
/// that exit, braked for and driven, so the tower's timing is the
/// aircraft's own. An ATR at XTST 27: the exit picked at touchdown, the
/// report within a few seconds of the slot's time for it.
#[test]
fn a_landings_slot_at_touchdown_is_the_rollout_it_flies() {
    let field = fixtures::xtst();
    let ac = arrival(&field, 32, "AT72", true);
    let mut fleet = Fleet::new(field);
    assert!(fleet.spawn(ac));
    let tower = RefCell::new(ScriptedTower::default());
    let (mut t, mut down, mut said) = (0.0, None, None);
    while t < 3600.0 && said.is_none() {
        let events = {
            let tw = &tower;
            fleet.step(0.1, None, &|id| tw.borrow().orders.get(&id).cloned())
        };
        t += 0.1;
        for ev in &events {
            tower.borrow_mut().hear(ev);
            match ev {
                Event::Touchdown { id: 32, .. } => {
                    let ac = fleet.get(32).unwrap();
                    assert!(ac.exit.is_some(), "its way off planned at the wheels");
                    down = Some((t, ac.slot().expect("a slot on the runway").clear_s));
                }
                Event::Call {
                    id: 32,
                    call: Call::Vacated,
                } => said = Some(t),
                _ => {}
            }
        }
    }
    let (t0, clear_s) = down.expect("it touched down");
    let flown = said.expect("runway vacated") - t0;
    assert!(
        (flown - clear_s).abs() <= 5.0_f64.max(0.15 * clear_s),
        "slot said off in {clear_s:.1} s, flown {flown:.1} s"
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
    assert_eq!(
        s.gear_up_low, 0,
        "substeps with the gear up low or on the ground"
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
    // Seed 6: since the schedule flies the field's own rate (2026-10-09,
    // four movements an hour here, not twelve) seed 5's hour has an
    // arrival taxiing in meet the next departure leaving its stand, and the
    // scripted tower's pair is towed apart (owed: ground spacing).
    let (done, tds) = fly_schedule(f, crate::schedule::Density::Normal, 6);
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
/// never said. The wait runs from the call: made once it is clear of the
/// runway's pavement (2026-10-10, said as it goes, no longer at the end of
/// its way off), and only `2 * PARKED_LINGER_S` later, still with nowhere
/// to park, does it leave.
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
        vacated_at > PARKED_LINGER_S,
        "called only off the strip, after the long roll down it: at {vacated_at:.0} s"
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
        slot: None,
    };
    // Number two needing the runway `s` from now.
    let own = |s: f64| crate::slot::RunwaySlot {
        thr: end.thr,
        far: end.far,
        to_thr_s: s,
        clear_s: s + 40.0,
        unjudged_s: 0.0,
        needs_clear_s: 0.0,
        committed: false,
        waiting: false,
        wake: crate::perf::Wake::Light,
        rejoining: false,
    };
    // A 172 at 1.5 nm, 70 kt: 79 s to the threshold, plus its runway and the gap.
    let slow = [seen(Phase::Final, 1.5, 70.0)];
    assert!(
        !leader_ready(1, &slow, &end, &own(100.0)),
        "a jet 100 s out would catch it"
    );
    assert!(
        leader_ready(1, &slow, &end, &own(160.0)),
        "a 172 160 s out follows it"
    );
    assert!(
        !leader_ready(1, &[seen(Phase::Vacate, 0.0, 10.0)], &end, &own(300.0)),
        "still on the runway"
    );
    assert!(
        leader_ready(1, &[seen(Phase::TaxiIn, 0.0, 10.0)], &end, &own(30.0)),
        "off it"
    );
    assert!(
        leader_ready(1, &[], &end, &own(30.0)),
        "a leader gone is no leader"
    );
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
        gs_kts: 0.0,
        slot: None,
        route: Vec::new(),
    };
    let gap = spacing_m(&crate::perf::PISTON_SINGLE, &crate::perf::JET_MED);
    let mut closest = f64::INFINITY;
    let mut t = 0.0;
    // Standing there 80 s: less than the gridlock guard once it has stopped.
    while t < 80.0 {
        let ev = fleet.step(0.1, Some(us.clone()), &|_| None);
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
        fleet.step(0.1, Some(moved.clone()), &|_| None);
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

/// A DEPARTURE WAITING FOR THE RUNWAY IS A QUEUE, NEVER A BLOCKAGE: LEMG
/// 12 (2026-10-14), two departures held on Delta for their takeoff were
/// towed off one after the other by a third joining from a branch behind
/// them, and vanished in front of our own aircraft (the geometry there is
/// not yet recreated: owed). [2] stands on the line under a "hold
/// position"; [3] comes up a branch to join the line behind it the same
/// way and queues. Ten minutes on, [2] is still there and was never towed.
#[test]
fn a_departure_held_for_the_runway_is_never_towed_by_one_joining_behind_it() {
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(taxiing(
        &field,
        2,
        "A320",
        vec![end.at(600.0, -150.0), end.at(900.0, -150.0)],
    ));
    fleet.spawn(taxiing(
        &field,
        3,
        "B738",
        vec![
            end.at(400.0, -450.0),
            end.at(400.0, -150.0),
            end.at(900.0, -150.0),
        ],
    ));
    let order = |id: u32| {
        Some(Order {
            state: TowerState::TaxiCleared,
            hold: id == 2,
            ..Default::default()
        })
    };
    let (mut t, mut all) = (0.0, Vec::new());
    while t < 600.0 {
        all.extend(fleet.step(0.1, None, &order));
        t += 0.1;
    }
    let (rerouted, towed, held) = guard_events(&all);
    println!("held departure: rerouted {rerouted} towed {towed} held {held}");
    assert_eq!(towed, 0, "the held departure was towed");
    assert_eq!(
        fleet.get(2).map(|a| a.phase),
        Some(Phase::TaxiOut),
        "still waiting for the runway"
    );
}

/// A HELD DEPARTURE FACING A MOVER IS STILL NO BLOCKAGE: [2] waits under a
/// "hold position" for its takeoff, its way ahead turning off north; [3]
/// comes west down the line and stops nose to it. [2] is not waiting on
/// [3] (its way does not run through it), and [3], on the tower's route,
/// has no other way: it waits. [2] is never towed.
#[test]
fn a_held_departure_nose_to_a_mover_is_never_towed() {
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let mut fleet = Fleet::new(field.clone());
    let mut held = taxiing(
        &field,
        2,
        "A320",
        vec![end.at(300.0, -150.0), end.at(300.0, 100.0)],
    );
    held.kin.hdg_deg = crate::kin::bearing(end.at(300.0, -150.0), end.at(600.0, -150.0));
    fleet.spawn(held);
    // On the tower's route, which the guard may not plan another way.
    let mut mover = taxiing(
        &field,
        3,
        "C172",
        vec![end.at(700.0, -150.0), end.at(0.0, -150.0)],
    );
    mover.tower_route = true;
    fleet.spawn(mover);
    let order = |id: u32| {
        Some(Order {
            state: TowerState::TaxiCleared,
            hold: id == 2,
            ..Default::default()
        })
    };
    let (mut t, mut all) = (0.0, Vec::new());
    while t < 600.0 {
        all.extend(fleet.step(0.1, None, &order));
        t += 0.1;
    }
    let (rerouted, towed, held) = guard_events(&all);
    println!("nose to a held departure: rerouted {rerouted} towed {towed} held {held}");
    assert_eq!(towed, 0, "the held departure was towed");
    assert!(fleet.get(2).is_some_and(|a| a.phase == Phase::TaxiOut));
}

/// A "READY" NOBODY ANSWERED IS CALLED AGAIN: LEMG (2026-10-14), a
/// departure at the hold called the tower before its strip held a taxi
/// clearance, Ground answered it, and it stood forty minutes silent. At
/// the hold, still only cleared to taxi and not held, it calls again every
/// [`READY_RETRY_S`]; held by the tower, or told to line up behind the
/// landing traffic, its one call stands.
#[test]
fn an_unanswered_ready_call_is_made_again() {
    use crate::aircraft::READY_RETRY_S;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let readies = |order: Order| {
        let mut fleet = Fleet::new(field.clone());
        let mut ac = taxiing(
            &field,
            1,
            "A320",
            vec![end.at(300.0, -150.0), end.at(300.0, -100.0)],
        );
        ac.phase = Phase::HoldShort;
        fleet.spawn(ac);
        let (mut t, mut n) = (0.0, 0);
        while t < 2.5 * READY_RETRY_S {
            for e in fleet.step(0.1, None, &|_| Some(order.clone())) {
                if matches!(
                    e,
                    Event::Call {
                        call: Call::Ready,
                        ..
                    }
                ) {
                    n += 1;
                }
            }
            t += 0.1;
        }
        n
    };
    let taxi = Order {
        state: TowerState::TaxiCleared,
        ..Default::default()
    };
    assert_eq!(readies(taxi.clone()), 3, "called, then again twice");
    let held = Order {
        hold: true,
        ..taxi.clone()
    };
    assert_eq!(readies(held), 0, "held: no call");
    let behind = Order {
        line_up_behind: true,
        ..taxi
    };
    assert_eq!(readies(behind), 1, "answered: one call");
}

/// A HOLD IS AN ANSWER (#286, LEMG 2026-10-14): EZY2264 called ready, was
/// held 117 s for a lander, and the hold lifted one tick before its takeoff
/// clearance arrived; counted from the first call it was "90 s unanswered"
/// and called ready again after reading back cleared for takeoff. The
/// retry counts from the release.
#[test]
fn a_released_hold_is_not_an_unanswered_ready() {
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let mut fleet = Fleet::new(field.clone());
    let mut ac = taxiing(
        &field,
        1,
        "A320",
        vec![end.at(300.0, -150.0), end.at(300.0, -100.0)],
    );
    ac.phase = Phase::HoldShort;
    fleet.spawn(ac);
    let taxi = Order {
        state: TowerState::TaxiCleared,
        ..Default::default()
    };
    let held = Order {
        hold: true,
        ..taxi.clone()
    };
    let cleared = Order {
        state: TowerState::TakeoffCleared,
        ..Default::default()
    };
    let mut readies = 0;
    let mut t = 0.0;
    // Called, then held for the lander, released, cleared a tick later.
    while t < 140.0 {
        let order = if t < 1.0 {
            taxi.clone()
        } else if t < 118.0 {
            held.clone()
        } else if t < 118.2 {
            taxi.clone()
        } else {
            cleared.clone()
        };
        for e in fleet.step(0.1, None, &|_| Some(order.clone())) {
            if matches!(e, Event::Call { call: Call::Ready, .. }) {
                readies += 1;
            }
        }
        t += 0.1;
    }
    assert_eq!(readies, 1, "one ready, answered by the hold and the clearance");
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

/// OUR ROUTE BUYS US NO RIGHT OF WAY (the pilot, 2026-10-14: "giving us
/// priority is not realistic"). We taxi east along the line, standing for
/// anything just ahead of us as our own guard does; [1] comes up a side
/// branch to turn west down the same line, straight at us. Published or
/// not, our route changes nothing in the meeting: who goes first is
/// Ground's to say (#297), never a silent yield to the player.
#[test]
fn our_published_route_buys_us_no_right_of_way() {
    use crate::phase::OwnTarget;
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    let junction = end.at(400.0, -150.0);
    let run = |publish: bool| {
        let mut fleet = Fleet::new(field.clone());
        fleet.spawn(taxiing(
            &field,
            1,
            "C172",
            vec![end.at(400.0, -300.0), junction, end.at(100.0, -150.0)],
        ));
        let taxi = Order {
            state: TowerState::TaxiCleared,
            ..Default::default()
        };
        let (mut x, mut t, mut on_our_stretch, mut stood) = (100.0, 0.0, false, 0.0);
        while t < 300.0 && x < 700.0 {
            let us = end.at(x, -150.0);
            let own = OwnTarget {
                pos: us,
                alt_ft: field.elev_ft(),
                on_ground: true,
                gs_kts: 8.0,
                slot: None,
                route: if publish {
                    vec![us, end.at(700.0, -150.0)]
                } else {
                    Vec::new()
                },
            };
            fleet.step(0.1, Some(own), &|_| Some(taxi.clone()));
            t += 0.1;
            let Some(a) = fleet.get(1) else { break };
            let (along, right) = end.frame(a.kin.pos());
            // On the line ahead of us, between us and the junction.
            if (right + 150.0).abs() < 20.0 && along > x && x < 400.0 {
                on_our_stretch = true;
            }
            // Our own guard: we stand for anything on the line just ahead.
            let blocked = (right + 150.0).abs() < 25.0 && along > x && along - x < 80.0;
            if blocked {
                stood += 0.1;
            } else {
                x += 8.0 * crate::kin::KT_TO_MS * 0.1;
            }
        }
        let held = fleet.get(1).map(|a| end.frame(a.kin.pos()));
        println!(
            "published {publish}: we reached {x:.0} m in {t:.0} s, stood {stood:.0} s, it on \
             our stretch {on_our_stretch}, it at {held:?}"
        );
        (x, stood, on_our_stretch)
    };
    let published = run(true);
    let unseen = run(false);
    assert!(published.2 && unseen.2, "it drove onto our stretch both times");
    assert_eq!(published, unseen, "the route changed the meeting");
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
        gs_kts: 0.0,
        slot: None,
        route: Vec::new(),
    };
    let gap = spacing_m(&crate::perf::PISTON_SINGLE, &crate::perf::JET_MED);
    let (mut closest, mut t, mut all) = (f64::INFINITY, 0.0, Vec::new());
    while t < 900.0 && fleet.get(1).unwrap().phase == Phase::TaxiOut {
        all.extend(fleet.step(0.1, Some(us.clone()), &|_| None));
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

/// THE ENAT CROSSING (the hour sweep, 2026-10-09): a Ryanair cleared across
/// runway 11 ran 227 m down the strip and turned off into an exit a Cirrus
/// was taxiing up, to cross back the other way. The Cirrus held short at
/// first, was let go up the lane as the crossing came on (the stretch they
/// shared is seen only to the lookahead, so its start ran toward the
/// Cirrus), and the Ryanair stopped ON the runway nose to nose with it at
/// its hold short: "expedite crossing", until the Cirrus was towed. Here a
/// 737 crosses XTST 09 the same way, 230 m down the strip and off into a
/// 300 m lane a 172 is starting up from its foot: the 172 holds well down
/// the lane (it drove up it to its hold short, 0.19 of the spacing from
/// the 737), and the 737 never stands on the runway; neither comes inside
/// the spacing of the other.
#[test]
fn a_crossing_down_the_runway_is_never_held_on_it_by_traffic_coming_up_its_exit() {
    let field = fixtures::xtst();
    let end = field.end("09").unwrap();
    // South of the runway (no taxiways there: the routes are given).
    let crossing = vec![
        end.at(930.0, 80.0),
        end.at(930.0, 0.0),
        end.at(700.0, 0.0),
        end.at(700.0, 80.0),
        end.at(700.0, 400.0),
        end.at(400.0, 400.0),
    ];
    let climbing = vec![end.at(700.0, 370.0), end.at(700.0, 80.0)];
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(taxiing(&field, 1, "B738", crossing));
    fleet.spawn(taxiing(&field, 2, "C172", climbing));
    let taxi = Order {
        state: TowerState::TaxiCleared,
        ..Default::default()
    };
    let (mut closest, mut stood_on_runway, mut t, mut all) = (f64::INFINITY, 0.0, 0.0, Vec::new());
    let mut held: f64 = 0.0;
    while t < 900.0 {
        let ev = fleet.step(0.1, None, &|_| Some(taxi.clone()));
        t += 0.1;
        all.extend(ev);
        closest = closest.min(closest_taxi_share(&fleet));
        if let Some(a) = fleet.get(1) {
            if a.kin.gs_kts < 0.5 && field.on_runway(a.kin.pos()) && t > 1.0 {
                stood_on_runway += 0.1;
            }
        }
        if let Some(b) = fleet.get(2) {
            if b.phase == Phase::TaxiOut && b.kin.gs_kts < 0.1 && t > 5.0 {
                held = held.max(end.frame(b.kin.pos()).1);
            }
        }
        if fleet
            .get(1)
            .is_none_or(|a| a.phase == Phase::HoldShort || end.frame(a.kin.pos()).1 > 300.0)
        {
            break;
        }
    }
    let (rerouted, towed, said) = guard_events(&all);
    println!(
        "ENAT crossing: {t:.0} s, the 737 stood on the runway {stood_on_runway:.1} s, the 172 \
         held {held:.0} m south of the centreline, closest {closest:.2} of the spacing, guard \
         {rerouted}/{towed}/{said}"
    );
    assert_eq!(stood_on_runway, 0.0, "never held on the runway");
    assert!(held > 100.0, "the 172 held well down its lane: {held:.0} m");
    assert!(closest >= 0.7, "never inside the spacing: {closest:.2}");
    let a = fleet.get(1);
    assert!(
        a.is_none_or(|a| !field.on_runway(a.kin.pos())),
        "and the 737 is off the runway"
    );
}

/// THE BIGH VACATE (#249, the runway-slot sweep, 2026-10-09): an arrival's
/// vacate ended exactly where the last arrival had ended its own and
/// stood, held short of a crossing of the same runway; closest taxiing
/// pair 0.00 of the spacing. With one standing at the end its vacate would
/// have, a 172 landing on XTST 27 ends its vacate off the runway and clear
/// of it, never inside the spacing.
#[test]
fn a_vacate_never_ends_where_an_aircraft_stands() {
    let field = fixtures::xtst();
    let lone = arrival(&field, 30, "C172", true);
    let mut alone = Fleet::new(field.clone());
    alone.spawn(lone);
    let mut vacate_end = None;
    let mut t = 0.0;
    while t < 1800.0 && vacate_end.is_none() {
        alone.step(0.1, None, &|_| {
            Some(Order {
                state: TowerState::ClearedToLand,
                ..Default::default()
            })
        });
        t += 0.1;
        if let Some(a) = alone.get(30).filter(|a| a.phase == Phase::Vacate) {
            vacate_end = a.route.path.last().copied();
        }
    }
    let spot = vacate_end.expect("it vacated");
    let mut standing = taxiing(&field, 4, "C172", vec![spot, (spot.0, spot.1 + 1e-5)]);
    standing.phase = Phase::HoldShort;
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(standing);
    fleet.spawn(arrival(&field, 30, "C172", true));
    let (mut ended, mut planned) = (None, None);
    let mut closest = f64::INFINITY;
    let mut t = 0.0;
    while t < 1800.0 && ended.is_none() {
        let ev = fleet.step(0.1, None, &|id| {
            (id == 30).then(|| Order {
                state: TowerState::ClearedToLand,
                ..Default::default()
            })
        });
        t += 0.1;
        closest = closest.min(closest_taxi_share(&fleet));
        if let Some(a) = fleet.get(30).filter(|a| a.phase == Phase::Vacate) {
            planned.get_or_insert(*a.route.path.last().unwrap());
        }
        if ev.iter().any(|e| {
            matches!(
                e,
                Event::Call {
                    id: 30,
                    call: Call::Vacated
                }
            )
        }) {
            ended = fleet.get(30).map(|a| a.kin.pos());
        }
    }
    let at = ended.expect("runway vacated");
    let planned = planned.expect("a vacate planned");
    let apart = dist_m(at, spot);
    println!(
        "BIGH vacate: planned to end {:.0} m from the one standing, ended {apart:.0} m from it, \
         closest {closest:.2} of the spacing",
        dist_m(planned, spot)
    );
    assert!(
        dist_m(planned, spot) >= 40.0,
        "the vacate is never planned onto its spot: {:.0} m",
        dist_m(planned, spot)
    );
    assert!(
        !field.beside_runway(planned),
        "and ends off the runway's side"
    );
    assert!(!field.on_runway(at), "off the runway");
    assert!(
        apart
            >= crate::spacing::spacing_m(&crate::perf::PISTON_SINGLE, &crate::perf::PISTON_SINGLE),
        "clear of the one standing: {apart:.0} m"
    );
    assert!(closest >= 0.7, "never inside the spacing: {closest:.2}");
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

/// An arrival to XTST 27 entering `nm` out on `bearing` from its threshold.
fn entering(
    field: &FieldModel,
    id: u32,
    icao_type: &str,
    straight_in: bool,
    nm: f64,
    bearing: f64,
) -> TrafficAircraft {
    let end = field.end("27").unwrap();
    let entry = crate::kin::project_m(end.thr, bearing, nm * 1852.0);
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

/// What two arrivals flown together did, per id: when each touched down,
/// when each was off the runway again, the most it extended its downwind,
/// where it began each orbit (metres from the threshold), go-arounds.
#[derive(Default, Debug)]
struct Pair {
    touchdown: HashMap<u32, f64>,
    off: HashMap<u32, f64>,
    ext_m: HashMap<u32, f64>,
    orbits_at_m: HashMap<u32, Vec<f64>>,
    go_arounds: usize,
}

/// Fly `first` from the start and `second` from `second_at` seconds, the
/// scripted tower clearing each, until both are off the runway again.
fn fly_pair(
    field: FieldModel,
    first: TrafficAircraft,
    second: TrafficAircraft,
    second_at: f64,
) -> Pair {
    let end = field.end("27").unwrap();
    let mut fleet = Fleet::new(field.clone());
    let ids = [first.id, second.id];
    fleet.spawn(first);
    let mut second = Some(second);
    let tower = RefCell::new(ScriptedTower::default());
    let mut p = Pair::default();
    let mut orbits: HashMap<u32, u32> = HashMap::new();
    let mut t = 0.0;
    while t < 3600.0 {
        if t >= second_at {
            if let Some(ac) = second.take() {
                fleet.spawn(ac);
            }
        }
        let events = fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned());
        t += 0.1;
        for e in &events {
            tower.borrow_mut().hear(e);
            match e {
                Event::Touchdown { id, .. } => {
                    p.touchdown.entry(*id).or_insert(t);
                }
                Event::Call {
                    call: Call::GoingAround,
                    ..
                } => p.go_arounds += 1,
                _ => {}
            }
        }
        for ac in fleet.aircraft() {
            let ext = p.ext_m.entry(ac.id).or_default();
            *ext = ext.max(ac.ext_m);
            let n = orbits.entry(ac.id).or_default();
            if ac.orbits > *n {
                p.orbits_at_m
                    .entry(ac.id)
                    .or_default()
                    .push(dist_m(ac.kin.pos(), end.thr));
            }
            *n = ac.orbits;
            if p.touchdown.contains_key(&ac.id)
                && ac.kin.on_ground
                && !field.on_runway(ac.kin.pos())
            {
                p.off.entry(ac.id).or_insert(t);
            }
        }
        if second.is_none() && ids.iter().all(|id| p.off.contains_key(id)) {
            break;
        }
    }
    p
}

/// Seconds from its spawn to its touchdown for `ac` flown alone.
fn touchdown_alone(field: &FieldModel, ac: TrafficAircraft) -> f64 {
    let id = ac.id;
    let never = entering(field, 99, "C172", true, 30.0, 0.0);
    fly_pair(field.clone(), ac, never, f64::INFINITY).touchdown[&id]
}

/// THE BIEG FINAL (the hour sweep, 2026-10-09): a circuit 172 turned base
/// in front of a straight-in on its long final and one of them went
/// around, nine to twelve times an hour. Timed so the straight-in reaches
/// the threshold just before the 172 would, the 172 now extends its
/// downwind and lands only once the straight-in is off the runway; the
/// straight-in flies on.
#[test]
fn a_circuit_extends_its_downwind_behind_a_straight_in_that_gets_there_first() {
    let field = fixtures::xtst();
    let join = || entering(&field, 1, "C172", false, 6.0, 140.0);
    let jet = || entering(&field, 2, "AT72", true, 10.0, 90.0);
    let join_alone = touchdown_alone(&field, join());
    let jet_alone = touchdown_alone(&field, jet());
    // The straight-in reaches the threshold 20 s before the 172 would: an
    // airliner appears 25 nm or more out (`crate::arrival`), so the 172
    // starts that much later.
    let join_at = jet_alone + 20.0 - join_alone;
    assert!(
        join_at > 0.0,
        "the 172 lands at {join_alone:.0} s, the jet {jet_alone:.0} s"
    );
    let p = fly_pair(field.clone(), jet(), join(), join_at);
    println!("{p:?}");
    assert_eq!(p.go_arounds, 0);
    assert!(p.ext_m[&1] > 0.0, "the downwind was extended");
    assert!(!p.orbits_at_m.contains_key(&2), "the straight-in flew on");
    assert!(
        p.touchdown[&1] > p.off[&2] + 20.0,
        "the 172 landed at {:.0} s, the jet was off at {:.0} s",
        p.touchdown[&1],
        p.off[&2]
    );
}

/// A straight-in that cannot fit behind the traffic committed ahead of it
/// is given its delay EARLY, an orbit well out (never inside 4 nm), and
/// lands once the one ahead is off: never a go-around on short final. A
/// light single joining VFR (an airliner slows first, below).
#[test]
fn a_straight_in_that_cannot_fit_behind_the_one_ahead_orbits_far_out() {
    let field = fixtures::xtst();
    let lead = || entering(&field, 1, "C172", true, 10.0, 90.0);
    let jet = || entering(&field, 2, "PA28", true, 10.0, 90.0);
    let lead_alone = touchdown_alone(&field, lead());
    let jet_alone = touchdown_alone(&field, jet());
    // The jet would touch down 30 s behind the 172, still on the runway.
    let jet_at = lead_alone + 30.0 - jet_alone;
    assert!(
        jet_at > 0.0,
        "the 172 lands at {lead_alone:.0} s, the jet {jet_alone:.0} s"
    );
    let p = fly_pair(field.clone(), lead(), jet(), jet_at);
    println!("{p:?}");
    assert_eq!(p.go_arounds, 0);
    let orbits = p.orbits_at_m.get(&2).expect("the jet orbited");
    assert!(
        orbits.iter().all(|m| *m >= crate::slot::ORBIT_MIN_M),
        "orbits at {orbits:?} m"
    );
    assert!(!p.orbits_at_m.contains_key(&1), "the 172 ahead flew on");
    assert!(
        p.touchdown[&2] > p.off[&1],
        "the jet landed at {:.0} s, the 172 was off at {:.0} s",
        p.touchdown[&2],
        p.off[&1]
    );
}

/// XTST with an approach unit (apt.dat row 1056): instrument arrivals
/// work it before the tower.
fn xtst_with_approach() -> FieldModel {
    let mut f = fixtures::xtst_ifr();
    f.airport.freqs.push(nav::AtcFreq {
        kind: nav::FREQ_APPROACH,
        khz: 126_825,
        name: "XTST Director".into(),
    });
    f
}

/// The realism review (2026-10-10): an airliner spawned 10 nm out and
/// joined a 6 nm final. Now it appears 25 to 40 nm out, high on the
/// 3 degree profile, in the 250 kt band, and joins the final 11 nm out; its calls
/// go to approach (check-in, established), then to the tower established
/// 9 nm out, in that order; nothing else said in the air.
#[test]
fn an_airliner_arrives_from_far_out_and_works_approach_then_the_tower() {
    let field = xtst_with_approach();
    let ac = entering(&field, 40, "AT72", true, 10.0, 100.0);
    let end = field.end("27").unwrap();
    let nm = dist_m(ac.kin.pos(), end.thr) / 1852.0;
    assert!((25.0..=40.0).contains(&nm), "spawned {nm:.1} nm out");
    assert!(
        ac.kin.alt_ft - end.elev_ft > 7000.0,
        "{:.0} ft",
        ac.kin.alt_ft
    );
    // The 250 band, at what a turboprop flies it.
    assert!(ac.kin.gs_kts >= 200.0, "{} kt", ac.kin.gs_kts);
    let join = ac.route.path[ac.route.thr_idx.unwrap() - 1];
    assert!((dist_m(join, end.thr) / 1852.0 - crate::arrival::JOIN_NM).abs() < 0.01);
    let s = fly(field, vec![ac], ScriptedTower::default(), 3600.0, 40);
    assert!(s.phases.contains(&Phase::Parked), "{:?}", s.phases);
    assert!(!s.phases.iter().any(|p| matches!(p, Phase::Pattern { .. })));
    let air: Vec<&Call> = s
        .calls
        .iter()
        .filter(|c| !matches!(c, Call::Vacated | Call::Taxi | Call::Readback(_)))
        .collect();
    assert!(
        matches!(
            air.as_slice(),
            [
                Call::ArrivalCheckIn { .. },
                Call::Established { approach: true, .. },
                Call::Established {
                    approach: false,
                    miles: 9,
                    ..
                },
            ]
        ),
        "{air:?}"
    );
    assert_eq!(s.gear_up_low, 0);
}

/// With no approach unit the tower is the first call, established 8 nm out.
#[test]
fn with_no_approach_unit_the_tower_is_called_established_at_eight_miles() {
    let field = fixtures::xtst_ifr();
    let ac = entering(&field, 41, "B738", true, 10.0, 80.0);
    let s = fly(field, vec![ac], ScriptedTower::default(), 3600.0, 41);
    assert!(
        matches!(
            s.calls.first(),
            Some(Call::Established {
                approach: false,
                miles: 8,
                ..
            })
        ),
        "{:?}",
        s.calls
    );
}

/// Two airliners on one approach, the second spawned close behind: it is
/// spaced by SPEED, not an orbit, and is at least its wake spacing (3 nm,
/// medium behind medium) out as the first crosses the threshold; nobody
/// goes around.
#[test]
fn a_follower_slows_to_keep_its_wake_spacing_on_final() {
    let field = fixtures::xtst_ifr();
    let end = field.end("27").unwrap();
    let mut fleet = Fleet::new(field.clone());
    let at = |id: u32, nm: f64| {
        let e = crate::kin::project_m(end.thr, 90.0, nm * 1852.0);
        TrafficAircraft::at_entry(
            id,
            &format!("T{id}"),
            "Traffic",
            "A320",
            Plan::Arrival {
                entry: e,
                star: None,
                runway: "27".into(),
                straight_in: true,
            },
            &field,
            id as u64,
        )
        .unwrap()
    };
    let (lead, follow) = (at(1, 30.0), at(2, 31.0));
    assert!(dist_m(lead.kin.pos(), follow.kin.pos()) < 1.5 * 1852.0);
    fleet.spawn(lead);
    fleet.spawn(follow);
    let tower = RefCell::new(ScriptedTower::default());
    let (mut t, mut gap, mut go_arounds, mut slowest) = (0.0, None, 0, f64::INFINITY);
    while t < 3600.0 && fleet.get(2).is_some_and(|a| a.phase != Phase::Vacate) {
        let events = fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned());
        t += 0.1;
        for e in &events {
            tower.borrow_mut().hear(e);
            match e {
                Event::Phase {
                    id: 1,
                    to: Phase::Landing,
                    ..
                } => {
                    let f = fleet.get(2).unwrap();
                    gap = Some(f.route.dist_to(f.kin.pos(), f.route.thr_idx.unwrap()) / 1852.0);
                }
                Event::Call {
                    call: Call::GoingAround,
                    ..
                } => go_arounds += 1,
                _ => {}
            }
        }
        if let Some(f) = fleet.get(2).filter(|f| f.phase == Phase::Arrive) {
            slowest = f64::min(slowest, f.kin.gs_kts);
        }
    }
    let gap = gap.expect("the leader landed");
    assert_eq!(go_arounds, 0);
    assert_eq!(fleet.get(2).map_or(0, |a| a.orbits), 0, "spaced by speed");
    assert!(
        (3.0..6.5).contains(&gap),
        "{gap:.2} nm behind at the threshold"
    );
    assert!(slowest < 200.0, "it slowed ({slowest:.0} kt)");
}

/// An airliner's go-around (here: no landing clearance by 300 ft) flies the
/// missed approach to its altitude, says so to approach, is vectored round
/// (never a VFR downwind) and lands on its second approach 10 to 20
/// minutes later.
#[test]
fn an_airliner_go_around_flies_the_missed_approach_and_comes_back_round() {
    let field = xtst_with_approach();
    let end = field.end("27").unwrap();
    let ac = entering(&field, 42, "AT72", true, 30.0, 90.0);
    let tower = ScriptedTower {
        refuse_landings: 1,
        ..Default::default()
    };
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(ac);
    let tower = RefCell::new(tower);
    let (mut t, mut went, mut landed, mut top) = (0.0, None, None, 0.0_f64);
    let mut calls = Vec::new();
    while t < 3600.0 && landed.is_none() {
        let events = fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned());
        t += 0.1;
        for e in &events {
            tower.borrow_mut().hear(e);
            match e {
                Event::Call { call, .. } => {
                    if *call == Call::GoingAround {
                        went = Some(t);
                    }
                    calls.push(call.clone());
                }
                Event::Phase { to, .. } => {
                    assert!(!matches!(to, Phase::Pattern { .. }), "a VFR circuit");
                }
                Event::Touchdown { .. } => landed = Some(t),
                _ => {}
            }
        }
        if went.is_some() && landed.is_none() {
            if let Some(a) = fleet.get(42) {
                top = top.max(a.kin.alt_ft - end.elev_ft);
            }
        }
    }
    let (went, landed) = (went.expect("it went around"), landed.expect("it landed"));
    let mins = (landed - went) / 60.0;
    assert!((10.0..20.0).contains(&mins), "back {mins:.1} min later");
    let ma = crate::arrival::missed_approach_alt_ft(end.elev_ft) - end.elev_ft;
    assert!(
        (top - ma).abs() < 150.0,
        "climbed to {top:.0} ft, told {ma:.0}"
    );
    let after: Vec<&Call> = calls
        .iter()
        .skip_while(|c| **c != Call::GoingAround)
        .collect();
    assert!(
        matches!(
            after.as_slice(),
            [
                Call::GoingAround,
                Call::MissedCheckIn { .. },
                Call::Established { approach: true, .. },
                Call::Established {
                    approach: false,
                    ..
                },
                ..
            ]
        ),
        "{after:?}"
    );
}

/// TURNAROUNDS (realism review, 2026-10-09): an arrival that parks stays
/// on its stand, dark and silent, for its turnaround, then leaves from that
/// stand as the same carrier's next flight; it was gone two minutes after
/// parking and every later departure appeared out of nothing.
#[test]
fn an_arrival_turns_round_and_leaves_as_its_carriers_next_flight() {
    use crate::schedule::{generate, next_flight, Density, MovementKind, SpawnCtx};
    let mut field = fixtures::xnoc();
    let ms = generate(&field, Density::Normal, 4, &SpawnCtx::default());
    // XNOC's three stands leave the arrivals still to come too few once one
    // is held (`STANDS_KEPT_FREE`): spare ones far off, as a real apron has.
    let spare = field.net.stands[0].clone();
    for k in 0..4 {
        let mut s = spare.clone();
        s.name = format!("spare {k}");
        s.lat += 0.01 * f64::from(k + 1);
        field.net.stands.push(s);
    }
    let arr = ms
        .iter()
        .find(|m| m.kind == MovementKind::Arrival)
        .expect("an arrival")
        .clone();
    let dep = ms
        .iter()
        .find(|m| m.kind == MovementKind::Departure)
        .expect("a departure")
        .clone();
    let mut fleet = Fleet::new(field);
    fleet.turn_round = true;
    assert!(fleet.spawn(arr.spawn(1, &fleet.field, &SpawnCtx::default()).unwrap()));
    let tower = RefCell::new(ScriptedTower::default());
    let mut t = 0.0;
    let mut turned_at = None;
    while t < 3600.0 && turned_at.is_none() {
        let events = fleet.step(0.1, None, &|id| tower.borrow().orders.get(&id).cloned());
        t += 0.1;
        for e in &events {
            tower.borrow_mut().hear(e);
            assert!(!matches!(e, Event::Despawn { .. }), "{e:?}");
            if matches!(e, Event::TurnedRound { id: 1 }) {
                turned_at = Some(t);
            }
        }
    }
    let turned_at = turned_at.expect("parked and turned round");
    let a = fleet.get(1).expect("still on its stand");
    let stand = a.stand.clone().expect("on a stand");
    assert!(a.dormant() && a.phase == Phase::Parked && a.kin.gs_kts == 0.0);
    assert!(!a.kin.lights.beacon && fleet.turned_round(1));
    assert_eq!(a.callsign, arr.callsign);
    let pos = a.kin.pos();
    // Not ready before its turnaround: a narrowbody stands 35 minutes at
    // least, an hour at most.
    let all = |_: &TrafficAircraft| true;
    let mut ready = None;
    while t < turned_at + 4000.0 {
        assert!(
            fleet.step(1.0, None, &|_| None).is_empty(),
            "silent and still"
        );
        t += 1.0;
        if fleet.ready_to_leave(&[], &all).is_some() {
            ready = Some(t - turned_at);
            break;
        }
    }
    let ready = ready.expect("ready within the hour");
    assert!((34.0 * 60.0..=61.0 * 60.0).contains(&ready), "{ready:.0} s");
    assert!(
        fleet.ready_to_leave(&[1], &all).is_none(),
        "held for another"
    );
    let (cs, spoken) =
        next_flight("XNOC", &arr.callsign, &mut crate::rng::XorShift64::new(1)).unwrap();
    assert_eq!(cs[..3], arr.callsign[..3], "the same carrier");
    assert_ne!(cs, arr.callsign);
    assert!(fleet.release_as(1, dep.plan(&fleet.field).unwrap(), &cs, &spoken));
    assert!(!fleet.turned_round(1));
    let events = fleet.step(1.0, None, &|_| None);
    assert!(events.iter().any(|e| matches!(
        e,
        Event::Call {
            id: 1,
            call: Call::Taxi
        }
    )));
    let a = fleet.get(1).unwrap();
    assert_eq!((a.callsign.as_str(), a.kin.pos()), (cs.as_str(), pos));
    assert_eq!(
        a.stand.as_deref(),
        Some(stand.as_str()),
        "from the stand it parked on"
    );
    assert_eq!(a.icao_type, arr.icao_type, "the same airframe");
}

/// Off (the phase machine's own default), a finished arrival lingers and
/// is gone as before.
#[test]
fn without_turnarounds_a_finished_arrival_still_goes() {
    let (done, _) = fly_schedule(fixtures::xtst_ifr(), crate::schedule::Density::Normal, 11);
    assert!(done
        .iter()
        .any(|(k, r, _)| *k == crate::schedule::MovementKind::Arrival
            && *r == crate::phase::Despawn::Parked));
}

/// The way still ahead (the EFB's selected aircraft): past the segment
/// being flown, thinned with the last point kept, empty once done.
#[test]
fn the_route_ahead_is_what_is_left_of_it() {
    let path: Vec<LatLon> = (0..50).map(|i| (36.0 + i as f64 * 0.001, -4.0)).collect();
    let mut r = crate::aircraft::Route::taxi(path.clone());
    r.follow.seg = 9;
    assert_eq!(r.ahead(100), path[10..].to_vec());
    let thin = r.ahead(5);
    assert_eq!(thin.len(), 5);
    assert_eq!(thin[0], path[10]);
    assert_eq!(*thin.last().unwrap(), path[49]);
    r.follow.done = true;
    assert!(r.ahead(5).is_empty());
}

/// THE WHOLE FLIGHT'S HANDOVER IN (2026-10-13, [`crate::track`]): an
/// arrival from Paris is modelled to the simulation's entry and spawned
/// there; at the handover the model and the spawned aircraft agree in
/// position, altitude and speed, and a second before it the model is a
/// second's flight short of it. The entry faces the origin, on the
/// approach side.
#[test]
fn an_arrival_enters_the_simulation_where_the_model_puts_it() {
    use crate::schedule::{Movement, MovementKind, SpawnCtx};
    use crate::track::{gc_dist_m, End, Whole};
    let field = fixtures::xnoc();
    let end = field.end("12").unwrap();
    let origin = crate::places::position("LFPG").unwrap();
    let entry = crate::arrival::entry_toward(&end, origin, 0.4, None).unwrap();
    let off = crate::kin::signed_diff_deg(
        crate::kin::bearing(end.thr, entry),
        end.course_deg + 180.0,
    );
    assert!(off.abs() <= crate::arrival::ENTRY_SIDE_DEG + 0.5, "{off}");
    let m = Movement {
        t_s: 3600.0,
        callsign: "AFR1234".into(),
        spoken: "Air France one two three four".into(),
        icao_type: "A320".into(),
        kind: MovementKind::Arrival,
        runway: "12".into(),
        stand: None,
        entry: Some(entry),
        straight_in: true,
        laps: 0,
        sid: None,
        star: None,
        exit_bearing: 0.0,
        seed: 77,
        parked_from_start: false,
        flight: None,
        start: None,
    };
    let anchor = m.entry_anchor(9, &field).unwrap();
    let here = field.reference().unwrap();
    let w = Whole::inbound(
        "LFPG",
        "XNOC",
        here,
        End::air(&anchor.kin),
        3600.0,
        4500.0,
        anchor.perf,
    )
    .unwrap();
    let spawned = m.spawn(9, &field, &SpawnCtx::default()).unwrap();
    let fix = w.at(3600.0).unwrap();
    let gap = gc_dist_m(fix.pos, spawned.kin.pos());
    assert!(gap < 1.0, "{gap} m apart at the handover");
    assert!((fix.alt_ft - spawned.kin.alt_ft).abs() < 1.0);
    assert!((fix.gs_kts - spawned.kin.gs_kts).abs() < 0.5);
    let before = w.at(3599.0).unwrap();
    let step = gc_dist_m(before.pos, spawned.kin.pos());
    let want = spawned.kin.gs_kts * crate::kin::KT_TO_MS;
    assert!((step - want).abs() < 0.05 * want, "{step} m in the last second");
    assert!(before.alt_ft >= spawned.kin.alt_ft, "descending into it");
    // It left Paris about an hour and a half before (440 nm).
    let up = (3600.0 - w.track.t_a) / 3600.0;
    assert!((0.9..1.6).contains(&up), "{up} h");
}

/// THE WHOLE FLIGHT'S HANDOVER OUT: a departure to Paris flown by the
/// simulation until it leaves the area; the model takes it from that very
/// state and time, on toward Paris, still climbing.
#[test]
fn a_departure_leaves_the_simulation_where_the_model_takes_it() {
    use crate::track::{gc_bearing, gc_dist_m, End, Whole};
    let field = fixtures::xnoc();
    let here = field.reference().unwrap();
    let paris = crate::places::position("LFPG").unwrap();
    let ac = TrafficAircraft::at_stand(
        30,
        "IBE3201",
        "Iberia three two zero one",
        "A320",
        Plan::Departure {
            runway: "12".into(),
            sid: None,
            exit_bearing: gc_bearing(here, paris),
        },
        &field,
        "S1",
        30,
    )
    .unwrap();
    let mut fleet = Fleet::new(field);
    assert!(fleet.spawn(ac));
    let tower = RefCell::new(ScriptedTower::default());
    let mut last = None;
    let mut left = Vec::new();
    for _ in 0..40_000 {
        let events = {
            let tw = &tower;
            fleet.step(0.1, None, &|id| tw.borrow().orders.get(&id).cloned())
        };
        for e in &events {
            tower.borrow_mut().hear(e);
        }
        left = fleet.take_left();
        if !left.is_empty() {
            break;
        }
        last = fleet.get(30).map(|a| (a.kin.clone(), fleet.clock.now_s()));
    }
    let (gone, at_s) = left.pop().expect("left the area");
    let (prev, prev_s) = last.unwrap();
    assert!((at_s - prev_s - crate::SUBSTEP_S).abs() < 1e-9);
    let w = Whole::outbound(
        "XNOC",
        here,
        "LFPG",
        End::air(&gone.kin),
        at_s,
        0.0,
        gone.perf,
    )
    .unwrap();
    assert!(w.at(at_s - 1.0).is_none(), "the simulation's until then");
    let fix = w.at(at_s).unwrap();
    assert!(gc_dist_m(fix.pos, gone.kin.pos()) < 0.01);
    assert!((fix.alt_ft - gone.kin.alt_ft).abs() < 1e-6);
    // The model's first second is as long as the simulation's last
    // substep was (scaled), the same way and still climbing.
    let next = w.at(at_s + 1.0).unwrap();
    let sim_ms = gc_dist_m(prev.pos(), gone.kin.pos()) / crate::SUBSTEP_S;
    let model_ms = gc_dist_m(gone.kin.pos(), next.pos);
    assert!((model_ms - sim_ms).abs() < 0.1 * sim_ms, "{model_ms} vs {sim_ms} m/s");
    let turn = crate::kin::signed_diff_deg(next.hdg_deg, gone.kin.hdg_deg).abs();
    assert!(turn < 15.0, "{turn} degrees off its heading");
    assert!(w.at(at_s + 120.0).unwrap().alt_ft > gone.kin.alt_ft);
    assert!(w.on_block_s > at_s + 3600.0, "an hour or more to Paris");
}

/// What one traffic arrival did behind OUR aircraft flown down the final:
/// when we were off the runway again (as our slot predicted it at our
/// touchdown), when it crossed the threshold, its orbits and go-arounds.
#[derive(Debug, Default)]
struct BehindUs {
    own_off_s: f64,
    thr_s: Option<f64>,
    orbits: u32,
    go_arounds: usize,
}

/// Fly our own `ours` down the final of XTST 27 from `own_nm` out,
/// starting `own_at` seconds in, with `traffic` from the start; our slot
/// in the picture when `in_picture`. We fly the type's schedule (an
/// airliner's bands, a light aircraft its approach speed), roll out on the
/// strip and are gone off it when our slot said at touchdown.
fn fly_behind_us(
    field: &FieldModel,
    ours: &str,
    own_nm: f64,
    own_at: f64,
    traffic: TrafficAircraft,
    in_picture: bool,
) -> BehindUs {
    use crate::phase::OwnTarget;
    use crate::slot::{own_slot, OwnArrival};
    let end = field.end("27").unwrap();
    let (perf, _) = crate::perf::class_for_type(ours);
    let id = traffic.id;
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(traffic);
    let tower = RefCell::new(ScriptedTower::default());
    let mut out = BehindUs::default();
    let mut along = -own_nm * 1852.0;
    let mut gs = 0.0;
    let mut off_at: Option<f64> = None;
    let mut t = 0.0;
    while t < 3600.0 {
        let own = if t < own_at || off_at.is_some_and(|o| t >= o) {
            None
        } else {
            let on_ground = along >= 0.0;
            gs = if on_ground {
                (gs - perf.decel_kts_s * 0.1).max(perf.taxi_kts)
            } else if crate::arrival::is_instrument(perf) {
                crate::arrival::scheduled_kts(perf, -along)
            } else {
                perf.app_kts
            };
            let pos = end.at(along, 0.0);
            let alt_ft = if on_ground {
                end.elev_ft
            } else {
                end.elev_ft + 50.0 - along * 3.28084 * 3f64.to_radians().tan()
            };
            let slot = own_slot(
                field,
                &OwnArrival {
                    pos,
                    alt_ft,
                    gs_kts: gs,
                    on_ground,
                    icao_type: ours.into(),
                    runway: "27".into(),
                    join_m: None,
                    cleared: false,
                },
            );
            if on_ground && off_at.is_none() {
                let s = slot.expect("our slot on the runway");
                off_at = Some(t + s.clear_s);
                out.own_off_s = t + s.clear_s;
            }
            along += gs * crate::kin::KT_TO_MS * 0.1;
            Some(OwnTarget {
                pos,
                alt_ft,
                on_ground,
                gs_kts: gs,
                slot: slot.filter(|_| in_picture),
                route: Vec::new(),
            })
        };
        let events = fleet.step(0.1, own, &|i| tower.borrow().orders.get(&i).cloned());
        t += 0.1;
        for e in &events {
            tower.borrow_mut().hear(e);
            if let Event::Call {
                id: i,
                call: Call::GoingAround,
            } = e
            {
                if *i == id {
                    out.go_arounds += 1;
                }
            }
        }
        let Some(ac) = fleet.get(id) else { break };
        out.orbits = out.orbits.max(ac.orbits);
        if out.thr_s.is_none() && !ac.kin.on_ground && end.frame(ac.kin.pos()).0 >= 0.0 {
            out.thr_s = Some(t);
        }
        if out.thr_s.is_some() && off_at.is_some_and(|o| t > o) {
            break;
        }
    }
    out
}

/// Seconds from the start to the threshold for `ac` flown alone.
fn threshold_alone(field: &FieldModel, ac: TrafficAircraft) -> f64 {
    let end = field.end("27").unwrap();
    let id = ac.id;
    let mut fleet = Fleet::new(field.clone());
    fleet.spawn(ac);
    let tower = RefCell::new(ScriptedTower::default());
    let mut t = 0.0;
    while t < 3600.0 {
        for e in fleet.step(0.1, None, &|i| tower.borrow().orders.get(&i).cloned()) {
            tower.borrow_mut().hear(&e);
        }
        t += 0.1;
        let a = fleet.get(id).unwrap();
        if end.frame(a.kin.pos()).0 >= 0.0 {
            return t;
        }
    }
    panic!("never reached the threshold");
}

/// ARRIVAL SEQUENCING (2026-10-14): our own aircraft was never in the
/// traffic's slot picture (`OwnTarget` carried a position for the ground
/// spacing only), so a traffic arrival close behind us on the final flew
/// on into our rollout and the tower sent one of us round. Our slot is in
/// the picture now and the traffic yields to it: a straight-in behind our
/// 8 nm final orbits or slows and reaches the threshold no sooner than
/// [`crate::slot::FOLLOW_GAP_S`] after we are off the runway, with no
/// go-around; without our slot it reached it inside the gap. An airliner a
/// minute behind ours, a light single half a minute behind ours (two
/// minutes is room already by the traffic's own rule: our 737 was off
/// 43 s after touchdown, its wake gap 82 s).
#[test]
fn traffic_behind_our_final_yields_to_us_and_never_lands_on_us() {
    use crate::slot::FOLLOW_GAP_S;
    let field = fixtures::xtst();
    for (ours, theirs, behind_s) in [("B738", "B738", 60.0), ("C172", "PA28", 30.0)] {
        let (perf, _) = crate::perf::class_for_type(ours);
        let traffic = || entering(&field, 7, theirs, true, 16.0, 90.0);
        let alone = threshold_alone(&field, traffic());
        // Our 8 nm final, flown at the type's schedule.
        let own_final_s = if crate::arrival::is_instrument(perf) {
            crate::arrival::scheduled_time_s(perf, 8.0 * 1852.0)
        } else {
            8.0 * 1852.0 / (perf.app_kts * crate::kin::KT_TO_MS)
        };
        // We reach the threshold `behind_s` before it would.
        let own_at = alone - behind_s - own_final_s;
        assert!(own_at > 0.0, "{theirs} alone at {alone:.0} s");
        let with = fly_behind_us(&field, ours, 8.0, own_at, traffic(), true);
        let without = fly_behind_us(&field, ours, 8.0, own_at, traffic(), false);
        println!(
            "{ours} ahead of {theirs}: alone {alone:.0} s, with {with:?}, without {without:?}"
        );
        let thr = with.thr_s.expect("it landed");
        assert_eq!(with.go_arounds, 0, "{ours}/{theirs}: no go-around");
        assert!(
            thr >= with.own_off_s + FOLLOW_GAP_S,
            "{ours}/{theirs}: at the threshold {thr:.0} s, we were off at {:.0} s",
            with.own_off_s
        );
        assert!(
            with.orbits > 0 || thr > alone + 30.0,
            "{ours}/{theirs}: neither orbited nor slowed"
        );
        let blind = without.thr_s.unwrap_or(f64::INFINITY);
        assert!(
            blind < without.own_off_s + FOLLOW_GAP_S,
            "{ours}/{theirs}: the test would pass without our slot ({blind:.0} s)"
        );
    }
}
