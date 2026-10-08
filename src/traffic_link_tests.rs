//! The traffic link's tests, and its HEADLESS HARNESS: the fleet and a
//! real station on a fake clock for a simulated field hour.
//!
//! A station installs process-wide statics (the strips, the field, the
//! runways the book reads), and the crew's 2,000 tests read the same party
//! line, so the harness runs in ITS OWN PROCESS: the plain test re-runs this
//! test binary on the ignored `..._inner` test with `XVL_TRAFFIC_HARNESS`
//! set, and asserts it passed.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::time::{Duration, Instant};

use traffic::{Call, Density, Despawn, Event, FieldModel, Movement, SpawnCtx};

use super::*;
use crate::party_line::Station;

/// The env var an `_inner` harness test needs to run (else it returns at
/// once, so `--ignored` over the whole suite does not install a station
/// among the crew's tests).
const HARNESS_ENV: &str = "XVL_TRAFFIC_HARNESS";

/// The traffic crate's fixture folder.
fn fixtures() -> String {
    format!("{}/crates/traffic/fixtures", env!("CARGO_MANIFEST_DIR"))
}

/// Re-run this test binary on one ignored test, alone, in its own process.
fn in_own_process(test: &str) {
    let out = std::process::Command::new(std::env::current_exe().expect("the test binary"))
        .args([
            test,
            "--exact",
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(HARNESS_ENV, "1")
        .env("XCOPILOT_BLACKBOX", "off")
        .env(
            "XVL_TRAFFIC_TRACE",
            std::env::var("XVL_TRAFFIC_TRACE").unwrap_or_default(),
        )
        .output()
        .expect("run the harness");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    println!("{stdout}");
    assert!(
        out.status.success() && stdout.contains("1 passed"),
        "{test} failed:\n{stdout}\n{stderr}"
    );
}

/// What a harness run measured.
#[derive(Debug, Default)]
pub(super) struct HarnessScore {
    pub(super) movements: usize,
    pub(super) completed: usize,
    pub(super) despawns: BTreeMap<String, usize>,
    pub(super) calls: usize,
    pub(super) readbacks: usize,
    pub(super) readbacks_accepted: usize,
    pub(super) tower_lines: usize,
    pub(super) conflicts: Vec<String>,
    pub(super) oracle: Vec<String>,
    pub(super) unfinished: Vec<String>,
    pub(super) touchdowns: usize,
    pub(super) go_arounds: usize,
    pub(super) secs: f64,
}

/// A station for `icao` from `apt` with its taxi network loaded, installed.
pub(super) fn installed_station(apt: &str, icao: &str) -> Station {
    let st = Station::load(apt, icao).expect("a towered field");
    install(st, apt, icao, true)
}

/// Install `st` and load its taxi network from `apt` (waited for when the
/// field has stands).
fn install(mut st: Station, apt: &str, icao: &str, has_stands: bool) -> Station {
    Station::uninstall();
    st.set_dialect(crate::party_line::dialect_for(icao));
    st.install();
    crate::party_line::spawn_taxi_net_load(apt.to_string(), icao.to_string());
    let t0 = Instant::now();
    while has_stands
        && !crate::party_line::stands_known()
        && t0.elapsed() < Duration::from_secs(120)
    {
        std::thread::sleep(Duration::from_millis(20));
    }
    st
}

/// The install's apt.dat (`XCOPILOT_APT_DAT` overrides).
fn install_apt() -> String {
    std::env::var("XCOPILOT_APT_DAT").unwrap_or_else(|_| {
        crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat")
    })
}

/// Fly `schedule` at `field` against the installed `station` on a fake
/// clock in `dt` ticks until every movement is done or `max_s` passes.
pub(super) fn fly_field(
    station: &mut Station,
    field: FieldModel,
    schedule: Vec<Movement>,
    max_s: f64,
) -> HarnessScore {
    let name = crate::party_line::spoken_airport(station.airport_name()).to_string();
    let mut link = Link::new(field, schedule, &name);
    let mut s = HarnessScore {
        movements: link.pending(),
        ..Default::default()
    };
    let epoch = Instant::now();
    let dt = 0.25;
    let mut t = 0.0;
    // Each readback the fleet gave, by flight: the tower's owed text.
    let mut readbacks: HashMap<u32, Vec<String>> = HashMap::new();
    let mut seen_ids: HashSet<u32> = HashSet::new();
    while t < max_s {
        t += dt;
        let now = epoch + Duration::from_secs_f64(t);
        let out = link.tick(None, dt, now);
        s.tower_lines += out.tower.len();
        for ev in &out.events {
            match ev {
                Event::Spawn { id } => {
                    seen_ids.insert(*id);
                }
                Event::Despawn { id, reason } => {
                    *s.despawns.entry(format!("{reason:?}")).or_default() += 1;
                    if matches!(reason, Despawn::LeftTheArea | Despawn::Parked) {
                        s.completed += 1;
                    } else {
                        s.unfinished.push(format!("{id} {reason:?} at {t:.0} s"));
                    }
                }
                Event::Call { id, call } => {
                    s.calls += 1;
                    if matches!(call, Call::GoingAround) {
                        s.go_arounds += 1;
                    }
                    if let Call::Readback(rb) = call {
                        s.readbacks += 1;
                        readbacks.entry(*id).or_default().push(rb.clone());
                    }
                }
                Event::Touchdown { .. } => s.touchdowns += 1,
                Event::Phase { .. } => {}
            }
        }
        // The command loop's half, as often as the monitor ticks.
        station.sync_traffic_desks();
        let trace = std::env::var("XVL_TRAFFIC_TRACE").is_ok_and(|v| !v.is_empty());
        for line in out.tower.iter().filter(|_| trace) {
            println!("{t:7.1} TWR(tick) [{}] {}", line.0, line.1);
        }
        for call in crate::world::take_traffic_calls() {
            let reply = station.answer_traffic(&call);
            if trace {
                println!(
                    "{t:7.1} [{}] {}\n{t:7.1}   TWR: {}",
                    call.id, call.text, reply.line
                );
            }
            // A readback is accepted when the tower no longer owes it.
            let owed = readbacks.get_mut(&call.id).and_then(|v| {
                let i = v
                    .iter()
                    .position(|rb| call.text.starts_with(rb.trim_end_matches('.')))?;
                Some(v.remove(i))
            });
            if let Some(rb) = owed {
                let still = crate::world::traffic_order(call.id)
                    .and_then(|o| o.readback_owed)
                    .is_some_and(|o| o == rb);
                if still {
                    s.oracle.push(format!(
                        "{t:.0} s readback refused: {:?} -> {:?}",
                        call.text, reply.line
                    ));
                } else {
                    s.readbacks_accepted += 1;
                }
            }
        }
        for c in crate::world::runway_conflicts(None) {
            if s.conflicts.len() < 20 {
                s.conflicts.push(format!("{t:.0} s {c}"));
            }
        }
        for ac in link.fleet().aircraft() {
            if let Some(line) = crate::world::traffic_compliance_oracle(ac.id, now) {
                if s.oracle.len() < 20 {
                    s.oracle
                        .push(format!("{t:.0} s {} {:?}: {line}", ac.callsign, ac.phase));
                }
            }
        }
        if link.pending() == 0 && link.fleet().aircraft().is_empty() {
            break;
        }
    }
    s.secs = t;
    for ac in link.fleet().aircraft() {
        s.unfinished.push(format!(
            "{} {} still {:?} at {t:.0} s",
            ac.id, ac.callsign, ac.phase
        ));
    }
    let _ = seen_ids;
    s
}

fn assert_clean(s: &HarnessScore, what: &str) {
    println!("{what}: {s:#?}");
    assert!(s.movements > 0, "{what}: nothing scheduled");
    assert_eq!(
        s.completed, s.movements,
        "{what}: every movement completes ({:?})",
        s.unfinished
    );
    assert!(s.unfinished.is_empty(), "{what}: {:?}", s.unfinished);
    assert!(
        s.conflicts.is_empty(),
        "{what}: runway conflicts {:?}",
        s.conflicts
    );
    assert!(
        s.oracle.is_empty(),
        "{what}: the oracle spoke {:?}",
        s.oracle
    );
    assert_eq!(
        s.readbacks, s.readbacks_accepted,
        "{what}: every readback accepted"
    );
}

/// The orders map one to one.
#[test]
fn an_order_maps_onto_the_fleet() {
    let o = crate::world::TrafficOrder {
        state: crate::tower::State::ClearedToLand,
        runway: "27".into(),
        taxi_path: vec![],
        holding_point: String::new(),
        to_stand: false,
        sequence: Some(2),
        follow: Some(crate::party_line::FlightId::Traffic(4)),
        hold: false,
        line_up_condition: None,
        go_around: true,
        readback_owed: Some("Going around, Cessna".into()),
    };
    let m = map_order(&o);
    assert_eq!(m.state, traffic::TowerState::ClearedToLand);
    assert_eq!(m.runway.as_deref(), Some("27"));
    assert_eq!(m.taxi_path, None);
    assert_eq!(m.holding_point, None);
    assert_eq!(m.follow, Some(4));
    assert!(m.go_around);
    let ours = crate::world::TrafficOrder {
        follow: Some(crate::party_line::FlightId::Own),
        ..o
    };
    assert_eq!(map_order(&ours).follow, None, "ours is no traffic id");
}

/// A FIELD HOUR: the generated schedule at XTST (circuits and ATR 72
/// departures and arrivals) flown against the real station on a fake
/// clock: every movement completes, the runway book never holds two
/// incompatible claims, the compliance oracle stays silent and every
/// traffic readback is accepted. In its own process (module docs).
#[test]
fn a_field_hour_against_the_real_station() {
    in_own_process("traffic_link::tests::a_field_hour_inner");
}

#[test]
#[ignore = "run in its own process by a_field_hour_against_the_real_station"]
fn a_field_hour_inner() {
    if std::env::var(HARNESS_ENV).is_err() {
        return;
    }
    let apt = format!("{}/xtst.dat", fixtures());
    let field = FieldModel::load(&apt, "XTST")
        .unwrap()
        .unwrap()
        .with_cifp(&format!("{}/cifp", fixtures()));
    let schedule = traffic::generate(&field, Density::Normal, 7, &SpawnCtx::default());
    let mut st = installed_station(&apt, "XTST");
    let s = fly_field(&mut st, field, schedule, 2.5 * 3600.0);
    assert_clean(&s, "XTST field hour");
    Station::uninstall();
}

/// The field hour at the REAL LEMG (no circuits there: IFR both ways, jets
/// on 13), against the real station, as XTST's. Needs the install.
#[test]
#[ignore = "needs X-Plane's Global Airports apt.dat; run with -- --ignored"]
fn real_install_lemg_traffic_hour() {
    in_own_process("traffic_link::tests::real_install_lemg_traffic_hour_inner");
}

#[test]
#[ignore = "run in its own process by real_install_lemg_traffic_hour"]
fn real_install_lemg_traffic_hour_inner() {
    if std::env::var(HARNESS_ENV).is_err() {
        return;
    }
    let apt = install_apt();
    let Ok(Some(field)) = FieldModel::load(&apt, "LEMG") else {
        println!("no install, no verdict");
        return;
    };
    let field = field.with_cifp(&crate::nav::test_xplane_path("Custom Data/CIFP"));
    let schedule = traffic::generate(&field, Density::Normal, 7, &SpawnCtx::default());
    let mut st = installed_station(&apt, "LEMG");
    let s = fly_field(&mut st, field, schedule, 3.0 * 3600.0);
    assert_clean(&s, "LEMG field hour");
    Station::uninstall();
}

/// THE WORLD SWEEP, traffic: a generated schedule at each of the first N
/// towered fields in the install (`XVL_TRAFFIC_SWEEP_N`, default 20),
/// flown for ten minutes against its real station: nothing panics.
/// Completion is not asked (minutes, not the hour); the numbers print.
#[test]
#[ignore = "needs X-Plane's Global Airports apt.dat; run with -- --ignored"]
fn real_install_traffic_world_sweep() {
    in_own_process("traffic_link::tests::real_install_traffic_world_sweep_inner");
}

#[test]
#[ignore = "run in its own process by real_install_traffic_world_sweep"]
fn real_install_traffic_world_sweep_inner() {
    if std::env::var(HARNESS_ENV).is_err() {
        return;
    }
    let apt = install_apt();
    let n: usize = std::env::var("XVL_TRAFFIC_SWEEP_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(20);
    let mut fields: Vec<(String, crate::nav::AirportInfo)> = Vec::new();
    let scanned = crate::nav::scan_airports(&apt, |icao, a| {
        if fields.len() < n && a.freq(crate::nav::FREQ_TOWER).is_some() {
            fields.push((icao.to_string(), a));
        }
    });
    if scanned.is_err() || fields.is_empty() {
        println!("no install, no verdict");
        return;
    }
    let cifp = crate::nav::test_xplane_path("Custom Data/CIFP");
    let (mut flown, mut spawned, mut calls) = (0, 0, 0);
    for (icao, a) in fields {
        let net = crate::nav::find_taxi_net(&apt, &icao).ok().flatten();
        let has_stands = net.as_ref().is_some_and(|n| !n.stands.is_empty());
        let field = FieldModel::new(&icao, a.clone(), net).with_cifp(&cifp);
        let schedule = traffic::generate(&field, Density::Normal, 3, &SpawnCtx::default());
        let Some(st) = Station::from_airport(a) else {
            continue;
        };
        let mut st = install(st, &apt, &icao, has_stands);
        let s = fly_field(&mut st, field, schedule, 600.0);
        println!(
            "{icao}: {} scheduled, {} calls, {} readbacks ({} accepted), {} conflicts, {} oracle",
            s.movements,
            s.calls,
            s.readbacks,
            s.readbacks_accepted,
            s.conflicts.len(),
            s.oracle.len()
        );
        for line in s.conflicts.iter().take(1).chain(s.oracle.iter().take(1)) {
            println!("    {line}");
        }
        flown += 1;
        spawned += s.movements;
        calls += s.calls;
    }
    Station::uninstall();
    println!("swept {flown} fields: {spawned} movements, {calls} calls, no panic");
    assert!(flown > 0);
}
