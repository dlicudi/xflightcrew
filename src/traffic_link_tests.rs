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
        .env(
            "XVL_TRAFFIC_SWEEP_ONLY",
            std::env::var("XVL_TRAFFIC_SWEEP_ONLY").unwrap_or_default(),
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
    /// Every line the tower said: its answers and its runway watch's.
    pub(super) said: Vec<String>,
    /// When each traffic transmission started (harness seconds), and
    /// whether it was a safety call.
    pub(super) keyed: Vec<(f64, bool)>,
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
    let trace = std::env::var("XVL_TRAFFIC_TRACE").is_ok_and(|v| !v.is_empty());
    while t < max_s {
        t += dt;
        let now = epoch + Duration::from_secs_f64(t);
        let out = link.tick(None, dt, now);
        s.tower_lines += out.tower.len();
        s.keyed
            .extend(out.transmissions.iter().map(|tx| (t, tx.safety)));
        s.said.extend(out.tower.iter().map(|(_, l)| l.clone()));
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
                        readbacks.entry(*id).or_default().push(rb.clone());
                    }
                }
                Event::Touchdown { .. } => s.touchdowns += 1,
                Event::Phase { id, from, to } => {
                    if trace {
                        let pos = link.fleet().get(*id).map(|a| a.kin.pos());
                        println!("{t:7.1} [{id}] {from:?} -> {to:?} at {pos:?}");
                    }
                }
            }
        }
        // The command loop's half, as often as the monitor ticks.
        station.sync_traffic_desks();
        for line in out.tower.iter().filter(|_| trace) {
            println!("{t:7.1} TWR(tick) [{}] {}", line.0, line.1);
        }
        for call in crate::world::take_traffic_calls() {
            let reply = station.answer_traffic(&call);
            s.said.push(reply.line.clone());
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
                // Counted as it KEYS: one still waiting its turn on the
                // frequency when the window closes was never heard.
                s.readbacks += 1;
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
                if trace {
                    println!(
                        "ORACLE {t:.1} [{}] {line}\n  at {:?} gs {:.1} hdg {:.0}",
                        ac.id,
                        ac.kin.pos(),
                        ac.kin.gs_kts,
                        ac.kin.hdg_deg
                    );
                }
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
    assert_paced(s, what);
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

/// The frequency was never crowded: no traffic transmission but a safety
/// call started within [`CALL_GAP_S`] of the one before it.
fn assert_paced(s: &HarnessScore, what: &str) {
    if let Some((gap, at)) = crowded(s) {
        panic!("{what}: two transmissions {gap:.2} s apart at {at:.1} s");
    }
}

/// The first transmission (not a safety call) that started within
/// [`CALL_GAP_S`] of the one before it: (the gap, when).
fn crowded(s: &HarnessScore) -> Option<(f64, f64)> {
    s.keyed.windows(2).find_map(|w| {
        let ((a, _), (b, safety)) = (w[0], w[1]);
        (!safety && b - a < CALL_GAP_S - 1e-6).then_some((b - a, b))
    })
}

/// The world sweep's window at each field: ten minutes of fleet time.
const SWEEP_S: f64 = 600.0;

/// What the sweep asserts of a field's window (completion is not asked of
/// ten minutes): the runway book never held two incompatible claims, the
/// compliance oracle stayed silent, every traffic readback was accepted.
fn assert_sweep_clean(s: &HarnessScore, what: &str) {
    assert_paced(s, what);
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
        crossing: Some("13".into()),
    };
    let m = map_order(&o);
    assert_eq!(m.state, traffic::TowerState::ClearedToLand);
    assert_eq!(m.runway.as_deref(), Some("27"));
    assert_eq!(m.taxi_path, None);
    assert_eq!(m.holding_point, None);
    assert_eq!(m.follow, Some(4));
    assert!(m.go_around);
    assert_eq!(m.cross_cleared.as_deref(), Some("13"));
    let ours = crate::world::TrafficOrder {
        follow: Some(crate::party_line::FlightId::Own),
        ..o
    };
    assert_eq!(map_order(&ours).follow, None, "ours is no traffic id");
}

/// THE FREQUENCY'S PACE (the spec's "speech flood" risk): one traffic
/// transmission started per [`CALL_GAP_S`], the rest waiting in the order
/// made (a readback first), a call already waiting not queued twice, a
/// departed flight's calls gone with it, and a safety call (a go-around,
/// its acknowledgement) never held and never dropped.
#[test]
fn the_traffic_frequency_keys_one_call_per_gap_and_never_holds_a_go_around() {
    let mut ob = Outbox::default();
    ob.push(1, Call::Taxi);
    ob.push(2, Call::Ready);
    ob.push(3, Call::Downwind);
    ob.push(2, Call::Ready);
    assert_eq!(ob.waiting(), 3, "the same call is not queued twice");
    assert_eq!(ob.due(0.0), vec![(1, Call::Taxi)]);
    assert!(ob.due(1.0).is_empty(), "inside the gap");
    // A readback made now goes before the older requests.
    ob.push(
        4,
        Call::Readback("Cleared to land runway two seven, N4.".into()),
    );
    assert_eq!(
        ob.due(CALL_GAP_S),
        vec![(
            4,
            Call::Readback("Cleared to land runway two seven, N4.".into())
        )]
    );
    // A go-around keys at once, inside the gap, and the gap runs from it.
    ob.push(5, Call::GoingAround);
    ob.push(6, Call::Readback("Going around, N6.".into()));
    assert!(is_safety_call(&Call::GoingAround));
    assert!(!is_safety_call(&Call::Readback(
        "Cleared to land, N4.".into()
    )));
    assert_eq!(
        ob.due(CALL_GAP_S + 0.5),
        vec![
            (5, Call::GoingAround),
            (6, Call::Readback("Going around, N6.".into()))
        ]
    );
    assert!(
        ob.due(2.0 * CALL_GAP_S).is_empty(),
        "the gap from the go-around"
    );
    // A flight that left takes its calls with it; the rest go in order.
    ob.forget(2);
    assert_eq!(ob.due(2.0 * CALL_GAP_S + 0.5), vec![(3, Call::Downwind)]);
    assert_eq!(ob.waiting(), 0);
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

/// A real field's layout from the fixtures (`crates/traffic/fixtures/<icao>.dat`,
/// cut from the install's apt.dat: runways, frequencies, the taxi network
/// and stands) with the procedure NAMES its CIFP publishes (what the
/// schedule reads), and the station for it, installed.
fn fixture_field(icao: &str, sids: &[&str], approaches: &[&str]) -> (FieldModel, Station, String) {
    let apt = format!("{}/{}.dat", fixtures(), icao.to_ascii_lowercase());
    let mut field = FieldModel::load(&apt, icao).unwrap().unwrap();
    field.procs.sids = sids.iter().map(|s| s.to_string()).collect();
    field.procs.approaches = approaches.iter().map(|s| s.to_string()).collect();
    let st = installed_station(&apt, icao);
    (field, st, apt)
}

/// ULMM's procedure names, as the install's CIFP lists them.
const ULMM_SIDS: [&str; 30] = [
    "ASGO2E", "ASGO2G", "ASGO2J", "ASGO2N", "ASGO2P", "ASGO2S", "EKLE2E", "EKLE2G", "EKLE2J",
    "EKLE2N", "EKLE2P", "EKLE2S", "LOPR2E", "LOPR2G", "LOPR2J", "LOPR2N", "LOPR2P", "LOPR2S",
    "PELO2E", "PELO2G", "PELO2J", "PELO2N", "PELO2P", "PELO2S", "ULME2E", "ULME2G", "ULME2J",
    "ULME2N", "ULME2P", "ULME2S",
];
const ULMM_APPROACHES: [&str; 14] = [
    "I13-Y", "I13-Z", "I31-Y", "I31-Z", "J13", "J31", "N13-X", "N13-Y", "N13-Z", "N31-X", "N31-Y",
    "N31-Z", "R13", "R31",
];

/// THE WORLD SWEEP'S ULMM (2026-10-08): the taxi routes from the north
/// apron reach the 13 holds (Foxtrot, Charlie) ACROSS runway 13/31 itself.
/// The tower cleared the crossing in the taxi clearance ("cross runway one
/// three") and the traffic drove straight over: 20 conflicts (a takeoff on
/// 13 with a flight crossing it) and the oracle's "you are on runway three
/// one without clearance" at the pavement's edge. Now each flight holds
/// short, asks, and is held while the strip is busy. The sweep's own
/// schedule (seed 3) on the real layout, flown to the END (three hours):
/// past the sweep's ten minutes the takeoff hold released a departure onto
/// 13 six seconds after two crossings of it were cleared, the crossers
/// still short of the pavement, until a cleared crossing held the strip in
/// the book.
#[test]
fn ulmm_traffic_holds_short_of_the_runway_it_crosses() {
    in_own_process("traffic_link::tests::ulmm_traffic_holds_short_inner");
}

#[test]
#[ignore = "run in its own process by ulmm_traffic_holds_short_of_the_runway_it_crosses"]
fn ulmm_traffic_holds_short_inner() {
    if std::env::var(HARNESS_ENV).is_err() {
        return;
    }
    let (field, mut st, _) = fixture_field("ULMM", &ULMM_SIDS, &ULMM_APPROACHES);
    let schedule = traffic::generate(&field, Density::Normal, 3, &SpawnCtx::default());
    let s = fly_field(&mut st, field, schedule, 3.0 * 3600.0);
    assert!(
        s.said
            .iter()
            .any(|l| l.contains("hold short runway one three, traffic on the runway")),
        "a crossing was held for the busy strip: {:?}",
        s.said
    );
    assert!(
        s.said
            .iter()
            .any(|l| l.ends_with("cross runway one three.")),
        "and cleared once it was free"
    );
    assert_clean(&s, "ULMM crossings");
    Station::uninstall();
}

/// THE WORLD SWEEP'S BIEG (2026-10-08): an arrival vacated 03 on the far
/// side and its taxi in crossed the strip again ("taxi to parking one,
/// cross runway zero three") while a departure released by the takeoff
/// hold rolled on 03: 20 conflicts. Now the taxi in holds short of 21,
/// asks, and waits for the departure.
#[test]
fn bieg_taxi_in_holds_short_of_the_runway_it_landed_on() {
    in_own_process("traffic_link::tests::bieg_taxi_in_holds_short_inner");
}

#[test]
#[ignore = "run in its own process by bieg_taxi_in_holds_short_of_the_runway_it_landed_on"]
fn bieg_taxi_in_holds_short_inner() {
    if std::env::var(HARNESS_ENV).is_err() {
        return;
    }
    let (field, mut st, _) = fixture_field(
        "BIEG",
        &[
            "BRUS2A", "ELVU2A", "FELI1B", "FELI3A", "VAD1A", "VAD1B", "VAD2A",
        ],
        &["I03", "Q03", "Q21", "R03", "R21"],
    );
    let schedule = traffic::generate(&field, Density::Normal, 3, &SpawnCtx::default());
    let s = fly_field(&mut st, field, schedule, SWEEP_S);
    assert!(
        s.said.iter().any(|l| l.contains("hold short runway")),
        "the taxi in held short: {:?}",
        s.said
    );
    assert_sweep_clean(&s, "BIEG crossings");
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
/// towered fields in the install (`XVL_TRAFFIC_SWEEP_N`, default 150;
/// `XVL_TRAFFIC_SWEEP_ONLY=ULMM,BIEG` for named ones), flown for ten
/// minutes against its real station: nothing panics, and at every field the
/// runway book never holds two incompatible claims, the compliance oracle
/// stays silent, every traffic readback is accepted and the frequency is
/// paced ([`assert_sweep_clean`]). Completion is not asked (minutes, not the
/// hour); the numbers print. Seconds in release.
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
        .unwrap_or(150);
    // `XVL_TRAFFIC_SWEEP_ONLY=ULMM,BIEG` sweeps just those fields.
    let only = std::env::var("XVL_TRAFFIC_SWEEP_ONLY")
        .ok()
        .filter(|v| !v.is_empty());
    let mut fields: Vec<(String, crate::nav::AirportInfo)> = Vec::new();
    let scanned = crate::nav::scan_airports(&apt, |icao, a| {
        let wanted = only
            .as_ref()
            .is_none_or(|o| o.split(',').any(|f| f == icao));
        if wanted && fields.len() < n && a.freq(crate::nav::FREQ_TOWER).is_some() {
            fields.push((icao.to_string(), a));
        }
    });
    if scanned.is_err() || fields.is_empty() {
        println!("no install, no verdict");
        return;
    }
    let cifp = crate::nav::test_xplane_path("Custom Data/CIFP");
    let (mut flown, mut spawned, mut calls) = (0, 0, 0);
    let mut dirty: Vec<String> = Vec::new();
    for (icao, a) in fields {
        let net = crate::nav::find_taxi_net(&apt, &icao).ok().flatten();
        let has_stands = net.as_ref().is_some_and(|n| !n.stands.is_empty());
        let field = FieldModel::new(&icao, a.clone(), net).with_cifp(&cifp);
        let schedule = traffic::generate(&field, Density::Normal, 3, &SpawnCtx::default());
        let Some(st) = Station::from_airport(a) else {
            continue;
        };
        let mut st = install(st, &apt, &icao, has_stands);
        let s = fly_field(&mut st, field, schedule, SWEEP_S);
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
        let clean = s.conflicts.is_empty()
            && s.oracle.is_empty()
            && s.readbacks == s.readbacks_accepted
            && crowded(&s).is_none();
        if !clean {
            dirty.push(format!(
                "{icao}: {:?} {:?} readbacks {}/{} crowded {:?}",
                s.conflicts.first(),
                s.oracle.first(),
                s.readbacks_accepted,
                s.readbacks,
                crowded(&s)
            ));
        }
        flown += 1;
        spawned += s.movements;
        calls += s.calls;
    }
    Station::uninstall();
    println!(
        "swept {flown} fields: {spawned} movements, {calls} calls, no panic, {} dirty",
        dirty.len()
    );
    assert!(flown > 0);
    assert!(dirty.is_empty(), "fields with findings: {dirty:#?}");
}

/// `XCOPILOT_TRAFFIC` unset is `off`, and `off` (or `real`, or ATC off, or
/// no station) flies nothing: the link only ever runs under `ours`.
#[test]
fn traffic_unset_is_off_and_only_ours_flies() {
    assert_eq!(TrafficMode::parse(""), TrafficMode::Off);
    assert_eq!(
        TrafficMode::parse(&crate::config::Config::default().traffic),
        TrafficMode::Off
    );
    assert_eq!(TrafficMode::parse(" Ours "), TrafficMode::Ours);
    assert_eq!(TrafficMode::parse("real"), TrafficMode::Real);
    assert_eq!(TrafficMode::parse("livetraffic"), TrafficMode::Off);
    let lemg = || Some("LEMG".to_string());
    assert_eq!(wanted_field(TrafficMode::Off, true, lemg()), None);
    assert_eq!(wanted_field(TrafficMode::Real, true, lemg()), None);
    assert_eq!(
        wanted_field(TrafficMode::Ours, false, lemg()),
        None,
        "ATC off"
    );
    assert_eq!(
        wanted_field(TrafficMode::Ours, true, None),
        None,
        "no station"
    );
    assert_eq!(wanted_field(TrafficMode::Ours, true, lemg()), lemg());
}

/// The three traffic knobs are in the settings registry, each pinned by its
/// environment variable, so the panel, the terminal and the EFB see them.
#[test]
fn the_traffic_knobs_are_registry_settings() {
    for (key, env) in [
        ("traffic", "XCOPILOT_TRAFFIC"),
        ("traffic_density", "XCOPILOT_TRAFFIC_DENSITY"),
        ("traffic_file", "XCOPILOT_TRAFFIC_FILE"),
    ] {
        let s = crate::settings::ALL
            .iter()
            .find(|s| s.key == key)
            .unwrap_or_else(|| panic!("{key} is not a setting"));
        assert_eq!(s.env, env, "{key}");
    }
    let mode = crate::settings::ALL
        .iter()
        .find(|s| s.key == "traffic")
        .unwrap();
    assert!(
        mode.kind.parse("ours").is_some() && mode.kind.parse("sometimes").is_none(),
        "a closed choice"
    );
}

/// EVERY TCAS READ GOES THROUGH `world::targets`: it returns our own fleet
/// while it flies (the plugin puts the same aircraft in the sim's TCAS, so
/// a read beside it would count each twice). The raw sim read is private to
/// the station's `tcas.rs`; nothing else names it.
#[test]
fn every_tcas_read_goes_through_world_targets() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut hits = Vec::new();
    for dir in ["src", "crates/atc/src"] {
        for f in crate::test_support::rust_files(&root.join(dir)) {
            let rel = f.strip_prefix(root).unwrap_or(&f).display().to_string();
            if rel.ends_with("station/tcas.rs") || rel.ends_with("traffic_link_tests.rs") {
                continue;
            }
            let text = std::fs::read_to_string(&f).expect("read source");
            for (n, line) in text.lines().enumerate() {
                let code = line.split("//").next().unwrap_or("");
                if code.contains("read_tcas_targets") || code.contains("TCAS_FLIGHT_ID") {
                    hits.push(format!("{rel}:{}: {}", n + 1, line.trim()));
                }
            }
        }
    }
    assert!(
        hits.is_empty(),
        "a TCAS read beside world::targets: {hits:#?}"
    );
}

/// THE REINSTALL RULE: an install clears the strips under the fleet, and
/// every aircraft not parked leaves the frequency; the parked stay. In its
/// own process (module docs).
#[test]
fn a_reinstall_sends_the_moving_traffic_off_the_frequency() {
    in_own_process("traffic_link::tests::a_reinstall_inner");
}

#[test]
#[ignore = "run in its own process by a_reinstall_sends_the_moving_traffic_off_the_frequency"]
fn a_reinstall_inner() {
    if std::env::var(HARNESS_ENV).is_err() {
        return;
    }
    let apt = format!("{}/xtst.dat", fixtures());
    let field = FieldModel::load(&apt, "XTST")
        .unwrap()
        .unwrap()
        .with_cifp(&format!("{}/cifp", fixtures()));
    let schedule = traffic::generate(&field, Density::Normal, 7, &SpawnCtx::default());
    let st = installed_station(&apt, "XTST");
    let mut link = Link::new(field, schedule, "Test");
    let epoch = Instant::now();
    let mut t = 0.0;
    let mut st = st;
    // Fly until someone is moving and someone is still parked.
    let mixed = |l: &Link| {
        let a = l.fleet().aircraft();
        a.iter().any(|x| x.phase != traffic::Phase::Parked)
            && a.iter().any(|x| x.phase == traffic::Phase::Parked)
    };
    while t < 1800.0 && !mixed(&link) {
        t += 0.25;
        let _ = link.tick(None, 0.25, epoch + Duration::from_secs_f64(t));
        let _ = drain_calls(&mut st);
    }
    assert!(mixed(&link), "a moving and a parked aircraft by {t:.0} s");
    let moving: Vec<u32> = link
        .fleet()
        .aircraft()
        .iter()
        .filter(|a| a.phase != traffic::Phase::Parked)
        .map(|a| a.id)
        .collect();
    let parked = link.fleet().aircraft().len() - moving.len();
    st.install();
    t += 0.25;
    let out = link.tick(None, 0.25, epoch + Duration::from_secs_f64(t));
    assert_eq!(out.left_frequency, moving, "the moving left the frequency");
    assert_eq!(link.fleet().aircraft().len(), parked, "the parked stayed");
    for ac in link.fleet().aircraft() {
        assert!(
            crate::world::traffic_order(ac.id).is_some(),
            "a parked aircraft's strip is open again"
        );
    }
    t += 0.25;
    let out = link.tick(None, 0.25, epoch + Duration::from_secs_f64(t));
    assert!(out.left_frequency.is_empty(), "said once");
    Station::uninstall();
}
