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
            "XVL_TRAFFIC_SWEEP_S",
            std::env::var("XVL_TRAFFIC_SWEEP_S").unwrap_or_default(),
        )
        .env(
            "XVL_TRAFFIC_SWEEP_ONLY",
            std::env::var("XVL_TRAFFIC_SWEEP_ONLY").unwrap_or_default(),
        )
        .env(
            "XVL_TRAFFIC_SWEEP_S",
            std::env::var("XVL_TRAFFIC_SWEEP_S").unwrap_or_default(),
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
    /// The ground spacing's guard behind an aircraft that would not move:
    /// planned round it, it towed clear, or waiting with neither.
    pub(super) spacing_rerouted: usize,
    pub(super) spacing_cleared: usize,
    pub(super) spacing_held: usize,
    pub(super) secs: f64,
    /// When each movement spawned (harness seconds) and its callsign.
    pub(super) spawned: Vec<(f64, String)>,
    /// Put on their stands parked at the start (parked aircraft and the
    /// departures waiting), and still parked at the end (never moved).
    pub(super) parked_at_start: usize,
    pub(super) parked: usize,
    /// Parked aircraft (no movement) towed off the way of a taxiing one.
    pub(super) parked_towed: usize,
    /// The closest two taxiing aircraft came, one of them moving, as a
    /// share of their spacing (`traffic::spacing::spacing_m`).
    pub(super) closest_share: f64,
    /// Who and where, for that closest pair.
    pub(super) closest_at: String,
    /// Two ground aircraft drawn inside each other: centres closer than a
    /// quarter of their two lengths (one driven through another, or two on
    /// one stand). Each pair once per spell.
    pub(super) overlaps: Vec<String>,
    /// Each full-stop landing's runway time, seconds: from the flare (over
    /// the threshold, near enough) to its "runway vacated".
    pub(super) runway_s: Vec<f64>,
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
/// clock in `dt` ticks until every movement is done or `max_s` passes. The
/// field's stands are parked at the start as the live link parks them
/// (`traffic::park_at_start`, normal density): the departures waiting on
/// their stands are movements, the parked aircraft are not, and they stay.
pub(super) fn fly_field(
    station: &mut Station,
    field: FieldModel,
    mut schedule: Vec<Movement>,
    max_s: f64,
) -> HarnessScore {
    let name = crate::party_line::spoken_airport(station.airport_name()).to_string();
    let parked = traffic::park_at_start(
        &field,
        Density::Normal,
        3,
        &sweep_ctx(&field),
        &mut schedule,
    );
    schedule.extend(parked);
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
    // Every flight given its movement (spawned, or released off its stand).
    let mut flying: HashSet<u32> = HashSet::new();
    let trace = std::env::var("XVL_TRAFFIC_TRACE").is_ok_and(|v| !v.is_empty());
    s.closest_share = f64::INFINITY;
    let mut overlapping: HashSet<(u32, u32)> = HashSet::new();
    // When each flight began its flare, for its runway time.
    let mut flared: HashMap<u32, f64> = HashMap::new();
    // The command loop notes the station's runway each turn, from the
    // install on (`station_work::drain_traffic_calls`).
    note_station_runway(station.runway_for_now().as_deref());
    while t < max_s {
        t += dt;
        let now = epoch + Duration::from_secs_f64(t);
        let out = link.tick(None, dt, now);
        s.tower_lines += out.tower.len();
        s.keyed
            .extend(out.transmissions.iter().map(|tx| (t, tx.safety)));
        s.said.extend(out.tower.iter().map(|(_, l)| l.clone()));
        s.spawned
            .extend(out.spawned.iter().map(|(_, cs)| (t, cs.clone())));
        flying.extend(out.spawned.iter().map(|(id, _)| *id));
        s.parked_at_start += out.parked.len();
        for ev in &out.events {
            match ev {
                Event::Spawn { id } => {
                    seen_ids.insert(*id);
                }
                Event::Despawn { id, reason } => {
                    *s.despawns.entry(format!("{reason:?}")).or_default() += 1;
                    if matches!(reason, Despawn::LeftTheArea | Despawn::Parked) {
                        s.completed += 1;
                    } else if *reason == Despawn::Towed && !flying.contains(id) {
                        // A parked aircraft towed off a stand link it stood
                        // on: no movement of the hour (2026-10-09).
                        s.parked_towed += 1;
                    } else {
                        s.unfinished.push(format!("{id} {reason:?} at {t:.0} s"));
                    }
                }
                Event::Call { id, call } => {
                    s.calls += 1;
                    if matches!(call, Call::GoingAround) {
                        s.go_arounds += 1;
                        flared.remove(id);
                    }
                    if matches!(call, Call::Vacated) {
                        if let Some(t0) = flared.remove(id) {
                            s.runway_s.push(t - t0);
                        }
                    }
                    if let Call::Readback(rb) = call {
                        readbacks.entry(*id).or_default().push(rb.clone());
                    }
                }
                Event::Touchdown { .. } => s.touchdowns += 1,
                Event::SpacingRerouted { .. }
                | Event::SpacingCleared { .. }
                | Event::SpacingHeld { .. } => {
                    match ev {
                        Event::SpacingRerouted { .. } => s.spacing_rerouted += 1,
                        Event::SpacingCleared { .. } => s.spacing_cleared += 1,
                        _ => s.spacing_held += 1,
                    }
                    if trace {
                        println!("{t:7.1} {ev:?}");
                    }
                }
                Event::Phase { id, from, to } => {
                    if *to == traffic::Phase::Landing {
                        flared.insert(*id, t);
                    } else if *from == traffic::Phase::Landing && *to != traffic::Phase::Vacate {
                        // A touch and go, or a go-around from the flare.
                        flared.remove(id);
                    }
                    if trace {
                        let pos = link.fleet().get(*id).map(|a| a.kin.pos());
                        println!("{t:7.1} [{id}] {from:?} -> {to:?} at {pos:?}");
                    }
                }
            }
        }
        // The command loop's half, as often as the monitor ticks.
        note_station_runway(station.runway_for_now().as_deref());
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
        closeness(&link, t, &mut s, &mut overlapping);
        if link.pending() == 0 && link.active() == 0 {
            break;
        }
    }
    s.secs = t;
    s.parked = link
        .fleet()
        .aircraft()
        .iter()
        .filter(|a| a.dormant())
        .count();
    for ac in link.fleet().aircraft().iter().filter(|a| !a.dormant()) {
        s.unfinished.push(format!(
            "{} {} still {:?} at {t:.0} s",
            ac.id, ac.callsign, ac.phase
        ));
    }
    let _ = seen_ids;
    s
}

/// Every pair of aircraft on the ground this tick: the closest two
/// TAXIING ones came against their spacing, and any two drawn inside each
/// other (the head-on pass-through and the shared stand, 2026-10-09).
fn closeness(link: &Link, t: f64, s: &mut HarnessScore, overlapping: &mut HashSet<(u32, u32)>) {
    use traffic::Phase;
    let taxiing = |p: Phase| {
        matches!(
            p,
            Phase::TaxiOut | Phase::TaxiIn | Phase::Vacate | Phase::LineUp | Phase::HoldShort
        )
    };
    let ground: Vec<&traffic::TrafficAircraft> = link
        .fleet()
        .aircraft()
        .iter()
        .filter(|a| a.kin.on_ground && a.phase != Phase::Gone)
        .collect();
    let mut now: HashSet<(u32, u32)> = HashSet::new();
    for (k, a) in ground.iter().enumerate() {
        for b in &ground[k + 1..] {
            let d = traffic::kin::dist_m(a.kin.pos(), b.kin.pos());
            if d > 200.0 {
                continue;
            }
            let moving = a.kin.gs_kts > 0.5 || b.kin.gs_kts > 0.5;
            if taxiing(a.phase) && taxiing(b.phase) && moving {
                let share = d / traffic::spacing::spacing_m(a.perf, b.perf);
                if share < s.closest_share {
                    s.closest_share = share;
                    s.closest_at = format!(
                        "{t:.0} s [{}] {:?} {:.1} kt and [{}] {:?} {:.1} kt {d:.1} m apart at {:?}",
                        a.id,
                        a.phase,
                        a.kin.gs_kts,
                        b.id,
                        b.phase,
                        b.kin.gs_kts,
                        a.kin.pos()
                    );
                }
            }
            if d < (a.perf.length_m + b.perf.length_m) / 4.0 {
                let key = (a.id.min(b.id), a.id.max(b.id));
                now.insert(key);
                if !overlapping.contains(&key) && s.overlaps.len() < 20 {
                    s.overlaps.push(format!(
                        "{t:.0} s [{}] {:?} and [{}] {:?} {d:.1} m apart",
                        a.id, a.phase, b.id, b.phase
                    ));
                }
            }
        }
    }
    *overlapping = now;
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

/// [`assert_clean`], a flight towed clear of the taxiway by the ground
/// spacing's guard ([`Despawn::Towed`]) counted as finished, and nobody
/// ever drawn inside anybody.
fn assert_clean_but_towed(s: &HarnessScore, what: &str) {
    let towed = s.despawns.get("Towed").copied().unwrap_or(0);
    println!("{what}: {towed} towed");
    let unfinished: Vec<&String> = s
        .unfinished
        .iter()
        .filter(|u| !u.contains("Towed"))
        .collect();
    assert!(unfinished.is_empty(), "{what}: {unfinished:?}");
    assert_eq!(
        s.completed + towed,
        s.movements,
        "{what}: every movement completes"
    );
    assert!(s.overlaps.is_empty(), "{what}: {:?}", s.overlaps);
    let clean = HarnessScore {
        completed: s.movements,
        unfinished: Vec::new(),
        conflicts: s.conflicts.clone(),
        oracle: s.oracle.clone(),
        keyed: s.keyed.clone(),
        movements: s.movements,
        readbacks: s.readbacks,
        readbacks_accepted: s.readbacks_accepted,
        ..Default::default()
    };
    assert_clean(&clean, what);
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

/// The mean of `v` (0 for none).
fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        0.0
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}

/// The time of day a harness flies at: `XVL_TRAFFIC_SWEEP_LOCAL_H=8` puts
/// every field at 08:00 its own solar time (`traffic::rate`), so a world
/// sweep is at each field's morning peak, `=3` at each one's night; unset,
/// no clock (the field's peak rate, an even split).
fn sweep_ctx(field: &FieldModel) -> SpawnCtx {
    let zulu_s = std::env::var("XVL_TRAFFIC_SWEEP_LOCAL_H")
        .ok()
        .and_then(|v| v.parse::<f64>().ok())
        .map(|h| traffic::rate::zulu_for_local(h, traffic::rate::field_lon(field)));
    SpawnCtx {
        zulu_s,
        ..SpawnCtx::default()
    }
}

/// The world sweep's window at each field: ten minutes of fleet time.
const SWEEP_S: f64 = 600.0;

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
    // The field has a runway for the traffic from the install on: the first
    // arrival spawns at its time, not ARRIVAL_WAIT_S later for a first call.
    assert!(st.runway_in_use().is_none() && st.runway_for_now().is_some());
    let first_in = schedule
        .iter()
        .find(|m| m.kind == traffic::MovementKind::Arrival)
        .map(|m| (m.t_s, m.callsign.clone()))
        .expect("an arrival");
    let s = fly_field(&mut st, field, schedule, 2.5 * 3600.0);
    assert_clean(&s, "XTST field hour");
    let spawned = s.spawned.iter().find(|(_, cs)| *cs == first_in.1);
    assert!(
        spawned.is_some_and(|(t, _)| *t < first_in.0 + 1.0),
        "the first arrival spawned at {spawned:?}, scheduled {:.0} s",
        first_in.0
    );
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
/// the book. The departures held short of 13 there wait for the crossing an
/// hour and more, on the one way off 13 to the north apron: an arrival
/// vacating that way once drove THROUGH them; now it stops, finds no other
/// way, and the holder is towed clear (2026-10-09), so a towed flight
/// counts as finished here.
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
    assert_clean_but_towed(&s, "ULMM crossings");
    Station::uninstall();
}

/// THE WORLD SWEEP'S BIEG (2026-10-08): an arrival vacated 03 on the far
/// side and its taxi in crossed the strip again ("taxi to parking one,
/// cross runway zero three") while a departure released by the takeoff
/// hold rolled on 03: 20 conflicts. Now the taxi in holds short of 21,
/// asks, and waits for the departure. Flown to the END of its schedule
/// (three hours): a 172 landing short rolled 1.2 km to the far exit at taxi
/// pace and despawned `NoStand` the moment the vacate ended, its "runway
/// vacated" never said and the strip held behind it, until the wait for a
/// stand ran from the vacate's end (#242). Then the far-side vacate itself
/// went: the apron's one exit is behind a landing on 03, so the rollout
/// turns round and backtracks to it (`FieldModel::vacate`), and the taxi in
/// crosses nothing at all.
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
    let s = fly_field(&mut st, field, schedule, 3.0 * 3600.0);
    let taxi_in: Vec<&String> = s
        .said
        .iter()
        .filter(|l| l.contains("taxi to parking"))
        .collect();
    assert!(taxi_in.len() >= 3, "arrivals taxied in: {taxi_in:?}");
    assert!(
        taxi_in
            .iter()
            .all(|l| !l.contains("runway zero three") && !l.contains("runway two one")),
        "backtracked to the apron's exit, nothing to cross: {taxi_in:?}"
    );
    assert_clean(&s, "BIEG crossings");
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
    println!(
        "LEMG at install: runway in use {:?}, for the traffic {:?}",
        st.runway_in_use(),
        st.runway_for_now()
    );
    let s = fly_field(&mut st, field, schedule, 3.0 * 3600.0);
    println!(
        "LEMG first spawns: {:?}, {} parked at the start, {} never moved",
        &s.spawned[..s.spawned.len().min(4)],
        s.parked_at_start,
        s.parked
    );
    assert!(s.parked > 0, "LEMG's stands are not empty");
    assert_clean(&s, "LEMG field hour");
    Station::uninstall();
}

/// THE WORLD SWEEP, traffic: a generated schedule at each of the first N
/// towered fields in the install (`XVL_TRAFFIC_SWEEP_N`, default 150;
/// `XVL_TRAFFIC_SWEEP_ONLY=ULMM,BIEG` for named ones), flown for ten
/// minutes against its real station: nothing panics, and at every field the
/// runway book never holds two incompatible claims, the compliance oracle
/// stays silent, every traffic readback is accepted and the frequency is
/// paced (`assert_clean` less completion). Completion is not asked (minutes, not the
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
    // `XVL_TRAFFIC_SWEEP_S=3600` flies each field for the hour instead.
    let window_s: f64 = std::env::var("XVL_TRAFFIC_SWEEP_S")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(SWEEP_S);
    let (mut flown, mut spawned, mut calls, mut completed) = (0, 0, 0, 0);
    let (mut overlaps, mut closest, mut go_arounds) = (0, f64::INFINITY, 0);
    let mut dirty: Vec<String> = Vec::new();
    let mut runway_s: Vec<f64> = Vec::new();
    for (icao, a) in fields {
        let net = crate::nav::find_taxi_net(&apt, &icao).ok().flatten();
        let has_stands = net.as_ref().is_some_and(|n| !n.stands.is_empty());
        let field = FieldModel::new(&icao, a.clone(), net).with_cifp(&cifp);
        let schedule = traffic::generate(&field, Density::Normal, 3, &sweep_ctx(&field));
        let Some(st) = Station::from_airport(a) else {
            continue;
        };
        let mut st = install(st, &apt, &icao, has_stands);
        let peak = traffic::rate::peak_per_h(&field);
        let s = fly_field(&mut st, field, schedule, window_s);
        print!("{icao} peak {peak:.0}/h, ");
        println!(
            "{icao}: {} scheduled, {} completed, {} calls, {} readbacks ({} accepted), \
             {} conflicts, {} oracle, spacing {}/{}/{} rerouted/towed/held, closest {:.2} of the spacing, \
             {} overlaps, {} go-arounds, runway {:.0} s over {} landings",
            s.movements,
            s.completed,
            s.calls,
            s.readbacks,
            s.readbacks_accepted,
            s.conflicts.len(),
            s.oracle.len(),
            s.spacing_rerouted,
            s.spacing_cleared,
            s.spacing_held,
            s.closest_share,
            s.overlaps.len(),
            s.go_arounds,
            mean(&s.runway_s),
            s.runway_s.len()
        );
        for line in s
            .conflicts
            .iter()
            .take(1)
            .chain(s.oracle.iter().take(1))
            .chain(s.overlaps.iter().take(2))
            .chain(std::iter::once(&s.closest_at).filter(|_| s.closest_share < 0.7))
        {
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
        overlaps += s.overlaps.len();
        go_arounds += s.go_arounds;
        runway_s.extend(&s.runway_s);
        closest = f64::min(closest, s.closest_share);
        spawned += s.movements;
        completed += s.completed;
        calls += s.calls;
    }
    Station::uninstall();
    println!(
        "swept {flown} fields: {spawned} movements, {completed} completed, {calls} calls, \
         no panic, {} dirty; {overlaps} overlaps, {go_arounds} go-arounds, closest taxiing \
         pair {closest:.2} of its spacing; runway time {:.0} s over {} landings",
        dirty.len(),
        mean(&runway_s),
        runway_s.len()
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
