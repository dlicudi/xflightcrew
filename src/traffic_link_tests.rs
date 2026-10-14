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
    /// Every traffic exchange as heard: the position that heard the call,
    /// the call as addressed to it, and the answer.
    pub(super) heard: Vec<(crate::voice::StationSpeaker, String, String)>,
    /// The aerodrome positions' handoff lines (`world::traffic_position_tick`).
    pub(super) position_lines: usize,
    /// When each traffic transmission started (fleet seconds), whether
    /// it was a safety call, the frequency it was keyed on, and when that
    /// frequency was reckoned clear after it (fleet seconds).
    pub(super) keyed: Vec<(f64, bool, crate::voice::StationSpeaker, f64)>,
    /// How long each call (not a safety call) waited for a clear
    /// frequency, fleet seconds (#260: the radio's pace costs time).
    pub(super) waits: Vec<f64>,
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
    /// Every aircraft seen, spawned or parked, by its airline's letters
    /// (`GA` for a registration) and type: the field's carrier and fleet mix.
    pub(super) mix: BTreeMap<String, usize>,
    /// Departures released off a stand they stood on from the start, off
    /// a stand an arrival of the hour turned round on, and spawned there
    /// from nothing.
    pub(super) dep_parked: usize,
    pub(super) dep_turned: usize,
    pub(super) dep_fresh: usize,
    /// Arrivals that turned round on their stand (the rest lingered and
    /// went: `despawns["Parked"]`).
    pub(super) turned: usize,
    /// Each full-stop landing's runway time, seconds: from the flare (over
    /// the threshold, near enough) to its "runway vacated".
    pub(super) runway_s: Vec<f64>,
    /// The spacing on final, nm: as each landing begins its flare (over
    /// the threshold, near enough), how far along its own approach the
    /// next arrival for that threshold is. Read for the wake spacing
    /// (3 to 5 nm on a real ILS).
    pub(super) final_gap_nm: Vec<f64>,
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
    schedule: Vec<Movement>,
    max_s: f64,
) -> HarnessScore {
    fly_field_hours(station, field, schedule, max_s, false)
}

/// The day the timetable fields fly in a harness: a summer Tuesday.
fn sweep_date() -> traffic::rotation::Date {
    traffic::rotation::Date::ymd(2026, 7, 14)
}

/// A timetable field's start as the live link builds it
/// (`timetable_start`), at the sweep's local hour (else 10:00) on
/// [`sweep_date`]: the schedule (placed flights, the parked fill) and the
/// board's flights.
fn timetable_schedule(
    field: &FieldModel,
    table: &traffic::routes::FieldRoutes,
) -> (Vec<Movement>, Vec<traffic::board::BoardFlight>, f64) {
    let local_h = std::env::var("XVL_TRAFFIC_SWEEP_LOCAL_H")
        .ok()
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(10.0);
    let date = sweep_date();
    let dst = if date.summer() { 1.0 } else { 0.0 };
    let zulu = (local_h - table.utc_offset_h - dst).rem_euclid(24.0) * 3600.0;
    timetable_start(
        field,
        table,
        Density::Normal,
        sweep_seed(),
        Some(zulu),
        Some(date.ordinal),
        None,
    )
}

/// [`fly_field`], and with `more_hours` the next generated hours too while
/// `max_s` lasts, as the live link flies them (the world sweep's longer
/// windows: the turnarounds leave in them).
pub(super) fn fly_field_hours(
    station: &mut Station,
    field: FieldModel,
    schedule: Vec<Movement>,
    max_s: f64,
    more_hours: bool,
) -> HarnessScore {
    fly_fields(station, field, schedule, max_s, more_hours, &mut [])
}

/// [`fly_field_hours`] with other fields flown at once on their own desks
/// (multi-field traffic stage 3, 2026-10-13), ids from one counter: every
/// tick each desk's field steps beside the station's, and no id is ever
/// two fields'.
pub(super) fn fly_fields(
    station: &mut Station,
    field: FieldModel,
    mut schedule: Vec<Movement>,
    max_s: f64,
    more_hours: bool,
    desks: &mut [DeskRun],
) -> HarnessScore {
    let name = crate::party_line::spoken_airport(station.airport_name()).to_string();
    // A timetable's schedule comes with its own stands filled
    // (`timetable_start`).
    let timetable = schedule.iter().any(|m| m.flight.is_some());
    if !timetable {
        let parked = traffic::park_at_start(
            &field,
            Density::Normal,
            sweep_seed(),
            &sweep_ctx(&field),
            &mut schedule,
        );
        schedule.extend(parked);
    }
    let zulu0 = sweep_ctx(&field).zulu_s;
    let mut hours: u32 = 1;
    let mut link = Link::new(field, schedule, &name);
    let ids = Ids::default();
    link.share_ids(&ids);
    for d in desks.iter_mut() {
        d.link.share_ids(&ids);
    }
    // Which field (0 the station's, k + 1 the k-th desk's) each id flew at.
    let mut owner: HashMap<u32, usize> = HashMap::new();
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
    // Who stood parked from the start, and who came in as an arrival: where
    // a departure's airframe came from.
    let mut parked_ids: HashSet<u32> = HashSet::new();
    let mut arrived: HashSet<u32> = HashSet::new();
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
        let fleet_s = link.fleet().clock.now_s();
        s.keyed.extend(
            out.transmissions
                .iter()
                .map(|tx| (fleet_s, tx.safety, tx.on, tx.clear_s)),
        );
        s.waits.extend(
            out.transmissions
                .iter()
                .filter(|tx| !tx.safety)
                .map(|tx| tx.waited_s),
        );
        s.said.extend(out.tower.iter().map(|(_, l)| l.clone()));
        s.said.extend(out.positions.iter().map(|(_, l, _)| l.clone()));
        s.position_lines += out.positions.len();
        s.spawned
            .extend(out.spawned.iter().map(|(_, cs)| (t, cs.clone())));
        for (id, _) in out.parked.iter().chain(&out.spawned) {
            let Some(ac) = link.fleet().get(*id) else {
                continue;
            };
            if trace {
                println!(
                    "{t:7.1} [{id}] {} {} {:?} on {:?}",
                    ac.callsign, ac.icao_type, ac.plan, ac.stand
                );
            }
            if !seen_ids.contains(id) {
                let entry = traffic::wire::roster_entry(ac);
                let code = if entry.airline.is_empty() {
                    "GA".to_string()
                } else {
                    entry.airline
                };
                *s.mix.entry(format!("{code} {}", ac.icao_type)).or_default() += 1;
            }
            seen_ids.insert(*id);
            match ac.plan {
                traffic::Plan::Arrival { .. } => {
                    arrived.insert(*id);
                }
                traffic::Plan::Departure { .. } if out.spawned.iter().any(|(i, _)| i == id) => {
                    if arrived.contains(id) {
                        s.dep_turned += 1;
                    } else if parked_ids.contains(id) {
                        s.dep_parked += 1;
                    } else {
                        s.dep_fresh += 1;
                    }
                }
                _ => {}
            }
        }
        parked_ids.extend(out.parked.iter().map(|(id, _)| *id));
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
                // An arrival parked and turned round on its stand: its
                // flight is done (it lingered and went, before 2026-10-10).
                Event::TurnedRound { .. } => {
                    s.completed += 1;
                    s.turned += 1;
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
                        if let Some(gap) = final_gap_nm(&link, *id) {
                            if trace {
                                println!("{t:7.1} [{id}] flare, next arrival {gap:.2} nm behind");
                            }
                            s.final_gap_nm.push(gap);
                        }
                    } else if *from == traffic::Phase::Landing && *to != traffic::Phase::Vacate {
                        // A touch and go, or a go-around from the flare.
                        flared.remove(id);
                    }
                    if trace {
                        let pos = link.fleet().get(*id).map(|a| a.kin.pos());
                        println!("{t:7.1} [{id}] {from:?} -> {to:?} at {pos:?}");
                        // Who stands near where a vacate or a landing begins
                        // (what a vacate is planned round).
                        if matches!(to, traffic::Phase::Vacate | traffic::Phase::TaxiIn) {
                            for o in link.fleet().aircraft().iter().filter(|o| {
                                o.id != *id
                                    && o.kin.on_ground
                                    && pos.is_some_and(|p| {
                                        traffic::kin::dist_m(p, o.kin.pos()) < 600.0
                                    })
                            }) {
                                println!(
                                    "          near: [{}] {} {:?} at {:?}",
                                    o.id,
                                    o.icao_type,
                                    o.phase,
                                    o.kin.pos()
                                );
                            }
                        }
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
        for line in out.positions.iter().filter(|_| trace) {
            println!("{t:7.1} {}(tick) [{}] {}", line.2.tag(), line.0, line.1);
        }
        for call in crate::world::take_traffic_calls() {
            let reply = station.answer_traffic(&call);
            s.said.push(reply.line.clone());
            s.heard.push((reply.speaker, reply.call.clone(), reply.line.clone()));
            if trace {
                println!(
                    "{t:7.1} [{}] {}\n{t:7.1}   {}: {}",
                    call.id,
                    reply.call,
                    reply.speaker.tag(),
                    reply.line
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
        // `XVL_TRAFFIC_WATCH=17,19` prints those flights every 5 s.
        if let Ok(w) = std::env::var("XVL_TRAFFIC_WATCH") {
            if (t % 5.0) < dt {
                for id in w.split(',').filter_map(|x| x.trim().parse::<u32>().ok()) {
                    if let Some(a) = link.fleet().get(id) {
                        println!(
                            "WATCH {t:7.1} [{id}] {:?} {:?} {:.1} kt hdg {:.0} free {:?} route {} seg {} done {} end {:?}",
                            a.phase,
                            a.kin.pos(),
                            a.kin.gs_kts,
                            a.kin.hdg_deg,
                            a.spacing.free_m,
                            a.route.path.len(),
                            a.route.follow.seg,
                            a.route.follow.done,
                            a.route.path.last()
                        );
                        for o in link.fleet().aircraft().iter().filter(|o| {
                            o.id != id
                                && o.kin.on_ground
                                && traffic::kin::dist_m(o.kin.pos(), a.kin.pos()) < 150.0
                        }) {
                            println!(
                                "      by [{}] {} {:?} {:.0} m at {:?}",
                                o.id,
                                o.icao_type,
                                o.phase,
                                traffic::kin::dist_m(o.kin.pos(), a.kin.pos()),
                                o.kin.pos()
                            );
                        }
                    }
                }
            }
        }
        closeness(&link, t, &mut s, &mut overlapping);
        for (k, d) in desks.iter_mut().enumerate() {
            d.tick(t, dt, now);
            for a in d.link.fleet().aircraft() {
                let was = *owner.entry(a.id).or_insert(k + 1);
                assert_eq!(was, k + 1, "id {} flew at two fields", a.id);
            }
        }
        for a in link.fleet().aircraft() {
            let was = *owner.entry(a.id).or_insert(0);
            assert_eq!(was, 0, "id {} flew at two fields", a.id);
        }
        // A window past the hour flies the next generated hours as the live
        // link does (`TrafficWorld::tick`): the turnarounds leave in them.
        let horizon = traffic::schedule::HORIZON_S;
        if link.pending() == 0
            && t >= f64::from(hours) * horizon
            && more_hours
            && t + horizon / 2.0 < max_s
        {
            hours += 1;
            let zulu = zulu0.map(|z| (z + t) % 86_400.0);
            let ms = next_hour(
                link.fleet(),
                Density::Normal,
                u64::from(hours) + 2,
                None,
                zulu,
            );
            s.movements += ms.len();
            link.add_schedule(ms, link.fleet().clock.now_s());
        }
        let desks_done = desks
            .iter()
            .all(|d| d.link.pending() == 0 && d.link.active() == 0);
        if link.pending() == 0
            && link.active() == 0
            && desks_done
            && (!more_hours || t + horizon / 2.0 >= max_s)
        {
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

/// A field's mix in one line: each carrier's share of every aircraft
/// seen, then its types, the biggest first.
fn mix_line(mix: &BTreeMap<String, usize>) -> String {
    let total: usize = mix.values().sum();
    let mut by_code: BTreeMap<&str, (usize, Vec<(&str, usize)>)> = BTreeMap::new();
    for (k, n) in mix {
        let (code, ty) = k.split_once(' ').unwrap_or((k, ""));
        let e = by_code.entry(code).or_default();
        e.0 += n;
        e.1.push((ty, *n));
    }
    let mut rows: Vec<_> = by_code.into_iter().collect();
    rows.sort_by(|a, b| b.1 .0.cmp(&a.1 .0));
    let parts: Vec<String> = rows
        .into_iter()
        .map(|(code, (n, mut types))| {
            types.sort_by(|a, b| b.1.cmp(&a.1));
            let ts: Vec<String> = types.iter().map(|(t, k)| format!("{t} {k}")).collect();
            format!(
                "{code} {:.0}% ({})",
                100.0 * n as f64 / total.max(1) as f64,
                ts.join(", ")
            )
        })
        .collect();
    format!("{total} aircraft: {}", parts.join("; "))
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
/// call keyed before the exchange ahead of it had cleared and the least
/// gap ([`KEY_GAP_S`]) had run.
fn assert_paced(s: &HarnessScore, what: &str) {
    if let Some((gap, at)) = crowded(s) {
        panic!("{what}: two transmissions {gap:.2} s apart at {at:.1} s");
    }
}

/// The first transmission (not a safety call) keyed ON ITS FREQUENCY
/// before the exchange ahead of it (the call, the controller's beat and
/// answer) had cleared and [`KEY_GAP_S`]'s least gap had run: (its gap
/// after the clear, when). Two positions are two frequencies: Ground's taxi
/// call and the Tower's landing clearance may overlap, as they do.
fn crowded(s: &HarnessScore) -> Option<(f64, f64)> {
    let mut clear: Vec<(crate::voice::StationSpeaker, f64)> = Vec::new();
    for &(b, safety, on, until) in &s.keyed {
        if let Some(e) = clear.iter_mut().find(|(o, _)| *o == on) {
            let a = e.1;
            e.1 = a.max(until);
            if !safety && b - a < KEY_GAP_S.0 - 1e-6 {
                return Some((b - a, b));
            }
        } else {
            clear.push((on, until));
        }
    }
    None
}

/// The mean of `v` (0 for none).
/// How far behind `lander` (just flaring) the next arrival for its
/// threshold is, nm along that arrival's own route; `None` with nobody
/// behind on an approach.
fn final_gap_nm(link: &Link, lander: u32) -> Option<f64> {
    let fleet = link.fleet();
    let thr_of =
        |a: &traffic::TrafficAircraft| a.route.thr_idx.and_then(|i| a.route.path.get(i).copied());
    let thr = thr_of(fleet.get(lander)?)?;
    fleet
        .aircraft()
        .iter()
        .filter(|a| a.id != lander)
        .filter(|a| matches!(a.phase, traffic::Phase::Final | traffic::Phase::Arrive))
        .filter(|a| matches!(a.plan, traffic::Plan::Arrival { .. }))
        .filter_map(|a| {
            let i = a.route.thr_idx?;
            let t = thr_of(a)?;
            (traffic::kin::dist_m(t, thr) < 150.0).then(|| a.route.dist_to(a.kin.pos(), i) / 1852.0)
        })
        .min_by(f64::total_cmp)
}

/// The least and the median of `v` (0 when empty).
fn min_median(v: &[f64]) -> (f64, f64) {
    let mut v = v.to_vec();
    v.sort_by(f64::total_cmp);
    match v.len() {
        0 => (0.0, 0.0),
        n => (v[0], v[n / 2]),
    }
}

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

/// The sweep's schedule seed: `XVL_TRAFFIC_SWEEP_SEED`, else 3 (the one
/// every recorded sweep flew).
fn sweep_seed() -> u64 {
    std::env::var("XVL_TRAFFIC_SWEEP_SEED")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(3)
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
        push: Some(Some(270.0)),
    };
    let m = map_order(&o);
    assert_eq!(m.state, traffic::TowerState::ClearedToLand);
    assert_eq!(m.runway.as_deref(), Some("27"));
    assert_eq!(m.taxi_path, None);
    assert_eq!(m.holding_point, None);
    assert_eq!(m.follow, Some(4));
    assert!(m.go_around);
    assert_eq!(m.cross_cleared.as_deref(), Some("13"));
    assert!(m.push_approved);
    assert_eq!(m.push_facing, Some(270.0));
    let ours = crate::world::TrafficOrder {
        follow: Some(crate::party_line::FlightId::Own),
        ..o
    };
    assert_eq!(map_order(&ours).follow, None, "ours is no traffic id");
}

/// THE FREQUENCY'S TURNS (#260): a call keys only once the exchange ahead
/// of it has cleared (its airtime, the controller's 1 to 3 s beat and
/// answer) and its own 1 to 4 s gap has run; the rest wait in the order
/// made (a readback first); a call already waiting is not queued twice, a
/// departed flight's calls go with it, and a safety call (a go-around, its
/// acknowledgement) is never held.
#[test]
fn the_traffic_frequency_keys_only_once_it_is_clear_and_never_holds_a_go_around() {
    let mut ob = Outbox::default();
    ob.push(1, Call::Taxi);
    ob.push(2, Call::Ready);
    ob.push(3, Call::Downwind);
    ob.push(2, Call::Ready);
    assert_eq!(ob.waiting(), 3, "the same call is not queued twice");
    assert_eq!(ob.due(0.0), vec![(1, Call::Taxi)]);
    let tower = crate::voice::StationSpeaker::Tower;
    let clear = ob.clear_at(tower).unwrap();
    // "Tower, November one two three, request taxi": 6 words, then the
    // beat and the reckoned answer.
    let call = airtime_s("Tower, November one two three, request taxi");
    assert!(
        clear >= call + REPLY_BEAT_S.0 + 4.0 && clear <= call + REPLY_BEAT_S.1 + 6.0,
        "{clear}"
    );
    let mut t = 0.25;
    while t < clear + KEY_GAP_S.0 {
        assert!(ob.due(t).is_empty(), "keyed into the exchange at {t}");
        t += 0.25;
    }
    // A readback made now goes before the older requests, once clear.
    ob.push(
        4,
        Call::Readback("Cleared to land runway two seven, N4.".into()),
    );
    let mut keyed = Vec::new();
    while keyed.is_empty() {
        keyed = ob.due(t);
        t += 0.25;
    }
    assert!(t - 0.25 <= clear + KEY_GAP_S.1 + 0.25, "{t}");
    assert_eq!(
        keyed,
        vec![(
            4,
            Call::Readback("Cleared to land runway two seven, N4.".into())
        )]
    );
    // A readback is answered with silence: the frequency clears after its
    // own airtime.
    let after_rb = ob.clear_at(tower).unwrap();
    assert!(after_rb - (t - 0.25) < 6.0, "{after_rb} {t}");
    // A go-around keys at once, inside the exchange, and the next caller
    // waits it out.
    ob.push(5, Call::GoingAround);
    ob.push(6, Call::Readback("Going around, N6.".into()));
    assert!(is_safety_call(&Call::GoingAround));
    assert!(!is_safety_call(&Call::Readback(
        "Cleared to land, N4.".into()
    )));
    assert_eq!(
        ob.due(t),
        vec![
            (5, Call::GoingAround),
            (6, Call::Readback("Going around, N6.".into()))
        ]
    );
    let after_ga = ob.clear_at(tower).unwrap();
    assert!(after_ga > t + 4.0, "the go-around's exchange: {after_ga}");
    assert!(ob.due(after_ga).is_empty(), "the gap after the go-around");
    // A flight that left takes its calls with it; the rest go in order.
    ob.forget(2);
    assert_eq!(ob.due(after_ga + KEY_GAP_S.1), vec![(3, Call::Downwind)]);
    assert_eq!(ob.waiting(), 0);
}

/// TWO CALLS NEVER OVERLAP ON ONE FREQUENCY: a crowd of calls on the
/// Tower and on Ground, stepped at the link's tick: on each frequency every
/// call keys after the exchange ahead has cleared and at least the least
/// gap, never more than the longest gap later while one waits; the two
/// frequencies never wait on each other.
#[test]
fn two_traffic_calls_never_overlap_on_one_frequency() {
    use crate::voice::StationSpeaker as P;
    let mut ob = Outbox::default();
    for id in 0..8u32 {
        ob.push_on(
            id,
            Call::Inbound {
                miles: 8,
                from: "south".into(),
            },
            P::Tower,
            0.0,
        );
        ob.push_on(100 + id, Call::Taxi, P::Ground, 0.0);
    }
    let mut keyed: Vec<(f64, Keyed)> = Vec::new();
    let mut t = 0.0;
    while t < 400.0 {
        let air = |_: u32, c: &Call| airtime_s(&c.text("Speedbird one two three", None));
        for k in ob.due_on(t, air) {
            keyed.push((t, k));
        }
        t += 0.25;
    }
    assert_eq!(keyed.len(), 16, "everyone keyed");
    for on in [P::Tower, P::Ground] {
        let mine: Vec<&(f64, Keyed)> = keyed.iter().filter(|(_, k)| k.on == on).collect();
        assert!(mine[0].0 < 0.25, "{on:?} keys at once");
        for w in mine.windows(2) {
            let (prev, next) = (&w[0].1, w[1].0);
            assert!(
                next >= prev.clear_s + KEY_GAP_S.0 - 1e-9,
                "{on:?}: keyed at {next} into an exchange clear at {}",
                prev.clear_s
            );
            assert!(
                next <= prev.clear_s + KEY_GAP_S.1 + 0.25 + 1e-9,
                "{on:?}: waited past its gap: {next} after {}",
                prev.clear_s
            );
        }
    }
    // Ground's first taxi did not wait on the Tower's first inbound.
    let first = |on: P| keyed.iter().find(|(_, k)| k.on == on).map(|(t, _)| *t);
    assert_eq!(first(P::Tower), first(P::Ground));
}

/// OUR PILOT IS NEVER STEPPED ON: while our own transmission is on the
/// frequency (the link notes it every tick), no traffic call keys there;
/// after it the tower's beat and a gap run first. Another position's
/// traffic keys as before.
#[test]
fn a_traffic_call_waits_out_our_pilots_transmission() {
    use crate::voice::StationSpeaker as P;
    let mut ob = Outbox::default();
    let air = |_: u32, c: &Call| airtime_s(&c.text("November one", None));
    let mut t = 0.0;
    let mut keyed: Vec<(f64, P)> = Vec::new();
    while t < 60.0 {
        // The pilot keys on the Tower from 2 to 9 s.
        if (2.0..9.0).contains(&t) {
            ob.occupy(P::Tower, t + OWN_TAIL_S);
        }
        if (t - 3.0).abs() < 1e-9 {
            ob.push_on(1, Call::Final, P::Tower, t);
            ob.push_on(2, Call::Taxi, P::Ground, t);
        }
        keyed.extend(ob.due_on(t, air).into_iter().map(|k| (t, k.on)));
        t += 0.25;
    }
    let at = |on: P| {
        keyed
            .iter()
            .find(|(_, o)| *o == on)
            .map(|(t, _)| *t)
            .unwrap()
    };
    // The last tick held was 8.75 s: clear at 11.75 s, then its gap.
    assert!(at(P::Tower) >= 8.75 + OWN_TAIL_S, "{}", at(P::Tower));
    assert!(
        at(P::Tower) <= 8.75 + OWN_TAIL_S + KEY_GAP_S.1,
        "{}",
        at(P::Tower)
    );
    assert_eq!(at(P::Ground), 3.0, "Ground's frequency was clear");
}

/// A QUEUED LINE OVERTAKEN BY EVENTS (#260, t=1791477548): only a
/// withdrawn clearance overtakes a line; the strip moving on, or closing,
/// does not; a go-around order is never overtaken by its own go-around.
#[test]
fn a_line_is_overtaken_only_by_a_withdrawn_clearance() {
    use crate::tower::State as S;
    let mark = |state, runway: &str, go_around| StripMark {
        state,
        runway: runway.into(),
        go_around,
    };
    let landing = mark(S::ClearedToLand, "27", false);
    assert_eq!(
        overtaken(&landing, Some(&mark(S::Inbound, "27", false))),
        Some("went_around")
    );
    assert_eq!(
        overtaken(&landing, Some(&mark(S::ClearedToLand, "27", true))),
        Some("go_around_ordered")
    );
    assert_eq!(overtaken(&landing, Some(&landing)), None);
    assert_eq!(
        overtaken(&landing, Some(&mark(S::Parked, "27", false))),
        None,
        "landed and taxiing in"
    );
    assert_eq!(overtaken(&landing, None), None, "handed off and gone");
    let takeoff = mark(S::TakeoffCleared, "27", false);
    assert_eq!(
        overtaken(&takeoff, Some(&mark(S::LinedUp, "27", false))),
        Some("takeoff_cancelled")
    );
    let lined = mark(S::LinedUp, "27", false);
    assert_eq!(overtaken(&lined, Some(&takeoff)), None, "moving on");
    assert_eq!(
        overtaken(&lined, Some(&mark(S::TaxiCleared, "27", false))),
        Some("line_up_cancelled")
    );
    let taxi = mark(S::TaxiCleared, "27", false);
    assert_eq!(
        overtaken(&taxi, Some(&mark(S::TaxiCleared, "09", false))),
        Some("runway_changed")
    );
    // The order to go around, aired after the flight acknowledged it.
    let order = mark(S::ClearedToLand, "27", true);
    assert_eq!(
        overtaken(&order, Some(&mark(S::Inbound, "27", false))),
        None
    );
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
    let mut deps = (0, 0, 0);
    let mut runway_s: Vec<f64> = Vec::new();
    let mut final_gaps: Vec<f64> = Vec::new();
    for (icao, a) in fields {
        let net = crate::nav::find_taxi_net(&apt, &icao).ok().flatten();
        let has_stands = net.as_ref().is_some_and(|n| !n.stands.is_empty());
        let field = FieldModel::new(&icao, a.clone(), net).with_cifp(&cifp);
        // A field with a route table starts mid-flow on its timetable, as
        // the live link does (2026-10-13); the rest generate their hour.
        let schedule = match traffic::routes::field_routes(&icao) {
            Some(t) => timetable_schedule(&field, t).0,
            None => traffic::generate(&field, Density::Normal, sweep_seed(), &sweep_ctx(&field)),
        };
        let Some(st) = Station::from_airport(a) else {
            continue;
        };
        let mut st = install(st, &apt, &icao, has_stands);
        let peak = traffic::rate::peak_per_h(&field);
        let more = window_s > traffic::schedule::HORIZON_S;
        let s = fly_field_hours(&mut st, field, schedule, window_s, more);
        print!("{icao} peak {peak:.0}/h, ");
        println!(
            "{icao}: {} scheduled, {} completed, {} calls, {} readbacks ({} accepted), \
             {} conflicts, {} oracle, spacing {}/{}/{} rerouted/towed/held, closest {:.2} of the spacing, \
             {} overlaps, {} go-arounds, runway {:.0} s over {} landings, \
             final gap {:.1}/{:.1} nm min/median over {}, \
             calls waited {:.1}/{:.1} s median/max",
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
            s.runway_s.len(),
            min_median(&s.final_gap_nm).0,
            min_median(&s.final_gap_nm).1,
            s.final_gap_nm.len(),
            min_median(&s.waits).1,
            s.waits.iter().copied().fold(0.0, f64::max)
        );
        println!(
            "    departures: {} off a stand parked from the start, {} turned round, {} from nothing; \
             arrivals: {} turned round, {} lingered and went; {} on stands at the start",
            s.dep_parked,
            s.dep_turned,
            s.dep_fresh,
            s.turned,
            s.despawns.get("Parked").copied().unwrap_or(0),
            s.parked_at_start
        );
        if only.is_some() || ["EGKK", "EGLL", "LXGB", "LEMG"].contains(&icao.as_str()) {
            println!("    mix: {}", mix_line(&s.mix));
        }
        deps.0 += s.dep_parked;
        deps.1 += s.dep_turned;
        deps.2 += s.dep_fresh;
        for line in s
            .conflicts
            .iter()
            .take(1)
            .chain(s.oracle.iter().take(1))
            .chain(s.overlaps.iter().take(if only.is_some() { 20 } else { 2 }))
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
        final_gaps.extend(&s.final_gap_nm);
        closest = f64::min(closest, s.closest_share);
        spawned += s.movements;
        completed += s.completed;
        calls += s.calls;
    }
    Station::uninstall();
    println!(
        "swept {flown} fields: {spawned} movements, {completed} completed, {calls} calls, \
         no panic, {} dirty; {overlaps} overlaps, {go_arounds} go-arounds, closest taxiing \
         pair {closest:.2} of its spacing; runway time {:.0} s over {} landings; \
         final gap {:.1}/{:.1} nm min/median over {}",
        dirty.len(),
        mean(&runway_s),
        runway_s.len(),
        min_median(&final_gaps).0,
        min_median(&final_gaps).1,
        final_gaps.len()
    );
    println!(
        "departures: {} off a stand parked from the start, {} turned round, {} from nothing",
        deps.0, deps.1, deps.2
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
    // A dormant one (turned round on its stand) has said nothing and holds
    // no strip; every other parked one's reopens.
    for ac in link.fleet().aircraft().iter().filter(|a| !a.dormant()) {
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

/// A NEW FLIGHT OF OURS AT THE FIELD UP is no reinstall: the station
/// resets our part in place (`Station::new_flight_here`), the strips stand,
/// and nobody leaves the frequency. LEMG 2026-10-13: an EFB "Move here"
/// rebuilt the station and thirteen moving aircraft were despawned. In its
/// own process (module docs).
#[test]
fn a_new_flight_of_ours_keeps_the_moving_traffic() {
    in_own_process("traffic_link::tests::a_new_flight_of_ours_inner");
}

#[test]
#[ignore = "run in its own process by a_new_flight_of_ours_keeps_the_moving_traffic"]
fn a_new_flight_of_ours_inner() {
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
    let mut link = Link::new(field, schedule, "Test");
    let epoch = Instant::now();
    let mut t = 0.0;
    let moving = |l: &Link| {
        l.fleet()
            .aircraft()
            .iter()
            .filter(|x| x.phase != traffic::Phase::Parked)
            .map(|x| x.id)
            .collect::<Vec<u32>>()
    };
    while t < 1800.0 && moving(&link).is_empty() {
        t += 0.25;
        let _ = link.tick(None, 0.25, epoch + Duration::from_secs_f64(t));
        let _ = drain_calls(&mut st);
    }
    let before = moving(&link);
    assert!(!before.is_empty(), "someone moving by {t:.0} s");
    let fleet = link.fleet().aircraft().len();
    // The crew's door, then the main loop's: the same field is up.
    crate::party_line::retire_for_new_flight();
    st.new_flight_here();
    for _ in 0..4 {
        t += 0.25;
        let out = link.tick(None, 0.25, epoch + Duration::from_secs_f64(t));
        assert!(out.left_frequency.is_empty(), "nobody left the frequency");
    }
    assert!(link.fleet().aircraft().len() >= fleet, "nobody was taken out");
    for id in &before {
        assert!(link.fleet().get(*id).is_some(), "{id} still flies");
    }
    Station::uninstall();
}

/// Every carrier our traffic flies (`traffic::airlines`) has its telephony
/// in the station's table, the same words: the tower reads a call's
/// telephony to know an airline (its go-around is the missed approach,
/// never the circuit), and a carrier missing there was a tail number to it.
#[test]
fn every_traffic_carrier_has_its_telephony_at_the_station() {
    for c in traffic::airlines::all_carriers() {
        let raw = format!("{}123", c.code);
        let spoken = crate::callsign::spoken_id_with(&raw, &[]);
        assert!(
            spoken.starts_with(c.telephony),
            "{}: the station says {spoken:?}, the traffic {:?}",
            c.code,
            c.telephony
        );
        assert!(
            crate::callsign::is_airline_telephony_with(
                &format!("{} one two three", c.telephony),
                &[]
            ),
            "{}",
            c.code
        );
    }
}

/// THE MID-FLOW START against the real station (2026-10-13): LXGB and LEMG
/// started at 10:00 local on a summer Tuesday on their timetables, flown
/// for half an hour. At the first tick nothing is on the runway, the
/// arrivals placed on the approach are spaced on it, the ground movers
/// keep apart; through the half hour the book never holds two claims, the
/// oracle stays silent and every readback is taken; the board has both
/// halves and statuses from the start.
#[test]
#[ignore = "needs X-Plane's Global Airports apt.dat; run with -- --ignored"]
fn real_install_midflow_start() {
    in_own_process("traffic_link::tests::real_install_midflow_start_inner");
}

#[test]
#[ignore = "run in its own process by real_install_midflow_start"]
fn real_install_midflow_start_inner() {
    if std::env::var(HARNESS_ENV).is_err() {
        return;
    }
    let apt = install_apt();
    let cifp = crate::nav::test_xplane_path("Custom Data/CIFP");
    for icao in ["LXGB", "LEMG"] {
        let Ok(Some(field)) = FieldModel::load(&apt, icao) else {
            println!("no install, no verdict");
            return;
        };
        let field = field.with_cifp(&cifp);
        let table = traffic::routes::field_routes(icao).expect("a table");
        let st = installed_station(&apt, icao);
        note_station_runway(st.runway_for_now().as_deref());
        let runway = st.runway_for_now();
        let (schedule, board, local_s) = {
            let (s, b, l) = timetable_schedule(&field, table);
            // The placement on the station's runway, as the live link
            // reads it before the first call.
            let _ = &runway;
            (s, b, l)
        };
        let placed: Vec<String> = schedule
            .iter()
            .filter(|m| !matches!(m.start, Some(traffic::Start::Away { .. })))
            .filter_map(|m| m.start.as_ref().map(|s| format!("{} {s:?}", m.callsign)))
            .collect();
        println!(
            "{icao} at {}: {} placed: {placed:#?}",
            traffic::rotation::hhmm(local_s),
            placed.len()
        );
        // At the first tick: the picture.
        let mut link = Link::new(field.clone(), schedule.clone(), "");
        link.set_board(board.clone(), local_s);
        let epoch = Instant::now();
        let _ = link.tick(None, 0.25, epoch);
        let fleet = link.fleet();
        let on_runway: Vec<&str> = fleet
            .aircraft()
            .iter()
            .filter(|a| a.kin.on_ground && fleet.field.on_runway(a.kin.pos()))
            .map(|a| a.callsign.as_str())
            .collect();
        assert!(
            on_runway.is_empty(),
            "{icao}: on the runway at the start {on_runway:?}"
        );
        let payload = link.board_payload(None, &[]);
        println!("{icao} board:\n{payload}");
        // Every departure shows its stand (2026-10-13: Boarding with an
        // empty Stand column), named as the field numbers it.
        for l in payload.lines().filter(|l| l.starts_with("D|")) {
            let f: Vec<&str> = l.split('|').collect();
            assert!(!f[7].is_empty(), "{icao}: no stand on {l}");
            assert!(!f[7].starts_with("Gate "), "{icao}: {l}");
        }
        assert!(
            payload.lines().any(|l| l.starts_with("D|")),
            "{icao} departures"
        );
        assert!(
            payload.lines().any(|l| l.starts_with("A|")),
            "{icao} arrivals"
        );
        drop(link);
        Station::uninstall();
        let mut st = installed_station(&apt, icao);
        let s = fly_field(&mut st, field, schedule, 1800.0);
        println!(
            "{icao} mid-flow: {} scheduled, {} completed, {} calls, {} readbacks ({} accepted), \
             {} conflicts, {} oracle, spacing {}/{}/{} rerouted/towed/held, {} overlaps, \
             {} go-arounds, final gap {:.1}/{:.1} nm min/median over {}; mix {}",
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
            s.overlaps.len(),
            s.go_arounds,
            min_median(&s.final_gap_nm).0,
            min_median(&s.final_gap_nm).1,
            s.final_gap_nm.len(),
            mix_line(&s.mix)
        );
        for l in s
            .conflicts
            .iter()
            .chain(&s.oracle)
            .chain(&s.overlaps)
            .take(10)
        {
            println!("    {l}");
        }
        assert!(s.conflicts.is_empty(), "{icao}: {:?}", s.conflicts);
        assert!(s.oracle.is_empty(), "{icao}: {:?}", s.oracle);
        assert_eq!(s.readbacks, s.readbacks_accepted, "{icao}: readbacks");
        assert_paced(&s, icao);
        let _ = st;
        Station::uninstall();
    }
}

/// THE BOARD'S FIELD SELECTOR (2026-10-13): the field the traffic runs at
/// first, then every other timetable field within 100 nm, nearest first;
/// a pick shows only one of the others, any other pick (none, the active
/// one, a field out of range) the active board.
#[test]
fn the_board_offers_the_timetable_fields_in_range() {
    let lemg = (36.67, -4.50);
    assert_eq!(board_choices("LEMG", lemg), ["LEMG", "LXGB"]);
    assert_eq!(board_choices("lxgb", (36.15, -5.35)), ["LXGB", "LEMG"]);
    // A field with no timetable still comes first: its board is the live one.
    assert_eq!(
        board_choices("LEZL", (37.42, -5.89)),
        ["LEZL", "LEMG", "LXGB"]
    );
    assert_eq!(board_choices("EGKK", (51.15, -0.19)), ["EGKK"]);
    let c = board_choices("LEMG", lemg);
    assert_eq!(board_field(Some("lxgb"), &c), Some("LXGB"));
    assert_eq!(board_field(Some("LEMG"), &c), None);
    assert_eq!(board_field(Some(""), &c), None);
    assert_eq!(board_field(Some("EGKK"), &c), None);
    assert_eq!(board_field(None, &c), None);
}

/// A field nobody simulates has its board from its timetable, by the
/// clock: both halves, a departure inside 40 minutes Boarding at its
/// planned stand, statuses moving with the sim's time, the selector's
/// fields on its header.
#[test]
fn a_board_for_a_field_nobody_flies() {
    use traffic::routes::{FieldRoutes, RouteRow};
    let gibraltar = traffic::routes::field_routes("LXGB").unwrap().rows[0];
    let rows: &'static [RouteRow] = Box::leak(Box::new([RouteRow {
        summer: 42,
        winter: 42,
        windows: &[(9.0, 12.0)],
        based: false,
        types: &[("A320", 100)],
        ..gibraltar
    }]));
    let table = FieldRoutes {
        icao: "ULMM",
        utc_offset_h: 3.0,
        city: "Ulmm",
        top_up: false,
        rows,
    };
    let apt = format!("{}/ulmm.dat", fixtures());
    let field = FieldModel::load(&apt, "ULMM").unwrap().unwrap();
    let date = traffic::rotation::Date::ymd(2026, 7, 14);
    let b = RemoteBoard::from_field(&field, &table, Density::Normal, date, 9.5 * 3600.0, 100.0);
    let picks = ["LEMG".to_string(), "ULMM".to_string()];
    let p = b.payload(100.0, &picks);
    assert!(p.starts_with("H|ULMM|09:30|LEMG,ULMM\n"), "{p}");
    let rows: Vec<Vec<&str>> = p.lines().skip(1).map(|l| l.split('|').collect()).collect();
    assert!(rows.iter().any(|r| r[0] == "D") && rows.iter().any(|r| r[0] == "A"));
    for r in &rows {
        if r[0] == "D" {
            assert!(!r[7].is_empty(), "a departure's stand: {r:?}");
        }
        if r[8] == "Boarding" {
            assert_eq!(r[0], "D");
        }
    }
    // An hour on, the clock has moved the statuses on.
    let later = b.payload(100.0 + 3600.0, &picks);
    assert!(later.starts_with("H|ULMM|10:30|"), "{later}");
    assert_ne!(p, later);
}

/// A test timetable at ULMM (its real layout from the fixtures): one A320
/// route flown from 06:00 to 22:00.
fn ulmm_table() -> traffic::routes::FieldRoutes {
    use traffic::routes::{FieldRoutes, RouteRow};
    let gibraltar = traffic::routes::field_routes("LXGB").unwrap().rows[0];
    let rows: &'static [RouteRow] = Box::leak(Box::new([RouteRow {
        summer: 42,
        winter: 42,
        windows: &[(6.0, 22.0)],
        based: false,
        types: &[("A320", 100)],
        ..gibraltar
    }]));
    FieldRoutes {
        icao: "ULMM",
        utc_offset_h: 3.0,
        city: "Ulmm",
        top_up: false,
        rows,
    }
}

/// A TIMETABLE FOLLOWS THE SIM'S CLOCK (#280, LXGB 2026-10-14): planned
/// at 23:00 local it holds only tomorrow's flights; moved to 10:00 the same
/// day, the re-plan's next flight comes within the hour's spacing, the
/// board's clock reads the new time, and the next re-plan sees no jump.
#[test]
fn a_timetable_follows_a_jump_of_the_sims_clock() {
    let table = ulmm_table();
    let apt = format!("{}/ulmm.dat", fixtures());
    let field = FieldModel::load(&apt, "ULMM").unwrap().unwrap();
    let date = traffic::rotation::Date::ymd(2026, 7, 14);
    // UTC+3 in summer is +4: 23:00 local is 19:00 zulu, 10:00 is 06:00.
    let (late, board, local_s) = timetable_start(
        &field,
        &table,
        Density::Normal,
        7,
        Some(19.0 * 3600.0),
        Some(date.ordinal),
        None,
    );
    let mut link = Link::new(field, late, "");
    link.set_board(board, local_s);
    link.timetable = Some(Timetable { density: Density::Normal, seed: 7, date });
    let first = |l: &Link| l.due.iter().map(|d| d.at_s).fold(f64::INFINITY, f64::min);
    assert!(first(&link) > 6.0 * 3600.0, "at 23:00 only tomorrow's: {}", first(&link));

    let (now_date, _, now_local) = timetable_clock(&table, Some(6.0 * 3600.0), Some(date.ordinal));
    let jump = clock_jump_s(date, link.local_now_s(), now_date, now_local);
    assert!((jump + 13.0 * 3600.0).abs() < 1.0, "13 h back: {jump}");

    let n = link.reanchor(&table, 6.0 * 3600.0, Some(date.ordinal), None).unwrap();
    assert!(n > 0 && link.pending() >= n);
    assert!(first(&link) < 3.0 * 3600.0, "the morning's flights: {}", first(&link));
    assert!((link.local_now_s() - 10.0 * 3600.0).abs() < 1.0);
    let again = clock_jump_s(link.timetable.unwrap().date, link.local_now_s(), now_date, now_local);
    assert!(again.abs() < CLOCK_JUMP_S, "no jump after the re-plan: {again}");
}

/// The jump across midnight counts the days: 23:50 to 00:10 the next day
/// is 20 minutes, never a day back.
#[test]
fn a_clock_jump_counts_the_days() {
    let d = traffic::rotation::Date::ymd(2026, 10, 13);
    let j = clock_jump_s(d, 23.0 * 3600.0 + 50.0 * 60.0, d.plus(1), 10.0 * 60.0);
    assert!((j - 1200.0).abs() < 1e-6, "{j}");
}

/// ONE COUNTER, NO CLASHES (multi-field traffic stage 2, 2026-10-13): two
/// fleets sharing the world's ids hand out every id once, whichever takes
/// first; a counter of its own still starts at 1.
#[test]
fn every_fleet_takes_its_ids_from_one_counter() {
    let ids = Ids::default();
    let other = ids.clone();
    assert_eq!(ids.peek(), 1);
    let mut seen = HashSet::new();
    for k in 0..50 {
        let c = if k % 3 == 0 { &ids } else { &other };
        assert!(seen.insert(c.peek()), "id {} twice", c.peek());
        c.bump();
    }
    assert_eq!(ids.peek(), 51);
    assert_eq!(Ids::default().peek(), 1);
}

/// A FIELD'S DAY IS ITS OWN (stage 2): the timetable start at a field with
/// its seed (`traffic::field_seed`) is the same day however often it is
/// built, and the board by the clock of that field (no fleet) shows the
/// same flights as the fleet's own board.
#[test]
fn a_fields_day_is_the_same_whoever_builds_it() {
    let table = ulmm_table();
    let apt = format!("{}/ulmm.dat", fixtures());
    let field = FieldModel::load(&apt, "ULMM").unwrap().unwrap();
    let date = traffic::rotation::Date::ymd(2026, 7, 14);
    let seed = traffic::field_seed("ULMM", SESSION_SEED);
    // 10:00 local in summer at UTC+3: 06:00 zulu.
    let zulu = Some(6.0 * 3600.0);
    let start = |s: u64| {
        timetable_start(
            &field,
            &table,
            Density::Normal,
            s,
            zulu,
            Some(date.ordinal),
            None,
        )
    };
    let (a, board_a, local_a) = start(seed);
    let (b, board_b, _) = start(seed);
    let key = |v: &[Movement]| {
        v.iter()
            .map(|m| (m.callsign.clone(), m.t_s.to_bits()))
            .collect::<Vec<_>>()
    };
    assert!(!board_a.is_empty());
    assert_eq!(key(&a), key(&b), "the same field, the same day");
    assert_eq!(board_a, board_b);
    let remote = RemoteBoard::from_field(&field, &table, Density::Normal, date, local_a, 0.0);
    assert_eq!(
        remote.flights, board_a,
        "the board by the clock is the fleet's"
    );
}

/// THE FIELDS IN RANGE (multi-field traffic stage 3, 2026-10-13): the
/// station's field always; LXGB and LEMG together from either field or
/// from between them (54 nm apart); a timetable field joins inside 100 nm
/// and leaves only past 120 nm; a field far from both flies alone.
#[test]
fn the_fields_in_range_join_and_leave_with_hysteresis() {
    let none = BTreeSet::new();
    let both: BTreeSet<String> = ["LEMG", "LXGB"].map(String::from).into();
    let lemg = traffic::places::position("LEMG").unwrap();
    let lxgb = traffic::places::position("LXGB").unwrap();
    assert_eq!(wanted_fields("lemg", lemg, &none), both);
    assert_eq!(wanted_fields("LXGB", lxgb, &none), both);
    let between = ((lemg.0 + lxgb.0) / 2.0, (lemg.1 + lxgb.1) / 2.0);
    assert_eq!(wanted_fields("LEMG", between, &none), both);
    assert_eq!(
        wanted_fields("EGKK", (51.15, -0.19), &none),
        BTreeSet::from(["EGKK".to_string()])
    );
    // West of Gibraltar along its parallel, past Malaga's reach.
    let west = |nm: f64| (lxgb.0, lxgb.1 - nm / (60.0 * lxgb.0.to_radians().cos()));
    let at = |nm: f64, running: &BTreeSet<String>| wanted_fields("LPFR", west(nm), running);
    assert!(traffic::places::dist_nm(west(110.0), lemg) > SIM_OUT_NM);
    let gib = BTreeSet::from(["LXGB".to_string()]);
    let alone = BTreeSet::from(["LPFR".to_string()]);
    assert!(at(95.0, &none).contains("LXGB"), "joins inside 100 nm");
    assert_eq!(at(110.0, &none), alone, "not joined at 110 nm");
    assert!(at(110.0, &gib).contains("LXGB"), "kept to 120 nm");
    assert_eq!(at(125.0, &gib), alone, "retired past 120 nm");
}

/// ONE BUILD AT A TIME, THE STATION'S FIRST (stage 3), then the others in
/// ICAO order; nothing once every field is built or failed.
#[test]
fn the_stations_field_is_built_first() {
    let wanted: BTreeSet<String> = ["LEMG", "LXGB"].map(String::from).into();
    let built = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<BTreeSet<_>>();
    assert_eq!(next_build("LXGB", &wanted, &built(&[])).as_deref(), Some("LXGB"));
    assert_eq!(next_build("LXGB", &wanted, &built(&["LXGB"])).as_deref(), Some("LEMG"));
    assert_eq!(next_build("LXGB", &wanted, &built(&["LXGB", "LEMG"])), None);
}

/// The runway conflicts on a desk's field this tick: an aircraft rolling
/// or landing (on the ground, or low over the strip) and another on the
/// ground on the same strip. Each pair (lower id first).
fn desk_conflicts(link: &Link) -> Vec<(u32, u32)> {
    let f = &link.fleet().field;
    let acs: Vec<&traffic::TrafficAircraft> = link
        .fleet()
        .aircraft()
        .iter()
        .filter(|a| !a.dormant())
        .collect();
    let mut out = Vec::new();
    for a in &acs {
        let using = matches!(a.phase, traffic::Phase::Roll | traffic::Phase::Landing)
            && (a.kin.on_ground || a.agl_ft(f) < 100.0);
        let Some((strip, _)) = f.strip_at(a.kin.pos()).filter(|_| using) else {
            continue;
        };
        for b in &acs {
            let on = b.kin.on_ground && f.strip_at(b.kin.pos()).is_some_and(|(i, _)| i == strip);
            if b.id != a.id && on {
                out.push((a.id.min(b.id), a.id.max(b.id)));
            }
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// A field flown by its desk in a harness: the link, what it measured,
/// the pairs drawn inside each other and in conflict as of last tick.
pub(super) struct DeskRun {
    pub(super) link: Link,
    pub(super) s: HarnessScore,
    overlapping: HashSet<(u32, u32)>,
    conflicting: HashSet<(u32, u32)>,
}

impl DeskRun {
    pub(super) fn new(link: Link) -> DeskRun {
        let s = HarnessScore {
            movements: link.pending(),
            closest_share: f64::INFINITY,
            ..Default::default()
        };
        DeskRun {
            link,
            s,
            overlapping: HashSet::new(),
            conflicting: HashSet::new(),
        }
    }

    /// One tick at harness second `t`: the fleet stepped by its desk, its
    /// events counted, closeness and runway conflicts measured.
    pub(super) fn tick(&mut self, t: f64, dt: f64, now: Instant) {
        let out = self.link.tick(None, dt, now);
        let s = &mut self.s;
        s.parked_at_start += out.parked.len();
        for ev in &out.events {
            match ev {
                Event::Despawn { reason, .. } => {
                    *s.despawns.entry(format!("{reason:?}")).or_default() += 1;
                    match reason {
                        Despawn::LeftTheArea | Despawn::Parked => s.completed += 1,
                        Despawn::Towed => {}
                        other => s.unfinished.push(format!("{other:?} at {t:.0} s")),
                    }
                }
                Event::TurnedRound { .. } => {
                    s.completed += 1;
                    s.turned += 1;
                }
                Event::Call { call, .. } => {
                    s.calls += 1;
                    if matches!(call, Call::GoingAround) {
                        s.go_arounds += 1;
                    }
                }
                Event::Touchdown { .. } => s.touchdowns += 1,
                Event::SpacingRerouted { .. } => s.spacing_rerouted += 1,
                Event::SpacingCleared { .. } => s.spacing_cleared += 1,
                Event::SpacingHeld { .. } => s.spacing_held += 1,
                _ => {}
            }
        }
        closeness(&self.link, t, &mut self.s, &mut self.overlapping);
        let now_pairs: HashSet<(u32, u32)> = desk_conflicts(&self.link).into_iter().collect();
        for p in now_pairs.difference(&self.conflicting) {
            if self.s.conflicts.len() < 20 {
                self.s.conflicts.push(format!("{t:.0} s [{}] and [{}]", p.0, p.1));
            }
        }
        self.conflicting = now_pairs;
        self.s.secs = t;
    }

    /// The towed count, as the sweeps print it.
    pub(super) fn towed(&self) -> usize {
        self.s.despawns.get("Towed").copied().unwrap_or(0)
    }
}

/// TWO FIELDS AT ONCE ON THEIR DESKS (stage 3): XTST's and ULMM's
/// generated hours flown together, each by its own runway desk, ids from
/// one counter: no id is ever both fields', every movement completes (a
/// tow counts), nobody is on a runway somebody is rolling or landing on,
/// nobody is drawn inside anybody, and both fields took off and landed.
/// Pure: a desk never reaches the station.
#[test]
fn two_fields_fly_at_once_on_their_desks() {
    let ids = Ids::default();
    let mut runs: Vec<DeskRun> = [("XTST", 7u64), ("ULMM", 3)]
        .iter()
        .map(|(icao, seed)| {
            let apt = format!("{}/{}.dat", fixtures(), icao.to_ascii_lowercase());
            let field = FieldModel::load(&apt, icao)
                .unwrap()
                .unwrap()
                .with_cifp(&format!("{}/cifp", fixtures()));
            let schedule = traffic::generate(&field, Density::Normal, *seed, &SpawnCtx::default());
            let mut link = Link::new(field, schedule, "");
            link.share_ids(&ids);
            link.make_desk();
            assert!(link.is_desk());
            DeskRun::new(link)
        })
        .collect();
    let mut owner: HashMap<u32, usize> = HashMap::new();
    let epoch = Instant::now();
    let dt = 0.25;
    let mut t = 0.0;
    while t < 2.5 * 3600.0 {
        t += dt;
        let now = epoch + Duration::from_secs_f64(t);
        for (k, run) in runs.iter_mut().enumerate() {
            run.tick(t, dt, now);
            for a in run.link.fleet().aircraft() {
                let was = *owner.entry(a.id).or_insert(k);
                assert_eq!(was, k, "id {} flew at both fields", a.id);
            }
        }
        if runs.iter().all(|r| r.link.pending() == 0 && r.link.active() == 0) {
            break;
        }
    }
    for (run, icao) in runs.iter().zip(["XTST", "ULMM"]) {
        let s = &run.s;
        println!(
            "{icao} on its desk: {} scheduled, {} completed, {} towed, {} touchdowns, \
             {} go-arounds, {} conflicts, {} overlaps",
            s.movements,
            s.completed,
            run.towed(),
            s.touchdowns,
            s.go_arounds,
            s.conflicts.len(),
            s.overlaps.len()
        );
        assert!(s.conflicts.is_empty(), "{icao}: {:?}", s.conflicts);
        assert!(s.overlaps.is_empty(), "{icao}: {:?}", s.overlaps);
        assert!(s.unfinished.is_empty(), "{icao}: {:?}", s.unfinished);
        assert!(s.touchdowns > 0, "{icao}: nobody landed");
        assert_eq!(
            s.completed + run.towed(),
            s.movements,
            "{icao}: every movement completes"
        );
        assert_eq!(run.link.active(), 0, "{icao}: still flying");
    }
}

/// LEMG AND LXGB AT ONCE (multi-field traffic stage 3, 2026-10-13): our
/// station at LEMG works LEMG's timetable as it always did, and LXGB's
/// timetable flies beside it on its own desk, both from their mid-flow
/// start at 10:00 on a summer Tuesday, half an hour
/// (`XVL_TRAFFIC_SWEEP_S`). LEMG is clean as the mid-flow test asks (the
/// book, the oracle, the readbacks, the pace); LXGB has nobody on a runway
/// somebody rolls or lands on and nobody drawn inside anybody; no id is
/// ever both fields'. The counts print for each field.
#[test]
#[ignore = "needs X-Plane's Global Airports apt.dat; run with -- --ignored"]
fn real_install_two_fields_at_once() {
    in_own_process("traffic_link::tests::real_install_two_fields_at_once_inner");
}

#[test]
#[ignore = "run in its own process by real_install_two_fields_at_once"]
fn real_install_two_fields_at_once_inner() {
    if std::env::var(HARNESS_ENV).is_err() {
        return;
    }
    let apt = install_apt();
    let cifp = crate::nav::test_xplane_path("Custom Data/CIFP");
    let load = |icao: &str| {
        FieldModel::load(&apt, icao)
            .ok()
            .flatten()
            .map(|f| f.with_cifp(&cifp))
    };
    let (Some(lemg), Some(lxgb)) = (load("LEMG"), load("LXGB")) else {
        println!("no install, no verdict");
        return;
    };
    let window_s: f64 = std::env::var("XVL_TRAFFIC_SWEEP_S")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1800.0);
    let (lxgb_schedule, lxgb_board, lxgb_local) =
        timetable_schedule(&lxgb, traffic::routes::field_routes("LXGB").unwrap());
    let mut desk = Link::new(lxgb, lxgb_schedule, "");
    desk.set_board(lxgb_board, lxgb_local);
    desk.make_desk();
    let mut desks = [DeskRun::new(desk)];
    let (lemg_schedule, _, _) =
        timetable_schedule(&lemg, traffic::routes::field_routes("LEMG").unwrap());
    let mut st = installed_station(&apt, "LEMG");
    let s = fly_fields(&mut st, lemg, lemg_schedule, window_s, false, &mut desks);
    let line = |icao: &str, s: &HarnessScore, towed: usize| {
        println!(
            "{icao}: {} scheduled, {} completed, {} touchdowns, {} conflicts, \
             {} overlaps, {} go-arounds, {} towed, spacing {}/{}/{} \
             rerouted/towed/held, {} on stands at the start, {} calls",
            s.movements,
            s.completed,
            s.touchdowns,
            s.conflicts.len(),
            s.overlaps.len(),
            s.go_arounds,
            towed,
            s.spacing_rerouted,
            s.spacing_cleared,
            s.spacing_held,
            s.parked_at_start,
            s.calls
        );
        for l in s.conflicts.iter().chain(&s.oracle).chain(&s.overlaps).take(10) {
            println!("    {l}");
        }
    };
    let towed = s.despawns.get("Towed").copied().unwrap_or(0) + s.parked_towed;
    line("LEMG (station)", &s, towed);
    let d = &desks[0];
    line("LXGB (desk)", &d.s, d.towed());
    println!(
        "LXGB board at the end:\n{}",
        d.link.board_payload(None, &[])
    );
    assert!(s.conflicts.is_empty(), "LEMG: {:?}", s.conflicts);
    assert!(s.oracle.is_empty(), "LEMG: {:?}", s.oracle);
    assert_eq!(s.readbacks, s.readbacks_accepted, "LEMG: readbacks");
    assert_paced(&s, "LEMG");
    // Overlaps print, as the mid-flow test's do: a taxi-out 18 to 19 m past
    // a parked aircraft is the stand plan's owed clearance (2026-10-13).
    assert!(d.s.conflicts.is_empty(), "LXGB: {:?}", d.s.conflicts);
    assert!(d.s.unfinished.is_empty(), "LXGB: {:?}", d.s.unfinished);
    assert!(d.s.calls > 0, "LXGB's traffic flew");
    // The desk's field changes nothing at the station's: LEMG alone flies
    // the same window the same way.
    Station::uninstall();
    let lemg = load("LEMG").expect("loaded above");
    let (alone_schedule, _, _) =
        timetable_schedule(&lemg, traffic::routes::field_routes("LEMG").unwrap());
    let mut st = installed_station(&apt, "LEMG");
    let alone = fly_field(&mut st, lemg, alone_schedule, window_s);
    let alone_towed = alone.despawns.get("Towed").copied().unwrap_or(0) + alone.parked_towed;
    line("LEMG (alone)", &alone, alone_towed);
    assert_eq!(
        (alone.completed, alone.touchdowns, alone.go_arounds, alone_towed),
        (s.completed, s.touchdowns, s.go_arounds, towed),
        "LEMG flies the same with LXGB beside it"
    );
    Station::uninstall();
}
