# Stage D: our own AI traffic (design, 2026-10-08)

Decided: our traffic REPLACES LiveTraffic as the default, the tower CONTROLS it, the plugin
draws it, schedules are generated or file-loaded and never fetched, deterministic and local.
First target: GA circuits plus a few IFR movements at a towered field, flown with the 172.
Builds on stage C (`stage-c-strips.md`): `FlightStrip`, `STRIPS`, `respond_for`, the computed
`RunwayBook`, the `_for` tick cores, `AircraftState`.

## Shape

```
  monitor tick (src/world_tick.rs)                   command loop (src/main/station_work.rs)
   traffic_link::Link::tick(snap, frozen, dt)          drain_traffic_calls():
    orders <- atc::world::traffic_order(id)              Station::sync_traffic_desks()
    Fleet::step -> Event::Call --> world::post_traffic_call   Station::answer_traffic(call)
    states --> world::note_traffic_state(id, st)              = traffic_ctx + respond_for
    world::traffic_tick(id, targets)  (_for cores)
    TrafficSink.publish(packed) --> xcopilot/traffic/{roster,frames}
                                         |
  plugin traffic.cpp: XPMP2 aircraft, interpolate 1 frame behind, TCAS slots, Bluebell CSL
```

- **`crates/traffic` depends on `nav` only** (plus anyhow, tracing, serde). Neither `atc` nor
  `traffic` depends on the other; they meet in the bin's `src/traffic_link.rs`, which owns the
  sink. `cargo tree -p traffic` stays as lean as nav.
- **The station never actuates**: traffic moves its own aircraft through its own sink, never
  through atc. The crew sees traffic only through `party_line::traffic_picture()`.
- **Runs in the monitor's tick** (`WorldTick`), no new runtime; no new sim reads (own state from
  the snapshot); one packed write per 250 ms on a detached latest-only task.
- **Sim-time clock** (`traffic::SimClock`, from a new `sim/time/total_running_time_sec`
  const): advances only when not `sim_frozen`, delta clamped 0..0.5 s, fixed 0.1 s substeps.
  Replay and pause freeze the fleet. Time acceleration above 1x freezes it too in v1 (atc's
  timers are wall `Instant`s), said and logged.

## The app side (`crates/traffic`)

```rust
pub struct TrafficAircraft { id: u32 /* = FlightId::Traffic(id) */, callsign, spoken,
  icao_type, wake: Wake, perf: &'static Perf, plan: Plan, kin: Kin, phase: Phase,
  radio: RadioState, seed: u64 }
pub struct Perf { taxi_kts, turn_taxi_dps, vr_kts, v2_kts, climb_fpm, climb_kts, cruise_kts,
  descent_fpm, app_kts, vref_kts, decel_kts_s, pattern_agl_ft, downwind_offset_nm, base_ext_nm }
  // tables: PISTON_SINGLE, TURBOPROP, JET_MED, JET_HEAVY
pub enum Plan { Circuit{runway, laps, left}, Departure{runway, sid, exit_bearing},
  Arrival{entry, star, runway, straight_in}, Parked }
pub struct Kin { lat, lon, alt_ft, on_ground, gs_kts, hdg_deg, track_deg, vs_fpm, pitch, roll,
  gear, flaps, lights }
pub enum Phase { Parked, TaxiOut{path}, HoldShort, LineUp, Roll, Climb, Leave, Arrive{leg},
  Pattern{leg}, Final, Landing, Vacate{path}, TaxiIn{path}, GoAround, Gone }
pub enum Call { Taxi, Ready, Inbound{pos}, Downwind, Base, Final, Vacated, GoingAround,
  Readback(String) }
pub struct Order { state /* mirrors tower::State */, runway, taxi_path, holding_point,
  follow: Option<u32>, hold: bool, go_around: bool, readback_owed: Option<String> }
impl Fleet { fn step(&mut self, dt, own: Option<OwnTarget>, orders: &dyn Fn(u32)->Option<Order>)
  -> Vec<Event> }   // Event::{Call, Spawn, Despawn}
```
`FieldModel` from `nav::AirportInfo` (runways, freqs, `allows_circuits`, pattern sides,
`crates/nav/src/airports.rs:73-97`) and `nav::TaxiNet` (stands, `taxi.rs:31,55`).

**Kinematics** (`traffic::kin`, pure, NOT the crew's frozen laws): one pure-pursuit
`follow(path, kin, kts, max_turn_dps, accel, dt)` for taxi and air; vs toward target altitude
clamped to the Perf rates; a 3 degree final from threshold + 50 ft, a flare below 30 ft, a
rollout at `decel_kts_s`; pattern legs from runway geometry and Perf. Heading = track in v1
(no wind triangle). Deterministic from `XorShift64(seed)` and the fixed substep.

## The ATC seam

- **Calls go in as TEXT** in the field's dialect ("Tower, Cessna three four five, ready for
  departure runway one three") through `tower::parse_intent` and `route_keyed`, readback gate
  included, into an inbox static (lifetime group `TrafficStrips`). `Station::answer_traffic`
  opens the strip and calls `respond_for`.
- **Traffic `Ctx`**: `Station::traffic_ctx(state, spoken, intent)`, synchronous: wind from
  `TOWER_WIND`, QNH from a `last_qnh` cached by our `build_ctx`, targets = fleet picture plus
  our aircraft, taxi route from `plan_taxi_route_to`, ends in the pure `make_ctx` (`ctx.rs:202`).
- **Field facts are ours**: every write to `runway_in_use` (`runway_pick.rs:233-393`,
  `respond.rs:96,364`), `TOWER_WIND`, `note_ground_traffic` and the ATIS letter is guarded by
  `!desk.strip.is_traffic()`; the one exception, pinned and logged: traffic may initialise
  `runway_in_use` when it is `None`.
- **Clearances come back as data, level-triggered**: `world::traffic_order(id)` reads the
  strip's cells (clearance, taxi route, holds, line-up condition, pending go-around). No edge
  queue to drop. Traffic obeys: taxi to the hold and stop; line up and wait; roll on takeoff
  clearance; no landing clearance by 300 ft AGL = go around and say so; a go-around order at
  once; an owed readback answered verbatim.
- **Desk syncs**: `drain_traffic_calls` runs `Station::sync_traffic_desks()` first, swapping
  each traffic desk in (`DeskSwap`) for the same `apply_*` our `sync_tower` does.
- **Positions**: a strip `watch` cell `seen: Option<AircraftState>` written by
  `note_traffic_state`; `RunwayBook::now` reads it, so Occupying, RunwayStart, Vacating and
  Crossing claims appear for traffic and our holds see it.
- **Ticks**: `world::traffic_tick(id, targets)` runs `note_aircraft_state_for`,
  `landing_hold_tick_for`, `takeoff_hold_tick_for`, `go_around_tick_for` with the synthetic
  picture (fleet minus self, plus us). `compliance_tick_for` runs in tests only, as an oracle
  that must stay silent.
- **Sequencing**: a strip `facts` cell `sequence: Option<(u8, Option<FlightId>)>` set where the
  tower already says "number two"; `Ctx.final_traffic_id` so "follow the Cessna on base" names
  type and id; traffic extends downwind until its leader is 1.5 nm on final.
- **Spacing**: departures on the existing `WAKE_SECS` and `DEPART_GAP_NM`, fed by
  `wake_note_targets(synthetic)`.
- **Reinstall rule**: an install clears `STRIPS`; every non-parked traffic aircraft despawns
  ("left the frequency"); `Link` regenerates when the installed field changes.

## Speech

Traffic transmits as `StationSpeaker::Ghost{seed: hash(callsign)}` through the existing
overheard path; the tower answers in its own voice. Spoken only when `tuned_speaker` is the
tower (one frequency). Traffic never waits on speech; an overheard line older than 8 s is
dropped; never spoken over our PTT or a pending pilot line (the fleet retries 3 to 5 s later).

## Schedules

`traffic::schedule::generate(field, density, seed)`: 1 to 3 circuit aircraft where circuits
are allowed; IFR arrivals every N minutes from a 10 nm entry or a STAR, departures by SID.
`XCOPILOT_TRAFFIC_DENSITY=off|light|normal`; `XCOPILOT_TRAFFIC_FILE` (YAML: `t_s, callsign,
type, kind, runway?, stand?`). Despawn: departures 12 nm out, arrivals 120 s after parking,
circuit aircraft after their laps. Never spawn on an occupied stand or within 3 nm of us.

## The sim side (plugin)

**XPMP2** (github.com/TwinFan/XPMP2, MIT, keep its notice; build WITHOUT `INCLUDE_FMOD_SOUND`),
vendored under `plugin/third_party/XPMP2` with its sources in the Makefile's `SRCS` like imgui.
It does what XPLMInstance would need months for: CSL model matching, animation (gear, flaps,
lights, engines, rotors), TCAS slots and override, wake, contrails, labels, map layer.

- **Models**: CSL packages are user-installed, never bundled (no redistribution licence).
  This machine has Bluebell under `plugins/LiveTraffic/Resources/CSL` (4.4 GB, 3,314 objects)
  plus `Doc8643.txt`, `related.txt`, `relOp.txt`: `XCOPILOT_TRAFFIC_CSL` defaults to that
  folder (read-only), the model count said at boot.
- **`plugin/src/traffic.cpp`**: `XPMPMultiplayerInit` after `drefs::start()`,
  `XPMPLoadCSLPackage` per path, a `TrafficAc : XPMP2::Aircraft` per id created and destroyed
  only in the flight loop on a roster change; `UpdatePosition` interpolates. Flight loop
  `BeforeFlightModel`, every frame, skips while loading, freezes while paused. Setters only
  copy bytes and flag (CLAUDE.md). `XPMPMultiplayerCleanup` in `XPluginStop`.
- **Wire** (little-endian, versioned, 16 KB buffers; `TextRef`'s 2 KB is too small):
  `xcopilot/traffic/roster` on change (id, type, airline, livery, callsign, modeS, squawk);
  `xcopilot/traffic/frames` at 4 Hz (header: version, seq, sim-time stamp, count; 40 bytes per
  aircraft: id, lat/lon f64, alt f32, hdg/pitch/roll centidegrees, gs, vs, gear, flaps,
  spoiler, reverser, lights, flags); `/enable`, read-only `/status` (models loaded, TCAS owner,
  count drawn), `/labels`. 60 aircraft are about 3.3 KB base64; 4 PATCH/s, about +5% of
  today's request rate, no per-aircraft requests. In `health::PLUGIN_DATAREFS`.
- **Interpolation**: two frames per id, rendered one frame behind (Hermite position, slerp
  heading), extrapolate at most 1.5 s then freeze; sim-time stamps; ground clamped to the mesh
  (`XPLMProbeTerrainXYZ`), not apt.dat elevation. ATC judges on the app's truth, never the
  drawn position (250 to 500 ms behind).
- **TCAS**: whoever takes the slots first wins; the pilot's LiveTraffic (`ai_on_request 0`)
  grabs them at start. Without them our planes still draw but TCAS and other plugins do not
  see them. `XPMPMultiplayerEnable`'s callback hands them over when freed. The owner is said at
  boot by name.

## What retires (`XCOPILOT_TRAFFIC=ours`, the default)

`chatter_tick` and `GHOSTS` stop being driven; `own_strip_claim`, `player_blocked_strip`,
`resequence_runway` and the `ghost_stale` memory stay compiled for `real` mode but are not
ticked. Every TCAS read goes through one `world::targets(xp)` returning the synthetic picture
under `ours` (XPMP2 fills TCAS with OUR planes; reading them as ghosts would double-count),
pinned by a test. `real` keeps today's read-only narration; `off` gives neither.

## Increments

| # | Step | Gate | Needs sim |
|---|---|---|---|
| 1 | `crates/traffic` skeleton: Perf, Kin, SimClock, XorShift64, `follow` | path follow converges, turn rate bounded, seed-identical traces | no |
| 2 | Phase machine on a scripted `Order`: taxi, roll, climb, pattern, final, rollout, vacate on a LEMG TaxiNet fixture | a traffic "sortie" test: 172 circuit stand to stand, deviation and touchdown zone asserted | no |
| 3 | Schedule generator, file format, despawn | 3 apt.dat fixtures; no spawn on an occupied stand | no |
| 4 | atc: strip `seen` cell, `note_traffic_state`, the book reads it | book matrix extended: traffic Occupying blocks our line-up; `book_tests` green | no |
| 5 | atc: `traffic_ctx`, `answer_traffic`, field-fact guards, the inbox | a traffic "ready" moves no `runway_in_use`/`TOWER_WIND`; corpus test; empty identical | no |
| 6 | atc: `traffic_order`, `sync_traffic_desks`, `traffic_tick`, `sequence` | the landing hold releases traffic; a go-around reaches the order | no |
| 7 | `traffic_link` headless harness: Fleet + a real Station on a fake clock, a field hour | every movement completes, compliance oracle silent, zero runway conflicts; world sweep with a generated schedule per field | no |
| 8 | Monitor wiring, `world::targets` switch, reinstall despawn | world_tick tests; seam pins | no |
| 9 | Speech: Ghost voice, tuned rule, stale drop | arbiter queue tests | no |
| 10 | Packed sink and the `xcopilot/traffic/*` contract | byte-layout round trip | plugin |
| 11 | Plugin: XPMP2 vendored, `traffic.cpp`, draw | live: 172 + 2 circuit aircraft + 1 IFR arrival | sim + plugin |
| 12 | Retire the ghost ticks under `ours`, `real` mode, CLAUDE.md | `real` path tests unchanged; `c172-regress.yaml` | sim |

## Risks
- Wall vs sim clocks (v1 refuses time acceleration above 1x for the fleet).
- Lock order for `note_traffic_state` (map, clone Arc, drop, then the cell).
- `answer_traffic` stays synchronous; no await inside `DeskSwap`.
- A traffic readback failing the gate forever: capped at 2, then "say again" once and despawn.
- A speech flood at busy fields: stale drop plus a per-field call rate cap.
- TCAS double count if a read bypasses `world::targets` (pinned).
- First-run experience depends on the pilot having Bluebell or LiveTraffic's CSL.
- LiveTraffic holds the TCAS slots by default: disable it (move to `plugins/Disabled`) or set
  its `ai_on_request 1` while ours is in use.
- Frame cost at 60 aircraft unmeasured: profile with `api_load` and fps.
- Our traffic's wake reaches the pilot through XP12 (realistic, new).

## Not in v1
Crew flight laws for traffic; online schedules; wind triangle; per-category wake; "land after"
or "number two cleared to land" conditionals; strips re-adopted across a reinstall; departure
or approach handoff for traffic (it leaves at 12 nm); traffic at uncontrolled fields; traffic
with ATC off (parked aircraft drawn only); any `pub static` for traffic.

## Deviations (as built)

Increment 1 (`crates/traffic` skeleton):
- **`sim` is in `cargo tree -p traffic`**, transitively: `nav` depends on `sim` for
  `sim::angles`. "As lean as nav" holds; "free of sim" cannot without moving the angle helpers
  out of `sim`, which this stage does not do. No whisper, cpal, ort or atc.
- **`follow` keeps its progress in a `Follow { seg, done }`** passed alongside the path, and
  takes a `FollowSpec` (speed, turn rate, accel, decel, stop at end, slow for corners, minimum
  lookahead) instead of loose arguments: `follow(path, &mut Follow, &mut Kin, &FollowSpec, dt)`.
- **The pursuit gain is twice the classic arc's** (`4 sin(alpha) / L`, critically damped with
  heading as the only state; the classic gain rang on a saturated intercept), the lookahead is
  never shorter than the turn radius, and in the air a corner is FLOWN BY (the turn begins
  `R tan(turn / 2)` before it). Measured: a 300 m offset intercept at 85 kt settles with no
  overshoot; a 90 degree corner overshoots its new leg by about 25 m and settles within a mile;
  a 90 degree taxi corner is cut by under 4 m (slowed to 35% of the taxi pace).
- `Perf` gained `roll_accel_kts_s`, `turn_air_dps` and `pattern_kts`; `class_for_type(icao)`
  picks the table and the `Wake`. `SimClock` also takes the sim's running time (`observe`).

Increment 2 (the phase machine, `aircraft.rs`, `fleet.rs`, `field.rs`, `phase.rs`):
- **A `Phase` carries no path.** The aircraft holds ONE `Route` (path, a `Leg` per segment,
  its `Follow`, the threshold's index), and the airborne phases follow from the leg reached:
  the roll's route runs on through upwind, crosswind, downwind, base and final, so a corner is
  flown by and never re-planned. `Phase::Arrive` and `Phase::Pattern { leg: Leg }` take a
  `Leg`, not an index.
- **`Order.state` is `TowerState`**, a copy of `tower::State`'s six names (the crate never
  sees atc); `Order::default()` (no strip) clears nothing, so an aircraft with no order waits on
  the stand and goes around at 300 ft.
- **`Event` gained `Phase { id, from, to }` and `Touchdown { id, at, along_m }`**; `Despawn`
  carries a reason (`LeftTheArea`, `Parked`, `Removed`). `Call::text(spoken, runway)` renders
  plain English ("Tower, Cessna three four five, ready for departure runway one three");
  `Call::Inbound` is `{ miles, from }` (a compass point), not a position.
- **The final is flown on a ramp** (fed forward on groundspeed, trimmed on the error), and so
  are the downwind and base once the 3 degree path along the rest of the circuit is below
  circuit height: the final is joined ON the path. In the air the follow adds a LEAD TURN (full
  rate when the arc to roll out on the leg needs all the room left) and halves its lookahead:
  without them the turn onto a short 172 final ended 10 to 25 m off and took a mile to settle.
- The 172's circuit is 0.8 nm out with a 1.0 nm base extension and 5 degrees a second in the
  turns (about 20 degrees of bank); the flare holds 200 fpm below 30 ft.
- `FieldModel` is `AirportInfo` plus `TaxiNet` with `RunwayEnd` geometry (`at`, `frame`,
  `side`), `taxi_to_runway` / `vacate` / `taxi_to_stand` over nav's planners, `free_stand`.
  LEMG exists only in the install, so its sortie is `#[ignore]`
  (`real_install_lemg_172_circuit_stand_to_stand`) and a synthetic `fixtures/xtst.dat`
  (09/27, 1,500 m, a parallel taxiway, four stands, 27 published right) covers the rest.
- Measured (the sortie tests print them): XTST 09 stand to stand in 810 s, taxi cross-track at
  most 3.2 m (corner cuts), final below 300 ft at most 0.6 m, touchdown 381 m into 1,500 m;
  LEMG 13 from stand W4 in 1,321 s, taxi 3.3 m, final 0.5 m, touchdown 381 m into 3,206 m.

Increment 3 (`schedule.rs`):
- **`generate(field, density, seed, &SpawnCtx)`**: a fourth argument carries the tower's
  runway (else `default_runway`: the longest assignable strip, the end with an approach first),
  our own position and the others on the ground. "Never within 3 nm of us" is read as AIRBORNE
  spawns (an entry is turned round the field until clear, else skipped); on the ground the rule
  is the stand: ours and everyone's are occupied (`STAND_TAKEN_M`, 20 m), and a stand is handed
  out once per schedule. `Movement::spawn(id, field, ctx)` applies both again at spawn time
  (`Refusal::{NoRunway, NoStand, NearOwn}`: the bin retries later); a named stand that is taken
  moves to a free one of the class (a stand that names categories must name the class's).
- **Procedures are data on the field** (`FieldModel.procs`, `Procedures::load(cifp_dir, icao)`
  over `nav::list_procedures`): departures only where SIDs exist, arrivals where approaches or
  STARs exist (straight in where the runway has an approach, else to the downwind, 10 nm out
  within 50 degrees of the centreline). The SID and STAR are names only in v1: departures leave
  on a seeded bearing.
- **Types by runway**: jets (A319, A320, B738) and the ATR 72 on 1,800 m or more, the ATR alone
  from 1,100 m, nothing IFR below; circuits are C172, C152, PA28, SR22 with 2 to 4 laps.
  Callsigns by region (`G`, `EC`, `F`, `D`, `I`... registrations, regional carriers), spoken in
  full.
- **The file** (`parse_file`, `from_file`): a YAML list of `t_s, callsign, type, kind`
  (`circuit|departure|arrival`), `runway?, stand?, laps?, spoken?`; unknown fields, a bad kind,
  a negative time or an empty callsign refuse the WHOLE file; a runway the field lacks drops its
  row. The crate reads no environment: the bin passes the text.
- **Despawn** stays in the phase machine (12 nm, 120 s parked, circuits after their laps); an
  arrival with nowhere to park waits clear of the runway for 240 s and goes (`Despawn::NoStand`).
- The flare holds half the approach's sink (never under 200 fpm): at 200 the ATR landed 560 m
  into 1,500 m. Generated hours, flown to the end against the scripted tower: XTST (circuits,
  ATRs) every touchdown under 500 m; XNOC (jets on 2,800 m) 427 to 440 m.
- Fixtures: `xtst.dat` (+ `cifp/XTST.dat`: a SID, an approach for 27), `xnoc.dat` (allows
  circuits 0, 2,800 m, jet gates, `cifp/XNOC.dat`: two SIDs, a STAR, ILS 12), `xbar.dat` (an
  800 m grass strip with no network: nothing generated). `real_install_schedule_lemg` is the
  ignored one over the install (LEMG says no circuits; IFR both ways, every movement spawns).
- Known: a turboprop's vacate at 16 kt cuts its 90 degree turn-off by up to 7 m (the 172's
  taxi stays under 3.3 m); not asserted for the ATR yet.
