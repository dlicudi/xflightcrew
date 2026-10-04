# Process-wide statics audit (XVoiceLayer, 2026-10-04)

Read-only audit of every `static` in `src/`, `crates/atc`, `crates/sim`. Nothing in the repo was edited.

## 0. Method and counts

- Inventory: perl scan for `^\s*(pub(\(..\))?\s+)?static NAME:` over every `.rs` file: **312 statics** (283 production, 29 test-only locks/fixtures).
  The brief's `grep -rnE '^\s*(pub(\(crate\))? )?static '` returns 273 on this Mac: BSD grep silently drops 39 `pub static` lines, including almost every loop flag (`autotaxi::ACTIVE`, `full_auto::ACTIVE`, `autoland::ABORT`, `pushback::ABORT`, `TAKEOFF_MARGIN_WAIVED`, `DESTINATION`...). Use `rg` or perl for any future source scan.
- Writers and readers: every in-file reference was attributed to its enclosing fn and classed write/read by a heuristic (`store/swap/fetch_/= /take/insert/clear...`); each accessor fn was then traced to its callers by `module::fn` across the tree. Every leak, race and multi-writer claim below was then confirmed by reading the code at the cited lines.
- Lifetime codes: **P** process, **S** session (app run, incl. per-plugin-load caches), **A** airframe (reset on swap), **F** flight (reset on new flight / reposition), **M** mission (one full-auto run), **L** loop (one automation run), **W** world/field (one installed ATC station), **C** constant or memo cache, **T** test-only.

Production lifetime distribution: L 85, S 47, F 40, A 37, P 24, M 20, W 18, C 10, mixed 2.

## 1. The reset doors that exist today

| Door | Where | Clears |
|---|---|---|
| `flight_state::publish_airframe(p)` | `src/flight_state.rs:34` (boot `detect.rs:19`, swap `detect.rs:46`) | 33 airframe selectors in `aircraft`, `autotakeoff`, `takeoff_config`, `cabin_air`, `radio`, `signs`, `circuit`, `lights`, `procedure` vs limits, `instruments`, `systems`, `anti_ice`. Plus `DEPARTURE_MINIMUMS` (a mission fact) and `FLIGHT_TRANSITION` (a flight fact). |
| Second airframe doors, outside it | `flow::install` (`main.rs:912,1488`), `instructor::install` (`instructor.rs:1695`), main's swap block clearing `main::LAST_READBACK` (`main.rs:1502`), `detect.rs:606-618` (systems, gear warning, nose from the `.acf`) | lists, overrides, lesson, copilot readback |
| `flight_state::stand_down()` | `src/flight_state.rs:89` (swap `detect.rs:41`, instructor `main.rs:1135`, connection lost `monitor.rs:5153`, every setup) | 13 loop aborts + 6 plain flags + arrival station pin. Pin test `stand_down_pins_every_active_loop` checks only statics literally named `ACTIVE`. |
| `party_line::retire_for_new_flight()` -> `station::retire_for_new_flight` | `crates/atc/src/station.rs:4144`; called by setup ramp `setup.rs:525`, approach `setup.rs:1257`, enroute `setup_enroute.rs:912`, foreign init `setup.rs:141` (only when ATC mode is on) | 21 party-line/station flight facts, sets `NEW_FLIGHT_PENDING` |
| `flow::stand_down(NewFlight)` | same setup sites | proposal + running list |
| `Station::install` / `Station::uninstall` | `station.rs:345` / `:441` | ~40 world statics; install keeps `REGION_HANDOFF`, `IFR_FLIGHT`, `ARRIVAL_PIN`, `PENDING_STATION`, `ZONE_CHALLENGE`, `SELF_TAXI` |
| Mission stop | `full_auto::abort` `src/full_auto.rs:154` + `clear_stage` + `set_arrival_runway(None)` | stage, mission, arrival runway, `TAKEOFF_MARGIN_WAIVED`, child loops |

There is **no crew-side "new flight" door**: a reposition runs `stand_down` + `flow::stand_down` + (ATC on) `retire_for_new_flight`, and nothing else. `flight_state.rs:20-27` already names the gap ("the next step is a `flight_begins` door") and lists 15 statics; this audit finds more and corrects one (`pushback::LEARNED_SENSE` is session by design, `pushback.rs:965-971`).

## 2. Leak risks, ranked

Ranked by (does it ACTUATE or speak something false) x (how ordinary the trigger is).

### 2.1 HIGH: actuates on the wrong aircraft or flight

1. **`hover::ROTOR`** (`src/hover.rs:340`), airframe fact, never cleared.
   Written only by `heli::Hover::new` (`src/tools/heli.rs:351`), which runs when the tool registry is built for a profile with `heli_start` (`src/tools/mod.rs:648`). A fixed-wing profile builds no `Hover`, so nothing writes `None`; it is not in `publish_airframe`. After an S-76 -> any aeroplane swap:
   - `taxi_in.rs:727` takes `collective_dref().is_some()` as "this is a helicopter" and hands the taxi-in to `heli_route::park` (a hover taxi writing `sim/cockpit2/engine/actuators/prop_ratio`, the propeller lever on a turboprop),
   - `hover::tower_on_ground` (`hover.rs:1274`, fed to the tower's compliance watch at `monitor.rs:3735`) reports an aeroplane under 100 ft AGL as on the ground,
   - `tools/taxi.rs:568` skips the runway vacate.
   This is exactly the "selector only set when present" bug class `publish_airframe` was written to end. Fix: `crate::hover::publish_rotor(p.heli_start.as_ref().map(..))` inside `publish_airframe`.

2. **Actuating loops outside `stand_down`** (the pin test only matches `static ACTIVE:`).
   - `tools::controls::BRAKES_HELD` (`src/tools/controls.rs:328`): the crew brake-hold loop renews `BRAKES_REGULAR` every `BRAKE_HOLD_SECS` while true. Not cleared by `stand_down`, so it survives a swap, a new-flight setup and **entering instructor mode** (CLAUDE.md: "Entering instructor stands every loop down").
   - `tools::autopilot::CLIMB_WATCHER_GEN` (`autopilot.rs:591,615-700`): a 20-minute capture watcher that writes `ALT_DIAL`, fires `CMD_ALT_HOLD` and sets the throttle. Only a newer capture command or a dial change stops it; `stand_down` does not bump the generation.
   - `tools/jet_start.rs:90` and `tools/power.rs:76` spawn start / power-up sequences with no cancel flag at all (grep for abort/cancel/ACTIVE finds none): a swap or instructor entry mid-start lets the old sequence keep firing switches.
   Fix: bump/clear all three in `stand_down`, give jet_start/power a `sim::run::Slot`; widen the pin test (section 6, stage 0).

3. **`tools::instruments::FLIGHT_TRANSITION`** (`instruments.rs:502`), flight fact cleared only per airframe.
   Set by the full-auto arm (`tools/full_auto.rs:1062,1620`) and `setup_enroute.rs:626`; cleared only by `publish_airframe` (`flight_state.rs:77`). Mission end, the approach setup (`setup.rs`) and a foreign new flight leave the last plan's TA/TL in force. `transition_in_force_ft` feeds the monitor's `AltimeterSetting` check (`monitor.rs:1828,2008`) which, with an automation flying, **writes** the altimeter (`Check::fix_write`). A plan LIEA-LEBL (TA 6,000) followed by a "put us on a ten mile final" elsewhere is judged against the old transition.

4. **Circuit -> autoland launch race after a stand-down** (`src/circuit.rs:5176-5215`).
   The circuit awaits `try_published_arrival(..)` ("takes a while off the reactor"), then, without re-checking its own `ABORT`, does `crate::autoland::ABORT.store(false)` and `run_autoland(..)` (the unscoped form: `parents` is `Vec::new()`, `autoland.rs:1496`). A `stand_down` landing inside that await (swap, instructor, reposition) sets `circuit::ABORT` and `autoland::ABORT`; the circuit thread then clears autoland's latch and arms a new autoland that no parent can cancel. The authored-route handoff (`circuit.rs:4234-4241`) also clears autoland's latch and launches unscoped. Fix: launch through `run_autoland_scoped` with the circuit's cancel as parent, and never let a caller clear another loop's `ABORT` (section 5).

### 2.2 MEDIUM: false statement, wrong runway, or a handoff to the wrong arm

5. **`autoland::TRACKING_ERROR`** (`autoland.rs:18`). Reset only at the next autoland arm (`:1912`), written per tick (`:3974`). `monitor.rs:4966` reads it every tick unconditionally and `interaction::Landing::tick` (`interaction.rs:86-89`) keeps any finite value below 100 ft AGL whether or not automation flies. Every later hand-flown landing, on any airframe and any flight, says "Last approach tracking N metres from centreline" from the last autoland. Fix: clear at autoland exit, or read only while `ACTIVE`.

6. **Handed-down autoland arguments that can outlive their arm.**
   - `autoland::NEXT_GATE` / `NEXT_GLIDEPATH_DEG` (`autoland.rs:5118,5137`): consumed by `arm_gate()` / `arm_glidepath()` at `:1849-1852`, but `spawn_autoland` returns at `:1818` (`if xp.cancelled() { return; }`) before consuming. A cancelled launch leaves a circuit's Visual 500 ft gate and a steepened circuit glidepath for the next arm, which may be another flight's instrument approach.
   - `procedure::MISSED_LEGS` (`procedure.rs:481`): stashed at a MAP (`:4585`), taken only on a steep go-around (`autoland.rs:2945`). A normal landing leaves the old procedure's missed approach for the next steep go-around (currently gated off behind `MAP_AUTOLAND`, so latent).
   - `autotakeoff::CLIMB_HANDOVER` is safe (cleared at every takeoff start, `autotakeoff.rs:3651`) and `procedure::SPLICE` is safe (refused unless the follower is active).
   Fix: consume at the top of `spawn_autoland` (before any return) and clear the stash in `autoland::abort` / `procedure::abort`.

7. **`station::TOWER_WIND`** (`station.rs:6696`): written per reply (`:2325`), cleared only in tests (`:11301`). After a region switch or `uninstall`, `resequence_after_go_around` (`:6831`) falls back to the previous field's wind when `surface_wind_at` has nothing for the new field. Fix: clear in `install`, `uninstall`, `retire_for_new_flight`.

8. **`party_line::ZONE_CHALLENGE`** (`party_line.rs:208`): set by the zone watch (`station.rs:3331`), cleared by `take_zone_challenge` and `uninstall` only, not by `install` or `retire_for_new_flight`. With a pilot flying (no consumer), a challenge latches; the next mission's first enroute pass (`full_auto.rs:1408`, `heli_route.rs:2059`) drains it as if raised on this flight.

9. **`autoland::LANDED_RUNWAY`** (`autoland.rs:1210`): overwritten per autoland touchdown, never cleared. `taxi_in::vacate_runway` (`taxi_in.rs:244`) falls back to it. The comment there accepts a stale ident because `plan_vacate` refuses off a runway edge, but after a reposition onto the same strip (the sortie runner's normal case: LEMG 13 repeated) a hand-landed 31 vacates as the old 13.

10. **`monitor::DESTINATION`** (`monitor.rs:113`): written by `plan::ensure_plan` (`plan.rs:486`), never set to `None` anywhere. After a swap or reposition the old plan's destination still drives `dest_dist_nm` / TOD ring (`monitor.rs:394`) and the answers (`answers.rs:1401,1697`, `main.rs:2845`) while a pilot flies.

11. **`station::OWN_FIXES`** (`station.rs:6028`): cleared only when a fix arrives on the ground (`:6047`). An airborne-to-airborne reposition (approach or enroute setup) keeps up to `TRACK_KEEP_S` = 8 s of pre-jump fixes, so `own_time_to_threshold` (`:6427`) derives a groundspeed across the jump for the go-around judgement. Clear in `retire_for_new_flight`.

### 2.3 LOW: latent, shadow-only, cosmetic, or self-expiring

12. `circuit::LAST_READBACK` (`circuit.rs:1528`): never cleared on a new flight (main's same-named `main::LAST_READBACK` is cleared on swap only). A record for identical words with `out_at: None` blocks `OwedReadback::due` (`full_auto.rs:6108-6121`); with `out_at` under 30 s it suppresses a re-send. Needs identical words across flights (sortie runner repeats), and new readbacks overwrite first.
13. `circuit::CREW_RADIO` queue survives a new flight; `circuit::holds_for` / `stale_reason` judge each call at send, so harmless today, but a call queued for the old flight is judged against the new position.
14. `flight::PILOT_TAKEOVER` (`flight.rs:243`): cleared only by `note_crew_takeover`; a reposition carries "the pilot has it" into the next flight's `Situation`. Duty engine is shadow-only.
15. `autotaxi::LAST_ARRIVED` (`autotaxi.rs:387`): the EGHI "at the hold at 1,475 ft" class named in `flight_state.rs:7`. Still cleared only by the next drive or `taxi_in.rs:788`; feeds `Situation.at_hold` (`monitor.rs:750`, shadow) and full auto / taxi-in after their own drives.
16. `station::STANDING` (`station.rs:6187`): not cleared by `install`/`uninstall` while its sibling `STRIP_SEEN` is; self-prunes after `GHOST_MEMORY_S`. `station::SELF_TAXI`: not cleared by `install` or `retire_for_new_flight`.
17. `tools::anti_ice::CLAIMED`, `tools::cabin_air::MASKS`: per airframe, not per flight; claims survive a reposition and are later switched off as "ours".
18. `tools::ground::CART_REQUESTED`, `cockpit_auto::PILOT_MOVED_LIGHTS`, `rapport::{OFFER, LAST_LANDING_BY_AUTOMATION}`, `instructor::LESSON` (abandoned on swap, not on a new flight): flight facts without a flight reset; impact cosmetic.

Verified NOT leaking (good patterns to copy): the follower's published views (`procedure::LEG`, `FIX_DIST`, `ROUTE_AHEAD`, `TOD`, `TERRAIN_FLOOR`, `PILOT_ALT`, `PILOT_SPEED`) are all read through accessors that return `None` unless `procedure::ACTIVE` (`procedure.rs:119-260`), so a stale value cannot be read. Counters with snapshot watchers (`party_line::GO_AROUND_ORDERS`, `go_around::PILOT_ORDERS`) are generation-safe by construction.

## 3. Statics with more than one writer

"One writer per dataref" applied to process state. Counted from source (in-module write sites + external modules that write the static or its setter).

| Static | Writers | Why it matters |
|---|---|---|
| `tools::speedhold::ACTIVE` | 14 sites in speedhold + `taxi_in`, `instructor`, `flight_state`, `main`, `reverse`, `autotakeoff`, `controls`, `taxi`, `setup_enroute` | the throttle governor's on/off is decided in 10 modules; any of them can switch off a governor another loop relies on |
| `tools::speedhold::TARGET_KTS` / `MODE` | `autotaxi` (13 call sites), `taxi_in`, `tools::taxi`, `procedure` (`arm_flight_with_floor`), speedhold | last writer wins between a taxi and a flight target |
| `autotaxi::ACTIVE` | 16 sites in autotaxi + `taxi_in`, `instructor`, `full_auto`, `flight_state`, `main`, `tools::taxi` | external stores bypass `autotaxi::halt()` (which also clears `BACKTRACKING` and `LAST_ARRIVED`) |
| `autoland::ABORT` | autoland + `circuit` (2), `full_auto`, `procedure`, `tools::autoland`, `tools::full_auto`, `tools::setup` | callers CLEAR another loop's cancel latch before launch: the root of race 2.1.4 |
| `pushback::{ACTIVE, ABORT, SKIP}` | pushback + `main` (3), `tools::ground` (2), `flight_state` | stop routing writes the push's latches from outside |
| `tools::reverse::ACTIVE` | reverse + `main`, `flight_state` | |
| `tools::engine::{MIX_MANAGED, LEAN_SEEKING}` | engine (6 each) + `full_auto` via setters + `flight_state` | |
| `autotaxi::LAST_ARRIVED` | autotaxi (4) + `taxi_in.rs:788` | |
| `autotakeoff::DEPARTURE_MINIMUMS` | `full_auto` (3) + `flight_state`; never written by autotakeoff | declared in the reader, owned by the mission: ownership inverted |
| `aircraft::TAKEOFF_MARGIN_WAIVED` | `full_auto` (3) only | declared in `aircraft`, a mission fact: ownership inverted |
| `full_auto::RUNWAY_WAIVER_PENDING` | `full_auto` + `tools::full_auto` (2) | |
| `tools::instruments::FLIGHT_TRANSITION` | `tools::full_auto` (2), `setup_enroute`, `flight_state`, mutated by `procedure` via `note_descending` | |
| `procedure::CLEARED_CEILING` | `full_auto` (4) + procedure (2) | |
| `climb_watch::ARMED` | armed by `procedure`, `circuit`, `autoland`, `autotakeoff`, `setup_enroute`; disarmed by `flight_state`, `autoland` | ONE slot shared by five loops: a second arm silently replaces the first loop's check |
| `autoland::PANEL` | autoland + `recover::set_panel` (7 sites) | two loops on one panel line |
| `station::{SELF_TAXI, UNCONTROLLED_ARRIVAL, SQUAWK_WATCH}` | world side + CREW via `full_auto` (`begin_uncontrolled_arrival`, `end_arrival`) and `party_line::plan_self_taxi*` | crew writes inside the world, beyond the two exceptions `party_line.rs:10-13` documents (`take_*`, `retire_for_new_flight`) |
| `mode::MODE` | `voice` (7), `instructor` (5), `main`, `settings`, `pollers` | acceptable (a setting); instructor entry's stand-down is caught by the main loop's mode-edge watcher (`main.rs:1135`), so it lands on the next loop pass, not at the write |
| `simbrief::SHELF` | `plan`, `flightplan`, `full_auto`, `loading`, `setup_enroute` | acceptable: one `publish_plan` door |
| `flight::PILOT_TAKEOVER`, `monitor::CREW_YOKE_MS`, `tools::LAST_BRAKE_CMD` | 3 to 10 modules | mailboxes/timestamps by design; fine |

(`aircraft::publish_speed_ceiling` and `procedure::publish_vs_limits` look multi-writer by grep but the extra callers are inside `#[cfg(test)]` modules.)

## 4. Clusters that belong together

| Cluster | Members | Natural owner |
|---|---|---|
| **Airframe facts** (37) | `aircraft::{LIVE_PROFILE, FUEL_UNIT, THROTTLE_DREF, NOSE_CASTORS, NOSE_FREE, GEAR_LIMIT_KTS, FLAP_LIMITS, BUS_VOLTS_NOMINAL, SPEED_FLOOR_KTS, SPEED_CEILING_KTS, APPROACH_FLOOR_KTS, STALL_CURVE, APPROACH_THRUST_MIN, ARREST_LEAD, FLARE_ELEVATOR, ROLL_RUDDER, LANDING_DISTANCE_M}`, `procedure::{CLIMB_FPM, DESCENT_FPM, BEST_CLIMB_FPM}`, `autotakeoff::{GEAR_WARNING, CLIMB_OUT, TAKEOFF_POWER}`, `takeoff_config::AIRFRAME`, `circuit::AP_ALTITUDE_CAPTURE`, `tools::{signs::SIGNS, cabin_air::CABIN_AIR, radio::MODE_KNOB, lights::EXTERIOR, instruments::ALTIMETER}`, `systems::CURRENT`, `hover::ROTOR`, `flow::{LISTS, OVERRIDES}`, `instructor::LESSON` refs | one immutable `Arc<AirframeFacts>` built from profile + `.acf` facts |
| **Airframe claims** | `anti_ice::CLAIMED`, `cabin_air::MASKS`, `instruments::ALTIMETER_TRUTH`, `main::LAST_READBACK` | flight context (they are about switches set on this flight) |
| **Flight facts (crew)** | `autoland::{LANDED_RUNWAY, TRACKING_ERROR}`, `autotaxi::LAST_ARRIVED`, `instruments::FLIGHT_TRANSITION`, `monitor::DESTINATION`, `flight::PILOT_TAKEOVER`, `circuit::{LAST_READBACK, CREW_RADIO}`, `cockpit_auto::PILOT_MOVED_LIGHTS`, `ground::CART_REQUESTED`, `rapport::{OFFER, LAST_LANDING_BY_AUTOMATION}`, `flow::PROPOSAL` | `FlightCtx` |
| **Handoff mailboxes** (one producer arms the next consumer) | `autoland::{NEXT_GATE, NEXT_GLIDEPATH_DEG, VACATE_FOLLOWS}`, `procedure::{MISSED_LEGS, NEXT_ARRIVAL_HANDOFF_KTS, SPLICE}`, `autotakeoff::CLIMB_HANDOVER`, `full_auto::RUNWAY_WAIVER_PENDING` | a typed launch argument, not a static (pass it in `AutolandLaunch` etc.) |
| **Mission** | `full_auto::{RUN, ACTIVE, ABORT, STAGE, MISSION, ARRIVAL_RUNWAY, WAIT, INBOUND_MADE, WAIVER, REFUSED}`, `aircraft::TAKEOFF_MARGIN_WAIVED`, `autotakeoff::DEPARTURE_MINIMUMS`, `procedure::CLEARED_CEILING`, `heli_route::INBOUND_SENT`, `blackbox::SORTIE`, `party_line::ARRIVAL_PIN`, `station::UNCONTROLLED_ARRIVAL` | `MissionCtx` owned by the full-auto lease |
| **Autoland run** | `ACTIVE, ABORT, GEN, LANDING_CONFIGURED, PANEL, GLIDEPATH_DEG, LAST_APPR_READS` | an autoland `Slot` + per-run struct |
| **Each other loop** | `{ACTIVE, ABORT, GEN/GENERATION, DETAIL, STOOD_DOWN, RUN}` in autotaxi, taxi_in, circuit, procedure, autotakeoff, pushback, hover, heli_route, heli_circuit, recover, flow, speedhold, reverse, engine, controls (brake hold), autopilot (capture watcher), ils (setup watch) | `sim::run::Slot` (already used by full_auto, procedure, autotakeoff) |
| **ATC flight (party line)** | 21 statics cleared together by `retire_for_new_flight` + `SQUAWK_WATCH, HANDOFF, PENDING_RESEQUENCE, OWN_FIXES, ZONE_CHALLENGE, SELF_TAXI` | one `PartyLine` struct replaced wholesale |
| **ATC station (field)** | `COMPLIANCE, ATIS, ACTIVE_FIELD, CONTROLLER_FREQS, ACTIVE_PANEL, ACTIVE_RUNWAYS, OUT_OF_REACH, SURFACE_WIND, GHOSTS, GO_AROUND_LAST, STRIP_SEEN, STANDING, TOWER_WIND, WAKE, WAKE_PREV, DEPARTURE_STREAM, ARRIVAL_STREAM, monitor::LAST` | one `StationState` struct built by `install` |
| **ATC handoffs that deliberately survive install** | `REGION_HANDOFF, PENDING_STATION, IFR_FLIGHT, ARRIVAL_PIN, NEW_FLIGHT_PENDING, LAST_REGION_SWITCH` | a small `Handoffs` struct with each survival rule written once |
| **Session caches** | `station::{REGION, FIELD_DIR, TAXI_NET, ILS_FACTS}`, `xplane::{ID_CACHE, COMMAND_CACHE, VALUE_CACHE, ABSENT, LOAD, LOAD_GATES}`, `detect::PROBE_INDEX`, `weather::WINDS_CACHE`, `plan::ELEV_CACHE`, `autoland::RUNWAY_WIDTHS`, `setup::LIB` | stay statics; they are memos of the world on disk |
| **Process plumbing** | tts barge-in, ORT lock, terrain mailbox, blackbox file/tap, clocks/epochs, rng, settings, mode | stay statics |

## 5. How loops coordinate today, and what goes wrong

- **`ACTIVE` = "I am flying"**, read by ~14 modules each (`autoland::ACTIVE` by monitor x12, full_auto x9, flow x5, speedhold, engine, cabin, ils, go_around, setup, station...). It is the de facto mutual-exclusion and "quiet" signal. It is a bool, not an identity: nobody can tell WHICH run is active, so a late store from a superseded run (`autoland` uses `GEN` to guard this; circuit and hover do not) can flip it.
- **`ABORT` = sticky cancel latch**, set by the canceller, cleared by the NEXT start. Two smells:
  1. callers clear a child's latch before launching it (`autoland::ABORT.store(false)` from 7 sites), which defeats any stand-down that landed in between (2.1.4);
  2. a latch left `true` between runs is harmless only because every reader is the loop itself; it is not true for `pushback::ABORT`, which `main` and `ground` both read for stop routing.
- **Three ownership primitives coexist**: `sim::run::Slot` leases (full_auto, procedure, autotakeoff), generation counters (`autoland::GEN`, `taxi_in::GEN`, `heli_route::GENERATION`, `recover::GEN`, `autotaxi::DRIVE_GEN`, `ils::SETUP_GEN`, `autopilot::CLIMB_WATCHER_GEN`, `tempo::TURN_GEN`), and bare flags (circuit, hover, heli_circuit, pushback, reverse, speedhold, engine, brake hold). Only `Slot` gives "cancelled worker still owns the slot until it returns" (`crates/sim/src/run.rs:37`) plus an outcome.
- **Parents**: `run_autoland_scoped` accepts parent cancel flags, but `circuit`, `tools::full_auto`, `tools::autoland`, `tools::setup` launch through the unscoped `run_autoland`, so cancelling the parent does not cancel the child.
- **stand_down completeness is checked by name**: `stand_down_pins_every_active_loop` scans for `static ACTIVE:` only, so `BRAKES_HELD`, the capture watcher and the jet/power sequences slip through.

## 6. Proposed ownership model

Principle: **a value cannot be read outside the lifetime it belongs to**, enforced by construction rather than by remembering a reset list.

### 6.1 Generation-scoped cells (the core primitive, in `crates/sim`)

```rust
pub struct Epoch(AtomicU64);                    // one per lifetime: AIRFRAME, FLIGHT, MISSION
pub struct Scoped<T> { epoch: &'static Epoch, cell: Mutex<Option<(u64, T)>> }
impl<T: Clone> Scoped<T> {
    pub fn set(&self, v: T)       { *lock = Some((self.epoch.now(), v)) }
    pub fn get(&self) -> Option<T> { lock.filter(|(g, _)| *g == self.epoch.now()).map(..) }
    pub fn take(&self) -> Option<T> { .. same filter, then None }
}
pub static FLIGHT: Epoch; pub static AIRFRAME: Epoch; pub static MISSION: Epoch;
```

`static TRACKING_ERROR: Scoped<f64> = Scoped::flight();` A new flight is `FLIGHT.bump()`: every flight-scoped cell is invalid at once, including ones nobody remembered. This is the existing `GO_AROUND_ORDERS` snapshot pattern, generalised. It costs one atomic load per read.

### 6.2 Doors

- `flight_state::airframe_begins(p)` = `AIRFRAME.bump(); FLIGHT.bump();` + build `AirframeFacts` (replaces `publish_airframe`; also absorbs `flow::install`, `instructor::install`, `hover::publish_rotor`, main's swap clears).
- `flight_state::flight_begins(why)` = `FLIGHT.bump()` + `stand_down()` + `flow::stand_down(NewFlight)` + (ATC on) `party_line::retire_for_new_flight()`. Called from the five new-flight sites (`setup.rs:525`, `:1257`, `:141`, `setup_enroute.rs:912`, and `detect.rs` swap) so they can no longer drift apart.
- Mission: `full_auto::start` bumps `MISSION`; `Lease` drop runs mission cleanup.

### 6.3 Airframe facts as one snapshot

`static AIRFRAME_FACTS: RwLock<Arc<AirframeFacts>>`. Keep every accessor name (`aircraft::speed_floor_kts()`, `flap_limits()`, `hover::collective_dref()`...) as a thin read of the snapshot so callers do not change. `.acf`-derived facts (systems survey, nose castor, gear warning) produce a new Arc via copy-on-write, so `publish_airframe` cannot miss a field: a struct literal that does not name every field does not compile.

### 6.4 Loops on `sim::run::Slot`, with a registry

- Every loop that can hold the aircraft owns a `Slot` (autoland, circuit, autotaxi, taxi_in, pushback, hover, heli_route, heli_circuit, recover, flow, speedhold governor, reverse, mixture manager, lean seek, brake hold, capture watcher, ILS setup watch, jet start, power-up).
- `static LOOPS: &[&Slot]` in `flight_state`; `stand_down` = cancel every slot. The pin test scans for every `sim::run::Slot` (and, transitionally, every `static .*: AtomicBool` that guards a spawned loop) and requires it in `LOOPS`.
- `pub static ACTIVE` becomes `pub fn active() -> bool { RUN.running() }`; external stores disappear (callers use the owner's `halt()`/`release()`).
- Child launches always pass the parent's cancel (`run_autoland_scoped` only); nobody writes another loop's latch.
- Loop-scoped published views use the `procedure` pattern (accessor gated on the owner's slot) or `Lease::with_cleanup` to clear them on drop.

### 6.5 Handoffs become arguments

`NEXT_GATE`, `NEXT_GLIDEPATH_DEG`, `MISSED_LEGS`, `VACATE_FOLLOWS`, `NEXT_ARRIVAL_HANDOFF_KTS`, `CLIMB_HANDOVER`: fields on the launch struct (`AutolandLaunch`, `start_scoped` args) so a cancelled launch drops them with it.

### 6.6 World side

- `static STATION: Mutex<Option<StationState>>`: `install` builds the whole struct, `uninstall` sets `None`. A new field in `StationState` must be initialised by the constructor, so "install forgot TOWER_WIND" cannot recur.
- `static PARTY_LINE: Mutex<PartyLine>`: `retire_for_new_flight` = `*pl = PartyLine::default()` except a named `Handoffs` sub-struct with documented survival (`REGION_HANDOFF`, `IFR_FLIGHT`, `PENDING_STATION`, `ARRIVAL_PIN`). Keeps the accessor-only rule of `party_line.rs` intact (statics stay crate-private).
- Crew writes into the world (`begin_uncontrolled_arrival`, `plan_self_taxi*`) get accessor names on the party line page so the exception list in `party_line.rs:10-13` is honest.

## 7. Staged migration (each stage green on `XCOPILOT_BLACKBOX=off cargo test --workspace`)

**Stage 0: guard rails (no behaviour change).**
- A lifetime manifest: `flight_state.rs` gets `const LIFETIMES: &[(&str, Lifetime)]` naming every production static (`"autoland::TRACKING_ERROR" => Flight`), and a source-scan test (perl-equivalent regex, `pub static` included) that fails on any static missing from it. This turns the table in section 8 into an enforced artefact, the way `health::PLUGIN_DATAREFS` pins the plugin.
- Widen `stand_down_pins_every_active_loop` to `BRAKES_HELD`, `CLIMB_WATCHER_GEN` and any `Slot`.

**Stage 1: the leak fixes, each with a unit test that reproduces it.**
`hover::publish_rotor` in `publish_airframe` (2.1.1); `BRAKES_HELD`/`CLIMB_WATCHER_GEN` in `stand_down`, cancel flag in jet_start/power (2.1.2); circuit re-checks `ABORT` and launches scoped (2.1.4); `TRACKING_ERROR` cleared at autoland exit (2.2.5); `arm_gate`/`arm_glidepath` before the early return and `MISSED_LEGS` cleared in aborts (2.2.6); `TOWER_WIND`, `ZONE_CHALLENGE`, `OWN_FIXES`, `STANDING`, `SELF_TAXI` added to install/retire (2.2.7, 8, 11, 16). All small, all local, each an `unflown` issue.

**Stage 2: the `flight_begins` door.**
Add `FLIGHT` epoch + `flight_begins(why)`; route the five new-flight sites through it; clear `FLIGHT_TRANSITION`, `LANDED_RUNWAY`, `DESTINATION`, `LAST_ARRIVED`, `PILOT_TAKEOVER`, `circuit::LAST_READBACK`, `CREW_RADIO` explicitly in it (explicit first, so behaviour is reviewable). Test: one test calls `flight_begins` and asserts every manifest entry marked `Flight` reads empty.

**Stage 3: `Scoped<T>` cells.**
Introduce `sim::scoped`; migrate the Flight cluster one static per commit, deleting its explicit clear from `flight_begins` as it goes. Test isolation: tests run on parallel threads, so a test that bumps `FLIGHT` would invalidate another test's cells; give `Epoch` a `#[cfg(test)]` thread-local override (the `GLIDEPATH_DEG_T` trick already in `autoland.rs:5140`) so each test thread has its own epochs.

**Stage 4: `AirframeFacts` snapshot.**
Replace the 37 airframe statics behind unchanged accessors; `publish_airframe` becomes `AirframeFacts::from_profile(p)` + install; `.acf` facts via `with_acf(..)`. Collapse `FLOOR_LOCK`, `FUEL_TEST_LOCK`, takeoff_config/signs/cabin_air/radio test locks into one `AIRFRAME_TEST_LOCK` or the thread-local override.

**Stage 5: loops onto `Slot`.**
Order by risk: brake hold, capture watcher, jet start, power (no cancel today) -> autoland (biggest reader fan-out; keep `pub static ACTIVE` as a re-exported fn for one release) -> circuit -> autotaxi/taxi_in/pushback (remove external `ACTIVE` stores) -> speedhold (single owner claim: `speedhold::claim(owner)` returning a guard) -> heli loops -> flow. After this, delete `ABORT` statics; `stand_down` iterates `LOOPS`.

**Stage 6: handoffs as launch arguments.** Move the 6.5 mailboxes into launch structs; delete the statics.

**Stage 7: world structs.** `StationState` and `PartyLine` behind the existing accessors; `SIDE_STATE_LOCK` tests keep working because the accessors are unchanged. `cargo tree -p atc` health check unaffected.

**Stage 8: `MissionCtx`.** Mission cluster into a struct owned by the full-auto lease; `TAKEOFF_MARGIN_WAIVED` and `DEPARTURE_MINIMUMS` move to their owner with read accessors in `aircraft`/`autotakeoff`.

Stages 0 and 1 are a day's work and close every HIGH and MEDIUM item; stages 2 to 8 are the structural cure and can go one cluster at a time.

## 8. Full inventory (283 production statics)

Columns: fn names with `(w)`/`(r)` are the in-file accessors that write/read the static (heuristic), `<-` lists the caller modules and call counts; `direct:` lists modules naming the static itself. Lifetime codes as in section 0.

#### crates/atc/src/callsign.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `LAST` :323 | Mutex<Option<(String, Source)>> | last resolved callsign (log dedupe) | note(r)<-atc1 | S | overwritten |

#### crates/atc/src/party_line.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `CLEARANCE` :31 | Mutex<Option<ClearanceSnap>> | standing clearance picture | clearance_state(r)<-full_auto11,station6,monitor5,autoland3,heli_route2,taxi_in1; clearance_runway(r)<-full_auto13,station8,heli_route2,monitor2,autoland2,taxi_in1; readback_owed(r)<-station10,full_auto9,heli_route1,monitor1,cir.. | F | install, uninstall, retire_for_new_flight |
| `SQUAWK_NUDGE` :37 | Mutex<Option<SquawkNudge>> | squawk nudge for crew | take_squawk_nudge(w)<-station4,monitor1 | F | install, uninstall, retire |
| `TAXI_ROUTE` :50 | Mutex<Option<TaxiWatch>> | cleared taxi route | with_taxi_route(r)<-full_auto5,taxi_in2,taxi2,autotaxi1,monitor1,main1 | F | install, uninstall, retire |
| `TAXI_TO_STAND` :56 | Mutex<bool> | route is an arrival | taxi_to_stand(r)<-taxi_in3,full_auto2,monitor1,taxi_guide1,taxi1 | F | install, uninstall, retire |
| `TAKEOFF_HOLD` :60 | Mutex<Option<TakeoffHold>> | held release | takeoff_hold_armed(r)<-full_auto1,monitor1,takeoff1,circuit1 | F | install, uninstall, retire |
| `LANDING_HOLD` :68 | Mutex<Option<LandingHold>> | landing withheld | landing_hold_armed(r)<-station7,autoland2,monitor1 | F | install, uninstall, retire |
| `PENDING_LANDING_RELEASE` :73 | Mutex<Option<(String, String)>> | landing release owed |  | F | install, uninstall, retire |
| `VECTORS` :76 | Mutex<Option<VectorPlan>> | vector plan | vectored_callsign(r)<-monitor1; assigned_vector(r) | F | install, uninstall, retire |
| `PENDING_STATION` :81 | Mutex<Option<(String, nav::AirportInfo.. | station swap waiting for main loop | station_swap_pending(r)<-main1,pollers1,setup1; take_pending_station(w)<-main1 | F (mailbox) | take_pending_station, uninstall |
| `ARRIVAL_PIN` :88 | Mutex<Option<i64>> | arrival station pin |  | M | pin_arrival_station(None) at stand_down, arm_new_flight, uninstall |
| `CALLSIGN_INPUTS` :95 | Mutex<Option<fn() -> callsign::Inputs>> | crew callsign provider fn | direct: callsign2;  | S | registered once |
| `CALLSIGN_TELEPHONY` :103 | Mutex<Option<fn() -> String>> | crew telephony provider fn | direct: callsign2,station1;  | S | registered once |
| `FILED_ROUTE` :111 | Mutex<Option<FiledRouteProvider>> | filed route provider fn | direct: station3; provide_filed_route(w)<-full_auto1,main1; filed_route_for(r)<-station2 | S | registered once |
| `PENDING_TAKEOFF_CLEARANCE` :145 | Mutex<bool> | released takeoff clearance | take_pending_takeoff_clearance(r)<-main1,station1 | F | take, install, uninstall, retire |
| `PENDING_LINE_UP` :150 | Mutex<bool> | line-up released | take_pending_line_up(r)<-main1 | F | take, install, uninstall, retire |
| `PENDING_GO_AROUND` :155 | Mutex<bool> | go-around pending (main loop) | take_pending_go_around(r)<-station2,main1 | F | take, install, uninstall, retire |
| `GO_AROUND_ORDERS` :163 | AtomicU64 | tower go-around order counter | arm(r); expected_runway(r); take(r) | P (counter) | monotonic, watchers snapshot |
| `RESEQUENCED_ORDER` :174 | Mutex<Option<(u64, String)>> | expect runway X with order | expected_runway(r) | F | install, uninstall, retire |
| `LINE_UP_CONDITION` :181 | Mutex<Option<bool>> | conditional line-up clear | line_up_condition_clear(r)<-station3,full_auto2 | F | install, uninstall, retire |
| `DEPARTURE_HANDOFF` :187 | Mutex<Option<(String, i64)>> | departure handoff owed | take_departure_handoff(w)<-station10,full_auto3,heli_route1 | F | take, install, uninstall, retire |
| `REGION_HANDOFF` :197 | Mutex<Option<RegionHandoff>> | region handoff owed | take_region_handoff(w)<-station9,full_auto2,heli_route1,monitor1 | F (survives install) | take, uninstall, arm_new_flight |
| `ZONE_CHALLENGE` :208 | Mutex<bool> | zone challenge raised | take_zone_challenge(r)<-station5,full_auto2,heli_route2 | F | take, uninstall only; NOT install or retire |
| `IFR_FLIGHT` :235 | Mutex<Option<IfrFlight>> | IFR flight code (survives install) | ifr_flight_code(r)<-station8,setup_enroute1 | F | landing, uninstall, retire, cancel IFR |
| `LANDING_HANDOFF` :254 | Mutex<Option<String>> | landing handoff runway | landing_handoff_runway(r)<-station5,monitor1 | F | install, uninstall, retire |
| `ASSIGNED_SQUAWK` :260 | Mutex<Option<i64>> | assigned code | assigned_squawk(r)<-full_auto7,heli_route1,monitor1,autotakeoff1,station1 | F | install, uninstall, retire |
| `IFR_CLEARANCE` :267 | Mutex<Option<tower::IfrClearance>> | accepted IFR clearance | assigned_clearance(r)<-station14,full_auto10,tower1 | F | install, uninstall, retire, cancel IFR |

#### crates/atc/src/station.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `SQUAWK_WATCH` :2770 | Mutex<Option<SquawkWatch>> | radar watch on our code | install(w); uninstall(w); assign_squawk(r); note_departure_intent(w); arm_departure_handoff(r); squawk_watch_active(r)<-monitor1; radar_contact_at_check_in(r); squawk_tick(r)<-monitor1; region_tick(r)<-monitor1; begin_uncontroll.. | F | install, uninstall, retire; crew writes via begin_uncontrolled_arrival |
| `COMPLIANCE` :2962 | Mutex<ComplianceState> | incursion/zone watch state | install(w); uninstall(w); compliance_tick_facing(r)<-monitor2; runway_occupancy_authorised(r)<-party_line2; own_departure_spacing(r) | W | install, uninstall |
| `HANDOFF` :3370 | Mutex<Option<Handoff>> | departure handoff worksheet | install(w); uninstall(w); respond(w); note_departure_intent(w); arm_departure_handoff(w); handoff_tick(r)<-monitor1; retire_for_new_flight(w)<-party_line2 | F | install, uninstall, retire |
| `ATIS` :3406 | Mutex<Option<AtisState>> | ATIS broadcast state | install(rw); uninstall(w); atis_due(r)<-monitor1; atis_retune(rw)<-monitor1; atis_tick(r)<-monitor1; atis_current_letter(r); suggest_call(r)<-main2; tuned_station(r)<-monitor1; void_note(r) | W | install, uninstall |
| `REGION` :3563 | Mutex<Vec<RegionApt>> | region airport index | resolve_destination(r); atis_retune(r)<-monitor1; region_loaded(r)<-main1; load_region(w)<-main1; region_tick(r)<-monitor1; fields_in_range(r)<-main2; arm_station_for_freq(r)<-full_auto1,heli_route1,main1; arm_new_flight(r)<-set.. | S | load_region |
| `FIELD_DIR` :3573 | Mutex<Vec<FieldDirectoryEntry>> | field directory | resolve_destination(r); load_region(w)<-main1 | S | load_region |
| `ACTIVE_FIELD` :3577 | Mutex<Option<(String, f64, f64)>> | active field name/pos | install(w); uninstall(w); compliance_tick_facing(r)<-monitor2; region_tick(r)<-monitor1; fields_in_range(r)<-main2; arm_station_for_freq(r)<-full_auto1,heli_route1,main1; active_station_in_range(r)<-main2,health1; note_aircraft(.. | W | install, uninstall |
| `LAST_REGION_SWITCH` :3579 | Mutex<Option<Instant>> | region switch cooldown stamp | region_tick(rw)<-monitor1; arm_station_for_freq(w)<-full_auto1,heli_route1,main1; arm_new_flight(w)<-setup1,party_line1; void_outcome(r) | S (timestamp) | window |
| `CONTROLLER_FREQS` :3907 | Mutex<Vec<(i64, StationSpeaker)>> | controller frequencies | install(w); uninstall(w); active_tower_khz(r)<-party_line2; tuned_controller(r)<-monitor5 | W | install, uninstall |
| `ACTIVE_PANEL` :3925 | Mutex<Option<ActivePanel>> | panel station facts | install(r); uninstall(w); compliance_tick_facing(r)<-monitor2; active_station_in_range(r)<-main2,health1; inbound_final_for_strip(r); suggest_call(r)<-main2; active_field(r)<-monitor3,main2 | W | install, uninstall |
| `UNCONTROLLED_ARRIVAL` :4023 | AtomicBool | crew arriving uncontrolled | install(w); uninstall(w); begin_uncontrolled_arrival(w)<-full_auto1; uncontrolled_arrival(r)<-party_line1; end_arrival(w)<-full_auto1; retire_for_new_flight(w)<-party_line2 | M | install, uninstall, retire, end_arrival; crew writes |
| `NEW_FLIGHT_PENDING` :4120 | AtomicBool | old station silenced until new install | install(w); uninstall(w); new_flight_pending(r); retire_for_new_flight(w)<-party_line2 | F (transition) | retire sets, install/uninstall clear |
| `OUT_OF_REACH` :4422 | AtomicBool | station beyond ground reach | install(w); uninstall(w); out_of_reach(r); note_aircraft(w)<-party_line2,monitor1 | W | install, uninstall, note_aircraft every tick |
| `SELF_TAXI` :4516 | Mutex<Option<String>> | self-planned taxi runway | uninstall(w); respond(w); begin_uncontrolled_arrival(w)<-full_auto1; note_aircraft(r)<-party_line2,monitor1; self_taxi_runway(r)<-party_line1; plan_self_taxi_to_stand(w)<-party_line1; plan_self_taxi(w)<-party_line2 | F | uninstall, respond, plan_self_taxi; NOT install or retire |
| `ACTIVE_RUNWAYS` :4720 | Mutex<Vec<nav::Runway>> | active field runways | install(w); uninstall(w); active_runways(r)<-party_line1; on_active_runway(r); on_active_runway_facing(r)<-monitor1; on_active_runway_known(r)<-party_line1; strip_under(r); runway_occupancy_authorised(r)<-party_line2; inbound_fi.. | W | install, uninstall |
| `SURFACE_WIND` :5609 | Mutex<Option<(f64, f64, f64, f64)>> | field surface wind from monitor | uninstall(w); set_surface_wind(r)<-full_auto1,monitor1,setup1; surface_wind_at(r) | W | uninstall; keyed by field pos at read |
| `GHOSTS` :5665 | Mutex<Option<HashMap<String, GhostMemo.. | ghost traffic memory | install(w); uninstall(w); chatter_tick(r)<-monitor1 | W | install, uninstall |
| `GO_AROUND_LAST` :6023 | Mutex<Option<Instant>> | go-around order rate limit | install(w); uninstall(w); go_around_tick(r)<-monitor2 | W | install, uninstall |
| `OWN_FIXES` :6028 | Mutex<VecDeque<(f64, f64, Instant)>> | our airborne fixes (8 s) | note_own_fix(r); own_time_to_threshold(r) | F | cleared on ground only; NOT on retire/install |
| `STRIP_SEEN` :6036 | Mutex<Option<GroundTracks>> | ghost ground tracks | install(w); uninstall(w); note_ground_traffic(r); strip_exits(r) | W | install, uninstall |
| `STANDING` :6187 | Mutex<Option<HashMap<String, Standing>>> | ghost standing clocks | note_ground_traffic(r); standing_secs(r); ghost_stale(r) | W | self-pruned; NOT cleared by install/uninstall |
| `GO_AROUND_DEFER_LOGGED` :6541 | Mutex<Option<Instant>> | log rate limit | go_around_tick(r)<-monitor2 | P (timestamp) | window |
| `TOWER_WIND` :6696 | Mutex<Option<(f64, f64)>> | wind the tower last used | build_ctx(r)<-callsign1; resequence_after_go_around(r) | W | NEVER cleared in production |
| `PENDING_RESEQUENCE` :6701 | Mutex<Option<(String, String)>> | resequence for FSM sync | install(w); uninstall(w); apply_monitor_go_around(r); retire_for_new_flight(w)<-party_line2; resequence_after_go_around(r) | F | install, uninstall, retire |
| `WAKE` :7397 | Mutex<Option<Wake>> | wake timer | install(w); uninstall(w); wake_note_targets(w)<-monitor1; wake_remaining(r) | W | install, uninstall |
| `WAKE_PREV` :7405 | Mutex<Option<HashMap<String, WakePrev>>> | wake previous ghost states | install(w); uninstall(w); wake_note_targets(r)<-monitor1 | W | install, uninstall |
| `DEPARTURE_STREAM` :7422 | Mutex<Option<(String, Instant)>> | last ghost departure end | install(w); uninstall(w); departure_end(r); field_streams(r); wake_note_targets(w)<-monitor1 | W | install, uninstall |
| `ARRIVAL_STREAM` :7446 | Mutex<Option<(String, Instant)>> | last ghost arrival end | install(w); uninstall(w); arrival_stream_end(r); field_streams(r); wake_note_targets(rw)<-monitor1 | W | install, uninstall |
| `TAXI_NET` :7624 | Mutex<Option<Arc<nav::TaxiNet>>> | taxi network of active field | compliance_tick_facing(r)<-monitor2; taxi_net(r)<-party_line4; spawn_taxi_net_load(w)<-main2; plan_taxi_route_to_stand(r); plan_vacate_from(r)<-party_line1; plan_taxi_route_to(r) | C/W | spawn_taxi_net_load |
| `ILS_FACTS` :7666 | Mutex<Option<Arc<IlsFacts>>> | ILS facts cache | arm_vectors(r); spawn_ils_load(rw)<-main2; plan_taxi_route_to(r) | C | spawn_ils_load |

#### crates/sim/src/blackbox.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `FILE` :39 | Mutex<Option<Option<std::fs::File>>> | blackbox file handle | write(r)<-sortie_runner1 | P | lazy open |
| `SORTIE` :40 | Mutex<Option<Sortie>> | current sortie file | sortie(r)<-setup4; begin_sortie(w)<-full_auto1; end_sortie(w)<-full_auto2; write(r)<-sortie_runner1 | M | begin/end_sortie by full_auto |
| `TAP` :41 | Mutex<Option<Tap>> | scorecard tap fn | set_tap(w)<-main1; record(r)<-autoland60,procedure52,heli_route46,monitor45,autotakeoff40,circuit39 | S | set_tap once |

#### crates/sim/src/xplane.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `MS` :46 | OnceLock<u64> | value cache TTL | value_cache_ttl(w) | C | once |
| `LOAD` :140 | Mutex<Option<LoadWindow>> | API load window stats | load_snapshot(r)<-health1; note_cache_hit(r); note_named(r); note_request(r) | S | rolled every 30 s |
| `LOAD_GATES` :425 | Mutex<Option<HashMap<String, LoadGate>>> | per-base load gate | with_gate(r); enable_load_gate(r)<-main1; forget_base(r); sim_loading(r)<-pollers8,plugin_status3,recover3,tts1,full_auto1,detect1 | S | enable_load_gate |
| `LOAD_CHECKING` :428 | Mutex<()> | serialise load check | hold_for_load(r) | P | lock only |
| `ID_CACHE` :565 | LazyLock<Mutex<PerBase<i64>>> | dataref id cache | forget_base(w); dataref_id(r); fetch_value(r); set_value(r); dataref_index(r); dataref_exists(r) | S (per plugin load) | forget_base on failure |
| `COMMAND_CACHE` :567 | LazyLock<Mutex<PerBase<i64>>> | command id cache | forget_base(w); command_id(r) | S (per plugin load) | forget_base |
| `VALUE_CACHE` :569 | LazyLock<Mutex<PerBase<(Instant, serde.. | 60 ms value cache | forget_base(w); get_value(r); fetch_value(r); set_value(r); set_float_index(r); run_command_for(r) | C | TTL, writes evict |
| `ABSENT` :572 | LazyLock<Mutex<PerBase<Instant>>> | absent dataref memo | forget_base(w); dataref_id(r) | S | forget_base, TTL |

#### src/aircraft.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `FUEL_UNIT` :1311 | Mutex<(f64, FuelSpoken)> | spoken fuel unit | publish_fuel_unit(w)<-flight_state1; fuel_unit(r) | A | publish_airframe |
| `THROTTLE_DREF` :2774 | Mutex<Option<String>> | throttle dataref | publish_throttle_dataref(w)<-flight_state1; throttle_dataref(r)<-full_auto2,takeoff_config2,autotaxi1,circuit1,autoland1,autotakeoff1 | A | publish_airframe |
| `NOSE_CASTORS` :2819 | AtomicBool | castoring nosewheel (.acf) | publish_nose_castors(w)<-detect1,flight_state1; nose_castors(r)<-autotaxi5,autoland1,autotakeoff1,taxi1 | A | publish_airframe false, detect sets |
| `NOSE_FREE` :2832 | AtomicBool | free castoring nose (.acf) | publish_nose_free(w)<-detect1,flight_state1; nose_free(r)<-autotaxi3,autoland2,autotakeoff2,ground_steer1 | A | publish_airframe false, detect sets |
| `GEAR_LIMIT_KTS` :2905 | Mutex<Option<f64>> | Vlo | publish_config_limits(w)<-flight_state1; gear_limit_kts(r)<-procedure1,circuit1,autoland1,autotakeoff1 | A | publish_airframe |
| `FLAP_LIMITS` :2906 | Mutex<Vec<(f64, Option<f64>)>> | flap placards | publish_config_limits(w)<-flight_state1; flap_limits(r)<-circuit10,autoland7,procedure2,monitor2 | A | publish_airframe |
| `BUS_VOLTS_NOMINAL` :2918 | Mutex<Option<f64>> | nominal bus volts | publish_bus_volts(w)<-flight_state1; min_start_volts(r)<-engine1 | A | publish_airframe |
| `SPEED_FLOOR_KTS` :2943 | Mutex<Option<f64>> | 1.2 Vs clean | publish_speed_floor(w)<-flight_state1; speed_floor_kts(r)<-procedure5,speedhold3,full_auto2,plan_audit2,answers1,autotakeoff1 | A | publish_airframe |
| `SPEED_CEILING_KTS` :2971 | Mutex<Option<f64>> | Vne - 15 | publish_speed_ceiling(w)<-speedhold2,flight_state1; speed_ceiling_kts(r)<-speedhold4,procedure1,heli_route1,setup_enroute1 | A | publish_airframe |
| `TAKEOFF_MARGIN_WAIVED` :3060 | AtomicBool | runway 1.4 waiver for this mission | direct: full_auto5; takeoff_factor(r) | M | full_auto start/abort/stop (owned by full_auto, declared in aircraft) |
| `APPROACH_FLOOR_KTS` :3163 | Mutex<Option<f64>> | landing-config floor | publish_approach_floor(w)<-flight_state1; approach_floor_kts(r)<-autoland2,speedhold2,procedure1 | A | publish_airframe |
| `STALL_CURVE` :3186 | Mutex<Option<StallCurve>> | per-notch stall table | publish_stall_curve(w)<-flight_state1; stall_curve(r) | A | publish_airframe |
| `APPROACH_THRUST_MIN` :3252 | Mutex<Option<f64>> | approach thrust floor | publish_approach_thrust_min(r)<-flight_state1; approach_thrust_min(r)<-autoland3 | A | publish_airframe |
| `ARREST_LEAD` :3270 | Mutex<ArrestLeadProfile> | arrest lead profile | publish_arrest_lead(r)<-flight_state1; arrest_lead(r)<-autoland1 | A | publish_airframe |
| `LIVE_PROFILE` :3292 | Mutex<Option<Arc<AircraftProfile>>> | the live AircraftProfile Arc | publish_live_profile(w)<-flight_state1; live_profile(r)<-autotakeoff7,autoland6,circuit3,ils3,plan_audit2,anti_ice2 | A | publish_airframe |
| `FLARE_ELEVATOR` :3305 | Mutex<Option<FlareElevatorProfile>> | flare elevator profile | publish_flare_elevator(r)<-flight_state1; flare_elevator(r)<-autoland2 | A | publish_airframe |
| `ROLL_RUDDER` :3321 | Mutex<Option<RollRudderProfile>> | roll rudder profile | publish_roll_rudder(w)<-flight_state1; roll_rudder(r)<-autotakeoff1 | A | publish_airframe |
| `LANDING_DISTANCE_M` :3336 | Mutex<Option<f64>> | book landing distance | publish_landing_distance(w)<-flight_state1; landing_distance_m(r)<-autoland4,plan_audit1 | A | publish_airframe |

#### src/answers.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `NEXT` :794 | AtomicUsize | rotation index for systems check | systems_check(w) | P | never |

#### src/approach_guide.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `GUIDE` :31 | Mutex<Option<ApproachPath>> | armed talk-down path | direct: station1; arm(w)<-procedure1,approach1; cancel(r)<-monitor2,flight_state1,approach1; armed(r)<-monitor1,auto_status1 | L | cancel in stand_down |

#### src/auto_status.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `AP_TARGETS` :67 | Detail | panel AP targets | set_ap_targets(w)<-procedure1,circuit1,autotakeoff1; clear_ap_targets(w); panel_extras(r) | L | clear_ap_targets |
| `LAST_SAID` :121 | Detail | panel last line said | note_said(w)<-voice5; panel_extras(r) | S | overwritten |

#### src/autoland.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `TRACKING_ERROR` :18 | Mutex<Option<f64>> | autoland last cross-track (m) | last_tracking_error(r)<-monitor1; spawn_autoland(w) | L (used as F) | reset only at next autoland arm; read unconditionally by monitor |
| `ABORT` :398 | AtomicBool | autoland cancel latch | direct: full_auto1,procedure1,circuit2,full_auto1,autoland1,setup1; abort(w)<-instructor1,full_auto1,procedure1,circuit1,flight_state1,autopilot1; spawn_autoland(r) | L | callers clear it before launch (circuit, full_auto, procedure, tools) |
| `ACTIVE` :400 | AtomicBool | autoland flying | direct: instructor1,flow5,full_auto9,autotaxi1,monitor12,cabin2,ils1,go_around1,auto_status2,full_auto1,approach1,go_around1,speedhold3,autoland1,engine2,setup2,station1; last_tracking_error(r)<-monitor1; abort(w)<-instructor1,f.. | L | abort/stand_down |
| `LANDING_CONFIGURED` :403 | AtomicBool | autoland set landing config | landing_configured(r)<-monitor1; spawn_autoland(w) | L | arm clears |
| `GEN` :413 | AtomicU64 | autoland launch generation | spawn_autoland(w) | L | monotonic |
| `PANEL` :418 | Mutex<String> | autoland/recover panel line | direct: monitor1,auto_status2; set_panel(r)<-recover7 | L | autoland and recover::set_panel both write |
| `VACATE_FOLLOWS` :1197 | AtomicBool | taxi-in follows the landing | direct: full_auto2; abort(w)<-instructor1,full_auto1,procedure1,circuit1,flight_state1,autopilot1; spawn_autoland(w) | M | abort clears; full_auto sets |
| `LANDED_RUNWAY` :1210 | Mutex<Option<String>> | runway the wheels touched | direct: flight_state1; landed_runway(r)<-taxi_in1; spawn_autoland(r) | F | overwritten per landing; NEVER cleared |
| `GLIDEPATH_DEG` :5089 | Mutex<(f64, bool)> | live path angle of the arm | live_glidepath(r) | L | each arm |
| `NEXT_GATE` :5118 | Mutex<Option<stable_approach::GateKind>> | stabilised gate handed to next arm | set_next_gate(w)<-circuit2; arm_gate(r) | L (handoff) | consumed at arm, AFTER an early cancelled() return |
| `NEXT_GLIDEPATH_DEG` :5137 | Mutex<Option<(f64, bool)>> | path angle handed to next arm | next_glidepath(r) | L (handoff) | consumed at arm, after cancelled() return |
| `LAST_APPR_READS` :6327 | Mutex<(Option<f64>, Option<f64>)> | last APPR status reads (add-on learning) |  | L | loop |
| `RUNWAY_WIDTHS` :7384 | Mutex<Option<HashMap<(String, String),.. | apt.dat runway widths memo |  | C | never |

#### src/autotakeoff.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `GEAR_WARNING` :489 | Mutex<Option<aircraft::GearWarning>> | gear warning thresholds (.acf) | publish_gear_warning(w)<-detect1,flight_state1; gear_warning(r)<-circuit1 | A | publish_airframe, detect |
| `RUN` :533 | sim::run::Slot | takeoff Slot | running(r)<-full_auto2,instructor1,circuit1; outcome(r)<-full_auto1,circuit1; cancel(r)<-instructor1,full_auto1,circuit1,flight_state1,main1; start_scoped(w)<-circuit1,takeoff1; reject(r); no_pitch_take_over(r); fly(r) | L | cancel |
| `ACTIVE` :545 | AtomicBool | takeoff flying | direct: instructor1,flow1,full_auto1,monitor5,auto_status2,crate1,takeoff2,taxi1,setup1; cancel(w)<-instructor1,full_auto1,circuit1,flight_state1,main1; status(r)<-auto_status1; start_scoped(rw)<-circuit1,takeoff1; fly(rw) | L | cancel/stand_down |
| `REJECT` :548 | AtomicBool | reject requested | direct: takeoff1; start_scoped(w)<-circuit1,takeoff1; fly(w) | L | start clears |
| `DETAIL` :553 | auto_status::Detail | panel detail | status(r)<-auto_status1; fly(rw) | L | fly |
| `DEPARTURE_MINIMUMS` :2666 | AtomicBool | SID minimums own the climb | direct: full_auto3,flight_state1; fly(r) | M | full_auto writes 3x, publish_airframe clears; declared in autotakeoff |
| `CLIMB_OUT` :2689 | Mutex<Option<ClimbOut>> | jet climb-out profile | publish_climb_out(w)<-flight_state1; climb_out_for(r) | A | publish_airframe |
| `TAKEOFF_POWER` :2693 | Mutex<Option<aircraft::TakeoffPowerPro.. | takeoff power profile | publish_takeoff_power(w)<-flight_state1; takeoff_power(r) | A | publish_airframe |
| `CLIMB_HANDOVER` :2914 | Mutex<Option<ClimbHandover>> | takeoff to circuit handover | publish_climb_handover(w); take_climb_handover(r)<-circuit1 | L (handoff) | None at fly start, Some at handoff; taken by circuit |

#### src/autotaxi.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ACTIVE` :362 | AtomicBool | taxi drive running | direct: taxi_in2,instructor2,full_auto1,monitor2,flight_state1,crate2,takeoff1,circuit1,taxi4; driving(r)<-taxi_in5,full_auto5,monitor2,auto_status1; halt(w)<-taxi_in3; status(r)<-auto_status1; start(w)<-taxi2,taxi_in1; start_ba.. | L | halt/stand_down/full_auto::abort/taxi_in (6 ext writers) |
| `BACKTRACKING` :368 | AtomicBool | backtrack drive running | direct: flight_state1; driving(r)<-taxi_in5,full_auto5,monitor2,auto_status1; halt(w)<-taxi_in3; start_backtrack(w)<-taxi1 | L | halt/stand_down |
| `LAST_ARRIVED` :387 | AtomicBool | last drive reached its hold/stand | direct: taxi_in3,full_auto2,monitor1,flight_state1; halt(w)<-taxi_in3; start(w)<-taxi2,taxi_in1; start_backtrack(r)<-taxi1; drive(w) | F | next drive start/halt; taxi_in clears; NOT on new flight |
| `RELOCATED` :393 | AtomicBool | drive saw a relocation jump | direct: taxi_in1; start(w)<-taxi2,taxi_in1; drive(w) | L | start clears |
| `PROGRESS` :504 | Mutex<ProgressWatch> | ProgressWatch for drive_stalled | progress_note(r); progress_rebase(r); progress_hold(r); drive_stalled(r)<-full_auto2; start(r)<-taxi2,taxi_in1 | L | start rebases |
| `EPOCH` :509 | OnceLock<Instant> | clock epoch | clock_s(w) | P | never |
| `DETAIL` :549 | auto_status::Detail | panel detail | status(r)<-auto_status1; drive(rw) | L | drive |
| `DRIVE_GEN` :2944 | AtomicU64 | drive generation | drive_gen(r)<-full_auto2; start(w)<-taxi2,taxi_in1; align_on_the_runway(w); drive(w) | L | monotonic |

#### src/cabin.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `STAFFING` :154 | AtomicU8 | cabin crew staffing setting | set_staffing(w)<-main1,settings1,pollers1; staffing(r)<-health1,settings1,pollers1 | S | settings |

#### src/circuit.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ACTIVE` :1413 | AtomicBool | circuit flying | direct: instructor1,flow4,full_auto3,monitor3,ils1,go_around1,auto_status2,crate1,approach1,circuit1,go_around1,speedhold1,setup1; status(r)<-auto_status1; abort(w)<-instructor1,full_auto1,flight_state1,main1,circuit1; give_up_p.. | L | abort/stand_down; 9 write sites in-module |
| `ABORT` :1415 | AtomicBool | circuit cancel latch | abort(w)<-instructor1,full_auto1,flight_state1,main1,circuit1; hold_clear_of_terrain(r)<-reversal1; fly_authored_route(rw); start(w)<-circuit1; fly(rw) | L | start clears |
| `DETAIL` :1421 | auto_status::Detail | panel detail | status(r)<-auto_status1; fly(rw) | L | fly |
| `CREW_RADIO` :1449 | Mutex<VecDeque<Queued>> | queued crew radio calls | take_crew_radio(r)<-main1; crew_radio_pending(r)<-full_auto1; enqueue_as(r) | F | drained by main; stale filter at send; NOT cleared on new flight |
| `LAST_READBACK` :1528 | Mutex<Option<ReadbackRec>> | last crew readback (queued/out) | last_readback(r)<-full_auto2; note_readback_out(r)<-main3; transmit_readback(w)<-full_auto7,heli_route1 | F | overwritten per readback; NOT cleared on new flight |
| `AP_ALTITUDE_CAPTURE` :1879 | AtomicBool | profile ap_altitude_capture | publish_ap_altitude_capture(w)<-flight_state1; fly_authored_route(r); fly(r) | A | publish_airframe |
| `LAST_BUG` :2121 | AtomicU64 | last heading bug written | write_bug(w); fly_authored_route(w); fly(w) | L | loop |
| `STOOD_DOWN` :4293 | Mutex<Option<&'static str>> | why circuit stood down | hold_clear_of_terrain(w)<-reversal1; start(w)<-circuit1; stood_down(r)<-full_auto2; stand_down(w) | L | start clears |

#### src/climb_watch.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `EPOCH` :577 | OnceLock<Instant> | clock epoch | now_s(w) | P | never |
| `TELEMETRY` :581 | Mutex<Option<(f64, PitchTelemetry)>> | pitch telemetry from monitor | publish_telemetry(w)<-monitor1; telemetry(r) | S (tick-fresh, stamped) | monitor |
| `SIM_SPEED` :594 | Mutex<Option<(f64, f64)>> | sim speed from monitor | publish_sim_speed(w)<-monitor1; fast_forward(r) | S (tick-fresh, stamped) | monitor |
| `ARMED` :619 | Mutex<Option<Armed>> | one armed pitch-authority check | arm_authority(w)<-procedure1,circuit1,autoland1,autotakeoff1,setup_enroute1; feed_authority(r)<-procedure1,circuit1,autoland1,autotakeoff1,setup_enroute1; disarm_authority(w)<-flight_state1,autoland1 | L | disarm in stand_down/autoland; last arm wins across 5 loops |

#### src/cockpit_auto.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `PILOT_MOVED_LIGHTS` :147 | AtomicBool | pilot moved a light switch | direct: flight_state1; note_pilot_moved_lights(w)<-lights1; take_pilot_moved_lights(w)<-monitor1 | F | take_ swap by monitor |

#### src/detect.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `PROBE_INDEX` :348 | Mutex<Option<ProbeIndex>> | DataRefs/Commands index per session | missing_profile_names(w) | S | never (by design) |

#### src/flight.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `PILOT_TAKEOVER` :243 | Mutex<Option<(f64, &'static str)>> | pilot took control mailbox (duty engine) | note_pilot_takeover(w)<-autotaxi2,autotakeoff2,instructor1,handover1,monitor1,main1; note_crew_takeover(w)<-handover1,main1,full_auto1,setup1; pilot_takeover(r)<-monitor1 | F | note_crew_takeover only; NOT on new flight |
| `PUBLISHED` :1492 | Mutex<Option<Status>> | flight model status snapshot | publish(w)<-monitor1; published(r)<-monitor2,main2 | S (tick-fresh) | monitor republishes each tick |

#### src/flow.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ACTIVE` :76 | AtomicBool | a checklist flow is running | direct: instructor1,monitor1,auto_status1; automation_flying(r)<-monitor2; mission_owner(r); stand_down(r)<-setup2,full_auto1,monitor1,setup_enroute1; phase_boundary(r)<-monitor1; take_reply(r)<-main1; abort(r)<-flight_state1; s.. | L | flow::abort sets ABORT only |
| `ABORT` :77 | AtomicBool | stop the running flow | stand_down(w)<-setup2,full_auto1,monitor1,setup_enroute1; classify(rw)<-main1; abort(w)<-flight_state1; start(w)<-main6,instructor1,takeoff_config1; wait_for(r); confirm(r); run(r) | L | start clears |
| `WAITING` :105 | Mutex<Option<Waiting>> | item awaiting a pilot reply | take_reply(r)<-main1; start(w)<-main6,instructor1,takeoff_config1; wait_for(w); confirm(w) | L | start/wait_for |
| `REPLY` :106 | Mutex<Option<Reply>> | pilot reply mailbox | take_reply(w)<-main1; start(w)<-main6,instructor1,takeoff_config1; take_pending_reply(w) | L | take_reply |
| `LISTS` :110 | Mutex<Option<Arc<Vec<Checklist>>>> | installed XChecklist sections | install(w)<-main3; lists(r) | A | flow::install on boot and swap (main.rs) |
| `OVERRIDES` :111 | Mutex<Vec<(String, String)>> | profile checklist_flow title map | install(w)<-main3; find_section(r)<-instructor1,takeoff_config1 | A | flow::install on swap |
| `PROPOSAL` :114 | Mutex<Option<(usize, Instant)>> | pending copilot proposal with TTL | install(w)<-main3; stand_down(w)<-setup2,full_auto1,monitor1,setup_enroute1; phase_boundary(w)<-monitor1; proposal(r)<-main1; decline_proposal(w)<-main1; start(w)<-main6,instructor1,takeoff_config1 | F | flow::stand_down, install, decline |
| `RUNNING` :117 | Mutex<Option<Flavour>> | flavour of the running list | stand_down(r)<-setup2,full_auto1,monitor1,setup_enroute1; start(w)<-main6,instructor1,takeoff_config1 | L | start |
| `STOOD_DOWN` :119 | Mutex<Option<StandDown>> | why the list was stood down | stand_down(r)<-setup2,full_auto1,monitor1,setup_enroute1; start(w)<-main6,instructor1,takeoff_config1; run(r) | L | start clears |
| `DETAIL` :123 | auto_status::Detail | panel detail line | status(r)<-auto_status1; start(w)<-main6,instructor1,takeoff_config1; run(w) | L | start/run |

#### src/free_nose_replay.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `NET` :574 | OnceLock<Option<(atc::nav::TaxiNet, Ve.. | LEMG taxi net for replay tool | lemg(w) | C | never (memo) |

#### src/full_auto.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `RUN` :43 | sim::run::Slot | mission Slot (lease) | running(r)<-instructor1,monitor1,full_auto1,setup1; abort(r)<-main3,full_auto2,instructor1,flight_state1; start(w)<-full_auto1; give_up(r); taxi_in_after_landing(r) | M | Slot cancel |
| `ACTIVE` :48 | AtomicBool | mission running | direct: instructor1,flow3,monitor4,ils1,auto_status2,crate1,approach1,setup2; status(r)<-auto_status1; abort(w)<-main3,full_auto2,instructor1,flight_state1; stop_active(w); start(rw)<-full_auto1; wait_status(r)<-auto_status1; fl.. | M | abort/stop_active |
| `ABORT` :50 | AtomicBool | mission cancel latch | abort(w)<-main3,full_auto2,instructor1,flight_state1; start(w)<-full_auto1; give_up(r); get_ifr_clearance(r); wait_enroute_working_the_radio(r); wait_until(r); wait_takeoff_working_the_radio(r); wait_for_drive(r); fly(r); fly_ro.. | M | start clears |
| `STAGE` :54 | Mutex<String> | mission stage name | stage(r); clear_stage(r); event(r)<-full_auto11,sortie_runner1 | M | clear_stage in abort |
| `MISSION` :58 | Mutex<&'static str> | mission kind (circuits etc) | clear_stage(r); status(r)<-auto_status1; start(r)<-full_auto1; flying_circuits(w) | M | clear_stage |
| `ARRIVAL_RUNWAY` :72 | Mutex<Option<String>> | the one arrival runway | arrival_runway(r); set_arrival_runway(r); retarget_arrival_runway(r)<-autoland3 | M | set_arrival_runway(None) at mission end |
| `WAIT` :399 | auto_status::Detail | panel wait detail | clear_stage(w); waiting(w); wait_status(r)<-auto_status1; event(w)<-full_auto11,sortie_runner1 | M | clear_stage |
| `INBOUND_MADE` :1287 | AtomicBool | inbound call made this mission | direct: flight_state1; start(w)<-full_auto1; wait_enroute_working_the_radio(r); fly_route(r); call_inbound(w); tower_arrival_runway(r) | M | start clears |
| `WAIVER` :6541 | Mutex<Option<Waiver>> | spoken margin waiver (120 s, per plan) | waived_for(r)<-plan_audit1,full_auto1; take_waiver(r)<-main1 | M (pre-arm) | TTL + take_waiver |
| `REFUSED` :6544 | Mutex<Option<(Gate, String, String, In.. | last refused gate awaiting waiver | note_refusal_for(w); take_waiver(w)<-main1 | M (pre-arm) | TTL |
| `RUNWAY_WAIVER_PENDING` :6548 | AtomicBool | waiver to hand the next start | direct: full_auto2; start(w)<-full_auto1 | M (pre-arm) | start swaps it; written from tools/full_auto too |

#### src/go_around.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `PILOT_ORDERS` :21 | AtomicU64 | pilot go-around order counter | order(w)<-go_around1; answer(r)<-circuit1,autoland1; arm(r); take(r) | P (counter) | monotonic, watchers snapshot |
| `ANSWER` :24 | Mutex<Option<(u64, Answer)>> | loop answer to an order | answer(r)<-circuit1,autoland1; answer_to(r) | L | keyed by order number |
| `LAST_TOWER_ORDER` :29 | Mutex<Option<Instant>> | tower order stamp (readback window) | note_tower_order(w)<-main1; note_radio_call(r)<-main3 | P (timestamp) | window |

#### src/health.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `PLUGIN_MISSING` :167 | Mutex<Option<Vec<String>>> | plugin datarefs missing at boot | set_plugin_missing(w)<-detect1; plugin_missing(r)<-detect2 | S (per plugin load) | detect |

#### src/heli_circuit.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ACTIVE` :26 | AtomicBool | heli circuit flying | direct: flow1,monitor2,crate1,heli2; status(r)<-auto_status1; abort(w)<-flight_state1,main1; start(w)<-heli1; fly(r) | L | abort/stand_down |
| `DETAIL` :29 | auto_status::Detail | panel detail | status(r)<-auto_status1; fly(rw) | L | loop |

#### src/heli_route.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ACTIVE` :37 | AtomicBool | heli route running | direct: flow1,monitor3,crate1,heli2,setup1; abort(w)<-flight_state1,main1; status(r)<-auto_status1; park(w)<-taxi_in1; fly_park(rw); start(w)<-heli2; before_route(r); fly(rw) | L | abort/stand_down |
| `GENERATION` :38 | AtomicU64 | heli route generation | abort(w)<-flight_state1,main1; park(w)<-taxi_in1; fly_park(w); start(w)<-heli2; fly(w) | L | abort bumps |
| `DETAIL` :45 | auto_status::Detail | panel detail | status(r)<-auto_status1; fly_park(rw); call_departure(w); before_route(w); fly(rw) | L | loop |
| `INBOUND_SENT` :1241 | AtomicBool | heli inbound call made | call_inbound(w); fly(rw) | M | fly() clears at start |

#### src/hover.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ACTIVE` :39 | AtomicBool | hover loop flying | direct: flow1,heli_route8,monitor2,crate1,heli_circuit1,heli9,setup1; depart_turn(r)<-main1; depart_speed(r)<-main1,heli1; retarget(r)<-heli_route8,heli_circuit3,heli2; status(r)<-auto_status1; release(rw)<-heli_route2; abort(w).. | L | abort/stand_down |
| `GOAL` :86 | Mutex<Option<Goal>> | hover goal | lock_goal(r) | L | request |
| `MOTION` :106 | Mutex<Motion> | hover motion state | lock_motion(r) | L | loop |
| `TURN_MAG` :171 | Mutex<Option<f64>> | depart turn magnitude | depart_turn(w)<-main1; fly(w) | L | loop |
| `ROTOR` :340 | Mutex<Option<(String, f64, String)>> | rotor rpm dref, governed rpm, collective dref | publish_rotor(w)<-heli1; rotor(r); collective_dref(r)<-taxi_in1,taxi1 | A | set by heli::Hover::new at registry build; NEVER cleared on swap to non-heli |
| `RELEASED` :454 | AtomicBool | hover released to cruise AP | release(w)<-heli_route2; fly(w) | L | loop |

#### src/ils.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `SETUP_GEN` :589 | AtomicU64 | ILS setup watch generation | stop_setup_watch(w)<-flight_state1; spawn_setup_watch(w)<-approach1 | L | stand_down bumps |

#### src/instructor.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `LESSON` :1670 | Mutex<Lesson> | current ACS lesson, briefed numbers | direct: flight_state1; lesson(r) | S/A | instructor::install(profile) abandons on swap; not on new flight |
| `START` :2802 | OnceLock<Instant> | instructor clock epoch | clock_s(r)<-monitor1,main1 | P | never |

#### src/main.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `LISTEN_OFF` :3902 | AtomicBool | mic listening off | main(w); listen_once(r) | S | main |
| `LAST_READBACK` :4042 | Mutex<Option<(String, Instant)>> | copilot last readback for say again | run_session(r); say_or(r) | A | cleared on swap only |

#### src/mode.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `MODE` :79 | AtomicU8 | app mode (both/copilot/atc/instructor) | set(w)<-voice7,instructor5,main1,settings1,pollers1; get(r)<-main2,health1,settings1,pollers1 | S | mode::set from 5 modules |
| `DEMONSTRATION` :138 | Mutex<Option<Demonstration>> | instructor show-me window | demonstration(r) | L | DEMONSTRATION_ARM_S |

#### src/monitor.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `DESTINATION` :113 | Mutex<Option<Destination>> | plan destination for TOD ring and answers | direct: plan1,crate1,answers2; read(r) | F (plan) | only overwritten by plan::ensure_plan; NEVER cleared |
| `PANEL_MUTE` :2561 | AtomicBool | panel mute switch | active(r); run(rw)<-main1 | S | monitor run |
| `EPOCH` :2753 | OnceLock<Instant> | process clock | process_secs(w) | P | never |
| `CREW_YOKE_MS` :2758 | AtomicU64 | last crew yoke write stamp | note_crew_yoke(w)<-autoland4,autotakeoff2,recover1; crew_yoke_fresh(r) | P (timestamp) | self-expiring |
| `PILOT_RELEASE_MS` :2777 | AtomicU64 | last pilot release stamp | note_pilot_release(w)<-autopilot1; pilot_released_recently(r) | P (timestamp) | self-expiring |
| `PREV` :2862 | Mutex<[bool; 4]> | previous intervention inputs | note_interventions(r) | S | tick |
| `BRAKE_LATCH` :2863 | Mutex<CrewBrakeLatch> | CrewBrakeLatch discount | note_interventions(r) | S | tick |
| `CALLSIGN_STALE` :5640 | AtomicBool | callsign setting changed, republish |  | S | consumed by tick |
| `LAST` :5654 | Mutex<Option<(Instant, String)>> | last surface-wind feed (time, icao) |  | W | compared per station |

#### src/plan.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ELEV_CACHE` :456 | Mutex<Option<(String, f64)>> | destination elevation cache | ensure_plan(r) | C | keyed by icao |

#### src/pollers.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `QUEUED` :113 | Mutex<VecDeque<Instant>> | queued typed line stamps | queued(r) | S | drained |
| `UI_SCALE` :250 | AtomicI64 | panel ui scale | spawn_settings_poller(rw) | S | poller |

#### src/procedure.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `RUN` :92 | sim::run::Slot | follower Slot | running(r)<-full_auto2; outcome(r)<-full_auto1; abort(r)<-full_auto2,circuit1,flight_state1,main1,procedure1; start_scoped(w)<-full_auto2,circuit1,autoland1,procedure1; stand_down(w); fly(r)<-course_leg1 | L | Slot cancel |
| `ACTIVE` :100 | AtomicBool | follower flying | direct: flow3,full_auto5,monitor5,ils1,auto_status2,crate1,approach1,procedure1,autopilot2,speedhold1,setup1; active_leg(r)<-answers3,reminder3,full_auto1,monitor1; fix_distance_nm(r)<-full_auto1; route_ahead(r)<-full_auto1; off.. | L | abort/stand_down |
| `ABORT` :102 | AtomicBool | follower cancel latch | abort(w)<-full_auto2,circuit1,flight_state1,main1,procedure1; start_scoped(w)<-full_auto2,circuit1,autoland1,procedure1; fly(rw)<-course_leg1 | L | start_scoped clears |
| `LEG` :116 | Mutex<Option<ActiveLeg>> | active leg (read gated on ACTIVE) | active_leg(r)<-answers3,reminder3,full_auto1,monitor1; publish_leg(w) | L | publish_leg(None) at end |
| `FIX_DIST` :130 | Mutex<Option<f64>> | nm to active fix (gated on ACTIVE) | fix_distance_nm(r)<-full_auto1; publish_fix_dist(w) | L | publish(None) at end |
| `ROUTE_AHEAD` :182 | Mutex<Option<(usize, Vec<ProcLeg>)>> | legs as flown (gated) | route_ahead(r)<-full_auto1; publish_route_ahead(w) | L | publish(None) at end |
| `SPLICE` :210 | Mutex<Option<Splice>> | offered STAR splice mailbox | offer_splice(w)<-full_auto2; take_splice(w) | L | take_splice; offer refused unless ACTIVE |
| `TOD` :248 | Mutex<Option<PlannedTod>> | planned top of descent (gated) | direct: flight_state1; planned_tod(r)<-monitor2,plan1; publish_tod(w) | L | publish_tod(None) at end |
| `DETAIL` :337 | auto_status::Detail | panel detail | status(r)<-auto_status1; fly(rw)<-course_leg1 | L | loop |
| `MISSED_LEGS` :481 | Mutex<Option<Vec<ProcLeg>>> | published missed approach handed to autoland | take_missed_legs(w)<-autoland1; stash_missed_legs(w) | F (handoff) | take only on steep go-around; NOT cleared otherwise |
| `NEXT_ARRIVAL_HANDOFF_KTS` :1218 | Mutex<Option<f64>> | speed for the next arrival handoff | set_next_arrival_handoff_kts(r)<-full_auto2; take_arrival_handoff_kts(r) | M (handoff) | taken by next follower; set None on busy |
| `TERRAIN_FLOOR` :1405 | Mutex<Option<f64>> | terrain floor ahead (gated) | terrain_floor_ahead_ft(r)<-autopilot2; publish_terrain_floor(w) | L | publish(None) at end |
| `PILOT_ALT` :1431 | Mutex<Option<f64>> | pilot-held altitude (gated) | take_altitude(w)<-autopilot2,monitor1; pilot_altitude(r); clear_pilot_altitude(w) | L | clear_pilot_altitude at end |
| `CLEARED_CEILING` :1461 | Mutex<Option<f64>> | climb capped at cleared altitude | hold_at_cleared(r)<-full_auto4; cleared_ceiling(r)<-full_auto1 | M | hold_at_cleared(None) at check-in/stand_down/follower end |
| `PILOT_SPEED` :1508 | Mutex<Option<f64>> | pilot-held speed (gated) | clear_pilot_altitude(w); take_speed(w)<-speedhold1; pilot_speed(r) | L | clear_pilot_altitude |
| `CLIMB_FPM` :1560 | Mutex<f64> | airframe climb rate | publish_vs_limits(r)<-full_auto2,flight_state1; climb_fpm(r)<-circuit1,plan_audit1,plan1,autoland1 | A | publish_vs_limits from publish_airframe |
| `DESCENT_FPM` :1561 | Mutex<f64> | airframe descent rate | publish_vs_limits(r)<-full_auto2,flight_state1; descent_fpm(r)<-autoland5,circuit2,plan_audit1,plan1 | A | publish_airframe |
| `BEST_CLIMB_FPM` :1562 | Mutex<Option<f64>> | airframe best climb | publish_vs_limits(r)<-full_auto2,flight_state1; best_climb_fpm(r)<-full_auto2,plan_audit1,autotakeoff1 | A | publish_airframe |
| `DRAG_OURS` :1821 | AtomicBool | speedbrake out is ours | start_scoped(w)<-full_auto2,circuit1,autoland1,procedure1; fly(rw)<-course_leg1 | L | start_scoped clears |

#### src/pushback.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `LEARNED_SENSE` :971 | AtomicI8 | learned tug turn sense of this X-Plane build | direct: flight_state1; run_inner(rw) | S (by design) | never; flight_state doc lists it as per flight, it is session |
| `ACTIVE` :979 | AtomicBool | push running | direct: monitor2,auto_status1,detect1,flight_state1,crate3,taxi_guide1,ground1; status(r)<-auto_status1; run(w)<-full_auto1 | L | run/stand_down; main and ground write ABORT/SKIP |
| `STEERED` :988 | AtomicBool | push is steered (stop routing) | steered(r)<-main1,ground1; run(w)<-full_auto1 | L | run clears at end |
| `ABORT` :1000 | AtomicBool | push abort latch | direct: crate1; stopping(r); run(rw)<-full_auto1; abort_brakes(w) | L | run clears; main/ground set |
| `SKIP` :1006 | AtomicBool | skip the pushback | direct: ground1; run(w)<-full_auto1; run_inner(rw) | L | run consumes; ground sets |
| `TUG_WARNED` :1716 | Mutex<Option<Instant>> | tug warning stamp for autotaxi | run_inner(w); take_tug_warned(r)<-autotaxi1 | L | take_tug_warned |

#### src/rapport.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `LAST_LANDING_BY_AUTOMATION` :439 | Mutex<Option<bool>> | who flew the last landing | touch_down(w); social_facts(r)<-persona1 | F | overwritten per landing |
| `OFFER` :440 | Mutex<Option<(Offer, Instant)>> | pending rapport offer | tick(w); offer(r)<-main1; clear_offer(w)<-main1 | F | TTL / clear_offer |

#### src/readback.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `STATE` :27 | AtomicU64 | ack rng state | rng_next(rw) | P | never |
| `LAST` :118 | AtomicU8 | last ack prefix | next_prefix(rw) | P | never |

#### src/scorecard.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `LIVE` :1068 | Mutex<Option<Scorer>> | live Scorer | tap(r)<-main1 | S | Scorer::new_flight handles repositions itself |

#### src/settings.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `LIVE` :31 | OnceLock<RwLock<Arc<Config>>> | live Config Arc | cell(w) | S | settings::live swap |

#### src/simbrief.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `SHELF` :36 | Mutex<Option<FlightPlan>> | the plan on the shelf | publish_plan(w)<-plan2,flightplan2,full_auto2,loading1,setup_enroute1; loaded_plan(r)<-full_auto3,monitor2,heli2,setup_enroute2,setup2,circuit1; local_plan(r)<-flightplan2,plan1,full_auto1,loading1,heli1,setup_enroute1 | S (plan) | replaced by explicit fetch/plan; survives swap on purpose |

#### src/systems.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `CURRENT` :395 | Mutex<Option<Systems>> | systems survey from .acf | note_mode(r)<-autoflight2; publish(w)<-detect1,flight_state1; current(r)<-anti_ice2,main1,approach1 | A | publish_airframe -> None, detect fills |

#### src/takeoff_config.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `AIRFRAME` :65 | Mutex<Option<aircraft::TakeoffConfigPr.. | takeoff_config profile | publish(w)<-flight_state1; airframe(r) | A | publish_airframe |

#### src/taxi_in.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ARREST_HOLD` :44 | AtomicBool | taxi-in holding the rollout arrest, monitor stays quiet | direct: monitor2,flight_state1; arrest_holding(r)<-monitor2; arrest_release(w); fly(w) | L | released by every taxi_in exit path |
| `ACTIVE` :60 | AtomicBool | taxi-in running | running(r)<-main1; abort(rw)<-flight_state1,main1; begin(w); drop(w); fly(r) | L | stand_down via taxi_in::abort(false) |
| `GEN` :63 | AtomicU64 | taxi-in run generation | begin(w); cancelled(w); drop(w) | L | bumped per begin |
| `CANCEL` :65 | AtomicBool | cancel latch for taxi-in | abort(w)<-flight_state1,main1; begin(w); cancelled(w); end_cancelled(r) | L | abort sets, begin clears |
| `CANCEL_HOLD` :69 | AtomicBool | cancel should brake and park | abort(w)<-flight_state1,main1; end_cancelled(r) | L | abort sets |

#### src/tempo.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `CAPTURE_END` :43 | Mutex<Option<Instant>> | end of pilot capture | note_capture_end(w)<-main1; begin(r) | S | per utterance |
| `TURN` :59 | Mutex<Option<TurnState>> | conversation turn state | begin(w); drop(r); standby_due(r); set_radio_turn(r)<-main2; reaction_pause(r)<-main1 | S | per utterance |
| `TURN_GEN` :60 | AtomicU64 | turn generation | begin(w) | P (counter) | monotonic |
| `WORKLOAD_HIGH` :65 | AtomicBool | busy flag from monitor | set_workload(w)<-monitor1; workload_high(r)<-voice1 | S (tick-fresh) | monitor |

#### src/terrain.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `MAILBOX` :17 | tokio::sync::Mutex<()> | serialises terrain probe mailbox | sample_ft(r)<-circuit2,full_auto2,plan_audit1,reversal1,autoland1,heli1 | P | lock only |

#### src/test_card.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ITEMS` :1858 | &[Item] | test card item table | report(r)<-debrief1 | C | const |

#### src/tools/anti_ice.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `CLAIMED` :617 | Mutex<Vec<IceKind>> | ice protection the crew switched on | claimed(r)<-monitor2; clear_claims(w)<-flight_state1; monitor_fix(r)<-monitor3 | A (should be F) | clear_claims in publish_airframe |

#### src/tools/autopilot.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `CLIMB_WATCHER_GEN` :591 | AtomicU64 | altitude capture watcher generation (20 min loop that writes ALT dial, ALT HOLD, throttle) | capture_at_altitude(w) | L | bumped only by the next capture; NOT in stand_down |

#### src/tools/cabin_air.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `CABIN_AIR` :18 | Mutex<Option<CabinAirProfile>> | cabin_air profile | publish(w)<-flight_state1; declared(r) | A | publish_airframe |
| `MASKS` :212 | Mutex<(MaskWatch, Option<Instant>)> | MaskWatch claim | publish(w)<-flight_state1; masks_tick(r)<-monitor1 | A (should be F) | publish_airframe resets |

#### src/tools/controls.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `BRAKES_HELD` :328 | AtomicBool | crew brake-hold loop running (renews brake command) | brakes_held(r)<-auto_status1,tools1; release_held_brakes(w)<-speedhold1; brake_holder(rw); execute(rw) | L | release_held_brakes, airborne; NOT in stand_down |

#### src/tools/engine.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `MIX_MANAGED` :347 | AtomicBool | mixture manager running | direct: flow1,full_auto1,flight_state1,crate2; execute(w); mixture_status(r)<-auto_status1; mixture_rich_for_landing(w)<-full_auto1; mixture_manager(rw); arm_mixture_manager(w)<-full_auto1 | L | stand_down |
| `LEAN_SEEKING` :352 | AtomicBool | lean seek running | direct: flight_state1,crate2; execute(w); lean_seek_cancelled(r); mixture_status(r)<-auto_status1; mixture_rich_for_landing(w)<-full_auto1; spawn_lean_seek(w) | L | stand_down |

#### src/tools/full_auto.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `SAID` :1842 | AtomicBool | arm hint said once | spoken_arm_line(w) | S | never |

#### src/tools/ground.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `CART_REQUESTED` :140 | Mutex<Option<Instant>> | GPU cart requested stamp | direct: flight_state1; cart_pending(r); execute(rw) | F | TTL/presence; NOT on new flight (listed in flight_state doc) |

#### src/tools/instruments.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ALTIMETER` :108 | Mutex<Option<aircraft::AltimeterProfil.. | altimeter profile | publish_altimeter(w)<-flight_state1; altimeter_profile(r) | A | publish_airframe |
| `ALTIMETER_TRUTH` :333 | Mutex<Option<(Instant, AltimeterTruth)>> | altimeter cross-check truth | publish_altimeter_truth(r)<-monitor1,flight_state1; altimeter_truth_now(r) | A/F (stamped) | publish_airframe None, monitor |
| `FLIGHT_TRANSITION` :502 | Mutex<Option<FlightTransition>> | the flight plan TA/TL in force | publish_flight_transition(w)<-full_auto2,flight_state1,setup_enroute1; note_descending(r)<-procedure1; transition_in_force_ft(r)<-monitor1,main1 | F | publish_airframe None; tools/full_auto + setup_enroute set; NOT on mission end or approach setup |

#### src/tools/lights.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `EXTERIOR` :28 | Mutex<Exterior> | exterior light profile set | publish_exterior(w)<-flight_state1; exterior(r) | A | publish_airframe |

#### src/tools/mod.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `LAST_BRAKE_CMD` :90 | Mutex<Option<Instant>> | last crew brake command stamp | note_brake_cmd(w)<-autotakeoff4,full_auto2,autotaxi1; brake_cmd_quiet(r)<-monitor5,full_auto3,scorecard1,speedhold1 | P (timestamp) | window |

#### src/tools/radio.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `MODE_KNOB` :345 | Mutex<Option<aircraft::TransponderMode.. | transponder mode knob | publish_mode_knob(w)<-flight_state1; set_mode(r)<-taxi_in1,setup_enroute1 | A | publish_airframe |

#### src/tools/recover.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ABORT` :68 | AtomicBool | recovery cancel | execute(w); take_over(w)<-full_auto2,procedure1,circuit1,autoland1,climb_watch1,autotakeoff1; may_write(r); stood_down(r); abort(w)<-flight_state1,autopilot1 | L | abort |
| `ACTIVE` :71 | AtomicBool | recovery flying | direct: full_auto4,monitor4,auto_status2,speedhold2,engine2; execute(r); take_over(r)<-full_auto2,procedure1,circuit1,autoland1,climb_watch1,autotakeoff1; run_recovery(w); finish(w); abort(w)<-flight_state1,autopilot1 | L | abort/stand_down |
| `GEN` :73 | AtomicU64 | recovery generation | run_recovery(w); may_write(r); stood_down(w); finish(w) | L | monotonic |

#### src/tools/reverse.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ACTIVE` :42 | AtomicBool | reverse thrust watcher | direct: flight_state1,crate2; status(r)<-auto_status1; execute(w); watcher(rw) | L | stand_down; main writes |

#### src/tools/setup.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `NEW_FLIGHT_LOADING` :42 | AtomicU32 | app new-flight load in progress (count) | begin(w); drop(w); new_flight_loading(r) | L | Loading guard |
| `OWN_INITS` :44 | AtomicU32 | inits posted by the app | new_flight_loading(r); run_init(w)<-setup_enroute2,sortie_runner1 | S | run_init |
| `KNOWN_INIT_SEQ` :48 | AtomicI64 | last seen init seq | spawn_new_flight_watch(rw)<-main1; run_init(w)<-setup_enroute2,sortie_runner1 | S | fetch_max |
| `LIB` :338 | OnceLock<Vec<(String, String)>> | aircraft library scan | aircraft_library(w)<-sortie_runner1 | C | never |

#### src/tools/signs.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `SIGNS` :34 | Mutex<Option<Signs>> | seat belt sign profile | publish(w)<-flight_state1; is_signs_dataref(r)<-flow1; set(r)<-full_auto3,jet_start1 | A | publish_airframe |

#### src/tools/speedhold.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ACTIVE` :125 | AtomicBool | speed governor running | direct: taxi_in2,instructor1,flow1,autotaxi1,flight_state1,crate5,autotakeoff1,reverse2,taxi1,controls1,setup_enroute1,engine1; taxi_hold_active(w)<-autotaxi3,taxi2,controls2,taxi_in1; flight_target(r)<-procedure2,answers1,setup.. | L | 14 in-module + 9 external writer modules |
| `TARGET_KTS` :307 | Mutex<f64> | governor target | flight_target(r)<-procedure2,answers1,setup_enroute1; taxi_target(r)<-autotaxi2; set_taxi_target(w)<-autotaxi13,taxi_in1,taxi1; arm_flight_with_floor(w)<-procedure1; status(r)<-auto_status1; execute(w); governor(r) | L | autotaxi, taxi_in, taxi, procedure, speedhold |
| `LAST_TAXI_KTS` :310 | Mutex<f64> | last taxi target | execute(rw) | S | execute |
| `MODE` :320 | Mutex<Mode> | taxi vs flight mode | taxi_hold_active(w)<-autotaxi3,taxi2,controls2,taxi_in1; flight_target(w)<-procedure2,answers1,setup_enroute1; arm_flight_with_floor(w)<-procedure1; status(r)<-auto_status1; execute(r); governor(r) | L | several entry points |
| `ACTUAL` :323 | Mutex<f64> | measured speed for panel | status(r)<-auto_status1; governor(w) | L | governor |

#### src/tools/taxi.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `SAID` :28 | AtomicBool | taxi hint said once | take_back_hint(w) | S | never |

#### src/tts.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `THREAD_CHANNEL` :78 | Cell<Channel> | thread-local playback channel tag |  | P | per thread |
| `VOLUME_BITS` :94 | [AtomicU32; 2] | per-channel volume |  | S | set at startup |
| `PLAYER_PID` :117 | Mutex<[Option<u32>; 2]> | audible child pid per channel (barge-in) |  | P | per utterance |
| `CUT_GEN` :118 | [AtomicU64; 2] | barge-in cut generation per channel |  | P | monotonic |
| `AUDIO_BUS` :292 | AtomicBool | voices via sim audio bus |  | S | settings |
| `AUDIO_BUS_PINNED` :293 | AtomicBool | env pinned the bus |  | S | startup |
| `BUS_WARNED` :294 | AtomicBool | one-shot bus failure warning |  | S | never |
| `BUS_BASE_URL` :295 | Mutex<String> | Web API base for bus writes |  | S | startup |
| `BUS_CLIENT` :298 | OnceLock<reqwest::blocking::Client> | blocking HTTP client |  | P | never |
| `BUS_IDS` :302 | [AtomicI64; 2] | cached audio mailbox dataref ids |  | S (per plugin load) | evicted on failed post |

#### src/tts_onnx.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `ORT_SESSION_INIT` :361 | Mutex<()> | serialise ONNX session creation | build_session(r) | P | lock only |

#### src/voice.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `LINES_QUEUED` :515 | AtomicU64 | lines queued counter | lines_queued(r)<-tempo2; push_intercom(w); push(w) | P (counter) | monotonic |
| `PILOT_ENGINE` :1880 | Mutex<Option<(String, String)>> | pilot voice engine/name | publish_pilot_engine(w); pilot_engine(r)<-persona1,health1 | S | startup |

#### src/weather.rs

| Static | Type | Meaning | Writers / readers (fn <- caller modules) | Lifetime | Reset by |
|---|---|---|---|---|---|
| `WINDS_CACHE` :497 | Mutex<Option<(Instant, WindsAloft)>> | winds-aloft download cache | winds_aloft(r)<-monitor1,plan1 | C | TTL |
## 9. Test-only statics (29)

Per-module test locks and fixtures; lifetime T, no action needed except that stage 3/4 thread-local epochs would retire most of the locks:

instructor::{TEST_DEMO_RUNNING, TEST_DEMO_STOPPED}, full_auto::{WAIVER_TEST_LOCK, ARRIVAL_RUNWAY_TEST_LOCK}, circuit::QUEUE_LOCK, go_around::LOCK, auto_status::D, readback::ACK_LOCK, autoland::{GLIDEPATH_DEG_T, NEXT_GLIDEPATH_DEG_T, LOCK}, takeoff_config::LOCK, settings::TEST_LOCK, aircraft::{FUEL_TEST_LOCK, FLOOR_LOCK}, climb_watch::LOCK, mode::TEST_LOCK, autotakeoff::CLIMB_OUT_TEST, tempo::LOCK, tools::{signs::LOCK, cabin_air::LOCK, ground::GPU_TEST_LOCK, radio::KNOB_LOCK, setup::FLYING}, atc::station::{SIDE_STATE_LOCK, PATH x2}, sim::run::ACTIVE, sim::xplane::GATE_TEST_LOCK.
