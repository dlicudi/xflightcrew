# Stage C item 8: per-flight strips (design, 2026-10-08)

Paths under `crates/atc/src/` unless noted. Line numbers as of main 81a5606.

## What the code does today (shapes the design)
- **Flight state lives in two places**: the `Station` worksheet (`station/mod.rs:76-174`, `&mut`,
  owned by the command loop) and statics the monitor's ticks touch (`src/world_tick.rs` via
  `world.rs`), which never see the `Station`. They talk through mailboxes
  (`PENDING_TAKEOFF_CLEARANCE`, `PENDING_LINE_UP`, `PENDING_GO_AROUND`, `PENDING_RESEQUENCE`,
  applied in `respond.rs:298-370`). So a strip has two halves keyed by one `FlightId`: a DESK
  (owned by `Station`) and a BOARD (shared).
- `Tower` already holds only per-aircraft state (`tower.rs:1927-1948`); it moves into the desk
  unchanged. The field context is already `Ctx`.
- **Locks nest** (`note_aircraft` holds `SELF_TAXI` while locking `TAXI_ROUTE`,
  `station/new_flight.rs:288-294`): one mutex per strip would deadlock; keep one per cell.
- Tests touch the statics ~500 times (`CLEARANCE` 134, `COMPLIANCE` 40, `SQUAWK_WATCH` 33) and
  `st.twr` 72 times.
- **The mic is always our flight**: other-addressed calls already drop (`station/routing.rs:29`
  `addressed_to_own`). Routing by callsign matters only for synthesised traffic calls later,
  through their own entry point.

## Target model
```rust
pub enum FlightId { Own, Traffic(u32) }   // Own is never keyed by callsign
pub struct AircraftState { lat: f64, lon: f64, alt_ft: f64, on_ground: bool,
    gs_kts: f64, psi_deg: Option<f64>, helicopter: bool, at: Instant }

// party_line.rs (statics crate-private). A macro generates the struct, NEW and one reset per
// lifetime group, so a new field without a group does not compile.
flight_strip! { pub(crate) struct FlightStrip {
  facts     /* install, uninstall, retire */ { clearance, squawk_nudge, taxi_route,
     taxi_to_stand, takeoff_hold, landing_hold, pending_landing_release, vectors,
     pending_takeoff_clearance, pending_line_up, pending_go_around, resequenced_order,
     line_up_condition, departure_handoff, zone_challenge, landing_handoff,
     assigned_squawk, ifr_clearance },
  worksheet /* same doors */ { squawk_watch, handoff, pending_resequence,
     uncontrolled_arrival, self_taxi, go_around_last },
  carried   /* uninstall, retire, never install */ { ifr_flight, own_fixes },
  watch     /* install, uninstall only */ { compliance_flight },
}}
pub(crate) static OWN_STRIP: FlightStrip = FlightStrip::NEW;
pub(crate) static STRIPS: Mutex<BTreeMap<u32, Arc<FlightStrip>>>; // empty until stage D

pub(super) struct StripDesk { twr: tower::Tower, told_runway, dep_checked_in, handed_to,
   circuits, pilot_info, assigned_squawk, planned_route, start_strip, ifr_runway,
   ifr_bearings, ifr_sid_basis, resequenced, call: CallScratch }
pub(super) struct CallScratch { stated_runway, stated_on_strip, runway_request,
   requested_ident, departing }      // per transmission, consumed by the pick
pub struct Station { /* field */ apt, tower_khz, approach_khz, departure_khz,
   runway_in_use, enforce_readback, dialect, procedures, sid_exits,
   desk: StripDesk,                  // the strip on frequency
   desks: BTreeMap<FlightId, StripDesk> }

pub enum ClaimKind { LineUp, Takeoff, Landing, Crossing, Occupying, Sequenced, Observed }
pub struct Claim { who: Claimant /* Strip(FlightId) | Ghost(String) */, end: String,
   whole_strip: bool, kind: ClaimKind, since: Option<Instant> }
pub struct RunwayBook { claims: Vec<Claim> }   // COMPUTED per query, never stored
```
The book is a pure function over strips' clearances, positions and the ghost memory: no new
static, no new lifetime.

## Where today's state goes
| Current | New home | Why |
|---|---|---|
| `party_line.rs:40-291` CLEARANCE, SQUAWK_NUDGE, TAXI_ROUTE, TAXI_TO_STAND, TAKEOFF_HOLD, LANDING_HOLD, PENDING_LANDING_RELEASE, VECTORS, PENDING_TAKEOFF_CLEARANCE, PENDING_LINE_UP, PENDING_GO_AROUND, RESEQUENCED_ORDER, LINE_UP_CONDITION, DEPARTURE_HANDOFF, ZONE_CHALLENGE, LANDING_HANDOFF, ASSIGNED_SQUAWK, IFR_CLEARANCE | strip `facts` | `PartyLineFacts` group (`state.rs:72`) |
| IFR_FLIGHT (`party_line.rs:259`) | strip `carried` | `FlightCarried` |
| `radar.rs:34` SQUAWK_WATCH, `radar.rs:151` HANDOFF, `strip.rs:678` PENDING_RESEQUENCE, `self_taxi.rs:30` SELF_TAXI, `region.rs:556` UNCONTROLLED_ARRIVAL | strip `worksheet` | `StationFlight` |
| `strip.rs:12` OWN_FIXES | strip `carried` | per aircraft |
| `strip.rs:7` GO_AROUND_LAST | strip `worksheet` **but keep its current lifetime** (`StationField`, not reset by retire) in this stage; changing it is a behaviour change, pinned and decided separately | per aircraft pacing |
| `compliance.rs:93` COMPLIANCE flight part (liftoff, runway_start, landed_strip, prev_on_ground, airborne_since, prev_strip) | strip `watch` | keeps its field lifetime |
| COMPLIANCE cooldowns (last_call, zone_last, off_route_last) | field static | radio cooldowns |
| REGION_HANDOFF, ARRIVAL_PIN, PENDING_STATION, OUT_OF_REACH (`new_flight.rs:267`), GO_AROUND_ORDERS | global, documented "own projection" | crew's swap and reach |
| TOWER_WIND, SURFACE_WIND, ACTIVE_*, ATIS, CONTROLLER_FREQS, session memos | global | field/session facts |
| `wake.rs:18` WAKE, WAKE_PREV, DEPARTURE_STREAM, ARRIVAL_STREAM | field statics, book inputs | runway-scoped |
| GHOSTS (`party_chatter.rs:22`), STRIP_SEEN (`strip.rs:20`), STANDING (`strip.rs:164`) | field statics feeding `Observed` claims | ghost memory |
| own_strip_claim (`party_chatter.rs:66`), player_blocked_strip (`:46`), conflicts_with_claim, runway_occupancy_authorised (`runways.rs:85`) | queries over `RunwayBook`, old names one-line wrappers | |
| `mod.rs:84` twr + 101-139, 159-173 | `StripDesk` | per flight |
| `mod.rs:92/96/105/112/115` | `StripDesk.call` | per transmission |
| `mod.rs:77-83/89/144-156` | `Station` | field |
| `ctx.rs:11-27/58-63/80-85/122-132` own position reads | `AircraftState` passed in | |
| `ctx.rs` wind, QNH, TCAS, COM1 (`routing.rs:285`), `callsign::live` | `SimView` reads | field facts; callsign never stored |
| `Ctx` (`tower.rs:164`) | unchanged | built by `tower_tests.rs` |

## Increments (one commit each, `XCOPILOT_BLACKBOX=off cargo test --workspace` green)
1. **`FlightStrip` over the published statics, own slot only**: `OWN_STRIP` with per-cell
   mutexes; mechanical rename (`CLEARANCE` to `OWN_STRIP.clearance`, ~650 sites); the lifetime
   doors call `OWN_STRIP.reset_<group>()`; `every_atc_static_has_a_lifetime` classifies
   `OWN_STRIP`/`STRIPS`; new test: each door resets exactly the groups it reset before.
2. **Desk out of `Station`**: `twr` and per-flight fields into `self.desk`; the test-built
   `Station` literal (`station/tests.rs` ~3740) changes once. Gate: station, tower and corpus
   tests (`transcript_corpus_parses_safely`).
3. **`AircraftState` in**: own state read once at the top of `build_ctx` (`ctx.rs:8`);
   `handle_transmission_with(state, ...)`/`try_handle_with`, crew passes its snapshot; old
   signatures read the sim and delegate; `note_aircraft_state` with `note_aircraft` a wrapper
   (`src/monitor/run.rs:316`). Gate: ctx built from a state with no sim; `atc_world_sweep`.
4. **`RunwayBook`, computed**: the four queries reimplemented over it; a matrix test (every
   `tower::State`, holds_runway, on/off strip, ghost end, twin parallel) with the old functions
   kept as test oracles. LANDING_HOLD/TAKEOFF_HOLD stay strip holds whose release asks the book.
5. **`STRIPS`, `FlightId`, routing**: `Station.desks`, `Station::respond_for(id, ...)` swapping
   the desk only around the synchronous `respond` (`respond.rs:8`) with a drop guard; traffic
   strips are book claimants, not gated by our `new_flight_pending` (`respond.rs:107`). Tests:
   a Traffic strip cleared to land blocks our line-up; its readback moves no `party_line`
   accessor; an empty map is identical.
6. **Ticks per strip**: go_around, landing_hold, takeoff_hold_from, handoff, squawk, vector and
   compliance ticks get `_for(&FlightStrip, &AircraftState)` cores; `world.rs` keeps its
   signatures wrapping `OWN_STRIP`. Gate: world_tick tests, `atc_world_sweep`.
7. **COMPLIANCE split + docs**: flight part to strip `watch`; `Tower` doc ("one aircraft",
   `tower.rs:1926`) and CLAUDE.md ATC section updated.

## Risks
- **Ordering**: `retire_for_new_flight` raises NEW_FLIGHT_PENDING first; install/uninstall drop
  it last (`state.rs:24-33`); strip resets stay between. FieldFacts are overwritten, not cleared.
- **Lock order**: lock `STRIPS`, clone the `Arc`, drop, then the cell. Never the map across a cell.
- **Async**: `build_ctx` awaits; swap desks only inside synchronous sections.
- **Lifetimes do not align with "a strip"**: four groups per strip; a whole-strip reset on
  retire would clear COMPLIANCE flight bits and GO_AROUND_LAST that survive retire today.
  `FlightCarried` survives install.
- **Traffic strips**: `STRIPS` cleared by install/uninstall, not retire.
- **`Station` is rebuilt at every install** (`new_flight.rs:33`): desks die with it; right for
  ours, wrong for traffic in stage D. Noted, not fixed now.
- **Callsign change**: `Own` is never keyed by callsign; snapshots keep the callsign spoken at
  issue (`ClearanceSnap.callsign` `compliance.rs:12`, `TakeoffHold`, `VectorPlan`, `IfrFlight`,
  `SquawkWatch`). Traffic ids are stable `u32`s with the callsign an attribute.
- **SIDE_STATE_LOCK** (`station/tests.rs:3777`, 106 uses): tests creating Traffic strips hold it
  and clear `STRIPS` through a drop guard.
- **Seam**: accessor names unchanged; the seam pin and the ~208 crew reads need no edits.

## Not yet (stage D)
No traffic producer, kinematics, `TrafficTarget` feed or `ClearanceQueue`; the ghost defences
stay (as book queries); no per-strip chatter or voices; no `World` outliving `Station`; no
callsign-keyed map, single strip mutex, `Ctx` split, world_tick cadence or TCAS change.

## Deviations (as built)
- **Increment 1**: `go_around_last` is in a `watch` group (reset by `install` and `uninstall`
  only), not `worksheet`: `worksheet` is reset by `retire_for_new_flight`, which would change
  its lifetime. `watch` is where `compliance_flight` lands in increment 7.
- **Increment 1**: the macro names each group's reset and name list explicitly
  (`facts (reset_facts, FACTS) { ... }`): `macro_rules!` cannot build `reset_<group>` without a
  paste crate. `FlightStrip::new()` is a `const fn`, not a `NEW` const (a const holding a
  `Mutex` trips clippy's `declare_interior_mutable_const`).
- **Increment 1**: the doors reset through data, `station::state::Group` and
  `INSTALL_RESETS`/`UNINSTALL_RESETS`/`RETIRE_RESETS`, so the door test
  (`each_door_resets_exactly_what_it_reset_before_the_strip`) pins the lists the doors run;
  `the_doors_reset_our_strip_by_group` checks one cell per strip group behaviourally.
  `PartyLineFacts` is gone (all its cells are strip `facts`); `StationFlight` keeps only
  `TOWER_WIND`, `FlightCarried` only `REGION_HANDOFF` and `ARRIVAL_PIN`. `OWN_STRIP` is
  classified as `FlightStrip` (`state::STRIP`); `STRIPS` is left for increment 5.
- **Increment 3**: `build_ctx(xp, state: Option<&AircraftState>, intent)` reads our state at its
  top only when none is handed in, so the old `try_handle` / `handle_transmission` keep their
  word gates and COM1 read BEFORE any read of our aircraft (the common cockpit command still
  costs nothing). `AircraftState` (party_line, crew-visible) marks a value that did not read as
  NaN, `position()` `None` then, which keeps every "both lat and lon read" branch as it was; the
  own read stays the separate LATITUDE/LONGITUDE reads (not `sim::position::read`), and adds the
  elevation and groundspeed reads nothing consumes yet.
- **Increment 3**: the crew is unchanged. The command loop's two `handle_transmission` sites and
  the one `try_handle` hold no aircraft snapshot (only the monitor does), and the monitor's
  `note_aircraft(lat, lon, on_ground)` stays: its snapshot has indicated altitude, no
  helicopter flag and a 0-on-failure heading, so mapping it is a decision, not a pass-through.
  `note_aircraft` is the wrapper over `note_aircraft_state`. atc gains `tokio` as a dev-dependency
  for the no-sim ctx test (`a_ctx_is_built_from_a_handed_in_state_with_no_sim`).
- **Increment 4**: `FlightId` arrives with the book (the claimant needs it), not with `STRIPS`.
  `ClaimKind` names what the four queries read apart: the design's `Occupying` is split into
  `Occupying` (on the pavement, on the ground, authorised or not: `player_blocked_strip`),
  `RunwayStart` and `Vacating` (the two occupations the book authorises), and `Held` covers a
  `holds_runway` in a state that claims no movement (Parked; never seen live, but the matrix
  pins the old answer). A `Claim` carries `held` (the clearance's `holds_runway`) beside its
  kind. The book stores the strips' states and the strip under each, and a snapshot of the
  field's runways, so every query is pure once computed. Ghosts' `Observed` claims come from
  the standing memory (`STANDING`, ground only); no query reads them yet. The runway start and
  the landed strip are read from `COMPLIANCE` for `Own` only until increment 7 moves them onto
  the strip. `chatter_tick` computes one book before the ghost memory's lock and asks it
  (`conflicts_with_claim` is gone; `own_strip_claim` stays as a wrapper). The oracles and the
  matrix live in `station/book_tests.rs` (`SIDE_STATE_LOCK` is `pub(super)` for it): 78,080
  cells, each checking the four queries (and one whole-state book) against the old functions.
- **Increment 5**: `respond_for` swaps the desk around `route_keyed` (the pure keyed path,
  readback gate included, which ends in `respond`), not `respond` alone: a traffic flight's
  readback must meet its own desk's gate. It returns `None` for a traffic number with no open
  strip. The desk carries its strip (`StripDesk.strip: StripRef`, `Own` or a traffic `Arc`), so
  the reply path writes `self.desk.strip` where it wrote `OWN_STRIP` (respond.rs, the keyed
  readback paths, `publish_clearance`, `publish_ifr_clearance`, `radar_contact_at_check_in`).
  `STRIPS` is classified as its own lifetime group, `TrafficStrips`, reset by `install` and
  `uninstall` (the door test now lists it). "Traffic strips are book claimants": `RunwayBook::now`
  reads them (by clearance; no traffic state is held), and a traffic strip holding a runway
  WHOLE (lined up, cleared for takeoff, cleared to land) occupies it for us: `make_ctx`'s
  `runway_occupied` (every intent, ground or air), the takeoff hold's release and the landing
  hold's release all ask the book (`strip_held_by_traffic`, a no-op with no strip open). The
  party line still withholds only against OUR claim. `party_line::lock` takes `&Mutex<T>`.
  Fourth test: the strips die with the station, never with our flight.
- **Increment 6**: the cores are `go_around_tick_for`, `landing_hold_tick_for`,
  `takeoff_hold_tick_for`, `handoff_tick_for`, `vector_tick_for`, `compliance_tick_for` and
  `note_aircraft_state_for`, each `(flight: &FlightStrip, state: &AircraftState, ...)` with the
  inputs the state does not carry kept as arguments (the go-around's AGL, COM1 tuned, the
  vectors' ground track, the field elevation); `squawk_tick_for(flight, ...)` takes no state
  (the transponder is not in it). The strip parameter is named `flight` where the station's
  `strip(runways, ident)` lookup is in scope. A core reads its position as
  `state.position().unwrap_or((NaN, NaN))` where the old body did geometry on raw lat/lon, so
  a failed read is no position exactly as before. `GO_AROUND_ORDERS` moves only for our strip
  (`FlightStrip::is_own`); `OUT_OF_REACH` and its stand-down stay in `note_aircraft_state`,
  ours alone. The holds' book question is now "held by any OTHER strip"
  (`strip_held_by_others`): ours is a claimant for a traffic strip's holds. The compliance core
  still keeps its flight memory in `COMPLIANCE` until increment 7, so its per-strip test lands
  there. Tests: one per tick (station/tick_tests.rs), each checking the traffic strip's cells
  moved and every cell of ours stayed at rest.
