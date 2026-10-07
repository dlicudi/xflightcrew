# Pillars audit (2026-10-07, HEAD dd905db, read-only)

The project converges on three pillars: (1) copilot + checklists, (2) ATC, (3) AI traffic.
This audit asks whether the foundations carry them. Three read-only analyses: crew package
mapped to pillars, ATC crate and the traffic seam, structural debt. Line-count and edge-scan
scripts were kept in the session scratchpad (not committed).

## Verdict

The shape is healthier than feared; the growth and the core of the two big loops are not.

**Sound**
- One build warning (`tiller.rs:70`), no unreferenced module, 0 TODO/FIXME.
- `crates/atc` depends only on `sim`, anyhow, tracing, serde_json; nothing crew-side.
- Refactor phases 0 to 3 are done as claimed (stand-down covers 22 runs, one airframe door,
  one new-flight door, `crew_has_aircraft`, nav and station split into files).
- 18 of 25 sampled CLAUDE.md rules are pinned by tests; 536 of 544 named identifiers resolve.

**Messy**
- **Growth**: 240k lines at the 2026-10-04 audit, 286k now (+46k in 3 days, 9.7k of it EFB).
- **Giant loops are growing, not shrinking**: `spawn_autoland` 2,957 to 3,667 lines,
  `procedure::fly` 1,683 to 1,877, `autotakeoff::fly` 1,133 to 1,297, `full_auto::fly`
  1,385 to 1,499. Phase 4 and 5 of the refactor never started. 36 functions over 300 lines.
- **Autonomy dominates pillar 1**: 44.0k code lines (38%) against copilot's 34.7k; with tests,
  113k of 286k.
- **Copilot reaches into autonomy**: 122 COPILOT to AUTONOMY references, plus 161 from shared
  code. Most are raw `ACTIVE`/`abort()` flags skipping `flight_state`, or utilities living in
  the wrong module; four are real knots (below).
- **ATC is single-aircraft at its core**: one `Tower` per `Station` (`station/mod.rs:84`,
  `tower.rs:1926` "one aircraft (the player), one runway"); 26 `party_line` singletons for
  "our flight"; other aircraft are TCAS ghosts with no type, heading or speed.
- **The crew/world seam is leaky in use**: `station` is `pub`, the crew calls 68 station items
  from 24 files; `world_tick.rs` (the world's scheduler) lives in the crew.

## Pillar by pillar

| Pillar | State | Blocking debt |
|---|---|---|
| 1. Copilot + checklists | Built, tested | `run_session` 2,811 lines, `grammar::parse_one` 1,392, `tools::build` 583: every new phrase touches all three. 11 `flow` statics. Copilot depends on autonomy. |
| 2. ATC | Built, clean crate | Single-flight model; station bypasses `party_line`; station reads own-ship datarefs itself (`ctx.rs:28-137`). |
| 3. AI traffic | Not started | Flight laws are bin-only giant loops on process-wide flags (one aircraft per process), so unusable for traffic. Plugin has SDK 4.4 `XPLMInstance.h` but no traffic code. |

### The four real copilot to autonomy knots
1. `tools/autopilot` arms `speedhold` and calls `procedure::take_altitude` /
   `terrain_floor_ahead_ft` (lines 206, 376-459, 726, 1140, 1167).
2. `safe_hold` depends on autoland, procedure and plan_audit (terrain floor, nearest runway).
3. `monitor` carries full_auto's runway-fit judgement (`monitor.rs:3989-4068`).
4. `party_line` gets the filed route from `full_auto::filed_route` (`main.rs:244`).

### What a plain copilot genuinely needs from autonomy
takeoff_config / takeoff_trim / `takeoff_thrust::rating_now`, ils setup, ap_engage and
autoflight (already shared), go_around, safe_hold, speedhold (non-A/T airframes), pushback
tug, reverse watch.

## Autonomy: three options

| | FREEZE (keep, no new laws) | EXTRACT (`crates/autonomy`) | CUT to the small set above |
|---|---|---|---|
| Size | 2 to 4 days | Weeks (113k lines move) | 1 to 2 weeks + re-proving safety |
| Breaks | Nothing | Every `crate::` path; needs a `crew_core` crate first | EFB layouts, instructor "show me", sortie runner and scorecard, plan check, rapport brief |
| Keeps open | EXTRACT and CUT | n/a | n/a |

**Recommended: FREEZE.** Its steps are the first stage of EXTRACT, so nothing is foreclosed.

## AI traffic: proposed shape

```
            crew (xvl: copilot, checklists, frozen loops)
                 | party_line (crew <-> world, the ONLY door)
                 v
  sim::SimView --> crates/atc (station, tower FSM per flight strip, RunwayBook)
   (read-only)       | ClearanceQueue (data)        ^ TrafficTarget feed
                     v                              |
                crates/traffic (AI fleet: plans, kinematics, obeys clearances)
                     | TrafficSink (writes xcopilot/traffic/* only)
                     v
          plugin traffic.cpp (XPLMInstance draw, TCAS slots if free)
  crates/nav (apt.dat, CIFP, taxi graph) shared by atc and traffic
  LiveTraffic --> sim/cockpit2/tcas/targets/* --> atc (read, narrate only)
```

- ATC stays read-only: it issues clearances as data; traffic is the only writer of its own
  aircraft, through a narrow sink, never a full client.
- **Replaces LiveTraffic** (decided 2026-10-07, reversing the 2026-07-10 player-only stance):
  our traffic is the default and owns the TCAS slots (`XPLMAcquirePlanes`), so every aircraft
  obeys the tower. Drawn with XPMP2 (MIT, as LiveTraffic does) over a pilot-installed CSL
  package. Schedules are generated or file-loaded, never fetched (SimBrief stays the one
  outbound call). A read-only "real traffic" mode over the TCAS arrays may stay as an option.
- The ghost defences (`own_strip_claim`, `resequence_runway`, `ghost_stale`,
  `player_blocked_strip`) retire once our traffic is the default.
- Traffic kinematics are simple and pure (taxi along the graph, depart, fly the procedure,
  land), NOT our crew flight laws.

## Ordered plan

Each stage keeps `XCOPILOT_BLACKBOX=off cargo test --workspace` green.

**Stage A: stop the spread (no sorties, about a week)**
1. FREEZE: every ownership check through `flight_state` (`main.rs:2460-2612` release block
   becomes one call); misplaced utilities to `runway`, `crew_radio`, `config`; drop the
   `pub use` aliases; a pin test rejecting new COPILOT to AUTONOMY paths.
2. Untie the four knots (above).
3. Station private behind `party_line`; the crew's world writes (`note_aircraft`,
   `set_surface_wind`, `pin_arrival_station`, `arm_station_for_freq`, ...) as named inputs on
   that page.
4. Refactor phase 4 (loops into their own files, pure moves).
5. Pin the tracing filter; `bug_toward` pin for the 8 direct `set_dial(HeadingBug)` calls;
   CLAUDE.md `probe_*` names corrected.

**Stage B: pillar 1 core (low risk)**
6. Split `run_session` into route, dispatch, speak; `parse_one` into per-domain matcher tables.
   Gated by grammar tests, `xvl analyze --now`, `sortie --check`.

**Stage C: ATC for many aircraft (offline-testable, one regression sortie)**
7. `nav` out to `crates/nav`.
8. Per-flight strips: `HashMap<Callsign, FlightStrip>` holding the `Tower` FSM, holds, squawk,
   handoff; `RunwayBook` as a list of claims with ours as one; positions passed in.
   `party_line` stays the crew's projection of OUR strip.

**Stage D: AI traffic**
9. `crates/traffic` (schedules, ground routing on `crates/nav`, departures, arrivals, wake
   spacing, kinematic profiles) + `plugin/src/traffic.cpp` on XPMP2 + CSL, after stage C.
10. Retire the ghost defences; keep TCAS reading as an optional real-traffic mode.

## Why autonomy grew this way, and the redesign

"Every miss becomes a rule" is right for the grammar (language is lookup: a new matcher changes
nothing else) and wrong for flight (a control problem: a new special case in a loop interacts
with the others). The approach section of CLAUDE.md reads as a list of exceptions, and
`spawn_autoland` grew 700 lines in 3 days. Missing: layers (plan, track, actuate are one
function), a measured model of the aircraft (gains tuned per crash), and use of the aircraft's
own autopilot (we fly over HDG/NAV/APR/ALT/VS and then fight it).

Redesign, for the 172, after the pillars, REPLACING the frozen code rather than patching it:
1. The crew operates the autopilot as a pilot does (enroute, climb, descent, coupled ILS): a
   copilot task, inside pillar 1.
2. Our own laws only where no mode exists: takeoff roll, flare and touchdown, rollout, taxi.
3. Those laws layered: an explicit phase state machine as data, one path tracker, per-airframe
   numbers measured once by a probe (as `xvl conform` does), never tuned per flight.
4. A simple simulated plant in `cargo test` so a regression fails before a flight finds it.

**Deferred**: refactor phase 5 (giant loops into state structs) superseded by the redesign above.

## Decisions (owner, 2026-10-07)
1. Autonomy: **FREEZE now, redesign later** (below).
2. **Bug fixes only** on the flying loops through stages A to C: crashes and safety, no new
   behaviour, no new airframe work.
3. **The Cessna 172 is the reference airframe**: the three pillars are built and proven on it;
   the other profiles stay as they are (checkable with `xvl conform`), no new work.
3. ~~Coexist or replace LiveTraffic?~~ Decided 2026-10-07: replace.

## Progress
- 2026-10-07: **Stage A done** (offline, pure refactors, `cargo test --workspace` all green:
  2,159 crew, 86 lib, 268 atc, 54 sim). Copilot to autonomy edges 122 to 63, pinned
  (`copilot_reaches_the_loops_only_through_flight_state`); `station` crate-private behind
  `party_line` and `atc::world` (`the_station_is_reached_only_through_the_seam`); the four
  knots untied (`terrain.rs`, `pilot_altitude.rs`, runway fit in `runway.rs`, filed route in
  `simbrief.rs`); heading-bug writes and the tracing filter pinned; the eight giant files split
  into child modules (only `autoland/run.rs`, `spawn_autoland` whole, stays over 3,000 lines).
  Next: stage B.
- 2026-10-08: **Stage B done** (offline, pure refactors, all green: 2,160 crew, 87 lib, 268
  atc, 54 sim). `run_session` 2,665 lines to 26: `src/main/` stages take, route, dispatch,
  answer, speak over a `Session` struct; no function over 205 lines; `sortie --check`
  identical on all 37 specs. `grammar::parse_one` 1,392 lines to a 17-line driver over an
  ordered `MATCHERS` table (21 entries, 13 files under `src/grammar/`); a 12,527-utterance
  snapshot from the real logs byte-identical (ignored test `grammar_snapshot`), `xvl analyze
  --now` unchanged. Found on the way: a multi-byte panic in `phrase::fix_value_mishear`, fixed.
  Next: stage C.
