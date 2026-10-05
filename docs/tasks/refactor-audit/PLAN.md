# Refactor plan (audit 2026-10-04, commit 365f504)

Sources: `audit-duplicates.md` (20 duplicated rules), `audit-statics.md` (283 production
statics by lifetime, leaks, ownership model), `audit-modules.md` (seams for the giant files).
Every phase keeps `XCOPILOT_BLACKBOX=off cargo test --workspace` green; phases 0 to 3 are
behaviour fixes or pure moves, phase 5 changes loop structure and needs sorties.

Note: BSD grep drops `pub static` lines; use `rg` or perl for source scans.

## Phase 0: real bugs (each with a reproducing test)
1. `tools/setup_enroute.rs:691` magnetic variation sign: `track_true - var` must be
   `autoland::true_to_mag` (dials 2x variation off). Same at `procedure.rs:3944` fallback.
2. Speed ceiling (Vne - 15) above the overspeed caution (0.9 Vne) for Vne > 150 kt.
3. `hover::ROTOR` never cleared on swap: an aeroplane after the S-76 hover-taxis, is "on the
   ground" to the tower under 100 ft, skips the vacate. Reset in `publish_airframe`.
4. Loops `stand_down` misses: `controls::BRAKES_HELD`, the altitude capture watcher
   (`autopilot.rs:591-700`), `jet_start.rs:90`, `power.rs:76` (no cancel check).
5. `instruments::FLIGHT_TRANSITION` survives approach setup / foreign new flight / mission end.
6. `circuit.rs:5176-5215` race: no own-ABORT recheck after an await, then clears
   `autoland::ABORT` and launches an uncancellable autoland.
7. Medium leaks: `autoland::TRACKING_ERROR`, `NEXT_GATE`/`NEXT_GLIDEPATH_DEG` (consumed after
   the early cancel return), `LANDED_RUNWAY`, `monitor::DESTINATION`, `station::TOWER_WIND`,
   `party_line::ZONE_CHALLENGE`, `station::OWN_FIXES`, `procedure::MISSED_LEGS`.
8. Stage 0 of statics: a lifetime manifest + a stand-down pin test that catches every loop,
   not just `static ACTIVE:`.

## Phase 1: one home per rule (duplicates, ranked)
1. `flight_state::crew_has_aircraft()` replacing 9+ "automation flying" copies.
2. Approach speed / Vr fallbacks: one `aircraft::*` rule (instructor and recover still 1.3 Vs).
3. Pilot "climb to X": airframe climb limiter and speed floor, under `climb_watch`.
4. `ApEngage` for every autopilot engage (FD first, confirmed servos).
5. Every heading write through one turn-safe bug (merge `bug_toward` and `HeadingHold`).
6. One stop-and-park (`hand_back_hold`) for rollout end, taxi-in arrest, pushback, vacate.
7. One "stopped on the ground" predicate and `datarefs::ON_GROUND_ANY` everywhere.
8. `aircraft::landing_needed_m` beside `takeoff_needed_m`.
9. Then 11 to 20: throttle law/dataref, flap placard lookup, slow ratios, angle helpers
   (signed vs absolute `heading_diff`!), geometry, on-runway envelope, arrival brief runway,
   terrain clearance, 250 kt cap.

## Phase 2: hygiene and mechanical moves (no behaviour change)
1. Delete dead code; drop `#![allow(dead_code)]` from `autoland.rs`.
2. Source-scan tests scan the module directory, not one file; fixtures via `CARGO_MANIFEST_DIR`.
3. Move inline test modules to sibling files (~48k lines out of the ten giants).
4. Split `nav.rs`.
5. Shared infra out of the wrong homes: `crew_radio.rs` (from circuit), `runway.rs` (from
   autoland), roll steering, `runway_fit`, line-up, `hand_back_hold`, `question_shortcut`;
   units/geo helpers, one `throttle_dref`, one `alert`, `sim::datarefs` names.
6. Pure clusters out of each giant; `world_tick.rs` for the station ticks `monitor::run` drives.
7. Never move a bin module into `lib.rs` (tracing target flips).

## Phase 3: ownership of state
1. `airframe_begins` / `flight_begins` doors replacing the drifting new-flight call sites.
2. `Scoped<T>` cells stamped with airframe/flight/mission generation.
3. Immutable airframe-facts snapshot behind current accessors.
4. Every loop on `sim::run::Slot`, registered, cancelled by `stand_down`; one writer for
   `speedhold::ACTIVE`, `autotaxi::ACTIVE`; no caller clears another loop's `ABORT`.
5. Hand-downs (`NEXT_GATE`, ...) as launch arguments.
6. `StationState` / `PartyLine` structs replaced wholesale by install/retire; split station.rs.
7. Mission context struct.

## Phase 4: loops into their own files (whole, unchanged)

## Phase 5: loop state structs (behaviour risk, gated on sorties)
`spawn_autoland` (2,957 lines), `run_session` (2,784), `monitor::run` (2,201),
`procedure::fly`, `circuit::fly`, `full_auto::fly`, `autotakeoff::fly`: locals into a state
struct with per-phase methods. Gate: bench replays, `xvl sortie --check`, real sorties.

## Progress 2026-10-05 (evening, merged to main d36cd4a, offline only)
- P0.8 crew side: `src/lifetimes.rs` (`every_crew_static_has_a_lifetime`, stale entries fail too).
- P3.1 stage 2: `forget_flight` also forgets the pilot takeover, crew radio queue and readbacks,
  `main::LAST_READBACK`, the GPU cart, the pilot-moved-lights flag, rapport's offer
  (masks and anti-ice claims deliberately kept: they need a flight).
- P3.4: the ILS setup watch (`ils::SETUP`) and the checklist runner (`flow::RUN`) are Slots in LOOPS.
- P3.5: `NEXT_GATE` / `NEXT_GLIDEPATH_DEG` are `autoland::HandDown` launch arguments;
  `NEXT_ARRIVAL_HANDOFF_KTS` is `procedure::Plan.arrival_handoff_kts`. Left: `VACATE_FOLLOWS`
  (set after another loop launched the landing), `CLIMB_HANDOVER` (an output, not an argument).
- P3.7 slice: `full_auto::forget_mission` owns `DEPARTURE_MINIMUMS` and the runway waiver
  (stored in aircraft.rs privately, since the lib cannot see full_auto).
- Autoflight: loop mode presses through `autoflight::bring` (pinned); four owed a flight remain.
- Duplicates closed: #11 (throttle lever everywhere, answers and undo on the airframe's lever),
  #15 partly (approach_guide, rapport), #16 (`nav::strip_frame`), #18 (arrival brief = crew's
  `wind_runway`), magnetic/true via `runway::true_to_mag` / `mag_to_true`, ~200 dataref literals
  onto `datarefs::*`, `voice::say_opt`, safe hold / recovery onto `sim::position`, `Modes`,
  `safe_hold::floor`, one MORA loader, `flap_within_limits`, `recover::unflyable_floor_kts`.
- Next: P3.2 `Scoped<T>`, P3.3 airframe facts snapshot, P2.6 `world_tick.rs`, phase 4 moves;
  sortie-gated items unchanged (see the 2026-10-04 audit lists).
- 2026-10-06: P2.3 grammar and instructor tests to `src/grammar/tests.rs`, `src/instructor/tests.rs`;
  P2.6 `src/world_tick.rs` (`WorldTick`: runway watch ahead of `quiet`, incursion, radio
  services, traffic, unheard vectors), five calls at the old points since crew work interleaves.
