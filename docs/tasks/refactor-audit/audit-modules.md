# Module split audit (read-only), 2026-10-04

Scope: the ten largest Rust modules. Numbers come from scripts over the tree at
`365f504` (outline, test-region mask, function spans, reference counts), then
spot reads. Line ranges are approximate (+/- a few lines).

## 0. Headline findings

1. **Half of every giant is tests.** Moving each inline `#[cfg(test)] mod` into its own file roughly halves the sizes, needs no visibility changes and is nearly risk free (see the hazards in section 3).

   | file | lines | test lines | test share | `#[test]` |
   |---|---|---|---|---|
   | src/autoland.rs | 20,310 | 11,443 | 56% | 198 |
   | crates/atc/src/station.rs | 18,528 | 9,667 | 52% | 139 |
   | src/circuit.rs | 10,149 | 4,139 | 41% | 99 |
   | src/autotakeoff.rs | 10,098 | 5,525 | 55% | 85 |
   | src/full_auto.rs | 9,955 | 2,848 | 29% | 67 |
   | crates/atc/src/nav.rs | 8,423 | 3,426 | 41% | 76 |
   | src/procedure.rs | 8,169 | 3,483 | 43% | 89 |
   | src/monitor.rs | 7,988 | 2,371 | 30% | 75 |
   | src/autotaxi.rs | 7,617 | 3,604 | 47% | 85 |
   | src/main.rs | 6,755 | 1,853 | 27% | 54 |

2. **The real weight is a handful of huge async functions, not the file count.** Splitting files will not touch these. Each one is a single loop that keeps dozens of `let mut` locals as its state:

   | function | lines | span | `let mut` locals |
   |---|---|---|---|
   | `autoland::spawn_autoland` (thread + loop) | 2,957 | 1795-4751 | ~70 |
   | `main::run_session` | 2,784 | 868-3651 | many |
   | `monitor::run` | 2,201 | 2967-5167 | 58 |
   | `procedure::fly` | 1,683 | 3011-4693 | 57 |
   | `circuit::fly` | 1,541 | 4374-5914 | 41 |
   | `full_auto::fly` | 1,385 | 1920-3304 | (stages 1-4) |
   | `autotakeoff::fly` | 1,133 | 3408-4540 | 28 |
   | `autotaxi::drive` | 762 | 3257-4018 | 24 |
   | `full_auto::fly_route` | 711 | 3334-4044 | |
   | `main::main` | 612 | 156-767 | |

3. **Most of the pure laws are already free functions and small structs.** They sit at the top of each file with the loop at the bottom, so extracting them is mechanical. The pure part of each file (non-test, non-loop) runs from about 2.5k lines in autoland to about 1k in autotaxi.

4. **Cross-file misplacement.** Each of the following is used by 7 to 12 other modules:
   - the crew radio queue lives in `circuit.rs`,
   - runway data and geometry live in `autoland.rs`,
   - the roll and wings-level laws live in `autotakeoff.rs`,
   - `runway_fit` lives in `autotakeoff.rs`,
   - heading-bug steering lives in `circuit.rs`.

5. **Dead code is hidden behind `#![allow(dead_code)]` at the top of `src/autoland.rs` (line 4).** That attribute covers all 20k lines.

## 1. Cross-cutting issues

### 1.1 Duplicated helpers (pick one home, re-export)
- **Angle and geo:**
  - `heading_diff`: autoland:78, reversal:39, course_leg:58
  - `angle_diff_deg`: nav:3409, station:8631, pushback:860, approach_guide:136
  - `angle_diff` / `bearing`: taxi_guide:718/724, ground_steer:652/656
  - `wrap180`: instructor:506, ils:418, test_card:183
  - `bearing_deg`: nav:3414, approach_guide:141
  - `metres_between`: station:6190
  - `dist_m`: rapport:180
  - Proposal: a `sim::geo` (or `atc::nav::geo`) module, since `nav` already exports `bearing_deg`, `haversine_nm` and `project_latlon`.
- **Unit constants** `M_TO_FT`, `MS_TO_KTS`, `M_PER_NM` are redefined in 15+ files (autoland, autotakeoff, autotaxi, circuit, full_auto, monitor, procedure, pushback, reversal, hover, terrain, flightplan, sortie_runner, tools/*). Put them in a `units` module.
- **`fn throttle_dref()`** (a one-line wrapper over `aircraft::throttle_dataref()`) is copied into autoland, autotakeoff, autotaxi, circuit, tools/autopilot, tools/cycle, tools/recover and tools/speedhold.
- **`fn alert(voice, priority, line)`** (speak plus mirror) is copied into circuit, autotakeoff, autotaxi and procedure. `announce` (full_auto) and `announce_autoland` (autoland) are the same thing. The only difference between them is the `tracing` target string.
- **Three heading-bug holders:**
  - `circuit::BugHold` / `bug_toward` / `slew_bug` (used by autoland, reversal, procedure),
  - `procedure::HeadingHold`,
  - `autoland::bug_knob` plus `AUTOLAND_*_DEADBAND`.

  CLAUDE.md already describes one rule ("the heading knob is set, not rewritten"). They should live in one `heading_bug.rs`.
- **Dataref literals**: the rule is "read names come from `datarefs::*`", but the big files still define local consts:

  | file | local `"sim/..."` / `"xcopilot/..."` consts | of which already in `sim::datarefs` |
  |---|---|---|
  | autoland | 35 | 10 |
  | autotakeoff | 29 | 9 |
  | procedure | 22 | 7 |
  | circuit | 19 | 6 |
  | autotaxi | 9 | 3 |
  | full_auto | 8 | 4 |

  `real_install_names` still catches typos, but the duplicates should point at `datarefs::`.

### 1.2 Hazards for any move (these make "behaviour-preserving" non-trivial)
- **Single-file source scans.** These break, or worse go vacuous, when code leaves the file:
  - `include_str!("full_auto.rs")`: 7 tests in full_auto, plus one in tools/setup.rs.
  - `include_str!("main.rs")`: in main, pushback and pollers.
  - `include_str!("monitor.rs")`: x2.
  - `include_str!("autoland.rs")` and `include_str!("circuit.rs")`: in autoland's tests around line 13305.
  - `heli_route.rs:3663` reads `src/full_auto.rs` and `src/heli_route.rs` by name.
  - `sortie_runner::READ_SHAPES` pins producers to `"src/main.rs"`, `"src/full_auto.rs"`, `"src/scorecard.rs"`, `"src/voice.rs"` and `"src/pollers.rs"`. This one fails loudly, which is good.

  Fix: before moving code, convert each single-file scan to scan the module's directory, the same way the recursive walkers already do (`flight_state.rs:129`, `aircraft.rs:3694`, `simbrief.rs:946`, `health.rs:308`). Those walkers survive any split. The `flight_state` scan splits text on `#[cfg(test)]`, so tests moved to their own files are fine.
- **Relative `include_str!` fixtures.** Examples: `"../bench/landings/*.jsonl"`, `"../bench/ofp/*.json"`, `"../profiles/*.yaml"`, `"../crates/atc/bench/cifp/..."`. These are relative to the containing file, so a test moved to `src/autoland/tests/x.rs` needs `"../../../bench/..."`. Use `concat!(env!("CARGO_MANIFEST_DIR"), "/bench/...")` while moving, so the paths never need touching again.
- **Tracing targets.** Moving within the bin crate keeps the `xvl::` prefix. `xvl::autoland::flare` still matches the `xvl=info` filter (`src/main.rs`), and explicit `target: "xvoicelayer::autoland"` strings are unaffected. Never move a module from the bin tree into `lib.rs` as part of a split: the default target would flip to `xvoicelayer::` (see CLAUDE.md "Wire contracts").
- **Test privacy.** Tests use `super::*` and reach private consts and fns. When a cluster moves to `parent/sub.rs`:
  - mark its formerly private items `pub(super)`,
  - add `use sub::*;` (or `pub use` for already-pub items) in the parent.

  The parent's `tests` module then still sees everything via `super::*`. Inside the bin crate, `pub` and `pub(crate)` are equivalent. In `crates/atc` (a lib), keep `pub use` for anything the crew package imports.
- **Rust 2018 non-mod-rs layout.** `src/autoland.rs` may declare `#[cfg(test)] mod tests;` and the compiler resolves it to `src/autoland/tests.rs`. The first steps therefore need no `git mv` of the parent file.

### 1.3 Dead code and stale allows
- `src/autoland.rs:4 #![allow(dead_code)]` (module-wide). Under it, these are definitely unused:
  - `HPATH_TRUE` (448), `PHI_TRUE` (450), `FLARE_PITCH_YIELD` (6322), `FLARE_ROLL_YIELD` (6330). These are leftovers from before `sim::state::F` took over the reads (`AUTOLAND_WANT` at 496).
  - Test-only: `commit_flare` (8194, superseded by `commit_flare_latched`) and `FlareModel::easing_on_any_climb` (6724).

  Remove the attribute first and let rustc list the rest.
- `crates/atc/src/nav.rs:3622 ProcLeg::has_fix_target`: no callers anywhere.
- Test-only pub items (keep under `#[cfg(test)]` or delete):
  - `station::compliance_tick` (2990, a wrapper over `_facing`), `station::takeoff_hold_tick` (7102, a wrapper over `_from`), `station::arrival_stream_end` (7454)
  - `circuit::Geometry::for_radius` (238), `circuit::alt_capture_vs` (585)
  - `autotakeoff::RollSteer::with_rudder` (1599)
  - `monitor::stall_speed_at_flap` (1031)
  - `autotaxi::TurnRound::TEARDROP` (2193)
- `pub(crate)` items with no use outside their own file (could be private): `autotakeoff::stop_on_centreline` (3077), `reject_hands` (3215), `spoken_height_ft` (4546). `autotaxi::join_fix` is used only internally.
- Stale `allow(dead_code)` comments that no longer hold:
  - `main.rs:37` on `mod flight;` ("shadow run: the consumers move over next"). CLAUDE.md says flight IS read now.
  - `monitor.rs:118` Phase and `:789` Severity ("M3").
  - `voice.rs:60` Priority and `:1449` `Voice::alert` ("next milestone"). `alert` is called everywhere.
  - `crates/atc/src/nav.rs` 178-368: `allow` on `pub` items in a lib crate is a no-op, and `FREQ_TOWER` and others are used.
  - `full_auto.rs:1591` `departure_check_in` is `cfg_attr(not(test), allow(dead_code))`: check whether a production path still uses it.
- Name-based scan caveat: the counts are word matches over `src/` and `crates/`, so common names (e.g. `slew`) may hide real dead items. rustc is the authority once the module-wide allow is gone.

### 1.4 Layering notes
- **Pure modules are mostly clean.** `stable_approach`, `course_leg`, `duty` and `flight` have no `.await` and no sim IO. `flight` and `climb_watch` call `blackbox::record` directly (2 sites each): acceptable side-effect logging, but it is the one thing that stops them being strictly pure.
- `reversal.rs` (the "pure plan beside the loop") does async terrain probing itself (`ground_under`, `terrain::sample_ft` at 685-825). Split it into `reversal/plan.rs` (pure) and `reversal/probe.rs` (async).
- **`monitor::run` hosts the WORLD's ticks.** Lines ~3608-4786 (about 1.2k lines) drive `station::vector_tick`, `compliance_tick_facing`, `takeoff_hold_tick_from`, `go_around_tick`, `landing_hold_tick`, `chatter_tick`, `squawk_tick`, `region_tick`, `atis_*`, `wake_note_targets` and the traffic advisories, about 25 distinct `station::` entry points. The crew's monitor is the world's scheduler. This is a seam violation in spirit: CLAUDE.md wants the world/crew seam to be `party_line`. Proposal: a `src/world_tick.rs` (crew side, still in the bin) that the monitor calls once per tick with a `WorldInputs` struct.
- **`main.rs` tests mostly exercise router, answers, undo and plan functions** (`classify_confidence`, `is_affirmative`, `plan_change_line`, `wrong_plan_complaint`, `looks_like_command`, `correction_value`, ...) imported into main. Those tests belong beside their functions.
- **`main.rs` owns logic other modules import:**
  - `Shortcut`, `question_shortcut`, `pre_grammar_capture` and `state_capture_note` are used by `sortie_runner` as `crate::...`.
  - `levenshtein` is used by tools/lights.
  - `TOD_NM_PER_1000FT` is used by `plan.rs`.

  These should move into `router` (or a new `shortcuts.rs`) and `phrase`/`plan`.
- **`crates/atc/station.rs` keeps 30+ private statics.** `Station::install` / `uninstall` (345-509) touch nearly every one (e.g. `ACTIVE_RUNWAYS` is referenced from 24 sites across 9 sections). Any split must give each submodule a `reset()` / `publish()` pair that install and uninstall call. That is also a better design than one function knowing every static.

## 2. Per-file proposals

Each file gets: clusters with ranges, coupling, pure vs IO, layout, and steps (smallest risk first).

### 2.1 src/autoland.rs (20,310; code ~8,740 in 1-8740, tests 8742-20310 in 17 modules)

**Clusters (code):**

| cluster | lines | key items | pure? |
|---|---|---|---|
| runway data + self-pick | 1-388 | `Runway`, `parse_runway_100`, `nearby_runways`, `best_landing_runway(_in_wind)`, `landing_score`, `self_pick`, `PlannedRunway`, `runway_by_id`, `heading_diff` | pure except apt.dat file read |
| loop statics + dataref consts | 389-540 | `ABORT`, `ACTIVE`, `GEN`, `PANEL`, `LANDING_CONFIGURED`, 35 consts, `AUTOLAND_WANT(_ILS)` | state |
| VS watchdog + AP engage | 542-765 | `vs_not_held`, `ApEngage`, `EngageStep`, `engage_hold_gains` | pure |
| rollout | 765-1230 | `rollout_brake_ratio`, `RolloutBrakeSteer`, `RolloutSteer`, `BounceHold`, `TouchHold`, `VACATE_FOLLOWS`, `LANDED_RUNWAY`, `disconnect_autopilot` (async) | pure (one async) |
| wheels-on, balk call, uncleared | 1277-1475 | `should_start_rollout`, `wheels_on_time`, `wheels_hold_the_landing`, `BalkWatch`, `go_around_call_due`, `UnclearedWatch` | pure |
| launch / arm | 1475-1795 | `run_autoland(_scoped)`, `AutolandLaunch`, `ArmedRunway`, `ApproachRuntime`, `arm_runway` (async, 190) | IO |
| **the loop** | 1795-4751 | `spawn_autoland` (2,957 lines) | IO |
| clearance helpers | 4775-4840 | `clearance_switch`, `armed_runway_why`, `landing_readback`, `clearance_differs` | pure |
| crab + track trim | 4842-5070 | `CrabFilter`, `TrackTrim` | pure |
| glidepath + stable gate state | 5070-5260 | `GLIDEPATH_DEG`, `NEXT_GATE`, `NEXT_GLIDEPATH_DEG` statics, `set_next_gate`, `set_next_glidepath_deg`, `gp_ft_per_nm`, `glidepath_gain_fpm_per_ft` | state + pure |
| approach geometry, speed, flap | 5261-5660 | `straight_in_losable`, `aims_at_join_point`, `lands_on_the_pavement`, `clean_approach_ias`, `gear_may_extend`, `flap_within_limits`, `flap_held_within_limits`, `flap_kept_flying`, `scheduled_ias`, `pattern_ias`, `approach_target_kts`, `hand_back_speed_kts`, `overflight_allowance_nm` | pure |
| approach power | 5660-5830 | `retard_reference_kts`, `roundout_float_lever`, `short_final_power_held`, `approach_power_floor`, `climb_power_floor` | pure |
| live terrain floor | 5831-5903 | `live_floor_leg` | pure (uses `circuit::live_floor_ft`) |
| arrest targets, dial, aim | 5904-6085 | `arrest_gates_ft`, `arrest_targets_fpm`, `flare_target_*`, `parked_altitude_dial_ft`, `aim_elevation_ft`, `aim_with_glideslope` | pure |
| flare power (retard) | 6085-6320 | `RetardKey`, `FlarePower`, `balloon_power` | pure |
| flare pitch + model | 6321-7060 | `FlarePitch`, `FlareGains`, `FlareModel`, `FlareModelOut`, `FinalAir` | pure (one `LAST_APPR_READS` static) |
| arrest lateral | 7060-7330 | `ArrestHands`, `arrest_hands_step`, `ShortFinal`, `short_final_cut_deg`, `arrest_wings`, `arrest_bank_target`, `side_velocity_m_s`, `vs_dial_fpm` | pure |
| landing limits + geometry | 7330-7530 | `landing_xtk_limit_m`, `flare_xtk_limit_m`, `RUNWAY_WIDTHS` static, `runway_width_m`, `late_go_around`, `flap_schedule`, `true_to_mag`, `along_track_nm`, `cross_track_nm` | pure + one static |
| go-around / missed | 7533-7948 | `MissedStage`, `GoAroundLatch`, `join_point_nm`, `missed_approach_guidance`, `missed_approach_speed_kts`, `missed_approach_throttle`, `go_around_vs_fpm`, `go_around_flap_due` | pure |
| guidance core | 7948-8740 | `bug_knob`, `ApproachPhase`, `tick_ms`, `Guidance`, `pursuit_aim_nm`, `hold_flare`, `flare_pavement_need_m`, `commit_flare(_latched)`, `hold_roundout`, `guidance`, `guidance_on` (383 lines) | pure |

**Coupling:**
- The loop calls every cluster. The pure clusters mostly call only constants and each other: guidance calls glidepath, geometry and commit; flare calls `FlareGains`.
- Imports from other modules:
  - autotakeoff: `WingsLevel`, `XtkRate`, `RollSteer`, `roll_target_heading` (rollout and arrest)
  - circuit: `BugHold`, `BugDeadband`, `live_floor_ft`
  - also `reversal`, `ils`, `stable_approach`, `climb_watch`
- Used by others:
  - `Runway` / `nearby_runways`: 8+ modules
  - `true_to_mag`: procedure, course_leg
  - `cross_track_nm`, `along_track_nm`: autotaxi, full_auto
  - `heading_diff`: full_auto
  - `runway_by_id`: procedure, autotaxi

**Pure vs IO:** about 5,000 of the 8,740 code lines are pure laws. The IO is `arm_runway`, `spawn_autoland` and `disconnect_autopilot`.

**Misplacement:**
- Runway data and geometry (`Runway`, `nearby_runways`, `best_landing_runway*`, `along/cross_track_nm`, `true_to_mag`, `heading_diff`, `RUNWAY_WIDTHS`) is shared infrastructure. Proposal: `src/runway.rs` (or `src/runway/{data,select,geometry}.rs`), with `autoland` re-exporting it for one release so call sites do not churn.
- The self-pick (`landing_score`, `self_pick_min_length_m`, `best_landing_runway_in_wind`) can go with it as `runway/select.rs`, or stay as `autoland/self_pick.rs`.

**Layout:**
```
src/autoland.rs            (mod decls, statics ABORT/ACTIVE/GEN/PANEL, pub use)
src/autoland/
  launch.rs        run_autoland*, arm_runway, AutolandLaunch, ApproachRuntime
  run.rs           spawn_autoland (later: LoopState + per-phase methods)
  engage.rs        ApEngage, vs_not_held
  guidance.rs      Guidance, guidance/guidance_on, pursuit_aim_nm, ApproachPhase, tick_ms, bug_knob
  glidepath.rs     glidepath statics + gp_* + stable gate statics
  approach.rs      speed/flap/geometry laws (5261-5660) + landing limits (7330-7470)
  power.rs         approach power floors + FlarePower/RetardKey/balloon_power
  trim.rs          CrabFilter, TrackTrim
  flare.rs         FlarePitch, FlareGains, FlareModel, FinalAir, arrest targets
  arrest.rs        ArrestHands, ShortFinal, arrest bank/wings, vs_dial_fpm
  commit.rs        commit_flare_latched, hold_roundout, hold_flare, flare_pavement_need_m, wheels-on, BalkWatch, UnclearedWatch
  go_around.rs     MissedStage, GoAroundLatch, missed_* laws
  rollout.rs       Rollout*, BounceHold, TouchHold, LANDED_RUNWAY, VACATE_FOLLOWS
  clearance.rs     clearance_switch & friends
  tests/           one file per current test module (17)
src/runway.rs      Runway, parse, nearby_runways, select, along/cross track, true_to_mag
```

**Steps:**
1. Delete the 4 dead consts and remove `#![allow(dead_code)]`. Fix or delete what rustc reports. This needs `cargo build`.
2. Move the 17 test modules to `src/autoland/tests/*.rs` via `#[cfg(test)] mod lemg13_sr22_flare_tests;` lines. Rewrite relative `include_str!` paths to `CARGO_MANIFEST_DIR`. Convert the `include_str!("autoland.rs")` / `("circuit.rs")` scan at ~13305 to a directory scan.
3. Extract leaf pure clusters in dependency order: `trim.rs`, `engage.rs`, `go_around.rs`, `rollout.rs`, `power.rs`, `flare.rs`, `arrest.rs`, `approach.rs`, `glidepath.rs`, `commit.rs`, `guidance.rs`. Each is one commit with `use x::*;` in the parent and private items made `pub(super)`.
4. Move the launch/arm block to `launch.rs` and the loop to `run.rs` unchanged.
5. Move runway data to `src/runway.rs` and leave `pub use crate::runway::{Runway, nearby_runways, ...}` in autoland so callers can migrate later.
6. (Behavioural-risk tier) Lift `spawn_autoland`'s ~70 locals into a `LoopState` struct, then carve the loop into methods along its own comment seams:
   - read/frozen (2083-2170)
   - rollout (2167-2660)
   - commit/balk/order handling (2657-2850)
   - go-around and reversal (2850-3500)
   - terrain hold and live floor (3500-3670)
   - engage (3670-3780)
   - flare model and arrest hands (3780-4280)
   - power (4280-4510)
   - config (4509-4635)
   - phase change (4664)

   Gate it on the bench/landings replays plus a real sortie night.

### 2.2 crates/atc/src/station.rs (18,528; code ~8,860, tests 8862-18528 as one module)

**Clusters:**

| cluster | lines | key items | pure? |
|---|---|---|---|
| Station core | 41-2515 | `Service`, `Wx`, `Station` (fields 76-178), `Dialect`, `dialect_for`, `spoken_pressure`, `impl Station` (~2,300 lines) | mixed |
| (impl) lifecycle | 218-509 | `load`, `from_airport`, `set_procedures*`, `install`, `uninstall` | side effects |
| (impl) routing | 509-790 | `try_handle`, `route_unkeyed`, `route_keyed`, `handle_transmission`, `tuned_speaker`, `tuned_service` | async read |
| (impl) responses | 791-1900 | `respond`, `respond_inner`, `respond_departure`, `cancel_ifr`, `amend_sid`, `ifr_offer`, `circuit_offer`, `assign_squawk`, `arm_departure_handoff`, `respond_approach`, `arm_vectors`, `apply_monitor_*` | mostly pure state machine |
| (impl) runway pick | 1896-2278 | `note_stated_runway`, `note_caller_on_strip`, `state_runway_change`, `apply_runway_request`, `pick_runway(_with)` (212 lines) | pure |
| (impl) ctx | 2278-2515 | `build_ctx` (async), `make_ctx` | read |
| IFR clearance helpers | 2516-2745 | `is_radio_call`, `SidBasis`, `publish_ifr_clearance`, `resolve_destination`, `resolve_runway_request` | pure + publish |
| squawk / radar watch | 2745-2880 | `discrete_squawk`, `SquawkWatch`, `SQUAWK_WATCH`, `squawk_tick`, `radar_contact_at_check_in` | state |
| compliance watch | 2879-3360 | `ClearanceSnap`, `ComplianceState`, `COMPLIANCE`, `compliance_tick(_facing)` (351 lines) | state |
| departure handoff | 3359-3396 | `Handoff`, `handoff_tick` | state |
| ATIS state | 3397-3555 | `AtisState`, `ATIS`, `AtisWx`, `atis_due`, `atis_retune`, `atis_tick`, `atis_current_letter` | state (text from `atis.rs`) |
| region / field directory / reach | 3556-4515 | `REGION`, `FIELD_DIR`, `ACTIVE_FIELD`, `load_region`, `region_tick` (195), `radar_position`, `region_handoff_line`, `fields_in_range`, arrival pin, `arm_station_for_freq`, new-flight arming (`retire_for_new_flight`, `arm_new_flight*`, `field_under`, `judge_departure_station`), `note_aircraft`, `OUT_OF_REACH` | state |
| self taxi (uncontrolled) | 4516-4710 | `SELF_TAXI`, `plan_self_taxi(_to_stand)` | state + pure |
| active runways / occupancy | 4710-5185 | `ACTIVE_RUNWAYS`, `on_active_runway*`, `runway_occupancy_authorised`, `ends_oppose`, `capped_departure_climb`, `arrival_end`, `final_traffic_for`, `on_independent_final` | pure over the static |
| TCAS + coach | 5185-5610 | `TcasTarget`, `read_tcas_targets` (async), `coach_query`, `Suggestion`, `suggest_call` (176), `active_field`, `void_outcome`, `SURFACE_WIND` | mixed |
| party line chatter | 5649-6022 | `GhostMemory`, `GHOSTS`, `PartyLine`, `own_strip_claim`, `conflicts_with_claim`, `chatter_tick` (232) | state |
| strip occupancy + go-around | 6023-6912 | `OWN_FIXES`, `STRIP_SEEN`, `StripExit`, `on_runway_pavement`, `STANDING`, `ghost_stale`, `occupies`, `strip_exit`, `go_around_due`, `landing_release_due`, `go_around_tick`, `resequence_runway`, `LandingHold`, `landing_hold_tick` | pure judges + state |
| takeoff hold | 6914-7383 | `TakeoffHold`, `stated_runway_stands`, `runway_stands_within`, `void_takeoff_hold_by_roll`, `takeoff_hold_tick_from` (181) | state |
| wake + streams | 7384-7617 | `WAKE`, `DEPARTURE_STREAM`, `ARRIVAL_STREAM`, `departure_end`, `field_streams`, `wake_note_targets`, `wake_remaining` | state |
| taxi routes + phraseology | 7618-8145 | `TAXI_NET`, `ILS_FACTS`, `spawn_*_load`, `plan_taxi_route_to(_stand)`, `stand_route_on`, `mark_runway_crossings`, `spoken_taxiway`, `spoken_via`, `spoken_stand`, polyline helpers | pure + loaders |
| tuning / vectors | 8146-8467 | `tuned_controller`, `heard_information`, `VectorPlan`, `vector_tick` (204) | state |
| traffic advisories | 8468-8640 | `TrafficFactor`, `nearest_traffic_factor`, `advisory_line`, `near_miss_line` | pure + read |
| freq/briefing | 8642-8860 | `Tune`, `services`, `freq_query`, `impl Station { briefing_text, void_note, tune_request }`, `spoken_wind` | pure |

**Coupling:** the statics are the coupling. `ACTIVE_RUNWAYS` (24 sites), `ACTIVE_FIELD` (13), `REGION` (13), `ATIS` (30+) and `SQUAWK_WATCH` (14) are read across sections. `install` and `uninstall` write them all. Moving each static with its section and exposing `pub(super)` accessors, or a `reset()`, is required.

**Pure vs IO:** the IO is the few async `SimView` reads (`read_tcas_targets`, `build_ctx`, `tuned_*`, `inbound_traffic_for_strip`, `nearest_traffic_factor`). Everything else is pure logic over process statics.

**Misplacement:**
- The ATIS state (3397-3555) belongs in `atis.rs`, which today is only text synthesis. Either add an `atis/state` part or put it in `station/atis.rs`.
- The traffic advisories (8468-8640) are a self-contained `traffic.rs`.
- The taxi phraseology (`spoken_taxiway`, `spoken_via`, `spoken_stand`, `first_designator`) belongs in a `phraseology.rs`, with `tower.rs`.
- `angle_diff_deg` and `metres_between` duplicate nav.

**Layout:**
```
crates/atc/src/station.rs       (Station struct + small impl glue, pub use)
crates/atc/src/station/
  lifecycle.rs   load/from_airport/install/uninstall -> calls each submodule's publish/reset
  routing.rs     try_handle, route_keyed/unkeyed, tuned_*
  respond.rs     respond*, departure, approach, IFR, cancel/amend
  runway_pick.rs pick_runway(_with), stated runway, caller on strip
  ctx.rs         build_ctx, make_ctx
  ifr.rs         SidBasis, publish_ifr_clearance, resolve_destination
  radar.rs       squawk watch, handoff, vectors
  compliance.rs  ComplianceState + tick
  region.rs      region/field directory/reach/new-flight arming/arrival pin
  self_taxi.rs
  runways.rs     ACTIVE_RUNWAYS + occupancy/end judges
  tcas.rs        TcasTarget, read_tcas_targets
  coach.rs       coach_*, Suggestion, suggest_call
  party_chatter.rs ghosts + chatter_tick
  strip.rs       strip occupancy, go-around, landing hold, resequence
  takeoff_hold.rs
  wake.rs        wake + streams
  taxi.rs        taxi net, ILS facts loaders, route planning
  briefing.rs    Tune, services, briefing_text, void_note
crates/atc/src/traffic.rs, phraseology.rs, atis.rs (+state)
```

**Steps:**
1. Move the single `mod tests` (9.7k lines) to `station/tests.rs`. It is one module, so consider splitting it later along the sections above.
2. Move the stateless leaves: `traffic.rs`, phraseology, `briefing.rs`, `coach.rs`, `ifr.rs`.
3. Give each static-owning section a `pub(super) fn reset()`. Make `uninstall` call them in the same order as today (diff the body first), then move each section with its statics.
4. Split `impl Station` across files. Multiple `impl Station` blocks in sibling modules are legal. Private fields need `pub(super)`.

### 2.3 src/circuit.rs (10,149; code ~6,000; tests in 11 modules from 6006, plus test-only items near 554/653/2217)

**Clusters:**

| cluster | lines | key items | pure? |
|---|---|---|---|
| turn + geometry + final intercept | 72-560 | `turn_radius_*`, `final_turn_radius_m`, `BankWatch`, `Geometry`, `interceptable`, `ready_for_autoland`, `handoff_residual_m`, `final_intercept_hdg`, `final_led_xtk_m` | pure |
| vertical path | 560-960 | `path_descent_fpm_on`, `alt_capture_vs(_on)`, `approach_leg_alt_on`, `path_dist_m`, `live_floor_ft`, on-the-path (#188) 767-944 | pure |
| speeds, flaps, climb | 957-1413 | `pattern_speed_kts`, `leg_want_kts`, `LegSpeeds`, `detent_above`, `cleanup_schedule`, `go_around_*`, `climb_floor_*`, `climb_limit_fpm`, `ClimbCarry`, `takeoff_flaps_to_clean` | pure |
| loop statics + panel | 1413-1449 | `ACTIVE`, `ABORT`, `DETAIL`, `status`, `circuit_detail` | state |
| **crew radio queue** | 1449-1834 | `CREW_RADIO`, `CrewCall`, `Holds`, `Queued`, `Readback`, `LAST_READBACK`, `holds_for`, `stale_reason`, `take_crew_radio`, `transmit*`, `crew_callsign` (async), `callsign_inputs`, `radio_call`, `spell_runway` | state, not circuit at all |
| params + AP capture | 1834-1930 | `Params`, `AP_ALTITUDE_CAPTURE`, `circuit_dial_ft`, `vs_stuck`, `ap_holds_level` | |
| legs + heading bug | 1930-2310 | `Leg`, `leg_heading`, `slew_bug`, `write_bug` (async), `BugHold`, `BugDeadband`, `hold_bug`, `LAST_BUG`, `bug_toward`, `Closure`, `leg_target_shaped` | pure + async writer |
| pattern guard + give-up | 2310-2607 | `PatternTrip`, `leg_limit_s`, `PatternGuard`, `give_up_pattern` (async) | |
| terrain hold | 2572-2770 | `terrain_hold_*`, `hold_clear_of_terrain` (async, 162) | IO |
| leg sequencing + descent check | 2771-3097 | `next_leg`, `pattern_advance`, `DescentShape`, `DescentLeg`, `GroundSample`, `descent_target_ft`, `check_descent(_from)` | pure |
| pattern choice + corridor | 3097-3583 | `SideFacts`, `PatternPick`, `choose_pattern(_with)`, `PickRules`, `corridor_frame`, `corridor_facts`, `probe_side` (async), `densify` | pure + one probe |
| authored route (LXGB) | 3583-4258 | `AuthoredWpt`, `authored_circuit`, `authored_vertical`, `join_waypoint`, `climb_reach_ft`, `raise_for_terrain`, `check_authored_route`, `fly_authored_route` (317) | mixed |
| start + lines | 4259-4373 | `start`, `alert`, `STOOD_DOWN`, `descent_shape_line`, `field_circuit_line` | |
| **loop** | 4374-5914 | `fly` (1,541) | IO |
| published arrival | 5919-6005 | `ArrivalCtx`, `try_published_arrival` | IO |

**Misplacement:**
- The crew radio queue is used by taxi_in, full_auto, heli_route, monitor, autoland, autotakeoff, tools/setup and main. `crew_callsign` and `spell_runway` alone are used by 10-12 modules. Move it to `src/crew_radio.rs`. This is the single highest-value cross-file move.
- The heading bug (`BugHold`, `bug_toward`, `slew_bug`, `write_bug`, `hold_bug`, `LAST_BUG`) is used by autoland, reversal and procedure. Move it to `src/heading_bug.rs` and merge `procedure::HeadingHold` and `autoland::bug_knob` into it later (a behaviour change, so a separate step).
- `climb_limit_fpm` is used by procedure and tools. Speeds and climb could form a `pattern_laws` module, or stay as `circuit/speeds.rs` re-exported.

**Layout:**
```
src/circuit.rs
src/circuit/{geometry.rs, vertical.rs, speeds.rs, guard.rs, terrain_hold.rs,
             descent.rs, pattern_pick.rs, authored.rs, fly.rs, arrival.rs, tests/}
src/crew_radio.rs   (from 1449-1834)
src/heading_bug.rs  (from 1941-2172)
```

**Steps:**
1. Move the tests out.
2. `crew_radio.rs`: keep `pub use crate::crew_radio::*` in circuit so the `circuit::transmit` and `circuit::crew_callsign` paths keep compiling. Then migrate callers with sed in a later commit.
3. `heading_bug.rs`, the same way.
4. The pure clusters: geometry, vertical, speeds, descent, pattern_pick.
5. The authored route.
6. The `fly` LoopState refactor along its phase comments (takeoff, authored track, pattern legs, go-around order, hand-over, vertical, power).

### 2.4 src/autotakeoff.rs (10,098; code ~4,570; tests 4720-10098)

**Clusters:**

| cluster | lines | key items | pure? |
|---|---|---|---|
| flap pacing | 331-490 | `flap_retract_step`, `paced_flap_step`, `FlapPace`, `flap_step_line` | pure (used by circuit) |
| gear warning | 489-532 | `GEAR_WARNING`, `gear_up_due` | |
| run slot + statics + Params | 533-640 | `RUN`, `ACTIVE`, `REJECT`, `DETAIL`, `Params` | state |
| runway fit | 642-948 | `runway_fit`, `NoFit`, `no_fit_line`, `why_no_fit(_cleared)`, `cleared_fits_here`, `runway_matches` | pure |
| roll centreline | 949-1270 | `roll_target_heading`, `roll_cut_deg`, `XtkRate`, `DriftFilter`, `RollCut`, `CutTick`, `slew` | pure |
| roll brakes + align | 1296-1550 | `roll_brake_split`, `roll_brake_fade`, `free_roll_brakes`, `RollAlign`, `with_hold_brake` | pure |
| RollSteer | 1551-1810 | `RollSteer`, `power_lead` | pure |
| rotation + climb pitch | 1810-2338 | `rotate_ok`, `RotatePitch` (impl 1962-2330), `ClimbHold`, `tail_clear_ceiling_deg` | pure |
| wings level | 2330-2427 | `climb_bank_target`, `WingsLevel` | pure (used by autoland arrest) |
| throttle | 2427-2523 | `throttle_ramp`, `throttle_cmd`, `throttle_slew`, `tick_throttle_want`, `thrust_reduced`, `reduction_slew` | pure |
| climb-out + handoff | 2523-2958 | `climb_vs_fpm`, `clean_up_at_handoff`, `handoff_*`, `climb_guard_vne`, `ClimbOut`, `CLIMB_OUT`, `TAKEOFF_POWER`, `torque_limited`, `ClimbVs`, `ClimbHandover`, `speed_window_after_acceleration` (async) | pure + statics |
| reject | 2958-3265 | `lateral_excursion`, `must_reject`, `RejectStop`, `stop_on_centreline`, `reject_end_line`, `reject_rolling`, `reject_hands`, `reject` | IO |
| AP engage helpers | 3274-3408 | `move_speed_window`, `ap_inherited`, `clear_inherited_autopilot`, `no_pitch_take_over`, `engage_climb` | IO |
| **loop** | 3408-4540 | `fly` (1,133) | IO |

**Misplacement:**
- The ground-track laws (`roll_target_heading`, `XtkRate`, `RollSteer`, `RollCut`, `DriftFilter`, the roll brake splits) are shared by the takeoff roll, autoland's rollout and `free_nose_replay`. `WingsLevel` is shared with the autoland arrest. These belong in `src/roll_laws.rs` (or `src/ground_track.rs`), next to `ground_steer.rs`.
- `runway_fit` and `NoFit` are used by full_auto, autotaxi, circuit, answers and tools/procedure. They belong with `aircraft::takeoff_needed_m`, in `src/runway.rs` (fit) or `takeoff_config.rs`.
- `FlapPace`, `paced_flap_step` and `ClimbVs` are used by circuit: a `climb_out.rs` shared by both.

**Layout:**
```
src/autotakeoff.rs
src/autotakeoff/{fit.rs, rotate.rs, climb_out.rs, throttle.rs, reject.rs, engage.rs, fly.rs, tests/}
src/roll_laws.rs  (roll centreline, RollSteer, brake splits, WingsLevel)
```

**Steps:**
1. Tests out.
2. `roll_laws.rs` with re-exports (autoland and free_nose_replay keep compiling).
3. `fit.rs`, `throttle.rs`, `rotate.rs`, `climb_out.rs`.
4. `reject.rs` and `engage.rs` (async, but self-contained).
5. The `fly` refactor.

### 2.5 src/full_auto.rs (9,955; code ~7,100; tests 7105-9954, 8 modules)

**Clusters:**

| cluster | lines | key items |
|---|---|---|
| mission state + start | 43-430 | `RUN`, `ACTIVE`, `ABORT`, `STAGE`, `MISSION`, `ARRIVAL_RUNWAY`, `retarget_arrival_runway`, `Mission`, `Route`, `SubTools`, `start`, `event` |
| sortie + give-up + hand-back | 462-710 | `SortieGuard`, `give_up`, `hand_back_line`, `hand_back_brakes`, `hand_back_hold` (used by taxi_in, sortie_runner, autotakeoff) |
| squawk + cleared runway | 709-912 | `squawk_owed`, `ensure_squawk`, `ClearedFit`, `cleared_runway_fit`, `judge_cleared_runway` |
| clearances (zone, strip, IFR) | 912-1245 | `transponder_before_departure`, `get_zone_clearance`, `zone_clearance_from_the_strip`, `get_ifr_clearance`, `ReadbackWait` |
| enroute radio + handoffs | 1245-1632 | `INBOUND_MADE`, `inbound_due`, `wait_enroute_working_the_radio` (160), `obey_departure_handoff`, `obey_region_handoff`, `departure_check_in*` |
| waits + departure station | 1632-1920 | `wait_until`, `TakeoffWait`, `wait_takeoff_working_the_radio`, `wait_for_drive`, `self_taxi`, `wait_departure_station`, `departure_station` |
| **fly** (stages 1-4) | 1920-3304 | power-up + engine (2046), altimeter (2262), onto the runway (2307-3236), mission (3237) |
| **fly_route** | 3334-4044 | takeoff (3352), enroute (3491), arrival (3640), final report (3937) |
| arrival watch, taxi-in, airborne end | 4073-4459 | `ArrivalWatch`, `taxi_in_after_landing`, `AirborneLook`, `hold_airborne_end` |
| arrival build + brief | 4459-4999 | `brief_arrival`, `mountain_field`, `call_inbound`, `arrival_wind`, `build_arrival` (138), `tower_arrival_runway` |
| arrival re-pick | 5000-5200 | `RepickWatch`, `ArrivalRepick`, `arrival_splice`, `repick_arrival` |
| departure procedure choice | 5200-5823 | `choose_departure` (137), SID ceilings, `rederive_sid_altitudes`, `cleared_sid_plan`, `request_sid`, `prefer_cleared_sid`, `filed_route*`, `payload_with_legs`, `GradientCheck`, `departure_gradients` |
| ramp radio calls | 5823-6320 | `ready_call`, `taxi_call`, `request_departure`, backtrack (`cleared_elsewhere`, `backtrack_from_runway_start`), `OwedReadback`, `ensure_takeoff_cleared` (142), `readback_before_moving` |
| arm fetch / load / preflight / waiver | 6320-6855 | `ArmFetch`, `ArmLoad`, `arm_load`, `PreflightFacts`, `Gate`, `WAIVER`, `REFUSED`, `take_waiver`, `preflight_gated` (224) |
| runway start + line-up check | 6855-7104 | `RunwayStart`, `classify_runway_start`, `runway_start`, `runway_check*`, `LineUpMiss`, `judge_lined_up`, `lined_up_check` |

**Coupling:**
- The orchestrator calls everything.
- `hand_back_hold` is reused by taxi_in, autotakeoff and sortie_runner, so it should move to a neutral `src/hand_back.rs` (or `flight_state`).
- The waiver statics are read by `grammar::waiver_request` consumers and by `aircraft::TAKEOFF_MARGIN_WAIVED`.

**Pure vs IO:** about 35% pure (judges, lines, SID/STAR plans, preflight). The rest is async orchestration that is already split into named async fns, which helps a lot.

**Layout:**
```
src/full_auto.rs  (state, start, event, pub use)
src/full_auto/{hand_back.rs (or src/hand_back.rs), clearance.rs, radio.rs, waits.rs,
               stages.rs (fly split into stage_power_up/stage_altimeter/stage_to_runway/stage_mission),
               route.rs (fly_route split into takeoff/enroute/arrival), arrival.rs, repick.rs,
               departure.rs, ramp_calls.rs, preflight.rs, runway_start.rs, airborne_end.rs, tests/}
```

**Steps:**
1. Convert the 7 `include_str!("full_auto.rs")` scans and the heli_route and tools/setup scans to a directory scan. Update the `sortie_runner` READ_SHAPES producer paths in the same commit as each move (they fail loudly, which is a useful guard).
2. Tests out (fixture paths to `CARGO_MANIFEST_DIR`).
3. Move the leaf pure modules: preflight, departure, repick, runway_start.
4. Move the async helpers: clearance, radio, waits, ramp_calls, arrival.
5. Split `fly` into 4 stage fns sharing a `MissionCtx` struct (the locals created at 1920-2046). The stage comments already mark the seams. Medium risk.

### 2.6 crates/atc/src/nav.rs (8,423; code ~5,000; tests 5005-8422)

This is the cleanest file: its sections are already separated by `// ---` / `// ===` banners.

| cluster | lines | key items |
|---|---|---|
| ILS | 16-280 | `Ils`, `find_ils`, `loc_courses`, `IlsChart`, `ils_chart*`, `airport_ils`, `ils_platform_alt_ft` |
| apt.dat airport | 281-733 | `Runway`, `AtcFreq`, `FREQ_*`, `AirportInfo`, `find_runways`, `find_airport`, `scan_airports`, parsers |
| taxi network | 733-3263 | `TaxiEdge`, `Stand`, `TaxiNet`, `VacatePlan`, `TaxiRoutePlan`, paint riding (1058-1937: `ride_paint`, `ride_corner_fillets`, `fill_paint_gaps`), `find_taxi_net` (218), `stitch_lines`, `impl TaxiNet` (2197-3263, ~1,070 lines) |
| runway choice | 3264-3409 | `best_runway_for_wind`, `close_parallel_main`, `assignable_runways`, `headwind_for` |
| geo | 3409-3441 | `angle_diff_deg`, `bearing_deg`, `haversine_nm`, `project_latlon` |
| CIFP procedures | 3442-4945 | `ProcKind`, `Fix`, `ProcLeg`, `Arc`, ARINC parsing, `parse_proc_legs`, `list_procedures`, approaches, `VisualSegment`, MSA, SID minimums, `MoraGrid`, STAR constraints, SID/STAR choice, `resolve_proc_fixes` (139), `reconstruct_approach` (125) |
| runway frame | 4947-4996 | `runway_threshold`, `runway_course_deg`, `runway_frame_m` |

**Layout:**
```
crates/atc/src/nav.rs (pub use all)
crates/atc/src/nav/{ils.rs, airport.rs, geo.rs, runway_choice.rs,
                    taxi/{mod.rs, net.rs, paint.rs, route.rs}, cifp/{legs.rs, approach.rs, sid_star.rs, msa_mora.rs}, tests/}
```

**Steps:**
1. Tests out.
2. `geo.rs` (and dedupe the station and crew copies against it).
3. `ils.rs`, `airport.rs`, `runway_choice.rs`.
4. `cifp/*`.
5. `taxi/paint.rs`, then `taxi/net.rs`.

Everything is pure plus file reads, so this is the lowest-risk file to split. Keep `pub use` so `crate::nav::X` paths in the crew package do not change. Drop `has_fix_target`.

### 2.7 src/procedure.rs (8,169; code ~4,700; tests 4712-8168)

| cluster | lines | key items |
|---|---|---|
| published follower state | 92-300 | `RUN`, `ACTIVE`, `ABORT`, `LEG`, `FIX_DIST`, `ROUTE_AHEAD`, `SPLICE`, `TOD`, `PlannedTod`, `held_tod`, `apply_splice` |
| FMS + leg detail | 300-480 | `fms_goto_payload`, `sequence_fms` (async), `leg_progress`, `DETAIL`, `EndAction`, `Plan` |
| MAP + missed | 481-560 | `MISSED_LEGS`, `map_handover_path`, `map_short_of_runway`, `map_path` |
| departure minimums | 557-835 | `dial_target_ft`, `DepartureMinimums`, `departure_target_ft`, `VsAction`, `vs_action`, `MinimumWatch`, `watch_minimum` |
| STAR windows + arrival caps | 835-1264 | `apply_arrival_constraints`, `windows_above_cruise`, `WindowMiss`, `star_window_miss`, `arrival_ceiling_ft`, `profile_ceiling_ft`, `ceilinged_leg_alt`, `arrival_speed_cap`, `star_speed_crossing`, `NEXT_ARRIVAL_HANDOFF_KTS`, `arrival_handoff_cap` |
| lateral leg logic | 1264-1405 | `bearing_to_fix`, `enroute_turn_radius_nm`, `capture_radius_nm`, `leg_turn_deg`, `leg_done`, `leg_captured`, `commanded_alt_ft` |
| pilot/clearance/terrain owners | 1405-1560 | `TERRAIN_FLOOR`, `PILOT_ALT`, `take_altitude`, `CLEARED_CEILING`, `hold_at_cleared`, `PILOT_SPEED`, `look_ahead_points` |
| vertical law | 1560-2140 | VS limits statics, `descent_target`, `vs_law_fpm`, `Vertical`, `DescentProfile`, `IdleDescent`, `speed_protected_vs`, `descent_drag`, `DragVoice`, `drag_barred`, `vertical_tick` |
| approach plan | 2139-2400 | `handoff_index`, `approach_plan_for` (112), `map_approach_legs`, `faf_off_course`, `power_hold_target` |
| cruise speed + steps | 2399-2795 | `cas_for_mach`, `scheduled_cruise_kts`, `cruise_pick`, `apply_step_climbs`, `step_climb_at`, `enroute_legs`, `extend_to_arrival_gateway` |
| start + heading hold | 2795-3010 | `start_scoped`, `alert`, `stand_down`, `HeadingHold` |
| **loop** | 3011-4693 | `fly` (lateral, vertical, terrain, owners, approach handoff) |

**Coupling:** the published statics (`LEG`, `TOD`, `PILOT_ALT`, `CLEARED_CEILING`, ...) are read by monitor, full_auto and tools. Keep them in `procedure.rs` (the module root) or in `procedure/state.rs` with re-exports.

**Layout:**
```
src/procedure.rs
src/procedure/{state.rs, fms.rs, departure.rs, arrival.rs, lateral.rs, vertical.rs,
               approach.rs, cruise.rs, fly.rs, tests/}
```

**Steps:** tests out, then vertical/arrival/departure/cruise (pure), then `state.rs`, then `fly.rs`. `HeadingHold` goes to `heading_bug.rs` (2.3).

### 2.8 src/monitor.rs (7,988; code ~5,600; tests 5532-7987 plus one at 7966)

| cluster | lines | key items |
|---|---|---|
| destination + Snapshot | 55-790 | `Destination`, `route_remaining_nm`, `DESTINATION`, `Phase`, `Snapshot` (129-656, impl ~350), `coarse_phase`, `flying_loop`, `situation`, `flight_sample` |
| Check framework | 790-1000 | `Severity`, `Check` trait, `FireState` |
| checks (one struct each) | 993-2290 plus 5177-5507 | `StallSpeed`, `Overspeed`, `EngineHealth`, `CarbIce`, `LowVacuum`, `AlternatorNotCharging`, `PhaseExpectation`, `TodAhead`, `TodPassed`, `HighOnProfile`, `SinkRate`, `AltitudeBust`, `CabinAltitude`, `RollingOnParkBrake`, `FuelSelectorOff`, `AltimeterSetting`, `CentreTankDry`, `AltimeterTruthCheck`, `Icing`, `StepClimb`, then (after `run`) `AlternatorOffline`, `GyroSync`, `FlapOverspeed`, `StabTrim`, `FuelEndurance`, `FuelPreTakeoff` |
| `build_checks` | 2287-2556 | registry (269 lines) |
| inputs + interventions | 2561-2967 | `Inputs`, `PitchInputs`, `read_inputs`, `note_crew_yoke`, `CrewBrakeLatch`, `note_interventions`, `WorldGates`, `compose_strip` |
| **run** | 2967-5167 | init (to 3178), reactive gate, icing, flow tracker (3386), wreck (3500), cockpit duties (3518), approach vectors (3608), compliance and world (3713-4786), blackbox state (4786), crash (4939) |
| words + region coach | 5507-5660 | `region_coach_line`, `spoken_miles`, `tod_ahead_words`, `CALLSIGN_STALE`, `feed_surface_wind` |

**Misplacement:**
- Checks are defined on both sides of `run`. Gather them all into `monitor/checks/*.rs`, grouped as engine, energy (stall/overspeed/flap), profile (tod/high/step climb), systems (altimeter/fuel/icing/cabin) and ground.
- The world ticks inside `run` (3608-4786) should become `src/world_tick.rs` (see 1.4).

**Layout:**
```
src/monitor.rs
src/monitor/{snapshot.rs, check.rs, checks/{energy.rs, engine.rs, profile.rs, systems.rs, ground.rs},
             registry.rs (build_checks), inputs.rs, run.rs, words.rs, tests/}
src/world_tick.rs
```

**Steps:**
1. Tests out (convert the 2 `include_str!("monitor.rs")` scans first).
2. checks + registry: pure-ish trait impls, no shared state beyond `Snapshot`.
3. snapshot, inputs.
4. `world_tick` extraction from `run`: hoist each `station::*_tick` block into a function taking the tick's inputs. Medium risk, since ordering inside the tick matters (`go_around_tick` runs before the `quiet` block).

### 2.9 src/autotaxi.rs (7,617; code ~4,000; tests 4077-7617 plus test-only items at 229 and 2723)

| cluster | lines | key items |
|---|---|---|
| hold/arrival judges | 138-250 | `arrived_on_line`, `at_the_hold`, `route_end_fix` |
| Pace | 249-362 | `Pace` (used by 10 modules) |
| drive state + progress watch | 362-560 | `ACTIVE`, `BACKTRACKING`, `LAST_ARRIVED`, `RELOCATED`, `relocation_jump_m`, `halt`, `ProgressWatch`, `PROGRESS`, `drive_stalled` |
| Route | 559-1026 | `Route`, `runway_spans`, `Bend`, `route_bends`, `impl Route` |
| steering laws | 1026-1460 | `steer_cmd_with`, `slew_steer`, `damped_err_deg`, `StraightTrim`, `steer_authority`, `brake_steer_split`, `pivot_*`, `BrakeSteer`, `align_pivot_brakes` |
| line-up align | 1460-1805 | `align_*`, `AlignStep`, `LineUpAlign` (used by autotakeoff, free_nose_replay) |
| join | 1805-2082 | `join_allowance_m`, `join_fix`, `JoinTurn`, `join_turn` (126), `JoinCapture` |
| line-up / backtrack geometry | 2082-2736 | `line_up_path*`, `backtrack_*`, `TurnRound`, `line_up_stops_m`, `pick_line_up_plan`, `pick_line_up` |
| lookahead + schedule | 2736-2890 | `lookahead_m_with`, `bend_lookahead_cap_floor_m`, `steer_look_m`, `stop_ramp_kts`, `schedule_kts` |
| start / backtrack / align IO | 2890-3257 | `start`, `start_backtrack`, `align_on_the_runway` (152) |
| **drive** | 3257-4018 | 762 lines |

**Misplacement:** the line-up family (align + geometry, about 1,000 lines) is used by autotakeoff, tools/taxi and scorecard. It is its own `src/line_up.rs` (or `autotaxi/line_up.rs` re-exported). `Pace` is a config-ish type used by 10 modules and could live in `aircraft.rs` beside the taxi numbers.

**Layout:**
```
src/autotaxi.rs
src/autotaxi/{judges.rs, progress.rs, route.rs, steer.rs, join.rs, lookahead.rs, drive.rs, tests/}
src/line_up.rs (align + paths)
```

**Steps:** tests out, then steer/lookahead/join/route (pure), then line_up, then drive.

### 2.10 src/main.rs (6,755; code ~4,900; tests 4906-6755)

| cluster | lines | key items |
|---|---|---|
| mod decls + shims + use lists | 1-155 | ~85 `mod` lines, `pub(crate) use ::atc::...` shims |
| `main` | 156-767 | arg dispatch: subcommands (analyze, debrief, sortie, stt-eval, transcribe...) and boot |
| Ptt + history consts | 773-868 | `Ptt`, `wants_procedural_power_up` |
| **run_session** | 868-3651 | setup (868-1050); per-utterance: mode edges (1091-1155), station swap (1155-1250), monitor-ordered go-around (1252), discarded transmission handling (1285-1450), radio gates (1590-1650), confidence (1651-1740), radio routing (1741-1880), domain gate / follow-up (1878-2272), shortcuts and conversation (2272-3079), grammar fast path + back-references (3079-3226), confirmation + execute + undo (3226-3631), history (3631) |
| helpers | 3655-4300 | `preview_checklist`, `levenshtein`, `load_wav_16k`, `transcribe_file`, `analyze_commands` (131), `listen_once`, `next_input`, `say`, `crew_radio_facts`, `pilot_line_pending`, `loop_owed*`, `say_or`, `CaptureTuning`, `boot_health`, `install_home_station` |
| shortcuts | 4301-4660 | `Shortcut`, `question_shortcut` (174), `pre_grammar_capture`, `state_capture_note` |
| ATC routing helpers | 4660-4906 | `try_route_content_atc`, `repeat_controller`, `void_briefing`, `radio_panel_line`, `atc_panel_payload`, `SimRadioSync` |

**Misplacement:**
- `Shortcut`, `question_shortcut`, `pre_grammar_capture` and `state_capture_note` belong in `src/shortcuts.rs`. `sortie_runner --check` imports them via `crate::`, and CLAUDE.md says "the loop's own, order test-pinned", so keep it one function, just in another file.
- `analyze_commands`, `transcribe_file` and `load_wav_16k` belong in `src/cli/` (or `analyze.rs`).
- `levenshtein` belongs in `phrase.rs`, `TOD_NM_PER_1000FT` in `plan.rs`, and the radio panel and ATC helpers in `src/atc_link.rs`.
- Tests of router, answers and undo functions belong in those modules.

**Layout:**
```
src/main.rs           (mods, main() dispatch only)
src/session.rs        run_session -> later session/{setup.rs, radio.rs, gates.rs, dispatch.rs, execute.rs}
src/shortcuts.rs, src/cli.rs (analyze/transcribe), src/atc_link.rs
```

**Steps:**
1. Move the tests that test other modules' functions to those modules.
2. `shortcuts.rs`: update the `sortie_runner` paths and the `include_str!("main.rs")` scans in main, pushback and pollers to scan `session.rs` / `shortcuts.rs`.
3. `cli.rs`.
4. `session.rs` with `run_session` moved whole.
5. Carve `run_session` into an `UtteranceCtx` plus stage fns along the comment seams. Highest risk in the repo: it carries `await`s, guard lifetimes ("no guard held across an await") and many `continue`s. Do it last, one stage at a time, with `xvl sortie --check` over all bench specs as the golden test.

## 3. Global step order (smallest risk first)

Each step is one commit and must keep `XCOPILOT_BLACKBOX=off cargo test --workspace` green.

1. **Hygiene, no moves.**
   - Delete the dead consts (autoland 448, 450, 6322, 6330) and `nav::has_fix_target`.
   - Remove `#![allow(dead_code)]` from autoland and act on what rustc reports.
   - Remove the stale allows (main.rs:37, monitor 118/789, voice 60/1449) where rustc agrees.
2. **Make source scans directory-aware.** `include_str!` single-file scans: full_auto x7, main, monitor x2, autoland, tools/setup, pushback, pollers, recover. Plus heli_route's file list. Replace each with the recursive walker already used in `flight_state.rs`. Fixtures go to `concat!(env!("CARGO_MANIFEST_DIR"), ...)`.
3. **Tests out of every file** (`#[cfg(test)] mod x;` resolving to `src/<module>/x.rs`, with no parent rename). About 48k lines leave the ten files, and nothing in production changes.
4. **Pure, stateless leaves in nav** (the lowest coupling): geo, ils, airport, runway_choice, cifp, taxi/paint, taxi/net.
5. **Shared-infrastructure moves with re-export shims:**
   - `crew_radio.rs` (from circuit)
   - `heading_bug.rs` (from circuit)
   - `roll_laws.rs` (from autotakeoff)
   - `runway.rs` (Runway and geometry from autoland, `runway_fit` from autotakeoff)
   - `line_up.rs` (from autotaxi)
   - `hand_back.rs` (from full_auto)
   - `units` / `geo` consolidation
   - `shortcuts.rs` / `cli.rs` (from main)

   Afterwards, a separate mechanical sed commit per shim to repoint callers, then delete the shim.
6. **Pure clusters inside each giant**, in this file order (least shared state first): autotaxi, autotakeoff, procedure, circuit, autoland, monitor checks, full_auto judges.
7. **Static-owning sections of station.rs**: give each a `reset()`, then move the sections with their statics, then split `impl Station`.
8. **Move the loops whole** into `run.rs` / `fly.rs` / `session.rs`. This is still mechanical: the bodies are unchanged.
9. **(Behavioural tier, separate project)** Convert each giant loop's locals into a `LoopState` struct and extract per-phase methods along the existing `// ---` comment seams. Order: autotaxi::drive, autotakeoff::fly, circuit::fly, procedure::fly, full_auto::fly (stage fns), monitor::run (world_tick), autoland::spawn_autoland, main::run_session. Gate each on the bench/landings and bench/follower replays, `xvl sortie --check` over bench/sorties, and one real sortie per airframe class. This tier needs an `unflown` issue per refactored loop, per the battle-test rules.

## 4. Size after steps 1-8 (estimate)

Every new file would be under about 1,500 lines except the untouched loops: `spawn_autoland` ~3k, `run_session` ~2.8k, `monitor::run` ~1.2k after `world_tick`, `procedure::fly` ~1.7k, `circuit::fly` ~1.5k. Those five functions are the remaining debt, and only step 9 addresses them.
