# Duplicated rules audit (read-only), 2026-10-04

Scope: `src/`, `crates/atc`, `crates/sim` at `365f504`. Every finding below was verified by reading the
code at the cited lines (line numbers as of that commit). Ranked by (bug risk x number of copies).
Ratings: Risk = what goes wrong when the copies disagree; Effort = to unify.

Two findings are plain bugs found while auditing, not just duplication: **#7 (variation sign in
setup_enroute and the procedure terrain-look fallback)** and **#4 (governor ceiling inside the
overspeed caution band)**. They are worth fixing even if nothing else here is.

---

## 1. "Is an automation flying?" : 9 copies with different membership

Rule: which loops count as the crew having the aircraft (silences pilot advice, picks Monitor vs
Crew checklists, picks caution factors, resets vs advises the altimeter, panel "copilot flying").

| Location | Members |
|---|---|
| `src/flow.rs:446` `automation_flying` | full_auto, procedure, circuit, autoland |
| `src/ils.rs:596` `automation_flying` | same 4 (verbatim copy) |
| `src/tools/approach.rs:125` `automation_flying` | autoland, procedure, circuit, full_auto |
| `src/monitor.rs:350` `Snapshot.automation_flying` | procedure, full_auto, circuit, autoland |
| `src/flow.rs:459` `mission_owner` | + heli_route, heli_circuit, autotakeoff, hover |
| `src/monitor.rs:682` `flying_loop` | + pushback, autotaxi, heli_*, hover, autotakeoff |
| `src/monitor.rs:703` `mission` | + instructor lesson |
| `src/monitor.rs:919` callout mute | recover, autoland only |
| `src/monitor.rs:4945` control-idle | Snapshot set + autotakeoff + recover |
| `src/monitor.rs:4898` `critical_loop` | autoland, autotakeoff, recover |
| `src/auto_status.rs:203` `panel_extras` | procedure, circuit, autoland, autotakeoff, recover |

How they differ: `tools::recover::ACTIVE` is in none of the four "automation flying" copies;
`heli_route`, `heli_circuit`, `hover` and `autotakeoff` are missing from `Snapshot.automation_flying`,
so with the helicopter route or a stand-alone takeoff flying, the monitor's checks treat a pilot as
flying: `AltimeterSetting` advises instead of resetting, `centre_tank_dry` advises instead of
switching, the stall caution uses the hand-flown 1.2 band (`monitor.rs:1066`), and the callout mute at
`monitor.rs:919` lets pilot-directed advice ("lower the nose, add power") play over the circuit and
the follower.

Single home: `flight_state.rs` (already "the one page" of process-wide facts) with one
`fn crew_has_aircraft() -> Option<Owner>` returning the innermost owner, plus `is_mission()`; every
copy above becomes a call. A source-scan test like the existing `static ACTIVE` pin keeps new loops
from being forgotten.

Risk: High. Effort: Low-Medium (mechanical, but each caller's intent must be checked).

---

## 2. Default approach speed (and Vr) when the profile declares none : 5 copies

Rule: CLAUDE.md "Approach = max(1.3 Vso, 1.2 Vs)".

| Location | Formula |
|---|---|
| `src/tools/mod.rs:871` `fallback_approach_kts` (registry, autoland) | max(1.3 Vso, 1.2 Vs, **70**) |
| `src/aircraft.rs:3514` `generic_from_acf` | max(1.3 Vso, 1.2 Vs), **no 70 floor** |
| `src/instructor.rs:385` `Refs::from_profile` | **1.3 x CLEAN Vs**, max 70 (the pre-fix rule) |
| `src/instructor.rs:391` "slow" ref fallback | 1.3 Vso |
| `src/circuit.rs:9440` test `Airframe::load` | 1.3 x clean Vs, max 70 |
| `src/tools/recover.rs:238` recovery hold | 1.3 x clean Vs, max 70 (Vs defaults to 60 at `tools/mod.rs:278`) |

Vr fallback, same family: `tools/mod.rs:312` 1.15 Vs (min 55) vs `aircraft.rs:3513` 1.1 Vs.

How they differ: the instructor still uses the rule the registry dropped after the SR22 floated and
bounced at 90 kt (LEMG 13, 2026-09-29). On an airframe without `approach_kts` the instructor briefs,
scores (`with_landing_speed`, `instructor.rs:2095`) and sizes the pattern (`pattern_speed_kts`,
`pattern_alt_agl_ft`) against a different approach speed than autoland and the circuit fly.

Single home: `aircraft::approach_kts(profile) -> f64` and `aircraft::vr_kts(profile)` (pure), used by
the registry, the instructor and tests; `generic_from_acf` writes its result into `reference` with the
same function.

Risk: High (wrong scoring, and any new caller picks the wrong copy). Effort: Low.

---

## 3. The pilot's "climb to X" runs a Cessna's climb law on every airframe

Rule: an autopilot climb is speed-protected and power-limited per airframe ("All automation numbers
are per-airframe").

| Location | Law |
|---|---|
| `src/tools/autopilot.rs:296-304, 333, 715-737` (`fly_to` watcher) | throttle firewalled to 1.0, VS capped 700 fpm (auto rate), eases VS only when IAS < **60 kt flat**, restores above 68, back to `CRUISE_THROTTLE` **0.65 flat** on capture |
| `src/circuit.rs:1282` `climb_limit_fpm` | fades toward blue line / 1.15 Vs of THIS airframe |
| `src/autotakeoff.rs:2523, 2630` `climb_vs_fpm`, `climb_vs_protected` | Vy / Vne based |
| `src/climb_watch.rs` | the "answer is watched" layer, only for circuit and follower |

How they differ: on the 737 or A330 (clean stall 130 to 170 kt) the tool's 60 kt protection never
fires; a pilot-commanded climb at full power with no follower running is protected by nothing, which
is the #181 failure mode (166 kt to 73 kt at 9,600 fpm) by another door. The 0.65 "cruise" lever is
also a 172 number.

Single home: one `climb_protection(ias, trend, profile)` (circuit's `climb_limit_fpm` with
`speed_floor_kts()` / `climb_floor_kts`) used by the tool's watcher, and `climb_watch` armed for the
tool's climb too.

Risk: High. Effort: Medium.

---

## 4. Speed ceilings: four numbers, and the governor's sits inside the overspeed caution

| Location | Ceiling |
|---|---|
| `src/aircraft.rs:2969-2984` `speed_ceiling_kts` (governor cap, `speedhold.rs:996`) | Vne - 15 |
| `src/monitor.rs:991, 2491` overspeed caution | 0.90 x Vne |
| `src/procedure.rs:2477, 2489` `cruise_cap_kts` | 0.90 x Vne - 10 |
| `src/autotakeoff.rs:2597, 2613, 2646` climb overspeed guard | min(Vne, 270) - 20 |

How they differ: Vne - 15 > 0.9 Vne whenever Vne > 150 kt, so the governor accepts targets that the
monitor then calls "Overspeed!" (172: 148 vs 146.7; 737 Vmo 340: 325 vs 306; A330 Vmo 330: 315 vs
297). Only the follower's cruise knows about the caution band.

Single home: `aircraft::speed_ceiling_kts()` defined as `min(Vne - 15, 0.9 Vne - margin)` (or the
caution factor moved into `aircraft` and the monitor reading it from there), with `cruise_cap_kts` and
the takeoff guard derived from it.

Risk: High (spurious cautions, and a pilot's "hold 320" is accepted into the band). Effort: Low.

---

## 5. Autopilot engage: a confirmed engager exists, most loops press blind

Rule: "The autopilot is engaged and CONFIRMED, never assumed" (`autoland::ApEngage`, `autoland.rs:620`:
FD commands first, re-press until `servos_on` reads on for 2 live ticks, hold level meanwhile, give up
in 10 s).

| Location | What it does |
|---|---|
| `src/autoland.rs:1880, 3673` | FD commands + servos, through `ApEngage` |
| `src/tools/setup_enroute.rs:687` | reuses `ApEngage`, FD commands |
| `src/tools/recover.rs:791` | FD commands + servos, then heading mode |
| `src/tools/autopilot.rs:247, 457, 841` | FD commands + servos, read once |
| `src/autotakeoff.rs:3368` `engage_climb` | servos press once, **no FD commands**, reads modes back (FD only at the arm, `autotakeoff.rs:3582`) |
| `src/circuit.rs:2516, 2657, 4067, 5297` | servos press, **no FD**, no confirm |
| `src/procedure.rs:3095, 3914` | servos press, **no FD**, no confirm |
| `src/full_auto.rs:4392` | servos press if read off, **no FD** |
| `src/autoland.rs:3935, 4592` | servos press, **no FD** |
| `src/tools/approach.rs:322` | servos press, read once, **no FD** |

How they differ: `ApConfig.fd_commands` exists because the C90's `fd_monitor` drops the servos unless
the FD switch is up (`tools/autopilot.rs:244`). Every re-engage in the circuit, follower and full auto
skips it, so on that class of airframe the "re-engage lateral" guards press a button that cannot hold.
Mode presses are also re-derived per site (some check status first, `procedure.rs:3913` presses
servos and HDG together whenever HDG reads off).

Single home: `autoflight::engage_servos(xp, fd_commands) -> Engaged` wrapping `ApEngage` (press, FD,
confirm), and `autoflight::ensure_mode(status, cmd)` (the read-before-fire toggle that
`tools/autopilot.rs:282 engage_mode` already is). The FD list should be readable from
`aircraft::live_profile()` so loops do not need it threaded.

Risk: High on FD airframes, Medium elsewhere. Effort: Medium.

---

## 6. Turn-reversal protection on the heading bug : 2 implementations, 3 loops with none

Rule: CLAUDE.md "every steer goes through `circuit::bug_toward`" (never reverse a turn in progress,
lead big turns by at most `TURN_ENTRY_LEAD_DEG`).

| Location | Behaviour |
|---|---|
| `src/circuit.rs:2123` `bug_toward` | stateless; reversal bank 1 deg when bug on nose, else 3; entry lead when turn > 25 |
| `src/procedure.rs:2904-2990` `HeadingHold::decide` | its own state machine (Entry / Rollout / Direct); entry lead when turn > **10**; corrections every 30 s past 5 deg. Production follower uses this; `bug_toward` appears in procedure only in tests (`procedure.rs:7080, 7089`) |
| `src/autoland.rs:4147-4210` | `circuit::BugHold` + `bug_knob`: once per phase / deadband, **no reversal protection** (pattern, reversal and FinalJoin legs) |
| `src/heli_route.rs:2110-2125, 2216, 2364` | 2 deg hysteresis only |
| `src/tools/setup_enroute.rs:690`, `src/hover.rs:1399` | direct writes |

How they differ: two pieces of code solve the same X-Plane HDG-mode long-way-round problem with
different thresholds; autoland's pattern legs and the helicopter route have neither.

Single home: one `HeadingKnob` (BugHold's once-per-key + deadband, with bug_toward's reversal logic
inside `decide`) used by circuit, follower, autoland and heli_route.

Risk: Medium-High (a reversal on a low pattern leg). Effort: Medium.

---

## 7. True to magnetic: one helper, inline copies, two with the wrong sign (BUG)

Rule: X-Plane's `magnetic_variation` is positive-WEST: `mag = true + var`
(`autoland::true_to_mag`, `autoland.rs:7483`, verified live at KPDK).

| Location | Formula | Verdict |
|---|---|---|
| `src/autoland.rs:7483` `true_to_mag` | true + var | correct (circuit, procedure, autotakeoff use it) |
| `src/course_leg.rs:79, 1032` | true = mag - var | correct, inline |
| `src/heli_route.rs:2050, 2126, 2209, 2362` | var re-derived as psi - magpsi, then track - that | correct, inline |
| `src/tools/setup_enroute.rs:691` | `bug = track_true - var` with the raw dataref (`:671`) | **wrong by 2 x var** |
| `src/procedure.rs:3944` terrain-look fallback | `true = bug_mag + mag_var` | **wrong by 2 x var** (fallback only) |

Impact: the enroute setup dials its first heading 2 x variation off (2 to 6 degrees in Europe, 20+ in
much of the US); the follower's terrain probe, when it has neither a fix bearing nor a heading read,
looks 2 x var off track.

Single home: `nav::true_to_mag` / `nav::mag_to_true` (move from autoland), and every inline
`+/- var` replaced.

Risk: High where variation is large. Effort: Low.

---

## 8. Stop and park: four sequences with different thresholds and mechanisms

Rule: "A mission that ENDS on the ground stops and parks": wheel brakes to a stop (max above 25 kt),
wait out the held-brake tail, then `set_park_brake_confirmed`.

| Location | Stopped at | Mechanism |
|---|---|---|
| `src/full_auto.rs:594` `hand_back_hold` | < 1 kt | `brakes_max`/regular pulses, waits `HAND_BACK_BRAKE_TAIL_S`, confirmed park |
| `src/autoland.rs:2562-2604` rollout end | <= 2 kt | plugin `taxi_brake` channel, then confirmed park, no tail wait |
| `src/taxi_in.rs:550-562` arrest | **> 3 kt or unread: sets the PARKING brake while rolling** | `set_park_brake_confirmed` as the brake |
| `src/taxi_in.rs:452, 781, 864, 888` | none | `drive_park_brake`, unconfirmed |
| `src/pushback.rs:1645` `answer_stop` | none | confirmed park, no stop phase |
| `src/hover.rs:2017` | `touchdown_brake_due` < 1 m/s | `drive_park_brake` |

How they differ: `hand_back_hold` says "a parking brake is a chock"; the taxi-in arrest uses it as the
brake on a rolling aircraft. The brake-tail wait (the LEMG SR22 lesson, park read back 0.294) exists
only in `hand_back_hold`.

Single home: `hand_back_hold` (already pure-testable against `dyn XPlane`) as the only stop-and-park,
parameterised by who calls; the taxi-in arrest and autoland's stopped branch call it.

Risk: Medium-High. Effort: Low-Medium.

---

## 9. "Stopped on the ground" : 12+ thresholds, onground dataref redeclared 15 times

| Location | Stopped means |
|---|---|
| `src/tools/jet_start.rs:260, 266` | onground > 0.5 and gs < 0.5 m/s (~1 kt) |
| `src/tools/loading.rs:176-185` | < 0.5 m/s |
| `src/tools/setup.rs:427-432` | < **1.0 m/s (~2 kt)** |
| `src/full_auto.rs:535` / `src/flight.rs:442` / `src/sortie_runner.rs:2774` | < 1 kt |
| `src/autoland.rs:2562` | <= 2 kt |
| `src/taxi_in.rs:559` | <= 3 kt |
| `src/autotaxi.rs:3685, 3701, 3747` | 1.5 kt / 0.7 kt |
| `src/tools/speedhold.rs:567` | 1.5 kt |
| `src/cabin.rs:283` | 1 kt |
| `src/hover.rs:518` | 1 m/s |
| `crates/atc/src/station.rs:3262` | 3 kt |
| `src/full_auto.rs:1724` `confirmed_ground` | onground AND agl <= 50 ft (a second definition of on-ground) |

`"sim/flightmodel/failures/onground_any"` is redeclared as a local const in 15 files
(`full_auto.rs:35`, `autotaxi.rs:63`, `autoland.rs:442`, `autotakeoff.rs:52`, `hover.rs:1157`,
`tools/gear.rs:15`, `tools/cycle.rs:45`, `tools/jet_start.rs:253`, `tools/reverse.rs:31`,
`tools/heli.rs:22`, `tools/speedhold.rs:56`, `tools/engine.rs:18`, plus inline strings in
`monitor.rs:310`, `answers.rs:122/210/343/569`, `heli_route.rs:2777`) despite
`datarefs::ON_GROUND_ANY`.

Single home: `sim::state` or `flight_state`: `fn stopped_on_ground(on_ground, gs_mps) -> bool` with
one `STOPPED_KTS`, and `datarefs::ON_GROUND_ANY` everywhere. Tolerances that are genuinely different
(a taxi governor's "arrived") can keep their own names.

Risk: Medium (the setup tool ends a flight at 2 kt while the load sheet refuses at 1). Effort: Low.

---

## 10. "Enough runway to LAND" : the twin of the five-copy takeoff case

| Location | Rule |
|---|---|
| `src/flightplan.rs:569` `runway_needed_ft` | landing distance, **else the TAKEOFF distance**, unfactored, vs longest runway |
| used by `src/plan.rs:557` (local planner), `src/full_auto.rs:6666` (preflight dest), `src/plan_audit.rs:677` | same |
| `src/autoland.rs:218` `self_pick_min_length_m` | landing distance x **1.3**, min **1,200 m** |
| `src/autoland.rs` `commit_flare_latched`, `hold_roundout`, `overflight_allowance_nm` | the book figure less the air flown (a different, legitimate question) |

How they differ: the planned path accepts a runway equal to the bare book landing distance; the
emergency path demands 1.3x and 1,200 m. A destination the preflight passes can be one the self-pick
would refuse, and an airframe with only a takeoff figure is checked against the wrong number.

Single home: `aircraft::landing_needed_m(weight?, factor)` beside `takeoff_needed_m`, with the factor
and floor in one place; `runway_needed_ft` deleted.

Risk: Medium-High. Effort: Low.

---

## 11. Throttle: the approach skips the airframe's governor, two tools skip its lever

a) Law. `speedhold::throttle_adjust_live` applies the profile's `speed_governor:` (lead, gain, step;
the #169 A330 lag fix). Used by `circuit.rs:4142, 5778` and autoland's missed approach
(`autoland.rs:7886`). **Autoland's approach law uses the plain `throttle_adjust`
(`autoland.rs:4381`)**, the shared GA law, on the one phase where speed control matters most.

b) Lever. `aircraft::throttle_dataref()` exists because the Citation X ignores `throttle_ratio_all`.
`src/tools/engine.rs:14, 175, 186` (SetThrottle reads and ramps) and `src/tools/controls.rs:322,
378, 479, 500` (brakes tool power gate and close) use the stock `throttle_ratio_all` directly; the
other 8 modules route through local `throttle_dref()` wrappers (8 identical copies of a one-line
function).

Single home: `throttle_adjust_live` as the only public law (make the plain one test-only), and
`aircraft::throttle_dataref()` for every read and write.

Risk: Medium-High. Effort: Low.

---

## 12. Flap placard lookup : 5 implementations

| Location | Rule |
|---|---|
| `src/circuit.rs:1074` `placard_of` | lowest Vfe of detents with ratio <= flap + 1e-6 **including ratio 0**; INF when none |
| `src/autoland.rs:5530` `flap_placard_kts` | same but **excludes ratio 0**; `None` when none |
| `src/monitor.rs:5317` FlapOverspeed | ratio <= flap + **0.05**; alerts at Vfe + 5 |
| `src/rapport.rs:366-390` | per detent, crossed at Vfe + 2, cleared at Vfe - 3 |
| `src/tools/flaps.rs:109` | target detent only, refuses at > Vfe |
| `src/autoland.rs:5370, 5393` `flap_within_limits`, `flap_held_within_limits` | extend margin 5, retract margin 8 |

How they differ: a profile entry at ratio 0 with a Vfe (some generic `.acf` reads do this) changes
the circuit's placard but not autoland's; the monitor's 0.05 tolerance can pick the next detent's
Vfe on closely spaced detents.

Single home: `aircraft::flap_placard_kts(ratio)` (pure, reading `flap_limits()`), with the margins as
named constants beside it.

Risk: Medium. Effort: Low.

---

## 13. "Too slow" ratios over Vs : 1.13, 1.15, 1.2, 1.3 with overlapping jobs

| Ratio | Where | Job |
|---|---|---|
| 1.13 | `monitor.rs:1026` `AUTO_CAUTION_FACTOR` | caution with an automation flying |
| 1.15 | `circuit.rs:1213, 1240, 1246` `climb_floor_kts`, `climb_floor_in_config` | climb limiter floor |
| 1.2 | `aircraft.rs:2946` `SPEED_FLOOR_RATIO` | "no automation target below" |
| 1.2 | monitor hand-flown caution | caution with a pilot |
| 1.3 | `circuit.rs` `PATTERN_MIN_STALL_MULT`, `recover.rs:238`, `instructor.rs:387` | pattern, recovery hold, approach |
| 0.9 x floor | `climb_watch.rs:40` `DEEP_FLOOR_RATIO` | take the climb at once |

How they differ: the climb limiter lets a climb bleed to 1.15 Vs, below the 1.2 Vs floor CLAUDE.md
says no automation target goes under, and the 1.13 caution then sits between the two. A change to one
(the #169 climb-floor fix was made at 1.15) does not move the others.

Single home: named ratios in `aircraft` (`FLOOR`, `CLIMB_FLOOR`, `CAUTION_AUTO`, `PATTERN_MIN`) with a
test asserting their ordering.

Risk: Medium. Effort: Low.

---

## 14. Angle wrap / heading difference : 20+ copies, same names with different meaning

Copies of `(a - b + 540) rem 360 - 180`: `free_nose_replay.rs:43`, `instructor.rs:506`,
`pushback.rs:860`, `reversal.rs:39`, `ils.rs:418`, `approach_guide.rs:136`, `test_card.rs:183`,
`answers.rs:90`, `autoland.rs:78`, `course_leg.rs:58`, `ground_steer.rs:656`, `taxi_guide.rs:724`,
`atc/station.rs:8631`, `atc/nav.rs:3409`, plus ~40 inline uses (circuit 11, autoland 9, nav 5).

Differences that bite:
- `autoland::heading_diff` returns the ABSOLUTE difference; `reversal::heading_diff` and
  `course_leg::heading_diff` with the same name are SIGNED.
- `answers::angle_delta_deg(a, b)` is b - a (opposite sign to every other `(a, b)`).
- `instructor::wrap180` uses `%`, wrong for inputs below -540.

Single home: `nav::angle_diff_deg` (signed) and `nav::angle_gap_deg` (abs), everything else deleted.

Risk: Low-Medium (a sign mix-up on reuse). Effort: Low.

---

## 15. Distance / bearing / runway projection math

- Canonical: `atc/nav.rs:3414` `bearing_deg`, `:3423` `haversine_nm`, `:3434` `project_latlon`
  (re-exported via `autoland.rs:16`), and `autoland::along_track_nm` / `cross_track_nm`
  (`autoland.rs:7491, 7504`, spherical, from `course_deg`).
- Copies: `flight.rs:1769` `distance_nm` (flat-earth), `rapport.rs:180` `dist_m` (flat, 6,371 km),
  `approach_guide.rs:141` `bearing_deg` (verbatim copy of nav's), `taxi_guide.rs:718` and
  `ground_steer.rs:652` local-frame `bearing` + `angle_diff`, `free_nose_replay.rs:358`.
- Runway frame re-derived flat from the END POINTS in the tower: `atc/station.rs:4844` `on_runway`
  and `:6161` `on_runway_pavement`, and ~35 inline `111_320` projections in `atc/nav.rs`.

Risk: Low (accurate at these ranges), but the crew measures a runway from `course_deg` and the tower
from end points, so the two can disagree by a metre or two near the edge. Effort: Medium (crate
boundary: put a `RunwayFrame` in `atc::nav` and have autoland's `Runway` use it).

---

## 16. "On the runway" : three geometries for one question

| Location | Lengthwise | Lateral |
|---|---|---|
| `atc/station.rs:4844` `on_runway` (also the crew's `party_line::on_runway`, `Sample.on_runway`, vacated report, after-landing list) | 2 % of length past each end | width/2 + 10 m (40 m default) |
| `atc/station.rs:6161` `on_runway_pavement` (tower occupancy, go-around judgement) | 30 m past each end | width/2 + 5 m |
| `autotakeoff.rs:840-912` runway under the nose | `LATERAL_MAX_M` | own |
| `autotaxi.rs:600` `on_runway_span` | route spans | none |

How they differ: on a 3 km runway the crew's "on the runway" reaches 60 m past the end and 10 m off
the edge while the tower's occupancy uses 30 m and 5 m, so the crew can still be "on" when the tower
has it clear (conservative) but the after-landing list and the vacated report use the looser test.

Single home: one `nav::RunwayFrame::locate(lat, lon) -> (along_m, cross_m)` and two named envelopes
(pavement, strip) chosen by the caller.

Risk: Medium-Low. Effort: Low-Medium.

---

## 17. Flight phase and "airborne" re-derived outside the flight model

CLAUDE.md: `flight.rs` is the ONE answer. Local derivations still in use:

- Airborne: `flight.rs:445-446` (50 ft for 1.5 s), `debrief.rs:22` (100 ft), `full_auto.rs:3323` and
  `circuit.rs:1392` `AIRBORNE_MIN_AGL_FT` 300 ft (a takeoff "completed"), `autotakeoff.rs:4151, 9024`
  (agl > 8 ft and IAS > 0.8 Vr), `hover.rs:1272` 100 ft.
- Approaching: `rapport.rs:378` (vsi < -250 or < 12 nm or agl < 3,000 with flap), `answers.rs:1545`
  (agl < 3,000 and vvi < -100, or flap > 0.3), `monitor.rs:670` (model Descent under 3,000 = Approach),
  `tempo.rs:207` (low < 1,000).

Risk: Medium (copilot answers "how's the speed" with the climb verdict while the model says
approach). Effort: Medium (route through `flight::published()`).

---

## 18. Into-wind runway : the brief can name a runway the crew will not fly

- `atc/nav.rs:3264` `best_runway_for_wind`: assignable runways only, longest parallel, used by the
  tower, `full_auto::wind_runway` (`full_auto.rs:5048`, calm under 4 kt), `plan_audit::choose_arrival`,
  `sortie_runner::pick_into_wind`.
- `rapport.rs:731` `into_wind_end` (the spoken arrival brief): calm under **3 kt**, no assignable
  filter, no parallel rule.
- `autoland.rs:246` self-pick tailwind, `plan_audit.rs:490` crosswind, `heli_route.rs:224`,
  `weather.rs:484`, `atc/station.rs:7011`: the headwind / crosswind trig written 7 times.

Risk: Medium-Low (said vs flown mismatch). Effort: Low (`into_wind_end` calls `wind_runway`; one
`nav::wind_components(from, kts, hdg)`).

---

## 19. Terrain clearance numbers per module

`procedure.rs:1380` 1,000 ft over the ground ahead (15 nm); `circuit.rs:727-732` live floor 500 / 400
/ 800; `circuit.rs:1402, 1410` pattern 1,000 / 600; `circuit.rs:2846` descent 500; `autoland.rs:5651`
500 AGL; `heli_route.rs:119` 400; `flightplan.rs:36` cruise 2,000; `tools/autopilot.rs:313`
`MIN_TARGET_AGL_FT` 500 under the wheels.

Most differences are legitimate by phase. The one that is not: the pilot's altitude tool checks the
ground AHEAD only while a follower publishes `terrain_floor_ahead_ft` (`tools/autopilot.rs:363`);
otherwise it checks 500 ft over the ground directly beneath, which is the 2026-09-13 ridge case the
comment itself describes.

Single home: a small `terrain::Clearance` table by phase, and the tool probing ahead itself
(`procedure::look_ahead_points` + `terrain::max_elevation_ft`) when no follower runs.

Risk: Medium. Effort: Medium.

---

## 20. 250 kt below 10,000 ft

`tools/speedhold.rs:183-184` (`BELOW_10K_LIMIT_KTS`, `SPEED_LIMIT_ALT_FT`, with hysteresis) and
`tools/setup_enroute.rs:55-56` (its own `BELOW_10K_FT`, `BELOW_10K_KTS`, no hysteresis). The
`speedhold.rs:435, 996` `unwrap_or(250.0)` ceiling fallbacks reuse the same number for a different
meaning (airframe ceiling unknown).

Risk: Low. Effort: Trivial.

---

## Not findings (checked, already unified)

- Takeoff runway length: every caller goes through `aircraft::takeoff_needed_*` (plan_audit included).
- Park brake writes: all through `tools::controls::drive_park_brake` (only the stop sequences differ, #8).
- Stall in configuration: `monitor::stall_speed_at_flap` delegates to `aircraft::stall_speed_in_config`.
- Gear limit: `aircraft::gear_limit_kts` everywhere.
- Replay / pause: only `xplane::sim_frozen` reads them.
- Stabilised approach: consolidated in `stable_approach.rs` (#199).
