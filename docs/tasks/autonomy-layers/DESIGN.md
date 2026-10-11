# Autonomy in layers (design, 2026-10-11)

The redesign the autonomy freeze in CLAUDE.md waits for. Status: design, nothing built.
The freeze stands until phase 2 below passes its acceptance test.

## 1. The problem

Every sortie that fails produces a fix tuned to one airframe at one field. The evidence:

- `src/aircraft/schema_laws.rs` carries 15 per-airframe flight-law structs. Each is a gain
  set or a fitted model: `FlareElevatorProfile`, `FlareModelProfile`, `ArrestLeadProfile`,
  `ArrestRollProfile`, `GroundPlantProfile`, `RolloutSteerProfile`, `SpeedGovernorProfile`,
  `FinalTrackProfile`, `RotateElevatorProfile`, `RollRudderProfile`, `FlarePowerProfile`, ...
  Each one was fitted from that airframe's sorties. A new airframe means new sorties and a
  new fit.
- The flying loops are about 43,000 lines (`src/autoland`, `src/circuit`, `src/autotakeoff`).
  `docs/dev-history.md` is 14,000 lines, most of it per-airframe repairs to those loops.
- We drive the aircraft's autopilot mostly as a vertical-speed and heading servo. Across
  `src/`, the presses are `VerticalSpeed` 31, `Heading` 24, `AltitudeHold` 17,
  `LevelChange` 8, `Approach` 9. A glidepath is flown as a VS dial rewritten each tick
  (`path_descent_fpm`), and a speed as our own throttle law. The aircraft's own modes, which
  its author tuned for it, are mostly left unused.
- Our inner loops run slowly and late. Control ticks: flare and arrest 200 ms, final 330 ms,
  other phases 540 ms (`autoland::guidance::tick_ms`). Web API: about 80 requests per
  second, single round trips of 30 to 80 ms (blackbox `api_load`, 2026-10-11). The VSI
  needle trails the real vertical speed by about a second (LEJR 02, 1791552408). A flare
  decision acts on information 0.3 to 0.5 s old, while the aircraft changes over 1 to 2 s.
  A loop with that much delay is stable only with gains fitted tightly to one airframe.
  That is the patching, built into the design.

## 2. The target: three layers, and only the bottom one knows the airframe

| Layer | Job | Who | Rate |
|---|---|---|---|
| 3. Mission | phases, ATC, checklists, decisions, gates | Rust, as today | event |
| 2. Guidance | paths, target speeds, headings, altitudes, from geometry and book numbers | Rust, one code path for every airframe | 1 to 2 Hz |
| 1. Control | targets into surfaces and levers | the aircraft's own AP and A/T, through `autoflight` | the sim's |
| 1b. Gaps | what no autopilot does: flare, takeoff roll, rotation, rollout, taxi | generic laws in the plugin's flight loop | every frame |

**The rule:** a profile declares capabilities and performance numbers, never controller gains.
Capabilities are modes the AP has, whether it has an A/T, and whether it flares. Performance
numbers are Vref, flap speeds, power settings and runway numbers. A new airframe should need a
profile and a clean `xvl conform`, and no sorties to fit gains.

### Layer 2 to layer 1: modes first

Guidance states its intent as modes, through the panel owner (`autoflight::Want`,
`reconcile`). It does not rewrite dials every tick:

| Phase | Lateral | Vertical | Speed |
|---|---|---|---|
| Climb | HDG, or NAV/GPSS/LNAV on the route | FLC at Vy (or VS where FLC is absent) | power: climb setting |
| Cruise and enroute | NAV/GPSS/LNAV | ALT; VNAV where declared | A/T, or cruise power setting |
| Descent | as cruise | VS from the planned profile, set once per segment; FLC on a jet | A/T, or descent power |
| Approach | APR (LOC or GPS) | APR glideslope or glidepath | A/T, or approach power plus a slow trim on IAS |
| Pattern (VFR circuit) | HDG, bug set once per leg (`heading_bug::BugHold`) | ALT, then VS on base | power by leg |
| Flare, roll, rollout, taxi | layer 1b | layer 1b | layer 1b |

- **Speed without an A/T** (the C172, every piston): fly a power SETTING from the profile's
  book table (`power_settings:`, per phase and configuration). Correct it slowly on the IAS
  trend at the outer rate. This is how a pilot flies a C172 on the autopilot. The outer
  correction runs at 1 Hz with a 1 s delay against a speed that changes over tens of
  seconds, so it needs no per-airframe gain fit. It replaces `speedhold` as the inner speed
  law on these airframes.
- **What the AP cannot do is declared**, so guidance never asks for it: no FLC, no APR GS,
  no VNAV, no A/T. The fallbacks are written once, generically: VS for FLC, our path for a
  GS, power settings for an A/T.
- **The watchdogs stay**, as monitors on an aircraft the AP flies: `climb_watch`,
  `stable_approach`, `safe_hold`, `ApEngage` confirmation. They judge the AP's answer.
  They do not fly in its place.

### Layer 1b: the plugin closes the fast loops

The plugin already applies our commands every frame. `xcopilot/pitch`, `taxi_steer` and
`taxi_brake` are 1 s heartbeats that drive the yoke and brakes in its flight loops
(`plugin/src/drefs.cpp`), with the pilot-yield latches. What changes: the plugin closes the
loop itself, and Rust sends it TARGETS.

- **Interface** (new `xcopilot/law/*` datarefs, one writer each, heartbeat-expiring like the
  channels):
  - `law/mode`: off, flare, roll, rotate, rollout, taxi.
  - The targets: for the flare, a sink-versus-height schedule (start height, target sink at
    the wheels, an exponential tau); for the roll, a centreline (two points) and Vr; for the
    rollout and taxi, a path point and a speed.
  - Out: `law/status` (engaged, yielded, saturated) and `law/trace` (the law's inputs and
    outputs each frame, sampled into the blackbox by Rust).
- **The laws are written in Rust** and compiled into the plugin as a static library behind a
  small C interface (`crates/laws`, `staticlib`, no allocation, no I/O). The plugin passes
  the frame's state in and writes the outputs. The laws stay plain `cargo test` with the
  plants the tests already use. The app still never links the SDK, and the plugin build
  gains one `cargo build` step (a universal arm64 and x86_64 macOS binary, as the `.xpl` is
  today).
- **The laws read physical state at frame rate**: `local_vy` (vertical velocity, not the VSI
  needle), true pitch and pitch rate, radio altitude, groundspeed, track and position. No
  transport delay, and the sim's own `dt`.
- **Generic, normalised laws.** Elevator authority is scaled by dynamic pressure, and the
  pitch loop's inner gain comes from the `.acf` (mass, wing area, elevator area and travel;
  `aircraft::generic_from_acf` already reads the weights, the rest is new parsing). One law
  covers the fleet. At frame rate
  the margins are wide enough that this is plausible. At 5 Hz with 0.4 s of delay it is not.
- **Adaptation, later and only if needed**: estimate the elevator-to-pitch-rate response on
  the approach, while the AP flies, from its own servo outputs and the aircraft's answer.
  Then scale the flare law to it at the gate. This is automatic, per flight and logged,
  never a hand fit from sorties.

### What stays in Rust

Everything above layer 1b: mission, guidance, ATC, checklists, the watchdogs, the gates
(`landing_call`, `commit_flare`, `stable_approach`), the go-around door, the blackbox and the
scorer. The flare's GO/NO-GO stays in Rust (`landing_call` at the roundout gate). Only the
column inside the decided flare moves to the plugin.

## 3. Migration, in phases, each with a measurable test

The old loops are not patched further (the freeze). Each phase runs behind a setting
(`XCOPILOT_AUTONOMY=layers`, registry entry, default off) and only on the airframes the
phase names. Once its acceptance test passes, the path it replaces is deleted. No permanent
parallel flow.

**Phase 0: measure (no behaviour change).**
Log the real loop delay: for each `xcopilot/pitch` write, the sim time it was computed from
and the sim time it was applied, in `autoland` and `autotakeoff` traces. Probe the reference
C172's autopilot modes parked through `xvl conform`: FLC, APR, VNAV, which datarefs read
them and which commands drive them.
*Done when:* the delay is a blackbox number and the C172's mode table is a probed fact.

**Phase 1: the C172 approach on its own autopilot (Rust only).**
From the final approach fix to the roundout gate the GFC 700 flies APR (ILS, or GPS where
the procedure gives a glidepath). Speed is the book approach power plus the slow IAS trim.
Our guidance only selects modes, sets the course and altitude once, and watches
(`stable_approach`). At the gate, hand over to today's flare (unchanged in this phase).
*Done when:* 5 approaches at 2 fields, stable at the 500 ft gate on every one, with no
`vs_dial` rewrites on final in the blackbox.

**Phase 2: the C172 flare in the plugin.**
`crates/laws` flare law, the `law/*` interface, plugin integration. Rust decides at the
roundout gate (`landing_call`), then sets `law/mode=flare` with the schedule. The plugin
flies the column to the wheels. Rust watches `law/status` and keeps the go-around door.
*Done when:* 10 landings at 3 fields with touchdown sink under 300 fpm, no bounce, the
scorecard PASS unattended, and the C172's `flare_elevator` and `arrest_lead` blocks no
longer read. Then delete the hand-flown arrest for the C172.

**Phase 3: the ground laws.**
Takeoff roll centreline, rotation, and rollout steering and braking, through the same
interface: `RollSteer`, `RotatePitch` and `RolloutBrakeSteer` move to `crates/laws`.
*Done when:* the C172 and SR22 take off and roll out on the plugin laws with
`roll_rudder`, `rotate_elevator`, `ground_plant` and `rollout_steer` unread.

**Phase 4: the acceptance test for the whole idea.**
Fly an airframe that has never been fitted, with only a profile of capabilities and book
numbers plus a clean `xvl conform`. The Baron 58 or the RV-10 are candidates; both
profiles exist and neither has gain blocks.
*Done when:* 5 circuits, gate to gate, PASS unattended, without one gain added. If that
fails, the failure says which layer still knows the airframe, and that is the next fix,
made generically.

Then the other airframes, one at a time. Each airframe's gain blocks are deleted as it
crosses over. Jets with an A/T and LNAV/VNAV need the least of layer 1b (flare and
rollout). A Zibo that declares `autopilot_lands` needs none of it on landing.

## 4. Open questions (answer before or during phase 0)

1. **Which autopilot does the reference `.acf` carry?** The C172 profile's words describe the
   GFC 700 (HDG, ALT, NAV, VNV, APR, VS, FLC), but the sortie specs load `Cessna_172SP.acf`.
   **Answered from the `.acf` files (2026-10-11):** `Cessna_172SP.acf` carries the S-TEC
   (`instruments/autopilot_s-tec`): HDG, NAV, APR, ALT, VS, no FLC, no VNAV.
   `Cessna_172SP_G1000.acf` carries the G1000 with the GFC 700 (adds FLC and VNAV). Still to
   probe parked: which modes answer which commands, and whether the S-TEC couples a
   glideslope. **Decided (pilot, 2026-10-11): the G1000 variant is the reference.**
   **Probed parked on the G1000 (2026-10-11)**, battery and avionics on, engine off, flight
   director on first (`sim/autopilot/fdir_on`): `heading` gives `heading_status` 2,
   `level_change` gives `speed_status` 2 (FLC) with `altitude_hold_status` 1 (armed),
   `vertical_speed` gives `vvi_status` 2, `altitude_hold` gives `altitude_hold_status` 2,
   `approach` arms `glideslope_status` 1. `NAV` and `vnav` changed nothing parked: they
   need a loaded plan or a tuned source; probe them on a flight. With the flight director
   OFF no mode engages, which is why `xvl conform --only mcp` FAILs heading, VS and ALT on
   this airframe: conform does not turn the flight director on first (to fix).
2. **Does the stock AP take its targets over the Web API** (FLC speed bug, the APR course
   on a GPS approach), or only through commands (`radio::step_knob` style)? The probe
   answers this per mode.
3. **Plugin build:** a Rust `staticlib` linked into the `.xpl` with the current `make`
   (universal binary, `-fvisibility`, symbol clashes with the SDK). Prove with a stub
   `laws_version()` before any law moves.
4. **The pilot's hands:** the plugin laws keep the existing yield latches (`pitch_yield`,
   `brake_yield`, `steer_yield`) unchanged. A yield ends the law, and Rust hears it on
   `law/status`.
5. **Pause and replay:** the plugin law holds its state while `sim_frozen` and resets on
   un-pause. Rust treats a frozen sim as no measurement, as today.

## 5. What this does not change

ATC, traffic, the copilot's grammar and tools, checklists, the EFB, and the wire contracts
(`xcopilot/*`, `XCOPILOT_*`, `xcopilot.xpl`). The new datarefs live under the existing
`xcopilot/` namespace.
