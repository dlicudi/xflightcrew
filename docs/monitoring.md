# Monitoring & Alerts

Proactive callouts: the copilot speaks up about problems a human second pilot
would catch. This is the **crew** half of the thesis — a copilot that watches you
bust an altitude and says nothing isn't a copilot.

## Goals / non-goals

- **Goal:** present-tense, advisory, sparse, acknowledgeable callouts that add
  *judgment* over the panel's own alarms.
- **Non-goals:**
  - echoing the aircraft's built-in alarms (stall horn, gear horn, master caution);
  - grading technique after the fact — that's Horizon, deliberately a separate
    product (see `CLAUDE.md`). Monitoring stays operational and in the moment;
  - touching the controls. Alerting is voice-only: it tells, it never seizes. It
    is the **proactive sibling of the tool safety gates** (e.g. "gear up on the
    ground → refuse") — the same safety logic, running continuously and speaking
    instead of refusing.

## Design discipline (what keeps it from being a nag)

- **Add judgment, don't echo sensors.** The unique value is config-vs-phase
  cross-checks no single alarm catches (flaps up on final, fuel selector on a
  near-dry tank), not raw redline buzzers.
- **Stay advisory.** Voice-only; lower risk than the acting tools already shipped.
- **Sparse, prioritized, phase-aware.** One speech queue; warnings jump ahead;
  edge-triggered (fire on entering the bad state, not every tick); hysteresis on
  clear; suppressed during known maneuvers; gated by phase.
- **Acknowledgeable.** "Roger / I see it" silences a repeat until it clears and
  re-arms. Endless repetition = a muted copilot.

## Decisions (locked)

- **Default posture:** on by default, **conservative** — only high-confidence
  config/phase checks and Warning-class alerts fire initially.
- **Preemption (v1):** queue-jump only. A higher-priority utterance speaks the
  instant the current one ends; no mid-utterance cut-off (`say`/piper aren't
  cheaply interruptible). True cut-off is a later iteration.

## The split (consistent with the project rule)

Rule *logic* lives in Rust (`src/monitor/`); per-airframe *numbers and enables*
live in the profile YAML — same division as tools owning datarefs while YAML
carries flap detents / Vfe. No general YAML rule-DSL (over-engineering).

## Architecture — three pieces

### 1. Speech arbiter — `src/voice.rs`  ✅ DONE (Milestone 1)

The single owner of TTS. Both the command loop (readbacks) and the monitor
(alerts) run on different threads; without arbitration they'd talk over each
other. All speech goes through one `Voice`: a dedicated speaker thread draining a
priority queue, so utterances are serialized.

- `Priority`: `Advisory < Readback < Caution < Warning` (max-heap pops highest;
  FIFO within a priority via a monotonic seq).
- `Voice::say(priority, text)` — **blocking**; the command loop must not resume
  listening until the readback finishes (else it captures its own TTS).
- `Voice::alert(priority, text)` — **fire-and-forget**; the monitor queues and
  moves on.

### 2. Monitor loop — `src/monitor/` (Milestone 2+)

A dedicated task at ~3 Hz (configurable; faster than the 1 Hz UI poll because
near-ground sink rate matters). Each tick:

1. snapshot ~15 datarefs into a `Snapshot`;
2. estimate `Phase` (Ground / Takeoff / Climb / Cruise / Approach / Landing)
   from on-ground, groundspeed, IAS, AGL, VSI;
3. run each enabled `Check`, producing zero or more alerts → `voice.alert(...)`.

```rust
enum Severity { Advisory, Caution, Warning }   // maps to voice::Priority
enum Phase { Ground, Takeoff, Climb, Cruise, Approach, Landing }

trait Check {
    fn id(&self) -> &'static str;
    fn phases(&self) -> &[Phase];                  // when it's armed
    fn eval(&self, s: &Snapshot, p: Phase) -> Option<Severity>;
    fn say(&self, s: &Snapshot) -> String;
}
```

Per-alert runtime state (in the monitor) enforces the discipline: edge-trigger,
hysteresis (separate trigger/clear bands), repeat policy (Warning re-announces
every N s until ack/clear; Caution once or twice; Advisory once), dedupe (one
active instance per id), phase gate.

### 3. Acknowledgement (Milestone 4)

When an alert is active, a tiny ack grammar ("roger" / "I see it" / "got it") is
accepted by the listen loop (bypassing the command-keyword gate) and silences
that alert id until it clears and re-arms.

## Profile schema (YAML additions)

```yaml
limits:        # numbers the checks read
  vfe_kts: 85
  vno_kts: 129
  vne_kts: 163
  fuel_low_gal: 8
  fuel_imbalance_gal: 5
alerts:        # enable + per-alert tuning; absent = off
  gear_up_on_approach:    { enabled: false }   # fixed-gear 172 → off
  fuel_selector_low_tank: { enabled: true }
  flaps_up_on_approach:   { enabled: true, severity: caution }
  overspeed_vne:          { enabled: true }
  low_fuel:               { enabled: true }
  alternator_offline:     { enabled: true }
  gyro_drift:             { enabled: true, deg: 10 }
  high_sink_near_ground:  { enabled: true, fpm: -1000 }
```

## First C172 alert set (config/phase sweet spot — lowest false-positive)

Discrete-state checks ship first; airspeed/energy trend math comes after the
plumbing is proven.

- fuel selector on a near-empty tank (selector position vs per-tank quantity)
- flaps up turning final (Approach phase + flaps ≈ 0)
- carb heat cold on descent (172-specific, icing-prone)
- parking brake set as throttle advances on the roll
- alternator offline / low bus volts with the engine running
- directional gyro precessed past ~10° vs the compass, airborne (advisory; pairs
  with the `align_gyro` tool to fix it)
- fuel endurance (minutes at current burn) into a reserve band — caution at the
  profile's `reserve_caution_min`, warning at `reserve_warning_min` (172: 45/30),
  airborne + engine-running only
- pre-takeoff fuel: on the ground + engine running, estimate endurance from the
  profile's nominal `cruise_burn_kg_hr` (idle burn would read as ~infinite) and
  caution once below the reserve — "low fuel for departure"
- overspeed approaching Vno → Vne (trend, not just redline)
- high sink rate near ground

## Integration points

- `src/voice.rs` (done), `src/monitor/` (new).
- `main.rs` poll thread (~line 115) also spawns the monitor task, sharing an
  `XPlaneClient` and a `Voice` handle.
- `aircraft.rs`: add `limits` + `alerts` to `AircraftProfile`; populate the 172
  YAML + Rust fallback.
- Config: `monitor_enabled` (default on), `monitor_hz`, `alert_verbosity`.

## Milestones

1. **Speech arbiter** — single TTS owner; `say()` routed through it. ✅ DONE.
2. **Monitor loop + phase estimator + one check** (alternator offline — easy to
   force, safe to test).
3. **Config/phase mismatch set** (fuel selector, flaps, carb heat, parking brake).
4. **Acknowledgement grammar.**
5. **Trend limits** (Vne, sink rate) once the rest is solid.
