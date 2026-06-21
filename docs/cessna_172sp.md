# Cessna 172SP — voice command reference

Everything you can say (or type) to x-copilot when flying the stock X-Plane
**Cessna 172SP Skyhawk**. The command surface is per-airframe — this is the set
the 172 profile exposes ([profiles/cessna_172sp.yaml](../profiles/cessna_172sp.yaml)).

How it works: you speak (or type in the TUI), a local LLM picks one curated
tool, x-copilot writes the dataref, then **reads back the actual result**
("Roger, flaps ten"). It confirms the *action*, not the intent.

- **Compound commands** work: "flaps ten and throttle seventy" runs both.
- **Follow-ups** work within ~45 s: after "throttle fifty percent", just say
  "seventy percent" to adjust it; "switch it back to both" resolves "it".
- **Safety-critical** commands (autopilot heading/altitude/VS) ask for a spoken
  "affirm" first (disable with `XCOPILOT_CONFIRM=false`).

---

## Checklists — the copilot runs them with you

Checklists are **interpreted from an XChecklist file**, not baked into the app —
[checklists/cessna_172sp.txt](../checklists/cessna_172sp.txt), the de-facto
X-Plane community format (the same files the XChecklist plug-in reads). The file
is read-only input; the copilot interprets it. Drop in any aircraft's `.txt`,
point the profile's `checklist:` at it, and it runs — so we're compatible with
the whole ecosystem.

Say a checklist's name and the copilot **runs it like a pilot-monitoring**, three
ways per item:

- **Acts** the systems it owns (each verified by the tool's readback) — "Parking
  brake, on."
- **Verifies** the ones it can read but doesn't actuate — reads the dataref and
  confirms or flags ("Static source, off. Confirmed.").
- **Calls out** the pilot's items (no dataref) — oil pressure, controls free,
  "flaps as required" — without pretending it did them.

Autonomous: no back-and-forth, it performs the whole list top to bottom.

```
🗣️  "run the before takeoff checklist"
🔊  Running the before takeoff checklist.
🔊  Parking brake, on.             ← acted
🔊  Mixture, full rich.            ← acted
🔊  Magneto test.                  ← narrated, then it runs the run-up
🔊  Strobe lights, on.             ← acted
🔊  Flaps (0 or 10), as required.  ← yours, called out
🔊  before takeoff checklist complete.
```

The full set, interpreted line-for-line from the file:

| Say | What the copilot does (it also calls out your items) |
|---|---|
| **"performance limits"** | speaks the V-speeds & key limits (reference, no actions) |
| **"preflight"** | the cabin power-up/test/power-down: brake, ignition off, the avionics/battery/pitot/flaps test toggles, trim takeoff |
| **"before starting engine"** | parking brake, autopilot off, avionics off, fuel both |
| **"starting engine"** | battery, beacon, throttle, mixture, carb heat, fuel-pump prime, **cranks the engine**, alternator + avionics on, flaps up |
| **"before taxi"** | transponder standby, taxi light on, parking brake set |
| **"before takeoff"** / "configure for takeoff" | brake, mixture rich, fuel both, **the mag run-up** (cycles ignition + throttle), strobe + landing on, autopilot off, trim takeoff |
| **"normal takeoff"** | taxi light off, transponder altitude, mixture rich |
| **"climb"** | landing light off, throttle full, mixture rich |
| **"cruise"** | mixture leaned |
| **"descent"** | mixture rich, fuel both |
| **"before landing"** / "configure for landing" | fuel both, mixture rich, landing + taxi lights on, autopilot off |
| **"normal landing"** | flaps up after rollout (the approach/touchdown/braking are yours) |
| **"after landing"** / "clean up" | flaps up, landing/strobe/nav lights off, transponder standby |
| **"engine shutdown"** / "shutdown" | brake, transponder/avionics off, idle, **engine off** (mixture cutoff), all lights off, alternator + master off |

A full flight by voice: *preflight → before starting engine → starting engine →
before taxi → before takeoff → normal takeoff → climb → cruise → descent → before
landing → normal landing → after landing → engine shutdown.*

> **Checklist vs single command.** A checklist *runs the whole phase*. A single
> command (next section) does one thing fast. Note the pair: **"starting engine"**
> runs the start *checklist*; **"start the engine"** just cranks. **"engine
> shutdown"** runs the shutdown *checklist*; **"shut down the engine"** just kills
> the engine.

---

## Single commands by system

### Flight controls
- **Flaps** — "flaps up / ten / twenty / full", "flaps down"
- **Trim** — "trim nose up / down", "set takeoff trim", "trim neutral"
- **Parking brake** — "parking brake on / off", "set the brakes"

### Autopilot
- **Heading** — "fly heading 270", "autopilot heading 090"
- **Altitude** — "climb and maintain 7000", "set altitude 3000"
- **Vertical speed** — "vertical speed 500 up", "descend at 700"
- **Level off** — "auto level", "wings level", "hold this altitude", "stop the climb"
- **Disconnect** — "autopilot off", "I have the airplane"

### Engine & fuel
- **Throttle** — "throttle 60 percent", "full power", "idle"
- **Mixture** — "mixture full rich", "lean", "mixture idle cutoff"
- **Start / stop** — "start the engine", "shut down the engine"
- **Carb heat** — "carb heat on / off" (hot / cold)
- **Fuel pump** — "fuel pump on / off"
- **Fuel selector** — "fuel selector both / left / right", "fuel valve both"
- **Magnetos / ignition** — "magnetos both / left / right / off", "ignition both / off"

### Electrical
- "battery on" · "master switch on / off" (battery + alternator) · "avionics on / off"
  · "alternator on / off" · "all electrical off"

### Lights
- **Exterior** — "landing / taxi / strobe / nav / beacon light on / off"
  (handles mishears like "floodlights"/"bloodlights")
- **Flood lights** — "flood lights on / off", "floods to 30 percent"

### Radios & transponder
- **COM** — "com one 118.5", "com two 121.9", "swap com one"
- **NAV** — "nav one 109.5", "set the OBS to 240"
- **Transponder** — "squawk 1200", "transponder standby / altitude / off", "ident"

### Instruments
- **Altimeter** — "set altimeter 29.92", "altimeter 30.10"
- **Pitot heat** — "pitot heat on / off"
- **Gyro** — "align the gyro", "sync the heading indicator"

### Approach
- **ILS** — "prepare the ILS for runway 28", "set up the ILS"

---

## Safety gates

x-copilot refuses (and explains) rather than do something unsafe:

- **Flap Vfe** — flaps to 10° need < 110 KIAS; 20°/full need < 85 KIAS. Too fast →
  refused ("103 knots is above the 85-knot limit for flaps full").
- **Autopilot altitude** — a target below **500 ft MSL** is refused (use "level
  off" to hold your current altitude). Catches a misheard/empty value.
- **Engine start** — won't crank a flat battery (needs ≥ 23 V); connect ground power.
- **Fixed gear** — the 172 has no gear tool; "gear up" does nothing (by design).

## 172SP limits (reference)

| | |
|---|---|
| Vne (never exceed) | 163 kt |
| Vno (max structural cruise) | 129 kt |
| Va (maneuvering) | 105 kt |
| Vfe 10° / 20°–full | 110 / 85 kt |
| Flap detents | up · 10° · 20° · full |
| Lights | nav · beacon · strobe · landing · taxi · floods |

## Background monitor

A separate watcher calls out problems unprompted (alternator offline, low fuel
endurance). See [monitoring.md](monitoring.md). Disable with `XCOPILOT_MONITOR=false`.
