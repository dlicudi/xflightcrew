# Cirrus SR22 — voice command reference

Everything you can say (or type) to x-copilot when flying the stock X-Plane
**Cirrus SR22** (Laminar). The command surface is per-airframe — this is the set
the SR22 profile exposes ([profiles/cirrus_sr22.yaml](../profiles/cirrus_sr22.yaml)).

How it works: you speak (or type in the TUI), a local LLM picks one curated
tool, x-copilot writes the dataref, then **reads back the actual result**
("Roger, flaps fifty"). It confirms the *action*, not the intent.

- **Compound commands** work: "flaps fifty and throttle seventy" runs both.
- **Follow-ups** work within ~45 s: after "throttle fifty percent", just say
  "seventy percent" to adjust it.
- **Safety-critical** commands (autopilot heading/altitude/VS) ask for a spoken
  "affirm" first (disable with `XCOPILOT_CONFIRM=false`).

> **Verified against the rustdecks deck.** The SR22 profile's datarefs and
> commands come from the GoofyJP checklist, the POH, and the tried-and-tested
> rustdecks SR22 deck. Wired and actuated: dual battery/alternator, throttle,
> mixture, engine start, **boost (fuel) pump**, **key-start ignition/magnetos**,
> **pitch trim**, and the off/left/right fuel selector. Still **call-out / verify
> only** (no tool): oxygen, cabin heat, alternate air, anti-ice. The copilot reads
> those back and flags them; it won't pretend it set them.

---

## Checklists — the copilot runs them with you

Checklists are **interpreted from an XChecklist file**, not baked into the app —
[checklists/cirrus_sr22.txt](../checklists/cirrus_sr22.txt) (the GoofyJP SR22
file), the de-facto X-Plane community format (the same files the XChecklist
plug-in reads). The file is read-only input; the copilot interprets it. Drop in
any aircraft's `.txt`, point the profile's `checklist:` at it, and it runs.

Say a checklist's name and the copilot **runs it like a pilot-monitoring**, three
ways per item:

- **Acts** the systems it owns (each verified by the tool's readback) — "Parking
  brake, on."
- **Verifies** the ones it can read but doesn't actuate — reads the dataref and
  confirms or flags ("Boost pump, on. Confirmed.").
- **Calls out** the pilot's items (no dataref) — oil pressure green, controls
  free, "fuel selector to fullest" — without pretending it did them.

**The checklists** (say any by name, normal flight order):

1. **preflight**
2. **before engine start**
3. **engine start**
4. **before taxiing**
5. **taxiing**
6. **holding point tests** — the run-up; comes in two parts (mag/alternator test,
   then **"holding point 2/2"** for the pitot-heat and flaps test)
7. **before takeoff**
8. **takeoff**
9. **after takeoff**
10. **climb**
11. **cruise**
12. **descent**
13. **before landing**
14. **landing**
15. **after landing**
16. **clear runway**
17. **shutdown**
18. **exit**
19. **balked landing** — the go-around drill (say it any time on approach)

Plus two reference read-backs that aren't runnable lists: **"initial"** (the file
header) and **"general data"** (limits & V-speeds).

What the copilot acts on vs. calls out, per list:

| Say | What the copilot does (it also verifies/calls out your items) |
|---|---|
| **"general data"** | speaks the limits & V-speeds (reference, no actions) |
| **"preflight"** | toggles the avionics master on then off for the fuel-quantity power-up test; batteries, doors, fuel quantity are verify/call-out |
| **"before engine start"** | parking brake set, flaps up, throttle idle, mixture lean (no confirm pre-start), fuel selector off; all-switches-off and doors are call-outs |
| **"engine start"** | strobe on, throttle, mixture rich, fuel selector left, boost pump, flaps, then **pauses to confirm the start** and cranks it (battery 1 + alternator 1); batteries/alternators, oil pressure are verify/call-out, then avionics on |
| **"before taxiing"** | nav light on, transponder standby, altimeter QNH, elevator trim T/O; HDG bug, flight director, autopilot preconfig, "fullest tank" are call-outs |
| **"taxiing"** | releases the parking brake (then sets it at the hold); brake test and HSI check are call-outs |
| **"holding point tests"** | mixture rich, boost pump, then **runs the mag check** (cycles ignition R/L/both) and the alternator test (battery/alt cycles called out); power settings called out |
| **"holding point 2/2"** | the pitot-heat test (pitot on→off) and the flaps test (full down→up) |
| **"before takeoff"** | transponder altitude, landing light on, pitot heat on, flaps 50%, elevator trim T/O; autopilot disconnect, doors, parking-brake release are verify/call-out |
| **"takeoff"** | mixture rich, boost pump, throttle full; "fullest tank", rotate & airborne are call-outs |
| **"after takeoff"** | flaps up, landing light off |
| **"climb"** | mixture lean for climb, boost pump; altimeter 29.92, oxygen, cabin heat, fuel balance are call-outs |
| **"cruise"** | mixture lean for cruise, boost pump off; oxygen, fuel balance are call-outs |
| **"descent"** | mixture rich, boost pump; oxygen off, altimeter QNH are call-outs |
| **"before landing"** | mixture rich, boost pump, landing light on, flaps 50%; fuel balance is a call-out |
| **"landing"** | mixture rich, landing light on, flaps full, parking-brake release; autopilot disengage and touchdown are call-outs |
| **"after landing"** | flaps up (then parking brake at the hold) |
| **"clear runway"** | transponder standby, landing light off, pitot heat off, mixture lean, boost pump off, elevator trim T/O; flight director is a call-out |
| **"shutdown"** | throttle idle, mixture cutoff, magnetos off, boost pump off, flaps up, fuel selector off, transponder/avionics off; alternators off, interior lights are verify/call-out |
| **"exit"** | doors & occupants — all call-outs |
| **"balked landing"** | throttle full, flaps 50% then up at speed; autopilot disengage called out |

A full flight by voice: *preflight → before engine start → engine start → before
taxiing → taxiing → holding point tests → before takeoff → takeoff → after
takeoff → climb → cruise → descent → before landing → landing → after landing →
clear runway → shutdown.*

> **Checklist vs single command.** A checklist *runs the whole phase*. A single
> command (next section) does one thing fast. Note the pair: **"engine start"**
> runs the start *checklist*; **"start the engine"** just cranks. **"shutdown"**
> runs the shutdown *checklist*; **"shut down the engine"** just kills the engine.

---

## Single commands by system

### Flight controls
- **Flaps** — "flaps up / fifty / full", "flaps down"
- **Parking brake** — "parking brake on / off", "set the brakes"
- **Pitch trim** — "trim nose up / down", "set takeoff trim", "trim neutral"
  (writes the universal elevator trim; takeoff = centred)

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
  (start brings up battery 1 + alternator 1 only — use the electrical commands for
  the second bus)
- **Fuel selector** — "fuel selector left / right / off", "fuel valve left"
  (low-wing tanks fed one at a time — there is **no "both"**)
- **Boost pump** — "boost pump on / off", "fuel pump on / off"
  (writes `fuel_pump_on[0]`, the dataref the rustdecks deck drives)
- **Magnetos / ignition** — "magnetos both / left / right / off"
  (key-start ignition; to start use "start the engine")
- *(Fuel-injected: no carb heat.)*

### Electrical (dual bus)
- **Batteries** — "battery one on", "battery two on", "both batteries on / off"
- **Alternators** — "alternator one on", "alternator two on", "both alternators on / off"
- **Avionics** — "avionics master on / off"
- **All** — "all electrical off"

### Lights
- **Exterior** — "nav / strobe / landing light on / off"
  *(SR22 profile wires nav, strobe, landing — no separate beacon/taxi/flood tools.)*

### Radios & transponder
- **COM** — "com one 118.5", "com two 121.9", "swap com one"
- **NAV** — "nav one 109.5", "set the OBS to 240"
- **Transponder** — "squawk 1200", "transponder standby / altitude / off", "ident"

### Instruments
- **Altimeter / baro** — "set altimeter 29.92", "altimeter 30.10"
- **Pitot heat** — "pitot heat on / off"

### Approach
- **ILS** — "prepare the ILS for runway 28", "set up the ILS"

---

## Safety gates

x-copilot refuses (and explains) rather than do something unsafe:

- **Flap Vfe** — flaps 50% need < 150 KIAS; full needs < 110 KIAS. Too fast →
  refused ("118 knots is above the 110-knot limit for flaps full").
- **Autopilot altitude** — a target below **500 ft MSL** is refused (use "level
  off" to hold your current altitude). Catches a misheard/empty value.
- **Engine start** — won't crank a flat battery; connect ground power.
- **Fixed gear** — the SR22 has no gear tool; "gear up" does nothing (by design).

## SR22 limits (reference)

| | |
|---|---|
| Vne (to 17,500 ft) | 205 kt |
| Vno (max structural cruise) | 176 kt |
| Vo (maneuvering, 3600 lb) | 140 kt |
| Vfe 50% / full | 150 / 110 kt |
| Stall — clean / landing | ~76 / ~61 kt |
| Flap detents | up · 50% · full |
| Lights | nav · strobe · landing |
| Fuel (usable / cruise burn) | ~92 gal / ~17 gal·hr⁻¹ |

## What's not wired yet

Items left as **verify / call-out** because there's no tool for them (or the
file's condition names no single target):

- **"Switch to the fullest tank"** — the fuel selector itself is wired (off/left/
  right, confirmed via `fuel_tank_selector`), but a "fullest tank" item names no
  specific position, so it stays a call-out
- **Dual battery/alternator in the checklist** — the tools exist (say "both
  batteries on"), but the file's battery/alt items use compound (`&&`) conditions
  the interpreter can't act on, so they're called out during a checklist
- **Oxygen, cabin heat, alternate air, anti-ice** — call-out only (no tool)

## Background monitor

A separate watcher calls out problems unprompted (alternator offline, low fuel
endurance). See [monitoring.md](monitoring.md). Disable with `XCOPILOT_MONITOR=false`.
