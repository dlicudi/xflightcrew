# XFlightCrew

**Your flight crew for X-Plane 12: copilot, ATC and cabin crew. Entirely local.**

You are one pilot in the cockpit. XFlightCrew is everyone else: the **copilot**
in the other seat who works the panel and can fly the aircraft, the **tower** on
the radio, and the **cabin crew** down the back. Three voices, one microphone,
and nothing leaves your machine.

You speak normally and they answer in their own voices: no menus, no key
combinations, and no connection to anything.

```
🎙️  "flaps ten"  →  whisper (STT)  →  deterministic grammar  →  curated tool + readback  →  TTS
```

[xflightcrew.com](https://xflightcrew.com)

Formerly XVoiceLayer. Until the rename is complete, the binary is still `xvl`
and the repository `xvoicelayer`.

## What it's for

**It has hands.** It doesn't just talk; it writes datarefs. It works the panel,
and it will taxi, line up, take off, fly a circuit or a published approach, and
land. That's a different trust model from a voice that only narrates, so every
action is gated and read back from the aircraft's *actual* state, never from the
intent.

**It's genuinely offline.** Ears (whisper), brain (a deterministic grammar and
rule set, no model), and mouth (ONNX/piper) all run locally. Nothing is metered
per call, and it works with the network off.

**The tower is deterministic.** ATC is a state machine, not an LLM. A mishear
can earn "say again", but it can never earn a clearance. Frequencies, taxi
routes and approach geometry come from X-Plane's own `apt.dat`, `earth_nav.dat`
and CIFP data, so the field you're at is the field it works.

## What it does

### Talk to the copilot

**Conversation is rules, not a model.** Ask for a reading, then “and now?” to
refresh it or “and heading?” to change the subject. “Why?” recalls the source
of the last supported answer or the actual report from a command, including
refusals. “What's our speed?” asks which speed you mean; reply “airspeed”,
“groundspeed”, or “vertical speed”. “What are you managing?” reads the current
automation status. “Keep it brief” adjusts explanations for this session.
These follow-ups never select a control action. See the
[supported conversation examples and boundaries](docs/deterministic-copilot.md).

You can also combine readings (“could you tell me our altitude and heading?”),
return to an earlier topic (“back to altitude”), or ask “what was my last
request?” For a checklist walkthrough, say “walk me through the before takeoff
checklist”, then “next item”, “previous item”, or “pause the preview”. “Resume
the preview” returns to the saved item after another question. These are previews
of the aircraft's loaded checklist; they do not perform or verify its items.

- **Commands** across every system the airframe has: flaps, gear, autopilot,
  engine, fuel, electrics, lights, radios, transponder, instruments.
- **Compound and conversational**: "flaps ten and throttle seventy", then
  "make it eighty", "switch it back to both".
- **Checklists it actually runs**: "run the before-takeoff checklist": it reads
  each item, **acts** the ones it can, **verifies** the ones it can read, and
  **calls out** the ones that are yours. Interpreted from **XChecklist** files,
  so any aircraft's list drops straight in.
- **Safety gates and readbacks**: refuses flap extension over Vfe, gear up on
  the ground, low-altitude AP targets; re-reads the dataref after writing and
  confirms what the aircraft *did*.
- **A background monitor** that speaks up unprompted: alternator offline, low
  fuel, gear, stall, terrain, altitude deviations.
- **Flight planning from a destination**: "plan a flight to Malaga", "divert
  to Gibraltar", "take us to lima echo mike golf". A direct route from the field
  you're at, cruise altitude picked over the sim's own terrain, time and fuel
  from the aircraft profile against what's in the tanks, and a refusal when the
  runway there is too short. It briefs the plan first; "load it" puts it in the
  GPS, and "fly my flight plan" can fly either this local plan or a SimBrief
  route. The local planner needs no account or network.

### Let it fly

Say it and the copilot takes that phase. Each hands back on "my controls", and
each refuses rather than guesses.

| Say | It does |
|---|---|
| "push back left" | Calls a tug and steers the push onto the lane |
| "taxi for me" | Drives the cleared route to the holding point |
| "line us up" | Rides the painted lead-on onto the centreline |
| "take off for me" | Rolls, tracks the centreline, rotates, climbs to 1,000 ft |
| "do a circuit" | The whole pattern, terrain- and airspace-aware, to a landing |
| "fly the ILS" | Follows the published approach from real chart data |
| "land us" | Autoland from wherever you are |
| "full auto" | All of it, cold and dark to the runway |
| "fly my flight plan" | Your SimBrief route, gate to gate |

**How it lands.** The way a pilot does. Above minimums the aircraft's own
autopilot flies: its approach mode on an ILS, otherwise a heading set with the
wind correction and changed only now and then, in whole degrees. At minimums the
copilot disconnects and hand-flies to the wheels with one pilot controller
(`src/pilot`): the ailerons for the centreline, the elevator for the path and
the flare, the throttle for the energy. The same controller flies every
fixed-wing type; what differs between types (how hard the aircraft answers each
control) is measured in flight, not typed in. The laws are tested on models
fitted to recorded flights of each aircraft.

### Talk to the tower

Real frequencies from `apt.dat`, region-accurate phraseology (ICAO/CAP 413 vs
FAA), and a controller that sequences you rather than reading a script:

- Separate **Delivery, Ground, Tower and Approach/radar positions** where the
  airport publishes them, with calls routed to the position on COM1 and
  frequency-specific handoffs.
- IFR clearance, pushback, taxi with **named routes** ("taxi to holding point
  Alpha one via Bravo, Link two seven"), line-up-and-wait, takeoff, circuits,
  vectors, instrument approaches, landing clearance and taxi in.
- **Readback enforcement** on the runway-critical items, a runway-in-use that
  can't drift mid-departure, **wake turbulence** separation, and departures
  threaded into gaps in the arrival stream.
- **ATIS** on its own frequency, and an **enroute region** that hands you down
  the airway from field to field.
- A **progressive taxi guide** in the copilot's voice: "next left, onto Sierra
  five Whiskey", with runway crossings called.

### Fly alongside local traffic

XFlightCrew can generate and fly its own deterministic AI traffic instead of
depending on LiveTraffic. The same tower that controls you controls every
traffic flight through a separate per-flight strip:

- GA circuits and IFR arrivals and departures, with field-appropriate airline
  and aircraft mixes, parked-aircraft turnarounds, generated schedules, or a
  user-supplied YAML schedule.
- Taxi, line-up, takeoff, climb-out, instrument arrival, missed approach,
  landing, runway vacation and taxi-in phase machines.
- Runway occupancy, arrival sequencing, departure gaps, wake-turbulence
  spacing, speed control, holding/orbits and go-arounds. Your auto-taxi also
  stops behind or yields to traffic on the ground.
- Real radio calls and readbacks routed through Delivery, Ground, Tower and
  Approach. You hear them only when your COM1 is tuned to that frequency.
- Native rendering through XPMP2 and user-installed CSL models, with animated
  gear, flaps, lights and engines plus optional TCAS publication.

Set `XCOPILOT_TRAFFIC=ours` to use this fleet, `real` to retain read-only
narration of X-Plane/LiveTraffic targets, or `off` (the current default). The
generated fleet is reproducible from its seed and freezes during pause, replay,
or accelerated simulation time above roughly 1x so its simulation clock cannot
diverge from ATC's timers.

### Talk to the cabin

On airframes that carry crew: "cabin crew, prepare for landing" gets an
acknowledgement, the interphone chime, and a "cabin secure" report when they
mean it. Sterile-cockpit rules apply.

## In the sim

A small companion plugin ([plugin/](plugin/)) adds what the Web API can't do: a
dedicated **push-to-talk** (separate keys for the intercom and the radio), the
FMS route load, terrain queries, and the control channels the ground automation
steers through. It also renders XFlightCrew's traffic via XPMP2 when CSL models
are installed.

It also draws an in-sim **status panel** (Flight, Transcript, ATC, Route and
Settings tabs) with the live strip, an automation status well, one-press
buttons for every automation, and the phraseology reference.

## On a tablet: the EFB

An Electronic Flight Bag served on your LAN for an iPad, a phone or a second
screen: Flight, Map, Plan and Radio tabs laid out by the phase of flight, the
crew's warnings, a hold-to-talk button, and buttons that say pilot English for
you (so every tap goes through the same grammar, gates and readbacks as your
voice). Off by default: `XCOPILOT_EFB=true`, or turn it on in Settings; the
link with its key is in the log. See [docs/efb.md](docs/efb.md).

## Quick start

Two local pieces:

1. **X-Plane 12** with the Web API on (Settings → Network → Web API, port 8086).
2. A **whisper.cpp** model: a ggml `.bin` in `models/`, e.g. `ggml-base.en.bin`.

Then:

```sh
cp .env.example .env
cargo run
```

Optional but recommended, for push-to-talk and the panel:

```sh
make -C plugin install
```

The aircraft profile is auto-detected from both `acf_ICAO` and the `.acf` path
(the stock 737 and Zibo share an ICAO code). Profiles currently ship for the
Airbus A330-300, Boeing 737-800, Zibo 738, Boeing 747-400, MD-82, Citation X,
Cirrus Vision SF50, Cessna 172SP, Cirrus SR22, Beechcraft Baron 58, King Air
C90B, Twin Otter, DC-3, Lancair Evolution, Van's RV-10 and Sikorsky S-76C.
An unmatched aircraft gets a generic profile derived from its `.acf` rather
than silently inheriting Cessna limits.

### Speaking to it

- **Continuous** (default): just talk.
- **Push-to-talk**: `XCOPILOT_LISTEN=ptt`, or bind the plugin's PTT commands in
  X-Plane. The **radio** PTT transmits to ATC; the **intercom** PTT talks to the
  copilot.
- **Type instead**: `XCOPILOT_UI=tui` gives a dashboard with a command line:
  no mic, no mishears, ideal for testing.

### Modes

`XCOPILOT_MODE` (or the panel's Settings tab, live):

- `both`: copilot and ATC (default)
- `copilot`: no simulated ATC, so real/online ATC owns the radio
- `atc`: the world without the crew
- `instructor`: a flight instructor that teaches one task at a time and grades
  it, hands off the controls ("show me" demonstrates once, then "your controls")

## Stack

| Stage | How |
|---|---|
| Mic capture | `cpal` |
| Speech-to-text | `whisper-rs` (whisper.cpp, Metal on Apple Silicon) |
| Commands and conversation | deterministic grammar, curated tools, conversation state |
| Flying | the aircraft's autopilot, plus one pilot controller (`src/pilot`) for what a pilot hand-flies |
| X-Plane read/write | `reqwest` → Web API (`PATCH /api/v1/datarefs/...`) |
| In-sim capability | native C++ plugin (PTT, panel, FMS, terrain, controls) |
| Navigation data | `nav` crate (`apt.dat`, fixes, navaids, CIFP, taxi geometry) |
| ATC world | `atc` crate (stations, positions, strips, sequencing; read-only sim view) |
| AI traffic | `traffic` crate (schedule, fleet, kinematics, phase machines) |
| Text-to-speech | in-process ONNX (default), or `piper` / macOS `say` |

Rust, because this is a latency- and audio-sensitive real-time loop. The
workspace splits the world from the crew: `crates/nav` owns navigation data and
geometry, `crates/atc` owns the read-only controller world, `crates/traffic`
owns the deterministic fleet, and `crates/sim` owns shared simulator plumbing.
ATC and traffic do not depend on each other; the application joins them, so the
controller cannot actuate either your aircraft or traffic. These boundaries are
compiler-checked rather than conventions.

The tool surface is per-airframe: a profile declares which systems exist and
their limits, and the tool registry is built from it, so a 172 is never
offered a gear lever. See [CLAUDE.md](CLAUDE.md) for the design rationale.

## Configuration

All via environment / `.env`; [.env.example](.env.example) is the documented
list. Common knobs:

| Var | Default | What |
|---|---|---|
| `XCOPILOT_MODE` | `both` | `both`, `copilot`, `atc` |
| `XCOPILOT_UI` | `plain` | `plain` or `tui` |
| `XCOPILOT_LISTEN` | `continuous` | `continuous`, `ptt`, or `off` (text only: no microphone, no whisper model) |
| `XCOPILOT_TTS` | `onnx` | `onnx`, `say`, `piper`, or `off` (text only: nothing synthesised, lines paced as if spoken) |
| `XCOPILOT_CONFIRM` | `true` | require "affirm" before safety-critical commands |
| `XCOPILOT_MONITOR` | `true` | background problem callouts |
| `XCOPILOT_ATC_AIRPORT` | `LXGB` | home field for the tower |
| `XCOPILOT_SIMBRIEF_ID` | (none) | your SimBrief username, for flight plans |
| `XCOPILOT_TRAFFIC` | `off` | `ours`, `real`, or `off` |
| `XCOPILOT_TRAFFIC_DENSITY` | `normal` | generated fleet: `off`, `light`, or `normal` |
| `XCOPILOT_TRAFFIC_FILE` | (none) | optional local YAML schedule instead of generation |
| `XCOPILOT_EFB` | `false` | serve the EFB page on the LAN |

For rendered traffic, install CSL packages and point the X-Plane plugin at
them with `XCOPILOT_TRAFFIC_CSL`. LiveTraffic commonly owns X-Plane's TCAS
slots first; disable its AI-slot request (or LiveTraffic itself) if you want
XFlightCrew traffic to appear on TCAS. Aircraft still render without TCAS
ownership.

The `xcopilot/*` datarefs, the `XCOPILOT_*` variables and the `xcopilot.xpl`
plugin keep the project's old working name on purpose: they are wire contracts
between the app, the plugin and your own config, and renaming them would break
live installs for nothing.

## Reference

- Commands and checklists: [Cessna 172SP](docs/cessna_172sp.md) ·
  [Cirrus SR22](docs/cirrus_sr22.md)
- The EFB: [docs/efb.md](docs/efb.md)
- Checking a new aircraft profile, parked: [docs/conform.md](docs/conform.md)
- Background callouts: [docs/monitoring.md](docs/monitoring.md)
- In-sim ATC soak test: [docs/atc-chaos-card.md](docs/atc-chaos-card.md)

## Tests

```sh
cargo test --workspace                        # everything deterministic, no sim needed
cargo test --release -p atc -- --ignored atc_world_sweep   # cross-airport sweep (needs apt.dat)
cargo run -- stt-eval bench/atc_stt_cases.yaml             # spoken-call accuracy
```

With the sim running:

```sh
xvl sortie bench/sorties/c172-g1000-ils13.yaml   # fly a scripted test flight, scored PASS/FAIL
xvl debrief                                      # the last flight's lessons and grades
xvl conform                                      # check every channel an aircraft profile declares, parked
```

Every flight writes `logs/blackbox.jsonl`; `xvl analyze` mines the logs for
phrasings the deterministic grammar missed.

The ordinary workspace suite is deterministic and runs without X-Plane. It
covers grammar utterances, safety gates and readbacks, navigation geometry,
ATC state machines, traffic sorties, packed plugin wire formats, flight-law
simulations and regressions reconstructed from real blackbox sorties. Ignored
tests are the explicit boundary: they need an X-Plane installation, its nav
data, or a running simulator.

## Current scope and limitations

- X-Plane 12 is the only simulator target. Most stock-dataref functions work
  through its Web API; PTT, panel, FMS injection, terrain/control channels and
  rendered traffic need the native plugin.
- The Cessna 172 is the reference airframe. Other profiles expose their
  declared systems and limits, but do not all have equal live-sortie coverage.
- The autonomous flying stack is being rebuilt in layers: the aircraft's own
  autopilot where a pilot would use it, one pilot controller for what a pilot
  hand-flies. The final approach and landing are done (validated on the C172,
  SR22, King Air, Lancair Evolution, Baron 58 and SF50); the jets fly the same
  controller but are not yet validated, and the other flying modes (takeoff,
  circuits, taxi, full auto) take only crash and safety fixes until their turn.
- Traffic does not fetch online schedules and requires user-installed CSL
  models for useful rendering. It currently freezes under accelerated time,
  has no wind triangle, and operates at controlled fields when ATC is enabled.
- Deterministic language is intentional: unsupported phrasing is refused or
  logged for a new rule. There is no runtime language-model fallback.

## Licensing

**GPL-3.0-or-later** ([LICENSE](LICENSE)). Free software, never commercial, and
X-Plane only. Contributions and derivative works are governed by the license.

Third-party components keep their own, all permissive and all GPL-compatible:
the X-Plane SDK is MIT/BSD, Dear ImGui is MIT, ImgWindow is BSD-3-Clause,
XPMP2 (B. Hoppe, `plugin/third_party/XPMP2/LICENSE`) is MIT, and the B612
font is under the SIL Open Font License. XPMP2 is built without FMOD; CSL
aircraft models are never bundled (the pilot installs them).
