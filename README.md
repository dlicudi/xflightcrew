# XVoiceLayer

**The voice layer for X-Plane 12: copilot, ATC and cabin crew. Entirely local.**

You are one pilot in the cockpit. XVoiceLayer is everyone else: the **copilot**
in the other seat who works the panel and can fly the aircraft, the **tower** on
the radio, and the **cabin crew** down the back. Three voices, one microphone,
and nothing leaves your machine.

You speak normally and they answer in their own voices — no menus, no key
combinations, and no connection to anything.

```
🎙️  "flaps ten"  →  whisper (STT)  →  deterministic grammar  →  curated tool + readback  →  TTS
```

Named after [XEarthLayer](https://github.com/samsoir/xearthlayer), which layers
in what you *see*; this layers in what you *hear*.

## What it's for

**It has hands.** It doesn't just talk — it writes datarefs. It works the panel,
and it will taxi, line up, take off, fly a circuit or a published approach, and
land. That's a different trust model from a voice that only narrates, so every
action is gated and read back from the aircraft's *actual* state, never from the
intent.

**It's genuinely offline.** Ears (whisper), brain (Ollama), and mouth (ONNX/
piper) all run locally. Nothing is metered per call, and it works with the
network off.

**The tower is deterministic.** ATC is a state machine, not an LLM — a mishear
can earn "say again", but it can never earn a clearance. Frequencies, taxi
routes and approach geometry come from X-Plane's own `apt.dat`, `earth_nav.dat`
and CIFP data, so the field you're at is the field it works.

## What it does

### Talk to the copilot

**Conversation works without an LLM.** Ask for a reading, then “and now?” to
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
- **Compound and conversational** — "flaps ten and throttle seventy", then
  "make it eighty", "switch it back to both".
- **Checklists it actually runs** — "run the before-takeoff checklist": it reads
  each item, **acts** the ones it can, **verifies** the ones it can read, and
  **calls out** the ones that are yours. Interpreted from **XChecklist** files,
  so any aircraft's list drops straight in.
- **Safety gates and readbacks** — refuses flap extension over Vfe, gear up on
  the ground, low-altitude AP targets; re-reads the dataref after writing and
  confirms what the aircraft *did*.
- **A background monitor** that speaks up unprompted: alternator offline, low
  fuel, gear, stall, terrain, altitude deviations.

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

### Talk to the tower

Real frequencies from `apt.dat`, region-accurate phraseology (ICAO/CAP 413 vs
FAA), and a controller that sequences you rather than reading a script:

- Clearance, taxi with **named routes** ("taxi to holding point Alpha one via
  Bravo, Link two seven"), line-up-and-wait, takeoff, circuits, vectors, ILS
  approaches, landing clearance, taxi in.
- **Readback enforcement** on the runway-critical items, a runway-in-use that
  can't drift mid-departure, **wake turbulence** separation, and departures
  threaded into gaps in the arrival stream.
- **ATIS** on its own frequency, and an **enroute region** that hands you down
  the airway from field to field.
- A **progressive taxi guide** in the copilot's voice: "next left, onto Sierra
  five Whiskey", with runway crossings called.

### Talk to the cabin

On airframes that carry crew: "cabin crew, prepare for landing" gets an
acknowledgement, the interphone chime, and a "cabin secure" report when they
mean it. Sterile-cockpit rules apply.

## In the sim

A small companion plugin ([plugin/](plugin/)) adds what the Web API can't do: a
dedicated **push-to-talk** (separate keys for the intercom and the radio), the
FMS route load, terrain queries, and the control channels the ground automation
steers through.

It also draws an in-sim **status panel** — Flight, Transcript, ATC, Route and
Settings tabs — with the live strip, an automation status well, one-press
buttons for every automation, and the phraseology reference.

## Quick start

Three local pieces:

1. **X-Plane 12** with the Web API on (Settings → Network → Web API, port 8086).
2. **[Ollama](https://ollama.com)** with a tool-calling model:
   ```sh
   ollama serve
   ollama pull qwen3:8b
   ```
3. A **whisper.cpp** model — a ggml `.bin` in `models/`, e.g. `ggml-base.en.bin`.

Then:

```sh
cp .env.example .env
cargo run
```

Optional but recommended, for push-to-talk and the panel:

```sh
make -C plugin install
```

The aircraft profile is auto-detected from `acf_ICAO`. Profiles ship for the
Cessna 172SP, Cirrus SR22, Cirrus Vision SF50, Citation X, King Air C90B,
Lancair Evolution, MD-82, Twin Otter and DC-3, with a built-in fallback for
anything else.

### Speaking to it

- **Continuous** (default): just talk.
- **Push-to-talk**: `XCOPILOT_LISTEN=ptt`, or bind the plugin's PTT commands in
  X-Plane. The **radio** PTT transmits to ATC; the **intercom** PTT talks to the
  copilot.
- **Type instead**: `XCOPILOT_UI=tui` gives a dashboard with a command line —
  no mic, no mishears, ideal for testing.

### Running only half of it

`XCOPILOT_MODE` (or the panel's Settings tab, live):

- `both` — copilot and ATC (default)
- `copilot` — no simulated ATC, so real/online ATC owns the radio
- `atc` — the world without the crew; needs no Ollama at all

## Stack

| Stage | How |
|---|---|
| Mic capture | `cpal` |
| Speech-to-text | `whisper-rs` (whisper.cpp, Metal on Apple Silicon) |
| Commands and conversation | deterministic grammar, curated tools, conversation state |
| Optional answer fallback | local **Ollama**, answer-only; off by default, unnecessary for the conversation features above |
| X-Plane read/write | `reqwest` → Web API (`PATCH /api/v1/datarefs/...`) |
| In-sim capability | native C++ plugin (PTT, panel, FMS, terrain, controls) |
| Text-to-speech | in-process ONNX (default), or `piper` / macOS `say` |

Rust, because this is a latency- and audio-sensitive real-time loop. The
workspace splits the world from the crew: `crates/atc` is the tower and can
only *read* the sim, `crates/sim` is the shared Web API client, and the copilot
embeds both. That boundary is compiler-checked, not a convention.

The tool surface is per-airframe: a profile declares which systems exist and
their limits, and the tool registry is built from it — so a 172 is never
offered a gear lever. See [CLAUDE.md](CLAUDE.md) for the design rationale.

## Configuration

All via environment / `.env` — [.env.example](.env.example) is the documented
list. Common knobs:

| Var | Default | What |
|---|---|---|
| `XCOPILOT_MODE` | `both` | `both`, `copilot`, `atc` |
| `XCOPILOT_MODEL` | `qwen3:8b` | Ollama tool-calling model |
| `XCOPILOT_UI` | `plain` | `plain` or `tui` |
| `XCOPILOT_LISTEN` | `continuous` | `continuous` or `ptt` |
| `XCOPILOT_CONFIRM` | `true` | require "affirm" before safety-critical commands |
| `XCOPILOT_MONITOR` | `true` | background problem callouts |
| `XCOPILOT_ATC_AIRPORT` | `LXGB` | home field for the tower |
| `XCOPILOT_SIMBRIEF_ID` | — | your SimBrief username, for flight plans |

The `xcopilot/*` datarefs, the `XCOPILOT_*` variables and the `xcopilot.xpl`
plugin keep the project's old working name on purpose: they are wire contracts
between the app, the plugin and your own config, and renaming them would break
live installs for nothing.

## Reference

- Commands and checklists: [Cessna 172SP](docs/cessna_172sp.md) ·
  [Cirrus SR22](docs/cirrus_sr22.md)
- Background callouts: [docs/monitoring.md](docs/monitoring.md)
- In-sim ATC soak test: [docs/atc-chaos-card.md](docs/atc-chaos-card.md)

## Tests

```sh
cargo test --workspace                        # everything deterministic, no sim or LLM needed
cargo test --release -p atc -- --ignored atc_world_sweep   # cross-airport sweep (needs apt.dat)
cargo run -- stt-eval bench/atc_stt_cases.yaml             # spoken-call accuracy
scripts/llm_bench.sh qwen3:8b                             # tool-pick battery (needs Ollama)
```

Every flight writes `logs/blackbox.jsonl`; `xvl analyze` mines the logs for
phrasings the deterministic grammar missed.

## Licensing

**GPL-3.0-or-later** ([LICENSE](LICENSE)). Free software, never commercial, and
X-Plane only. Copyleft is the point: the flight-sim ecosystem's payware norm
shouldn't be able to absorb this work without giving it back.

Third-party components keep their own, all permissive and all GPL-compatible:
the X-Plane SDK is MIT/BSD, Dear ImGui is MIT, ImgWindow is BSD-3-Clause, and
the B612 font is under the SIL Open Font License.
