# XVoiceLayer

**The voice layer for X-Plane 12: copilot, ATC and cabin crew. Entirely local.**

X-Plane simulates the aircraft and the world. It does not simulate the people.
This does. You speak a command, a **local** LLM picks the matching action,
XVoiceLayer writes the X-Plane dataref, and it reads the result back aloud.

```
🎙️  "flaps ten"  →  whisper (STT)  →  LLM picks a tool  →  write dataref  →  "Roger, flaps ten."
```

It's the **actor** in the cockpit: you fly and command, it works the panel and
confirms — hands stay on the controls, eyes stay outside. The whole loop is local
and real-time (ears, brain, and mouth all run on your machine; nothing is metered
per call).

Three voices share the one microphone: the **copilot** on the intercom, the
**tower** on the radio, and the **cabin crew** down the back. Named after
[XEarthLayer](https://github.com/samsoir/xearthlayer), which layers in what you
see; this layers in what you hear.

## What it can do

- **Single commands** across every 172 system — flaps, autopilot, engine, fuel,
  electrics, lights, radios, transponder, instruments.
- **Checklists the copilot runs** — say "run the before-takeoff checklist" and it
  reads each item aloud, **acts** the ones it can, **verifies** the ones it can
  read, and **calls out** the ones that are yours. The lists are **interpreted
  from XChecklist files** (the X-Plane community format), so any aircraft's
  checklist drops straight in — preflight to shutdown.
- **Compound commands** — "flaps ten and throttle seventy" runs both.
- **Conversational context** — "throttle fifty percent" then "seventy percent";
  "switch it back to both".
- **Safety gates + readbacks** — refuses unsafe actions (flap Vfe, low-altitude
  AP targets), confirms safety-critical ones, and reads back the *actual* state.
- **Background monitor** — calls out problems unprompted (alternator offline, low
  fuel).

➡️ **Full command + checklist reference:**
[Cessna 172SP](docs/cessna_172sp.md) · [Cirrus SR22](docs/cirrus_sr22.md)
· Monitor details: [docs/monitoring.md](docs/monitoring.md)

## Quick start

You need three local pieces running:

1. **X-Plane 12** with the Web API enabled (Settings → Network → Web API, port 8086).
2. **[Ollama](https://ollama.com)** with a tool-calling model:
   ```sh
   ollama serve
   ollama pull qwen3:8b        # the default; won the tool-pick battery
   ```
3. A **whisper.cpp** model for speech-to-text — a ggml `.bin` in `models/`
   (e.g. `ggml-base.en.bin`).

Then:

```sh
cp .env.example .env          # all defaults are sensible; tweak if needed
cargo run                     # plain stdout UI
XCOPILOT_UI=tui cargo run     # terminal dashboard (conversation, aircraft panel, mic)
```

The aircraft profile is auto-detected from X-Plane (`acf_ICAO`); the stock 172SP
is the built-in default.

### Speak or type

- **Continuous** (default): just speak commands.
- **Push-to-talk**: `XCOPILOT_LISTEN=ptt` — Enter to start/stop recording.
- **TUI text input**: in the dashboard, press **Enter** to open the command line,
  type a command, Enter to run it (Esc leaves, `q` quits). Great for testing — no
  mic, no STT mishears.

## Stack

| Stage | How |
|---|---|
| Mic capture | `cpal` |
| Speech-to-text | `whisper-rs` (whisper.cpp, Metal GPU on Apple Silicon) |
| LLM tool-picker | local **Ollama** (`/api/chat`), backend-neutral tool schema |
| X-Plane write | `reqwest` → X-Plane Web API (`PATCH /api/v1/datarefs/...`) |
| Text-to-speech | macOS `say` (default) or `piper` |

The tool surface is per-airframe: an aircraft profile declares which systems
exist and their limits, and the tool registry is built from it. See
[CLAUDE.md](CLAUDE.md) for the design rationale.

## Configuration

All via environment / `.env` — see [.env.example](.env.example) for the full,
documented list. Common knobs:

| Var | Default | What |
|---|---|---|
| `XCOPILOT_MODEL` | `qwen3:8b` | Ollama tool-calling model |
| `XCOPILOT_UI` | `plain` | `plain` or `tui` |
| `XCOPILOT_LISTEN` | `continuous` | `continuous` or `ptt` |
| `XCOPILOT_CONFIRM` | `true` | require "affirm" before safety-critical commands |
| `XCOPILOT_MONITOR` | `true` | background problem callouts |
| `XCOPILOT_END_SILENCE_MS` | `900` | trailing silence that ends an utterance |

## Tests

```sh
cargo test                                   # unit tests (no sim/LLM needed)
scripts/llm_bench.sh qwen3:8b                # compare models on the tool-pick battery (needs Ollama)
```

## Licensing

**GPL-3.0-or-later** ([LICENSE](LICENSE)). This is free software and will never
be commercial: copyleft is the point, so that the flight-sim ecosystem's payware
norm can't absorb this work without giving it back.

Third-party components keep their own (all permissive, all GPL-compatible): the
X-Plane SDK is MIT/BSD, Dear ImGui is MIT, ImgWindow is BSD-3-Clause, and the
B612 font is under the SIL Open Font License.
