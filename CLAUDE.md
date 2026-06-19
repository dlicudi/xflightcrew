# x-copilot

A voice-commanded copilot for X-Plane. The pilot speaks a command, an LLM picks
the matching action, the agent writes the X-Plane dataref, and the copilot reads
the action back aloud.

```
🎙️  "Flaps up"   →  STT  →  LLM picks a tool  →  write dataref  →  TTS "Flaps up!"
```

`x-copilot` is the **working name** (scaffolding). The intended product name is
**Roger** — it encodes the whole thesis: you speak, it acts, it confirms ("Roger,
flaps up"). Rename before going public.

## Relationship to Horizon

This is a **separate product** from Horizon (`~/GitHub/horizon`), not an extension —
and the distinction is the whole point:

- **Horizon = instructor.** One-way, grades you, and *deliberately never touches
  the controls*. That's what teaching is.
- **x-copilot = crew.** Two-way, *acts on the aircraft* by writing datarefs.
  Different trust model, failure modes, and UX. Merging them would muddy both.

Horizon is Python; x-copilot is Rust, so code isn't lifted directly — but two
patterns port over conceptually: its X-Plane Web API client (~30 lines of HTTP)
and its "shell out to the `piper` binary" TTS trick.

## Stack (decided)

Rust, because this is a latency- and audio-sensitive real-time loop — where Rust
earns its keep over a Python runtime + GIL.

| Stage | Crate / approach |
|---|---|
| Mic capture | `cpal` |
| STT | `whisper-rs` (whisper.cpp bindings) — push-to-talk for v1 |
| LLM tool-use | `reqwest` → Anthropic API, hand-rolled tool schema |
| X-Plane write | `reqwest` PATCH to the Web API (`PATCH /api/v1/datarefs/{id}/value`) |
| TTS readback | shell out to `piper`, or run Kokoro ONNX via the `ort` crate later |

**X-Plane access: REST first.** Start with the Web API over HTTP (separate
process, same API Horizon polls). A native X-Plane plugin via the C SDK (`xplm`
crate) is the lower-latency endgame — keep it in the back pocket, don't start there.
Note: read Laminar's X-Plane SDK license before distributing a plugin.

## Core design rule: curated tool surface, not raw dataref access

**Never let the LLM write arbitrary datarefs.** Expose a small, explicit set of
tools — `set_flaps(notch)`, `gear(up|down)`, `set_lights(...)`, `set_freq(...)` —
where each tool owns:

1. its dataref + valid range (so "flaps up" can't accidentally write throttle),
2. a **readback** that re-reads the dataref *after* writing and confirms the
   actual value — a real copilot confirms the *action*, not the *intent*,
3. **safety gates** (e.g. "gear up" while on the ground → refuse + explain why).

## v1 milestone

Smallest flyable slice, end to end: **push-to-talk → whisper → one `set_flaps`
tool → piper readback.** If that single loop feels good in the cockpit, the rest
is just more tools. Don't over-build the framework before that loop works.

## Licensing

Currently proprietary (no open-source LICENSE = all rights reserved). It's a
maybe-commercial product, so keep it closed until there's a reason to open it —
you can open later, never un-open. *If* it's ever opened: dual `MIT OR Apache-2.0`
per Rust convention (Apache's patent grant matters more than MIT for a product).
