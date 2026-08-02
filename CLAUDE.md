# XVoiceLayer

The voice layer for X-Plane 12: copilot, ATC and cabin crew, entirely local. The
pilot speaks a command, an LLM picks the matching action, the agent writes the
X-Plane dataref, and the copilot reads the action back aloud.

```
🎙️  "Flaps up"   →  STT  →  LLM picks a tool  →  write dataref  →  TTS "Flaps up!"
```

**Named 2026-07-30** (the working title `XVoiceLayer` is retired, and so is the
long-parked naming question). The name follows
[XEarthLayer](https://github.com/samsoir/xearthlayer), a sibling X-Plane 12
project on the same stack (Rust, open source, driven over the Web API): it
layers in what you SEE, this layers in what you HEAR. "Voice" is the honest
through-line, because STT, TTS, the copilot, the tower and the cabin crew have
nothing in common except that they are all speech in both directions.

Three names survive the rename ON PURPOSE, because they are wire contracts, not
branding, and changing them breaks live installs for nothing: the `xcopilot/*`
DATAREF namespace (the app and the plugin agree on these strings), the
`XCOPILOT_*` ENV VARS (the pilot's own `.env` and `prefs.json`), and the plugin
binary `xcopilot.xpl` (rename it and X-Plane happily loads BOTH copies, which
double-registers every dataref). Everything else DID move on 2026-07-30: the
GitHub repo (`dlicudi/xvoicelayer`, old URL redirects, so stale clones keep
working) and the local directory (`~/GitHub/xvoicelayer`). The directory move
was the fiddly one, because three things key on the absolute path and none of
them announce it when they break: `~/.claude/projects/-Users-duanelicudi-GitHub-*`
(25 memories and 55 session transcripts, silently orphaned by a plain `mv` of
the repo alone), the absolute paths in `.claude/settings*.json`, and the
cross-reference at `~/GitHub/rustdecks/CLAUDE.md:34`. The Rust crate
identifier DID change, to `xvoicelayer` — see the tracing-filter warning in the
battle-test section, since a stale filter silently drops every crew-side log
line.

## Relationship to Horizon

This is a **separate product** from Horizon (`~/GitHub/horizon`), not an extension —
and the distinction is the whole point:

- **Horizon = instructor.** One-way, grades you, and *deliberately never touches
  the controls*. That's what teaching is.
- **XVoiceLayer = crew.** Two-way, *acts on the aircraft* by writing datarefs.
  Different trust model, failure modes, and UX. Merging them would muddy both.

Horizon is Python; XVoiceLayer is Rust, so code isn't lifted directly — but two
patterns port over conceptually: its X-Plane Web API client (~30 lines of HTTP)
and its "shell out to the `piper` binary" TTS trick.

## Sibling reference: rustdecks

`~/GitHub/rustdecks` (a Loupedeck Live controller for X-Plane) is the other
half of a two-way dataref/command exchange — when hunting a dataref or command
name, check there before the sim's DataRefs.txt. Its `src/sim.rs` is the
proven Web API client [src/livestate.rs](src/livestate.rs) was ported from;
its `examples/*.yaml` profiles (and the `~/GitHub/cockpitdecks` /
`~/GitHub/cockpitdecks-configs` clones it references) carry per-airframe
dataref/command names verified against real panels (SR22 AP, radios,
transponder, engine gauges). The flow runs both ways: datarefs/commands
proven here (datarefs.rs, the tool layer, aircraft profiles, xlua
study-level tricks) are worth mining for rustdecks pages, and vice versa —
its CLAUDE.md carries the mirror of this note.

## Stack (decided)

Rust, because this is a latency- and audio-sensitive real-time loop — where Rust
earns its keep over a Python runtime + GIL.

| Stage | Crate / approach |
|---|---|
| Mic capture | `cpal` |
| STT | `whisper-rs` (whisper.cpp bindings) — push-to-talk for v1 |
| LLM tool-use | **Local** — `reqwest` → Ollama (`/api/chat`), backend-neutral tool schema |
| X-Plane write | `reqwest` PATCH to the Web API (`PATCH /api/v1/datarefs/{id}/value`) |
| TTS readback | in-process piper VITS via the `ort` crate (ONE engine, see below) |

**LLM runs locally — cloud is ruled out** because per-call API metering doesn't
fit a real-time copilot you'd use for a whole flight. The whole loop is offline:
whisper (ears) and piper (mouth) were always local, and the tool-picker (brain)
is a local model served by Ollama. The tool schema is kept backend-neutral (just
`name` + `description` + JSON-schema `parameters`) so the *local* backend can be
swapped (Ollama → llama.cpp server, a different model) without touching the tool
layer — not as a hook for a cloud option.

**X-Plane access: REST + a native companion plugin.** The app talks to the sim
over the Web API (separate process, same API Horizon polls) — every stock
dataref read/write goes that way. But some things only a plugin can do, so a
small C++ plugin ([plugin/src/](plugin/src/) — since the 2026-07-15 tidy-up,
`xcopilot.cpp` is a thin lifecycle entry over `drefs.cpp` (dataref/command
registration + accessors), `panel.cpp` (the Dear ImGui window and tabs), and
`fms.cpp` (the FMS mailbox and flight-plan composer); `make install` from
[plugin/](plugin/)) lives inside the sim and
*extends the dataref surface* rather than replacing the REST channel: it
registers `xcopilot/*` datarefs (PTT command + `ptt_active`/`radio_ptt_active`,
panel state like phase/heard/action/coach/strip, and mailboxes like
`xcopilot/fms/*`) that the app reads and writes over the same Web API. So the
architecture is REST for transport, plugin for capability — the app never links
the SDK itself. Note: read Laminar's X-Plane SDK license before distributing
the plugin.

## Aircraft profiles drive the tool surface

Tools and their limits are **per-airframe**, not global. An aircraft profile
([src/aircraft.rs](src/aircraft.rs)) declares which systems exist and their
limits; the tool registry is built *from* the profile. The stock **Cessna 172SP**
is the first profile and shows why this is necessary: it has **fixed gear** (so
no `gear` tool is exposed) and **discrete flap detents** (0°/10°/20°/30°) with
**per-notch Vfe** (110 KIAS for the first 10°, 85 KIAS beyond) — none of which a
generic dataref layer could know. Profiles load from `profiles/<id>.yaml`
([profiles/cessna_172sp.yaml](profiles/cessna_172sp.yaml)) with a built-in Rust
fallback, and the active aircraft is **auto-detected** from `acf_ICAO`
(`XCOPILOT_AIRCRAFT=auto`). Datarefs/commands stay in the Rust tools; the YAML
carries per-airframe config (flap detents, lights, capability flags).

The automation's NUMBERS are per-airframe for the same reason, and the trap
is a figure that fits the 172 being read as universal (2026-07-28, both
found in one Luton MD-82 sortie). **Taxi pace** (`taxi:` → `autotaxi::Pace`,
carried BY THE ROUTE, never a static) was one 10/6/4 for everything, tuned
where 10 knots is a THROTTLE setting; on a jet it is a BRAKE setting, since
an MD-82 rolls past it at idle thrust, and the airline figure is 15-20 down
a long taxiway with 10 through the bends (a 1814 m Luton taxi took 6:51).
**Takeoff length** is worse than slow, it is a refusal: `takeoff_roll_ft` is
now the GROUND ROLL the automation checks a runway against, distinct from
the recited `takeoff_distance_ft`, which on an airliner is a factored FIELD
LENGTH at max weight. Read as a roll it earned the consumers' own ×1.4 on
top and demanded 3179 m, more than the whole of Luton's 25, so the line-up
backtracked for length it did not need and `full_auto::lined_up` then judged
a perfectly good line-up a failure, twice, and the MD-82 could not depart
the field at all. Both default to the old figures when a profile is silent,
so every GA airframe is untouched.

A STOCK airframe can expose a gap the add-ons never did (2026-08-01, the
Beechcraft Baron 58, `BE58`). It is the first piston twin whose systems are
all stock: the DC-3 is a twin too, but every system on it is xlua-overridden,
so it goes through the cycle-switch model and the generic tools were never
asked to drive two of anything. They half-did. `set_magnetos` wrote
`ignition_key[0]` only, so the right engine kept whatever the pilot last
selected while the readback — also element 0 — confirmed the change, which is
the mag check confirming itself. `start_engine` cranked `engage_starter_1` and
watched `ENGN_running[0]`, so it would have reported a successful start with
the right engine cold, and full auto would have taxied out on one. Both now
take `engines` and run per engine (the readback reads the LAST one written,
since element 0 can be right while a later one silently refused), and the
starter command is derived per engine rather than a constant. Singles are
byte-identical: one engine writes element 0 and cranks starter 1, exactly as
before. The profile itself is worth reading for where its numbers come from,
because two sources disagree and both are right about different things: the
`.acf` is what the flight model FLIES (the monitor's clean stall is its 84 kt,
not the POM card's 80) and the shipped POM is what the copilot RECITES. Per-notch
Vfe matters here as much as on the 172 — approach flap is good to 152 kt and
full only to 122, and the POM's single "Vfe 152" would put full flap out thirty
knots early. `vyse_kts` and `vmca_kts` joined the reference card for it: blue
line and red line are the two speeds only a twin gets asked for, and they stay
`None` on a single so the copilot says nothing rather than inventing one.

The same airframe found the ground-power tool telling a confident lie the same
day. **A cart and a contactor are two different things.** `set_ground_power`
summoned the cart (`sim/ground_ops/toggle_gpu_request`) and judged success from
`GPU_generator_volts`, which is what the CART is generating and says nothing
about whether the aircraft is drawing from it. The cart arrived, generated 28
volts, and the tool answered "ground power's already connected" — twice, to a
pilot whose battery full auto had just called at 21 volts, with the bus still
on that battery at 20.7. The missing half is `sim/cockpit/electrical/gpu_on`,
the external-power contactor: on an airframe with an EXT PWR switch the pilot
closes it, and the Baron models no such switch, so nobody was ever going to.
The tool now does both, in order (contactor last on connect, first on
disconnect), fires the cart TOGGLE only when the cart's presence is not already
what it wants — firing it blind dismisses the cart you just asked for — and
reports BUS volts instead of the request, because "requesting ground power"
while the battery sits at 21 volts is the answer that started this.

And a third, in the circuit, from the same sortie: **being able to close an
offset is not being lined up.** The 2026-07-30 fix made the handover to
autoland wait for `interceptable` — can autoland's 35-degree pursuit actually
reach the centreline from here — which fixed a threshold crossed 188 m off the
paint. It is the wrong QUESTION for a base leg. The circuit's own final leg
commands the runway heading and nothing else; it has no intercept, so the turn
onto final is the only thing that ever puts the aircraft on the centreline, and
`turning_final` means that turn may BEGIN. At EGHI the Baron was handed over on
base at 290 degrees with 1,813 m still to go to the centreline and 4,092 m to
run: a 28-degree cut, comfortably "interceptable", so autoland took it there
and flew the diagonal in. The aircraft approached and touched down at an angle,
having never flown the turn that was armed a second earlier. `ready_for_autoland`
now waits for our own turn to ROLL OUT (ground TRACK within
`ROLLED_OUT_HDG_DEG` of the runway course — track, not heading, since
2026-08-02: the final leg commands intercept-plus-crab, so in any crosswind
the NOSE sits a whole wind-correction angle off the course on a perfect
centreline, and a heading gate could never pass — 12 degrees at 16 kt across
a 172's 74 kt, with every handover then falling through the last-ditch
valve), with a last-ditch valve
so a botched turn with `LAST_DITCH_ALONG_M` left still gets flown by autoland
rather than by a pattern leg with nowhere to go. A failed track read counts as
NOT rolled out: it can only cost a tick, where the opposite hands over the very
thing the guard exists to prevent.

**A name in a profile is not evidence the thing exists, and the copilot has
hands as well as a voice** (2026-08-02, the Baron again). Adding its roof flood
lights meant probing the live sim for them, and the probe convicted the entry
already sitting two lines above: `flood_lights_dataref` was
`laminar/b58/lighting/flood_light_switch`, and NO DATAREF OF THAT NAME EXISTS.
The name belongs to a COMMAND ("Flood Light Toggle Switch"), so every
`set_flood_lights` on this airframe wrote into nothing and then read nothing
back, and nobody noticed because a tool that fails silently and a tool that
works look identical from the cockpit unless you watch the switch. Its real
position is `laminar/b58/flood_light_toggle/switch_pos`, read-only, so it needs
the same read-before-firing toggle the Citation X taught (`flood_lights_toggle`,
carried through full auto's own copy of the tool as well, or the automation and
the pilot get different behaviour from one switch). The roof floods themselves
are two writable pods, `roof_L_on` and `roof_R_on`, written together for the
landing-light reason: one pod lit is a readback confirming the half it looked
at. The general lesson is the cheap one: `is_writable` and existence are one
`curl` away whenever the sim is up, and a profile line copied from a deck config
is a HYPOTHESIS until then.

The new half is [src/cockpit_auto.rs](src/cockpit_auto.rs), the first thing the
copilot does to the cockpit unasked rather than on request: interior lights
flagged `auto_at_night` come on once the aircraft is powered and the sun is
down, and the GNS430's welcome page gets its two ENT presses
(`gps_ack_command`) once per power-up. "Night falls", "battery on" and "engines
started" are not three triggers but ONE condition becoming true — powered AND
dark — which is why the state machine is over (bus volts, sun pitch) and not
over events. Three rules shape it, and they are all the same rule. It is
one-shot per spell and never re-asserted, because a watch that re-fires every
tick is not an automation, it is a fight with the pilot over a switch (the gyro
sync's cooldown is the same instinct). "Off" is only ever commanded for a light
the copilot itself switched on, and the `set_lights` tool drops that claim the
moment the pilot moves the switch themselves, so daybreak cannot undo a
decision made at midnight. And only INTERIOR lights may be flagged: switching an
exterior light on the pilot's behalf changes what other traffic sees, and a
profile test enforces it rather than trusting the YAML. The sun threshold
carries hysteresis (0 degrees down, +5 up) because a single threshold chatters
through every ripple in the read while the sun sits on the horizon. The watch
runs on its own 2 s clock rather than the monitor's 3 Hz — the sun moves a
quarter degree a minute — and is skipped entirely on an airframe declaring
neither duty, so nothing pays a read for a capability it does not have. It sits
ABOVE the `quiet` narration gate on purpose: a pilot who mutes the panel still
wants their lights. Every light write, the copilot's and the pilot's, now goes
through one `tools::lights::apply` (the KT73 rule), which also made the plain
write path compare its readback against the INTENT — it used to report whatever
came back, so a write that silently failed answered "Roger, landing lights off"
to "landing lights on", a readback confirming itself. Blackboxes as
`"kind":"cockpit_auto"`. NOT yet flown.

A recurring add-on trap the profile exists to absorb: on a study-level
airframe the stock actuator can be the add-on's OUTPUT, not its input — the
model drives the sim dataref from its own switch every frame, so the
universal write is not wrong, it is simply connected to nothing. Two are
wired now, both on the RWDesigns Twin Otter: `fuel_pump_commands` (master.lua
owns `fuel_tank_pump_on` from its FWD/AFT switches) and, added 2026-07-27,
`transponder_mode_knob` — its KT73 is a SASL instrument whose `KT73()`
rewrites `sim/cockpit/radios/transponder_mode` from `TO/radios/KT73SelKnob`
every frame, so the copilot's standby/ALT switching had been a silent no-op
since it shipped (the knob never moved; the mode snapped back within a
frame). All mode writes now go through one shared writer
([src/tools/radio.rs](src/tools/radio.rs) `set_mode`: the tool, the
liftoff-to-ALT in [src/autotakeoff.rs](src/autotakeoff.rs), the
standby-at-the-stand in [src/taxi_in.rs](src/taxi_in.rs)), which turns the
KNOB where one is declared and CONFIRMS from the knob — the airframe's ALT is
knob 5 driving stock mode 2, not 3, so a stock readback would call the right
answer wrong. The selector is republished on EVERY profile swap including to
`None` (the radio-power latch of the same day is the cautionary tale: a
process-wide fact that outlives its airframe aims the next aircraft's writes
at a knob it doesn't have). The lesson generalises — when a tool "does
nothing" on an add-on, read its Lua before touching the tool.

## SimBrief: fetch over HTTP, inject via the plugin's FMS mailbox

The copilot can brief and load the user's SimBrief flight plan
([src/simbrief.rs](src/simbrief.rs), `XCOPILOT_SIMBRIEF_ID`). The fetch is the
**one outbound network call besides X-Plane**: on demand (never in the realtime
loop) it pulls the latest OFP as JSON, distils it to the briefed essentials
(route, cruise, ETE, fuel in gallons, TOD distance) plus up to 14 named
waypoints, and answers deterministically — no LLM.

Loading the route into the GPS/FMS is a three-hop path, because the FMS is
unreachable through datarefs — only a plugin can touch it:

1. The app serialises waypoints as `IDENT,lat,lon,alt|...` and writes two
   plugin mailbox datarefs over REST: `xcopilot/fms/route` (payload) then
   `xcopilot/fms/load` (edge-triggered).
2. The plugin resolves each ident to a navaid (`XPLMFindNavAid` near the
   SimBrief lat/lon, for the right .fms row type; lat/lon row as fallback) and
   composes an X-Plane 11 format flight plan **in memory** — nothing is written
   to `Output/FMS plans/`. The AIRAC cycle is read from `earth_nav.dat`'s
   header and stamped in so the loader doesn't warn about a cycle mismatch.
3. `XPLMLoadFMSFlightPlan` ingests the buffer, then the active leg is set to
   entry 1 (first fix after the origin) on both the legacy GPS and native FMS
   plans. Load-a-whole-plan, not the per-entry API, is deliberate: only a plan
   *load* makes X-Plane parse ADEP/ADES and fill the CDU's ORIGIN/DEST fields —
   per-entry writes leave them blank even with an airport at entry 0.

The plan also feeds [src/monitor.rs](src/monitor.rs)'s `DESTINATION` (TOD ring
against the real field elevation) and the fuel check briefs against what's
actually in the tanks.

## ATC lives inside the copilot binary (the world, not the crew)

ATC (tower dialogue) runs **in the copilot process**, as a frequency-routed
station ([crates/atc/src/station.rs](crates/atc/src/station.rs)): one mic, one PTT, one speech
arbiter. After the normal capture/confidence/echo gates, an utterance goes to
the tower iff COM1 is tuned to the station's frequency AND the words parse as
a radio call ([crates/atc/src/tower.rs](crates/atc/src/tower.rs) — deterministic FSM, keyword
intents, "say again" over guessed clearances); everything else falls through
to the copilot unchanged. The station speaks in its own voice through the
same arbiter (`Voice::say_station`), which since 2026-07-14 runs TWO
independent half-duplex channels like a real audio panel — the copilot's
INTERCOM and the station RADIO, each with its own queue, speaker thread, and
volume (`XCOPILOT_VOICE_VOL` / `XCOPILOT_ATC_VOL`, WAV paths only). The
channels may overlap; realism priority is crew discipline, not ducking: the
intercom defers STARTING anything below Warning while the radio is on the
air (perishable cues drop if they expire waiting), and a Warning (stall,
terrain) talks straight over the radio. Radio-only cuts (retune, avionics
off, crash, replay) kill station audio without touching the copilot's voice
(`tts::stop_channel`). Trust boundary is the module boundary: the station only reads the
sim and returns text — it can never actuate. Frequencies come from apt.dat
comm rows (parsed in [crates/atc/src/nav.rs](crates/atc/src/nav.rs)), not hardcoding; config via
`XCOPILOT_ATC_AIRPORT` (default LXGB), `XCOPILOT_ATC_VOICE`, `XCOPILOT_ATC_RATE`.

Depth (added 2026-07-09): **Mayday/pan-pan** is the one intent that earns a
landing clearance without a final report (strict keywords, checked before the
question guard; approach/departure keep an emergency and vector it home);
**go-arounds** void the clearance and rejoin; **pattern reports**
(downwind/base/crosswind) sequence the circuit; **ATIS broadcasts in-process**
on its own frequency ("with information X" then drops the QNH from
clearances; `XCOPILOT_ATC_ATIS_PIPER_VOICE`); an **enroute region**
(`XCOPILOT_ATC_REGION=off` to disable) indexes every towered field at startup
and hands the flight to whichever tower becomes decisively nearer (monitor
`region_tick` speaks, main loop consumes `PENDING_STATION`) — airborne
"nearest" is service-weighted (`SERVICE_BIAS_NM`), so a real airport outranks
a strip that borrowed its frequencies (Trebujena's rows name "Jerez Tower";
raw distance vectored a Jerez arrival onto its 800 m runway, 2026-07-11), and
a fresh boot with no contact on record gets a crew note, never "radar
services terminated" from a controller that never worked you; and **traffic
advisories** narrate the TCAS picture whenever any controller is tuned, not
just under vectors. Station side state (ATIS/region/controller freqs) is
published by `Station::install()`, called by the owner — never by
`from_airport`, so sweeps and tests stay side-effect free.

**The crew/world boundary is one page** (2026-07-11): everything ATC
publishes for the copilot lives in [crates/atc/src/party_line.rs](crates/atc/src/party_line.rs) —
seven statics, read only through its narrow accessors (`clearance_state`,
`with_taxi_route`, `takeoff_hold_armed`, `vectored_callsign`,
`assigned_vector` — what the controller actually TRANSMITTED (heading,
platform altitude, whether the approach clearance has been issued), for a
crew that intends to COMPLY rather than fly its own path beside a controller
who believes it is being vectored; `vectored_callsign` only says a plan
exists. NO CONSUMER YET, deliberately: full auto's vectored arrival was
written and REVERTED the same day (2026-07-30), because `arm_vectors` lives
on the APPROACH controller and the tower answers a vector request with
"contact Approach on ...". Requesting on the tower frequency and then waiting
is a no-op at every field that HAS approach service, and it cost 45 s of
unmanaged flight at the fields that don't. Doing it properly needs an
`approach_khz` accessor (the Station has the field, the crew has no reader),
a COM1 retune, and a second request on the new frequency —
`landing_handoff_runway` — the approach→tower handoff that still owes the
tower a "final" report, feeding the crew's landing-clearance coach — plus
two `take_*` queue consumers for the main loop). The world side (station/tower)
writes them; the crew side (main, the monitor's crew watches, taxi_guide)
never does. Every other station static is private. Since the workspace split
(2026-07-14) this is compiler-checked, not convention: the statics are
crate-private to `crates/atc`, so a crew-side write is an E0603, not a review
comment. When the crew needs a new fact it gets a new accessor there, never
a new `pub static`. Station tests that
touch ANY process-wide static must hold `SIDE_STATE_LOCK` — `respond()`
publishes on every reply, so one unguarded test races every guarded one
(the vectored-arrival test stomped five others, 2026-07-11).

Depth (added 2026-07-11): the FSM grew **readback enforcement** (ICAO's
runway-critical items only — takeoff/landing/line-up/hold; a correct echo
earns silence, the wrong RUNWAY gets "negative" + repeat, anything else two
challenges then the gate lets go; physics staleness voids a takeoff readback
once airborne; `XCOPILOT_ATC_READBACK=false` keeps the coach suggestions and
drops the challenges); a **LinedUp state** ("line up and wait", the
conditional "behind the landing <telephony>, line up behind" against exactly
ONE arrival ≥1.5 nm out, and cancel-takeoff now lands here so the release
watch owes the re-clearance); a **runway in use that can't drift**
(`pick_runway`: FROZEN while a departure is committed —
TaxiCleared/LinedUp/TakeoffCleared — plus `RUNWAY_SWAP_MARGIN_KTS`
hysteresis, because two parallels have the same heading to within rounding
error and the ≥5 kt guard never protected against THAT. At EGLL the pick
flip-flopped between 27R and 27L on noise in the wind direction, invisibly:
both are "two seven" in speech. A silent flip after the 27L taxi clearance
pointed the takeoff hold at Heathrow's ARRIVAL stream, which never has a
15 s gap — held at November one for the full eight minutes, "traffic on
short final" for a departure runway two probes found empty, 2026-07-27.
The arm log names the runway and every gate now, and a held departure logs
WHAT is holding it every 15 s, because "held for traffic" is undiagnosable
afterwards otherwise); a release gate that THREADS the arrival stream
instead of waiting it out (the runway being occupied is absolute, but an
arrival on final is a question of GAP: `DEPART_GAP_NM` 3 nm, about eighty
seconds at approach speed, is the gap real towers depart into. Requiring
`final_traffic_for`'s whole 4 nm corridor to be empty made the release
unreachable under injected traffic — a single-runway field working a landing
every couple of minutes never HAS an empty final. Gatwick 26L, held at
holding point Mike behind a lander at half a mile, then eight minutes of
stream and a hand-back; every plane landed except ours, 2026-07-27. Inside
the gap but with the strip clear the departure is STAGED onto the pavement
with a line-up-and-wait, under the same one-arrival/`CONDITIONAL_MIN_NM`
rule the tower's own conditional line-up uses — a lander at half a mile
earns the hold, not the runway); **wake turbulence** (a 2-minute clock started
by an airline-prefixed ghost lifting off our strip — TCAS carries no type,
so airline callsign = medium-or-heavier — which turns "cleared for takeoff"
into LUAW and blocks the hold release, but measured from when the HOLD went
on, NOT the last liftoff: separation is owed behind the departure ahead of
you IN SEQUENCE, and a ghost that took your slot while you sat at the
taxiway restarts nothing you owe. Un-capped, a field whose departure
interval is shorter than the separation itself made the release condition
unreachable — Heathrow 27L lifts one off about every two minutes, and full
auto sat out its whole 480 s budget at holding point Sierra before handing
back, 2026-07-27. The same watch now SEQUENCES a held departure instead of
leaving it behind the line: strip clear + wake clock running = "line up and
wait" (`PENDING_LINE_UP` → `apply_monitor_line_up`, the
`PENDING_TAKEOFF_CLEARANCE` pattern one state earlier), never onto an
occupied runway. `ensure_takeoff_cleared` takes `accept_line_up` for the
same reason — held SHORT, a LUAW is the answer we're waiting for; already
lined up, only the clearance will do, and without the distinction full auto
waited out its budget behind the hold line while the tower believed it was
on the strip); and **named taxi routes** (apt.dat
1201/1202 network, background-loaded per station in `TAXI_NET`, Dijkstra
with runway edges ×25 → "taxi to holding point Alpha one runway two seven
right via Bravo, Link two seven", crossings called out — detected from
strip GEOMETRY, never the network flags (`mark_runway_crossings`: authors
leave runway edges unnamed AND draw crossing links as plain taxiway edges
across the strip — EGLL's N7 crossed an active 09R with no clause, no
guide call, 2026-07-20; any leg sampling onto a physical strip is renamed
to the designator pair and runway-flagged, feeding the clause, the
guide's crossing cue, and the cleared-crossing exemption; the post-trim
final leg is exempt) — and a crossing on the cleared
route is never berated as an incursion: silence while moving, "expedite"
when stopped (the tower shouted "on runway two seven without clearance" at
a pilot following its own route, 2026-07-14) — off-route nudges
from the compliance watch via `TAXI_ROUTE`. A runway destination snaps to a
node that can REACH the runway, never merely the taxiway node nearest the
threshold (`plan_to_runway` / `runway_entrance_near`): EGHI's parallel
taxiway ends in a dead-end stub 172 m abeam the strip, which being 631 m
from the 20 threshold against 1047 m for the nearest real entrance won the
plain snap — so the tower cleared us to "holding point Alpha five runway two
zero" and the drive stopped at a stub connected to nothing, with no runway
edge at the goal the hold-short trim never ran and `lead_on` came out empty,
leaving the line-up to invent the entrance the drawn lead-on exists to
supply (2026-07-30). An ENTRANCE is decided by connectivity, never edge
names, for the same reason `mark_runway_crossings` distrusts the flags: on
a runway edge, or one hop from one. It is a class and not a field — 3,446
of 22,334 runway ends worldwide move to a real entrance, 776 have none
authored at all and keep the plain snap (never a refusal: no entrance in
the data is not a reason to refuse to taxi), and zero previously-routable
ends became unroutable. The split is by CALLER, not inferred from geometry,
because the three destinations genuinely differ: a threshold, a stand, and
the off-route replan's already-trimmed holding point — which sits a
standoff back and must not be dragged onto the junction the trim removed). The coach's first suggestion is
always the readback owed (`ClearanceSnap.pending_readback`); `suggest_call`
(one source for the voice coach AND the panel's Calls tab, 2026-07-13)
returns a structured `Suggestion` — a prerequisite line when one exists
(mistuned COM1 → "first tune tower ...", an ATIS nobody has listened to →
"first copy the ATIS ..."), the call itself (the REAL holding point + runway
read from `TAXI_ROUTE`, never an invented "Alpha"), and approach-phase
coverage (under vectors → "no call owed", ILS clearance ridden → the
"established" report, tower handoff → the owed "final") — and the panel
payload leads with two marker bytes: urgency (an owed readback renders amber
with a dot on the Calls tab) and phase group (departure/arrival, lighting
the matching phraseology-reference section). PTT has TWO
channels for CONTENT (the **radio PTT**, `xcopilot/radio_ptt_active`, is
transmit to ATC; the **intercom PTT** — keyboard PTT or
`xcopilot/ptt_active` — is talking to the copilot), and when the radio key
is bound, **physical routing beats content routing in BOTH directions**
(`Ptt::has_radio_channel`): radio-keyed speech never reaches the copilot,
and intercom speech never content-routes to ATC ("requesting clearance"
said to the copilot was answered by the tower, 2026-07-12). Content routing
survives only for setups with no radio key (keyboard PTT, continuous),
where it's the sole path to ATC. For AUDIO the intercom and radio channels
play independently (2026-07-14) — EXCEPT under PTT: **any PTT mutes
everything while held** (`set_hold`; copilot, controllers, ATIS, ghosts) —
that rule is about MIC HYGIENE, not channel symmetry: on speakers, any voice
still playing bleeds into the capture — and release rejoins LIVE audio;
whatever was said during the press is simply gone, walkie-talkie style, no
replay (the earlier stepped-on/resume machinery is deliberately REMOVED,
2026-07-12: the pilot flies on speakers and headset interchangeably, and a
muted-everything press keeps the mic clean on both). Recovery stays
procedural: "say again" / readback enforcement, never an auto-repeat.
ONNX session CREATION is serialised across every voice thread
([src/tts_onnx.rs](src/tts_onnx.rs) `build_session`): the runtime builds its
environment and graph-optimizer registries lazily on whichever thread makes
the first session, and two arriving together SEGFAULTS inside the transpose
optimizer rather than erroring — a boot crash on 2026-07-28 that ran fine on
the next attempt, which is the signature of exactly that race. Inference
stays per-session and parallel. The barge-in cut kills by REGISTERED pid + a PER-CHANNEL cut generation
([src/tts.rs](src/tts.rs) `run_audible`), not `pkill` by name — a press
landing in the spawn window used to be missed entirely and the whole
transmission talked over the pilot. The radio needs
POWER in both directions: with the avionics master OFF, *station* audio
(controllers, ATIS, party-line traffic) is dropped, never queued — the
arbiter's `radio_powered` flag, published by the monitor off the airframe's
`avionics` readback ([src/monitor.rs](src/monitor.rs)
`avionics_readback_dref`) — and a keyed TRANSMISSION goes nowhere: the
tower never hears it, the FSM never moves, and the copilot (crew on the
intercom, no radio needed) says "radios are dead" (a cold-dark SR22 raised
Gibraltar and got a squawk, 2026-07-12). FAIL OPEN by design, both
directions: an airframe with no avionics system, or a read that fails,
keeps the radio powered — a mysteriously dead radio is worse than one that
works with the panel dark. Fail open is RESTORED every tick
(`radio_powered_from`), never merely defaulted at boot: the flag is
process-wide and outlives an airframe switch, so a watch that simply stops
publishing latches whatever the LAST aircraft said. A cold-and-dark 172
published false, the pilot swapped to a Twin Otter (no avionics master,
nothing left to publish), and the stale false silently binned every crew
radio call for the rest of the session — full auto's clearance and taxi
requests never left the aircraft and it blamed the tower ("no taxi
clearance came back", Heathrow 2026-07-27). The drop is no longer silent
either: `take_crew_radio` drains unconditionally, so a failed gate loses
the call for good and the caller reads the silence as ATC not answering.
And ONE frequency at a time: monitor-side
controller calls (compliance berates, incursion shouts, go-around orders)
speak as the controller COM1 is actually tuned to (`tuned_speaker` in
[src/monitor.rs](src/monitor.rs)), and tower-frequency traffic (the
after-takeoff handoff, the held-clearance release) goes silently into the
void when tuned elsewhere — a berate hardcoded to the tower's voice while
the pilot sat on the approach frequency put two controllers on one radio
(2026-07-12). And a REPLAY is nobody's flight (2026-07-14):
`sim/time/is_in_replay`, read each monitor tick and published via
`voice.set_replay`, gates the whole reactive surface — the monitor skips
its tick (ATC compliance/incursion/region watches, crash recording, the
flight-ended debrief, callouts), the arbiter drops station audio (the crew
intercom still answers), keyed and content-routed transmissions are refused
with a crew line, and the autoland loop pauses and won't arm. A replayed
crash landing had earned "you landed on runway two seven without clearance"
plus a second blackbox crash event. Fail open: a failed read means live.
PAUSE (`sim/time/paused`) shares the freeze but not the audio drop: the
flight is real, so a controller mid-word finishes the sentence and station
audio is NOT cut — but the monitor tick, the transmit paths, and every
control loop still freeze (`xplane::sim_frozen` covers both flags: the
speed-hold / autoland P+D throttle laws would walk the lever to its clamp
against a frozen airspeed and surge on unpause, and the recover watcher
slides its 25 s deadline instead of burning it). Arming autoland WHILE
paused is allowed on purpose (real position, flies on unpause); arming in
replay is refused.

Depth (added 2026-07-11, evening): **ILS approaches from real chart data**,
and **region-accurate phraseology**. `spawn_ils_load` (like the taxi net, a
background scan per station) reads the field's localizers/glideslopes from
earth_nav.dat and the per-runway platform (glideslope-intercept) altitude
from the CIFP approach legs ([crates/atc/src/nav.rs](crates/atc/src/nav.rs) `airport_ils` /
`ils_platform_alt_ft` — the FAF leg's crossing altitude, desc-code 4th char
`F`). When the runway in use has an ILS, "request vectors" arms an ILS plan
instead of the visual: descend to the REAL platform, the clearance rides
the intercept turn ("cleared ILS approach runway two seven right, report
established"), and "established" (which parses as the final report —
[crates/atc/bench/atc_corpus.yaml](crates/atc/bench/atc_corpus.yaml) pins the question-form
safety) earns the tower handoff from approach or the landing clearance from
tower. Phraseology follows geography like the voices do (`Dialect` in
[crates/atc/src/station.rs](crates/atc/src/station.rs), K/P/C prefixes = FAA grammar, everywhere
else ICAO/CAP 413: "descend to altitude ... feet" + "degrees" vs "descend
and maintain" + the FAA PTAC clearance) — applied to the approach/vector
phrases AND the pressure clause everywhere (`spoken_pressure`: `ctx.qnh`
carries the whole dialect-correct phrase, so FAA fields hear "altimeter two
niner niner two" in inches and `tower::qnh_clause` is the one seam);
rolling the full dialect through the tower's remaining strings (and the
ATIS broadcast, which still reads hPa QNH everywhere) is future work. The
copilot's
existing `visual_approach` talk-down and NAV1 ILS auto-tune compose with
all of this. Real-install smoke test:
`cargo test -- --ignored real_install_ils`.

Depth (added 2026-07-11, crew side): the **progressive taxi guide**
([src/taxi_guide.rs](src/taxi_guide.rs)) is the copilot's half of the taxi
clearance — the pilot monitoring with the airport diagram out. The tower's
route publication (`TaxiWatch`, now carrying per-segment names + runway
flags from the planner) feeds pure geometry that speaks just-in-time cues
in the copilot's voice at Advisory: "next left, onto Sierra five Whiskey",
the runway-crossing check, "straight ahead onto..." after it, and the
holding point. Division of labor: the guide gives the gentle EARLY cue and
goes quiet past the same 60 m where the compliance watch's ATC challenge
takes over — never two voices on one mistake. On-demand: "which way?" /
"say the taxi route" answers deterministically off the same route
(`is_taxi_query` in main.rs, question-shaped so a taxi READBACK never
triggers it). `XCOPILOT_TAXI_GUIDE=false` to disable. All of it battle-
tests as plain `cargo test` (synthetic routes in
[src/taxi_guide.rs](src/taxi_guide.rs)); cues blackbox as
`"kind":"taxi_guide"`.

Depth (added 2026-07-20): **auto taxi** — "taxi for me" / "you have the
taxi" / bare "taxi" (grammar-deterministic; a "taxi to holding point ..."
READBACK never arms it, same trap `is_taxi_query` dodges) and the copilot
drives the cleared route to the holding point.
[src/autotaxi.rs](src/autotaxi.rs) + the `auto_taxi` tool
([src/tools/taxi.rs](src/tools/taxi.rs)) compose the two existing halves:
speed is the SAME taxi governor, armed through `hold_speed`'s front door
(every parking-brake/held-brake gate applies) and retargeted live via
`speedhold::set_taxi_target` (6/4 kt caps into bends, a √(2·a·d) ramp to a
stop at the hold-short point, "Holding position, brakes set" — the ROUTE
ends there, best data first: apt.dat 1204 ACTIVE-ZONE rows (parsed onto
`TaxiEdge.zone_of`; `ils` rows skipped until LVP ops exist) mark which
network edges sit inside each runway's protected zone, and `plan` drops
the trailing zone edges for the destination runway so the route ends at
the authored per-stub holding point (EGLL marks every entry); fields
without zones fall back to a geometric trim of the runway-junction tail,
class-scaled in `plan_taxi_route_to` — ILS on the strip = 100 m (the GA
40 m parked a Heathrow taxi on the 09L entry stub past the CAT line,
2026-07-20), else width-scaled, floor 40. Untrimmed routes ended ON the
centreline junction and parked the nose across the hold line, same day);
steering
is pure pursuit over the party-line polyline, written through the plugin's
`xcopilot/taxi_steer` channel (the taxi_brake pattern exactly: per-frame
`yoke_heading_ratio` under `override_joystick_heading`, 1 s expiry, so any
death of the writer frees the pedals by silence). The pilot's feet win
IN-PLUGIN: real yaw-axis deflection (baseline captured at engage, so an
off-centre resting axis can't false-trigger) latches `xcopilot/steer_yield`
and drops the override that frame — the app loop then stands down to
"you steer, I have the throttle". Handbacks: "my steering"/pedals release
steering only, "my controls" everything; the governor standing down for ANY
reason takes steering with it (speed authority gone = nobody stopping it);
established, >25 m off route = stop + release, never a driven recovery on
uncleared pavement. But an engage that STARTS off the route or facing
against it is a JOIN, not a refusal (2026-07-20 — "turn us around first"
and "get us onto it and ask again" both bounced live requests): the loop
engages anyway, pure-pursuits toward the route at bend pace (sharp-corner
speed while the nose is >60° off), and the off-route guard widens to the
engage distance plus 40 m of turning room until captured (<15 m and
aligned), when the tight guard resumes. Steering authority tapers with
groundspeed (`steer_authority`: 0.35 parked, full by 3 kt) — full nosewheel
lock at a standstill is a chock the governor's 0.35 throttle cap can't
break away against (a 69°-off join sat static at Gatwick, 2026-07-20) — a join can never end farther from
the route than it started, and >150 m off (`MAX_JOIN_M`, where the
projection stops meaning anything) still refuses. Request-shaped asks with
a destination attached arm it too ("let's taxi to holding point" —
apostrophes stripped, so `lets taxi`; "taxi us ..."): a readback never
opens first-person plural, so the readback trap only excludes bare "taxi
to holding point ..." shapes (both phrasings went unanswered live,
2026-07-20). Spoken "stop"/"go"/"taxi at N" fold into the schedule as
external retargets (`LAST_TAXI_KTS` untouched by schedule writes, so "go"
resumes the pilot's pace). Refuses in replay, airborne, way off the route,
or with no published clearance. The schedule also knows where the aircraft
IS, not only how the paint bends ahead (`Pace::track_cap_kts`, 2026-07-30):
the bend caps read the route and nothing else, and the JOIN cap stops at the
`join_done` latch — 15 m and 45 degrees, loose enough to fire in the middle
of a capture that has not happened. Southampton: the SF50 latched joined at
14.8 m and -28 degrees, the last leg was straight so the cap returned to
12 kt, and it accelerated 6.5 to 12.7 kt while still 13 m off with 27
degrees of crossing angle and 33 m to run. On a castoring-wheel jet whose
whole steering authority is a quarter of one-sided brake (~3.5 deg/s flat
out) that is unrecoverable — it floats wide, which is exactly the trail.
Cross-track and heading error now cap the schedule at the bend pace, then
the sharp pace, every tick and past the latch; slowing is what buys a weak
plant more degrees per metre. It can only ever slow, so an airframe holding
the centreline is untouched. The governor evaporating
mid-drive is ARRESTED, never silent: the throttle-override check can't
tell a pilot's hand from a hardware-axis bump the sim re-drives, and the
first Gatwick sortie (2026-07-20) ran 10 to 50 kt through the grass at
taxi power with nobody on the throttle and no voice line — now hold-gone
on the ground closes the throttle, fires one expiring brake pulse (the
pilot's own brake key, frees itself in a second), and says "your
aircraft" at Caution; airborne it touches nothing. Both override
detectors are debounced against no-touch hardware noise (same sortie,
pilot's hands in his lap): the governor CHALLENGES the first deviated
throttle read (re-command once, look again — a hand keeps re-driving and
wins the second read, a pot-jitter re-drive to the quadrant's resting
position doesn't), and the plugin's pedal yield arms 0.5 s after engage
with the baseline following the axis until then (the probe-time baseline
read feet coming OFF as a press) plus a 4-frame sustained-deflection
requirement. The taxi guide keeps
narrating over it — one voice, the copilot calling its own turns. Laws
battle-test as plain `cargo test`; events blackbox as `"kind":"autotaxi"`.
Steering on a castoring nosewheel is differential BRAKE
(`brake_steer_split`), because yoke_heading has nothing to work with there —
no nosewheel bite and no prop wash over the rudder. It shipped 2026-07-28
writing the stock `cockpit2/controls/*_brake_ratio` over REST, which is a
NO-OP whenever a toe-brake axis is assigned: XP12 re-drives those ratios from
the pilot's hardware every frame, exactly as the symmetric taxi_brake channel
had already learned in July. Probed at a Southampton stand (2026-07-30) a
PATCH of 0.25 read back 0.0000 within 400 ms bare, and held 0.2500 under
`override_toe_brakes` — so the SF50 spent every taxi with its only real
control surface disconnected, and the blackbox shows it: the same command
magnitude that gave -4.93 deg/s in the first 180 m returned -0.27 for the
remaining 700, cross-track wandering to 13.9 m in a limit cycle that never
damped. Every brake-steer tuning note from the 2026-07-28/29 sorties
(`BRAKE_STEER_MAX` 0.25 vs 0.16, `BRAKE_STEER_SAT` 0.6 and its "starvation"
diagnosis) was therefore tuning a number that never reached the aircraft and
is worth re-deriving. The split now goes through plugin channels
(`xcopilot/taxi_brake_l` / `_r`) which hold the override at frame rate, and
the plugin arbitrates them against the governor's stopping brake PER WHEEL —
one owner, since two writers on one dataref is the whole bug. The app writes
them EVERY tick: the channel is a heartbeat that expires in a second, so the
old change-gate (fine against a persistent dataref) would drop the brake in
the middle of a turn, which is exactly where a command sits still. All three
control writes share one `tokio::join!`, so the tick still costs one round
trip. Not yet flown — the sim was shut down before the live re-test. A brake
YIELD is still future work: the override deafens the pilot's toe brakes while
it is up, and only the 1 s expiry frees them, where steering has
`steer_yield`.

Depth (added 2026-07-20, same day): **auto takeoff** — "take off for me" /
"your takeoff" / bare "take off" (first/second-person shapes ONLY; the
readback "cleared for takeoff runway two seven" and the call "ready for
takeoff" carry none of them, same trap discipline as auto taxi).
[src/autotakeoff.rs](src/autotakeoff.rs) + the `auto_takeoff` tool
([src/tools/takeoff.rs](src/tools/takeoff.rs), registered with the AP
tools) composes the existing parts: the runway comes from apt.dat
(autoland's parser, `runway_fit` — aligned within 20°, within 40 m of the
centreline, enough length ahead vs the POH roll ×1.4), the roll steers the
centreline through the same plugin steer channel with authority TAPERING as
speed builds (the inverse of the taxi taper — full rudder at 50 kt is a
swerve), and the climb-out is AP-first like autoland: bug and VS dial are
PRE-SET parked (the knob slew is one HTTP press per degree — doing it at
rotation stalls the loop while the steer channel expires), servos + HDG +
VS engage at Vr, then a speed-protected climb law (rate backs off while
below Vy, floored positive) to the 1000 ft AGL handoff — "your aircraft,
the autopilot is holding runway heading and the climb". Being a normal duty
and not an emergency, it REFUSES without a takeoff clearance when a tower
is running (LinedUp explains the hold; `party_line::clearance_runway` is
the new accessor feeding a line-up crosscheck — lined up on 09 with a
27 clearance is a refusal, the backwards-line-up trap); no station
(copilot-only mode, real ATC) leaves the clearance question to the pilot.
In-loop guards, not just arm-time: not accelerating in 10 s = reject,
runway remaining under 250 m below 0.9 Vr = reject (throttle idle, one
expiring brake pulse, "your aircraft"); "abort takeoff"/bare "abort" is
the arresting reject path, "my controls" is a silent HANDOVER (power stays
set — a pilot taking over mid-roll wants it); pedal yield hands over with
"power is set"; a throttle pull is challenged once (axis jitter) then
honored as a pilot reject. Vr/Vy/climb rate come from the profile's
reference figures with stall-speed fallbacks. Laws battle-test as plain
`cargo test`; events blackbox as `"kind":"autotakeoff"` (per-tick `roll`
events carry gs/ias/err/steer/trim/xtk — mine them to tune). Two sorties
(LXGB 27, 2026-07-20) hardened it: throttle RAMPS to full over 2.5 s
(slammed at a standstill is maximum P-factor yaw at zero rudder airflow);
the taxi governor is stood down INSIDE the loop only after every refusal
gate passes (standing it down in the tool armed both failure modes: sortie
one it braked against the roll — it was still holding a manual taxi — and
sortie two a refusal mid-line-up left the aircraft coasting uncontrolled
for half a minute); the tool refuses while an auto drive is ACTIVE and
above 5 kt groundspeed (engaging on a moving, swinging aircraft is how
sortie two left the pavement); and the roll law is `RollSteer` — P+lead
damping PLUS a slow capped INTEGRATOR that learns the standing rudder
P-factor demands (pure P settled at a standing 18° error, then the old
authority taper starved it to 0.21 of deflection and the aircraft ran 86 m
off the centreline and rotated from the grass 39° off heading; the trim
holds rudder with the nose straight, authority floor 0.65, cap 0.85,
slew-limited). Sortie three flipped the low-speed picture: under takeoff
power PROP WASH makes the tail hyper-effective at a standstill — a 0.85
command at brake release (fixing the 14° the line-up left) spot-turned
the aircraft at 25 deg/s into a runway pirouette — so the deflection cap
is now tiny parked (0.12) growing to full by 15 kt, the trim integrator
is frozen below 10 kt, the stagnation reject watches GROUNDSPEED (the
172's ASI reads zero below ~20 kt and the reject fired against a dead
instrument), and the line-up's alignment leg grew to 80 m (its arrival
is distance-based; 60 m stopped 11° off). Power and rudder move together
like a pilot's hand and foot: the ramp HOLDS at half power while the
nose is swinging >10° at low speed (`throttle_cmd`), and `power_lead`
feeds right rudder in proportional to the throttle actually commanded
(fading with speed, integrator learns the residual; sign assumes a
clockwise prop). Ground-loop comfort, same day: the taxi governor's
brakes go through the plugin's proportional channel for stops AND trims,
slewed (`brake_slew` — squeeze on, quick release; the binary brake key
survives only as fallback), and the taxi/line-up steering straightens the
nosewheel over the last meters of any stop (`stop_straighten` — braking
shifts weight onto a deflected nosewheel and the aircraft carved at
every stop). Rotation additionally waits
for a roughly straight nose
(`rotate_ok`, unless the runway is running out), and the airborne latch
needs 8 ft AGL so a ground-effect bounce can't stop the steering at speed.
Sortie four (2026-07-21) reached the 1000 ft handoff but wove ±8° the
whole roll, and the telemetry convicted the LOOP, not the gains: eleven
sequential HTTP reads dragged the 150 ms tick to half a second, and the
0.25-per-tick steer slew (sortie three's anti-jerk) swept only 0.5/s —
the rudder chased the error in exact 0.25 steps a second-plus behind, a
textbook slew-induced limit cycle. The reads are now one `tokio::join!`
burst, the slew is per-SECOND and dt-scaled (`STEER_SLEW_PER_S` 1.5; the
low-speed cap owns the parked case), the authority floor dropped back to
0.35 (the trim integrator adds OUTSIDE the taper now, so lowering P at
speed can't re-create sortie two's starvation), the trim clamps the error
it learns from (±6° — a 39° transient wound it to the cap in 2 s and the
stale trim fueled the next overshoot; cap 0.3), and the throttle
ADVANCES are slewed (`throttle_slew` — the swinging-nose hold's release
jumped half-to-full in one tick and kicked 19° of yaw at 12 kt; releases
stay instant). Roll events carry `dt` now — check it stayed near 0.15
before touching gains again.
Sortie five (same day) proved the loop fix (±2.7° all roll) and convicted
the ROTATION: the AP's VS mode syncs its dial to the CURRENT vertical
speed at engage — zero on the runway, whatever was pre-set — so "Rotate"
at 55 kt left the nose held down and the aircraft levitated off at 98 kt
with 250 m left; and the first engage sat 12 s at full power against the
parking brake the line-up's "brakes set" had latched (stagnant reject).
Rotation is ELEVATOR now: a new plugin channel `xcopilot/pitch`
(yoke_pitch_ratio under override_joystick_pitch, the taxi_steer pattern
exactly — 1 s expiry, `xcopilot/pitch_yield` latches on real pitch-axis
deflection and the yoke is the pilot's that frame) driven by `RotatePitch`,
an integrating attitude law (8° target, rate-lead damped, caps 0.65/-0.10)
from Vr through the hand-flown initial climb; the AP takes the ESTABLISHED
climb at 300 ft AGL — servos + HDG + VS engaged there, the VS dial written
AFTER the mode (engage syncs it away; the climb block also rewrites it
after any VS re-engage), then the pitch channel just expires. The takeoff
releases the parking brake itself ("Brakes released. Rolling"), and the
tool probes the pitch channel at arm time (an old plugin would roll fine
and sail off the far end unrotated).
**The handoff line must be true about who has the aircraft** (2026-08-01,
caught in the cockpit mid-flight, not in a log). It read "One thousand feet,
128 knots. Your aircraft. The autopilot is holding runway heading and the
climb." — unconditionally, and it was wrong twice over on every circuit. A
takeoff armed as a STAGE of the circuit, full auto or a procedure hands to the
ORCHESTRATOR, not the pilot, so "your aircraft" told the pilot they had control
at the exact moment the circuit was taking it; a pilot who believes that either
fights the automation for the yoke or sits back while they think somebody else
is flying, and the whole trust model rests on the copilot being right about
this one thing. The circuit also hands off at 500 ft, where the hardcoded "one
thousand feet" was a number the pilot could see was wrong on the altimeter, in
the same breath as the claim about control. `Params.handoff_to_caller` (set by
the two construction sites, which happen to be exactly the two cases) drops the
"your aircraft" clause, and `spoken_height_ft` calls the height actually
reached.

**The rotation rang for four days because the damping term was measuring the
wrong thing, and no test could fail** (2026-08-02). The initial climb oscillated
about its 8 degree target from liftoff to the 300 ft AP handover on every
sortie. It was diagnosed twice and "fixed" twice: the rate was smoothed (which
made it measurably worse, since a filter puts lag where the phase margin is
thinnest), then the law went P+I on the argument that elevator to pitch rate to
attitude is already a double integration and a third could only ring. The
structure was right and the ringing did not stop. Across 49 takeoffs in the
blackbox the same-airframe pair either side of that change (SF50, EGHI 20, one
evening) went 1.26 to 1.89 direction reversals per second and 20% to 48% of
ticks pinned against `PITCH_MIN`: the peak-to-peak halved and the oscillation
got FASTER, which is the shape of a damping problem, not a structural one. Both
faults were in the one term nobody had suspected. The rate was DIFFERENCED from
the attitude over the loop's 0.25 s tick, which amplifies every wobble in the
read by 1/dt and carries an inherent half-sample delay, so the damping arrived
about 50 degrees out of phase at the ~0.9 s period it actually rang at and drove
the motion instead of opposing it. And `PITCH_LEAD_S` was 1.0 s, the far side of
a cliff: swept against a plant, the worst-case spread is flat at 2.3 degrees
anywhere in 0.3 to 0.5 and breaks to 10.2 at 1.0. It now damps on the sim's
MEASURED pitch rate (`sim/flightmodel/position/Q`, already used by the recover
tool), which needs neither differencing nor a filter, at a 0.3 s lead. 0.3 and
not 0.5 because it is the only value that also survives the fallback when the
read fails: a failed read must cost accuracy, not stability. The reason three
gain changes shipped with every test passing is that the sortie test's pitch
model is a first-order lag, and a single lag cannot ring at any gain, so it
would have passed for ANY of them. `the_rotation_settles_instead_of_ringing`
flies the law against a second-order plant through the loop's real transport
delay and asserts the two numbers the blackbox reports, attitude spread and time
spent against a stop; it fails at the shipped 1.0. Per-tick `climb_manual`
events now carry `q` and `dt`, because the ring was in the damping path and the
record carried nothing about what it damped on. NOT yet flown.

Related trap fixed the same day: `looks_like_command` (the continuous-mode
domain gate) matched per-token keywords only, so the split compounds
"take off ..." and "line us up" were swallowed with no miss chime —
bigrams are checked on the raw text now. The same gate was missing **"stop"**
entirely (fixed 2026-08-01), which nobody noticed because the command loop has
a PRE-GATE stop/go branch that fires while the speed governor is running — so
the word worked in the one case anyone tested it in, and was dropped as ambient
everywhere else, with no miss chime. A push runs with no governor and the
brakes off, which is exactly where it matters: a Baron rolling off its stand at
21 kt, "Stop." at 0.9996 confidence, nothing. The keyword is in the table now
(passing the GATE is not actuating — the grammar's bare-stop rule still wants
the whole utterance, so "stop for fuel" reaches the router and not the brakes),
and the pre-gate branch grew a `pushback::ACTIVE` arm that sets
`pushback::ABORT` and cancels full auto with it, because "stop" during a push
means stop, not "skip the push and start taxiing". The arm learned who is
listening (audit, 2026-08-02): ABORT only has a consumer when full auto's
STEERED loop is flying the push (`pushback::steered()`), and the spoken
tool's NATIVE push latches ACTIVE with no loop at all — so against a native
push the branch swallowed every stop-phrase (the panel CANCEL's own "stop
pushback" included) while the tug kept pushing and the copilot claimed
brakes it never set. The native arm now sends `pushback_stop` and sets the
brakes itself and clears the flag; airborne, a set flag is definitionally
stale, so it is healed and the words route on ("stop the descent" must never
cascade `full_auto::abort()` into whatever is flying — which is also why the
steered arm's abort is what it always was, full auto's own); and with the
governor also driving the branch falls through so "stop" still reaches the
brakes it was for.

**Line up** (same day) bridges the two: "line us up" / "line me up" /
bare "line up" (the LUAW readback always carries "wait", "lining up" is
the short echo — both excluded) drives from the hold-short position onto
the runway and stops aligned on the centreline. `line_up_path` /
`pick_line_up` in [src/autotaxi.rs](src/autotaxi.rs) build a synthetic
three-point route (entry ahead of abeam by the lateral offset, then 60 m
down the centreline to finish pointing down the strip), picked against
`party_line::clearance_runway` so a "line up and wait runway two seven"
can never be driven onto the 09 end, and refused with less than 400 m
remaining past the stop (the wrong-end trap). But synthetic geometry is the
FALLBACK, never the first choice: the entry onto the runway is DRAWN, and
`line_up_path_from_lead_on` rides it — the lead-on is the tail the
hold-short trim removed from the cleared route (`TaxiRoutePlan.lead_on`,
the painted line from the holding point to the runway junction), with a
sampled fillet where the network models a right angle and the paint curves.
Inventing that entry is how an aircraft ends up in the grass, and the
backtrack proved the lesson doesn't transfer for free: `pick_line_up_plan`
sends a full-length departure off a mid-runway hold on a
`backtrack_pocket_path`, and THAT one still aimed leg one at a centreline
point 30 to 60 m nearer the threshold — 23 degrees off the stub the
aircraft was standing on. A Luton MD-82, cleared to line up on 25 from a
hold 430 m down the strip, left the pavement inside fifty metres, bogged in
the dirt, and the governor called it stagnant (2026-07-27).
`backtrack_pocket_path_from_lead_on` is the same shape as the plain one now:
up the drawn stub, fillet, then down the runway to the turn pocket — the
only thing we choose is which way to turn at the top. The drive IS an auto taxi:
same governor front door (parking-brake gates apply), same steering loop,
same handbacks — only the arrival line differs (`Route.done_line`:
"Lined up runway two seven. Holding position."). Gated on
LinedUp/TakeoffCleared when a tower is running; TaxiCleared is refused
with why. An auto-takeoff asked from the hold now answers "say line us
up, then give me the takeoff" when the line-up is in reach.

Depth (added 2026-07-23/24): **the autoflight layer** — three orchestrators
over the proven loops, no new flight laws. [src/circuit.rs](src/circuit.rs)
("do a circuit") flies takeoff, a POSITION-triggered rectangle scaled to the
AP's real turn radius (timed legs and fixed widths degenerated into loops;
the heading bug is SLEWED short-way, never jumped across the 0/360 seam),
terrain+airspace side/height pick (`choose_pattern` — probes both downwinds
via the plugin terrain channel and the OpenAir file, switches sides off the
Rock, raises TPA when boxed in), then hands the final to autoland or, when
positioned for it, the published approach. [src/procedure.rs](src/procedure.rs)
("fly the ILS/SID") follows CIFP legs — great-circle to each fix,
wind-crabbed, VS law to constraints, FAF handoff to autoland; refuses
honestly at fields with no coded procedure (falls back to an approach
reconstructed from the runway's DME fixes — Gibraltar). [src/full_auto.rs](src/full_auto.rs)
("full auto") chains it all from cold: power (turbine only), engine start
(ONE per airframe CLASS — `start_kind`: the PT6 sequence, the jet's
APU/bleed-air `start_engines`, or the piston one-shot. The jet arm was
missing until 2026-07-27 and `turbine_start` airframes fell through to the
piston start: full auto sat at a cold MD-82 announcing "cranking, the engine
hasn't caught, check fuel and mixture" about levers it doesn't have. The
same sortie found the profile's V-speeds missing, which is worse than it
looks because the fallbacks are stall-based — no `vs_clean_kts` means vs
defaults to 60 kt, so the MD-82's rotate came out at 69 knots. With the arm
wired the NEXT sortie found the stage's two remaining piston assumptions
(Luton, 2026-07-27): the wait for the engine was one 40 s figure, but a jet
start is a PROCEDURE — bleed pressure, then crank/fuel/idle on the right
engine, then the same on the left, which is the one full auto watches — so
full auto called "the engine didn't catch, your aircraft" forty seconds in
while the sequence was still waiting on the manifold, and then cranked both
engines for a run that had already ended. The deadline is per airframe CLASS
now (`StartKind::engine_start_secs`, 300 s for the jet). And the power-up ran
through the `PowerUp` tool, which SPAWNS the bring-up and answers
immediately, so full auto walked straight on to the start, which found the
APU still down and started a second copy of the same sequence — two racing
copies both read a cross-tie as open and both fire its toggle, which leaves
it shut. The orchestrator calls `power::run_prep` inline instead: it is
already on its own thread, so waiting is free and "power up, then crank" is
true again),
request-taxi over the crew radio, PUSHBACK when we're nose in, auto taxi,
departure call, line up
CLEARANCE-AWARE (waits out traffic holds; verifies the runway fit before
ever arming the airborne phase), then the MISSION — a circuit, or
(flight=true, "fly my flight plan" / "fly the simbrief flight") the pilot's
actual SimBrief plan: `procedure::enroute_legs` distils the OFP waypoints
to direct-to legs at the plan's own crossing altitudes (climb fixes command
cruise directly so the departure never stair-steps; the post-TOD numbers
fly the descent), the follower flies them under `EndAction::Enroute` (quiet
completion — the orchestrator owns what's next), and the arrival prefers
the destination's published approach (runway picked by the field's surface
wind — `nav::best_runway_for_wind` when it's `>= WIND_DECISIVE_KTS`, else the
end most aligned with the arrival bearing; the visual autoland fallback is
FORCED onto the same end via `run_autoland`'s `forced_rwy`, so a tailwind
can't be re-picked back in by track — a 17 kt tailwind put the arrival on
Gibraltar's 09 while the tower worked 27, 2026-07-24), tuning
COM1 to whoever owns the destination (the enroute region watch has been
handing the flight down the airway) with "inbound for landing" then the
final report at the autoland handoff — the call that earns the landing
clearance. The arming tool fetches the OFP on demand, refuses without a
plan or when the plan departs a field we aren't at, and best-effort loads
the route into the GPS via the FMS mailbox. Cruise POWER is a speed, not a
lever position: the enroute legs arm the flight speed-hold at the
profile's `reference.cruise_ias_kts` (procedure.rs `cruise_ias_kts` —
throttle finds the power that holds the IAS, full while still slow out of
the climb, so it self-adjusts for weight/altitude/temp; pitch owns
altitude; released at every exit so autoland owns the throttle alone).
The number is the AIRFRAME's, never SimBrief's — the plan supplies the
altitudes, the profile the speed (172: 108 KIAS; Baron: 180, still
VERIFY), and a profile without one leaves the throttle at climb power,
which was the only behaviour before this existed. Flying the OFP's own
cruise schedule (cost index / mach / planned TAS) is the part that is
still future work. Everything blackboxes as `"kind":"full_auto"` /
`"circuit"` / `"procedure"`.

Depth (added 2026-07-28): **the approach is sized for the airframe, and a
missed approach is a procedure**. One MD-82 circuit at Luton exposed four
faults in a row, and they compound: the pattern delivered the aircraft LOW,
the vertical law then flew it HIGH, the balked landing had nowhere to go,
and nobody was told. (1) The base leg descended to a flat 700 ft, so the
handover height depended only on how wide the pattern was — the jet's turns
final around 4 nm, where a 3° path is 1,340 ft up, and it arrived 520 ft
BELOW it (`circuit::approach_leg_alt_ft` now descends the PATH, bounded by
the handoff floor below and the pattern altitude above, so it can only ever
delay a descent, never climb on base). A jet also flies a 1,500 ft circuit,
not a trainer's 1,000 (`pattern_alt_agl_ft`), and sets up at
`approach_leg_kts` rather than carrying Vy — 200 kt in this airframe, which
arrived on base at 226 against a 140 kt approach speed. Never faster than
the pattern speed, so GA airframes are untouched. (2) SPEED PROTECTION was
shallower than the path: a fixed 40 fpm per knot capped at 700 is steeper
than a trainer's path and gentler than a jet's, so 9 kt slow commanded 360
fpm against the 740 the path wanted, HALVED the descent, and made the
aircraft slower still — the exact opposite of the intent. It is floored at
the path rate now. The MD-82 crossed the threshold 51 ft high at 152 kt and
overflew. (3) The go-around then had no procedure. The latch released the
instant it was clear of the ground, leaving the aircraft 800 ft over the far
end with the field behind it, and the STATELESS guidance re-decided every
tick: outbound while the path was 500 ft below, inbound as soon as it
wasn't, cycling; and vertically the reposition floor (~1,590 ft AGL) and the
path tracker met at a step, +800 against -1,200. It orbited for three and a
half minutes between 1.1 and 4.1 nm and escaped by luck. `MAX_GO_AROUNDS`
can never catch this — a second miss needs another approach, so the backstop
never armed and nothing gave up. Now `MissedStage` makes it sticky: Climb
straight ahead, then Reposition outbound to `join_point_nm` at platform
height, released only when established out there on the approach side (not
merely outside the FAP, which hands back mid-outbound-leg pointing away from
the field). The reposition leg never chases the path it is flying away from,
and `descent_limit_fpm` fades the permitted descent to zero AT the floor
instead of through it. (4) Nothing was ever transmitted, so the tower held a
landing clearance for an aircraft that was not landing while it worked the
traffic underneath — the FSM sat in ClearedToLand the whole time. The loop
now calls "going around" once per balked landing over the same crew radio
channel the circuit's position reports use, and a test pins the phrase
against `tower::parse_intent`.

Depth (added 2026-08-01, evening): **and autoland's own pursuit lead was
collapsing on short final.** The fixed-lead pursuit added 2026-07-28 (against
"never in line with the runway, always at an angle" at EGSS) was undone by its
own floor: the aim was `(along - lead).max(0.15)`, so inside one lead distance
the aim STOPPED ADVANCING and the effective lead shrank — 0.875 nm at 1.2 nm
out, 0.55 at 0.7, 0.20 at 0.35 — which is the d/2 law the fixed lead exists to
replace, back again exactly where being lined up matters most. At Southampton
the commanded cut sat at 16 degrees from 0.87 nm all the way in to 0.32, then
snapped to the runway course over the numbers with the aircraft still 6 degrees
off and still turning. `pursuit_aim_nm` keeps the full lead all the way in,
which means the aim walks PAST the threshold and on down the runway — not a
trick, since the extended centreline and the runway centreline are one line —
bounded by the pavement so it is always a point on the runway. It is a pure
function now, so the lead staying constant is a test rather than a comment.
The same page closed the **last-ditch bypass**: `ready_for_autoland` returned
true on distance alone before the alignment test, so inside 1,500 m any offset
handed over. The valve existed because the final leg had no intercept and
genuinely could not close one; it has one now, so the valve is for a turn that
went WRONG (`LAST_DITCH_XTK_M`, twice the established tolerance), not for one
that simply has not finished.

Depth (added 2026-08-01, evening): **a leg whose job is to join the centreline
must converge on it.** The final leg commanded the runway course and nothing
else — which makes it a leg that flies PARALLEL to the line it is supposed to
be joining. That was survivable only while the turn onto final was assumed to
roll out ON the line, and it does not. And the reason it rolled out short is
arithmetic, not luck: `AP_BANK_DEG` is a deliberately shallow 14 degrees and
`gs_est` pads the speed by 1.1, both of which make the radius BIGGER — safe for
every dimension they size except this one, where a bigger capture distance
starts the turn EARLIER and rolls out SHORT. The model wanted 1.3 x 1,443 =
1,876 m; the aircraft turned in 1,048 (a 90-degree turn closes exactly one
radius of cross-track, so the trail measures the radius directly); the
predicted residual is 828 m against 776 flown. Back-solved at the real approach
groundspeed the AP banks 15.8 degrees, not 14. So `final_turn_radius_m` sizes
that one capture on the turn actually flown — approach speed, no padding,
`AP_BANK_FINAL_DEG` 15.5 (rounded shallow so it still errs EARLY, because with
an intercept a short roll-out is a gentle closure and a long one is an S-turn
back across the centreline) — which takes the residual from 776 m to 346.
`Geometry::for_radii` keeps the conservative radius for pattern width and
downwind length, where undersizing is what caused the LXGB figure-eights.
`final_capture_m` had been 1.3 turn radii on the assumption that
"rolling out slightly EARLY is free; autoland tracks in": at
Southampton the turn began 1,850 m out and consumed only ~1,050, so it rolled
out **776 m short of the centreline** and then held runway heading all the way
to the handoff, closing almost nothing. "Slightly early" was three quarters of
a radius, and free it was not — autoland inherited the offset on top of an
approach it was already 206 ft high on, flew the diagonal in, and the aircraft
crossed the threshold **still 6 degrees off the runway heading, still
turning**. That is the landing-at-an-angle the 2026-07-30 fix was supposed to
have ended; that fix added a heading test, and a heading test PASSES for an
aircraft flying perfectly parallel 776 m to the side. `final_intercept_hdg`
gives the leg a real intercept — pure pursuit at a point down the centreline,
capped at `MAX_FINAL_CUT_DEG` (30, inside autoland's own 35), held at the cap
until the offset is inside ~520 m and washing out from there, so it converges
like an instrument approach instead of asymptoting (a gentler 2,000 m aim had
faded to 8 degrees by 300 m off and only just made the centreline before it ran
out of runway — the simulated intercept from the live roll-out is a test). And
`ready_for_autoland` now requires being ESTABLISHED, not merely rolled out:
heading AND `ESTABLISHED_XTK_M`. That constant is 300 m rather than something
tighter on purpose — the intercept has to be able to REACH it before the
handoff, and a gate the leg cannot satisfy deadlocks into the last-ditch valve,
which hands over the exact diagonal the gate exists to prevent.

Depth (added 2026-08-01, evening): **the glidepath distance is measured DOWN
the approach course, never across it.** `approach_leg_alt_ft` took track miles
— along-track plus cross-track — and the cross-track term inflates the target
by exactly however far off the centreline the aircraft is, which on a base leg
is the whole leg. Southampton, Baron: base began 3,324 m off the centreline, so
track miles read 3.33 nm and asked for 1,101 ft when the real distance to the
threshold was 1.53 nm and the 3° path was at 529. The aircraft flew base and
most of final LEVEL at 800 ft AGL, the target fell below it only eleven seconds
before the handoff, and autoland was handed an aircraft 206 ft high at 1.7 nm.
It never recovered: still 118 ft high at half a mile, still 80 ft high over the
threshold, crossed at 180 ft AGL and 102 kt, overflew the whole runway and went
around. Plain along-track is wrong in the DANGEROUS direction though — on base
the aircraft is crossing the extended centreline, so its along-track distance
is still GROWING, and aiming at the path for the distance it has now is a dive
toward a point it is about to move away from (529 ft with 3.3 km still to fly).
So `path_dist_m` splits it: base aims at the ROLL-OUT, the path altitude where
the turn onto final will actually deliver us, which the pattern geometry
already knows as `downwind_past_m` (predicted 4,087 m against 3,966 flown);
final follows the real along-track distance down. Which is also just how an
approach works — platform until the path meets you, then down the path. The
2026-07-28 note that introduced track miles was fixing a different thing (a
straight-line HYPOTENUSE growing mid-turn), and the ratchet it also added was
what actually fixed that; the ratchet stays.

Depth (added 2026-08-01, late): **a clean pattern must clear the CLEAN stall,
and the freeze flags were costing frames.** Two loose ends from the same
sortie. (1) The circuit flies clean and the callouts are computed clean, but
the pattern speed had no floor against the clean stall: the Baron's Vy is 105
against 84, i.e. 1.25 Vs, which is FOUR knots above the airspeed caution band
(1.2 Vs = 100.8) — so every normal variation on downwind, base and final dipped
into it and six callouts fired in one circuit, every one of them correct. Flap
was 0.00 throughout, which is right; nothing knew that made the stall reference
84 and not the flapped 70. `pattern_speed_kts` takes `vs_clean_kts` and floors
at `PATTERN_MIN_STALL_MULT` (1.3 Vs), which the approach legs inherit. It only
bites on an airframe whose Vy sits near its clean stall — a 172 (Vy 74, stall
48), an MD-82 (200/100) and a Citation (200/136) are all untouched, and a
profile with no stall figure gets no floor rather than an invented one. Same
family as the climb limiter: a speed that is fine flapped is not fine clean.
(2) The Web API cache gained a per-dataref TTL. The 60 ms window was sized for
DUPLICATION and recovers ~2.5% because the loops read DIFFERENT datarefs;
VOLUME is the problem, and the histogram named it — after latitude and
longitude came `is_in_replay` and `paused`, ~4 reads/s between them for two
flags that change a handful of times in a whole flight. `SLOW_DATAREFS` get
500 ms. Nothing needs them sooner: a pause noticed half a second late costs one
control tick, every loop re-checks each tick, and `sim_frozen` fails open. The
evidence it matters: at 20:42 the sim dropped to **7 fps against a 60 fps
median** in the same 30 s window that logged 76 req/s and a 1,040 ms slowest
response — the API is served from inside X-Plane's flight loop, so that is
frame time the sim does not get.

Depth (added 2026-08-02): **the limiter's full-rate speed is the speed the
climb is FLOWN at, and two same-evening fixes had never met.** The stall floor
above raised the Baron's pattern speed to 109.2 kt — four knots ABOVE its Vy of
105 — and the climb limiter below took that same pattern speed as its full-rate
threshold, while the throttle law holds the aircraft AT it. That parks the
steady operating point on the exact top edge of a nine-knot fade band: full
climb is unreachable while climbing (climbing at Vy is what the airframe
does), and every knot of ordinary jitter — amplified by the 5 s trend
projection, whose swing is as wide as the whole band — slams the granted rate
across half its range. The next morning's circuit porpoised exactly as the
projection was supposed to prevent: cmd_vs 700 to 0 on crosswind with the
Baron DESCENDING at 190 fpm at 670 ft AGL mid-climb-out, then the whole
downwind hung 100 ft below pattern altitude with the capture choked to
0-150 fpm and the speed see-sawing 104-113 on a 25 s cycle (EGHI). The unit
test had pinned `want = 105.0` — Vy — all along; the call site passed
`pattern_kts`, and the four-knot difference between two numbers that used to
be equal was the entire failure. `climb_full_rate_kts` (Vy capped below the
leg speed — a jet's book Vy is a cruise-climb figure and must never RAISE the
threshold — and floored a margin above the fade floor so a degenerate profile
gets a band, not a cliff) is now the one seam, and the test wires it exactly
as the loop does, so the test and the call site can never again agree on
semantics while disagreeing on the number.

The SAME sortie's re-fly then showed the other half (2026-08-02, later): with
the limiter fixed, the downwind climbed clean at the full 700, but one deep
dive-and-recover cycle survived on crosswind — and its driver was the THROTTLE
law, not the limiter. The circuit inherits the takeoff handoff at 131 kt and
full power against a 109 kt target; `throttle_adjust`'s P+D is algebraically a
projection at 2.7 s of lead, shorter than the lever's own slew, so it was
still pulling at 124 kt with the speed already falling 2.5 kt/s toward the
target — the sag ran through to 98.7 kt, ONE KNOT BELOW BLUE LINE, and full
recovery authority only unlocked at the old gate's raw err > 10, i.e. once the
aircraft was already 10 kt slow. The limiter (correctly) cut the climb on the
way through and the aircraft levelled, DESCENDED 65 ft at 700 ft AGL, and
climbed again — the "oscillation" the pilot reported after the first fix. The
law now projects 5 s (`THROTTLE_LEAD_S`) and both escalation gates watch the
PROJECTION: the pull eases while the excess is still burning off, and power
returns before the undershoot — urgency about where the speed will be, not
where it was. Every old behaviour a test pinned (deadband, SF50 runaway pull,
anti-stall reflex) is unchanged; the new test pins the handoff transient
itself.

Depth (added 2026-08-02): **a landing is one continuous manoeuvre, and both of
its rough edges came from treating it as segments.** The Baron's first
successful circuit was firm on and harsh after, and the blackbox names both.
(1) The touchdown: past the threshold the (unsigned) glidepath distance grows
again, alt_err goes negative, and the vertical law's level-hold branch fired at
27 ft OVER THE RUNWAY — the aircraft floated level, bled the very airspeed the
flare needed, and dropped on from the mush at -380 fpm one knot above the clean
stall. A ROUNDOUT band (`ROUNDOUT_MULT` x the flare height, same
pavement/cross-track gates as the flare) now eases the target to
`ROUNDOUT_VS_FPM` (-250) so the -100 flare is entered from a gentle descent
with flying speed — the two wrong answers there, chasing the path rate and
holding level, were both the same missing stage. (2) The rollout: a flat 0.45
brake ratio measured 3.3 m/s² from 79 kt to a DEAD STOP — max-autobrake
territory on a runway with a kilometre to spare — and then the vacate
re-accelerated the aircraft it had just parked. `rollout_brake_ratio` brakes by
wanted deceleration (comfort 1.6 m/s², overridden by the stop-in-what-remains
term, so a short runway still gets everything), and when full auto's taxi-in
follows, the rollout hands off ROLLING at `VACATE_HANDOFF_KTS` (10 kt) straight
into the vacate drive — stopping dead on an active runway to then leave it was
neither procedure nor comfort. The channel is `autoland::VACATE_FOLLOWS`, set
by `taxi_in_after_landing` before the landing, CONSUMED at rollout entry and
cleared on every no-landing path, because a stale process-wide fact is the
radio-power-latch bug in a new coat: a later standalone "land for me" must
brake to a stop exactly as before (nobody follows it). The safety half lives in
taxi_in: any vacate failure after a rolling handoff arrests the roll (parking
brake + blackbox `arrest_rolling`) before the minutes-long taxi request wait —
same rule as pushback, a stage that is not moving the aircraft is holding it.
The radio order was already right and stays: vacate first (completing the
landing clearance we hold — no call owed from the strip), "runway vacated" from
the taxiway, then the taxi request.

Depth (added 2026-08-02, afternoon): **the overflight rule must know a landing
float from an overflight, and a give-up must hand back a flyable aircraft.**
One chained failure, four fixes, from a single balked landing that should not
have been balked. The roundout (same morning's fix) legitimately floats
further down the runway than the old at-path-rate descent ever did, and
`OVERFLIGHT_GO_AROUND_NM`'s fixed 0.15 nm was tuned on the old profile: at
EGHI it declared an overflight at 26.3 ft radio — ONE FOOT above the 25 ft
flare gate — with 1,400 m of runway ahead, and balked a good landing.
`overflight_allowance_nm` makes the question runway-aware when the aircraft is
LOW over the pavement (below the roundout band): float on while
`FLOAT_MIN_REMAINING_M` (900 m) remains, capped at half the runway, floored at
the old 0.15 so short strips keep the old rule — and HIGH past the threshold
keeps the hair-trigger, so the 167 ft overflight and the MD-82's 51-ft float
stay caught. The flare/roundout zone gate stretches with it (`in_flare_zone`),
because past the threshold `dist_nm` is unsigned and the plain 0.35 nm bound
would have ended the flare zone mid-float and resurrected the level-hold bug.
Then the cascade: the missed approach flew its reposition compliantly but
`MAX_MISSED_APPROACH_SECS` (300 s) was SHORTER than the flight the reposition
itself commands (climb to 800, turn back, overfly, out to a ~6 nm join point:
5+ min at 110 kt), so it expired mid-leg — now 600 s. The give-up then set
half throttle and walked away from an aircraft the balk had left with GEAR
DOWN and LANDING FLAP, which cannot hold level flight at 0.5: it decayed 110
to 80 kt at -900 fpm in thirty seconds. Give-ups now hand back climb power
(`GIVE_UP_THROTTLE` 0.75) with the VS dial zeroed and a line that says what is
actually held ("the autopilot is holding heading and altitude, gear is down" —
the handoff-truth rule again). And the pilot's panic FULL AUTO at 1,475 ft
descending then walked the GROUND chain — a stale `LAST_ARRIVED` from the
morning's taxi read as "at the hold" and it requested a departure clearance
from a sinking aircraft — so full auto now refuses to arm airborne and names
the tools that fly ("say do a circuit, fly the flight plan, or land for me").

Depth (added 2026-08-01): **a climb is never worth more than the airspeed left,
and landing includes getting off the runway.** The Baron 58's first full-auto
circuit reached the gate at Southampton, and found two things the 172 never
could. (1) The autopilot's vertical mode buys rate with knots and will keep
buying: rolled into the crosswind turn at 127 kt it came out at 85 — one knot
over its clean stall and one over Vmca — with the VS law still asking for
880 fpm and the stabilizer trimmed to the nose-up stop. Six airspeed callouts
fired across the pattern and every one was right. The throttle law cannot fix
this alone; it was already commanding more power, and in a climbing turn on a
piston twin there is none left. `circuit::climb_limit_fpm` fades the commanded
CLIMB to zero as the speed falls from the pattern speed to `climb_floor_kts` —
blue line where the airframe publishes one (a twin below Vyse has no
single-engine climb at all), else 1.15x the clean stall. Full rate AT the
pattern speed, so an aircraft flown correctly is never penalised; descents pass
through untouched; a profile with neither reference speed gets 0 and no
protection rather than an invented number, which is why every GA airframe is
unchanged (a 172's floor is 55 kt against a Vy of 74 — it never meets the
limiter). (2) Autoland stops ON the strip and hands back, and for a CIRCUIT
nothing came after it: full auto armed the circuit and EXITED, so the aircraft
sat where it stopped. It held runway 02 for ninety seconds with an arrival
closing 2.6 nm to 0.5 — six `runway_incursion` records and the tower telling
the lander "continue approach, traffic on the runway ahead" — and only moved
because the pilot said "taxi to parking". `taxi_in_after_landing` is now the
last stage of BOTH missions (the flight path already had it), so full auto
stays ACTIVE through the circuit and vacates.

**And vacating asks nobody** (same day, the second half of that fix).
[src/taxi_in.rs](src/taxi_in.rs) is two phases now — VACATE, then taxi — because
the original had real procedure backwards: it transmitted the taxi-in request
from wherever autoland stopped and waited up to sixty seconds for a stand
route, sitting on the strip the whole time, with the give-up path handing back
"your aircraft" still on the runway. Clearing the runway is the COMPLETION of
the landing clearance we already hold, not a new instruction: no controller in
the loop, no readback owed, and the call ("runway vacated") goes out from the
taxiway afterwards, which is the order a real crew flies.
`nav::plan_vacate` reuses the departure machinery rather than inventing a
second idea of where a runway meets a taxiway — `zone_of` and the hold-short
trim, run BACKWARDS (the trim drops trailing edges inside the runway's
protected zone so a departure stops short; the vacate keeps going until it is
`VACATE_CLEAR_M` past the same boundary, because the tail has to be over it
too) — with two things that genuinely differ. Metres covered while still INSIDE
the zone earn no credit toward that clearance, or the drive finishes at the far
end of the entry link, which by the trim's own logic IS the holding point, i.e.
exactly the line we are trying to get past. And an exit is NOT an entrance:
`runway_entrance_near` accepts a node one hop off the runway (a departure holds
there and the drawn lead-on carries it on), but an arrival must decide at the
JUNCTION — measuring "is this exit ahead of me" at the far end of a link says
nothing about where that link meets the strip, and a junction 22 m behind the
aircraft reads as an exit 150 m off to the side. Exits behind the rollout are
refused outright: reaching one means taxiing back down the runway. Thin data is
never a refusal (no zones, no runway edges — it degrades; only a field with no
network at all declines), the drive arms through the same governor front door
as the auto taxi and the line-up, and every failure falls through to the
taxi-in exactly as it behaved before. Reached from the crew side through
`party_line::plan_vacate` — the one accessor there that is not a published
clearance, deliberately: the world owns the airport's geometry and the crew
should not keep a second copy, but this returns pavement, not permission.
Gated on a STANDING landing clearance (`taxi_in::vacate_runway`), which holds
over exactly the window between touchdown and asking for the stand — reading
`clearance_runway` alone would hand a stale ident to a drive aiming for a
runway exit from the middle of a taxiway.

Depth (added 2026-07-27): **pushback** ([src/pushback.rs](src/pushback.rs)) is
the chain's first metres, between the taxi clearance and the drive. The
trigger is the CLEARED ROUTE, not a stand database or a per-airframe flag:
the first route point that isn't the stand itself is the taxi lane, and its
bearing sitting past the wing line (`BEHIND_DEG` 100) is what nose-in MEANS —
a tie-down you can taxi straight out of plans `None` and never calls a tug.
The push is STEERED, so it ends on the lane pointing DOWN it (the finish
heading is the lane's own direction, node to next node, never the reciprocal
of our push — a stand entered at an angle would otherwise leave us aimed
across the taxiway) and the airliner never has to turn itself around on the
apron. But steered means CHOSEN, not flown: X-Plane's three pushback commands
are whole manoeuvres the tug executes and finishes, not a rudder — read
Commands.txt, "Nose 90 left" / "Straight back" / "Nose 90 right", and
`pushback_stop` is "stop and LET GO". So the pattern is picked ONCE from the
initial heading error and re-issued, never re-steered per tick. The first
version treated the three as a steering channel and the trap sprang
immediately: it picked `left` correctly, then its own twelve-second nudge
sent `straight` unconditionally, which RE-AIMED the tug — and since the
tracked command still read Left, nothing ever sent Left again. A Gatwick
MD-82 was pushed 51 m dead straight with its heading unchanged to a tenth of
a degree, 115 degrees off the lane (2026-07-27). Two more things fall out of
the same semantics: the command issued before a tug exists only SUMMONS one
(the push began only after the nudge re-issued it once hooked), so the
pattern is sent again on hook-up; and a tug that stops having moved has
FINISHED, which is why `STALL_SECS` ends the push — motionless, the same
sortie sat out 203 further seconds to `MAX_SECS`. Short of the plan the tug
is asked for another pattern, bounded by `MAX_ISSUES`. A pushback command is ONE
complete manoeuvre, and COMPLETION INCLUDES LETTING GO of the bar: the
Stansted trail (2026-07-28) shows "Nose 90 left" executed to the letter —
err pinned at −98.5 for 30 m of straight push, a clean ~90° arc, bar off —
with nothing sent after the initial issue, so "attached dropped after we
rolled" is the manoeuvre's normal END, not a tug abandoning us (three pushes
were failed as `tug_released` for finishing correctly). The release handler
now judges completion (aligned within `ALIGN_DEG`, or `done()`); short of the
plan it issues the NEXT pattern — which summons the tug back to the bar (it
is standing at the nose; `REHOOK_SECS` bounds the wait) — bounded by
`MAX_ISSUES`. Which way `pushback_left` swings the nose is still LEARNED per
session (`sense_is_wrong` flips once, `LEARNED_SENSE` keeps the verdict), but
the DEFAULT guess is now the measured one: on X-Plane 12.4.3 `pushback_left`
swings the nose CLOCKWISE, opposite to its "Nose 90 left" description —
three pushes, three identical full-turn trails. A correction is never sent
mid-manoeuvre (that re-aims the tug); the release handler and the hook-up
re-issue are the moments a pattern takes. Brakes are the stage's own too:
SET for the tug wait (full auto's early stage-2 release left an idle jet
brakeless for the minutes-long wait and it crept into the grass, same day —
the release is deleted; the taxi governor, pushback and auto-takeoff each
release at their own moment), released at hook-up, re-set at disconnect and
on the mid-apron failure exits. Every tug
instruction goes through the
same `pushback` tool a spoken "push back left" reaches — one owner for the
ground-ops commands. The tug handshake is ONE dataref plus our own position:
`pushback_attached` is the bar, and "under tow" is measured by having ROLLED
(`MOVED_M`). `sim/flightmodel2/misc/tow_in_progress` sounds like the second
half of that handshake and is not — probed live at a stand with the aircraft
rolling backwards under a tug, it read 0 from hook-up to release
(2026-07-27). Everything that had been gated on it was therefore dead code
with a plausible story attached: the push never believed it was moving, so
the tug-released check could never fire and the nudge went out at twelve
seconds on every normal push; the wait-for-the-tug-to-clear before taxiing
returned true on its first read with the tug still on the nose; and the
monitor's callouts, which CLAUDE.md claimed narrated the push, spoke only
"tug connected" — never "pushing back" or "tug clear". The dataref is still
read everywhere it was, as extra evidence only, and the per-tick `push`
events now carry `att`/`tow` so the next sortie can re-settle it. Clear is
the BAR coming off, never the rolling stopping, or a pause mid-push
announces a tug that is still hooked on. The stage is INLINE in full auto, not a spawned
sub-loop like the taxi governor — nothing else can arm a pushback, so there
is no second owner to arbitrate with — and it is deliberately BEST EFFORT: no
tug, a timeout, or a tug that lets go early falls back to the taxi join's own
turn-around, which is exactly the behaviour that shipped before this stage
existed. Which failure it was has to reach the pilot, though: full auto spoke
ONE line for all of them, so a tug that came, hooked on and pushed us fifty
metres was reported as "no tug came" by a copilot the pilot was watching it
through the window (2026-07-27) — the failure line is per-reason now.
Narration is otherwise the MONITOR's (its pushback watch calls the tug
connecting, the push starting, and the tug clearing), so the module speaks
only what that watch cannot know: the ask, and the failures. Laws
battle-test as plain `cargo test`; events blackbox as `"kind":"pushback"`
(the arm log carries the stand position AND the lane node, because "no tug
came" is undiagnosable without them; per-tick `push` events carry
back_m/err/steer/sense). First live push (Gatwick, Twin Otter, 2026-07-27)
proved the trigger and the fallback — nose-in at 141 degrees off the route,
turned around under its own steam — and convicted the TUG WAIT: the tug is a
vehicle crossing an airport, not a dataref flipping (X-Plane searches 1500 m
for one and lets it drive 6 km:
`sim/private/controls/trucks/max_search_radius_m` / `max_drive_distance_m`),
and a 30 s limit gave up on a tug that was still driving. The wait is 180 s
now, with a crew line at 30 s so a stand-sitting aircraft never reads as a
hang, and the give-up still cancels the request so a late tug can't hook onto
an aircraft that has started taxiing. A tug DOES come: watched live at a
Gatwick stand the same evening, `pushback_attached` flipped to 1 and the
aircraft was rolling three seconds later — well inside 180 s, well outside
30 s. The SENSE CLOCK ran on the wrong thing until 2026-07-28: `CAL_SECS`
was measured from the ASK, and a verdict of "sense is fine" LATCHED. At
Luton the tug hooked on 90 s after the request and did not roll for another
74, so the four-second window expired in that dead air, compared 64.6
degrees against an unchanged 64.6, and closed the case. The push then swung
the nose ninety degrees the WRONG way over fifty-five metres with no
recourse left and the tug let go 155 degrees off the lane, which full auto
reported as `pushback_failed`. `sense_is_wrong` now gates on being under
tow, and the verdict is re-judged EVERY tick and acted on once (a tug that
reverses straight back before it starts swinging would otherwise spend the
whole window showing an unchanged error and buy a wrong guess a free pass).
`sense_learned` had never fired once across every push in the log.

**Progress along a push is a SIGNED quantity, and the brakes belong to whoever
is not holding the aircraft** (2026-08-01, Southampton, one Baron and one
sortie). `back_m` was the plain haversine distance from the stand, which is a
MAGNITUDE — the same trap the taxi guide hit when four knots backwards read as
four forwards, one module over. The tug hooked on and never pushed; the
aircraft, brakes off for the hook-up and both engines idling, crept FORWARD off
the stand. That creep counted as pushback progress, crossed `MOVED_M`, latched
`towed`, and with the attach flag reading false the loop concluded the tug had
let go after pushing us 1.2 m. `back_along` now projects displacement onto the
reverse of the STARTING heading (start, not current: a "Nose 90" pattern puts
the nose across the displacement, but the displacement is still behind where
the nose began), and rolling forward past `CREEP_M` with nobody towing ends the
push, brakes on, as `creeping`. The second half is that the re-hook wait ran
its full minute BRAKELESS — the hook-up release rule ("the first moment
releasing them is safe is when something is holding the aircraft") was written
for the tug wait in July and not applied one state later, so the Baron
accelerated to 21 kt across the apron while the module believed it was standing
still, the tower berated us for leaving the cleared route, and the pilot
stopped it by voice. Brakes now go on before the re-hook wait and come off
again on the re-hook, and the ABORT path sets them too — it only ever stopped
the TUG. Every one of these is the same rule: a stage that is not moving the
aircraft is holding it.

**And the creep guard then killed the pushes it was guarding** (2026-08-02,
same stand, three in a row). The tug's dead air is MINUTES (Luton: hooked for
74 s before rolling), the brakes come off at the hook-up, and a Baron at idle
walks forward in seconds — so every push ran: hooked, brakes released, forward
3 m in four seconds, `creeping`, dead, before the tug ever moved. Failing on
the creep was the wrong verb; the right one is HOLD: `should_hold_creep`
squeezes the expiring proportional brake channel (`xcopilot/taxi_brake`,
`HOLD_BRAKE` 0.4, from `HOLD_AT_M` 0.3 m of forward drift) every tick while
nobody is pushing — it arrests the walk this tick and frees itself in a
second, so a tug beginning its manoeuvre never fights a latched brake, and it
is never applied once towing/towed/attached says a push is under way (a "Nose
90" arc can carry the reference point forward mid-swing). `CREEP_M` survives
as the backstop for a plugin without the channel. The same three pushes showed
the attach flag BLIPPING once in the hook wait and reading false from the
first push tick on, so `saw_attached` now gates the bar-came-off branch: a bar
never seen ON cannot come off, and on such an airframe a real push ends
through the movement paths (`done()`, or stall-after-moving = completion)
rather than being declared released `MOVED_M` metres in and re-aimed
mid-manoeuvre. And the blip itself is now treated as what it is (2026-08-02,
after the pilot's ground truth "of course push works" — the sim's tug DOES
push this aircraft when the brakes are held until it is really connected):
CONNECTED means connected, not glimpsed. The hook wait requires the
attach/tow flag up for `HOOK_SOLID_POLLS` consecutive polls before it counts,
so the brakes stay ON through the tug's positioning instead of releasing on a
one-poll flicker, and the pattern is issued BEFORE the release — the aircraft
is never brakeless with a tug that has no instruction yet. The
still-unanswered question is narrower now: whether the flag goes solid once
the tug finishes positioning (the pilot's experience says it will), and the
next sortie's blackbox answers it.

Depth (added 2026-07-28, evening): four traps of the same family — a fact one
part of the system knows and another needs. The taxi GUIDE narrated a
**pushback** ("Rolling again. Sharp left next, onto Bravo" with the tug
connected): groundspeed is a MAGNITUDE, so four knots backwards reads exactly
like four forwards, and the stand IS the cleared route's first point. It gates
on `pushback::ACTIVE` now, and on the route itself — a fix whose along-track
progress went BACKWARDS is never a moment for "next left". **Full auto** polled
`autotaxi::ACTIVE` to wait out a drive, but a BACKTRACK drops that flag in the
gap between its two legs; it read the gap as "finished" and called a running
line-up a failure one second after leg two engaged. `driving()` (ACTIVE OR
`BACKTRACKING`) is what a caller waits on. The ground drives (`auto_taxi`,
`line_up`, `taxi_to_parking`) were registered behind `!autothrottle` alongside
`hold_speed` — right for the AIRBORNE speed tool, since an airliner sets its
speed bug, and wrong for these: on the ground the taxi governor is the only
speed control there is. Neither jet had a taxi tool at all, and only full auto
worked because it builds its own copies. And one PANEL fact: a light can be two
switches (`also_datarefs`) and a system's only control can be a TOGGLE
(`write_datarefs`) — the Citation X's landing lights are independent left and
right, and firing `cmd_avionics_toggle` as "on" turns the avionics OFF whenever
they were already on, which is worse than having no tool.

Related trap (2026-07-24): `Voice::alert` now
mirrors spoken lines to the panel transcript itself — any watch that ALSO
posts `s.action(same line)` double-writes the chat (the Jerez relocation
note; gyro-sync and autoland hit it first). Mirror OR action, never both.

**The panel is an automation instrument now** (2026-07-29 revamp). Tabs are
Flight | Transcript | ATC | Route | Settings — Flight first and default: the live
strip, an automation status well fed by `xcopilot/status/auto` (composed by
[src/auto_status.rs](src/auto_status.rs): marker byte '1' = a manager is
actually flying/driving (the AUTO annunciator; since the audit of 2026-08-02
CANCEL renders unconditionally plugin-side, because the "my controls"
fallback made cancellable always-true and the chip lit amber parked
cold-and-dark), then
one line per active manager, primary as "NAME|stage"; `full_auto::event()`
stores its stage string, and the mission is folded into the name so the
button pressed matches the headline: "FULL AUTO CIRCUIT [request taxi]" /
"FULL AUTO FLIGHT [...]"),
published change-gated from the monitor tick — NOT the 1 Hz radio thread,
which only exists when ATC is configured — with an unconditional resend every
5 ticks because a plugin reload zeroes the buffers and a change-gate would
leave the tab blank. Below the well, BUTTONS for every automation
(FULL AUTO / FLY PLAN / PUSH BACK / TAXI / LINE UP / TAKEOFF / CIRCUIT /
LAND + guarded RECOVER / MY CONTROLS): each injects the same English a pilot
would speak through the request channel (`auto_status::REQUEST_PHRASES`,
codes 4-10; a grammar test pins every phrase to its tool so a button can
never reach the LLM), and CANCEL (code 11) has no fixed phrase — the app
picks per what's running ("cancel full auto" outranks all, its abort
cascades), falling back to "my controls" when NOTHING is running
(2026-08-02): the old None fell exactly when the pilot most needed the button
— after the missed-approach give-up every ACTIVE flag was already false while
the AP still flew the aircraft dirty and slow, and CANCEL was silently
consumed with nothing to say. An abort that goes dead precisely when things
have gone wrong is worse than none; CANCEL always aborts SOMETHING now, and
the fallback is the universal hand-back (autopilot off, every hold released).
Mechanics worth knowing when diagnosing "the button did nothing": presses
ride the same typed-command mpsc queue the keyboard uses, consumed one per
session-loop pass, so presses fire in order and a press can wait out a busy
loop — but the phrase (including the cancel pick) is resolved at PRESS time
in the poller thread, so a queued CANCEL still cancels what was running when
the pilot pressed it. The old Calls tab folded into ATC (coach block on top, radio
chips, stations, phraseology reference in CollapsingHeaders; the
readback-owed dot moved to the ATC tab); the shared header rows are gone —
each tab owns its space, and the status bar gained an AUTO annunciator lit
from any tab. Type is B612 (the Airbus cockpit face, OFL — TTFs +
license in plugin/third_party/b612, embedded via xxd headers), baked at
THREE sizes because the ImGui atlas binds at the first ImgWindow ctor and
can never rebake: UI scale (Settings radios, `xcopilot/opt/ui_scale`,
publish/pick protocol, persisted via prefs.json) is a font SWAP plus
`ScaleAllSizes` from a stored base style, never an atlas touch. The B612
glyph ranges include en/em dash and bullet; the app-side dash scrub stays
for old plugins.

**The terminal is the OTHER screen: setup and diagnosis** (2026-07-31). Once
the panel became the automation instrument, the desktop dashboard
([src/ui.rs](src/ui.rs)) was flying in formation with it: an attitude strip,
a radio box and an automation list, all repeating what the cockpit and the
panel already showed, on a screen nobody looks at mid-manoeuvre. The split is
by what each surface CAN do. The panel owns act-and-glance. The terminal owns
what the panel structurally cannot: whether every dependency is actually up,
what we are costing the sim, and what was said. Tabs are Health (default) and
Transcript, with Settings and a blackbox Events tail to follow. The aircraft
strip and radio box are gone, and with them nine dataref reads a second that
were charged to X-Plane's own frame loop for a display that told the pilot
nothing the windscreen did not; the poll thread now costs ONE plugin probe.
Health reads facts that were previously discoverable only by grepping the boot
log, three of which have each cost a sortie: the Web API load window, read
live through `sim::xplane::load_snapshot` rather than waiting for the 30 s
report (the slowest response climbing is the shape a freeze arrives in);
whether the plugin is loaded at all, probed by reading `xcopilot/ptt_active`,
since the app never links the SDK and the datarefs existing is the only
evidence available; and the EFFECTIVE TTS engine (`voice::pilot_engine`),
which is what the speaker thread built after every fallback, not what the
config asked for. A model that fails to load drops the copilot to macOS `say`,
which is audible and otherwise diagnosable only in the boot log. COM1 stays,
as a DIAGNOSIS and not a second radio: the frequency next to the service it
belongs to, or amber when it is on none of them, which is the whole of "why is
nobody answering". The screen renders under `TestBackend` in plain
`cargo test`, so the layout arithmetic (a transcript scrolled past both ends,
a 20x6 terminal) is pinned rather than found in the cockpit.

**Settings are a registry, and the live value lives in one place**
(2026-07-31). Forty-six `XCOPILOT_*` variables were readable only at boot, and
exactly four choices reached prefs.json, so tuning a mic gate by ear cost a
restart per attempt. The block was never per-knob plumbing: `Config` was built
once by `from_env` and CLONED into each thread, which made every field a boot
constant even where its consumer already re-read it every tick.
[src/settings.rs](src/settings.rs) holds it in an `Arc<Config>` instead, and a
consumer takes a fresh `settings::live()` handle at the top of its loop (one
line in the monitor, one in the session loop) — copy-on-write, so a tick that
started before an edit finishes on the snapshot it began with rather than
seeing half of one. That single change made the taxi guide, the party line, the
confirm gate, the confidence bands, the LLM fallback and the SimBrief lookup
live, because they were all already per-tick reads. The knobs that are NOT read
per tick get pushed instead: the mic trio lives inside the open audio device
(`Listener::retune`, `Segmenter::retune`, and `PttMic`'s gate tail became an
atomic so a shared reference can retune it), pushed once between utterances by
`CaptureTuning`, and readback enforcement lives inside the tower FSM, so it is
re-pushed per tick rather than only at station install. A knob is DECLARED once
in the registry (name, pinning variable, how to render, validate, step, apply
and persist), and the Settings screen is a view of that list, never a second
copy of the schema — which is how the in-sim panel's three bespoke pickers can
move onto it later without inventing a third precedence chain. The rule that
matters is the pin: precedence is env > saved pref > default, so a knob whose
variable is set (including from `.env`, which dotenvy has already loaded) is
REFUSED with the variable named on screen. Accepting that edit and letting the
next boot silently revert it is the "always end up back with ryan" bug wearing
a new face, and a settings screen is exactly where it would happen again.
Values persist to a generic `settings` map in prefs.json, with the four
existing typed fields left alone — mode and cabin crew are `Persist::Own`,
since the panel protocol already owns their boot path, and a second one here
would be two precedence chains for one value. Stored forms carry no unit
(a "900 ms" its own parser rejects would forget itself at the next boot, which
a test pins), and a value outside the declared range — legal, because env vars
are parsed by config.rs and not by the registry — renders fine and is pulled in
by the first keypress.

**Web API load is a sim cost, and it is measured** (2026-07-28). The API is
served from inside X-Plane's flight loop, so every request is frame time the
sim does not get. Two hangs that day could not be attributed either way,
because the logs recorded only FAILURES — nothing ever counted the requests
that succeeded. `crates/sim/src/xplane.rs` now reports rate, cache hits,
failures and the slowest response every 30 s, to the log and the blackbox
(`"kind":"api_load"`). The first numbers were 70-80 req/s sustained, with the
per-window slowest response climbing 50 → 100 → 125 ms in the minutes before a
freeze: the shape of load, not of a single event. Reads are served from a 60 ms
value cache (shorter than any control tick; a write drops its own entry so the
readback every tool's trust rests on still sees the real post-write value) —
but the DUPLICATION that cache was built for is NOT where the volume comes
from. Measured, it recovers about 2.5%, because the loops each read DIFFERENT
datarefs rather than the same ones twice, and a whole EGHI sortie logged
`cached=0` in every 30 s window. Do not read that zero as a broken cache; it
is the honest answer, and it is why the load report also records a per-dataref
histogram to the blackbox (`top`), which is the only thing that can actually
name a hog. That sortie's: latitude and longitude 7,312 each, then
`is_in_replay` 3,803 and `paused` 3,791 (the freeze check, ~4/s between them,
for two flags that change almost never — the obvious candidate for a longer
per-dataref TTL), `onground_any` 3,723, elevation 3,375, groundspeed 3,070,
psi 2,877, over 61 windows. Whether any of this causes the hangs is still
unknown: the per-window slowest response reached 4,533 ms mid-flight, and the
plugin (in-process, drawing every frame, never profiled) and LiveTraffic remain
the other two suspects.

Battle-test surface (all deterministic, in `cargo test --workspace` unless
noted — plain `cargo test` at the root only runs the crew package now): the
state-by-intent matrix test in [crates/atc/src/tower.rs](crates/atc/src/tower.rs); the adversarial
transcript corpus [crates/atc/bench/atc_corpus.yaml](crates/atc/bench/atc_corpus.yaml) (real whisper
output mined from flight logs — add every new wart here); scripted sortie
dialogues over the pure routing layer (`route_keyed`/`route_unkeyed`) in
[crates/atc/src/station.rs](crates/atc/src/station.rs); the spoken-call eval judged on intent
accuracy (`cargo run -- stt-eval bench/atc_stt_cases.yaml`); the cross-airport
world sweep (`cargo test --release -p atc -- --ignored atc_world_sweep`, needs
the sim's apt.dat); and the in-sim soak checklist
[docs/atc-chaos-card.md](docs/atc-chaos-card.md). Every routing decision is
blackboxed as a `"kind":"atc"` event — mine `logs/blackbox.jsonl` after each
flight. Safety property: a mishear may earn "say again", never a clearance.
The tracing filter must name EVERY workspace crate
([src/main.rs](src/main.rs) defaults to
`xvoicelayer=info,xvl=info,atc=info,sim=info` — and the crew needs BOTH of its
names, because a line's default target is `module_path!()`, which is the BIN
name for code in a binary and the PACKAGE name for code in the lib. Naming only
the package dropped all 158 default-target sites, the startup/build line
included, while the explicit `target: "xvoicelayer::..."` lines kept arriving
and made the log look healthy, 2026-07-30. Renaming either must move this):
after the crate split a crew-only default silently dropped the whole world
side, so "release watch armed" and "wake clock started" never reached
`logs/` and the Heathrow hold above had to be diagnosed by probing the live
sim instead of grepping (2026-07-27).

History: ATC started as a separate binary ([src/bin/atc.rs](src/bin/atc.rs),
still buildable — tower + ATIS glue around the same lib modules) on the "the
world is a different process" theory. Flying it proved the split premature:
mic/PTT/voice are single shared resources and the second terminal was
unusable in the cockpit. The right split landed 2026-07-14 as a **compile-time
boundary, not a second process**: the world lives in the `atc` workspace crate
(`crates/atc` — station, tower, nav, atis, party_line, speaker identity), both
sides stand on `crates/sim` (Web API client, dataref names, blackbox), and the
copilot embeds the world as a library — still one process, one mic, one PTT.
The atc crate returns `(text, StationSpeaker)` and never speaks or actuates —
its sim handle is `sim::xplane::SimView`, a read-only view with no writes on
it, so "the world never touches the controls" is a type, not a promise; its
dep tree must stay free of whisper/cpal/ort (`cargo tree -p atc` is the
standalone-product health check, and a future ATC-only product is a repo
extraction away). Next: live-traffic
observation via TCAS datarefs (ATC only ever *controls the player* and
*narrates* other traffic), a radio-PTT plugin dataref if content routing ever
collides.

## App modes: Both / Copilot only / ATC only (runtime-toggleable)

The two halves sell separately (2026-07-14): `mode.rs` holds one process-wide
mode — **both** (default), **copilot** (no simulated ATC: no station ever
loads, radio-keyed speech is IGNORED because real/external ATC owns the
frequency — no void briefing talking over a VATSIM controller), and **atc**
(the world without the crew: deterministic tower/ATIS answer the radio, no
intercom persona, no tools, and the LLM is never touched — an ATC-only boot
needs no Ollama at all). Chosen from the plugin's Settings tab
(`xcopilot/opt/mode`, radio buttons; `spawn_mode_poller` syncs it), persisted
via prefs.json; `XCOPILOT_MODE` env sets only the BOOT value (env > saved pref
> both), a panel flip wins from that moment and applies live within ~2 s (the
main loop's idle tick reconciles). Live transitions: ATC off =
`Station::uninstall()` (retires ALL published world state — party line + side
state, the same page install() writes) + radio-channel cut; ATC on = the same
`install_home_station` path boot uses (background scans guard against paying
boot costs twice — `region_loaded()`); copilot off = intercom cut + Ollama
evict (`Llm::evict`, keep_alive 0 — reclaims the GPU frames); copilot on =
re-warmup off the critical path. Monitor watches split along the same seam:
crew watches (callouts, debrief speech, taxi guide, landing coach, talk-down)
gate on `mode::copilot_enabled()`; world ticks are inert once uninstall clears
their state, except `region_tick` which gates explicitly (its airport index
survives for cheap re-enable). In ATC-only, content routing still reaches the
station when no radio key is bound (it's the only path to ATC there), and the
crew speaks exactly one line ("copilot is off") on the first intercom attempt.
The atc crate itself never reads the mode — the world stays mode-blind and is
simply not driven when off.

Depth (added 2026-07-15): the "no crew in ATC-only" rule moved from scattered
per-site gates into the arbiter itself — `Voice::alert` / `alert_stale_by` /
`chime` (and the cabin paths) drop at the one enqueue door when
`copilot_enabled()` is false, so no background watch can forget the gate (the
void-transmission "No answer there" brief, the carry-forward brief, gyro sync,
and the crew branches of the world watches all leaked before). Blocking `say`
is NOT gated there: the main loop mode-routes it, and the one-line "copilot is
off" note must speak. The gyro auto-sync ACTION (not just its line) also
stands down: no crew, no hands on the panel. Tests that set the process-wide
mode hold `mode::TEST_LOCK`.

## Cabin crew: the second intercom persona (2026-07-15)

Channels are physics, personas are tenants: the intercom now carries TWO
voices — the copilot and the cabin crew ([src/cabin.rs](src/cabin.rs)), with
no third PTT and no third audio channel. The address IS the routing ("cabin
crew, prepare for landing" — an explicit "cabin crew"/"flight attendants"
prefix always routes; bare "cabin" routes only with an unmistakable crew word,
because cabin lights/pressure/the Lancair's cabin dump switch are copilot
business). Calls are answered deterministically (keyword intents, "say again"
over guessing — the tower's safety property), always preceded by the
interphone chime, in their own voice (`XCOPILOT_CABIN_VOICE`, lazily loaded so
crewless flights never pay for the model). The crew exists per-airframe
(profile `cabin_crew: true` — the MD-82 only, so far); staffing is a panel
setting like the app mode (Settings tab radios Auto/Aboard/Off →
`xcopilot/opt/cabin_crew`, synced by the settings poller, persisted via
prefs.json, applied live at the routing sites — staff a 172 mid-flight);
`XCOPILOT_CABIN_CREW` sets only the BOOT value (env > saved pref > auto).
Behavior: "prepare
for departure/landing" is acknowledged then owes a delayed "cabin secure"
report (spoken by the monitor's cabin watch — a SAFETY report, exempt from
sterile cockpit); the seatbelt-sign watch (the annunciator dataref, so auto
modes count) calls up only when the sign interrupts a running service AND
above the sterile floor (10,000 ft AGL, `STERILE_FLOOR_AGL_FT`); climbing
through the floor releases the crew to the service. Cabin speech rides the
intercom queue, so it inherits crew discipline, PTT mic hygiene, and the
ATC-only mode gate for free. On a crewless airframe an addressed call gets the
copilot's honest "no cabin crew on this airframe", never an LLM improvising a
flight attendant. Everything blackboxes as `"kind":"cabin"`. Parked: a
physical mic-selector dataref (selector-beats-content precedence, same rule as
the radio key), PA announcements, and folding the mode enum into a tenant set.

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

**`GPL-3.0-or-later`** (2026-07-30, `LICENSE` at the root, declared once in
`[workspace.package]` and inherited by every crate). Two things are now settled
and not open questions: this is **open source, never commercial**, and it is
**X-Plane only** — no MSFS port, whatever the incumbents' platform reach. Only
the product NAME stays parked.

Copyleft over the dual `MIT OR Apache-2.0` the Rust convention would suggest,
because the threat here is specific and demonstrated: the flight-sim ecosystem
is payware-dominated, and Skylink Simulations sells this exact product shape
(deterministic ATC + voice copilot + cabin crew) for $49. Under a permissive
license the tower FSM, the taxi planner, the wake-separation logic and the
ground-drive laws could be lifted straight into closed payware. The usual
argument FOR permissive is commercial adoption, which is a LIBRARY concern —
this is an end-user application, so it costs us nothing to refuse. It is also
the reversible direction: a sole author can always relicense GPL code as
permissive later, but MIT grants already handed out can never be clawed back.
Two bonuses: `3.0` specifically because several deps are Apache-2.0 (compatible
with GPLv3, NOT GPLv2), and matching Welly's ATC now makes his IFR/live-ATIS
code legally liftable, where the old rule was patterns-yes-code-no.

The linking question people assume blocks this doesn't: `plugin/SDK/license.txt`
is plain MIT/BSD, not a proprietary SDK license — as are Dear ImGui (MIT) and
ImgWindow (BSD-3-Clause); B612 stays OFL. All GPL-compatible, all keep their own
terms. Sole-author relicensing stays free only until outside contributions are
merged — settle any change of heart BEFORE accepting a PR. One consequence to
keep in mind: `crates/atc` is built for extraction, and GPL means any future
consumer of it must be GPL too — `LGPL-3.0` on that one crate is the escape
hatch if it should ever be broadly embeddable, deliberately not done now.
