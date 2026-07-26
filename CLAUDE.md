# x-copilot

A voice-commanded copilot for X-Plane. The pilot speaks a command, an LLM picks
the matching action, the agent writes the X-Plane dataref, and the copilot reads
the action back aloud.

```
🎙️  "Flaps up"   →  STT  →  LLM picks a tool  →  write dataref  →  TTS "Flaps up!"
```

`x-copilot` is the **working name** (scaffolding). Naming is **parked**
(2026-07-11); the leading candidate is **Roger** — it encodes the whole thesis:
you speak, it acts, it confirms ("Roger, flaps up"). Rename before going public.

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
| TTS readback | shell out to `piper`, or run Kokoro ONNX via the `ort` crate later |

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
watch owes the re-clearance); **wake turbulence** (a 2-minute clock started
by an airline-prefixed ghost lifting off our strip — TCAS carries no type,
so airline callsign = medium-or-heavier — which turns "cleared for takeoff"
into LUAW and blocks the hold release); and **named taxi routes** (apt.dat
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
from the compliance watch via `TAXI_ROUTE`). The coach's first suggestion is
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
procedural: "say again" / readback enforcement, never an auto-repeat. The
barge-in cut kills by REGISTERED pid + a PER-CHANNEL cut generation
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
works with the panel dark. And ONE frequency at a time: monitor-side
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
or with no published clearance. The governor evaporating
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
Steering on a castoring nosewheel (SR22) is untested — yoke_heading does
little there at low speed; differential-brake steering is future work.

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
Related trap fixed the same day: `looks_like_command` (the continuous-mode
domain gate) matched per-token keywords only, so the split compounds
"take off ..." and "line us up" were swallowed with no miss chime —
bigrams are checked on the raw text now.

**Line up** (same day) bridges the two: "line us up" / "line me up" /
bare "line up" (the LUAW readback always carries "wait", "lining up" is
the short echo — both excluded) drives from the hold-short position onto
the runway and stops aligned on the centreline. `line_up_path` /
`pick_line_up` in [src/autotaxi.rs](src/autotaxi.rs) build a synthetic
three-point route (entry ahead of abeam by the lateral offset, then 60 m
down the centreline to finish pointing down the strip), picked against
`party_line::clearance_runway` so a "line up and wait runway two seven"
can never be driven onto the 09 end, and refused with less than 400 m
remaining past the stop (the wrong-end trap). The drive IS an auto taxi:
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
("full auto") chains it all from cold: power (turbine only), engine start,
request-taxi over the crew radio, auto taxi, departure call, line up
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
the route into the GPS via the FMS mailbox. Cruise POWER is untouched
(the climb-out throttle rides through cruise — a real cruise-power pull
is future work). Everything blackboxes as `"kind":"full_auto"` /
`"circuit"` / `"procedure"`. Related trap (2026-07-24): `Voice::alert` now
mirrors spoken lines to the panel transcript itself — any watch that ALSO
posts `s.action(same line)` double-writes the chat (the Jerez relocation
note; gyro-sync and autoland hit it first). Mirror OR action, never both.

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

Direction decided, exact license **parked** (2026-07-11): this will be **open
source, not commercial** — the earlier maybe-commercial framing is dead. No
LICENSE file yet (repo is private, so the interim all-rights-reserved default is
harmless). The parked choice is between dual `MIT OR Apache-2.0` (Rust
convention, Apache patent grant, maximum adoption) and `GPL-3.0-or-later`
(copyleft — stops payware bundling without giving back; matches Welly's ATC).
Sole-author means relicensing stays free until outside contributions are
accepted — decide before merging any.
