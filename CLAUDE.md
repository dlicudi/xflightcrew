# XVoiceLayer

The voice layer for X-Plane 12: copilot, ATC and cabin crew, entirely local
and entirely deterministic. The pilot speaks a command, the grammar picks the
matching tool, the agent writes the X-Plane dataref, and the copilot reads the
action back aloud. **There is no model at runtime** (retired 2026-09-13; the
history has the evidence): every miss in the logs becomes a rule.

```
🎙️  "Flaps up"   →  STT  →  grammar picks a tool  →  write dataref  →  TTS "Flaps up!"
```

**Full engineering history lives in [docs/dev-history.md](docs/dev-history.md)**:
every sortie-by-sortie diagnosis, date, and piece of evidence behind the rules
below. Grep it BEFORE reworking any subsystem named here; this file keeps only
the standing rules and contracts.

## Wire contracts (never rename)

Named 2026-07-30 (repo `dlicudi/xvoicelayer`, dir `~/GitHub/xvoicelayer`, after
XEarthLayer: it layers what you SEE, this layers what you HEAR). Three names
deliberately survived the rename because they are wire contracts, not branding:

- the `xcopilot/*` dataref namespace (app and plugin agree on these strings),
- the `XCOPILOT_*` env vars (the pilot's own `.env` and `prefs.json`),
- the plugin binary `xcopilot.xpl` (rename it and X-Plane loads BOTH copies
  and double-registers every dataref).

The Rust package IS `xvoicelayer`, binary `xvl`. The tracing filter must name
every workspace crate AND both crew names:
`xvoicelayer=info,xvl=info,atc=info,sim=info` ([src/main.rs](src/main.rs)) -
a line's default target is the BIN name for code in the binary and the PACKAGE
name for code in the lib, so dropping either silently loses half the log while
explicit-target lines keep arriving and make it look healthy.

## Neighbours

- **Horizon** (`~/GitHub/horizon`): ABANDONED (2026-09-11; it had become a
  charter logbook, never an instructor). Nothing lives there. A flight
  instructor, if built, is a third persona HERE, mode-gated like ATC-only so
  it never actuates: the monitor's callouts, the ATC coach, the checklist
  flow's Instruct flavour and the automations as demonstrations exist; a
  blackbox scorer against ACS tolerances, a lesson state machine and
  narration do not.
- **rustdecks** (`~/GitHub/rustdecks`): sibling Loupedeck controller. When
  hunting a dataref/command name, check its `examples/*.yaml` (and the
  cockpitdecks clones it references) before the sim's DataRefs.txt - names
  there are verified against real panels. The flow runs both ways; its
  CLAUDE.md mirrors this note.

## Stack (decided)

Rust, because this is a latency- and audio-sensitive real-time loop.

| Stage | Crate / approach |
|---|---|
| Mic capture | `cpal` |
| STT | `whisper-rs` (whisper.cpp bindings), push-to-talk |
| Intent | deterministic: [src/grammar.rs](src/grammar.rs) picks the tool, [src/conversation.rs](src/conversation.rs) and the state answerers handle questions, [src/dialogue.rs](src/dialogue.rs) back-references |
| X-Plane write | `reqwest` PATCH to the Web API (`PATCH /api/v1/datarefs/{id}/value`) |
| TTS readback | in-process piper VITS via the `ort` crate |

**No LLM, local or cloud.** The answer-only Ollama fallback was measured live
on 2026-09-12 (qwen3:8b: four calls, no useful answer, sim frame rate cost)
and removed the next day with its RAG index code, config, panel switch and
health probe. The work goes into the deterministic layers instead: `xvl
analyze` lists the most frequent missed phrasings in the logs, and each one
is a grammar rule, a matcher or a profile entry with the live transcript as
its test. The one model-shaped future use is an instructor answering handbook
questions; if it returns, it returns as that persona, never as a fallback.

**X-Plane access: REST for transport, plugin for capability.** Every stock
dataref read/write goes over the Web API. The C++ plugin
([plugin/src/](plugin/src/): `xcopilot.cpp` lifecycle, `drefs.cpp`
registration/accessors, `panel.cpp` ImGui window, `fms.cpp` FMS mailbox;
`make install` from [plugin/](plugin/)) extends the dataref surface with
`xcopilot/*` datarefs the app drives over the same Web API - the app never
links the SDK.
**Voices can ride the sim's audio** (`plugin/src/audio.cpp`, off by default):
the app already writes each utterance to a WAV; with `XCOPILOT_AUDIO_BUS=true`
or the panel's Settings switch (`xcopilot/opt/audio_bus`) it posts
"channel|volume|path" into `xcopilot/audio/play` and the plugin plays it
with `XPLMPlayPCMOnBus` - the station on COM1, the intercom on the copilot bus
- so it sits in the cockpit mix, follows the sim's volumes and headset, and
mutes with the sim. `xcopilot/audio/cut` is the barge-in: the waiter in
`tts::play_on_bus` is the ONE writer and sends it when the channel's cut
generation moves, so a cut can never overtake the next play. A failed post
evicts the cached id (plugin reloads renumber datarefs) and falls back to
the local player for that clip. The panel switch rides the settings poller
(`XCOPILOT_AUDIO_BUS=true` pins it on; the pick is refused and republished).

## Core design rule: curated tool surface, not raw dataref access

**Nothing writes arbitrary datarefs; the grammar picks from curated tools.** Each tool owns:

1. its dataref + valid range,
2. a **readback** that re-reads AFTER writing and compares against the
   INTENT - never merely reporting whatever came back (a silently failed
   write must not answer "Roger"),
3. **safety gates** ("gear up" on the ground refuses and explains).

Back-references ("a bit more", "come left ten", "turn it off", "do that
again") resolve deterministically against the **dialogue stack**
([src/dialogue.rs](src/dialogue.rs)) of calls actually executed: CLOSED
vocabulary (one word outside the shape never resolves - "more coffee" is
chatter, not a speed change), 120 s anchor TTL, automations never re-armed
from a back-reference, stack cleared on airframe swap. Resolved calls ride
the normal tool path, so every gate and readback still applies.
Configuration-state questions ("is the gear down?") answer from the SAME
datarefs the tools write; the question OPENER is what routes them - the
words inside are a command's. A judged speed question ("is our speed okay?")
gets a verdict against the governor, the floor, then the phase reference
(`speed_verdict`), never a bare number. Weather AWAY from the aircraft
(arrival, alternate, a spoken ICAO) comes from X-Plane's real-weather METAR
file ([src/weather.rs](src/weather.rs)), never the datarefs of the air we
are in. A named fix ("how far to PIMOS", whisper-spelled) is matched fuzzily
against the LOADED plan only (`fix_query`).

## Aircraft profiles drive the tool surface

Profiles ([src/aircraft.rs](src/aircraft.rs), `profiles/<id>.yaml`,
auto-detected from `acf_ICAO`, `XCOPILOT_AIRCRAFT=auto`) declare which systems
exist and their limits; the tool registry is built FROM the profile (172SP:
fixed gear so no gear tool, per-notch Vfe). Datarefs/commands stay in the Rust
tools; YAML carries per-airframe config. Standing rules:

- **All automation numbers are per-airframe**, defaulting to the old GA
  figures when a profile is silent: taxi pace (carried BY the route, never a
  static), `takeoff_roll_ft` (the ground roll the automation checks runways
  against) vs `takeoff_distance_ft` (the recited, factored field length),
  pattern altitude/speeds, park-brake valve sense, etc.
- **Multi-engine tools take `engines` and act per engine**; the readback reads
  the LAST element written (element 0 can be right while a later one refused).
  Singles stay byte-identical (element 0, starter 1).
- Twin-only speeds (`vyse_kts`, `vmca_kts`) stay `None` on singles: the
  copilot says nothing rather than inventing a number.
- **An unknown type is never silently the 172** (RV-10, 2026-09-11: flown
  on the 172's 65 kt approach, its own clean stall, into a stall).
  `known_profile` maps ICAO to a curated profile; anything else gets
  `aircraft::generic_from_acf` (speeds, flap detents and Vfe, gear, injected
  vs carburetted, weights from the sim's own `.acf` via `acf_relative_path`
  under `xplane_root()`), the pilot hears "no profile for this airframe",
  and only if the file is unreadable do the 172 numbers apply, said aloud.
  Approach speed derived = max(1.3 Vso, 1.2 Vs). `bus_volts_nominal`
  scales the start gate (12 volt systems exist).
- **No automation target goes below the airframe's speed floor**
  (`aircraft::speed_floor_kts`, 1.2 x clean Vs, republished per swap):
  `speedhold::arm_flight` and `run_autoland` raise the target to it, say so,
  and blackbox `target_floored` / `approach_ias_floored`. A wrong number
  handed down from a profile, a plan or a pilot is still a wrong number.
- **A profile line is a HYPOTHESIS until probed live**: `is_writable` and
  existence are one curl away when the sim is up. And it IS probed: on
  every airframe the detection thread walks the live profile for every
  dataref/command name (`aircraft::profile_references`, a walk over the
  profile's own serialisation) and asks the sim (`probe_dataref` /
  `probe_command`), speaks the missing ones (`probe_summary`) and blackboxes
  `profile_probe`. A new profile field with a name in it is probed the day it
  is added. A "dataref" name may really
  be a command; read-only switch positions need read-before-fire toggle tools
  (`*_toggle`), carried through full auto's copies too. Paired lights are
  written together (one pod lit is a readback confirming the half it looked at).
- **Study-level add-ons**: the stock dataref can be the add-on's OUTPUT (its
  Lua rewrites it every frame), so a stock write is a silent no-op. When a
  tool "does nothing" on an add-on, read its Lua. Declared knobs (e.g.
  `transponder_mode_knob`) are turned and CONFIRMED from the knob via one
  shared writer ([src/tools/radio.rs](src/tools/radio.rs) `set_mode`);
  selectors are republished on EVERY profile swap including to `None` -
  process-wide facts must never outlive their airframe.
- **A cart and a contactor are two different things**: ground power drives the
  cart request (only when presence differs from want - firing the toggle blind
  dismisses the cart) AND `gpu_on`, and reports BUS volts, not the request.
- [src/cockpit_auto.rs](src/cockpit_auto.rs) (unasked cockpit actions:
  `auto_at_night` interior lights, GNS430 ack): state machine over conditions
  (powered AND dark), one-shot per spell, never re-asserted (a watch that
  re-fires every tick is a fight with the pilot over a switch). "Off" only for
  lights the copilot itself lit, and the claim drops when the pilot moves the
  switch. Only INTERIOR lights may be flagged (test-enforced). Sun threshold
  has hysteresis. Sits above the `quiet` gate. Every light write, both paths,
  goes through `tools::lights::apply`.

## SimBrief

[src/simbrief.rs](src/simbrief.rs), `XCOPILOT_SIMBRIEF_ID`. The one outbound
network call besides X-Plane; on demand, never in the realtime loop; answers
deterministically, no LLM. FMS injection (the FMS is plugin-only): app writes
`xcopilot/fms/route` (payload `IDENT,lat,lon,alt|...`) then `xcopilot/fms/load`
(edge-triggered); plugin resolves idents via `XPLMFindNavAid`, composes an
XP11-format plan IN MEMORY (AIRAC stamped from earth_nav.dat), loads via
`XPLMLoadFMSFlightPlan`, active leg to entry 1. Whole-plan load, not per-entry:
only a load fills the CDU's ORIGIN/DEST. The plan also feeds the monitor's
`DESTINATION` and the fuel check. **The follower sequences it**: the copilot
flies the heading bug, never GPS mode, so nothing else moves the FMS; each
leg the follower first steers at is written as `IDENT,lat,lon` to
`xcopilot/fms/goto` and the plugin makes the matching loaded entry (ident,
nearest namesake; a nameless point within a mile) the destination on both
plans. An ident the plan lacks (an approach leg) changes nothing.

**Local planner** ([src/flightplan.rs](src/flightplan.rs), "plan a flight to
X" / "divert to X" / a bare "divert" then "to where?"): a DIRECT plan in the
same `FlightPlan` shape (`PlanSource::Local`), so every consumer above is
shared. Origin is the nearest field with land runways (the plugin labels the
first and last points as airports, so an airborne origin is still a field);
destination resolves from one apt.dat pass by exact ICAO (spelled phonetically
or written; 3 letters also tried with K) or by name words (towered and longest
runway win a tie). Cruise altitude is the rule in `cruise_altitude_ft`
(distance-earned, terrain-floored over the plugin probe, ceiling and oxygen
capped, semicircular by bearing, VFR +500 unless pressurised). Fuel comes
from the profile's cruise speed and `cruise_burn_kg_hr` or is NOT spoken. A
destination runway shorter than the landing (else takeoff) distance is
REFUSED. A local plan never expires into a SimBrief refetch and needs no
SimBrief ID; only an explicit refresh may replace it. The brief is read
first; "load it" is a separate word and a plan on the shelf authorises
nothing. Still owed: an enroute point so full auto can fly a local plan
(`enroute_legs` yields nothing from two airports), airspace dog-legs, airways.

## ATC: the world, not the crew

ATC runs in-process as a frequency-routed station (`crates/atc`): one mic, one
PTT, one speech arbiter. An utterance goes to the tower iff COM1 is tuned to
the station AND the words parse as a radio call (deterministic FSM in
[crates/atc/src/tower.rs](crates/atc/src/tower.rs)); everything else falls
through to the copilot. Frequencies come from apt.dat comm rows, never
hardcoding. Config: `XCOPILOT_ATC_AIRPORT` / `_VOICE` / `_RATE` / `_REGION`.

**Hard boundaries:**

- The station only reads the sim and returns text; it can NEVER actuate. Its
  sim handle is `sim::xplane::SimView` (read-only view, no writes on it), and
  its dep tree must stay free of whisper/cpal/ort (`cargo tree -p atc` is the
  standalone-product health check).
- **The crew/world seam is one page**:
  [crates/atc/src/party_line.rs](crates/atc/src/party_line.rs). World writes,
  crew reads through narrow accessors. Statics are crate-private (a crew-side
  write is a compile error). A new crew-side need gets a new ACCESSOR, never a
  new `pub static`. `assigned_vector`/`landing_handoff_runway` have no
  consumer yet on purpose (see history: vectored arrival needs an approach
  retune first).
- Station tests touching ANY process-wide static hold `SIDE_STATE_LOCK`;
  tests setting the app mode hold `mode::TEST_LOCK`.
- `Station::install()` publishes side state, never `from_airport` (sweeps and
  tests stay side-effect free); `uninstall()` retires it all.
- **Safety property: a mishear may earn "say again", never a clearance.**
  Mayday/pan-pan is the only intent that earns a landing clearance without a
  final report.

**Tower FSM behaviours** (each with history behind it): readback enforcement
(runway-critical items only; `XCOPILOT_ATC_READBACK=false` keeps coaching,
drops challenges), LinedUp + conditional line-up, a runway-in-use pick FROZEN
while a departure is committed (plus wind hysteresis - parallel runways
flip-flop invisibly), a release gate that THREADS the arrival stream
(`DEPART_GAP_NM`; inside the gap but strip clear = stage with LUAW), wake
clock measured from when the HOLD went on (not the last liftoff), named taxi
routes (Dijkstra over apt.dat 1201/1202; runway crossings detected from strip
GEOMETRY, never network flags; a crossing on the cleared route is never
berated; runway destinations snap to a node that can REACH the runway,
decided by connectivity, split by CALLER), ILS approaches from earth_nav.dat +
CIFP platform altitudes, dialect by ICAO prefix (FAA vs ICAO/CAP 413;
`qnh_clause` is the one pressure seam), `suggest_call` as the single source
for coach and panel.

**Audio and PTT rules:**

- TWO independent half-duplex channels: copilot INTERCOM and station RADIO,
  own queues/threads/volumes. Intercom defers starting anything below Warning
  while the radio is on air, and a routine line IN PROGRESS is stepped on
  when the radio keys up: cut mid-word, its remainder (from 0.4 s before the
  cut, rendered buffer) queued to follow once the frequency is quiet,
  perishable at 20 s (`interrupt_intercom_for_radio`); a subprocess voice
  with no buffer is said again whole. A Warning talks straight over and is
  never stepped on. Radio-only cuts
  (retune, avionics off, crash, replay) never touch the copilot's voice.
- Radio PTT vs intercom PTT: when a radio key is bound, **physical routing
  beats content routing in BOTH directions**. Content routing to ATC survives
  only for setups with no radio key.
- **Any PTT mutes everything while held** (mic hygiene on speakers); release
  rejoins LIVE audio, no replay - recovery is procedural ("say again"), never
  an auto-repeat.
- The radio needs POWER both directions (station audio dropped, transmissions
  go nowhere, copilot says "radios are dead"). **FAIL OPEN, and the flag is
  RESTORED every tick** - a process-wide fact that merely stops being
  published latches across an airframe swap (recurring bug class).
  `take_crew_radio` drains unconditionally so a failed gate loses the call
  for good.
- ONE frequency: controller speech renders as whoever COM1 is tuned to
  (`tuned_speaker`); traffic on other frequencies goes silently into the void.
- REPLAY gates the whole reactive surface (monitor tick, station audio,
  transmissions, autoland arming); PAUSE freezes ticks and control loops but
  does not cut audio. `xplane::sim_frozen` covers both; fail open (failed
  read = live). Arming autoland while paused is allowed; in replay refused.
- ONNX session CREATION is serialised across voice threads
  ([src/tts_onnx.rs](src/tts_onnx.rs) `build_session`) - two concurrent first
  sessions segfault. Inference stays parallel. Barge-in kills by REGISTERED
  pid + per-channel cut generation, never pkill by name.
- `Voice::alert` mirrors spoken lines to the panel transcript itself: mirror
  OR `s.action(...)`, never both (double-writes the chat).

## Ground & flight automation

Orchestrators compose proven loops; no new flight laws in orchestrators. All
laws battle-test as plain `cargo test`; everything blackboxes to
`logs/blackbox.jsonl` (kinds: `atc`, `autotaxi`, `autotakeoff`, `pushback`, `checklist`,
`taxi_guide`, `cockpit_auto`, `full_auto`, `circuit`, `procedure`, `cabin`,
`api_load`). Cross-cutting rules, each learned the hard way (details in
history):

- **A stage that is not moving the aircraft is holding it**: brakes set for
  every wait, released only when something else holds the aircraft; every
  abort/give-up path sets them too.
- **One writer per dataref.** Plugin override channels (`taxi_steer`,
  `taxi_brake`, `taxi_brake_l/_r`, `pitch`) are 1 s expiring heartbeats
  written EVERY tick (a change-gate drops the command exactly where it sits
  still, mid-turn); pilot yield latches (`steer_yield`, `pitch_yield`,
  debounced) drop the override that frame; writer death frees by silence.
- **Park brake goes through the valve-aware seam**
  (`ensure_park_brake_valve` / `release_park_brake_for_roll`), never a raw
  `parking_brake_ratio` write: with the valve SET, any brake application is
  latched into a permanent parking brake. Valve sense is per-airframe
  (`park_brake_valve_set`).
- Sequential HTTP reads wreck a control tick: batch them in one
  `tokio::join!`. Watch `dt` in per-tick blackbox events before touching
  gains.
- **Signed vs unsigned**: groundspeed and route distance are magnitudes (four
  knots backwards reads as four forwards); anything directional keeps its
  sign (`pushback::back_along`, `taxi_guide::project_signed` for blackbox
  `lat_m`; gates use `.abs()`).
- **Readbacks and echoes must never arm automations**: the arming grammars
  accept first/second-person request shapes only ("taxi for me", "line us
  up"); `looks_like_command` checks bigrams on raw text, and "stop" routes by
  context (governor running = retarget; steered push = full-auto abort;
  native push = stop command + brakes; airborne = heal the stale flag and
  route the words on).
- **Auto taxi** ([src/autotaxi.rs](src/autotaxi.rs)): speed via the taxi
  governor through `hold_speed`'s front door; pure-pursuit steering; an
  off-route engage is a JOIN, not a refusal (refuse only past `MAX_JOIN_M`);
  cross-track/heading error cap the pace every tick (can only slow); governor
  evaporating on the ground ARRESTS (throttle closed, expiring brake pulse,
  "your aircraft"); both override detectors are debounced against axis noise.
  Castoring nosewheels steer by differential brake through the PLUGIN
  channels only - REST writes to `*_brake_ratio` are re-driven by hardware
  every frame and silently do nothing.
- **The sim brakes against hard rudder on its own** (auto-toe brakes for a
  pilot without toe-brake axes, per the acf's "brake power to help with
  steering"), and our steer channel IS rudder input. The plugin holds
  `override_toe_brakes` for the whole time `xcopilot/taxi_steer` is applied,
  not only while braking; a pilot brake command or toe axis during a steered
  taxi latches `xcopilot/brake_yield` and hands the brakes back that frame.
  Never "fix" a corner that stops the aircraft by tuning the governor first.
- **Line up**: rides the DRAWN lead-on (`TaxiRoutePlan.lead_on`); synthetic
  geometry is the fallback, never the first choice (inventing the entry is
  how aircraft end up in the grass - same for backtracks); crosschecked
  against `party_line::clearance_runway`; refuses with <400 m remaining.
- **Auto takeoff** ([src/autotakeoff.rs](src/autotakeoff.rs)): refuses
  without a takeoff clearance when a tower runs; throttle ramps and holds
  while the nose swings; `RollSteer` = P + lead damping + slow clamped
  integrator, tiny deflection cap parked growing by 15 kt (prop wash);
  stagnation reject watches GROUNDSPEED (ASI is dead below ~20 kt); rotation
  is the `xcopilot/pitch` channel, AP takes the ESTABLISHED climb at 300 ft
  with the VS dial written AFTER mode engage (engage syncs the dial away);
  pitch damping uses the sim's measured `Q`, never attitude differenced over
  a tick, lead 0.3 s. **The handoff line must be true about who has the
  aircraft** (`handoff_to_caller`) and about the height actually reached.
- **Pushback** ([src/pushback.rs](src/pushback.rs)): X-Plane's three pushback
  commands are WHOLE manoeuvres the tug executes (a "Nose 90" is ~90 degrees,
  and that belongs in the plan's arithmetic) - sent ONCE, at the summon,
  never steered per tick; `pushback_stop` means "stop and let go". **A
  command to a tug ON THE BAR drops it off the bar** (five pushes for five,
  2026-09-12: hooked, re-issued, detached on the next poll, back ninety
  seconds later): no re-issue at hook-up, no nudge while attached, no
  second pattern - a tug that stops short is stopped and judged
  (`push_delivered`), the residual is the taxi's. "Skip / cancel the
  pushback" is `pushback::SKIP` (the push ends, the mission turns the
  aircraft under its own steam); the bare "stop" word aborts the mission.
  The bar coming off ENDS the push - judge `push_delivered`, never wait for a
  re-hook (measured: released tugs never come back). Hook needs
  `HOOK_SOLID_POLLS` consecutive attached polls before brakes release, with
  the pattern issued BEFORE release; forward creep is HELD via the expiring
  brake channel, not failed; `tow_in_progress` is dead (probed 0 while under
  tow), read as evidence only. Trigger is the cleared route's first-leg
  bearing (nose-in), never a stand database for the DIRECTION; but **a tug
  is only asked for ON A STAND** (`pushback::on_a_stand`: the nearest
  apt.dat 1300 row via `party_line::nearest_stand` within 35 m, nose within
  60° of its parking heading; a field with no stand rows keeps the
  bearing-only rule) - every restart from an off-route stop mid-taxiway
  called a tug (LEMG, 2026-09-12).
- **Taxi in** ([src/taxi_in.rs](src/taxi_in.rs)): VACATE first - completing
  the landing clearance we hold, no call owed, planned by running the
  hold-short trim BACKWARDS (no credit for metres inside the zone; exits
  behind the rollout refused; an exit is decided at the JUNCTION) - then
  "runway vacated" from the taxiway, THEN the taxi request.
  `vacate_runway` falls back to `autoland::landed_runway` (landing without a
  clearance makes vacating more urgent, not less). Any vacate failure after a
  rolling handoff arrests the roll before the minutes-long wait.
- **Routes ride the paint** (`ride_paint`): the 1201/1202 network is
  authoritative for topology/names/crossings, the drawn `120` centrelines for
  geometry; samples slide sideways only, onto near-parallel lines; runway
  legs and the vacate's start point never move; thin data degrades, never
  refuses. Stands get a `plan_to_stand` lead-in continuation (unnamed leg).
  **An arrival's route is never a departure's clearance** - check the party
  line's `taxi_to_stand`, not route proximity.
- **Circuit / procedure / full auto** ([src/circuit.rs](src/circuit.rs),
  [src/procedure.rs](src/procedure.rs), [src/full_auto.rs](src/full_auto.rs)):
  position-triggered pattern sized to the AP's real turn radius; terrain +
  airspace side/height pick; pattern numbers per-airframe with a clean-stall
  floor (`pattern_speed_kts` >= 1.3 Vs clean); `climb_limit_fpm` fades
  commanded climb to zero toward blue line / 1.15 Vs (a climb is never worth
  more than the airspeed left); the throttle law and its escalation gates
  watch a 5 s PROJECTION of speed, not the raw error; base descends the PATH
  to the predicted roll-out point, final follows real along-track distance
  (`path_dist_m`) - never track-miles, whose cross-track term inflates the
  target by the whole leg; the final leg has a REAL intercept
  (`final_intercept_hdg`, capped 30°) sized on the turn actually flown
  (`final_turn_radius_m`, `AP_BANK_FINAL_DEG`); `ready_for_autoland` requires
  ESTABLISHED (ground-track rolled out + `ESTABLISHED_XTK_M`), with the
  last-ditch valve reserved for turns that went WRONG. Glidepath tracking
  feed-forwards the path's own descent rate (`path_descent_fpm`, from ground
  closure - a P controller cannot track a ramp) and the lead compares two
  futures. Full auto: engine start per airframe CLASS (`StartKind`, per-class
  deadlines), power-up runs INLINE (two racing bring-ups both toggle a
  cross-tie and leave it shut), waits on `driving()` (not `ACTIVE`, which
  gaps during backtracks), refuses to arm airborne (names the tools that
  fly), and both missions end with `taxi_in_after_landing`. Cruise power is a
  SPEED from the profile (`reference.cruise_ias_kts`), never SimBrief's.
  **Exterior lights follow the practice**, one rule set in
  [src/tools/lights.rs](src/tools/lights.rs) (`Exterior`, republished per
  swap; `dark_now` = `cockpit_auto::dusk_or_dark`): the taxi light for the
  taxi, or after dusk the landing light on an airframe with no taxi light
  (the SR22's one lamp); the landing light over every runway span of the
  route (crossings, backtracks, the line-up leg: `Route.runway_spans`, lit
  40 m ahead) unless it is already the taxi light tonight; strobes and
  landing lights lined up; landing lights off passing 1,000 ft AGL
  (`lights_climb`); at the vacate landing off and taxi on
  (`after_vacate`); all off at the stand.
  **The arrival is briefed at the gate** (`full_auto::brief_arrival` in the
  arming ack: published approach, straight-in, or "I'll land visually"; a
  destination the data lacks is refused there) - found 25 nm out it was a
  crash. **The follower never sequences the FMS**: the sim's GPS sits on the
  plan's first entry for the whole flight, so anything that names the next
  fix (progress questions, the course check, the waypoint reminder) reads
  `procedure::active_leg()` first and the GPS only when nothing is flying.
  **Full auto flies the checklist items it owns; the lists are its
  cross-check, never its hands** (see the checklist flows below): on a
  continuous-mixture piston (`piston_mixture`, the profile's `engine` group,
  never the DC-3's notched lever) the enroute leg arms the mixture manager
  (`engine::arm_mixture_manager`: rich below `LEAN_FLOOR_ALT_FT`, peak EGT
  above, a FULL walk from a rich lever via `manager_seek_window`, re-trimmed
  per `needs_retrim`) and the arrival sets full rich through
  `mixture_rich_for_landing` whatever the field elevation; the manager
  richens itself when it yields to autoland or a recovery, and stands down
  on every give-up ("your aircraft" means every lever).
- **Autoland**: pursuit aim keeps its full lead all the way in
  (`pursuit_aim_nm` - the aim walks down the runway, bounded by pavement);
  roundout band (`ROUNDOUT_MULT`/`ROUNDOUT_VS_FPM`) before the flare - a
  landing is one continuous manoeuvre, and level-hold over the runway is the
  bug; rollout brakes by wanted deceleration, not a flat ratio; when
  taxi-in follows, hand off ROLLING at `VACATE_HANDOFF_KTS` via
  `autoland::VACATE_FOLLOWS` (consumed at rollout entry, cleared on every
  no-landing path); overflight allowance is runway-aware only when LOW over
  the pavement (high stays hair-trigger); the missed approach is a sticky
  `MissedStage` with a timer longer than the manoeuvre it commands (600 s);
  give-ups hand back a FLYABLE aircraft (climb power, VS zeroed, a line that
  says what is actually held). **A tower's go-around order outranks
  everything** (`GoAroundLatch::order`): consumed in the autoland loop,
  drained at arm time, refused on the ground; a fresh "final" report is owed
  after any go-around, made inside the FAP. **The wind crab fed forward on
  final is LOW-PASSED** (`CrabFilter`, `CRAB_TAU_S`): the raw heading-minus-
  track added to the bug at unity gain hunted the LEMG 13 final into the
  lateral balk (2026-09-12); the blackbox keeps `crab` (applied) and
  `crab_raw`. **The crab cancels wind and nothing else**: a bias between
  the BUG and the HEADING (the 172 settling 2° left of its bug, LEMG 13,
  same day) passes straight through it and parks a proportional final
  beside the centreline (0.6 nm of lead x tan 2° = 39 m, on the 37 m gate:
  two balks). `TrackTrim` integrates the final's track error (wanted track
  minus `hpath`, 30 s, ±5°, final only, reset by any intercept) into the
  bug; the tick carries `trim` and `xtk_m`, and every balk names its gate
  (`Guidance.balk`, on the `going_around` event) so the debrief says why. **Every planned arrival hands its runway down**
  (`EndAction::Approach { rwy_id }` to `run_autoland`'s forced runway): the
  self-pick is the emergency path and at a field with parallels it takes
  the nearer threshold (ILS 12 flown, landing on 13, cleared for 12). The
  tower's clearance outranks the plan for an end at the SAME field, said
  aloud and blackboxed `runway_from_clearance`.
- **A straight-in is only as good as its fixes** (LXGB 09, 2026-09-11): a
  DME-fix reconstruction keeps only fixes ON the final approach course
  (`RECON_ALIGN_DEG`), and `approach_plan_for` refuses any FAF autoland could
  not intercept from (`faf_off_course`, the circuit's own `interceptable`
  test) so the arrival falls back to the visual landing. Approach plans hold
  the CLEAN approach speed (`power_hold_target`); autoland's gear and flaps
  wait for the profile's VLO/Vfe (`gear_may_extend`, `flap_within_limits`,
  republished per airframe); taxi-in never follows `has_crashed`.
  Above the floor the same geometry is a REPOSITION to the join point, not
  a "final": "land the aircraft" from 1.5 nm abeam flew a saturated cut.
  **Below the terrain floor a final must still be interceptable**
  (`stabilised_xtk` = the circuit's `interceptable`, inside the FAP only) or
  it balks with height in hand - the flare gate under 100 ft was the only
  lateral balk, which is why the crash happened at 95 ft.
- **The crew works the radio as full auto flies** (same sortie): a
  conditional line-up waits on `party_line::line_up_condition_clear()` (the
  world's traffic picture, published by the release watch every tick) before
  crossing the hold line; the after-takeoff handoff is consumed via
  `take_departure_handoff()` (readback on the old frequency, retune, check-in
  with altitude); the inbound call waits for the destination's station swap
  (`home_tower_khz`) before it goes out. A descent waits for its top of
  descent (`descent_target_ft`), a cruise fix within 300 ft of cruise IS
  cruise, a JOIN projects along the nose (`project_signed_facing`), and the
  taxi law's idle-and-brake needs the speed to actually be over by 2 kt
  unless the build is a surge (`SURGE_KTS_S`).
- **Taxi guide** ([src/taxi_guide.rs](src/taxi_guide.rs)): early gentle cue,
  quiet inside the 60 m where the compliance watch takes over - never two
  voices on one mistake; gates on `pushback::ACTIVE` and along-track progress
  direction. On-demand "which way?" is question-shaped so a readback never
  triggers it.

## Checklist flows: the list at its phase, three flavours

[src/flow.rs](src/flow.rs) runs the airframe's XChecklist sections
([src/checklist.rs](src/checklist.rs) interprets the file) on their own
thread, replies taken from the command loop (`flow::take_reply`, CLOSED
vocabulary: yes/no at a hazard confirm, next/set/checked/skip at an
instructor's item, stop always; "flaps ten" mid-list is still flaps ten).
Every `Item::Act` carries the file's own pass test, so a runner reads the
state FIRST: an item already in state is confirmed, never re-written under
the pilot's hand, and an item a running loop owns (`held_by_automation`:
governor/throttle, mixture manager, autoland/flaps, the flying loops/AP) is
named as held, never written (one writer per dataref). Flavours:

- **Crew** (pilot flying, copilot acting): the spoken "descent checklist"
  lands here.
- **Monitor** (an automation flying): read-only cross-check, narration
  skipped, a discrepancy spoken and blackboxed (`checklist` event
  `discrepancy`), never a write. The mission's laws act; the list audits.
- **Instruct** ("talk / walk / guide me through the X checklist",
  `instruct_request`): never writes by construction; says the item, watches
  the dataref for the pilot to do it ("Good."), moves on at "next"/"set" or
  `INSTRUCT_WAIT_SECS`. The first piece of the instructor persona; the
  conversation module's "preview / explain the X checklist" stays the
  read-only recital.

**Who starts one**: the monitor's `flow::Tracker` (sustained conditions,
one-shot per flight, a flight begins at the takeoff roll and ends with the
after-landing list): climb established (`CLIMB_AGL_FT`), level 30 s above
2,000 ft AGL, descending 20 s from cruise height, below 1,500 ft descending
10 s, rolled out under 30 kt for 10 s. Under an automation the Monitor
flavour starts at once; with a pilot flying the copilot PROPOSES ("Climb
checklist when you're ready?", `PROPOSAL_TTL`) and "go ahead" runs it,
"talk me through it" teaches it, "not now" declines - a proposal never acts
by itself (the cockpit_auto rule). Section-to-title is the same matcher a
spoken "climb checklist" uses; a file whose titles differ maps them in the
profile's `checklist_flow:`; a section the file lacks proposes nothing.
Panel: `flow::status()` ("checklist: climb, 3 of 8") and CANCEL picks "stop
the checklist" when nothing is flying.

## App modes & cabin crew

`mode.rs`: **both** (default) / **copilot** (no simulated ATC; radio-keyed
speech IGNORED - real ATC owns the frequency) / **atc** (the world only, no
crew). Chosen from the panel Settings tab, persisted;
`XCOPILOT_MODE` sets only the BOOT value; applies live within ~2 s. The "no
crew in ATC-only" gate lives in the arbiter's enqueue door (`Voice::alert` and
friends), not scattered per-site; blocking `say` is mode-routed in the main
loop so "copilot is off" can speak. The atc crate never reads the mode.

Cabin crew ([src/cabin.rs](src/cabin.rs)): second intercom persona, no third
PTT/channel. The address is the routing (explicit "cabin crew" always; bare
"cabin" only with an unmistakable crew word). Deterministic keyword intents,
"say again" over guessing, chime first, own lazily-loaded voice. Exists
per-airframe (`cabin_crew: true`) + panel staffing Auto/Aboard/Off. "Cabin
secure" is a safety report, exempt from sterile cockpit
(`STERILE_FLOOR_AGL_FT` 10,000). On a crewless airframe: an honest "no cabin
crew on this airframe", never an improvisation.

## Panel, terminal, settings

- **The panel is the act-and-glance instrument** (Flight | Transcript | ATC |
  Route | Settings). Status well fed by `xcopilot/status/auto`
  ([src/auto_status.rs](src/auto_status.rs)), published change-gated from the
  monitor tick with an unconditional resend every 5 ticks (plugin reloads
  zero the buffers). **Every manager line carries its live detail**
  (`auto_status::Detail`, set by the loop from the numbers it already has,
  rounded so the feed is not re-sent for jitter, cleared by a `DetailGuard`
  on every exit): the follower's fix, range, minutes and altitude, the
  circuit's leg, the taxi's metres to go, the takeoff's sub-stage, full
  auto's "waiting for the squawk" (`wait_status`, cleared by the next stage
  event), the governor's target against the actual. Two panel-only lines
  follow the managers and are never spoken: the AP targets (`set_ap_targets`,
  bug/alt/VS as dialled) and the crew's last spoken line (`note_said`, from
  `Voice::alert`/`say`). Buttons inject the same English a pilot would speak
  (`REQUEST_PHRASES`, grammar-test-pinned so a button always parses);
  CANCEL is picked per what's running at PRESS time and ALWAYS aborts
  something ("my controls" fallback). Presses ride the typed-command queue,
  one per loop pass, as does the Transcript tab's typed line
  (`xcopilot/typed`, consumed by a NUL write). The panel renders through
  X-Plane's native panel-graphics API (SDK 4.4.0, X-Plane 12.4.4+;
  [plugin/src/imgui_xp.cpp](plugin/src/imgui_xp.cpp), no OpenGL); fonts are
  B612 rasterised per frame at the asked size, so UI scale (100..200%) is a
  real size change. Window geometry persists in the sim's preferences folder.
- **The terminal is setup and diagnosis** ([src/ui.rs](src/ui.rs), Health +
  Transcript): Web API load window, plugin-loaded probe, EFFECTIVE TTS engine
  (what the speaker thread built after fallbacks), COM1 as diagnosis (amber
  when tuned to no known service). Renders under `TestBackend` in plain
  `cargo test`.
- **Settings are a registry** ([src/settings.rs](src/settings.rs)): live
  `Arc<Config>`, consumers take `settings::live()` at the top of each loop
  (copy-on-write); non-per-tick knobs are PUSHED (mic trio, readback
  enforcement). A knob is declared once (render/validate/step/apply/persist);
  precedence is env > saved pref > default, and an env-pinned knob is REFUSED
  on screen with the variable named. Stored forms carry no unit.

## Web API load is a sim cost

The API is served from inside X-Plane's flight loop - every request is frame
time the sim does not get. `crates/sim/src/xplane.rs` reports rate / cache
hits / failures / slowest every 30 s (log + blackbox `api_load`, with a
per-dataref histogram - the only thing that can name a hog). Reads go through
a 60 ms value cache (a write drops its own entry so readbacks stay real);
near-static flags (`is_in_replay`, `paused`, ...) are `SLOW_DATAREFS` at
500 ms. Volume, not duplication, is where the cost is: `cached=0` in a window
is the honest answer, not a broken cache.

## Battle-test surface

All deterministic, in `cargo test --workspace` (plain `cargo test` at the root
runs only the crew package): the state-by-intent matrix in
[crates/atc/src/tower.rs](crates/atc/src/tower.rs); the adversarial corpus
[crates/atc/bench/atc_corpus.yaml](crates/atc/bench/atc_corpus.yaml) (real
whisper output - **add every new wart here**); scripted sortie dialogues over
`route_keyed`/`route_unkeyed` in
[crates/atc/src/station.rs](crates/atc/src/station.rs); the spoken-call eval
(`cargo run -- stt-eval bench/atc_stt_cases.yaml`); the world sweep
(`cargo test --release -p atc -- --ignored atc_world_sweep`, needs apt.dat);
ILS smoke test (`cargo test -- --ignored real_install_ils`); the in-sim soak
checklist [docs/atc-chaos-card.md](docs/atc-chaos-card.md). `xvl debrief`
([src/debrief.rs](src/debrief.rs)) reads `logs/blackbox.jsonl` after every
flight: aircraft, the numbers, every callout, and the narrative of events an
instructor would write down (crashes, balks, void calls, floored targets,
taxi stops). Run it before diagnosing by hand.

When a control law misbehaves in the air but its test passes, suspect the
test's plant first: a first-order lag cannot ring at any gain. Sortie tests
should fly second-order plants through the loop's real transport delay and
assert the numbers the blackbox reports.

## Licensing

**GPL-3.0-or-later** (root `LICENSE`, inherited via `[workspace.package]`).
Settled: open source, never commercial, X-Plane only. Copyleft on purpose
(the ecosystem is payware-dominated and this exact product shape sells for
$49). SDK and Dear ImGui are MIT/BSD; B612 is OFL - all
GPL-compatible. Sole-author relicensing stays free only until outside
contributions merge: settle any change of heart BEFORE accepting a PR.
`crates/atc` could become LGPL-3.0 if it should ever be broadly embeddable -
deliberately not done now.
