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
  charter logbook, never an instructor). Nothing lives there. The flight
  instructor IS the third persona here (2026-09-13, `mode::Instructor`,
  [src/instructor.rs](src/instructor.rs); see "App modes"): mode-gated so
  it never actuates, with the ACS scorer, the lesson state machine and the
  narration Horizon never got.
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
| Intent | deterministic: [src/router.rs](src/router.rs) decides command / question / follow-up / echo, [src/grammar.rs](src/grammar.rs) picks the tool, [src/conversation.rs](src/conversation.rs) and the state answerers ([src/answers.rs](src/answers.rs), [src/plan.rs](src/plan.rs) for the plan's questions) handle questions, [src/dialogue.rs](src/dialogue.rs) back-references; `main.rs` keeps boot and the command loop, [src/detect.rs](src/detect.rs) the airframe swap |
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
- **"Enough runway" is ONE rule** (`aircraft::takeoff_needed_m` /
  `takeoff_needed_now_m`: the roll scaled to the weight on the wheels, x 1.4,
  the caller's floor) behind the takeoff, the circuit, the line-up's
  full-length check, full auto's `runway_check` and the arm preflight. It was
  five copies and the weight fix reached one: full auto said "I couldn't get
  us onto the runway" to an A330 lined up on EGPH 24 (2026-09-20). A failed
  fit is SPOKEN by its gate (`autotakeoff::no_fit_line`), never as "not on
  the runway".
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
- **A flow runs at a crew's pace and the voice says milestones, not
  switches** (`jet_start::AirlinerStart::run_steps`, 2026-09-20): a switch
  is fired, its position READ BACK until it agrees, then a beat; one
  already set is passed over; every step is blackboxed and on the panel,
  only a profile `say:` milestone is spoken, and a switch that never took
  is named in the closing line. Twenty presses in two seconds is a macro.
- **No automation target goes below the airframe's speed floor**
  (`aircraft::speed_floor_kts`, 1.2 x clean Vs, republished per swap):
  `speedhold::arm_flight` and `run_autoland` raise the target to it, say so,
  and blackbox `target_floored` / `approach_ias_floored`. A wrong number
  handed down from a profile, a plan or a pilot is still a wrong number.
  The governor's CEILING is the airframe's too (`speed_ceiling_kts`, Vne
  less 15, republished per swap; 250 only with no Vne on file): a flat 250
  held an A330's 290 kt cruise with nothing said (2026-09-20).
- **A profile line is a HYPOTHESIS until probed live**: `is_writable` and
  existence are one curl away when the sim is up. And it IS probed: on
  every airframe the detection thread walks the live profile for every
  dataref/command name (`aircraft::profile_references`, a walk over the
  profile's own serialisation) and asks the sim (`probe_dataref` /
  `probe_command`), speaks the missing ones (`probe_summary`) and blackboxes
  `profile_probe`. A new profile field with a name in it is probed the day it
  is added. **The Rust constants are hypotheses too**: two local spellings
  (`vvi_dial_fpm`, `compass_heading_deg_magnetic`) did not exist and failed
  for months into fallbacks that impersonated a measurement (the flare lead
  led nothing, 2026-09-15); read names come from `datarefs::*`, and
  `cargo test -- --ignored real_install_names` checks every `"sim/..."`
  literal in the workspace against the install's DataRefs.txt/Commands.txt.
  **The plugin's side of the wire contract is probed the same way**
  (`health::PLUGIN_DATAREFS`, every `xcopilot/*` literal in the source,
  test-pinned, on the profile probe's index fetch): a stale `.xpl` is said
  at boot and shown red on Health. Editing `plugin/src` is not done until
  `make -C plugin`, the install AND the reload have happened: the app read
  `xcopilot/terrain/valid` for two days from a plugin built before it
  existed, every terrain probe failed as a "timeout", the follower's floor
  retired itself ten seconds after each takeoff, and the airborne resume
  refused on the route (2026-09-16).
  **A failed read is NO measurement, never a plausible number**: a control
  loop skips the tick and blackboxes `null`. A "dataref" name may really
  be a command; read-only switch positions need read-before-fire toggle tools
  (`*_toggle`), carried through full auto's copies too. Paired lights are
  written together (one pod lit is a readback confirming the half it looked at).
  **The gear comes up outside its own warning** (`autotakeoff::gear_up_due`,
  `aircraft::GearWarning`, read from the `.acf`'s `_gear_warn_*` keys at
  detection when the profile is silent): X-Plane sounds the horn below
  `below_kias` with the throttle under its percentage or the flaps past
  their ratio, and the flat 200 ft rule raised the SF50's gear at 122 kt
  with its 0.5 takeoff flap, a warning on every departure (ETND 26,
  2026-09-23). Above 200 ft, once raising it is silent (+3 kt), and always
  by 1,000 ft, 10 kt under Vlo, or at the handoff.
  **A read-only KNOB is stepped by its commands** (`TransponderModeKnob`
  `up_command`/`down_command`, `radio::step_knob`: read, one step, read
  again, bounded): the A330's transponder knob reports `is_writable: false`
  and every mode write on it was a silent no-op (2026-09-20). Still owed:
  the profile probe checks existence, not writability.
- **Study-level add-ons**: the stock dataref can be the add-on's OUTPUT (its
  Lua rewrites it every frame), so a stock write is a silent no-op. When a
  tool "does nothing" on an add-on, read its Lua. Declared knobs (e.g.
  `transponder_mode_knob`) are turned and CONFIRMED from the knob via one
  shared writer ([src/tools/radio.rs](src/tools/radio.rs) `set_mode`);
  selectors are republished on EVERY profile swap including to `None` -
  process-wide facts must never outlive their airframe. **The crew's
  side has one page for that** ([src/flight_state.rs](src/flight_state.rs)):
  `publish_airframe` sets every selector from the profile (boot and swap),
  `stand_down` stops every loop that can hold the aircraft (swap, instructor
  entry, connection lost), and a source-scanning test pins that list to every
  `static ACTIVE` in the tree. A new loop or selector goes there, never in a
  hand list in a caller.
- **A crew configures the aircraft before it moves**
  ([src/takeoff_config.rs](src/takeoff_config.rs), EGPF 2026-09-21: flaps
  written after "Rolling", the dial at the 100 ft the A330 loads with, spoilers
  and autobrake never set). One page, three callers: full auto after the
  start, the takeoff before the brakes come off (it WAITS for the flaps to
  arrive), autoland at the gear. Flaps, trim and the altitude dial everywhere
  (`initial_altitude_ft`: the caller's level, else 3,000 ft above the field;
  a dial the pilot set is the pilot's); spoilers and autobrake where the
  profile's `takeoff_config:` declares them. **Armed for the roll is an
  Airbus habit**: a Boeing declares `takeoff_speedbrake_down` (the 737's own
  BEFORE TAKEOFF item; the lever is put DOWN, a reject above 40 kt raises it,
  `on_reject`), and spoilers armed for a roll are disarmed after the gear
  (`after_liftoff`; the 737 climbed through 7,000 ft with SPEED BRAKE ARMED
  lit, EGPH 2026-09-21). Read first, left alone in state,
  read back, named when it did not take, one milestone line. **The cabin's
  air** ([src/tools/cabin_air.rs](src/tools/cabin_air.rs), profile
  `cabin_air:`): bleed and crew oxygen on for EVERY full-auto mission, an
  airborne resume included, and at the monitor's cabin-altitude callout;
  past the warning band a flying follower is handed 10,000 ft. An oxygen
  button that is a TOGGLE is fired only when it reads off, and the valve
  behind it is written too (EDLP-LOWI 2026-09-24: bleed never switched,
  cabin at 24,000 ft, the pilot blacked out at FL260). **Seat belt
  signs** ([src/tools/signs.rs](src/tools/signs.rs)): on before anything
  turns, off through ten thousand, on again through ten thousand, on for the
  arrival; an add-on's own switch is declared (`seat_belt_signs:`) and
  stepped, a stock cabin gets the stock switch.
- **A box with several knobs is confirmed from its output**
  (`TransponderModeKnob.also`, `radio::stock_transmits`): the A330 in AUTO
  with TCAS in STBY is stock mode 1, and `transponder_alt ok: true` was
  logged from the one knob that was turned while the flight squawked standby.
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
**The plan is the WHOLE navlog; the mailbox is the payload's problem**
(`simbrief::fms_route_payload`): `xcopilot/fms/route` is a 512-byte `TextRef`
that truncates at 511 mid-entry, so the GPS route is thinned from the MIDDLE
(origin and destination always survive - they are what fills ORIGIN/DEST).
Capping the PARSE at fourteen fixes instead amputated the tail: EGLL-EGPH,
2026-09-17, the plan stopped at RIBEL and the TARTN arrival and EGPH itself
were never in it, so the follower ended 110 nm short and the approach armed
from 121 nm out (see history). A cap on the front of a list is an amputation
of its end.

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
SimBrief ID; only an explicit refresh may replace it. **"Fly the flight plan" is preflighted AT THE ARM**
(`full_auto::preflight`, pure and tested; blackbox `preflight` on the
`full_auto` kind): the destination's longest runway against the profile's
landing distance, this field's against the factored takeoff roll
(`takeoff_needed_m`, the line-up's own rule), the sim's all-up weight
(`weight/m_total`) against the plan's max takeoff weight (else the profile's
max gross), the tanks against the plan's MINIMUM takeoff fuel (refused
below, said with both numbers between it and the block), and the sim's
payload (`weight/m_fixed`) against the OFP's load sheet (`PlanWeights`:
pax, bags, cargo, payload, TOW/LDW limits, converted to kilos from the
plan's own unit). A refusal names the number; an unknown figure is skipped,
never guessed; the pilot hears the fuel and who is aboard in the arming
ack. Until 2026-09-15 the arm checked only geography: the fuel was found at
the enroute caution and the runway at the hold. **A spoken ask for the
plan ("load / what's the flight plan") re-fetches a SimBrief plan EVERY
time** (`PlanFetch::Asked`; the 10 min TTL is only for the implicit
consumers: fuel, TOD, ETA): the pilot regenerates on the website and says
the same words (LEMG-LEJR re-sent from the shelf four times, 2026-09-14).
"That's the wrong plan" re-fetches and says whether SimBrief moved
(`plan_change_line`). The brief is read
first; "load it" is a separate word and a plan on the shelf authorises
nothing. **The crew does the load sheet when asked**
([src/tools/loading.rs](src/tools/loading.rs), `load_aircraft`: "load the
aircraft", "set the fuel from the plan", "board the passengers"): on the
ground and stopped only, the plan's BLOCK fuel wings first and the centreline
tanks with the remainder (`distribute_fuel`, from `acf_tank_rat` and
`acf_tank_X`), the sheet's payload into `m_fixed`, refused with both numbers
past the tanks or `acf_m_max`, left alone in state, read back against the
intent ("the fuel did not take" on an add-on that owns its tanks). The words
have the FMS load's shape ("load the fuel from the flight plan"), so the main
loop asks `grammar::load_aircraft_request` first; "refuel" stays the fuel
truck. Still owed: an enroute point so full auto can fly a local plan
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
drops challenges; the zone clearance's readback is SUGGESTED, never
enforced, and the crew reads it back under full auto), a departure check-in
answered with the climb it announced (`climb_announced`), a calm-wind
runway pick that follows the arrival stream (`arrival_end`, only with no
runway in use yet), LinedUp + conditional line-up, a runway-in-use pick FROZEN
while a departure is committed (plus wind hysteresis - parallel runways
flip-flop invisibly), a release gate that THREADS the arrival stream
(`DEPART_GAP_NM`; inside the gap but strip clear = stage with LUAW), wake
clock measured from when the HOLD went on (not the last liftoff), named taxi
routes (Dijkstra over apt.dat 1201/1202; runway crossings detected from strip
GEOMETRY, never network flags; a crossing on the cleared route is never
berated; runway destinations snap to a node that can REACH the runway,
decided by connectivity, split by CALLER, and ON THE DESTINATION STRIP's own
pavement - two thresholds 18 m apart and diverging put a 737 at runway 09's
hold with a clearance to 07, 169 m from the centreline the line-up was
looking for, and the refusal said only "I couldn't get us onto the runway"
(LFPB 2026-09-22); a gate that refuses NAMES the strip and the distance,
`NoFit::NoneNear` included), ILS approaches from earth_nav.dat +
CIFP platform altitudes, dialect by ICAO prefix (FAA vs ICAO/CAP 413;
`qnh_clause` is the one pressure seam), `suggest_call` as the single source
for coach and panel.

**The tower's wind is the FIELD's** (`station::set_surface_wind`, fed from
the METAR file by `monitor::feed_surface_wind`, above the quiet gate, keyed
by position): the wind datarefs are the air the aircraft is in, and at
2,700 ft they picked the opposite end to the crew's METAR pick (EGTE,
2026-09-21). Arrival states freeze the runway in use like departure states
do; an arrival's STATED runway stands when the wind is inside the swap margin
and no stream lands the other way (`note_stated_runway`). A frequency read
back is a readback (`frequency_readback`); a readback within 45 s of a region
switch is `handoff_readback`, not `void`. **A REPORT is acknowledged and
spends nothing** (`Intent::RunwayVacated`, the crew's own call from the
taxiway, answered "say again" until 2026-09-22): the state stands, so the bare
"request taxi" after it is still the taxi IN, and a call that also ASKS is
that request instead.

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
- **A crew call is routed on the frequency it was keyed on, before it is
  spoken** (`say` blocks until the frequency is quiet; a handoff readback
  held 19 s was routed after the retune and Departure said "say again",
  2026-09-15). A call with a live number (`CrewCall::CheckIn`) is rendered
  by the main loop as it goes out, after a bounded wait for `radio_on_air`.
- Radio PTT vs intercom PTT: when a radio key is bound, **physical routing
  beats content routing in BOTH directions**. Content routing to ATC survives
  only for setups with no radio key.
- **Any PTT mutes everything while held** (mic hygiene on speakers); release
  rejoins LIVE audio, no replay - recovery is procedural ("say again"), never
  an auto-repeat. **And the recovery knows what was cut**
  (`voice::SpeechMemory`, `router::recall_line`, 2026-09-23): a press keeps
  the copilot line it silenced and the ones it drained as OWED (a radio
  remainder too, until it plays), 90 s; "what were you saying?" says the owed
  lines, "say again" the NEWEST of owed, last readback and last volunteered
  line. It used to know only the last command readback, so a caution the
  pilot keyed over was gone and "say again" answered with an older line.
  **The frequency's "say again" repeats the controller** (`tower::is_say_again`,
  `voice::controller_repeat`, `main::repeat_controller`): the controller's
  LAST transmission addressed to us (logged at air time, or when a press
  blocked it; never ATIS, a ghost, or `alert_station_overheard`), word for
  word, within 60 s, on the SAME COM1 frequency. Being the last word, a
  cancelled clearance is never on file. Before, every one earned "say again"
  back, even straight after a takeoff clearance. "What did the tower say?"
  on the intercom reports the same line (5 min); with external ATC the crew
  says it cannot hear the frequency.
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
`instructor`, `loading`, `turbine_start`, `heli_start`, `hover`, `heli_circuit`,
`heli_route`, `rapport`, `cabin_air`, `api_load`). Cross-cutting rules, each learned the hard way (details in
history):

- **A stage that is not moving the aircraft is holding it**: brakes set for
  every wait, released only when something else holds the aircraft; every
  abort/give-up path sets them too. **And a mission that ENDS hands the hold
  to the PARKING brake** (`full_auto::hand_back_brakes`, before the line is
  spoken, through `drive_park_brake`): a taxi that reaches its holding point
  leaves the governor running on purpose, and at LFPB (2026-09-22) it went on
  holding a 737 on its wheel brakes for 27 minutes after "your aircraft" -
  3,736 ticks of Web API load, `park` at 0.0, a squeeze the pilot cannot
  release. Only the TAXI flavour, only on the ground, only under 25 kt (a
  park brake is a chock, not a stopping device); the governor's next tick
  sees the brake and stands itself down.
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
  without a takeoff clearance when a tower runs; a `brake_steer` airframe
  (castoring nosewheel) brake-steers the roll like the taxi, faded out 20 to
  40 kt (`roll_brake_split`; the SF50 had no low-speed steering at all,
  EDLP 2026-09-24); the transponder goes to ALT as the roll starts; throttle ramps and holds
  while the nose swings; `RollSteer` = P + lead damping + slow clamped
  integrator, tiny deflection cap parked growing by 15 kt (prop wash);
  stagnation reject watches GROUNDSPEED (ASI is dead below ~20 kt); rotation
  is the `xcopilot/pitch` channel, AP takes the ESTABLISHED climb at 300 ft
  with the VS dial written AFTER mode engage (engage syncs the dial away);
  pitch damping uses the sim's measured `Q`, never attitude differenced over
  a tick, lead 0.3 s. **The handoff line must be true about who has the
  aircraft** (`handoff_to_caller`) and about the height actually reached.
  **A pilot handoff leaves a FLYABLE aircraft** (EGPH 2026-09-20: takeoff
  thrust, VS +1,500, flap 1, the dial at 100 ft, Vmo by 10,000): the handoff
  waits, still climbing, for the clean-up (`handoff_waits_for_clean_up`,
  30 s); a dial the climb cannot capture is set 2,000 ft up
  (`handoff_altitude_dial_ft`); the flight governor takes the speed the
  loop's own overspeed guard was holding (`handoff_speed_kts`, 250 below
  10,000), and the line names both. A flap schedule must be flyable inside
  its placards (test-enforced over every profile). **A jet's climb-out is
  declared** (`reference.acceleration_height_agl_ft`, `autotakeoff::ClimbOut`,
  republished per swap): below the height the VS dial HOLDS
  `initial_climb_kts` by climbing harder (`climb_out_vs_fpm`, 100 fpm a knot,
  to `best_climb_fpm`) and the flaps stay put; above it the old 1,500 fpm
  accelerates, the schedule steps, and the handoff waits for the schedule
  (`handoff_waits_for_schedule`, 45 s) instead of one shot to zero. One
  capped 1,500 fpm at takeoff thrust was 240 kt through 1,000 ft with the
  flaps up by 740 (EGPH, 2026-09-21). A circuit (handoff 500 ft) keeps the
  old climb-out whole. Still owed: the hand-flown 8 degrees below 300 ft is
  where the first fifteen knots go.
- **Pushback** ([src/pushback.rs](src/pushback.rs)): X-Plane's three pushback
  commands are WHOLE manoeuvres the tug executes (a "Nose 90" is ~90 degrees,
  and that belongs in the plan's arithmetic) - sent ONCE, at the summon,
  never steered per tick; `pushback_stop` means "stop and let go". **A
  tug mid-swing finishes its pattern whatever it is told**: `done`'s
  distance test waits while the bar is on and the nose is still moving
  (`swing_in_progress`; LXGB 2026-09-15, a Nose 90 that ran 45 m straight
  first), and **a tug on the bar owns the aircraft**: the taxi drive holds
  on `pushback_attached` (`tug_hold`, no stagnation clock, `TUG_WAIT_SECS`)
  and rolls when the bar comes off. **A
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
  **A vacate turns TOWARD the parking and never onto another runway's
  pavement** (`nearest_stand` gives the side, the rollout heading is the
  axis; an edge inside a zone that is not ours is refused outright, not
  merely ranked last): at EBLG the nearest junction was C3, the link between
  the parallels, so a 737 vacated 04R 148 m onto the wrong side, 62 m from
  04L and inside its zone, and the taxi-in clearance came back across the
  runway needing a 180 on a taxiway (2026-09-22). Stands on both sides pick
  the near apron; no stands in the data keeps the distance-only rule.
  `vacate_runway` falls back to `autoland::landed_runway` (landing without a
  clearance makes vacating more urgent, not less). A turn-off up to 120 degrees
  off the rollout is an exit (EDLP's B at 108; EGLL's N4W at 169 is not),
  the vacate's budget is sized on its route (`vacate_budget_secs`), a stand
  is entered along its parking line (`nav::stand_lead_in`, 25 m back), and
  **a mission ends shut down** (`taxi_in::shut_down_at_stand`, full auto
  and the helicopter route only): the registry's own shutdown for the class,
  every engine confirmed from `ENGN_running`, then all electrics off; a
  spoken "taxi to parking" leaves the engines to the pilot. Any vacate failure after a
  rolling handoff arrests the roll before the minutes-long wait.
- **A jump in a projection is never a movement of the aircraft**
  (`taxi_guide::project_signed_facing`): the facing filter used to be
  all-or-nothing, so the moment ONE far segment entered the 90-degree window
  it became the whole projection - 13 m to 155 m and 5 m to 144.6 m along in
  one tick, which tripped the off-route guard and stopped the aircraft with
  "we're off the cleared route" about a route it was standing on (EBLG
  2026-09-22). A facing pick must be within `MAX_JOIN_M` or nearer than the
  plain answer.
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
  landing lights lined up; landing AND taxi lights off passing 1,000 ft AGL
  (`lights_climb`, `after_climb`: the taxi light lit for the taxi has no
  other exit, and on the 172 it shares the wing cluster with the landing
  lamp); at the vacate landing off and taxi on
  (`after_vacate`); all off at the stand.
  **The autopilot's own capture holds the level legs where the airframe
  declares it** (`ap_altitude_capture`, `circuit::circuit_dial_ft`,
  `ap_holds_level`): the level target goes on the dial, a CAPTURED hold on
  that dial owns the leg and the VS law stands off, a new level is a new dial
  plus VS, the glidepath parks the dial above. Per airframe because it rests
  on the stock VS command leaving ALT (unproven on the 737); an autopilot
  that refuses VS for 8 s is said once (`vs_refused`). The authored route
  does not do it yet.
  **The sim's heading mode never reverses a turn in progress** (LXGB
  2026-09-14, five minutes of orbits): a bug stepped to the OUTSIDE of a
  bank goes round the long way, so every pursuit steer goes through
  `circuit::bug_toward` (bug on the nose until the wings are level, then the
  short way), and from level the bug leads the nose by at most 25 degrees
  until the bank is established the right way (`TURN_ENTRY_LEAD_DEG`): the
  rectangle circuit bypassed the door and slewed 90 degrees off the nose
  from wings level, and the SF50 flew 13 of its 30 circuit turns as a 270
  right for a 90 left (ETND 26, 2026-09-23, a final handed over 170 m off) and the authored follower has a closure watchdog
  (`authored_stalled`). The authored LXGB route JOINS at the nearest leg
  (`join_waypoint`), climbs through `climb_limit_fpm` like the rectangle,
  and is terrain-probed along its LEGS every 150 m (six waypoints on sea
  either side of the Rock hid the ridge between them): terrain under a
  pattern-altitude leg raises the pattern altitude (`raise_for_terrain`,
  said aloud), under a descending leg refuses, and a leg is checked
  against the altitude the CLIMB can have reached at its start
  (`climb_reach_ft`), never the altitude the route will eventually hold
  (2026-09-15: cleared on paper by 1,010 ft, crossed climbing by 454). The
  Rock's downwind is a fixed distance from the runway (`LXGB_ROCK_CLEAR_M`,
  south of Europa Point, over the Strait), never a multiple of the
  airframe's pattern width. The takeoff sets the trim
  into the takeoff band before the brakes come off (a nose-down stop trim
  is what the AP's climb lost to at the engage: -1,040 fpm at 325 ft).
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
- **The follower's vertical has three owners, in this order**
  (`procedure::vertical_tick`, pure and tested): the PLAN's constraint, paced
  to its top of descent against the last COMMANDED altitude (`prev_cmd_alt`)
  and never against the leg constraint it is about to become - conflating them
  defeated the deferral after one tick per leg and started the 5,300 ft
  descent to JRZ 56 nm out, into the Sierra de Grazalema (2026-09-13); then
  the PILOT's altitude, HANDED to the follower (`procedure::take_altitude`,
  from `fly_to` and `level_off`) instead of written to the dial, because a
  follower that rewrites the dial every tick undoes a spoken override in
  500 ms and the capture watcher then retires itself - sticky by design, no
  self-expiry; then the GROUND AHEAD, which outranks both. Nothing the
  follower commands goes below the highest terrain on the track
  (`terrain_floor_ahead_ft`: 16 probe points, 15 nm or to the fix, every 10 s,
  plus 1,000 ft; skipped on an approach, whose platform altitudes own the
  descent; retired for the flight after TWO unanswered probes with the sim LIVE - a
  probe that fell into a pause is no verdict on the plugin, `terrain_probe_fatal`;
  one did, 2026-09-15, and the floor was off for the whole flight). A plan's
  altitude is only as good as the ground under it, whoever handed it down.
  **The follower flies the ALTIMETER** (`altitude_ft_pilot`): the dial, the
  plan, the chart and the controller all mean indicated feet, and on a hot
  day true elevation runs hundreds of feet above it (LINTO, 2026-09-14:
  7,012 true, 6,838 indicated, VS zeroed, ALTS never captured). Only the
  terrain floor is true feet, converted through the live offset
  (`floor_indicated`); the departure check-in's "passing" and the arrival's
  platform check read the altimeter too. A wrong altimeter setting is
  therefore a wrong altitude (EGSS 2026-09-22: the 737 loaded with its baro
  at 0.0, a 350-press cap moved it 3.5 inHg per ask, every caller said
  "set", and the follower "levelled at 4,600 ft" at 17,000: `set_altimeter`
  now steps in rounds EARNED by progress, `keep_stepping`, and
  `AltimeterSync::line` says "did not take" with both numbers), so the crew
  keeps it right: ONE rule for the
  correct setting (`instruments::correct_setting_inhg`: QNH below the
  profile's transition altitude, standard above) behind the spoken tool,
  full auto's departure set, the enroute follower's arm
  (`Plan.transition_altitude_ft`, said only when the knob moved) and the
  monitor's `AltimeterSetting` check (0.03 inHg, 20 s sustain): with an
  automation flying it RESETS the knob (`Check::fix_write`) and says so;
  with the pilot flying it is the pilot's knob, advised once per episode,
  never re-asserted.
  An autopilot target is judged against that SAME floor, not just the ground
  under the wheels: a climb still into it is RAISED to the floor and said
  aloud, a descent below it refused with the number that works.
- **Autoland takes the autopilot out at the wheels**
  (`disconnect_autopilot`, at the rollout latch): a heading servo on the
  ground cannot close its error and winds the yoke over (EGCC, 2026-09-21).
  **The arrest takes it out at the roundout gate** (`arrest_hands_step`,
  blackbox `arrest_hands`; LFPG 09R 2026-09-22, -672 fpm at 0.1 degrees of
  pitch): the elevator flare writes `xcopilot/pitch` under the joystick
  override and the engaged pitch servo writes the yoke back the same frame,
  so every 737 flare since the elevator law shipped held 0.35 of elevator
  for nothing and the sink followed the VS dial's slow shape. The channel is
  proven with the servos OFF (the takeoff rotates on it). So at the gate:
  `servos_off_any`, pitch on the elevator channel, the runway track on
  `xcopilot/roll` (the takeoff's `WingsLevel`, a shallow bank toward the
  crabbed heading), power on the governor, the arrest ticking at the
  flare's 100 ms, the VS watchdog stood down; a balk or leaving the band
  puts the servos and both modes back; the rollout latch finds them off.
  **A debrief touchdown is the steepest sink in the last 2.5 s**
  (`instructor::touchdown_sink_fpm`): at 1 Hz the last "airborne" sample
  was already on the oleos and the debrief said -221 for a -690 arrival.
  **In the flare the column only comes BACK** (`FlarePitch::step`'s
  `ratchet`, the Flare phase only; the roundout above it stays free both
  ways): a tracker that BEATS its target relaxes, and forward column a few
  feet up is a nosewheel-first landing. EGLF 06 (2026-09-22) the arrest
  worked and then handed the runway back - it pulled the 737 from -511 to
  -102 against a -197 target, the error went negative and the elevator came
  back 0.35 to 0.185 at thirteen feet, while `FlarePower`, unlocked by that
  very arrest (sink now inside `hold_sink_fpm`), walked the lever 0.29 to
  idle over the last 3.4 s. The thrust left DURING the arrest instead of
  before it, the sink rebuilt to -358 and worsening, and the law was
  saturated at `FLARE_PITCH_MAX` with nothing left. The cap still bounds
  the attitude, so the ratchet cannot buy a tail strike; a float it can buy
  is the overflight guard's to judge. **But not through a climb**: a BALLOON (vvi above zero) eases the pull toward neutral, never a push, and a flare stays a flare until the top of the roundout band (`hold_flare`; above it the balk is `balloon`): the Citation held its pull 28 to 58 ft, flipped back to final, was pushed at -1,200 and landed at -1,031 fpm (EDDV 27L, 2026-09-22). Still owed: the retard is keyed on
  the sink the arrest is removing, so it fires latest exactly when it
  should already be done.
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
  says what is actually held). **The altitude dial is parked at the arm**
  (`parked_altitude_dial_ft`: 1,000 ft above the arm altitude, rounded up,
  VS re-selected) - autoland flies the vertical through the VS dial and
  never touched the altitude dial, so the plan's last constraint sat 130 ft
  under the LEAX arm and the AP's capture ignored -1,200 for six seconds
  ("not holding the descent", 2026-09-15). **The glidepath aims at the
  pavement's own elevation** (`aim_elevation_ft`: the threshold and the
  touchdown aim probed from the sim's mesh at the arm, the higher used when
  it differs from the apt.dat header by more than 20 ft and less than 300,
  said aloud, blackboxed `threshold_elev`): LEAX's header says 160 ft, its
  30 threshold lies at 117, and the 172 crossed the numbers 99 ft up and
  balked "overflew the runway" 300 m in. **A low flare with room to stop is COMMITTED**
  (`autoland::commit_flare`, blackbox `committed_flare`): below the roundout
  band an `overflew_the_runway` verdict does not stand while the pavement
  left covers the airframe's OWN landing distance, because a go-around from
  an idle flare cannot climb (the vertical law climbs only with speed in
  hand). ETND 26 (2026-09-23): the SF50 ballooned to 32 ft, the float
  allowance (1,244 m less the Baron's 900 m floor) ran out at 344 m with
  900 m ahead, and the go-around met the runway at about -740 fpm. A BALLOON is
  committed the same way below `BALLOON_COMMIT_FT` (100 ft), coming back
  down at the roundout's sink on the approach's own thrust
  (`balloon_power`), and the flare's ratchet lets go when a balloon is
  `BALLOON_LEAD_S` away, not once it is climbing: EDLP 24 (2026-09-24) the
  SF50 held 0.3 of elevator to +850 fpm and 52 ft, balked `balloon`, and the
  go-around from idle at 81 kt, given 0 fpm by a law that climbs only with
  speed in hand, stalled onto the runway at -1,568 fpm. **The flare's elevator law is per airframe** (`flare_elevator:`,
  `autoland::FlareGains`: gain, integrator, cap, push, and a balloon that
  EASES the pull instead of dropping it to the trimmed attitude); the SF50's
  numbers were read off the EDLP 24 trace, not fitted (2026-09-24). Still
  owed: a go-around law that pitches for SPEED when a
  balk is slow and low instead of holding 0 fpm into a stall. **A tower's go-around order outranks
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
  (`Guidance.balk`, on the `going_around` event) so the debrief says why.
  **The landing arrest leads the autopilot's lag** (`vs_dial_fpm`, LXGB 27
  2026-09-14: the 172's VS mode follows its dial as a 4.5 s lag, longer
  than the flare itself, and three landings in a row touched down at the
  roundout's rate): in the roundout and the flare (`Guidance.arresting`)
  the dial is target + gain x (target - predicted sink), capped; above the
  band the dial is the target. **The gain, the cap and the trend horizon
  are the airframe's** (`ArrestLeadProfile`, default the 172's 2 / +300 /
  0 s; the 737's VS mode is two 1.5 s lags with 2 s of dead time and the
  172's numbers pinned its dial at +300 into a balloon and an overflight
  go-around, EGCC 23L 2026-09-19: it declares 1.5 / +100 / 1.5 s), and
  **the arrest's gates and targets are seconds of sink, not feet**
  (`arrest_gates_ft`, `arrest_targets_fpm`: 25 / 50 ft and -250 / -100 at
  a 172's speeds, 35 / 76 ft and -420 / -175 at a 737's). Over the
  pavement above the band the path's own sink continues (the unsigned
  glidepath distance used to turn the level-hold on at 60 ft), the lever
  never advances in an arrest, and the overflight allowance is geometric
  and knows the recited landing distance. The tick blackboxes `cmd_vs`,
  `vs_dial`, `vvi` and `vvi_trend` so they can be told apart; the sortie
  tests fly a first-order 172 plant AND the 737's second-order plant with
  its dead time through the real law, with the flare's throttle-cut sink,
  without which a pure lag passes the old law.
  **Every planned arrival hands its runway down**
  (`EndAction::Approach { rwy_id }` to `run_autoland`'s forced runway): the
  self-pick is the emergency path and at a field with parallels it takes
  the nearer threshold (ILS 12 flown, landing on 13, cleared for 12). The
  self-pick itself (`autoland::self_pick`, `landing_score`) takes the
  AIRFRAME's minimum (`self_pick_min_length_m`, landing distance x 1.3, never
  under 1,200 m), each field's METAR surface wind (a tailwind costs half a
  mile a knot) and a small bonus for the longer parallel. The
  tower's clearance outranks the plan for an end at the SAME field, said
  aloud and blackboxed `runway_from_clearance`, and it is watched THROUGH
  the approach, not read once at the arm: the final report that earns the
  clearance goes out after the arm, so its answer ("negative, runway zero
  niner ... cleared to land", LXGB 2026-09-14, 27 flown into a 21 kt
  tailwind) arrives after any one-time read. Every tick autoland reads
  back an owed landing clearance (`party_line::readback_owed`) and, on an
  arrival-shaped clearance for another end of this field
  (`clearance_switch`), re-arms itself forced onto it; a superseded loop
  leaves `ACTIVE` to its successor. The arrival's wind pick reads the
  DESTINATION's METAR before the aircraft's own wind datarefs
  (`weather::surface_wind`, `arrival_wind`; the datarefs at 30 nm and
  7,000 ft are the winds aloft), blackboxed on the `arrival_*` event.
- **A reposition is a PATTERN, never a point** ([src/reversal.rs](src/reversal.rs),
  EGTE 2026-09-21: a 737 re-armed from the 26 final onto 08 homed on a
  centreline point, called "final" flying away from the runway, overshot by
  1,880 m and balked at 94 ft, twice). Outbound on a line PARALLEL to the
  centreline three sizing radii out, base, inbound on the final's own
  pursuit; the latch's Reposition stage is HELD until the pattern ends, and
  an ordinary approach starts one when `reversal::needed` (a final called
  more than 75 degrees off the course, or an aim point it would reach
  outbound). **The pattern has a height rule** (`reversal::too_high_to_turn`,
  EGSS 2026-09-22, final turned 12,600 ft above the field): the downwind
  extends while the rest of the pattern cannot lose the height (25 nm cap,
  said once), and far above the platform it descends at the airframe's
  enroute rate capped at 1,500 fpm (`pattern_descent_fpm`). An inbound leg
  past the threshold is OVER (`reversal::overflown`): left alone it flew the
  runway heading 52 nm away, the cut flipping at every centreline crossing. Sized for a SHALLOW bank on purpose; platform in MSL, fixed at
  the start (a radio-altimeter platform chases the hills); half flap and
  `pattern_ias` on the downwind; three patterns, then a give-up. **The
  stabilised gate is a thousand feet and it is about TIME**
  (`unstabilised_at_gate`, `reversal::predicted_residual_m`): position alone
  calls 1,880 m at 3.3 nm interceptable, and a jet with 74 s to run arrives
  207 m off. **A missed approach is never re-aimed** (`clearance_switch` takes
  `going_around`): the go-around's own ClearedToLand to Inbound published a
  flipped runway in use and replaced the climb-away at 60 ft. **The receding
  guard measures from as far out as the approach has legitimately been**
  (`recede_ref_nm`). **Flaps do not cycle on a placard**
  (`flap_held_within_limits`). **A give-up hands the SPEED to the governor**
  (`hand_back_speed_kts`): the flat 0.75 lever took a level 737 to 346 kt.
- **An approach is a straight-in to THIS runway or it is not offered**
  (`nav::straight_in_legs`, LOWI 2026-09-24): a runway fix for the exact
  designator (a grass "26R" never inherits "26"), no RF arc before it, the
  legs cut at the threshold (the missed approach is never the approach);
  circling procedures are not approaches. The arrival picks hard runways and
  tries another end with a flyable approach (tailwind 10 kt or less) before
  a visual landing, and flies the tower's inbound runway when it has one it
  can fly (`tower_arrival_runway`). A plan cruising above the ceiling is
  refused at the arm.
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
  **The VS watchdog excuses a slow autopilot, never a dead one**
  (`vs_not_held`: 700 fpm off the command AND not closing on it), and its
  line names what is commanded (descent, climb, level).
- **The centreline cut is sized for a CLOSURE TIME, not an angle per metre**
  (`autotakeoff::roll_target_heading`, ONE rule behind the takeoff roll and
  autoland's rollout; `XTK_GAIN_REF_KTS`, `XTK_LEAD_S`): a fixed 0.4 deg/m
  winds the outer loop tighter the faster you go - 7 s of closure at 40 kt,
  under 2 at 150 - until it sits on the inner loop's own lag and the pair
  weaves. Both ends of one sortie did (2026-09-22): EGLF 24 wandered +1.7,
  -4.6, +10.1 m and LIFTED OFF ten metres right of the centreline, and
  LFPB 03's rollout wove +3.1 / -5.7 m through four degrees of heading.
  Above the reference pace the gain scales by ref/gs; below it nothing
  changes, so the low-speed roll stays as tuned. The cut is also taken
  against where the offset WILL be (`XTK_LEAD_S`), the damping autoland's
  final has had for a year and the roll never got. The rollout blackboxes
  `rollout` per tick - it kept no record at all, and the weave had to be
  reconstructed from 1 Hz `state` samples - and its steering law is handed
  the REAL tick time, having been given a hardcoded 0.4 s while the loop
  ticked at 300 ms plus two round trips.
- **A runway start is a lined-up aircraft, and it still calls**
  (`tower::Ctx::own_on_runway`, full auto's runway-start branch): the sim's
  spawn on the numbers used to skip the call, the clearance, the squawk and
  the strobes, and the A330 flew Glasgow's zone uncleared (2026-09-21).
- **The crew works the radio as full auto flies** (same sortie): a
  conditional line-up waits on `party_line::line_up_condition_clear()` (the
  world's traffic picture, published by the release watch every tick) before
  crossing the hold line; the after-takeoff handoff is consumed via
  `take_departure_handoff()` (readback on the old frequency, retune, check-in
  with altitude); the region boundary's "radar services terminated, squawk
  seven thousand, contact X Tower" via `take_region_handoff()` (survives
  `Station::install` on purpose; readback, 7000 through `SetTransponder`,
  retune; the arrival stage drains it without a readback; with the pilot
  flying the monitor says once what is still undone, `region_coach_line`);
  the inbound call waits for the destination's station swap
  (`home_tower_khz`) before it goes out: 45 s at the arrival's start, then
  DEFERRED and made at the swap while the approach flies (the region's
  nearest field held the frequency for ten more minutes across the strait
  and the call went into the void), before the final report at the latest.
  **But the ENROUTE leg makes it first when it can** (`inbound_due`,
  `INBOUND_CALL_NM` 10 nm, `INBOUND_MADE`): once the region has handed us to
  the destination's OWN tower (its apt.dat row, never whoever holds the
  frequency) and the field is inside 10 nm, or the moment that tower asks
  "say callsign and intentions" (`party_line::take_zone_challenge`, drained
  every pass) - a plan whose last fix is a mile from the runway flew
  Granada's whole zone through three challenges before the arrival stage
  could call (LEJR-LEGR, 2026-09-15). A crossing field's challenge is
  blackboxed `zone_challenge`, never answered "inbound". **The crew's retune to the destination's tower ARMS
  that station** (`station::arm_station_for_freq`, the pilot's "you call
  it, you get it" rule): the region pick weighs published services, so a
  tower-only strip beside a primary never wins the sky over its own runway
  (LEAX 123.5 against Malaga's five rows, inbound and final into the void,
  2026-09-15). A descent waits for its top of
  descent (`descent_target_ft`), and THAT point is the only one announced
  while the follower flies (`procedure::planned_tod`: the monitor's
  `tod_ahead`/`tod_passed` and the spoken "how far to top of descent" read
  it before the plan's ring; the ring said "start down" three minutes
  before the hands moved, LEJR-LXGB 2026-09-14). **While a follower flies
  the ring is not a FALLBACK, it is silence** (`Snapshot::follower_flying`,
  gating `tod_ahead` / `tod_passed` / `high_on_profile`): between the legs
  that have a descent the follower publishes no point at all, and the ring
  filled the gap on its own geometry - four calls on one arrival, from two
  geometries, one of them a Caution, over a descent flown correctly leg by
  leg (EGLF-LFPB 2026-09-22). **And a level the plan handed down is not a
  level to argue with**: the step-climb advisory is silent while the
  follower has a top of descent published, and a cruise that wanders under
  the autopilot is still a cruise (`CRUISE_WANDER_FPM`; at the old 300 fpm
  gate every wobble ended the episode and re-armed it, and the same advice
  went out three times in six minutes, twice naming the level the plan had
  just left), a cruise fix within
  300 ft of cruise IS cruise, a JOIN projects along the nose (`project_signed_facing`), and the
  taxi law's idle-and-brake needs the speed to actually be over by 2 kt
  unless the build is a surge (`SURGE_KTS_S`).
- **A descent that outruns the governor gets drag, then a shallower rate**
  (`procedure::descent_drag`: speedbrake at 8 kt over the target where the
  airframe declares `arm_spoilers`, in at 3, 75 % of the rate at 12 over and
  half at 20; stowed on every exit). 2,000 fpm at idle reached the last fix at
  278 kt (EGTE, 2026-09-21). **The inbound call is keyed on the tower's
  frequency** (`call_inbound` tunes COM1 first): a region NOTICE near a field
  retunes nothing, and the call went out on Carlisle's, marked as made.
  **The transition altitude is the FLIGHT's** (`instruments::FlightTransition`,
  from the plan's `origin/trans_alt` and `destination/trans_level`, published
  at the arm, resolved by every consumer of `correct_setting_inhg`): the
  profile's figure is only the default.
- **Taxi guide** ([src/taxi_guide.rs](src/taxi_guide.rs)): early gentle cue,
  quiet inside the 60 m where the compliance watch takes over - never two
  voices on one mistake; gates on `pushback::ACTIVE` and along-track progress
  direction. On-demand "which way?" is question-shaped so a readback never
  triggers it.

## Helicopters: the hover loop

Rotorcraft withhold every fixed-wing law (`tools::ROTORCRAFT_WITHHELD`); a
helicopter flies on its own loop ([src/hover.rs](src/hover.rs), blackbox
`hover`), measured on the S-76C before and while it was written (EDDV,
2026-09-22/23):

- **The split is the classic one: cyclic to the autopilot, collective to
  us.** The AP engages only with the stick trim / inverter on and never moves
  the collective (no autothrottle); `servos_fdir_off` disconnects it, plain
  `servos_off` does not. The collective is `prop_ratio[0]`, written INDEXED
  (element 1 moves by itself).
- **The AP's hover mode is a groundspeed HOLD, and it captures**: engaging it
  (`sim/autopilot/hover`, fired only when `hover_status` reads off - it may
  toggle) writes the velocity of the moment into `hover_vector` (x east, z
  south, m/s). A recapture while drifting told it to hold the drift; the
  loop owns the vector: command + `DriftTrim` (an integral on the measured
  `local_vx/vz` error, capped 3 m/s, reset on the ground), rewritten when it
  moves. Asked for zero it still ran forward at 4 to 8 kt without the trim.
- **A moving target is an air taxi, a departure, a circuit**: the hover mode
  followed a moving vector to 0.1-0.7 m/s at 5 to 40 kt. Speed slews at
  `RAMP_ACCEL_MPS2`, the track at 3 deg/s (`slew_track`), and the goals are
  `Hover`, `Land` (holds height until slowed, then down), `Depart` (level
  until `DEPART_CLIMB_MPS`, the height-velocity diagram, then 500 fpm) and
  `Fly` (an orchestrator's vector). **The mode lets go at about 47 kt** and
  will not re-engage that fast: `DEPART_MAX_MPS` is 40 kt, faster is the
  cruise modes' (not built).
- **`groundspeed` carries the vertical speed** (1.27 m/s at 250 fpm): drift
  and error are judged on `local_vx/vz`.
- **The pedals are a yaw-RATE command with a heading hold** (0.1 of pedal =
  4.7 deg/s, neutral holds the heading to 0.01 deg, measured in the hover):
  in the hover the aircraft keeps its own heading; above `PEDALS_ON_MPS` the
  nose is held on the track through the plugin's `taxi_steer` channel by a
  law that asks a RATE (`HeadingHold`), never damps on `R` (a D term at 2.4
  of loop gain through a quarter-second tick rang the tail until the
  `PEDAL_FAULT_DPS` latch stood it down). Without it the S-76 flew a turn 45
  degrees sideways and a leg backwards.
- **The collective follows the power curve** (`level_collective`: 0.66 in an
  OGE hover, 0.46 at 30 kt, 0.42 at 72): the trim is moved by the curve's
  slope as the commanded speed changes, and an in-flight takeover starts the
  trim at the curve's figure, not at the lever. Ground effect is the trim's
  (0.555 at lift-off, 0.582 at 9 ft; KI 0.12 so it chases the rise).
- **In the air the autopilot is never simply turned off** (`hand_back`):
  above 43 kt the cruise modes at the heading and altitude of the moment
  (altitude hold at 35 kt drifted backwards and sank); slower, the hover
  mode asked for zero; the lever at `steady()`, near the power curve. Twice
  an AP-off hand-back left an S-76 with nobody flying it.
- **The collective trim is bounded to the power curve** (-0.15/+0.20 of
  `level_collective`, the lever never 0.05 under the band): an integrator on
  a big error ran it to 0.9 on a takeover, then to 0.0 on the landing asked
  in the climb that followed, and a 3,100 fpm plunge crashed it (ETHC,
  2026-09-23). **The rotor guard** caps it too (`droop_ceiling`: no higher
  below 97 % Nr, lower below 94 %); a lift that droops 5 s low down is set
  down. **The start reads every throttle back at FLY** (engine 2 sat at 0.61
  and a heavy lift drooped the rotor to 83 %), and **locks the tail wheel**
  first (read, then the TOGGLE; the engage command does nothing on the
  S-76): unlocked, torque spun it on the ground at 20 deg/s with the brake
  set until the engines were cut.
- **Airborne requests take over** below `TAKEOVER_MAX_MPS` and
  `DEPART_MAX_FT` ("hold position", "set it down", "hover at N"); a drift in
  a hover becomes a set-down, a departure that stops following its command
  stops to a hover at the height reached.
- **The circuit** ([src/heli_circuit.rs](src/heli_circuit.rs), blackbox
  `heli_circuit`): laid out from the take-off spot and the nose (upwind,
  crosswind, downwind back past the spot, base), flown as `Fly` vectors at
  300 ft / 30 kt, and a final whose speed tapers to stop on the spot at
  `FINAL_DECEL_MPS2` on a 6-degree path LED by the height loop's lag (the
  first final ran 30 ft high), then `Land`. Two flown, landing 1.9 and 6.4 m
  from the spot. Any goal change from outside ends it.
- **The route** ([src/heli_route.rs](src/heli_route.rs), blackbox
  `heli_route`) flies the SimBrief plan and is the whole flight, as full
  auto is: the engine start from cold, the lift, a hover-loop departure to
  1,000 ft AGL at 35 kt, the hand-over to heading and altitude hold
  (`hover::release`, the AP never goes off) with [`SpeedLaw`] on the
  collective to 120 kt, the bug stepped only past 2 degrees (rewritten each
  tick it jittered the yaw), the slow-down on the distance still to FLY to
  38 kt, a hover-loop takeover aimed along the captured velocity, a 6-degree
  final to the helipad nearest the inbound fix (fixes within 10 km of it
  dropped: CEL lies past ETHC), then the aeroplanes' own taxi-in. Flown
  EDDV-ETNW end to end (7.3 m from the point), then ETNW-ETHB from cold with
  nobody touching it: start, lift, cruise, pad H1, and the tower's taxi-in to
  the stand the pad sits on. It takes a local plan first (as full auto
  does), refuses a plan that starts more than 5 km away, calls "ready for
  departure" (60 s bounded wait) and "inbound for landing" on the
  destination tower's apt.dat frequency, and cruises 1,000 ft over the
  HIGHEST ground probed along its legs (plugin terrain, 1 km spacing); the
  last three unflown. **It works the radio with the aeroplanes' code**
  (`full_auto::get_zone_clearance` before the departure call,
  `obey_departure_handoff` / `obey_region_handoff` from the fly loop, off
  the control tick; source-pinned by
  `every_mission_that_leaves_a_field_works_the_radio`): EDVW-ETND
  (2026-09-23) left on a takeoff clearance alone, ignored the region's
  handoff to Eimbeckhausen, and the crew called its own zone "without
  clearance" four times. Transit fields' challenges are still unanswered
  (full auto's gap too). **The inbound call is the aeroplanes' sequence**
  (tune, `arm_station_for_freq`, pin, wait up to 45 s for the station to be
  the destination, then call; spawned off the tick): into ETND it went out
  15 s before Diepholz became the station and the final was challenged twice.
  **The approach comes down at speed** (`Phase::Approach`, 2026-09-23): 70 kt
  on the collective, the autopilot's vertical-speed mode down a 6.5 degree
  path (`approach_target_ft`, `approach_vs_fpm`: the target's own rate fed
  forward, 2 fpm a foot) to a gate 500 ft over the pad, lifted to clear the
  ground still ahead by 400 ft (`gate_agl_ft`, probed at the approach's
  start); the dial is set only with the mode's engage (moved under altitude
  hold it leaves the pitch unheld); the slow-down is level at the gate and
  the hover loop takes over where the final's own path from the gate begins
  (`approach_slow_m`), never above it (`final_command` dives to its path).
  Before, the whole descent was the hover loop's at 38 kt from the cruise
  height: 5.5 min from 1,000 ft, 8.3 into ETND from 2,000 ft, 7 of them on a
  final a headwind held to 21 kt. No pad elevation (no probe), the old
  slow-down at the cruise height. Offline only (sortie test on a lagged VS
  plant); the final's GROUNDSPEED cap in a headwind is still owed, pending
  whether the hover mode's 47 kt let-go is airspeed or groundspeed.
- **Parking is the aeroplanes' tool** (`taxi_to_parking`, unwithheld for a
  rotorcraft): the same call to the tower, the same clearance; only the drive
  differs, a hover taxi of the cleared route (`heli_route::park`, 12 ft, 10
  kt, slowed for corners by `corner_factor`). For the TOWER a helicopter
  under 100 ft is on the ground (`hover::tower_on_ground`), so a hover taxi
  is not an uncleared takeoff.
- Words: "hover at N", "set it down" / "land the helicopter", "move forward
  at N knots", "hold position", "take off" / "depart to N feet", "turn
  heading N" and "speed N knots" during a departure (the AP's mode tools are
  refused while the loop flies: `hover::blocks`), "fly a circuit".

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
- **Monitor** (an automation flying): read-only cross-check, silent item
  by item (satisfied, held and call-out items alike), every discrepancy
  blackboxed (`checklist` event `discrepancy`), ONE spoken line at the end
  naming what is not as listed (`flow::summary_line`), never a write. The
  mission's laws act and announce as they act; the list audits. A file
  item conditioned on a height the list does not run at (lights above
  10,000 in a list that runs at 2,000 ft) is a call-out, not a test.
- **Instruct** ("talk / walk / guide me through the X checklist",
  `instruct_request`): never writes by construction; says the item, watches
  the dataref for the pilot to do it ("Good."), moves on at "next"/"set" or
  `INSTRUCT_WAIT_SECS`. The first piece of the instructor persona; the
  conversation module's "preview / explain the X checklist" stays the
  read-only recital.

**Who starts one**: the monitor's `flow::Tracker` (sustained conditions,
one-shot per flight, a flight begins at the takeoff roll and ends with the
after-landing list): climb established (`CLIMB_AGL_FT` held `CLIMB_SECS` -
EVERY boundary here is sustained, and the climb was the one that fired on a
single tick, straight into the automation's own thousand-foot flap and light
actions, auditing them half done: EGLF 2026-09-22, "flaps, up on schedule"
at 0.125 and "taxi light, off" still lit, four and six seconds before each
finished), level 30 s above
2,000 ft AGL, descending 20 s from cruise height, below 1,500 ft descending
10 s, rolled out under 30 kt for 10 s. Under an automation the Monitor
flavour starts at once; with a pilot flying the copilot PROPOSES ("Climb
checklist when you're ready?", `PROPOSAL_TTL`) and "go ahead" runs it,
"talk me through it" teaches it, "not now" declines - a proposal never acts
by itself (the cockpit_auto rule). The after-landing list is the exception:
the copilot's own clean-up off the runway, it runs at once in the Crew
flavour, no proposal (asked, 2026-09-22). Section-to-title is the same matcher a
spoken "climb checklist" uses; a file whose titles differ maps them in the
profile's `checklist_flow:`; a section the file lacks proposes nothing.
Panel: `flow::status()` ("checklist: climb, 3 of 8") and CANCEL picks "stop
the checklist" when nothing is flying.

## App modes & cabin crew

`mode.rs`: **both** (default) / **copilot** (no simulated ATC; radio-keyed
speech IGNORED - real ATC owns the frequency) / **atc** (the world only, no
crew) / **instructor** (the crew with its hands off: persona, callouts, ATC
coach, checklist read-and-watch and lessons on; every tool refused). Chosen
from the panel Settings tab, persisted; `XCOPILOT_MODE` sets only the BOOT
value; applies live within ~2 s. The "no crew in ATC-only" gate lives in
the arbiter's enqueue door (`Voice::alert` and friends), not scattered
per-site; blocking `say` is mode-routed in the main loop so "copilot is
off" can speak. The atc crate never reads the mode. **`mode::crew_actuates()`
is the hands gate** (false in atc AND instructor): the main loop's tool
dispatch (one door, before any confirm prompt; the action is named and the
pilot does it), the monitor's `fix_action`s, `cockpit_auto`, `flow::start`
(the Crew flavour becomes Instruct there) and undo all check it; entering
instructor mode runs the airframe-swap stand-down so no loop keeps the
aircraft, and leaving it abandons the lesson.

Instructor ([src/instructor.rs](src/instructor.rs), 2026-09-13): the third
persona, on the crew voice, one ACS task at a time (straight and level,
climb, descent, steep turns, slow flight, pattern, landing). "Teach me
steep turns" BRIEFS with numbers the instructor ASSIGNS from the book and
the aircraft's own state (Vy, the approach speed, the altitude rounded to
100, the heading to 10, a climb to the next 500 at least 1,000 up, a
descent never below ground plus 1,500) so the scorer never guesses the
intent; "begin" re-assigns from where the aircraft is NOW and starts the
scorer; a sustained excursion (3 s) is narrated once with its correction
("Altitude, 120 low. Nose up a touch."), the recovery acknowledged, a
standing one re-said at 20 s; the task completes on its own milestone (the
level-off, both roll-outs, the touchdown, the clock) and reads the grade
back. Tolerances are the ACS's per level (`Task::standards`, private or
commercial, "commercial standards" switches); an axis fails on more than
3 s outside or any excursion past twice the tolerance; a wrong level-off,
a firm (>300 fpm) or drifting (>5 degrees) touchdown fails the task. An
AP-shaped command during a lesson is a RETARGET ("heading two seven zero"
moves the heading axis, `take_target`), never a write. Lessons need
instructor mode ("teach me" elsewhere says so); the panel CANCEL ends one.
**The ground is taught too** (2026-09-14): `Task::Taxi` (pace over the
profile's straight-leg `taxi.kts`, default 15, by more than 3 kt is the
fault, never slow; complete after 10 s stopped past 150 m, FAILED by
rolling through the hold line) and `Task::Takeoff` (the centreline is the
heading captured at 15 kt on the roll, judged from 25 kt to 1,000 ft AGL;
the rotation speed against Vr, 10 under is early and a fail; Vy +10/-5
from 200 ft to the top). **"Teach me a full flight" / "lesson from cold
and dark" is the syllabus** (`Syllabus`, `STEPS`): the ramp lists in the
Instruct flavour by section key (preflight, before start, engine start,
before taxi; a missing one is skipped aloud), the engine waited for, the
taxi graded, the before takeoff list, the takeoff graded, then the pilot's
own flight with the tracker's phase lists run as Instruct at once
(`flow::phase_boundary` checks `syllabus_active`, no proposal), the landing
armed on final once the flight has been above 1,000 ft, 20 s stopped, the
shutdown list, and one summary per graded task. Engine already running
starts at before taxi; airborne is refused. The flow is behind the
`SyllabusIo` seam (`FlowIo` in the monitor, a fake in the tests), so the
whole flight is a plain `cargo test`.
`xvl debrief` prints the lessons AND `grade_flight`: the same scorer over
the `state` records of ANY flight, cut into straight sub-runs at turns,
targets the median the pilot settled on, the final and touchdown judged
against the profile's approach speed - the examiner's read of a full auto
sortie too (LEMG-LEJR: the follower's cruise within 55 ft, the autoland's
302 fpm arrival "firm"). The `state` record carries `bank` and `track` for
it; older logs grade without steep turns and drift. Not done: a demonstration
("show me") with the automations, handbook questions, centreline tracking
on the taxi (the guide's cross-track is not yet a lesson axis).

Cabin crew ([src/cabin.rs](src/cabin.rs)): second intercom persona, no third
PTT/channel. **The cabin is a state machine, not a commentator**: it has its
own mute (`cabin_quiet`: a recovery, the panel, a wreck) and is NOT inside the
monitor's `quiet`, which includes `autoland::ACTIVE` - gated with the world's
narration it was never called from the autoland arm to the rollout handoff, so
it missed its own landing (the first ground sample it saw was the vacate taxi
at 26 kt, under the 30 kt roll test), forgot the flight and announced "cabin
is secure for departure" while taxiing in (EBLG 2026-09-22). A flight that
came down through the descent gate has landed whatever speed the tick catches. The address is the routing (explicit "cabin crew" always; bare
"cabin" only with an unmistakable crew word). Deterministic keyword intents,
"say again" over guessing, chime first, own lazily-loaded voice. Exists
per-airframe (`cabin_crew: true`) + panel staffing Auto/Aboard/Off. "Cabin
secure" is a safety report, exempt from sterile cockpit
(`STERILE_FLOOR_AGL_FT` 10,000). **The cabin works the flight unasked**
(`Cabin::watch`, before `tick`, 2026-09-21: five of the first seven live
calls had earned "say again" and an uncalled cabin said nothing all flight):
secure during the taxi (10 s over 5 kt under its own power, never under a
tug, report 60 s on), silently secure at liftoff on a runway start, the
service through the floor, secure for landing once the descent has given
back 40 % of the flight's peak height (the floor on an airliner's profile;
a flight that never reached 3,000 ft secures nothing), doors and deplaning
at the shutdown after a landing ROLL (a reposition to a ramp ends the
flight silently; an app started in cruise invents no news). Each once per
flight and never after the flight deck made the same call. A bare "cabin
crew?" answers "go ahead" and opens the line 20 s for ONE un-addressed
cabin call (`take_open_line`); "take your seats", "you're released" and
"ten minutes to landing" are calls too. The cabin's voice is the panel's
second picker (`xcopilot/opt/cabin_voice`, same roster; the copilot's own
voice is refused, `pollers::cabin_pick`). On a crewless airframe: an honest "no cabin
crew on this airframe", never an improvisation.

## The copilot as a person: timing, persona, rapport

The words stopped being the machine tell once `readback::vary_ack` shipped;
TIMING, memory and judgement were what remained (2026-09-23). Three modules,
every rule deterministic and built from a measured fact:

- **Timing** ([src/tempo.rs](src/tempo.rs)): a `Turn` per intercom
  utterance. The first reply waits a jittered 280-480 ms from the end of the
  CAPTURE (so the transcription counts toward it; never when busy, never
  typed), and a turn with nothing queued on either channel after 1.6 s hears
  "stand by" (never over a content-routed radio call: `set_radio_turn`).
  The monitor publishes `busy_now` (the roll, under 1,000 ft, a landing,
  takeoff or recovery loop, a checklist or lesson talking); the copilot voice
  renders each line at `line_tempo` (a warning quick and flat, chat a touch
  looser, brisk when busy; ONNX only, the cabin untouched).
- **Persona** ([src/persona.rs](src/persona.rs)): `XCOPILOT_CREW_CHATTER`
  (quiet / normal / chatty) gates what is VOLUNTEERED, never an answer, a
  readback or a safety call. The idiom follows the VOICE's accent (en_GB
  "cheers", en_US "you bet"; `XCOPILOT_CREW_DIALECT` overrides), because one
  person does not change accent at a border. Small talk is a CLOSED set (the
  whole utterance less address words); "thanks, gear up" is still gear up.
  `XCOPILOT_PILOT_NAME` is used on the first praise of a session only.
- **Rapport** ([src/rapport.rs](src/rapport.rs), blackbox `rapport`): a state
  machine fed by the monitor tick that SEES every tick, autoland included.
  The landing word is the instructor's own touchdown scorer, said once the
  roll is under 30 kt, and replaces the old spoken "Landing summary: last
  airborne sample..." report (still blackboxed as `landing_summary`). Praise
  goes to the pilot, never an automation; a hard landing (600 fpm) is said at
  every level; a bounce is counted at the SECOND contact, so a touch and go
  is not one. Memory: go-arounds per flight (detected, pilot or automation),
  landings per field, "back where we started". Offers (pilot flying only, a
  sentence and never an action): the NEXT flap once the speed comes down
  through its placard, the gear under its limit, the arrival brief 20-40 nm
  out (`rapport::arrival_brief`, METAR, runway into the wind, approaches,
  elevation; answered with the checklist proposal's closed yes/no words).
  Anything deferrable that arrives busy is an ASIDE, said after 8 s of calm,
  several joined, "By the way" once held over 20 s, dropped after 5 min.

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
  (`xcopilot/typed`, consumed by a NUL write). **The Route tab** carries the
  load sheet (plan against actual for fuel, payload and weight; the plan's
  side is the `xcopilot/status/loadsheet` mailbox on the auto feed's gate,
  the aircraft's side the plugin reads itself; "Load aircraft" is request 12)
  and the ACTIVE fix with magnetic bearing, distance and minutes
  (`fms::active_fix`: the FMS's destination entry, or the follower's last
  goto when the plan lacks that leg). The panel renders through
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
500 ms. A COMMAND clears the whole value cache (it can move any dataref, and
the stepped-knob tools rest on the read after it). Volume, not duplication, is where the cost is: `cached=0` in a window
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
flight: aircraft, the numbers, the lessons and the ACS grade of every
straight segment and the landing (`instructor::grade_flight`), every
callout, and the narrative of events an instructor would write down
(crashes, balks, void calls, floored targets, taxi stops). Run it before
diagnosing by hand. **Run tests with `XCOPILOT_BLACKBOX=off`**: any test
that walks a real path records into the live log (22 test lines landed as
"flight 117" on 2026-09-13).

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
