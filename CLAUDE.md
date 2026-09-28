# XVoiceLayer

The voice layer for X-Plane 12: copilot, ATC and cabin crew, local and deterministic. The pilot
speaks, the grammar picks a tool, the agent writes the dataref, the copilot reads it back aloud.
**There is no model at runtime** (retired 2026-09-13): every miss in the logs becomes a rule.

```
🎙️  "Flaps up"   →  STT  →  grammar picks a tool  →  write dataref  →  TTS "Flaps up!"
```

**The evidence lives in [docs/dev-history.md](docs/dev-history.md)**: every sortie, date and number
behind these rules. Grep it BEFORE reworking any subsystem named here. This file keeps each rule and
its one-line why. Each section's pre-diet text is in the history under "2026-09-26 trim:
<section>", so any airport, date or name that left this file can still be grepped there.

## Wire contracts (never rename)

Named 2026-07-30 (repo `dlicudi/xvoicelayer`, dir `~/GitHub/xvoicelayer`). Three names survived the
rename because they are wire contracts:

- the `xcopilot/*` dataref namespace (app and plugin agree on the strings),
- the `XCOPILOT_*` env vars (the pilot's own `.env` and `prefs.json`),
- the plugin binary `xcopilot.xpl` (renamed, X-Plane loads BOTH copies and double-registers).

Package `xvoicelayer`, binary `xvl`. The tracing filter names every crate AND both crew names:
`xvoicelayer=info,xvl=info,atc=info,sim=info` (`src/main.rs`). A line's default target is the BIN
name in the binary and the PACKAGE name in the lib: drop either and half the log goes silently while
explicit-target lines make it look healthy.

## Neighbours

- **Horizon** (`~/GitHub/horizon`): ABANDONED 2026-09-11. The instructor is the third persona here
  (`mode::Instructor`, `src/instructor.rs`).
- **rustdecks** (`~/GitHub/rustdecks`): sibling Loupedeck controller. Hunting a dataref or command
  name, check its `examples/*.yaml` (verified on real panels) before DataRefs.txt. It mirrors this.

## Stack (decided)

Rust: a latency- and audio-sensitive real-time loop.

| Stage | Crate / approach |
|---|---|
| Mic capture | `cpal` |
| STT | `whisper-rs` (whisper.cpp bindings), push-to-talk |
| Intent | deterministic: `src/router.rs` (command / question / follow-up / echo), `src/grammar.rs` (the tool), `src/conversation.rs`, `src/answers.rs`, `src/plan.rs` (questions), `src/dialogue.rs` (back-references); `main.rs` boot and command loop, `src/detect.rs` airframe swap |
| X-Plane write | `reqwest` PATCH to the Web API (`PATCH /api/v1/datarefs/{id}/value`) |
| TTS readback | in-process piper VITS via the `ort` crate |

- **No LLM, local or cloud**: the Ollama fallback was measured useless and a frame-rate cost.
  `xvl analyze` lists the top missed phrasings; each becomes a rule, matcher or profile entry with
  the live transcript as its test. `xvl analyze --now --since 2026-09-20` replays misses through
  TODAY's STT repairs and grammar (the 172's registry); waiver, plan-load and ATC branches live
  outside it. A model returns only as the instructor answering handbook questions, never a fallback.
- **REST for transport, plugin for capability.** Stock datarefs go over the Web API; the C++ plugin
  (`plugin/src/`: `xcopilot.cpp`, `drefs.cpp`, `panel.cpp`, `fms.cpp`; `make install` in `plugin/`)
  adds `xcopilot/*` datarefs driven over the same API. The app never links the SDK.
- **Voices can ride the sim's audio** (`plugin/src/audio.cpp`, off by default;
  `XCOPILOT_AUDIO_BUS=true` pins it, else panel `xcopilot/opt/audio_bus`): "channel|volume|path" to
  `xcopilot/audio/play`, played by `XPLMPlayPCMOnBus` (station on COM1, intercom on the copilot bus)
  so it follows the sim's mix. `xcopilot/audio/cut` is the barge-in, its ONE writer the waiter in
  `tts::play_on_bus`, so a cut never overtakes the next play. A failed post evicts the cached id
  (reloads renumber datarefs) and plays locally.

## Core design rule: curated tool surface, not raw dataref access

**Nothing writes arbitrary datarefs; the grammar picks curated tools.** Each owns (1) its dataref and
range, (2) a **readback** re-read AFTER writing and compared with the INTENT (a silently failed write
must not answer "Roger"), (3) **safety gates** ("gear up" on the ground refuses and explains).

- **Back-references** ("a bit more", "come left ten", "do that again") resolve against the calls
  actually executed (`src/dialogue.rs`): CLOSED vocabulary ("more coffee" is chatter), 120 s TTL,
  automations never re-armed, cleared on swap; they ride the tool path, gates included.
- **State questions** ("is the gear down?") answer from the SAME datarefs the tools write; the
  question OPENER routes them. A judged speed question gets a verdict (`speed_verdict`: governor,
  floor, phase reference), never a bare number.
- Weather away from the aircraft comes from the METAR file (`src/weather.rs`), never the datarefs of
  the air we are in. A named fix is matched fuzzily against the LOADED plan only (`fix_query`).

## Aircraft profiles drive the tool surface

Profiles (`src/aircraft.rs`, `profiles/<id>.yaml`, `XCOPILOT_AIRCRAFT=auto`) are detected from the
profile's `match:` block (`acf_ICAO` AND the `.acf` path, `detect::resolve`, one resolver for boot
and swap), never the ICAO alone: the stock 737 and the Zibo both say B738. An unmatched model of a
known type is generic, said aloud. Plan-type questions use `aircraft::profile_for_type`. They
declare systems and limits; the registry is built FROM them (172SP: no gear tool, per-notch Vfe).
Datarefs and commands stay in the Rust tools; YAML is per-airframe config.

**Numbers are the airframe's**
- **All automation numbers are per-airframe**, the old GA figure when silent: taxi pace (carried BY the
  route), `takeoff_roll_ft` (runways are checked against it) vs `takeoff_distance_ft` (recited),
  pattern numbers, park-brake valve sense.
- **"Enough runway" is ONE rule** (`aircraft::takeoff_needed_m` / `takeoff_needed_now_m`: the roll
  scaled to the weight on the wheels x 1.4, the caller's floor) behind takeoff, circuit, line-up,
  `runway_check` and the preflight: as five copies, a fix reached one (EGPH 2026-09-20). A failed
  fit is SPOKEN by its gate (`autotakeoff::no_fit_line`), never "not on the runway".
- Multi-engine tools take `engines`, act per engine, and read back the LAST element written;
  singles stay byte-identical (element 0, starter 1). `vyse_kts`, `vmca_kts` stay `None` on
  singles: say nothing rather than invent.
- **An unknown type is never silently the 172** (an RV-10 flew the 172's numbers into a stall).
  A `match:` hit loads the profile; else `aircraft::generic_from_acf` reads speeds, flaps, gear and weights
  from the `.acf` (`acf_relative_path` under `xplane_root()`) and the pilot hears "no profile for
  this airframe"; only an unreadable file gets the 172, said aloud. Approach = max(1.3 Vso,
  1.2 Vs). `bus_volts_nominal` scales the start gate.
- **A slatted wing's stall is per notch** (`FlapDetent.vs_kts`, `aircraft::stall_speed_in_config`,
  from the .acf's slat schedule by `detent_stall_from_acf`): the monitor, `config_floor_kts` and
  `speed_verdict` read the same table; with nothing declared it is the straight line (EGPF A330).
- **No automation target below the speed floor** (`aircraft::speed_floor_kts`, 1.2 x clean Vs,
  per swap): `speedhold::arm_flight` and `run_autoland` raise it and say so (`target_floored`,
  `approach_ias_floored`); a wrong number from a profile, plan or pilot is still wrong. The ceiling
  is the airframe's too (`speed_ceiling_kts`, Vne less 15), not a flat 250.
- **A speed hold on the runway has landed** (`speedhold` `TOUCHDOWN_HOLD_S`, `touchdown_idle`): a
  second on the wheels closes the throttle, says so, lets go; else it powers the rollout.
- **A flow runs at crew pace and the voice says milestones** (`jet_start::AirlinerStart::run_steps`):
  fire, READ BACK until it agrees, a beat; a set switch is passed over; every step blackboxed; only a
  profile `say:` is spoken; a switch that never took is named at the end. Twenty presses in two
  seconds is a macro.

**A profile line is a hypothesis until probed**
- Every airframe, `aircraft::profile_references` walks every name in the profile, `probe_dataref` /
  `probe_command` ask the sim, `probe_summary` speaks the missing, blackbox `profile_probe`. A new
  field is probed the day it is added.
- **Rust constants too**: read names come from `datarefs::*`; `cargo test -- --ignored
  real_install_names` checks every `"sim/..."` literal against DataRefs.txt/Commands.txt (two
  made-up names fed fallbacks that impersonated a measurement for months).
- **The plugin side too** (`health::PLUGIN_DATAREFS`, test-pinned): a stale `.xpl` is said at boot
  and red on Health. Editing `plugin/src` is done only after `make -C plugin`, install AND reload.
- **Position is one frame** (`xcopilot/position`, `sim::position::read`): the plugin copies lat,
  lon, elevation, true heading, groundspeed, vertical speed and y_agl after the flight model each frame,
  read in one request; an older plugin falls back to split reads (`pos: split`). Read separately, lat
  and lon paired two frames: 2.9 m of cross-track for a tick on LEMG 13 (#157).
- **A failed read is NO measurement**: a loop skips the tick and blackboxes `null`. A "dataref" may
  be a command; read-only switches need read-before-fire `*_toggle` tools, in full auto's copies
  too. Paired lights are written together (one lit pod confirms half).
- **A read-only KNOB is stepped by its commands** (`TransponderModeKnob` `up_command` /
  `down_command`, `radio::step_knob`: read, step, read, bounded): a write to it is a silent no-op.
- **The gear comes up outside its own warning** (`autotakeoff::gear_up_due`,
  `aircraft::GearWarning` from the `.acf`'s `_gear_warn_*` keys): X-Plane's horn sounds below
  `below_kias` with throttle or flaps past their thresholds. Above 200 ft once silent; always by
  1,000 ft, 10 kt under Vlo, or at the handoff.

**Add-ons and process-wide facts**
- A stock dataref can be an add-on's OUTPUT (its Lua rewrites it every frame): when a tool "does
  nothing", read the Lua. Declared knobs (`transponder_mode_knob`) are turned and CONFIRMED through
  one writer (`src/tools/radio.rs` `set_mode`). A box with several knobs is confirmed from its
  OUTPUT (`TransponderModeKnob.also`, `radio::stock_transmits`), not from the knob turned.
- **Selectors are republished on EVERY swap, including to `None`**: facts must never outlive their
  airframe. `src/flight_state.rs` is the one page: `publish_airframe` sets every selector,
  `stand_down` stops every loop that can hold the aircraft (swap, instructor entry, connection
  lost); a source scan pins it to every `static ACTIVE`. New ones go there, never in a caller.
- **Anti-ice** (`src/tools/anti_ice.rs` `set_anti_ice`, profile `anti_ice:`, else the `.acf`'s
  loads, `systems::IceKit`): read first, read back, an undeclared wing left alone and said; the
  monitor's `icing` switches probes, windows, engines and props in visible moisture at +10 C or
  colder and the wing on accretion with an automation flying, advises a pilot once, and switches off
  only its own claims after 5 min clear.
- **A cart and a contactor are two things**: ground power moves the cart request only when presence
  differs from want (a blind toggle dismisses the cart) AND `gpu_on`, and reports BUS volts.

**A crew configures before it moves** (`src/takeoff_config.rs`, EGPF 2026-09-21): one page for
full auto after the start, the takeoff before brakes off (it WAITS for the flaps), autoland at the
gear. Flaps, trim and the altitude dial (`initial_altitude_ft`: the caller's level, else field +
3,000; a dial the pilot set stays); spoilers and autobrake where `takeoff_config:` declares them.
Read first, left alone in state, read back, named when not taken, one milestone line.
- **Armed for the roll is an Airbus habit**: a Boeing declares `takeoff_speedbrake_down` (lever
  DOWN, raised on a reject, `on_reject`); roll-armed spoilers are disarmed after the gear
  (`after_liftoff`).
- **Cabin air** (`src/tools/cabin_air.rs`, `cabin_air:`): bleed and crew oxygen on for EVERY
  full-auto mission (an airborne resume too) and at the cabin-altitude callout; past the warning
  band a flying follower is handed 10,000 ft. A TOGGLE oxygen button fires only when it reads off,
  and the valve behind it is written too (a hypoxic climb, 2026-09-24).
- **Centre tank pumps** (`src/tools/fuel_pumps.rs`, `fuel_tank_pumps_off_when_empty:`): an empty
  centre with its pumps on and an engine running is switched off by the monitor's
  `centre_tank_dry` with an automation flying, advised once with a pilot flying (the altimeter's
  shape); a main tank never, by `acf_tank_X` (the 737's start pumped a dry centre, 2026-09-27).
- **Seat belt signs** (`src/tools/signs.rs`): on before anything turns, off through ten thousand,
  on again through it and for the arrival; an add-on's own switch is declared (`seat_belt_signs:`).

**Unasked actions** (`src/cockpit_auto.rs`: `auto_at_night` interior lights, GNS430 ack): a state
machine over conditions (powered AND dark), one-shot per spell, never re-asserted (a watch that
re-fires fights the pilot). "Off" only for lights it lit; the claim drops when the pilot moves the
switch. INTERIOR lights only (test-enforced); sun hysteresis; above the `quiet` gate. Every light
write goes through `tools::lights::apply`. The DG auto-sync is said only when the drift READ BACK
fell (`monitor::GyroSync`); one that does not take is silent and latched off per airframe (the
stock 737 "re-synced" 14 times, 2026-09-27).

## SimBrief

`src/simbrief.rs`, `XCOPILOT_SIMBRIEF_ID`: the one other outbound call, on demand, never realtime.

- **FMS injection is plugin-only**: the app writes `xcopilot/fms/route` (`IDENT,lat,lon,alt|...`)
  then `xcopilot/fms/load` (edge); the plugin resolves idents (`XPLMFindNavAid`), composes an XP11
  plan in memory and loads it whole (`XPLMLoadFMSFlightPlan`), because only a load fills
  ORIGIN/DEST. It feeds the monitor's `DESTINATION` and the fuel check.
- **The follower sequences the FMS** (it flies the bug, never GPS mode): each leg goes as
  `IDENT,lat,lon` to `xcopilot/fms/goto`; the plugin makes the matching entry the destination.
- **The plan is the WHOLE navlog** (`simbrief::fms_route_payload`): the 512-byte `TextRef` is
  thinned from the MIDDLE, origin and destination always kept. A cap on the front of a list
  amputates its end (EGLL-EGPH 2026-09-17).
- **Local planner** (`src/flightplan.rs`, `src/route_finder.rs`; "plan a flight to X", "divert [to
  X]"): the same `FlightPlan` (`PlanSource::Local`), its lateral route decided at plan time: an IFR
  plan over airways (`earth_awy.dat`, the level inside each segment's band, within 1.20x direct, half
  on airways), else direct, dog-legged round P/R/Q volumes the flight would be inside (hull 2 nm
  clear, within 1.30x; one it cannot go round is NAMED, one over a field only blackboxed). The
  arm's `local_route` keeps those points and adds its own cruise, alignment and final fixes. Origin the nearest land-runway field; destination by
  ICAO (3 letters also with K) or name words. Cruise by `cruise_altitude_ft` (distance, terrain,
  ceiling, oxygen, semicircular, VFR +500 unpressurised). Fuel from `cruise_burn_kg_hr` or unsaid.
  A short destination runway is REFUSED. Only an explicit refresh replaces it.
- **"Fly the flight plan" is preflighted AT THE ARM** (`full_auto::preflight`, blackbox
  `preflight`): both runways (`takeoff_needed_m`), `weight/m_total` vs the plan's MTOW (else max
  gross), tanks vs the MINIMUM takeoff fuel, `weight/m_fixed` vs the load sheet (`PlanWeights`). A
  refusal names the number; an unknown is skipped, never guessed. Why: these used to be found at
  the enroute caution and at the hold. **Before it, on the ground and stopped, the arm fetches the
  plan fresh and loads it** (#95): `ArmFetch` says a changed plan or a stale shelf (blackbox
  `plan_fetch`); `arm_load` loads through `load_from_plan` only the half not already aboard (2 %),
  never an airframe declaring `load_sheet: unprobed` / `own_fuel_system` (named; blackbox
  `arm_load`). The FMS gets the plan loaded and preflighted; an airborne resume does neither.
- **A MARGIN is the pilot's to waive, the airframe never** (`full_auto::Gate`): "say fly it anyway"
  only for the runway's 1.4 factor (`OriginRunway`; `BookRoll` and 500 m still hold) and the plan's
  weight limit (`PlanWeight`; max gross holds), by `grammar::waiver_request` within 120 s, for THAT
  plan; the runway waiver lives as long as the mission (`aircraft::TAKEOFF_MARGIN_WAIVED`, set by
  `start`, cleared by every stop), blackbox `waiver`. Fuel, ceiling, destination runway: never.
- **A spoken ask for the plan re-fetches EVERY time** (`PlanFetch::Asked`; the TTL is for fuel,
  TOD, ETA): the pilot regenerates on the website and says the same words. "That's the wrong plan"
  says whether it moved (`plan_change_line`). "Load it" is a separate word; a plan on the shelf
  authorises nothing.
- **The load sheet** (`src/tools/loading.rs`, `load_aircraft`): ground and stopped only; BLOCK fuel
  wings first (`distribute_fuel`, from `acf_tank_rat`, `acf_tank_X`), payload into `m_fixed`,
  refused with both numbers past the tanks or `acf_m_max`, read back; full auto's arm reuses it. The
  loop asks `grammar::load_aircraft_request` first (same shape as the FMS load); "refuel" is the
  fuel truck.

Still owed: issues labelled `owed` + `area:full-auto`.

## ATC: the world, not the crew

An in-process station (`crates/atc`): one mic, one PTT, one speech arbiter. The tower hears an
utterance iff COM1 is on the station AND it parses as a radio call (FSM in
`crates/atc/src/tower.rs`); else the copilot. Frequencies from apt.dat comm rows, never hardcoded.
Config: `XCOPILOT_ATC_AIRPORT` / `_VOICE` / `_RATE` / `_REGION`.

**Hard boundaries**
- The station returns text and can NEVER actuate (`sim::xplane::SimView`, read-only); no
  whisper/cpal/ort in its deps (`cargo tree -p atc` is the standalone health check).
- **The crew/world seam is one page**, `crates/atc/src/party_line.rs`: statics crate-private (a
  crew write is a compile error); a new crew need gets an ACCESSOR, never a `pub static`.
  `assigned_vector` / `landing_handoff_runway` have no consumer yet, on purpose.
- Tests touching a process-wide static hold `SIDE_STATE_LOCK`; setting the mode, `mode::TEST_LOCK`.
- `Station::install()` publishes side state, never `from_airport` (sweeps and tests stay free of
  side effects); `uninstall()` retires it all.
- **A mishear may earn "say again", never a clearance.** Only mayday/pan-pan lands without a final.
- **One callsign for tower and crew** (`atc::callsign::resolve`): `XCOPILOT_CALLSIGN` > the SimBrief
  plan's `atc/callsign` (only on the plan's type) > tail, heavy last; read through a crew provider,
  never stored; a call naming it is addressed (blackbox `callsign`). The pilot's own telephony
  (`airline_telephony`, "WAT=Walker") is asked before the built-in tables, by a second provider.

**Tower behaviours** (each one's sortie is in the history)
- Readbacks enforced for runway-critical items (`XCOPILOT_ATC_READBACK=false` coaches only); the
  zone clearance's readback is suggested, never enforced. A departure check-in is answered with the
  climb it announced (`climb_announced`).
- **IFR clearance** (`Intent::RequestIfrClearance`: "IFR clearance to X", "clearance to <place>",
  "ready to copy"; a bare "request clearance" stays the zone one): destination from the region
  index, else any land field by its ICAO (`FIELD_DIR`), else "say again destination"; SID off the runway in use that JOINS THE FILED
  ROUTE (`nav::sid_for_filed_route`, the crew's own order, via `party_line::provide_filed_route`; a
  route no SID joins is runway heading; no plan: `nav::sid_toward`), never invented; a SID asked for by
  name (`Intent::RequestSid`, `Station::amend_sid`) amends the clearance or the cleared one stands; 5,000 ft or field + 3,000;
  ICAO or FAA phrasing; SID, altitude and code read back ENFORCED; the crew sees it only once
  accepted (`party_line::assigned_clearance`). **Cancel IFR** (`Intent::CancelIfr`,
  `Station::cancel_ifr`) retires the clearance, `IFR_FLIGHT` and the frozen runway.
- **"Radar contact" is the radar unit's, on its own frequency** (`squawk_tick` `tuned`,
  `radar_unit`): tuned to a tower the watch holds it; with a departure handoff armed it is the
  check-in's reply (`check_in_owed`, `radar_contact_at_check_in`); nudges are still relayed.
- **Levels are the field's**: at or above apt.dat's `1302 transition_level`
  (`Ctx.transition_level_ft`) a level is a flight level in clearance, readback and correction
  (`tower::climb_item`); `heard_altitude_ft` hears either form; the crew's check-in says flight
  levels above the flight's transition altitude (`departure_check_in_above`).
- Runway in use: calm wind follows the live stream, arrivals first (`arrival_end`) else the last ghost
  liftoff's end (`departure_end`, `DEPARTURE_STREAM_SECS`); **one strip, one direction**: an
  uncommitted calm pick against the stream turns round to it (`ends_oppose`, `runway_follows_stream`),
  a stated runway into the departures is refused, and **the party line clears nobody against our
  runway** (`own_strip_claim`, `conflicts_with_claim`, in the air too; `party_line_withheld`; LEMG 31
  vs 13, 2026-09-28); FROZEN while a departure or
  arrival is committed, an IFR clearance included (`ifr_runway`), with wind hysteresis (parallels
  flip-flop invisibly); an arrival's STATED runway stands when the wind does not argue: in calm, or within a 5 kt tailwind on no worse a crosswind (`stated_runway_stands`, `note_stated_runway`; LEMG 13, 2026-09-28).
  **The wind picks a direction, the longest parallel the strip** (`best_runway_for_wind`); a
  parallel under 760 m away is its main twin (`close_parallel_main`; EGKK 26R, 2026-09-27).
  LinedUp and conditional line-up.
- Release threads the arrival stream (`DEPART_GAP_NM`; strip clear = LUAW); wake timed from when
  the HOLD went on. **Circuit traffic stays with the tower** (`tower::wants_circuits`,
  `Station.circuits`): "for circuits" on the taxi or ready call withholds the departure handoff
  (`departure_handoff_withheld`) and the radar watch; full auto's circuit says it (`taxi_call`,
  `ready_call`; LEMG 31 2026-09-28); its zone request is cleared FOR THE CIRCUIT, never out of the
  zone (`Ctx.circuit`, `tower::circuit_clearance_lines`: runway, circuit height `circuit_height_ft`,
  pressure, code, "report ready"; FAA "closed traffic approved"). **A reject is a radio call** (`Intent::RejectingTakeoff`, the crew's
  `autotakeoff::REJECT_CALL`): the clearance and all it armed die; a held release lives only in
  TaxiCleared/LinedUp and dies at a roll (`void_takeoff_hold_by_roll`; EGKK 2026-09-27).
- **The tower's book is the runway's**: a landing holds its strip until the vacated report, a
  runway start is lined up, a cleared crossing that keeps moving is legal
  (`station::runway_occupancy_authorised`, the incursion watch's one question; EGKK 2026-09-27).
- **The runway watch ticks whoever flies the final** (`go_around_tick` / `landing_hold_tick` run
  ahead of the monitor's `quiet` block, which closes under `autoland::ACTIVE`); **the crew never lands
  uncleared**: autoland's `UnclearedWatch` asks at 1,000 ft and goes around at 300 ft while the book is
  Inbound (`landing_clearance_asked`, `uncleared_go_around`; EGBB 15 2026-09-28). The strip we LANDED
  on, cleared or not, is ours while rolling off it (`landed_strip`); the incursion shout names the end
  we point down. **One departure on the runway at a time** (LinedUp/TakeoffCleared claim the whole
  strip, no ghost departure within `WAKE_SECS` of our liftoff, a ghost change not seen live is never
  narrated, `PARTY_LINE_FRESH_S`; the wake clock on the departure's own end). The taxi request names
  the cleared runway; departure clears no higher than the FILED cruise (`capped_departure_climb`).
- **A landing is never cleared onto an occupied runway, and the go-around is judged on time**: a
  final onto a ghost on our strip is "continue approach ..., expect late landing clearance"
  (`LANDING_HOLD`), cleared by the monitor once the strip is clear or its occupants move off in time
  (`landing_hold_tick`, `landing_release_due`, `sync_landing_release`); the order (`go_around_due`)
  comes only inside 60 s of the threshold, when an occupant judged from its track (`strip_exit`) will
  not be off 10 s before we cross (blackbox `landing_withheld`, `landing_released`,
  `go_around_deferred`, `go_around_judged`; LEMG 13 2026-09-28: ordered around at 3.8 nm for a
  departure airborne 81 s later).
- **Taxi routes**: Dijkstra over apt.dat 1201/1202; crossings from strip GEOMETRY, never network
  flags; a crossing on the cleared route is never berated. A runway destination snaps to a node
  that can REACH the runway, per CALLER, ON THE DESTINATION STRIP's own pavement (LFPB 2026-09-22).
  A refusing gate names strip and distance (`NoFit::NoneNear`).
- ILS from earth_nav.dat + CIFP; dialect by ICAO prefix (FAA vs ICAO/CAP 413); `qnh_clause` is the
  one pressure seam, `suggest_call` the one source for coach and panel.
- **The tower's wind is the FIELD's** (`station::set_surface_wind` from
  `monitor::feed_surface_wind`): the wind datarefs are the air at the aircraft, and aloft they pick
  the other end.
- A frequency read back is a readback (`frequency_readback`); within 45 s of a region switch it is
  `handoff_readback`, not `void`. **A REPORT spends nothing** (`Intent::RunwayVacated`): the next
  "request taxi" is still the taxi IN.

**Audio and PTT**
- TWO half-duplex channels, intercom and radio. Intercom defers below Warning while the radio is on
  air; a routine line in progress is cut when the radio keys and its remainder follows once quiet,
  perishable (`interrupt_intercom_for_radio`). A Warning talks over and is never stepped on.
  Radio-only cuts (retune, avionics off, crash, replay) never touch the copilot.
- **A position report dies with the position it reports** (`circuit::holds_for`, `stale_reason`,
  judged as it goes out from `crew_radio_facts`): mile-final, final, downwind, base, going-around,
  inbound and the check-in hold only airborne (approach reports above 100 ft, within about a mile
  flown) with a max age; "runway vacated" only on the ground; readbacks and requests keep their own
  rules; dropped = `crew_radio_stale`, identical queued calls collapse (`crew_radio_collapsed`), a sent
  call logs `age_s` (LEMG 13 2026-09-28: "5 mile final" x4, the first in the flare).
- **A crew call is routed on the frequency it was keyed on, before it is spoken** (`say` blocks
  until quiet), else a held readback lands on the next frequency. A live number
  (`CrewCall::CheckIn`) is rendered as it goes out, after a bounded wait on `radio_on_air`.
- With a radio key bound, **physical routing beats content routing both ways**.
- **The command loop never holds a pilot's line behind its own speech**: `say`, the crew-radio
  drain and the tower's reply give way to a line in hand or waiting (`pilot_line_pending`,
  `Voice::say_station_until`); blackbox `typed` `routed` `held_ms` (LEMG 2026-09-28: "my controls"
  routed 26.1 s late).
- **Any PTT mutes everything while held**; release rejoins LIVE, no replay. Recovery is procedural
  and knows what was cut (`voice::SpeechMemory`, `router::recall_line`): silenced lines are OWED;
  "what were you saying?" says them, "say again" the newest of owed, readback and volunteered line.
- **The frequency's "say again" repeats the controller** (`tower::is_say_again`,
  `voice::controller_repeat`, `main::repeat_controller`): its last call to us, verbatim, on the same
  frequency (never ATIS, a ghost or `alert_station_overheard`). "What did the tower say?" reports
  it; with external ATC the crew says it cannot hear the frequency.
- **A station hears only its own field on the ground** (`station::note_aircraft`,
  `GROUND_REACH_NM`): beyond it it is silent (`tuned_service`) and retires what it issued;
  `party_line::station_reaches()` is false, so full auto departs as UNCONTROLLED with its own taxi
  (`party_line::plan_self_taxi`), published like a tower route (LILB 2026-09-25). Airborne the last
  controller keeps the flight. Via names are unspoken (untowered apt.dat names are an author's).
- The radio needs POWER both ways ("radios are dead"). **FAIL OPEN, the flag RESTORED every tick**:
  a fact that merely stops being published latches across a swap (recurring bug class).
  `take_crew_radio` drains unconditionally, so a failed gate loses the call.
- ONE frequency: speech renders as whoever COM1 is on (`tuned_speaker`); the rest is the void.
- REPLAY gates the reactive surface (monitor, station audio, transmissions, autoland arming); PAUSE
  freezes ticks and loops, not audio; `xplane::sim_frozen` covers both, failing open.
- ONNX session CREATION is serialised (`src/tts_onnx.rs` `build_session`): two first sessions at
  once segfault. Barge-in kills by REGISTERED pid + per-channel cut generation, never pkill.
- `Voice::alert` mirrors to the panel transcript itself: mirror OR `s.action(...)`, never both.

## Ground & flight automation

Orchestrators compose proven loops; no new flight laws in them. Laws test as plain `cargo test`;
everything blackboxes to `logs/blackbox.jsonl` (kinds: `atc`, `autotaxi`, `autotakeoff`,
`pushback`, `checklist`, `taxi_guide`, `cockpit_auto`, `full_auto`, `circuit`, `procedure`,
`cabin`, `instructor`, `loading`, `turbine_start`, `heli_start`, `hover`, `heli_circuit`,
`heli_route`, `rapport`, `cabin_air`, `api_load`, `plan_audit`, `handover`, `start_item`,
`sim_radio`, `fuel_pumps`, `electrical`, `typed`, `crew_radio`).

### Holding, writers, reads
- **A stage not moving the aircraft is holding it**: brakes for every wait, abort and give-up. **A
  mission that ENDS on the ground stops and parks** (`full_auto::hand_back_hold`, governor or not, a
  pilot's "my controls" included): wheel brakes to a stop below `HAND_BACK_BRAKE_MAX_KTS`, then
  `set_park_brake_confirmed`, read back and said; airborne, fast, a tug on the bar or an unread speed
  is `hand_back_skipped`. A give-up the abort flag ended is the pilot's cancel (`give_up_words`) (LFPB
  2026-09-22; LEMG 2026-09-28: 5.8 kt and rising after "your aircraft").
- **One writer per dataref.** Plugin channels (`taxi_steer`, `taxi_brake`, `taxi_brake_l/_r`,
  `pitch`) are 1 s expiring heartbeats written EVERY tick (a change-gate drops them mid-turn);
  debounced yield latches (`steer_yield`, `pitch_yield`) drop the override that frame; writer death
  frees by silence.
- **Park brake through the valve-aware seam** (`ensure_park_brake_valve` /
  `release_park_brake_for_roll`), never a raw `parking_brake_ratio` write: with the valve SET any
  application latches. Valve sense per airframe (`park_brake_valve_set`).
- Batch a tick's reads in one `tokio::join!` (sequential HTTP wrecks it); check `dt` before gains.
- **Signed vs unsigned**: groundspeed and route distance are magnitudes; directional values keep
  their sign (`pushback::back_along`, `taxi_guide::project_signed` for `lat_m`); gates use `.abs()`.
- **Readbacks and echoes never arm automations**: arming grammars take request shapes only ("taxi
  for me"); `looks_like_command` checks bigrams on raw text. "stop" routes by context (governor =
  retarget; steered push = abort; native push = stop + brakes; airborne = heal the stale flag).

### Taxi, push, line-up
- **Auto taxi** (`src/autotaxi.rs`): speed through `hold_speed`'s front door; pure pursuit with the
  aim capped in a bend so the cut stays small (`bend_lookahead_cap_m`); an off-route engage is a
  JOIN (refused past `MAX_JOIN_M`); errors can only slow the pace; a governor evaporating on the
  ground ARRESTS ("your aircraft"). Castoring nosewheels brake-steer through the PLUGIN channels
  only: REST `*_brake_ratio` writes are re-driven by hardware every frame.
- **The sim brakes against hard rudder** and our steer is rudder, so the plugin holds
  `override_toe_brakes` while `xcopilot/taxi_steer` applies; a pilot brake latches
  `xcopilot/brake_yield`. Never "fix" a corner that stops the aircraft by tuning the governor.
- **A jump in a projection is never a movement** (`taxi_guide::project_signed_facing`): a facing
  pick must be within `MAX_JOIN_M` or nearer than the plain answer, else a jump stops the drive
  (EBLG 2026-09-22).
- **Routes ride the paint** (`ride_paint`): 1201/1202 for topology, names, crossings; drawn `120`
  centrelines for geometry, sliding sideways only; runway legs and the vacate's start never move;
  thin data degrades, never refuses; a chord of a drawn curve rides the curve between two samples on
  it (`fill_paint_gaps`; LEMG 13 2026-09-26). Stands get a `plan_to_stand` lead-in. **An arrival's route is
  never a departure's clearance**: check the party line's `taxi_to_stand`, not proximity.
- **A hold is reached ON the line** (`autotaxi::arrived_on_line`, 5 m; else `stopped_off_line`, a
  taxi failure; full auto's hand-taxied check `at_the_hold`; LSZS 03 2026-09-24).
- **Line up** rides the DRAWN lead-on (`TaxiRoutePlan.lead_on`), synthetic geometry only as a
  fallback (invented entries end in the grass); crosschecked against `party_line::clearance_runway`;
  refused with too little runway left.
- **Pushback** (`src/pushback.rs`): the tug commands are WHOLE manoeuvres (a "Nose 90" is in the
  plan's arithmetic), sent ONCE at the summon, never steered; `pushback_stop` = stop and let go.
  - **We stop the tug when we are there** (`PushJudge`): distance made and the nose on the lane
    is `Aligned`, stopped mid-swing, led by the rate (`stop_on_the_lane`, `STOP_LATENCY_S`). The
    swing (`SwingWatch`) only blocks a FALSE stop: no stall verdict and no distance-only finish
    while the nose is swung, a rate dip is not the end. A swing is 2 degrees in 2 s OR in 4 s (a
    slow build is a swing); the pavement look-behind is a straight push's test, mid-swing only
    leaving the pavement stops it (EGBB 2026-09-27). A tug that swings on after the stop is
    logged (`stop_ignored`), never told again. A tug on the bar owns the aircraft: the taxi holds
    on `pushback_attached` (`tug_hold`, `TUG_WAIT_SECS`). The pilot's stop is answered from the
    park brake's read-back by the push loop (`abort_brakes`, `stop_line`).
  - **Any command to a tug ON THE BAR drops it off**: no re-issue, no nudge, no second pattern; a
    short push is judged (`push_delivered`), the rest is the taxi's. The bar coming off ENDS the
    push (released tugs never come back).
  - `HOOK_SOLID_POLLS` before brakes release, pattern issued first; creep HELD by the brake
    channel. `tow_in_progress` is dead, evidence only.
  - Direction from the cleared route's first leg; a tug only ON A STAND (`pushback::on_a_stand` via
    `party_line::nearest_stand`; a field without stand rows keeps the bearing rule), else every
    restart mid-taxiway calls a tug. "Skip the pushback" is `pushback::SKIP`; bare "stop" aborts.
- **Taxi guide** (`src/taxi_guide.rs`): early gentle cue, quiet inside the range where the
  compliance watch takes over (never two voices on one mistake); gates on `pushback::ACTIVE`;
  "which way?" is question-shaped so a readback never triggers it.

### Takeoff
- **Auto takeoff** (`src/autotakeoff.rs`): refuses without a clearance when a tower runs; ALT on the
  transponder at the roll; `RollSteer` (P, lead damping, slow clamped integrator, a cap that grows
  with speed; SCHEDULED on a declared `roll_rudder:`, lead on the sim's `R`, EGKK 2026-09-27); stagnation reject on GROUNDSPEED (the ASI is dead when slow); rotation on
  `xcopilot/pitch`; the AP takes the ESTABLISHED climb at 300 ft, the VS dial written AFTER engage
  (engage syncs it away); pitch damping on the sim's `Q`, never attitude differenced; trim into the
  takeoff band before brakes off. `brake_steer` airframes brake-steer the roll (`roll_brake_split`).
- **The centreline cut is sized for a CLOSURE TIME** (`autotakeoff::roll_target_heading`, one rule
  for roll and rollout; `XTK_GAIN_REF_KTS`, `XTK_LEAD_S`): a fixed deg/m gain tightens with speed
  until the loops weave. The rollout blackboxes `rollout` per tick with the REAL tick time. So a
  steady drift is held off only by a standing offset: the scheduled roll feeds its own drift
  forward (`DriftFilter`, the final's crab pattern; EGBB 2026-09-27).
- **The declared rudder is believed only up to the speed it was measured at** (`RollSteer` held flat
  above `roll_rudder:` `at_kts`, ceiling `RUDDER_FULL_DEG_FAST`; extrapolated as v^2 it cut the gain to a
  third by Vr: EGKK 08R lifted off 5.4 m left, 2026-09-28). The scheduled roll's cut is `RollCut`: the
  median of three cross-track reads (a mixed-frame pair read up to 2.9 m off for a tick until `xcopilot/position`), cut harder from `XTK_GAIN_REF_KTS` to `at_kts`, plus a slow cross-track trim.
- **The handoff line is true** about who has the aircraft and the height (`handoff_to_caller`). **A
  pilot handoff is FLYABLE**: it waits for the clean-up (`handoff_waits_for_clean_up`), sets a
  capturable dial (`handoff_altitude_dial_ft`), and gives the governor the overspeed guard's speed
  (`handoff_speed_kts`). Flap schedules fit their placards (test-enforced over every profile).
- **A jet's climb-out is declared** (`reference.acceleration_height_agl_ft`,
  `autotakeoff::ClimbOut`): below it the VS dial holds `initial_climb_kts` (`climb_out_vs_fpm` to
  `best_climb_fpm`) and the flaps stay; above it the schedule steps and the handoff waits for it
  (`handoff_waits_for_schedule`), each step only once the flaps ARRIVE at the last
  (`paced_flap_step`) and called by its detent ("Flaps one.", `flap_step_line`). A circuit keeps
  the old climb-out.
- **The initial climb holds V2 + 15 by pitch** (`autotakeoff::ClimbHold`, profile
  `climb_pitch_max_deg`): liftoff to the 300 ft engage, the attitude target walks on the airspeed
  trend, floored at the rotation attitude and capped by what the tail has cleared; silent profiles
  keep 8 degrees (LEMG 13 2026-09-28: 162 to 185 kt at 8 degrees). The climb attitude is a CEILING the nose never
  passes (`RotatePitch::ceiling_guard`, `CEILING_APPROACH_S`, target `CEILING_MARGIN_DEG` under it;
  `climb_manual` `pitch_lim`; EGKK 08R: 22.4 under 18, the loop's overshoot).
- **The autopilot takes the climb being flown** (`autotakeoff::ClimbVs`, `vh_ind_fpm`): the 300 ft
  engage writes the climb the pitch was flying and every later command, the circuit's included
  (`circuit::ClimbCarry`), moves at `CLIMB_VS_SLEW_FPM_S`; a circuit takes its flaps from the
  takeoff (`flaps_to_caller`) and walks them up on the paced schedule; the MCP window leaves V2 at the
  first flap step (`speed_window_after_acceleration`; LEMG 13 2026-09-28).
- **A jet configures its autoflight before the roll** (`takeoff_config:` `flight_directors`,
  `autothrottle_armed` (armed, never engaged: the takeoff owns the levers), `v2_kts`; probed) and
  reduces thrust at `thrust_reduction_agl_ft` to `climb_thrust_ratio` at a crew's pace
  (`thrust_reduced`, blackbox `thrust_reduction` with `n1`); the BEFORE TAKEOFF list audits it as a
  Monitor once the roll's transponder check is done (`before_takeoff_audit`). A330: FD and V2 only.
- **A runway start is a lined-up aircraft and still calls** (`tower::Ctx::own_on_runway`), never
  an incursion (`ComplianceState.runway_start`: a strip first seen stopped); its clearance comes
  first from the strip and names it (`full_auto::zone_clearance_from_the_strip`; asked again when the
  held one names another runway); the tower gives a caller standing on the stated strip that runway
  ahead of the calm-wind stream when the wind does not argue (`note_caller_on_strip`); too little ahead is a BACKTRACK, never a taxi (`full_auto::RunwayStart`,
  `backtrack_from_runway_start`, `Intent::RequestBacktrack`; EGKK 26L 2026-09-27); a clearance for
  another runway is taxied to (`under_clearance`); a start whose end opposes the live stream takes the
  stream's end (a 31 start facing 13 departures takes 13). `why_no_fit` diagnoses the aligned end we are
  ON (parallels tie on heading).

### Circuit, procedure, full auto
(`src/circuit.rs`, `src/procedure.rs`, `src/full_auto.rs`)
- The pattern is sized to the AP's real turn radius, side and height by terrain and airspace,
  `pattern_speed_kts` >= 1.3 Vs clean. `climb_limit_fpm` fades climb toward blue line / 1.15 Vs (a
  climb is never worth more than the airspeed left). Throttle laws watch a speed PROJECTION.
- Base descends the PATH; final uses along-track `path_dist_m`, never track-miles; a REAL intercept
  (`final_intercept_hdg`) on the turn actually flown (`final_turn_radius_m`, `AP_BANK_FINAL_DEG`);
  `ready_for_autoland` needs ESTABLISHED (`ESTABLISHED_XTK_M`), and on a runway of known width ON
  THE LINE (within landing limits, tracking within `ON_LINE_TRACK_DEG`, autoland's own residual at the
  threshold inside the landing limit). **The turn onto final is sized on the bank the autopilot
  flew** (`BankWatch`, `final_capture_on_bank`; the 737 banks 25, not the Baron's 15.5). The glidepath feeds forward
  `path_descent_fpm` (a P controller cannot track a ramp).
- **The sim's heading mode never reverses a turn in progress** (a bug outside a bank goes the long
  way): every steer goes through `circuit::bug_toward` (on the nose until wings level), leading from
  level by at most `TURN_ENTRY_LEAD_DEG`. Authored routes have a watchdog (`authored_stalled`).
- **The AP's own capture holds level legs where declared** (`ap_altitude_capture`,
  `circuit::circuit_dial_ft`, `ap_holds_level`); an AP that refuses VS is said once (`vs_refused`).
- LXGB's authored route joins at the nearest leg (`join_waypoint`) and is terrain-probed along its
  LEGS against the altitude the climb can have reached (`climb_reach_ft`): a pattern leg raises the
  pattern (`raise_for_terrain`, said), a descending leg refuses. Downwind fixed
  (`LXGB_ROCK_CLEAR_M`).
- Full auto: engine start per class (`StartKind`); power-up INLINE (racing bring-ups shut a
  cross-tie); waits on `driving()`, not `ACTIVE`; refuses to arm airborne, naming the tools that
  fly; ends with `taxi_in_after_landing`. **A resume past the route near the destination is the
  arrival** (`resume_join`, `ARRIVAL_REJOIN_NM`, autoland on the plan's runway); every refusal names
  its number. Cruise is a SPEED: the OFP's on its cruise legs
  (`procedure::scheduled_cruise_kts`: the lower of planned IAS and planned Mach at the level,
  `cas_for_mach`; `cruise_pick` only inside floor and ceiling), else `reference.cruise_ias_kts`. **A
  planned step is flown at its fix** (`apply_step_climbs` at the arm, all or nothing; `step_climb_at`).
- **The arrival is briefed at the arm** (`full_auto::brief_arrival`); a destination the data lacks
  is refused there, not on the approach.
- **The next fix is `procedure::active_leg()` first**: the GPS sits on the plan's first entry.
- **Full auto flies the checklist items it owns; lists only cross-check.** A continuous-mixture
  piston (`piston_mixture`, the profile's `engine` group) arms `engine::arm_mixture_manager` enroute
  (rich below `LEAN_FLOOR_ALT_FT`, `manager_seek_window`, `needs_retrim`) and
  `mixture_rich_for_landing` on arrival; it stands down on every give-up.
- **Exterior lights follow the practice** (`src/tools/lights.rs` `Exterior`; `dark_now` =
  `cockpit_auto::dusk_or_dark`): taxi light for the taxi (the landing light after dusk where there is
  none); landing light over each runway span (`Route.runway_spans`); landing AND taxi off passing
  1,000 ft AGL (`lights_climb`, `after_climb`); landing off, taxi on at the vacate
  (`after_vacate`); all off at the stand.
- **The crew picks its own procedures**: a cleared runway the SID does not serve gets that runway's
  SID joining furthest along (`nav::sids_for_route`, `full_auto::choose_departure`); a METAR
  favouring another end gets its STAR entering earliest (`nav::stars_for_route`,
  `plan_audit::choose_arrival`, `star_chosen`). Either reloads the GPS
  (`full_auto::payload_with_legs`). A cleared SID that serves the runway is flown first
  (`prefer_cleared_sid`, `departure_rechosen` source cleared|chosen); a departure other than the cleared SID is ASKED for on the radio and flown only once the
  clearance names it (`request_sid`), else handed back on the ramp, never flown silently, and the climb is capped at the
  cleared altitude until the departure check-in (`procedure::hold_at_cleared`).
- **Charts are flown**: SID minimums (`nav::sid_minimums` from `sid_ident` / `plan_rwy`, as
  `climb_minimums`) are judged at the arm (`departure_gradients`), climbed at best rate and watched
  (`watch_minimum`, a miss handed over); a minimum is a FLOOR held until its fix and met once
  (`DepartureMinimums`; EGKK D156J hunted 5,000/6,000 and 210/250 at 5,900, #153); the owners are
  plan, minimum, then the clearance's cap (`departure_target_ft`): a minimum above the clearance waits
  for the check-in, said (`minimum_capped`, `cleared_climb`); the VS wheel is written only in VS mode
  (`vs_action`: an ALT HOLD closing on our dial is left). **The 250 below 10,000 cap, once begun,
  holds** (`speedhold::AirspaceLimit`: lifted only at 10,300, both edges legal; `airspace_capped`). STAR windows (`nav::star_constraints`, `star_ident`,
  `destination/plan_rwy`) clamp the plan (`apply_arrival_constraints`); each target is capped so a
  later "at or below" stays reachable (`arrival_ceiling_ft`, `star_ceiling`; plan altitudes too,
  `profile_ceiling_ft`); speed limits are slowed for in time (`arrival_speed_cap`, `star_speed`).
- **The whole plan is dry-run first** (`src/plan_audit.rs`, at the arm and on "check the flight
  plan"): the follower's own rules over the route. REFUSED: another type, cruise over the ceiling,
  fuel over the tanks, range past the burn + 45 min, a short runway, a climb into PROBED ground.
  CAUTIONS: a leg under grid MORA (`earth_mora.dat`, row = the cell's south edge), STAR crossings,
  trip burn, overweight landing, crosswind, unpressurised height. The all-clear names what could
  not be checked.
- **The crew works the radio as it flies**: a conditional line-up waits on
  `party_line::line_up_condition_clear()`; `take_departure_handoff()` (readback, retune, check-in);
  an IFR readback is judged from when it goes OUT (`ReadbackWait`, `circuit::crew_radio_pending`);
  `take_region_handoff()` (survives `Station::install`; 7000 via `SetTransponder`;
  `region_coach_line` with a pilot flying). **An IFR flight keeps its code** (party line
  `IFR_FLIGHT`, survives install, retired at the landing): no service terminated, no 7000
  (`region_handoff_line`, `RegionHandoff.ifr_code`; LIEA-LEBL 2026-09-27). Airborne it is handed
  to a field's radar position, never a tower-only one (`radar_position`; the pinned arrival keeps
  its tower; FL150 to "Oxford Kidlington Tower", EGBB-EGKK 2026-09-27).
- **The inbound call** goes from the enroute leg once handed to the destination's OWN tower and
  close (`inbound_due`, `INBOUND_CALL_NM`, `INBOUND_MADE`) or on its challenge
  (`party_line::take_zone_challenge`); else at the station swap (`home_tower_khz`), before the
  final at the latest. `call_inbound` tunes COM1 first, and the retune ARMS that station
  (`station::arm_station_for_freq`: you call it, you get it). A crossing field's challenge is
  `zone_challenge`, never answered "inbound".
- **Top of descent**: a descent waits for it (`descent_target_ft`) and only
  `procedure::planned_tod` is announced. **While a follower flies, the plan's ring is silence**
  (`Snapshot::follower_flying` gates `tod_ahead`, `tod_passed`, `high_on_profile`). A PILOT-held
  level gets one reminder naming who holds it (`PlannedTod.held_ft`, `procedure::held_tod`). None on
  an approach or in a climb (`CLIMBING_FPM`), and the family speaks only in the model's cruise
  above 1,000 ft AGL (`monitor::tod_in_cruise`; a ring behind the departure, LSZS-LILO 2026-09-24).
  The plan's ring is along-route and compared with the route still to fly
  (`monitor::route_remaining_nm`, `ring_dist_nm`); a mile is "a mile" and the time comes from the
  distance (`tod_ahead_words`).
- **A level the plan handed down is not argued with**: no step-climb advice while a follower flies,
  with under 30 min of cruise left, or more than three levels under optimum (a level PLANNED low,
  `STEP_CLIMB_MIN_CRUISE_MIN`; EGBB-EGKK 2026-09-27); a wandering cruise is still cruise
  (`CRUISE_WANDER_FPM`); a JOIN projects along the nose (`project_signed_facing`); the taxi law's
  idle-and-brake waits unless a surge (`SURGE_KTS_S`).
- **A descent that outruns the governor gets drag, then a shallower rate**
  (`procedure::descent_drag`: speedbrake where `arm_spoilers` is declared; stowed on every exit).
  Never with the gear down or flap past the first detent (`drag_barred`, `why`; EGKK 2026-09-27).
  A brake out stays out through a level-off until the speed is in, holds each state
  `DRAG_MIN_HOLD_S`, and is said at most every two minutes (`DragVoice`); a STAR speed limit once
  begun holds (`arrival_speed_cap` `begun`); a capture of the dial is not a dropped VS mode
  (`vs_mode_capturing`; EGKK-EGBB 2026-09-27).

### The follower's vertical and the altimeter
- **Three owners in order** (`procedure::vertical_tick`, pure): the PLAN's constraint, paced to its
  top of descent from the last COMMANDED altitude (`prev_cmd_alt`), never the constraint it is
  about to become (a descent began 56 nm early, 2026-09-13); then the PILOT's altitude, HANDED over
  (`procedure::take_altitude` from `fly_to`, `level_off`), sticky, never written to the dial (a
  follower rewriting the dial undoes the pilot in 500 ms); then the GROUND AHEAD, over both.
  **A descent never climbs back** to a navlog altitude above the level commanded
  (`DescentProfile`, `descent_no_climb`; LIEA-LEBL 2026-09-27); step climbs before the top do.
  Mirror: a climb never descends below it until the cruise is FLOWN (`climb_no_descent`; LFBD).
- **Terrain floor** (`terrain_floor_ahead_ft`: probes ahead + 1,000 ft), skipped on an approach,
  retired only after unanswered probes with the sim LIVE (`terrain_probe_fatal`). An AP target is
  judged against it: a climb into it RAISED and said, a descent below it refused with the number.
- **The follower flies the ALTIMETER** (`altitude_ft_pilot`): dial, plan, chart and controller mean
  indicated feet; only the floor is true feet (`floor_indicated`).
- **So the altimeter is kept right**: ONE rule (`instruments::correct_setting_inhg`: QNH below the
  transition, standard above) behind the tool, full auto, the follower's arm
  (`Plan.transition_altitude_ft`), both sides (the first officer's too), a declared `altimeter:
  std_command` going to 29.92 first when it saves ten presses (`std_fast_path_pays`), and the monitor's `AltimeterSetting` check: an automation flying
  RESETS it (`Check::fix_write`), a pilot is advised once. `set_altimeter` steps in rounds EARNED by
  progress (`keep_stepping`); `AltimeterSync::line` says "did not take" (EGSS 2026-09-22). The
  transition is the FLIGHT's (`instruments::FlightTransition`, `origin/trans_alt` /
  `destination/trans_level`).

### Approach and autoland
- **Every planned arrival hands its runway down** (`EndAction::Approach { rwy_id }` to
  `run_autoland`); the self-pick (`autoland::self_pick`, `landing_score`, `self_pick_min_length_m`,
  METAR tailwind, `DESTINATION_BONUS_NM`) is the emergency path.
- **A new flight takes its clearances with it** (`party_line::retire_for_new_flight`, called by
  `setup_approach` before the arm; the old station publishes nothing until the new one is installed,
  `NEW_FLIGHT_PENDING`, and its radar watch; whoever starts it: a mailbox run the app did not post
  (the runner's starts) stands every loop down and retires them too (`setup::spawn_new_flight_watch`,
  `judge_init_seq`, blackbox `setup` `foreign_new_flight`); **its station is the field under the NEW
  position** (`station::arm_new_flight_at`, `field_under`) chosen from a SETTLED position
  (`setup::SettleJudge`: the load reports a runway's midpoint on the ground 157 ft up), installed
  before the flag drops; full auto's mission waits for it (`judge_departure_station`, 20 s, else
  `give_up` `departure_station`; EGKK 2026-09-28); nothing arms while the app's own new
  flight loads (`setup::new_flight_loading`: a load reports the first runway's midpoint); a refused stated runway is blackboxed `stated_runway_refused`; LEMG 2026-09-28).
- **The tower's clearance outranks the plan** (`runway_from_clearance`), watched THROUGH the
  approach since its answer comes after the arm: owed readbacks (`party_line::readback_owed`), a
  re-arm on another end (`clearance_switch`, never `going_around`) moves the ONE arrival runway
  (`full_auto::retarget_arrival_runway`, also the #79 re-pick's write point); a superseded loop leaves
  `ACTIVE` to its successor. Arrival wind is the destination METAR (`weather::surface_wind`,
  `arrival_wind`), not the datarefs aloft.
- **A straight-in to THIS runway or none** (`nav::straight_in_legs`: the exact designator, no RF
  arc, legs cut at the threshold; circling is not an approach); another end before a visual; the
  tower's inbound runway when flyable (`tower_arrival_runway`).
- **A straight-in is only as good as its fixes** (`RECON_ALIGN_DEG`; `approach_plan_for` refuses a
  FAF autoland cannot intercept, `faf_off_course`, `interceptable`). Clean speed
  (`power_hold_target`); gear and flaps wait for their limits (`gear_may_extend`,
  `flap_within_limits`); no taxi-in after `has_crashed`. **A final is judged against the
  pavement it lands on** (`runway_width_m`, `guidance_on`, `landing_xtk_limit_m`,
  `flare_xtk_limit_m`): below the floor it must be interceptable AND its pursuit must leave the
  offset inside the landing limit at the threshold (`lands_on_the_pavement`), or it balks with height
  in hand, ordered even on the first tick (LEMG 13 45 m, 36.7 m right at the wheels, 2026-09-28).
- **The autopilot's own modes are tools, not yet the follower's** (`tools::autoflight` `ap_mode`:
  autothrust, level change, GPSS; `systems::ap_modes`: proved > `autoflight:` > type): toggles read
  before firing, refused where the map says No; the follower adopts `engage` later (probes owed).
- **The arrival is re-picked in flight** (`RepickWatch`): the destination tower's runway at once, the
  METAR's wind runway on two readings in a row beyond 25 nm (never in calm, never against the tower),
  at most `MAX_ARRIVAL_REPICKS`; the new STAR is spliced from its entry (`plan_audit::star_splice`,
  `procedure::offer_splice`), windows clamped, GPS reloaded (`arrival_repick`).
- **An RF leg is flown round its arc** (`course_leg::arc_track`, `ProcLeg::arc()`, ends at the end
  radial) and **a hold as a racetrack** (`HoldFlight`: direct entry, each 180 split into two 90s,
  chart time else 1 / 1.5 min; HF one circuit, HA to its altitude or 6 circuits; HM flown through).
- **A CF leg is flown on its course** (`course_leg::course_track`: a 30 degree intercept rolled out
  at the fly-by lead), never homed on; **an IF behind us is joined by autoland's pattern** against
  the FAF's course until the intercept fits 2 nm out (`FinalJoin`; EGKK 26L 2026-09-27).
- **The autopilot is engaged and CONFIRMED, never assumed** (`ApEngage`): pressed every tick until
  `servos_on` reads on for two live ticks, each try blackboxed `ap_engage` then `ap_engaged`; until
  then the loop holds level on `xcopilot/pitch` (`engage_hold_gains`, `ap_hold`), and 10 s of live
  flight without it is said and handed over (`ap_not_engaged`). A new flight's start drops a press
  made before it (LEMG 13 2026-09-28, 405 ft AGL).
- **The VS watchdog excuses a slow autopilot, never a dead one** (`vs_not_held`).
- **A mountain field is a flyable approach or a hand-over** (`mountain_field`, `earth_msa.dat`): a
  MAP short of the runway is flown on its path (`map_path`) and HANDED OVER with the talk-down armed
  (`map_handover_path`, `nav::visual_segment`). The steep autoland (`gp_ft_per_nm`,
  `take_missed_legs`) is off behind `MAP_AUTOLAND` (LSZS 2026-09-24). **A MAP on the runway fix is a
  landing** (`map_short_of_runway`), handed over at the FAF (`handoff_index`; EGKK 26L 2026-09-27).
- **The ILS is tuned, watched and, per airframe, flown** (`src/ils.rs`): `ils::tune` at every arm
  (read back, `ils` `tuned`); `XCheck` calls a needle held off on final; `Coupling` only with the
  profile's `ils_coupled`, never writing what a capture owns.
- **The copilot knows the systems before reaching for one** (`src/systems.rs`: the `.acf` and
  `preconfigured_ap_type`, blackbox `systems`, reset per swap): a fact, never a licence.
- **"Set up the ILS" is a pilot monitoring's setup** (`src/tools/approach.rs`, `SetupWatch`): APPR
  armed only from OFF (a toggle), with the servos in, inside coverage (`arm_verdict`), read back,
  never retried. A pilot hand flying keeps the aircraft. A localiser bearing is
  `magnetic x 360 + true` (`nav::loc_courses`).
- **A reposition is a PATTERN, never a point** (`src/reversal.rs`, `reversal::needed`): the downwind
  extends while the height cannot be lost (`reversal::too_high_to_turn`, `pattern_descent_fpm`); an
  inbound past the threshold is OVER (`reversal::overflown`); `pattern_ias`. **The stabilised gate
  is about TIME** (`unstabilised_at_gate`, `reversal::predicted_residual_m`), not position.
- The receding guard measures from the furthest legitimate point (`recede_ref_nm`); flaps never cycle
  on a placard (`flap_held_within_limits`); **an approach never takes away the flap the wing flies
  on** (`flap_kept_flying`, `last_flap` read at the arm) and **a climb is flown on thrust**
  (`climb_power_floor`); a pattern's inbound leg the glidepath reaches is the approach
  (`reversal::path_joined`); a give-up hands the SPEED to the governor
  (`hand_back_speed_kts`).
- **Autoland core**: the pursuit aim keeps full lead (`pursuit_aim_nm`); a roundout band
  (`ROUNDOUT_MULT`, `ROUNDOUT_VS_FPM`) before the flare (level-hold over the runway is the bug); a
  rolling handoff at `VACATE_HANDOFF_KTS` (`autoland::VACATE_FOLLOWS`); a sticky `MissedStage`;
  give-ups hand back a FLYABLE aircraft. The dial is parked at the arm (`parked_altitude_dial_ft`),
  else its capture ignores the VS dial; the glidepath aims at the pavement (`aim_elevation_ft`,
  `threshold_elev`), not the apt.dat header.
- **The crab is LOW-PASSED** (`CrabFilter`, `CRAB_TAU_S`; `crab`, `crab_raw`) and cancels wind only;
  a bias is `TrackTrim`'s, **learnt only from a settled track** (the trim inside 3 degrees,
  the short final wings level and bounded: `TRACK_TRIM_ERR_CAP_DEG`, `SHORT_FINAL_LEARN_*`). Each balk names its gate (`Guidance.balk`; ticks carry `trim`, `xtk_m`).
  **A tower's go-around order outranks everything, and reaches whoever flies the final**: each loop
  watches it (`party_line::GoAroundOrders`, never the main loop's `take_pending_go_around`); autoland
  latches it (`GoAroundLatch::order`), the circuit flies it from Base or Final
  (`flies_go_around_order`, clean-up on `go_around_flap_step` / `go_around_speed_kts`; LEMG 31
  2026-09-28).
- **The landing arrest leads the AP's lag** (`vs_dial_fpm`, `Guidance.arresting`): gain, cap and
  horizon are the airframe's (`ArrestLeadProfile`); gates and targets are seconds of sink
  (`arrest_gates_ft`, `arrest_targets_fpm`). Ticks log `cmd_vs`, `vs_dial`, `vvi`, `vvi_trend`.
- **The arrest takes the servos out at the roundout gate** (`arrest_hands_step`, `arrest_hands`),
  since an engaged servo writes the yoke back over `xcopilot/pitch`: `servos_off_any`, the track on
  `xcopilot/roll` (`WingsLevel`). **The AP comes out at the wheels** (`disconnect_autopilot`): a
  heading servo on the ground winds the yoke over.
- **A model flare holds its column in rough air** (`FlareModel::step` `rough`, `FinalAir`,
  `ROUGH_AIR_FPM`): past 40 fpm rms of stray on the final, only a measured climb eases the pull; in
  smooth air the prediction flies it and a near-level float never takes it forward. A gust and a
  strong elevator look alike for the seconds that matter (LEMG 13 2026-09-28: 422 fpm in gusts; a
  still-air hold through the 09-26 17:25 plant ballooned to 506).
- **The flare retard is keyed on height, time to the wheels and speed, never on the sink the
  arrest is removing** (`FlarePower`, `retard_below_ft`, `retard_at_s`, `retard_fast_kts`,
  `retard_slow_kts`, `retard_max_per_s`; `power` `retard`; LEMG 13 2026-09-28), and **a FLOAT takes the
  thrust off** (a sink under `RETARD_FLOAT_FPM` now or `RETARD_FLOAT_LEAD_S` ahead retards at any height
  and speed; slow only delays a retard not yet begun); a climb is a balloon only past
  `BALLOON_CLIMB_FPM` (EGBB 15 2026-09-28).
- **A slow, low go-around pitches for speed** (`go_around_vs_fpm`: level over the configuration's
  floor, below it a descent no steeper than the ground 10 s away; `go_around_flap_due`): 0 fpm held
  into a stall (EDLP 24 SF50, 2026-09-24).
- **The heading knob is set, not rewritten** (`circuit::BugHold`, `autoland::bug_knob`,
  `heading_set`): once per leg or phase, steering of 5 degrees or more at once, corrections past a
  deadband; the short final and the flare still steer every tick.
- **In the flare the column only comes BACK** (`FlarePitch::step` `ratchet`, `FLARE_PITCH_MAX`):
  forward column a few feet up is a nosewheel landing. A BALLOON eases toward neutral, never a push
  (`hold_flare`; above the band `balloon`). Gains per airframe (`flare_elevator:`,
  `autoland::FlareGains`).
- **A low flare with room to stop is COMMITTED** (`autoland::commit_flare`, `committed_flare`): no
  `overflew_the_runway` go-around while the pavement covers the landing distance (a go-around from
  an idle flare cannot climb); a balloon under `BALLOON_COMMIT_FT` likewise (`balloon_power`,
  `BALLOON_LEAD_S`). The need is the book LESS the air already flown from 50 ft
  (`flare_pavement_need_m`; LFBD 11 missed by 12 m). **A low balk is called on the radio only once it
  CLIMBS** (`go_around_call_due`, `BalkWatch`); one the wheels take keeps the clearance
  (`balk_landed`).
- A debrief touchdown is the steepest recent sink (`instructor::touchdown_sink_fpm`).

### Taxi in
(`src/taxi_in.rs`) VACATE first (completing the landing clearance, decided at the JUNCTION), then
"runway vacated", THEN the request. **A vacate turns toward the parking and never onto another
runway** (`nearest_stand`). `vacate_runway` falls back to `autoland::landed_runway`;
`vacate_budget_secs`; stands entered on their line (`nav::stand_lead_in`). **A stand route asked
from the runway starts with the vacate, forward** (`station::stand_route_on`, `nav::after_vacate`;
EDLP 24 2026-09-24). **A jump in position is a relocation, never a movement**
(`autotaxi::relocation_jump_m`, `RELOCATED`; `vacate_still_here`): a drive, vacate or request that
finds the aircraft moved under it stands down; "runway vacated" needs the pavement to agree
(`vacated_report_due`). **A mission ends shut
down** (`taxi_in::shut_down_at_stand`: engines confirmed from `ENGN_running`, electrics off); a
spoken "taxi to parking" leaves the engines to the pilot.

### Still owed
Issues labelled `owed` + `area:approach-landing` / `area:takeoff` / `area:full-auto` (`ils_coupled` #81).

## Helicopters: the hover loop

Rotorcraft withhold every fixed-wing law (`tools::ROTORCRAFT_WITHHELD`) and fly their own loop
(`src/hover.rs`, blackbox `hover`), measured on the S-76C. Reuse aeroplane paths; branch only the
physical drive.

- **Cyclic to the autopilot, collective to us.** The AP never moves the collective;
  `servos_fdir_off` disconnects it, `servos_off` does not. The collective is `prop_ratio[0]`,
  written INDEXED.
- **The hover mode is a groundspeed HOLD that captures** (`sim/autopilot/hover`, fired only when
  `hover_status` reads off): engaging writes the moment's velocity to `hover_vector`, so the loop
  owns the vector (command + `DriftTrim` on `local_vx/vz`). `groundspeed` carries vertical speed.
- **A moving target is air taxi, departure, circuit** (`RAMP_ACCEL_MPS2`, `slew_track`; goals
  `Hover`, `Land`, `Depart` via `DEPART_CLIMB_MPS`, `Fly`). The mode lets go near 47 kt, hence
  `DEPART_MAX_MPS`.
- **Pedals are a yaw-RATE command with a heading hold**: above `PEDALS_ON_MPS` `HeadingHold` asks a
  RATE via `taxi_steer`, never damping on `R` (it rang until `PEDAL_FAULT_DPS` stood it down).
- **The collective follows the power curve at the hover's LOADING** (`level_collective_at`,
  `PowerBudget`, `LoadingEstimate`: W/sigma corrected by the lever the rotor lifts at, `liftoff`;
  `prelift`; a hover the lever cannot give is refused on the wheels or held in the cushion, with its
  numbers; a lift that droops on the wheels is given up; EDVW 2026-09-23, #84). The collective follows the power curve (`level_collective`); a takeover starts at the curve.
  The trim is bounded to the curve: an unbounded integrator crashed one (ETHC 2026-09-23). The rotor
  guard caps it (`droop_ceiling`); a long droop low down is a set-down.
- **In the air the AP is never simply turned off** (`hand_back`: cruise modes when fast, the hover
  mode at zero when slow, the lever at `steady()`).
- The start reads every throttle back at FLY and locks the tail wheel first (read, then TOGGLE).
- Airborne requests take over below `TAKEOVER_MAX_MPS` and `DEPART_MAX_FT`.
- **The final is capped by what the hover mode can hold** over the ground and through the air (a
  slow-down in a tailwind aims lower by the tailwind; ETND 17 kt headwind, #82); **a transit field's
  challenge is answered with a transit request**, the destination's with the inbound call, each
  station at most twice (`heli_route` `zone_challenge`; tower "request zone transit", #83).
- **Circuit** (`src/heli_circuit.rs`, `heli_circuit`): `Fly` vectors, a final tapering at
  `FINAL_DECEL_MPS2` led by the height loop's lag, then `Land`.
- **Route** (`src/heli_route.rs`, `heli_route`): the whole flight, as full auto: cold start, lift,
  departure, hand-over to heading and altitude hold (`hover::release`, the AP never off), `SpeedLaw`
  on the collective, bug stepped only past 2 degrees (rewritten each tick it jittered the yaw),
  final to the helipad, the aeroplanes' taxi-in. Cruise over the highest ground on a corridor
  (`sample_corridor`).
- **It works the radio with the aeroplanes' code** (`full_auto::get_zone_clearance`,
  `obey_departure_handoff`, `obey_region_handoff`; pinned by
  `every_mission_that_leaves_a_field_works_the_radio`), inbound via `arm_station_for_freq`.
- **The approach comes down at speed** (`Phase::Approach`, `approach_target_ft`, `approach_vs_fpm`)
  to a gate over the pad clear of the ground ahead (`gate_agl_ft`, `gate_lowered`); the dial is set
  only with the mode's engage; the hover loop takes over at the final's path (`approach_slow_m`,
  `final_command`).
- **The climb is checked at the arm** (`heli_route::climb_conflict`): a climb into a ridge crashed
  one (LILO-LILB 2026-09-24). `fast_climb` only where the ground allows.
- **Below 50 kt collective is not a speed control**: approach and slow-down floor it
  (`APPROACH_COLLECTIVE_MIN`); the hover loop takes over once slowed, whatever the height.
- No helipad: the runway end nearest the arrival (`runway_spot`). A hover taxi off a runway is said
  and waits for "my controls". Parking brake only when stopped (`touchdown_brake_due`).
- **Parking is the aeroplanes' tool** (`taxi_to_parking`); only the drive differs
  (`heli_route::park`, `corner_factor`). Low, a helicopter is on the ground for the tower
  (`hover::tower_on_ground`).
- Words: "hover at N", "set it down", "hold position", "take off", "fly a circuit"; the AP's mode
  tools are refused while the loop flies (`hover::blocks`).

Still owed: issues labelled `area:helicopter` (`owed`: none;
`unflown`: approach and gate #47, fast climb #48).

## The flight model: one state, read by everyone (in SHADOW)

`src/flight.rs`: the ONE answer to "where are we and what do we hold". Pure, fed per monitor tick,
time from the sample so recorded flights replay (`flight::Replay`). Tracks `Phase` (sustained, in
the `legal` table), `Control`, `Abnormal`, `Memory`, `Situation` (the pilot's takeover via
`flight::note_pilot_takeover`).

- **A ground strike is not a landing**: a climb into it or a sink past `IMPACT_SINK_FPM` is an
  `Impact`; a wreck has no phases until at rest.
- It is READ: checks use `monitor::coarse_phase` of it (`monitor::estimate_phase` is gone); the
  checklist tracker and cabin (`FlightFacts`) take it; "how are we doing?" is `flight::status_line`.
  `fphase`, `crashed` ride `state`; events are blackboxed `flight`.
- **The duty engine** (`src/duty.rs`, pure) says what the flight owes next; SHADOW only (blackbox
  `duty`) until its logs agree with flights flown.
- **The hand-over reads the model** (`src/handover.rs`): "you have control" (`grammar::TAKE_OVER`)
  resolves from `flight::published()` into the calls the phase needs (`level_off` as `if_idle`) or a
  spoken decline. The upset RECOVERY is only for an actual upset or distress words. "I'll taxi" /
  "let me fly" is the release guard (`note_crew_takeover`; blackbox `handover`).

Rapport's touchdown, bounce and time aloft are the model's events (`Rapport::tick` takes `&Flight` and
the tick's events; `Memory::aloft_t`); an `Impact` is never praised. Go-arounds are still rapport's
own count (the two disagreed on 2 of 4 recorded).

## Checklist flows: the list at its phase, three flavours

`src/flow.rs` runs the airframe's XChecklist sections (`src/checklist.rs`); replies via
`flow::take_reply` (CLOSED vocabulary). Every `Item::Act` carries its pass test, so a runner reads
FIRST: in state is confirmed, never re-written; an item a loop owns (`held_by_automation`) is named
as held, never written (one writer per dataref).

- **Crew** (pilot flying, copilot acting): the spoken "descent checklist".
- **Monitor** (an automation flying): read-only and silent, discrepancies blackboxed (`checklist`
  `discrepancy`), ONE line at the end (`flow::summary_line`). The laws act; the list audits.
- **Instruct** ("talk me through", `instruct_request`): never writes; says, watches ("Good."), moves
  on at "next" or `INSTRUCT_WAIT_SECS`.

**Who starts one**: `flow::Tracker`, one-shot per flight, every boundary SUSTAINED (a single-tick
boundary audits the automation's own actions half done; `CLIMB_AGL_FT`, `CLIMB_SECS`). Under an
automation Monitor starts at once; with a pilot flying the copilot PROPOSES (`PROPOSAL_TTL`) and
never acts by itself; the after-landing list runs at once as Crew. The profile's `checklist_flow:`
maps titles. Panel: `flow::status()`.

**A list belongs to the flight it began in** (`flow::stand_down`): an automation taking the aircraft
after a Crew list began stands it down (`CrewWatch` over `mission_owner`, latched, also called at full
auto's start); a new flight stands down any list (`InitWatch` on `init_seq`, the ramp and approach
setups, the model's `Reposition`); a Crew list asked for under a mission names what it does not find
as held, never writes (blackbox `checklist` `stood_down`; LEMG 2026-09-28).

## App modes & cabin crew

`mode.rs`: **both** (default) / **copilot** (no simulated ATC; radio-keyed speech ignored) / **atc**
(the world only) / **instructor** (hands off; every tool refused). `XCOPILOT_MODE` sets the BOOT
value only. The no-crew gate is the arbiter's enqueue door (`Voice::alert`); the atc crate never
reads the mode. **`mode::crew_actuates()` is the hands gate** (false in atc AND instructor): tool
dispatch (one door), `fix_action`s, `cockpit_auto`, `flow::start`, undo. Entering instructor stands
every loop down.

**Instructor** (`src/instructor.rs`): one ACS task at a time (`Task::Taxi`, `Task::Takeoff` too).
- The brief ASSIGNS the numbers from the book and the aircraft, so the scorer never guesses;
  "begin" re-assigns from NOW. An excursion is narrated once, with its correction.
- ACS tolerances per level (`Task::standards`); the taxi's centreline (`line_m`, `Axis::Centreline`)
  is judged by the scorecard's own `LineJudge` (vertex cuts are cuts) from the guide's turn card
  (`TaxiLine`), and `state` carries `taxi` for `grade_flight`. An AP-shaped command in a lesson is a RETARGET
  (`take_target`), never a write. Taxi pace is judged against `taxi.kts`, never too slow.
- **"Teach me a full flight"** (`Syllabus`, `STEPS`, `flow::phase_boundary`, `syllabus_active`),
  behind `SyllabusIo` (`FlowIo` live), so the whole flight is a plain `cargo test`.
- `xvl debrief` prints lessons and `instructor::grade_flight` over any flight's `state` records.
- **"Show me" is the one exception to folded arms** (`instructor::show_me`, `Demo`): the task's crew
  automation (taxi, takeoff, the pattern from the ground, a landing) is armed with the panel button's
  words through the tool door, opened ONCE plus its off switch (`mode::tool_may_actuate`,
  `DEMONSTRATION_ARM_S`); the instructor narrates each point once, grades it as its OWN (`demo_end`
  `by: instructor`), and hands back "Your controls." to the task briefed. Air work is hands.
- Still owed: #87 handbook questions (see docs/tasks/2026-09-28-instructor-handbook-questions.md:
  authored answers, then lexical retrieval, no model).

**Cabin crew** (`src/cabin.rs`): the second intercom persona.
- **A state machine, not a commentator**, with its own mute (`cabin_quiet`), NOT the monitor's
  `quiet` (which includes `autoland::ACTIVE`, so it missed its own landing, EBLG 2026-09-22).
- Addressed by "cabin crew" (bare "cabin" only with a crew word); "say again" over guessing; per
  airframe (`cabin_crew: true`); "cabin secure" is exempt from sterile cockpit
  (`STERILE_FLOOR_AGL_FT`).
- **Works the flight unasked** (`Cabin::watch`, before `tick`), each call once per flight and never
  after the flight deck made it.
- A bare "cabin crew?" opens the line (`take_open_line`). Its voice is the second picker
  (`xcopilot/opt/cabin_voice`; the copilot's refused, `pollers::cabin_pick`). Crewless: said so.

## The copilot as a person: timing, persona, rapport

- **Timing** (`src/tempo.rs`): a `Turn` per utterance, a jittered pause from the end of CAPTURE,
  "stand by" when nothing is queued (never over a radio call, `set_radio_turn`); `busy_now` sets
  `line_tempo`.
- **Persona** (`src/persona.rs`): `XCOPILOT_CREW_CHATTER` gates only what is VOLUNTEERED, never an
  answer or safety call; the idiom follows the VOICE's accent (`XCOPILOT_CREW_DIALECT`); small talk
  is a CLOSED set; `XCOPILOT_PILOT_NAME` on the first praise. Acks vary (`readback::vary_ack`).
- **Rapport** (`src/rapport.rs`): its wheels are the flight model's; sees every tick, autoland included; the landing word is the
  instructor's scorer (`landing_summary` still blackboxed); praise goes to the pilot, never an
  automation; a bounce counts at the SECOND contact. Offers are sentences, never actions
  (`rapport::arrival_brief`); busy deferrables become ASIDES.

## Panel, terminal, settings

- **The panel is the act-and-glance instrument**, fed by `xcopilot/status/auto`
  (`src/auto_status.rs`), change-gated with a periodic resend (plugin reloads zero buffers). Lines
  carry live detail (`auto_status::Detail`, `DetailGuard`, `wait_status`); panel-only
  `set_ap_targets`, `note_said`.
- **The taxi turn card** (`taxi_guide::TurnCard`, `xcopilot/status/taxi`): the guide's next decision
  point, aim bearing and cross-track, drawn as a GPS-style arrow by the plugin (rotated against live
  heading per frame); kind X off route points nowhere.
- Buttons inject pilot English (`REQUEST_PHRASES`, grammar-test-pinned); CANCEL ALWAYS aborts
  something. The typed line is `xcopilot/typed`; every pilot line is stamped on the channel
  (`pollers::queue_command`) and the loop's readback wait gives way to one (`Voice::say_until`;
  blackbox `typed`, `age_s` from `xcopilot/typed_age`, `queued_ms`). Route tab: `xcopilot/status/loadsheet`,
  `fms::active_fix`. Native panel-graphics API (SDK 4.4.0; `plugin/src/imgui_xp.cpp`).
- **The terminal is setup and diagnosis** (`src/ui.rs`, renders under `TestBackend`).
- **Settings are a registry** (`src/settings.rs`, `Arc<Config>`, `settings::live()`): env > pref >
  default; an env-pinned knob is REFUSED on screen, naming the variable. Text knobs reach the panel
  as rows on `xcopilot/opt/text`, edits come back on `xcopilot/opt/text_edit` (Enter or focus loss).
- **Only settings persist** (`prefs.json`, env, geometry); a `mission.json` is parked. Never a state
  dump: persisted facts outlive their context.

## Web API load is a sim cost

The API runs in X-Plane's flight loop: every request is frame time. `crates/sim/src/xplane.rs`
reports every 30 s (blackbox `api_load`, per-dataref histogram). A 60 ms read cache (a write drops
its entry); near-static flags (`is_in_replay`, `paused`) are `SLOW_DATAREFS`; a COMMAND clears it.
`cached=0` is an honest answer.

## Battle-test surface

`cargo test --workspace` (root `cargo test` is the crew package only). **Run tests with
`XCOPILOT_BLACKBOX=off`**, or they record into the live log.

**A fix merged offline gets an `unflown` issue** listing the blackbox fields to check; the sortie that
confirms it closes it (`fixes #N` in the commit). Known gaps are `owed` issues.

- `crates/atc/src/tower.rs` matrix; `crates/atc/bench/atc_corpus.yaml` (real whisper, **add every
  new wart**); `route_keyed`/`route_unkeyed` in `crates/atc/src/station.rs`; STT eval
  `cargo run -- stt-eval bench/atc_stt_cases.yaml`; world sweep
  `cargo test --release -p atc -- --ignored atc_world_sweep`; ILS smoke
  `cargo test -- --ignored real_install_ils`; `docs/atc-chaos-card.md`.
- `xvl debrief` (`src/debrief.rs`) after every flight, before diagnosing by hand.
- **A sortie is one file with a verdict** (`full_auto::SortieGuard`, `sortie`,
  `logs/sorties/<id>.jsonl`; `src/scorecard.rs` `stage_result`, `sortie_result`: PASS with no failed
  stage, no give-up, no crash inside it, at least one stage scored, and UNATTENDED: no `intervention` and no `tool` record, the pilot's command to
  the crew), grouped by mission in `xvl debrief`. `state` logs `in` and `ctl`; speech is `said`. A
  complaint not yet a scorecard number is not yet testable.
- **A verdict that is wrong hides the real failures** (2026-09-26): the crew's own flare and
  rotation read back through the yoke are not a hand (`monitor::note_crew_yoke`; the scorer
  discounts the same window in old logs), nor is a brake latch our own command raised
  (`monitor::CrewBrakeLatch`, `crew_brake_discounted`); a taxi is judged from its first tick ON the line after a
  join and never past its planned stop, and inside a route VERTEX it turns through (the projection
  jumped across it) is a cut, `TAXI_MAX_CUT_M`, not off the line; a drive that stops short (`stagnant`, `off_route`, a
  give-up) fails; a landing is scored at its FIRST contact and at its sink: the flight model's `touchdown` when logged,
  else the autoland window ending at the contact, else `touchdown_sink_fpm`, never the max of two
  windows (461 fpm on a 130, LEMG 31 2026-09-28); a bounce is recorded (`bounce_ft`). **A blackbox
  record is one write** (`blackbox::append_line`, one `write_all` under a file lock: the runner is a
  second process on the same file; 70 corrupt lines 2026-09-28).
  `bench/sorties/` holds trimmed real sorties whose verdicts `cargo test` pins.
- **A night of landings is one command** (`xvl sortie bench/sorties/<spec>.yaml --repeat N`,
  `src/sortie_runner.rs`): set up, type the mission, the `sortie_result` is the row
  (`logs/sorties/run-<stamp>.md`); an approach setup is its own sortie (`setup::hold_sortie_while`).
  It refuses over a flying loop or replay; a crash counts only once the wreck flag has read clear
  since the case's line (`CRASH_CLEAR_S`), and a wreck that outlives the setup stops the run.
  **An airborne aircraft is never handed to nobody** (`hand_back`): on the ground a crash or deadline
  types "my controls"; in the air the loops keep it until the next setup, and a run that ends in the
  air pauses X-Plane (`secure_at_end`); Ctrl-C pauses first. A line the app has TAKEN is never
  retyped (`Phase::Queued`, `QUEUED_S`). **No setup under a moving aircraft** (`SettleWatch`,
  `settle_before_setup`): before each setup the last aircraft is at rest (marker idle, on the ground
  under `STOPPED_KTS`, held `QUIET_S`), waited up to `STOP_MAX_S`, then "my controls" (`hand_back_hold`
  parks); in the air it is taken back at once; the row's "Before" says which.
  A case scripts `steps` (`say` at `start`, `after_s`, `ias_above`, `agl_above`, `phase`, `event`),
  loads `setup.aircraft` (waits for its `profile_probe`), reads `plan_origin` / `runway: plan` from
  SimBrief, and may `expect_give_up`; `runway: wind` is the tower's into-wind end at the setup (`resolve_wind`, METAR else
  the sim's wind on the field); an unset deadline scales with the approach speed; **a reposition or
  swap is a new flight to the scorer** (`Scorer::new_flight`); specs `bench/sorties/backlog-737.yaml`, `backlog-types.yaml`.
- A law that misbehaves in the air while its test passes: suspect the plant. A first-order lag
  cannot ring; fly second-order plants through the real transport delay.

## Licensing

**GPL-3.0-or-later** (`LICENSE`, `[workspace.package]`): open source, never commercial, X-Plane
only, copyleft on purpose. SDK and Dear ImGui MIT/BSD, B612 OFL. Relicensing is free only until
outside contributions merge. `crates/atc` could become LGPL-3.0; deliberately not done.
