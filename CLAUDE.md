# XFlightCrew

The rest of the crew, and the world around you, for X-Plane 12: copilot, ATC, AI traffic and
cabin crew, local and deterministic, with voice as the interface (formerly XVoiceLayer, binary
`xvl`, renamed 2026-10-13). The pilot speaks, the grammar picks a tool, the agent writes the dataref, the copilot reads it back aloud.
**There is no model at runtime** (retired 2026-09-13): every miss in the logs becomes a rule.

```
🎙️  "Flaps up"   →  STT  →  grammar picks a tool  →  write dataref  →  TTS "Flaps up!"
```

**The evidence lives in [docs/dev-history.md](docs/dev-history.md)**: every sortie, date and number
behind these rules. Grep it BEFORE reworking any subsystem named here. This file keeps each rule,
its one-line why and its home in the code. Each section's earlier text is in the history under
"2026-09-26 trim: <section>" and "2026-10-06 trim: <section>", so any airport, date, number or
identifier that left this file can still be grepped there.

## Direction (2026-10-07, `docs/tasks/pillars-audit/REPORT.md`)

Three pillars: **copilot + checklists, ATC, AI traffic** (our own, REPLACING LiveTraffic; the
tower controls it). **The Cessna 172 is the reference airframe**; other profiles stay as they
are. **Autonomy is redesigned in layers, loop by loop** (docs/tasks/autonomy-layers): the
aircraft's own autopilot flies where a pilot would use it, and ONE pilot controller (`src/pilot`)
flies by hand what a pilot hand-flies, the same laws for every fixed-wing airframe, every airframe
number MEASURED in flight, never typed into a profile. Laws are designed from first principles on
plants FITTED to recorded flights, never tuned flight by flight. Done: the final approach and
landing (2026-10-12). The other flying loops (full_auto, circuit, procedure, autotakeoff,
autotaxi, taxi_in, pushback, heli_*, recover, reversal...) stay FROZEN until their turn: crash
and safety fixes only. "Every miss becomes a rule" is for the grammar, never a flight law.
Order: stage A (untangle copilot from autonomy, station behind `party_line`), B (command loop),
C (ATC per-flight strips, `crates/nav`), D (`crates/traffic`).

## Wire contracts (never rename)

Named 2026-07-30. These names survived the renames because they are wire contracts:

- the `xcopilot/*` dataref namespace (app and plugin agree on the strings),
- the `XCOPILOT_*` env vars (the pilot's own `.env` and `prefs.json`),
- the plugin SIGNATURE `com.dlicudi.xcopilot`.

The plugin itself was renamed XFlightCrew on 2026-10-13 (`plugins/xflightcrew/mac_x64/
xflightcrew.xpl`, source `plugin/src/xflightcrew.cpp`): `make -C plugin install` removes the old
`plugins/xcopilot/` folder FIRST, since two copies would both load and register every dataref
twice. A hand install must do the same.

Package `xflightcrew`, binary `xfc`. The tracing filter names every crate (`atc`, `nav`, `sim`)
AND both crew names: `xflightcrew=info,xfc=info,atc=info,nav=info,sim=info,traffic=info` (`src/main.rs`
`DEFAULT_LOG_FILTER`, pinned by
`the_log_filter_names_every_crate_and_both_crew_names`). A line's default target is the BIN
name in the binary and the PACKAGE name in the lib: drop either and half the log goes silently while
explicit-target lines make it look healthy.

## Stack (decided)

Rust: a latency- and audio-sensitive real-time loop.

| Stage | Crate / approach |
|---|---|
| Mic capture | `cpal` |
| STT | `whisper-rs` (whisper.cpp bindings), push-to-talk |
| Intent | deterministic: `src/router.rs` (command / question / follow-up / echo), `src/grammar.rs` (the tool: an ordered `MATCHERS` table over `src/grammar/*`, order is semantics), `src/conversation.rs`, `src/answers.rs`, `src/plan.rs` (questions), `src/dialogue.rs` (back-references); `main.rs` boot, the command loop in `src/main/` (`session.rs`: take, route, dispatch, answer, speak), `src/detect.rs` airframe swap |
| X-Plane write | `reqwest` PATCH to the Web API (`PATCH /api/v1/datarefs/{id}/value`) |
| TTS readback | in-process piper VITS via the `ort` crate |

- **No LLM, local or cloud**: `xfc analyze` lists the top missed phrasings; each becomes a rule,
  matcher or profile entry with the live transcript as its test (`--now --since <date>` replays
  misses through TODAY's grammar). A model returns only as the instructor answering handbook
  questions, never a fallback.
- **REST for transport, plugin for capability.** Stock datarefs go over the Web API; the C++ plugin
  (`plugin/src/`: `xflightcrew.cpp`, `drefs.cpp`, `panel.cpp`, `fms.cpp`; `make install` in `plugin/`)
  adds `xcopilot/*` datarefs driven over the same API. The app never links the SDK.
- **Voices can ride the sim's audio** (`plugin/src/audio.cpp`, `XCOPILOT_AUDIO_BUS`, off by
  default): `xcopilot/audio/cut`'s ONE writer is `tts::play_on_bus`, so a cut never overtakes the
  next play; a failed post plays locally.
- **Voices and speech models are installed by `xfc voices`** (`src/voices.rs`, into
  `config::models_dir()`): the third outbound call after SimBrief and the METAR file, on demand
  only, never at boot or in the command loop; every download size- and checksum-verified before
  its rename, an in-use model never removed.

## Core design rule: curated tool surface, not raw dataref access

**Nothing writes arbitrary datarefs; the grammar picks curated tools.** Each owns (1) its dataref and
range, (2) a **readback** re-read AFTER writing and compared with the INTENT (a silently failed write
must not answer "Roger"), (3) **safety gates** ("gear up" on the ground refuses and explains).

- **Back-references** ("a bit more", "do that again") resolve against the calls actually executed
  (`src/dialogue.rs`): CLOSED vocabulary, 120 s TTL, automations never re-armed, cleared on swap;
  they ride the tool path, gates included.
- **State questions** answer from the SAME datarefs the tools write; the question OPENER routes
  them. A judged speed question gets a verdict (`speed_verdict`), never a bare number.
- Weather away from the aircraft comes from the METAR file (`src/weather.rs`), never the datarefs of
  the air we are in. A named fix is matched fuzzily against the LOADED plan only (`fix_query`).

## Aircraft profiles drive the tool surface

Profiles (`src/aircraft.rs`, `profiles/<id>.yaml`, `XCOPILOT_AIRCRAFT=auto`) are detected from the
profile's `match:` block (`acf_ICAO` AND the `.acf` path, `detect::resolve`, one resolver for boot
and swap), never the ICAO alone: the stock 737 and the Zibo both say B738. An unmatched model of a
known type is generic, said aloud. Plan-type questions use `aircraft::profile_for_type`. Profiles
declare systems and limits; the registry is built FROM them. Datarefs and commands stay in the
Rust tools; YAML is per-airframe config.

**Numbers are the airframe's**
- **All automation numbers are per-airframe**, the old GA figure when silent: taxi pace,
  `takeoff_roll_ft` (checked) vs `takeoff_distance_ft` (recited), pattern numbers, valve sense.
- **"Enough runway" is ONE rule** (`aircraft::takeoff_needed_m` / `takeoff_needed_now_m`; as five
  copies a fix reached one); a failed fit is SPOKEN by its gate (`autotakeoff::no_fit_line`). Its
  landing twin is `aircraft::landing_needed_m` (the BOOK figure).
- **One home each**: who flies `flight_state::Owner` / `crew_has_aircraft`; stopped `sim::ground`;
  angles `sim::angles`; AP engage `ap_engage::ApEngage`; speed ceiling `ceiling_from_vne` (0.9 Vne less 10).
- **The autopilot panel has ONE owner** (`src/autoflight.rs`, pinned by
  `every_mcp_write_goes_through_the_owner`). Modes' truth is the `autopilot_state` bitfield
  (`Modes`), or a declared MCP's own lamps where the stock modes do not follow it (the Zibo:
  `lamps:`, `windows:`, switches; every status read is `read_status`, the bitfield
  `read_ap_state`), the panel the airframe's (`Panel`, `autoflight:`). A loop states INTENT (`Want`);
  `reconcile` presses on a difference only, never over a capture, nothing on a failed read (blind
  presses dropped most 737 captures). The window is parked where no capture takes the wheel
  (`WindowRule`). Primitives: `engage_confirmed`, `hold_level`, `hold_heading`, `climb_to`,
  `take_thrust`, `drop_thrust_now`, `arm_approach`. Its ledger (`ours_since`, `Ours`) answers "was
  this change ours?". **The thrust has one owner**: a loop flying the levers takes an ENGAGED A/T
  out to ARM (`ThrustWatch`).
- **A lever-driven A/THR flies the speed, as an Airbus crew has it** (`autoflight:`
  `athr_cl_detent`, the A330; `speedhold::athr_tick` on `ap_lands::AtSpeed`): the governor's
  airborne hold sets the FCU window, puts the levers IN the CL detent, reads the A/THR back
  active and stands aside, its thrust watch quiet; dropped out or astray 30 s (`athr_astray`), our
  lever again, said once, kept BELOW CL (`manual_lever_cap`): our lever through FLX re-armed it
  every frame (the A333 script) and three takes "would not stay off" (#295). The takeoff's
  reduction puts the levers in CL and hands it the climb (`takeoff_watch_reads`).
- Multi-engine tools take `engines`, read back the LAST element; singles stay byte-identical.
  `vyse_kts`, `vmca_kts` stay `None` on singles: say nothing rather than invent.
- **An unknown type is never silently the 172** (an RV-10 stalled on its numbers):
  `aircraft::generic_from_acf` reads the `.acf` and says "no profile"; only an unreadable file
  gets the 172, said aloud.
- **A slatted wing's stall is per notch** (`aircraft::stall_speed_in_config`,
  `detent_stall_from_acf`): monitor, `config_floor_kts` and `speed_verdict` read one table.
- **No automation target below the speed floor** (`aircraft::speed_floor_kts`; raised and said by
  `speedhold::arm_flight`, `run_autoland`); nor above the ceiling (`speed_ceiling_kts`). **A speed
  hold on the runway has landed** (`touchdown_idle`): close the throttle, say so, let go.
- **A flow runs at crew pace and the voice says milestones** (`jet_start::AirlinerStart::run_steps`):
  fire, READ BACK, a beat; a set switch passed over; a switch that never took named at the end.
  Twenty presses in two seconds is a macro.

**A profile line is a hypothesis until probed**
- `aircraft::profile_references` walks every name, `dataref_exists` / `command_exists` ask the sim,
  `probe_summary` speaks the missing. A new field is probed the day it is added; never into a load
  (`detect::missing_with_cache`).
- **Rust constants too**: read names come from `datarefs::*`; `cargo test -- --ignored
  real_install_names` checks every `"sim/..."` literal (made-up names fed fallbacks for months).
- **The plugin side too** (`health::PLUGIN_DATAREFS`): a stale `.xpl` is said at boot. Editing
  `plugin/src` is done only after `make -C plugin`, install AND reload.
- **Position is one frame** (`xcopilot/position`, `sim::position::read`): split reads pair frames.
- **A failed read is NO measurement**: skip the tick, blackbox `null`. A "dataref" may be a
  command; read-only switches need read-before-fire `*_toggle` tools. Paired lights written together.
- **A read-only KNOB is stepped by its commands** (`radio::step_knob`): a write is a silent no-op.
  **The gear comes up outside its own warning** (`autotakeoff::gear_up_due`, `GearWarning`).

**Add-ons and process-wide facts**
- A stock dataref can be an add-on's OUTPUT: when a tool "does nothing", read the Lua. Declared
  knobs are CONFIRMED through one writer (`radio::set_mode`), from the box's OUTPUT
  (`radio::stock_transmits`).
- **Selectors are republished on EVERY swap, including to `None`**: facts must never outlive their
  airframe. `src/flight_state.rs` is the one page (`publish_airframe`; `stand_down` cancels every
  run in `flight_state::LOOPS`). A new flight or airframe goes through ONE door, `flight_begins` /
  `airframe_begins`. A loop OWNS its flag through its `Slot` lease (`claim`, `take_over`); autoland
  launches are `run_autoland_scoped`. Every static names its lifetime (`src/lifetimes.rs`,
  `crates/atc/src/station/state.rs`).
- **Anti-ice** (`src/tools/anti_ice.rs` `set_anti_ice`, `systems::IceKit`): read first, read
  back, an undeclared wing left alone and said; the monitor's `icing` advises a pilot once and
  switches off only its own.
- **A cart and a contactor are two things**: ground power moves the cart only when presence
  differs from want (a blind toggle dismisses it) AND `gpu_on`, and reports BUS volts.

**A crew configures before it moves** (`src/takeoff_config.rs`): flaps, trim, altitude dial
(`initial_altitude_ft`; a pilot's dial stays), spoilers, autobrake. Read first, read back, named
when not taken. Thrust against a set parking brake is idled first (`thrust_against_brake`).
- **Armed for the roll is an Airbus habit**: a Boeing declares `takeoff_speedbrake_down`.
- **Cabin air** (`src/tools/cabin_air.rs`): bleed and oxygen SUPPLY on for every mission; past the
  warning band a flying follower is handed 10,000 ft. **The masks are not the supply**
  (`oxygen_masks`, `MaskWatch`): donned in the warning band, doffed only if ours, never over the
  pilot.
- **Centre tank pumps** (`src/tools/fuel_pumps.rs`, monitor `centre_tank_dry`): off when empty
  under an automation, advised to a pilot; a main tank never.
- **Seat belt signs** (`src/tools/signs.rs`): on before anything turns, off through ten thousand,
  on again for the arrival.

**Unasked actions** (`src/cockpit_auto.rs`): a state machine over conditions, one-shot per spell,
never re-asserted (a watch that re-fires fights the pilot). "Off" only for lights it lit; INTERIOR
lights only (test-enforced), through `tools::lights::apply`. The DG auto-sync is said only when the
drift READ BACK fell (`monitor::GyroSync`), else latched off per airframe.

## SimBrief

`src/simbrief.rs`, `XCOPILOT_SIMBRIEF_ID`: the one other outbound call, on demand, never realtime.

- **FMS injection is plugin-only**: `xcopilot/fms/route` then `xcopilot/fms/load`; the plugin
  loads the plan WHOLE (only a load fills ORIGIN/DEST), in its flight loop (`FmsLoop`).
- **The follower sequences the FMS** via `xcopilot/fms/goto`, from its OWN active leg. Where the
  airframe declares LNAV (`autoflight:` `lnav_command`/`lnav_status`, the 737) the enroute
  follower's plain legs to a fix fly on it once the FMS's active ident IS the leg, the bug the
  watched fallback, said and retried next leg (`src/lnav.rs`, #227); elsewhere the bug, never GPS.
- **The plan is the WHOLE navlog** (`simbrief::fms_route_payload`): thinned from the MIDDLE. A cap
  on the front of a list amputates its end.
- **Local planner** (`src/flightplan.rs`, `src/route_finder.rs`; "plan a flight to X"): the same
  `FlightPlan` (`PlanSource::Local`), airways or direct round restricted volumes (one it cannot go
  round is NAMED); a short destination runway REFUSED; only an explicit refresh replaces it.
- **"Fly the flight plan" is preflighted AT THE ARM** (`full_auto::preflight`): a refusal names the
  number; an unknown is skipped, never guessed. On the ground the arm fetches the plan fresh
  (`ArmFetch`) and loads only what is not aboard (`arm_load`). An airborne resume does neither.
- **A MARGIN is the pilot's to waive, the airframe never** (`full_auto::Gate`): "fly it anyway"
  waives only the runway factor and the plan's weight limit (`grammar::waiver_request`,
  `aircraft::TAKEOFF_MARGIN_WAIVED`). Fuel, ceiling, destination runway: never.
- **A spoken ask for the plan re-fetches EVERY time** (`PlanFetch::Asked`; `plan_change_line`).
  "Load it" is a separate word; a plan on the shelf authorises nothing.
- **The load sheet** (`src/tools/loading.rs`, `load_aircraft`): ground and stopped; fuel wings
  first (`distribute_fuel`), refused past the limits with both numbers, read back. "Refuel" is the
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
- **Nav data is its own crate** (`crates/nav`: apt.dat, earth_nav/earth_fix, CIFP, `TaxiNet`,
  paint, geometry), pure data, never station state, re-exported as `atc::nav` so every path
  stands; a traffic crate takes `nav` without the station (`cargo tree -p nav` lean too).
- **The crew/world seam is one page**, `crates/atc/src/party_line.rs`: statics crate-private; a
  new crew need gets an ACCESSOR, never a `pub static`. `station` is `pub(crate)`: the crew's
  inputs, reads and types are `party_line`'s, the ticks `atc::world`'s
  (`the_station_is_reached_only_through_the_seam`).
- **One strip per flight** (`party_line::FlightStrip`, every cell inside its lifetime group):
  `OWN_STRIP` ours, `STRIPS` the traffic's (cleared by install/uninstall, never by a new flight of
  ours); the desk half (`StripDesk`, tower FSM and worksheet) per flight, a traffic desk swapped
  in only for its synchronous answer (`Station::respond_for`, drop guard); the ticks are
  `_for(strip, state)` cores. `FlightId::Own` is never keyed by callsign. **A new flight of
  ours at the field already up resets OUR part in place** (`Station::new_flight_here`: our desk,
  our strip's cells, `NEW_FLIGHT_HERE_RESETS`), never a reinstall: a rebuild clears `STRIPS` and
  the traffic link's reinstall rule despawns every moving aircraft (LEMG 2026-10-13).
- **The runway book is COMPUTED, never stored** (`station::RunwayBook`): every strip's claim from
  its clearance and position, plus the ghost memory; `own_strip_claim`, `player_blocked_strip`,
  `runway_occupancy_authorised` and the holds' releases are its queries (pinned to the old
  functions by `the_runway_book_answers_as_the_functions_it_replaced`).
- **The landing sequence is COMPUTED too** (`station::landing_sequence`, 2026-10-14): every
  strip's runway times as its producer works them out (`party_line::RunwayTimes`, a strip
  `watch` cell fed by `world::note_traffic_times` / `note_own_times`), ordered by time to the
  threshold; `own_number`, `own_leader` its accessors. **An order once SPOKEN to us holds unless
  the gap reverses by more than 30 s** (`SEQUENCE_REVERSE_S`, `arrival_order`).
- Tests touching a process-wide static hold `SIDE_STATE_LOCK` (a test opening a traffic strip
  clears `STRIPS` through a drop guard, `strip_tests::ClearStrips`); setting the mode,
  `mode::TEST_LOCK`.
- `Station::install()` publishes side state, never `from_airport`; `uninstall()` retires it all.
- **A mishear may earn "say again", never a clearance.** Only mayday/pan-pan lands without a final.
- **One callsign for tower and crew** (`atc::callsign::resolve`): env > plan > tail, read through a
  crew provider, never stored; the pilot's telephony (`airline_telephony`) first.

**Tower behaviours** (each one's sortie is in the history)
- Readbacks enforced for runway-critical items (`XCOPILOT_ATC_READBACK=false` coaches only); the
  zone clearance's is suggested. A departure check-in is answered with its climb (`climb_announced`).
- **IFR clearance** (`Intent::RequestIfrClearance`): the SID JOINS THE FILED ROUTE
  (`nav::sid_for_filed_route`), never invented; amended by name (`Station::amend_sid`); read back
  ENFORCED; seen by the crew only once accepted (`party_line::assigned_clearance`). **Cancel IFR**
  (`Station::cancel_ifr`) retires it all.
- **"Radar contact" is the radar unit's, on its own frequency** (`squawk_tick`, `radar_unit`).
- **Levels are the field's**: at or above the transition level a level is a flight level
  (`Ctx.transition_level_ft`, `tower::climb_item`).
- **Runway in use** follows the live stream in calm (`field_streams`), **one strip, one direction**
  (`runway_follows_stream`), and **the party line clears nobody against our runway**
  (`own_strip_claim`). A turn of our runway is SAID (`state_runway_change`). FROZEN while committed;
  a stated runway stands when the wind does not argue (`stated_runway_stands`). **The wind picks a
  direction, the longest parallel the strip** (`best_runway_for_wind`, `close_parallel_main`).
- **Release threads the arrival stream** (`DEPART_GAP_NM`); **a strip cleared to land and still
  flying is an ARRIVAL to a departure, never "traffic on the runway"**
  (`RunwayBook::departure_blocked`), at any distance, and **a departure goes ahead of it only if
  the lander's own go-around watch would never revoke it**: ONE rule for reply and release
  (`DepartureBlock::holds`, `departure_clears_lander`: the predicted exits fed through
  `go_around_due`, `LINE_UP_S` from the hold, `LINED_UP_ROLL_S` lined up; in time, never
  distance). **A hold keeps its word**
  (`on_independent_final`, `occupies`, `TakeoffHold.held_for`).
- **Circuit traffic stays with the tower** (`tower::wants_circuits`): cleared FOR THE CIRCUIT
  (`tower::circuit_clearance_lines`), no departure handoff. **A reject is a radio call**
  (`Intent::RejectingTakeoff`): the clearance and all it armed die. **The tower's book is the
  runway's** (`station::runway_occupancy_authorised`).
- **The runway watch ticks whoever flies the final** (`go_around_tick` / `landing_hold_tick` ahead
  of the monitor's `quiet`); **the crew never lands uncleared** (`UnclearedWatch`). **One departure
  on the runway at a time** (`WAKE_SECS`, ours included: `wake_remaining_for`), **and a takeoff
  clearance's claim ends when the aircraft flies, ours too** (`RunwayBook::add_strip`), never on
  a next call a hand-flown departure may not make; departure clears no higher than the FILED cruise
  (`capped_departure_climb`).
- **A landing is never cleared onto an occupied runway, and the go-around is judged on time**
  (`LANDING_HOLD`, `landing_release_due`, `go_around_due`, `strip_exit`); **an occupant the tower
  cleared for takeoff is a DEPARTURE, judged on its roll, never as a rollout**
  (`departures_on_the_ground`, `strip_exit_as`). **An occupant is the
  PAVEMENT's** (`on_runway_pavement`); traffic that will not leave re-sequences
  (`resequence_runway`).
- **Our arrival is in the traffic's slot picture and the traffic yields to it**
  (`traffic::slot::own_slot`, `traffic_link::own_arrival`): the traffic's own slot rule for our
  type down the path we fly, committed in the picture (we are the player); a straight-in behind
  orbits or slows, a circuit extends (`slot_free`, `off_before`, `leader_ready`, no second fit
  test). The tower naming US as a leader is `traffic::fleet::OWN_ID`.
- **Approach says our number and the traffic we follow ONCE per change, and controls our speed on
  the traffic's own bands** (`station::approach_sequence_lines`, `APPROACH_SPEED_BANDS` pinned to
  `traffic::arrival::SPEED_BANDS`): a band only where our type can fly it (the crew's
  `provide_approach_speeds`; unknown, none), the minimum approach speed behind a leader we are
  short of, then the join moved out ("extend downwind"); the 160 simply ends at 4 nm. **A speed
  is a readback item** (`judge_speed_readback`, `party_line::assigned_speed`), void at the tower
  handoff and a go-around. Blackbox `sequence`, `speed_assigned`, `downwind_extended`.
- **A final report behind traffic landing ahead is number two, never a clearance the tower must
  cancel** (#237, `station::landing_leader_blocking`, `tower::LandingLeader`): "continue
  approach, number two, traffic ...", the clearance held (`LandingHold::behind_lander`) until
  the runway is CLEAR.
- **Taxi routes**: Dijkstra over apt.dat 1201/1202; crossings from strip GEOMETRY, never network
  flags; a runway destination snaps to a node ON THE DESTINATION STRIP's pavement. A refusing gate
  names strip and distance (`NoFit::NoneNear`).
- ILS from earth_nav.dat + CIFP; dialect by ICAO prefix (FAA vs ICAO/CAP 413); `qnh_clause` is the
  one pressure seam, `suggest_call` the one source for coach and panel.
- **The tower's wind is the FIELD's** (`station::set_surface_wind`). A frequency read back is a
  readback (`frequency_readback`). **A REPORT spends nothing** (`Intent::RunwayVacated`).

**Audio and PTT**
- TWO half-duplex channels, intercom and radio. A routine intercom line is cut when the radio keys
  and resumes once quiet (`interrupt_intercom_for_radio`); a Warning talks over and is never
  stepped on. Radio-only cuts never touch the copilot.
- **A position report dies with the position it reports** (`crew_radio::holds_for`, `stale_reason`;
  `crew_radio_stale`, `crew_radio_collapsed`).
- **A crew call is routed on the frequency it was keyed on, before it is spoken**; a live number
  (`CrewCall::CheckIn`) is rendered as it goes out. With a radio key bound, **physical routing
  beats content routing both ways**.
- **The command loop never holds a pilot's line behind its own speech** (`pilot_line_pending`),
  **nor an armed station swap**, which only the loop installs (`loop_owed`).
- **Any PTT mutes everything while held**; release rejoins LIVE. Silenced lines are OWED
  (`voice::SpeechMemory`, `router::recall_line`). **The frequency's "say again" repeats the
  controller** verbatim (`voice::controller_repeat`).
- **A station hears only its own field on the ground** (`GROUND_REACH_NM`): beyond it full auto
  departs UNCONTROLLED (`party_line::plan_self_taxi`); airborne the last controller keeps it.
- The radio needs POWER both ways. **FAIL OPEN, the flag RESTORED every tick**: a fact that merely
  stops being published latches across a swap (recurring bug class). `take_crew_radio` drains
  unconditionally.
- ONE frequency: speech renders as whoever COM1 is on (`tuned_speaker`); the rest is the void.
- REPLAY gates the reactive surface; PAUSE freezes ticks, not audio; `xplane::sim_frozen` covers
  both, failing open.
- ONNX session CREATION is serialised (`src/tts_onnx.rs` `build_session`): two first sessions at
  once segfault. Barge-in kills by REGISTERED pid + per-channel cut generation, never pkill.
- `Voice::alert` mirrors to the panel transcript itself: mirror OR `s.action(...)`, never both.

## Ground & flight automation

Orchestrators compose proven loops; no new flight laws in them. Laws test as plain `cargo test`;
everything blackboxes to `logs/blackbox.jsonl`. A record's kind is the first argument of
`blackbox::record` (about 90), listed by
`rg -oNU --no-filename 'record\(\s*"(\w+)"' -r '$1' src crates | sort -u`; a name like `ap_engage` or `recovery_plan` is usually an `event` field inside a kind.

### Holding, writers, reads
- **An airborne aircraft NOBODY flies is held safe** (`src/safe_hold.rs`, its own thread, never
  stood down): no loop, no pilot (a dial moved that `autoflight::ours_since` says was not ours is a
  pilot). The autopilot's path must end above the ground ahead plus 1,000 ft, else a climb or
  bounded descent through the panel owner, watched, else `recover::take_over`; the ESCAPE
  (`escape_due`) acts at once. Never judged on an aligned final (`on_approach`).
- **A stage not moving the aircraft is holding it**: brakes for every wait, abort and give-up. **A
  mission that ENDS on the ground stops and parks** (`tools::controls::hand_back_hold`, "my
  controls" included), braked from ANY speed, the park brake confirmed and said.
- **One writer per dataref.** Plugin channels (`taxi_steer`, `taxi_brake`, `pitch`) are 1 s
  expiring heartbeats written EVERY tick (a change-gate drops them mid-turn); yield latches
  (`steer_yield`, `pitch_yield`) drop the override that frame.
- **Park brake through the valve-aware seam** (`ensure_park_brake_valve` /
  `release_park_brake_for_roll`), never a raw `parking_brake_ratio` write.
- Batch a tick's reads in one `tokio::join!` (sequential HTTP wrecks it); check `dt` before gains.
- **Signed vs unsigned**: groundspeed and route distance are magnitudes; directional values keep
  their sign (`taxi_guide::project_signed`); gates use `.abs()`.
- **Readbacks and echoes never arm automations**: arming grammars take request shapes only
  (`looks_like_command`). "stop" routes by context.

### Taxi, push, line-up
- **Auto taxi** (`src/autotaxi.rs`): speed via `hold_speed`; pure pursuit (`bend_lookahead_cap_m`);
  an off-route engage is a JOIN (`MAX_JOIN_M`); a lost governor on the ground ARRESTS. Castoring
  nosewheels brake-steer through the PLUGIN channels only (REST brake writes are re-driven by
  hardware), the rollout too (`autoland::RolloutBrakeSteer`).
- **A drive is judged by the ground it makes, never a flat clock** (`autotaxi::ProgressWatch`).
- **Traffic ahead is stopped behind its TAIL, planned for the governor's lag**
  (`traffic_guard::traffic_stop_in_m`, `GOVERNOR_LAG_S`): the gap less the ground covered in 3.5 s
  at our speed; our nose from the `.acf` (`GroundGeometry::nose_m`, ahead of the CG), the target's
  tail from its type (`traffic::perf::length_of`).
- **The world never favours our aircraft**: no silent right of way, ever (the pilot,
  2026-10-14); who goes first between us and traffic is Ground's to say, to us too (#297). Our
  auto taxi's route (`autotaxi::route_ahead`, `traffic::OwnTarget::route`) serves only the
  deadlock breaker: traffic standing on it waiting for us while we stand 2 x `STUCK_S` is towed,
  as two traffic deadlocked are, ours never (EGKK 2026-10-14, a 737 nose to nose for good;
  `spacing_cleared` `why: own`, `own_route_m`).
- **The sim brakes against hard rudder**, so the plugin holds `override_toe_brakes` while steering;
  a pilot brake latches `xcopilot/brake_yield`. Never "fix" a corner that stops the aircraft by
  tuning the governor.
- **A jump in a projection is never a movement** (`taxi_guide::project_signed_facing`,
  `autotaxi::join_fix`).
- **Routes ride the paint** (`ride_paint`): 1201/1202 topology, drawn `120` centrelines geometry,
  sliding sideways only; thin data degrades, never refuses. **An arrival's route is never a
  departure's clearance** (`taxi_to_stand`).
- **A hold is reached ON the line** (`autotaxi::arrived_on_line`). **Line up** rides the DRAWN
  lead-on (`TaxiRoutePlan.lead_on`; invented entries end in the grass), crosschecked against
  `party_line::clearance_runway`.
- **Pushback** (`src/pushback.rs`): tug commands are WHOLE manoeuvres, sent ONCE, never steered.
  **We stop the tug when we are there** (`PushJudge`; `SwingWatch` only blocks a false stop). **Any
  command to a tug ON THE BAR drops it off**; the bar coming off ENDS the push. A tug only ON A
  STAND (`pushback::on_a_stand`); **a light aircraft turns itself off the stand**
  (`pushback::tug_for_weight`).
- **Taxi guide** (`src/taxi_guide.rs`): never two voices on one mistake; gates on
  `pushback::ACTIVE`; "which way?" is question-shaped.

### Takeoff
- **Auto takeoff** (`src/autotakeoff.rs`): refuses without a clearance when a tower runs;
  `RollSteer`; stagnation reject on GROUNDSPEED (the ASI is dead when slow); rotation on
  `xcopilot/pitch`; the AP takes the ESTABLISHED climb at 300 ft, the VS dial written AFTER engage.
- **The centreline cut is sized for a CLOSURE TIME** (`autotakeoff::roll_target_heading`): a fixed
  gain weaves with speed. A steady drift is fed forward (`DriftFilter`). **The declared rudder is
  believed only up to its measured speed** (`roll_rudder:` `at_kts`); a propeller declares its
  standing rudder (`power_steer`).
- **A reject from any speed is FLOWN to a full stop, then parked, then handed back**
  (`autotakeoff::reject_rolling`).
- **A takeoff starts with the autopilot OFF and an engage reads the SERVOS**
  (`clear_inherited_autopilot`, `servos_on`). **A pattern that is no pattern is given back loudly**
  (`circuit::PatternGuard`), **but never to nobody** (`recover::unflyable`, `recover::take_over`).
- **The autopilot's ANSWER is watched, never assumed** (`src/climb_watch.rs`,
  `PatternTrip::ClimbNotHeld`): a runaway climb is TAKEN and the AP returned only once WATCHED
  holding. **Every vertical engage proves its pitch AUTHORITY** (`climb_watch::AuthorityCheck`),
  else hand flown (`ap_no_pitch`). **A recovery is an upset recovery** (`stall_roll_cap`,
  `ground_escape`) and **never writes into the next flight** (`recover::may_write`).
- **The handoff line is true** (`handoff_to_caller`) and **a pilot handoff is FLYABLE**
  (`handoff_waits_for_clean_up`). Flap schedules fit their placards (test-enforced); ONE speed per
  circuit leg (`circuit::leg_want_kts`).
- **A jet's climb-out is declared** (`autotakeoff::ClimbOut`): each flap step only once the last
  ARRIVED (`paced_flap_step`). **The initial climb holds V2 + 15 by pitch** (`ClimbHold`), the climb
  attitude a CEILING (`RotatePitch::ceiling_guard`). **The autopilot takes the climb being flown**
  (`ClimbVs`, `circuit::ClimbCarry`).
- **A jet configures its autoflight before the roll** (`takeoff_config:`; the A/T armed, never
  engaged) and reduces thrust at `thrust_reduction_agl_ft`; audited by `before_takeoff_audit`.
- **A runway start is a lined-up aircraft and still calls** (`tower::Ctx::own_on_runway`), never an
  incursion; cleared from its strip (`full_auto::zone_clearance_from_the_strip`). Too little ahead
  or the reciprocal is a BACKTRACK (`full_auto::RunwayStart`); another runway is taxied to, nothing
  read back from the wrong strip (`cleared_elsewhere`); an end opposing the live stream takes the
  stream's end.

### Circuit, procedure, full auto
(`src/circuit.rs`, `src/procedure.rs`, `src/full_auto.rs`)
- The pattern is sized to the AP's real turn radius, terrain and airspace (`pattern_speed_kts`,
  `climb_limit_fpm`); throttle laws watch a speed PROJECTION (`speed_governor:`).
- **The rectangle is judged on the CORRIDOR it is flown in, down to the runway** (`choose_pattern`,
  `corridor_frame`): raised, steepened, shortened, else REFUSED with the numbers; live,
  `live_floor_ft`.
- Final uses along-track `path_dist_m`; a REAL intercept (`final_intercept_hdg`); autoland only ON
  THE LINE (`ready_for_autoland`). **The turn onto final is sized on the bank flown** (`BankWatch`).
  The glidepath feeds forward `path_descent_fpm` (a P controller cannot track a ramp).
- **The sim's heading mode never reverses a turn in progress**: every steer goes through
  `heading_bug::bug_toward` (`every_heading_bug_write_is_turn_safe`: a direct write is allowlisted
  with its reason). **The AP's own capture holds level legs where declared**
  (`ap_altitude_capture`).
- LXGB's authored route joins at the nearest leg (`join_waypoint`) and is terrain-probed on its legs.
- Full auto: engine start per class (`StartKind`); power-up INLINE (racing bring-ups shut a
  cross-tie); waits on `driving()`, not `ACTIVE`; ends with `taxi_in_after_landing`.
  **A declared preflight runs between the battery and the ramp** (`src/preflight.rs`,
  `preflight:`; the Zibo's clist order, APU first so AC lamps read back), its IRS alignment
  waited on before the taxi; **an inline start's reply is its verdict** (`inline_start_hand_back`):
  engines not running once it answers ends the mission.
- **Airborne, "fly the flight plan" and "land the aircraft" are ONE planner, from ANY state**
  (`src/recovery_plan.rs`, `choose_join`): refused only for the physically impossible, with the
  number (`impossible`); the nearest runway only when asked, an engine out, or no plan.
- **Cruise is a SPEED** (`procedure::scheduled_cruise_kts`); **a planned step is flown at its fix**
  (`apply_step_climbs`). **The next fix is `procedure::active_leg()` first**: the GPS sits on the
  plan's first entry. **The arrival is briefed at the arm** (`full_auto::brief_arrival`).
  **A route-only arrival crosses its gateway on the field's 3 degree path**
  (`procedure::arrival_gateway_alt_ft`): never the last fix's cruise-side altitude.
- **Full auto flies the checklist items it owns; lists only cross-check** (mixture:
  `engine::arm_mixture_manager`, `mixture_rich_for_landing`).
- **Exterior lights follow the practice** (`src/tools/lights.rs` `Exterior`): taxi light to taxi,
  landing light over each runway span, both off passing 1,000 ft, all off at the stand.
- **The crew picks its own procedures** (`full_auto::choose_departure`,
  `plan_audit::choose_arrival`); a departure other than the cleared SID is ASKED for
  (`request_sid`), never flown silently. **A spliced SID flies the PLAN's levels**
  (`rederive_sid_altitudes`).
- **Charts are flown**: SID minimums (`DepartureMinimums`, `departure_target_ft`), STAR windows
  (`apply_arrival_constraints`) and speeds (`arrival_speed_cap`); the VS wheel only in VS mode
  (`vs_action`). **The 250 below 10,000 cap, once begun, holds** (`speedhold::AirspaceLimit`).
- **The whole plan is dry-run first** (`src/plan_audit.rs`): REFUSED or CAUTIONED by name; the
  all-clear names what could not be checked.
- **The crew works the radio as it flies**: **a readback goes out once**
  (`crew_radio::transmit_readback`), **heard, read back, then rolled** (`readback_before_moving`);
  `take_departure_handoff()`, `take_region_handoff()`. **An IFR flight keeps its code** (party line
  `IFR_FLIGHT`); airborne it goes to a radar position (`radar_position`).
- **The inbound call** goes to the destination's OWN tower (`inbound_due`) or on its challenge;
  `call_inbound` tunes COM1 and arms that station (`station::arm_station_for_freq`). A crossing
  field's challenge is `zone_challenge`, never "inbound". **An instrument arrival works APPROACH
  first** where the field has one (`full_auto::arrival_radio`, `party_line::own_instrument_arrival`):
  cleared on its own navigation (`Station::clear_instrument_approach`), handed to the tower
  established (`ApproachWatch`, `take_arrival_handoff`); an early tower call or a go-around goes
  back to approach. **"Final" is reported ESTABLISHED on it**
  (`autoland::final_report_due`, `established_on_final`), never as autoland takes the arrival.
- **Top of descent**: only `procedure::planned_tod` is announced; **while a follower flies, the
  plan's ring is silence** (`Snapshot::follower_flying`); only in cruise (`monitor::tod_in_cruise`).
  **A level the plan handed down is not argued with** (`STEP_CLIMB_MIN_CRUISE_MIN`).
- **A descent that outruns the governor gets drag, then a shallower rate**
  (`procedure::descent_drag`), never with the gear down or flap past the first detent
  (`drag_barred`).

### The follower's vertical and the altimeter
- **Three owners in order** (`procedure::vertical_tick`, pure): the PLAN's constraint, paced from
  the last COMMANDED altitude (`prev_cmd_alt`); the PILOT's altitude (`pilot_altitude::take`),
  never written to the dial; the GROUND AHEAD over both. **A descent never climbs back**
  (`descent_no_climb`), nor a climb descends (`climb_no_descent`).
- **Terrain floor** (`terrain::floor_ahead_ft`): a climb into it RAISED and said, a descent below it
  refused.
- **The follower flies the ALTIMETER** (`altitude_ft_pilot`); only the floor is true feet.
- **So the altimeter is kept right**: ONE rule (`instruments::correct_setting_inhg`) everywhere;
  the monitor's `AltimeterSetting` resets it under an automation, advises a pilot. `set_altimeter`
  steps in rounds EARNED by progress (`keep_stepping`). The transition is the FLIGHT's
  (`instruments::FlightTransition`).

### Approach and autoland
- **Every planned arrival hands its runway down** (`EndAction::Approach { rwy_id }`); the
  self-pick (`autoland::self_pick`) keeps to the plan's destination.
- **A new flight takes its clearances with it** (`party_line::retire_for_new_flight`); **its
  station is the field under the NEW, SETTLED position** (`station::arm_new_flight_at`; the field
  already up keeps its station, our desk reset, `Station::new_flight_here`;
  `setup::SettleJudge`); nothing arms while it loads (`setup::new_flight_loading`).
- **The tower's clearance outranks the plan** (`runway_from_clearance`), watched THROUGH the
  approach; it moves the ONE arrival runway (`full_auto::retarget_arrival_runway`). Arrival wind
  is the destination METAR (`weather::surface_wind`).
- **A straight-in to THIS runway or none** (`nav::straight_in_legs`), **only as good as its fixes**
  (`approach_plan_for`, `interceptable`). **A final is judged against the pavement it lands on**
  (`lands_on_the_pavement`), else it balks with height in hand.
- **A final asks for descent OR deceleration, planned back from the gate**
  (`src/approach_energy.rs`, `idle_decel_kts_s`); a planned final never accelerates
  (`energy_cap`). **Established on the final path beyond the FAP the descent is the final's**
  (`established_on_final_path`). **The follower crosses the FAF at the plan's join speed and
  flap** (`procedure::faf_speed_cap`, `faf_flap`: slowed for in time, the descent owed first),
  never clean at the clean approach speed. **A final that cannot make its gate takes the
  speedbrake** (`approach_energy::final_drag`, declared `speedbrake_max_ratio` only, back as
  found), and **TECS bounds a FAST error only by idle** (`pilot::energy::speed_term_fpm`).
- **A descent flies its speed first, at idle** (`speed_protected_vs`); the speedbrake is for
  HEIGHT. A STAR window over the cruise is flown at the cruise (`windows_above_cruise`).
- **One stabilised-approach rule for every final** (`src/stable_approach.rs`): a breach held 3 s
  goes around; the missed approach flies `missed_approach_speed_kts`, never full power.
- **The arrival is re-picked in flight** (`RepickWatch`). **An RF leg is flown round its arc**
  (`course_leg::arc_track`), **a hold as a racetrack** (`HoldFlight`), **a CF leg on its course**
  (`course_leg::course_track`); **an IF behind us is joined by autoland's pattern** (`FinalJoin`).
- **The autopilot is engaged and CONFIRMED, never assumed** (`ApEngage`): until then level on
  `xcopilot/pitch`, handed over after 10 s (`ap_not_engaged`). **The VS watchdog excuses a slow
  autopilot, never a dead one** (`vs_not_held`).
- **A mountain field is a flyable approach or a hand-over** (`mountain_field`); **a MAP on the
  runway fix is a landing** (`map_short_of_runway`). **The ILS is tuned, watched and flown
  coupled wherever the airframe's autopilot has an approach mode** (`src/ils.rs`,
  `coupled_allowed`: the profile's `ils_coupled` when declared, else the survey's
  `ApType::has_approach`); **a positive `vdef` is ABOVE the beam**
  (`ils::above_glideslope`). A declared MCP course window is the OBS's source (`windows.course`).
- **Where the airframe declares it, its autopilot LANDS and the crew monitors**
  (`autopilot_lands:`, `src/ap_lands.rs`, the Zibo): APP from below the beam, the captures the
  FMA's (`modes_with_fma`: an APP lamp is lit armed or captured alike), CMD B after G/S
  (`Landing`), its flare, retard (`AtSpeed`: the A/T in SPEED on our target) and rollout to
  `disconnect_kts` (`rollout_hand_back`); our arrest, column, lever and track law stand aside,
  the gates and the go-around door stay ours (TO/GA pressed). Not dual by `dual_by_agl_ft`: our
  own law, said; not armed by `land_by_agl_ft` or lost above `own_flare_below_agl_ft`: a
  go-around; lost below it: our flare. Every FMA column raw on `autoland` `coupled_tick`.
- **The copilot knows the systems before reaching for one** (`src/systems.rs`): a fact, never a
  licence. **The autopilot's own modes are tools, not yet the follower's** (`tools::autoflight`
  `ap_mode`): toggles read before firing.
- **"Set up the ILS" is a pilot monitoring's setup** (`src/tools/approach.rs`, `SetupWatch`): APPR
  armed only from OFF, read back, never retried. A localiser bearing is `magnetic x 360 + true`.
- **A reposition is a PATTERN, never a point** (`src/reversal.rs`), only when a re-intercept is not
  flyable (`straight_in_losable`); **the stabilised gate is about TIME** (`unstabilised_at_gate`);
  **its side and platform are the circuit's rule over its corridor** (`reversal::plan`); a refusal
  climbs and hands over level (`TerrainHold`). **No-go airspace is the circuit's rule too**
  (`side_airspace`; a side no pattern keeps clear of is flown for the ground and SAID,
  `reversal_airspace`), and **the downwind is never EXTENDED into it**: the height is lost in a
  hold short of it (`AirspaceLimit`, `limit_step`, `reversal_hold`), else it turns in high
  (`extension_capped`).
- **An approach never takes away the flap the wing flies on** (`flap_kept_flying`); flaps never
  cycle on a placard; **a climb is flown on thrust** (`climb_power_floor`); a give-up hands the
  SPEED to the governor (`hand_back_speed_kts`).
- **The final is flown as a pilot flies it**: ABOVE MINIMUMS the autopilot (its approach mode
  where coupled, else HDG set by `FinalHeading`: course, crab and a cut, whole degrees, no oftener
  than every 10 s, less the autopilot's own standing error, learnt); FROM MINIMUMS
  (`arrest::minimums_ft`: the coupled ILS's `ils_decouple_agl_ft`, else 200; any other final 300)
  the servos come out and `src/pilot` hand-flies to the wheels: bank through the ailerons to the
  PAINTED centreline (`pilot::line`, NPFG; a coupled line was 8 to 23 m off it), the rudder
  coordinating then DECRABBING before the wheels (`pilot::yaw`: the wing down into the wind with
  it), the attitude law for the path then the flare, power on TECS (`pilot::energy`). The heading
  bug is never a control stick: nothing writes it below minimums.
- **The pilot flies on MEASURED responses, never typed gains** (2026-10-13): each axis' response
  (`aircraft::learned::Response`: effectiveness, damping, the airflow angle's spring and path,
  delay) is an identification's (`xfc identify`, `src/identify.rs`, stored per `.acf` in
  `profiles/learned/`, keyed by the `.acf`'s fingerprint and a schema version, loaded by the
  detector into the facts' `acf_model`), else the fleet's prior; scaled by the air
  (`Response::at`); every loop's gains by ONE rule (`pilot::response::design`, McRuer's
  crossover at a 50 degree margin); the roll refined in flight (`RollResponse`). TECS's
  `K_thr2STE` and spool are measured or the book's (twice `best_climb_fpm`, the engine class).
  The NOSE is limited by the `.acf`'s geometry (tail strike, mains first; else the profile's
  `tail_strike_deg`), never the elevator. A profile never declares `flare_elevator`,
  `flare_power`, `arrest_lead` or `final_track` (`no_profile_types_the_pilots_numbers`).
- **Autoland core**: a sticky `MissedStage`; give-ups hand back a FLYABLE aircraft; the glidepath
  aims at the pavement (`aim_elevation_ft`). **The crab is LOW-PASSED** (`CrabFilter`, 5 s).
- **A tower's go-around order outranks everything, and reaches whoever flies the final**
  (`party_line::GoAroundOrders`). **The pilot's go-around is flown the same way**
  (`src/go_around.rs`, `GoAroundWatch`): closed words, reads back the loop's ANSWER. **A slow, low
  go-around pitches for speed** (`go_around_vs_fpm`).
- **The servos come out at minimums** (`arrest_hands_step`): an engaged servo writes the yoke
  back. **The AP comes out at the wheels** (`disconnect_autopilot`). **Once taken, the servos stay
  out** (`hold_roundout`).
- **The flare is the attitude law** (`pilot::attitude`), one law for every fixed-wing airframe:
  the nose angle for the path (gamma = theta - alpha) from a gate of the sink times 3 s, a
  smoothstep to a -70 fpm hold-off, a disturbance observer for the elevator the nose needs (it
  finds the low wings' ground effect: no declared pull); its path trim closes inside the flare and
  never winds below zero (a gust wound it down: 441 fpm). Tested on plants FITTED to recorded
  flights (`pilot::plants_tests`: six airframes' flares and arrests, the 737's identification),
  each IDENTIFIED first as `xfc identify` flies it, never on an invented plant (one with a lift
  slope five times too steep hid the windy sag).
- **The flare's power is TECS's**: height first, the lever only back, at idle with the engine
  spooled down as the wheels the flare PLANS arrive (`attitude::planned_s`); underspeed and bad
  descent protections on the hand-flown path.
- **A low flare with room to stop is COMMITTED** (`autoland::commit_flare`) and **stays committed**
  (`commit_flare_latched`). **The go/no-go is decided at the roundout gate, never at 4 ft**
  (`landing_call`, where `landing_roll_ft` is declared): the predicted touchdown must leave the ROLL,
  else "Long landing. Going around." with height in hand; a LAND commits the flare. **A low balk is called on the radio only once it CLIMBS** (`BalkWatch`).
- A debrief touchdown is the steepest recent sink (`instructor::touchdown_sink_fpm`).

### Taxi in
(`src/taxi_in.rs`) VACATE first, then "runway vacated", THEN the request. **A vacate turns toward
the parking and never onto another runway** (`nearest_stand`). **A stand route asked from the
runway starts with the vacate** (`station::stand_route_on`). **A jump in position is a relocation,
never a movement** (`autotaxi::relocation_jump_m`). **A mission ends shut down**
(`taxi_in::shut_down_at_stand`); a spoken "taxi to parking" leaves the engines to the pilot. **"My
controls" ends the taxi-in itself** (`taxi_in::abort`), braked and parked; a stand-down only lets
go.

### Still owed
Issues labelled `owed` + `area:approach-landing` / `area:takeoff` / `area:full-auto` (`ils_coupled` on the 737 is unflown through the owner, #81; the Zibo's `autopilot_lands` is unflown, its speedbrake not yet armed for it).
Also owed from 2026-10-05: the loops' mode presses go through `autoflight::bring` (reconcile,
then apply; `loop_mode_presses_go_through_reconcile`) except four that need a flight: the pattern
guard's hand-back VS, autoland's arrest return, window-park and VS-watchdog presses;
`idle_decel_kts_s` fitted for the other airframes; the speedbrake's own deceleration fitted
into the plan (`final_drag` is closed-loop on the idle prediction).

## Helicopters: the hover loop

Rotorcraft withhold every fixed-wing law (`tools::ROTORCRAFT_WITHHELD`) and fly their own loop
(`src/hover.rs`, blackbox `hover`), measured on the S-76C. Reuse aeroplane paths; branch only the
physical drive.

- **Cyclic to the autopilot, collective to us.** `servos_fdir_off` disconnects it, `servos_off`
  does not. The collective is `prop_ratio[0]`, written INDEXED.
- **The hover mode is a groundspeed HOLD that captures** (`sim/autopilot/hover`, fired only when
  `hover_status` reads off): the loop owns `hover_vector` (`DriftTrim`).
- **A moving target is air taxi, departure, circuit** (`slew_track`; goals `Hover`, `Land`,
  `Depart`, `Fly`); `DEPART_MAX_MPS` since the mode lets go near 47 kt. **Pedals are a yaw-RATE
  command with a heading hold** (`HeadingHold`), never damping on `R`.
- **The collective follows the power curve at the hover's LOADING** (`level_collective_at`,
  `PowerBudget`): a hover the lever cannot give is refused with its numbers; the trim is bounded to
  the curve (an unbounded integrator crashed one).
- **In the air the AP is never simply turned off** (`hand_back`). The start reads every throttle
  back at FLY and locks the tail wheel first. Airborne requests take over below
  `TAKEOVER_MAX_MPS` and `DEPART_MAX_FT`.
- **A hover lost up high is flown out, never handed back** (#282, `hover::at_height`, `let_go`):
  above `SET_DOWN_MAX_FT` no stop, set-down or hover-mode hand-back; into the wind
  (`fly_out_vector`) to the cruise modes' airspeed (`FlyOut`), handed to them WATCHED
  (`CruiseWatch`: sink, height, AP) and taken back if not held.
- **The final is capped by what the hover mode can hold**; **a transit field's challenge is
  answered with a transit request** (`heli_route` `zone_challenge`).
- **Circuit** (`src/heli_circuit.rs`) and **Route** (`src/heli_route.rs`: the whole flight, as
  full auto, `hover::release`, `SpeedLaw`, `sample_corridor`); **it works the radio with the
  aeroplanes' code** (`every_mission_that_leaves_a_field_works_the_radio`).
- **The approach comes down at speed** (`gate_agl_ft`); **the climb is checked at the arm**
  (`heli_route::climb_conflict`). **Below 50 kt collective is not a speed control** (`SlowLaw`): a
  slow-down that cannot make the pad goes around, never orbits (`slow_check`).
- No helipad: `runway_spot`. **Parking is the aeroplanes' tool** (`heli_route::park`); low, a
  helicopter is on the ground for the tower (`hover::tower_on_ground`).
- Words: "hover at N", "set it down", "hold position", "take off", "fly a circuit"; the AP's mode
  tools are refused while the loop flies (`hover::blocks`).

Still owed: issues labelled `area:helicopter` (`owed`: none;
`unflown`: approach and gate #47, fast climb #48).

## The flight model: one state, read by everyone

`src/flight.rs`: the ONE answer to "where are we and what do we hold". Pure, fed per monitor tick,
time from the sample so recorded flights replay (`flight::Replay`). Tracks `Phase` (sustained,
`losing_height`), `Control`, `Abnormal`, `Memory`, `Situation`.

- **A ground strike is not a landing**: an `Impact`; a wreck has no phases.
- It is READ: `monitor::coarse_phase`, the checklist tracker, cabin (`FlightFacts`), "how are we
  doing?" (`flight::status_line`), rapport's touchdown and bounce (`Rapport::tick`; an `Impact` is
  never praised).
- **The duty engine** (`src/duty.rs`, pure) says what the flight owes next; SHADOW only (blackbox
  `duty`) until its logs agree with flights flown.
- **The hand-over reads the model** (`src/handover.rs`): "you have control" resolves into the calls
  the phase needs or a spoken decline; the upset RECOVERY only for an actual upset or distress
  words. "Let me fly" is the release guard (`note_crew_takeover`).

## Checklist flows: the list at its phase, three flavours

`src/flow.rs` runs the airframe's XChecklist sections (`src/checklist.rs`); replies via
`flow::take_reply` (CLOSED vocabulary). Every `Item::Act` carries its pass test, so a runner reads
FIRST: in state is confirmed, never re-written; an item a loop owns (`held_by_automation`) is named
as held, never written (one writer per dataref).

- **Crew** (pilot flying, copilot acting): the spoken "descent checklist".
- **Monitor** (an automation flying): read-only and silent, ONE line at the end
  (`flow::summary_line`). The laws act; the list audits.
- **Instruct** ("talk me through", `instruct_request`): never writes; says, watches, moves on.

**A compound test is read whole** (`checklist::Expr`, `&&` over `||`): every term read, a term
that does not read is no verdict; an ACT only through its ONE settable term that maps, else a
read. History operators (`><`, `a:b`, `+>`) stay call-outs. A file the aircraft ships is
`checklist: acf:<file>`, resolved at detection (`checklist::resolve_acf_path`), never copied
here; a phase split into parts is ONE list (`checklist_flow:` key: [titles]).

**Who starts one**: `flow::Tracker`, one-shot per flight, every boundary SUSTAINED. Under an
automation Monitor starts at once (BEFORE LANDING only once configured,
`flow::landing_config_for`); with a pilot flying the copilot PROPOSES and never acts by itself;
the after-landing list runs once OFF the runway pavement. **A list belongs to the flight it began
in** (`flow::stand_down`).

## App modes & cabin crew

`mode.rs`: **both** (default) / **copilot** (no simulated ATC) / **atc** (the world only) /
**instructor** (hands off; every tool refused). `XCOPILOT_MODE` sets the BOOT value only. The
no-crew gate is the arbiter's enqueue door (`Voice::alert`); the atc crate never reads the mode.
**`mode::crew_actuates()` is the hands gate** (false in atc AND instructor): tool dispatch,
`fix_action`s, `cockpit_auto`, `flow::start`, undo. Entering instructor stands every loop down.

**Instructor** (`src/instructor.rs`): one ACS task at a time.
- The brief ASSIGNS the numbers, so the scorer never guesses. ACS tolerances per level
  (`Task::standards`); the taxi line is judged by the scorecard's `LineJudge`. An AP-shaped command
  in a lesson is a RETARGET (`take_target`), never a write.
- **"Teach me a full flight"** (`Syllabus`, behind `SyllabusIo`): the whole flight is a plain
  `cargo test`. `xfc debrief` prints lessons and `instructor::grade_flight`.
- **"Show me" is the one exception to folded arms** (`instructor::show_me`,
  `mode::tool_may_actuate`): the task's automation armed once through the tool door, narrated,
  graded as the instructor's own, then "Your controls."
- Still owed: #87 handbook questions (see docs/tasks/2026-09-28-instructor-handbook-questions.md:
  authored answers, then lexical retrieval, no model).

**Cabin crew** (`src/cabin.rs`): the second intercom persona.
- **A state machine, not a commentator**, with its own mute (`cabin_quiet`), NOT the monitor's
  `quiet` (which includes `autoland::ACTIVE`, so it missed its own landing).
- Addressed by "cabin crew"; "say again" over guessing; per airframe (`cabin_crew: true`); "cabin
  secure" is exempt from sterile cockpit.
- **Works the flight unasked** (`Cabin::watch`), each call once per flight, never after the flight
  deck made it. A bare "cabin crew?" opens the line (`take_open_line`); its voice is the second
  picker (`pollers::cabin_pick`).

## The copilot as a person: timing, persona, rapport

- **Timing** (`src/tempo.rs`): a jittered pause from the end of CAPTURE, "stand by" when nothing is
  queued (never over a radio call).
- **Persona** (`src/persona.rs`): `XCOPILOT_CREW_CHATTER` gates only what is VOLUNTEERED, never an
  answer or safety call; idiom follows the voice (`XCOPILOT_CREW_DIALECT`); small talk is CLOSED.
- **Rapport** (`src/rapport.rs`): praise goes to the pilot, never an automation; a bounce counts at
  the SECOND contact; offers are sentences, never actions.

## Panel, terminal, settings

- **The panel is the act-and-glance instrument**, fed by `xcopilot/status/auto`
  (`src/auto_status.rs`), change-gated with a periodic resend (plugin reloads zero buffers). The
  taxi turn card is `taxi_guide::TurnCard` on `xcopilot/status/taxi`.
- Buttons inject pilot English (`REQUEST_PHRASES`, grammar-test-pinned); CANCEL ALWAYS aborts
  something. The typed line is `xcopilot/typed` (`pollers::queue_command`); a readback wait gives
  way to it (`Voice::say_until`). Native panel-graphics API (SDK 4.4.0; `plugin/src/imgui_xp.cpp`).
- **The terminal is setup and diagnosis** (`src/ui.rs`, renders under `TestBackend`).
- **Settings are a registry** (`src/settings.rs`, `settings::live()`): env > pref > default; an
  env-pinned knob is REFUSED on screen, naming the variable. A panel toggle is a pick only when it
  differs from our LAST PUBLISH (`pollers::panel_sync`); else a terminal or EFB change is published.
- **Only settings persist** (`prefs.json`, env, geometry). Never a state dump: persisted facts
  outlive their context. The one other file is an airframe's MEASURED model
  (`profiles/learned/<acf>.json`, `aircraft::learned`): a property of the `.acf` like a profile
  line, written only by `xfc identify`, keyed to the `.acf`'s fingerprint (an edit retires it,
  said) and a schema version, reviewed and committed like a profile.
- **Every file root has ONE owner** (`sim::paths`, pinned by
  `every_file_root_goes_through_the_owner`): `resources()` ships (profiles, checklists, assets),
  `data()` is the user's (models, logs, prefs, `.env`, `profiles/learned`; learned READ there
  then shipped, WRITTEN there). Decided once, logged: `XCOPILOT_HOME` override, a dev checkout
  (the cwd, kept relative so every path is the string it was, or the binary's own checkout),
  installed (`~/Library/Application Support/XFlightCrew`, XDG on Linux). `xfc paths` says where.
- **One `xfc` file needs nothing beside it** (2026-10-13): the resources are built in
  (`crates/sim/build.rs`, a missing folder fails the build) and unpacked once to
  `data()/resources/<build id>/` only when no `profiles/*.yaml` is on disk; on-disk and the
  checkout always win. `xfc plugin status|install` (`src/plugin_install.rs`, the `embed-plugin`
  feature, `scripts/build-release.sh`): `xcopilot/` removed first, backups to
  `data()/plugin-backups/`, never inside `plugins/`; refused while X-Plane runs unless
  `--while-running`; it NEVER reloads plugins. `xfc licenses` prints the notices verbatim.
- **The home screen** (2026-10-14, `src/home.rs`, `xfc home`/`setup`, `--print` for support):
  checks with a fix each (X-Plane folder, ONE Web API GET on open or `c`, plugin, mic NAME,
  starter set), the voices screen, the dashboard's own Settings view (`ui::SettingsPane`),
  files; `r` boots in-process. Plain `xfc` shows it only on a TTY with a REQUIRED check red
  (folder, plugin missing, starter incomplete: file checks only); `xfc run` never. The starter
  set is the config's named single voices plus its whisper model (`voices::starter`, never a
  list of its own; `xfc voices install --starter`, `s`). The ATC pool draws INSTALLED voices
  only, a missing pick falls back once (`voice::installed_pool`, `loadable_station_model`).

## EFB: a page on the LAN (`src/efb/`, `docs/efb.md`)

- **No new actuation path**: a button or the typed box POSTs pilot English (`/say`) onto the
  command channel exactly like the panel's typed line (`pollers::queue_command`, blackbox `typed`
  `taken` `source: efb`); the grammar picks the tool, gates and readback included. Phrases live in
  `efb::state::PHRASES`, each pinned to its route (`every_button_routes_where_it_says`). That is
  for CREW actions; SIM MANAGEMENT (the Map tab's "Move here" to a stand) calls the owning tool's
  gated function directly through its own endpoint (`POST /reposition` ->
  `tools::setup::reposition_to_stand`: its gates, the mailbox, the readback), never a raw dataref
  write and never via an invented phrase (`no_phrase_moves_the_aircraft_to_a_stand`).
- **The Map tab's traffic and stands are the traffic world's** (`TrafficWorld::note_efb_traffic`,
  1 s while serving): free or taken is the fleet's own `Fleet::stand_holder`, plus our aircraft on
  one; no world simulating the field is UNKNOWN, never free (`airport::stand_states`).
  **A timetabled flight exists from blocks to blocks** (2026-10-13): beyond the simulation the
  pure whole-flight model places it by time (`traffic::track`, `src/traffic_whole.rs`), an
  arrival's model ENDING at the very aircraft its spawn makes (`Movement::entry_anchor`, entry
  toward its origin) and a departure's STARTING from the state it left the area in
  (`Fleet::take_left`), same id both sides: no flight appears or vanishes
  (`a_whole_flight_never_jumps_at_either_handover`). Drawn by the plugin only inside the draw
  ring, never a TCAS target for the station; on the map with its `journey`.
- **No new sim reads**: the page is ONE pure `state::build` over `Facts` gathered from what the app
  holds (flight model, owners, the shelf's plan, the follower, the party line, the voice's last
  controller line, the METAR file); `efb::feed` taps the speakers, the command loop and the monitor's
  tick as they already run. A fact nobody holds is a dash. The plan check and preflight are kept in
  flight-scoped cells (`feed::AUDIT`, `PREFLIGHT`).
- **Five tabs** (v3, `docs/tasks/efb-v3/mockup.html`): Flight, Map, Plan, Radio, Board, Settings behind
  the gear; MY CONTROLS, HOLD TO TALK and the phase's ONE suggested action (a `PHRASES` entry) in
  a fixed bar. **The Flight tab is laid out by the flight model's phase**
  (`efb::flight_tab::layout_for`, `each_phase_has_its_layout`): a readout with no source is
  dropped, never filled. The running checklist (`flow::sheet`) is a card on it in any phase.
- **The transcript is labelled FROM -> TO by one pure rule** (`efb::transcript::label`): our
  transmissions by OUR CALLSIGN ("by copilot" when the crew keyed), a controller to the callsign
  it opens with, the cabin address, the party line dimmed. The taps carry what it needs from
  their call sites (the voice's `overheard`, the drain's readback flag, COM1's position, the
  follower's LNAV via `feed::note_lateral`). The sim panel rows get the same label on
  `xcopilot/status/fromto` (`panel_label`), skipped for a plugin without it.
- Settings is the registry through `settings::set` (pinned and the EFB's own link read-only, mode
  and EFB-off confirmed, server-checked); alerts are tapped where the voice mirrors to the panel
  (`feed::note_alert`, flight-scoped; a warning banner acknowledged on the page only). **The
  banner and chip are HAZARDS** (`Voice::hazard` with a `monitor::Severity`: the monitor's
  checks, safe hold, recovery, unstable go-around, aborted start), never every line at
  `Priority::Warning`: a priority is how hard a line interrupts, not how bad it is. **The
  EFB's PTT is a heartbeat** (`efb::ptt`): held only while beats are fresh (600 ms), capped at
  60 s, OR'd into the intercom key.
- **The Map tab is ONE world map** (`map.js`, 2026-10-13): Web Mercator, north up, pan, pinch,
  wheel and double tap, from a region to a stand, FOLLOW on by default (a pan turns it off, its
  button back on). Every layer in one world frame: X-Plane's INSTALLED water (`efb::basemap`,
  `GET /basemap`, only the view's tiles, the page never asks wider than `MAX_TILES`), fields as
  dots then labels, the plan (`efb::map`, lat/lon; `map::fit` is the Overview button), our
  traffic and aircraft in lat/lon at every zoom, and closer in ANY field's diagram
  (`efb::airport`, apt.dat in a LOCAL metric frame, `GET /airport?icao=` built on demand, 202
  while building, cached, ETag'd), placed by ONE transform after its north is re-measured on
  Mercator once at load (`fieldFrame`, pinned under node by
  `the_page_places_a_field_where_the_server_measured_it`). The wanted field
  (`airport::wanted_field`) carries the taxi route, `TurnCard` and runway (`airport::page`);
  every known field's stands are `map.stands` (`airport::stands_by_field`).
- Off by default (`XCOPILOT_EFB`, `_PORT`, `_TOKEN`; the registry's `efb*`); every request needs
  the token, a wrong one is a 403 blackboxed `efb` `auth_failed`; one line per 500 ms. Its own
  thread and runtime, never the command loop's; no lock held across an await.

## Web API load is a sim cost

The API runs in X-Plane's flight loop: every request is frame time. `crates/sim/src/xplane.rs`
reports every 30 s (blackbox `api_load`). A 60 ms read cache (a write drops its entry);
near-static flags are `SLOW_DATAREFS`; a COMMAND clears it.
- **Nothing asks a loading sim**: `xplane::LoadGate` holds every request but the mailbox's while
  the plugin's `xcopilot/sim/loading` is set; fails open. Plugin work never runs in a Web API set
  handler: store, flag, act in a flight loop.

## Battle-test surface

`cargo test --workspace` (root `cargo test` is the crew package only). **Run tests with
`XCOPILOT_BLACKBOX=off`**, or they record into the live log.

**A fix merged offline gets an `unflown` issue** listing the blackbox fields to check; the sortie that
confirms it closes it (`fixes #N` in the commit). Known gaps are `owed` issues.

- `crates/atc/src/tower.rs` matrix; `crates/atc/bench/atc_corpus.yaml` (real whisper, **add every
  new wart**); `route_keyed`/`route_unkeyed` in `crates/atc/src/station/routing.rs`; STT eval
  `cargo run -- stt-eval bench/atc_stt_cases.yaml`; world sweep
  `cargo test --release -p atc -- --ignored atc_world_sweep`; ILS smoke
  `cargo test -- --ignored real_install_ils`; `docs/atc-chaos-card.md`.
- `xfc debrief` (`src/debrief.rs`) after every flight, before diagnosing by hand.
  A failed sortie: `xfc debrief --sortie <id|last>` (the moment it ended, the decisions before
  it, contradiction flags; `src/debrief/sortie.rs` `RULES`); one airframe's: `--airframe <id> [--since]`.
- **A new airframe or profile edit is checked parked first** (`xfc conform [--only <area>]`,
  `src/conform.rs`, `docs/conform.md`): every declared channel through its own seam, read back,
  put back; a row per channel and a blackbox `conform` record each. `--prepare` brings it up
  from cold first (battery, APU, preflight, start, IRS) as full auto's crew does.
- **A sortie is one file with a verdict** (`full_auto::SortieGuard`, `logs/sorties/<id>.jsonl`,
  `src/scorecard.rs` `sortie_result`): PASS needs no failure, give-up or crash, the ground at the
  end, and UNATTENDED (no `intervention`, no `tool` record). A complaint not yet a scorecard number
  is not yet testable.
- **A verdict that is wrong hides the real failures**: the crew's own yoke and brake are not a hand
  (`monitor::note_crew_yoke`, `monitor::CrewBrakeLatch`); a landing is scored at its FIRST contact.
  **A blackbox record is one write** (`blackbox::append_line`, file-locked: the runner is a second
  process). `bench/sorties/` pins real verdicts; `bench/replay/` replays real flights through a
  pure decision.
- **A night of landings is one command** (`xfc sortie bench/sorties/<spec>.yaml --repeat N`,
  `src/sortie_runner.rs`). **An airborne aircraft is never handed to nobody** (`hand_back`,
  `pause_safe`); a TAKEN line is never retyped; **no setup under a moving aircraft**
  (`SettleWatch`); **a reposition or swap is a new flight to the scorer** (`Scorer::new_flight`).
- **`--check` first, no sim** (`xfc sortie <spec> --check`): every typed line routed as the loop
  routes it, every record the runner reads pinned to its producer (`READ_SHAPES`).
- **An arrival case starts IN THE AIR on the plan** (`setup: {enroute: ...}`,
  `setup_enroute::place`), **ready only once settled** (`SettleJudge`).
- **Fast forward only where the laws tolerate it** (`SpeedGuard`): our ticks are wall time, so 4x
  is 4x the gain and delay; 1x outside cruise-like flight and around every setup.
- A law that misbehaves in the air while its test passes: suspect the plant. A first-order lag
  cannot ring; fly second-order plants through the real transport delay.

## Licensing

**GPL-3.0-or-later** (`LICENSE`, `[workspace.package]`): open source, never commercial, X-Plane
only, copyleft on purpose. SDK and Dear ImGui MIT/BSD, B612 OFL. Relicensing is free only until
outside contributions merge. `crates/atc` could become LGPL-3.0; deliberately not done.
