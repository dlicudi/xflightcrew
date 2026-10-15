# XFlightCrew

The rest of the crew, and the world around you, for X-Plane 12: copilot, ATC, AI traffic and
cabin crew, local and deterministic, with voice as the interface (formerly XVoiceLayer, binary
`xvl`). The pilot speaks, the grammar picks a tool, the agent writes the dataref, the copilot
reads it back aloud. **There is no model at runtime**: every miss in the logs becomes a rule.

```
🎙️  "Flaps up"   →  STT  →  grammar picks a tool  →  write dataref  →  TTS "Flaps up!"
```

**The evidence lives in [docs/dev-history.md](docs/dev-history.md)**: every sortie, date and number
behind these rules. Grep it BEFORE reworking any subsystem named here. This file keeps each rule,
its one-line why and its home in the code. Each section's fuller earlier text is in the history
under "<date> trim: <section>" (2026-09-26, 2026-10-06, 2026-10-15), so any airport, date, number
or identifier that left this file can still be grepped there.

## Direction (`docs/tasks/pillars-audit/REPORT.md`)

Three pillars: **copilot + checklists, ATC, AI traffic** (our own, replacing LiveTraffic; the
tower controls it). **The Cessna 172 is the reference airframe.** **Autonomy is redesigned in
layers, loop by loop** (`docs/tasks/autonomy-layers`): the aircraft's own autopilot flies where a
pilot would use it, and ONE pilot controller (`src/pilot`) hand-flies the rest, the same laws for
every fixed-wing airframe, every airframe number MEASURED in flight, never typed into a profile.
Laws are designed from first principles on plants FITTED to recorded flights, never tuned flight
by flight. Done: the final approach and landing. Every other flying loop (full_auto, circuit,
procedure, autotakeoff, autotaxi, taxi_in, pushback, heli_*, recover, reversal) is FROZEN until
its turn: crash and safety fixes only. "Every miss becomes a rule" is for the grammar, never a
flight law. Order: stage A (untangle copilot from autonomy, station behind `party_line`), B
(command loop), C (ATC per-flight strips, `crates/nav`), D (`crates/traffic`).

## How we work

**Keep it simple**: the smallest correct implementation that meets the requirements. Reuse
existing code and patterns. No speculative features, abstractions, frameworks or dependencies;
nothing the pilot did not ask for. A refactor reduces code and complexity. A brief for an agent
is held to the same rule.

**Roles**: the primary agent (Fable) designs, plans, briefs and reviews; the `implementer` agent
(Opus) codes, refactors, debugs and tests. Delegate substantial implementation, narrowly scoped;
do trivial changes directly; parallel agents only for genuinely independent work. Review every
result for correctness, unnecessary complexity, architectural consistency and scope; simplify
what overshot.

## Wire contracts (never rename)

- the `xcopilot/*` dataref namespace (app and plugin agree on the strings),
- the `XCOPILOT_*` env vars (the pilot's own `.env` and `prefs.json`),
- the plugin SIGNATURE `com.dlicudi.xcopilot`.

The plugin is `plugins/xflightcrew/mac_x64/xflightcrew.xpl` (source `plugin/src/xflightcrew.cpp`);
`make -C plugin install` removes the old `plugins/xcopilot/` folder FIRST (two copies register
every dataref twice). Package `xflightcrew`, binary `xfc`. The tracing filter names every crate
AND both crew names (`src/main.rs` `DEFAULT_LOG_FILTER`, pinned by
`the_log_filter_names_every_crate_and_both_crew_names`): drop either and half the log goes silent.

## Stack (decided)

Rust, a latency- and audio-sensitive real-time loop.

| Stage | Crate / approach |
|---|---|
| Mic capture | `cpal` |
| STT | `whisper-rs`, push-to-talk |
| Intent | deterministic: `src/router.rs` (command / question / follow-up / echo), `src/grammar.rs` (an ordered `MATCHERS` table over `src/grammar/*`, order is semantics), `src/conversation.rs`, `src/answers.rs`, `src/plan.rs` (questions), `src/dialogue.rs` (back-references); `main.rs` boot, the command loop in `src/main/` (`session.rs`), `src/detect.rs` airframe swap |
| X-Plane write | `reqwest` PATCH to the Web API |
| TTS readback | in-process piper VITS via `ort` |

- **No LLM, local or cloud**: `xfc analyze` lists the top missed phrasings; each becomes a rule
  with the live transcript as its test (`--now --since <date>`). A model returns only as the
  instructor answering handbook questions, never a fallback.
- **REST for transport, plugin for capability.** Stock datarefs over the Web API; the C++ plugin
  (`plugin/src/`) adds `xcopilot/*` datarefs over the same API. The app never links the SDK.
- **Voices can ride the sim's audio** (`plugin/src/audio.cpp`, `XCOPILOT_AUDIO_BUS`, off by
  default): `xcopilot/audio/cut`'s ONE writer is `tts::play_on_bus`; a failed post plays locally.
- **Voices and speech models are installed by `xfc voices`** (`src/voices.rs`): on demand only,
  never at boot or in the command loop; every download size- and checksum-verified, an in-use
  model never removed.

## Core design rule: curated tools, never raw dataref access

**Nothing writes arbitrary datarefs; the grammar picks curated tools.** Each owns (1) its dataref
and range, (2) a **readback** re-read AFTER writing and compared with the INTENT (a silently
failed write must not answer "Roger"), (3) **safety gates** ("gear up" on the ground refuses and
explains).

- **Back-references** ("a bit more", "do that again") resolve against the calls actually executed
  (`src/dialogue.rs`): CLOSED vocabulary, 120 s TTL, automations never re-armed, cleared on swap;
  they ride the tool path, gates included.
- **State questions** answer from the SAME datarefs the tools write; the question OPENER routes
  them. A judged speed question gets a verdict (`speed_verdict`), never a bare number.
- Weather away from the aircraft comes from the METAR file (`src/weather.rs`), never the air we
  are in. A named fix is matched fuzzily against the LOADED plan only (`fix_query`).

## Aircraft profiles drive the tool surface

Profiles (`src/aircraft.rs`, `profiles/<id>.yaml`, `XCOPILOT_AIRCRAFT=auto`) are detected from
the `match:` block (`acf_ICAO` AND the `.acf` path, `detect::resolve`), never the ICAO alone (the
stock 737 and the Zibo both say B738). An unmatched model of a known type is generic, said aloud;
an unknown type is never silently the 172 (`aircraft::generic_from_acf`). Profiles declare systems
and limits; datarefs and commands stay in the Rust tools.

- **Numbers are the airframe's**, the old GA figure when silent. "Enough runway" is ONE rule
  (`aircraft::takeoff_needed_m`, `landing_needed_m`), a failed fit SPOKEN by its gate. No
  automation target below `speed_floor_kts` nor above `speed_ceiling_kts`; a slatted wing's
  stall is per notch (`stall_speed_in_config`). `vyse_kts`, `vmca_kts` stay `None` on singles:
  say nothing rather than invent. Multi-engine tools take `engines`, read back the LAST element.
- **One home each**: who flies `flight_state::Owner`; stopped `sim::ground`; angles `sim::angles`;
  AP engage `ap_engage::ApEngage`; speed ceiling `ceiling_from_vne`.
- **The autopilot panel has ONE owner** (`src/autoflight.rs`,
  `every_mcp_write_goes_through_the_owner`). Modes' truth is the `autopilot_state` bitfield or a
  declared MCP's own lamps (`lamps:`, `windows:`, `read_status`). A loop states INTENT (`Want`);
  `reconcile` presses on a difference only, never over a capture, nothing on a failed read; the
  window is parked where no capture takes the wheel (`WindowRule`); `ours_since` answers "was
  this change ours?". **The thrust has one owner**: a loop flying the levers takes an ENGAGED
  A/T out to ARM (`ThrustWatch`); a lever-driven A/THR flies the speed from the detent
  (`athr_cl_detent`, `speedhold::athr_tick`), our lever again only once it drops out or strays
  (`manual_lever_cap`). A speed hold on the runway has landed (`touchdown_idle`).
- **A flow runs at crew pace and the voice says milestones** (`jet_start::AirlinerStart::run_steps`):
  fire, READ BACK, a beat; a switch that never took named at the end.
- **A profile line is a hypothesis until probed**: `aircraft::profile_references` walks every
  name, `dataref_exists` / `command_exists` ask the sim, `probe_summary` speaks the missing, never
  into a load. Rust constants too: read names come from `datarefs::*`, `cargo test -- --ignored
  real_install_names` checks every `"sim/..."` literal; the plugin side too
  (`health::PLUGIN_DATAREFS`). Editing `plugin/src` means `make -C plugin`, install AND reload.
- **A failed read is NO measurement**: skip the tick, blackbox `null`. Position is one frame
  (`sim::position::read`). A "dataref" may be a command; read-only switches need read-before-fire
  `*_toggle` tools; a read-only KNOB is stepped by its commands (`radio::step_knob`). A stock
  dataref can be an add-on's OUTPUT: when a tool "does nothing", read the Lua; declared knobs are
  CONFIRMED from the box's output through one writer (`radio::set_mode`).
- **Selectors are republished on EVERY swap, including to `None`**: facts never outlive their
  airframe. `src/flight_state.rs` is the one page (`publish_airframe`; `stand_down` cancels every
  run in `LOOPS`); a new flight or airframe goes through `flight_begins` / `airframe_begins`; a
  loop OWNS its flag through its `Slot` lease. Every static names its lifetime (`src/lifetimes.rs`).
- **Systems tools read first, read back, and never touch what is undeclared or not theirs**:
  anti-ice (`src/tools/anti_ice.rs`), ground power (a cart and a contactor are two things,
  `gpu_on`), cabin air (`cabin_air.rs`: SUPPLY on for every mission; the masks are not the
  supply, `MaskWatch`, never over the pilot), centre tank pumps (`fuel_pumps.rs`: off when empty
  under an automation, advised to a pilot, a main tank never), seat belt signs (`signs.rs`).
- **A crew configures before it moves** (`src/takeoff_config.rs`): flaps, trim, altitude dial (a
  pilot's dial stays), spoilers, autobrake; a Boeing declares `takeoff_speedbrake_down`; thrust
  against a set parking brake is idled first.
- **Unasked actions** (`src/cockpit_auto.rs`): a state machine over conditions, one-shot per
  spell, never re-asserted; INTERIOR lights only, "off" only for lights it lit. The DG auto-sync
  is said only when the drift READ BACK fell (`monitor::GyroSync`).

## SimBrief

`src/simbrief.rs`, `XCOPILOT_SIMBRIEF_ID`: on demand, never realtime.

- **FMS injection is plugin-only**: `xcopilot/fms/route` then `xcopilot/fms/load`; the plugin
  loads the plan WHOLE in its flight loop (`FmsLoop`). **The plan is the WHOLE navlog**
  (`fms_route_payload`), thinned from the MIDDLE.
- **The follower sequences the FMS** via `xcopilot/fms/goto` from its OWN active leg. Where the
  airframe declares LNAV (`lnav_command`/`lnav_status`) plain legs fly on it once the FMS's active
  ident IS the leg, the bug the watched fallback (`src/lnav.rs`); elsewhere the bug, never GPS.
- **Local planner** (`src/flightplan.rs`, `src/route_finder.rs`; "plan a flight to X"): the same
  `FlightPlan` (`PlanSource::Local`), airways or direct round restricted volumes; a short
  destination runway REFUSED; only an explicit refresh replaces it.
- **"Fly the flight plan" is preflighted AT THE ARM** (`full_auto::preflight`): a refusal names
  the number; an unknown is skipped, never guessed. On the ground the arm fetches the plan fresh
  (`ArmFetch`) and loads only what is not aboard (`arm_load`). **A MARGIN is the pilot's to
  waive, the airframe never** (`full_auto::Gate`): "fly it anyway" waives only the runway factor
  and the plan's weight limit (`grammar::waiver_request`); fuel, ceiling, destination runway never.
- **A spoken ask for the plan re-fetches EVERY time** (`PlanFetch::Asked`). "Load it" is a separate
  word; a plan on the shelf authorises nothing.
- **The load sheet** (`src/tools/loading.rs`, `load_aircraft`): ground and stopped; fuel wings
  first, refused past the limits with both numbers, read back. "Refuel" is the fuel truck.

## ATC: the world, not the crew

An in-process station (`crates/atc`): one mic, one PTT, one speech arbiter. The tower hears an
utterance iff COM1 is on the station AND it parses as a radio call (FSM in
`crates/atc/src/tower.rs`); else the copilot. Frequencies from apt.dat, never hardcoded. Config:
`XCOPILOT_ATC_AIRPORT` / `_VOICE` / `_RATE` / `_REGION`.

**Hard boundaries**
- The station returns text and can NEVER actuate (`sim::xplane::SimView`, read-only); no
  whisper/cpal/ort in its deps (`cargo tree -p atc`). **Nav data is its own crate** (`crates/nav`,
  pure data, re-exported as `atc::nav`); a traffic crate takes `nav` without the station.
- **The crew/world seam is one page**, `crates/atc/src/party_line.rs`: statics crate-private, a
  new crew need gets an ACCESSOR, never a `pub static`; `station` is `pub(crate)`
  (`the_station_is_reached_only_through_the_seam`).
- **One strip per flight** (`party_line::FlightStrip`): `OWN_STRIP` ours, `STRIPS` the traffic's
  (cleared by install/uninstall only); the desk half (`StripDesk`) per flight, a traffic desk
  swapped in only for its synchronous answer (`Station::respond_for`). `FlightId::Own` is never
  keyed by callsign. A new flight of ours at the field already up resets OUR part in place
  (`Station::new_flight_here`), never a reinstall (a rebuild despawns every moving aircraft).
- **The runway book and the landing sequence are COMPUTED, never stored** (`station::RunwayBook`,
  `station::landing_sequence` over `party_line::RunwayTimes`): every strip's claim from its
  clearance and position, every strip's time to the threshold; `own_number`, `own_leader`. An
  order once SPOKEN to us holds unless the gap reverses by more than `SEQUENCE_REVERSE_S`.
- Tests touching a process-wide static hold `SIDE_STATE_LOCK` (`strip_tests::ClearStrips`);
  setting the mode, `mode::TEST_LOCK`. `Station::install()` publishes side state; `uninstall()`
  retires it all.
- **A mishear may earn "say again", never a clearance.** Only mayday/pan-pan lands without a final.
- **One callsign for tower and crew** (`atc::callsign::resolve`): env > plan > tail, never stored.

**Tower behaviours**
- Readbacks enforced for runway-critical items (`XCOPILOT_ATC_READBACK=false` coaches only); a
  speed and a frequency are readback items too (`judge_speed_readback`, `frequency_readback`). A
  REPORT spends nothing (`Intent::RunwayVacated`). A departure check-in is answered with its
  climb (`climb_announced`), no higher than the FILED cruise (`capped_departure_climb`).
- **IFR clearance** (`Intent::RequestIfrClearance`): the SID JOINS THE FILED ROUTE
  (`nav::sid_for_filed_route`), never invented; amended by name (`Station::amend_sid`); seen by
  the crew only once accepted (`party_line::assigned_clearance`); `Station::cancel_ifr` retires
  it all. "Radar contact" is the radar unit's, on its own frequency (`squawk_tick`). At or above
  the transition level a level is a flight level (`tower::climb_item`).
- **Runway in use** follows the live stream in calm (`field_streams`, `runway_follows_stream`),
  the wind picking a direction and the longest parallel the strip (`best_runway_for_wind`); a
  turn of our runway is SAID, FROZEN while committed, a stated runway standing when the wind
  does not argue (`stated_runway_stands`). The party line clears nobody against our runway
  (`own_strip_claim`).
- **One departure on the runway at a time** (`WAKE_SECS`); a takeoff clearance's claim ends when
  the aircraft flies, ours too (`RunwayBook::add_strip`). Release threads the arrival stream
  (`DEPART_GAP_NM`): a strip cleared to land and still flying is an ARRIVAL, never "traffic on
  the runway" (`RunwayBook::departure_blocked`), and a departure goes ahead of it only if the
  lander's own go-around watch would never revoke it, ONE rule for reply and release
  (`DepartureBlock::holds`, in time, never distance). A hold keeps its word
  (`TakeoffHold.held_for`). A reject is a radio call (`Intent::RejectingTakeoff`): the
  clearance and all it armed die.
- **A landing is never cleared onto an occupied runway, and the go-around is judged on time**
  (`LANDING_HOLD`, `landing_release_due`, `go_around_due`, `strip_exit`); an occupant cleared
  for takeoff is a DEPARTURE, judged on its roll; an occupant is the PAVEMENT's
  (`on_runway_pavement`); traffic that will not leave re-sequences (`resequence_runway`). The
  runway watch ticks whoever flies the final (`go_around_tick`, `landing_hold_tick`); the crew
  never lands uncleared (`UnclearedWatch`). A final report behind traffic landing ahead is
  number two, never a clearance the tower must cancel (`station::landing_leader_blocking`).
- **Circuit traffic stays with the tower** (`tower::wants_circuits`, `circuit_clearance_lines`).
- **Our arrival is in the traffic's slot picture and the traffic yields to it**
  (`traffic::slot::own_slot`, `traffic_link::own_arrival`; the tower naming US as a leader is
  `traffic::fleet::OWN_ID`). **Approach says our number and the traffic we follow ONCE per
  change, and controls our speed on the traffic's own bands** (`station::approach_sequence_lines`,
  `APPROACH_SPEED_BANDS` pinned to `traffic::arrival::SPEED_BANDS`), a band only where our type
  can fly it (`provide_approach_speeds`); the speed is void at the tower handoff and a go-around.
- **The traffic world runs on UTC, with or without the sim** (`traffic_link::utc_now_s`): our
  aircraft is an optional input, the plugin and the voices wait for the sim, and a sim clock off
  UTC is said once, never followed (`TrafficWorld::check_sim_clock`).
- **Taxi routes**: Dijkstra over apt.dat 1201/1202; crossings from strip GEOMETRY, never network
  flags; a runway destination snaps to a node ON THE DESTINATION STRIP; a refusing gate names
  strip and distance (`NoFit::NoneNear`).
- ILS from earth_nav.dat + CIFP; dialect by ICAO prefix (FAA vs ICAO/CAP 413); `qnh_clause` is
  the one pressure seam, `suggest_call` the one source for coach and panel; the tower's wind is
  the FIELD's (`station::set_surface_wind`).

**Audio and PTT**
- TWO half-duplex channels, intercom and radio: a routine intercom line is cut when the radio keys
  and resumes once quiet (`interrupt_intercom_for_radio`); a Warning talks over and is never
  stepped on; radio-only cuts never touch the copilot. ONE frequency: speech renders as whoever
  COM1 is on (`tuned_speaker`); the rest is the void.
- **A crew call is routed on the frequency it was keyed on, before it is spoken**; with a radio
  key bound, physical routing beats content routing both ways. A position report dies with the
  position it reports (`crew_radio::holds_for`).
- **Any PTT mutes everything while held**; silenced lines are OWED (`voice::SpeechMemory`,
  `router::recall_line`); "say again" on the frequency repeats the controller verbatim
  (`voice::controller_repeat`). The command loop never holds a pilot's line behind its own
  speech (`pilot_line_pending`), nor an armed station swap (`loop_owed`).
- **A station hears only its own field on the ground** (`GROUND_REACH_NM`): beyond it full auto
  departs UNCONTROLLED (`party_line::plan_self_taxi`); airborne the last controller keeps it.
- The radio needs POWER both ways. **FAIL OPEN, the flag RESTORED every tick**: a fact that merely
  stops being published latches across a swap. `take_crew_radio` drains unconditionally.
- REPLAY gates the reactive surface; PAUSE freezes ticks, not audio; `xplane::sim_frozen` covers
  both, failing open. ONNX session CREATION is serialised (`tts_onnx::build_session`). Barge-in
  kills by REGISTERED pid + per-channel cut generation, never pkill. `Voice::alert` mirrors to
  the panel itself: mirror OR `s.action(...)`, never both.

## Ground & flight automation

Orchestrators compose proven loops; no new flight laws in them. Laws test as plain `cargo test`;
everything blackboxes to `logs/blackbox.jsonl`. A record's kind is the first argument of
`blackbox::record`, listed by `rg -oNU --no-filename 'record\(\s*"(\w+)"' -r '$1' src crates | sort -u`.
Known gaps are issues labelled `owed` with their `area:` label.

### Holding, writers, reads
- **An airborne aircraft NOBODY flies is held safe** (`src/safe_hold.rs`, its own thread, never
  stood down): the autopilot's path must end above the ground ahead plus 1,000 ft, else a climb
  or bounded descent through the panel owner, else `recover::take_over`; the ESCAPE
  (`escape_due`) acts at once. Never judged on an aligned final.
- **A stage not moving the aircraft is holding it**: brakes for every wait, abort and give-up. A
  mission that ENDS on the ground stops and parks (`tools::controls::hand_back_hold`), the park
  brake confirmed and said.
- **One writer per dataref.** Plugin channels (`taxi_steer`, `taxi_brake`, `pitch`) are 1 s
  expiring heartbeats written EVERY tick; yield latches (`steer_yield`, `pitch_yield`) drop the
  override that frame. Park brake through the valve-aware seam (`ensure_park_brake_valve` /
  `release_park_brake_for_roll`), never a raw write.
- Batch a tick's reads in one `tokio::join!`; check `dt` before gains. Groundspeed and route
  distance are magnitudes; directional values keep their sign (`taxi_guide::project_signed`).
- **Readbacks and echoes never arm automations**: arming grammars take request shapes only
  (`looks_like_command`). "stop" routes by context.

### Taxi, push, line-up
- **Auto taxi** (`src/autotaxi.rs`): speed via `hold_speed`; pure pursuit; an off-route engage is
  a JOIN (`MAX_JOIN_M`); a lost governor on the ground ARRESTS; a drive is judged by the ground it
  makes, never a flat clock (`ProgressWatch`); a jump in a projection is never a movement
  (`join_fix`, `relocation_jump_m`). Castoring nosewheels brake-steer through the PLUGIN channels
  only (the plugin holds `override_toe_brakes` while steering; a pilot brake latches
  `xcopilot/brake_yield`), the rollout too (`autoland::RolloutBrakeSteer`). Never "fix" a corner
  that stops the aircraft by tuning the governor.
- **Routes ride the paint** (`ride_paint`): 1201/1202 topology, drawn centrelines geometry; thin
  data degrades, never refuses. A hold is reached ON the line (`arrived_on_line`); line up rides
  the DRAWN lead-on (`TaxiRoutePlan.lead_on`), crosschecked against `party_line::clearance_runway`;
  an arrival's route is never a departure's clearance (`taxi_to_stand`).
- **Traffic ahead is stopped behind its TAIL, planned for the governor's lag**
  (`traffic_guard::traffic_stop_in_m`, `GOVERNOR_LAG_S`). **The world never favours our
  aircraft**: no silent right of way; who goes first is Ground's to say, to us too; our route
  (`autotaxi::route_ahead`) serves only the deadlock breaker (`STUCK_S`), and we are never towed.
- **Pushback** (`src/pushback.rs`): tug commands are WHOLE manoeuvres, sent ONCE, never steered;
  we stop the tug when we are there (`PushJudge`); any command to a tug ON THE BAR drops it off
  and the bar coming off ENDS the push; a tug only ON A STAND; a light aircraft turns itself off
  the stand (`tug_for_weight`).
- **Taxi guide** (`src/taxi_guide.rs`): never two voices on one mistake; gates on
  `pushback::ACTIVE`; "which way?" is question-shaped.

### Takeoff
- **Auto takeoff** (`src/autotakeoff.rs`): refuses without a clearance when a tower runs;
  `RollSteer` cuts to the centreline for a CLOSURE TIME (`roll_target_heading`, `DriftFilter`),
  the declared rudder believed only to its measured speed (`roll_rudder:` `at_kts`); stagnation
  reject on GROUNDSPEED; a reject from any speed is FLOWN to a full stop, parked, then handed back
  (`reject_rolling`); rotation on `xcopilot/pitch`; the AP takes the ESTABLISHED climb, the VS
  dial written AFTER engage.
- **A takeoff starts with the autopilot OFF and an engage reads the SERVOS**
  (`clear_inherited_autopilot`, `servos_on`). **The autopilot's ANSWER is watched, never assumed**
  (`src/climb_watch.rs`): a runaway climb is TAKEN and the AP returned only once WATCHED holding;
  every vertical engage proves its pitch AUTHORITY (`AuthorityCheck`), else hand flown. A pattern
  that is no pattern is given back loudly (`circuit::PatternGuard`), but never to nobody
  (`recover::take_over`); a recovery never writes into the next flight (`recover::may_write`).
- **The handoff line is true** (`handoff_to_caller`) and a pilot handoff is FLYABLE
  (`handoff_waits_for_clean_up`). Flap schedules fit their placards (test-enforced).
- **A jet's climb-out is declared** (`autotakeoff::ClimbOut`, each flap step once the last
  ARRIVED; V2 + 15 by pitch, `ClimbHold`; the AP takes the climb being flown, `ClimbVs`) and its
  autoflight configured before the roll (`takeoff_config:`, the A/T armed never engaged, thrust
  reduced at `thrust_reduction_agl_ft`; `before_takeoff_audit`).
- **A runway start is a lined-up aircraft and still calls** (`tower::Ctx::own_on_runway`); too
  little ahead or the reciprocal is a BACKTRACK (`full_auto::RunwayStart`); another runway is
  taxied to (`cleared_elsewhere`).

### Circuit, procedure, full auto
(`src/circuit.rs`, `src/procedure.rs`, `src/full_auto.rs`)
- **The pattern is judged on the CORRIDOR it is flown in** (`choose_pattern`, `corridor_frame`):
  sized to the AP's real turn radius, terrain and airspace, raised, steepened, shortened, else
  REFUSED with the numbers. Throttle laws watch a speed PROJECTION (`speed_governor:`); ONE speed
  per leg (`leg_want_kts`). Final uses along-track `path_dist_m`, a REAL intercept
  (`final_intercept_hdg`) sized on the bank flown (`BankWatch`), autoland only ON THE LINE
  (`ready_for_autoland`), the glidepath fed forward (`path_descent_fpm`).
- **The sim's heading mode never reverses a turn in progress**: every steer goes through
  `heading_bug::bug_toward` (`every_heading_bug_write_is_turn_safe`). The AP's own capture holds
  level legs where declared (`ap_altitude_capture`).
- **Full auto**: engine start per class (`StartKind`), power-up INLINE, a declared preflight
  between the battery and the ramp (`src/preflight.rs`), an inline start's reply its verdict;
  waits on `driving()`; ends with `taxi_in_after_landing`. It flies the checklist items it owns
  (`engine::arm_mixture_manager`); lists only cross-check. Exterior lights follow the practice
  (`tools::lights::Exterior`).
- **Airborne, "fly the flight plan" and "land the aircraft" are ONE planner, from ANY state**
  (`src/recovery_plan.rs`, `choose_join`): refused only for the physically impossible, with the
  number; the nearest runway only when asked, an engine out, or no plan.
- **The whole plan is dry-run first** (`src/plan_audit.rs`): REFUSED or CAUTIONED by name; the
  all-clear names what could not be checked. **The crew picks its own procedures**
  (`full_auto::choose_departure`, `plan_audit::choose_arrival`); a departure other than the
  cleared SID is ASKED for (`request_sid`); a spliced SID flies the PLAN's levels. The arrival is
  briefed at the arm (`brief_arrival`) and re-picked in flight (`RepickWatch`).
- **Charts are flown**: SID minimums (`departure_target_ft`), STAR windows
  (`apply_arrival_constraints`) and speeds (`arrival_speed_cap`), a route-only arrival's gateway
  on the field's 3 degree path (`arrival_gateway_alt_ft`); the VS wheel only in VS mode
  (`vs_action`); the 250 below 10,000 cap, once begun, holds (`speedhold::AirspaceLimit`).
- **Cruise is a SPEED** (`scheduled_cruise_kts`); a planned step is flown at its fix
  (`apply_step_climbs`); the next fix is `active_leg()` first; only `planned_tod` is announced,
  and while a follower flies the plan's ring is silence. A descent that outruns the governor
  gets drag, then a shallower rate (`descent_drag`), never with the gear down or flap past the
  first detent.
- **The crew works the radio as it flies**: a readback goes out once
  (`crew_radio::transmit_readback`), heard, read back, then rolled (`readback_before_moving`);
  `take_departure_handoff()`, `take_region_handoff()`; an IFR flight keeps its code
  (`IFR_FLIGHT`). The inbound call goes to the destination's OWN tower (`inbound_due`;
  `call_inbound` tunes COM1 and arms that station); an instrument arrival works APPROACH first
  where the field has one (`arrival_radio`, `Station::clear_instrument_approach`,
  `ApproachWatch`); "final" is reported ESTABLISHED on it (`autoland::final_report_due`).

### The follower's vertical and the altimeter
- **Three owners in order** (`procedure::vertical_tick`, pure): the PLAN's constraint, paced from
  the last COMMANDED altitude; the PILOT's altitude (`pilot_altitude::take`), never written to
  the dial; the GROUND AHEAD over both. A descent never climbs back (`descent_no_climb`), nor a
  climb descends.
- **Terrain floor** (`terrain::floor_ahead_ft`): a climb into it RAISED and said, a descent below
  it refused. The follower flies the ALTIMETER (`altitude_ft_pilot`); only the floor is true feet.
- **So the altimeter is kept right**: ONE rule (`instruments::correct_setting_inhg`); the
  monitor's `AltimeterSetting` resets it under an automation, advises a pilot; `set_altimeter`
  steps in rounds EARNED by progress. The transition is the FLIGHT's (`instruments::FlightTransition`).

### Approach and autoland
- **Every planned arrival hands its runway down** (`EndAction::Approach { rwy_id }`); the
  self-pick (`autoland::self_pick`) keeps to the plan's destination; **the tower's clearance
  outranks the plan** (`runway_from_clearance`, `retarget_arrival_runway`), watched THROUGH the
  approach. Arrival wind is the destination METAR.
- **A new flight takes its clearances with it** (`party_line::retire_for_new_flight`); its station
  is the field under the NEW, SETTLED position (`station::arm_new_flight_at`, `setup::SettleJudge`);
  nothing arms while it loads.
- **A straight-in to THIS runway or none** (`nav::straight_in_legs`, `interceptable`); a final is
  judged against the pavement it lands on (`lands_on_the_pavement`), else it balks with height in
  hand. An RF leg is flown round its arc, a hold as a racetrack, a CF leg on its course
  (`course_leg`, `HoldFlight`); an IF behind us is joined by autoland's pattern (`FinalJoin`).
- **A final asks for descent OR deceleration, planned back from the gate**
  (`src/approach_energy.rs`, `idle_decel_kts_s`), never accelerating; beyond the FAP the descent
  is the final's (`established_on_final_path`); the FAF is crossed at the plan's join speed and
  flap (`faf_speed_cap`, `faf_flap`). A descent flies its speed first, at idle
  (`speed_protected_vs`); the speedbrake is for HEIGHT (`final_drag`, declared
  `speedbrake_max_ratio` only); TECS bounds a FAST error only by idle.
- **One stabilised-approach rule for every final** (`src/stable_approach.rs`): a breach held 3 s
  goes around; the missed approach flies `missed_approach_speed_kts`.
- **The autopilot is engaged and CONFIRMED, never assumed** (`ApEngage`), else handed over after
  10 s; the VS watchdog excuses a slow autopilot, never a dead one (`vs_not_held`). A mountain
  field is a flyable approach or a hand-over (`mountain_field`); a MAP on the runway fix is a
  landing.
- **The ILS is tuned, watched and flown coupled wherever the airframe's autopilot has an
  approach mode** (`src/ils.rs`, `coupled_allowed`; a positive `vdef` is ABOVE the beam). Where
  the airframe declares it, **its autopilot LANDS and the crew monitors** (`autopilot_lands:`,
  `src/ap_lands.rs`): the captures the FMA's, its flare, retard and rollout to `disconnect_kts`;
  the gates and the go-around door stay ours; not dual, not armed or lost high: our law or a
  go-around; lost low: our flare. "Set up the ILS" is a pilot monitoring's setup
  (`src/tools/approach.rs`, `SetupWatch`): APPR armed only from OFF, read back, never retried. The
  copilot knows the systems before reaching for one (`src/systems.rs`); the AP's own modes are
  tools (`tools::autoflight::ap_mode`), toggles read before firing.
- **A reposition is a PATTERN, never a point** (`src/reversal.rs`), only when a re-intercept is
  not flyable (`straight_in_losable`); its side, platform and no-go airspace are the circuit's
  rule (`reversal::plan`, `side_airspace`); the downwind is never EXTENDED into airspace
  (`AirspaceLimit`); a refusal climbs and hands over level (`TerrainHold`).
- **An approach never takes away the flap the wing flies on** (`flap_kept_flying`); a climb is
  flown on thrust (`climb_power_floor`); a give-up hands the SPEED to the governor
  (`hand_back_speed_kts`) and a FLYABLE aircraft.
- **The final is flown as a pilot flies it**: ABOVE MINIMUMS the autopilot (coupled, else HDG by
  `FinalHeading`, whole degrees, no oftener than every 10 s); FROM MINIMUMS (`arrest::minimums_ft`)
  the servos come out (`arrest_hands_step`, and stay out: `hold_roundout`) and `src/pilot`
  hand-flies to the wheels: bank to the PAINTED centreline (`pilot::line`, NPFG), the rudder
  coordinating then DECRABBING (`pilot::yaw`), the attitude law for the path then the flare
  (`pilot::attitude`), power on TECS (`pilot::energy`), the AP off at the wheels. Nothing writes
  the heading bug below minimums. The crab is LOW-PASSED (`CrabFilter`); the glidepath aims at the
  pavement (`aim_elevation_ft`).
- **The pilot flies on MEASURED responses, never typed gains**: each axis' response
  (`aircraft::learned::Response`) is an identification's (`xfc identify`, `profiles/learned/`,
  keyed by the `.acf`'s fingerprint and a schema version), else the fleet's prior; every loop's
  gains by ONE rule (`pilot::response::design`, McRuer's crossover); the roll refined in flight
  (`RollResponse`); TECS's `K_thr2STE` and spool measured or the book's; the NOSE limited by the
  `.acf`'s geometry. A profile never declares `flare_elevator`, `flare_power`, `arrest_lead` or
  `final_track` (`no_profile_types_the_pilots_numbers`).
- **The flare is the attitude law**, one law for every fixed-wing airframe: the nose angle for
  the path from a gate of the sink, a smoothstep to a hold-off, a disturbance observer for the
  elevator; its path trim never winds below zero. Its power is TECS's: height first, the lever
  only back, idle as the wheels the flare PLANS arrive (`attitude::planned_s`). Tested on plants
  FITTED to recorded flights (`pilot::plants_tests`), never an invented plant.
- **A low flare with room to stop is COMMITTED** (`commit_flare`, `commit_flare_latched`); the
  go/no-go is decided at the roundout gate (`landing_call`, `landing_roll_ft`): the predicted
  touchdown must leave the ROLL, else "Long landing. Going around." A low balk is called on the
  radio only once it CLIMBS (`BalkWatch`).
- **A tower's go-around order outranks everything, and reaches whoever flies the final**
  (`party_line::GoAroundOrders`); the pilot's go-around is flown the same way (`src/go_around.rs`),
  a slow, low one pitching for speed (`go_around_vs_fpm`); `MissedStage` is sticky.

### Taxi in
(`src/taxi_in.rs`) VACATE first, then "runway vacated", THEN the request. A vacate turns toward
the parking and never onto another runway (`nearest_stand`); a stand route asked from the runway
starts with the vacate (`station::stand_route_on`). A mission ends shut down
(`shut_down_at_stand`); a spoken "taxi to parking" leaves the engines to the pilot. "My controls"
ends the taxi-in itself (`taxi_in::abort`), braked and parked; a stand-down only lets go.

## Helicopters: the hover loop

Rotorcraft withhold every fixed-wing law (`tools::ROTORCRAFT_WITHHELD`) and fly their own loop
(`src/hover.rs`, blackbox `hover`), measured on the S-76C. Reuse aeroplane paths; branch only the
physical drive.

- **Cyclic to the autopilot, collective to us.** `servos_fdir_off` disconnects it, `servos_off`
  does not. The collective is `prop_ratio[0]`, written INDEXED.
- **The hover mode is a groundspeed HOLD that captures** (`sim/autopilot/hover`, fired only when
  `hover_status` reads off): the loop owns `hover_vector` (`DriftTrim`). A moving target is air
  taxi, departure, circuit (`slew_track`; `DEPART_MAX_MPS` since the mode lets go near 47 kt).
  Pedals are a yaw-RATE command with a heading hold (`HeadingHold`).
- **The collective follows the power curve at the hover's LOADING** (`level_collective_at`,
  `PowerBudget`): a hover the lever cannot give is refused with its numbers; the trim is bounded
  to the curve.
- **In the air the AP is never simply turned off** (`hand_back`). Airborne requests take over
  below `TAKEOVER_MAX_MPS` and `DEPART_MAX_FT`. A hover lost up high is flown out, never handed
  back (`hover::at_height`, `FlyOut`, `CruiseWatch`).
- **Circuit** (`src/heli_circuit.rs`) and **Route** (`src/heli_route.rs`: the whole flight, as
  full auto, working the radio with the aeroplanes' code). The approach comes down at speed
  (`gate_agl_ft`); the climb is checked at the arm (`climb_conflict`). Below 50 kt collective is
  not a speed control (`SlowLaw`): a slow-down that cannot make the pad goes around.
- No helipad: `runway_spot`. Parking is the aeroplanes' tool (`heli_route::park`); low, a
  helicopter is on the ground for the tower (`hover::tower_on_ground`). The AP's mode tools are
  refused while the loop flies (`hover::blocks`).

## The flight model: one state, read by everyone

`src/flight.rs`: the ONE answer to "where are we and what do we hold". Pure, fed per monitor
tick, time from the sample so recorded flights replay (`flight::Replay`). Tracks `Phase`,
`Control`, `Abnormal`, `Memory`, `Situation`. A ground strike is an `Impact`, not a landing.
Read by `monitor::coarse_phase`, the checklist tracker, cabin (`FlightFacts`), "how are we
doing?" (`flight::status_line`), rapport's touchdown and bounce. **The duty engine**
(`src/duty.rs`, pure) says what the flight owes next; SHADOW only (blackbox `duty`). **The
hand-over reads the model** (`src/handover.rs`): "you have control" resolves into the calls the
phase needs or a spoken decline; the upset RECOVERY only for an actual upset or distress words.
"Let me fly" is the release guard (`note_crew_takeover`).

## Checklist flows: the list at its phase, three flavours

`src/flow.rs` runs the airframe's XChecklist sections (`src/checklist.rs`); replies via
`flow::take_reply` (CLOSED vocabulary). Every `Item::Act` carries its pass test, so a runner reads
FIRST: in state is confirmed, never re-written; an item a loop owns (`held_by_automation`) is
named as held, never written.

- **Crew** (pilot flying, copilot acting): the spoken "descent checklist". **Monitor** (an
  automation flying): read-only and silent, ONE line at the end (`flow::summary_line`).
  **Instruct** ("talk me through"): never writes; says, watches, moves on.
- **A compound test is read whole** (`checklist::Expr`): a term that does not read is no
  verdict; an ACT only through its ONE settable term. A file the aircraft ships is
  `checklist: acf:<file>` (`checklist::resolve_acf_path`); a phase split into parts is ONE list
  (`checklist_flow:`).
- **Who starts one**: `flow::Tracker`, one-shot per flight, every boundary SUSTAINED. Under an
  automation Monitor starts at once (BEFORE LANDING only once configured); with a pilot flying the
  copilot PROPOSES and never acts by itself; the after-landing list runs once OFF the runway. A
  list belongs to the flight it began in (`flow::stand_down`).

## App modes & cabin crew

`mode.rs`: **both** (default) / **copilot** (no simulated ATC) / **atc** (the world only) /
**instructor** (hands off). `XCOPILOT_MODE` sets the BOOT value only. The no-crew gate is the
arbiter's enqueue door (`Voice::alert`); the atc crate never reads the mode.
**`mode::crew_actuates()` is the hands gate** (false in atc AND instructor): tool dispatch,
`fix_action`s, `cockpit_auto`, `flow::start`, undo. Entering instructor stands every loop down.

**Instructor** (`src/instructor.rs`): one ACS task at a time. The brief ASSIGNS the numbers, so
the scorer never guesses; ACS tolerances per level (`Task::standards`). An AP-shaped command in a
lesson is a RETARGET (`take_target`), never a write. "Teach me a full flight" (`Syllabus`): the
whole flight is a plain `cargo test`; `xfc debrief` prints lessons. "Show me" is the one exception
to folded arms (`instructor::show_me`, `mode::tool_may_actuate`): armed once through the tool
door, narrated, graded, then "Your controls."

**Cabin crew** (`src/cabin.rs`): the second intercom persona. A state machine, not a commentator,
with its own mute (`cabin_quiet`), NOT the monitor's `quiet`. Addressed by "cabin crew"; "say
again" over guessing; per airframe (`cabin_crew: true`); "cabin secure" is exempt from sterile
cockpit. Works the flight unasked (`Cabin::watch`), each call once per flight, never after the
flight deck made it. A bare "cabin crew?" opens the line (`take_open_line`).

## The copilot as a person

- **Timing** (`src/tempo.rs`): a jittered pause from the end of CAPTURE, "stand by" when nothing
  is queued (never over a radio call).
- **Persona** (`src/persona.rs`): `XCOPILOT_CREW_CHATTER` gates only what is VOLUNTEERED, never an
  answer or safety call; idiom follows the voice (`XCOPILOT_CREW_DIALECT`); small talk is CLOSED.
- **Rapport** (`src/rapport.rs`): praise goes to the pilot, never an automation; a bounce counts
  at the SECOND contact; offers are sentences, never actions.

## Panel, terminal, settings

- **The panel is the act-and-glance instrument**, fed by `xcopilot/status/auto`
  (`src/auto_status.rs`), change-gated with a periodic resend; the taxi turn card on
  `xcopilot/status/taxi`. Buttons inject pilot English (`REQUEST_PHRASES`, grammar-test-pinned);
  CANCEL ALWAYS aborts something. The typed line is `xcopilot/typed` (`pollers::queue_command`).
  Native panel-graphics API (SDK 4.4.0; `plugin/src/imgui_xp.cpp`).
- **Settings are a registry** (`src/settings.rs`, `settings::live()`): env > pref > default; an
  env-pinned knob is REFUSED on screen, naming the variable. **Every pilot-facing variable is a
  setting** (`every_environment_variable_is_a_setting_or_named_env_only`; developer knobs on
  `ENV_ONLY` with a reason), in a `Section`. A boot knob (`Tier::Restart`) reads its pref where
  boot reads it (`config::knob`, `BOOT_KNOBS`), pinned by `every_setting_survives_a_restart`. A
  panel toggle is a pick only when it differs from our LAST PUBLISH (`pollers::panel_sync`). Who
  speaks with what is ONE function (`voice::cast`); every voice pick goes through `settings::set`.
  `.env` pins move into prefs only on the pilot's word (`env_import`, backed up).
- **Only settings persist** (`prefs.json`, env, geometry), never a state dump. The one other file
  is an airframe's MEASURED model (`profiles/learned/<acf>.json`), written only by `xfc identify`,
  reviewed and committed like a profile.
- **Every file root has ONE owner** (`sim::paths`, `every_file_root_goes_through_the_owner`):
  `resources()` ships, `data()` is the user's; `XCOPILOT_HOME` overrides, a dev checkout is the
  cwd, installed is `~/Library/Application Support/XFlightCrew` (XDG on Linux); `xfc paths` says
  where. **One `xfc` file needs nothing beside it**: resources built in (`crates/sim/build.rs`),
  unpacked once only when no `profiles/*.yaml` is on disk. `xfc plugin status|install`
  (`src/plugin_install.rs`): `xcopilot/` removed first, backups outside `plugins/`, refused while
  X-Plane runs unless `--while-running`, NEVER reloads plugins.
- **One app, no setup step** (`src/app.rs`): plain `xfc` on a TTY is ONE TUI, a sidebar and a
  page (Flight, Checks, Voices, one per `settings::Section`, Plugin, Files); Enter in, Esc one
  level back, a setting changes only after Enter, `/` searches. **The crew boots itself** the
  moment the required checks pass (`spawn_tui_crew`). Headless is `XCOPILOT_UI=plain`. **A crew
  restart is the process starting again in place, ON THE GROUND ONLY** (`src/restart.rs`; an
  in-process re-boot is unsound), with the SHELL's environment (`capture_original_env`). The ATC
  pool draws INSTALLED voices only (`voice::installed_pool`); the starter set is `voices::starter`.

## EFB: a page on the LAN (`src/efb/`, `docs/efb.md`)

- **No new actuation path**: a button or the typed box POSTs pilot English (`/say`) onto the
  command channel like the panel's typed line; the grammar picks the tool, gates and readback
  included; phrases in `efb::state::PHRASES`, each pinned to its route
  (`every_button_routes_where_it_says`). SIM MANAGEMENT ("Move here") calls the owning tool's
  gated function through its own endpoint (`POST /reposition` ->
  `tools::setup::reposition_to_stand`), never a raw write, never an invented phrase.
- **No new sim reads**: the page is ONE pure `state::build` over `Facts` the app already holds;
  `efb::feed` taps the speakers, the command loop and the monitor's tick; a fact nobody holds is
  a dash. The Flight tab is laid out by the flight model's phase (`efb::flight_tab::layout_for`);
  the transcript is labelled FROM -> TO by one pure rule (`efb::transcript::label`, the panel's
  `xcopilot/status/fromto` too). The banner and chip are HAZARDS (`Voice::hazard` with a
  `monitor::Severity`), never every `Priority::Warning` line. The EFB's PTT is a heartbeat
  (`efb::ptt`), capped at 60 s.
- **The Map tab is ONE world map** (`map.js`, Web Mercator, FOLLOW by default), every layer in
  one frame: X-Plane's INSTALLED water (`efb::basemap`, never wider than `MAX_TILES`), the plan
  (`efb::map`), traffic and our aircraft, and closer in ANY field's diagram (`efb::airport`,
  built on demand, cached, ETag'd), placed by ONE transform (`fieldFrame`,
  `the_page_places_a_field_where_the_server_measured_it`). Traffic and stands are the traffic
  world's (`TrafficWorld::note_efb_traffic`, `Fleet::stand_holder`; no world simulating the
  field is UNKNOWN, never free). A timetabled flight exists from blocks to blocks
  (`traffic::track`, `src/traffic_whole.rs`, `a_whole_flight_never_jumps_at_either_handover`),
  drawn only inside the draw ring, never a TCAS target for the station.
- Off by default (`XCOPILOT_EFB`, `_PORT`, `_TOKEN`); every request needs the token (a wrong one
  a 403, blackbox `efb` `auth_failed`). Its own thread and runtime, never the command loop's; no
  lock held across an await.

## Web API load is a sim cost

The API runs in X-Plane's flight loop: every request is frame time. `crates/sim/src/xplane.rs`
reports every 30 s (blackbox `api_load`). A 60 ms read cache (a write drops its entry);
near-static flags are `SLOW_DATAREFS`; a COMMAND clears it. **Nothing asks a loading sim**
(`xplane::LoadGate`, fails open). Plugin work never runs in a Web API set handler: store, flag,
act in a flight loop.

## Battle-test surface

`cargo test --workspace` (root `cargo test` is the crew package only), with
`XCOPILOT_BLACKBOX=off` or they record into the live log. Format only the files you edited, never
a bare `cargo fmt`. **A fix merged offline gets an `unflown` issue** listing the blackbox fields to
check; the sortie that confirms it closes it (`fixes #N`).

- ATC: the `tower.rs` matrix; `crates/atc/bench/atc_corpus.yaml` (real whisper, add every new
  wart); `route_keyed`/`route_unkeyed`; `cargo run -- stt-eval bench/atc_stt_cases.yaml`;
  `cargo test --release -p atc -- --ignored atc_world_sweep`; `docs/atc-chaos-card.md`.
- **`xfc debrief` after every flight, before diagnosing by hand** (`--sortie <id|last>`,
  `--airframe <id>`). **A new airframe or profile edit is checked parked first** (`xfc conform`,
  `docs/conform.md`): every declared channel through its own seam, read back, put back.
- **A sortie is one file with a verdict** (`full_auto::SortieGuard`, `logs/sorties/<id>.jsonl`,
  `scorecard::sortie_result`): PASS needs no failure, give-up or crash, the ground at the end,
  and UNATTENDED; the crew's own yoke and brake are not a hand; a landing is scored at its FIRST
  contact. `bench/sorties/` pins real verdicts; `bench/replay/` replays real flights through a
  pure decision. A blackbox record is one write (`blackbox::append_line`, file-locked).
- **A night of landings is one command** (`xfc sortie <spec> --repeat N`, `src/sortie_runner.rs`;
  `--check` first, no sim): an airborne aircraft is never handed to nobody (`hand_back`,
  `pause_safe`); no setup under a moving aircraft (`SettleWatch`); an arrival case starts IN THE
  AIR on the plan (`setup_enroute::place`); fast forward only where the laws tolerate it
  (`SpeedGuard`).
- A law that misbehaves in the air while its test passes: suspect the plant. A first-order lag
  cannot ring; fly second-order plants through the real transport delay.

## Licensing

**GPL-3.0-or-later** (`LICENSE`, `[workspace.package]`): open source, never commercial, X-Plane
only, copyleft on purpose. SDK and Dear ImGui MIT/BSD, B612 OFL. Relicensing is free only until
outside contributions merge.
