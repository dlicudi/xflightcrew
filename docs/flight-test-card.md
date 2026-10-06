# Flight-test card

One card for every change that shipped on offline tests alone and has not
yet been proven by a sortie. Run it after any flight:

```
xvl debrief logs/blackbox.jsonl
```

The last section of the debrief judges every row below over the WHOLE file
it is given (not just the last flight): `verified` or `FAILED` with the
number, `not exercised` (grouped by the sortie that would exercise it), or
`by hand` for the rows whose criterion is not a number in the log.

The rows live in Rust (`src/test_card.rs`, `ITEMS`: name, what to fly, the
evidence, the criterion and the predicate), and a test
(`test_card::tests::the_doc_and_the_table_agree`) holds the first five
columns of this file to that table, so the two cannot drift. Edit the table
first, then regenerate the rows with
`cargo test test_card::tests::print_rows -- --ignored --nocapture`. The
status column is free text: the debrief's verdict, copied in when a sortie
settles a row.

Rules the predicates keep:

- An item counts only evidence recorded at or after the commit that shipped
  it: a flight before the commit flew the old code.
- An item for one airframe counts only that airframe's records (the latest
  `flight_meta` before each record). A sortie file has no `flight_meta`, so
  airframe rows read the main blackbox.
- Every event and field named here was read off the source that records it
  (`blackbox::record("<kind>", ...)`). Full auto's stage fields live under
  `detail`; the flight model's name is `ev`; a callout's is `id`.
- `state` `in.ap` is not the servos: it reads `autopilot_on`, which stayed 1
  through every arrest and rollout on 2026-09-26. Servos are judged from
  `autoland arrest_hands` and `ap_disconnected`.

## Settled by the logs already flown

Judged over `logs/blackbox.jsonl` as it stood on 2026-09-26 (records from
2026-09-25 20:03Z to 2026-09-26 17:44Z; times below are UTC): the 172
LIPK-LIPR and LIPR-LIBP legs, the 737 LIBP-LEMG leg (the 12:54 takeoff
crash, then the flight from 13:34 that the pilot took over on the LEMG 13
approach), the 737 circuit at LEMG from 16:48, and five repositioned 737
autolands onto LEMG 13 from 17:14 while the flare was being worked on.

Notes on the failures:

- `roll_trim_scaled`: the trim now shrinks with speed (0.12 above 90 kt on
  the 13:43 roll against 0.21 on the crash roll), but the roll still went
  9.9 m and 6.9 m off the centreline against the scorecard's 5 m. The
  smoothed cross-track rate (3a64155, 17:01Z) came after both rolls and is
  unflown on a takeoff.
- `touchdown_centreline` / `rollout_law`: the two 737 landings after
  f107203 touched down 11.8 m and 14.9 m left; the rollout law itself held
  those offsets (no crossing) and grew one of them by 3.8 m.
- `b738_flare`: 411 and 409 fpm by the scorecard after the last flare
  change (f107203); the limit is 400.
- `star_windows`: FOFUS crossed at 10,386 ft against its 10,000 ft ceiling
  (UNTO2A, LEMG).
- `own_star`: the arm chose runway 12 for UNTO2A on a 120/9 wind, but the
  approach tuned and flew the ILS 13 and landed 13 (self-picked after the
  pilot took over), so the choice was never carried into the arrival.
- `stand_shutdown`: the 737's electrics-off left the battery on.
- `tug_unhook`: the 13:39 push ended with the tug never showing clear.
- `duty_engine`: 13 of 124 shadow duty changes disagreed with the running
  automation (line_up, taxi, taxi_to_stand, vacate).

### Airliner full-auto IFR flight (737, A330)

| item | shipped | what to fly (airframe, field, action) | blackbox evidence (kind, event, fields) | pass criterion | status |
|---|---|---|---|---|---|
| `b738_speedbrake_down` A Boeing takeoff puts the speedbrake lever down, never armed for the roll | 2026-09-21 3950a94 | 737, any full-auto departure | autotakeoff takeoff_config: set, not_taken | no "spoilers armed" in any 737 set | **verified** 2026-09-26: 4 takeoff configs, none armed the spoilers (["flaps","trim","autobrake max"]) |
| `jet_climb_out` Jet climb-out: VS holds V2+20 below the acceleration height, flaps wait for it | 2026-09-21 3950a94 | 737 or A330, full-auto departure to the handoff | autotakeoff climb_out: accel_agl_ft; flap_step: agl_ft, ias; flaps_handoff | every flap_step at or above accel_agl_ft (25 ft slack), no flaps_handoff after a climb_out | **verified** 2026-09-26: 13:43: flap steps at 1002, 1018, 1374 ft against 1000 |
| `roll_trim_scaled` Takeoff roll rudder trim shrinks with the square of the speed (LIBP 04 crash) | 2026-09-26 3fd8e6b | 737, full-auto takeoff | autotakeoff roll: xtk_m, trim, gs | most off the centreline on the roll within the scorecard's 5 m | **FAILED** 2026-09-26: 13:43: 9.9 m off at worst, trim 0.12 above 90 kt; 16:58: 6.9 m off at worst, trim 0.07 above 90 kt |
| `rollout_law` Rollout steering leads with a smoothed cross-track rate | 2026-09-26 3a64155 | any autoland | autoland rollout: xtk_m, xtk_rate, gs | the offset grows no more than 2 m after the first tick, at most one crossing above 40 kt | **FAILED** 2026-09-26: 17:32: 11.8 m off at the first tick, grew 3.8 m, 0 crossing(s) |
| `touchdown_centreline` Over the runway with the servos out the wings follow the centreline law plus the crab | 2026-09-26 f107203 | 737 autoland | autoland rollout: xtk_m at the first tick | within the scorecard's 5 m of the centreline at the wheels | **FAILED** 2026-09-26: 17:32: 11.8 m left at the wheels; 17:43: 14.9 m left at the wheels |
| `ap_off_at_wheels` Autoland takes the autopilot out at the wheels | 2026-09-21 3950a94 | any autoland to a stop | autoland ap_disconnected: ok; state in.ap | AP off within 10 s of the first rollout tick (already off counts) | **verified** 2026-09-26: 09:02: servos out since the arrest at 49 ft; 11:11: servos out since the arrest at 50 ft; 16:29: servos out since the arrest at 79 ft; 17:04: servos out since the arrest at 81 ft; 17:17: servos out since the arrest at 79 ft; 17:25: servos out since the ... |
| `b738_flare` 737 flare: retard by height, exponential target, pull-only arrest from the roundout's pull | 2026-09-26 630506f..f107203 | 737 autoland | scorecard landing stage (sink_fpm) from state and autoland ticks; autoland flare_elev, cmd_vs, vvi | touchdown at or under the scorecard's 400 fpm | **FAILED** 2026-09-26: 17:32: 411 fpm; 17:44: 409 fpm |
| `altimeter_rounds` The altimeter is stepped in rounds earned by progress and says when it did not take (EGSS) | 2026-09-22 125e2b3 | 737 loaded cold (baro memory 0.0), full auto | full_auto altimeter_set: took, want, inhg; procedure altimeter_set: took; callout altimeter_reset: took | every setting took | **verified** 2026-09-26: 24 settings, every one took |
| `reversal_pattern` A reposition is a pattern, never a point (EGTE) | 2026-09-21 3950a94 | any, autoland armed facing away from the runway | autoland reversal: side, platform_ft; reversal_done: xtk_m, dist | a pattern that runs to its end is reversal_done within 100 m of the centreline | **verified** 2026-09-26: 10:58: done 58 m off at 5.5 nm; 3 more interrupted by the pilot or a re-arm |
| `descent_drag` A descent that outruns the governor gets the speedbrake, stowed on every exit | 2026-09-21 3950a94 | 737 full-auto arrival | procedure speedbrake: out, ias, target | every extension stowed by the end | **verified** 2026-09-26: 13 extensions, all stowed; last at 16:17 at 147 kt against 145 |
| `inbound_retune` The inbound call is keyed on the tower's frequency | 2026-09-21 3950a94 | any full-auto arrival into a towered field | full_auto inbound_retune: khz, was; inbound_call: khz | every retune is followed by the inbound call on that frequency | **verified** 2026-09-26: 08:38: called on 119100; 16:00: called on 118155 |
| `star_windows` STAR altitude windows clamped into the plan and flown | 2026-09-24 b30865d | 737 full auto with a SimBrief STAR | plan_audit star_constraints: changed_legs, constraints; procedure star_ceiling; procedure leg: fix, alt_ft, dist_nm | every constrained fix crossed inside its window (300 ft slack) | **FAILED** 2026-09-26: FOFUS at 10386 (7000..10000) |
| `star_speeds` STAR speed limits slowed for in time and held | 2026-09-24 b30865d | 737 full auto with a STAR speed limit | procedure star_speed: fix, kts, ias; state ias at the fix | IAS at each limited fix within 10 kt of the limit | **verified** 2026-09-26: MG539 at 230 kt (limit 230); OBLAC at 229 kt (limit 220); FOFUS at 229 kt (limit 220); MG529 at 211 kt (limit 210); MG533 at 208 kt (limit 210); MG537 at 208 kt (limit 210); MG541 at 207 kt (limit 210); MG437 at 210 kt (limit 210); MG433 at 211 kt (limit ... |
| `own_star` The crew picks its STAR for the METAR-favoured runway at the arm | 2026-09-25 f30f1f0 | 737 full auto into a field whose wind favours another end than the plan's | plan_audit star_chosen: star, runway, planned_star, planned_runway, wind; taxi_in vacating: rwy | the runway landed on is the runway chosen | **FAILED** 2026-09-26: 13:34: chose UNTO2A for 12 (planned UNTO2A for 13), landed 13 |
| `plan_audit` The whole plan is dry-run before it is flown | 2026-09-24 b30865d | any full auto with a plan | plan_audit (no event): findings, profile, probed_nm; sortie begin: origin, dest | every planned mission audited at the arm, none armed over a refusal | **verified** 2026-09-26: 20:04 LIPE-LIPK: audited, 1 finding(s); 07:42 LIPK-LIPR: audited, 1 finding(s); 07:47 LIPK-LIPR: audited, 1 finding(s); 08:12 LIPK-LIPR: audited, 1 finding(s); 09:26 LIPR-LIBP: audited, 2 finding(s); 12:46 LIBP-LEMG: audited, 1 finding(s); 13:18 LIBP-LEMG: ... |
| `load_sheet` "Load the aircraft" sets block fuel (wings first) and payload from the plan | 2026-09-21 3950a94 | 737 or A330 on the ramp: "load the aircraft" | loading load_aircraft: ok, fuel_want_kg, fuel_after_kg, payload_want_kg, payload_after_kg, per_tank_kg | ok, fuel and payload within 2 % (or 5 kg) of the plan | **verified** 2026-09-26: 20:04: fuel 94/94 kg, payload 209/209 kg; 09:15: fuel 102/102 kg, payload 275/275 kg; 12:46: fuel 10573/10573 kg, payload 16130/16130 kg |
| `takeoff_config` A crew configures the aircraft before it moves | 2026-09-21 3950a94 | any full-auto departure | autotakeoff takeoff_config: set, not_taken, waited_flaps | nothing in not_taken | **verified** 2026-09-26: 15 takeoff configs, nothing not taken |
| `seat_belt_signs` Seat belt signs on before anything turns, off through 10,000, on again for the arrival | 2026-09-24 8dddc5b | 737 or A330 full auto through 10,000 ft both ways | cabin seat_belt_signs: on, ok | every switching took, and an off is followed by an on | **verified** 2026-09-26: 7 switchings, sequence 1110111 (1 on, 0 off) |
| `ils_tuned` NAV1 tuned to the ILS and its needles cross-checked on every landing | 2026-09-25 f30f1f0 | any full-auto arrival to an ILS runway | ils tuned: ok, freq_mhz, course; ils xcheck: hdef, vdef | every tune read back | **verified** 2026-09-26: 13 tunes read back, 4 needle cross-check call(s) |
| `systems_survey` The systems survey at detection knows each autopilot | 2026-09-25 5d5bf46 | any airframe swap (172, 737, A330) | systems: profile, ap_type, glass_hsi, tcas, auto_pressurization | every survey names an autopilot type | **verified** 2026-09-26: cessna_172sp Gfc700, boeing_737_800 Airliner |
| `self_pick_destination` "Land the aircraft" prefers the plan's destination | 2026-09-25 ee493cc | any, autoland asked near the destination with a plan loaded | autoland self_pick: destination, rwy, min_length_m | the picked runway is at the destination | **verified** 2026-09-26: 16:17: LEMG 13 for LEMG |

### Light aircraft circuit or hop (172, SF50, Evolution, SR22)

| item | shipped | what to fly (airframe, field, action) | blackbox evidence (kind, event, fields) | pass criterion | status |
|---|---|---|---|---|---|
| `xpdr_alt_at_roll` The transponder goes to ALT as the roll starts | 2026-09-24 8dddc5b | any full-auto or spoken takeoff | autotakeoff transponder_alt: ok | every roll confirmed ALT | **verified** 2026-09-26: 11 of 11 confirmed ALT |
| `leg_stall_per_leg` The follower counts stalls per leg with a turn grace (LIPK approach stand-down) | 2026-09-25 ee493cc | 172 full auto with an approach that turns more than 90 degrees | procedure leg_stalled: idx, stalls; procedure stand_down: reason | no stand_down for leg_stalled | **verified** 2026-09-26: 28832 follower ticks, 1 leg_stalled (most 1 on one leg) |
| `heading_set_once` The follower sets the heading once per leg like a pilot, no flip-flop | 2026-09-26 327d121 | any full-auto enroute leg | procedure heading_set: idx, hdg | no three sets on one leg inside 30 s that go back and forth | **verified** 2026-09-26: 70 heading sets, no flip-flop |
| `circuit_short_way` Circuit turns go the short way (bug entry lead 25 deg) | 2026-09-23 f1d6791 | 172 or SF50 rectangle circuit | circuit leg: leg, psi, bug, tgt | no leg turns more than 200 degrees net | **verified** 2026-09-26: 5 legs, the most any leg turned was 93 deg |
| `stand_shutdown` A mission ends shut down at the stand, engines and electrics off | 2026-09-24 8dddc5b | any full auto to the stand | taxi_in shutdown: tool, stopped; taxi_in electrics_off: line | engines stopped and the electrics line names no exception | **FAILED** 2026-09-26: 17:07 boeing_737_800: engines stopped, "All electrical systems off, except: The battery didn't move." |
| `vacate_stand_side` The vacate turns toward the parking and never onto another runway (EBLG) | 2026-09-22 125e2b3 | 737 into a field with parallels (EBLG 04R) | taxi_in vacating: rwy, exit; taxi_in vacated: arrived; autotaxi off_route | vacated and arrived with no off_route stop on the taxi in (the side is judged by eye) | **verified** 2026-09-26: 09:02: runway 30 via B; 11:12: runway 04 via C; 16:30: runway 13 via E2; 17:05: runway 13 via E2 |
| `margin_waiver` "Fly it anyway" waives a margin gate and re-arms | 2026-09-25 389728a | any, a plan refused on the runway factor or the plan weight | full_auto waiver: gate | a takeoff follows the waiver | **verified** 2026-09-26: 09:26: PlanWeight waived, took off |
| `touchdown_idle` A flight speed hold on the runway closes the throttle and lets go | 2026-09-25 f30f1f0 | any, a speed hold still armed at the touchdown | speedhold touchdown_idle: ias, target | the hold idles on the runway | **verified** 2026-09-26: 21:11: idled at 54 kt (target 60) |

### Ramp session (start, pushback, cabin, hand-over words)

| item | shipped | what to fly (airframe, field, action) | blackbox evidence (kind, event, fields) | pass criterion | status |
|---|---|---|---|---|---|
| `tug_unhook` The toe brakes hold until the tug is off the bar, then the parking brake | 2026-09-26 327d121 | 737 pushback from a stand | pushback brakes_set: tug_clear; pushback tug_not_clear | the tug shows clear on every push | **FAILED** 2026-09-26: 13:39: tug never showed clear |
| `cabin_watch` The cabin works the flight unasked and never calls a departure after a landing (EBLG) | 2026-09-22 125e2b3 | any airliner leg to the stand | cabin (route watch) securing_for_departure, secure_at_liftoff, securing_for_landing, deplaning; flight touchdown, flight_complete | no securing_for_departure between a touchdown and the flight's end | **verified** 2026-09-26: 9 landings, no departure call after any |

### Hand-flown session (rapport, tempo, say again)

| item | shipped | what to fly (airframe, field, action) | blackbox evidence (kind, event, fields) | pass criterion | status |
|---|---|---|---|---|---|
| `rapport_landing_word` The landing word: praise for the pilot only, a hard landing always said | 2026-09-23 f1d6791 | any landing, pilot and automation | rapport touchdown: sink_fpm, by_automation; rapport said: line | no praise after an automation's landing, "hard" said at 600 fpm or more | **verified** 2026-09-26: 21:11: 126 fpm by pilot, said "Nice one, about 130 feet a minute."; 09:02: 509 fpm by automation, said "By the way, firmer than I'd like, about 510 feet a minute."; 11:11: 411 fpm by automation, said "By the way, firmer than I'd like, about 410 feet a ... |
| `duty_engine` The duty engine (shadow) agrees with the flights flown | 2026-09-25 cadf9f8 | any flight | duty: duty, phase, why, agrees | no disagreement with the running automation | **FAILED** 2026-09-26: 13 of 124 duty changes disagreed (line_up, taxi, taxi_to_stand, vacate) |
| `flight_model_touchdowns` The flight model sees every landing | 2026-09-25 389728a | any landing | flight touchdown: sink, gs; rapport touchdown | a flight touchdown within 5 s of every rapport touchdown | **verified** 2026-09-26: every touchdown seen by the model (21:11; 09:02; 11:11; 16:29; 17:04; 17:17; 17:25; 17:32; 17:43) |

## Still to fly, by sortie

Nothing in the logs yet since the item shipped, or the criterion is a
person's to judge. Pick the sortie, fly the action in the third column, run
the debrief.

### Airliner full-auto IFR flight (737, A330)

| item | shipped | what to fly (airframe, field, action) | blackbox evidence (kind, event, fields) | pass criterion | status |
|---|---|---|---|---|---|
| `reject_to_a_stop` A rolling reject is flown to a stop on the centreline under max braking | 2026-09-26 3fd8e6b | any, a takeoff that rejects (excursion or stagnation) | autotakeoff reject; reject_rollout: gs, xtk_m | last reject_rollout tick under 25 kt, never more than 15 m off | not exercised |
| `pattern_height` The pattern extends its downwind while it is too high to turn (EGSS) | 2026-09-22 125e2b3 | 737, autoland armed high above the field | autoland downwind_extended: above_ft, along_nm | the extended pattern ends in reversal_done, not a go-around or give-up | not exercised |
| `descent_path` A descent is flown down a path at 75 % of the rate from an earlier top of descent | 2026-09-26 dd501bc | 737 full-auto arrival from cruise | procedure leg: want_alt, alt_ft; procedure speedbrake: out | at most 2 speedbrake extensions over the descent (13 on LIBP-LEMG before) | not exercised |
| `tod_once` The top of descent is called once, from the cruise | 2026-09-26 b417a7f | any full-auto arrival with stepped descents | callout tod_ahead, tod_passed | at most one ahead and one passed call per flight | not exercised |
| `own_sid` The crew picks the SID from the cleared runway | 2026-09-25 f30f1f0 | full auto where the tower clears a runway the plan's SID does not serve (LIME 10) | full_auto departure_rechosen: line | a liftoff follows the re-chosen departure | not exercised |
| `cabin_air` Bleed and crew oxygen on for every full-auto mission (EDLP-LOWI hypoxia) | 2026-09-24 8dddc5b | SF50 (the only profile with cabin_air) full auto above FL100 | full_auto cabin_air: bleed, oxygen (the supply), changed; cabin_air masks don/doff/released | bleed and oxygen both read on | not exercised |
| `ils_coupled_737` The 737 flies the ILS coupled through its own APP button (app_press, app_status) | 2026-09-26 18627ec | 737 autoland on an ILS runway | ils coupling: from, to, appr_reads, dist | a coupling reaches Coupled, none Abandoned | not exercised |
| `late_go_around` A late high approach inside the FAP goes around straight ahead before it repositions | 2026-09-26 3b29f25 | 737, autoland armed high inside the FAP (LEMG 13 at 2.6 nm) | autoland late_go_around: dist, radio, above_path_ft | the go-around is flown with no crash after it | not exercised |
| `map_flaps_on_speed` The final to a runway MAP puts the flaps out on speed as well as by distance | 2026-09-26 e4a196f | 737 full auto onto an RNAV approach whose MAP is the runway | procedure map_config: flap, ias; callout airspeed_caution, airspeed_warning | no low-speed callout in the 5 min after the first map_config | not exercised |
| `a330_transponder_knob` The A330's transponder knobs are stepped by command and confirmed from the output | 2026-09-21 3950a94 | A330 full-auto departure (EGPF circuit is the reference) | autotakeoff transponder_alt: ok | every ALT set confirmed | not exercised |
| `a330_spoilers_disarmed` Ground spoilers armed for the roll are disarmed after the gear | 2026-09-21 3950a94 | A330 full-auto departure | autotakeoff spoilers_disarmed: ok; gear_up | one disarm per gear retraction, every one taken | not exercised |
| `a330_runway_start` A runway start still calls ready for departure and waits for the clearance | 2026-09-21 3950a94 | A330 spawned on the numbers, full auto | full_auto request_departure, readback_takeoff; atc party_line | the call and the clearance precede the roll (read the tower states Parked, LinedUp, TakeoffCleared) (by hand) | unflown (by hand) |
| `a330_alt_capture` The autopilot's own capture holds the circuit's level legs | 2026-09-21 3950a94 | A330 circuit | circuit leg: ap_level, vs_on; circuit vs_refused | ap_level true on the level legs, no vs_refused | not exercised |
| `lnav_flown` The 737 follower flies its plain legs on LNAV, the heading bug the watched fallback | 2026-10-06 #227 | 737 enroute setup over a fix, then "fly the flight plan" | lnav engage, engaged, captured: idx, fix, xtk, secs; dropped: why, xtk, status; fallback; reengaged; handback; retired | 3 or more legs captured on LNAV, every fallback followed by a re-engage, never retired | not exercised |

### Light aircraft circuit or hop (172, SF50, Evolution, SR22)

| item | shipped | what to fly (airframe, field, action) | blackbox evidence (kind, event, fields) | pass criterion | status |
|---|---|---|---|---|---|
| `sf50_gear_warning` The SF50's gear comes up outside its warning envelope | 2026-09-24 8dddc5b | SF50 full-auto takeoff | autotakeoff gear_up: agl, ias, flap, at_handoff | gear up at 128 kt or more, or 1,000 ft, or at the handoff | not exercised |
| `sf50_flare` The SF50's flare gains (read off the EDLP 24 trace) | 2026-09-24 8dddc5b | SF50 autoland | scorecard landing stage (sink_fpm); autoland flare_elev, vvi | touchdown at or under the scorecard's 400 fpm | not exercised |
| `committed_flare` A low flare or balloon with the airframe's landing distance ahead is committed | 2026-09-24 8dddc5b | SF50 autoland (a float past the allowance) | autoland committed_flare: radio, remaining_m; flight touchdown: sink | a touchdown within 60 s and no crash | not exercised |
| `untowered_departure` An out-of-reach station leaves full auto to plan its own taxi (LILB) | 2026-09-25 389728a | any, full auto from an untowered field | full_auto departure_field: towered; full_auto lined_up | an untowered departure reaches the line-up | not exercised |
| `evolution_taxi` The Evolution taxis on a capped bend lookahead and smoothed brakes | 2026-09-25 f30f1f0 | Lancair Evolution full-auto taxi | scorecard taxi stage from autotaxi drive ticks | every taxi stage passes the scorecard | not exercised |
| `sr22_speed_refused` A misheard spoken speed past the ceiling or under the floor is refused | 2026-09-24 5f8a38c | SR22, ask for an impossible speed | speedhold spoken_target_refused: asked | the refusal fires | not exercised |
| `sr22_lineup_pivot` The line-up pivots on the brake for a big heading error under 7 kt | 2026-09-24 5f8a38c | SR22 full-auto line-up at a turn of more than 60 degrees | autotaxi drive: bs, bl, br in the slow turn | the inside brake leads and the line-up ends aligned (read the ticks) (by hand) | unflown (by hand) |
| `lfpb_strip` The taxi route ends on the cleared strip where two thresholds share an end | 2026-09-22 125e2b3 | 737, full-auto departure from LFPB 07 (or any of the 1,815 such fields) | full_auto line_up_no_fit: why; full_auto lined_up | lined up with no line_up_no_fit at a field whose ends converge (by hand) | unflown (by hand) |

### Helicopter route (S-76)

| item | shipped | what to fly (airframe, field, action) | blackbox evidence (kind, event, fields) | pass criterion | status |
|---|---|---|---|---|---|
| `heli_approach` The S-76 comes down at speed to a gate, then the hover loop's final | 2026-09-23 f1d6791 | S-76 route into a lowland pad | heli_route approach, descent_vs, slow (approach true), handover_hover: ias, to_pad_m; arrived: on_ground | the hover loop takes over at 45 kt or less and it lands on the pad, no crash | not exercised |
| `heli_inbound` The helicopter's inbound call waits for the destination's station | 2026-09-23 f1d6791 | S-76 route into a towered field | heli_route inbound_call: khz, station_swapped | station_swapped true | not exercised |
| `heli_radio` The helicopter works the radio with the aeroplanes' code (zone clearance, handoffs) | 2026-09-23 f1d6791 | S-76 route out of a towered field | heli_route zone_clearance, departure_call, departure_handoff, region_handoff | a zone clearance before every departure call | not exercised |
| `heli_route_terrain` Climb gate, corridor floors, fast climb and a collective floor on the approach (LILO-LILB crashes) | 2026-09-24 b30865d | S-76 route through a valley, lowland arrival first | heli_route start: cruise_alt_ft, terrain_ft; gate_lowered; arrived: on_ground; crash | every started route lands, none crashes | not exercised |
| `heli_pad_shutdown` A helipad landing ends there, shut down | 2026-09-24 858a0d4 | S-76 route to a helipad | heli_route pad_shutdown; taxi_in shutdown: stopped | the engines stop on the pad | not exercised |
| `hover_hand_back` In the air the autopilot is never simply turned off | 2026-09-23 058c6ff | S-76, any hover-loop hand-back | hover hand_back: why, cruise_modes, hover_mode | every hand-back leaves cruise modes or the hover mode on | not exercised |

### Ramp session (start, pushback, cabin, hand-over words)

| item | shipped | what to fly (airframe, field, action) | blackbox evidence (kind, event, fields) | pass criterion | status |
|---|---|---|---|---|---|
| `push_lane` The push follows the stand's painted lead-in and ends at the planned distance | 2026-09-26 3226e77 | 737 pushback from a stand with a drawn lead-in | full_auto push_lane: back_m, target_hdg; pushback stopped: back_m | pushed within 10 m of the planned distance | not exercised |
| `hand_back_brakes` The hand-back's parking brake is set on an unscoped client ("automation cancelled") | 2026-09-26 a8730cb | any full auto that gives up on the ground | full_auto hand_back_brakes: held, tries, err | held true | not exercised |
| `citation_cold_start` The Citation X starts from a cold APU (loaded at 99, bleed read back) | 2026-09-23 b3091d8 | Citation X cold and dark: "engines on" | turbine_start apu, apu_loaded: air, volts; turbine_start crank: engine, verdict, peak_spool | every engine's crank verdict is Lit | not exercised |
| `cabin_open_line` A bare "cabin crew?" answers go ahead and opens the line | 2026-09-22 125e2b3 | 737 or A330 on the ramp: "cabin crew?", then a cabin call | cabin (route call): call, reply | the reply is "Go ahead, flight deck." | not exercised |
| `cabin_voice_picker` The panel's cabin voice picker | 2026-09-22 125e2b3 | reload the plugin with status/auto empty, restart xvl, pick a cabin voice | Health has no missing xcopilot/opt/cabin_voice; the new voice checks in | "Flight deck, cabin crew. New voice, checked in." in the picked voice (by hand) | unflown (by hand) |
| `voice_handover` "You have control" is a hand-over from the flight model, the recovery only for an upset | 2026-09-25 ee493cc | any phase: "you have control", then "I'll take it" | handover decide: took, why, phase, upset; recover | no upset recovery runs after a hand-over without an upset | not exercised |

### Hand-flown session (rapport, tempo, say again)

| item | shipped | what to fly (airframe, field, action) | blackbox evidence (kind, event, fields) | pass criterion | status |
|---|---|---|---|---|---|
| `ils_setup` "Set up the ILS" as a pilot monitoring: tuned, APPR from off, needles called, missed altitude | 2026-09-25 5d5bf46 | 737 or 172 hand-flown arrival: "set up the ILS", then "couple the approach" | ils setup: rwy, freq_mhz, dh_set, arm, verdict; ils appr_pressed; ils watch: said; ils missed_alt_set: alt | a callout, APPR press or missed altitude follows every setup within 10 min | not exercised |
| `say_again_repeat` The frequency's "say again" repeats the controller's last line | 2026-09-23 f1d6791 | any towered field: key over a clearance, then "say again" on the radio | atc (outcome say_again_repeat): reply, age_s, service | the repeat is of a line under 60 s old | not exercised |
| `recall_cut_lines` "What were you saying?" says the lines a press cut | 2026-09-23 f1d6791 | any: key the intercom over a copilot line, then ask | said records around the press; the xvl log's recall line | the cut line is said again (by hand) | unflown (by hand) |
| `tempo_persona` Reaction pause, "stand by", line pace and small talk | 2026-09-23 f1d6791 | any hand-flown session with some chat | said: text, prio (no tempo event is recorded) | the pilot says it sounds like a person (by hand) | unflown (by hand) |
| `crew_speech_trims` Fewer pre-announcements, narration perishes at 6 s, one take-back hint a session | 2026-09-26 327d121 | any full auto from cold | said lines against the xvl log's "dropped stale" count | no backlog of stale lines (about 20 dropped before) (by hand) | unflown (by hand) |
| `mountain_map_handover` A mountain field's MAP approach is flown to the MAP and handed over with the talk-down | 2026-09-24 858a0d4 | LSZS RNAV 03 in the SR22 or SF50, full auto | procedure map_config, visual_segment; autoland glidepath | handed over at the MAP on a flyable visual segment, the talk-down heard (by hand) | unflown (by hand) |

## Retiring a row

When a row has held `verified` on a sortie flown for it, delete it from
`ITEMS` and from this file in the same commit, and say which sortie retired
it. A `FAILED` row stays until a fix ships; the fix gets a new row (its own
commit as `since`), so the old evidence cannot pass the new code.
