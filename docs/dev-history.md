# XVoiceLayer engineering history

This is the full pre-trim CLAUDE.md (archived 2026-09-05): every sortie-by-sortie diagnosis, date, and piece of evidence behind the rules that CLAUDE.md now states tersely. Grep this file before reworking any subsystem; the stories here explain WHY each invariant exists and what it cost to learn.

---

## The chatter of one departure, LXGB to GMTN, 2026-09-15

The same sortie's radio and intercom, read line by line after the push
was fixed. What the transcript showed and what changed:

- **The handoff readback went to the wrong controller.** Tower said
  "contact Gibraltar Departure on one three zero decimal four" at 600 ft;
  the crew's readback was queued at 1,000 ft, but the main loop SPEAKS a
  crew call (blocking, waiting for the frequency to go quiet) before it
  routes it, and a party-line exchange plus a traffic advisory held it
  19 s. Full auto's fixed 4 s wait had retuned by then, so the readback
  was routed to DEPARTURE, which answered "say again". A crew call is now
  routed on the frequency it was keyed on, before it is spoken; the
  audible order is unchanged.
- **"Electrical set. Generators and bleeds on."** on a 172. The line is
  "Electrical set." now; the start line already names the alternator and
  avionics, and the profile's command list is what was actually set.
- **"Taking the runway" twice** ninety seconds apart: a backtrack is two
  executions of the line-up tool and each was announced. Once, now; the
  tool's replies name the legs ("Backtracking runway two seven for the
  full length", "Lining us up, runway two seven").
- **"My takeoff" twice in one breath**: full auto's "Cleared to go" line
  said it and the takeoff tool's first line says it. Dropped from the
  first.
- **"about 0 hours 24 minutes enroute"** and **"You're at 49, 4951 to
  climb to cruise"** in the brief on the ground. Minutes alone under an
  hour; no altitude line at all on the ground, and "feet" on it in the air.
- **"Fly the flight man."** at 0.97 confidence was a dead end while the
  mission was down; "flag plan" and "flag then" the day before. Three
  whole-phrase mishears in `PHRASE_MISHEARS`, with the live transcripts as
  the test.

The rest of the list, done the same afternoon:

- **The departure clearance is read back.** The tower SUGGESTS the words
  (`Tower::clearance_readback`: "Not above two thousand feet, QNH one zero
  two two, squawk four five zero three, callsign"), enforces nothing
  (ICAO's mandatory list stops at the runway), publishes them as the
  `pending_readback` the coach shows a human pilot, and answers the echo
  with silence, clearing the suggestion. Full auto transmits it once the
  squawk is in (`clearance_readback` event), before asking for taxi.
- **Departure clears the climb it was told about.** The check-in's
  "climbing five thousand" is parsed (`climb_announced`) and the first
  reply is "radar contact, climb five thousand"; a check-in with no climb
  gets plain radar contact as before.
- **The check-in names the altitude being passed as it is SPOKEN.** The
  crew radio queue carries a `CrewCall::CheckIn`, rendered by the main
  loop after a bounded wait for the frequency to go quiet, from the
  altimeter at that moment (`Voice::radio_on_air`). The blackbox keeps
  `alt_ft_queued` for the comparison.
- **Three lines at a thousand feet, not four.** The mixture manager is
  armed silently; its own line when it first leans says what it did.
- **In calm wind the runway in use follows the arrival stream**
  (`arrival_end` over the TCAS picture: the threshold the nearest
  descending arrival is beyond, inside the same corridor as the release
  watch; `pick_runway_with`). Only with no runway in use yet and under
  5 kt; a real wind and a held pick both outrank it.

## The push was called done mid-swing, and the taxi drove against the tug, 2026-09-15

LXGB stand 1, "fly the flight plan" from cold. The plan asked the tug for
40 m back and a 177-degree turn-out (a Nose 90 left, which delivers ninety;
the rest is the taxi's). The sim's tug ran 45 m STRAIGHT before it began to
turn, so by the time the nose was moving the aircraft was already past the
plan's 40 m plus the 25 m chase margin, and at 65.3 m `done` fired on
distance with the nose still 140 degrees off and swinging at three degrees
a second. "Stop" went to the tug; a tug on the bar finishes the pattern it
was given whatever it is told, so it kept turning, the bar stayed on
through the whole 30 s clear wait (`tug_not_clear`), and full auto said
"rolling out" and engaged the taxi anyway. The drive ticks show `tug: true`
on every one: 34 s at half a knot with the heading error unwinding 82 to 3
as the TUG turned us, until the 20 s stagnation clock called it stuck and
the mission stood down `taxi_incomplete`. Forty seconds later the tug let
go by itself; the restart found the aircraft off the stand and taxied out
under its own steam without trouble.

Two rules:

- **A swing in progress outranks the distance test**
  (`pushback::swing_in_progress`, `SWING_DEG` over `SWING_WINDOW_S`): while
  the bar is on and the heading error moved two degrees in the last two
  seconds, `done` waits (blackboxed once as `swing_wait`); the push then
  ends at the tug's OWN stop through the stall path and `push_delivered`,
  as every completed pattern does. The test carries the sortie's numbers.
- **A tug on the bar owns the aircraft** (`autotaxi`, `TUG_WAIT_SECS`):
  the drive reads `pushback_attached` every tick (it already did, for the
  log), and while it is set it parks the governor at zero, sends no steer
  heartbeat, does not run the stagnation clock, says once "the tug's still
  on the nose", and rolls when the bar comes off (`tug_hold` /
  `tug_released`); three minutes without a release stands it down with the
  brakes set. The taxi never again mistakes being pushed for driving.

## The flare lead never flew: two dataref names the sim does not have, 2026-09-15

The LXGB 09 circuit landing on the 14th was the first sortie with the led
flare dial (`vs_dial_fpm`), and it arrived at 339 fpm, the sink growing from
271 to 339 in the last second, exactly the throttle-cut signature the lead
was written against. The tick log says why: on all 288 ticks of the
approach `vvi`, `vs_dial` and `cmd_vs` are the same number to twelve
decimals. Autoland read its VSI from
`sim/cockpit2/gauges/indicators/vvi_dial_fpm`, a name X-Plane 12 does not
have (`DataRefs.txt` has `vvi_fpm_pilot` and `vh_ind_fpm`); the read failed
on every tick since 330fbc7 and `unwrap_or(target)` handed the lead law the
command as its measurement, so it saw zero error and led nothing. The same
dead read is the VS watchdog's input (`|vvi - target| > 700` for 5 s), so
the LEMI-style "autopilot is not holding the descent" warning could never
have fired either. A second instance, same class: the stall recovery's
`compass_heading_deg_magnetic` (real name `compass_heading_deg_mag`)
failed into `unwrap_or(0.0)`, so every recovery bugged the aircraft north
and then said "holding heading 0".

Three rules from it:

- **The shared names are the names**: both readers now take
  `datarefs::VVI_FPM` / `datarefs::COMPASS_HEADING_DEG` rather than a local
  spelling.
- **A failed read is NO measurement, never the command in disguise**: the
  autoland tick keeps `vvi` as an `Option`, the dial gets the bare target
  and the watchdog skips the tick when it is `None`, the blackbox writes
  `null` (and now `arresting`), and the loop warns once per approach. The
  recovery skips the bug write and says "holding the dialled heading".
- **The Rust constants are probed too**: `aircraft::real_install_names`
  (`cargo test -- --ignored real_install_names`) scans every `"sim/..."`
  literal in the workspace against the install's `DataRefs.txt` and
  `Commands.txt`. The live profile probe covers the YAML; this covers the
  code. It found exactly these two on first run.

Still to fly: the led dial itself, now that it reads a VSI.

## The instructor teaches from cold and dark, 2026-09-14

"Can the instructor help me taxi?" and "shouldn't the instructor be able to
teach from cold dark?" Both no, the day after it was built: the lessons
were air work, and the ground half of a flight is where a student needs
the most talking-through. The pieces existed (Instruct checklists, the
taxi guide's cues, the coach) but nothing chained them.

**Two ground tasks.** `Task::Taxi` grades PACE, a fifth axis
(`Axis::Pace`, its own words: "Pace, 5 over. Ease the power back and hold
it with the brakes."): the profile's straight-leg `taxi.kts` (15 when
silent), over by more than 3 kt is the fault, slow never is; judged only
while moving; complete after 10 s stopped once 150 m are behind it
("Stopped. Holding short, I take it."), FAILED outright by rolling through
the hold line at more than 30 kt. `Task::Takeoff` waits for the roll (15 kt
on the wheels), takes the heading THEN as the centreline (the aircraft at
the hold line points anywhere, so the brief's heading is a placeholder the
scorer retargets), judges it from 25 kt to 1,000 ft AGL, notes the rotation
against Vr from the last sample on the wheels ("Rotated at 54 knots, Vr is
55."; 10 under is early and a fail, 15 over a note), and Vy +10/-5 from
200 ft to the top. Vr rides the targets, not a lookup: the first cut read
it from the lesson statics inside a brief that already held the lock.

**The syllabus.** "Teach me a full flight" / "a lesson from cold and dark"
walks `STEPS`: the ramp lists (preflight, before start, engine start) in
the Instruct flavour by section key, each skipped aloud when the file
lacks it ("No preflight list on this airframe. Do it from memory and carry
on."); the engine waited for, nagged once a minute; before taxi; the taxi
graded; before takeoff; the takeoff graded; then "Your flight" with the
tracker's phase lists run as Instruct at once (`flow::phase_boundary`
checks `syllabus_active`, no proposal to answer); the landing armed on
final once the flight has been above 1,000 ft (never on the takeoff's own
climb-out); 20 s stopped; the shutdown list; "That's the flight" with one
verdict per graded task and the lists done. Engine already running starts
at before taxi; airborne is refused. A retarget during a syllabus task
lands on that task's scorer (a "12 knots" during the taxi moves the pace).

**The flow behind a seam.** The syllabus needs to find, start and watch
checklist flows from the monitor's tick, and the tests need none of that:
`SyllabusIo` (find_list / start_list / list_running), `FlowIo` in the
monitor over `flow::find_section` / `flow::start` / `flow::ACTIVE`, a fake
in the tests that "runs" each list for two ticks. The whole cold-and-dark
flight is one plain test, ramp to shutdown, 250 ticks.

Not done: centreline tracking on the taxi (the guide's cross-track is the
obvious axis), the run-up as a graded item, and the first in-sim run of
the whole thing.

## The instructor: a mode, a scorer, a lesson, 2026-09-13

"Have we built instructor features yet?" Partly: the checklist flow's
Instruct flavour, the monitor's callouts and the ATC coach were the pieces;
the persona, the mode that keeps its hands off, the ACS scorer and the
lesson were the CLAUDE.md sentence "if built". Built today, entirely
deterministic, tested as plain `cargo test`.

**A mode, because hands-off must be structural.** `mode::Instructor`
(panel index 3, "Instructor (teaches and grades, never touches)") keeps the
persona (`copilot_enabled` stays true: callouts, coach, checklist
read-and-watch, lessons) and drops the hands: `mode::crew_actuates()` is
false there and in ATC-only, and every crew writer that is not a spoken
tool checks it (the monitor's `fix_action`s, `cockpit_auto`, `flow::start`,
where the Crew flavour becomes Instruct, and undo). The spoken tools are
refused at the ONE door in the main loop's dispatch, before any confirm
prompt, with the action named from `callout_label` ("Flaps 1: your hands")
and the full "I don't touch the controls" line once per spell in the mode.
Entering the mode runs `stand_down_for_aircraft_change` (a lesson and a
loop cannot share the aircraft); leaving it abandons the lesson. A checklist
asked for by title in instructor mode runs Instruct without a special case
at the call site.

**The instructor assigns; the scorer never guesses.** The hard problem of
grading a hand-flown task is knowing what the pilot meant. The ACS answer is
that the examiner specifies, so `Task::assign` does: straight and level at
the altitude rounded to 100, the heading to 10, the profile's cruise IAS; a
climb to the next 500 at least 1,000 up at Vy; a descent to the 500 at least
1,000 down but never below ground plus 1,500; steep turns at 45 (commercial
50) at or below Va; slow flight at the airframe's speed floor; the pattern
at field plus `pattern_alt_agl_ft` at the circuit's pattern speed; the
landing at the approach speed. "Begin" re-assigns from where the aircraft
IS (a brief at 3,000 and a begin at 3,500 would otherwise grade the wrong
altitude for the whole task). An AP-shaped command during a lesson
("heading two seven zero") is refused as a write and taken as a retarget.

**Narration with a sustain.** An axis outside its tolerance for 3 s is
spoken once with the correction ("Altitude, 120 low. Nose up a touch."),
the hint only the first time; the recovery is acknowledged; a standing
excursion is re-said at 20 s. Advisory priority, so the radio wins. The
task completes on its own milestone: the two-minute clock, the level-off
held 5 s within 300 ft of the assignment (a level-off elsewhere, 20 s, ends
the task with "the assignment was X" and fails it), both roll-outs (bank
under 10 after over 30; the roll-out heading is judged with one second of
weight, the bank only inside the turn, or the roll-out itself read as 45
degrees shallow), the touchdown (sink from the last airborne sample, drift
from heading minus track).

**The pass rule.** An axis passes if it was never more than 3 s outside in
total and never past twice the tolerance; a firm (>300 fpm) or drifting
(>5 degrees) touchdown or a wrong level-off fails the task whatever the
axes say. Examiners forgive a corrected bump; they do not forgive 250 ft.

**The same scorer runs offline.** `instructor::grade_flight` in `xvl
debrief` takes the blackbox `state` records of any flight, classifies
samples (ground, turn where bank is recorded, climb, descent, level),
merges flickers under 8 s, then cuts every run into STRAIGHT sub-runs at
turns (20 degrees off the run's reference; the turn skipped while
consecutive samples swing faster than 1 degree/s), because the first pass
graded an 18-minute cross-country cruise leg as "heading 180 degrees right
(FAIL)". Targets are what the pilot settled on: the circular mean heading
and the median speed (the entry speed of a level run is still the climb
speed: "15 knots slow" until the median), the entry altitude rounded to
100, the level-off altitude for a climb. Descents are trimmed at 800 ft AGL
so the final belongs to the landing grade. On the LEMG-LEJR full auto sortie
it reads as an examiner would: the follower's cruise within 55 ft, the
climb 21 kt over Vy (a cruise climb), the autoland's 302 fpm arrival "firm"
and 8 kt fast on final. The `state` record now carries `bank` and `track`.

**Tests write the live blackbox.** The lesson test's events landed in
`logs/blackbox.jsonl` as "flight 117" (22 lines, stripped); the flow tests'
"declined" was already there. `XCOPILOT_BLACKBOX=off cargo test` from now
on; the rule is in CLAUDE.md.

Not done: a demonstration flavour ("show me a steep turn" with the
automations flying it), the pattern task offline (its altitude window needs
the field), handbook questions (the model-shaped use, still out), a voice of
its own (the instructor speaks with the crew voice). Not flown yet: the
first in-sim lesson will say whether 3 s is the right sustain and whether
the roll-out threshold of 10 degrees is too tight for a hand-flown 172.

## The plan flew into the Sierra, 2026-09-13

LEMG to LEJR, 172SP, full auto, the pilot riding along and asking questions.
The follower armed the leg to JRZ at 55.9 nm with the plan's 2,700 ft
constraint, and started down immediately from 8,000. Forty minutes later the
aircraft was over the Sierra de Grazalema at 220 ft AGL, the "Before landing"
checklist had run itself 44 nm out (the monitor's below-1,500-AGL trigger,
firing on terrain rather than on an approach), and the pilot said "We're going
to crash", then "Climb to four thousand feet", then, forty-seven seconds
later, "Climb to four thousand feet now." The blackbox shows every leg event
in between carrying `want_alt: 2700.0`. A terrain probe along the track at
that moment: 3,861 ft at 5.1 nm, 3,803 ft at 6.7 nm. The aircraft was
oscillating between 2,715 and 2,860 ft, which is the signature of two writers
on one dial.

Three faults, in the order they bit, each one now a rule.

**The top-of-descent deferral worked for exactly one tick per leg.**
`descent_target_ft` defers a descent until the top of descent and otherwise
returns `carry`, the previously commanded altitude, which is the whole point
of the Gibraltar fix of 2026-09-11 (18 nm flown level at 1,400 ft across the
Strait). But the caller read `carry_alt` and then immediately overwrote it
with the new leg's own constraint, so from the second tick onward the function
compared the constraint with itself and returned it. The logs always showed
one honest `want 8000.0` followed by the dive, and nobody read the pair as a
symptom. The last COMMANDED altitude and the last leg CONSTRAINT are two
different quantities and are now two variables (`prev_cmd_alt`, `carry_alt`).
Tested by ticking the real bookkeeping forward twenty times, not by calling
`descent_target_ft` once, which always passed.

**A plan's altitude is only as good as the ground under it.** The local
planner floors its own cruise altitude over the plugin's terrain probe, but a
constraint handed down from a plan (SimBrief's, or the local planner's own
destination leg) was flown as given. The follower now probes 16 points along
the track it is steering, out to 15 nm or the fix, every 10 s, and never
commands an altitude below the highest ground found plus 1,000 ft. It says
"Terrain ahead, holding X feet" once per rise and blackboxes `terrain_floor`.
Skipped on an approach, whose published platform altitudes own the descent
below the enroute floor. An old plugin with no probe channel disables the
look-ahead for the flight after one failed answer rather than stalling the
tick for two seconds in ten.

**The pilot's own override was the thing the automation was strongest
against.** The follower rewrites `altitude_dial_ft` every tick; the autopilot
tool wrote the dial and spawned a capture watcher whose documented stand-down
condition is "the dial moved off our target". So the follower undid the climb
within 500 ms and the watcher retired itself, twice, and there was no way for
the pilot to win by speaking. A commanded altitude is now HANDED to the
follower (`procedure::take_altitude`) instead of written: one writer, the
follower's VS law flies the change, and the pilot's number outranks the plan's
until they say another one. Deliberately sticky, with no self-expiry, because
an override that quietly lapses is this exact accident. `level_off` hands over
the same way and touches neither the wings nor the throttle, which the lateral
law and the governor own. The panel says whose altitude is being flown.

And the gate that let 4,000 ft through: `reject_unsafe_target` judged a target
against the ground UNDER the aircraft, which was 1,700 ft while a 3,861 ft
ridge sat five miles ahead. It now consults the follower's look-ahead floor,
and a CLIMB to a target that still doesn't clear it is RAISED to the floor and
spoken, not refused: the pilot asking to go up is already reacting to the
terrain, and "negative" leaves the aircraft where it is. A descent below the
floor is still refused, with the number that would work.

---

## The LLM retired, and a night of rules — 2026-09-13

"Is the LLM not worth having?" The answer-only Ollama fallback (qwen3:8b,
thinking off, off by default since July) was switched on from the panel and
tested live on the LEMG sortie the evening before: four utterances reached
it in about a second each, three were correctly refused by its own prompt
("Say again?" for a statement, chatter and an opinion question), the one
real question came back as a three-word fragment, the cold load took 96 s,
and Ollama generating on Metal is the one measured cause of the sim dropping
to 20 to 30 fps. Everything worth answering that night had been answered
before the model was consulted. Decision, the pilot's: "the better approach
is to augment our deterministic code", and "wipe out LLM config etc".

Removed: `llm.rs` and `rag.rs` (the RAG index's only consumer was the answer
path, and its embeddings came from Ollama), the `index` and `ask`
subcommands, the config knobs (`XCOPILOT_LLM`, `XCOPILOT_MODEL`,
`OLLAMA_BASE_URL`, keepalive, GPU layers), the settings-registry knob, the
Health screen's Ollama probe and LLM line, the plugin's `xcopilot/llm_enabled`
dataref and its Settings checkbox, the warmup on the mode edge, the
`ToolSpec` plumbing and each checklist's model-facing description. The
`ToolCall` type moved to `tools`. The correction path ("negative, 500") now
re-parses through the grammar instead of re-asking a model. `xvl analyze`
counts handled against missed and lists the most frequent missed phrasings:
the ratchet. The `indexes/` files stay on disk, untracked, for the day an
instructor answers handbook questions.

The rules from the same log, each with the transcript as its test: the 172
profile gets a `locations:` map (the fuel pump was asked for three times;
the Laminar switch row and the pedestal, from the real aircraft); "what's
the squawk code" reads the code and mode; "where are we" gives a position
report (altitude, heading, groundspeed, the fix being flown, the range and
bearing to the destination) instead of an echo of the question; "please take
over" is the hand-over "take control" already was, with the same direction
test; the echo memory no longer remembers the copilot's own coached phrase
("Say navigate to follow the plan" was repeated by the pilot and rejected as
echo, without a word); the instructor flavour performs an item on "can you
help me with that item" / "you do it" / "do it for me"; the climb list
triggers at 1,200 ft, after the lights rule has put the landing light out at
1,000; and autoland lights the landing light at the final, which the BEFORE
LANDING cross-check had honestly flagged as not set. Still open from that
log: "navigate" on the 172 reports "LNAV didn't engage" (needs the sim to
probe the KAP 140's status datarefs), and "fly the flight plan" airborne is
refused by design while the follower has no airborne-join.

## "Where are we going?" at a thousand feet — 2026-09-12

Same evening, airborne after a restart: "Where are we going?" got "We
don't have a taxi clearance yet. Call the tower and request taxi." The
taxi-guide question gate (`is_taxi_query`) ran before the flight-plan
question path and claimed the words regardless of height. It now applies
on the ground only (a failed read keeps the ground reading); airborne the
same words reach the plan path, which names the destination.

## The squawk that arrived 13 seconds late — 2026-09-12

"I think the copilot is not setting transponder code." LEMG to LERL under
full auto, 23:06Z. The blackbox has `request_clearance` at :09 and the
tower's clearance with squawk 4503 at :52, but the crew's call only left at
:45: it rides the speech queue, behind the seventeen-second arming brief
and a Ryanair the tower was working. The squawk wait was 30 s from the
QUEUEING, so at :39 full auto said "Tuning the tower, requesting taxi" and
moved on, and the clearance that came back 13 s later had nobody waiting
for it. The transponder showed 7000 all the way to the climb, where the
departure controller asked twice to "confirm squawk four five zero three".

The taxi request learned this in July (25 s, then 90 s, each beaten by a
second; now 180 s). The squawk gets the same treatment in two halves: the
wait is 60 s, and the squawk is OWED rather than missed. `ensure_squawk`
(the pure `squawk_owed` decides: assigned, and the transponder does not
show it) runs after the wait, at the holding point before the departure
call, and lined up; a late code is dialled and announced wherever it is
first seen, blackboxed `squawk_set` with what the transponder showed.

## Checklist flows: the list at its phase, and an instructor's first lesson — 2026-09-12

"When we fly, copilot should be following checklists; it should also be
capable as an instructor to tell me what to do." Both gaps had the same
missing piece. The XChecklist interpreter (`checklist.rs`) knew every item's
words, pass test and action, but was reached only from a spoken request and
ran INLINE in the command loop, listening for its own confirms with
`listen_once`; nothing read CLIMB, CRUISE, DESCENT, BEFORE LANDING or AFTER
LANDING when the flight got there, full auto flew its own hard-coded
configuration (the 2026-09-12 mixture entry parked "reading the lists at
the phase boundaries" for its own ask), and the "instructor" was a note in
CLAUDE.md: a third persona, mode-gated, never actuating.

`flow.rs`. One runner on its own thread (the automations' pattern: a
`std::thread` with its own current-thread runtime, since the main runtime
blocks on the mic), speaking through the crew voice (`Voice::say_crew`:
blocking like `say`, crew-gated and panel-mirrored like `alert`), replies
taken from the command loop through `flow::take_reply` with a CLOSED
vocabulary. Three flavours of the same list. **Crew**: the old runner's
behaviour, plus a read BEFORE the write (an item already in state is
confirmed, not re-written under the pilot's hand) and `held_by_automation`
(the governor's throttle, the manager's mixture, autoland's flaps, the
flying loops' AP are named as held, never written: one writer per dataref).
**Monitor**: under an automation the copilot IS the pilot flying, so the
list is a read-only cross-check: narration skipped, each item confirmed
from its dataref, a discrepancy spoken ("Autopilot, off. Not set.") and
blackboxed, never a write. The mission's laws act; the list audits them,
and the debrief now prints the audit. **Instruct**: never writes by
construction; every item is an instruction, then a 500 ms watch on the
item's dataref for the pilot to satisfy it ("Good."), advanced by
"next"/"set"/"checked", skipped by "skip", the clock restarted by "hold
on", timed out at 45 s ("Still not done. Moving on."). To make the read and
the watch possible `Item::Act` now carries the file's dataref, index and
condition alongside the tool call.

Who starts one. The monitor's `flow::Tracker` (pure, tested with a whole
flight and a pattern): climb at 800 ft AGL climbing, cruise after 30 s
level above 2,000 ft AGL, descent after 20 s down from a flight that
reached cruise height, before landing after 10 s descending below 1,500 ft,
after landing after 10 s under 30 kt on the ground; each once per flight,
re-armed by the next takeoff roll, so a circuit at 1,000 ft earns climb and
before-landing only. At a boundary `phase_boundary` starts the Monitor
flavour if full auto, the follower, the circuit or autoland is flying;
otherwise the copilot PROPOSES ("Climb checklist when you're ready?") and
the command loop's `proposal_reply` takes "go ahead"/"yes"/"ready" (Crew),
"talk me through it" (Instruct) or "not now"/"later" (decline), 120 s TTL,
never acting on its own. By voice: "descent checklist" and the LLM's
checklist pick start Crew; "talk / walk / guide / coach me through the X
checklist" (`instruct_request`) starts Instruct, taken from the
conversation module's preview, which keeps "preview / explain the X
checklist" as the read-only recital. Sections resolve by the same
`match_request` a spoken title uses (the 172 and SR22 files match their own
words); a profile whose titles differ maps them in `checklist_flow:`; a
missing section proposes nothing and blackboxes `no_section`.

Retired: `run_checklist` and its `interrupt` flag (the PTT press is now how
the pilot ANSWERS a list, not how they stop it; the word "stop" stops it,
after the governor and pushback pre-gates so a "stop" on the ground still
stops the aircraft first). The panel: `flow::status()` ("checklist: climb,
3 of 8") in the managers, CANCEL picks "stop the checklist" when nothing is
flying. Blackbox kind `checklist` (start, proposed, declined, held,
discrepancy, not_done, complete, abort, no_section), in the debrief.

Not done: a generic list for the six airframes with no checklist file (the
Baron, DC-3, Otter, C90B, Evolution, RV-10 and every generic airframe hear
nothing at the boundaries); an instructor MODE that refuses the crew's
tools; the ACS scorer over the blackbox. Not flown yet: the first in-sim
sortie will say whether the tracker's thresholds and the Monitor flavour's
wording are right.

## Exterior lights by the book — 2026-09-12

"So for the SR22, landing lights = landing + taxi lights?" Yes, and "ok
let's do it" was the three stages still owed plus the night gate. One rule
set in `tools::lights` now, on a process-wide `Exterior` (landing and taxi
`LightProfile`s, republished on every profile swap like the transponder
knob) and a shared dusk test (`cockpit_auto::dusk_or_dark`, the sun below
the 5 degree day threshold; a failed read counts as dark, lights on being
the side to err on):

- taxi: `taxi_light(dark)`: the taxi light, else after dusk the landing
  light, else none. The SR22 taxis dark by day now, lit at night.
- runway spans: `Route.runway_spans` from the cleared route's own segment
  flags (crossings, backtracks, a line-up's last leg), merged where
  contiguous; the drive lights the landing light 40 m ahead of each and
  drops it clear of it, blackboxed `crossing_lights`. Not when the landing
  lamp is already the taxi light tonight (`crossing_light`).
- climb: off passing 1,000 ft AGL, in the enroute radio wait
  (`lights_climb`).
- vacate: `after_vacate`: taxi on and landing off where both exist; a
  lone landing lamp stays on after dusk and goes off by day
  (`lights_vacated`).

Pure decisions tested; the writes ride `lights::apply` like every other
light write. Not flown yet: the sim was closed for the evening.

## The taxi light rode the whole flight — 2026-09-14

"It says landing lights, off. But I don't think it actually turned landing
lights off?" On the 172, LEJR-LXGB under full auto. It had: the sim read
`landing_lights_on` = 0 and every `landing_lights_switch` element 0, and
the panel screenshot showed LAND down. TAXI was up. `lights_taxi` lit it
for the taxi, line-up added strobes and landing, `lights_climb` put the
landing light out at 1,006 ft AGL, and nothing in the mission ever touched
the taxi light again: the 172's two lamps sit side by side in the left
wing, so a burning taxi light IS the landing light to anyone looking at
the aeroplane. The climb checklist (Monitor flavour) then confirmed
"Landing lights, off" honestly, because that item tests the landing
switch. Fix: `lights::after_climb_for` (every ground light the airframe
declares goes off, tested like `after_vacate_for`), applied in the same
stage; the `lights_climb` event now carries `off: {landing, taxi}`. While
there: `after_vacate` and `landing` reported `apply(..).is_ok() && on`, so
an "off" could never blackbox `true` and a wrong readback still counted as
done. Both now compare the state read back against the intent (`reached`).

## The SR22 has no taxi light — 2026-09-12

"When taxi should we use landing lights?" Not on a taxiway; the taxi light
is for the taxi and the landing lights go on lined up, which is what full
auto did. Except: "SR22 does not have a taxi light switch." True, the
profile declares nav, strobe and landing only (the .acf has one landing
lite), so `light("taxi")` returned false and the SR22 taxied dark, silently.
Full auto now uses the landing light for the taxi on any airframe that
declares no taxi light, which is how a Cirrus is taxied at night, and the
stand shutdown already turns it off. Still owed, all airframes: landing
lights off and taxi light on at the vacate, landing lights for a runway
crossing on the taxi route, and off in the climb.

## A tug from the middle of a taxiway — 2026-09-12

"It seems we are using the tug sometimes without reason." Four full-auto
restarts on the SR22, each from wherever the last taxi had stood down
(mid-Bravo, 25 m off route), each one "We're nose in. Calling for
pushback." and a two-to-three-minute wait for a tug to push the aircraft
back down a taxiway. The trigger (`pushback::plan`) asks one question:
is the route's first node more than 100 degrees off the nose? After an
off-route stop it usually is, because the tower re-plans from the nearest
node, which is behind. The trigger never asked whether we were on a
stand.

Now it does. `party_line::nearest_stand` reads the loaded taxi network's
apt.dat 1300 rows (any kind: gate, tie-down, misc), and
`pushback::on_a_stand` says yes only within 35 m of one with the nose
within 60 degrees of its parking heading; a field with no stand rows keeps
the old rule. Off a stand with the way out behind, full auto says so
("not on a stand, so no tug, I'll turn us round under our own steam"),
blackboxes `pushback_skipped`, and the taxi join does the turn, which it
was built for. The direction of a push still comes from the route, never
from the stand database.

## The SR22 weave — 2026-09-12

"Oscillating taxi on SR22": the LiveTraffic trail down LEMG's Bravo was a
+/-20 m sine, then a loop at the Golf turn and a 25 m off-route
stand-down. The blackbox: brakes zero throughout (no `brake_steer`), full
steer held for forty seconds for about one degree per second of yaw, the
turn arriving late and overshooting on every cycle. The SR22 profile had
no `taxi:` block at all; it was steering a castoring nosewheel through
the rudder channel, the gap the SF50 profile's notes had already named as
"the same gap the SR22 carries". The SF50's block (differential brakes,
9 s aim, 2 s lead, 42 degree full-scale, 0.8/s slew) is carried over as a
first pass, to be judged on the next taxi.

## The tug that connected and disconnected at once — 2026-09-12

"The tug seems to connect and disconnect almost immediately, and then
reconnect. We are doing something wrong." We were. Every push in the
blackbox (five for five) reads the same: `hooked` after three solid
attached polls, the re-issue of the pattern the code sent "because the tug
ignores a command issued before it was on the bar", and the attach flag
false on the very next poll. The tug drives off, the twelve-second nudge
sends the command again to a tug that has already gone, and it comes back
about ninety seconds later to push. The 2026-07-27 premise behind the
re-issue ("nothing moved until the command was sent again, and the push
began a minute later") was this same cycle seen from the inside. The stall
re-issue had the same effect at the far end: `issued` #2 while attached,
`tug_finished` 0.6 s later, which is what the 2026-08-03 guard had already
half-noticed.

Rule: **a command to a tug on the bar drops it off the bar.** One command
per push, at the summon (it carries the pattern); the tug pushes when the
brakes come off. No re-issue at hook-up (`on_bar` event instead), the nudge
only re-summons a tug that hooked, let go without pushing and has been gone
30 s, and a tug that stops short is stopped and judged with
`push_delivered` (`short` if not) rather than asked for a second pattern
that it answers by leaving. `MAX_ISSUES` is gone.

And on the same push: "Skip pushback." and "Cancel pushback, just fly."
both parsed as a fresh straight push, two more tug requests for the pilot
refusing one. The grammar's refusal words (cancel, skip, abort, no,
without, forget, don't, never) are all "stop"; under a steered push the
tool sets `pushback::SKIP`, the loop stops the tug, sets the brakes and
returns `skipped`, and full auto says "Skipping the push. I'll turn us
round under our own steam." The bare "stop" word still aborts the whole
mission, as before. The push loop's `tow_in_progress` note stands: it
reads 0 throughout.

## The Flight tab says what the copilot is doing — 2026-09-12

"Could the Flight tab be more informative of what the copilot is doing?"
Under full auto enroute it read "FULL AUTO FLIGHT | enroute / flying a
procedure / speed 95 kt / mixture managed": every sub-manager a bare name,
nothing about which fix, how far, what altitude it was aiming for, or what
a wait was waiting on.

Now each loop publishes a detail line from the numbers it already has that
tick (no new sim reads), through `auto_status::Detail`, folded into its
`status()` as "name: detail" and cleared by a `DetailGuard` held for the
loop's whole run so no exit path leaves it stale: the follower "to PIMOS,
12.3 nm, 6 min, climbing to 5,500 ft" (`leg_detail`); the circuit
"downwind runway 27, 1,200 ft to 1,500, 75 kt"; the taxi "340 m to go";
the takeoff "rolling, 45 kt, rotate at 55 kt" / "rotating" / "climbing,
250 ft, handoff at 300 ft"; the governor "speed 95 kt (at 93)"; and full
auto's waits ("waiting for the squawk", "the landing traffic to pass",
"the destination's frequency"), a `waiting()` call before each bounded
wait, cleared by the next stage event. Two panel-only lines come last and
are never in the spoken status: "ap: hdg 084, alt 5,500, vs +500" from
whichever loop drives the AP, and "said: Next, PIMOS." from the voice
arbiter. Everything is rounded to what a pilot would dial (tenths of a
mile, whole minutes, 10 ft, 5 kt, 50 fpm) so the change-gated publish is
not re-sent for jitter. The plugin renders the extra lines as it always
has (dim, wrapped); no plugin change, so the Health tab and "what are you
doing?" improve for free.

## The released tug that came back — 2026-09-12

LEMG stand, full auto. The plan needed 138 degrees of swing; one "Nose
90" delivered 99 and the tug let go at 40.6 off, judged not delivered,
and the fallback taxi started from there. The stall clock had issued a
SECOND pattern 0.6 s before the release branch saw the bar off (the
2026-08-03 guard covers the tick AFTER the bar is seen off, not the one
before). Seventeen seconds into the taxi the aircraft, at 5 kt with full
right steer commanded, stopped turning: heading held at 260 for ten
seconds against a +100 degree error, then ran 25 m off the route and
full auto gave up "taxi incomplete". Wind calm, yield latches clear,
plugin steer path symmetric. The one thing left pending was that second
pushback request, which a tug standing at the nose answers in seconds,
and X-Plane owns the nosewheel while it is on the bar. The early-return
path never sent `pushback_stop`; it does now (the completion path always
did). Evidence for next time: the taxi tick blackboxes `tug`
(`pushback_attached`, a slow dataref) so the record can say whether the
tug was on. And a stalled-out give-up holds the brakes at the governor
until the pilot speaks again; that is by design ("a stage that is not
moving the aircraft is holding it").

## The follower sequences the FMS — 2026-09-12

"Will the copilot make use of GPS now with a flight plan?" No: the follower
loads the plan into the sim's FMS so the GPS and CDU display it, then flies
its own bearing-to-fix on the heading bug and never engages GPS/NAV mode.
Nothing sequenced the FMS, so the GPS page, the G1000 readouts and every
`gps_*` dataref described entry 1 from LXGB to LEMG while the aircraft was
five fixes on; the enroute-questions fix that morning stopped TRUSTING the
GPS (`procedure::active_leg()` first) without making it true.

Now it is true. `xcopilot/fms/goto` (`plugin/src/fms.cpp`) takes
`IDENT,lat,lon`, finds the entry among the LOADED plan's entries (whoever
loaded it; `XPLMCountFMSEntries`/`XPLMGetFMSEntryInfo`): an ident match
wins, distance breaks ties between namesakes, a nameless payload or an
unmatched ident falls back to the nearest entry within a mile, and no match
changes nothing (approach legs are not in the enroute plan and must not
drag the destination around). The hit becomes the destination on both the
legacy GPS plan and the native FMS plan, as the load does. The follower
(`procedure::sequence_fms`) writes it once per leg, the first tick it
steers at that leg, blackboxed `fms_goto`. Steering is unchanged: heading
bug, VS dial, altitude preselect. Flying ON the GPS (NAV mode, the sim's
own CDI, per-airframe HSI source knobs) stays a separate, bigger job.

Installed by remove-and-copy under a running full-auto flight; the plugin
is not reloaded while `xcopilot/status/auto` is non-empty, so the first
flight on it is the next sim start or an explicit reload.

## Full auto and the mixture — 2026-09-12

"When copilot flies flight plan it's not following checklists? And it is
not leaning mixture after climbing?" Both true. The XChecklist interpreter
(`checklist.rs`) is reached only from a spoken request in `main.rs`; the
full-auto orchestrator never consults the CLIMB, CRUISE, DESCENT or BEFORE
LANDING lists, it flies its own hard-coded configuration (lights, squawk,
gear and flaps on final). And the whole mission had no mixture write after
the engine start: the enroute leg arms the flight speed hold (a throttle
law), `lean_mixture` and its continuous manager existed but only the
pilot's voice reached them, and autoland had no full-rich step, so a lever
the pilot had leaned by hand at cruise flew the go-around and the landing.

Fix, deterministic and through the existing tools. `SubTools.piston_mixture`
(the profile's generic `engine` group, deliberately NOT `piston_engine`,
which includes the DC-3's notched xlua lever the manager must not creep):
after the enroute follower starts, full auto arms `arm_mixture_manager`
and says so ("I have the mixture: full rich below three thousand, leaned
for cruise"); at `enroute_complete`, before the arrival is built, it calls
the new `engine::mixture_rich_for_landing` (releases the manager and any
seek, glides the lever to full rich, "Mixture full rich." only if it
moved) so a high field never inherits a leaned lever from the manager's
MSL floor.

Found on the way: the manager could not actually lean from a rich lever.
Its re-trim bracketed ±0.05 around the lever it found, so armed at full
rich (which is what `lean_mixture` below the floor does, promising to lean
once higher) it sampled 0.95 once, called that the peak, and parked at
0.98 for the cruise. `manager_seek_window` now returns the whole range in
0.03 steps from a lever at or above 0.90, the ±0.05 bracket in 0.015 steps
otherwise (tested). The seek loop also checks `MIX_MANAGED` each step so a
release mid-hunt (pilot, landing) leaves the lever where the releaser put
it instead of overwriting it with the seek's own answer a second later.
And the manager richens on the way out when it yields to autoland or a
recovery, and full auto's `give_up` clears it: "your aircraft" means every
lever. Blackbox: `full_auto` events `mixture_managed` and `mixture_rich`.

Not done: reading the checklist files aloud at the phase boundaries. That
is a different feature (step-by-step with confirmation on hazard items,
and the CRUISE list's other items are the pilot's), left for its own ask.

## The sim's own toe brakes on the copilot's rudder — 2026-09-12

"There is a lot of jerky braking when steering with the 172 during taxi."
The blackbox for the LXGB taxi (two drives, 557 m and 1114 m) showed the
governor's brake trace almost empty: a handful of 0.1 to 0.2 pulses at bend
caps and after post-turn overshoots, nothing that could be felt as "a lot".
What it DID show, in every corner: with the steer command past ~0.65 (the
172's nosewheel has 10 degrees, so that is 6.5 of them) the aircraft fell
from 6 kt to under 1 kt in four seconds with the yaw rate spiking to 11
deg/s and NO brake commanded, then crawled at 1 kt for ten seconds while the
throttle rebuilt from the bend cap's idle, then surged to 12 kt out of the
turn and earned a real brake pulse for the overshoot. At the join it sat
sawing 1.5 -> 0.9 kt every 1.2 s: `steer_authority` tapers the command with
groundspeed, so the command crossed some threshold, a brake bit, the speed
fell, the command shrank below it, the brake let go, and round again.

The threshold is X-Plane's. Since 11.30 the sim applies **automatic
differential toe brakes against hard rudder input** for a pilot with no
toe-brake axes assigned, per the aircraft's Plane Maker "brake power to
help with steering (if no rudder pedals present)" parameter (the 172's is
on; the developer notes: "X-Plane can apply the left toe brake for you when
you apply a lot of left rudder"). Our `xcopilot/taxi_steer` channel drives
`yoke_heading_ratio`, which is rudder input, and the rule that switches the
auto-brake off is "a plugin is controlling the toe brakes", i.e.
`override_toe_brakes`, which the plugin only raised while it was actually
braking. So every hard nosewheel command came with the sim's brake on the
inside wheel. This rig has no toe-brake axes (assignments 1/2/3/4/11/14/20/21
only), so the feature was live the whole time.

Fix, plugin only (`drefs.cpp`): the toe-brake override is held for the
WHOLE time the steer channel is applied, ratios zero unless a channel wants
brake. That would lock the pilot out of their brakes for a steered taxi, so
the yield mirrors the steer channel's: a brake COMMAND (`brakes_regular`,
`brakes_max`, both toggles, watched by a non-consuming before-handler) or a
toe-brake AXIS moving off its engage baseline, while the override is up for
steering alone, latches `xcopilot/brake_yield` and drops the override that
frame, so the sim's own brake handling runs, the ratios show the press, and
the governor's takeback detector sees it exactly as before. A channel that
WANTS brake keeps the override regardless (a governor holding a stop must
never lose its hold to a tap on the key). Re-armed when the steer channel
goes stale. The takeoff roll and the autoland rollout steer through the same
channel and inherit both halves.

Verification owed on the next taxi: `sim/flightmodel2/gear/tire_part_brake`
must read zero on every wheel while the copilot steers with no brake
commanded, and the corners should slow at the idle-coast rate the wide left
turn already showed (about 0.5 kt/s), not collapse. The bend-cap brake
pulses (0.1 to 0.2 for under a second, tuned down three times already) are
untouched and are the next thing to judge once the sim's brake is out of the
picture. Installing while the sim runs: remove the old `.xpl` and copy, never
overwrite in place (the running sim has it mapped).

## The panel on the sim's own renderer, and the field you are standing at — 2026-09-10

X-Plane 12.4.4 (SDK 4.4.0) ships a native panel-graphics API with an ImGui
helper: a window of content type PanelGraphics gets a draw callback in
which `XPLMDrawCalls` takes ImGui's vertex/index buffers as-is (20-byte
`ImDrawVert`, 16-bit indices, per-command scissor in window-local top-left
pixels) and `XPLMCreateTexture` takes the RGBA8 font atlas. The plugin's
panel moved onto it (`plugin/src/imgui_xp.cpp`, replacing the vendored
ImgWindow): no OpenGL, no GL-on-Metal bridge, no fixed-pipeline renderer,
and Dear ImGui went from 1.80 to 1.92.9b. Two things that were structural
limits are gone with it. The atlas used to bind ONE GL texture at the first
window's construction and could never rebake, which is why B612 was baked
at three sizes and UI scale was a font swap; 1.92's dynamic atlas plus the
texture-update protocol (`ImTextureStatus_WantCreate/WantUpdates/
WantDestroy`, answered by create/destroy, both cheap enough per frame)
means the two faces are added once at no size and `PushFont(font, px)`
rasterises whatever the frame asks for, so scale is 100..200% in quarter
steps and any glyph the face carries renders. And `XPLMReloadThisPlugin`
reloads only us, so the Settings-tab reload no longer bounces every plugin.
Input is the event-queue API (`AddKeyEvent`/`AddMousePosEvent`), with the
X-Plane virtual keys mapped and Ctrl kept as Ctrl (`ConfigMacOSXBehaviors`
off); the pointer is polled once a frame against the window's global
bounds so a hover never sticks after the mouse leaves; the cursor callback
answers hand over items and I-beam over text. UX added while there: a
typed-command line on the Transcript tab (`xcopilot/typed`, a text dataref
the app polls beside `xcopilot/request`, consumed by writing one NUL back,
injected on the same typed-command queue so every gate and readback
applies), and the window's geometry and open state persist in
`Output/preferences/xvoicelayer_panel.txt` (written 2 s after a move
settles and at unload, restored only if it fits the current screen). ImGui
1.92 renames met on the way: `TabUnfocused` is `TabDimmed`, `TabActive` is
`TabSelected`, `AddRect` swapped its thickness/flags order,
`GetWindowContentRegionMax` is gone (cursor plus `GetContentRegionAvail`),
`BeginChild` takes child flags. The plugin now needs 12.4.4+; the Makefile
defines XPLM420..440 and targets macOS 12 (the 4.4 framework's minimum).
Rule kept: never reload the plugin while an automation is running, the
override channels die with it. And a rule learned the expensive way: the
first cut created the atlas texture inside the draw callback, which the
header permits ("in callbacks") and the runtime forbids ("Never call this
function from within a panel draw callback") by ABORTING X-Plane, mid
taxi-in, on the pilot's own reload. XLua's window creates its texture at
context creation and its draw callback only submits draw calls; ours now
runs the whole ImGui frame and every texture create/update/destroy from a
per-frame flight loop, and the draw callback does nothing but
`XPLMDrawCalls` on the retained draw data (valid until the next NewFrame).
A draw command whose texture is not yet uploaded is skipped, never
created. Corollary for the workflow: an untested plugin build must not be
installed into the live folder while a flight is running, because any
reload, the pilot's included, loads it.

The same session's taxi from Anchorage's GA ramp went 1.4 km on a beeline
across the aprons before the drawn route began, and the tower was Lake Hood
Strip's. `region_tick` picked the ground field by raw distance to
`AirportInfo::position()`, the FIRST runway threshold: Anchorage's first
row is 7R/25L, 2.7 nm from that ramp, so Lake Hood's single threshold
(0.9 nm) was "nearer", and the 5 nm enroute hysteresis then pinned it
there for good. The planner's start snap accepts a taxiway node up to
3,000 m away, so Lake Hood's network (nearest node 1,393 m) was reached by
a straight line. Fix in `nav::AirportInfo::ground_distance_nm`: on the
ground the field is measured by FOOTPRINT, the nearest point on any runway
centreline (0.56 nm to 15/33 against 0.91 nm to Lake Hood's strip), with a
0.2 nm ground margin instead of the enroute 5 nm; the test drives both the
ramp (Anchorage takes over, crew note) and Lake Hood's own parallel taxiway
60 m off 14/32 (Lake Hood stays). Airborne scoring is untouched. Also
heard: "then the holding point via ." when a route had no named legs; the
via clause is now omitted when empty. Not changed: the 3,000 m start snap,
which is what lets thin Gateway networks degrade instead of refusing; a
start that far from the network is a station problem, now fixed upstream.

## Local flight planner, stage one — 2026-09-10

The only way to get a route into the FMS was a SimBrief plan. The plugin's
`xcopilot/fms/route` mailbox takes an ident/lat/lon list and labels the first
and last points as airports (unknown idents fall back to lat/lon rows), so a
local planner only had to produce the same list. `src/flightplan.rs` parses
"plan a flight to X" / "divert to X" (ICAO spelled or written, or airport
name words folded for accents), resolves origin and destination in one
apt.dat pass, picks a cruise altitude by rule over the sim's terrain probe,
works time and fuel from the profile, and refuses a destination whose longest
runway is under the landing distance. The plan lands in the same cache as
SimBrief's with `PlanSource::Local`, so TOD, ETA, runway advice, the monitor's
destination and the GPS load all work unchanged; `ensure_plan` no longer
demands a SimBrief ID when a local plan is shelved, and never refetches over
one without an explicit refresh. A bare "divert" asks "to where?" and takes a
short place-shaped answer for 30 s, guarded by the command grammar so "flaps
ten" can never become an airport lookup. Rounding lesson from the tests:
`{:.0}` on 12.5 prints 12 (half to even), so fixtures avoid halves. Owed:
an enroute point so full auto can fly a local plan (`enroute_legs` drops
both airports and yields no legs), airspace dog-legs, airways with SID/STAR.

## Deterministic conversation foundation — 2026-09-06

The command-oriented history could not support Q&A follow-ups, and spoken
fragments could die at the domain gate before reaching a conversation handler.
`src/conversation.rs` now owns separate, expiring evidence for supported answers,
action reports, reference knowledge, and automation status. It runs before that
gate and returns only speech or read-only questions; never tools. Actual tool
readbacks are preserved, because `Ok(String)` also represents a safety refusal
and is not a success flag. Numeric command history is now cleared alongside the
dialogue stack on airframe swaps. Groundspeed queries were also incorrectly
answering airspeed through the generic speed matcher; the two are now distinct.
See [the conversation contract](deterministic-copilot.md) for supported exchanges,
timeouts, test coverage, and the remaining crew-model work. No LLM is needed.

### Broader questions, topic history, checklist previews — 2026-09-07

Whole-sentence matching and one topic were still too restrictive. Conversation
now composes question openers with closed subject phrases (up to four readings,
queried concurrently), keeps 12 recent evidence records, and supports explicit
returns to a named reading. An unrelated command still retires the implicit
topic; history never becomes an action anchor. Compound action reports are
recorded once per turn, rather than storing each partial result as a separate
request. The read-only adapter now routes fuel to `fuel_report`: it is absent
from the generic instrument reader, which previously made “and fuel?” miss.

Checklist walkthroughs preview the loaded items without calling the runner or
`run_step`. Ambiguous titles prompt for a selection; next/previous navigate a
preview, and an explicit resume restores the item after another topic. Both
selection and preview expire, and airframe swaps clear them. No item is marked
complete. Tests exercise actual parsed checklist content, title clarification,
compound questions, history expiry, interrupt/resume, and adversarial phrases.

### Enroute questions that missed — 2026-09-12

LXGB to LEMG in the 172SP under full auto, the pilot asking the copilot the
questions a passenger in the right seat asks. The blackbox `heard` events and
the log's `deterministic-miss` lines gave five silent misses and two wrong
answers, each now a rule with the live transcript as its test
(`gap_tests_2026_09_12` in `main.rs`):

- "How much left for pimmons?" and "what's the waypoint to act at Pimos?":
  whisper spells a fix it has never seen phonetically. `fix_query` takes the
  word after a preposition, matches it against the LOADED plan's idents
  (first letter must agree, two edits allowed on a five-letter fix, doubled
  letters collapsed first), and answers range and time to it or what follows
  it. Only fires with a range cue, an "after" word or the word waypoint/fix,
  so "set heading to 270" never has its object matched; only a plan already
  on the shelf is consulted, so a stray "for X" never triggers a SimBrief
  fetch. Airports at either end are excluded (that is the destination ETA).
- "Remind me when we get there.": "there" is whatever the copilot last
  answered about. `conversation::resolve_there` rewrites it to "the next
  waypoint" or "top of descent" from the last observation and the grammar
  arms the reminder it already knows. The referent is captured BEFORE
  `Conversation::respond`, which retires the topic on any utterance it does
  not own. A destination ETA stays unresolved on purpose (the tool has no
  destination reminder); the nav answers now `observe()` so they can be a
  referent at all.
- "Aren't we going a bit slow?" missed (no question opener) and "Is our speed
  okay?" answered a bare "72 knots". Negated auxiliaries are openers now, in
  both the raw and the apostrophe-stripped form, and "should/shall we" join
  the do-support rule. A judged speed question gets a verdict: the governor's
  target first, the airframe floor above everything, then Vy in a climb, the
  approach reference low-and-descending or with flap, cruise IAS otherwise,
  and nothing invented when the profile is silent (`speed_verdict`).
- "Should we check in with ATC?": `station::coach_yes_no` routes the
  yes/no shape to the coach, which now leads with "Yes." and the call, or
  "No call owed right now."
- "What weather can we expect on arrival?" was answered from the sim's own
  weather datarefs, the air we were IN, 45 nm from Malaga. New `weather.rs`
  reads X-Plane 12's real-weather download (`Output/real weather/metar-*.txt`,
  one METAR per station) and briefs the plan's destination, alternate, origin
  or a spoken ICAO; a stale file says how old it is rather than pretending.
  The sim's datarefs still answer "here".
- "How long till the next waypoint?" answered "PIMOS, 18 miles" at 13:48
  and "PIMOS, 45 miles" at 14:33 while the procedure had called LUNUM,
  MG452, GUMAZ, BUKOX, OBIGE and MG430 in between; the sim confirmed it live
  (`gps_nav_id` PIMOS at 48 nm, blackbox leg 6 MG430 at 2.4 nm). The
  follower flies its legs in heading mode and never sequences the FMS, so
  the GPS sits on the plan's first entry for the whole flight and every
  reader of it (`nav_progress`, the on-course check, the `next_waypoint`
  reminder armed at 13:52 that never fired) named a fix long passed. The
  follower now publishes its leg (`procedure::active_leg`, cleared when the
  thread ends, `None` while inactive); the question answers distance and
  minutes to THAT fix from position and groundspeed, the course check says
  the autopilot is steering the plan instead of reading HSI dots against
  nobody's course, and the reminder tracks the follower's fix and treats
  the follower ending as the end of what it was watching, never as
  "now tracking PIMOS".
- The mixture manager's "Leaned to peak EGT" line appeared twice in the
  transcript, blue then green: the seek thread posted it to the panel as an
  advisory AND handed it to the voice, which mirrors what it speaks. The
  CLAUDE.md rule (mirror OR post, never both) applied; it posts only when
  there is no voice to say it.

Still open from earlier sorties (09-06, 09-10): "do we have any passengers",
"what kind of questions can I ask you", "how long before we reach the holding
point", "check electrical systems", "let's get clearance for taxi".

---

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

### A spoken ask re-fetches; the TTL is for the side consumers (2026-09-14)

Four "load the flight plan" in two minutes after "that's the wrong flag plan",
and every one re-sent LEMG-LEJR to the GPS in under 50 ms: the plan was on a
10 min TTL and the only words that forced a fetch were "refresh / reload /
update", which nobody says when the plan they regenerated on SimBrief comes
back wrong. Fixed as a fetch POLICY (`PlanFetch`): `Cached` for the implicit
consumers (fuel projection, TOD, ETA, destination elevation), `Asked` for
anything the pilot says by name (a SimBrief plan is fetched every time; a
local plan stays, so "load it" after "plan a flight to" never swaps it out),
`Forced` for the refresh words (over a local plan too). `ensure_plan` now
reports whether a fresh fetch replaced the shelf, so a load that wanted
SimBrief and got the old plan says "couldn't reach SimBrief, using the plan I
already had". The complaint itself is a rule now: `wrong_plan_complaint`
(closed shape: a wrongness word AND a plan word, whisper's "flag plan"
included) forces a fetch and `plan_change_line` compares origin, destination
and route string: "SimBrief still has LEMG to LEJR, generate the new plan
there" or "got the new plan, X to Y, say load it". Full auto was never
affected: it fetches its own plan at arm time.

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

**The Gibraltar straight-in that wasn't (LEMG-LXGB, 12:46Z, 2026-09-11).** Full
auto flew a Baron 58 gate to gate and crashed it into the North Front four
seconds into a go-around. The chain: LXGB has no coded approach, so the arrival
reconstructed a "straight-in" to 09 from the `09D05`/`09D06` DME fixes, and
those fixes are on the VOR/DME approach's OFFSET track, 50 degrees south of the
extended centreline (the ident says "runway 09, 5 DME", not "on the 09
course"). The follower flew them faithfully, handed autoland the aircraft at
the FAF 3.6 km abeam with 5.5 km to run, and autoland's 35 degree cut needs
5.1 km to close 3.6 km: a saturated final that was still 306 m off at 95 ft,
"Going around", and the wreck. Two aggravators rode along. An approach plan
had `cruise_ias_kts: None` ("manages its own speed") while the enroute
governor had just been released, so the throttle stayed at cruise power down
the whole straight-in: 195 kt at the handoff on a 100 kt approach, and autoland
put the gear and flaps out at once against placards of 152. And afterwards the
sim's `has_crashed` left the airframe "on the ground", autoland read the stop as
a rollout, and full auto said "On the ground. I'll clear the runway", called
for taxi, and armed auto taxi on the wreckage. Fixes, each with a test:
`reconstruct_approach` keeps only DME fixes within `RECON_ALIGN_DEG` (10) of
the reciprocal course (the real 09 fixes yield NO straight-in, the visual
autoland takes it; the 27 fixes are on course and still reconstruct);
`approach_plan_for` refuses any FAF that fails the circuit's `interceptable`
test (`faf_off_course`); `power_hold_target` arms the governor at the clean
approach speed (1.45 x Vapp) for approach plans and lets go BEFORE autoland's
throttle law starts; the gear waits for the profile's VLO and each flap detent
for its Vfe (`gear_may_extend`, `flap_within_limits`, published per airframe
next to the thrust-lever selector); `taxi_in_after_landing` reads
`has_crashed`. Six more from the same sortie, fixed the same day, each with a
test: (1) the conditional line-up ("behind the landing Bee-line, line up
behind") was flown as unconditional, full auto rolled five seconds after the
words with the lander at 2.4 nm, and reached the pavement 15 s after the
touchdown only because the line-up crawled for 85 s. The release watch now
publishes the condition every tick (`LINE_UP_CONDITION`: the named arrival is
off the final) and full auto waits on `line_up_condition_clear()` before
crossing the hold line. (2) "Contact Malaga Departure on 121.025" had no crew
consumer: no readback, no retune, and the flight heard Malaga's party line for
thirteen minutes until the region swap. `handoff_tick` now publishes the
station and frequency (`take_departure_handoff`), and the enroute wait reads
it back on the tower frequency, retunes, and checks in with altitude
(`departure_check_in`, shaped to earn "radar contact"). (3) The inbound call
went out three seconds BEFORE the region swapped to Gibraltar: void, and the
zone watch challenged the arrival as unknown traffic. The call now waits for
`home_tower_khz` to become the tuned frequency (45 s bound). (4) The first
approach leg's 1,397 ft was dialled 25 nm out and the aircraft flew 18 nm level
at 1,400 ft across the Strait past Europa Point; `descent_target_ft` holds the
previous target until the 700 fpm top of descent. (5) SimBrief charted MG834
at 3,900 between two 4,000 ft fixes and the dial dipped for nothing; a fix
within 300 ft of cruise is cruise. (6) The tug let go 60 m short of its node
with the heading already right, the plain projection put the aircraft on the
stand-to-node leg behind its tail, and the join drove a loop back to the node;
a join now projects along the nose (`project_signed_facing`). (7) The 6 kt
line-up pace coasted to a dead stop three times in 205 m: at 6.9 kt building
0.9 kt/s the projection called it 3 kt over, idle-and-brake followed, prop
drag stopped the Baron, and the rebuild from idle repeated it. Idle-and-brake
now needs the speed to actually be 2 kt over unless the build is a surge
(1.2 kt/s), and a trim that has taken leaves the lever alone. And the reason the crash was a crash rather than a
go-around: autoland's only lateral balk was the flare gate (under 100 ft,
inside 0.6 nm), so a final that never converged descended to 95 ft still
306 m off the centreline before balking, with a 470 fpm sink and full flap
to reverse and the North Front ahead. The go-around itself was right (full
power and +800 fpm on the same tick); it was four seconds too late. Below
the terrain floor and inside the FAP the final must now still be
interceptable (`stabilised_xtk`, the circuit's own `interceptable` test: the
35 degree cut can close the offset with margin in the distance left) or the
guidance flips to Intercept and the latch flies the balk from 500 ft. Not an
angle gate: the pursuit deliberately flies a 370 m offset out from 0.8 nm,
and a 5 degree rule would have balked that.

**The RV-10 that had no profile (LXGB, 16:37Z, 2026-09-11).** The pilot
loaded the stock Van's RV-10 and every number the copilot used was a Cessna
172's, because `icao_to_profile` fell through to `cessna_172sp` for any type
it did not know and said nothing. The consequences, in order: "flaps twenty"
refused at 89 kt against the 172's 85 kt limit (the RV-10 is placarded 96);
"start engines" warned "battery's low, only 12 volts" on a healthy 12 volt
system, against a 23 volt gate written for the 172's 24; and "land the
aircraft" at 1.5 nm abeam Gibraltar's 09 armed autoland, whose guidance
called any position inside the FAP "final" and flew a saturated 35 degree cut
while the speed schedule pulled the aircraft to the 172's 65 kt approach
speed, which is the RV-10's CLEAN stall (acf Vs 65). It stalled at 63 kt with
a third of flap, departed to 33 kt and +1,900 fpm, recovered, and stalled
again. Fixes: `profiles/vans_rv10.yaml` (acf figures where the POM and the
model disagree, the xlua fuel selector, 12 volts); `known_profile` returns
`None` for unknown types and `generic_from_acf` builds a profile from the
aircraft's own `.acf` placards (speeds, detents with Vfe1..Vfem, gear from
Vle, injected vs carburetted, weights; approach = max(1.3 Vso, 1.2 Vs) because
the acf's Vso is what the author typed and the model stalls above it with
partial flap), with the pilot told "no profile for this airframe" and the
172 fallback used only when the file is unreadable, said aloud;
`bus_volts_nominal` scales the start gate; and autoland's guidance treats an
uninterceptable position above the terrain floor as a reposition to the join
point rather than a final (the below-floor balk from the same day stays).

**Assumptions into checks (2026-09-11, the same evening).** Both of the day's
failures were the system acting confidently on something it had never
checked, so three checks went in. (1) A speed floor under every automation:
`aircraft::speed_floor_kts` is 1.2 x the live profile's clean Vs, published per
swap, and both arming points, `speedhold::arm_flight` and `run_autoland` ,
raise a lower target to it, say so and record it. The RV-10's 65 kt would have
become 78 before anything flew it. (2) The arrival is briefed at the gate:
`full_auto::brief_arrival` runs when "fly the flight plan" is armed and puts
what the destination offers into the acknowledgement ("Gibraltar has a
straight-in on two seven. The wind at the time picks the runway."), refusing
a destination the airport data lacks. What was discovered at 25 nm is now a
briefing item at the stand. (3) `xvl debrief` reads the blackbox and prints
the sortie: aircraft (flagging a no-profile airframe), max altitude and
speed, minimum airspeed airborne and where, every touchdown with its sink
rate, taxi stops, callouts by id, low-confidence utterances, and the
narrative, autoland armings, crashes, balks, ordered go-arounds, give-ups,
floored targets, void radio calls, compliance challenges, handoffs, region
swaps, full-auto stages. Run on the RV-10 flight it printed "min IAS
airborne 23 kt at 16:41:22Z, 810 ft AGL" unprompted: the two hours of grep
that found the day's faults, as a command, and the first piece of the
instructor's scorer.

**The profile probe, and voices on the sim's buses (2026-09-11, late).** Two
more from the same day's list. The probe makes the standing rule ("a profile
line is a hypothesis until probed live") code: `aircraft::profile_references`
walks the profile's own YAML serialisation for every `a/b/c` name (skipping
prose keys, file paths and `[index]` suffixes, so a new field with a dataref
in it is covered the day it is added), and the detection thread asks the sim
about each one once per airframe, `probe_dataref` (existence and
`is_writable`) then `probe_command`, speaks the missing ones ("Profile
check: 2 of 41 names this profile relies on are not in this aircraft: tank
sel left, tank sel right. The tools that use them will fail.") and blackboxes
`profile_probe`. The RV-10's xlua fuel selector and autopilot would have been
verified on load rather than on the flight. The audio bus is the plugin's
first sound: `plugin/src/audio.cpp` registers `xcopilot/audio/play` (a
"channel|volume|path" mailbox, the app writes every utterance to a WAV
already), `xcopilot/audio/cut` and `xcopilot/opt/audio_bus`; the flight loop
reads the WAV and plays it with `XPLMPlayPCMOnBus`, the station on COM1 and
the intercom on the copilot bus, one voice per channel, volume from the app's
channel gains. App side, `tts::play_on_bus` posts the file with a blocking
Web API write (the speaker thread is a plain thread; on an async one it
refuses rather than panic), waits out the clip's duration plus a tail, and
sends the cut the moment the channel's generation moves, the same barge-in
contract as the local player. Off by default (`XCOPILOT_AUDIO_BUS`, or the
Settings switch live); a failed post plays locally and warns once. Unflown:
the bus choice (copilot vs interior for the intercom), the volume scale
against the sim's own mix, and whether the headset simulation treats the
copilot bus the way it treats COM1.

**The radio steps on the copilot (2026-09-11, night).** "Will the copilot
stop speaking if ATC is speaking to us?" It did not: the arbiter gated what
STARTS (routine lines wait for a quiet frequency) but a line already playing
ran through a transmission that began a second later. In a real cockpit both
voices are in the headset and the crew stops mid-word to listen, before
anyone knows whether the call is theirs, then picks up the remainder. Now the
intercom loop registers what it is saying (`IntercomPlay`: priority, the
rendered buffer from the warm ONNX path, the start instant), `Speaker::speak`
renders first and hands the buffer over before playback, and the radio loop
calls `interrupt_intercom_for_radio` as it goes on the air: a line below
Warning is cut (`tts::stop_channel`) and its remainder, from 0.4 s before
the cut, so the interrupted word is heard whole, is queued as an
`Utterance::resume` at the same priority, perishable at 20 s, to play once
the frequency is quiet under the ordinary deferral. A subprocess voice has no
buffer and is said again whole; a line with less than the lead-in left is not
owed; a Warning is never stepped on. The remainder rides the normal queue, so
a PTT press or a second call can step on it again, and the audio bus needs
nothing: the cut is the same generation bump the bus already honours. Still unowned: the zone clearance's
"not above two thousand feet" (neither side carries the altitude limit) and
the squawk watch's "radar contact" spoken before the handoff.

**The final that hunted, and the parallel it landed on (LEMG, 15:12Z,
2026-09-12).** Full auto flew the SimBrief plan LXGB to LEMG and the ILS 12
end to end: clearance, taxi, a released tug, the crossing of 27, departure
handoff, the enroute leg, the inbound call and "join final runway one two",
FI12 captured at 15:04:30 and autoland armed from 8.1 nm. Then two things
the debrief shows in one line each. First, autoland said "runway 13": the
procedure's FAF handoff passed `run_autoland` no forced runway, so it
self-picked by track, and at a field with parallels the self-pick takes the
nearer threshold. The tower cleared "runway one two" twice; the aircraft flew
13, and would have landed on it. `EndAction::Approach` now carries `rwy_id`
(full auto, the circuit and the spoken procedure tool all fill it) and the
handoff passes it down; if the standing clearance names a different end AT
THE SAME FIELD the clearance wins, aloud, blackboxed `runway_from_clearance`
(a stale departure clearance can never send us to a "09" forty miles away).
Second, the balk at 74 ft: a final that the blackbox shows glued to the
glidepath within 5 ft and 65 kt the whole way down went around with
"climbing straight ahead" and no other explanation. The gate was the lateral
one under 100 ft (`FLARE_MAX_XTK_NM`, 37 m), reconstructed from the command:
inside 0.30 nm the cut is `-xtk * 80`, and `cmd_hdg` minus the course walked
1.59, 1.60 degrees, i.e. the aircraft sat 35 to 37 m left of the centreline
from 0.30 nm in and drifted over the line at 74 ft. Why it was there is the
`crab` column: standard deviation 0.5 degrees through the whole intercept,
then 1.2, 1.4, 1.8, 2.5, 2.7, 3.1, 3.7, 4.0 down the final, a ±9 degree
swing with a 13 second period at the numbers. The intercept phase does not
feed the crab back; the final does, and it fed the RAW heading-minus-track
read into the heading bug at unity gain every tick, so every nose swing the
autopilot made came straight back as a bug move and the loop hunted, growing
all the way down. `CrabFilter` (15 s time constant, estimated every tick so
it is settled when the final begins, a missed read holds the estimate) now
supplies the crab; the 13 s swing comes through at a seventh of its
amplitude, tested against that signature. The blackbox keeps both `crab`
(applied) and `crab_raw`. The go-around itself was flown correctly (full
power, +800 fpm, straight ahead to 800 ft, back to the join point, second
approach from 6.3 nm), against a tower that had asked for runway heading and
a downwind report; the crew's missed approach and the tower's instructions
are still two different pictures, noted, not fixed here.

**Same sortie, the position column (15:27Z, the second balk at 40 ft).**
The debrief after landing showed a second go-around at 40 ft, again with no
reason attached, and the pilot took the aircraft and landed by hand. The
state events carry lat/lon, and projected onto the 13 centreline from
apt.dat they say something the command column could not: the aircraft was
NOT hunting laterally. It sat 33 to 37 m LEFT of the centreline from 1.3 km
out (280 ft) to the numbers on the FIRST final, steady to the metre, and 32
to 38 m left on the second, identically, and each balk fired the tick the
standing offset crept past the 37 m gate (`FLARE_MAX_XTK_NM`, 0.02 nm):
74 ft on the first, 40 ft on the second, both `off_centreline_low`. The
heading was swinging 123 to 136 the whole way (the hunt the filter above
ends), but the swing's mean is what set the offset, and the mean was wrong:
true heading averaged 1.9° LEFT of the true-frame bug on both finals, and
1.07° left of it through the whole intercept, where no crab is in the loop
at all (193 ticks; the PANC 25R intercept the night before read 0.01°, so
this is not the variation sign, and the earlier LEMG 13 and 31 finals read
+0.3 and +0.7). A steady bias between the BUG and the HEADING passes
straight through the crab loop: the crab is heading minus track, so it
cancels wind and nothing else, and both laws on final are proportional (the
fixed-lead pursuit, then the 80°/nm short-final cut), so they equilibrate
where the cut equals the bias: 0.6 nm of lead times tan 2° is 39 m, and
2°/80 is 46 m, which is exactly the walk from 34 to 37.7 m inside 0.30 nm
that tripped the gate. Every earlier approach in the blackbox sits within
±14 m at the same point; this one is an outlier the law had no answer to.

`TrackTrim`: an integral term on the final's TRACK error (the guidance
heading, i.e. the wanted track, minus `hpath`), 30 s time constant, ±5°
cap, errors over 10° ignored as a turn, final only, held through the flare,
reset by any intercept so a missed approach never carries a stale
correction into the next attempt. Added to the bug alongside the crab.
Battle-tested through the real `guidance` with a second-order autopilot
that settles 2° off its bug and the loop's own tick and crab filter: without
the trim the plant parks 40 m left and balks, exactly the flight; with it
the final is inside 9 m at 100 ft for ±2° and 1° biases, the trim carrying
the bias, and inventing nothing when there is none. Why the 172's heading
sat left of the bug that day is not known (16 kt of wind on the nose wants
more power on the path, and more torque); the trim does not need to know.

Also from the same debrief: `Guidance.balk` names the gate that balked
(`off_centreline_at_flare`, `off_centreline_low`, `unstabilised_below_floor`,
`overflew_the_runway`) and the `going_around` event carries it with the
cross-track in metres, the tick carries `xtk_m` and `trim`, and the debrief
prints "GO-AROUND at 74 ft radio: off centreline low, 38 m left of the
centreline"; the debrief was also rendering the `final_report` event as a
second "autoland armed: 13 from 5.0 nm, 0 ft radio, 0 kt, phase " line
(arming detection now wants a tick, not any event with a runway). The
second final report said "final runway one three" and the tower answered
"runway one two, cleared to land" as if nothing was wrong: the tower now
corrects a final report that names the wrong runway in the same breath
("negative, runway one two, I say again runway one two"), corpus case
added. And the manual landing's "taxi to parking" at 38 kt: the taxi-in
arrested the roll on the parking brake, silently, and four seconds later the
monitor said "Parking brake is set and we're rolling, release it" about
the copilot's own brake. The arrest now says "Brakes on, holding for the
taxi clearance" and holds a claim (`taxi_in::arrest_holding`) the caution
reads, dropped when the brake is released for the drive or the taxi-in
gives the aircraft back. Still open from this sortie: a landing clearance
whose runway changes after the arming (a go-around, then "cleared to land
runway 31") is not re-read by the autoland loop.


Depth (added 2026-07-28): **the approach is sized for the airframe, and a
missed approach is a procedure**. One MD-82 circuit at Luton exposed four
faults in a row, and they compound: the pattern delivered the aircraft LOW,
the vertical law then flew it HIGH, the balked landing had nowhere to go,
and nobody was told. (1) The base leg descended to a flat 700 ft, so the
handover height depended only on how wide the pattern was, the jet's turns
final around 4 nm, where a 3° path is 1,340 ft up, and it arrived 520 ft
BELOW it (`circuit::approach_leg_alt_ft` now descends the PATH, bounded by
the handoff floor below and the pattern altitude above, so it can only ever
delay a descent, never climb on base). A jet also flies a 1,500 ft circuit,
not a trainer's 1,000 (`pattern_alt_agl_ft`), and sets up at
`approach_leg_kts` rather than carrying Vy, 200 kt in this airframe, which
arrived on base at 226 against a 140 kt approach speed. Never faster than
the pattern speed, so GA airframes are untouched. (2) SPEED PROTECTION was
shallower than the path: a fixed 40 fpm per knot capped at 700 is steeper
than a trainer's path and gentler than a jet's, so 9 kt slow commanded 360
fpm against the 740 the path wanted, HALVED the descent, and made the
aircraft slower still, the exact opposite of the intent. It is floored at
the path rate now. The MD-82 crossed the threshold 51 ft high at 152 kt and
overflew. (3) The go-around then had no procedure. The latch released the
instant it was clear of the ground, leaving the aircraft 800 ft over the far
end with the field behind it, and the STATELESS guidance re-decided every
tick: outbound while the path was 500 ft below, inbound as soon as it
wasn't, cycling; and vertically the reposition floor (~1,590 ft AGL) and the
path tracker met at a step, +800 against -1,200. It orbited for three and a
half minutes between 1.1 and 4.1 nm and escaped by luck. `MAX_GO_AROUNDS`
can never catch this, a second miss needs another approach, so the backstop
never armed and nothing gave up. Now `MissedStage` makes it sticky: Climb
straight ahead, then Reposition outbound to `join_point_nm` at platform
height, released only when established out there on the approach side (not
merely outside the FAP, which hands back mid-outbound-leg pointing away from
the field). The reposition leg never chases the path it is flying away from,
and `descent_limit_fpm` fades the permitted descent to zero AT the floor
instead of through it. (4) Nothing was ever transmitted, so the tower held a
landing clearance for an aircraft that was not landing while it worked the
traffic underneath, the FSM sat in ClearedToLand the whole time. The loop
now calls "going around" once per balked landing over the same crew radio
channel the circuit's position reports use, and a test pins the phrase
against `tower::parse_intent`.

Depth (added 2026-08-01, evening): **and autoland's own pursuit lead was
collapsing on short final.** The fixed-lead pursuit added 2026-07-28 (against
"never in line with the runway, always at an angle" at EGSS) was undone by its
own floor: the aim was `(along - lead).max(0.15)`, so inside one lead distance
the aim STOPPED ADVANCING and the effective lead shrank, 0.875 nm at 1.2 nm
out, 0.55 at 0.7, 0.20 at 0.35, which is the d/2 law the fixed lead exists to
replace, back again exactly where being lined up matters most. At Southampton
the commanded cut sat at 16 degrees from 0.87 nm all the way in to 0.32, then
snapped to the runway course over the numbers with the aircraft still 6 degrees
off and still turning. `pursuit_aim_nm` keeps the full lead all the way in,
which means the aim walks PAST the threshold and on down the runway, not a
trick, since the extended centreline and the runway centreline are one line ,
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
else, which makes it a leg that flies PARALLEL to the line it is supposed to
be joining. That was survivable only while the turn onto final was assumed to
roll out ON the line, and it does not. And the reason it rolled out short is
arithmetic, not luck: `AP_BANK_DEG` is a deliberately shallow 14 degrees and
`gs_est` pads the speed by 1.1, both of which make the radius BIGGER, safe for
every dimension they size except this one, where a bigger capture distance
starts the turn EARLIER and rolls out SHORT. The model wanted 1.3 x 1,443 =
1,876 m; the aircraft turned in 1,048 (a 90-degree turn closes exactly one
radius of cross-track, so the trail measures the radius directly); the
predicted residual is 828 m against 776 flown. Back-solved at the real approach
groundspeed the AP banks 15.8 degrees, not 14. So `final_turn_radius_m` sizes
that one capture on the turn actually flown, approach speed, no padding,
`AP_BANK_FINAL_DEG` 15.5 (rounded shallow so it still errs EARLY, because with
an intercept a short roll-out is a gentle closure and a long one is an S-turn
back across the centreline), which takes the residual from 776 m to 346.
`Geometry::for_radii` keeps the conservative radius for pattern width and
downwind length, where undersizing is what caused the LXGB figure-eights.
`final_capture_m` had been 1.3 turn radii on the assumption that
"rolling out slightly EARLY is free; autoland tracks in": at
Southampton the turn began 1,850 m out and consumed only ~1,050, so it rolled
out **776 m short of the centreline** and then held runway heading all the way
to the handoff, closing almost nothing. "Slightly early" was three quarters of
a radius, and free it was not, autoland inherited the offset on top of an
approach it was already 206 ft high on, flew the diagonal in, and the aircraft
crossed the threshold **still 6 degrees off the runway heading, still
turning**. That is the landing-at-an-angle the 2026-07-30 fix was supposed to
have ended; that fix added a heading test, and a heading test PASSES for an
aircraft flying perfectly parallel 776 m to the side. `final_intercept_hdg`
gives the leg a real intercept, pure pursuit at a point down the centreline,
capped at `MAX_FINAL_CUT_DEG` (30, inside autoland's own 35), held at the cap
until the offset is inside ~520 m and washing out from there, so it converges
like an instrument approach instead of asymptoting (a gentler 2,000 m aim had
faded to 8 degrees by 300 m off and only just made the centreline before it ran
out of runway, the simulated intercept from the live roll-out is a test). And
`ready_for_autoland` now requires being ESTABLISHED, not merely rolled out:
heading AND `ESTABLISHED_XTK_M`. That constant is 300 m rather than something
tighter on purpose, the intercept has to be able to REACH it before the
handoff, and a gate the leg cannot satisfy deadlocks into the last-ditch valve,
which hands over the exact diagonal the gate exists to prevent.

Depth (added 2026-08-01, evening): **the glidepath distance is measured DOWN
the approach course, never across it.** `approach_leg_alt_ft` took track miles
, along-track plus cross-track, and the cross-track term inflates the target
by exactly however far off the centreline the aircraft is, which on a base leg
is the whole leg. Southampton, Baron: base began 3,324 m off the centreline, so
track miles read 3.33 nm and asked for 1,101 ft when the real distance to the
threshold was 1.53 nm and the 3° path was at 529. The aircraft flew base and
most of final LEVEL at 800 ft AGL, the target fell below it only eleven seconds
before the handoff, and autoland was handed an aircraft 206 ft high at 1.7 nm.
It never recovered: still 118 ft high at half a mile, still 80 ft high over the
threshold, crossed at 180 ft AGL and 102 kt, overflew the whole runway and went
around. Plain along-track is wrong in the DANGEROUS direction though, on base
the aircraft is crossing the extended centreline, so its along-track distance
is still GROWING, and aiming at the path for the distance it has now is a dive
toward a point it is about to move away from (529 ft with 3.3 km still to fly).
So `path_dist_m` splits it: base aims at the ROLL-OUT, the path altitude where
the turn onto final will actually deliver us, which the pattern geometry
already knows as `downwind_past_m` (predicted 4,087 m against 3,966 flown);
final follows the real along-track distance down. Which is also just how an
approach works, platform until the path meets you, then down the path. The
2026-07-28 note that introduced track miles was fixing a different thing (a
straight-line HYPOTENUSE growing mid-turn), and the ratchet it also added was
what actually fixed that; the ratchet stays.

Depth (added 2026-08-01, late): **a clean pattern must clear the CLEAN stall,
and the freeze flags were costing frames.** Two loose ends from the same
sortie. (1) The circuit flies clean and the callouts are computed clean, but
the pattern speed had no floor against the clean stall: the Baron's Vy is 105
against 84, i.e. 1.25 Vs, which is FOUR knots above the airspeed caution band
(1.2 Vs = 100.8), so every normal variation on downwind, base and final dipped
into it and six callouts fired in one circuit, every one of them correct. Flap
was 0.00 throughout, which is right; nothing knew that made the stall reference
84 and not the flapped 70. `pattern_speed_kts` takes `vs_clean_kts` and floors
at `PATTERN_MIN_STALL_MULT` (1.3 Vs), which the approach legs inherit. It only
bites on an airframe whose Vy sits near its clean stall, a 172 (Vy 74, stall
48), an MD-82 (200/100) and a Citation (200/136) are all untouched, and a
profile with no stall figure gets no floor rather than an invented one. Same
family as the climb limiter: a speed that is fine flapped is not fine clean.
(2) The Web API cache gained a per-dataref TTL. The 60 ms window was sized for
DUPLICATION and recovers ~2.5% because the loops read DIFFERENT datarefs;
VOLUME is the problem, and the histogram named it, after latitude and
longitude came `is_in_replay` and `paused`, ~4 reads/s between them for two
flags that change a handful of times in a whole flight. `SLOW_DATAREFS` get
500 ms. Nothing needs them sooner: a pause noticed half a second late costs one
control tick, every loop re-checks each tick, and `sim_frozen` fails open. The
evidence it matters: at 20:42 the sim dropped to **7 fps against a 60 fps
median** in the same 30 s window that logged 76 req/s and a 1,040 ms slowest
response, the API is served from inside X-Plane's flight loop, so that is
frame time the sim does not get.

Depth (added 2026-08-02): **the limiter's full-rate speed is the speed the
climb is FLOWN at, and two same-evening fixes had never met.** The stall floor
above raised the Baron's pattern speed to 109.2 kt, four knots ABOVE its Vy of
105, and the climb limiter below took that same pattern speed as its full-rate
threshold, while the throttle law holds the aircraft AT it. That parks the
steady operating point on the exact top edge of a nine-knot fade band: full
climb is unreachable while climbing (climbing at Vy is what the airframe
does), and every knot of ordinary jitter, amplified by the 5 s trend
projection, whose swing is as wide as the whole band, slams the granted rate
across half its range. The next morning's circuit porpoised exactly as the
projection was supposed to prevent: cmd_vs 700 to 0 on crosswind with the
Baron DESCENDING at 190 fpm at 670 ft AGL mid-climb-out, then the whole
downwind hung 100 ft below pattern altitude with the capture choked to
0-150 fpm and the speed see-sawing 104-113 on a 25 s cycle (EGHI). The unit
test had pinned `want = 105.0`, Vy, all along; the call site passed
`pattern_kts`, and the four-knot difference between two numbers that used to
be equal was the entire failure. `climb_full_rate_kts` (Vy capped below the
leg speed, a jet's book Vy is a cruise-climb figure and must never RAISE the
threshold, and floored a margin above the fade floor so a degenerate profile
gets a band, not a cliff) is now the one seam, and the test wires it exactly
as the loop does, so the test and the call site can never again agree on
semantics while disagreeing on the number.

The SAME sortie's re-fly then showed the other half (2026-08-02, later): with
the limiter fixed, the downwind climbed clean at the full 700, but one deep
dive-and-recover cycle survived on crosswind, and its driver was the THROTTLE
law, not the limiter. The circuit inherits the takeoff handoff at 131 kt and
full power against a 109 kt target; `throttle_adjust`'s P+D is algebraically a
projection at 2.7 s of lead, shorter than the lever's own slew, so it was
still pulling at 124 kt with the speed already falling 2.5 kt/s toward the
target, the sag ran through to 98.7 kt, ONE KNOT BELOW BLUE LINE, and full
recovery authority only unlocked at the old gate's raw err > 10, i.e. once the
aircraft was already 10 kt slow. The limiter (correctly) cut the climb on the
way through and the aircraft levelled, DESCENDED 65 ft at 700 ft AGL, and
climbed again, the "oscillation" the pilot reported after the first fix. The
law now projects 5 s (`THROTTLE_LEAD_S`) and both escalation gates watch the
PROJECTION: the pull eases while the excess is still burning off, and power
returns before the undershoot, urgency about where the speed will be, not
where it was. Every old behaviour a test pinned (deadband, SF50 runaway pull,
anti-stall reflex) is unchanged; the new test pins the handoff transient
itself.

Depth (added 2026-08-02): **a landing is one continuous manoeuvre, and both of
its rough edges came from treating it as segments.** The Baron's first
successful circuit was firm on and harsh after, and the blackbox names both.
(1) The touchdown: past the threshold the (unsigned) glidepath distance grows
again, alt_err goes negative, and the vertical law's level-hold branch fired at
27 ft OVER THE RUNWAY, the aircraft floated level, bled the very airspeed the
flare needed, and dropped on from the mush at -380 fpm one knot above the clean
stall. A ROUNDOUT band (`ROUNDOUT_MULT` x the flare height, same
pavement/cross-track gates as the flare) now eases the target to
`ROUNDOUT_VS_FPM` (-250) so the -100 flare is entered from a gentle descent
with flying speed, the two wrong answers there, chasing the path rate and
holding level, were both the same missing stage. (2) The rollout: a flat 0.45
brake ratio measured 3.3 m/s² from 79 kt to a DEAD STOP, max-autobrake
territory on a runway with a kilometre to spare, and then the vacate
re-accelerated the aircraft it had just parked. `rollout_brake_ratio` brakes by
wanted deceleration (comfort 1.6 m/s², overridden by the stop-in-what-remains
term, so a short runway still gets everything), and when full auto's taxi-in
follows, the rollout hands off ROLLING at `VACATE_HANDOFF_KTS` (10 kt) straight
into the vacate drive, stopping dead on an active runway to then leave it was
neither procedure nor comfort. The channel is `autoland::VACATE_FOLLOWS`, set
by `taxi_in_after_landing` before the landing, CONSUMED at rollout entry and
cleared on every no-landing path, because a stale process-wide fact is the
radio-power-latch bug in a new coat: a later standalone "land for me" must
brake to a stop exactly as before (nobody follows it). The safety half lives in
taxi_in: any vacate failure after a rolling handoff arrests the roll (parking
brake + blackbox `arrest_rolling`) before the minutes-long taxi request wait ,
same rule as pushback, a stage that is not moving the aircraft is holding it.
The radio order was already right and stays: vacate first (completing the
landing clearance we hold, no call owed from the strip), "runway vacated" from
the taxiway, then the taxi request.

Depth (added 2026-08-02, afternoon): **the overflight rule must know a landing
float from an overflight, and a give-up must hand back a flyable aircraft.**
One chained failure, four fixes, from a single balked landing that should not
have been balked. The roundout (same morning's fix) legitimately floats
further down the runway than the old at-path-rate descent ever did, and
`OVERFLIGHT_GO_AROUND_NM`'s fixed 0.15 nm was tuned on the old profile: at
EGHI it declared an overflight at 26.3 ft radio, ONE FOOT above the 25 ft
flare gate, with 1,400 m of runway ahead, and balked a good landing.
`overflight_allowance_nm` makes the question runway-aware when the aircraft is
LOW over the pavement (below the roundout band): float on while
`FLOAT_MIN_REMAINING_M` (900 m) remains, capped at half the runway, floored at
the old 0.15 so short strips keep the old rule, and HIGH past the threshold
keeps the hair-trigger, so the 167 ft overflight and the MD-82's 51-ft float
stay caught. The flare/roundout zone gate stretches with it (`in_flare_zone`),
because past the threshold `dist_nm` is unsigned and the plain 0.35 nm bound
would have ended the flare zone mid-float and resurrected the level-hold bug.
Then the cascade: the missed approach flew its reposition compliantly but
`MAX_MISSED_APPROACH_SECS` (300 s) was SHORTER than the flight the reposition
itself commands (climb to 800, turn back, overfly, out to a ~6 nm join point:
5+ min at 110 kt), so it expired mid-leg, now 600 s. The give-up then set
half throttle and walked away from an aircraft the balk had left with GEAR
DOWN and LANDING FLAP, which cannot hold level flight at 0.5: it decayed 110
to 80 kt at -900 fpm in thirty seconds. Give-ups now hand back climb power
(`GIVE_UP_THROTTLE` 0.75) with the VS dial zeroed and a line that says what is
actually held ("the autopilot is holding heading and altitude, gear is down" ,
the handoff-truth rule again). And the pilot's panic FULL AUTO at 1,475 ft
descending then walked the GROUND chain, a stale `LAST_ARRIVED` from the
morning's taxi read as "at the hold" and it requested a departure clearance
from a sinking aircraft, so full auto now refuses to arm airborne and names
the tools that fly ("say do a circuit, fly the flight plan, or land for me").

Depth (added 2026-08-01): **a climb is never worth more than the airspeed left,
and landing includes getting off the runway.** The Baron 58's first full-auto
circuit reached the gate at Southampton, and found two things the 172 never
could. (1) The autopilot's vertical mode buys rate with knots and will keep
buying: rolled into the crosswind turn at 127 kt it came out at 85, one knot
over its clean stall and one over Vmca, with the VS law still asking for
880 fpm and the stabilizer trimmed to the nose-up stop. Six airspeed callouts
fired across the pattern and every one was right. The throttle law cannot fix
this alone; it was already commanding more power, and in a climbing turn on a
piston twin there is none left. `circuit::climb_limit_fpm` fades the commanded
CLIMB to zero as the speed falls from the pattern speed to `climb_floor_kts` ,
blue line where the airframe publishes one (a twin below Vyse has no
single-engine climb at all), else 1.15x the clean stall. Full rate AT the
pattern speed, so an aircraft flown correctly is never penalised; descents pass
through untouched; a profile with neither reference speed gets 0 and no
protection rather than an invented number, which is why every GA airframe is
unchanged (a 172's floor is 55 kt against a Vy of 74, it never meets the
limiter). (2) Autoland stops ON the strip and hands back, and for a CIRCUIT
nothing came after it: full auto armed the circuit and EXITED, so the aircraft
sat where it stopped. It held runway 02 for ninety seconds with an arrival
closing 2.6 nm to 0.5, six `runway_incursion` records and the tower telling
the lander "continue approach, traffic on the runway ahead", and only moved
because the pilot said "taxi to parking". `taxi_in_after_landing` is now the
last stage of BOTH missions (the flight path already had it), so full auto
stays ACTIVE through the circuit and vacates.

**And vacating asks nobody** (same day, the second half of that fix).
[src/taxi_in.rs](src/taxi_in.rs) is two phases now, VACATE, then taxi, because
the original had real procedure backwards: it transmitted the taxi-in request
from wherever autoland stopped and waited up to sixty seconds for a stand
route, sitting on the strip the whole time, with the give-up path handing back
"your aircraft" still on the runway. Clearing the runway is the COMPLETION of
the landing clearance we already hold, not a new instruction: no controller in
the loop, no readback owed, and the call ("runway vacated") goes out from the
taxiway afterwards, which is the order a real crew flies.
`nav::plan_vacate` reuses the departure machinery rather than inventing a
second idea of where a runway meets a taxiway, `zone_of` and the hold-short
trim, run BACKWARDS (the trim drops trailing edges inside the runway's
protected zone so a departure stops short; the vacate keeps going until it is
`VACATE_CLEAR_M` past the same boundary, because the tail has to be over it
too), with two things that genuinely differ. Metres covered while still INSIDE
the zone earn no credit toward that clearance, or the drive finishes at the far
end of the entry link, which by the trim's own logic IS the holding point, i.e.
exactly the line we are trying to get past. And an exit is NOT an entrance:
`runway_entrance_near` accepts a node one hop off the runway (a departure holds
there and the drawn lead-on carries it on), but an arrival must decide at the
JUNCTION, measuring "is this exit ahead of me" at the far end of a link says
nothing about where that link meets the strip, and a junction 22 m behind the
aircraft reads as an exit 150 m off to the side. Exits behind the rollout are
refused outright: reaching one means taxiing back down the runway. Thin data is
never a refusal (no zones, no runway edges, it degrades; only a field with no
network at all declines), the drive arms through the same governor front door
as the auto taxi and the line-up, and every failure falls through to the
taxi-in exactly as it behaved before. Reached from the crew side through
`party_line::plan_vacate`, the one accessor there that is not a published
clearance, deliberately: the world owns the airport's geometry and the crew
should not keep a second copy, but this returns pavement, not permission.
Gated on a STANDING landing clearance (`taxi_in::vacate_runway`), which holds
over exactly the window between touchdown and asking for the stand, reading
`clearance_runway` alone would hand a stale ident to a drive aiming for a
runway exit from the middle of a taxiway.

Depth (added 2026-07-27): **pushback** ([src/pushback.rs](src/pushback.rs)) is
the chain's first metres, between the taxi clearance and the drive. The
trigger is the CLEARED ROUTE, not a stand database or a per-airframe flag:
the first route point that isn't the stand itself is the taxi lane, and its
bearing sitting past the wing line (`BEHIND_DEG` 100) is what nose-in MEANS ,
a tie-down you can taxi straight out of plans `None` and never calls a tug.
The push is STEERED, so it ends on the lane pointing DOWN it (the finish
heading is the lane's own direction, node to next node, never the reciprocal
of our push, a stand entered at an angle would otherwise leave us aimed
across the taxiway) and the airliner never has to turn itself around on the
apron. But steered means CHOSEN, not flown: X-Plane's three pushback commands
are whole manoeuvres the tug executes and finishes, not a rudder, read
Commands.txt, "Nose 90 left" / "Straight back" / "Nose 90 right", and
`pushback_stop` is "stop and LET GO". So the pattern is picked ONCE from the
initial heading error and re-issued, never re-steered per tick. The first
version treated the three as a steering channel and the trap sprang
immediately: it picked `left` correctly, then its own twelve-second nudge
sent `straight` unconditionally, which RE-AIMED the tug, and since the
tracked command still read Left, nothing ever sent Left again. A Gatwick
MD-82 was pushed 51 m dead straight with its heading unchanged to a tenth of
a degree, 115 degrees off the lane (2026-07-27). Two more things fall out of
the same semantics: the command issued before a tug exists only SUMMONS one
(the push began only after the nudge re-issued it once hooked), so the
pattern is sent again on hook-up; and a tug that stops having moved has
FINISHED, which is why `STALL_SECS` ends the push, motionless, the same
sortie sat out 203 further seconds to `MAX_SECS`. Short of the plan the tug
is asked for another pattern, bounded by `MAX_ISSUES`. A pushback command is ONE
complete manoeuvre, and COMPLETION INCLUDES LETTING GO of the bar: the
Stansted trail (2026-07-28) shows "Nose 90 left" executed to the letter ,
err pinned at −98.5 for 30 m of straight push, a clean ~90° arc, bar off ,
with nothing sent after the initial issue, so "attached dropped after we
rolled" is the manoeuvre's normal END, not a tug abandoning us (three pushes
were failed as `tug_released` for finishing correctly). The release handler
now judges completion (aligned within `ALIGN_DEG`, or `done()`); short of the
plan it issues the NEXT pattern, which summons the tug back to the bar (it
is standing at the nose; `REHOOK_SECS` bounds the wait), bounded by
`MAX_ISSUES`. Which way `pushback_left` swings the nose is still LEARNED per
session (`sense_is_wrong` flips once, `LEARNED_SENSE` keeps the verdict), but
the DEFAULT guess is now the measured one: on X-Plane 12.4.3 `pushback_left`
swings the nose CLOCKWISE, opposite to its "Nose 90 left" description ,
three pushes, three identical full-turn trails. A correction is never sent
mid-manoeuvre (that re-aims the tug); the release handler and the hook-up
re-issue are the moments a pattern takes. Brakes are the stage's own too:
SET for the tug wait (full auto's early stage-2 release left an idle jet
brakeless for the minutes-long wait and it crept into the grass, same day ,
the release is deleted; the taxi governor, pushback and auto-takeoff each
release at their own moment), released at hook-up, re-set at disconnect and
on the mid-apron failure exits. Every tug
instruction goes through the
same `pushback` tool a spoken "push back left" reaches, one owner for the
ground-ops commands. The tug handshake is ONE dataref plus our own position:
`pushback_attached` is the bar, and "under tow" is measured by having ROLLED
(`MOVED_M`). `sim/flightmodel2/misc/tow_in_progress` sounds like the second
half of that handshake and is not, probed live at a stand with the aircraft
rolling backwards under a tug, it read 0 from hook-up to release
(2026-07-27). Everything that had been gated on it was therefore dead code
with a plausible story attached: the push never believed it was moving, so
the tug-released check could never fire and the nudge went out at twelve
seconds on every normal push; the wait-for-the-tug-to-clear before taxiing
returned true on its first read with the tug still on the nose; and the
monitor's callouts, which CLAUDE.md claimed narrated the push, spoke only
"tug connected", never "pushing back" or "tug clear". The dataref is still
read everywhere it was, as extra evidence only, and the per-tick `push`
events now carry `att`/`tow` so the next sortie can re-settle it. Clear is
the BAR coming off, never the rolling stopping, or a pause mid-push
announces a tug that is still hooked on. The stage is INLINE in full auto, not a spawned
sub-loop like the taxi governor, nothing else can arm a pushback, so there
is no second owner to arbitrate with, and it is deliberately BEST EFFORT: no
tug, a timeout, or a tug that lets go early falls back to the taxi join's own
turn-around, which is exactly the behaviour that shipped before this stage
existed. Which failure it was has to reach the pilot, though: full auto spoke
ONE line for all of them, so a tug that came, hooked on and pushed us fifty
metres was reported as "no tug came" by a copilot the pilot was watching it
through the window (2026-07-27), the failure line is per-reason now.
Narration is otherwise the MONITOR's (its pushback watch calls the tug
connecting, the push starting, and the tug clearing), so the module speaks
only what that watch cannot know: the ask, and the failures. Laws
battle-test as plain `cargo test`; events blackbox as `"kind":"pushback"`
(the arm log carries the stand position AND the lane node, because "no tug
came" is undiagnosable without them; per-tick `push` events carry
back_m/err/steer/sense). First live push (Gatwick, Twin Otter, 2026-07-27)
proved the trigger and the fallback, nose-in at 141 degrees off the route,
turned around under its own steam, and convicted the TUG WAIT: the tug is a
vehicle crossing an airport, not a dataref flipping (X-Plane searches 1500 m
for one and lets it drive 6 km:
`sim/private/controls/trucks/max_search_radius_m` / `max_drive_distance_m`),
and a 30 s limit gave up on a tug that was still driving. The wait is 180 s
now, with a crew line at 30 s so a stand-sitting aircraft never reads as a
hang, and the give-up still cancels the request so a late tug can't hook onto
an aircraft that has started taxiing. A tug DOES come: watched live at a
Gatwick stand the same evening, `pushback_attached` flipped to 1 and the
aircraft was rolling three seconds later, well inside 180 s, well outside
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
MAGNITUDE, the same trap the taxi guide hit when four knots backwards read as
four forwards, one module over. The tug hooked on and never pushed; the
aircraft, brakes off for the hook-up and both engines idling, crept FORWARD off
the stand. That creep counted as pushback progress, crossed `MOVED_M`, latched
`towed`, and with the attach flag reading false the loop concluded the tug had
let go after pushing us 1.2 m. `back_along` now projects displacement onto the
reverse of the STARTING heading (start, not current: a "Nose 90" pattern puts
the nose across the displacement, but the displacement is still behind where
the nose began), and rolling forward past `CREEP_M` with nobody towing ends the
push, brakes on, as `creeping`. The second half is that the re-hook wait ran
its full minute BRAKELESS, the hook-up release rule ("the first moment
releasing them is safe is when something is holding the aircraft") was written
for the tug wait in July and not applied one state later, so the Baron
accelerated to 21 kt across the apron while the module believed it was standing
still, the tower berated us for leaving the cleared route, and the pilot
stopped it by voice. Brakes now go on before the re-hook wait and come off
again on the re-hook, and the ABORT path sets them too, it only ever stopped
the TUG. Every one of these is the same rule: a stage that is not moving the
aircraft is holding it.

**And the creep guard then killed the pushes it was guarding** (2026-08-02,
same stand, three in a row). The tug's dead air is MINUTES (Luton: hooked for
74 s before rolling), the brakes come off at the hook-up, and a Baron at idle
walks forward in seconds, so every push ran: hooked, brakes released, forward
3 m in four seconds, `creeping`, dead, before the tug ever moved. Failing on
the creep was the wrong verb; the right one is HOLD: `should_hold_creep`
squeezes the expiring proportional brake channel (`xcopilot/taxi_brake`,
`HOLD_BRAKE` 0.4, from `HOLD_AT_M` 0.3 m of forward drift) every tick while
nobody is pushing, it arrests the walk this tick and frees itself in a
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
after the pilot's ground truth "of course push works", the sim's tug DOES
push this aircraft when the brakes are held until it is really connected):
CONNECTED means connected, not glimpsed. The hook wait requires the
attach/tow flag up for `HOOK_SOLID_POLLS` consecutive polls before it counts,
so the brakes stay ON through the tug's positioning instead of releasing on a
one-poll flicker, and the pattern is issued BEFORE the release, the aircraft
is never brakeless with a tug that has no instruction yet. The
still-unanswered question is narrower now: whether the flag goes solid once
the tug finishes positioning (the pilot's experience says it will), and the
next sortie's blackbox answers it. **It did** (Malaga, 2026-08-03): the hook
went solid and held, the brakes came off on a real connection, and the tug
pushed. So the push itself is SOLVED, and what the same two sorties then
exposed is that everything downstream of it was judging the result against
numbers a tug cannot deliver.

**A pattern is ninety degrees, and that belongs in the arithmetic** (2026-08-03,
Malaga, two identical sorties at one stand). The tug ran a textbook Nose 90 both
times: 55 m back, the nose swung 041 to 311, then it stopped and let go exactly
as the command says it will. Both were reported to the pilot as "the tug let go
early", the same wrong verb the 2026-07-28 Stansted fix removed one state
earlier, and this time it cost the whole ground chain. Three faults compound and
they are all the same fault. (1) The PLAN wanted 135.4 degrees, which one
pattern can never satisfy, and nothing checked that when the plan was made:
`done()` wants ±6 and the release-time "good enough" test wanted ±20, so a
44.6-degree residual failed both. (2) The loop's answer to that residual was
`steer_for(-44.6)`, ANOTHER 90-degree swing, which would have left us 45
degrees off the other way. `worth_another_pattern` is the missing question:
another pattern helps only while real DISTANCE is owed (a straight segment
earns its ask) or at least HALF a swing is, because 45 degrees is the point
below which the next pattern overshoots by more than the error it was sent to
remove. (3) The re-issue then waited `REHOOK_SECS` for a tug that has never
once come back, `reissued_after_release` fired four times across the entire
blackbox and `rehooked` answered it zero times, four for four, each costing the
full deadline before the identical give-up. The premise was "it is standing at
the nose"; measured, the hook takes MINUTES (262 s and 268 s here, 90 s at
Luton) because it is a vehicle driving across an airport, and a released one
drives away. So the bar coming off now ENDS the push, and the only question is
`push_delivered`, has the tug delivered what a tug CAN deliver. Forty degrees
and 55 m off the stand is the taxi join's problem, and the join is built for
it (141 degrees under its own steam, Gatwick 2026-07-27); a metre and a half
is still a genuine failure and still reported as one. The stall path also
learned that only a tug ON THE BAR can take an instruction: without that gate
the stall clock and the release branch both fired inside one tick (`issued` #2
and `reissued_after_release` #3, 0.6 s apart), spending the whole budget on a
tug that had already gone.

**The taxi governor was braking itself to a standstill, through the park-brake
VALVE** (2026-08-03, Malaga, three taxis over two sorties). Every drive died the
same way: roll cleanly to about 9 kt, make one small anticipatory squeeze, stop
dead within four seconds, and sit there until `stagnant` fired with 602 of
631 m still to run. Re-engaging repeated it one squeeze later.

The valve is the whole of it, and it had been read as an animation flag. While
`park_brake_valve` sits in the SET position the sim traps any brake application
into `parking_brake_ratio` and LATCHES it, which is just what a park-brake
valve physically does, connect the wheel-brake line to the accumulator so
pressure put into the line stays in it. So the proportional channel
`xcopilot/taxi_brake`, whose entire contract is that a squeeze expires within a
second, was writing a PERMANENT parking brake at the squeeze's own value. The
blackbox shows it exactly: `park` steps 0.0 to 0.1 on a `cmd 0.10` and 0.0 to
0.3 on an `out 0.28`, and the second one was still holding the aircraft an hour
later. Then the governor's own next tick reads that ratio as the pilot's foot
and pauses, idle throttle, waiting for a pilot who is not braking. Both of the
self-pollution guards there (`governor_braking`, `brake_cmd_quiet`) are CLOCKS,
and a clock cannot see a latch: anything that makes one of our own applications
permanent outlives both, which is why the fix has to be upstream and not another
gate.

Nothing ever opened the valve because the SET path went through the valve-aware
`drive_park_brake` and the RELEASE path did not, three call sites wrote
`parking_brake_ratio = 0.0` raw (both taxi_in phases and the auto-takeoff's
brake release), which looks complete and leaves the valve armed behind it. The
asymmetry IS the bug. `ensure_park_brake_valve` is now the one seam, commanded
absolutely (`park_brake_valve_open`/`_close`, not a toggle guarded by a stale
read), the governor isolates the valve on every taxi arm rather than only as a
side effect of releasing a brake that happened to be set, and
`release_park_brake_for_roll` replaces the three raw writes. The BE58 profile
declares `park_brake_valve_set: 1.0`, measured both ways live, and per-airframe
for the usual reason: SET is 1 here and on the SR22, 0 on the C90B, so a
constant would strand half the fleet with the valve backwards. An airframe
declaring no valve is untouched.

Worth keeping: the FIRST diagnosis of this, the same day, was wrong. It blamed
the toe RATIOS latching across a standdown (expiry frees the plugin override,
never the ratios) and shipped `should_release_brake_channel` against that story.
Probed live the next day with the aircraft still stuck, `left_brake_ratio` and
`right_brake_ratio` both read 0.0 and `override_toe_brakes` 0, the plugin's
expiry cleans up after itself exactly as documented, and the 0.5 that had been
read off "the ratios" was the parking brake all along. The arm-time clear stays
because zeroing a channel you have not zeroed is honest and costs one write, not
because it was ever the bug; its doc comment now says so, since a fix with a
plausible story attached is how `tow_in_progress` survived a fortnight. The exit
release stays too, and means more now than it did: with the valve isolated a
governor that dies mid-squeeze leaves NOTHING holding the aircraft, so it
releases only when the parking brake is actually set and logs the case where it
is not. NOT yet flown.

**The route is the network's; the LINE is the paint's** (2026-08-05, Malaga,
noticed from the cockpit window rather than from a log). The Baron taxied
1,214 m to runway 13 tracking its planned route to 0.05-0.13 m, and did the
whole taxi 1.7 to 2.8 m to one side of the yellow line, 5 to 6 m through the
taxiway E junction. Nothing was wrong with the drive. apt.dat carries two
descriptions of the same pavement and the routing network is authoritative
about only one of them: 1201/1202 are hand-authored TOPOLOGY (what connects to
what, and what it is called), while the yellow line itself is drawn separately
as a 120 linear feature, and at LEMG the two sit metres apart. Following the
network exactly is therefore following an approximation exactly.
`find_taxi_net` now also collects the drawn centrelines (line types 1 and 51,
beziers sampled, since the fillet at a junction is a curve and its chord is
not), and `ride_paint` resamples every planned route onto them: each leg is cut
into 5 m samples and each sample slid sideways onto the nearest drawn line
running the same way, inside `PAINT_SNAP_M` (8 m; wider is a different
taxiway, not a mis-drawn network) and `PAINT_PARALLEL_DEG` (25 degrees, which
is what stops a route being dragged onto the taxiway it is CROSSING, or onto a
hold bar drawn square across it). It can only ever move a point sideways onto a
line the leg was already following: the topology, the names, the crossings and
the holding point stay the network's. The departure route rides it, so does the
lead-on (which IS a painted line) and the vacate; runway legs are left alone,
and with them the vacate's start point, which is the aircraft's own position and
the one point in any route that must never move. At LEMG 230 of the route's 242
points now sit ON the line (median 0.00 m, from 1.7-2.8), and the 12 that don't
are where nothing is drawn, thin data degrades, it never refuses. One
knock-on, from the same family as every other unit change: a leg used to mean a
network EDGE, so `mark_runway_crossings` exempting "the last leg" exempted the
whole approach to the holding point, and a leg is now metres long, the
exemption is the route's TAIL (`TAIL_EXEMPT_M`) or a route announces a crossing
of the runway it is holding short of. NOT yet flown.

The same sortie's other half is a diagnosis fix: **an unsigned quantity
standing in for a signed one reads plausibly and is wrong half the time.** The
taxi blackbox's `lat_m` was a DISTANCE from the route, so the entire log of
every taxi ever flown records how far off the line the aircraft ran and never
which side, which is the only question when the pilot can see the paint.
`taxi_guide::project_signed` keeps the side (positive is right of the route's
direction of travel) and `project` is its `.abs()`, because every GATE
genuinely wants the magnitude: "off the route by more than X" is the same fault
on both sides, and a signed value silently disables such a test on one of them.
Exactly the trap the taxi guide hit when four knots backwards read as four
forwards during a pushback, one module over.

**One unread flag cost the entire arrival** (2026-08-05, Malaga, the circuit
flown right after the paint fix). At 13:41:06 the tower cleared us to land; at
13:41:07 it ordered "go around, traffic on the runway". `PENDING_GO_AROUND` had
been published on the party line since the day it was written and NOTHING EVER
CONSUMED IT, so the approach flew on for another seventy seconds, through a
0.5 nm same-altitude conflict, and balked only because it happened to arrive
high. Then the cascade, where every link is a fact the previous link owed it.
The order VOIDS the landing clearance, and the missed approach never reported
again (the circuit makes the first "final" call; autoland's re-approach made
none), so eleven minutes later the aircraft landed without a clearance and the
tower said so. No clearance meant `vacate_runway`, gated on a STANDING
ClearedToLand, returned `None`, the vacate never ran, `taxi_in` arrested the
roll where it stood, and the Baron sat ON runway 13 through four incursion
warnings with an arrival closing to 1.3 nm. One fix per link.
`GoAroundLatch::order` flies a controller's order from wherever the approach
is, and it outranks everything the aircraft notices about itself, because the
tower can see the runway and we cannot; consumed in the autoland loop (the
thing that can ACT on it) rather than the main loop, drained at arm time so a
stale order cannot balk the next approach, and refused once on the ground,
where a landing that has already happened is not one to abandon. Autoland then
owes a fresh `final` report after any go-around, made established inside the
FAP because a position report has to be true. And `vacate_runway` falls back to
`autoland::landed_runway`, the strip the wheels actually rolled out on: no
standing clearance is not evidence that we are not on a runway (an ordered
go-around voids it, and copilot-only mode never has one), and landing without
one makes clearing the runway more urgent, not less. A stale ident is harmless
where the stop was not, since `plan_vacate` refuses unless we are on an
authored runway edge. NOT yet flown.

**The apron is where the network stops, and an arrival has to get past it**
(same sortie). apt.dat routes taxiways; the lead-in painted into a stand is
mostly not in the network at all, so `plan`'s destination snap put the last
point at whatever node lay nearest the stand and the drive finished there ,
66 m short of stand W5 at Malaga, parked across the live taxiway the tower had
just cleared somebody else down, twice, having named the stand both times.
`plan_to_stand` continues the route from that node to the stand itself, and the
continuation rides the paint like everything else (`STAND_LEAD_IN_MAX_M` 120 m,
because a stand further out than that is not the one this route serves and
driving blind across an unseen apron is worse than stopping short of it). The
leg carries no NAME: an apron lead-in is not a taxiway, and the guide would
otherwise call a turn "onto" something that is on no chart. At LEMG the route
now finishes 0.0 m from W5. The departure mirror has been right since the
lead-on landed, a route ends short of the runway ON PURPOSE there, and the
drawn line carries it the rest of the way.

**A proportional controller cannot track a ramp, and a glidepath is a ramp**
(2026-08-05, the same Malaga circuit, the reason the first approach was
already lost before autoland ever saw it). The aircraft was handed over 174 ft
above the 3 degree path with 2,827 m to run, and autoland closed only half of
that in a mile, so it crossed the numbers 59 ft high and balked. The pattern
had rolled out onto final only 54 ft high; the error then GREW, monotonically,
to 172 ft in forty seconds. Two faults in `alt_capture_vs`, and the second one
hides inside the fix for the first. (1) There was no feed-forward for the
target's OWN rate. On final the target is the path at our distance to run, so
it walks down at 318 ft per mile covered, 510 fpm at the Baron's closure, and
a proportional law tracking a ramp settles at exactly ramp over gain: 510/3 is
170 ft, against 174 flown. The clamp said the same thing twice over, since a
flat -500 fpm is LESS than a 3 degree path's own rate at any approach speed
above 94 kt, so the leg could not have held the path with zero error and
perfect control. `path_descent_fpm` feeds the rate forward (from the measured
GROUND closure, because the path descends per mile COVERED and an IAS proxy
would over-command by the whole headwind), and the descent authority is that
rate plus a fixed correction budget, which leaves a standing target's clamp
exactly where it was. (2) The 8 s lead then projected the AIRCRAFT forward and
compared it against where the target is NOW, which in a steady descent reads
the aircraft as rate-times-lead lower than it is and parks the law 77 ft high
all by itself. The lead compares two futures now; for a fixed target its future
is its present, so the downwind and the base leg are bit-for-bit unchanged.
Simulated against the flown profile the old law grows 51 ft of error to 250 and
pins the command at -500 the whole way; the new one converges to zero. NOT yet
flown.

**And a route to a stand is never a departure's** (same sortie, one taxi
later). Full auto asks the tower for a taxi clearance unless it already holds
one, where "holds one" was `route_is_ours`: does the published route pass
within 200 m of us. An arrival's clearance ENDS where we are parked, so it
passes that test with room to spare, full auto adopted the taxi-in route it
had just finished driving, skipped the request entirely (no `request_taxi`, no
`tuned_tower` in the stage log), drove 33 s of an already-complete 88 m route
back toward the stand from 66 m off it, and announced itself at the holding
point. The tower's answer to the departure request was "taxi to holding point
Hotel Whiskey one FIRST", and the line-up failed. The party line already knew
which kind of route it had published (`taxi_to_stand`); nobody had asked it.

Depth (added 2026-07-28, evening): four traps of the same family, a fact one
part of the system knows and another needs. The taxi GUIDE narrated a
**pushback** ("Rolling again. Sharp left next, onto Bravo" with the tug
connected): groundspeed is a MAGNITUDE, so four knots backwards reads exactly
like four forwards, and the stand IS the cleared route's first point. It gates
on `pushback::ACTIVE` now, and on the route itself, a fix whose along-track
progress went BACKWARDS is never a moment for "next left". **Full auto** polled
`autotaxi::ACTIVE` to wait out a drive, but a BACKTRACK drops that flag in the
gap between its two legs; it read the gap as "finished" and called a running
line-up a failure one second after leg two engaged. `driving()` (ACTIVE OR
`BACKTRACKING`) is what a caller waits on. The ground drives (`auto_taxi`,
`line_up`, `taxi_to_parking`) were registered behind `!autothrottle` alongside
`hold_speed`, right for the AIRBORNE speed tool, since an airliner sets its
speed bug, and wrong for these: on the ground the taxi governor is the only
speed control there is. Neither jet had a taxi tool at all, and only full auto
worked because it builds its own copies. And one PANEL fact: a light can be two
switches (`also_datarefs`) and a system's only control can be a TOGGLE
(`write_datarefs`), the Citation X's landing lights are independent left and
right, and firing `cmd_avionics_toggle` as "on" turns the avionics OFF whenever
they were already on, which is worse than having no tool.

Related trap (2026-07-24): `Voice::alert` now
mirrors spoken lines to the panel transcript itself, any watch that ALSO
posts `s.action(same line)` double-writes the chat (the Jerez relocation
note; gyro-sync and autoland hit it first). Mirror OR action, never both.

**The panel is an automation instrument now** (2026-07-29 revamp). Tabs are
Flight | Transcript | ATC | Route | Settings, Flight first and default: the live
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
published change-gated from the monitor tick, NOT the 1 Hz radio thread,
which only exists when ATC is configured, with an unconditional resend every
5 ticks because a plugin reload zeroes the buffers and a change-gate would
leave the tab blank. Below the well, BUTTONS for every automation
(FULL AUTO / FLY PLAN / PUSH BACK / TAXI / LINE UP / TAKEOFF / CIRCUIT /
LAND + guarded RECOVER / MY CONTROLS): each injects the same English a pilot
would speak through the request channel (`auto_status::REQUEST_PHRASES`,
codes 4-10; a grammar test pins every phrase to its tool so a button can
never reach the LLM), and CANCEL (code 11) has no fixed phrase, the app
picks per what's running ("cancel full auto" outranks all, its abort
cascades), falling back to "my controls" when NOTHING is running
(2026-08-02): the old None fell exactly when the pilot most needed the button
, after the missed-approach give-up every ACTIVE flag was already false while
the AP still flew the aircraft dirty and slow, and CANCEL was silently
consumed with nothing to say. An abort that goes dead precisely when things
have gone wrong is worse than none; CANCEL always aborts SOMETHING now, and
the fallback is the universal hand-back (autopilot off, every hold released).
Mechanics worth knowing when diagnosing "the button did nothing": presses
ride the same typed-command mpsc queue the keyboard uses, consumed one per
session-loop pass, so presses fire in order and a press can wait out a busy
loop, but the phrase (including the cancel pick) is resolved at PRESS time
in the poller thread, so a queued CANCEL still cancels what was running when
the pilot pressed it. The old Calls tab folded into ATC (coach block on top, radio
chips, stations, phraseology reference in CollapsingHeaders; the
readback-owed dot moved to the ATC tab); the shared header rows are gone ,
each tab owns its space, and the status bar gained an AUTO annunciator lit
from any tab. Type is B612 (the Airbus cockpit face, OFL, TTFs +
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
line in the monitor, one in the session loop), copy-on-write, so a tick that
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
copy of the schema, which is how the in-sim panel's three bespoke pickers can
move onto it later without inventing a third precedence chain. The rule that
matters is the pin: precedence is env > saved pref > default, so a knob whose
variable is set (including from `.env`, which dotenvy has already loaded) is
REFUSED with the variable named on screen. Accepting that edit and letting the
next boot silently revert it is the "always end up back with ryan" bug wearing
a new face, and a settings screen is exactly where it would happen again.
Values persist to a generic `settings` map in prefs.json, with the four
existing typed fields left alone, mode and cabin crew are `Persist::Own`,
since the panel protocol already owns their boot path, and a second one here
would be two precedence chains for one value. Stored forms carry no unit
(a "900 ms" its own parser rejects would forget itself at the next boot, which
a test pins), and a value outside the declared range, legal, because env vars
are parsed by config.rs and not by the registry, renders fine and is pulled in
by the first keypress.

**Web API load is a sim cost, and it is measured** (2026-07-28). The API is
served from inside X-Plane's flight loop, so every request is frame time the
sim does not get. Two hangs that day could not be attributed either way,
because the logs recorded only FAILURES, nothing ever counted the requests
that succeeded. `crates/sim/src/xplane.rs` now reports rate, cache hits,
failures and the slowest response every 30 s, to the log and the blackbox
(`"kind":"api_load"`). The first numbers were 70-80 req/s sustained, with the
per-window slowest response climbing 50 → 100 → 125 ms in the minutes before a
freeze: the shape of load, not of a single event. Reads are served from a 60 ms
value cache (shorter than any control tick; a write drops its own entry so the
readback every tool's trust rests on still sees the real post-write value) ,
but the DUPLICATION that cache was built for is NOT where the volume comes
from. Measured, it recovers about 2.5%, because the loops each read DIFFERENT
datarefs rather than the same ones twice, and a whole EGHI sortie logged
`cached=0` in every 30 s window. Do not read that zero as a broken cache; it
is the honest answer, and it is why the load report also records a per-dataref
histogram to the blackbox (`top`), which is the only thing that can actually
name a hog. That sortie's: latitude and longitude 7,312 each, then
`is_in_replay` 3,803 and `paused` 3,791 (the freeze check, ~4/s between them,
for two flags that change almost never, the obvious candidate for a longer
per-dataref TTL), `onground_any` 3,723, elevation 3,375, groundspeed 3,070,
psi 2,877, over 61 windows. Whether any of this causes the hangs is still
unknown: the per-window slowest response reached 4,533 ms mid-flight, and the
plugin (in-process, drawing every frame, never profiled) and LiveTraffic remain
the other two suspects.

Battle-test surface (all deterministic, in `cargo test --workspace` unless
noted, plain `cargo test` at the root only runs the crew package now): the
state-by-intent matrix test in [crates/atc/src/tower.rs](crates/atc/src/tower.rs); the adversarial
transcript corpus [crates/atc/bench/atc_corpus.yaml](crates/atc/bench/atc_corpus.yaml) (real whisper
output mined from flight logs, add every new wart here); scripted sortie
dialogues over the pure routing layer (`route_keyed`/`route_unkeyed`) in
[crates/atc/src/station.rs](crates/atc/src/station.rs); the spoken-call eval judged on intent
accuracy (`cargo run -- stt-eval bench/atc_stt_cases.yaml`); the cross-airport
world sweep (`cargo test --release -p atc -- --ignored atc_world_sweep`, needs
the sim's apt.dat); and the in-sim soak checklist
[docs/atc-chaos-card.md](docs/atc-chaos-card.md). Every routing decision is
blackboxed as a `"kind":"atc"` event, mine `logs/blackbox.jsonl` after each
flight. Safety property: a mishear may earn "say again", never a clearance.
The tracing filter must name EVERY workspace crate
([src/main.rs](src/main.rs) defaults to
`xvoicelayer=info,xvl=info,atc=info,sim=info`, and the crew needs BOTH of its
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
still buildable, tower + ATIS glue around the same lib modules) on the "the
world is a different process" theory. Flying it proved the split premature:
mic/PTT/voice are single shared resources and the second terminal was
unusable in the cockpit. The right split landed 2026-07-14 as a **compile-time
boundary, not a second process**: the world lives in the `atc` workspace crate
(`crates/atc`, station, tower, nav, atis, party_line, speaker identity), both
sides stand on `crates/sim` (Web API client, dataref names, blackbox), and the
copilot embeds the world as a library, still one process, one mic, one PTT.
The atc crate returns `(text, StationSpeaker)` and never speaks or actuates ,
its sim handle is `sim::xplane::SimView`, a read-only view with no writes on
it, so "the world never touches the controls" is a type, not a promise; its
dep tree must stay free of whisper/cpal/ort (`cargo tree -p atc` is the
standalone-product health check, and a future ATC-only product is a repo
extraction away). Next: live-traffic
observation via TCAS datarefs (ATC only ever *controls the player* and
*narrates* other traffic), a radio-PTT plugin dataref if content routing ever
collides.

### The region handoff had no crew consumer (2026-09-14)

The enroute watch's "Radar services terminated. Squawk seven thousand.
Contact Malaga Tower on one one eight decimal one" was the world's line and
nothing else: the station armed `PENDING_STATION`, the main loop installed
the new tower, and the crew did nothing with the words. Full auto stayed on
the departure frequency until its own arrival stage tuned the destination
(the thirteen minutes of Malaga's party line that the departure handoff fix
of 2026-07-24 only half cured), the transponder kept the discrete code for
the rest of the flight, and a pilot flying by hand heard no coach either.

Same shape as `take_departure_handoff`: the station publishes
`party_line::RegionHandoff { to, khz, squawk_vfr }` with the spoken line and
`take_region_handoff()` is the one accessor. It survives `Station::install`
ON PURPOSE (install runs on the main loop's very next pass, before the crew's
2 s poll; the departure handoff dies there because the OLD tower's climb
handoff must not fire at the new field) and `uninstall` ends it. Full auto's
enroute wait reads it back on the old frequency ("squawking seven thousand,
Malaga Tower on ...", an `is_readback` shape so the controller answers with
silence), waits the readback beat, dials 7000 through `SetTransponder`,
retunes COM1 and blackboxes `region_handoff`; the arrival stage, which has
already tuned the destination, drains it without a readback (a readback
there lands on the NEW tower as a garbled call) and still gives the code
back. With anyone else flying (`full_auto::ACTIVE` false: the pilot, or the
bare follower) the monitor takes it and after 20 s says once what is still
undone (`monitor::region_coach_line`: "Radar's done with us: squawk seven
thousand, and Malaga Tower on ..."), blackboxed `region_coach`, never
re-asserted. Not done: the pilot's own spoken readback is not checked
against the handoff (the station stays silent to any readback shape).

## App modes: Both / Copilot only / ATC only (runtime-toggleable)

The two halves sell separately (2026-07-14): `mode.rs` holds one process-wide
mode, **both** (default), **copilot** (no simulated ATC: no station ever
loads, radio-keyed speech is IGNORED because real/external ATC owns the
frequency, no void briefing talking over a VATSIM controller), and **atc**
(the world without the crew: deterministic tower/ATIS answer the radio, no
intercom persona, no tools, and the LLM is never touched, an ATC-only boot
needs no Ollama at all). Chosen from the plugin's Settings tab
(`xcopilot/opt/mode`, radio buttons; `spawn_mode_poller` syncs it), persisted
via prefs.json; `XCOPILOT_MODE` env sets only the BOOT value (env > saved pref
> both), a panel flip wins from that moment and applies live within ~2 s (the
main loop's idle tick reconciles). Live transitions: ATC off =
`Station::uninstall()` (retires ALL published world state, party line + side
state, the same page install() writes) + radio-channel cut; ATC on = the same
`install_home_station` path boot uses (background scans guard against paying
boot costs twice, `region_loaded()`); copilot off = intercom cut + Ollama
evict (`Llm::evict`, keep_alive 0, reclaims the GPU frames); copilot on =
re-warmup off the critical path. Monitor watches split along the same seam:
crew watches (callouts, debrief speech, taxi guide, landing coach, talk-down)
gate on `mode::copilot_enabled()`; world ticks are inert once uninstall clears
their state, except `region_tick` which gates explicitly (its airport index
survives for cheap re-enable). In ATC-only, content routing still reaches the
station when no radio key is bound (it's the only path to ATC there), and the
crew speaks exactly one line ("copilot is off") on the first intercom attempt.
The atc crate itself never reads the mode, the world stays mode-blind and is
simply not driven when off.

Depth (added 2026-07-15): the "no crew in ATC-only" rule moved from scattered
per-site gates into the arbiter itself, `Voice::alert` / `alert_stale_by` /
`chime` (and the cabin paths) drop at the one enqueue door when
`copilot_enabled()` is false, so no background watch can forget the gate (the
void-transmission "No answer there" brief, the carry-forward brief, gyro sync,
and the crew branches of the world watches all leaked before). Blocking `say`
is NOT gated there: the main loop mode-routes it, and the one-line "copilot is
off" note must speak. The gyro auto-sync ACTION (not just its line) also
stands down: no crew, no hands on the panel. Tests that set the process-wide
mode hold `mode::TEST_LOCK`.

## The follower flew true altitude; the dial is indicated (2026-09-14)

VJF-LINTO leg on the 172, cruise 7,000: the AP sat at 6,820 indicated with
ALTS armed and VS +4 fpm, never capturing. The blackbox `leg` events said
`alt_ft` 7,009 against `want_alt` 7,000, so the follower commanded VS zero
and was content. Live reads: `sim/flightmodel/position/elevation` 2,137 m
(7,012 ft), `altitude_ft_pilot` 6,838, altimeter 30.15, OAT 61F (ISA+14C).
A hot day puts true altitude above indicated; the follower targeted true
feet from the elevation dataref while the AP's preselect, the plan, the
chart and the controller all mean the altimeter.

Fix: the follower's `alt_ft` is now `altitude_ft_pilot` (true elevation
only as a fallback when the read fails); the terrain floor, which IS true
feet (probed ground plus clearance), converts through the live
true-minus-indicated offset (`procedure::floor_indicated`, tested both
ways) so a hot day never eats the 1,000 ft. Full auto's departure check-in
("passing two thousand") and the arrival's `approach_flyable_from` read the
altimeter as well. The `leg` event keeps `alt_true` beside `alt_ft`.
Consequence to remember: a mis-set altimeter is now a mis-flown altitude,
exactly as it is for a pilot. The circuit still sizes its pattern from true
elevation (field plus 1,000 AGL), which is what a pattern height is.

Which raised the next question, the same afternoon: who sets QNH? Until
now, full auto once at the gate (from `qnh_pas`, never again), the pilot by
asking, and nobody for "fly the flight plan" (that sortie: 30.15 on the
knob, QNH 30.10, 46 ft of the gap the altimeter's own). So: one rule for
the correct setting (`tools::instruments::correct_setting_inhg`, QNH below
the profile's transition altitude, 29.92 above) shared by the spoken tool,
full auto's departure set, the follower's arm (`Plan.transition_altitude_ft`,
`Some` on the enroute plan only; the knob is set and said only when it
moved) and a new monitor check, `AltimeterSetting`: 0.03 inHg off for 20 s
with the engine running. Under an automation (follower, full auto, circuit,
autoland: `Snapshot::automation_flying`) the copilot resets the knob through
the new `Check::fix_write` hook ("Altimeter reset, three zero one zero")
and blackboxes the callout; with the pilot flying it advises once
("Altimeter check, QNH is three zero one zero, you have three zero one
five") and never turns the pilot's knob, the cockpit_auto rule. Both
datarefs joined `SLOW_DATAREFS`. Still owed: reading the setting the tower
gives in a clearance back into the knob (`qnh_clause` is one-way today).

## The mouth and the hands disagreed on top of descent (2026-09-14)

LEJR-LXGB full auto on the 172, straight-in to 27. The enroute completed
at 7,000 and the approach armed with 27D10 (reconstructed at 3,143 ft)
44.7 nm out. At 11:18 the monitor said "Descent coming up, top of descent
in about 6 minutes", at 11:25 "We're past the descent point. About 1100
feet a minute makes the profile", at 11:28 the spoken answer to "how long
before top of descent" was "We're at top of descent, time to start down".
The follower held 6,998 ft through all of it and started down at 11:31,
11.2 nm from 27D10, exactly where `descent_target_ft` puts it (700 fpm to
the NEXT fix plus a mile of lead). Nothing was late: the advisor and the
answer measured a different point (SimBrief's ring, else 3.5 nm per
1,000 ft to the FIELD), and the pilot asked three times whether the
copilot knew.

Fix: `procedure::descent_point_nm` is the one formula, and the follower
publishes `PlannedTod` (fix, miles to it, miles to the descent point,
target altitude, started) every tick, cleared with the loop and gated on
`ACTIVE` like `active_leg`. The monitor's snapshot carries it: `tod_ahead`
speaks the follower's point and altitude ("12 miles ahead, down to 3,100
feet"), `tod_passed` fires only on the follower's OWN miss (a mile past
its point with the descent still uncommanded, the rate to make the fix),
and `high_on_profile` stands down (the follower's 700 fpm to a fix is not
the ring's 300 ft/nm to the field). `time_to_tod` reads it before the FMS
and the plan, and says "already descending, down to X feet at FIX" once
the descent is commanded. Not changed: the follower's own slope. From
7,000 to 27D10 it is 3.3 degrees at 700 fpm, flyable on the 172 but the
whole descent in one step; a stepped profile is a separate decision.

## The tower said 09 and the copilot landed on 27 (2026-09-14)

Same LEJR-LXGB sortie, the arrival. Full auto read the aircraft's wind
datarefs 30 nm out at 7,000 ft (a light westerly aloft, below the 4 kt
decisive threshold), picked the end by geometry, and set up the
straight-in to 27. At 11:40:30 it reported "final runway two seven";
autoland armed two seconds later and read the party line's clearance
runway once, at the arm: nothing there. At 11:40:37 the tower answered
"negative, runway zero niner, I say again runway zero niner. Runway zero
niner, wind zero niner zero degrees, two one knots, cleared to land"
(`correct_final_runway`, the LEMG 12/13 fix, working exactly as built).
The words were spoken over the cockpit and nothing read them: no readback
went out, and the approach flew 27 with 21 kt behind it. Two smaller
misses on the way in: the inbound call at 11:12 was made 10 minutes before
the region swapped from Ibn Battouta (genuinely the nearest field across
the strait) to Gibraltar, so it went into the void and the final report
was the tower's first word from us; and the crew has never read back a
landing clearance, only takeoff and line-up.

Fixes, each with the transcript as its test:

- **Autoland watches the tower's word every tick** (`clearance_switch`, a
  pure decision; the tick block above the rollout law). An arrival-shaped
  clearance (Inbound or ClearedToLand: a departure state's runway is the
  departure's) naming a different end is looked up in apt.dat once, and if
  it is at THIS field the loop announces "Tower cleared us for runway zero
  niner, not two seven", blackboxes `runway_from_clearance` with
  `in_flight`, calls `run_autoland` again forced onto the cleared end and
  exits on the generation bump. The re-arm is the whole approach from the
  top: the fresh loop repositions to 09's join point from wherever we are
  (the Intercept phase's job) and the arm-time clearance check finds
  forced == cleared and says nothing. A superseded loop no longer clears
  `ACTIVE` on its way out (`GEN == my_gen` guard): the successor set it
  for itself, and clearing it told full auto "landed or went around".
- **The landing clearance is read back** by the loop that is flying:
  `party_line::readback_owed` is the phrased echo the tower waits for
  (`ClearanceSnap.pending_readback`, already published); a landing one
  (`landing_readback`, never a hold or a line-up) goes out once per
  clearance as "cleared to land runway zero niner", the runway spelled
  the way the proven takeoff readback spells it. Blackboxed
  `landing_readback`.
- **The arrival's wind is the field's** (`weather::surface_wind` from the
  real-weather METAR file, fresh per `staleness`, variable = none;
  `arrival_wind` falls back to the aircraft datarefs and names the source
  on the `arrival_*` event). The tower's own pick is from the sim wind at
  the aircraft when it is near the field, which the METAR is the surface
  proxy for; a mismatch now self-corrects through the watch above.
- **The inbound call is deferred to the swap** (`inbound_owed`,
  `call_inbound`): 45 s at the arrival's start as before, then the
  final-call watch loop polls `home_tower_khz` and calls the moment the
  destination's station is up, draining the region handoff there; a field
  that never swaps gets the inbound right before the final. Blackboxed
  `inbound_deferred`.

Not done: the deferred inbound on a PROCEDURE arrival still cannot move the
plan (the reply's "join final runway zero niner" is obeyed only once
autoland has the aircraft, since the procedure's FAF is 27's); a
switch during the procedure leg would be an `EndAction` rewrite.

## The flare arrived at the dial, not at the wheels (2026-09-14)

Same LEJR-LXGB sortie, the last ten seconds. The debrief graded the
touchdown firm (309 fpm at 62 kt), the third firm 172 arrival in three days
(LEMG 13 on the 12th, LEJR on the 13th, both around 300), and the tick log
says exactly why. The vertical law did everything it was written to do:
`cmd_vs` stepped to the roundout's -250 at 49 ft radio and to the flare's
-100 at 23 ft, on time, over the pavement, on the centreline. The `vsi`
column never followed. Four seconds after the roundout dial the aircraft was
still sinking 386; the flare's -100 eased it to -257 and then, with the
throttle closed, the sink GREW back to -309 at the wheels. Fitted to the
log, the 172's VS mode follows its dial as a first-order lag of about
4.5 s behind a tick of dead time, and a flare from 23 ft at approach sink
lasts five seconds: the whole manoeuvre was shorter than the servo. The
guidance was right and the autopilot never heard it in time.

The fix is the plant's, not the law's: in the roundout and the flare the
DIAL now leads the target by the error the autopilot still owes
(`vs_dial_fpm`: target + 2 x (target - actual), capped at a +300 fpm
"climb" so a balloon is never commanded), which makes the closed lag about
1.5 s. The dial may read a climb for a moment near the ground; that is the
yoke coming back, which is what a flare is. Above the band nothing changes:
the glidepath's own P law tracks its ramp through `path_descent_fpm`, and
every approach test stays byte-identical. `Guidance` carries `arresting` so
the loop knows which regime it is in, and the autoland tick blackboxes
`vs_dial` and `vvi` beside `cmd_vs`, so the next debrief can tell what the
law wanted from what the autopilot was told from what it flew.

The test is the sortie: `the_landing_arrest_leads_the_autopilots_lag`
flies the real vertical law from 60 ft against a lagging VS mode AND the
flare's throttle cut as a 250 fpm sink disturbance the same lag has to
reject. The disturbance matters: a pure lag lands the bare dial at -140 and
would have passed the old law (the plant, again, before the gain). With it
the bare dial reproduces the live arrival (-330, 10 s) and the led dial
arrests it to about -100 in 14 s, no balloon, five seconds of extra float
the overflight allowance already budgets for.

Not done: the flare is still flown through the autopilot. A servo this slow
argues for the takeoff's answer, the `xcopilot/pitch` channel with the
rotation law's Q damping, which is a pitch-attitude flare a real hand
would fly; if the led dial still arrives firm, that is the next step, and
the tick log will say so in the `vs_dial`/`vvi` columns.

## The circuit orbited the bay: the AP will not reverse a turn (2026-09-14)

LXGB, 172, "let's do the circuits" from cold and dark, runway 09, the
authored Rock circuit. Four faults in one sortie, each with the blackbox
line that names it.

**The takeoff sank at the AP engage.** Trim was at the nose-down stop from
the previous flight (the monitor called it four seconds after liftoff). At
325 ft AGL the AP took the climb with the dial at 584 fpm and the aircraft
went to -1,040 fpm, down to 178 ft AGL, 83 to 111 kt, before the servo wound
the trim back; 500 ft came 1.8 km late. The takeoff loop now sets the trim
into the takeoff band before the brakes come off (`controls::trim_for_takeoff`,
`trim_set` event) and records the trim on `ap_engaged`.

**The first leg went the wrong way.** The route was authored with waypoint 0
(the climb-out point) 1.8 km BEHIND the aircraft, and the follower steered
back to it: a 270-degree turn from 500 ft, climbing at a flat 700 fpm with
the throttle law holding pattern speed. IAS fell to 60 kt in the turn (Vx,
1.26 Vs). The authored follower had its own VS clamp and never called
`climb_limit_fpm`; the rectangle loop always had. Now it joins at the nearest
leg (`join_waypoint`, `authored_join` event) and climbs only as fast as the
airspeed pays for.

**The downwind crossed the Rock.** At 357 s the ground under the track was
998 ft and the aircraft 523 ft above it, at the 1,650 ft pattern altitude.
`authored_route_clears_terrain` probed the six WAYPOINTS, which sit on sea
either side of the ridge; the leg between them crossed it. The rectangle's
own polyline had the same hole (eight downwind samples 580 m apart reported
650 ft under a side whose ridge stands at ~1,150). Both are sampled every
150 m now (`leg_samples`, `densify`; `terrain::max_elevation_ft` chunks
past the plugin's 24-point query instead of silently truncating). Terrain
under a pattern-altitude leg RAISES the pattern altitude
(`raise_for_terrain`, said aloud, `authored_route` carries `raised_ft`);
terrain under a descending leg or a raise past 1,000 ft refuses the route.

**Then five minutes of orbits.** Capturing waypoint 2 the aircraft was
banked 4.3 degrees right for it; the bug stepped 25 degrees LEFT for
waypoint 3 and the AP rolled on to 12 right and went round the long way. A
live read confirmed heading mode engaged, the bug written and read back
identical, the bug on the left, the aircraft turning right. X-Plane's
heading mode does not reverse a turn in progress, and pure pursuit keeps the
bug moving in the direction of the turn, so it never caught up: standard-rate
circles drifting downwind across the bay, each later waypoint caught only
when a circle happened to pass within 350 m, and the copilot silent
throughout. `bug_toward` now holds the bug ON THE NOSE whenever the wanted
turn is against a bank over 3 degrees (a capture is the one thing that rolls
that AP level), then slews the short way from level. Behind it a watchdog
(`Closure`, `authored_stalled`): no new low on the distance for 45 s levels
off and re-steers, three of those and the aircraft goes back with heading and
altitude held.

Also fixed: the climb checklist under a circuit flagged "throttle, full"
(the circuit's speed law owns the lever; `held_by_automation` names it now)
and the landing light stayed on (the authored loop never called
`lights::after_climb`; it does at 1,000 ft). Still open: the tower handed a
circuit to Departure at 719 ft because the takeoff request never said "for
circuits"; nobody answered, which was right, but the request should carry it.

## The Rock circuit went over the Rock (2026-09-15)

Runway 09, the 172, "fly a circuit". The authored loop's downwind sat
1.3 x the pattern width from the runway line: 2,050 m, which at Gibraltar
is Middle Hill, not the sea. The leg probe (150 m samples, the 2026-09-14
fix) read 1,098 ft under the downwind, `raise_for_terrain` found the
2,108 ft pattern altitude already 1,010 ft above it and passed the route
with `raised_ft: 0`. The follower joined leg 1 at 500 ft, turned onto the
downwind at 1,130 ft and crossed the ridge at 1,412 ft climbing 1,000 fpm:
454 ft over 958 ft of ground (`state` records, t+204 s), on a route the
check had called clear by 600 ft. The screenshot from the map showed the
aircraft south-east of the field pointing at the east face.

Two wrong assumptions, one each:

- **The downwind was scaled to the airframe, the Rock is not.** The
  "around the Rock" bulge was a multiple of the turn radius; a 172's
  pattern width is 1,577 m and the Rock runs 4,500 m south of the runway
  line to Europa Point. `LXGB_ROCK_CLEAR_M` (5,400 m) is now the floor for
  the downwind offset on either side: crosswind south over the sea off the
  east face, downwind over the Strait 800 m south of the point, base back
  north over the bay (or the Mediterranean on 27), the final handoff where
  it was. Every waypoint checked against the real 09/27 thresholds: sea.
  The loop is bigger (about seven minutes) and goes AROUND the granite,
  which is what the words meant.
- **"Flown at pattern altitude" was assumed, not reached.** The terrain
  check compared each pattern-altitude leg with the altitude the route
  would eventually hold, and the follower leaves the takeoff's handoff at
  500 ft and climbs along the route. `raise_for_terrain` now carries a
  climb model (`climb_reach_ft`: field elevation plus
  `authored_climb_gradient`, 75% of the follower's 700 fpm clamp at the
  pattern speed, over the path flown from the threshold) and refuses a
  leg whose terrain needs more than the climb can have reached at the
  leg's START ("entered at N ft in the climb"). With the new geometry the
  Rock is under no leg; the check is there so a future waypoint that puts
  it back is refused on the ground instead of cleared by 454 ft. Pure,
  tested (`a_leg_entered_below_its_terrain_in_the_climb_is_refused`,
  `the_rock_circuit_downwind_runs_south_of_europa_point`).

The refusal falls back to the rectangle as before. The base is now 4.4 km
long, so the descent from a raised pattern altitude to the 711 ft handoff
has the room the old 1.2 km base never had; not yet flown in the sim.

## Cabin crew: the second intercom persona (2026-07-15)

Channels are physics, personas are tenants: the intercom now carries TWO
voices, the copilot and the cabin crew ([src/cabin.rs](src/cabin.rs)), with
no third PTT and no third audio channel. The address IS the routing ("cabin
crew, prepare for landing", an explicit "cabin crew"/"flight attendants"
prefix always routes; bare "cabin" routes only with an unmistakable crew word,
because cabin lights/pressure/the Lancair's cabin dump switch are copilot
business). Calls are answered deterministically (keyword intents, "say again"
over guessing, the tower's safety property), always preceded by the
interphone chime, in their own voice (`XCOPILOT_CABIN_VOICE`, lazily loaded so
crewless flights never pay for the model). The crew exists per-airframe
(profile `cabin_crew: true`, the MD-82 only, so far); staffing is a panel
setting like the app mode (Settings tab radios Auto/Aboard/Off →
`xcopilot/opt/cabin_crew`, synced by the settings poller, persisted via
prefs.json, applied live at the routing sites, staff a 172 mid-flight);
`XCOPILOT_CABIN_CREW` sets only the BOOT value (env > saved pref > auto).
Behavior: "prepare
for departure/landing" is acknowledged then owes a delayed "cabin secure"
report (spoken by the monitor's cabin watch, a SAFETY report, exempt from
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
tools, `set_flaps(notch)`, `gear(up|down)`, `set_lights(...)`, `set_freq(...)` ,
where each tool owns:

1. its dataref + valid range (so "flaps up" can't accidentally write throttle),
2. a **readback** that re-reads the dataref *after* writing and confirms the
   actual value, a real copilot confirms the *action*, not the *intent*,
3. **safety gates** (e.g. "gear up" while on the ground → refuse + explain why).

## v1 milestone

Smallest flyable slice, end to end: **push-to-talk → whisper → one `set_flaps`
tool → piper readback.** If that single loop feels good in the cockpit, the rest
is just more tools. Don't over-build the framework before that loop works.

## Licensing

**`GPL-3.0-or-later`** (2026-07-30, `LICENSE` at the root, declared once in
`[workspace.package]` and inherited by every crate). Two things are now settled
and not open questions: this is **open source, never commercial**, and it is
**X-Plane only**, no MSFS port, whatever the incumbents' platform reach. Only
the product NAME stays parked.

Copyleft over the dual `MIT OR Apache-2.0` the Rust convention would suggest,
because the threat here is specific and demonstrated: the flight-sim ecosystem
is payware-dominated, and Skylink Simulations sells this exact product shape
(deterministic ATC + voice copilot + cabin crew) for $49. Under a permissive
license the tower FSM, the taxi planner, the wake-separation logic and the
ground-drive laws could be lifted straight into closed payware. The usual
argument FOR permissive is commercial adoption, which is a LIBRARY concern ,
this is an end-user application, so it costs us nothing to refuse. It is also
the reversible direction: a sole author can always relicense GPL code as
permissive later, but MIT grants already handed out can never be clawed back.
Two bonuses: `3.0` specifically because several deps are Apache-2.0 (compatible
with GPLv3, NOT GPLv2), and matching Welly's ATC now makes his IFR/live-ATIS
code legally liftable, where the old rule was patterns-yes-code-no.

The linking question people assume blocks this doesn't: `plugin/SDK/license.txt`
is plain MIT/BSD, not a proprietary SDK license, as are Dear ImGui (MIT) and
ImgWindow (BSD-3-Clause); B612 stays OFL. All GPL-compatible, all keep their own
terms. Sole-author relicensing stays free only until outside contributions are
merged, settle any change of heart BEFORE accepting a PR. One consequence to
keep in mind: `crates/atc` is built for extraction, and GPL means any future
consumer of it must be GPL too, `LGPL-3.0` on that one crate is the escape
hatch if it should ever be broadly embeddable, deliberately not done now.

## LXGB-LEAX in the 172: the arm under a capture, a threshold 43 ft below its header, a strip that could not win its own sky - 2026-09-15

Full auto, Gibraltar to Leoni Benabu, five enroute fixes at 8,000, the
visual landing (LEAX has no coded approach). Four findings from the blackbox,
three fixed, one left as it is.

1. **"Autopilot is not holding the descent" at the autoland arm (13:52Z).**
The follower's last leg dialled PIZCU's 2,600 ft; the aircraft was at 2,733
descending 700 fpm when the arrival handed to autoland, i.e. INSIDE the AP's
altitude capture, which was already arresting the descent (vsi -697 to -182
over ten seconds, before the arm). Autoland wrote -1,200 to the VS dial and
the AP, mid-capture, ignored it; flaps 10 went in at 100 kt and the aircraft
ballooned to +295 fpm; after 12 ticks the divergence watchdog re-pressed VS
mode and the descent resumed at -1,300 within a second. The per-tick
`VS_STATUS <= 0.5` re-select never fired because `vvi_status` stayed 2 through
the capture. Autoland had never touched the altitude dial in its life; it
now parks it at the arm (`parked_altitude_dial_ft`, 1,000 ft above the arm
altitude rounded up to the hundred, only when the dial is within 500 ft
above or anywhere below) and re-selects VS once, blackboxed
`alt_dial_parked`. A go-around's climb captures the parked level at worst,
and the per-tick re-select takes it down again from an ALT hold (status 0).

2. **"Going around", overflew the runway, at 52 ft radio 300 m past the LEAX
30 numbers (14:00Z).** The approach was clean: 65-67 kt, on the 3° path to
the foot, cross-track 30 m, trim 0.5°. The path was aimed at the apt.dat
header elevation, 160 ft; the runway 30 threshold's mesh is at ~117 ft
(`alt - agl` at the crossing: 216 - 99), so the aircraft crossed the
threshold 99 ft up instead of ~57, never entered the roundout band, and on a
1,062 m runway the float allowance is the 0.15 nm floor, so "overflew" fired
at 0.16. The arm now probes the sim's own mesh at the threshold and the
touchdown aim (`terrain::sample_ft`, two points) and aims at the higher when
it differs from the header by more than 20 ft and less than 300
(`aim_elevation_ft`, blackbox `threshold_elev`, said aloud: "Runway three
zero threshold is 39 feet below the field elevation. Aiming at 121 feet.").
The test pins the LEAX crossing height either way.

3. **Inbound and final calls into the void; no LEAX station ever installed.**
The region pick is service-weighted (`region_services` x 3 nm, the Trebujena
rule): Malaga's ATIS/ground/approach/departure rows pull it 12 nm nearer than
it is, and Leoni Benabu's one tower row pulls nothing, so at 4 nm from the
strip the arithmetic was 4 + 5 against 17 - 12: Malaga keeps the sky over
LEAX's own runway for ever. The crew had tuned 123.5 and waited 45 s for the
swap that could never come. The crew's retune to the destination's tower now
calls `station::arm_station_for_freq` - the pilot's "you call it, you get it"
rule (2026-07-10) applied to the crew's own hand on the knob - and the
existing swap wait finds the station on the next loop pass. Test:
`a_tower_only_strip_loses_the_region_but_wins_its_own_frequency`.

4. **`terrain_probe_unavailable` at 13:12Z, the floor off for the whole
flight.** The pilot paused 0.6 s after a probe's query went out; the
two-second wait for the plugin's `gen` bump ran into the pause and the None
was read as "no plugin". `terrain_probe_fatal`: a miss while `sim_frozen` is
not counted, two live misses in a row retire the floor, one is deferred
(blackbox `terrain_probe_deferred`).

Left alone: the region handoff's readback (13:26Z) went "void" because the
main loop consumes the pending station on the pass after the monitor arms it,
and the crew's readback keys two seconds later on a frequency nobody holds.
It is a readback to a controller who has just signed off; harmless, and
routing it through the departing station would mean holding the swap for the
crew. Noted, not fixed.

The copilot process ended at 14:00:19Z, seven seconds into the go-around,
with no shutdown line and no macOS crash report; the sim was found paused at
100 ft AGL in the climb. Cause unknown (a Ctrl-C at the pause is the simplest
reading).

## LEJR-LEGR: the inbound call made at the zone, not at the last fix - 2026-09-15

Evening sortie, the 172 from Jerez to Granada under full auto. The enroute
region handed the flight to Granada Tower at 21:34:36Z, twenty miles out
("radar services terminated, contact Granada Tower on one one eight decimal
eight five"); the crew read it back, retuned and flew on. At 21:40:10Z, five
and a half minutes later and inside Granada's 5 nm zone, the tower's
compliance watch challenged: "Aircraft in the Granada control zone, you are
inside controlled airspace without clearance. Say callsign and intentions."
Again at 21:42:10Z and 21:44:12Z. Nobody answered. The inbound call went out
at 21:44:31Z, at `enroute_complete`, because the plan's last enroute fix is
GRA, a mile from the runway, and the inbound call lived in the arrival stage
that only starts when the follower finishes the last leg. The blackbox shows
it plainly: three `atc` events with `route: compliance, crew: false` and no
crew transmission between the `region_handoff` stage and `inbound_call`.

Two rules, both in the enroute wait (`wait_enroute_working_the_radio`):

- **The inbound call is owed at the zone approach.** Once the region has
  handed us to the DESTINATION's own tower (`home_tower_khz` equals the
  destination's apt.dat tower row, never a transit field's, which is the
  LINTO/Tangier lesson again) and the field is inside `INBOUND_CALL_NM`
  (10 nm, twice the tower's zone), the enroute leg makes the call
  (`inbound_due`, pure and tested; blackbox `inbound_enroute` with the
  distance). `INBOUND_MADE` tells the arrival stage the tower already knows
  us, so it skips its swap wait and call (`inbound_already_made`).
- **The tower's question is data the crew can hear.** The zone watch's tuned
  challenge arms `party_line::ZONE_CHALLENGE` (a new accessor,
  `take_zone_challenge`, drained every crew pass so a stale question never
  answers a later one); from the destination's tower it earns the inbound
  call at any range, in the enroute loop and in the arrival's owed-call
  loop alike. A challenge from a field we are only crossing is blackboxed
  (`zone_challenge`, `answered: false`) and left: the tower has no transit
  intent yet, so there is nothing true to say to it.

Still owed from the same tape: the region handoff readback goes into the
void every time (`outcome: void` on "roger, Granada Tower on ...", and on the
14th's "squawking seven thousand, Malaga Tower on ..."): the region switch
uninstalls the old station a tick before the crew's readback is keyed on the
old frequency, so nobody is listening. Harmless on the air, wrong in the
debrief's narrative; a readback to a controller who just said goodbye should
not be judged a void call. And a zone transit intent, so a crossing field's
challenge can be answered honestly.

## The plan preflighted at the gate: fuel, load and both runways before an engine turns - 2026-09-15

Asked whether "fly the flight plan" checks the fuel and the runway, the
honest answer was no. The arm (`tools/full_auto.rs`) checked that the plan
departed from this field, had enroute fixes and a destination the data knew
(`brief_arrival`, added after the Gibraltar crash); the block-fuel-versus-
tanks line lived only in the spoken plan summary and full auto never called
it. The monitor's pre-takeoff fuel caution compares endurance to the
profile's reserve, not to the trip, so a three-hour plan with two hours in
the tanks departed without a word. The departure runway was judged at the
line-up (the roll x 1.4 against the pavement ahead) after the start and the
taxi; the destination's length was never judged for a SimBrief plan at all
(only the local planner had `runway_refusal`).

SimBrief's fetcher carries the load sheet (`weights`: `pax_count_actual`,
`bag_count_actual`, `cargo`, `payload`, `est_tow`/`max_tow`,
`est_ldw`/`max_ldw`, in `params/units`) and the minimum takeoff fuel
(`fuel/min_takeoff`). The live plan that morning: LXGB-LEAX in the 172,
three aboard, 67 lb of cargo, 240 lb block, 227 lb minimum, planned at max
takeoff weight to the pound. `simbrief::PlanWeights` now parses it to kilos
(the sim's `weight/m_*` unit; the gallon figures go through avgas density
and are a third out on a jet) and remembers the plan's unit for speech.

`full_auto::preflight` runs at the arm with the sim's `m_fuel_total`,
`m_fixed` (payload) and `m_total` read in one join and both fields from one
apt.dat pass. Refusals, each with the number: destination shorter than the
landing distance (`runway_needed_ft`), origin shorter than
`takeoff_needed_m` (the line-up's rule, now shared), all-up weight over the
plan's max TOW (else the profile's `max_gross_lb`), fuel under the plan's
minimum. Said, not refused: fuel between the minimum and the block (both
figures), and a payload the sim disagrees with by more than 10 percent and
20 kg. The ack then says who is aboard. Everything unreadable or absent is
skipped: a local plan has no sheet and is briefed on fuel alone, a field
with no rows is `brief_arrival`'s. Tests: `a_plan_that_fits_is_briefed...`,
`a_runway_too_short_at_either_end...`, `overweight_and_under_fuelled...`,
`a_load_the_sim_disagrees_with...`, and the parser's
`the_load_sheet_parses_in_kilos...` over the fetcher's own strings.



## Procedure failure handling and the full-auto ground gate — 2026-09-15

Code review found four gaps: the procedure follower still called `slew_bug`
directly after the circuit's bank-aware reversal fix; failed altitude/VS writes
advanced their suppression caches; failed altitude reads became zero or another
instrument and a failed IAS read bypassed climb protection; either failed input
to full auto's airborne refusal could admit the ground sequence.

The follower now shares `bug_toward`, reads bank with the instrument burst, and
uses the circuit's `Closure` watchdog on positional legs. Two stalled intervals
re-steer from the nose; the third hands back and cancels an enclosing full-auto
mission so it cannot mistake failure for enroute completion. Paused and unread
ticks do not advance the progress clock. Missing/non-finite required instruments
skip the tick and blackbox `missing_measurement` with nulls; startup needs real
heading, variation and indicated altitude before AP engagement. The initial
unconstrained altitude is indicated, like the rest of the follower.

Altitude and VS suppression caches advance only after a successful write and a
readback within one unit of the intent. `write_unconfirmed` records the target
and actual (null when unread); failure leaves the target eligible for retry.
Full auto now requires both finite AGL at or below 50 ft and ground contact;
unread/conflicting inputs refuse before the ground stages (`ground_unconfirmed`).

Regression tests inject write errors, readback errors, silent refusal, missing
and non-finite instruments, contradictory ground state, and the LXGB 4.3-degree
bank reversal followed by a failed heading write and recovery. The existing
circuit closure tests cover orbit detection versus closing flight. Verified by
the xvl unit tests and the ignored local-install dataref-name check; these are
code/test findings, not a new live sortie.


## Worker ownership, explicit outcomes and locally planned full-auto flights — 2026-09-15

The next review followed cancellation through the actual worker lifetimes.
`ACTIVE=false` did not mean the worker had returned: a quick restart could clear
its shared abort flag, and old cleanup could erase the new run's status. The
procedure, takeoff and full-auto workers now hold `sim::run::Slot` leases with
monotonic identities and per-run cancellation flags through I/O and cleanup.
An immediate restart is refused until the old worker exits. Drop cleanup also
runs on panic. Scoped X-Plane clients reject mutations after cancellation,
including after asynchronous ID resolution and between HTTP retries; child
procedure/takeoff clients inherit the parent's flags. Procedure/full-auto
handoffs carry those flags into autoland, including a clearance-driven rearm.
An HTTP request already sent cannot be recalled; a replacement worker cannot
start over it because the old lease has not been released.

Outcomes distinguish completion, handoff, cancellation, pilot override and
failure. Full auto's enroute wait and the circuit/full-auto takeoff waits require
`Completed`, rather than interpreting every inactive worker as success. A
failed full-auto parent revokes its scoped children. The shared `control`
guards require finite instrument samples and confirmed dial writes. Takeoff
now includes pitch, bank and pitch rate in the required burst; missing attitude
cannot become level flight. Its AP handoff is deferred in pause, on failed mode
reads, refused mode commands or an unconfirmed VS write. Normal handoff requires
live heading/VS mode readbacks. Startup variation also comes from a real read.

Full auto previously fetched SimBrief directly and rejected an unset ID even
with a local plan on the speech loop's shelf. The shelf is now published to the
tool; SimBrief remains an explicit fetch when no local plan is present. Local
flights get a cruise fix, a sixteen-mile final-alignment fix and an eight-mile
arrival fix. The route reserves climb/descent distance using the airframe's
speeds and climb rate, picks a sufficiently long runway by wind or arrival
bearing, and probes the climb and arrival legs every 150 m. Missing/obstructed
terrain refuses the arm. The longer track increases the fuel requirement while
preserving reserve; the expanded plan goes back to the same shelf and FMS.
A changed arrival runway requires replanning rather than silently abandoning the
checked corridor. This is still a conservative direct route: no airway or
controlled-airspace routing has been added.

That route check exposed a plugin assumption: a failed terrain probe was
published as 0 m, on the premise that circuit-sized queries were always inside
loaded scenery. That is false for a local flight. The plugin now publishes
`xcopilot/terrain/valid`; any failed/non-finite probe invalidates the entire
batch. The app serialises query/generation/result as one mailbox transaction
and refuses missing validity or partial results. Local full-auto therefore
requires this plugin build and terrain within the loaded scenery region.
Other terrain consumers retain their existing unavailable-data behavior.

Regression coverage: cancellation/restart exclusion, failure/panic cleanup,
late child creation after parent cancellation, real HTTP ID rollover after a
plugin reload, cancellation during ID lookup, pause/resume, refused AP handoff,
missing/invalid instruments, failed/refused dial writes, local route geometry,
climb/runway refusal, missing/obstructed terrain, and local shelf replacement.
Validation includes the full workspace tests, install-name audit and universal
macOS plugin build. No new live sortie has been flown for these changes.

## B58 flare: stop removing power into renewed sink (2026-09-16)

Two LEBL autolands reproduced a late sink after an initially successful
roundout. On 06L, flare entry at 23.3 ft / 98.7 KIAS / -403 fpm became
-65 fpm at 18.5 ft, then -446 fpm at 2.5 ft. On runway 20, entry at
23.0 ft / 97.3 KIAS / -394 fpm became -24 fpm at 20.0 ft, then -515 fpm
at 0.9 ft / 85.8 KIAS. These are sampled VSI values, not exact impact
measurements. The second landing had `park=0`, so the first landing's
`park=1` does not explain the repeated airborne sink. Throttle/prop RPM
were not recorded, so prop drag is a plausible contributor, not proved.

The implementation commanded throttle 0 on EVERY Flare tick. It bypassed
the approach governor regardless of measured sink, while the VS lead first
arrested and then chased the renewed descent. The old plant regression is
fitted to a 172, not this Baron. FAA Airplane Flying Handbook, Chapter 13,
Normal Approach and Landing, describes gradually reducing residual power
in roundout and the limited float of twins with windmilling propellers.
It also discusses residual-power touchdowns on some twins with ample runway;
that is general guidance, not a numerical Baron power prescription:
https://www.faasafety.gov/files/events/SO/SO15/2022/SO15118118/TransitionToMultiengineAirplanes.pdf
Laminar's B58 manual p.52 says props fully forward, power as required:
https://x-plane.com/manuals/Baron_Pilot_Operating_Manual.pdf

`flare_power` is now an optional aircraft-profile setting carried with the
mission's ApConfig (including scoped autoland and runway re-arms). Only the
B58 opts in: preserve measured entry power on the first flare tick, then
reduce at 0.04 throttle ratio/s; pause reduction above 200 fpm sink or when
the one-second sink projection exceeds it. Those are EXPERIMENTAL sim tuning
values, not POH numbers. The controller never raises power, follows a lower
observed lever, cannot restart its ramp from a stale higher readback, and
resets outside flare/on a miss. Rollout idle and go-around full power retain
their existing priority. Missing throttle/VSI skips the progressive power
write rather than inventing a measurement. Other profiles retain idle flare.

Autoland now records `event: power`: throttle before/command/after, write
success, progressive policy, dt, VSI, radio height and IAS. The after value
is a readback, not just a successful PATCH. The ramp is tested against the
recorded runway-20 sink sequence, worsening-sink prediction, re-approach
reset, balloon reduction, idle convergence, stale readback and invalid
measurements. This is a controller regression, NOT proof of a softer landing.
A repeatable live B58 approach is still owed before calling the tuning proven;
watch float/runway remaining as well as touchdown sink. No direct-pitch
flare, prop-RPM change or parking-brake change is included in this patch.

Validation: `cargo test --workspace -- --test-threads=1` passed (796 tests,
25 ignored); `cargo build --release --bin xvl` succeeded. The parallel
workspace run hit an existing mock HTTP server WouldBlock failure in
`cancellation_during_resolution_prevents_the_later_patch`; the sim tests
and then the entire workspace passed serially with local socket access.

## 2026-09-16 — Explicit control ownership and flight interaction

After the B58 crash/reload, “resume flight plan” briefed the plan and the pilot
had to request autoland separately. Resume now has an exact request grammar
(`resume the flight plan`, `resume my flight plan`, `resume the route`,
`continue the flight plan`) which bypasses the briefing branch. It uses the
in-memory loaded SimBrief plan, never silently fetches a replacement, and arms
an airborne full-auto mission without repeating startup, taxi or takeoff.
The existing follower, arrival and taxi-in orchestration remain the flight laws.

The rejoin selects the forward endpoint of an existing segment within 3 NM;
ambiguous crossings, invalid positions and being past the route are refused.
Arming needs live, stable flight above 1,000 ft AGL, profile speed clearance,
running engines and a matching plan aircraft. The join is bounded to 30 NM
and terrain-probed every 150 m against the lower of present and target true
altitude, with 1,000 ft clearance. Unavailable terrain refuses. Arrival data
is verified, and movement during verification causes a retry rather than
arming a stale join. Recovery and existing flying managers exclude resume.
Local plans currently refuse airborne resume: their checked runway metadata
is not retained on the shelf. A process restart also requires loading/reviewing
the plan again. This is deliberately not automatic resumption on reconnect.

The monitor stops automation after a sustained connection failure, speaks the
ownership change and the reconnect, and detects an airborne manager exit.
It does not claim the simulator's autopilot servos are disconnected: the pilot
is explicitly asked to check them. The panel shows monitoring/ownership even
when no manager is active. Route altitude readbacks say the route continues;
a follower-managed climb/descent no longer promises an exact requested VS.

Cabin warnings use known profile pressurisation capability (B58 unpressurised;
unknown profiles use neutral wording). Cabin alerts repeat on worsening
1,000-ft bands or severity escalation, rather than at a fixed warning interval.
Critical warnings retain their existing reminder policy. Airspeed and cabin
recovery are announced once after five seconds clear, only if the associated
alert actually spoke.

The new pure interaction helpers collect a landing summary after a slow,
settled rollout: last airborne VSI sample, last available autoland cross-track
error, possible bounces, and whether automation was interrupted. They explicitly
say the exact touchdown rate is unavailable. Sampling can miss a short bounce;
this is not an impact sensor or landing-quality certification.

Validation: workspace regression suite passed (803 tests, 25 ignored), including
explicit-vs-echo resume grammar, conservative route rejoin, stable one-shot
recovery, cabin escalation and delayed sampled landing summaries. Live flight
verification of resume, reconnect and audio timing remains outstanding.
Final verification: the 621 binary tests passed after the status/readback edits;
all 33 monitor tests passed after grouping overspeed with airspeed recovery.
`cargo build --release --bin xvl` succeeded. The live process was not restarted.

## 2026-09-16: resume refused five times over the Gulf of Lion; the plugin was two days stale

LEBL-LFLL in the Baron under full auto, the app restarted at 18:05 on the
DIBER-SOSUR leg (25 nm to run, 7,000 ft). "Resume flight plan" was refused
five times in three minutes, each by a different gate, and the debrief could
not say why because the tool blackboxed nothing:

1. "No plan is loaded": the shelf is in memory and a restart empties it (by
   design; "load flight plan" refilled it from SimBrief).
2. "Terrain verification is unavailable", on the route, wings level. This
   was the real defect. `terrain::sample_ft` has read `xcopilot/terrain/valid`
   since 09-15; the loaded plugin was the 09-13 build (Log.txt: `db3a8a0`),
   which never registered it, so the read failed and every answer was
   discarded. The same failure had been retiring the follower's terrain floor
   ten seconds after every takeoff since 09-15 13:12 (`terrain_probe_unavailable`
   on four flights), in silence. Probed live: the plugin answered the query
   (gen 3 to 4, sea level back) and the `valid` name did not exist.
   `plugin/build/mac_x64/xcopilot.xpl` had been built at 22:38 on the 15th
   and never installed. Rebuilt as `e2f82ed-dirty`, staged into the sim
   (rm then cp, the loaded mapping kept); the reload waits for the aircraft
   to be parked.
3. and 4. "Resume needs live, stable flight ... with adequate airspeed", in a
   28 degree orbit both times (the pilot was circling while the plan was
   argued about). The bank gate was right; the line named nothing.
5. "Outside the route's rejoin corridor": after the orbit the aircraft was
   4.8 nm right of DIBER-SOSUR against a 3 nm corridor. Right, and useless.

Rules from it:

- **The plugin's side of the wire contract is probed by name** (`health::PLUGIN_DATAREFS`,
  pinned to every `"xcopilot/..."` literal in the source by a plain test),
  on the same index fetch as the profile probe. A stale build is said once
  at boot ("Plugin check: the loaded plugin is out of date. It has no
  terrain valid ...") and the Health screen's plugin line goes red with the
  first missing name. `plugin_probe` blackboxed per airframe.
- **The follower says when it loses the terrain floor**
  (`terrain_probe_unavailable` is now spoken), and resume's terrain refusal
  points at the plugin build.
- **Resume's gates name themselves**: paused, on the ground, the AGL, the
  bank in degrees, the speed against the floor. Every refusal and every arm
  is blackboxed on the `full_auto` kind (`resume_refused` / `resume`).
- **The corridor refusal says where the route is**: the abeam leg by its
  fixes, the side ("5 miles right of course on the DIBER to SOSUR leg"), and
  the magnetic heading for the leg's end fix (`interaction::rejoin_index`,
  `REJOIN_CORRIDOR_NM` 3 nm, unchanged). The pilot closes it and asks again.
- Whisper's "Resume site plan." is the resume request (`resume_plan_request`).

## 2026-09-16: the stock airliners get profiles (B738, A333)

Decided: stock airliners before the add-ons (Zibo, ToLiss), because the risk
is the product shape, not the datarefs. Every flying law here is GA-shaped
(rotation through the pitch channel, the flare through the VS dial, speed
through a throttle governor); an airliner is flown through the MCP/FCU and
the box, and "full auto" there becomes program-the-box, set-the-modes,
read-the-flows. The stock 737 and A330 expose those gaps on datarefs that
can be trusted, and an add-on is then a delta on the profile.

Without a profile both would have flown under `generic_from_acf` with the
GA defaults for the runway check (`takeoff_roll_ft`) and the pattern, which
is the RV-10 fault at 174,000 lb. `profiles/boeing_737_800.yaml` and
`profiles/airbus_a330_300.yaml` carry the airframes' own numbers: V-speeds
on one mid-weight card, the POM's field lengths recited and a mid-weight
ground roll checked, the acf's stall, Vfe, Vle, Vmo and weights, every
flap gate with its placard limit (the acf agrees at both ends of each
ladder), pressurised, cabin crew, two engines, tail-strike attitude, a
burn for the endurance check. `known_profile` maps B738/B737/B739 and
A333/A332/A330.

What each profile deliberately does NOT declare: a start sequence (the 737's
xlua replaces the stock starters, the A330's `A333.engine` is its own; the
MD-82 and CitX sequences are written against their own datarefs), so full
auto says "no start sequence" instead of cranking blind; and the
`autothrottle` tool (md82 datarefs; declaring it withholds `hold_speed`).
The A330's xlua replaces the battery toggles, generator on/off, the APU and
bleed commands and autobrake, so its electrical block is stock commands
reaching the aircraft's own handlers: hypotheses for the swap-time probe
and a live readback. Steering is the open question on both: the steer
channel is rudder input and the 737 steers by tiller at the gate.

## 2026-09-16: the first helicopter (S-76C) is a profile that withholds

Nothing in the code knew a rotor existed (zero hits for helicopter or
rotorcraft), and the S-76C's acf carries no stall speed, so under the generic
path it would have had the 172's numbers and every fixed-wing law on offer:
autoland would have flared it through the VS dial, auto takeoff rotated it
through the pitch channel, hold speed governed its twist grips. The new
`rotorcraft: true` flag makes the registry withhold that whole set
(`tools::ROTORCRAFT_WITHHELD`: autoland, auto takeoff, circuit, the follower,
full auto, recover, the talk-down, ILS prep, hold speed, auto taxi, line up,
taxi to parking, pushback, reverse thrust) whatever the other flags say, and
a test builds the registry from `profiles/sikorsky_s76c.yaml` to prove it.
What stays is the crew: switches, lights, gear (the S-76's xlua mirrors the
stock gear handle), radios, transponder, altimeter, pitot heat, brakes,
reminders, ATC, and the autopilot dials as a hypothesis (its script reads
only the stock airspeed dial). No V-speeds are declared, so none are spoken.
`known_profile` maps S76/S76C/S76D. Not done, and honestly not started: a
rotorcraft flight law, helicopter phraseology in the tower (air taxi, a
helipad instead of a runway), the search light's real switch.

## 2026-09-16: main.rs split along its seams; flight_state, one page for the crew's statics

Two refactors on a fresh branch, no law touched. `main.rs` had grown to
9,764 lines: boot, the command loop, the aircraft detector, the plan cache,
the ICAO map, the undo stack, the routing vocabularies and forty state
answerers in one file. Six modules now hold whole items moved by a script
that cut each top-level item with its doc comment and made it
crate-visible; `main.rs` imports them by name, so `run_session` and its
1,400 lines of tests read exactly as before (test-only names are
`cfg(test)` imports). `pollers` (panel buttons, typed line, voice picker,
settings switches), `detect` (the ICAO map, the live profile, the detector
thread), `router` (command / question / follow-up / echo / bare value),
`undo`, `plan` (shelf, fetch policy, FMS load, brief, TOD, ETA, named fix)
and `answers` (everything read from the datarefs the tools write).
`main.rs` is 5,370 lines: boot and the loop.

The statics: 76 process-wide cells across the loop files, and the history's
recurring bug class is one that outlives its airframe or its process. The
party line already showed the cure for the world's side (one page, the
writes on it). `flight_state.rs` is the crew's: `publish_airframe` (the six
selectors, previously duplicated in `LiveProfile::new` and `swap`) and
`stand_down` (the hand list from the detector, now also called by the
instructor-mode entry and the monitor's connection-loss path through the
same door). A test scans the tree for every `static ACTIVE` and asserts the
module is named in `stand_down`; it found one the hand list had missed:
`flow`, so a running checklist now stops on a swap, on entering instructor
mode and on connection loss, the one deliberate behaviour change. The
module doc inventories the per-FLIGHT facts still reset on their own paths
(`LAST_ARRIVED`, `LANDED_RUNWAY`, `INBOUND_MADE`, the follower's `TOD`,
`TERRAIN_FLOOR`, `PILOT_ALT`, `LEG`, and the rest); a `flight_begins` door
for those is the next step, not this one.

## 2026-09-17: the 737's first sortie (LFLL 35L): two go-arounds, three laws sized for a 172

The stock 737-800 flew "fly the flight plan" from the stand at Lyon: clearance,
squawk, pushback (77 of 89 m, judged short, the taxi joined 54 m out anyway),
auto taxi to 35L, takeoff on clearance, a left circuit, and two balked
landings. `xvl debrief` and the ticks named three laws whose numbers were the
172's:

- **The approach speed floor was the CLEAN one.** `run_autoland` floored the
  145 kt Vref against `speed_floor_kts` (1.2 x clean Vs 139 = 167). The 737
  flew the whole final at 165-169 kt, the flap schedule could never pass the
  flaps-40 gate (Vfe 162) and landed on flaps 30, and the first approach
  overflew the numbers 51 ft high. New `aircraft::approach_floor_kts`: 1.2 x
  Vso when the profile declares one (137 here), the clean floor otherwise,
  published per swap beside the clean floor; the autoland uses it, the
  circuit and the plan keep the clean one.
- **The glidepath correction had a fixed TIME constant.** 1.2 fpm per foot of
  error is 50 s: 0.9 nm on a 172, 2.4 nm on a 737 whose last mile is 22 s.
  Armed 118 ft below the path at 1.6 nm, the autoland levelled, started down
  late through the VS mode's lag, and crossed the threshold 51 ft high with
  86 fpm of correction in hand. `glidepath_gain_fpm_per_ft` scales the gain
  by groundspeed over 65 kt (3.1 at 170), floored at the old law, capped 3x.
- **The missed approach released the reposition ABEAM the join point.** The
  latch handed back on distance alone; a 737's turn-back left it 2.7 nm
  beside the centreline at 6.7 nm, `interceptable` accepted a 19 degree cut,
  and the pursuit's 30 s time constant had 110 s of final for 3.5 km of
  offset: balked at 95 ft still 84 m off. `GoAroundLatch::update` takes the
  cross-track and releases only inside `REPOSITION_RELEASE_XTK_NM` (0.5 nm).

Seen and not yet fixed: the takeoff's pitch channel has 0.10 of nose-down
authority (`PITCH_MIN`) and the 737 ran from the 8 degree target to 17.8
degrees after liftoff with the channel pinned, accelerating the whole time;
the AP took over at 300 ft with a 150 fpm climb command (`climb_vs_fpm`'s
60 fpm per knot under Vy is a GA slope: 30 kt under 210 costs 1,800 fpm);
the base turn bottomed at 154 kt clean-ish (flaps 5) at 800 ft, the
airspeed caution firing, before the 167 floor caught it; the fuel caution
re-said itself seven times in five minutes as the burn-based estimate swung
25 to 60 minutes; and the 737's checklist file has no climb or before
landing section, so the tracker proposes nothing there. A withheld tool on a
helicopter now says "not on a helicopter" instead of "isn't wired up yet"
(`router::unregistered_tool_line`).

**Are the datarefs right?** (same day, the aircraft parked): every name exists
(`real_install_names`, the profile probe: 30 of 30) and the sortie wrote them
all without a failed write; the semantics were checked against the 737's
xlua, which assigns crew-used stock datarefs only in its command handlers and
presets, except one. `B738_baro_memory` copies `laminar/B738/barometer/
capt_setting` into the stock altimeter dial in before_physics EVERY frame and
that dataref is not writable over the API: a dial write is a silent no-op,
and the tool's immediate readback said "Roger, altimeter 30.11" to an ask of
30.00 (the exact failure the readback rule forbids). The setter is now
`instruments::set_altimeter`: write, re-read a frame later, and when the
write did not stand, step the knob with `barometer_up/down` (the 737 replaces
those, a hundredth a press; eleven presses proven live) and read again; the
spoken tool, full auto's departure set and the monitor's fix all go through
it, and the tool says "didn't take" when the dial disagrees. The stock
landing-light command proved to set all four `landing_lights_switch[]`
entries the lamps read; the taxi lamp is unproven (the cockpit switch drives
`generic_lights_switch[4]`, the stock switch is what the readback reads).

## 2026-09-17 (later): the 737's second sortie landed; the approach and landing model gets its jet laws

With the morning's three fixes the 737 flew stand-to-stand: pushback,
taxi, takeoff 35R, circuit, autoland, vacate at V4, taxi to stand, secured.
The landing was poor and the ticks say why, each a 172 law at 737 speeds:

- **The circuit handed over fast, low and at flaps 5.** Base and final were
  flown at Vref+20 (165) with the takeoff flap, 170 ft under the path at the
  handoff. The autoland then idled the engines for the whole last mile to
  lose 24 kt, put flaps 30 and 40 out at 300 ft, went through Vref at 170 ft
  still at idle, and spooled from 0.1 to 0.39 as the speed fell to 141.
  Now: the circuit extends the landing flaps on base (half travel) and
  final (full) by the placard gates as the speed allows, the same
  `flap_within_limits` law autoland runs, and final slows to the approach
  speed floored by the flaps ACTUALLY out (`aircraft::config_floor_kts`,
  interpolating the clean and landing floors by flap ratio: flaps 5 floors
  at 156, flaps 40 at 137). Autoland floors its own target the same way.
- **The lever idled on a stabilised final and cut to zero at 22 ft.** A
  swept wing at idle sinks at once: the arrest ballooned to +39 fpm at 20 ft
  while the speed bled 142 to 131, then dropped in at -449 fpm. New profile
  field `approach_thrust_min` (0.3 on the 737, VERIFY) is the least the
  speed law pulls the lever to on final when not more than 10 kt fast
  (`autoland::approach_power_floor`), and the 737 gets a `flare_power`
  profile so the flare retards progressively (hold while sinking over 400).
- **The give-up after three go-arounds released the aircraft at 95 ft**
  with landing flaps and 75% thrust, "holding altitude": it accelerated at
  300 ft AGL to 236 kt through the flap limit, and "land the aircraft" from
  there re-armed onto 17R at 140 ft and 237 kt; the reposition's speed law
  pulled the throttle to idle for its 179 kt target and the aircraft
  settled into the fields ten miles out with +800 fpm on the dial, nine
  ground contacts at 130-180 kt. Now the give-up waits for the latch's
  Climb stage to reach 800 ft AGL (full power, go-around flap), hands back
  there with the flaps retracted and says the height; and in a reposition
  below the terrain floor the lever floor is FULL power: excess speed is
  spent on the climb, never on the lever.

Still open on the 737: the arrest law's +300 fpm dial cap with this AP's
lag (the balloon at 20 ft); the takeoff pitch channel's 0.10 nose-down
authority; the climb command of 150 fpm at the AP engage; whether the
737's MCP accepted the stock VS-mode command at the 09:05 re-arm (the dial
said +800 and the aircraft descended: needs a live probe from ALT HOLD);
the self-pick of 17R (nearest threshold, the reciprocal of the runways in
use); the checklist file's missing climb / before landing sections.

**The 737's engine start and checklist** (same day). The pilot asked for
XChecklist files. Two exist: ch4dwick's short Laminar-737 list (all 27
names exist on the stock aircraft; its start items name
`laminar/B738/spring_knob/starter_1/2` and the stock fuel pump, APU, bleed
and generator datarefs) and vin_kaizen's 1,762-line Zibo list, whose
`sw_item_c` items lean on Zibo-only names (181 of 391 exist here). The real
find was in the aircraft's own xlua: `B738.Z.autostart` REPLACES
`sim/operation/auto_start` with a scripted auto-board (battery, standby
power, cond levers, fire tests, APU, packs) chained into an auto-start
(fuel pumps, APU gens, packs off, starter 2 then 1 on N2, condition levers
to idle, generators, probe heat, engine bleeds, APU off, continuous
ignition). Fired once on the parked cold aircraft: both engines running and
the APU shut down at 124 s. So the profile now declares
`scripted_auto_start: true`, `StartKind::Scripted` fires the command and
waits on the engines (300 s deadline), and "start the engines" is the
`jet_start::ScriptedAutoStart` tool on such airframes; the MD-82 sequence
stays off it. `checklists/boeing_737_800.txt` is a normal checklist written
from the POM, the xlua's own step list and ch4dwick's items, every dataref
verified to exist over the API, section titles equal to the phase keys so
the tracker's climb / descent / before landing / after landing lists and
the syllabus's ramp lists all resolve (test-pinned in `flow.rs`).

## 2026-09-17 (third pass): LFLL to EGLL, and the six things the blackbox named

The 737's first real sortie, stand to stand across France, plus the
morning's three circuits at Lyon read back out of `logs/blackbox.jsonl`.
Six defects, none of them in a control law:

- **The region tick stole the station from the destination tower,
  mid-approach.** At 12:52 the inbound call reached Heathrow (118.505) and
  was answered "join final runway two seven right, report final". At
  13:06:42, fifteen miles out, the enroute region's service-weighted
  nearest-field pick found London City eight miles off the nose and took
  the station. Everything after that went nowhere: the final report at
  13:08:24 `void`, so the 737 landed on 27L with no landing clearance; the
  taxi-in request at 13:28:31 `void` on London City's 118.075, sixty
  seconds of silence, then a false "No taxi-in clearance came back, your
  aircraft". The region only announced "We're at London Heathrow" at
  13:28:32, twenty-three seconds AFTER touchdown. A committed arrival now
  PINS the station to its field (`station::pin_arrival_station`, set at the
  inbound call and at the arrival's own retune): while it is set the region
  may hand the station TO that field and never away from it. The pin is
  released at the stand, on every `stand_down`, at the next mission's arm,
  and by `Station::uninstall` — and on the GROUND a relocation to the field
  under the wheels outranks it and drops it, so a forgotten pin can never
  strand the station at a field we have left.
- **The vacate turned a 737 around on Heathrow's active runway.** The first
  attempt was `vacate_no_plan` (the loaded taxi network was London City's —
  the same root cause). The second, planned once the station was right,
  took the nearest junction ahead, 84 m away: its only link is N4W, a rapid
  exit built for 09R traffic that runs back east-north-east, 169 degrees
  off our rollout. 624 m of route, a U-turn on the strip, back east past
  our own touchdown point, `vacate_timeout` with `arrived: false`, and four
  `runway_incursion` events as traffic closed from 2.6 to 0.5 nm. A
  junction being AHEAD was never enough: it must also offer a taxiway you
  can turn onto (`VACATE_TURN_MAX_DEG`, the same 100 degrees `ahead` uses),
  and the first step of the walk is now that turn-off rather than whatever
  lies furthest from the aircraft. Heathrow's own data picks N5W, 220 m
  further on and 54 degrees off. A field whose every turn-off is backwards
  yields no plan and the taxi-in starts from the runway, which is the
  degradation the dead end already made and better than the U-turn.
- **The takeoff flaps never came up.** LFLL to FL360 to Heathrow's top of
  descent — eighty-five minutes at flaps 5, the whole climb sitting on
  their 250 kt placard. The retraction is a one-shot inside the takeoff
  loop gated on `ias > vy`, and the 737 hands off at 1,000 ft doing 181 kt
  against a Vy of 210, so it never fired and nothing downstream owns it.
  The handoff now cleans up (`clean_up_at_handoff`) or says it cannot: a
  takeoff that ends with the aircraft still configured for takeoff has not
  finished. Below the airframe's clean-configuration floor the flaps STAY
  and the copilot says why — the retraction must never be the stall the
  schedule exists to avoid.
- **The initial climb was commanded at 150 fpm.** `ap_engaged` at 306 ft
  logged `cmd_vs: 150`, held to the handoff; the jet levelled at 1,030 ft
  at takeoff thrust and accelerated to 220 before the follower picked it
  up. The climb law protects the speed it is handed and it was handed the
  ENROUTE climb speed, so 174 kt (V2+20, exactly what the aircraft has at
  400 ft) read as thirty-six knots slow and 2,150 fpm came off the rate.
  `ReferenceProfile::initial_climb_kts` splits the two numbers; it defaults
  to `vy_kts`, so every trainer is unchanged, and the circuit keeps `vy`
  for its pattern. 737: 170. A330: 173.
- **The 737 had no way to be shut down.** At the stand the pilot asked
  seven ways in forty-five seconds ("let's shut down", "engines off",
  "turn engines off", "shut down aircraft", "turn off engines") and gave up
  with "my aircraft". The `scripted_auto_start` branch of the registry
  added the START tool and not the stop, so every phrasing parsed to a tool
  that had never been built. It registers `StopEngines` now — an aircraft
  may script its own start, but the stop is always the crew's. Two of those
  phrasings did not parse either: "turn off engines" (the other word order)
  and "cut the engines", which is in the tool's own description of itself.
- **"Short" was the wrong word for both failed pushes.** One asked for
  88.7 m, got 77.4 with the nose 1.1 degrees off the lane — a push anyone
  at the window watched finish — and failed by 1.3 m against a flat
  ten-metre threshold sized for a GA stand. The other went twelve metres
  PAST its plan and left the nose 71 degrees off, and was blackboxed
  "short" too. `push_delivered` now also accepts a completed swing that
  covered `DELIVERED_FRACTION` of the distance (the residual is the taxi
  drive's, which is what its own doc always said), and `push_residual`
  names what is actually left: "short" or "unturned".

And one found while reading the wake gate rather than the log. The gate is
capped at one full separation interval AT THE HOLD so that a field whose
departure stream refreshes the clock faster than it expires still lets you
go (2026-07-27) — but the cap is measured from `hold.armed_at`, and every
new takeoff request re-stamped it. Ask a second time at Heathrow and you
hand your own clock back to zero, hear "wake turbulence, departing traffic
ahead of you" again, and never take off. A hold already standing on the
same runway now keeps the moment it was armed (`hold_armed_at`): the clock
belongs to the departure ahead of us in sequence, not to our last
transmission.

## 2026-09-17 (fourth pass): EGLL to EGPH ended in a field near Perth, killed by a 30-minute timer

The 737 flew the SimBrief plan to Edinburgh, followed it fix by fix down to
RIBEL, and the arrival armed the field's published ILS 24. Thirty-one
minutes later it hit the ground eight miles north of the airfield with the
copilot on its twelfth airspeed warning. The blackbox says the whole story
in two lines one second apart:

```
17:19:08  full_auto {"stage":"arrival_owns"}
17:19:08  procedure {"event":"stand_down","reason":"abort"}
```

and then nothing at all for eleven minutes, to the `crash` record at
17:30:04.

**What happened.** The approach armed at 16:49:07 and the follower steered
at CI24, the ILS 24 initial fix, 121 nm away: SimBrief's route ended at
RIBEL, in the Lake District, and the fix it had to fly to next was on the
far side of Edinburgh. It flew it properly. Every `leg` record from 16:50
to 17:19 shows the distance coming down at 4 nm a minute on a steady 338
bug, and the vertical law paced its own descent from 26,000 to the leg's
3,000 ft constraint over the same half hour. At 4.19 nm to run, two
minutes from the fix, `full_auto`'s wait on "the arrival to hand to
autoland" hit its flat `Duration::from_secs(1800)` - counted from the arm -
and the mission returned. Its lease dropped, the scope cancelled the
follower underneath it, and `stand_down` released the speed governor with
the throttles where a descent had left them.

Nobody flew the aircraft after that. The autopilot held the last dialled
altitude while the speed decayed 200, 179, 161, 142, 131 kt; the airspeed
warning fired every 30 seconds from 17:23:52; the 737 mushed down through
2,000 ft of Perthshire and stopped in a field at 719 ft. The pilot got one
Caution line ("Cancelled. Your aircraft.") from the dying follower and no
word at all from the mission, which blackboxed `arrival_owns` - the event
that means the arrival loops have it - as it walked away.

**Three things, and the first is the bug.**

- **A clock cannot tell a long arrival from a stuck one.** The 1,800 s was
  sized for an approach armed at the top of descent, not one armed 121 nm
  from its first fix, and it expired on an arrival that was doing exactly
  what it had been told. The wait is now a watchdog on CLOSURE
  (`ArrivalWatch`): the fix the follower is steering at and the miles to
  it, fed once a second, stalling only after `ARRIVAL_STALL_SECS` with no
  new low. A sequenced leg jumps the distance up and is judged afresh; a
  follower on a to-altitude leg holds the clock rather than starting one.
  The absolute cap survives at three hours, which is a bound on a wedged
  thread, not an opinion about an approach.
- **The distance comes from the follower, not from the sim.**
  `procedure::fix_distance_nm()` republishes the number the tick already
  worked out, beside `active_leg` and `planned_tod`. An orchestrator that
  flies its own reads to judge a loop it can already hear from is Web API
  load for nothing, and it would be measuring a different distance from
  the one being closed.
- **A give-up hands back a FLYABLE aircraft, and says so.** This one
  handed back an airliner at 4,900 ft with descent power and a live
  altitude hold, silently. The expiry path now stands the follower down
  first (its stand-down is what releases the governor, so the throttle
  write has to follow it), sets `GIVE_UP_THROTTLE` and zeroes the VS dial
  through the mission's own still-live client, and speaks the Caution:
  "The arrival isn't getting us there. Your aircraft. The autopilot is
  holding heading and altitude, climb power set." Autoland's balk path has
  had this rule since EGHI, 2026-08-02; the orchestrator did not.

A note on the scope, because it is the reason nothing could be done from
the follower's side: a cancelled `XPlaneClient` REFUSES writes by design -
a dying worker must not actuate. So the hand-back has to happen in
whatever is still alive, which is the mission, before it returns. That is
also why the fix belongs in `full_auto` and not in `procedure::stand_down`.

And one more, found by asking why the arrival was covering the last hundred
miles at all: `parse_waypoints` capped the navlog at the first FOURTEEN
fixes. See below - it is the reason the aircraft was ever in that position.

## 2026-09-17 (fifth pass): the fourteen-fix cap amputated the arrival

The EGPH plan ended at RIBEL because the parser stopped there. SimBrief's
navlog for that OFP is twenty-one entries:

```
D257C D280G BUR D355D UMLAT WOBUN WELIN AKUPA TIMPO ELVOS TOC
TNT POL NELSA RIBEL TOD INPIP INREV ESKDO TARTN EGPH
```

Nineteen real fixes once TOC and TOD are dropped, and `parse_waypoints`
took the first fourteen and broke out of the loop. Fourteen lands exactly on
RIBEL. Everything after it - INPIP, INREV, ESKDO, TARTN, the whole TARTN
arrival into Edinburgh - and the destination airport itself were never in
the plan the crew flew. That is why the briefing said "the 14 enroute
fixes", why the follower's last leg ended 110 nm short of the field, why
the arrival armed the ILS 24 from 121 nm out, and ultimately why a
thirty-minute timer had anything to expire on.

The cap was there for a real constraint: `xcopilot/fms/route` is a
`TextRef`, 512 bytes, and `StoreText` truncates a longer write at 511 -
mid-entry, silently. But that is the PAYLOAD's constraint, not the plan's,
and a cap applied to the front of a list is an amputation of its tail. The
plan the crew flies is now the whole navlog; `simbrief::fms_route_payload`
owns the mailbox and thins the GPS route to fit by dropping fixes from the
MIDDLE, so the origin and the destination - the two the CDU's ORIGIN/DEST
and the whole-plan load actually need - always survive. Both payload sites
(the spoken "load the flight plan" and full auto's own write) go through it.

Thinning the GPS costs the crew nothing: the follower flies the PLAN's
fixes, and `xcopilot/fms/goto` moves the pointer for the idents the loaded
plan does hold - an ident it lacks changes nothing on the plugin side.

The same OFP through the new code: twenty waypoints, EGLL to EGPH, with the
follower's last enroute leg ending at TARTN - 20.6 nm from CI24 at the
plan's own 6,400 ft, which is where an ILS is armed rather than a hundred
miles out at cruise. The GPS route loses exactly two mid-route fixes
(TIMPO, ELVOS) and fits in 486 bytes.

## 2026-09-19: the 737 profile reviewed; the monitor's checklist goes quiet; narration perishes

A read of `profiles/boeing_737_800.yaml` against the install (every name in
DataRefs.txt, the acf's limits: Vs 139, Vso 114, Vfe 250/162, Vle 270, Vne
340, Mmo 0.82, MTOW 174,200) and against the LFLL-EGLL blackbox. The names
and the placard ladder are right. What was not:

- **The cruise speed target was above Mmo.** `cruise_ias_kts: 280` (the
  acf's Vno) is M0.78 at FL290 but M0.84 at FL360, and nothing in the crew
  reads Mach. On the 17th the aircraft could not reach it at FL360 and sat
  at 240 KIAS on the stop for the whole cruise. Now 255 (M0.78 at FL360).
  A Mach-aware governor, or an `mmo` field, is the proper fix.
- **Three checklist items were guaranteed discrepancies.** The climb list
  runs at climb established (2,100 ft on the 17th) and tested "landing
  lights off above 10,000" and "altimeters standard at transition"; the
  descent list runs at the top of descent and tested "landing lights on
  below 10,000". Each said "Not set" on every flight. They are call-outs
  now: the lights law and the altimeter check own those switches at the
  right height. "Taxi light on" left the before-landing list (the lights
  law lights it at the vacate; it said "Not set" on final at Heathrow).
- Duplicated fact, code side: the local planner derives pressurised from
  the `kind` string (`plan.rs`), the profile declares `pressurised: true`.
  They agree on the 737. Left as is, noted.

**The monitor flavour recited every item.** `flow::stateful` returned Say
for a satisfied item in every flavour, so under full auto the climb list
was ten lines at 2,000 ft and the before-landing list nine lines on short
final at Heathrow, queued behind the radio, each confirming a switch the
mission had set and announced itself. The doc said "narration skipped"; the
code skipped only `Narrate` items. Now the Monitor flavour is silent item
by item (satisfied, held, call-out and unreadable alike; every discrepancy
still blackboxed) and speaks ONE line per list, `flow::summary_line`:
"Climb checklist complete." or "Climb checklist complete, 2 items not as
listed: flaps, up on schedule; altimeters, standard at transition." The
crew and instructor flavours are unchanged (they said each item as they
went, so their summary keeps the count). On the 17th's sortie that is 102
spoken lines down to about 70, with the two bursts gone.

**Speech lagged and never caught up** (the pilot's observation, same day).
The arbiter's fire-and-forget `alert` had no expiry: the mission queued
three advisories in one second at the LFLL start and they played out over
the next ten, and a burst behind a radio exchange arrived a minute after
the events. The perishable path existed (`alert_stale_by`, used by the taxi
guide's cues and the stepped-on remainders) and nothing else used it. Now
every ADVISORY queued through `alert` perishes at `ADVISORY_STALE_S` (12 s)
if it has not started; readbacks (blocking, the pilot asked), cautions and
warnings never expire. A dropped line is logged ("dropped stale intercom
line") and was mirrored to the panel transcript at enqueue, so nothing is
lost, only not said late.

The ordering question ("should the speech happen first?") was answered from
the code rather than changed: a spoken command executes, re-reads and THEN
reads back, and that order is the safety property (a failed write must be
said after the fact). Automations announce through the non-blocking queue,
so their words trail the switch by synthesis time plus whatever is queued;
the right shape is "announce as you act", the alert issued immediately
before the write, and a follow-up only when the readback disagrees. Still
owed: the mission sets flaps 5, strobes and the landing and taxi lights
without a word; the pilot-monitoring's bare callout ("Flaps five.",
"Strobes and landing lights on.") at the write site, coalesced per stage.

## 2026-09-19 (later): EGLL-EGCC, the 737 could not land: two overflight balks from three 172 numbers

Full auto flew the 737 from the stand at Heathrow to the RNAV 23L at
Manchester: pushback, taxi, 27R, the plan, the region handoffs, the inbound
call at the station swap, the approach armed from 7.4 nm. Then two
go-arounds, both "overflew the runway", and the pilot landed the third
approach by hand. `xvl debrief` and the ticks (blackbox flight 125):

- **First approach, balked at 57 ft radio, 322 m past the numbers,
  sinking 724 fpm, 2 m off the centreline.** The aircraft crossed the
  threshold at 100 ft (the path's 51 ft, plus 20 ft of header-versus-probe
  elevation just under the 20 ft trust band, plus a 32 ft standing error
  the P law carries against this autopilot, which flies 80 fpm shallower
  than its dial). At 64 ft, past the threshold, `alt_err` went negative
  (the glidepath distance is unsigned) and the level-hold branch dialled
  ZERO. One tick later, 57 ft is above the 50 ft roundout band, so the
  overflight allowance was the fixed 0.15 nm and 0.174 nm is past it. A
  737 sinking 724 fpm from 57 ft touches 340 m further on, 660 m in, with
  2,400 m ahead: that was a landing.
- **Second approach, balked at 11 ft radio, 1,537 m in, climbing.** Same
  100 ft crossing, same zero dial at 60 ft. The roundout opened at 47 ft
  with the aircraft sinking 712: `vs_dial_fpm` asked for -250 + 2 x 462 =
  +674, capped to +300, and the dial sat at +300 for 2.7 s. The 737's VS
  mode followed it: -712, -680, -623, -526, -382, -225, -90, +1, +19. It
  levelled at 30 ft and floated 500 m while the speed law read the
  roundout's own deceleration as five knots slow and walked the lever from
  0.32 to 0.55. The dial then swung to -788, the aircraft dived back to
  -513 at the 25 ft flare gate, the flare's -100 target pinned the dial at
  +300 again for 3.6 s, and with 0.55 of lever held (the flare power's
  hold-while-sinking rule) it climbed away from two feet at 155 kt to the
  float allowance's end. Fitted to those ticks the 737's VS response is
  two 1.5 s lags in series with about two seconds of dead time; the 172's
  law was fitted to a first-order 4.5 s lag, and a first-order plant
  cannot ring at any gain.

Four laws, all shared, the 172's numbers unchanged by construction:

1. **Over the pavement above the band the path's own sink continues**
   (`guidance`: Final, `along_nm < 0`, on the centreline, target =
   `nominal_vs`), never the level-hold and never a dive at the aim point.
2. **The arrest's gates are seconds of sink, its targets a fraction of it**
   (`arrest_gates_ft`: the flare gate is the greater of 25 ft and 3 s of
   the nominal path sink, the band the greater of twice that and 6.5 s;
   `arrest_targets_fpm`: roundout the lesser of -250 and 60% of the path,
   flare the lesser of -100 and 25%). A 172 at 75 kt over the ground keeps
   25 / 50 ft and -250 / -100 exactly; the 737 at 132 kt gets 35 / 76 ft
   and -420 / -175 (a jet asked to hold -100 from 35 ft floats a kilometre).
   The tests' 95 kt approaches now roundout at -300.
3. **The dial's lead is the airframe's** (`ArrestLeadProfile`: gain, cap,
   and a trend term that predicts the sink `trend_seconds` ahead from the
   tick-to-tick change, the damping a second-order autopilot needs;
   published per swap, `flight_state`). The default is the 172's (2, +300,
   0). The 737 declares 1.5, +100, 1.5 s: swept over three plant fits the
   arrest lands at 120 to 180 fpm without a balloon; with the 172's numbers
   the same plant balloons to +70 and floats past 1,000 m
   (`the_jets_arrest_neither_balloons_nor_floats`, a second-order plant with
   three ticks of dead time through the real guidance and dial law).
4. **The lever never advances in an arrest** (the roundout's `adj.min(0)`;
   the floor still stands) and **the overflight allowance is geometric and
   knows the airframe's landing distance** (`overflight_allowance_nm`: the
   float's room-to-land-and-stop, less the run a 3 degree descent needs to
   reach the band from here, floored at the old 0.15 nm; the pavement
   needed is the recited `landing_distance_ft` when there is one, never
   less than the 900 m GA figure; `publish_landing_distance`). EGCC 23L
   at 57 ft gives the 737 1,406 m of float, not 278; a 172 at 160 ft over
   EGHI still gets the hair-trigger.

Left alone, noted: the 100 ft crossing itself. The aim trusts the header
within 20 ft (the probe said 19.45), and the P law has no integral term
against an autopilot that under-flies its dial by 80 fpm; together they are
190 m of runway on a 737. The mission's give-up and the throttle's flare
hold-while-sinking rule (`hold_sink_fpm`) were not touched: with the lever
no longer advanced in the roundout the flare holds the approach setting,
which is what the rule was written for.

## 2026-09-20 (later): the A330 reviewed against its manuals and its scripts; EGPH's twenty-two-minute "approach"

The stock A330's own folder was read end to end: both PDFs (the POM's 84
pages and the MCDU manual's 44), the `.acf`, and every xlua script under
`plugins/xlua/scripts`, the last with every per-frame write to a stock
dataref mapped to the function that makes it. The same day's blackbox at
Edinburgh was read alongside. What the three sources said, and what changed:

**The scripts own more than the profile knew.** Beyond the replaced
commands the profile header already listed, `A333.switches.lua`,
`A333.lighting.lua`, `A333.transponder.lua` and the electrical ECMU rewrite
the stock seat-belt and no-smoking signs, `transponder_mode`, every
probe-heat switch, `navigation_lights_on`, `strobe_lights_on`,
`battery_on[]`, `generator_on[]`, `cross_tie`, `apu_bleed`,
`engine_bleed_sov[]`, the fuel pumps and the electric hydraulic pumps EVERY
FRAME from the aircraft's own `laminar/A333` positions. So the profile's
stock nav and strobe commands went nowhere (the readback read what the
aircraft was doing, which happened to agree), and the transponder mode tool
was the Twin Otter no-op. The profile now drives the laminar commands for
the nav, strobe and taxi lights (the nose light is a three-position switch
on `landing_lights_switch[1]`, OFF / TAXI 0.5 / T.O. 1.0, so
`lights::read_switch` reads the half as lit), declares the transponder MODE
knob (`auto_on_knob_pos`: STBY -1 / AUTO 0 / ON 1, AUTO being the SOP
position and the "altitude" mapping), the parking brake VALVE (its own
auto-board sets `park_brake_valve` 1 for SET), and the seat-belt advisor's
hand is `seatbelt_signs_up`. Stock writes that DO reach the aircraft, for
the record: throttle (no detent snapping in Lua; the detents are the acf's
FADEC TLA table, CLB 0.53 / MCT 0.76 / TOGA 1.00), the AP dials (heading
only in selected HDG), flaps, gear, speedbrake, parking brake ratio,
autobrake level, barometer, COM frequencies, transponder code.

**The start left the packs with no air.** The after-start list did packs
on, APU bleed off, APU master off, and never opened the engine bleeds or
touched a generator. The POM's own order (p.68) is generators on, APU
generator off, APU bleed off, engine bleeds on, packs on, APU master off,
and that is now the list, every position the aircraft's own pushbutton.
The APU generator comes on with the APU (POM step 7). Number two starts
first (`start_order`, p.66).

**The start now runs at a crew's pace and says milestones, not switches.**
`AirlinerStart::run_steps` fires a switch, WAITS for its position to read
back (3 s), then a beat and a half, and passes over a switch already set
without a pause; every step goes to the blackbox and the panel transcript,
and the voice says only the profile's `say:` milestones ("Fuel pumps on,
eight of eight."), only once the step confirmed. A switch that never
reaches its position is named in the closing line ("One switch didn't
take: left1 pump."), which is the readback-against-intent rule applied to a
flow. Flown in `cargo test` with the clock paused (`tokio` `test-util`,
now a dev-dependency) against a cockpit fake whose switches move when
their commands fire, and one whose command reaches nothing.

**The numbers.** The POM's burn is 15,800 lb/h for the pair, not the
12,000 the profile had (5,500 kg/h → 7,170); the SIM's tanks are the acf's
240,712 lb (the POM's 109,000 kg), not the real aircraft's 97,530 L, so
`fuel_capacity_gal` is 35,900 and the endurance 15 h. `approach_kts` 145
sat under the 1.2 × Vso floor (150) and was raised aloud on every arm; the
MCDU's own VAPP is 154 and that is the figure. The 737's landing laws
(`approach_thrust_min`, `flare_power`, `arrest_lead`) were absent, so the
A330's final would have been flown to idle with the throttle cut at the
flare and the 172's arrest gain; they are declared as the 737's numbers,
VERIFY on the first landing. A checklist file was written from the POM's
own lists (pp.55-84), on the aircraft's positions; until it existed the
tracker logged `no_section` at every phase.

**Two laws had a GA number in them that the A330 found.** The flight
governor's target cap was a flat 250 kt: the profile's 290 kt cruise was
armed and 250 held, with nothing said (the floor speaks, the cap did not).
It is now the airframe's Vne less 15 (`aircraft::speed_ceiling_kts`,
republished per swap; the 250 below 10,000 ft is still applied as
airspace). And the monitor's flap-stall was a fixed 17% off clean Vs: 175 ×
0.83 = 145, so a stabilised 150 kt final was inside the WARNING band the
whole way down. With a Vso on file the stall runs straight from clean to
Vso (`stall_speed_at_flap`).

**EGPH 24 then "06", the same afternoon.** Auto takeoff at TOGA with a
1,500 fpm dial reached 273 kt by the 1,000 ft handoff (the climb's
overspeed guard watched Vne 330 alone); the pilot's own climb then reached
Vmo and the A330's high-speed protection pitched it into a 5,400 fpm climb.
Autoland was armed on the reciprocal 3.6 nm out at 333 kt and 2,400 fpm up.
For the next twenty-two minutes the loop commanded -1,200 fpm on the VS
dial, the aircraft climbed to FL350 at full throttle and 180 kt, 90 nm from
the runway with the phase still `Intercept`, and the watchdog said
"Autopilot is not holding the descent" exactly once. Three changes: the
takeoff's overspeed guard is Vne OR the 250 kt rule plus the band
(`climb_guard_vne`), whichever is lower, since every takeoff is below
10,000 ft; the VS watchdog re-engages the SERVOS as well as the mode at
its warning (an AP kicked off by an airframe protection answers to neither
a dial nor a mode command) and GIVES UP with a flyable aircraft after
another minute of divergence (`vs_divergence`); and an approach whose
distance to the runway opens past the arm distance plus 5 nm, not on a
missed approach, is abandoned as `receding`.

**ATC: the tower now says "heavy".** The station read the tail number
alone. It reads `sim/aircraft/weight/acf_m_max` too and
`tower::spoken_callsign_wake` appends the suffix at 136 t
(`HEAVY_MTOW_KG`); the crew's own calls go through
`circuit::crew_callsign`, which the five duplicated sites now share. Not
done: the wake clock is still a flat 120 s behind any airline-shaped
departure regardless of either aircraft's category, and there is no
airline telephony for the player (a Lufthansa A330 is worked by its
registration); both are open.

Open after this pass: nothing here is flown yet. The first A330 sortie
should read `airliner_start` `step` events for any `NotTaken`, the
`autoland` `cmd_vs` / `vs_dial` / `vvi` columns against the 737's arrest
numbers, and the transponder knob's readback. The pitot-heat tool still
writes the stock switch, which the aircraft rewrites; its AUTO logic
(airborne, or an engine running on the ground) makes the advisor's read
true in flight, so the write's silence is masked rather than fixed.

## 2026-09-21: "I couldn't get us onto the runway", said on the runway

EGPH 24, the A330, full auto, 22:28Z on the 20th (build 40dba24). The line-up
drove the lead-on and stopped on the centreline, heading 239, 211 m in from
the pavement end (the displaced threshold Delta's lead-on joins at); the
tower's release watch logged `lined_up=true` for three minutes. Full auto ran
the line-up twice, then gave up `line_up_failed` with "I couldn't get us onto
the runway. Your aircraft."

The fit had failed on LENGTH, not position: 6,000 ft x 1.4 = 2,560 m against
a 2,549 m strip. That exact case had been fixed the day before
(`aircraft::takeoff_roll_m`, the roll scaled to the weight on the wheels) but
only in the spoken takeoff tool. The pavement rule existed in FIVE copies:
the takeoff tool, the circuit tool, the line-up tool's full-length check,
full auto's `lined_up`, and the arm preflight. Four still asked the MTOW
figure at every weight.

Fixed: one rule, `aircraft::takeoff_needed_m` (pure) and
`takeoff_needed_now_m` (reads `m_total`), behind all five; `mtow_kg` added to
the `LineUp` and `Circuit` tools. At the ~200 t flown it asks ~2,110 m, inside
the 2,338 m left from the displaced threshold. Full auto's check is now
`runway_check`, which returns the failed gate (`why_no_fit`), and the give-up
speaks `autotakeoff::no_fit_line` (one wording, extracted from the takeoff's
refusal) and blackboxes `line_up_no_fit`; a `RunwayTooShort` gives up after
the first attempt, since a second line-up adds no pavement. "Couldn't get us
onto the runway" survives only for the gates where it is true.

Noted, not changed: the first line-up went out eight seconds BEFORE the
tower's conditional "behind the landing ... line up behind" arrived, and the
second attempt re-drove a line-up the aircraft had already completed.

## 2026-09-21 (later): the takeoff handed a pilot a runaway

Same sortie. After the give-up the pilot said "Let's take off" (22:32:55Z;
the spoken takeoff tool had the weight fix, so the fit passed). The roll and
rotation were clean. At 1,011 ft and 203 kt the loop said "Flaps are still
out, we're too slow to clean up. Watch the placard", then "Your aircraft. The
autopilot is holding runway heading and the climb", and exited. What it left:
takeoff thrust, VS +1,500, flap 1, the altitude dial at 100 ft. Flap
overspeed (236 on a 215 placard) twelve seconds later; "through the selected
altitude, 100 set" three times; Vmo (330 kt) by 10,000 ft, the overspeed
warning every thirty seconds, flap 1 still out at 11,400 ft.

Four causes, each its own:

- **The schedule could not be flown.** The A330 profile retracted to clean
  "above 230" out of a CONF 1 whose placard is 215, and to CONF 1 "above 200"
  out of a CONF 2 whose placard is 196 (the flight stepped at 200.25). Now
  210 and 180, and `every_profiles_flap_schedule_can_be_flown_inside_its_placards`
  walks every profile: a detent's retraction gate must sit under the placard
  of the detent being left. The test found the second gate; only the first
  had been diagnosed.
- **The handoff left at the height, whatever the wing.** Three more seconds
  of climb would have cleaned up. `handoff_waits_for_clean_up`: dirty, under
  the clean floor, inside `CLEAN_UP_WAIT_S` (30 s), the loop keeps flying the
  climb and the schedule does its job. Both handoff kinds.
- **The loop's overspeed guard died with the loop.** `climb_throttle` holds a
  jet back while the takeoff runs; at a PILOT handoff nothing replaced it.
  Now the flight governor is armed (`handoff_speed_kts`, the profile's cruise
  IAS through `airspace_limited`, so 250 below 10,000) and the line says so.
  An orchestrator's handoff arms nothing: it owns the throttle.
- **The dial was nobody's.** `handoff_altitude_dial_ft`: a dial the climb can
  never capture (under us plus 500) is set 2,000 ft up, rounded up to the
  thousand, blackboxed `handoff_alt_dial`; a dial the pilot set above us
  stands. The line names the level either way.

The handoff line is now, for example: "one thousand feet, 212 knots. Your
aircraft. The autopilot is holding runway heading and the climb to 4,000
feet, and I'm holding 250 knots."

Still owed from this sortie: the overspeed warning says "ease off the
descent" in a climb; the spoken takeoff never checks the transponder (two
"negative radar contact, recycle transponder"); "Do we have enough runway for
takeoff?" got no answer. And a flake:
`station::tests::squawk_watch_calls_radar_contact_only_when_the_code_shows`
fails about one run in four on the speaker, holding `SIDE_STATE_LOCK`, so
another test writes the watch without it.

## 2026-09-21 (third pass): the rest of the Edinburgh sortie, and the self-pick that chose Glasgow

**The overspeed warning** said "ease off the descent" five times in a
2,600 fpm climb. `monitor::overspeed_line` names the descent only when the
aircraft is in one (under -300 fpm).

**The transponder: the knob was never writable.** The takeoff does set ALT at
liftoff, through the shared writer, and discarded the result. The A330's
`laminar/A333/transponder/auto_on_knob_pos` was declared in the profile with
a VERIFY on whether it takes a write; asked live, the Web API says
`is_writable: false` (knob -1, stock mode 1, in the air). Every mode write on
this airframe had been a silent no-op, and the flight got "negative radar
contact, recycle transponder" twice. `TransponderModeKnob` gains
`up_command` / `down_command`; with both declared `radio::set_mode` STEPS the
knob (`step_knob`: read, one command toward the target, 150 ms, read again,
six steps at most, the dataref never written) and answers with what the knob
reads. The A330 declares `auto_on_off_right` / `_left` (from
A333.transponder.lua; both confirmed present on `/api/v2/commands`). The
takeoff's set now runs on its own task (stepping a knob costs a few hundred
milliseconds at the rotation), blackboxes `transponder_alt` and SAYS a
failure. The pilot's ELATES clist for the same aircraft names the same knob
and positions: a second source, used as a reference only.

Owed from this: the profile probe checks that a name EXISTS, not that a
dataref the profile WRITES is writable. `is_writable` is in the same index
the probe already fetches.

**"Do we have enough runway for takeoff?"** fell to `deterministic-miss`.
`answers::takeoff_runway_query` (length-shaped questions only) and
`takeoff_runway_answer`: the takeoff's own `runway_fit` with the takeoff's
own `takeoff_needed_now_m`, said as two numbers ("Yes. Runway two four, 2,340
metres ahead of us, and we need 2,110 at this weight"), or the gate that
fails. `aircraft::profile_takeoff_roll` is now the one derivation of the roll
and its weight, for the registry and the answer.

**The flaky station test** was order, not a race. `assign_squawk` re-arms the
watch only for a DIFFERENT code, and the code is derived from the callsign:
an earlier test with the same callsign left its watch (Departure's, already
nudged) for `squawk_watch_calls_radar_contact_only_when_the_code_shows` to
find, about one run in four. The test clears the watch first; and in
production a re-issue of the same code by a new controller now moves the
watch's VOICE to that controller, keeping its history.

**The VS watchdog** ("Autopilot is not holding the descent") fired at the arm
of the EGPF approach, where the A330's VS mode was flying toward a -1,200
step and simply takes longer than five seconds, and four more times in the
missed approach, climbing at 970 fpm on a +150 dial. `vs_not_held`: more than
700 fpm from the command AND not closing on it by 15 fpm a tick; a LEMI dive
(steady, cmd 0) still counts every tick. `vs_not_held_line` says descent,
climb or level by what is commanded.

**The self-pick.** "Land the aircraft" at 13,500 ft chose EGPF 23 from
18 miles by distance plus turn, nothing else. Now `self_pick`:
`self_pick_min_length_m` (the published landing distance x 1.3, never under
the old 1,200 m: 1,882 m for the A330), the SURFACE wind of each candidate's
field from the METAR file (`weather::surface_wind`, one lookup per field,
never the aircraft's wind datarefs), a tailwind costing half a mile a knot
(`TAILWIND_NM_PER_KT`), and up to half a mile for pavement beyond the minimum
so the longer parallel wins (`LENGTH_BONUS_MAX_NM`). `landing_score` is pure
and tested; the pick blackboxes `self_pick` with the minimum, the runway and
the wind it chose on. The overflight balk at EGPF (166 ft at 0.4 nm) is NOT
diagnosed here.

## 2026-09-21 (fourth pass): the Sparky 747, audited before it was flown

Sparky744 (mSparks' xtlua 747-400, `Aircraft/Sparky/747-400`, ICAO B744)
installed; the app built `generic_b744` from the acf. Audited from its scripts
(four folders: xtlua, xtlua_keysystems, xtlua_fdr, xtlua_nd; names only, the
aircraft is CC-BY-NC; its own B747_Commands.txt / B747_Datarefs.txt ship
empty) and probed parked, cold and dark.

**`is_writable` proves nothing on an xtlua aircraft**: every dataref it
creates reports true, and the script may rewrite the value next frame. The
probe is a write and a read a second later (scratch `probe.py`). That way:
a stock taxi-light write is erased within a second; `laminar/B747/
toggle_switch/position[]` ignores writes (an animation output); the light
TOGGLE commands move the real switches (nav = 9, taxi = 7; 0.0 is ON) and
the stock light datarefs follow as honest reads.

**It owns the autopilot**, and that is why the profile declares
`autopilot: false`. `B747.70.xt.autopilot.lua` parks the STOCK altitude dial
at 150,000 ft (`simDR_disabled_autopilot_altitude_ft`) and captures off
`laminar/B747/autopilot/altitude_dial_ft` (the MCP window is
`laminar/B747/autopilot/heading/altitude_dial_ft`); `fma_rollModes`, every
frame, rewrites the stock heading bug from `laminar/B747/autopilot/heading/
degrees` in HDG SEL; the servos engage on `button_switch/cmd_L`. Every
autopilot tool and automation here writes the stock dials. Unpowered, a stock
dial write and a laminar dial write never reach each other. The aircraft's
replacement for `sim/autopilot/vertical_speed` is a SET (vs_status = 2), not
a toggle, so that one command is compatible. OWED: a per-airframe autopilot
seam (heading input, altitude input, engage command) published through
`flight_state::publish_airframe`; until then no AP tools, takeoff, circuit,
autoland or full auto register on this airframe.

Also read: a FADEC (`B747.42.xt.EEC` holds `override_throttles`, PIDs
`ENGN_thro_use` to a lever-angle target); the transponder's stock mode is
min(selector + 1, 2) off `laminar/B747/flt_mgmt/txpdr/mode_sel_pos`; the
fuel control switches are `laminar/B747/fuel/fuel_control/toggle_sw_pos[0..3]`
(mixture is their output); the gear lever is three-position
`laminar/B747/actuator/gear_handle` moved by the REPLACED stock gear
commands; the park brake bleeds off without hydraulic pressure and is forced
on while chocked; the acf's clean stall (216) puts the 1.2 floor at 259 kt,
above the 250 cap.

Shipped: `profiles/boeing_747_400.yaml` (every line proven, read from the
scripts, or marked VERIFY; all 26 names exist in the sim), B744/B748/B74F in
`detect::known_profile`, the 747 in the turbine-profile and flap-schedule
gates. `gear_by_command` (the gear tool fires `landing_gear_up/_down` instead
of writing the handle), and the gear readback now compares with the INTENT: a
failed read used to answer with whatever was asked. A fuel cutoff lever may
be an array element (`split_index`, written by index). The pilot's XChecklist
clist for this aircraft (royaloak / SkyBloo) supplied several names, all
confirmed live; it carries `laminar/md82/*` items that do not exist here, and
is a reference, never a source.

## 2026-09-21 (fifth pass): the A330 round Glasgow, and everything a crew sets before it moves

EGPF 05, the A330, "let's do a circuit" from a runway start, 07:34Z. The
pilot's questions came first (why does "A.P.U." sound robotic, are we setting
the takeoff config, when do the seat belt signs go on, why is the selected
altitude 100, and the transponder?) and the blackbox answered each of them.
Fixed offline (the sim was closed for the rest of the pass); every new profile
line is marked VERIFY for the next ramp.

- **"A.P.U." was three sentences.** Every dot is a full stop downstream:
  `speech_chunks` cut "A.P.U. running." into two synths with the 0.22 s
  sentence gap between them, and `phonemize` put a stop's pause after each
  letter. The readbacks' own convention is "A P U" (`say_letters`); the A330
  flow and jet_start used dots. `tts_onnx::undot_acronyms` rewrites the dotted
  form before the chunker sees it, keeping the closing dot only where it also
  ends the sentence. The source lines stay dotted, so the panel still reads
  "A.P.U." and a profile `say:` cannot bring it back. In-process voice only.
- **The takeoff configuration was set during the roll, or never.** The flap
  handle was written after "Brakes released. Rolling" (CONF 2 still running
  out at sixty knots), the altitude dial sat at the 100 ft the aircraft loads
  with ("through the selected altitude, 100 set" twice on the climb), and
  nothing had ever armed the ground spoilers or set the autobrake although the
  checklist asks for both, so the Monitor list named them every flight.
  `src/takeoff_config.rs`, one page, three callers: full auto after the engine
  start, the auto takeoff before the brakes come off (where it WAITS for the
  flaps to arrive, `flap_handle_deploy_ratio`, 40 s), and autoland at the gear
  (`set_for_landing`: spoilers armed only from stowed, asked again while the
  descent has the speedbrake out; the landing autobrake level). Read first,
  left alone in state, read back, named when it did not take, one milestone
  line. The profile's `takeoff_config:` declares THAT the airframe has
  spoilers to arm and an autobrake (stock datarefs, no names). The altitude:
  `initial_altitude_ft` - the caller's level to the hundred, else a dial the
  climb could never capture is lifted 3,000 ft above the field, and a dial
  the pilot set is the pilot's.
- **The transponder logged `ok: true` and squawked standby.** Read live in
  Glasgow's zone: mode knob 0 (AUTO), `ta_ra_knob_pos` 0, stock mode 1. The
  A330's script computes the stock mode from THREE knobs, and with TCAS in
  STBY, where the aircraft loads, AUTO is standby. The readback believed the
  one knob it turned. `TransponderModeKnob.also` (companion knobs, stepped by
  command, ON and ALTITUDE only) and the verdict is then the STOCK mode
  (`stock_transmits`: 2, 3, 5, 6, 7), which on such a box is the add-on's own
  answer over all of them.
- **The seat belt signs went on four seconds after liftoff.** The only hand on
  the switch was an advisor row pressing "up" once (OFF to AUTO), then again
  seventy seconds later when AUTO read dark with the gear up. `tools::signs`:
  on before anything turns (full auto and the spoken airliner start), AUTO
  through ten thousand in the climb, on again through ten thousand in the
  descent (`Watch`, 300 ft band), on for the arrival whatever the cruise did.
  The A330's switch is its own and read-only (`seat_belt_signs:`, stepped);
  any other airframe with a cabin gets the stock switch; no cabin, no signs.
  The advisor row keeps takeoff, approach and landing only.
- **More of the autopilot.** The circuit flew altitude through the VS dial
  ("never ALT_HOLD") and wandered 1,796 to 1,840 ft down the downwind behind
  a dial at 100. Where the profile declares `ap_altitude_capture`, the level
  target goes on the dial (`circuit_dial_ft`), a CAPTURED hold on that dial
  owns the leg and the VS law stands off (`ap_holds_level`: re-pressing VS
  would drop the hold every tick), a new level target is a new dial plus VS,
  and on the glidepath the dial is parked above by autoland's own rule. It
  rests on the stock VS command leaving ALT, unproven on the 737's MCP, so it
  is per airframe (the A330 only) and an autopilot that will not take VS for
  8 s is said once (`vs_refused`). The tick blackboxes `ap_level`.
- **A runway start skipped the tower.** X-Plane put the aircraft on the
  numbers; full auto's "already lined up? skip to the circuit" skipped the
  call, the clearance, the squawk and the strobes, and the tower's FSM, still
  Parked, would have answered "taxi to holding point Alpha first" anyway. The
  A330 flew the zone to "you are inside controlled airspace without
  clearance". `Ctx::own_on_runway` (the station's own read, the strip in use):
  a "ready for departure" from the strip is a lined-up aircraft, cleared like
  one and held like one. Full auto's runway-start branch lights up, calls,
  waits on `ensure_takeoff_cleared`, and dials an owed squawk.
- Smaller: the start flow read `battery_on[0]` as a scalar (a 400), so both
  battery steps logged null and "did not take" (`read_position`); the
  rectangle circuit never turned the landing and taxi lights off in the climb
  although the authored route's comment said it did ("taxi light, off" on the
  climb list); "let's do a circuit" on the 747 got "that command isn't wired
  up yet" (`unregistered_tool_line` now says the autopilot cannot be reached
  on an airframe with `autopilot: false`); "start the engine" on a twin.

Owed from the same sortie, all needing the sim: the base leg overshot final
by four kilometres at 225 kt (flaps wait for Vfe, and the pattern speed is
above CONF 1's); the monitor called "getting slow" at 170 kt with CONF 2
because `stall_speed_at_flap` is a straight line and a slatted wing takes most
of its reduction at the first detent (per-detent stall speeds want real
numbers, not guesses); the stock autothrust is still unused (`autothrottle:
false`: the tool is the MD-82's); the authored LXGB route has no altitude
capture; signs off after shutdown at the stand.

**The 737, probed parked the same afternoon** (a write and a read 1.2 s
later, each restored): `fasten_seat_belts` holds 0 / 1 / 2 and the annunciator
follows, so the stock signs path is right for it, and its checklist's "SEAT
BELT SIGNS ON" tested for 1, which is AUTO (now 2); `auto_brake_level` 0 holds
and `speedbrake_ratio` -0.5 holds, so the 737 declares `takeoff_config` too
(landing autobrake 2 = stock level 3). Its advisor row lost `climb` for the
same reason the A330's did. `ap_altitude_capture` stays off on the 737: VS out
of ALT HOLD is still the open question from 2026-09-19 and cannot be probed on
the ground.

## 2026-09-21 (sixth pass): EGPH-EGTE in the 737, three approaches that could not land

Full auto flew Edinburgh to Exeter (flight 130). Gate to the last fix was
clean: start, clearance, pushback (77.6 m, delivered), taxi, takeoff, the
departure handoff, thirty fixes at 17,000 ft, the descent. Then three
approaches, two go-arounds under 150 ft and a give-up, and the pilot landed it
by hand at 12:39Z (347 fpm, firm). The blackbox, in order:

- **Crew and tower picked opposite ends.** The arrival chose 26 from Exeter's
  METAR (190 at 3). "Final runway two six" earned "negative, runway zero
  eight ... wind three five zero degrees, eight knots, cleared to land": the
  tower picks its runway by the sim's wind datarefs, which are the air the
  AIRCRAFT is in (2,700 ft), and 350 is dead across both ends, so the pick was
  a fraction of a knot of headwind. Autoland re-armed onto 08 from the 26
  final, 6.5 nm the wrong side of the field.
- **The reposition cannot work for a jet.** Every reposition aimed AT a point
  on the extended centreline (the FAP, the join point). From the far side of
  the field that is a converging downwind: heading 271 against a 259
  reciprocal, 5.3 km of spacing closed to 1.25 km. "Final" was called at
  3.3 nm (position "interceptable") with the nose pointing away from the
  runway; the 170 degree turn is 3 km wide at 160 kt, overshot to 1,880 m on
  the far side, and the pursuit closed exponentially with 74 s to run:
  430 m off at 1 nm, 207 m at 94 ft, `off_centreline_low`. Second try: 2,300 m
  overshoot, 1.2 km off at 575 ft AGL, `unstabilised_below_floor` at 143 ft.
- **The missed approach was replaced at 60 ft.** The go-around took the tower
  from ClearedToLand to Inbound; the ident the party line publishes with the
  state is the runway IN USE, which flipped to 26 in that moment.
  `clearance_switch` accepted Inbound and ran during the go-around: "Tower
  cleared us for runway two six" five seconds in, the sticky missed stage
  dropped for a fresh loop (which also lost `owe_final_report`, so the second
  approach flew to 143 ft with the tower still at Inbound: no clearance).
- **The third try was killed on the tick its turn began.** The `receding`
  guard measured from the ARM distance; armed over the runway in a go-around
  (0.9 nm) the limit was 5.9 nm, the join point at platform height is 6.5.
  Heading command 077 to 256 at 12:26:06, give-up at 12:26:07.
- **The hand-back was a runaway.** Every give-up writes a flat 0.75 lever (a
  piston figure: gear and flap out, EGHI 2026-08-02). Level 737, half flap:
  152 kt at the hand-back, 240 forty seconds later, 346 kt and ten overspeed
  warnings.
- On the way round: full flap on the downwind at 1,900 ft (the distance
  schedule), flap 40 cycling 1.0 / 0.5 / 1.0 on its 162 kt placard, IAS
  hunting 139 to 162, VS command +330 to -360 in "level" flight (the
  reposition's platform is read off the radio altimeter, so it chases the
  hills).

Before the approach:

- **278 kt at the last fix.** 2,000 fpm down with the governor at idle and no
  drag device: 258 through ten thousand against 250, 267 at six, 278 at TIVER.
- **Inbound call into the void, marked as made.** The enroute leg calls when
  the region has made the destination's tower the station, but a region
  change near a field is a crew NOTICE ("closest field is Exeter"), no
  handoff, so nothing retuned: COM1 was still on Carlisle from 11:17. The
  arrival stage skipped the call (`inbound_already_made`) and Exeter
  challenged "inside controlled airspace without clearance".
- **QNH at 17,000 ft over England.** The 737's `transition_altitude_ft` is
  18,000 and its comment says the plan overrides it; nothing parsed one. The
  monitor RESET the knob to each new QNH three times in the cruise.
- "Edinburgh Departure on one two four decimal eight two five" earned
  "Edinburgh Tower, say again": no readback verb in it. The region handoff's
  readback has logged "void" on six flights of six (the station is swapped
  before the crew reads back). The descent list spoke "seat belt signs, on" as
  a discrepancy at the top of descent; the signs law turns them on at ten
  thousand. `altimeter stepped ... now=` read the value from BEFORE the
  presses (the knob is in the 500 ms slow cache and a command dropped nothing).
  The debrief counted the halt AT the holding point as a taxi that "rolled to
  a stop before its holding point", and graded the first twenty degrees of
  every turn as the straight run's heading error (six FAILs of nine).

Fixes, each with the flight's numbers as its test:

- [src/reversal.rs](../src/reversal.rs): the course reversal as a PATTERN.
  Outbound on a line parallel to the centreline three sizing radii out
  (pursued, never homed on), base perpendicular a radius before the base
  position beyond the join point, inbound on the final's own fixed-lead
  pursuit until the ordinary guidance calls a final with the nose within 75
  degrees of the course. Sized for 18 degrees of bank on purpose (an
  over-estimated radius turns early, which the inbound pursuit closes; an
  under-estimated one is the overshoot). Side: the aircraft's own when it has
  one, else the lower ground of two terrain probes, else left-hand. Platform
  in MSL, fixed at the start (raised to terrain + 1,000), deadbanded. Started
  by the latch's Reposition stage (which it HOLDS until the pattern ends) or
  by `reversal::needed` (a final called more than 75 degrees off the course,
  or a centreline aim point that would be reached flying away). Three
  patterns, then a give-up. Half flap and the pattern speed on the downwind,
  gear on the inbound leg. The sortie test flies a 24 degree 737 and a 15.5
  degree single through it: under 60 m off inside 3 nm.
- `unstabilised_at_gate`: between 1,000 and 500 ft on final, a predicted
  residual over 120 m (`reversal::predicted_residual_m`: saturated cut, then
  exponential in time; it says 162 m for the final that arrived 207 off) is a
  go-around, ORDERED through the latch so the tower hears it.
- `clearance_switch` never during a missed approach; the arm-time check goes
  through it too. `recede_ref_nm` follows any pattern flown and the guard
  stands off during one. `flap_held_within_limits` (5 kt to extend, 8 over to
  retract). Every give-up hands the speed to the flight governor
  (`hand_back_speed_kts`) and says so.
- Tower: arrival states freeze the runway in use; an arrival's stated runway
  stands when the wind is inside the swap margin and no stream lands the
  other way (`note_stated_runway`); the app feeds the field's METAR surface
  wind (`station::set_surface_wind`, keyed by position; `monitor::
  feed_surface_wind`, above the quiet gate) and an airborne caller gets it.
  `frequency_readback`; `void_outcome` files a readback within 45 s of a
  region switch as `handoff_readback`.
- `procedure::descent_drag`: speedbrake at 8 over the governor's target (in at
  3), 75 % of the rate at 12 over and half at 20; stowed on every exit by a
  wrapper round the fly loop. `call_inbound` tunes COM1 first. The plan's
  `origin/trans_alt` and `destination/trans_level` are parsed and published
  at the arm (`instruments::FlightTransition`), resolved by every consumer of
  `correct_setting_inhg`. The signs are `held_by_automation` under full auto.
  A command clears the value cache.

Owed, all needing the sim: the pattern has not been flown live (the turn-in
lead on a real autopilot, the terrain side pick, the go-around straight into
it); flight-level phraseology does not exist on either side of the radio
("climbing one seven thousand" above a 6,000 ft transition); the monitor has
no Descent phase (a 2,000 fpm descent is "Cruise"); the climb segments grade
against Vy on a jet; a region NOTICE near the destination still retunes
nothing until the inbound call does; autoland's intercept tick ran at 0.9 to
1.3 s (the heading bug's knob presses).

## 2026-09-21 (seventh pass): EGPH-EGCC in the 737, SPEED BRAKE ARMED at 7,000 ft

The pilot, in the climb: "I don't think the copilot is doing a proper takeoff
config", with the annunciator lit. The blackbox agreed with both halves: the
after-start call had set `["flaps", "trim", "spoilers armed", "autobrake
max"]`, the pre-roll call found nothing to add, and `speedbrake_ratio` still
read -0.5 at 250 kt. Two faults. The fifth pass wrote the A330's habit
(ground spoilers armed for the roll) into one `arm_spoilers` flag and the 737
profile copied it, against the 737's own BEFORE TAKEOFF list in this repo
(`SPEED BRAKE|DOWN DETENT`), which never ran to say so: the phase tracker's
flight begins at the takeoff roll, so full auto audits no list before it.
And nothing on either airframe ever disarmed the lever after liftoff.

Fixed: `takeoff_speedbrake_down` (737: the lever is read and put DOWN; a
reject above 40 kt raises it, `takeoff_config::on_reject`, blackbox
`reject_speedbrake`, because nothing is armed to do it), and
`takeoff_config::after_liftoff` at the gear (armed only; a lever that is OUT
is somebody's; blackbox `spoilers_disarmed`). Tests carry the flight's state.

**The climb-out.** Rotate 145, liftoff 157, 185 kt when the AP took it at
340 ft, then VS +1,500 at takeoff thrust: `flap_step` at 402, 500 and 739 ft
on speed alone, 240 kt at the 1,008 ft handoff. `climb_fpm` is
`best_climb_fpm x 0.8` clamped to 1,500, a trainer's ceiling: the 737 has
about 3,900 fpm of excess at that weight (4.1 kt/s, a knot a second per
1,000 fpm, both read off this flight's `climb` records) and everything the
dial did not take went into speed. Fixed as a declaration
(`reference.acceleration_height_agl_ft`, 1,000 ft on the 737 and the A330,
`autotakeoff::ClimbOut`, republished per swap): below the height
`climb_out_vs_fpm` adds 100 fpm per knot over `initial_climb_kts`, capped at
`best_climb_fpm`; the flap gate is the height, not 400 ft; above it the old
law accelerates and the schedule steps; the handoff waits up to 45 s for the
SCHEDULE (`handoff_waits_for_schedule`), because its own clean-up is one
shot to zero on the clean floor. The gain closes the speed loop near
0.1 rad/s against the 737's VS mode (two 1.5 s lags, 2 s dead), and the
sortie test flies that plant: the old law reproduces the flight (240 kt, the
flaps gone under 1,000 ft), the new one reaches the height under 210 kt with
flaps 5, steps 2, 1, up at their speeds and hands off clean, no hunting. A
circuit's 500 ft handoff keeps the old climb-out whole. Blackbox:
`autotakeoff` `climb_out` at the arm; read `climb` `ias`/`cmd_vs` to 1,000 ft.

Still NOT fixed: no BEFORE TAKEOFF Monitor audit under full auto; the MCP
speed window left at 125 (no V2), no flight directors or autothrottle arm;
the hand-flown segment below 300 ft holds 8 degrees (the 737's tail-strike
ceiling) and is where 157 became 185 kt; takeoff thrust stays on until the
follower's governor takes it (no thrust reduction height).

**The arrival, EGCC 23L, same flight.** The morning's approach fixes held on
an ordinary ILS: intercept 67 m off closing to 0.2 m at the flare, on the
path from 4 nm, 145 kt at 39 ft, no balk, rolling handoff at 8.7 kt. Three
findings. (1) The pilot: "why is the yoke hard right?" on the taxi in.
`servos_on` 1, `heading_status` 2, `yoke_roll_ratio` 0, `total_roll_ratio`
0.50, the bug 0.25 degrees off the nose: autoland flies the AP to the
ground and NOTHING ever disconnected it, and a heading servo that cannot
bank an aircraft on its wheels winds the ailerons over. Fixed:
`autoland::disconnect_autopilot` at the rollout latch (`servos_off_any`,
read first, read back, blackbox `ap_disconnected`). (2) NOT fixed: the
rolling handoff came at 8.7 kt with exit W 1,292 m ahead, the vacate drove it
at 10 kt, `vacate_timeout` at 120 s with the aircraft still on 23L and the
incursion watch counting traffic down to 0.4 nm. A vacate that far away
wants the rollout to keep its speed, not taxi pace. (3) The speedbrake was
fully out from about 6 nm to 3 nm at the pilot's hand (no `speedbrake`
event); `landing_config` deferred the arm correctly and armed at 20:18:50
once it was stowed. Touchdown "about 500 fpm", firm.


## 2026-09-21 (eighth pass): the crew does the load sheet, and the Route tab shows it

The pilot: "when we load the flight plan does the copilot set up the load and
fuel?" It did not. The plan's block fuel and payload were only ever CHECKED
(the brief's "short about", `full_auto::preflight`'s refusals) and set by
hand in the sim's Weight & Balance screen. Every `weight/m_*` use in the tree
was a read.

Probed live on the parked 737 (EGPH ramp, engines off): `weight/m_fuel`
(per-tank array) and `weight/m_fixed` are writable and TAKE (tanks +10 kg,
centre 25 kg, payload +50 kg: `m_fuel_total` and `m_total` followed within a
second; restored). `acf_tank_rat` 0.187 / 0.626 / 0.187 of
`acf_m_fuel_tot` 20,894 kg, `acf_tank_X` -7.0 / 0 / +7.0 m: the lateral
position tells a wing tank from a centre tank on any airframe.

Built: `load_aircraft` ([src/tools/loading.rs](../src/tools/loading.rs),
registered on every airframe). "Load the aircraft", "load / set up the fuel
and payload", "set the fuel from the plan", "fuel the aircraft", "board the
passengers" (`grammar::load_aircraft_call`, before the compound gate; never
the fuel SYSTEM, the fuel truck's "refuel", a question, or "load the flight
plan", and the main loop's FMS-load branch asks `load_aircraft_request`
first because "load the fuel from the flight plan" has its shape). Gates: on
the ground, stopped. The plan: a local one stays, a SimBrief one is fetched
again (the `Asked` rule) and published. Fuel is the BLOCK, wings first in
proportion to capacity, the centreline tanks with the remainder
(`distribute_fuel`; a proportional fill would put 3.6 t of a 5.8 t load in
the 737's centre tank). More than the tanks hold, or an all-up weight over
`acf_m_max`, is refused with both numbers and nothing written; over the
PLAN's takeoff limit is said, since that is the preflight's refusal later. A
half already in state (1 %, 2 kg) is left alone. Read back from
`m_fuel_total` / `m_fixed` against the intent: an add-on that manages its own
tanks earns "the fuel did not take", never "Roger". A plan with no sheet
(local) loads the fuel and says the payload is as it was. Blackbox kind
`loading`, event `load_aircraft`. The preflight's fuel refusal and the
brief's "short about" (on the ground) now name the words.

The Route tab: a load sheet block above the route (plan, actual, diff for
fuel, payload and weight against max TOW; green inside the preflight's own
tolerances, red short of fuel or over the limit, amber payload) with a "Load
aircraft" button (request code 12, the same English, grammar-pinned). The
plan's side is a new mailbox, `xcopilot/status/loadsheet`
(`simbrief::loadsheet_payload`, from the monitor tick on the auto feed's
gate and resend); the aircraft's side the plugin reads itself every frame.
And the ACTIVE fix, with its magnetic bearing, distance and minutes at the
present groundspeed, its row lit green: `fms::active_fix` is the FMS's own
destination entry (whoever sequenced it), or the follower's last goto when
that named a leg the plan lacks (an approach leg), cleared by the next load.
Plugin rebuilt, installed and reloaded (build 3950a94-dirty.231119).

NOT flown: the tool has only run against the fake; the panel block has not
been looked at. Not probed: the A330 and any add-on with its own fuel
system. Owed: full auto could load at the arm instead of refusing.

## 2026-09-22: into Stansted in the 737, an altimeter at 17.5 inches

The pilot: "approach failed, I have had to ask copilot to land again". The
blackbox said why in one pair of columns: on the follower's LOREL leg the
altimeter read 4,675 ft and the true altitude was 16,884. It had been that
way since the wheels left the ground (-13,410 indicated at 1,724 true).

The 737 loaded this session with its baro memory at 0.0 (its xlua takes
`laminar/B738/barometer/capt_setting` from the stock dial at flight start,
and the dial was 0; the copilot's side still is). `set_altimeter` steps that
knob with the barometer commands, and `baro_steps` capped a call at 350
presses "because anything more is a bad read". Here 0.0 was a TRUE read, so
every ask moved the knob 3.5 inHg and stopped: 0, 3.5, 7, 10.5, 14, 17.5
across five asks (full auto's departure set twice, the follower's arm twice,
the monitor's reset). Every caller said it had worked: full auto "Altimeter
set" with whatever the knob read, the monitor "Altimeter reset, three zero
four four" BEFORE the write and nothing after it. With the pilot flying the
monitor then said the truth once ("you have seven zero zero") and, by its
own rule, never again.

The follower flies the altimeter. It cruised "FL190" at 31,480 ft, descended
on schedule, "reached 4,600 ft" at 17,000, closed nothing on LOREL
(`leg_stalled`, three stalls) and stood down. "Land the aircraft" self-picked
EGSS 22 from 16,700 ft and 13 nm; the reversal pattern has no height rule,
descended at its 800 fpm and turned final 12,600 ft above the field.

Fixed, offline:

- `set_altimeter` steps in ROUNDS of 350, re-reads after each, and another
  round is earned only by a dial that answered the last one
  (`keep_stepping`: closed on the ask by half of what was pressed; ten
  rounds). A dead knob or a bad read still gets one round. Test: a knob at
  0.0 is walked to 30.40 in one ask (3,040 presses, fake with
  `FakeXPlane::step_on_command`); a dead knob gets 350.
- `AltimeterSync` carries `want`, `took()` and `line()`: full auto and the
  follower's arm say "The altimeter did not take. It reads X, the setting
  is Y. Set it by hand." at Warning; the monitor's reset reads the result
  back and says the same (blackbox `callout` id `altimeter_reset`, `took`).
- The reversal pattern has a height rule (`reversal::too_high_to_turn`):
  the downwind extends while the aircraft is higher than the crossing and
  the run back in can lose (bounded at 25 nm along, said once, blackbox
  `downwind_extended`), and more than 1,500 ft above the platform descends
  at the airframe's enroute rate capped at 1,500 fpm
  (`pattern_descent_fpm`) instead of 800.

Then the pilot: "I think it's stuck". Not stuck: 52 nm southwest of the
field at 2,300 ft AGL and going. The pattern's INBOUND leg had crossed the
field 12,600 ft up, never a final, and that leg has no exit once the runway
is behind it: the pursuit's aim is astern, the 35 degree cut flips sign at
every crossing of the centreline (cmd_hdg 258, 188, 258), and the pattern
masks the overflight balk (`balk: None`). `reversal::overflown` (inbound and
past the threshold) drops the pattern, blackbox `reversal_overflown`; the
ordinary `needed` test starts the next one or reaches the give-up.

The flight was recovered from outside the app, over the Web API and the
typed-command mailbox (`xcopilot/typed`), with the pilot's say-so: a typed
"my controls" ended the pattern (and released everything: the gear was
re-raised and the speed re-held afterwards), the baro was stepped 17.5 to
30.42 with 1,292 `barometer_up` presses (30 s, no frame-rate complaint), a
script steered the autopilot's bug and VS dial abeam the field, downwind to
13 nm at 3,000 ft, base, and onto the 22 centreline, flaps 5 and 190 kt
through the copilot, and typed "land the aircraft" at 11 nm, 0.1 nm off the
line. The pilot had restarted xvl on the fixed build by then. Autoland
self-picked EGSS 22, intercepted from below, touched down a metre off the
centreline at 146 kt, "Down and stopped. Brakes on, your aircraft." So the
ordinary approach is sound on the 737 from a sane start; both failures
belonged to the start it was given.

Three more from the recovery:

- A typed "hold two two zero knots" armed the governor at FOUR knots (said
  as 55, the floor): `grammar::number` summed digit words. Speech is
  normalised to digits before the grammar; typed text is not. Digit words in
  a row now concatenate (test `digit_words_in_a_row_are_one_number`).
- The fuel caution was said every 31 s, word for word, for four minutes:
  the estimate rides the live flow, the governor walked it over the 60
  minute edge, and each wobble was a cleared-and-recurred episode; the
  Warning band repeats on the 30 s clock by design. A band once entered now
  holds until the estimate is 10 minutes clear (`FuelEndurance.entered`),
  a fuel line is said again only per ten minutes lost (`repeat_level`,
  `fuel_warning` exempted from the Warning clock the way the cabin altitude
  is), the later lines have their own words, and the em dashes are gone.
- OWED: "Brakes on" was not true. `parking_brake_ratio` read 0.03 after the
  stop on the 737 (the autobrake was holding it); the spoken "set the parking
  brake" tool then set it (1.0). The rollout's closing write needs the
  readback its line claims.

Flown: the ordinary autoland and the restart on the new build. NOT flown:
the altimeter rounds, the pattern's height rule, `overflown`, the fuel
lines, the digit words. Owed: a cross-check of indicated against true altitude in the
follower (temperature error is 10 % at cruise, so the gate is not a flat
number); the 737's fast path (two presses of
`laminar/B738/EFIS_control/capt/push_button/std_press` put the memory at
29.92 from anywhere, 160 presses from any QNH, where 0 to 30.40 is 3,040
API commands); the copilot's side altimeter, still 0.0; why the dial was 0
at load.

### 2026-09-22, EGSS-LFPG: the arming ack was a paragraph

"Full auto, EGSS to LFPG. Fuel 12,880 pounds aboard for the plan's 12,990
pounds. 175 passengers and 10,030 pounds of cargo, loaded as planned. I'll
start up, get our taxi, take off, and fly the 14 enroute fixes at 25000
feet, then the approach into Paris - Charles De Gaulle. Paris - Charles De
Gaulle has the ILS on zero nine left, the ILS on two seven right, the ILS
on zero nine right, the ILS on two seven left, the ILS on zero eight left,
the ILS on two six right, the ILS on zero eight right, t..." and the panel
ran out of room before the wind sentence. Three cuts, no facts lost:

- The fuel unit is said once ("12,880 pounds for the plan's 12,990"), the
  load is "as planned".
- The mission is not described; the pilot who said "fly the flight plan"
  knows what full auto is. The ack is the plan's numbers: "Cruise 25,000
  feet, 14 enroute fixes." The destination's name lives in the arrival
  line, not twice.
- The arrival brief is SIZED TO THE FIELD (`full_auto::arrival_brief_line`,
  pure, tested): every runway end covered is "the ILS on every runway" (or
  "a published approach" when the kinds differ), more than two ends is a
  count ("on 5 of 8 runway ends"), a small field is still named end by end
  with its straight-ins. "The wind at the time picks the runway" is gone:
  it said nothing the pilot did not know.

Now: "Full auto, EGSS to LFPG. Fuel 12,880 pounds for the plan's 12,990.
175 passengers and 10,030 pounds of cargo as planned. Cruise 25,000 feet,
14 enroute fixes. Paris - Charles De Gaulle has the ILS on every runway.
Cancel or my controls any time."

### 2026-09-22, LFPG 09R: the elevator flare never reached the elevator

The sim's landing report: -672 fpm, 2.5 g, 0.1 degrees of pitch. The
debrief: "-221 fpm, within private standards". The autoland ticks: the
roundout opened at 83 ft radio sinking 801, the flare gate at 36 ft
sinking 772, `flare_elev` pinned at the 0.35 cap for three seconds, and
the sink at the wheels about 690. A 737 does not touch down at 0.1
degrees of pitch with a third of aft yoke held; the elevator command was
going nowhere.

Every 737 landing since the elevator law shipped (2026-09-21, 12:39,
18:19, 23:00, and this one) shows the same: 0.31 to 0.35 of elevator
commanded, and a sink curve with exactly the shape the VS dial alone
produced before the law existed. The plugin's own comment already named
the owner: "the pilot's pitch hardware (or the engaged AP servo) owns the
elevator the very next frame". `override_joystick_pitch` stops the
joystick being read into the yoke; it does not stop the autopilot's servo
writing it, and the servos were on through the flare. The takeoff, where
the channel is proven, rotates with the servos OFF. Probed on the parked
aircraft: a 0.3 write with the servos off put the yoke at 0.30 and the
elevator at 6.6 degrees, so the channel and the installed plugin are
fine; the servos-on half could not be probed because the Laminar
autopilot refuses to engage with the engines off.

- **The arrest takes the servos** (`arrest_hands_step`, blackbox
  `arrest_hands` with `taken`/`ok`/`radio`): `servos_off_any` at the
  roundout gate, pitch on `xcopilot/pitch`, the runway track on
  `xcopilot/roll` through the takeoff's `WingsLevel` with
  `climb_bank_target` on the crabbed-heading error (wings level with no
  heading read, no command with no bank read), power on the governor. The
  tick blackboxes `flare_ail`, `hands` and `phi`. A balk or leaving the
  band is `Return`: servos on, HDG and VS re-engaged, the dial rewritten.
  The rollout latch's `disconnect_autopilot` finds them already off. Mode
  housekeeping is skipped while the arrest has the servos (an MCP mode
  press that engages the master would put the servo back on the yoke at
  fifty feet), the VS watchdog stands down inside the arrest, and the
  roundout ticks at the flare's 100 ms (`tick_ms` takes `arresting`).
  The roll yield latch is honoured like the pitch one ("You have the
  roll. I'm on the elevator and the power."); the pitch line is now true
  about the wings ("I'm holding the wings and the power").
- **The debrief's touchdown was a sample on the oleos.** The state
  records run at about 1 Hz and the ground flag samples late: -690 at
  12 ft, then -221 at 1.6 ft with the flag still clear, then ground at
  -38. The steeper-of-two rule saw 221. `instructor::touchdown_sink_fpm`
  takes the steepest sink inside `TOUCHDOWN_WINDOW_S` (2.5 s) of airborne
  samples or the ground sample, and both the debrief line and the ACS
  landing grade use it: today's flight now reads -708 fpm, "Firm. Below
  private standards".
- **The sortie test's plant was three times too quick.** `fly_jet_arrest`
  flew two 1.5 s lags and 2 s of dead time (the EGCC fit); the LFPG dial
  sequence fits 3 s of dead time and two 5 s lags (RMS 51 fpm), and
  through that servo the dial path arrives below -450 whatever the lead,
  a pinned +1000 included
  (`the_dial_path_cannot_arrest_the_737_through_its_logged_servo`). That
  is the arithmetic behind taking the servos, and the test that would
  have failed on the 19th.

NOT flown: the hand-flown arrest. The `FlarePitch` law (P 0.06 per
100 fpm, D 0.05, trim 0.015, cap 0.35) has only ever flown its test
plant; the first live flare with the servos off is its first flight, and
the things to read are `flare_elev` against `vvi` from the `arrest_hands`
event to the wheels, `flare_ail` against `phi` and `xtk_m`, and the sim's
own pitch at touchdown. A balloon is the cap coming down; a tail strike
is the cap coming down further.

## 2026-09-22 (second pass): EGLF 06 in the 737, the first live elevator flare

The pilot: "another hard landing?" Yes, but not the same one. The arrest
took the servos exactly as designed - `arrest_hands taken: true ok: true` at
83 ft radio - and the elevator channel flew the flare for the first time in
anger. It pulled the aircraft from -511 fpm to -102 in 2.4 seconds against a
-197 target. Then it handed the runway back, and the wheels arrived at about
-360 fpm and still worsening (`vvi_trend` -218 at 1.6 ft, the last tick
before the oleos). The debrief's own line: "about 150 feet per minute down"
- the 1 Hz sampler again catching the float, not the arrival, which is the
window `touchdown_sink_fpm` was widened for.

The tick log, radio altitude against `flare_elev`, `vvi` and the lever:

| time | radio ft | vvi | flare_elev | throttle |
|---|---|---|---|---|
| 14:50:01.1 | 21.4 | -511 | 0.35 (cap) | 0.288 |
| 14:50:02.5 | 12.8 | -358 | 0.298 | 0.288, retard begins |
| 14:50:04.3 | 13.2 | -102 | 0.185 | 0.140 |
| 14:50:05.7 | 7.0 | -228 | 0.35 (cap) | 0.022 |
| 14:50:06.4 | 1.6 | -358 | 0.35 (cap) | 0.001 |

Two laws walked into each other at thirteen feet, and each was doing what
it was written to do:

- **`FlarePitch` is a tracker, and a tracker that BEATS its target
  relaxes.** -102 against -197 is a negative error: the P term went
  negative, the D term went negative with the easing trend, and the law
  commanded the column forward from 0.35 to 0.185 - at thirteen feet, which
  is a nosewheel-first landing in the making. Nothing in the law knew the
  ground was there. Its test plant could not show this because the plant
  has no thrust in it: the arrest settles on the target and stays.
- **`FlarePower` was unlocked by that very arrest.** Its gate is
  `hold_sink_fpm` (400 for this airframe): hold the lever while sinking
  fast, retard once the sink is inside it. The elevator put the sink
  inside it, so the retard started at 12.8 ft and walked 0.288 to 0.001 at
  0.08/s across the last 3.4 s. Every foot of the retard happened DURING
  the arrest instead of before it. The gate is written to protect a fast
  sink and it fires latest exactly when it should already be finished.

Together: the thrust left while the column came back, the sink rebuilt
-102 to -358 in 2.3 s, and by the time the law wanted the pull again it was
saturated at `FLARE_PITCH_MAX` with nothing left to give. The 172 never
showed this in six months of landings because it never had 0.29 of a lever
to lose in the flare.

**In the flare the column only comes back.** `FlarePitch::step` takes a
`ratchet` flag, true in the Flare phase and false in the roundout above it,
and while it is set the commanded elevator is its own running maximum. The
roundout is still ordinary flying and stays free both ways; the cap still
bounds the attitude, so the ratchet cannot buy a tail strike, and a float it
can buy is the overflight guard's to judge - that guard already knows the
recited landing distance. `reset()` now clears the ratchet with the
integrator, so a balk starts the next approach from nothing.

The sortie test is the flare plant with the thrust leaving underneath it: a
sinking bias ramped in over 3.4 s, which is what a third of a lever at
140 kt feels like. The tracker reproduces the firm landing (-326 fpm in the
plant) and the ratchet lands it (-20). Both bounds are asserted, so the test
fails if the plant ever stops showing the bug it was written for.

FLOWN THE SAME DAY: EGLF-LFPB, runway 03, 16:35 local. The arrest took the
servos at 81 ft; in the roundout the elevator rose 0.159 to 0.197 and then
RELAXED to 0.039 as the sink eased, which is the roundout staying free both
ways; at the flare gate (36 ft) it went 0.245, 0.280, 0.323, 0.350 and held
0.350 to the wheels without ever coming back. The sink entered the flare at
-392, worsened to -616 at 16 ft as the thrust came off, and was then
arrested continuously: -501, -425, -346, -271, -222, and -217 fpm at 1.8 ft
with the trend still easing. Touchdown about -215 fpm, 4 m off the
centreline. EGLF's was -358 and worsening.

The float did not happen, and the reason is worth keeping: the flare ran
506 m against EGLF's 607 m. The un-ratcheted flare had BALLOONED - EGLF's
radio altitude went 12.2 ft to 13.7 and hung there while the law relaxed -
and a balloon spends more runway than a continuous arrest does. Touchdown
0.39 nm past the aim on LFPB 03's 2,395 m, rolling out to taxi speed in
about 705 m.

Still owed, from the same log: the retard is keyed on the
sink the arrest is removing, which is backwards - a Boeing flare puts the
levers to idle at the flare and lands on what is left, it does not spend
the last four feet getting there. The height-keyed retard is the next
change, and this one should fly first so the two are told apart. Also
unexplained and parked: the state sampler read flap 1.00 at 13 ft and 0.50
by the oleos, with no flap write in the log and nothing on the tick - the
sink was already rebuilt before it, so it did not cause this, but a flap
that moves itself in the flare is worth a look.

## 2026-09-22 (third pass): the weave at both ends of the roll, and three advisors arguing

The pilot, on the LFPB landing: "could we improve staying aligned to runway
when we land, instead of veering? is there some understeer perhaps?"

Not understeer. An oscillation, and the numbers name it. The rollout kept no
blackbox of its own, so it had to come out of the 1 Hz `state` samples,
projected onto LFPB 03's centreline from apt.dat:

| gs | heading | cross-track |
|---|---|---|
| 139 | 24.3 | -4.2 m |
| 113 | 21.8 | -1.9 |
| 99 | 23.6 | +2.4 |
| 92 | 26.4 | **+3.1** |
| 77 | 28.8 | -0.6 |
| 63 | 25.5 | -5.3 |
| 56 | 22.9 | **-5.7** |
| 36 | 21.9 | -1.7 |
| 23 | 24.3 | -1.0 |

Runway course 24.76. The nose swung 21.8 to 28.8 to 20.4 - four degrees
either side - with an eight-second period, decaying as the speed came off.
Then the takeoff from EGLF earlier the same flight, which blackboxes its
roll properly: +1.74 m at 45 kt, -3.40 at 86, -4.61 at 99, +2.39 at 127,
+5.73 at 135, and **+10.14 m at 149 kt**, which is where the wheels left the
ground. The same law, the same shape, and worse, because the takeoff is
still accelerating when the loop goes marginal.

`roll_target_heading` cut 0.4 degrees per metre of offset, clamped at 8, and
`RollSteer` closed the resulting heading error. A cascade. The outer loop's
closure time constant is `1 / (V * gain)`: at 40 kt that is 7.2 seconds and
the loop is placid; at 150 kt it is 1.9, which is the same order as the
inner loop's own lag (the nose takes a second or two to reach a commanded
cut and as long again to give it back). Marginal by construction, and the
faster the aircraft the more marginal. The takeoff roll was tuned at low
speed - the sortie-three pirouette, the tiny parked cap - and nobody had
looked at what the same number does at Vr.

**So the cut is sized for a closure TIME.** Above `XTK_GAIN_REF_KTS` (40 kt,
the pace the 0.4 was fitted at) the gain scales by ref/gs, which holds the
closure at the placid figure for every speed; below it nothing changes at
all. A 10 m offset at 150 kt now earns 1.1 degrees instead of 4. And the cut
is taken against `xtk + rate * XTK_LEAD_S` - where the offset WILL be, not
where it is. Autoland's final has had exactly that damping since 2026-07-24
("P alone converged asymptotically - the banana join - and S-ed under
wind"); the roll never got it.

The test plant is the cascade, fitted to the LFPB tick log: 18 deg/s per
unit of rudder through a 0.7 s lag, the loop's own half-second of transport
delay, the ground geometry closing the cross-track. Both laws fly it,
because a test that only shows the fix working proves nothing about the bug.
The old law weaves 11.4 m and ends 5.6 m off; the new one spans 4.8 and ends
on the line. On the takeoff seed, 2.5 m down to 1.7, ending +0.1 instead of
-0.7. Two more things fell out of reading it: the rollout was handing its
steering law a HARDCODED 0.4 s while the loop actually ticks at 300 ms plus
two round trips, so its lead term and its slew limit were both computed
against a clock that did not exist; and the rollout had no blackbox at all,
which is no way to read a control loop. It records `rollout` per tick now.

### Three advisors that were arguing with the flight plan

Same flight, nothing to do with the aircraft.

**The Climb checklist audited the automation mid-action.** It reported
"flaps, up on schedule" at 0.125 and "taxi light, off" still lit. Both were
true when it looked and false four and six seconds later: the flap schedule
finished at 15:52:19 and `lights_climb` put both lamps out at 15:52:21,
1,379 ft. The list fired at 1,200 ft on a single tick - the only boundary in
`flow::Tracker` that was not sustained, in a module whose own doc comment
says they all are. It is held `CLIMB_SECS` now, which puts the audit after
the thousand-foot actions instead of through them. The mission's laws act
and the list audits; an audit that runs during the action is a race.

**The step-climb advisory said the same thing three times in six minutes**,
twice recommending FL250 while at FL230 - the level the plan had just
brought us down from. Two faults. Its episode ended on any vertical
excursion past 300 fpm, and a 737 holding a level cruise wanders past that
(352 fpm at 16:10:51), so every wobble re-armed it: `CRUISE_WANDER_FPM` is
600 now, and `repeat_level` is keyed on the level it would ask for, so it
must have something new to say. And a level the plan handed down is not a
level to argue with: while the follower has a top of descent published the
aircraft is going DOWN shortly on the plan's own constraint, and money saved
by climbing first is money nobody can spend.

**The descent advisors took turns.** Four calls on one arrival: the
follower's "about a minute, 2 miles ahead, down to 23,000", then "12 miles
ahead" with no altitude (the destination RING, not the follower), then the
ring's "we're past the descent point" as a CAUTION, then the follower's
"7 miles ahead, down to 20,200". CLAUDE.md has said since LEJR-LXGB that the
follower's point is the only one announced while it flies - but the code
honoured that only when `planned_tod` returned `Some`. It returns `None` on
a leg with no fix or whose constraint is not below us, and the checks then
fell through to the ring, which measures to the FIELD. Between the legs that
have a descent, the ring filled the silence on a completely different
geometry. `Snapshot::follower_flying` now gates all three: while a follower
flies, the ring is not a fallback, it is silence.

NOT flown, any of it.

## 2026-09-22 (fourth pass): LFPB, the holding point for a runway we were not going to

The 737 at Le Bourget, full auto, plan to Liege. Everything worked: the
preflight passed, the A330-sized start ran, the takeoff config set flaps,
trim and autobrake, the altimeter took 30.33, the tower cleared "taxi to
holding point Alpha runway zero seven via Echo, Victor Echo, Victor three,
Charlie two, Alpha, cross runway zero three", the drive went 1,991 m of a
1,996 m route and stopped. Then:

```
15:22:38  ready for departure
15:22:44  I can't find runway zero seven next to us. Taxi us closer and ask again.
15:22:46  ... runway zero seven, wind zero two zero, six knots, cleared for takeoff
15:22:55  I can't find runway zero seven next to us. Taxi us closer and ask again.
15:23:00  I couldn't get us onto the runway. Your aircraft.
```

Blackbox: `line_up_no_fit {"why": "Some(NoneNear)"}`, then `give_up
line_up_failed`. The aircraft sat at 48.96259, 2.42169 - **169 m from
runway 07's centreline**, 46 m past its threshold, heading 266.9. The
line-up's lateral gate is 150 m (`LINE_UP_MAX_LATERAL_M`), so it refused;
the authored lead-on it would otherwise have driven ended 34 m off the 07
centreline, and `line_up_path_from_lead_on` wants 20 m, so that refused too.
Both gates were right. The aircraft was in the wrong place, and it was in
the wrong place because that is where it had been sent.

**The route had been planned to runway 09's entrance.** LFPB's 07 and 09
thresholds are 18 m apart at the same west end and diverge east (courses
67.8 and 85.2). `runway_entrance_near` takes the nearest node that sits on a
runway edge or one hop from one, and its doc comment has said since EGHI
that it makes no check the entrance belongs to the runway asked for, "a
field that breaks that assumption would need the destination runway's own
geometry passed in". Le Bourget breaks it end-on: node 201, the stub onto
runway 09, is 43 m from 07's threshold; node 134, the stub onto 07, is 67 m.
So 09's won, the active-zone trim then dropped the last three edges of the
route (taxiway A from node 198 to 201 is inside the 07/09/25/27 arrival
zone, authored), and the drive stopped at 198 - which is a real authored
hold line, just not one that can see runway 07.

`plan_to_runway` takes the destination strip now and the entrance search
keeps to its pavement: a runway node counts only when it lies inside that
strip's own rectangle (half width, floored at 15 m, plus a 15 m shoulder,
along-track within the length plus the same). Node 129 is 45 m off 07 and
drops out; 133 and 134 stay. A strip no runway node sits on, or no strip at
all, keeps the old nearest-anywhere rule - thin data degrades, never
refuses. Measured across apt.dat, the class is 5,228 of 75,812 runway ends
starting within 300 m of a different strip's end on a different course, at
1,815 fields.

Two tests: a synthetic field of two strips from a common west end where the
wrong one's entrance is provably nearer, and `live_lfpb_runway_07` over the
real install (holding point A, a 45-point lead-on ending 2.5 m off 07).
And the crew side of the same failure, `live_lfpb_line_up_finds_runway_07`,
drives the tower's own plan through `pick_line_up_plan` from the route's end:
49 authored points, runway 07, no backtrack.

**And the refusal named nothing.** `NoFit::NoneNear` carried no data, so it
fell to the `_` arm of both `no_fit_line` and full auto's give-up, which is
the vagueness the Edinburgh fix was about two days earlier - it just missed
this variant. It carries the strip it looked for and how far off it is now
("Runway zero seven is 169 metres away from us - too far for me to line up
on."), and full auto routes it through the same wording as every other gate.
Empty ident (no pavement in the data at all) keeps the old line, because
there is nothing truthful to name. A length question still answers "not
lined up on a runway, so I can't measure it": naming the strip is the
refusal's job, not the measurement's.

Two spoken lines in `no_fit_line` also carried 22 stray spaces mid-sentence,
from a `\` continuation that lost its backslash. Collapsed.

### The governor that held the aircraft for twenty-seven minutes

Same sortie, found by reading past the give-up. The taxi ended at 15:22:37
and the governor went on ticking every 0.4 s until the log ends: **3,736
ticks after "your aircraft"**, `event: brake, out: 0.5, target: 0.0`, and
`park: 0.0` in every `state` record for the whole time.

A taxi that reaches its holding point leaves the governor running on purpose
(`autotaxi`: "the governor stays, holding position on the brakes, exactly as
a spoken stop - go past this point is the pilot's call"), which is right while
a mission is still running and is why the next clearance moves the aircraft
instantly. Nothing ever turned that into a real parking brake when the
mission ENDED. So the hand-back was three things at once: a control loop
still holding the aircraft after "your aircraft", 2.5 requests a second of
Web API load the sim did not get, and a proportional brake squeeze the pilot
can neither find nor release - which the governor's own exit path spells out
("a governor that stops while squeezing hands the pilot a boot on the pedals
they cannot find"). CLAUDE.md has carried the rule since the first
orchestrator - brakes set for every wait, every abort and give-up path sets
them too - and `full_auto::give_up` was the path that did not.

`give_up` is async now and takes the sim and the airframe's valve sense; all
29 of its call sites pass them. It calls `hand_back_brakes` BEFORE it speaks,
so the line is true when it is heard, and the brake goes on through
`drive_park_brake` - the valve-aware seam, because a Baron's latching valve
is why a raw ratio write is never it. The governor's own next tick then sees
`parking_brake_ratio > 0.5`, closes the throttle, stands down and releases
its squeeze: the path the successful taxi-in has always taken at the stand
("speed governor: parking brake set, closing throttle, standing down").

Three gates, all of them the same rule that a brake is only released when
something else holds the aircraft, run backwards: only while the TAXI
flavour of the governor is the thing holding us (so nothing of the pilot's is
touched), only on the ground, and only below 25 kt - a parking brake is a
chock, not a stopping device. A groundspeed that will not read is no
measurement and no stop. `hand_back_brakes_if` takes the flavour as an
argument so all five cases are a plain `cargo test` against `FakeXPlane`
rather than a process-wide static, and the write is blackboxed
`hand_back_brakes` with whether the brake agreed.

Still owed: the taxi's arrival line says "holding position, brakes set" while
the park brake is 0.0 and only the governor holds. True enough of the wheels,
and deliberate, but it is not what the word means.

NOT flown.

## 2026-09-22 (fifth pass): EBLG, a vacate onto the other runway

LFPB-EBLG in the 737, full auto, the arrival. Three faults, one sortie, and
the first two are the same sortie's taxi.

### The vacate turned away from the apron

Landed 04R, rolling handoff at 7.8 kt, `taxi_in vacating {exit: "C3"}`, and
665 m later the drive stopped at taxiway node 51: **148 m from 04R's
centreline on the side away from the terminal**, 62 m from runway 04L's
centreline, on the link that joins the two parallels. Liege's stands are
almost all on the other side (76 of 92 of them; stand 35, the one we were
given, at +283 m). So the taxi-in clearance came back "taxi to stand three
five via Charlie three, Sierra three, **cross runway zero four right**" - back
the way we had just come - and the drive was handed a route whose first leg
ran 179 degrees from the nose. A 737 does not turn round on a taxiway.

`plan_vacate` picked C3 because its junction with 04R is 513 m down the
rollout and S3's is 549. Distance was the only test it applied. S3, 35 m
further on, leads straight to the apron, and so do the two other exits
ahead. And the walk out along C3 ran onto edges carrying `departure 04L,22R`
and `arrival 04L,22R`: the vacate cleared the runway it landed on by parking
inside the parallel's active zone, which is what `zone_rank` already ranked
worst and then used anyway.

Two rules now. The walk and the turn-off picker REFUSE a rank-2 edge outright
- vacating one runway is covered by the landing clearance we hold, entering
another is covered by nothing. And the exit picker prefers a turn-off on the
same side of the rollout as the parking (`nearest_stand`, which needs no
strip geometry passed in: the rollout heading IS the runway direction). A
field with parking on both sides picks the near apron, which is the answer
anyway; a field with no stands in the data keeps the distance-only rule. At
EBLG the vacate now takes S3 and ends +150 m, on the stands' side, and the
taxi in crosses nothing.

### A projection that jumped 142 m, and the aircraft stopped for it

What the pilot actually saw was `auto taxi: off route, stopping
lateral=155.06` fourteen seconds into the taxi in, twice. The tick record
says the aircraft was **13.0 m** off the route at the previous sample, 0.2 s
earlier, at 4.6 kt. It had not moved 142 m; the projection had.

`project_signed_facing` filters the route to segments within 90 degrees of
the nose - the Malaga pushback fix - and falls back to the plain nearest only
when NOTHING survives the filter. At the engage nothing did (the route ran
back the way we came), so the fallback gave the honest answer: 0.9 m off, 5 m
along, `remaining` 338.8 of 343.8. It stayed frozen there for fourteen
seconds while the drive tried to turn around. Then the nose swung far enough
for one far segment - the leg back across 04R - to enter the window, and
being the only survivor it became the entire projection: offset 13 m to 155,
progress 5 m to 144.6, in one tick. Both attempts logged
`progress_m: 144.55716959254474`, identical to fourteen digits, which is what
a segment endpoint looks like and not what an aircraft's position looks like.

The facing pick now has to be paint we could actually be joining: within
`autotaxi::MAX_JOIN_M` of us, or nearer than the plain answer. Further off
than the auto-taxi's own engage limit is a refusal there, so it cannot be the
thing we steer at here. A jump in a projection is never a movement of the
aircraft.

### "Cabin is secure for departure", taxiing to the gate

Thirteen seconds after the rollout handoff the cabin armed its DEPARTURE
secure, and reported it a minute later while the aircraft taxied in.

The monitor's cabin block sits inside `if !quiet`, and `quiet` includes
`autoland::ACTIVE`. So on every autoland arrival the cabin is not called at
all from the arm until the rollout hands off. Its state machine then gets one
sample at 26 kt on the ground, under the 30 kt `LANDING_ROLL_KTS` test, reads
that as the sim REPOSITIONING the aircraft onto a ramp rather than as the
landing it had just flown, calls `end_flight`, and the vacate taxi at 12 kt
is then a departure taxi by every test it has. The touchdown it never saw was
at 133.9 kt.

`quiet` mutes the WORLD's narration while the flight deck is busy - vectors,
the party line, traffic advisories - and an autoland is exactly when it
should. The cabin is not a commentator, it is a state machine that has to see
the flight, and it owns its own sterile-cockpit rule already. It has its own
gate now (`cabin_quiet`: a recovery, the panel's mute, a wreck - not a
landing). And the transition test no longer depends on catching the right
tick: a flight that came down through the descent gate has LANDED, whatever
speed the sample we happen to get shows.

### And the tower now knows what "runway vacated" means

The crew makes that call from the taxiway on every arrival, before the taxi
request, and the FSM had no intent for it: `Unknown`, "say again", twice in
this one arrival. It reads as a controller who did not hear the one call they
most want to hear.

`Intent::RunwayVacated`, and it is deliberately the cheapest kind of intent
there is: acknowledged ("roger, thank you") and it changes NOTHING. The state
stands at `ClearedToLand` on purpose, because a bare "request taxi" after a
landing clearance is inferred as the taxi IN, and moving to `Parked` here
would turn the next one into a departure clearance to the holding point. A
call that also ASKS is that request instead - the station's own suggested
wording is "runway vacated, request taxi to parking", and the parking rule
owns it. "Cleared to land" read back is not a vacate report, which the parse
tests pin because the two calls are seconds apart on the tape.

On departure or approach frequency it is runway-shaped, so it earns "contact
Tower", like every other runway call. And it is radio speech for unkeyed
content routing for the same reason a pattern call is - nobody tells their
copilot they have left the runway - which is safe here precisely because the
intent cannot buy anything. `vacated`/`vacating`, never the imperative
`vacate`, so the copilot keeps "vacate the runway".

Six new corpus cases. Still owed: reporting vacated is also how a real tower
learns its runway is free, and ours still learns that from the taxi request.

NOT flown.

## 2026-09-23: the S-76 flies itself, pad to pad

Two sessions, the helicopter from a hover that crept to a flight plan flown
end to end. The rules are in CLAUDE.md's helicopter section; this is the
evidence.

### The hover mode is a groundspeed hold, and it captures

`sim/autopilot/hover` writes the velocity of the moment into
`hover_vector`: after a drift hand-back the vector read [-4.9, -4.7] m/s,
the drift it had been told to hold by a recapture at 6.8 m/s. Asked for
zero it still ran forward at 4 to 8 kt (nose +1.5 against the good hover's
+6.2), so the loop owns the vector: command plus `DriftTrim`, an integral
on the measured `local_vx/vz` error. The first hands-off hover after it held
10 ft and 30 ft to 0.1 to 0.4 m/s. The yaw swings of the first sessions were
the pilot's twist axis, not the aircraft.

### A moving target is the whole low-speed envelope

The same mode followed a moving vector: 5 and 10 kt air taxis to 0.1-0.2 m/s,
a departure to 30 kt and 300 ft to 0.7, a turn to 0.4. The nose does not
follow the track (a turn flew 45 degrees sideways, a leg backwards at -28 kt
indicated), and the pedal law that fixed it was the second: the S-76's pedal
is a yaw-rate command (0.1 = 4.7 deg/s, neutral holds to 0.01 deg, measured
in a hover), and a D term on `R` at 2.4 of loop gain through a quarter-second
tick rang the tail until the fault latch stood it down. Two circuits landed
1.9 and 6.4 m from their spot; a third on the final build 4.6 m. The mode
lets go near 47 kt and will not re-engage that fast.

### Two AP-off hand-backs and a plunge

A speed probe above the hover mode's range, then a takeover from a hard
deceleration (collective 0.30, nose 12 up), each ended in a hand-back that
turned the autopilot off with nobody flying: one caught by hand at 54 kt,
one at 1,600 fpm into a field and a ground spin. The EDDV-ETHC route crashed
through a chain: CEL lies past the pad so the slow-down came late, the
arrival takeover aimed its captured speed at the pad, the hand-back put
altitude hold on a 35 kt aircraft that drifted backwards, a "hold position"
wound the trim to 0.9, the attitude hand-back parked the lever there, and
the pilot's "land the aircraft" integrated it to 0.0: 3,100 fpm. The fixes:
never AP-off in the air, a trim bounded to the power curve (a test flies the
crash's own climb: 1,317 fpm of sink without the band, 401 with), a takeover
aimed along the velocity it captured, the slow-down on the distance still to
fly, terminal fixes dropped.

### The route, and parking like an aeroplane

The cruise modes work: hand-over at 36-40 kt, heading and altitude hold, a
collective speed law at 120.0 kt. EDDV-ETNW flew end to end, 7.3 m from the
landing point. Parking was first built as a helicopter's own direct hover
taxi, then as its own tower request, and the pilot's objection was right
both times ("helicopter should be like another aircraft"): it is now the
aeroplanes' `taxi_to_parking` and taxi-in, and only the drive differs. The
tower called the hover taxi an uncleared takeoff; under 100 ft a helicopter
is on the ground for it now.

### What the aircraft taught besides

Engine 2's throttle sat at 0.61 after "throttles to fly" (a heavy lift
drooped the rotor to 83 % at 0.9 collective): the start reads every throttle
back, and a rotor guard caps the collective. An unlocked tail wheel let the
torque spin it on the ground at up to 20 deg/s with the brake set and the
pedals neutral, until the engines were cut; the engage command does nothing
on the S-76, the toggle does, read first. `groundspeed` carries the vertical
speed. Whisper's "hover" arrives as "how about", "over at", "prove at",
"hova"; "execute the flight plan" read the plan's brief instead of flying
it. Restarting the app mid hover taxi forced a manual landing: restarts wait
for the ground.

## 2026-09-23: the copilot as a person, and the SF50's turns and gear

Asked "how do we make the copilot more human", the answer was not the words
(`vary_ack` already varied them) but timing, memory and judgement: a reply
that starts a beat after the pilot stops talking, "stand by" instead of three
silent seconds, a warning said quicker and flatter, a word about the landing
from the instructor's own sink scorer, offers of the next flap and the gear
as the speed allows, the arrival brief thirty miles out, small talk from a
closed set in the voice's own idiom, and anything that can wait held until
the flight deck is quiet (`tempo`, `persona`, `rapport`). The monitor's
spoken "Landing summary: last airborne sample about N feet per minute down;
exact touchdown rate unavailable" was the clearest machine tell left; it is
blackboxed now and the rapport says the landing instead.

The same evening the SF50 at ETND showed two older faults. Its gear came up
at 200 ft and 122 kt with the takeoff flap at 0.5, inside the `.acf`'s own
gear-warning envelope (below 125 kt, flaps past 0.5), and the horn ran to
335 ft on every departure: the gear now waits for its warning to be silent.
And its circuit went round the long way on 13 of 30 turns: the rectangle
circuit slewed the bug 90 degrees off the nose from wings level without the
`bug_toward` door, the autopilot turned 270 right for a 90 left at crosswind,
downwind and base, the base ended 1.9 km short of plan, the final was handed
to autoland 170 m off at 357 ft, and it balked `off_centreline_low` at
0.1 nm. The bug now leads the nose by 25 degrees until the bank is
established the right way.

The second approach flew the reposition well and crossed the numbers on the
centreline, then the landing came apart: the arrest pulled the sink to -50
fpm at 41 ft (target -250), the roundout pushed, the flare at idle ballooned
to +158 fpm at 32 ft, and the overflight gate, allowing only 344 m of float on
a 1,244 m runway because of the Baron's 900 m floor, went around with 900 m
ahead. The go-around could not climb (80 kt against an 87 kt approach speed,
idle engines) and the aircraft met the runway at about -740 fpm, then rolled
out. A low flare with room to stop on the airframe's own landing distance is
now committed; the flare law's balloon is still owed.

The next landing crashed, at EDLP 24 (2026-09-24). Arrest at 49 ft, the
roundout pushed to -0.10 and the sink rebuilt; the flare took 0.3 of elevator
at 24 ft and the ratchet held it while the sink vanished at +400 fpm a second,
releasing only once the aircraft was climbing; it ballooned to 52 ft at +850
fpm, above the 50 ft band, and balked `balloon`. The go-around went from idle
at 81 kt and slowing; the vertical law, which climbs only with speed over the
approach figure, gave it 0 fpm, and the SF50 stalled onto the runway at
-1,568 fpm. Low balloons with room to stop are now landed (100 ft), on
approach power, and the ratchet lets go half a second ahead of a balloon.

## 2026-09-24: the tower cleared an SF50 onto the grass

ETND at 260/6: the field has 08/26, 1,240 m of asphalt, and 08L/26R, 446 m
of grass beside it. The tower's wind pick weighed only headwind, the grass
strip faced the wind a hair better, and the SF50 was taxied to holding point
Alpha "runway two six right", and
the line-up found no 26R near the hold it reached ("08L is 152 metres away") and full
auto gave up. `nav::best_runway_for_wind` now picks only from the field's
main runways (`assignable_runways`: hard surface where the field has one,
never under half the longest of that kind); a grass field keeps its grass.
Still owed: the crew's line-up refusal named the reciprocal end, not the
cleared one, and the crew never judged the cleared runway's LENGTH against
the airframe before taxiing (a 446 m clearance should be refused and a
longer runway requested at the taxi call).

## 2026-09-24: ETND-EDLP in the SF50, the flare, the vacate and the stand

The first full flight after the runway fix went ETND 26 to EDLP 24 with no
go-around: the balloon commit held (the flare climbed to 24 ft at +190 and
landed instead of balking). But the arrival was -421 fpm "firm" and the
elevator porpoised between its limits: 0.18 at 49 ft took -572 fpm to +36
in two seconds against a -250 target and the damping then pushed; 0.35 at 22
ft ballooned it; the release dropped it to neutral in one tick (neutral is
the trimmed approach attitude) and the sink was -590 at the wheels while the
last full pull did nothing at 70 kt against a 67 kt Vso. The SF50's engine is
high on the tail, so the lever is a pitch input: cut to idle at 22 ft, back
to approach thrust in the balloon, cut again. A plant fitted to the tick log
did not reproduce it (RMS 170-195 fpm; thrust pitch and spool lag are not in
it), so the numbers were chosen from the trace, not tuned against a model:
the flare's elevator law is now per airframe (`flare_elevator:`,
`autoland::FlareGains`), the SF50 at half gain, three times the integrator,
0.30 cap, a balloon easing the pull at 0.15/s instead of letting go; and a
progressive `flare_power` instead of the step to idle.

Taxi in: the vacate refused B (108 degrees off the rollout, 400 m ahead) at
the 100 degree limit set against EGLL's 169 degree N4W, took C 970 m further
down, drove 1,217 m at turn pace, and timed out on the runway at the flat
120 s with 714 m to go; the stand route then started from the runway with a
backtrack the tower phrased as "cross runway zero six". The limit is 120
degrees and the budget scales with the route (`vacate_budget_secs`). The
drive reached 7C 2 m from its spot but 147 degrees across it (the lead-in ran
straight from the last taxiway node), so securing said "stopped short of the
stand"; the lead-in now enters along the stand's parking line through a
point 25 m back (`stand_lead_in`). And a mission (full auto, the helicopter
route) now ends shut down: at the stand the registry's own shutdown for the
airframe's class runs, every engine is confirmed stopped from `ENGN_running`,
then every electrical system goes off (`taxi_in::shut_down_at_stand`); a
spoken "taxi to parking" still leaves the shutdown to the pilot.
Still owed: a stand route planned from ON a runway snaps to the nearest node
regardless of heading (the backtrack), and a backtrack along the strip we
occupy is phrased as a crossing.

## 2026-09-24: EDLP-LOWI, a hypoxic climb in the SF50

The takeoff roll drifted off the centreline and the pilot took over; "resume
flight plan" was then refused because the plan's FL290 was over the profile's
28,000 ft ceiling (the Vision Jet's is FL310; the arm had never checked it,
the resume did). After a restart the resume climbed on, and at FL253 the
screen went black: the cabin was at 24,000 ft. Full auto had started with the
engine already running from the previous flight, so the bleed, which only
ever came on as part of an engine start, was never switched; two "cabin
altitude" callouts went by unanswered; the oxygen button read on and the
stock valve behind it was shut. And the follower's climb protection
(`climb_vs_fpm`, 60 fpm a knot off its cap, never under 150) still asked
about 1,000 fpm at 104 kt and FL260 against a 120 kt Vy. Rescued live over
the Web API (bleed on, valve opened, felt altitude 24,400 to 15,000 ft; a
typed "descend to one eight thousand"). Now: `cabin_air:` in the profile and
[src/tools/cabin_air.rs](../src/tools/cabin_air.rs) switch bleed and oxygen on
for every mission and at the cabin callout, the warning band hands a flying
follower 10,000 ft, and the follower's climb also fades to nothing at the
airframe's speed floor (`circuit::climb_limit_fpm`, the circuit's own rule).
Still owed: the takeoff roll's centreline on the SF50, and the arm checking
the plan's cruise against the ceiling.

## 2026-09-24: the EDLP-LOWI debrief, fixed

- **The arrival picked approaches with no runway.** LOWI's RNAV 08 V and 26 E
  are circling procedures (no runway fix), 08 Z turns onto the runway on RF
  arcs, and the loaded legs carried the missed approach, so full auto would
  have flown the missed approach into the valley as its approach; the
  alignment pick chose the grass "26R", which a "26" approach matched.
  `nav::straight_in_legs` now requires a runway fix for the exact runway and
  no RF leg before it, and cuts the legs at the threshold; `best_approach`
  only offers such approaches; the arrival picks hard runways and, when the
  wind's end has nothing flyable, another end with one inside 10 kt of
  tailwind (`approach_fallback_ends`) before a visual landing. LOWI now plans
  RNAV 26 Z at every wind: WI002 9,500, WI103, RW26.
- **The tower's runway is flown when we can** (`tower_arrival_runway`): the
  inbound reply's "join final runway two six" came two seconds after the crew
  armed 08; now the arrival is rebuilt on the tower's end if it has a
  flyable approach, and otherwise the crew says it is staying with its own.
- **The SF50's takeoff roll had no steering**: castoring nosewheel, no air
  over the rudder, the heading error grew 8 to 27 degrees at full deflection
  until the pilot took the pedals at 21 kt. The roll now brake-steers like the
  taxi on a `brake_steer` airframe, fading out 20 to 40 kt
  (`autotakeoff::roll_brake_split`); and the transponder goes to ALT as the
  roll starts, not at liftoff (a handed-back roll left it on standby).
- **The speed hold sat at idle, silent**, 234 kt against 180 descending into
  the valley; it had already closed the lever the pilot then pulled.
  `IdleStillFast` says once, after 15 s, how fast and to shallow the descent.
- **The altimeter flipped QNH/standard/QNH at FL180**: a resume never
  published the flight's transition (Germany's 5,000 ran as the profile's US
  18,000), and the knob change itself moves the indicated altitude ~260 ft
  across the line. The resume publishes it now, and within 500 ft of the
  transition either setting stands.
- **A cruise above the ceiling is refused at the arm** (`preflight`).
- **Stands**: the lead-in's back point is only taken when the turn onto the
  stand line is 90 degrees or less (LOWI circled on a reversal), and parked
  for securing is within 35 m whatever the heading.

## 2026-09-24: LOWI-LSZS, making the copilot able to fly into the mountains

The SimBrief plan LOWI 26 MOGT3H to LSZS 03 exposed three gaps. The follower
flew SID fixes as plain waypoints at cruise speed: MOGT3H's 7,850 ft at
WI506 (8.3 nm from the departure end, 714 ft/nm) would have been crossed
about 2,200 ft low at the 375 ft/nm the SF50 made climbing at 190 kt. In calm
air the tower could have picked 08, which the SID does not serve. And LSZS's
only approaches end at a missed approach point two miles short, so the
arrival fell back to a visual autoland into the Engadin valley (sectors
15,500/13,500 ft). Now: SID minimums from the CIFP (plus a gradient gate at
the arm and a crossing watch that warns and hands over); the SID's runway
asked for at the taxi request; `earth_msa.dat` defines a mountain field, which
is an approach or a hand-over; MAP approaches flown from their transition
(PELAD 16,000, ZS700 13,000, ZS716, ZS701 12,400, ZS702 11,800) to the MAP on
the published 4.49 degree path, and landed from the MAP at 4.4 degrees to the
aim point by an autoland whose glidepath is now per approach; a go-around
there flies the published missed approach (ZS706, RONAG 14,000). Also fixed
on the way: CIFP turn direction read from field 9, not 12. Unflown: the
SF50's best climb (2,000 fpm, profile VERIFY) behind the gradient gate, and a
4.4 degree SF50 flare.

## 2026-09-24: the SF50's takeoff swing was our own rudder

LOWI 26, both SF50 takeoffs of the day: the heading error built to the right
as the power came up, 18 degrees with the steering pinned at -0.55 and the
aircraft 60 m right of the centreline by 60 kt, on a 45 m runway. The roll
law's power lead (`power_lead`, up to 0.25 of right rudder with the
throttle) exists for a propeller's left-turning tendency; the SF50 has no
propeller, so the copilot was flying the swing itself. A jet (profile
`reference.kind` containing "jet") now takes no power lead
(`RollSteer::for_airframe`). The brake steering added earlier the same day
did its job below 20 kt (-13 degrees to 0 by 10 kt).

## 2026-09-24: the first LOWI-LSZS departure, handed back at 3,000 ft

The new minimum watch fired one second after the takeoff handed over at
1,000 ft: its climb-rate estimate started at ZERO, so it predicted arriving at
WI505 (4,000, 1.3 nm) at the present 2,990 ft and handed the aircraft back
("Unable 4,000 at WI505 ... Your aircraft"), full auto gave up, and the pilot
was left in the valley with the altitude dial at 3,000. The resume refusal
then advised "steer 343 for WI506" from below the ridges, and the next
samples show 971 ft above the ground in 28 degrees of bank. Also: the
takeoff's climb-out was capped at the enroute 1,500 fpm, 750 ft a mile
against WI505's 816. Now: the rate is seeded from the first real sample and
judged only after ten; a predicted miss while climbing is SAID and flown on at
best rate (only an aircraft not climbing is handed back); the takeoff climbs
at the best rate holding its initial climb speed while SID minimums are ahead
(`autotakeoff::DEPARTURE_MINIMUMS`); and a rejoin refusal below either field's
sector altitude says "climb first" before any heading.

## 2026-09-24: the backtrack turn at LOWI 26

Entering 26 at an intersection, the line-up backtracked 460 m cleanly and
then turned 180 on the runway badly: pointing backwards the heading error
flipped between +177 and -179 on noise and the steering followed it (right,
left, full left), and the castoring SF50 got 0.06 of inside brake at full
lock because the taxi's brake split fades out below 7 kt. It swung 19 m off
the centreline of a 45 m runway. Now a reversal keeps its first direction past
150 degrees until back under 90 (`committed_err`), and a join turned through
more than 60 degrees brakes like a pivot: inside brake from 1 kt, up to 0.45
(`pivot_brake_split`). Neither behaviour was changed by the day's other work.
Also the same afternoon: the talk-down takes a per-path slope and arms at a
MAP hand-over; the steep MAP autoland is off (`MAP_AUTOLAND`).

## 2026-09-25: LIPE-LIPK in the 172, the approach stood down 10 nm out

Full auto flew LUMA1U to LUMAV and the plan ended there, 51 nm from ESFOK,
the ILS 30 IF (a correct direct-to, 30 minutes at 95 kt). Two things broke
it. At the start of that leg the heading bug alternated tick by tick between
the nose and 25 degrees right of it for 50 s: the aircraft was banked 2 to 5
degrees LEFT for a right turn, `bug_toward` put the bug on the nose past 3
degrees and the entry lead back on under 3, and the 172 turned 35 degrees the
wrong way (why the first left bank came from a lead set wings-level is not
found; the same lead worked at FI30). That was two `leg_stalled`. Thirty
minutes later the 141 degree turn onto FI30 was judged "not closing" 45 s in,
still turning: the third stall, counted across legs, stood the approach down
at 3,000 ft 5 nm from the FAF. Resume then refused below the 7,500 ft MSA,
and "land the aircraft" self-picked Cervia 11 over Forli 30 at the same
6.4 nm with LIPK on the shelf. The pilot hand-flew it into Forli.

Fixed, offline: the stall count is per leg and a leg's first turn is given
`leg_turn_grace_s` (the turn at 2 degrees a second) before any verdict
(`Closure::with_grace`); a bug on the nose for a big turn holds until the
wings are level (1 degree), not 3; the self-pick takes 8 nm off the plan's
destination (`DESTINATION_BONUS_NM`, blackbox `self_pick.destination`); a
ground advisor row owing a switch ON waits for a running engine (the taxi
light was lit before the start and after the shutdown); "Shut down." passes
the domain gate; "switch off the aircraft" and its shapes are all electrics
off.

## 2026-09-25: "you have control" was the upset recovery

Asked whether the copilot could take the aircraft by voice, the grammar
answered for itself: "take the controls", "you have control", "you have the
aircraft" and "take over" all armed `recover`, the upset recovery (joystick
overrides, unstall, roll upright, altitude hold), with no ground check, so a
hand-over on the taxiway would have flown a recovery. "I'll taxi", "let me
fly", "I'll take it", "you fly" and "your aircraft" parsed to nothing, and
"take over" did not pass the domain gate at all. Now the hand-over is
`handover::decide` over the flight model's status, run as the ordinary calls
the phase needs; the recovery is kept for an actual upset and the distress
words, and the panel's RECOVER button injects "recover the aircraft". The
pilot's phrases release like "my controls". Offline only.


# 2026-09-26 trim: CLAUDE.md before its second diet

CLAUDE.md had regrown to 1,637 lines and 112 KB since the 2026-09-05 trim,
most of it the sortie that taught each rule: airport, date, numbers. It was
cut to the standing rules, their one-line why and a pointer here. Nothing was
deleted: each section below is that section of CLAUDE.md as it stood at
commit 18627ec, verbatim, so every airport, date, number and identifier the
old file named can still be grepped. The headings match CLAUDE.md's, so a
rule's evidence is under the heading of the section that keeps the rule.

## 2026-09-26 trim: preamble

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

## 2026-09-26 trim: Wire contracts (never rename)

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

## 2026-09-26 trim: Neighbours

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

## 2026-09-26 trim: Stack (decided)

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
its test. `xvl analyze --now --since 2026-09-20` replays every miss through
TODAY's STT repairs and grammar (the 172's registry) and lists only what is
still missed, questions apart; the waiver, flight-plan load and ATC branches
live outside the grammar, so a few of its "misses" are handled live. The one model-shaped future use is an instructor answering handbook
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

## 2026-09-26 trim: Core design rule: curated tool surface, not raw dataref access

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

## 2026-09-26 trim: Aircraft profiles drive the tool surface

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
- **A flight speed hold on the runway has landed** (`speedhold`
  `TOUCHDOWN_HOLD_S`, blackbox `touchdown_idle`): a second on the wheels and
  it closes the throttle, says so and lets go. An 80 kt hold asked for in the
  flare powered LIPE's rollout from 64 to 75 kt (2026-09-25).
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

## 2026-09-26 trim: SimBrief

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
the enroute caution and the runway at the hold. **A MARGIN is the pilot's to
waive, the airframe never** (`full_auto::Gate`, 2026-09-25): a refusal
offers "say fly it anyway" only for the runway's 1.4 factor
(`OriginRunway`; waived, the unfactored roll and the 500 m floor still
hold, `BookRoll`) and the plan's operational weight limit (`PlanWeight`;
the profile's max gross still holds). A closed phrase
(`grammar::waiver_request`) inside 120 s waives THAT gate for THAT plan and
re-arms; the ack names what was given up, the runway waiver lives only as
long as the mission (`aircraft::TAKEOFF_MARGIN_WAIVED`, set by a successful
`start`, cleared by every stop), blackbox `waiver`. Fuel, ceiling and the
destination runway (already unfactored) are never waivable. **A spoken ask for the
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

## 2026-09-26 trim: ATC: the world, not the crew

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
- **A station hears only its own field on the ground**
  (`station::note_aircraft`, `GROUND_REACH_NM` 10 nm, every monitor tick):
  beyond it the station is silent (`tuned_service`, the ground ticks) and
  retires what it issued, and `party_line::station_reaches()` is false, so
  full auto departs as from an UNCONTROLLED field: no clearance call, its
  own taxi planned from the field's network (`party_line::plan_self_taxi`,
  the plan's runway unless 5 kt of tailwind, else the wind's), published
  like a tower's route so the push, drive and line-up are unchanged
  (LILB 2026-09-25: Gibraltar cleared a 172 parked in Italy and full auto
  waited three minutes for a route that could be ours). Airborne the last
  controller still keeps the flight. The via names are not spoken (an
  untowered field's apt.dat names are an author's typing).
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

## 2026-09-26 trim: Ground & flight automation

Orchestrators compose proven loops; no new flight laws in orchestrators. All
laws battle-test as plain `cargo test`; everything blackboxes to
`logs/blackbox.jsonl` (kinds: `atc`, `autotaxi`, `autotakeoff`, `pushback`, `checklist`,
`taxi_guide`, `cockpit_auto`, `full_auto`, `circuit`, `procedure`, `cabin`,
`instructor`, `loading`, `turbine_start`, `heli_start`, `hover`, `heli_circuit`,
`heli_route`, `rapport`, `cabin_air`, `api_load`, `plan_audit`, `handover`, `start_item`,
`sim_radio`). Cross-cutting rules, each learned the hard way (details in
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
  governor through `hold_speed`'s front door; pure-pursuit steering whose aim
  is capped in a gentle bend so the L^2/2R cut stays under 1.5 m
  (`bend_lookahead_cap_m`, never under the 16 m minimum; the Evolution's
  55 m aim ran 8 to 13 m inside LIME's bends, 2026-09-25); an
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
  `AltimeterSync::line` says "did not take" with both numbers; LIBP
  2026-09-26: QNH 1024, the 737's dial read 30.06 after the write, 30.23
  after eighteen presses with the eighteenth still in the sim's queue,
  30.25 after the "correcting" nineteenth, and a knob one step PAST the ask
  looked like no progress, so the crew said "set it by hand" over a
  hundredth: the post-round settle is 300 ms and a dial one step past the
  ask that moved this round earns the one press back), so the crew
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
- **A departure is flown to its chart** (LOWI MOGT3H, 2026-09-24): the
  planned SID's CIFP minimums (`nav::sid_minimums`, SimBrief `sid_ident` /
  `plan_rwy`) ride the plan as `climb_minimums`; the arm judges each one's
  gradient from the departure end against Vy and the best climb
  (`departure_gradients`, refused past 90 %, said past 75 %); the follower
  climbs at Vy and the best rate while one is unmet, predicts every crossing,
  says a short one and hands over when one will not be made
  (`watch_minimum`). The taxi request names the SID's runway (the tower
  honours it unless the wind really favours another end), and a different
  cleared runway is said and handed back on the ramp. **An arrival is flown
  to its chart too** (`nav::star_constraints`, SimBrief `star_ident` /
  `destination/plan_rwy`, 2026-09-24, unflown): the STAR's windows are
  clamped into the plan's altitudes at the arm (`apply_arrival_constraints`,
  never climbed back out of), every fix's target is capped so each "at or
  below" further on stays reachable at the descent rate
  (`arrival_ceiling_ft`, blackbox `star_ceiling`; the plan's own later
  altitudes are ceilings the same way, `profile_ceiling_ft`, since LIME-LIPE
  2026-09-25 paced each leg to its own fix and arrived 14,200 ft at 11 nm),
  and each speed limit is
  slowed for in time and held after (`arrival_speed_cap`, `star_speed`).
  **The crew picks its own procedures** (LIME 2026-09-25: the tower gave
  10 to a plan built on LEGL1P from 28, and full auto gave up): a cleared
  runway the plan's SID does not serve gets the SID from THAT runway that
  joins the route furthest along (`nav::sids_for_route`,
  `full_auto::choose_departure`, gradient-checked like the planned one; none
  flyable: direct to the PLANNED SID's exit, the one the route starts on).
  At the arm, a destination METAR wind that favours another end than
  SimBrief's gets the STAR serving it that enters the route earliest,
  straight legs before arcs and vectors (`nav::stars_for_route`,
  `plan_audit::choose_arrival`, blackbox `star_chosen`), the planned STAR
  kept when it serves the runway. Either choice reloads the GPS with the
  procedure flown (`full_auto::payload_with_legs`). Still owed: a re-pick
  in flight (the METAR moves, the tower names a runway).
- **The whole plan is dry-run before it is flown**
  ([src/plan_audit.rs](src/plan_audit.rs), at the full-auto arm and on
  "check the flight plan" / "is the plan safe" / "is the aircraft suitable",
  asked BEFORE the systems check): the follower's own vertical rules flown
  over the route at the airframe's rates. REFUSED: a plan for another type,
  a cruise above the ceiling, block fuel over the tanks, a distance past the
  airframe's own burn plus 45 min, a runway short at either end, a STAR
  minimum over the ceiling, a climb that cannot clear PROBED ground near the
  departure at best rate (the only scenery loaded before takeoff). CAUTIONS
  (said, flown): a leg under the grid MORA (`earth_mora.dat`, row = the
  cell's south edge) outside 25 nm of either field, a STAR crossing predicted
  high or low, a STAR speed under the clean floor, a trip burn under 70 % of
  ours, an overweight landing, the destination's crosswind now, a cruise
  never reached, unpressurised above 12,500. The all-clear names what was
  checked AND what could not be. Unflown (2026-09-24).
- **A mountain field is an approach we can fly or a hand-over, never a
  visual autoland** (`mountain_field`: MSA from `earth_msa.dat` 3,000 ft or
  more above the field). An approach that ends at a MAP short of the runway
  (LSZS RNAV 03) is flown from its transition to the MAP on its published
  path (`map_path`, rate fed forward), configured at landing speed, and at
  the MAP HANDED TO THE PILOT with the talk-down armed on the segment
  actually flown (`map_handover_path`, from there to 50 ft over the
  threshold; `nav::visual_segment` judges it flyable: within 3 nm, 0.4 nm of
  the centreline, 30 degrees, 5 degrees). The steep autoland on that angle
  (`gp_ft_per_nm`, 3 degrees exactly otherwise; its go-around flies the
  PUBLISHED missed approach, `take_missed_legs`) is off behind
  `MAP_AUTOLAND` since LSZS 2026-09-24: it porpoised in 18 kt of mountain
  wind and levelled at 238 ft for a reason not yet found.
- **The ILS is tuned, watched and, per airframe, flown**
  ([src/ils.rs](src/ils.rs), 2026-09-25: full auto "flew the ILS" at LIPE
  with NAV1 never tuned): `ils::tune` sets NAV1 and the course (read back,
  blackbox `ils` `tuned`) at the approach setup and at every autoland arm;
  `XCheck` says a needle held past a dot for 8 s on final (the copilot's path
  and the ILS disagree); `Coupling`, only with the profile's `ils_coupled`
  (off everywhere until a capture is flown on the type), arms APPR on final
  inside 15 nm and 2 dots, stops writing the bug at localizer capture and
  the VS dial at glideslope capture (and never re-engages HDG/VS over them),
  hands back to HDG/VS on a go-around or a lost capture, and leaves the
  roundout gate's arrest to take the servos as always. **A MAP on the
  runway fix is a landing** (`map_short_of_runway`): LIPE I12 codes its MAP
  on RW12, and a high arrival that never captured the FAF was handed over as
  a "visual segment" 0.8 nm out.
- **The copilot knows the aircraft's systems before it reaches for one**
  ([src/systems.rs](src/systems.rs), 2026-09-25): at detection the `.acf`
  (glass HSI, TCAS, auto pressurisation, APU bleed) and the live
  `preconfigured_ap_type` (which modes the autopilot HAS: a single-axis
  KAP 140 flies no glideslope) are surveyed, blackboxed `systems`, reset per
  swap, and answered for "what autopilot have we got". A survey is a fact to
  consult, never a licence: a facility is still read back when used.
  **"Set up the ILS" is a pilot monitoring's setup**
  ([src/tools/approach.rs](src/tools/approach.rs), [src/ils.rs](src/ils.rs)
  `SetupWatch`): the runway spoken, else the tower's ARRIVAL clearance, else
  the plan's (found at the destination, so it can be set up in the cruise);
  tuned through `ils::tune`; the course source to NAV1; the chart's decision
  altitude said (threshold + 200) and the radio altimeter bug set on
  transport avionics; APPR armed only from OFF (a toggle: the old tool
  disarmed it on a second ask), only with the servos ALREADY in, only inside
  the localiser's coverage on an intercept (`arm_verdict`), read back within
  6 s and never retried; the needles called alive and the modes captured;
  the missed approach altitude set at glideslope capture. A pilot hand
  flying keeps the aircraft ("couple the approach" is the ask for the
  autopilot); an automation flying gets a tune for the cross-check only.
  A localiser row's bearing is `magnetic x 360 + true` (`nav::loc_courses`;
  the old divide read EGPH 24 as 239.66).
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
  leg (EGLF-LFPB 2026-09-22). **Nor over a level the PILOT holds**
  (`PlannedTod.held_ft`, `procedure::held_tod`): a spoken altitude is sticky
  and outranks the plan, so its descents are the plan's, reminded ONCE with
  who holds the level, never "past the descent point" (LIME-LIPE
  2026-09-25: eight descent calls over a 19,000 ft the pilot had asked
  for). No top of descent is published on an approach, and none is called
  in a climb (`CLIMBING_FPM`). **And a level the plan handed down is not a
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

## 2026-09-26 trim: Helicopters: the hover loop

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
- **The climb out is checked against the ground at the arm**
  (`heli_route::climb_conflict`, 15 nm probed 500 m either side, the 500 fpm
  departure within 500 ft of it refuses with the numbers; LILO-LILB
  2026-09-24 flew into a 7,728 ft ridge 4.3 nm out: the cruise was sized over
  it, the climb to it never was). The cruise itself is sized over a 1 km
  corridor (`sample_corridor`), not the track. No helipad lands on the hard
  runway end nearest the arrival (`runway_spot`), never the reference point.
  A helipad landing ENDS there, shut down; a hover taxi off a runway is said
  and waits 5 s for "my controls" (the pilot stopped an unannounced lift off
  LILO's H1). The parking brake is set at a landing only under 2 kt
  (`touchdown_brake_due`: the first cut set it at 19 kt on the ridge).
  **The climb out is FAST where the ground allows** (`fast_climb`, picked at
  the arm by `climb_conflict` on both profiles): the hover loop to 500 ft,
  then the autopilot on vertical speed at 800 fpm to the cruise dial with the
  collective on 75 kt, eased to 400 fpm under 55 kt; it is shallower over the
  ground than the 35 kt climb, so a valley that only the slow climb clears
  gets the slow climb, said. **The approach gate comes down** as the high
  ground passes behind (re-probed at each fix, `gate_lowered`). Both unflown.
  **Below 50 kt collective is not a speed control** (LILO-LILB, 2026-09-24,
  crashed): a gate reached at 86 kt put the speed law's collective at its
  0.25 floor, the VS mode held the level with 15 degrees of nose-up, the
  speed fell to 25 kt, the law drove the collective to 0.65 and the
  helicopter CLIMBED, and the pitch/collective cycle grew past the 20 degree
  limit over a ridge. The approach and slow-down floor the collective at
  0.35 (`APPROACH_COLLECTIVE_MIN`), the hover loop takes over once slowed
  whatever the height, the approach floor is probed over the 1 km corridor,
  and a point near the pad is dropped only when it lies PAST the pad (the
  drop of the pilot's lake point sent the last leg over the hills). Unflown.
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

## 2026-09-26 trim: The flight model: one state, read by everyone (in SHADOW)

[src/flight.rs](src/flight.rs) (2026-09-25) is the ONE answer to "where are
we and what do we hold", replacing `monitor::estimate_phase` (one snapshot,
no memory) and the private peak/airborne/landed flags of the checklist
tracker, the cabin and rapport. Pure, fed once per monitor tick, time from
the sample so recorded flights replay exactly (`flight::Replay`). Tracks:
the `Phase` (every edge sustained and in the `legal` table), `Control`
(the innermost loop with hands on, the AP, the pilot), `Abnormal`,
`Memory` per flight, and the crew's `Situation` (the outermost mission,
the plan on the shelf, the active fix, the party line's clearance, the
pilot's last takeover via `flight::note_pilot_takeover`, set by "my
controls" and every yield stand-down). **A ground strike is not a landing**:
climbing into it, or a sink past `IMPACT_SINK_FPM`, is an `Impact`, and a
wreck (impact or the crash flag) has no phases until it is at rest (the
S-76's ridge strike at LILO read as a touchdown at +491 fpm, then its slide
as a takeoff roll and a reject). Live: `fphase` and `crashed` ride the
`state` record, events are blackboxed `flight`; `xvl debrief` prints the
shadow against the old estimate. It is now READ: it ticks first in the
monitor, the checks' coarse phase is `monitor::coarse_phase` of it
(`estimate_phase` is gone), the checklist tracker's flight is the model's,
the cabin takes its `FlightFacts` (landed, repositioned), and "how are we
doing?" answers from `flight::status_line`. Rapport still keeps its own
touchdown bookkeeping (owed). **The duty engine** ([src/duty.rs](src/duty.rs),
pure) answers "what does the flight owe next" from that state every tick;
SHADOW ONLY: blackboxed `duty` on change with whether the running
automation agrees, listed by the debrief. It earns the right to act once
its logs agree with flights actually flown.
**The hand-over reads the model** ([src/handover.rs](src/handover.rs),
2026-09-25, pure): "you have control" / "take over" / "your aircraft" is
`grammar::TAKE_OVER`, resolved in the main loop from `flight::published()`
into the calls the phase needs (taxi out, taxi to parking, hold at the hold
or on the runway, autoland on the approach, resume the plan with
`level_off` as the `if_idle` fallback, level off with no plan, the hover
hold), or a spoken decline (parked, on the roll, in the flare, a loop
already flying). The upset RECOVERY is only for an actual upset (bank past
45, pitch outside -20..25) or the distress words; every hand-over used to
arm it. The pilot's side ("I'll taxi", "let me fly", "I'll take it") is the
release guard. Both directions land in the model (`note_crew_takeover`
clears the pilot's takeover; blackbox `handover`).

## 2026-09-26 trim: Checklist flows: the list at its phase, three flavours

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

## 2026-09-26 trim: App modes & cabin crew

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

## 2026-09-26 trim: The copilot as a person: timing, persona, rapport

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

## 2026-09-26 trim: Panel, terminal, settings

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
- **Only settings persist** (`prefs.json`, env, the plugin's window
  geometry); every mission, clearance and pilot-held target is in memory,
  and a restart mid-flight is recovered by "resume" from the sim. Considered
  and parked (2026-09-25, super low priority): a narrow `mission.json` (plan,
  mission kind, held altitude/speed; offered at boot, never applied, dropped
  if stale). Never a general state dump: persisting process-wide facts is
  the outlives-its-context bug class across a process boundary.

## 2026-09-26 trim: Web API load is a sim cost

The API is served from inside X-Plane's flight loop - every request is frame
time the sim does not get. `crates/sim/src/xplane.rs` reports rate / cache
hits / failures / slowest every 30 s (log + blackbox `api_load`, with a
per-dataref histogram - the only thing that can name a hog). Reads go through
a 60 ms value cache (a write drops its own entry so readbacks stay real);
near-static flags (`is_in_replay`, `paused`, ...) are `SLOW_DATAREFS` at
500 ms. A COMMAND clears the whole value cache (it can move any dataref, and
the stepped-knob tools rest on the read after it). Volume, not duplication, is where the cost is: `cached=0` in a window
is the honest answer, not a broken cache.

## 2026-09-26 trim: Battle-test surface

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
diagnosing by hand. **A sortie is one file with a verdict** (2026-09-25, for unattended
testing): every mission is a sortie (`full_auto::SortieGuard`, blackbox
`sortie` begin/end, every record tagged `sortie`, copied to
`logs/sorties/<id>.jsonl`); [src/scorecard.rs](src/scorecard.rs) turns the
ticks into `stage_result` (taxi, line_up, alignment, takeoff, landing: a
number and a limit each) and a `sortie_result` (PASS only with no failed
stage, no give-up and no `intervention`), live through a blackbox tap and
in `xvl debrief`. `state` carries the hands and levers (`in`: throttle,
beta, brakes, yoke, AP, wind, sim speed, torque, the plugin's yield
latches) and `ctl`; every spoken line is `said`. The main file rotates past
200 MB. A complaint that is not yet a scorecard number is not yet testable. **Run tests with `XCOPILOT_BLACKBOX=off`**: any test
that walks a real path records into the live log (22 test lines landed as
"flight 117" on 2026-09-13).

When a control law misbehaves in the air but its test passes, suspect the
test's plant first: a first-order lag cannot ring at any gain. Sortie tests
should fly second-order plants through the loop's real transport delay and
assert the numbers the blackbox reports.

## 2026-09-26 trim: Licensing

**GPL-3.0-or-later** (root `LICENSE`, inherited via `[workspace.package]`).
Settled: open source, never commercial, X-Plane only. Copyleft on purpose
(the ecosystem is payware-dominated and this exact product shape sells for
$49). SDK and Dear ImGui are MIT/BSD; B612 is OFL - all
GPL-compatible. Sole-author relicensing stays free only until outside
contributions merge: settle any change of heart BEFORE accepting a PR.
`crates/atc` could become LGPL-3.0 if it should ever be broadly embeddable -
deliberately not done now.

## 2026-09-26: taxi turn card (GPS-style arrow) on the panel

The strip's next-turn hint was a line of text; the Flight tab now carries a
turn card the plugin can draw. The app publishes `xcopilot/status/taxi` as
`kind|dist_m|label|aim_true_deg|xtk_m|turn_deg|remaining_m` from
`taxi_guide::TurnCard`: the same geometry as the strip hint, computed by
`Guide::turn_card` every monitor tick, change-gated and riding the periodic
resend, empty when there is no guide. Off the route past 60 m the card is
kind X and points nowhere, so the compliance watch keeps that conversation
(never two voices on one mistake). The aim bearing is the route 25 m ahead
of the projection (`AIM_AHEAD_M`), and the plugin rotates it against the
live true heading each frame, so the arrow moves at the sim's rate while the
app ticks at the monitor's. The plugin draws the arrow, the distance, the
label and a cross-track bar on the Flight tab. Layer two, arrows on the
pavement through XPLMInstance, is still owed. Offline only.

## 2026-09-27: local Web API clients skip the system proxy

Three `crates/sim` fake-server tests (`plugin_reload_re_resolves_a_stale_write_and_reads_back_the_new_id`,
`cancellation_during_resolution_prevents_the_later_patch`,
`pause_defers_handoff_until_live_measurements_resume`) failed now and then with
`missing GET /api/v1/datarefs?`. The request was never wrong: the fake gives
the client 10 s to connect, and the first `reqwest::Client` build in the
process spent that time in reqwest's default system proxy lookup (macOS).
Measured: 7 to 12 s for the first default build, under 1 ms with
`.no_proxy()`; `scutil --proxy` itself answered in 10 ms, so the stall is the
in-process lookup (possibly the sandbox; unconfirmed outside it). Every
client that only ever talks to the local Web API now builds with
`.no_proxy()`: `XPlaneClient::new` (`crates/sim/src/xplane.rs`),
`plugin_status::run`, `livestate::run`, the audio bus writer
(`tts::bus_write`) and the three panel pollers in `src/pollers.rs` (request
buttons, voice picker, settings). SimBrief keeps the default client. The sim
tests went from about 7 s (12 s when they failed) to 0.13 s.

## 2026-09-27: four ATC loose ends (vacated report, transponder, line-up refusal, runway request)

From `docs/tasks/2026-09-27-atc-loose-ends.md`. Offline only.

- **The region handoff readback into the void** (2026-09-15) was already
  fixed by `void_outcome` (2026-09-21): every sortie since logs it
  `handoff_readback` (four in `logs/sorties`), and the whole blackbox has no
  `outcome: void` left.
- **"Runway vacated" frees the strip.** The tower FSM keeps a `vacated` flag
  on the landing (`Tower::holds_runway`, `runway_vacated`; the state still
  stands at `ClearedToLand`, so a bare "request taxi" is still the taxi in;
  set only on the ground, dies with the landing). The clearance snapshot
  publishes `holds_runway`; the runway pick stops freezing on a landing that
  has been reported off; the party line's blocked strip
  (`station::player_blocked_strip`) is the strip we stand on or the one the
  book still gives us on the ground, so a ghost's landing gets "continue
  approach" and a ghost's departure off our strip goes un-narrated until the
  report. There is no second controlled aircraft in the FSM: the ghosts are
  the "second aircraft".
- **One transponder check** (`radio::departure_check`): the assigned code
  dialled where the box does not show it and read back, ALT through
  `set_mode`. Full auto at the hold and before a runway start's call
  (`transponder_check`), the helicopter's pad departure, and the takeoff's
  roll and liftoff (`transponder_alt` with `code_ok`/`mode_ok`), so the spoken
  takeoff and the circuit leave with the squawk too. Why the tower said
  "recycle" twice at EGPH: the radar watch nudges up to its cap of two when
  the code shows and the box does not reply (stock mode below ON), airborne
  only; not a height rule. The watch now publishes its nudge
  (`party_line::take_squawk_nudge`) and the crew recycles (standby, code, ALT)
  or re-dials, says so and reads "squawking ..." back (blackbox `atc`
  `squawk_answer`); the watch's next tick is "radar contact".
- **The line-up refusal names the cleared runway**
  (`autotakeoff::why_no_fit_cleared`): ETND's recorded stop (52.585512 N
  8.349835 E, 016) said "08L is 152 metres away" under a 26R clearance; it
  now says 26R, same 152 m. Other gates stand as diagnosed.
- **A short cleared runway is refused and a longer one requested.** The
  tower has `Intent::RequestRunway` ("request runway zero eight", "unable two
  six, request zero eight for length"; the runway read by
  `tower::requested_runway`, the intent kept `Copy`), resolved by
  `resolve_runway_request` against `nav::assignable_runways`, granted as the
  taxi to it (route re-planned, runway in use committed), refused with the
  reason, "say again" with no runway in the words. Full auto judges the
  cleared runway at the taxi clearance (`judge_cleared_runway`,
  `cleared_runway_fit`, the ONE rule at the weight on the wheels; blackbox
  `cleared_runway_fit`, `runway_granted`): short, it says both numbers and
  asks for the longest assignable runway that fits, flown the cleared way; a
  pilot taxiing hears the numbers and the words to say (`atc`
  `cleared_runway_short`). `party_line::field_runways` is the new accessor.

## 2026-09-27: the tower issues IFR clearances and the crew flies them

The gap: every "request clearance" got the VFR zone clearance ("cleared to
leave the zone, not above two thousand feet, squawk ..."), readback only
suggested, while full auto flew SimBrief plans, picked its own SID
(`choose_departure`) and set its own initial altitude. Nobody ever said
"cleared to Edinburgh via the ... departure, climb five thousand feet".

- **The intent** `tower::Intent::RequestIfrClearance { dest }` (`ifr_request`,
  after the question guard, before the runway request): "request IFR
  clearance to EGPH", "request clearance to Edinburgh", "IFR to Malaga, ready
  to copy", "ready to copy", "as filed", a spelled code ("Echo Golf Papa
  Hotel" is EGPH). "Clearance to leave/taxi/land/start/the holding point"
  names no place and a bare "request clearance" stays the zone clearance.
  `Intent` is Clone, no longer Copy (the dest is a String); the loose-ends
  entry above says "kept Copy", which the merge ended.
- **The clearance** (station `ifr_offer`, tower `ifr_clearance_line`): the
  destination resolved against the region index (`resolve_destination`: code,
  three letters also with K, else name words; two fields by one name get no
  guess), unknown is "say again destination", none named "say destination".
  Initial altitude `tower::ifr_initial_altitude_ft`: the higher of 5,000 ft and
  field + 3,000, rounded up. The code: the discrete one at a radar field.
  ICAO / CAP 413: "cleared to Edinburgh via the Gosa one Charlie departure,
  runway zero niner, climb five thousand feet, squawk four five two one, QNH
  ...". FAA: "cleared to Edinburgh airport via the ... departure, then as
  filed, climb and maintain five thousand, departure frequency ..., squawk
  ...". No SID: "after departure fly runway heading", never an invented name.
- **The SID leaves toward the destination** (`nav::sid_exits`,
  `nav::sid_toward`): `Station::set_procedures` positions every SID's exit fix
  (last fix of its runway transition plus common route, one scan of each nav
  file) on a background thread; the SID whose exit bearing from the field is
  nearest the great-circle bearing to the destination is given. No positioned
  exits yet: the first SID in the file for the runway (`sid_for_runway`).
- **The readback is ENFORCED** (`Awaited::IfrClearance`,
  `check_ifr_readback`): SID (by name or designator, "one Charlie"),
  altitude and code must come back, a runway named must be ours. Wrong items
  get "negative, ..." for as long as it takes; missing ones "read back ..."
  and spend a challenge; a mishear is challenged twice, then let through with
  NOTHING published. With `XCOPILOT_ATC_READBACK=false` any readback that says
  the clearance stands.
- **The seam**: `party_line::assigned_clearance()` (crate-private
  `IFR_CLEARANCE`), set only on the accepted readback, retired with the squawk
  by `install`, `uninstall` and the out-of-reach stand-down.
- **Full auto flies it**: a SimBrief plan, or a local one above the
  transition (`plan_is_ifr`), asks for it with the ICAO spelled
  (`get_ifr_clearance`), reads back the tower's own coach line, dials the code
  once accepted, and falls back to the zone clearance when none comes.
  `choose_departure` tries the cleared SID first (`prefer_cleared_sid`,
  `cleared_sid_for`); `departure_rechosen` names its source, cleared or chosen;
  a plan whose runway stands flies the tower's SID or keeps its own
  (`cleared_sid_not_flown`), never a third. The takeoff dials the cleared
  altitude and the follower's climb is capped there
  (`procedure::hold_at_cleared`, `cap_at_clearance`; a pilot's altitude, the
  terrain floor and a SID minimum still outrank it) until the departure
  check-in, or 90 s level with nobody to hand us over.

Known weaknesses: the destination must be in the region index (staffed towers
only; an untowered destination or `XCOPILOT_ATC_REGION=off` falls back to the
zone clearance); a tower-only field clears with the code already held (7000
unless assigned); the cap is released when the check-in is TRANSMITTED, not
when departure answers; a second IFR request while a readback is owed gets
"read back: ...", not a new clearance.

Blackbox: `atc` `ifr_clearance` (side tower with `sid_exit_brg`, `dest_brg`;
side crew), `full_auto` `request_clearance` with `ifr: true`,
`ifr_clearance_missing`, `ifr_clearance_not_accepted`, `departure_rechosen`
`source`, `cleared_sid_not_flown`, `cleared_ceiling_released`, `procedure`
`cleared_ceiling`.

NOT flown.

## 2026-09-27: flap retraction waits for the flaps and names the detent

Sortie `1790494983-full_auto` (B738, Alghero): after takeoff the crew said
"Flaps coming up a stage." twice, 0.5 s apart: `flap_step` to 0.25 at t 752.5
(1,011 ft AGL, 191 kt), to 0.125 at t 753.0 (1,039 ft, 191 kt), then "Flaps
up." to 0.0 at t 766.6 (1,710 ft, 210 kt). Same pattern on the day's earlier
sorties. The retraction is held to the acceleration height
(`climb_out.accel_agl_ft`); by then the IAS was past both detents'
`retract_above_kts`, so `flap_retract_step` returned a step on two
consecutive ticks, each commanding the next detent before the flaps had
moved, each with the same vague line.

- **Paced on the readback**: `autotakeoff::paced_flap_step` takes a step only
  once `flap_handle_deploy_ratio` has arrived at the last commanded detent
  (`takeoff_config::flaps_arrived`, the takeoff config's own `FLAP_DEPLOY` /
  `FLAP_ARRIVED`, now shared). Read only while a step is due, not every tick
  of the roll; a failed read is no step. The handoff's clean-up is
  unchanged (it only acts on flaps still out, and speaks once).
- **Named by its detent**: `flap_step_line` says "Flaps one.", "Flaps up.",
  from the detent's `spoken` (`aircraft::flap_detent_spoken` over the live
  profile); no name keeps "Flaps coming up a stage.". `flap_step` records
  `detent` and `deployed`.
- **The 737's schedule matches its comment**: detent 2 declared
  `retract_above_kts: 180` and added a step to the 5, 1, up the profile
  describes; removed. No other profile's schedule contradicts its comment.

NOT flown.

## 2026-09-27: centre tank pumps come off when the centre is dry

The pilot found both CTR LOW PRESSURE lights lit on the stock 737 with the
engines running. Probed live: `sim/flightmodel/weight/m_fuel` `[2916.8, 0.0,
2918.1]` (tank 1, the centre, empty; `acf_tank_rat` `[0.187, 0.626, 0.187]`)
and `sim/cockpit2/fuel/fuel_tank_pump_on` `[1, 1, 1]`. The pilot switched CTR
off, element 1 read 0 and `laminar/B738/annunciator/low_fuel_press_c1`/`_c2`
went out: the stock array IS the cockpit switch on this airframe. The xlua:
`B738.Z.autostart.lua` step 4 switches the CTR pumps ON whatever the centre
holds (before either engine turns), `B738.systems.lua:3248` writes 0 only at
cold and dark, nothing rewrites it per frame. Our side never touched it, and
the load sheet fills the wings first (`loading::distribute_fuel`), so every
sector that fits in the wings flew its whole length with the pumps dry.
(Laminar's own lights come on below 625/640 kg in the centre, so a light can
be lit over fuel that still feeds; the rule below waits for empty.)

- Profile: `fuel_tank_pumps_off_when_empty: [..]`, tank indices of the stock
  array; silent = off. `aircraft::profile_references` adds the three names
  the declaration relies on (pump switch, `m_fuel`, `acf_tank_X`), since the
  field itself is only numbers. Declared: the 737 `[1]`; the MD-82 `[0]`
  (overhead switches animate straight from the stock elements, the xlua writes
  it only at flight start and the boarding step; tank 0 is "Center" at x 0;
  read from the files, not probed live). Not declared: the A330 (its
  `A333.systems.lua:2113` rewrites element 1 from its own two CTR buttons, the
  stock array is an output there), the Sparky 747 (its xtlua runs its own
  pump model and never touches the stock array).
- A main tank is never touched: `fuel_pumps::is_centre_tank` honours a
  declared tank only when `acf_tank_X` puts it on the centreline (0.1 m, the
  load sheet's rule) with wing tanks beside it; a wrong declaration is logged
  (blackbox `fuel_pumps` `rejected`) and ignored, never written. A single-tank
  airframe's centreline tank is its main.
- One decision, pure: `fuel_pumps::pumps_to_switch_off` (pump on AND tank
  `<= EMPTY_TANK_KG` 5 kg AND an engine running; a failed read of any array
  is no measurement). The monitor's `centre_tank_dry` check (sustained 5 s)
  is the altimeter's shape: an automation flying switches them off through
  `fix_write` (dispatched to `fuel_pumps::switch_pumps`, the one writer,
  read back, a switch that did not take said), a pilot flying hears it once
  per episode and nothing is written. Full auto needs no step of its own: the
  monitor runs on the ground and `automation_flying` includes
  `full_auto::ACTIVE`, so the fix lands within seconds of the start.
- Spoken: "centre pumps off" / "center pumps on" / "turn off the centre
  pumps" is `set_centre_pumps`, registered on every airframe so one without
  them answers why. Off is refused with the kilos while the centre holds fuel
  (its pumps are what feeds it); on is allowed whatever it holds.
- One writer: the MD-82 `power_up` switched all three pumps on with raw
  writes; it now goes through `fuel_pumps::switch_pumps` too. `set_fuel_pump`
  writes the ENGINE pump array, the Twin Otter override fires commands.
- Blackbox `fuel_pumps`: `switched` (source monitor | pilot | power_up; tank,
  kg, before, after, took), `advised`, `rejected`. Offline only; the proof is
  a 737 start at a stand on a wings-only load, a `switched` record from the
  monitor for tank 1 within seconds of both engines running.

## 2026-09-27: a descent never climbs back, and an IFR flight keeps its code

LIEA-LEBL, B738, full auto, unattended (sortie `1790494983-full_auto`).

**A climb in the descent.** The plan's profile (the `plan_audit` record at
t 1790494983.0): KIFMI 31000, ELVOP 28000, MUREN 28000, CHELY 29000, VERSO
25000, OSTUR 16000. The follower called its top of descent at 08:22:00 ("12
miles ahead, down to 28,000"), began down from 31,000 at 08:23:46, levelled
at MUREN's 28,000 (08:25:51), then armed CHELY at 08:26:43 and CLIMBED to
28,999 by 08:28:37 before VERSO (star_ceiling 23,655) took it down again.
SimBrief's navlog altitudes down a descent are predictions, not a step climb.

- `procedure::vertical_tick` takes a `DescentProfile`: the plan's cruise (its
  highest leg altitude; `None` on an approach), the highest altitude the PLAN
  has commanded (never the pilot's or the floor's), and whether the leg is
  past `plan_top_index` (the last leg at cruise, which covers a follower
  armed mid-descent). Descending = commanded the cruise and now 100 ft under
  it, or past the top and under the cruise. Then a plan altitude above the
  last commanded altitude is held at it (`Vertical.descent_held`), the carry
  too, until a lower one takes over. The pilot's handed altitude still wins
  and the terrain floor still raises it (both applied after); a step climb
  before the top of descent never reaches the cruise first, so it climbs.
- Blackbox `procedure` `descent_no_climb` once per leg: idx, fix, plan_ft,
  held_ft, alt_ft.
- Tests replay the recorded profile through the pure vertical (the old law
  climbs to 29,000 at CHELY, the new one holds 28,000 and VERSO descends),
  plus a pilot climb, a floor above the held level, a pre-TOD 29,000 to
  31,000 step and a mid-descent arm.

**Seven thousand on an IFR flight.** The flight accepted an IFR clearance at
the ramp (4501, `ifr_clearance` side tower and crew, `squawk_set` 4501). At
08:36:52, about FL200, the one region switch (LIEA to LEBL) said "Radar
services terminated. Squawk seven thousand. Contact Barcelona Tower on one
one eight decimal one zero five." and full auto set 7000 (`region_handoff`
`squawk_vfr=true`, `squawk_set=true`). The squawk decision read only the
radar watch. `IFR_CLEARANCE` is the departure field's worksheet and dies at
the next install, so it could not carry the flight rules either.

- Party line `IFR_FLIGHT` (code, callsign, airborne): set with the accepted
  readback (`publish_ifr_clearance`), NOT cleared by `Station::install`,
  retired at the landing (`note_aircraft`: airborne since, then on the
  ground; blackbox `atc` `ifr_flight_ended`), by a ground stand-down out of
  reach, by a new clearance offer and by `uninstall`. Accessor
  `party_line::ifr_flight_code`. There is no "cancel IFR" intent yet, so a
  cancellation retires nothing.
- `region_tick` with it: `squawk_vfr=false`, `RegionHandoff.ifr_code` the
  code, and the line `region_handoff_line`: "<callsign>, contact Barcelona
  Tower on ..." (no service terminated, no squawk). VFR is byte-identical.
- Crew: `obey_region_handoff` already dialled only for `squawk_vfr`; it now
  records `ifr_code` and the transponder's `code` after the handoff (takes
  `&dyn XPlane`, so a fake sim tests it). The monitor's coach says "We've
  been handed over" for IFR, not "Radar's done with us".
- Left as it was: the handoff still goes to the destination's TOWER. The
  crew's arrival checks in with the tower (`call_inbound`); a handoff to
  Approach would sit the flight on a frequency nobody calls until then.
- Next IFR full-auto sortie, check: `procedure` `descent_no_climb` (if the
  navlog bumps up in the descent) and the leg ticks' `want_alt` never rising
  after the top of descent; `full_auto` `region_handoff` with
  `squawk_vfr=false`, `ifr_code` and `code` both the cleared code, `squawk_set`
  null; the `atc_region` line without "seven thousand"; `atc`
  `ifr_flight_ended` after the touchdown. Offline only.

## 2026-09-27: a profile per aircraft model, never per ICAO type

Detection mapped `sim/aircraft/view/acf_ICAO` to a profile through a Rust
table (`detect::known_profile`). The stock Laminar 737 and the Zibo 737 both
say B738, so the Zibo would have flown `boeing_737_800`: the stock datarefs,
the stock xlua assumptions, the stock `scripted_auto_start`. The same shape as
the RV-10 on the 172's numbers (2026-09-11), one level down: a type is not a
model.

- Every `profiles/*.yaml` declares a `match:` block: `icao` (required) and
  substrings of `acf_relative_path` (`acf`), optionally `author` and
  `description` (substrings of `acf_author` / `acf_descrip`). Case-insensitive,
  backslashes as slashes. Every key given must hit. The table is gone; the
  blocks are read by `aircraft::profile_matchers` through the same reader as
  `aircraft::load` (built-in 172 and SR22 carry theirs for a missing YAML).
- One pure resolver, `detect::resolve(&AcfIdentity, matchers)`: candidates
  are the profiles claiming the ICAO; the most keys matched wins; a tie is a
  profile-authoring error, warned and picked by id (a test pins no tie on any
  installed .acf). An ICAO hit with no model match is `UnknownModel`: the
  model's own `.acf` placards under `generic_<icao>_<acf stem>`, and the pilot
  hears "This is a B738 but not one I have a profile for. I'm flying it on the
  placards from its own aircraft file. Treat every automation as untested."
  `detect::pick_profile` wraps it for BOTH the startup probe and the swap
  poller (they were two copies of the decision). The poller also says an
  uncurated airframe at boot, which the old startup path only logged.
- The poller's change key is (icao, normalised .acf path) (`model_key`), so a
  stock-to-Zibo swap re-resolves; author and description are read only on a
  change; the livery path is never read.
- Type-level callers stay type-level: the stand pick from a SimBrief plan
  (`tools/setup.rs`) asks `aircraft::profile_for_type`; `plan_audit`'s
  "another type" refusal stays on the ICAO. A plan names no model.
- Matchers: the Laminar folders for the stock aircraft, `AWXDC3/` (the dc3
  profile is written for the AWX DC-3), `DHC6-300 Twin Otter V2/`, and
  `Sparky/747-400/` plus author `see github` from the Sparky's own
  `acf/_author` line. A renamed folder makes the model generic, said aloud;
  the YAML comment says what to change. `DHC6F.acf` (the float Twin Otter)
  has no `acf/_ICAO` line at all, so the detector sees an empty ICAO and never
  resolved it before or after.
- Blackbox: `profile_probe` gains `identity` (icao, relative_path, author,
  descrip, resolution `profile` / `unknown_model` / `unknown_type`, the keys
  that matched, `tied_with`). Next session, check it on the first probe, and
  on any add-on that shares a stock ICAO the `unknown_model` resolution and
  the spoken line. Offline only.

## 2026-09-27: the gyro auto-sync reads its drift back

On the stock 737-800 (LEBL-LFBD full auto, sortie `1790504948-full_auto`) the
monitor said "Heading indicator had drifted about N degrees, re-synced to the
compass." 14 times, once every 120 s, while `dg_drift_vac_deg` climbed from
11.8 to 15.6: `sim/instruments/DG_sync_mag` was ACCEPTED and did nothing (no
vacuum DG, the heading is IRS and glass), and the line went out on the
command's acceptance, never on a readback. The core rule again: a silently
failed write must not answer "Roger". Now `monitor::GyroSync` owes a readback
after the fire: at least 1 s later the drift is read directly (a failed read
is no measurement; the snapshot's 0.0 default would have looked like a perfect
sync) and judged by `gyro_sync_took` (the drift must at least halve). Took:
the line, blackbox `gyro_sync` `took: true`. Did not take: silent, `took:
false` with `drift_deg` and `after_deg`, and latched off for the airframe
(the loop builds a fresh `GyroSync` on every swap). No read within 3 s:
`took: null`, silent, not latched. Deliberately not gated on the systems
survey's `glass_hsi`: an SR22 flagged glass genuinely drifted 166 degrees
(2026-07-12); the readback is the guard. The spoken line lost its em dash.
Offline only.

## 2026-09-27: a flare is credited the air it has flown; a balk that lands never calls "going around"

Sortie 1790504948-full_auto, the stock 737-800 into LFBD 11 (2,418 m), the
second approach, cleared to land. The flare opened at 38 ft at about 148 kt
and floated. At 7.9 ft radio, 796 m past the threshold (0.43 nm) and sinking
184 fpm, `guidance()` balked `overflew_the_runway`: the float allowance
(`overflight_allowance_nm` with the POM's 5,360 ft = 1,634 m) ran out at
784 m. `commit_flare` did not save it: 1,622 m ahead against 1,634 m
needed, short by 12 m. The copilot said "Going around. Full power, climbing
straight ahead.", the throttle went to 1.0 for about a second, "going
around" went out on the radio on the balk tick, and the tower answered
"roger, go around ... report downwind" and dropped ClearedToLand to
Inbound. The wheels touched 1.3 s later with the engines still spooling,
the AP came out, the rollout took it and it stopped with about 700 m to
spare (touchdown about 1,578 m from the end, 880 m of roll). The crew had
called a go-around and then landed uncleared.

**The committed flare's need** (`autoland::flare_pavement_need_m`): a
recited landing distance runs from 50 ft over the threshold
(`LANDING_SCREEN_FT`), and a flare at 8 ft has already flown the part of it
between the two, so the whole figure counted it twice. The need is now the
book figure less the air a 3 degree path covers from 50 ft down to this
height (`SCREEN_AIR_M` = 50 ft x 0.3048 / tan 3 degrees = 291 m, credited
in proportion: 245 m at 7.9 ft, none at or above 50 ft, so a balloon asks
the whole figure). The book's own air segment is longer than that geometry
(it includes the float), so what is asked is the ground roll plus a float,
never less. Floored at half the book figure: the 172's book is 1,335 ft
with 575 ft of roll (43%), and the geometric credit alone would ask 116 m to
stop in 175; a jet never reaches the floor (737: 1,343 against 817). With
no declared figure the need stays the Baron's 900 m whole: that is a
measured rollout plus margin, with no air in it to credit. At LFBD the need
at 7.9 ft is 1,389 m and 1,622 m commits. The SF50 test's "no room" case
moved from 400 m to 300 m at 20 ft (it now asks 350 m). The float's own
judgement above the band (`overflight_allowance_nm`) is unchanged: it
already subtracts the run to the band, and the low end is what the commit
owns.

**No radio call for a balk that lands** (`go_around_call_due`,
`BalkWatch`): the latch and the intercom's "Going around" still act on the
balk tick (the power is coming and the pilot must know why), but the tower
call, and the `owe_final_report` it carries, wait until the go-around is
real. At or above `GO_AROUND_CALL_AGL_FT` (200 ft, twice what a 737 loses
spooling from idle at the path's sink) it goes at once; below, once the
radio is `GO_AROUND_RISE_FT` (10 ft) above its lowest since the balk with
the VSI (one tick old) positive, or the radio's rise alone when the VSI
could not be read. A balk whose wheels touched calls only if it then climbs
through 200 ft (a touch that really left); one the rollout takes never
calls, keeps the clearance, blackboxes `balk_landed` (`why`, `balk_radio`,
`radio`, `gs`, `remaining_m`, `called`) and, if the intercom said "going
around", corrects it: "Correction, we're down. Landing, not going around."
The `going_around` event's `radio` and `why` are now the BALK's (the
debrief's "GO-AROUND at N ft radio: why"), with `called_radio`, `low_ft`
and `touched` added; the debrief prints `balk_landed`. The first LFBD
approach's `off_centreline_low` at 96 ft still calls as soon as it climbs.
Tests: `a_flare_is_credited_the_air_it_has_already_flown`,
`a_low_balk_is_called_on_the_radio_only_once_it_climbs`. Offline only.

## 2026-09-27: a climb never descends to a minimum it has passed; the cruise is reached by flying it

LFBD-EGKK, stock 737-800, full auto, cruise FL340 (sortie
`1790510400-full_auto`, issue #103). The follower flew the CNA6R from runway
29 as charted (`charted_departure_legs`): DF DER29 and DF BD923 carrying the
plan's 34,000 (neither is in the plan, so both take the join's altitude),
and between them the chart's CA "course 296, at or above 660 ft" (CIFP
`SID:020,5,CNA6R,RW29,...,CA,...,2960,...,+,00660`), which keeps its own
altitude. Climbing through 7,638 ft at t 1790511150.8 the follower
sequenced to the CA (idx 1, done at once, the aircraft being far above it)
and to BD923 (idx 2):

- idx 1: `commanded_alt_ft` gave the CA's 660, below the aircraft. The
  terrain floor (probe plus 1,000 ft, 705) was above it, so `terrain_floor`
  fired with floor 705 and "Terrain ahead, holding 800 feet." went out; the
  floor was only ever a minimum, the plan target was the fault.
- idx 2: BD923's 34,000 went through `DescentProfile::descending` with
  `prev_cmd` 705. `peak_ft` was the highest altitude the plan had COMMANDED,
  and FL340 had been commanded since DER29's first tick, so the profile
  counted the cruise as reached and 705 as a descent from it:
  `descent_no_climb` held BD923 (705), ROVFU (1,011), CNA (1,190), MANAK
  (1,336). The 737 went down to about 1,200 ft and flew there for 14
  minutes until the pilot said "set altitude 34000" (t 1790511982).

Fixes (`procedure::vertical_tick`, `DescentProfile`):

- `peak_ft` is now the highest altitude FLOWN (the tick's `alt_ft`), and
  `descending` needs the cruise flown to within `STEP_FT` or the leg past
  `plan_top_index`. FL340 commanded but 7,700 flown is not the cruise.
- The mirror rule, `DescentProfile::climbing` (a cruise, not past the top,
  never flown, no arrival ceiling): a plan altitude below the level already
  commanded is held at that level (`Vertical.climb_held`), the carry too. A
  CA leg's "at or above" and a navlog prediction below the aircraft are
  minimums passed, never a descent. The pilot's altitude and the terrain
  floor still apply after it.
- `DescentProfile.arrival_ceiling`, set when `ceilinged_leg_alt` lowered the
  leg: a STAR window or a later plan altitude kept reachable is the plan's
  own descent, so a short flight that never reaches its cruise still makes
  its "at or below". It does not make the flight `descending`.
- Blackbox `procedure` `climb_no_descent` once per leg: idx, fix, plan_ft,
  held_ft, alt_ft.
- Tests: `a_climb_never_descends_to_a_minimum_it_has_passed` (the sortie's
  DER29, CA 660 and BD923 at 7,618 to 7,658 ft with the floor at 705: 34,000
  throughout, no floor, no `descent_held`),
  `the_lfbd_climb_bit_without_the_profile` (the old CA tick gives 705),
  `a_cruise_commanded_but_not_flown_is_not_descending`,
  `an_arrival_ceiling_in_the_climb_is_flown`; the LIEA-LEBL descent tests
  still pass (their step-climb test now pins the flown peak).
- Not touched: why idx 0 (DER29, a DF) was flown for three minutes to 7,600
  ft before sequencing.
- Next IFR full-auto sortie with a charted SID, check: `procedure`
  `climb_no_descent` on the CA leg with held_ft the cruise, the leg ticks'
  `want_alt` never falling before the top of descent, no `terrain_floor`
  in the climb unless the ground is really above the plan, `descent_no_climb`
  only after the cruise was flown. Offline only.

## 2026-09-27: the speedbrake is never out with landing flap or gear

EGKK 26L, stock 737, sortie `1790510400-full_auto` (issue #104). The
follower's descent drag (`procedure::descent_drag`) put the speedbrake out
at t 1790515336 (218.4 kt against a 210.25 target, vs_cmd -2,000, flaps up)
and the speed kept it out while the follower configured for the final to a
MAP on the runway: `map_config` gear down with flaps 5 at t 1790515473, then
flaps 10, 25, 30 and 40 by t 1790515517. It came in only at t 1790515530,
by speed (146.8 against 145), and went out again from t 1790515587 to
1790515592 at flaps 40 with the gear down (153.6 against 145, 968 fpm
down). The 737's SPEEDBRAKE EXTENDED caution kept tripping the master
caution through the approach and the pilot asked why. The speed rule had
no notion of configuration: a crew never flies speedbrake with landing flap
or gear.

Fixes (`src/procedure.rs`):

- `drag_barred(flap_handle, gear_handle_down, flap_limits)`, pure: `"gear"`
  with the gear handle down, `"flaps"` with the flap handle past the
  airframe's first detent (`aircraft::flap_limits`, the 737's flaps 1 at
  0.125; any flap at all when no detent is declared). Flaps 1 is still the
  clean descent.
- The follower's tick reads `flap_ratio` and `gear_handle_down` (one
  `tokio::join!`, only while the drag is out or wanted) and a barred
  configuration forces the drag in: stowed if it is ours, never deployed.
  `descent_drag`'s shallower rate still applies, so the speed is taken out
  of the descent instead. A failed read deploys nothing and leaves a brake
  of ours to the speed rule. `DRAG_OURS` and the one writer are unchanged:
  a lever the pilot pulled stays the pilot's.
- Blackbox `procedure` `speedbrake` now carries `why`: `speed` for the old
  rule, `flaps` or `gear` for a configuration stow.
- Every exit already stows: `fly` returns on the autoland handoff, the MAP
  handover and every stand-down, and the spawn wrapper puts away a brake of
  ours (`follower_ended`) whoever ended it. The sortie's handoff at
  t 1790515620 found the brake already in.
- The test card's `descent_drag` row fails a `map_config` with gear or
  flap past 0.13 while the brake is out and no stow follows within 2 s;
  over this sortie it now reads "speedbrake out at 13:24 while configuring
  (flap 0.38)".
- Test: `the_speedbrake_is_never_out_with_landing_flap_or_gear` (the
  sortie's 218.36 against 210.25 at -2,000 wants it by speed; clean and
  flaps 1 may, flaps 5, 10, 40 and the gear bar it; no detents means any
  flap; the 153.6 against 145 re-deployment is barred; a runaway still
  shallows to half).
- Next 737 full-auto arrival, check: `procedure` `speedbrake` `out:true`
  never after a `map_config` with gear or flap past 0.125 until an
  `out:false`; a stow with `why` `flaps` or `gear` when the final is
  configured with the brake out; no SPEEDBRAKE EXTENDED master caution on
  the approach. Offline only.

## 2026-09-27: a MAP on the runway fix hands autoland the approach at the FAF

LFBD-EGKK, stock 737-800, full auto, runway 26L (sortie
`1790510400-full_auto`, issue #105). After the STAR (WILLO, 7,000 ft) full
auto armed "the ILS approach runway two six left" through
`approach_plan_for` (`straight_in_legs` of the CIFP final: IF CF26L, CF
FF26L flagged FAF "E  F", CF RW26L flagged MAP "G  M"; no transition, so the
join to CF26L was direct from WILLO, bug 021 to 042, a near-reversal onto
the 256 final). Autoland was handed the aircraft at RW26L:

- `handoff_index` took `legs.last().is_map` as the LSZS case (a MAP short
  of the runway, flown to and handed over there) and returned idx 2. The
  CIFP codes the MAP on the threshold of nearly every ILS final (EGLL 27R,
  LEJR 20, LIPE 12 carry the same "G  M"), so since 2026-09-24 every such
  approach was handed over at the runway, not the FAF. LIPE's 2026-09-25
  fix (`map_short_of_runway`) went into the hand-over's wording and the
  flap schedule, never into `handoff_index`; LIPR 30's "handed to autoland
  120 ft high at 0.8 nm" (2026-09-26) is the same fault.
- At t 1790515472 CF26L was captured 2.9 nm out (its turn onto FF26L
  about 150 degrees, capture 2.7 nm) and FF26L in the same tick (1.9 nm
  away, its own turn onto RW26L as large): "Next, FF26L." and "Next,
  RW26L." together, the aircraft 4.3 nm from the threshold, 2.8 km south of
  the centreline, track 042. The follower flies every fix leg DIRECT TO the
  fix (`bearing_to_fix` plus crab), a CF's course included: the turn crossed
  the centreline inside FF26L and overshot to 2.2 km north (t 1790515536),
  and the leg to RW26L homed on the threshold at track 240 against the
  runway's 258, 1.2 km off at 2.2 nm, 550 m at 1.1 nm.
- "At the fix. Landing us." at t 1790515620 at 310 ft radio 1 nm out,
  "Autoland to EGKK runway 26L, 1 miles", localizer and glideslope 2.5 dots,
  `going_around` `off_centreline_low` at 97 ft, xtk 244.5 m, 0.15 nm. The
  reversal pattern flew the second approach and landed.

Fix (`procedure::handoff_index`): the MAP rule asks `map_short_of_runway`,
so a MAP on the runway fix falls through to the FAF. On the sortie's tick
FF26L is done at once and autoland takes the aircraft 4.3 nm out on the
wrong side of the course, where `reversal::needed` flies its pattern onto
the final (the pattern the second approach landed from) instead of a 1 nm
final 550 m off. `faf_off_course` and `approach_flyable_from` now judge
FF26L, the FAF, as their comments always said. Test
`a_runway_map_hands_autoland_the_approach_at_the_faf`: the real EGKK I26L
rows, the FAF chosen, and the sortie's tick (51.141136, -0.052538, 261 kt)
completing FF26L more than 4 nm from the threshold.

Not touched (why the final was 244 m off, still open):

- The follower flies a CF leg as a direct-to, never on its course; a final
  flown by the follower past a FAF (the LSZS MAP legs) or any course leg
  entered off its line homes on the fix at an angle.
- An approach joined at its IF from the far side is a reversal the fly-by
  cannot fly (capture capped at the turn radius, two fixes sequenced in
  one tick); the published transition (WILLO, D113V, MAY, the FD to 8.5 DME
  from MAY) is not loaded, and autoland's pattern is the only reversal we have.
- Next EGKK-style arrival, check: `procedure` `handoff_autoland` idx at the
  FAF, the autoland arm's distance ("Autoland to ... N miles" of 4 or more),
  `reversal` events when the join was from the far side, no
  `off_centreline_low`. Offline only.

## 2026-09-27: the compliance watch runs through an autoland

Issue #63. The monitor's world block sits under `quiet` (a recovery,
`autoland::ACTIVE`, the panel's mute, a wreck), and the compliance watch
(`station::compliance_tick_facing`) was inside it, so it was never ticked
from the autoland arm to the rollout handoff. Its air/ground edge
(`prev_on_ground`) stayed the last airborne sample before the arm, and the
first tick after the handoff read as the touchdown. 2026-09-26 (N816NN, no
landing clearance, the tower still at `Parked`): touchdown at t 1790448341.9
(flight `touchdown`, 146 kt), autoland handed over stopped at 1790448368.3,
"you landed on runway three zero without clearance" at 1790448368.5, 26.6 s
after the wheels and 27.9 s after radio 0. 2026-09-27 (LFBD, sortie
`1790504948-full_auto`): the same shape, 20.8 s (touchdown 1790509565.7,
handoff 1790509586.3, call 1790509586.5); that one's clearance had been
spent by the crew's own "going around" call from a balk the wheels took,
which `balk_landed` already fixes.

Fix (`src/monitor.rs`): `world_gates` splits the gate. `quiet` (chatter,
incursion, ATIS, squawk, the landing-clearance coach) is unchanged. The
compliance watch has its own block before it: it TICKS on every live tick
but a wreck (`compliance_ticks`), so its edges never go stale, and SPEAKS
unless a recovery, the panel's mute or a wreck silences the world
(`compliance_speaks`, the cabin's gate); a muted call is consumed, never said
late. The COM1 read and `tuned_speaker` are hoisted out of the quiet block to
serve both. No duplicate voice: the crew's own landing readback
(`autoland` `landing_readback`) moves the tower to `ClearedToLand` and the
watch stays silent; an owed readback is still `ClearedToLand`; the
landing-clearance coach stays under `quiet`. Blackbox: `atc` `route:
compliance` gains `autoland` and `said`. Tests:
`an_uncleared_landing_is_called_at_touchdown_whoever_flies` (atc: every tick
calls on the touchdown tick; the old skipped window calls 26 s late; cleared
to land, silent through the rollout),
`the_compliance_watch_runs_through_an_autoland`,
`the_compliance_tick_is_not_under_the_quiet_gate`. Next autoland without a
landing clearance, check: the `atc` `compliance` record's `t` within a tick
of the flight `touchdown`, with `autoland: true, said: true`. Offline only.

## 2026-09-27: "Brakes on" at the stop is the parking brake's read-back

Issue #65, owed since EGSS 22 (2026-09-22, stock 737): autoland stopped on
the runway (gs 0.2 kt at t 1790031655), said "Down and stopped. Brakes on,
your aircraft.", and `state` read `park` 0.0 for the next 55 s (0.03 by hand,
the autobrake's residue) until the pilot's spoken "set the parking brake"
put it at 1.0 (t 1790031712). The rollout's closing write was
`xcopilot/taxi_brake` 0.5, the plugin's one-second heartbeat channel: the
loop broke on the next line, nothing renewed it, and nothing ever touched
the parking brake. The line was a constant.

Fixed, offline:

- `controls::set_park_brake_confirmed` (up to N drives of
  `drive_park_brake`, valve-aware, then the ratio read back into
  `ParkBrakeSet { held, tries, err, park }`); `full_auto::hand_back_brakes_if`
  now uses it instead of its own copy of the three-try loop.
- Autoland's stop hands the hold to it (valve from `live_profile`) and speaks
  `autoland::stopped_line(held)`: "Brakes on" only when it reads set, else
  "The parking brake did not set, hold the brakes." Blackbox `autoland`
  `stopped_brakes` {held, tries, park, err, valve_set}.
- The same bug elsewhere: taxi-in's rolling arrest ("Brakes on, holding for
  the taxi clearance", `arrest_rolling` now carries held/tries/park) and the
  native push stop ("Stopping the push. Brakes on") discarded the result;
  both now speak from it. "Belay that" after `set_parking_brake` restored
  the ratio with a raw write (valve left behind, "restored" on the write
  alone); it now drives the seam and says restored only on the read-back.
- Left: the steered push's "Stopping the push. Brakes on" is said before the
  push loop sets the brake (the loop owns it, `pushback.rs`).

Tests `the_stop_says_brakes_on_only_when_the_parking_brake_reads_set`,
`a_parking_brake_is_reported_from_its_read_back` (0.03 through three drives
is not held). Next 737 landing to a stop: `autoland` `stopped_brakes`
`held: true`, `park` 1.0, and the next `state` records `park` 1.0. Offline
only.

## 2026-09-27: a top of descent belongs to a cruise (issue #68)

LSZS-LILO, SR22, 2026-09-24 (pilot flying, full auto's takeoff not in
charge, no follower): `callout` `tod_passed` "We're past the descent point.
About 300 feet a minute makes the profile." at t 1790265228.8, 51.3 ft AGL,
5,638 ft, vsi +730, five seconds after liftoff from Samedan's 5,600 ft
(the issue's "138 ft" is the `resume_refused` six seconds later, "we're 138
feet above the ground"). Why: the plan's ring (`monitor::DESTINATION`,
SimBrief's along-route `tod_to_dest_nm`) is compared with the STRAIGHT-line
distance to the field; the valley route to Sondrio (898 ft, 14 waypoints)
made the ring longer than the 23.6 nm straight line, so the aircraft was
"past the descent point" on the runway. The recorded 300 fpm solves to a
ring past 24.6 nm: (5,638 - 2,898) ft over (23.6 - 5) nm at 116 kt. The
check then had no climb gate at all; 2026-09-25 added `CLIMBING_FPM`
(silent above +300 fpm), which catches this tick but not a level moment in
the climb-out, nor a level-off low.

Fix (`src/monitor.rs`): `tod_in_cruise` gates `TodAhead` and `TodPassed` on
the flight model's coarse phase being `Cruise` (a level cruise, or a
descent still above 3,000 ft AGL, `coarse_phase`) and `agl_ft` at least
`TOD_MIN_AGL_FT` (`flight::INITIAL_CLIMB_AGL_FT`, 1,000 ft): an initial
climb is `Climb`, and a level held low becomes "levelled low" Cruise after
30 s, which the height keeps silent. `HighOnProfile` takes `Cruise` or
`Approach` and the same height. The existing gates (`follower_flying`,
`follower_tod`, approach, `CLIMBING_FPM`) stand. Test
`a_climb_out_under_a_ring_behind_the_departure_calls_no_top_of_descent`
replays the sortie's state records (roll to 139 ft) through `Flight` with
LILO's apt.dat position and a 25 nm ring: silent throughout, silent at
139 ft level for an instant and for 40 s, and a 10,400 ft cruise 22 nm out
still calls. `high_on_profile_is_a_descent_from_the_cruise` pins the phases.

Not touched: the ring is still along-route against a straight line, so a
winding route calls its TOD early in the cruise too. Next flight, check:
no `callout` `tod_ahead`/`tod_passed`/`high_on_profile` whose preceding
`state` has `fphase` other than Cruise/Descent or `agl` under 1,000.
Offline only.

## 2026-09-27: a tower switch moves the one arrival runway (#64)

The arrival-procedure brief (2026-09-26) made `full_auto::ARRIVAL_RUNWAY`
the one value carrying the runway from the arm to the landing, but left the
seam where autoland obeys the tower: `clearance_switch` (on the approach)
and `runway_from_clearance` (at autoland's arm) re-armed or flew the
cleared end while `arrival_runway()` stayed on the old one, so every reader
after the switch (the inbound call, the flight-test card's `own_star` row)
read a runway nobody was flying.

Fix: `full_auto::retarget_arrival_runway(rwy, why)` is THE write point for a
change after the arm. It moves a runway the mission holds and never invents
one (no mission, a circuit, a plan with no runway: `None` stays `None`, so a
standalone autoland leaves nothing stale), blackboxed `full_auto`
`arrival_runway` {was, now, why}. Autoland calls it at the in-flight switch
(why `clearance`, before the new loop arms, whose `ils::tune` then tunes the
same end) and at every arm (`autoland::armed_runway_why`: `clearance` when
the tower's end won at the arm, `self_pick` when the forced end was missing,
`arm` with no change). The owed in-flight re-pick (#79) writes there too.
The card's `own_star` row passes a landing on the tower's end when the
`arrival_runway` record says `clearance`. `ils::tune_from` takes a named
nav file so the retune is testable. Test
`a_clearance_switch_moves_the_arrival_runway_and_the_ils`: 12 to 30 on a
cleared-to-land, NAV1 to the 30's frequency and course; a go-around moves
nothing; the mission's end clears it.

Next flight with a runway switch, check: `full_auto` `arrival_runway` with
why `clearance` and `now` the cleared end, then `autoland`
`runway_from_clearance`, `ils` `tuned` rwy the new end, `taxi_in`
`vacating` rwy the same. Offline only.

## 2026-09-27: a dip in the tug's swing rate is not the end of the swing (issue #62)

LIBP, sortie 1790429680, 13:38:44Z 2026-09-26: a Nose 90 left, plan 55.3 m
to 217.4. The tug swung the nose at 1.2 deg/s through the plan's distance,
and at 60.9 m, err 12.6, `done` saw the push finished and the swing over,
and the loop said "stop". The swing test was `SWING_DEG` (2 degrees) over
`SWING_WINDOW_S` (2 s), a 1 deg/s threshold standing in for "the swing is
over", and its window was trimmed to samples NEWER than two seconds, so at
the 0.55 s tick it spanned 1.7 s: 14.6 to 12.6 read as under two degrees.
The tug took no notice: over the next 31 s (`state` records) it swung the
nose on to 262, dipping to 0.7 deg/s on the way but never turning less than
3.2 degrees in any four seconds, stood still, and let go 4 s later. The
only other mid-swing stop in the log is the stall clock's: LIPK 1790408535
said `tug_finished` with the nose turning 3.5 deg/s (back_m had stopped
growing, the heading had not).

Fix (`pushback::SwingWatch`, `PushJudge`): one pure judgement over the push
ticks. A swing starts on `SWING_DEG` over a span of AT LEAST
`SWING_WINDOW_S` (measured, never assumed from the trim) and ends only when
the nose has turned under `SWING_STILL_DEG` (1 degree) over
`SWING_SETTLE_S` (4 s). While the bar is on and it says swinging, neither
`done`, the straight/lane finish nor the stall clock may end the push (a
nose being swung is the tug moving us), so "stop" goes out at the tug's own
stop: the bar off, or 12 s of true stillness. The pavement edge still stops
at once (safety, logs `swinging`); every `push` tick now carries `swinging`. Tests replay the recorded LIBP ticks
(the old rule stops mid-swing; the new one ends at the release at +34 s,
or with the bar left on at the stall 12 s after the nose stood still), and
the LIEA straight push still stops on its recorded tick.

Next push, check: `swing_wait` then `tug_finished` or the release, never
`stopped` while the `state` heading is still moving; the LIPK-style
3.5 deg/s stall stop is gone, so the tug finishes its pattern past the
lane there (the taxi owns the residual). Offline only.
