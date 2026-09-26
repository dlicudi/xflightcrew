# Task: "put us on approach" is one command, and the copilot takes the aircraft as it lands

Read docs/tasks/COMMON.md first.

## The evidence (LEMG, 737, 2026-09-26 evening)

The plugin's flight-init mailbox (`xcopilot/sim/init_flight`, JSON in the
sim's own XPLMInitFlight format, `plugin/src/flightinit.cpp`;
`src/tools/setup.rs` is the app side, today only a cold-and-dark ramp
start from the plan) was driven by hand tonight with

```
{"aircraft":{"path":"Aircraft/Laminar Research/Boeing 737-800/b738.acf"},
 "runway_start":{"airport_id":"LEMG","runway":"13","final_distance_in_nautical_miles":10},
 "engine_status":{"all_engines":{"running":true}}}
```

It worked (seq 1, result 0 in 6 s; the SDK doc for the format is
https://developer.x-plane.com/article/flight-initialization-api/ and
`~/Downloads/SDK/CHeaders/XPLM/XPLMPlanes.h`). But X-Plane places the
aircraft with NOBODY flying: autopilot off, 180 to 210 kt, sinking 1,000
to 3,500 fpm on a 10 nm final that crosses 2,300 ft hills. The copilot
called "Airspeed!", "Overspeed!", "Sink rate! 3270 feet a minute at 1331
feet", proposed the descent checklist, and did nothing, because in `both`
mode it has no hands until asked. A typed "recover the aircraft" caught it
at 159 ft AGL, and the pilot's spoken "land the aircraft" then flew a
landing. Blackbox: `logs/blackbox.jsonl` t=1790448105 (the `flight`
`reposition` event) to 1790448170. The typed command was written to
`xcopilot/typed` about 15 s before its `tool` record (t=1790448150.9):
the typed queue is consumed one line per main-loop pass and a pass that
speaks blocks.

## What to build

1. **A spoken/typed tool** on the existing setup page (`src/tools/setup.rs`,
   grammar in `src/grammar.rs`, the same shapes the pilot uses: "put us on
   approach to LEMG 13", "set us up on a 10 mile final to runway 13",
   "start us on final for one three", "put us on a 5 mile final", the
   field defaulting to the plan's destination, then the nearest field, the
   distance defaulting to 10 nm, the runway to the tower's or the plan's or
   the wind's): writes the JSON above with the CURRENT aircraft
   (`acf_relative_path`), engines running, and waits on `init_seq` /
   `init_result` as `follow_up` does (the result codes are in the SDK
   header: 4 missing airport, 6 missing runway; name them in the spoken
   line).
2. **The copilot takes the aircraft the moment the flight exists**: on
   result 0, arm the autoland FORCED onto that runway (the way
   `EndAction::Approach { rwy_id }` hands `run_autoland` its runway) with
   no typed-queue hop, and say so ("On a 10 mile final for one three, I
   have the aircraft"). If the pilot wants to hand-fly the approach, the
   words say so ("put us on final, my controls" / "...and I'll fly it")
   and the copilot only monitors. Tonight the aircraft flew 40 s with no
   hands; the gap must be zero.
3. **The tower knows**: the flight is a new arrival, so the station's
   state for us must be reset the way an airframe swap resets it (grep
   `Station::install`, `party_line`, `flight::note` for what a reposition
   clears; the flight model already logs `reposition`), and the inbound
   call goes out so the landing is cleared (tonight's landing earned "you
   landed without clearance"). A runway start at the numbers (no
   `final_distance`) must keep the existing runway-start behaviour.
4. **Typed-queue latency**: find why a typed line waited about 15 s for
   its pass and fix it if the cause is the loop blocking on speech (a
   typed emergency word must not wait for a sentence to finish). Measure
   with a test if you can; report the cause either way.
5. Panel: a request phrase in `REQUEST_PHRASES` only if there is a free
   button; otherwise the typed line is enough.

Tests: the JSON builder pure and tested (airport, runway, distance,
engines), the grammar shapes pinned, the result-code lines pinned, and a
test that the arm follows the result with the forced runway. Do not
change any flight law. Report as COMMON.md says.
