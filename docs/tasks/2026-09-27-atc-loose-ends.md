# Task: four ATC loose ends from the history

Read docs/tasks/COMMON.md first, then CLAUDE.md's "ATC: the world, not the
crew" section. Each item below has its evidence in docs/dev-history.md at the
line given (line numbers as of 2026-09-27); read the whole entry around it
before changing anything. Verify each item is STILL broken before fixing it:
a later entry may have fixed part of it. If one is already fixed, say so in
the report and move on.

## A. The region handoff readback goes into the void (history line ~3630, 2026-09-15)

The region switch uninstalls the old station a tick before the crew's readback
("roger, Granada Tower on one one eight decimal ...") is keyed on the OLD
frequency, so the call is judged `void`. CLAUDE.md now says "within 45 s of a
region switch it is `handoff_readback`, not `void`" (`frequency_readback`),
so check the newest sortie logs (`grep -c '"outcome":"void"'` on
`logs/sorties/*.jsonl` and the region handoff events near them) whether the
void still happens. If it does, the fix is in `station.rs`: the old station's
readback expectation survives the switch for one call, or the crew keys the
readback BEFORE the swap (CLAUDE.md: "a crew call is routed on the frequency
it was keyed on, before it is spoken"). Test: a region handoff followed by the
readback, outcome `handoff_readback`.

## B. The tower learns its runway is free from the taxi request, not the vacated report (history line ~5700, 2026-09-22)

`Intent::RunwayVacated` is a report that spends nothing (correct), but the
tower's runway occupancy for OUR aircraft is only cleared by the following
`RequestTaxiIn`. A real tower frees the runway on "runway vacated". Make the
vacated report clear the strip for the departure release and the arrival
stream (grep `TAKEOFF_HOLD`, `line_up_condition_clear`, `inbound_final_for_strip`,
`State::` transitions after a landing), keep the taxi-in request as today
(still the taxi in, still not a departure). Test in the tower matrix: after
"cleared to land" then "runway vacated", a second aircraft's takeoff request
is released without waiting for our taxi request; without the vacated report
the old behaviour stands.

## C. The takeoff never checks the transponder (history line ~4640, 2026-09-21)

Two "negative radar contact, recycle transponder" calls went unanswered. Full
auto already checks (`src/full_auto.rs` `squawk_owed` about line 514) but the
spoken "take off" path (`src/autotakeoff.rs`) and the circuit path do not.
Build ONE check used by every path that calls ready for departure: mode ALT
(or the airframe's equivalent through the declared knob, `radio::set_mode`,
never a raw write) and the assigned code set and READ BACK, before the call;
and when the tower says recycle, the crew recycles (`SetTransponder`), says
so, and calls again. Find why the tower said it twice (does the station's
`SquawkCheck` watch see mode C only above a height, or the code only?). Test:
a takeoff request with the wrong code earns the recycle line; the crew's
recycle earns "radar contact".

## D. The line-up refusal named the reciprocal end, and a short cleared runway is never refused (history line ~5830, 2026-09-24)

1. The crew's line-up refusal ("08L is 152 metres away") named the runway the
   crew was near, not the one it was cleared for. The refusal must name the
   CLEARED runway (`party_line::clearance_runway`) and the distance to ITS
   entrance. Test on the recorded numbers.
2. At the taxi clearance the crew never judges the cleared runway's LENGTH
   against the airframe. Use the ONE rule (`aircraft::takeoff_needed_m`,
   CLAUDE.md "Enough runway is ONE rule") at the taxi call: a runway that does
   not fit is refused aloud with both numbers ("runway 26 is 446 metres, we
   need 780") and the crew REQUESTS a longer one: a new radio intent
   `Intent::RequestRunway { rwy }` in `tower.rs` ("request runway 08L",
   "unable 26, request 08L for length"), granted when the runway exists and is
   assignable (`nav::assignable_runways`), the taxi route re-planned to it
   (`RequestTaxi` already does this for the runway in use), refused with the
   reason otherwise. Full auto asks for the longest assignable runway that
   fits; a pilot taxiing hears the refusal and the suggested request through
   the coach. Corpus entries for every phrasing. Tests: tower matrix grant and
   refusal; full auto refuses the 446 m runway and requests the longer one;
   a fitting runway is taxied as today.

## Tests and report

Every item ships with the test named above. Report per item: still broken or
already fixed, what changed (files, functions), the test, and the blackbox
event that proves it on the next sortie.

## Rules that bite here

- The station returns text and can NEVER actuate; the crew reads the world
  through `party_line` accessors only.
- A mishear may earn "say again", never a clearance.
- No em or en dashes in any string. Do not touch the sim.
