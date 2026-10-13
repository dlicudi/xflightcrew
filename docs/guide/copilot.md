# The copilot

## Commands

The copilot works the systems the loaded aircraft has: flaps, gear,
autopilot, engines, fuel, electrics, lights, radios, transponder and
instruments. A 172 is never offered a gear lever.

- Commands chain: "flaps ten and throttle seventy".
- Follow-ups refer to the last action: "make it eighty", "a bit more",
  "do that again".
- Unsafe commands are refused with the reason: gear up on the ground, flaps
  over Vfe, an autopilot altitude below the terrain.
- Every write is read back from the aircraft. If a switch did not move, the
  copilot says so instead of "roger".
- With `XCOPILOT_CONFIRM=true` (the default), safety-critical commands wait
  for "affirm".

## Questions

Ask about the aircraft's state ("what's our airspeed?", "how much fuel?")
and follow up with "and now?", "and heading?" or "why?". A judged question
("are we too fast?") gets a verdict, not just a number. Weather at another
field comes from the METAR. See
[deterministic-copilot.md](../deterministic-copilot.md) for the full range.

## Checklists

Checklists come from the aircraft's XChecklist file, where its profile names
one. "Run the before takeoff checklist" reads each item in turn: it sets the
items it can, checks the ones it can read and calls out the ones that are
yours. "Talk me through the before landing checklist" reads them without
touching anything.

The panel's Checklist tab shows every list for the aircraft, with each item
ticked live as the switches move. Run starts the list as if you had asked
for it.

When an automation is flying, the copilot runs the matching list silently as
a cross-check and tells you only what is not as listed.

## Watching

A background monitor speaks up about problems a second pilot would catch: a
failed alternator, low fuel, the gear, a stall, terrain, an altitude
deviation. `XCOPILOT_MONITOR=false` turns it off. See
[monitoring.md](../monitoring.md).

## Flight plans

- "Plan a flight to Malaga" builds a route from where you are, with a cruise
  altitude above the terrain and fuel and time from the aircraft profile. It
  refuses a destination runway that is too short.
- With `XCOPILOT_SIMBRIEF_ID` set, "fetch the plan" gets your SimBrief
  plan.
- "Load it" puts the plan in the FMS.

## Letting it fly

| Say | It does |
|---|---|
| "push back left" | Calls a tug and pushes onto the taxiway |
| "taxi for me" | Taxis the cleared route to the holding point |
| "line us up" | Lines up on the centreline |
| "take off for me" | Takes off and climbs to 1,000 ft |
| "do a circuit" | Flies a circuit to a landing |
| "fly the ILS" | Flies the published approach |
| "land us" | Lands from wherever you are |
| "fly my flight plan" | Flies the loaded plan, gate to gate |

"My controls" hands the aircraft back at any point. Where a manoeuvre is not
possible the copilot says so with the reason.

On approach the aircraft's own autopilot flies down to minimums. From there
the copilot hand-flies to touchdown with one controller for every fixed-wing
type. What differs between aircraft, such as how strongly each one responds
to the controls, is measured in flight rather than entered by hand.

## Cabin crew

On aircraft that carry cabin crew, "cabin crew, prepare for landing" gets an
acknowledgement and a "cabin secure" report when they are ready. They also
work the flight unasked, from the seat belt signs to the arrival.
