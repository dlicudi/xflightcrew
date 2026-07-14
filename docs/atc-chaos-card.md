# ATC chaos card

A kneeboard checklist for soak-testing the ATC in the sim. Everything below is
hostile on purpose: the goal is to catch the copilot or a controller doing
something a real one never would. Fly it occasionally, and after any change to
`tower.rs` / `station.rs` routing.

The one safety property that outranks everything: **a mishear may earn
"say again"; it must never earn a clearance you didn't ask for.** A missed
valid call is a wart. An invented clearance is a bug.

Every routing decision is recorded to `logs/blackbox.jsonl` as a `"kind":"atc"`
event (transcript, gate outcome, intent, state transition, reply), so each
sortie leaves a reviewable trace. Warts found here become cases in
`crates/atc/bench/atc_corpus.yaml`, which `cargo test` then enforces forever.

## Preflight

Build and launch as usual, then keep a trace running in a spare terminal:

```
tail -f logs/blackbox.jsonl | grep '"atc"'
```

## The card

Work through these in one or two flights. Tick, and note anything odd.

### Garble and dead air

| Do this | Expect |
|---|---|
| Key the radio PTT and cough | "Station calling, say again." (never a clearance) |
| Key and say nothing, release | Nothing, or "say again" |
| Mumble a half call ("uhh tower... something") | "say again", state unchanged |
| Call with a wrong or mangled station name ("Chipotle tower, radio check") | Normal reply; the mangled name must not block routing |
| Call a DIFFERENT airport by name ("Malaga tower, do you copy") | Reply from the tuned station (it answers its frequency, not its name) |
| Transmit on an empty frequency | "No answer" plus the comm plan, once; short "no answer" on repeats |

### Crew talk vs the radio

| Do this | Expect |
|---|---|
| Unkeyed: "anything else we need to do before takeoff?" | Copilot answers; NO tower reply |
| Unkeyed: "land the aircraft" | Copilot/panel path; NO join clearance |
| Unkeyed: "takeoff checklist" | Checklist runs; tower silent |
| KEYED: "set flaps for takeoff" (a known trap: cockpit command on the radio) | Worst case a taxi-first reply; note anything worse |
| Unkeyed: "what runway shall I take off from?" | Copilot answers; never CLEARED FOR TAKEOFF |
| Ask the copilot a question while COM1 is on the tower | Copilot answers in its own voice, tower stays quiet |

### Clearance discipline

| Do this | Expect |
|---|---|
| "Request takeoff" straight from the stand | Taxi instruction first, never a takeoff clearance |
| "Final" while parked on the apron | Landing clearance (documented: the tower trusts geometry); note if it feels wrong live |
| Read back every clearance you get, verbatim | Silence each time; no duplicate clearances, no "say again" |
| Read back with one number wrong | Today: silence (no correction logic yet); note how it feels |
| Ask for the squawk three times in a row | The code, three times, no attitude |
| "Request touch and go", then fly the full pattern | "Cleared touch and go" off the final report, not "cleared to land" |
| Land, then "request taxi" | "Taxi to parking", never the holding point |

### The frequency chain

| Do this | Expect |
|---|---|
| Depart normally, climb through ~700 ft | Handoff to departure, spoken exactly once |
| Check in with departure ("passing one thousand five hundred") | "Radar contact", once; later readbacks earn silence |
| Ask DEPARTURE for taxi | Redirect to the tower frequency, tower sequence unmoved |
| Ask APPROACH for takeoff | Redirect to the tower |
| Request vectors, then fly deliberately through final at 90 degrees | No approach clearance while crossing; a corrective heading instead |
| Ignore a vector for 60+ seconds | A repeated or corrected heading, not silence forever |
| Switch frequency mid-readback | Nothing weird on either frequency |

### Weather and the runway

| Do this | Expect |
|---|---|
| Fly in calm wind, request takeoff twice a minute apart | SAME runway both times (no calm-air runway flip) |
| Set a strong opposite wind between two calls | The runway changes, and the clearance says so |
| "Wind check" / "say QNH" / "how's the wind" | Wind and QNH, read from the sim, never invented |

### Emergencies and the pattern (added 2026-07-09)

| Do this | Expect |
|---|---|
| "Mayday, mayday" from any phase | "Roger mayday", cleared to land, emergency services — never "say again" |
| "Pan pan" / "declaring an emergency" | Same priority handling |
| Mayday to APPROACH or DEPARTURE | "Roger mayday" + vectors home, never "contact tower" |
| "Emergency landing checklist" (cockpit, unkeyed) | The copilot's checklist, NOT a mayday |
| Land, then "going around" mid-flare | Clearance voided, "report downwind", pattern re-sequences |
| "Downwind" then "turning base" then "final" | "Report base" / "report final" / cleared to land, in order |
| "Base" alone with no pattern context | Say again (too common a word to trust bare) |

### ATIS (added 2026-07-09)

| Do this | Expect |
|---|---|
| Tune the ATIS frequency | The broadcast within a tick, then a repeat ~45 s later |
| Stay tuned through a big QNH change | The letter advances (Bravo), numbers update |
| Sit through calm-wind wobble | The letter does NOT advance per broadcast |
| Call tower "with information <current>" | Clearances drop the QNH clause |
| Call with a STALE letter | Clearances keep the QNH (safety over brevity) |

### Enroute and traffic (added 2026-07-09)

| Do this | Expect |
|---|---|
| Fly toward another towered field 20+ nm away | One handoff ("radar services terminated, contact ... Tower on ..."), then the new tower answers calls |
| Orbit the midpoint between two fields | NO ping-pong handoffs (5 nm hysteresis + cooldown) |
| Stay on the runway/ground | Never a region handoff |
| With AI traffic on, fly near another aircraft while tuned to any controller | "Traffic, x o'clock, y miles" in that controller's voice, at most one per 45 s |
| Same but tuned off-frequency | Silence — nobody is watching |

### Squawks and compliance (added 2026-07-10)

| Do this | Expect |
|---|---|
| Request flight following (or vectors) | A discrete 45xx code assigned, "radar contact" ONLY after you actually set it (mode ON/ALT) |
| Ignore the assigned code for 20+ s | "Confirm squawk ...", twice at most, then the controller gives up |
| Ask "say my squawk" after an assignment | The ASSIGNED code, not seven thousand; before any assignment, seven thousand |
| Taxi onto the runway with only a taxi clearance | "You are on runway ... without clearance, exit immediately"; repeats each minute while you squat |
| Take off without a takeoff clearance (after calling in) | "You were not cleared for takeoff", once |
| Land without a landing clearance | "Taxi to parking, I have a number for you to copy", once |
| Land legally, roll out, vacate slowly | NO occupancy shout for ~2 min (vacate grace) |
| Fly through the zone (< 5 nm, < 3,000 AGL) without ever calling | Tuned: the controller challenges. Untuned: the COPILOT flags it with the tower frequency |
| Overfly the field above 3,000 AGL, radios cold | Silence — overflight is not an incursion |
| Request takeoff with AI traffic on final | "Hold position, traffic on x mile final", state unmoved; cleared on re-ask once final is clear |
| Report downwind with traffic on final | "Number two, follow the traffic on final" |

### Party line and traffic protection (added 2026-07-10)

| Do this | Expect |
|---|---|
| Sit tuned to the tower with LiveTraffic arriving | "Speedbird three zero six, runway ..., cleared to land" narrated BEFORE the ghost lands; each ghost's call once, ≥45 s apart |
| Watch a ghost depart | "..., cleared for takeoff" at rotation |
| `XCOPILOT_ATC_CHATTER=false` | Party line silent; go-around/hold watches still on |
| Tune off the tower frequency | No party line (it's what you hear ON frequency) |
| Get cleared to land, then a ghost taxis onto the runway | "Go around, traffic on the runway" (tuned) / crew "go around" (not tuned), repeats each 30 s while the conflict holds |
| Hold position for landing traffic, wait | ~15 s after the final truly clears: "traffic clear, runway ..., cleared for takeoff" unprompted — and the FSM/handoff arm as if asked |
| After a traffic advisory, let the target diverge | "clear of traffic" — the loop closes |

### Readbacks, line-up, wake, taxi routes (added 2026-07-11)

| Do this | Expect |
|---|---|
| Get cleared for takeoff, read it back with the runway | Silence — a correct readback is a quiet frequency |
| Get cleared for takeoff, say something else | "Read back: ... cleared for takeoff", the instruction repeated; twice at most, then the tower lets it go |
| Read back the WRONG runway ("cleared for takeoff runway 27" when given 09) | "Negative. I say again: ..." — never waved through |
| Get cleared to land, skip the readback, land anyway | The debt dies silently at touchdown (physics consumed it); no nagging on the rollout |
| Ask for takeoff with ONE arrival ≥1.5 nm final | "Behind the landing <telephony>, line up runway ... behind" — then the clearance comes unprompted once the lander vacates |
| Echo the conditional WITHOUT "behind" | Challenged — the condition is part of the clearance |
| Ask for takeoff with a STREAM on final (2+) | "Hold position" — a conditional is only legal against exactly one |
| Ask for takeoff right after an airliner ghost rotates off your strip | "Line up and wait ..., wake turbulence, two minutes behind the departing traffic"; cleared when the clock AND the final allow |
| Same, but the ghost was a GA tail | No wake clock — no airline prefix, no assumed weight class |
| Ask for taxi at a field with a taxi network (Heathrow) | "Taxi to holding point <name> runway ... via <Alpha, Bravo>", crossings called out ("cross runway ...") |
| Wander 60+ m off the cleared taxi route while rolling | "You appear to be off the cleared taxi route, taxi via ..." — a nudge each 45 s, not a violation |
| Line up as told, sit on the runway | NO occupancy shout — LinedUp is the tower's own instruction |
| Say "confirm cleared for takeoff?" | The instruction repeated (the challenge doubles as the answer), never a fresh clearance |
| `XCOPILOT_ATC_READBACK=false` | Challenges off; the coach still shows the owed readback |
| Ask "what do I say?" while a readback is owed | The exact readback line, before anything else |

## Post-flight

Pull the trace apart. Every command below reads `logs/blackbox.jsonl`.

All ATC events from the last flight:

```
jq -c 'select(.kind=="atc")' logs/blackbox.jsonl | tail -50
```

The false-accept hunt (the only bug class that matters). Everything that
earned a reply: read each transcript and ask "did the pilot actually request
that?":

```
jq -c 'select(.kind=="atc" and .outcome=="reply") | {transcript, intent, state, reply}' logs/blackbox.jsonl
```

The false-reject list (warts, corpus fodder). Radio-shaped words that fell
through to the copilot, plus keyed calls that got "say again":

```
jq -c 'select(.kind=="atc" and .outcome=="fallthrough")' logs/blackbox.jsonl
jq -c 'select(.kind=="atc" and .outcome=="reply" and (.reply | contains("say again"))) | .transcript' logs/blackbox.jsonl
```

Calls made while off frequency (should each have been a deliberate test):

```
jq -c 'select(.kind=="atc" and (.outcome=="off_frequency" or .outcome=="void"))' logs/blackbox.jsonl
```

## Closing the loop

For each wart: add the exact transcript to `crates/atc/bench/atc_corpus.yaml` with the
intent it SHOULD have parsed to (or `unknown` if "say again" was right), run
`cargo test`, and fix the parser until the corpus passes. Real mishears from
the logs beat invented ones; that is the whole point of the trace.

Known gaps, documented in the corpus as `unknown` until the intents exist:
(none currently — downwind/base reports, go-arounds and mayday landed
2026-07-09; greetings and flight following landed 2026-07-10.)
