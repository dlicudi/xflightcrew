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
`bench/atc_corpus.yaml`, which `cargo test` then enforces forever.

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

For each wart: add the exact transcript to `bench/atc_corpus.yaml` with the
intent it SHOULD have parsed to (or `unknown` if "say again" was right), run
`cargo test`, and fix the parser until the corpus passes. Real mishears from
the logs beat invented ones; that is the whole point of the trace.

Known gaps, documented in the corpus as `unknown` until the intents exist:
flight following, downwind/base position reports, go-around calls.
