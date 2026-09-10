# Deterministic crew conversation

The conversation system runs with `XCOPILOT_LLM=off` (the default).
It uses no model, embeddings, cloud service, new dependency, or background
simulator polling. Whisper transcription and the existing speech voices remain.

## Supported exchanges

| Pilot | Behavior |
|---|---|
| “What's our altitude?” → “How do you know?” | Recalls the simulator answer as a historical reading. |
| “And now?” / “Check that again” | Rechecks the supported reading through the existing state-query path, bypassing the pushed cache. |
| “And heading?” / “And fuel?” / “And flaps?” | Changes the topic to a read-only question. |
| “What's our speed?” → “Ground speed” | Clarifies the speed type and reads groundspeed, converted to knots. |
| A command → “Why?” / “What did you just do?” | Repeats the actual report, including refusals, errors, and asynchronous-start messages. It does not invent a reason or claim completion. |
| A reference-speed question → “Why that speed?” | Attributes the figure to the active aircraft profile, not today's performance calculation. |
| “And best glide?” / “And blue line?” | Consults that profile. An absent figure is reported as unavailable. |
| “What are you managing?” / “Why are we waiting?” | Reports the existing automation status. It does not infer clearance or AP engagement from an empty manager list. |
| “How do your checklists work?” → “Tell me more” | Gives authored explanations of the product's checklist behavior. It does not run a checklist. |
| “Keep the explanations shorter from now on” | Changes conversational detail for the session. Tool readbacks and safety callouts remain complete. |
| “Forget the question” | Drops the conversation topic and pending clarification; it does not cancel flight automation. |
| “Could you please tell me our altitude and heading?” | Combines up to four supported reading subjects; reads run concurrently. |
| “Tell me about our oil pressure” / “Check the battery voltage” | Composes question openers with supported system names, without needing a separate whole-sentence alias. |
| “Back to altitude” → “And now?” | Recalls the most recent altitude report, then requests a fresh reading. |
| “What was my last request?” | Recalls the last recorded action request and its report, even after discussing another subject. |
| “Recap our conversation” | Summarizes up to four recent records, explicitly as historical reports. |
| “Check that again” with no topic → “Oil pressure” | Asks which reading, then resolves the named read-only subject. |
| “Walk me through the before takeoff checklist” | Previews the actual loaded checklist, one item at a time, without running or verifying it. |
| “Next item” / “Previous item” | Navigates the active preview. It never marks an item complete. |
| “Pause the preview” → another question → “Resume the preview” | Returns to the saved item. A different command also suspends the preview. |
| “Preview a checklist” → “Before Takeoff” | Offers available titles and accepts a title selection for preview only. |

Supported reading subjects include altitude, heading, airspeed, groundspeed,
vertical speed, fuel quantity, endurance, flap position, gear handle, parking
brake, wind, weather, visibility, outside temperature, throttle, mixture, pitot
heat, oil pressure, oil temperature, fuel flow, and battery voltage. Each maps
to an existing read-only reporter. Questions must contain complete supported
subject phrases: unknown words and action values are not silently discarded.

Other authored topics explain how command matching works, why clarification is
needed, and how to request control back. The conversational phrase catalog is
deliberately explicit. Unrecognized speech retains the existing command/question
routing; enabling the optional legacy answer fallback is a separate choice.

## Boundaries

`src/conversation.rs` is a pure state machine with explicit time input. Its
response enum can return speech, canonical state questions, a profile reference
question, a checklist preview request, or current automation status. It cannot return a tool
call or access the simulator. The session handles those read-only responses
before the spoken domain gate, after physical radio routing and urgent control
handling. New commands continue through the existing grammar and tool gates.

Conversation evidence distinguishes observations, action reports, authored
knowledge, and live automation status. `Ok(String)` from a tool is never treated
as proof of success: existing tools also use it for safety refusals. Compound
command reports retain all returned readbacks. This is an evidence record, not
yet a structured reason-code migration of every tool.

Up to 12 recent evidence records are retained for five minutes. Unrelated
utterances retire the immediate topic, while explicit “back to altitude” can
retrieve a previous record. Pending reading and checklist-title clarifications
expire after 30 seconds. A preview expires after five minutes without advancing
or repeating an item. Acknowledgments and detail preferences retain the topic.
Airframe changes clear topics, history, preview position, pending questions,
and the older numeric command history; the session's detail preference survives.
The independent action-reference stack keeps its existing 120-second contract.

Historical readings are labeled as historical. Supported “check that again”
queries use REST rather than potentially old WebSocket cache values; they still
benefit from the simulator client's short value cache. Unsupported recheck
subjects ask the pilot to name the reading. No command is repeated by this path.

Conversation responses appear in the transcript and as `conversation` events in
the blackbox log. This increment speaks only when addressed; it adds no proactive
advisories and no additional tick-time load.

## Validation and subsequent increments

`cargo test --workspace` exercises scripted follow-ups, real aircraft reference
answers, command fall-through, refusals, stale context, airframe resets, and
adversarial phrases. Live voice timing and in-sim interruption behavior still
need a flight check.

This establishes conversational continuity and read-only checklist walkthroughs;
it is not the complete crew model.
Further increments can add structured reason codes from individual tools and
automation stages, explicit task/deferred-intent tracking, resumable execution of
checklists (distinct from the delivered previews), more curated aircraft
knowledge, and workload-aware advisory policy.
Each needs its own scenario tests and must retain the same no-conversational-
actuation boundary. A stored plan or an explanation is never authorization to
arm an automation.
