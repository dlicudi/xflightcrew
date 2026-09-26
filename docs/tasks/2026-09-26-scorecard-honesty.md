# Task: make the sortie scorecard trustworthy

Read docs/tasks/COMMON.md first.

The scorecard (`src/scorecard.rs`, `sortie_result` / `stage_result`, the
Scorecard block of `xvl debrief`) is the project's only pass/fail signal for
unattended testing, and today every sortie fails, partly for wrong reasons.
A verdict that is wrong is worse than none: it hides the real failures.

## Known wrong

1. **The crew's own flare is counted as a pilot intervention.**
   `monitor::note_interventions` (`src/monitor.rs` around line 2170) flags
   `yoke_under_ap` when the yoke pitch exceeds 0.15 while an automation
   flies with `ap_on`. The autoland's arrest writes the elevator through
   `xcopilot/pitch` with the servos off but the AP flag still reads on, so
   every 737 landing logs two interventions (yp 0.21 to 0.45, exactly the
   flare's pull) and the sortie fails on `interventions > 0`. See the
   `intervention` records in `logs/sorties/1790441293-full_auto.jsonl` at
   t=1790442273.5 and 1790442279.2 against the `autoland` ticks of the same
   seconds. An intervention is a hand that is not ours: exclude the crew's
   own channel writes (the elevator, roll and steer channels while the loop
   that owns them is in its arrest or roll), and prove it with a test.
2. **The debrief prints Rust debug output**: `failed Array [String("taxi")]`.
   Print the stage names.
3. **Stage records outside a mission lack the `sortie` tag** and still appear
   under the debrief's Scorecard block, interleaved with the missions'
   verdicts, so the reader cannot tell a mission's landing from a stand-alone
   autoland. Group them by mission and label the untagged ones.

## To verify, then fix or leave alone

4. **The lateral limits versus what is measured.** Taxi 3 m, line-up 2 m,
   roll 5 m, rollout 5 m. Read how `max_lat_m` and the takeoff's `xtk_m` are
   sampled: are they measured against the segment being driven, do they
   include a stand lead-in, a pushback, a backtrack turn or a projection jump?
   Another agent is fixing the driving laws (docs/tasks/2026-09-26-ground-
   lateral.md) and is told NOT to edit `scorecard.rs`; you own the metric.
   If the metric samples something the law is not steering to, fix the
   sampling, not the limit. Do not loosen a limit to make a sortie pass; if a
   limit is genuinely unrealistic for the airframe (a 737 on a 23 m taxiway
   versus a 172), make it per airframe from the profile with the current
   number as the default and say so.
5. **A replay test.** `flight::Replay` replays recorded ticks. Add a test
   that feeds a trimmed sortie file (a few hundred records, committed under
   `bench/` or the test's data) through the scorecard and pins today's
   verdicts, so any change to the scorecard is visible in `cargo test`. Then
   `xvl debrief <sortie file>` and the test agree by construction.
6. **`unattended`**: check what sets it and that it means what the name says.

Report each item: what was wrong (with the record that shows it), the fix,
the test.
