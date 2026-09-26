# Task: the 737 touches down at 400 to 770 fpm after five flare changes

Read docs/tasks/COMMON.md first.

## The evidence

Five commits on 2026-09-26 changed the 737's flare (exponential sink target,
retard by height, the flare starting from the roundout's pull, holding the
gentlest sink reached, softer pull-only arrest at gain 0.6). The landings
flown AFTER them (LEMG, 737-800, from the debrief; `TOUCHDOWN_MAX_SINK_FPM`
is 400):

| time Z | sink fpm | IAS kt |
|---|---|---|
| 16:29 | 445 | 138 |
| 17:04 | 278 | 138 |
| 17:17 | 537 | 143 |
| 17:25 | 769 | 145 |
| 17:32 | 411 | 142 |
| 17:43 | 390 | 142 |

Only one is inside the limit and it is not repeatable. The touchdown speed
is 10 kt fast every time (the debrief's "airspeed 10 knots fast" on the
final). The profile is `profiles/boeing_737_800.yaml` (`flare_elevator:`
gain 0.6, max 0.45, max_push 0, `retard_below_ft` 20, `arrest_lead:`).

Records: `autoland` per tick (`cmd_vs`, `vs_dial`, `vvi`, `vvi_trend`,
`arrest_hands`, `committed_flare`, elevator written, throttle), `state` with
`in`, `stage_result` landing. Sortie `1790441293-full_auto` has the 17:04
landing; the 17:17 to 17:43 landings are in the main blackbox untagged.

CLAUDE.md's "Still owed" lines that bear on this: "the retard is keyed on the
sink the arrest is removing, so it fires latest exactly when it should
already be done", and "a go-around law that pitches for SPEED when a balk is
slow and low". Read the autoland sections of CLAUDE.md and grep
dev-history.md for "flare" before changing the law.

## What to do

1. Overlay the six flares tick by tick (height, vvi, cmd, elevator, thrust)
   and name what differs between the 278 fpm landing and the 769 fpm one.
   The suspects, in order: the retard timing against the arrest (thrust
   leaving during the pull), the exponential target's 4 s constant against
   the 737's VS lag, the pull-only cap starving the arrest when the sink is
   already high at the gate, and the approach 10 kt fast changing the
   roundout's entry.
2. The `autoland` sortie tests fly a second-order 737 plant with dead time
   through the real law. Fit the plant to the recorded response first (the
   elevator written vs the vvi that followed); if the recorded response does
   not match the test's plant, the test is the first fix. Then reproduce the
   769 fpm landing in that test and make the law pass the 400 fpm limit on
   ALL six recorded entries (height, sink, speed at the roundout gate), not
   only the average.
3. The approach speed: find why the final holds 10 kt over the profile's
   approach speed (the speed floor for the flaps actually out, from today's
   commit, or the governor's target) and fix the number at its source.
4. Keep the other airframes byte-identical: their flat flare stays.
5. Do not touch the rollout or the ground steering (another agent owns the
   lateral stages) and do not edit `scorecard.rs`.

Report: the per-landing diagnosis table, the plant fit, the change, the
test, and which `autoland` fields prove it next sortie.
