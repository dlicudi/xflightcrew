# Task: the ground stages fail on lateral error, every sortie

Read docs/tasks/COMMON.md first.

## The evidence

Eight full-auto sorties on 2026-09-26 (LIBP to LEMG, a 737-800, then a 172)
and not one passed a ground stage. `scorecard.rs` limits: taxi 3 m off the
line, line-up 2 m, roll 5 m, rollout 5 m. Measured (blackbox `stage_result`):

| stage | sorties (max_lat_m or xtk_m) |
|---|---|
| taxi | 14.5, 14.9, 15.0, 9.5, 5.2, 4.0, 3.8, 3.5, 3.3 |
| line_up | 7.5, 7.2, 6.7, 6.0, 3.1 |
| takeoff (liftoff xtk) | 9.9, 9.7 (reject), 6.9, 6.3 |
| landing rollout | 18.1, 16.8, 15.6, 13.6, 12.4, 8.7, 8.0 |

The taxi's ~14.5 to 15 m figure recurs at the same magnitude across four
sorties from the same origin: it is one place on the route, not noise. The
auto taxi also "rolled to a stop 3 times before its holding point" (debrief).
The rollout figure has grown across the day's landings (8 to 18 m) while the
takeoff figure stayed 6 to 10 m.

Sortie files: `1790441293-full_auto`, `1790429680-full_auto`,
`1790426786-full_auto`, `1790416823-full_auto`, `1790414806-full_auto`,
`1790410371-full_auto`, `1790408844-full_auto` (the last two sorties of the
day also have stand-alone `autoland` landings with no sortie tag in the
main log, 17:17 to 17:43Z).

Records: `autotaxi` per tick (the drive), `taxi_guide` (the projection),
`autotakeoff` per tick (`rollout`-style fields: xtk, heading error, rudder),
`autoland` per tick including the `rollout` event, `state` at 1 Hz with
`in` (throttle, brakes, yoke, yields) and `ctl`.

## What to do

1. For EACH stage, locate the ticks that produced the maximum and say what
   happened in the aircraft's terms: where on the route, the aim point, the
   cross-track sign, the steer command, the yield latches, the groundspeed.
   Decide whether the number is a real excursion or a measurement artefact
   (a projection jump, a stand lead-in, a backtrack leg, the pushback,
   sampling from the wrong segment). CLAUDE.md's "A jump in a projection is
   never a movement of the aircraft" and "Routes ride the paint" are the
   history here; the scorecard's `max_lat_m` must measure the aircraft
   against the segment it is actually driving.
2. Fix the cause in the law that owns it, never in the scorecard's limit
   (a different agent owns `scorecard.rs`; if you conclude the METRIC is
   wrong, say so in your report with the tick that proves it and do not edit
   `scorecard.rs`). Owners: `autotaxi.rs` (pure pursuit, `bend_lookahead_cap_m`,
   the taxi governor), `taxi_guide.rs` (`project_signed_facing`),
   `autotakeoff.rs` (`roll_target_heading`, `RollSteer`, `XTK_LEAD_S`, the
   smoothed cross-track rate from today's commit), `autoland.rs` rollout
   (shares `roll_target_heading`; `disconnect_autopilot` at the wheels; the
   crab law with the servos out from today's commit f107203).
3. The rollout: five 737 landings today all rolled out 12 to 18 m off with
   the yoke recorded under the AP (`intervention` `yoke_under_ap`, yp 0.2 to
   0.45, which is the flare's own elevator on `xcopilot/pitch`). Check whether
   the roll channel (`xcopilot/roll`) or the rudder is still being written
   after touchdown, whether the heading servo is really off at the latch, and
   whether the crab (a wind correction) is still added on the ground.
4. Each fix ships with a test over the recorded ticks (copy the minimum
   window of records into the test as data, not the whole file) or a
   second-order plant, asserting the scorecard number the stage would report.
5. Do not tune gains until the `dt` in the per-tick records is understood
   (a 300 ms tick plus two round trips is the real transport delay).

Report per stage: cause, fix, test, and the blackbox field that will show
the improvement on the next sortie.
