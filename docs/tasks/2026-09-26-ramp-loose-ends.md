# Task: the ramp's loose ends (battery at shutdown, tug never clear, duty engine)

Read docs/tasks/COMMON.md first. The flight-test card
(`docs/flight-test-card.md`, rows `stand_shutdown`, `tug_unhook`,
`duty_engine`) found these from the 2026-09-26 logs (live repo
`logs/blackbox.jsonl` and `logs/sorties/`).

## 1. The mission's shutdown left the battery on (`stand_shutdown`)

17:07Z, 737 at the stand: engines stopped, then "All electrical systems
off, except: The battery didn't move." (`taxi_in` `electrics_off` `line`).
Find which dataref/command the shutdown drives for the 737's battery and
why the readback did not agree (the stock 737 has two battery switches or
a guarded one; the profile's `shutdown:` / electrics list; a stock write
the aircraft's own logic rewrites; CLAUDE.md's "a dataref name may really
be a command; read-before-fire toggle tools"). Fix it in the registry's
shutdown for the class with a readback, and keep the honest line for a
switch that still does not move.

## 2. The tug never showed clear (`tug_unhook`)

13:39Z push: `pushback` `brakes_set` with `tug_clear: false`, and a
`tug_not_clear` event. Today's commit 327d121 made the toe brakes hold
until the tug is off the bar, then the parking brake. Read the push's
ticks (`pushback_attached`, the tug distance if logged, the timing) and
find whether the tug really never cleared, the clear test reads the wrong
thing, or the wait timed out. Fix the test or the wait; a tug that is
genuinely still attached must keep the toe brakes (never a parking brake
against a tug), said aloud once.

## 3. The duty engine disagrees 13 times in 124 (`duty_engine`)

`src/duty.rs` is SHADOW ONLY (it acts on nothing) and earns the right to
act once its logs agree with flights flown. `duty` records carry `duty`,
`phase`, `why`, `agrees`. List the 13 disagreements (line_up, taxi,
taxi_to_stand, vacate) with the running automation's stage at that tick
and decide each: the duty engine is wrong (fix its rule, with the recorded
state as the test), or the automation is wrong (report it, do not change
the automation), or the comparison is unfair (a transition tick, a
stage name mismatch: fix the comparison). Report the tally.

Do not touch autoland.rs, setup.rs, procedure.rs or plan_audit.rs (other
agents are in them). Report as COMMON.md says.
