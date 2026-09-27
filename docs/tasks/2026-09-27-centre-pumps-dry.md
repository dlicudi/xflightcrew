# Task: centre tank pumps left running dry

Read docs/tasks/COMMON.md first, then CLAUDE.md ("Core design rule", "A profile
line is a hypothesis until probed", "Unasked actions", and the altimeter rule
under "The follower's vertical and the altimeter", whose check-and-fix shape
this copies).

## The evidence (probed live, 2026-09-27, stock Laminar B738, engines running)

The pilot found both CTR LOW PRESSURE lights lit, CTR switches ON:

- `sim/flightmodel/weight/m_fuel` = `[2916.8, 0.0, 2918.1, 0...]`: tank 1 (the
  centre, `acf_tank_rat` `[0.187, 0.626, 0.187]`) EMPTY.
- `sim/cockpit2/fuel/fuel_tank_pump_on` = `[1,1,1,...]`. The pilot switched the
  CTR pumps off and element 1 read 0, lights out
  (`laminar/B738/annunciator/low_fuel_press_c1`/`_c2` = 0). The stock array IS
  the cockpit switch on this airframe; it is writable.
- The aircraft's xlua (`~/X-Plane 12/Aircraft/Laminar Research/Boeing
  737-800/plugins/xlua/scripts/`): `B738.Z.autostart.lua:2229` sets
  `fuel_tank_pump_on[1] = 1` during the scripted start WHATEVER the centre
  holds; `B738.systems.lua:3248` writes 0 only at cold-and-dark init. Nothing
  else rewrites it, so a write from us holds.
- Our code never touched it: the profile has `fuel_pump: false`
  (`profiles/boeing_737_800.yaml:161`); the start is `scripted_auto_start`.
  The load sheet fills wings first (`loading::distribute_fuel`), so any sector
  that fits in the wings leaves the centre empty and its pumps running dry for
  the whole flight. A centre tank that had fuel also empties in cruise, when a
  real crew switches its pumps off at the LOW PRESSURE lights.

## What to build

1. **A profile declaration, per airframe, probed.** Something like
   `fuel_tank_pumps_off_when_empty: [1]` (name it to fit the profile's style):
   the tank indices of `fuel_tank_pump_on` whose pumps come off when that tank
   is empty. NEVER a main tank: a main tank running empty is a fuel emergency,
   not a switch to tidy. Silent = the feature is off (old behaviour). Declare
   `[1]` on the 737. Add the name to `aircraft::profile_references` so the
   swap probe covers it (CLAUDE.md: a new field is probed the day it is
   added). Check the other airliner profiles (A330, MD-82, 747 notes) and say
   in the report whether they have a centre tank on the stock array; declare
   only what the xlua proves, else leave them silent and say why.

2. **A monitor check** (`src/monitor.rs`, next to `AltimeterSetting`): for each
   declared tank, condition = pump switch reads 1 AND tank `m_fuel` below an
   "empty" threshold (a named const with a comment; a few kg of unusable
   fuel, not zero) AND engines running, sustained a few seconds
   (`min_duration`) so the scripted start's transient is not a callout. A
   failed read of either array is NO measurement: no callout, no write.
   - An automation flying (the same test the altimeter check uses to choose
     `fix_write` over advice): `fix_write` sets that element to 0, read back
     through the ordinary readback path. The line is a milestone, e.g.
     "Centre tank's empty, centre pumps off." Blackbox it (kind `cabin_air`
     is taken; use a new kind `fuel_pumps`, add it to CLAUDE.md's kinds list)
     with tank, kg, pump before and after.
   - A pilot flying: advise ONCE per episode, no write: "Centre tank's empty,
     centre pumps are still on." (An unasked write to a pilot's fuel panel is
     not ours to make; `mode::crew_actuates()` gates any write regardless.)
   - Check whether `fix_write` supports an indexed dataref
     (`fuel_tank_pump_on[1]`); if it cannot, extend it the smallest way, not a
     parallel write path. One writer per dataref: confirm nothing else in the
     app writes `fuel_tank_pump_on` (grep; `set_fuel_pump` uses the engine
     array, but check the Twin Otter override and checklist `FuelPump`).

3. **Full auto after the start**: the check covers it once engines run, so no
   separate step is needed IF the monitor runs on the ground with full auto
   armed. Verify that; if the monitor is gated off on the ground or during the
   start, run the same decision function once at the end of the start
   (`takeoff_config` is the page full auto runs after the start) rather than
   duplicating the rule. One decision function, pure, tested.

4. **A spoken command** for a pilot who wants it done: "centre pumps off" /
   "center pumps off" / "turn off the centre pumps" routes to a tool that
   writes the declared tanks' pumps off and reads back (refused, naming why,
   when the airframe declares none; if the tank still holds fuel, refuse with
   the kg: turning off the pumps of a tank that has fuel strands it). Add the
   transcript as the grammar test. "Centre pumps on" is the mirror (writes 1),
   allowed whatever the tank holds.

## Tests

- The pure decision: empty + on + engines -> off; fuel in the tank -> nothing;
  pump already off -> nothing; failed read -> nothing; undeclared airframe ->
  nothing; a main tank index is never touched even if declared wrongly (reject
  it at profile load, or ignore with a warning; your call, say which).
- The monitor check with the recorded numbers above (`[2916.8, 0.0, 2918.1]`,
  pumps `[1,1,1]`): automation flying -> fix_write element 1 to 0; pilot
  flying -> advice, no write.
- The grammar test for the spoken command.

## Unflown

Say in the report which blackbox event (`fuel_pumps`) proves it on the next
737 sortie: start at a stand with the wings-only load, the event should fire
within seconds of both engines running.
