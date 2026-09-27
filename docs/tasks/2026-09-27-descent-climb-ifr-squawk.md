# Task: a climb in the descent, and a VFR squawk handed to an IFR flight

Read docs/tasks/COMMON.md first, then CLAUDE.md ("The follower's vertical and
the altimeter", "Top of descent", the IFR clearance under "Tower behaviours",
and `take_region_handoff` under "The crew works the radio as it flies").

Evidence: sortie `logs/sorties/1790494983-full_auto.jsonl` (B738, LIEA-LEBL,
2026-09-27, full auto, unattended). Grep it; it is 5 MB.

## A. The follower climbed 1,000 ft in the middle of its descent

The plan's profile (the untagged `plan_audit` record at t 1790494983.0 in
`logs/blackbox.jsonl`, same numbers as the plan): ... KIFMI 31000, ELVOP
28000, MUREN 28000, CHELY 29000, VERSO 25000, OSTUR 16000 ...

The `procedure` `leg` ticks (`want_alt`, `alt_ft`):
- 08:22:00Z "Top of descent in about 2 minutes, 12 miles ahead, down to 28,000
  feet." Descent from 31,000 from 08:23:46.
- 08:25:51 leg 9 MUREN want 28000, levels at 28000.
- 08:26:43 leg 10 CHELY want 29000: CLIMBS to 28,999 by 08:28:37.
- 08:29:08 leg 11 VERSO: descends again (star_ceiling VERSO 23,655).

A crew that has begun its descent never climbs back for a plan altitude
above it (SimBrief navlog altitudes in the descent are not a step climb).
Fix in the pure vertical (`procedure::vertical_tick` and whatever feeds
`want_alt` from the plan leg; read `prev_cmd_alt`, `planned_tod`,
`descent_target_ft`): once the flight is descending on the plan's profile
(past its planned TOD or with a commanded altitude below cruise), a later
plan altitude ABOVE the current commanded altitude is held at the commanded
altitude, not climbed to, until the next lower constraint takes over. Keep:
the pilot's handed altitude still wins (a pilot or controller may ask for a
climb), the terrain floor still raises it (a climb for ground is always
right), and the climb phase is untouched (a plan whose altitudes rise before
TOD is a climb, including step climbs before TOD). Blackbox the held case
once per leg (`procedure` `descent_no_climb`, fix, plan_ft, held_ft).

Test: replay the recorded profile above through the pure vertical: from
31,000 at MUREN's 28,000, CHELY's 29,000 yields 28,000 held, VERSO's 25,000
descends. Plus: a pilot-handed climb in the descent still climbs; a terrain
floor above 28,000 still climbs; a pre-TOD step from 29,000 to 31,000 still
climbs.

## B. The region handoff squawked 7000 on an IFR flight

08:36:52Z the ATC region line: "Radar services terminated. Squawk seven
thousand. Contact Barcelona Tower on one one eight decimal one zero five."
Full auto read it back and set 7000 (`region_handoff` `squawk_vfr=true`,
`squawk_set=true`) at about FL200. The flight had accepted an IFR clearance at
the ramp (`request_clearance` `ifr=true`, `squawk_set` code 4501,
`party_line::assigned_clearance`). A real IFR flight is handed from
approach/area to the next unit with its code kept; "radar services
terminated, squawk 7000" is a VFR service ending.

Fix in `crates/atc` (grep the region handoff line and `take_region_handoff`,
`region_coach_line`): when our aircraft holds an accepted IFR clearance
(the tower side of `assigned_clearance`, which must survive the region's
`Station::install`, as the handoff already does), the handoff keeps the
squawk ("Contact Barcelona Tower on ..." with no squawk instruction; an
arrival is handed to the destination's approach or tower in the ICAO/FAA
dialect the field uses), and the crew side sets no 7000 (the `SetTransponder`
to 7000 happens only for the VFR form). The IFR state is retired at the
landing or when the clearance is cancelled, never at a region switch. VFR
flights keep today's behaviour exactly.

Tests: an IFR-cleared flight crossing a region boundary gets a handoff with
no squawk and the crew keeps 4501; a VFR flight still gets "squawk seven
thousand" and sets 7000; add the recorded handoff line to
`crates/atc/bench/atc_corpus.yaml` if the corpus covers handoffs.

## Unflown

Report the blackbox events that prove each on the next IFR full-auto sortie
(`descent_no_climb`; `region_handoff` with `squawk_vfr=false` and the code
unchanged).
