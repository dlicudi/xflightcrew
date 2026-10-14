# Arrival sequencing and speed control (2026-10-14)

Read `docs/tasks/COMMON.md` first; its rules apply. Read CLAUDE.md's ATC
section whole. Grep `docs/dev-history.md` for "vectors", "slot", "spacing",
"LANDING_HOLD" before changing those subsystems.

## The gap

Approach vectors ONE aircraft (us) onto ONE final as if the sky were empty
(`crates/atc/src/station/radar.rs` `vector_tick_for`, `respond.rs`
`arm_vectors`; `ApproachWatch` for an instrument arrival on its own
navigation). Traffic arrivals already sequence among THEMSELVES by time
(`crates/traffic/src/slot.rs` `RunwaySlot`, `slot_free`, `leader_ready`;
`arrival.rs` `SPEED_BANDS`, `spacing_kts`, orbits), but our own aircraft is
not in that picture: `OwnTarget` (`phase.rs`) carries only position for
GROUND spacing. So:

- traffic on final never spaces behind us, and we are never spaced behind it;
- approach gives us no number, no traffic to follow, no speed;
- #237: the tower clears us to land with traffic landing ahead
  (`tower.rs` `Intent::ReportFinal` checks only `ctx.runway_occupied` now),
  then sends us round.

## What to build, in this order (commit after each)

### 1. Our aircraft in the slot picture (crates/traffic + src/traffic_link.rs)

Airborne and arriving at the traffic's field (we hold a vectors plan, an
approach watch, a landing clearance, or are inside the final wedge for a
runway end of the field), our aircraft publishes a `RunwaySlot` like any
traffic aircraft: time to threshold along the path we will fly (along the
final from the join, plus the remaining distance to the join when not yet
on it) at our groundspeed bounded by the schedule; runway time from our
type's perf where known, else `JET_MED`/`PISTON_SINGLE` by the profile's
class. Carry it in `OwnTarget` (or beside it), fed from `traffic_link.rs`
`own_target`. Our slot is `committed` once we are on final inside
`ORBIT_MIN_M` or cleared to land; before that it is NOT waiting, it is the
player: **traffic yields to us**, as it does on the ground ("traffic waits
for us"). A traffic straight-in that cannot fit behind us takes its delay
early (its existing orbit), a circuit extends its downwind. ONE rule:
reuse `slot_free` / `off_before` / `leader_ready`; do not write a second
fit test.

Test: a `sortie_tests.rs`-style run with our aircraft on an 8 nm final and a
traffic straight-in 2 minutes behind: the traffic orbits or slows, never
reaches the threshold inside `FOLLOW_GAP_S` of our runway-off time, no
go-around ordered.

### 2. The landing sequence (crates/atc, computed never stored)

One query on the station, beside `RunwayBook` and in its spirit:
`landing_sequence(runway) -> Vec<(FlightId, to_thr_s)>`, ordered by time to
threshold, computed each tick from strips' positions (traffic strips carry
what the traffic link publishes; if the slot times are not on the strip,
publish them through the link the same way positions are, as a strip cell,
never a `pub static`). Hysteresis: an order once SPOKEN to us holds unless
the gap reverses by more than 30 s (a number that flips every tick is worse
than none). `own_number()` and `own_leader() -> Option<(callsign, type,
dist_nm)>` are its accessors for the vectors and the tower.

### 3. Approach speaks the sequence and controls our speed

In `vector_tick_for` and the `ApproachWatch` (the vectors' arm reply too):

- Once a leader exists, the number and the traffic, ONCE per change, in the
  dialect (CAP 413 6.5 / FAA 7110.65 5-9): "number two, following a Boeing
  737 on eight mile final" (`traffic::perf` type names where known).
- Speed control on the arrival's schedule, the SAME bands the traffic flies
  (`arrival::SPEED_BANDS`: 250 below FL100 beyond 20 nm, 210, 180 to the
  join, 160 to 4 nm), issued only when the band changes and only to a type
  that can fly it (never below `aircraft speed_floor`-equivalent: the
  station gets the type's approach speed + 10 from the crew via a
  `party_line` accessor; unknown = no speed issued). Behind a leader and
  short of the gap, a reduction toward minimum approach speed
  (`MIN_SPACING_OVER_APP_KTS`) instead; still short beyond the join, the
  base turn is DELAYED (the join point moved out along the final, "extend
  downwind" when on a downwind-shaped leg). At 4 nm: "no speed
  restrictions" is NOT said; the 160 simply ends ("maintain 160 knots to 4
  DME" says it).
- A speed is a readback item: enforced like headings and levels
  (`coach.rs` / `compliance.rs` patterns; grep how a heading readback is
  checked). Published for the crew as `party_line::assigned_speed()`
  (accessor, cleared on handoff to tower, new flight, go-around).
- Blackbox `atc` events: `sequence` {number, leader, gap_s}, `speed_assigned`
  {kts, why: band|spacing}, `downwind_extended` {join_nm}.

The crew's FOLLOWER obeying an assigned speed is NOT in scope: the flying
loops are frozen (CLAUDE.md "Direction"). Read it back through the crew
radio as any other item; file an `owed` issue "follower flies the assigned
approach speed" naming `party_line::assigned_speed`.

### 4. The tower: number two, never a clearance it must cancel (#237)

`Intent::ReportFinal` (and the landing clearance coach) with a leader
ahead on the same runway that will not be off `FOLLOW_GAP_S` before we need
the runway: "continue approach, number two, traffic <type> on short final",
and the clearance then comes through the existing `LANDING_HOLD` /
`landing_release_due` path once it is off. Test pinned to #237's numbers:
leader 1.4 nm ahead at our 997 ft final report, no `cleared to land` before
the leader vacates, no `ordered_go_around`. `fixes #237` only after it is
flown; reference it until then.

## Out of scope

Holding stacks (next task), enroute sectors (#291), traffic flying STARs
(#262), parallel runways (#261). Note where each would plug in, in your report.

## Done means

`XCOPILOT_BLACKBOX=off cargo test --workspace` green (except the known
flaky #271 set; say if they fail); `cargo tree -p atc` still has no
`traffic` dependency, nor `traffic` an `atc` one; CLAUDE.md's ATC section
gets one bullet per new rule with its home, in the file's voice; an
`unflown` issue listing the blackbox events above for the first sortie.
