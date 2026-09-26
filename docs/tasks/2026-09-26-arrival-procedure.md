# Task: the arrival flies the STAR it chose, inside its windows

Read docs/tasks/COMMON.md first. The flight-test card
(`docs/flight-test-card.md`, rows `own_star` and `star_windows`, and
`xvl debrief`'s card section) found both of these from the 2026-09-26 logs
(LIBP to LEMG, 737, sortie files under the live repo's `logs/sorties/`).

## 1. The STAR chosen at the arm never reached the arrival (`own_star`)

At 13:34Z the arm chose UNTO2A for runway 12 on a 120/9 METAR wind (the
plan had UNTO2A for 13; blackbox `plan_audit` event `star_chosen`), and the
arrival then tuned and flew ILS 13 and landed on 13 (`ils` `tuned`,
`taxi_in` `vacating`). Find the seam where the arm's choice
(`plan_audit::choose_arrival`, `full_auto::payload_with_legs`, the plan's
`destination/plan_rwy`, `EndAction::Approach { rwy_id }`) is dropped before
`run_autoland`'s forced runway and the ILS setup, and make ONE value carry
from the arm to the approach: the runway chosen is the runway tuned,
briefed (`brief_arrival`), reported to the tower (the inbound and final
calls), and landed on, unless the tower clears another end
(`clearance_switch` outranks it, said aloud). Per CLAUDE.md the re-pick in
flight (METAR moves, tower names a runway) is still owed; do not build it,
but leave the seam so a re-pick has one place to write.

## 2. A STAR window was crossed high (`star_windows`)

FOFUS on UNTO2A carries 7,000 to 10,000 ft and was crossed at 10,386 ft
(300 ft of slack is the card's pass line; `procedure` `leg` records with
`fix`, `alt_ft`, `dist_nm`; `plan_audit` `star_constraints`; `procedure`
`star_ceiling`). Reconstruct from the ticks: the commanded altitude leg by
leg, the top of descent, the rate flown, and where the 386 ft was lost
(late top of descent, a rate under the planned 75 %, the ceiling rule
`arrival_ceiling_ft` applied to the wrong fix, the pilot's or the
governor's speed, the speedbrake logic from today's descent-path commit).
Fix the cause in the follower's vertical (`procedure::vertical_tick` is
pure and tested) with a test that replays the recorded legs and asserts
FOFUS inside its window.

Do not touch autoland.rs (three other agents are in it) or setup.rs.
Report as COMMON.md says: cause, fix, test, the blackbox fields that show
it next sortie.
