# Task: one flight-test card for everything that is unflown

Read docs/tasks/COMMON.md first.

More than twenty changes shipped between 2026-09-21 and 2026-09-26 that no
sortie has verified. They are recorded in two places: CLAUDE.md's
"unflown" / "Still owed" lines, and the pilot's session notes in
the agent's memory directory
(read `MEMORY.md` there for the index; every file whose title says
"unflown" is one item, and each names the blackbox events to check). The
flight model itself is "in shadow" (`src/flight.rs`, `src/duty.rs`) waiting
for logs that agree with flights flown.

Every sortie the pilot flies could retire several of these, but nobody can
hold twenty checklists in their head, so none get ticked off. Build ONE
card the pilot can run after any sortie.

## Deliverable 1: docs/flight-test-card.md

A table, one row per unflown item:

| item | shipped | what to fly (airframe, field, action) | blackbox evidence (kind, event, fields) | pass criterion | status |

Group by what a single sortie can cover: a 737 full-auto IFR flight (SID /
STAR / ILS / plan audit / load sheet / takeoff config / cabin air), a 172
circuit, a helicopter route, a ramp-only session (start, pushback, cabin
voice picker, hand-over words), a hand-flown session (rapport, tempo,
say again, instructor). Put the items that can be verified from logs of
flights ALREADY flown today first: check the sortie files under
`logs/sorties/` and the main blackbox
for each item's event, and if the evidence is there, fill in the status
column now (verified, failed with the numbers, or not exercised). Be
precise about the event and field names: read the source that emits each
one (`crate::blackbox::record("<kind>", ...)`) rather than trusting the
note.

## Deliverable 2: `xvl debrief` prints the card's status

Add a section to `src/debrief.rs` (pure, tested on a small fixture) that
walks the same blackbox and, for every item on the card that has a
machine-checkable criterion, prints verified / failed / not exercised with
the number. Keep the card and the code in one place: the list of items
lives in Rust (a static table with name, kind, event, predicate) and
`docs/flight-test-card.md` is generated or at least checked against it by a
test, so the two cannot drift.

Do not change any flight law or any grammar in this task. Report: how many
items, how many already verified or failed from today's logs (with the
numbers), and the ones whose criterion you could not make machine-checkable.
