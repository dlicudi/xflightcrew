# Task: put CLAUDE.md on a diet without losing a rule

Read docs/tasks/COMMON.md first.

CLAUDE.md is 1,637 lines and 112 KB (about 28,000 tokens). It is loaded into
every session before any work starts, and AGENTS.md is a symlink to it. It
was trimmed once (2026-09-05, the pre-trim file became docs/dev-history.md,
now 6,032 lines) and has since regrown: most bullets now carry the sortie
that taught the rule, with airport, date, and the numbers. That evidence is
what makes the rules credible, and it belongs in the history, not in the
file that is read every time.

## The rule for this task

MOVE, never delete. Every sentence that leaves CLAUDE.md lands verbatim in
docs/dev-history.md under a dated heading for its subsystem (append a
"2026-09-26 trim" section per subsystem, or merge into an existing dated
section where one matches the sortie date). A reader who greps the history
for any airport code, date, function name or number that is in today's
CLAUDE.md must still find it afterwards. Check that with a script before
you finish: extract every ICAO code, every `YYYY-MM-DD`, every backticked
identifier from the old CLAUDE.md, and assert each still occurs in
CLAUDE.md or dev-history.md.

## What stays in CLAUDE.md

- Every wire contract, every "never" and "always", every named function,
  constant, dataref, env var, file and blackbox kind that a rule points at.
- The one-line WHY of each rule (a rule without its why gets reverted).
- The "Still owed" / unflown items, gathered into ONE short list at the end
  of each section instead of scattered mid-paragraph.
- The structure: keep the section headings and their order so existing
  memory notes and the history's cross-references still land.

## What moves

- The sortie narratives: the airport, the date, what the aircraft did, the
  numbers. Replace each with a pointer in the form "(LFPB 2026-09-22, see
  history)" only where the date is the key to finding it; otherwise drop the
  parenthetical entirely because the history's heading carries it.
- Repeated statements of the same rule in different sections (say it once,
  in the section that owns the function).

## Target

Under 600 lines and under 45 KB, every rule intact, every pointer resolvable.
Rewrite for the reader who has never flown the sortie: short bullets, the
rule first, the why second, the pointer last. No em dashes or en dashes
(the file has many; replace with a hyphen or a full stop).

Verify: `cargo test` still passes (some tests read CLAUDE.md or pin names
that appear in it; grep the tests for `CLAUDE.md`), the symlink is intact,
the extraction check above passes. Commit the history move and the trim as
two commits so the diff is reviewable.

Report: line and byte counts before and after, the extraction check's
result, and any sentence you were unsure about moving (list them; the pilot
decides).
