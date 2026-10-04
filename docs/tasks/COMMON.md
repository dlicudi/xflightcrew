# Common rules for every delegated task (2026-09-26)

Read the repo's CLAUDE.md fully before touching anything. Its rules are
standing rules; its named functions are where the work goes. Grep
docs/dev-history.md before reworking any subsystem it names.

- Evidence lives in the main checkout, not the worktree: sortie files are
  `logs/sorties/<id>.jsonl` (one file per mission, every record tagged
  `sortie`), the whole log is `logs/blackbox.jsonl` (40 MB; grep it, never
  cat it), both relative to the main checkout. `xvl debrief [file]` prints a
  flight's numbers.
- Run tests with `XCOPILOT_BLACKBOX=off cargo test --workspace` (a test that
  walks a real path otherwise records into the live log). The suite takes
  seconds once built; the build takes minutes the first time.
- Never touch the sim: no Web API calls, no plugin reload, no `xvl` start or
  kill. The pilot may be flying on this machine.
- Never run a bare `cargo fmt`; format only the files you edited
  (`rustfmt <file>`), then check `git diff --stat` for foreign hunks.
- No em dashes or en dashes anywhere (TTS and the panel font choke on them).
  Use a plain hyphen or a new sentence.
- A control-law change ships with a test that fails on the recorded numbers
  and passes after. Replay the recorded ticks where you can; where you need a
  plant, a second-order plant with the loop's real transport delay (a
  first-order lag cannot ring at any gain). Assert the number the blackbox
  reports, not an internal.
- Keep the project's contracts: one writer per dataref, readback against the
  intent, a failed read is no measurement, numbers per airframe from the
  profile with the old default when silent.
- Commit on your worktree branch as you go, small commits, each message a
  sentence in the repo's style (what changed and the evidence that made it
  change). Do not push. Do not merge into main.
- Your final message is the only thing that reaches the pilot. Report: what
  you found (numbers), what you changed (files, functions), the tests you
  added, what is still unflown and which blackbox event proves it on the
  next sortie, and anything you left undone with the reason.
