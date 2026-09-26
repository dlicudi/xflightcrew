# Task: an unattended sortie runner, so a landing test is one command and a night's testing is one file

Read docs/tasks/COMMON.md first. Read CLAUDE.md's "Battle-test surface"
(the sortie, `full_auto::SortieGuard`, `src/scorecard.rs`, `xvl debrief`,
`src/test_card.rs`) and `src/tools/setup.rs` (the flight-init mailbox
`xcopilot/sim/init_flight`, the pure JSON builders, the new `SetUpApproach`).

## Why

Tonight's landing test at LEMG (2026-09-26, 19:42 to 19:48Z) took a person
at the keyboard for every step: reposition, arm, watch, read the verdict.
The pilot asked for testing to be faster and automatic. Everything the
runner needs already exists as pieces: the mailbox puts an aircraft on a
final or a stand, the typed queue (`xcopilot/typed`, one line per
main-loop pass) takes any command, the scorecard emits `stage_result` and
`sortie_result`, and `logs/sorties/<id>.jsonl` holds each mission. What is
missing is the driver.

## Deliverable: `xvl sortie <spec.yaml>` (a subcommand, like `xvl debrief`)

Runs against a LIVE X-Plane and a LIVE xvl (the runner is a second
process talking to the sim's Web API and to the app through the plugin's
mailboxes; it never links the sim or the STT). For each case in the spec:

1. **Set up the flight** from the case: `approach` (field, runway, final
   nm; the same JSON `SetUpApproach` builds, call the pure builder, do not
   copy it), `runway_start` (field, runway, engines running or not) or
   `ramp` (field, stand, cold and dark). Write the mailbox, wait on
   `init_seq`/`init_result`, survive the sim's reload (the Web API errors
   and dataref ids change while it reloads: re-resolve ids, retry, bounded).
2. **Wait for the sim to settle** (a few seconds; on an approach case the
   aircraft is placed with nobody flying, so step 3 follows within ONE
   second of the result).
3. **Issue the mission** through the typed queue: the case's command, e.g.
   `land the aircraft`, `fly a circuit`, `full auto`, `fly the flight plan`.
   Confirm the app took it (the `tool` blackbox record or the `said`
   readback within 5 s; otherwise retry once, then fail the case).
4. **Wait for the verdict**: the `sortie_result` for a mission, or for an
   approach case the landing `stage_result` followed by the mission end
   (make the approach case a SORTIE: today a stand-alone autoland has no
   `sortie` tag and no `sortie_result`; wrap it with `SortieGuard` the way
   full auto does, so every case ends in one `sortie_result` and one file).
   A deadline per case (the case's own, default 20 min), a crash
   (`has_crashed`), or a give-up ends the case as failed with the reason.
5. **Record and continue**: append one row to a run table (case, sortie id,
   pass/fail, each stage's number and limit, interventions, the reason),
   printed as it goes and written to `logs/sorties/run-<stamp>.md`. Exit
   non-zero if any case failed. `--repeat N` runs each case N times (the
   same approach flown five times is the test of a flare law).
6. **Safety**: refuse to start if the app reports a loop flying
   (`xcopilot/status/auto`) or the sim is in replay; never send a second
   setup while a case is running; "my controls" through the typed queue
   on abort (Ctrl-C) so nothing keeps the aircraft.

The spec is YAML: a list of cases with `name`, `setup` (one of the three
shapes), `command`, `deadline_min`. Ship `bench/sorties/lemg-737.yaml`
(a 10 nm final to 13, a 5 nm final to 13, a runway start with a circuit)
and document the shape at the top of the file.

## Speed

Report, do not decide: whether X-Plane's `sim/time/sim_speed` at 2x keeps
the laws honest. Control loops tick in wall time and the blackbox `state`
already logs the sim speed. Add a `sim_speed` field per case, run the same
approach at 1x and 2x if the sim is up when you finish (it may not be; the
pilot may be away), and if you cannot fly, leave the field working and
say so. Shorter cases are the real speed-up: a 5 nm final is a two-minute
landing test.

## Rules

Read the blackbox as the app writes it (tail `logs/blackbox.jsonl`, or
the plugin's status mailbox); never parse the TUI. No flight law changes.
Do not edit `src/tools/setup.rs` or `src/autoland.rs` beyond what the
`SortieGuard` wrapping needs (another agent is fixing the setup tool's
follow-up right now: its mailbox write worked, its wait-for-result did
not, so build your own wait and re-resolve ids after the reload). Tests:
the spec parser, the case state machine over a fake feed of blackbox
records (settle, took the command, verdict, deadline, crash), the run
table. Report as COMMON.md says, plus the exact command the pilot types
to run a night of landings.
