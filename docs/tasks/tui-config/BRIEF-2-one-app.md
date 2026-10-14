# Part 2: one app, no setup step (2026-10-14)

Read `docs/tasks/COMMON.md` first. Starts AFTER part 1
(`BRIEF-1-registry-and-casting.md`) is merged: it uses the registry's
`section`, `Tier::Restart`, `Kind::Path`/`Kind::Voice` and the casting rows.

## Why

The pilot (2026-10-14): "we should not have to run xfc setup". Today `xfc
setup` / `xfc home` (`src/home.rs`) is a separate screen; plain `xfc` shows it
only when a required check is red, and `r` leaves it to boot
(`src/main.rs:235`). The running dashboard (`src/ui.rs`) is a different screen.

## What to build

- **Plain `xfc` on a TTY is ONE TUI**, tabs: Flight (today's dashboard), Checks,
  Voices (the voices screen and part 1's Cast view), then one tab per registry
  `section` (ATC, Audio and PTT, Identity and SimBrief, Traffic, Paths, Engine,
  EFB), Plugin, Files. Reuse `home.rs`'s checks and fixes and
  `voices::screen`; move code, do not fork it.
- **The crew boots itself**: required checks green, it boots at once. Red, the
  app opens on Checks and boots the moment they all pass, no `r`, no restart.
  The Flight tab says "waiting on: <check>" until then.
- **Settings change inside the running app**: `Tier::Live` applies at once;
  `Tier::Restart` is marked, and the app offers "restart the crew" which
  re-runs boot in-process ON THE GROUND ONLY (CLAUDE.md: never restart while
  flying; read `flight_state` / `sim::ground` and refuse airborne, said). Work
  out what an in-process re-boot must stand down and re-arm; if it is not
  sound, fall back to "quit and start xfc again" and say why in the report.
- **ATC tab**: home field, region, plus a "cast at" box: type
  any ICAO, see who speaks with what voice there (part 1's casting rows).
- Search (`/`) across every setting by label and help; a filter for pinned.
- `xfc setup` and `xfc home` stay as hidden aliases that open the Checks tab.
  `xfc run` stays headless, never a TUI; `--print` stays.
- Rendering tests under `TestBackend` as `ui.rs` does today.

## Found in part 1 (merged ac27b6f)

- `atc_airport`'s check and `voice::cast::cast_now(Some(icao))` each scan the
  ~400 MB apt.dat on the calling thread: the "cast at" box and the ATC tab need
  a worker thread or a cache; the UI never blocks on it.
- `voice::RUNNING` is a `OnceLock` set once at boot (`register_running`): an
  in-process crew restart must be able to replace it.
- Restart knobs read a prefs SNAPSHOT taken by `Config::from_env`
  (`config::knob`, `reload_boot_knobs`); `knob_next` is what the next start
  reads. A crew restart must take a fresh snapshot.
- `.env` pins moved by `env_import` still pin until the process restarts
  (dotenvy loaded them): a crew restart does not clear the process env, so say
  "quit and start xfc" for those, or remove the moved vars from the process
  env as part of the move. Decide and report.
- `XCOPILOT_ATIS_FREQ` is env-only (only the standalone `atc` binary reads it):
  the ATC tab does NOT offer an ATIS frequency.
- Cheap live candidates (optional here, only if the restart path is not
  sound): the radio loop's ATC voices and pool read `settings::live()` per
  transmission; chime and tones; `say_*`.

## Out of scope

Flight loops, the command loop, the plugin, the EFB page.

## Done means

`XCOPILOT_BLACKBOX=off cargo test --workspace` green; report what plain `xfc`
does from a clean data folder, from a complete one, and with a `.env` pin.
