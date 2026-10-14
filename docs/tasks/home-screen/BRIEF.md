# The home screen: setup, voices, settings, run (2026-10-14)

Fourth step of the distribution work, after `xfc voices`, the one data folder
(`sim::paths`) and the single file (`xfc plugin`, embedded resources). Goal:
a new user downloads one `xfc`, runs it, and the screen walks them to a
working crew. Read `src/voices.rs` + `src/voices/`, `src/plugin_install.rs`,
`crates/sim/src/paths.rs` and `src/ui.rs` first: reuse them, never fork them.

Three parts, in this order (part 1 is a safety fix and lands even if the
rest slips).

## 1. A voice that is not installed is never drawn

Today `voice::station_model` (`src/voice.rs`, ~line 291) picks controllers
and ghost pilots from `atc_voice_pool` whether or not the file exists. The
default pool names ~40 voices; a user with only the starter set would get
a missing model for some fields.

- The pool is filtered to INSTALLED entries (the `.onnx` AND its
  `.onnx.json` present; strip `#n` before the check) once, where the pool
  is parsed, not per call. Seeds stay stable over the filtered list.
- An empty filtered pool falls back to the curated per-position voices, as
  an empty pool does today.
- Any chosen model that is missing at load time (curated ones included, a
  user may remove them) falls back: the tower voice, then the copilot voice,
  then whatever IS installed; said ONCE in the log per missing file, never
  per line. Find where the model is loaded for a speaker and put the
  fallback there, one place.
- `local_voices_on_disk` already checks existence: leave it.
- Tests: a pool with missing entries draws only installed ones; a missing
  curated voice falls back; pure, on a scratch models dir.

## 2. The starter set

ONE definition (`voices::starter(&Config)`): the whisper model the config
names (default `ggml-base.en`) plus every SINGLE voice the config names:
copilot, cabin, tower, approach, departure, ATIS when set. Derived from the
live config, never a second hardcoded list, so changing a default changes
the starter. Pool entries are NOT in it (optional, `xfc voices` adds them).

- `xfc voices install --starter` installs what is missing of it, with
  progress, same atomic path as today. Print the total size first.
- `xfc voices` screen: a line in the header ("starter set: complete" or
  "starter set: 3 missing, 420 MB, press s"), `s` installs it.

## 3. The home screen

`xfc home` (alias `xfc setup`): a ratatui screen in the house style
(`src/ui.rs`, `src/voices/screen.rs`).

```
 XFlightCrew                                   build 2026.10.14
 ─────────────────────────────────────────────────────────────
  Setup      ✓ X-Plane found  ~/X-Plane 12
             ✓ Web API on 8086
             ✗ Plugin older than this build (2026.10.13)  enter: update
             ✓ Microphone  MacBook Pro Microphone
             ! Starter voices: 2 missing, 180 MB          enter: get them
  Voices     35 installed · 2.5 GB                         enter: open
  Settings   mode both · listen ptt · EFB off              enter: open
  Files      data ~/Library/Application Support/XFlightCrew
 ─────────────────────────────────────────────────────────────
  r run   ↑↓ move   enter fix/open   q quit
```

- **Checks**, each a pure verdict over gathered facts (tested), each with a
  fix on enter:
  - X-Plane folder (`config::xplane_root`): found / not found (fix: say how
    to set `XCOPILOT_XPLANE_ROOT`; a path prompt that saves it to prefs if
    the settings registry has a home for it, else just say the variable).
  - Web API: ONE `GET` to `{xplane_base_url}/api/capabilities` (or the
    cheapest endpoint the sim crate already uses for a liveness check; find
    it) with a short timeout, ONLY when the user opens the screen or
    presses a re-check key, never polled. Not reachable: say "X-Plane not
    running, or the Web API is off (Settings, Network)".
  - Plugin: `plugin_install` status verdict; fix = install (its own
    refusal while X-Plane runs stands; show its message).
  - Microphone: the default input device name via cpal, no recording.
    Fix: run the existing `mic-check` level meter inside the screen if that
    is easy, else say `xfc mic-check`.
  - Starter set: part 2; fix = install with progress on the screen.
- **Voices** opens the existing voices screen (return to home on quit).
- **Settings** opens the settings registry: reuse the terminal dashboard's
  settings view (`src/ui.rs` `draw_settings`) if it can be lifted without
  dragging the live crew along; else a read-only list with "edit in the
  panel or EFB" and leave editing owed. Say which you did.
- **Files**: `sim::paths` mode and roots, as `xfc paths` prints.
- **r** leaves the screen and starts the crew exactly as plain `xfc` does
  today (same process: restore the terminal, then fall through to the
  normal boot). A red check does not block `r`; it warns once.

### When it appears

The pilot's workflow (`cargo run`, plain `xfc`) must not change when all is
well. So plain `xfc`:
- interactive terminal (stdin and stdout are TTYs) AND any REQUIRED check
  is red (X-Plane folder, plugin missing, starter set incomplete): show
  the home screen;
- otherwise boot the crew as today.
Web API down is NOT required (the crew already waits for the sim).
`xfc run` always boots straight; `xfc home` always shows the screen.
Running checks for this decision must be cheap and offline: file checks
only, no network, no Web API call, no mic open.

## Constraints

- A live xfc may be running and the pilot may be flying. Never kill or
  restart xfc or X-Plane; never reload plugins; never install into the
  real `~/X-Plane 12` (use `XCOPILOT_XPLANE_ROOT` or `--xplane` pointing at
  a scratch tree); never write the checkout's `models/`, `logs/`,
  `prefs.json`, `.env` or the real `~/Library/Application Support/XFlightCrew`
  (use `XCOPILOT_HOME=<scratch>`).
- The ONE Web API GET in the home screen is allowed against the real sim
  for a manual check, once; nothing else talks to the sim.
- `XCOPILOT_BLACKBOX=off cargo test --workspace` passes. TestBackend render
  tests for the home screen (all green, a red plugin, starter missing).
- No em or en dashes. Format only files you touched.
- CLAUDE.md: one terse bullet near the `xfc` single-file bullet (home
  screen, when it appears, starter set = the config's named voices,
  pool draws installed only). `docs/guide/install.md`: the new-user path
  becomes "run `xfc`, follow the screen".

## Done when

Tests pass; show the home screen's TestBackend renders; in a scratch
`XCOPILOT_HOME` with an empty models dir and a scratch X-Plane tree, plain
`xfc` (TTY) shows the screen (you may not have a TTY: then show `xfc home
--print`, a one-shot text dump of the checks you add for exactly this and
for support); `xfc voices install --starter` into scratch installs the set
and prints its size first; the pool filter test passes.
