# One data folder: xfc runs from anywhere (2026-10-13)

Second step of the distribution work (after `xfc voices`). Today every file
path is relative to the working directory (`models/`, `profiles/`,
`checklists/`, `assets/`, `logs/`, `prefs.json`, `.env`), so `xfc` only works
started from the repo. After this step a copied `xfc` runs from any folder,
and the repo checkout behaves exactly as it does now.

Next steps, NOT this one: the plugin and profiles embedded in the binary, the
TUI home screen, release packaging.

## The rule: ONE owner of every root

A new module owns the roots; nothing else names a top-level folder. It lives
where every crate can reach it: `crates/sim` (the blackbox writer is there),
e.g. `sim::paths`. Two kinds of root:

| Root | What | Writable |
|---|---|---|
| `resources()` | what ships: `profiles/` (the YAMLs), `checklists/`, `assets/` | no |
| `data()` | the user's: `models/`, `logs/` (incl. `sorties/`, `blackbox.jsonl`), `prefs.json`, `.env`, `profiles/learned/` | yes |

Resolution, first match wins, decided ONCE at startup and logged:

1. `XCOPILOT_HOME=<dir>`: both roots are that dir (an explicit override).
2. **Dev checkout**: the working directory (or the binary's location walked
   up to a `Cargo.toml` whose package is `xflightcrew`) contains `profiles/`
   and `Cargo.toml`: both roots are the checkout. This is the pilot's setup
   and must stay byte-identical in behaviour: same files read and written.
3. **Installed**:
   - `data()` = `~/Library/Application Support/XFlightCrew` on macOS,
     `$XDG_DATA_HOME/xflightcrew` (default `~/.local/share/xflightcrew`) on
     Linux. Created on first use. Use the `dirs` crate (MIT/Apache, fine
     under GPL-3) or derive from `HOME`; no heavier dependency.
   - `resources()` = the first that has `profiles/`: `<exe dir>/`,
     `<exe dir>/../share/xflightcrew/` (Homebrew layout). If none, say so
     plainly at startup (the embedding step will make this unreachable).

`XCOPILOT_MODELS_DIR` keeps working and still wins for models;
`config::models_dir()` becomes `data()/models` by default (it is the one
owner already: change its default only).

## What moves

Find every cwd-relative path in non-test code (`rg -n -g '*.rs'
'"(\./)?(logs|models|profiles|checklists|assets|prefs\.json)[/"]' src crates`,
minus `include_str!`/`CARGO_MANIFEST_DIR` and test modules) and route it
through the owner. Known homes, not exhaustive:

- `src/main.rs`: the tracing appender (`logs`), blackbox default path,
  `analyze_commands` reading `logs/`, the squelch WAV.
- `crates/sim/src/blackbox.rs`, `src/sortie_runner.rs` (`logs/sorties`).
- `src/prefs.rs` (`prefs.json`), `src/settings.rs`.
- `src/aircraft.rs` / `src/detect.rs` / `src/aircraft/learned.rs`: profiles
  load from `resources()/profiles`; LEARNED models are READ from
  `data()/profiles/learned` first, then `resources()/profiles/learned` (the
  committed ones ship), and WRITTEN only to `data()/profiles/learned` (`xfc
  identify`). In a dev checkout both are the same folder, as today.
- `src/checklist.rs` (`checklists/`, `resolve_acf_path` keeps its X-Plane
  `acf:` resolution untouched), `src/config.rs` (`assets/afv/*`, every
  `models/...` default including the long `atc_voice_pool` default and the
  `resolve_persona_voices` alternates: build them from `model_path`).
- `.env`: dotenvy today loads from the cwd and its parents. Keep that, and in
  the installed case also load `data()/.env`. The cwd one wins.
- `crates/atc` writes `crates/atc/logs/`? Find out; route it or leave a note.

Test code keeps its `CARGO_MANIFEST_DIR` and relative paths: tests run in the
checkout. Do not churn test files.

**Pin it**: a test (in the style of `every_mcp_write_goes_through_the_owner`)
that scans non-test `src/` and `crates/*/src/` for those literals outside the
paths module and fails naming the file and line. Allowlist with a reason only
where unavoidable.

## Say where things are

- At startup, one log line: the mode (override / dev checkout / installed)
  and both roots.
- `xfc paths`: prints the mode, both roots, the models dir, the log file and
  the prefs file. The future home screen shows the same.
- `xfc voices` header already shows the models dir: unchanged.

## Out of scope

Moving or migrating the pilot's existing files (dev mode keeps them where they
are). Embedding anything in the binary. Windows. Any flight, ATC or traffic
behaviour.

## Constraints

- A live `xfc` may be running from the checkout and the pilot may be flying:
  never kill or restart it, never write to the checkout's `logs/`,
  `models/`, `prefs.json` or `.env` in manual checks, never touch the real
  `~/Library/Application Support/XFlightCrew`. For the installed-mode check,
  copy the built binary to a scratch dir and run it with `HOME=<scratch>`
  (and `XDG_DATA_HOME` unset) from a cwd outside the repo, with a scratch
  resources dir beside it holding a copy of `profiles/`, `checklists/` and
  `assets/`. Do not run the crew loop itself (it talks to X-Plane); `xfc
  paths` and `xfc voices list --installed` are enough.
- Never send requests to X-Plane's Web API.
- `XCOPILOT_BLACKBOX=off cargo test --workspace` must pass.
- No em or en dashes anywhere. Format only files you touched; check
  `git diff --stat` for foreign hunks.
- Update CLAUDE.md in its terse style: one bullet where "Only settings
  persist" is, naming the owner and the three modes. Update
  `docs/guide/install.md` / `configuration.md` briefly (`XCOPILOT_HOME`,
  `xfc paths`, where logs and prefs live when installed).

## Done when

Tests pass, no new warnings, the pin test is in, `xfc paths` prints correct
roots in all three modes (show the output), and a copied binary in a scratch
HOME lists voices from `<scratch>/Library/Application Support/XFlightCrew/models`.
