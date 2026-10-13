# Voice manager: `xfc voices` (2026-10-13)

First piece of the distribution work: a user installs, previews, uses and
removes voices and speech models from one command, never by hand-editing
`.env` or curling Hugging Face.

## What to build

`xfc voices`: a scrollable ratatui screen (the crate is already a dependency,
see `src/ui.rs` for the house style), plus non-interactive forms for scripts:

```
xfc voices                    the screen
xfc voices list [--installed] [--filter en_GB]
xfc voices install <key>...   e.g. en_GB-alan-medium, or ggml-base.en
xfc voices remove <key>...
```

Screen sketch:

```
 Voices                         installed 41 · 2.9 GB      filter: en_GB_
 ─────────────────────────────────────────────────────────────────────────
 ✓ en_GB-alan-medium        Great Britain   medium   1 spk    63 MB   tower
 ✓ en_GB-alba-medium        Great Britain   medium   1 spk    63 MB   ATC default
   en_GB-cori-high          Great Britain   high     1 spk   114 MB
 ↓ en_GB-vctk-medium        Great Britain   medium 109 spk    77 MB   42%
 ─────────────────────────────────────────────────────────────────────────
 ↑↓ move  / filter  enter install  d uninstall  p preview  u use as...  q quit
```

Two tabs: **Voices** (Piper) and **Speech recognition** (whisper ggml).

## The catalogues

- **Piper**: `https://huggingface.co/rhasspy/piper-voices/resolve/main/voices.json`
  (177 entries). Per key: `language.{code,country_english}`, `quality`,
  `num_speakers`, `speaker_id_map`, `files` = `{ "<path>": { size_bytes,
  md5_digest } }`. Download URL is `.../resolve/main/<path>`. Install only the
  `.onnx` and `.onnx.json` (skip MODEL_CARD); both land flat in the models dir
  as `<key>.onnx` / `<key>.onnx.json` (that is how `models/` is laid out today).
- **Whisper**: no upstream manifest. Ship our own small table in the code
  (key, file, size, sha256) for `ggml-base.en`, `ggml-small.en`,
  `ggml-large-v3-turbo` (and `ggml-medium.en` if cheap) from
  `https://huggingface.co/ggerganov/whisper.cpp/resolve/main/<file>`. Get the
  real sizes and sha256 from the HF API (`/api/models/ggerganov/whisper.cpp/tree/main`
  reports `lfs.oid`); never invent a checksum, a test pins the table's shape.
- Cache `voices.json` in the models dir (`.catalogue/voices.json`) for 24 h;
  offline uses the cached copy and says so; no cache and offline lists the
  installed voices only, said on screen.
- This is the third outbound call after SimBrief and the METAR file: on demand
  only, never at boot or in the command loop. Add one line to CLAUDE.md under
  Stack saying so (keep CLAUDE.md's terse style).

## Rules

- **Models dir**: `XCOPILOT_MODELS_DIR`, default `models` (relative, as every
  default path in `src/config.rs` is today). One function owns it; the
  distribution work will later change its default to the user data folder.
- **A download is atomic**: stream to `<file>.part`, verify size AND checksum
  (md5 for Piper: add the `md-5` crate, RustCrypto, MIT/Apache, fine under
  GPL-3; sha256 via `sha2` for whisper), rename only when both pass; the
  `.onnx.json` is installed with its `.onnx` or neither is. A failed or
  interrupted download leaves nothing that looks installed; stale `.part`
  files are cleaned on the next run. Progress on the screen per file.
- Use the workspace `reqwest` (blocking is enabled), no new HTTP stack. Run
  downloads off the UI thread so the list stays scrollable.
- **Installed** = both files present in the models dir. Voices found on disk
  that are not in the catalogue are still listed (as "local").
- **In use** tags come from the live config: copilot (`piper_voice`), cabin
  (`cabin_piper_voice`), tower / approach / departure / ATIS voices, and every
  entry of `atc_voice_pool` (entries carry `#n` speaker suffixes; strip them).
  Whisper: the configured `whisper_model_path`. **Remove refuses an in-use
  model and names who uses it.**
- **Use as** (`u`): copilot or cabin only in v1, through the SAME prefs the
  panel's voice picker writes (`crate::prefs::pilot_voice` / `cabin_voice`
  and their setters; find them). ATC roles stay env-configured; the screen
  shows the variable to set. Say on screen that a running xfc picks the change
  up on restart.
- **Preview** (`p`): speak a short line ("Tower, Cessna one two three, ready
  for departure") with the in-process ONNX path (`src/tts_onnx.rs`; `xfc tts`
  in `src/main.rs` already does a one-off synthesis, reuse it). Multi-speaker
  voices preview speaker 0. Respect `build_session` serialisation.
- Subcommand wiring: follow the existing `std::env::args().nth(1) ==
  Some("...")` pattern in `src/main.rs` (around `conform`, `identify`). Put
  the logic in a new module (`src/voices.rs`, sub-modules if it grows), not
  in `main.rs`.
- Pure core, thin shell: catalogue parsing, the installed/in-use join, the
  filter, and the remove gate are pure functions with plain `cargo test`
  tests over a trimmed `voices.json` fixture (put it under `bench/` or a test
  fixture dir, a few entries incl. one multi-speaker). No network in tests.
- No em or en dashes anywhere (code, strings, docs). Do not run `cargo fmt`
  on the workspace; format only the files you touched.
- Update `docs/guide/install.md` and `docs/guide/configuration.md` briefly:
  how to get voices with `xfc voices`.

## Out of scope

The user data folder, `xfc setup`, release packaging, embedding assets. Do
not touch flight loops, ATC or traffic code.

## Done when

- `XCOPILOT_BLACKBOX=off cargo test --workspace` passes.
- `cargo build` clean of new warnings.
- Manually: `xfc voices list --filter en_GB` prints the catalogue with the
  installed ones marked (a real network call is fine here);
  `xfc voices install en_GB-alan-medium` into a scratch
  `XCOPILOT_MODELS_DIR` works and verifies; `remove` of a default-config voice
  refuses. Never touch the repo's real `models/` in these checks.
- One commit on the branch, message in the repo's style.
