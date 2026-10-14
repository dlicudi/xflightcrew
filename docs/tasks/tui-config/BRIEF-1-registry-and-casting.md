# Part 1: every pilot-facing knob in the registry, voices cast per role (2026-10-14)

Read `docs/tasks/COMMON.md` first; its rules apply. Read CLAUDE.md, especially
"Panel, terminal, settings" and the home screen paragraph.

## Why

The pilot (2026-10-14): "I want everything configurable via xfc" and "we should
not have to run xfc setup". Today the settings registry (`src/settings.rs`
`ALL`) covers 25 of the 73 env vars the code reads; voices for ATC positions,
the ATC home field, PTT, SimBrief, paths and names exist only as `.env` lines.
Part 2 (a later brief, `BRIEF-2-one-app.md`) folds the home screen into the
running dashboard as tabs. This part is the DATA under those tabs: make every
pilot-facing knob a registry `Setting`, and make voice casting per role a
first-class thing. Keep the current screens working; part 2 redesigns them.

## What to build

### 1. Registry: two new tiers, sections, the missing knobs

- `Tier::Restart`: a knob read once at boot. Its saved value must be in force
  at the NEXT boot through the same env > pref > default chain. Check where
  each knob is consumed: `settings::install` runs at `src/main.rs:306`, after
  `Config::from_env`; a knob consumed before that line (or captured into a
  thread before it) must instead read its pref inside `Config::from_env`, the
  way `piper_voice` does with `prefs::pilot_voice()`. Pin that order with a
  test per knob class, not per knob. `describe()` says "applies on restart".
- Where a knob is cheap to make live (already re-read per tick or per
  utterance), make it `Tier::Live` instead. Do not refactor a consumer just to
  make it live in this part; note the candidates in your report.
- A `section` field on `Setting` (an enum): Crew, Voices, ATC, Audio and PTT,
  Identity and SimBrief, Traffic, Paths, Engine, EFB. Part 2's tabs are these.
  Assign every existing setting one.
- `Kind::Path { dir: bool }` (existence checked on set, said when missing) and
  `Kind::Voice` (a model spec, `path.onnx[#N]`, accepted only when
  `voice::voice_installed` says so; the error names `xfc voices`).
- Add these env vars as settings (env name stays the pin, a wire contract):
  `PIPER_VOICE` (copilot voice, keep the existing `prefs::pilot_voice` field:
  `Persist::Own`), `XCOPILOT_CABIN_VOICE` (same, `prefs::cabin_voice`),
  `XCOPILOT_ATC_PIPER_VOICE`, `_ATC_APPROACH_PIPER_VOICE`,
  `_ATC_DEPARTURE_PIPER_VOICE`, `_ATC_ATIS_PIPER_VOICE`, `_ATC_VOICE_POOL`
  (a list of voice specs; empty = curated voices everywhere), `_ATC_VOICE_SHIFT`,
  `_TTS_LENGTH`, `_TTS_NOISE`, `_TTS_NOISE_W`, `_ATC_TTS_LENGTH`,
  `_ATC_TTS_NOISE_W`, `_SAY_VOICE`, `_SAY_RATE`, `_ATC_VOICE`, `_ATC_RATE`,
  `_ATC_AIRPORT`, `_ATC_REGION`, `_ATIS_FREQ`, `_PTT_KEY`, `_PTT_DATAREF`,
  `_RADIO_PTT_DATAREF`, `_MUTE_XP_RADIO`, `_SIMBRIEF_ID`, `_PILOT_NAME`,
  `_COPILOT_NAME`, `_AIRCRAFT`, `_XPLANE_ROOT`, `_APT_DAT`, `_MODELS_DIR`,
  `_CHIME`, `_NEGATIVE_TONE`, `_SQUELCH_WAV`, `_WHISPER_GPU`, `_TTS_THREADS`,
  `_TTS`, `_MONITOR`, and the whisper model choice if it is an env var today
  (find it in `src/config.rs`).
- Stay env-only (developer knobs, do NOT register): `XCOPILOT_BUILD`,
  `_BLACKBOX`, `_DREF_CACHE_MS`, `_AUDITION_DIR`, `_PLUGIN_STATUS`, `_HOME`,
  `_MONITOR_HZ`, `_UI`, `_LISTEN`, `PIPER_BIN`. `XCOPILOT_HOME` and
  `XCOPILOT_MODELS_DIR` decide where prefs.json itself lives or what it points
  at: think through the chicken-and-egg (`sim::paths` decides `data()` once) and
  register `_MODELS_DIR` only if it is sound; say what you decided.
- Validate as the consumer would: an ICAO the nav data knows, a frequency in
  the COM band, a SimBrief id of digits or a username as `simbrief.rs` accepts.
- A test that every `"XCOPILOT_*"` literal in `src` and `crates` is either a
  registered `Setting.env` or on an explicit env-only list with a reason, so a
  new env var cannot slip in unregistered.

### 2. Voice casting per role

- One pure function that answers "who speaks with what voice" for the CURRENT
  config: copilot, cabin, and per `StationSpeaker` at the home field and at any
  other ICAO (`voice::station_model` with that field's seed and
  `local_voices_on_disk`). Each row: role, voice spec, installed or not, and
  where the value came from (env, pref, default, pool, regional accent). The
  existing casting code is the truth: call it, do not copy it.
- In the `xfc voices` screen (`src/voices/screen.rs`): a Cast view listing
  those rows; on a row, pick from INSTALLED voices (with the existing preview),
  which goes through `settings::set` so pinning and persistence are the
  registry's. A missing voice in a role is red with a one-key install.
  Replace the current copilot/cabin "use" path (`voices.rs:196`) with
  `settings::set` so there is one door.
- The pool is editable there: add or remove a voice from it.

### 3. `.env` import

- At boot, if a `.env` exists and holds registered knobs that differ from
  prefs: nothing changes automatically (env still pins). The terminal Settings
  screen shows which knobs are pinned by `.env` (it already refuses them) and
  offers ONE action, "move these into xfc settings": write them to prefs and
  comment the lines out of `.env` (`# moved to xfc settings 2026-..`), after
  showing the list. Backup the `.env` first. Never touch the shell env.

## Out of scope

- The screen redesign, tabs, self-starting boot, crew restart from the UI:
  part 2. Keep `ui::SettingsPane` rendering the new knobs (grouped by section
  is enough) and the home screen's Settings view working.
- Any flight loop, the command loop, the plugin.

## Done means

- `XCOPILOT_BLACKBOX=off cargo test --workspace` green.
- Every registered setting round-trips: set, persisted, read back at the next
  `Config::from_env` + `settings::install` (test with a temp `XCOPILOT_HOME`).
- Report: the knobs added with their tier, any you left env-only and why,
  consumers that could become live cheaply, and anything in part 2's way.
