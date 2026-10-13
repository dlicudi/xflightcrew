# Everything small inside xfc; xfc installs the plugin (2026-10-13)

Third step of the distribution work, after `xfc voices` and the one data folder
(`sim::paths`, read `crates/sim/src/paths.rs` first). Goal: a single `xfc`
file, copied anywhere, needs nothing beside it. Only the big models are
downloaded (`xfc voices`). Next step, NOT this one: the TUI home screen.

## 1. Resources built into the binary

Embed `profiles/` (the YAMLs AND `profiles/learned/*.json`), `checklists/`
and `assets/` (with `assets/afv/AFV_COPYING.md` and `README.md`).

- **Unpack, do not rewrite readers.** Every reader opens real paths today
  (YAML, checklist text, the WAV loader). So: when `sim::paths` finds no
  on-disk resources (the installed case that today says "no profiles/
  found"), unpack the embedded tree to `data()/resources/<build id>/` once
  and make THAT `resources()`. A marker file written last marks a complete
  unpack (an interrupted one is redone); older `<build id>` folders are
  removed. Never unpack in the dev checkout or when `XCOPILOT_HOME` or a
  folder beside the binary has `profiles/`: on-disk wins, so editing a
  profile in the checkout still works.
- Mechanism: the `include_dir` crate (MIT/Apache) or a `build.rs`
  generated table; your call, say which and why. `cargo:rerun-if-changed`
  on the three folders so an edited profile rebuilds.
- `assets/afv/Click_f32.wav` and `Crackle_f32.wav` are gitignored today, so a
  clean CI checkout would not have them. They are BSD-3 (AFV-Native, notice
  in `AFV_COPYING.md`, total under 1 MB): commit them (`git add -f`, and fix
  the ignore rule) so every build embeds them. If a resource folder is
  missing at build time, the build FAILS loudly; never silently embed an
  empty tree.
- `xfc paths` shows the unpacked mode ("embedded, unpacked to ...").
- Learned writes stay `data()/profiles/learned` (unchanged); the unpacked
  tree is read-only.

## 2. The plugin inside the binary: `xfc plugin`

The `.xpl` needs the X-Plane SDK and clang, so `cargo build` cannot build it.

- Cargo feature `embed-plugin` (off by default): `include_bytes!` from the
  path in env `XFC_PLUGIN_XPL` at build time (the build fails with a clear
  message if the feature is on and the variable or file is missing). Also
  embed the plugin's version and build id as strings (from the Makefile's
  `VERSION`/`BUILD`, or read from the file; see below).
- `scripts/build-release.sh`: `make -C plugin` (no install), then
  `cargo build --release --features embed-plugin` with `XFC_PLUGIN_XPL`
  set, then copy the result to `dist/xfc`, print its size. No shell
  comments in it beyond a short header. Add `dist/` to `.gitignore`.
- Commands:
  - `xfc plugin status`: the X-Plane folder (`config::xplane_root`, i.e.
    `XCOPILOT_XPLANE_ROOT` or `~/X-Plane 12`; `--xplane <dir>` overrides),
    whether `Resources/plugins/xflightcrew/mac_x64/xflightcrew.xpl` exists,
    its version and build, whether an old `plugins/xcopilot/` exists, the
    embedded plugin's version and build (or "this build carries no
    plugin"), and a verdict: up to date / older / newer / missing.
    The plugin's version strings are compiled in (`XCOPILOT_VERSION`,
    `XCOPILOT_BUILD` in `plugin/Makefile`); find a robust way to read them
    from an installed `.xpl` (e.g. a marker-prefixed string the plugin
    source already carries or you add, scanned in the bytes). If you add a
    marker to `plugin/src`, keep it a plain `static const char[]`, no
    behaviour change.
  - `xfc plugin install [--xplane <dir>] [--dry-run]`: what `make -C plugin
    install` does, from the embedded bytes: remove `plugins/xcopilot/` FIRST
    (two copies would both load, CLAUDE.md "Wire contracts"), back up an
    existing `plugins/xflightcrew/` to `data()/plugin-backups/<stamp>/`,
    never next to it (X-Plane loads every folder under `plugins/`, so a
    backup there would load as a second copy). Then write the `.xpl`
    atomically (temp file in the same folder, rename). `--dry-run` prints
    every step and changes nothing.
  - **X-Plane running** (a process named `X-Plane`): install refuses unless
    `--while-running`, and then says the plugin must be reloaded or X-Plane
    restarted. NEVER reload plugins automatically (a reload while the panel
    is live has broken it before).
  - Without the embedded plugin: install says so and points at
    `make -C plugin install`.
- Pure core, thin shell: the status verdict, the install plan (a list of
  steps) and the version comparison are pure and tested on scratch folders.

## 3. Licence notices

Shipping one binary means shipping its notices: `xfc licenses` prints the
GPL-3.0 notice and the third-party notices already listed in the README
(X-Plane SDK, Dear ImGui, XPMP2, B612, AFV-Native), embedded from files in
the repo (find or add them under a `licenses/` folder; never paraphrase a
licence text).

## Constraints

- X-Plane may be running and the pilot may be flying. NEVER install into,
  delete from, or write to the real `~/X-Plane 12` (or whatever
  `XCOPILOT_XPLANE_ROOT` says). Every install check uses `--xplane
  <scratch>` with a fake `Resources/plugins/` tree you create (include a
  fake `xcopilot/` folder to prove it goes). `xfc plugin status` against the
  real folder is read-only and fine.
- Never call X-Plane's Web API. Never kill or restart xfc or X-Plane. Never
  touch the checkout's `logs/`, `models/`, `prefs.json`, `.env`, or the real
  `~/Library/Application Support/XFlightCrew`.
- `make -C plugin` (build only, never `install`) is fine; it writes
  `plugin/build/`.
- For the unpacked-resources check: copy the release binary to a scratch
  folder with NOTHING beside it and run it with `XCOPILOT_HOME=<scratch
  data>` from a cwd outside the repo, then `xfc paths`; check the unpacked
  tree and the marker. (Setting `HOME` may be refused by the sandbox;
  `XCOPILOT_HOME` is enough. But note XCOPILOT_HOME's resources is the
  folder itself: make sure the embedded unpack also applies when that
  folder has no `profiles/`, which is exactly the fresh-user case.)
- `XCOPILOT_BLACKBOX=off cargo test --workspace` passes, default features
  and with `embed-plugin` built at least once via the script.
- No em or en dashes. Format only files you touched.
- Docs: CLAUDE.md, one terse bullet near the paths bullet (embedded and
  unpacked, on-disk wins; `xfc plugin` installs, never reloads, backups out
  of `plugins/`). `docs/guide/install.md`: the single-binary install
  (`xfc plugin install`, `xfc voices`) alongside the from-source one.

## Done when

Tests pass; the release script builds `dist/xfc` with the plugin; a bare
copy runs `xfc paths` showing the unpacked tree; `xfc plugin install
--xplane <scratch>` removes the fake `xcopilot/`, backs up the old
`xflightcrew/` to the data folder and installs; `--dry-run` changes nothing;
`xfc plugin status` reads correctly against scratch and (read-only) the real
folder; `xfc licenses` prints. Show all outputs.
