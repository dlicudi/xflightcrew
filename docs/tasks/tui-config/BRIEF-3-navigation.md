# Part 3: one navigation model, sidebar and content, mouse too (2026-10-15)

Read `docs/tasks/COMMON.md` first. Parts 1 and 2 are merged (`src/app.rs`
is the one app; `ui::SettingsPane`, `voices::screen::Pane`,
`home::screen::Checks`, `ui::Dashboard` are its pages).

## Why

The pilot (2026-10-15): navigation must improve dramatically; Tab and cursor
for different menus is unintuitive. Today Tab cycles ~16 top tabs (nine are
settings sections), each page has its own letters (up to 20), Left/Right
steps a value in Settings but switches inner tabs in Voices, Esc means
something different per page. The pilot chose a sidebar and the mouse.

## The model (the same on every page)

```
+ XFlightCrew --------+- Voices > Cast ------------------------+
|   Flight            |  copilot    en_GB-alan     your pick   |
|   Checks        *   |  cabin      hfc_female     built-in    |
| > Voices            |> tower      vctk#3         pool        |
|   Settings          |  ...                                   |
|     Crew            |                                        |
|     ATC  ...        |                                        |
|   Plugin            |                                        |
|   Files             |                                        |
+---------------------+----------------------------------------+
| up/down move  left back  enter change  space preview  / ?    |
+--------------------------------------------------------------+
```

- **Two focus levels**: the sidebar and the page. Focus is always visible
  (highlighted border/title on the focused side, a cursor row on each).
- **Sidebar**: Up/Down (and `j`/`k`) move; Right or Enter goes into the page;
  `1`-`9` jump to a top-level page. Settings is a parent whose sections are
  indented children (each a page); Checks shows a red dot when a required
  check is red. Moving in the sidebar shows the page at once (no Enter
  needed to preview it).
- **Page**: Up/Down move rows, PageUp/PageDown/Home/End; Enter acts on the
  row (edit, pick, install, fix); Left or Esc returns to the sidebar (or out
  of an editor/picker first: Esc always goes back exactly ONE level, never
  quits). Inner tabs (Voices: Piper, Whisper, Cast) become sidebar children
  too, never a second tab bar.
- **Editing a value**: Enter opens it; a choice or bool cycles with
  Left/Right or Space INSIDE the editor; a number steps with Left/Right and
  takes typed digits; text and paths are typed; Enter saves, Esc cancels. No
  value changes from the row without Enter (today Left/Right on a row edits,
  which collides with "back").
- **Few global keys**: `/` search (results a page; Enter jumps to the
  setting in its section), `?` a key overlay for the focused page, `R`
  restart crew (the existing gate), `q` quit only from the sidebar (from a
  page it is ignored or goes back; never loses typed text), Ctrl-C quits.
- **Per-page actions stay, but are LISTED**: each page declares its actions
  (key, label, enabled-when) in one table the footer and `?` render from;
  the footer shows only what works for the focused row. Prefer Enter plus a
  small action menu over a letter where a page has more than ~4 letters.
  Space is preview on Voices rows.
- **Flight page**: the typed command line is reached by Enter (or `:`), not
  by capturing every letter; while typing, every key is text and Esc leaves.
- **Mouse** (crossterm mouse capture): click a sidebar entry, click a row
  (selects it; a second click or double click is Enter), scroll wheel scrolls
  the list under the pointer, click a footer key acts. Keys remain the full
  interface. Restore the terminal's mouse mode on every exit path, panic
  included (the restart re-exec too).
- Small terminals: below ~60 columns the sidebar collapses to a one-line
  page picker (Left/Esc opens it); every page still usable at 40x10.

## Constraints

- One key table, one dispatcher: pages stop reading raw `KeyCode` for
  navigation. A test walks every page with only the arrow keys, Enter and Esc
  and reaches every row and every action (no page needs a letter to be
  usable).
- A test that Esc from any depth reaches the sidebar in at most N presses and
  never quits; that `q` on a page never quits.
- A test that every action key a page handles appears in its footer or `?`.
- `TestBackend` render tests at 120x32, 80x24, 40x10 for every page.
- Keep `xfc voices` standalone working (same Pane, its own frame).
- No flight loop, command loop logic, plugin or EFB change.

## Done means

`XCOPILOT_BLACKBOX=off cargo test --workspace` green; report the final key
map per page, what moved from a letter to Enter or a menu, and anything left.
