//! The `xfc voices` screen: three tabs (Voices, Speech recognition, Cast),
//! a scrollable filtered list, downloads on worker threads so the list stays
//! live, a preview, and "use as" for every role. The Cast tab is who speaks
//! with what (`voice::cast`): a role's voice is picked from the INSTALLED
//! ones, a missing one installed with one key, the away-field pool edited;
//! every pick goes through `settings::set`, the one door.

use std::collections::BTreeMap;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::time::Duration;

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, List, ListItem, ListState, Paragraph, Wrap};
use ratatui::Frame;

use super::catalogue::{self, Kind, Row};
use super::download::{self, Progress};
use super::Store;
use crate::voice::cast;

/// From the worker threads to the screen.
enum Msg {
    Progress(String, Progress),
    Installed(String, Result<(), String>),
    Previewed(String, Result<(), String>),
}

/// A question the screen is waiting on.
#[derive(Clone, PartialEq, Eq)]
enum Prompt {
    Remove(String),
    UseAs(String),
}

/// Which tab is up.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Tab {
    List(Kind),
    Cast,
}

/// What a voice picked in the Cast tab is for.
#[derive(Clone, PartialEq, Eq, Debug)]
enum Target {
    /// A registry voice setting (a cast row's).
    Setting(&'static str),
    /// One more voice for the away-field pool.
    PoolAdd,
}

/// The installed-voice picker the Cast tab opens.
struct Pick {
    target: Target,
    choices: Vec<Row>,
    list: ListState,
}

struct App {
    store: Store,
    tab: Tab,
    rows: Vec<Row>,
    /// The Cast tab: one row per role, at the home field.
    cast: Vec<cast::Row>,
    /// The away-field pool's entries, as the registry has them.
    pool: Vec<String>,
    /// The Cast tab's pool list is open (add, remove).
    pool_open: bool,
    pick: Option<Pick>,
    filter: String,
    editing_filter: bool,
    list: ListState,
    downloads: BTreeMap<String, Progress>,
    previewing: Option<String>,
    prompt: Option<Prompt>,
    /// The last thing that happened, and whether it went well.
    notice: Option<(String, bool)>,
    /// A page of the one app: its sidebar switches the tabs, Esc goes back.
    in_app: bool,
}

impl App {
    fn new(store: Store) -> Self {
        let rows = store.rows(Kind::Piper);
        let mut list = ListState::default();
        list.select(Some(0));
        App {
            notice: store.note.clone().map(|n| (n, false)),
            store,
            tab: Tab::List(Kind::Piper),
            rows,
            cast: Vec::new(),
            pool: Vec::new(),
            pool_open: false,
            pick: None,
            filter: String::new(),
            editing_filter: false,
            list,
            downloads: BTreeMap::new(),
            previewing: None,
            prompt: None,
            in_app: false,
        }
    }

    fn refresh(&mut self) {
        match self.tab {
            Tab::List(k) => self.rows = self.store.rows(k),
            Tab::Cast => {
                self.cast = cast::cast_now(None);
                self.pool = crate::settings::live()
                    .atc_voice_pool
                    .split(',')
                    .map(str::trim)
                    .filter(|s| !s.is_empty())
                    .map(str::to_string)
                    .collect();
            }
        }
        let n = self.count();
        let sel = self.list.selected().unwrap_or(0).min(n.saturating_sub(1));
        self.list.select(Some(sel));
    }

    /// How many rows the cursor walks: the list, the cast (and its pool
    /// line), or the pool's entries.
    fn count(&self) -> usize {
        match self.tab {
            Tab::List(_) => self.shown().len(),
            Tab::Cast if self.pool_open => self.pool.len(),
            Tab::Cast => self.cast.len() + 1,
        }
    }

    fn shown(&self) -> Vec<&Row> {
        catalogue::filter(&self.rows, &self.filter, false)
    }

    fn selected(&self) -> Option<Row> {
        let i = self.list.selected()?;
        self.shown().get(i).map(|r| (*r).clone())
    }

    /// The cast row under the cursor (`None` on the pool line).
    fn cast_row(&self) -> Option<&cast::Row> {
        self.cast.get(self.list.selected()?)
    }

    fn step(&mut self, by: isize) {
        let n = self.count();
        if n == 0 {
            return;
        }
        let i = self.list.selected().unwrap_or(0) as isize + by;
        self.list.select(Some(i.clamp(0, n as isize - 1) as usize));
    }

    fn switch(&mut self, tab: Tab) {
        self.tab = tab;
        self.pool_open = false;
        self.list.select(Some(0));
        self.refresh();
    }
}

/// The screen as a pane of the one app (`src/app.rs`): its state and its
/// workers' channel, drawn into an area, fed its keys. `xfc voices` runs
/// the same pane full screen ([`run`]).
pub struct Pane {
    app: App,
    tx: Sender<Msg>,
    rx: Receiver<Msg>,
}

impl Pane {
    pub fn new(store: Store) -> Self {
        let (tx, rx) = channel();
        Pane {
            app: App::new(store),
            tx,
            rx,
        }
    }

    /// The workers' news (downloads, previews). Returns whether anything
    /// changed on screen.
    pub fn poll(&mut self) -> bool {
        let mut dirty = false;
        while let Ok(m) = self.rx.try_recv() {
            dirty = true;
            take(&mut self.app, m);
        }
        dirty
    }

    pub fn draw(&mut self, f: &mut Frame, area: ratatui::layout::Rect) {
        draw_in(f, &mut self.app, area);
    }

    /// One key. Returns true for the screen's own quit (q, Esc at the top).
    pub fn key(&mut self, code: KeyCode) -> bool {
        key(&mut self.app, code, &self.tx)
    }

    /// Show tab `i` (0 Voices, 1 Speech recognition, 2 Cast): the app's
    /// sidebar moved.
    pub fn show(&mut self, i: usize) {
        self.app.in_app = true;
        let tab = [Tab::List(Kind::Piper), Tab::List(Kind::Whisper), Tab::Cast][i.min(2)];
        if self.app.tab != tab {
            self.app.switch(tab);
        }
    }

    /// A field, a picker or a question is open: every key is the pane's.
    pub fn capturing(&self) -> bool {
        self.app.editing_filter || self.app.pick.is_some() || self.app.prompt.is_some()
    }

    /// The model files changed elsewhere (the starter set installed from
    /// the Checks tab): the rows and the cast again.
    pub fn refresh(&mut self) {
        self.app.store.refresh_uses();
        self.app.refresh();
    }

    /// Downloads still running, for the line printed at quit.
    pub fn downloads(&self) -> usize {
        self.app.downloads.len()
    }
}

/// Run the screen until q.
pub fn run(store: Store) -> std::io::Result<()> {
    let mut pane = Pane::new(store);
    let mut terminal = ratatui::init();
    let res = event_loop(&mut terminal, &mut pane);
    ratatui::restore();
    if pane.downloads() > 0 {
        eprintln!(
            "left with {} download(s) unfinished: nothing half-installed, \
             the parts are cleaned on the next run",
            pane.downloads()
        );
    }
    res
}

fn event_loop(terminal: &mut ratatui::DefaultTerminal, pane: &mut Pane) -> std::io::Result<()> {
    let mut dirty = true;
    loop {
        dirty |= pane.poll();
        if dirty {
            terminal.draw(|f| pane.draw(f, f.area()))?;
            dirty = false;
        }
        if !event::poll(Duration::from_millis(80))? {
            continue;
        }
        match event::read()? {
            Event::Key(k) if k.kind == KeyEventKind::Press => {
                dirty = true;
                if k.modifiers.contains(KeyModifiers::CONTROL) && k.code == KeyCode::Char('c') {
                    return Ok(());
                }
                if pane.key(k.code) {
                    return Ok(());
                }
            }
            Event::Resize(_, _) => dirty = true,
            _ => {}
        }
    }
}

/// One message from a worker.
fn take(app: &mut App, m: Msg) {
    match m {
        Msg::Progress(key, p) => {
            app.downloads.insert(key, p);
        }
        Msg::Installed(key, r) => {
            app.downloads.remove(&key);
            app.notice = Some(match r {
                Ok(()) => (format!("{key} installed, size and checksum verified"), true),
                Err(e) => (
                    format!("{key}: install failed, nothing installed: {e}"),
                    false,
                ),
            });
            app.refresh();
        }
        Msg::Previewed(key, r) => {
            app.previewing = None;
            if let Err(e) = r {
                app.notice = Some((format!("{key}: preview failed: {e}"), false));
            }
        }
    }
}

/// The tab after this one.
fn next_tab(t: Tab) -> Tab {
    match t {
        Tab::List(Kind::Piper) => Tab::List(Kind::Whisper),
        Tab::List(Kind::Whisper) => Tab::Cast,
        Tab::Cast => Tab::List(Kind::Piper),
    }
}

/// One key. Returns true to quit.
fn key(app: &mut App, code: KeyCode, tx: &Sender<Msg>) -> bool {
    if app.pick.is_some() {
        pick_key(app, code, tx);
        return false;
    }
    if app.editing_filter {
        match code {
            KeyCode::Esc => {
                app.filter.clear();
                app.editing_filter = false;
            }
            KeyCode::Enter => app.editing_filter = false,
            KeyCode::Backspace => {
                app.filter.pop();
            }
            KeyCode::Char(c) => app.filter.push(c),
            KeyCode::Down => app.step(1),
            KeyCode::Up => app.step(-1),
            _ => {}
        }
        app.list.select(Some(0));
        app.refresh();
        return false;
    }
    if let Some(prompt) = app.prompt.take() {
        answer(app, prompt, code);
        return false;
    }
    match code {
        KeyCode::Esc if app.pool_open => {
            app.pool_open = false;
            app.list.select(Some(app.cast.len()));
        }
        KeyCode::Char('q') | KeyCode::Esc => return true,
        KeyCode::Down | KeyCode::Char('j') => app.step(1),
        KeyCode::Up | KeyCode::Char('k') => app.step(-1),
        KeyCode::PageDown => app.step(15),
        KeyCode::PageUp => app.step(-15),
        KeyCode::Home => app.list.select(Some(0)),
        KeyCode::End => app.step(isize::MAX / 2),
        // `xfc voices` alone: Tab cycles its three tabs. In the app the
        // sidebar shows them (`Pane::show`).
        KeyCode::Tab => app.switch(next_tab(app.tab)),
        _ if app.tab == Tab::Cast => cast_key(app, code, tx),
        KeyCode::Char('/') => {
            app.editing_filter = true;
        }
        KeyCode::Enter => start_install(app, tx),
        KeyCode::Char('s') => start_starter(app, tx),
        KeyCode::Char('d') => {
            if let Some(r) = app.selected() {
                match catalogue::remove_gate(&r) {
                    Ok(()) => app.prompt = Some(Prompt::Remove(r.entry.key)),
                    Err(e) => app.notice = Some((e.to_string(), false)),
                }
            }
        }
        KeyCode::Char('p') => {
            if let Some(r) = app.selected() {
                start_preview(app, r, tx);
            }
        }
        KeyCode::Char('u') => {
            if let Some(r) = app.selected() {
                if !r.installed {
                    app.notice = Some((format!("install {} first (enter)", r.entry.key), false));
                } else if r.entry.kind == Kind::Whisper {
                    // One role for a speech model: no question to ask.
                    app.notice = Some(match super::use_as(&r, "speech recognition") {
                        Ok(m) => (m, true),
                        Err(e) => (e.to_string(), false),
                    });
                    after_pick(app);
                } else {
                    app.prompt = Some(Prompt::UseAs(r.entry.key));
                }
            }
        }
        _ => {}
    }
    false
}

/// A key on the Cast tab (the cast rows, or the pool's list).
fn cast_key(app: &mut App, code: KeyCode, tx: &Sender<Msg>) {
    if app.pool_open {
        match code {
            KeyCode::Char('a') => open_pick(app, Target::PoolAdd),
            KeyCode::Char('d') => {
                let Some(spec) = app.list.selected().and_then(|i| app.pool.get(i)).cloned() else {
                    return;
                };
                app.notice = Some(match super::edit_pool(&spec, false) {
                    Ok(m) => (m, true),
                    Err(e) => (e, false),
                });
                after_pick(app);
            }
            _ => {}
        }
        return;
    }
    match code {
        KeyCode::Enter => match app.cast_row().cloned() {
            None => {
                app.pool_open = true;
                app.list.select(Some(0));
                app.refresh();
            }
            Some(r) => match r.setting {
                Some(key) => open_pick(app, Target::Setting(key)),
                None => app.notice = Some((format!("{}: the pool decides here", r.role), false)),
            },
        },
        KeyCode::Char('i') => {
            let Some(r) = app.cast_row().cloned() else {
                return;
            };
            if r.installed {
                app.notice = Some((format!("{}'s voice is installed", r.role), true));
                return;
            }
            let found = catalogue::key_of_spec(&r.spec).and_then(|k| app.store.row(&k));
            match found {
                Some(row) => spawn_install(app, row, tx),
                None => {
                    app.notice = Some((
                        format!(
                            "{} is in no catalogue: pick another (enter)",
                            crate::settings::voice_name(&r.spec)
                        ),
                        false,
                    ))
                }
            }
        }
        _ => {}
    }
}

/// Open the installed-voice picker for a target.
fn open_pick(app: &mut App, target: Target) {
    let choices: Vec<Row> = app
        .store
        .rows(Kind::Piper)
        .into_iter()
        .filter(|r| r.installed)
        .collect();
    if choices.is_empty() {
        app.notice = Some((
            "no voice is installed: the Voices tab installs them".into(),
            false,
        ));
        return;
    }
    let mut list = ListState::default();
    list.select(Some(0));
    app.pick = Some(Pick {
        target,
        choices,
        list,
    });
}

/// A key in the picker: move, preview, use, cancel.
fn pick_key(app: &mut App, code: KeyCode, tx: &Sender<Msg>) {
    let Some(pick) = app.pick.as_mut() else {
        return;
    };
    let n = pick.choices.len();
    let at = pick.list.selected().unwrap_or(0);
    match code {
        KeyCode::Esc | KeyCode::Char('q') => app.pick = None,
        KeyCode::Down | KeyCode::Char('j') => pick.list.select(Some((at + 1).min(n - 1))),
        KeyCode::Up | KeyCode::Char('k') => pick.list.select(Some(at.saturating_sub(1))),
        KeyCode::Char('p') => {
            let r = pick.choices[at].clone();
            start_preview(app, r, tx);
        }
        KeyCode::Enter => {
            let spec = super::voice_spec(&pick.choices[at].entry.key);
            let result = match &pick.target {
                Target::Setting(key) => super::set_key(key, &spec),
                Target::PoolAdd => super::edit_pool(&spec, true),
            };
            app.pick = None;
            app.notice = Some(match result {
                Ok(m) => (m, true),
                Err(e) => (e, false),
            });
            after_pick(app);
        }
        _ => {}
    }
}

/// After a pick: the tags and the cast follow the registry.
fn after_pick(app: &mut App) {
    app.store.refresh_uses();
    app.refresh();
}

fn answer(app: &mut App, prompt: Prompt, code: KeyCode) {
    match prompt {
        Prompt::Remove(k) => {
            if code != KeyCode::Char('y') {
                app.notice = Some((format!("{k} kept"), true));
                return;
            }
            // Re-gated on the current rows: nothing changed under the question.
            let Some(r) = app.rows.iter().find(|r| r.entry.key == k).cloned() else {
                return;
            };
            app.notice = Some(
                match catalogue::remove_gate(&r)
                    .and_then(|()| download::remove(r.entry.kind, &k, &app.store.dir))
                {
                    Ok(()) => (format!("{k} removed"), true),
                    Err(e) => (e.to_string(), false),
                },
            );
            app.refresh();
        }
        Prompt::UseAs(k) => {
            let role = match code {
                KeyCode::Char('c') => "copilot",
                KeyCode::Char('b') => "cabin",
                KeyCode::Char('t') => "tower",
                KeyCode::Char('a') => "approach",
                KeyCode::Char('d') => "departure",
                KeyCode::Char('i') => "ATIS",
                KeyCode::Char('o') => "pool",
                _ => return,
            };
            let Some(r) = app.rows.iter().find(|r| r.entry.key == k).cloned() else {
                return;
            };
            app.notice = Some(match super::use_as(&r, role) {
                Ok(m) => (m, true),
                Err(e) => (e.to_string(), false),
            });
            after_pick(app);
        }
    }
}

fn start_install(app: &mut App, tx: &Sender<Msg>) {
    let Some(r) = app.selected() else { return };
    spawn_install(app, r, tx);
}

/// `s`: every missing starter item, each on its own worker as `enter` does.
fn start_starter(app: &mut App, tx: &Sender<Msg>) {
    let status = app.store.starter_status();
    if status.complete() {
        app.notice = Some(("the starter set is complete".into(), true));
        return;
    }
    let rows = app.store.starter_rows();
    if rows.is_empty() {
        app.notice = Some((
            format!(
                "{} not in the catalogue: nothing to download",
                status.unknown.join(", ")
            ),
            false,
        ));
        return;
    }
    let n = rows.len();
    for r in rows {
        spawn_install(app, r, tx);
    }
    app.notice = Some((
        format!(
            "downloading the starter set: {n} item(s), {}",
            catalogue::human_size(status.bytes)
        ),
        true,
    ));
}

/// Install one row on a worker thread, its progress on the list.
fn spawn_install(app: &mut App, r: Row, tx: &Sender<Msg>) {
    let key = r.entry.key.clone();
    if r.installed {
        app.notice = Some((format!("{key} is installed"), true));
        return;
    }
    if r.entry.is_local() {
        app.notice = Some((format!("{key} is local: nothing to download"), false));
        return;
    }
    if app.downloads.contains_key(&key) {
        return;
    }
    app.downloads.insert(
        key.clone(),
        Progress {
            file: 0,
            files: r.entry.files.len(),
            done: 0,
            total: r.entry.download_size(),
        },
    );
    app.notice = Some((format!("downloading {key}"), true));
    let dir = app.store.dir.clone();
    let tx = tx.clone();
    std::thread::spawn(move || {
        let mut last = u8::MAX;
        let result = download::install(&r.entry, &dir, &mut |p| {
            if p.percent() != last {
                last = p.percent();
                let _ = tx.send(Msg::Progress(key.clone(), p));
            }
        });
        let _ = tx.send(Msg::Installed(key, result.map_err(|e| format!("{e:#}"))));
    });
}

fn start_preview(app: &mut App, r: Row, tx: &Sender<Msg>) {
    if app.previewing.is_some() {
        return;
    }
    if r.entry.kind != Kind::Piper || !r.installed {
        app.notice = Some((
            format!("{}: install a voice to preview it", r.entry.key),
            false,
        ));
        return;
    }
    let key = r.entry.key.clone();
    app.previewing = Some(key.clone());
    let tx = tx.clone();
    std::thread::spawn(move || {
        let result = super::preview(&r).map_err(|e| format!("{e:#}"));
        let _ = tx.send(Msg::Previewed(key, result));
    });
}

#[cfg(test)]
fn draw(f: &mut Frame, app: &mut App) {
    draw_in(f, app, f.area());
}

fn draw_in(f: &mut Frame, app: &mut App, area: ratatui::layout::Rect) {
    let [head, body, notice, hint] = Layout::vertical([
        Constraint::Length(1),
        Constraint::Min(3),
        Constraint::Length(2),
        Constraint::Length(1),
    ])
    .areas(area);

    let tab = |t: Tab, name: &'static str| {
        let style = if app.tab == t {
            Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD | Modifier::UNDERLINED)
        } else {
            Style::default().fg(Color::DarkGray)
        };
        Span::styled(name, style)
    };
    let mut head_spans = vec![
        Span::raw(" "),
        tab(Tab::List(Kind::Piper), "Voices"),
        Span::raw("   "),
        tab(Tab::List(Kind::Whisper), "Speech recognition"),
        Span::raw("   "),
        tab(Tab::Cast, "Cast"),
    ];
    if let Tab::List(_) = app.tab {
        let installed: Vec<&Row> = app.rows.iter().filter(|r| r.installed).collect();
        let bytes: u64 = installed.iter().map(|r| r.disk_bytes).sum();
        let filter_style = if app.editing_filter {
            Style::default().fg(Color::Cyan)
        } else {
            Style::default().fg(Color::DarkGray)
        };
        let cursor = if app.editing_filter { "▏" } else { "" };
        head_spans.push(Span::styled(
            format!(
                "      installed {} · {}",
                installed.len(),
                catalogue::human_size(bytes)
            ),
            Style::default().fg(Color::Gray),
        ));
        head_spans.push(Span::styled(
            format!("      filter: {}{cursor}", app.filter),
            filter_style,
        ));
    }
    let status = app.store.starter_status();
    let style = if status.complete() {
        Style::default().fg(Color::Green)
    } else {
        Style::default().fg(Color::Yellow)
    };
    head_spans.push(Span::styled(
        format!("      {}", catalogue::starter_line(&status, "press s")),
        style,
    ));
    f.render_widget(Paragraph::new(Line::from(head_spans)), head);

    if app.pick.is_some() {
        draw_pick(f, app, body);
    } else if app.tab == Tab::Cast {
        draw_cast(f, app, body);
    } else {
        draw_list(f, app, body);
    }

    let notice_line = match &app.prompt {
        Some(Prompt::Remove(k)) => Line::styled(
            format!(" remove {k}? y to remove, any other key keeps it"),
            Style::default().fg(Color::Yellow),
        ),
        Some(Prompt::UseAs(k)) => Line::styled(
            format!(
                " use {k} as: c copilot  b cabin  t tower  a approach  d departure  i ATIS  o add to the away pool   esc cancel"
            ),
            Style::default().fg(Color::Yellow),
        ),
        None => match &app.notice {
            Some((n, ok)) => Line::styled(
                format!(" {n}"),
                Style::default().fg(if *ok { Color::Green } else { Color::Red }),
            ),
            None => Line::raw(""),
        },
    };
    f.render_widget(
        Paragraph::new(notice_line).wrap(Wrap { trim: false }),
        notice,
    );

    let hint_text = if app.pick.is_some() {
        " ↑↓ move  enter use  p preview  esc cancel"
    } else if app.editing_filter {
        " type to filter   enter keep   esc clear"
    } else if app.tab == Tab::Cast && app.pool_open {
        " ↑↓ move  a add a voice  d take it out  esc back"
    } else if app.tab == Tab::Cast {
        " ↑↓ move  enter pick a voice (the pool: edit it)  i install a missing one"
    } else {
        " ↑↓ move  / filter  enter install  s starter  d uninstall  p preview  u use as"
    };
    let way_out = if app.in_app {
        "  ← back"
    } else {
        "  tab switch  q quit"
    };
    let hint_text = if app.pick.is_some() || app.editing_filter || app.pool_open {
        hint_text.to_string()
    } else {
        format!("{hint_text}{way_out}")
    };
    f.render_widget(
        Paragraph::new(hint_text).style(Style::default().fg(Color::DarkGray)),
        hint,
    );
}

/// The Voices and Speech recognition tabs: the catalogue joined with disk.
fn draw_list(f: &mut Frame, app: &mut App, body: ratatui::layout::Rect) {
    let shown = app.shown();
    let items: Vec<ListItem> = shown
        .iter()
        .map(|r| {
            let [key, place, quality, spk, size] = super::row_columns(r);
            let dl = app.downloads.get(&r.entry.key);
            let (mark, mark_style) = match (dl, r.installed) {
                (Some(_), _) => ("↓", Style::default().fg(Color::Yellow)),
                (None, true) => ("✓", Style::default().fg(Color::Green)),
                (None, false) => (" ", Style::default()),
            };
            let right = match dl {
                Some(p) if p.files > 1 => {
                    format!("{}%  file {}/{}", p.percent(), p.file + 1, p.files)
                }
                Some(p) => format!("{}%", p.percent()),
                None if app.previewing.as_deref() == Some(r.entry.key.as_str()) => {
                    "speaking".into()
                }
                None => r.tag(),
            };
            ListItem::new(Line::from(vec![
                Span::styled(format!(" {mark} "), mark_style),
                Span::raw(format!(
                    "{key:<40} {place:<16} {quality:<8} {spk:>7} {size:>8}   "
                )),
                Span::styled(right, Style::default().fg(Color::Cyan)),
            ]))
        })
        .collect();
    let empty = items.is_empty();
    let list = List::new(items)
        .block(Block::default().borders(Borders::TOP | Borders::BOTTOM))
        .highlight_style(Style::default().add_modifier(Modifier::REVERSED));
    f.render_stateful_widget(list, body, &mut app.list);
    if empty {
        let what = if app.rows.is_empty() {
            "nothing to list: no catalogue and nothing installed"
        } else {
            "nothing matches the filter"
        };
        f.render_widget(
            Paragraph::new(format!("   {what}")).style(Style::default().fg(Color::DarkGray)),
            Layout::vertical([Constraint::Length(1), Constraint::Length(1)]).split(body)[1],
        );
    }
}

/// The Cast tab: who speaks with what at the home field, where each voice
/// came from, a missing one red; then the away-field pool's line (or, open,
/// its entries).
fn draw_cast(f: &mut Frame, app: &mut App, body: ratatui::layout::Rect) {
    let items: Vec<ListItem> = if app.pool_open {
        app.pool
            .iter()
            .map(|spec| {
                let ok = crate::voice::voice_installed(spec);
                ListItem::new(Line::from(vec![
                    Span::styled(
                        if ok { " ✓ " } else { " ✗ " },
                        Style::default().fg(if ok { Color::Green } else { Color::Red }),
                    ),
                    Span::raw(crate::settings::voice_name(spec)),
                    Span::styled(
                        if ok {
                            ""
                        } else {
                            "   not installed: never drawn"
                        },
                        Style::default().fg(Color::Red),
                    ),
                ]))
            })
            .collect()
    } else {
        let mut items: Vec<ListItem> = app
            .cast
            .iter()
            .map(|r| {
                let name = if r.spec.is_empty() {
                    "(macOS say)".to_string()
                } else {
                    crate::settings::voice_name(&r.spec)
                };
                let mut spans = vec![
                    Span::styled(
                        if r.installed { " ✓ " } else { " ✗ " },
                        Style::default().fg(if r.installed {
                            Color::Green
                        } else {
                            Color::Red
                        }),
                    ),
                    Span::raw(format!("{:<10} ", r.role)),
                    Span::styled(
                        format!("{name:<40} "),
                        Style::default().fg(if r.installed {
                            Color::White
                        } else {
                            Color::Red
                        }),
                    ),
                    Span::styled(r.source.describe(), Style::default().fg(Color::Cyan)),
                ];
                if !r.installed {
                    let instead = r
                        .instead
                        .as_deref()
                        .map(|i| format!(", {} speaks instead", crate::settings::voice_name(i)))
                        .unwrap_or_default();
                    spans.push(Span::styled(
                        format!("   missing: i installs{instead}"),
                        Style::default().fg(Color::Red),
                    ));
                }
                ListItem::new(Line::from(spans))
            })
            .collect();
        let on_disk = app
            .pool
            .iter()
            .filter(|s| crate::voice::voice_installed(s))
            .count();
        items.push(ListItem::new(Line::from(vec![
            Span::raw("   "),
            Span::raw(format!("{:<10} ", "away pool")),
            Span::raw(format!(
                "{} voices, {on_disk} installed: every other field's controllers",
                app.pool.len()
            )),
        ])));
        items
    };
    let title = if app.pool_open {
        " away-field pool (a running crew reads it at the next start) "
    } else {
        " who speaks at the home field "
    };
    let list = List::new(items)
        .block(Block::default().borders(Borders::ALL).title(title))
        .highlight_style(Style::default().add_modifier(Modifier::REVERSED));
    f.render_stateful_widget(list, body, &mut app.list);
}

/// The installed-voice picker over the body.
fn draw_pick(f: &mut Frame, app: &mut App, body: ratatui::layout::Rect) {
    let previewing = app.previewing.clone();
    let Some(pick) = app.pick.as_mut() else {
        return;
    };
    let what = match &pick.target {
        Target::Setting(key) => crate::settings::ALL
            .iter()
            .find(|s| s.key == *key)
            .map_or(*key, |s| s.label)
            .to_string(),
        Target::PoolAdd => "the away-field pool".to_string(),
    };
    let items: Vec<ListItem> = pick
        .choices
        .iter()
        .map(|r| {
            let [key, place, quality, spk, _] = super::row_columns(r);
            let right = if previewing.as_deref() == Some(r.entry.key.as_str()) {
                "speaking".to_string()
            } else {
                r.tag()
            };
            ListItem::new(Line::from(vec![
                Span::raw(format!(" {key:<40} {place:<16} {quality:<8} {spk:>7}   ")),
                Span::styled(right, Style::default().fg(Color::Cyan)),
            ]))
        })
        .collect();
    let list = List::new(items)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!(" a voice for {what} (installed voices) ")),
        )
        .highlight_style(Style::default().add_modifier(Modifier::REVERSED));
    f.render_stateful_widget(list, body, &mut pick.list);
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::backend::TestBackend;

    fn store(dir: std::path::PathBuf) -> Store {
        let in_dir = |f: &str| dir.join(f).to_string_lossy().into_owned();
        let starter = catalogue::starter_items(
            &catalogue::Configured {
                tower: in_dir("en_GB-alba-medium.onnx"),
                copilot: in_dir("en_GB-alan-medium.onnx"),
                ..Default::default()
            },
            true,
        );
        Store {
            dir,
            piper: catalogue::parse_piper(include_str!("../../bench/voices_catalogue.json"))
                .unwrap(),
            whisper: catalogue::whisper_catalogue(),
            uses: catalogue::uses(&catalogue::Configured {
                tower: "models/en_GB-alba-medium.onnx".into(),
                whisper: "models/ggml-base.en.bin".into(),
                ..Default::default()
            }),
            starter,
            note: None,
        }
    }

    fn render(app: &mut App) -> String {
        let mut terminal = ratatui::Terminal::new(TestBackend::new(140, 14)).unwrap();
        terminal.draw(|f| draw(f, app)).unwrap();
        let buf = terminal.backend().buffer().clone();
        (0..buf.area.height)
            .map(|y| {
                (0..buf.area.width)
                    .map(|x| buf[(x, y)].symbol().to_string())
                    .collect::<String>()
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    #[test]
    fn the_screen_lists_marks_tags_and_filters() {
        let dir = std::env::temp_dir().join(format!("xfc-voices-screen-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("en_GB-alba-medium.onnx"), b"x").unwrap();
        std::fs::write(dir.join("en_GB-alba-medium.onnx.json"), b"x").unwrap();
        let mut app = App::new(store(dir.clone()));
        let screen = render(&mut app);
        assert!(screen.contains("installed 1"), "{screen}");
        assert!(screen.contains("✓ en_GB-alba-medium"), "{screen}");
        assert!(screen.contains("tower"), "{screen}");
        assert!(screen.contains("109 spk"), "{screen}");
        // alba is on disk, alan is not: one missing, priced, `s` named.
        assert!(
            screen.contains("starter set: 1 missing, 63 MB, press s"),
            "{screen}"
        );

        let (tx, _rx) = channel();
        for c in "/de_DE".chars() {
            key(&mut app, KeyCode::Char(c), &tx);
        }
        let screen = render(&mut app);
        assert!(screen.contains("de_DE-thorsten-high"), "{screen}");
        assert!(!screen.contains("en_GB-alba-medium"), "{screen}");

        // An in-use voice is refused at the key, naming who.
        key(&mut app, KeyCode::Esc, &tx);
        app.list.select(Some(2));
        assert_eq!(app.selected().unwrap().entry.key, "en_GB-alba-medium");
        key(&mut app, KeyCode::Char('d'), &tx);
        assert!(app.prompt.is_none());
        assert!(app.notice.as_ref().unwrap().0.contains("tower"));

        key(&mut app, KeyCode::Tab, &tx);
        let screen = render(&mut app);
        assert!(screen.contains("ggml-large-v3-turbo"), "{screen}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The Cast tab lists every role with its voice and where it came from,
    /// a missing one red with its one-key install, then the pool's line;
    /// enter on a role opens the picker of INSTALLED voices only, and esc
    /// closes it without a pick. Nothing here writes prefs.json.
    #[test]
    fn the_cast_tab_shows_every_role_and_picks_from_installed_voices() {
        let dir = std::env::temp_dir().join(format!("xfc-voices-cast-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("en_GB-alba-medium.onnx"), b"x").unwrap();
        std::fs::write(dir.join("en_GB-alba-medium.onnx.json"), b"x").unwrap();
        let mut app = App::new(store(dir.clone()));
        let (tx, _rx) = channel();
        key(&mut app, KeyCode::Tab, &tx);
        key(&mut app, KeyCode::Tab, &tx);
        assert_eq!(app.tab, Tab::Cast);
        // The cast is the live registry's (whatever this machine has).
        assert_eq!(app.cast.len(), 8);
        let mut terminal = ratatui::Terminal::new(TestBackend::new(160, 20)).unwrap();
        terminal.draw(|f| draw(f, &mut app)).unwrap();
        let buf = terminal.backend().buffer().clone();
        let screen: String = (0..buf.area.height)
            .map(|y| {
                (0..buf.area.width)
                    .map(|x| buf[(x, y)].symbol().to_string())
                    .collect::<String>()
                    + "\n"
            })
            .collect();
        for role in [
            "copilot",
            "cabin",
            "tower",
            "ground",
            "approach",
            "ATIS",
            "away pool",
        ] {
            assert!(screen.contains(role), "{role}: {screen}");
        }
        // Enter on the copilot row: the picker holds only installed voices.
        app.list.select(Some(0));
        key(&mut app, KeyCode::Enter, &tx);
        let pick = app.pick.as_ref().expect("picker open");
        assert_eq!(pick.target, Target::Setting("copilot_voice"));
        assert!(pick.choices.iter().all(|r| r.installed));
        assert_eq!(pick.choices.len(), 1);
        key(&mut app, KeyCode::Esc, &tx);
        assert!(app.pick.is_none());
        // The pool line opens the pool's list.
        app.list.select(Some(app.cast.len()));
        key(&mut app, KeyCode::Enter, &tx);
        assert!(app.pool_open);
        key(&mut app, KeyCode::Esc, &tx);
        assert!(!app.pool_open);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
