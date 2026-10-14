//! The `xfc voices` screen: two tabs (Voices, Speech recognition), a
//! scrollable filtered list, downloads on worker threads so the list stays
//! live, a preview, and "use as" for the copilot and the cabin.

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

struct App {
    store: Store,
    tab: Kind,
    rows: Vec<Row>,
    filter: String,
    editing_filter: bool,
    list: ListState,
    downloads: BTreeMap<String, Progress>,
    previewing: Option<String>,
    prompt: Option<Prompt>,
    /// The last thing that happened, and whether it went well.
    notice: Option<(String, bool)>,
}

impl App {
    fn new(store: Store) -> Self {
        let rows = store.rows(Kind::Piper);
        let mut list = ListState::default();
        list.select(Some(0));
        App {
            notice: store.note.clone().map(|n| (n, false)),
            store,
            tab: Kind::Piper,
            rows,
            filter: String::new(),
            editing_filter: false,
            list,
            downloads: BTreeMap::new(),
            previewing: None,
            prompt: None,
        }
    }

    fn refresh(&mut self) {
        self.rows = self.store.rows(self.tab);
        let n = self.shown().len();
        let sel = self.list.selected().unwrap_or(0).min(n.saturating_sub(1));
        self.list.select(Some(sel));
    }

    fn shown(&self) -> Vec<&Row> {
        catalogue::filter(&self.rows, &self.filter, false)
    }

    fn selected(&self) -> Option<Row> {
        let i = self.list.selected()?;
        self.shown().get(i).map(|r| (*r).clone())
    }

    fn step(&mut self, by: isize) {
        let n = self.shown().len();
        if n == 0 {
            return;
        }
        let i = self.list.selected().unwrap_or(0) as isize + by;
        self.list.select(Some(i.clamp(0, n as isize - 1) as usize));
    }
}

/// Run the screen until q.
pub fn run(store: Store) -> std::io::Result<()> {
    let (tx, rx) = channel();
    let mut app = App::new(store);
    let mut terminal = ratatui::init();
    let res = event_loop(&mut terminal, &mut app, &tx, &rx);
    ratatui::restore();
    if !app.downloads.is_empty() {
        eprintln!(
            "left with {} download(s) unfinished: nothing half-installed, \
             the parts are cleaned on the next run",
            app.downloads.len()
        );
    }
    res
}

fn event_loop(
    terminal: &mut ratatui::DefaultTerminal,
    app: &mut App,
    tx: &Sender<Msg>,
    rx: &Receiver<Msg>,
) -> std::io::Result<()> {
    let mut dirty = true;
    loop {
        while let Ok(m) = rx.try_recv() {
            dirty = true;
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
        if dirty {
            terminal.draw(|f| draw(f, app))?;
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
                if key(app, k.code, tx) {
                    return Ok(());
                }
            }
            Event::Resize(_, _) => dirty = true,
            _ => {}
        }
    }
}

/// One key. Returns true to quit.
fn key(app: &mut App, code: KeyCode, tx: &Sender<Msg>) -> bool {
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
        KeyCode::Char('q') | KeyCode::Esc => return true,
        KeyCode::Down | KeyCode::Char('j') => app.step(1),
        KeyCode::Up | KeyCode::Char('k') => app.step(-1),
        KeyCode::PageDown => app.step(15),
        KeyCode::PageUp => app.step(-15),
        KeyCode::Home => app.list.select(Some(0)),
        KeyCode::End => app.step(isize::MAX / 2),
        KeyCode::Tab | KeyCode::Left | KeyCode::Right => {
            app.tab = match app.tab {
                Kind::Piper => Kind::Whisper,
                Kind::Whisper => Kind::Piper,
            };
            app.list.select(Some(0));
            app.refresh();
        }
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
        KeyCode::Char('p') => start_preview(app, tx),
        KeyCode::Char('u') => {
            if let Some(r) = app.selected() {
                if r.entry.kind == Kind::Whisper {
                    app.notice = Some((
                        format!(
                            "speech recognition is set with WHISPER_MODEL={} in .env; {}",
                            crate::config::model_path(&format!("{}.bin", r.entry.key)),
                            super::RESTART_NOTE
                        ),
                        true,
                    ));
                } else if !r.installed {
                    app.notice = Some((format!("install {} first (enter)", r.entry.key), false));
                } else {
                    app.prompt = Some(Prompt::UseAs(r.entry.key));
                }
            }
        }
        _ => {}
    }
    false
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
                KeyCode::Char('a') => {
                    app.notice = Some((super::atc_hint(&k), true));
                    return;
                }
                _ => return,
            };
            let Some(r) = app.rows.iter().find(|r| r.entry.key == k).cloned() else {
                return;
            };
            app.notice = Some(match super::use_as(&r, role) {
                Ok(m) => (m, true),
                Err(e) => (e.to_string(), false),
            });
            // The tags follow the pick: this screen's own view of the roles.
            if app.notice.as_ref().is_some_and(|n| n.1) {
                for (key, u) in app.store.uses.iter_mut() {
                    if u.role == role {
                        key.clone_from(&k);
                    }
                }
                app.refresh();
            }
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

fn start_preview(app: &mut App, tx: &Sender<Msg>) {
    let Some(r) = app.selected() else { return };
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

fn draw(f: &mut Frame, app: &mut App) {
    let [head, body, notice, hint] = Layout::vertical([
        Constraint::Length(1),
        Constraint::Min(3),
        Constraint::Length(2),
        Constraint::Length(1),
    ])
    .areas(f.area());

    let installed: Vec<&Row> = app.rows.iter().filter(|r| r.installed).collect();
    let bytes: u64 = installed.iter().map(|r| r.disk_bytes).sum();
    let tab = |k: Kind, name: &'static str| {
        let style = if app.tab == k {
            Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD | Modifier::UNDERLINED)
        } else {
            Style::default().fg(Color::DarkGray)
        };
        Span::styled(name, style)
    };
    let filter_style = if app.editing_filter {
        Style::default().fg(Color::Cyan)
    } else {
        Style::default().fg(Color::DarkGray)
    };
    let cursor = if app.editing_filter { "▏" } else { "" };
    f.render_widget(
        Paragraph::new(Line::from(vec![
            Span::raw(" "),
            tab(Kind::Piper, "Voices"),
            Span::raw("   "),
            tab(Kind::Whisper, "Speech recognition"),
            Span::styled(
                format!(
                    "      installed {} · {}",
                    installed.len(),
                    catalogue::human_size(bytes)
                ),
                Style::default().fg(Color::Gray),
            ),
            Span::styled(
                format!("      filter: {}{cursor}", app.filter),
                filter_style,
            ),
            {
                let status = app.store.starter_status();
                let style = if status.complete() {
                    Style::default().fg(Color::Green)
                } else {
                    Style::default().fg(Color::Yellow)
                };
                Span::styled(
                    format!("      {}", catalogue::starter_line(&status, "press s")),
                    style,
                )
            },
        ])),
        head,
    );

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

    let notice_line = match &app.prompt {
        Some(Prompt::Remove(k)) => Line::styled(
            format!(" remove {k}? y to remove, any other key keeps it"),
            Style::default().fg(Color::Yellow),
        ),
        Some(Prompt::UseAs(k)) => Line::styled(
            format!(
                " use {k} as: c copilot   b cabin   a ATC (shows the variable)   esc cancel   ({})",
                super::RESTART_NOTE
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

    let hint_text = if app.editing_filter {
        " type to filter   enter keep   esc clear"
    } else {
        " ↑↓ move  tab switch  / filter  enter install  s starter  d uninstall  p preview  u use as  q quit"
    };
    f.render_widget(
        Paragraph::new(hint_text).style(Style::default().fg(Color::DarkGray)),
        hint,
    );
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
}
