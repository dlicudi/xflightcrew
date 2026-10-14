//! The home screen's TUI, in the house style of `src/ui.rs` and
//! `src/voices/screen.rs`: the checks with their fixes, the voices, the
//! settings registry (the dashboard's own Settings view, lifted) and the
//! files. Live facts arrive from worker threads; nothing is polled.

use std::sync::mpsc::{channel, Receiver, Sender};
use std::time::Duration;

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Paragraph, Wrap};
use ratatui::Frame;

use super::{Check, Facts, Fix, Leave, Mark, Summary};
use crate::config::Config;
use crate::ui::SettingsPane;
use crate::voices::catalogue;
use crate::voices::download::{self, Progress};
use crate::voices::Store;

/// From the worker threads to the screen.
enum Msg {
    WebApi(Result<(), String>),
    Mic(Option<String>),
    Store(Box<Store>),
    /// key, progress, which item (from 0), of how many.
    Progress(String, Progress, usize, usize),
    Installed(String, Result<(), String>),
    StarterDone,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum View {
    Home,
    Settings,
    Files,
}

/// What a key asks of the event loop.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Act {
    None,
    Leave(Leave),
    OpenVoices,
    Recheck,
    AskWebApi,
    InstallStarter,
}

/// The rows: the five checks, then these.
const VOICES_ROW: usize = 5;
const SETTINGS_ROW: usize = 6;
const FILES_ROW: usize = 7;
const ROWS: usize = 8;

struct App {
    config: Config,
    facts: Facts,
    summary: Summary,
    sel: usize,
    view: View,
    settings: SettingsPane,
    store: Option<Store>,
    /// key, progress, which item, of how many.
    downloading: Option<(String, Progress, usize, usize)>,
    notice: Option<(String, bool)>,
    /// `r` with a red check warns once; the second `r` runs.
    warned_run: bool,
}

impl App {
    fn new(config: Config, facts: Facts, summary: Summary) -> Self {
        App {
            config,
            facts,
            summary,
            sel: 0,
            view: View::Home,
            settings: SettingsPane::default(),
            store: None,
            downloading: None,
            notice: None,
            warned_run: false,
        }
    }

    fn checks(&self) -> Vec<Check> {
        super::checks(&self.facts)
    }

    /// The offline facts again (after a fix, an install, the voices
    /// screen), keeping what was asked live.
    fn refresh_offline(&mut self) {
        let fresh = super::offline(&self.config);
        let web_api = self.facts.web_api.take();
        let mic = self.facts.mic.take();
        self.facts = Facts {
            web_api,
            mic,
            starter: self.store.as_ref().map(Store::starter_status),
            ..fresh
        };
        self.summary = super::summary();
    }

    /// The red checks, by name, for the warning on `r`.
    fn red(&self) -> Vec<String> {
        self.checks()
            .iter()
            .filter(|c| c.mark == Mark::Bad || c.required)
            .map(|c| c.text.clone())
            .collect()
    }
}

/// Run the screen until `r` or `q`.
pub fn run(config: &Config) -> std::io::Result<Leave> {
    let (tx, rx) = channel();
    let mut app = App::new(config.clone(), super::offline(config), super::summary());
    ask_live(&app.config, &tx);
    load_store(&app.config, &tx);
    let mut terminal = ratatui::init();
    let res = event_loop(&mut terminal, &mut app, &tx, &rx);
    ratatui::restore();
    if app.downloading.is_some() {
        eprintln!(
            "left with a download unfinished: nothing half-installed, the parts are cleaned on the next run"
        );
    }
    res
}

/// The two live facts, each on its own worker: ONE Web API GET and the
/// microphone's name. Only on open and on `c`, never polled.
fn ask_live(config: &Config, tx: &Sender<Msg>) {
    ask_web_api(config, tx);
    ask_mic(tx);
}

fn ask_web_api(config: &Config, tx: &Sender<Msg>) {
    let base = config.xplane_base_url.clone();
    let t = tx.clone();
    std::thread::spawn(move || {
        let _ = t.send(Msg::WebApi(super::ask_web_api(&base)));
    });
}

fn ask_mic(tx: &Sender<Msg>) {
    let t = tx.clone();
    std::thread::spawn(move || {
        let _ = t.send(Msg::Mic(super::mic_name()));
    });
}

/// The catalogue, for the starter's size and its install: the same store
/// `xfc voices` loads (a day's cache, else one fetch), on a worker.
fn load_store(config: &Config, tx: &Sender<Msg>) {
    let config = config.clone();
    let t = tx.clone();
    std::thread::spawn(move || {
        let _ = t.send(Msg::Store(Box::new(Store::load(&config))));
    });
}

fn event_loop(
    terminal: &mut ratatui::DefaultTerminal,
    app: &mut App,
    tx: &Sender<Msg>,
    rx: &Receiver<Msg>,
) -> std::io::Result<Leave> {
    let mut dirty = true;
    loop {
        while let Ok(m) = rx.try_recv() {
            dirty = true;
            take(app, m);
        }
        if dirty {
            terminal.draw(|f| draw(f, app))?;
            dirty = false;
        }
        if !event::poll(Duration::from_millis(80))? {
            continue;
        }
        let k = match event::read()? {
            Event::Key(k) if k.kind == KeyEventKind::Press => k,
            Event::Resize(_, _) => {
                dirty = true;
                continue;
            }
            _ => continue,
        };
        dirty = true;
        let ctrl = k.modifiers.contains(KeyModifiers::CONTROL);
        if ctrl && k.code == KeyCode::Char('c') {
            return Ok(Leave::Quit);
        }
        match key(app, k.code, ctrl) {
            Act::None => {}
            Act::Leave(l) => return Ok(l),
            Act::Recheck => {
                app.facts.web_api = None;
                app.facts.mic = None;
                app.refresh_offline();
                ask_live(&app.config, tx);
            }
            Act::AskWebApi => {
                app.facts.web_api = None;
                ask_web_api(&app.config, tx);
            }
            Act::OpenVoices => {
                ratatui::restore();
                let opened = crate::voices::open_screen(&app.config);
                *terminal = ratatui::init();
                if let Err(e) = opened {
                    app.notice = Some((format!("the voices screen failed: {e:#}"), false));
                }
                app.refresh_offline();
                load_store(&app.config, tx);
            }
            Act::InstallStarter => start_starter(app, tx),
        }
    }
}

/// One message from a worker.
fn take(app: &mut App, m: Msg) {
    match m {
        Msg::WebApi(r) => app.facts.web_api = Some(r),
        Msg::Mic(name) => app.facts.mic = Some(name),
        Msg::Store(store) => {
            app.facts.starter = Some(store.starter_status());
            if let Some(note) = &store.note {
                if app.notice.is_none() {
                    app.notice = Some((note.clone(), false));
                }
            }
            app.store = Some(*store);
        }
        Msg::Progress(key, p, i, n) => app.downloading = Some((key, p, i, n)),
        Msg::Installed(key, r) => {
            app.notice = Some(match r {
                Ok(()) => (format!("{key} installed, size and checksum verified"), true),
                Err(e) => (
                    format!("{key}: install failed, nothing installed: {e}"),
                    false,
                ),
            });
            app.refresh_offline();
        }
        Msg::StarterDone => {
            app.downloading = None;
            app.refresh_offline();
            if app.facts.starter_missing == 0 {
                app.notice = Some(("the starter set is installed".into(), true));
            }
        }
    }
}

/// One key. The Settings view owns its keys; Esc or q goes home.
fn key(app: &mut App, code: KeyCode, ctrl: bool) -> Act {
    match app.view {
        View::Settings => {
            if app.settings.edit.is_some() {
                app.settings.edit_key(code, ctrl);
            } else if !app.settings.confirming()
                && matches!(code, KeyCode::Esc | KeyCode::Char('q'))
            {
                app.view = View::Home;
                app.summary = super::summary();
            } else {
                app.settings.key(code);
            }
            return Act::None;
        }
        View::Files => {
            if matches!(code, KeyCode::Esc | KeyCode::Char('q') | KeyCode::Enter) {
                app.view = View::Home;
            }
            return Act::None;
        }
        View::Home => {}
    }
    match code {
        KeyCode::Char('q') | KeyCode::Esc => Act::Leave(Leave::Quit),
        KeyCode::Down | KeyCode::Char('j') => {
            app.sel = (app.sel + 1).min(ROWS - 1);
            Act::None
        }
        KeyCode::Up | KeyCode::Char('k') => {
            app.sel = app.sel.saturating_sub(1);
            Act::None
        }
        KeyCode::Char('c') => {
            app.notice = None;
            Act::Recheck
        }
        KeyCode::Char('r') => {
            let red = app.red();
            if red.is_empty() || app.warned_run {
                return Act::Leave(Leave::Run);
            }
            app.warned_run = true;
            app.notice = Some((
                format!(
                    "not ready: {}. r again starts the crew anyway",
                    red.join("; ")
                ),
                false,
            ));
            Act::None
        }
        KeyCode::Enter => enter(app),
        _ => Act::None,
    }
}

/// Enter on the selected row: its fix, or open it.
fn enter(app: &mut App) -> Act {
    match app.sel {
        VOICES_ROW => return Act::OpenVoices,
        SETTINGS_ROW => {
            app.view = View::Settings;
            return Act::None;
        }
        FILES_ROW => {
            app.view = View::Files;
            return Act::None;
        }
        _ => {}
    }
    let Some(check) = app.checks().into_iter().nth(app.sel) else {
        return Act::None;
    };
    match check.fix {
        Fix::None => Act::None,
        Fix::XPlaneHint | Fix::PluginHint | Fix::MicHint => {
            app.notice = Some((super::hint_text(check.fix), true));
            Act::None
        }
        Fix::Recheck => Act::AskWebApi,
        Fix::InstallPlugin => {
            // Its own refusal while X-Plane runs stands, said as it says it.
            let mut out = String::new();
            let ok = crate::plugin_install::install_to(
                &app.facts.xplane,
                crate::plugin_install::embedded(),
                false,
                false,
                &mut out,
            );
            app.notice = Some(match ok {
                Ok(done) => (out.trim().replace('\n', " · "), done),
                Err(e) => (format!("{} · install failed: {e:#}", out.trim()), false),
            });
            app.refresh_offline();
            Act::None
        }
        Fix::InstallStarter => Act::InstallStarter,
    }
}

/// The starter set, one item after another on ONE worker, progress on the
/// screen, through the same verified, atomic install as `xfc voices`.
fn start_starter(app: &mut App, tx: &Sender<Msg>) {
    if app.downloading.is_some() {
        return;
    }
    let Some(store) = app.store.as_ref() else {
        app.notice = Some((
            "fetching the voice catalogue, a moment, then enter again".into(),
            true,
        ));
        return;
    };
    let status = store.starter_status();
    let rows = store.starter_rows();
    if rows.is_empty() {
        app.notice = Some(if status.complete() {
            ("the starter set is complete".into(), true)
        } else {
            (
                format!(
                    "{} not in the catalogue: nothing to download (offline?)",
                    status.unknown.join(", ")
                ),
                false,
            )
        });
        return;
    }
    let n = rows.len();
    app.notice = Some((
        format!(
            "downloading the starter set: {n} item(s), {}",
            catalogue::human_size(status.bytes)
        ),
        true,
    ));
    app.downloading = Some((
        rows[0].entry.key.clone(),
        Progress {
            file: 0,
            files: rows[0].entry.files.len(),
            done: 0,
            total: rows[0].entry.download_size(),
        },
        0,
        n,
    ));
    let dir = store.dir.clone();
    let tx = tx.clone();
    std::thread::spawn(move || {
        for (i, r) in rows.iter().enumerate() {
            let key = r.entry.key.clone();
            let mut last = u8::MAX;
            let result = download::install(&r.entry, &dir, &mut |p| {
                if p.percent() != last {
                    last = p.percent();
                    let _ = tx.send(Msg::Progress(key.clone(), p, i, n));
                }
            });
            let _ = tx.send(Msg::Installed(key, result.map_err(|e| format!("{e:#}"))));
        }
        let _ = tx.send(Msg::StarterDone);
    });
}

const LABEL_W: usize = 11;

fn mark_style(m: Mark) -> Style {
    match m {
        Mark::Ok => Style::default().fg(Color::Green),
        Mark::Bad => Style::default().fg(Color::Red),
        Mark::Warn => Style::default().fg(Color::Yellow),
        Mark::Wait => Style::default().fg(Color::DarkGray),
    }
}

/// One row: the section label, a mark, the text, the hint on the right.
fn row(
    label: &str,
    mark: Option<Mark>,
    text: &str,
    hint: &str,
    selected: bool,
    width: usize,
) -> Line<'static> {
    let mut spans = vec![Span::styled(
        format!("  {label:<LABEL_W$}"),
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD),
    )];
    let mut used = 2 + LABEL_W;
    if let Some(m) = mark {
        spans.push(Span::styled(format!("{} ", m.symbol()), mark_style(m)));
        used += 2;
    }
    let text_style = if selected {
        Style::default().add_modifier(Modifier::REVERSED)
    } else {
        Style::default()
    };
    spans.push(Span::styled(text.to_string(), text_style));
    used += text.chars().count();
    if !hint.is_empty() {
        let pad = width.saturating_sub(used + hint.chars().count() + 1).max(2);
        spans.push(Span::raw(" ".repeat(pad)));
        spans.push(Span::styled(
            hint.to_string(),
            Style::default().fg(Color::DarkGray),
        ));
    }
    Line::from(spans)
}

fn rule(width: u16) -> Line<'static> {
    Line::styled(
        format!(" {}", "─".repeat(width.saturating_sub(2) as usize)),
        Style::default().fg(Color::DarkGray),
    )
}

fn draw(f: &mut Frame, app: &App) {
    let [head, top, body, bottom, notice, hint] = Layout::vertical([
        Constraint::Length(1),
        Constraint::Length(1),
        Constraint::Min(3),
        Constraint::Length(1),
        Constraint::Length(3),
        Constraint::Length(1),
    ])
    .areas(f.area());
    let width = f.area().width;

    let title = " XFlightCrew";
    let build = format!("build {} ", app.summary.version);
    let pad = (width as usize).saturating_sub(title.len() + build.chars().count());
    f.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled(
                title,
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw(" ".repeat(pad)),
            Span::styled(build, Style::default().fg(Color::DarkGray)),
        ])),
        head,
    );
    f.render_widget(Paragraph::new(rule(width)), top);
    f.render_widget(Paragraph::new(rule(width)), bottom);

    match app.view {
        View::Home => draw_home(f, app, body),
        View::Settings => crate::ui::draw_settings(f, &app.settings, body),
        View::Files => {
            let lines: Vec<Line> = crate::paths_report()
                .lines()
                .map(|l| Line::raw(format!("  {l}")))
                .collect();
            f.render_widget(Paragraph::new(lines).wrap(Wrap { trim: false }), body);
        }
    }

    let notice_line = match (&app.downloading, &app.notice) {
        (Some((key, p, i, n)), _) => Line::styled(
            format!(
                "  downloading {key}  {}% of {}  ({} of {n})",
                p.percent(),
                catalogue::human_size(p.total),
                i + 1
            ),
            Style::default().fg(Color::Yellow),
        ),
        (None, Some((text, ok))) => Line::styled(
            format!("  {text}"),
            Style::default().fg(if *ok { Color::Green } else { Color::Yellow }),
        ),
        (None, None) => Line::raw(""),
    };
    f.render_widget(
        Paragraph::new(notice_line).wrap(Wrap { trim: false }),
        notice,
    );

    let hint_line = match app.view {
        View::Settings => app.settings.edit_line().unwrap_or_else(|| {
            Line::styled(
                "  ←/→ change   ↑↓ select   esc back",
                Style::default().fg(Color::DarkGray),
            )
        }),
        View::Files => Line::styled("  esc back", Style::default().fg(Color::DarkGray)),
        View::Home => Line::styled(
            "  r run   ↑↓ move   enter fix/open   c check again   q quit",
            Style::default().fg(Color::DarkGray),
        ),
    };
    f.render_widget(Paragraph::new(hint_line), hint);
}

fn draw_home(f: &mut Frame, app: &App, area: Rect) {
    let width = area.width as usize;
    let mut lines: Vec<Line> = app
        .checks()
        .iter()
        .enumerate()
        .map(|(i, c)| {
            row(
                if i == 0 { "Setup" } else { "" },
                Some(c.mark),
                &c.text,
                c.hint,
                app.sel == i,
                width,
            )
        })
        .collect();
    lines.push(row(
        "Voices",
        None,
        &app.summary.voices,
        "enter: open",
        app.sel == VOICES_ROW,
        width,
    ));
    lines.push(row(
        "Settings",
        None,
        &app.summary.settings,
        "enter: open",
        app.sel == SETTINGS_ROW,
        width,
    ));
    lines.push(row(
        "Files",
        None,
        &app.summary.files,
        "enter: open",
        app.sel == FILES_ROW,
        width,
    ));
    f.render_widget(Paragraph::new(lines), area);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plugin_install::Verdict;
    use ratatui::backend::TestBackend;

    fn summary() -> Summary {
        Summary {
            version: "2026.10.14 · abc1234".into(),
            voices: "35 installed · 2.5 GB".into(),
            settings: "mode both · listen ptt · EFB off".into(),
            files: "data ~/Library/Application Support/XFlightCrew".into(),
        }
    }

    fn render(app: &App) -> String {
        let mut terminal = ratatui::Terminal::new(TestBackend::new(78, 16)).unwrap();
        terminal.draw(|f| draw(f, app)).unwrap();
        let buf = terminal.backend().buffer().clone();
        (0..buf.area.height)
            .map(|y| {
                (0..buf.area.width)
                    .map(|x| buf[(x, y)].symbol().to_string())
                    .collect::<String>()
                    .trim_end()
                    .to_string()
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    fn app(facts: Facts) -> App {
        App::new(Config::default(), facts, summary())
    }

    #[test]
    fn all_green_renders_every_check_ticked() {
        let a = app(super::super::tests::green());
        let screen = render(&a);
        println!("{screen}");
        assert!(
            screen.contains("✓ X-Plane found  /Users/pilot/X-Plane 12"),
            "{screen}"
        );
        assert!(screen.contains("✓ Web API on 8086"), "{screen}");
        assert!(screen.contains("✓ Plugin 2026.10.14"), "{screen}");
        assert!(
            screen.contains("✓ Microphone  MacBook Pro Microphone"),
            "{screen}"
        );
        assert!(screen.contains("✓ Starter voices: complete"), "{screen}");
        assert!(
            screen.contains("Voices     35 installed · 2.5 GB"),
            "{screen}"
        );
        assert!(screen.contains("r run"), "{screen}");
        assert!(!screen.contains('✗'), "{screen}");
    }

    #[test]
    fn a_red_plugin_offers_its_update_and_r_warns_once() {
        let mut a = app(Facts {
            plugin: Verdict::Older,
            plugin_installed: Some("2026.10.13".into()),
            ..super::super::tests::green()
        });
        let screen = render(&a);
        println!("{screen}");
        assert!(
            screen.contains("✗ Plugin older than this build (2026.10.13)"),
            "{screen}"
        );
        assert!(screen.contains("enter: update"), "{screen}");
        // A red check never blocks the run: it warns once.
        assert_eq!(key(&mut a, KeyCode::Char('r'), false), Act::None);
        let warned = &a.notice.as_ref().unwrap().0;
        assert!(warned.contains("Plugin older"), "{warned}");
        assert!(
            warned.contains("r again starts the crew anyway"),
            "{warned}"
        );
        assert_eq!(
            key(&mut a, KeyCode::Char('r'), false),
            Act::Leave(Leave::Run)
        );
    }

    #[test]
    fn a_missing_starter_set_is_offered_with_its_size() {
        let mut a = app(Facts {
            starter_missing: 2,
            starter: Some(catalogue::StarterStatus {
                missing: vec![
                    catalogue::StarterItem {
                        kind: catalogue::Kind::Piper,
                        key: "en_US-amy-medium".into(),
                        spec: String::new(),
                    },
                    catalogue::StarterItem {
                        kind: catalogue::Kind::Whisper,
                        key: "ggml-base.en".into(),
                        spec: String::new(),
                    },
                ],
                bytes: 211_000_000,
                unknown: Vec::new(),
            }),
            ..super::super::tests::green()
        });
        let screen = render(&a);
        println!("{screen}");
        assert!(
            screen.contains("! Starter voices: 2 missing, 211 MB"),
            "{screen}"
        );
        assert!(screen.contains("enter: get them"), "{screen}");
        // Enter on it asks for the install (the catalogue still loading here).
        a.sel = 4;
        assert_eq!(key(&mut a, KeyCode::Enter, false), Act::InstallStarter);
        let (tx, _rx) = channel();
        start_starter(&mut a, &tx);
        assert!(render(&a).contains("fetching the voice catalogue"));
    }

    #[test]
    fn settings_and_files_open_and_close() {
        let mut a = app(super::super::tests::green());
        a.sel = SETTINGS_ROW;
        assert_eq!(key(&mut a, KeyCode::Enter, false), Act::None);
        assert_eq!(a.view, View::Settings);
        let screen = render(&a);
        assert!(screen.contains("app mode"), "{screen}");
        assert_eq!(key(&mut a, KeyCode::Esc, false), Act::None);
        assert_eq!(a.view, View::Home);
        a.sel = FILES_ROW;
        key(&mut a, KeyCode::Enter, false);
        assert!(render(&a).contains("mode:"));
        key(&mut a, KeyCode::Esc, false);
        a.sel = VOICES_ROW;
        assert_eq!(key(&mut a, KeyCode::Enter, false), Act::OpenVoices);
        assert_eq!(
            key(&mut a, KeyCode::Char('q'), false),
            Act::Leave(Leave::Quit)
        );
    }
}
