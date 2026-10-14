//! The one app: plain `xfc` on a terminal. One TUI, its tabs Flight (the
//! running crew's dashboard, `ui::Dashboard`), Checks (`home::screen`),
//! Voices (`voices::screen`, its Cast view included), one tab per
//! settings section (`settings::Section`, through `ui::SettingsPane`),
//! Plugin and Files.
//!
//! **The crew boots itself.** With every REQUIRED check green it boots at
//! once; red, the app opens on Checks and boots the moment they pass, the
//! Flight tab saying "waiting on: <check>" until then. The boot runs on
//! its own thread (`crate::spawn_tui_crew`), never the screen's.
//!
//! **Settings change inside the running app.** A live knob applies at once;
//! a boot knob (`Tier::Restart`) is marked and waits, and `R` restarts the
//! crew ON THE GROUND ONLY (`restart`: the process starting again in place,
//! since an in-process re-boot is not sound there).
//!
//! `/` searches every setting by label and help, `P` lists the pinned.

use std::sync::mpsc::{channel, Receiver, Sender};
use std::time::{Duration, Instant};

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::layout::{Alignment, Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, List, ListItem, Paragraph, Wrap};
use ratatui::Frame;

use crate::config::Config;
use crate::home::screen::Checks;
use crate::settings::Section;
use crate::ui::{self, Dashboard, Report, Scope, SettingsPane};
use crate::voice::cast;
use crate::voices::Store;

/// How the app ends.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exit {
    Quit,
    /// Start the process again in place (`restart::exec_again`).
    Restart,
}

/// The tab the app opens on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Open {
    /// Flight, or Checks when a required check is red.
    Default,
    /// `xfc home` / `xfc setup`.
    Checks,
}

/// The tabs, in the bar's order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tab {
    Flight,
    Checks,
    Voices,
    Settings(Section),
    Plugin,
    Files,
}

impl Tab {
    pub fn all() -> Vec<Tab> {
        let mut t = vec![Tab::Flight, Tab::Checks, Tab::Voices];
        t.extend(Section::ALL.iter().map(|s| Tab::Settings(*s)));
        t.extend([Tab::Plugin, Tab::Files]);
        t
    }

    pub fn label(self) -> &'static str {
        match self {
            Tab::Flight => "Flight",
            Tab::Checks => "Checks",
            Tab::Voices => "Voices",
            // The Voices tab is the voices themselves; their numbers here.
            Tab::Settings(Section::Voices) => "Voice settings",
            Tab::Settings(Section::AudioPtt) => "Audio and PTT",
            Tab::Settings(Section::Identity) => "Identity",
            Tab::Settings(s) => s.title(),
            Tab::Plugin => "Plugin",
            Tab::Files => "Files",
        }
    }
}

/// Where the crew is.
pub enum Crew {
    /// A required check is red: these, by name.
    Waiting(Vec<String>),
    Booting,
    Up,
    Failed(String),
}

/// What spawns the crew: the real boot in `main`, nothing in a test.
pub type Spawn = fn(Sender<Report>, Sender<String>, Receiver<String>);

/// The restart's state.
enum Restart {
    Idle,
    /// `R` pressed: the next key answers.
    Confirm,
    /// The gate is being read on a worker.
    Checking(Receiver<Result<(), String>>),
}

/// The settings search.
struct Search {
    pane: SettingsPane,
    query: String,
    pinned: bool,
    /// Keys type into the query.
    typing: bool,
}

impl Search {
    fn new(pinned: bool) -> Self {
        Search {
            pane: SettingsPane::scoped(Scope::Search {
                query: String::new(),
                pinned,
            }),
            query: String::new(),
            pinned,
            typing: !pinned,
        }
    }

    fn rescope(&mut self) {
        self.pane.set_scope(Scope::Search {
            query: self.query.clone(),
            pinned: self.pinned,
        });
    }
}

/// The ATC tab's "cast at" box: any ICAO, who speaks there with what.
#[derive(Default)]
struct CastAt {
    /// The ICAO being typed.
    field: Option<String>,
    /// The last field asked and its cast, once the worker answers.
    shown: Option<(String, Vec<cast::Row>)>,
    pending: Option<(String, Receiver<Vec<cast::Row>>)>,
}

impl CastAt {
    /// Ask on a worker: `cast_now` reads apt.dat for the field's name.
    fn ask(&mut self, icao: String) {
        let (tx, rx) = channel();
        let at = icao.clone();
        std::thread::spawn(move || {
            let _ = tx.send(cast::cast_now(Some(&at)));
        });
        self.pending = Some((icao, rx));
    }

    fn poll(&mut self) -> bool {
        let Some((icao, rx)) = self.pending.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Ok(rows) => {
                self.shown = Some((icao.clone(), rows));
                self.pending = None;
                true
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => false,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.pending = None;
                true
            }
        }
    }
}

/// The app's state.
pub struct App {
    config: Config,
    tabs: Vec<Tab>,
    pub tab: usize,
    pub checks: Checks,
    voices: Option<crate::voices::screen::Pane>,
    store_rx: Option<Receiver<Store>>,
    panes: Vec<SettingsPane>,
    search: Option<Search>,
    cast_at: CastAt,
    pub crew: Crew,
    dashboard: Dashboard,
    reports: Option<Receiver<Report>>,
    cmd_tx: Option<Sender<String>>,
    spawn: Spawn,
    restart: Restart,
    /// The app's own last word (restart, plugin), shown above the hints.
    pub notice: Option<(String, bool)>,
    /// What waits for a crew restart, by label (refreshed per key).
    waiting: Vec<String>,
    plugin_text: String,
    last_refresh: Instant,
    /// The restart passed its gate: the event loop ends with it.
    restart_now: bool,
    /// The crew came up once in this process: its threads (the monitor,
    /// the safe hold, any loop) may still run after its command loop
    /// ended, so a restart is gated even then.
    booted: bool,
    /// The real process: the catalogue loads, the waiting crew's settings
    /// are re-read. False in tests, which touch neither.
    real: bool,
}

/// While the crew waits, the file checks are asked again this often.
const WAIT_REFRESH: Duration = Duration::from_secs(2);

impl App {
    /// The app over these checks; the catalogue loads on a worker unless
    /// `load_store` is false (tests).
    pub fn new(config: Config, checks: Checks, spawn: Spawn, open: Open, real: bool) -> Self {
        let tabs = Tab::all();
        let panes = Section::ALL
            .iter()
            .map(|s| SettingsPane::scoped(Scope::Section(*s)))
            .collect();
        let store_rx = real.then(|| {
            let (tx, rx) = channel();
            let c = config.clone();
            std::thread::spawn(move || {
                let _ = tx.send(Store::load(&c));
            });
            rx
        });
        let waiting = checks.waiting_on();
        let mut app = App {
            config,
            tab: 0,
            checks,
            voices: None,
            store_rx,
            panes,
            search: None,
            cast_at: CastAt::default(),
            crew: Crew::Waiting(waiting.clone()),
            dashboard: Dashboard::new(crate::health::Boot::default()),
            reports: None,
            cmd_tx: None,
            spawn,
            restart: Restart::Idle,
            notice: None,
            waiting: Vec::new(),
            plugin_text: String::new(),
            last_refresh: Instant::now(),
            restart_now: false,
            booted: false,
            real,
            tabs,
        };
        app.refresh_plugin();
        if open == Open::Checks || !waiting.is_empty() {
            app.go(Tab::Checks);
        }
        if waiting.is_empty() {
            app.start_crew();
        }
        app
    }

    fn current(&self) -> Tab {
        self.tabs[self.tab]
    }

    pub fn go(&mut self, t: Tab) {
        if let Some(i) = self.tabs.iter().position(|x| *x == t) {
            self.tab = i;
        }
        if t == Tab::Plugin {
            self.refresh_plugin();
        }
    }

    fn refresh_plugin(&mut self) {
        let xplane = std::path::PathBuf::from(crate::config::xplane_root());
        let carried = crate::plugin_install::embedded();
        self.plugin_text = crate::plugin_install::render_status(
            &crate::plugin_install::status(&xplane, carried),
            carried.is_some(),
        );
    }

    /// Boot the crew on its own thread; its reports feed the Flight tab.
    fn start_crew(&mut self) {
        let (tx, rx) = channel();
        let (cmd_tx, cmd_rx) = channel();
        (self.spawn)(tx, cmd_tx.clone(), cmd_rx);
        self.reports = Some(rx);
        self.cmd_tx = Some(cmd_tx);
        self.crew = Crew::Booting;
    }

    fn crew_started(&self) -> bool {
        matches!(self.crew, Crew::Booting | Crew::Up | Crew::Failed(_))
    }

    /// While the crew waits: the settings and boot knobs as a boot would
    /// read them (nothing runs that holds them), then the checks again;
    /// all green boots it.
    fn recheck_waiting(&mut self) {
        if self.crew_started() {
            return;
        }
        if self.real {
            if let Ok(fresh) = Config::from_loaded_env() {
                let live = (*crate::settings::install(fresh)).clone();
                self.checks.set_config(&live);
                self.config = live;
            }
            self.checks.refresh_offline();
        }
        let waiting = self.checks.waiting_on();
        if waiting.is_empty() {
            self.start_crew();
            self.notice = Some((
                "every required check passes: starting the crew".into(),
                true,
            ));
        } else {
            self.crew = Crew::Waiting(waiting);
        }
    }

    /// Everything the workers and the crew sent. Returns whether anything
    /// changed on screen.
    pub fn poll(&mut self) -> bool {
        let mut dirty = self.checks.poll();
        if self.checks.take_models_changed() {
            if let Some(v) = self.voices.as_mut() {
                v.refresh();
            }
            dirty = true;
        }
        if let Some(rx) = self.store_rx.as_ref() {
            if let Ok(store) = rx.try_recv() {
                self.checks.set_store(store.clone());
                self.voices = Some(crate::voices::screen::Pane::new(store));
                self.store_rx = None;
                dirty = true;
            }
        }
        if let Some(v) = self.voices.as_mut() {
            dirty |= v.poll();
        }
        for p in &mut self.panes {
            dirty |= p.poll();
        }
        if let Some(s) = self.search.as_mut() {
            dirty |= s.pane.poll();
        }
        dirty |= self.cast_at.poll();
        if let Some(rx) = self.reports.as_ref() {
            let mut failed = None;
            while let Ok(r) = rx.try_recv() {
                match r {
                    Report::BootFailed(e) => failed = Some(e),
                    Report::Booted(b) => {
                        self.crew = Crew::Up;
                        self.booted = true;
                        dirty |= self.dashboard.take(Report::Booted(b));
                    }
                    other => dirty |= self.dashboard.take(other),
                }
            }
            if let Some(e) = failed {
                self.crew = Crew::Failed(e);
                dirty = true;
            }
        }
        if let Restart::Checking(rx) = &self.restart {
            if let Ok(r) = rx.try_recv() {
                self.restart = Restart::Idle;
                if let Err(e) = r {
                    self.notice = Some((e, false));
                } else {
                    self.notice = Some(("restarting the crew".into(), true));
                    self.restart_now = true;
                }
                dirty = true;
            }
        }
        if !self.crew_started() && self.last_refresh.elapsed() >= WAIT_REFRESH {
            self.last_refresh = Instant::now();
            self.recheck_waiting();
            dirty = true;
        }
        dirty
    }

    /// `R`: the crew restarts, gated ([`crate::restart::verdict`]). Not
    /// yet started, there is nothing to restart; a failed boot has
    /// nothing flying and restarts at once.
    fn ask_restart(&mut self) {
        match self.crew {
            Crew::Waiting(_) | Crew::Booting => {
                self.notice = Some((
                    "the crew has not started: it starts with every setting as saved".into(),
                    true,
                ))
            }
            // Never came up: nothing of ours runs, it starts again at once.
            Crew::Failed(_) if !self.booted => self.restart_now = true,
            Crew::Up | Crew::Failed(_) => {
                self.restart = Restart::Confirm;
                self.notice = None;
            }
        }
    }

    /// The restart confirmed: the gate read on a worker.
    fn check_restart(&mut self) {
        if let Some(what) = crate::restart::held_by_crew() {
            self.notice = Some((
                crate::restart::verdict(Some(&what), &crate::restart::SimAnswer::NotRunning)
                    .unwrap_err(),
                false,
            ));
            return;
        }
        let (tx, rx) = channel();
        let base = self.config.xplane_base_url.clone();
        std::thread::spawn(move || {
            let sim = crate::restart::read_sim(&base);
            // Read again after the sim: a loop armed meanwhile refuses.
            let held = crate::restart::held_by_crew();
            let _ = tx.send(crate::restart::verdict(held.as_deref(), &sim));
        });
        self.restart = Restart::Checking(rx);
        self.notice = Some(("asking the sim where we are".into(), true));
    }

    /// Is a text field or a question open somewhere on the current tab?
    fn capturing(&self) -> bool {
        if self.search.as_ref().is_some_and(|s| s.typing) {
            return true;
        }
        if let Some(p) = self.pane() {
            if p.edit.is_some() || p.confirming() {
                return true;
            }
        }
        match self.current() {
            Tab::Flight => self.dashboard.editing(),
            Tab::Voices => self.voices.as_ref().is_some_and(|v| v.capturing()),
            Tab::Settings(Section::Atc) => self.cast_at.field.is_some(),
            _ => false,
        }
    }

    /// The settings pane in front: the search's, else the tab's.
    fn pane(&self) -> Option<&SettingsPane> {
        if let Some(s) = &self.search {
            return Some(&s.pane);
        }
        match self.current() {
            Tab::Settings(sec) => Section::ALL
                .iter()
                .position(|s| *s == sec)
                .map(|i| &self.panes[i]),
            _ => None,
        }
    }

    fn pane_mut(&mut self) -> Option<&mut SettingsPane> {
        let section = match self.current() {
            Tab::Settings(sec) => Section::ALL.iter().position(|s| *s == sec),
            _ => None,
        };
        if let Some(s) = self.search.as_mut() {
            return Some(&mut s.pane);
        }
        section.map(|i| &mut self.panes[i])
    }

    /// One key. Returns how the app ends, when it does.
    pub fn key(&mut self, code: KeyCode, mods: KeyModifiers) -> Option<Exit> {
        let ctrl = mods.contains(KeyModifiers::CONTROL);
        if ctrl && code == KeyCode::Char('c') {
            return Some(Exit::Quit);
        }
        let out = self.key_inner(code, ctrl);
        // A settings key may have changed what waits for a restart, or
        // (while the crew waits) what it waits on.
        if self.crew_started() {
            if self.real {
                self.waiting = crate::restart::waiting();
            }
        } else if self.pane().is_some() || self.current() == Tab::Checks {
            self.recheck_waiting();
        }
        if std::mem::take(&mut self.restart_now) {
            return Some(Exit::Restart);
        }
        out
    }

    fn key_inner(&mut self, code: KeyCode, ctrl: bool) -> Option<Exit> {
        if matches!(self.restart, Restart::Confirm) {
            self.restart = Restart::Idle;
            if code == KeyCode::Char('y') {
                self.check_restart();
            } else {
                self.notice = Some(("the crew keeps running".into(), true));
            }
            return None;
        }
        if self.capturing() {
            self.captured_key(code, ctrl);
            return None;
        }
        match code {
            KeyCode::Tab => {
                self.search = None;
                let t = self.tabs[(self.tab + 1) % self.tabs.len()];
                self.go(t);
                return None;
            }
            KeyCode::BackTab => {
                self.search = None;
                let t = self.tabs[(self.tab + self.tabs.len() - 1) % self.tabs.len()];
                self.go(t);
                return None;
            }
            KeyCode::Char('q') => return Some(Exit::Quit),
            KeyCode::Char('R') => {
                self.ask_restart();
                return None;
            }
            KeyCode::Char('/') if self.current() != Tab::Voices => {
                self.search = Some(Search::new(false));
                return None;
            }
            KeyCode::Char('P') => {
                self.search = Some(Search::new(true));
                return None;
            }
            _ => {}
        }
        if let Some(s) = self.search.as_mut() {
            match code {
                KeyCode::Esc => self.search = None,
                KeyCode::Char('/') => s.typing = true,
                KeyCode::Char('p') => {
                    s.pinned = !s.pinned;
                    s.rescope();
                }
                other => {
                    s.pane.key(other);
                }
            }
            return None;
        }
        match self.current() {
            Tab::Flight => {
                if code == KeyCode::Enter && matches!(self.crew, Crew::Up) {
                    self.dashboard.start_editing();
                } else {
                    self.dashboard.key(code);
                }
            }
            Tab::Checks => {
                self.checks.key(code);
            }
            Tab::Voices => {
                if let Some(v) = self.voices.as_mut() {
                    // Its own quit (Esc at the top) is not the app's.
                    let _ = v.key(code);
                }
            }
            Tab::Settings(sec) => {
                if sec == Section::Atc && code == KeyCode::Char('a') {
                    self.cast_at.field = Some(String::new());
                } else if let Some(p) = self.pane_mut() {
                    p.key(code);
                }
            }
            Tab::Plugin => {
                if code == KeyCode::Char('i') || code == KeyCode::Enter {
                    let (text, ok) = self.checks.install_plugin();
                    self.notice = Some((text, ok));
                    self.refresh_plugin();
                }
            }
            Tab::Files => {}
        }
        None
    }

    /// A key while something on the tab holds the keyboard.
    fn captured_key(&mut self, code: KeyCode, ctrl: bool) {
        if let Some(s) = self.search.as_mut() {
            if s.typing {
                match code {
                    KeyCode::Esc => self.search = None,
                    KeyCode::Enter | KeyCode::Down => s.typing = false,
                    KeyCode::Backspace => {
                        s.query.pop();
                        s.rescope();
                    }
                    KeyCode::Char(c) if !ctrl => {
                        s.query.push(c);
                        s.rescope();
                    }
                    _ => {}
                }
                return;
            }
        }
        if let Some(p) = self.pane_mut() {
            if p.edit.is_some() {
                p.edit_key(code, ctrl);
                return;
            }
            if p.confirming() {
                p.key(code);
                return;
            }
        }
        match self.current() {
            Tab::Flight => self.dashboard.edit_key(code, ctrl, self.cmd_tx.as_ref()),
            Tab::Voices => {
                if let Some(v) = self.voices.as_mut() {
                    let _ = v.key(code);
                }
            }
            Tab::Settings(Section::Atc) => {
                let Some(buf) = self.cast_at.field.as_mut() else {
                    return;
                };
                match code {
                    KeyCode::Esc => self.cast_at.field = None,
                    KeyCode::Backspace => {
                        buf.pop();
                    }
                    KeyCode::Char(c) if !ctrl && c.is_ascii_alphanumeric() && buf.len() < 4 => {
                        buf.push(c.to_ascii_uppercase())
                    }
                    KeyCode::Enter => {
                        let icao = buf.trim().to_string();
                        self.cast_at.field = None;
                        if (3..=4).contains(&icao.len()) {
                            self.cast_at.ask(icao);
                        } else {
                            self.notice =
                                Some(("an ICAO code is 3 or 4 letters and digits".into(), false));
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }

    /// The whole screen.
    pub fn draw(&mut self, f: &mut Frame) {
        let [status, bar, body, notice, hint] = Layout::vertical([
            Constraint::Length(1),
            Constraint::Length(1),
            Constraint::Min(3),
            Constraint::Length(2),
            Constraint::Length(1),
        ])
        .areas(f.area());
        self.draw_status(f, status);
        self.draw_tabs(f, bar);
        if let Some(s) = &self.search {
            ui::draw_settings(f, &s.pane, body);
        } else {
            match self.current() {
                Tab::Flight => self.draw_flight(f, body),
                Tab::Checks => self.checks.draw(f, body),
                Tab::Voices => match self.voices.as_mut() {
                    Some(v) => v.draw(f, body),
                    None => f.render_widget(
                        Paragraph::new("  loading the voice catalogue...")
                            .style(Style::default().fg(Color::DarkGray)),
                        body,
                    ),
                },
                Tab::Settings(Section::Atc) => self.draw_atc(f, body),
                Tab::Settings(_) => {
                    if let Some(p) = self.pane() {
                        ui::draw_settings(f, p, body);
                    }
                }
                Tab::Plugin => {
                    let mut lines: Vec<Line> = self
                        .plugin_text
                        .lines()
                        .map(|l| Line::raw(format!("  {l}")))
                        .collect();
                    lines.push(Line::raw(""));
                    lines.push(Line::styled(
                        "  i installs the plugin this build carries (refused while X-Plane runs)",
                        Style::default().fg(Color::DarkGray),
                    ));
                    f.render_widget(Paragraph::new(lines).wrap(Wrap { trim: false }), body);
                }
                Tab::Files => {
                    let lines: Vec<Line> = crate::paths_report()
                        .lines()
                        .map(|l| Line::raw(format!("  {l}")))
                        .collect();
                    f.render_widget(Paragraph::new(lines).wrap(Wrap { trim: false }), body);
                }
            }
        }
        f.render_widget(
            Paragraph::new(self.notice_lines()).wrap(Wrap { trim: false }),
            notice,
        );
        f.render_widget(Paragraph::new(self.hint_line()), hint);
    }

    fn draw_status(&self, f: &mut Frame, area: Rect) {
        let boot = &self.dashboard.boot;
        let mut left = vec![Span::styled(
            format!(" XFlightCrew {} ", env!("XVL_VERSION")),
            Style::default().add_modifier(Modifier::BOLD),
        )];
        if !boot.build.is_empty() {
            // The build the crew booted from (a rebuilt binary shows here
            // only after a restart).
            left.push(Span::styled(
                format!("({} {}) ", boot.version, boot.build),
                Style::default().fg(Color::DarkGray),
            ));
        }
        let mut right = String::new();
        match &self.crew {
            Crew::Waiting(w) => left.push(Span::styled(
                format!(" waiting on: {}", w.join("; ")),
                Style::default().fg(Color::Yellow),
            )),
            Crew::Booting => left.push(Span::styled(
                " starting the crew",
                Style::default().fg(Color::Yellow),
            )),
            Crew::Failed(_) => left.push(Span::styled(
                " the crew is not running",
                Style::default().fg(Color::Red),
            )),
            Crew::Up => {
                let (phase, r) = self.dashboard.status();
                left.push(Span::raw(" "));
                left.push(phase);
                right = r;
            }
        }
        let [l, r] = Layout::horizontal([
            Constraint::Min(20),
            Constraint::Length(right.chars().count() as u16 + 1),
        ])
        .areas(area);
        f.render_widget(Paragraph::new(Line::from(left)), l);
        f.render_widget(
            Paragraph::new(Span::styled(
                format!("{right} "),
                Style::default().fg(Color::DarkGray),
            ))
            .alignment(Alignment::Right),
            r,
        );
    }

    /// The tab bar: as many as fit, the current one always among them.
    fn draw_tabs(&self, f: &mut Frame, area: Rect) {
        let widths: Vec<usize> = self
            .tabs
            .iter()
            .map(|t| t.label().chars().count() + 3)
            .collect();
        let room = area.width as usize;
        let mut start = 0;
        while start < self.tab && widths[start..=self.tab].iter().sum::<usize>() + 2 > room {
            start += 1;
        }
        let mut spans = vec![Span::raw(if start > 0 { "‹" } else { " " })];
        let mut used = 1;
        for (i, t) in self.tabs.iter().enumerate().skip(start) {
            if used + widths[i] > room {
                spans.push(Span::raw("›"));
                break;
            }
            let style = if i == self.tab && self.search.is_none() {
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::Cyan)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(Color::DarkGray)
            };
            spans.push(Span::styled(format!(" {} ", t.label()), style));
            spans.push(Span::raw(" "));
            used += widths[i];
        }
        f.render_widget(Paragraph::new(Line::from(spans)), area);
    }

    fn draw_flight(&self, f: &mut Frame, area: Rect) {
        let lines: Vec<Line> =
            match &self.crew {
                Crew::Up => return self.dashboard.draw(f, area),
                Crew::Waiting(w) => {
                    let mut l = vec![Line::raw(""), Line::raw("  The crew is waiting on:")];
                    l.extend(w.iter().map(|c| {
                        Line::styled(format!("    ✗ {c}"), Style::default().fg(Color::Red))
                    }));
                    l.push(Line::raw(""));
                    l.push(Line::styled(
                        "  Fix them on the Checks tab: the crew starts the moment they pass.",
                        Style::default().fg(Color::DarkGray),
                    ));
                    l
                }
                Crew::Booting => vec![
                    Line::raw(""),
                    Line::raw("  Starting the crew: the aircraft, the speech model, the voices."),
                ],
                Crew::Failed(e) => vec![
                    Line::raw(""),
                    Line::styled(
                        format!("  The crew is not running: {e}"),
                        Style::default().fg(Color::Red),
                    ),
                    Line::raw(""),
                    Line::styled(
                        "  Fix it (Checks, Voices, settings), then R starts it again.",
                        Style::default().fg(Color::DarkGray),
                    ),
                ],
            };
        f.render_widget(Paragraph::new(lines).wrap(Wrap { trim: false }), area);
    }

    /// The ATC tab: its settings, then the "cast at" box.
    fn draw_atc(&self, f: &mut Frame, area: Rect) {
        let [top, bottom] =
            Layout::vertical([Constraint::Min(8), Constraint::Length(12)]).areas(area);
        if let Some(p) = self.pane() {
            ui::draw_settings(f, p, top);
        }
        let mut items: Vec<ListItem> = Vec::new();
        let title = if let Some(buf) = &self.cast_at.field {
            format!(" cast at: {buf}▏  (Enter shows who speaks there) ")
        } else if let Some((icao, _)) = &self.cast_at.pending {
            format!(" cast at {icao}: reading the airports... ")
        } else if let Some((icao, _)) = &self.cast_at.shown {
            format!(" who speaks at {icao} · a: another field ")
        } else {
            " cast at: press a and type any ICAO ".to_string()
        };
        if let Some((_, rows)) = &self.cast_at.shown {
            items = rows
                .iter()
                .filter(|r| r.setting.is_none() || !matches!(r.role, "copilot" | "cabin"))
                .map(|r| {
                    let name = if r.spec.is_empty() {
                        "(macOS say)".to_string()
                    } else {
                        crate::settings::voice_name(&r.spec)
                    };
                    ListItem::new(Line::from(vec![
                        Span::styled(
                            if r.installed { " ✓ " } else { " ✗ " },
                            Style::default().fg(if r.installed {
                                Color::Green
                            } else {
                                Color::Red
                            }),
                        ),
                        Span::raw(format!("{:<10} {name:<36} ", r.role)),
                        Span::styled(r.source.describe(), Style::default().fg(Color::Cyan)),
                    ]))
                })
                .collect();
        }
        f.render_widget(
            List::new(items).block(Block::default().borders(Borders::ALL).title(title)),
            bottom,
        );
    }

    fn notice_lines(&self) -> Vec<Line<'static>> {
        let mut out = Vec::new();
        if matches!(self.restart, Restart::Confirm) {
            out.push(Line::styled(
                " restart the crew now (on the ground only)? y restarts, any other key keeps it running",
                Style::default().fg(Color::Yellow),
            ));
        } else if let Some((text, ok)) = &self.notice {
            out.push(Line::styled(
                format!(" {text}"),
                Style::default().fg(if *ok { Color::Green } else { Color::Yellow }),
            ));
        } else if self.current() == Tab::Checks {
            if let Some(l) = self.checks.notice_line() {
                out.push(l);
            }
        }
        if matches!(self.crew, Crew::Up) && !self.waiting.is_empty() {
            out.push(Line::styled(
                format!(
                    " ⟳ waits for a crew restart: {}. R restarts it (on the ground)",
                    self.waiting.join(", ")
                ),
                Style::default().fg(Color::Yellow),
            ));
        }
        out
    }

    fn hint_line(&self) -> Line<'static> {
        if let Some(l) = self
            .dashboard
            .input_line()
            .filter(|_| self.current() == Tab::Flight)
        {
            return l;
        }
        if let Some(l) = self.pane().and_then(|p| p.edit_line()) {
            return l;
        }
        let own = if let Some(s) = &self.search {
            if s.typing {
                "type to search · Enter results · Esc close"
            } else {
                "←/→ change · ↑↓ select · / search again · p pinned only · Esc close"
            }
        } else {
            match self.current() {
                Tab::Flight if matches!(self.crew, Crew::Up) => {
                    "Enter type a command · ←/→ health, transcript · or just speak"
                }
                Tab::Flight => "the crew starts when the checks pass",
                Tab::Checks => "↑↓ move · enter fix · c check again",
                Tab::Voices => "←/→ voices, speech, cast",
                Tab::Settings(Section::Atc) => "←/→ change · ↑↓ select · a cast at a field",
                Tab::Settings(_) => "←/→ change · ↑↓ select · m move .env pins",
                Tab::Plugin => "i install",
                Tab::Files => "where every file lives",
            }
        };
        Line::styled(
            format!(" {own} · Tab next · / search · P pinned · R restart crew · q quit"),
            Style::default().fg(Color::DarkGray),
        )
    }
}

/// Run the app until `q` or a restart. The boot facts, the checks and the
/// settings are ready (`home::prepare_settings` ran).
pub fn run(config: Config, open: Open, spawn: Spawn) -> std::io::Result<Exit> {
    let checks = Checks::new(&config);
    let mut app = App::new(config, checks, spawn, open, true);
    let mut terminal = ratatui::init();
    let res = event_loop(&mut terminal, &mut app);
    ratatui::restore();
    if app.checks.downloading.is_some() {
        eprintln!(
            "left with a download unfinished: nothing half-installed, the parts are cleaned on the next run"
        );
    }
    res
}

fn event_loop(terminal: &mut ratatui::DefaultTerminal, app: &mut App) -> std::io::Result<Exit> {
    let mut dirty = true;
    loop {
        dirty |= app.poll();
        if std::mem::take(&mut app.restart_now) {
            return Ok(Exit::Restart);
        }
        if dirty {
            terminal.draw(|f| app.draw(f))?;
            dirty = false;
        }
        if !event::poll(Duration::from_millis(50))? {
            continue;
        }
        match event::read()? {
            Event::Key(k) if k.kind == KeyEventKind::Press => {
                dirty = true;
                if let Some(exit) = app.key(k.code, k.modifiers) {
                    return Ok(exit);
                }
            }
            Event::Resize(_, _) => dirty = true,
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests;
