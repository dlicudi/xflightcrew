//! Terminal dashboard (opt-in via `XCOPILOT_UI=tui`).
//!
//! The voice loop runs on a background thread and emits [`Report`]s over a
//! channel; the dashboard renders them on the main thread. A [`Reporter`]
//! abstracts the sink so the same loop code drives either plain stdout (default)
//! or the TUI — see [`Reporter::send`].
//!
//! **What this screen is for.** The in-sim panel owns act-and-glance while
//! flying: the automation buttons, the status well, the coach, the route. Your
//! eyes are in the cockpit, so anything you need mid-manoeuvre belongs there.
//! The terminal is the other screen, and it owns what the panel structurally
//! cannot do — is every dependency actually up, what is this costing the sim,
//! and what was said. It deliberately does NOT repeat the aircraft's own
//! instruments: an attitude strip and a radio box here were a second cockpit
//! nobody was looking at, and the nine dataref reads a second they cost were
//! charged to X-Plane's frame loop (see the load panel for why that matters).

use std::sync::mpsc::{Receiver, Sender};

use ratatui::crossterm::event::KeyCode;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Gauge, List, ListItem, Paragraph};
use ratatui::Frame;

use crate::health::{self, Probe};
use crate::settings;

/// What a log line represents (drives icon + colour).
#[derive(Clone, Copy)]
pub enum LogKind {
    Heard,
    Tool,
    Readback,
    Confirm,
    Ok,
    Warn,
    Info,
}

/// Current activity, shown in the status bar.
#[derive(Clone, Copy, PartialEq)]
pub enum Phase {
    Idle,
    Listening,
    Thinking,
    Speaking,
}

/// One message from the voice loop (or poll thread) to the dashboard.
pub enum Report {
    Log {
        kind: LogKind,
        text: String,
    },
    Status(Phase),
    Mic(f32),
    /// Boxed: the health snapshot dwarfs every other variant, and this channel
    /// carries a mic report every 30 ms.
    Health(Box<health::Live>),
    /// The crew is up (the app's in-process boot, `crate::spawn_tui_crew`):
    /// the session's settled facts.
    Booted(Box<health::Boot>),
    /// The crew's boot failed, said as the error says it.
    BootFailed(String),
}

/// Where log/status output goes: plain stdout or the TUI channel.
#[derive(Clone)]
enum Sink {
    Plain,
    Tui(Sender<Report>),
}

/// Output sink for the voice loop. Drives the chosen UI and, when present, also
/// mirrors phase/heard/action into the in-sim plugin panel.
#[derive(Clone)]
pub struct Reporter {
    sink: Sink,
    status: Option<crate::plugin_status::StatusSink>,
}

impl Reporter {
    /// Plain-stdout reporter (no TUI).
    pub fn plain() -> Self {
        Reporter {
            sink: Sink::Plain,
            status: None,
        }
    }
    /// TUI reporter feeding the dashboard channel.
    pub fn tui(tx: Sender<Report>) -> Self {
        Reporter {
            sink: Sink::Tui(tx),
            status: None,
        }
    }
    /// Also mirror status into the in-sim plugin panel.
    pub fn with_status(mut self, status: crate::plugin_status::StatusSink) -> Self {
        self.status = Some(status);
        self
    }
    /// The in-sim panel sink, for handing to a `ToolCtx` (e.g. the autopilot
    /// capture watcher's later level-off announcement).
    pub fn status_sink(&self) -> Option<&crate::plugin_status::StatusSink> {
        self.status.as_ref()
    }

    /// Log to the desktop dashboard ONLY — no in-sim panel mirror. For
    /// plumbing lines ("not a command: ...") that read as debug noise on the
    /// cockpit panel but still belong in the desktop transcript.
    pub fn log_quiet(&self, kind: LogKind, text: impl Into<String>) {
        self.send(Report::Log {
            kind,
            text: text.into(),
        });
    }

    pub fn log(&self, kind: LogKind, text: impl Into<String>) {
        let text = text.into();
        // Readback mirrors verbatim to the panel; Heard is mirrored via `heard()`
        // so the panel gets the clean transcript without the desktop log's
        // confidence suffix.
        if let Some(s) = &self.status {
            match kind {
                LogKind::Readback => s.action(&text),
                // Surface refusals/cancellations and confirm prompts on the panel too.
                LogKind::Warn => s.alert(crate::plugin_status::AlertLevel::Caution, &text),
                LogKind::Confirm => s.alert(crate::plugin_status::AlertLevel::Advisory, &text),
                _ => {}
            }
        }
        self.send(Report::Log { kind, text });
    }
    /// Report a heard utterance: the desktop log shows the transcript with its
    /// confidence (useful for tuning), the in-sim panel gets just the transcript.
    pub fn heard(&self, transcript: &str, confidence: f32, radio: bool) {
        if let Some(s) = &self.status {
            s.heard(transcript, radio);
        }
        self.send(Report::Log {
            kind: LogKind::Heard,
            text: format!("\"{transcript}\" ({confidence:.2})"),
        });
    }
    pub fn status(&self, phase: Phase) {
        crate::efb::feed::note_activity(phase_code(phase));
        if let Some(s) = &self.status {
            s.phase(phase_code(phase));
        }
        self.send(Report::Status(phase));
    }
    pub fn mic(&self, level: f32) {
        self.send(Report::Mic(level));
    }

    fn send(&self, r: Report) {
        match &self.sink {
            Sink::Tui(tx) => {
                let _ = tx.send(r);
            }
            Sink::Plain => print_plain(&r),
        }
    }
}

/// Activity code shared with the plugin panel: 0 idle, 1 listening, 2 thinking,
/// 3 speaking.
fn phase_code(p: Phase) -> u8 {
    match p {
        Phase::Idle => 0,
        Phase::Listening => 1,
        Phase::Thinking => 2,
        Phase::Speaking => 3,
    }
}

/// Plain-stdout rendering of a report (TUI-off fallback). Only log lines print;
/// status/mic/health are dashboard-only.
fn print_plain(r: &Report) {
    if let Report::Log { kind, text } = r {
        println!("{} {text}", icon(*kind));
    }
}

fn icon(k: LogKind) -> &'static str {
    match k {
        LogKind::Heard => "👂",
        LogKind::Tool => "  →",
        LogKind::Readback => "🔊",
        LogKind::Confirm => "🔒",
        LogKind::Ok => "✅",
        LogKind::Warn => "🚫",
        LogKind::Info => "  ",
    }
}

fn color(k: LogKind) -> Color {
    match k {
        LogKind::Heard => Color::White,
        LogKind::Tool => Color::Cyan,
        LogKind::Readback => Color::Green,
        LogKind::Confirm => Color::Yellow,
        LogKind::Ok => Color::Green,
        LogKind::Warn => Color::Red,
        LogKind::Info => Color::DarkGray,
    }
}

/// The Flight tab's two views. Events (a blackbox tail) is the next one.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum View {
    Health,
    Transcript,
}

impl View {
    pub const ALL: [View; 2] = [View::Health, View::Transcript];
    pub fn label(self) -> &'static str {
        match self {
            View::Health => "Health",
            View::Transcript => "Transcript",
        }
    }
}

/// The running crew's dashboard: the Flight tab of the one app
/// (`src/app.rs`). Fed by the crew's [`Report`]s, drawn into an area.
pub struct Dashboard {
    /// Session facts settled at boot.
    pub boot: health::Boot,
    /// Live dependency/flight state, refreshed by the poll thread.
    health: health::Live,
    log: Vec<(LogKind, String)>,
    phase: Phase,
    mic: f32,
    /// The last drawn mic deadband bucket (see [`MIC_REDRAW_STEP`]).
    mic_bucket: i32,
    pub view: View,
    /// Rows scrolled back from the bottom of the transcript. 0 = following the
    /// tail, which is where it stays unless you deliberately scroll up.
    scrollback: usize,
    /// Typed-command line (you can type a command instead of speaking it).
    input: String,
    /// Text mode: Enter enters it, Esc leaves. In text mode every key types
    /// (so `q` is a normal letter, e.g. "qnh 2992").
    editing: bool,
}

/// The settings registry as a screen: every edit goes through
/// `settings::set`, so a pinned knob is refused the same way on every tab
/// of the app (`src/app.rs`), whichever [`Scope`] shows it.
#[derive(Default)]
pub struct SettingsPane {
    /// Which settings this pane lists.
    pub scope: Scope,
    /// Selected row, in [`SettingsPane::rows`].
    pub cursor: usize,
    /// What the last settings edit did, shown under the list: an accepted
    /// change or, more usefully, the reason one was refused.
    pub notice: Option<(String, bool)>,
    /// A text setting being edited (callsign, airline telephony): the
    /// buffer, saved on Enter, dropped on Esc.
    pub edit: Option<String>,
    /// The `.env` move was asked for: the list is shown, `y` makes it.
    pub confirm_import: bool,
    /// A save whose check is slow (the home field's apt.dat scan) runs on a
    /// worker; its answer arrives here ([`SettingsPane::poll`]).
    pending: Option<Receiver<SlowSave>>,
}

/// A slow save's answer: the setting, then the line to show or the refusal.
type SlowSave = (&'static settings::Setting, Result<String, String>);

/// Which settings a [`SettingsPane`] lists.
#[derive(Clone, Debug, Default, PartialEq)]
pub enum Scope {
    /// Every setting, grouped by section.
    #[default]
    All,
    /// One section's (a tab of the app).
    Section(settings::Section),
    /// Every setting whose label, help, key or variable holds every word
    /// of `query`; only the env-pinned ones when `pinned`.
    Search { query: String, pinned: bool },
}

impl Scope {
    /// Whether a setting is listed (pure over the setting and its pin).
    pub fn admits(&self, s: &settings::Setting, pinned: bool) -> bool {
        match self {
            Scope::All => true,
            Scope::Section(sec) => s.section == *sec,
            Scope::Search {
                query,
                pinned: only,
            } => {
                if *only && !pinned {
                    return false;
                }
                let hay = format!("{} {} {} {}", s.label, s.help, s.key, s.env).to_lowercase();
                query
                    .to_lowercase()
                    .split_whitespace()
                    .all(|w| hay.contains(w))
            }
        }
    }
}

/// The settings whose check is too slow for the UI thread: the home
/// field's reads the ~400 MB apt.dat (`settings::check_icao`).
const SLOW_SAVES: &[&str] = &["atc_airport"];

const MAX_LOG: usize = 500;

/// Mic level moves the meter only in steps of roughly this much (RMS). The mic
/// reporter fires ~33×/s (one per 30 ms audio frame), and the meter's `{:.3}`
/// label would change on sub-milli jitter, so redrawing on every mic report
/// pinned a full-screen ratatui render at ~20 fps even in a silent cockpit.
/// Quantizing the redraw trigger to this deadband means a quiet room (rms
/// hovering at the noise floor) stays in one bucket and triggers no redraws,
/// while real speech still animates the meter. Tunable: smaller = smoother
/// meter, more idle redraws.
const MIC_REDRAW_STEP: f32 = 0.002;

impl Dashboard {
    pub fn new(boot: health::Boot) -> Self {
        Dashboard {
            boot,
            health: health::Live::default(),
            log: Vec::new(),
            phase: Phase::Idle,
            mic: 0.0,
            mic_bucket: i32::MIN,
            view: View::Health,
            scrollback: 0,
            input: String::new(),
            editing: false,
        }
    }

    /// One report from the crew's threads. Returns whether the screen
    /// changed (redraw only then: a full render every iteration was the
    /// dashboard's steady-state CPU cost).
    pub fn take(&mut self, r: Report) -> bool {
        match r {
            Report::Log { kind, text } => {
                self.log.push((kind, text));
                if self.log.len() > MAX_LOG {
                    self.log.drain(0..self.log.len() - MAX_LOG);
                }
                true
            }
            Report::Status(p) => {
                let changed = self.phase != p;
                self.phase = p;
                changed
            }
            Report::Mic(l) => {
                // Keep the latest value for the label, but only repaint when
                // the meter crosses a deadband bucket (see MIC_REDRAW_STEP).
                self.mic = l;
                let bucket = (l / MIC_REDRAW_STEP).round() as i32;
                let changed = bucket != self.mic_bucket;
                self.mic_bucket = bucket;
                changed
            }
            Report::Health(h) => {
                let changed = self.health != *h;
                self.health = *h;
                changed
            }
            Report::Booted(b) => {
                self.boot = *b;
                true
            }
            Report::BootFailed(_) => false,
        }
    }

    /// The typed-command line is open: every key types.
    pub fn editing(&self) -> bool {
        self.editing
    }

    /// Open the typed-command line.
    pub fn start_editing(&mut self) {
        self.editing = true;
    }

    /// A key while the typed-command line is open: keys type; Enter runs
    /// the command (staying in text mode for the next one), stamped like
    /// the panel's lines; Esc leaves text mode.
    pub fn edit_key(&mut self, key: KeyCode, ctrl: bool, cmd_tx: Option<&Sender<String>>) {
        match key {
            KeyCode::Esc => {
                self.editing = false;
                self.input.clear();
            }
            KeyCode::Enter => {
                let cmd = self.input.trim().to_string();
                self.input.clear();
                if let (false, Some(tx)) = (cmd.is_empty(), cmd_tx) {
                    // Loop gone = quitting anyway.
                    let _ = crate::pollers::queue_command(tx, cmd);
                }
            }
            KeyCode::Backspace => {
                self.input.pop();
            }
            KeyCode::Char(c) if !ctrl => self.input.push(c),
            _ => {}
        }
    }

    /// A key the Flight tab owns when not typing: Left/Right switch the
    /// view, Up/Down/End scroll the transcript. Returns whether anything
    /// changed on screen.
    pub fn key(&mut self, key: KeyCode) -> bool {
        match key {
            KeyCode::Left | KeyCode::Right => {
                self.view = match self.view {
                    View::Health => View::Transcript,
                    View::Transcript => View::Health,
                };
                true
            }
            _ if self.view == View::Transcript => match key {
                KeyCode::Down | KeyCode::Char('j') => {
                    self.scrollback = self.scrollback.saturating_sub(1);
                    true
                }
                KeyCode::Up | KeyCode::Char('k') => {
                    self.scrollback = (self.scrollback + 1).min(self.log.len().saturating_sub(1));
                    true
                }
                KeyCode::End => {
                    self.scrollback = 0;
                    true
                }
                _ => false,
            },
            _ => false,
        }
    }

    /// The Flight tab's body: the view bar, then the view.
    pub fn draw(&self, f: &mut Frame, area: Rect) {
        let [bar, body] = Layout::vertical([Constraint::Length(1), Constraint::Min(3)]).areas(area);
        let mut spans = vec![Span::raw(" ")];
        for v in View::ALL {
            let style = if v == self.view {
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD | Modifier::UNDERLINED)
            } else {
                Style::default().fg(Color::DarkGray)
            };
            spans.push(Span::styled(v.label(), style));
            spans.push(Span::raw("   "));
        }
        f.render_widget(Paragraph::new(Line::from(spans)), bar);
        match self.view {
            View::Health => draw_health(f, self, body),
            View::Transcript => draw_transcript(f, self, body),
        }
    }

    /// The typed-command line as the bottom line, while it is open.
    pub fn input_line(&self) -> Option<Line<'static>> {
        if !self.editing {
            return None;
        }
        Some(Line::from(vec![
            Span::styled(
                " › ",
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw(self.input.clone()),
            Span::styled("▏", Style::default().fg(Color::Cyan)),
            Span::styled(
                "   Enter run · Esc done",
                Style::default().fg(Color::DarkGray),
            ),
        ]))
    }

    /// The phase dot and its word (left), the mic meter and the aircraft
    /// (right): the app's status bar while the crew is up. The mic rides
    /// the status bar because it is the one thing you want visible on
    /// every screen: a dead mic looks exactly like a copilot ignoring you.
    pub fn status(&self) -> (Span<'static>, String) {
        let (dot, label, c) = match self.phase {
            Phase::Listening => ("●", "listening", Color::Green),
            Phase::Thinking => ("◐", "thinking", Color::Yellow),
            Phase::Speaking => ("◑", "speaking", Color::Cyan),
            Phase::Idle => ("○", "idle", Color::DarkGray),
        };
        (
            Span::styled(format!("{dot} {label}"), Style::default().fg(c)),
            format!("{}  {}", mic_bar(self.mic), self.health.profile),
        )
    }
}

impl SettingsPane {
    /// A pane listing these settings.
    pub fn scoped(scope: Scope) -> Self {
        SettingsPane {
            scope,
            ..Default::default()
        }
    }

    /// The settings this pane lists, in the screen's order (grouped by
    /// section, `settings::by_section`).
    pub fn rows(&self) -> Vec<&'static settings::Setting> {
        settings::by_section()
            .into_iter()
            .filter(|s| self.scope.admits(s, s.env_pinned()))
            .collect()
    }

    /// Change what the pane lists; the cursor back to the top.
    pub fn set_scope(&mut self, scope: Scope) {
        if self.scope != scope {
            self.scope = scope;
            self.cursor = 0;
        }
    }

    /// The setting under the cursor.
    pub fn selected(&self) -> Option<&'static settings::Setting> {
        self.rows().get(self.cursor).copied()
    }

    /// A slow save is out (its field stays open, its keys wait).
    pub fn saving(&self) -> bool {
        self.pending.is_some()
    }

    /// A slow save's answer, when it has come. Returns whether anything
    /// changed on screen.
    pub fn poll(&mut self) -> bool {
        let Some(rx) = self.pending.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Ok((s, Ok(shown))) => {
                self.pending = None;
                self.notice = Some((changed_line(s, &shown), true));
                self.edit = None;
                true
            }
            Ok((_, Err(e))) => {
                self.pending = None;
                self.notice = Some((e, false));
                true
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => false,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.pending = None;
                self.notice = Some(("the save stopped before it answered".into(), false));
                true
            }
        }
    }

    /// A question is open (the `.env` move): every key answers it, so `q`
    /// cannot quit from under it.
    pub fn confirming(&self) -> bool {
        self.confirm_import
    }

    /// A navigation or edit key on the list. Returns whether anything changed.
    pub fn key(&mut self, key: KeyCode) -> bool {
        if self.confirm_import {
            self.confirm_import = false;
            self.notice = Some(if key == KeyCode::Char('y') {
                match crate::env_import::apply(&crate::env_import::pending()) {
                    Ok(m) => (m, true),
                    Err(e) => (e, false),
                }
            } else {
                (".env left as it is".into(), true)
            });
            return true;
        }
        match key {
            KeyCode::Down | KeyCode::Char('j') => {
                self.cursor = (self.cursor + 1).min(self.rows().len().saturating_sub(1));
                self.notice = None;
                true
            }
            KeyCode::Up | KeyCode::Char('k') => {
                self.cursor = self.cursor.saturating_sub(1);
                self.notice = None;
                true
            }
            KeyCode::Right | KeyCode::Char(' ') | KeyCode::Char('l') => self.adjust(true),
            KeyCode::Left | KeyCode::Char('h') => self.adjust(false),
            KeyCode::Char('m') => {
                let moves = crate::env_import::pending();
                if crate::env_import::movable(&moves).is_empty() {
                    self.notice = Some(("nothing in .env to move".into(), true));
                } else {
                    self.confirm_import = true;
                    self.notice = None;
                }
                true
            }
            _ => false,
        }
    }

    /// A key while a text field is open: keys type, Enter saves through the
    /// registry, Esc drops the edit.
    pub fn edit_key(&mut self, key: KeyCode, ctrl: bool) {
        if self.saving() {
            // A slow save is out: the field waits for its answer.
            return;
        }
        let Some(buf) = self.edit.as_mut() else {
            return;
        };
        match key {
            KeyCode::Esc => {
                self.edit = None;
                self.notice = None;
            }
            KeyCode::Enter => self.save_text(),
            KeyCode::Backspace => {
                buf.pop();
            }
            KeyCode::Char(c) if !ctrl => buf.push(c),
            _ => {}
        }
    }

    /// Step the selected setting and apply it. A refusal (an env-pinned knob, a
    /// value the setting won't take) is shown rather than swallowed: silently
    /// ignoring a keypress is how a settings screen teaches you to distrust it.
    /// A text, path or voice knob cannot be stepped: the key opens its field
    /// instead (a pinned one says so).
    fn adjust(&mut self, up: bool) -> bool {
        let Some(s) = self.selected() else {
            return false;
        };
        if matches!(
            s.kind,
            settings::Kind::Text { .. } | settings::Kind::Path { .. } | settings::Kind::Voice
        ) {
            if s.env_pinned() {
                self.notice = Some((format!("pinned by {}, edit your .env", s.env), false));
            } else {
                let current = s.value().as_text().unwrap_or_default().to_string();
                self.edit = Some(current);
                self.notice = None;
            }
            return true;
        }
        let next = s.kind.step(&s.value(), up);
        self.notice = Some(match settings::set(s.key, &next) {
            Ok(()) => (changed_line(s, &s.kind.render(&next)), true),
            Err(e) => (e, false),
        });
        true
    }

    /// Save the text setting being edited through the registry (validate, apply
    /// live, persist), saying what happened. A refusal keeps the field open.
    /// A setting whose check is slow ([`SLOW_SAVES`]) is saved on a worker,
    /// the field held open until it answers ([`SettingsPane::poll`]): the
    /// screen never blocks on an apt.dat scan.
    pub fn save_text(&mut self) {
        let (Some(buf), Some(s)) = (self.edit.as_ref(), self.selected()) else {
            self.edit = None;
            return;
        };
        let parsed = s
            .kind
            .parse(buf)
            .ok_or_else(|| format!("{} is too long or not one line", s.label));
        if SLOW_SAVES.contains(&s.key) {
            let v = match parsed {
                Ok(v) => v,
                Err(e) => {
                    self.notice = Some((e, false));
                    return;
                }
            };
            let (tx, rx) = std::sync::mpsc::channel();
            self.pending = Some(rx);
            self.notice = Some((format!("checking {} ...", s.kind.render(&v)), true));
            std::thread::spawn(move || {
                let answer = settings::set(s.key, &v).map(|()| s.rendered());
                let _ = tx.send((s, answer));
            });
            return;
        }
        let result = parsed.and_then(|v| settings::set(s.key, &v));
        match result {
            Ok(()) => {
                self.notice = Some((changed_line(s, &s.rendered()), true));
                self.edit = None;
            }
            Err(e) => self.notice = Some((e, false)),
        }
    }

    /// The open text field as the bottom line, or `None`.
    pub fn edit_line(&self) -> Option<Line<'static>> {
        let buf = self.edit.as_ref()?;
        let label = self.selected().map_or("", |s| s.label);
        let hint = if self.saving() {
            "   checking..."
        } else {
            "   Enter save · Esc cancel"
        };
        Some(Line::from(vec![
            Span::styled(
                format!(" {label}: "),
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw(buf.clone()),
            Span::styled("▏", Style::default().fg(Color::Cyan)),
            Span::styled(hint, Style::default().fg(Color::DarkGray)),
        ]))
    }
}

/// What an accepted change says: the new value, and for a boot knob that
/// it waits for the crew's next start.
fn changed_line(s: &settings::Setting, shown: &str) -> String {
    match s.tier {
        settings::Tier::Live => format!("{} → {shown}", s.label),
        settings::Tier::Restart => {
            format!("{} → {shown} (applies when the crew restarts)", s.label)
        }
    }
}

/// Eight cells of mic level, so speech is visible without reading a number.
fn mic_bar(level: f32) -> String {
    const CELLS: usize = 8;
    let filled = ((level * 12.0).clamp(0.0, 1.0) * CELLS as f32).round() as usize;
    format!(
        "mic {}{}",
        "▇".repeat(filled),
        "·".repeat(CELLS.saturating_sub(filled))
    )
}

/// The health screen: dependencies on the left, what we are costing the sim on
/// the right, the session's settled facts underneath.
fn draw_health(f: &mut Frame, app: &Dashboard, area: Rect) {
    let [top, bottom] = Layout::vertical([Constraint::Length(10), Constraint::Min(3)]).areas(area);
    let [left, right] =
        Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)]).areas(top);

    // --- systems ---
    let h = &app.health;
    let mut systems = vec![
        probe_row("X-Plane", &h.xplane, Some(&app.boot.xplane_url)),
        probe_row("plugin", &h.plugin, None),
        kv("whisper", app.boot.whisper.clone()),
    ];
    systems.push(match &h.engine {
        Some((backend, voice)) => kv("voice", format!("{backend} · {voice}")),
        None => kv("voice", "starting".into()),
    });
    systems.push(kv("mic", app.boot.mic.clone()));
    systems.push(kv("listen", app.boot.listen.clone()));
    systems.push(kv("brain", "deterministic, no model".to_string()));
    systems.push(kv(
        "EFB",
        crate::efb::serving_url().unwrap_or_else(|| "off (XCOPILOT_EFB=true or Settings)".into()),
    ));
    f.render_widget(
        Paragraph::new(systems).block(Block::default().borders(Borders::ALL).title(" systems ")),
        left,
    );

    // --- web api load ---
    // The Web API is served from inside X-Plane's flight loop, so this is the
    // one number on the screen that is a cost TO the sim rather than a fact
    // about it. The slowest response climbing across windows is the shape a
    // freeze arrives in (2026-07-28), which is why it gets a colour.
    let load_lines: Vec<Line> = match &h.load {
        None => vec![Line::from(Span::styled(
            "no requests yet",
            Style::default().fg(Color::DarkGray),
        ))],
        Some(l) => {
            let slow_style = Style::default().fg(if l.slowest_ms >= 1000 {
                Color::Red
            } else if l.slowest_ms >= 200 {
                Color::Yellow
            } else {
                Color::White
            });
            let mut v = vec![
                kv("rate", format!("{:.0}/s", l.rate_per_s())),
                kv(
                    "window",
                    format!("{} req in {:.0}s", l.requests, l.window_s),
                ),
                kv("cached", format!("{}", l.cache_hits)),
                Line::from(vec![
                    Span::styled(
                        format!("{:<9}", "failures"),
                        Style::default().fg(Color::DarkGray),
                    ),
                    Span::styled(
                        l.failures.to_string(),
                        Style::default().fg(if l.failures > 0 {
                            Color::Red
                        } else {
                            Color::White
                        }),
                    ),
                ]),
                Line::from(vec![
                    Span::styled(
                        format!("{:<9}", "slowest"),
                        Style::default().fg(Color::DarkGray),
                    ),
                    Span::styled(format!("{} ms", l.slowest_ms), slow_style),
                ]),
            ];
            if !l.top.is_empty() {
                v.push(Line::from(Span::styled(
                    "busiest datarefs",
                    Style::default().fg(Color::DarkGray),
                )));
                for (name, count) in l.top.iter().take(3) {
                    v.push(Line::from(Span::styled(
                        format!("  {} ×{count}", short_dref(name)),
                        Style::default().fg(Color::White),
                    )));
                }
            }
            v
        }
    };
    f.render_widget(
        Paragraph::new(load_lines).block(
            Block::default()
                .borders(Borders::ALL)
                .title(" web api load "),
        ),
        right,
    );

    // --- flight: who is running, on which radio, with what automation ---
    let [flight, autos] =
        Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)]).areas(bottom);

    let mut rows = vec![kv("mode", h.mode.clone()), kv("cabin", h.cabin.clone())];
    if app.boot.atc_airport.is_empty() {
        rows.push(kv("atc", "off".into()));
    } else {
        rows.push(kv(
            "station",
            if h.station.is_empty() {
                format!("{} (out of range)", app.boot.atc_airport)
            } else {
                h.station.clone()
            },
        ));
        // COM1 against the station's comm plan: the "why is nobody answering"
        // check, which is a diagnosis and so belongs here rather than as a
        // second copy of the cockpit radio.
        rows.push(match (h.com1_khz, h.tuned_service()) {
            (Some(khz), Some(service)) => Line::from(vec![
                Span::styled(
                    format!("{:<9}", "com1"),
                    Style::default().fg(Color::DarkGray),
                ),
                Span::styled(
                    format!("{:.3} {service}", khz as f64 / 1000.0),
                    Style::default().fg(Color::Green),
                ),
            ]),
            (Some(khz), None) => Line::from(vec![
                Span::styled(
                    format!("{:<9}", "com1"),
                    Style::default().fg(Color::DarkGray),
                ),
                Span::styled(
                    format!("{:.3} (no station)", khz as f64 / 1000.0),
                    Style::default().fg(Color::Yellow),
                ),
            ]),
            (None, _) => kv("com1", "--".into()),
        });
    }
    if app.boot.simbrief {
        rows.push(kv("simbrief", "configured".into()));
    }
    for (persona, voice) in &app.boot.voices {
        rows.push(kv(persona, voice.clone()));
    }
    f.render_widget(
        Paragraph::new(rows).block(Block::default().borders(Borders::ALL).title(" flight ")),
        flight,
    );

    // What the copilot's continuous loops are working right now — the same
    // list the panel's automation well shows, kept here because "is anything
    // still flying the aircraft" is the first question after a handback.
    let mg: Vec<ListItem> = if h.managers.is_empty() {
        vec![ListItem::new(Span::styled(
            "hands off",
            Style::default().fg(Color::DarkGray),
        ))]
    } else {
        h.managers
            .iter()
            .map(|m| ListItem::new(Span::styled(m.clone(), Style::default().fg(Color::Green))))
            .collect()
    };
    f.render_widget(
        List::new(mg).block(Block::default().borders(Borders::ALL).title(" automation ")),
        autos,
    );
}

/// The settings screen: every knob the registry declares, its live value, and
/// whether an environment variable has taken it out of your hands.
///
/// The pin is the important column. Precedence is env > saved pref > default,
/// so a knob with its variable set cannot be changed from here — and a screen
/// that let you change it anyway, only for the next boot to put it back, is
/// the documented "always end up back with ryan" failure wearing a new face.
/// The settings list and the selected row's help: the dashboard's Settings
/// tab and the home screen's.
pub fn draw_settings(f: &mut Frame, app: &SettingsPane, area: Rect) {
    let moves = crate::env_import::pending();
    let movable = crate::env_import::movable(&moves).len();
    let banner = u16::from(movable > 0 || app.confirm_import);
    let [top, list, detail] = Layout::vertical([
        Constraint::Length(banner),
        Constraint::Min(3),
        Constraint::Length(4),
    ])
    .areas(area);

    // The `.env` pins, and the one action that moves them: the list first
    // when asked, so nothing is rewritten unseen.
    if app.confirm_import {
        let names: Vec<String> = moves
            .iter()
            .map(|m| match &m.stored {
                Ok(_) => m.env.to_string(),
                Err(_) => format!("({} stays)", m.env),
            })
            .collect();
        f.render_widget(
            Paragraph::new(format!(
                " move {} into xfc settings (.env backed up, lines commented)? y moves, any other key keeps",
                names.join(", ")
            ))
            .style(Style::default().fg(Color::Yellow)),
            top,
        );
    } else if movable > 0 {
        f.render_widget(
            Paragraph::new(format!(
                " {movable} setting(s) pinned by .env: m moves them into xfc settings"
            ))
            .style(Style::default().fg(Color::Yellow)),
            top,
        );
    }

    // The rows as drawn: a title line per section, then its settings.
    enum Row {
        Title(&'static str),
        Knob(usize, &'static settings::Setting),
    }
    let ordered = app.rows();
    let titled = !matches!(app.scope, Scope::Section(_));
    let mut rows_all: Vec<Row> = Vec::new();
    let mut last = None;
    for (i, s) in ordered.iter().enumerate() {
        if titled && last != Some(s.section) {
            rows_all.push(Row::Title(s.section.title()));
            last = Some(s.section);
        }
        rows_all.push(Row::Knob(i, s));
    }
    let selected_row = rows_all
        .iter()
        .position(|r| matches!(r, Row::Knob(i, _) if *i == app.cursor))
        .unwrap_or(0);

    // Keep the cursor on screen without a scrollbar: page the window so the
    // selected row is always inside it (its section title too when it fits).
    let rows = list.height.saturating_sub(2).max(1) as usize;
    let start = selected_row
        .saturating_sub(rows.saturating_sub(1))
        .min(rows_all.len().saturating_sub(rows));

    let items: Vec<ListItem> = rows_all
        .iter()
        .skip(start)
        .take(rows)
        .map(|r| match r {
            Row::Title(t) => ListItem::new(Line::from(Span::styled(
                format!(" {t}"),
                Style::default()
                    .fg(Color::Gray)
                    .add_modifier(Modifier::BOLD),
            ))),
            Row::Knob(i, s) => {
                let selected = *i == app.cursor;
                let pinned = s.env_pinned();
                let label_style = if pinned {
                    Style::default().fg(Color::DarkGray)
                } else if selected {
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(Color::White)
                };
                let mut spans = vec![
                    Span::styled(
                        if selected { "› " } else { "  " },
                        Style::default().fg(Color::Cyan),
                    ),
                    Span::styled(format!("{:<22}", s.label), label_style),
                    Span::styled(
                        format!("{:<12}", s.rendered()),
                        if pinned {
                            Style::default().fg(Color::DarkGray)
                        } else {
                            Style::default().fg(Color::Green)
                        },
                    ),
                ];
                if pinned {
                    spans.push(Span::styled(
                        format!(" pinned by {}", s.env),
                        Style::default().fg(Color::Yellow),
                    ));
                }
                if s.tier == settings::Tier::Restart {
                    spans.push(Span::styled(" ⟳", Style::default().fg(Color::DarkGray)));
                }
                ListItem::new(Line::from(spans))
            }
        })
        .collect();
    let name = match &app.scope {
        Scope::All => "settings".to_string(),
        Scope::Section(sec) => sec.title().to_string(),
        Scope::Search { query, pinned } => {
            let what = if *pinned { "pinned" } else { "search" };
            format!("{what}: {query}▏ {} found", ordered.len())
        }
    };
    let title = if rows_all.len() > rows {
        format!(" {name} · {}/{} ", app.cursor + 1, ordered.len())
    } else {
        format!(" {name} ")
    };
    let items = if ordered.is_empty() {
        vec![ListItem::new(Line::styled(
            "  nothing matches",
            Style::default().fg(Color::DarkGray),
        ))]
    } else {
        items
    };
    f.render_widget(
        List::new(items).block(Block::default().borders(Borders::ALL).title(title)),
        list,
    );

    let mut lines = Vec::new();
    if let Some(s) = app.selected() {
        lines.push(Line::from(Span::styled(
            s.help,
            Style::default().fg(Color::White),
        )));
        // When it takes effect, then where it lives: spelled out per row,
        // since the boot knobs (⟳) wait for the next start.
        let mut provenance = vec![s.tier.describe().to_string()];
        if !s.env.is_empty() {
            provenance.push(s.env.to_string());
        }
        provenance.push("saved to prefs.json".into());
        lines.push(Line::from(Span::styled(
            provenance.join(" · "),
            Style::default().fg(Color::DarkGray),
        )));
    }
    if let Some((text, ok)) = &app.notice {
        lines.push(Line::from(Span::styled(
            text.clone(),
            Style::default().fg(if *ok { Color::Green } else { Color::Yellow }),
        )));
    }
    f.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .borders(Borders::ALL)
                .title(" ←/→ change (text: edit) · ↑/↓ select · ⟳ applies on restart "),
        ),
        detail,
    );
}

/// The transcript, full width, with the mic meter under it. Scrolled back, the
/// title says so — a screen that has silently stopped following the tail reads
/// exactly like a copilot that has stopped talking.
fn draw_transcript(f: &mut Frame, app: &Dashboard, area: Rect) {
    let [logarea, micarea] =
        Layout::vertical([Constraint::Min(3), Constraint::Length(3)]).areas(area);

    let rows = logarea.height.saturating_sub(2) as usize;
    let max_back = app.log.len().saturating_sub(rows);
    let back = app.scrollback.min(max_back);
    let end = app.log.len() - back;
    let start = end.saturating_sub(rows);
    let items: Vec<ListItem> = app.log[start..end]
        .iter()
        .map(|(k, t)| {
            ListItem::new(Line::from(vec![
                Span::styled(format!("{} ", icon(*k)), Style::default().fg(color(*k))),
                Span::styled(t.clone(), Style::default().fg(color(*k))),
            ]))
        })
        .collect();
    let title = if back > 0 {
        format!(" conversation · {back} back (End to follow) ")
    } else {
        " conversation ".to_string()
    };
    f.render_widget(
        List::new(items).block(Block::default().borders(Borders::ALL).title(title)),
        logarea,
    );

    let level = (app.mic * 12.0).clamp(0.0, 1.0) as f64; // ~0.08 RMS ≈ full
    f.render_widget(
        Gauge::default()
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(format!(" mic · {} ", app.boot.mic)),
            )
            .gauge_style(Style::default().fg(Color::Green))
            .ratio(level)
            .label(format!("{:.3}", app.mic)),
        micarea,
    );
}

/// Dataref names are long and share prefixes; the tail is what identifies them.
fn short_dref(name: &str) -> &str {
    name.rsplit('/').next().unwrap_or(name)
}

/// One dependency: state, detail, and (where there is one) the endpoint we are
/// talking to. The endpoint matters on the day it is wrong — a second sim, a
/// stale port in `.env` — which reads identically to "X-Plane is down".
fn probe_row(label: &str, p: &Probe, endpoint: Option<&str>) -> Line<'static> {
    let (mark, detail, c) = match p {
        Probe::Unknown => ("…", "checking".to_string(), Color::DarkGray),
        Probe::Up(d) => ("●", d.clone(), Color::Green),
        Probe::Down(d) => ("✗", d.clone(), Color::Red),
    };
    let mut spans = vec![
        Span::styled(format!("{label:<9}"), Style::default().fg(Color::DarkGray)),
        Span::styled(format!("{mark} "), Style::default().fg(c)),
        Span::styled(detail, Style::default().fg(c)),
    ];
    if let Some(url) = endpoint {
        spans.push(Span::styled(
            format!("  {}", url.trim_start_matches("http://")),
            Style::default().fg(Color::DarkGray),
        ));
    }
    Line::from(spans)
}

fn kv(k: &str, v: String) -> Line<'static> {
    Line::from(vec![
        Span::styled(format!("{k:<9}"), Style::default().fg(Color::DarkGray)),
        Span::styled(v, Style::default().fg(Color::White)),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::backend::TestBackend;
    use std::time::Duration;

    fn app_with(log: Vec<(LogKind, String)>) -> Dashboard {
        Dashboard {
            boot: health::Boot {
                version: "2026.9.22".into(),
                build: "abc1234, release".into(),
                whisper: "base.en · gpu".into(),
                listen: "ptt dataref · xcopilot/ptt_active".into(),
                mic: "MacBook Pro Microphone".into(),
                xplane_url: "http://127.0.0.1:8086".into(),
                atc_airport: "LXGB".into(),
                simbrief: true,
                voices: vec![("cabin", "en_US-hfc_female-medium".into())],
            },
            health: health::Live {
                xplane: Probe::Up("12 ms".into()),
                plugin: Probe::Down("no xcopilot/* datarefs".into()),
                load: Some(sim::xplane::LoadSnapshot {
                    window_s: 12.0,
                    requests: 900,
                    cache_hits: 20,
                    failures: 1,
                    slowest_ms: 1400,
                    top: vec![(crate::datarefs::LATITUDE.into(), 300)],
                }),
                profile: "Cessna 172SP (cessna_172sp)".into(),
                mode: "both".into(),
                cabin: "auto".into(),
                station: "Gibraltar".into(),
                services: vec![("tower".into(), 131_200)],
                com1_khz: Some(131_200),
                engine: Some(("onnx".into(), "en_GB-alba-medium".into())),
                managers: vec!["auto taxi".into()],
            },
            log,
            phase: Phase::Listening,
            mic: 0.03,
            mic_bucket: i32::MIN,
            view: View::Health,
            scrollback: 0,
            input: String::new(),
            editing: false,
        }
    }

    fn render(app: &Dashboard, w: u16, h: u16) -> String {
        let mut terminal = ratatui::Terminal::new(TestBackend::new(w, h)).expect("test terminal");
        terminal.draw(|f| app.draw(f, f.area())).expect("draw");
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

    /// The health screen has to answer its questions on sight: what is up, what
    /// is down, and what we are costing the sim.
    #[test]
    fn the_health_screen_shows_every_dependency_and_the_api_load() {
        let out = render(&app_with(Vec::new()), 120, 30);
        assert!(out.contains("X-Plane"), "{out}");
        assert!(out.contains("127.0.0.1:8086"), "{out}");
        // A failed probe must read as failed, not as an absent row.
        assert!(out.contains("no xcopilot/* datarefs"), "{out}");
        assert!(out.contains("75/s"), "900 requests in 12 s: {out}");
        assert!(out.contains("1400 ms"), "{out}");
        // The tuned frequency is a diagnosis ("why is nobody answering"), so
        // the service it belongs to has to be named.
        assert!(out.contains("131.200 tower"), "{out}");
        assert!(out.contains("auto taxi"), "{out}");
        // ...and none of the cockpit's own instruments are repeated here.
        assert!(!out.contains("VSI"), "{out}");
    }

    /// Scrolled back past the end of a short log, the slice must stay in
    /// bounds — the transcript is the one panel whose length the pilot drives.
    #[test]
    fn the_transcript_survives_scrolling_past_both_ends() {
        let log: Vec<(LogKind, String)> = (0..5)
            .map(|i| (LogKind::Info, format!("line {i}")))
            .collect();
        let mut app = app_with(log);
        app.view = View::Transcript;
        for back in [0usize, 3, 500] {
            app.scrollback = back;
            let out = render(&app, 80, 20);
            assert!(out.contains("conversation"), "{out}");
        }
        // A tall panel shows everything, however far back you asked to scroll:
        // there is nothing above line 0 to scroll to.
        app.scrollback = 500;
        let out = render(&app, 80, 20);
        assert!(out.contains("line 0") && out.contains("line 4"), "{out}");
    }

    /// A narrow terminal must not panic on the layout arithmetic.
    #[test]
    fn a_cramped_terminal_still_renders() {
        for (w, h) in [(40u16, 10u16), (20, 6), (200, 60)] {
            let mut app = app_with(vec![(LogKind::Heard, "flaps ten".into())]);
            for view in View::ALL {
                app.view = view;
                render(&app, w, h);
            }
            // The settings list too, with the cursor at the far end, so the
            // window arithmetic is exercised at both ends.
            let mut pane = SettingsPane {
                cursor: settings::ALL.len().saturating_sub(1),
                ..Default::default()
            };
            render_settings(&pane, w, h);
            pane.cursor = 0;
            render_settings(&pane, w, h);
        }
    }

    fn render_settings(pane: &SettingsPane, w: u16, h: u16) -> String {
        let mut terminal = ratatui::Terminal::new(TestBackend::new(w, h)).expect("test terminal");
        terminal
            .draw(|f| draw_settings(f, pane, f.area()))
            .expect("draw");
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

    /// The settings screen shows the live value and the help for the selected
    /// row, and every declared knob is reachable with the cursor.
    #[test]
    fn the_settings_screen_lists_every_knob_with_its_value() {
        let mut pane = SettingsPane::default();
        let out = render_settings(&pane, 100, 22);
        assert!(out.contains("app mode"), "{out}");
        assert!(out.contains(settings::by_section()[0].help), "{out}");
        assert!(out.contains("applies immediately"), "{out}");
        // Walking down past the end parks on the last row rather than running off.
        for _ in 0..settings::ALL.len() + 5 {
            pane.key(KeyCode::Down);
        }
        assert_eq!(pane.cursor, settings::ALL.len() - 1);
        let out = render_settings(&pane, 100, 22);
        assert!(
            out.contains(settings::by_section()[settings::ALL.len() - 1].label),
            "the last row must be visible once selected: {out}"
        );
    }

    /// A section's pane lists that section only, without section titles,
    /// and its cursor stops at its own last row.
    #[test]
    fn a_section_pane_lists_its_own_settings_only() {
        let mut pane = SettingsPane::scoped(Scope::Section(settings::Section::Atc));
        let rows = pane.rows();
        assert!(!rows.is_empty());
        assert!(rows.iter().all(|s| s.section == settings::Section::Atc));
        assert_eq!(
            rows.len(),
            settings::ALL
                .iter()
                .filter(|s| s.section == settings::Section::Atc)
                .count()
        );
        for _ in 0..50 {
            pane.key(KeyCode::Down);
        }
        assert_eq!(pane.cursor, rows.len() - 1);
        let out = render_settings(&pane, 100, 20);
        assert!(out.contains(rows[0].label), "{out}");
        assert!(!out.contains("app mode"), "{out}");
    }

    /// Search matches every word against the label, the help, the key and
    /// the variable; the pinned filter keeps only what the environment pins.
    #[test]
    fn search_finds_by_label_help_and_variable_and_filters_pinned() {
        let _guard = settings::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let search = |q: &str, pinned: bool| {
            SettingsPane::scoped(Scope::Search {
                query: q.into(),
                pinned,
            })
            .rows()
            .iter()
            .map(|s| s.key)
            .collect::<Vec<_>>()
        };
        assert!(search("callsign", false).contains(&"callsign"));
        // By the variable, in any case.
        assert!(search("xcopilot_atc_chatter", false).contains(&"atc_chatter"));
        // Every word must hold.
        assert!(search("radio volume", false).contains(&"atc_volume"));
        assert!(search("radio zzzz", false).is_empty());
        // SAFETY: the lock above serialises every env-mutating test.
        unsafe { std::env::set_var("XCOPILOT_ATC_CHATTER", "true") };
        let pinned = search("", true);
        assert!(pinned.contains(&"atc_chatter"), "{pinned:?}");
        assert!(pinned
            .iter()
            .all(|k| settings::ALL.iter().any(|s| s.key == *k && s.env_pinned())));
        unsafe { std::env::remove_var("XCOPILOT_ATC_CHATTER") };
        let out = render_settings(
            &SettingsPane::scoped(Scope::Search {
                query: "zzzz".into(),
                pinned: false,
            }),
            80,
            12,
        );
        assert!(out.contains("nothing matches"), "{out}");
    }

    /// The home field's check scans apt.dat: its save runs on a worker, the
    /// field held open and its keys ignored until the answer comes.
    #[test]
    fn the_home_field_is_saved_off_the_screen_thread() {
        let mut pane = SettingsPane::scoped(Scope::Section(settings::Section::Atc));
        pane.cursor = pane
            .rows()
            .iter()
            .position(|s| s.key == "atc_airport")
            .expect("declared");
        // Refused by its own shape, so nothing is written to prefs.json.
        pane.edit = Some("X1-".into());
        pane.save_text();
        assert!(pane.saving());
        pane.edit_key(KeyCode::Char('z'), false);
        assert_eq!(pane.edit.as_deref(), Some("X1-"), "keys wait for the save");
        let deadline = std::time::Instant::now() + Duration::from_secs(10);
        while !pane.poll() {
            assert!(
                std::time::Instant::now() < deadline,
                "the save never answered"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(!pane.saving());
        assert!(pane.edit.is_some(), "a refusal keeps the field");
        let (text, ok) = pane.notice.clone().unwrap();
        assert!(!ok && text.contains("ICAO"), "{text}");
    }

    /// A refused edit says why. Silently ignoring the key is how a settings
    /// screen teaches you to distrust it.
    #[test]
    fn a_pinned_setting_reports_its_refusal_on_screen() {
        let s = settings::by_section()
            .iter()
            .position(|s| s.key == "atc_chatter")
            .expect("declared");
        let _guard = settings::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let mut pane = SettingsPane {
            cursor: s,
            ..Default::default()
        };
        // SAFETY: the lock above serialises every env-mutating test.
        unsafe { std::env::set_var("XCOPILOT_ATC_CHATTER", "true") };
        assert!(pane.key(KeyCode::Right));
        let out = render_settings(&pane, 100, 22);
        assert!(out.contains("pinned by XCOPILOT_ATC_CHATTER"), "{out}");
        unsafe { std::env::remove_var("XCOPILOT_ATC_CHATTER") };
    }

    /// A text knob is edited in place: the key opens its field with the
    /// live value, Esc drops it untouched; a pinned one says so and opens
    /// nothing.
    #[test]
    fn a_text_setting_opens_a_field_and_a_pinned_one_refuses() {
        let _guard = settings::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let i = settings::by_section()
            .iter()
            .position(|s| s.key == "callsign")
            .expect("declared");
        let mut pane = SettingsPane {
            cursor: i,
            ..Default::default()
        };
        // SAFETY: the lock above serialises every env-mutating test.
        unsafe { std::env::remove_var("XCOPILOT_CALLSIGN") };
        settings::update(|c| c.callsign = "WAT2516".into());
        assert!(pane.key(KeyCode::Right));
        assert_eq!(pane.edit.as_deref(), Some("WAT2516"));
        let line = pane.edit_line().expect("the field").to_string();
        assert!(line.contains("callsign: WAT2516"), "{line}");
        assert!(line.contains("Enter save"), "{line}");
        pane.edit = None;

        unsafe { std::env::set_var("XCOPILOT_CALLSIGN", "BAW306") };
        assert!(pane.key(KeyCode::Right));
        assert!(pane.edit.is_none());
        let out = render_settings(&pane, 100, 22);
        assert!(out.contains("pinned by XCOPILOT_CALLSIGN"), "{out}");
        // A refused save (pinned) keeps the field and says why.
        pane.edit = Some("WAT2516".into());
        pane.save_text();
        assert!(pane.edit.is_some());
        assert!(pane
            .notice
            .as_ref()
            .is_some_and(|(n, ok)| !ok && n.contains("XCOPILOT_CALLSIGN")));
        unsafe { std::env::remove_var("XCOPILOT_CALLSIGN") };
        settings::update(|c| c.callsign = String::new());
    }

    #[test]
    fn com1_belongs_to_the_service_it_is_within_five_khz_of() {
        let mut live = health::Live {
            services: vec![("tower".into(), 131_200), ("ground".into(), 121_800)],
            com1_khz: Some(131_205),
            ..Default::default()
        };
        assert_eq!(live.tuned_service(), Some("tower"));
        live.com1_khz = Some(131_300);
        assert_eq!(live.tuned_service(), None);
        live.com1_khz = None;
        assert_eq!(live.tuned_service(), None);
    }
}
