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
use std::time::Duration;

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};
use ratatui::layout::{Alignment, Constraint, Layout, Rect};
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
    pub fn heard(&self, transcript: &str, confidence: f32) {
        if let Some(s) = &self.status {
            s.heard(transcript);
        }
        self.send(Report::Log {
            kind: LogKind::Heard,
            text: format!("\"{transcript}\" ({confidence:.2})"),
        });
    }
    pub fn status(&self, phase: Phase) {
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

/// The screens. Events (a blackbox tail) is the next one.
#[derive(Clone, Copy, PartialEq)]
enum Tab {
    Health,
    Settings,
    Transcript,
}

impl Tab {
    const ALL: [Tab; 3] = [Tab::Health, Tab::Settings, Tab::Transcript];
    fn label(self) -> &'static str {
        match self {
            Tab::Health => "Health",
            Tab::Settings => "Settings",
            Tab::Transcript => "Transcript",
        }
    }
}

/// Dashboard UI state.
struct App {
    /// Session facts settled at boot.
    boot: health::Boot,
    /// Live dependency/flight state, refreshed by the poll thread.
    health: health::Live,
    log: Vec<(LogKind, String)>,
    phase: Phase,
    mic: f32,
    tab: Tab,
    /// Rows scrolled back from the bottom of the transcript. 0 = following the
    /// tail, which is where it stays unless you deliberately scroll up.
    scrollback: usize,
    /// Selected row on the Settings screen.
    cursor: usize,
    /// What the last settings edit did, shown under the list — an accepted
    /// change or, more usefully, the reason one was refused.
    notice: Option<(String, bool)>,
    /// Typed-command line (you can type a command instead of speaking it).
    input: String,
    /// Text mode: Enter enters it, Esc leaves. In command mode `q` quits; in text
    /// mode every key types (so `q` is a normal letter, e.g. "qnh 2992").
    editing: bool,
    /// A text setting being edited on the Settings screen (callsign, airline
    /// telephony): the buffer, saved on Enter, dropped on Esc.
    setting_edit: Option<String>,
}

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

/// Run the dashboard on the calling (main) thread until the user quits. Drains
/// `rx` for updates and polls the keyboard for `q`/Esc/Ctrl-C.
pub fn run_dashboard(
    boot: health::Boot,
    rx: Receiver<Report>,
    cmd_tx: Sender<String>,
) -> std::io::Result<()> {
    let mut terminal = ratatui::init();
    let mut app = App {
        boot,
        health: health::Live::default(),
        log: Vec::new(),
        phase: Phase::Idle,
        mic: 0.0,
        tab: Tab::Health,
        scrollback: 0,
        cursor: 0,
        notice: None,
        input: String::new(),
        editing: false,
        setting_edit: None,
    };

    // Redraw only when something visibly changed, not on a fixed timer — a full
    // ratatui render every iteration was the dashboard's steady-state CPU cost.
    // `dirty` starts true to paint the first frame.
    let mut dirty = true;
    let mut mic_bucket = i32::MIN; // last drawn mic deadband bucket
    let res = loop {
        // Drain everything queued from the voice/poll threads, marking the frame
        // dirty only when the update actually changes what's on screen.
        while let Ok(r) = rx.try_recv() {
            match r {
                Report::Log { kind, text } => {
                    app.log.push((kind, text));
                    if app.log.len() > MAX_LOG {
                        app.log.drain(0..app.log.len() - MAX_LOG);
                    }
                    dirty = true;
                }
                Report::Status(p) => {
                    if app.phase != p {
                        app.phase = p;
                        dirty = true;
                    }
                }
                Report::Mic(l) => {
                    // Keep the latest value for the label, but only repaint when
                    // the meter crosses a deadband bucket (see MIC_REDRAW_STEP).
                    app.mic = l;
                    let bucket = (l / MIC_REDRAW_STEP).round() as i32;
                    if bucket != mic_bucket {
                        mic_bucket = bucket;
                        dirty = true;
                    }
                }
                Report::Health(h) => {
                    if app.health != *h {
                        app.health = *h;
                        dirty = true;
                    }
                }
            }
        }

        if dirty {
            if let Err(e) = terminal.draw(|f| draw(f, &app)) {
                break Err(e);
            }
            dirty = false;
        }

        // Block up to 50 ms for input — this also paces the drain loop without a
        // busy-wait. A keypress that isn't quit, or a resize, just needs a repaint.
        if event::poll(Duration::from_millis(50))? {
            match event::read()? {
                Event::Key(k) if k.kind == KeyEventKind::Press => {
                    let ctrl = k.modifiers.contains(event::KeyModifiers::CONTROL);
                    if ctrl && k.code == KeyCode::Char('c') {
                        break Ok(()); // Ctrl-C always quits
                    }
                    if let Some(buf) = app.setting_edit.as_mut() {
                        // A text setting's field: keys type, Enter saves it
                        // through the registry, Esc drops the edit.
                        match k.code {
                            KeyCode::Esc => {
                                app.setting_edit = None;
                                app.notice = None;
                            }
                            KeyCode::Enter => save_text_setting(&mut app),
                            KeyCode::Backspace => {
                                buf.pop();
                            }
                            KeyCode::Char(c) if !ctrl => buf.push(c),
                            _ => {}
                        }
                        dirty = true;
                    } else if app.editing {
                        // Text mode: keys type; Enter runs the command (staying in
                        // text mode for the next one); Esc leaves text mode.
                        match k.code {
                            KeyCode::Esc => {
                                app.editing = false;
                                app.input.clear();
                                dirty = true;
                            }
                            KeyCode::Enter => {
                                let cmd = app.input.trim().to_string();
                                app.input.clear();
                                if !cmd.is_empty() {
                                    let _ = cmd_tx.send(cmd); // loop gone = quitting anyway
                                }
                                dirty = true;
                            }
                            KeyCode::Backspace => {
                                app.input.pop();
                                dirty = true;
                            }
                            KeyCode::Char(c) if !ctrl => {
                                app.input.push(c);
                                dirty = true;
                            }
                            _ => {}
                        }
                    } else {
                        // Command mode: q/Esc quit, Enter opens the text line,
                        // Tab and the digits switch screens. Everything else is
                        // the screen's own — Left/Right edits a setting where
                        // it would only change tabs elsewhere, because on the
                        // Settings screen that is the obvious key for it.
                        match k.code {
                            KeyCode::Char('q') | KeyCode::Esc => break Ok(()),
                            KeyCode::Enter => {
                                app.editing = true;
                                dirty = true;
                            }
                            KeyCode::Tab => {
                                app.tab = next_tab(app.tab);
                                dirty = true;
                            }
                            KeyCode::Char(c @ '1'..='3') => {
                                let i = c as usize - '1' as usize;
                                app.tab = Tab::ALL[i];
                                dirty = true;
                            }
                            other => {
                                dirty |= tab_key(&mut app, other);
                            }
                        }
                    }
                }
                Event::Resize(_, _) => dirty = true,
                _ => {}
            }
        }
    };

    ratatui::restore();
    res
}

fn next_tab(current: Tab) -> Tab {
    let i = Tab::ALL.iter().position(|t| *t == current).unwrap_or(0);
    Tab::ALL[(i + 1) % Tab::ALL.len()]
}

/// A key the active screen owns. Returns whether anything changed on screen.
fn tab_key(app: &mut App, key: KeyCode) -> bool {
    match app.tab {
        Tab::Settings => match key {
            KeyCode::Down | KeyCode::Char('j') => {
                app.cursor = (app.cursor + 1).min(settings::ALL.len().saturating_sub(1));
                app.notice = None;
                true
            }
            KeyCode::Up | KeyCode::Char('k') => {
                app.cursor = app.cursor.saturating_sub(1);
                app.notice = None;
                true
            }
            KeyCode::Right | KeyCode::Char(' ') | KeyCode::Char('l') => adjust(app, true),
            KeyCode::Left | KeyCode::Char('h') => adjust(app, false),
            _ => false,
        },
        Tab::Transcript => match key {
            KeyCode::Down | KeyCode::Char('j') => {
                app.scrollback = app.scrollback.saturating_sub(1);
                true
            }
            KeyCode::Up | KeyCode::Char('k') => {
                app.scrollback = (app.scrollback + 1).min(app.log.len().saturating_sub(1));
                true
            }
            KeyCode::End => {
                app.scrollback = 0;
                true
            }
            _ => false,
        },
        Tab::Health => false,
    }
}

/// Step the selected setting and apply it. A refusal (an env-pinned knob, a
/// value the setting won't take) is shown rather than swallowed: silently
/// ignoring a keypress is how a settings screen teaches you to distrust it.
/// A text knob cannot be stepped: the key opens its field instead (a pinned
/// one says so).
fn adjust(app: &mut App, up: bool) -> bool {
    let Some(s) = settings::ALL.get(app.cursor) else {
        return false;
    };
    if matches!(s.kind, settings::Kind::Text { .. }) {
        if s.env_pinned() {
            app.notice = Some((format!("pinned by {}, edit your .env", s.env), false));
        } else {
            let current = s.value().as_text().unwrap_or_default().to_string();
            app.setting_edit = Some(current);
            app.notice = None;
        }
        return true;
    }
    let next = s.kind.step(&s.value(), up);
    app.notice = Some(match settings::set(s.key, &next) {
        Ok(()) => (format!("{} → {}", s.label, s.kind.render(&next)), true),
        Err(e) => (e, false),
    });
    true
}

/// Save the text setting being edited through the registry (validate, apply
/// live, persist), saying what happened. A refusal keeps the field open.
fn save_text_setting(app: &mut App) {
    let (Some(buf), Some(s)) = (app.setting_edit.as_ref(), settings::ALL.get(app.cursor)) else {
        app.setting_edit = None;
        return;
    };
    let result = s
        .kind
        .parse(buf)
        .ok_or_else(|| format!("{} is too long or not one line", s.label))
        .and_then(|v| settings::set(s.key, &v));
    match result {
        Ok(()) => {
            app.notice = Some((format!("{} = {}", s.label, s.rendered()), true));
            app.setting_edit = None;
        }
        Err(e) => app.notice = Some((e, false)),
    }
}

fn draw(f: &mut Frame, app: &App) {
    let [status, tabs, body, hint] = Layout::vertical([
        Constraint::Length(1),
        Constraint::Length(1),
        Constraint::Min(3),
        Constraint::Length(1),
    ])
    .areas(f.area());

    draw_status_bar(f, app, status);
    draw_tab_bar(f, app, tabs);
    match app.tab {
        Tab::Health => draw_health(f, app, body),
        Tab::Settings => draw_settings(f, app, body),
        Tab::Transcript => draw_transcript(f, app, body),
    }

    // --- bottom line: command-mode hint, or the text input when editing ---
    let input_line = if let Some(buf) = &app.setting_edit {
        let label = settings::ALL.get(app.cursor).map_or("", |s| s.label);
        Line::from(vec![
            Span::styled(
                format!(" {label}: "),
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw(buf.as_str()),
            Span::styled("▏", Style::default().fg(Color::Cyan)),
            Span::styled(
                "   Enter save · Esc cancel",
                Style::default().fg(Color::DarkGray),
            ),
        ])
    } else if app.editing {
        Line::from(vec![
            Span::styled(
                " › ",
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw(app.input.as_str()),
            Span::styled("▏", Style::default().fg(Color::Cyan)),
            Span::styled(
                "   Enter run · Esc done",
                Style::default().fg(Color::DarkGray),
            ),
        ])
    } else {
        Line::from(Span::styled(
            " q quit · Tab screen · Enter to type a command · or just speak",
            Style::default().fg(Color::DarkGray),
        ))
    };
    f.render_widget(Paragraph::new(input_line), hint);
}

/// Title and phase on the left, the live mic level and the active aircraft on
/// the right. The mic rides the status bar rather than a panel because it is
/// the one thing you want visible on every screen: a dead mic looks exactly
/// like a copilot ignoring you.
fn draw_status_bar(f: &mut Frame, app: &App, area: Rect) {
    let (dot, label, c) = match app.phase {
        Phase::Listening => ("●", "listening", Color::Green),
        Phase::Thinking => ("◐", "thinking", Color::Yellow),
        Phase::Speaking => ("◑", "speaking", Color::Cyan),
        Phase::Idle => ("○", "idle", Color::DarkGray),
    };
    let right = format!("{}  {}", mic_bar(app.mic), app.health.profile);
    let [left, rightarea] = Layout::horizontal([
        Constraint::Min(20),
        Constraint::Length(right.chars().count() as u16 + 1),
    ])
    .areas(area);
    f.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled(
                format!(" XVoiceLayer {} ", app.boot.version),
                Style::default().add_modifier(Modifier::BOLD),
            ),
            Span::styled(
                format!("({}) ", app.boot.build),
                Style::default().fg(Color::DarkGray),
            ),
            Span::raw(" "),
            Span::styled(format!("{dot} {label}"), Style::default().fg(c)),
        ])),
        left,
    );
    f.render_widget(
        Paragraph::new(Span::styled(
            format!("{right} "),
            Style::default().fg(Color::DarkGray),
        ))
        .alignment(Alignment::Right),
        rightarea,
    );
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

fn draw_tab_bar(f: &mut Frame, app: &App, area: Rect) {
    let mut spans = vec![Span::raw(" ")];
    for (i, t) in Tab::ALL.iter().enumerate() {
        let style = if *t == app.tab {
            Style::default()
                .fg(Color::Black)
                .bg(Color::Cyan)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::DarkGray)
        };
        spans.push(Span::styled(format!(" {} {} ", i + 1, t.label()), style));
        spans.push(Span::raw(" "));
    }
    f.render_widget(Paragraph::new(Line::from(spans)), area);
}

/// The health screen: dependencies on the left, what we are costing the sim on
/// the right, the session's settled facts underneath.
fn draw_health(f: &mut Frame, app: &App, area: Rect) {
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
fn draw_settings(f: &mut Frame, app: &App, area: Rect) {
    let [list, detail] = Layout::vertical([Constraint::Min(3), Constraint::Length(4)]).areas(area);

    // Keep the cursor on screen without a scrollbar: page the window so the
    // selected row is always inside it.
    let rows = list.height.saturating_sub(2).max(1) as usize;
    let start = app
        .cursor
        .saturating_sub(rows.saturating_sub(1))
        .min(settings::ALL.len().saturating_sub(rows));

    let items: Vec<ListItem> = settings::ALL
        .iter()
        .enumerate()
        .skip(start)
        .take(rows)
        .map(|(i, s)| {
            let selected = i == app.cursor;
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
                Span::styled(format!("{:<20}", s.label), label_style),
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
                    format!("pinned by {}", s.env),
                    Style::default().fg(Color::Yellow),
                ));
            }
            ListItem::new(Line::from(spans))
        })
        .collect();
    let title = if settings::ALL.len() > rows {
        format!(" settings · {}/{} ", app.cursor + 1, settings::ALL.len())
    } else {
        " settings ".to_string()
    };
    f.render_widget(
        List::new(items).block(Block::default().borders(Borders::ALL).title(title)),
        list,
    );

    let mut lines = Vec::new();
    if let Some(s) = settings::ALL.get(app.cursor) {
        lines.push(Line::from(Span::styled(
            s.help,
            Style::default().fg(Color::White),
        )));
        // When it takes effect, then where it lives. The first half stops
        // being uniform as soon as the reload-tier knobs (voices, whisper, the
        // station) arrive, which is why it is spelled out per row.
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
                .title(" ←/→ change (text: edit) · ↑/↓ select "),
        ),
        detail,
    );
}

/// The transcript, full width, with the mic meter under it. Scrolled back, the
/// title says so — a screen that has silently stopped following the tail reads
/// exactly like a copilot that has stopped talking.
fn draw_transcript(f: &mut Frame, app: &App, area: Rect) {
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

    fn app_with(log: Vec<(LogKind, String)>) -> App {
        App {
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
                    top: vec![("sim/flightmodel/position/latitude".into(), 300)],
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
            tab: Tab::Health,
            scrollback: 0,
            cursor: 0,
            notice: None,
            input: String::new(),
            editing: false,
            setting_edit: None,
        }
    }

    fn render(app: &App, w: u16, h: u16) -> String {
        let mut terminal = ratatui::Terminal::new(TestBackend::new(w, h)).expect("test terminal");
        terminal.draw(|f| draw(f, app)).expect("draw");
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
        app.tab = Tab::Transcript;
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
            for tab in Tab::ALL {
                app.tab = tab;
                // Also with the cursor at the far end of the settings list, so
                // the window arithmetic is exercised at both ends.
                app.cursor = settings::ALL.len().saturating_sub(1);
                render(&app, w, h);
            }
        }
    }

    /// The settings screen shows the live value and the help for the selected
    /// row, and every declared knob is reachable with the cursor.
    #[test]
    fn the_settings_screen_lists_every_knob_with_its_value() {
        let mut app = app_with(Vec::new());
        app.tab = Tab::Settings;
        let out = render(&app, 100, 24);
        assert!(out.contains("app mode"), "{out}");
        assert!(out.contains(settings::ALL[0].help), "{out}");
        assert!(out.contains("applies immediately"), "{out}");
        // Walking down past the end parks on the last row rather than running off.
        for _ in 0..settings::ALL.len() + 5 {
            tab_key(&mut app, KeyCode::Down);
        }
        assert_eq!(app.cursor, settings::ALL.len() - 1);
        let out = render(&app, 100, 24);
        assert!(
            out.contains(settings::ALL[settings::ALL.len() - 1].label),
            "the last row must be visible once selected: {out}"
        );
    }

    /// A refused edit says why. Silently ignoring the key is how a settings
    /// screen teaches you to distrust it.
    #[test]
    fn a_pinned_setting_reports_its_refusal_on_screen() {
        let s = settings::ALL
            .iter()
            .position(|s| s.key == "atc_chatter")
            .expect("declared");
        let _guard = settings::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let mut app = app_with(Vec::new());
        app.tab = Tab::Settings;
        app.cursor = s;
        // SAFETY: the lock above serialises every env-mutating test.
        unsafe { std::env::set_var("XCOPILOT_ATC_CHATTER", "true") };
        assert!(tab_key(&mut app, KeyCode::Right));
        let out = render(&app, 100, 24);
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
        let i = settings::ALL
            .iter()
            .position(|s| s.key == "callsign")
            .expect("declared");
        let mut app = app_with(Vec::new());
        app.tab = Tab::Settings;
        app.cursor = i;
        // SAFETY: the lock above serialises every env-mutating test.
        unsafe { std::env::remove_var("XCOPILOT_CALLSIGN") };
        settings::update(|c| c.callsign = "WAT2516".into());
        assert!(tab_key(&mut app, KeyCode::Right));
        assert_eq!(app.setting_edit.as_deref(), Some("WAT2516"));
        let out = render(&app, 100, 24);
        assert!(out.contains("callsign: WAT2516"), "{out}");
        assert!(out.contains("Enter save"), "{out}");
        app.setting_edit = None;

        unsafe { std::env::set_var("XCOPILOT_CALLSIGN", "BAW306") };
        assert!(tab_key(&mut app, KeyCode::Right));
        assert!(app.setting_edit.is_none());
        let out = render(&app, 100, 24);
        assert!(out.contains("pinned by XCOPILOT_CALLSIGN"), "{out}");
        // A refused save (pinned) keeps the field and says why.
        app.setting_edit = Some("WAT2516".into());
        save_text_setting(&mut app);
        assert!(app.setting_edit.is_some());
        assert!(app
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
