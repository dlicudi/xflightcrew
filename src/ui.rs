//! Terminal dashboard (opt-in via `XCOPILOT_UI=tui`).
//!
//! The voice loop runs on a background thread and emits [`Report`]s over a
//! channel; the dashboard renders them on the main thread. A [`Reporter`]
//! abstracts the sink so the same loop code drives either plain stdout (default)
//! or the TUI — see [`Reporter::send`].

use std::sync::mpsc::{Receiver, Sender};
use std::time::Duration;

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Gauge, List, ListItem, Paragraph};
use ratatui::Frame;

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

/// Live aircraft state for the right-hand panel.
#[derive(Clone, Default)]
pub struct Aircraft {
    pub heading: f64,
    pub altitude: f64,
    pub ias: f64,
    pub vsi: f64,
    pub flaps_pct: f64,
    pub ap_mode: String,
}

/// One message from the voice loop (or poll thread) to the dashboard.
pub enum Report {
    Log { kind: LogKind, text: String },
    Status(Phase),
    Mic(f32),
    Aircraft(Aircraft),
}

/// Output sink for the voice loop: either plain stdout or the TUI channel.
#[derive(Clone)]
pub enum Reporter {
    Plain,
    Tui(Sender<Report>),
}

impl Reporter {
    pub fn log(&self, kind: LogKind, text: impl Into<String>) {
        self.send(Report::Log { kind, text: text.into() });
    }
    pub fn status(&self, phase: Phase) {
        self.send(Report::Status(phase));
    }
    pub fn mic(&self, level: f32) {
        self.send(Report::Mic(level));
    }

    fn send(&self, r: Report) {
        match self {
            Reporter::Tui(tx) => {
                let _ = tx.send(r);
            }
            Reporter::Plain => print_plain(&r),
        }
    }
}

/// Plain-stdout rendering of a report (TUI-off fallback). Only log lines print;
/// status/mic/aircraft are dashboard-only.
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

/// Dashboard UI state.
struct App {
    title: String,
    log: Vec<(LogKind, String)>,
    phase: Phase,
    mic: f32,
    aircraft: Aircraft,
    commands: Vec<String>,
}

const MAX_LOG: usize = 500;

/// Run the dashboard on the calling (main) thread until the user quits. Drains
/// `rx` for updates and polls the keyboard for `q`/Esc/Ctrl-C.
pub fn run_dashboard(
    title: String,
    commands: Vec<String>,
    rx: Receiver<Report>,
) -> std::io::Result<()> {
    let mut terminal = ratatui::init();
    let mut app = App {
        title,
        log: Vec::new(),
        phase: Phase::Idle,
        mic: 0.0,
        aircraft: Aircraft::default(),
        commands,
    };

    let res = loop {
        // Drain everything queued from the voice/poll threads.
        while let Ok(r) = rx.try_recv() {
            match r {
                Report::Log { kind, text } => {
                    app.log.push((kind, text));
                    if app.log.len() > MAX_LOG {
                        app.log.drain(0..app.log.len() - MAX_LOG);
                    }
                }
                Report::Status(p) => app.phase = p,
                Report::Mic(l) => app.mic = l,
                Report::Aircraft(a) => app.aircraft = a,
            }
        }

        if let Err(e) = terminal.draw(|f| draw(f, &app)) {
            break Err(e);
        }

        // ~20 fps; also services keypresses.
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(k) = event::read()? {
                if k.kind == KeyEventKind::Press {
                    let quit = matches!(k.code, KeyCode::Char('q') | KeyCode::Esc)
                        || (k.code == KeyCode::Char('c')
                            && k.modifiers.contains(event::KeyModifiers::CONTROL));
                    if quit {
                        break Ok(());
                    }
                }
            }
        }
    };

    ratatui::restore();
    res
}

fn draw(f: &mut Frame, app: &App) {
    let [status, body, hint] = Layout::vertical([
        Constraint::Length(1),
        Constraint::Min(3),
        Constraint::Length(1),
    ])
    .areas(f.area());

    // --- status bar ---
    let (dot, label, c) = match app.phase {
        Phase::Listening => ("●", "listening", Color::Green),
        Phase::Thinking => ("◐", "thinking", Color::Yellow),
        Phase::Speaking => ("◑", "speaking", Color::Cyan),
        Phase::Idle => ("○", "idle", Color::DarkGray),
    };
    let status_line = Line::from(vec![
        Span::styled(format!(" {} ", app.title), Style::default().add_modifier(Modifier::BOLD)),
        Span::raw("  "),
        Span::styled(format!("{dot} {label}"), Style::default().fg(c)),
    ]);
    f.render_widget(Paragraph::new(status_line), status);

    // --- body: log (left) | aircraft + commands (right) ---
    let [left, right] = Layout::horizontal([Constraint::Min(30), Constraint::Length(26)]).areas(body);

    // left: log (top) + mic meter (bottom)
    let [logarea, micarea] =
        Layout::vertical([Constraint::Min(3), Constraint::Length(3)]).areas(left);

    let rows = logarea.height.saturating_sub(2) as usize;
    let start = app.log.len().saturating_sub(rows);
    let items: Vec<ListItem> = app.log[start..]
        .iter()
        .map(|(k, t)| {
            ListItem::new(Line::from(vec![
                Span::styled(format!("{} ", icon(*k)), Style::default().fg(color(*k))),
                Span::styled(t.clone(), Style::default().fg(color(*k))),
            ]))
        })
        .collect();
    f.render_widget(
        List::new(items).block(Block::default().borders(Borders::ALL).title(" conversation ")),
        logarea,
    );

    let level = (app.mic * 12.0).clamp(0.0, 1.0) as f64; // ~0.08 RMS ≈ full
    f.render_widget(
        Gauge::default()
            .block(Block::default().borders(Borders::ALL).title(" mic "))
            .gauge_style(Style::default().fg(Color::Green))
            .ratio(level)
            .label(format!("{:.3}", app.mic)),
        micarea,
    );

    // right: aircraft (top) + commands (bottom)
    let [acarea, cmdarea] =
        Layout::vertical([Constraint::Length(9), Constraint::Min(3)]).areas(right);

    let a = &app.aircraft;
    let ac = vec![
        kv("HDG", format!("{:03.0}", a.heading)),
        kv("ALT", format!("{:.0} ft", a.altitude)),
        kv("IAS", format!("{:.0} kt", a.ias)),
        kv("VSI", format!("{:+.0} fpm", a.vsi)),
        kv("FLAP", format!("{:.0}%", a.flaps_pct)),
        kv("AP", if a.ap_mode.is_empty() { "--".into() } else { a.ap_mode.clone() }),
    ];
    f.render_widget(
        Paragraph::new(ac).block(Block::default().borders(Borders::ALL).title(" aircraft ")),
        acarea,
    );

    let cmds: Vec<ListItem> = app
        .commands
        .iter()
        .map(|c| ListItem::new(Span::styled(c.clone(), Style::default().fg(Color::DarkGray))))
        .collect();
    f.render_widget(
        List::new(cmds).block(Block::default().borders(Borders::ALL).title(" say ")),
        cmdarea,
    );

    // --- hint ---
    f.render_widget(
        Paragraph::new(Span::styled(
            " q quit · speak to command",
            Style::default().fg(Color::DarkGray),
        )),
        hint,
    );
}

fn kv(k: &str, v: String) -> Line<'static> {
    Line::from(vec![
        Span::styled(format!("{k:<5}"), Style::default().fg(Color::DarkGray)),
        Span::styled(v, Style::default().fg(Color::White)),
    ])
}
