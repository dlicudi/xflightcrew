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
#[derive(Clone, Default, PartialEq)]
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
    /// Typed-command line (you can type a command instead of speaking it).
    input: String,
    /// Text mode: Enter enters it, Esc leaves. In command mode `q` quits; in text
    /// mode every key types (so `q` is a normal letter, e.g. "qnh 2992").
    editing: bool,
    /// Scroll offset (top row) of the "say" example-command panel — Up/Down in
    /// command mode scroll it, since the list is longer than the panel.
    say_scroll: usize,
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
    title: String,
    commands: Vec<String>,
    rx: Receiver<Report>,
    cmd_tx: Sender<String>,
) -> std::io::Result<()> {
    let mut terminal = ratatui::init();
    let mut app = App {
        title,
        log: Vec::new(),
        phase: Phase::Idle,
        mic: 0.0,
        aircraft: Aircraft::default(),
        commands,
        input: String::new(),
        editing: false,
        say_scroll: 0,
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
                Report::Aircraft(a) => {
                    if app.aircraft != a {
                        app.aircraft = a;
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
                    if app.editing {
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
                        // Command mode: q/Esc quit; Enter opens the text line;
                        // Up/Down scroll the "say" example panel.
                        match k.code {
                            KeyCode::Char('q') | KeyCode::Esc => break Ok(()),
                            KeyCode::Enter => {
                                app.editing = true;
                                dirty = true;
                            }
                            KeyCode::Down | KeyCode::Char('j') => {
                                app.say_scroll =
                                    (app.say_scroll + 1).min(app.commands.len().saturating_sub(1));
                                dirty = true;
                            }
                            KeyCode::Up | KeyCode::Char('k') => {
                                app.say_scroll = app.say_scroll.saturating_sub(1);
                                dirty = true;
                            }
                            _ => {}
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

    // "say" example panel — scrollable (Up/Down in command mode) since the hint
    // list is longer than the box. Slice from the scroll offset, clamped so the
    // last page can't scroll into emptiness, and flag more-above/below in the title.
    let rows = cmdarea.height.saturating_sub(2).max(1) as usize;
    let len = app.commands.len();
    let max_start = len.saturating_sub(rows);
    let start = app.say_scroll.min(max_start);
    let cmds: Vec<ListItem> = app.commands[start..]
        .iter()
        .take(rows)
        .map(|c| ListItem::new(Span::styled(c.clone(), Style::default().fg(Color::DarkGray))))
        .collect();
    let title = if len > rows {
        format!(" say {}{} ", if start > 0 { "↑" } else { " " }, if start < max_start { "↓" } else { " " })
    } else {
        " say ".to_string()
    };
    f.render_widget(
        List::new(cmds).block(Block::default().borders(Borders::ALL).title(title)),
        cmdarea,
    );

    // --- bottom line: command-mode hint, or the text input when editing ---
    let input_line = if app.editing {
        Line::from(vec![
            Span::styled(" › ", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
            Span::raw(app.input.as_str()),
            Span::styled("▏", Style::default().fg(Color::Cyan)),
            Span::styled("   Enter run · Esc done", Style::default().fg(Color::DarkGray)),
        ])
    } else {
        Line::from(Span::styled(
            " q quit · Enter to type a command · or just speak",
            Style::default().fg(Color::DarkGray),
        ))
    };
    f.render_widget(Paragraph::new(input_line), hint);
}

fn kv(k: &str, v: String) -> Line<'static> {
    Line::from(vec![
        Span::styled(format!("{k:<5}"), Style::default().fg(Color::DarkGray)),
        Span::styled(v, Style::default().fg(Color::White)),
    ])
}
