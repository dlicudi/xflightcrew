//! The Checks tab of the one app (`src/app.rs`): the setup checks with
//! their fixes, in the house style of `src/ui.rs` and
//! `src/voices/screen.rs`. Live facts arrive from worker threads; nothing
//! is polled but the file checks the app asks for while the crew waits.

use std::sync::mpsc::{channel, Receiver, Sender};

use ratatui::crossterm::event::KeyCode;
use ratatui::layout::Rect;
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::Paragraph;
use ratatui::Frame;

use super::{Check, Facts, Fix, Mark, Summary};
use crate::config::Config;
use crate::voices::catalogue;
use crate::voices::download::{self, Progress};
use crate::voices::Store;

/// From the worker threads to the tab.
enum Msg {
    WebApi(Result<(), String>),
    Mic(Option<String>),
    /// key, progress, which item (from 0), of how many.
    Progress(String, Progress, usize, usize),
    Installed(String, Result<(), String>),
    StarterDone,
}

/// The checks, their facts and their fixes.
pub struct Checks {
    config: Config,
    pub facts: Facts,
    pub summary: Summary,
    pub sel: usize,
    /// The voice catalogue, for the starter's size and its install: the
    /// app's one load (`Store::load` cleans stale download parts, so it
    /// is never loaded twice beside a running download).
    store: Option<Store>,
    /// key, progress, which item, of how many.
    pub downloading: Option<(String, Progress, usize, usize)>,
    pub notice: Option<(String, bool)>,
    /// The model files changed (a starter install): the Voices tab
    /// refreshes. Taken by [`Checks::take_models_changed`].
    models_changed: bool,
    tx: Sender<Msg>,
    rx: Receiver<Msg>,
}

impl Checks {
    /// The offline facts now; the two live ones asked on workers.
    pub fn new(config: &Config) -> Self {
        let mut c = Checks::with_facts(config, super::offline(config), super::summary());
        c.ask_live();
        c
    }

    /// The tab over given facts (nothing asked).
    pub fn with_facts(config: &Config, facts: Facts, summary: Summary) -> Self {
        let (tx, rx) = channel();
        Checks {
            config: config.clone(),
            facts,
            summary,
            sel: 0,
            store: None,
            downloading: None,
            notice: None,
            models_changed: false,
            tx,
            rx,
        }
    }

    pub fn checks(&self) -> Vec<Check> {
        super::checks(&self.facts)
    }

    /// The required checks that are red: what the crew waits on.
    pub fn waiting_on(&self) -> Vec<String> {
        if !super::needs_home(&self.facts) {
            return Vec::new();
        }
        self.checks()
            .into_iter()
            .filter(|c| c.required)
            .map(|c| c.text)
            .collect()
    }

    /// The catalogue has come (the app's one load).
    pub fn set_store(&mut self, store: Store) {
        self.facts.starter = Some(store.starter_status());
        if let Some(note) = &store.note {
            if self.notice.is_none() {
                self.notice = Some((note.clone(), false));
            }
        }
        self.store = Some(store);
    }

    /// The configuration the checks judge (the waiting crew's settings
    /// re-read by the app).
    pub fn set_config(&mut self, config: &Config) {
        self.config = config.clone();
    }

    /// Install the plugin this build carries; its own refusal while
    /// X-Plane runs stands, said as it says it. The Checks fix and the
    /// Plugin tab's `i`.
    pub fn install_plugin(&mut self) -> (String, bool) {
        let mut out = String::new();
        let ok = crate::plugin_install::install_to(
            &self.facts.xplane,
            crate::plugin_install::embedded(),
            false,
            false,
            &mut out,
        );
        let said = match ok {
            Ok(done) => (out.trim().replace('\n', " · "), done),
            Err(e) => (format!("{} · install failed: {e:#}", out.trim()), false),
        };
        self.refresh_offline();
        said
    }

    /// Whether a starter install changed the model files since last asked.
    pub fn take_models_changed(&mut self) -> bool {
        std::mem::take(&mut self.models_changed)
    }

    /// The offline facts again (after a fix, an install, a setting),
    /// keeping what was asked live.
    pub fn refresh_offline(&mut self) {
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

    /// The two live facts, each on its own worker: ONE Web API GET and the
    /// microphone's name. Only on open and on `c`, never polled.
    fn ask_live(&mut self) {
        self.ask_web_api();
        let t = self.tx.clone();
        std::thread::spawn(move || {
            let _ = t.send(Msg::Mic(super::mic_name()));
        });
    }

    fn ask_web_api(&mut self) {
        self.facts.web_api = None;
        let base = self.config.xplane_base_url.clone();
        let t = self.tx.clone();
        std::thread::spawn(move || {
            let _ = t.send(Msg::WebApi(super::ask_web_api(&base)));
        });
    }

    /// The workers' news. Returns whether anything changed on screen.
    pub fn poll(&mut self) -> bool {
        let mut dirty = false;
        while let Ok(m) = self.rx.try_recv() {
            dirty = true;
            self.take(m);
        }
        dirty
    }

    fn take(&mut self, m: Msg) {
        match m {
            Msg::WebApi(r) => self.facts.web_api = Some(r),
            Msg::Mic(name) => self.facts.mic = Some(name),
            Msg::Progress(key, p, i, n) => self.downloading = Some((key, p, i, n)),
            Msg::Installed(key, r) => {
                self.notice = Some(match r {
                    Ok(()) => (format!("{key} installed, size and checksum verified"), true),
                    Err(e) => (
                        format!("{key}: install failed, nothing installed: {e}"),
                        false,
                    ),
                });
                self.models_changed = true;
                self.refresh_offline();
            }
            Msg::StarterDone => {
                self.downloading = None;
                self.refresh_offline();
                if self.facts.starter_missing == 0 {
                    self.notice = Some(("the starter set is installed".into(), true));
                }
            }
        }
    }

    /// One key: move, fix, check again. Returns whether anything changed.
    pub fn key(&mut self, code: KeyCode) -> bool {
        let n = self.checks().len();
        match code {
            KeyCode::Down | KeyCode::Char('j') => self.sel = (self.sel + 1).min(n - 1),
            KeyCode::Up | KeyCode::Char('k') => self.sel = self.sel.saturating_sub(1),
            KeyCode::Char('c') => {
                self.notice = None;
                self.facts.mic = None;
                self.refresh_offline();
                self.ask_live();
            }
            KeyCode::Enter => self.enter(),
            _ => return false,
        }
        true
    }

    /// Enter on the selected check: its fix.
    fn enter(&mut self) {
        let Some(check) = self.checks().into_iter().nth(self.sel) else {
            return;
        };
        match check.fix {
            Fix::None => {}
            Fix::XPlaneHint | Fix::PluginHint | Fix::MicHint => {
                self.notice = Some((super::hint_text(check.fix), true));
            }
            Fix::Recheck => self.ask_web_api(),
            Fix::InstallPlugin => self.notice = Some(self.install_plugin()),
            Fix::InstallStarter => self.start_starter(),
        }
    }

    /// The starter set, one item after another on ONE worker, progress on
    /// the screen, through the same verified, atomic install as `xfc voices`.
    pub fn start_starter(&mut self) {
        if self.downloading.is_some() {
            return;
        }
        let Some(store) = self.store.as_ref() else {
            self.notice = Some((
                "fetching the voice catalogue, a moment, then enter again".into(),
                true,
            ));
            return;
        };
        let status = store.starter_status();
        let rows = store.starter_rows();
        if rows.is_empty() {
            self.notice = Some(if status.complete() {
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
        self.notice = Some((
            format!(
                "downloading the starter set: {n} item(s), {}",
                catalogue::human_size(status.bytes)
            ),
            true,
        ));
        self.downloading = Some((
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
        let tx = self.tx.clone();
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

    /// The notice line: a download's progress, else the last notice.
    pub fn notice_line(&self) -> Option<Line<'static>> {
        match (&self.downloading, &self.notice) {
            (Some((key, p, i, n)), _) => Some(Line::styled(
                format!(
                    "  downloading {key}  {}% of {}  ({} of {n})",
                    p.percent(),
                    catalogue::human_size(p.total),
                    i + 1
                ),
                Style::default().fg(Color::Yellow),
            )),
            (None, Some((text, ok))) => Some(Line::styled(
                format!("  {text}"),
                Style::default().fg(if *ok { Color::Green } else { Color::Yellow }),
            )),
            (None, None) => None,
        }
    }

    /// The checks, then the summary lines under them.
    pub fn draw(&self, f: &mut Frame, area: Rect) {
        let width = area.width as usize;
        let mut lines: Vec<Line> = self
            .checks()
            .iter()
            .enumerate()
            .map(|(i, c)| {
                row(
                    if i == 0 { "Setup" } else { "" },
                    Some(c.mark),
                    &c.text,
                    c.hint,
                    self.sel == i,
                    width,
                )
            })
            .collect();
        lines.push(Line::raw(""));
        lines.push(row("Voices", None, &self.summary.voices, "", false, width));
        lines.push(row(
            "Settings",
            None,
            &self.summary.settings,
            "",
            false,
            width,
        ));
        lines.push(row("Files", None, &self.summary.files, "", false, width));
        f.render_widget(Paragraph::new(lines), area);
    }
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

    fn render(c: &Checks) -> String {
        let mut terminal = ratatui::Terminal::new(TestBackend::new(78, 12)).unwrap();
        terminal.draw(|f| c.draw(f, f.area())).unwrap();
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

    fn checks(facts: Facts) -> Checks {
        Checks::with_facts(&Config::default(), facts, summary())
    }

    #[test]
    fn all_green_renders_every_check_ticked_and_waits_on_nothing() {
        let c = checks(super::super::tests::green());
        let screen = render(&c);
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
        assert!(!screen.contains('✗'), "{screen}");
        assert!(c.waiting_on().is_empty());
    }

    #[test]
    fn a_red_plugin_offers_its_update_without_holding_the_crew() {
        let c = checks(Facts {
            plugin: Verdict::Older,
            plugin_installed: Some("2026.10.13".into()),
            ..super::super::tests::green()
        });
        let screen = render(&c);
        assert!(
            screen.contains("✗ Plugin older than this build (2026.10.13)"),
            "{screen}"
        );
        assert!(screen.contains("enter: update"), "{screen}");
        // Red but not required: the crew boots anyway.
        assert!(c.waiting_on().is_empty());
        // A missing one is required: the crew waits on it, by name.
        let c = checks(Facts {
            plugin: Verdict::Missing,
            ..super::super::tests::green()
        });
        assert_eq!(c.waiting_on(), vec!["Plugin not installed".to_string()]);
    }

    #[test]
    fn a_missing_starter_set_is_offered_with_its_size() {
        let mut c = checks(Facts {
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
        let screen = render(&c);
        println!("{screen}");
        assert!(
            screen.contains("! Starter voices: 2 missing, 211 MB"),
            "{screen}"
        );
        assert!(screen.contains("enter: get them"), "{screen}");
        assert_eq!(c.waiting_on().len(), 1);
        // Enter on it asks for the install (the catalogue still loading here).
        c.sel = 4;
        assert!(c.key(KeyCode::Enter));
        let line = c.notice_line().expect("a notice").to_string();
        assert!(line.contains("fetching the voice catalogue"), "{line}");
    }

    #[test]
    fn the_cursor_stays_on_the_checks() {
        let mut c = checks(super::super::tests::green());
        for _ in 0..10 {
            c.key(KeyCode::Down);
        }
        assert_eq!(c.sel, 4);
        for _ in 0..10 {
            c.key(KeyCode::Up);
        }
        assert_eq!(c.sel, 0);
        assert!(!c.key(KeyCode::Char('x')));
    }
}
