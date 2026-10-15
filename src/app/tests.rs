//! The one app under `TestBackend`: the crew boots itself, waits on a red
//! check by name, every tab renders at any size, search, the restart's
//! question, the typed line. Nothing here boots a crew or writes a file.

use super::*;
use crate::home::Facts;
use crate::plugin_install::Verdict;
use ratatui::backend::TestBackend;
use ratatui::crossterm::event::{MouseButton, MouseEventKind};
use std::path::PathBuf;

fn green() -> Facts {
    Facts {
        xplane: PathBuf::from("/Users/pilot/X-Plane 12"),
        xplane_found: true,
        plugin: Verdict::UpToDate,
        plugin_installed: Some("2026.10.14".into()),
        plugin_carried: Some("2026.10.14".into()),
        starter_missing: 0,
        starter: None,
        web_api: Some(Ok(())),
        api_port: "8086".into(),
        mic: Some(Some("MacBook Pro Microphone".into())),
    }
}

fn summary() -> crate::home::Summary {
    crate::home::Summary {
        version: "2026.10.14 · abc1234".into(),
        voices: "35 installed · 2.5 GB".into(),
        settings: "mode both · listen ptt · EFB off".into(),
        files: "data ~/Library/Application Support/XFlightCrew".into(),
    }
}

/// A crew that is up the moment it is spawned.
fn boots(tx: Sender<Report>, _cmd: Sender<String>, _rx: Receiver<String>) {
    let _ = tx.send(Report::Booted(Box::new(crate::health::Boot {
        version: "2026.10.14".into(),
        build: "abc1234, debug".into(),
        ..Default::default()
    })));
}

/// A crew whose boot fails.
fn fails(tx: Sender<Report>, _cmd: Sender<String>, _rx: Receiver<String>) {
    let _ = tx.send(Report::BootFailed("no whisper model".into()));
}

/// A crew that never answers (still booting).
fn silent(_tx: Sender<Report>, _cmd: Sender<String>, _rx: Receiver<String>) {}

fn app(facts: Facts, spawn: Spawn, open: Open) -> App {
    let checks = Checks::with_facts(&Config::default(), facts, summary());
    App::new(Config::default(), checks, spawn, open, false)
}

fn render(app: &mut App, w: u16, h: u16) -> String {
    let mut terminal = ratatui::Terminal::new(TestBackend::new(w, h)).unwrap();
    terminal.draw(|f| app.draw(f)).unwrap();
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

fn press(app: &mut App, code: KeyCode) -> Option<Exit> {
    app.key(code, KeyModifiers::NONE)
}

/// The sidebar: each page, its children under it (every settings section
/// under Settings, the voices' views under Voices).
#[test]
fn every_section_is_a_child_of_settings() {
    let tabs = Tab::all();
    assert_eq!(&tabs[..3], &[Tab::Flight, Tab::Transcript, Tab::Checks]);
    let settings = tabs.iter().position(|t| *t == Tab::Settings(None)).unwrap();
    for (i, s) in Section::ALL.iter().enumerate() {
        assert_eq!(tabs[settings + 1 + i], Tab::Settings(Some(*s)));
        assert!(Tab::Settings(Some(*s)).child());
    }
    assert!(Tab::Voices(2).child() && !Tab::Voices(0).child());
    assert_eq!(&tabs[tabs.len() - 2..], &[Tab::Plugin, Tab::Files]);
}

/// All green: the crew boots at once, no key pressed, and the Flight tab
/// is its dashboard.
#[test]
fn green_checks_boot_the_crew_at_once() {
    let mut a = app(green(), boots, Open::Default);
    assert_eq!(a.current(), Tab::Flight);
    assert!(matches!(a.crew, Crew::Booting));
    a.poll();
    assert!(matches!(a.crew, Crew::Up));
    let screen = render(&mut a, 120, 30);
    assert!(screen.contains("systems"), "{screen}");
    assert!(screen.contains("abc1234"), "{screen}");
    assert!(!screen.contains("waiting on"), "{screen}");
}

/// `xfc home` / `xfc setup` open on Checks; the crew still boots.
#[test]
fn the_setup_alias_opens_on_checks_and_still_boots() {
    let mut a = app(green(), boots, Open::Checks);
    assert_eq!(a.current(), Tab::Checks);
    a.poll();
    assert!(matches!(a.crew, Crew::Up));
}

/// A required check red: the app opens on Checks, the crew waits and the
/// Flight tab names what it waits on; the moment it passes, it boots.
#[test]
fn a_red_required_check_holds_the_crew_until_it_passes() {
    let mut a = app(
        Facts {
            plugin: Verdict::Missing,
            ..green()
        },
        boots,
        Open::Default,
    );
    assert_eq!(a.current(), Tab::Checks);
    assert!(matches!(&a.crew, Crew::Waiting(w) if w == &vec!["Plugin not installed".to_string()]));
    let screen = render(&mut a, 120, 30);
    assert!(
        screen.contains("waiting on: Plugin not installed"),
        "{screen}"
    );
    a.go(Tab::Flight);
    let screen = render(&mut a, 120, 30);
    assert!(screen.contains("The crew is waiting on:"), "{screen}");
    assert!(screen.contains("✗ Plugin not installed"), "{screen}");
    // Typing is refused until the crew is up: Enter (into the page, then
    // on it) does nothing.
    press(&mut a, KeyCode::Enter);
    assert!(a.on_page);
    press(&mut a, KeyCode::Enter);
    assert!(!a.dashboard.editing());

    // The plugin installed (here, by hand): the next look boots it.
    a.checks.facts.plugin = Verdict::UpToDate;
    a.go(Tab::Checks);
    press(&mut a, KeyCode::Down);
    assert!(matches!(a.crew, Crew::Booting));
    a.poll();
    assert!(matches!(a.crew, Crew::Up));
}

#[test]
fn a_failed_boot_is_said_and_r_starts_again() {
    let mut a = app(green(), fails, Open::Default);
    a.poll();
    assert!(matches!(a.crew, Crew::Failed(_)));
    let screen = render(&mut a, 120, 30);
    assert!(
        screen.contains("The crew is not running: no whisper model"),
        "{screen}"
    );
    // It never came up, nothing of ours runs: R restarts at once, no gate.
    assert_eq!(press(&mut a, KeyCode::Char('R')), Some(Exit::Restart));

    // Up once, then its loop ended: its threads may still fly, so R asks
    // and gates as for a running crew.
    let mut a = app(green(), boots, Open::Default);
    a.poll();
    let (tx, rx) = channel();
    a.reports = Some(rx);
    tx.send(Report::BootFailed("voice loop ended".into()))
        .unwrap();
    a.poll();
    assert!(matches!(a.crew, Crew::Failed(_)));
    assert_eq!(press(&mut a, KeyCode::Char('R')), None);
    assert!(matches!(a.restart, Restart::Confirm));
}

/// R with the crew up asks first; any key but y keeps it running. Before
/// it has started there is nothing to restart.
#[test]
fn the_restart_asks_first_and_never_before_the_crew_is_up() {
    let mut a = app(green(), silent, Open::Default);
    assert!(matches!(a.crew, Crew::Booting));
    assert_eq!(press(&mut a, KeyCode::Char('R')), None);
    assert!(a.notice.as_ref().unwrap().0.contains("has not started"));

    let mut a = app(green(), boots, Open::Default);
    a.poll();
    assert_eq!(press(&mut a, KeyCode::Char('R')), None);
    let screen = render(&mut a, 120, 30);
    assert!(
        screen.contains("restart the crew now (on the ground only)?"),
        "{screen}"
    );
    assert_eq!(press(&mut a, KeyCode::Char('n')), None);
    assert!(a.notice.as_ref().unwrap().0.contains("keeps running"));
}

/// Down on the sidebar reaches every page, showing each at once, and each
/// renders, cramped or wide, sidebar or page focused, with no panic.
#[test]
fn the_sidebar_reaches_every_page_and_each_renders() {
    let _guard = crate::settings::TEST_LOCK
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let mut a = app(green(), boots, Open::Default);
    a.poll();
    let n = a.tabs.len();
    for i in 0..n {
        assert_eq!(a.tab, i);
        for on_page in [false, true] {
            a.on_page = on_page;
            for (w, h) in [(120u16, 32u16), (80, 24), (40, 10)] {
                render(&mut a, w, h);
            }
        }
        a.on_page = false;
        let screen = render(&mut a, 120, 32);
        let label = a.current().label();
        assert!(screen.contains(label), "{label}: {screen}");
        press(&mut a, KeyCode::Down);
    }
    assert_eq!(a.current(), Tab::Files, "Down stops at the last page");
    press(&mut a, KeyCode::Home);
    assert_eq!(a.current(), Tab::Flight);
}

/// From any page and any depth Esc goes back one level at a time to the
/// sidebar and never quits; `q` on a page is nobody's, on the sidebar it
/// quits.
#[test]
fn esc_never_quits_and_q_quits_from_the_sidebar_only() {
    let _guard = crate::settings::TEST_LOCK
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let mut a = app(green(), boots, Open::Default);
    a.poll();
    for t in Tab::all() {
        a.go(t);
        assert_eq!(press(&mut a, KeyCode::Right), None);
        assert!(a.on_page, "{t:?}");
        assert_eq!(press(&mut a, KeyCode::Char('q')), None, "{t:?}");
        // One level in where the page has one: a settings editor.
        if matches!(t, Tab::Settings(_)) {
            press(&mut a, KeyCode::Enter);
        }
        for _ in 0..2 {
            assert_eq!(press(&mut a, KeyCode::Esc), None, "{t:?}");
        }
        assert!(!a.on_page, "{t:?}: two Esc reach the sidebar");
        assert_eq!(press(&mut a, KeyCode::Esc), None, "{t:?}");
    }
    assert_eq!(press(&mut a, KeyCode::Char('q')), Some(Exit::Quit));
}

/// A click on a sidebar entry shows its page; the wheel over the sidebar
/// moves it.
#[test]
fn the_mouse_picks_a_page() {
    let mut a = app(green(), boots, Open::Default);
    a.poll();
    render(&mut a, 120, 32);
    let (side, _) = a.side;
    let checks = a.tabs.iter().position(|t| *t == Tab::Checks).unwrap() as u16;
    a.mouse(
        MouseEventKind::Down(MouseButton::Left),
        side.x + 2,
        side.y + checks,
    );
    assert_eq!(a.current(), Tab::Checks);
    a.mouse(MouseEventKind::ScrollDown, side.x + 2, side.y);
    assert_eq!(a.current(), Tab::Voices(0));
    a.mouse(
        MouseEventKind::Down(MouseButton::Left),
        side.right() + 5,
        side.y + 3,
    );
    assert!(a.on_page);
}

/// `/` searches every setting by label and help; Esc closes it; `P` lists
/// the pinned ones.
#[test]
fn search_finds_a_setting_from_any_tab() {
    let _guard = crate::settings::TEST_LOCK
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let mut a = app(green(), boots, Open::Default);
    a.poll();
    press(&mut a, KeyCode::Char('/'));
    for c in "telephony".chars() {
        press(&mut a, KeyCode::Char(c));
    }
    // `q` types into the query, it does not quit.
    assert_eq!(press(&mut a, KeyCode::Char('q')), None);
    press(&mut a, KeyCode::Backspace);
    press(&mut a, KeyCode::Enter);
    let rows = a.search.as_ref().unwrap().pane.rows();
    assert!(
        rows.iter().any(|s| s.key == "airline_telephony"),
        "{:?}",
        rows.len()
    );
    let screen = render(&mut a, 120, 30);
    assert!(screen.contains("search: telephony"), "{screen}");
    press(&mut a, KeyCode::Esc);
    assert!(a.search.is_none());
    press(&mut a, KeyCode::Char('P'));
    let s = a.search.as_ref().unwrap();
    assert!(s.pinned && !s.typing);
    assert!(s.pane.rows().iter().all(|s| s.env_pinned()));
}

/// The Flight tab's typed line: Enter opens it once the crew is up, keys
/// type (q included), Enter sends it on the crew's channel.
#[test]
fn the_typed_line_reaches_the_crew() {
    // The channel's stamps are process-wide (`pollers::QUEUED`).
    let _q = crate::pollers::QUEUE_TEST_LOCK
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let mut a = app(green(), boots, Open::Default);
    a.poll();
    let (tx, rx) = channel();
    a.cmd_tx = Some(tx);
    press(&mut a, KeyCode::Enter);
    press(&mut a, KeyCode::Enter);
    assert!(a.dashboard.editing());
    for c in "qnh 2992".chars() {
        assert_eq!(press(&mut a, KeyCode::Char(c)), None);
    }
    press(&mut a, KeyCode::Enter);
    assert!(rx.try_recv().unwrap().contains("qnh 2992"));
    assert!(
        crate::pollers::took_command().is_some(),
        "stamped like the panel's"
    );
    press(&mut a, KeyCode::Esc);
    assert_eq!(press(&mut a, KeyCode::Char('q')), None, "q on a page");
    press(&mut a, KeyCode::Left);
    assert_eq!(press(&mut a, KeyCode::Char('q')), Some(Exit::Quit));
}

/// The ATC tab's "cast at" box takes an ICAO only, upper-cased, four at
/// most; a short one is refused before any worker runs.
#[test]
fn the_cast_at_box_takes_an_icao() {
    let mut a = app(green(), boots, Open::Default);
    a.go(Tab::Settings(Some(Section::Atc)));
    press(&mut a, KeyCode::Enter);
    press(&mut a, KeyCode::Char('a'));
    for c in "le-mgx".chars() {
        press(&mut a, KeyCode::Char(c));
    }
    assert_eq!(a.cast_at.field.as_deref(), Some("LEMG"));
    let screen = render(&mut a, 120, 32);
    assert!(screen.contains("cast at: LEMG"), "{screen}");
    press(&mut a, KeyCode::Esc);
    assert!(a.cast_at.field.is_none());
    press(&mut a, KeyCode::Char('a'));
    press(&mut a, KeyCode::Char('L'));
    press(&mut a, KeyCode::Enter);
    assert!(a.cast_at.pending.is_none());
    assert!(a.notice.as_ref().unwrap().0.contains("3 or 4"));
}
