//! The panel's Checklist tab (2026-10-13): the airframe's PROFILED lists
//! (`profile.checklist`, as [`crate::flow::installed`] holds them) on
//! `xcopilot/status/checklist`, the one section on screen judged live.
//!
//! No list, no fallback: an airframe whose profile names no checklist is
//! published with no titles, and the tab says it has none.
//!
//! The section on screen follows the flight ([`shown`]): the running list,
//! else the flight phase's ([`phase_list`]), else this flight's last list,
//! else the first. The pilot's pick in the combo overrides it only until the
//! next phase change or list start ([`PickGate`]).
//!
//! Live judging reads only while the tab is ON SCREEN (the plugin's
//! [`DR_CHECKLIST_VIEW`]): about once a second, every term of every item of
//! the shown section in one join, each dataref once, judged by the pure
//! [`checklist::judge_read`]. A failed read is no measurement, so no verdict
//! (unknown, never "not yet"). While a flow runs, or for this flight's last
//! list, the flow's sheet ([`crate::flow::sheet`]) overrides the live state
//! of that list's rows ([`state_code`]).
//!
//! Nothing here is kept between ticks but the change gate: the payload is
//! rebuilt from the installed lists every second, so a swap (lists replaced
//! or emptied) is republished at once and nothing outlives its airframe.
//!
//! Wire format, '\n'-separated lines (the plugin parses it in panel.cpp):
//!
//! ```text
//! H|aircraft|shown|sheet|run|phrase|title~title~...
//! kind|state|by|text
//! ```
//!
//! Header: `shown` the index of the section the rows are of (-1: no
//! lists), `sheet` the index of the list the flow's sheet is of (-1: none),
//! `run` 'R' while that flow runs, 'L' for this flight's last list, '-'
//! none, `phrase` the pilot English the Run button types for the shown
//! section (empty: no phrase routes to it alone), then every title.
//! One row per item of the shown section: `kind` A act, V verify, C
//! call-out, N narration; `state` live p pass, n not yet, u unknown (not
//! read, or off screen), '-' no test; from the sheet R now, D done, H held
//! (`by` names the holder), X discrepancy, S skipped, F failed, U unread,
//! W said; `text` the spoken line, last so it may hold anything.

use std::time::{Duration, Instant};

use crate::checklist::{self, Checklist, Item};
use crate::flight::Phase;
use crate::flow::{ItemState, Sheet};
use crate::xplane::XPlaneClient;

/// The panel's view, plugin to app: "1|seq|title" while the Checklist tab
/// is on screen, "0|seq|title" while it is not (title: the pilot's pick,
/// empty for ours; seq: bumped on every pick in the combo). An older plugin
/// sends "1|title" (no seq), or has none: nothing is ever read live.
pub(crate) const DR_CHECKLIST_VIEW: &str = "xcopilot/opt/checklist_view";

/// How often the tab is built (and, on screen, read).
const EVERY: Duration = Duration::from_secs(1);
/// Resent regardless this often: a plugin reload zeroes its buffer.
const RESEND: Duration = Duration::from_secs(10);
/// The plugin's slot is 6 KB; the longest profiled section (34 items) fits.
pub(crate) const CHECKLIST_MAX: usize = 6000;
/// One field's longest, bytes.
const FIELD_MAX: usize = 96;

/// An item's live verdict.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Live {
    Pass,
    NotYet,
    /// Not read (a failed read, or the tab off screen): no verdict.
    Unknown,
    /// A call-out or narration: nothing to read.
    NoTest,
}

impl Live {
    fn code(self) -> char {
        match self {
            Live::Pass => 'p',
            Live::NotYet => 'n',
            Live::Unknown => 'u',
            Live::NoTest => '-',
        }
    }
}

/// A row's state: the sheet's when the runner has decided it, else the
/// live verdict. Pure.
pub(crate) fn state_code(live: Live, sheet: Option<ItemState>) -> char {
    match sheet {
        None | Some(ItemState::Pending) => live.code(),
        Some(ItemState::Now) => 'R',
        Some(ItemState::Done) => 'D',
        Some(ItemState::Held) => 'H',
        Some(ItemState::Discrepancy) => 'X',
        Some(ItemState::Skipped) => 'S',
        Some(ItemState::Failed) => 'F',
        Some(ItemState::Unread) => 'U',
        Some(ItemState::Said) => 'W',
    }
}

fn kind_code(item: &Item) -> char {
    match item {
        Item::Act { .. } => 'A',
        Item::Verify { .. } => 'V',
        Item::CallOut(_) => 'C',
        Item::Narrate(_) => 'N',
    }
}

/// The plugin's view of the tab.
#[derive(Debug, PartialEq, Eq)]
pub(crate) struct View {
    pub open: bool,
    /// The pick's counter; `None` from an older plugin.
    pub seq: Option<u64>,
    pub title: String,
}

/// "1|7|Before takeoff" -> on screen, pick number 7, that title; an older
/// plugin's "1|Before takeoff" has no number. Anything else is off screen.
/// Pure.
pub(crate) fn parse_view(raw: &str) -> View {
    let raw = raw.trim_matches(char::from(0)).trim();
    let (flag, rest) = raw.split_once('|').unwrap_or((raw, ""));
    let (seq, title) = match rest.split_once('|') {
        Some((n, t)) if n.trim().parse::<u64>().is_ok() => (n.trim().parse().ok(), t),
        _ => (None, rest),
    };
    View {
        open: flag.trim() == "1",
        seq,
        title: title.trim().to_string(),
    }
}

/// The flight phase's section key; `None` where no list belongs to it (a
/// parked aircraft shows the first list, a hover the last run). Pure.
pub(crate) fn phase_section(phase: Phase) -> Option<&'static str> {
    use Phase::*;
    match phase {
        EngineStart | Ready | Pushback | TaxiOut | Holding | LinedUp | TakeoffRoll
        | RejectedTakeoff => Some("before takeoff"),
        InitialClimb | Climb | GoAround => Some("climb"),
        Cruise => Some("cruise"),
        Descent => Some("descent"),
        Approach | Landing => Some("before landing"),
        Rollout | TaxiIn => Some("after landing"),
        Parked | Hover => None,
    }
}

/// The phase's list: its section through `find`, the first list when
/// parked, `None` when the phase has none or the file lacks it.
pub(crate) fn phase_list(
    lists: &[Checklist],
    phase: Phase,
    find: impl Fn(&str) -> Option<usize>,
) -> Option<usize> {
    match phase_section(phase) {
        Some(key) => find(key),
        None if phase == Phase::Parked && !lists.is_empty() => Some(0),
        None => None,
    }
}

/// Whether the pilot's pick stands: only one made (its counter bumped)
/// since the last automatic change, a phase list change or a list start.
#[derive(Default)]
pub(crate) struct PickGate {
    /// The pick counter at the last automatic change.
    floor: Option<u64>,
    phase: Option<usize>,
    running: Option<String>,
}

impl PickGate {
    /// One tick: the view's counter, the phase's list and the running
    /// list's title. An older plugin (no counter) keeps its pick. Pure.
    pub(crate) fn step(
        &mut self,
        seq: Option<u64>,
        phase: Option<usize>,
        running: Option<&str>,
    ) -> bool {
        let started = running.is_some() && running != self.running.as_deref();
        let changed = phase != self.phase || started;
        self.phase = phase;
        self.running = running.map(str::to_string);
        let Some(seq) = seq else {
            return true;
        };
        // The first view, an automatic change, or a reloaded plugin's
        // counter starting again: every pick before it is spent.
        if changed || self.floor.is_none_or(|f| seq < f) {
            self.floor = Some(seq);
        }
        self.floor.is_some_and(|f| seq > f)
    }
}

/// The list the sheet is of: its title and length. Pure.
pub(crate) fn sheet_list(lists: &[Checklist], sheet: &Sheet) -> Option<usize> {
    lists
        .iter()
        .position(|c| c.title == sheet.title && c.items.len() == sheet.items.len())
}

/// The section on screen: the pilot's standing pick when it names one,
/// else the running list, else the phase's, else this flight's last list,
/// else the first. `None` with no lists. Pure.
pub(crate) fn shown(
    lists: &[Checklist],
    pick: &str,
    running: Option<usize>,
    phase: Option<usize>,
    last: Option<usize>,
) -> Option<usize> {
    if lists.is_empty() {
        return None;
    }
    let pick = pick.trim();
    let picked = (!pick.is_empty())
        .then(|| {
            lists
                .iter()
                .position(|c| c.title.eq_ignore_ascii_case(pick))
        })
        .flatten();
    [picked, running, phase, last]
        .into_iter()
        .flatten()
        .find(|i| *i < lists.len())
        .or(Some(0))
}

/// The pilot English that starts list `idx` through the command loop: its
/// title's words and "checklist", the very matcher a spoken request meets
/// ([`checklist::match_request`]). `None` when those words reach another
/// list or none: no Run button rather than the wrong list. Pure.
pub(crate) fn run_phrase(lists: &[Checklist], idx: usize) -> Option<String> {
    let cl = lists.get(idx)?;
    let phrase = format!("{} checklist", cl.id.replace('_', " "));
    (checklist::match_request(&phrase, lists)?.id == cl.id).then_some(phrase)
}

/// A field on the wire: no separators, no dashes, bounded.
fn field(s: &str) -> String {
    let s: String = crate::phrase::scrub_dashes(s)
        .chars()
        .map(|c| {
            if matches!(c, '|' | '~' | '\n' | '\r') {
                ' '
            } else {
                c
            }
        })
        .collect();
    let s = s.trim();
    let mut end = s.len().min(FIELD_MAX);
    while !s.is_char_boundary(end) {
        end -= 1;
    }
    s[..end].to_string()
}

/// The tab's payload (format in the module docs). `sheet`: the list the
/// flow's sheet is of, and the sheet; `live` the shown section's verdicts
/// in item order (short or empty: unknown). Whole rows only: a section
/// past the slot ends with a narration naming what was left out. Pure.
pub(crate) fn payload(
    aircraft: &str,
    lists: &[Checklist],
    shown: Option<usize>,
    sheet: Option<(usize, &Sheet)>,
    live: &[Live],
) -> String {
    let idx = |i: Option<usize>| i.map_or("-1".to_string(), |i| i.to_string());
    let run = match sheet {
        Some((_, s)) if s.running => 'R',
        Some(_) => 'L',
        None => '-',
    };
    let phrase = shown.and_then(|i| run_phrase(lists, i)).unwrap_or_default();
    let titles: Vec<String> = lists.iter().map(|c| field(&c.title)).collect();
    let mut out = format!(
        "H|{}|{}|{}|{run}|{phrase}|{}",
        field(aircraft),
        idx(shown),
        idx(sheet.map(|(i, _)| i)),
        titles.join("~")
    );
    let Some(cl) = shown.and_then(|i| lists.get(i)) else {
        return out;
    };
    let sheet = sheet.filter(|(i, _)| Some(*i) == shown).map(|(_, s)| s);
    // Room kept for the closing "not shown" row.
    const TAIL: usize = 40;
    for (i, item) in cl.items.iter().enumerate() {
        let live = match live.get(i) {
            Some(l) => *l,
            None if item.has_test() => Live::Unknown,
            None => Live::NoTest,
        };
        let row = sheet.and_then(|s| s.items.get(i));
        let by = row
            .and_then(|r| r.by.as_deref())
            .map(field)
            .unwrap_or_default();
        let line = format!(
            "\n{}|{}|{by}|{}",
            kind_code(item),
            state_code(live, row.map(|r| r.state)),
            field(crate::flow::item_line(item))
        );
        if out.len() + line.len() + TAIL > CHECKLIST_MAX {
            let left = cl.items.len() - i;
            out.push_str(&format!("\nN|-||{left} more items not shown"));
            break;
        }
        out.push_str(&line);
    }
    out
}

/// One dataref's read: a scalar, or a whole array for the indexed terms.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Value {
    Scalar(f64),
    Array(Vec<f64>),
}

/// The datarefs a section's pass tests read, each once: (name, array).
pub(crate) fn read_keys(cl: &Checklist) -> Vec<(String, bool)> {
    let mut keys: Vec<(String, bool)> = Vec::new();
    for a in cl.items.iter().flat_map(Item::atoms) {
        let k = (a.dataref, a.index.is_some());
        if !keys.contains(&k) {
            keys.push(k);
        }
    }
    keys
}

/// Every item judged from the reads of [`read_keys`] (`None`: that read
/// failed, so every term on it is no measurement). Pure.
pub(crate) fn judge_all(
    cl: &Checklist,
    keys: &[(String, bool)],
    reads: &[Option<Value>],
) -> Vec<Live> {
    cl.items
        .iter()
        .map(|item| {
            if !item.has_test() {
                return Live::NoTest;
            }
            let values: Vec<Option<f64>> = item
                .atoms()
                .iter()
                .map(|a| {
                    let k = keys
                        .iter()
                        .position(|(d, arr)| *d == a.dataref && *arr == a.index.is_some())?;
                    match (reads.get(k)?.as_ref()?, a.index) {
                        (Value::Scalar(v), None) => Some(*v),
                        (Value::Array(v), Some(i)) => v.get(i).copied(),
                        _ => None,
                    }
                })
                .collect();
            match checklist::judge_read(item, &values) {
                Some(true) => Live::Pass,
                Some(false) => Live::NotYet,
                None => Live::Unknown,
            }
        })
        .collect()
}

/// Read a section's pass tests: every dataref once, all in one join.
async fn read_live(xp: &XPlaneClient, cl: &Checklist) -> Vec<Live> {
    let keys = read_keys(cl);
    let reads: Vec<Option<Value>> =
        futures::future::join_all(keys.iter().map(|(d, arr)| async move {
            if *arr {
                xp.get_float_array(d).await.ok().map(Value::Array)
            } else {
                xp.get_float(d).await.ok().map(Value::Scalar)
            }
        }))
        .await;
    judge_all(cl, &keys, &reads)
}

/// The tab's publisher, ticked by the monitor. Holds only its change gate
/// and the view's back-off.
#[derive(Default)]
pub(crate) struct ChecklistTab {
    at: Option<Instant>,
    sent: Option<Instant>,
    last: String,
    /// A plugin without the view is not asked again until then.
    view_quiet_until: Option<Instant>,
    picks: PickGate,
}

impl ChecklistTab {
    /// Build (and on screen, read) at most once a second; publish on a
    /// change and every [`RESEND`]. `aircraft` is asked only when due;
    /// `phase` is the flight model's.
    pub(crate) async fn tick(
        &mut self,
        xp: &XPlaneClient,
        sink: &crate::plugin_status::StatusSink,
        aircraft: impl FnOnce() -> String,
        phase: Phase,
        now: Instant,
    ) {
        if self
            .at
            .is_some_and(|t| now.saturating_duration_since(t) < EVERY)
        {
            return;
        }
        self.at = Some(now);
        let lists = crate::flow::installed();
        let view = if lists.is_empty() {
            None
        } else {
            self.view(xp, now).await
        };
        let sheet = crate::flow::sheet();
        let sheet_idx = sheet.as_ref().and_then(|s| sheet_list(&lists, s));
        let running = sheet.as_ref().filter(|s| s.running);
        let phase_idx = phase_list(&lists, phase, crate::flow::find_section);
        let stands = self.picks.step(
            view.as_ref().and_then(|v| v.seq),
            phase_idx,
            running.map(|s| s.title.as_str()),
        );
        let pick = view
            .as_ref()
            .filter(|_| stands)
            .map_or("", |v| v.title.as_str());
        let shown = shown(
            &lists,
            pick,
            sheet_idx.filter(|_| running.is_some()),
            phase_idx,
            sheet_idx,
        );
        // Off screen nothing is read, so nothing is judged.
        let live = match shown.filter(|_| view.as_ref().is_some_and(|v| v.open)) {
            Some(i) => read_live(xp, &lists[i]).await,
            None => Vec::new(),
        };
        let text = payload(
            &aircraft(),
            &lists,
            shown,
            sheet_idx.zip(sheet.as_ref()),
            &live,
        );
        let due = text != self.last
            || self
                .sent
                .is_none_or(|t| now.saturating_duration_since(t) >= RESEND);
        if due {
            sink.checklist(&text);
            self.last = text;
            self.sent = Some(now);
        }
    }

    /// The plugin's view; `None` for a plugin without it (not asked again
    /// for a minute).
    async fn view(&mut self, xp: &XPlaneClient, now: Instant) -> Option<View> {
        if self.view_quiet_until.is_some_and(|t| now < t) {
            return None;
        }
        match xp.get_string(DR_CHECKLIST_VIEW).await {
            Ok(v) => {
                self.view_quiet_until = None;
                Some(parse_view(&v))
            }
            Err(_) => {
                self.view_quiet_until = Some(now + Duration::from_secs(60));
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flow::{Flavour, SheetItem};

    const TEXT: &str = "\
sw_checklist:BEFORE TAKEOFF
sw_remark:Run-up
sw_item:PARKING BRAKE|SET:sim/flightmodel/controls/parkbrake:1
sw_item:THROTTLE|1800 RPM:sim/cockpit2/engine/actuators/throttle_ratio[0]:>0.5
sw_item:FLIGHT CONTROLS|FREE AND CORRECT
sw_checklist:LANDING
sw_item:FLAPS|AS REQUIRED
";

    fn lists() -> Vec<Checklist> {
        let profile = crate::aircraft::load("cessna_172sp").expect("profile");
        checklist::parse(TEXT, &profile)
    }

    fn sheet(states: &[ItemState], running: bool) -> Sheet {
        Sheet {
            title: "Before takeoff".into(),
            flavour: Flavour::Crew,
            items: states
                .iter()
                .map(|s| SheetItem {
                    line: String::new(),
                    state: *s,
                    by: (*s == ItemState::Held).then(|| "the speed hold".to_string()),
                })
                .collect(),
            running,
            summary: None,
        }
    }

    #[test]
    fn the_sheet_overrides_the_live_state_and_pending_falls_through() {
        assert_eq!(state_code(Live::Pass, None), 'p');
        assert_eq!(state_code(Live::NotYet, Some(ItemState::Pending)), 'n');
        assert_eq!(state_code(Live::Unknown, None), 'u');
        assert_eq!(state_code(Live::NoTest, None), '-');
        assert_eq!(state_code(Live::NotYet, Some(ItemState::Done)), 'D');
        assert_eq!(state_code(Live::Pass, Some(ItemState::Discrepancy)), 'X');
        assert_eq!(state_code(Live::Pass, Some(ItemState::Now)), 'R');
        assert_eq!(state_code(Live::Pass, Some(ItemState::Held)), 'H');
        assert_eq!(state_code(Live::Pass, Some(ItemState::Failed)), 'F');
        assert_eq!(state_code(Live::Pass, Some(ItemState::Skipped)), 'S');
        assert_eq!(state_code(Live::Pass, Some(ItemState::Unread)), 'U');
        assert_eq!(state_code(Live::NoTest, Some(ItemState::Said)), 'W');
    }

    #[test]
    fn the_payload_carries_the_titles_and_one_row_per_item() {
        let l = lists();
        let live = [Live::NoTest, Live::Pass, Live::NotYet, Live::NoTest];
        let p = payload("Cessna 172", &l, Some(0), None, &live);
        let mut lines = p.lines();
        assert_eq!(
            lines.next(),
            Some("H|Cessna 172|0|-1|-|before takeoff checklist|Before takeoff~Landing")
        );
        let rows: Vec<&str> = lines.collect();
        assert_eq!(rows.len(), l[0].items.len());
        assert!(rows[0].starts_with("N|-||"), "{}", rows[0]);
        assert!(
            rows[1].starts_with("A|p||") || rows[1].starts_with("V|p||"),
            "{}",
            rows[1]
        );
        assert!(rows[2].contains("|n||"), "{}", rows[2]);
        assert!(rows[3].starts_with("C|-||"), "{}", rows[3]);
    }

    #[test]
    fn a_running_flow_overrides_its_own_list_only() {
        let l = lists();
        let s = sheet(
            &[
                ItemState::Said,
                ItemState::Held,
                ItemState::Now,
                ItemState::Pending,
            ],
            true,
        );
        // The sheet is of the list with its title and length.
        assert_eq!(sheet_list(&l, &s), Some(0));
        assert_eq!(sheet_list(&l, &sheet(&[ItemState::Done], true)), None);
        let live = [Live::NoTest, Live::NotYet, Live::Pass, Live::NoTest];
        let p = payload("C172", &l, Some(0), Some((0, &s)), &live);
        let rows: Vec<&str> = p.lines().collect();
        assert!(rows[0].starts_with("H|C172|0|0|R|"), "{}", rows[0]);
        assert!(rows[1].starts_with("N|W||"), "{}", rows[1]);
        assert!(rows[2].contains("|H|the speed hold|"), "{}", rows[2]);
        assert!(rows[3].contains("|R||"), "{}", rows[3]);
        assert!(rows[4].starts_with("C|-||"), "{}", rows[4]);
        // Another section on screen: the sheet is named, never applied.
        let p = payload("C172", &l, Some(1), Some((0, &s)), &[]);
        assert!(p.starts_with("H|C172|1|0|R|"), "{p}");
        assert!(p.lines().nth(1).unwrap().starts_with("C|-||"), "{p}");
        // The last list of the flight, no longer running.
        let done = sheet(&[ItemState::Done; 4], false);
        assert!(payload("C172", &l, Some(0), Some((0, &done)), &[]).starts_with("H|C172|0|0|L|"));
    }

    #[test]
    fn no_list_is_a_header_with_no_titles() {
        assert_eq!(
            payload("Generic", &[], None, None, &[]),
            "H|Generic|-1|-1|-||"
        );
        assert_eq!(shown(&[], "anything", None, None, None), None);
    }

    #[test]
    fn the_view_names_the_pick_and_whether_it_is_on_screen() {
        assert_eq!(
            parse_view("1|3|LANDING"),
            View {
                open: true,
                seq: Some(3),
                title: "LANDING".into()
            }
        );
        // An older plugin's view: no counter.
        assert_eq!(
            parse_view("1|LANDING"),
            View {
                open: true,
                seq: None,
                title: "LANDING".into()
            }
        );
        assert_eq!(
            parse_view("0|0|"),
            View {
                open: false,
                seq: Some(0),
                title: String::new()
            }
        );
        assert!(!parse_view("").open);
        let l = lists();
        assert_eq!(shown(&l, "landing", Some(0), Some(0), None), Some(1));
        assert_eq!(
            shown(&l, "a list the last airframe had", None, None, None),
            Some(0)
        );
    }

    /// The tab follows the flight: the running list, else the phase's, else
    /// the last run, else the first.
    #[test]
    fn the_tab_follows_the_flight() {
        let l = lists();
        assert_eq!(shown(&l, "", Some(0), Some(1), None), Some(0));
        assert_eq!(shown(&l, "", None, Some(1), Some(0)), Some(1));
        assert_eq!(shown(&l, "", None, None, Some(1)), Some(1));
        assert_eq!(shown(&l, "", None, None, None), Some(0));
        let find = |k: &str| (k == "before landing").then_some(1);
        assert_eq!(phase_list(&l, Phase::Approach, find), Some(1));
        assert_eq!(phase_list(&l, Phase::Landing, find), Some(1));
        assert_eq!(phase_list(&l, Phase::Parked, find), Some(0));
        assert_eq!(phase_list(&l, Phase::Cruise, find), None, "no cruise list");
        assert_eq!(phase_list(&l, Phase::Hover, find), None);
        assert_eq!(phase_section(Phase::Holding), Some("before takeoff"));
        assert_eq!(phase_section(Phase::InitialClimb), Some("climb"));
        assert_eq!(phase_section(Phase::TaxiIn), Some("after landing"));
    }

    /// The pilot's pick stands until the next phase change or list start;
    /// a pick made since stands again.
    #[test]
    fn a_pick_stands_until_the_next_automatic_change() {
        let mut g = PickGate::default();
        // The plugin's pick from before the app started is spent.
        assert!(!g.step(Some(4), Some(0), None));
        // A new pick: it stands, through ticks of the same phase.
        assert!(g.step(Some(5), Some(0), None));
        assert!(g.step(Some(5), Some(0), None));
        // The phase's list changes: the pick is spent.
        assert!(!g.step(Some(5), Some(1), None));
        assert!(g.step(Some(6), Some(1), None));
        // A list starts: spent again; it running on is no new start.
        assert!(!g.step(Some(6), Some(1), Some("Landing")));
        assert!(g.step(Some(7), Some(1), Some("Landing")));
        assert!(g.step(Some(7), Some(1), Some("Landing")));
        // A reloaded plugin counts from zero again.
        assert!(!g.step(Some(0), Some(1), Some("Landing")));
        assert!(g.step(Some(1), Some(1), Some("Landing")));
        // An older plugin keeps its pick, as it always did.
        assert!(g.step(None, Some(0), None));
    }

    #[test]
    fn a_failed_read_is_no_verdict_and_each_dataref_is_read_once() {
        let l = lists();
        let keys = read_keys(&l[0]);
        assert_eq!(
            keys,
            vec![
                ("sim/flightmodel/controls/parkbrake".to_string(), false),
                (
                    "sim/cockpit2/engine/actuators/throttle_ratio".to_string(),
                    true
                ),
            ]
        );
        let ok = [Some(Value::Scalar(1.0)), Some(Value::Array(vec![0.2, 0.9]))];
        assert_eq!(
            judge_all(&l[0], &keys, &ok),
            vec![Live::NoTest, Live::Pass, Live::NotYet, Live::NoTest]
        );
        let failed = [None, Some(Value::Array(vec![0.9]))];
        assert_eq!(
            judge_all(&l[0], &keys, &failed),
            vec![Live::NoTest, Live::Unknown, Live::Pass, Live::NoTest]
        );
        // An array too short for the index is no measurement either.
        let short = [Some(Value::Scalar(0.0)), Some(Value::Array(vec![]))];
        assert_eq!(
            judge_all(&l[0], &keys, &short),
            vec![Live::NoTest, Live::NotYet, Live::Unknown, Live::NoTest]
        );
    }

    #[test]
    fn a_long_section_ends_on_a_whole_row() {
        let mut text = String::from("sw_checklist:LONG\n");
        for i in 0..200 {
            text.push_str(&format!(
                "sw_item:A VERY LONG CHALLENGE NAMED NUMBER {i} FOR THE BUDGET|CHECKED AND CONFIRMED\n"
            ));
        }
        let profile = crate::aircraft::load("cessna_172sp").expect("profile");
        let l = checklist::parse(&text, &profile);
        let p = payload("C172", &l, Some(0), None, &[]);
        assert!(p.len() <= CHECKLIST_MAX, "{}", p.len());
        assert!(p.ends_with("more items not shown"), "{p}");
    }

    #[test]
    fn separators_never_leak_from_a_title_or_line() {
        assert_eq!(field("A|B~C\nD"), "A B C D");
        assert_eq!(field("x".repeat(300).as_str()).len(), FIELD_MAX);
    }
}
