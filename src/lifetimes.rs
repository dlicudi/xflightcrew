//! The crew's lifetime manifest: every production static in `src/`, by its
//! module path, with how long the fact it holds may live. The world side has
//! its own (`atc::station::state`, `every_atc_static_has_a_lifetime`); this
//! is the crew's. `every_crew_static_has_a_lifetime` scans the source and
//! fails on a static declared without an entry here (a `#[cfg(test)]` one
//! excepted), and on an entry whose static is gone.
//!
//! The recurring bug class is a fact that outlives its context (CLAUDE.md:
//! "facts must never outlive their airframe"). Naming the lifetime when the
//! static is added is the cheap half of the cure; the doors in
//! `flight_state` (`publish_airframe`, `forget_flight`, `stand_down`) are the
//! other half, and an `A` or `F` entry should be reset by its door.
//!
//! | Code | Lifetime | Reset by |
//! |---|---|---|
//! | `P` | process: config, locks, the audio plumbing | never |
//! | `S` | session with the sim: memos of what it said, counters | never, or its own expiry |
//! | `A` | airframe: a selector from the profile or the `.acf` | `flight_state::publish_airframe` |
//! | `F` | flight: a fact about the flight being flown | `flight_state::forget_flight` or its own path |
//! | `M` | mission: one full auto / route / lesson | the mission's own stop |
//! | `L` | loop: a run's flag, lease, panel detail or hand-down | the loop's exit, `flight_state::stand_down` |
//! | `C` | const or memo cache: a table, a file read once, an epoch | never (holds no flight fact) |

#![cfg_attr(not(test), allow(dead_code))]

/// Every production static under `src/`, as `module::path::NAME` and its
/// lifetime code (see the module table).
pub(crate) const LIFETIMES: &[(&str, char)] = &[
    ("aircraft::AIRFRAME_FACTS", 'A'),
    ("aircraft::TAKEOFF_MARGIN_WAIVED", 'M'),
    ("answers::NEXT", 'C'),
    ("approach_guide::GUIDE", 'L'),
    ("auto_status::AP_TARGETS", 'L'),
    ("auto_status::LAST_SAID", 'S'),
    ("autoflight::LEDGER", 'S'),
    ("autoland::ACTIVE", 'L'),
    ("autoland::GLIDEPATH_DEG", 'L'),
    ("autoland::LANDED_RUNWAY", 'F'),
    ("autoland::LANDING_CONFIGURED", 'L'),
    ("autoland::PANEL", 'L'),
    ("autoland::RUN", 'L'),
    ("autoland::TRACKING_ERROR", 'F'),
    ("autoland::VACATE_FOLLOWS", 'F'),
    ("autotakeoff::ACTIVE", 'L'),
    ("autotakeoff::CLIMB_HANDOVER", 'L'),
    ("autotakeoff::DETAIL", 'L'),
    ("autotakeoff::REJECT", 'L'),
    ("autotakeoff::RUN", 'L'),
    ("autotaxi::ACTIVE", 'L'),
    ("autotaxi::BACKTRACKING", 'L'),
    ("autotaxi::DETAIL", 'L'),
    ("autotaxi::DRIVES_STARTED", 'S'),
    ("autotaxi::EPOCH", 'C'),
    ("autotaxi::LAST_ARRIVED", 'F'),
    ("autotaxi::PROGRESS", 'L'),
    ("autotaxi::RELOCATED", 'L'),
    ("autotaxi::RUN", 'L'),
    ("cabin::STAFFING", 'P'),
    ("circuit::ACTIVE", 'L'),
    ("circuit::DETAIL", 'L'),
    ("circuit::LAST_BUG", 'L'),
    ("circuit::RUN", 'L'),
    ("circuit::STOOD_DOWN", 'M'),
    ("climb_watch::ARMED", 'L'),
    ("climb_watch::EPOCH", 'C'),
    ("climb_watch::SIM_SPEED", 'S'),
    ("climb_watch::TELEMETRY", 'S'),
    ("cockpit_auto::PILOT_MOVED_LIGHTS", 'F'),
    ("crew_radio::CREW_RADIO", 'F'),
    ("crew_radio::LAST_READBACK", 'F'),
    ("detect::PROBE_INDEX", 'S'),
    ("efb::feed::ACTIVITY", 'S'),
    ("efb::feed::ALERTS", 'F'),
    ("efb::feed::ALERT_SEQ", 'S'),
    ("efb::feed::AUDIT", 'F'),
    ("efb::feed::CALLSIGN", 'S'),
    ("efb::feed::COM1", 'S'),
    ("efb::feed::LINES", 'S'),
    ("efb::feed::PREFLIGHT", 'F'),
    ("efb::feed::RADIO_PENDING", 'S'),
    ("efb::feed::TICK", 'S'),
    ("efb::mod::SERVING", 'S'),
    ("efb::ptt::AVAILABLE", 'P'),
    ("efb::ptt::HOLD", 'S'),
    ("flight::PILOT_TAKEOVER", 'F'),
    ("flight::PUBLISHED", 'S'),
    ("flight_state::FLIGHTS_BEGUN", 'S'),
    ("flight_state::LAST_BEGUN", 'S'),
    ("flight_state::LOOPS", 'C'),
    ("flow::ACTIVE", 'L'),
    ("flow::DETAIL", 'L'),
    ("flow::LISTS", 'A'),
    ("flow::OVERRIDES", 'A'),
    ("flow::PROPOSAL", 'F'),
    ("flow::REPLY", 'L'),
    ("flow::RUN", 'L'),
    ("flow::RUNNING", 'L'),
    ("flow::SHEET", 'F'),
    ("flow::STOOD_DOWN", 'F'),
    ("flow::WAITING", 'L'),
    ("free_nose_replay::NET", 'C'),
    ("full_auto::ABORT", 'M'),
    ("full_auto::ACTIVE", 'M'),
    ("full_auto::ARRIVAL_RUNWAY", 'M'),
    ("full_auto::DEPARTURE_MINIMUMS", 'M'),
    ("full_auto::INBOUND_MADE", 'M'),
    ("full_auto::MISSION", 'M'),
    ("full_auto::REFUSED", 'M'),
    ("full_auto::RUN", 'L'),
    ("full_auto::RUNWAY_WAIVER_PENDING", 'M'),
    ("full_auto::STAGE", 'M'),
    ("full_auto::WAIT", 'L'),
    ("full_auto::WAIVER", 'M'),
    ("go_around::ANSWER", 'L'),
    ("go_around::LAST_TOWER_ORDER", 'F'),
    ("go_around::PILOT_ORDERS", 'S'),
    ("health::PLUGIN_MISSING", 'S'),
    ("heli_circuit::ACTIVE", 'L'),
    ("heli_circuit::DETAIL", 'L'),
    ("heli_circuit::RUN", 'L'),
    ("heli_route::ACTIVE", 'M'),
    ("heli_route::DETAIL", 'L'),
    ("heli_route::INBOUND_SENT", 'M'),
    ("heli_route::RUN", 'L'),
    ("hover::ACTIVE", 'L'),
    ("hover::GOAL", 'L'),
    ("hover::MOTION", 'L'),
    ("hover::RELEASED", 'L'),
    ("hover::RUN", 'L'),
    ("hover::TURN_MAG", 'L'),
    ("ils::SETUP", 'L'),
    ("instructor::LESSON", 'M'),
    ("instructor::START", 'C'),
    ("main::LAST_READBACK", 'F'),
    ("main::LISTEN_OFF", 'P'),
    ("mode::DEMONSTRATION", 'L'),
    ("mode::MODE", 'P'),
    ("monitor::BRAKE_LATCH", 'S'),
    ("monitor::CALLSIGN_STALE", 'P'),
    ("monitor::CREW_YOKE_MS", 'S'),
    ("monitor::DESTINATION", 'F'),
    ("monitor::EPOCH", 'C'),
    ("monitor::LAST", 'S'),
    ("monitor::PANEL_MUTE", 'P'),
    ("monitor::PILOT_RELEASE_MS", 'S'),
    ("monitor::PREV", 'S'),
    ("plan::ELEV_CACHE", 'C'),
    ("plan_audit::GRID", 'C'),
    ("pollers::QUEUED", 'S'),
    ("pollers::UI_SCALE", 'P'),
    ("procedure::ABORT", 'L'),
    ("procedure::ACTIVE", 'L'),
    ("procedure::CLEARED_CEILING", 'F'),
    ("procedure::DETAIL", 'L'),
    ("procedure::DRAG_OURS", 'L'),
    ("procedure::FIX_DIST", 'L'),
    ("procedure::LEG", 'L'),
    ("procedure::MISSED_LEGS", 'L'),
    ("procedure::PILOT_ALT", 'L'),
    ("procedure::PILOT_SPEED", 'L'),
    ("procedure::ROUTE_AHEAD", 'L'),
    ("procedure::RUN", 'L'),
    ("procedure::SPLICE", 'L'),
    ("procedure::TERRAIN_FLOOR", 'L'),
    ("procedure::TOD", 'F'),
    ("pushback::ABORT", 'L'),
    ("pushback::ACTIVE", 'L'),
    ("pushback::LEARNED_SENSE", 'S'),
    ("pushback::NATIVE", 'L'),
    ("pushback::PUSH", 'L'),
    ("pushback::SKIP", 'L'),
    ("pushback::STEERED", 'L'),
    ("pushback::TUG_WARNED", 'F'),
    ("rapport::LAST_LANDING_BY_AUTOMATION", 'F'),
    ("rapport::OFFER", 'F'),
    ("readback::LAST", 'C'),
    ("readback::STATE", 'C'),
    ("runway::RUNWAY_WIDTHS", 'C'),
    ("scorecard::LIVE", 'M'),
    ("settings::LIVE", 'P'),
    ("simbrief::SHELF", 'S'),
    ("simbrief::SHELF_AT", 'S'),
    ("systems::CURRENT", 'A'),
    ("taxi_in::ACTIVE", 'L'),
    ("taxi_in::ARREST_HOLD", 'F'),
    ("taxi_in::CANCEL_HOLD", 'L'),
    ("taxi_in::RUN", 'L'),
    ("tempo::CAPTURE_END", 'S'),
    ("tempo::TURN", 'S'),
    ("tempo::TURN_GEN", 'S'),
    ("tempo::WORKLOAD_HIGH", 'S'),
    ("terrain::MAILBOX", 'P'),
    ("test_card::ITEMS", 'C'),
    ("tools::anti_ice::CLAIMED", 'A'),
    ("tools::autopilot::CAPTURE", 'L'),
    ("tools::cabin_air::MASKS", 'S'),
    ("tools::controls::BRAKES_HELD", 'L'),
    ("tools::controls::BRAKE_HOLD", 'L'),
    ("tools::engine::LEAN_SEEK", 'L'),
    ("tools::engine::LEAN_SEEKING", 'L'),
    ("tools::engine::MIXTURE_MANAGER", 'L'),
    ("tools::engine::MIX_MANAGED", 'L'),
    ("tools::full_auto::SAID", 'S'),
    ("tools::ground::CART_REQUESTED", 'F'),
    ("tools::instruments::ALTIMETER_TRUTH", 'A'),
    ("tools::instruments::FLIGHT_TRANSITION", 'F'),
    ("tools::jet_start::JET_START", 'L'),
    ("tools::mod::LAST_BRAKE_CMD", 'S'),
    ("tools::power::POWER_UP", 'L'),
    ("tools::recover::ACTIVE", 'L'),
    ("tools::recover::RUN", 'L'),
    ("tools::reverse::ACTIVE", 'L'),
    ("tools::reverse::WATCH", 'L'),
    ("tools::setup::KNOWN_INIT_SEQ", 'S'),
    ("tools::setup::LIB", 'C'),
    ("tools::setup::NEW_FLIGHT_LOADING", 'S'),
    ("tools::setup::OWN_INITS", 'S'),
    ("tools::speedhold::ACTIVE", 'L'),
    ("tools::speedhold::ACTUAL", 'L'),
    ("tools::speedhold::GOVERNOR", 'L'),
    ("tools::speedhold::LAST_TAXI_KTS", 'S'),
    ("tools::speedhold::MODE", 'L'),
    ("tools::speedhold::TARGET_KTS", 'L'),
    ("tools::taxi::SAID", 'S'),
    ("tts::AUDIO_BUS", 'P'),
    ("tts::AUDIO_BUS_PINNED", 'P'),
    ("tts::BUS_BASE_URL", 'P'),
    ("tts::BUS_CLIENT", 'P'),
    ("tts::BUS_IDS", 'S'),
    ("tts::BUS_WARNED", 'S'),
    ("tts::CUT_GEN", 'P'),
    ("tts::PLAYER_PID", 'P'),
    ("tts::THREAD_CHANNEL", 'P'),
    ("tts::VOLUME_BITS", 'P'),
    ("tts_onnx::ORT_SESSION_INIT", 'P'),
    ("voice::LINES_QUEUED", 'S'),
    ("voice::PILOT_ENGINE", 'P'),
    ("weather::WINDS_CACHE", 'C'),
];

#[cfg(test)]
mod tests {
    use super::LIFETIMES;
    use std::collections::{BTreeMap, BTreeSet};
    use std::path::Path;

    /// The name a line declares as a static (`static`, `pub static`,
    /// `pub(crate) static`, `static mut`, a `thread_local!` entry), else None.
    fn static_name(line: &str) -> Option<String> {
        let mut rest = line.trim_start();
        if rest.starts_with("//") {
            return None;
        }
        if let Some(r) = rest.strip_prefix("pub") {
            rest = r.trim_start();
            if rest.starts_with('(') {
                rest = &rest[rest.find(')')? + 1..];
            }
            rest = rest.trim_start();
        }
        let rest = rest.strip_prefix("static ")?.trim_start();
        let rest = rest.strip_prefix("mut ").unwrap_or(rest);
        let name: String = rest
            .chars()
            .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
            .collect();
        (!name.is_empty() && rest[name.len()..].trim_start().starts_with(':')).then_some(name)
    }

    /// A `#[cfg(...)]` that compiles only under test (`not(test)` is live).
    fn test_only_attr(line: &str) -> bool {
        let t = line.trim_start();
        t.starts_with("#[cfg(") && t.contains("test") && !t.contains("not(test)")
    }

    /// The change in brace depth over a line, outside strings, chars and
    /// line comments.
    fn brace_delta(line: &str) -> i32 {
        let mut d = 0;
        let b = line.as_bytes();
        let mut i = 0;
        let mut in_str = false;
        while i < b.len() {
            let c = b[i];
            if in_str {
                if c == b'\\' {
                    i += 1;
                } else if c == b'"' {
                    in_str = false;
                }
            } else if c == b'/' && b.get(i + 1) == Some(&b'/') {
                break;
            } else if c == b'"' {
                in_str = true;
            } else if c == b'\'' && b.get(i + 2) == Some(&b'\'') {
                i += 2;
            } else if c == b'{' {
                d += 1;
            } else if c == b'}' {
                d -= 1;
            }
            i += 1;
        }
        d
    }

    /// The statics a file declares outside `#[cfg(test)]` items.
    fn live_statics(text: &str) -> Vec<String> {
        let mut out = Vec::new();
        let mut pending = false;
        // Inside a test-only item: its brace depth so far, and whether it
        // has opened a block.
        let mut skipping: Option<(i32, bool)> = None;
        for line in text.lines() {
            let t = line.trim_start();
            if let Some((depth, opened)) = skipping.as_mut() {
                let d = brace_delta(line);
                *depth += d;
                *opened |= line.contains('{');
                if *depth <= 0 && (*opened || t.trim_end().ends_with(';')) {
                    skipping = None;
                }
                continue;
            }
            if test_only_attr(t) {
                pending = true;
                continue;
            }
            if pending {
                if t.starts_with("#[") || t.starts_with("//") || t.is_empty() {
                    continue;
                }
                pending = false;
                let d = brace_delta(line);
                let opened = line.contains('{');
                if !(d <= 0 && (opened || t.trim_end().ends_with(';'))) {
                    skipping = Some((d, opened));
                }
                continue;
            }
            if let Some(n) = static_name(line) {
                out.push(n);
            }
        }
        out
    }

    /// `tools/ground.rs` -> `tools::ground`.
    fn module_path(rel: &Path) -> String {
        let s = rel.with_extension("").display().to_string();
        s.replace('/', "::")
    }

    #[test]
    fn every_crew_static_has_a_lifetime() {
        let src = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut found: BTreeSet<String> = BTreeSet::new();
        for f in crate::test_support::rust_files(&src) {
            if crate::test_support::is_test_file(&f) {
                continue;
            }
            let module = module_path(f.strip_prefix(&src).unwrap());
            let text = std::fs::read_to_string(&f).expect("read source");
            for name in live_statics(&text) {
                let key = format!("{module}::{name}");
                assert!(
                    found.insert(key.clone()),
                    "static {key} declared twice: rename one so each has its own lifetime"
                );
            }
        }
        assert!(
            found.len() > 150,
            "the scan found only {} statics",
            found.len()
        );
        let mut table: BTreeMap<&str, char> = BTreeMap::new();
        for (k, c) in LIFETIMES {
            assert!("PSAFMLC".contains(*c), "{k}: unknown lifetime code {c}");
            assert!(
                table.insert(k, *c).is_none(),
                "{k} listed twice in LIFETIMES"
            );
        }
        let missing: Vec<&String> = found
            .iter()
            .filter(|k| !table.contains_key(k.as_str()))
            .collect();
        assert!(
            missing.is_empty(),
            "statics with no lifetime, name them in src/lifetimes.rs LIFETIMES: {missing:#?}"
        );
        let stale: Vec<&&str> = table.keys().filter(|k| !found.contains(**k)).collect();
        assert!(
            stale.is_empty(),
            "LIFETIMES names statics no longer declared: {stale:?}"
        );
    }

    /// A `Scoped::flight()` cell dies with its flight, so its manifest code
    /// says no longer: `F`, or `L` for a loop's hand-down. A cell declared
    /// flight-scoped but listed as living longer is one of the two wrong.
    #[test]
    fn a_flight_scoped_cell_is_listed_as_living_one_flight() {
        let src = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let table: BTreeMap<&str, char> = LIFETIMES.iter().copied().collect();
        let mut scoped = Vec::new();
        for f in crate::test_support::rust_files(&src) {
            if crate::test_support::is_test_file(&f) {
                continue;
            }
            let module = module_path(f.strip_prefix(&src).unwrap());
            let text = std::fs::read_to_string(&f).expect("read source");
            let lines: Vec<&str> = text.lines().collect();
            for (i, line) in lines.iter().enumerate() {
                let Some(name) = static_name(line) else { continue };
                let decl = format!("{line}{}", lines.get(i + 1).copied().unwrap_or(""));
                if decl.contains("Scoped::flight()") {
                    scoped.push(format!("{module}::{name}"));
                }
            }
        }
        assert!(scoped.len() >= 3, "the scoped cells: {scoped:?}");
        for k in &scoped {
            let code = table.get(k.as_str()).copied();
            assert!(
                matches!(code, Some('F') | Some('L')),
                "{k} is Scoped::flight() but listed as {code:?}"
            );
        }
    }

    #[test]
    fn the_scan_reads_every_declaration_shape_and_skips_test_items() {
        let src = "\
static A: u8 = 0;
pub static B: u8 = 0;
pub(crate) static C: u8 = 0;
pub static mut D: u8 = 0;
thread_local! {
    static E: std::cell::Cell<u8> = const { std::cell::Cell::new(0) };
}
/// static FAKE: not a declaration
fn f(x: &'static str) {}
#[cfg(test)]
static T1: u8 = 0;
#[cfg(not(test))]
static G: u8 = 0;
#[cfg(test)]
thread_local! {
    static T2: u8 = 0;
}
#[cfg(test)]
mod tests {
    fn g() { let s = \"}\"; }
    static T3: u8 = 0;
}
static H: u8 = 0;
";
        assert_eq!(live_statics(src), ["A", "B", "C", "D", "E", "G", "H"]);
    }
}
