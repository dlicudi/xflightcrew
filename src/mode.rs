//! The app's operating mode — which half of the product is on.
//!
//! `Both` is the full crew+world experience. `CopilotOnly` is for pilots who
//! fly with external ATC (VATSIM, SayIntentions, stock): no simulated station
//! ever loads, and radio-keyed speech is none of our business. `AtcOnly` is
//! the world without the crew: tower/approach/ATIS answer the radio, but no
//! intercom persona, no tools, and the LLM is never touched (ATC is fully
//! deterministic). `Instructor` is the crew with its hands off: the
//! intercom persona, the monitor's callouts, the ATC coach and the lessons
//! ([`crate::instructor`]) are on, and every curated tool, fix action and
//! cockpit duty is refused at its door (`crew_actuates`), so nothing of
//! ours can touch a control while the pilot is being taught. The one
//! exception is the instructor's own demonstration ("show me a takeoff"):
//! [`tool_may_actuate`] lets exactly that task's automation through the
//! tool door, once, and nothing else.
//!
//! Runtime-toggleable from the plugin's Settings tab (`xcopilot/opt/mode`);
//! the pick persists via [`crate::prefs`]. Precedence: `XCOPILOT_MODE` env
//! sets the INITIAL mode at boot, a panel change wins from that moment on
//! and becomes the next boot's default. One process-wide atomic, read by the
//! main loop (routing + station lifecycle) and the monitor (which watches
//! tick) — the atc crate itself never reads it: the world stays mode-blind
//! and is simply not driven when it's off.

use std::sync::atomic::{AtomicU8, Ordering};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Both,
    CopilotOnly,
    AtcOnly,
    Instructor,
}

impl Mode {
    /// Parse a config/prefs value. Accepts the mode names plus a few
    /// forgiving spellings; `None` for anything unrecognized.
    pub fn parse(s: &str) -> Option<Mode> {
        match s.trim().to_lowercase().as_str() {
            "both" | "all" | "full" => Some(Mode::Both),
            "copilot" | "copilot-only" | "copilot_only" | "crew" => Some(Mode::CopilotOnly),
            "atc" | "atc-only" | "atc_only" | "world" => Some(Mode::AtcOnly),
            "instructor" | "instruct" | "cfi" | "lesson" => Some(Mode::Instructor),
            _ => None,
        }
    }

    /// Canonical name — what prefs.json stores.
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Both => "both",
            Mode::CopilotOnly => "copilot",
            Mode::AtcOnly => "atc",
            Mode::Instructor => "instructor",
        }
    }

    /// The panel's radio-button index (`xcopilot/opt/mode` value).
    pub fn panel_index(self) -> i64 {
        match self {
            Mode::Both => 0,
            Mode::CopilotOnly => 1,
            Mode::AtcOnly => 2,
            Mode::Instructor => 3,
        }
    }

    pub fn from_panel_index(v: i64) -> Option<Mode> {
        match v {
            0 => Some(Mode::Both),
            1 => Some(Mode::CopilotOnly),
            2 => Some(Mode::AtcOnly),
            3 => Some(Mode::Instructor),
            _ => None,
        }
    }
}

static MODE: AtomicU8 = AtomicU8::new(0);

pub fn set(m: Mode) {
    MODE.store(m.panel_index() as u8, Ordering::Relaxed);
    // A demonstration permit belongs to the instructor's seat.
    if m != Mode::Instructor {
        close_demonstration();
    }
}

pub fn get() -> Mode {
    Mode::from_panel_index(MODE.load(Ordering::Relaxed) as i64).unwrap_or(Mode::Both)
}

/// Is the world side on — station lifecycle, world ticks, radio routing.
pub fn atc_enabled() -> bool {
    get() != Mode::CopilotOnly
}

/// Is the crew side on: the intercom persona, crew watches, the tools'
/// door (which `crew_actuates` may still keep shut).
pub fn copilot_enabled() -> bool {
    get() != Mode::AtcOnly
}

/// May the crew move anything? False in instructor mode (hands off by
/// construction) and with no crew at all. Every writer the crew owns that
/// is not a spoken tool checks THIS: the monitor's fix actions, the cockpit
/// duties, the checklist's Crew flavour.
pub fn crew_actuates() -> bool {
    !matches!(get(), Mode::AtcOnly | Mode::Instructor)
}

/// Is the instructor in the right seat?
pub fn instructing() -> bool {
    get() == Mode::Instructor
}

// ------------------------------------------------------ the demonstration

/// How long a "show me" permit waits for its tool to reach the door. The
/// instructor hands the loop the request's words in the same breath, so the
/// dispatch is immediate; a permit left lying about is no permit.
pub const DEMONSTRATION_ARM_S: f64 = 15.0;

/// The one exception to instructor mode's folded arms: the instructor's
/// own demonstration of a task ("show me a takeoff"), flown by the crew's
/// automation for that task and narrated by the instructor.
struct Demonstration {
    /// The one tool the demonstration may dispatch.
    tool: &'static str,
    /// The arm is one-shot: the first dispatch of `tool` spends it.
    armed: bool,
    opened: std::time::Instant,
    /// The pilot switched it off at the door (the panel's CANCEL, "cancel
    /// autoland"): the demonstration ended in the pilot's hands.
    stopped: bool,
}

static DEMONSTRATION: std::sync::Mutex<Option<Demonstration>> = std::sync::Mutex::new(None);

fn demonstration() -> std::sync::MutexGuard<'static, Option<Demonstration>> {
    DEMONSTRATION.lock().unwrap_or_else(|e| e.into_inner())
}

/// The instructor opens a demonstration of `tool` (only in instructor
/// mode: elsewhere the crew's hands are on anyway).
pub fn open_demonstration(tool: &'static str) {
    if !instructing() {
        return;
    }
    *demonstration() = Some(Demonstration {
        tool,
        armed: true,
        opened: std::time::Instant::now(),
        stopped: false,
    });
}

/// The demonstration is over (handed back, stood down, the mode changed).
pub fn close_demonstration() {
    *demonstration() = None;
}

/// The tool a demonstration stands open for, if any.
#[cfg(test)]
pub fn demonstration_tool() -> Option<&'static str> {
    demonstration().as_ref().map(|d| d.tool)
}

/// Did the pilot switch the demonstration off at the tool door?
pub fn demonstration_stopped() -> bool {
    demonstration().as_ref().is_some_and(|d| d.stopped)
}

/// Is this call a STOP of a tool (its off switch)? A pilot cancelling the
/// instructor's own demonstration is taking the aircraft back, never an
/// actuation to refuse.
fn is_stop(args: &serde_json::Value) -> bool {
    ["off", "cancel"]
        .iter()
        .any(|k| args.get(*k).and_then(|v| v.as_bool()) == Some(true))
}

/// The tool dispatch's hands gate. Everything the crew may do when it
/// actuates; in instructor mode, exactly the open demonstration and nothing
/// else: its tool ONCE (the arm, within [`DEMONSTRATION_ARM_S`]) and that
/// same tool's off switch while it stands. Any other tool, a second arm or
/// a stale permit is refused as before. [`crew_actuates`] itself stays
/// false through a demonstration: the monitor's fix actions, the cockpit
/// duties and the checklist's Crew flavour keep their hands off.
pub fn tool_may_actuate(tool: &str, args: &serde_json::Value) -> bool {
    if crew_actuates() {
        return true;
    }
    if !instructing() {
        return false;
    }
    let mut d = demonstration();
    let Some(demo) = d.as_mut() else {
        return false;
    };
    if demo.tool != tool {
        return false;
    }
    if is_stop(args) {
        demo.stopped = true;
        crate::blackbox::record(
            "instructor",
            serde_json::json!({ "event": "demo_stop_dispatch", "tool": tool, "args": args }),
        );
        return true;
    }
    if demo.armed && demo.opened.elapsed().as_secs_f64() <= DEMONSTRATION_ARM_S {
        demo.armed = false;
        crate::blackbox::record(
            "instructor",
            serde_json::json!({ "event": "demo_dispatch", "tool": tool, "args": args, "by": "instructor" }),
        );
        return true;
    }
    false
}

/// Tests that SET the process-wide mode must hold this, or they stomp each
/// other's assertions across modules (same discipline as the atc crate's
/// SIDE_STATE_LOCK). Set the mode back to `Both` before releasing.
#[cfg(test)]
pub static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_roundtrips_and_forgives() {
        for m in [
            Mode::Both,
            Mode::CopilotOnly,
            Mode::AtcOnly,
            Mode::Instructor,
        ] {
            assert_eq!(Mode::parse(m.as_str()), Some(m));
            assert_eq!(Mode::from_panel_index(m.panel_index()), Some(m));
        }
        assert_eq!(Mode::parse("ATC-Only"), Some(Mode::AtcOnly));
        assert_eq!(Mode::parse("crew"), Some(Mode::CopilotOnly));
        assert_eq!(Mode::parse("Instructor"), Some(Mode::Instructor));
        assert_eq!(Mode::parse(""), None);
        assert_eq!(Mode::parse("banana"), None);
        assert_eq!(Mode::from_panel_index(-1), None);
    }

    #[test]
    fn enablement_follows_mode() {
        let _guard = TEST_LOCK.lock().unwrap();
        set(Mode::AtcOnly);
        assert!(atc_enabled() && !copilot_enabled());
        assert!(!crew_actuates() && !instructing());
        set(Mode::CopilotOnly);
        assert!(!atc_enabled() && copilot_enabled() && crew_actuates());
        set(Mode::Instructor);
        assert!(atc_enabled() && copilot_enabled(), "the persona stays");
        assert!(!crew_actuates() && instructing(), "the hands do not");
        set(Mode::Both);
        assert!(atc_enabled() && copilot_enabled() && crew_actuates());
    }

    #[test]
    fn the_demonstration_opens_exactly_one_tool_once() {
        let _guard = TEST_LOCK.lock().unwrap();
        let on = serde_json::json!({});
        let off = serde_json::json!({ "off": true });
        set(Mode::Instructor);
        close_demonstration();
        assert!(
            !tool_may_actuate("auto_takeoff", &on),
            "no demonstration, no hands"
        );
        open_demonstration("auto_takeoff");
        assert_eq!(demonstration_tool(), Some("auto_takeoff"));
        assert!(
            !tool_may_actuate("set_flaps", &on),
            "another tool stays refused"
        );
        assert!(!tool_may_actuate("autopilot_off", &on));
        assert!(!crew_actuates(), "the monitor's fixes and duties stay off");
        assert!(tool_may_actuate("auto_takeoff", &on), "the arm, once");
        assert!(!tool_may_actuate("auto_takeoff", &on), "never a second arm");
        assert!(!demonstration_stopped());
        assert!(
            tool_may_actuate("auto_takeoff", &off),
            "its off switch while it stands"
        );
        assert!(demonstration_stopped(), "the pilot's cancel is remembered");
        close_demonstration();
        assert!(!tool_may_actuate("auto_takeoff", &off), "closed: nothing");
        // Outside instructor mode the permit is never opened, and the crew
        // actuates by itself anyway.
        set(Mode::AtcOnly);
        open_demonstration("auto_taxi");
        assert_eq!(demonstration_tool(), None);
        assert!(!tool_may_actuate("auto_taxi", &on));
        set(Mode::Both);
        assert!(tool_may_actuate("set_flaps", &on));
    }
}
