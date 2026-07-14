//! The app's operating mode — which half of the product is on.
//!
//! `Both` is the full crew+world experience. `CopilotOnly` is for pilots who
//! fly with external ATC (VATSIM, SayIntentions, stock): no simulated station
//! ever loads, and radio-keyed speech is none of our business. `AtcOnly` is
//! the world without the crew: tower/approach/ATIS answer the radio, but no
//! intercom persona, no tools, and the LLM is never touched (ATC is fully
//! deterministic).
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
}

impl Mode {
    /// Parse a config/prefs value. Accepts the mode names plus a few
    /// forgiving spellings; `None` for anything unrecognized.
    pub fn parse(s: &str) -> Option<Mode> {
        match s.trim().to_lowercase().as_str() {
            "both" | "all" | "full" => Some(Mode::Both),
            "copilot" | "copilot-only" | "copilot_only" | "crew" => Some(Mode::CopilotOnly),
            "atc" | "atc-only" | "atc_only" | "world" => Some(Mode::AtcOnly),
            _ => None,
        }
    }

    /// Canonical name — what prefs.json stores.
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Both => "both",
            Mode::CopilotOnly => "copilot",
            Mode::AtcOnly => "atc",
        }
    }

    /// The panel's radio-button index (`xcopilot/opt/mode` value).
    pub fn panel_index(self) -> i64 {
        match self {
            Mode::Both => 0,
            Mode::CopilotOnly => 1,
            Mode::AtcOnly => 2,
        }
    }

    pub fn from_panel_index(v: i64) -> Option<Mode> {
        match v {
            0 => Some(Mode::Both),
            1 => Some(Mode::CopilotOnly),
            2 => Some(Mode::AtcOnly),
            _ => None,
        }
    }
}

static MODE: AtomicU8 = AtomicU8::new(0);

pub fn set(m: Mode) {
    MODE.store(m.panel_index() as u8, Ordering::Relaxed);
}

pub fn get() -> Mode {
    Mode::from_panel_index(MODE.load(Ordering::Relaxed) as i64).unwrap_or(Mode::Both)
}

/// Is the world side on — station lifecycle, world ticks, radio routing.
pub fn atc_enabled() -> bool {
    get() != Mode::CopilotOnly
}

/// Is the crew side on — LLM/tools, intercom persona, crew watches.
pub fn copilot_enabled() -> bool {
    get() != Mode::AtcOnly
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_roundtrips_and_forgives() {
        for m in [Mode::Both, Mode::CopilotOnly, Mode::AtcOnly] {
            assert_eq!(Mode::parse(m.as_str()), Some(m));
            assert_eq!(Mode::from_panel_index(m.panel_index()), Some(m));
        }
        assert_eq!(Mode::parse("ATC-Only"), Some(Mode::AtcOnly));
        assert_eq!(Mode::parse("crew"), Some(Mode::CopilotOnly));
        assert_eq!(Mode::parse(""), None);
        assert_eq!(Mode::parse("banana"), None);
        assert_eq!(Mode::from_panel_index(-1), None);
    }

    #[test]
    fn enablement_follows_mode() {
        set(Mode::AtcOnly);
        assert!(atc_enabled() && !copilot_enabled());
        set(Mode::CopilotOnly);
        assert!(!atc_enabled() && copilot_enabled());
        set(Mode::Both);
        assert!(atc_enabled() && copilot_enabled());
    }
}
