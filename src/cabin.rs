//! The cabin crew: the second persona on the intercom.
//!
//! Real aircraft mechanics, scaled to v1: the flight deck and the cabin talk
//! over the interphone (chime first, then a voice from the back), the pilot
//! addresses them in words ("cabin crew, prepare for landing" — no third PTT,
//! the address IS the routing), and below 10,000 ft the sterile-cockpit rule
//! keeps the cabin quiet except for safety reports. Everything here is
//! deterministic — no LLM: cabin calls are rare and formulaic, so keyword
//! intents answer them the same way the tower's FSM answers radio calls.
//!
//! Division of responsibility mirrors the crew/world split: this module is
//! pure state + text (testable as plain `cargo test`); the main loop routes
//! addressed calls here and speaks replies via `Voice::say_cabin`; the
//! monitor ticks the owed reports (`tick`) and the seatbelt-sign watch
//! (`seatbelt`) and speaks them via `Voice::alert_cabin`. The persona exists
//! only on airframes whose profile says so (`cabin_crew: true` — a 172 has
//! no galley), overridable from the panel's Settings tab
//! (`xcopilot/opt/cabin_crew`, persisted via [`crate::prefs`]) so any
//! airframe can be crewed for testing; `XCOPILOT_CABIN_CREW` sets only the
//! boot value, same precedence discipline as the app mode.

use std::sync::atomic::{AtomicU8, Ordering};
use std::time::{Duration, Instant};

/// Sterile cockpit floor: below this the cabin initiates nothing but safety
/// reports. The real rule is 10,000 ft MSL; AGL is what the monitor snapshot
/// carries and is the more conservative read over high terrain.
pub const STERILE_FLOOR_AGL_FT: f64 = 10_000.0;

/// How long the cabin takes to secure after an acknowledged "prepare for
/// departure/landing" before the "cabin secure" report comes back. Landing
/// takes longer: carts to stow, a full cabin walk.
const SECURE_SECS_DEPARTURE: u64 = 40;
const SECURE_SECS_LANDING: u64 = 75;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Departure,
    Landing,
}

impl Phase {
    fn spoken(self) -> &'static str {
        match self {
            Phase::Departure => "departure",
            Phase::Landing => "landing",
        }
    }
}

/// Who staffs the back — the runtime setting behind [`aboard`]. `Auto`
/// defers to the airframe profile (`cabin_crew: true`); `Aboard`/`Off`
/// override it either way ("what if I want cabin crew on a 172?" — pick
/// Aboard on the Settings tab and she's aboard). Same lifecycle as
/// [`crate::mode`]: one process-wide atomic, panel-toggleable, persisted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Staffing {
    Auto,
    Aboard,
    Off,
}

impl Staffing {
    /// Parse a config/prefs value. Accepts the canonical names plus the
    /// legacy env spellings (`true`/`false`); `None` for anything else.
    pub fn parse(s: &str) -> Option<Staffing> {
        match s.trim().to_ascii_lowercase().as_str() {
            "auto" | "profile" | "airframe" => Some(Staffing::Auto),
            "aboard" | "true" | "on" | "1" | "yes" => Some(Staffing::Aboard),
            "off" | "false" | "0" | "no" => Some(Staffing::Off),
            _ => None,
        }
    }

    /// Canonical name — what prefs.json stores.
    pub fn as_str(self) -> &'static str {
        match self {
            Staffing::Auto => "auto",
            Staffing::Aboard => "aboard",
            Staffing::Off => "off",
        }
    }

    /// The panel's radio-button index (`xcopilot/opt/cabin_crew` value).
    pub fn panel_index(self) -> i64 {
        match self {
            Staffing::Auto => 0,
            Staffing::Aboard => 1,
            Staffing::Off => 2,
        }
    }

    pub fn from_panel_index(v: i64) -> Option<Staffing> {
        match v {
            0 => Some(Staffing::Auto),
            1 => Some(Staffing::Aboard),
            2 => Some(Staffing::Off),
            _ => None,
        }
    }
}

static STAFFING: AtomicU8 = AtomicU8::new(0);

pub fn set_staffing(s: Staffing) {
    STAFFING.store(s.panel_index() as u8, Ordering::Relaxed);
}

pub fn staffing() -> Staffing {
    Staffing::from_panel_index(STAFFING.load(Ordering::Relaxed) as i64).unwrap_or(Staffing::Auto)
}

/// Is the cabin crew aboard THIS airframe, per the live setting? Read at
/// the point of use (routing, watches) so a panel flip applies to the very
/// next call — no restart, no aircraft swap needed.
pub fn aboard(profile_has_cabin_crew: bool) -> bool {
    match staffing() {
        Staffing::Auto => profile_has_cabin_crew,
        Staffing::Aboard => true,
        Staffing::Off => false,
    }
}

/// The cabin's side of the interphone: what they've secured, what they owe
/// the flight deck, and whether the service is out of the carts.
pub struct Cabin {
    /// What the cabin was last reported secure for.
    secured: Option<Phase>,
    /// An acknowledged "prepare for X" owes a secure report when due.
    pending: Option<(Instant, Phase)>,
    /// Seatbelt-sign state at the last watch tick, for edge detection.
    /// `None` until the first good read (no edge invented at boot).
    seatbelt_on: Option<bool>,
    /// Meal/drinks service is underway (flavor state: the sign coming on
    /// mid-service earns "securing the service" instead of silence).
    service_open: bool,
}

impl Cabin {
    pub fn new() -> Self {
        Cabin {
            secured: None,
            pending: None,
            seatbelt_on: None,
            service_open: false,
        }
    }

    /// The words after the interphone address, if this utterance is FOR the
    /// cabin. The address is the routing (no third PTT — see module docs), so
    /// it must lead the utterance: "cabin crew, seats for departure" routes,
    /// "the cabin crew are great" does not.
    pub fn addressed(text: &str) -> Option<String> {
        let t = text
            .trim()
            .trim_start_matches(['.', ',', ' '])
            .to_ascii_lowercase();
        // An explicit address always routes, whatever follows.
        for prefix in ["cabin crew", "flight attendants", "flight attendant"] {
            if let Some(rest) = t.strip_prefix(prefix) {
                return Some(rest.trim_start_matches([',', '.', ' ', ':']).to_string());
            }
        }
        // Bare "cabin" is ambiguous — cabin lights, cabin pressure, the
        // Lancair's cabin dump switch are all COPILOT business — so it only
        // addresses the crew when the rest is unmistakably a crew instruction.
        if let Some(rest) = t.strip_prefix("cabin") {
            let rest = rest.trim_start_matches([',', '.', ' ', ':']);
            const CREW_WORDS: [&str; 9] = [
                "prepare", "secure", "seats", "stations", "doors", "report", "status", "how",
                "ready",
            ];
            if CREW_WORDS.iter().any(|k| rest.contains(k)) {
                return Some(rest.to_string());
            }
        }
        None
    }

    /// Answer an addressed call. Always answers — an unparsed call gets
    /// "say again", never a guess (same safety property as the tower).
    pub fn handle(&mut self, rest: &str, now: Instant) -> String {
        let r = rest.to_ascii_lowercase();
        let wants = |kws: &[&str]| kws.iter().any(|k| r.contains(k));

        // "prepare for takeoff" / "secure for departure" / "seats for landing"
        let prepare = wants(&["prepare", "secure", "seats", "ready", "stations"]);
        if prepare && wants(&["takeoff", "take off", "departure", "depart"]) {
            self.service_open = false;
            self.pending = Some((
                now + Duration::from_secs(SECURE_SECS_DEPARTURE),
                Phase::Departure,
            ));
            self.secured = None;
            return "Securing the cabin for departure, flight deck.".to_string();
        }
        if prepare && wants(&["landing", "arrival", "approach"]) {
            self.service_open = false;
            self.pending = Some((
                now + Duration::from_secs(SECURE_SECS_LANDING),
                Phase::Landing,
            ));
            self.secured = None;
            return "Securing the cabin for landing, flight deck.".to_string();
        }
        // Airline SOP flavor: arming/disarming the slides.
        if wants(&["doors"]) {
            if wants(&["automatic", "arm"]) {
                return "Doors to automatic, cross checked.".to_string();
            }
            if wants(&["manual", "disarm"]) {
                return "Doors to manual, cross checked.".to_string();
            }
        }
        // "how's the cabin" / "cabin report" / "is the cabin secure"
        if wants(&["how", "report", "status", "secure", "ready", "okay", "ok"]) {
            return match (self.pending, self.secured) {
                (Some((_, p)), _) => format!(
                    "Still securing for {}, flight deck. A minute more.",
                    p.spoken()
                ),
                (None, Some(p)) => format!("Cabin is secure for {}.", p.spoken()),
                (None, None) if self.service_open => {
                    "Cabin's good, service is running.".to_string()
                }
                (None, None) => "All good back here, flight deck.".to_string(),
            };
        }
        // Heard, not understood: ask again, never guess (a misheard cabin
        // call is a shrug, but the pattern is the same everywhere).
        "Flight deck, say again?".to_string()
    }

    /// Owed reports, called every monitor tick. The "cabin secure" report is
    /// a SAFETY report — exempt from the sterile rule (real crews report
    /// cabin secure on descent through the sterile floor; that's the point
    /// of the call). Flavor initiations gate on `sterile`.
    pub fn tick(&mut self, now: Instant, agl_ft: f64, on_ground: bool) -> Option<String> {
        if let Some((due, phase)) = self.pending {
            if now >= due {
                self.pending = None;
                self.secured = Some(phase);
                return Some(format!(
                    "Flight deck, cabin is secure for {}.",
                    phase.spoken()
                ));
            }
        }
        // Through the sterile floor on climb-out with a secured cabin: the
        // crew gets up and the service starts — one flavor line, above the
        // floor by definition.
        if !on_ground
            && agl_ft > STERILE_FLOOR_AGL_FT
            && self.secured == Some(Phase::Departure)
            && self.pending.is_none()
        {
            self.secured = None;
            self.service_open = true;
            return Some("Flight deck, cabin crew released. Starting the service.".to_string());
        }
        // Parked after a landing the cabin was secured for: the flight is
        // over back there; reset silently for the next leg.
        if on_ground && self.secured == Some(Phase::Landing) {
            self.secured = None;
            self.service_open = false;
        }
        None
    }

    /// Seatbelt-sign watch (the annunciator, so auto modes count). The sign
    /// IS the message in a real cabin — the crew only calls back when it
    /// interrupts something: sign ON mid-service earns "securing the
    /// service"; below the sterile floor the state still updates but the
    /// cabin stays quiet (sterile rule — and on approach the sign is
    /// routine, not news).
    pub fn seatbelt(&mut self, on: bool, agl_ft: f64) -> Option<String> {
        let prev = self.seatbelt_on.replace(on);
        let rose = prev == Some(false) && on;
        if rose && self.service_open {
            self.service_open = false;
            if agl_ft > STERILE_FLOOR_AGL_FT {
                return Some("Flight deck, seatbelt sign is on. Securing the service.".to_string());
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t0() -> Instant {
        Instant::now()
    }

    #[test]
    fn address_routes_and_subject_does_not() {
        assert_eq!(
            Cabin::addressed("cabin crew, prepare for landing").as_deref(),
            Some("prepare for landing")
        );
        assert_eq!(
            Cabin::addressed("Cabin, seats for departure").as_deref(),
            Some("seats for departure")
        );
        assert_eq!(
            Cabin::addressed("flight attendants prepare for takeoff").as_deref(),
            Some("prepare for takeoff")
        );
        // "cabin" as a SUBJECT stays with the copilot: systems named "cabin
        // something" are tools and locations, not people (the Lancair's cabin
        // dump switch, cabin lights, cabin pressure).
        assert!(Cabin::addressed("cabin pressure is dropping").is_none());
        assert!(Cabin::addressed("cabin altitude check").is_none());
        assert!(Cabin::addressed("cabin dump on").is_none());
        assert!(Cabin::addressed("cabin lights off").is_none());
        assert!(Cabin::addressed("set flaps ten").is_none());
        // ...but a crew instruction after the bare address still routes.
        assert_eq!(
            Cabin::addressed("cabin, doors to manual").as_deref(),
            Some("doors to manual")
        );
        assert!(Cabin::addressed("cabin secure?").is_some());
    }

    #[test]
    fn prepare_ack_then_secure_report_when_due() {
        let mut c = Cabin::new();
        let now = t0();
        let ack = c.handle("prepare for departure", now);
        assert!(ack.contains("Securing the cabin for departure"), "{ack}");
        // Not due yet: nothing owed.
        assert_eq!(c.tick(now, 0.0, true), None);
        // Due: the secure report comes back once, then goes quiet.
        let later = now + Duration::from_secs(SECURE_SECS_DEPARTURE + 1);
        let report = c.tick(later, 0.0, true).expect("secure report owed");
        assert!(report.contains("cabin is secure for departure"), "{report}");
        assert_eq!(c.tick(later, 0.0, true), None);
    }

    #[test]
    fn landing_secure_report_is_sterile_exempt() {
        let mut c = Cabin::new();
        let now = t0();
        c.handle("cabin crew prepare for landing", now);
        let later = now + Duration::from_secs(SECURE_SECS_LANDING + 1);
        // 4,000 ft AGL on approach — inside sterile, report still comes.
        let report = c.tick(later, 4_000.0, false).expect("safety report exempt");
        assert!(report.contains("secure for landing"), "{report}");
    }

    #[test]
    fn status_reads_reflect_state() {
        let mut c = Cabin::new();
        let now = t0();
        assert!(c.handle("how's the cabin", now).contains("All good"));
        c.handle("prepare for landing", now);
        assert!(c.handle("cabin report", now).contains("Still securing"));
        let later = now + Duration::from_secs(SECURE_SECS_LANDING + 1);
        c.tick(later, 12_000.0, false);
        assert!(c
            .handle("is the cabin secure", later)
            .contains("secure for landing"));
    }

    #[test]
    fn unknown_call_gets_say_again_never_a_guess() {
        let mut c = Cabin::new();
        assert_eq!(
            c.handle("bring me a coffee spanner", t0()),
            "Flight deck, say again?"
        );
    }

    #[test]
    fn service_starts_above_sterile_floor_after_departure() {
        let mut c = Cabin::new();
        let now = t0();
        c.handle("secure for takeoff", now);
        let due = now + Duration::from_secs(SECURE_SECS_DEPARTURE + 1);
        c.tick(due, 0.0, true).expect("secure report");
        // Climbing through 8,000: sterile — quiet.
        assert_eq!(c.tick(due, 8_000.0, false), None);
        // Through the floor: crew released, one line.
        let line = c.tick(due, 11_000.0, false).expect("release line");
        assert!(line.contains("Starting the service"), "{line}");
        assert_eq!(c.tick(due, 12_000.0, false), None);
    }

    #[test]
    fn seatbelt_sign_interrupting_service_calls_up_only_above_floor() {
        let mut c = Cabin::new();
        c.service_open = true;
        // First read establishes state without inventing an edge.
        assert_eq!(c.seatbelt(false, 15_000.0), None);
        // Sign on in cruise mid-service: one call.
        let line = c
            .seatbelt(true, 15_000.0)
            .expect("service interruption call");
        assert!(line.contains("Securing the service"), "{line}");
        // Again below the floor: state updates, cabin stays sterile-quiet.
        let mut c = Cabin::new();
        c.service_open = true;
        assert_eq!(c.seatbelt(false, 5_000.0), None);
        assert_eq!(c.seatbelt(true, 5_000.0), None);
        assert!(!c.service_open, "state still updated silently");
    }

    #[test]
    fn staffing_is_profile_with_panel_override() {
        // Sets process-wide state: same discipline as mode-setting tests.
        let _guard = crate::mode::TEST_LOCK.lock().unwrap();
        set_staffing(Staffing::Auto);
        assert!(!aboard(false)); // a 172 has no cabin crew
        assert!(aboard(true)); // the MD-82 does
        set_staffing(Staffing::Aboard);
        assert!(aboard(false)); // ...unless you staff one for testing
        set_staffing(Staffing::Off);
        assert!(!aboard(true)); // ...or send them home
        set_staffing(Staffing::Auto);
    }

    #[test]
    fn staffing_parses_and_roundtrips() {
        for s in [Staffing::Auto, Staffing::Aboard, Staffing::Off] {
            assert_eq!(Staffing::parse(s.as_str()), Some(s));
            assert_eq!(Staffing::from_panel_index(s.panel_index()), Some(s));
        }
        // Legacy env spellings still land somewhere sensible.
        assert_eq!(Staffing::parse("true"), Some(Staffing::Aboard));
        assert_eq!(Staffing::parse("false"), Some(Staffing::Off));
        assert_eq!(Staffing::parse("banana"), None);
        assert_eq!(Staffing::from_panel_index(-1), None);
    }

    #[test]
    fn doors_sop_flavor() {
        let mut c = Cabin::new();
        assert!(c.handle("doors to automatic", t0()).contains("automatic"));
        assert!(c
            .handle("doors to manual and cross check", t0())
            .contains("manual"));
    }
}
