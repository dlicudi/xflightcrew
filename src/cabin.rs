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
/// Unasked, from the start of the taxi: the demo and the walk-through.
const SECURE_SECS_TAXI: u64 = 60;
/// A taxi is a taxi after this long above `TAXI_KTS` under its own power.
const TAXI_SUSTAIN_SECS: u64 = 10;
const TAXI_KTS: f64 = 5.0;
/// Above this the roll is a takeoff: a report a minute later would come in
/// the climb.
const TAXI_MAX_KTS: f64 = 40.0;
/// A flight that never climbed this high is a circuit: nothing to secure
/// from, no landing preparation of the cabin's own.
const DESCENT_MIN_PEAK_AGL_FT: f64 = 3_000.0;
/// The cabin prepares for landing when the descent has given back this
/// fraction of the flight's peak height (the sterile floor on an airliner's
/// profile, lower on a short hop).
const DESCENT_PEAK_FRACTION: f64 = 0.6;
const DESCENT_FPM: f64 = -300.0;
/// Faster than this at the first tick on the ground is a landing roll.
const LANDING_ROLL_KTS: f64 = 30.0;

/// What the monitor's snapshot gives the cabin each tick.
#[derive(Debug, Clone, Copy)]
pub struct View {
    pub agl_ft: f64,
    pub on_ground: bool,
    pub groundspeed_kts: f64,
    pub vsi_fpm: f64,
    pub engine_running: bool,
    pub pushing_back: bool,
}

/// Something the cabin did unasked: blackboxed by name, spoken when it has
/// a line (the start of a securing is silent; its report is what is heard).
#[derive(Debug, PartialEq)]
pub struct Initiative {
    pub event: &'static str,
    pub line: Option<String>,
}

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
    /// A bare "cabin crew?" was answered "go ahead": the interphone is open
    /// until then, and the NEXT utterance may be theirs without the address.
    line_open_until: Option<Instant>,
    /// Told to sit down ("take your seats"): the service is suspended until
    /// a release, and the status read says so.
    seated: bool,
    /// One-shots per flight: the cabin's OWN departure and landing
    /// preparation. A call from the flight deck sets them too, so the cabin
    /// never secures twice for one event.
    departure_done: bool,
    landing_done: bool,
    /// The flight so far, from the watch: when the taxi began, whether the
    /// aircraft has flown, and how high.
    taxi_since: Option<Instant>,
    flown: bool,
    peak_agl_ft: f64,
    /// The watch has seen this aircraft on the ground (false = the app came
    /// up in the air), and the flight ended in a landing roll, not a
    /// reposition to a ramp.
    seen_ground: bool,
    landed: bool,
    /// The last view, so a call can be answered for where the aircraft is.
    last_on_ground: bool,
}

/// How long the interphone stays open after a bare address.
const OPEN_LINE_SECS: u64 = 20;

impl Cabin {
    pub fn new() -> Self {
        Cabin {
            secured: None,
            pending: None,
            seatbelt_on: None,
            service_open: false,
            line_open_until: None,
            seated: false,
            departure_done: false,
            landing_done: false,
            taxi_since: None,
            flown: false,
            peak_agl_ft: 0.0,
            seen_ground: false,
            landed: false,
            last_on_ground: true,
        }
    }

    /// The cabin's own initiative, every monitor tick BEFORE [`Cabin::tick`]:
    /// a real cabin is not waiting to be asked. It secures during the taxi
    /// and reports, it secures on the way down and reports, and it says when
    /// the doors are disarmed at the stand. Each once per flight, and never
    /// after the flight deck made the same call itself.
    pub fn watch(&mut self, now: Instant, v: View) -> Option<Initiative> {
        let was_on_ground = std::mem::replace(&mut self.last_on_ground, v.on_ground);
        if v.on_ground {
            self.seen_ground = true;
            // Air to ground: a landing arrives rolling. Anything else is the
            // sim putting the aircraft on a ramp, and nobody deplanes from
            // a flight that never landed.
            //
            // ...and the speed at the tick we HAPPENED to see is not the
            // evidence, because the tick can arrive late. The cabin block sat
            // inside the monitor's `quiet` gate, which includes
            // `autoland::ACTIVE`, so on every autoland arrival the cabin was
            // not called from the arm until the rollout handed off: at Liege
            // (2026-09-22) the first ground sample it ever saw was the vacate
            // taxi at 26 kt, under the 30 kt roll test, so it decided the sim
            // had repositioned us to a ramp, forgot the flight it had just
            // flown, read the taxi to the gate as a DEPARTURE taxi and
            // announced "cabin is secure for departure" while taxiing in.
            // A flight that came down through the descent gate
            // (`landing_done`) has landed whatever speed the tick catches; a
            // reposition mid-cruise has no such gate behind it.
            if self.flown && !was_on_ground {
                if v.groundspeed_kts > LANDING_ROLL_KTS || self.landing_done {
                    self.landed = true;
                } else {
                    self.end_flight();
                }
            }
            // Parked with the engines stopped after a landing: the leg is
            // over back there.
            if self.landed && !v.engine_running && v.groundspeed_kts < 1.0 {
                self.end_flight();
                return Some(Initiative {
                    event: "deplaning",
                    line: Some("Flight deck, doors to manual. Passengers are deplaning.".into()),
                });
            }
            let taxiing = !self.flown
                && v.engine_running
                && !v.pushing_back
                && v.groundspeed_kts > TAXI_KTS
                && v.groundspeed_kts < TAXI_MAX_KTS;
            if !taxiing {
                self.taxi_since = None;
                return None;
            }
            let since = *self.taxi_since.get_or_insert(now);
            if !self.departure_done
                && now.duration_since(since) >= Duration::from_secs(TAXI_SUSTAIN_SECS)
            {
                self.departure_done = true;
                self.secured = None;
                self.pending = Some((
                    now + Duration::from_secs(SECURE_SECS_TAXI),
                    Phase::Departure,
                ));
                return Some(Initiative {
                    event: "securing_for_departure",
                    line: None,
                });
            }
            return None;
        }

        // Airborne.
        if !self.flown {
            // A turnaround with the engines kept running never ended the
            // last leg: this liftoff does.
            if self.landing_done {
                self.end_flight();
            }
            self.flown = true;
            // The app came up in the air: above the floor the service is
            // already out, and announcing it now would be inventing news.
            if !self.seen_ground && v.agl_ft > STERILE_FLOOR_AGL_FT {
                self.departure_done = true;
                self.service_open = true;
                self.peak_agl_ft = v.agl_ft;
                return Some(Initiative {
                    event: "joined_in_cruise",
                    line: None,
                });
            }
            // Whatever the cabin was doing on the roll, it is strapped in
            // now; a secure report in the climb is news to nobody. This is
            // also the runway start, which never taxied.
            if self.secured != Some(Phase::Departure) {
                self.pending = None;
                self.secured = Some(Phase::Departure);
                self.departure_done = true;
                return Some(Initiative {
                    event: "secure_at_liftoff",
                    line: None,
                });
            }
        }
        self.peak_agl_ft = self.peak_agl_ft.max(v.agl_ft);
        let gate_ft = STERILE_FLOOR_AGL_FT.min(self.peak_agl_ft * DESCENT_PEAK_FRACTION);
        if !self.landing_done
            && self.peak_agl_ft >= DESCENT_MIN_PEAK_AGL_FT
            && v.agl_ft <= gate_ft
            && v.vsi_fpm < DESCENT_FPM
        {
            self.landing_done = true;
            self.service_open = false;
            self.seated = false;
            self.secured = None;
            self.pending = Some((
                now + Duration::from_secs(SECURE_SECS_LANDING),
                Phase::Landing,
            ));
            return Some(Initiative {
                event: "securing_for_landing",
                line: None,
            });
        }
        None
    }

    fn end_flight(&mut self) {
        self.flown = false;
        self.landed = false;
        self.peak_agl_ft = 0.0;
        self.departure_done = false;
        self.landing_done = false;
        self.taxi_since = None;
        self.secured = None;
        self.pending = None;
        self.service_open = false;
        self.seated = false;
    }

    /// Whether a bare address is still waiting for its call.
    pub fn line_open(&self, now: Instant) -> bool {
        self.line_open_until.is_some_and(|until| now <= until)
    }

    /// The utterance after a bare "cabin crew?", if it is a cabin call. One
    /// shot: the line closes whatever was said, and words the cabin does not
    /// know stay the copilot's ("flaps ten" on an open line is still flaps
    /// ten). A status read needs a cabin word here: its openers ("how",
    /// "ok") are any question's.
    pub fn take_open_line(&mut self, text: &str, now: Instant) -> Option<String> {
        let open = self
            .line_open_until
            .take()
            .is_some_and(|until| now <= until);
        if !open {
            return None;
        }
        let r = text.to_ascii_lowercase();
        let cabin_words = [
            "prepare",
            "secure",
            "seat",
            "stations",
            "doors",
            "board",
            "cabin",
            "report",
            "service",
            "release",
            "minutes",
            "turbulence",
        ];
        if !cabin_words.iter().any(|k| r.contains(k)) {
            return None;
        }
        self.try_handle(text, now)
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
        // The address alone ("Cabin crew?", three of the first seven live
        // calls): answer the interphone and keep the line open.
        if !rest.chars().any(|c| c.is_alphanumeric()) {
            self.line_open_until = Some(now + Duration::from_secs(OPEN_LINE_SECS));
            return "Go ahead, flight deck.".to_string();
        }
        // Heard, not understood: ask again, never guess (a misheard cabin
        // call is a shrug, but the pattern is the same everywhere).
        self.try_handle(rest, now)
            .unwrap_or_else(|| "Flight deck, say again?".to_string())
    }

    /// The reply to a call the cabin understands, `None` for one it does not.
    fn try_handle(&mut self, rest: &str, now: Instant) -> Option<String> {
        let r = rest.to_ascii_lowercase();
        let wants = |kws: &[&str]| kws.iter().any(|k| r.contains(k));

        // "prepare for takeoff" / "secure for departure" / "seats for landing"
        let prepare = wants(&["prepare", "secure", "seats", "ready", "stations"]);
        // "depat": whisper writes "depature".
        if prepare && wants(&["takeoff", "take off", "depart", "depat"]) {
            self.departure_done = true;
            self.service_open = false;
            self.pending = Some((
                now + Duration::from_secs(SECURE_SECS_DEPARTURE),
                Phase::Departure,
            ));
            self.secured = None;
            return Some("Securing the cabin for departure, flight deck.".to_string());
        }
        // "ten minutes to landing" is the same instruction with a clock on it.
        let landing = wants(&["landing", "arrival", "approach"]);
        if landing && (prepare || wants(&["minutes"])) {
            self.landing_done = true;
            self.seated = false;
            self.service_open = false;
            self.pending = Some((
                now + Duration::from_secs(SECURE_SECS_LANDING),
                Phase::Landing,
            ));
            self.secured = None;
            return Some("Securing the cabin for landing, flight deck.".to_string());
        }
        // "twenty minutes to top of descent": heard, nothing to secure yet.
        if wants(&["minutes"]) && wants(&["descent", "descend"]) {
            return Some("Thank you, flight deck. We'll start clearing in.".to_string());
        }
        // Turbulence: "take your seats", "be seated".
        if wants(&["take your seat", "be seated", "turbulence", "sit down"]) {
            self.service_open = false;
            self.seated = true;
            return Some("Taking our seats, flight deck.".to_string());
        }
        // "you're released", "start / resume the service".
        if wants(&["release", "service"]) {
            if self.last_on_ground {
                return Some("We're still on the ground, flight deck.".to_string());
            }
            if let Some((_, p)) = self.pending {
                return Some(format!("We're securing for {}, flight deck.", p.spoken()));
            }
            if self.secured == Some(Phase::Landing) {
                return Some("The cabin is secure for landing, flight deck.".to_string());
            }
            self.seated = false;
            self.secured = None;
            self.service_open = true;
            return Some("Thank you, flight deck. Starting the service.".to_string());
        }
        // Airline SOP flavor: arming/disarming the slides.
        if wants(&["doors"]) {
            if wants(&["automatic", "arm"]) {
                return Some("Doors to automatic, cross checked.".to_string());
            }
            if wants(&["manual", "disarm"]) {
                return Some("Doors to manual, cross checked.".to_string());
            }
        }
        // "cabin crew and passengers boarded?"
        if wants(&["board"]) {
            return Some("Everyone's aboard and seated, flight deck.".to_string());
        }
        // "how's the cabin" / "cabin report" / "is the cabin secure"
        if wants(&["how", "report", "status", "secure", "ready", "okay", "ok"]) {
            return Some(match (self.pending, self.secured) {
                (Some((_, p)), _) => format!(
                    "Still securing for {}, flight deck. A minute more.",
                    p.spoken()
                ),
                (None, _) if self.seated => {
                    "We're seated, flight deck. The service is suspended.".to_string()
                }
                (None, Some(p)) => format!("Cabin is secure for {}.", p.spoken()),
                (None, None) if self.service_open => {
                    "Cabin's good, service is running.".to_string()
                }
                (None, None) => "All good back here, flight deck.".to_string(),
            });
        }
        None
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

    /// The first seven live calls, five of them "say again" (2026-07/08).
    #[test]
    fn live_calls_that_earned_say_again() {
        let mut c = Cabin::new();
        let rest = Cabin::addressed("cabin crew, prepare for depature").unwrap();
        assert!(c.handle(&rest, t0()).contains("for departure"));
        let rest = Cabin::addressed("Cabin crew and passengers boarded.").unwrap();
        assert!(c.handle(&rest, t0()).contains("aboard"));
    }

    #[test]
    fn bare_address_opens_the_line_for_one_cabin_call() {
        let mut c = Cabin::new();
        let now = t0();
        let rest = Cabin::addressed("Cabin crew?").unwrap();
        assert_eq!(c.handle(&rest, now), "Go ahead, flight deck.");
        let reply = c.take_open_line("Prepare for landing.", now + Duration::from_secs(5));
        assert!(reply.unwrap().contains("for landing"));
        // One shot.
        assert!(c
            .take_open_line("Prepare for landing.", now + Duration::from_secs(6))
            .is_none());

        // The copilot's words stay the copilot's, and close the line.
        c.handle("", now);
        assert!(c
            .take_open_line("Is the gear okay?", now + Duration::from_secs(2))
            .is_none());
        assert!(c
            .take_open_line("Prepare for landing.", now + Duration::from_secs(3))
            .is_none());

        // Expired.
        c.handle("?", now);
        assert!(c
            .take_open_line(
                "Prepare for landing.",
                now + Duration::from_secs(OPEN_LINE_SECS + 1)
            )
            .is_none());
    }

    fn view(agl_ft: f64, on_ground: bool, gs: f64, vsi: f64, engine: bool) -> View {
        View {
            agl_ft,
            on_ground,
            groundspeed_kts: gs,
            vsi_fpm: vsi,
            engine_running: engine,
            pushing_back: false,
        }
    }

    /// An autoland arrival, where the cabin is not called for the whole
    /// approach: Liege 2026-09-22. The monitor's `quiet` gate includes
    /// `autoland::ACTIVE`, so from the arm to the rollout handoff the cabin
    /// saw nothing; the first ground sample it ever got was the vacate taxi at
    /// 26 kt, under the 30 kt roll test, so it read the landing as the sim
    /// repositioning us to a ramp, forgot the flight, and announced "cabin is
    /// secure for departure" while taxiing to the gate.
    #[test]
    fn a_landing_seen_late_is_still_a_landing() {
        let mut c = Cabin::new();
        let t = t0();
        let s = Duration::from_secs;
        fn step(c: &mut Cabin, at: Instant, v: View, heard: &mut Vec<String>) {
            if let Some(line) = c.watch(at, v).and_then(|i| i.line) {
                heard.push(line);
            }
            if let Some(line) = c.tick(at, v.agl_ft, v.on_ground) {
                heard.push(line);
            }
        }
        let mut heard: Vec<String> = Vec::new();
        macro_rules! step {
            ($at:expr, $v:expr) => {
                step(&mut c, $at, $v, &mut heard)
            };
        }
        // Taxi out, climb, cruise, and down through the descent gate.
        step!(t, view(0.0, true, 12.0, 0.0, true));
        step!(t + s(11), view(0.0, true, 12.0, 0.0, true));
        step!(t + s(72), view(0.0, true, 0.0, 0.0, true));
        step!(t + s(200), view(500.0, false, 160.0, 2000.0, true));
        step!(t + s(900), view(33_000.0, false, 450.0, 0.0, true));
        step!(t + s(2000), view(12_000.0, false, 300.0, -1800.0, true));
        step!(t + s(2100), view(8_000.0, false, 250.0, -1500.0, true));
        heard.clear();
        // ...and now the gap. Nothing from the autoland arm through the flare
        // and the whole rollout; the next sample is the vacate taxi.
        step!(t + s(2500), view(0.0, true, 26.0, 0.0, true));
        step!(t + s(2512), view(0.0, true, 12.0, 0.0, true));
        step!(t + s(2575), view(0.0, true, 12.0, 0.0, true));
        assert!(
            !heard.iter().any(|l| l.contains("secure for departure")),
            "a taxi to the gate is not a departure taxi: {heard:?}"
        );
        // The leg still ends properly at the stand.
        step!(t + s(2900), view(0.0, true, 0.0, 0.0, false));
        assert!(
            heard.iter().any(|l| l.contains("deplaning")),
            "and the arrival is still an arrival: {heard:?}"
        );
    }

    /// A whole airliner leg with nobody calling the cabin: secure on the
    /// taxi, the service through the floor, secure on the way down, the
    /// doors at the stand. Then the next leg does it again.
    #[test]
    fn an_uncalled_cabin_works_the_whole_flight() {
        let mut c = Cabin::new();
        let t = t0();
        let s = Duration::from_secs;
        let mut heard: Vec<String> = Vec::new();
        let mut step = |c: &mut Cabin, at: Instant, v: View| {
            if let Some(line) = c.watch(at, v).and_then(|i| i.line) {
                heard.push(line);
            }
            if let Some(line) = c.tick(at, v.agl_ft, v.on_ground) {
                heard.push(line);
            }
        };
        // Parked, then a push under the tug, then the taxi.
        step(&mut c, t, view(0.0, true, 0.0, 0.0, true));
        step(
            &mut c,
            t + s(5),
            View {
                pushing_back: true,
                ..view(0.0, true, 6.0, 0.0, true)
            },
        );
        step(&mut c, t + s(10), view(0.0, true, 12.0, 0.0, true));
        step(&mut c, t + s(21), view(0.0, true, 12.0, 0.0, true));
        step(&mut c, t + s(82), view(0.0, true, 0.0, 0.0, true));
        // Up through the floor, a cruise, then down through it.
        step(&mut c, t + s(200), view(500.0, false, 160.0, 2000.0, true));
        step(
            &mut c,
            t + s(600),
            view(10_500.0, false, 300.0, 1500.0, true),
        );
        step(&mut c, t + s(900), view(33_000.0, false, 450.0, 0.0, true));
        step(
            &mut c,
            t + s(2000),
            view(12_000.0, false, 300.0, -1800.0, true),
        );
        step(
            &mut c,
            t + s(2100),
            view(9_900.0, false, 280.0, -1500.0, true),
        );
        step(
            &mut c,
            t + s(2176),
            view(8_000.0, false, 250.0, -1000.0, true),
        );
        // Landed, taxied in, shut down.
        step(&mut c, t + s(2500), view(0.0, true, 130.0, 0.0, true));
        step(&mut c, t + s(2600), view(0.0, true, 20.0, 0.0, true));
        step(&mut c, t + s(2900), view(0.0, true, 0.0, 0.0, false));
        // The next leg's taxi reports again.
        step(&mut c, t + s(4000), view(0.0, true, 10.0, 0.0, true));
        step(&mut c, t + s(4011), view(0.0, true, 10.0, 0.0, true));
        step(&mut c, t + s(4072), view(0.0, true, 10.0, 0.0, true));
        assert_eq!(
            heard,
            [
                "Flight deck, cabin is secure for departure.",
                "Flight deck, cabin crew released. Starting the service.",
                "Flight deck, cabin is secure for landing.",
                "Flight deck, doors to manual. Passengers are deplaning.",
                "Flight deck, cabin is secure for departure.",
            ]
        );
    }

    #[test]
    fn a_runway_start_is_secure_at_liftoff_and_a_circuit_secures_nothing() {
        let mut c = Cabin::new();
        let t = t0();
        // The takeoff roll is not a taxi.
        assert!(c.watch(t, view(0.0, true, 60.0, 0.0, true)).is_none());
        let up = c.watch(
            t + Duration::from_secs(30),
            view(200.0, false, 150.0, 1800.0, true),
        );
        assert_eq!(up.unwrap().event, "secure_at_liftoff");
        assert!(c.tick(t + Duration::from_secs(31), 200.0, false).is_none());
        // Pattern altitude and back down: no landing preparation.
        c.watch(
            t + Duration::from_secs(90),
            view(1_500.0, false, 180.0, 0.0, true),
        );
        assert!(c
            .watch(
                t + Duration::from_secs(200),
                view(600.0, false, 140.0, -700.0, true)
            )
            .is_none());
    }

    #[test]
    fn joined_in_cruise_or_put_on_a_ramp_invents_no_news() {
        let mut c = Cabin::new();
        let t = t0();
        // xvl started at cruise: no "starting the service", but the status
        // read knows it is out, and the descent still secures.
        let joined = c.watch(t, view(30_000.0, false, 450.0, 0.0, true)).unwrap();
        assert_eq!(joined.event, "joined_in_cruise");
        assert!(c.tick(t, 30_000.0, false).is_none());
        assert!(c.handle("cabin report", t).contains("service is running"));
        // The sim puts the aircraft on a ramp, cold: nobody deplanes.
        assert!(c
            .watch(t + Duration::from_secs(5), view(0.0, true, 0.0, 0.0, false))
            .is_none());
        assert!(c
            .watch(t + Duration::from_secs(6), view(0.0, true, 0.0, 0.0, false))
            .is_none());
    }

    #[test]
    fn the_flight_decks_own_call_is_not_repeated_by_the_cabin() {
        let mut c = Cabin::new();
        let t = t0();
        c.watch(t, view(33_000.0, false, 450.0, 0.0, true));
        assert!(c
            .handle("ten minutes to landing", t)
            .contains("for landing"));
        assert!(c
            .watch(
                t + Duration::from_secs(5),
                view(9_000.0, false, 280.0, -1500.0, true)
            )
            .is_none());
    }

    #[test]
    fn seats_for_turbulence_then_released() {
        let mut c = Cabin::new();
        let t = t0();
        assert!(c
            .handle("you're released", t)
            .contains("still on the ground"));
        c.watch(t, view(20_000.0, false, 400.0, 0.0, true));
        assert!(c
            .handle("take your seats, turbulence ahead", t)
            .contains("Taking our seats"));
        assert!(c.handle("how's the cabin", t).contains("suspended"));
        assert!(c
            .handle("you're released", t)
            .contains("Starting the service"));
        assert!(c.handle("cabin report", t).contains("service is running"));
        assert!(c
            .handle("twenty minutes to top of descent", t)
            .contains("clearing in"));
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
