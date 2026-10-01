//! The flight, as the crew understands it: ONE model with memory, fed once
//! per monitor tick, read by everyone.
//!
//! Before this, "where are we in the flight" was answered four times over:
//! `monitor::estimate_phase` from a single snapshot with no memory (a cruise
//! losing 400 fpm in a gust at 2,500 ft was an approach), and the checklist
//! tracker, the cabin and rapport each keeping their own peak height and
//! their own airborne and landed flags. Every one of them has had a bug of
//! its own: the climb list fired on a single tick, and the cabin missed its
//! own landing (EBLG 2026-09-22).
//!
//! Four independent tracks, statechart style:
//!
//! - [`Phase`]: the flight's progress, every edge sustained (a gust is not a
//!   descent) and legal (a cruise does not become a taxi; [`legal`] is the
//!   whole table and [`Flight::go`] refuses anything else),
//! - [`Control`]: who is flying,
//! - [`Abnormal`]: normal, a failure, an emergency, a wreck,
//! - [`Memory`]: what has happened this flight,
//!
//! and the crew's side of the situation, carried as evidence in
//! [`Situation`] and tracked the same way (a change is an [`Event`]): the
//! mission armed, the plan loaded and the fix being flown, the controller's
//! clearance, and the moment the pilot last took the aircraft back.
//!
//! Pure: no I/O, no clock of its own. Time is the sample's `t` (seconds), so
//! a flight recorded in `logs/blackbox.jsonl` replays exactly as it flew
//! ([`Sample::from_state_record`]), and the tests replay real sorties.
//!
//! Evidence beats inference: a field the caller cannot know is `None`, and a
//! phase that needs it (an engine start without the starter, a line-up
//! without the runway) is simply never entered from inference alone.

use std::collections::VecDeque;

/// The flight's progress. The order is the normal order of a flight.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Phase {
    /// On the ground, stopped, engines off.
    Parked,
    /// A starter is turning (evidence only).
    EngineStart,
    /// Engines running, not yet moving, no flight flown since the start.
    Ready,
    /// A tug on the bar (evidence only).
    Pushback,
    /// Moving on the ground towards a departure.
    TaxiOut,
    /// Stopped during the taxi out: a hold-short, a queue, a runup.
    Holding,
    /// Stopped on a runway before the roll (evidence only).
    LinedUp,
    TakeoffRoll,
    RejectedTakeoff,
    /// Airborne from a takeoff, under [`INITIAL_CLIMB_AGL_FT`].
    InitialClimb,
    Climb,
    Cruise,
    Descent,
    Approach,
    /// The last [`LANDING_AGL_FT`] of an approach: the flare.
    Landing,
    GoAround,
    /// Airborne and slow: a helicopter's hover.
    Hover,
    /// On the ground after a landing, still fast.
    Rollout,
    /// On the ground after a landing, taxi speed or stopped, engines running.
    TaxiIn,
}

impl Phase {
    pub fn airborne(self) -> bool {
        matches!(
            self,
            Phase::InitialClimb
                | Phase::Climb
                | Phase::Cruise
                | Phase::Descent
                | Phase::Approach
                | Phase::Landing
                | Phase::GoAround
                | Phase::Hover
        )
    }

    /// Before the takeoff: the ground phases a departure passes through.
    pub fn departing(self) -> bool {
        matches!(
            self,
            Phase::Ready
                | Phase::EngineStart
                | Phase::Pushback
                | Phase::TaxiOut
                | Phase::Holding
                | Phase::LinedUp
                | Phase::TakeoffRoll
                | Phase::RejectedTakeoff
        )
    }

    /// The words a crew uses for it.
    pub fn spoken(self) -> &'static str {
        match self {
            Phase::Parked => "parked",
            Phase::EngineStart => "starting the engines",
            Phase::Ready => "ready to taxi",
            Phase::Pushback => "pushing back",
            Phase::TaxiOut => "taxiing out",
            Phase::Holding => "holding on the taxi out",
            Phase::LinedUp => "lined up",
            Phase::TakeoffRoll => "on the takeoff roll",
            Phase::RejectedTakeoff => "rejecting the takeoff",
            Phase::InitialClimb => "in the initial climb",
            Phase::Climb => "climbing",
            Phase::Cruise => "in the cruise",
            Phase::Descent => "descending",
            Phase::Approach => "on the approach",
            Phase::Landing => "landing",
            Phase::GoAround => "going around",
            Phase::Hover => "in the hover",
            Phase::Rollout => "on the landing roll",
            Phase::TaxiIn => "taxiing in",
        }
    }
}

/// Every edge the phase track may take. [`Flight::go`] refuses the rest, so
/// a new rule that produces a nonsense edge fails its test instead of
/// flying. A reposition is not an edge: it resets the track
/// ([`Event::Reposition`]).
pub fn legal(from: Phase, to: Phase) -> bool {
    use Phase::*;
    if from == to {
        return false;
    }
    // Any airborne phase can meet the ground (a forced landing is a landing)
    // and any can slow to a hover.
    if from.airborne() && matches!(to, Rollout | TaxiIn | Hover) {
        return true;
    }
    // A helicopter lifts from wherever it stands.
    if matches!(from, Ready | Pushback | TaxiOut | Holding | LinedUp) && to == Hover {
        return true;
    }
    match from {
        Parked => matches!(to, EngineStart | Ready),
        EngineStart => matches!(to, Ready | Parked),
        Ready => {
            matches!(
                to,
                Parked | EngineStart | Pushback | TaxiOut | Holding | TakeoffRoll
            ) || matches!(to, InitialClimb)
        }
        Pushback => matches!(to, Ready | TaxiOut | Holding | Parked),
        TaxiOut => matches!(to, Holding | LinedUp | TakeoffRoll | Parked | InitialClimb),
        Holding => matches!(to, TaxiOut | LinedUp | TakeoffRoll | Parked | InitialClimb),
        LinedUp => matches!(to, TaxiOut | Holding | TakeoffRoll | Parked | InitialClimb),
        TakeoffRoll => matches!(to, RejectedTakeoff | InitialClimb),
        RejectedTakeoff => matches!(to, TaxiOut | Holding | Parked | TakeoffRoll),
        InitialClimb => matches!(to, Climb | Cruise | Descent | Approach),
        Climb => matches!(to, Cruise | Descent | Approach),
        Cruise => matches!(to, Climb | Descent | Approach),
        Descent => matches!(to, Climb | Cruise | Approach),
        Approach => matches!(to, Landing | GoAround | Cruise),
        Landing => matches!(to, Approach | GoAround),
        GoAround => matches!(to, Climb | Cruise | Approach),
        Hover => matches!(to, InitialClimb | Climb | Cruise | Descent | Approach),
        Rollout => matches!(to, TaxiIn | InitialClimb),
        TaxiIn => matches!(to, Parked | TakeoffRoll | InitialClimb | Hover),
    }
}

/// Who is flying.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Control {
    Unknown,
    Pilot,
    Autopilot,
    /// One of our loops, by name ("full auto", "autoland", ...).
    Automation(&'static str),
}

/// The abnormal track, orthogonal to the phase: an engine can fail in any
/// phase and the phase goes on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Abnormal {
    Normal,
    /// Systems failed now, by spoken label.
    Failure(Vec<String>),
    /// Declared: a mayday or pan.
    Emergency,
    Crashed,
}

/// The flight plan on the shelf, as the situation needs it (the fix being
/// flown is [`Situation::active_fix`], so a new leg is not a new plan).
#[derive(Debug, Clone, PartialEq)]
pub struct PlanFact {
    /// "simbrief" or "local".
    pub source: &'static str,
    pub origin: String,
    pub destination: String,
    pub cruise_alt_ft: f64,
}

/// The controller's standing clearance for us (the party line's).
#[derive(Debug, Clone, PartialEq)]
pub struct ClearanceFact {
    /// The tower's state for us: "TaxiCleared", "TakeoffCleared", ...
    pub state: String,
    pub runway: Option<String>,
    pub squawk: Option<i64>,
    /// A readback the controller is still waiting for.
    pub readback_owed: bool,
}

/// The crew's side of one tick: what we are doing and what we hold. The
/// live tick knows all of it; a replayed flight knows none of it
/// (`Sample::situation` is `None`), and the tracks then stand as they are.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Situation {
    /// The outermost mission armed, by name ("full auto", "circuit",
    /// "lesson", ...); `None` = the pilot is flying their own flight.
    pub mission: Option<&'static str>,
    pub plan: Option<PlanFact>,
    /// The fix the follower is steering at.
    pub active_fix: Option<String>,
    pub clearance: Option<ClearanceFact>,
    /// When the pilot last took the aircraft back from a loop, and from
    /// which ([`note_pilot_takeover`]).
    pub pilot_takeover: Option<(f64, &'static str)>,
    /// A simulated tower reaches us (`party_line::station_reaches`, ATC on).
    pub towered: Option<bool>,
    /// The taxi drive ARRIVED at its departure holding point (a stop short
    /// of it is not the hold). `None` when unknown.
    pub at_hold: Option<bool>,
}

/// The one piece of this module that is not the pure model: the mailbox a
/// stand-down caused by the PILOT writes (a "my controls", feet on the
/// pedals, hands on the yoke), read into [`Situation::pilot_takeover`].
static PILOT_TAKEOVER: std::sync::Mutex<Option<(f64, &'static str)>> = std::sync::Mutex::new(None);

/// The pilot took the aircraft back from `what`.
pub fn note_pilot_takeover(what: &'static str) {
    let t = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0);
    *PILOT_TAKEOVER.lock().unwrap_or_else(|e| e.into_inner()) = Some((t, what));
    // An unattended sortie is only unattended if every takeover is on file.
    sim::blackbox::record("intervention", serde_json::json!({ "what": what }));
}

/// The pilot handed the aircraft TO the crew ("you have control",
/// `handover::decide`): any standing pilot takeover is over, so the duty
/// engine stops reading "the pilot has it".
pub fn note_crew_takeover(why: &'static str) {
    *PILOT_TAKEOVER.lock().unwrap_or_else(|e| e.into_inner()) = None;
    sim::blackbox::record("handover", serde_json::json!({ "to": "crew", "why": why }));
}

pub fn pilot_takeover() -> Option<(f64, &'static str)> {
    *PILOT_TAKEOVER.lock().unwrap_or_else(|e| e.into_inner())
}

/// One tick of the aircraft. Mirrors the blackbox `state` record, plus
/// evidence only the live tick has (`None` = unknown, never guessed).
#[derive(Debug, Clone, Default)]
pub struct Sample {
    pub t: f64,
    pub on_ground: bool,
    pub gs_kts: f64,
    pub ias_kts: f64,
    pub vsi_fpm: f64,
    pub agl_ft: f64,
    pub alt_ft: f64,
    pub engine_running: bool,
    pub fuel_kg: Option<f64>,
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    // --- evidence (live only) ---
    /// A starter is engaged on any engine.
    pub starter: Option<bool>,
    /// A pushback tug is attached.
    pub tug_attached: Option<bool>,
    /// On runway pavement.
    pub on_runway: Option<bool>,
    /// An approach is being flown by an automation (autoland armed, the
    /// follower's approach legs).
    pub approach: Option<bool>,
    /// A go-around was commanded (the latch, the tower's order).
    pub go_around: Option<bool>,
    /// The automation flying, by name.
    pub automation: Option<&'static str>,
    /// The autopilot's servos are engaged.
    pub autopilot: Option<bool>,
    /// Failed systems, by spoken label.
    pub failed: Option<Vec<String>>,
    pub emergency: Option<bool>,
    pub crashed: Option<bool>,
    /// The crew's side (`None` = unknown: a replayed flight).
    pub situation: Option<Situation>,
}

impl Sample {
    /// A blackbox `state` record, or `None` when it is not one.
    pub fn from_state_record(v: &serde_json::Value) -> Option<Sample> {
        if v.get("kind")?.as_str()? != "state" {
            return None;
        }
        let f = |k: &str| v.get(k).and_then(|x| x.as_f64());
        Some(Sample {
            t: f("t")?,
            on_ground: v.get("ground")?.as_bool()?,
            gs_kts: f("gs").unwrap_or(0.0),
            ias_kts: f("ias").unwrap_or(0.0),
            // The flight model's vertical speed where the record has it,
            // the lagging gauge in records from before #198.
            vsi_fpm: f("vs").or(f("vsi")).unwrap_or(0.0),
            agl_ft: f("agl").unwrap_or(0.0),
            alt_ft: f("alt").unwrap_or(0.0),
            engine_running: v.get("eng").and_then(|x| x.as_bool()).unwrap_or(true),
            fuel_kg: f("fuel_kg"),
            lat: f("lat"),
            lon: f("lon"),
            crashed: v.get("crashed").and_then(|x| x.as_bool()),
            ..Sample::default()
        })
    }
}

/// What happened this flight.
#[derive(Debug, Clone, Default)]
pub struct Memory {
    /// Flights begun since the app started (a takeoff roll, or a join).
    pub flight_no: u32,
    /// The flight was joined in the air (app started or repositioned
    /// airborne): its departure is unknown, not "none".
    pub joined_airborne: bool,
    pub origin: Option<(f64, f64)>,
    pub takeoff_t: Option<f64>,
    /// When this flight last left the ground, or was first seen in the air:
    /// the takeoff, a touch and go, or the join. The time aloft a landing
    /// ends (rapport's "an hour in the air").
    pub aloft_t: Option<f64>,
    pub fuel_at_takeoff_kg: Option<f64>,
    pub peak_alt_ft: f64,
    pub peak_agl_ft: f64,
    pub landings: u32,
    pub touch_and_goes: u32,
    pub go_arounds: u32,
    pub bounces: u32,
    pub rejected_takeoffs: u32,
    /// Ground met climbing or past [`IMPACT_SINK_FPM`]: never a landing.
    pub impacts: u32,
    /// Times the pilot took the aircraft back from one of our loops.
    pub pilot_takeovers: u32,
    pub last_touchdown: Option<Touchdown>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Touchdown {
    pub t: f64,
    /// The steepest sink at the wheels (`instructor::touchdown_sink_fpm`).
    pub sink_fpm: f64,
    pub gs_kts: f64,
    pub lat: Option<f64>,
    pub lon: Option<f64>,
}

/// What one tick changed.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    Phase {
        from: Phase,
        to: Phase,
        why: &'static str,
    },
    /// The sim moved the aircraft (a reposition, a new flight loaded): the
    /// phase track restarted from what the aircraft is doing now.
    Reposition {
        to: Phase,
    },
    Takeoff,
    Touchdown(Touchdown),
    /// The ground met in a way no landing meets it: climbing into it, or
    /// sinking past [`IMPACT_SINK_FPM`]. The phase holds until the wreck is
    /// at rest.
    Impact(Touchdown),
    Bounce,
    TouchAndGo,
    GoAround,
    RejectedTakeoff,
    /// Stopped with the engines off after a landing: the flight is over.
    FlightComplete,
    Control {
        from: Control,
        to: Control,
    },
    Abnormal {
        from: Abnormal,
        to: Abnormal,
    },
    Mission {
        from: Option<&'static str>,
        to: Option<&'static str>,
    },
    /// A plan loaded, replaced or taken off the shelf.
    Plan {
        from: Option<PlanFact>,
        to: Option<PlanFact>,
    },
    ActiveFix {
        from: Option<String>,
        to: Option<String>,
    },
    Clearance {
        from: Option<ClearanceFact>,
        to: Option<ClearanceFact>,
    },
    /// The pilot took the aircraft back from `what`, while `control` held it.
    PilotTookOver {
        what: &'static str,
        control: Control,
    },
}

// --- the rules' numbers ---

/// A takeoff roll is under way past this groundspeed (the monitor's and the
/// checklist tracker's figure).
pub const ROLL_KTS: f64 = 35.0;
const ROLL_SECS: f64 = 2.0;
/// A roll that has given back this much of its best speed is a reject.
const REJECT_DROP_KTS: f64 = 10.0;
const REJECT_SECS: f64 = 2.0;
/// Moving on the ground.
const TAXI_KTS: f64 = 3.0;
const TAXI_SECS: f64 = 3.0;
const STOPPED_KTS: f64 = 1.0;
const HOLDING_SECS: f64 = 5.0;
/// Airborne once the wheels have been off this long, or this high.
const AIRBORNE_SECS: f64 = 1.5;
const AIRBORNE_AGL_FT: f64 = 50.0;
pub const INITIAL_CLIMB_AGL_FT: f64 = 1000.0;
/// The vertical classifier: climbing, level and descending are each a
/// sustained state (the checklist tracker's figures).
const VS_BAND_FPM: f64 = 300.0;
const CLIMB_SECS: f64 = 15.0;
const LEVEL_SECS: f64 = 30.0;
const DESCENT_SECS: f64 = 20.0;
/// ...or the height LOST over the last [`DESCENT_SECS`] is a descent past the
/// band, whatever the needle did inside the window: an idle descent that
/// protects its speed saws between -2,000 and level every 15 s, and each
/// level tick restarted the held clock (EGKK-EGBB 2026-09-28, #174: Climb
/// from the 16,300 ft top to 12,700, then Cruise to 9,345). The window's
/// oldest sample may be at most this old (a gap in the record is no trend).
const TREND_MAX_SECS: f64 = 2.0 * DESCENT_SECS;
/// A level this far under the peak, in a descent, is a step of the descent
/// until it has held this long.
const STEP_BELOW_PEAK_FT: f64 = 1000.0;
const STEP_MAX_SECS: f64 = 600.0;
/// Approach: descending under this height, held.
pub const APPROACH_AGL_FT: f64 = 1500.0;
const APPROACH_VS_FPM: f64 = -100.0;
const APPROACH_SECS: f64 = 10.0;
/// An approach levelled this high this long was not an approach.
const APPROACH_EXIT_AGL_FT: f64 = 2500.0;
const APPROACH_EXIT_SECS: f64 = 60.0;
/// The flare.
pub const LANDING_AGL_FT: f64 = 100.0;
const LANDING_EXIT_AGL_FT: f64 = 200.0;
/// A go-around: climbing this hard this long out of an approach.
const GO_AROUND_VS_FPM: f64 = 500.0;
const GO_AROUND_SECS: f64 = 4.0;
/// ...and it is over this far above its lowest point, or level this long.
const GO_AROUND_DONE_FT: f64 = 500.0;
const GO_AROUND_LEVEL_SECS: f64 = 20.0;
/// The rollout is over under this groundspeed, held (the after-landing
/// list's figure).
const ROLLOUT_END_KTS: f64 = 30.0;
const ROLLOUT_END_SECS: f64 = 3.0;
/// A touch and go: airborne this long and climbing, or this high.
const TOUCH_AND_GO_SECS: f64 = 8.0;
const TOUCH_AND_GO_AGL_FT: f64 = 150.0;
/// Parked: engines off and stopped, held.
const PARKED_SECS: f64 = 5.0;
/// A hover: airborne, this slow over the ground and through the air, held.
const HOVER_GS_KTS: f64 = 15.0;
const HOVER_IAS_KTS: f64 = 25.0;
const HOVER_SECS: f64 = 5.0;
const HOVER_EXIT_KTS: f64 = 25.0;
const HOVER_EXIT_SECS: f64 = 3.0;
/// A move no aircraft makes between two ticks is the sim moving it.
const REPOSITION_KTS: f64 = 1200.0;
const REPOSITION_MIN_NM: f64 = 1.0;
const REPOSITION_FPM: f64 = 30_000.0;
const REPOSITION_ALT_FT: f64 = 1000.0;
const REPOSITION_AGL_FT: f64 = 300.0;
/// On the ground at a standstill this soon after being placed in the air is
/// the sim settling the aircraft, not a landing.
const SETTLE_SECS: f64 = 15.0;
const LOADING_AGL_FT: f64 = -50.0;
/// Airborne samples kept for the touchdown sink.
const RECENT_SECS: f64 = 5.0;
/// A sink at the wheels past this is an impact, not a landing (the hard
/// landing rapport says aloud is 600).
pub const IMPACT_SINK_FPM: f64 = 1500.0;
/// A wreck stopped this long is at rest: the phase track resumes.
const WRECK_REST_SECS: f64 = 10.0;

/// A condition held since when; `None` while it is false.
#[derive(Debug, Clone, Copy, Default)]
struct Held(Option<f64>);

impl Held {
    /// Feed the condition at `t`; true once it has held `secs`.
    fn feed(&mut self, on: bool, t: f64, secs: f64) -> bool {
        if !on {
            self.0 = None;
            return false;
        }
        t - *self.0.get_or_insert(t) >= secs
    }
}

#[derive(Debug, Clone, Default)]
struct Timers {
    roll: Held,
    reject: Held,
    taxi: Held,
    stopped: Held,
    parked: Held,
    airborne: Held,
    climbing: Held,
    level: Held,
    descending: Held,
    approach: Held,
    approach_exit: Held,
    go_around: Held,
    go_around_level: Held,
    rollout_end: Held,
    hover: Held,
    hover_exit: Held,
    wreck_rest: Held,
}

/// The model. One per monitor loop; [`Flight::tick`] once per sample.
#[derive(Debug, Clone)]
pub struct Flight {
    phase: Phase,
    since_t: f64,
    control: Control,
    abnormal: Abnormal,
    memory: Memory,
    timers: Timers,
    last: Option<Sample>,
    started: bool,
    /// When the track last restarted (the start, a reposition).
    restarted_t: f64,
    /// The best groundspeed of the current takeoff roll.
    roll_peak_kts: f64,
    /// The lowest height of the current go-around.
    go_around_floor_ft: f64,
    /// Airborne `(t, vsi)` for the touchdown sink.
    recent: VecDeque<(f64, f64)>,
    /// Airborne `(t, alt)` over the descent window ([`Flight::losing_height`]).
    alt_track: VecDeque<(f64, f64)>,
    /// Off the wheels again during the rollout: a bounce if it comes down.
    off_again: bool,
    situation: Situation,
    /// A situation has been fed at least once.
    situation_seen: bool,
    /// A wreck: the sim's crash flag, or an impact. Nothing it does is a
    /// phase of flight (a wreck sliding at 44 kt is not a takeoff roll).
    wreck: bool,
    /// An illegal edge a rule asked for (a bug; tests assert none).
    refused: Vec<(Phase, Phase, &'static str)>,
}

impl Default for Flight {
    fn default() -> Self {
        Flight::new()
    }
}

impl Flight {
    pub fn new() -> Flight {
        Flight {
            phase: Phase::Parked,
            since_t: 0.0,
            control: Control::Unknown,
            abnormal: Abnormal::Normal,
            memory: Memory::default(),
            timers: Timers::default(),
            last: None,
            started: false,
            restarted_t: f64::MIN,
            roll_peak_kts: 0.0,
            go_around_floor_ft: f64::MAX,
            recent: VecDeque::new(),
            alt_track: VecDeque::new(),
            off_again: false,
            situation: Situation::default(),
            situation_seen: false,
            wreck: false,
            refused: Vec::new(),
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }
    /// Seconds in the current phase at sample time `t`.
    pub fn in_phase_secs(&self, t: f64) -> f64 {
        (t - self.since_t).max(0.0)
    }
    pub fn control(&self) -> Control {
        self.control
    }
    pub fn abnormal(&self) -> &Abnormal {
        &self.abnormal
    }
    pub fn memory(&self) -> &Memory {
        &self.memory
    }
    /// The crew's side as last known.
    pub fn situation(&self) -> &Situation {
        &self.situation
    }

    /// Edges a rule asked for and [`legal`] refused. Empty unless a rule is
    /// wrong.
    pub fn refused(&self) -> &[(Phase, Phase, &'static str)] {
        &self.refused
    }

    /// Feed one sample; everything it changed.
    pub fn tick(&mut self, s: &Sample) -> Vec<Event> {
        let mut ev = Vec::new();
        if loading(s) {
            // No evidence at all: the track waits for a real sample.
            return ev;
        }
        if !self.started || self.repositioned(s) {
            self.restart(s, &mut ev);
        } else if !self.wrecked(s, &mut ev) {
            self.step_phase(s, &mut ev);
        }
        self.remember(s);
        // The takeover is judged against who held the aircraft BEFORE this
        // tick's control change (the loop is already gone when it is seen).
        let held_by = self.control;
        self.step_control(s, &mut ev);
        self.step_abnormal(s, &mut ev);
        if let Some(now) = &s.situation {
            self.step_situation(now, held_by, &mut ev);
        }
        self.last = Some(s.clone());
        ev
    }

    /// Take an edge if it is legal; a refused edge is kept for the tests.
    fn go(&mut self, to: Phase, why: &'static str, t: f64, ev: &mut Vec<Event>) -> bool {
        if !legal(self.phase, to) {
            self.refused.push((self.phase, to, why));
            return false;
        }
        ev.push(Event::Phase {
            from: self.phase,
            to,
            why,
        });
        self.phase = to;
        self.since_t = t;
        // A new phase starts every sustained clock afresh: a condition
        // held in the last phase is no evidence in this one.
        self.timers = Timers::default();
        true
    }

    /// Whether the phase is held for a wreck. The sim's flag holds it for
    /// as long as it is raised (a reset clears it, usually with a
    /// reposition); an impact holds it until the wreck is at rest.
    fn wrecked(&mut self, s: &Sample, ev: &mut Vec<Event>) -> bool {
        if s.crashed == Some(true) {
            // The flag can rise a tick before the wreck is on the ground
            // (LILB 2026-09-24); meeting it then is still the impact.
            if self.phase.airborne() && s.on_ground {
                let td = self.contact(s);
                self.impact(s, td, "crashed", ev);
            }
            self.wreck = true;
            return true;
        }
        if !self.wreck {
            return false;
        }
        let at_rest = self.timers.wreck_rest.feed(
            s.on_ground && s.gs_kts < STOPPED_KTS,
            s.t,
            WRECK_REST_SECS,
        );
        if at_rest {
            self.wreck = false;
            self.timers = Timers::default();
        }
        !at_rest
    }

    fn repositioned(&self, s: &Sample) -> bool {
        let Some(p) = &self.last else { return false };
        let dt = (s.t - p.t).max(0.1);
        // Height no aircraft gains between two ticks: the sim loading a
        // flight in the air where it stood (AGL -2,026 ft on the ground,
        // 2,150 ft airborne the next second, in the recorded flights).
        let dalt = (s.alt_ft - p.alt_ft).abs();
        let jumped = dalt > REPOSITION_ALT_FT && dalt / dt * 60.0 > REPOSITION_FPM
            || (p.on_ground != s.on_ground && (s.agl_ft - p.agl_ft).abs() > REPOSITION_AGL_FT);
        if jumped {
            return true;
        }
        let (Some(a), Some(b), Some(c), Some(d)) = (p.lat, p.lon, s.lat, s.lon) else {
            return false;
        };
        let nm = distance_nm(a, b, c, d);
        nm > REPOSITION_MIN_NM && nm / dt * 3600.0 > REPOSITION_KTS
    }

    /// Start (or restart) from what the aircraft is doing now.
    fn restart(&mut self, s: &Sample, ev: &mut Vec<Event>) {
        let to = infer(s);
        let first = !self.started;
        self.started = true;
        self.restarted_t = s.t;
        self.phase = to;
        self.since_t = s.t;
        self.timers = Timers::default();
        self.recent.clear();
        self.alt_track.clear();
        self.roll_peak_kts = 0.0;
        self.go_around_floor_ft = f64::MAX;
        self.off_again = false;
        self.wreck = false;
        self.memory = Memory {
            flight_no: self.memory.flight_no + u32::from(to.airborne()),
            joined_airborne: to.airborne(),
            aloft_t: to.airborne().then_some(s.t),
            ..Memory::default()
        };
        if !first {
            ev.push(Event::Reposition { to });
        }
    }

    fn step_phase(&mut self, s: &Sample, ev: &mut Vec<Event>) {
        use Phase::*;
        let t = s.t;
        let airborne_now = !s.on_ground;
        let airborne = self.timers.airborne.feed(airborne_now, t, AIRBORNE_SECS)
            || (airborne_now && s.agl_ft > AIRBORNE_AGL_FT);
        let moving = self.timers.taxi.feed(s.gs_kts > TAXI_KTS, t, TAXI_SECS);
        let stopped = self
            .timers
            .stopped
            .feed(s.gs_kts < STOPPED_KTS, t, HOLDING_SECS);
        let rolling = self.timers.roll.feed(s.gs_kts > ROLL_KTS, t, ROLL_SECS);
        // Moving under its own power is running, whatever the flag says (it
        // reads one engine; a jet rolled at 184 kt with it off).
        let own_power = moving && s.tug_attached != Some(true);
        let running = s.engine_running || own_power;

        match self.phase {
            Parked | EngineStart | Ready | Pushback | TaxiOut | Holding | LinedUp => {
                if airborne && matches!(self.phase, Parked | EngineStart) {
                    // Airborne with nothing running: the sim put it there.
                    self.restart(s, ev);
                    return;
                }
                if airborne {
                    // Off the ground without a roll: a helicopter's lift,
                    // or a roll between two ticks.
                    let to = if s.gs_kts < HOVER_GS_KTS {
                        Hover
                    } else {
                        InitialClimb
                    };
                    if self.go(to, "lifted off without a roll", t, ev) {
                        self.begin_flight(s, ev);
                    }
                    return;
                }
                if running && rolling && self.phase != Parked {
                    self.roll_peak_kts = s.gs_kts;
                    self.go(TakeoffRoll, "accelerating past the roll speed", t, ev);
                    return;
                }
                self.step_ground_departure(s, running, moving, stopped, ev);
            }
            TakeoffRoll => {
                self.roll_peak_kts = self.roll_peak_kts.max(s.gs_kts);
                if airborne {
                    if self.go(InitialClimb, "liftoff", t, ev) {
                        self.begin_flight(s, ev);
                    }
                } else if self.timers.reject.feed(
                    s.gs_kts < self.roll_peak_kts - REJECT_DROP_KTS,
                    t,
                    REJECT_SECS,
                ) {
                    self.memory.rejected_takeoffs += 1;
                    ev.push(Event::RejectedTakeoff);
                    self.go(RejectedTakeoff, "gave back the roll's speed", t, ev);
                }
            }
            RejectedTakeoff => {
                if rolling && s.gs_kts > self.roll_peak_kts {
                    self.go(TakeoffRoll, "rolling again", t, ev);
                } else if s.gs_kts < ROLL_KTS - 5.0 {
                    self.go(TaxiOut, "slowed to taxi speed", t, ev);
                }
            }
            Rollout => self.step_rollout(s, ev),
            TaxiIn => {
                if airborne {
                    let to = if s.gs_kts < HOVER_GS_KTS {
                        Hover
                    } else {
                        InitialClimb
                    };
                    if self.go(to, "lifted off from the taxi in", t, ev) {
                        self.begin_flight(s, ev);
                    }
                } else if s.engine_running && rolling {
                    // Taxied back and rolling again without a shutdown: a
                    // new flight.
                    self.roll_peak_kts = s.gs_kts;
                    if self.go(TakeoffRoll, "rolling again after a landing", t, ev) {
                        ev.push(Event::FlightComplete);
                    }
                } else if self.timers.parked.feed(
                    !s.engine_running && s.gs_kts < STOPPED_KTS,
                    t,
                    PARKED_SECS,
                ) {
                    self.go(Parked, "engines off at a stop", t, ev);
                    ev.push(Event::FlightComplete);
                }
            }
            _ => self.step_airborne(s, ev),
        }
    }

    fn step_ground_departure(
        &mut self,
        s: &Sample,
        running: bool,
        moving: bool,
        stopped: bool,
        ev: &mut Vec<Event>,
    ) {
        use Phase::*;
        let t = s.t;
        let engines_off_parked =
            self.timers
                .parked
                .feed(!s.engine_running && s.gs_kts < STOPPED_KTS, t, PARKED_SECS);
        match self.phase {
            Parked => {
                if s.starter == Some(true) {
                    self.go(EngineStart, "starter engaged", t, ev);
                } else if running {
                    self.go(Ready, "engines running", t, ev);
                }
            }
            EngineStart => {
                if s.engine_running && s.starter != Some(true) {
                    self.go(Ready, "engines running", t, ev);
                } else if s.starter == Some(false) && !s.engine_running {
                    self.go(Parked, "the start was abandoned", t, ev);
                }
            }
            Ready => {
                if engines_off_parked {
                    self.go(Parked, "engines off", t, ev);
                } else if s.starter == Some(true) {
                    self.go(EngineStart, "starter engaged", t, ev);
                } else if s.tug_attached == Some(true) {
                    self.go(Pushback, "tug on the bar", t, ev);
                } else if moving {
                    self.go(TaxiOut, "moving", t, ev);
                } else if stopped && s.on_runway == Some(true) {
                    self.go(Holding, "on a runway", t, ev);
                }
            }
            Pushback => {
                if s.tug_attached == Some(false) {
                    let to = if s.gs_kts > TAXI_KTS { TaxiOut } else { Ready };
                    self.go(to, "tug off the bar", t, ev);
                } else if engines_off_parked && s.tug_attached.is_none() {
                    self.go(Parked, "engines off", t, ev);
                }
            }
            TaxiOut => {
                if engines_off_parked {
                    self.go(Parked, "engines off", t, ev);
                } else if stopped {
                    let to = if s.on_runway == Some(true) {
                        LinedUp
                    } else {
                        Holding
                    };
                    self.go(to, "stopped", t, ev);
                }
            }
            Holding => {
                if engines_off_parked {
                    self.go(Parked, "engines off", t, ev);
                } else if s.on_runway == Some(true) && s.gs_kts < STOPPED_KTS {
                    self.go(LinedUp, "on the runway", t, ev);
                } else if moving {
                    self.go(TaxiOut, "moving", t, ev);
                }
            }
            LinedUp => {
                if engines_off_parked {
                    self.go(Parked, "engines off", t, ev);
                } else if moving && s.on_runway == Some(false) {
                    self.go(TaxiOut, "left the runway", t, ev);
                }
            }
            _ => {}
        }
    }

    fn step_rollout(&mut self, s: &Sample, ev: &mut Vec<Event>) {
        let t = s.t;
        if !s.on_ground {
            // Off the wheels again: a bounce until it climbs away. A touch
            // and go climbs away; a bounce, however long, comes back down
            // (-107 fpm, airborne 6 s, down again, in the recorded flights).
            self.off_again = true;
            let climbing_away = self.timers.airborne.feed(true, t, TOUCH_AND_GO_SECS)
                && s.vsi_fpm > VS_BAND_FPM
                || s.agl_ft > TOUCH_AND_GO_AGL_FT;
            if climbing_away && self.go(Phase::InitialClimb, "touch and go", t, ev) {
                self.off_again = false;
                self.memory.touch_and_goes += 1;
                self.memory.takeoff_t = Some(t);
                self.memory.aloft_t = Some(t);
                ev.push(Event::TouchAndGo);
            }
            return;
        }
        self.timers.airborne.feed(false, t, TOUCH_AND_GO_SECS);
        if std::mem::take(&mut self.off_again) {
            // Counted at the second contact, as rapport counts it.
            self.memory.bounces += 1;
            ev.push(Event::Bounce);
        }
        if self
            .timers
            .rollout_end
            .feed(s.gs_kts < ROLLOUT_END_KTS, t, ROLLOUT_END_SECS)
        {
            self.go(Phase::TaxiIn, "slowed to taxi speed", t, ev);
        }
    }

    fn step_airborne(&mut self, s: &Sample, ev: &mut Vec<Event>) {
        use Phase::*;
        let t = s.t;
        if s.on_ground {
            self.alt_track.clear();
            self.touch_down(s, ev);
            return;
        }
        let losing_height = self.losing_height(s);
        self.recent.push_back((t, s.vsi_fpm));
        while self
            .recent
            .front()
            .is_some_and(|(rt, _)| t - rt > RECENT_SECS)
        {
            self.recent.pop_front();
        }

        // A hover, from any airborne phase but the flare.
        if self.phase != Hover
            && self.phase != Landing
            && self.timers.hover.feed(
                s.gs_kts < HOVER_GS_KTS && s.ias_kts < HOVER_IAS_KTS,
                t,
                HOVER_SECS,
            )
        {
            self.go(Hover, "slow in the air", t, ev);
            return;
        }

        let climbing = self
            .timers
            .climbing
            .feed(s.vsi_fpm > VS_BAND_FPM, t, CLIMB_SECS);
        let level = self
            .timers
            .level
            .feed(s.vsi_fpm.abs() < VS_BAND_FPM, t, LEVEL_SECS);
        let descending = self
            .timers
            .descending
            .feed(s.vsi_fpm < -VS_BAND_FPM, t, DESCENT_SECS)
            || losing_height;
        let approach = s.approach == Some(true)
            || self.timers.approach.feed(
                s.agl_ft < APPROACH_AGL_FT && s.vsi_fpm < APPROACH_VS_FPM,
                t,
                APPROACH_SECS,
            );
        let go_around = s.go_around == Some(true)
            || self
                .timers
                .go_around
                .feed(s.vsi_fpm > GO_AROUND_VS_FPM, t, GO_AROUND_SECS);

        let moving_off = self
            .timers
            .hover_exit
            .feed(s.gs_kts > HOVER_EXIT_KTS, t, HOVER_EXIT_SECS);

        match self.phase {
            InitialClimb => {
                if s.agl_ft >= INITIAL_CLIMB_AGL_FT {
                    self.go(Climb, "through the initial climb height", t, ev);
                } else if approach {
                    self.go(Approach, "descending low", t, ev);
                } else if level {
                    self.go(Cruise, "levelled low", t, ev);
                }
            }
            Climb | Cruise | Descent => {
                if approach {
                    self.go(Approach, "descending low", t, ev);
                } else if climbing && self.phase != Climb {
                    self.go(Climb, "climbing", t, ev);
                } else if level && self.phase != Cruise && !self.stepping_down(s) {
                    self.go(Cruise, "level", t, ev);
                } else if descending && self.phase != Descent {
                    self.go(Descent, "descending", t, ev);
                }
            }
            Approach => {
                if go_around && s.approach != Some(true) || s.go_around == Some(true) {
                    self.start_go_around(s, ev);
                } else if s.agl_ft < LANDING_AGL_FT && s.vsi_fpm < 0.0 {
                    self.go(Landing, "into the flare", t, ev);
                } else if s.approach != Some(true)
                    && self.timers.approach_exit.feed(
                        s.agl_ft > APPROACH_EXIT_AGL_FT && s.vsi_fpm.abs() < VS_BAND_FPM,
                        t,
                        APPROACH_EXIT_SECS,
                    )
                {
                    self.go(Cruise, "level and high: no approach after all", t, ev);
                }
            }
            Landing => {
                if go_around {
                    self.start_go_around(s, ev);
                } else if s.agl_ft > LANDING_EXIT_AGL_FT {
                    self.go(Approach, "back above the flare", t, ev);
                }
            }
            GoAround => {
                self.go_around_floor_ft = self.go_around_floor_ft.min(s.agl_ft);
                let gained = s.agl_ft - self.go_around_floor_ft >= GO_AROUND_DONE_FT;
                let settled = self.timers.go_around_level.feed(
                    s.vsi_fpm.abs() < VS_BAND_FPM,
                    t,
                    GO_AROUND_LEVEL_SECS,
                );
                if gained {
                    self.go(Climb, "the go-around is climbing away", t, ev);
                } else if settled {
                    self.go(Cruise, "levelled after the go-around", t, ev);
                } else if approach && s.vsi_fpm < APPROACH_VS_FPM && s.go_around != Some(true) {
                    self.go(Approach, "back on an approach", t, ev);
                }
            }
            Hover if moving_off => {
                let to = vertical(s);
                self.go(to, "moving off from the hover", t, ev);
            }
            _ => {}
        }
    }

    /// Whether the height lost over the last [`DESCENT_SECS`] is a descent
    /// past the band: the trend of the altitude, not the needle's run.
    /// Keeps one sample at or before the window's start as its anchor.
    fn losing_height(&mut self, s: &Sample) -> bool {
        let t = s.t;
        self.alt_track.push_back((t, s.alt_ft));
        while self
            .alt_track
            .get(1)
            .is_some_and(|&(t1, _)| t - t1 >= DESCENT_SECS)
        {
            self.alt_track.pop_front();
        }
        let Some(&(t0, a0)) = self.alt_track.front() else {
            return false;
        };
        let span = t - t0;
        if !(DESCENT_SECS..=TREND_MAX_SECS).contains(&span) {
            return false;
        }
        (s.alt_ft - a0) / span * 60.0 < -VS_BAND_FPM
    }

    /// A level-off well below the flight's highest level, in a descent, is a
    /// step of the descent (the follower's leg-by-leg constraints), not a
    /// new cruise, until it has held long enough to be one.
    fn stepping_down(&self, s: &Sample) -> bool {
        self.phase == Phase::Descent
            && self.memory.peak_alt_ft - s.alt_ft > STEP_BELOW_PEAK_FT
            && self.in_phase_secs(s.t) < STEP_MAX_SECS
    }

    fn start_go_around(&mut self, s: &Sample, ev: &mut Vec<Event>) {
        if self.go(Phase::GoAround, "climbing out of the approach", s.t, ev) {
            self.go_around_floor_ft = s.agl_ft;
            self.memory.go_arounds += 1;
            ev.push(Event::GoAround);
        }
    }

    fn touch_down(&mut self, s: &Sample, ev: &mut Vec<Event>) {
        if s.t - self.restarted_t < SETTLE_SECS
            && self.memory.takeoff_t.is_none()
            && s.gs_kts < TAXI_KTS
        {
            // Placed in the air a moment ago and already down: the sim
            // settling a loaded aircraft onto its wheels, not a landing.
            self.restart(s, ev);
            return;
        }
        let td = self.contact(s);
        let to = Self::off_the_air(s);
        if td.sink_fpm > VS_BAND_FPM || td.sink_fpm < -IMPACT_SINK_FPM {
            let why = if td.sink_fpm > 0.0 {
                "flew into the ground"
            } else {
                "struck the ground"
            };
            self.impact(s, td, why, ev);
            return;
        }
        let why = match self.phase {
            Phase::Landing | Phase::Approach | Phase::Hover => "touchdown",
            _ => "met the ground out of an approach we never saw",
        };
        if self.go(to, why, s.t, ev) {
            self.memory.landings += 1;
            self.memory.last_touchdown = Some(td);
            self.off_again = false;
            self.recent.clear();
            ev.push(Event::Touchdown(td));
        }
    }

    /// The wheels' first contact, as a touchdown or an impact reads it.
    fn contact(&self, s: &Sample) -> Touchdown {
        let recent: Vec<(f64, f64)> = self.recent.iter().copied().collect();
        Touchdown {
            t: s.t,
            sink_fpm: crate::instructor::touchdown_sink_fpm(&recent, s.t, s.vsi_fpm),
            gs_kts: s.gs_kts,
            lat: s.lat,
            lon: s.lon,
        }
    }

    /// The ground phase an arrival on the wheels starts in.
    fn off_the_air(s: &Sample) -> Phase {
        if s.gs_kts >= ROLLOUT_END_KTS {
            Phase::Rollout
        } else {
            Phase::TaxiIn
        }
    }

    fn impact(&mut self, s: &Sample, td: Touchdown, why: &'static str, ev: &mut Vec<Event>) {
        if self.go(Self::off_the_air(s), why, s.t, ev) {
            self.memory.impacts += 1;
            self.recent.clear();
            self.wreck = true;
            ev.push(Event::Impact(td));
        }
    }

    /// A takeoff: the flight's memory starts here.
    fn begin_flight(&mut self, s: &Sample, ev: &mut Vec<Event>) {
        self.memory = Memory {
            flight_no: self.memory.flight_no + 1,
            origin: s.lat.zip(s.lon),
            takeoff_t: Some(s.t),
            aloft_t: Some(s.t),
            fuel_at_takeoff_kg: s.fuel_kg,
            ..Memory::default()
        };
        self.recent.clear();
        ev.push(Event::Takeoff);
    }

    fn remember(&mut self, s: &Sample) {
        if !s.on_ground {
            self.memory.peak_alt_ft = self.memory.peak_alt_ft.max(s.alt_ft);
            self.memory.peak_agl_ft = self.memory.peak_agl_ft.max(s.agl_ft);
        }
    }

    fn step_control(&mut self, s: &Sample, ev: &mut Vec<Event>) {
        let to = match (s.automation, s.autopilot) {
            (Some(name), _) => Control::Automation(name),
            (None, Some(true)) => Control::Autopilot,
            (None, Some(false)) => Control::Pilot,
            (None, None) => Control::Unknown,
        };
        if to != self.control {
            ev.push(Event::Control {
                from: self.control,
                to,
            });
            self.control = to;
        }
    }

    fn step_situation(&mut self, now: &Situation, held_by: Control, ev: &mut Vec<Event>) {
        let was = &mut self.situation;
        if now.mission != was.mission {
            ev.push(Event::Mission {
                from: was.mission,
                to: now.mission,
            });
        }
        if now.plan != was.plan {
            ev.push(Event::Plan {
                from: was.plan.clone(),
                to: now.plan.clone(),
            });
        }
        if now.active_fix != was.active_fix {
            ev.push(Event::ActiveFix {
                from: was.active_fix.clone(),
                to: now.active_fix.clone(),
            });
        }
        if now.clearance != was.clearance {
            ev.push(Event::Clearance {
                from: was.clearance.clone(),
                to: now.clearance.clone(),
            });
        }
        // A NEW takeover (the mailbox keeps the last one for good). The first
        // situation seen adopts whatever is there: a takeover from before
        // the model started is no news.
        let fresh = match (was.pilot_takeover, now.pilot_takeover) {
            (Some((a, _)), Some((b, _))) => b > a,
            (None, Some(_)) => self.situation_seen,
            _ => false,
        };
        if let (true, Some((_, what))) = (fresh, now.pilot_takeover) {
            self.memory.pilot_takeovers += 1;
            ev.push(Event::PilotTookOver {
                what,
                control: held_by,
            });
        }
        self.situation = now.clone();
        self.situation_seen = true;
    }

    fn step_abnormal(&mut self, s: &Sample, ev: &mut Vec<Event>) {
        let to = if s.crashed == Some(true) {
            Abnormal::Crashed
        } else if s.emergency == Some(true) {
            Abnormal::Emergency
        } else {
            match &s.failed {
                Some(f) if !f.is_empty() => Abnormal::Failure(f.clone()),
                Some(_) => Abnormal::Normal,
                // Unknown this tick: the last verdict stands, except a wreck
                // or an emergency, which need their evidence every tick.
                None => match &self.abnormal {
                    Abnormal::Failure(f) => Abnormal::Failure(f.clone()),
                    _ => Abnormal::Normal,
                },
            }
        };
        if to != self.abnormal {
            ev.push(Event::Abnormal {
                from: self.abnormal.clone(),
                to: to.clone(),
            });
            self.abnormal = to;
        }
    }
}

/// The sim loading a flight: "airborne" with the ground hundreds of feet
/// (or millions) above the wheels, for up to minutes, in the recorded
/// flights. No aircraft flies below the ground.
fn loading(s: &Sample) -> bool {
    !s.on_ground && s.agl_ft < LOADING_AGL_FT
}

/// The phase a sample is in with no history: the start, and a reposition.
pub fn infer(s: &Sample) -> Phase {
    if s.on_ground {
        if !s.engine_running {
            Phase::Parked
        } else if s.gs_kts > ROLL_KTS {
            Phase::TakeoffRoll
        } else if s.gs_kts > TAXI_KTS {
            Phase::TaxiOut
        } else if s.on_runway == Some(true) {
            Phase::LinedUp
        } else {
            Phase::Ready
        }
    } else if s.gs_kts < HOVER_GS_KTS && s.ias_kts < HOVER_IAS_KTS {
        Phase::Hover
    } else if s.approach == Some(true) {
        Phase::Approach
    } else {
        vertical(s)
    }
}

/// The airborne phase an instantaneous vertical speed suggests.
fn vertical(s: &Sample) -> Phase {
    if s.vsi_fpm > VS_BAND_FPM {
        if s.agl_ft < INITIAL_CLIMB_AGL_FT {
            Phase::InitialClimb
        } else {
            Phase::Climb
        }
    } else if s.vsi_fpm < -VS_BAND_FPM {
        if s.agl_ft < APPROACH_AGL_FT {
            Phase::Approach
        } else {
            Phase::Descent
        }
    } else {
        Phase::Cruise
    }
}

impl Event {
    /// The blackbox `flight` record's fields.
    pub fn to_json(&self) -> serde_json::Value {
        use serde_json::json;
        let ph = |p: &Phase| format!("{p:?}");
        let ctl = |c: &Control| match c {
            Control::Automation(n) => n.to_string(),
            other => format!("{other:?}"),
        };
        let abn = |a: &Abnormal| match a {
            Abnormal::Failure(f) => format!("Failure: {}", f.join(", ")),
            other => format!("{other:?}"),
        };
        match self {
            Event::Phase { from, to, why } => {
                json!({ "ev": "phase", "from": ph(from), "to": ph(to), "why": why })
            }
            Event::Reposition { to } => json!({ "ev": "reposition", "to": ph(to) }),
            Event::Touchdown(td) => json!({
                "ev": "touchdown", "sink": td.sink_fpm.round(), "gs": td.gs_kts.round(),
            }),
            Event::Impact(td) => json!({
                "ev": "impact", "sink": td.sink_fpm.round(), "gs": td.gs_kts.round(),
            }),
            Event::Control { from, to } => {
                json!({ "ev": "control", "from": ctl(from), "to": ctl(to) })
            }
            Event::Abnormal { from, to } => {
                json!({ "ev": "abnormal", "from": abn(from), "to": abn(to) })
            }
            Event::Mission { from, to } => json!({ "ev": "mission", "from": from, "to": to }),
            Event::Plan { from, to } => {
                let p = |p: &Option<PlanFact>| {
                    p.as_ref().map(|p| {
                        format!(
                            "{} {}-{} FL{:03.0}",
                            p.source,
                            p.origin,
                            p.destination,
                            p.cruise_alt_ft / 100.0
                        )
                    })
                };
                json!({ "ev": "plan", "from": p(from), "to": p(to) })
            }
            Event::ActiveFix { from, to } => json!({ "ev": "active_fix", "from": from, "to": to }),
            Event::Clearance { from, to } => {
                let c = |c: &Option<ClearanceFact>| {
                    c.as_ref().map(|c| {
                        json!({ "state": c.state, "rwy": c.runway, "squawk": c.squawk,
                                "readback_owed": c.readback_owed })
                    })
                };
                json!({ "ev": "clearance", "from": c(from), "to": c(to) })
            }
            Event::PilotTookOver { what, control } => {
                json!({ "ev": "pilot_took_over", "what": what, "control": ctl(control) })
            }
            Event::Takeoff => json!({ "ev": "takeoff" }),
            Event::Bounce => json!({ "ev": "bounce" }),
            Event::TouchAndGo => json!({ "ev": "touch_and_go" }),
            Event::GoAround => json!({ "ev": "go_around" }),
            Event::RejectedTakeoff => json!({ "ev": "rejected_takeoff" }),
            Event::FlightComplete => json!({ "ev": "flight_complete" }),
        }
    }
}

/// A recorded flight fed back through the model: `state` records are the
/// samples, and a `crash` record is the sim's crash flag from then until the
/// track restarts (older state records do not carry it).
#[derive(Debug, Default)]
pub struct Replay {
    pub flight: Flight,
    crashed: bool,
}

impl Replay {
    /// One blackbox record: the sample it made (if a `state` record) and
    /// what the model did with it.
    pub fn feed(&mut self, v: &serde_json::Value) -> Option<(Sample, Vec<Event>)> {
        if v.get("kind").and_then(|k| k.as_str()) == Some("crash") {
            self.crashed = true;
            return None;
        }
        let mut s = Sample::from_state_record(v)?;
        match s.crashed {
            Some(c) => self.crashed = c,
            None => s.crashed = Some(self.crashed),
        }
        let ev = self.flight.tick(&s);
        if ev.iter().any(|e| matches!(e, Event::Reposition { .. })) {
            self.crashed = false;
        }
        Some((s, ev))
    }
}

/// The flight as the crew would sum it up: the model's view after the last
/// tick, published by the monitor for the question "how are we doing?".
#[derive(Debug, Clone)]
pub struct Status {
    pub phase: Phase,
    pub control: Control,
    pub abnormal: Abnormal,
    pub situation: Situation,
    pub alt_ft: f64,
    pub ias_kts: f64,
    pub gs_kts: f64,
    /// The highest the flight has been (the cruise, once it has had one).
    pub peak_alt_ft: f64,
}

impl Flight {
    /// The summary after the last tick; `None` before the first sample.
    pub fn status(&self) -> Option<Status> {
        let s = self.last.as_ref()?;
        Some(Status {
            phase: self.phase,
            control: self.control,
            abnormal: self.abnormal.clone(),
            situation: self.situation.clone(),
            alt_ft: s.alt_ft,
            ias_kts: s.ias_kts,
            gs_kts: s.gs_kts,
            peak_alt_ft: self.memory.peak_alt_ft,
        })
    }
}

static PUBLISHED: std::sync::Mutex<Option<Status>> = std::sync::Mutex::new(None);

/// The monitor's latest summary, for the command loop.
pub fn publish(s: Option<Status>) {
    *PUBLISHED.lock().unwrap_or_else(|e| e.into_inner()) = s;
}

pub fn published() -> Option<Status> {
    PUBLISHED.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// "How are we doing?", answered from the flight: where we are, who is
/// flying, the plan, and whether anything has failed. Pure. The open question
/// earned silence five times in a row over LILB (2026-09-25).
pub fn status_line(s: &Status) -> String {
    let mut out = Vec::new();
    let mut first = s.phase.spoken().to_string();
    if let Some(c) = first.get_mut(0..1) {
        c.make_ascii_uppercase();
    }
    if s.phase.airborne() {
        let alt = (s.alt_ft / 100.0).round() * 100.0;
        out.push(format!(
            "{first} through {} feet at {:.0} knots.",
            crate::flightplan::spoken_feet(alt).trim_end_matches(" feet"),
            s.ias_kts
        ));
    } else if s.gs_kts >= 3.0 {
        out.push(format!("{first} at {:.0} knots.", s.gs_kts));
    } else {
        out.push(format!("{first}."));
    }
    let who = match (s.situation.mission, s.control) {
        (Some(m), _) => Some(format!("{} is flying.", capitalise(m))),
        (None, Control::Automation(n)) => Some(format!("{} is flying.", capitalise(n))),
        (None, Control::Autopilot) => Some("The autopilot has it.".to_string()),
        (None, Control::Pilot) => Some("You're flying.".to_string()),
        (None, Control::Unknown) => None,
    };
    out.extend(who);
    if let Some(p) = &s.situation.plan {
        match &s.situation.active_fix {
            Some(fix) => out.push(format!("{} to {}, next fix {fix}.", p.origin, p.destination)),
            None => out.push(format!("Plan {} to {}.", p.origin, p.destination)),
        }
    }
    out.push(match &s.abnormal {
        Abnormal::Normal => "No failures.".to_string(),
        Abnormal::Failure(f) => format!("Failed: {}.", f.join(", ")),
        Abnormal::Emergency => "We've declared an emergency.".to_string(),
        Abnormal::Crashed => "We've crashed.".to_string(),
    });
    out.join(" ")
}

/// The panel's State tab: the flight model and the duty engine, one line
/// each, for the pilot to glance at while it flies. `phase_s` is the time in
/// the phase (shown to the nearest 10 s, so the feed is not re-sent every
/// second). Pure.
pub fn state_lines(
    s: &Status,
    m: &Memory,
    phase_s: f64,
    duty: (&str, &str, Option<bool>),
) -> String {
    let mut out = Vec::new();
    out.push(format!(
        "Phase: {:?}, {} s",
        s.phase,
        ((phase_s / 10.0).floor() * 10.0).max(0.0) as i64
    ));
    out.push(format!(
        "Flying: {}",
        match s.control {
            Control::Automation(n) => n.to_string(),
            Control::Autopilot => "autopilot".to_string(),
            Control::Pilot => "you".to_string(),
            Control::Unknown => "unknown".to_string(),
        }
    ));
    out.push(format!("Mission: {}", s.situation.mission.unwrap_or("none")));
    out.push(match (&s.situation.plan, &s.situation.active_fix) {
        (Some(p), Some(f)) => format!("Plan: {}-{}, fix {f}", p.origin, p.destination),
        (Some(p), None) => format!("Plan: {}-{}", p.origin, p.destination),
        (None, _) => "Plan: none".to_string(),
    });
    let tower = match s.situation.towered {
        Some(true) => "tower in reach",
        Some(false) => "no tower in reach",
        None => "tower unknown",
    };
    out.push(match &s.situation.clearance {
        Some(c) => format!(
            "Clearance: {}{}{}, {tower}{}",
            c.state,
            c.runway.as_ref().map(|r| format!(" rwy {r}")).unwrap_or_default(),
            c.squawk.map(|q| format!(", squawk {q}")).unwrap_or_default(),
            if c.readback_owed { ", readback owed" } else { "" }
        ),
        None => format!("Clearance: none, {tower}"),
    });
    let (name, why, agrees) = duty;
    out.push(format!(
        "Next duty: {} ({why}){}",
        name.replace('_', " "),
        match agrees {
            Some(true) => ", automation agrees",
            Some(false) => ", automation DISAGREES",
            None => "",
        }
    ));
    out.push(match &s.abnormal {
        Abnormal::Normal => "Status: normal".to_string(),
        Abnormal::Failure(f) => format!("Status: failed {}", f.join(", ")),
        Abnormal::Emergency => "Status: EMERGENCY".to_string(),
        Abnormal::Crashed => "Status: CRASHED".to_string(),
    });
    out.push(format!(
        "This flight: {} landing(s), {} go-around(s), {} takeover(s), peak {:.0} ft",
        m.landings, m.go_arounds, m.pilot_takeovers, m.peak_alt_ft
    ));
    out.join("\n")
}

fn capitalise(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_ascii_uppercase().to_string() + c.as_str(),
        None => String::new(),
    }
}

/// Most phases listed in the shadow trail, so a long flight stays readable.
const SHADOW_TRAIL_MAX: usize = 40;
/// A ground/air disagreement shorter than this is the model confirming.
const SHADOW_DISAGREE_SECS: f64 = 5.0;

/// The shadow run, read back: the flight's `state` records replayed through
/// the model beside the old `monitor::estimate_phase` label each record
/// carries. The one hard comparison is ground against air (the two must
/// agree on it; everything finer is the model's to decide); the rest is
/// how often each changed its mind. Works on flights recorded before the
/// model existed. Empty when the flight has no state records.
pub fn shadow_report(records: &[&serde_json::Value]) -> Vec<String> {
    let mut r = Replay::default();
    let (mut ticks, mut old_flips, mut new_changes) = (0u32, 0u32, 0u32);
    let mut old_prev: Option<String> = None;
    // Ground/air disagreements lasting SHADOW_DISAGREE_SECS: the model's
    // own confirmation (a liftoff held 1.5 s) is not one.
    let mut ground_air: Vec<(f64, f64, String, Phase)> = Vec::new();
    let mut run: Option<(f64, f64, String, Phase)> = None;
    let mut trail: Vec<String> = Vec::new();
    for v in records {
        let Some((s, events)) = r.feed(v) else {
            continue;
        };
        let f = &r.flight;
        ticks += 1;
        for e in events {
            match e {
                Event::Phase { to, .. } => {
                    new_changes += 1;
                    trail.push(format!("{to:?}"));
                }
                Event::Reposition { to } => {
                    new_changes += 1;
                    trail.push(format!("[reposition {to:?}]"));
                }
                Event::GoAround => trail.push("(go-around)".into()),
                Event::Bounce => trail.push("(bounce)".into()),
                Event::TouchAndGo => trail.push("(touch and go)".into()),
                Event::RejectedTakeoff => trail.push("(rejected)".into()),
                Event::Touchdown(td) => trail.push(format!("(touchdown {:.0} fpm)", td.sink_fpm)),
                Event::Impact(td) => trail.push(format!("(IMPACT {:.0} fpm)", td.sink_fpm)),
                Event::Abnormal {
                    to: Abnormal::Crashed,
                    ..
                } => trail.push("(CRASHED)".into()),
                _ => {}
            }
        }
        let Some(old) = v.get("phase").and_then(|p| p.as_str()) else {
            continue;
        };
        if old_prev.as_deref().is_some_and(|p| p != old) {
            old_flips += 1;
        }
        old_prev = Some(old.to_string());
        // The model waits out a loading sample and a started track; judge
        // it only once it has one.
        let old_ground = matches!(old, "Ground" | "Takeoff");
        if f.started && old_ground == f.phase().airborne() {
            match run.as_mut() {
                Some(r) => r.1 = s.t,
                None => run = Some((s.t, s.t, old.to_string(), f.phase())),
            }
        } else if let Some(r) = run.take() {
            ground_air.push(r);
        }
    }
    ground_air.extend(run);
    ground_air.retain(|(a, b, ..)| b - a >= SHADOW_DISAGREE_SECS);
    if ticks == 0 {
        return Vec::new();
    }
    let f = &r.flight;
    let mut out = vec![format!(
        "  {ticks} ticks: the model changed phase {new_changes} times, the old estimate {old_flips}."
    )];
    if trail.is_empty() {
        trail.push(format!("{:?}", f.phase()));
    }
    let extra = trail.len().saturating_sub(SHADOW_TRAIL_MAX);
    trail.truncate(SHADOW_TRAIL_MAX);
    let more = if extra > 0 {
        format!(" > ... {extra} more")
    } else {
        String::new()
    };
    out.push(format!("  {}{more}", trail.join(" > ")));
    let m = f.memory();
    out.push(format!(
        "  Last flight's memory: {} landing(s), {} touch and go, {} go-around(s), {} bounce(s), {} reject(s), {} impact(s), peak {:.0} ft.",
        m.landings, m.touch_and_goes, m.go_arounds, m.bounces, m.rejected_takeoffs, m.impacts, m.peak_alt_ft
    ));
    if let Some((a, b, old, new)) = ground_air.first() {
        out.push(format!(
            "  GROUND/AIR DISAGREE {} time(s), first for {:.0} s at t={a:.0}: old {old}, model {new:?}.",
            ground_air.len(),
            b - a
        ));
    }
    for (from, to, why) in f.refused() {
        out.push(format!("  ILLEGAL EDGE asked: {from:?} -> {to:?} ({why})."));
    }
    // The crew's side is only known live: what the flight recorded of it.
    let situation: Vec<String> = records
        .iter()
        .filter(|v| v.get("kind").and_then(|k| k.as_str()) == Some("flight"))
        .filter_map(|v| {
            let ev = v.get("ev")?.as_str()?;
            let tv = |k: &str| match v.get(k) {
                Some(serde_json::Value::Null) | None => "none".to_string(),
                Some(serde_json::Value::String(x)) => x.clone(),
                Some(x) => x
                    .get("state")
                    .and_then(|s| s.as_str())
                    .unwrap_or("?")
                    .to_string(),
            };
            match ev {
                "mission" | "plan" | "clearance" => Some(format!("{ev} {}", tv("to"))),
                "pilot_took_over" => Some(format!("PILOT TOOK OVER from {}", tv("what"))),
                _ => None,
            }
        })
        .collect();
    if !situation.is_empty() {
        let extra = situation.len().saturating_sub(SHADOW_TRAIL_MAX);
        let more = if extra > 0 {
            format!(" > ... {extra} more")
        } else {
            String::new()
        };
        out.push(format!(
            "  Situation: {}{more}",
            situation
                .iter()
                .take(SHADOW_TRAIL_MAX)
                .cloned()
                .collect::<Vec<_>>()
                .join(" > ")
        ));
    }
    out
}

fn distance_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let x = (lon2 - lon1).to_radians() * ((lat1 + lat2) / 2.0).to_radians().cos();
    let y = (lat2 - lat1).to_radians();
    (x * x + y * y).sqrt() * 3440.065
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ground(t: f64, gs: f64, eng: bool) -> Sample {
        Sample {
            t,
            on_ground: true,
            gs_kts: gs,
            ias_kts: gs,
            engine_running: eng,
            ..Sample::default()
        }
    }

    fn air(t: f64, agl: f64, vsi: f64, gs: f64) -> Sample {
        Sample {
            t,
            on_ground: false,
            gs_kts: gs,
            ias_kts: gs,
            vsi_fpm: vsi,
            agl_ft: agl,
            alt_ft: agl + 100.0,
            engine_running: true,
            ..Sample::default()
        }
    }

    /// Feed a script of samples at 1 Hz; the phases visited, in order.
    fn phases(samples: impl IntoIterator<Item = Sample>) -> (Vec<Phase>, Flight, Vec<Event>) {
        let mut f = Flight::new();
        let mut seen = Vec::new();
        let mut all = Vec::new();
        for s in samples {
            let ev = f.tick(&s);
            if seen.last() != Some(&f.phase()) {
                seen.push(f.phase());
            }
            all.extend(ev);
        }
        assert!(f.refused().is_empty(), "illegal edges: {:?}", f.refused());
        (seen, f, all)
    }

    /// A 172 circuit: start, taxi, hold, roll, climb to 1,000 AGL, level
    /// downwind, descend, land, taxi in, shut down.
    fn circuit() -> Vec<Sample> {
        let mut v = Vec::new();
        let mut t = 0.0;
        let push = |s: Sample, n: usize, v: &mut Vec<Sample>, t: &mut f64| {
            for _ in 0..n {
                let mut s = s.clone();
                s.t = *t;
                v.push(s);
                *t += 1.0;
            }
        };
        push(ground(0.0, 0.0, false), 10, &mut v, &mut t);
        push(ground(0.0, 0.0, true), 10, &mut v, &mut t);
        push(ground(0.0, 10.0, true), 60, &mut v, &mut t);
        push(ground(0.0, 0.0, true), 30, &mut v, &mut t);
        for i in 0..20 {
            push(ground(0.0, 5.0 + i as f64 * 3.0, true), 1, &mut v, &mut t);
        }
        // Climb 700 fpm from 0 to 1,000 ft: ~86 s.
        for i in 0..90 {
            push(air(0.0, i as f64 * 11.7, 700.0, 70.0), 1, &mut v, &mut t);
        }
        push(air(0.0, 1050.0, 0.0, 90.0), 60, &mut v, &mut t);
        // Descend 500 fpm to the flare.
        for i in 0..124 {
            push(
                air(0.0, 1050.0 - i as f64 * 8.3, -500.0, 65.0),
                1,
                &mut v,
                &mut t,
            );
        }
        push(air(0.0, 20.0, -300.0, 60.0), 1, &mut v, &mut t);
        push(ground(0.0, 55.0, true), 5, &mut v, &mut t);
        push(ground(0.0, 12.0, true), 30, &mut v, &mut t);
        push(ground(0.0, 0.0, true), 10, &mut v, &mut t);
        push(ground(0.0, 0.0, false), 10, &mut v, &mut t);
        v
    }

    #[test]
    fn a_circuit_visits_every_phase_in_order() {
        use Phase::*;
        let (seen, f, ev) = phases(circuit());
        assert_eq!(
            seen,
            vec![
                // Off the hold line onto the runway is taxi; the downwind
                // at 1,000 ft goes straight into the approach.
                Parked,
                Ready,
                TaxiOut,
                Holding,
                TaxiOut,
                TakeoffRoll,
                InitialClimb,
                Climb,
                Cruise,
                Approach,
                Landing,
                Rollout,
                TaxiIn,
                Parked
            ]
        );
        let m = f.memory();
        assert_eq!(m.landings, 1);
        assert_eq!(m.go_arounds, 0);
        assert!(ev.contains(&Event::Takeoff));
        assert!(ev.contains(&Event::FlightComplete));
        let td = m.last_touchdown.unwrap();
        assert!((td.sink_fpm - -500.0).abs() < 1.0, "{td:?}");
    }

    #[test]
    fn a_gust_is_not_a_descent() {
        // Level at 2,500 ft AGL, one 12 s spell of -400 fpm: still cruise.
        let mut v: Vec<Sample> = (0..40).map(|t| air(t as f64, 2500.0, 0.0, 110.0)).collect();
        v.extend((40..52).map(|t| air(t as f64, 2450.0, -400.0, 110.0)));
        v.extend((52..90).map(|t| air(t as f64, 2420.0, 0.0, 110.0)));
        let (seen, _, _) = phases(v);
        assert_eq!(seen, vec![Phase::Cruise]);
    }

    #[test]
    fn the_old_estimate_called_a_low_cruise_sink_an_approach() {
        // The stateless estimate: agl < 3,000 and vsi < -300 was Approach
        // on the very tick. Held level at 2,500 with a 15 s -350 fpm sink
        // is a cruise; only a sustained low descent is an approach.
        let mut v: Vec<Sample> = (0..40).map(|t| air(t as f64, 2500.0, 0.0, 110.0)).collect();
        v.extend((40..55).map(|t| air(t as f64, 2450.0, -350.0, 110.0)));
        let (seen, _, _) = phases(v);
        assert_eq!(seen, vec![Phase::Cruise]);
    }

    #[test]
    fn a_go_around_is_counted_and_climbs_away() {
        use Phase::*;
        let mut v: Vec<Sample> = (0..30)
            .map(|t| air(t as f64, 800.0 - t as f64 * 10.0, -600.0, 70.0))
            .collect();
        v.extend((30..80).map(|t| air(t as f64, 500.0 + (t - 30) as f64 * 15.0, 900.0, 70.0)));
        let (seen, f, ev) = phases(v);
        assert_eq!(seen, vec![Approach, GoAround, Climb]);
        assert_eq!(f.memory().go_arounds, 1);
        assert!(ev.contains(&Event::GoAround));
    }

    /// LILO 2026-09-24: the S-76 climbing at +708 fpm met the ridge, then
    /// the wreck slid at 44 kt. An impact, never a landing, and the slide
    /// is no takeoff roll and no reject.
    #[test]
    fn flying_into_the_ground_is_an_impact_and_the_wreck_has_no_phases() {
        let mut v: Vec<Sample> = (0..40).map(|i| air(i as f64, 20.0, 700.0, 36.0)).collect();
        let mut hit = ground(40.0, 33.0, true);
        hit.vsi_fpm = 708.0;
        v.push(hit);
        for i in 41..60 {
            v.push(ground(i as f64, 44.0, true));
        }
        for i in 60..80 {
            v.push(ground(i as f64, 0.0, false));
        }
        let (seen, f, ev) = phases(v);
        assert!(ev.iter().any(|e| matches!(e, Event::Impact(_))), "{ev:?}");
        assert!(!ev.iter().any(|e| matches!(e, Event::Touchdown(_))));
        assert!(!seen.contains(&Phase::TakeoffRoll), "{seen:?}");
        assert!(!seen.contains(&Phase::RejectedTakeoff), "{seen:?}");
        assert_eq!(f.memory().landings, 0);
        assert_eq!(f.memory().impacts, 1);
        assert_eq!(
            f.phase(),
            Phase::Parked,
            "the wreck at rest resumes the track"
        );
    }

    #[test]
    fn a_sink_past_the_impact_figure_is_not_a_landing() {
        let mut v: Vec<Sample> = (0..20)
            .map(|i| air(i as f64, 80.0 - i as f64 * 4.0, -2900.0, 35.0))
            .collect();
        let mut hit = ground(20.0, 2.0, false);
        hit.vsi_fpm = -2886.0;
        v.push(hit);
        let (_, f, ev) = phases(v);
        assert!(ev.iter().any(|e| matches!(e, Event::Impact(_))), "{ev:?}");
        assert_eq!(f.memory().landings, 0);
    }

    /// The sim's crash flag holds the phase for as long as it is raised.
    #[test]
    fn the_crash_flag_freezes_the_phase() {
        let mut f = Flight::new();
        for i in 0..30 {
            f.tick(&air(i as f64, 3000.0, 0.0, 120.0));
        }
        let before = f.phase();
        for i in 30..60 {
            // Slow in the air: a hover, were it not a wreck.
            let mut s = air(i as f64, 3000.0, 0.0, 5.0);
            s.crashed = Some(true);
            f.tick(&s);
        }
        assert_eq!(f.phase(), before);
        assert_eq!(f.abnormal(), &Abnormal::Crashed);
        assert_eq!(f.memory().landings, 0);
    }

    fn with(t: f64, sit: Situation) -> Sample {
        let mut s = ground(t, 0.0, true);
        s.situation = Some(sit);
        s
    }

    #[test]
    fn the_situation_tracks_report_each_change_once() {
        let mut f = Flight::new();
        let plan = PlanFact {
            source: "simbrief",
            origin: "EGPH".into(),
            destination: "EGLL".into(),
            cruise_alt_ft: 35000.0,
        };
        let cleared = ClearanceFact {
            state: "TaxiCleared".into(),
            runway: Some("24".into()),
            squawk: Some(4521),
            readback_owed: false,
        };
        let mut sit = Situation::default();
        assert!(f
            .tick(&with(0.0, sit.clone()))
            .iter()
            .all(|e| !matches!(e, Event::Plan { .. })));
        sit.plan = Some(plan.clone());
        let ev = f.tick(&with(1.0, sit.clone()));
        assert!(
            ev.contains(&Event::Plan {
                from: None,
                to: Some(plan.clone())
            }),
            "{ev:?}"
        );
        // Unchanged: no news.
        assert!(!f
            .tick(&with(2.0, sit.clone()))
            .iter()
            .any(|e| matches!(e, Event::Plan { .. })));
        sit.mission = Some("full auto");
        sit.clearance = Some(cleared.clone());
        let ev = f.tick(&with(3.0, sit.clone()));
        assert!(ev.contains(&Event::Mission {
            from: None,
            to: Some("full auto")
        }));
        assert!(ev.iter().any(
            |e| matches!(e, Event::Clearance { to: Some(c), .. } if c.state == "TaxiCleared")
        ));
        assert_eq!(f.situation().plan.as_ref(), Some(&plan));
        // A replayed sample (no situation) leaves the tracks standing.
        f.tick(&ground(4.0, 0.0, true));
        assert_eq!(f.situation().mission, Some("full auto"));
    }

    /// A takeover is news once, credited to whoever held the aircraft the
    /// tick before; one from before the model started is not news.
    #[test]
    fn a_pilot_takeover_is_counted_once_against_the_loop_that_had_it() {
        let mut f = Flight::new();
        let mut sit = Situation {
            pilot_takeover: Some((-50.0, "my controls")),
            ..Situation::default()
        };
        let mut s = with(0.0, sit.clone());
        s.automation = Some("taxi");
        assert!(!f
            .tick(&s)
            .iter()
            .any(|e| matches!(e, Event::PilotTookOver { .. })));
        sit.pilot_takeover = Some((1.0, "taxi: pedals"));
        let mut s = with(1.0, sit.clone());
        s.autopilot = Some(false);
        let ev = f.tick(&s);
        assert!(
            ev.contains(&Event::PilotTookOver {
                what: "taxi: pedals",
                control: Control::Automation("taxi")
            }),
            "{ev:?}"
        );
        assert_eq!(f.memory().pilot_takeovers, 1);
        assert!(!f
            .tick(&with(2.0, sit))
            .iter()
            .any(|e| matches!(e, Event::PilotTookOver { .. })));
        assert_eq!(f.memory().pilot_takeovers, 1);
    }

    #[test]
    fn the_state_tab_shows_the_flight_and_the_next_duty() {
        let s = Status {
            phase: Phase::TaxiOut,
            control: Control::Automation("taxi"),
            abnormal: Abnormal::Normal,
            situation: Situation {
                mission: Some("full auto"),
                towered: Some(true),
                ..Situation::default()
            },
            alt_ft: 700.0,
            ias_kts: 10.0,
            gs_kts: 10.0,
            peak_alt_ft: 0.0,
        };
        let t = state_lines(&s, &Memory::default(), 47.0, ("taxi", "engines running", Some(true)));
        assert!(t.starts_with("Phase: TaxiOut, 40 s"), "{t}");
        assert!(t.contains("Flying: taxi"), "{t}");
        assert!(t.contains("Next duty: taxi (engines running), automation agrees"), "{t}");
        assert!(t.contains("Clearance: none, tower in reach"), "{t}");
        assert!(t.len() < 500, "fits the plugin's buffer: {}", t.len());
    }

    #[test]
    fn how_are_we_doing_is_answered_from_the_flight() {
        let s = Status {
            phase: Phase::Climb,
            control: Control::Automation("hover"),
            abnormal: Abnormal::Normal,
            situation: Situation {
                mission: Some("helicopter route"),
                plan: Some(PlanFact {
                    source: "local",
                    origin: "LILB".into(),
                    destination: "LIMC".into(),
                    cruise_alt_ft: 3000.0,
                }),
                active_fix: None,
                ..Situation::default()
            },
            alt_ft: 2028.9,
            ias_kts: 75.8,
            gs_kts: 74.6,
            peak_alt_ft: 0.0,
        };
        let line = status_line(&s);
        assert!(line.starts_with("Climbing through"), "{line}");
        assert!(line.contains("76 knots"), "{line}");
        assert!(line.contains("Helicopter route is flying."), "{line}");
        assert!(line.contains("Plan LILB to LIMC."), "{line}");
        assert!(line.ends_with("No failures."), "{line}");
    }

    /// The replay reads a `crash` record as the flag on the samples after it.
    #[test]
    fn the_replay_reads_the_crash_record() {
        let mut r = Replay::default();
        let state = |t: f64, ground: bool, gs: f64| {
            serde_json::json!({"t": t, "kind": "state", "ground": ground, "gs": gs,
                "ias": gs, "vsi": 0.0, "agl": if ground { 0.0 } else { 3000.0 },
                "alt": 3000.0, "eng": true})
        };
        for i in 0..30 {
            r.feed(&state(i as f64, false, 120.0));
        }
        r.feed(&serde_json::json!({"t": 30.0, "kind": "crash"}));
        let (s, _) = r.feed(&state(31.0, true, 44.0)).unwrap();
        assert_eq!(s.crashed, Some(true));
        assert_eq!(r.flight.abnormal(), &Abnormal::Crashed);
    }

    #[test]
    fn a_bounce_is_counted_once_and_is_not_a_touch_and_go() {
        use Phase::*;
        let mut v: Vec<Sample> = (0..5)
            .map(|t| air(t as f64, 50.0 - t as f64 * 10.0, -500.0, 60.0))
            .collect();
        v.push(ground(5.0, 58.0, true));
        v.push(air(6.0, 8.0, 200.0, 57.0));
        v.push(air(7.0, 6.0, -200.0, 56.0));
        v.extend((8..30).map(|t| ground(t as f64, 50.0 - (t - 8) as f64 * 2.0, true)));
        let (seen, f, ev) = phases(v);
        assert_eq!(seen, vec![Approach, Landing, Rollout, TaxiIn]);
        assert_eq!(f.memory().bounces, 1);
        assert_eq!(f.memory().touch_and_goes, 0);
        assert_eq!(ev.iter().filter(|e| **e == Event::Bounce).count(), 1);
    }

    #[test]
    fn a_touch_and_go_is_a_landing_and_a_takeoff() {
        use Phase::*;
        let mut v: Vec<Sample> = (0..5)
            .map(|t| air(t as f64, 50.0 - t as f64 * 10.0, -500.0, 60.0))
            .collect();
        v.extend((5..15).map(|t| ground(t as f64, 55.0, true)));
        v.extend((15..40).map(|t| air(t as f64, (t - 14) as f64 * 12.0, 700.0, 65.0)));
        let (seen, f, ev) = phases(v);
        assert_eq!(seen, vec![Approach, Landing, Rollout, InitialClimb]);
        assert_eq!(f.memory().landings, 1);
        assert_eq!(f.memory().touch_and_goes, 1);
        assert!(ev.contains(&Event::TouchAndGo));
    }

    #[test]
    fn a_long_bounce_is_a_bounce_not_a_touch_and_go() {
        use Phase::*;
        let mut v: Vec<Sample> = (0..5)
            .map(|t| air(t as f64, 50.0 - t as f64 * 10.0, -500.0, 60.0))
            .collect();
        v.push(ground(5.0, 58.0, true));
        v.extend((6..12).map(|t| air(t as f64, 25.0, -107.0, 55.0)));
        v.extend((12..40).map(|t| ground(t as f64, (50.0 - (t - 12) as f64 * 2.0).max(5.0), true)));
        let (seen, f, _) = phases(v);
        assert_eq!(seen, vec![Approach, Landing, Rollout, TaxiIn]);
        assert_eq!((f.memory().bounces, f.memory().touch_and_goes), (1, 0));
    }

    #[test]
    fn a_stepped_descent_is_one_descent() {
        // Cruise at 9,000, down to 6,000, level 90 s (a leg's constraint),
        // down to 4,000: one descent, not three cruises.
        let mut v: Vec<Sample> = Vec::new();
        let mut t = 0.0;
        let mut alt = 9000.0;
        let mut seg = |n: usize, vsi: f64, v: &mut Vec<Sample>| {
            for _ in 0..n {
                let mut s = air(t, alt - 500.0, vsi, 180.0);
                s.alt_ft = alt;
                v.push(s);
                alt += vsi / 60.0;
                t += 1.0;
            }
        };
        seg(60, 0.0, &mut v);
        seg(180, -1000.0, &mut v);
        seg(90, 0.0, &mut v);
        seg(120, -1000.0, &mut v);
        let (seen, _, _) = phases(v);
        assert_eq!(seen, vec![Phase::Cruise, Phase::Descent]);
    }

    /// EGKK-EGBB 2026-09-28 (sortie 1790624015-full_auto, #174): the 737's
    /// idle descent from the 16,300 ft top saws between -2,000 fpm and level
    /// every 15 s, and every level tick restarted the held needle. Live the
    /// model said Climb to 12,700, Cruise to 9,345, and Descent only at
    /// +1748.9. The height lost over the window is the descent.
    #[test]
    fn a_sawtooth_descent_from_the_top_is_a_descent() {
        const TOD: &str = include_str!("../bench/flight/egkk_egbb_top_of_descent.jsonl");
        let t0 = 1790624015.0;
        let mut r = Replay::default();
        let mut went_down: Option<(f64, f64)> = None;
        let mut after = Vec::new();
        for v in TOD.lines().filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok()) {
            let Some((s, _)) = r.feed(&v) else { continue };
            let live = v["fphase"].as_str().unwrap_or("");
            if went_down.is_none() && r.flight.phase() == Phase::Descent {
                went_down = Some((s.t - t0, s.alt_ft));
                // The recorded failure, on the same tick.
                assert_eq!(live, "Climb", "live at +{:.1}", s.t - t0);
            }
            if went_down.is_some() {
                after.push(r.flight.phase());
            }
        }
        assert!(r.flight.refused().is_empty(), "{:?}", r.flight.refused());
        let (at, alt) = went_down.expect("a descent");
        assert!(at < 1630.0 && alt > 14_500.0, "Descent at +{at:.1}, {alt:.0} ft");
        // The 35 s level-off at 12,700 is a step of it, not a cruise.
        assert!(after.iter().all(|p| *p == Phase::Descent), "{after:?}");
    }

    #[test]
    fn the_sim_settling_a_loaded_aircraft_is_not_a_landing() {
        let mut v: Vec<Sample> = (0..3).map(|t| air(t as f64, 2.0, 0.0, 0.0)).collect();
        v.extend((3..10).map(|t| ground(t as f64, 0.0, false)));
        let (seen, f, ev) = phases(v);
        assert_eq!(seen.last(), Some(&Phase::Parked));
        assert_eq!(f.memory().landings, 0);
        assert!(!ev.iter().any(|e| matches!(e, Event::Touchdown(_))));
    }

    #[test]
    fn a_loading_sim_is_no_evidence() {
        let mut v = vec![air(0.0, -3_533_175.0, 0.0, 0.0), air(1.0, -410.0, 0.0, 0.0)];
        v.extend((2..10).map(|t| ground(t as f64, 0.0, false)));
        let (seen, f, ev) = phases(v);
        assert_eq!(seen, vec![Phase::Parked]);
        assert!(ev.is_empty());
        assert_eq!(f.memory().landings, 0);
    }

    #[test]
    fn a_rejected_takeoff_goes_back_to_the_taxi() {
        use Phase::*;
        let mut v: Vec<Sample> = (0..5).map(|t| ground(t as f64, 0.0, true)).collect();
        v.extend((5..20).map(|t| ground(t as f64, (t - 5) as f64 * 5.0, true)));
        v.extend((20..40).map(|t| ground(t as f64, (70.0 - (t - 20) as f64 * 5.0).max(0.0), true)));
        let (seen, f, ev) = phases(v);
        assert_eq!(
            seen,
            vec![
                Ready,
                TaxiOut,
                TakeoffRoll,
                RejectedTakeoff,
                TaxiOut,
                Holding
            ]
        );
        assert_eq!(f.memory().rejected_takeoffs, 1);
        assert!(ev.contains(&Event::RejectedTakeoff));
        assert!(!ev.contains(&Event::Takeoff));
    }

    #[test]
    fn a_helicopter_lifts_to_a_hover_and_moves_off() {
        use Phase::*;
        let mut v: Vec<Sample> = (0..5).map(|t| ground(t as f64, 0.0, true)).collect();
        v.extend((5..20).map(|t| air(t as f64, 10.0, 50.0, 2.0)));
        v.extend((20..40).map(|t| air(t as f64, 10.0 + (t - 20) as f64 * 8.0, 500.0, 35.0)));
        let (seen, _, ev) = phases(v);
        assert_eq!(seen, vec![Ready, Hover, InitialClimb]);
        assert!(ev.contains(&Event::Takeoff));
    }

    #[test]
    fn a_reposition_restarts_the_track_from_what_the_aircraft_is_doing() {
        let mut a = air(0.0, 5000.0, 0.0, 200.0);
        a.lat = Some(40.0);
        a.lon = Some(-3.0);
        let mut b = ground(1.0, 0.0, true);
        b.lat = Some(51.0);
        b.lon = Some(0.0);
        let mut f = Flight::new();
        f.tick(&a);
        assert_eq!(f.phase(), Phase::Cruise);
        assert!(f.memory().joined_airborne);
        let ev = f.tick(&b);
        assert_eq!(ev, vec![Event::Reposition { to: Phase::Ready }]);
        assert_eq!(f.phase(), Phase::Ready);
        assert!(!f.memory().joined_airborne);
    }

    #[test]
    fn evidence_opens_the_phases_inference_cannot() {
        use Phase::*;
        let mut v = vec![ground(0.0, 0.0, false)];
        let mut s = ground(1.0, 0.0, false);
        s.starter = Some(true);
        v.push(s);
        let mut s = ground(2.0, 0.0, true);
        s.starter = Some(false);
        v.push(s);
        let mut s = ground(3.0, 0.0, true);
        s.tug_attached = Some(true);
        v.push(s);
        let mut s = ground(4.0, 0.0, true);
        s.tug_attached = Some(false);
        v.push(s);
        let (seen, _, _) = phases(v);
        assert_eq!(seen, vec![Parked, EngineStart, Ready, Pushback, Ready]);
    }

    #[test]
    fn an_automation_approach_is_an_approach_at_any_height() {
        let mut s = air(0.0, 4000.0, -800.0, 180.0);
        s.approach = Some(true);
        let mut f = Flight::new();
        f.tick(&air(-1.0, 4000.0, 0.0, 180.0));
        f.tick(&s);
        assert_eq!(f.phase(), Phase::Approach);
    }

    #[test]
    fn control_and_abnormal_are_their_own_tracks() {
        let mut f = Flight::new();
        let mut s = air(0.0, 3000.0, 0.0, 120.0);
        s.autopilot = Some(true);
        f.tick(&s);
        assert_eq!(f.control(), Control::Autopilot);
        let mut s = air(1.0, 3000.0, 0.0, 120.0);
        s.automation = Some("full auto");
        s.failed = Some(vec!["engine 1".into()]);
        let ev = f.tick(&s);
        assert_eq!(f.control(), Control::Automation("full auto"));
        assert_eq!(f.abnormal(), &Abnormal::Failure(vec!["engine 1".into()]));
        assert_eq!(f.phase(), Phase::Cruise);
        assert_eq!(ev.len(), 2);
    }

    #[test]
    fn the_edge_table_is_closed() {
        // Nothing on the ground goes straight to the cruise, nothing
        // airborne goes to the taxi out.
        assert!(!legal(Phase::TaxiOut, Phase::Cruise));
        assert!(!legal(Phase::Cruise, Phase::TaxiOut));
        assert!(!legal(Phase::Parked, Phase::TakeoffRoll));
        assert!(legal(Phase::Cruise, Phase::Rollout));
    }

    /// One recorded flight, every event with the sample that caused it.
    /// `FLIGHT_REPLAY=path cargo test -- --ignored replay_one_verbose --nocapture`
    #[test]
    #[ignore]
    fn replay_one_verbose() {
        let path = std::env::var("FLIGHT_REPLAY").expect("FLIGHT_REPLAY=<jsonl>");
        let text = std::fs::read_to_string(path).expect("readable");
        let mut r = Replay::default();
        for v in text
            .lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
        {
            let Some((s, events)) = r.feed(&v) else {
                continue;
            };
            for e in events {
                println!(
                    "{:.0} gnd={} gs={:.0} ias={:.0} vsi={:.0} agl={:.0} eng={} old={} | {:?}",
                    s.t,
                    s.on_ground,
                    s.gs_kts,
                    s.ias_kts,
                    s.vsi_fpm,
                    s.agl_ft,
                    s.engine_running,
                    v["phase"].as_str().unwrap_or("-"),
                    e
                );
            }
        }
    }

    /// Every recorded flight in the live blackbox, replayed: no illegal
    /// edge, and the phase sequence of each printed for the eye.
    /// `cargo test -- --ignored replay_the_blackbox --nocapture`
    #[test]
    #[ignore]
    fn replay_the_blackbox() {
        use std::io::BufRead;
        let file = std::fs::File::open("logs/blackbox.jsonl").expect("logs/blackbox.jsonl");
        let mut r = Replay::default();
        let mut line = String::new();
        let mut last_t = 0.0;
        let mut trail: Vec<String> = Vec::new();
        let mut refused = Vec::new();
        let flush = |trail: &mut Vec<String>| {
            if trail.len() > 1 {
                println!("{}", trail.join(" > "));
            }
            trail.clear();
        };
        let mut reader = std::io::BufReader::new(file);
        while {
            line.clear();
            reader.read_line(&mut line).unwrap_or(0) > 0
        } {
            if !line.contains("\"kind\":\"state\"") && !line.contains("\"kind\":\"crash\"") {
                continue;
            }
            let Ok(v) = serde_json::from_str::<serde_json::Value>(&line) else {
                continue;
            };
            let t = v.get("t").and_then(|t| t.as_f64()).unwrap_or(last_t);
            if t - last_t > 1800.0 {
                flush(&mut trail);
                refused.extend_from_slice(r.flight.refused());
                r = Replay::default();
            }
            last_t = t;
            let Some((_, events)) = r.feed(&v) else {
                continue;
            };
            for e in events {
                match e {
                    Event::Phase { to, .. } => trail.push(format!("{to:?}")),
                    Event::Reposition { to } => trail.push(format!("[reposition {to:?}]")),
                    Event::GoAround
                    | Event::Bounce
                    | Event::TouchAndGo
                    | Event::RejectedTakeoff => trail.push(format!("({e:?})")),
                    Event::Touchdown(td) => trail.push(format!("(touchdown {:.0})", td.sink_fpm)),
                    Event::Impact(td) => trail.push(format!("(IMPACT {:.0})", td.sink_fpm)),
                    _ => {}
                }
            }
            if trail.is_empty() {
                trail.push(format!("{:?}", r.flight.phase()));
            }
        }
        flush(&mut trail);
        refused.extend_from_slice(r.flight.refused());
        assert!(refused.is_empty(), "illegal edges: {refused:?}");
    }

    /// LEMG 13, 2026-09-26 (sortie 1790441293): full auto lined up, stopped
    /// on the runway, and the takeoff rolled. The `state` records' ground
    /// speed from the taxi's last metres to past the roll speed. The live
    /// monitor never fed `on_runway`, so the model read the stop as
    /// "Holding" and the roll as "TaxiOut" for 11 s, and the duty engine
    /// asked for a line-up and then a taxi under the running takeoff. With
    /// the runway fed the same ticks read LinedUp, then the roll.
    #[test]
    fn a_lined_up_stop_and_its_roll_with_the_runway_fed() {
        let gs = [
            (-24.5, 9.8), (-23.2, 10.6), (-21.8, 11.2), (-20.3, 11.0), (-18.9, 11.0),
            (-17.5, 10.1), (-16.1, 9.7), (-14.8, 8.9), (-13.4, 7.0), (-12.1, 6.6),
            (-10.8, 2.6), (-9.4, 2.5), (-8.1, 1.6), (-6.7, 1.5), (-5.3, 0.1),
            (-3.9, 0.0), (-2.5, 0.0), (-1.2, 0.0), (0.2, 0.0), (1.7, 0.0),
            (3.0, 0.0), (4.4, 0.2), (5.7, 1.2), (7.0, 4.5), (8.5, 10.5),
            (9.8, 19.2), (11.1, 27.7), (12.4, 35.6), (13.7, 44.1), (14.9, 54.2),
            (16.2, 64.0), (17.5, 74.0),
        ];
        let run = |runway: Option<bool>| {
            // Taxiing out from the start, then the recorded ticks. The last
            // 20 m of the taxi were the line-up leg, on the runway.
            let mut v: Vec<Sample> = (0..30)
                .map(|i| Sample { on_runway: Some(false), ..ground(-60.0 + i as f64, 10.0, true) })
                .collect();
            v.extend(gs.iter().map(|&(t, g)| Sample { on_runway: runway, ..ground(t, g, true) }));
            phases(v).0
        };
        let blind = run(None);
        assert!(
            blind.ends_with(&[Phase::Holding, Phase::TaxiOut, Phase::TakeoffRoll]),
            "the recorded failure: {blind:?}"
        );
        let fed = run(Some(true));
        assert!(
            fed.ends_with(&[Phase::TaxiOut, Phase::LinedUp, Phase::TakeoffRoll]),
            "{fed:?}"
        );
    }
}
