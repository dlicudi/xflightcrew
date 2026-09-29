//! The pilot's go-around order: the crew side of "go around".
//!
//! The tower's order lives on the party line (`party_line::GoAroundOrders`,
//! the world's). The pilot's is the crew's own and never touches the atc
//! crate: a counter with the same watch semantics, so a loop flying an
//! approach sees each order once, whoever else is watching, and an order
//! given before the loop armed is history. Both are read through ONE watch
//! ([`GoAroundWatch`]), so autoland (`GoAroundLatch::order`) and the circuit
//! (`circuit::flies_go_around_order`) fly a pilot's go-around exactly as they
//! fly the tower's: the same climb-out, clean-up, pattern and "going around"
//! call to the tower.
//!
//! The loop that took an order answers it ([`answer`]), so the `go_around`
//! tool reads back what the aircraft is doing, never what it was asked.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

/// Every go-around the pilot has ordered, counted and never reset.
static PILOT_ORDERS: AtomicU64 = AtomicU64::new(0);

/// The last order a loop answered, and how.
static ANSWER: Mutex<Option<(u64, Answer)>> = Mutex::new(None);

/// When the main loop last saw the TOWER order a go-around: the pilot's
/// "going around" on the radio inside [`READBACK_WINDOW`] of it is the
/// readback, which the crew is already flying.
static LAST_TOWER_ORDER: Mutex<Option<Instant>> = Mutex::new(None);

/// A pilot's "going around" this soon after the tower's order reads it back.
pub const READBACK_WINDOW: Duration = Duration::from_secs(60);

/// How long the tool waits for the loop's answer before reading back.
/// A loop ticks at 2.5 to 5 Hz; the circuit's arrival lookup can take longer,
/// and then the order is owed and flown on its next tick.
pub const ANSWER_WAIT: Duration = Duration::from_millis(2500);

/// Who ordered the go-around a loop is about to fly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoAroundBy {
    Tower,
    Pilot,
}

impl GoAroundBy {
    /// For the blackbox.
    pub fn label(self) -> &'static str {
        match self {
            GoAroundBy::Tower => "tower",
            GoAroundBy::Pilot => "pilot",
        }
    }
}

/// How a loop took a pilot's order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Answer {
    /// The missed approach is being flown from this tick.
    Flying,
    /// A missed approach was already being flown.
    Already,
    /// The wheels are on the runway: a landing, never abandoned.
    OnGround,
    /// The circuit is on a leg with no landing to go around from.
    NotOnApproach,
}

/// The pilot orders a go-around. Returns the order's number, for [`wait_answer`].
pub fn order() -> u64 {
    let n = PILOT_ORDERS.fetch_add(1, Ordering::SeqCst) + 1;
    crate::blackbox::record("go_around", serde_json::json!({"event": "ordered", "n": n}));
    n
}

/// A loop's answer to the pilot's latest order. A `Flying` answer is never
/// overwritten by a lesser one for the same order (autoland flying it while
/// the circuit, handed over, has nothing to go around from).
pub fn answer(a: Answer) {
    let n = PILOT_ORDERS.load(Ordering::SeqCst);
    let mut slot = ANSWER.lock().unwrap_or_else(|e| e.into_inner());
    if matches!(*slot, Some((m, Answer::Flying)) if m == n) {
        return;
    }
    *slot = Some((n, a));
}

/// The answer to order `n`, if a loop has given one. Pure read.
pub fn answer_to(n: u64) -> Option<Answer> {
    match *ANSWER.lock().unwrap_or_else(|e| e.into_inner()) {
        Some((m, a)) if m >= n => Some(a),
        _ => None,
    }
}

/// Wait up to [`ANSWER_WAIT`] for a loop to answer order `n`.
pub async fn wait_answer(n: u64) -> Option<Answer> {
    let t0 = Instant::now();
    loop {
        if let Some(a) = answer_to(n) {
            return Some(a);
        }
        if t0.elapsed() >= ANSWER_WAIT {
            return None;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

/// One crew loop's watch on every go-around order, the tower's and the
/// pilot's. Armed when the loop starts flying (every order before is
/// history), asked each tick; each watch sees each order once, and the
/// tower's is read through its own party-line watch, which takes nothing
/// from the main loop's FSM sync.
#[derive(Debug, Clone, Copy)]
pub struct GoAroundWatch {
    tower: crate::party_line::GoAroundOrders,
    pilot_seen: u64,
}

impl GoAroundWatch {
    /// Start watching: only orders given from now on count.
    pub fn arm() -> Self {
        Self {
            tower: crate::party_line::GoAroundOrders::arm(),
            pilot_seen: PILOT_ORDERS.load(Ordering::SeqCst),
        }
    }

    /// A go-around ordered since the last look, and by whom: both are
    /// consumed, the tower's named when both came (it can see the runway).
    pub fn take(&mut self) -> Option<GoAroundBy> {
        let tower = self.tower.take();
        let now = PILOT_ORDERS.load(Ordering::SeqCst);
        let pilot = now != self.pilot_seen;
        self.pilot_seen = now;
        if tower {
            Some(GoAroundBy::Tower)
        } else if pilot {
            Some(GoAroundBy::Pilot)
        } else {
            None
        }
    }
}

/// Why the `go_around` tool refuses before ordering anything, from what is
/// flying and whether the wheels are down. Pure.
pub fn refusal(autoland: bool, circuit: bool, on_ground: bool) -> Option<&'static str> {
    if on_ground {
        return Some(if autoland {
            "We're down, staying down."
        } else {
            "We're on the ground. Nothing to go around from."
        });
    }
    if !autoland && !circuit {
        return Some("I'm not flying an approach. Your go-around: power up, pitch up, clean up.");
    }
    None
}

/// The tool's readback of a loop's answer (`None`: nobody answered in time). Pure.
pub fn readback(a: Option<Answer>) -> &'static str {
    match a {
        Some(Answer::Flying) => "Going around.",
        Some(Answer::Already) => "Already going around.",
        Some(Answer::OnGround) => "We're down, staying down.",
        Some(Answer::NotOnApproach) => "Nothing to go around from on this leg.",
        None => "Go-around ordered, the approach has not taken it yet.",
    }
}

/// The main loop saw the tower order a go-around.
pub fn note_tower_order() {
    *LAST_TOWER_ORDER.lock().unwrap_or_else(|e| e.into_inner()) = Some(Instant::now());
}

/// Whether a pilot's radio words are their OWN go-around: "going around" or
/// "missed approach" on the air (the tower's `Intent::GoAround`), not inside
/// [`READBACK_WINDOW`] of the tower's order, which the crew already flies.
/// Pure over `since_tower_order`.
pub fn radio_call_is_order(transcript: &str, since_tower_order: Option<Duration>) -> bool {
    let t = transcript.to_ascii_lowercase();
    crate::tower::parse_intent(&t) == crate::tower::Intent::GoAround
        && since_tower_order.is_none_or(|d| d >= READBACK_WINDOW)
}

/// A pilot's radio call heard (keyed, or content-routed to the tower): their
/// own go-around is ordered for the crew too, when a loop flies an approach.
/// The tower answers the call; the loop's intercom line says the rest.
pub fn note_radio_call(transcript: &str) {
    let since = LAST_TOWER_ORDER
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .map(|t| t.elapsed());
    if !radio_call_is_order(transcript, since) {
        return;
    }
    let flying = crate::autoland::ACTIVE.load(Ordering::SeqCst)
        || crate::circuit::ACTIVE.load(Ordering::SeqCst);
    if flying && crate::mode::crew_actuates() {
        let n = order();
        crate::blackbox::record(
            "go_around",
            serde_json::json!({"event": "radio_call", "n": n, "transcript": transcript}),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The pilot counter is process-wide; these tests post to it.
    static LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn a_watch_sees_a_pilot_order_once_and_armed_later_never() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut circuit = GoAroundWatch::arm();
        let mut autoland = GoAroundWatch::arm();
        assert_eq!(circuit.take(), None);
        order();
        assert_eq!(circuit.take(), Some(GoAroundBy::Pilot));
        assert_eq!(circuit.take(), None, "once");
        assert_eq!(
            autoland.take(),
            Some(GoAroundBy::Pilot),
            "a second reader is not robbed"
        );
        let mut later = GoAroundWatch::arm();
        assert_eq!(later.take(), None, "an order before the arm is history");
    }

    #[test]
    fn autoland_flies_a_pilot_order_as_it_flies_the_towers() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut watch = GoAroundWatch::arm();
        let mut latch = crate::autoland::GoAroundLatch::new();
        order();
        // The shape of autoland's tick: the watch, then the latch.
        let by = watch.take().expect("the pilot's order");
        assert_eq!(by, GoAroundBy::Pilot);
        assert!(latch.order(false), "flown from wherever we are");
        assert!(latch.active());
        assert_eq!(latch.attempts(), 1);
        // A second order while it flies is the same go-around.
        order();
        assert!(watch.take().is_some());
        assert!(!latch.order(false));
        assert_eq!(latch.attempts(), 1);
        // On the wheels an order is a landing, never abandoned.
        let mut down = crate::autoland::GoAroundLatch::new();
        assert!(!down.order(true));
        assert!(!down.active());
    }

    #[test]
    fn the_circuit_flies_a_pilot_order_on_the_approach_legs_only() {
        use crate::circuit::{flies_go_around_order, Leg};
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut watch = GoAroundWatch::arm();
        order();
        assert_eq!(watch.take(), Some(GoAroundBy::Pilot));
        assert!(flies_go_around_order(Leg::Final));
        assert!(flies_go_around_order(Leg::Base));
        assert!(!flies_go_around_order(Leg::Upwind));
        // The intercom names whose go-around it is.
        assert_eq!(
            crate::circuit::go_around_line(GoAroundBy::Pilot),
            "Going around."
        );
        assert!(crate::circuit::go_around_line(GoAroundBy::Tower).contains("Tower's"));
    }

    #[test]
    fn a_flying_answer_stands_and_the_tool_reads_it_back() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let n = order();
        assert_eq!(answer_to(n), None, "no loop has answered this one");
        answer(Answer::Flying);
        answer(Answer::NotOnApproach);
        assert_eq!(
            answer_to(n),
            Some(Answer::Flying),
            "autoland's answer stands"
        );
        assert_eq!(readback(answer_to(n)), "Going around.");
        let m = order();
        assert_eq!(answer_to(m), None, "a new order waits for its own answer");
        answer(Answer::OnGround);
        assert_eq!(readback(answer_to(m)), "We're down, staying down.");
        assert!(readback(None).contains("not taken"));
    }

    #[test]
    fn the_gate_refuses_without_an_approach_and_after_touchdown() {
        assert!(refusal(false, false, false).is_some_and(|r| r.contains("not flying")));
        assert_eq!(
            refusal(true, false, true),
            Some("We're down, staying down.")
        );
        assert!(refusal(false, true, true).is_some_and(|r| r.contains("on the ground")));
        assert_eq!(refusal(true, false, false), None);
        assert_eq!(refusal(false, true, false), None);
    }

    #[test]
    fn a_radio_going_around_is_an_order_unless_it_reads_back_the_tower() {
        let call = "gibraltar tower, november one two three, going around";
        assert!(radio_call_is_order(call, None));
        assert!(radio_call_is_order("missed approach", None));
        assert!(
            !radio_call_is_order(call, Some(Duration::from_secs(4))),
            "the readback"
        );
        assert!(radio_call_is_order(call, Some(Duration::from_secs(120))));
        assert!(!radio_call_is_order("request taxi", None));
    }
}
