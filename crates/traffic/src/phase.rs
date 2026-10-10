//! The phase machine's vocabulary: what a traffic aircraft is doing
//! ([`Phase`]), what it means to do ([`Plan`]), what the tower lets it do
//! ([`Order`]), what it says ([`Call`]) and what the fleet reports
//! ([`Event`]).

use crate::kin::LatLon;

/// The tower's view of one traffic strip, mirroring `atc::tower::State`
/// (this crate never depends on atc; the bin maps one onto the other).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TowerState {
    #[default]
    Parked,
    TaxiCleared,
    LinedUp,
    TakeoffCleared,
    Inbound,
    ClearedToLand,
}

/// What the tower has cleared one traffic aircraft to do, read
/// level-triggered every substep (no edge queue to drop).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Order {
    pub state: TowerState,
    /// The runway assigned; the plan's when `None`.
    pub runway: Option<String>,
    /// The cleared route; the aircraft plans its own on the network when
    /// `None`.
    pub taxi_path: Option<Vec<LatLon>>,
    pub holding_point: Option<String>,
    /// Number two to this traffic id: the downwind is extended until the
    /// leader will be off the runway before we reach it.
    pub follow: Option<u32>,
    /// "Hold position": stop where you are, and no "ready" call yet.
    pub hold: bool,
    /// "Go around": at once, from a final or a landing.
    pub go_around: bool,
    /// A readback the tower is owed, answered verbatim once.
    pub readback_owed: Option<String>,
    /// The runway the tower has cleared this aircraft to cross ("cross
    /// runway one three"): the taxi goes on past the stop short of it.
    pub cross_cleared: Option<String>,
    /// Ground has approved the push ("push and start approved, facing
    /// west").
    pub push_approved: bool,
    /// The way Ground said the nose will face once pushed, degrees true;
    /// `None` with no facing given (the aircraft picks its own).
    pub push_facing: Option<f64>,
}

/// What a traffic aircraft is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Phase {
    Parked,
    TaxiOut,
    HoldShort,
    LineUp,
    Roll,
    /// The initial climb on the runway heading.
    Climb,
    /// A departure leaving the field on its exit bearing.
    Leave,
    /// An arrival flying from its entry point to the circuit or the final.
    Arrive,
    Pattern {
        leg: Leg,
    },
    Final,
    /// The flare, touchdown and rollout.
    Landing,
    Vacate,
    TaxiIn,
    GoAround,
    Gone,
}

impl Phase {
    /// Flying (not on the ground, not gone).
    pub fn airborne(self) -> bool {
        matches!(
            self,
            Phase::Climb
                | Phase::Leave
                | Phase::Arrive
                | Phase::Pattern { .. }
                | Phase::Final
                | Phase::GoAround
        )
    }
}

/// One leg of a route: what each segment of the path an aircraft follows
/// is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Leg {
    Taxi,
    /// The takeoff run or the landing roll.
    Runway,
    Upwind,
    Crosswind,
    Downwind,
    Base,
    Final,
    /// From an arrival's entry point.
    Inbound,
    /// A departure's way out.
    Outbound,
}

/// What a traffic aircraft is out to do.
#[derive(Debug, Clone, PartialEq)]
pub enum Plan {
    /// Circuits from a stand: `laps` of them, touch and go between, the
    /// last to a full stop and back to a stand.
    Circuit {
        runway: String,
        laps: u32,
        left: bool,
    },
    /// Taxi, take off, leave on `exit_bearing` (degrees true); gone 12 nm
    /// out.
    Departure {
        runway: String,
        sid: Option<String>,
        exit_bearing: f64,
    },
    /// From `entry`, a straight in or a join to the downwind, land, taxi
    /// to a stand.
    Arrival {
        entry: LatLon,
        star: Option<String>,
        runway: String,
        straight_in: bool,
    },
    /// Stays on its stand (drawn only).
    Parked,
}

impl Plan {
    pub fn runway(&self) -> Option<&str> {
        match self {
            Plan::Circuit { runway, .. }
            | Plan::Departure { runway, .. }
            | Plan::Arrival { runway, .. } => Some(runway),
            Plan::Parked => None,
        }
    }
}

/// A radio call a traffic aircraft makes. Rendered to words by
/// [`Call::text`]; the field's dialect is the tower's business.
#[derive(Debug, Clone, PartialEq)]
pub enum Call {
    /// Request taxi (out to the runway, or in to parking after a landing).
    Taxi,
    /// On a nose-in stand, to Ground: "request push and start". `stand` is
    /// the stand's name as drawn. Vocabulary only for now: the fleet's push
    /// is owed (branch `wip-ground-push`).
    PushStart {
        stand: Option<String>,
    },
    /// Ready for departure at the holding point.
    Ready,
    /// Inbound for landing, `miles` out, `from` a compass point.
    Inbound {
        miles: u32,
        from: String,
    },
    Downwind,
    Base,
    Final,
    Vacated,
    GoingAround,
    /// Holding short of a runway the taxi route crosses, asking to cross.
    RequestCrossing {
        runway: String,
    },
    /// The owed readback, verbatim.
    Readback(String),
    /// "Say again": the readback was not taken twice, the tower is asked to
    /// repeat once before the flight gives up.
    SayAgain,
    /// An instrument arrival's first call, to the APPROACH unit, at the
    /// level it is descending to (feet, the thousands said).
    ArrivalCheckIn {
        alt_ft: u32,
    },
    /// Off a missed approach, to the APPROACH unit, climbing to its
    /// missed approach altitude.
    MissedCheckIn {
        alt_ft: u32,
    },
    /// Established on the final approach: to the approach unit (which
    /// hands it to the tower) or, `miles` out, to the tower (which clears
    /// it). `how` is the approach flown ("ILS", "RNP approach"), empty for
    /// a final with no procedure.
    Established {
        approach: bool,
        how: String,
        miles: u32,
    },
}

impl Call {
    /// The call is made to the approach unit, not the tower: it opens with
    /// "Approach, ", which the bin addresses by the field's name.
    pub fn to_approach(&self) -> bool {
        match self {
            Call::ArrivalCheckIn { .. } | Call::MissedCheckIn { .. } => true,
            Call::Established { approach, .. } => *approach,
            _ => false,
        }
    }

    /// The call is GROUND's business where the field has Ground: the push,
    /// the taxi out and, off the runway, the taxi in (the station routes
    /// each call to the position that owns it; this is the frequency it is
    /// paced on).
    pub fn to_ground(&self) -> bool {
        matches!(self, Call::Taxi | Call::PushStart { .. } | Call::Vacated)
    }

    /// Plain English, addressed to the tower: "Tower, Cessna three four
    /// five, ready for departure runway one three". `runway` is the
    /// aircraft's runway where the call names one.
    pub fn text(&self, spoken: &str, runway: Option<&str>) -> String {
        let rwy = runway
            .map(|r| format!(" runway {}", runway_words(r)))
            .unwrap_or_default();
        match self {
            Call::Taxi => format!("Tower, {spoken}, request taxi"),
            Call::PushStart { stand } => match stand.as_deref().map(stand_words) {
                Some(s) if !s.is_empty() => {
                    format!("Ground, {spoken}, stand {s}, request push and start")
                }
                _ => format!("Ground, {spoken}, request push and start"),
            },
            Call::Ready => format!("Tower, {spoken}, ready for departure{rwy}"),
            Call::Inbound { miles, from } => format!(
                "Tower, {spoken}, {} miles {from}, inbound for landing",
                number_words(*miles)
            ),
            Call::Downwind => format!("{spoken}, downwind{rwy}"),
            // "base" alone is too common a word for the tower's parser: the
            // pattern report is "base leg".
            Call::Base => format!("{spoken}, base leg{rwy}"),
            Call::Final => format!("{spoken}, final{rwy}"),
            // Off the runway and asking for the stand in one call, in the
            // tower's own suggested wording.
            Call::Vacated => format!("{spoken}, runway vacated, request taxi to parking"),
            Call::GoingAround => format!("{spoken}, going around"),
            Call::SayAgain => format!("Tower, {spoken}, say again"),
            // CAP 413 6.4: the check-in names the level; "inbound" asks for
            // the approach.
            Call::ArrivalCheckIn { alt_ft } => format!(
                "Approach, {spoken}, descending altitude {} feet, inbound",
                crate::arrival::alt_words(*alt_ft as f64)
            ),
            Call::MissedCheckIn { alt_ft } => format!(
                "Approach, {spoken}, missed approach, climbing altitude {} feet",
                crate::arrival::alt_words(*alt_ft as f64)
            ),
            Call::Established {
                approach: true,
                how,
                ..
            } if how.is_empty() => format!("Approach, {spoken}, established on final{rwy}"),
            Call::Established {
                approach: true,
                how,
                ..
            } => format!("Approach, {spoken}, established {how}{rwy}"),
            Call::Established { how, miles, .. } if how.is_empty() => format!(
                "Tower, {spoken}, long final{rwy}, {} miles",
                number_words(*miles)
            ),
            Call::Established { how, miles, .. } => format!(
                "Tower, {spoken}, established {how}{rwy}, {} miles",
                number_words(*miles)
            ),
            Call::RequestCrossing { runway } => format!(
                "Tower, {spoken}, holding short runway {}, request crossing",
                runway_words(runway)
            ),
            // The tower's phrasing already names the flight: said as it is.
            Call::Readback(text) if text.contains(spoken) => text.trim_end_matches('.').to_string(),
            Call::Readback(text) => format!("{text}, {spoken}"),
        }
    }
}

/// A stand's name as said: "Stand 52" and "Gate 52" as "five two",
/// "A7" as "A seven".
pub fn stand_words(name: &str) -> String {
    let bare = name.trim();
    let bare = ["stand ", "gate ", "parking "]
        .iter()
        .find_map(|p| {
            bare.get(..p.len())
                .filter(|h| h.eq_ignore_ascii_case(p))
                .map(|_| &bare[p.len()..])
        })
        .unwrap_or(bare);
    let mut out: Vec<String> = Vec::new();
    for c in bare.chars() {
        match c.to_digit(10) {
            Some(d) => out.push(DIGITS[d as usize].to_string()),
            None if c.is_alphabetic() => out.push(c.to_ascii_uppercase().to_string()),
            None => {}
        }
    }
    out.join(" ")
}

const DIGITS: [&str; 10] = [
    "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "niner",
];

/// One digit as said on the radio ("niner").
pub fn digit_word(d: u32) -> &'static str {
    DIGITS[(d % 10) as usize]
}

/// "13" as "one three", "09L" as "zero niner left".
pub fn runway_words(ident: &str) -> String {
    let mut words: Vec<String> = ident
        .chars()
        .filter_map(|c| c.to_digit(10).map(|d| DIGITS[d as usize].to_string()))
        .collect();
    match ident.chars().last().map(|c| c.to_ascii_uppercase()) {
        Some('L') => words.push("left".into()),
        Some('R') => words.push("right".into()),
        Some('C') => words.push("centre".into()),
        _ => {}
    }
    words.join(" ")
}

/// A small count in words ("ten"); digits past twenty.
pub fn number_words(n: u32) -> String {
    const SMALL: [&str; 21] = [
        "zero",
        "one",
        "two",
        "three",
        "four",
        "five",
        "six",
        "seven",
        "eight",
        "nine",
        "ten",
        "eleven",
        "twelve",
        "thirteen",
        "fourteen",
        "fifteen",
        "sixteen",
        "seventeen",
        "eighteen",
        "nineteen",
        "twenty",
    ];
    SMALL
        .get(n as usize)
        .map(|s| s.to_string())
        .unwrap_or_else(|| n.to_string())
}

/// What the fleet reports from a step.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    Call {
        id: u32,
        call: Call,
    },
    Spawn {
        id: u32,
    },
    Despawn {
        id: u32,
        reason: Despawn,
    },
    /// A phase change (for the strip's picture, the blackbox and tests).
    Phase {
        id: u32,
        from: Phase,
        to: Phase,
    },
    /// Main wheels on: where, and how far down the runway.
    Touchdown {
        id: u32,
        at: LatLon,
        along_m: f64,
    },
    /// Held [`crate::spacing::STUCK_S`] on the ground behind an aircraft
    /// that will not move for it (`around`, or our own when `None`), it
    /// taxis another way round it ([`crate::spacing::apply`]).
    SpacingRerouted {
        id: u32,
        around: Option<u32>,
    },
    /// ...and with no other way, the traffic aircraft in its way was towed
    /// clear (`removed`, gone with [`Despawn::Towed`]).
    SpacingCleared {
        id: u32,
        removed: u32,
    },
    /// ...and with no other way and nothing to tow (our own aircraft, or one
    /// that may yet move), it waits where it is: said once per blocker.
    SpacingHeld {
        id: u32,
        behind: Option<u32>,
    },
}

/// Why an aircraft left the fleet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Despawn {
    /// A departure past [`crate::aircraft::DEPART_GONE_NM`].
    LeftTheArea,
    /// Parked long enough after its flight.
    Parked,
    /// Taken out by the caller (a reinstall, a schedule change).
    Removed,
    /// Landed with nowhere to park, after waiting clear of the runway.
    NoStand,
    /// Its readback was not taken twice and the tower's repeat did not
    /// help: it leaves the frequency rather than be challenged forever.
    ReadbackFailed,
    /// Towed off the taxiway: it stood in the way of another with no way
    /// round it ([`Event::SpacingCleared`]).
    Towed,
}

/// Our own aircraft, as the fleet needs it: where not to spawn.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OwnTarget {
    pub pos: LatLon,
    pub alt_ft: f64,
    pub on_ground: bool,
}

/// The radio half of one traffic aircraft: what it has said, so a call is
/// made once.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RadioState {
    /// The readback owed now and last given, so an owed text is answered
    /// once per owing (forgotten once nothing is owed: the next lap's
    /// identical clearance is read back again).
    pub last_readback: Option<String>,
    /// Times the owed readback has been given.
    pub readback_tries: u32,
    /// "Say again" has been asked for the owed readback.
    pub said_again: bool,
    /// When the owed readback (or the "say again") last went out, the
    /// aircraft's own seconds.
    pub readback_at_s: f64,
    /// The phase's call has gone out (reset on every phase change).
    pub called: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calls_render_in_plain_english() {
        assert_eq!(
            Call::Ready.text("Cessna three four five", Some("13")),
            "Tower, Cessna three four five, ready for departure runway one three"
        );
        assert_eq!(runway_words("09L"), "zero niner left");
        assert_eq!(
            Call::Inbound {
                miles: 10,
                from: "north".into()
            }
            .text("Speedbird one", None),
            "Tower, Speedbird one, ten miles north, inbound for landing"
        );
        assert_eq!(
            Call::Readback("Cleared to land runway two seven".into()).text("Cessna one", None),
            "Cleared to land runway two seven, Cessna one"
        );
        assert_eq!(
            Call::Readback("Cleared to land runway two seven, Cessna one.".into())
                .text("Cessna one", None),
            "Cleared to land runway two seven, Cessna one",
            "a readback already naming the flight is said as it is"
        );
        assert_eq!(
            Call::RequestCrossing {
                runway: "13".into()
            }
            .text("Cessna one", None),
            "Tower, Cessna one, holding short runway one three, request crossing"
        );
        assert_eq!(
            Call::Base.text("Cessna one", Some("27")),
            "Cessna one, base leg runway two seven"
        );
    }

    /// An instrument arrival's calls (CAP 413 ch. 6): the check-in and the
    /// established report to the approach unit, then to the tower with
    /// its distance.
    #[test]
    fn instrument_arrival_calls_render_in_plain_english() {
        let s = "Speedbird one two three";
        let check_in = Call::ArrivalCheckIn { alt_ft: 9600 };
        assert!(check_in.to_approach());
        assert_eq!(
            check_in.text(s, Some("26L")),
            "Approach, Speedbird one two three, descending altitude nine thousand feet, inbound"
        );
        let est = |approach| Call::Established {
            approach,
            how: "ILS".into(),
            miles: 9,
        };
        assert_eq!(
            est(true).text(s, Some("26L")),
            "Approach, Speedbird one two three, established ILS runway two six left"
        );
        assert!(!est(false).to_approach());
        assert_eq!(
            est(false).text(s, Some("26L")),
            "Tower, Speedbird one two three, established ILS runway two six left, nine miles"
        );
        assert_eq!(
            Call::MissedCheckIn { alt_ft: 3000 }.text(s, None),
            "Approach, Speedbird one two three, missed approach, climbing altitude three thousand feet"
        );
    }
}
