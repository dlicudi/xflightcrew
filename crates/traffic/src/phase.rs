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
    /// leader is [`crate::aircraft::FOLLOW_FINAL_NM`] on final.
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
}

impl Call {
    /// Plain English, addressed to the tower: "Tower, Cessna three four
    /// five, ready for departure runway one three". `runway` is the
    /// aircraft's runway where the call names one.
    pub fn text(&self, spoken: &str, runway: Option<&str>) -> String {
        let rwy = runway
            .map(|r| format!(" runway {}", runway_words(r)))
            .unwrap_or_default();
        match self {
            Call::Taxi => format!("Tower, {spoken}, request taxi"),
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

const DIGITS: [&str; 10] = [
    "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "niner",
];

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
    /// The last readback given, so the same owed text is answered once.
    pub last_readback: Option<String>,
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
}
