//! Tower service: a deterministic clearance state machine plus phraseology.
//!
//! The `atc` binary owns the radio I/O (PTT capture, whisper, TTS); everything
//! here is pure — transcript in, spoken reply and state transition out — so
//! the dialogue logic unit-tests without a simulator or a microphone. Intents
//! are keyword-matched, not LLM-picked, in line with the deterministic-first
//! rule: a misheard call gets "say again", never a guessed clearance.

/// Whisper decoding bias for radio calls (see [`crate::stt::Stt::set_prompt`]).
pub const STT_PROMPT: &str = "Radio calls to the tower: request taxi, request \
departure clearance, ready for departure, request takeoff, radio check, \
holding point, runway zero niner, runway two seven. Numbers are spoken digit \
by digit.";

/// Where the aircraft is in the clearance sequence. Deliberately minimal for
/// the first slice — taxi and departure only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Parked,
    TaxiCleared,
    TakeoffCleared,
}

/// What the pilot asked for, keyword-matched from the transcript.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Intent {
    RadioCheck,
    RequestTaxi,
    RequestClearance,
    RequestTakeoff,
    Unknown,
}

/// Everything a reply is phrased from. The binary rebuilds this per call with
/// live wind/QNH, so a clearance always carries current numbers.
#[derive(Debug, Clone)]
pub struct Ctx {
    /// Spoken callsign ("November one seven two Sierra Papa").
    pub callsign: String,
    /// Spoken station name ("Gibraltar").
    pub airport: String,
    /// Runway in use, spoken ("zero niner").
    pub runway: String,
    /// QNH, spoken digits ("one zero one seven").
    pub qnh: String,
    /// Wind, spoken ("zero niner zero degrees, three knots" or "calm").
    pub wind: String,
    /// Holding point name ("Alpha").
    pub holding_point: String,
}

pub fn parse_intent(transcript: &str) -> Intent {
    let t = transcript.to_lowercase();
    if t.contains("radio check") || t.contains("how do you read") {
        return Intent::RadioCheck;
    }
    // Takeoff before taxi: "ready for departure" / "request takeoff" must not
    // fall through to a fresh taxi clearance.
    if t.contains("takeoff")
        || t.contains("take off")
        || t.contains("take-off")
        || (t.contains("ready") && t.contains("departure"))
    {
        return Intent::RequestTakeoff;
    }
    if t.contains("clearance") {
        return Intent::RequestClearance;
    }
    if t.contains("taxi") {
        return Intent::RequestTaxi;
    }
    Intent::Unknown
}

/// The tower's side of the dialogue: one aircraft (the player), one runway.
pub struct Tower {
    pub state: State,
}

impl Tower {
    pub fn new() -> Self {
        Tower { state: State::Parked }
    }

    /// Answer one transmission, advancing the clearance state.
    pub fn handle(&mut self, intent: Intent, ctx: &Ctx) -> String {
        match intent {
            Intent::RadioCheck => {
                format!("{}, {} Tower, reading you five.", ctx.callsign, ctx.airport)
            }
            Intent::RequestClearance => format!(
                "{}, cleared to leave the {} zone, not above two thousand feet, \
                 QNH {}, squawk seven zero zero zero.",
                ctx.callsign, ctx.airport, ctx.qnh
            ),
            Intent::RequestTaxi => {
                self.state = State::TaxiCleared;
                format!(
                    "{}, taxi to holding point {} runway {}, QNH {}. \
                     Report ready for departure.",
                    ctx.callsign, ctx.holding_point, ctx.runway, ctx.qnh
                )
            }
            Intent::RequestTakeoff => match self.state {
                // A takeoff request from the apron gets the taxi first — the
                // tower never clears a takeoff from a parking stand.
                State::Parked => {
                    self.state = State::TaxiCleared;
                    format!(
                        "{}, taxi to holding point {} runway {} first, QNH {}. \
                         Report ready for departure.",
                        ctx.callsign, ctx.holding_point, ctx.runway, ctx.qnh
                    )
                }
                State::TaxiCleared | State::TakeoffCleared => {
                    self.state = State::TakeoffCleared;
                    format!(
                        "{}, runway {}, wind {}, cleared for takeoff.",
                        ctx.callsign, ctx.runway, ctx.wind
                    )
                }
            },
            Intent::Unknown => {
                format!("{}, {} Tower, say again.", ctx.callsign, ctx.airport)
            }
        }
    }
}

/// A tail number spoken ICAO-style: letters as NATO words, digits read out.
/// "N172SP" -> "November one seven two Sierra Papa".
pub fn spoken_callsign(tail: &str) -> String {
    tail.chars()
        .filter_map(|c| match c {
            'A'..='Z' => Some(crate::atis::nato(c).to_string()),
            'a'..='z' => Some(crate::atis::nato(c.to_ascii_uppercase()).to_string()),
            '0'..='9' => Some(crate::atis::spoken_digits(&c.to_string())),
            _ => None, // drop dashes and anything else silently
        })
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx() -> Ctx {
        Ctx {
            callsign: "November one seven two Sierra Papa".into(),
            airport: "Gibraltar".into(),
            runway: "zero niner".into(),
            qnh: "one zero one seven".into(),
            wind: "zero niner zero degrees, three knots".into(),
            holding_point: "Alpha".into(),
        }
    }

    #[test]
    fn intents_parse_from_natural_calls() {
        assert_eq!(parse_intent("Gibraltar Tower, November 172SP, request taxi"), Intent::RequestTaxi);
        assert_eq!(parse_intent("ready for departure runway 09"), Intent::RequestTakeoff);
        assert_eq!(parse_intent("request takeoff"), Intent::RequestTakeoff);
        assert_eq!(parse_intent("request departure clearance"), Intent::RequestClearance);
        assert_eq!(parse_intent("radio check"), Intent::RadioCheck);
        assert_eq!(parse_intent("good morning"), Intent::Unknown);
        // "taxi" inside a takeoff call must not read as a taxi request.
        assert_eq!(parse_intent("holding point taxi complete, ready for departure"), Intent::RequestTakeoff);
    }

    #[test]
    fn taxi_then_takeoff_is_the_happy_path() {
        let mut twr = Tower::new();
        let c = ctx();
        let taxi = twr.handle(parse_intent("request taxi"), &c);
        assert!(taxi.contains("taxi to holding point Alpha runway zero niner"), "{taxi}");
        assert!(taxi.contains("QNH one zero one seven"), "{taxi}");
        assert_eq!(twr.state, State::TaxiCleared);

        let takeoff = twr.handle(parse_intent("ready for departure"), &c);
        assert!(takeoff.contains("cleared for takeoff"), "{takeoff}");
        assert!(takeoff.contains("wind zero niner zero degrees, three knots"), "{takeoff}");
        assert_eq!(twr.state, State::TakeoffCleared);
    }

    #[test]
    fn takeoff_from_the_apron_gets_taxi_first() {
        let mut twr = Tower::new();
        let reply = twr.handle(parse_intent("request takeoff"), &ctx());
        assert!(!reply.contains("cleared for takeoff"), "{reply}");
        assert!(reply.contains("taxi to holding point"), "{reply}");
        assert_eq!(twr.state, State::TaxiCleared);
    }

    #[test]
    fn unknown_gets_say_again_and_no_state_change() {
        let mut twr = Tower::new();
        let reply = twr.handle(parse_intent("uhh what's the weather like"), &ctx());
        assert!(reply.contains("say again"), "{reply}");
        assert_eq!(twr.state, State::Parked);
    }

    #[test]
    fn callsigns_speak_nato() {
        assert_eq!(
            spoken_callsign("N172SP"),
            "November one seven two Sierra Papa"
        );
        assert_eq!(spoken_callsign("G-ABCD"), "Golf Alpha Bravo Charlie Delta");
        assert_eq!(spoken_callsign(""), "");
    }
}
