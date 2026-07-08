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

/// Where the aircraft is in the clearance sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Parked,
    TaxiCleared,
    TakeoffCleared,
    /// Told to join final; waiting on the "final" report.
    Inbound,
    ClearedToLand,
}

/// What the pilot asked for, keyword-matched from the transcript.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Intent {
    RadioCheck,
    RequestTaxi,
    RequestClearance,
    RequestTakeoff,
    /// Arriving traffic ("inbound for landing", "request touch and go").
    RequestLanding {
        touch_and_go: bool,
    },
    /// "final" — position report on final approach.
    ReportFinal,
    /// "say wind" / "wind check" / "say QNH".
    WindCheck,
    /// "request vectors" — the tower redirects to the approach frequency.
    RequestVectors,
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
    /// Approach frequency, spoken ("one two two decimal eight") — for the
    /// vectors redirect. `None` = the field has no approach service.
    pub approach: Option<String>,
}

pub fn parse_intent(transcript: &str) -> Intent {
    let t = transcript.to_lowercase();
    let word = |w: &str| t.split(|c: char| !c.is_alphanumeric()).any(|tok| tok == w);
    if t.contains("radio check") || t.contains("how do you read") {
        return Intent::RadioCheck;
    }
    if t.contains("vector") {
        return Intent::RequestVectors;
    }
    // "say wind" / "wind check" / "say QNH" — a report, not a clearance.
    if word("qnh") && (word("say") || t.contains("check") || t.contains("what"))
        || (word("wind") && (word("say") || t.contains("check") || t.contains("what")))
    {
        return Intent::WindCheck;
    }
    // Questions are crew talk, not transmissions. "What runway shall I take
    // off from?" must reach the copilot (or get "say again" on a keyed radio),
    // never parse as a takeoff request — the 2026-07-08 flight was CLEARED FOR
    // TAKEOFF for asking it. Radio check and wind check stay above this guard:
    // those are the two question-shaped calls a tower genuinely answers.
    if word("what") || word("which") || word("how") || word("where") || word("when")
        || word("why") || word("shall") || word("should") || word("can") || word("could")
    {
        return Intent::Unknown;
    }
    // "final" is a position report; checked before the landing request so
    // "final for landing" reads as the report (it earns the clearance). Not
    // when it's a cockpit question ("what's our final approach speed").
    if word("final") && !t.contains("what") && !t.contains("how") && !t.contains("speed") {
        return Intent::ReportFinal;
    }
    if t.contains("touch and go") {
        return Intent::RequestLanding { touch_and_go: true };
    }
    // "landing light on" / "landing gear down" are cockpit commands, not
    // arrival calls — same discipline as the taxi guard below.
    if (t.contains("inbound") || word("landing") || word("land") || t.contains("full stop"))
        && !t.contains("light")
        && !t.contains("gear")
    {
        return Intent::RequestLanding {
            touch_and_go: false,
        };
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
    // "taxi light on" / "taxi hold" are COCKPIT commands that happen to contain
    // "taxi" — in the merged binary this parser sees every utterance, so a taxi
    // request must be a request, not any sentence with the word in it.
    if t.contains("taxi") && !t.contains("light") && !t.contains("hold") {
        return Intent::RequestTaxi;
    }
    Intent::Unknown
}

/// Is this transmission a pilot READBACK of an instruction ("descending to one
/// thousand five hundred", "heading two five zero, November 172SP") rather than
/// a new request? A correct readback earns silence on a real frequency — the
/// controller only speaks up to correct a wrong one. Without this, every
/// readback got "say again" (2026-07-08 flight), which reads as ATC not
/// listening. Only consulted when no intent matched.
pub fn is_readback(transcript: &str) -> bool {
    let t = transcript.to_lowercase();
    let word = |w: &str| t.split(|c: char| !c.is_alphanumeric()).any(|tok| tok == w);
    ["descending", "descend", "maintaining", "maintain", "climbing", "climb",
     "heading", "cleared", "roger", "wilco", "squawk", "squawking", "copy", "copied"]
        .iter()
        .any(|w| word(w))
}

/// The tower's side of the dialogue: one aircraft (the player), one runway.
pub struct Tower {
    pub state: State,
    /// The arrival asked for a touch and go; shapes the landing clearance.
    touch_and_go: bool,
}

impl Tower {
    pub fn new() -> Self {
        Tower {
            state: State::Parked,
            touch_and_go: false,
        }
    }

    /// Pre-arm the touch-and-go option for an arrival that asked for it on
    /// another frequency (approach): its "final" report lands here with no
    /// inbound call, and the clearance must still say "touch and go".
    pub fn expect_touch_and_go(&mut self, touch_and_go: bool) {
        self.touch_and_go = touch_and_go;
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
                State::TaxiCleared | State::TakeoffCleared => {
                    self.state = State::TakeoffCleared;
                    format!(
                        "{}, runway {}, wind {}, cleared for takeoff.",
                        ctx.callsign, ctx.runway, ctx.wind
                    )
                }
                // A takeoff request from anywhere else gets the taxi first —
                // the tower never clears a takeoff from a parking stand.
                _ => {
                    self.state = State::TaxiCleared;
                    format!(
                        "{}, taxi to holding point {} runway {} first, QNH {}. \
                         Report ready for departure.",
                        ctx.callsign, ctx.holding_point, ctx.runway, ctx.qnh
                    )
                }
            },
            Intent::RequestLanding { touch_and_go } => {
                self.state = State::Inbound;
                self.touch_and_go = touch_and_go;
                format!(
                    "{}, join final runway {}, QNH {}. Report final.",
                    ctx.callsign, ctx.runway, ctx.qnh
                )
            }
            // A final report earns the clearance from ANY state: a vectored
            // arrival reaches the tower already on final, with no inbound call
            // to this frequency — the tower doesn't argue with geometry.
            Intent::ReportFinal => {
                self.state = State::ClearedToLand;
                let clearance = if self.touch_and_go {
                    "cleared touch and go"
                } else {
                    "cleared to land"
                };
                format!(
                    "{}, runway {}, wind {}, {}.",
                    ctx.callsign, ctx.runway, ctx.wind, clearance
                )
            }
            Intent::WindCheck => {
                format!("{}, wind {}, QNH {}.", ctx.callsign, ctx.wind, ctx.qnh)
            }
            Intent::RequestVectors => match &ctx.approach {
                Some(freq) => format!(
                    "{}, contact {} Approach on {} for vectors.",
                    ctx.callsign, ctx.airport, freq
                ),
                None => {
                    self.state = State::Inbound;
                    format!(
                        "{}, no approach service at {}. Join final runway {}, report final.",
                        ctx.callsign, ctx.airport, ctx.runway
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
            approach: Some("one two two decimal eight".into()),
        }
    }

    #[test]
    fn intents_parse_from_natural_calls() {
        assert_eq!(
            parse_intent("Gibraltar Tower, November 172SP, request taxi"),
            Intent::RequestTaxi
        );
        assert_eq!(
            parse_intent("ready for departure runway 09"),
            Intent::RequestTakeoff
        );
        assert_eq!(parse_intent("request takeoff"), Intent::RequestTakeoff);
        assert_eq!(
            parse_intent("request departure clearance"),
            Intent::RequestClearance
        );
        assert_eq!(parse_intent("radio check"), Intent::RadioCheck);
        assert_eq!(parse_intent("good morning"), Intent::Unknown);
        // "taxi" inside a takeoff call must not read as a taxi request.
        assert_eq!(
            parse_intent("holding point taxi complete, ready for departure"),
            Intent::RequestTakeoff
        );
        // Cockpit commands containing "taxi" are NOT radio calls — in the
        // merged binary this parser sees every utterance the pilot speaks.
        assert_eq!(parse_intent("taxi light on"), Intent::Unknown);
        assert_eq!(parse_intent("release the taxi hold"), Intent::Unknown);
        // Arrival calls.
        assert_eq!(
            parse_intent("inbound for landing"),
            Intent::RequestLanding {
                touch_and_go: false
            }
        );
        assert_eq!(
            parse_intent("request clearance for landing"),
            Intent::RequestLanding {
                touch_and_go: false
            },
            "landing beats the departure-clearance keyword"
        );
        assert_eq!(
            parse_intent("request touch and go"),
            Intent::RequestLanding { touch_and_go: true }
        );
        assert_eq!(
            parse_intent("november one seven two, final"),
            Intent::ReportFinal
        );
        assert_eq!(
            parse_intent("request vectors for the approach"),
            Intent::RequestVectors
        );
        assert_eq!(parse_intent("say wind"), Intent::WindCheck);
        assert_eq!(parse_intent("wind check"), Intent::WindCheck);
        // Cockpit vocabulary that must NOT read as arrival calls.
        assert_eq!(parse_intent("landing light on"), Intent::Unknown);
        assert_eq!(parse_intent("landing gear down"), Intent::Unknown);
        assert_eq!(
            parse_intent("what is our final approach speed"),
            Intent::Unknown
        );
        // Questions are crew talk, never clearance requests — asking "what
        // runway shall I take off from" once earned a real takeoff clearance.
        assert_eq!(
            parse_intent("What runway shall I take off from?"),
            Intent::Unknown
        );
        assert_eq!(
            parse_intent("should we request takeoff now"),
            Intent::Unknown
        );
        assert_eq!(parse_intent("can we land at gibraltar"), Intent::Unknown);
        // The two question-shaped calls a tower genuinely answers survive.
        assert_eq!(parse_intent("how do you read"), Intent::RadioCheck);
        assert_eq!(parse_intent("what's the wind"), Intent::WindCheck);
    }

    #[test]
    fn readbacks_earn_silence_not_say_again() {
        // Instruction echoes: no intent, but recognizably a readback.
        for rb in [
            "descending to one thousand five hundred feet",
            "heading two five zero, november one seven two",
            "roger, wilco",
            "squawking seven thousand",
        ] {
            assert_eq!(parse_intent(rb), Intent::Unknown, "{rb}");
            assert!(is_readback(rb), "{rb}");
        }
        // Genuine garble is NOT a readback — it still gets "say again".
        assert!(!is_readback("uhh gibraltar something something"));
        assert!(!is_readback("requesting flight following"));
    }

    #[test]
    fn arrival_flow_inbound_final_cleared() {
        let mut twr = Tower::new();
        let c = ctx();
        let join = twr.handle(parse_intent("gibraltar tower, inbound for landing"), &c);
        assert!(join.contains("join final runway zero niner"), "{join}");
        assert!(join.contains("Report final"), "{join}");
        assert_eq!(twr.state, State::Inbound);
        let clear = twr.handle(parse_intent("final"), &c);
        assert!(clear.contains("cleared to land"), "{clear}");
        assert_eq!(twr.state, State::ClearedToLand);
        // Touch and go shapes the clearance.
        let mut twr = Tower::new();
        twr.handle(parse_intent("request touch and go"), &c);
        let clear = twr.handle(parse_intent("final"), &c);
        assert!(clear.contains("cleared touch and go"), "{clear}");
    }

    #[test]
    fn touch_and_go_survives_the_approach_handoff() {
        // The option was requested on the approach frequency; the tower only
        // ever hears "final" — the clearance must still say touch and go.
        let mut twr = Tower::new();
        twr.expect_touch_and_go(true);
        let clear = twr.handle(parse_intent("final"), &ctx());
        assert!(clear.contains("cleared touch and go"), "{clear}");
    }

    #[test]
    fn vectored_arrival_gets_cleared_straight_from_final() {
        // A vectored arrival never made an inbound call to the tower; the
        // "final" report alone earns the clearance.
        let mut twr = Tower::new();
        let clear = twr.handle(parse_intent("final"), &ctx());
        assert!(clear.contains("cleared to land"), "{clear}");
    }

    #[test]
    fn vectors_redirect_to_approach_or_join_final() {
        let mut twr = Tower::new();
        let redirect = twr.handle(parse_intent("request vectors"), &ctx());
        assert!(
            redirect.contains("contact Gibraltar Approach on one two two decimal eight"),
            "{redirect}"
        );
        assert_eq!(twr.state, State::Parked, "redirect changes nothing");
        // No approach service → straight-in join instead.
        let mut no_app = ctx();
        no_app.approach = None;
        let join = twr.handle(parse_intent("request vectors"), &no_app);
        assert!(join.contains("Join final"), "{join}");
        assert_eq!(twr.state, State::Inbound);
    }

    #[test]
    fn wind_check_reads_wind_and_qnh() {
        let mut twr = Tower::new();
        let r = twr.handle(parse_intent("say wind"), &ctx());
        assert!(
            r.contains("wind zero niner zero degrees, three knots"),
            "{r}"
        );
        assert!(r.contains("QNH one zero one seven"), "{r}");
    }

    #[test]
    fn taxi_then_takeoff_is_the_happy_path() {
        let mut twr = Tower::new();
        let c = ctx();
        let taxi = twr.handle(parse_intent("request taxi"), &c);
        assert!(
            taxi.contains("taxi to holding point Alpha runway zero niner"),
            "{taxi}"
        );
        assert!(taxi.contains("QNH one zero one seven"), "{taxi}");
        assert_eq!(twr.state, State::TaxiCleared);

        let takeoff = twr.handle(parse_intent("ready for departure"), &c);
        assert!(takeoff.contains("cleared for takeoff"), "{takeoff}");
        assert!(
            takeoff.contains("wind zero niner zero degrees, three knots"),
            "{takeoff}"
        );
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
