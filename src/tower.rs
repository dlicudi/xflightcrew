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
    /// "what's my squawk" / "say transponder" — the pilot asking for the code
    /// again. Asked four times across the 2026-07-08 flights; each got "say
    /// again", which reads as a controller refusing to repeat a number.
    SquawkCheck,
    /// "taxi to parking" — an arrival leaving the runway, not a departure.
    RequestTaxiIn,
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
    // A checklist call-out is cockpit ritual, never a radio call: "takeoff
    // checklist" on a keyed radio must get "say again", not a clearance.
    if word("checklist") {
        return Intent::Unknown;
    }
    // "do you read/copy/hear me" / "are you there" are how pilots actually ask
    // for a radio check (five times across the 2026-07 flights, all
    // stonewalled with "say again").
    if t.contains("radio check")
        || t.contains("do you read")
        || t.contains("do you copy")
        || t.contains("do you hear")
        || t.contains("are you there")
    {
        return Intent::RadioCheck;
    }
    // "request vectors for taxi to parking" (real 2026-07-09 call) wants the
    // taxi, not the vectoring service — ground words beat the vector keyword.
    if t.contains("vector") && !t.contains("taxi") && !word("parking") {
        return Intent::RequestVectors;
    }
    // "say wind" / "wind check" / "how's the wind" — a report, not a clearance.
    // "win" is whisper's favourite spelling of "wind" ("win check", measured
    // in the stt-eval).
    if word("qnh") && (word("say") || t.contains("check") || t.contains("what"))
        || ((word("wind") || word("win"))
            && (word("say") || t.contains("check") || t.contains("what") || t.contains("how")))
    {
        return Intent::WindCheck;
    }
    // "what's my squawk" / "what about my transponder code" — question-shaped,
    // but a controller repeats a code without blinking. Must sit above the
    // question guard, like radio check and wind check.
    if (word("squawk") || word("transponder"))
        && (word("what") || word("say") || t.contains("confirm") || word("my") || word("code"))
    {
        return Intent::SquawkCheck;
    }
    // Questions are crew talk, not transmissions. "What runway shall I take
    // off from?" must reach the copilot (or get "say again" on a keyed radio),
    // never parse as a takeoff request — the 2026-07-08 flight was CLEARED FOR
    // TAKEOFF for asking it. Radio check, wind check and squawk check stay
    // above this guard: the question-shaped calls a tower genuinely answers.
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
    // "the option" is real phraseology for a touch and go, and also what
    // whisper writes when it garbles "touch and" ("request option go",
    // measured in the stt-eval).
    if t.contains("touch and go") || (word("option") && (t.contains("request") || word("go"))) {
        return Intent::RequestLanding { touch_and_go: true };
    }
    // "I've just landed" — an arrival already on the runway. Taxi in, never a
    // fresh join clearance (a real 2026-07 call, answered with "say again").
    if word("landed") {
        return Intent::RequestTaxiIn;
    }
    // "landing light on" / "landing gear down" / "flaps set for landing" are
    // cockpit commands, not arrival calls — same discipline as the taxi guard
    // below.
    if (t.contains("inbound") || word("landing") || word("land") || t.contains("full stop"))
        && !t.contains("light")
        && !t.contains("gear")
        && !t.contains("flaps")
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
    // "taxi light on" / "taxi hold" are COCKPIT commands that happen to contain
    // "taxi" — in the merged binary this parser sees every utterance, so a taxi
    // request must be a request, not any sentence with the word in it. Checked
    // before the clearance keyword: "request clearance for taxi" (the way the
    // 2026-07-08 pilot asked, three times) wants the taxi, not the zone
    // departure clearance.
    // The "hold" exclusion protects the copilot's taxi-hold governor
    // ("release the taxi hold"), but "taxi to holding point alpha" is the most
    // standard taxi phraseology there is — "holding point" survives it.
    if t.contains("taxi")
        && !t.contains("light")
        && (!t.contains("hold") || t.contains("holding point"))
    {
        // "taxi to parking/the ramp/the stand" is an arrival call — it must
        // never earn a fresh departure taxi to the holding point.
        if word("parking") || word("ramp") || word("apron") || word("stand") || word("gate") {
            return Intent::RequestTaxiIn;
        }
        return Intent::RequestTaxi;
    }
    if t.contains("clearance") {
        return Intent::RequestClearance;
    }
    Intent::Unknown
}

/// Parse a corpus label ("request_taxi") into the intent it names — the shared
/// vocabulary of the battle-test manifests (`bench/atc_corpus.yaml`, the ATC
/// cases in the stt-eval manifest). `None` for a label no intent carries.
pub fn intent_from_label(label: &str) -> Option<Intent> {
    Some(match label {
        "radio_check" => Intent::RadioCheck,
        "request_taxi" => Intent::RequestTaxi,
        "request_clearance" => Intent::RequestClearance,
        "request_takeoff" => Intent::RequestTakeoff,
        "request_landing" => Intent::RequestLanding { touch_and_go: false },
        "request_touch_and_go" => Intent::RequestLanding { touch_and_go: true },
        "report_final" => Intent::ReportFinal,
        "wind_check" => Intent::WindCheck,
        "squawk_check" => Intent::SquawkCheck,
        "request_taxi_in" => Intent::RequestTaxiIn,
        "request_vectors" => Intent::RequestVectors,
        "unknown" => Intent::Unknown,
        _ => return None,
    })
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
    ["descending", "descend", "maintaining", "maintain", "climbing", "climb", "passing",
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
            // An arrival taxiing in — by request wording, or because the last
            // clearance was a landing one (a generic "request taxi" right
            // after "cleared to land" is someone leaving the runway, not a
            // fresh departure wanting the holding point).
            Intent::RequestTaxiIn => {
                self.state = State::Parked;
                format!("{}, taxi to parking, this frequency.", ctx.callsign)
            }
            Intent::RequestTaxi if self.state == State::ClearedToLand => {
                self.state = State::Parked;
                format!("{}, taxi to parking, this frequency.", ctx.callsign)
            }
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
            // VFR conspicuity — the only code this tower ever assigns, so
            // repeating it is always safe.
            Intent::SquawkCheck => {
                format!("{}, squawk seven zero zero zero.", ctx.callsign)
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
        // The question-shaped calls a tower genuinely answers survive.
        assert_eq!(parse_intent("how do you read"), Intent::RadioCheck);
        assert_eq!(parse_intent("what's the wind"), Intent::WindCheck);
        // Squawk questions get the code repeated, not "say again" — asked four
        // times on the 2026-07-08 flights, stonewalled every time.
        assert_eq!(
            parse_intent("What about my transponder code?"),
            Intent::SquawkCheck
        );
        assert_eq!(parse_intent("say squawk"), Intent::SquawkCheck);
        assert_eq!(parse_intent("what's my squawk"), Intent::SquawkCheck);
        // A readback of the code is still a readback, not a question.
        assert_eq!(parse_intent("squawk seven zero zero zero"), Intent::Unknown);
        assert!(is_readback("squawk seven zero zero zero"));
        // Arrivals taxi IN — never a fresh departure taxi.
        assert_eq!(parse_intent("request taxi to parking"), Intent::RequestTaxiIn);
        assert_eq!(parse_intent("taxi to the ramp"), Intent::RequestTaxiIn);
        // "I've just landed" is an arrival on the runway, not "say again".
        assert_eq!(parse_intent("i've just landed"), Intent::RequestTaxiIn);
        assert_eq!(parse_intent("we have landed"), Intent::RequestTaxiIn);
        // "request clearance for taxi" wants the taxi, not the zone clearance
        // (asked three times on 2026-07-08; each got "cleared to leave the
        // zone" while parked at the stand).
        assert_eq!(
            parse_intent("request clearance for taxi"),
            Intent::RequestTaxi
        );
        assert_eq!(
            parse_intent("requesting clearance for taxi"),
            Intent::RequestTaxi
        );
        assert_eq!(
            parse_intent("request departure clearance"),
            Intent::RequestClearance,
            "clearance without taxi words is still the zone clearance"
        );
        // "do you copy/read/hear me" / "are you there" are radio checks — five
        // real calls across the 2026-07 flights, all stonewalled.
        assert_eq!(parse_intent("gibraltar tower, do you copy"), Intent::RadioCheck);
        assert_eq!(parse_intent("malaga tower, do you hear me?"), Intent::RadioCheck);
        assert_eq!(parse_intent("chipotle tower, are you there?"), Intent::RadioCheck);
        assert_eq!(parse_intent("how's the wind looking"), Intent::WindCheck);
        // Checklist call-outs are cockpit ritual, never radio calls.
        assert_eq!(parse_intent("takeoff checklist"), Intent::Unknown);
        assert_eq!(parse_intent("after landing checklist"), Intent::Unknown);
        assert_eq!(parse_intent("flaps set for landing"), Intent::Unknown);
        // "holding point" survives the taxi-hold guard; the governor commands
        // still don't.
        assert_eq!(
            parse_intent("request taxi to holding point alpha"),
            Intent::RequestTaxi
        );
        assert_eq!(parse_intent("release the taxi hold"), Intent::Unknown);
        // "transponder code" alone deserves the code (real 2026-07-08 call).
        assert_eq!(parse_intent("transponder code"), Intent::SquawkCheck);
    }

    /// Every (state, intent) cell of the FSM, exhaustively: the exact end
    /// state, plus the invariants a tower must never break anywhere in the
    /// grid. The named tests above cover the flown paths; this covers the
    /// cells nobody has flown yet.
    #[test]
    fn state_intent_matrix_holds_the_invariants() {
        use Intent::*;
        use State::*;
        let states = [Parked, TaxiCleared, TakeoffCleared, Inbound, ClearedToLand];
        let intents = [
            RadioCheck,
            RequestTaxi,
            RequestClearance,
            RequestTakeoff,
            RequestLanding { touch_and_go: false },
            RequestLanding { touch_and_go: true },
            ReportFinal,
            WindCheck,
            SquawkCheck,
            RequestTaxiIn,
            RequestVectors,
            Unknown,
        ];
        let c = ctx();
        for &start in &states {
            for &intent in &intents {
                let mut twr = Tower::new();
                twr.state = start;
                let reply = twr.handle(intent, &c);
                let end = twr.state;
                let cell = format!("{start:?} + {intent:?}");

                // A controller always answers, and always by callsign.
                assert!(!reply.is_empty(), "{cell}: silent");
                assert!(reply.contains(&c.callsign), "{cell}: no callsign in {reply:?}");

                // A takeoff clearance is only ever spoken to an aircraft that
                // asked for one FROM the holding-point states — never handed
                // out by any other cell of the grid.
                if reply.contains("cleared for takeoff") {
                    assert_eq!(intent, RequestTakeoff, "{cell}: unasked takeoff clearance");
                    assert!(
                        matches!(start, TaxiCleared | TakeoffCleared),
                        "{cell}: takeoff clearance from the wrong state"
                    );
                    assert!(reply.contains(&c.runway), "{cell}: clearance without runway");
                    assert!(reply.contains(&c.wind), "{cell}: clearance without wind");
                }
                // A landing clearance only ever answers a final report.
                if reply.contains("cleared to land") || reply.contains("cleared touch and go") {
                    assert_eq!(intent, ReportFinal, "{cell}: unasked landing clearance");
                    assert!(reply.contains(&c.runway), "{cell}: clearance without runway");
                    assert!(reply.contains(&c.wind), "{cell}: clearance without wind");
                }

                // The full transition table — any change here must be a
                // deliberate edit to this test, not a surprise.
                let want = match intent {
                    RadioCheck | WindCheck | SquawkCheck | Unknown | RequestClearance => start,
                    // ctx() has an approach frequency, so vectors redirect
                    // without touching the sequence.
                    RequestVectors => start,
                    RequestTaxiIn => Parked,
                    RequestTaxi => {
                        if start == ClearedToLand {
                            Parked
                        } else {
                            TaxiCleared
                        }
                    }
                    RequestTakeoff => {
                        if matches!(start, TaxiCleared | TakeoffCleared) {
                            TakeoffCleared
                        } else {
                            TaxiCleared
                        }
                    }
                    RequestLanding { .. } => Inbound,
                    ReportFinal => ClearedToLand,
                };
                assert_eq!(end, want, "{cell}: wrong transition");
            }
        }

        // With no approach service, a vectors request joins the pattern
        // instead — from every state.
        let mut no_app = ctx();
        no_app.approach = None;
        for &start in &states {
            let mut twr = Tower::new();
            twr.state = start;
            twr.handle(RequestVectors, &no_app);
            assert_eq!(twr.state, Inbound, "{start:?}: no-approach vectors must join");
        }
    }

    #[test]
    fn squawk_question_repeats_the_code() {
        let mut twr = Tower::new();
        let r = twr.handle(parse_intent("what about my transponder code?"), &ctx());
        assert!(r.contains("squawk seven zero zero zero"), "{r}");
    }

    #[test]
    fn arrivals_taxi_to_parking_not_the_holding_point() {
        // By wording, from any state.
        let mut twr = Tower::new();
        let r = twr.handle(parse_intent("request taxi to parking"), &ctx());
        assert!(r.contains("taxi to parking"), "{r}");
        assert!(!r.contains("holding point"), "{r}");
        assert_eq!(twr.state, State::Parked);
        // By state: a generic taxi request right after a landing clearance is
        // the arrival leaving the runway.
        let mut twr = Tower::new();
        twr.handle(parse_intent("inbound for landing"), &ctx());
        twr.handle(parse_intent("final"), &ctx());
        let r = twr.handle(parse_intent("request taxi"), &ctx());
        assert!(r.contains("taxi to parking"), "{r}");
        assert_eq!(twr.state, State::Parked);
        // A departure from the stand still gets the holding point.
        let mut twr = Tower::new();
        let r = twr.handle(parse_intent("request taxi"), &ctx());
        assert!(r.contains("holding point"), "{r}");
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
