//! Tower service: a deterministic clearance state machine plus phraseology.
//!
//! The `atc` binary owns the radio I/O (PTT capture, whisper, TTS); everything
//! here is pure — transcript in, spoken reply and state transition out — so
//! the dialogue logic unit-tests without a simulator or a microphone. Intents
//! are keyword-matched, not LLM-picked, in line with the deterministic-first
//! rule: a misheard call gets "say again", never a guessed clearance.

/// Whisper decoding bias for radio calls (see `Stt::set_prompt` in the crew stt module).
pub const STT_PROMPT: &str = "Radio calls to the tower: request taxi, request \
departure clearance, ready for departure, request takeoff, radio check, \
holding point, runway zero niner, runway two seven, mayday, pan pan, going \
around, downwind, turning base, line up and wait, lining up, holding \
position, wake turbulence, taxi via alpha bravo, request vectors, established \
on the localizer, established ILS. Numbers are spoken digit by digit.";

/// Where the aircraft is in the clearance sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Parked,
    TaxiCleared,
    /// Authorized onto the runway, NOT to fly: "line up and wait" (plain or
    /// conditional), or a cancelled takeoff clearance — the aircraft may be
    /// on the pavement, so the release watch owes it a clearance or a vacate.
    LinedUp,
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
    /// "request start-up" / "request clearance to start engines" — asking to
    /// turn the engines, NOT asking for the departure clearance. Distinct on
    /// purpose: a parked, engines-off aircraft asking to start must hear
    /// "start-up approved", never "cleared to leave the zone" (a Malaga gate
    /// got the zone clearance plus a squawk for it, 2026-07-12).
    RequestStartup,
    /// "request shutdown" / "clearance to switch engines off" — the arrival
    /// bookend of start-up. Approval, not a clearance: a parked arrival
    /// asking got "cleared to leave the zone" plus a squawk, twice
    /// (Gibraltar, 2026-07-13).
    RequestShutdown,
    RequestTakeoff,
    /// Arriving traffic ("inbound for landing", "request touch and go").
    RequestLanding {
        touch_and_go: bool,
    },
    /// "final" — position report on final approach.
    ReportFinal,
    /// "downwind" — pattern position report; earns the base call.
    ReportDownwind,
    /// "turning base" / "base leg" — earns "report final".
    ReportBase,
    /// "crosswind" / "upwind" — early pattern legs; earns "report downwind".
    ReportCrosswind,
    /// "going around" / "missed approach" — a balked landing rejoining the
    /// pattern. The landing clearance is void; the tower re-sequences.
    GoAround,
    /// "mayday" / "pan pan" / "declaring an emergency" — priority handling.
    /// The ONE intent that earns a landing clearance without a final report:
    /// an emergency outranks the sequence, never the other way round.
    Mayday,
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
    /// "good evening, Merida" / "hello tower" — an initial hail carrying no
    /// request yet. A real controller invites the message ("pass your
    /// message"); "say again" reads as a controller who wasn't listening.
    Greeting,
    /// "request flight following" — VFR radar advisories. Asked twice on
    /// 2026-07-10, both stonewalled with "say again". A tower hands it to
    /// approach when the field has one, or says honestly that it has no
    /// radar; approach/departure grant it.
    RequestFlightFollowing,
    /// "request frequency change" / "switching to approach" — radio
    /// housekeeping, never an arrival. "London" garbles to "land on", and a
    /// keyed "change to London Heathrow approach frequency" earned a JOIN
    /// CLEARANCE (2026-07-10) — the exact mishear-to-clearance failure the
    /// corpus exists to prevent.
    RequestFrequencyChange,
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
    /// Spoken pressure phrase in the field's own dialect and units — "QNH
    /// one zero one seven" (hPa, ICAO) or "altimeter two niner niner two"
    /// (inHg, FAA). Blank = the pilot has the current ATIS information.
    pub qnh: String,
    /// Wind, spoken ("zero niner zero degrees, three knots" or "calm").
    pub wind: String,
    /// Holding point name ("Alpha").
    pub holding_point: String,
    /// Spoken parking-stand name for an ARRIVAL taxi-in ("stand one two"),
    /// planned to the nearest suitable stand. Empty on a departure, or when the
    /// field publishes no stands (the reply falls back to "taxi to parking").
    pub stand: String,
    /// Approach frequency, spoken ("one two two decimal eight") — for the
    /// vectors redirect. `None` = the field has no approach service.
    pub approach: Option<String>,
    /// The squawk to state when asked: this flight's ASSIGNED discrete code
    /// once a radar service has issued one, else the European VFR
    /// conspicuity "seven zero zero zero". Spoken digits.
    pub squawk: String,
    /// Nearest live traffic on final (nm), off the TCAS picture, read when
    /// the ctx was built. `Some` makes the tower sequence around it: takeoff
    /// clearances hold, downwind joins go number two. `None` = final clear.
    pub final_traffic_nm: Option<f64>,
    /// The spoken callsign of that traffic ("Speedbird three zero six"), when
    /// the TCAS picture carries one — names the condition in a conditional
    /// line-up clearance.
    pub final_traffic_callsign: Option<String>,
    /// How many arrivals are in the corridor. A conditional clearance is only
    /// legal with ONE intervening aircraft ("behind the landing traffic" is
    /// ambiguous against a stream).
    pub final_traffic_count: usize,
    /// A ghost is ON our strip (on the ground): nobody lines up onto an
    /// occupied runway.
    pub runway_occupied: bool,
    /// Wake-turbulence delay still to run behind the last departure from our
    /// strip, seconds. `Some` turns "cleared for takeoff" into "line up and
    /// wait, wake turbulence".
    pub wake_secs_left: Option<u64>,
    /// The taxi route to the holding point, as spoken taxiway names
    /// ("Alpha", "Bravo two"). Empty = no taxi network for this field; the
    /// clearance stays route-less, exactly as before.
    pub taxi_via: Vec<String>,
    /// Runways the taxi route crosses, spoken ("zero niner left").
    pub taxi_crossings: Vec<String>,
    /// The aircraft is on the ground — shapes the mayday answer (a parked
    /// emergency gets the trucks, not a landing clearance; seen live
    /// 2026-07-09: mayday from the stand was "cleared to land").
    pub on_ground: bool,
    /// The aircraft is standing ON the strip in use: the sim put it there (a
    /// runway start), nobody taxied it. A "ready for departure" from there is
    /// a lined-up aircraft asking for its clearance, not a stand asking to
    /// skip the taxi.
    pub own_on_runway: bool,
}

/// Spoken position of traffic on final: "short final" inside 1.5 nm, else
/// "N mile final".
fn spoken_final(nm: f64) -> String {
    if nm < 1.5 {
        "short final".to_string()
    } else {
        format!("{:.0} mile final", nm.max(1.0))
    }
}

/// The " via Alpha, Bravo" clause of a taxi clearance — empty when the field
/// has no parsed taxi network, so route-less fields phrase exactly as before.
fn via_clause(ctx: &Ctx) -> String {
    if ctx.taxi_via.is_empty() {
        String::new()
    } else {
        format!(" via {}", ctx.taxi_via.join(", "))
    }
}

/// The arrival taxi-in clearance: "taxi to stand one two via Alpha, cross
/// runway zero niner, this frequency." Falls back to the flat "taxi to parking"
/// when no stand route was planned (field has no stands, or off the network).
fn taxi_in_reply(ctx: &Ctx) -> String {
    if ctx.stand.is_empty() {
        return format!("{}, taxi to parking, this frequency.", ctx.callsign);
    }
    format!(
        "{}, taxi to {}{}{}, this frequency.",
        ctx.callsign,
        ctx.stand,
        via_clause(ctx),
        crossing_clause(ctx),
    )
}

/// The ", cross runway ..." clause when the taxi route crosses one.
fn crossing_clause(ctx: &Ctx) -> String {
    ctx.taxi_crossings
        .iter()
        .map(|r| format!(", cross runway {r}"))
        .collect()
}

/// The pressure clause of a clearance (", QNH one zero two zero" or
/// ", altimeter two niner niner two" — `ctx.qnh` carries the whole phrase in
/// the field's own dialect and units). Empty when the pilot has the current
/// ATIS information (the station blanks `ctx.qnh`), so the clearance skips
/// the numbers the broadcast already gave them. `pub(crate)`: the approach
/// controller's vector phrases attach the same clause.
pub fn qnh_clause(ctx: &Ctx) -> String {
    if ctx.qnh.is_empty() {
        String::new()
    } else {
        format!(", {}", ctx.qnh)
    }
}

pub fn parse_intent(transcript: &str) -> Intent {
    let t = transcript.to_lowercase();
    let word = |w: &str| t.split(|c: char| !c.is_alphanumeric()).any(|tok| tok == w);
    // A checklist call-out is cockpit ritual, never a radio call: "takeoff
    // checklist" on a keyed radio must get "say again", not a clearance.
    if word("checklist") {
        return Intent::Unknown;
    }
    // Emergency first — a mayday outranks every other reading of the words,
    // including the question guard ("mayday, can you hear me" is still a
    // mayday). Strict keywords on purpose: this is the one intent that earns
    // a clearance without the normal sequence, so it must never be reachable
    // by a garble. The real 2026-07-09 mayday ("mayday, mayday" then
    // "requesting emergency landing") got "say again" twice before the word
    // "landing" happened to carry it.
    if word("mayday")
        || t.contains("may day") // whisper splits the word
        || t.contains("pan pan")
        || ((word("declare") || word("declaring")) && word("emergency"))
        || (word("emergency") && (word("landing") || word("land") || word("descent")))
    {
        return Intent::Mayday;
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
    // "say again ..." is the pilot asking for a repeat — it must never parse
    // as whatever noun follows ("say again the clearance" read as a zone
    // clearance request, corpus 2026-07-11). The gate answers it by
    // repeating an outstanding instruction; otherwise it's "say again" back.
    if t.contains("say again") {
        return Intent::Unknown;
    }
    // A line-up phrase is a READBACK of "line up and wait", never a request —
    // and "behind the landing traffic, lining up" must not read the word
    // "landing" as an arrival call (corpus 2026-07-11).
    if t.contains("line up") || t.contains("lining up") || t.contains("line-up") {
        return Intent::Unknown;
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
    // "request flight following" — a service request, and one whisper mangles
    // hard ("but a host tower requesting flight following" = Badajoz Tower,
    // 2026-07-10). Above the question guard: "can I get flight following" is
    // still the request.
    if t.contains("flight following") || t.contains("flight follow") {
        return Intent::RequestFlightFollowing;
    }
    // Frequency housekeeping, before anything can read "land on" (garbled
    // "London") as a landing: any sentence about changing/switching a
    // frequency is that request and nothing else.
    if t.contains("frequenc")
        && (word("switch") || word("change") || word("changing") || word("tune") || word("request"))
    {
        return Intent::RequestFrequencyChange;
    }
    // Parking is an arrival's business: "request clearance to parking",
    // "where shall we park?" — above the question guard (a tower genuinely
    // answers it) and above the "clearance" catch-all, which swallowed it and
    // answered a parked arrival with the ZONE clearance plus a squawk, twice
    // ("no, i said i want to park the aircraft", Gibraltar 2026-07-13).
    // "brake" keeps "set the parking brake" a cockpit command; landing and
    // takeoff context keeps "full stop, then parking" runway business, which
    // the blocks below own.
    if (word("park") || word("parking") || word("ramp") || word("apron"))
        && !t.contains("brake")
        && !t.contains("inbound")
        && !word("landing")
        && !word("land")
        && !t.contains("takeoff")
        && !t.contains("take off")
    {
        return Intent::RequestTaxiIn;
    }
    // Questions are crew talk, not transmissions. "What runway shall I take
    // off from?" must reach the copilot (or get "say again" on a keyed radio),
    // never parse as a takeoff request — the 2026-07-08 flight was CLEARED FOR
    // TAKEOFF for asking it. Radio check, wind check and squawk check stay
    // above this guard: the question-shaped calls a tower genuinely answers.
    if word("what")
        || word("which")
        || word("how")
        || word("where")
        || word("when")
        || word("why")
        || word("shall")
        || word("should")
        || word("can")
        || word("could")
        || word("confirm")
    {
        return Intent::Unknown;
    }
    // "final" is a position report; checked before the landing request so
    // "final for landing" reads as the report (it earns the clearance). Not
    // when it's a cockpit question ("what's our final approach speed").
    // "Established" (ILS/localizer) is the same report by other words: to
    // the tower it earns the landing clearance, to approach it earns the
    // tower handoff — both exactly what the final report does.
    // Cockpit vocabulary is excluded the same way the landing request below
    // excludes light/gear/flaps: "final checks complete", "flaps set for
    // final" and "established in the cruise" are crew talk, and this is the
    // arm that earns the clearance most directly, so a mishear here is the
    // exact failure the corpus exists to prevent. A real final report never
    // carries these words, so the honest calls all still parse.
    if (word("final") || t.contains("establish"))
        && !t.contains("what")
        && !t.contains("how")
        && !t.contains("speed")
        && !word("check")
        && !word("checks")
        && !word("flaps")
        && !word("gear")
        && !word("light")
        && !word("lights")
        && !word("set")
        && !word("cruise")
    {
        return Intent::ReportFinal;
    }
    // A balked landing rejoining the pattern — before the landing request so
    // "going around for another landing" reads as the go-around.
    if t.contains("going around")
        || t.contains("go around")
        || t.contains("go-around")
        || t.contains("missed approach")
    {
        return Intent::GoAround;
    }
    // Pattern position reports, before the landing request so "downwind for
    // landing" reads as the report. "base" alone is too common a word — it
    // needs pattern context ("turning base", "base leg", "on base", "left/
    // right base").
    if word("downwind") {
        return Intent::ReportDownwind;
    }
    if word("base")
        && (word("turning") || word("leg") || word("on") || word("left") || word("right"))
    {
        return Intent::ReportBase;
    }
    if word("crosswind") || word("upwind") {
        return Intent::ReportCrosswind;
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
    // Engine start, before the clearance keyword can swallow it: "request
    // clearance to start engines" wants the start-up approval, not the zone
    // departure clearance (parsed as RequestClearance at a Malaga gate,
    // engines off, park brake set, 2026-07-12). Bare "start the engines" has
    // no request word and stays a cockpit command via is_radio_call.
    if t.contains("start up")
        || t.contains("start-up")
        || word("startup")
        || t.contains("request start")
        || (word("start") && (word("engine") || word("engines")))
    {
        return Intent::RequestStartup;
    }
    // The other bookend: "request shutdown" / "requesting clearance to switch
    // engines off". Approval, not a clearance — the catch-all below answered
    // it with "cleared to leave the zone" (Gibraltar, 2026-07-13).
    if t.contains("shut down")
        || t.contains("shut-down")
        || word("shutdown")
        || ((word("engine") || word("engines")) && word("off"))
    {
        return Intent::RequestShutdown;
    }
    if t.contains("clearance") {
        return Intent::RequestClearance;
    }
    // A bare hail — greeting words with no request. LAST, so any real request
    // above already claimed the words ("good morning, request taxi" is the
    // taxi). "Checking in" is the same speech act ("checking in, radio check"
    // parses as the radio check first).
    if word("hello")
        || word("greetings")
        || t.contains("good morning")
        || t.contains("good evening")
        || t.contains("good afternoon")
        || t.contains("good day")
        || t.contains("checking in")
    {
        return Intent::Greeting;
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
        "request_startup" => Intent::RequestStartup,
        "request_shutdown" => Intent::RequestShutdown,
        "request_takeoff" => Intent::RequestTakeoff,
        "request_landing" => Intent::RequestLanding {
            touch_and_go: false,
        },
        "request_touch_and_go" => Intent::RequestLanding { touch_and_go: true },
        "report_final" => Intent::ReportFinal,
        "report_downwind" => Intent::ReportDownwind,
        "report_base" => Intent::ReportBase,
        "report_crosswind" => Intent::ReportCrosswind,
        "go_around" => Intent::GoAround,
        "mayday" => Intent::Mayday,
        "wind_check" => Intent::WindCheck,
        "squawk_check" => Intent::SquawkCheck,
        "request_taxi_in" => Intent::RequestTaxiIn,
        "request_vectors" => Intent::RequestVectors,
        "greeting" => Intent::Greeting,
        "flight_following" => Intent::RequestFlightFollowing,
        "frequency_change" => Intent::RequestFrequencyChange,
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
    // A confirm-question or a repeat request WANTS an answer — silence would
    // read as ATC ignoring it.
    if word("confirm") || t.contains("say again") {
        return false;
    }
    // "transponder set to 4503" is the squawk readback in cockpit words
    // (real 2026-07-10 call, answered "say again" while the watch was
    // already confirming the code).
    [
        "descending",
        "descend",
        "maintaining",
        "maintain",
        "climbing",
        "climb",
        "passing",
        "heading",
        "cleared",
        "roger",
        "wilco",
        "squawk",
        "squawking",
        "transponder",
        "copy",
        "copied",
        "holding",
        "lining",
    ]
    .iter()
    .any(|w| word(w))
        || t.contains("line up")
        || frequency_readback(&t)
}

/// A frequency change read back: the facility and the digits, nothing else
/// ("Edinburgh Departure on one two four decimal eight two five"). It carries
/// none of the readback verbs, so the crew's own handoff readback earned
/// "Edinburgh Tower, say again" on every departure (EGPH, 2026-09-21).
fn frequency_readback(t: &str) -> bool {
    let word = |w: &str| t.split(|c: char| !c.is_alphanumeric()).any(|tok| tok == w);
    (word("decimal") || word("point"))
        && [
            "tower",
            "ground",
            "departure",
            "approach",
            "radar",
            "control",
            "director",
            "information",
        ]
        .iter()
        .any(|w| word(w))
}

/// The runway-critical instructions whose readback is MANDATORY (ICAO Doc
/// 4444 keeps the list short: clearances to enter, cross, take off from or
/// land on a runway, plus hold instructions). Kept to exactly those on
/// purpose — enforcing taxi/QNH echoes against whisper's transcription would
/// be a nag, not realism.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Awaited {
    /// "hold position" — any holding words satisfy it.
    HoldPosition,
    /// "line up and wait" — lining-up words; a CONDITIONAL clearance
    /// ("behind the landing traffic ... behind") must echo the condition.
    LineUp {
        runway: String,
        behind: bool,
    },
    Takeoff {
        runway: String,
    },
    Landing {
        runway: String,
        touch_and_go: bool,
    },
}

/// A clearance awaiting its readback: what must be echoed, the instruction
/// sentence to repeat on a challenge, and how many challenges have been
/// spent — two, then the tower lets it go. A garbled radio must never
/// deadlock the FSM; the safety property is "a mishear never EARNS a
/// clearance", not "a mishear traps you at the holding point".
#[derive(Debug, Clone)]
pub struct PendingReadback {
    pub awaited: Awaited,
    /// The instruction sentence, re-spoken when challenging.
    pub instruction: String,
    pub nudges: u8,
}

/// What a transmission is, measured against an awaited readback.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReadbackCheck {
    Correct,
    /// The echo names a DIFFERENT runway — the dangerous readback error, and
    /// the whole reason controllers listen to readbacks.
    WrongRunway {
        heard: String,
    },
    NotAReadback,
}

/// Parse a runway designator at the START of a token slice: spelled digits
/// ("two seven"), literal digits ("27", "09R"), an optional side word.
fn designator_at(toks: &[String]) -> Option<String> {
    let digit = |tok: &str| match tok {
        "zero" | "0" => Some('0'),
        "one" | "1" => Some('1'),
        "two" | "2" => Some('2'),
        "three" | "3" => Some('3'),
        "four" | "4" => Some('4'),
        "five" | "5" => Some('5'),
        "six" | "6" => Some('6'),
        "seven" | "7" => Some('7'),
        "eight" | "8" => Some('8'),
        "nine" | "niner" | "9" => Some('9'),
        _ => None,
    };
    let mut digits = String::new();
    let mut side: Option<char> = None;
    let mut j = 0;
    // "27" / "09R" as one token.
    if let Some(first) = toks.first() {
        if first.len() <= 3 && first.chars().next().is_some_and(|c| c.is_ascii_digit()) {
            for c in first.chars() {
                match c {
                    '0'..='9' if digits.len() < 2 => digits.push(c),
                    'l' => side = Some('L'),
                    'r' => side = Some('R'),
                    'c' => side = Some('C'),
                    _ => {}
                }
            }
            j = 1;
        }
    }
    // Spelled out, digit by digit.
    while side.is_none() && digits.len() < 2 {
        match toks.get(j).and_then(|t| digit(t)) {
            Some(d) => {
                digits.push(d);
                j += 1;
            }
            None => break,
        }
    }
    if digits.is_empty() {
        return None;
    }
    if side.is_none() {
        side = toks.get(j).and_then(|s| match s.as_str() {
            "left" => Some('L'),
            "right" => Some('R'),
            "center" | "centre" => Some('C'),
            _ => None,
        });
    }
    let trimmed = digits.trim_start_matches('0');
    let mut out = if trimmed.is_empty() {
        "0".to_string()
    } else {
        trimmed.to_string()
    };
    if let Some(s) = side {
        out.push(s);
    }
    Some(out)
}

fn lower_tokens(t: &str) -> Vec<String> {
    t.split(|c: char| !c.is_alphanumeric())
        .filter(|s| !s.is_empty())
        .map(|s| s.to_lowercase())
        .collect()
}

/// Canonical designator from a bare runway string ("zero niner right",
/// "27R") — for the EXPECTED side of a comparison, where the whole string is
/// the designator.
fn canon_runway(t: &str) -> Option<String> {
    let toks = lower_tokens(t);
    (0..toks.len()).find_map(|i| designator_at(&toks[i..]))
}

/// The runway an ARRIVAL names for itself ("final runway two six", "inbound
/// runway zero eight"), canonical, for the station's runway pick.
pub fn stated_runway(transcript: &str) -> Option<String> {
    runway_after_keyword(transcript)
}

/// [`canon_runway`] for the station: does this strip ident match a canonical
/// designator a transmission named?
pub fn runway_matches(ident: &str, canon: &str) -> bool {
    canon_runway(ident).as_deref() == Some(canon)
}

/// The designator a TRANSMISSION names — only after the word "runway", so a
/// callsign's digits ("november one seven two...") can never read as one.
fn runway_after_keyword(t: &str) -> Option<String> {
    let toks = lower_tokens(t);
    let pos = toks.iter().position(|w| w == "runway")?;
    designator_at(&toks[pos + 1..])
}

/// Measure a transmission against the awaited readback. Deterministic
/// keyword checks, same discipline as [`parse_intent`]: lenient on filler
/// (whisper drops words), strict on contradiction (a WRONG runway is always
/// caught, never waved through).
pub fn check_readback(transcript: &str, awaited: &Awaited) -> ReadbackCheck {
    let t = transcript.to_lowercase();
    let word = |w: &str| t.split(|c: char| !c.is_alphanumeric()).any(|tok| tok == w);
    // A fresh request, a readiness call, or a confirm-question is never the
    // echo — those get the challenge (which repeats the instruction, so
    // "confirm cleared for takeoff?" is answered by the repeat).
    if word("request")
        || word("requesting")
        || word("ready")
        || word("confirm")
        || t.contains("say again")
    {
        return ReadbackCheck::NotAReadback;
    }
    let verb_ok = match awaited {
        Awaited::HoldPosition => word("hold") || word("holding"),
        Awaited::LineUp { behind, .. } => {
            let lining = t.contains("line up")
                || t.contains("lining up")
                || t.contains("line-up")
                || word("lineup");
            // The condition is part of the clearance: it must come back.
            lining && (!behind || word("behind"))
        }
        Awaited::Takeoff { .. } => {
            t.contains("takeoff") || t.contains("take off") || t.contains("take-off")
        }
        Awaited::Landing { touch_and_go, .. } => {
            if *touch_and_go {
                t.contains("touch and go") || word("option") || word("cleared")
            } else {
                word("land") || word("landing") || word("cleared")
            }
        }
    };
    if !verb_ok {
        return ReadbackCheck::NotAReadback;
    }
    // The echo carries a runway: it must be OUR runway. No runway spoken is
    // accepted (whisper eats numbers); the wrong one never is.
    let expect = match awaited {
        Awaited::LineUp { runway, .. }
        | Awaited::Takeoff { runway }
        | Awaited::Landing { runway, .. } => canon_runway(runway),
        Awaited::HoldPosition => None,
    };
    if let (Some(e), Some(h)) = (expect, runway_after_keyword(&t)) {
        if e != h {
            return ReadbackCheck::WrongRunway { heard: h };
        }
    }
    ReadbackCheck::Correct
}

/// A conditional line-up needs the arrival far enough out that the pilot can
/// actually get behind it — inside this, "hold position" is the honest call.
pub(crate) const CONDITIONAL_MIN_NM: f64 = 1.5;

/// The tower's side of the dialogue: one aircraft (the player), one runway.
pub struct Tower {
    pub state: State,
    /// The arrival asked for a touch and go; shapes the landing clearance.
    touch_and_go: bool,
    /// A runway-critical clearance awaiting its mandatory readback.
    pending: Option<PendingReadback>,
    /// The departure clearance's readback, SUGGESTED and never enforced
    /// (ICAO's mandatory list stops at the runway): the zone cap, the QNH
    /// and the squawk. The crew reads it when it flies; a human pilot sees
    /// it in the coach. Dropped by the next request.
    clearance_readback: Option<String>,
}

impl Tower {
    pub fn new() -> Self {
        Tower {
            state: State::Parked,
            touch_and_go: false,
            pending: None,
            clearance_readback: None,
        }
    }

    /// The crew has read the departure clearance back (or moved on).
    pub fn clear_clearance_readback(&mut self) {
        self.clearance_readback = None;
    }

    /// Pre-arm the touch-and-go option for an arrival that asked for it on
    /// another frequency (approach): its "final" report lands here with no
    /// inbound call, and the clearance must still say "touch and go".
    pub fn expect_touch_and_go(&mut self, touch_and_go: bool) {
        self.touch_and_go = touch_and_go;
    }

    /// The clearance still awaiting its readback, if any. Consumed by the
    /// battle tests; the binary reads it via [`Tower::suggested_readback`].
    #[allow(dead_code)]
    pub fn pending(&self) -> Option<&PendingReadback> {
        self.pending.as_ref()
    }

    /// Arm the takeoff readback for a clearance the MONITOR spoke (the held-
    /// clearance release) — the FSM must expect the echo exactly as if the
    /// clearance had been issued on the reply path.
    pub fn arm_takeoff_readback(&mut self, runway: String, instruction: String) {
        self.pending = Some(PendingReadback {
            awaited: Awaited::Takeoff { runway },
            instruction,
            nudges: 0,
        });
    }

    /// Arm the line-up readback for a line-up-and-wait the MONITOR spoke
    /// (the held-clearance watch sequencing a departure onto the runway
    /// while the wake clock runs) — same contract as
    /// [`Tower::arm_takeoff_readback`], one state earlier.
    pub fn arm_line_up_readback(&mut self, runway: String, instruction: String) {
        self.pending = Some(PendingReadback {
            awaited: Awaited::LineUp {
                runway,
                behind: false,
            },
            instruction,
            nudges: 0,
        });
    }

    /// The MONITOR ordered a go-around (traffic on the runway): the landing
    /// clearance is void and the flight is back in the pattern, exactly as
    /// if the pilot had called the go-around on the reply path. Any owed
    /// readback dies with the clearance it echoed.
    pub fn apply_go_around(&mut self) {
        self.state = State::Inbound;
        self.pending = None;
    }

    fn arm(&mut self, awaited: Awaited, instruction: &str) {
        self.pending = Some(PendingReadback {
            awaited,
            instruction: instruction.to_string(),
            nudges: 0,
        });
    }

    /// The exact readback the pilot owes, phrased — the coach's line, and the
    /// Calls tab's SAY NOW row.
    pub fn suggested_readback(&self, callsign: &str) -> Option<String> {
        if self.pending.is_none() {
            return self.clearance_readback.clone();
        }
        Some(match &self.pending.as_ref()?.awaited {
            Awaited::HoldPosition => format!("Holding position, {callsign}."),
            Awaited::LineUp {
                runway,
                behind: false,
            } => {
                format!("Lining up runway {runway}, {callsign}.")
            }
            Awaited::LineUp {
                runway,
                behind: true,
            } => {
                format!("Behind the landing traffic, lining up runway {runway} behind, {callsign}.")
            }
            Awaited::Takeoff { runway } => {
                format!("Cleared for takeoff runway {runway}, {callsign}.")
            }
            Awaited::Landing {
                runway,
                touch_and_go: false,
            } => {
                format!("Cleared to land runway {runway}, {callsign}.")
            }
            Awaited::Landing {
                runway,
                touch_and_go: true,
            } => {
                format!("Cleared touch and go runway {runway}, {callsign}.")
            }
        })
    }

    /// The readback gate, run on a KEYED transmission before normal routing.
    /// `Some(reply)` means the gate consumed the transmission: empty = a
    /// correct readback (the frequency stays silent), otherwise the
    /// challenge or correction to speak. `None` = route normally.
    pub fn gate(&mut self, transcript: &str, intent: Intent, ctx: &Ctx) -> Option<String> {
        let pending = self.pending.as_ref()?;
        // Physics consumed the instruction: demanding the echo of a takeoff
        // clearance from an aircraft already flying (or of a landing
        // clearance from one already down) is theater. Drop it silently.
        let stale = match pending.awaited {
            Awaited::Landing { .. } => ctx.on_ground,
            _ => !ctx.on_ground,
        };
        if stale {
            self.pending = None;
            return None;
        }
        // Safety outranks discipline: an emergency or a go-around is never
        // answered with "read back".
        if matches!(intent, Intent::Mayday | Intent::GoAround) {
            self.pending = None;
            return None;
        }
        match check_readback(transcript, &pending.awaited) {
            ReadbackCheck::Correct => {
                self.pending = None;
                Some(String::new())
            }
            ReadbackCheck::WrongRunway { .. } => {
                let instruction = pending.instruction.clone();
                Some(format!(
                    "{}, negative. I say again: {instruction}",
                    ctx.callsign
                ))
            }
            ReadbackCheck::NotAReadback => {
                // The question-shaped calls a controller answers regardless.
                if matches!(
                    intent,
                    Intent::RadioCheck | Intent::WindCheck | Intent::SquawkCheck | Intent::Greeting
                ) {
                    return None;
                }
                let p = self.pending.as_mut().unwrap();
                if p.nudges >= 2 {
                    // Two challenges spent: let the call through — the tower
                    // must never trap a garbled radio in a readback loop.
                    self.pending = None;
                    return None;
                }
                p.nudges += 1;
                let instruction = p.instruction.clone();
                Some(format!("{}, read back: {instruction}", ctx.callsign))
            }
        }
    }

    /// A final report that names the WRONG runway is corrected, not waved
    /// through. The clearance that answers it already names the right one,
    /// but a pilot who just said "final runway one three" hears "runway one
    /// two, cleared to land" as confirmation, not contradiction: the crew's
    /// autoland reported 13, the tower cleared 12 twice, and the aircraft
    /// flew 13 both times (LEMG, 2026-09-12). Wraps the reply [`handle`]
    /// produced; every other call passes through untouched. A report with no
    /// runway in it is not wrong (whisper eats numbers).
    pub fn correct_final_runway(transcript: &str, intent: Intent, ctx: &Ctx, reply: String) -> String {
        if intent != Intent::ReportFinal {
            return reply;
        }
        let (Some(ours), Some(heard)) = (canon_runway(&ctx.runway), runway_after_keyword(transcript))
        else {
            return reply;
        };
        if ours == heard {
            return reply;
        }
        let prefix = format!("{}, ", ctx.callsign);
        let body = reply.strip_prefix(&prefix).unwrap_or(&reply);
        format!(
            "{}negative, runway {}, I say again runway {}. {body}",
            prefix, ctx.runway, ctx.runway
        )
    }

    /// Answer one transmission, advancing the clearance state.
    pub fn handle(&mut self, intent: Intent, ctx: &Ctx) -> String {
        // Any new REQUEST voids an outstanding readback demand — the gate
        // already had its chance to challenge; processing the call means the
        // dialogue has moved on. The answer-only intents leave it armed.
        if !matches!(
            intent,
            Intent::RadioCheck
                | Intent::WindCheck
                | Intent::SquawkCheck
                | Intent::Greeting
                | Intent::RequestFlightFollowing
                | Intent::RequestFrequencyChange
                | Intent::RequestShutdown
                | Intent::Unknown
        ) {
            self.pending = None;
        }
        // A new request moves the dialogue past the clearance readback too.
        if intent != Intent::Unknown {
            self.clearance_readback = None;
        }
        match intent {
            Intent::RadioCheck => {
                format!("{}, {} Tower, reading you five.", ctx.callsign, ctx.airport)
            }
            Intent::RequestClearance => {
                self.clearance_readback = Some(format!(
                    "Not above two thousand feet{}, squawk {}, {}.",
                    qnh_clause(ctx),
                    ctx.squawk,
                    ctx.callsign
                ));
                format!(
                    "{}, cleared to leave the {} zone, not above two thousand feet{}, \
                     squawk {}.",
                    ctx.callsign,
                    ctx.airport,
                    qnh_clause(ctx),
                    ctx.squawk
                )
            }
            // Start-up approval is not a clearance to go anywhere: no squawk,
            // no state change — the zone/taxi/takeoff chain still follows.
            Intent::RequestStartup => {
                format!("{}, start-up approved{}.", ctx.callsign, qnh_clause(ctx))
            }
            // The arrival bookend: nothing to clear, nowhere to go.
            Intent::RequestShutdown => {
                format!("{}, shutdown approved. Good day.", ctx.callsign)
            }
            // An arrival taxiing in — by request wording, or because the last
            // clearance was a landing one (a generic "request taxi" right
            // after "cleared to land" is someone leaving the runway, not a
            // fresh departure wanting the holding point).
            Intent::RequestTaxiIn => {
                self.state = State::Parked;
                taxi_in_reply(ctx)
            }
            Intent::RequestTaxi if self.state == State::ClearedToLand => {
                self.state = State::Parked;
                taxi_in_reply(ctx)
            }
            Intent::RequestTaxi => {
                self.state = State::TaxiCleared;
                format!(
                    "{}, taxi to holding point {} runway {}{}{}{}. \
                     Report ready for departure.",
                    ctx.callsign,
                    ctx.holding_point,
                    ctx.runway,
                    via_clause(ctx),
                    crossing_clause(ctx),
                    qnh_clause(ctx)
                )
            }
            Intent::RequestTakeoff => match self.state {
                // Already cleared, but traffic has since appeared on final:
                // the clearance is CANCELLED, not repeated — the 2026-07-10
                // tape has a takeoff clearance and a landing clearance on the
                // same runway eight seconds apart. The aircraft may already
                // be on the pavement, so it lands in LinedUp: the release
                // watch owes it the re-clearance.
                State::TakeoffCleared if ctx.final_traffic_nm.is_some() => {
                    self.state = State::LinedUp;
                    let on_final = spoken_final(ctx.final_traffic_nm.unwrap_or(0.0));
                    let line = format!(
                        "{}, cancel takeoff clearance, hold position, traffic on \
                         {on_final}.",
                        ctx.callsign
                    );
                    self.arm(Awaited::HoldPosition, &line);
                    line
                }
                // Landing traffic owns the runway. Exactly ONE arrival, far
                // enough out: a CONDITIONAL line-up puts the departure on the
                // pavement the moment the lander is past — real towers fill
                // this gap instead of parking you. Closer, more than one, or
                // already lined up: hold, honestly.
                State::TaxiCleared
                    if ctx.final_traffic_nm.is_some()
                        && ctx.final_traffic_count <= 1
                        && ctx.final_traffic_nm.unwrap_or(0.0) >= CONDITIONAL_MIN_NM =>
                {
                    self.state = State::LinedUp;
                    let traffic = ctx
                        .final_traffic_callsign
                        .clone()
                        .unwrap_or_else(|| "traffic".to_string());
                    let line = format!(
                        "{}, behind the landing {traffic}, line up runway {} behind.",
                        ctx.callsign, ctx.runway
                    );
                    self.arm(
                        Awaited::LineUp {
                            runway: ctx.runway.clone(),
                            behind: true,
                        },
                        &line,
                    );
                    line
                }
                State::TaxiCleared | State::LinedUp if ctx.final_traffic_nm.is_some() => {
                    let on_final = spoken_final(ctx.final_traffic_nm.unwrap_or(0.0));
                    let line = format!(
                        "{}, hold position, traffic on {on_final}. Expect clearance \
                         after the landing traffic.",
                        ctx.callsign
                    );
                    self.arm(Awaited::HoldPosition, &line);
                    line
                }
                // Wake turbulence: the last departure off our strip was
                // heavier than us and its two minutes haven't run. Line up
                // and wait IS the classic use — on the runway, ready, while
                // the clock runs; the release watch clears you when it has.
                State::TaxiCleared if ctx.wake_secs_left.is_some() && !ctx.runway_occupied => {
                    self.state = State::LinedUp;
                    let line = format!(
                        "{}, line up and wait runway {}, wake turbulence, two minutes \
                         behind the departing traffic.",
                        ctx.callsign, ctx.runway
                    );
                    self.arm(
                        Awaited::LineUp {
                            runway: ctx.runway.clone(),
                            behind: false,
                        },
                        &line,
                    );
                    line
                }
                State::LinedUp if ctx.wake_secs_left.is_some() => {
                    let line = format!(
                        "{}, hold position, wake turbulence. Expect clearance shortly.",
                        ctx.callsign
                    );
                    self.arm(Awaited::HoldPosition, &line);
                    line
                }
                // Already cleared, and a ghost has since taken the strip: the
                // clearance is CANCELLED, exactly like the final-traffic case
                // above. A re-ask must never repeat "cleared for takeoff"
                // over traffic on the runway. The aircraft may be on the
                // pavement, so it lands in LinedUp and the release watch owes
                // the re-clearance.
                State::TakeoffCleared if ctx.runway_occupied => {
                    self.state = State::LinedUp;
                    let line = format!(
                        "{}, cancel takeoff clearance, hold position, traffic on the runway.",
                        ctx.callsign
                    );
                    self.arm(Awaited::HoldPosition, &line);
                    line
                }
                // A ghost is ON the strip: nobody lines up onto an occupied
                // runway.
                State::TaxiCleared | State::LinedUp if ctx.runway_occupied => {
                    let line = format!("{}, hold position, traffic on the runway.", ctx.callsign);
                    self.arm(Awaited::HoldPosition, &line);
                    line
                }
                State::TaxiCleared | State::LinedUp | State::TakeoffCleared => {
                    self.state = State::TakeoffCleared;
                    let line = format!(
                        "{}, runway {}, wind {}, cleared for takeoff.",
                        ctx.callsign, ctx.runway, ctx.wind
                    );
                    self.arm(
                        Awaited::Takeoff {
                            runway: ctx.runway.clone(),
                        },
                        &line,
                    );
                    line
                }
                // Standing on the strip with no taxi behind it: a runway
                // start. The sim's own spawn point answered "taxi to holding
                // point Alpha first", full auto therefore never called at
                // all, and the A330 flew Glasgow's zone to "you are inside
                // controlled airspace without clearance" (2026-09-21). It is
                // a lined-up aircraft; every gate a lined-up aircraft meets
                // (traffic, wake, an occupied strip) is met by asking again
                // from LinedUp.
                _ if ctx.own_on_runway && ctx.on_ground => {
                    self.state = State::LinedUp;
                    return self.handle(Intent::RequestTakeoff, ctx);
                }
                // A takeoff request from anywhere else gets the taxi first —
                // the tower never clears a takeoff from a parking stand.
                _ => {
                    self.state = State::TaxiCleared;
                    format!(
                        "{}, taxi to holding point {} runway {} first{}{}{}. \
                         Report ready for departure.",
                        ctx.callsign,
                        ctx.holding_point,
                        ctx.runway,
                        via_clause(ctx),
                        crossing_clause(ctx),
                        qnh_clause(ctx)
                    )
                }
            },
            Intent::RequestLanding { touch_and_go } => {
                self.state = State::Inbound;
                self.touch_and_go = touch_and_go;
                format!(
                    "{}, join final runway {}{}. Report final.",
                    ctx.callsign,
                    ctx.runway,
                    qnh_clause(ctx)
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
                let line = format!(
                    "{}, runway {}, wind {}, {}.",
                    ctx.callsign, ctx.runway, ctx.wind, clearance
                );
                self.arm(
                    Awaited::Landing {
                        runway: ctx.runway.clone(),
                        touch_and_go: self.touch_and_go,
                    },
                    &line,
                );
                line
            }
            // Pattern position reports: the tower sequences the circuit leg by
            // leg. All land in Inbound — "in the pattern, awaiting final".
            // Live traffic on final makes the downwind join number two, off
            // the same TCAS picture the advisories narrate.
            Intent::ReportDownwind => {
                self.state = State::Inbound;
                match ctx.final_traffic_nm {
                    Some(nm) => format!(
                        "{}, number two, follow the traffic on {}, report base.",
                        ctx.callsign,
                        spoken_final(nm)
                    ),
                    None => format!("{}, number one, report base.", ctx.callsign),
                }
            }
            Intent::ReportBase => {
                self.state = State::Inbound;
                format!("{}, roger, report final.", ctx.callsign)
            }
            Intent::ReportCrosswind => {
                self.state = State::Inbound;
                format!("{}, roger, report downwind.", ctx.callsign)
            }
            // A balked landing: the clearance is void, rejoin the pattern.
            Intent::GoAround => {
                self.state = State::Inbound;
                format!(
                    "{}, roger, go around. Climb to circuit altitude, runway heading, \
                     report downwind.",
                    ctx.callsign
                )
            }
            // Priority handling — the ONE clearance spoken without a final
            // report. An emergency outranks the sequence. On the GROUND there
            // is nothing to clear: send the trucks, hold the traffic.
            Intent::Mayday if ctx.on_ground => {
                format!(
                    "{}, roger mayday. Emergency services on their way to you — \
                     hold position if able.",
                    ctx.callsign
                )
            }
            Intent::Mayday => {
                self.state = State::ClearedToLand;
                self.touch_and_go = false;
                format!(
                    "{}, roger mayday, cleared to land runway {}, or any runway you \
                     need. Wind {}. Emergency services standing by.",
                    ctx.callsign, ctx.runway, ctx.wind
                )
            }
            Intent::WindCheck => {
                format!("{}, wind {}{}.", ctx.callsign, ctx.wind, qnh_clause(ctx))
            }
            // Repeats the ASSIGNED code when a radar service issued one, the
            // VFR conspicuity otherwise — a controller repeats YOUR code, not
            // the default (always 7000 until 2026-07-10).
            Intent::SquawkCheck => {
                format!("{}, squawk {}.", ctx.callsign, ctx.squawk)
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
            // An initial hail: invite the message. No state change — the
            // request that follows does the driving.
            Intent::Greeting => {
                format!(
                    "{}, {} Tower, pass your message.",
                    ctx.callsign, ctx.airport
                )
            }
            // Radio housekeeping: approve and let them go. No state change.
            Intent::RequestFrequencyChange => {
                format!("{}, frequency change approved.", ctx.callsign)
            }
            // Flight following is a radar service: approach's job when the
            // field has one; a tower without a scope says so and keeps the
            // aircraft on the VFR conspicuity code. No state change.
            Intent::RequestFlightFollowing => match &ctx.approach {
                Some(freq) => format!(
                    "{}, for flight following contact {} Approach on {}.",
                    ctx.callsign, ctx.airport, freq
                ),
                None => format!(
                    "{}, no radar service at {}. Squawk {} and remain with me.",
                    ctx.callsign, ctx.airport, ctx.squawk
                ),
            },
            Intent::Unknown => {
                format!("{}, {} Tower, say again.", ctx.callsign, ctx.airport)
            }
        }
    }
}

impl Default for Tower {
    fn default() -> Self {
        Self::new()
    }
}

/// ICAO wake turbulence: HEAVY is a maximum takeoff mass of 136,000 kg or
/// more. The A330-300 (242 t) is one; the 737-800 (79 t) is a MEDIUM.
pub const HEAVY_MTOW_KG: f64 = 136_000.0;

/// The callsign with its wake suffix, the way a heavy identifies on every
/// initial call and the tower addresses it: "... Kilo November heavy". An
/// A330 was worked at Edinburgh as plain "Delta Alpha India Kilo November"
/// and the tower's wake clock knew nothing about it (2026-09-20). Unknown or
/// unreadable mass = no suffix, never a guess.
pub fn spoken_callsign_wake(tail: &str, mtow_kg: Option<f64>) -> String {
    let base = spoken_callsign(tail);
    if base.is_empty() {
        return base;
    }
    match mtow_kg {
        Some(m) if m.is_finite() && m >= HEAVY_MTOW_KG => format!("{base} heavy"),
        _ => base,
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
            qnh: "QNH one zero one seven".into(),
            wind: "zero niner zero degrees, three knots".into(),
            holding_point: "Alpha".into(),
            stand: String::new(),
            approach: Some("one two two decimal eight".into()),
            squawk: "seven zero zero zero".into(),
            final_traffic_nm: None,
            final_traffic_callsign: None,
            final_traffic_count: 0,
            runway_occupied: false,
            own_on_runway: false,
            wake_secs_left: None,
            taxi_via: Vec::new(),
            taxi_crossings: Vec::new(),
            on_ground: false,
        }
    }

    /// ctx() with one arrival on final at `nm`.
    fn busy(nm: f64) -> Ctx {
        Ctx {
            final_traffic_nm: Some(nm),
            final_traffic_callsign: Some("Speedbird three zero six".into()),
            final_traffic_count: 1,
            ..ctx()
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
        // A bare hail is an initial call, answered "pass your message" —
        // not "say again" (2026-07-10: "Good evening, Merida").
        assert_eq!(parse_intent("good morning"), Intent::Greeting);
        assert_eq!(parse_intent("good evening, merida"), Intent::Greeting);
        assert_eq!(parse_intent("hello, banahos"), Intent::Greeting);
        // Greeting words never outrank a real request in the same breath.
        assert_eq!(
            parse_intent("good morning, request taxi"),
            Intent::RequestTaxi
        );
        // Flight following, straight and through whisper's mangle.
        assert_eq!(
            parse_intent("request flight following"),
            Intent::RequestFlightFollowing
        );
        assert_eq!(
            parse_intent("but a host tower requesting flight following"),
            Intent::RequestFlightFollowing
        );
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
        // "Established" is the final report by other words — the ILS
        // arrival's version of it, to tower (clearance) and approach
        // (handoff) alike. Whisper often splits the acronym.
        assert_eq!(
            parse_intent("november one seven two, established i l s runway zero niner"),
            Intent::ReportFinal
        );
        assert_eq!(parse_intent("localizer established"), Intent::ReportFinal);
        // But a cockpit question about it stays crew talk.
        assert_eq!(
            parse_intent("how do I know when we're established"),
            Intent::Unknown
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
        assert_eq!(
            parse_intent("request taxi to parking"),
            Intent::RequestTaxiIn
        );
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
        // The word "clearance" must not swallow an engine-start request
        // (real 2026-07-12 call, answered with the zone clearance while
        // parked, engines off).
        assert_eq!(
            parse_intent("request clearance to start engines."),
            Intent::RequestStartup
        );
        assert_eq!(parse_intent("requesting startup"), Intent::RequestStartup);
        assert_eq!(parse_intent("request start-up"), Intent::RequestStartup);
        assert_eq!(
            parse_intent("malaga tower, november five five zero charlie romeo, request start up"),
            Intent::RequestStartup
        );
        // ...and it must not swallow a PARKED arrival either — all four are
        // real 2026-07-13 Gibraltar calls; the first two were answered
        // "cleared to leave the zone" plus a squawk, ending in "no, no, no,
        // no" from the pilot.
        assert_eq!(
            parse_intent("request clearance to parking."),
            Intent::RequestTaxiIn
        );
        assert_eq!(
            parse_intent("requesting clearance to switch engines off."),
            Intent::RequestShutdown
        );
        assert_eq!(
            parse_intent("where shall i park at?"),
            Intent::RequestTaxiIn,
            "a parking question is one the tower genuinely answers"
        );
        assert_eq!(
            parse_intent("no, i said i want to park the aircraft."),
            Intent::RequestTaxiIn
        );
        assert_eq!(parse_intent("request shutdown"), Intent::RequestShutdown);
        // The parking-brake checklist item stays a cockpit command.
        assert_eq!(parse_intent("set the parking brake"), Intent::Unknown);
        // "do you copy/read/hear me" / "are you there" are radio checks — five
        // real calls across the 2026-07 flights, all stonewalled.
        assert_eq!(
            parse_intent("gibraltar tower, do you copy"),
            Intent::RadioCheck
        );
        assert_eq!(
            parse_intent("malaga tower, do you hear me?"),
            Intent::RadioCheck
        );
        assert_eq!(
            parse_intent("chipotle tower, are you there?"),
            Intent::RadioCheck
        );
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
        let states = [
            Parked,
            TaxiCleared,
            LinedUp,
            TakeoffCleared,
            Inbound,
            ClearedToLand,
        ];
        let intents = [
            RadioCheck,
            RequestTaxi,
            RequestClearance,
            RequestStartup,
            RequestShutdown,
            RequestTakeoff,
            RequestLanding {
                touch_and_go: false,
            },
            RequestLanding { touch_and_go: true },
            ReportFinal,
            ReportDownwind,
            ReportBase,
            ReportCrosswind,
            GoAround,
            Mayday,
            WindCheck,
            SquawkCheck,
            RequestTaxiIn,
            RequestVectors,
            Greeting,
            RequestFlightFollowing,
            RequestFrequencyChange,
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
                assert!(
                    reply.contains(&c.callsign),
                    "{cell}: no callsign in {reply:?}"
                );

                // A takeoff clearance is only ever spoken to an aircraft that
                // asked for one FROM the holding-point states — never handed
                // out by any other cell of the grid.
                if reply.contains("cleared for takeoff") {
                    assert_eq!(intent, RequestTakeoff, "{cell}: unasked takeoff clearance");
                    assert!(
                        matches!(start, TaxiCleared | LinedUp | TakeoffCleared),
                        "{cell}: takeoff clearance from the wrong state"
                    );
                    assert!(
                        reply.contains(&c.runway),
                        "{cell}: clearance without runway"
                    );
                    assert!(reply.contains(&c.wind), "{cell}: clearance without wind");
                }
                // A landing clearance only ever answers a final report — or a
                // MAYDAY, the one deliberate exception: an emergency outranks
                // the sequence (priority handling is the point of the call).
                if reply.contains("cleared to land") || reply.contains("cleared touch and go") {
                    assert!(
                        matches!(intent, ReportFinal | Mayday),
                        "{cell}: unasked landing clearance"
                    );
                    assert!(
                        reply.contains(&c.runway),
                        "{cell}: clearance without runway"
                    );
                    assert!(reply.contains(&c.wind), "{cell}: clearance without wind");
                }

                // The full transition table — any change here must be a
                // deliberate edit to this test, not a surprise.
                let want = match intent {
                    RadioCheck | WindCheck | SquawkCheck | Unknown | RequestClearance
                    | RequestStartup | RequestShutdown => start,
                    // A hail, flight following, and radio housekeeping are
                    // answered without touching the sequence.
                    Greeting | RequestFlightFollowing | RequestFrequencyChange => start,
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
                        if matches!(start, TaxiCleared | LinedUp | TakeoffCleared) {
                            TakeoffCleared
                        } else {
                            TaxiCleared
                        }
                    }
                    RequestLanding { .. } => Inbound,
                    ReportFinal => ClearedToLand,
                    // Pattern legs and a go-around all land in "in the
                    // pattern, awaiting final"; a mayday is cleared at once.
                    ReportDownwind | ReportBase | ReportCrosswind | GoAround => Inbound,
                    Mayday => ClearedToLand,
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
            assert_eq!(
                twr.state, Inbound,
                "{start:?}: no-approach vectors must join"
            );
        }

        // A ghost ON the strip: a re-ask never repeats the takeoff clearance
        // from ANY of the committed states. Holding-point states hold where
        // they are; an already-cleared aircraft has its clearance CANCELLED
        // and lands in LinedUp (it may be on the pavement), with the release
        // watch owing the re-clearance.
        let mut occupied = ctx();
        occupied.runway_occupied = true;
        for &start in &[TaxiCleared, LinedUp, TakeoffCleared] {
            let mut twr = Tower::new();
            twr.state = start;
            let r = twr.handle(RequestTakeoff, &occupied);
            assert!(
                !r.contains("cleared for takeoff"),
                "{start:?}: cleared onto an occupied runway: {r}"
            );
            assert!(r.contains("hold position"), "{start:?}: {r}");
            if start == TakeoffCleared {
                assert!(r.contains("cancel takeoff clearance"), "{r}");
                assert_eq!(
                    twr.state, LinedUp,
                    "clearance withdrawn, still on the pavement"
                );
            } else {
                assert_eq!(twr.state, start, "{start:?}: hold moves nobody");
            }
        }
    }

    #[test]
    fn mayday_gets_priority_from_any_state() {
        // The real 2026-07-09 sequence: "mayday, mayday" got "say again",
        // twice, and only "requesting emergency landing" happened to carry it
        // via the word "landing". Never again: a mayday is answered with a
        // clearance and emergency services, from anywhere in the sequence.
        for call in [
            "mayday, mayday, mayday",
            "mayday, engine failure",
            "pan pan, pan pan",
            "declaring an emergency",
            "requesting emergency landing",
            "emergency descent",
            "mayday, can you hear me", // the question guard must not eat it
        ] {
            assert_eq!(parse_intent(call), Intent::Mayday, "{call}");
        }
        use State::*;
        for start in [
            Parked,
            TaxiCleared,
            LinedUp,
            TakeoffCleared,
            Inbound,
            ClearedToLand,
        ] {
            let mut twr = Tower::new();
            twr.state = start;
            let r = twr.handle(Intent::Mayday, &ctx());
            assert!(r.contains("roger mayday"), "{start:?}: {r}");
            assert!(r.contains("cleared to land"), "{start:?}: {r}");
            assert!(r.contains("Emergency services"), "{start:?}: {r}");
            assert_eq!(twr.state, State::ClearedToLand);
        }
        // On the GROUND there is nothing to clear: the trucks come to you,
        // the sequence doesn't move, and no landing clearance is spoken
        // (a parked mayday was "cleared to land" live, 2026-07-09).
        let mut ground = ctx();
        ground.on_ground = true;
        let mut twr = Tower::new();
        let r = twr.handle(Intent::Mayday, &ground);
        assert!(r.contains("roger mayday"), "{r}");
        assert!(r.contains("Emergency services"), "{r}");
        assert!(!r.contains("cleared to land"), "{r}");
        assert_eq!(twr.state, State::Parked);
        // But a lone "emergency" without land/descend/declare context is NOT
        // a mayday — "emergency fuel pump on" is a cockpit command.
        assert_eq!(parse_intent("emergency fuel pump on"), Intent::Unknown);
        // And the emergency CHECKLIST is cockpit ritual, not a distress call.
        assert_eq!(parse_intent("emergency landing checklist"), Intent::Unknown);
    }

    #[test]
    fn go_around_voids_the_clearance_and_rejoins() {
        let mut twr = Tower::new();
        let c = ctx();
        twr.handle(parse_intent("inbound for landing"), &c);
        twr.handle(parse_intent("final"), &c);
        assert_eq!(twr.state, State::ClearedToLand);
        let r = twr.handle(parse_intent("november 172SP, going around"), &c);
        assert!(r.contains("go around"), "{r}");
        assert!(r.contains("report downwind"), "{r}");
        assert_eq!(twr.state, State::Inbound, "back in the pattern");
        // The circuit then re-earns the clearance leg by leg.
        let r = twr.handle(parse_intent("downwind"), &c);
        assert!(r.contains("report base"), "{r}");
        let r = twr.handle(parse_intent("turning base"), &c);
        assert!(r.contains("report final"), "{r}");
        let r = twr.handle(parse_intent("final"), &c);
        assert!(r.contains("cleared to land"), "{r}");
        // "missed approach" is the same call in IFR words.
        assert_eq!(parse_intent("missed approach"), Intent::GoAround);
    }

    #[test]
    fn pattern_reports_sequence_the_circuit() {
        assert_eq!(parse_intent("downwind runway 09"), Intent::ReportDownwind);
        assert_eq!(parse_intent("downwind for landing"), Intent::ReportDownwind);
        assert_eq!(parse_intent("turning base"), Intent::ReportBase);
        assert_eq!(
            parse_intent("left base runway two seven"),
            Intent::ReportBase
        );
        assert_eq!(parse_intent("crosswind"), Intent::ReportCrosswind);
        assert_eq!(
            parse_intent("upwind, november 172SP"),
            Intent::ReportCrosswind
        );
        // "base" without pattern context is not a report.
        assert_eq!(parse_intent("base"), Intent::Unknown);
        // A crosswind QUESTION is crew talk, not a position report.
        assert_eq!(parse_intent("what's the max crosswind"), Intent::Unknown);
        let mut twr = Tower::new();
        let r = twr.handle(Intent::ReportCrosswind, &ctx());
        assert!(r.contains("report downwind"), "{r}");
    }

    #[test]
    fn squawk_question_repeats_the_code() {
        let mut twr = Tower::new();
        let r = twr.handle(parse_intent("what about my transponder code?"), &ctx());
        assert!(r.contains("squawk seven zero zero zero"), "{r}");
    }

    /// EGPF, 2026-09-21: the sim starts the aircraft ON the runway, so the
    /// FSM is still Parked. "Ready for departure" from there is a lined-up
    /// aircraft, cleared like one and held like one; from a stand it is still
    /// "taxi first", and airborne the flag means nothing.
    #[test]
    fn a_runway_start_is_a_lined_up_aircraft() {
        let mut on_strip = ctx();
        on_strip.own_on_runway = true;
        on_strip.on_ground = true;
        let mut twr = Tower::new();
        let r = twr.handle(Intent::RequestTakeoff, &on_strip);
        assert!(r.contains("cleared for takeoff"), "{r}");
        assert_eq!(twr.state, State::TakeoffCleared);

        // Traffic on final holds it, exactly as it holds a taxied departure.
        let mut held = busy(1.8);
        held.own_on_runway = true;
        held.on_ground = true;
        let mut twr = Tower::new();
        let r = twr.handle(Intent::RequestTakeoff, &held);
        assert!(r.contains("hold position"), "{r}");
        assert_eq!(twr.state, State::LinedUp);

        // A stand is still a stand.
        let mut twr = Tower::new();
        let r = twr.handle(Intent::RequestTakeoff, &ctx());
        assert!(r.contains("first"), "{r}");
    }

    #[test]
    fn takeoff_clearance_is_cancelled_when_traffic_appears_on_final() {
        // Cleared with a clear final; traffic appears; the re-ask CANCELS
        // (the 2026-07-10 tape had a takeoff and a landing clearance on the
        // same runway eight seconds apart). The aircraft may already be on
        // the pavement, so cancellation lands in LinedUp — the release
        // watch owes it the re-clearance.
        let mut twr = Tower::new();
        twr.state = State::TaxiCleared;
        let r = twr.handle(Intent::RequestTakeoff, &ctx());
        assert!(r.contains("cleared for takeoff"), "{r}");
        let r = twr.handle(Intent::RequestTakeoff, &busy(1.8));
        assert!(r.contains("cancel takeoff clearance"), "{r}");
        assert!(r.contains("hold position"), "{r}");
        assert_eq!(
            twr.state,
            State::LinedUp,
            "clearance withdrawn, still on the pavement"
        );
    }

    #[test]
    fn final_traffic_gates_takeoff_and_sequences_downwind() {
        // Traffic on SHORT final: too close for a conditional — hold, and the
        // state doesn't move.
        let mut twr = Tower::new();
        twr.state = State::TaxiCleared;
        let r = twr.handle(Intent::RequestTakeoff, &busy(0.9));
        assert!(r.contains("hold position"), "{r}");
        assert!(r.contains("short final"), "{r}");
        assert_eq!(twr.state, State::TaxiCleared, "clearance withheld");
        // Final clears on the next ask: the clearance comes.
        let r = twr.handle(Intent::RequestTakeoff, &ctx());
        assert!(r.contains("cleared for takeoff"), "{r}");
        assert_eq!(twr.state, State::TakeoffCleared);

        // Downwind with traffic on short final: number two, follow it.
        let mut twr = Tower::new();
        twr.state = State::Inbound;
        let r = twr.handle(Intent::ReportDownwind, &busy(0.9));
        assert!(r.contains("number two"), "{r}");
        assert!(r.contains("short final"), "{r}");
        // Empty final: number one, as before.
        let r = twr.handle(Intent::ReportDownwind, &ctx());
        assert!(r.contains("number one"), "{r}");
    }

    #[test]
    fn one_arrival_far_enough_earns_a_conditional_line_up() {
        // Exactly one arrival, far enough out: "behind the landing traffic,
        // line up behind" — the departure is staged, not parked.
        let mut twr = Tower::new();
        twr.state = State::TaxiCleared;
        let r = twr.handle(Intent::RequestTakeoff, &busy(2.4));
        assert!(
            r.contains("behind the landing Speedbird three zero six"),
            "{r}"
        );
        assert!(r.contains("line up runway zero niner behind"), "{r}");
        assert!(!r.contains("cleared for takeoff"), "{r}");
        assert_eq!(twr.state, State::LinedUp);
        // The condition must be read back — echoing the line-up without
        // "behind" is challenged, with the condition echoed it's silence.
        let p = twr.pending().expect("conditional awaits its readback");
        assert_eq!(
            check_readback("lining up runway zero niner, november 172SP", &p.awaited),
            ReadbackCheck::NotAReadback,
            "the condition is part of the clearance"
        );
        assert_eq!(
            check_readback(
                "behind the landing traffic, lining up runway zero niner behind, november 172SP",
                &p.awaited
            ),
            ReadbackCheck::Correct
        );
        // A STREAM on final never gets a conditional — ambiguous condition.
        let mut stream = busy(3.0);
        stream.final_traffic_count = 2;
        let mut twr = Tower::new();
        twr.state = State::TaxiCleared;
        let r = twr.handle(Intent::RequestTakeoff, &stream);
        assert!(r.contains("hold position"), "{r}");
        assert_eq!(twr.state, State::TaxiCleared);
        // Re-asking while lined up with traffic still on final: hold, no
        // takeoff clearance, still lined up.
        let mut twr = Tower::new();
        twr.state = State::LinedUp;
        let r = twr.handle(Intent::RequestTakeoff, &busy(2.0));
        assert!(r.contains("hold position"), "{r}");
        assert!(!r.contains("line up"), "{r}");
        assert_eq!(twr.state, State::LinedUp);
        // Final clear: the lined-up aircraft gets the clearance.
        let r = twr.handle(Intent::RequestTakeoff, &ctx());
        assert!(r.contains("cleared for takeoff"), "{r}");
        assert_eq!(twr.state, State::TakeoffCleared);
    }

    #[test]
    fn wake_turbulence_lines_up_and_waits() {
        // A heavier departure just rolled: line up and wait while the two
        // minutes run — the classic use of LUAW.
        let mut wake = ctx();
        wake.wake_secs_left = Some(95);
        let mut twr = Tower::new();
        twr.state = State::TaxiCleared;
        let r = twr.handle(Intent::RequestTakeoff, &wake);
        assert!(r.contains("line up and wait runway zero niner"), "{r}");
        assert!(r.contains("wake turbulence"), "{r}");
        assert!(!r.contains("cleared for takeoff"), "{r}");
        assert_eq!(twr.state, State::LinedUp);
        // Impatience while the clock runs: hold, still no clearance.
        let r = twr.handle(Intent::RequestTakeoff, &wake);
        assert!(r.contains("hold position"), "{r}");
        assert!(r.contains("wake turbulence"), "{r}");
        assert_eq!(twr.state, State::LinedUp);
        // Clock expired: cleared.
        let r = twr.handle(Intent::RequestTakeoff, &ctx());
        assert!(r.contains("cleared for takeoff"), "{r}");
        // An occupied runway blocks the line-up — nobody lines up onto a
        // runway with traffic on it, wake clock or not.
        let mut occupied = ctx();
        occupied.wake_secs_left = Some(60);
        occupied.runway_occupied = true;
        let mut twr = Tower::new();
        twr.state = State::TaxiCleared;
        let r = twr.handle(Intent::RequestTakeoff, &occupied);
        assert!(r.contains("hold position"), "{r}");
        assert_eq!(twr.state, State::TaxiCleared);
    }

    #[test]
    fn readback_gate_holds_the_runway_items() {
        // Takeoff scenarios play out ON the ground — the gate voids a
        // takeoff readback the moment physics says airborne.
        let mut c = ctx();
        c.on_ground = true;
        let mut twr = Tower::new();
        twr.state = State::TaxiCleared;
        let clearance = twr.handle(Intent::RequestTakeoff, &c);
        assert!(clearance.contains("cleared for takeoff"), "{clearance}");
        // A correct readback earns silence and clears the debt.
        let mut t2 = Tower::new();
        t2.state = State::TaxiCleared;
        t2.handle(Intent::RequestTakeoff, &c);
        let g = t2.gate(
            "cleared for takeoff runway zero niner, november 172SP",
            parse_intent("cleared for takeoff runway zero niner, november 172SP"),
            &c,
        );
        assert_eq!(g, Some(String::new()), "correct readback = silence");
        assert!(t2.pending().is_none());
        // The WRONG runway is corrected, never waved through.
        let mut t3 = Tower::new();
        t3.state = State::TaxiCleared;
        t3.handle(Intent::RequestTakeoff, &c);
        let g = t3
            .gate(
                "cleared for takeoff runway two seven, november 172SP",
                Intent::Unknown,
                &c,
            )
            .expect("gate speaks");
        assert!(g.contains("negative"), "{g}");
        assert!(g.contains("I say again"), "{g}");
        assert!(t3.pending().is_some(), "debt stands until the right echo");
        // The callsign's own digits never read as a runway.
        let p = t3.pending().unwrap();
        assert_eq!(
            check_readback(
                "november one seven two sierra papa, cleared for takeoff",
                &p.awaited
            ),
            ReadbackCheck::Correct,
            "no runway named at all is accepted; callsign digits are not a runway"
        );
        // Anything that isn't the echo gets ONE instruction repeat...
        let mut t4 = Tower::new();
        t4.state = State::TaxiCleared;
        t4.handle(Intent::RequestTakeoff, &c);
        let g = t4
            .gate("uhh what was that", Intent::Unknown, &c)
            .expect("challenge");
        assert!(g.contains("read back"), "{g}");
        assert!(
            g.contains("cleared for takeoff"),
            "the challenge repeats the instruction: {g}"
        );
        // ...the question-shaped calls a tower answers pass straight through...
        assert_eq!(t4.gate("say wind", Intent::WindCheck, &c), None);
        assert!(
            t4.pending().is_some(),
            "an answered question keeps the debt"
        );
        // ...and two spent challenges release the gate — a garbled radio must
        // never deadlock the FSM.
        t4.gate("garble garble", Intent::Unknown, &c);
        assert_eq!(
            t4.gate("garble again", Intent::Unknown, &c),
            None,
            "gate lets go"
        );
        assert!(t4.pending().is_none());
        // A mayday is NEVER answered with "read back".
        let mut t5 = Tower::new();
        t5.state = State::TaxiCleared;
        t5.handle(Intent::RequestTakeoff, &c);
        assert_eq!(t5.gate("mayday, mayday", Intent::Mayday, &c), None);
        assert!(t5.pending().is_none());
        // Physics consumes the debt: airborne, the takeoff echo is moot.
        let mut t6 = Tower::new();
        t6.state = State::TaxiCleared;
        let mut ground = c.clone();
        ground.on_ground = true;
        t6.handle(Intent::RequestTakeoff, &ground);
        assert!(t6.pending().is_some());
        let flying = ctx(); // on_ground: false
        assert_eq!(
            t6.gate("some new call downwind", Intent::ReportDownwind, &flying),
            None
        );
        assert!(
            t6.pending().is_none(),
            "airborne voids the takeoff readback"
        );
    }

    #[test]
    fn landing_clearance_awaits_its_readback() {
        let c = ctx();
        let mut twr = Tower::new();
        twr.handle(parse_intent("inbound for landing"), &c);
        twr.handle(parse_intent("final"), &c);
        let p = twr.pending().expect("landing clearance awaits the echo");
        assert_eq!(
            p.awaited,
            Awaited::Landing {
                runway: "zero niner".into(),
                touch_and_go: false
            }
        );
        assert_eq!(
            check_readback(
                "cleared to land runway zero niner, november 172SP",
                &p.awaited
            ),
            ReadbackCheck::Correct
        );
        assert_eq!(
            check_readback("cleared to land runway two seven right", &p.awaited),
            ReadbackCheck::WrongRunway {
                heard: "27R".into()
            }
        );
        // The coach knows the exact line owed.
        let s = twr
            .suggested_readback("November 172SP")
            .expect("a suggestion");
        assert!(s.contains("Cleared to land runway zero niner"), "{s}");
        // A go-around clears the debt along with the clearance.
        twr.handle(Intent::GoAround, &c);
        assert!(twr.pending().is_none());
    }

    #[test]
    fn a_monitor_go_around_voids_the_landing_clearance() {
        // The tower cancelled the clearance itself (traffic on the runway):
        // same outcome as the pilot calling it — Inbound, no readback owed.
        let c = ctx();
        let mut twr = Tower::new();
        twr.handle(parse_intent("inbound for landing"), &c);
        twr.handle(parse_intent("final"), &c);
        assert_eq!(twr.state, State::ClearedToLand);
        assert!(twr.pending().is_some());
        twr.apply_go_around();
        assert_eq!(twr.state, State::Inbound);
        assert!(
            twr.pending().is_none(),
            "the dead clearance's echo is not owed"
        );
    }

    // LEMG 2026-09-12: "final runway one three" answered with "runway one
    // two, cleared to land" and nobody heard the difference.
    #[test]
    fn a_final_report_on_the_wrong_runway_is_corrected_aloud() {
        let c = ctx();
        let mut t = Tower::default();
        t.state = State::Inbound;
        let reply = t.handle(Intent::ReportFinal, &c);
        let fixed = Tower::correct_final_runway(
            "november one seven two sierra papa, final runway one three",
            Intent::ReportFinal,
            &c,
            reply.clone(),
        );
        assert!(
            fixed.starts_with("November one seven two Sierra Papa, negative, runway zero niner, I say again runway zero niner. runway zero niner, wind"),
            "{fixed}"
        );
        assert!(fixed.ends_with("cleared to land."), "{fixed}");
        // The right runway, or none spoken: the reply as it was.
        for t in ["final runway zero niner", "november one seven two, final"] {
            assert_eq!(Tower::correct_final_runway(t, Intent::ReportFinal, &c, reply.clone()), reply, "{t}");
        }
        // Only final reports: a taxi request naming another runway is the
        // parser's business, not this one's.
        assert_eq!(
            Tower::correct_final_runway("request taxi runway one three", Intent::RequestTaxi, &c, reply.clone()),
            reply
        );
    }

    #[test]
    fn canon_runway_normalizes_all_spellings() {
        for (raw, want) in [
            ("zero niner", "9"),
            ("two seven right", "27R"),
            ("27R", "27R"),
            ("27 right", "27R"),
            ("09L", "9L"),
            ("three one", "31"),
        ] {
            assert_eq!(canon_runway(raw).as_deref(), Some(want), "{raw}");
        }
        assert_eq!(canon_runway("alpha"), None);
        // Transcript extraction only trusts digits after the word "runway".
        assert_eq!(
            runway_after_keyword("november one seven two, cleared for takeoff runway two seven"),
            Some("27".into())
        );
        assert_eq!(
            runway_after_keyword("november one seven two, cleared for takeoff"),
            None,
            "callsign digits are not a runway"
        );
    }

    #[test]
    fn taxi_clearance_speaks_the_route() {
        let mut c = ctx();
        c.taxi_via = vec!["Alpha".into(), "Bravo two".into()];
        c.taxi_crossings = vec!["two three".into()];
        let mut twr = Tower::new();
        let r = twr.handle(Intent::RequestTaxi, &c);
        assert!(r.contains("via Alpha, Bravo two"), "{r}");
        assert!(r.contains("cross runway two three"), "{r}");
        // No network: the clearance phrases exactly as before.
        let mut twr = Tower::new();
        let r = twr.handle(Intent::RequestTaxi, &ctx());
        assert!(!r.contains(" via "), "{r}");
        assert!(!r.contains("cross runway"), "{r}");
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

    #[test]
    fn a_heavy_says_so_and_a_medium_does_not() {
        assert_eq!(
            spoken_callsign_wake("D-AIKN", Some(242_000.0)),
            "Delta Alpha India Kilo November heavy",
            "the A330 is a heavy"
        );
        assert_eq!(
            spoken_callsign_wake("G-ABCD", Some(79_000.0)),
            "Golf Alpha Bravo Charlie Delta",
            "the 737 is a medium"
        );
        assert_eq!(
            spoken_callsign_wake("N172SP", None),
            "November one seven two Sierra Papa",
            "unknown mass is no suffix"
        );
        assert_eq!(spoken_callsign_wake("", Some(300_000.0)), "");
    }

    #[test]
    fn arrival_taxi_in_clears_to_the_planned_stand() {
        // Fresh off landing, "taxi to parking" earns a real taxi-in clearance
        // to the planned stand — with the via and any runway crossing — and
        // drops the FSM back to Parked.
        let mut c = ctx();
        c.stand = "stand one two".into();
        c.taxi_via = vec!["Alpha".into(), "Bravo".into()];
        c.taxi_crossings = vec!["zero niner".into()];
        let mut twr = Tower::new();
        twr.state = State::ClearedToLand;
        let reply = twr.handle(Intent::RequestTaxiIn, &c);
        assert!(reply.contains("taxi to stand one two"), "{reply:?}");
        assert!(reply.contains("via Alpha, Bravo"), "{reply:?}");
        assert!(reply.contains("cross runway zero niner"), "{reply:?}");
        assert!(reply.contains("this frequency"), "{reply:?}");
        assert_eq!(twr.state, State::Parked);

        // A generic "request taxi" right after landing is an arrival too.
        let mut twr2 = Tower::new();
        twr2.state = State::ClearedToLand;
        assert!(twr2
            .handle(Intent::RequestTaxi, &c)
            .contains("taxi to stand one two"));

        // No stand planned (field with no parking, or off the network) → the
        // honest flat fallback, never a fabricated stand.
        let mut c3 = ctx();
        c3.stand = String::new();
        let mut twr3 = Tower::new();
        twr3.state = State::ClearedToLand;
        assert!(twr3
            .handle(Intent::RequestTaxiIn, &c3)
            .contains("taxi to parking"));
    }
}
