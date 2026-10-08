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
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Intent {
    RadioCheck,
    RequestTaxi,
    RequestClearance,
    /// "request IFR clearance to EGPH" / "IFR to Edinburgh, ready to copy" /
    /// "request clearance as filed": the instrument departure clearance, a
    /// route, a SID, an altitude and a code. Distinct from the zone
    /// clearance on purpose: a bare "request clearance" with no IFR word and
    /// no destination stays [`Intent::RequestClearance`], exactly as before.
    /// `dest` is the destination as heard: an ICAO code (upper case, a
    /// spelled one already joined: "EGPH") or the name words in lower case
    /// ("edinburgh"). The station resolves it against its airport index;
    /// `None` = the call named none, and the tower asks for it.
    RequestIfrClearance {
        dest: Option<String>,
    },
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
    /// "stopping" / "rejecting takeoff" / "aborting takeoff": a departure
    /// stopping on the runway. A REPORT that ends the takeoff clearance, the
    /// way a go-around ends the landing one. EGKK 2026-09-27: a rejected
    /// 737 stopped on 26L and the tower kept it cleared for takeoff; the
    /// resume heard a stale hold release replay "cleared for takeoff" and its
    /// taxi request was answered "read back: cleared for takeoff".
    RejectingTakeoff,
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
    /// "runway vacated" / "clear of the runway" — an arrival reporting itself
    /// off the strip. A REPORT, not a request: it earns an acknowledgement and
    /// changes nothing, because the call that tells a tower its runway is free
    /// must not also be able to hand out a clearance. Our own crew makes it
    /// from the taxiway before asking for the taxi in, and it earned "say
    /// again" every time — twice in one arrival at Liege (2026-09-22), which
    /// reads as a controller who did not hear the one call they most want.
    RunwayVacated,
    /// "request runway zero eight" / "unable two six, request zero eight for
    /// length": the crew asks for a DIFFERENT runway than the one in use. The
    /// runway itself is read from the words by [`requested_runway`] (the
    /// intent stays `Copy` and a corpus label, like [`stated_runway`]); the
    /// station resolves it against the field and hands the answer down in
    /// [`Ctx::runway_refusal`]. ETND 2026-09-24: an SF50 was cleared onto
    /// 446 m of grass and had no words to ask for the paved 1,240 m beside it.
    RequestRunway,
    /// "request backtrack runway two six left" / "requesting backtrack for
    /// full length": an aircraft ON the runway (a runway start mid-strip) or
    /// at its holding point asking to taxi down the runway to the threshold
    /// and turn round. Answered with a line-up ("backtrack approved runway
    /// ..., line up and wait") when the strip and its final are clear. EGKK
    /// 2026-09-27: a 737 started 1,340 m from the end of 26L, needing 1,897,
    /// had no words for it and was taxied off the runway to its far hold.
    RequestBacktrack,
    /// "cancel IFR" / "cancelling IFR": an IFR flight going VFR. The station
    /// retires the IFR clearance and the flight's IFR standing, so the next
    /// region handoff terminates the service like any VFR flight's.
    CancelIfr,
    /// "unable Kene three Papa, request the Lam five Papa departure": an IFR
    /// flight asking to fly a different SID from the runway it was cleared
    /// on. The SID's words are read by [`requested_departure`] (the station
    /// matches them against the field's SIDs, [`match_sid`]); the station
    /// holds the IFR book and amends the clearance, or says the cleared one
    /// stands. The crew's alternative to flying another SID silently (EGKK
    /// 08R, 2026-09-28, #152).
    RequestSid,
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
    /// "request zone transit" / "transiting your zone northbound": an
    /// aircraft passing through the zone to somewhere else, the answer to the
    /// zone watch's "say callsign and intentions" from a field it is only
    /// crossing. Cleared through with the pressure and "report leaving the
    /// zone"; the sequence is untouched (it neither departs nor lands here).
    /// Until #83 a helicopter route had no words for it and a transit field
    /// challenged it on every cooldown (EDVW-ETND, 2026-09-23); full auto
    /// blackboxed the question unanswered.
    RequestTransit,
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
    /// Which flight that nearest arrival is, when the picture names it
    /// (our own AI traffic, stage D: a traffic flight's picture names ours
    /// and the other traffic). `None` for a ghost or with no traffic: the
    /// "number two, follow the traffic" leader a traffic flight follows.
    pub final_traffic_id: Option<crate::party_line::FlightId>,
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
    /// The aircraft is a helicopter (the sim's own `is_helicopter`): a
    /// "ready for departure" from the ground is cleared from its present
    /// position, never sent to a runway holding point (the S-76 on its pad at
    /// ETHB got "taxi to holding point Foxtrot runway two five", 2026-09-23).
    pub helicopter: bool,
    /// The IFR clearance the station has made up for THIS call, when the
    /// call is an IFR request whose destination the station's index knows.
    /// `None` on every other call, and on an IFR request to a destination it
    /// cannot find: the tower then asks for it and clears nothing.
    pub ifr: Option<IfrOffer>,
    /// A runway request the station could NOT grant: (the runway asked for,
    /// spoken; the reason, spoken). `None` on a grantable request (then
    /// `runway` IS the requested runway and the route is planned to it) and
    /// on every other call. An empty runway means the words named none.
    pub runway_refusal: Option<(String, String)>,
    /// The field's transition LEVEL, feet (apt.dat `1302 transition_level`):
    /// an assigned level at or above it is spoken as a flight level
    /// ([`crate::atis::is_flight_level`]). `None` = unknown, everything in feet.
    pub transition_level_ft: Option<f64>,
    /// The flight has said it stays in the pattern (the station's
    /// `circuits`, noted from THIS call's words before the reply): a zone
    /// clearance request is then answered with the circuit clearance, never
    /// "cleared to leave the zone". `None` on every other flight.
    pub circuit: Option<CircuitOffer>,
}

/// What the station hands the tower to phrase a CIRCUIT clearance: the
/// pattern altitude and the dialect. LEMG 13 (sorties 1790590663 and
/// 1790591256, 2026-09-28): "request clearance, runway one three, for
/// circuits" got "cleared to leave the Malaga zone, not above two thousand
/// feet", and a circuit never leaves the zone.
#[derive(Debug, Clone, PartialEq)]
pub struct CircuitOffer {
    /// Circuit height (ICAO) / pattern altitude (FAA) on the QNH, feet:
    /// [`circuit_height_ft`] of the field.
    pub height_ft: f64,
    /// FAA phraseology ("closed traffic", "pattern altitude") instead of
    /// ICAO / CAP 413 ("circuit height").
    pub faa: bool,
    /// The runway's published pattern side (apt.dat `1101`, #182):
    /// `Some(true)` left, named in the clearance; `None` unpublished.
    pub left: Option<bool>,
    /// The field's data says no circuits (apt.dat `1302 allows_circuits 0`,
    /// #182; LEMG and EGKK): the tower says so, then clears the one asked
    /// for (the bench flies its circuits there).
    pub not_available: bool,
}

/// The circuit height on the QNH: 1,000 ft above the field, to the nearest
/// hundred (LEMG at 52 ft is one thousand one hundred; 598 ft is one
/// thousand six hundred).
pub fn circuit_height_ft(field_elev_ft: f64) -> f64 {
    ((field_elev_ft.max(0.0) + 1000.0) / 100.0).round() * 100.0
}

/// The circuit clearance and its suggested readback, in the field's dialect:
/// the runway, the height, the pressure and the code, then "report ready".
/// ICAO / CAP 413: "runway one three, circuit height one thousand one
/// hundred feet, QNH ..., squawk ..., report ready for departure". FAA:
/// "runway one three, closed traffic approved, pattern altitude one thousand
/// one hundred, altimeter ..., squawk ..., report ready for departure".
pub fn circuit_clearance_lines(ctx: &Ctx, offer: &CircuitOffer) -> (String, String) {
    let alt = crate::atis::spoken_alt(offer.height_ft);
    let qnh = qnh_clause(ctx);
    let side = |faa: bool| match (offer.left, faa) {
        (Some(true), false) => "left hand circuit, ",
        (Some(false), false) => "right hand circuit, ",
        (Some(true), true) => "left ",
        (Some(false), true) => "right ",
        (None, _) => "",
    };
    // A field whose data says no circuits: said first, once per clearance.
    let notice = match (offer.not_available, offer.faa) {
        (false, _) => String::new(),
        (true, false) => format!("circuits are not normally available at {}, ", ctx.airport),
        (true, true) => format!("closed traffic is not normally available at {}, ", ctx.airport),
    };
    if offer.faa {
        let s = side(true);
        let rb = match offer.left {
            Some(true) => "Left closed traffic",
            Some(false) => "Right closed traffic",
            None => "Closed traffic",
        };
        (
            format!(
                "{}, {notice}runway {}, {s}closed traffic approved, pattern altitude {alt}{qnh}, \
                 squawk {}, report ready for departure.",
                ctx.callsign, ctx.runway, ctx.squawk
            ),
            format!(
                "{rb} runway {}, pattern altitude {alt}{qnh}, squawk {}, {}.",
                ctx.runway, ctx.squawk, ctx.callsign
            ),
        )
    } else {
        let s = side(false);
        (
            format!(
                "{}, {notice}runway {}, {s}circuit height {alt} feet{qnh}, squawk {}, \
                 report ready for departure.",
                ctx.callsign, ctx.runway, ctx.squawk
            ),
            format!(
                "Runway {}, {s}circuit height {alt} feet{qnh}, squawk {}, {}.",
                ctx.runway, ctx.squawk, ctx.callsign
            ),
        )
    }
}

/// The IFR departure clearance as issued and, once read back correctly, as
/// published to the crew (`party_line::assigned_clearance`). Every field is
/// something the tower actually said.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IfrClearance {
    /// The destination's ICAO code.
    pub dest: String,
    /// The SID's CIFP ident ("GOSA1C"), `None` when the field has no SID
    /// from the runway in use: the clearance is then "after departure fly
    /// runway heading", never an invented name.
    pub sid: Option<String>,
    /// The runway END in use, as apt.dat names it ("06", "27R").
    pub runway: String,
    /// The initial altitude, feet ([`ifr_initial_altitude_ft`]).
    pub initial_alt_ft: i64,
    /// The transponder code, the one the station issues this flight.
    pub squawk: i64,
}

/// What the station hands the tower to phrase an IFR clearance: the
/// clearance itself plus the spoken bits only the station knows.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IfrOffer {
    pub clearance: IfrClearance,
    /// The destination as a controller says it ("Edinburgh").
    pub dest_spoken: String,
    /// FAA phraseology ("climb and maintain", "then as filed", the
    /// departure frequency) instead of ICAO / CAP 413.
    pub faa: bool,
    /// The departure (else approach) frequency, spoken, for the FAA
    /// clearance's "departure frequency" item; `None` at a tower-only field.
    pub departure_freq: Option<String>,
}

/// The IFR initial altitude: the higher of 5,000 ft and the field elevation
/// plus 3,000, rounded UP to the thousand. One function, so the tower and
/// anything that checks it agree (EGPH at 136 ft gets 5,000; LOWI at 1,906
/// gets 5,000; LSZS at 5,600 gets 9,000).
pub fn ifr_initial_altitude_ft(field_elev_ft: f64) -> i64 {
    let above = ((field_elev_ft.max(0.0) + 3000.0) / 1000.0).ceil() * 1000.0;
    above.max(5000.0) as i64
}

/// A SID's CIFP ident as a controller says it: "GOSA1C" is "Gosa one
/// Charlie", "LEGL1P" is "Legl one Papa". The letters before the number are
/// said as the word they abbreviate; the number and the letter after it are
/// spoken the radio way. An ident with no number is said as it is.
pub fn spoken_sid(ident: &str) -> String {
    let Some(at) = ident.find(|c: char| c.is_ascii_digit()) else {
        return title_case(ident);
    };
    let (name, rest) = ident.split_at(at);
    let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
    let tail: String = rest.chars().skip(digits.len()).collect();
    let mut out = vec![title_case(name)];
    let d = crate::atis::spoken_digits(&digits);
    if !d.is_empty() {
        out.push(d);
    }
    for c in tail.chars().filter(|c| c.is_ascii_alphabetic()) {
        out.push(crate::atis::nato(c.to_ascii_uppercase()).to_string());
    }
    out.retain(|w| !w.is_empty());
    out.join(" ")
}

fn title_case(w: &str) -> String {
    let lower = w.to_lowercase();
    let mut c = lower.chars();
    match c.next() {
        Some(f) => f.to_uppercase().chain(c).collect(),
        None => String::new(),
    }
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
/// The answer to a crew's "going around" after the tower sent it round for
/// an occupant that will not leave on its own clock and offered another
/// runway (`station::resequence_runway`, issue #162): the runway given up
/// and the one to expect. `from` and `to` are spoken.
pub fn go_around_resequenced_line(callsign: &str, from: &str, to: &str) -> String {
    format!(
        "{callsign}, roger, go around. Climb to circuit altitude, runway heading. \
         Runway {from} is blocked, expect runway {to}, report downwind."
    )
}

/// A reply that moves a flight off the runway its clearance named says so
/// first (issue #178): "<callsign>, runway change, runway one three in use.
/// Taxi to holding point ...". `to` is spoken. A reply not opening with the
/// callsign is returned as it is. Pure.
pub fn runway_change_line(callsign: &str, to: &str, reply: &str) -> String {
    let Some(rest) = reply.strip_prefix(&format!("{callsign}, ")) else {
        return reply.to_string();
    };
    let mut chars = rest.chars();
    let rest = match chars.next() {
        Some(c) => c.to_uppercase().chain(chars).collect::<String>(),
        None => return reply.to_string(),
    };
    format!("{callsign}, runway change, runway {to} in use. {rest}")
}

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
    // "Cancel IFR" / "cancelling IFR": before every other reading, the
    // clearance word included ("cancel my IFR clearance" is not a request
    // for one). Whisper writes the letters spaced or dotted.
    if (word("cancel") || word("cancelling") || word("canceling"))
        && (word("ifr") || t.contains("i f r") || t.contains("i.f.r") || t.contains("instrument flight"))
    {
        return Intent::CancelIfr;
    }
    // A line-up phrase is a READBACK of "line up and wait", never a request —
    // and "behind the landing traffic, lining up" must not read the word
    // "landing" as an arrival call (corpus 2026-07-11).
    if t.contains("line up") || t.contains("lining up") || t.contains("line-up") {
        return Intent::Unknown;
    }
    // "Cleared ..." is a READBACK of a clearance, never a request: a pilot
    // asking says "request clearance", never "cleared". The route already
    // silenced these, but the intent said RequestLanding / RequestTakeoff,
    // and an unkeyed one could still reach the tower as that request (EGKK
    // 26L, 2026-09-27: "..., cleared to land runway two six left" logged as
    // a landing request in ClearedToLand). A go-around or a reject said in
    // the same breath is still that report.
    if word("cleared")
        && !t.contains("going around")
        && !t.contains("go around")
        && !rejecting_takeoff(&t)
    {
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
    // A zone transit, before the IFR clearance and every runway and landing
    // word: "request clearance to transit the zone" names no destination,
    // and "transiting to Diepholz for landing" is still a transit HERE. The
    // readback ("cleared to transit") was already stopped by the guard above.
    if transit_request(&t) {
        return Intent::RequestTransit;
    }
    // The IFR clearance, right after the question guard and before every
    // runway word: a destination's name must never read as a runway call
    // ("clearance to Lands End" is not a landing request), and "ready to
    // copy" is the IFR pilot's whole request.
    if let Some(dest) = ifr_request(&t) {
        return Intent::RequestIfrClearance { dest };
    }
    // A named departure procedure ("request the Lam five Papa departure"),
    // before "departure" can read as a takeoff or a runway request.
    if requested_departure(&t).is_some() {
        return Intent::RequestSid;
    }
    // A request for a named runway ("request runway zero eight", "unable two
    // six, request zero eight left for length"), before the takeoff, landing
    // and taxi words below can claim "for takeoff" or "for departure". The
    // designator must follow the request word itself: "request taxi, runway
    // two seven" stays the taxi request with a stated runway.
    // A backtrack request, before the runway request can read "request
    // backtrack runway two six left" as a request for runway two six left.
    // Request-shaped only: "backtracking" is the readback of the approval.
    if (word("backtrack") || t.contains("back track") || t.contains("back-track"))
        && (word("request") || word("requesting") || word("requests"))
    {
        return Intent::RequestBacktrack;
    }
    if requested_runway(&t).is_some() {
        return Intent::RequestRunway;
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
    // A departure stopping on the runway, before the takeoff words below can
    // read "rejecting takeoff" as a takeoff REQUEST. "Stopping" is the ICAO
    // word for it (Doc 4444, CAP 413); "rejecting/aborting takeoff" is how
    // crews also say it.
    if rejecting_takeoff(&t) {
        return Intent::RejectingTakeoff;
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
    // "runway vacated" / "clear of the runway" — off the strip, reported.
    // Anything that also ASKS is that request instead: the station's own
    // suggested wording is "runway vacated, request taxi to parking", and the
    // parking rule above owns it. "cleared of" is not "clear of" — a landing
    // clearance read back must not become a vacate report.
    if (word("vacated")
        || word("vacating")
        || t.contains("clear of the runway")
        || t.contains("clear of runway"))
        && !t.contains("taxi")
        && !t.contains("request")
    {
        return Intent::RunwayVacated;
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

/// Words that, straight after "clearance to", say what the clearance is FOR
/// rather than where it goes: "clearance to leave the zone" is the zone
/// clearance, "clearance to start" the start-up, "clearance to cross" a
/// runway crossing. None of them is a destination.
const NOT_A_DESTINATION: &[&str] = &[
    "leave", "enter", "cross", "transit", "depart", "start", "taxi", "push", "pushback", "land",
    "zone", "join", "climb", "descend", "proceed", "fly", "go", "be", "copy", "the", "a", "hold",
    "line", "backtrack", "switch", "shut", "change", "contact", "take", "operate", "takeoff",
    "departure", "vacate", "park", "parking", "runway", "taxiway", "stand", "gate", "ramp",
    "apron", "holding", "startup", "engines", "engine", "shutdown", "power", "run", "use",
];

/// Where a destination's words stop: the rest of the call.
const DEST_STOPS: &[&str] = &[
    "ready", "copy", "request", "requesting", "as", "filed", "with", "information", "via",
    "please", "over", "and", "runway", "ifr", "clearance", "flight", "plan", "for", "to",
    "squawk", "november", "callsign", "departure", "on", "at",
];

/// Filler words dropped from a destination ("to the Edinburgh airport").
const DEST_FILLER: &[&str] = &["the", "airport", "airfield", "international", "intl", "field"];

/// The spelling alphabet as whisper writes it: one letter per word.
fn phonetic_letter(w: &str) -> Option<char> {
    Some(match w {
        "alpha" | "alfa" => 'A',
        "bravo" => 'B',
        "charlie" => 'C',
        "delta" => 'D',
        "echo" => 'E',
        "foxtrot" => 'F',
        "golf" => 'G',
        "hotel" => 'H',
        "india" => 'I',
        "juliet" | "juliett" => 'J',
        "kilo" => 'K',
        "lima" => 'L',
        "mike" => 'M',
        "november" => 'N',
        "oscar" => 'O',
        "papa" => 'P',
        "quebec" => 'Q',
        "romeo" => 'R',
        "sierra" => 'S',
        "tango" => 'T',
        "uniform" => 'U',
        "victor" => 'V',
        "whiskey" | "whisky" => 'W',
        "xray" => 'X',
        "yankee" => 'Y',
        "zulu" => 'Z',
        _ => return None,
    })
}

/// The destination words after a "to", as heard: a spelled ICAO joined
/// ("echo golf papa hotel" is "EGPH"), a written one upper-cased ("egph" is
/// "EGPH"; a three-letter one stays three letters and the station tries the
/// K too), else the name words lower-case ("edinburgh"). `None` when the
/// words say what the clearance is for instead ("to leave the zone").
fn destination_after(words: &[&str]) -> Option<String> {
    let first = *words.first()?;
    if NOT_A_DESTINATION.contains(&first) && first != "the" {
        return None;
    }
    // "to the zone" / "to the holding point" is not a place either.
    if first == "the"
        && words
            .get(1)
            .is_some_and(|w| NOT_A_DESTINATION.contains(w) || *w == "holding" || *w == "zone")
    {
        return None;
    }
    // Four spelled letters (phonetic words or single letters) are a code.
    let spelled: Vec<char> = words
        .iter()
        .take(4)
        .map_while(|w| {
            phonetic_letter(w).or_else(|| {
                (w.len() == 1 && w.chars().all(|c| c.is_ascii_alphabetic()))
                    .then(|| w.chars().next().unwrap().to_ascii_uppercase())
            })
        })
        .collect();
    if spelled.len() == 4 {
        return Some(spelled.into_iter().collect());
    }
    let name: Vec<&str> = words
        .iter()
        .copied()
        .take_while(|w| !DEST_STOPS.contains(w))
        .filter(|w| !DEST_FILLER.contains(w))
        .take(4)
        .collect();
    if name.is_empty() || name.iter().any(|w| w.chars().any(|c| c.is_ascii_digit())) {
        return None;
    }
    // One short token is carried as a code ("egph" is "EGPH"); the station
    // tries it as a code first and as a name ("nice", "bern") second.
    if name.len() == 1 && (3..=4).contains(&name[0].len()) {
        return Some(name[0].to_ascii_uppercase());
    }
    Some(name.join(" "))
}

/// Is this an IFR clearance request, and to where? `Some(None)` = the IFR
/// shape with no destination named ("ready to copy", "request clearance as
/// filed"); `Some(Some(dest))` = one named; `None` = not the IFR call (a bare
/// "request clearance" is the ZONE clearance and stays so).
fn ifr_request(t: &str) -> Option<Option<String>> {
    let toks: Vec<&str> = t
        .split(|c: char| !c.is_alphanumeric())
        .filter(|s| !s.is_empty())
        .collect();
    let has = |w: &str| toks.contains(&w);
    let ifr = has("ifr") || t.contains("i f r");
    let clearance = has("clearance");
    let copy = t.contains("ready to copy");
    let filed = t.contains("as filed");
    // The destination: the words after the first "to" that follows the
    // clearance or IFR word (the one in "ready to copy" names nothing).
    let anchor = toks
        .iter()
        .position(|w| *w == "clearance" || *w == "ifr")
        .or_else(|| copy.then_some(0));
    let dest = anchor.and_then(|a| {
        // Only the FIRST "to": "clearance to taxi to holding point Alpha"
        // is a taxi request, and its second "to" names no destination.
        let i = toks.iter().skip(a).position(|w| *w == "to")? + a;
        destination_after(&toks[i + 1..])
    });
    if ifr && (clearance || copy || dest.is_some() || has("request") || has("requesting")) {
        return Some(dest);
    }
    if copy || (filed && clearance) {
        return Some(dest);
    }
    // "request clearance to Edinburgh": a clearance to a PLACE is the IFR
    // one; "clearance to leave the zone" names no place and stays the zone
    // clearance.
    if clearance && dest.is_some() {
        return Some(dest);
    }
    None
}

/// A zone transit (lower case in): "transit" / "transiting" as a word, or
/// crossing or passing THROUGH the zone. "Cross runway two seven" is a
/// runway crossing, never this: the zone word is required with "cross".
fn transit_request(t: &str) -> bool {
    let words: Vec<&str> = t
        .split(|c: char| !c.is_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    let has = |w: &str| words.contains(&w);
    let zone = has("zone") || t.contains("control zone") || has("ctr");
    has("transit")
        || has("transiting")
        || ((has("cross") || has("crossing") || has("through")) && zone)
}

/// "stopping", "rejecting takeoff", "aborting the takeoff" (lower case in):
/// a departure calling its own stop. The takeoff word must sit with the
/// reject word so "abort" alone (a cockpit word) never reaches it; "stopping"
/// alone is the standard call.
fn rejecting_takeoff(t: &str) -> bool {
    let words: Vec<&str> = t
        .split(|c: char| !c.is_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    let has = |w: &str| words.contains(&w);
    let takeoff = t.contains("takeoff") || t.contains("take off") || t.contains("take-off");
    has("stopping")
        || (takeoff
            && [
                "reject",
                "rejecting",
                "rejected",
                "abort",
                "aborting",
                "aborted",
            ]
            .iter()
            .any(|w| has(w)))
}

/// Parse a corpus label ("request_taxi") into the intent it names — the shared
/// vocabulary of the battle-test manifests (`bench/atc_corpus.yaml`, the ATC
/// cases in the stt-eval manifest). `None` for a label no intent carries.
pub fn intent_from_label(label: &str) -> Option<Intent> {
    Some(match label {
        "radio_check" => Intent::RadioCheck,
        "request_taxi" => Intent::RequestTaxi,
        "request_clearance" => Intent::RequestClearance,
        // The destination is the case's own field (`dest` in the corpus).
        "request_ifr_clearance" => Intent::RequestIfrClearance { dest: None },
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
        "rejecting_takeoff" => Intent::RejectingTakeoff,
        "mayday" => Intent::Mayday,
        "wind_check" => Intent::WindCheck,
        "squawk_check" => Intent::SquawkCheck,
        "request_taxi_in" => Intent::RequestTaxiIn,
        "runway_vacated" => Intent::RunwayVacated,
        "request_runway" => Intent::RequestRunway,
        "request_backtrack" => Intent::RequestBacktrack,
        "cancel_ifr" => Intent::CancelIfr,
        "request_sid" => Intent::RequestSid,
        "request_vectors" => Intent::RequestVectors,
        "greeting" => Intent::Greeting,
        "flight_following" => Intent::RequestFlightFollowing,
        "frequency_change" => Intent::RequestFrequencyChange,
        "request_transit" => Intent::RequestTransit,
        "unknown" => Intent::Unknown,
        _ => return None,
    })
}

/// Is this transmission the pilot asking the controller to REPEAT its last
/// transmission ("say again", "Gibraltar Tower, say again all after runway",
/// "repeat your last")? Until 2026-09-23 it parsed as `Unknown` and every
/// one was answered "say again" back, clearance or not, so a clearance the
/// pilot keyed over (a press cuts the station, no resume) was gone for good.
/// The station half of the answer lives with the audio (the crew side knows
/// what actually went out on the air); a request that also asks for
/// something ("say again, request taxi") is that request instead.
pub fn is_say_again(transcript: &str) -> bool {
    let t = transcript.to_lowercase();
    let asks = t.contains("say again")
        || t.contains("repeat your last")
        || t.contains("repeat the last")
        || t.contains("repeat last");
    asks && !t.contains("request") && !t.contains("ready")
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
    /// The IFR clearance: the SID (when one was given), the altitude and the
    /// squawk must all come back, and a runway named must be ours. The
    /// spoken bits ride along so the coach and the correction say them the
    /// way the clearance did.
    IfrClearance {
        clearance: IfrClearance,
        dest_spoken: String,
        runway_spoken: String,
        faa: bool,
        /// The initial altitude was given as a flight level (at or above the
        /// field's transition level), so the readback says it that way too.
        level: bool,
    },
}

/// One item of the IFR clearance, for the readback's verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IfrItem {
    Departure,
    Altitude,
    Squawk,
    Runway,
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
    /// An IFR clearance read back with items WRONG (a different altitude,
    /// code or runway) or MISSING (not said at all). Both earn the
    /// correction; neither earns the clearance.
    IfrItems {
        wrong: Vec<IfrItem>,
        missing: Vec<IfrItem>,
    },
    NotAReadback,
}

/// A number said digit by digit or written ("four five two one", "4521",
/// "niner"), read from `toks[i..]`: the digits and how many tokens they
/// took. At most `max` digits.
fn digits_at(toks: &[String], i: usize, max: usize) -> (String, usize) {
    let digit = |tok: &str| -> Option<&'static str> {
        Some(match tok {
            // Not "to" or "for": "climb to five thousand" is five.
            "zero" => "0",
            "one" => "1",
            "two" => "2",
            "three" | "tree" => "3",
            "four" | "fower" => "4",
            "five" | "fife" => "5",
            "six" => "6",
            "seven" => "7",
            "eight" => "8",
            "nine" | "niner" => "9",
            _ => return None,
        })
    };
    let mut out = String::new();
    let mut j = i;
    while j < toks.len() && out.len() < max {
        let t = toks[j].as_str();
        if let Some(d) = digit(t) {
            out.push_str(d);
        } else if !t.is_empty() && t.chars().all(|c| c.is_ascii_digit()) {
            if out.len() + t.len() > max {
                break;
            }
            out.push_str(t);
        } else {
            break;
        }
        j += 1;
    }
    (out, j - i)
}

/// Tokens with the thousands separators whisper writes ("5,000") closed up,
/// so a written altitude is one token.
fn number_tokens(t: &str) -> Vec<String> {
    let chars: Vec<char> = t.chars().collect();
    let mut closed = String::with_capacity(t.len());
    for (k, &c) in chars.iter().enumerate() {
        let between_digits = k > 0
            && chars[k - 1].is_ascii_digit()
            && chars.get(k + 1).is_some_and(|n| n.is_ascii_digit());
        if c == ',' && between_digits {
            continue;
        }
        closed.push(c);
    }
    lower_tokens(&closed)
}

/// The squawk a readback says: the four digits after "squawk" (or
/// "transponder", "code"). `None` when no code was said.
pub fn heard_squawk(transcript: &str) -> Option<i64> {
    let toks = number_tokens(&transcript.to_lowercase());
    for (i, w) in toks.iter().enumerate() {
        if matches!(w.as_str(), "squawk" | "squawking" | "transponder" | "code") {
            let (d, _) = digits_at(&toks, i + 1, 4);
            if d.len() == 4 {
                return d.parse().ok();
            }
        }
    }
    None
}

/// The altitude a readback says, feet: "five thousand", "one zero thousand
/// five hundred", "5000", "5,000 feet". The squawk's four digits are never
/// read as one (they follow "squawk"). `None` when no altitude was said.
pub fn heard_altitude_ft(transcript: &str) -> Option<i64> {
    let toks = number_tokens(&transcript.to_lowercase());
    let tens = |w: &str| -> Option<i64> {
        Some(match w {
            "ten" => 10,
            "eleven" => 11,
            "twelve" => 12,
            "thirteen" => 13,
            "fourteen" => 14,
            "fifteen" => 15,
            "sixteen" => 16,
            "seventeen" => 17,
            "eighteen" => 18,
            "nineteen" => 19,
            _ => return None,
        })
    };
    // Where the squawk's digits sit, so they are skipped.
    let mut skip = vec![false; toks.len()];
    for (i, w) in toks.iter().enumerate() {
        if matches!(w.as_str(), "squawk" | "squawking" | "transponder" | "code") {
            let (_, n) = digits_at(&toks, i + 1, 4);
            for s in skip.iter_mut().skip(i + 1).take(n) {
                *s = true;
            }
        }
    }
    // A flight level: "flight level one two zero", "flight level 120",
    // "FL120", "FL 120" is feet on the standard setting (12,000). Before the
    // thousands, so a level said in the same breath as an altitude is the
    // level assigned.
    if let Some(ft) = heard_flight_level_ft(&toks, &skip) {
        return Some(ft);
    }
    for (i, w) in toks.iter().enumerate() {
        if skip[i] {
            continue;
        }
        if w == "thousand" {
            // The thousands: the digits just before (spoken or written), or
            // a teen word ("twelve thousand").
            let mut start = i;
            while start > 0 && !skip[start - 1] && digits_at(&toks, start - 1, 1).1 == 1 {
                start -= 1;
                if i - start >= 2 {
                    break;
                }
            }
            let th: Option<i64> = if start < i {
                digits_at(&toks, start, 2).0.parse().ok()
            } else {
                i.checked_sub(1).and_then(|k| tens(&toks[k]))
            };
            let Some(th) = th else { continue };
            let mut ft = th * 1000;
            let (h, n) = digits_at(&toks, i + 1, 1);
            if n == 1 && toks.get(i + 2).is_some_and(|w| w == "hundred") {
                ft += h.parse::<i64>().unwrap_or(0) * 100;
            }
            return Some(ft);
        }
        // A written altitude ("5000").
        if w.len() >= 4 && w.len() <= 5 && w.chars().all(|c| c.is_ascii_digit()) {
            if let Ok(v) = w.parse::<i64>() {
                // Altitudes are whole hundreds; a written tail number
                // ("12345") is not one.
                if (1000..=60000).contains(&v) && v % 100 == 0 {
                    return Some(v);
                }
            }
        }
    }
    None
}

/// The flight level a transmission names, in feet: two or three digits
/// after "flight level" (or "level" on its own, which whisper leaves when it
/// eats "flight"), or after "fl"; "fl120" written as one token too. `None`
/// when no level is said. Levels are 10 to 600.
fn heard_flight_level_ft(toks: &[String], skip: &[bool]) -> Option<i64> {
    let level = |d: &str| -> Option<i64> {
        let n: i64 = d.parse().ok()?;
        (d.len() >= 2 && (10..=600).contains(&n)).then_some(n * 100)
    };
    for (i, w) in toks.iter().enumerate() {
        if skip.get(i).copied().unwrap_or(false) {
            continue;
        }
        if w == "level" || w == "fl" {
            let (d, _) = digits_at(toks, i + 1, 3);
            if let Some(ft) = level(&d) {
                return Some(ft);
            }
        }
        if let Some(rest) = w.strip_prefix("fl") {
            if !rest.is_empty() && rest.chars().all(|c| c.is_ascii_digit()) {
                if let Some(ft) = level(rest) {
                    return Some(ft);
                }
            }
        }
    }
    None
}

/// Does a readback name this SID? Lenient on the name (whisper spells a fix
/// name any way it likes): the name's word, or its number-and-letter
/// designator ("one Charlie", "1C") is enough.
fn heard_sid(transcript: &str, ident: &str) -> bool {
    let toks = number_tokens(&transcript.to_lowercase());
    let Some(at) = ident.find(|c: char| c.is_ascii_digit()) else {
        let want = ident.to_lowercase();
        return toks.contains(&want);
    };
    let (name, rest) = ident.split_at(at);
    let name = name.to_lowercase();
    let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
    let letter = rest
        .chars()
        .skip(digits.len())
        .find(|c| c.is_ascii_alphabetic())
        .map(|c| c.to_ascii_lowercase());
    // The name: the whole abbreviation, or a word it begins ("gosam" for
    // "GOSA"), four letters at least so a short one cannot match noise.
    if name.len() >= 3
        && toks
            .iter()
            .any(|w| *w == name || (w.len() >= 4 && name.len() >= 4 && w.starts_with(&name)))
    {
        return true;
    }
    // The designator: "one charlie" / "1 charlie" / "1c".
    let Some(letter) = letter else { return false };
    let nato = crate::atis::nato(letter.to_ascii_uppercase()).to_lowercase();
    let joined = format!("{digits}{letter}");
    toks.iter().enumerate().any(|(i, w)| {
        if *w == joined {
            return true;
        }
        let (d, n) = digits_at(&toks, i, digits.len());
        n > 0
            && d == digits
            && toks
                .get(i + n)
                .is_some_and(|l| *l == nato || (l.len() == 1 && l.starts_with(letter)))
    })
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

/// Does a departure request say the flight stays in the pattern ("request
/// taxi for circuits", "ready for departure, closed traffic", "remain in the
/// pattern")? Read from the words beside the intent, like
/// [`requested_runway`]: the station keeps such a flight on the tower (no
/// "contact Departure", no radar watch). LEMG 31 (sortie 1790584715,
/// 2026-09-28): a right-hand circuit was sent to Malaga Departure on the
/// crosswind leg, because nothing on the radio said it was a circuit.
pub fn wants_circuits(transcript: &str) -> bool {
    let t = transcript.to_lowercase();
    let word = |w: &str| t.split(|c: char| !c.is_alphanumeric()).any(|tok| tok == w);
    word("circuit")
        || word("circuits")
        || t.contains("closed traffic")
        || t.contains("closed pattern")
        || t.contains("pattern work")
        || t.contains("remain in the pattern")
        || t.contains("stay in the pattern")
        || t.contains("remain in the circuit")
}

/// The runway a REQUEST names, canonical: the designator right after
/// "request"/"requesting" (optionally "request runway ..."). `None` when the
/// request is for something else ("request taxi, runway two seven").
pub fn requested_runway(transcript: &str) -> Option<String> {
    let toks = lower_tokens(transcript);
    toks.iter().enumerate().find_map(|(i, w)| {
        if !matches!(w.as_str(), "request" | "requesting" | "requests") {
            return None;
        }
        let rest = &toks[i + 1..];
        match rest.first().map(String::as_str) {
            Some("runway") => designator_at(&rest[1..]),
            _ => designator_at(rest),
        }
    })
}

/// The SID a transmission asks for: the words between the LAST request word
/// and "departure" ("unable Kene three Papa, request the Lam five Papa
/// departure" is "lam five papa"). A name word, then a number spoken or
/// written, then letters; never a runway, a taxi or a clearance ("request
/// runway two six left for departure" is the runway request). `None` when
/// the call names none. Pure.
pub fn requested_departure(transcript: &str) -> Option<String> {
    let toks = lower_tokens(transcript);
    let req = toks
        .iter()
        .rposition(|w| matches!(w.as_str(), "request" | "requesting" | "requests"))?;
    let dep = toks.iter().skip(req + 1).position(|w| w == "departure")? + req + 1;
    let mut words: Vec<&str> = toks[req + 1..dep].iter().map(String::as_str).collect();
    if words.first() == Some(&"the") {
        words.remove(0);
    }
    const NOT_A_SID: [&str; 14] = [
        "runway", "taxi", "for", "to", "via", "and", "clearance", "ifr", "vfr", "immediate",
        "takeoff", "backtrack", "a", "the",
    ];
    let digit = |w: &str| spoken_digit(w).is_some();
    // An ident written whole ("lam5p").
    let typed = |w: &str| {
        w.starts_with(|c: char| c.is_ascii_alphabetic()) && w.chars().any(|c| c.is_ascii_digit())
    };
    let ok = match words.as_slice() {
        [one] => typed(one),
        _ => {
            words.len() >= 2
                && words.iter().all(|w| !NOT_A_SID.contains(w))
                && words.iter().any(|w| digit(w))
                && !digit(words[0])
        }
    };
    ok.then(|| words.join(" "))
}

/// A digit as spoken or written ("five", "niner", "nine", "5"). Pure.
fn spoken_digit(w: &str) -> Option<char> {
    Some(match w {
        "zero" | "0" => '0',
        "one" | "1" => '1',
        "two" | "2" => '2',
        "three" | "3" => '3',
        "four" | "4" => '4',
        "five" | "5" => '5',
        "six" | "6" => '6',
        "seven" | "7" => '7',
        "eight" | "8" => '8',
        "nine" | "niner" | "9" => '9',
        _ => return None,
    })
}

/// Which of `idents` (the field's SIDs) the words of [`requested_departure`]
/// name: the name word begins with the ident's letters ("lam", "lambourne"
/// for LAM5P), the digits match, and the NATO letters spell the suffix; an
/// ident typed whole ("lam5p") also names it. `None` when none, or more than
/// one, fits. Pure.
pub fn match_sid<'a>(words: &str, idents: &'a [String]) -> Option<&'a str> {
    let toks: Vec<&str> = words.split_whitespace().collect();
    let read = |ident: &str| -> bool {
        if toks.len() == 1 {
            return toks[0].eq_ignore_ascii_case(ident);
        }
        let at = ident.find(|c: char| c.is_ascii_digit()).unwrap_or(ident.len());
        let (name, rest) = ident.split_at(at);
        let Some(first) = toks.first() else { return false };
        if name.is_empty() || !first.starts_with(&name.to_ascii_lowercase()) {
            return false;
        }
        let mut digits = String::new();
        let mut letters = String::new();
        for w in &toks[1..] {
            if let Some(d) = spoken_digit(w) {
                if !letters.is_empty() {
                    return false;
                }
                digits.push(d);
            } else if w.chars().count() == 1 {
                letters.push(w.chars().next().unwrap_or(' ').to_ascii_uppercase());
            } else {
                let Some(c) = w.chars().next() else { return false };
                let c = c.to_ascii_uppercase();
                if !crate::atis::nato(c).eq_ignore_ascii_case(w) {
                    return false;
                }
                letters.push(c);
            }
        }
        rest.eq_ignore_ascii_case(&format!("{digits}{letters}"))
    };
    let mut hits = idents.iter().filter(|i| read(i));
    let first = hits.next()?;
    if hits.next().is_some() {
        return None;
    }
    Some(first.as_str())
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
    if let Awaited::IfrClearance { clearance, .. } = awaited {
        return check_ifr_readback(&t, clearance);
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
        Awaited::IfrClearance { .. } => unreachable!("judged above"),
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
        Awaited::HoldPosition | Awaited::IfrClearance { .. } => None,
    };
    if let (Some(e), Some(h)) = (expect, runway_after_keyword(&t)) {
        if e != h {
            return ReadbackCheck::WrongRunway { heard: h };
        }
    }
    ReadbackCheck::Correct
}

/// The IFR readback, item by item: the SID (if one was given), the
/// altitude, the squawk, and a runway if one is named. Nothing of the
/// clearance heard at all is no readback (a mishear, a new call): it earns
/// the challenge, never the clearance.
fn check_ifr_readback(t: &str, c: &IfrClearance) -> ReadbackCheck {
    let alt = heard_altitude_ft(t);
    let code = heard_squawk(t);
    let sid_heard = c.sid.as_deref().map(|s| heard_sid(t, s));
    let rwy = runway_after_keyword(t);
    if alt.is_none() && code.is_none() && sid_heard != Some(true) {
        return ReadbackCheck::NotAReadback;
    }
    let mut wrong = Vec::new();
    let mut missing = Vec::new();
    if sid_heard == Some(false) {
        missing.push(IfrItem::Departure);
    }
    if let (Some(h), Some(e)) = (rwy, canon_runway(&c.runway)) {
        if h != e {
            wrong.push(IfrItem::Runway);
        }
    }
    match alt {
        Some(a) if a != c.initial_alt_ft => wrong.push(IfrItem::Altitude),
        Some(_) => {}
        None => missing.push(IfrItem::Altitude),
    }
    match code {
        Some(k) if k != c.squawk => wrong.push(IfrItem::Squawk),
        Some(_) => {}
        None => missing.push(IfrItem::Squawk),
    }
    if wrong.is_empty() && missing.is_empty() {
        ReadbackCheck::Correct
    } else {
        ReadbackCheck::IfrItems { wrong, missing }
    }
}

/// The climb item of an IFR clearance, in the dialect and the level's own
/// words: ICAO "climb five thousand feet" / "climb flight level one two
/// zero"; FAA "climb and maintain five thousand" / "climb and maintain
/// flight level one eight zero". Shared by the clearance, its readback and
/// the correction, so the three always say the level the same way.
pub fn climb_item(alt_ft: i64, level: bool, faa: bool) -> String {
    let words = if level {
        crate::atis::spoken_flight_level(alt_ft as f64)
    } else {
        crate::atis::spoken_alt(alt_ft as f64)
    };
    match (faa, level) {
        (true, _) => format!("climb and maintain {words}"),
        (false, true) => format!("climb {words}"),
        (false, false) => format!("climb {words} feet"),
    }
}

/// The IFR clearance, phrased in the field's dialect: the sentence the
/// tower says (with the callsign) and the instruction a challenge repeats
/// (without it). ICAO / CAP 413: "cleared to Edinburgh via the Gosa one
/// Charlie departure, runway two four, climb five thousand feet, squawk four
/// five two one, QNH ...". FAA: "cleared to Chicago airport via the ...
/// departure, then as filed, climb and maintain five thousand, departure
/// frequency ..., squawk ...". No SID: "after departure fly runway heading".
pub fn ifr_clearance_line(ctx: &Ctx, offer: &IfrOffer) -> (String, String) {
    let c = &offer.clearance;
    let level = crate::atis::is_flight_level(c.initial_alt_ft as f64, ctx.transition_level_ft);
    let climb = climb_item(c.initial_alt_ft, level, offer.faa);
    let code = crate::atis::spoken_digits(&format!("{:04}", c.squawk));
    let body = if offer.faa {
        let route = match &c.sid {
            Some(sid) => format!("via the {} departure, then as filed", spoken_sid(sid)),
            None => "as filed, after departure fly runway heading".to_string(),
        };
        let freq = offer
            .departure_freq
            .as_ref()
            .map(|f| format!(", departure frequency {f}"))
            .unwrap_or_default();
        format!(
            "cleared to {} airport {route}, {climb}{freq}, squawk {code}.",
            offer.dest_spoken
        )
    } else {
        let route = match &c.sid {
            Some(sid) => format!(" via the {} departure, runway {}", spoken_sid(sid), ctx.runway),
            None => format!(", runway {}, after departure fly runway heading", ctx.runway),
        };
        format!(
            "cleared to {}{route}, {climb}, squawk {code}{}.",
            offer.dest_spoken,
            qnh_clause(ctx)
        )
    };
    (format!("{}, {body}", ctx.callsign), body)
}

/// The IFR readback a pilot owes, phrased: the coach's line and the crew's
/// transmission. Items only (the QNH and the frequency are not enforced).
fn ifr_readback_line(
    c: &IfrClearance,
    dest_spoken: &str,
    runway_spoken: &str,
    faa: bool,
    level: bool,
    callsign: &str,
) -> String {
    let climb = climb_item(c.initial_alt_ft, level, faa);
    let code = crate::atis::spoken_digits(&format!("{:04}", c.squawk));
    if faa {
        let route = match &c.sid {
            Some(sid) => format!("via the {} departure, then as filed", spoken_sid(sid)),
            None => "as filed, runway heading".to_string(),
        };
        format!("Cleared to {dest_spoken} {route}, {climb}, squawk {code}, {callsign}.")
    } else {
        let route = match &c.sid {
            Some(sid) => format!(" via the {} departure, runway {runway_spoken}", spoken_sid(sid)),
            None => format!(", runway {runway_spoken}, runway heading"),
        };
        format!("Cleared to {dest_spoken}{route}, {climb}, squawk {code}, {callsign}.")
    }
}

/// The correction for an IFR readback with wrong or missing items: the
/// wrong ones said again after "negative", the missing ones asked for.
fn ifr_correction(
    callsign: &str,
    awaited: &Awaited,
    wrong: &[IfrItem],
    missing: &[IfrItem],
) -> String {
    let Awaited::IfrClearance {
        clearance: c,
        runway_spoken,
        faa,
        level,
        ..
    } = awaited
    else {
        return format!("{callsign}, say again.");
    };
    let phrase = |item: &IfrItem| -> String {
        match item {
            IfrItem::Departure => format!(
                "the {} departure",
                c.sid.as_deref().map(spoken_sid).unwrap_or_default()
            ),
            IfrItem::Altitude => climb_item(c.initial_alt_ft, *level, *faa),
            IfrItem::Squawk => format!(
                "squawk {}",
                crate::atis::spoken_digits(&format!("{:04}", c.squawk))
            ),
            IfrItem::Runway => format!("runway {runway_spoken}"),
        }
    };
    let mut out = callsign.to_string();
    if !wrong.is_empty() {
        let w: Vec<String> = wrong.iter().map(phrase).collect();
        out.push_str(&format!(", negative, {}.", w.join(", ")));
    }
    if !missing.is_empty() {
        let m: Vec<String> = missing.iter().map(phrase).collect();
        if wrong.is_empty() {
            out.push_str(&format!(", read back {}.", m.join(", ")));
        } else {
            out.push_str(&format!(" Read back {}.", m.join(", ")));
        }
    }
    out
}

/// A conditional line-up needs the arrival far enough out that the pilot can
/// actually get behind it — inside this, "hold position" is the honest call.
pub(crate) const CONDITIONAL_MIN_NM: f64 = 1.5;

/// The tower's side of the dialogue with ONE flight: its FSM and its owed
/// readbacks, no field facts. Each flight's strip has its own on its desk
/// (`station::StripDesk`): ours on frequency, a traffic flight's swapped in
/// for the answer to its call (`Station::respond_for`).
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
    /// An IFR clearance whose readback has just been accepted, for the
    /// station to publish (`take_accepted_ifr`). Only a correct readback
    /// (or, with enforcement off, any readback) puts it here.
    accepted_ifr: Option<IfrClearance>,
    /// The landed arrival has REPORTED the runway vacated. The state stays
    /// `ClearedToLand` (the next bare "request taxi" is still the taxi in),
    /// but the strip is the tower's again: a real controller frees it on
    /// that report, not on the taxi request after it (Liege 2026-09-22).
    /// Meaningless in any other state, and cleared by every transition.
    vacated: bool,
}

impl Tower {
    pub fn new() -> Self {
        Tower {
            state: State::Parked,
            touch_and_go: false,
            pending: None,
            clearance_readback: None,
            accepted_ifr: None,
            vacated: false,
        }
    }

    /// The IFR clearance the pilot has just read back correctly, once.
    pub fn take_accepted_ifr(&mut self) -> Option<IfrClearance> {
        self.accepted_ifr.take()
    }

    /// Readback enforcement is OFF (`XCOPILOT_ATC_READBACK=false`) and the
    /// pilot has read the IFR clearance back, right or wrong: the clearance
    /// was issued, the tower coaches and does not challenge, so it stands.
    /// Nothing happens when no IFR readback is owed.
    pub fn accept_ifr_unenforced(&mut self) {
        if let Some(PendingReadback {
            awaited: Awaited::IfrClearance { clearance, .. },
            ..
        }) = &self.pending
        {
            self.accepted_ifr = Some(clearance.clone());
            self.pending = None;
        }
    }

    /// "Cancel IFR": an IFR clearance still owed its readback, or accepted
    /// and not yet published, dies here. True when there was one.
    pub fn cancel_ifr(&mut self) -> bool {
        let owed = self.ifr_readback_owed();
        if owed {
            self.pending = None;
        }
        owed | self.accepted_ifr.take().is_some()
    }

    /// An IFR clearance's readback is owed.
    pub fn ifr_readback_owed(&self) -> bool {
        matches!(
            self.pending.as_ref().map(|p| &p.awaited),
            Some(Awaited::IfrClearance { .. })
        )
    }

    /// Our aircraft holds the runway in the tower's book: cleared onto it
    /// (line-up, takeoff) or landed on it and not yet reported off. The
    /// vacated report frees it; so does any transition out of the landing.
    pub fn holds_runway(&self) -> bool {
        match self.state {
            State::LinedUp | State::TakeoffCleared => true,
            State::ClearedToLand => !self.vacated,
            _ => false,
        }
    }

    /// The landed arrival reported the runway vacated (see [`Tower::holds_runway`]).
    pub fn runway_vacated(&self) -> bool {
        self.state == State::ClearedToLand && self.vacated
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

    /// The clearance still awaiting its readback, if any (the station's
    /// routing and replies read it).
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
        self.vacated = false;
        self.pending = None;
    }

    /// The landing clearance as the tower speaks it ("..., runway one three,
    /// wind one three zero degrees, five knots, cleared to land."): the reply
    /// to a final report, and the line a withheld clearance is released with.
    pub fn landing_clearance_line(&self, ctx: &Ctx) -> String {
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

    /// The MONITOR released a withheld landing clearance (the strip cleared,
    /// `station::landing_hold_tick`): the state moves to ClearedToLand and the
    /// readback is owed, exactly as if the clearance had been the reply to
    /// the final report.
    pub fn release_landing(&mut self, runway: String, instruction: String) {
        self.state = State::ClearedToLand;
        self.vacated = false;
        self.pending = Some(PendingReadback {
            awaited: Awaited::Landing {
                runway,
                touch_and_go: self.touch_and_go,
            },
            instruction,
            nudges: 0,
        });
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
            Awaited::IfrClearance {
                clearance,
                dest_spoken,
                runway_spoken,
                faa,
                level,
            } => ifr_readback_line(clearance, dest_spoken, runway_spoken, *faa, *level, callsign),
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
        // "Cancel IFR" is never answered "read back": the clearance it
        // would echo is the one being cancelled (the station retires it).
        if intent == Intent::CancelIfr {
            return None;
        }
        // Safety outranks discipline: an emergency or a go-around is never
        // answered with "read back".
        // A reject is the same: the clearance it would read back just died.
        if matches!(
            intent,
            Intent::Mayday | Intent::GoAround | Intent::RejectingTakeoff
        ) {
            self.pending = None;
            return None;
        }
        // A taxi request is not taking the takeoff or the line-up it would
        // echo: the tower answers the request (and cancels what it
        // supersedes), never "read back: cleared for takeoff" to a flight
        // asking to taxi. EGKK 2026-09-27: that answer, unread, ran full
        // auto's taxi wait out to `no_taxi_clearance`.
        if matches!(
            pending.awaited,
            Awaited::Takeoff { .. } | Awaited::LineUp { .. }
        ) && matches!(intent, Intent::RequestTaxi | Intent::RequestTaxiIn)
            && check_readback(transcript, &pending.awaited) == ReadbackCheck::NotAReadback
        {
            self.pending = None;
            return None;
        }
        match check_readback(transcript, &pending.awaited) {
            ReadbackCheck::Correct => {
                if let Awaited::IfrClearance { clearance, .. } = &pending.awaited {
                    self.accepted_ifr = Some(clearance.clone());
                }
                self.pending = None;
                Some(String::new())
            }
            ReadbackCheck::IfrItems { wrong, missing } => {
                let line = ifr_correction(&ctx.callsign, &pending.awaited, &wrong, &missing);
                // A wrong item is corrected as long as it takes, like a
                // wrong runway. Only MISSING items spend a challenge: two,
                // then the tower lets the call through, and the clearance
                // stays unpublished (a mishear never earns it).
                if wrong.is_empty() {
                    let p = self.pending.as_mut().unwrap();
                    if p.nudges >= 2 {
                        self.pending = None;
                        return None;
                    }
                    p.nudges += 1;
                }
                Some(line)
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
        let reply = self.answer(intent, ctx);
        // The vacated report lives only as long as the landing it closed.
        if self.state != State::ClearedToLand {
            self.vacated = false;
        }
        reply
    }

    fn answer(&mut self, intent: Intent, ctx: &Ctx) -> String {
        match intent {
            Intent::RadioCheck => {
                format!("{}, {} Tower, reading you five.", ctx.callsign, ctx.airport)
            }
            Intent::RequestIfrClearance { dest } => match &ctx.ifr {
                Some(offer) => {
                    let (line, instruction) = ifr_clearance_line(ctx, offer);
                    self.arm(
                        Awaited::IfrClearance {
                            clearance: offer.clearance.clone(),
                            dest_spoken: offer.dest_spoken.clone(),
                            runway_spoken: ctx.runway.clone(),
                            faa: offer.faa,
                            level: crate::atis::is_flight_level(
                                offer.clearance.initial_alt_ft as f64,
                                ctx.transition_level_ft,
                            ),
                        },
                        &instruction,
                    );
                    line
                }
                // No destination named: the tower asks for it and clears
                // nothing. One it cannot find: "say again", never a guess.
                None if dest.is_none() => format!("{}, say destination.", ctx.callsign),
                None => format!("{}, say again destination.", ctx.callsign),
            },
            // A circuit stays in the zone: its clearance is the pattern's.
            Intent::RequestClearance if ctx.circuit.is_some() => {
                let offer = ctx.circuit.as_ref().expect("guarded");
                let (line, readback) = circuit_clearance_lines(ctx, offer);
                self.clearance_readback = Some(readback);
                line
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
            // A named runway asked for. Refused with the station's reason (a
            // runway the field lacks, or one it does not hand out), or with
            // "say again" when the words named none: a mishear never buys a
            // runway. Granted on the ground as the taxi clearance TO it (the
            // station planned the route there); in the air, as the join.
            Intent::RequestRunway => match &ctx.runway_refusal {
                Some((rwy, _)) if rwy.is_empty() => {
                    format!("{}, say again the runway you are requesting.", ctx.callsign)
                }
                Some((rwy, why)) => format!("{}, unable runway {rwy}, {why}.", ctx.callsign),
                None if ctx.on_ground => {
                    self.state = State::TaxiCleared;
                    format!(
                        "{}, runway {} approved, taxi to holding point {} runway {}{}{}{}. \
                         Report ready for departure.",
                        ctx.callsign,
                        ctx.runway,
                        ctx.holding_point,
                        ctx.runway,
                        via_clause(ctx),
                        crossing_clause(ctx),
                        qnh_clause(ctx)
                    )
                }
                None => {
                    self.state = State::Inbound;
                    format!(
                        "{}, runway {} approved, join final runway {}{}. Report final.",
                        ctx.callsign,
                        ctx.runway,
                        ctx.runway,
                        qnh_clause(ctx)
                    )
                }
            },
            // Acknowledged, and the STRIP is freed, nothing else. The state
            // deliberately stands: a bare "request taxi" after a landing
            // clearance is inferred as the taxi IN (below), and moving to
            // Parked here would turn the next one into a departure clearance
            // to the holding point. Only a landed arrival frees a strip: the
            // same words from the air are a garble, not a report.
            Intent::RunwayVacated => {
                if self.state == State::ClearedToLand && ctx.on_ground {
                    self.vacated = true;
                }
                format!("{}, roger, thank you.", ctx.callsign)
            }
            Intent::RequestTaxi if self.state == State::ClearedToLand => {
                self.state = State::Parked;
                taxi_in_reply(ctx)
            }
            Intent::RequestTaxi => {
                // A flight cleared for takeoff that asks to taxi is not
                // taking it: the clearance is cancelled out loud, never left
                // standing behind the new route.
                let cancel = if self.state == State::TakeoffCleared {
                    "takeoff clearance cancelled, "
                } else {
                    ""
                };
                self.state = State::TaxiCleared;
                format!(
                    "{}, {cancel}taxi to holding point {} runway {}{}{}{}. \
                     Report ready for departure.",
                    ctx.callsign,
                    ctx.holding_point,
                    ctx.runway,
                    via_clause(ctx),
                    crossing_clause(ctx),
                    qnh_clause(ctx)
                )
            }
            // A backtrack: the strip and its final must be clear, as for a
            // line-up; then onto (or down) the runway to the threshold,
            // turned round and lined up. From the runway itself (a runway
            // start) or from a taxi clearance's hold; from anywhere else it
            // is a taxi request first.
            Intent::RequestBacktrack => {
                if !ctx.on_ground || ctx.helicopter {
                    return format!("{}, say again.", ctx.callsign);
                }
                let at_runway = ctx.own_on_runway
                    || matches!(self.state, State::TaxiCleared | State::LinedUp | State::TakeoffCleared);
                if !at_runway {
                    return self.handle(Intent::RequestTaxi, ctx);
                }
                if ctx.runway_occupied {
                    let line = format!("{}, hold position, traffic on the runway.", ctx.callsign);
                    self.arm(Awaited::HoldPosition, &line);
                    return line;
                }
                if let Some(nm) = ctx.final_traffic_nm {
                    let line = format!(
                        "{}, hold position, traffic on {}. Expect backtrack after the landing traffic.",
                        ctx.callsign,
                        spoken_final(nm)
                    );
                    self.arm(Awaited::HoldPosition, &line);
                    return line;
                }
                self.state = State::LinedUp;
                let line = format!(
                    "{}, backtrack approved runway {}, line up and wait.",
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
            // The station answers a cancellation (it holds the IFR book);
            // the tower alone has nothing to cancel.
            Intent::CancelIfr => format!("{}, say again.", ctx.callsign),
            // Likewise a SID amendment: the station holds the IFR book.
            Intent::RequestSid => format!("{}, say again.", ctx.callsign),
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
                // A helicopter on the ground departs from where it is: no
                // taxi, no runway, and so no runway to read back.
                _ if ctx.helicopter && ctx.on_ground => {
                    self.state = State::TakeoffCleared;
                    format!(
                        "{}, cleared for takeoff from present position, wind {}.",
                        ctx.callsign, ctx.wind
                    )
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
                    self.handle(Intent::RequestTakeoff, ctx)
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
            // Never cleared to land onto an occupied runway: the arrival
            // continues and the clearance follows once the strip is clear
            // (the station arms the release watch, `landing_hold_tick`).
            // LEMG 13 (sortie 1790594471-full_auto, 2026-09-28): "cleared to
            // land" at 3.8 nm with a departure on the runway, then "go
            // around, traffic on the runway" 0.8 s later.
            Intent::ReportFinal if ctx.runway_occupied && !ctx.on_ground => {
                self.state = State::Inbound;
                self.pending = None;
                format!(
                    "{}, continue approach runway {}, traffic on the runway. \
                     Expect late landing clearance.",
                    ctx.callsign, ctx.runway
                )
            }
            Intent::ReportFinal => {
                self.state = State::ClearedToLand;
                let line = self.landing_clearance_line(ctx);
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
            // Traffic standing on the runway: the downwind is number two
            // behind it, and the final report's clearance is withheld until
            // the strip clears (issue #162: the hold, not a second order).
            Intent::ReportDownwind if ctx.runway_occupied && !ctx.on_ground => {
                self.state = State::Inbound;
                format!(
                    "{}, number two, traffic on the runway, report final.",
                    ctx.callsign
                )
            }
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
            // A departure stopping on the runway: the takeoff clearance is
            // void and the aircraft is on the pavement with none, which is
            // LinedUp in this book (the runway stays ours, nobody else is
            // cleared onto it). Nothing to read back: "hold position" here is
            // the tower acknowledging a crew with its hands full. From
            // anywhere else on the ground the words change nothing; in the
            // air they are a garble.
            Intent::RejectingTakeoff
                if ctx.on_ground
                    && matches!(self.state, State::TakeoffCleared | State::LinedUp) =>
            {
                self.state = State::LinedUp;
                format!("{}, roger. Hold position, report ready.", ctx.callsign)
            }
            Intent::RejectingTakeoff if ctx.on_ground => format!("{}, roger.", ctx.callsign),
            Intent::RejectingTakeoff => format!("{}, say again.", ctx.callsign),
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
            // Through the zone to somewhere else: cleared through, the
            // pressure, report leaving. No state change: a transit neither
            // departs nor lands here. The dialect rides the pressure phrase
            // (`qnh_clause` is the one seam: "altimeter" is the FAA's).
            Intent::RequestTransit => {
                if ctx.qnh.to_lowercase().starts_with("altimeter") {
                    format!(
                        "{}, transition through the {} airspace approved{}, report clear.",
                        ctx.callsign,
                        ctx.airport,
                        qnh_clause(ctx)
                    )
                } else {
                    format!(
                        "{}, cleared to transit the {} zone{}, report leaving the zone.",
                        ctx.callsign,
                        ctx.airport,
                        qnh_clause(ctx)
                    )
                }
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
    with_wake(&spoken_callsign(tail), mtow_kg)
}

/// A spoken callsign, from any source (`callsign::resolve`), with its wake
/// suffix. One already saying "heavy" (a pilot's own override) keeps one.
pub fn with_wake(spoken: &str, mtow_kg: Option<f64>) -> String {
    let base = spoken.trim();
    if base.is_empty()
        || base
            .rsplit(' ')
            .next()
            .is_some_and(|w| w.eq_ignore_ascii_case("heavy"))
    {
        return base.to_string();
    }
    match mtow_kg {
        Some(m) if m.is_finite() && m >= HEAVY_MTOW_KG => format!("{base} heavy"),
        _ => base.to_string(),
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
#[path = "tower_tests.rs"]
mod tests;
