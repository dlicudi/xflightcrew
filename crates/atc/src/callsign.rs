//! The aircraft's callsign: ONE resolver for the tower (`station::build_ctx`),
//! the crew's own radio calls (`circuit::crew_callsign`) and the standalone
//! `atc` binary, and the airline telephony that speaks both our flight id and
//! a ghost's.
//!
//! Order: the pilot's override (`XCOPILOT_CALLSIGN`), then the SimBrief OFP's
//! ATC callsign (`atc/callsign`), then the tail number; the wake suffix goes
//! on last, whatever the source. Before this the callsign was ONLY the tail,
//! resolved in three places, and a 737 on an OFP as an airline flight was
//! called by its registration (2026-09-27).
//!
//! The atc crate never reads the app: the crew registers a provider
//! ([`provide_inputs`]) that reads the override and the plan shelf at the
//! moment of asking, so no callsign is ever stored where it could outlive its
//! plan. A plan's callsign applies only to an airframe of the plan's type
//! ([`plan_fits`]): a swap to another type is the tail again, with no state to
//! clear.

use std::sync::Mutex;

use sim::datarefs;
use sim::xplane::SimView;

use crate::atis;

/// Where the callsign in use came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// The pilot's own setting, `XCOPILOT_CALLSIGN`.
    Override,
    /// The loaded SimBrief plan's ATC callsign.
    Simbrief,
    /// The aircraft's registration, `acf_tailnum`.
    Tail,
}

impl Source {
    pub fn as_str(self) -> &'static str {
        match self {
            Source::Override => "override",
            Source::Simbrief => "simbrief",
            Source::Tail => "tail",
        }
    }
}

/// A SimBrief plan's callsign and the type it was planned for.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PlanCallsign {
    /// As the OFP gives it ("BAW306").
    pub id: String,
    /// The plan's `aircraft/icaocode`.
    pub plan_type: String,
    /// Other types a plan for the LOADED airframe may be filed as (the
    /// profile's `simbrief_types`, the plan audit's rule).
    pub also_types: Vec<String>,
}

/// What the crew knows that the sim does not: the override and the plan.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Inputs {
    pub override_id: Option<String>,
    pub plan: Option<PlanCallsign>,
}

/// The callsign in use, spoken, with its wake suffix.
#[derive(Debug, Clone, PartialEq)]
pub struct Resolved {
    pub spoken: String,
    pub source: Source,
}

/// Is this plan's callsign ours? A plan names a TYPE: the loaded airframe
/// must be it (or one of the types its profile is filed as). An unreadable or
/// empty `acf_ICAO`, or a plan without a type, is no evidence against it.
pub fn plan_fits(plan: &PlanCallsign, acf_icao: Option<&str>) -> bool {
    let planned = plan.plan_type.trim();
    let Some(acf) = acf_icao.map(str::trim).filter(|a| !a.is_empty()) else {
        return true;
    };
    planned.is_empty()
        || planned.eq_ignore_ascii_case(acf)
        || plan
            .also_types
            .iter()
            .any(|t| t.trim().eq_ignore_ascii_case(planned))
}

/// The resolver: override > plan > tail, wake suffix last. `None` when no
/// source says anything (the caller's "aircraft calling").
pub fn resolve(
    inputs: &Inputs,
    tail: Option<&str>,
    acf_icao: Option<&str>,
    mtow_kg: Option<f64>,
) -> Option<Resolved> {
    let over = inputs
        .override_id
        .as_deref()
        .map(spoken_id)
        .filter(|s| !s.is_empty())
        .map(|s| (s, Source::Override));
    let plan = || {
        inputs
            .plan
            .as_ref()
            .filter(|p| plan_fits(p, acf_icao))
            .map(|p| spoken_id(&p.id))
            .filter(|s| !s.is_empty())
            .map(|s| (s, Source::Simbrief))
    };
    let tail = || {
        tail.map(crate::tower::spoken_callsign)
            .filter(|s| !s.is_empty())
            .map(|s| (s, Source::Tail))
    };
    let (base, source) = over.or_else(plan).or_else(tail)?;
    Some(Resolved {
        spoken: crate::tower::with_wake(&base, mtow_kg),
        source,
    })
}

/// An airline flight id by shape: three letters, one to four digits, up to
/// two trailing letters ("BAW306", "BAW5GU", "RYR12QT"). Anything else is a
/// registration ("N172SP", "G-ABCD", "DAIKN").
pub fn is_airline_id(id: &str) -> bool {
    let b = id.as_bytes();
    if b.len() < 4 || !b[..3].iter().all(u8::is_ascii_uppercase) {
        return false;
    }
    let digits = b[3..].iter().take_while(|c| c.is_ascii_digit()).count();
    let letters = &b[3 + digits..];
    (1..=4).contains(&digits) && letters.len() <= 2 && letters.iter().all(u8::is_ascii_uppercase)
}

/// A callsign as the pilot or the plan wrote it, spoken. A raw id is spoken
/// like a controller would ("BAW306" is "Speedbird three zero six", an
/// airline the tables do not know is spelled, a registration NATO); words
/// already spoken ("Speedbird 306") stay words, their digits read out. Split
/// ids ("BAW 306", "G ABCD") are ids.
pub fn spoken_id(raw: &str) -> String {
    let tokens: Vec<&str> = raw.split_whitespace().collect();
    let is_word = |t: &&str| t.len() >= 4 && t.chars().all(|c| c.is_ascii_alphabetic());
    if tokens.len() > 1 && tokens.iter().any(is_word) {
        return tokens
            .iter()
            .map(|t| {
                if t.chars().all(|c| c.is_ascii_digit()) {
                    atis::spoken_digits(t)
                } else {
                    t.to_string()
                }
            })
            .collect::<Vec<_>>()
            .join(" ");
    }
    let id = tokens.concat().to_ascii_uppercase();
    if is_airline_id(&id) {
        spoken_ghost(&id)
    } else {
        crate::tower::spoken_callsign(&id)
    }
}

/// One way of writing a transmission for comparison: lower case, no spaces
/// or punctuation, digits as their words, the letters of a mixed token
/// ("12qt", "172sp") as NATO words, whisper's spelling variants folded.
fn squash(text: &str) -> String {
    let mut out = String::new();
    for tok in text
        .split(|c: char| !c.is_alphanumeric())
        .filter(|t| !t.is_empty())
    {
        let tok = tok.to_lowercase();
        if tok.chars().any(|c| c.is_ascii_digit()) {
            for c in tok.chars() {
                if c.is_ascii_digit() {
                    out.push_str(&atis::spoken_digits(&c.to_string()));
                } else if c.is_ascii_alphabetic() {
                    out.push_str(&fold(&atis::nato(c.to_ascii_uppercase()).to_lowercase()));
                }
            }
            continue;
        }
        out.push_str(&fold(&tok));
    }
    out.retain(|c| c.is_ascii_alphanumeric());
    out
}

/// Whisper's and the tables' spellings of the same spoken word, folded.
fn fold(w: &str) -> String {
    match w {
        "nine" => "niner",
        "juliett" => "juliet",
        "whisky" => "whiskey",
        "alfa" => "alpha",
        t => t,
    }
    .to_string()
}

/// Does this transmission name this callsign? The whole spoken callsign
/// (less "heavy"), however whisper wrote it: "Speedbird 306", "speed bird
/// three zero six". Too short a callsign names nothing.
pub fn addressed(transcript: &str, spoken_callsign: &str) -> bool {
    let words: Vec<&str> = spoken_callsign
        .split_whitespace()
        .filter(|w| !w.eq_ignore_ascii_case("heavy"))
        .collect();
    if words.len() < 2 {
        return false;
    }
    let needle = squash(&words.join(" "));
    needle.len() >= 8 && squash(transcript).contains(&needle)
}

/// Does this transmission name OUR override or plan callsign? The tower's
/// unkeyed gate asks before it reads the sim: an utterance that opens with
/// "Speedbird three zero six" is a radio call whatever follows. The tail's
/// form needs no help (its calls carry the radio words already); the plan's
/// callsign is asked without the type check, since hearing it costs nothing.
pub fn addressed_to_own(transcript: &str) -> bool {
    let i = inputs();
    i.override_id
        .iter()
        .map(|o| spoken_id(o))
        .chain(i.plan.iter().map(|p| spoken_id(&p.id)))
        .any(|s| addressed(transcript, &s))
}

/// Register the crew's provider of [`Inputs`]. Called once at boot; the
/// provider reads the live setting and the plan shelf each time it is asked.
pub fn provide_inputs(f: fn() -> Inputs) {
    *crate::party_line::CALLSIGN_INPUTS
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = Some(f);
}

/// The crew's inputs right now; none without a provider (tests, a bare
/// station).
pub fn inputs() -> Inputs {
    let f = *crate::party_line::CALLSIGN_INPUTS
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    f.map(|f| f()).unwrap_or_default()
}

/// The last callsign blackboxed: the `callsign` record is written when it
/// CHANGES, from whichever side resolved it first.
static LAST: Mutex<Option<(String, Source)>> = Mutex::new(None);

/// Blackbox a resolved callsign when it differs from the last one
/// (`atc` `callsign` with `spoken` and `source`).
pub fn note(r: &Resolved) {
    let mut last = LAST.lock().unwrap_or_else(|e| e.into_inner());
    let now = (r.spoken.clone(), r.source);
    if last.as_ref() == Some(&now) {
        return;
    }
    *last = Some(now);
    tracing::info!(spoken = %r.spoken, source = r.source.as_str(), "callsign");
    sim::blackbox::record(
        "atc",
        serde_json::json!({
            "event": "callsign", "spoken": r.spoken, "source": r.source.as_str(),
        }),
    );
}

/// Read the sim and resolve with these inputs, blackboxing a change. A failed
/// read is no source: an unreadable tail falls to `None`, an unreadable type
/// never refuses the plan.
pub async fn live_with(inputs: &Inputs, xp: SimView<'_>) -> Option<Resolved> {
    let tail = xp.get_string(datarefs::TAILNUM).await.ok();
    let icao = if inputs.plan.is_some() {
        xp.get_string(datarefs::ACF_ICAO).await.ok()
    } else {
        None
    };
    let mtow_kg = xp.get_float(datarefs::ACF_M_MAX_KG).await.ok();
    let r = resolve(inputs, tail.as_deref(), icao.as_deref(), mtow_kg)?;
    note(&r);
    Some(r)
}

/// [`live_with`] the registered provider's inputs.
pub async fn live(xp: SimView<'_>) -> Option<Resolved> {
    live_with(&inputs(), xp).await
}

/// Airline telephony for the party line: the common carriers over Iberia;
/// anything else is spoken NATO-style like a tail number.
pub(crate) const TELEPHONY: &[(&str, &str)] = &[
    ("IBE", "Iberia"),
    ("BAW", "Speedbird"),
    ("RYR", "Ryanair"),
    ("EZY", "Easy"),
    ("VLG", "Vueling"),
    ("AEA", "Air Europa"),
    ("ANE", "Air Nostrum"),
    ("SWT", "Swift"),
    ("DLH", "Lufthansa"),
    ("AFR", "Air France"),
    ("KLM", "KLM"),
    ("TAP", "Air Portugal"),
    ("EIN", "Shamrock"),
    ("WZZ", "Wizz Air"),
    ("SWR", "Swiss"),
    ("UAE", "Emirates"),
    ("QTR", "Qatari"),
    ("AAL", "American"),
    ("UAL", "United"),
    ("DAL", "Delta"),
    ("VIR", "Virgin"),
    ("EXS", "Channex"),
    ("EWG", "Eurowings"),
    ("MEA", "Cedar Jet"),
    ("BEL", "Bee-line"),
    ("AUA", "Austrian"),
    ("SAS", "Scandinavian"),
    ("THY", "Turkish"),
    ("FIN", "Finnair"),
    ("ICE", "Iceair"),
    ("ACA", "Air Canada"),
    ("CPA", "Cathay"),
    ("SIA", "Singapore"),
    ("ANA", "All Nippon"),
    ("ETD", "Etihad"),
    ("ELY", "El Al"),
    ("QFA", "Qantas"),
    ("ANZ", "New Zealand"),
    ("JBU", "JetBlue"),
    ("CFG", "Condor"),
    ("TRA", "Transavia"),
    ("NAX", "Nor Shuttle"),
    ("TOM", "Tomjet"),
    ("LOG", "Logan"),
    ("AEE", "Aegean"),
    ("LOT", "LOT"),
    ("AIC", "Air India"),
    ("PIA", "Pakistan"),
    ("WJA", "WestJet"),
    ("PGT", "Sunturk"),
];

/// IATA two-letter prefixes. LiveTraffic's flight ids arrive this way
/// ("BA306", measured live 2026-07-10), so both spellings get telephony.
pub(crate) const TELEPHONY_IATA: &[(&str, &str)] = &[
    ("BA", "Speedbird"),
    ("IB", "Iberia"),
    ("FR", "Ryanair"),
    ("U2", "Easy"),
    ("VY", "Vueling"),
    ("UX", "Air Europa"),
    ("LH", "Lufthansa"),
    ("AF", "Air France"),
    ("KL", "KLM"),
    ("TP", "Air Portugal"),
    ("EI", "Shamrock"),
    ("W6", "Wizz Air"),
    ("LX", "Swiss"),
    ("EK", "Emirates"),
    ("QR", "Qatari"),
    ("AA", "American"),
    ("UA", "United"),
    ("DL", "Delta"),
    ("TG", "Thai"),
    ("VS", "Virgin"),
    ("LS", "Channex"),
    ("EW", "Eurowings"),
    ("ME", "Cedar Jet"),
    ("SN", "Bee-line"),
    ("OS", "Austrian"),
    ("SK", "Scandinavian"),
    ("TK", "Turkish"),
    ("AY", "Finnair"),
    ("FI", "Iceair"),
    ("AC", "Air Canada"),
    ("CX", "Cathay"),
    ("SQ", "Singapore"),
    ("NH", "All Nippon"),
    ("EY", "Etihad"),
    ("LY", "El Al"),
    ("QF", "Qantas"),
    ("NZ", "New Zealand"),
    ("B6", "JetBlue"),
    ("DE", "Condor"),
    ("HV", "Transavia"),
    ("DY", "Nor Shuttle"),
    ("BY", "Tomjet"),
    ("LM", "Logan"),
    ("A3", "Aegean"),
    ("LO", "LOT"),
    ("AI", "Air India"),
    ("PK", "Pakistan"),
    ("WS", "WestJet"),
    ("PC", "Sunturk"),
];

/// A ghost's flight id spoken like a controller would: airline telephony +
/// digits for "IBE3421" / "BA306", full NATO for a tail number.
pub fn spoken_ghost(id: &str) -> String {
    let id = id.trim();
    let spoken_rest = |rest: &str| {
        (!rest.is_empty() && rest.chars().all(|c| c.is_ascii_alphanumeric()))
            .then(|| crate::tower::spoken_callsign(rest))
    };
    // ICAO 3-letter prefix first ("IBE3421"), then IATA 2-letter where the
    // third character is a digit ("BA306", but never "BAC" of a tail).
    // Byte-boundary-safe slicing on purpose: the flight id comes off raw
    // TCAS bytes through from_utf8_lossy, and a garbage feed's replacement
    // characters land mid-slice, and `get` answers None where `split_at` would
    // panic the monitor thread.
    if id.len() > 3 {
        if let (Some(prefix), Some(rest)) = (id.get(..3), id.get(3..)) {
            if let Some((_, telephony)) = TELEPHONY.iter().find(|(p, _)| *p == prefix) {
                if let Some(rest) = spoken_rest(rest) {
                    return format!("{telephony} {rest}");
                }
            }
        }
    }
    if id.len() > 2 && id.as_bytes()[2].is_ascii_digit() {
        if let (Some(prefix), Some(rest)) = (id.get(..2), id.get(2..)) {
            if let Some((_, telephony)) = TELEPHONY_IATA.iter().find(|(p, _)| *p == prefix) {
                if let Some(rest) = spoken_rest(rest) {
                    return format!("{telephony} {rest}");
                }
            }
        }
    }
    crate::tower::spoken_callsign(id)
}

#[cfg(test)]
mod tests {
    use super::*;

    const A330_MTOW: f64 = 242_000.0;
    const B738_MTOW: f64 = 79_000.0;

    fn plan(id: &str, ty: &str) -> Option<PlanCallsign> {
        Some(PlanCallsign {
            id: id.into(),
            plan_type: ty.into(),
            also_types: Vec::new(),
        })
    }

    fn inputs(over: Option<&str>, plan: Option<PlanCallsign>) -> Inputs {
        Inputs {
            override_id: over.map(str::to_string),
            plan,
        }
    }

    /// Override beats the plan beats the tail; an empty override is none.
    #[test]
    fn override_beats_plan_beats_tail() {
        let all = inputs(Some("N172SP"), plan("BAW306", "B738"));
        let r = resolve(&all, Some("G-ABCD"), Some("B738"), Some(B738_MTOW)).unwrap();
        assert_eq!(r.source, Source::Override);
        assert_eq!(r.spoken, "November one seven two Sierra Papa");

        let no_over = inputs(None, plan("BAW306", "B738"));
        let r = resolve(&no_over, Some("G-ABCD"), Some("B738"), Some(B738_MTOW)).unwrap();
        assert_eq!(r.source, Source::Simbrief);
        assert_eq!(r.spoken, "Speedbird three zero six");

        for blank in ["", "   "] {
            let empty = inputs(Some(blank), plan("BAW306", "B738"));
            let r = resolve(&empty, Some("G-ABCD"), Some("B738"), None).unwrap();
            assert_eq!(r.source, Source::Simbrief, "{blank:?} is no override");
        }

        let tail_only = inputs(None, None);
        let r = resolve(&tail_only, Some("G-ABCD"), Some("B738"), None).unwrap();
        assert_eq!(r.source, Source::Tail);
        assert_eq!(r.spoken, "Golf Alpha Bravo Charlie Delta");

        assert_eq!(resolve(&tail_only, None, None, None), None);
        assert_eq!(resolve(&tail_only, Some(""), None, None), None);
    }

    /// A plan's callsign is the plan type's: after a swap to a 172 the
    /// 737's OFP no longer names us, and nothing had to be cleared.
    #[test]
    fn a_plan_for_another_type_is_the_tail_again() {
        let i = inputs(None, plan("BAW306", "B738"));
        let r = resolve(&i, Some("N172SP"), Some("C172"), None).unwrap();
        assert_eq!(r.source, Source::Tail);
        // Filed under a type the profile declares (simbrief_types).
        let mut p = plan("BAW306", "P46T").unwrap();
        p.also_types = vec!["P46T".into()];
        assert!(plan_fits(&p, Some("EVOT")));
        // An unreadable type is no evidence against the plan.
        assert!(plan_fits(&plan("BAW306", "B738").unwrap(), None));
        assert!(plan_fits(&plan("BAW306", "").unwrap(), Some("C172")));
    }

    /// Airline ids by telephony, an unknown airline spelled, registrations
    /// as the tail always was.
    #[test]
    fn ids_are_spoken_like_a_controller() {
        assert_eq!(spoken_id("BAW306"), "Speedbird three zero six");
        assert_eq!(spoken_id("baw306"), "Speedbird three zero six");
        assert_eq!(spoken_id("BAW 306"), "Speedbird three zero six");
        assert_eq!(spoken_id("BAW5GU"), "Speedbird five Golf Uniform");
        assert_eq!(spoken_id("XYZ123"), "X-ray Yankee Zulu one two three");
        assert_eq!(spoken_id("WAT2516"), "Whiskey Alpha Tango two five one six");
        assert_eq!(spoken_id("N172SP"), "November one seven two Sierra Papa");
        assert_eq!(spoken_id("G-ABCD"), "Golf Alpha Bravo Charlie Delta");
        assert_eq!(spoken_id("N172SP"), crate::tower::spoken_callsign("N172SP"));
        assert_eq!(spoken_id("G-ABCD"), crate::tower::spoken_callsign("G-ABCD"));
        // Already words: kept, digits read out.
        assert_eq!(spoken_id("Speedbird 306"), "Speedbird three zero six");
        assert_eq!(
            spoken_id("Speedbird three zero six"),
            "Speedbird three zero six"
        );
        assert!(is_airline_id("RYR12QT"));
        assert!(!is_airline_id("N172SP"));
        assert!(!is_airline_id("GABCD"));
        assert!(!is_airline_id("BAW12345"));
    }

    /// The heavy suffix goes on last, whatever the source, and never twice.
    #[test]
    fn heavy_on_every_source() {
        let over = inputs(Some("BAW306"), None);
        let r = resolve(&over, Some("D-AIKN"), None, Some(A330_MTOW)).unwrap();
        assert_eq!(r.spoken, "Speedbird three zero six heavy");
        let words = inputs(Some("Speedbird 306 heavy"), None);
        let r = resolve(&words, None, None, Some(A330_MTOW)).unwrap();
        assert_eq!(r.spoken, "Speedbird three zero six heavy");
        let p = inputs(None, plan("DLH400", "A333"));
        let r = resolve(&p, Some("D-AIKN"), Some("A333"), Some(A330_MTOW)).unwrap();
        assert_eq!(
            (r.source, r.spoken.as_str()),
            (Source::Simbrief, "Lufthansa four zero zero heavy")
        );
        let t = inputs(None, None);
        let r = resolve(&t, Some("D-AIKN"), Some("A333"), Some(A330_MTOW)).unwrap();
        assert_eq!(r.spoken, "Delta Alpha India Kilo November heavy");
        let r = resolve(&p, Some("D-AIKN"), Some("A333"), Some(B738_MTOW)).unwrap();
        assert!(!r.spoken.ends_with("heavy"), "{}", r.spoken);
        let r = resolve(&p, Some("D-AIKN"), Some("A333"), None).unwrap();
        assert!(!r.spoken.ends_with("heavy"), "unknown mass, no suffix");
    }

    /// The tower hears its airline callsign however whisper writes it, and
    /// nothing short of the whole callsign.
    #[test]
    fn a_transmission_names_the_callsign_however_whisper_writes_it() {
        let us = "Speedbird three zero six heavy";
        for heard in [
            "Speedbird 306, ready to taxi",
            "speedbird three zero six request taxi",
            "Speed Bird 306 with you",
            "Tower, Speedbird 3-0-6, holding short",
        ] {
            assert!(addressed(heard, us), "{heard:?}");
        }
        for not in [
            "Speedbird 307, ready to taxi",
            "speedbird ready to taxi",
            "three zero six",
        ] {
            assert!(!addressed(not, us), "{not:?}");
        }
        assert!(addressed("Ryanair 12QT ready", &spoken_ghost("RYR12QT")));
        assert!(addressed(
            "November 172SP ready to taxi",
            "November one seven two Sierra Papa"
        ));
        assert!(addressed(
            "Lufthansa 409, ready",
            "Lufthansa four zero niner"
        ));
        assert!(addressed(
            "lufthansa four zero nine ready",
            "Lufthansa four zero niner"
        ));
        assert!(!addressed("anything", ""));
    }
}
