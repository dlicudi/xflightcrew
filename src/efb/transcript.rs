//! Who said a line to whom: the EFB's transcript labels (FROM -> TO, the
//! speaker's colour, the tags), ONE pure function over a line as the taps
//! kept it ([`label`]). The sim panel's transcript reads the same rules
//! ([`panel_label`]).
//!
//! The rules, from the lines of real flights:
//!
//! - Intercom: the copilot speaks to YOU, or to the CABIN when it opens with
//!   the cabin address ("Cabin crew, seats for landing."); the cabin persona
//!   speaks to the FLIGHT DECK; the pilot speaks to the COPILOT, or to the
//!   CABIN when the line carries the grammar's cabin address
//!   (`cabin::Cabin::addressed`).
//! - Radio, ours: OUR CALLSIGN to the position COM1 was on, never "copilot";
//!   tagged "by copilot" when the crew keyed it, "readback" for a readback.
//! - Radio, a controller: the POSITION to the callsign that opens the line
//!   (ours or another's); a party-line exchange not addressed to us is dimmed.
//! - Radio, traffic: the ghost's callsign (where the line names it) to the
//!   position, dimmed.
//! - A "say again" either way is tagged.

use serde::Serialize;

use crate::efb::feed::{Line, Who};

/// What the labeller knows about us.
#[derive(Clone, Debug, Default)]
pub(crate) struct Ctx<'a> {
    /// Our callsign as the resolver speaks it ("Whiskey Alpha Tango five one
    /// niner").
    pub(crate) spoken: Option<&'a str>,
    /// Our callsign as the label writes it ("WAT519").
    pub(crate) written: Option<&'a str>,
}

/// A line, labelled.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Labelled {
    /// "radio" or "intercom".
    pub(crate) chan: &'static str,
    pub(crate) from: String,
    pub(crate) to: String,
    /// The speaker's colour: "you", "copilot", "cabin", "atc", "us" (our
    /// transmission) or "other" (another aircraft).
    pub(crate) cls: &'static str,
    /// Not to or from us: shown dimmed.
    pub(crate) dim: bool,
    pub(crate) tags: Vec<&'static str>,
}

const YOU: &str = "YOU";
const COPILOT: &str = "COPILOT";
const CABIN: &str = "CABIN";
const FLIGHT_DECK: &str = "FLIGHT DECK";

/// The position a station tag names, as the label writes it.
pub(crate) fn position(tag: Option<&str>) -> &'static str {
    match tag {
        Some("TWR") => "TOWER",
        Some("APP") => "APPROACH",
        Some("DEP") => "DEPARTURE",
        Some("ATIS") => "ATIS",
        _ => "ATC",
    }
}

/// The copilot addresses the cabin: an explicit address opens the line.
fn copilot_to_cabin(text: &str) -> bool {
    let t = text
        .trim()
        .trim_start_matches(['.', ',', ' '])
        .to_ascii_lowercase();
    ["cabin crew", "flight attendants", "flight attendant"]
        .iter()
        .any(|p| t.starts_with(p))
}

fn nato(w: &str) -> Option<char> {
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
        "xray" | "x-ray" => 'X',
        "yankee" => 'Y',
        "zulu" => 'Z',
        _ => return None,
    })
}

fn digit(w: &str) -> Option<char> {
    Some(match w {
        "zero" => '0',
        "one" | "wun" => '1',
        "two" | "too" => '2',
        "three" | "tree" => '3',
        "four" | "fower" => '4',
        "five" | "fife" => '5',
        "six" => '6',
        "seven" => '7',
        "eight" | "ait" => '8',
        "nine" | "niner" => '9',
        _ => return None,
    })
}

/// A spoken callsign written as a label: NATO words as their letters,
/// number words as digits, a short written id kept, other words in capitals
/// ("Whiskey Alpha Tango five one niner" is "WAT519", "Easy eight seven
/// niner one" is "EASY 8791", "XVL 214" is "XVL214"). Pure.
pub(crate) fn compact(spoken: &str) -> String {
    let mut out = String::new();
    // Was the last thing pushed glued characters (a letter or digit run)?
    let mut glued = false;
    for raw in spoken.split(|c: char| c.is_whitespace() || c == ',' || c == '.') {
        let tok = raw.trim_matches(|c: char| !c.is_alphanumeric() && c != '-');
        if tok.is_empty() {
            continue;
        }
        let lower = tok.to_ascii_lowercase();
        let piece: Option<String> = if let Some(c) = nato(&lower) {
            Some(c.to_string())
        } else if let Some(c) = digit(&lower) {
            Some(c.to_string())
        } else if tok.len() <= 4
            && tok
                .chars()
                .all(|c| c.is_ascii_digit() || c.is_ascii_uppercase())
        {
            Some(tok.to_string())
        } else if tok.chars().all(|c| c.is_ascii_digit()) {
            Some(tok.to_string())
        } else {
            None
        };
        match piece {
            Some(p) => {
                if !glued && !out.is_empty() {
                    out.push(' ');
                }
                out.push_str(&p);
                glued = true;
            }
            None => {
                if !out.is_empty() {
                    out.push(' ');
                }
                out.push_str(&tok.to_ascii_uppercase());
                glued = false;
            }
        }
    }
    out
}

/// Words that make a chunk an instruction or a place, never a callsign.
const NOT_A_CALLSIGN: [&str; 24] = [
    "runway",
    "tower",
    "approach",
    "departure",
    "ground",
    "delivery",
    "information",
    "wind",
    "cleared",
    "contact",
    "heading",
    "climb",
    "descend",
    "squawk",
    "qnh",
    "level",
    "taxi",
    "hold",
    "maintain",
    "turn",
    "report",
    "line",
    "stations",
    "radar",
];

/// Does this chunk read as a callsign: a few words, some of them a number,
/// none of them an instruction?
fn looks_like_callsign(chunk: &str) -> bool {
    let words: Vec<String> = chunk
        .split_whitespace()
        .map(|w| {
            w.trim_matches(|c: char| !c.is_alphanumeric())
                .to_ascii_lowercase()
        })
        .filter(|w| !w.is_empty())
        .collect();
    if words.is_empty() || words.len() > 8 {
        return false;
    }
    let numeric = words
        .iter()
        .any(|w| digit(w).is_some() || w.chars().any(|c| c.is_ascii_digit()));
    numeric && !words.iter().any(|w| NOT_A_CALLSIGN.contains(&w.as_str()))
}

/// The chunk before the first comma (or full stop), trimmed.
fn first_chunk(text: &str) -> &str {
    text.split([',', '.']).next().unwrap_or("").trim()
}

/// The chunk after the last comma, trimmed of its full stop.
fn last_chunk(text: &str) -> &str {
    text.trim()
        .trim_end_matches(['.', '!', '?'])
        .rsplit(',')
        .next()
        .unwrap_or("")
        .trim()
}

/// Is this chunk OUR callsign, however it was spoken or written?
fn is_ours(chunk: &str, c: &Ctx) -> bool {
    if let Some(s) = c.spoken.filter(|s| !s.trim().is_empty()) {
        if crate::callsign::addressed(chunk, s) {
            return true;
        }
    }
    let mine = c
        .written
        .map(|w| w.replace(' ', "").to_ascii_uppercase())
        .filter(|w| !w.is_empty());
    let mine = mine.or_else(|| c.spoken.map(|s| compact(s).replace(' ', "")));
    mine.is_some_and(|m| m == compact(chunk).replace(' ', ""))
}

/// Our callsign as the label writes it, `YOU` when nobody resolved one.
fn us(c: &Ctx) -> String {
    c.written
        .map(str::to_string)
        .filter(|w| !w.trim().is_empty())
        .or_else(|| c.spoken.map(compact).filter(|w| !w.is_empty()))
        .unwrap_or_else(|| YOU.to_string())
}

/// Who a controller's line is addressed to: `(label, ours)`. The callsign
/// that opens it; with none, us unless the voice said it was overheard.
fn addressee(text: &str, overheard: bool, c: &Ctx) -> (String, bool) {
    let chunk = first_chunk(text);
    if is_ours(chunk, c) {
        return (us(c), true);
    }
    if looks_like_callsign(chunk) {
        return (compact(chunk), false);
    }
    if overheard {
        ("TRAFFIC".into(), false)
    } else {
        (us(c), true)
    }
}

/// A ghost pilot's callsign: where its line names it (the readback's end,
/// or a call's start).
fn traffic_callsign(text: &str) -> String {
    [last_chunk(text), first_chunk(text)]
        .into_iter()
        .find(|ch| looks_like_callsign(ch))
        .map(compact)
        .unwrap_or_else(|| "TRAFFIC".into())
}

/// The line, labelled. Pure.
pub(crate) fn label(l: &Line, c: &Ctx) -> Labelled {
    let say_again = crate::tower::is_say_again(&l.text);
    let mut tags: Vec<&'static str> = Vec::new();
    if !l.radio {
        let (from, to, cls) = match l.who {
            Who::Pilot => (
                YOU,
                if crate::cabin::Cabin::addressed(&l.text).is_some() {
                    CABIN
                } else {
                    COPILOT
                },
                "you",
            ),
            Who::Cabin => (CABIN, FLIGHT_DECK, "cabin"),
            Who::Crew | Who::Atc | Who::Atis | Who::Traffic => (
                COPILOT,
                if copilot_to_cabin(&l.text) {
                    CABIN
                } else {
                    YOU
                },
                "copilot",
            ),
        };
        return Labelled {
            chan: "intercom",
            from: from.into(),
            to: to.into(),
            cls,
            dim: false,
            tags,
        };
    }
    let pos = position(l.tag);
    let (from, to, cls, dim) = match l.who {
        Who::Pilot | Who::Crew | Who::Cabin => {
            if l.who == Who::Crew {
                tags.push("by copilot");
            }
            if l.readback || (l.who == Who::Pilot && crate::tower::is_readback(&l.text)) {
                tags.insert(0, "readback");
            }
            (us(c), pos.to_string(), "us", false)
        }
        Who::Atc => {
            let (to, ours) = addressee(&l.text, l.overheard, c);
            (pos.to_string(), to, "atc", l.overheard || !ours)
        }
        Who::Atis => ("ATIS".into(), "ALL STATIONS".into(), "atc", false),
        Who::Traffic => (traffic_callsign(&l.text), pos.to_string(), "other", true),
    };
    if say_again {
        tags.push("say again");
    }
    Labelled {
        chan: "radio",
        from,
        to,
        cls,
        dim,
        tags,
    }
}

/// Our callsign as the label writes it: the plan's or the pilot's written
/// id when it IS the callsign the resolver speaks, else the spoken one
/// compacted. Pure.
pub(crate) fn written_callsign(spoken: &str, ids: &[String]) -> Option<String> {
    let spoken = spoken.trim();
    if spoken.is_empty() {
        return None;
    }
    let speak_of = |id: &str| crate::callsign::spoken_id(id);
    ids.iter()
        .map(|i| i.trim())
        .filter(|i| !i.is_empty())
        .find(|i| crate::callsign::addressed(&speak_of(i), spoken) || compact(spoken) == compact(i))
        .map(|i| i.replace(' ', "").to_ascii_uppercase())
        .or_else(|| Some(compact(spoken)).filter(|s| !s.is_empty()))
}

/// The sim panel's transcript row label ("COPILOT > YOU", "TOWER >
/// WAT519"), from the panel feed's own prefix convention: `TWR: `, `APP: `,
/// `DEP: `, `ATIS: ` a controller, `TFC: ` traffic, `CAB: ` the cabin, a
/// leading `~` overheard, anything else the copilot; `pilot` for the
/// pilot's heard line. ASCII only: the panel font has no arrow. Pure.
pub(crate) fn panel_label(text: &str, pilot: bool, c: &Ctx) -> String {
    let overheard = text.starts_with('~');
    let body = text.trim_start_matches('~');
    let (who, tag, rest) = if pilot {
        (Who::Pilot, None, body)
    } else {
        let station = [
            ("TWR: ", Who::Atc, "TWR"),
            ("APP: ", Who::Atc, "APP"),
            ("DEP: ", Who::Atc, "DEP"),
            ("ATIS: ", Who::Atis, "ATIS"),
            ("TFC: ", Who::Traffic, "TWR"),
            ("CAB: ", Who::Cabin, ""),
        ]
        .into_iter()
        .find(|(p, _, _)| body.starts_with(p));
        match station {
            Some((p, w, t)) => (w, Some(t).filter(|t| !t.is_empty()), &body[p.len()..]),
            None => (Who::Crew, None, body),
        }
    };
    let radio = matches!(who, Who::Atc | Who::Atis | Who::Traffic);
    let line = Line {
        t: 0.0,
        who,
        text: rest.to_string(),
        radio,
        tag,
        khz: None,
        overheard,
        readback: false,
    };
    let l = label(&line, c);
    format!("{} > {}", l.from, l.to)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(who: Who, text: &str, radio: bool, tag: Option<&'static str>) -> Line {
        Line {
            t: 0.0,
            who,
            text: text.into(),
            radio,
            tag,
            khz: Some(131_200),
            overheard: false,
            readback: false,
        }
    }

    fn wat() -> Ctx<'static> {
        Ctx {
            spoken: Some("Whiskey Alpha Tango five one niner"),
            written: Some("WAT519"),
        }
    }

    fn ft(l: &Labelled) -> String {
        format!("{} -> {}", l.from, l.to)
    }

    /// The crew's own call, keyed by the crew (real line, LXGB 2026-10-05):
    /// our callsign to the position, never "copilot".
    #[test]
    fn the_crews_call_is_ours_by_callsign() {
        let l = line(
            Who::Crew,
            "Whiskey Alpha Tango five one niner, request IFR clearance to Echo Golf Kilo Kilo.",
            true,
            Some("TWR"),
        );
        let out = label(&l, &wat());
        assert_eq!(ft(&out), "WAT519 -> TOWER");
        assert_eq!((out.chan, out.cls, out.dim), ("radio", "us", false));
        assert_eq!(out.tags, ["by copilot"]);
        let mut rb = l.clone();
        rb.readback = true;
        assert_eq!(label(&rb, &wat()).tags, ["readback", "by copilot"]);
    }

    /// A controller clearing other traffic: the position to THEIR callsign,
    /// dimmed (real line, LXGB party line).
    #[test]
    fn a_controller_to_other_traffic_is_dimmed() {
        let mut l = line(
            Who::Atc,
            "Easy eight seven niner one, runway zero niner, cleared to land.",
            true,
            Some("TWR"),
        );
        l.overheard = true;
        let out = label(&l, &wat());
        assert_eq!(ft(&out), "TOWER -> EASY 8791");
        assert_eq!((out.cls, out.dim), ("atc", true));
        // Even without the voice's flag, another callsign is not ours.
        l.overheard = false;
        assert!(label(&l, &wat()).dim);
        // To us: by our callsign, bright.
        let to_us = line(
            Who::Atc,
            "Whiskey Alpha Tango five one niner, taxi to holding point Echo, runway two seven.",
            true,
            Some("TWR"),
        );
        let out = label(&to_us, &wat());
        assert_eq!(ft(&out), "TOWER -> WAT519");
        assert!(!out.dim);
        // A line with no callsign, not overheard, is ours ("Radar contact.").
        let bare = line(Who::Atc, "Radar contact.", true, Some("APP"));
        assert_eq!(ft(&label(&bare, &wat())), "APPROACH -> WAT519");
    }

    /// The copilot to the cabin, and the cabin to the flight deck.
    #[test]
    fn the_cabin_address_turns_the_line() {
        let l = line(Who::Crew, "Cabin crew, seats for landing.", false, None);
        let out = label(&l, &wat());
        assert_eq!(ft(&out), "COPILOT -> CABIN");
        assert_eq!((out.chan, out.cls), ("intercom", "copilot"));
        let l = line(Who::Crew, "Gear down, three green.", false, None);
        assert_eq!(ft(&label(&l, &wat())), "COPILOT -> YOU");
        let l = line(Who::Cabin, "Cabin secure for landing.", false, None);
        let out = label(&l, &wat());
        assert_eq!(ft(&out), "CABIN -> FLIGHT DECK");
        assert_eq!(out.cls, "cabin");
        let l = line(
            Who::Pilot,
            "Cabin crew, prepare for departure.",
            false,
            None,
        );
        assert_eq!(ft(&label(&l, &wat())), "YOU -> CABIN");
        let l = line(Who::Pilot, "Flaps five.", false, None);
        let out = label(&l, &wat());
        assert_eq!(ft(&out), "YOU -> COPILOT");
        assert_eq!(out.cls, "you");
    }

    /// The pilot's keyed call is ours too, a readback tagged; a ghost's
    /// readback is its callsign to the position, dimmed; a say again tagged.
    #[test]
    fn the_pilot_on_the_radio_traffic_and_say_again() {
        let l = line(
            Who::Pilot,
            "Cleared to land runway two seven, Whiskey Alpha Tango five one niner",
            true,
            Some("TWR"),
        );
        let out = label(&l, &wat());
        assert_eq!(ft(&out), "WAT519 -> TOWER");
        assert!(out.tags.contains(&"readback"));
        assert!(!out.tags.contains(&"by copilot"));
        let g = line(
            Who::Traffic,
            "Cleared to land runway zero niner, Easy eight seven niner one.",
            true,
            Some("TWR"),
        );
        let out = label(&g, &wat());
        assert_eq!(ft(&out), "EASY 8791 -> TOWER");
        assert_eq!((out.cls, out.dim), ("other", true));
        let sa = line(
            Who::Atc,
            "Whiskey Alpha Tango five one niner, say again.",
            true,
            Some("TWR"),
        );
        assert_eq!(label(&sa, &wat()).tags, ["say again"]);
        let atis = line(
            Who::Atis,
            "Gibraltar information Alpha.",
            true,
            Some("ATIS"),
        );
        assert_eq!(ft(&label(&atis, &wat())), "ATIS -> ALL STATIONS");
        // No callsign resolved: our side is YOU.
        let none = Ctx::default();
        let l = line(Who::Crew, "Request taxi.", true, Some("TWR"));
        assert_eq!(ft(&label(&l, &none)), "YOU -> TOWER");
    }

    #[test]
    fn a_spoken_callsign_compacts_like_a_strip() {
        assert_eq!(compact("Whiskey Alpha Tango five one niner"), "WAT519");
        assert_eq!(compact("Easy eight seven niner one"), "EASY 8791");
        assert_eq!(compact("XVL 214"), "XVL214");
        assert_eq!(
            compact("Speedbird three zero six heavy"),
            "SPEEDBIRD 306 HEAVY"
        );
        assert_eq!(compact("November one seven two Sierra Papa"), "N172SP");
        assert!(!looks_like_callsign("runway zero niner"));
        assert!(!looks_like_callsign(
            "contact London Control one three two decimal eight"
        ));
        assert!(looks_like_callsign("Speedbird 306"));
    }

    #[test]
    fn the_written_callsign_is_the_plans_when_it_is_ours() {
        assert_eq!(
            written_callsign("Speedbird three zero six heavy", &["BAW306".into()]).as_deref(),
            Some("BAW306")
        );
        assert_eq!(
            written_callsign("Whiskey Alpha Tango five one niner", &[]).as_deref(),
            Some("WAT519")
        );
        assert_eq!(
            written_callsign("Golf Alpha Bravo Charlie Delta", &["BAW306".into()]).as_deref(),
            Some("GABCD"),
            "a plan that is not the callsign in use is not shown"
        );
        assert_eq!(written_callsign("  ", &[]), None);
    }

    /// The panel's feed, labelled by the same rules.
    #[test]
    fn the_panel_rows_carry_the_same_labels() {
        let c = wat();
        assert_eq!(panel_label("Flaps five.", true, &c), "YOU > COPILOT");
        assert_eq!(panel_label("Gear down.", false, &c), "COPILOT > YOU");
        assert_eq!(
            panel_label("Cabin crew, seats for landing.", false, &c),
            "COPILOT > CABIN"
        );
        assert_eq!(
            panel_label("CAB: Cabin secure.", false, &c),
            "CABIN > FLIGHT DECK"
        );
        assert_eq!(
            panel_label(
                "TWR: Whiskey Alpha Tango five one niner, cleared to land.",
                false,
                &c
            ),
            "TOWER > WAT519"
        );
        assert_eq!(
            panel_label(
                "~TWR: Easy eight seven niner one, cleared to land.",
                false,
                &c
            ),
            "TOWER > EASY 8791"
        );
        assert_eq!(
            panel_label(
                "~TFC: Cleared to land, Easy eight seven niner one.",
                false,
                &c
            ),
            "EASY 8791 > TOWER"
        );
    }
}
