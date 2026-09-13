//! Spoken-number normalization for transcripts, before the grammar sees them.
//!
//! Whisper writes numbers inconsistently ("two seven zero", "niner", "decimal"),
//! and the commands here are number-heavy (headings, altitudes, frequencies,
//! squawks). This pass converts the *digit-by-digit* aviation style into plain
//! digits — "heading two seven zero" → "heading 270", "squawk one two zero zero"
//! → "squawk 1200", "one one eight decimal five" → "118.5" — so the model gets a
//! clean number.
//!
//! Deliberately conservative: it only collapses *runs of single-digit words*
//! (zero–nine). Additive numbers ("two thousand five hundred") contain
//! hundred/thousand and are left for the LLM, which handles them well — so we
//! never mis-fold "two thousand" into "2000"-style ambiguity here.

/// Value of a single spoken digit (0–9), including ICAO variants
/// (niner/fife/tree) and "oh"/"zero". The one place the digit vocabulary lives —
/// the STT normalizer, the grammar fast-path, and the bare-value gate all build
/// on this, so a spelling can't drift between them.
pub fn digit_value(w: &str) -> Option<i64> {
    Some(match w {
        "zero" | "oh" => 0,
        "one" => 1,
        "two" => 2,
        "three" | "tree" => 3,
        "four" => 4,
        "five" | "fife" => 5,
        "six" => 6,
        "seven" => 7,
        "eight" => 8,
        "nine" | "niner" => 9,
        _ => return None,
    })
}

/// Value of any single number word: a digit (via [`digit_value`]), a teen, a ten,
/// or a hundred/thousand multiplier — `(value, is_multiplier)`. Shared by the
/// grammar number parser and the bare-value gates so the vocabulary stays in one
/// place. (Stops at thousand: the grammar's additive parser hardcodes the
/// thousand step, so larger multipliers belong to the LLM.)
#[rustfmt::skip]
pub fn word_value(w: &str) -> Option<(i64, bool)> {
    if let Some(d) = digit_value(w) {
        return Some((d, false));
    }
    let n = match w {
        "ten" => 10, "eleven" => 11, "twelve" => 12, "thirteen" => 13,
        "fourteen" => 14, "fifteen" => 15, "sixteen" => 16, "seventeen" => 17,
        "eighteen" => 18, "nineteen" => 19, "twenty" => 20, "thirty" => 30,
        "forty" => 40, "fifty" => 50, "sixty" => 60, "seventy" => 70,
        "eighty" => 80, "ninety" => 90,
        "hundred" => return Some((100, true)),
        "thousand" => return Some((1000, true)),
        _ => return None,
    };
    Some((n, false))
}

/// True if `w` (lowercase) is any spoken number word.
pub fn is_number_word(w: &str) -> bool {
    word_value(w).is_some()
}

/// Single-digit words → digit char, for building digit strings.
fn digit_word(w: &str) -> Option<char> {
    digit_value(w).map(|d| (b'0' + d as u8) as char)
}

/// Magnitude words mark an *additive* number ("five hundred", "two thousand"),
/// which we leave for the LLM rather than folding digit-by-digit.
fn is_magnitude(w: &str) -> bool {
    matches!(w, "hundred" | "thousand" | "million")
}

/// A collapsible digit at `idx`: a single-digit word that is NOT immediately
/// followed by a magnitude word (which would make it additive).
fn collapsible_digit(tokens: &[&str], idx: usize) -> Option<char> {
    let core = split_trailing_punct(tokens[idx]).0.to_ascii_lowercase();
    let d = digit_word(&core)?;
    let next_is_magnitude = tokens
        .get(idx + 1)
        .map(|t| is_magnitude(&split_trailing_punct(t).0.to_ascii_lowercase()))
        .unwrap_or(false);
    (!next_is_magnitude).then_some(d)
}

/// Normalize spoken digit runs in `text`. Case-insensitive on number words;
/// other words are passed through with their original spelling and spacing.
pub fn normalize_numbers(text: &str) -> String {
    let tokens: Vec<&str> = text.split_whitespace().collect();
    let mut out: Vec<String> = Vec::with_capacity(tokens.len());
    let mut i = 0;
    while i < tokens.len() {
        if let Some(d) = collapsible_digit(&tokens, i) {
            // Start of a digit run — consume consecutive digit words, allowing a
            // "point"/"decimal" separator between them (for frequencies/altimeter).
            let mut digits = String::from(d);
            let mut last_trail = split_trailing_punct(tokens[i]).1;
            i += 1;
            while i < tokens.len() {
                if let Some(d) = collapsible_digit(&tokens, i) {
                    digits.push(d);
                    last_trail = split_trailing_punct(tokens[i]).1;
                    i += 1;
                } else {
                    let lc = split_trailing_punct(tokens[i]).0.to_ascii_lowercase();
                    if (lc == "point" || lc == "decimal") && peek_is_digit(&tokens, i + 1) {
                        digits.push('.');
                        last_trail = "";
                        i += 1;
                    } else {
                        break;
                    }
                }
            }
            out.push(format!("{digits}{last_trail}"));
        } else {
            out.push(tokens[i].to_string());
            i += 1;
        }
    }
    out.join(" ")
}

/// Split a token into its core and any trailing ASCII punctuation.
fn split_trailing_punct(tok: &str) -> (&str, &str) {
    let end = tok
        .rfind(|c: char| c.is_alphanumeric())
        .map(|i| i + tok[i..].chars().next().unwrap().len_utf8())
        .unwrap_or(0);
    tok.split_at(end)
}

/// Is the token at `idx` a digit word (ignoring trailing punctuation)?
fn peek_is_digit(tokens: &[&str], idx: usize) -> bool {
    tokens
        .get(idx)
        .map(|t| digit_word(&split_trailing_punct(t).0.to_ascii_lowercase()).is_some())
        .unwrap_or(false)
}

/// Replace em/en dashes with plain ASCII. X-Plane's TTS and the in-sim panel
/// font both choke on `—`/`–`, so no spoken or displayed text should carry them.
/// A spaced dash becomes a comma (a natural speech pause); a bare one a hyphen.
pub fn scrub_dashes(s: &str) -> String {
    if !s.contains('—') && !s.contains('–') {
        return s.to_string();
    }
    s.replace(" — ", ", ")
        .replace(" – ", ", ")
        .replace(['—', '–'], "-")
}

/// Word-set Jaccard similarity in 0..1, case/punctuation-insensitive. Used to spot
/// the copilot hearing its own readback over the speakers: a high score against
/// something it just spoke means the "command" is an echo, not the pilot.
pub fn word_similarity(a: &str, b: &str) -> f64 {
    use std::collections::HashSet;
    let toks = |s: &str| -> HashSet<String> {
        s.to_ascii_lowercase()
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|w| !w.is_empty())
            .map(str::to_string)
            .collect()
    };
    let (a, b) = (toks(a), toks(b));
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }
    a.intersection(&b).count() as f64 / a.union(&b).count() as f64
}

/// Fraction of `heard`'s words that also appear in `whole` (0..1). High means
/// `heard` is mostly *contained in* `whole` — which catches a partial echo of a
/// long readback, where the symmetric similarity is diluted by the readback's many
/// extra words. Pair with a minimum word count so a short real command that
/// happens to be a subset of a recent readback isn't dropped.
pub fn word_containment(heard: &str, whole: &str) -> f64 {
    use std::collections::HashSet;
    let toks = |s: &str| -> HashSet<String> {
        s.to_ascii_lowercase()
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|w| !w.is_empty())
            .map(str::to_string)
            .collect()
    };
    let (h, w) = (toks(heard), toks(whole));
    if h.is_empty() {
        return 0.0;
    }
    h.intersection(&w).count() as f64 / h.len() as f64
}

/// Number of word tokens in `s`.
pub fn word_count(s: &str) -> usize {
    s.split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .count()
}

/// Repair distinctive *whole-word* mishearings whisper makes on cockpit terms,
/// where the entire keyword is gone — so neither the domain gate's fuzzy match
/// nor the grammar's token match can recover it (those only catch near-misses
/// that keep the word, like "think gyro"). Only tokens that are meaningless in a
/// cockpit are rewritten, so the false-positive risk is negligible. Run on
/// spoken input before the gate, so both the gate and the grammar see the real
/// word.
pub fn fix_mishears(text: &str) -> String {
    // "gyro" (JY-roh) is reliably rendered as "J-Rule"/"J rule" (jay-rool), which
    // splits into tokens too far from "gyro" for the fuzzy gate to recover.
    let mut out = replace_ci(text, "j-rule", "gyro");
    out = replace_ci(&out, "j rule", "gyro");
    for (garble, fix) in PHRASE_MISHEARS {
        out = replace_word_ci(&out, garble, fix);
    }
    for (garble, fix, leading_only) in VALUE_MISHEARS {
        out = fix_value_mishear(&out, garble, fix, *leading_only);
    }
    fix_trailing_of(&out)
}

/// Whisper drops the final consonant of a trailing "off": "Engines of",
/// "electrical systems of" (four dead ends in one 2026-07-09 flight). An
/// utterance-final "of" is never real speech — "of" is a preposition and
/// needs an object — so the final word alone repairs; an "of" mid-sentence
/// ("state of charge") is untouched.
fn fix_trailing_of(text: &str) -> String {
    let trimmed = text
        .trim_end()
        .trim_end_matches(|c: char| !c.is_ascii_alphanumeric());
    if trimmed.len() >= 2
        && trimmed[trimmed.len() - 2..].eq_ignore_ascii_case("of")
        && !trimmed[..trimmed.len() - 2]
            .chars()
            .next_back()
            .is_some_and(|c| c.is_ascii_alphanumeric())
        && !trimmed[..trimmed.len() - 2].trim_end().is_empty()
    {
        let head = &text[..trimmed.len() - 2];
        let tail = &text[trimmed.len()..];
        format!("{head}off{tail}")
    } else {
        text.to_string()
    }
}

/// Unconditional whole-phrase repairs. Every entry is a whisper spelling SEEN
/// LIVE in the flight logs whose garble is meaningless in a cockpit (or only
/// meaningful as the repaired phrase), so no value guard is needed. Multi-word
/// needles match with word boundaries at both ends; case-insensitive.
const PHRASE_MISHEARS: &[(&str, &str)] = &[
    // Transponder VFR: whisper's favourite victim (8 dead ends in 4 days).
    ("vfa", "vfr"),                                   // "Transponder VFA."
    ("vfo", "vfr"),                                   // "Transponder VFO."
    ("via file", "vfr"),                              // "Transponder via file."
    ("responder", "transponder"),                     // "Responder VFA."
    ("coq", "squawk"),                                // "Set coq to VFR."
    ("transponder out mode", "transponder alt mode"), // "alt" heard as "out" (twice, 2026-07-10)
    // Strobes. Bare "probe"/"robe"/"show" are real words, so only the paired
    // "... light(s)" forms repair.
    ("strokes on", "strobes on"), // "Strokes on." (0.998 confidence!)
    ("strokes off", "strobes off"),
    ("shrobe", "strobe"),           // "Shrobe lights on."
    ("shrove", "strobe"),           // "Shrove lights off."
    ("robe light", "strobe light"), // "Robe light on."
    ("robe lights", "strobe lights"),
    ("probe light", "strobe light"), // "Probe lights on." (pitot probes
    ("probe lights", "strobe lights"), //  exist; probe LIGHTS don't)
    ("show lights on", "strobe lights on"),
    // Vertical speed.
    ("vertigo speed", "vertical speed"), // "Vertigo speed 0."
    ("medical speed", "vertical speed"), // "Medical speed 0."
    // Headings: garbled verb before a bare cardinal, and the bare "turn
    // <direction>" phrasing pilots keep using (repaired live by re-saying
    // "turn heading east").
    ("adding north", "heading north"), // "Adding north."
    ("adding south", "heading south"),
    ("adding east", "heading east"),
    ("adding west", "heading west"),
    ("bearing north", "heading north"), // "Bearing south."
    ("bearing south", "heading south"),
    ("bearing east", "heading east"),
    ("bearing west", "heading west"),
    ("turn north", "turn heading north"),
    ("turn south", "turn heading south"),
    ("turn east", "turn heading east"), // still dead-ending on 2026-07-09
    ("turn west", "turn heading west"),
    // Flap detents garbled on the value side.
    ("flaps fall", "flaps full"),    // "Flaps fall."
    ("flaps fold", "flaps full"),    // "Flaps fold."
    ("flaps cool", "flaps full"),    // "Flaps cool."
    ("flaps voting", "flaps forty"), // "Flaps voting."
    ("full flash", "full flaps"),    // "Full flash."
    // Parking brake.
    ("backing brake", "parking brake"), // "And set, backing brake."
    ("sparking brake", "parking brake"), // "Really? Sparking brake?"
    ("spoken brake", "parking brake"),  // "Release spoken brake."
    ("park and brake", "parking brake"), // "Park and brake off."
    ("back and break", "parking brake"), // "Set back and break."
    ("rock and break", "parking brake"), // "Drop, rock and break off."
    // Engines.
    ("engine zone", "engines on"),   // "engine zone." (three flights)
    ("jeans on", "engines on"),      // "And jeans on?" (2026-07-10)
    ("star engine", "start engine"), // "Star engines."
    ("star engines", "start engines"),
    // Climb / descend.
    ("they send to", "descend to"), // "They send to five thousand."
    ("claim to", "climb to"),       // "Claim to 8,000 feet." (Twotter, 2026-07-09)
    // Top of descent: whisper garbled it five ways in one evening flight
    // (2026-07-09) — each a dead-end retry of the same question.
    // Station names: whisper mangles "Heathrow" hard (2026-07-10 flight).
    ("hifra", "heathrow"), // "switch to hifra approach"
    ("heat flow approach", "heathrow approach"),
    ("land on heat flow", "london heathrow"),
    ("top of the dissent", "top of descent"),
    ("top of dissent", "top of descent"),
    ("top of the center", "top of descent"),
    ("top of this end", "top of descent"),
    // Props.
    ("props to cruz", "props to cruise"), // "Props to Cruz."
    ("pross", "props"),                   // "Max Pross?" (Twotter, 2026-07-09)
    ("full prompts", "full props"),       // "Full prompts?"
    // Controls and autopilot.
    ("mic controls", "my controls"),  // "Mic controls."
    ("nine controls", "my controls"), // "9 controls."
    ("9 controls", "my controls"),
    ("my control", "my controls"), // "My control." ("my controls" is
    //  boundary-safe: the s blocks it)
    ("autopilot dog", "autopilot off"),    // "Autopilot dog."
    ("go to pilot off", "autopilot off"),  // "Go to pilot off."
    ("all to pilot off", "autopilot off"), // "All to pilot off."
    ("flap school", "flaps full"),         // "I flap school."
    ("electrical systems down", "electrical systems off"),
    ("i like tricks off", "electrics off"), // "I like tricks off."
    ("said pounds of", "set power to"),     // "said pounds of 5 percent."
    // Undo: whisper renders it "I'm due" / "undue" ("100% undue." then "I'm
    // due." twice while the pilot fought to undo a throttle command). Neither
    // is cockpit speech; "undue stress" isn't said over PTT.
    ("i'm due", "undo"),
    ("undue", "undo"),
    // QNH.
    ("q&a", "qnh"), // "Q&A to 1017."
    // Fuel selector.
    ("view selector", "fuel selector"),  // "View selector right."
    ("queue selector", "fuel selector"), // "Queue selector left."
    ("power idol", "power idle"),        // "Power Idol."
];

/// Value-guarded keyword repairs: `(garble, replacement, leading_only)`. Every
/// entry is a whole-keyword mishear SEEN LIVE in the flight logs, rewritten only
/// when a settable value (number, percent, or lever/detent word) also appears in
/// the utterance — so prose uses of the same word ("total fuel", "describe the
/// fuel system") are never touched. `leading_only` restricts the rewrite to the
/// utterance's first word, for garbles that are real words elsewhere in a
/// sentence; a word that is meaningless anywhere in a cockpit repairs anywhere.
const VALUE_MISHEARS: &[(&str, &str, bool)] = &[
    ("total", "throttle", true),     // "Total twenty percent."
    ("frontal", "throttle", true),   // "Frontal 70." (leading-only: frontal weather is real speech)
    ("fraudal", "throttle", false),  // "Fraudal twenty percent."
    ("fraudful", "throttle", false), // "Fraudful 30%"
    ("flopful", "throttle", false),  // "Flopful 0."
    ("flottle", "throttle", false),  // "Flottle sixty percent."
    ("frottle", "throttle", false),
    ("thruffling", "throttle", false), // "maximum thruffling."
    ("commodal", "throttle", false),   // "Commodal twenty percent?"
    ("approval", "throttle", true),    // "Approval 50 percent."
    ("for all", "throttle", true),     // "For all, twenty percent."
    ("for also", "throttle", true),    // "For also twenty percent?"
    ("travel", "throttle", true), // "Travel ten percent." (leading-only: travel is real speech)
    ("waffle", "throttle", false), // "Reduce waffle to fifty percent."
    ("laps", "flaps", true),      // "Laps eleven."
    ("claps", "flaps", true),     // "Claps ten."
    ("taps", "flaps", true),      // "Taps twenty."
    ("map's", "flaps", true),     // "Map's up."
    ("maps", "flaps", true),
    ("collapse", "flaps", true),    // "collapse ten."
    ("flash", "flaps", true),       // "Flash full."
    ("crops", "props", true),       // "Crops, cruise."
    ("describe", "descend", true),  // "Describe five thousand feet."
    ("descends", "descend", false), // "Descends to one thousand five hundred."
    ("clank", "climb", true),       // "Clank five thousand feet."
    ("client", "climb", true),      // "Client five thousand feet."
];

/// Apply one [`VALUE_MISHEARS`] entry: rewrite `garble` to `fix` when the
/// utterance carries a value for it to set. Whole-word only, case-insensitive,
/// original spacing and casing elsewhere preserved.
fn fix_value_mishear(text: &str, garble: &str, fix: &str, leading_only: bool) -> String {
    let low = text.to_ascii_lowercase();
    let has_value = low.contains("percent")
        || low.contains('%')
        || low.split(|c: char| !c.is_ascii_alphanumeric()).any(|w| {
            !w.is_empty()
                && (is_number_word(w)
                    || w.bytes().all(|b| b.is_ascii_digit())
                    || matches!(
                        w,
                        "full" | "idle" | "max" | "maximum" | "up" | "down" | "cruise"
                    ))
        });
    if !has_value {
        return text.to_string();
    }
    if leading_only {
        let trimmed = text.trim_start();
        let leads = trimmed.len() >= garble.len()
            && trimmed[..garble.len()].eq_ignore_ascii_case(garble)
            && trimmed[garble.len()..]
                .chars()
                .next()
                .is_none_or(|c| !c.is_ascii_alphanumeric());
        if !leads {
            return text.to_string();
        }
        let lead = &text[..text.len() - trimmed.len()];
        format!("{lead}{fix}{}", &trimmed[garble.len()..])
    } else {
        replace_word_ci(text, garble, fix)
    }
}

/// Replace every case-insensitive WHOLE-WORD occurrence of `needle` (given in
/// lowercase) with `replacement` — an occurrence flanked by alphanumerics is
/// part of a longer word and left alone ("waffles" keeps its "waffle").
fn replace_word_ci(haystack: &str, needle: &str, replacement: &str) -> String {
    let lower = haystack.to_ascii_lowercase();
    let bytes = lower.as_bytes();
    let mut out = String::with_capacity(haystack.len());
    let mut i = 0;
    while let Some(pos) = lower[i..].find(needle) {
        let start = i + pos;
        let end = start + needle.len();
        let standalone = (start == 0 || !bytes[start - 1].is_ascii_alphanumeric())
            && bytes.get(end).is_none_or(|b| !b.is_ascii_alphanumeric());
        out.push_str(&haystack[i..start]);
        out.push_str(if standalone {
            replacement
        } else {
            &haystack[start..end]
        });
        i = end;
    }
    out.push_str(&haystack[i..]);
    out
}

/// Join intra-word hyphens whisper inserts into compounds ("auto-pilot" →
/// "autopilot", "pre-flight" → "preflight") so the keyword gate and the grammar
/// see one word — otherwise they split on the hyphen and the keyword is lost.
/// Only a hyphen flanked by ASCII letters is removed; a digit hyphen is left
/// intact (it isn't a compound, and could be a range).
pub fn collapse_hyphens(text: &str) -> String {
    let chars: Vec<char> = text.chars().collect();
    let mut out = String::with_capacity(text.len());
    for (i, &c) in chars.iter().enumerate() {
        let joins_letters = c == '-'
            && i > 0
            && chars[i - 1].is_ascii_alphabetic()
            && chars.get(i + 1).is_some_and(|n| n.is_ascii_alphabetic());
        if !joins_letters {
            out.push(c);
        }
    }
    out
}

/// Replace every case-insensitive occurrence of `needle` (which must be given in
/// lowercase) with `replacement`, preserving the surrounding text verbatim.
fn replace_ci(haystack: &str, needle: &str, replacement: &str) -> String {
    let lower = haystack.to_ascii_lowercase();
    let mut out = String::with_capacity(haystack.len());
    let mut i = 0;
    while let Some(pos) = lower[i..].find(needle) {
        let start = i + pos;
        out.push_str(&haystack[i..start]);
        out.push_str(replacement);
        i = start + needle.len();
    }
    out.push_str(&haystack[i..]);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn echo_scores_high_real_command_low() {
        // A full readback coming back (numbers spelled out, minor mishears) scores
        // above the 0.55 echo threshold.
        let spoken = "Roger, autopilot engaged, vertical speed mode, climb 500 feet per minute.";
        let echo =
            "Roger. Autopilot engaged. Vertical speed mode. Climb five hundred feet per minute.";
        assert!(
            word_similarity(echo, spoken) >= 0.55,
            "echo should score high"
        );
        // An echo fragment glued to a real command stays below, so the command survives.
        let mixed = "Throttle one hundred percent. Descend to three thousand feet.";
        assert!(
            word_similarity(mixed, "Throttle 100 percent.") < 0.55,
            "mixed should score low"
        );
        // Unrelated commands don't collide.
        assert!(word_similarity("flaps up", "throttle 100 percent") < 0.55);
        // A partial echo of a long readback: low Jaccard, but high containment.
        let spoken = "We're in the McDonnell Douglas MD-82, a twin-engine jet. Cruise about 439 knots. Max speed 340. Service ceiling 37,000 feet.";
        let echo = "We're in the McDonnell Douglas MD-82, a twin engine jet cruise.";
        assert!(word_similarity(echo, spoken) < 0.55, "diluted Jaccard");
        assert!(
            word_containment(echo, spoken) >= 0.85,
            "but mostly contained"
        );
        // A short real command isn't mostly-contained enough to be a false echo.
        assert!(
            word_containment("heading 180", "autopilot engaged, heading mode, bug to 270") < 0.85
        );
    }

    #[test]
    fn fixes_gyro_mishears() {
        // "gyro" lost to "J-Rule"/"J rule" is restored, case-insensitively.
        assert_eq!(
            fix_mishears("And then you fix the J-Rule."),
            "And then you fix the gyro."
        );
        assert_eq!(fix_mishears("you think the j rule?"), "you think the gyro?");
        // Unrelated text is untouched.
        assert_eq!(fix_mishears("align the gyro"), "align the gyro");
    }

    /// Every whisper spelling in the mishear tables, exercised with the exact
    /// transcript from the flight log that put it there. `(garbled, repaired)`
    /// pairs; the repaired form is what the grammar must see.
    #[test]
    fn fixes_logged_whisper_garbles() {
        for (heard, want) in [
            // Transponder VFR (the biggest dead-end cluster).
            ("Transponder VFA.", "Transponder vfr."),
            ("Transponder VFO.", "Transponder vfr."),
            ("Transponder via file.", "Transponder vfr."),
            ("Responder VFA.", "transponder vfr."),
            ("Set coq to VFR.", "Set squawk to VFR."),
            // Strobes.
            ("Shrobe lights on.", "strobe lights on."),
            ("Shrove lights off.", "strobe lights off."),
            ("Robe light on.", "strobe light on."),
            ("Probe lights on.", "strobe lights on."),
            ("Show lights on.", "strobe lights on."),
            // Vertical speed.
            ("Vertigo speed 0.", "vertical speed 0."),
            ("Medical speed 0.", "vertical speed 0."),
            // Headings.
            ("Adding north.", "heading north."),
            ("Bearing south.", "heading south."),
            ("Turn east.", "turn heading east."),
            ("Turn west.", "turn heading west."),
            // Throttle family.
            ("Fraudful 30%", "throttle 30%"),
            ("Flottle sixty percent.", "throttle sixty percent."),
            ("maximum thruffling.", "maximum throttle."),
            ("Commodal twenty percent?", "throttle twenty percent?"),
            ("Approval 50 percent.", "throttle 50 percent."),
            ("For all, twenty percent.", "throttle, twenty percent."),
            ("For also twenty percent?", "throttle twenty percent?"),
            ("Travel ten percent.", "throttle ten percent."),
            ("Power Idol.", "power idle."),
            // Flaps.
            ("Taps twenty.", "flaps twenty."),
            ("Map's up.", "flaps up."),
            ("collapse ten.", "flaps ten."),
            ("Flash full.", "flaps full."),
            ("Full flash.", "full flaps."),
            ("Flaps fall.", "flaps full."),
            ("Flaps cool.", "flaps full."),
            ("Flaps voting.", "flaps forty."),
            // Parking brake.
            ("And set, backing brake.", "And set, parking brake."),
            ("Really? Sparking brake?", "Really? parking brake?"),
            ("Release spoken brake.", "Release parking brake."),
            ("Park and brake off.", "parking brake off."),
            // Engines.
            ("engine zone.", "engines on."),
            ("And jeans on?", "And engines on?"),
            ("Star engines.", "start engines."),
            // Climb / descend.
            ("They send to five thousand.", "descend to five thousand."),
            ("Client five thousand feet.", "climb five thousand feet."),
            // Props.
            ("Props to Cruz.", "props to cruise."),
            ("Full prompts?", "full props?"),
            ("Crops, cruise.", "props, cruise."),
            // Controls.
            ("Mic controls.", "my controls."),
            ("9 controls.", "my controls."),
            ("My control.", "my controls."),
            ("Autopilot dog.", "autopilot off."),
            // QNH.
            ("Q&A to 1017.", "qnh to 1017."),
            // Fuel selector.
            ("View selector right.", "fuel selector right."),
            ("Queue selector left.", "fuel selector left."),
            // 2026-07-09 second flight.
            ("Go to pilot off.", "autopilot off."),
            ("All to pilot off.", "autopilot off."),
            ("I flap school.", "I flaps full."),
            ("electrical systems down.", "electrical systems off."),
            ("Flopful 0.", "throttle 0."),
            // A trailing "off" losing its second f.
            ("Engines of", "Engines off"),
            ("electrical systems of", "electrical systems off"),
            ("Electrical systems of.", "Electrical systems off."),
            // 2026-07-09 afternoon flight.
            ("Strokes on.", "strobes on."),
            ("Set back and break.", "Set parking brake."),
            ("Drop, rock and break off.", "Drop, parking brake off."),
            ("I like tricks off.", "electrics off."),
            ("said pounds of 5 percent.", "set power to 5 percent."),
            ("I'm due.", "undo."),
            ("100% undue.", "100% undo."),
            (
                "Descends to one thousand five hundred.",
                "descend to one thousand five hundred.",
            ),
            // 2026-07-09 Twin Otter flights.
            ("Claim to 8,000 feet.", "climb to 8,000 feet."),
            ("Max Pross?", "Max props?"),
            // 2026-07-09 evening cruise: "top of descent" in five disguises.
            (
                "How much for top of the dissent?",
                "How much for top of descent?",
            ),
            (
                "How much for top of the center?",
                "How much for top of descent?",
            ),
            (
                "Can you remind me when we reach top of this end?",
                "Can you remind me when we reach top of descent?",
            ),
            (
                "Can you remind me when we reach top of the center?",
                "Can you remind me when we reach top of descent?",
            ),
        ] {
            assert_eq!(fix_mishears(heard), want, "garble {heard:?}");
        }
        // Mid-sentence "of" is real speech and survives.
        assert_eq!(fix_mishears("state of charge"), "state of charge");
        assert_eq!(
            fix_mishears("of"),
            "of",
            "a lone 'of' has nothing to repair"
        );
    }

    /// Real words that share letters with the garbles must survive untouched:
    /// the repairs only fire on whole words/phrases, and the value-guarded
    /// ones only with a settable value present.
    #[test]
    fn mishear_fixes_leave_real_speech_alone() {
        for s in [
            "the pitot probe heats up",
            "show me the lights",
            "my controls",
            "what's the bearing to the field",
            "turn to the east side of the bay",
            "we travel a lot",
            "how long is the flight",
            "return east after departure",
            "strobe lights on",
            "what's our total fuel",
            "flash of lightning ahead",
            "the client is happy",
            "the aircraft descends smoothly",
            "keep your strokes smooth on the yoke",
        ] {
            assert_eq!(fix_mishears(s), s, "{s:?} must not be rewritten");
        }
    }

    #[test]
    fn fixes_total_throttle_mishear() {
        // "throttle" → "total" before a value is recovered.
        assert_eq!(
            fix_mishears("Total twenty percent."),
            "throttle twenty percent."
        );
        assert_eq!(fix_mishears("total 40"), "throttle 40");
        assert_eq!(fix_mishears("total full"), "throttle full");
        // A genuine "total" (no throttle value) is left alone.
        assert_eq!(
            fix_mishears("what's our total fuel"),
            "what's our total fuel"
        );
        assert_eq!(fix_mishears("total fuel"), "total fuel");
        assert_eq!(fix_mishears("totally fine"), "totally fine");
        assert_eq!(fix_mishears("flaps ten"), "flaps ten");
    }

    #[test]
    fn fixes_value_guarded_mishears_from_the_logs() {
        // Every garble here was transcribed live (2026-07-02..04 logs) and fell
        // through as a deterministic-miss. With a value present, it repairs.
        assert_eq!(fix_mishears("Laps eleven."), "flaps eleven.");
        assert_eq!(fix_mishears("Claps ten"), "flaps ten");
        assert_eq!(fix_mishears("laps up"), "flaps up");
        assert_eq!(fix_mishears("Frontal 70"), "throttle 70");
        assert_eq!(
            fix_mishears("Fraudal twenty percent."),
            "throttle twenty percent."
        );
        assert_eq!(
            fix_mishears("Reduce waffle to fifty percent."),
            "Reduce throttle to fifty percent."
        );
        assert_eq!(
            fix_mishears("Describe five thousand feet."),
            "descend five thousand feet."
        );
        assert_eq!(
            fix_mishears("Clank five thousand feet"),
            "climb five thousand feet"
        );
        // The value guard: the same words in prose stay untouched.
        assert_eq!(
            fix_mishears("describe the fuel system"),
            "describe the fuel system"
        );
        assert_eq!(
            fix_mishears("a frontal system is passing"),
            "a frontal system is passing"
        );
        assert_eq!(fix_mishears("she claps"), "she claps");
        // Whole-word only: no rewriting inside longer words.
        assert_eq!(fix_mishears("waffles at 9"), "waffles at 9");
    }

    #[test]
    fn collapses_intra_word_hyphens_only() {
        // Compound the keyword gate/grammar would otherwise split on.
        assert_eq!(collapse_hyphens("auto-pilot off"), "autopilot off");
        assert_eq!(collapse_hyphens("pre-flight check"), "preflight check");
        // Digit hyphens and dangling hyphens are left intact.
        assert_eq!(collapse_hyphens("flaps 10"), "flaps 10");
        assert_eq!(collapse_hyphens("set 29-92"), "set 29-92");
        assert_eq!(collapse_hyphens("- gear up"), "- gear up");
    }

    #[test]
    fn collapses_digit_runs() {
        assert_eq!(normalize_numbers("heading two seven zero"), "heading 270");
        assert_eq!(normalize_numbers("squawk one two zero zero"), "squawk 1200");
        assert_eq!(
            normalize_numbers("descend at five hundred"),
            "descend at five hundred"
        ); // additive left alone
    }

    #[test]
    fn handles_icao_variants_and_decimals() {
        assert_eq!(
            normalize_numbers("altimeter two niner niner two"),
            "altimeter 2992"
        );
        assert_eq!(
            normalize_numbers("com one one eight decimal five"),
            "com 118.5"
        );
        assert_eq!(
            normalize_numbers("nav one zero nine point five"),
            "nav 109.5"
        );
        assert_eq!(normalize_numbers("squawk one two tree fife"), "squawk 1235");
    }

    #[test]
    fn preserves_punctuation_and_words() {
        assert_eq!(normalize_numbers("flaps ten"), "flaps ten"); // "ten" isn't a single digit
        assert_eq!(normalize_numbers("heading two seven zero."), "heading 270.");
        assert_eq!(normalize_numbers("gear up"), "gear up");
        // A lone "point" with no following digit is left as a word.
        assert_eq!(normalize_numbers("to the point"), "to the point");
    }
}

/// A line with the copilot's own coaching of what to SAY cut out, for the
/// echo memory. "We're already flying, so full auto can't start. Say navigate
/// to follow the plan, fly the ILS for the arrival, or land for me." coached
/// exactly the words the pilot then spoke, and the echo gate rejected
/// "Navigate to follow the plan" as the copilot's voice bleeding back through
/// the mic, without a word (LEMG, 2026-09-12). Every sentence that opens with
/// "say" (or "just say", "or say", "then say") goes; the rest of the line
/// still guards against real echo.
pub fn without_coached_phrase(text: &str) -> String {
    let mut out = String::new();
    let mut start = 0;
    let bytes = text.as_bytes();
    for (i, b) in bytes.iter().enumerate() {
        if matches!(b, b'.' | b'!' | b'?') || i + 1 == bytes.len() {
            let end = if matches!(b, b'.' | b'!' | b'?') { i + 1 } else { i + 1 };
            let sentence = &text[start..end];
            let lead = sentence
                .trim_start_matches(|c: char| !c.is_alphanumeric())
                .to_ascii_lowercase();
            let coached = ["say ", "just say ", "or say ", "then say ", "you can say "]
                .iter()
                .any(|p| lead.starts_with(p));
            if !coached {
                out.push_str(sentence);
            }
            start = end;
        }
    }
    out
}

#[cfg(test)]
mod coached_tests {
    use super::without_coached_phrase;

    #[test]
    fn a_coached_phrase_leaves_the_echo_memory() {
        let line = "We're already flying, so full auto can't start. Say navigate to follow the plan, fly the ILS for the arrival, or land for me.";
        let kept = without_coached_phrase(line);
        assert!(kept.contains("already flying"));
        assert!(!kept.to_lowercase().contains("navigate"), "{kept}");
        // A plain readback is untouched.
        assert_eq!(without_coached_phrase("Flaps ten, set."), "Flaps ten, set.");
        // "Say again" is the copilot's own line too, and stays.
        assert_eq!(without_coached_phrase("Say again?"), "");
    }
}
