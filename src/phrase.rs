//! Spoken-number normalization for transcripts, before the LLM sees them.
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
    fn fixes_gyro_mishears() {
        // "gyro" lost to "J-Rule"/"J rule" is restored, case-insensitively.
        assert_eq!(fix_mishears("And then you fix the J-Rule."), "And then you fix the gyro.");
        assert_eq!(fix_mishears("you think the j rule?"), "you think the gyro?");
        // Unrelated text is untouched.
        assert_eq!(fix_mishears("align the gyro"), "align the gyro");
        assert_eq!(fix_mishears("flaps ten"), "flaps ten");
    }

    #[test]
    fn collapses_digit_runs() {
        assert_eq!(normalize_numbers("heading two seven zero"), "heading 270");
        assert_eq!(normalize_numbers("squawk one two zero zero"), "squawk 1200");
        assert_eq!(normalize_numbers("descend at five hundred"), "descend at five hundred"); // additive left alone
    }

    #[test]
    fn handles_icao_variants_and_decimals() {
        assert_eq!(normalize_numbers("altimeter two niner niner two"), "altimeter 2992");
        assert_eq!(normalize_numbers("com one one eight decimal five"), "com 118.5");
        assert_eq!(normalize_numbers("nav one zero nine point five"), "nav 109.5");
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
