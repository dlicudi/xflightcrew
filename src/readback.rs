//! Spoken-response variation — so the copilot doesn't repeat itself word-for-word.
//!
//! Two scoped kinds of variety, deliberately limited (real phraseology is terse
//! and standardized — we vary the *wrapper*, never the value):
//!
//! - [`vary_ack`]: tools confirm an affirmative action with a leading "Roger"
//!   (e.g. `"Roger, flaps ten."`). That marker means "this is a successful
//!   confirmation", so only those lines get a varied acknowledgment — most often
//!   *none* (the bare callout "Flaps ten." is the most authentic), occasionally
//!   "Roger,"/"Copy,". Lines without the marker (refusals, errors, prompts) pass
//!   through untouched, so we never prepend an affirmation to a refusal.
//! - [`one_of`]: pick one phrasing from a set, for the repeated conversational
//!   lines ("Say again?" / "Didn't catch that.") where exact wording doesn't
//!   matter and repetition grates most.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// Cheap process-wide xorshift PRNG (no `rand` dependency). Lazily seeded from
/// the clock; a benign race on first use only affects which variant is picked.
fn rng_next() -> u64 {
    static STATE: AtomicU64 = AtomicU64::new(0);
    let mut s = STATE.load(Ordering::Relaxed);
    if s == 0 {
        s = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0x9E37_79B9_7F4A_7C15)
            | 1; // never zero (xorshift would stay stuck)
    }
    s ^= s << 13;
    s ^= s >> 7;
    s ^= s << 17;
    STATE.store(s, Ordering::Relaxed);
    s
}

/// Pick one of `opts` at random. Returns `""` if empty.
pub fn one_of<'a>(opts: &[&'a str]) -> &'a str {
    if opts.is_empty() {
        return "";
    }
    opts[(rng_next() % opts.len() as u64) as usize]
}

/// Vary the acknowledgment on an affirmative action readback. Lines the tools
/// mark with a leading "Roger" become a bare callout (most of the time) or get a
/// rotated prefix; everything else is returned unchanged.
pub fn vary_ack(line: &str) -> String {
    let Some(body) = strip_roger(line) else {
        return line.to_string(); // not an affirmative confirmation — leave it
    };
    match weighted_prefix() {
        None => capitalize_first(body),     // "Flaps ten."
        Some(p) => format!("{p} {body}"),   // "Copy, flaps ten."
    }
}

/// Strip a leading "Roger" plus its connector punctuation, returning the callout
/// body. `None` if the line doesn't start with the affirmative marker.
fn strip_roger(line: &str) -> Option<&str> {
    let rest = line.strip_prefix("Roger")?;
    let body = rest.trim_start_matches([',', '—', '-', ':', ' ']);
    // Guard: bare "Roger" or "Roger." with no body isn't an action callout.
    (!body.is_empty()).then_some(body)
}

/// The acknowledgment prefix: ~60% none (bare callout), then "Roger,"/"Copy,".
/// Weighted toward none so the bare standard callout dominates and "Roger" stays
/// a signature rather than wallpaper.
fn weighted_prefix() -> Option<&'static str> {
    match rng_next() % 10 {
        0..=5 => None,                  // 60% — "Flaps ten."
        6..=8 => Some("Roger,"),        // 30%
        _ => Some("Copy,"),             // 10%
    }
}

fn capitalize_first(s: &str) -> String {
    let mut chars = s.chars();
    match chars.next() {
        Some(first) => first.to_uppercase().collect::<String>() + chars.as_str(),
        None => String::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn affirmatives_vary_but_keep_the_callout() {
        // Over many draws, every rendering must preserve the callout exactly and
        // be one of the allowed wrappers (bare-capitalized or a known prefix).
        let allowed = ["Flaps ten.", "Roger, flaps ten.", "Copy, flaps ten.", "Wilco, flaps ten."];
        for _ in 0..200 {
            let out = vary_ack("Roger, flaps ten.");
            assert!(allowed.contains(&out.as_str()), "unexpected rendering: {out:?}");
        }
    }

    #[test]
    fn em_dash_style_is_handled() {
        let allowed = [
            "Autopilot off. You have the airplane.",
            "Roger, autopilot off. You have the airplane.",
            "Copy, autopilot off. You have the airplane.",
            "Wilco, autopilot off. You have the airplane.",
        ];
        for _ in 0..200 {
            let out = vary_ack("Roger — autopilot off. You have the airplane.");
            assert!(allowed.contains(&out.as_str()), "unexpected: {out:?}");
        }
    }

    #[test]
    fn non_affirmatives_pass_through_untouched() {
        for line in [
            "Gear stays down — we're on the ground.",
            "Confirm, gear up?",
            "Something went wrong actioning that.",
            "Say again?",
        ] {
            assert_eq!(vary_ack(line), line, "{line:?} must be unchanged");
        }
    }

    #[test]
    fn one_of_returns_a_member() {
        let opts = ["a", "b", "c"];
        for _ in 0..50 {
            assert!(opts.contains(&one_of(&opts)));
        }
        assert_eq!(one_of(&[]), "");
    }
}
