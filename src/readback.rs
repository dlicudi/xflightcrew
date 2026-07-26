//! Spoken-response variation — so the copilot doesn't repeat itself word-for-word.
//!
//! Two scoped kinds of variety, deliberately limited (real phraseology is terse
//! and standardized — we vary the *wrapper*, never the value):
//!
//! - [`vary_ack`]: tools confirm an affirmative action with a leading "Roger"
//!   (e.g. `"Roger, flaps ten."`). That marker means "this is a successful
//!   confirmation", so only those lines get a varied acknowledgment — most often
//!   *none* (the bare callout "Flaps ten." is the most authentic), otherwise a
//!   conversational ack. The wrappers are INTERCOM words ("Okay," "Got it,"),
//!   not radio phraseology: the right seat is talking to you across the cockpit,
//!   not keying a mic, so "Roger"/"Copy" stay in the mix but rare (that lives on
//!   the radio channel). And the same wrapper never lands twice running — a
//!   fixed prefix on every callout is the machine tell we're spending this whole
//!   module to kill. Lines without the marker (refusals, errors, prompts) pass
//!   through untouched, so we never prepend an affirmation to a refusal.
//! - [`one_of`]: pick one phrasing from a set, for the repeated conversational
//!   lines ("Say again?" / "Didn't catch that.") where exact wording doesn't
//!   matter and repetition grates most.

use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
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
///
/// `busy` is the heads-down signal (takeoff/landing roll): a real crew goes
/// terse when the workload is high, so a busy ack is always the bare callout —
/// no conversational wrapper, no trailing tag.
pub fn vary_ack(line: &str, busy: bool) -> String {
    let Some(body) = strip_roger(line) else {
        return line.to_string(); // not an affirmative confirmation — leave it
    };
    let callout = capitalize_first(body); // "Flaps ten."
                                          // Heads-down on the roll: terse. Leave the anti-repeat state untouched so a
                                          // busy spell doesn't bias which wrapper the next quiet ack collapses away.
    if busy {
        return callout;
    }
    // A small share render as a trailing tag ("Flaps ten, got it.") — the one
    // ack that reads naturally AFTER the value. Only on a clean single-clause
    // callout, so we never tag the second sentence of a two-part readback.
    if let Some(tagged) = trailing_ack(&callout) {
        return tagged;
    }
    match next_prefix() {
        None => callout,
        Some(p) => format!("{p} {body}"), // "Got it, flaps ten."
    }
}

/// Occasionally (~10%) render a single-clause callout with a trailing "got it."
/// Returns `None` when it doesn't fire, or when the callout isn't a clean single
/// clause ending in a period (a two-sentence readback would tag its tail oddly).
fn trailing_ack(callout: &str) -> Option<String> {
    if rng_next() % 10 != 0 {
        return None;
    }
    let head = callout.strip_suffix('.')?;
    // Interior ". " means more than one sentence — leave those to lead or bare.
    if head.contains(". ") {
        return None;
    }
    Some(format!("{head}, got it."))
}

/// Strip a leading "Roger" plus its connector punctuation, returning the callout
/// body. `None` if the line doesn't start with the affirmative marker.
fn strip_roger(line: &str) -> Option<&str> {
    let rest = line.strip_prefix("Roger")?;
    let body = rest.trim_start_matches([',', '—', '-', ':', ' ']);
    // Guard: bare "Roger" or "Roger." with no body isn't an action callout.
    (!body.is_empty()).then_some(body)
}

/// The acknowledgment wrappers, in the atomic-index order used below (index 0 is
/// the bare callout — no prefix). Intercom-first: "Okay,"/"Got it," are what you
/// say across the cockpit; "Roger,"/"Copy," are radio words kept rare so they
/// read as flavour, not the default.
const PREFIXES: &[&str] = &["Okay,", "Got it,", "Alright,", "Roger,", "Copy,"];

/// Pick the acknowledgment prefix: ~55% bare callout, the rest a conversational
/// ack weighted toward the intercom words. Never repeats the previous non-bare
/// wrapper — a fixed prefix on every callout is exactly what reads as robotic —
/// falling back to the bare callout (standard callouts DO repeat, so that's the
/// natural place to collapse to).
fn next_prefix() -> Option<&'static str> {
    // Index into PREFIXES + 1; 0 == bare. LAST remembers the previous emission
    // so we can forbid an immediate repeat of the same wrapper.
    static LAST: AtomicU8 = AtomicU8::new(0);
    let idx = match rng_next() % 100 {
        0..=54 => 0,  // 55% — bare "Flaps ten."
        55..=71 => 1, // "Okay,"
        72..=84 => 2, // "Got it,"
        85..=92 => 3, // "Alright,"
        93..=97 => 4, // "Roger,"
        _ => 5,       // "Copy,"
    };
    // Same wrapper twice running → collapse to the bare callout instead.
    let idx = if idx != 0 && idx == LAST.load(Ordering::Relaxed) {
        0
    } else {
        idx
    };
    LAST.store(idx, Ordering::Relaxed);
    (idx != 0).then(|| PREFIXES[(idx - 1) as usize])
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
    use std::sync::Mutex;

    /// The anti-repeat state (`next_prefix`'s `LAST`) is one process-wide static;
    /// tests that draw acks must not interleave, or one test's calls stop being
    /// consecutive from `LAST`'s view and the no-repeat assertion goes flaky.
    static ACK_LOCK: Mutex<()> = Mutex::new(());

    /// The wrapper on a rendered ack: the leading "Prefix," or `None` for the
    /// bare callout. Shared by the variation tests.
    fn wrapper_of(out: &str) -> Option<&'static str> {
        PREFIXES.iter().copied().find(|p| out.starts_with(p))
    }

    #[test]
    fn affirmatives_vary_but_keep_the_callout() {
        let _g = ACK_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Over many draws every rendering must preserve the callout exactly, as
        // one of: bare, a leading wrapper, or the trailing "got it." tag.
        for _ in 0..300 {
            let out = vary_ack("Roger, flaps ten.", false);
            let ok = out == "Flaps ten."
                || out == "Flaps ten, got it."
                || wrapper_of(&out).is_some_and(|p| out == format!("{p} flaps ten."));
            assert!(ok, "unexpected rendering: {out:?}");
        }
    }

    #[test]
    fn em_dash_style_is_handled() {
        let _g = ACK_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Multi-sentence body: never gets the trailing tag, so bare or leading only.
        let body = "autopilot off. You have the airplane.";
        for _ in 0..200 {
            let out = vary_ack("Roger — autopilot off. You have the airplane.", false);
            match wrapper_of(&out) {
                None => assert_eq!(
                    out, "Autopilot off. You have the airplane.",
                    "bare: {out:?}"
                ),
                Some(p) => assert_eq!(out, format!("{p} {body}"), "unexpected: {out:?}"),
            }
        }
    }

    #[test]
    fn busy_acks_are_always_the_bare_callout() {
        let _g = ACK_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Heads-down on the roll: no wrapper, no trailing tag, ever.
        for _ in 0..200 {
            assert_eq!(vary_ack("Roger, flaps up.", true), "Flaps up.");
        }
    }

    #[test]
    fn wrappers_are_intercom_words_and_never_repeat_back_to_back() {
        let _g = ACK_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // The same non-bare wrapper must never land on two consecutive acks — a
        // fixed prefix every time is the machine tell this module exists to kill.
        let mut prev: Option<&str> = None;
        for _ in 0..500 {
            let out = vary_ack("Roger, gear up.", false);
            if let Some(p) = wrapper_of(&out) {
                assert_ne!(Some(p), prev, "wrapper {p:?} repeated back-to-back");
            }
            prev = wrapper_of(&out);
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
            assert_eq!(vary_ack(line, false), line, "{line:?} must be unchanged");
            assert_eq!(
                vary_ack(line, true),
                line,
                "{line:?} must be unchanged when busy"
            );
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
