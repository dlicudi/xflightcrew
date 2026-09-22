//! The dialogue stack: deterministic back-references to what was just done.
//!
//! "A bit more", "come left ten", "turn it back on", "do that again" are not
//! commands the grammar can parse — they only mean something relative to the
//! action that came before. The grammar deliberately bails on them (its
//! `RELATIVE` list) and the LLM fallback is answer-only, so until now they
//! were dead air. This module records every tool call the loop actually
//! executed and resolves those shapes against it, with no LLM:
//!
//! - **Closed vocabulary.** An utterance with a single word outside the
//!   shape's whitelist never resolves — "more coffee" is chatter, not a speed
//!   change. Resolution can only ever be triggered on purpose.
//! - **Freshness TTL.** An anchor older than [`ANCHOR_TTL`] is stale context,
//!   not intent: "a bit more" ten minutes after the last speed change gets an
//!   honest miss, never a guess.
//! - **Automations are never re-armed from a back-reference.** "Again"
//!   repeats a switch, not a takeoff (`REPEAT_DENY`): the same rule as the
//!   arming grammars accepting request shapes only.
//!
//! Resolved calls ride the normal tool path, so every safety gate and
//! intent-checked readback still applies — this module picks the call, it
//! never touches a dataref. The stack is cleared on an airframe swap
//! (a process-wide fact must not outlive its airframe).

use std::time::{Duration, Instant};

use serde_json::Value;

use crate::phrase::word_value;
use crate::tools::Registry;
use crate::tools::ToolCall;

/// How long a recorded action stays a valid anchor for a back-reference.
/// Matches the conversational horizon: past this, "a bit more" has lost its
/// antecedent and an honest miss beats a stale guess.
pub const ANCHOR_TTL: Duration = Duration::from_secs(120);

/// Bounded depth: back-references reach the recent past, not the whole flight.
const STACK_MAX: usize = 8;

/// Tools whose repetition would arm an automation or move the aircraft. A
/// vague "again" must never do either — say the full command.
const REPEAT_DENY: &[&str] = &[
    "full_auto",
    "circuit",
    "fly_procedure",
    "auto_takeoff",
    "auto_taxi",
    "autoland",
    "visual_approach",
    "prepare_ils",
    "pushback",
    "line_up",
    "navigate",
    "taxi_to_parking",
    "recover",
];

/// A numeric argument the pilot can nudge relative to its last commanded
/// value. `subjects` are the words that name it out loud; `wrap` is for the
/// compass. The step is the unspoken default for "a bit"; an explicit number
/// in the utterance always wins.
struct Adjustable {
    tool: &'static str,
    key: &'static str,
    step: f64,
    min: f64,
    max: f64,
    wrap: bool,
    subjects: &'static [&'static str],
}

const ADJUSTABLE: &[Adjustable] = &[
    // Heading only ever moves by left/right, never by more/less — "a bit
    // more" cannot mean a turn direction, so the compass row is excluded
    // from generic increase/decrease below.
    Adjustable {
        tool: "set_heading",
        key: "heading",
        step: 10.0,
        min: 0.0,
        max: 359.0,
        wrap: true,
        subjects: &["heading"],
    },
    Adjustable {
        tool: "set_altitude",
        key: "altitude",
        step: 500.0,
        min: 0.0,
        max: 60000.0,
        wrap: false,
        subjects: &["altitude"],
    },
    Adjustable {
        tool: "set_vertical_speed",
        key: "fpm",
        step: 100.0,
        min: -6000.0,
        max: 6000.0,
        wrap: false,
        subjects: &["rate", "vertical"],
    },
    Adjustable {
        tool: "hold_speed",
        key: "knots",
        step: 5.0,
        min: 0.0,
        max: 400.0,
        wrap: false,
        subjects: &["speed"],
    },
    Adjustable {
        tool: "set_speed",
        key: "knots",
        step: 5.0,
        min: 0.0,
        max: 400.0,
        wrap: false,
        subjects: &["speed"],
    },
    Adjustable {
        tool: "set_throttle",
        key: "percent",
        step: 5.0,
        min: 0.0,
        max: 100.0,
        wrap: false,
        subjects: &["throttle", "power"],
    },
    Adjustable {
        tool: "set_mixture",
        key: "percent",
        step: 5.0,
        min: 0.0,
        max: 100.0,
        wrap: false,
        subjects: &["mixture"],
    },
];

/// Direction words that also IMPLY a subject, so "faster" right after a
/// heading change still adjusts the speed, not the newest anchor.
const IMPLIES_SPEED: &[&str] = &["faster", "slower", "slow", "fast"];
const IMPLIES_ALTITUDE: &[&str] = &["higher", "lower"];

const INCREASE: &[&str] = &[
    "more", "higher", "faster", "increase", "raise", "up", "another", "add", "fast",
];
const DECREASE: &[&str] = &[
    "less", "lower", "slower", "reduce", "decrease", "down", "ease", "slow",
];
const LEFT: &[&str] = &["left", "port"];
const RIGHT: &[&str] = &["right", "starboard"];
/// "a lot more" doubles the default step; "a bit more" is just the step.
const BIG: &[&str] = &["lot", "much", "way"];
const SMALL: &[&str] = &["bit", "little", "touch", "tad", "slightly"];
/// Glue allowed inside an adjustment without breaking the closed vocabulary.
const ADJUST_FILLER: &[&str] = &[
    "a", "the", "please", "of", "by", "and", "just", "degrees", "degree", "knots", "knot", "feet",
    "foot", "percent", "fpm", "for", "me", "us", "it", "that", "them", "come", "bring", "make",
    "give",
];

const REPEAT_VOCAB: &[&str] = &[
    "do", "that", "it", "them", "again", "same", "once", "more", "one", "time", "as", "before",
    "last", "thing", "please",
];

const ONOFF_VOCAB: &[&str] = &[
    "turn", "switch", "put", "flip", "it", "that", "them", "those", "back", "on", "off", "the",
    "please", "all",
];

/// One executed tool call, newest last.
struct Recent {
    name: String,
    args: Value,
    at: Instant,
}

/// What a back-reference resolved to: concrete calls for the normal tool
/// path, or an honest spoken line when the shape matched but the anchor is
/// missing ("More what?") — better than a negative tone for words that were
/// clearly addressed to us.
pub enum Resolution {
    Calls(Vec<ToolCall>),
    Say(String),
}

#[derive(Default)]
pub struct Dialogue {
    recent: Vec<Recent>,
}

impl Dialogue {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record an executed call (call AFTER a successful execute, so refused
    /// gates still produce an anchor — repeating a refusal is harmless — but
    /// transport failures don't).
    pub fn record(&mut self, name: &str, args: &Value) {
        self.recent.push(Recent {
            name: name.to_string(),
            args: args.clone(),
            at: Instant::now(),
        });
        if self.recent.len() > STACK_MAX {
            self.recent.remove(0);
        }
    }

    /// The airframe changed: its conversation dies with it.
    pub fn clear(&mut self) {
        self.recent.clear();
    }

    /// Resolve a normalized utterance against the stack. `None` means "not a
    /// back-reference" — the caller's miss handling proceeds unchanged.
    pub fn resolve(&self, t: &str, registry: &Registry) -> Option<Resolution> {
        let tokens: Vec<String> = t
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|w| !w.is_empty())
            .map(|w| w.to_ascii_lowercase())
            .collect();
        if tokens.is_empty() || tokens.len() > 6 {
            return None;
        }
        // Order matters: "one more time" is a repeat, not a "more" delta.
        if let Some(r) = self.resolve_repeat(&tokens, registry) {
            return Some(r);
        }
        if let Some(r) = self.resolve_onoff(&tokens, registry) {
            return Some(r);
        }
        self.resolve_delta(&tokens, registry)
    }

    /// Newest fresh anchor passing `pick`.
    fn newest(&self, pick: impl Fn(&Recent) -> bool) -> Option<&Recent> {
        self.recent
            .iter()
            .rev()
            .find(|r| r.at.elapsed() < ANCHOR_TTL && pick(r))
    }

    /// "Do that again" / "same again" / "once more" / "one more time".
    fn resolve_repeat(&self, tokens: &[String], registry: &Registry) -> Option<Resolution> {
        let within = tokens.iter().all(|w| REPEAT_VOCAB.contains(&w.as_str()));
        let has = |w: &str| tokens.iter().any(|x| x == w);
        let pair = |a: &str, b: &str| tokens.windows(2).any(|p| p[0] == a && p[1] == b);
        // "say again"/"repeat that" belong to the conversational layer
        // (repeat the READBACK); this shape must carry a doing word or a
        // repeat idiom, never match bare "again?" chatter.
        let shape = pair("that", "again")
            || pair("it", "again")
            || pair("same", "again")
            || pair("once", "more")
            || (has("one") && has("more") && has("time"))
            || (has("do") && has("again"))
            || (has("same") && has("thing"));
        if !(within && shape) {
            return None;
        }
        let Some(last) = self.newest(|_| true) else {
            return Some(Resolution::Say("Nothing recent to do again.".to_string()));
        };
        if REPEAT_DENY.contains(&last.name.as_str()) || registry.get(&last.name).is_none() {
            return Some(Resolution::Say(
                "Say the full command for that one and I'll do it.".to_string(),
            ));
        }
        Some(Resolution::Calls(vec![ToolCall {
            name: last.name.clone(),
            arguments: last.args.clone(),
        }]))
    }

    /// "Turn it off" / "switch them back on" — re-issue the last boolean
    /// `on` call with the asked state, every other argument preserved (the
    /// lights call keeps WHICH light it was).
    fn resolve_onoff(&self, tokens: &[String], registry: &Registry) -> Option<Resolution> {
        let within = tokens.iter().all(|w| ONOFF_VOCAB.contains(&w.as_str()));
        let has = |w: &str| tokens.iter().any(|x| x == w);
        let on = has("on");
        let off = has("off");
        let pronoun = has("it") || has("that") || has("them") || has("those");
        if !(within && pronoun && (on ^ off)) {
            return None;
        }
        let Some(last) = self.newest(|r| r.args.get("on").and_then(Value::as_bool).is_some())
        else {
            return Some(Resolution::Say(format!(
                "Turn what {}? Give me the system.",
                if on { "on" } else { "off" }
            )));
        };
        if registry.get(&last.name).is_none() {
            return None;
        }
        let mut args = last.args.clone();
        args["on"] = Value::Bool(on);
        Some(Resolution::Calls(vec![ToolCall {
            name: last.name.clone(),
            arguments: args,
        }]))
    }

    /// "A bit more" / "ten knots less" / "come left 10" / "reduce by 20" —
    /// a nudge on the last commanded value of an adjustable tool.
    fn resolve_delta(&self, tokens: &[String], registry: &Registry) -> Option<Resolution> {
        let mut number: Option<f64> = None;
        let mut sign = 0i8;
        let mut lateral = 0i8; // heading only: left/right
        let mut big = false;
        let mut named: Vec<&Adjustable> = Vec::new();
        for w in tokens {
            let w = w.as_str();
            if let Some((v, mult)) = word_value(w) {
                if mult {
                    return None; // bare "hundred"/"thousand" — not our shape
                }
                number = Some(v as f64);
                continue;
            }
            if let Ok(v) = w.parse::<f64>() {
                number = Some(v);
                continue;
            }
            let subj: Vec<&Adjustable> = ADJUSTABLE
                .iter()
                .filter(|a| a.subjects.contains(&w))
                .collect();
            if !subj.is_empty() {
                named.extend(subj);
                // A subject word may double as glue elsewhere; it's consumed.
                continue;
            }
            if LEFT.contains(&w) {
                lateral = -1;
            } else if RIGHT.contains(&w) {
                lateral = 1;
            } else if INCREASE.contains(&w) {
                if sign == -1 {
                    return None; // "more less" — garbage
                }
                sign = 1;
            } else if DECREASE.contains(&w) {
                if sign == 1 {
                    // "slow down", "ease down" — two decrease-family words is
                    // fine, but a genuine conflict (increase then decrease)
                    // isn't. INCREASE/DECREASE overlap is resolved by listing
                    // the ambiguous pair ("fast"/"slow") in their own family
                    // only, so reaching here means real conflict.
                    return None;
                }
                sign = -1;
            } else if BIG.contains(&w) {
                big = true;
            } else if SMALL.contains(&w) || ADJUST_FILLER.contains(&w) {
                // glue
            } else {
                return None; // one word outside the shape: not an adjustment
            }
        }
        // Direction words that imply their subject, so "faster" adjusts the
        // speed even when the newest anchor is something else.
        if named.is_empty() {
            if tokens.iter().any(|w| IMPLIES_SPEED.contains(&w.as_str())) {
                named.extend(ADJUSTABLE.iter().filter(|a| a.key == "knots"));
            } else if tokens
                .iter()
                .any(|w| IMPLIES_ALTITUDE.contains(&w.as_str()))
            {
                named.extend(ADJUSTABLE.iter().filter(|a| a.tool == "set_altitude"));
            }
        }
        let subject_named = !named.is_empty();
        // A turn needs left/right; everything else needs more/less. Neither
        // spoken is a bare number ("ten more" has a sign; "ten" alone is the
        // bare-value follow-up path, not ours).
        if sign == 0 && lateral == 0 {
            return None;
        }
        let candidates: Vec<&Adjustable> = if !named.is_empty() {
            named
        } else if lateral != 0 {
            ADJUSTABLE.iter().filter(|a| a.wrap).collect()
        } else {
            // Generic more/less never means a turn direction.
            ADJUSTABLE.iter().filter(|a| !a.wrap).collect()
        };
        // The compass row only moves laterally; a named "more heading" stays
        // unresolved rather than guessing a direction.
        let candidates: Vec<&Adjustable> = candidates
            .into_iter()
            .filter(|a| if a.wrap { lateral != 0 } else { sign != 0 })
            .collect();
        if candidates.is_empty() {
            return None;
        }
        let anchor = self.newest(|r| {
            candidates
                .iter()
                .any(|a| a.tool == r.name && r.args.get(a.key).and_then(Value::as_f64).is_some())
        });
        let Some(anchor) = anchor else {
            // The shape was clearly an adjustment addressed to us; a named
            // (or implied — "faster") subject earns an honest ask, while an
            // anonymous "a bit more" with nothing fresh on the stack falls
            // through to the normal miss handling.
            return if subject_named {
                let what = candidates[0].subjects.first().copied().unwrap_or("that");
                Some(Resolution::Say(format!(
                    "I haven't set a {what} to adjust. Give me the number you want."
                )))
            } else {
                None
            };
        };
        let a = candidates
            .iter()
            .find(|a| a.tool == anchor.name)
            .expect("anchor picked from candidates");
        if registry.get(&anchor.name).is_none() {
            return None;
        }
        let old = anchor.args.get(a.key).and_then(Value::as_f64)?;
        let dir = if a.wrap {
            f64::from(lateral)
        } else {
            f64::from(sign)
        };
        let amount = match number {
            Some(n) if n > 0.0 => n,
            _ => a.step * if big { 2.0 } else { 1.0 },
        };
        let mut new = old + dir * amount;
        if a.wrap {
            new = new.rem_euclid(360.0);
        }
        new = new.clamp(a.min, a.max);
        let mut args = anchor.args.clone();
        args[a.key] = Value::from(new.round() as i64);
        Some(Resolution::Calls(vec![ToolCall {
            name: anchor.name.clone(),
            arguments: args,
        }]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn registry() -> Registry {
        let profile = crate::aircraft::load("cessna_172sp").expect("cessna profile");
        Registry::for_aircraft(&profile)
    }

    fn calls(r: Option<Resolution>) -> (String, Value) {
        match r {
            Some(Resolution::Calls(mut v)) => {
                let c = v.remove(0);
                (c.name, c.arguments)
            }
            Some(Resolution::Say(s)) => panic!("expected calls, got say: {s}"),
            None => panic!("expected calls, got none"),
        }
    }

    fn say(r: Option<Resolution>) -> String {
        match r {
            Some(Resolution::Say(s)) => s,
            Some(Resolution::Calls(v)) => panic!("expected say, got call {}", v[0].name),
            None => panic!("expected say, got none"),
        }
    }

    #[test]
    fn a_nudge_moves_the_last_commanded_value_by_the_step() {
        let reg = registry();
        let mut d = Dialogue::new();
        d.record("hold_speed", &json!({"knots": 70}));
        let (name, args) = calls(d.resolve("a bit faster", &reg));
        assert_eq!(
            (name.as_str(), args["knots"].as_i64()),
            ("hold_speed", Some(75))
        );
        let (_, args) = calls(d.resolve("a bit slower", &reg));
        assert_eq!(args["knots"].as_i64(), Some(65));
        // "a lot" doubles the step; an explicit number beats it entirely.
        let (_, args) = calls(d.resolve("much faster", &reg));
        assert_eq!(args["knots"].as_i64(), Some(80));
        let (_, args) = calls(d.resolve("10 knots less", &reg));
        assert_eq!(args["knots"].as_i64(), Some(60));
        let (_, args) = calls(d.resolve("reduce by 20", &reg));
        assert_eq!(args["knots"].as_i64(), Some(50));
    }

    #[test]
    fn a_turn_is_left_or_right_never_more_or_less() {
        let reg = registry();
        let mut d = Dialogue::new();
        d.record("set_heading", &json!({"heading": 350}));
        let (name, args) = calls(d.resolve("come left 10", &reg));
        assert_eq!(
            (name.as_str(), args["heading"].as_i64()),
            ("set_heading", Some(340))
        );
        // Wraps the compass, never clamps at it.
        let (_, args) = calls(d.resolve("right a bit", &reg));
        assert_eq!(args["heading"].as_i64(), Some(0));
        // Generic "a bit more" must NOT guess a turn direction: with only a
        // heading on the stack there is nothing it may adjust.
        assert!(d.resolve("a bit more", &reg).is_none());
    }

    #[test]
    fn direction_words_imply_their_subject_over_recency() {
        let reg = registry();
        let mut d = Dialogue::new();
        d.record("hold_speed", &json!({"knots": 70}));
        d.record("set_heading", &json!({"heading": 90}));
        // Heading is newer, but "faster" can only mean the speed.
        let (name, args) = calls(d.resolve("faster", &reg));
        assert_eq!(
            (name.as_str(), args["knots"].as_i64()),
            ("hold_speed", Some(75))
        );
    }

    #[test]
    fn one_word_outside_the_shape_never_resolves() {
        let reg = registry();
        let mut d = Dialogue::new();
        d.record("hold_speed", &json!({"knots": 70}));
        // The closed vocabulary is the safety: chatter that happens to carry
        // "more" must not become a speed change.
        for t in [
            "more coffee",
            "less turbulence would be nice",
            "we need more fuel at the next stop",
            "the more i fly the more i like it",
        ] {
            assert!(d.resolve(t, &reg).is_none(), "{t:?} resolved");
        }
    }

    #[test]
    fn a_named_subject_with_no_anchor_asks_for_the_number() {
        let reg = registry();
        let d = Dialogue::new();
        let line = say(d.resolve("a bit more power", &reg));
        assert!(line.contains("haven't set"), "{line}");
        // Anonymous "a bit more" with an empty stack is a plain miss — the
        // normal miss handling (tone, log) applies, no spoken guess.
        assert!(d.resolve("a bit more", &reg).is_none());
    }

    #[test]
    fn again_repeats_a_switch_but_never_an_automation() {
        let reg = registry();
        let mut d = Dialogue::new();
        d.record("set_flaps", &json!({"notch": "10"}));
        let (name, args) = calls(d.resolve("do that again", &reg));
        assert_eq!(
            (name.as_str(), args["notch"].as_str()),
            ("set_flaps", Some("10"))
        );
        // An automation arm is never repeated from a vague back-reference.
        d.record("full_auto", &json!({}));
        let line = say(d.resolve("do that again", &reg));
        assert!(line.contains("full command"), "{line}");
        // Empty stack: honest, not silent — the words were addressed to us.
        let empty = Dialogue::new();
        let line = say(empty.resolve("do that again", &reg));
        assert!(line.contains("Nothing recent"), "{line}");
    }

    #[test]
    fn turn_it_off_reissues_the_last_boolean_call_preserving_its_target() {
        let reg = registry();
        let mut d = Dialogue::new();
        d.record("set_lights", &json!({"light": "landing", "on": true}));
        let (name, args) = calls(d.resolve("turn it off", &reg));
        assert_eq!(name, "set_lights");
        assert_eq!(args["light"].as_str(), Some("landing"));
        assert_eq!(args["on"].as_bool(), Some(false));
        let (_, args) = calls(d.resolve("switch them back on", &reg));
        assert_eq!(args["on"].as_bool(), Some(true));
        // No boolean call on the stack: ask, don't guess.
        let empty = Dialogue::new();
        let line = say(empty.resolve("turn it off", &reg));
        assert!(line.contains("Turn what"), "{line}");
    }

    #[test]
    fn stale_anchors_expire_and_a_swap_clears_the_stack() {
        let reg = registry();
        let mut d = Dialogue::new();
        d.record("hold_speed", &json!({"knots": 70}));
        d.recent[0].at = Instant::now() - (ANCHOR_TTL + Duration::from_secs(1));
        // A two-minute-old anchor is stale context, not intent: the implied
        // subject earns the honest ask, never a nudge on the expired value.
        let line = say(d.resolve("a bit faster", &reg));
        assert!(line.contains("haven't set"), "{line}");
        assert!(
            d.resolve("a bit more", &reg).is_none(),
            "anonymous nudge with only a stale anchor is a plain miss"
        );
        d.record("set_lights", &json!({"light": "taxi", "on": true}));
        d.clear();
        assert!(matches!(
            d.resolve("turn it off", &reg),
            Some(Resolution::Say(_))
        ));
    }
}
