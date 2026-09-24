//! Who the copilot is: how much they volunteer, the idiom they speak in,
//! what they call you, and the handful of social replies a person in the
//! right seat gives without thinking.
//!
//! Three rules keep it honest:
//!
//! - **Chattiness gates what is VOLUNTEERED, never what is owed.** An answer
//!   to the pilot, a readback and a safety call go out at every level;
//!   `quiet` only removes the landing word, the offers and the remarks.
//! - **The idiom follows the voice.** One person does not change accent at
//!   a border, so the dialect is the copilot voice's own (an `en_GB` model
//!   says "cheers", an `en_US` one "you bet"), unless the pilot picks one.
//! - **Small talk is a CLOSED set.** The whole utterance, less a few address
//!   words, must be one of the phrases here; anything else is not small talk
//!   and routes on ("thanks, gear up" is still gear up). Nothing is improvised.

use crate::readback::one_of;

/// How much the copilot volunteers.
#[derive(Clone, Copy, PartialEq, Eq, Debug, PartialOrd, Ord)]
pub enum Chatter {
    Quiet,
    Normal,
    Chatty,
}

impl Chatter {
    pub fn parse(s: &str) -> Chatter {
        match s.trim().to_ascii_lowercase().as_str() {
            "quiet" | "terse" | "off" => Chatter::Quiet,
            "chatty" | "more" => Chatter::Chatty,
            _ => Chatter::Normal,
        }
    }
    pub fn as_str(self) -> &'static str {
        match self {
            Chatter::Quiet => "quiet",
            Chatter::Normal => "normal",
            Chatter::Chatty => "chatty",
        }
    }
}

/// The copilot's idiom.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Dialect {
    British,
    American,
}

impl Dialect {
    /// `auto` (or anything unknown) follows `voice`, the copilot voice's
    /// model spec or stem: British for an `en_GB` model, American otherwise.
    pub fn resolve(pick: &str, voice: &str) -> Dialect {
        match pick.trim().to_ascii_lowercase().as_str() {
            "british" | "uk" | "gb" => Dialect::British,
            "american" | "us" => Dialect::American,
            _ if voice.contains("en_GB") => Dialect::British,
            _ => Dialect::American,
        }
    }
}

/// The persona as the rapport needs it, one snapshot per tick.
#[derive(Clone, Debug)]
pub struct Persona {
    pub chatter: Chatter,
    pub dialect: Dialect,
    pub pilot_name: Option<String>,
}

impl Persona {
    pub fn live() -> Persona {
        Persona {
            chatter: chatter(),
            dialect: dialect(),
            pilot_name: pilot_name(),
        }
    }
}

/// The live chattiness setting.
pub fn chatter() -> Chatter {
    Chatter::parse(&crate::settings::live().crew_chatter)
}

/// The live dialect: the setting, else the voice the speaker thread actually
/// built (after any fallback), else the configured model.
pub fn dialect() -> Dialect {
    let cfg = crate::settings::live();
    let voice = crate::voice::pilot_engine()
        .map(|(_, stem)| stem)
        .unwrap_or_else(|| cfg.piper_voice.clone());
    Dialect::resolve(&cfg.crew_dialect, &voice)
}

/// The pilot's name, when one is set.
pub fn pilot_name() -> Option<String> {
    let n = crate::settings::live().pilot_name.trim().to_string();
    (!n.is_empty()).then_some(n)
}

fn copilot_name() -> Option<String> {
    let n = crate::settings::live().copilot_name.trim().to_string();
    (!n.is_empty()).then_some(n)
}

/// "Stand by": what fills a turn that is taking a while (`tempo::Turn`).
pub fn standby_line(d: Dialect) -> &'static str {
    match d {
        Dialect::British => one_of(&["Stand by.", "One moment.", "Bear with me."]),
        Dialect::American => one_of(&["Stand by.", "One sec.", "Working on it."]),
    }
}

/// What the social reply needs to know about the flight.
#[derive(Clone, Copy, Debug, Default)]
pub struct Social {
    /// Who flew the last landing, when there was one this session.
    pub last_landing_by_automation: Option<bool>,
}

/// The pilot's words, reduced for the closed match: lowercase, letters and
/// digits only, and the address words ("copilot", "mate", the copilot's own
/// name) peeled off either end, so "thanks mate" and "copilot, thanks" are
/// both "thanks".
fn reduce(text: &str, own_name: Option<&str>) -> String {
    let lower = text.to_lowercase().replace(['\'', '\u{2019}'], "");
    let mut words: Vec<&str> = lower
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    let own = own_name.map(|n| n.to_lowercase());
    let address = |w: &str| {
        matches!(
            w,
            "copilot" | "mate" | "buddy" | "pal" | "man" | "dude" | "chief" | "captain"
        ) || own.as_deref() == Some(w)
    };
    while words.first().is_some_and(|w| address(w)) {
        words.remove(0);
    }
    while words.last().is_some_and(|w| address(w)) {
        words.pop();
    }
    words.join(" ")
}

/// The reply to a closed piece of small talk, or `None` when the words are
/// not small talk. Pure: the variation comes from `one_of`, the facts from
/// the arguments.
pub fn social_reply(
    text: &str,
    d: Dialect,
    pilot: Option<&str>,
    own_name: Option<&str>,
    s: Social,
) -> Option<String> {
    let t = reduce(text, own_name);
    let brit = d == Dialect::British;
    let with_name = |line: &str| match pilot {
        Some(n) => format!("{}, {n}.", line.trim_end_matches('.')),
        None => line.to_string(),
    };
    let line: String = match t.as_str() {
        "thanks"
        | "thank you"
        | "thanks a lot"
        | "thank you very much"
        | "thanks very much"
        | "cheers"
        | "ta"
        | "much appreciated"
        | "appreciate it" => if brit {
            one_of(&["Cheers.", "No worries.", "Any time."])
        } else {
            one_of(&["You bet.", "No problem.", "Any time."])
        }
        .to_string(),
        "good job" | "nice job" | "nice work" | "good work" | "well done" | "nice one"
        | "great job" | "great work" | "good stuff" => if brit {
            one_of(&["Cheers.", "Thanks. Team effort."])
        } else {
            one_of(&["Thanks.", "Appreciate it.", "Thanks. Team effort."])
        }
        .to_string(),
        "nice landing" | "good landing" | "great landing" | "smooth landing" | "lovely landing" => {
            match s.last_landing_by_automation {
                Some(false) => if brit {
                    "That one was all yours."
                } else {
                    "That was all you."
                }
                .to_string(),
                Some(true) => {
                    one_of(&["Thanks. It felt good.", "Thanks. I'll take that."]).to_string()
                }
                None => "We haven't landed yet.".to_string(),
            }
        }
        "good morning" | "morning" => with_name("Morning."),
        "good afternoon" | "afternoon" => with_name("Afternoon."),
        "good evening" | "evening" => with_name("Evening."),
        "hello" | "hi" | "hey" | "hi there" | "hello there" | "hey there" => {
            with_name(if brit { "Hello." } else { "Hey." })
        }
        "how are you" | "how are you doing" | "how you doing" | "hows it going"
        | "how is it going" | "you alright" | "you ok" | "you okay" | "all good" => if brit {
            one_of(&["Not bad, thanks. All steady here.", "All good, thanks."])
        } else {
            one_of(&["Doing good, thanks. All steady here.", "All good, thanks."])
        }
        .to_string(),
        "you there" | "are you there" | "anyone there" | "are you with me" | "you with me" => {
            one_of(&["Here.", "Go ahead.", "With you."]).to_string()
        }
        "sorry" | "my bad" | "my mistake" | "oops" => {
            one_of(&["No problem.", "All good."]).to_string()
        }
        "whats your name" | "what is your name" | "who are you" => match own_name {
            Some(n) => format!("{n}. Your first officer today."),
            None => "Just your first officer. No name on the roster yet.".to_string(),
        },
        "see you" | "see you later" | "bye" | "goodbye" | "good night" | "goodnight"
        | "thats all" | "thats it for today" => if brit {
            "Cheers. See you next time."
        } else {
            "Take care. See you next time."
        }
        .to_string(),
        _ => return None,
    };
    Some(line)
}

/// The live form: the closed match with the settings and the flight's facts.
pub fn reply_now(text: &str) -> Option<String> {
    social_reply(
        text,
        dialect(),
        pilot_name().as_deref(),
        copilot_name().as_deref(),
        crate::rapport::social_facts(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    const NO: Social = Social {
        last_landing_by_automation: None,
    };

    #[test]
    fn the_idiom_follows_the_voice_unless_picked() {
        assert_eq!(
            Dialect::resolve("auto", "models/en_GB-alan-medium.onnx"),
            Dialect::British
        );
        assert_eq!(Dialect::resolve("", "en_US-amy-medium"), Dialect::American);
        assert_eq!(
            Dialect::resolve("british", "en_US-amy-medium"),
            Dialect::British
        );
        assert_eq!(Chatter::parse(""), Chatter::Normal);
        assert_eq!(Chatter::parse("Quiet"), Chatter::Quiet);
        assert!(Chatter::Chatty > Chatter::Normal);
    }

    #[test]
    fn thanks_gets_an_answer_in_the_copilots_own_idiom() {
        for _ in 0..20 {
            let gb = social_reply("Thanks mate.", Dialect::British, None, None, NO).unwrap();
            assert!(
                ["Cheers.", "No worries.", "Any time."].contains(&gb.as_str()),
                "{gb}"
            );
            let us = social_reply("copilot, thank you", Dialect::American, None, None, NO).unwrap();
            assert!(
                ["You bet.", "No problem.", "Any time."].contains(&us.as_str()),
                "{us}"
            );
        }
    }

    #[test]
    fn small_talk_is_a_closed_set() {
        // Anything more than the phrase is not small talk: it routes on.
        for t in [
            "thanks, gear up",
            "good job on the flaps, now flaps ten",
            "hello tower",
            "how are you doing on fuel",
            "morning checklist",
            "sorry, flaps twenty",
        ] {
            assert_eq!(
                social_reply(t, Dialect::British, None, None, NO),
                None,
                "{t}"
            );
        }
    }

    #[test]
    fn a_landing_compliment_knows_who_landed() {
        let pilot = Social {
            last_landing_by_automation: Some(false),
        };
        assert_eq!(
            social_reply("nice landing", Dialect::American, None, None, pilot).as_deref(),
            Some("That was all you.")
        );
        let none = social_reply("nice landing", Dialect::American, None, None, NO).unwrap();
        assert!(none.contains("haven't landed"), "{none}");
    }

    #[test]
    fn greetings_use_the_pilots_name_and_the_own_name_is_an_address_word() {
        assert_eq!(
            social_reply("good morning", Dialect::British, Some("Sam"), None, NO).as_deref(),
            Some("Morning, Sam.")
        );
        assert_eq!(
            social_reply("thanks Alex", Dialect::British, None, Some("Alex"), NO).is_some(),
            true
        );
        assert_eq!(
            social_reply("what's your name", Dialect::British, None, Some("Alex"), NO).as_deref(),
            Some("Alex. Your first officer today.")
        );
    }

    #[test]
    fn no_reply_carries_a_dash() {
        let phrases = [
            "thanks",
            "good job",
            "nice landing",
            "morning",
            "hello",
            "how are you",
            "you there",
            "sorry",
            "whats your name",
            "bye",
        ];
        for d in [Dialect::British, Dialect::American] {
            for p in phrases {
                for _ in 0..10 {
                    let r = social_reply(p, d, Some("Sam"), None, NO).unwrap();
                    assert!(!r.contains('\u{2014}') && !r.contains('\u{2013}'), "{r}");
                }
            }
        }
        for _ in 0..10 {
            for d in [Dialect::British, Dialect::American] {
                assert!(standby_line(d).ends_with('.'));
            }
        }
    }
}
