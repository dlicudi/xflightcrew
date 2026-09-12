//! Deterministic, read-only crew conversation. Separate from `dialogue`, whose
//! short-lived anchors can select tools. This module cannot return a tool call,
//! touch the simulator, or interpret a conversational reply as authorization.
//!
//! Composable closed question shapes, pending questions, and expiring evidence
//! make follow-ups useful without guessing. Historical reports are never current
//! readings, and a tool returning a line is NOT proof it performed an action
//! (safety refusals also return `Ok`).

use std::collections::VecDeque;
use std::time::{Duration, Instant};

const TOPIC_TTL: Duration = Duration::from_secs(300);
const CLARIFICATION_TTL: Duration = Duration::from_secs(30);
const HISTORY_MAX: usize = 12;

#[derive(Debug, PartialEq)]
pub enum Response {
    Say(String),
    /// Only canonical, question-shaped state reads; never fed to the grammar.
    Read(&'static str),
    ReadMany(Vec<&'static str>),
    Reference(&'static str),
    Checklist(String),
    /// Refresh the existing crew automation status, without inferring AP state.
    Status,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum Detail {
    Brief,
    #[default]
    Normal,
    Full,
}

#[derive(Clone, Debug, PartialEq)]
enum Evidence {
    Readings {
        queries: Vec<&'static str>,
        lines: Vec<String>,
    },
    Observation {
        query: String,
        line: String,
    },
    /// `failed` means execution errored. `false` only means a report was returned,
    /// NOT success: the line may describe a refusal or an asynchronous start.
    Action {
        request: String,
        line: String,
        failed: bool,
    },
    Knowledge {
        short: String,
        explanation: String,
    },
    Status,
    Preview,
}

struct Preview {
    title: String,
    items: Vec<String>,
    index: usize,
    at: Instant,
}

#[derive(Default)]
pub struct Conversation {
    topic: Option<(Evidence, Instant)>,
    pending_speed: Option<Instant>,
    detail: Detail,
    history: VecDeque<(Evidence, Instant)>,
    pending_reading: Option<Instant>,
    checklist_choices: Option<(Vec<String>, Instant)>,
    preview: Option<Preview>,
}

fn words(text: &str) -> String {
    text.to_ascii_lowercase()
        .chars()
        .filter(|c| !matches!(c, '\'' | '’'))
        .map(|c| if c.is_ascii_alphanumeric() { c } else { ' ' })
        .collect::<String>()
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
}

impl Conversation {
    /// Call for every new intercom utterance, including commands handled earlier
    /// in the loop. Unrelated speech retires the topic rather than making a later
    /// "why?" explain the wrong event. Radio calls never enter this state machine.
    pub fn respond(&mut self, text: &str, now: Instant) -> Option<Response> {
        self.history
            .retain(|(_, at)| now.duration_since(*at) < TOPIC_TTL);
        if self
            .preview
            .as_ref()
            .is_some_and(|p| now.duration_since(p.at) >= TOPIC_TTL)
        {
            self.preview = None;
        }
        if self
            .topic
            .as_ref()
            .is_some_and(|(_, at)| now.duration_since(*at) >= TOPIC_TTL)
        {
            self.topic = None;
        }
        if self
            .pending_speed
            .is_some_and(|at| now.duration_since(at) >= CLARIFICATION_TTL)
        {
            self.pending_speed = None;
        }
        let t = words(text);
        let t = t.strip_prefix("please ").unwrap_or(&t);
        let t = t.strip_suffix(" from now on").unwrap_or(t);
        let t = t.strip_suffix(" please").unwrap_or(t);
        // Conversational wrappers compose with closed question subjects. Never
        // remove command verbs, numbers, negation, or unknown words.
        let t = t
            .strip_prefix("could you ")
            .or_else(|| t.strip_prefix("can you "))
            .or_else(|| t.strip_prefix("would you "))
            .unwrap_or(t);
        let t = t.strip_prefix("please ").unwrap_or(t);
        let preference = match t {
            "keep it brief" | "keep the explanations shorter" | "short answers" => {
                Some(Detail::Brief)
            }
            "more detailed answers" | "give me detailed explanations" => Some(Detail::Full),
            "normal answers" | "normal explanations" => Some(Detail::Normal),
            _ => None,
        };
        if let Some(detail) = preference {
            self.detail = detail;
            return Some(Response::Say(
                match detail {
                    Detail::Brief => "I'll keep explanations brief. Safety callouts stay complete.",
                    Detail::Full => "I'll include the available explanation when you ask.",
                    Detail::Normal => "Back to normal explanations.",
                }
                .into(),
            ));
        }
        if matches!(
            t,
            "thanks" | "thank you" | "okay" | "ok" | "understood" | "got it"
        ) {
            return Some(Response::Say("Standing by.".into()));
        }
        // The existing speech-replay handler owns these. Keep the evidence that
        // explains the replayed answer, without adding a second replay mechanism.
        if matches!(t, "say again" | "repeat that" | "what did you say") {
            return None;
        }
        if matches!(
            t,
            "never mind" | "forget the question" | "leave that question"
        ) {
            self.clear();
            return Some(Response::Say("Question dropped.".into()));
        }
        if self
            .pending_reading
            .take()
            .is_some_and(|at| now.duration_since(at) < CLARIFICATION_TTL)
        {
            if let Some(q) = subject_query(t) {
                return Some(Response::Read(q));
            }
        }
        if self.pending_speed.take().is_some() {
            if let Some(query) = speed_choice(t) {
                return Some(Response::Read(query));
            }
            // A reply outside the closed set is a new turn, not a guessed choice.
        }
        if let Some((choices, at)) = self.checklist_choices.take() {
            if now.duration_since(at) < CLARIFICATION_TTL {
                if let Some(choice) = choices.iter().find(|c| {
                    words(c) == t.trim_start_matches("the ").trim_end_matches(" checklist")
                }) {
                    return Some(Response::Checklist(choice.clone()));
                }
            }
        }
        if let Some(request) = preview_request(t) {
            self.topic = None;
            self.pending_speed = None;
            self.preview = None;
            return Some(Response::Checklist(request.into()));
        }
        if matches!(
            t,
            "resume the preview" | "resume checklist preview" | "back to the checklist preview"
        ) {
            if self.preview.is_some() {
                self.topic = Some((Evidence::Preview, now));
                return Some(Response::Say(self.preview_line()));
            }
            return Some(Response::Say(
                "No checklist preview to resume. Name the checklist you want to discuss.".into(),
            ));
        }
        if matches!(self.topic, Some((Evidence::Preview, _))) {
            let step = match t {
                "next" | "next item" | "go on" | "continue the preview" => Some(1),
                "previous item" | "go back one item" => Some(-1),
                "repeat this item" | "which item are we on" => Some(0),
                _ => None,
            };
            if let Some(step) = step {
                if let Some(p) = self.preview.as_mut() {
                    if step > 0 && p.index + 1 >= p.items.len() {
                        return Some(Response::Say("That's the last preview item. No checklist items have been performed or verified by this preview.".into()));
                    }
                    p.index = p.index.saturating_add_signed(step);
                    p.at = now;
                    self.topic = Some((Evidence::Preview, now));
                    return Some(Response::Say(self.preview_line()));
                }
            }
            if matches!(t, "pause the preview" | "pause checklist preview") {
                self.topic = None;
                return Some(Response::Say(
                    "Preview paused. Say resume the preview to return to this item.".into(),
                ));
            }
        }
        if matches!(
            t,
            "what have we discussed" | "recap our conversation" | "what did we talk about"
        ) {
            let lines: Vec<_> = self
                .history
                .iter()
                .rev()
                .take(4)
                .map(|(e, _)| evidence_line(e))
                .collect();
            return Some(Response::Say(if lines.is_empty() {
                "No recent conversation to recap.".into()
            } else {
                format!("Recent reports, not fresh readings: {}", lines.join("; "))
            }));
        }
        if matches!(
            t,
            "what was my last request" | "what did i last ask you to do"
        ) {
            return Some(Response::Say(
                self.history
                    .iter()
                    .rev()
                    .find_map(|(e, _)| match e {
                        Evidence::Action { request, line, .. } => {
                            Some(format!("Your request was {request}. My report was: {line}"))
                        }
                        _ => None,
                    })
                    .unwrap_or_else(|| "No recent action request recorded.".into()),
            ));
        }
        if let Some(subject) = t
            .strip_prefix("back to ")
            .or_else(|| t.strip_prefix("go back to "))
        {
            if let Some(query) = subject_query(subject) {
                if let Some(record) = self.history.iter().rev().find(|(e, _)| {
                    matches!(e,
                    Evidence::Observation { query: prior, .. } if state_query(prior) == Some(query))
                }) {
                    self.topic = Some(record.clone());
                    return Some(self.explain());
                }
                return Some(Response::Say(
                    "We haven't discussed that reading recently. Ask me to check it.".into(),
                ));
            }
        }
        if matches!(
            t,
            "what speed" | "which speed" | "what is our speed" | "whats our speed"
        ) {
            self.topic = None;
            self.pending_speed = Some(now);
            return Some(Response::Say(
                "Airspeed, groundspeed, or vertical speed?".into(),
            ));
        }
        if matches!(
            t,
            "what are you doing"
                | "what are you managing"
                | "whats the plan now"
                | "why are we waiting"
                | "what are we waiting for"
        ) {
            self.topic = Some((Evidence::Status, now));
            self.remember();
            return Some(Response::Status);
        }
        if matches!(
            t,
            "why"
                | "why is that"
                | "why that speed"
                | "why that number"
                | "why did you do that"
                | "explain that"
                | "tell me more"
                | "explain that further"
                | "where did that come from"
                | "how do you know"
                | "what did you just do"
        ) {
            return Some(self.explain());
        }
        if matches!(
            t,
            "and now" | "what about now" | "check that again" | "is that still the same"
        ) {
            return Some(match self.topic.as_ref().map(|(e, _)| e) {
                Some(Evidence::Readings { queries, .. }) => Response::ReadMany(queries.clone()),
                Some(Evidence::Observation { query, .. }) => match state_query(query) {
                    Some(q) => Response::Read(q),
                    None => Response::Say("Name the reading you want me to check again.".into()),
                },
                Some(Evidence::Status) => Response::Status,
                _ => {
                    self.pending_reading = Some(now);
                    Response::Say("Which reading should I check?".into())
                }
            });
        }
        if let Some(queries) = question_queries(t) {
            self.topic = None;
            return Some(if queries.len() == 1 {
                Response::Read(queries[0])
            } else {
                Response::ReadMany(queries)
            });
        }
        // Elliptical questions are READS, even for a system that also has a tool.
        if let Some(subject) = t
            .strip_prefix("and ")
            .or_else(|| t.strip_prefix("what about "))
        {
            if self.topic.is_some() {
                if let Some(q) = subject_query(subject) {
                    return Some(Response::Read(q));
                }
                if let Some(q) = reference_query(subject) {
                    return Some(Response::Reference(q));
                }
            }
        }
        if let Some((short, explanation)) = knowledge(t) {
            self.topic = Some((
                Evidence::Knowledge {
                    short: short.into(),
                    explanation: explanation.into(),
                },
                now,
            ));
            self.remember();
            return Some(Response::Say(if self.detail == Detail::Full {
                format!("{short} {explanation}")
            } else {
                short.into()
            }));
        }
        // Do not keep a previous answer behind an unrelated command or question.
        self.topic = None;
        None
    }

    fn explain(&self) -> Response {
        Response::Say(match self.topic.as_ref().map(|(e, _)| e) {
            Some(Evidence::Readings { lines, .. }) => format!("Those were the simulator reports when you asked: {}", lines.join("; ")),
            Some(Evidence::Observation { line, .. }) => format!(
                "That was my simulator report when you asked: {line}{}",
                if self.detail == Detail::Brief {
                    ""
                } else {
                    " Say check that again for a new reading."
                }
            ),
            Some(Evidence::Action {
                request,
                line,
                failed,
            }) => {
                if *failed {
                    format!("I couldn't complete your request, {request}. {line}")
                } else if self.detail == Detail::Brief {
                    format!("My report was: {line}")
                } else {
                    format!("For your request, {request}, I reported: {line}")
                }
            }
            Some(Evidence::Knowledge { short, explanation }) => {
                if self.detail == Detail::Brief {
                    short.clone()
                } else {
                    explanation.clone()
                }
            }
            Some(Evidence::Status) => return Response::Status,
            Some(Evidence::Preview) => "This is a preview of the aircraft's checklist instructions, not a report that an item is complete.".into(),
            None => "Which answer or action do you mean?".into(),
        })
    }

    /// The words of the last state question answered, while its topic is
    /// fresh: what "there" refers to in "remind me when we get there".
    pub fn recent_observation_query(&self, now: Instant) -> Option<String> {
        match &self.topic {
            Some((Evidence::Observation { query, .. }, at))
                if now.duration_since(*at) <= TOPIC_TTL =>
            {
                Some(query.clone())
            }
            _ => None,
        }
    }

    pub fn observe(&mut self, query: &str, line: &str, now: Instant) {
        self.topic = Some((
            Evidence::Observation {
                query: words(query),
                line: line.into(),
            },
            now,
        ));
        self.remember();
    }

    pub fn observe_many(&mut self, queries: Vec<&'static str>, lines: Vec<String>, now: Instant) {
        for (query, line) in queries.iter().zip(&lines) {
            self.observe(query, line, now);
        }
        self.topic = Some((Evidence::Readings { queries, lines }, now));
    }

    pub fn report(&mut self, request: &str, line: &str, failed: bool, now: Instant) {
        self.topic = Some((
            Evidence::Action {
                request: request.into(),
                line: line.into(),
                failed,
            },
            now,
        ));
        self.remember();
    }

    pub fn reference(&mut self, aircraft: &str, line: &str, now: Instant) {
        self.topic = Some((Evidence::Knowledge {
            short: line.into(),
            explanation: format!(
                "That comes from the {aircraft} aircraft profile: {line} These are reference figures, not a calculation for today's loading and conditions."
            ),
        }, now));
        self.remember();
    }

    fn remember(&mut self) {
        if let Some(record) = &self.topic {
            if self.history.back().is_some_and(|last| last.0 == record.0) {
                self.history.pop_back();
            }
            self.history.push_back(record.clone());
            while self.history.len() > HISTORY_MAX {
                self.history.pop_front();
            }
        }
    }

    pub fn choose_checklist(&mut self, choices: Vec<String>, now: Instant) -> String {
        let line = if choices.is_empty() {
            "No checklists are available for this aircraft.".into()
        } else {
            format!("Which checklist should we preview? {}.", choices.join(", "))
        };
        self.checklist_choices = (!choices.is_empty()).then_some((choices, now));
        line
    }

    pub fn start_preview(&mut self, title: String, items: Vec<String>, now: Instant) -> String {
        if items.is_empty() {
            return "That checklist has no preview items.".into();
        }
        self.preview = Some(Preview {
            title,
            items,
            index: 0,
            at: now,
        });
        self.topic = Some((Evidence::Preview, now));
        format!(
            "Preview only; I'm not performing the checklist. {} Say next item to continue.",
            self.preview_line()
        )
    }

    fn preview_line(&self) -> String {
        let Some(p) = &self.preview else {
            return "No active checklist preview.".into();
        };
        format!(
            "{} preview, item {} of {}: {}",
            p.title,
            p.index + 1,
            p.items.len(),
            p.items[p.index]
        )
    }

    /// Airframe swaps and explicit topic cancellation retire all evidence and
    /// pending replies. The pilot's speech preference survives for this session.
    pub fn clear(&mut self) {
        self.clear_topic();
        self.history.clear();
        self.preview = None;
    }

    /// A missing reading retires only the current focus. Earlier evidence stays
    /// explicitly historical, and an interrupted preview can still be resumed.
    pub fn clear_topic(&mut self) {
        self.topic = None;
        self.pending_speed = None;
        self.pending_reading = None;
        self.checklist_choices = None;
    }
}

fn speed_choice(t: &str) -> Option<&'static str> {
    match t {
        "airspeed" | "air speed" | "indicated" | "indicated airspeed" => {
            Some("what is our airspeed")
        }
        "groundspeed" | "ground speed" | "over the ground" => Some("what is our groundspeed"),
        "vertical" | "vertical speed" | "climb rate" | "descent rate" => {
            Some("what is our vertical speed")
        }
        _ => None,
    }
}

fn subject_query(t: &str) -> Option<&'static str> {
    let t = t
        .strip_prefix("our ")
        .or_else(|| t.strip_prefix("the "))
        .unwrap_or(t);
    speed_choice(t).or(match t {
        "altitude" | "our altitude" => Some("what is our altitude"),
        "heading" | "our heading" => Some("what is our heading"),
        "fuel" | "our fuel" => Some("how much fuel do we have"),
        "endurance" | "fuel endurance" => Some("how long will the fuel last"),
        "flaps" | "the flaps" => Some("where are the flaps"),
        "gear" | "the gear" => Some("is the gear down"),
        "wind" | "the wind" => Some("what is the wind"),
        "parking brake" | "the parking brake" => Some("is the parking brake set"),
        "weather" | "visibility" | "temperature" => Some(match t {
            "weather" => "what is the weather",
            "visibility" => "what is the visibility",
            _ => "what is the outside temperature",
        }),
        "throttle" | "power setting" => Some("what is the throttle set to"),
        "mixture" => Some("what is the mixture set to"),
        "pitot heat" => Some("is pitot heat on"),
        "oil pressure" => Some("what is the oil pressure"),
        "oil temperature" => Some("what is the oil temperature"),
        "fuel flow" => Some("what is the fuel flow"),
        "battery" | "battery voltage" => Some("what is the battery voltage"),
        _ => None,
    })
}

/// Question opener + up to four complete subject phrases. Every word must
/// belong to the grammar; a matching noun inside a longer command is no match.
fn question_queries(t: &str) -> Option<Vec<&'static str>> {
    let t = t
        .strip_prefix("could you ")
        .or_else(|| t.strip_prefix("can you "))
        .or_else(|| t.strip_prefix("would you "))
        .unwrap_or(t);
    let t = t
        .strip_prefix("please ")
        .unwrap_or(t)
        .trim_end_matches(" please");
    let subject = [
        "tell me about ",
        "tell me ",
        "what is ",
        "what are ",
        "whats ",
        "how is ",
        "hows ",
        "check our ",
        "check the ",
    ]
    .iter()
    .find_map(|prefix| t.strip_prefix(prefix))?;
    let subjects: Vec<_> = subject.split(" and ").collect();
    if subjects.len() > 4 {
        return None;
    }
    let mut queries = Vec::new();
    for subject in subjects {
        let query = subject_query(subject)?;
        if !queries.contains(&query) {
            queries.push(query);
        }
    }
    Some(queries)
}

fn preview_request(t: &str) -> Option<&str> {
    if matches!(
        t,
        "preview a checklist" | "preview the checklist" | "walk me through a checklist"
    ) {
        return Some("");
    }
    let rest = [
        "walk me through ",
        "talk me through ",
        "preview ",
        "explain ",
    ]
    .iter()
    .find_map(|prefix| t.strip_prefix(prefix))?;
    rest.trim_start_matches("the ").strip_suffix(" checklist")
}

fn evidence_line(e: &Evidence) -> String {
    match e {
        Evidence::Observation { query, line } => format!("{query}: {line}"),
        Evidence::Action { request, line, .. } => format!("request {request}: {line}"),
        Evidence::Knowledge { short, .. } => short.clone(),
        Evidence::Readings { lines, .. } => lines.join("; "),
        Evidence::Status => "crew automation status".into(),
        Evidence::Preview => "checklist preview".into(),
    }
}

fn reference_query(t: &str) -> Option<&'static str> {
    match t {
        "best glide" | "best glide speed" => Some("what is best glide speed"),
        "stall speed" => Some("what is stall speed"),
        "flap limit" | "flap speed limit" => Some("what is the flap limit"),
        "cruise speed" => Some("what is cruise speed"),
        "blue line" => Some("what is blue line"),
        _ => None,
    }
}

/// Closed query catalog: a stored arbitrary utterance can never be replayed as
/// an action, nor can an unrelated reading be silently substituted.
fn state_query(t: &str) -> Option<&'static str> {
    if let Some(queries) = question_queries(t) {
        if queries.len() == 1 {
            return Some(queries[0]);
        }
    }
    match t {
        "what is our altitude" | "whats our altitude" | "how high are we" => {
            Some("what is our altitude")
        }
        "what is our heading" | "whats our heading" | "what heading are we on" => {
            Some("what is our heading")
        }
        "what is our airspeed" | "whats our airspeed" | "how fast are we" => {
            Some("what is our airspeed")
        }
        "what is our groundspeed" | "whats our groundspeed" => Some("what is our groundspeed"),
        "what is our vertical speed" | "whats our vertical speed" => {
            Some("what is our vertical speed")
        }
        "how much fuel do we have" | "how much fuel do we have left" => {
            Some("how much fuel do we have")
        }
        "how long will the fuel last" => Some("how long will the fuel last"),
        "where are the flaps" | "are the flaps up" => Some("where are the flaps"),
        "is the gear down" | "is our gear down" => Some("is the gear down"),
        "what is the wind" | "whats the wind" => Some("what is the wind"),
        "is the parking brake set" => Some("is the parking brake set"),
        _ => None,
    }
}

/// Authored explanations of THIS product's behavior, derived from its control
/// contracts. Aircraft procedures and performance figures remain in profiles
/// and checklists; no generated aviation advice or invented manual knowledge.
fn knowledge(t: &str) -> Option<(&'static str, &'static str)> {
    Some(match t {
        "how do you work" | "how do you understand me" => (
            "I match supported phrases to aircraft-specific commands and questions.",
            "Commands use curated tools with their own limits and readbacks. Conversation can request readings, but cannot move a control."),
        "why wont you guess" | "why do you ask for clarification" => (
            "An ambiguous request can refer to different aircraft systems.",
            "I ask you to name the intended system instead of substituting another control. A conversational answer never authorizes an automation."),
        "how do your checklists work" | "how do checklists work" => (
            "I read this aircraft's checklist, perform supported items, and verify items with available readings.",
            "An item with no simulator check remains a pilot callout. Ask for the checklist by name; discussing one does not start it."),
        "how do i get control back" | "what does my controls do" => (
            "Say my controls to request the aircraft back.",
            "That command goes through the normal control hand-back path. This explanation has not changed any controls."),
        "do you use an llm for this" | "does this conversation need an llm" => (
            "These conversation responses use rules and recorded facts, without an LLM.",
            "I can explain recorded reports and recheck supported readings. Open-ended questions outside those rules need an explicit supported answer."),
        _ => return None,
    })
}

/// Resolve "there" in a reminder against the last navigation answer:
/// "remind me when we get there" after "how long to the next waypoint" is a
/// next-waypoint reminder; after a top-of-descent answer it is a TOD one.
/// Returns the rewritten utterance, or `None` when nothing needs resolving
/// (no "there", an explicit target already named, or no fresh nav topic).
/// A destination answer has no reminder the tool can arm, so it stays
/// unresolved and the reminder grammar's own reply names what it can do.
pub fn resolve_there(text: &str, topic_query: Option<&str>) -> Option<String> {
    let t = words(text);
    let toks: Vec<&str> = t.split(' ').collect();
    let reminder = ["remind", "wake", "notify", "tell", "know", "heads"]
        .iter()
        .any(|w| toks.contains(w));
    if !reminder || !toks.contains(&"there") {
        return None;
    }
    let explicit = ["waypoint", "fix", "descent", "tod", "feet", "minutes", "minute", "altitude"]
        .iter()
        .any(|w| toks.contains(w));
    if explicit {
        return None;
    }
    let topic = topic_query?;
    let target = if topic.contains("waypoint") || topic.contains("fix") || topic.contains("how far") {
        "the next waypoint"
    } else if topic.contains("descent") || topic.contains("tod") {
        "top of descent"
    } else {
        return None;
    };
    let rewritten: Vec<&str> = toks
        .iter()
        .map(|w| if *w == "there" { target } else { *w })
        .collect();
    Some(rewritten.join(" "))
}

pub fn status_answer(managers: &[String]) -> String {
    if managers.is_empty() {
        "No crew automation is reporting an active task. That doesn't tell me whether the aircraft's autopilot is engaged.".into()
    } else {
        format!("Current crew automation: {}.", managers.join("; "))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn say(r: Option<Response>) -> String {
        match r {
            Some(Response::Say(s)) => s,
            other => panic!("expected speech, got {other:?}"),
        }
    }

    #[test]
    fn question_clarification_is_read_only_and_expires() {
        let now = Instant::now();
        let mut c = Conversation::default();
        assert!(say(c.respond("What's our speed?", now)).contains("groundspeed"));
        assert_eq!(
            c.respond("ground speed", now),
            Some(Response::Read("what is our groundspeed"))
        );
        c.respond("what speed", now);
        assert_eq!(c.respond("vertical", now + CLARIFICATION_TTL), None);
        c.respond("what speed", now);
        assert_eq!(c.respond("airspeed and gear up", now), None);
        assert_eq!(c.respond("airspeed", now), None);
    }

    #[test]
    fn multi_turn_reading_rechecks_and_changes_subject() {
        let now = Instant::now();
        let mut c = Conversation::default();
        c.observe("what's our altitude?", "3500 feet", now);
        assert_eq!(c.respond("say again", now), None);
        assert!(say(c.respond("why?", now)).contains("when you asked"));
        assert_eq!(
            c.respond("and now?", now),
            Some(Response::Read("what is our altitude"))
        );
        c.observe("what is our altitude", "3600 feet", now);
        assert_eq!(
            c.respond("and heading?", now),
            Some(Response::Read("what is our heading"))
        );
        c.observe("what is our heading", "heading 270", now);
        assert!(say(c.respond("how do you know?", now)).contains("heading 270"));
        assert!(!say(c.respond("why", now)).contains("3500"));
    }

    #[test]
    fn reports_never_turn_refusals_or_async_starts_into_success() {
        let now = Instant::now();
        let mut c = Conversation::default();
        for line in [
            "Negative, too fast for flaps.",
            "Starting the takeoff sequence.",
        ] {
            c.report("flaps ten", line, false, now);
            let answer = say(c.respond("what did you just do", now));
            assert!(answer.contains("reported:"));
            assert!(answer.contains(line));
            assert_eq!(
                c.respond("check that again", now),
                Some(Response::Say("Which reading should I check?".into()))
            );
        }
        c.report("flaps ten", "Simulator unavailable.", true, now);
        assert!(say(c.respond("why", now)).contains("couldn't complete"));
    }

    #[test]
    fn unrelated_turns_and_airframe_swaps_retire_context() {
        let now = Instant::now();
        let mut c = Conversation::default();
        c.observe("what is our altitude", "3500 feet", now);
        assert_eq!(c.respond("gear up", now), None);
        assert!(say(c.respond("why", now)).starts_with("Which"));
        c.respond("what speed", now);
        c.clear();
        assert_eq!(c.respond("vertical", now), None);
        c.observe("what is our altitude", "3500 feet", now);
        assert!(say(c.respond("why", now + TOPIC_TTL)).starts_with("Which"));
    }

    #[test]
    fn authored_knowledge_has_followups_and_session_preferences() {
        let now = Instant::now();
        let mut c = Conversation::default();
        assert!(say(c.respond("how do your checklists work", now)).contains("verify"));
        assert!(say(c.respond("tell me more", now)).contains("pilot callout"));
        c.respond("keep the explanations shorter from now on", now);
        c.clear();
        assert_eq!(c.detail, Detail::Brief);
        c.respond("how do your checklists work", now);
        assert!(!say(c.respond("tell me more", now)).contains("pilot callout"));
    }

    #[test]
    fn adversarial_conversation_never_selects_an_action() {
        let now = Instant::now();
        for text in [
            "more coffee",
            "repeat the takeoff",
            "why gear up",
            "thanks gear up",
            "what about gear up",
            "and heading 270",
            "yes",
            "do that again",
            "stop",
            "my controls",
        ] {
            let mut c = Conversation::default();
            c.observe("what is our altitude", "3500 feet", now);
            assert_eq!(c.respond(text, now), None, "{text}");
        }
    }

    #[test]
    fn live_status_does_not_invent_a_wait_reason_or_control_ownership() {
        assert!(status_answer(&[]).contains("doesn't tell me"));
        assert!(
            status_answer(&["full auto: wait for clearance".into()]).contains("wait for clearance")
        );
        let mut c = Conversation::default();
        let now = Instant::now();
        assert_eq!(c.respond("why are we waiting", now), Some(Response::Status));
        assert_eq!(c.respond("why", now), Some(Response::Status));
    }

    #[test]
    fn composed_questions_cover_subjects_but_never_partially_match() {
        let now = Instant::now();
        let mut c = Conversation::default();
        assert_eq!(
            c.respond(
                "Could you please tell me our altitude and the heading?",
                now
            ),
            Some(Response::ReadMany(vec![
                "what is our altitude",
                "what is our heading"
            ]))
        );
        for subject in [
            "oil pressure",
            "oil temperature",
            "fuel flow",
            "battery voltage",
            "pitot heat",
            "weather",
            "endurance",
        ] {
            assert!(matches!(
                c.respond(&format!("Tell me about the {subject}, please."), now),
                Some(Response::Read(_))
            ));
        }
        assert_eq!(
            c.respond("what is our altitude and altitude", now),
            Some(Response::Read("what is our altitude"))
        );
        for text in [
            "tell me altitude and gear up",
            "check our heading 270",
            "tell me about the fuel pump on",
            "could you not check our altitude",
            "what is our altitude and coffee",
            "what is our altitude and heading and fuel and wind and flaps",
        ] {
            assert_eq!(c.respond(text, now), None, "{text}");
        }
    }

    #[test]
    fn earlier_topics_and_requests_survive_a_different_question() {
        let now = Instant::now();
        let mut c = Conversation::default();
        c.report("flaps ten", "Negative, too fast.", false, now);
        c.observe("could you tell me our altitude", "3500 feet", now);
        c.observe("what is our heading", "heading 270", now);
        assert!(say(c.respond("back to altitude", now)).contains("3500 feet"));
        assert_eq!(
            c.respond("and now", now),
            Some(Response::Read("what is our altitude"))
        );
        assert!(say(c.respond("what was my last request", now)).contains("Negative, too fast"));
        assert!(say(c.respond("recap our conversation", now)).contains("heading 270"));
        assert!(!say(c.respond("back to altitude", now + TOPIC_TTL)).contains("3500"));
        c.clear();
        assert!(say(c.respond("recap our conversation", now)).starts_with("No recent"));
    }

    #[test]
    fn grouped_readings_refresh_as_a_group_and_individual_topics_are_recallable() {
        let now = Instant::now();
        let mut c = Conversation::default();
        let queries = vec!["what is our altitude", "what is our heading"];
        c.observe_many(
            queries.clone(),
            vec!["3500 feet".into(), "heading 270".into()],
            now,
        );
        assert_eq!(c.respond("and now", now), Some(Response::ReadMany(queries)));
        assert!(say(c.respond("why", now)).contains("heading 270"));
        assert!(say(c.respond("back to altitude", now)).contains("3500"));
    }

    #[test]
    fn clarification_accepts_a_named_reading_and_retires_on_new_topics() {
        let now = Instant::now();
        let mut c = Conversation::default();
        assert!(say(c.respond("check that again", now)).contains("Which reading"));
        assert_eq!(
            c.respond("the oil pressure", now),
            Some(Response::Read("what is the oil pressure"))
        );
        c.respond("what speed", now);
        c.respond("back to altitude", now);
        assert_eq!(
            c.respond("vertical", now),
            None,
            "new topic retired speed question"
        );
        c.respond("check that again", now);
        assert_eq!(c.respond("oil pressure", now + CLARIFICATION_TTL), None);
    }

    #[test]
    fn checklist_preview_can_be_interrupted_and_explicitly_resumed() {
        let now = Instant::now();
        let mut c = Conversation::default();
        assert_eq!(
            c.respond("walk me through the before takeoff checklist", now),
            Some(Response::Checklist("before takeoff".into()))
        );
        let line = c.start_preview(
            "Before Takeoff".into(),
            vec!["Brakes set".into(), "Flaps ten".into()],
            now,
        );
        assert!(line.contains("not performing"));
        assert!(say(c.respond("next item", now)).contains("Flaps ten"));
        c.respond("pause the preview", now);
        assert_eq!(c.respond("next item", now), None);
        assert!(say(c.respond("resume the preview", now)).contains("item 2 of 2"));
        assert!(say(c.respond("previous item", now)).contains("item 1 of 2"));
        assert_eq!(
            c.respond("flaps ten", now),
            None,
            "explicit commands reach normal grammar"
        );
        assert_eq!(
            c.respond("next", now),
            None,
            "an unrelated command suspends preview"
        );
        c.respond("resume the preview", now);
        c.respond("next item", now);
        assert!(say(c.respond("next item", now)).contains("last preview item"));
        c.clear();
        assert!(say(c.respond("resume the preview", now)).contains("No checklist preview"));
    }

    #[test]
    fn checklist_choice_and_preview_expire_without_arming_anything() {
        let now = Instant::now();
        let mut c = Conversation::default();
        c.choose_checklist(vec!["Before Takeoff".into(), "Before Landing".into()], now);
        assert_eq!(
            c.respond("before takeoff", now),
            Some(Response::Checklist("Before Takeoff".into()))
        );
        c.choose_checklist(vec!["Before Takeoff".into()], now);
        assert_eq!(c.respond("before takeoff", now + CLARIFICATION_TTL), None);
        c.start_preview("Before Takeoff".into(), vec!["Brakes set".into()], now);
        assert!(
            say(c.respond("resume the preview", now + TOPIC_TTL)).contains("No checklist preview")
        );
        for text in [
            "walk me through takeoff and do it",
            "preview gear up",
            "next item and start engines",
        ] {
            assert_eq!(c.respond(text, now), None);
        }
    }

    #[test]
    fn history_is_bounded_even_for_a_long_session() {
        let now = Instant::now();
        let mut c = Conversation::default();
        for n in 0..100 {
            c.observe("what is our altitude", &format!("{n} feet"), now);
        }
        assert_eq!(c.history.len(), HISTORY_MAX);
        assert!(say(c.respond("back to altitude", now)).contains("99 feet"));
    }

    #[test]
    fn unavailable_reading_does_not_erase_other_topics() {
        let now = Instant::now();
        let mut c = Conversation::default();
        c.observe("what is our altitude", "3500 feet", now);
        c.respond("tell me our heading", now);
        c.clear_topic();
        assert!(say(c.respond("why", now)).starts_with("Which"));
        assert!(say(c.respond("back to altitude", now)).contains("3500 feet"));
        c.clear();
        assert!(!say(c.respond("back to altitude", now)).contains("3500"));
    }

    #[test]
    fn there_resolves_against_the_last_navigation_answer() {
        let now = Instant::now();
        let mut c = Conversation::default();
        assert!(resolve_there("remind me when we get there", None).is_none());
        c.observe("How long till next waypoint?", "Next waypoint PIMOS, 18 miles, 18 minutes.", now);
        let topic = c.recent_observation_query(now);
        assert_eq!(topic.as_deref(), Some("how long till next waypoint"));
        assert_eq!(
            resolve_there("remind me when we get there", topic.as_deref()).as_deref(),
            Some("remind me when we get the next waypoint")
        );
        // An explicit target is left alone; so is a reminder with no "there".
        assert!(resolve_there("remind me when we get to the next waypoint", topic.as_deref()).is_none());
        assert!(resolve_there("remind me in ten minutes", topic.as_deref()).is_none());
        // Top of descent as the topic.
        c.observe("when is top of descent", "About 20 minutes to top of descent, 30 miles ahead.", now);
        assert_eq!(
            resolve_there("wake me when we get there", c.recent_observation_query(now).as_deref()).as_deref(),
            Some("wake me when we get top of descent")
        );
        // A destination ETA has no reminder to arm: unresolved, on purpose.
        c.observe("how long shall we get to arrival", "About 41 minutes to LEMG.", now);
        assert!(resolve_there("remind me when we get there", c.recent_observation_query(now).as_deref()).is_none());
        // The topic expires with the conversation's own TTL.
        c.observe("how far to the next fix", "Next waypoint PIMOS, 5 miles.", now);
        assert!(c.recent_observation_query(now + TOPIC_TTL + Duration::from_secs(1)).is_none());
    }
}
