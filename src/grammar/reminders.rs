//! Reminders the pilot asks for.

use super::*;

/// Reminders: top of descent, the next waypoint, minutes, an altitude.
pub(super) fn reminder(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // Reminders: "wake me at top of descent", "tell me when we reach 5000 feet",
    // "remind me in 10 minutes", "let me know at the next waypoint". The verb gates
    // it so a plain "top of descent" or a bare altitude isn't swallowed; an
    // unrecognized target falls through (e.g. "tell me the weather").
    // A complaint or past-tense reference ("you didn't remind me", "you never woke
    // me") is not a request to arm one.
    let negated = t.contains("didnt")
        || t.contains("did not")
        || t.contains("never")
        || t.contains("forgot")
        || t.contains("should have")
        || t.contains("supposed to");
    let reminder_verb = !negated
        && (has(t, "wake")
            || has(t, "remind")
            || has(t, "notify")
            || (has(t, "tell") && has(t, "me"))
            || t.contains("let me know")
            || t.contains("heads up")
            || t.contains("heads-up"));
    if reminder_verb {
        if t.contains("top of descent") || has(t, "tod") || (has(t, "descent") && !has(t, "rate")) {
            return Some(call("set_reminder", json!({ "kind": "top_of_descent" })));
        }
        if has(t, "waypoint") || has(t, "fix") {
            return Some(call("set_reminder", json!({ "kind": "next_waypoint" })));
        }
        if (has(t, "minute") || has(t, "minutes")) && num.is_some() {
            return Some(call(
                "set_reminder",
                json!({ "kind": "minutes", "value": num }),
            ));
        }
        if num.is_some()
            && (has(t, "feet")
                || has(t, "foot")
                || has(t, "altitude")
                || has(t, "reach")
                || has(t, "at")
                || has(t, "when"))
        {
            return Some(call(
                "set_reminder",
                json!({ "kind": "altitude", "value": num }),
            ));
        }
    }

    None
}
