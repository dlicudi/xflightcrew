//! Approaches: the ILS, a named autoland runway, the talk-down, NAV coupling.

use super::*;

/// Approaches: the ILS, autoland to a named runway, the talk-down, NAV coupling.
pub(super) fn approach(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // ILS approach setup, was LLM-only, so every phrasing missed with the LLM
    // off ("can we set up ILS for runway 28" heard live, twice). "ils" appears
    // in no other command; the number (if any) is the runway. The tool answers
    // honestly when the field has no ILS, which beats a silent miss.
    // "Couple the approach" / "arm the approach" is the explicit ask for the
    // autopilot to fly it (a pilot hand flying keeps the aircraft through a
    // plain setup). The side of a parallel is kept: "two four left".
    let couple = has(t, "couple")
        || has(t, "coupled")
        || t.contains("arm the approach")
        || t.contains("arm approach")
        || t.contains("approach mode");
    if has(t, "ils")
        || (couple && (has(t, "approach") || has(t, "localizer") || has(t, "localiser")))
    {
        let side = if has(t, "left") {
            "L"
        } else if has(t, "right") {
            "R"
        } else if has(t, "center") || has(t, "centre") {
            "C"
        } else {
            ""
        };
        let runway = num.map(|n| format!("{n:02}{side}")).unwrap_or_default();
        return Some(call(
            "prepare_ils",
            json!({ "runway": runway, "couple": couple }),
        ));
    }

    // "Autoland runway 31": the crew lands it on a NAMED runway at the field
    // we are at (the plain "land at runway 31" is the pilot's talk-down
    // below). Without it autoland could only take the plan's destination or
    // the nearest, and the nearest at LEMG was 30, not 31's ILS (2026-10-07).
    if (has(t, "autoland") || t.contains("auto land")) && has(t, "runway") {
        if let Some(n) = num {
            return Some(call("autoland", json!({ "runway": n.to_string() })));
        }
    }
    // Talk-down: spoken visual-approach guidance to any runway (the approach a
    // procedure-less field like LERL will never have). Cancel words first so
    // "stop the guidance" doesn't re-arm it.
    let talkdown = t.contains("talk me down")
        || t.contains("guide me in")
        || t.contains("call the approach")
        || t.contains("approach guidance")
        || t.contains("guidance")
        || (has(t, "visual") && has(t, "approach"))
        // "I want to land at runway 28" (lead filler peels to "land at runway
        // 28"), a landing-help request; the advisory talk-down is that help.
        || (has(t, "land") && has(t, "runway"));
    if talkdown {
        if has(t, "stop") || has(t, "cancel") || has(t, "enough") || has(t, "quit") {
            return Some(call("visual_approach", json!({ "cancel": true })));
        }
        let runway = num.map(|n| n.to_string()).unwrap_or_default();
        return Some(call("visual_approach", json!({ "runway": runway })));
    }

    // Couple the autopilot to the GPS flight plan (NAV mode). "nav" alone is the
    // nav lights, so require navigate/follow/track + a route word, "nav mode",
    // or an explicit AP coupling ("set AP to nav", "set autopilot to navigation",
    // both missed live, twice, before "navigation mode" finally landed).
    if has(t, "navigate")
        || ((has(t, "follow") || has(t, "track") || has(t, "fly"))
            && (t.contains("flight plan")
                || t.contains("route")
                || t.contains("root")
                || t.contains("road")
                || has(t, "plan")))
        || (has(t, "nav") && (t.contains("mode") || has(t, "engage")) && !t.contains("light"))
        || ((has(t, "ap") || t.contains("autopilot"))
            && (has(t, "nav") || t.contains("navigation"))
            && !t.contains("light"))
        || t.contains("navigation mode")
    {
        return Some(call("navigate", json!({})));
    }

    None
}
