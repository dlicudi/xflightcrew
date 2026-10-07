//! Speed: stop and go on the ground, the speed hold, the AP speed bug.

use super::*;

/// Speed: stop and go, the speed hold, the AP airspeed select, slow down.
pub(super) fn speed(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // Speed hold, the copilot works the throttle ("hold 70 knots", "maintain
    // approach speed", "can you maintain an appropriate speed", asked live with
    // no tool to answer it). Verbs first, so it beats the AP speed-bug rule
    // below on airframes that have this tool; the release phrasings first of
    // all, so "stop holding speed" doesn't re-arm.
    // STOP, brakes to a halt and hold. The bare word only as the whole
    // utterance ("stop", "stop stop stop"), so "stop holding speed" below
    // keeps its meaning; "stop taxiing" means stop the AIRCRAFT, not drop
    // the hold and coast (asked for live, 2026-07-11). "go around" never
    // lands here: these are cockpit words, and around isn't in the set.
    {
        let words: Vec<&str> = t
            .split_whitespace()
            .map(|w| w.trim_matches(|c: char| !c.is_ascii_alphanumeric() && c != '\''))
            .filter(|w| !w.is_empty())
            .collect();
        let bare_stop = !words.is_empty() && words.len() <= 3 && words.iter().all(|w| *w == "stop");
        if bare_stop
            || t.contains("stop here")
            || t.contains("stop the aircraft")
            || t.contains("stop taxi") // covers "stop taxiing" too
            || t.contains("hold position")
            || t.contains("hold short")
            || t.contains("holding short")
        {
            return Some(call("hold_speed", json!({ "knots": 0 })));
        }
        // GO, resume the last held taxi speed after a stop. Bare "go" only
        // as the whole utterance; the phrases must name the roll.
        let bare_go = matches!(
            words.as_slice(),
            ["go"] | ["go", "go"] | ["go", "go", "go"] | ["lets", "go"] | ["let's", "go"]
        );
        if bare_go
            || t.contains("continue taxi")
            || t.contains("resume taxi")
            || t.contains("keep going")
            || t.contains("carry on")
        {
            return Some(call("hold_speed", json!({ "resume": true })));
        }
    }
    if t.contains("stop holding")
        || t.contains("my throttle")
        || t.contains("my power")
        || (has(t, "release") && (has(t, "throttle") || has(t, "speed")))
    {
        return Some(call("hold_speed", json!({ "off": true })));
    }
    // "taxi at 10 knots" / "taxi 12 knots", the taxi governor (the tool picks
    // taxi vs flight mode from being on the ground + the number). Requires the
    // number so "taxi light on" (no knots) stays with the lights.
    if has(t, "taxi") && (has(t, "knots") || has(t, "knot")) {
        // The number AFTER "taxi" is the speed: "6? Taxi at 20 knots" (LIPE,
        // 2026-09-25) was a stray word, then the call, and took the 6.
        let after = t.find("taxi").and_then(|i| number(&t[i..]));
        if let Some(n) = after.or(num) {
            return Some(call("hold_speed", json!({ "knots": n })));
        }
    }
    let holdish = has(t, "hold") || has(t, "maintain") || has(t, "keep") || has(t, "manage");
    if holdish
        && (has(t, "speed") || has(t, "knots") || has(t, "airspeed"))
        && !has(t, "vertical")
        && !has(t, "ground")
    {
        if has(t, "approach") || has(t, "appropriate") || has(t, "landing") {
            return Some(call("hold_speed", json!({ "preset": "approach" })));
        }
        return Some(match num {
            Some(n) => call("hold_speed", json!({ "knots": n })),
            None => call("hold_speed", json!({})), // hold what we're doing now
        });
    }
    // "Can you manage throttle for landing?" (SR22 on final, LSZS-LILO
    // 2026-09-24, a dead end): the lever named instead of the speed. Only
    // the asking verbs "manage"/"handle" take the throttle this way ("keep
    // the power up" is not a request for the governor), and only with the
    // landing named: that is the approach speed, which the governor idles
    // at the touchdown. A bare "manage the throttle" holds what we have.
    if (has(t, "manage") || has(t, "handle"))
        && (has(t, "throttle") || has(t, "throttles") || has(t, "power"))
        && !t.contains("percent")
    {
        if has(t, "landing") || has(t, "approach") || has(t, "final") {
            return Some(call("hold_speed", json!({ "preset": "approach" })));
        }
        return Some(call("hold_speed", json!({})));
    }

    // AP airspeed select: "set speed 250", "speed 280", "airspeed 250". Excludes
    // vertical/ground speed (handled elsewhere) and percent (that's throttle).
    // A bare knots unit qualifies too ("reduced to 60 knots", seen dead-ending
    // live), but never wind talk ("wind is ten knots" is weather, not a bug).
    if (has(t, "speed")
        || has(t, "airspeed")
        || ((has(t, "knots") || has(t, "knot")) && !has(t, "wind")))
        && !has(t, "vertical")
        && !has(t, "ground")
        && !has(t, "climb")
        && !has(t, "descend")
        && !t.contains("percent")
    {
        // "speed for landing" / "approach speed" / "landing speed", the tool
        // pulls the profile's final-approach speed.
        if has(t, "landing") || has(t, "land") || has(t, "approach") {
            return Some(call("set_speed", json!({ "preset": "approach" })));
        }
        // Relative before the bare number: "reduce speed by 20" is minus twenty
        // from the bug, not a 20-knot target. Default nudge is 20 knots.
        // "reduce speed TO 180" is a target, not a decrement, "to" marks it,
        // same as everywhere else in command speech.
        if has(t, "reduce") || has(t, "decrease") || has(t, "lower") {
            if has(t, "to") {
                if let Some(n) = num {
                    return Some(call("set_speed", json!({ "knots": n })));
                }
            }
            return Some(call("set_speed", json!({ "delta": -num.unwrap_or(20) })));
        }
        if has(t, "increase") || has(t, "raise") {
            if has(t, "to") {
                if let Some(n) = num {
                    return Some(call("set_speed", json!({ "knots": n })));
                }
            }
            return Some(call("set_speed", json!({ "delta": num.unwrap_or(20) })));
        }
        if let Some(n) = num {
            return Some(call("set_speed", json!({ "knots": n })));
        }
    }
    // "slow down" / "slow to 180", no "speed" token, so it needs its own rule.
    if has(t, "slow") && !has(t, "vertical") && !t.contains("percent") {
        if let Some(n) = num {
            return Some(call("set_speed", json!({ "knots": n })));
        }
        return Some(call("set_speed", json!({ "delta": -20 })));
    }

    None
}
