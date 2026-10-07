//! The autopilot: its modes, heading, and the vertical.

use super::*;

/// The autopilot's own modes: level change, LNAV, autothrottle.
pub(super) fn modes(line: &Line) -> Option<ToolCall> {
    let t = line.t;

    // The autopilot's own modes (#92): level change and GPS steering. A mode
    // word with no "off" is an engage; "level change" carries no number, so
    // the flight-level rule never sees it.
    if t.contains("level change") || has(t, "flch") || has(t, "flc") {
        let on = !(has(t, "off") || t.contains("disengage") || t.contains("cancel"));
        return Some(call("ap_mode", json!({ "mode": "level_change", "on": on })));
    }
    if has(t, "gpss") || t.contains("gps steering") || has(t, "lnav") || t.contains("l nav") {
        let on = !(has(t, "off") || t.contains("disengage") || t.contains("cancel"));
        return Some(call("ap_mode", json!({ "mode": "lnav", "on": on })));
    }
    // Autothrottle (jets), before the throttle rule. "autothrottle on/off".
    if t.contains("autothrottle") || t.contains("auto throttle") {
        let on = !(has(t, "off") || t.contains("disconnect") || t.contains("disengage"));
        return Some(call("autothrottle", json!({ "on": on })));
    }

    None
}

/// Heading: a number, the current heading or a compass word.
pub(super) fn heading(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // heading 270 / heading north. Whisper wraps a bare "heading north" in a
    // false first-person opener ("So I'm heading north" for a turn command,
    // 2026-07-12), those ARE commands here, so no narration guard: a genuine
    // question ("is that heading north?") is withheld upstream by its
    // interrogative opener, not by refusing to parse it. (The earlier
    // "narration re-bugged the heading" scare was really the drifted gyro.)
    if has(t, "heading") {
        if let Some(n) = num {
            return Some(call("set_heading", json!({ "heading": n })));
        }
        // "keep/hold/maintain current heading", "hold this heading", bug
        // what we're flying now ("keep current heading" beeped, 2026-07-12).
        if has(t, "current") || has(t, "present") || t.contains("this heading") {
            return Some(call("set_heading", json!({ "current": true })));
        }
        // Intercardinals first (so "northeast"/"north east" aren't shadowed by the
        // cardinal checks), each in one-word and two-word spellings.
        let dir = if has(t, "northeast") || (has(t, "north") && has(t, "east")) {
            Some(45)
        } else if has(t, "southeast") || (has(t, "south") && has(t, "east")) {
            Some(135)
        } else if has(t, "southwest") || (has(t, "south") && has(t, "west")) {
            Some(225)
        } else if has(t, "northwest") || (has(t, "north") && has(t, "west")) {
            Some(315)
        } else if has(t, "north") {
            Some(0)
        } else if has(t, "east") {
            Some(90)
        } else if has(t, "south") {
            Some(180)
        } else if has(t, "west") {
            Some(270)
        } else {
            None
        };
        if let Some(d) = dir {
            return Some(call("set_heading", json!({ "heading": d })));
        }
    }

    None
}

/// Climb and descent: an altitude target, a rate, or both.
pub(super) fn vertical(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // climb/descend TO a target AT a rate -> set_climb (AP captures & levels off)
    if has(t, "climb") || has(t, "descend") || has(t, "descent") || has(t, "altitude") {
        if let Some((before, after)) = t.split_once(" at ") {
            if let (Some(target), Some(rate)) = (
                before.rsplit_once(" to ").and_then(|(_, a)| number(a)),
                number(after),
            ) {
                return Some(call(
                    "set_climb",
                    json!({ "altitude": target, "fpm": rate }),
                ));
            }
        }
    }

    // climb / descend / altitude, one number, routed to an altitude TARGET or a
    // vertical-speed RATE:
    //   target  if "to X" / "altitude X" / "maintain X", or a value too big to be
    //           a VS rate ("climb 7000" → altitude, not 7000 fpm)
    //   rate    otherwise ("climb 500", "descend at 700")
    let vertical = has(t, "vertical") || has(t, "climb") || has(t, "descend") || has(t, "descent");
    if vertical || has(t, "altitude") || has(t, "maintain") {
        if let Some(n) = num {
            // An explicit "vertical speed" command is ALWAYS a rate, never an
            // altitude target: the keyword names the VS control directly, so the
            // "to X" / big-number heuristics that disambiguate a bare "climb X"
            // must not override it ("set vertical speed to minus 1000" is -1000
            // fpm, not an altitude of 1000; "vertical speed 3000 down" is -3000
            // fpm, not a climb to 3000 ft).
            let explicit_vs = has(t, "vertical");
            let is_target = !explicit_vs
                && (t.contains(" to ")
                    || has(t, "altitude")
                    || has(t, "maintain")
                    || n.abs() > 2500);
            if is_target {
                return Some(call("set_altitude", json!({ "altitude": n.abs() })));
            }
            let down = has(t, "descend")
                || has(t, "descent")
                || has(t, "down")
                || has(t, "minus")
                || has(t, "negative");
            let fpm = if down { -n.abs() } else { n.abs() };
            return Some(call("set_vertical_speed", json!({ "fpm": fpm })));
        }
    }

    None
}
