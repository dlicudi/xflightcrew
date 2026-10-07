//! The flight surfaces: speedbrakes, gear, flaps, pitch trim.

use super::*;

/// Speedbrakes, gear and flaps.
pub(super) fn config(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // speedbrakes / spoilers: extend / retract / arm.
    if t.contains("speedbrake")
        || t.contains("speed brake")
        || has(t, "spoiler")
        || has(t, "spoilers")
    {
        let action = if has(t, "arm") {
            "arm"
        } else if has(t, "out") || t.contains("extend") || t.contains("deploy") || has(t, "up") {
            "extend"
        } else {
            "retract" // in / retract / stow / down
        };
        return Some(call("speedbrake", json!({ "action": action })));
    }

    // gear up / down (retractable-gear airframes; the tool itself gates on-ground).
    if has(t, "gear") {
        if has(t, "up") || t.contains("retract") || t.contains("raise") {
            return Some(call("gear", json!({ "position": "up" })));
        }
        if has(t, "down") || t.contains("lower") || t.contains("extend") {
            return Some(call("gear", json!({ "position": "down" })));
        }
    }

    // flaps up / ten / twenty / full / approach / down / quarter / half. The
    // detent token must match a label the airframe declares; an unmatched value
    // (e.g. "flaps 10" on the King Air) still routes to set_flaps, which replies
    // with the valid options rather than a bare "Say again?". Order matters:
    // "three quarter" before "quarter", words before the bare number.
    if has(t, "flaps") || has(t, "flap") {
        let notch = if has(t, "up") {
            "up".to_string()
        } else if t.contains("three quarter") || t.contains("three-quarter") {
            "three-quarter".to_string()
        // Whisper garbles of "full" heard live on the Twotter approach
        // (2026-07-09): "Flaps, ooh." / "Flaps, cool." / "Flaps, pull.",
        // four dead ends in a row for the same lever. "up" wins above, so
        // "pull the flaps up" is untouched.
        } else if has(t, "full") || has(t, "ooh") || has(t, "cool") || has(t, "pull") {
            "full".to_string()
        } else if has(t, "approach") {
            "approach".to_string()
        } else if has(t, "down") {
            "down".to_string()
        } else if has(t, "half") {
            "half".to_string()
        } else if has(t, "quarter") {
            "quarter".to_string()
        // "flaps for landing" / "landing flaps", the tool resolves the alias to
        // the airframe's last detent (and the Vfe gate still refuses if fast).
        } else if has(t, "landing") || has(t, "land") {
            "landing".to_string()
        } else if let Some(n) = num {
            n.to_string()
        } else {
            return line.stop();
        };
        return Some(call("set_flaps", json!({ "notch": notch })));
    }

    None
}

/// Pitch trim.
pub(super) fn trim(line: &Line) -> Option<ToolCall> {
    let t = line.t;

    // pitch trim
    if has(t, "trim") {
        let action = if t.contains("nose up") || has(t, "up") {
            "nose_up"
        } else if t.contains("nose down") || has(t, "down") {
            "nose_down"
        } else if has(t, "neutral") {
            "neutral"
        } else if has(t, "takeoff") {
            "takeoff"
        } else {
            return line.stop();
        };
        return Some(call("set_pitch_trim", json!({ "action": action })));
    }

    None
}
