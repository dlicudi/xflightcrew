//! The fuel selector.

use super::*;

/// The fuel selector.
pub(super) fn selector(line: &Line) -> Option<ToolCall> {
    let t = line.t;

    // fuel selector / tank. Explicit side wins; a swap verb with no side is the
    // routine "switch tanks" (the SR22 has no BOTH, so it's said every half hour)
    // so the tool reads the current tank and feeds from the other one. Also match
    // bare "fuel" with a swap verb or an explicit "to <side>" ("switch fuel to
    // right", "set fuel to left"), seen live, five times in one session, routed
    // to the quantity report instead. "fuel pump"/"boost" stays with the pump rule.
    let swap_verb = has(t, "switch")
        || has(t, "swap")
        || has(t, "change")
        || has(t, "alternate")
        || has(t, "rotate");
    let fuel_to_side = has(t, "fuel")
        && !t.contains("pump")
        && !has(t, "boost")
        && (swap_verb || t.contains("to left") || t.contains("to right") || t.contains("to off"));
    if t.contains("fuel selector") || has(t, "tank") || has(t, "tanks") || fuel_to_side {
        let pos = if has(t, "both") {
            "both"
        } else if has(t, "left") {
            "left"
        } else if has(t, "right") {
            "right"
        } else if has(t, "off") {
            "off"
        } else if swap_verb {
            "switch"
        } else {
            return line.stop();
        };
        return Some(call("set_fuel_selector", json!({ "position": pos })));
    }

    None
}
