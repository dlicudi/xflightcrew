//! On the ground: the ramp and the brakes.

use super::*;

/// Ramp ops: chocks, ground power, pushback, ground service.
pub(super) fn ramp(line: &Line) -> Option<ToolCall> {
    let t = line.t;

    // Ground / ramp ops first, "ground power" must beat the throttle branch (which
    // matches "power"), and these are unambiguous ground-crew calls.
    if has(t, "chocks") || has(t, "chock") {
        let on = !(has(t, "out")
            || has(t, "off")
            || has(t, "remove")
            || has(t, "pull")
            || has(t, "away"));
        return Some(call("set_chocks", json!({ "on": on })));
    }
    if (has(t, "ground") && has(t, "power")) || has(t, "gpu") {
        let on = !(has(t, "off") || has(t, "disconnect") || has(t, "remove") || has(t, "away"));
        return Some(call("set_ground_power", json!({ "on": on })));
    }
    if has(t, "pushback") || (has(t, "push") && has(t, "back")) {
        // "Skip pushback" and "cancel pushback, just fly" parsed as a fresh
        // straight push and summoned the tug the pilot was refusing (LEMG,
        // 2026-09-12): every word of refusal is a stop.
        let action = if has(t, "stop")
            || has(t, "halt")
            || has(t, "hold")
            || has(t, "cancel")
            || has(t, "skip")
            || has(t, "abort")
            || has(t, "no")
            || has(t, "without")
            || has(t, "forget")
            || has(t, "don")
            || has(t, "dont")
            || has(t, "never")
        {
            "stop"
        } else if has(t, "left") {
            "left"
        } else if has(t, "right") {
            "right"
        } else {
            "straight"
        };
        return Some(call("pushback", json!({ "action": action })));
    }
    if has(t, "refuel")
        || (has(t, "ground") && (has(t, "service") || has(t, "services")))
        || (has(t, "service") && has(t, "plane"))
        || (has(t, "top") && has(t, "off"))
    {
        return Some(call("request_service", json!({})));
    }

    None
}

/// The parking brake and the crew-held brakes.
pub(super) fn brakes(line: &Line) -> Option<ToolCall> {
    let t = line.t;

    // parking brake (STT often hears "brake" as "break")
    if t.contains("parking brake")
        || t.contains("park brake")
        || t.contains("parking break")
        || t.contains("park break")
    {
        let release = has(t, "release") || has(t, "released") || has(t, "off");
        return Some(call("set_parking_brake", json!({ "on": !release })));
    }

    // regular brakes, crew-held: "brakes on" / "hold the brakes" / "brakes
    // off" / "release the brakes". After the parking-brake rule so "parking
    // brake off" keeps its lever (speedbrakes matched far above); a verb is
    // required, bare "brakes" alone is ambiguous and falls through. Same
    // "break" mishear as the parking brake.
    if (has(t, "brakes") || has(t, "brake") || has(t, "breaks") || has(t, "break"))
        && !t.contains("speed")
    {
        if has(t, "off") || has(t, "release") || has(t, "released") || has(t, "disengage") {
            return Some(call("hold_brakes", json!({ "on": false })));
        }
        if has(t, "on") || has(t, "hold") || has(t, "holding") || has(t, "set") {
            return Some(call("hold_brakes", json!({ "on": true })));
        }
    }

    None
}
