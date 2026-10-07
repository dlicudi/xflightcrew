//! Who flies the aircraft: releases, hand-overs, recovery, autoland, level off.

use super::*;

/// Who flies: the pilot's release, the crew hand-over, recovery, autoland, level off, autopilot off.
pub(super) fn handover(line: &Line) -> Option<ToolCall> {
    let t = line.t;

    // EMERGENCY recovery, take the controls and fly out of an upset/stall.
    // Whole-aircraft distress phrasings ONLY: "take control of the throttle"
    // once engaged a full recovery on a calm approach (the copilot seized the
    // aircraft and the pilot needed four phrasings to get it back), and "help
    // me land ..." is a request for guidance, not a mayday. A named system
    // after "take control of" means a system request, never recovery.
    // Direction of transfer matters: "take control" is only a request for the
    // COPILOT to take over when it isn't the pilot announcing they're taking it
    // themselves. "Let me take controls" once engaged a full recovery against
    // the pilot, the exact inversion of what was said.
    let pilot_takes = t.contains("let me")
        || has(t, "ill")
        || t.starts_with("i will ")
        || t.contains(" i will ")
        || t.contains("i take")
        || t.contains("ill take")
        || t.contains("i will take")
        || t.contains("im taking")
        || t.contains("i am taking")
        || t.contains("can i take")
        || t.contains("i want");
    // "Please take over" was dropped without a word (LEMG, 2026-09-12):
    // the same hand-over as "take control", with the same direction test.
    let whole_aircraft_takeover =
        (t.contains("take control") || t.contains("take the controls") || t.contains("take over"))
            && !pilot_takes
            && ![
                "throttle", "power", "radio", "radios", "flaps", "speed", "heading", "lights",
            ]
            .iter()
            .any(|sys| has(t, sys));
    // The pilot taking over is a release: autopilot off, recovery dropped.
    // "I'll taxi", "let me fly", "I'll take it" (2026-09-25): the pilot
    // taking the aircraft, not asking for anything.
    if pilot_takes
        && (t.contains("control")
            || t.contains("the aircraft")
            || t.contains("the plane")
            || t.contains("take over")
            || t.ends_with("take it")
            || t.ends_with("taxi")
            || t.ends_with("fly")
            || t.ends_with("fly it"))
    {
        return Some(call("autopilot_off", json!({})));
    }
    // The crew hand-over: the copilot takes the aircraft and does what the
    // flight needs now (taxi, rejoin the plan, land, hold the level), read
    // from the flight model. Until 2026-09-25 every one of these armed the
    // upset RECOVERY, on a calm cruise or on the taxiway alike; an actual
    // upset still gets it from the hand-over itself.
    if whole_aircraft_takeover
        || t.contains("you have control")
        || t.contains("you have the aircraft")
        || t.contains("you have the airplane")
        || t.contains("your controls")
        || t.contains("your aircraft")
        || t.contains("your airplane")
        || t == "you have it"
        || t == "you fly"
        || t == "you fly it"
    {
        return Some(call(TAKE_OVER, json!({})));
    }
    // Distress: fly us out of it.
    if has(t, "recover")
        || has(t, "stabilize")
        || has(t, "stabilise")
        || t.contains("save it")
        || t.contains("save the aircraft")
        || t.contains("save us")
        || t == "help"
        || t == "help me"
    {
        return Some(call("recover", json!({})));
    }
    // Emergency autoland, hand off to the autoland plugin. Cancel first so
    // "cancel/stop the autoland" doesn't match the arm phrases.
    if t.contains("cancel autoland")
        || t.contains("cancel the autoland")
        || t.contains("stop autoland")
        || t.contains("stop the autoland")
        || t.contains("cancel the landing")
    {
        return Some(call("autoland", json!({ "cancel": true })));
    }
    // The NEAREST field, whatever the plan: asked for by name, or the
    // emergency words. A plain "land the aircraft" is the plan's
    // destination (the recovery planner; LEMG 2026-10-05).
    if (has(t, "land") || has(t, "landing") || t.contains("divert"))
        && (t.contains("nearest") || t.contains("closest"))
        || t.contains("emergency landing")
        || t == "land here"
        || t == "land right"
        || t == "land right now"
    {
        return Some(call("autoland", json!({ "nearest": true })));
    }
    if t.contains("land the aircraft")
        || t == "land aircraft"
        // "Now you can land." twice before "Please land the aircraft."
        // (LEGR 2026-10-06); "Bring the aircraft down." (2026-10-04). Exact
        // shapes: "bring us down to 3000" is a descent.
        || t.ends_with("you can land")
        || t == "can you land"
        || t == "bring the aircraft down"
        || t == "bring the plane down"
        || t.contains("land the plane")
        || t.contains("land us")
        // The bare ask, which is what a pilot actually says: "Let's land."
        // (737, 2026-09-17) and "Let's land right now." (A330, 2026-09-20)
        // both peel to these. Exact shapes, so "land at Jerez" (a diversion,
        // not this tool) and any longer sentence still fall through. The
        // normaliser peels a trailing "now", which is what leaves "land
        // right".
        || t == "land"
        || t == "land now"
        || t == "land right"
        || t == "land right now"
        || t == "land here"
        || t.contains("get us down")
        || t.contains("get us on the ground")
        || has(t, "autoland")
        || t.contains("auto land")
        || t.contains("emergency landing")
    {
        return Some(call("autoland", json!({})));
    }
    if t.contains("level off")
        || t.contains("level out")
        || t.contains("level the aircraft")
        || t.contains("wings level")
    {
        return Some(call("level_off", json!({})));
    }
    // "my controls" / "I have the aircraft" hands it fully back (disengages the AP
    // and releases any active recovery).
    // Bare "disengage" means the AP only when no other subject is named:
    // "disengage brakes" rode this into autopilot_off (live 2026-07-20).
    if t.contains("autopilot off")
        || t.contains("cancel autopilot")
        || (has(t, "disengage")
            && !(has(t, "brake") || has(t, "brakes") || has(t, "break") || has(t, "breaks")))
        || t.contains("my controls")
        || t.contains("my aircraft")
        || t.contains("my airplane")
        || t.contains("i have control")
        || t.contains("i have the aircraft")
        || t.contains("i have the airplane")
    {
        return Some(call("autopilot_off", json!({})));
    }

    None
}
