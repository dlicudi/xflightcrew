//! The engines: mixture, props, throttle, reversers, start and shutdown.

use super::*;

/// Smart leaning: an explicit "lean" or a bare mixture adjustment.
pub(super) fn lean_mixture(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // Smart leaning. An explicit "lean ..." (auto/power) must beat the throttle
    // branch ("max power" → throttle), since "lean" means the mixture. A bare
    // mixture adjustment with no target ("adjust the mixture", "mixture for
    // cruise") also means "set it appropriately" → lean_mixture, rather than the
    // LLM guessing a percentage. A number, idle cutoff, or a rich/full/max request
    // falls through to set_mixture below.
    if num.is_none() && !t.contains("cutoff") {
        let lean_word = has(t, "lean");
        let bare_mixture =
            has(t, "mixture") && !t.contains("rich") && !t.contains("full") && !t.contains("max");
        if lean_word || bare_mixture {
            let mode = if t.contains("power") || t.contains("rpm") {
                "power"
            } else {
                "auto"
            };
            return Some(call("lean_mixture", json!({ "mode": mode })));
        }
    }

    None
}

/// The levers: props, throttle and thrust, idle, mixture.
pub(super) fn levers(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // Constant-speed prop levers, before throttle/power (a prop request never
    // means the throttle). "props full forward", "props up", "props to cruise",
    // "prop rpm 1900", "props to 1600", "coarse pitch". Match any token STARTING
    // with "prop" so whisper's "prope"/"propp"/"propeller" all land, without
    // grabbing "appropriate" mid-word. Named modes are the primary interface
    // (robust where an RPM value is transcription-fragile); an explicit RPM still
    // works, and a bare "props" asks which setting rather than missing.
    let is_prop = t
        .split(|c: char| !c.is_ascii_alphanumeric())
        .any(|w| w.starts_with("prop"));
    // "Prop heat on" / "prop de-ice on" is the ice protection, not the lever.
    if is_prop {
        if let Some(c) = anti_ice_call(t) {
            return Some(c);
        }
    }
    if is_prop {
        if let Some(n) = num {
            return Some(call("set_prop", json!({ "rpm": n })));
        }
        // Whole-word matching for the short detent tokens, substring checks
        // misfire ("flow" ⊃ "low", "backup" ⊃ "back", both seen in the logs).
        // "take off" = whisper splitting "takeoff".
        if has(t, "full")
            || has(t, "forward")
            || has(t, "up")
            || has(t, "high")
            || has(t, "max")
            || has(t, "fine")
            || t.contains("takeoff")
            || t.contains("take off")
        {
            return Some(call("set_prop", json!({ "setting": "full" })));
        }
        // "cruz" = whisper mishearing "cruise" (seen live).
        if t.contains("cruise") || has(t, "cruz") || has(t, "climb") {
            return Some(call("set_prop", json!({ "setting": "cruise" })));
        }
        if has(t, "low")
            || has(t, "back")
            || has(t, "down")
            || t.contains("coarse")
            || has(t, "min")
            || has(t, "aft")
        {
            return Some(call("set_prop", json!({ "setting": "low" })));
        }
        // A prop command with no target ("set props"), let the tool ask which
        // setting rather than falling through to a bare miss.
        return Some(call("set_prop", json!({})));
    }

    // throttle / power ("power" is a throttle synonym: "power thirty percent",
    // "40 percent power", "maximum/full throttle", "idle").
    // "Thrust" is the jet pilot's word for the same levers ("reduce thrust
    // to 20 percent", LEMG 2026-10-05), never the reversers or the A/T.
    let thrust = has(t, "thrust")
        && !(has(t, "reverse") || has(t, "reverser") || has(t, "reversers") || has(t, "auto"));
    if has(t, "throttle") || has(t, "power") || thrust {
        if t.contains("maximum") || t.contains("max") || t.contains("full") {
            return Some(call("set_throttle", json!({ "percent": 100 })));
        }
        if t.contains("minimum") || t.contains("min") || has(t, "idle") {
            return Some(call("set_throttle", json!({ "percent": 0 })));
        }
        // Relative verbs: "reduce throttle by ten percent" means minus ten from
        // the current lever, NOT throttle to 10 (a power chop, seen live). A
        // relative verb without "to" is a delta; with "to" it's the target.
        // Past-tense forms are whisper slips on the imperative ("increased
        // powers of 60 percent" heard live), and "of" before the number is the
        // same slip on "to", target, not delta.
        let dec = has(t, "reduce")
            || has(t, "reduced")
            || has(t, "decrease")
            || has(t, "decreased")
            || has(t, "lower")
            || has(t, "lowered")
            || has(t, "back")
            || has(t, "ease")
            || has(t, "pull");
        let inc = has(t, "increase")
            || has(t, "increased")
            || has(t, "add")
            || has(t, "raise")
            || has(t, "raised")
            || has(t, "advance")
            || has(t, "bump")
            || has(t, "push");
        if let Some(n) = num {
            if (dec || inc) && !has(t, "to") && !has(t, "of") {
                return Some(call(
                    "set_throttle",
                    json!({ "delta": if dec { -n } else { n } }),
                ));
            }
            return Some(call("set_throttle", json!({ "percent": n })));
        }
        // Bare "increase power" / "reduce throttle", a nudge, ten percent
        // (both went silent live; a copilot would just add a bit).
        if dec || inc {
            return Some(call(
                "set_throttle",
                json!({ "delta": if dec { -10 } else { 10 } }),
            ));
        }
        // "throttle to cruise" / "cruise power" with no number → a typical cruise
        // setting rather than guessing via the LLM.
        if t.contains("cruise") {
            return Some(call("set_throttle", json!({ "percent": 65 })));
        }
    }
    if has(t, "idle") {
        return Some(call("set_throttle", json!({ "percent": 0 })));
    }

    // mixture: full rich / maximum → 100, idle cutoff → 0, else numeric. (A bare
    // mixture adjustment was already routed to lean_mixture above.) "rich" as a
    // substring so "enrich"/"richen" also go full rich, not lean.
    if has(t, "mixture") {
        if t.contains("maximum") || t.contains("max") || t.contains("full") || t.contains("rich") {
            return Some(call("set_mixture", json!({ "percent": 100 })));
        }
        if t.contains("cutoff") || t.contains("cut off") || has(t, "cut") {
            return Some(call("set_mixture", json!({ "percent": 0 })));
        }
        if let Some(n) = num {
            return Some(call("set_mixture", json!({ "percent": n })));
        }
    }

    None
}

/// Reverse thrust, engine start and shutdown.
pub(super) fn reverse_and_start(line: &Line) -> Option<ToolCall> {
    let t = line.t;

    // Reverse thrust (landing rollout), the levers, not a direction:
    // "reverse heading/course" and "reverse the turn" are navigation talk and
    // must fall through untouched.
    if (has(t, "reverse") || has(t, "reversers"))
        && !(has(t, "heading") || has(t, "course") || has(t, "turn"))
    {
        let stow = has(t, "stow")
            || has(t, "cancel")
            || has(t, "off")
            || t.contains("out of")
            || has(t, "end");
        return Some(call("reverse_thrust", json!({ "on": !stow })));
    }

    // A list named by its title ("engine start checklist", "shutdown
    // checklist") is the list, never the start or the shutdown it describes:
    // the title matcher runs it (the panel's Run button, 2026-10-13).
    if crate::checklist::mentions_checklist(t) {
        return None;
    }

    // fixed verbs
    if (has(t, "start") && (has(t, "engine") || has(t, "engines")))
        || t.contains("engine on")
        || t.contains("engines on")
        || t.contains("start up")
        || t.contains("start her up")
        || t.contains("fire up")
        || t.contains("fire them up")
        || t.contains("spool")
        || has(t, "crank")
    {
        // Name is remapped to the airframe's actual start tool (singular piston
        // vs plural twin/turboprop/jet) in `try_parse`.
        return Some(call("start_engine", json!({})));
    }
    if (has(t, "stop") && (has(t, "engine") || has(t, "engines")))
        || t.contains("engine off")
        || t.contains("engines off")
        // The other word order, which the pilot reached for twice at the
        // Heathrow stand while nothing happened ("turn off engines", 2026-09-17).
        // Anchored on "off <the> engine" rather than a loose off + engine, so
        // "engine bleeds off" and friends stay other tools' words.
        || t.contains("off engine")
        || t.contains("off the engine")
        || t.contains("shut down")
        || has(t, "shutdown")
        || t.contains("secure the engine")
        // "cut the engines" / "kill the engines", both are in the tool's own
        // description of itself, and only one of them parsed.
        || ((has(t, "kill") || has(t, "cut")) && (has(t, "engine") || has(t, "engines")))
    {
        return Some(call("stop_engine", json!({})));
    }

    None
}
