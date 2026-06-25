//! Deterministic command fast-path.
//!
//! The bulk of real commands are regular — `<subject> <number>`, `<system>
//! on/off`, `flaps <detent>`, and a handful of fixed verbs (log-mined from actual
//! usage: ~80%). This parses those straight to tool calls with **no LLM** — no
//! GPU burst, no latency, and it physically can't act on a question or pick the
//! wrong tool. Anything it can't confidently match (compound, relative, fuzzy,
//! novel) returns `None`, and the caller falls back to the LLM.
//!
//! Every candidate is **validated against the live tool schema** before use, so a
//! value this airframe doesn't support (e.g. a flap detent it lacks) fails closed
//! to the LLM rather than executing something wrong. Extend the tables here as the
//! logs surface new recurring phrasings (see `x-copilot analyze`).

use serde_json::{json, Value};

use crate::aircraft::AircraftProfile;
use crate::llm::ToolCall;
use crate::tools::Registry;

/// Markers of a relative/contextual command only the LLM can resolve.
const RELATIVE: &[&str] = &[
    "more", "less", "again", "instead", "other", "same", "undo", "revert",
    "switch it", "a bit", "back to",
];

/// Resolve `transcript` to tool call(s) deterministically, or `None` to fall back
/// to the LLM. Input is expected post-normalization (digits, not number words).
pub fn try_parse(
    transcript: &str,
    registry: &Registry,
    profile: &AircraftProfile,
) -> Option<Vec<ToolCall>> {
    let t = transcript.trim().trim_end_matches(['.', '!', '?', ',']).to_lowercase();
    if t.is_empty() || t.contains(" and ") {
        return None; // empty or compound -> LLM
    }
    if RELATIVE.iter().any(|w| t.contains(w)) {
        return None; // relative/contextual -> LLM
    }
    let c = parse_one(&t)?;
    valid(&c, registry, profile).then_some(vec![c])
}

fn call(name: &str, args: Value) -> ToolCall {
    ToolCall { name: name.to_string(), arguments: args }
}

/// Value of a single number word: `(value, is_multiplier)`. Multipliers are
/// hundred/thousand; everything else is a digit/teen/ten.
fn word_value(w: &str) -> Option<(i64, bool)> {
    let n = match w {
        "zero" | "oh" => 0,
        "one" => 1, "two" => 2, "three" => 3, "four" => 4, "five" => 5,
        "six" => 6, "seven" => 7, "eight" => 8, "nine" => 9, "ten" => 10,
        "eleven" => 11, "twelve" => 12, "thirteen" => 13, "fourteen" => 14,
        "fifteen" => 15, "sixteen" => 16, "seventeen" => 17, "eighteen" => 18,
        "nineteen" => 19, "twenty" => 20, "thirty" => 30, "forty" => 40,
        "fifty" => 50, "sixty" => 60, "seventy" => 70, "eighty" => 80, "ninety" => 90,
        "hundred" => return Some((100, true)),
        "thousand" => return Some((1000, true)),
        _ => return None,
    };
    Some((n, false))
}

/// First number in the text — digits (already-normalized "270") or spoken words
/// ("eighty" → 80, "five hundred" → 500, "two thousand" → 2000). Returns the
/// magnitude; sign (climb/descend) is applied by the caller.
fn number(t: &str) -> Option<i64> {
    let (mut total, mut cur, mut found) = (0i64, 0i64, false);
    for tok in t.split(|c: char| !c.is_ascii_alphanumeric()) {
        if tok.is_empty() {
            continue;
        }
        if let Ok(d) = tok.parse::<i64>() {
            cur = if cur == 0 { d } else { cur * 10 + d };
            found = true;
        } else if let Some((val, mult)) = word_value(tok) {
            if mult && val == 100 {
                cur = cur.max(1) * 100;
            } else if mult {
                total += cur.max(1) * 1000;
                cur = 0;
            } else {
                cur += val;
            }
            found = true;
        } else if found {
            break; // number run ended
        }
    }
    found.then_some(total + cur)
}

/// First decimal number (for the altimeter, e.g. 29.92).
fn first_float(t: &str) -> Option<f64> {
    t.split(|c: char| !(c.is_ascii_digit() || c == '.'))
        .find_map(|w| if w.contains('.') { w.parse::<f64>().ok() } else { None })
}

/// Whole-word membership (so "on" doesn't match inside "engine").
fn has(t: &str, word: &str) -> bool {
    t.split(|c: char| !c.is_ascii_alphanumeric()).any(|w| w == word)
}

/// on/off (+ synonyms) as a whole word; `None` if neither is present.
fn onoff(t: &str) -> Option<bool> {
    if has(t, "on") || has(t, "open") {
        Some(true)
    } else if has(t, "off") || has(t, "close") || has(t, "closed") {
        Some(false)
    } else {
        None
    }
}

fn parse_one(t: &str) -> Option<ToolCall> {
    let num = number(t);

    // flaps up / ten / twenty / full
    if has(t, "flaps") || has(t, "flap") {
        let notch = if has(t, "up") {
            "up".to_string()
        } else if has(t, "full") {
            "full".to_string()
        } else if let Some(n) = num {
            n.to_string()
        } else {
            return None;
        };
        return Some(call("set_flaps", json!({ "notch": notch })));
    }

    // throttle / power
    if has(t, "throttle") {
        if let Some(n) = num {
            return Some(call("set_throttle", json!({ "percent": n })));
        }
    }
    if t.contains("maximum power") || t.contains("max power") || t.contains("full power") {
        return Some(call("set_throttle", json!({ "percent": 100 })));
    }
    if has(t, "idle") || t.contains("minimum power") || t.contains("min power") {
        return Some(call("set_throttle", json!({ "percent": 0 })));
    }

    // mixture (numeric only; rich/lean left to the LLM)
    if has(t, "mixture") {
        if let Some(n) = num {
            return Some(call("set_mixture", json!({ "percent": n })));
        }
    }

    // heading 270 / heading north
    if has(t, "heading") {
        if let Some(n) = num {
            return Some(call("set_heading", json!({ "heading": n })));
        }
        let dir = if has(t, "north") {
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

    // vertical speed / climb / descend (when not an altitude command)
    if (has(t, "vertical") || has(t, "climb") || has(t, "descend") || has(t, "descent"))
        && !has(t, "altitude")
        && !has(t, "maintain")
    {
        if let Some(n) = num {
            let down = has(t, "descend") || has(t, "descent") || has(t, "down");
            let fpm = if down { -n.abs() } else { n.abs() };
            return Some(call("set_vertical_speed", json!({ "fpm": fpm })));
        }
    }

    // altitude / maintain
    if has(t, "altitude") || has(t, "maintain") {
        if let Some(n) = num {
            return Some(call("set_altitude", json!({ "altitude": n })));
        }
    }

    // flood lights (on/off or a percent). Substring match so "floodlights" glued
    // into one word by STT also resolves.
    if t.contains("flood") {
        if let Some(n) = num {
            return Some(call("set_flood_lights", json!({ "percent": n })));
        }
        if let Some(on) = onoff(t) {
            return Some(call("set_flood_lights", json!({ "on": on })));
        }
        return None;
    }

    // <system> on/off
    if let Some(on) = onoff(t) {
        for (kw, key) in [
            ("landing", "landing"),
            ("taxi", "taxi"),
            ("beacon", "beacon"),
            ("strobe", "strobe"),
            ("strobes", "strobe"),
        ] {
            if has(t, kw) {
                return Some(call("set_lights", json!({ "light": key, "on": on })));
            }
        }
        // nav LIGHT (require the word light/lights so "nav one 109" stays radio)
        if has(t, "nav") && (has(t, "light") || has(t, "lights")) {
            return Some(call("set_lights", json!({ "light": "nav", "on": on })));
        }
        for (kw, key) in [
            ("avionics", "avionics"),
            ("battery", "battery"),
            ("alternator", "alternator"),
            ("generator", "generator"),
            ("master", "master"),
        ] {
            if has(t, kw) {
                return Some(call("set_electrical", json!({ "system": key, "on": on })));
            }
        }
        if has(t, "electrical") || has(t, "electrics") {
            return Some(call("set_electrical", json!({ "system": "all", "on": on })));
        }
        if t.contains("fuel pump") {
            return Some(call("set_fuel_pump", json!({ "on": on })));
        }
        if has(t, "pitot") {
            return Some(call("set_pitot_heat", json!({ "on": on })));
        }
        if has(t, "carb") || has(t, "carburetor") {
            return Some(call("set_carb_heat", json!({ "on": on })));
        }
    }

    // fixed verbs
    if (has(t, "start") && has(t, "engine"))
        || t.contains("engine on")
        || t.contains("engines on")
        || has(t, "crank")
    {
        return Some(call("start_engine", json!({})));
    }
    if (has(t, "stop") && has(t, "engine"))
        || t.contains("engine off")
        || t.contains("engines off")
        || t.contains("shut down")
        || has(t, "shutdown")
        || (has(t, "kill") && has(t, "engine"))
    {
        return Some(call("stop_engine", json!({})));
    }
    if has(t, "gyro") {
        // align/sync/set gyro — also catches mishears like "think/zinc gyro".
        return Some(call("align_gyro", json!({})));
    }
    if t.contains("level off") || t.contains("level out") || t.contains("level the aircraft")
        || t.contains("wings level")
    {
        return Some(call("level_off", json!({})));
    }
    if t.contains("autopilot off") || t.contains("cancel autopilot") || has(t, "disengage") {
        return Some(call("autopilot_off", json!({})));
    }

    // parking brake (STT often hears "brake" as "break")
    if t.contains("parking brake") || t.contains("park brake")
        || t.contains("parking break") || t.contains("park break")
    {
        let release = has(t, "release") || has(t, "released") || has(t, "off");
        return Some(call("set_parking_brake", json!({ "on": !release })));
    }

    // fuel selector / tank
    if t.contains("fuel selector") || t.contains("fuel tank") {
        let pos = if has(t, "both") {
            "both"
        } else if has(t, "left") {
            "left"
        } else if has(t, "right") {
            "right"
        } else if has(t, "off") {
            "off"
        } else {
            return None;
        };
        return Some(call("set_fuel_selector", json!({ "position": pos })));
    }

    // transponder / squawk
    if has(t, "squawk") || has(t, "transponder") {
        if has(t, "standby") {
            return Some(call("set_transponder_mode", json!({ "mode": "standby" })));
        }
        if has(t, "ident") {
            return Some(call("set_transponder_mode", json!({ "mode": "ident" })));
        }
        if has(t, "vfr") {
            return Some(call("set_transponder", json!({ "code": 1200 })));
        }
        if let Some(n) = num {
            return Some(call("set_transponder", json!({ "code": n })));
        }
    }

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
            return None;
        };
        return Some(call("set_pitch_trim", json!({ "action": action })));
    }

    // altimeter / barometer
    if has(t, "altimeter") || has(t, "barometer") || has(t, "baro") {
        if let Some(f) = first_float(t) {
            return Some(call("set_barometer", json!({ "inhg": f })));
        }
    }

    None
}

/// The tool exists, every arg key is in its schema, and any enum-typed string arg
/// is a valid member of that airframe's schema. Fails closed → LLM fallback.
fn valid(c: &ToolCall, registry: &Registry, _profile: &AircraftProfile) -> bool {
    let Some(tool) = registry.get(&c.name) else {
        return false;
    };
    let schema = tool.parameters();
    let props = schema.get("properties").and_then(|p| p.as_object());
    let Some(args) = c.arguments.as_object() else {
        return true;
    };
    for (k, v) in args {
        let Some(props) = props else {
            return false; // args given but the tool takes none
        };
        let Some(pschema) = props.get(k) else {
            return false; // unknown argument
        };
        if let (Some(en), Some(s)) = (pschema.get("enum").and_then(|e| e.as_array()), v.as_str()) {
            if !en.iter().any(|e| e.as_str() == Some(s)) {
                return false; // value not valid for this airframe
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft;
    use crate::tools::Registry;

    fn setup() -> (Registry, aircraft::AircraftProfile) {
        let profile = aircraft::load("cessna_172sp").expect("cessna profile");
        (Registry::for_aircraft(&profile), profile)
    }

    fn one(t: &str) -> Option<(String, Value)> {
        let (reg, prof) = setup();
        try_parse(t, &reg, &prof).map(|mut v| {
            let c = v.remove(0);
            (c.name, c.arguments)
        })
    }

    #[test]
    fn common_commands_resolve_deterministically() {
        assert_eq!(one("flaps ten").unwrap().0, "set_flaps");
        assert_eq!(one("flaps up").unwrap().1, json!({"notch":"up"}));
        assert_eq!(one("flaps full").unwrap().1, json!({"notch":"full"}));
        assert_eq!(one("throttle eighty percent"), Some(("set_throttle".into(), json!({"percent":80}))));
        assert_eq!(one("maximum power").unwrap().1, json!({"percent":100}));
        assert_eq!(one("mixture 70 percent").unwrap().0, "set_mixture");
        assert_eq!(one("heading 270").unwrap().1, json!({"heading":270}));
        assert_eq!(one("turn heading south").unwrap().1, json!({"heading":180}));
        assert_eq!(one("descend 500").unwrap().1, json!({"fpm":-500}));
        assert_eq!(one("climb 500").unwrap().1, json!({"fpm":500}));
        assert_eq!(one("avionics on").unwrap(), ("set_electrical".into(), json!({"system":"avionics","on":true})));
        assert_eq!(one("landing lights off").unwrap(), ("set_lights".into(), json!({"light":"landing","on":false})));
        assert_eq!(one("strobes off").unwrap().1, json!({"light":"strobe","on":false}));
        assert_eq!(one("start the engine").unwrap().0, "start_engine");
        assert_eq!(one("align gyro").unwrap().0, "align_gyro");
        assert_eq!(one("think gyro").unwrap().0, "align_gyro"); // mishear handled
        assert_eq!(one("level off").unwrap().0, "level_off");
        assert_eq!(one("autopilot off").unwrap().0, "autopilot_off");
        assert_eq!(one("parking brake set").unwrap(), ("set_parking_brake".into(), json!({"on":true})));
    }

    #[test]
    fn stt_mishears_and_glued_words_still_resolve() {
        // Live-flight gaps: "floodlights" glued, "brake" heard as "break".
        assert_eq!(one("floodlights on").unwrap().0, "set_flood_lights");
        assert_eq!(one("turn the floodlights on").unwrap().1, json!({"on":true}));
        assert_eq!(one("parking break off").unwrap(), ("set_parking_brake".into(), json!({"on":false})));
        assert_eq!(one("parking break set").unwrap().1, json!({"on":true}));
    }

    #[test]
    fn ambiguous_falls_back_to_llm() {
        assert_eq!(one("a bit more flaps"), None); // relative
        assert_eq!(one("taxi and nav lights on"), None); // compound
        assert_eq!(one("switch it back to both"), None); // relative
        assert_eq!(one("nice weather today"), None); // not a command
        assert_eq!(one("what's our heading"), None); // question
    }

    #[test]
    fn invalid_values_fall_back() {
        // The Cessna 172 has no 25-degree detent — must not fabricate it.
        assert_eq!(one("flaps 25"), None);
    }
}
