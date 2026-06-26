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
    if t.is_empty() || is_compound(&t) {
        return None; // empty or a real multi-action compound -> LLM
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
        } else if tok == "and" && found {
            // spoken connector inside a number ("two hundred and forty")
        } else if found {
            break; // number run ended
        }
    }
    found.then_some(total + cur)
}

/// True if `t` is a real multi-action compound ("taxi and nav lights on") — an
/// "and" whose neighbours aren't both number words. The "and" inside a spoken
/// number ("two hundred and forty") is NOT a compound, so it stays on the
/// fast-path.
fn is_compound(t: &str) -> bool {
    let toks: Vec<&str> = t.split_whitespace().collect();
    let numish = |w: &str| {
        let w = w.trim_matches(|c: char| !c.is_ascii_alphanumeric());
        w.chars().all(|c| c.is_ascii_digit()) && !w.is_empty()
            || word_value(&w.to_ascii_lowercase()).is_some()
    };
    for (i, w) in toks.iter().enumerate() {
        if *w == "and" {
            // "climb and maintain 7000" is one altitude command, not a compound.
            if toks.get(i + 1).is_some_and(|n| *n == "maintain") {
                continue;
            }
            let prev_num = i.checked_sub(1).and_then(|j| toks.get(j)).is_some_and(|p| numish(p));
            let next_num = toks.get(i + 1).is_some_and(|n| numish(n));
            if !(prev_num && next_num) {
                return true; // "and" joins non-numbers → genuine compound
            }
        }
    }
    false
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

    // throttle / power ("power" is a throttle synonym: "power thirty percent",
    // "40 percent power", "maximum/full throttle", "idle").
    if has(t, "throttle") || has(t, "power") {
        if t.contains("maximum") || t.contains("max") || t.contains("full") {
            return Some(call("set_throttle", json!({ "percent": 100 })));
        }
        if t.contains("minimum") || t.contains("min") || has(t, "idle") {
            return Some(call("set_throttle", json!({ "percent": 0 })));
        }
        if let Some(n) = num {
            return Some(call("set_throttle", json!({ "percent": n })));
        }
    }
    if has(t, "idle") {
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

    // climb/descend TO a target AT a rate -> set_climb (AP captures & levels off)
    if has(t, "climb") || has(t, "descend") || has(t, "descent") || has(t, "altitude") {
        if let Some((before, after)) = t.split_once(" at ") {
            if let (Some(target), Some(rate)) = (
                before.rsplit_once(" to ").and_then(|(_, a)| number(a)),
                number(after),
            ) {
                return Some(call("set_climb", json!({ "altitude": target, "fpm": rate })));
            }
        }
    }

    // climb / descend / altitude — one number, routed to an altitude TARGET or a
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
                && (t.contains(" to ") || has(t, "altitude") || has(t, "maintain") || n.abs() > 2500);
            if is_target {
                return Some(call("set_altitude", json!({ "altitude": n.abs() })));
            }
            let down = has(t, "descend") || has(t, "descent") || has(t, "down")
                || has(t, "minus") || has(t, "negative");
            let fpm = if down { -n.abs() } else { n.abs() };
            return Some(call("set_vertical_speed", json!({ "fpm": fpm })));
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
        // "power" is a throttle synonym, and "maximum throttle"/"% power" forms.
        assert_eq!(one("power thirty percent").unwrap(), ("set_throttle".into(), json!({"percent":30})));
        assert_eq!(one("40 percent power").unwrap().1, json!({"percent":40}));
        assert_eq!(one("maximum throttle").unwrap().1, json!({"percent":100}));
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
    fn climb_to_target_at_rate_and_spoken_number_and() {
        // climb/descend TO a target AT a rate -> set_climb
        assert_eq!(
            one("climb to seven thousand at five hundred").unwrap(),
            ("set_climb".into(), json!({"altitude":7000,"fpm":500}))
        );
        assert_eq!(
            one("descend to 2000 at a thousand").unwrap(),
            ("set_climb".into(), json!({"altitude":2000,"fpm":1000}))
        );
        // "and" inside a spoken number is NOT a compound — stays on the fast-path.
        assert_eq!(one("heading two hundred and forty").unwrap().1, json!({"heading":240}));
        // but a genuine multi-action compound still goes to the LLM.
        assert_eq!(one("flaps ten and gear up"), None);
    }

    #[test]
    fn explicit_vertical_speed_is_never_an_altitude_target() {
        // Regression: "vertical speed" commands were grabbed by set_altitude when
        // they said "to X", or named a rate above the 2500 fpm disambiguation cut.
        let vs = |s: &str| one(s).unwrap();
        assert_eq!(vs("vertical speed descent three thousand"), ("set_vertical_speed".into(), json!({"fpm":-3000})));
        assert_eq!(vs("vertical speed 3000 minus"), ("set_vertical_speed".into(), json!({"fpm":-3000})));
        assert_eq!(vs("set vertical speed to minus one thousand"), ("set_vertical_speed".into(), json!({"fpm":-1000})));
        assert_eq!(vs("vertical speed negative five hundred"), ("set_vertical_speed".into(), json!({"fpm":-500})));
        assert_eq!(vs("vertical speed to seven hundred"), ("set_vertical_speed".into(), json!({"fpm":700})));
        // Bare climb/descend WITHOUT "vertical" still disambiguates to an altitude
        // target on "to X" or a value too big to be a rate.
        assert_eq!(one("climb 7000").unwrap().0, "set_altitude");
        assert_eq!(one("climb to 7000").unwrap(), ("set_altitude".into(), json!({"altitude":7000})));
    }

    #[test]
    fn altitude_target_vs_vertical_speed_rate() {
        // "to X" / big number / "maintain" → altitude target
        assert_eq!(one("descend to one thousand two hundred feet").unwrap(), ("set_altitude".into(), json!({"altitude":1200})));
        assert_eq!(one("climb to seven thousand").unwrap(), ("set_altitude".into(), json!({"altitude":7000})));
        assert_eq!(one("climb and maintain seven thousand").unwrap(), ("set_altitude".into(), json!({"altitude":7000})));
        assert_eq!(one("descend and maintain two thousand").unwrap(), ("set_altitude".into(), json!({"altitude":2000})));
        assert_eq!(one("climb 7000").unwrap().0, "set_altitude"); // too big to be a VS rate
        // no "to", in-range number → vertical-speed rate
        assert_eq!(one("descend 500").unwrap(), ("set_vertical_speed".into(), json!({"fpm":-500})));
        assert_eq!(one("climb at five hundred").unwrap(), ("set_vertical_speed".into(), json!({"fpm":500})));
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
