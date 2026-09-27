//! Deterministic command fast-path.
//!
//! The bulk of real commands are regular — `<subject> <number>`, `<system>
//! on/off`, `flaps <detent>`, and a handful of fixed verbs (log-mined from actual
//! usage: ~80%). This parses those straight to tool calls deterministically — no
//! GPU burst, no latency, and it can't pick the wrong tool. Anything it can't
//! confidently match (compound, relative, fuzzy, novel) returns `None`, and the
//! caller treats the utterance as a miss. Note this matches on shape, not intent: a
//! question like "is our heading 270?" has the same keyword+number form as a
//! command, so the caller must withhold questions from the fast-path (the answerers
//! arbitrates act-vs-answer) before calling in.
//!
//! Every candidate is **validated against the live tool schema** (the tool exists
//! and the arg keys are known). An out-of-range *value* for the airframe (e.g. a
//! flap detent it lacks) is NOT rejected here — it's routed to the tool, which
//! replies with the valid options ("I have up, approach, down"), far more useful
//! than a bare "Say again?". Extend the tables here as the logs surface new
//! recurring phrasings (see `xvl analyze`).

use serde_json::{json, Value};

use crate::aircraft::AircraftProfile;
use crate::phrase::word_value;
use crate::tools::Registry;
use crate::tools::ToolCall;

/// Markers of a relative/contextual command the dialogue stack resolves, not the grammar.
const RELATIVE: &[&str] = &[
    "more",
    "less",
    "again",
    "instead",
    "other",
    "same",
    "undo",
    "revert",
    "switch it",
    "a bit",
    "back to",
];

/// Resolve `transcript` to tool call(s) deterministically, or `None` to fall back
/// to the LLM. Input is expected post-normalization (digits, not number words).
/// Lowercase, strip trailing punctuation, and peel off conversational filler /
/// politeness so natural phrasings reduce to the bare command or question the
/// matchers know: "could you start the engines" → "start the engines", "okay now
/// flaps ten please" → "flaps ten". Used by both the command grammar and (in the
/// command loop) the question/state path, so politeness never causes a miss.
pub fn normalize(transcript: &str) -> String {
    // Leading wrappers, peeled repeatedly (longest first so "could you please"
    // goes before "could you"). Conservative: only filler/politeness, never a
    // command verb like "set"/"turn"/"give".
    const LEAD: &[&str] = &[
        "could you please",
        "can you please",
        "would you please",
        "would you mind",
        "could you",
        "can you",
        "would you",
        "will you",
        "i would like you to",
        "id like you to",
        "i would like to",
        "id like to",
        "i would like",
        "id like",
        "i want you to",
        "i need you to",
        "i want to",
        "i need to",
        "lets go ahead and",
        "go ahead and",
        "lets",
        "let us",
        "please",
        "okay",
        "ok",
        "alright",
        "all right",
        "hey",
        "uh",
        "um",
        "er",
        "well",
        "so",
        "now",
        "yeah",
        "yep",
        "yes",
        "gimme",
        "give me",
        "just",
        // Narrative connectives whisper keeps from the pilot thinking aloud
        // ("And then, fly heading 120." missed live twice on 2026-07-09). A
        // leading "and" is never load-bearing; mid-sentence "and" is untouched
        // (compound detection runs after peeling).
        "and then",
        "and",
        "then",
    ];
    // Trailing politeness, peeled the same way.
    const TRAIL: &[&str] = &[
        "please",
        "for me",
        "now",
        "thanks",
        "thank you",
        "would you",
        "okay",
        "ok",
    ];

    // Drop apostrophes: whisper spuriously fits them ("engine's on" for "engines
    // on", "flap's" for "flaps"), which breaks substring matches like "engines
    // on". Both the ASCII and curly forms. (Question detection already accepts the
    // stripped forms — "whats"/"hows" are openers alongside "what's"/"how's".)
    let mut s = transcript
        .trim()
        .trim_end_matches(['.', '!', '?', ','])
        .trim()
        .to_lowercase()
        .replace(['\'', '\u{2019}'], "");
    // Whisper mishearings: fixed homophone slips it makes on short cockpit
    // commands, corrected before matching. "electrics on" comes back as "electric
    // zone(s)" (the "s on" glues into "zone(s)") — nobody commands "electric
    // zones", so the rewrite is safe. Plural before singular.
    // Multi-word homophone slips — distinctive enough for a plain replace.
    for (heard, meant) in [
        ("electric system zones", "electrics on"),
        ("electric systems zones", "electrics on"),
        ("electric zones", "electrics on"),
        ("electric zone", "electrics on"),
        // ...and the same glue heard as a word: "Electric song?" (live 2026-07-11).
        ("electric song", "electrics on"),
        // "pitot" defeats whisper: all of these observed live for "pitot heat"
        // (apostrophes are already stripped, so "Pito's" arrives as "pitos").
        // Nobody says "pilot heat" or "it was heat" meaning anything else.
        ("pitos heat", "pitot heat"),
        ("pito heat", "pitot heat"),
        ("pilot heat", "pitot heat"),
        ("it was heat", "pitot heat"),
        ("a bit of heat", "pitot heat"),
        // "talk me down" garbles, both observed live back to back.
        ("top me down", "talk me down"),
        ("toadown", "talk me down"),
        // "my throttle" (the speed-hold release) defeats whisper: both observed
        // live while the pilot tried five times to take the lever back.
        ("mic throttle", "my throttle"),
        ("my throat", "my throttle"),
        // "idle" heard as "idol" after a power word ("Power Idol" seen live);
        // guarded by the leading word so a genuine "idol" is never touched.
        ("power idol", "power idle"),
        ("throttle idol", "throttle idle"),
        // "parking brake" split into "park and brake" (seen live: "Park and
        // brake release."). Nobody commands "park and brake" otherwise.
        ("park and brake", "parking brake"),
        // Spoken "Q-N-H" (cue-en-aitch): whisper hears the "en" as "and"/"&", so
        // "set QNH 1013" arrives as "set q and h 1013" or "set q&h 1013" (both
        // seen live 2026-07-12) and the "qnh" whole-word check never fires. Also
        // the letter-by-letter spelling. Nobody says any of these otherwise.
        ("q and h", "qnh"),
        ("q&h", "qnh"),
        ("q n h", "qnh"),
        // Whisper's renderings of "taxi for me" ("Tax you for me?" at 0.99
        // confidence, "Okay. Taxi foamy?" — both live 2026-07-20). Nobody
        // says either otherwise.
        ("tax you for me", "taxi for me"),
        ("taxi foamy", "taxi for me"),
        // Whisper splits the compound; nobody says "back track" otherwise.
        ("back track", "backtrack"),
        // "engines on" heard with a ghost copula ("Engine is on." live
        // 2026-07-22 — said as the start command, missed as a statement).
        ("engine is on", "engines on"),
        // "electrical systems on" mangles (live 2026-07-20/22): the noun
        // slips to "electronics", the trailing "on" to "zone".
        ("electronics systems", "electrical systems"),
        ("electrical system zone", "electrical systems on"),
        // "my controls" — the universal hand-back — heard as "Mind controls."
        // (737, 2026-09-20). A missed hand-back leaves an automation flying
        // an aircraft the pilot has already taken, so this one is not
        // optional.
        ("mind controls", "my controls"),
        // "flight plan" garbles on the plan phrasings: "Resume the fight
        // plan." (737, 2026-09-20), "That's the wrong flag plan." and "Fly
        // the flight man." (172, 2026-09-17). None of the three is a phrase.
        ("fight plan", "flight plan"),
        ("flag plan", "flight plan"),
        ("flight man", "flight plan"),
        // "autoland" splits and slips: "All to land." / "Cancel all to
        // land?" / "Cancel Autolamp." (Baron, 2026-09-16, the pilot arming
        // and cancelling). "All to land" is not English anyone speaks.
        ("all to land", "autoland"),
        ("autolamp", "autoland"),
        // The load sheet, verb pluralised: "Loads the aircraft." (737,
        // 2026-09-22). The whole phrase, so a bare "loads" is untouched.
        ("loads the aircraft", "load the aircraft"),
    ] {
        s = s.replace(heard, meant);
    }
    // Whole-word single-token slips: "crops"/"crop" → "props"/"prop" (whisper
    // hears the p as a hard c; seen live on "Crops, cruise"). Pad with spaces so
    // only the standalone word is rewritten, never a substring.
    {
        // "powers" is whisper's plural slip on "power" ("increased powers of 60
        // percent" heard live) — nobody commands "powers".
        let fixed = format!(" {s} ")
            .replace(" crops ", " props ")
            .replace(" crop ", " prop ")
            // "knots" heard as "notes" ("Taxi at ten notes.", 737,
            // 2026-09-20) — the only unit a cockpit number carries, and
            // nobody dictates notes to a copilot.
            .replace(" notes ", " knots ")
            .replace(" powers ", " power ");
        s = fixed.trim().to_string();
    }
    loop {
        let before = s.clone();
        for w in LEAD {
            if let Some(rest) = s.strip_prefix(w) {
                if rest.is_empty() || rest.starts_with([' ', ',']) {
                    s = rest.trim_start_matches([' ', ',']).to_string();
                }
            }
        }
        for w in TRAIL {
            if let Some(head) = s.strip_suffix(w) {
                if head.is_empty() || head.ends_with([' ', ',']) {
                    s = head.trim_end_matches([' ', ',']).to_string();
                }
            }
        }
        if s == before {
            break;
        }
    }
    s
}

/// If the utterance is a recognizable command whose CAPABILITY this airframe
/// lacks, return an honest spoken decline — so the caller says that instead of a
/// bare "Say again?". The classic case: an autopilot command ("climb to 4000",
/// "heading 270", "autopilot on") on an aircraft with no AP (the AWX DC-3).
/// `None` = nothing to decline; fall through to the normal miss handling.
pub fn declined(transcript: &str, registry: &Registry) -> Option<String> {
    let t = normalize(transcript);
    if t.is_empty() {
        return None;
    }
    // Autopilot family. Guard the ambiguous words ("heading"/"altitude" are also
    // questions) — but questions are handled before this is reached.
    let ap_intent = t.contains("autopilot")
        || t.contains("auto pilot")
        || t.contains("climb to")
        || t.contains("descend to")
        || t.contains("level off")
        || t.contains("nav mode")
        || t.contains("vertical speed")
        || t.contains("heading")
        || t.contains("altitude");
    if ap_intent && registry.get("set_heading").is_none() && registry.get("set_altitude").is_none()
    {
        return Some("I don't have an autopilot on this aircraft.".to_string());
    }
    // Carb heat on a fuel-injected engine (the 172SP's IO-360, the SR22, the
    // Baron): the words are a real command, the airframe just has no
    // carburettor. Name that rather than fall through to "Say again?".
    if (t.contains("carb") || t.contains("carburetor") || t.contains("carburettor"))
        && registry.get("set_carb_heat").is_none()
    {
        return Some("This engine is fuel injected, so there's no carburettor heat.".to_string());
    }
    // Engine start/stop on an airframe that declares no engine tool (the
    // S-76C: `engine: false`, because the twist grips are the pilot's). The
    // words are a real command and the profile is deliberate, so say which it
    // is. At the pad on 2026-09-17 the pilot asked four ways in eleven
    // seconds — "start engines", "start the engines", "engine's on", "engines
    // on" — and every one went into the silence of a miss.
    let engine_intent = (has(&t, "start") || has(&t, "stop") || has(&t, "shut"))
        && (has(&t, "engine") || has(&t, "engines"))
        || t == "engines on"
        || t == "engine on"
        || t == "engines off"
        || t == "engine off";
    if engine_intent
        && registry.get("start_engine").is_none()
        && registry.get("start_engines").is_none()
        && registry.get("stop_engine").is_none()
        && registry.get("stop_engines").is_none()
    {
        return Some(
            "The engines are yours on this aircraft. I have no start or shutdown here.".to_string(),
        );
    }
    None
}

pub fn try_parse(
    transcript: &str,
    registry: &Registry,
    profile: &AircraftProfile,
) -> Option<Vec<ToolCall>> {
    let t = normalize(transcript);
    // Full auto (the whole flight from cold) and the circuit are each ONE
    // intent even though their phrasings carry "and" ("start up and fly", "take
    // off and land") — checked on the raw text BEFORE the compound gate would
    // split them. Full auto first: "start up and fly a circuit" contains
    // "circuit" and must not fall to the circuit rule.
    // A new flight on a final: its hand-fly clause carries "and", and its
    // runway words must never reach the mission grammar.
    if let Some(c) = approach_setup_call(&t) {
        return valid(&c, registry, profile).then_some(vec![c]);
    }
    if let Some(c) = flight_plan_call(&t) {
        // A helicopter flies the plan on its own route follower: full auto
        // is withheld there. Stopping it is a hold.
        if registry.get("hover").is_some() && registry.get(&c.name).is_none() {
            let off = c.arguments.get("off").and_then(|v| v.as_bool()) == Some(true);
            return Some(vec![if off {
                call("hover", json!({ "action": "hold" }))
            } else {
                call("hover", json!({ "action": "route" }))
            }]);
        }
        return Some(vec![c]);
    }
    // "Load the fuel and payload" carries "and" too, and is one intent.
    if let Some(c) = load_aircraft_call(&t) {
        return valid(&c, registry, profile).then_some(vec![c]);
    }
    if let Some(c) = full_auto_call(&t) {
        return Some(vec![c]);
    }
    if let Some(c) = circuit_call(&t) {
        // A helicopter flies its own circuit on the hover loop: the fixed-
        // wing mission is withheld there. Stopping it is a hold.
        if registry.get("hover").is_some() && registry.get(&c.name).is_none() {
            let off = c.arguments.get("off").and_then(|v| v.as_bool()) == Some(true);
            let right = c.arguments.get("right").and_then(|v| v.as_bool()) == Some(true);
            return Some(vec![if off {
                call("hover", json!({ "action": "hold" }))
            } else {
                call("hover", json!({ "action": "circuit", "right": right }))
            }]);
        }
        return Some(vec![c]);
    }
    // A helicopter's hover and set-down. Only where the airframe has the
    // tool: "set it down" means nothing to an aeroplane.
    if registry.get("hover").is_some() {
        if let Some(c) = hover_call(&t) {
            return Some(vec![c]);
        }
    }
    // Fly a real published procedure ("fly the ILS", "shoot the approach", "fly
    // the SID") — request-shaped, checked before the compound gate and never on
    // an ATC readback ("cleared ILS approach ...").
    if let Some(c) = procedure_call(&t) {
        return Some(vec![c]);
    }
    if t.is_empty() || is_compound(&t) {
        return None; // empty or a real multi-action compound -> miss
    }
    if RELATIVE.iter().any(|w| t.contains(w)) {
        return None; // relative/contextual -> the dialogue stack
    }
    let mut c = parse_one(&t)?;
    // Engine start/stop come in a singular flavor (piston single: `start_engine`/
    // `stop_engine`) and a plural one (twin/turboprop/jet: `start_engines`/
    // `stop_engines`). The grammar emits the singular; swap to the plural when
    // that's the tool this airframe actually registered.
    for (singular, plural) in [
        ("start_engine", "start_engines"),
        ("stop_engine", "stop_engines"),
    ] {
        if c.name == singular && registry.get(singular).is_none() && registry.get(plural).is_some()
        {
            c.name = plural.to_string();
        }
    }
    // "set speed 60" / "change speed to 60 knots" resolves to the AP airspeed
    // select — which a C172-class panel doesn't register. The pilot still
    // means "fly this speed": hand the target to the speed-hold governor when
    // that's the tool this airframe has. Ten consecutive dead ends on the
    // 2026-07-09 final approach. Only the absolute forms map (knots/preset);
    // a relative "reduce by 20" needs the AP bug and stays unmapped.
    // A helicopter's speed is the hover loop's (a departure's), and the AP
    // speed select and the governor are not on it: "speed fifty knots" in a
    // departure went nowhere (EDDV, 2026-09-23).
    if c.name == "set_speed"
        && registry.get("set_speed").is_none()
        && registry.get("hover").is_some()
    {
        if let Some(k) = c.arguments.get("knots").and_then(|v| v.as_f64()) {
            return Some(vec![call(
                "hover",
                json!({ "action": "speed", "speed_kts": k }),
            )]);
        }
    }
    if c.name == "set_speed"
        && registry.get("set_speed").is_none()
        && registry.get("hold_speed").is_some()
        && (c.arguments.get("knots").is_some() || c.arguments.get("preset").is_some())
    {
        c.name = "hold_speed".to_string();
    }
    valid(&c, registry, profile).then_some(vec![c])
}

/// "Lift to a hover", "hover at fifteen feet", "lift off"; "set it down",
/// "set us down", "put it down". A height is feet.
fn hover_call(t: &str) -> Option<ToolCall> {
    // The destination, never here: "land at the airport", "not here, land at
    // the airport", "take us to the pad" (LIME 2026-09-25, where the words
    // earned nothing and the set-down went on).
    let pad_word = ["airport", "pad", "helipad", "heliport", "destination"]
        .iter()
        .any(|w| has(t, w));
    if pad_word
        && !t.contains("light")
        && ["land", "take us to", "fly to", "go to", "fly us to"]
            .iter()
            .any(|p| t.contains(p))
    {
        return Some(call("hover", json!({ "action": "pad" })));
    }
    // "Land" on a helicopter is the set-down: autoland is withheld there, and
    // "land the aircraft" / "let's land" went nowhere at the S-76's first
    // hover (2026-09-22). Whole phrases only: "landing light" and "landing
    // gear" are other tools.
    // "Land the aircraft": the pad when one is in reach, else here (the
    // tool decides). "Land here", "land now": here.
    let land = t == "land"
        || [
            "land the aircraft",
            "land the helicopter",
            "land it",
            "land us",
            "lets land",
            "let's land",
        ]
        .iter()
        .any(|p| t.contains(p))
        // Whisper's "land 3 helicopter?" for "land the helicopter" (EDDV,
        // 2026-09-23): a land word and the airframe's name is the request.
        || (t.starts_with("land ") && t.contains("helicopter"));
    let here = t.contains("land here") || t.contains("land now");
    if land && !here {
        return Some(call("hover", json!({ "action": "land" })));
    }
    let land = here;
    // "Willis on the ground" is whisper's "put us on the ground".
    let down = land
        || t.contains("willis on the ground")
        || [
            "set it down",
            "set us down",
            "put it down",
            "put us down",
            "set down",
            "put it on the ground",
            "put us on the ground",
        ]
        .iter()
        .any(|p| t.contains(p));
    if down {
        return Some(call("hover", json!({ "action": "down" })));
    }
    // Out of the hover into forward flight. "Take off" here is the
    // helicopter's own: the fixed-wing takeoff is withheld on rotorcraft.
    if [
        "take off",
        "takeoff",
        "depart",
        "transition",
        "forward flight",
    ]
    .iter()
    .any(|p| t.contains(p))
        && !t.contains("clear")
    {
        let mut args = json!({ "action": "depart" });
        if let Some(n) = number(t) {
            // "climb to five hundred" vs "at forty knots": a knots word
            // makes it the speed.
            if t.contains("knot") {
                args["speed_kts"] = json!(n as f64);
            } else {
                args["height_ft"] = json!(n as f64);
            }
        }
        return Some(call("hover", args));
    }
    // Stop over the spot: before "move", which "stop moving" also contains.
    if [
        "hold position",
        "hold your position",
        "hold the hover",
        "stop moving",
        "hover here",
        "stay here",
    ]
    .iter()
    .any(|p| t.contains(p))
    {
        return Some(call("hover", json!({ "action": "hold" })));
    }
    // An air taxi: "move forward", "air taxi forward at five knots", "hover
    // taxi left". A move word AND a direction, so "forward" alone (a seat,
    // a CG) is not a manoeuvre.
    let moving = [
        "move ",
        "air taxi",
        "hover taxi",
        "taxi ",
        "drift ",
        "slide ",
    ]
    .iter()
    .any(|p| t.contains(p) || t.starts_with(p.trim()));
    let direction = [
        ("forward", "forward"),
        ("ahead", "forward"),
        ("back", "back"),
        ("backward", "back"),
        ("left", "left"),
        ("right", "right"),
    ]
    .iter()
    .find(|(w, _)| has(t, w))
    .map(|(_, d)| *d);
    if let (true, Some(d)) = (moving, direction) {
        let mut args = json!({ "action": "move", "direction": d });
        if let Some(n) = number(t) {
            args["speed_kts"] = json!(n as f64);
        }
        return Some(call("hover", args));
    }
    // Whisper's "hover" at a height (2026-09-23, every one live): "how about
    // ten feet", "what about ten feet", "over at 30 feet", "over ten feet",
    // "prove at thirty feet", "hova, fifty feet", "hold that at 30 feet". A
    // height in FEET with one of those is the hover; without the unit, "over"
    // and "how about" are conversation.
    let feet = has(t, "feet") || has(t, "foot") || has(t, "ft");
    let hover_wart = feet
        && number(t).is_some()
        && [
            "how about",
            "what about",
            "over at",
            "over ",
            "prove at",
            "hova",
            "hold that at",
            "hold at",
        ]
        .iter()
        .any(|p| t.starts_with(p) || t.contains(&format!(" {p}")));
    let up = has(t, "hover") || hover_wart || t.contains("lift off") || t.contains("liftoff");
    if !up {
        return None;
    }
    // "Hover at zero feet" is the set-down (EDDV, 2026-09-22: refused as
    // outside the 3 to 50 ft band, then said again as "hover at 1 foot").
    if number(t) == Some(0) && feet {
        return Some(call("hover", json!({ "action": "down" })));
    }
    let mut args = json!({ "action": "up" });
    if let Some(n) = number(t) {
        args["height_ft"] = json!(n as f64);
    }
    Some(call("hover", args))
}

/// "You have control": the crew hand-over, resolved in the main loop.
pub const TAKE_OVER: &str = "take_over";

fn call(name: &str, args: Value) -> ToolCall {
    ToolCall {
        name: name.to_string(),
        arguments: args,
    }
}

/// First number in the text — digits (already-normalized "270") or spoken words
/// ("eighty" → 80, "five hundred" → 500, "two thousand" → 2000). Returns the
/// magnitude; sign (climb/descend) is applied by the caller.
fn number(t: &str) -> Option<i64> {
    // Whisper writes large numbers with a thousands separator ("7,000"); strip it
    // so the digit group parses as one number, not "7" then "000" → 70. (A comma
    // elsewhere is already a token separator, so removing it changes nothing.)
    let t = t.replace(',', "");
    let (mut total, mut cur, mut found) = (0i64, 0i64, false);
    // "N to M" means M — "to" marks the target in command speech ("throttle to
    // fifty", which whisper garbled live as "12 to 50 percent" and the first-run
    // rule set 12% instead of 50). The pre-"to" value is kept as a fallback so
    // "50 to, uh, taxi" still yields 50 rather than a miss.
    let mut before_to: Option<i64> = None;
    // A digit WORD straight after a digit word is the next digit of the same
    // number, the way a speed or a heading is said on the air: "two two zero"
    // is 220. Summed (2 + 2 + 0) it armed the governor at four knots on a
    // 737 at 2,000 ft (typed line, EGSS 2026-09-22; speech is normalised to
    // digits before it gets here, typed text is not).
    let mut after_digit_word = false;
    for tok in t.split(|c: char| !c.is_ascii_alphanumeric()) {
        if tok.is_empty() {
            continue;
        }
        let digit_word = crate::phrase::word_value(tok).is_some_and(|(v, mult)| !mult && v <= 9)
            && tok.parse::<i64>().is_err();
        if digit_word && after_digit_word {
            let (v, _) = crate::phrase::word_value(tok).unwrap_or((0, false));
            cur = cur * 10 + v;
            continue;
        }
        after_digit_word = digit_word;
        if let Ok(d) = tok.parse::<i64>() {
            // A single digit after a bare tens value ADDS: whisper writes
            // "twenty-eight" as "twenty 8", and the old concatenation made 208
            // ("set flaps to twenty 8" → notch 208, seen live). Nobody speaks
            // 208 that way (it would be "two zero eight" or "two hundred
            // eight"), so 28 is the only sensible reading. Digit-group runs
            // still concatenate ("2 7 0" → 270).
            cur = if cur == 0 {
                d
            } else if (20..=90).contains(&cur) && cur % 10 == 0 && d <= 9 {
                cur + d
            } else {
                cur * 10 + d
            };
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
        } else if tok == "to" && found {
            before_to = Some(total + cur);
            (total, cur, found) = (0, 0, false);
        } else if found {
            break; // number run ended
        }
    }
    found.then_some(total + cur).or(before_to)
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
            let prev_num = i
                .checked_sub(1)
                .and_then(|j| toks.get(j))
                .is_some_and(|p| numish(p));
            let next_num = toks.get(i + 1).is_some_and(|n| numish(n));
            if !(prev_num && next_num) {
                return true; // "and" joins non-numbers → genuine compound
            }
        }
    }
    false
}

/// Parse a spoken/normalized altimeter setting to inches of mercury. Handles the
/// three forms that occur after number-normalization (which only inserts a decimal
/// for an explicit "point", so "two niner niner two" arrives as "2992"):
///   - a decimal inHg ("29.92", "30.15") — used as-is;
///   - a bare 4-digit inHg×100 ("two niner niner two" → "2992" → 29.92);
///   - a 3–4 digit hPa/QNH ("set QNH one zero one three" → "1013" → 29.91 inHg).
///
/// The inHg×100 (2800–3150) and hPa (950–1100) ranges don't overlap, so a bare
/// integer is unambiguous. Returns None if no token looks like a setting.
fn altimeter_inhg(t: &str) -> Option<f64> {
    /// hPa per inHg (1013.25 hPa ≈ 29.92 inHg).
    const HPA_PER_INHG: f64 = 33.8639;
    for w in t.split(|c: char| !(c.is_ascii_digit() || c == '.')) {
        if w.is_empty() {
            continue;
        }
        if w.contains('.') {
            if let Ok(f) = w.parse::<f64>() {
                return Some(f); // already inHg, e.g. 29.92
            }
        } else if let Ok(n) = w.parse::<i64>() {
            if (2800..=3150).contains(&n) {
                return Some(n as f64 / 100.0); // inHg×100, e.g. 2992 → 29.92
            }
            if (950..=1100).contains(&n) {
                return Some(n as f64 / HPA_PER_INHG); // hPa/QNH → inHg
            }
        }
    }
    None
}

/// Whole-word membership (so "on" doesn't match inside "engine").
fn has(t: &str, word: &str) -> bool {
    t.split(|c: char| !c.is_ascii_alphanumeric())
        .any(|w| w == word)
}

/// The SimBrief-flight intent — full auto with the pilot's actual flight plan
/// as the mission ("fly my flight plan", "fly the simbrief flight"). Checked
/// BEFORE `full_auto_call` so "full auto, fly the flight plan" carries the
/// flight flag. Request-shaped: the verb "fly" (or a full-auto phrase) is
/// required, so "load the flight plan" (the GPS load) and "brief the flight
/// plan" (the deterministic brief, handled upstream in main) never land here.
pub fn resume_plan_request(t: &str) -> bool {
    let t = normalize(t);
    // Whisper's warts on the phrase ("Resume site plan.", 2026-09-16): the
    // plan word misheard, or pluralised. Still the closed shape after that.
    let t = t
        .trim_end_matches(['.', '?', '!'])
        .replace("site plan", "flight plan")
        .replace("flight plans", "flight plan");
    // "I said resume" (LIME 2026-09-25, said once it had been ignored).
    let t = t
        .strip_prefix("i said ")
        .or_else(|| t.strip_prefix("i said, "))
        .map(str::to_string)
        .unwrap_or(t);
    // "Resume" with only these words after it is still the resume: whisper
    // pads and garbles the tail ("resume flight now" five times, "resumes
    // like that", "resume the fight plan", "resume flight 9"; 2026-09-2x).
    // A CLOSED set, so "resume taxi" or "resume the checklist" never lands.
    const RESUME_TAIL: &[&str] = &[
        "the", "my", "flight", "fight", "plan", "now", "right", "like", "that", "9", "nine",
    ];
    let lead = t.strip_prefix("resumes ").or_else(|| t.strip_prefix("resume "));
    if let Some(rest) = lead {
        let words: Vec<&str> = rest
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|w| !w.is_empty())
            .collect();
        if !words.is_empty()
            && words.iter().all(|w| RESUME_TAIL.contains(w))
            && words.iter().any(|w| matches!(*w, "flight" | "fight" | "now" | "like" | "right"))
        {
            return true;
        }
    }
    matches!(
        t.as_str(),
        "resume the flight plan"
            | "resume flight plan"
            | "resume my flight plan"
            | "resume the route"
            | "continue the flight plan"
            // The bare word, and the flight or mission (LIME 2026-09-25:
            // "Resume." earned nothing after a taxi stand-down).
            | "resume"
            | "resume the flight"
            | "continue the flight"
            | "resume the mission"
            | "continue the mission"
    )
}

/// "Take us to Bologna", "fly us to lima india papa echo": arm the flight,
/// with the place the pilot named for the tool to check against the plan's
/// destination. Never a place on the ground ("take us to the runway"), an
/// altitude or a number.
fn take_us_to(t: &str) -> Option<ToolCall> {
    const GROUND: &[&str] = &[
        "runway", "gate", "stand", "parking", "ramp", "apron", "hold", "holding", "pad",
        "helipad", "heliport", "hangar", "taxiway",
    ];
    let lead = ["take us to ", "fly us to ", "take me to ", "get us to "]
        .iter()
        .find_map(|l| t.find(l).map(|i| &t[i + l.len()..]))?;
    if GROUND.iter().any(|g| has(lead, g)) {
        return None;
    }
    let d = crate::flightplan::parse_destination(lead)?;
    Some(call(
        "full_auto",
        json!({ "flight": true, "to": d.spoken(), "to_codes": d.codes }),
    ))
}

/// "Set up the flight": the plan's aircraft at its origin (`set_up_flight`).
/// CLOSED: it ends the current flight, so nothing near it ("set up the ILS",
/// "set up for landing") may land here.
pub fn set_up_flight_request(t: &str) -> bool {
    matches!(
        t.trim_end_matches(['.', '!', '?']),
        "set up the flight"
            | "set up my flight"
            | "set up the flight plan"
            | "set up my flight plan"
            | "set up for the flight"
            | "prepare the flight"
            | "prepare my flight"
            | "set up the flight from the plan"
            | "load the flight and the aircraft"
    )
}

/// "Put us on a 10 mile final to LEMG 13", "set us up on final for runway
/// one three", "start us on approach to Malaga", "put us on a five mile
/// final, my controls": a NEW FLIGHT on a runway final (`setup_approach`),
/// started through the sim's own flight initialisation. The field, the
/// runway and the distance are each optional (the tool defaults them); a
/// hand-fly clause ("my controls", "and I'll fly it") keeps the copilot's
/// hands off. CLOSED: a lead that puts US somewhere, then ON a final or an
/// approach and nothing but a distance, a field and a runway; never an
/// approach TYPE ("put us on the ILS" is the coupling's), never a pattern
/// word. Checked before the compound gate ("and I'll fly it" carries "and").
pub fn approach_setup_call(t: &str) -> Option<ToolCall> {
    const LEADS: &[&str] = &[
        "put us ", "put me ", "set us up ", "set me up ", "start us ", "start me ",
        "place us ", "position us ", "reposition us ",
    ];
    const HAND_FLY: &[&str] = &[
        "my controls", "my control", "i will fly", "ill fly", "let me fly", "i fly it",
        "ill hand fly", "i will hand fly", "i have control", "ill take it from there",
    ];
    const NOT_THIS: &[&str] = &[
        "ils", "localizer", "localiser", "glideslope", "glidepath", "rnav", "vor", "gps",
        "visual", "circuit", "pattern", "downwind", "base", "missed", "go", "around",
        "hold", "holding", "short",
    ];
    const MILES: &[&str] = &["mile", "miles", "nm", "mi", "nautical"];
    const CONNECT: &[&str] = &["to", "for", "at", "into", "runway", "rwy", "the"];
    let t = t.trim().trim_end_matches(['.', '!', '?']);
    let lead = LEADS.iter().find(|l| t.starts_with(**l))?;
    let mut rest = &t[lead.len()..];
    let mut hand_fly = false;
    for h in HAND_FLY {
        if let Some(i) = find_phrase(rest, h) {
            hand_fly = true;
            rest = &rest[..i];
        }
    }
    let mut words: Vec<&str> = rest
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    while words.last().is_some_and(|w| matches!(*w, "and" | "then" | "but")) {
        words.pop();
    }
    if !matches!(words.first().copied(), Some("on" | "onto" | "for" | "in")) {
        return None;
    }
    if words.iter().any(|w| NOT_THIS.contains(w)) {
        return None;
    }
    let noun = words.iter().position(|w| matches!(*w, "final" | "approach"))?;
    // The distance: the number said with a mile word, wherever it sits
    // ("a 10 mile final", "final ten miles out").
    let mut used = vec![false; words.len()];
    let mut final_nm: Option<i64> = None;
    if let Some(m) = words.iter().position(|w| MILES.contains(w)) {
        let numeric =
            |w: &str| w.parse::<i64>().is_ok() || word_value(w).is_some_and(|(_, mult)| !mult);
        let mut k = m;
        while k > 0 && m - k < 2 && numeric(words[k - 1]) {
            k -= 1;
        }
        if k == m {
            return None; // "a mile final": no number
        }
        final_nm = number(&words[k..m].join(" "));
        for u in used.iter_mut().take(m + 1).skip(k) {
            *u = true;
        }
        let mut after = m + 1;
        if matches!(words.get(after), Some(&"miles") | Some(&"mile")) {
            used[after] = true;
            after += 1;
        }
        if words.get(after) == Some(&"out") {
            used[after] = true;
        }
    }
    // Before the noun: the preposition, an article, the distance. Nothing else.
    for (i, w) in words.iter().enumerate().take(noun).skip(1) {
        if !used[i] && !matches!(*w, "a" | "an" | "the" | "long") {
            return None;
        }
    }
    // After it: the field, then the runway, connectors between.
    let mut field: Vec<&str> = Vec::new();
    let mut rwy: Vec<&str> = Vec::new();
    for (i, w) in words.iter().enumerate().skip(noun + 1) {
        if used[i] || matches!(*w, "final" | "approach") || CONNECT.contains(w) {
            continue;
        }
        let runway_word = w.chars().next().is_some_and(|c| c.is_ascii_digit())
            || crate::phrase::digit_value(w).is_some()
            || word_value(w).is_some_and(|(v, mult)| !mult && (10..=36).contains(&v))
            || (!rwy.is_empty()
                && matches!(*w, "left" | "right" | "center" | "centre" | "l" | "r" | "c"));
        if runway_word {
            rwy.push(w);
        } else if rwy.is_empty() {
            field.push(w);
        } else {
            return None; // nothing follows the runway
        }
    }
    let mut args = serde_json::Map::new();
    if let Some(n) = final_nm {
        args.insert("final_nm".into(), json!(n));
    }
    if !rwy.is_empty() {
        args.insert("runway".into(), json!(runway_designator(&rwy)?));
    }
    if !field.is_empty() {
        let d = crate::flightplan::parse_destination(&field.join(" "))?;
        args.insert("to".into(), json!(d.spoken()));
        args.insert("to_codes".into(), json!(d.codes));
    }
    if hand_fly {
        args.insert("hand_fly".into(), json!(true));
    }
    Some(call("setup_approach", Value::Object(args)))
}

/// Is this the ask for a new flight on a final ([`approach_setup_call`])?
/// The main loop asks before its content routing and its release guard:
/// "put us on a ten mile final" carries the radio word "final", and "put
/// us on final, my controls" a release word, and neither is either.
pub fn approach_setup_request(t: &str) -> bool {
    approach_setup_call(&normalize(t)).is_some()
}

/// The first whole-phrase occurrence of `needle` in `hay`: "ill fly" is not
/// inside "i will fly".
fn find_phrase(hay: &str, needle: &str) -> Option<usize> {
    let mut from = 0;
    while let Some(i) = hay[from..].find(needle).map(|i| i + from) {
        let before = hay[..i].chars().next_back();
        let after = hay[i + needle.len()..].chars().next();
        if !before.is_some_and(|c| c.is_ascii_alphanumeric())
            && !after.is_some_and(|c| c.is_ascii_alphanumeric())
        {
            return Some(i);
        }
        from = i + needle.len();
    }
    None
}

/// A runway designator from its spoken or written words: "13", "one three",
/// "thirteen", "31 left", "09r". `None` unless it is a runway (1 to 36, one
/// suffix at most, nothing after the suffix). Two digits, as apt.dat has it.
pub fn runway_designator(words: &[&str]) -> Option<String> {
    let mut digits = String::new();
    let mut suffix: Option<char> = None;
    for w in words {
        if suffix.is_some() {
            return None;
        }
        let lead: String = w.chars().take_while(|c| c.is_ascii_digit()).collect();
        let tail = &w[lead.len()..];
        if !lead.is_empty() {
            digits.push_str(&lead);
            match tail {
                "" => {}
                "l" | "r" | "c" => suffix = tail.chars().next().map(|c| c.to_ascii_uppercase()),
                _ => return None,
            }
        } else if let Some(d) = crate::phrase::digit_value(w) {
            digits.push_str(&d.to_string());
        } else if let Some((v, false)) = word_value(w) {
            if !digits.is_empty() {
                return None;
            }
            digits.push_str(&v.to_string());
        } else {
            suffix = Some(match *w {
                "left" | "l" => 'L',
                "right" | "r" => 'R',
                "center" | "centre" | "c" => 'C',
                _ => return None,
            });
        }
    }
    if digits.is_empty() || digits.len() > 2 {
        return None;
    }
    let n: u32 = digits.parse().ok()?;
    if !(1..=36).contains(&n) {
        return None;
    }
    Some(match suffix {
        Some(c) => format!("{n:02}{c}"),
        None => format!("{n:02}"),
    })
}

fn flight_plan_call(t: &str) -> Option<ToolCall> {
    if set_up_flight_request(t) {
        return Some(call("set_up_flight", json!({})));
    }
    if resume_plan_request(t) {
        return Some(call("full_auto", json!({ "resume": true })));
    }
    if matches!(
        t.trim_end_matches(['.', '!']),
        "start the flight" | "begin the flight" | "commence the flight" | "start the mission"
            | "begin the mission"
    ) {
        return Some(call("full_auto", json!({ "flight": true })));
    }
    if let Some(c) = take_us_to(t) {
        return Some(c);
    }
    // Whisper garbles the last word and keeps the shape: "fly the flight
    // gun / path / bluff / class / time", "fly the flat band" (737 and S-76,
    // 2026-09-22/23, each followed by "fly the flight plan"). Nothing else in
    // a cockpit is flown "the flight" anything.
    let garbled_plan = (has(t, "fly") || has(t, "blow"))
        && (t.contains("the flight ") || t.contains("the flat "))
        && !t.contains("director")
        && !t.contains("level");
    let plan_words = t.contains("flight plan")
        || garbled_plan
        || (has(t, "execute") && (has(t, "plan") || has(t, "flight")))
        || t.contains("simbrief")
        || t.contains("sim brief")
        || t.contains("the plan");
    if !plan_words {
        return None;
    }
    // "Execute the flight plan" is the same ask (said live, 2026-09-23, and
    // read back as the plan's brief).
    // "Play the flight plan" is whisper's "fly" (EBLG, 2026-09-22: read back
    // as the brief, then said again as "fly the flight plan").
    let fly_shaped = has(t, "fly")
        || has(t, "flying")
        // "Blow the flight gun" (2026-09-2x): the garbled plan word with
        // "fly" heard as "blow".
        || (garbled_plan && has(t, "blow"))
        || has(t, "execute")
        || t.starts_with("flight to flight plan")
        || t.starts_with("play the flight plan")
        || t.contains("full auto");
    if !fly_shaped {
        return None;
    }
    // Never on a load/brief/read ask, even if "fly" appears somewhere.
    if has(t, "load") || has(t, "program") || has(t, "brief") || has(t, "read") || has(t, "what") {
        return None;
    }
    if has(t, "cancel") || has(t, "stop") || has(t, "abort") {
        return Some(call("full_auto", json!({ "off": true })));
    }
    Some(call("full_auto", json!({ "flight": true })))
}

/// Is this a request to FLY (or resume, or stop) the flight plan? The main
/// loop asks before its plan-brief branch, so the brief never answers words
/// the grammar would fly: "Flight to flight plan" got the brief twice while
/// the grammar had flown it since 2026-09-20.
pub fn flight_plan_request(t: &str) -> bool {
    flight_plan_call(&normalize(t)).is_some()
}

/// Is this an ask to check the whole plan before flying it (`plan_audit`)?
/// CLOSED shape: a checking word and a plan word, as WORDS ("checklist" is
/// not "check"): "check the flight plan", "is the plan safe", "any problems
/// with the route", "can this aircraft do the plan", "is the aircraft
/// suitable for this flight plan". Never "fly the flight plan".
pub fn plan_check_request(t: &str) -> bool {
    let t = normalize(t);
    let words: Vec<&str> = t
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    let has = |w: &str| words.contains(&w);
    let checking = ["check", "verify", "validate", "audit", "safe", "suitable", "problems",
        "problem", "issues", "concerns", "feasible", "workable"]
        .iter()
        .any(|w| has(w))
        || (has("can") && (has("aircraft") || has("we")) && (has("do") || has("make")));
    let plan = has("plan") || has("route") || has("flight");
    checking && plan && !has("fly") && !has("load") && !has("checklist")
}

/// Is this the pilot waiving a refusal they were just offered ("fly it
/// anyway", "I've checked the runway", "override")? CLOSED: the whole
/// utterance is one of these, less address words and filler, so "fly it"
/// or "override the autopilot" is never a waiver. Only meaningful inside
/// `full_auto::WAIVE_WINDOW` of a waivable refusal; the main loop asks.
pub fn waiver_request(t: &str) -> bool {
    const WAIVERS: &[&str] = &[
        "fly it anyway",
        // Whisper, live (LILB 2026-09-25).
        "play it anyway",
        "flight anyway",
        "do it anyway",
        "go anyway",
        "go ahead anyway",
        "take off anyway",
        "fly anyway",
        "override",
        "override it",
        "override that",
        "override the runway",
        "override the weight",
        "ive checked the runway",
        "i have checked the runway",
        "i checked the runway",
        "ive checked the weight",
        "i have checked the weight",
        "i checked the weight",
        "ive checked it",
        "i have checked it",
        "i checked it",
        "my responsibility",
        "on my responsibility",
        "my call",
    ];
    // The answer to the offer is an ASSENT as often as the offered words:
    // "Yes, it's fine. Let's go ahead." earned nothing at LILB (2026-09-25).
    // Only asked inside the window of a refusal that offered a waiver.
    const ASSENTS: &[&str] = &[
        "go ahead",
        "lets go ahead",
        "go for it",
        "lets go",
        "its fine",
        "thats fine",
        "its ok",
        "its okay",
        "proceed",
        "continue",
        "do it",
        "affirmative",
        "confirmed",
        "confirm",
        "send it",
    ];
    const FILLER: &[&str] = &["okay", "ok", "copilot", "please", "yes", "yeah", "then", "just", "sure"];
    const YES: &[&str] = &["yes", "yeah", "affirmative", "sure"];
    let t = t.to_ascii_lowercase().replace(['\'', '\u{2019}'], "");
    let words = |clause: &str| -> String {
        clause
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|w| !w.is_empty() && !FILLER.contains(w))
            .collect::<Vec<_>>()
            .join(" ")
    };
    if WAIVERS.contains(&words(&t).as_str()) {
        return true;
    }
    // Clause by clause: each one an assent (or bare filler), and at least
    // one real word of assent in the whole ("okay" alone is not an answer).
    let clauses: Vec<String> = t.split(['.', ',', '!', '?', ';']).map(words).collect();
    let all_assent = clauses
        .iter()
        .all(|c| c.is_empty() || ASSENTS.contains(&c.as_str()) || WAIVERS.contains(&c.as_str()));
    let any_assent = clauses.iter().any(|c| !c.is_empty())
        || t.split(|c: char| !c.is_ascii_alphanumeric()).any(|w| YES.contains(&w));
    all_assent && any_assent
}

/// Is this the load-sheet request ("load the aircraft", "set the fuel and
/// payload from the plan")? The main loop asks before its FMS-load branch,
/// whose "load ... plan" shape these words also have.
pub fn load_aircraft_request(t: &str) -> bool {
    load_aircraft_call(&normalize(t)).is_some()
}

/// The `load_aircraft` intent: the plan's block fuel and payload into the
/// sim. CLOSED shape, a loading verb and a load-sheet object:
/// "load the aircraft", "load / set up the fuel and payload", "set the fuel
/// from the plan", "fuel the aircraft", "board the passengers". Never the fuel
/// SYSTEM ("set fuel to left", "fuel pump on"), never the fuel truck
/// ("refuel", "top off the tanks": `request_service`), never a question
/// ("did you load the fuel?"), never the FMS load ("load the flight plan").
fn load_aircraft_call(t: &str) -> Option<ToolCall> {
    const QUESTION_OPENERS: &[&str] = &[
        "what", "how", "is", "are", "did", "do", "does", "have", "has", "why", "when", "where",
        "which", "should", "will",
    ];
    const FUEL_SYSTEM: &[&str] = &[
        "pump",
        "pumps",
        "selector",
        "left",
        "right",
        "both",
        "flow",
        "cutoff",
        "crossfeed",
        "valve",
        "tank",
        "off",
        "on",
        "refuel",
    ];
    let first = t.split_whitespace().next()?;
    if QUESTION_OPENERS.contains(&first) || FUEL_SYSTEM.iter().any(|w| has(t, w)) {
        return None;
    }
    let fuel_obj = has(t, "fuel");
    let payload_obj = [
        "payload",
        "passengers",
        "pax",
        "cargo",
        "bags",
        "baggage",
        "loadsheet",
    ]
    .iter()
    .any(|w| has(t, w))
        || t.contains("load sheet")
        || t.contains("weight and balance");
    let craft_obj = ["aircraft", "airplane", "aeroplane", "plane", "jet"]
        .iter()
        .any(|w| has(t, w));
    let plan_ref = t.contains("plan") || t.contains("simbrief") || t.contains("sim brief");
    let verb_load = has(t, "load") && !t.contains("the load") || has(t, "board");
    // "the load" as a noun is the payload: "set up the load and fuel".
    let load_noun = t.contains("the load") || t.contains("load and fuel");
    let verb_set = has(t, "set") || has(t, "setup") || has(t, "match");
    // "fuel" as the verb: "fuel the aircraft", "fuel us up".
    let verb_fuel = first == "fuel" && (craft_obj || has(t, "us") || has(t, "up") || plan_ref);

    let what = if verb_fuel {
        if payload_obj {
            "both"
        } else {
            "fuel"
        }
    } else if verb_load && (fuel_obj || payload_obj) || verb_set && (payload_obj || load_noun) {
        match (fuel_obj, payload_obj || load_noun) {
            (true, true) => "both",
            (true, false) => "fuel",
            _ => "payload",
        }
    } else if verb_load && craft_obj && !plan_ref || verb_load && craft_obj && has(t, "from") {
        "both"
    } else if verb_set && fuel_obj && plan_ref {
        "fuel"
    } else {
        return None;
    };
    Some(call("load_aircraft", json!({ "what": what })))
}

/// The `full_auto` intent — the entire flight from cold (power up, engine,
/// taxi, line up, circuit). Request-shaped only; recognised before the compound
/// gate because "start up and fly" carries "and". `None` when it isn't one.
fn full_auto_call(t: &str) -> Option<ToolCall> {
    let is_full_auto = t.contains("full auto")
        || t.contains("fully automated")
        || t.contains("full automatic")
        || t == "do everything"
        || t.contains("do everything")
        || t.contains("do it all")
        || t.contains("start up and fly")
        || t.contains("start up and do")
        || t.contains("startup and fly")
        || t.contains("take us up from cold")
        || t.contains("cold start and fly")
        || t.contains("fly from cold");
    if !is_full_auto {
        return None;
    }
    if has(t, "cancel") || has(t, "stop") || has(t, "abort") {
        return Some(call("full_auto", json!({ "off": true })));
    }
    let right = has(t, "right")
        && (t.contains("circuit") || t.contains("pattern") || t.contains("traffic"));
    Some(call("full_auto", json!({ "right": right })))
}

/// The `circuit` intent — the whole traffic pattern in one command. Recognised
/// on the normalized text before the compound gate, because its natural
/// phrasings carry "and" ("take off and land"). Request-shaped only, so no ATC
/// readback ("cleared for takeoff runway two seven") can reach it. `None` when
/// the utterance isn't a circuit ask.
fn circuit_call(t: &str) -> Option<ToolCall> {
    let circuit_word = t.contains("circuit")
        || t.contains("the pattern")
        || t.contains("traffic pattern")
        || t.contains("circuits and bumps");
    let toff = t.contains("take off") || t.contains("takeoff");
    // "take off AND ..." — the compound shapes that mean a full circuit, not a
    // bare departure. Bare takeoffs (no landing/return words) are NOT caught
    // here; they fall through to the auto_takeoff rule.
    let takeoff_and_land = toff
        && (has(t, "land")
            || t.contains("come back")
            || t.contains("back around")
            || t.contains("bring us back"));
    if !circuit_word && !takeoff_and_land {
        return None;
    }
    // "do a circuit" means the WHOLE flight to the pilot — from wherever the
    // aircraft is. So it routes to the full-auto orchestrator, which starts the
    // engine / taxis / lines up as needed and skips straight to the airborne
    // circuit when already on the runway. (The `circuit` tool still exists;
    // full-auto drives it internally for the airborne part.)
    if has(t, "abort") || has(t, "cancel") || has(t, "stop") || (has(t, "off") && circuit_word) {
        return Some(call("full_auto", json!({ "off": true })));
    }
    let right = has(t, "right")
        && (t.contains("circuit") || t.contains("pattern") || t.contains("traffic"));
    Some(call("full_auto", json!({ "right": right })))
}

/// The `fly_procedure` intent — fly a real published SID or approach from the
/// CIFP. Request-shaped ("fly the ILS", "shoot the approach", "fly the SID");
/// deliberately NOT triggered by an ATC readback ("cleared ILS approach runway
/// two seven, report established"), which carries the clearance verbs. `None`
/// when the utterance isn't a fly-the-procedure ask.
fn procedure_call(t: &str) -> Option<ToolCall> {
    // ATC readback shapes carry these — never route them to an actuator.
    if t.contains("cleared") || t.contains("report") || t.contains("established") {
        return None;
    }
    // The circuit/pattern words are the circuit orchestrator's, handled earlier.
    if t.contains("circuit") || t.contains("pattern") {
        return None;
    }
    let departure = t.contains("the departure") || t.contains("the sid") || t.contains("a sid");
    let approach = t.contains("the approach")
        || t.contains("an approach")
        || t.contains("the ils")
        || t.contains("the r nav")
        || t.contains("the rnav")
        || t.contains("the arrival")
        || t.contains("me in")
        || t.contains("us in")
        || t.contains("me down");
    let cancel = has(t, "cancel") || has(t, "stop") || has(t, "abort");
    // Arming needs a fly-shaped verb; cancelling ("cancel the approach") needs
    // only the cancel word plus a procedure word, no verb.
    let verb = t.contains("fly")
        || t.contains("shoot")
        || t.starts_with("take us")
        || t.contains("bring us in")
        || t.contains("bring us down");
    if !verb && !cancel {
        return None;
    }
    if departure {
        return Some(if cancel {
            call("fly_procedure", json!({ "off": true }))
        } else {
            call("fly_procedure", json!({ "kind": "departure" }))
        });
    }
    if approach {
        return Some(if cancel {
            call("fly_procedure", json!({ "off": true }))
        } else {
            call("fly_procedure", json!({ "kind": "approach" }))
        });
    }
    None
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

    // Reminders: "wake me at top of descent", "tell me when we reach 5000 feet",
    // "remind me in 10 minutes", "let me know at the next waypoint". The verb gates
    // it so a plain "top of descent" or a bare altitude isn't swallowed; an
    // unrecognized target falls through (e.g. "tell me the weather").
    // A complaint or past-tense reference ("you didn't remind me", "you never woke
    // me") is not a request to arm one.
    let negated = t.contains("didnt")
        || t.contains("did not")
        || t.contains("never")
        || t.contains("forgot")
        || t.contains("should have")
        || t.contains("supposed to");
    let reminder_verb = !negated
        && (has(t, "wake")
            || has(t, "remind")
            || has(t, "notify")
            || (has(t, "tell") && has(t, "me"))
            || t.contains("let me know")
            || t.contains("heads up")
            || t.contains("heads-up"));
    if reminder_verb {
        if t.contains("top of descent") || has(t, "tod") || (has(t, "descent") && !has(t, "rate")) {
            return Some(call("set_reminder", json!({ "kind": "top_of_descent" })));
        }
        if has(t, "waypoint") || has(t, "fix") {
            return Some(call("set_reminder", json!({ "kind": "next_waypoint" })));
        }
        if (has(t, "minute") || has(t, "minutes")) && num.is_some() {
            return Some(call(
                "set_reminder",
                json!({ "kind": "minutes", "value": num }),
            ));
        }
        if num.is_some()
            && (has(t, "feet")
                || has(t, "foot")
                || has(t, "altitude")
                || has(t, "reach")
                || has(t, "at")
                || has(t, "when"))
        {
            return Some(call(
                "set_reminder",
                json!({ "kind": "altitude", "value": num }),
            ));
        }
    }

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

    // Ground / ramp ops first — "ground power" must beat the throttle branch (which
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

    // Radio tuning by number: "tune radio to 122.8", "com one 118.5", "set com
    // two to 121.9", "nav one 109.5". Was LLM-only, so with the LLM off every
    // phrasing missed (six in a row on the 2026-07-08 night flight, stranding
    // the pilot off-frequency). Requires a radio word AND a decimal in the
    // right band, so headings ("heading 122") and altimeter settings never
    // match. Whisper glues the radio index into the number ("com 1 122.8" →
    // "com 1122.8"), so 1xxx.x/2xxx.x recover com1/com2 + the frequency.
    // A change/switch verb with a decimal is a tuning request even without a
    // radio word — "change to 125.655" missed live (2026-07-20) while
    // "change to frequency 125.655" matched. The band check below is the
    // real guard; the subject exclusions keep fractional non-radio values
    // ("change heading 122.5") out of the block entirely.
    let change_with_decimal = (has(t, "change") || has(t, "switch"))
        && t.contains('.')
        && !(has(t, "heading")
            || has(t, "course")
            || has(t, "altitude")
            || has(t, "qnh")
            || has(t, "altimeter")
            || has(t, "barometer")
            || has(t, "baro"));
    if has(t, "com")
        || has(t, "radio")
        || has(t, "frequency")
        || has(t, "freq")
        || has(t, "nav")
        || has(t, "tune")
        || change_with_decimal
    {
        let com_band = 118.0..=136.975;
        let nav_band = 108.0..=117.95;
        // Spoken decimals survive as words ("one two two decimal eight");
        // rejoin them so the band check sees one number.
        let spelled = t.replace(" decimal ", ".").replace(" point ", ".");
        let com2_named = spelled.contains("com two") || spelled.contains("com 2") || has(t, "com2");
        let mut found: Option<(&str, f64)> = None;
        for w in spelled.split(|c: char| !(c.is_ascii_digit() || c == '.')) {
            let Ok(v) = w.parse::<f64>() else { continue };
            if !w.contains('.') && v < 1000.0 {
                continue; // integers below the glue range: headings, squawks
            }
            found = if nav_band.contains(&v) && has(t, "nav") {
                Some(("set_nav1", v))
            } else if com_band.contains(&v) && !has(t, "nav") {
                Some((if com2_named { "set_com2" } else { "set_com1" }, v))
            } else if com_band.contains(&(v - 1000.0)) {
                // "com 1122.8" glue; re-round to kHz (float subtraction dust).
                Some(("set_com1", ((v - 1000.0) * 1000.0).round() / 1000.0))
            } else if com_band.contains(&(v - 2000.0)) {
                Some(("set_com2", ((v - 2000.0) * 1000.0).round() / 1000.0))
            } else {
                None
            };
            if found.is_some() {
                break;
            }
        }
        // No decimal token: whisper also writes frequencies as spoken parts —
        // "130 four hundred", "1 thirty four hundred", "131, 200" (all shapes
        // from the 2026-07-09 morning log, every one a miss). Fold the number
        // atoms into integer MHz + kHz: a leading 1 glues onto a tens word
        // ("1 thirty one" = 131), "four hundred" multiplies onto the 4, and a
        // 2-3 digit trailer is the kHz part (a single digit is tenths).
        if found.is_none() && !has(t, "nav") {
            let mut atoms: Vec<i64> = Vec::new();
            for tok in spelled.split(|c: char| !c.is_ascii_alphanumeric()) {
                if tok.is_empty() {
                    continue;
                }
                if let Ok(d) = tok.parse::<i64>() {
                    atoms.push(d);
                } else if let Some((v, mult)) = word_value(tok) {
                    if mult && v == 100 {
                        if let Some(last) = atoms.last_mut() {
                            *last *= 100;
                        }
                    } else if !mult {
                        atoms.push(v);
                    }
                }
            }
            let mut i = 0;
            while i < atoms.len() && found.is_none() {
                let (int_mhz, j) = if atoms[i] == 1
                    && atoms
                        .get(i + 1)
                        .is_some_and(|v| (20..=90).contains(v) && v % 10 == 0)
                {
                    let mut v = 100 + atoms[i + 1];
                    let mut j = i + 2;
                    if atoms.get(j).is_some_and(|u| (1..=9).contains(u)) {
                        v += atoms[j];
                        j += 1;
                    }
                    (v, j)
                } else {
                    (atoms[i], i + 1)
                };
                if com_band.contains(&(int_mhz as f64)) {
                    let frac = match atoms.get(j) {
                        Some(&k) if (10..=999).contains(&k) => k as f64 / 1000.0,
                        Some(&d) if (1..=9).contains(&d) => d as f64 / 10.0,
                        _ => 0.0,
                    };
                    let tool = if com2_named { "set_com2" } else { "set_com1" };
                    found = Some((tool, int_mhz as f64 + frac));
                }
                i += 1;
            }
        }
        if let Some((tool, mhz)) = found {
            return Some(call(tool, json!({ "mhz": mhz })));
        }
    }

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
        // (2026-07-09): "Flaps, ooh." / "Flaps, cool." / "Flaps, pull." —
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
        // "flaps for landing" / "landing flaps" — the tool resolves the alias to
        // the airframe's last detent (and the Vfe gate still refuses if fast).
        } else if has(t, "landing") || has(t, "land") {
            "landing".to_string()
        } else if let Some(n) = num {
            n.to_string()
        } else {
            return None;
        };
        return Some(call("set_flaps", json!({ "notch": notch })));
    }

    // Autothrottle (jets) — before the throttle rule. "autothrottle on/off".
    if t.contains("autothrottle") || t.contains("auto throttle") {
        let on = !(has(t, "off") || t.contains("disconnect") || t.contains("disengage"));
        return Some(call("autothrottle", json!({ "on": on })));
    }
    // Auto takeoff — the copilot flies the roll and initial climb. Only
    // first/second-person shapes: a READBACK ("cleared for takeoff runway
    // two seven") and a call to the tower ("ready for takeoff") carry none
    // of these, so neither can ever arm it — the same trap the auto-taxi
    // rules dodge. Abort shapes first, so "abort the takeoff" never re-arms.
    {
        let takeoff_word =
            t.contains("takeoff") || t.contains("take off") || t.contains("take us off");
        if takeoff_word
            && (has(t, "abort") || has(t, "reject") || has(t, "cancel") || has(t, "stop"))
        {
            return Some(call("auto_takeoff", json!({ "off": true })));
        }
        if t == "abort" || t == "abort abort" || t == "abort abort abort" {
            return Some(call("auto_takeoff", json!({ "off": true })));
        }
        // The normalizer peels trailing politeness, so "take off for me"
        // arrives as the bare words — and like bare "taxi", nobody says
        // them to a copilot meaning anything else (a readback always
        // carries "cleared for" plus the runway, so it stays longer).
        if t == "takeoff" || t == "take off"
            || t.contains("auto takeoff") || t.contains("auto take off")
            || t.contains("take off for me") || t.contains("takeoff for me")
            || t.contains("take off for us") || t.contains("takeoff for us")
            || t.contains("take us off") || t.contains("take off when ready")
            || t.contains("you have the takeoff") || t.contains("you have the take off")
            || t.contains("your takeoff") || t.contains("your take off")
            || t.contains("you fly the takeoff") || t.contains("fly the takeoff")
            // Whisper's renderings of the shapes above (live 2026-07-20):
            // "you have the takeoff" → "you have to take off", "your
            // takeoff" → "you'll take off" (apostrophe already stripped),
            // "you handle the takeoff" → "use handle to take off". All
            // still second-person; no readback or tower call carries them.
            || t.contains("you have to take off") || t.contains("youll take off")
            || t.contains("you handle the takeoff") || t.contains("handle the takeoff")
            || t.contains("handle to take off")
        {
            return Some(call("auto_takeoff", json!({})));
        }
    }
    // Line up — the copilot drives from the hold-short position onto the
    // runway centreline. The pilot's READBACK "line up and wait ..." always
    // carries "wait" (and "lining up" is the short echo), so both are
    // excluded; the command shapes are first/second-person. Trailing "for
    // me" peels, so bare "line up" as the whole utterance is the command
    // ("can you line me up to the runway" went unanswered live, 2026-07-20).
    if !has(t, "wait")
        && !t.contains("lining")
        && (t == "line up" || t == "line up to the runway" || t == "line up on the runway"
            || t.contains("line us up") || t.contains("line me up")
            || t.contains("get us lined up") || t.contains("take us onto the runway")
            // "let's line up with the runway" peels its "lets" and arrives
            // as this shape (three phrasings went unanswered live,
            // 2026-07-20). The wait/lining guards above still exclude the
            // LUAW readback and the short echo.
            || (t.starts_with("line up") && has(t, "runway")))
    {
        return Some(call("line_up", json!({})));
    }
    // Backtrack — the full-length line-up (drive to the threshold, turn
    // around). Whole-word "backtrack" so the -ing echo never matches, and
    // the "wait" guard mirrors the LUAW readback trap. The normalizer folds
    // whisper's split "back track".
    if has(t, "backtrack")
        && !has(t, "wait")
        && !t.contains("lining")
        && (t == "backtrack"
            || t.contains("backtrack for me")
            || t.contains("backtrack for us")
            || t.contains("backtrack us")
            || t.contains("backtrack the runway")
            || t.contains("backtrack runway"))
    {
        return Some(call("line_up", json!({ "backtrack": true })));
    }
    // Taxi to PARKING after landing — the copilot requests the taxi-in from ATC
    // and drives to the nearest stand. BEFORE auto_taxi (which is the departure
    // taxi to a holding point). Generic/request-shaped forms only: a specific
    // "taxi to stand 12 via ..." is a taxi READBACK to the tower, not a request
    // to the copilot (the same trap auto_taxi dodges).
    if t.contains("taxi to parking")
        // Whisper, twice live: "request parking to taxi", "talk to taxi".
        || t.contains("parking to taxi")
        || t == "talk to taxi"
        || t.contains("taxi to the stand")
        || t.contains("taxi to the gate")
        || t.contains("taxi to the ramp")
        || t.contains("taxi to the apron")
        || t.contains("taxi us in")
        || t == "taxi in"
        || t.contains("take us to parking")
        || t.contains("take us to the stand")
        || t.contains("take us to the gate")
        || t.contains("take us to the ramp")
        || t.contains("park us")
        || t.contains("park the aircraft")
        || t.contains("park the plane")
    {
        return Some(call("taxi_to_parking", json!({})));
    }
    // Auto taxi — the copilot steers the cleared route to the holding point.
    // Before every speed-hold rule, so "taxi for me at 12 knots" doesn't fall
    // into plain hold_speed. Deliberately NOT matching "taxi to holding point
    // ..." — that's the pilot's taxi READBACK to the tower, never a request
    // to drive (the same trap is_taxi_query dodges by being question-shaped).
    if t.contains("auto taxi") || t.contains("autotaxi") {
        return Some(if has(t, "stop") || has(t, "cancel") || has(t, "off") {
            call("auto_taxi", json!({ "off": true }))
        } else if let Some(n) = num {
            call("auto_taxi", json!({ "knots": n }))
        } else {
            call("auto_taxi", json!({}))
        });
    }
    // Bare "taxi" as the whole utterance lands here too: the normalizer
    // peels trailing politeness, so "taxi for me" arrives as just "taxi" —
    // and nobody says the bare word to a copilot meaning anything else.
    // The exact "taxi to (the) holding point" with NO specifics is a
    // request too: "let's taxi to holding point" peels its leading "lets"
    // and arrived here as exactly that, then fell into the readback trap
    // and went unanswered (live, 2026-07-20). A real readback always
    // carries the specifics being echoed (the point name, runway, or
    // "via ..."), so anything longer still never arms the drive.
    if t == "taxi"
        || t == "taxi to holding point"
        || t == "taxi to the holding point"
        // The runway named as the destination and nothing else ("Taxi to
        // runway.", Twin Otter, 2026-09-22, followed by three more tries and
        // a give-up). Exact like the holding-point shapes above and for the
        // same reason: a taxi READBACK always carries the specifics being
        // echoed (the runway number, the route, "via ..."), so it is longer
        // than this and never lands here.
        || t == "taxi to runway"
        || t == "taxi to the runway"
        || t == "taxi to the active"
        || t.contains("taxi for me")
        || t.contains("taxi for us")
        || t.contains("you have the taxi")
        || t.contains("you take the taxi")
        || t.contains("you taxi")
        || t.contains("take us to the holding")
        || t.contains("take us to holding")
        || t.contains("taxi us")
    {
        return Some(match num {
            Some(n) => call("auto_taxi", json!({ "knots": n })),
            None => call("auto_taxi", json!({})),
        });
    }
    // Speed hold — the copilot works the throttle ("hold 70 knots", "maintain
    // approach speed", "can you maintain an appropriate speed" — asked live with
    // no tool to answer it). Verbs first, so it beats the AP speed-bug rule
    // below on airframes that have this tool; the release phrasings first of
    // all, so "stop holding speed" doesn't re-arm.
    // STOP — brakes to a halt and hold. The bare word only as the whole
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
        // GO — resume the last held taxi speed after a stop. Bare "go" only
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
    // "taxi at 10 knots" / "taxi 12 knots" — the taxi governor (the tool picks
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

    // AP airspeed select: "set speed 250", "speed 280", "airspeed 250". Excludes
    // vertical/ground speed (handled elsewhere) and percent (that's throttle).
    // A bare knots unit qualifies too ("reduced to 60 knots", seen dead-ending
    // live) — but never wind talk ("wind is ten knots" is weather, not a bug).
    if (has(t, "speed")
        || has(t, "airspeed")
        || ((has(t, "knots") || has(t, "knot")) && !has(t, "wind")))
        && !has(t, "vertical")
        && !has(t, "ground")
        && !has(t, "climb")
        && !has(t, "descend")
        && !t.contains("percent")
    {
        // "speed for landing" / "approach speed" / "landing speed" — the tool
        // pulls the profile's final-approach speed.
        if has(t, "landing") || has(t, "land") || has(t, "approach") {
            return Some(call("set_speed", json!({ "preset": "approach" })));
        }
        // Relative before the bare number: "reduce speed by 20" is minus twenty
        // from the bug, not a 20-knot target. Default nudge is 20 knots.
        // "reduce speed TO 180" is a target, not a decrement — "to" marks it,
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
    // "slow down" / "slow to 180" — no "speed" token, so it needs its own rule.
    if has(t, "slow") && !has(t, "vertical") && !t.contains("percent") {
        if let Some(n) = num {
            return Some(call("set_speed", json!({ "knots": n })));
        }
        return Some(call("set_speed", json!({ "delta": -20 })));
    }

    // Constant-speed prop levers — before throttle/power (a prop request never
    // means the throttle). "props full forward", "props up", "props to cruise",
    // "prop rpm 1900", "props to 1600", "coarse pitch". Match any token STARTING
    // with "prop" so whisper's "prope"/"propp"/"propeller" all land, without
    // grabbing "appropriate" mid-word. Named modes are the primary interface
    // (robust where an RPM value is transcription-fragile); an explicit RPM still
    // works, and a bare "props" asks which setting rather than missing.
    let is_prop = t
        .split(|c: char| !c.is_ascii_alphanumeric())
        .any(|w| w.starts_with("prop"));
    if is_prop {
        if let Some(n) = num {
            return Some(call("set_prop", json!({ "rpm": n })));
        }
        // Whole-word matching for the short detent tokens — substring checks
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
        // A prop command with no target ("set props") — let the tool ask which
        // setting rather than falling through to a bare miss.
        return Some(call("set_prop", json!({})));
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
        // Relative verbs: "reduce throttle by ten percent" means minus ten from
        // the current lever, NOT throttle to 10 (a power chop — seen live). A
        // relative verb without "to" is a delta; with "to" it's the target.
        // Past-tense forms are whisper slips on the imperative ("increased
        // powers of 60 percent" heard live), and "of" before the number is the
        // same slip on "to" — target, not delta.
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
        // Bare "increase power" / "reduce throttle" — a nudge, ten percent
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

    // heading 270 / heading north. Whisper wraps a bare "heading north" in a
    // false first-person opener ("So I'm heading north" for a turn command,
    // 2026-07-12) — those ARE commands here, so no narration guard: a genuine
    // question ("is that heading north?") is withheld upstream by its
    // interrogative opener, not by refusing to parse it. (The earlier
    // "narration re-bugged the heading" scare was really the drifted gyro.)
    if has(t, "heading") {
        if let Some(n) = num {
            return Some(call("set_heading", json!({ "heading": n })));
        }
        // "keep/hold/maintain current heading", "hold this heading" — bug
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

    // ILS approach setup — was LLM-only, so every phrasing missed with the LLM
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
    if has(t, "ils") || (couple && (has(t, "approach") || has(t, "localizer") || has(t, "localiser"))) {
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
        return Some(call("prepare_ils", json!({ "runway": runway, "couple": couple })));
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
        // 28") — a landing-help request; the advisory talk-down is that help.
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
    // or an explicit AP coupling ("set AP to nav", "set autopilot to navigation" —
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

    // The whole aircraft off, the pilot's last words at the stand ("switch
    // off the aircraft", LIPK 2026-09-25, a miss; "electrics off" a second
    // later worked). Before the power-up shapes: "power the aircraft down"
    // contains "power the aircraft". Exact shapes, so "switch off the
    // aircraft lights" stays the lights'.
    if [
        "switch off the aircraft",
        "turn off the aircraft",
        "switch the aircraft off",
        "turn the aircraft off",
        "power down the aircraft",
        "power the aircraft down",
        "power down",
    ]
    .contains(&t.trim_end_matches(['.', '!']))
    {
        return Some(call(
            "set_electrical",
            json!({ "system": "all", "on": false }),
        ));
    }

    // "power up" / "powering up" / "bring the electrical up" → bring everything up.
    // No on/off token, so it'd otherwise miss the block below and fall to the LLM.
    // The main loop remaps set_electrical:all to the step-by-step power_up tool on
    // airframes that have it (auto_prep); on a GA single it just switches all on.
    if t.contains("power up")
        || t.contains("powering up")
        || t.contains("power the aircraft")
        || (has(t, "bring") && (has(t, "electrical") || has(t, "electrics")))
    {
        return Some(call(
            "set_electrical",
            json!({ "system": "all", "on": true }),
        ));
    }

    // Bare electrical shapes with NO state word: whisper hears the trailing
    // "on" as "are" ("Electrical systems are", live 2026-07-22). The ask is
    // obviously ON — cold-dark that's the only thing it can mean, and if
    // they're already on the tool's readback says so, which beats a silent
    // miss. Exact shapes only, so longer sentences (and question forms,
    // which the question gate catches first anyway) never land here.
    if t == "electrical systems are"
        || t == "electrical systems"
        || t == "electrical system"
        || t == "electrics"
    {
        return Some(call(
            "set_electrical",
            json!({ "system": "all", "on": true }),
        ));
    }

    // The centre tank pumps ("centre pumps off", "turn off the center
    // pumps", "ctr pumps on"): before the fuel pump rule, which would take
    // "centre fuel pumps" to the engine boost pumps.
    if (has(t, "centre") || has(t, "center") || has(t, "ctr"))
        && (has(t, "pump") || has(t, "pumps"))
    {
        if let Some(on) = onoff(t) {
            return Some(call("set_centre_pumps", json!({ "on": on })));
        }
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
        // Roof floods (the Baron's cockpit pods). The copilot sets these itself
        // after dark, but the pilot must still be able to say so — and "roof"
        // needs no light/lights qualifier, since nothing else on the aircraft
        // is called one.
        if has(t, "roof") {
            return Some(call("set_lights", json!({ "light": "roof", "on": on })));
        }
        for (kw, key) in [
            ("avionics", "avionics"),
            ("battery", "battery"),
            ("alternator", "alternator"),
            ("generator", "generator"),
            ("master", "master"),
            // The inverter powers the C90's AC bus (avionics/EFIS/autopilot); on
            // airframes without one the tool just lists what it has.
            ("inverter", "inverter"),
            ("inverters", "inverter"),
            // EFIS power switch (separate from avionics master) — lights the EADI/EHSI.
            ("efis", "efis"),
            // Bleed air valves (Twin Otter pneumatics: heat, anti-ice, and the
            // AP run off them) — a profile key like any other switchable system.
            ("bleed", "bleed"),
        ] {
            if has(t, kw) {
                // Split systems: "alternator two", "battery 1" name one of a
                // numbered pair (SR22 BAT 1/2, ALT 1/2). The tool resolves a
                // bare noun to the pair, and strips the digit back off on
                // airframes with a single unnumbered system.
                let key = if matches!(key, "battery" | "alternator" | "generator") {
                    if has(t, "two") || has(t, "2") {
                        format!("{key}2")
                    } else if has(t, "one") || has(t, "1") {
                        format!("{key}1")
                    } else {
                        key.to_string()
                    }
                } else {
                    key.to_string()
                };
                return Some(call("set_electrical", json!({ "system": key, "on": on })));
            }
        }
        if has(t, "electrical") || has(t, "electrics") || has(t, "electric") {
            return Some(call("set_electrical", json!({ "system": "all", "on": on })));
        }
        // "fuel pump(s)" or "boost pump(s)" — the King Air's standby boost pumps
        // are what clears an L/R FUEL PRESS annunciator, and pilots call them
        // either way. "boost" alone counts (whisper often drops "pump").
        if t.contains("fuel pump") || t.contains("boost pump") || has(t, "boost") {
            return Some(call("set_fuel_pump", json!({ "on": on })));
        }
        if has(t, "pitot") {
            return Some(call("set_pitot_heat", json!({ "on": on })));
        }
        if has(t, "carb") || has(t, "carburetor") {
            return Some(call("set_carb_heat", json!({ "on": on })));
        }
    }

    // Reverse thrust (landing rollout) — the levers, not a direction:
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
        // "cut the engines" / "kill the engines" — both are in the tool's own
        // description of itself, and only one of them parsed.
        || ((has(t, "kill") || has(t, "cut")) && (has(t, "engine") || has(t, "engines")))
    {
        return Some(call("stop_engine", json!({})));
    }
    if has(t, "gyro") {
        // align/sync/set gyro — also catches mishears like "think/zinc gyro".
        return Some(call("align_gyro", json!({})));
    }
    // EMERGENCY recovery — take the controls and fly out of an upset/stall.
    // Whole-aircraft distress phrasings ONLY: "take control of the throttle"
    // once engaged a full recovery on a calm approach (the copilot seized the
    // aircraft and the pilot needed four phrasings to get it back), and "help
    // me land ..." is a request for guidance, not a mayday. A named system
    // after "take control of" means a system request — never recovery.
    // Direction of transfer matters: "take control" is only a request for the
    // COPILOT to take over when it isn't the pilot announcing they're taking it
    // themselves. "Let me take controls" once engaged a full recovery against
    // the pilot — the exact inversion of what was said.
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
    // Emergency autoland — hand off to the autoland plugin. Cancel first so
    // "cancel/stop the autoland" doesn't match the arm phrases.
    if t.contains("cancel autoland")
        || t.contains("cancel the autoland")
        || t.contains("stop autoland")
        || t.contains("stop the autoland")
        || t.contains("cancel the landing")
    {
        return Some(call("autoland", json!({ "cancel": true })));
    }
    if t.contains("land the aircraft")
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

    // fuel selector / tank. Explicit side wins; a swap verb with no side is the
    // routine "switch tanks" (the SR22 has no BOTH, so it's said every half hour)
    // — the tool reads the current tank and feeds from the other one. Also match
    // bare "fuel" with a swap verb or an explicit "to <side>" ("switch fuel to
    // right", "set fuel to left") — seen live, five times in one session, routed
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
        // Mode words before on/off: "mode C on" and "squawk altitude" are the
        // altitude mode, not a bare "on". The 2026-07-10 pilot said
        // "transponder on" ten ways and every one missed to the LLM — only
        // standby/ident/vfr/codes were covered.
        if has(t, "altitude")
            || has(t, "alt")
            || has(t, "charlie")
            || (has(t, "mode") && has(t, "c"))
        {
            return Some(call("set_transponder_mode", json!({ "mode": "altitude" })));
        }
        if has(t, "off") {
            return Some(call("set_transponder_mode", json!({ "mode": "off" })));
        }
        if has(t, "on") || has(t, "activate") || has(t, "enable") {
            return Some(call("set_transponder_mode", json!({ "mode": "on" })));
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
    if has(t, "altimeter") || has(t, "barometer") || has(t, "baro") || has(t, "qnh") {
        if let Some(inhg) = altimeter_inhg(t) {
            return Some(call("set_barometer", json!({ "inhg": inhg })));
        }
        // No setting spoken ("set the altimeter", "altimeter to field") → dial in
        // the current local QNH. But a number that just didn't parse as a valid
        // setting ("altimeter 50") falls to the LLM rather than silently setting it.
        if !t.chars().any(|c| c.is_ascii_digit()) {
            return Some(call("set_altimeter_current", json!({})));
        }
    }

    None
}

/// The tool exists and every arg key is in its schema. We deliberately do NOT
/// reject an out-of-range *enum value* here: every value-taking tool validates
/// its own argument and replies with the valid options ("I have up, approach,
/// down") — which is far more useful than a bare "Say again?". So a recognized
/// command with a wrong value for this airframe (e.g. "flaps 10" on the King Air)
/// is routed to the tool to teach the right one, not swallowed. Unknown tools and
/// unknown arg keys still fail closed.
fn valid(c: &ToolCall, registry: &Registry, _profile: &AircraftProfile) -> bool {
    // Not a tool: the main loop resolves it against the flight model
    // (`handover::decide`) into the calls the flight needs now.
    if c.name == TAKE_OVER {
        return true;
    }
    let Some(tool) = registry.get(&c.name) else {
        return false;
    };
    let schema = tool.parameters();
    let props = schema.get("properties").and_then(|p| p.as_object());
    let Some(args) = c.arguments.as_object() else {
        return true;
    };
    for k in args.keys() {
        let Some(props) = props else {
            return false; // args given but the tool takes none
        };
        if !props.contains_key(k) {
            return false; // unknown argument
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

    /// The load sheet: a loading verb and a load-sheet object, and nothing
    /// that belongs to the fuel system, the fuel truck or the FMS.
    #[test]
    fn the_load_sheet_is_one_intent_and_never_the_fuel_system() {
        for (words, what) in [
            ("load the aircraft", "both"),
            ("Load the aircraft from the flight plan.", "both"),
            ("load the fuel and payload", "both"),
            ("set up the load and fuel", "both"),
            ("set the fuel and payload from the plan", "both"),
            ("load the fuel from the flight plan", "fuel"),
            ("set the fuel from the plan", "fuel"),
            ("fuel the aircraft", "fuel"),
            ("fuel us up for the flight plan", "fuel"),
            ("board the passengers", "payload"),
            ("set the payload", "payload"),
            ("load the passengers and cargo", "payload"),
        ] {
            assert_eq!(
                one(words),
                Some(("load_aircraft".into(), json!({ "what": what }))),
                "{words}"
            );
            assert!(load_aircraft_request(words), "{words}");
        }
        for words in [
            "load the flight plan",
            "load it",
            "load the route into the gps",
            "load the flight plan into the aircraft",
            "did you load the fuel",
            "how much fuel do we have",
            "set fuel to left",
            "fuel pump on",
            "fuel selector both",
            "we should stop for fuel at the next field",
            "set the flaps",
        ] {
            assert!(!load_aircraft_request(words), "{words}");
        }
        assert_eq!(one("refuel").unwrap().0, "request_service");
    }

    /// Four sorties' worth of missed phrasings from the logs, each the live
    /// transcript that went unanswered. Whisper's warts first: the hand-back,
    /// the plan word, the autoland, the electrics and the unit.
    #[test]
    fn the_warts_the_logs_caught_resolve_to_the_tool_the_pilot_meant() {
        // "my controls" — a missed hand-back is an automation still flying.
        assert_eq!(one("Mind controls.").unwrap().0, "autopilot_off");
        // The plan word, three ways.
        assert!(resume_plan_request("Resume the fight plan."));
        assert_eq!(one("Fly the flight man.").unwrap().0, "full_auto");
        assert!(normalize("That's the wrong flag plan.").contains("flight plan"));
        // Autoland, armed and cancelled.
        assert_eq!(one("All to land.").unwrap(), ("autoland".into(), json!({})));
        assert_eq!(
            one("Cancel all to land?").unwrap(),
            ("autoland".into(), json!({ "cancel": true }))
        );
        assert_eq!(
            one("Cancel Autolamp.").unwrap(),
            ("autoland".into(), json!({ "cancel": true }))
        );
        // "Electric source / sauce / software" were once rewritten to
        // "electrics ON". Every one was said at the SHUTDOWN, straight after
        // "engines off", and followed within seconds by "electrics off"
        // (172 2026-09-14, Baron 2026-09-16, 737 2026-09-22): the pilot meant
        // OFF. With no confirm on an all-off (it runs airborne too), a guess
        // either way is wrong: they miss, and the pilot says it again.
        for words in ["Electric source.", "Electric sauce.", "Electric software."] {
            assert!(one(words).is_none(), "{words}");
        }
        // The unit, and the load sheet's verb. A pace with a number in it is
        // the governor's front door, which is where a taxi pace belongs.
        assert_eq!(
            one("Taxi at ten notes.").unwrap(),
            ("hold_speed".into(), json!({ "knots": 10 }))
        );
        assert!(load_aircraft_request("Loads the aircraft."));
        // ...and the words they sound like are left alone.
        assert_eq!(normalize("power idle"), "power idle");
    }

    /// The taxi and the landing asked in the fewest words there are. Both
    /// were logged as misses with the pilot trying again and giving up.
    #[test]
    fn the_shortest_taxi_and_landing_asks_are_requests() {
        // Twin Otter, 2026-09-22: four tries at the taxi, then full auto.
        assert_eq!(one("Taxi to runway.").unwrap().0, "auto_taxi");
        assert_eq!(one("taxi to the runway").unwrap().0, "auto_taxi");
        // ...but a readback carries its specifics and is still not a request.
        assert!(
            one("taxi to runway zero nine via alpha").is_none_or(|(tool, _)| tool != "auto_taxi")
        );
        assert!(
            one("taxi to holding point charlie one runway zero four left")
                .is_none_or(|(tool, _)| tool != "auto_taxi")
        );
        // 737 2026-09-17 and A330 2026-09-20.
        assert_eq!(one("Let's land.").unwrap(), ("autoland".into(), json!({})));
        assert_eq!(
            one("Let's land right now.").unwrap(),
            ("autoland".into(), json!({}))
        );
        // A named field is a diversion, not the autoland tool.
        assert!(one("let's land at jerez").is_none_or(|(tool, _)| tool != "autoland"));
    }

    /// The S-76C declares `engine: false` — the twist grips are the pilot's,
    /// so no start or shutdown tool is built. At the pad on 2026-09-17 the
    /// pilot asked four ways in eleven seconds and heard nothing at all. A
    /// capability the airframe lacks is said, not swallowed.
    #[test]
    fn the_helicopters_engines_reach_its_start_tool() {
        // The S-76C had no start tool and its engine words were declined
        // aloud (2026-09-22, morning); it has one now, measured on the
        // aircraft, and the same words reach it.
        let prof = aircraft::load("sikorsky_s76c").expect("s76 profile");
        let reg = Registry::for_aircraft(&prof);
        for (words, tool) in [
            ("Start engines.", "start_engines"),
            ("Start the engines.", "start_engines"),
            ("Engine's on.", "start_engines"),
            ("Engines on.", "start_engines"),
            ("shut down the engines", "stop_engines"),
        ] {
            assert!(declined(words, &reg).is_none(), "{words}");
            let calls = try_parse(words, &reg, &prof).unwrap_or_default();
            assert_eq!(
                calls.first().map(|c| c.name.as_str()),
                Some(tool),
                "{words}"
            );
        }
        // The airframes that do have the tool are untouched.
        let (creg, _) = setup();
        assert!(declined("start the engine", &creg).is_none());
        let jet = aircraft::load("boeing_737_800").expect("737 profile");
        let jreg = Registry::for_aircraft(&jet);
        assert!(declined("engines off", &jreg).is_none());
    }

    #[test]
    fn the_flight_plan_is_flown_through_whispers_garbles() {
        // Every one said live and followed by "fly the flight plan" (737 and
        // S-76, 2026-09-22/23): the last word goes, the shape stays.
        let jet = aircraft::load("boeing_737_800").expect("737 profile");
        let jreg = Registry::for_aircraft(&jet);
        let heli = aircraft::load("sikorsky_s76c").expect("s76 profile");
        let hreg = Registry::for_aircraft(&heli);
        for words in [
            "Fly the flight path.",
            "Fly the flight bluff.",
            "Let's fly the flight bluff.",
            "Fly the flight class.",
            "Fly the flat band?",
            "Okay, fly the flight time.",
            "Play the flight plan.",
            "Flight to flight plan.",
        ] {
            let c = try_parse(words, &jreg, &jet).expect(words);
            assert_eq!(c[0].name, "full_auto", "{words}");
            assert_eq!(c[0].arguments, json!({ "flight": true }), "{words}");
            assert!(
                flight_plan_request(words),
                "the brief must not catch {words}"
            );
            let h = try_parse(words, &hreg, &heli).expect(words);
            assert_eq!(h[0].arguments, json!({ "action": "route" }), "{words}");
        }
        // Not the plan: other things flown, and the plan's brief.
        for words in [
            "Fly the flight director.",
            "Fly flight level three five zero.",
            "What's the flight plan?",
            "Load the flight plan.",
        ] {
            assert!(!flight_plan_request(words), "{words}");
        }
    }

    #[test]
    fn a_helicopter_hovers_and_sets_down_by_voice() {
        let prof = aircraft::load("sikorsky_s76c").expect("s76 profile");
        let reg = Registry::for_aircraft(&prof);
        let first = |w: &str| try_parse(w, &reg, &prof).and_then(|mut v| v.drain(..).next());
        // A hover at zero feet is the set-down (EDDV, 2026-09-22: refused as
        // outside the band).
        let c = first("Hover at 0 feet.").expect("zero");
        assert_eq!(c.arguments, json!({ "action": "down" }));
        // The pad is not here, and "land the aircraft" lets the tool choose.
        let c = first("Not here, land at the airport.").expect("pad");
        assert_eq!(c.arguments, json!({ "action": "pad" }));
        let c = first("take us to the pad").expect("pad");
        assert_eq!(c.arguments, json!({ "action": "pad" }));
        let c = first("Let's land the aircraft.").expect("land");
        assert_eq!(c.arguments, json!({ "action": "land" }));
        let c = first("land here").expect("here");
        assert_eq!(c.arguments, json!({ "action": "down" }));
        let c = first("set it down").expect("down");
        assert_eq!(c.arguments, json!({ "action": "down" }));
        assert!(first("landing light on").is_none_or(|c| c.name != "hover"));
        // A helicopter's pad is not a flight's destination.
        assert!(first("take us to the pad").is_some_and(|c| c.name == "hover"));
        let c = first("Lift to a hover.").expect("hover");
        assert_eq!(
            (c.name.as_str(), c.arguments["action"].as_str()),
            ("hover", Some("up"))
        );
        let c = first("hover at fifteen feet").expect("height");
        assert_eq!(c.arguments["height_ft"].as_f64(), Some(15.0));
        let c = first("Set it down.").expect("down");
        assert_eq!(c.arguments["action"].as_str(), Some("down"));
        // A general "land" lets the tool choose the pad when one is in reach
        // (LIME 2026-09-25); putting it on the ground is here.
        for (w, action) in [
            ("Land the aircraft.", "land"),
            ("Let's land.", "land"),
            ("land", "land"),
            ("Land 3 helicopter?", "land"),
            ("Put it on the ground.", "down"),
        ] {
            let c = first(w).unwrap_or_else(|| panic!("{w}"));
            assert_eq!(
                (c.name.as_str(), c.arguments["action"].as_str()),
                ("hover", Some(action)),
                "{w}"
            );
        }
        let c = first("Move forward at five knots.").expect("move");
        assert_eq!(c.arguments["action"].as_str(), Some("move"));
        assert_eq!(c.arguments["direction"].as_str(), Some("forward"));
        assert_eq!(c.arguments["speed_kts"].as_f64(), Some(5.0));
        let c = first("air taxi left").expect("left");
        assert_eq!(c.arguments["direction"].as_str(), Some("left"));
        for w in ["Hold position.", "stop moving", "hover here"] {
            let c = first(w).unwrap_or_else(|| panic!("{w}"));
            assert_eq!(c.arguments["action"].as_str(), Some("hold"), "{w}");
        }
        for w in [
            "Take off.",
            "depart to five hundred feet",
            "transition to forward flight",
        ] {
            let c = first(w).unwrap_or_else(|| panic!("{w}"));
            assert_eq!(c.arguments["action"].as_str(), Some("depart"), "{w}");
        }
        let c = first("depart to five hundred feet").unwrap();
        assert_eq!(c.arguments["height_ft"].as_f64(), Some(500.0));
        // A readback of a clearance is not a request.
        assert!(first("cleared for takeoff").is_none_or(|c| c.name != "hover"));
        // A departure's speed rides the AP's speed words: EDDV 2026-09-23,
        // both missed.
        for w in [
            "speed fifty knots",
            "speed forty knots",
            "set speed 60 knots",
        ] {
            let c = first(w).unwrap_or_else(|| panic!("{w}: {:?}", try_parse(w, &reg, &prof)));
            assert_eq!(
                (c.name.as_str(), c.arguments["action"].as_str()),
                ("hover", Some("speed")),
                "{w}"
            );
        }
        for w in ["fly the flight plan", "fly my flight plan"] {
            let c = first(w).unwrap_or_else(|| panic!("{w}"));
            assert_eq!(c.arguments["action"].as_str(), Some("route"), "{w}");
        }
        // Parking is the aeroplanes' own tool on a helicopter too.
        for w in ["taxi to parking", "park us", "take us to the stand"] {
            let c = first(w).unwrap_or_else(|| panic!("{w}"));
            assert_eq!(c.name, "taxi_to_parking", "{w}");
        }
        // Every one of these was heard live on 2026-09-23 and missed.
        for (w, h) in [
            ("How about ten feet?", 10.0),
            ("What about ten feet?", 10.0),
            ("Over at 30 feet.", 30.0),
            ("Over ten feet.", 10.0),
            ("Prove at thirty feet.", 30.0),
            ("Hova, fifty feet.", 50.0),
            ("Hold that at 30 feet.", 30.0),
        ] {
            let c = first(w).unwrap_or_else(|| panic!("{w}"));
            assert_eq!(c.arguments["height_ft"].as_f64(), Some(h), "{w}");
        }
        assert!(first("how about the weather").is_none_or(|c| c.name != "hover"));
        let c = first("Willis on the ground.").expect("willis");
        assert_eq!(c.arguments["action"].as_str(), Some("down"));
        for w in [
            "Execute flight plan.",
            "Execute plan.",
            "execute flight.",
            "and fly the flight gun.",
            "Flight to flight plan.",
        ] {
            let c = first(w).unwrap_or_else(|| panic!("{w}"));
            assert_eq!(c.arguments["action"].as_str(), Some("route"), "{w}");
        }
        for w in ["Request parking to taxi.", "Talk to taxi."] {
            let c = first(w).unwrap_or_else(|| panic!("{w}"));
            assert_eq!(c.name, "taxi_to_parking", "{w}");
        }
        let c = first("Let's fly a circuit.").expect("circuit");
        assert_eq!(
            (c.name.as_str(), c.arguments["action"].as_str()),
            ("hover", Some("circuit"))
        );
        let c = first("right hand circuit").expect("right");
        assert_eq!(c.arguments["right"].as_bool(), Some(true));
        let c = first("cancel the circuit").expect("cancel");
        assert_eq!(c.arguments["action"].as_str(), Some("hold"));
        // The lights and the gear keep their own tools.
        assert!(first("landing light on").is_none_or(|c| c.name != "hover"));
        assert!(first("gear down for landing").is_none_or(|c| c.name != "hover"));
        // An aeroplane has no hover: the words are not grabbed there.
        let (creg, cprof) = setup();
        assert!(try_parse("set it down", &creg, &cprof).is_none_or(|v| v[0].name != "hover"));
    }

    fn one(t: &str) -> Option<(String, Value)> {
        let (reg, prof) = setup();
        try_parse(t, &reg, &prof).map(|mut v| {
            let c = v.remove(0);
            (c.name, c.arguments)
        })
    }

    /// Like [`one`] but on a named profile's registry — for tools only some
    /// airframes expose (reverse thrust on the Twin Otter).
    fn one_on(profile_id: &str, t: &str) -> Option<(String, Value)> {
        let prof = aircraft::load(profile_id).expect("profile");
        let reg = Registry::for_aircraft(&prof);
        try_parse(t, &reg, &prof).map(|mut v| {
            let c = v.remove(0);
            (c.name, c.arguments)
        })
    }

    #[test]
    fn a_new_flight_on_a_final_is_one_command() {
        // The shapes the pilot uses (LEMG 2026-09-26, driven by hand that
        // night): the field, runway and distance each optional.
        let (name, a) = one("put us on approach to LEMG 13").unwrap();
        assert_eq!(name, "setup_approach");
        assert_eq!(a["runway"], "13");
        assert_eq!(a["to_codes"], json!(["LEMG"]));
        assert!(a.get("final_nm").is_none());
        let (_, a) = one("Set us up on a 10 mile final to runway 13.").unwrap();
        assert_eq!(a["final_nm"], 10);
        assert_eq!(a["runway"], "13");
        assert!(a.get("to").is_none());
        let (_, a) = one("start us on final for one three").unwrap();
        assert_eq!(a["runway"], "13");
        let (_, a) = one("put us on a 5 mile final").unwrap();
        assert_eq!(a["final_nm"], 5);
        assert!(a.get("runway").is_none());
        let (_, a) = one("put us on a five mile final").unwrap();
        assert_eq!(a["final_nm"], 5);
        let (_, a) =
            one("set us up on a ten mile final to lima echo mike golf runway three one left")
                .unwrap();
        assert_eq!(a["final_nm"], 10);
        assert_eq!(a["runway"], "31L");
        assert_eq!(a["to_codes"], json!(["LEMG"]));
        let (_, a) = one("put us on final twelve miles out for runway 13").unwrap();
        assert_eq!(a["final_nm"], 12);
        assert_eq!(a["runway"], "13");
        // The pilot flies it: the copilot only monitors.
        for w in [
            "Put us on final, my controls.",
            "put us on a 10 mile final and I'll fly it",
            "set us up on final for runway 13, I will fly",
        ] {
            let (name, a) = one(w).unwrap();
            assert_eq!(name, "setup_approach", "{w}");
            assert_eq!(a["hand_fly"], true, "{w}");
        }
        assert!(one("put us on a 10 mile final").unwrap().1.get("hand_fly").is_none());
    }

    #[test]
    fn a_final_that_is_not_a_new_flight_is_left_alone() {
        for w in [
            "put us on the ils",
            "set us up for the ils approach",
            "put us on downwind",
            "put us on short final",
            "set up the approach",
            "final runway one three",
            "turn final",
            "put the gear down",
            "put us on a mile final",
            "put us on final runway 13 please now go",
            "put us on final to runway 40",
        ] {
            assert!(
                one(w).is_none_or(|(n, _)| n != "setup_approach"),
                "{w} became a new flight"
            );
        }
        assert!(approach_setup_request("Put us on a 10 mile final."));
        assert!(!approach_setup_request("final runway one three"));
    }

    #[test]
    fn a_runway_designator_is_spoken_or_written() {
        assert_eq!(runway_designator(&["13"]).as_deref(), Some("13"));
        assert_eq!(runway_designator(&["one", "three"]).as_deref(), Some("13"));
        assert_eq!(runway_designator(&["thirteen"]).as_deref(), Some("13"));
        assert_eq!(runway_designator(&["3"]).as_deref(), Some("03"));
        assert_eq!(runway_designator(&["31", "left"]).as_deref(), Some("31L"));
        assert_eq!(runway_designator(&["09r"]).as_deref(), Some("09R"));
        assert_eq!(runway_designator(&["zero", "niner"]).as_deref(), Some("09"));
        assert_eq!(runway_designator(&["40"]), None);
        assert_eq!(runway_designator(&["1", "3", "5"]), None);
        assert_eq!(runway_designator(&["31", "left", "2"]), None);
    }

    #[test]
    fn set_up_the_flight_is_a_closed_phrase() {
        for w in ["set up the flight", "Set up my flight.", "prepare the flight"] {
            assert_eq!(one(w).unwrap().0, "set_up_flight", "{w}");
        }
        for w in ["set up the ils", "set up for landing", "set up the approach"] {
            assert!(!set_up_flight_request(w), "{w}");
        }
    }

    #[test]
    fn resume_plan_is_an_explicit_request_not_a_brief_or_echo() {
        for words in [
            "resume the flight plan",
            "resume flight plan",
            "please resume my flight plan",
            "continue the flight plan",
            "Resume site plan.",
            "resume flight plans",
            // Live misses, 2026-09-2x.
            "resume flight now",
            "resume flight",
            "resumes like that",
            "resumes right now",
            "resume the fight plan",
            "resume flight 9",
        ] {
            assert!(resume_plan_request(words), "{words}");
            assert_eq!(
                one(words).unwrap(),
                ("full_auto".into(), json!({"resume": true}))
            );
        }
        for words in [
            "load the flight plan",
            "what is the flight plan",
            "the copilot will resume the flight plan",
            "do not resume the flight plan",
            "resume the checklist",
            "resume taxi",
            "resume flight plan tomorrow",
        ] {
            assert!(!resume_plan_request(words), "{words}");
        }
    }

    #[test]
    fn reverse_thrust_is_levers_not_navigation() {
        let rev = |t: &str| one_on("dhc6_twin_otter", t);
        // The levers, on an airframe that has them...
        assert_eq!(
            rev("reverse thrust").unwrap(),
            ("reverse_thrust".into(), json!({"on":true}))
        );
        assert_eq!(rev("full reverse").unwrap().1, json!({"on":true}));
        assert_eq!(rev("reversers").unwrap().1, json!({"on":true}));
        assert_eq!(rev("stow reverse").unwrap().1, json!({"on":false}));
        assert_eq!(rev("cancel reverse").unwrap().1, json!({"on":false}));
        assert_eq!(rev("out of reverse").unwrap().1, json!({"on":false}));
        assert_eq!(rev("reverse thrust off").unwrap().1, json!({"on":false}));
        // ...but "reverse" as a direction is navigation talk, never the levers.
        assert_ne!(
            rev("reverse heading").map(|c| c.0),
            Some("reverse_thrust".into())
        );
        assert_ne!(
            rev("reverse course").map(|c| c.0),
            Some("reverse_thrust".into())
        );
        assert_ne!(
            rev("reverse the turn").map(|c| c.0),
            Some("reverse_thrust".into())
        );
        // "Max Pross?" → props full ("pross" repaired upstream; needs a
        // constant-speed airframe — the C172 registry has no prop tool).
        assert_eq!(
            rev("max props").unwrap(),
            ("set_prop".into(), json!({"setting":"full"}))
        );
    }

    #[test]
    fn speed_targets_reach_the_governor_on_a_c172() {
        // Ten consecutive dead ends on the 2026-07-09 final: every "set/change
        // speed" phrasing resolved to set_speed (the AP airspeed select, which
        // the 172 doesn't register) and died. They remap to the speed-hold
        // governor, the tool this airframe actually has.
        for say in [
            "change speed to 60 knots",
            "Change speed to 60.",
            "set airspeed to 60 knots",
            "Speed to 60 knots.",
            "60 knots speed.",
            "Reduced to 60 knots.",
        ] {
            assert_eq!(
                one(say).unwrap(),
                ("hold_speed".into(), json!({"knots": 60})),
                "{say}"
            );
        }
        // "reduce speed TO 180" is a target; "reduce speed BY 20" a decrement.
        assert_eq!(
            one("reduce speed to 40 knots").unwrap(),
            ("hold_speed".into(), json!({"knots": 40}))
        );
        // Weather talk must never become a speed command.
        assert_eq!(one("the wind is 10 knots"), None);
    }

    #[test]
    fn com_tuning_parses_deterministically() {
        // The 2026-07-08 night flight: six tuning phrasings in a row missed
        // (LLM off), stranding the pilot off-frequency. All deterministic now.
        assert_eq!(
            one("tune radio to 122.8").unwrap(),
            ("set_com1".into(), json!({ "mhz": 122.8 }))
        );
        assert_eq!(
            one("com one 118.5").unwrap(),
            ("set_com1".into(), json!({ "mhz": 118.5 }))
        );
        // Whisper glues the radio index into the number ("com 1 122.8").
        assert_eq!(
            one("switch to com 1122.8").unwrap(),
            ("set_com1".into(), json!({ "mhz": 122.8 }))
        );
        assert_eq!(
            one("set com two to 121.9").unwrap(),
            ("set_com2".into(), json!({ "mhz": 121.9 }))
        );
        assert_eq!(
            one("nav one 109.5").unwrap(),
            ("set_nav1".into(), json!({ "mhz": 109.5 }))
        );
        assert_eq!(
            one("set frequency to 131.2").unwrap(),
            ("set_com1".into(), json!({ "mhz": 131.2 }))
        );
        // No frequency spoken → not a radio command here (the tuner or LLM's
        // job), and non-band numbers never match.
        assert!(one("change com 1 frequency").is_none());
        assert!(one("heading 122").map(|c| c.0) != Some("set_com1".into()));
        assert!(one("set altimeter 29.92").map(|c| c.0) != Some("set_com1".into()));
        // Spoken-parts shapes, verbatim from the 2026-07-09 morning log:
        // whisper writes the kHz as hundreds words, glues "1 thirty" for 130,
        // and comma-splits "131, 200".
        assert_eq!(
            one("switch frequency 1 thirty four hundred").unwrap(),
            ("set_com1".into(), json!({ "mhz": 130.4 }))
        );
        assert_eq!(
            one("change to frequency 130 four hundred").unwrap(),
            ("set_com1".into(), json!({ "mhz": 130.4 }))
        );
        assert_eq!(
            one("change comm frequency to 1 thirty four hundred").unwrap(),
            ("set_com1".into(), json!({ "mhz": 130.4 }))
        );
        assert_eq!(
            one("okay, frequency 131, 200").unwrap(),
            ("set_com1".into(), json!({ "mhz": 131.2 }))
        );
        assert_eq!(
            one("frequency 1 thirty one 200").unwrap(),
            ("set_com1".into(), json!({ "mhz": 131.2 }))
        );
    }

    #[test]
    fn reminders_resolve_deterministically() {
        assert_eq!(
            one("wake me at top of descent").unwrap(),
            ("set_reminder".into(), json!({"kind":"top_of_descent"}))
        );
        assert_eq!(
            one("let me know at the next waypoint").unwrap(),
            ("set_reminder".into(), json!({"kind":"next_waypoint"}))
        );
        assert_eq!(
            one("remind me in 10 minutes").unwrap(),
            ("set_reminder".into(), json!({"kind":"minutes","value":10}))
        );
        assert_eq!(
            one("wake me at 5000 feet").unwrap(),
            (
                "set_reminder".into(),
                json!({"kind":"altitude","value":5000})
            )
        );
        // A reminder verb with no recognized target falls through (not a reminder).
        assert!(one("tell me the altitude").map(|c| c.0) != Some("set_reminder".into()));
        // A complaint must not arm a new reminder.
        assert!(
            one("you didn't remind me of the last waypoint").map(|c| c.0)
                != Some("set_reminder".into())
        );
    }

    #[test]
    fn ap_speed_and_autothrottle_resolve() {
        let prof = aircraft::load("md82").expect("md82");
        let reg = Registry::for_aircraft(&prof);
        let g = |t: &str| {
            try_parse(t, &reg, &prof).map(|mut v| {
                let c = v.remove(0);
                (c.name, c.arguments)
            })
        };
        assert_eq!(
            g("set speed 250").unwrap(),
            ("set_speed".into(), json!({"knots":250}))
        );
        assert_eq!(g("airspeed 280").unwrap().0, "set_speed");
        assert_eq!(
            g("autothrottle on").unwrap(),
            ("autothrottle".into(), json!({"on":true}))
        );
        assert_eq!(g("autothrottle off").unwrap().1, json!({"on":false}));
        // "vertical speed 1000" must still be a rate, not an airspeed select.
        assert_eq!(g("vertical speed 1000").unwrap().0, "set_vertical_speed");
        // Landing setup phrases (born of an LEAL–LEMD arrival that got busy):
        // approach preset, relative slow-downs, and the flap alias.
        assert_eq!(
            g("set speed for landing").unwrap(),
            ("set_speed".into(), json!({"preset":"approach"}))
        );
        assert_eq!(g("approach speed").unwrap().1, json!({"preset":"approach"}));
        assert_eq!(g("reduce speed by 20").unwrap().1, json!({"delta":-20}));
        assert_eq!(g("reduce speed").unwrap().1, json!({"delta":-20}));
        assert_eq!(g("slow down").unwrap().1, json!({"delta":-20}));
        assert_eq!(g("slow to 180").unwrap().1, json!({"knots":180}));
        assert_eq!(
            g("set flaps for landing").unwrap(),
            ("set_flaps".into(), json!({"notch":"landing"}))
        );
        // "reduce throttle by ten percent" must stay a throttle call, not a speed one.
        assert_eq!(
            g("reduce throttle by ten percent").unwrap().0,
            "set_throttle"
        );
    }

    #[test]
    fn gear_resolves_deterministically() {
        // The 172 has no gear tool, so use a retractable airframe (MD-82).
        let prof = aircraft::load("md82").expect("md82");
        let reg = Registry::for_aircraft(&prof);
        let g = |t: &str| {
            try_parse(t, &reg, &prof).map(|mut v| {
                let c = v.remove(0);
                (c.name, c.arguments)
            })
        };
        assert_eq!(
            g("gear up").unwrap(),
            ("gear".into(), json!({"position":"up"}))
        );
        assert_eq!(
            g("gear down").unwrap(),
            ("gear".into(), json!({"position":"down"}))
        );
        assert_eq!(
            g("positive rate, gear up").unwrap().1,
            json!({"position":"up"})
        );
        assert_eq!(
            g("put the gear down for landing").unwrap().1,
            json!({"position":"down"})
        );
    }

    #[test]
    fn pitot_mishearings_resolve() {
        // "pitot" defeats whisper — every one of these observed live (LEAL–LEMD,
        // four failed attempts at 14,000 ft) must land on set_pitot_heat.
        let on = ("set_pitot_heat".to_string(), json!({"on":true}));
        assert_eq!(one("pitot heat on").unwrap(), on);
        assert_eq!(one("pitos heat on").unwrap(), on); // "Pito's heat on."
        assert_eq!(one("pito heat on").unwrap(), on);
        assert_eq!(one("pilot heat on").unwrap(), on);
        assert_eq!(one("it was heat on").unwrap(), on);
        assert_eq!(one("a bit of heat on").unwrap(), on);
        assert_eq!(one("pitot heat off").unwrap().1, json!({"on":false}));
    }

    #[test]
    fn tens_word_plus_digit_adds_not_concatenates() {
        // Whisper writes "twenty-eight" as "twenty 8"; concatenation made 208
        // (an impossible flap notch, seen live). It must ADD to 28.
        assert_eq!(
            one("set flaps to twenty 8").unwrap(),
            ("set_flaps".into(), json!({"notch":"28"}))
        );
        assert_eq!(
            one("throttle to sixty 5 percent").unwrap().1,
            json!({"percent":65})
        );
        // Digit-group runs still concatenate, and plain values are untouched.
        assert_eq!(one("heading 2 7 0").unwrap().1, json!({"heading":270}));
        assert_eq!(one("flaps thirty").unwrap().1, json!({"notch":"30"}));
    }

    #[test]
    fn electrics_off_stays_off() {
        // The whole-system OFF direction: the main loop must not remap this to
        // power_up (it did once, powering the aircraft back up post-shutdown).
        assert_eq!(
            one("electrics off").unwrap(),
            ("set_electrical".into(), json!({"system":"all","on":false}))
        );
        // The whole aircraft, at the stand (LIPK 2026-09-25), and never the
        // power-up that "power the aircraft" means.
        for words in [
            "Switch off the aircraft.",
            "Turn the aircraft off.",
            "Power the aircraft down.",
            "Power down.",
        ] {
            assert_eq!(
                one(words).unwrap(),
                ("set_electrical".into(), json!({"system":"all","on":false})),
                "{words}"
            );
        }
        assert_eq!(one("Shut down.").unwrap().0, "stop_engine");
        assert_eq!(
            one("switch off electrical systems").unwrap().1,
            json!({"system":"all","on":false})
        );
        assert_eq!(
            one("electric systems off").unwrap().1,
            json!({"system":"all","on":false})
        );
    }

    #[test]
    fn common_commands_resolve_deterministically() {
        assert_eq!(one("flaps ten").unwrap().0, "set_flaps");
        assert_eq!(one("flaps up").unwrap().1, json!({"notch":"up"}));
        assert_eq!(one("flaps full").unwrap().1, json!({"notch":"full"}));
        // Whisper's "full" garbles, comma'd so the phrase-level repair can't
        // span them — four dead ends in a row live (Twotter, 2026-07-09).
        assert_eq!(one("flaps, ooh").unwrap().1, json!({"notch":"full"}));
        assert_eq!(one("flaps, cool").unwrap().1, json!({"notch":"full"}));
        assert_eq!(one("flaps, pull").unwrap().1, json!({"notch":"full"}));
        // ...while "up" still wins over "pull" in the same sentence.
        assert_eq!(one("pull the flaps up").unwrap().1, json!({"notch":"up"}));
        assert_eq!(
            one("throttle eighty percent"),
            Some(("set_throttle".into(), json!({"percent":80})))
        );
        assert_eq!(one("maximum power").unwrap().1, json!({"percent":100}));
        // "power" is a throttle synonym, and "maximum throttle"/"% power" forms.
        assert_eq!(
            one("power thirty percent").unwrap(),
            ("set_throttle".into(), json!({"percent":30}))
        );
        assert_eq!(one("40 percent power").unwrap().1, json!({"percent":40}));
        assert_eq!(one("maximum throttle").unwrap().1, json!({"percent":100}));
        assert_eq!(
            one("throttle to cruise").unwrap(),
            ("set_throttle".into(), json!({"percent":65}))
        );
        // Transponder modes — the 2026-07-10 pilot said "transponder on" ten
        // ways and every one missed to the LLM; only standby/ident/vfr/codes
        // were covered. Mode words beat on/off ("mode C on" is altitude).
        assert_eq!(
            one("transponder on").unwrap(),
            ("set_transponder_mode".into(), json!({"mode":"on"}))
        );
        assert_eq!(one("transponder off").unwrap().1, json!({"mode":"off"}));
        assert_eq!(one("activate transponder").unwrap().1, json!({"mode":"on"}));
        assert_eq!(
            one("set transponder mode on").unwrap().1,
            json!({"mode":"on"})
        );
        assert_eq!(
            one("mode on for transponder").unwrap().1,
            json!({"mode":"on"})
        );
        assert_eq!(
            one("transponder standby").unwrap().1,
            json!({"mode":"standby"})
        );
        assert_eq!(
            one("squawk altitude").unwrap().1,
            json!({"mode":"altitude"})
        );
        assert_eq!(
            one("transponder alt mode").unwrap().1,
            json!({"mode":"altitude"})
        );
        assert!(one("mode c on").is_none()); // no transponder word — not ours
        assert_eq!(
            one("transponder mode c on").unwrap().1,
            json!({"mode":"altitude"})
        );
        assert_eq!(
            one("squawk vfr").unwrap(),
            ("set_transponder".into(), json!({"code":1200}))
        );
        assert_eq!(one("squawk 4521").unwrap().1, json!({"code":4521}));
        assert_eq!(one("mixture 70 percent").unwrap().0, "set_mixture");
        assert_eq!(
            one("maximum mixture").unwrap(),
            ("set_mixture".into(), json!({"percent":100}))
        );
        assert_eq!(one("mixture full rich").unwrap().1, json!({"percent":100}));
        assert_eq!(one("mixture idle cutoff").unwrap().1, json!({"percent":0}));
        // Smart leaning (no number) routes to lean_mixture; an explicit % stays set_mixture.
        assert_eq!(
            one("lean for cruise").unwrap(),
            ("lean_mixture".into(), json!({"mode":"auto"}))
        );
        assert_eq!(one("lean it out").unwrap().0, "lean_mixture");
        assert_eq!(
            one("mixture for cruise").unwrap(),
            ("lean_mixture".into(), json!({"mode":"auto"}))
        );
        assert_eq!(
            one("lean for max power").unwrap(),
            ("lean_mixture".into(), json!({"mode":"power"}))
        );
        assert_eq!(
            one("lean the mixture to 70 percent").unwrap().0,
            "set_mixture"
        );
        // A bare mixture adjustment (no number) means "set it right" → lean_mixture.
        assert_eq!(
            one("adjust the mixture").unwrap(),
            ("lean_mixture".into(), json!({"mode":"auto"}))
        );
        assert_eq!(one("sort out the mixture").unwrap().0, "lean_mixture");
        // ...but enrich/richen/full still go full rich, not lean.
        assert_eq!(
            one("enrich the mixture").unwrap(),
            ("set_mixture".into(), json!({"percent":100}))
        );
        assert_eq!(one("richen the mixture").unwrap().1, json!({"percent":100}));
        assert_eq!(one("heading 270").unwrap().1, json!({"heading":270}));
        assert_eq!(one("turn heading south").unwrap().1, json!({"heading":180}));
        assert_eq!(
            one("change heading to northeast").unwrap().1,
            json!({"heading":45})
        );
        // Whisper's false first-person opener is a QUIRK, not narration: the
        // pilot meant a turn ("So I'm heading north" for "turn heading
        // north", 2026-07-12). These parse as commands; the real question
        // form ("is that heading north?") is caught upstream by its opener.
        assert_eq!(
            one("so i'm heading south").unwrap().1,
            json!({"heading": 180})
        );
        assert_eq!(
            one("we're heading east now").unwrap().1,
            json!({"heading": 90})
        );
        assert_eq!(one("i am heading 270").unwrap().1, json!({"heading": 270}));
        // "Keep current heading" holds what we're flying — it beeped live
        // (2026-07-12) with no way to say it.
        assert_eq!(
            one("keep current heading").unwrap().1,
            json!({"current": true})
        );
        assert_eq!(
            one("hold this heading").unwrap().1,
            json!({"current": true})
        );
        assert_eq!(
            one("maintain present heading").unwrap().1,
            json!({"current": true})
        );
        // Whisper writes "sync" as "sink" — the gyro word alone carries it.
        assert_eq!(one("sink gyro").unwrap().0, "align_gyro");
        assert_eq!(one("auto sync the gyro").unwrap().0, "align_gyro");
        // A leading "and then"/"and" is pilot narration, not a compound
        // ("And then, fly heading 120." missed live twice on 2026-07-09).
        assert_eq!(
            one("and then, fly heading 120").unwrap().1,
            json!({"heading":120})
        );
        assert_eq!(
            one("and maintain heading 090").unwrap().1,
            json!({"heading":90})
        );
        // The garbled repeat still parses, so the execute-side range guard can
        // answer "say again" instead of a dead miss beep.
        assert_eq!(
            one("and then, fly heading 2700, fly heading 2700")
                .unwrap()
                .1,
            json!({"heading":2700})
        );
        // A mid-sentence "and" joining two actions is still a compound.
        assert!(one("taxi and nav lights on and flaps ten").is_none());
        // navigate (couple AP to GPS) vs nav lights
        assert_eq!(one("navigate").unwrap().0, "navigate");
        assert_eq!(one("follow the flight plan").unwrap().0, "navigate");
        assert_eq!(one("engage nav").unwrap().0, "navigate");
        // Direction of control transfer: the pilot ANNOUNCING a takeover is a
        // release (autopilot off + recovery drop), never a recovery engage —
        // "let me take controls" once flipped to the copilot seizing the plane.
        assert_eq!(one("let me take controls").unwrap().0, "autopilot_off");
        assert_eq!(one("ill take control").unwrap().0, "autopilot_off");
        assert_eq!(one("im taking control").unwrap().0, "autopilot_off");
        // ...while asking the COPILOT to take over is the crew hand-over
        // (resolved against the flight model), no longer the recovery.
        assert_eq!(one("take control").unwrap().0, TAKE_OVER);
        assert_eq!(one("please take over").unwrap().0, TAKE_OVER);
        assert_eq!(one("can you take over").unwrap().0, TAKE_OVER);
        assert_eq!(one("let me take over").unwrap().0, "autopilot_off");
        assert_ne!(
            one("take over the radios").map(|(t, _)| t).as_deref(),
            Some("recover")
        );
        assert_eq!(one("you have the aircraft").unwrap().0, TAKE_OVER);
        // Talk-down arm/cancel phrasings, including the live whisper garbles
        // ("top me down", "toadown") and the natural landing-help ask.
        assert_eq!(
            one("talk me down runway 28").unwrap(),
            ("visual_approach".into(), json!({"runway":"28"}))
        );
        assert_eq!(
            one("top me down runway 28").unwrap().1,
            json!({"runway":"28"})
        );
        assert_eq!(one("toadown, runway 28").unwrap().1, json!({"runway":"28"}));
        assert_eq!(
            one("i want to land at runway 28").unwrap().1,
            json!({"runway":"28"})
        );
        assert_eq!(one("guide me in").unwrap().1, json!({"runway":""}));
        assert_eq!(
            one("visual approach runway 10").unwrap().1,
            json!({"runway":"10"})
        );
        assert_eq!(one("stop the guidance").unwrap().1, json!({"cancel":true}));
        // Speed hold: copilot-managed power toward a target IAS.
        assert_eq!(
            one("hold 70 knots").unwrap(),
            ("hold_speed".into(), json!({"knots":70}))
        );
        assert_eq!(one("maintain 65 knots").unwrap().1, json!({"knots":65}));
        assert_eq!(one("keep the speed at 80").unwrap().1, json!({"knots":80}));
        assert_eq!(
            one("can you maintain an appropriate speed").unwrap().1,
            json!({"preset":"approach"})
        );
        assert_eq!(
            one("maintain approach speed").unwrap().1,
            json!({"preset":"approach"})
        );
        assert_eq!(one("hold this speed").unwrap().1, json!({}));
        assert_eq!(one("stop holding speed").unwrap().1, json!({"off":true}));
        assert_eq!(one("my throttle").unwrap().1, json!({"off":true}));
        // Release phrasings observed live while the pilot fought to let go.
        assert_eq!(one("mic throttle").unwrap().1, json!({"off":true}));
        assert_eq!(one("my throat").unwrap().1, json!({"off":true}));
        assert_eq!(one("my power").unwrap().1, json!({"off":true}));
        // Taxi phrasings — the tool picks taxi mode from being on the ground.
        assert_eq!(
            one("taxi at 10 knots").unwrap(),
            ("hold_speed".into(), json!({"knots":10}))
        );
        assert_eq!(one("taxi 12 knots").unwrap().1, json!({"knots":12}));
        assert_eq!(one("6? Taxi at 20 knots.").unwrap().1, json!({"knots":20}));
        assert_eq!(one("hold short").unwrap().1, json!({"knots":0}));
        // STOP brakes to a halt and holds; "stop taxiing" means stop the
        // AIRCRAFT, not drop the hold and coast (2026-07-11).
        assert_eq!(one("stop taxiing").unwrap().1, json!({"knots":0}));
        assert_eq!(one("Stop!").unwrap().1, json!({"knots":0}));
        assert_eq!(one("stop stop stop").unwrap().1, json!({"knots":0}));
        assert_eq!(one("hold position").unwrap().1, json!({"knots":0}));
        assert_eq!(one("stop here").unwrap().1, json!({"knots":0}));
        // GO resumes the remembered pace; bare only as the whole utterance,
        // and "go around" is the tower's word, never a taxi resume.
        assert_eq!(one("go").unwrap().1, json!({"resume":true}));
        assert_eq!(one("let's go").unwrap().1, json!({"resume":true}));
        assert_eq!(one("continue taxi").unwrap().1, json!({"resume":true}));
        assert!(one("go around").is_none_or(|(tool, _)| tool != "hold_speed"));
        // "taxi light on" must still be the lights, not a taxi hold.
        assert_eq!(one("taxi light on").unwrap().0, "set_lights");
        // Auto taxi — the copilot steering is a different tool, and a pace
        // in the phrase rides along instead of falling into hold_speed.
        assert_eq!(one("taxi for me").unwrap(), ("auto_taxi".into(), json!({})));
        assert_eq!(one("you have the taxi").unwrap().0, "auto_taxi");
        assert_eq!(one("take us to the holding point").unwrap().0, "auto_taxi");
        assert_eq!(
            one("taxi for me at 12 knots").unwrap().1,
            json!({"knots":12})
        );
        assert_eq!(one("auto taxi").unwrap().1, json!({}));
        assert_eq!(one("cancel auto taxi").unwrap().1, json!({"off":true}));
        assert_eq!(one("stop the auto taxi").unwrap().1, json!({"off":true}));
        // "let's taxi ..." peels its leading "lets", so these arrive as the
        // bare destination form — a request, not a readback, because no
        // specifics are being echoed (both went unanswered live).
        assert_eq!(one("let's taxi to holding point").unwrap().0, "auto_taxi");
        assert_eq!(
            one("let's taxi to the holding point").unwrap().0,
            "auto_taxi"
        );
        assert_eq!(one("lets taxi").unwrap().0, "auto_taxi");
        assert_eq!(one("taxi us to the holding point").unwrap().0, "auto_taxi");
        // The pilot's taxi READBACK to the tower must never arm the drive.
        assert!(
            one("taxi to holding point alpha one runway two seven via bravo")
                .is_none_or(|(tool, _)| tool != "auto_taxi")
        );
        // Live warts (2026-07-20/22): "the"-less destination, and whisper's
        // renderings of "taxi for me".
        assert_eq!(one("take us to holding point").unwrap().0, "auto_taxi");
        assert_eq!(one("tax you for me").unwrap().0, "auto_taxi");
        assert_eq!(one("okay taxi foamy").unwrap().0, "auto_taxi");
        // Taxi to PARKING (arrival) — distinct from the departure auto_taxi.
        assert_eq!(
            one("taxi to parking").unwrap(),
            ("taxi_to_parking".into(), json!({}))
        );
        assert_eq!(one("take us to the stand").unwrap().0, "taxi_to_parking");
        assert_eq!(one("taxi to the gate").unwrap().0, "taxi_to_parking");
        assert_eq!(one("park us").unwrap().0, "taxi_to_parking");
        assert_eq!(one("park the aircraft").unwrap().0, "taxi_to_parking");
        assert_eq!(one("taxi us in").unwrap().0, "taxi_to_parking");
        // ...but the departure taxi to a holding point is still auto_taxi.
        assert_eq!(one("taxi for me").unwrap().0, "auto_taxi");
        // Auto takeoff — first/second-person shapes only.
        assert_eq!(
            one("take off for me").unwrap(),
            ("auto_takeoff".into(), json!({}))
        );
        assert_eq!(one("you have the takeoff").unwrap().0, "auto_takeoff");
        assert_eq!(one("your takeoff").unwrap().0, "auto_takeoff");
        assert_eq!(one("auto takeoff").unwrap().1, json!({}));
        assert_eq!(one("take us off").unwrap().0, "auto_takeoff");
        assert_eq!(one("fly the takeoff").unwrap().0, "auto_takeoff");
        // Whisper's live renderings of the second-person shapes (2026-07-20).
        assert_eq!(one("you have to take off").unwrap().0, "auto_takeoff");
        assert_eq!(one("you'll take off").unwrap().0, "auto_takeoff");
        assert_eq!(one("use handle to take off").unwrap().0, "auto_takeoff");
        assert_eq!(one("abort takeoff").unwrap().1, json!({"off":true}));
        assert_eq!(one("reject the takeoff").unwrap().1, json!({"off":true}));
        assert_eq!(one("cancel the takeoff").unwrap().1, json!({"off":true}));
        assert_eq!(one("abort").unwrap().1, json!({"off":true}));
        assert_eq!(one("abort abort abort").unwrap().1, json!({"off":true}));
        // Auto circuit — the whole pattern in one word. Request shapes route;
        // a compound "take off and ..." goes here, not to a bare takeoff.
        assert_eq!(
            one("do a circuit").unwrap(),
            ("full_auto".into(), json!({"right": false}))
        );
        assert_eq!(one("fly a circuit").unwrap().0, "full_auto");
        assert_eq!(one("fly the pattern").unwrap().0, "full_auto");
        assert_eq!(one("take off and land").unwrap().0, "full_auto");
        assert_eq!(one("take off and come back around").unwrap().0, "full_auto");
        assert_eq!(one("circuits and bumps").unwrap().0, "full_auto");
        assert_eq!(
            one("make a right circuit").unwrap(),
            ("full_auto".into(), json!({"right": true}))
        );
        assert_eq!(
            one("cancel the circuit").unwrap(),
            ("full_auto".into(), json!({"off": true}))
        );
        assert_eq!(one("stop the circuit").unwrap().0, "full_auto");
        // A plain takeoff must NOT be swallowed by the circuit rule.
        assert_eq!(one("take off for me").unwrap().0, "auto_takeoff");
        // Full auto — the whole flight from cold. "start up and fly a circuit"
        // must route here, not to the circuit rule.
        assert_eq!(
            one("full auto").unwrap(),
            ("full_auto".into(), json!({"right": false}))
        );
        assert_eq!(one("do everything").unwrap().0, "full_auto");
        assert_eq!(one("start up and fly a circuit").unwrap().0, "full_auto");
        assert_eq!(one("cancel full auto").unwrap().1, json!({"off": true}));
        // The SimBrief flight-plan mission — full auto with the plan as the
        // mission. Request-shaped: "fly ..." arms it, load/brief never do.
        assert_eq!(
            one("fly my flight plan").unwrap(),
            ("full_auto".into(), json!({"flight": true}))
        );
        assert_eq!(
            one("fly the flight plan").unwrap().1,
            json!({"flight": true})
        );
        assert_eq!(
            one("fly the simbrief flight").unwrap().1,
            json!({"flight": true})
        );
        assert_eq!(
            one("fly my simbrief plan").unwrap().1,
            json!({"flight": true})
        );
        assert_eq!(
            one("full auto fly the flight plan").unwrap().1,
            json!({"flight": true})
        );
        assert_eq!(
            one("cancel the flight plan flying").unwrap().1,
            json!({"off": true})
        );
        // Load and brief shapes never arm the flight (upstream deterministic
        // paths own them; the grammar must not steal them either).
        assert!(one("load the flight plan")
            .is_none_or(|(tool, args)| tool != "full_auto" || args != json!({"flight": true})));
        assert!(one("what is our flight plan")
            .is_none_or(|(tool, args)| tool != "full_auto" || args != json!({"flight": true})));
        // A bare "full auto" stays a circuit, never silently a cross-country.
        assert_eq!(one("full auto").unwrap().1, json!({"right": false}));
        // Fly a real published procedure — approach and departure.
        assert_eq!(
            one("fly the approach").unwrap(),
            ("fly_procedure".into(), json!({"kind": "approach"}))
        );
        assert_eq!(one("fly the ILS").unwrap().0, "fly_procedure");
        assert_eq!(one("shoot the approach").unwrap().0, "fly_procedure");
        assert_eq!(one("fly me in").unwrap().0, "fly_procedure");
        assert_eq!(
            one("fly the arrival").unwrap().1,
            json!({"kind": "approach"})
        );
        assert_eq!(
            one("fly the departure").unwrap(),
            ("fly_procedure".into(), json!({"kind": "departure"}))
        );
        assert_eq!(one("fly the SID").unwrap().1, json!({"kind": "departure"}));
        assert_eq!(
            one("cancel the approach").unwrap(),
            ("fly_procedure".into(), json!({"off": true}))
        );
        // The ATC clearance readback carries "cleared ILS approach ... report
        // established" — it must NEVER arm the follower (the safety trap).
        assert!(
            one("cleared ILS approach runway two seven report established")
                .is_none_or(|(tool, _)| tool != "fly_procedure")
        );
        assert!(one("established on the ILS").is_none_or(|(tool, _)| tool != "fly_procedure"));
        // A circuit ask stays with the circuit orchestrator, not the procedure.
        assert_eq!(one("fly a circuit").unwrap().0, "full_auto");
        // Line up — the maneuver between the hold and the takeoff.
        assert_eq!(one("line us up").unwrap(), ("line_up".into(), json!({})));
        assert_eq!(
            one("can you line me up to the runway").unwrap().0,
            "line_up"
        );
        assert_eq!(one("line up for me").unwrap().0, "line_up");
        assert_eq!(one("get us lined up").unwrap().0, "line_up");
        assert_eq!(one("take us onto the runway").unwrap().0, "line_up");
        // "let's line up with/to the runway" peels its "lets" and lands on
        // the runway-suffixed shape (three phrasings missed live, 2026-07-20).
        assert_eq!(one("lets line up with the runway").unwrap().0, "line_up");
        assert_eq!(one("line up with the runway").unwrap().0, "line_up");
        // The LUAW readback and its short echo must never drive us on.
        assert!(one("line up and wait runway two seven").is_none_or(|(tool, _)| tool != "line_up"));
        assert!(one("lining up runway two seven").is_none_or(|(tool, _)| tool != "line_up"));
        // Backtrack: the full-length line-up (LXGB 09 needs it, 2026-07-22).
        assert_eq!(
            one("backtrack").unwrap(),
            ("line_up".into(), json!({"backtrack": true}))
        );
        assert_eq!(
            one("backtrack runway zero nine").unwrap().1,
            json!({"backtrack": true})
        );
        assert_eq!(one("back track for me").unwrap().0, "line_up");
        // The -ing echo and a wait-carrying readback never arm it.
        assert!(one("backtracking runway zero nine").is_none_or(|(tool, _)| tool != "line_up"));
        assert!(
            one("backtrack runway zero nine and wait").is_none_or(|(tool, _)| tool != "line_up")
        );
        // "disengage brakes" rode the bare-"disengage" AP rule into
        // autopilot_off (live 2026-07-20); the named subject wins now.
        assert_eq!(
            one("disengage brakes").unwrap(),
            ("hold_brakes".into(), json!({"on": false}))
        );
        assert_eq!(one("release breaks").unwrap().1, json!({"on": false}));
        assert_eq!(one("disengage").unwrap().0, "autopilot_off");
        // Bare "qnh <hpa>" (the gate ate "q and h 1014" live; the grammar
        // must hold up its half once it gets through).
        assert_eq!(one("q and h 1014").unwrap().0, "set_barometer");
        // A change verb with a com-band decimal needs no radio word...
        assert_eq!(
            one("change to 125.655").unwrap(),
            ("set_com1".into(), json!({"mhz": 125.655}))
        );
        // ...but named non-radio subjects stay out of the tuner.
        assert!(one("change heading to 122.5").is_none_or(|(tool, _)| tool != "set_com1"));
        // Electrical/engine mangles (live 2026-07-20/22).
        assert_eq!(one("electronics systems on").unwrap().0, "set_electrical");
        assert_eq!(one("electrical system zone").unwrap().0, "set_electrical");
        assert_eq!(one("engine is on").unwrap().0, "start_engine");
        // Truncated trailing "on" heard as "are", and the bare noun: default ON.
        assert_eq!(
            one("electrical systems are").unwrap().1,
            json!({"system": "all", "on": true})
        );
        assert_eq!(
            one("electrical systems").unwrap().1,
            json!({"system": "all", "on": true})
        );
        // ...but an explicit OFF still wins over the default.
        assert_eq!(
            one("electrical systems off").unwrap().1,
            json!({"system": "all", "on": false})
        );
        // The READBACK and the call to the tower must never arm it.
        assert!(one("cleared for takeoff runway two seven")
            .is_none_or(|(tool, _)| tool != "auto_takeoff"));
        assert!(one("ready for takeoff").is_none_or(|(tool, _)| tool != "auto_takeoff"));
        assert!(one("ready for departure").is_none_or(|(tool, _)| tool != "auto_takeoff"));
        // ...and prop/flap takeoff settings stay with their levers (the
        // prop phrasing resolves to set_prop on airframes that carry one —
        // pinned by prop_lever tests — and must never arm the takeoff).
        assert!(one("props to take off").is_none_or(|(tool, _)| tool != "auto_takeoff"));
        assert!(one("flaps to takeoff").is_none_or(|(tool, _)| tool != "auto_takeoff"));
        // ...but altitude/heading holds stay with the autopilot rules.
        assert_eq!(one("climb and maintain 7000").unwrap().0, "set_altitude");
        // Bare relative throttle nudges (both went silent live).
        assert_eq!(one("increase power").unwrap().1, json!({"delta":10}));
        assert_eq!(one("increase throttle").unwrap().1, json!({"delta":10}));
        assert_eq!(one("reduce power").unwrap().1, json!({"delta":-10}));
        // "increase power to 50 percent" is a target, not +50.
        assert_eq!(
            one("increase power to 50 percent").unwrap().1,
            json!({"percent":50})
        );
        // ILS setup, both live phrasings: with a runway and without.
        assert_eq!(
            one("set up ils for runway 28").unwrap(),
            ("prepare_ils".into(), json!({"runway":"28", "couple": false}))
        );
        assert_eq!(one("can we set up ils").unwrap().1, json!({"runway":"", "couple": false}));
        assert_eq!(
            one("set up the ils runway two four left").unwrap().1,
            json!({"runway":"24L", "couple": false})
        );
        assert_eq!(
            one("couple the approach").unwrap(),
            ("prepare_ils".into(), json!({"runway":"", "couple": true}))
        );
        assert_eq!(one("arm the approach").unwrap().0, "prepare_ils");
        assert_eq!(one("couple the ils runway 9").unwrap().1, json!({"runway":"09", "couple": true}));
        assert_eq!(
            one("prepare the ils for runway 10").unwrap().1,
            json!({"runway":"10", "couple": false})
        );
        // AP-coupling phrasings, both missed live before "navigation mode" landed.
        assert_eq!(one("set ap to nav").unwrap().0, "navigate");
        assert_eq!(one("set autopilot to navigation").unwrap().0, "navigate");
        assert_eq!(one("navigation mode").unwrap().0, "navigate");
        // "nav lights on" must still be lights, not the autopilot.
        assert_eq!(one("nav lights on").unwrap().0, "set_lights");
        assert_eq!(one("nav lights on").unwrap().0, "set_lights");
        assert_eq!(one("heading north west").unwrap().1, json!({"heading":315}));
        assert_eq!(
            one("fly heading southeast").unwrap().1,
            json!({"heading":135})
        );
        assert_eq!(one("descend 500").unwrap().1, json!({"fpm":-500}));
        assert_eq!(one("climb 500").unwrap().1, json!({"fpm":500}));
        assert_eq!(
            one("avionics on").unwrap(),
            (
                "set_electrical".into(),
                json!({"system":"avionics","on":true})
            )
        );
        // Numbered split systems (SR22 BAT 1/2, ALT 1/2); a bare noun stays
        // unnumbered for the tool to resolve against the profile.
        assert_eq!(
            one("alternator two on").unwrap(),
            (
                "set_electrical".into(),
                json!({"system":"alternator2","on":true})
            )
        );
        assert_eq!(
            one("battery 1 off").unwrap().1,
            json!({"system":"battery1","on":false})
        );
        assert_eq!(
            one("alternator on").unwrap().1,
            json!({"system":"alternator","on":true})
        );
        // Inverter (C90 AC power) — resolves like any other electrical system.
        assert_eq!(
            one("inverter on").unwrap(),
            (
                "set_electrical".into(),
                json!({"system":"inverter","on":true})
            )
        );
        assert_eq!(
            one("inverter off").unwrap(),
            (
                "set_electrical".into(),
                json!({"system":"inverter","on":false})
            )
        );
        // EFIS power switch (C90 EADI/EHSI displays).
        assert_eq!(
            one("efis power on").unwrap(),
            ("set_electrical".into(), json!({"system":"efis","on":true}))
        );
        assert_eq!(
            one("efis on").unwrap(),
            ("set_electrical".into(), json!({"system":"efis","on":true}))
        );
        // Bleed air (Twin Otter pneumatics) — routes like any electrical key.
        assert_eq!(
            one("bleed air on").unwrap(),
            ("set_electrical".into(), json!({"system":"bleed","on":true}))
        );
        assert_eq!(
            one("bleed air off").unwrap(),
            (
                "set_electrical".into(),
                json!({"system":"bleed","on":false})
            )
        );
        // Reverse thrust needs the airframe to HAVE it: on the 172 the phrase
        // stays a miss (capability gate), never a phantom lever.
        assert!(one("reverse thrust").is_none());
        // "electrics on" both spoken cleanly and via whisper's "electric zone(s)" /
        // "electric system(s)" slips — all resolve to the whole-system power call.
        let elec = (
            "set_electrical".to_string(),
            json!({"system":"all","on":true}),
        );
        assert_eq!(one("electrics on").unwrap(), elec);
        assert_eq!(one("electric systems on").unwrap(), elec);
        assert_eq!(one("electric zones").unwrap(), elec);
        assert_eq!(one("electric zone").unwrap(), elec);
        assert_eq!(one("electric system zones").unwrap(), elec);
        // "Electric song?" — the same "s on" glue heard as a word (live 2026-07-11).
        assert_eq!(one("Electric song?").unwrap(), elec);
        // "power up" has no on/off token but must still resolve deterministically
        // (the main loop remaps set_electrical:all → power_up on auto_prep airframes).
        assert_eq!(
            one("power up").unwrap(),
            ("set_electrical".into(), json!({"system":"all","on":true}))
        );
        assert_eq!(one("powering up the aircraft").unwrap().0, "set_electrical");
        assert_eq!(
            one("landing lights off").unwrap(),
            ("set_lights".into(), json!({"light":"landing","on":false}))
        );
        // RELATIVE throttle: "by N" adjusts, never sets the absolute value (a
        // live power chop: "reduce throttle by ten percent" went to 10%).
        assert_eq!(
            one("reduce throttle by ten percent").unwrap(),
            ("set_throttle".into(), json!({"delta":-10}))
        );
        // Whisper's full garble heard live: plural "powers", past-tense
        // "increased", and "of" for "to" — still a 60% target, not a +60 delta.
        assert_eq!(
            one("increased powers of 60 percent").unwrap(),
            ("set_throttle".into(), json!({"percent":60}))
        );
        assert_eq!(
            one("increase power by 10 percent").unwrap().1,
            json!({"delta":10})
        );
        // "N to M" means M: whisper garbled "throttle to fifty" as "12 to 50
        // percent" live and the first-number rule chopped power to 12%.
        assert_eq!(
            one("throttle 12 to 50 percent").unwrap().1,
            json!({"percent":50})
        );
        assert_eq!(
            one("power 12 to 50 percent").unwrap().1,
            json!({"percent":50})
        );
        // ...but a dangling "to" with no second number keeps the first.
        assert_eq!(
            one("throttle 40 to, taxi").unwrap().1,
            json!({"percent":40})
        );
        assert_eq!(
            one("increase power by 20 percent").unwrap().1,
            json!({"delta":20})
        );
        assert_eq!(
            one("reduce throttle to 20 percent").unwrap().1,
            json!({"percent":20})
        );
        // Tank management: "switch tanks" feeds from the other wing (SR22 routine);
        // an explicit side always wins over the swap verb.
        assert_eq!(
            one("switch tanks").unwrap(),
            ("set_fuel_selector".into(), json!({"position":"switch"}))
        );
        assert_eq!(
            one("swap fuel tanks").unwrap().1,
            json!({"position":"switch"})
        );
        assert_eq!(
            one("switch to the left tank").unwrap().1,
            json!({"position":"left"})
        );
        assert_eq!(
            one("select the right tank").unwrap().1,
            json!({"position":"right"})
        );
        // Bare "fuel" phrasings, no "tank"/"selector" said (seen live: these read
        // back the quantity report instead of moving the selector).
        assert_eq!(
            one("switch fuel to right").unwrap(),
            ("set_fuel_selector".into(), json!({"position":"right"}))
        );
        assert_eq!(
            one("set fuel to left").unwrap().1,
            json!({"position":"left"})
        );
        assert_eq!(one("switch fuel").unwrap().1, json!({"position":"switch"}));
        // ...but the pump keeps its own tool, and a bare quantity mention parses
        // as nothing here (stays a question for the fuel report).
        assert_eq!(one("switch fuel pump on").unwrap().0, "set_fuel_pump");
        // The centre tank pumps: the 737's CTR pumps left running dry over an
        // empty centre (2026-09-27). Their own tool, never the boost pumps'.
        for (said, on) in [
            ("Centre pumps off.", false),
            ("center pumps off", false),
            ("turn off the centre pumps", false),
            ("Turn the center fuel pumps off.", false),
            ("switch the CTR pumps off", false),
            ("centre pumps on", true),
            ("Center pumps on.", true),
        ] {
            assert_eq!(
                one_on("boeing_737_800", said),
                Some(("set_centre_pumps".into(), json!({ "on": on }))),
                "{said}"
            );
        }
        // Registered everywhere, so the 172 answers why it cannot.
        assert_eq!(one("centre pumps off").unwrap().0, "set_centre_pumps");
        assert!(one("how much fuel do we have").is_none());
        assert_eq!(
            one("strobes off").unwrap().1,
            json!({"light":"strobe","on":false})
        );
        assert_eq!(one("start the engine").unwrap().0, "start_engine");
        // Politeness/filler stripped by the normalization layer (Cessna registry,
        // so use systems it has — flaps/engine/lights).
        assert_eq!(
            one("could you start the engines").unwrap().0,
            "start_engine"
        );
        assert_eq!(one("flaps up please").unwrap().1, json!({"notch":"up"}));
        assert_eq!(one("okay now flaps ten").unwrap().0, "set_flaps");
        assert_eq!(
            one("i'd like flaps full").unwrap().1,
            json!({"notch":"full"})
        );
        assert_eq!(
            one("go ahead and turn on the nav lights").unwrap().0,
            "set_lights"
        );
        assert_eq!(
            normalize("Could you please set the heading to 270?"),
            "set the heading to 270"
        );
        assert_eq!(normalize("flaps up please"), "flaps up");
        // Plural / natural start phrasings the King Air run surfaced as misses.
        assert_eq!(one("start the engines").unwrap().0, "start_engine");
        assert_eq!(one("start up").unwrap().0, "start_engine");
        assert_eq!(one("fire them up").unwrap().0, "start_engine");
        assert_eq!(one("stop the engines").unwrap().0, "stop_engine");
        // Whisper's spurious apostrophe ("engine's on" for "engines on") — the
        // exact miss from the C90B panel. normalize() strips it.
        assert_eq!(one("engine's on").unwrap().0, "start_engine");
        assert_eq!(one("engines on").unwrap().0, "start_engine");
        assert_eq!(one("align gyro").unwrap().0, "align_gyro");
        assert_eq!(one("think gyro").unwrap().0, "align_gyro"); // mishear handled
        assert_eq!(one("level off").unwrap().0, "level_off");
        assert_eq!(one("autopilot off").unwrap().0, "autopilot_off");
        assert_eq!(
            one("parking brake set").unwrap(),
            ("set_parking_brake".into(), json!({"on":true}))
        );
    }

    #[test]
    fn altimeter_accepts_digit_runs_and_hpa() {
        // Input is post-normalization, which only inserts a decimal for a spoken
        // "point" — so "set altimeter two niner niner two" arrives as "2992".
        let inhg = |t: &str| -> f64 {
            let (name, args) = one(t).unwrap_or_else(|| panic!("no parse: {t:?}"));
            assert_eq!(name, "set_barometer", "{t:?}");
            args["inhg"].as_f64().expect("inhg is a number")
        };
        assert!((inhg("set altimeter 2992") - 29.92).abs() < 0.005); // digit run
        assert!((inhg("altimeter 29.92") - 29.92).abs() < 0.005); // explicit decimal
        assert!((inhg("set baro 3010") - 30.10).abs() < 0.005); // 4-digit inHg×100
        assert!((inhg("set qnh 1013") - 29.91).abs() < 0.02); // hPa/QNH → inHg
                                                              // Whisper's spelling of spoken "Q-N-H": the "en" comes back as "and"/"&"
                                                              // (both seen live 2026-07-12). normalize() rewrites them to "qnh".
        assert!((inhg("set q and h 1013") - 29.91).abs() < 0.02);
        assert!((inhg("q&h 1013") - 29.91).abs() < 0.02);
        assert!((inhg("set q n h 1013") - 29.91).abs() < 0.02);
        // A bare integer outside both the inHg and hPa ranges isn't a setting —
        // fall back to the LLM rather than write garbage (and not the auto-setter).
        assert!(one("set altimeter 50").is_none());
        // No value spoken → set to the current local QNH automatically.
        assert_eq!(one("set the altimeter").unwrap().0, "set_altimeter_current");
        assert_eq!(
            one("give me the local altimeter").unwrap().0,
            "set_altimeter_current"
        );
        assert_eq!(
            one("altimeter to field").unwrap().0,
            "set_altimeter_current"
        );
        assert_eq!(one("set the local qnh").unwrap().0, "set_altimeter_current");
    }

    #[test]
    fn ground_ops_resolve() {
        assert_eq!(
            one("chocks out").unwrap(),
            ("set_chocks".into(), json!({"on":false}))
        );
        assert_eq!(
            one("set the chocks").unwrap(),
            ("set_chocks".into(), json!({"on":true}))
        );
        assert_eq!(one("remove the chocks").unwrap().1, json!({"on":false}));
        assert_eq!(
            one("request ground power").unwrap(),
            ("set_ground_power".into(), json!({"on":true}))
        );
        assert_eq!(
            one("disconnect ground power").unwrap().1,
            json!({"on":false})
        );
        assert_eq!(
            one("push back").unwrap(),
            ("pushback".into(), json!({"action":"straight"}))
        );
        assert_eq!(one("pushback left").unwrap().1, json!({"action":"left"}));
        assert_eq!(one("stop pushback").unwrap().1, json!({"action":"stop"}));
        for refusal in [
            "skip pushback",
            "cancel pushback, just fly",
            "no pushback",
            "don't push back",
            "forget the pushback",
        ] {
            assert_eq!(
                one(refusal).unwrap().1,
                json!({"action":"stop"}),
                "{refusal}"
            );
        }
        assert_eq!(one("request ground services").unwrap().0, "request_service");
        assert_eq!(one("refuel").unwrap().0, "request_service");
        // "ground power" must NOT be grabbed by the throttle branch (it has "power").
        assert_ne!(one("ground power on").unwrap().0, "set_throttle");
    }

    #[test]
    fn stt_mishears_and_glued_words_still_resolve() {
        // Live-flight gaps: "floodlights" glued, "brake" heard as "break".
        assert_eq!(one("floodlights on").unwrap().0, "set_flood_lights");
        assert_eq!(
            one("turn the floodlights on").unwrap().1,
            json!({"on":true})
        );
        assert_eq!(
            one("parking break off").unwrap(),
            ("set_parking_brake".into(), json!({"on":false}))
        );
        // Roof floods, deterministically — the copilot sets them itself after
        // dark, and the pilot must still be able to say so without the LLM.
        assert_eq!(
            one("roof lights on").unwrap(),
            ("set_lights".into(), json!({"light":"roof","on":true}))
        );
        assert_eq!(
            one("roof lights off").unwrap(),
            ("set_lights".into(), json!({"light":"roof","on":false}))
        );
        assert_eq!(
            one("turn off the roof lights").unwrap(),
            ("set_lights".into(), json!({"light":"roof","on":false}))
        );
        // The panel floods are a DIFFERENT switch and must not be swallowed by
        // the roof rule: the Baron has both, and they are wired independently.
        assert_eq!(one("flood lights on").unwrap().0, "set_flood_lights");
        assert_eq!(one("parking break set").unwrap().1, json!({"on":true}));
    }

    #[test]
    fn regular_brakes_resolve_and_never_steal_other_levers() {
        assert_eq!(
            one("brakes on").unwrap(),
            ("hold_brakes".into(), json!({"on":true}))
        );
        assert_eq!(one("hold the brakes").unwrap().1, json!({"on":true}));
        assert_eq!(
            one("brakes off").unwrap(),
            ("hold_brakes".into(), json!({"on":false}))
        );
        assert_eq!(one("release the brakes").unwrap().1, json!({"on":false}));
        // The same "break" mishear the parking brake gets.
        assert_eq!(one("breaks on").unwrap().0, "hold_brakes");
        assert_eq!(one("break off").unwrap().1, json!({"on":false}));
        // The word "parking" keeps its own lever; speedbrakes keep theirs
        // (on the 172 registry the speedbrake tool doesn't exist, so the
        // phrase parses to it and filters to None — the point is it never
        // falls through to hold_brakes).
        assert_eq!(one("parking brake off").unwrap().0, "set_parking_brake");
        assert!(one("speed brakes out").is_none_or(|(n, _)| n == "speedbrake"));
        // Bare "brakes" with no verb is ambiguous: fall through, never guess.
        assert!(one("brakes").is_none());
    }

    /// EGSS, 2026-09-22: a typed "hold two two zero knots" armed the governor
    /// at 2 + 2 + 0 = 4 knots. Digit words said in a row are digits.
    #[test]
    fn digit_words_in_a_row_are_one_number() {
        assert_eq!(number("hold two two zero knots"), Some(220));
        assert_eq!(number("heading zero niner zero"), Some(90));
        assert_eq!(number("two hundred and forty"), Some(240));
        assert_eq!(number("twenty two"), Some(22));
        assert_eq!(number("two thousand five hundred"), Some(2500));
        assert_eq!(number("1 thousands"), Some(1000), "LILB 2026-09-25: set 1 fpm");
        assert_eq!(
            one("hold two two zero knots").unwrap(),
            ("hold_speed".into(), json!({"knots": 220}))
        );
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
        assert_eq!(
            one("heading two hundred and forty").unwrap().1,
            json!({"heading":240})
        );
        // but a genuine multi-action compound still goes to the LLM.
        assert_eq!(one("flaps ten and gear up"), None);
    }

    #[test]
    fn explicit_vertical_speed_is_never_an_altitude_target() {
        // Regression: "vertical speed" commands were grabbed by set_altitude when
        // they said "to X", or named a rate above the 2500 fpm disambiguation cut.
        let vs = |s: &str| one(s).unwrap();
        assert_eq!(
            vs("vertical speed descent three thousand"),
            ("set_vertical_speed".into(), json!({"fpm":-3000}))
        );
        assert_eq!(
            vs("vertical speed 3000 minus"),
            ("set_vertical_speed".into(), json!({"fpm":-3000}))
        );
        assert_eq!(
            vs("set vertical speed to minus one thousand"),
            ("set_vertical_speed".into(), json!({"fpm":-1000}))
        );
        assert_eq!(
            vs("vertical speed negative five hundred"),
            ("set_vertical_speed".into(), json!({"fpm":-500}))
        );
        assert_eq!(
            vs("vertical speed to seven hundred"),
            ("set_vertical_speed".into(), json!({"fpm":700}))
        );
        // Bare climb/descend WITHOUT "vertical" still disambiguates to an altitude
        // target on "to X" or a value too big to be a rate.
        assert_eq!(one("climb 7000").unwrap().0, "set_altitude");
        assert_eq!(
            one("climb to 7000").unwrap(),
            ("set_altitude".into(), json!({"altitude":7000}))
        );
    }

    #[test]
    fn altitude_target_vs_vertical_speed_rate() {
        // "to X" / big number / "maintain" → altitude target
        assert_eq!(
            one("descend to one thousand two hundred feet").unwrap(),
            ("set_altitude".into(), json!({"altitude":1200}))
        );
        assert_eq!(
            one("climb to seven thousand").unwrap(),
            ("set_altitude".into(), json!({"altitude":7000}))
        );
        assert_eq!(
            one("climb and maintain seven thousand").unwrap(),
            ("set_altitude".into(), json!({"altitude":7000}))
        );
        assert_eq!(
            one("descend and maintain two thousand").unwrap(),
            ("set_altitude".into(), json!({"altitude":2000}))
        );
        // Whisper renders large numbers with a thousands separator — must not
        // parse "7,000" as 70 (a dangerous altitude misread).
        assert_eq!(
            one("climb and maintain 7,000").unwrap(),
            ("set_altitude".into(), json!({"altitude":7000}))
        );
        assert_eq!(one("climb 7000").unwrap().0, "set_altitude"); // too big to be a VS rate
                                                                  // no "to", in-range number → vertical-speed rate
        assert_eq!(
            one("descend 500").unwrap(),
            ("set_vertical_speed".into(), json!({"fpm":-500}))
        );
        assert_eq!(
            one("climb at five hundred").unwrap(),
            ("set_vertical_speed".into(), json!({"fpm":500}))
        );
    }

    #[test]
    fn ambiguous_is_a_miss_not_a_guess() {
        assert_eq!(one("a bit more flaps"), None); // relative
        assert_eq!(one("taxi and nav lights on"), None); // compound
        assert_eq!(one("switch it back to both"), None); // relative
        assert_eq!(one("nice weather today"), None); // not a command
        assert_eq!(one("what's our heading"), None); // question
    }

    #[test]
    fn invalid_flap_value_routes_to_tool_for_a_helpful_reply() {
        // The Cessna 172 has no 25-degree detent. We don't fabricate it, but we DO
        // route to set_flaps so the tool can answer "I have up, 10, 20, full"
        // rather than a bare "Say again?". (The tool rejects the value at runtime.)
        assert_eq!(
            one("flaps 25").unwrap(),
            ("set_flaps".into(), json!({ "notch": "25" }))
        );
    }

    #[test]
    fn declines_autopilot_when_the_airframe_has_none() {
        // The AWX DC-3 has no autopilot — AP commands must decline honestly, not
        // fall through to "Say again?".
        let prof = aircraft::load("dc3").expect("dc3 profile");
        let reg = Registry::for_aircraft(&prof);
        assert!(declined("climb to four thousand feet", &reg).is_some());
        assert!(declined("autopilot on", &reg).is_some());
        assert!(declined("turn heading north", &reg).is_some());
        assert!(declined("descend to 3000", &reg).is_some());
        // The Cessna HAS an autopilot — no decline (it acts on these).
        let (creg, _) = setup();
        assert!(declined("climb to 4000", &creg).is_none());
        assert!(declined("heading 270", &creg).is_none());
        // A non-AP command never declines here (handled elsewhere).
        assert!(declined("flaps up", &reg).is_none());
    }

    /// At the Heathrow stand, 2026-09-17, the pilot asked seven ways over
    /// forty-five seconds and nothing happened, then gave up with "my
    /// aircraft". Every phrasing parsed — to `stop_engine`, remapped to
    /// `stop_engines`, which the 737's registry had never built: the
    /// scripted-start branch added the START and not the stop. An airframe
    /// you can start is an airframe you can shut down.
    #[test]
    fn the_737_can_be_shut_down_in_the_words_the_pilot_used() {
        let prof = aircraft::load("boeing_737_800").expect("737 profile");
        let reg = Registry::for_aircraft(&prof);
        assert!(reg.get("start_engines").is_some(), "the scripted start");
        assert!(reg.get("stop_engines").is_some(), "and the way back");
        assert!(
            reg.get("stop_engine").is_none(),
            "a jet has no single lever"
        );
        for words in [
            "let's shut down",
            "let's turn engines off",
            "engines off",
            "turn engines off",
            "shut down aircraft",
            "shut down",
            "turn off engines",
            "cut the engines",
        ] {
            assert_eq!(
                one_on("boeing_737_800", words).map(|(n, _)| n),
                Some("stop_engines".to_string()),
                "{words}"
            );
        }
    }

    #[test]
    fn declines_carb_heat_on_a_fuel_injected_engine() {
        // The 172SP is an IO-360: fuel injected, no carburettor, no knob on the
        // panel (verified 2026-09-13). Asking for carb heat must be declined by
        // name, not answered with "Say again?" or a write to carb_heat_ratio.
        let prof = aircraft::load("cessna_172sp").expect("172 profile");
        let reg = Registry::for_aircraft(&prof);
        assert!(reg.get("set_carb_heat").is_none(), "IO-360s are injected");
        assert!(declined("carb heat on", &reg).is_some());
        assert!(declined("carburettor heat off", &reg).is_some());
        // The DC-3's radials DO have it — no decline there.
        let dc3 = aircraft::load("dc3").expect("dc3 profile");
        assert!(declined("carb heat on", &Registry::for_aircraft(&dc3)).is_none());
    }

    #[test]
    fn word_detents_parse() {
        // Word-named detents (King Air approach/down, DC-3 quarter/half/three-
        // quarter) must parse to their label, not miss. `valid()` lets them through
        // even on the Cessna registry (the tool validates per-airframe at runtime).
        assert_eq!(
            one("flaps approach").unwrap().1,
            json!({ "notch": "approach" })
        );
        assert_eq!(one("flaps down").unwrap().1, json!({ "notch": "down" }));
        assert_eq!(one("flaps half").unwrap().1, json!({ "notch": "half" }));
        assert_eq!(
            one("flaps quarter").unwrap().1,
            json!({ "notch": "quarter" })
        );
        assert_eq!(
            one("flaps three quarters").unwrap().1,
            json!({ "notch": "three-quarter" })
        );
    }

    #[test]
    fn emergency_recover_and_release_resolve() {
        // Distress phrases → recover; a crew hand-over → take_over (which
        // recovers only in an actual upset); calm "level off" stays level_off.
        assert_eq!(one("recover the aircraft").unwrap().0, "recover");
        assert_eq!(one("stabilize").unwrap().0, "recover");
        assert_eq!(one("save it").unwrap().0, "recover");
        assert_eq!(one("help").unwrap().0, "recover");
        for words in [
            "take control",
            "take the controls",
            "you have control",
            "your controls",
            "your aircraft",
            "you have it",
            "you fly",
            "take over",
        ] {
            assert_eq!(one(words).unwrap().0, TAKE_OVER, "{words}");
        }
        // The pilot taking it: a release, whatever the verb.
        for words in [
            "I'll taxi",
            "I will taxi",
            "let me taxi",
            "I'll fly",
            "let me fly",
            "I'll take it",
            "I will take it",
        ] {
            assert_eq!(one(words).unwrap().0, "autopilot_off", "{words}");
        }
        // "Will you take over?" is the copilot's, not the pilot's.
        assert_eq!(one("will you take over").unwrap().0, TAKE_OVER);
        assert_eq!(one("level off").unwrap().0, "level_off");
        // Handing it back → autopilot_off (which also aborts an active recovery).
        assert_eq!(one("my controls").unwrap().0, "autopilot_off");
        assert_eq!(one("i have the aircraft").unwrap().0, "autopilot_off");
    }

    /// The pilot's own words for "fly the flight plan" (LIME 2026-09-25:
    /// "I don't like that command").
    #[test]
    fn the_flight_is_armed_in_a_pilots_words() {
        let (n, a) = one("Take us to Bologna.").expect("take us to");
        assert_eq!(n, "full_auto");
        assert_eq!(a["flight"], json!(true));
        assert_eq!(a["to"].as_str(), Some("bologna"));
        let (_, a) = one("fly us to lima india papa echo").expect("spelled");
        assert_eq!(a["to_codes"], json!(["LIPE"]));
        assert_eq!(one("start the flight").unwrap(), ("full_auto".into(), json!({"flight": true})));
        assert_eq!(one("Begin the flight.").unwrap(), ("full_auto".into(), json!({"flight": true})));
        assert_eq!(one("Resume.").unwrap(), ("full_auto".into(), json!({"resume": true})));
        assert_eq!(one("I said resume.").unwrap(), ("full_auto".into(), json!({"resume": true})));
        for w in ["Resume.", "I said resume.", "Begin the flight.", "Take us to Bologna."] {
            assert!(crate::router::looks_like_command(w), "the gate drops {w}");
        }
        assert_eq!(one("continue the flight").unwrap(), ("full_auto".into(), json!({"resume": true})));
        assert!(one("take us to the runway").is_none_or(|(n, _)| n != "full_auto"));
        assert!(one("take us to five thousand feet").is_none_or(|(n, _)| n != "full_auto"));
    }

    #[test]
    fn autoland_arm_and_cancel_resolve() {
        assert_eq!(
            one("land the aircraft").unwrap(),
            ("autoland".into(), json!({}))
        );
        assert_eq!(one("get us down").unwrap(), ("autoland".into(), json!({})));
        assert_eq!(
            one("emergency landing").unwrap(),
            ("autoland".into(), json!({}))
        );
        assert_eq!(one("autoland").unwrap(), ("autoland".into(), json!({})));
        assert_eq!(
            one("cancel the autoland").unwrap(),
            ("autoland".into(), json!({"cancel":true}))
        );
        assert_eq!(
            one("stop the autoland").unwrap(),
            ("autoland".into(), json!({"cancel":true}))
        );
    }

    #[test]
    fn prop_levers_resolve_on_the_king_air() {
        let prof = aircraft::load("king_air_c90b").expect("c90b");
        let reg = Registry::for_aircraft(&prof);
        let g = |t: &str| {
            try_parse(t, &reg, &prof).map(|mut v| {
                let c = v.remove(0);
                (c.name, c.arguments)
            })
        };
        // Named detents (the many spoken words fold to full/cruise/low).
        assert_eq!(
            g("props full forward").unwrap(),
            ("set_prop".into(), json!({"setting":"full"}))
        );
        assert_eq!(
            g("props up").unwrap(),
            ("set_prop".into(), json!({"setting":"full"}))
        );
        assert_eq!(
            g("set the props to cruise").unwrap(),
            ("set_prop".into(), json!({"setting":"cruise"}))
        );
        assert_eq!(
            g("props low").unwrap(),
            ("set_prop".into(), json!({"setting":"low"}))
        );
        assert_eq!(
            g("coarse prop pitch").unwrap(),
            ("set_prop".into(), json!({"setting":"low"}))
        );
        // Explicit RPM wins over any word in the phrase.
        assert_eq!(
            g("prop rpm 1900").unwrap(),
            ("set_prop".into(), json!({"rpm":1900}))
        );
        assert_eq!(
            g("props to 1600").unwrap(),
            ("set_prop".into(), json!({"rpm":1600}))
        );
        assert_eq!(
            g("set the props 2000").unwrap(),
            ("set_prop".into(), json!({"rpm":2000}))
        );
        // Whisper mishears "prop" as "prope"/"propp" — a token starting with "prop"
        // still resolves (this was a live miss on "Prope RPM 1900").
        assert_eq!(
            g("prope rpm 1900").unwrap(),
            ("set_prop".into(), json!({"rpm":1900}))
        );
        assert_eq!(
            g("propp full forward").unwrap(),
            ("set_prop".into(), json!({"setting":"full"}))
        );
        // A bare prop command with no target routes to the tool (which asks which
        // setting) rather than falling through to a miss.
        assert_eq!(g("set props").unwrap(), ("set_prop".into(), json!({})));
        // "appropriate" must NOT be read as a prop command (prefix match is
        // token-anchored, so mid-word "prop" doesn't count).
        assert!(g("set an appropriate heading 270").map(|c| c.0) != Some("set_prop".into()));
        // Live-log mishears: split "takeoff", "cruise"→"Cruz", "props"→"crops".
        assert_eq!(
            g("props to take off").unwrap(),
            ("set_prop".into(), json!({"setting":"full"}))
        );
        assert_eq!(
            g("props to cruz").unwrap(),
            ("set_prop".into(), json!({"setting":"cruise"}))
        );
        assert_eq!(
            g("crops cruise").unwrap(),
            ("set_prop".into(), json!({"setting":"cruise"}))
        );
        // "prop flow" must NOT read as "prop low" (whole-word fixes the "flow"⊃"low"
        // substring bug) — it asks which setting instead of guessing 1600.
        assert_eq!(g("prop flow").unwrap(), ("set_prop".into(), json!({})));
    }

    /// The plugin panel's buttons inject English through the request channel
    /// ([`crate::auto_status::REQUEST_PHRASES`] + the CANCEL button's
    /// context phrases) — every one must parse HERE, deterministically, or a
    /// button press would fall through to the LLM (or to nothing). Pins the
    /// whole panel button surface to the grammar.
    #[test]
    fn panel_request_phrases_parse_deterministically() {
        let expect: &[(i64, &str)] = &[
            (1, "recover"),
            (2, "autoland"),
            (3, "autopilot_off"),
            (4, "full_auto"),
            (5, "full_auto"),
            (6, "pushback"),
            (7, "auto_taxi"),
            (8, "line_up"),
            (9, "auto_takeoff"),
            (10, "full_auto"),
            (12, "load_aircraft"),
        ];
        assert_eq!(expect.len(), crate::auto_status::REQUEST_PHRASES.len());
        for (code, phrase) in crate::auto_status::REQUEST_PHRASES {
            let tool = expect
                .iter()
                .find(|(c, _)| c == code)
                .map(|(_, t)| *t)
                .unwrap_or_else(|| panic!("no expectation for request code {code}"));
            assert_eq!(
                one(phrase).map(|(t, _)| t).as_deref(),
                Some(tool),
                "request code {code}: {phrase:?}"
            );
        }
        // The CANCEL button (code 11): every phrase cancel_phrase() can pick.
        assert_eq!(
            one("cancel full auto").unwrap(),
            ("full_auto".into(), json!({"off": true}))
        );
        assert_eq!(
            one("cancel autoland").unwrap(),
            ("autoland".into(), json!({"cancel": true}))
        );
        assert_eq!(
            one("abort takeoff").unwrap(),
            ("auto_takeoff".into(), json!({"off": true}))
        );
        assert_eq!(
            one("cancel the circuit").unwrap(),
            ("full_auto".into(), json!({"off": true}))
        );
        assert_eq!(
            one("cancel the approach").unwrap(),
            ("fly_procedure".into(), json!({"off": true}))
        );
        assert_eq!(
            one("stop pushback").unwrap(),
            ("pushback".into(), json!({"action": "stop"}))
        );
        assert_eq!(
            one("cancel auto taxi").unwrap(),
            ("auto_taxi".into(), json!({"off": true}))
        );
        // ...and the nothing-running fallback: CANCEL is never a dead button.
        // After the missed-approach give-up every ACTIVE flag was false while
        // the AP still flew the aircraft dirty and slow, and CANCEL mapped to
        // None and did nothing, silently (EGHI, 2026-08-02).
        assert_eq!(one("my controls").unwrap().0, "autopilot_off");
    }
    #[test]
    fn a_plan_check_is_its_own_ask() {
        for yes in [
            "check the flight plan",
            "Is the plan safe?",
            "any problems with the route",
            "is the aircraft suitable for this flight plan",
            "can this aircraft do the plan",
            "verify the route please",
        ] {
            assert!(plan_check_request(yes), "{yes}");
        }
        for no in [
            "fly the flight plan",
            "load the flight plan",
            "before takeoff checklist",
            "what's the flight plan",
            "check the flaps",
        ] {
            assert!(!plan_check_request(no), "{no}");
        }
    }

    #[test]
    fn a_waiver_is_the_whole_utterance_and_nothing_else() {
        for yes in [
            "Fly it anyway.",
            "Play it anyway.",
            "Yes, it's fine. Let's go ahead.",
            "Yes.",
            "Go ahead",
            "Okay, fly it anyway",
            "I've checked the runway.",
            "I have checked the weight",
            "Override.",
            "copilot, my call",
        ] {
            assert!(super::waiver_request(yes), "{yes}");
        }
        for no in [
            "fly it",
            "fly the flight plan",
            "override the autopilot",
            "check the runway",
            "is the runway long enough",
            "anyway",
            "okay",
            "yes, flaps up",
            "go ahead and lower the gear",
            "no",
        ] {
            assert!(!super::waiver_request(no), "{no}");
        }
    }
}
