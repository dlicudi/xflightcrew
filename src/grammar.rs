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
//! recurring phrasings (see `xfc analyze`).

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
        // "Fold flaps." then "Full flaps." and "Can you set up ALS?" then
        // "Can you arm ILS?" (2026-10-01): the whole phrase, so no other
        // "fold" or "als" is touched.
        ("fold flaps", "full flaps"),
        ("set up als", "set up ils"),
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
    // The pilot's go-around: a closed set of words, so a readback of the
    // tower's order (a callsign, "roger", "wilco") never lands here.
    if go_around_request(&t) {
        let c = call("go_around", json!({}));
        return valid(&c, registry, profile).then_some(vec![c]);
    }
    // Full auto (the whole flight from cold) and the circuit are each ONE
    // intent even though their phrasings carry "and" ("start up and fly", "take
    // off and land") — checked on the raw text BEFORE the compound gate would
    // split them. Full auto first: "start up and fly a circuit" contains
    // "circuit" and must not fall to the circuit rule.
    // A new flight on a final: its hand-fly clause carries "and", and its
    // runway words must never reach the mission grammar.
    if let Some(c) = approach_setup_call(&t).or_else(|| enroute_setup_call(&t)) {
        return valid(&c, registry, profile).then_some(vec![c]);
    }
    if let Some(c) = flight_plan_call(&t) {
        // A helicopter flies the plan on its own route follower: full auto
        // is withheld there. Stopping it is a hold.
        if registry.get("hover").is_some() && registry.get(&c.name).is_none() {
            let off = c.arguments.get("off").and_then(|v| v.as_bool()) == Some(true);
            let resume = c.arguments.get("resume").and_then(|v| v.as_bool()) == Some(true);
            return Some(vec![if off {
                call("hover", json!({ "action": "hold" }))
            } else {
                call("hover", json!({ "action": "route", "resume": resume }))
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
    // "Autothrottle on" is the MD-82's own tool where it has one; elsewhere
    // the stock autothrottle through the mode tool (#92).
    if c.name == "autothrottle"
        && registry.get("autothrottle").is_none()
        && registry.get("ap_mode").is_some()
    {
        let on = c.arguments.get("on").and_then(|v| v.as_bool()).unwrap_or(true);
        c = call("ap_mode", json!({ "mode": "autothrust", "on": on }));
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

/// The pilot's go-around order (#77, #102): "go around", "going around",
/// "go-around", "take it around", "missed approach", "go missed". CLOSED:
/// the phrase plus filler only ("let's go around", "we're going around
/// now"), so the tower's order read back with a callsign or a "roger", or a
/// "don't go around", is never the pilot's order. Pure.
pub fn go_around_request(t: &str) -> bool {
    const PHRASES: &[&str] = &[
        "go around",
        "going around",
        "goaround",
        "take it around",
        "take us around",
        "taking it around",
        "missed approach",
        "go missed",
        "going missed",
    ];
    const FILLER: &[&str] = &[
        "go", "going", "around", "goaround", "take", "taking", "it", "us", "missed",
        "approach", "we", "were", "are", "well", "lets", "let", "im", "i", "am", "ill",
        "do", "a", "the", "fly", "execute", "please", "now", "ok", "okay", "right",
    ];
    let t = normalize(t).replace('-', " ");
    let words: Vec<&str> = t
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    let joined = words.join(" ");
    !words.is_empty()
        && words.iter().all(|w| FILLER.contains(w))
        && PHRASES.iter().any(|p| joined.contains(p))
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
        // "Resume flight then." (LXGB 2026-10-06); "Resume slides." twice and
        // "Resume flying." (2026-10-01/02).
        "then", "please", "flying", "flights", "slides",
    ];
    let lead = t.strip_prefix("resumes ").or_else(|| t.strip_prefix("resume "));
    if let Some(rest) = lead {
        let words: Vec<&str> = rest
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|w| !w.is_empty())
            .collect();
        if !words.is_empty()
            && words.iter().all(|w| RESUME_TAIL.contains(w))
            && words.iter().any(|w| {
                matches!(
                    *w,
                    "flight" | "fight" | "now" | "like" | "right" | "flying" | "flights" | "slides"
                )
            })
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

/// "Put us enroute 30 miles before top of descent", "put us enroute 40
/// miles before the destination", "put us enroute at WELIN", "put us en
/// route at the top of descent": a NEW FLIGHT in the air on the loaded plan
/// (`setup_enroute`), for the sortie runner's arrival cases. CLOSED: a lead
/// that puts US somewhere, the word enroute, then exactly a fix (one ident
/// after "at" or "over") or a distance and what it is before.
pub fn enroute_setup_call(t: &str) -> Option<ToolCall> {
    const LEADS: &[&str] = &[
        "put us ", "put me ", "set us up ", "set me up ", "start us ", "start me ",
        "place us ", "position us ", "reposition us ",
    ];
    const MILES: &[&str] = &["mile", "miles", "nm", "mi", "nautical"];
    let t = t.trim().trim_end_matches(['.', '!', '?']);
    let lead = LEADS.iter().find(|l| t.starts_with(**l))?;
    let rest = &t[lead.len()..];
    let rest = rest.strip_prefix("enroute ").or_else(|| rest.strip_prefix("en route "))?;
    let words: Vec<&str> = rest
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    let tod = |w: &[&str]| {
        matches!(w, ["top", "of", "descent"] | ["the", "top", "of", "descent"] | ["tod"])
    };
    match words.as_slice() {
        ["at", rest @ ..] if tod(rest) => {
            return Some(call("setup_enroute", json!({ "at": "before_tod", "nm": 0 })));
        }
        ["at" | "over", id]
            if (2..=7).contains(&id.len())
                && id.chars().any(|c| c.is_ascii_alphabetic())
                && !matches!(*id, "the" | "top" | "cruise") =>
        {
            let fix = id.to_ascii_uppercase();
            return Some(call("setup_enroute", json!({ "at": "fix", "fix": fix })));
        }
        _ => {}
    }
    let m = words.iter().position(|w| MILES.contains(w))?;
    let numeric =
        |w: &str| w.parse::<i64>().is_ok() || word_value(w).is_some_and(|(_, mult)| !mult);
    if m == 0 || m > 3 || !words[..m].iter().all(|w| numeric(w)) {
        return None;
    }
    let nm = number(&words[..m].join(" "))?;
    let mut tail = &words[m + 1..];
    if tail.first() == Some(&"miles") {
        tail = &tail[1..];
    }
    let (first, what) = tail.split_first()?;
    if !matches!(*first, "before" | "from" | "out" | "short" | "to") {
        return None;
    }
    let what = match what {
        ["of", w @ ..] | ["from", w @ ..] => w,
        w => w,
    };
    let at = if tod(what) {
        "before_tod"
    } else if matches!(what, ["destination"] | ["the", "destination"]) {
        "before_destination"
    } else {
        return None;
    };
    Some(call("setup_enroute", json!({ "at": at, "nm": nm })))
}

/// Is this the ask for a new flight in the air ([`enroute_setup_call`])?
pub fn enroute_setup_request(t: &str) -> bool {
    enroute_setup_call(&normalize(t)).is_some()
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

/// The anti-ice grammar (#100). `None` without an ice word and a state.
fn anti_ice_call(t: &str) -> Option<ToolCall> {
    let on = onoff(t)?;
    let anti = (has(t, "anti") && (has(t, "ice") || has(t, "icing")))
        || has(t, "antiice")
        || has(t, "deice")
        || (has(t, "de") && has(t, "ice"))
        || t.contains("ice protection");
    let window = (has(t, "window") || has(t, "windshield") || has(t, "windscreen"))
        && (has(t, "heat") || anti);
    let prop = (has(t, "prop") || has(t, "props") || has(t, "propeller"))
        && (has(t, "heat") || anti);
    let boots = has(t, "boots") || has(t, "boot");
    let engine = (has(t, "engine") || has(t, "engines") || has(t, "inlet") || has(t, "cowl"))
        && (anti || (has(t, "inlet") && has(t, "heat")));
    let wing = (has(t, "wing") || has(t, "wings")) && anti;
    let system = if window {
        "window"
    } else if prop {
        "prop"
    } else if boots || wing {
        "wing"
    } else if engine {
        "engine"
    } else if anti {
        "all"
    } else {
        return None;
    };
    Some(call("set_anti_ice", json!({ "system": system, "on": on })))
}

/// "flight level one two zero" (or "fl120", "fl 120", "level 120") said as
/// the feet it means, "12000 feet", so every altitude rule below reads a
/// level the way it reads an altitude (issue #97). Summed as digits it was
/// "climb 120": a 120 fpm climb, or an altitude of 120 ft. `None` when no
/// level (10 to 600) is named.
pub fn flight_level_as_feet(t: &str) -> Option<String> {
    let toks: Vec<&str> = t.split_whitespace().collect();
    let clean = |w: &str| {
        w.trim_matches(|c: char| !c.is_ascii_alphanumeric())
            .to_ascii_lowercase()
    };
    let digits_from = |start: usize| -> (String, usize) {
        let mut d = String::new();
        let mut j = start;
        while j < toks.len() && d.len() < 3 {
            let w = clean(toks[j]);
            if !w.is_empty() && w.chars().all(|c| c.is_ascii_digit()) {
                if d.len() + w.len() > 3 {
                    break;
                }
                d.push_str(&w);
            } else if let Some((v, false)) = crate::phrase::word_value(&w).filter(|(v, m)| !m && *v <= 9) {
                d.push_str(&v.to_string());
            } else {
                break;
            }
            j += 1;
        }
        (d, j)
    };
    let level = |d: &str| -> Option<i64> {
        let n: i64 = d.parse().ok()?;
        (d.len() >= 2 && (10..=600).contains(&n)).then_some(n * 100)
    };
    for i in 0..toks.len() {
        let w = clean(toks[i]);
        let (from, ft, to) = if w == "level" || w == "fl" {
            let (d, j) = digits_from(i + 1);
            let from = if w == "level" && i > 0 && clean(toks[i - 1]) == "flight" { i - 1 } else { i };
            (from, level(&d), j)
        } else if let Some(rest) = w.strip_prefix("fl").filter(|r| !r.is_empty() && r.chars().all(|c| c.is_ascii_digit())) {
            (i, level(rest), i + 1)
        } else {
            continue;
        };
        if let Some(ft) = ft {
            let mut out: Vec<String> = toks[..from].iter().map(|w| w.to_string()).collect();
            out.push(format!("{ft} feet"));
            out.extend(toks[to..].iter().map(|w| w.to_string()));
            return Some(out.join(" "));
        }
    }
    None
}

/// One line as every matcher sees it: the text with any flight level
/// already spelled as feet, and its first number.
pub(super) struct Line<'a> {
    pub(super) t: &'a str,
    pub(super) num: Option<i64>,
    stopped: std::cell::Cell<bool>,
}

impl Line<'_> {
    /// A deliberate miss: the line is this matcher's and means nothing
    /// ("flaps" with no detent), so no later matcher may take it.
    pub(super) fn stop(&self) -> Option<ToolCall> {
        self.stopped.set(true);
        None
    }
}

/// The matchers, tried in THIS order: the first that answers wins, so
/// the order is the grammar's semantics ("ground power" before the
/// throttle's "power", the prop levers before the throttle, the parking
/// brake before the brakes). A domain whose rules sit apart in the order
/// has one entry per run.
const MATCHERS: &[fn(&Line) -> Option<ToolCall>] = &[
    reminders::reminder,
    engines::lean_mixture,
    ground::ramp,
    radio::tune,
    surfaces::config,
    autopilot::modes,
    automation::ground_drive,
    speed::speed,
    engines::levers,
    autopilot::heading,
    approach::approach,
    autopilot::vertical,
    systems::systems,
    engines::reverse_and_start,
    instruments::gyro,
    handover::handover,
    ground::brakes,
    fuel::selector,
    radio::squawk,
    surfaces::trim,
    instruments::altimeter,
];

fn parse_one(t: &str) -> Option<ToolCall> {
    let fl_text = flight_level_as_feet(t);
    let t = fl_text.as_deref().unwrap_or(t);
    let line = Line {
        t,
        num: number(t),
        stopped: std::cell::Cell::new(false),
    };
    for m in MATCHERS {
        if let Some(c) = m(&line) {
            return Some(c);
        }
        if line.stopped.get() {
            return None;
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

mod approach;
mod automation;
mod autopilot;
mod engines;
mod fuel;
mod ground;
mod handover;
mod instruments;
mod radio;
mod reminders;
mod speed;
mod surfaces;
mod systems;

#[cfg(test)]
mod tests;
