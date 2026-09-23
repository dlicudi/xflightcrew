//! Utterance routing: the closed vocabularies that decide whether a transcript
//! is a command, a question, a follow-up, a bare value, a readback echo or a
//! confirmation, before any tool is picked. Split out of main.rs on 2026-09-16.

use super::*;

/// Negation words — a reply containing any of these is never an affirmative.
pub(crate) const NEGATIONS: &[&str] = &[
    "negative",
    "cancel",
    "abort",
    "stop",
    "belay",
    "disregard",
    "wrong",
    "no",
];

/// What to do with a spoken (non-radio) command given its STT confidence — the
/// three-band model that replaces a hard act/drop cutoff.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum ConfidenceAction {
    /// Confident enough — act on it outright.
    Act,
    /// Marginal, but a deliberate command the grammar recognises — read it back
    /// for a yes/no before acting, rather than dropping a recoverable capture.
    Confirm,
    /// A deliberate command we couldn't make out — ask for it again.
    SayAgain,
    /// Not deliberate (continuous-mode ambient, or crew off) — ignore silently.
    Ignore,
}

/// Classify a spoken command by confidence. `deliberate` = a PTT/wake command
/// with the crew active (so there's someone to answer). `grammar_maps` = the
/// deterministic grammar recognised it (we never hand a marginal reading to the
/// LLM). The middle `[floor, min)` band is where a recognised deliberate command
/// earns a read-back-and-confirm instead of being dropped.
pub(crate) fn classify_confidence(
    confidence: f32,
    min: f32,
    floor: f32,
    deliberate: bool,
    grammar_maps: bool,
) -> ConfidenceAction {
    if confidence >= min {
        return ConfidenceAction::Act;
    }
    if !deliberate {
        return ConfidenceAction::Ignore;
    }
    if confidence >= floor && grammar_maps {
        ConfidenceAction::Confirm
    } else {
        ConfidenceAction::SayAgain
    }
}

/// Is a spoken reply an affirmative confirmation? Defaults to `false` (safe) —
/// only an explicit yes proceeds; an explicit no, or anything unclear, cancels.
pub(crate) fn is_affirmative(reply: &str) -> bool {
    let r = reply.to_lowercase();
    const YES: &[&str] = &[
        "affirm",
        "affirmative",
        "confirm",
        "yes",
        "yeah",
        "yep",
        "roger",
        "correct",
        "copy",
        "proceed",
        "go ahead",
        "do it",
        "right",
        "indeed",
        "okay",
        "absolutely",
    ];
    if NEGATIONS.iter().any(|w| r.contains(w)) {
        return false;
    }
    YES.iter().any(|w| r.contains(w))
}

/// If a non-affirmative reply still carries content beyond negation words,
/// return that content as a correction ("negative, 500" → "500"). A bare
/// "negative" returns `None` — a plain cancel.
pub(crate) fn correction_text(reply: &str) -> Option<String> {
    let kept: Vec<&str> = reply
        .split_whitespace()
        .filter(|w| {
            let norm: String = w
                .chars()
                .filter(|c| c.is_alphanumeric())
                .flat_map(|c| c.to_lowercase())
                .collect();
            !norm.is_empty() && !NEGATIONS.contains(&norm.as_str())
        })
        .collect();
    let s = kept.join(" ");
    let s = s.trim();
    if s.is_empty() {
        None
    } else {
        Some(s.to_string())
    }
}

/// The pilot's recovery words, and which line they ask for.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub(crate) enum RecallAsk {
    /// "Say again" / "repeat that": the newest thing the copilot said, or was
    /// saying when it was cut off.
    Newest,
    /// "What were you saying?": the line the pilot keyed over (or the radio
    /// stepped on), even when a readback has been said since.
    Interrupted,
}

/// Is this utterance a request to hear the copilot again? `t` is normalized.
pub(crate) fn recall_ask(t: &str) -> Option<RecallAsk> {
    if t.contains("were you saying")
        || t.contains("you were saying")
        || t.contains("you were going to say")
        || t.contains("you were about to say")
    {
        return Some(RecallAsk::Interrupted);
    }
    (t.contains("say again")
        || t.contains("repeat")
        || t.contains("come again")
        || t.contains("what did you say")
        || t.contains("didnt catch that"))
    .then_some(RecallAsk::Newest)
}

/// A question about the time or distance to the DESTINATION ("when do we
/// land", "time to destination", "how far before we land", "how much to
/// land?"). The answer (`eta_to_destination`) gives both, minutes and miles,
/// so a distance cue is as good as a time cue. Until 2026-09-23 it needed a
/// time word: "how far before we land?" fell to the next-WAYPOINT branch
/// ("Next waypoint NIE, 11 miles", with ETND 34 miles out) and "how much to
/// land?" missed. Never a fuel or runway question ("how much fuel will we
/// land with", "how much runway to land").
pub(crate) fn eta_question(tl: &str) -> bool {
    let destination = tl.contains("land")
        || tl.contains("destination")
        || tl.contains("arrive")
        || tl.contains("arrival")
        || tl.contains("get there")
        || tl.contains("get in")
        || tl.contains("en route")
        || tl.contains("enroute")
        || tl.contains("on route");
    let time = tl.contains("time")
        || tl.contains("eta")
        || tl.contains("when")
        || tl.contains("how long")
        || tl.contains("estimat")
        || tl.contains("remaining");
    let distance = tl.contains("how far")
        || tl.contains("how much")
        || tl.contains("how many miles")
        || tl.contains("distance");
    let other = ["fuel", "runway", "gear", "flap", "light", "weight", "speed"]
        .iter()
        .any(|w| tl.contains(w));
    destination && !other && (time || distance)
}

/// "What did the tower say?" / "what was that from ATC?": the crew asked
/// what the CONTROLLER said, not the copilot. `t` is normalized.
pub(crate) fn atc_recall_request(t: &str) -> bool {
    const WHO: &[&str] = &[
        "tower",
        "atc",
        "approach",
        "departure",
        "controller",
        "ground",
    ];
    let names_atc = t
        .split(|c: char| !c.is_alphanumeric())
        .any(|w| WHO.contains(&w));
    names_atc
        && (t.contains("what did") && t.contains("say")
            || t.contains("what was that")
            || t.contains("what were we told")
            || t.contains("did you catch"))
}

/// How long the crew can report the controller's last word to us. Longer
/// than the controller's own repeat: the crew remembers a clearance.
pub(crate) const ATC_RECALL_TTL: Duration = Duration::from_secs(300);

/// The crew's answer to "what did the tower say?", from the controller's
/// last transmission addressed to us as it went out on the air. Grounded:
/// with external ATC the crew hears nothing and says so; with nothing on
/// file it says it heard nothing, never a guess.
pub(crate) fn atc_recall_line(
    simulated_atc: bool,
    last: Option<&crate::voice::ControllerLine>,
    now: Instant,
) -> String {
    if !simulated_atc {
        return "I can't hear the real controller. Ask them to say again.".to_string();
    }
    match last.filter(|l| now.duration_since(l.at) < ATC_RECALL_TTL) {
        Some(l) => {
            let who = match l.speaker {
                crate::voice::StationSpeaker::Approach => "Approach",
                crate::voice::StationSpeaker::Departure => "Departure",
                _ => "Tower",
            };
            format!("{who} said: {}", l.text)
        }
        None => "Nothing from ATC in the last five minutes.".to_string(),
    }
}

/// Owed lines said again at most this many at a time: a recovery, not a
/// monologue. The newest are the ones kept.
const RECALL_MAX_LINES: usize = 3;

/// The line to say for a recovery ask, and whether it spends the owed
/// (`unsaid`) lines. `unsaid` are copilot lines cut before the end, oldest
/// first; `readback` the last reply to a pilot request; `volunteered` the last
/// line the copilot offered unasked. `None`: there is nothing to say again,
/// and the copilot says so rather than inventing a line.
pub(crate) fn recall_line(
    ask: RecallAsk,
    unsaid: &[(String, Instant)],
    readback: Option<&(String, Instant)>,
    volunteered: Option<&(String, Instant)>,
) -> Option<(String, bool)> {
    let owed = (!unsaid.is_empty()).then(|| {
        let from = unsaid.len().saturating_sub(RECALL_MAX_LINES);
        let text = unsaid[from..]
            .iter()
            .map(|(t, _)| t.trim())
            .collect::<Vec<_>>()
            .join(" ");
        (text, unsaid.last().map(|(_, at)| *at).expect("non-empty"))
    });
    if ask == RecallAsk::Interrupted {
        if let Some((text, _)) = owed {
            return Some((text, true));
        }
    }
    // Newest of the three, by when it was said (or cut).
    let mut best: Option<(String, Instant, bool)> = None;
    let mut consider = |text: &str, at: Instant, spends: bool| {
        if best.as_ref().is_none_or(|(_, b, _)| at > *b) {
            best = Some((text.to_string(), at, spends));
        }
    };
    if let Some((text, at)) = &owed {
        consider(text, *at, true);
    }
    if let Some((text, at)) = readback {
        consider(text, *at, false);
    }
    if let Some((text, at)) = volunteered {
        consider(text, *at, false);
    }
    best.map(|(text, _, spends)| (text, spends))
}

/// Deterministic conversational glue — the cockpit chatter that isn't a command or
/// a state question: "what can you do", "say again", and acknowledgments. Returns
/// the spoken reply, or `None` to fall through to the normal command/question path.
/// `t` is already normalized (filler stripped). Capabilities are read live from the
/// registry, so the answer matches the actual aircraft.
pub(crate) fn conversational(t: &str, registry: &Registry) -> Option<String> {
    let has = |w: &str| t.split(|c: char| !c.is_alphanumeric()).any(|tok| tok == w);
    // "say again" / "what were you saying" are answered before this, from the
    // voice's memory (`recall_ask` / `recall_line`).
    // "what can you do" / "help" / "what can I ask" → list live capabilities.
    // Bare "help me" only: "help me land on runway 28" is a request for that
    // specific help (the talk-down), not a menu (it got the menu, live).
    if t.contains("what can you do")
        || t.contains("what can i")
        || t.contains("what do you do")
        || t.contains("what are you able")
        || t == "help me"
        || t == "help"
        || t.contains("capabilities")
        || t.contains("what commands")
    {
        let mut caps: Vec<&str> = Vec::new();
        let any = |names: &[&str]| names.iter().any(|n| registry.get(n).is_some());
        if any(&["set_flaps"]) {
            caps.push("flaps");
        }
        if any(&["gear"]) {
            caps.push("gear");
        }
        if any(&["set_throttle"]) {
            caps.push("throttle");
        }
        if any(&["start_engines", "start_engine"]) {
            caps.push("engine start and shutdown");
        }
        if any(&["set_heading", "set_altitude"]) {
            caps.push("the autopilot");
        }
        if any(&["set_com1", "set_nav1"]) {
            caps.push("radios");
        }
        if any(&["set_transponder_mode"]) {
            caps.push("the transponder");
        }
        if any(&["set_lights", "set_flood_lights"]) {
            caps.push("lights");
        }
        if any(&["set_electrical", "set_power"]) {
            caps.push("electrical");
        }
        if any(&["set_fuel_selector"]) {
            caps.push("fuel selectors");
        }
        if any(&["set_barometer"]) {
            caps.push("the altimeter");
        }
        let list = match caps.len() {
            0 => "the basics".to_string(),
            1 => caps[0].to_string(),
            _ => format!(
                "{}, and {}",
                caps[..caps.len() - 1].join(", "),
                caps[caps.len() - 1]
            ),
        };
        return Some(format!(
            "I can work the {list}. I can also read back the altitude, heading, speed, fuel, and weather. Just say what you want."
        ));
    }
    // Acknowledgments / pleasantries / presence — reply briefly, take no action.
    if has("thanks") || t.contains("thank you") || t.contains("appreciate it") || t == "cheers" {
        return Some("You're welcome.".to_string());
    }
    if has("roger") || has("copy") || has("wilco") || t.contains("copy that") {
        return Some("Standing by.".to_string());
    }
    // Bare praise ("good", "nice", "perfect") only as a SHORT standalone ack — so a
    // longer question like "is the weather good" isn't hijacked.
    let words = t.split_whitespace().count();
    if words <= 3
        && (has("good") || has("nice") || has("great") || has("perfect") || t.contains("well done"))
    {
        return Some("Standing by.".to_string());
    }
    if t.contains("you there")
        || t.contains("are you there")
        || t.contains("you with me")
        || t == "hello"
        || t == "hi"
        || t.contains("you awake")
        || t.contains("you copy")
    {
        return Some("Standing by, go ahead.".to_string());
    }
    None
}

/// What the pilot hears when the grammar names a tool the registry never
/// built for this airframe. On a rotorcraft the fixed-wing laws are withheld
/// on purpose ([`tools::ROTORCRAFT_WITHHELD`]), and "isn't wired up yet"
/// sends the pilot looking for a bug; say why instead (S-76C, 2026-09-17).
/// Anywhere else the generic line stands.
pub(crate) fn unregistered_tool_line(profile: &aircraft::AircraftProfile, tool: &str) -> String {
    if profile.rotorcraft && tools::ROTORCRAFT_WITHHELD.contains(&tool) {
        "Not on a helicopter. Every law I have for that is fixed-wing, so it isn't offered on this airframe."
            .to_string()
    } else if !profile.autopilot && tools::ROTORCRAFT_WITHHELD.contains(&tool) {
        // The 747-400: its scripts own the autopilot, so nothing that flies
        // through the stock dials registers. "Let's do a circuit" got the
        // generic line and the pilot loaded another aircraft to find out why
        // (EGPF, 2026-09-21).
        "Not on this airframe. I can't reach its autopilot yet, so the automations aren't offered here."
            .to_string()
    } else {
        "That command isn't wired up yet.".to_string()
    }
}

/// Human label for the on-screen callout: "FLAPS → 10", "GEAR → DOWN", "PROPS →
/// CRUISE". Generic: the tool name minus set_, plus the argument values.
pub(crate) fn callout_label(tool: &str, args: &serde_json::Value) -> String {
    let name = tool
        .trim_start_matches("set_")
        .replace('_', " ")
        .to_uppercase();
    let vals: Vec<String> = args
        .as_object()
        .map(|o| {
            o.values()
                .map(|v| match v {
                    serde_json::Value::String(s) => s.to_uppercase(),
                    serde_json::Value::Bool(b) => {
                        if *b {
                            "ON".into()
                        } else {
                            "OFF".into()
                        }
                    }
                    other => other.to_string(),
                })
                .collect()
        })
        .unwrap_or_default();
    if vals.is_empty() {
        name
    } else {
        format!("{name} > {}", vals.join(" "))
    }
}

/// Release-shaped speech: does this utterance plausibly mean "give me back the
/// controls"? Deliberately generous — it is ONLY consulted while the copilot
/// actually holds a control authority (speed hold etc.), where the cost of a
/// false release is a lever going still and the cost of a missed release is a
/// pilot fighting their own copilot. Seen live: "my throttle" came through as
/// "my thruffles", "mic control", "my throat", "my problem" — seven phrasings
/// in a row missed while the governor kept taxiing.
pub(crate) fn release_intent(t: &str) -> bool {
    const CUES: &[&str] = &[
        "release",
        "let go",
        "let me go",
        "stop touching",
        "stop holding",
        "stop taxi",
        "hands off",
        "leave the throttle",
        "leave it",
        "stop steering",
        "my steering",
        "my pedals",
        "my rudder",
        "give me command",
        "give me control",
        "give me the",
        "my controls",
        "my control",
        "mic control",
        "my throttle",
        "my thr",
        "my power",
        "my aircraft",
        "my plane",
        "my airplane",
        "my mixture",
        "my mix",
        "i have control",
        "i have the",
        "im taking",
        "i am taking",
        "taking control",
    ];
    CUES.iter().any(|c| t.contains(c))
}

/// Example commands shown in the dashboard's "say" panel.
/// Aviation vocabulary that a real command contains at least one of. Used to
/// reject phonetic nonsense that whisper transcribes with high confidence
/// ("Nautilapuku") before it reaches the LLM, which would otherwise force-match
/// it to the nearest tool. Whole-word, case-insensitive.
pub(crate) const COMMAND_KEYWORDS: &[&str] = &[
    // a helicopter's hover ("lift to a hover" was dropped here as "not a
    // command", twice, on the S-76's first hover, 2026-09-22)
    "hover",
    "liftoff",
    // radio calls to the ATC station ("request taxi"/"radio"/"takeoff"/
    // "departure" are already covered below; these two aren't)
    "clearance",
    "tower",
    // flaps / gear / trim
    "flap",
    "flaps",
    "gear",
    "trim",
    // autopilot / vertical nav
    "autopilot",
    "heading",
    "altitude",
    "climb",
    "descend",
    "maintain",
    "level",
    "vertical",
    "vs",
    "approach",
    "ils",
    "localizer",
    "glideslope",
    "obs",
    "course",
    "radial",
    "runway",
    // vertical-nav synonyms whisper actually produced that the gate missed:
    // "ascend 500" (for climb), "physical speed 500" (mishear of vertical speed).
    "ascend",
    "ascent",
    "speed",
    // airliner autoflight: AP speed select + autothrottle
    "airspeed",
    "autothrottle",
    // speedbrakes / spoilers
    "speedbrake",
    "speedbrakes",
    "spoiler",
    "spoilers",
    // reverse thrust: "thrust" is above, but the bare rollout call is
    // "Reverse thrusters!" — and "thrusters" is 3 edits from "thrust", so the
    // fuzzy match never saves it. Ate the pilot's actual landing-rollout call
    // on the Twotter's first reverse flight (2026-07-09) while the wordier
    // "set reverse power" phrasings sailed through on "power".
    "reverse",
    "reversers",
    "thrusters",
    "stow",
    // lights
    "light",
    "lights",
    "beacon",
    "strobe",
    "strobes",
    "taxi",
    "landing",
    "nav",
    "navigation",
    "flood",
    "floods",
    // engine / fuel
    "throttle",
    "thrust",
    "power",
    "idle",
    "mixture",
    "rich",
    "lean",
    "engine",
    "engines",
    "start",
    "ignition",
    "shutdown",
    "magneto",
    "magnetos",
    "mags",
    "carb",
    "carburetor",
    "fuel",
    "pump",
    "boost",
    "primer",
    "selector",
    // constant-speed props ("props", so len>=5 fuzzy also catches whisper's
    // "prope"/"propp"; "rpm" for the explicit-RPM phrasing).
    "prop",
    "props",
    "propeller",
    "propellers",
    "rpm",
    "feather",
    // electrical
    "battery",
    "alternator",
    "generator",
    "avionics",
    "electrical",
    "electric",
    "electrics",
    "bus",
    "master",
    "inverter",
    "inverters",
    "efis",
    // radios / transponder
    "com",
    "comm",
    "radio",
    "frequency",
    "tune",
    "squawk",
    "transponder",
    "ident",
    "barometer",
    "altimeter",
    "baro",
    "kollsman",
    "qnh",
    "vor",
    // instruments / misc controls ("break(s)" = the STT homophone the
    // grammar already tolerates; "release breaks" was gate-swallowed)
    "gyro",
    "pitot",
    "brake",
    "brakes",
    "break",
    "breaks",
    // "stop" is a control word, not chatter. It was missing, and the only
    // reason it ever worked was the pre-gate branch below that fires while the
    // speed governor is running — so the moment the aircraft was moving with
    // no governor, the one word a pilot actually shouts was dropped as ambient
    // with no miss chime. Southampton: a Baron rolling off its stand at 21 kt,
    // "Stop." at 0.9996 confidence, nothing (2026-08-01). The grammar's bare
    // stop already requires the WHOLE utterance to be it, so "stop for fuel"
    // still just reaches the router rather than actuating.
    "stop",
    "parking",
    "heat",
    // ramp / ground-ops: chocks, ground power, pushback, services
    "chocks",
    "chock",
    "pushback",
    "push",
    "tug",
    "gpu",
    "ground",
    "refuel",
    "service",
    "services",
    // checklist invocations: "checklist(s)" is the trigger word every one shares
    // (so "normal take-off checklist" reaches the LLM even when STT splits
    // "takeoff"), plus the phase words for the shorthand ("configure for takeoff").
    "checklist",
    "checklists",
    "takeoff",
    "configure",
    "config",
    "departure",
    "secure",
    "clean",
    "cruise",
    "preflight",
    "limits",
    "performance",
    // systems-status check + reference-limit / V-speed queries
    "systems",
    "system",
    "vne",
    "vno",
    "vfe",
    "vspeed",
    "vspeeds",
    // aircraft-info queries
    "aircraft",
    "plane",
    "cessna",
    "skyhawk",
    // SimBrief flight plan + loading the route into the GPS/FMS
    "simbrief",
    "flightplan",
    "plan",
    "route",
    "load",
    // the load sheet ("board the passengers", "set the payload")
    "payload",
    "passengers",
    "cargo",
    "program",
    "gps",
    "fms",
    // couple the autopilot to the flight plan
    "navigate",
    "follow",
    "track",
    // navigation progress queries
    "waypoint",
    "eta",
    // ETA to the destination ("time before we land", "when do we arrive")
    "land",
    "destination",
    "arrive",
    "arrival",
    // time/distance to top of descent
    "descent",
    "tod",
    // reminders ("wake me at top of descent", "remind me in 10 minutes")
    "wake",
    "remind",
    "notify",
    "heads-up",
    "reminder",
    "reminders",
    "cancel",
    // emergency abort/undo of the last action
    "belay",
    "undo",
    "abort",
    "disregard",
    "scratch",
    "revert",
    // emergency recovery / take-control + autoland handoff ("land"/"ground" already above)
    "recover",
    "stabilize",
    "stabilise",
    "control",
    "controls",
    "autoland",
    // the full-length line-up ("backtrack runway 09")
    "backtrack",
    // auto circuit — the whole traffic pattern ("do a circuit", "fly the
    // pattern", "circuits and bumps"). Without these the domain gate dropped
    // "do a circuit" as not-a-command, before the grammar ever saw it
    // (2026-07-23). "takeoff"/"land" already cover "take off and land".
    "circuit",
    "circuits",
    "pattern",
    "traffic",
    // full auto — the whole flight from cold ("full auto", "do everything",
    // "start up and fly", "fly from cold").
    "auto",
    "automated",
    "everything",
    "cold",
    "fly",
    // fly_procedure — real published SID/approach ("fly the ILS", "shoot the
    // approach", "fly the SID/arrival/departure").
    "approach",
    "ils",
    "rnav",
    "sid",
    "departure",
    "arrival",
    "shoot",
];

/// True if the utterance is a question about the flight (answered from state,
/// not actioned). Interrogative openers or a trailing "?". Kept conservative so
/// it doesn't hijack commands, but NOT so conservative it misses a question that
/// shares a command's keywords: "Where's the parking brake?" once SET the brake
/// because the contraction "where's" wasn't an opener and "parking brake" parsed
/// as a command (seen live). Every opener needs its contraction alongside, in
/// both spellings, since one caller passes raw text and another passes the
/// normalized form with apostrophes stripped. Command-ish openers like "can you
/// set…" are deliberately excluded.
pub(crate) fn looks_like_question(text: &str) -> bool {
    let t = text.trim().to_ascii_lowercase();
    if t.ends_with('?') {
        return true;
    }
    const OPENERS: &[&str] = &[
        "what",
        "what's",
        "whats",
        "how",
        "how's",
        "hows",
        "how much",
        "how many",
        "how far",
        "how high",
        "how fast",
        "where",
        "where's",
        "wheres",
        "when",
        "when's",
        "whens",
        "which",
        "why",
        "why's",
        "whys",
        "who",
        "who's",
        "whos",
        "tell me",
        // Generic auxiliaries: no command starts with these, and they must beat the
        // command grammar ("is our heading 270?" answers, never acts).
        "is",
        "are",
        "am",
        "have",
        "has",
        // Negated auxiliaries ("aren't we going a bit slow?", "isn't the gear
        // down?"): a question every time, in both the raw and the
        // apostrophe-stripped form the normaliser leaves (missed live 2026-09-12).
        "isn't",
        "isnt",
        "aren't",
        "arent",
        "wasn't",
        "wasnt",
        "weren't",
        "werent",
        "don't",
        "dont",
        "doesn't",
        "doesnt",
        "didn't",
        "didnt",
        "haven't",
        "havent",
        "hasn't",
        "hasnt",
        "shouldn't",
        "shouldnt",
        "couldn't",
        "couldnt",
        "wouldn't",
        "wouldnt",
        "won't",
        "wont",
        "can't",
        "cant",
    ];
    if OPENERS
        .iter()
        .any(|o| t == *o || t.starts_with(&format!("{o} ")))
    {
        return true;
    }
    // do-support ("do/does/did") is a question ONLY with a subject pronoun after
    // it — "do we have fuel?", "does it hold?". Followed by anything else it's an
    // imperative: "do a circuit", "do a touch and go", "do a go-around" are
    // COMMANDS, and treating the "do" opener as a question swallowed them before
    // the fast path ever ran (the circuit command missed live, 2026-07-23).
    const DO_SUBJECTS: &[&str] = &["you", "we", "i", "it", "they", "he", "she", "this", "that"];
    for aux in ["do", "does", "did", "should", "shall"] {
        if let Some(rest) = t.strip_prefix(&format!("{aux} ")) {
            return DO_SUBJECTS
                .iter()
                .any(|s| rest == *s || rest.starts_with(&format!("{s} ")));
        }
    }
    false
}

/// True if the utterance plausibly is an aircraft command (contains at least one
/// keyword). Phonetic garbage that happens to transcribe confidently shares no
/// vocabulary with the domain and is rejected here.
pub(crate) fn looks_like_command(text: &str) -> bool {
    // Two-word compounds whisper splits: keyword matching is per-token, so
    // neither half clears the gate alone — "take off for me" (vs the fused
    // keyword "takeoff") and "line us up" were both swallowed WITHOUT EVEN
    // A MISS CHIME, live 2026-07-20. Checked on the raw text first.
    let t = text.to_ascii_lowercase();
    if t.contains("take off")
        || t.contains("line up")
        || t.contains("line us up")
        || t.contains("line me up")
        || t.contains("lined up")
        || t.contains("lift off")
        || t.contains("set it down")
        || t.contains("set us down")
        || t.contains("put it down")
        || t.contains("put us down")
    {
        return true;
    }
    let token_pass = |s: &str| {
        s.split(|c: char| !c.is_ascii_alphabetic())
            .filter(|w| !w.is_empty())
            .any(|w| {
                let w = w.to_ascii_lowercase();
                if matches_keyword(&w) {
                    return true;
                }
                // Whisper sometimes glues a spoken two-word compound into one token
                // ("flood lights" → "floodlights", "landing light" → "landinglight"),
                // which then matches no standalone keyword. Re-check the stem after
                // dropping a trailing light/lights so the compound still reaches the
                // LLM. The stem goes through the same fuzzy check, so the mishear
                // "bloodlights" also passes (blood ~ flood, one edit).
                if let Some(stem) = w.strip_suffix("lights").or_else(|| w.strip_suffix("light")) {
                    return !stem.is_empty() && matches_keyword(stem);
                }
                false
            })
    };
    // Raw text first, then the normalized form: the alias table is what
    // folds "q and h 1014" into "qnh 1014" — checked only raw, four QNH
    // settings in a row were swallowed without even a miss chime (live
    // 2026-07-20, the same failure mode as the split compounds above).
    token_pass(text) || token_pass(&grammar::normalize(text))
}

/// True if an utterance looks like a *continuation* of the previous command, so
/// it can clear the gate (within the follow-up window) without a domain keyword
/// and let the LLM resolve it from history. Two shapes count:
/// - back-references / adjustments ("it", "that", "again", "more", "the other one");
/// - a bare quantity ("seventy percent", "60%", "two thousand") — almost always
///   re-stating the value of the command just given ("throttle fifty" → "seventy").
pub(crate) fn looks_like_followup(text: &str) -> bool {
    const FOLLOWUP_WORDS: &[&str] = &[
        "it", "that", "them", "those", "again", "back", "undo", "revert", "reset", "more", "less",
        "instead", "also", "other", "same", "too",
    ];
    // A SHORT bare quantity is an adjustment to the last command ("sixty
    // percent", "two thousand five hundred"). A long string of numbers ("one two
    // three … ten") is counting/noise, not a follow-up — rewriting it into a
    // command spawns a cascade of conflicting actions, so cap the length.
    if has_number(text) && text.split_whitespace().count() <= 5 {
        return true;
    }
    text.split(|c: char| !c.is_ascii_alphabetic())
        .filter(|w| !w.is_empty())
        .any(|w| FOLLOWUP_WORDS.contains(&w.to_ascii_lowercase().as_str()))
}

/// Spoken or written numbers, for spotting a bare-value follow-up.
/// True if a token is a value word for the bare-value gates: any spoken number
/// (shared with the STT normalizer and grammar via [`phrase::is_number_word`])
/// plus the unit "percent".
pub(crate) fn is_value_word(w: &str) -> bool {
    phrase::is_number_word(w) || w == "percent"
}

/// True if the utterance carries a digit or a spoken number.
pub(crate) fn has_number(text: &str) -> bool {
    text.chars().any(|c| c.is_ascii_digit())
        || text
            .split(|c: char| !c.is_ascii_alphabetic())
            .any(|w| is_value_word(&w.to_ascii_lowercase()))
}

/// True if the utterance is *only* a number/percent value (digits, number words,
/// and light filler like "to"/"at"/"point") — a genuine bare follow-up safe to
/// re-anchor onto the last command. "fraudal at eighty percent" is NOT bare, so
/// it won't be force-attached to the wrong tool.
pub(crate) fn is_bare_value(text: &str) -> bool {
    let mut any_token = false;
    let mut any_number = false;
    for w in text.split(|c: char| !c.is_ascii_alphanumeric()) {
        if w.is_empty() {
            continue;
        }
        any_token = true;
        let lw = w.to_ascii_lowercase();
        let is_digits = lw.chars().all(|c| c.is_ascii_digit());
        if is_digits || is_value_word(&lw) {
            any_number = true;
        } else if !matches!(lw.as_str(), "to" | "at" | "point" | "decimal" | "a") {
            return false; // a real content word — not bare
        }
    }
    any_token && any_number
}

/// "I said eighty percent" / "no, two five zero" / "make it forty" — the pilot
/// correcting the VALUE of the last command (whisper heard "8" for "80" and the
/// wrong value already executed; the fix must not dead-end). Returns the bare
/// value remainder when the utterance is a correction prefix plus nothing but a
/// value — the caller re-anchors it on the last command's subject, exactly like
/// a bare-value follow-up. "No, flaps up" returns None (a full command, not a
/// bare value), and so does a lone "no".
pub(crate) fn correction_value(text: &str) -> Option<String> {
    const PREFIXES: &[&[&str]] = &[
        &["no", "i", "said"],
        &["no", "i", "meant"],
        &["i", "said"],
        &["i", "meant"],
        &["make", "it"],
        &["make", "that"],
        // "How about eighty percent?" — the pilot renegotiating the value
        // (heard live 2026-07-09 after "Power 8 percent" executed a mishear).
        &["how", "about"],
        // ...and whisper dropping the "how": "About 40 percent?" (same flight,
        // same renegotiation, dead-ended). Safe alone: the remainder must
        // still be a bare value, inside the follow-up window.
        &["about"],
        &["correction"],
        &["no"],
    ];
    let lower = text.to_ascii_lowercase();
    let toks: Vec<&str> = lower
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    let rest = PREFIXES
        .iter()
        .find_map(|p| toks.as_slice().strip_prefix(*p))?
        .join(" ");
    (!rest.is_empty() && is_bare_value(&rest)).then_some(rest)
}

/// The leading non-numeric words of a command — its "subject" — used to
/// re-anchor a bare-value follow-up onto the same system. "throttle eighty
/// percent" → "throttle"; "set heading 270" → "set heading". Empty if the
/// command starts with a number.
pub(crate) fn command_subject(cmd: &str) -> String {
    let mut out: Vec<&str> = Vec::new();
    for w in cmd.split_whitespace() {
        let bare = w.trim_matches(|c: char| !c.is_ascii_alphanumeric());
        if bare.chars().any(|c| c.is_ascii_digit()) || is_value_word(&bare.to_ascii_lowercase()) {
            break;
        }
        out.push(w);
    }
    out.join(" ")
}

/// True if a single lowercased token is, or is a one-letter mishearing of, a
/// command keyword. Whisper routinely mangles a word by a letter ("thrust" →
/// "trust", "descend" → "descent"); the near-miss is restricted to 5+ char
/// tokens/keywords so short common words don't match by luck.
pub(crate) fn matches_keyword(w: &str) -> bool {
    if COMMAND_KEYWORDS.contains(&w) {
        return true;
    }
    w.len() >= 5
        && COMMAND_KEYWORDS
            .iter()
            .filter(|k| k.len() >= 5)
            .any(|k| levenshtein(w, k) <= 1)
}

#[cfg(test)]
mod unregistered_tool_tests {
    use super::*;

    /// The hover's words clear the domain gate: the S-76's first "lift to a
    /// hover" never reached the grammar (2026-09-22).
    #[test]
    fn recovery_words_pick_their_line() {
        assert_eq!(recall_ask("say again"), Some(RecallAsk::Newest));
        assert_eq!(recall_ask("didnt catch that"), Some(RecallAsk::Newest));
        assert_eq!(
            recall_ask("what were you saying"),
            Some(RecallAsk::Interrupted)
        );
        assert_eq!(
            recall_ask("sorry you were saying"),
            Some(RecallAsk::Interrupted)
        );
        assert_eq!(recall_ask("flaps ten"), None);
        assert_eq!(recall_ask("what's our altitude"), None);
    }

    #[test]
    fn say_again_is_the_newest_line_and_what_were_you_saying_the_cut_one() {
        let t0 = Instant::now();
        let at = |s: u64| t0 + Duration::from_secs(s);
        // The copilot was saying the caution when the pilot keyed up (t=10)
        // to ask for flaps; "Flaps ten" was read back at t=12.
        let cut = vec![("Gear's still up.".to_string(), at(10))];
        let readback = ("Flaps ten.".to_string(), at(12));
        // "Say again": the readback, the newest thing said. The cut line stays owed.
        assert_eq!(
            recall_line(RecallAsk::Newest, &cut, Some(&readback), None),
            Some(("Flaps ten.".to_string(), false))
        );
        // "What were you saying?": the cut line, which it then spends.
        assert_eq!(
            recall_line(RecallAsk::Interrupted, &cut, Some(&readback), None),
            Some(("Gear's still up.".to_string(), true))
        );
        // Keyed over with nothing said since: "say again" is the cut line too.
        let old_readback = ("Flaps ten.".to_string(), at(5));
        assert_eq!(
            recall_line(RecallAsk::Newest, &cut, Some(&old_readback), None),
            Some(("Gear's still up.".to_string(), true))
        );
        // A volunteered line newer than the readback outranks it: before, "say
        // again" after an unasked caution repeated an older command readback.
        let volunteered = ("Top of descent in two minutes.".to_string(), at(20));
        assert_eq!(
            recall_line(RecallAsk::Newest, &[], Some(&readback), Some(&volunteered)),
            Some(("Top of descent in two minutes.".to_string(), false))
        );
        // Nothing cut: "what were you saying" is the newest line.
        assert_eq!(
            recall_line(RecallAsk::Interrupted, &[], Some(&readback), None),
            Some(("Flaps ten.".to_string(), false))
        );
        // Nothing at all: no invented line.
        assert_eq!(recall_line(RecallAsk::Newest, &[], None, None), None);
    }

    #[test]
    fn what_did_the_tower_say_is_grounded_in_what_went_out() {
        use crate::voice::{ControllerLine, StationSpeaker};
        assert!(atc_recall_request("what did the tower say"));
        assert!(atc_recall_request("what did approach say"));
        assert!(atc_recall_request("what was that from atc"));
        assert!(!atc_recall_request("what did you say"));
        assert!(!atc_recall_request("what's the tower frequency"));
        let t0 = Instant::now();
        let line = ControllerLine {
            text: "November one seven two Sierra Papa, descend three thousand feet.".into(),
            speaker: StationSpeaker::Approach,
            khz: Some(120_650),
            at: t0,
        };
        let later = t0 + Duration::from_secs(40);
        assert_eq!(
            atc_recall_line(true, Some(&line), later),
            "Approach said: November one seven two Sierra Papa, descend three thousand feet."
        );
        // External ATC (copilot mode): the crew cannot hear it, and says so.
        let ext = atc_recall_line(false, Some(&line), later);
        assert!(ext.contains("can't hear"), "{ext}");
        assert!(!ext.contains("descend"), "{ext}");
        // Nothing on file, or too old: no invented clearance.
        assert!(atc_recall_line(true, None, later).starts_with("Nothing from ATC"));
        assert!(
            atc_recall_line(true, Some(&line), t0 + ATC_RECALL_TTL).starts_with("Nothing from ATC")
        );
    }

    #[test]
    fn landing_questions_get_the_destination_not_the_next_fix() {
        for q in [
            "how far before we land?",
            "how much to land?",
            "how much before we land?",
            "how long left before we land?",
            "when do we land?",
            "time to destination.",
            "how far to the destination",
        ] {
            assert!(eta_question(&q.to_lowercase()), "{q}");
        }
        for q in [
            "how much fuel will we land with?",
            "how much runway do we need to land?",
            "landing lights on",
            "how far to the next waypoint?",
            "what's our landing speed?",
        ] {
            assert!(!eta_question(q), "{q}");
        }
    }

    #[test]
    fn owed_lines_are_said_again_newest_three_in_order() {
        let t0 = Instant::now();
        let cut: Vec<(String, Instant)> = ["One.", "Two.", "Three.", "Four."]
            .iter()
            .enumerate()
            .map(|(i, t)| (t.to_string(), t0 + Duration::from_secs(i as u64)))
            .collect();
        assert_eq!(
            recall_line(RecallAsk::Interrupted, &cut, None, None),
            Some(("Two. Three. Four.".to_string(), true))
        );
    }

    #[test]
    fn a_helicopters_words_are_commands() {
        for w in [
            "Lift to a hover.",
            "hover at fifteen feet",
            "Set it down.",
            "set us down",
            "lift off",
        ] {
            assert!(looks_like_command(w), "{w}");
        }
    }

    /// The S-76C: a withheld fixed-wing law is refused with its reason; an
    /// unrelated missing tool, and any tool on a fixed-wing profile, keep
    /// the generic line.
    #[test]
    fn a_withheld_law_on_a_rotorcraft_names_its_reason() {
        let heli = aircraft::load("sikorsky_s76c").expect("S-76C profile loads");
        assert!(heli.rotorcraft);
        assert!(unregistered_tool_line(&heli, "circuit").starts_with("Not on a helicopter"));
        assert_eq!(
            unregistered_tool_line(&heli, "no_such_tool"),
            "That command isn't wired up yet."
        );
        let mut fixed = heli.clone();
        fixed.rotorcraft = false;
        fixed.autopilot = true;
        assert_eq!(
            unregistered_tool_line(&fixed, "circuit"),
            "That command isn't wired up yet."
        );
        // An airframe whose autopilot we cannot drive says that instead.
        fixed.autopilot = false;
        assert!(unregistered_tool_line(&fixed, "full_auto").starts_with("Not on this airframe"));
        assert_eq!(
            unregistered_tool_line(&fixed, "no_such_tool"),
            "That command isn't wired up yet."
        );
    }
}
