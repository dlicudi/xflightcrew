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
    assert_eq!(one("Land aircraft.").unwrap(), ("autoland".into(), json!({})));
    // "Right now" is the nearest runway, not the plan's destination.
    assert_eq!(
        one("Let's land right now.").unwrap(),
        ("autoland".into(), json!({ "nearest": true }))
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
        assert_eq!(h[0].arguments, json!({ "action": "route", "resume": false }), "{words}");
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
    let c = first("resume flight plan").expect("helicopter resume");
    assert_eq!(c.arguments, json!({ "action": "route", "resume": true }));
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

/// The pilot's go-around reaches its tool on every fixed-wing airframe;
/// the tower's order read back, a negation or a helicopter never does
/// (#77, #102).
#[test]
fn the_pilots_go_around_is_its_tool_and_a_readback_is_not() {
    for p in [
        "go around",
        "Go around!",
        "going around",
        "Go-around.",
        "take it around",
        "missed approach",
        "go missed",
        "let's go around",
        "we're going around",
        "do a go around",
        "go around, go around",
    ] {
        assert_eq!(one(p).map(|c| c.0).as_deref(), Some("go_around"), "{p:?}");
        assert!(crate::router::looks_like_command(p), "{p:?} passes the gate");
    }
    for id in ["cirrus_vision_sf50", "boeing_737_800", "cessna_172sp"] {
        assert_eq!(one_on(id, "go around").map(|c| c.0).as_deref(), Some("go_around"));
    }
    for r in [
        "go around, november one two three",
        "roger, going around",
        "wilco, going around",
        "going around, speedbird three zero six",
        "don't go around",
        "we're not going around",
        "why did we go around",
    ] {
        assert!(one(r).is_none_or(|(tool, _)| tool != "go_around"), "{r:?}");
    }
    assert!(one_on("sikorsky_s76c", "go around").is_none_or(|(t, _)| t != "go_around"));
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

/// #100: the ice words reach `set_anti_ice` with the system named, and
/// pitot and carb heat keep their own tools.
#[test]
fn anti_ice_words_resolve() {
    let ai = |s: &str, on: bool| ("set_anti_ice".to_string(), json!({"system": s, "on": on}));
    assert_eq!(one("anti-ice on").unwrap(), ai("all", true));
    assert_eq!(one("Anti ice off.").unwrap(), ai("all", false));
    assert_eq!(one("turn on the ice protection").unwrap(), ai("all", true));
    assert_eq!(one("de-ice on").unwrap(), ai("all", true));
    assert_eq!(one("engine anti-ice on").unwrap(), ai("engine", true));
    assert_eq!(one("engine anti ice off").unwrap(), ai("engine", false));
    assert_eq!(one("wing anti-ice on").unwrap(), ai("wing", true));
    assert_eq!(one("boots on").unwrap(), ai("wing", true));
    assert_eq!(one("window heat on").unwrap(), ai("window", true));
    assert_eq!(one("prop heat on").unwrap(), ai("prop", true));
    assert_eq!(one("pitot heat on").unwrap().0, "set_pitot_heat");
    assert!(one("carb heat on").is_none_or(|c| c.0 != "set_anti_ice"));
    assert!(one("anti-ice").is_none(), "no state, no switch");
}

/// #92: the autopilot's own modes on an airliner; "autothrottle on" on
/// an airframe without the MD-82's tool is the stock autothrottle.
#[test]
fn autopilot_mode_words_resolve() {
    let m = |mode: &str, on: bool| ("ap_mode".to_string(), json!({"mode": mode, "on": on}));
    let b = "boeing_737_800";
    assert_eq!(one_on(b, "level change").unwrap(), m("level_change", true));
    assert_eq!(one_on(b, "FLCH").unwrap(), m("level_change", true));
    assert_eq!(one_on(b, "level change off").unwrap(), m("level_change", false));
    assert_eq!(one_on(b, "GPS steering on").unwrap(), m("lnav", true));
    assert_eq!(one_on(b, "engage LNAV").unwrap(), m("lnav", true));
    assert_eq!(one_on(b, "autothrottle on").unwrap(), m("autothrust", true));
    assert_eq!(one_on(b, "autothrottle off").unwrap(), m("autothrust", false));
    // The MD-82 keeps its own switch.
    assert_eq!(one_on("md82", "autothrottle on").unwrap().0, "autothrottle");
    // A flight level is still a flight level.
    assert_ne!(one_on(b, "climb flight level one two zero").unwrap().0, "ap_mode");
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
fn a_flight_level_is_an_altitude_target() {
    assert_eq!(
        one("climb flight level one two zero").unwrap(),
        ("set_altitude".into(), json!({"altitude":12000}))
    );
    assert_eq!(
        one("descend to flight level 80").unwrap(),
        ("set_altitude".into(), json!({"altitude":8000}))
    );
    assert_eq!(
        one("climb and maintain fl350").unwrap(),
        ("set_altitude".into(), json!({"altitude":35000}))
    );
    assert_eq!(
        one("climb to flight level one five zero at 1500").unwrap(),
        ("set_climb".into(), json!({"altitude":15000,"fpm":1500}))
    );
    assert_eq!(flight_level_as_feet("set flaps 20"), None);
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
        ("autoland".into(), json!({ "nearest": true }))
    );
    // The nearest field by name; a plain landing is the destination's.
    for nearest in [
        "land at the nearest airport",
        "land at the closest airport",
        "divert to the nearest field",
        "let's land at the nearest",
    ] {
        assert_eq!(
            one(nearest).unwrap(),
            ("autoland".into(), json!({ "nearest": true })),
            "{nearest}"
        );
    }
    assert!(one("taxi to the nearest stand").is_none_or(|(tool, _)| tool != "autoland"));
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

/// The live chain on the SR22: whisper's text through the STT repairs
/// (`stt.rs`), the domain gate, then the grammar.
fn sr22_live(heard: &str) -> Option<(String, Value)> {
    let t = crate::phrase::collapse_hyphens(heard);
    let t = crate::phrase::fix_mishears(&t);
    let t = crate::phrase::normalize_numbers(&t);
    assert!(
        crate::router::looks_like_command(&t),
        "{heard:?} ({t:?}) must pass the domain gate"
    );
    one_on("cirrus_sr22", &t)
}

/// Issue #69, the SR22's dead ends from LSZS-LILO (2026-09-24), each
/// transcript verbatim from the blackbox `heard` records.
#[test]
fn sr22_lszs_lilo_misses_are_rules() {
    // t 1790265258.8, climbing out: the box stayed in standby.
    assert_eq!(
        sr22_live("Set transponder."),
        Some(("set_transponder_mode".into(), json!({"mode": "altitude"})))
    );
    // t 1790266795.0: dropped silently at the domain gate ("SP" is
    // no keyword); the pilot re-said it and got "885 knots".
    assert_eq!(
        sr22_live("Set SP to 85 knots."),
        Some(("hold_speed".into(), json!({"knots": 85})))
    );
    // The same clip from 2026-09-20.
    assert_eq!(
        sr22_live("Set SP to 160 knots."),
        Some(("hold_speed".into(), json!({"knots": 160})))
    );
    // t 1790266907.4, on the final: a deterministic miss.
    assert_eq!(
        sr22_live("Can you manage throttle for landing?"),
        Some(("hold_speed".into(), json!({"preset": "approach"})))
    );
}

/// The new shapes do not over-match: "SP" is repaired only after "set",
/// a transponder with "code" and no number still asks, and the lever
/// words take the governor only with an asking verb.
#[test]
fn sr22_rules_stay_narrow() {
    assert_eq!(crate::phrase::fix_mishears("SP 85"), "SP 85");
    assert_eq!(crate::phrase::fix_mishears("set spoilers"), "set spoilers");
    assert_eq!(
        crate::phrase::fix_mishears("set sp to 85"),
        "set speed to 85"
    );
    assert!(one_on("cirrus_sr22", "set transponder code").is_none());
    assert_eq!(
        one_on("cirrus_sr22", "set transponder 7000"),
        Some(("set_transponder".into(), json!({"code": 7000})))
    );
    assert_eq!(
        one_on("cirrus_sr22", "set transponder standby"),
        Some(("set_transponder_mode".into(), json!({"mode": "standby"})))
    );
    assert!(one_on("cirrus_sr22", "keep the power up").is_none_or(|(n, _)| n != "hold_speed"));
    assert_eq!(
        one_on("cirrus_sr22", "my throttle"),
        Some(("hold_speed".into(), json!({"off": true}))),
        "the release still wins"
    );
    assert!(one_on("cirrus_sr22", "throttle 50 percent").is_none_or(|(n, _)| n != "hold_speed"));
}

/// Live misses since 2026-10-04, each the pilot's own words.
#[test]
fn thrust_is_the_throttle_and_landing_is_asked_plainly() {
    // "We reduce thrust to 20 percent." missed, the rephrase "Reduced power
    // 20 percent" took the lever to 80 (LEMG, 2026-10-05).
    for (w, want) in [
        ("We reduce thrust to 20 percent.", json!({ "percent": 20 })),
        ("set thrust to 20 percent", json!({ "percent": 20 })),
        ("reduce thrust by 10 percent", json!({ "delta": -10 })),
        ("thrust idle", json!({ "percent": 0 })),
    ] {
        assert_eq!(one(w), Some(("set_throttle".into(), want)), "{w}");
    }
    for w in ["reverse thrust", "auto thrust off", "thrust reversers"] {
        assert!(one(w).is_none_or(|(n, _)| n != "set_throttle"), "{w}");
    }
    // "Now you can land." twice, then "Please land the aircraft." (LEGR,
    // 2026-10-06); "Bring the aircraft down." (2026-10-04).
    for w in ["Now you can land.", "you can land", "Can you land?", "Bring the aircraft down."] {
        assert_eq!(one(w).map(|c| c.0), Some("autoland".into()), "{w}");
    }
    assert!(one("bring us down to 3000 feet").is_none_or(|c| c.0 != "autoland"));
    // "Resume flight then." (LXGB, 2026-10-06); "Resume slides.",
    // "Resume flying." (2026-10-01/02).
    for w in ["Resume flight then.", "Resume slides.", "Resume flying."] {
        assert!(resume_plan_request(w), "{w}");
    }
    assert_eq!(one("Fold flaps.").map(|c| c.0), Some("set_flaps".into()));
    assert_eq!(one("Can you set up ALS?").map(|c| c.0), Some("prepare_ils".into()));
}
