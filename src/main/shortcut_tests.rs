    //! The command loop's pre-grammar shortcuts are one function's decisions
    //! (`question_shortcut`), asked by the loop and by `xvl sortie --check`.
    use super::*;

    fn b738() -> (aircraft::AircraftProfile, Registry) {
        let p = aircraft::load("boeing_737_800").unwrap();
        let r = Registry::for_aircraft(&p);
        (p, r)
    }

    /// The loop asks every shortcut, each once, in the function's order: a
    /// branch that decides on its own words again is a branch the check
    /// cannot see.
    #[test]
    fn the_loop_asks_every_shortcut_in_order() {
        let src = crate::test_support::module_source("main");
        let start = src.find("async fn run_session(").expect("the command loop");
        let end = start + src[start..].find("\n}\n").expect("its end");
        let body = &src[start..end];
        let asked: Vec<String> = body
            .match_indices("Some(Shortcut::")
            .map(|(i, _)| {
                let rest = &body[i + "Some(Shortcut::".len()..];
                rest[..rest.find(')').unwrap()].to_string()
            })
            .collect();
        let want: Vec<String> = Shortcut::ALL.iter().map(|s| format!("{s:?}")).collect();
        assert_eq!(asked, want);
        let mut sorted = Shortcut::ALL.to_vec();
        sorted.sort();
        assert_eq!(sorted, Shortcut::ALL.to_vec(), "ALL is the enum's order");
        // The loop's own copies of the conditions are gone.
        let gone = ["let is_reminder", "let wants_load", "let is_plan", "is_fuel_query(&command)"];
        for gone in gone {
            assert!(!body.contains(gone), "{gone}");
        }
    }

    /// LEMG (2026-09-28): the enroute setup's words were answered as a top
    /// of descent question and the setup never ran. No setup's words are a
    /// shortcut's; the questions still are.
    #[test]
    fn a_setup_is_never_a_question() {
        use tools::setup_enroute::{enroute_line, EnroutePoint};
        let (p, r) = b738();
        for point in [
            EnroutePoint::BeforeTod(30.0),
            EnroutePoint::BeforeTod(0.0),
            EnroutePoint::Fix("WELIN".into()),
            EnroutePoint::BeforeDestination(40.0),
        ] {
            let line = enroute_line(&point);
            assert_eq!(question_shortcut(&line, &r, &p, None), None, "{line}");
            assert_eq!(pre_grammar_capture(&line, &r, &p), None, "{line}");
        }
        let line = "put us on a 10 mile final to LEMG runway 13";
        assert_eq!(pre_grammar_capture(line, &r, &p), None);
        for (line, want) in [
            ("how long to top of descent", Some(Shortcut::TopOfDescent)),
            ("wake me at top of descent", None),
            ("what's the weather at the destination", Some(Shortcut::RemoteWeather)),
            ("how far to the next waypoint", Some(Shortcut::NavProgress)),
            ("cancel the reminders", Some(Shortcut::ReminderCancel)),
            ("do we have a reminder", Some(Shortcut::ReminderList)),
            ("could you give me a status update", Some(Shortcut::SystemsCheck)),
            ("plan a flight to LEZL", Some(Shortcut::PlanFlight)),
            ("load the flight plan", Some(Shortcut::LoadRoute)),
            ("load the aircraft", None),
            ("fly the flight plan", None),
            ("what's our flight plan", Some(Shortcut::PlanBrief)),
        ] {
            assert_eq!(question_shortcut(line, &r, &p, None), want, "{line}");
        }
        // Past the shortcuts: a question never reaches a tool.
        assert_eq!(
            pre_grammar_capture("is the gear down?", &r, &p),
            Some("state question")
        );
        for line in ["fly a circuit", "reject the takeoff", "resume flight plan", "flaps five"] {
            assert_eq!(pre_grammar_capture(line, &r, &p), None, "{line}");
        }
        assert_eq!(
            state_capture_note("my controls"),
            Some("the release guard takes it while a loop holds a control")
        );
        assert_eq!(state_capture_note("fly a circuit"), None);
    }
