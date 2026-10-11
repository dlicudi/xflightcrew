    use super::*;

    const LEMG: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/sorties/lemg-737.yaml"));

    /// Every shipped spec, parsed against the install when there is one
    /// (a machine without X-Plane checks the shapes, not the paths).
    fn bench_specs() -> Vec<(std::path::PathBuf, Spec)> {
        let installed = std::path::Path::new(&crate::config::xplane_root()).is_dir();
        let mut out = Vec::new();
        for f in std::fs::read_dir("bench/sorties").unwrap().flatten() {
            let p = f.path();
            if p.extension().is_some_and(|e| e == "yaml") {
                let text = std::fs::read_to_string(&p).unwrap();
                let spec = if installed {
                    parse_spec(&text)
                } else {
                    parse_spec_with(&text, &|_| true)
                };
                out.push((
                    p.clone(),
                    spec.unwrap_or_else(|e| panic!("{}: {e}", p.display())),
                ));
            }
        }
        out.sort_by(|a, b| a.0.cmp(&b.0));
        out
    }

    /// The profile a bench aircraft flies with: the detector's own answer
    /// when the `.acf` is installed, pinned to this table either way.
    fn bench_profile(acf: Option<&str>) -> crate::aircraft::AircraftProfile {
        let Some(acf) = acf else {
            return crate::aircraft::load("boeing_737_800").unwrap();
        };
        let file = acf.rsplit('/').next().unwrap();
        let id = match file {
            "b738.acf" => "boeing_737_800",
            "b738_4k.acf" => "zibo_738",
            "Cirrus SR22.acf" => "cirrus_sr22",
            "CirrusSF50.acf" => "cirrus_vision_sf50",
            "A330.acf" => "airbus_a330_300",
            "Cessna_CitationX.acf" => "citation_x",
            "S-76C.acf" => "sikorsky_s76c",
            "Cessna_172SP.acf" | "Cessna_172SP_G1000.acf" => "cessna_172sp",
            "N844X.acf" => "lancair_evolution",
            "MD80.acf" => "md82",
            "Baron_58.acf" => "beechcraft_baron_58",
            "C90B.acf" => "king_air_c90b",
            "RV-10.acf" => "vans_rv10",
            other => panic!("{other}: add the bench aircraft to this table"),
        };
        if let Some(icao) = std::fs::read_to_string(crate::config::xplane_path(acf))
            .ok()
            .and_then(|t| crate::tools::setup::acf_icao(&t))
        {
            assert_eq!(
                crate::detect::resolved_id(&icao, acf).as_deref(),
                Some(id),
                "{acf} ({icao}) is detected as another profile"
            );
        }
        crate::aircraft::load(id).unwrap_or_else(|| panic!("profile {id}"))
    }

    /// The tool a typed line reaches, as the command loop routes it: the
    /// local planner's branch first (`flightplan::parse_request`), then the
    /// grammar.
    fn routed(line: &str, prof: &crate::aircraft::AircraftProfile) -> Option<String> {
        route_of(line, &crate::tools::Registry::for_aircraft(prof), prof)
    }

    /// Every line a shipped spec types is the app's real phrasing: each
    /// step reaches the tool it names, each command reaches a tool, on the
    /// case's own airframe.
    #[test]
    fn every_scripted_line_routes_to_its_tool() {
        let mut checked = 0;
        for (path, spec) in bench_specs() {
            for c in &spec.cases {
                let prof = bench_profile(c.aircraft.as_deref());
                let at = |what: &str| format!("{} {:?} {what}", path.display(), c.name);
                if let Some(cmd) = &c.command {
                    assert!(routed(cmd, &prof).is_some(), "{}", at(cmd));
                    checked += 1;
                }
                for s in c.steps.iter().filter(|s| s.is_say()) {
                    let want = s
                        .tool
                        .as_deref()
                        .unwrap_or_else(|| panic!("{}: name the tool", at(&s.say)));
                    assert_eq!(
                        routed(&s.say, &prof).as_deref(),
                        Some(want),
                        "{}",
                        at(&s.say)
                    );
                    checked += 1;
                }
            }
        }
        assert!(checked >= 15, "{checked}");
        // The reject is the pilot's command, auto_takeoff's off switch; the
        // crew's radio call is autotakeoff::REJECT_CALL, a different line.
        let b738 = bench_profile(None);
        let reg = crate::tools::Registry::for_aircraft(&b738);
        let calls = crate::grammar::try_parse("reject the takeoff", &reg, &b738).unwrap();
        assert_eq!(calls[0].arguments, json!({"off": true}));
        let calls = crate::grammar::try_parse("resume flight plan", &reg, &b738).unwrap();
        assert_eq!(calls[0].arguments["resume"], true, "{calls:?}");
    }

    /// `xvl sortie --check` over every shipped spec: every line routed as
    /// the command loop routes it (the pre-grammar shortcuts first), on the
    /// case's own airframe, and no problem anywhere.
    #[test]
    fn the_check_passes_every_shipped_spec() {
        let mut lines = 0;
        for (path, spec) in bench_specs() {
            for c in &spec.cases {
                let prof = bench_profile(c.aircraft.as_deref());
                for r in check_case(c, &prof, "") {
                    assert!(r.problem.is_none(), "{} {r:?}", path.display());
                    lines += 1;
                }
            }
        }
        assert!(lines >= 20, "{lines}");
    }

    fn check_spec_of(text: &str) -> Vec<LineCheck> {
        let spec = parse_spec_with(text, &|_| true).unwrap();
        let prof = bench_profile(None);
        spec.cases
            .iter()
            .flat_map(|c| check_case(c, &prof, ""))
            .collect()
    }

    /// The check fails a line a pre-grammar shortcut takes (the enroute
    /// setup answered as a top of descent question, LEMG 2026-09-28), a
    /// step that reaches another tool, and a command that reaches none;
    /// the setup's own words are checked, an unknown runway is skipped.
    #[test]
    fn the_check_fails_a_line_the_loop_would_not_route_to_its_tool() {
        let rows = check_spec_of(
            r#"
cases:
  - name: a
    setup: { enroute: { at: before_tod_nm, value: 30 } }
    command: fly the flight plan
  - name: a2
    setup: { enroute: { at: before_destination_nm, value: 40 } }
    command: fly the flight plan
  - name: a3
    setup: { enroute: { at: fix, value: WELIN } }
    command: fly the flight plan
  - name: b
    setup: { ramp: { field: LEMG } }
    steps:
      - { say: fly a circuit, when: start, tool: full_auto }
      - { say: how long to top of descent, when: { after_s: 5 }, tool: full_auto }
      - { say: flaps five, when: { after_s: 5 }, tool: full_auto }
      - { say: what were you saying, when: { after_s: 5 }, tool: recall }
  - name: c
    setup: { runway_start: { field: LEMG, runway: "13" } }
    command: banana split
  - name: d
    setup: { approach: { field: LEMG, runway: wind } }
  - name: e
    setup: { approach: { field: LEMG, runway: "13", final_nm: 8 } }
"#,
        );
        let row = |line: &str| rows.iter().find(|r| r.line == line).unwrap();
        // "Before the destination" carries "route" and "the", the flight
        // plan brief's words: this check found it (2026-09-29).
        for line in [
            "put us enroute 30 miles before top of descent",
            "put us enroute 40 miles before the destination",
            "put us enroute at WELIN",
        ] {
            let r = row(line);
            assert_eq!((r.routes_to.as_str(), &r.problem), ("setup_enroute", &None), "{line}");
        }
        assert!(row("fly the flight plan").problem.is_none());
        let tod = row("how long to top of descent");
        assert_eq!(tod.routes_to, "top of descent question");
        assert_eq!(
            tod.problem.as_deref(),
            Some("taken before the grammar (top of descent question)")
        );
        assert_eq!(row("flaps five").problem.as_deref(), Some("the step names full_auto"));
        // A step that names the capture wants it.
        let recall = row("what were you saying");
        assert_eq!((recall.routes_to.as_str(), &recall.problem), ("recall", &None));
        assert_eq!(row("banana split").problem.as_deref(), Some("reaches no tool"));
        let wind = row("put us on a 10 mile final to LEMG runway WIND");
        assert_eq!((wind.routes_to.as_str(), &wind.problem), ("-", &None));
        let e = row("put us on a 8 mile final to LEMG runway 13");
        assert_eq!((e.routes_to.as_str(), &e.problem), ("setup_approach", &None));
        let table = check_table("x.yaml", &rows, &["a note".into()]);
        assert!(table.contains("| PROBLEM: the step names full_auto |"), "{table}");
        assert!(table.contains("- a note\n"), "{table}");
        assert!(table.ends_with("3 problems in 13 lines.\n"), "{table}");
    }

    /// A producer's text as `--check` reads it (the module, a directory
    /// included), the checkout walked once.
    fn source(path: &str) -> Option<String> {
        static SOURCES: std::sync::OnceLock<Vec<(String, String)>> = std::sync::OnceLock::new();
        module_text(path, SOURCES.get_or_init(|| checkout_sources().unwrap()))
    }

    /// Every record shape the runner matches on is one the app writes, at
    /// the snippet named and with the fields read; a producer that changes
    /// shape is caught. d270a29: full auto's give-up was read as `event`
    /// `give_up` with a top-level `reason`, a shape it never wrote.
    #[test]
    fn every_record_the_runner_reads_is_one_the_app_writes() {
        assert_eq!(shape_problems(&source), Vec::<String>::new());
        // Full auto writing its give-up the way the runner once read it.
        let moved = |p: &str| {
            let t = source(p)?;
            Some(match p {
                "src/full_auto.rs" => t.replace(
                    r#"json!({ "stage": stage, "detail": detail })"#,
                    r#"json!({ "event": stage, "reason": detail })"#,
                ),
                "src/scorecard.rs" => t.replace(r#""ended_airborne""#, r#""airborne""#),
                _ => t,
            })
        };
        let problems = shape_problems(&moved);
        assert!(
            problems.iter().any(|p| p.starts_with("full_auto stage=give_up: src/full_auto.rs")),
            "{problems:?}"
        );
        assert!(
            problems
                .iter()
                .any(|p| p == "sortie_result: no producer writes the field ended_airborne"),
            "{problems:?}"
        );
    }

    /// Every field the runner's code reads from a record is in
    /// [`READ_SHAPES`] (or the writer's own), so a new read is pinned to a
    /// producer the day it is written.
    #[test]
    fn every_field_the_runner_reads_is_in_the_shape_table() {
        let text = source("src/sortie_runner.rs").unwrap();
        let code = &text[..text.find("#[cfg(test)]\nmod tests").unwrap()];
        let mut known: Vec<&str> = WRITER_FIELDS.to_vec();
        for s in READ_SHAPES {
            known.push(s.key);
            known.extend(s.fields);
        }
        known.push("event");
        let mut read = Vec::new();
        for prefix in ["st(r, \"", "st(s, \"", ".get(\"", "n(\"", "f(\"", "pointer(\""] {
            for (i, _) in code.match_indices(prefix) {
                let before = code[..i].chars().next_back().unwrap_or(' ');
                if prefix.len() <= 3 && (before.is_alphanumeric() || before == '_') {
                    continue;
                }
                let rest = &code[i + prefix.len()..];
                let lit = &rest[..rest.find('"').unwrap()];
                read.extend(lit.split('/').filter(|w| !w.is_empty()).map(str::to_string));
            }
        }
        assert!(read.iter().any(|f| f == "reason"), "{read:?}");
        assert!(read.iter().any(|f| f == "relative_path"), "{read:?}");
        let unknown: Vec<&String> = read.iter().filter(|f| !known.contains(&f.as_str())).collect();
        assert!(unknown.is_empty(), "read but not in READ_SHAPES: {unknown:?}");
    }

    /// A spec's event triggers and expected give-ups name words the app
    /// writes; a made-up one is caught.
    #[test]
    fn a_specs_event_and_give_up_are_words_the_app_writes() {
        let sources = checkout_sources().unwrap();
        for (path, spec) in bench_specs() {
            assert_eq!(
                spec_word_problems(&spec, &sources),
                Vec::<String>::new(),
                "{}",
                path.display()
            );
        }
        let spec = parse_spec_with(
            r#"
cases:
  - name: x
    setup: { ramp: { field: LEMG } }
    command: fly a circuit
    expect_give_up: took_a_nap
    steps:
      - { say: flaps five, when: { event: { kind: loading, event: nap_time } }, tool: flaps }
"#,
            &|_| true,
        )
        .unwrap();
        assert_eq!(
            spec_word_problems(&spec, &sources),
            vec![
                "case \"x\": no record writes \"nap_time\"".to_string(),
                "case \"x\": no give-up is named \"took_a_nap\"".to_string(),
            ]
        );
    }

    /// The aircraft named in a spec is an installed `.acf`, checked when the
    /// spec is read, never found missing at the setup.
    #[test]
    fn a_setup_aircraft_must_be_an_installed_acf() {
        let spec = |a: &str| {
            format!(
                "cases: [{{name: a, setup: {{aircraft: \"{a}\", runway_start: {{field: LEMG, runway: 13}}}}, command: fly a circuit}}]"
            )
        };
        let there = |p: &str| p == "Aircraft/x/y.acf";
        let s = parse_spec_with(&spec("Aircraft/x/y.acf"), &there).unwrap();
        assert_eq!(s.cases[0].aircraft.as_deref(), Some("Aircraft/x/y.acf"));
        let e = parse_spec_with(&spec("Aircraft/x/z.acf"), &there).unwrap_err();
        assert!(e.to_string().contains("is not installed under"), "{e}");
        let e = parse_spec_with(&spec("Aircraft/x/y"), &there).unwrap_err();
        assert!(e.to_string().contains("not an .acf path"), "{e}");
        // "plan" is resolved from the fetched plan, not checked here.
        let s = parse_spec_with(&spec("plan"), &there).unwrap();
        assert!(s.cases[0].uses_plan());
        // Every setup kind takes one.
        for setup in [
            "{aircraft: Aircraft/x/y.acf, approach: {field: LEMG, runway: 13}}",
            "{aircraft: Aircraft/x/y.acf, ramp: {field: LEMG}}",
        ] {
            let text = format!("cases: [{{name: a, setup: {setup}, steps: []}}]").replace(
                "steps: []",
                if setup.contains("ramp") {
                    "command: full auto"
                } else {
                    "deadline_min: 5"
                },
            );
            let s = parse_spec_with(&text, &there).unwrap_or_else(|e| panic!("{setup}: {e}"));
            assert_eq!(s.cases[0].aircraft.as_deref(), Some("Aircraft/x/y.acf"));
        }
    }

    #[test]
    fn steps_parse_every_trigger_and_refuse_a_bad_script() {
        let s = parse_spec_with(
            "cases: [{name: a, setup: {runway_start: {field: LEMG, runway: 13}}, steps: [\n\
               {say: fly a circuit, when: start, tool: full_auto},\n\
               {say: a, when: {after_s: 20}},\n\
               {say: b, when: {ias_above: 60}},\n\
               {say: c, when: {agl_above: 400}},\n\
               {say: d, when: {phase: takeoffroll}},\n\
               {say: e, when: {event: {kind: autotakeoff, event: reject}}},\n\
               {say: f, when: {event: {kind: loading}}}]}]",
            &|_| true,
        )
        .unwrap();
        let c = &s.cases[0];
        assert_eq!(c.first_line(), "fly a circuit");
        let whens: Vec<Trigger> = c.later_steps().into_iter().map(|s| s.when).collect();
        assert_eq!(
            whens,
            vec![
                Trigger::AfterS(20.0),
                Trigger::IasAbove(60.0),
                Trigger::AglAbove(400.0),
                Trigger::Phase("TakeoffRoll".into()),
                Trigger::Event {
                    kind: "autotakeoff".into(),
                    event: Some("reject".into())
                },
                Trigger::Event {
                    kind: "loading".into(),
                    event: None
                },
            ]
        );
        // The command is sugar for the start step; the steps after it.
        let s = parse_spec_with(
            "cases: [{name: a, setup: {ramp: {field: LEMG}}, command: full auto, steps: [{say: x, when: {after_s: 5}}]}]",
            &|_| true,
        )
        .unwrap();
        assert_eq!(s.cases[0].first_line(), "full auto");
        assert_eq!(s.cases[0].later_steps().len(), 1);
        // An approach's steps all come after its setup.
        let s = parse_spec_with(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, steps: [{say: my controls, when: {after_s: 30}}]}]",
            &|_| true,
        )
        .unwrap();
        assert_eq!(s.cases[0].later_steps().len(), 1);
        let bad = [
            ("steps: [{say: x, when: {after_s: 5}}]", "needs a command"),
            ("steps: [{say: x}]", "needs a when"),
            (
                "command: y, steps: [{say: x, when: start}]",
                "the command is the start step",
            ),
            (
                "steps: [{say: x, when: start}, {say: y, when: start}]",
                "only the first step",
            ),
            ("steps: [{say: x, when: soon}]", "when is start"),
            (
                "steps: [{say: x, when: start}, {say: y, when: {phase: Taxiing}}]",
                "phase \"Taxiing\"",
            ),
            (
                "steps: [{say: x, when: start}, {say: y, when: {after_s: -1}}]",
                "zero or more",
            ),
            (
                "steps: [{say: x, when: start}, {say: y, when: {after_s: 1, ias_above: 2}}]",
                "when is start",
            ),
            (
                "steps: [{say: x, when: start}, {say: y, when: {event: {event: z}}}]",
                "needs a kind",
            ),
            ("steps: [{say: x, when: start, tol: z}]", "unknown field"),
            ("command: x, expect_give_up: \" \"", "expect_give_up"),
        ];
        for (tail, want) in bad {
            let text = format!(
                "cases: [{{name: a, setup: {{runway_start: {{field: LEMG, runway: 13}}}}, {tail}}}]"
            );
            let e = parse_spec_with(&text, &|_| true)
                .expect_err(tail)
                .to_string();
            assert!(e.contains(want), "{tail}: {e}");
        }
        let e = parse_spec_with(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, steps: [{say: x, when: start}]}]",
            &|_| true,
        )
        .unwrap_err();
        assert!(e.to_string().contains("no start step"), "{e}");
        let e = parse_spec_with(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: plan}}}]",
            &|_| true,
        )
        .unwrap_err();
        assert!(e.to_string().contains("runway plan needs"), "{e}");
    }

    /// Every phase a spec may name is one the flight model has: this match
    /// stops compiling when a phase is added, and the names are `{:?}`'s.
    #[test]
    fn the_phase_names_are_the_flight_models() {
        use crate::flight::Phase as P;
        let all = [
            P::Parked,
            P::EngineStart,
            P::Ready,
            P::Pushback,
            P::TaxiOut,
            P::Holding,
            P::LinedUp,
            P::TakeoffRoll,
            P::RejectedTakeoff,
            P::InitialClimb,
            P::Climb,
            P::Cruise,
            P::Descent,
            P::Approach,
            P::Landing,
            P::GoAround,
            P::Hover,
            P::Rollout,
            P::TaxiIn,
        ];
        for p in all {
            match p {
                P::Parked
                | P::EngineStart
                | P::Ready
                | P::Pushback
                | P::TaxiOut
                | P::Holding
                | P::LinedUp
                | P::TakeoffRoll
                | P::RejectedTakeoff
                | P::InitialClimb
                | P::Climb
                | P::Cruise
                | P::Descent
                | P::Approach
                | P::Landing
                | P::GoAround
                | P::Hover
                | P::Rollout
                | P::TaxiIn => {}
            }
        }
        let names: Vec<String> = all.iter().map(|p| format!("{p:?}")).collect();
        assert_eq!(names, PHASES);
    }

    fn step(say: &str, when: Trigger) -> ScriptStep {
        ScriptStep {
            say: say.into(),
            when,
            tool: None,
            sim_speed: None,
        }
    }

    #[test]
    fn each_trigger_fires_on_its_own_measure_after_the_previous_line() {
        let mut s = Script::new(vec![
            step("a", Trigger::AfterS(20.0)),
            step("b", Trigger::IasAbove(60.0)),
            step("c", Trigger::AglAbove(400.0)),
            step("d", Trigger::Phase("Climb".into())),
            step(
                "e",
                Trigger::Event {
                    kind: "autotakeoff".into(),
                    event: Some("reject".into()),
                },
            ),
            step(
                "f",
                Trigger::Event {
                    kind: "loading".into(),
                    event: None,
                },
            ),
        ]);
        // Nothing before the opening line is answered.
        assert!(!s.due(1000.0, Some(200.0), Some(5000.0)));
        s.arm(100.0);
        assert!(!s.due(119.0, None, None));
        assert!(s.due(120.0, None, None), "after_s from the arm");
        s.sent(121.0);
        assert_eq!(s.wants_reads(), (true, false));
        assert!(!s.due(200.0, Some(60.0), None), "above, not at");
        assert!(
            !s.due(200.0, None, None),
            "an unreadable airspeed is no measurement"
        );
        assert!(s.due(200.0, Some(61.0), None));
        s.sent(201.0);
        assert_eq!(s.wants_reads(), (false, true));
        assert!(!s.due(210.0, Some(200.0), Some(390.0)));
        assert!(s.due(210.0, None, Some(410.0)));
        s.sent(211.0);
        // The phase from the state records and the flight model's events.
        s.feed(&json!({"kind": "state", "fphase": "InitialClimb"}));
        assert!(!s.due(212.0, None, None));
        s.feed(&json!({"kind": "flight", "ev": "phase", "from": "InitialClimb", "to": "Climb"}));
        assert!(s.due(213.0, None, None));
        s.sent(214.0);
        // Another event, or another kind, is not it.
        s.feed(&json!({"kind": "autotakeoff", "event": "roll"}));
        s.feed(&json!({"kind": "autoland", "event": "reject"}));
        assert!(!s.due(215.0, None, None));
        s.feed(&json!({"kind": "autotakeoff", "event": "reject", "reason": "pilot"}));
        assert!(s.due(215.0, None, None));
        s.sent(216.0);
        // An event seen before the previous line was typed does not count.
        let mut early = Script::new(vec![
            step("x", Trigger::AfterS(0.0)),
            step(
                "y",
                Trigger::Event {
                    kind: "loading".into(),
                    event: None,
                },
            ),
        ]);
        early.arm(0.0);
        early.feed(&json!({"kind": "loading", "event": "load_aircraft"}));
        early.sent(1.0);
        assert!(!early.due(2.0, None, None));
        // Any event of the kind.
        s.feed(&json!({"kind": "loading", "event": "load_aircraft"}));
        assert!(s.due(217.0, None, None));
        s.sent(218.0);
        assert_eq!(s.pending(), 0);
        assert!(!s.due(1e9, Some(1e3), Some(1e5)));
    }

    #[test]
    fn the_plans_words_become_its_fields_runways_and_type() {
        let plan = PlanFacts {
            origin: "EGBB".into(),
            destination: "EGKK".into(),
            origin_rwy: Some("33".into()),
            dest_rwy: Some("26l".into()),
            aircraft: "B738".into(),
            ..PlanFacts::default()
        };
        let find = |icao: &str| {
            (icao == "B738")
                .then(|| "Aircraft/Laminar Research/Boeing 737-800/b738.acf".to_string())
        };
        let spec = parse_spec_with(
            "cases: [\n\
             {name: a, setup: {aircraft: plan, ramp: {field: plan_origin}}, command: fly the flight plan},\n\
             {name: b, setup: {approach: {field: plan_destination, runway: plan, final_nm: 15}}},\n\
             {name: c, setup: {runway_start: {field: plan_origin, runway: 27}}, command: fly a circuit},\n\
             {name: d, setup: {ramp: {field: LEMG}}, command: full auto}]",
            &|_| false,
        )
        .unwrap();
        assert!(spec.cases[..3].iter().all(Case::uses_plan));
        assert!(!spec.cases[3].uses_plan());
        let got: Vec<Case> = spec
            .cases
            .iter()
            .map(|c| resolve_case(c, &plan, &find).unwrap())
            .collect();
        assert_eq!(
            got[0].setup,
            Setup::Ramp {
                field: "EGBB".into(),
                stand: None
            }
        );
        assert_eq!(
            got[0].aircraft.as_deref(),
            Some("Aircraft/Laminar Research/Boeing 737-800/b738.acf")
        );
        assert_eq!(
            got[1].setup,
            Setup::Approach {
                field: "EGKK".into(),
                runway: "26L".into(),
                final_nm: 15
            }
        );
        assert_eq!(
            got[1].first_line(),
            "put us on a 15 mile final to EGKK runway 26L"
        );
        assert_eq!(
            got[2].setup,
            Setup::RunwayStart {
                field: "EGBB".into(),
                runway: "27".into(),
                engines_running: false
            }
        );
        assert_eq!(got[3].setup, spec.cases[3].setup);
        // What the plan lacks is said, naming the case.
        let bare = PlanFacts {
            aircraft: "A20N".into(),
            ..plan.clone()
        };
        let e = resolve_case(&spec.cases[0], &bare, &find).unwrap_err();
        assert!(
            e.to_string()
                .contains("no installed aircraft of the plan's type A20N"),
            "{e}"
        );
        let no_rwy = PlanFacts {
            dest_rwy: None,
            ..plan
        };
        let e = resolve_case(&spec.cases[1], &no_rwy, &find).unwrap_err();
        assert!(e.to_string().contains("names no runway"), "{e}");
    }

    /// The detector's record for the model the case loads, and no other.
    #[test]
    fn the_swap_is_the_profile_probe_of_the_named_acf() {
        let rec = |p: &str| {
            json!({"kind": "profile_probe", "profile": "cirrus_sr22", "missing": [],
                   "identity": {"icao": "SR22", "relative_path": p}})
        };
        let want = "Aircraft/Laminar Research/Cirrus SR22/Cirrus SR22.acf";
        assert!(probe_is_for(&rec(want), want));
        assert!(probe_is_for(
            &rec("aircraft\\laminar research\\cirrus sr22\\cirrus sr22.acf"),
            want
        ));
        assert!(!probe_is_for(
            &rec("Aircraft/Laminar Research/Boeing 737-800/b738.acf"),
            want
        ));
        assert!(!probe_is_for(
            &json!({"kind": "systems", "identity": {"relative_path": want}}),
            want
        ));
        // A plugin probe or an older record without the identity is no swap.
        assert!(!probe_is_for(
            &json!({"kind": "profile_probe", "profile": "cirrus_sr22"}),
            want
        ));
    }

    fn begin(t: f64, s: &str) -> Value {
        rec(t, "sortie", json!({"event": "begin", "sortie": s}))
    }

    fn result(t: f64, s: &str, pass: bool, outcome: &str, commands: u64) -> Value {
        rec(
            t,
            "sortie_result",
            json!({"pass": pass, "outcome": outcome, "interventions": 0,
                   "commands": commands, "unattended": commands == 0, "sortie": s}),
        )
    }

    fn consumed(t: f64, text: &str) -> Value {
        rec(t, "typed", json!({"event": "consumed", "text": text}))
    }

    /// The resume case: "my controls" ends the approach's sortie on the
    /// way; the verdict is the resume's, typed after it.
    #[test]
    fn a_sortie_the_script_ends_is_not_the_verdict_the_next_one_is() {
        let mut m = CaseMachine::once(0.0, 900.0);
        m.pending_steps = 2;
        m.feed(&rec(0.5, "tool", json!({"name": "setup_approach"})));
        assert!(!m.opened(true), "an approach opens at its sortie");
        assert!(m.opened(false));
        assert_eq!(m.feed(&begin(60.0, "1-approach")), Step::Began);
        assert!(m.opened(true));
        m.note_step_sent(90.0, "my controls");
        m.feed(&consumed(90.2, "my controls"));
        m.feed(&rec(
            90.3,
            "tool",
            json!({"name": "autopilot_off", "sortie": "1-approach"}),
        ));
        m.feed(&rec(
            91.0,
            "stage_result",
            json!({"stage": "landing", "pass": false, "why": ["no touchdown"], "sortie": "1-approach"}),
        ));
        assert_eq!(
            m.feed(&result(91.1, "1-approach", false, "ended", 1)),
            Step::Wait,
            "a step is still to come"
        );
        assert!(matches!(m.phase, Phase::AwaitSortie { .. }));
        // No mission while a step is pending is no failure.
        assert_eq!(m.tick(91.1 + SORTIE_GRACE_S + 5.0, Some(false)), Step::Wait);
        let t = 91.1 + SORTIE_GRACE_S + 10.0;
        m.note_step_sent(t, "resume flight plan");
        m.feed(&consumed(t + 0.1, "resume flight plan"));
        m.feed(&rec(t + 0.2, "tool", json!({"name": "full_auto"})));
        assert_eq!(m.feed(&begin(t + 0.3, "2-full_auto")), Step::Began);
        m.feed(&rec(
            t + 300.0,
            "stage_result",
            json!({"stage": "landing", "pass": true, "why": [], "sortie": "2-full_auto"}),
        ));
        let v = finished(m.feed(&result(t + 301.0, "2-full_auto", true, "ended", 0)));
        assert!(v.pass, "{v:?}");
        assert_eq!(v.sortie.as_deref(), Some("2-full_auto"));
        assert_eq!(v.stages.len(), 1, "only the verdict sortie's stages");
        assert_eq!(v.note, "before it: 1-approach failed (ended)");
        // And the grace runs again once every step is typed.
        let mut m = CaseMachine::new(0.0, 900.0);
        m.pending_steps = 1;
        m.feed(&rec(0.5, "tool", json!({"name": "load_aircraft"})));
        assert_eq!(m.tick(SORTIE_GRACE_S + 10.0, Some(false)), Step::Wait);
        m.note_step_sent(SORTIE_GRACE_S + 20.0, "fly the flight plan");
        assert_eq!(m.tick(2.0 * SORTIE_GRACE_S + 19.0, Some(false)), Step::Wait);
        let v = finished(m.tick(2.0 * SORTIE_GRACE_S + 21.0, Some(false)));
        assert!(v.reason.contains("no mission began"), "{v:?}");
    }

    /// A mission a step supersedes is on the way too; without a step it is
    /// someone else's, and the case's failure.
    #[test]
    fn a_mission_superseded_by_a_step_hands_the_verdict_on() {
        let mut m = CaseMachine::new(0.0, 900.0);
        m.pending_steps = 1;
        m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
        m.feed(&begin(1.0, "1-full_auto"));
        m.note_step_sent(30.0, "fly the flight plan");
        m.feed(&consumed(30.1, "fly the flight plan"));
        assert_eq!(
            m.feed(&result(30.2, "1-full_auto", false, "superseded", 1)),
            Step::Wait
        );
        m.feed(&begin(30.3, "2-full_auto"));
        m.feed(&rec(
            99.0,
            "stage_result",
            json!({"stage": "landing", "pass": true, "sortie": "2-full_auto"}),
        ));
        let v = finished(m.feed(&result(100.0, "2-full_auto", true, "ended", 0)));
        assert!(
            v.pass && v.sortie.as_deref() == Some("2-full_auto"),
            "{v:?}"
        );

        let mut m = CaseMachine::new(0.0, 900.0);
        m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
        m.feed(&begin(1.0, "1-full_auto"));
        let v = finished(m.feed(&result(30.2, "1-full_auto", false, "superseded", 0)));
        assert!(v.reason.contains("superseded by another mission"), "{v:?}");
    }

    /// The scorer calls a sortie with a pilot's command attended; the
    /// runner knows which commands were its script's.
    #[test]
    fn the_scripts_own_commands_are_not_a_pilots() {
        let landing = |s: &str, pass: bool| {
            rec(
                50.0,
                "stage_result",
                json!({"stage": "landing", "pass": pass, "why": if pass { json!([]) } else { json!(["touched down at 700 fpm"]) }, "sortie": s}),
            )
        };
        let run = |sent: usize, commands: u64, stage_pass: bool| {
            let mut m = CaseMachine::new(0.0, 900.0);
            m.pending_steps = sent;
            m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
            m.feed(&begin(1.0, "s"));
            for i in 0..sent {
                let line = format!("line {i}");
                m.note_step_sent(10.0, &line);
                m.feed(&consumed(10.1, &line));
            }
            m.feed(&landing("s", stage_pass));
            finished(m.feed(&result(60.0, "s", false, "ended", commands)))
        };
        let v = run(1, 1, true);
        assert!(v.pass, "{v:?}");
        assert_eq!(v.scripted, 1);
        assert_eq!(v.note, "1 commands were the script's");
        // One more than the script typed: a pilot's.
        let v = run(1, 2, true);
        assert!(!v.pass);
        assert_eq!(
            v.reason,
            "attended (0 interventions, 2 commands, 1 of them the script's)"
        );
        // A failed stage still fails, and the script is not blamed.
        let v = run(1, 1, false);
        assert!(!v.pass);
        assert_eq!(v.reason, "landing: touched down at 700 fpm");
        // No script, a command: attended, as before.
        let v = run(0, 1, true);
        assert!(!v.pass && v.reason.starts_with("attended"), "{v:?}");
        // Ended in the air (#160): the script's words never make that a pass.
        let mut m = CaseMachine::new(0.0, 900.0);
        m.pending_steps = 1;
        m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
        m.feed(&begin(1.0, "s"));
        m.note_step_sent(10.0, "line 0");
        m.feed(&consumed(10.1, "line 0"));
        m.feed(&landing("s", true));
        let mut r = result(60.0, "s", false, "ended", 1);
        r["ended_airborne"] = json!(true);
        let v = finished(m.feed(&r));
        assert!(!v.pass, "{v:?}");
        assert_eq!(v.reason, "it ended in the air");
    }

    /// A rejected takeoff is meant to end full auto: that give-up passes,
    /// the roll it cut short too, and a roll off the centreline does not.
    #[test]
    fn a_meant_give_up_passes_and_nothing_else_does() {
        let run = |reason: &str, takeoff_why: Value, interventions: u64| {
            let mut m = CaseMachine::new(0.0, 900.0);
            m.expect_give_up = Some("takeoff_not_completed".into());
            m.pending_steps = 1;
            m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
            m.feed(&begin(1.0, "s"));
            m.note_step_sent(30.0, "reject the takeoff");
            m.feed(&consumed(30.1, "reject the takeoff"));
            m.feed(&rec(
                30.2,
                "tool",
                json!({"name": "auto_takeoff", "sortie": "s"}),
            ));
            m.feed(&rec(
                40.0,
                "stage_result",
                json!({"stage": "alignment", "pass": true, "why": [], "sortie": "s"}),
            ));
            let pass = takeoff_why.as_array().is_some_and(|a| a.is_empty());
            m.feed(&rec(
                40.0,
                "stage_result",
                json!({"stage": "takeoff", "pass": pass, "why": takeoff_why, "sortie": "s"}),
            ));
            m.feed(&rec(
                41.0,
                "full_auto",
                json!({"stage": "give_up", "detail": {"reason": reason}, "sortie": "s"}),
            ));
            finished(m.feed(&rec(
                42.0,
                "sortie_result",
                json!({"pass": false, "outcome": "gave_up", "interventions": interventions,
                       "commands": 1, "unattended": false, "sortie": "s"}),
            )))
        };
        let v = run("takeoff_not_completed", json!(["ended by give_up"]), 0);
        assert!(v.pass, "{v:?}");
        assert_eq!(
            v.note,
            "gave up as meant (takeoff_not_completed); 1 commands were the script's"
        );
        let v = run(
            "takeoff_not_completed",
            json!(["ended by give_up", "6.1 m off the centreline"]),
            0,
        );
        assert!(!v.pass);
        assert!(v.reason.contains("6.1 m off the centreline"), "{v:?}");
        let v = run("takeoff_rejected", json!(["ended by give_up"]), 0);
        assert!(!v.pass);
        assert!(
            v.reason
                .contains("expected the give-up takeoff_not_completed, the outcome was gave_up (takeoff_rejected)"),
            "{}",
            v.reason
        );
        let v = run("takeoff_not_completed", json!([]), 1);
        assert!(
            !v.pass && v.reason.contains("attended (1 interventions"),
            "{v:?}"
        );
        // The give-up as full auto really writes it (LEMG 13, sortie
        // 1790631201-full_auto): a stage with the reason under detail.
        let mut m = CaseMachine::new(0.0, 900.0);
        m.expect_give_up = Some("takeoff_not_completed".into());
        m.pending_steps = 1;
        m.feed(&rec(0.5, "tool", json!({"name": "full_auto"})));
        m.feed(&begin(1.0, "s"));
        m.note_step_sent(30.0, "reject the takeoff");
        m.feed(&consumed(30.1, "reject the takeoff"));
        m.feed(&rec(30.2, "tool", json!({"name": "auto_takeoff", "sortie": "s"})));
        m.feed(&rec(
            40.0,
            "stage_result",
            json!({"stage": "takeoff", "pass": false, "why": ["ended by reject"], "sortie": "s"}),
        ));
        m.feed(&rec(
            41.0,
            "full_auto",
            json!({"stage": "give_up", "detail": {"reason": "takeoff_not_completed"},
                   "sortie": "s"}),
        ));
        let v = finished(m.feed(&rec(
            42.0,
            "sortie_result",
            json!({"pass": false, "outcome": "gave_up", "interventions": 0,
                   "commands": 1, "unattended": false, "sortie": "s"}),
        )));
        assert!(v.pass, "{v:?}");
    }

    #[test]
    fn the_shipped_spec_parses() {
        let spec = parse_spec(LEMG).expect("lemg spec");
        assert_eq!(spec.cases.len(), 3);
        assert_eq!(
            spec.cases[0].setup,
            Setup::Approach {
                field: "LEMG".into(),
                runway: RUNWAY_FROM_WIND.into(),
                final_nm: 10
            }
        );
        assert_eq!(
            spec.cases[1].setup,
            Setup::Approach {
                field: "LEMG".into(),
                runway: RUNWAY_FROM_WIND.into(),
                final_nm: 5
            }
        );
        assert!(matches!(
            spec.cases[2].setup,
            Setup::RunwayStart {
                engines_running: true,
                ..
            }
        ));
        assert!(spec.cases.iter().all(Case::wants_wind), "#158: the wind's runway");
        assert_eq!(spec.cases[2].command.as_deref(), Some("fly a circuit"));
        assert_eq!(spec.cases[1].deadline_s(), 8.0 * 60.0);
        assert_eq!(spec.cases[0].deadline_s(), 10.0 * 60.0);
        let bare =
            parse_spec("cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}}]").unwrap();
        assert_eq!(bare.cases[0].deadline_s(), DEFAULT_DEADLINE_MIN * 60.0);
        assert_eq!(
            bare.cases[0].setup,
            Setup::Approach {
                field: "LEMG".into(),
                runway: "13".into(),
                final_nm: 10
            },
            "a final with no distance is the setup tool's ten miles"
        );
        assert!(bench_specs().len() >= 3);
    }

    #[test]
    fn a_low_descending_aircraft_is_not_frozen() {
        assert!(!pause_safe(204.0, -2_500.0));
        assert!(pause_safe(204.0, 500.0));
        assert!(pause_safe(3_000.0, -2_500.0));
        // Above the escape height but 44 s from the ground: the Zibo frozen
        // 2.6 s into a recovery at LEMG 30 (2026-10-07, #232) waits now.
        assert!(!pause_safe(1_343.0, -1_811.0));
        assert!(pause_safe(1_343.0, -150.0));
    }

    #[test]
    fn traffic_is_a_switch_per_case() {
        let spec = parse_spec(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, traffic: off},\n\
             {name: b, setup: {approach: {field: LEMG, runway: 13}}, traffic: on},\n\
             {name: c, setup: {approach: {field: LEMG, runway: 13}}}]",
        )
        .unwrap();
        let t: Vec<_> = spec.cases.iter().map(|c| c.traffic).collect();
        assert_eq!(t, vec![Some(false), Some(true), None]);
        let bad = parse_spec(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, traffic: dim}]",
        );
        assert!(bad.is_err());
    }

    #[test]
    fn a_bad_spec_is_refused_naming_the_case() {
        let cases = [
            ("cases: []", "no cases"),
            ("cases: [{name: a, setup: {ramp: {field: LEMG}}}]", "needs a command"),
            (
                "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, command: land the aircraft}]",
                "has no command",
            ),
            ("cases: [{name: a, setup: {approach: {field: LEMG, runway: 13, final_nm: 40}}}]", "final_nm"),
            ("cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, sim_speed: 0}]", "sim_speed"),
            ("cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}, deadline_min: 0}]", "deadline"),
            ("cases: [{name: a, setup: {hover: {field: LEMG}}, command: x}]", "unknown field `hover`"),
            (
                "cases: [{name: a, setup: {ramp: {field: LEMG}, runway_start: {field: LEMG, runway: 1}}, command: x}]",
                "exactly one of",
            ),
            ("cases: [{name: a, setup: {ramp: {field: LEMG}}, comand: x}]", "unknown field"),
        ];
        for (yaml, want) in cases {
            let e = parse_spec(yaml).expect_err(yaml).to_string();
            assert!(e.contains(want), "{yaml}: {e}");
        }
        // A runway YAML reads as a number, and a lower-case field, are names.
        let s = parse_spec("cases: [{name: a, setup: {runway_start: {field: lemg, runway: 31}}, command: full auto}]")
            .unwrap();
        assert_eq!(
            s.cases[0].setup,
            Setup::RunwayStart {
                field: "LEMG".into(),
                runway: "31".into(),
                engines_running: false
            }
        );
    }

    #[test]
    fn the_approach_line_is_the_setup_tool_in_the_grammar() {
        let prof = crate::aircraft::load("boeing_737_800").expect("737 profile");
        let reg = crate::tools::Registry::for_aircraft(&prof);
        for (field, rwy, nm, want_rwy) in [("LEMG", "13", 10, "13"), ("EGKK", "26L", 5, "26L")] {
            let line = approach_line(field, rwy, nm);
            let calls =
                crate::grammar::try_parse(&line, &reg, &prof).unwrap_or_else(|| panic!("{line}"));
            assert_eq!(calls[0].name, "setup_approach", "{line}");
            let a = &calls[0].arguments;
            assert_eq!(a["final_nm"], nm, "{line}");
            assert_eq!(a["runway"], want_rwy, "{line}");
            assert_eq!(a["to_codes"], json!([field]), "{line}");
            assert!(a.get("hand_fly").is_none(), "{line}");
        }
        // The shipped runway-start case's command is full auto's circuit,
        // a sortie of its own.
        let spec = parse_spec(LEMG).unwrap();
        let cmd = spec.cases[2].command.clone().unwrap();
        let calls = crate::grammar::try_parse(&cmd, &reg, &prof).expect("circuit command");
        assert_eq!(calls[0].name, "full_auto");
    }

    #[test]
    fn the_runway_start_json_is_the_sdks_shape() {
        let v: Value =
            serde_json::from_str(&runway_start_json("Aircraft/x.acf", "LEMG", "13", true)).unwrap();
        assert_eq!(v["runway_start"]["airport_id"], "LEMG");
        assert_eq!(v["runway_start"]["runway"], "13");
        assert!(v["runway_start"]
            .get("final_distance_in_nautical_miles")
            .is_none());
        assert_eq!(v["engine_status"]["all_engines"]["running"], true);
        assert_eq!(v["aircraft"]["path"], "Aircraft/x.acf");
    }

    fn rec(t: f64, kind: &str, extra: Value) -> Value {
        let mut v = json!({"t": t, "kind": kind});
        for (k, x) in extra.as_object().unwrap() {
            v[k] = x.clone();
        }
        v
    }

    fn finished(s: Step) -> Verdict {
        match s {
            Step::Finished(v) => v,
            other => panic!("not finished: {other:?}"),
        }
    }

    #[test]
    fn a_landing_case_runs_from_the_command_to_the_verdict() {
        let mut m = CaseMachine::new(100.0, 600.0);
        assert_eq!(
            m.feed(&rec(100.5, "tool", json!({"name": "setup_approach"}))),
            Step::Wait
        );
        assert!(matches!(m.phase, Phase::AwaitSortie { .. }));
        // The reload's wait is no failure inside the grace.
        assert_eq!(m.tick(150.0, None), Step::Wait);
        let s = "1790600000-approach";
        assert_eq!(
            m.feed(&rec(
                160.0,
                "sortie",
                json!({"event": "begin", "sortie": s})
            )),
            Step::Began
        );
        // Another sortie's stage is not this case's.
        m.feed(&rec(
            170.0,
            "stage_result",
            json!({"stage": "taxi", "pass": false, "sortie": "x"}),
        ));
        m.feed(&rec(
            300.0,
            "stage_result",
            json!({"stage": "landing", "pass": true, "why": [],
            "sink_fpm": 222.0, "drift_deg": 1.2, "rollout_max_xtk_m": 0.8, "sortie": s}),
        ));
        let v = finished(m.feed(&rec(
            320.0,
            "sortie_result",
            json!({"pass": true, "outcome": "ended",
            "interventions": 0, "commands": 0, "unattended": true, "sortie": s}),
        )));
        assert!(v.pass, "{v:?}");
        assert_eq!(v.sortie.as_deref(), Some(s));
        assert_eq!(v.stages.len(), 1);
        assert!(v.reason.is_empty());
        assert!(!v.abort);
        assert_eq!(
            m.tick(330.0, Some(true)),
            Step::Wait,
            "a finished case stays finished"
        );
    }

    #[test]
    fn a_failed_sortie_names_its_stages_and_the_hands() {
        let mut m = CaseMachine::new(0.0, 600.0);
        m.feed(&rec(1.0, "tool", json!({"name": "full_auto"})));
        m.feed(&rec(
            5.0,
            "sortie",
            json!({"event": "begin", "sortie": "9-full_auto"}),
        ));
        m.feed(&rec(
            50.0,
            "full_auto",
            json!({"stage": "give_up", "detail": {"reason": "stagnant"},
                   "sortie": "9-full_auto"}),
        ));
        m.feed(&rec(
            50.0,
            "stage_result",
            json!({"stage": "landing", "pass": false,
            "why": ["touched down at 794 fpm", "bounced to 12 ft"], "sortie": "9-full_auto"}),
        ));
        let v = finished(m.feed(&rec(
            51.0,
            "sortie_result",
            json!({"pass": false, "outcome": "gave_up",
            "interventions": 1, "commands": 0, "unattended": false, "sortie": "9-full_auto"}),
        )));
        assert!(!v.pass);
        assert_eq!(
            v.reason,
            "landing: touched down at 794 fpm, bounced to 12 ft; gave up (stagnant); attended (1 interventions, 0 commands)"
        );
        assert_eq!(v.interventions, 1);
    }

    #[test]
    fn a_command_nobody_takes_is_sent_again_once_then_fails() {
        let mut m = CaseMachine::new(0.0, 600.0);
        assert_eq!(m.tick(4.0, Some(false)), Step::Wait);
        assert_eq!(m.tick(5.5, Some(false)), Step::Resend);
        assert_eq!(m.tick(9.0, Some(false)), Step::Wait);
        let v = finished(m.tick(11.0, Some(false)));
        assert!(v.reason.contains("did not take"), "{v:?}");
        assert!(!v.abort, "nothing was armed, nothing to take back");
        // A crew line is an answer too: the resend's clock is stopped.
        let mut m = CaseMachine::new(0.0, 600.0);
        m.feed(&rec(
            2.0,
            "said",
            json!({"who": "crew", "text": "Setting up a 5 mile final."}),
        ));
        assert!(matches!(m.phase, Phase::AwaitSortie { .. }));
        // A setup line is never typed twice.
        let mut m = CaseMachine::once(0.0, 600.0);
        let v = finished(m.tick(5.5, Some(false)));
        assert!(v.reason.contains("did not take"), "{v:?}");
    }

    /// LEMG 2026-09-28 (sortie 1790586620, #135): "fly a circuit" was taken
    /// by the app's poller at 1790586608.6, retyped at 614.4, failed as "the
    /// app did not take the command", and both copies were read at 620.2 and
    /// 620.3 behind a tower reply: the mission flew unobserved and the second
    /// copy got "I'm already running the full sequence". A TAKEN line is
    /// waited for, never retyped, and the case follows the mission it began.
    #[test]
    fn a_line_the_app_has_taken_is_waited_for_never_typed_again() {
        let t0 = 1790586608.3;
        let mut m = CaseMachine::new(t0, 1200.0);
        m.line = Some("fly a circuit".into());
        m.feed(&rec(
            1790586608.6,
            "typed",
            json!({"event": "taken", "text": "fly a circuit", "age_s": 0.25}),
        ));
        assert!(matches!(m.phase, Phase::Queued { .. }));
        // The old clocks: a resend at +5 s, a failure at +10 s. Neither now.
        assert_eq!(m.tick(t0 + 5.5, Some(false)), Step::Wait);
        assert_eq!(m.tick(t0 + 11.0, Some(false)), Step::Wait);
        // A crew line while it waits is chatter (the After landing list).
        m.feed(&rec(1790586615.0, "said", json!({"who": "crew", "text": "After landing."})));
        assert!(matches!(m.phase, Phase::Queued { .. }));
        m.feed(&rec(
            1790586620.2,
            "typed",
            json!({"event": "consumed", "text": "fly a circuit", "queued_ms": 11663}),
        ));
        m.feed(&rec(1790586620.2, "tool", json!({"name": "full_auto"})));
        assert!(matches!(m.phase, Phase::AwaitSortie { .. }));
        assert_eq!(
            m.feed(&rec(
                1790586620.3,
                "sortie",
                json!({"event": "begin", "sortie": "1790586620-full_auto"}),
            )),
            Step::Began
        );

        // Taken and never read: failed, and the aircraft taken back, since
        // the app still holds the line and may act on it later.
        let mut m = CaseMachine::new(0.0, 1200.0);
        m.line = Some("fly a circuit".into());
        m.feed(&rec(0.3, "typed", json!({"event": "taken", "text": "fly a circuit"})));
        assert_eq!(m.tick(59.0, Some(false)), Step::Wait);
        let v = finished(m.tick(61.0, Some(false)));
        assert!(v.reason.contains("never read it"), "{v:?}");
        assert!(v.abort);

        // Another line taken (the pilot's own) is not this case's.
        let mut m = CaseMachine::new(0.0, 1200.0);
        m.line = Some("fly a circuit".into());
        m.feed(&rec(0.3, "typed", json!({"event": "taken", "text": "my controls"})));
        assert_eq!(m.tick(5.5, Some(false)), Step::Resend);

        // Read at once and never answered: no second copy either.
        let mut m = CaseMachine::new(0.0, 1200.0);
        m.feed(&rec(0.3, "typed", json!({"event": "taken", "text": "full auto"})));
        m.feed(&rec(0.4, "typed", json!({"event": "consumed", "text": "full auto"})));
        let v = finished(m.tick(5.5, Some(false)));
        assert!(v.reason.contains("did not take"), "{v:?}");
    }

    #[test]
    fn a_setup_that_never_arms_fails_with_the_crews_words() {
        let mut m = CaseMachine::new(0.0, 600.0);
        m.feed(&rec(1.0, "tool", json!({"name": "setup_approach"})));
        m.feed(&rec(
            2.0,
            "said",
            json!({"who": "crew", "text": "The approach setup didn't answer. Check X-Plane."}),
        ));
        let v = finished(m.feed(&rec(
            3.0,
            "setup",
            json!({"event": "approach_result", "outcome": "timeout",
            "result": null, "armed": false}),
        )));
        assert!(v.reason.contains("did not arm (timeout"), "{}", v.reason);
        assert!(v.reason.contains("didn't answer"), "{}", v.reason);
        // Taken, and no mission ever began: the grace ends it, with the
        // refusal the crew spoke.
        let mut m = CaseMachine::new(0.0, 6000.0);
        m.feed(&rec(1.0, "tool", json!({"name": "full_auto"})));
        m.feed(&rec(
            1.5,
            "said",
            json!({"who": "crew", "text": "We're in replay."}),
        ));
        assert_eq!(m.tick(100.0, Some(false)), Step::Wait);
        let v = finished(m.tick(1.0 + SORTIE_GRACE_S + 1.0, Some(false)));
        assert_eq!(
            v.reason,
            "no mission began after full_auto; the crew said \"We're in replay.\""
        );
        assert!(v.abort);
    }

    #[test]
    fn a_crash_or_the_deadline_ends_the_case_and_takes_the_aircraft_back() {
        let mut m = CaseMachine::new(0.0, 600.0);
        m.feed(&rec(1.0, "tool", json!({"name": "full_auto"})));
        m.feed(&rec(
            2.0,
            "sortie",
            json!({"event": "begin", "sortie": "s"}),
        ));
        // A failed read is no measurement, never a crash.
        assert_eq!(m.tick(10.0, None), Step::Wait);
        assert_eq!(m.tick(10.5, Some(false)), Step::Wait);
        let v = finished(m.tick(11.0, Some(true)));
        assert_eq!(v.reason, "the aircraft crashed");
        assert_eq!(v.sortie.as_deref(), Some("s"));
        assert!(v.abort && v.crashed && !v.stop_run);
        assert_eq!(
            hand_back(&v, Some(false)),
            HandBack::MyControls,
            "a wreck is nobody's flight"
        );
        let mut m = CaseMachine::new(0.0, 600.0);
        m.feed(&rec(1.0, "tool", json!({"name": "full_auto"})));
        m.feed(&rec(
            2.0,
            "sortie",
            json!({"event": "begin", "sortie": "s"}),
        ));
        assert_eq!(m.tick(600.0, Some(false)), Step::Wait);
        let v = finished(m.tick(601.0, Some(false)));
        assert_eq!(v.reason, "deadline 10 min passed");
        assert!(v.abort && !v.pass && !v.crashed);
    }

    /// LEMG 2026-09-28: 1790584266-approach crashed, and the next three cases
    /// each failed in 2 s as "the aircraft crashed", no sortie: the wreck
    /// flag stays up until the sim resets, and the new case's setup is that
    /// reset.
    #[test]
    fn the_last_cases_wreck_is_waited_out_then_stops_the_run() {
        // The setup's reload clears it: the case flies on, and a wreck
        // after that is this case's.
        let mut m = CaseMachine::once(0.0, 600.0);
        assert_eq!(m.tick(1.0, Some(true)), Step::Wait, "the last case's wreck");
        m.feed(&rec(1.5, "tool", json!({"name": "setup_approach"})));
        assert_eq!(m.tick(60.0, Some(true)), Step::Wait, "still reloading");
        assert_eq!(
            m.tick(70.0, Some(false)),
            Step::Wait,
            "the reload cleared it"
        );
        m.feed(&rec(
            71.0,
            "sortie",
            json!({"event": "begin", "sortie": "s"}),
        ));
        let v = finished(m.tick(200.0, Some(true)));
        assert_eq!(v.reason, "the aircraft crashed");
        assert!(v.crashed && !v.stop_run);

        // It never clears: one line, and the run stops.
        let mut m = CaseMachine::once(0.0, 600.0);
        m.feed(&rec(1.5, "tool", json!({"name": "setup_approach"})));
        assert_eq!(m.tick(CRASH_CLEAR_S, Some(true)), Step::Wait);
        let v = finished(m.tick(CRASH_CLEAR_S + 0.5, Some(true)));
        assert_eq!(v.reason, STILL_CRASHED);
        assert!(v.stop_run && !v.crashed && !v.abort, "{v:?}");
        assert_eq!(hand_back(&v, None), HandBack::Nothing);
    }

    #[test]
    fn a_case_ended_in_the_air_is_never_handed_to_nobody() {
        let deadline = Verdict {
            abort: true,
            reason: "deadline 10 min passed".into(),
            ..Verdict::default()
        };
        // 1,600 ft on a 5 nm final: the loops keep it.
        assert_eq!(hand_back(&deadline, Some(false)), HandBack::LeaveFlying);
        // Unreadable wheels are no measurement: treated as flying.
        assert_eq!(hand_back(&deadline, None), HandBack::LeaveFlying);
        assert_eq!(hand_back(&deadline, Some(true)), HandBack::MyControls);
        assert_eq!(
            hand_back(&Verdict::default(), Some(false)),
            HandBack::Nothing
        );
    }

    #[test]
    fn the_run_table_has_a_row_per_case_with_numbers_and_limits() {
        let pass = Row {
            case: "LEMG 13, 5 nm final".into(),
            rep: 2,
            secs: 181.4,
            verdict: Verdict {
                pass: true,
                sortie: Some("1790600000-approach".into()),
                stages: vec![json!({"stage": "landing", "pass": true, "sink_fpm": 222.0,
                    "drift_deg": 1.2, "rollout_max_xtk_m": 0.8})],
                outcome: "ended".into(),
                ..Verdict::default()
            },
        };
        let fail = Row {
            case: "LEMG circuit".into(),
            rep: 1,
            secs: 900.0,
            verdict: Verdict {
                sortie: Some("1790600300-full_auto".into()),
                stages: vec![
                    json!({"stage": "taxi", "pass": false, "max_lat_m": 5.8, "max_cut_m": 1.0}),
                    json!({"stage": "alignment", "pass": true, "err_deg": 0.4, "xtk_m": 0.3}),
                    json!({"stage": "takeoff", "pass": true, "max_xtk_m": 1.1}),
                ],
                interventions: 1,
                reason: "taxi: 5.8 m off the line | x".into(),
                settle: Some(Settle {
                    secs: 42.3,
                    end: SettleEnd::AtRest,
                }),
                fast_s: 600.2,
                fast_x: Some(3.94),
                ..Verdict::default()
            },
        };
        assert!(
            row_line(&fail).contains("in 900 s (fast 600 s at x3.9) (waited 42 s"),
            "{}",
            row_line(&fail)
        );
        assert_eq!(
            row_line(&pass),
            "PASS LEMG 13, 5 nm final #2: landing pass (sink 222/240 fpm, drift 1.2/5 deg, rollout 0.8/5 m) \
             in 181 s, sortie 1790600000-approach, 0 interventions"
        );
        let t = table(
            "bench/sorties/lemg-737.yaml",
            "20260928-010203Z",
            &[pass, fail],
        );
        let lines: Vec<&str> = t.lines().collect();
        assert_eq!(lines[0], "# Sortie run 20260928-010203Z");
        assert!(lines[4].starts_with("| Case |"), "{t}");
        assert_eq!(
            lines[7],
            "| LEMG circuit | 1 | 1790600300-full_auto | FAIL | taxi FAIL (lat 5.8/3 m, cut 1/5 m); \
             alignment pass (err 0.4/3 deg, xtk 0.3/2 m); takeoff pass (xtk 1.1/5 m) | 1 | 900 | 600 s at x3.9 | \
             waited 42 s for the last aircraft to stop | taxi: 5.8 m off the line / x |"
        );
        assert!(
            lines[6].contains("| 181 | - |"),
            "nothing waited for is a dash: {t}"
        );
        assert_eq!(*lines.last().unwrap(), "1 of 2 passed.");
    }

    #[test]
    fn the_tail_reads_whole_new_lines_and_survives_a_rotation() {
        use std::io::Write;
        let dir = std::env::temp_dir().join(format!("xvl-tail-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("bb.jsonl");
        std::fs::write(&p, "{\"kind\":\"old\"}\n").unwrap();
        let mut tail = BlackboxTail::at_end(&p);
        assert!(tail.poll().is_empty(), "what was there before is not fed");
        let mut f = std::fs::OpenOptions::new().append(true).open(&p).unwrap();
        write!(f, "{{\"kind\":\"a\"}}\n{{\"kind\":\"b\"").unwrap();
        f.flush().unwrap();
        let got = tail.poll();
        assert_eq!(got.len(), 1, "a half-written line waits");
        assert_eq!(got[0]["kind"], "a");
        writeln!(f, "}}").unwrap();
        f.flush().unwrap();
        assert_eq!(tail.poll()[0]["kind"], "b");
        // Rotated: the new, shorter file from its start.
        std::fs::write(&p, "{\"kind\":\"c\"}\n").unwrap();
        assert_eq!(tail.poll()[0]["kind"], "c");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// #144: a corrupt line (two records interleaved, as the 2026-09-28 log
    /// held 70 of) is skipped and the whole line after it is still read; a
    /// multi-byte character split across two polls arrives intact.
    #[test]
    fn the_tail_skips_a_corrupt_line_and_keeps_the_next() {
        use std::io::Write;
        let dir = std::env::temp_dir().join(format!("xvl-tail-bad-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("bb.jsonl");
        std::fs::write(&p, "").unwrap();
        let mut tail = BlackboxTail::at_end(&p);
        let mut f = std::fs::OpenOptions::new().append(true).open(&p).unwrap();
        write!(
            f,
            "{{\"{{dtcache_hits\"\"::0.6,,\"event\":\"power\"\n\
             throttle_before\":0.27,\"write_ok\":true}}\n\
             {{\"kind\":\"stage_result\",\"stage\":\"landing\"}}\n"
        )
        .unwrap();
        f.flush().unwrap();
        let got = tail.poll();
        assert_eq!(got.len(), 1, "{got:?}");
        assert_eq!(got[0]["kind"], "stage_result");
        let line = "{\"kind\":\"said\",\"text\":\"10\u{b0}C\"}\n".as_bytes();
        let cut = line.iter().position(|&b| b == 0xC2).unwrap() + 1;
        f.write_all(&line[..cut]).unwrap();
        f.flush().unwrap();
        assert!(tail.poll().is_empty());
        f.write_all(&line[cut..]).unwrap();
        f.flush().unwrap();
        assert_eq!(tail.poll()[0]["text"], "10\u{b0}C");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_stamp_is_a_utc_date() {
        let s = chrono_stamp();
        assert_eq!(s.len(), 16, "{s}");
        assert!(s.starts_with("20") && s.ends_with('Z'), "{s}");
    }

    /// A sim that answers the runner's reads, takes typed lines at once, and
    /// writes the app's blackbox records for the approach it was typed.
    struct FakeSim {
        typed: std::sync::Mutex<Vec<String>>,
        speeds: std::sync::Mutex<Vec<f64>>,
        log: std::path::PathBuf,
        flying: std::sync::atomic::AtomicBool,
        inits: std::sync::Mutex<Vec<String>>,
        /// The wreck flag, as the sim reports it.
        crashed: std::sync::atomic::AtomicBool,
        /// The wheels are off the ground.
        airborne: std::sync::atomic::AtomicBool,
        /// A typed mission begins and then never ends (the deadline's case).
        endless: std::sync::atomic::AtomicBool,
        commands: std::sync::Mutex<Vec<String>>,
        /// After a mission's verdict the aircraft rolls on (its taxi-in, no
        /// loop on the marker, as LEMG 31 read) for this many groundspeed
        /// reads; `u32::MAX` until "my controls".
        roll_after_mission: std::sync::atomic::AtomicU32,
        rolling_reads: std::sync::atomic::AtomicU32,
        /// A setup (mailbox post or approach line) while it rolled.
        setups_while_rolling: std::sync::atomic::AtomicU32,
        /// A mailbox post loading another aircraft is answered by the app's
        /// detector (`profile_probe` of it).
        probe_on_swap: std::sync::atomic::AtomicBool,
        /// The METAR on file for any field (direction true, knots).
        metar: std::sync::Mutex<Option<(f64, f64)>>,
        /// The aircraft's position and the wind at it (m/s).
        at: std::sync::Mutex<(f64, f64, f64, f64)>,
        /// Every aircraft's approach speed.
        vapp: std::sync::Mutex<Option<f64>>,
        /// `sim/time/sim_speed` (and what it achieves).
        speed: std::sync::Mutex<f64>,
        /// The follower's cruise: 3,000 m up, the servos in, the panel on
        /// full auto's cruise, and an endless mission's model in Cruise.
        enroute: std::sync::atomic::AtomicBool,
        /// The first fast write wrecks the aircraft.
        crash_when_fast: std::sync::atomic::AtomicBool,
        /// The app refuses a typed enroute setup (its `enroute_refused`).
        enroute_refuses: std::sync::atomic::AtomicBool,
    }

    /// The fake's taxi-in pace, 14 kt in metres per second.
    const ROLL_MPS: f64 = 7.2;

    impl FakeSim {
        /// A setup reloads the sim: whatever rolled is stopped, and counted.
        fn setup_posted(&self) {
            use std::sync::atomic::Ordering::SeqCst;
            if self.rolling_reads.swap(0, SeqCst) > 0 {
                self.setups_while_rolling.fetch_add(1, SeqCst);
            }
        }
        fn new(log: std::path::PathBuf) -> Self {
            Self {
                typed: Default::default(),
                speeds: Default::default(),
                log,
                flying: false.into(),
                inits: Default::default(),
                crashed: false.into(),
                airborne: false.into(),
                endless: false.into(),
                commands: Default::default(),
                roll_after_mission: 0.into(),
                rolling_reads: 0.into(),
                setups_while_rolling: 0.into(),
                probe_on_swap: false.into(),
                metar: Default::default(),
                at: Default::default(),
                vapp: Default::default(),
                speed: std::sync::Mutex::new(1.0),
                enroute: false.into(),
                crash_when_fast: false.into(),
                enroute_refuses: false.into(),
            }
        }
    }

    #[async_trait]
    impl RunnerSim for FakeSim {
        async fn read(&self, name: &str) -> Result<f64> {
            use std::sync::atomic::Ordering::SeqCst;
            let posted = self.inits.lock().unwrap().len() as f64;
            let flag = |b: bool| if b { 1.0 } else { 0.0 };
            Ok(match name {
                "xcopilot/sim/init_seq" => posted,
                crate::datarefs::ON_GROUND_ANY => flag(!self.airborne.load(SeqCst)),
                crate::datarefs::HAS_CRASHED => flag(self.crashed.load(SeqCst)),
                crate::datarefs::LATITUDE => self.at.lock().unwrap().0,
                crate::datarefs::LONGITUDE => self.at.lock().unwrap().1,
                crate::datarefs::WIND_DIR_DEG_TRUE => self.at.lock().unwrap().2,
                crate::datarefs::WIND_SPEED_MSC => self.at.lock().unwrap().3,
                SIM_SPEED | SIM_SPEED_ACTUAL => *self.speed.lock().unwrap(),
                crate::datarefs::AGL_METERS if self.enroute.load(SeqCst) => 3_000.0,
                crate::datarefs::SERVOS_ON => flag(self.enroute.load(SeqCst)),
                GROUNDSPEED => {
                    let left = self.rolling_reads.load(SeqCst);
                    if left > 0 && left != u32::MAX {
                        self.rolling_reads.store(left - 1, SeqCst);
                    }
                    if left > 0 {
                        ROLL_MPS
                    } else {
                        0.0
                    }
                }
                // The pause the runner commanded, read back.
                PAUSED => f64::from(u8::from(
                    self.commands.lock().unwrap().iter().any(|c| c == PAUSE_ON),
                )),
                _ => 0.0,
            })
        }
        async fn read_string(&self, name: &str) -> Result<String> {
            Ok(match name {
                STATUS_AUTO
                    if self.enroute.load(std::sync::atomic::Ordering::SeqCst)
                        && self.flying.load(std::sync::atomic::Ordering::SeqCst) =>
                {
                    "1FULL AUTO|cruise\nflying a procedure: to PIMOS, 42.0 nm".into()
                }
                STATUS_AUTO if self.flying.load(std::sync::atomic::Ordering::SeqCst) => {
                    "1AUTOLAND|final".into()
                }
                STATUS_AUTO => "0MONITORING|".into(),
                ACF_PATH => "Aircraft/Laminar Research/Boeing 737-800/b738.acf".into(),
                _ => String::new(),
            })
        }
        async fn write_float(&self, name: &str, v: f64) -> Result<()> {
            use std::sync::atomic::Ordering::SeqCst;
            assert_eq!(name, SIM_SPEED);
            self.speeds.lock().unwrap().push(v);
            *self.speed.lock().unwrap() = v;
            if v > 1.0 && self.crash_when_fast.load(SeqCst) {
                self.crashed.store(true, SeqCst);
            }
            Ok(())
        }
        async fn command(&self, name: &str) -> Result<()> {
            self.commands.lock().unwrap().push(name.to_string());
            Ok(())
        }
        async fn metar_wind(&self, _icao: &str) -> Option<(f64, f64)> {
            *self.metar.lock().unwrap()
        }
        async fn approach_kts(&self, _acf: &str) -> Option<f64> {
            *self.vapp.lock().unwrap()
        }
        async fn write_bytes(&self, name: &str, bytes: &[u8]) -> Result<()> {
            use std::sync::atomic::Ordering::SeqCst;
            let line = String::from_utf8_lossy(bytes).to_string();
            if name == "xcopilot/sim/init_flight" {
                self.setup_posted();
                let v: Value = serde_json::from_str(&line).unwrap();
                let acf = v["aircraft"]["path"].as_str().unwrap_or("").to_string();
                self.inits.lock().unwrap().push(line);
                if self.probe_on_swap.load(SeqCst) && !acf.ends_with("b738.acf") {
                    use std::io::Write;
                    let mut f = std::fs::OpenOptions::new()
                        .append(true)
                        .open(&self.log)
                        .unwrap();
                    let r = json!({"t": 1.0, "kind": "profile_probe", "missing": [],
                                   "identity": {"relative_path": acf}});
                    writeln!(f, "{r}").unwrap();
                }
                return Ok(());
            }
            if line.starts_with("put us on") {
                self.setup_posted();
            }
            if line.starts_with("put us enroute") {
                // The app's enroute setup: its tool, the new flight in the
                // air with the servos confirmed (or its refusal).
                self.setup_posted();
                self.typed.lock().unwrap().push(line.clone());
                use std::io::Write;
                let mut f = std::fs::OpenOptions::new()
                    .append(true)
                    .open(&self.log)
                    .unwrap();
                let end = if self.enroute_refuses.load(SeqCst) {
                    json!({"t": 2.0, "kind": "setup", "event": "enroute_refused",
                           "why": "The plan is for a A320, and this is a B738."})
                } else {
                    self.airborne.store(true, SeqCst);
                    json!({"t": 2.0, "kind": "setup", "event": "enroute_result", "outcome": "ok",
                           "engaged": true, "next_fix": "WELIN"})
                };
                for r in [json!({"t": 1.0, "kind": "tool", "name": "setup_enroute"}), end] {
                    writeln!(f, "{r}").unwrap();
                }
                return Ok(());
            }
            assert_eq!(name, TYPED);
            self.typed.lock().unwrap().push(line.clone());
            use std::io::Write;
            let mut f = std::fs::OpenOptions::new()
                .append(true)
                .open(&self.log)
                .unwrap();
            if line == ABORT_LINE {
                self.flying.store(false, SeqCst);
                // hand_back_hold: brakes to a stop in a few reads.
                let left = self.rolling_reads.load(SeqCst);
                self.rolling_reads.store(left.min(3), SeqCst);
            } else if self.endless.load(SeqCst) {
                let s = format!("{}-endless", self.typed.lock().unwrap().len());
                self.flying.store(true, SeqCst);
                for r in [
                    json!({"t": 1.0, "kind": "tool", "name": "setup_approach"}),
                    json!({"t": 2.0, "kind": "sortie", "event": "begin", "sortie": s}),
                ] {
                    writeln!(f, "{r}").unwrap();
                }
                if self.enroute.load(SeqCst) {
                    let r = json!({"t": 3.0, "kind": "state", "fphase": "Cruise", "sortie": s});
                    writeln!(f, "{r}").unwrap();
                }
            } else {
                let s = "77-mission";
                let tool = if line.starts_with("put us on") {
                    "setup_approach"
                } else {
                    "full_auto"
                };
                for r in [
                    json!({"t": 1.0, "kind": "tool", "name": tool}),
                    json!({"t": 2.0, "kind": "sortie", "event": "begin", "sortie": s}),
                    json!({"t": 3.0, "kind": "stage_result", "stage": "landing", "pass": true,
                           "why": [], "sink_fpm": 180.0, "sortie": s}),
                    json!({"t": 4.0, "kind": "sortie_result", "pass": true, "outcome": "ended",
                           "interventions": 0, "commands": 0, "unattended": true, "sortie": s}),
                ] {
                    writeln!(f, "{r}").unwrap();
                }
                self.rolling_reads
                    .store(self.roll_after_mission.load(SeqCst), SeqCst);
            }
            Ok(())
        }
    }

    #[tokio::test]
    async fn the_driver_flies_a_spec_against_a_fake_sim() {
        let dir = std::env::temp_dir().join(format!("xvl-runner-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let log = dir.join("blackbox.jsonl");
        std::fs::write(&log, "").unwrap();
        let sim = FakeSim::new(log.clone());
        let spec = parse_spec(
            "cases: [{name: five, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}}, sim_speed: 2},\n\
                     {name: circuit, setup: {runway_start: {field: LEMG, runway: 13, engines_running: true}},\n\
                      command: fly a circuit}]",
        )
        .unwrap();
        let pace = Pace {
            poll: std::time::Duration::from_millis(5),
            init: InitTiming::LIVE,
            still_s: 0.0,
            still_max_s: 1.0,
            settle_scale: 0.0,
            idle_max_s: 1.0,
            quiet_s: 0.02,
            stop_max_s: 0.5,
            crash_clear_s: 0.2,
            swap_max_s: 1.0,
            fast_resume_s: 0.0,
            enroute_max_s: 1.0,
        };
        let mut tail = BlackboxTail::at_end(&log);
        let mut seen = Vec::new();
        let rows = run_all(&sim, &mut tail, &spec, 2, "", &pace, &mut |r: &Row| {
            seen.push(row_line(r))
        })
        .await;
        assert_eq!(rows.len(), 4);
        assert!(rows.iter().all(|r| r.verdict.pass), "{seen:?}");
        assert_eq!(rows[1].rep, 2);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec![
                "put us on a 5 mile final to LEMG runway 13",
                "put us on a 5 mile final to LEMG runway 13",
                "fly a circuit",
                "fly a circuit",
            ],
            "one setup per case, never a second while one runs"
        );
        // The runway start went through the mailbox, once per case.
        let inits = sim.inits.lock().unwrap().clone();
        assert_eq!(inits.len(), 2);
        let v: Value = serde_json::from_str(&inits[0]).unwrap();
        assert_eq!(v["runway_start"]["runway"], "13");
        assert_eq!(v["engine_status"]["all_engines"]["running"], true);
        assert!(
            sim.speeds.lock().unwrap().is_empty(),
            "the case's 2x is never flown on the ground (the guard)"
        );
        // A flying loop refuses the run.
        sim.flying.store(true, std::sync::atomic::Ordering::SeqCst);
        assert!(preflight(&sim)
            .await
            .unwrap_err()
            .to_string()
            .contains("AUTOLAND|final"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A case naming another aircraft loads it (an approach: on its runway,
    /// engines running, before the app's own setup) and waits for the app's
    /// detector to have swapped to it; one never taken is a setup failure.
    #[tokio::test]
    async fn a_named_aircraft_is_loaded_and_waited_for_before_the_first_line() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("swap");
        let sim = FakeSim::new(log.clone());
        let sr22 = "Aircraft/Laminar Research/Cirrus SR22/Cirrus SR22.acf";
        let case = |a: &str| {
            parse_spec_with(
                &format!(
                    "cases: [{{name: a, setup: {{aircraft: \"{a}\", approach: {{field: LEMG, runway: 13, final_nm: 10}}}}}}]"
                ),
                &|_| true,
            )
            .unwrap()
            .cases
            .remove(0)
        };
        let mut tail = BlackboxTail::at_end(&log);
        // The loaded aircraft: the approach tool sets it up, nothing posted.
        let same = case("Aircraft/Laminar Research/Boeing 737-800/b738.acf");
        set_up_start(&sim, &mut tail, &same, "", &quick_pace())
            .await
            .unwrap();
        assert!(sim.inits.lock().unwrap().is_empty());
        // Another: loaded on the runway with engines running, then the swap.
        sim.probe_on_swap.store(true, SeqCst);
        set_up_start(&sim, &mut tail, &case(sr22), "", &quick_pace())
            .await
            .unwrap();
        let inits = sim.inits.lock().unwrap().clone();
        assert_eq!(inits.len(), 1);
        let v: Value = serde_json::from_str(&inits[0]).unwrap();
        assert_eq!(v["aircraft"]["path"], sr22);
        assert_eq!(v["runway_start"]["airport_id"], "LEMG");
        assert_eq!(v["engine_status"]["all_engines"]["running"], true);
        // An app that never probes it: named, not flown on the wrong type.
        sim.probe_on_swap.store(false, SeqCst);
        let e = set_up_start(&sim, &mut tail, &case(sr22), "", &quick_pace())
            .await
            .unwrap_err();
        assert!(e.to_string().contains("the app never took"), "{e}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// `setup: {enroute: {at, value}}`: a point on the plan, the plan's
    /// origin and runway kept for a swap; a point that is none is refused
    /// naming the case, and an enroute case types its command.
    #[test]
    fn an_enroute_case_parses_and_refuses_what_is_no_point() {
        let spec = |setup: &str, rest: &str| {
            format!("cases: [{{name: e, setup: {{enroute: {setup}}}{rest}}}]")
        };
        let cmd = ", command: fly the flight plan";
        let one = |setup: &str| parse_spec_with(&spec(setup, cmd), &|_| true);
        let c = one("{at: before_tod_nm, value: 30}").unwrap().cases.remove(0);
        assert_eq!(
            c.setup,
            Setup::Enroute {
                field: FIELD_PLAN_ORIGIN.into(),
                runway: RUNWAY_FROM_PLAN.into(),
                point: EnroutePoint::BeforeTod(30.0),
            }
        );
        assert!(c.uses_plan() && !c.wants_wind());
        assert_eq!(c.first_line(), "fly the flight plan");
        let fix = one("{at: fix, value: welin}").unwrap().cases.remove(0);
        let at = |c: &Case| match &c.setup {
            Setup::Enroute { point, .. } => point.clone(),
            other => panic!("{other:?}"),
        };
        assert_eq!(at(&fix), EnroutePoint::Fix("WELIN".into()));
        let dest = one("{at: before_destination_nm, value: 40}").unwrap().cases.remove(0);
        assert_eq!(at(&dest), EnroutePoint::BeforeDestination(40.0));
        // Fast forward is a want the guard decides, as on any case.
        let fast_case = spec("{at: before_tod_nm, value: 30}", &format!("{cmd}, sim_speed: 4"));
        let fast = parse_spec_with(&fast_case, &|_| true).unwrap();
        assert_eq!(fast.cases[0].sim_speed, Some(4));
        for (bad, why) in [
            (
                "{at: top_of_climb, value: 3}",
                "is not one of fix, before_tod_nm, before_destination_nm",
            ),
            ("{at: before_tod_nm, value: -5}", "whole number of miles"),
            ("{at: before_tod_nm, value: 2.5}", "whole number of miles"),
            ("{at: before_tod_nm, value: WELIN}", "needs a number of miles"),
            ("{at: before_destination_nm, value: 0}", "at least 1"),
            ("{at: fix, value: 30}", "names one fix"),
            ("{at: fix, value: \"WEL IN\"}", "names one fix"),
            ("{at: fix}", "missing field"),
        ] {
            let e = one(bad).unwrap_err().to_string();
            assert!(e.contains(why), "{bad}: {e}");
        }
        let e = parse_spec_with(&spec("{at: fix, value: WELIN}", ""), &|_| true).unwrap_err();
        assert!(e.to_string().contains("needs a command"), "{e}");
        let e = parse_spec_with(
            "cases: [{name: e, setup: {enroute: {at: fix, value: WELIN}, ramp: {field: EGKK}},\
             command: x}]",
            &|_| true,
        )
        .unwrap_err();
        assert!(e.to_string().contains("exactly one of"), "{e}");
    }

    /// The point is checked on the fetched plan: a fix it does not hold, an
    /// end of it, a top of descent its navlog lacks, and the placement
    /// itself (the pilot's EGKK-EGBB OFP) are refused before the run.
    #[test]
    fn an_enroute_point_is_checked_on_the_plan() {
        let v: Value =
            serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/ofp/egkk-egbb-738.trimmed.json"))).unwrap();
        let fp = crate::simbrief::plan_from_ofp(&v).unwrap();
        let facts = PlanFacts::of(&fp);
        assert!(facts.has_tod && facts.fixes.contains(&"WELIN".to_string()));
        let find = |_: &str| None;
        let case = |point: EnroutePoint| Case {
            name: "e".into(),
            setup: Setup::Enroute {
                field: FIELD_PLAN_ORIGIN.into(),
                runway: RUNWAY_FROM_PLAN.into(),
                point,
            },
            aircraft: None,
            command: Some("fly the flight plan".into()),
            steps: Vec::new(),
            deadline_min: None,
            sim_speed: None,
            settle_s: None,
            expect_give_up: None,
            after: None,
            traffic: None,
        };
        let ok = resolve_case(&case(EnroutePoint::BeforeTod(30.0)), &facts, &find).unwrap();
        assert_eq!(
            ok.setup,
            Setup::Enroute {
                field: "EGKK".into(),
                runway: "26L".into(),
                point: EnroutePoint::BeforeTod(30.0)
            }
        );
        let e = |p: EnroutePoint, f: &PlanFacts| {
            resolve_case(&case(p), f, &find).unwrap_err().to_string()
        };
        assert!(e(EnroutePoint::Fix("NOSUCH".into()), &facts).contains("not on the flight plan"));
        assert!(e(EnroutePoint::Fix("EGBB".into()), &facts).contains("an end of the flight plan"));
        let no_tod = PlanFacts { has_tod: false, ..facts.clone() };
        assert!(e(EnroutePoint::BeforeTod(30.0), &no_tod).contains("no top of descent"));
        // A plan without an origin runway still sets up on the loaded type.
        let no_rwy = PlanFacts { origin_rwy: None, ..facts.clone() };
        let r = resolve_case(&case(EnroutePoint::Fix("HEMEL".into())), &no_rwy, &find).unwrap();
        assert!(matches!(r.setup, Setup::Enroute { ref runway, .. } if runway.is_empty()));
        // The placement the app will make, with the 737's cruise.
        let speeds = crate::tools::setup_enroute::Speeds {
            profile_kts: Some(255.0),
            ..Default::default()
        };
        let place = |p: EnroutePoint| crate::tools::setup_enroute::place(&fp, &p, &speeds);
        assert!(place(EnroutePoint::BeforeTod(30.0)).is_ok());
        let low = place(EnroutePoint::BeforeDestination(2.0)).unwrap_err();
        assert!(low.contains("under the 3000"), "{low}");
    }

    #[test]
    fn the_enroute_outcome_is_read_from_the_apps_records() {
        let r = |v: Value| enroute_outcome(&v);
        assert_eq!(r(json!({"kind": "setup", "event": "enroute_init"})), None);
        assert_eq!(r(json!({"kind": "tool", "name": "setup_enroute"})), None);
        assert_eq!(
            r(json!({"kind": "setup", "event": "enroute_result", "outcome": "ok",
                     "engaged": true})),
            Some(Ok(()))
        );
        let no_ap = r(json!({"kind": "setup", "event": "enroute_result", "outcome": "ok",
                             "engaged": false}));
        assert!(no_ap.unwrap().unwrap_err().contains("without the autopilot"));
        let refused = r(json!({"kind": "setup", "event": "enroute_result", "outcome": "refused",
                               "engaged": false, "why": "X-Plane refused it, code 1."}));
        assert_eq!(refused, Some(Err("X-Plane refused it, code 1.".into())));
        let said =
            r(json!({"kind": "setup", "event": "enroute_refused", "why": "No flight plan."}));
        assert_eq!(said, Some(Err("the app refused: No flight plan.".into())));
    }

    /// An enroute case: the app's own setup words, the new flight in the
    /// air waited for, then the case's command and its sortie's verdict. A
    /// refusal is the case's failure, in the app's words, nothing typed
    /// after it. The typed words route to the setup tool on the 737.
    #[tokio::test]
    async fn an_enroute_case_is_set_up_in_the_air_then_resumed() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("enroute");
        let sim = FakeSim::new(log.clone());
        let mut spec = parse_spec_with(
            "cases: [{name: arrival, setup: {enroute: {at: before_tod_nm, value: 30}},\n\
                      command: fly the flight plan}]",
            &|_| true,
        )
        .unwrap();
        spec.cases[0].setup = Setup::Enroute {
            field: "EGKK".into(),
            runway: "26L".into(),
            point: EnroutePoint::BeforeTod(30.0),
        };
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(&sim, &mut tail, &spec, 1, "", &quick_pace(), &mut |_: &Row| {}).await;
        assert!(rows[0].verdict.pass, "{:?}", rows[0].verdict);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec!["put us enroute 30 miles before top of descent", "fly the flight plan"]
        );
        assert!(sim.inits.lock().unwrap().is_empty(), "the app's setup, not a runner post");
        let b738 = bench_profile(None);
        assert_eq!(
            routed("put us enroute 30 miles before top of descent", &b738).as_deref(),
            Some("setup_enroute")
        );
        assert_eq!(routed("fly the flight plan", &b738).as_deref(), Some("full_auto"));
        // Refused: the case fails with the app's words, no command typed.
        sim.typed.lock().unwrap().clear();
        sim.airborne.store(false, SeqCst);
        sim.enroute_refuses.store(true, SeqCst);
        let rows = run_all(&sim, &mut tail, &spec, 1, "", &quick_pace(), &mut |_: &Row| {}).await;
        assert!(!rows[0].verdict.pass);
        let why = &rows[0].verdict.reason;
        assert!(why.contains("the app refused: The plan is for a A320"), "{why}");
        assert_eq!(sim.typed.lock().unwrap().len(), 1);
        // Another aircraft: loaded on the plan's origin runway first.
        sim.enroute_refuses.store(false, SeqCst);
        sim.probe_on_swap.store(true, SeqCst);
        let mut swap = spec.cases[0].clone();
        swap.aircraft = Some("Aircraft/Laminar Research/Cirrus SR22/Cirrus SR22.acf".into());
        set_up_start(&sim, &mut tail, &swap, "", &quick_pace()).await.unwrap();
        let inits = sim.inits.lock().unwrap().clone();
        let v: Value = serde_json::from_str(inits.last().unwrap()).unwrap();
        assert_eq!(v["runway_start"]["airport_id"], "EGKK");
        assert_eq!(v["runway_start"]["runway"], "26L");
        swap.setup = Setup::Enroute {
            field: "EGKK".into(),
            runway: String::new(),
            point: EnroutePoint::BeforeTod(30.0),
        };
        let e = set_up_start(&sim, &mut tail, &swap, "", &quick_pace()).await.unwrap_err();
        assert!(e.to_string().contains("names no runway at EGKK"), "{e}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// LEMG's two runways, verbatim from the Global Airports apt.dat.
    fn lemg_apt(dir: &std::path::Path) -> String {
        let p = dir.join("apt.dat");
        std::fs::write(
            &p,
            "1    52 0 0 LEMG Malaga\n\
             100 45.11 1 1 0.00 1 3 0  12 36.6910753 -4.5078270 0 190 3 1 1 1 30 36.6795648 -4.4805125 0 489 3 0 0 1\n\
             100 45.11 27 27 0.00 1 3 0  13 36.6845573 -4.5126253 0 75 3 1 0 2 31 36.6654027 -4.4858140 0 75 3 1 0 3\n\
             1   1027 0 0 0MU1 Sunderland\n",
        )
        .unwrap();
        p.to_string_lossy().to_string()
    }

    /// #158: the backlog named LEMG 13 with the wind at 280/5; the tower
    /// sent arrivals to 30. `runway: wind` is picked at the setup by the
    /// tower's own rule, from the METAR, else the sim's wind on the field,
    /// and the row says which.
    #[tokio::test]
    async fn runway_wind_is_the_towers_into_wind_end_at_the_setup() {
        let (dir, log) = scratch_log("wind-pick");
        let apt = lemg_apt(&dir);
        let sim = FakeSim::new(log.clone());
        let spec = parse_spec(
            "cases: [{name: final, setup: {approach: {field: LEMG, runway: wind, final_nm: 5}}},\n\
                     {name: start, setup: {runway_start: {field: LEMG, runway: wind, engines_running: true}},\n\
                      command: fly a circuit}]",
        )
        .unwrap();
        assert!(spec.cases.iter().all(Case::wants_wind));
        // The tower's rule on the recorded wind: 30, not 13.
        let runways = crate::nav::find_runways(&apt, "LEMG").unwrap();
        let p = pick_into_wind(&runways, 280.0, 5.0, "METAR").unwrap();
        assert_eq!(p.runway, "30", "{p:?}");
        assert_eq!(p.words(), "runway 30 into 280/5 METAR, headwind 4.8 kt");
        assert_eq!(pick_into_wind(&runways, 130.0, 8.0, "sim").unwrap().runway, "13");
        *sim.metar.lock().unwrap() = Some((280.0, 5.0));
        let mut tail = BlackboxTail::at_end(&log);
        let mut seen = Vec::new();
        let rows = run_all(&sim, &mut tail, &spec, 1, &apt, &quick_pace(), &mut |r: &Row| {
            seen.push(row_line(r))
        })
        .await;
        assert!(rows.iter().all(|r| r.verdict.pass), "{seen:?}");
        assert_eq!(
            sim.typed.lock().unwrap()[0],
            "put us on a 5 mile final to LEMG runway 30"
        );
        let inits = sim.inits.lock().unwrap().clone();
        let v: Value = serde_json::from_str(&inits[0]).unwrap();
        assert_eq!(v["runway_start"]["runway"], "30");
        assert!(
            seen[0].starts_with("PASS final #1 (runway 30 into 280/5 METAR, headwind 4.8 kt): "),
            "{seen:?}"
        );
        // No METAR: the sim's wind at the aircraft on the field (LEMG 13's
        // threshold, 130 at 8 kt) picks 13.
        *sim.metar.lock().unwrap() = None;
        *sim.at.lock().unwrap() = (36.6845, -4.5126, 130.0, 8.0 / MPS_TO_KTS);
        let one = Spec { cases: vec![spec.cases[1].clone()], after: None };
        let rows = run_all(&sim, &mut tail, &one, 1, &apt, &quick_pace(), &mut |_| {}).await;
        assert!(rows[0].verdict.pass, "{:?}", rows[0].verdict);
        assert!(rows[0].verdict.setup.starts_with("runway 13 into 130/8 sim"), "{:?}", rows[0].verdict);
        // Away from the field the aircraft's wind is not the field's: the
        // case fails its setup, naming why, and nothing is flown.
        *sim.at.lock().unwrap() = (51.15, -0.18, 130.0, 4.0);
        let typed = sim.typed.lock().unwrap().len();
        let rows = run_all(&sim, &mut tail, &one, 1, &apt, &quick_pace(), &mut |_| {}).await;
        assert!(!rows[0].verdict.pass);
        assert!(rows[0].verdict.reason.contains("no surface wind for LEMG"), "{:?}", rows[0].verdict);
        assert_eq!(sim.typed.lock().unwrap().len(), typed, "nothing typed");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// #158: a case with no deadline of its own gets the default scaled by
    /// the airframe's approach speed; one that names its deadline keeps it.
    #[tokio::test]
    async fn the_default_deadline_scales_with_the_approach_speed() {
        assert_eq!(default_deadline_min(None), DEFAULT_DEADLINE_MIN);
        assert_eq!(default_deadline_min(Some(145.0)), DEFAULT_DEADLINE_MIN);
        // A faster airframe never gets less than the default.
        assert_eq!(default_deadline_min(Some(154.0)), DEFAULT_DEADLINE_MIN);
        // The SF50's 87 kt: 145/87 of it.
        assert!((default_deadline_min(Some(87.0)) - 20.0 * 145.0 / 87.0).abs() < 1e-9);
        // A 50 kt ultralight is capped.
        assert_eq!(default_deadline_min(Some(50.0)), DEFAULT_DEADLINE_MIN * DEADLINE_SCALE_MAX);
        let spec = parse_spec(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13}}},\n\
                     {name: b, setup: {approach: {field: LEMG, runway: 13}}, deadline_min: 7}]",
        )
        .unwrap();
        assert_eq!(spec.cases[0].deadline_s_for(Some(87.0)), 20.0 * 145.0 / 87.0 * 60.0);
        assert_eq!(spec.cases[1].deadline_s_for(Some(87.0)), 7.0 * 60.0);
        // The driver sizes the case's deadline from the airframe: an endless
        // mission on an 87 kt aircraft is judged at the scaled deadline, and
        // the row says so.
        let (dir, log) = scratch_log("vapp-deadline");
        let sim = FakeSim::new(log.clone());
        *sim.vapp.lock().unwrap() = Some(87.0);
        let mut tail = BlackboxTail::at_end(&log);
        let one = Spec { cases: vec![spec.cases[0].clone()], after: None };
        let rows = run_all(&sim, &mut tail, &one, 1, "", &quick_pace(), &mut |_| {}).await;
        assert_eq!(rows[0].verdict.setup, "deadline 33 min at 87 kt", "{:?}", rows[0].verdict);
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn quick_pace() -> Pace {
        Pace {
            poll: std::time::Duration::from_millis(5),
            init: InitTiming::LIVE,
            still_s: 0.0,
            still_max_s: 1.0,
            settle_scale: 0.0,
            idle_max_s: 1.0,
            quiet_s: 0.02,
            stop_max_s: 0.5,
            crash_clear_s: 0.2,
            swap_max_s: 1.0,
            fast_resume_s: 0.0,
            enroute_max_s: 1.0,
        }
    }

    fn scratch_log(tag: &str) -> (std::path::PathBuf, std::path::PathBuf) {
        let dir = std::env::temp_dir().join(format!("xvl-runner-{tag}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let log = dir.join("blackbox.jsonl");
        std::fs::write(&log, "").unwrap();
        (dir, log)
    }

    /// LEMG 2026-09-28: three cases burned in 2 s each on the last case's
    /// wreck. Now the wreck that outlives a setup stops the run, once.
    #[tokio::test]
    async fn a_wreck_that_outlives_the_setup_stops_the_run_once() {
        let (dir, log) = scratch_log("wreck");
        let sim = FakeSim::new(log.clone());
        sim.crashed.store(true, std::sync::atomic::Ordering::SeqCst);
        sim.endless.store(true, std::sync::atomic::Ordering::SeqCst);
        let spec = parse_spec(
            "cases: [{name: five, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}}}]",
        )
        .unwrap();
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            3,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert_eq!(rows.len(), 1, "the run stops at the first case");
        assert_eq!(rows[0].verdict.reason, STILL_CRASHED);
        assert!(rows[0].verdict.stop_run);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec!["put us on a 5 mile final to LEMG runway 13"],
            "no \"my controls\" over a wreck that was never this case's"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The deadline typed "my controls" at 1,600 ft on a 5 nm final: the
    /// aircraft was handed to nobody. Now the loops keep it until the next
    /// case's setup replaces the flight, and a run that ends pauses the sim.
    #[tokio::test]
    async fn a_deadline_in_the_air_leaves_the_loops_flying_to_the_next_setup() {
        let (dir, log) = scratch_log("deadline");
        let sim = FakeSim::new(log.clone());
        sim.airborne
            .store(true, std::sync::atomic::Ordering::SeqCst);
        sim.endless.store(true, std::sync::atomic::Ordering::SeqCst);
        // 0.005 min: 0.3 s.
        let spec = parse_spec(
            "cases: [{name: five, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}}, deadline_min: 0.005}]",
        )
        .unwrap();
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            2,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert_eq!(rows.len(), 2);
        for r in &rows {
            assert!(r.verdict.reason.starts_with("deadline"), "{:?}", r.verdict);
            assert!(r.verdict.left_flying && !r.verdict.pass);
        }
        let setup = "put us on a 5 mile final to LEMG runway 13";
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec![setup, ABORT_LINE, setup],
            "taken back only to be replaced by the next setup, never after the last"
        );
        assert_eq!(
            *sim.commands.lock().unwrap(),
            vec![PAUSE_ON],
            "the run ends with the aircraft in the air: paused, still flown"
        );
        assert!(sim.flying.load(std::sync::atomic::Ordering::SeqCst));

        // On the ground the deadline takes the aircraft back, and nothing is paused.
        let sim = FakeSim::new(log.clone());
        sim.endless.store(true, std::sync::atomic::Ordering::SeqCst);
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            1,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert!(!rows[0].verdict.left_flying);
        assert_eq!(*sim.typed.lock().unwrap(), vec![setup, ABORT_LINE]);
        assert!(sim.commands.lock().unwrap().is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// #145, LEMG 31 (sortie 1790584715, 2026-09-28): the verdict came at
    /// the taxi_in stage at 1790585617.8 with the aircraft at 16.1 kt, and
    /// the next runway start was posted 0.3 s later. On the way to the
    /// stand the taxi-in stood still with no loop driving from 618.2 to
    /// 620.7 (vacated, then the taxi request), which is not its end.
    #[test]
    fn the_next_setup_waits_for_the_last_aircraft_at_rest() {
        let pace = Pace {
            quiet_s: QUIET_S,
            stop_max_s: STOP_MAX_S,
            idle_max_s: 60.0,
            ..Pace::LIVE
        };
        let t0 = 1790585617.8;
        let mut w = SettleWatch::new(t0, &pace, false);
        // Rolling at 16 kt, the marker idle: never at rest.
        assert_eq!(
            w.tick(t0, Some(false), Some(16.1), Some(true)),
            SettleStep::Wait
        );
        // The vacate-to-request pause: stopped 2.5 s, then driving again.
        assert_eq!(
            w.tick(618.2 + 1790585000.0, Some(false), Some(0.2), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(620.6 + 1790585000.0, Some(false), Some(0.2), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(620.9 + 1790585000.0, Some(true), Some(0.5), Some(true)),
            SettleStep::Wait
        );
        // A loop driving at walking pace, or a failed read, is not at rest.
        let t = 1790585700.0;
        assert_eq!(
            w.tick(t, Some(true), Some(0.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(t + 20.0, None, Some(0.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(t + 40.0, Some(false), None, Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(t + 60.0, Some(false), Some(0.0), None),
            SettleStep::Wait
        );
        // At the stand: at rest, held QUIET_S.
        let s = 1790585960.0;
        assert_eq!(
            w.tick(s, Some(false), Some(0.3), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(s + QUIET_S - 0.5, Some(false), Some(0.3), Some(true)),
            SettleStep::Wait
        );
        match w.tick(s + QUIET_S, Some(false), Some(0.3), Some(true)) {
            SettleStep::Ready(Settle {
                secs,
                end: SettleEnd::AtRest,
            }) => {
                assert!((secs - (s + QUIET_S - t0)).abs() < 1e-6, "{secs}")
            }
            other => panic!("{other:?}"),
        }

        // Past the bound on the ground: "my controls", then at rest.
        let mut w = SettleWatch::new(0.0, &pace, false);
        assert_eq!(
            w.tick(STOP_MAX_S, Some(false), Some(14.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(STOP_MAX_S + 0.5, Some(false), Some(14.0), Some(true)),
            SettleStep::TypeMyControls
        );
        assert_eq!(
            w.tick(STOP_MAX_S + 5.0, Some(false), Some(6.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(STOP_MAX_S + 20.0, Some(false), Some(0.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(
                STOP_MAX_S + 20.0 + QUIET_S,
                Some(false),
                Some(0.0),
                Some(true)
            ),
            SettleStep::Ready(Settle {
                secs: STOP_MAX_S + 20.0 + QUIET_S,
                end: SettleEnd::MyControls
            })
        );
        // ...and one that never stops fails the case, naming the speed.
        let mut w = SettleWatch::new(0.0, &pace, false);
        w.tick(STOP_MAX_S + 0.5, Some(false), Some(14.0), Some(true));
        match w.tick(STOP_MAX_S + 61.0, Some(false), Some(9.0), Some(true)) {
            SettleStep::Fail(why) => assert!(why.contains("still moving at 9.0 kt"), "{why}"),
            other => panic!("{other:?}"),
        }

        // In the air nothing is waited for: never handed to nobody for
        // longer than the take-back before the setup (#132).
        let mut w = SettleWatch::new(0.0, &pace, false);
        assert_eq!(
            w.tick(0.0, Some(true), Some(140.0), Some(false)),
            SettleStep::TypeMyControls
        );
        assert_eq!(
            w.tick(1.0, Some(true), Some(140.0), Some(false)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(2.0, Some(false), Some(140.0), Some(false)),
            SettleStep::Ready(Settle {
                secs: 2.0,
                end: SettleEnd::TakenBack
            })
        );
        // Loops left flying with the wheels unreadable: taken back at once.
        let mut w = SettleWatch::new(0.0, &pace, true);
        assert_eq!(
            w.tick(0.0, Some(true), None, None),
            SettleStep::TypeMyControls
        );
        // Past the bound with the wheels unreadable: no measurement, as flying.
        let mut w = SettleWatch::new(0.0, &pace, false);
        assert_eq!(
            w.tick(STOP_MAX_S + 1.0, Some(true), Some(3.0), None),
            SettleStep::TypeMyControls
        );
        assert_eq!(
            w.tick(STOP_MAX_S + 2.0, Some(false), None, None),
            SettleStep::Ready(Settle {
                secs: STOP_MAX_S + 2.0,
                end: SettleEnd::TakenBack
            })
        );
    }

    fn circuit_spec() -> Spec {
        parse_spec(
            "cases: [{name: circuit, setup: {runway_start: {field: LEMG, runway: 31, engines_running: true}},\n\
              command: fly a circuit}]",
        )
        .unwrap()
    }

    /// #145 on the fake sim: the circuit's verdict leaves the aircraft
    /// rolling on its taxi-in with the marker idle; the next runway start
    /// goes only once it is at rest, and the row says how long that took.
    #[tokio::test]
    async fn a_rolling_taxi_in_is_waited_out_before_the_next_setup() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("taxi-in");
        let sim = FakeSim::new(log.clone());
        sim.roll_after_mission.store(30, SeqCst);
        let mut tail = BlackboxTail::at_end(&log);
        let pace = Pace {
            stop_max_s: 5.0,
            ..quick_pace()
        };
        let rows = run_all(
            &sim,
            &mut tail,
            &circuit_spec(),
            2,
            "",
            &pace,
            &mut |_: &Row| {},
        )
        .await;
        assert_eq!(rows.len(), 2);
        assert!(rows.iter().all(|r| r.verdict.pass), "{:?}", rows[1].verdict);
        assert_eq!(
            sim.setups_while_rolling.load(SeqCst),
            0,
            "no setup under a running taxi-in"
        );
        assert_eq!(sim.inits.lock().unwrap().len(), 2);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec!["fly a circuit", "fly a circuit"],
            "a taxi-in that finishes by itself is never interrupted"
        );
        let settle = rows[1].verdict.settle.expect("the wait is on the row");
        assert_eq!(settle.end, SettleEnd::AtRest);
        assert!(settle.secs > 0.0);
        assert_eq!(
            rows[0].verdict.settle.map(|s| s.end),
            Some(SettleEnd::AtRest),
            "the first case had nothing to wait for"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A taxi-in that never ends by itself (the LEMG 31 drive stood down 1 km
    /// off its line and was taken over by a second one): past the bound on
    /// the ground "my controls" stops it (`hand_back_hold` parks), then the
    /// setup, and the row says so.
    #[tokio::test]
    async fn a_taxi_in_past_the_bound_gets_my_controls_then_the_setup() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("taxi-in-bound");
        let sim = FakeSim::new(log.clone());
        sim.roll_after_mission.store(u32::MAX, SeqCst);
        let mut tail = BlackboxTail::at_end(&log);
        let pace = Pace {
            stop_max_s: 0.1,
            ..quick_pace()
        };
        let mut seen = Vec::new();
        let rows = run_all(
            &sim,
            &mut tail,
            &circuit_spec(),
            2,
            "",
            &pace,
            &mut |r: &Row| seen.push(row_line(r)),
        )
        .await;
        assert_eq!(rows.len(), 2);
        assert_eq!(sim.setups_while_rolling.load(SeqCst), 0);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec!["fly a circuit", ABORT_LINE, "fly a circuit"],
        );
        assert_eq!(
            rows[1].verdict.settle.map(|s| s.end),
            Some(SettleEnd::MyControls)
        );
        assert!(
            seen[1].contains("my controls after 0 s, the last aircraft still moving"),
            "{}",
            seen[1]
        );
        // The run's last taxi-in is left to finish: no setup follows it.
        assert_eq!(sim.rolling_reads.load(SeqCst), u32::MAX);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn after_is_the_cases_else_the_specs_else_park() {
        let s = parse_spec_with(
            "after: stop\n\
             cases: [{name: a, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}}},\n\
                     {name: b, setup: {ramp: {field: LEMG}}, command: fly a circuit,\n\
                      after: park}]",
            &|_| true,
        )
        .unwrap();
        assert_eq!(s.cases[0].after(), After::Stop);
        assert_eq!(s.cases[1].after(), After::Park);
        let s = parse_spec_with(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}}}]",
            &|_| true,
        )
        .unwrap();
        assert_eq!(s.cases[0].after(), After::Park, "silent is today's rule");
        let bad = parse_spec_with(
            "cases: [{name: a, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}},\n\
                      after: taxi}]",
            &|_| true,
        );
        assert!(bad.is_err());
        // The bench specs parse with it, and keep the taxi-in cases parking.
        for (file, parked) in [
            ("lemg-737.yaml", vec![]),
            ("backlog-737.yaml", vec!["737 gate to gate on the SimBrief plan"]),
            (
                "backlog-types.yaml",
                vec![
                    "SR22 LEMG circuit from the ramp",
                    "S-76 LEMG to LXGB on a local plan",
                    "S-76 LEMG runway start into the wind, circuit",
                ],
            ),
        ] {
            let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("bench/sorties")
                .join(file);
            let spec = parse_spec_with(&std::fs::read_to_string(path).unwrap(), &|_| true)
                .unwrap();
            for c in &spec.cases {
                let want = match parked.contains(&c.name.as_str()) {
                    true => After::Park,
                    false => After::Stop,
                };
                assert_eq!(c.after(), want, "{file}: {}", c.name);
            }
        }
    }

    /// `after: stop`: the last verdict came with the aircraft on its taxi-in
    /// (LEMG, 2026-09-28: the SF50 circuit's vacate and taxi-in held the next
    /// setup 306 s). "my controls" at once, again if a loop still drives past
    /// [`STOP_RETYPE_S`], then the quiet hold; in the air, today's take-back.
    #[test]
    fn a_stop_after_the_verdict_takes_the_aircraft_back_on_the_ground() {
        let pace = Pace {
            quiet_s: QUIET_S,
            stop_max_s: STOP_MAX_S,
            idle_max_s: 60.0,
            ..Pace::LIVE
        };
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        assert_eq!(
            w.tick(0.0, Some(true), Some(14.0), Some(true)),
            SettleStep::TypeMyControls
        );
        // Still driving past the retype: once more, and only once.
        assert_eq!(
            w.tick(STOP_RETYPE_S + 0.5, Some(true), Some(12.0), Some(true)),
            SettleStep::TypeMyControls
        );
        assert_eq!(
            w.tick(STOP_RETYPE_S + 1.0, Some(true), Some(10.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(15.0, Some(false), Some(0.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(15.0 + QUIET_S, Some(false), Some(0.0), Some(true)),
            SettleStep::Ready(Settle {
                secs: 15.0 + QUIET_S,
                end: SettleEnd::Stopped
            })
        );
        // Already at rest: nothing typed, the quiet hold as always.
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        assert_eq!(
            w.tick(0.0, Some(false), Some(0.0), Some(true)),
            SettleStep::Wait
        );
        assert_eq!(
            w.tick(QUIET_S, Some(false), Some(0.0), Some(true)),
            SettleStep::Ready(Settle {
                secs: QUIET_S,
                end: SettleEnd::AtRest
            })
        );
        // In the air: taken back right before the setup, as without it.
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        assert_eq!(
            w.tick(0.0, Some(true), Some(140.0), Some(false)),
            SettleStep::TypeMyControls
        );
        assert_eq!(
            w.tick(1.0, Some(false), Some(140.0), Some(false)),
            SettleStep::Ready(Settle {
                secs: 1.0,
                end: SettleEnd::TakenBack
            })
        );
        // Unreadable wheels are no measurement: never stopped on a guess.
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        assert_eq!(w.tick(0.0, Some(true), Some(14.0), None), SettleStep::Wait);
    }

    /// #168, run 20260928-203723Z case 2 (737 LEMG 13): a rejected takeoff
    /// rolled on unbraked at 73 kt, "my controls" at 1790627961.4, the park
    /// brake set at 2007.9, under a knot by 2012.7. The 60 s bound came at
    /// 2021.4 with the aircraft 9 s into its 10 s quiet hold, and the case
    /// failed "still moving at 0 kt". At rest at the bound is settling.
    #[test]
    fn a_stopped_aircraft_at_the_bound_settles() {
        let pace = Pace {
            quiet_s: QUIET_S,
            stop_max_s: STOP_MAX_S,
            idle_max_s: 60.0,
            ..Pace::LIVE
        };
        // The state records' groundspeed (kt), the brake at 2007.9.
        let gs = [
            (1961.4, 72.9),
            (1979.5, 60.3),
            (2006.4, 50.6),
            (2007.9, 50.1),
            (2009.5, 36.1),
            (2011.1, 17.7),
            (2012.7, 0.1),
            (2014.3, 0.0),
            (2040.0, 0.0),
        ];
        let at = |t: f64| {
            let i = gs.windows(2).position(|w| t < w[1].0).unwrap_or(gs.len() - 2);
            let ((t0, g0), (t1, g1)) = (gs[i], gs[i + 1]);
            g0 + (g1 - g0) * ((t - t0) / (t1 - t0)).clamp(0.0, 1.0)
        };
        let t0 = 1790627961.2;
        let mut w = SettleWatch::new(t0, &pace, false).stop_now(true);
        assert_eq!(
            w.tick(t0, Some(false), Some(73.3), Some(true)),
            SettleStep::TypeMyControls
        );
        let mut t = 1961.7;
        let end = loop {
            let step = w.tick(t0 - 1961.2 + t, Some(false), Some(at(t)), Some(true));
            if step != SettleStep::Wait {
                break (t, step);
            }
            t += 0.5;
            assert!(t < 2040.0, "never settled");
        };
        assert!(end.0 > 2021.4 && end.0 < 2024.0, "settled at {}", end.0);
        assert!(
            matches!(end.1, SettleStep::Ready(Settle { end: SettleEnd::Stopped, .. })),
            "{:?}",
            end.1
        );
        // Still rolling at the bound: the reason names the speed.
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        w.tick(0.0, Some(false), Some(73.0), Some(true));
        match w.tick(61.0, Some(false), Some(4.2), Some(true)) {
            SettleStep::Fail(why) => assert!(why.contains("still moving at 4.2 kt"), "{why}"),
            s => panic!("{s:?}"),
        }
        // Not a groundspeed at all: the reason says which read failed.
        let mut w = SettleWatch::new(0.0, &pace, false).stop_now(true);
        w.tick(0.0, Some(false), Some(73.0), Some(true));
        match w.tick(61.0, Some(false), Some(0.0), None) {
            SettleStep::Fail(why) => assert!(why.contains("wheels unreadable"), "{why}"),
            s => panic!("{s:?}"),
        }
    }

    /// The fake sim's taxi-in never ends by itself; with `after: stop` the
    /// runner never waits for the bound: "my controls" after each verdict
    /// (the last one's at the run's end), and the rows say so.
    #[tokio::test]
    async fn after_stop_types_my_controls_at_the_verdict() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("after-stop");
        let sim = FakeSim::new(log.clone());
        sim.roll_after_mission.store(u32::MAX, SeqCst);
        let mut tail = BlackboxTail::at_end(&log);
        let pace = Pace {
            stop_max_s: 30.0,
            ..quick_pace()
        };
        let mut spec = circuit_spec();
        spec.cases[0].after = Some(After::Stop);
        let mut seen = Vec::new();
        let t0 = std::time::Instant::now();
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            2,
            "",
            &pace,
            &mut |r: &Row| seen.push(row_line(r)),
        )
        .await;
        assert!(t0.elapsed().as_secs_f64() < 20.0, "never waited out the bound");
        assert_eq!(rows.len(), 2);
        assert!(rows.iter().all(|r| r.verdict.pass && r.verdict.stop_owed));
        assert_eq!(sim.setups_while_rolling.load(SeqCst), 0);
        assert_eq!(
            *sim.typed.lock().unwrap(),
            vec!["fly a circuit", ABORT_LINE, "fly a circuit", ABORT_LINE],
        );
        assert_eq!(
            rows[1].verdict.settle.map(|s| s.end),
            Some(SettleEnd::Stopped)
        );
        assert!(seen[1].contains("my controls at the last verdict"), "{}", seen[1]);
        assert_eq!(sim.rolling_reads.load(SeqCst), 0, "the last one stopped too");
        let _ = std::fs::remove_dir_all(&dir);
    }

    // ---- Fast forward ------------------------------------------------------------

    #[test]
    fn a_speed_step_parses_and_a_bad_one_is_refused() {
        let s = parse_spec_with(
            "cases: [{name: a, setup: {ramp: {field: LEMG}}, sim_speed: 2, steps: [\n\
               {say: fly the flight plan, when: start, tool: full_auto},\n\
               {sim_speed: 4, when: {phase: cruise}},\n\
               {sim_speed: 1, when: {event: {kind: procedure, event: tod}}}]}]",
            &|_| true,
        )
        .unwrap();
        let c = &s.cases[0];
        assert_eq!(c.sim_speed, Some(2));
        let later = c.later_steps();
        assert_eq!(later.len(), 2);
        assert_eq!(later[0].sim_speed, Some(4));
        assert_eq!(later[0].when, Trigger::Phase("Cruise".into()));
        assert!(!later[0].is_say() && later[0].say.is_empty());
        assert_eq!(later[1].sim_speed, Some(1));
        let mut script = Script::new(later);
        assert_eq!((script.pending(), script.pending_says()), (2, 0));
        script.arm(0.0);
        script.sent(1.0);
        assert_eq!(script.remaining().len(), 1);
        let bad = [
            ("sim_speed: 0", "sim_speed is 1 to 16"),
            ("sim_speed: 17", "sim_speed is 1 to 16"),
            (
                "steps: [{sim_speed: 0, when: {after_s: 1}}]",
                "sim_speed is 1 to 16",
            ),
            (
                "steps: [{sim_speed: 32, when: {after_s: 1}}]",
                "sim_speed is 1 to 16",
            ),
            (
                "steps: [{sim_speed: -4, when: {after_s: 1}}]",
                "sim_speed is 1 to 16",
            ),
            (
                "steps: [{say: x, sim_speed: 4, when: {after_s: 1}}]",
                "not both",
            ),
            (
                "steps: [{sim_speed: 4, when: {after_s: 1}, tool: full_auto}]",
                "routes to no tool",
            ),
            ("steps: [{sim_speed: 4, when: start}]", "never the start"),
            ("steps: [{sim_speed: 4}]", "needs a when"),
            ("steps: [{when: {after_s: 1}}]", "needs a say"),
        ];
        for (tail, want) in bad {
            let text = format!(
                "cases: [{{name: a, setup: {{runway_start: {{field: LEMG, runway: 13}}}}, command: fly a circuit, {tail}}}]"
            );
            let e = parse_spec_with(&text, &|_| true)
                .expect_err(tail)
                .to_string();
            assert!(e.contains(want), "{tail}: {e}");
        }
    }

    #[test]
    fn a_slow_loop_on_the_panel_is_named_the_follower_is_not() {
        assert_eq!(
            slow_loop("0MONITORING|You have control; check autopilot"),
            None
        );
        assert_eq!(
            slow_loop("1FULL AUTO|cruise\nflying a procedure: to PIMOS\ncontrol: copilot flying"),
            None
        );
        for (s, want) in [
            (
                "1AUTOLAND LEMG 13 12.4nm FINAL",
                "AUTOLAND LEMG 13 12.4nm FINAL",
            ),
            ("1FULL AUTO|approach\nautoland LEMG 13", "autoland LEMG 13"),
            (
                "1AUTO CIRCUIT|downwind runway 13",
                "AUTO CIRCUIT|downwind runway 13",
            ),
            (
                "1FULL AUTO|taxi\nauto taxi: 340 m to go",
                "auto taxi: 340 m to go",
            ),
            ("1RECOVERING: STALL|", "RECOVERING: STALL|"),
            ("1ROUTE|cruise", "ROUTE|cruise"),
            ("1ROUTE|cruise\nflying 40 kt, 500 ft", "ROUTE|cruise"),
            ("1FLYING 40 KT, 500 FT|", "FLYING 40 KT, 500 FT|"),
            ("1PUSHBACK|", "PUSHBACK|"),
            ("1TALK-DOWN LSZS 21|", "TALK-DOWN LSZS 21|"),
        ] {
            assert_eq!(slow_loop(s).as_deref(), Some(want), "{s}");
        }
    }

    /// Every reading of the follower's cruise that lets the sim run fast.
    fn cruising() -> FastReads {
        FastReads {
            crashed: Some(false),
            on_ground: Some(false),
            agl_ft: Some(20_000.0),
            servos_on: Some(true),
            status: Some("1FULL AUTO|cruise\nflying a procedure: to PIMOS".into()),
        }
    }

    /// A guard flying 4x in cruise, its sortie begun at t 0 and fast from t 20.
    fn fast_guard() -> SpeedGuard {
        let mut g = SpeedGuard::new(Some(4), FAST_RESUME_S);
        g.feed(&json!({"kind": "sortie", "event": "begin", "sortie": "9-full_auto"}));
        g.feed(&json!({"kind": "state", "fphase": "Cruise", "sortie": "9-full_auto"}));
        assert_eq!(g.tick(0.0, &cruising()), SpeedAct::Keep, "held first");
        assert_eq!(g.tick(9.0, &cruising()), SpeedAct::Keep);
        assert!(matches!(
            g.tick(10.0, &cruising()),
            SpeedAct::Write { to: 4, .. }
        ));
        g.written(4, Some(4.0));
        assert_eq!(g.tick(11.0, &cruising()), SpeedAct::Keep);
        g
    }

    /// The runner's guard: 1x the moment any condition fails, the reason
    /// said; fast again only after they have all held; the latches last the
    /// sortie.
    #[test]
    fn the_guard_drops_to_1x_on_each_condition() {
        let one = |g: &mut SpeedGuard, x: FastReads, want: &str| match g.tick(30.0, &x) {
            SpeedAct::Write { to: 1, reason } => {
                assert!(reason.contains(want), "{reason} vs {want}");
                g.written(1, Some(1.0));
            }
            other => panic!("{want}: {other:?}"),
        };
        let cases: Vec<(FastReads, &str)> = vec![
            (
                FastReads {
                    crashed: Some(true),
                    ..cruising()
                },
                "crashed",
            ),
            (
                FastReads {
                    crashed: None,
                    ..cruising()
                },
                "wreck flag unreadable",
            ),
            (
                FastReads {
                    on_ground: Some(true),
                    ..cruising()
                },
                "on the ground",
            ),
            (
                FastReads {
                    on_ground: None,
                    ..cruising()
                },
                "wheels unreadable",
            ),
            (
                FastReads {
                    agl_ft: Some(4_900.0),
                    ..cruising()
                },
                "4900 ft AGL",
            ),
            (
                FastReads {
                    agl_ft: None,
                    ..cruising()
                },
                "height unreadable",
            ),
            (
                FastReads {
                    servos_on: Some(false),
                    ..cruising()
                },
                "servos off",
            ),
            (
                FastReads {
                    servos_on: None,
                    ..cruising()
                },
                "servos unreadable",
            ),
            (
                FastReads {
                    status: Some("1FULL AUTO|approach\nautoland LEMG 13".into()),
                    ..cruising()
                },
                "autoland LEMG 13 flying",
            ),
            (
                FastReads {
                    status: None,
                    ..cruising()
                },
                "status unreadable",
            ),
        ];
        for (x, want) in cases {
            let mut g = fast_guard();
            one(&mut g, x.clone(), want);
            // Held (said once), then back only after the conditions held.
            assert_eq!(g.tick(31.0, &x), SpeedAct::Keep);
            assert_eq!(g.tick(40.0, &cruising()), SpeedAct::Keep);
            assert!(matches!(
                g.tick(50.0, &cruising()),
                SpeedAct::Write { to: 4, .. }
            ));
        }
        // The phase: an approach or the ground.
        for p in ["Approach", "InitialClimb", "Landing", "GoAround"] {
            let mut g = fast_guard();
            g.feed(&json!({"kind": "flight", "ev": "phase", "from": "Descent", "to": p}));
            one(&mut g, cruising(), &format!("phase {p}"));
        }
        // Latched for the sortie: a give-up, a hand, a pilot's command, an
        // impact; a new sortie clears it.
        for (r, want) in [
            (
                json!({"kind": "full_auto", "stage": "give_up",
                       "detail": {"reason": "ap_not_engaged"}}),
                "give-up (ap_not_engaged)",
            ),
            (
                json!({"kind": "intervention", "what": "yoke_under_ap"}),
                "intervention (yoke_under_ap)",
            ),
            (
                json!({"kind": "tool", "name": "set_heading"}),
                "pilot's command (set_heading)",
            ),
            (json!({"kind": "flight", "ev": "impact"}), "impact"),
        ] {
            let mut g = fast_guard();
            g.feed(&r);
            one(&mut g, cruising(), want);
            assert_eq!(
                g.tick(100.0, &cruising()),
                SpeedAct::Keep,
                "{want}: latched"
            );
            assert_eq!(
                g.tick(200.0, &cruising()),
                SpeedAct::Keep,
                "{want}: latched"
            );
            g.feed(&json!({"kind": "sortie", "event": "begin", "sortie": "10-full_auto"}));
            assert_eq!(g.tick(201.0, &cruising()), SpeedAct::Keep);
            assert!(matches!(
                g.tick(211.0, &cruising()),
                SpeedAct::Write { to: 4, .. }
            ));
        }
        // The sortie's end, and the spec's own 1x.
        let mut g = fast_guard();
        g.feed(&json!({"kind": "sortie_result", "sortie": "9-full_auto", "pass": true}));
        one(&mut g, cruising(), "no sortie flying");
        let mut g = fast_guard();
        g.want(1);
        one(&mut g, cruising(), "the spec wants 1x");
        assert!(!g.needs_reads(), "nothing wanted, nothing owed: no reads");
        // Withheld before it ever ran: said once per reason, never written.
        let mut g = SpeedGuard::new(Some(4), FAST_RESUME_S);
        let ground = FastReads {
            on_ground: Some(true),
            ..cruising()
        };
        assert_eq!(
            g.tick(0.0, &ground),
            SpeedAct::Held {
                reason: "no sortie flying".into()
            }
        );
        assert_eq!(g.tick(1.0, &ground), SpeedAct::Keep);
        // A fast write that does not read back is latched off and restored.
        let mut g = SpeedGuard::new(Some(4), 0.0);
        g.feed(&json!({"kind": "sortie", "event": "begin", "sortie": "s"}));
        g.feed(&json!({"kind": "state", "fphase": "Climb"}));
        assert!(matches!(
            g.tick(0.0, &cruising()),
            SpeedAct::Write { to: 4, .. }
        ));
        g.written(4, Some(1.0));
        assert!(!g.owes_restore());
        assert!(matches!(
            g.tick(1.0, &cruising()),
            SpeedAct::Held { reason } if reason.contains("sim_speed 4 read back 1")
        ));
        let mut g = SpeedGuard::new(Some(4), 0.0);
        g.feed(&json!({"kind": "sortie", "event": "begin", "sortie": "s"}));
        g.feed(&json!({"kind": "state", "fphase": "Descent"}));
        assert!(matches!(
            g.tick(0.0, &cruising()),
            SpeedAct::Write { to: 4, .. }
        ));
        g.written(4, None);
        assert!(g.owes_restore(), "an unread multiplier is restored");
        assert!(matches!(
            g.tick(1.0, &cruising()),
            SpeedAct::Write { to: 1, .. }
        ));
        // The clock: wall seconds fast, and what the sim achieved.
        let mut g = fast_guard();
        g.accrue(11.0, Some(3.8));
        g.accrue(21.0, Some(3.8));
        g.accrue(31.0, None);
        assert!((g.fast_s - 20.0).abs() < 1e-9);
        assert!((g.achieved().unwrap() - 3.8).abs() < 1e-9);
    }

    /// Against a fake sim: a speed step at the cruise runs the sim fast
    /// through the guard, and every way out puts it back to 1x: a sim left
    /// fast before the setup, the case's end, a crash.
    #[tokio::test]
    async fn the_cruise_flies_fast_and_every_end_comes_back_to_1x() {
        use std::sync::atomic::Ordering::SeqCst;
        let (dir, log) = scratch_log("fast");
        let sim = FakeSim::new(log.clone());
        sim.airborne.store(true, SeqCst);
        sim.endless.store(true, SeqCst);
        sim.enroute.store(true, SeqCst);
        // A sim left fast by something else: 1x before the setup.
        *sim.speed.lock().unwrap() = 8.0;
        // 0.01 min: 0.6 s, then the deadline.
        let spec = parse_spec(
            "cases: [{name: cruise, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}},\n\
                      deadline_min: 0.01, steps: [{sim_speed: 4, when: {phase: Cruise}}]}]",
        )
        .unwrap();
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            1,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert_eq!(
            *sim.speeds.lock().unwrap(),
            vec![1.0, 4.0, 1.0],
            "1x before the setup, 4x in the cruise, 1x at the case's end"
        );
        let v = &rows[0].verdict;
        assert!(v.reason.starts_with("deadline"), "{v:?}");
        assert!(v.fast_s > 0.0, "{v:?}");
        assert_eq!(v.fast_x, Some(4.0));
        assert!(
            row_line(&rows[0]).contains("(fast "),
            "{}",
            row_line(&rows[0])
        );

        // A crash while fast: 1x at once, and the case ends crashed.
        let sim = FakeSim::new(log.clone());
        sim.airborne.store(true, SeqCst);
        sim.endless.store(true, SeqCst);
        sim.enroute.store(true, SeqCst);
        sim.crash_when_fast.store(true, SeqCst);
        let spec = parse_spec(
            "cases: [{name: cruise, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}},\n\
                      deadline_min: 0.05, sim_speed: 4}]",
        )
        .unwrap();
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            1,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert!(rows[0].verdict.crashed, "{:?}", rows[0].verdict);
        assert_eq!(*sim.speeds.lock().unwrap(), vec![4.0, 1.0]);
        assert_eq!(*sim.speed.lock().unwrap(), 1.0);

        // Below the floor (or on any failed condition) the case's speed is
        // never written at all.
        let sim = FakeSim::new(log.clone());
        sim.airborne.store(true, SeqCst);
        sim.endless.store(true, SeqCst);
        let spec = parse_spec(
            "cases: [{name: cruise, setup: {approach: {field: LEMG, runway: 13, final_nm: 5}},\n\
                      deadline_min: 0.005, sim_speed: 4}]",
        )
        .unwrap();
        let mut tail = BlackboxTail::at_end(&log);
        let rows = run_all(
            &sim,
            &mut tail,
            &spec,
            1,
            "",
            &quick_pace(),
            &mut |_: &Row| {},
        )
        .await;
        assert_eq!(rows[0].verdict.fast_s, 0.0);
        assert!(sim.speeds.lock().unwrap().is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }
