//! Stage 3 of the command loop: dispatch (undo, the stop and go words,
//! flow and offer replies, the instructor, conversation, the hand-over
//! and the tools). The answers taken on words alone are `answer.rs`.
use super::*;

/// A call's way past its doors: `Skip` this one, `Stop` the turn here.
pub(crate) enum Door {
    Open,
    Skip,
    Stop,
}

impl Session {
    /// EMERGENCY ABORT: "belay that" walks back the last command, first and
    /// before every gate.
    pub(crate) async fn undo(&mut self, u: &mut Utterance) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let profile = &self.profile;
        let registry = &self.registry;
        let reminders = &self.reminders;
        let config = &*u.config;
        let command = &u.command;
        // EMERGENCY ABORT — the very first thing checked, before the confidence,
        // echo, and domain gates and any LLM. "belay that", "undo", "abort",
        // "scratch that", "as you were" restore what the last command changed. It
        // must beat every gate: a panic word that's low-confidence, or glued to an
        // echoed readback, still has to land. Deterministic and instant.
        if is_undo(&command) {
            self.conversation.clear();
            if !mode::crew_actuates() {
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Readback,
                    "Nothing of mine to undo in instructor mode. Your hands.",
                );
                return Line::Spent;
            }
            match self.undo_stack.pop() {
                None => say(&voice, &reporter, ui::LogKind::Warn, "Nothing to undo."),
                Some(snap) => {
                    tracing::info!(undid = %snap.label, "undo");
                    let mut acts: Vec<String> = Vec::new();
                    // Recover non-value actions first (e.g. restart after a shutdown).
                    for (tname, targs) in &snap.inverse {
                        if let Some(tool) = registry.get(tname) {
                            let ctx = ToolCtx {
                                xplane: xplane,
                                config: &config,
                                voice: Some(&voice),
                                status: reporter.status_sink(),
                                reminders: Some(&reminders),
                            };
                            if tool.execute(targs, &ctx).await.is_ok() {
                                acts.push(recovery_label(tname));
                            }
                        }
                    }
                    // Disengage the autopilot before resetting its dials — stop the
                    // aircraft flying the wrong thing first. servos_off_any is clean.
                    if snap.disengage_ap
                        && autoflight::press(xplane, autoflight::Button::Disconnect)
                            .await
                            .is_ok()
                    {
                        acts.push("autopilot disengaged".to_string());
                    }
                    for (dr, label, old, is_int) in &snap.reverts {
                        // The parking brake goes back through its valve-aware
                        // seam and is "restored" only when it READS back so: a
                        // raw ratio write left the valve behind and claimed
                        // success on the write alone (#65).
                        let restored = if dr == tools::PARK_BRAKE {
                            matches!(
                                tools::controls::drive_park_brake(
                                    xplane,
                                    *old > 0.5,
                                    profile.park_brake_valve_set,
                                )
                                .await,
                                Ok(true)
                            )
                        } else if let Some(d) = autoflight::dial_named(dr) {
                            // An autopilot dial goes back through its owner.
                            autoflight::set_dial(xplane, d, *old).await.is_ok()
                        } else if dr == datarefs::FLAP_RATIO {
                            // The flaps, the speedbrake and the gear go back
                            // through their seams: stepped by command where
                            // the airframe's lever ignores a write.
                            tools::flaps::set_handle(xplane, *old).await.is_ok()
                        } else if dr == tools::controls::SPEEDBRAKE {
                            tools::controls::speedbrake_set(xplane, *old).await.is_ok()
                        } else if dr == datarefs::GEAR_HANDLE_DOWN {
                            tools::gear::set_handle(xplane, *old < 0.5).await.is_ok()
                        } else if *is_int {
                            xplane.set_int(dr, old.round() as i64).await.is_ok()
                        } else {
                            xplane.set_float(dr, *old).await.is_ok()
                        };
                        if restored {
                            acts.push(format!("{label} restored"));
                        }
                    }
                    let line = if acts.is_empty() {
                        "I couldn't revert that.".to_string()
                    } else {
                        format!("Belay that, {}.", join_and(&acts))
                    };
                    say(&voice, &reporter, ui::LogKind::Warn, &line);
                }
            }
            return Line::Spent;
        }
        Line::Open
    }

    /// While a push or the taxi governor holds the aircraft, stop and go words
    /// act before the domain gate.
    pub(crate) async fn stop_go_words(&mut self, u: &mut Utterance) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let profile = &self.profile;
        let registry = &self.registry;
        let reminders = &self.reminders;
        let config = &*u.config;
        let command = &u.command;
        // Domain gate, follow-up, and bare-value rewrite apply to SPOKEN input
        // only — phonetic nonsense and STT follow-ups are voice problems. A typed
        // command is taken as-is. Questions are let through too (the LLM decides
        // act-vs-answer), so they aren't rejected as "not a command".
        // While the taxi governor is driving, stop/go words act BEFORE the
        // domain gate — which knows nothing of "go, go, go" or a mangled
        // "stop the aircraft" ("stop the echo", both heard live 2026-07-12,
        // both silently dropped while the copilot held the brakes). Same
        // philosophy as the release path: when we hold a control, ambiguity
        // resolves toward the pilot's authority over it — and on the ground,
        // stopping is always the safe reading.
        // A PUSH holds the aircraft with no governor running, so the branch
        // below could never see it — and a push is exactly where "stop" is
        // most urgent, since the brakes are off for the tug. Cancel the push;
        // its abort path stops the tug and sets the brakes.
        if pushback::ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
            let tl = command.to_ascii_lowercase();
            if tl
                .split_whitespace()
                .any(|w| w.trim_matches(|c: char| !c.is_ascii_alphanumeric()) == "stop")
                && !tl.contains("stop holding")
            {
                // The flag can outlive its push: the native tool latches it
                // and nothing clears it when the tug finishes on its own.
                // Airborne, a push is definitionally over — heal the flag and
                // let the words route normally, because "stop the descent" is
                // not about a tug, and consuming it here would cascade
                // full_auto::abort() into whatever is flying. A failed read
                // counts as on the ground: down there, stopping is always the
                // safe reading.
                let airborne = xplane
                    .get_float(datarefs::ON_GROUND_ANY)
                    .await
                    .is_ok_and(|og| og < 0.5);
                if airborne {
                    tracing::info!("pushback flag stale airborne; cleared, not consumed");
                    pushback::clear();
                } else if pushback::steered() {
                    self.conversation.clear();
                    // Full auto's steered push: its loop consumes ABORT, stops
                    // the tug and sets the brakes itself.
                    tracing::info!("pushback stop word, pre-gate");
                    pushback::stop_word();
                    // And the sequence that armed it. "Stop" during a push
                    // means stop, not "skip the push and start taxiing" —
                    // which is what aborting only the push would have full
                    // auto do next. Only a steered push can be full auto's.
                    full_auto::abort();
                    // Nothing said HERE: the push loop answers once it has
                    // set the parking brake, from the read-back
                    // (`pushback::stop_line`). This line used to claim the
                    // brakes before the loop had even seen the word (#110).
                    return Line::Spent;
                } else {
                    // A NATIVE push (the spoken tool's one-shot command): no
                    // loop is listening, so the stop must reach the tug from
                    // HERE. ABORT alone was a flag nobody read — "stop
                    // pushback", the panel CANCEL's own phrase, was swallowed
                    // by this branch forever while the tug kept pushing and
                    // the copilot claimed brakes it never set (audit,
                    // 2026-08-02).
                    tracing::info!("native pushback stop, pre-gate");
                    self.conversation.clear();
                    let _ = xplane.run_command(tools::ground::CMD_PUSH_STOP).await;
                    let held =
                        pushback::park_after_stop(xplane, profile.park_brake_valve_set).await;
                    pushback::clear();
                    // The line from the read-back, never the attempt (#65).
                    say(
                        &voice,
                        &reporter,
                        ui::LogKind::Readback,
                        pushback::stop_line(held),
                    );
                    // A governor also driving means the flag was stale, not a
                    // push — fall through so "stop" reaches the brakes it was
                    // actually for.
                    if !tools::speedhold::ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
                        return Line::Spent;
                    }
                }
            }
        }
        if tools::speedhold::ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
            let tl = command.to_ascii_lowercase();
            let words: Vec<&str> = tl
                .split_whitespace()
                .map(|w| w.trim_matches(|c: char| !c.is_ascii_alphanumeric() && c != '\''))
                .filter(|w| !w.is_empty())
                .collect();
            let stopish = words.contains(&"stop")
                && !tl.contains("holding")
                && !tl.contains("my throttle")
                && !tl.contains("my power");
            let goish = matches!(
                words.as_slice(),
                ["go"] | ["go", "go"] | ["go", "go", "go"] | ["lets", "go"] | ["let's", "go"]
            ) || tl.contains("keep going")
                || tl.contains("carry on");
            if stopish || goish {
                self.conversation.clear();
                if let Some(tool) = registry.get("hold_speed") {
                    let args = if stopish {
                        serde_json::json!({ "knots": 0 })
                    } else {
                        serde_json::json!({ "resume": true })
                    };
                    tracing::info!(stop = stopish, "governor stop/go word, pre-gate");
                    let ctx = ToolCtx {
                        xplane: xplane,
                        config: &config,
                        voice: Some(&voice),
                        status: reporter.status_sink(),
                        reminders: Some(&reminders),
                    };
                    match tool.execute(&args, &ctx).await {
                        Ok(line) => say(&voice, &reporter, ui::LogKind::Readback, &line),
                        Err(e) => say(
                            &voice,
                            &reporter,
                            ui::LogKind::Warn,
                            &format!("Tried, but: {e:#}"),
                        ),
                    }
                    return Line::Spent;
                }
            }
        }
        Line::Open
    }

    /// The closed replies a running flow or a standing offer owns, the
    /// checklist taught by name and the instructor's lessons (whose "show me"
    /// routes on with its own words).
    pub(crate) async fn flow_replies(&mut self, u: &mut Utterance) -> Line {
        let voice = &self.voice;
        let reporter = &self.reporter;
        let checklists = &self.checklists;
        let config = &*u.config;
        let command = &mut u.command;
        // A checklist flow owns a closed set of short replies while it runs
        // (yes/no at a hazard confirm, next/set/checked/skip at an instructor's
        // item, stop always); a standing proposal ("Climb checklist when you're
        // ready?") owns "go ahead" / "talk me through it" / "not now". Anything
        // outside those words routes on: "flaps ten" mid-checklist is still
        // flaps ten. After the governor and pushback pre-gates, so a "stop"
        // on the ground stops the aircraft before it stops a list.
        if let Some(reply) = flow::take_reply(&command) {
            tracing::info!(?reply, path = "checklist-reply", "command path");
            reporter.log(ui::LogKind::Info, format!("checklist reply: {reply:?}"));
            return Line::Spent;
        }
        if let Some(idx) = flow::proposal() {
            if let Some(answer) = flow::proposal_reply(&command) {
                tracing::info!(?answer, path = "checklist-proposal", "command path");
                let flavour = match answer {
                    flow::ProposalReply::Run => Some(flow::Flavour::Crew),
                    flow::ProposalReply::Instruct => Some(flow::Flavour::Instruct),
                    flow::ProposalReply::Decline => None,
                };
                match flavour {
                    Some(f) => {
                        flow::start(idx, f, (*config).clone(), voice.clone());
                    }
                    None => {
                        flow::decline_proposal();
                        say(
                            &voice,
                            &reporter,
                            ui::LogKind::Readback,
                            "Okay. Say the word when you want it.",
                        );
                    }
                }
                return Line::Spent;
            }
        }
        // A standing offer from the copilot ("Want the arrival brief?") owns
        // the same closed yes/no words as a checklist proposal; anything else
        // routes on. And the brief asked for by name.
        let brief_asked = rapport::arrival_brief_request(&command);
        let offer_answer = rapport::offer().and_then(|o| {
            flow::proposal_reply(&command).map(|a| (o, a != flow::ProposalReply::Decline))
        });
        if brief_asked || offer_answer.is_some() {
            rapport::clear_offer();
            if matches!(offer_answer, Some((_, false))) && !brief_asked {
                tracing::info!(path = "rapport-offer", "declined");
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Readback,
                    readback::one_of(&["Okay.", "No problem. Just ask."]),
                );
                return Line::Spent;
            }
            tracing::info!(path = "arrival-brief", "command path");
            if !config.simbrief_id.trim().is_empty() {
                let _ = ensure_plan(&config, &mut self.fp_cache, PlanFetch::Cached).await;
            }
            let (apt, cifp, fixes) = (
                config.apt_data_path.clone(),
                config.cifp_dir.clone(),
                config.fix_data_path.clone(),
            );
            let line = tokio::task::spawn_blocking(move || {
                rapport::arrival_brief(&apt, &cifp, &fixes, &crate::config::xplane_root())
            })
            .await
            .unwrap_or_else(|_| "I couldn't put the arrival brief together.".to_string());
            self.conversation.observe(&command, &line, Instant::now());
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // "Talk me through the descent checklist": the instructor flavour, by
        // name. It never writes; it says each item and watches you do it.
        if let Some(request) = flow::instruct_request(&command) {
            tracing::info!(path = "checklist-instruct", "command path");
            match flow::find_titled(&checklists, &request) {
                Ok(idx) => {
                    if !flow::start(
                        idx,
                        flow::Flavour::Instruct,
                        (*config).clone(),
                        voice.clone(),
                    ) {
                        say(
                            &voice,
                            &reporter,
                            ui::LogKind::Warn,
                            "A checklist is already running. Say stop the checklist to end it.",
                        );
                    }
                }
                Err(choices) => {
                    let msg = if choices.is_empty() {
                        "I have no checklists for this aircraft.".to_string()
                    } else {
                        format!(
                            "Which checklist? I have: {}.",
                            choices.join(", ").to_lowercase()
                        )
                    };
                    say(&voice, &reporter, ui::LogKind::Readback, &msg);
                }
            }
            return Line::Spent;
        }
        // The instructor's lessons: "teach me steep turns", "begin", "how am
        // I doing", "end the lesson". Closed vocabulary, and only in
        // instructor mode, where nothing of ours can touch a control while
        // the pilot flies the task.
        if let Some(req) = instructor::request(&command) {
            tracing::info!(?req, path = "instructor", "command path");
            let line = if mode::instructing() {
                instructor::handle(req, instructor::clock_s())
            } else {
                "Lessons run in instructor mode, where I keep my hands off. \
                 Flip the mode in Settings and ask again."
                    .to_string()
            };
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            // "Show me": the demonstration's own request words route on to
            // the tool door, where the open demonstration lets exactly that
            // tool through (`mode::tool_may_actuate`).
            match instructor::take_demo_command() {
                Some(words) => *command = words.to_string(),
                None => return Line::Spent,
            }
        }
        Line::Open
    }

    /// Read-only conversation, before the spoken domain gate.
    pub(crate) async fn converse(&mut self, u: &mut Utterance) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let profile = &self.profile;
        let checklists = &self.checklists;
        let command = &u.command;
        if let Some(response) = self.conversation.respond(&command, Instant::now()) {
            let line = match response {
                conversation::Response::Say(line) => line,
                conversation::Response::Checklist(request) => preview_checklist(
                    &request,
                    &checklists,
                    &mut self.conversation,
                    Instant::now(),
                ),
                conversation::Response::ReadMany(queries) => {
                    let answers = futures::future::join_all(
                        queries
                            .iter()
                            .map(|query| conversation_read(query, xplane, &profile)),
                    )
                    .await;
                    let lines: Vec<String> = queries
                        .iter()
                        .zip(answers)
                        .map(|(query, answer)| {
                            answer.unwrap_or_else(|| format!("No simulator reading for: {query}."))
                        })
                        .collect();
                    let line = lines.join(" ");
                    self.conversation
                        .observe_many(queries, lines, Instant::now());
                    line
                }
                conversation::Response::Status => {
                    conversation::status_answer(&auto_status::active_managers())
                }
                conversation::Response::FlightStatus => flight::published()
                    .map(|s| flight::status_line(&s))
                    .unwrap_or_else(|| "I don't have the flight picture yet.".to_string()),
                conversation::Response::Reference(query) => {
                    match reference_limits(&profile, query) {
                        Some(line) => {
                            self.conversation.reference(
                                &profile.display_name,
                                &line,
                                Instant::now(),
                            );
                            line
                        }
                        None => {
                            self.conversation.clear_topic();
                            "That reference isn't available in this aircraft's profile.".into()
                        }
                    }
                }
                conversation::Response::Read(query) => {
                    // Explicit rechecks use REST rather than an un-timestamped
                    // pushed cache entry that might predate a disconnected socket.
                    match conversation_read(query, xplane, &profile).await {
                        Some(line) => {
                            self.conversation.observe(query, &line, Instant::now());
                            line
                        }
                        None => {
                            self.conversation.clear_topic();
                            "I couldn't get that reading from the simulator.".into()
                        }
                    }
                }
            };
            tracing::info!(path = "conversation", "command path");
            blackbox::record(
                "conversation",
                serde_json::json!({
                    "utterance": &command, "reply": &line,
                }),
            );
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        Line::Open
    }

    /// "You have control": the hand-over resolved against the flight model into
    /// ordinary calls. `None` is a line spent.
    pub(crate) async fn hand_over(
        &mut self,
        calls: Vec<tools::ToolCall>,
    ) -> Option<Vec<tools::ToolCall>> {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let registry = &self.registry;
        // "You have control": the crew hand-over, resolved against the flight
        // model into what the flight needs now (`handover::decide`), then run
        // as ordinary calls so every gate and readback still applies.
        let calls = if calls.first().is_some_and(|c| c.name == grammar::TAKE_OVER) {
            let (bank, pitch) = tokio::join!(
                xplane.get_float(datarefs::BANK_DEG),
                xplane.get_float(datarefs::PITCH_DEG),
            );
            let hand_live = handover::Live {
                upset: matches!((bank, pitch), (Ok(b), Ok(p)) if handover::upset(b, p)),
                running: monitor::flying_loop(),
                // The recovery is withheld exactly where a helicopter is.
                rotorcraft: registry.get("recover").is_none(),
            };
            let status = flight::published();
            let decision = handover::decide(status.as_ref(), &hand_live);
            let (took, why) = match &decision {
                handover::Handover::Take { why, .. } => (true, *why),
                handover::Handover::Decline { why, .. } => (false, *why),
            };
            blackbox::record(
                "handover",
                serde_json::json!({
                    "event": "decide", "took": took, "why": why,
                    "phase": status.as_ref().map(|s| format!("{:?}", s.phase)),
                    "upset": hand_live.upset, "running": hand_live.running,
                }),
            );
            match decision {
                handover::Handover::Decline { line, .. } => {
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    return None;
                }
                handover::Handover::Take { calls, lead, why } => {
                    flight::note_crew_takeover(why);
                    if let Some(lead) = lead {
                        say(&voice, &reporter, ui::LogKind::Readback, lead);
                    }
                    calls
                        .into_iter()
                        .map(|(name, arguments)| tools::ToolCall {
                            name: name.to_string(),
                            arguments,
                        })
                        .collect()
                }
            }
        } else {
            calls
        };
        Some(calls)
    }

    /// The calls run in order, each through its doors, its confirmation and
    /// its execution; what they did is gathered into the turn.
    pub(crate) async fn run_calls(
        &mut self,
        u: &mut Utterance,
        calls: Vec<tools::ToolCall>,
    ) -> TurnRecord {
        let mut turn = TurnRecord::default();
        for mut call in calls {
            match self.call_door(u, &mut call) {
                Door::Skip => continue,
                Door::Stop => break,
                Door::Open => {}
            }
            let proceed = self.confirm_call(u, &mut call).await;
            if !proceed {
                continue;
            }
            self.execute_call(u, &call, &mut turn).await;
        }
        turn
    }

    /// The doors before any confirmation: a hand-over's fallback, the
    /// power-up remap, the instructor's folded arms, the hover's own modes
    /// and a checklist run as a flow.
    pub(crate) fn call_door(&mut self, u: &mut Utterance, call: &mut tools::ToolCall) -> Door {
        let voice = &self.voice;
        let reporter = &self.reporter;
        let registry = &self.registry;
        let checklists = &self.checklists;
        let config = &*u.config;
        // A hand-over's fallback: only if the calls before it left the
        // aircraft to nobody (a refused resume).
        if let Some(args) = call.arguments.as_object_mut() {
            if args.remove(handover::IF_IDLE).is_some() {
                if monitor::flying_loop().is_some() {
                    return Door::Skip;
                }
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Readback,
                    "Then I'll hold the level. I have control.",
                );
            }
        }
        // On aircraft with a procedural power-up, "all electrical on" must run the
        // step-by-step procedure, not set_electrical's one-shot "all" (which would
        // skip the APU start and the callouts). Remap it here so it's caught
        // whether grammar or the LLM produced it. Only the *on* direction: "electrics
        // off" must stay set_electrical (the remap once turned it into power_up,
        // powering the aircraft back up on a shutdown request).
        if wants_procedural_power_up(&call.name, &call.arguments)
            && registry.get("power_up").is_some()
        {
            call.name = "power_up".to_string();
            call.arguments = serde_json::json!({});
        }
        tracing::info!(tool = %call.name, args = %call.arguments, "tool selected");
        // The pilot asked the crew to act: a sortie with one of these in
        // it was attended, whatever the yield latches saw ("set power
        // 100%" mid full auto left no record but a readback line).
        blackbox::record(
            "tool",
            serde_json::json!({ "name": call.name, "args": call.arguments }),
        );
        reporter.log(
            ui::LogKind::Tool,
            format!("{} {}", call.name, call.arguments),
        );

        // Instructor mode: nothing of ours actuates. A checklist runs in
        // its Instruct flavour (flow::start decides that itself); an
        // autopilot-shaped call during a lesson becomes the lesson's
        // target; everything else is refused with the action named, so
        // the pilot does it. One door, before any confirm prompt. The
        // instructor's own demonstration ("show me") is the one tool
        // let through, once.
        if !mode::tool_may_actuate(&call.name, &call.arguments) {
            if let Some(i) = checklists.iter().position(|c| c.id == call.name) {
                if !flow::start(i, flow::Flavour::Instruct, (*config).clone(), voice.clone()) {
                    say(
                        &voice,
                        &reporter,
                        ui::LogKind::Warn,
                        "A checklist is already running. Say stop the checklist to end it.",
                    );
                }
                return Door::Stop;
            }
            let line = instructor::take_target(&call.name, &call.arguments).unwrap_or_else(|| {
                let label = callout_label(&call.name, &call.arguments);
                instructor::refusal(&label, !self.instructor_noted)
            });
            self.instructor_noted = true;
            blackbox::record(
                "instructor",
                serde_json::json!({ "event": "refused", "tool": call.name, "args": call.arguments }),
            );
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Door::Stop;
        }

        // A helicopter's hover owns the autopilot's modes: a heading
        // selected mid-hover replaced the hover mode and flew the S-76
        // forward out of it (2026-09-22). One door, before any confirm.
        // ...and a speed, which changes the speed it is flying.
        if call.name == "set_speed" {
            if let Some(line) = call
                .arguments
                .get("knots")
                .and_then(|k| k.as_f64())
                .and_then(hover::depart_speed)
            {
                blackbox::record(
                    "hover",
                    serde_json::json!({ "event": "depart_speed_asked", "args": call.arguments }),
                );
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                return Door::Stop;
            }
        }
        // ...except a heading during a departure, which turns the
        // track the hover mode is flying.
        if call.name == "set_heading" {
            if let Some(line) = call
                .arguments
                .get("heading")
                .and_then(|h| h.as_f64())
                .and_then(hover::depart_turn)
            {
                blackbox::record(
                    "hover",
                    serde_json::json!({ "event": "depart_turn_asked", "args": call.arguments }),
                );
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                return Door::Stop;
            }
        }
        if let Some(line) = hover::blocks(&call.name) {
            blackbox::record(
                "hover",
                serde_json::json!({ "event": "blocked", "tool": call.name }),
            );
            say(&voice, &reporter, ui::LogKind::Readback, line);
            return Door::Stop;
        }

        // A checklist isn't a one-shot tool — the copilot runs it as a FLOW
        // on its own thread (`flow::start`, Crew flavour): reads each item,
        // does the ones it can, calls out the pilot's, and takes its
        // yes/no/stop replies from this loop. A checklist is a whole phase,
        // run only on demand — so it's never compounded: start this one and
        // ignore any other calls the LLM returned in the same turn (guards
        // against chaining through several checklists at once).
        if let Some(i) = checklists.iter().position(|c| c.id == call.name) {
            if !flow::start(i, flow::Flavour::Crew, (*config).clone(), voice.clone()) {
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Warn,
                    "A checklist is already running. Say stop the checklist to end it.",
                );
            }
            return Door::Stop;
        }
        Door::Open
    }

    /// The confirmation gate with in-line correction; `true` proceeds.
    pub(crate) async fn confirm_call(
        &mut self,
        u: &mut Utterance,
        call: &mut tools::ToolCall,
    ) -> bool {
        let command = &u.command;
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let profile = &self.profile;
        let registry = &self.registry;
        let stt = &self.stt;
        let ptt = &self.ptt;
        let ptt_mic = &self.ptt_mic;
        let config = &*u.config;
        // Confirmation gate with in-line correction ("negative, 500"). Applies
        // per call, so each safety-critical action in a compound command is
        // confirmed on its own.
        let proceed = 'confirm: loop {
            let Some(tool) = registry.get(&call.name) else {
                tracing::warn!("grammar picked a tool the registry lacks: {}", call.name);
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Warn,
                    &router::unregistered_tool_line(&profile, &call.name),
                );
                break 'confirm false;
            };
            // Stopped on the ground: a shutdown here is the end of a
            // flight, not a hazard, so it is not confirmed (the pilot
            // asked, 2026-09-22: "Confirm, shut both engines down?" at
            // every stand). A failed read is not stopped.
            let parked_shutdown =
                call.name == "stop_engines" && sim::ground::read_stopped_on_ground(xplane).await;
            // An all-off asks only with an engine turning on the ground:
            // cold it is the normal shutdown, airborne the tool refuses
            // (issue #66).
            let electrics_clear = call.name == "set_electrical"
                && tools::electrical::is_all_off(&call.arguments)
                && tools::electrical::all_off_gate(xplane).await
                    != tools::electrical::AllOffGate::Confirm;
            let intent = if config.confirm_critical && !parked_shutdown && !electrics_clear {
                tool.confirm_prompt(&call.arguments)
            } else {
                None
            };
            // A command heard at marginal confidence is read back for a yes/no
            // even when the tool isn't safety-critical — this reuses the gate to
            // recover a shaky capture ("Confirm, I heard gear up?") rather than
            // making the pilot repeat the whole thing.
            let intent =
                intent.or_else(|| u.force_confirm_low.then(|| format!("I heard {command}")));
            let Some(intent) = intent else {
                break 'confirm true;
            };

            say(
                &voice,
                &reporter,
                ui::LogKind::Confirm,
                &format!("Confirm, {intent}?"),
            );
            reporter.status(ui::Phase::Listening);
            let Some((reply, _)) = listen_once(&mut self.listener, &ptt_mic, &stt, &ptt, None)
            else {
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Warn,
                    "Cancelled, no confirmation heard.",
                );
                break 'confirm false;
            };
            if is_affirmative(&reply.text) {
                reporter.log(ui::LogKind::Ok, format!("confirmed: \"{}\"", reply.text));
                break 'confirm true;
            }
            match correction_text(&reply.text) {
                Some(correction) => {
                    reporter.log(ui::LogKind::Info, format!("correction: \"{correction}\""));
                    reporter.status(ui::Phase::Thinking);
                    // The correction re-parses through the grammar with the
                    // original words in front ("flaps ten" + "twenty" reads
                    // as flaps twenty); nothing outside the grammar is tried.
                    match grammar::try_parse(
                        &format!("{command}, {correction}"),
                        &registry,
                        &profile,
                    ) {
                        Some(mut cs) if !cs.is_empty() => {
                            *call = cs.remove(0);
                            // The correction WAS the pilot's clear intent, so the
                            // re-picked tool no longer needs the marginal read-back
                            // (a safety-critical one still self-confirms).
                            u.force_confirm_low = false;
                            reporter.log(
                                ui::LogKind::Tool,
                                format!("{} {}", call.name, call.arguments),
                            );
                            continue 'confirm;
                        }
                        _ => {
                            say(&voice, &reporter, ui::LogKind::Warn, "Cancelled.");
                            break 'confirm false;
                        }
                    }
                }
                None => {
                    say(&voice, &reporter, ui::LogKind::Warn, "Cancelled.");
                    break 'confirm false;
                }
            }
        };
        proceed
    }

    /// The curated tool executed (writes its dataref, reads back), its undo
    /// snapshot taken, the camera's glance and the readback spoken.
    pub(crate) async fn execute_call(
        &mut self,
        u: &mut Utterance,
        call: &tools::ToolCall,
        turn: &mut TurnRecord,
    ) {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let live = &self.live;
        let profile = &self.profile;
        let registry = &self.registry;
        let reminders = &self.reminders;
        let config = &*u.config;
        // Execute the curated tool (writes the dataref + reads back).
        reporter.status(ui::Phase::Thinking);
        let Some(tool) = registry.get(&call.name) else {
            return;
        };
        // Snapshot the datarefs this tool is about to change, so "belay that"
        // can put them back. Captured before the write; stored only if the write
        // succeeds. Tools with no reversible dataref (engine start, pushback)
        // contribute nothing and simply can't be undone.
        let mut pre: Vec<(String, String, f64, bool)> = Vec::new();
        for (dr, label, is_int) in undo_targets(&call.name) {
            // A lever the airframe steps by command is read where the
            // lever is (the same seam that will put it back).
            let read = if dr == datarefs::FLAP_RATIO {
                tools::flaps::read_handle(xplane).await
            } else if dr == tools::controls::SPEEDBRAKE {
                tools::controls::speedbrake_read(xplane).await
            } else {
                xplane.get_float(&dr).await
            };
            if let Ok(v) = read {
                pre.push((dr, label.to_string(), v, is_int));
            }
        }
        // If this command can switch the autopilot on and it's off right now,
        // undo should switch it back off (not leave it flying the reset dial).
        let ap_was_off = engages_autopilot(&call.name)
            && matches!(crate::autoflight::read_status(xplane, datarefs::SERVOS_ON).await, Ok(v) if v <= 0.5);
        // Snap the camera to the panel for this action, if the airframe has a
        // saved view for it — so you watch the switch move instead of hunting for
        // it. Remember the current view first (to restore after the glance), then
        // select the 3D cockpit and write the saved head position, then settle a
        // beat BEFORE the action so your eye lands on the panel as the switch
        // moves (not while the camera is still travelling). All best-effort.
        // The panel's "camera snap" setting; OFF unless explicitly enabled —
        // moving the pilot's view proved disorienting, so the callout toast
        // carries the information and the glance is opt-in via Settings.
        let snap_enabled = live.get(DR_OPT_SNAP).is_some_and(|v| v > 0.5);
        // Where the control sits relative to the final (blended) view — the
        // rotation REMAINDER — so the plugin can place the callout toast over
        // it on screen instead of dead-centre.
        let mut callout_off: Option<(f64, f64)> = None;
        let restore_view: Option<[f64; 6]> =
            if let Some(v) = profile.view_map.get(&call.name).filter(|_| snap_enabled) {
                let mut cur = [0.0f64; 6];
                let mut have_cur = true;
                for (slot, dref) in cur.iter_mut().zip(HEAD_DREFS) {
                    match xplane.get_float(dref).await {
                        Ok(read) => *slot = read,
                        Err(_) => have_cur = false,
                    }
                }
                let _ = xplane.run_command("sim/view/3d_cockpit_cmnd_look").await;
                // Glance, don't teleport: move only `view_blend` of the way from the
                // current pose toward the saved one (1.0 = the captured close-up,
                // 0.5 = turn halfway toward the panel without zooming in).
                let blend = profile.view_blend.clamp(0.0, 1.0);
                let saved = [v.x, v.y, v.z, v.psi, v.the, v.phi];
                let mut applied = saved;
                if have_cur && blend < 1.0 {
                    for i in 0..6 {
                        applied[i] = if i < 3 {
                            cur[i] + (saved[i] - cur[i]) * blend
                        } else {
                            lerp_angle_deg(cur[i], saved[i], blend)
                        };
                    }
                }
                for (dref, val) in HEAD_DREFS.into_iter().zip(applied) {
                    let _ = xplane.set_float(dref, val).await;
                }
                // + psi remainder = the control is right of centre; + the = above.
                callout_off = Some((
                    crate::angles::turn_deg(applied[3], saved[3]),
                    saved[4] - applied[4],
                ));
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
                have_cur.then_some(cur)
            } else {
                None
            };
        // On-screen callout: flash WHAT is being changed over the cockpit (the
        // plugin draws it centered for a couple of seconds), synced with the
        // camera glance so your eye lands on the right control.
        if let Some(sink) = reporter.status_sink() {
            let label = callout_label(&call.name, &call.arguments);
            // "nx,ny|" aims the toast toward the control (fractions of the
            // half-screen, from the glance remainder; ~30°/25° of view ≈ edge).
            let text = match callout_off {
                Some((dpsi, dthe)) => format!(
                    "{:.2},{:.2}|{label}",
                    (dpsi / 30.0).clamp(-1.0, 1.0),
                    (dthe / 25.0).clamp(-1.0, 1.0)
                ),
                None => label,
            };
            sink.callout(&text);
        }
        let ctx = ToolCtx {
            xplane: xplane,
            config: &config,
            voice: Some(&voice),
            status: reporter.status_sink(),
            reminders: Some(&reminders),
        };
        // Execute, retrying once on error: a single transient blip (a dropped
        // Web API request, including the autopilot-engage commands the client
        // doesn't itself retry) shouldn't fail an action. Every tool is safe to
        // re-run — value writes are idempotent and the toggle tools read state
        // before toggling — so a retry can't double-apply.
        let (readback, ok) = match tool.execute(&call.arguments, &ctx).await {
            Ok(line) => (line, true),
            Err(first) => {
                tracing::warn!("tool {} failed: {first:#}; retrying once", call.name);
                match tool.execute(&call.arguments, &ctx).await {
                    Ok(line) => (line, true),
                    Err(e) => {
                        tracing::error!("tool {} failed on retry: {e:#}", call.name);
                        ("Something went wrong actioning that.".to_string(), false)
                    }
                }
            }
        };
        turn.failed |= !ok;
        // Gather this action's recovery into the turn (pushed once after the loop).
        if ok {
            // Anchor for back-references ("a bit more", "turn it off"):
            // what was actually executed, exactly as called.
            self.dialogue.record(&call.name, &call.arguments);
            turn.reverts.extend(pre);
            turn.disengage_ap |= ap_was_off;
            // Actions with no value to revert get an explicit inverse command.
            if call.name == "stop_engine" {
                turn.inverse
                    .push(("start_engine".to_string(), serde_json::json!({})));
            }
        }
        // Vary the acknowledgment on affirmative readbacks (bare callout most
        // of the time, else an intercom-natural ack, never the same one
        // twice running); the canonical readback is what's recorded for
        // history context. Heads-down workload proxy: on the ground and
        // rolling fast (takeoff/landing roll) clips the crew to bare callouts.
        let busy = live.get(datarefs::ON_GROUND_ANY).is_some_and(|v| v > 0.5)
            && live.get(datarefs::AIRSPEED_KTS).is_some_and(|v| v > 40.0);
        say(
            &voice,
            &reporter,
            ui::LogKind::Readback,
            &readback::vary_ack(&readback, busy),
        );
        // Glance done — return the camera to where it was before the snap (the
        // readback just played, so you've seen the switch move).
        if let Some(head) = restore_view {
            for (dref, val) in HEAD_DREFS.into_iter().zip(head) {
                let _ = xplane.set_float(dref, val).await;
            }
        }
        turn.readbacks.push(readback);
    }
}
