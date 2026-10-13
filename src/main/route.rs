//! Stage 2 of the command loop: route the line (radio or crew, the
//! confidence, echo and domain gates, the rewrites) and resolve its
//! words to calls.
use super::*;

impl Session {
    /// A keyed call from wreckage, a replay, a paused sim or a dead radio
    /// goes nowhere; the copilot says why.
    pub(crate) fn radio_gates(&mut self, u: &mut Utterance) -> Line {
        let voice = &self.voice;
        let reporter = &self.reporter;
        let from_radio = u.from_radio;
        // Wreckage transmits nothing: the avionics bus stays hot on the crash
        // screen, so the power gate below can't catch it — a mayday from the
        // crashed airframe was answered with "hold position if able"
        // (2026-07-13). The copilot is in the wreck too; they answer on the
        // intercom, and the world stays silent until the flight resets.
        if from_radio && voice.crashed() {
            say(
                &voice,
                &reporter,
                ui::LogKind::Warn,
                "Nobody's reading us. The airplane's wrecked, that radio is gone.",
            );
            return Line::Spent;
        }

        // A replay transmits nothing: the world on screen is a recording, so a
        // keyed call can't reach a controller — the FSM must not move off
        // replayed positions (a replayed crash landing earned a compliance
        // berate, 2026-07-14). The copilot is still on the intercom.
        if from_radio && voice.replay() {
            say(
                &voice,
                &reporter,
                ui::LogKind::Warn,
                "We're in replay, nobody's on the air. Go back to live and I'll key it.",
            );
            return Line::Spent;
        }

        // Paused: the flight is real but the world isn't turning — a call
        // keyed into a frozen sim must not move the tower FSM (the reply
        // would land on positions that aren't advancing). The copilot is
        // crew on the intercom and still answers.
        if from_radio && voice.paused() {
            say(
                &voice,
                &reporter,
                ui::LogKind::Warn,
                "Sim's paused, the radio call can wait. Unpause and I'll key it.",
            );
            return Line::Spent;
        }

        // Dead transmitter: a keyed transmission needs a powered radio — with
        // the avionics master off the panel can't transmit any more than it can
        // receive, so the tower never hears you (a cold-and-dark SR22 raised
        // Gibraltar and got a squawk, 2026-07-12). The COPILOT is crew on the
        // intercom, not radio, so they're the one who tells you why it's quiet.
        // Fail open like the receive gate: `radio_powered` is false only on a
        // confident avionics-off read. Before the confidence gate, so a garbled
        // dead-radio press can't earn the tower's "say again" either.
        if from_radio && !voice.radio_powered() {
            say(
                &voice,
                &reporter,
                ui::LogKind::Warn,
                "Radios are dead — we're not transmitting. Check the avionics master.",
            );
            return Line::Spent;
        }
        Line::Open
    }

    /// The confidence and echo gates (spoken input only): a garbled
    /// transmission gets the tower's "say again", a marginal command is
    /// confirmed, the copilot's own readback is dropped.
    pub(crate) async fn confidence_gates(&mut self, u: &mut Utterance) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let profile = &self.profile;
        let registry = &self.registry;
        let explicit = self.explicit;
        let config = &*u.config;
        let command = &u.command;
        let from_radio = u.from_radio;
        let spoken_confidence = u.spoken_confidence;
        // Confidence + echo gates (spoken input only) — now that abort has had its
        // chance. Confidence: drop what whisper wasn't sure of. Echo: drop the
        // copilot's own readback bleeding back through the speakers, so it can't
        // re-trigger itself. Both bypassed for typed input.
        if let Some(confidence) = spoken_confidence {
            if confidence < config.stt_min_confidence {
                // A garbled TRANSMISSION gets the tower's "say again" (a real
                // controller answers garble; the crew never heard it at all).
                if from_radio {
                    if let Some(st) = self.atc_station.as_mut() {
                        if let Some(who) = st.tuned_speaker(xplane.view()).await {
                            blackbox::record(
                                "atc",
                                serde_json::json!({
                                    "route": "keyed", "outcome": "garbled",
                                    "transcript": command, "confidence": confidence,
                                }),
                            );
                            let line = "Station calling, say again.";
                            reporter.log(ui::LogKind::Readback, format!("{}: {line}", who.tag()));
                            voice.say_station(Priority::Readback, who, line);
                            return Line::Spent;
                        }
                    }
                    reporter.log(
                        ui::LogKind::Info,
                        "garbled transmission, nobody on frequency",
                    );
                    return Line::Spent;
                }
                // Non-radio: classify the marginal capture. "Deliberate" means a
                // PTT/wake command with the crew actually active (ATC-only has no
                // crew to answer). The grammar match (not the LLM) is required so a
                // marginal reading is never fed to the model to guess at.
                let maps = grammar::try_parse(&grammar::normalize(&command), &registry, &profile)
                    .is_some();
                let deliberate = explicit && mode::copilot_enabled();
                match classify_confidence(
                    confidence,
                    config.stt_min_confidence,
                    config.stt_confirm_floor,
                    deliberate,
                    maps,
                ) {
                    // Marginal but a recognised, deliberate command: read it back for
                    // a yes/no (the confirm loop below) rather than dropping it — a
                    // shaky capture costs one word, not the whole command again.
                    ConfidenceAction::Confirm => {
                        u.force_confirm_low = true;
                        reporter.log(
                            ui::LogKind::Info,
                            format!(
                                "marginal confidence {confidence:.2}, will confirm before acting"
                            ),
                        );
                        // fall through to dispatch
                    }
                    ConfidenceAction::SayAgain => {
                        say(
                            &voice,
                            &reporter,
                            ui::LogKind::Info,
                            readback::one_of(&[
                                "Say again?",
                                "Didn't catch that.",
                                "One more time?",
                            ]),
                        );
                        return Line::Spent;
                    }
                    // Not deliberate (continuous-mode ambient audio): ignore silently.
                    ConfidenceAction::Ignore => {
                        reporter.log(ui::LogKind::Info, "low confidence, ignored");
                        return Line::Spent;
                    }
                    // Unreachable inside `confidence < min`, but total for clarity.
                    ConfidenceAction::Act => {}
                }
            }
            if voice.is_self_echo(&command) {
                reporter.log(ui::LogKind::Info, "ignored self-echo (own readback)");
                return Line::Spent;
            }
        }
        Line::Open
    }

    /// A RADIO PTT line goes to the tuned station or into the void, never
    /// to the copilot.
    pub(crate) async fn route_keyed(&mut self, u: &mut Utterance) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let command = &u.command;
        let from_radio = u.from_radio;
        // RADIO PTT: the utterance was transmitted, so it goes to the tuned
        // station or into the void — never to the copilot, and never through
        // the cockpit-command gates below. Physical routing beats content
        // routing whenever the transmit button is bound.
        if from_radio {
            // "Going around" on the air is the pilot's decision, whoever
            // hears it: the loop flying the approach flies it too (never the
            // readback of our tower's order, which it already flies).
            go_around::note_radio_call(&command);
            // Copilot-only mode: the pilot is talking to REAL external ATC
            // (VATSIM, SayIntentions, stock). Their transmission is none of
            // our business — no simulated reply, no void briefing, nothing
            // to step on the real controller's answer.
            if !mode::atc_enabled() {
                reporter.log(ui::LogKind::Info, "transmitted; external ATC has the radio");
                return Line::Spent;
            }
            // "Say again" on the air: the controller on this frequency
            // repeats its last transmission to us, word for word. Never a
            // new clearance: only what was already said, and only its LAST
            // word (see `voice::controller_repeat`). Nothing on file falls
            // through to the station, whose answer is "say again" back.
            if self.atc_station.is_some()
                && tower::is_say_again(&command)
                && repeat_controller(&voice, &reporter, &command, "keyed")
            {
                return Line::Spent;
            }
            if let Some(st) = self.atc_station.as_mut() {
                if let Some((reply, who)) = st
                    .handle_transmission(&command.to_ascii_lowercase(), xplane.view())
                    .await
                {
                    // Empty reply = a readback the controller accepted; a real
                    // frequency stays quiet, so we do too.
                    if reply.is_empty() {
                        reporter.log(ui::LogKind::Info, "readback acknowledged (frequency quiet)");
                        return Line::Spent;
                    }
                    reporter.status(ui::Phase::Speaking);
                    reporter.log(ui::LogKind::Readback, format!("{}: {reply}", who.tag()));
                    voice.say_station(Priority::Readback, who, &reply);
                    reporter.status(ui::Phase::Listening);
                    return Line::Spent;
                }
                // Transmitted into the void: nobody is on this frequency. Dead
                // air is realistic but unhelpful (the 2026-07-08 flight called
                // twice into silence, lost) — so the CREW, not ATC, chimes in
                // with the comm plan: which frequencies exist, where COM1 is.
                // Full briefing at most every few minutes; repeats get a short
                // "no answer" (working down a list of foreign towers, the crew
                // reciting the whole plan each call reads as butting in).
                // In ATC-only mode there is no crew to butt in — dead air is
                // the honest answer (the copilot's "No answer there" was the
                // one voice still heard after flipping the crew off, 2026-07-15).
                if !mode::copilot_enabled() {
                    reporter.log(ui::LogKind::Info, "transmitted, nobody on this frequency");
                    return Line::Spent;
                }
                let line = if self
                    .last_void_brief
                    .is_none_or(|t| t.elapsed() > std::time::Duration::from_secs(300))
                {
                    self.last_void_brief = Some(Instant::now());
                    format!("No answer there. {}", void_briefing(xplane, st).await)
                } else {
                    match xplane.get_float(datarefs::COM1_FREQ_KHZ).await {
                        Ok(v) => format!("No answer on {}.", atis::spoken_mhz(v.round() as i64)),
                        Err(_) => "No answer there.".to_string(),
                    }
                };
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                return Line::Spent;
            }
            reporter.log(ui::LogKind::Info, "transmitted, nobody on this frequency");
            return Line::Spent;
        }
        Line::Open
    }

    /// ATC-only mode: content routing to the station, else one note that
    /// the copilot is off.
    pub(crate) async fn crew_off(&mut self, u: &mut Utterance) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let dual_ptt = self.dual_ptt;
        let command = &u.command;
        // Everything below is the COPILOT's business — intercom commands,
        // corrections, tools, questions. In ATC-only mode there is no crew,
        // but content routing must survive: with no radio key bound it is
        // the ONLY path to the station, and killing it would strand
        // keyboard-PTT and continuous setups off the air entirely.
        if !mode::copilot_enabled() {
            if let Some(st) = self.atc_station.as_mut() {
                let tl = command.to_lowercase();
                if try_route_content_atc(st, &tl, dual_ptt, &voice, &reporter, xplane).await {
                    return Line::Spent;
                }
            }
            // Not a radio call: one honest note the first time, silence
            // after (mirrors the "radios are dead" pattern — discoverable,
            // never naggy).
            if !self.atc_only_noted {
                self.atc_only_noted = true;
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Readback,
                    "Copilot is off. ATC only. Flip the mode in Settings to bring me back.",
                );
            } else {
                reporter.log(ui::LogKind::Info, "intercom ignored (ATC-only mode)");
            }
            return Line::Spent;
        }
        Line::Open
    }

    /// The words rewritten before any gate: a spoken correction of the last
    /// value, a bare "cancel" made the panel's CANCEL.
    pub(crate) fn rewrite(&mut self, u: &mut Utterance) {
        let reporter = &self.reporter;
        let command = &mut u.command;
        let from_text = u.from_text;
        // "I said eighty percent" — a spoken correction of the value the last
        // command set. Whisper heard "8" for "80" live, executed 8%, and the
        // pilot's fix dead-ended; a correction re-anchors on the last subject
        // immediately, before any gate can drop it.
        if !from_text {
            let recent = self
                .history
                .last()
                .is_some_and(|(_, _, t)| t.elapsed() < FOLLOWUP_WINDOW);
            if recent {
                if let (Some(rest), Some(subj)) = (correction_value(&command), &self.last_subject) {
                    let rewritten = format!("{subj} {rest}");
                    reporter.log(ui::LogKind::Info, format!("correction → \"{rewritten}\""));
                    *command = rewritten;
                }
            }
        }

        // A bare "cancel" names no automation: it means what the panel's
        // CANCEL button means, chosen from what is running.
        if router::bare_cancel(&command) {
            if let Some(phrase) = auto_status::cancel_phrase() {
                reporter.log(ui::LogKind::Info, format!("bare cancel → \"{phrase}\""));
                *command = phrase.to_string();
            }
        }
    }

    /// The spoken domain gate: not a command, not a question, not the cabin's
    /// and no follow-up window open, the line is a miss; a bare value
    /// follow-up is anchored on the last subject.
    pub(crate) fn domain_gate(&mut self, u: &mut Utterance) -> Line {
        let voice = &self.voice;
        let reporter = &self.reporter;
        let live = &self.live;
        let cabin_state = &self.cabin_state;
        let explicit = self.explicit;
        let command = &mut u.command;
        let from_text = u.from_text;
        if !from_text
            && !looks_like_command(&command)
            && !looks_like_question(&command)
            // An addressed interphone call is in-domain by construction
            // ("cabin crew, doors to manual" carries no cockpit keyword) —
            // the cabin handler below answers it deterministically.
            && cabin::Cabin::addressed(&command).is_none()
            // So is the call that follows a bare "cabin crew?".
            && !cabin_state
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .line_open(Instant::now())
        {
            // A conversational follow-up shortly after a command is let through
            // even without a keyword. Outside the window, the strict gate holds.
            let in_followup = self
                .history
                .last()
                .is_some_and(|(_, _, t)| t.elapsed() < FOLLOWUP_WINDOW);
            if !(in_followup && looks_like_followup(&command)) {
                // Same treatment as a grammar miss — ONE consistent cue for every
                // clearly-heard utterance we won't act on: log the miss, sound the
                // negative tone. (Only genuinely low-confidence audio gets a spoken
                // "Say again?", where repeating actually helps.)
                // Desktop-only: on the cockpit panel this reads as debug noise
                // shouting CAUTION at the pilot for thinking out loud.
                reporter.log_quiet(ui::LogKind::Warn, format!("not a command: \"{command}\""));
                if explicit && live.get(DR_OPT_TONE).is_none_or(|v| v > 0.5) {
                    voice.negative();
                }
                return Line::Spent;
            }
            // Re-anchor a *truly bare* value follow-up onto the last command's
            // system ("seventy percent" → "throttle seventy percent"). Only when
            // the utterance is nothing but a number/percent — otherwise a misheard
            // command ("fraudal at eighty percent") would get force-anchored to the
            // wrong tool. Non-bare follow-ups go to the LLM with history as-is.
            if is_bare_value(&command) {
                if let Some(subj) = &self.last_subject {
                    let rewritten = format!("{subj} {command}");
                    reporter.log(ui::LogKind::Info, format!("follow-up → \"{rewritten}\""));
                    *command = rewritten;
                }
            }
        }
        Line::Open
    }

    /// "Fly it anyway" with a refusal on offer: the gate waived and the arm
    /// asked again.
    pub(crate) fn take_waiver(&mut self, u: &mut Utterance) {
        let voice = &self.voice;
        let reporter = &self.reporter;
        let command = &mut u.command;
        // A pilot waiving the refusal just said ("fly it anyway"): the gate
        // is waived for that plan and the arm is asked again in the same
        // words the button uses. Words with no refusal on offer fall through.
        if grammar::waiver_request(&command) {
            if let Some(gate) = full_auto::take_waiver() {
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Readback,
                    &full_auto::waiver_ack(gate),
                );
                *command = "fly my flight plan".to_string();
            }
        }
    }

    /// The words resolved to calls: recall and conversational glue answered,
    /// a state question answered, the grammar's calls or the dialogue's
    /// back-reference, else a decline or a miss. `None` is a line spent.
    pub(crate) async fn resolve_calls(
        &mut self,
        u: &mut Utterance,
        nav_topic: Option<String>,
    ) -> Option<Vec<tools::ToolCall>> {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let live = &self.live;
        let profile = &self.profile;
        let registry = &self.registry;
        let checklists = &self.checklists;
        let explicit = self.explicit;
        let command = &u.command;
        // Strip conversational filler/politeness once ("could you tell me the
        // altitude" → "tell me the altitude", "okay flaps ten please" → "flaps
        // ten") and match against this, so polite phrasings don't miss. The raw
        // `command` is kept for logging/history.
        let matchable = grammar::normalize(&command);
        // "Remind me when we get there" is a reminder whose target is the last
        // thing the copilot answered about (the next waypoint, top of descent).
        // Resolved here, deterministically, from the conversation topic; the
        // grammar then sees the words it already understands.
        let matchable = match conversation::resolve_there(&matchable, nav_topic.as_deref()) {
            Some(resolved) => {
                tracing::info!(from = %matchable, to = %resolved, "there resolved");
                resolved
            }
            None => matchable,
        };

        // "Say again" / "what were you saying?": the line the pilot keyed over
        // or the radio stepped on is owed, and outranks an older readback.
        // "What did the tower say?": only what actually went out on OUR
        // radio. With external ATC (copilot mode) the crew never hears the
        // frequency, and says so rather than pretend.
        if let Some(line) = atc_recall_request(&matchable).then(|| {
            atc_recall_line(
                mode::atc_enabled(),
                voice.last_controller_line().as_ref(),
                Instant::now(),
            )
        }) {
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return None;
        }
        if let Some(ask) = recall_ask(&matchable) {
            let readback = LAST_READBACK.lock().ok().and_then(|l| l.clone());
            let line = match recall_line(
                ask,
                &voice.unsaid(),
                readback.as_ref(),
                voice.last_volunteered().as_ref(),
            ) {
                Some((line, spends)) => {
                    if spends {
                        voice.forget_unsaid();
                    }
                    tracing::info!(?ask, owed = spends, line = %line, "recall");
                    line
                }
                None => "Nothing to repeat yet.".to_string(),
            };
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return None;
        }
        // Conversational glue (acks, "what can you do") — deterministic, takes
        // no action. Checked before commands/questions so pleasantries don't
        // fall through to "Say again?".
        if let Some(reply) = conversational(&matchable, &registry) {
            say(&voice, &reporter, ui::LogKind::Readback, &reply);
            return None;
        }

        // Cockpit geography — "where is the inverter?" answered from the profile's
        // verified `locations:` map: deterministic, instant, LLM-off. Runs before
        // the generic question path so a known location beats a state-query miss.
        if let Some(answer) = location_answer(&matchable, &profile) {
            say(&voice, &reporter, ui::LogKind::Readback, &answer);
            return None;
        }

        // Whisper habitually APPENDS "?" to short spoken commands ("Fuel tank,
        // right?"), which used to shunt them down the question path (seen live).
        // Punctuation alone doesn't outrank a parseable command: it's a question
        // only with an interrogative OPENER ("is/are/what/how...") or when the
        // grammar can't parse it anyway.
        // Checklists resolve deterministically by title ("before takeoff
        // checklist"); without this a grammar miss was the end of the road and
        // every checklist was voice-unreachable (seen live in the G1000 172,
        // 2026-07-14). Joined into `parsed` so whisper's habitual trailing "?"
        // doesn't shunt a match down the question path.
        let parsed = grammar::try_parse(&matchable, &registry, &profile).or_else(|| {
            checklist::match_request(&matchable, &checklists).map(|cl| {
                vec![tools::ToolCall {
                    name: cl.id.clone(),
                    arguments: serde_json::json!({}),
                }]
            })
        });
        let opener_q = looks_like_question(matchable.trim_end_matches(['?', '.', ' ']));
        let is_question = looks_like_question(&matchable) && (opener_q || parsed.is_none());

        // Other state reads ("what's our altitude/heading/airspeed") — deterministic
        // too, but only when it's a question, so "set heading 270" still commands.
        if is_question {
            if let Some(answer) =
                answer_state_query(&matchable, xplane, Some(&live), &profile).await
            {
                self.conversation.observe(&command, &answer, Instant::now());
                say(&voice, &reporter, ui::LogKind::Readback, &answer);
                return None;
            }
        }

        // Deterministic fast-path: resolve the common, regular commands with no
        // LLM at all — no GPU burst, no latency, and it can't pick the wrong tool.
        // Genuine questions are held back ("is our heading 270?" shares a command's
        // keyword+number shape — the OPENER decides); they go to the answer path.
        let mut fast = if is_question { None } else { parsed };
        // Back-references ride the dialogue stack: "a bit more", "come left
        // ten", "turn it off", "do that again" mean nothing to the grammar
        // (its RELATIVE bail-out) and the LLM fallback cannot actuate, so
        // resolve them deterministically against the calls just executed.
        // Closed vocabulary — anything else falls through to the normal miss.
        if fast.is_none() && !is_question {
            match self.dialogue.resolve(&matchable, &registry) {
                Some(dialogue::Resolution::Calls(c)) => {
                    tracing::info!(
                        path = "dialogue",
                        tool = c.first().map(|x| x.name.as_str()).unwrap_or(""),
                        "command path"
                    );
                    fast = Some(c);
                }
                Some(dialogue::Resolution::Say(line)) => {
                    tracing::info!(path = "dialogue-say", "command path");
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    return None;
                }
                None => {}
            }
        }
        // A recognized command whose CAPABILITY this airframe lacks (an autopilot
        // command on the AP-less DC-3) — decline honestly rather than fall through
        // to "Say again?" or a vague LLM answer.
        if fast.is_none() {
            if let Some(msg) = grammar::declined(&matchable, &registry) {
                tracing::info!(path = "declined", "command path");
                self.conversation
                    .report(&command, &msg, false, Instant::now());
                say(&voice, &reporter, ui::LogKind::Readback, &msg);
                return None;
            }
            // "Checklist" was heard but no section resolved (a bare "run the
            // checklist", an ambiguous "engine checklist", a section this file
            // doesn't have) - name the options instead of the unrecognized
            // tone. Question-shaped asks ("what checklists do you have?") land
            // here too, and the list is the honest answer either way.
            if checklist::mentions_checklist(&matchable)
                && checklist::match_request(&matchable, &checklists).is_none()
            {
                tracing::info!(path = "checklist-decline", "command path");
                let msg = if checklists.is_empty() {
                    "I have no checklists for this aircraft.".to_string()
                } else {
                    let names = checklists
                        .iter()
                        .map(|c| c.title.to_lowercase())
                        .collect::<Vec<_>>()
                        .join(", ");
                    format!("Which checklist? I have: {names}.")
                };
                say(&voice, &reporter, ui::LogKind::Readback, &msg);
                return None;
            }
        }
        let calls = if let Some(c) = fast {
            tracing::info!(
                path = "fast",
                tool = c.first().map(|x| x.name.as_str()).unwrap_or(""),
                "command path"
            );
            c
        } else {
            // Nothing recognized it. The transcript was clear (it passed the
            // confidence gate), so a spoken "Say again?" would falsely imply a
            // mishear and repeating won't help. Log the miss to the panel and,
            // when addressed directly (PTT), acknowledge with the negative tone.
            // Continuous mode stays silent. Every miss here is a rule not yet
            // written: `xfc analyze` lists them by frequency.
            tracing::info!(path = "deterministic-miss", "command path");
            reporter.log(ui::LogKind::Warn, format!("unrecognized: \"{command}\""));
            if explicit && live.get(DR_OPT_TONE).is_none_or(|v| v > 0.5) {
                voice.negative();
            }
            return None;
        };
        Some(calls)
    }
}
