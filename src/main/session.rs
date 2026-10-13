//! The command loop (`run_session`), one pilot line at a time through
//! its stages, each in its own file and run in the loop's order:
//!
//! 1. take a line (`take.rs`): push-to-talk or continuous capture, the
//!    typed line (panel, terminal, EFB), the airframe it is for; between
//!    taking and routing, the loop's own station work
//!    (`station_work.rs`: mode, station swaps, tower syncs) and the crew's
//!    queued radio calls (`speak.rs`);
//! 2. route it (`route.rs`): radio or crew, the confidence, echo and
//!    domain gates, the rewrites, the words resolved to calls;
//! 3. dispatch (`dispatch.rs`, `answer.rs`): undo, stop and go words,
//!    flow and offer replies, the instructor, conversation, the answers
//!    on words alone, the hand-over and the tools;
//! 4. speak (`speak.rs`): the crew's radio calls and the turn recorded
//!    for the next back-reference.
//!
//! The stages were moved out of one 2,665-line function whole: their
//! statements and their order are the loop's.
use super::*;

/// The command loop's state: what `run_session` held in locals, lifted
/// whole so each stage reads what it needs.
pub(crate) struct Session {
    // The run's wiring, as `run_session` was handed it.
    pub(crate) stt: Stt,
    pub(crate) live_profile: std::sync::Arc<LiveProfile>,
    pub(crate) voice: Voice,
    pub(crate) explicit: bool,
    pub(crate) ptt: Option<Ptt>,
    pub(crate) reporter: Reporter,
    pub(crate) typed: Option<std::sync::mpsc::Receiver<String>>,
    pub(crate) cabin_state: std::sync::Arc<std::sync::Mutex<cabin::Cabin>>,
    // Built at the start (see `Session::start` for each one's why).
    pub(crate) xplane: XPlaneClient,
    pub(crate) dual_ptt: bool,
    pub(crate) xp_radio_muted: bool,
    pub(crate) sim_radio_sync: SimRadioSync,
    pub(crate) live: LiveState,
    pub(crate) listener: Option<Listener>,
    pub(crate) ptt_mic: Option<PttMic>,
    pub(crate) capture_tuning: CaptureTuning,
    // Profile-derived, rebuilt on an airframe swap (`follow_airframe`).
    pub(crate) active_gen: u64,
    pub(crate) profile: aircraft::AircraftProfile,
    pub(crate) registry: Registry,
    pub(crate) checklists: std::sync::Arc<Vec<checklist::Checklist>>,
    // The world side and the mode the loop has applied.
    pub(crate) atc_station: Option<party_line::Station>,
    pub(crate) home_airport_name: Option<String>,
    pub(crate) applied_mode: mode::Mode,
    pub(crate) atc_only_noted: bool,
    pub(crate) instructor_noted: bool,
    // The conversation's memory.
    pub(crate) history: Vec<(String, String, Instant)>,
    pub(crate) last_subject: Option<String>,
    pub(crate) fp_cache: Option<(Instant, simbrief::FlightPlan)>,
    pub(crate) plan_pending: Option<Instant>,
    pub(crate) last_void_brief: Option<Instant>,
    pub(crate) reminders: tools::reminder::Reminders,
    pub(crate) undo_stack: Vec<UndoSnapshot>,
    pub(crate) dialogue: dialogue::Dialogue,
    pub(crate) conversation: conversation::Conversation,
}

/// One pilot line on its way through the stages: what the loop held in
/// per-utterance locals.
pub(crate) struct Utterance {
    /// This utterance's settings, read fresh (`settings::live`).
    pub(crate) config: std::sync::Arc<Config>,
    /// The words, rewritten on the way (a correction, a bare cancel, a
    /// follow-up anchored, a waiver, a demonstration's own words).
    pub(crate) command: String,
    pub(crate) from_text: bool,
    pub(crate) spoken_confidence: Option<f32>,
    pub(crate) from_radio: bool,
    // Set when a marginal-confidence command is let through to be read back
    // for a yes/no instead of dropped (the confirm loop below honours it).
    // Fresh each utterance.
    pub(crate) force_confirm_low: bool,
}

/// What a stage did with the line: `Spent` ends the utterance (the old
/// loop's `continue`), `Open` passes it on to the next stage.
#[must_use]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Line {
    Spent,
    Open,
}

impl Line {
    pub(crate) fn spent(self) -> bool {
        self == Line::Spent
    }
}

impl Session {
    /// The session's state built, the barge-in watcher started and the mic
    /// opened, as the loop's first lines did.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn start(
        config: Config,
        mut stt: Stt,
        live_profile: std::sync::Arc<LiveProfile>,
        voice: Voice,
        continuous: bool,
        explicit: bool,
        ptt: Option<Ptt>,
        reporter: Reporter,
        typed: Option<std::sync::mpsc::Receiver<String>>,
        cabin_state: std::sync::Arc<std::sync::Mutex<cabin::Cabin>>,
    ) -> Result<Self> {
        // Build the X-Plane client here, on whatever runtime this session runs on
        // (main for plain, the voice thread for TUI) — a reqwest client's I/O is
        // bound to the runtime it's used on.
        let xplane = XPlaneClient::new(config.xplane_base_url.clone());
        // Two PTT channels bound (intercom + radio): physical routing is complete,
        // so intercom speech never content-routes to ATC (see Ptt::has_radio_channel).
        let dual_ptt = ptt.as_ref().is_some_and(Ptt::has_radio_channel);
        // X-Plane's own radio voice (built-in ATC + its ATIS) speaks on the same
        // frequencies as our stations — heard live as two ATIS voices at once
        // (2026-07-10). While our ATC is on, Roger owns the radio soundscape:
        // the sim's "Radios" volume goes to zero (this also silences nav-aid
        // morse idents; XCOPILOT_MUTE_XP_RADIO=false keeps the sim's audio).
        // Mode-aware, boot AND live toggles: copilot-only means the radio — and
        // its soundscape — belongs to whatever external ATC the pilot uses, so
        // leaving OUR world gives X-Plane its voice back.
        // Applied by `SimRadioSync` in the loop, not once here: the boot write
        // failed silently whenever X-Plane was not up yet, and the audio bus
        // changes what the right answer is (see `tts::sim_radio`).
        let xp_radio_muted = config.mute_xp_radio && !config.atc_airport.trim().is_empty();
        let sim_radio_sync = SimRadioSync::default();
        // Subscribe to the continuously-read state over the WebSocket so snapshots are
        // instant cache reads instead of a REST burst. Best-effort: reads fall back to
        // REST until (or unless) the subscription warms up.
        let live = LiveState::start(&config.xplane_base_url, SNAPSHOT_DATAREFS);

        // Profile-derived state, rebuilt whenever the aircraft switches (see the
        // generation check at the top of the loop). Seeded from the live profile.
        let active_gen = live_profile.generation();
        let profile = live_profile.get();
        let registry = Registry::for_aircraft(&profile);
        // The flow module owns the parsed lists (its phase-boundary runner and
        // the panel need them off this loop); this is the same Arc for matching.
        let checklists = flow::install(&profile);
        instructor::install(&profile);
        // The ATC station (tower dialogue), merged from the standalone atc binary:
        // it answers when COM1 is tuned to its tower frequency and the words parse
        // as a radio call. Read-only toward the sim; speaks in its own voice
        // through the same arbiter. None = no station, copilot behaves as before —
        // and copilot-only mode never loads one (the radio belongs to external ATC).
        let atc_station = if mode::atc_enabled() {
            install_home_station(&config, &voice)
        } else {
            None
        };
        let home_airport_name = atc_station.as_ref().map(|s| s.airport_name().to_string());
        // The mode the loop has already applied — a panel/env change is noticed
        // at the top of the next iteration (≤2 s: idle ticks) and reconciled.
        let applied_mode = mode::get();
        // ATC-only: the crew note "copilot is off" is spoken once, not per utterance.
        let atc_only_noted = false;
        // Instructor mode: the full "I don't touch the controls" line once per
        // spell in the mode, a short "your hands" after.
        let instructor_noted = false;
        // Radio-call vocabulary rides in a SEPARATE whisper bias prompt applied
        // only to radio-keyed captures: prompt prefill is GPU work per decode, and
        // the tower phrasing more than doubles it (483 → 1,047 ms median, felt as
        // sim fps dips) — cockpit commands must never pay for it.
        stt.set_prompt(stt::build_prompt(&profile));
        stt.set_radio_prompt(format!(
            "{} {}",
            stt::build_prompt(&profile),
            tower::STT_PROMPT
        ));
        // Rolling conversation memory: recent (command, readback, when) turns, used
        // to resolve back-references and to gate conversational follow-ups.
        let history: Vec<(String, String, Instant)> = Vec::new();
        // Subject of the last successful named command ("throttle", "set heading"),
        // used to re-anchor a bare-value follow-up ("sixty percent" → "throttle
        // sixty percent") into an explicit command.
        let last_subject: Option<String> = None;
        // Cached SimBrief plan (it doesn't change mid-flight); re-fetched on TTL or "refresh".
        let fp_cache: Option<(Instant, simbrief::FlightPlan)> = None;
        // A bare "divert" / "plan a flight" asked "to where?"; the next short
        // utterance inside this window is the answer.
        let plan_pending: Option<Instant> = None;
        // Last time the crew read the full comm plan after a dead-air transmission.
        // The 2026-07-08 night flight called Malaga, Jerez and Cadiz in a row and got
        // the whole Gibraltar briefing every time — once is help, thrice is butting in.
        let last_void_brief: Option<Instant> = None;
        // Memory of armed reminders, so the copilot can recall and cancel them.
        let reminders = tools::reminder::Reminders::default();
        // Recovery for recent utterances (a stack), so "belay that" can walk back the
        // last command, then the one before it, each with how to undo it.
        let undo_stack: Vec<UndoSnapshot> = Vec::new();
        // The dialogue stack: executed calls, so back-references ("a bit more",
        // "turn it off", "do that again") resolve deterministically against what
        // was actually just done.
        let dialogue = dialogue::Dialogue::new();
        let conversation = conversation::Conversation::default();

        // Barge-in: a watcher thread that, while the copilot is speaking, kills the
        // audio the instant PTT is pressed — so the pilot is never talked over and a
        // long readback yields immediately. Readbacks block the command loop while
        // speaking, so the interrupt can't come from it; it needs its own thread.
        // (A checklist flow runs on its own thread and is stopped by the WORD
        // "stop", not the press: the press is how the pilot answers it.)
        spawn_barge_in(&ptt, &voice);
        let listener = if continuous {
            // Share the speech arbiter's mic gate so the listener mutes itself while
            // the copilot speaks (readbacks and monitor alerts alike).
            Some(Listener::start(
                config.vad_threshold,
                config.end_silence_ms,
                reporter.clone(),
                voice.mic_gate(),
                std::time::Duration::from_millis(config.mic_gate_tail_ms),
            )?)
        } else {
            None
        };
        // Warm push-to-talk mic: kept open across presses so the device warm-up never
        // clips the start of a command. Only one of `listener`/`ptt_mic` opens the
        // device — they're mutually exclusive (continuous vs PTT).
        let ptt_mic = if ptt.is_some() {
            Some(PttMic::start(
                voice.mic_gate(),
                std::time::Duration::from_millis(config.mic_gate_tail_ms),
            )?)
        } else {
            None
        };

        let capture_tuning = CaptureTuning::of(&config);

        reporter.log(
            ui::LogKind::Info,
            match &ptt {
                Some(Ptt::Key(_)) => format!("Ready — hold {} to talk.", config.ptt_key),
                Some(Ptt::Dataref { .. }) => {
                    "Ready — hold your bound PTT button to talk.".to_string()
                }
                None if !continuous => "Ready — push-to-talk (Enter to start/stop).".to_string(),
                None => "Listening — just speak commands.".to_string(),
            },
        );

        Ok(Session {
            stt,
            live_profile,
            voice,
            explicit,
            ptt,
            reporter,
            typed,
            cabin_state,
            xplane,
            dual_ptt,
            xp_radio_muted,
            sim_radio_sync,
            live,
            listener,
            ptt_mic,
            capture_tuning,
            active_gen,
            profile,
            registry,
            checklists,
            atc_station,
            home_airport_name,
            applied_mode,
            atc_only_noted,
            instructor_noted,
            history,
            last_subject,
            fp_cache,
            plan_pending,
            last_void_brief,
            reminders,
            undo_stack,
            dialogue,
            conversation,
        })
    }

    /// The loop: a line taken (or the idle tick), the station work, then
    /// the line through its stages.
    pub(crate) async fn run(mut self) -> Result<()> {
        loop {
            // This utterance's settings, read fresh: the confidence bands, the
            // confirm gate, the LLM fallback and the readback challenges all
            // become live changes rather than boot constants. Boot-time values
            // (paths, base URLs, the capture wiring) stay as they were resolved.
            let config = settings::live();
            // Capture tuning IS re-applied, because tuning a mic gate by ear is
            // the reason a settings screen exists: a change lands on the next
            // utterance instead of the next launch.
            self.capture_tuning
                .follow(&config, self.listener.as_mut(), self.ptt_mic.as_ref());
            self.reporter.status(ui::Phase::Listening);
            let input = next_input(
                &mut self.listener,
                &self.ptt_mic,
                &self.stt,
                &self.ptt,
                self.typed.as_ref(),
            );

            self.station_work(&config, &input).await;

            // Station work consumed — an idle tick carries no utterance to route.
            let Some(mut u) = self.take_line(input, config) else {
                continue;
            };
            self.handle(&mut u).await;
        }
    }

    /// One line through the stages, in the loop's order. Every early return
    /// is the old loop's `continue`, so the turn's guard drops where it did.
    async fn handle(&mut self, u: &mut Utterance) {
        self.follow_airframe();
        if self.undo(u).await.spent() {
            return;
        }
        if self.radio_gates(u).spent() {
            return;
        }
        if self.confidence_gates(u).await.spent() {
            return;
        }
        if self.route_keyed(u).await.spent() {
            return;
        }

        // The copilot's turn: the first reply waits a human beat, and a turn
        // that goes quiet past `tempo::STANDBY_AFTER` hears "stand by"
        // instead of silence. Dropped on every `continue` below.
        let _turn = {
            let voice = self.voice.clone();
            tempo::Turn::begin(!u.from_text, move || {
                if mode::copilot_enabled() {
                    let line = persona::standby_line(persona::dialect());
                    tracing::info!(spoke = %line, "standby filler");
                    voice.alert(Priority::Readback, line);
                }
            })
        };

        if self.crew_off(u).await.spent() {
            return;
        }
        self.rewrite(u);
        if self.stop_go_words(u).await.spent() {
            return;
        }
        if self.flow_replies(u).await.spent() {
            return;
        }
        // Read-only conversation runs before the spoken domain gate: "and now"
        // and clarification replies have no command keyword. Its response type
        // has no tool/action variant, and refreshed questions never enter grammar.
        // What the last state answer was about, taken BEFORE respond() retires
        // the topic for an unrelated utterance: "remind me when we get there"
        // is unrelated to the conversation module but not to the pilot.
        let nav_topic = self.conversation.recent_observation_query(Instant::now());
        if self.converse(u).await.spent() {
            return;
        }
        if self.domain_gate(u).spent() {
            return;
        }
        self.take_waiver(u);

        // From here the utterance is resolved deterministically: the grammar
        // (one call per distinct action, so "taxi and nav lights on" makes
        // two), the dialogue stack for back-references, the state and
        // knowledge answerers for questions. Nothing guesses; a miss is a miss.
        self.reporter.status(ui::Phase::Thinking);

        // Deterministic state queries answered without the LLM — the model proved
        // unreliable here (it had the fuel in its context and still said "I don't
        // have fuel information"). A fuel question reads the datarefs and is spoken
        // straight back.
        // The pre-grammar shortcuts' decisions, taken once from the words
        // (`question_shortcut`): every branch below that answers on words
        // alone asks this, so `xfc sortie --check` sees what the loop does.
        let shortcut = question_shortcut(
            &u.command,
            &self.registry,
            &self.profile,
            self.fp_cache.as_ref().map(|(_, fp)| fp),
        );
        if self.answer_fuel(u, shortcut).await.spent() {
            return;
        }
        if self.cabin_call(u).spent() {
            return;
        }
        if self.taxi_question(u).await.spent() {
            return;
        }
        if self.desk(u, shortcut).await.spent() {
            return;
        }
        let Some(calls) = self.resolve_calls(u, nav_topic).await else {
            return;
        };
        let Some(calls) = self.hand_over(calls).await else {
            return;
        };
        let turn = self.run_calls(u, calls).await;
        self.record_turn(u, turn);
    }
}

/// The barge-in watcher thread, started once for the session.
fn spawn_barge_in(ptt: &Option<Ptt>, voice: &Voice) {
    if let Some(ptt) = ptt.clone() {
        let wv = voice.clone();
        std::thread::Builder::new()
            .name("barge-in".into())
            .spawn(move || {
                let mut was_ptt = false;
                loop {
                    // ANY PTT down — radio transmit OR intercom to the copilot —
                    // mutes the whole cockpit while held: you don't talk over the
                    // frequency, and (the reason this matters on speakers) the mic
                    // can't re-hear ATC over the speakers while you give a command.
                    // Half-duplex: a station transmission isn't erased, it's
                    // STEPPED ON and its remainder resumes on release. Radio and
                    // intercom still route CONTENT differently (transmit to ATC vs
                    // command the copilot); only the audio hold is shared here.
                    let ptt_down = ptt.is_pressed();
                    if ptt_down && !was_ptt {
                        if wv.is_speaking() {
                            wv.shutup();
                        }
                        wv.set_hold(true);
                    }
                    if !ptt_down && was_ptt {
                        // Release: the stepped-on transmission's remainder (if
                        // real time hasn't outrun it) plays from mid-word, then
                        // the held backlog.
                        wv.set_hold(false);
                    }
                    was_ptt = ptt_down;
                    std::thread::sleep(Duration::from_millis(30));
                }
            })
            .expect("spawn barge-in watcher");
    }
}
