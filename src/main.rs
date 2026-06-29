//! x-copilot (working name; product name: Roger)
//!
//! The voice loop, end to end and fully local:
//!   listen → whisper (STT) → local LLM picks one tool →
//!   write dataref + read back → say/piper (TTS) confirms.
//!
//! Two capture modes (config `listen_mode`): `continuous` (always listening,
//! VAD-segmented) or `ptt` (push-to-talk).

mod aircraft;
mod audio;
mod checklist;
mod config;
mod datarefs;
mod grammar;
mod livestate;
mod llm;
mod monitor;
mod nav;
mod phrase;
mod plugin_status;
mod readback;
mod simbrief;
mod stt;
mod stt_eval;
mod tts;
#[cfg(feature = "onnx-tts")]
mod tts_onnx;
mod tts_sidecar;
mod tools;
mod ui;
mod vad;
mod voice;
mod xplane;

use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};

use audio::{Listener, PttMic};
use config::Config;
use livestate::LiveState;
use llm::{Decision, Llm};
use stt::Stt;
use tools::{Registry, ToolCtx};
use ui::Reporter;
use voice::{Priority, Voice};
use xplane::XPlaneClient;

// Single-threaded runtime: the cpal stream isn't `Send`, and the loop is
// inherently sequential (listen → think → act → speak), so there's nothing to
// gain from a multi-threaded scheduler.
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    use tracing_subscriber::prelude::*;
    let config = Config::from_env()?;
    let tui = config.ui == "tui";

    // Always log to a daily-rolling file. In TUI mode the stderr layer would
    // scribble over the dashboard, so it's omitted there (file only).
    let file_appender = tracing_appender::rolling::daily("logs", "x-copilot.log");
    let (file_writer, _log_guard) = tracing_appender::non_blocking(file_appender);
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| "x_copilot=info".into());
    let stderr_layer =
        (!tui).then(|| tracing_subscriber::fmt::layer().with_writer(std::io::stderr));
    tracing_subscriber::registry()
        .with(filter)
        .with(tracing_subscriber::fmt::layer().with_ansi(false).with_writer(file_writer))
        .with(stderr_layer)
        .init();

    // Route whisper/ggml C logging through tracing so its model-load chatter
    // goes to the log file, not onto the terminal (which would corrupt the TUI).
    whisper_rs::install_whisper_tracing_trampoline();

    // Diagnostic mode: `cargo run -- mic-check` prints live mic levels and exits
    // on Ctrl-C. Use it to confirm capture and tune XCOPILOT_VAD_RMS.
    if std::env::args().nth(1).as_deref() == Some("mic-check") {
        return audio::mic_check(config.vad_threshold);
    }

    // `cargo run -- key-probe` prints each key's name as you press it, so you can
    // find the right value for XCOPILOT_PTT_KEY (e.g. press Right Command → "RMeta").
    if std::env::args().nth(1).as_deref() == Some("key-probe") {
        use device_query::{DeviceQuery, DeviceState};
        let ds = DeviceState::new();
        println!("Press a key to see its name (grant Input Monitoring if prompted). Ctrl-C to quit.");
        let mut last: Vec<device_query::Keycode> = Vec::new();
        loop {
            let keys = ds.get_keys();
            if keys != last {
                if !keys.is_empty() {
                    println!("  {keys:?}");
                }
                last = keys;
            }
            std::thread::sleep(std::time::Duration::from_millis(40));
        }
    }

    // `cargo run -- transcribe <audio-file>` prints the whisper transcript of a
    // clip to stdout and exits. Used by `voice/voices.py` to auto-caption an
    // imported voice sample (Spark needs the reference text to match the audio).
    if std::env::args().nth(1).as_deref() == Some("transcribe") {
        let path = std::env::args()
            .nth(2)
            .ok_or_else(|| anyhow!("usage: x-copilot transcribe <audio-file>"))?;
        let text = transcribe_file(&config.whisper_model_path, &path)?;
        println!("{text}");
        return Ok(());
    }

    // `cargo run -- analyze` mines the logs for fast-path coverage and the
    // ranked LLM-fallback phrasings — the candidates to promote into the
    // deterministic grammar so coverage ratchets up over time.
    if std::env::args().nth(1).as_deref() == Some("analyze") {
        return analyze_commands();
    }

    // `cargo run -- stt-eval [manifest.yaml]` runs the offline speech-accuracy
    // harness: each case through whisper + grammar, reporting WER and command
    // accuracy. `WHISPER_MODEL=...` switches the model under test. Offline, so it
    // uses a fixed profile (no X-Plane needed).
    if std::env::args().nth(1).as_deref() == Some("stt-eval") {
        let manifest = std::env::args().nth(2).unwrap_or_else(|| "bench/stt_cases.yaml".to_string());
        let id = if config.aircraft.eq_ignore_ascii_case("auto") {
            "cessna_172sp".to_string()
        } else {
            config.aircraft.clone()
        };
        let profile = aircraft::load(&id).ok_or_else(|| anyhow!("unknown aircraft profile: {id}"))?;
        return stt_eval::run(&config.whisper_model_path, &profile, &manifest);
    }

    // Resolve the aircraft profile: auto-detect from X-Plane's acf_ICAO when
    // configured as "auto", otherwise use the configured id. Falls back to the
    // Cessna if detection fails (e.g. X-Plane not reachable yet). The probe
    // client is used and dropped here — `run_session` makes its own (a reqwest
    // client must be used on the runtime it runs on).
    let aircraft_id = if config.aircraft.eq_ignore_ascii_case("auto") {
        let probe = XPlaneClient::new(config.xplane_base_url.clone());
        match probe.aircraft_icao().await {
            Ok(icao) => {
                let id = icao_to_profile(&icao);
                tracing::info!(icao = %icao, profile = %id, "auto-detected aircraft");
                id
            }
            Err(e) => {
                tracing::warn!("aircraft auto-detect failed ({e:#}); defaulting to cessna_172sp");
                "cessna_172sp".to_string()
            }
        }
    } else {
        config.aircraft.clone()
    };
    let profile = aircraft::load(&aircraft_id)
        .ok_or_else(|| anyhow!("unknown aircraft profile: {aircraft_id}"))?;

    tracing::info!(
        model = %config.model,
        aircraft = %profile.display_name,
        mode = %config.listen_mode,
        "starting x-copilot (local LLM via Ollama)"
    );

    // whisper model load is the one slow init — do it once, up front. Bias its
    // decoding toward the active aircraft's actual systems and phraseology.
    let stt = Stt::load(&config.whisper_model_path, config.whisper_gpu)?;
    let llm = Llm::new(&config);
    // The tool registry, STT prompt, and checklists are all derived from the
    // profile inside run_session — and rebuilt there when the aircraft switches.

    // Push-to-talk, if configured. When set it takes over capture: the copilot
    // listens only while the button/key is held, so there's no wake word and
    // ambient/ATC can't trigger it. A sim dataref (the bundled X-Plane plugin)
    // takes precedence over a global key — it needs no OS permission.
    let ptt = if !config.ptt_dataref.is_empty() {
        // Leak the name to 'static for the dedicated subscription (lives for the
        // whole run); a one-time leak of one config string.
        let name: &'static str = Box::leak(config.ptt_dataref.clone().into_boxed_str());
        let live = std::sync::Arc::new(LiveState::start(&config.xplane_base_url, &[name]));
        Some(Ptt::Dataref { live, name: config.ptt_dataref.clone() })
    } else if !config.ptt_key.is_empty() {
        match audio::parse_ptt_key(&config.ptt_key) {
            Some(k) => Some(Ptt::Key(k)),
            None => {
                tracing::warn!(key = %config.ptt_key, "XCOPILOT_PTT_KEY not recognised — ignoring");
                None
            }
        }
    } else {
        None
    };
    // Continuous only when not using PTT. "Explicit" = the user clearly addressed
    // the copilot (any push-to-talk); a non-command is then ignored silently rather
    // than answered with an apology.
    let continuous = config.listen_mode == "continuous" && ptt.is_none();
    let explicit = !continuous;

    // The speech arbiter is shared by the command loop and the monitor, so it's
    // created here (above both) and handed to each.
    let voice = Voice::new(config.clone());

    // In-sim status panel (the bundled plugin): mirror phase/heard/action into its
    // datarefs. Best-effort — a no-op if the plugin isn't loaded.
    let status_sink = config
        .plugin_status
        .then(|| plugin_status::StatusSink::start(&config.xplane_base_url));

    // The active profile, shared so the aircraft can be switched live. Seeded with
    // the startup detection; the poller below corrects it if the sim's aircraft
    // differs or changes (or if X-Plane wasn't up yet at launch).
    let live_profile = LiveProfile::new(profile.clone());

    // Aircraft-change poller: re-read acf_ICAO periodically and swap the profile
    // when it changes, so the copilot reconfigures itself (tools, limits, monitor)
    // instead of staying on whatever was loaded at startup. Auto-detect only.
    if config.aircraft.eq_ignore_ascii_case("auto") {
        let live = live_profile.clone();
        let base = config.xplane_base_url.clone();
        let det_voice = voice.clone();
        let det_status = status_sink.clone();
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else { return };
            rt.block_on(async move {
                let xp = XPlaneClient::new(base);
                loop {
                    if let Ok(icao) = xp.aircraft_icao().await {
                        let icao = icao.trim().to_string();
                        if !icao.is_empty() {
                            let id = icao_to_profile(&icao);
                            if id != live.id() {
                                if let Some(p) = aircraft::load(&id) {
                                    let name = p.display_name.clone();
                                    tracing::info!(icao = %icao, profile = %id, "aircraft changed; switching profile");
                                    live.swap(p);
                                    if let Some(s) = det_status.as_ref() {
                                        s.alert(plugin_status::AlertLevel::Advisory, &format!("Now set up for the {name}."));
                                    }
                                    det_voice.alert(voice::Priority::Advisory, format!("Now set up for the {name}."));
                                }
                            }
                        }
                    }
                    tokio::time::sleep(std::time::Duration::from_secs(3)).await;
                }
            });
        });
    }

    // Background monitor: watches for problems and speaks up. Its own thread +
    // runtime + X-Plane client, sharing the speech arbiter. Runs in both UIs.
    // Reads the live profile so its limits switch with the aircraft.
    if config.monitor_enabled {
        let mon_voice = voice.clone();
        let mon_config = config.clone();
        let mon_live = live_profile.clone();
        let mon_status = status_sink.clone();
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else { return };
            rt.block_on(monitor::run(mon_config, mon_live, mon_voice, mon_status));
        });
    }

    if config.ui == "tui" {
        // TUI: voice loop + aircraft poll run on background threads, each with
        // its own runtime; the dashboard renders on the main thread.
        let (tx, rx) = std::sync::mpsc::channel::<ui::Report>();
        let title = format!("x-copilot {} · {}", env!("CARGO_PKG_VERSION"), profile.display_name);
        // The active engines, shown in the status bar: STT (whisper model), LLM,
        // and TTS backend — so it's obvious at a glance what's running.
        let engines = format!(
            "whisper {} · llm {} · tts {}",
            whisper_model_label(&config.whisper_model_path),
            config.model,
            config.tts_backend,
        );
        let commands = command_hints();

        let tx_poll = tx.clone();
        let poll_base = config.xplane_base_url.clone();
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else { return };
            rt.block_on(async move {
                let xp = XPlaneClient::new(poll_base);
                loop {
                    if let Some(ac) = poll_aircraft(&xp).await {
                        if tx_poll.send(ui::Report::Aircraft(ac)).is_err() {
                            break; // dashboard gone
                        }
                    }
                    tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
                }
            });
        });

        // Typed-command channel: the dashboard (main thread) sends commands the
        // pilot types; the voice loop picks them up alongside spoken utterances.
        let (cmd_tx, cmd_rx) = std::sync::mpsc::channel::<String>();
        let mut reporter = Reporter::tui(tx);
        if let Some(s) = status_sink.clone() {
            reporter = reporter.with_status(s);
        }
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else { return };
            if let Err(e) = rt.block_on(run_session(
                config, stt, llm, live_profile, voice, continuous, explicit, ptt, reporter, Some(cmd_rx),
            )) {
                tracing::error!("voice loop ended: {e:#}");
            }
        });

        ui::run_dashboard(title, engines, commands, rx, cmd_tx)?; // blocks until the user quits
        Ok(())
    } else {
        println!("\n  x-copilot — {} ({}). Ctrl-C to quit.\n", profile.display_name, config.listen_mode);
        let mut reporter = Reporter::plain();
        if let Some(s) = status_sink {
            reporter = reporter.with_status(s);
        }
        run_session(config, stt, llm, live_profile, voice, continuous, explicit, ptt, reporter, None).await
    }
}

/// The voice loop: listen → transcribe → pick a tool → confirm → execute → read
/// back. Progress is emitted through `reporter` (plain stdout or the TUI), so
/// the same code drives both UIs.
/// Recent turns kept for back-reference resolution (see [`Llm::choose_tool`]).
const HISTORY_MAX: usize = 3;
/// Turns older than this are dropped from context — stale state shouldn't bleed
/// into a new command.
const HISTORY_TTL: Duration = Duration::from_secs(90);
/// After a command, a window in which a continuation utterance ("switch it back",
/// "a bit more") is accepted even without a hard aviation keyword — the LLM
/// resolves it from history. Outside the window the strict keyword gate applies.
const FOLLOWUP_WINDOW: Duration = Duration::from_secs(45);

/// How the loop decides when to capture audio. Absent (`None`) means continuous
/// VAD or Enter-to-talk; present means push-to-talk, gated either way.
#[derive(Clone)]
enum Ptt {
    /// Hold a global key — polled via `device_query`, so it works while X-Plane
    /// has focus. Needs macOS Input-Monitoring permission.
    Key(device_query::Keycode),
    /// A sim dataref goes nonzero while a bound button is held (the bundled
    /// X-Plane plugin's `xcopilot/ptt_active`). No OS permission, no radio keying.
    Dataref { live: std::sync::Arc<LiveState>, name: String },
}

/// The active aircraft profile, shared between the command loop, the monitor, and
/// the detection poller. The poller swaps it when the sim's aircraft changes;
/// consumers notice via the generation counter and rebuild off the new profile.
pub(crate) struct LiveProfile {
    inner: std::sync::RwLock<aircraft::AircraftProfile>,
    generation: std::sync::atomic::AtomicU64,
}

impl LiveProfile {
    fn new(p: aircraft::AircraftProfile) -> std::sync::Arc<Self> {
        std::sync::Arc::new(Self {
            inner: std::sync::RwLock::new(p),
            generation: std::sync::atomic::AtomicU64::new(0),
        })
    }
    /// A clone of the current profile.
    pub(crate) fn get(&self) -> aircraft::AircraftProfile {
        self.inner.read().unwrap().clone()
    }
    fn id(&self) -> String {
        self.inner.read().unwrap().id.clone()
    }
    /// Bumps every time the profile is swapped — consumers compare to know to rebuild.
    pub(crate) fn generation(&self) -> u64 {
        self.generation.load(std::sync::atomic::Ordering::Acquire)
    }
    fn swap(&self, p: aircraft::AircraftProfile) {
        *self.inner.write().unwrap() = p;
        self.generation.fetch_add(1, std::sync::atomic::Ordering::Release);
    }
}

impl Ptt {
    /// True while the PTT dataref reads pressed (only meaningful for `Dataref`).
    fn dataref_active(live: &LiveState, name: &str) -> bool {
        live.get(name).unwrap_or(0.0) > 0.5
    }

    /// True while PTT is held, whichever source — for the barge-in watcher.
    fn is_pressed(&self) -> bool {
        match self {
            Ptt::Key(key) => {
                use device_query::DeviceQuery;
                device_query::DeviceState::new().get_keys().contains(key)
            }
            Ptt::Dataref { live, name } => Ptt::dataref_active(live, name),
        }
    }
}

#[allow(clippy::too_many_arguments)]
async fn run_session(
    config: Config,
    mut stt: Stt,
    llm: Llm,
    live_profile: std::sync::Arc<LiveProfile>,
    voice: Voice,
    continuous: bool,
    explicit: bool,
    ptt: Option<Ptt>,
    reporter: Reporter,
    typed: Option<std::sync::mpsc::Receiver<String>>,
) -> Result<()> {
    // Build the X-Plane client here, on whatever runtime this session runs on
    // (main for plain, the voice thread for TUI) — a reqwest client's I/O is
    // bound to the runtime it's used on.
    let xplane = XPlaneClient::new(config.xplane_base_url.clone());
    // Subscribe to the continuously-read state over the WebSocket so snapshots are
    // instant cache reads instead of a REST burst. Best-effort: reads fall back to
    // REST until (or unless) the subscription warms up.
    let live = LiveState::start(&config.xplane_base_url, SNAPSHOT_DATAREFS);

    // Profile-derived state, rebuilt whenever the aircraft switches (see the
    // generation check at the top of the loop). Seeded from the live profile.
    let mut active_gen = live_profile.generation();
    let mut profile = live_profile.get();
    let mut registry = Registry::for_aircraft(&profile);
    let mut checklists = checklist::load(&profile).unwrap_or_else(|e| {
        tracing::error!("checklist load failed: {e:#}; running without checklists");
        Vec::new()
    });
    stt.set_prompt(stt::build_prompt(&profile));
    // Tool specs for the LLM = curated tools + each checklist as a no-arg "tool"
    // the loop intercepts and runs interactively.
    let mut specs = registry.specs();
    specs.extend(checklists.iter().map(|c| c.spec()));
    // Rolling conversation memory: recent (command, readback, when) turns, used
    // to resolve back-references and to gate conversational follow-ups.
    let mut history: Vec<(String, String, Instant)> = Vec::new();
    // Subject of the last successful named command ("throttle", "set heading"),
    // used to re-anchor a bare-value follow-up ("sixty percent" → "throttle
    // sixty percent") into an explicit command.
    let mut last_subject: Option<String> = None;
    // Cached SimBrief plan (it doesn't change mid-flight); re-fetched on TTL or "refresh".
    let mut fp_cache: Option<(Instant, simbrief::FlightPlan)> = None;
    // Memory of armed reminders, so the copilot can recall and cancel them.
    let reminders = tools::reminder::Reminders::default();
    // Recovery for recent utterances (a stack), so "belay that" can walk back the
    // last command, then the one before it, each with how to undo it.
    let mut undo_stack: Vec<UndoSnapshot> = Vec::new();

    // Barge-in: a watcher thread that, while the copilot is speaking, kills the
    // audio the instant PTT is pressed — so the pilot is never talked over and a
    // long readback or checklist yields immediately. Readbacks and checklists block
    // the command loop while speaking, so the interrupt can't come from it; it needs
    // its own thread. The flag lets a running checklist see the press and stop.
    let interrupt = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    if let Some(ptt) = ptt.clone() {
        let wv = voice.clone();
        let wi = interrupt.clone();
        std::thread::Builder::new()
            .name("barge-in".into())
            .spawn(move || {
                let mut was_pressed = false;
                loop {
                    let pressed = ptt.is_pressed();
                    if pressed && !was_pressed && wv.is_speaking() {
                        wv.shutup();
                        wi.store(true, std::sync::atomic::Ordering::Relaxed);
                    }
                    was_pressed = pressed;
                    std::thread::sleep(Duration::from_millis(30));
                }
            })
            .expect("spawn barge-in watcher");
    }
    let mut listener = if continuous {
        // Share the speech arbiter's mic gate so the listener mutes itself while
        // the copilot speaks (readbacks and monitor alerts alike).
        Some(Listener::start(config.vad_threshold, config.end_silence_ms, reporter.clone(), voice.mic_gate())?)
    } else {
        None
    };
    // Warm push-to-talk mic: kept open across presses so the device warm-up never
    // clips the start of a command. Only one of `listener`/`ptt_mic` opens the
    // device — they're mutually exclusive (continuous vs PTT).
    let ptt_mic = if ptt.is_some() { Some(PttMic::start()?) } else { None };

    reporter.log(
        ui::LogKind::Info,
        match &ptt {
            Some(Ptt::Key(_)) => format!("Ready — hold {} to talk.", config.ptt_key),
            Some(Ptt::Dataref { .. }) => "Ready — hold your bound PTT button to talk.".to_string(),
            None if !continuous => "Ready — push-to-talk (Enter to start/stop).".to_string(),
            None => "Listening — just speak commands.".to_string(),
        },
    );

    loop {
        reporter.status(ui::Phase::Listening);
        // Input is spoken (gated like before) or typed in the TUI. A typed
        // command is deliberate, so it skips the confidence/wake/domain gates and
        // goes straight to the model.
        // Confidence of a spoken utterance, applied *after* the abort check so a
        // panic word lands even if whisper wasn't sure. `None` for typed input.
        let mut spoken_confidence: Option<f32> = None;
        let (mut command, from_text) = match next_input(&mut listener, &ptt_mic, &stt, &ptt, typed.as_ref()) {
            None => continue,
            Some(Input::Typed(text)) => {
                tracing::info!(typed = %text, "typed command");
                reporter.log(ui::LogKind::Heard, format!("⌨  \"{text}\""));
                (text, true)
            }
            Some(Input::Spoken(heard)) => {
                tracing::info!(transcript = %heard.text, confidence = heard.confidence, "heard utterance");
                reporter.heard(&heard.text, heard.confidence);
                spoken_confidence = Some(heard.confidence);
                // Continuous mode acts on the transcript directly — the downstream
                // domain gate (`looks_like_command`) filters non-commands.
                (heard.text, false)
            }
        };

        // Aircraft switched? Rebuild everything profile-derived before acting on this
        // command, so the tools, limits, prompt, and checklists match what's actually
        // in the sim. (The poller swaps the shared profile; the monitor reacts on its
        // own.) The switch itself is announced by the poller.
        if live_profile.generation() != active_gen {
            active_gen = live_profile.generation();
            profile = live_profile.get();
            registry = Registry::for_aircraft(&profile);
            checklists = checklist::load(&profile).unwrap_or_default();
            stt.set_prompt(stt::build_prompt(&profile));
            specs = registry.specs();
            specs.extend(checklists.iter().map(|c| c.spec()));
            tracing::info!(aircraft = %profile.display_name, "reconfigured for new aircraft");
        }

        // EMERGENCY ABORT — the very first thing checked, before the confidence,
        // echo, and domain gates and any LLM. "belay that", "undo", "abort",
        // "scratch that", "as you were" restore what the last command changed. It
        // must beat every gate: a panic word that's low-confidence, or glued to an
        // echoed readback, still has to land. Deterministic and instant.
        if is_undo(&command) {
            match undo_stack.pop() {
                None => say(&voice, &reporter, ui::LogKind::Warn, "Nothing to undo."),
                Some(snap) => {
                    tracing::info!(undid = %snap.label, "undo");
                    let mut acts: Vec<String> = Vec::new();
                    // Recover non-value actions first (e.g. restart after a shutdown).
                    for (tname, targs) in &snap.inverse {
                        if let Some(tool) = registry.get(tname) {
                            let ctx = ToolCtx { xplane: &xplane, config: &config, voice: Some(&voice), status: reporter.status_sink(), reminders: Some(&reminders) };
                            if tool.execute(targs, &ctx).await.is_ok() {
                                acts.push(recovery_label(tname));
                            }
                        }
                    }
                    // Disengage the autopilot before resetting its dials — stop the
                    // aircraft flying the wrong thing first. servos_off_any is clean.
                    if snap.disengage_ap && xplane.run_command("sim/autopilot/servos_off_any").await.is_ok() {
                        acts.push("autopilot disengaged".to_string());
                    }
                    for (dr, label, old, is_int) in &snap.reverts {
                        let restored = if *is_int {
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
            continue;
        }

        // Confidence + echo gates (spoken input only) — now that abort has had its
        // chance. Confidence: drop what whisper wasn't sure of. Echo: drop the
        // copilot's own readback bleeding back through the speakers, so it can't
        // re-trigger itself. Both bypassed for typed input.
        if let Some(confidence) = spoken_confidence {
            if confidence < config.stt_min_confidence {
                if explicit {
                    say(&voice, &reporter, ui::LogKind::Info,
                        readback::one_of(&["Say again?", "Didn't catch that.", "One more time?"]));
                } else {
                    reporter.log(ui::LogKind::Info, "low confidence, ignored");
                }
                continue;
            }
            if voice.is_self_echo(&command) {
                reporter.log(ui::LogKind::Info, "ignored self-echo (own readback)");
                continue;
            }
        }

        // Domain gate, follow-up, and bare-value rewrite apply to SPOKEN input
        // only — phonetic nonsense and STT follow-ups are voice problems. A typed
        // command is taken as-is. Questions are let through too (the LLM decides
        // act-vs-answer), so they aren't rejected as "not a command".
        if !from_text && !looks_like_command(&command) && !looks_like_question(&command) {
            // A conversational follow-up shortly after a command is let through
            // even without a keyword. Outside the window, the strict gate holds.
            let in_followup = history
                .last()
                .is_some_and(|(_, _, t)| t.elapsed() < FOLLOWUP_WINDOW);
            if !(in_followup && looks_like_followup(&command)) {
                if explicit {
                    say(&voice, &reporter, ui::LogKind::Warn, readback::one_of(&[
                        "Sorry, that didn't sound like a command.",
                        "That didn't sound like a command.",
                        "Didn't catch a command there.",
                    ]));
                } else {
                    reporter.log(ui::LogKind::Info, format!("not a command: \"{command}\""));
                }
                continue;
            }
            // Re-anchor a *truly bare* value follow-up onto the last command's
            // system ("seventy percent" → "throttle seventy percent"). Only when
            // the utterance is nothing but a number/percent — otherwise a misheard
            // command ("fraudal at eighty percent") would get force-anchored to the
            // wrong tool. Non-bare follow-ups go to the LLM with history as-is.
            if is_bare_value(&command) {
                if let Some(subj) = &last_subject {
                    let rewritten = format!("{subj} {command}");
                    reporter.log(ui::LogKind::Info, format!("follow-up → \"{rewritten}\""));
                    command = rewritten;
                }
            }
        }

        // History is only useful for the non-numeric voice references the rewrite
        // can't handle ("switch it back to both"); a typed or named command is
        // self-contained, and stale repeated turns only confuse the model.
        let ctx_history: Vec<(String, String)> = if from_text || looks_like_command(&command) {
            Vec::new()
        } else {
            history
                .iter()
                .filter(|(_, _, t)| t.elapsed() < HISTORY_TTL)
                .map(|(u, a, _)| (u.clone(), a.clone()))
                .collect()
        };

        // The LLM arbitrates: ACT on a command (one call per distinct action, so
        // "taxi and nav lights on" makes two) or ANSWER a question in prose — a
        // polite "could you start the engines?" acts; "what's our heading?"
        // answers. A live state snapshot grounds both ("a bit more flaps", "how
        // high are we?") — but only the LLM and the correction path read it, never
        // the fast-path, so it's computed lazily below: the common fast-path
        // command pays nothing for the burst of X-Plane reads it never uses.
        reporter.status(ui::Phase::Thinking);

        // Deterministic state queries answered without the LLM — the model proved
        // unreliable here (it had the fuel in its context and still said "I don't
        // have fuel information"). A fuel question reads the datarefs and is spoken
        // straight back.
        if is_fuel_query(&command) {
            let line = fuel_report(&xplane, &profile, Some(&live)).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            continue;
        }
        // Live systems check + static reference limits — often phrased as commands,
        // not questions ("check all systems", "performance limits"), so they're
        // routed here, not behind the question gate.
        {
            let tl = command.to_ascii_lowercase();
            // Reminder memory: cancel or list what's armed ("cancel the reminders",
            // "do we have a reminder set?"). Checked before set_reminder so "cancel"
            // wins over arming.
            if tl.contains("reminder") {
                if tl.contains("cancel") || tl.contains("clear") || tl.contains("forget")
                    || tl.contains("remove") || tl.contains("delete") || tl.contains("scrap")
                    || tl.contains("never mind") || tl.contains("nevermind")
                {
                    let n = reminders.cancel_all();
                    let line = match n {
                        0 => "No reminders to cancel.".to_string(),
                        1 => "Cancelled the reminder.".to_string(),
                        _ => format!("Cancelled {n} reminders."),
                    };
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    continue;
                }
                // A mention of reminders with no arming cue is a recall ("do we have a
                // reminder?"). If it names a condition ("set a reminder to wake me at
                // 5000 feet"), fall through to arm it instead.
                let arming = tl.contains("wake") || tl.contains("descent") || tl.contains("feet")
                    || tl.contains("altitude") || tl.contains("minute") || tl.contains("waypoint")
                    || tl.contains("reach");
                if !arming {
                    let list = reminders.active();
                    let line = if list.is_empty() {
                        "No reminders set.".to_string()
                    } else if list.len() == 1 {
                        format!("One reminder, {}.", list[0])
                    } else {
                        format!("{} reminders: {}.", list.len(), list.join(", "))
                    };
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    continue;
                }
            }
            if tl.contains("all systems") || tl.contains("systems check") || tl.contains("safe parameter")
                || tl.contains("in the green") || tl.contains("status check")
            {
                let line = systems_check(&xplane).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            if let Some(line) = reference_limits(&profile, &tl) {
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            if let Some(line) = aircraft_info(&profile, &tl) {
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // "wake me / remind me / let me know / tell me when ..." is a reminder to
            // arm, not a status query to answer now. Let it fall through to the tool
            // layer so it doesn't get hijacked by the nav-progress branch below (both
            // mention "next waypoint").
            let is_reminder = tl.contains("wake") || tl.contains("remind") || tl.contains("notify")
                || tl.contains("let me know") || tl.contains("tell me when") || tl.contains("heads up");
            // Time/distance to top of descent ("how long before TOD"). Guarded by
            // is_reminder so "wake me at top of descent" still arms the reminder.
            // "top of descent" is reliably misheard ("when is top of descent" came in
            // as "1 is top descent"), so match top+descent loosely rather than the
            // exact phrase. This also keeps such an utterance out of the vertical-speed
            // grammar, which would otherwise read "descend 1 feet per minute".
            if !is_reminder
                && ((tl.contains("top") && tl.contains("descent"))
                    || tl.contains("t o d")
                    || tl.split(|c: char| !c.is_ascii_alphanumeric()).any(|w| w == "tod"))
            {
                let line = time_to_tod(&config, &xplane, &mut fp_cache).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // ETA to the destination ("time before we land", "when do we get in"):
            // distance to the destination airport over groundspeed. Needs a destination
            // cue plus a time cue, so it beats the next-waypoint branch below.
            if !is_reminder
                && (tl.contains("land") || tl.contains("destination") || tl.contains("arrive")
                    || tl.contains("arrival") || tl.contains("get there") || tl.contains("get in")
                    || tl.contains("en route") || tl.contains("enroute") || tl.contains("on route"))
                && (tl.contains("time") || tl.contains("eta") || tl.contains("when")
                    || tl.contains("how long") || tl.contains("estimat") || tl.contains("remaining"))
            {
                let line = eta_to_destination(&config, &xplane, &mut fp_cache).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // Navigation progress (often a command, not a question): ETA / distance /
            // time to the next waypoint.
            if !is_reminder
                && (tl.contains("next waypoint") || tl.contains("next fix") || tl.contains("eta")
                    || tl.contains("how far") || tl.contains("time to") || tl.contains("distance to"))
            {
                let line = nav_progress(&xplane, Some(&live)).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // "route" is reliably misheard as "root"/"road" (both meaningless in a
            // cockpit), so accept them as synonyms.
            let routeish = tl.contains("route") || tl.contains("root") || tl.contains("road");
            // Load the SimBrief route into the GPS/FMS (checked before the brief, so
            // "load the flight plan" loads rather than just reading it back).
            let wants_load = (tl.contains("load") || tl.contains("program") || tl.contains("send"))
                && (routeish || tl.contains("plan") || tl.contains("gps") || tl.contains("fms") || tl.contains("flight"));
            if wants_load {
                reporter.status(ui::Phase::Thinking);
                let refresh = tl.contains("refresh") || tl.contains("reload") || tl.contains("update");
                let line = load_route_to_fms(&config, &xplane, &mut fp_cache, refresh).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // SimBrief flight plan (cached fetch) + advice. Route, full brief, or
            // refresh. Not when the intent is to *fly* it ("follow/navigate/track"),
            // which is the navigate tool (handled via the grammar/LLM path below).
            let nav_intent = tl.contains("navigate") || tl.contains("follow") || tl.contains("track");
            let is_plan = !nav_intent
                && (tl.contains("flight plan") || tl.contains("simbrief") || tl.contains("fetch the plan")
                    || tl.contains("where are we going") || tl.contains("where are we headed")
                    || (routeish && (tl.contains("our") || tl.contains("the") || tl.contains("what"))));
            if is_plan {
                reporter.status(ui::Phase::Thinking);
                let refresh = tl.contains("refresh") || tl.contains("reload") || tl.contains("update");
                let want_route = routeish;
                let line = flight_plan_report(&config, &xplane, &mut fp_cache, refresh, want_route).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
        }
        // Other state reads ("what's our altitude/heading/airspeed") — deterministic
        // too, but only when it's a question, so "set heading 270" still commands.
        if looks_like_question(&command) {
            if let Some(answer) = answer_state_query(&command, &xplane, Some(&live)).await {
                say(&voice, &reporter, ui::LogKind::Readback, &answer);
                continue;
            }
        }

        let mut state: Option<String> = None;

        // Deterministic fast-path: resolve the common, regular commands with no
        // LLM at all — no GPU burst, no latency, and it can't pick the wrong tool.
        // Anything it can't confidently parse falls through to the LLM. Questions
        // are held back from the fast-path entirely ("is our heading 270?" shares
        // the keyword+number shape of a command, so the grammar would *act* on it);
        // they go to the LLM, which arbitrates act-vs-answer. `path` is logged so
        // `x-copilot analyze` can mine the LLM-fallback tail for new phrasings.
        let fast = if looks_like_question(&command) {
            None
        } else {
            grammar::try_parse(&command, &registry, &profile)
        };
        let calls = if let Some(c) = fast {
            tracing::info!(path = "fast", tool = c.first().map(|x| x.name.as_str()).unwrap_or(""), "command path");
            c
        } else {
            // Fast-path missed — now the LLM needs live state.
            let ts = Instant::now();
            let snap = state_snapshot(&xplane, &profile, Some(&live)).await;
            tracing::info!(ms = ts.elapsed().as_millis(), state = snap.as_deref().unwrap_or("(none)"), "state snapshot");
            let t = Instant::now();
            let decision = llm.act_or_answer(&command, &specs, &ctx_history, snap.as_deref()).await;
            tracing::info!(ms = t.elapsed().as_millis(), path = "llm", "command path");
            state = snap; // reuse for the correction path if it confirms
            match decision {
                Ok(Decision::Act(calls)) => calls,
                Ok(Decision::Reply(text)) => {
                    say(&voice, &reporter, ui::LogKind::Readback, &text);
                    continue;
                }
                Ok(Decision::None) => {
                    if explicit {
                        say(&voice, &reporter, ui::LogKind::Warn, readback::one_of(&[
                            "Sorry, I didn't catch that.",
                            "I can't action that one.",
                            "Say again?",
                        ]));
                    } else {
                        reporter.log(ui::LogKind::Info, format!("no response: \"{command}\""));
                    }
                    continue;
                }
                Err(e) => {
                    tracing::error!("LLM act-or-answer failed: {e:#}");
                    reporter.log(ui::LogKind::Warn, "LLM unreachable");
                    continue;
                }
            }
        };

        // Readbacks from this utterance's actions, recorded as the assistant
        // turn afterwards so the next command can refer back to what was done.
        let mut turn_readbacks: Vec<String> = Vec::new();
        // Recovery for everything this utterance does, gathered so one "belay that"
        // undoes the whole set of commands, not just the last tool.
        let mut turn_reverts: Vec<(String, String, f64, bool)> = Vec::new();
        let mut turn_disengage_ap = false;
        let mut turn_inverse: Vec<(String, serde_json::Value)> = Vec::new();
        for mut call in calls {
            tracing::info!(tool = %call.name, args = %call.arguments, "tool selected");
            reporter.log(ui::LogKind::Tool, format!("{} {}", call.name, call.arguments));

            // A checklist isn't a one-shot tool — the copilot runs it: reads each
            // item, does the ones it can, calls out the pilot's. Needs the voice,
            // so it's handled here, not via the plain ToolCtx. A checklist is a
            // whole phase, run only on demand — so it's never compounded: run this
            // one and ignore any other calls the LLM returned in the same turn
            // (guards against chaining through several checklists at once).
            if let Some(cl) = checklists.iter().find(|c| c.id == call.name) {
                run_checklist(cl, &xplane, &config, &mut listener, &ptt_mic, &stt, &ptt, typed.as_ref(), &voice, &reporter, &interrupt).await;
                break;
            }

            // Confirmation gate with in-line correction ("negative, 500"). Applies
            // per call, so each safety-critical action in a compound command is
            // confirmed on its own.
            let proceed = 'confirm: loop {
                let Some(tool) = registry.get(&call.name) else {
                    tracing::warn!("LLM picked unknown tool: {}", call.name);
                    say(&voice, &reporter, ui::LogKind::Warn, "That command isn't wired up yet.");
                    break 'confirm false;
                };
                let intent = if config.confirm_critical {
                    tool.confirm_prompt(&call.arguments)
                } else {
                    None
                };
                let Some(intent) = intent else { break 'confirm true };

                say(&voice, &reporter, ui::LogKind::Confirm, &format!("Confirm, {intent}?"));
                reporter.status(ui::Phase::Listening);
                let Some(reply) = listen_once(&mut listener, &ptt_mic, &stt, &ptt, None) else {
                    say(&voice, &reporter, ui::LogKind::Warn, "Cancelled, no confirmation heard.");
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
                        // A fast-path command never took the snapshot — take it now
                        // so the correction is grounded in live state.
                        if state.is_none() {
                            state = state_snapshot(&xplane, &profile, Some(&live)).await;
                        }
                        match llm.choose_tool(&format!("{command}, {correction}"), &specs, &ctx_history, state.as_deref()).await {
                            Ok(mut cs) if !cs.is_empty() => {
                                call = cs.remove(0);
                                reporter.log(ui::LogKind::Tool, format!("{} {}", call.name, call.arguments));
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
            if !proceed {
                continue;
            }

            // Execute the curated tool (writes the dataref + reads back).
            reporter.status(ui::Phase::Thinking);
            let Some(tool) = registry.get(&call.name) else { continue };
            // Snapshot the datarefs this tool is about to change, so "belay that"
            // can put them back. Captured before the write; stored only if the write
            // succeeds. Tools with no reversible dataref (engine start, pushback)
            // contribute nothing and simply can't be undone.
            let mut pre: Vec<(String, String, f64, bool)> = Vec::new();
            for (dr, label, is_int) in undoable_drefs(&call.name) {
                if let Ok(v) = xplane.get_float(dr).await {
                    pre.push((dr.to_string(), label.to_string(), v, *is_int));
                }
            }
            // If this command can switch the autopilot on and it's off right now,
            // undo should switch it back off (not leave it flying the reset dial).
            let ap_was_off = engages_autopilot(&call.name)
                && matches!(xplane.get_float(datarefs::SERVOS_ON).await, Ok(v) if v <= 0.5);
            let ctx = ToolCtx { xplane: &xplane, config: &config, voice: Some(&voice), status: reporter.status_sink(), reminders: Some(&reminders) };
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
            // Gather this action's recovery into the turn (pushed once after the loop).
            if ok {
                turn_reverts.extend(pre);
                turn_disengage_ap |= ap_was_off;
                // Actions with no value to revert get an explicit inverse command.
                if call.name == "stop_engine" {
                    turn_inverse.push(("start_engine".to_string(), serde_json::json!({})));
                }
            }
            // Vary the acknowledgment on affirmative readbacks (bare callout most
            // of the time, "Roger,"/"Copy," occasionally); the canonical readback
            // is what's recorded for history context.
            say(&voice, &reporter, ui::LogKind::Readback, &readback::vary_ack(&readback));
            turn_readbacks.push(readback);
        }

        // Push this utterance's recovery onto the undo stack (newest on top), bounded.
        if !turn_reverts.is_empty() || turn_disengage_ap || !turn_inverse.is_empty() {
            undo_stack.push(UndoSnapshot {
                label: command.clone(),
                reverts: turn_reverts,
                disengage_ap: turn_disengage_ap,
                inverse: turn_inverse,
            });
            if undo_stack.len() > UNDO_STACK_MAX {
                undo_stack.remove(0);
            }
        }

        // Record this turn for back-reference context, then bound the history by
        // count and age so stale state can't accumulate.
        if !turn_readbacks.is_empty() {
            // Anchor for the next bare-value follow-up: the subject of this
            // command if it named a system (covers rewritten follow-ups too,
            // which are explicit by now, so a chain 80 → 70 → 60 stays anchored).
            if looks_like_command(&command) {
                let subj = command_subject(&command);
                if !subj.is_empty() {
                    last_subject = Some(subj);
                }
            }
            history.push((command.clone(), turn_readbacks.join(" "), Instant::now()));
            let excess = history.len().saturating_sub(HISTORY_MAX);
            if excess > 0 {
                history.drain(0..excess);
            }
            history.retain(|(_, _, t)| t.elapsed() < HISTORY_TTL);
        }
    }
}

/// Run a checklist the way a pilot-monitoring would: read each item aloud,
/// perform the ones the copilot can (each tool reads the dataref back, so the
/// action is verified), and call out the pilot's items. Mostly autonomous — it
/// performs the flow top to bottom, but **pauses to confirm destructive steps**
/// (engine start/stop, the run-up). Narration is naturally paced by the speech
/// queue (one utterance plays before the next).
///
/// Abortable: press PTT at any point (the barge-in watcher stops it between
/// items), say "stop"/"abort"/"cancel" at any confirm prompt, or type it in the
/// TUI — it stops where it is, leaving what's already done.
#[allow(clippy::too_many_arguments)]
async fn run_checklist(
    cl: &checklist::Checklist,
    xplane: &XPlaneClient,
    config: &Config,
    listener: &mut Option<Listener>,
    ptt_mic: &Option<PttMic>,
    stt: &Stt,
    ptt: &Option<Ptt>,
    typed: Option<&std::sync::mpsc::Receiver<String>>,
    voice: &Voice,
    reporter: &Reporter,
    interrupt: &std::sync::atomic::AtomicBool,
) {
    use checklist::Item;
    use std::sync::atomic::Ordering;
    // Start clean — only a press *during* this run should stop it.
    interrupt.store(false, Ordering::Relaxed);
    say(voice, reporter, ui::LogKind::Info, &format!("Running the {} checklist.", cl.title));
    let ctx = ToolCtx { xplane, config, voice: Some(voice), status: reporter.status_sink(), reminders: None };
    let mut failed = 0usize;
    let mut aborted = false;
    for item in &cl.items {
        // Barge-in (PTT pressed during the run) stops the checklist where it is.
        if interrupt.swap(false, Ordering::Relaxed) {
            aborted = true;
            break;
        }
        // Typed abort (TUI): drain anything queued since the last item.
        if let Some(rx) = typed {
            while let Ok(msg) = rx.try_recv() {
                if is_abort(&msg) {
                    aborted = true;
                }
            }
            if aborted {
                break;
            }
        }
        match item {
            // Spoken context (remarks, reference lines).
            Item::Narrate(text) => say(voice, reporter, ui::LogKind::Info, text),
            // The pilot's item — call it out, don't perform it.
            Item::CallOut(text) => say(voice, reporter, ui::LogKind::Confirm, text),
            // The copilot's item — do it, then read the item back (or flag it).
            Item::Act { say: line, tool, args, confirm } => {
                // Destructive step — ask first. Yes does it; "stop/abort/cancel"
                // aborts the whole checklist; anything else just skips this step.
                // Exception: mixture-to-cutoff only stops a *running* engine, so
                // pre-start (the SR22's "mixture lean" before-start item) it's the
                // normal lean — skip the prompt and just do it.
                let ask = match confirm {
                    Some(h) if *h == checklist::CONFIRM_MIXTURE_CUTOFF => {
                        checklist::engine_running(xplane).await
                    }
                    other => other.is_some(),
                };
                if ask {
                    let hazard = confirm.expect("ask implies a confirm hazard");
                    say(voice, reporter, ui::LogKind::Confirm, &format!("Confirm, {hazard}?"));
                    reporter.status(ui::Phase::Listening);
                    let reply = listen_once(listener, ptt_mic, stt, ptt, None);
                    match reply {
                        Some(r) if is_affirmative(&r.text) => {} // proceed
                        Some(r) if is_abort(&r.text) => {
                            aborted = true;
                            break;
                        }
                        _ => {
                            say(voice, reporter, ui::LogKind::Info, &format!("{line} Skipped — your call."));
                            continue;
                        }
                    }
                }
                if checklist::run_step(tool, args, &cl.lights, &cl.flaps, &cl.electrical, &cl.fuel_selector, &ctx).await.is_ok() {
                    say(voice, reporter, ui::LogKind::Readback, line);
                } else {
                    say(voice, reporter, ui::LogKind::Warn, &format!("{line} Didn't take — check it."));
                    failed += 1;
                }
            }
            // A dataref we don't actuate — read it and confirm or flag.
            Item::Verify { say: line, dataref, index, cond } => {
                match checklist::read_dataref(xplane, dataref, *index).await {
                    Some(v) if cond.eval(v) => {
                        say(voice, reporter, ui::LogKind::Readback, &format!("{line} Confirmed."))
                    }
                    Some(_) => {
                        say(voice, reporter, ui::LogKind::Warn, &format!("{line} Not set — check it."));
                        failed += 1;
                    }
                    // Couldn't read it — just call it out, don't flag.
                    None => say(voice, reporter, ui::LogKind::Confirm, line),
                }
            }
        }
    }
    let summary = if aborted {
        format!("{} checklist aborted — stopped where we were.", cl.title)
    } else if failed == 0 {
        format!("{} checklist complete.", cl.title)
    } else {
        format!(
            "{} checklist complete — {failed} item{} didn't take, check the panel.",
            cl.title,
            if failed == 1 { "" } else { "s" }
        )
    };
    say(voice, reporter, ui::LogKind::Ok, &summary);
}

/// Is a reply an abort — "stop", "abort", "cancel" the whole checklist (distinct
/// from a plain "no" at a step, which just skips that one item)?
fn is_abort(reply: &str) -> bool {
    let r = reply.to_lowercase();
    ["abort", "stop", "cancel", "belay", "disregard", "halt", "quit"]
        .iter()
        .any(|w| r.contains(w))
}

/// Standard Levenshtein edit distance.
pub(crate) fn levenshtein(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    let mut cur = vec![0usize; b.len() + 1];
    for i in 1..=a.len() {
        cur[0] = i;
        for j in 1..=b.len() {
            let cost = if a[i - 1] == b[j - 1] { 0 } else { 1 };
            cur[j] = (prev[j] + 1).min(cur[j - 1] + 1).min(prev[j - 1] + cost);
        }
        std::mem::swap(&mut prev, &mut cur);
    }
    prev[b.len()]
}

/// Transcribe an audio file with whisper, for `x-copilot transcribe <file>`.
/// Decodes/resamples to 16 kHz mono via macOS `afconvert` (handles mp3/m4a/wav/…
/// — whisper needs 16 kHz mono), reads the PCM, and returns the text. Reuses the
/// same STT model the live loop uses, so no extra model download.
/// Decode any audio file to 16 kHz mono f32 samples via `afconvert` (macOS).
fn load_wav_16k(input: &str) -> Result<Vec<f32>> {
    use std::process::Command;
    if !std::path::Path::new(input).exists() {
        return Err(anyhow!("no such file: {input}"));
    }
    let wav = std::env::temp_dir().join("x-copilot-16k.wav");
    let status = Command::new("afconvert")
        .args([input, wav.to_str().unwrap(), "-d", "LEI16@16000", "-c", "1", "-f", "WAVE"])
        .status()
        .map_err(|e| anyhow!("spawning afconvert: {e}"))?;
    if !status.success() {
        return Err(anyhow!("afconvert failed to decode {input}"));
    }
    let reader = hound::WavReader::open(&wav).map_err(|e| anyhow!("reading {}: {e}", wav.display()))?;
    reader
        .into_samples::<i16>()
        .map(|s| s.map(|v| v as f32 / i16::MAX as f32))
        .collect::<std::result::Result<_, _>>()
        .map_err(|e| anyhow!("decoding samples: {e}"))
}

fn transcribe_file(model_path: &str, input: &str) -> Result<String> {
    let samples = load_wav_16k(input)?;
    // Offline one-shot — GPU is fine here (not competing with the sim loop).
    let stt = Stt::load(model_path, true)?;
    // Verbatim: a voice-clone reference caption must match the audio's actual
    // words, so don't apply the command-loop's digit/mishear normalization
    // ("two seven zero" must stay words, not become "270").
    Ok(stt.transcribe_verbatim(&samples)?.text.trim().to_string())
}

/// Mine the logs for fast-path coverage and the LLM-fallback tail. Pairs each
/// spoken transcript with the `command path` line that followed it, then reports
/// the fast/LLM split and the most frequent LLM-fallback phrasings — the ones
/// worth promoting into [`grammar`] next. The ratchet that makes the copilot
/// cheaper and more deterministic the more it's flown.
fn analyze_commands() -> Result<()> {
    use std::collections::HashMap;
    let mut last: Option<String> = None;
    let (mut fast, mut llm) = (0u32, 0u32);
    let mut tail: HashMap<String, u32> = HashMap::new();

    let mut paths: Vec<std::path::PathBuf> = std::fs::read_dir("logs")
        .map(|d| d.filter_map(|e| e.ok().map(|e| e.path())).collect())
        .unwrap_or_default();
    paths.sort();
    for p in paths {
        let is_log = p.file_name().and_then(|n| n.to_str()).is_some_and(|n| n.starts_with("x-copilot.log"));
        if !is_log {
            continue;
        }
        let Ok(content) = std::fs::read_to_string(&p) else { continue };
        for line in content.lines() {
            if let Some(i) = line.find("transcript=") {
                let rest = &line[i + "transcript=".len()..];
                last = Some(rest.split(" confidence=").next().unwrap_or(rest).trim().to_lowercase());
            } else if line.contains("⌨") {
                if let (Some(a), Some(b)) = (line.find('"'), line.rfind('"')) {
                    if b > a {
                        last = Some(line[a + 1..b].trim().to_lowercase());
                    }
                }
            } else if line.contains("command path") {
                if line.contains("path=\"fast\"") {
                    fast += 1;
                } else if line.contains("path=\"llm\"") {
                    llm += 1;
                    if let Some(t) = &last {
                        *tail.entry(t.clone()).or_default() += 1;
                    }
                }
            }
        }
    }

    let total = fast + llm;
    if total == 0 {
        println!("No command-path data in logs yet — run the app and issue some commands first.");
        return Ok(());
    }
    println!("commands analyzed: {total}");
    println!("  fast-path (no LLM): {fast:>5}  ({}%)", fast * 100 / total);
    println!("  LLM fallback:       {llm:>5}  ({}%)", llm * 100 / total);
    println!("\npromotion candidates — frequent LLM-fallback phrasings to add to grammar.rs:");
    let mut v: Vec<(String, u32)> = tail.into_iter().collect();
    v.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    for (t, n) in v.into_iter().take(20) {
        println!("  {n:>3}  {t}");
    }
    Ok(())
}

/// Map an X-Plane ICAO type code to a profile id. Unknown types fall back to
/// the Cessna so the app stays usable; add rows as profiles are added.
fn icao_to_profile(icao: &str) -> String {
    match icao.to_uppercase().as_str() {
        "C172" => "cessna_172sp",
        "SR22" => "cirrus_sr22",
        "MD82" | "MD80" | "MD83" | "MD88" => "md82",
        _ => "cessna_172sp",
    }
    .to_string()
}

/// Capture one utterance (continuous VAD or push-to-talk) and transcribe it.
/// Returns `None` on silence/noise or a capture error.
fn listen_once(
    listener: &mut Option<Listener>,
    ptt_mic: &Option<PttMic>,
    stt: &Stt,
    ptt: &Option<Ptt>,
    idle_timeout: Option<Duration>,
) -> Option<stt::Transcription> {
    let audio = match (ptt, ptt_mic, listener) {
        // Hold-a-key push-to-talk (works while X-Plane has focus), warm mic.
        (Some(Ptt::Key(key)), Some(mic), _) => {
            use device_query::DeviceQuery;
            let dev = device_query::DeviceState::new();
            let key = *key;
            mic.capture(|| dev.get_keys().contains(&key), idle_timeout).ok()?
        }
        // Sim-dataref push-to-talk (the X-Plane plugin's bound button), warm mic.
        (Some(Ptt::Dataref { live, name }), Some(mic), _) => {
            mic.capture(|| Ptt::dataref_active(live, name), idle_timeout).ok()?
        }
        // Continuous, VAD-segmented.
        (None, _, Some(l)) => l.next_utterance(idle_timeout).ok()?,
        // Enter-to-talk.
        (None, _, None) => audio::record_push_to_talk().ok()?,
        // PTT configured but the warm mic failed to open (start() would have
        // errored earlier) — nothing to capture.
        (Some(_), None, _) => return None,
    };
    if audio.is_empty() {
        return None; // idle timeout (continuous) or empty capture
    }
    match stt.transcribe(&audio) {
        Ok(t) if !t.text.is_empty() => Some(t),
        _ => None,
    }
}

/// One pilot input — spoken (a transcript) or typed in the TUI.
enum Input {
    Spoken(stt::Transcription),
    Typed(String),
}

/// Block until the next input arrives from either source. With a `typed`
/// channel, the mic listen times out periodically so we can poll for typed
/// commands without one starving the other; without it, just listen.
fn next_input(
    listener: &mut Option<Listener>,
    ptt_mic: &Option<PttMic>,
    stt: &Stt,
    ptt: &Option<Ptt>,
    typed: Option<&std::sync::mpsc::Receiver<String>>,
) -> Option<Input> {
    let Some(typed) = typed else {
        return listen_once(listener, ptt_mic, stt, ptt, None).map(Input::Spoken);
    };
    loop {
        if let Ok(t) = typed.try_recv() {
            let t = t.trim().to_string();
            if !t.is_empty() {
                return Some(Input::Typed(t));
            }
        }
        if let Some(t) = listen_once(listener, ptt_mic, stt, ptt, Some(Duration::from_millis(150))) {
            return Some(Input::Spoken(t));
        }
        // Timed out idle (or a discarded blip) — loop to re-check the typed channel.
    }
}

/// Negation words — a reply containing any of these is never an affirmative.
const NEGATIONS: &[&str] = &[
    "negative", "cancel", "abort", "stop", "belay", "disregard", "wrong", "no",
];

/// Is a spoken reply an affirmative confirmation? Defaults to `false` (safe) —
/// only an explicit yes proceeds; an explicit no, or anything unclear, cancels.
fn is_affirmative(reply: &str) -> bool {
    let r = reply.to_lowercase();
    const YES: &[&str] = &[
        "affirm", "affirmative", "confirm", "yes", "yeah", "yep", "roger", "correct",
        "copy", "proceed", "go ahead", "do it", "right", "indeed", "okay", "absolutely",
    ];
    if NEGATIONS.iter().any(|w| r.contains(w)) {
        return false;
    }
    YES.iter().any(|w| r.contains(w))
}

/// If a non-affirmative reply still carries content beyond negation words,
/// return that content as a correction ("negative, 500" → "500"). A bare
/// "negative" returns `None` — a plain cancel.
fn correction_text(reply: &str) -> Option<String> {
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

/// Report a line and speak it through the arbiter, verbatim. Blocks until the
/// line has been spoken (so the loop doesn't listen over its own voice), setting
/// the Speaking→Listening status around it. Acknowledgment variation is NOT
/// applied here — only curated tool readbacks get that, via [`readback::vary_ack`]
/// at the call site, so it never touches LLM free-text answers, refusals, or
/// prompts (an answer that happened to start with "Roger" would lose its first
/// word otherwise).
fn say(voice: &Voice, reporter: &Reporter, kind: ui::LogKind, line: &str) {
    tracing::info!(spoke = %line, "spoke");
    reporter.status(ui::Phase::Speaking);
    reporter.log(kind, line);
    voice.say(Priority::Readback, line);
    reporter.status(ui::Phase::Listening);
}

/// The datarefs the live-state cache subscribes to over the WebSocket — the
/// continuously-read snapshot + autopilot-status set. (Arrays like engine-running,
/// and the tool path's read-after-write confirms, stay on REST.)
const SNAPSHOT_DATAREFS: &[&str] = &[
    datarefs::ALTITUDE_FT, datarefs::AIRSPEED_KTS, datarefs::VVI_FPM,
    datarefs::COMPASS_HEADING_DEG, datarefs::FLAP_RATIO, datarefs::GEAR_HANDLE_DOWN,
    datarefs::ON_GROUND_ANY, datarefs::FUEL_TOTAL_KG, datarefs::WIND_DIR_DEG_TRUE,
    datarefs::WIND_SPEED_MSC, datarefs::OAT_DEG_C, datarefs::VISIBILITY_SM,
    datarefs::QNH_PASCALS, datarefs::SERVOS_ON, datarefs::APPROACH_STATUS,
    datarefs::NAV_STATUS, datarefs::HEADING_STATUS,
];

/// Read a dataref preferring the live-state cache (instant, no round-trip), with a
/// REST read as the fallback on a cache miss — so it's never worse than polling
/// and near-free once the WebSocket subscription is warm.
async fn read1(live: Option<&LiveState>, xp: &XPlaneClient, name: &str) -> Result<f64> {
    if let Some(v) = live.and_then(|l| l.get(name)) {
        return Ok(v);
    }
    xp.get_float(name).await
}

/// 100LL avgas ≈ 2.72 kg/gal (~6 lb/gal) — to report fuel in gallons.
const AVGAS_KG_PER_GAL: f64 = 2.72;

/// Meters per second → knots.
const MS_TO_KTS: f64 = 1.943_844;

/// The active GPS waypoint with distance and time/ETA to it — answers "ETA",
/// "distance/time to next waypoint", "how far". Often phrased as a command, not a
/// question, so it's dispatched unconditionally rather than via answer_state_query.
async fn nav_progress(xp: &XPlaneClient, live: Option<&LiveState>) -> String {
    let wp = xp.get_string("sim/cockpit2/radios/indicators/gps_nav_id").await.unwrap_or_default();
    let dist = read1(live, xp, "sim/cockpit2/radios/indicators/gps_dme_distance_nm").await.ok();
    let tmin = read1(live, xp, "sim/cockpit2/radios/indicators/gps_dme_time_min").await.ok();
    if wp.trim().is_empty() && dist.is_none() {
        return "No active GPS waypoint. Load and navigate a route first.".to_string();
    }
    let mut s = String::new();
    if !wp.trim().is_empty() {
        s.push_str(&format!("Next waypoint {}, ", wp.trim()));
    }
    if let Some(d) = dist {
        s.push_str(&format!("{d:.0} miles", ));
    }
    if let Some(tm) = tmin {
        let (h, mn) = ((tm / 60.0) as i32, (tm % 60.0).round() as i32);
        let time = if h > 0 { format!("{h} hours {mn} minutes") } else { format!("{mn} minutes") };
        s.push_str(&format!(", {time}"));
    }
    format!("{}.", s.trim_end_matches([',', ' ']))
}

/// A 3-degree descent loses ~1000 ft every 3 nm; Stratos (and most pilots) use a
/// slightly gentler ~3.5 nm per 1000 ft as the planning ring. Used only when
/// neither the FMS nor the SimBrief navlog gives a top-of-descent directly.
pub(crate) const TOD_NM_PER_1000FT: f64 = 3.5;

/// Time and distance to the top of descent ("how long before TOD"), with three
/// fallbacks so it works whether or not the box has VNAV:
///   1. the FMS's own computed TOD distance (when the avionics publish one);
///   2. SimBrief's planned TOD ring (`tod_to_dest_nm`) measured back from the
///      destination, which most GA boxes never compute themselves;
///   3. the altitude rule of thumb, cruise_alt/1000 * 3.5 nm, as a last resort.
async fn time_to_tod(
    config: &Config,
    xp: &XPlaneClient,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
) -> String {
    const TOD_DIST: &str = "sim/cockpit2/radios/indicators/fms_distance_to_tod_pilot";
    // 1. The avionics' own VNAV TOD, if it computes one.
    let fms = xp.get_float(TOD_DIST).await.unwrap_or(0.0);
    if fms > 0.0 {
        return tod_phrase(fms, xp).await;
    }
    // 2/3. Derive it from the flight plan: the SimBrief TOD ring, else the formula.
    if let Err(msg) = ensure_plan(config, cache, false).await {
        return msg;
    }
    let Some((_, fp)) = cache.as_ref() else {
        return "Couldn't fetch your SimBrief plan.".to_string();
    };
    let Some(dest) = fp.waypoints.last().cloned() else {
        return "No destination in the plan.".to_string();
    };
    let ring_nm = if fp.tod_to_dest_nm > 0.0 {
        fp.tod_to_dest_nm
    } else {
        fp.cruise_alt_ft / 1000.0 * TOD_NM_PER_1000FT
    };
    let (lat, lon) = match (
        xp.get_float("sim/flightmodel/position/latitude").await,
        xp.get_float("sim/flightmodel/position/longitude").await,
    ) {
        (Ok(a), Ok(b)) => (a, b),
        _ => return "I can't read our position right now.".to_string(),
    };
    let to_tod = haversine_nm(lat, lon, dest.lat, dest.lon) - ring_nm;
    tod_phrase(to_tod, xp).await
}

/// Format a distance-to-TOD as a spoken line, paced by groundspeed.
async fn tod_phrase(dist_nm: f64, xp: &XPlaneClient) -> String {
    if dist_nm <= 0.0 {
        return "We're at top of descent, time to start down.".to_string();
    }
    let gs_kts = xp.get_float("sim/flightmodel/position/groundspeed").await.unwrap_or(0.0) * MS_TO_KTS;
    if gs_kts < 30.0 {
        return format!("Top of descent about {dist_nm:.0} miles ahead.");
    }
    let mins = (dist_nm / gs_kts * 60.0).round() as i64;
    let time = if mins >= 60 {
        format!("{} hours {} minutes", mins / 60, mins % 60)
    } else {
        format!("{mins} minutes")
    };
    format!("About {time} to top of descent, {dist_nm:.0} miles ahead.")
}

/// Great-circle distance in nautical miles between two lat/lon points (haversine).
pub(crate) fn haversine_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const EARTH_RADIUS_NM: f64 = 3440.065;
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let a = (dlat / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_NM * a.sqrt().asin()
}

/// Estimated time to the destination ("time before we land", "when do we get in"):
/// great-circle distance from our position to the destination airport over current
/// groundspeed. Needs the SimBrief plan (for the destination coordinates) and us
/// actually moving — a straight-line estimate, not the routed distance.
async fn eta_to_destination(
    config: &Config,
    xp: &XPlaneClient,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
) -> String {
    if let Err(msg) = ensure_plan(config, cache, false).await {
        return msg;
    }
    let Some((_, fp)) = cache.as_ref() else {
        return "Couldn't fetch your SimBrief plan.".to_string();
    };
    let Some(dest) = fp.waypoints.last().cloned() else {
        return "No destination in the plan.".to_string();
    };
    let dest_name = if fp.destination.is_empty() { dest.ident.clone() } else { fp.destination.clone() };
    let (lat, lon, gs_ms) = match (
        xp.get_float("sim/flightmodel/position/latitude").await,
        xp.get_float("sim/flightmodel/position/longitude").await,
        xp.get_float("sim/flightmodel/position/groundspeed").await,
    ) {
        (Ok(a), Ok(b), Ok(c)) => (a, b, c),
        _ => return "I can't read our position right now.".to_string(),
    };
    let dist = haversine_nm(lat, lon, dest.lat, dest.lon);
    let gs_kts = gs_ms * MS_TO_KTS;
    if gs_kts < 30.0 {
        return format!(
            "{dest_name} is {dist:.0} miles out. I'll have a time once we're up and moving."
        );
    }
    let mins = (dist / gs_kts * 60.0).round() as i64;
    let (h, m) = (mins / 60, mins % 60);
    let eta = if h > 0 { format!("{h} hours {m} minutes") } else { format!("{m} minutes") };
    format!("About {eta} to {dest_name}, {dist:.0} miles out at {gs_kts:.0} knots.")
}

/// First element of a per-engine array dataref (engine 1), or `None` on a failed
/// read. Engine indicators are arrays, so they bypass the scalar `read1`/cache.
async fn read_engine0(xp: &XPlaneClient, name: &str) -> Option<f64> {
    xp.get_float_array(name).await.ok().and_then(|v| v.first().copied())
}

/// Live "are all systems within safe parameters?" check — reads the engine,
/// vacuum, and electrical instruments and reports status against the same limits
/// the monitor uses, instead of reciting static POH numbers. (oil temp / CHT
/// datarefs read °F despite their names.) The deterministic answer to "check all
/// systems".
async fn systems_check(xp: &XPlaneClient) -> String {
    let running = read_engine0(xp, "sim/flightmodel/engine/ENGN_running").await.is_some_and(|v| v > 0.5);
    if !running {
        return "Engine's not running, nothing to check.".to_string();
    }
    let mut ok: Vec<String> = Vec::new();
    let mut flags: Vec<String> = Vec::new();
    if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_pressure_psi").await {
        if (20.0..=105.0).contains(&v) { ok.push(format!("oil pressure {v:.0}")); }
        else { flags.push(format!("oil pressure {v:.0} pounds")); }
    }
    if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_temperature_deg_C").await {
        if v < 240.0 { ok.push(format!("oil temp {v:.0}")); } else { flags.push(format!("oil temp {v:.0} degrees")); }
    }
    if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/CHT_deg_C").await {
        if v < 450.0 { ok.push(format!("C H T {v:.0}")); } else { flags.push(format!("C H T {v:.0} degrees")); }
    }
    if let Ok(v) = xp.get_float("sim/cockpit/misc/vacuum").await {
        if v >= 4.0 { ok.push(format!("vacuum {v:.1}")); } else { flags.push(format!("low vacuum {v:.1}")); }
    }
    if let Some(v) = read_engine0(xp, "sim/cockpit2/electrical/battery_amps").await {
        if v >= -5.0 { ok.push("charging".to_string()); } else { flags.push(format!("battery discharging {:.0} amps", v.abs())); }
    }
    if flags.is_empty() {
        format!("All systems in the green. {}.", ok.join(", "))
    } else {
        format!("Heads up, {}. Otherwise {}.", flags.join(", "), ok.join(", "))
    }
}

/// How long a fetched SimBrief plan is reused before re-fetching (it doesn't
/// change mid-flight). "refresh"/"reload" forces a re-fetch.
const FLIGHT_PLAN_TTL: Duration = Duration::from_secs(600);

/// Make sure `cache` holds a usable plan: reuse it unless stale or `refresh`, and
/// on a failed re-fetch keep the cached one. `Err` carries a spoken message.
async fn ensure_plan(
    config: &Config,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
    refresh: bool,
) -> Result<(), String> {
    if config.simbrief_id.trim().is_empty() {
        return Err("No SimBrief ID set. Add XCOPILOT_SIMBRIEF_ID and I can pull your flight plan.".to_string());
    }
    let fresh = cache.as_ref().is_some_and(|(t, _)| t.elapsed() < FLIGHT_PLAN_TTL);
    if refresh || !fresh {
        match simbrief::fetch(&config.simbrief_id).await {
            Ok(fp) => *cache = Some((Instant::now(), fp)),
            Err(e) => {
                tracing::warn!("simbrief fetch failed: {e:#}");
                if cache.is_none() {
                    return Err("Couldn't fetch your SimBrief plan. Generate one first, or check the ID.".to_string());
                }
            }
        }
    }
    Ok(())
}

/// Load the SimBrief route into the GPS/FMS: hand the waypoints to the plugin
/// (only it can touch the FMS, via XPLMNavigation) and trigger the load.
async fn load_route_to_fms(
    config: &Config,
    xp: &XPlaneClient,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
    refresh: bool,
) -> String {
    if let Err(msg) = ensure_plan(config, cache, refresh).await {
        return msg;
    }
    let Some((_, fp)) = cache.as_ref() else {
        return "Couldn't fetch your SimBrief plan.".to_string();
    };
    if fp.waypoints.is_empty() {
        return "No waypoints in the plan to load.".to_string();
    }
    // "IDENT,lat,lon,alt|..." — the plugin parses this and builds the FMS route.
    let payload = fp
        .waypoints
        .iter()
        .map(|w| format!("{},{:.4},{:.4},{}", w.ident, w.lat, w.lon, w.alt as i64))
        .collect::<Vec<_>>()
        .join("|");
    let (origin, dest, n) = (fp.origin.clone(), fp.destination.clone(), fp.waypoints.len());
    if xp.set_data("xcopilot/fms/route", payload.as_bytes()).await.is_err()
        || xp.set_int("xcopilot/fms/load", 1).await.is_err()
    {
        return "Couldn't reach the plugin to load the route. Is the X-COPILOT plugin running?".to_string();
    }
    format!("Loading {n} waypoints into the GPS, {origin} to {dest}. Check the flight plan page.")
}

/// Fetch (or reuse) the SimBrief flight plan and brief it. Deterministic (no LLM);
/// the one network call beyond X-Plane. `want_route` reads back the airway route;
/// otherwise it's a full brief: route summary, climb-to-cruise, top of descent,
/// and a fuel check against what's actually on board.
async fn flight_plan_report(
    config: &Config,
    xp: &XPlaneClient,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
    refresh: bool,
    want_route: bool,
) -> String {
    if let Err(msg) = ensure_plan(config, cache, refresh).await {
        return msg;
    }
    let Some((_, fp)) = cache.as_ref() else {
        return "Couldn't fetch your SimBrief plan.".to_string();
    };

    if want_route {
        return if fp.route.is_empty() {
            "No route in the plan.".to_string()
        } else {
            format!("Route, {} {} {}.", fp.origin, fp.route, fp.destination)
        };
    }

    let mut line = fp.summary();
    // Where we are vs planned cruise.
    if let Ok(alt) = xp.get_float(datarefs::ALTITUDE_FT).await {
        let diff = fp.cruise_alt_ft - alt;
        if diff > 300.0 {
            line.push_str(&format!(" You're at {alt:.0}, {diff:.0} to climb to cruise."));
        } else if diff < -300.0 {
            line.push_str(&format!(" You're at {alt:.0}, above planned cruise."));
        } else {
            line.push_str(" At planned cruise.");
        }
    }
    // Top of descent.
    if fp.tod_to_dest_nm > 0.5 {
        line.push_str(&format!(" Top of descent about {:.0} miles before {}.", fp.tod_to_dest_nm, fp.destination));
    }
    // Fuel check: planned block vs what's in the tanks (read in kg, report gallons).
    if let Ok(kg) = xp.get_float(datarefs::FUEL_TOTAL_KG).await {
        let aboard = kg / AVGAS_KG_PER_GAL;
        if aboard + 0.5 >= fp.block_fuel_gal {
            line.push_str(&format!(" You have {aboard:.0} gallons aboard, enough for the plan."));
        } else {
            line.push_str(&format!(
                " You have {aboard:.0} aboard, but the plan needs {:.0}; short about {:.0}.",
                fp.block_fuel_gal,
                fp.block_fuel_gal - aboard
            ));
        }
    }
    line
}

/// A brief spoken answer to "what are we flying / tell me about this aircraft",
/// from the profile + the 172 reference figures, instead of the LLM declining for
/// lack of data. `None` if the utterance isn't asking about the aircraft.
fn aircraft_info(profile: &aircraft::AircraftProfile, t: &str) -> Option<String> {
    let asks = t.contains("what are we flying")
        || t.contains("what aircraft")
        || t.contains("what plane")
        || t.contains("what are we in")
        || ((t.contains("tell me about") || t.contains("describe") || t.contains("about the"))
            && (t.contains("aircraft") || t.contains("plane") || t.contains("cessna")
                || t.contains("172") || t.contains("skyhawk")));
    if !asks {
        return None;
    }
    let mut line = format!("We're in the {}, a single-engine aircraft.", profile.display_name);
    let r = profile.reference.as_ref();
    if let Some(v) = r.and_then(|r| r.cruise_kts) {
        line.push_str(&format!(" Cruise about {v:.0} knots."));
    }
    if let Some(v) = profile.vne_kts {
        line.push_str(&format!(" Vne {v:.0}."));
    }
    if let Some(v) = r.and_then(|r| r.service_ceiling_ft) {
        line.push_str(&format!(" Service ceiling {} feet.", with_commas(v)));
    }
    if let (Some(g), Some(h)) = (
        r.and_then(|r| r.fuel_useable_gal),
        r.and_then(|r| r.endurance_hours),
    ) {
        line.push_str(&format!(" {g:.0} gallons useable for roughly {h:.0} hours.", ));
    }
    Some(line)
}

/// Static reference figures (V-speeds, weights, limits) answered as a spoken query
/// — the old "performance limits" checklist, now a question. Every figure comes
/// from the airframe profile ([`aircraft::ReferenceProfile`]), so it's per-aircraft
/// and not hardcoded. `None` if the profile has no figures or the utterance asks
/// about none of them.
fn reference_limits(profile: &aircraft::AircraftProfile, t: &str) -> Option<String> {
    let r = profile.reference.as_ref();
    let mut parts: Vec<String> = Vec::new();
    let kt = |v: f64| format!("{v:.0} knots");

    if t.contains("vne") || t.contains("never exceed") {
        if let Some(v) = profile.vne_kts {
            parts.push(format!("Vne {}", kt(v)));
        }
    }
    if t.contains("vno") || t.contains("max cruise") || t.contains("structural cruise") {
        if let Some(v) = r.and_then(|r| r.vno_kts) {
            parts.push(format!("Vno {}", kt(v)));
        }
    }
    if t.contains("maneuver") {
        if let Some(v) = r.and_then(|r| r.va_kts) {
            parts.push(format!("Va, maneuvering speed, {}", kt(v)));
        }
    }
    if t.contains("vfe") || t.contains("flap speed") || t.contains("flaps speed") || t.contains("flap limit") {
        if let Some(p) = vfe_phrase(profile) {
            parts.push(p);
        }
    }
    if t.contains("stall") {
        match (profile.vs_clean_kts, r.and_then(|r| r.vs0_kts)) {
            (Some(c), Some(f)) => parts.push(format!("stall {c:.0} knots clean, {f:.0} with full flaps")),
            (Some(c), None) => parts.push(format!("stall {c:.0} knots clean")),
            (None, Some(f)) => parts.push(format!("stall {f:.0} knots with full flaps")),
            (None, None) => {}
        }
    }
    if t.contains("vx") || t.contains("best angle") {
        if let Some(v) = r.and_then(|r| r.vx_kts) {
            parts.push(format!("Vx, best angle of climb, {}", kt(v)));
        }
    }
    if t.contains("vy") || t.contains("best rate") || t.contains("best climb") {
        if let Some(v) = r.and_then(|r| r.vy_kts) {
            parts.push(format!("Vy, best rate of climb, {}", kt(v)));
        }
    }
    if t.contains("rotat") || t.contains("liftoff") || t.contains("lift off") || t.contains("takeoff speed") {
        if let Some(v) = r.and_then(|r| r.vr_kts) {
            parts.push(format!("rotate at {}", kt(v)));
        }
    }
    if t.contains("glide") {
        if let Some(v) = r.and_then(|r| r.best_glide_kts) {
            parts.push(format!("best glide {}", kt(v)));
        }
    }
    if t.contains("approach") || t.contains("landing speed") || t.contains("final approach") {
        if let Some(v) = r.and_then(|r| r.approach_kts) {
            parts.push(format!("approach around {}", kt(v)));
        }
    }
    if t.contains("crosswind") {
        if let Some(v) = r.and_then(|r| r.max_crosswind_kts) {
            parts.push(format!("max demonstrated crosswind {}", kt(v)));
        }
    }
    if t.contains("rate of climb") || t.contains("climb rate") {
        if let Some(v) = r.and_then(|r| r.best_climb_fpm) {
            parts.push(format!("best rate of climb about {v:.0} feet per minute"));
        }
    }
    if t.contains("gross weight") || t.contains("max weight") || t.contains("max gross") {
        if let Some(v) = r.and_then(|r| r.max_gross_lb) {
            parts.push(format!("max gross weight {} pounds", with_commas(v)));
        }
    }
    if t.contains("useful load") {
        if let Some(v) = r.and_then(|r| r.useful_load_lb) {
            parts.push(format!("useful load {} pounds", with_commas(v)));
        }
    }
    if t.contains("empty weight") {
        if let Some(v) = r.and_then(|r| r.empty_weight_lb) {
            parts.push(format!("empty weight {} pounds", with_commas(v)));
        }
    }
    if t.contains("service ceiling") || t.contains("max altitude") {
        if let Some(v) = r.and_then(|r| r.service_ceiling_ft) {
            parts.push(format!("service ceiling {} feet", with_commas(v)));
        }
    }
    if t.contains("fuel capacity") || t.contains("tank capacity") {
        match (r.and_then(|r| r.fuel_capacity_gal), r.and_then(|r| r.fuel_useable_gal)) {
            (Some(c), Some(u)) => parts.push(format!("fuel capacity {c:.0} gallons, {u:.0} useable")),
            (Some(c), None) => parts.push(format!("fuel capacity {c:.0} gallons")),
            _ => {}
        }
    }
    if t.contains("takeoff distance") || t.contains("takeoff roll") {
        if let Some(v) = r.and_then(|r| r.takeoff_distance_ft) {
            parts.push(format!("takeoff distance about {} feet", with_commas(v)));
        }
    }
    if t.contains("landing distance") || t.contains("landing roll") {
        if let Some(v) = r.and_then(|r| r.landing_distance_ft) {
            parts.push(format!("landing distance about {} feet", with_commas(v)));
        }
    }
    if t.contains("oil") && (t.contains("limit") || t.contains("range") || t.contains("normal")) {
        if let (Some(tmin), Some(tmax), Some(pmin), Some(pmax)) = (
            r.and_then(|r| r.oil_temp_min_f),
            r.and_then(|r| r.oil_temp_max_f),
            r.and_then(|r| r.oil_press_min_psi),
            r.and_then(|r| r.oil_press_max_psi),
        ) {
            parts.push(format!(
                "oil temperature normal {tmin:.0} to {tmax:.0} Fahrenheit, oil pressure {pmin:.0} to {pmax:.0} PSI"
            ));
        }
    }
    if t.contains("redline") || t.contains("max rpm") || t.contains("maximum rpm") || t.contains("red line") {
        if let Some(v) = r.and_then(|r| r.max_rpm) {
            parts.push(format!("maximum RPM {}", with_commas(v)));
        }
    }
    // A general "performance limits" / "V-speeds" ask gets the key summary.
    if parts.is_empty()
        && (t.contains("performance limit") || t.contains("v speed") || t.contains("v-speed")
            || (t.contains("limits") && t.contains("performance")))
    {
        let mut s: Vec<String> = Vec::new();
        if let Some(v) = profile.vne_kts { s.push(format!("Vne {v:.0}")); }
        if let Some(v) = r.and_then(|r| r.vno_kts) { s.push(format!("Vno {v:.0}")); }
        if let Some(v) = r.and_then(|r| r.va_kts) { s.push(format!("Va {v:.0}")); }
        if let Some(v) = r.and_then(|r| r.vy_kts) { s.push(format!("Vy {v:.0}")); }
        if let Some(v) = r.and_then(|r| r.best_glide_kts) { s.push(format!("best glide {v:.0}")); }
        if let Some(v) = profile.vs_clean_kts { s.push(format!("stall {v:.0} clean")); }
        if let Some(v) = r.and_then(|r| r.max_gross_lb) { s.push(format!("max gross {} pounds", with_commas(v))); }
        if let Some(v) = r.and_then(|r| r.service_ceiling_ft) { s.push(format!("service ceiling {} feet", with_commas(v))); }
        if !s.is_empty() {
            parts.push(s.join(", "));
        }
    }
    (!parts.is_empty()).then(|| format!("{}.", parts.join(", ")))
}

/// Vfe phrase built from the flap detents' per-notch limits, grouped by speed:
/// "Vfe 110 at ten degrees, 85 at twenty degrees and full". `None` if no detent
/// carries a Vfe.
fn vfe_phrase(profile: &aircraft::AircraftProfile) -> Option<String> {
    let detents = &profile.flaps.as_ref()?.detents;
    let mut groups: Vec<(i64, Vec<String>)> = Vec::new();
    for d in detents {
        let Some(vfe) = d.vfe_kts else { continue };
        let v = vfe.round() as i64;
        match groups.last_mut() {
            Some(g) if g.0 == v => g.1.push(d.spoken.clone()),
            _ => groups.push((v, vec![d.spoken.clone()])),
        }
    }
    if groups.is_empty() {
        return None;
    }
    let phrase = groups
        .iter()
        .map(|(v, notches)| format!("{v} at {}", join_and(notches)))
        .collect::<Vec<_>>()
        .join(", ");
    Some(format!("Vfe {phrase}"))
}

/// Format an integer-valued number with thousands separators ("2558" -> "2,558").
fn with_commas(n: f64) -> String {
    let n = n.round() as i64;
    let digits = n.abs().to_string();
    let bytes = digits.as_bytes();
    let mut out = String::new();
    for (i, c) in bytes.iter().enumerate() {
        if i > 0 && (bytes.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(*c as char);
    }
    if n < 0 {
        format!("-{out}")
    } else {
        out
    }
}

/// True if the utterance is asking about fuel/endurance (a question we answer
/// deterministically), not commanding the fuel *system* (pump/selector).
fn is_fuel_query(text: &str) -> bool {
    let t = text.to_ascii_lowercase();
    let asks = t.contains("fuel") || t.contains("gas") || t.contains("endurance");
    // Exclude fuel *commands* — system switches (pump/selector) and the ground
    // refuel request ("refuel"/"top off"/"fill") — those are actions, not queries.
    let command = t.contains("pump")
        || t.contains("selector")
        || t.contains("boost")
        || t.contains("primer")
        || t.contains("refuel")
        || t.contains("top off")
        || t.contains("fill");
    // "fuel flow" is an engine-instrument read, answered in answer_state_query.
    asks && !command && !t.contains("flow")
}

/// Deterministic spoken fuel report — total in gallons, endurance from the
/// profile's cruise burn, and the left/right balance. Read straight from the
/// datarefs (cache when warm, REST otherwise); never goes near the LLM.
async fn fuel_report(xp: &XPlaneClient, profile: &aircraft::AircraftProfile, live: Option<&LiveState>) -> String {
    let Ok(total_kg) = read1(live, xp, datarefs::FUEL_TOTAL_KG).await else {
        return "I can't read the fuel right now — check the connection to X-Plane.".to_string();
    };
    let gal = total_kg / AVGAS_KG_PER_GAL;
    let mut line = format!("{gal:.0} gallons on board");
    if let Some(f) = &profile.fuel {
        if f.cruise_burn_kg_hr > 0.0 {
            let hours = total_kg / f.cruise_burn_kg_hr;
            let (h, m) = (hours.trunc() as i32, (hours.fract() * 60.0).round() as i32);
            let mut dur = String::new();
            if h > 0 {
                dur.push_str(&format!("{h} hour{}", if h == 1 { "" } else { "s" }));
            }
            if m > 0 {
                if !dur.is_empty() {
                    dur.push(' ');
                }
                dur.push_str(&format!("{m} minute{}", if m == 1 { "" } else { "s" }));
            }
            if !dur.is_empty() {
                line.push_str(&format!(", about {dur} at cruise"));
            }
        }
    }
    // Per-tank balance, when the airframe has two tanks.
    if let Ok(tanks) = xp.get_float_array(datarefs::FUEL_PER_TANK_KG).await {
        if tanks.len() == 2 {
            let (l, r) = (tanks[0] / AVGAS_KG_PER_GAL, tanks[1] / AVGAS_KG_PER_GAL);
            line.push_str(&format!(". Left {l:.0}, right {r:.0}"));
            if (l - r).abs() >= 5.0 {
                line.push_str(if l > r { " — favor the right" } else { " — favor the left" });
            }
        }
    }
    // A question — just answer it; "Roger" is for acknowledging a command.
    format!("{line}.")
}

/// Answer the common "what's our X" state questions deterministically, straight
/// from the datarefs — no LLM (it proved unreliable even with the data in hand).
/// Multiple fields can be asked at once ("altitude and heading"). Returns `None`
/// for anything not a recognized state read, to fall through to the LLM for the
/// open-ended questions ("do we have enough fuel to reach Bristol?"). Only call
/// this for utterances that are questions, so "set heading 270" still commands.
async fn answer_state_query(
    cmd: &str,
    xp: &XPlaneClient,
    live: Option<&LiveState>,
) -> Option<String> {
    let t = cmd.to_ascii_lowercase();
    let mut parts: Vec<String> = Vec::new();

    // Current control settings (e.g. "what's the mixture/throttle set to").
    if t.contains("mixture") {
        if let Ok(v) = read1(live, xp, "sim/cockpit2/engine/actuators/mixture_ratio_all").await {
            parts.push(format!("mixture {:.0} percent", v * 100.0));
        }
    }
    if t.contains("throttle") {
        if let Ok(v) = read1(live, xp, "sim/cockpit2/engine/actuators/throttle_ratio_all").await {
            parts.push(format!("throttle {:.0} percent", v * 100.0));
        }
    }

    if t.contains("altitude") || t.contains("how high") {
        if let Ok(v) = read1(live, xp, datarefs::ALTITUDE_FT).await {
            parts.push(format!("{v:.0} feet"));
        }
    }
    if t.contains("heading") {
        if let Ok(v) = read1(live, xp, datarefs::COMPASS_HEADING_DEG).await {
            parts.push(format!("heading {v:03.0}"));
        }
    }
    let vs_q = t.contains("vertical speed") || t.contains("climb rate") || t.contains("rate of");
    if vs_q {
        if let Ok(v) = read1(live, xp, datarefs::VVI_FPM).await {
            parts.push(format!("{v:+.0} feet per minute"));
        }
    }
    // "speed" alone → airspeed, unless it was "vertical speed".
    if t.contains("airspeed") || t.contains("air speed") || t.contains("how fast") || (t.contains("speed") && !vs_q) {
        if let Ok(v) = read1(live, xp, datarefs::AIRSPEED_KTS).await {
            parts.push(format!("{v:.0} knots"));
        }
    }
    // Weather — answered deterministically, not via the LLM (same reliability fix
    // as fuel). "weather" gives the lot; specific words give just that field.
    let wx = t.contains("weather") || t.contains("metar") || t.contains("conditions");
    if wx || t.contains("wind") {
        if let (Ok(dir), Ok(spd_ms)) =
            (read1(live, xp, datarefs::WIND_DIR_DEG_TRUE).await, read1(live, xp, datarefs::WIND_SPEED_MSC).await)
        {
            let kts = spd_ms * MS_TO_KTS;
            parts.push(if kts < 1.0 { "wind calm".to_string() } else { format!("wind {dir:03.0} at {kts:.0} knots") });
        }
    }
    if wx || t.contains("visibility") {
        if let Ok(v) = read1(live, xp, datarefs::VISIBILITY_SM).await {
            parts.push(format!("visibility {v:.0} statute miles"));
        }
    }
    // "pressure" alone could mean the altimeter or the oil — exclude oil pressure
    // so "how's the oil pressure?" doesn't also blurt the altimeter setting.
    if wx || t.contains("altimeter") || t.contains("qnh") || t.contains("barometric")
        || (t.contains("pressure") && !t.contains("oil"))
    {
        if let Ok(v) = read1(live, xp, datarefs::QNH_PASCALS).await {
            // qnh in Pascals → altimeter setting in inHg (1 inHg = 3386.389 Pa).
            parts.push(format!("altimeter {:.2}", v / 3386.389));
        }
    }
    if wx || (t.contains("temperature") && !t.contains("oil")) || t.contains("how cold") || t.contains("how hot") || t.contains(" oat") {
        if let Ok(v) = read1(live, xp, datarefs::OAT_DEG_C).await {
            parts.push(format!("outside air temperature {v:.0} degrees celsius"));
        }
    }

    // General "how's the engine?" → a one-line health summary. Only when no
    // specific instrument is named (those are handled below, no double-up).
    if t.contains("engine")
        && !["oil", "egt", "exhaust", "cht", "cylinder", "rpm", "fuel flow"].iter().any(|k| t.contains(k))
    {
        let running = read_engine0(xp, "sim/flightmodel/engine/ENGN_running").await.is_some_and(|v| v > 0.5);
        if !running {
            parts.push("engine off".to_string());
        } else {
            let mut bits = vec!["engine running".to_string()];
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/prop_speed_rpm").await {
                bits.push(format!("{v:.0} R P M"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_pressure_psi").await {
                bits.push(format!("oil pressure {v:.0} pounds"));
            }
            // oil_temp / CHT datarefs read °F despite their names.
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_temperature_deg_C").await {
                bits.push(format!("oil temp {v:.0} degrees"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/CHT_deg_C").await {
                bits.push(format!("C H T {v:.0}"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/fuel_flow_kg_sec").await {
                bits.push(format!("fuel flow {:.1} gallons an hour", v * 3600.0 / AVGAS_KG_PER_GAL));
            }
            parts.push(bits.join(", "));
        }
    }

    // Engine instruments (per-engine arrays → element 0). Temperatures spoken in
    // °F to match the 172's gauges and the POH. Names verified against the stock
    // 172's cockpitdecks mapping.
    let c_to_f = |c: f64| c * 9.0 / 5.0 + 32.0;
    if t.contains("oil pressure") {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_pressure_psi").await {
            parts.push(format!("oil pressure {v:.0} pounds"));
        }
    }
    if t.contains("oil temp") || (t.contains("oil") && t.contains("temperature")) {
        // oil_temperature_deg_C is misnamed — reads °F already.
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_temperature_deg_C").await {
            parts.push(format!("oil temperature {v:.0} degrees"));
        }
    }
    if t.contains("egt") || t.contains("exhaust") {
        // EGT_deg_cel is genuinely Celsius → convert to °F for the readout.
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/EGT_deg_cel").await {
            parts.push(format!("exhaust gas temperature {:.0} degrees", c_to_f(v)));
        }
    }
    if t.contains("cht") || t.contains("cylinder") {
        // CHT_deg_C is misnamed — reads °F already.
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/CHT_deg_C").await {
            parts.push(format!("cylinder head temperature {v:.0} degrees"));
        }
    }
    if t.contains("rpm") || t.contains("r p m") || t.contains("engine speed") {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/prop_speed_rpm").await {
            parts.push(format!("{v:.0} R P M"));
        }
    }
    if t.contains("fuel flow") || t.contains("flow") || t.contains("burning") {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/fuel_flow_kg_sec").await {
            parts.push(format!("fuel flow {:.1} gallons an hour", v * 3600.0 / AVGAS_KG_PER_GAL));
        }
    }
    if t.contains("ammeter") || t.contains("amps") || t.contains("charging") {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/electrical/battery_amps").await {
            let state = if v > 1.0 { "charging" } else if v < -1.0 { "discharging" } else { "steady" };
            parts.push(format!("ammeter {v:+.0} amps, {state}"));
        }
    }
    if t.contains("vacuum") || t.contains("suction") {
        if let Ok(v) = read1(live, xp, "sim/cockpit/misc/vacuum").await {
            parts.push(format!("suction {v:.1} inches"));
        }
    }
    // A question — answer plainly; "Roger" is a command acknowledgment.
    (!parts.is_empty()).then(|| format!("{}.", parts.join(", ")))
}

/// A compact live-state line for the LLM's context, so it can ground relative or
/// ambiguous commands ("a bit more flaps", "level off") in what's actually true.
/// Each field is best-effort: a dataref that won't read is skipped, and if none
/// read (X-Plane unreachable) this returns `None` and the model runs context-free.
async fn state_snapshot(
    xp: &XPlaneClient,
    profile: &aircraft::AircraftProfile,
    live: Option<&LiveState>,
) -> Option<String> {
    // Reads prefer the live-state WebSocket cache (instant, no round-trip) and
    // fall back to a concurrent REST read on a cache miss — so this is never worse
    // than the old all-REST burst, and near-free once the subscription is warm.
    let (alt, ias, vs, hdg, flap, gear, og, fuel, ap, engine, wind_dir, wind_kts, oat, vis, qnh) = tokio::join!(
        read1(live, xp, datarefs::ALTITUDE_FT),
        read1(live, xp, datarefs::AIRSPEED_KTS),
        read1(live, xp, datarefs::VVI_FPM),
        read1(live, xp, datarefs::COMPASS_HEADING_DEG),
        read1(live, xp, datarefs::FLAP_RATIO),
        read1(live, xp, datarefs::GEAR_HANDLE_DOWN),
        read1(live, xp, datarefs::ON_GROUND_ANY),
        read1(live, xp, datarefs::FUEL_TOTAL_KG),
        ap_mode(xp, live),
        checklist::engine_running(xp),
        // Weather, so "what's the weather?" gets real numbers. Wind/OAT are the
        // cockpit-relative indicators (mirror the gauges); visibility and QNH come
        // from the XP12 aircraft-weather datarefs. All confirmed reading live.
        read1(live, xp, datarefs::WIND_DIR_DEG_TRUE),
        read1(live, xp, datarefs::WIND_SPEED_MSC),
        read1(live, xp, datarefs::OAT_DEG_C),
        read1(live, xp, datarefs::VISIBILITY_SM),
        read1(live, xp, datarefs::QNH_PASCALS),
    );
    let mut parts: Vec<String> = Vec::new();
    if let Ok(v) = alt {
        parts.push(format!("altitude {v:.0} ft"));
    }
    if let Ok(v) = ias {
        parts.push(format!("airspeed {v:.0} knots"));
    }
    if let Ok(v) = vs {
        parts.push(format!("vertical speed {v:+.0} fpm"));
    }
    if let Ok(v) = hdg {
        parts.push(format!("heading {v:03.0}"));
    }
    if let Ok(v) = flap {
        parts.push(format!("flaps {:.0}%", v * 100.0));
    }
    if profile.retractable_gear {
        if let Ok(v) = gear {
            parts.push(format!("gear {}", if v > 0.5 { "down" } else { "up" }));
        }
    }
    if let Ok(v) = og {
        parts.push(format!("on the ground: {}", if v > 0.5 { "yes" } else { "no" }));
    }
    if let Ok(v) = fuel {
        // Report in gallons (how a GA pilot thinks) with the mass alongside, plus an
        // endurance estimate from the profile's cruise burn so "how long can we fly?"
        // has a real answer.
        let gal = v / AVGAS_KG_PER_GAL;
        let mut s = format!("fuel on board {gal:.0} gallons ({v:.0} kg)");
        if let Some(f) = &profile.fuel {
            if f.cruise_burn_kg_hr > 0.0 {
                let hours = v / f.cruise_burn_kg_hr;
                s.push_str(&format!(", about {hours:.1} hours endurance at cruise"));
            }
        }
        parts.push(s);
    }
    // Weather: wind as "direction at speed" (calm if barely moving), plus OAT.
    // Wind speed is m/s from the aircraft-weather dataref → knots.
    if let (Ok(dir), Ok(spd_ms)) = (wind_dir, wind_kts) {
        let kts = spd_ms * MS_TO_KTS;
        if kts < 1.0 {
            parts.push("wind calm".to_string());
        } else {
            parts.push(format!("wind {dir:03.0} at {kts:.0} knots"));
        }
    }
    if let Ok(v) = oat {
        parts.push(format!("outside air temperature {v:.0} degrees celsius"));
    }
    if let Ok(v) = vis {
        parts.push(format!("visibility {v:.0} statute miles"));
    }
    if let Ok(v) = qnh {
        // qnh_pas is sea-level pressure in Pascals; report as the altimeter
        // setting in inches of mercury (1 inHg = 3386.389 Pa).
        parts.push(format!("altimeter {:.2}", v / 3386.389));
    }
    // Engine status — only when X-Plane is actually reachable (some scalar read
    // succeeded), since engine_running defaults to true when it can't read.
    if !parts.is_empty() {
        parts.push(format!("engines {}", if engine { "running" } else { "off" }));
    }
    if ap != "--" {
        parts.push(format!("autopilot {ap}"));
    }
    (!parts.is_empty()).then(|| parts.join(", "))
}

/// Read the live aircraft state for the dashboard panel. Returns `None` if
/// X-Plane is unreachable (panel keeps its last values).
async fn poll_aircraft(xp: &XPlaneClient) -> Option<ui::Aircraft> {
    Some(ui::Aircraft {
        heading: xp.get_float(datarefs::COMPASS_HEADING_DEG).await.ok()?,
        altitude: xp.get_float(datarefs::ALTITUDE_FT).await.ok()?,
        ias: xp.get_float(datarefs::AIRSPEED_KTS).await.ok()?,
        vsi: xp.get_float(datarefs::VVI_FPM).await.ok()?,
        flaps_pct: xp.get_float(datarefs::FLAP_RATIO).await.ok()? * 100.0,
        ap_mode: ap_mode(xp, None).await,
    })
}

/// Summarize the active autopilot lateral mode for the panel. Reads prefer the
/// live-state cache when given one (snapshot's hot path), else REST (the poller).
async fn ap_mode(xp: &XPlaneClient, live: Option<&LiveState>) -> String {
    let (servos, appr, nav, hdg) = tokio::join!(
        read1(live, xp, datarefs::SERVOS_ON),
        read1(live, xp, datarefs::APPROACH_STATUS),
        read1(live, xp, datarefs::NAV_STATUS),
        read1(live, xp, datarefs::HEADING_STATUS),
    );
    if servos.unwrap_or(0.0) <= 0.5 {
        return "--".to_string();
    }
    let (appr, nav, hdg) = (appr.unwrap_or(0.0), nav.unwrap_or(0.0), hdg.unwrap_or(0.0));
    if appr >= 1.0 {
        "APR".into()
    } else if nav >= 1.0 {
        "NAV".into()
    } else if hdg >= 1.0 {
        "HDG".into()
    } else {
        "ON".into()
    }
}

/// How to recover from one utterance's worth of actions. The command loop keeps a
/// stack of these so "belay that" can walk back command by command, each with its
/// own recovery (the whole point: remember what was done and how to undo it).
struct UndoSnapshot {
    /// What the pilot said, for the readback ("belay the throttle").
    label: String,
    /// (dataref, spoken label, previous value, is-integer) to write back.
    reverts: Vec<(String, String, f64, bool)>,
    /// The action switched the autopilot on (it was off before), so undo should
    /// switch it back off, not just reset the dial it was flying to.
    disengage_ap: bool,
    /// Tools to run to recover an action that isn't a simple value write — e.g.
    /// `start_engine` to undo an engine shutdown.
    inverse: Vec<(String, serde_json::Value)>,
}

/// Keep recovery for the last dozen utterances — enough to walk back a bad stretch,
/// bounded so it can't grow without limit over a long flight.
const UNDO_STACK_MAX: usize = 12;

/// Tools that can engage the autopilot servos. If one of these turns the AP on,
/// "belay that" disengages it again.
fn engages_autopilot(tool: &str) -> bool {
    matches!(
        tool,
        "set_heading" | "set_altitude" | "set_vertical_speed" | "set_climb" | "navigate"
    )
}

/// One reversible dataref a tool writes: (dataref, spoken label, is it an integer
/// dataref). The int flag matters on restore — transponder/radio datarefs are
/// integers and must be written with `set_int`, not `set_float`.
type Undoable = (&'static str, &'static str, bool);
const F: bool = false; // float dataref
const I: bool = true; // integer dataref

/// The reversible dataref(s) a tool writes, so the command loop can snapshot them
/// before the write and restore on "belay that". Tools that fire commands rather
/// than set values (engine start, pushback, gyro align) have no entry; engine
/// shutdown is undone by an inverse command instead (see the capture loop). Kept
/// here, next to the undo logic, rather than in each tool: undo is a loop-level
/// concern and this keeps it in one place.
fn undoable_drefs(tool: &str) -> &'static [Undoable] {
    match tool {
        "set_flaps" => &[("sim/cockpit2/controls/flap_ratio", "flaps", F)],
        "gear" => &[("sim/cockpit2/controls/gear_handle_down", "the gear", F)],
        "set_throttle" => &[("sim/cockpit2/engine/actuators/throttle_ratio_all", "throttle", F)],
        "set_mixture" | "lean_mixture" => &[("sim/cockpit2/engine/actuators/mixture_ratio_all", "mixture", F)],
        "set_pitch_trim" => &[("sim/cockpit2/controls/elevator_trim", "pitch trim", F)],
        "set_barometer" | "set_altimeter_current" => {
            &[("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot", "the altimeter", F)]
        }
        "set_parking_brake" => &[("sim/cockpit2/controls/parking_brake_ratio", "the parking brake", F)],
        "set_heading" => &[("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot", "the heading bug", F)],
        "set_altitude" => &[("sim/cockpit2/autopilot/altitude_dial_ft", "the altitude select", F)],
        "set_vertical_speed" | "set_climb" => &[("sim/cockpit/autopilot/vertical_velocity", "vertical speed", F)],
        "set_transponder" => &[("sim/cockpit2/radios/actuators/transponder_code", "the squawk", I)],
        "set_transponder_mode" => &[("sim/cockpit2/radios/actuators/transponder_mode", "the transponder mode", I)],
        "set_com1" => &[("sim/cockpit2/radios/actuators/com1_frequency_hz_833", "com 1", I)],
        "set_com2" => &[("sim/cockpit2/radios/actuators/com2_frequency_hz_833", "com 2", I)],
        "set_nav1" => &[("sim/cockpit2/radios/actuators/nav1_frequency_hz", "nav 1", I)],
        "set_obs" => &[("sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot", "the OBS", F)],
        _ => &[],
    }
}

/// Spoken label for an inverse recovery tool run during undo.
fn recovery_label(tool: &str) -> String {
    match tool {
        "start_engine" => "engine restarted".to_string(),
        other => format!("{} run", other.replace('_', " ")),
    }
}

/// An emergency abort/undo of the last action. Deterministic and matched before any
/// gate or model, so it always works. Phrased a few natural ways a pilot would say
/// it under stress.
fn is_undo(text: &str) -> bool {
    let t = text.to_ascii_lowercase();
    t.contains("belay")
        || t.contains("undo")
        || t.contains("scratch that")
        || t.contains("as you were")
        || t.contains("disregard")
        || t.contains("revert")
        || t.contains("reverse that")
        || t.contains("take it back")
        || t.contains("cancel that")
        || t.contains("abort that")
        || t.contains("abort the last")
        || t.contains("abort")
        // "abort" is reliably misheard as "a boat" / "a board" — both meaningless in
        // a cockpit, and too important a word to lose to a transcription slip.
        || t.contains("a boat")
        || t.contains("a board")
}

/// Join words as "a", "a and b", or "a, b and c" for a natural spoken list.
fn join_and(items: &[String]) -> String {
    match items {
        [] => String::new(),
        [a] => a.clone(),
        [a, b] => format!("{a} and {b}"),
        [rest @ .., last] => format!("{}, and {last}", rest.join(", ")),
    }
}

/// Example commands shown in the dashboard's "say" panel.
/// Aviation vocabulary that a real command contains at least one of. Used to
/// reject phonetic nonsense that whisper transcribes with high confidence
/// ("Nautilapuku") before it reaches the LLM, which would otherwise force-match
/// it to the nearest tool. Whole-word, case-insensitive.
const COMMAND_KEYWORDS: &[&str] = &[
    // flaps / gear / trim
    "flap", "flaps", "gear", "trim",
    // autopilot / vertical nav
    "autopilot", "heading", "altitude", "climb", "descend", "maintain", "level",
    "vertical", "vs", "approach", "ils", "localizer", "glideslope", "obs",
    "course", "radial", "runway",
    // vertical-nav synonyms whisper actually produced that the gate missed:
    // "ascend 500" (for climb), "physical speed 500" (mishear of vertical speed).
    "ascend", "ascent", "speed",
    // lights
    "light", "lights", "beacon", "strobe", "strobes", "taxi", "landing", "nav",
    "navigation", "flood", "floods",
    // engine / fuel
    "throttle", "thrust", "power", "idle", "mixture", "rich", "lean", "engine",
    "engines", "start", "ignition", "shutdown", "magneto", "magnetos", "mags", "carb",
    "carburetor", "fuel", "pump", "boost", "primer", "selector",
    // electrical
    "battery", "alternator", "generator", "avionics", "electrical", "electric",
    "electrics", "bus", "master",
    // radios / transponder
    "com", "comm", "radio", "frequency", "tune", "squawk", "transponder",
    "ident", "barometer", "altimeter", "baro", "kollsman", "qnh", "vor",
    // instruments / misc controls
    "gyro", "pitot", "brake", "brakes", "parking", "heat",
    // ramp / ground-ops: chocks, ground power, pushback, services
    "chocks", "chock", "pushback", "push", "tug", "gpu", "ground", "refuel",
    "service", "services",
    // checklist invocations: "checklist(s)" is the trigger word every one shares
    // (so "normal take-off checklist" reaches the LLM even when STT splits
    // "takeoff"), plus the phase words for the shorthand ("configure for takeoff").
    "checklist", "checklists", "takeoff", "configure", "config", "departure",
    "secure", "clean", "cruise", "preflight", "limits", "performance",
    // systems-status check + reference-limit / V-speed queries
    "systems", "system", "vne", "vno", "vfe", "vspeed", "vspeeds",
    // aircraft-info queries
    "aircraft", "plane", "cessna", "skyhawk",
    // SimBrief flight plan + loading the route into the GPS/FMS
    "simbrief", "flightplan", "plan", "route", "load", "program", "gps", "fms",
    // couple the autopilot to the flight plan
    "navigate", "follow", "track",
    // navigation progress queries
    "waypoint", "eta",
    // ETA to the destination ("time before we land", "when do we arrive")
    "land", "destination", "arrive", "arrival",
    // time/distance to top of descent
    "descent", "tod",
    // reminders ("wake me at top of descent", "remind me in 10 minutes")
    "wake", "remind", "notify", "heads-up", "reminder", "reminders", "cancel",
    // emergency abort/undo of the last action
    "belay", "undo", "abort", "disregard", "scratch", "revert",
];

/// True if the utterance is a question about the flight (answered from state,
/// not actioned). Interrogative openers or a trailing "?". Kept conservative so
/// it doesn't hijack commands — and misrouting is harmless anyway (answering is
/// read-only). Command-ish openers like "can you set…" are deliberately excluded.
fn looks_like_question(text: &str) -> bool {
    let t = text.trim().to_ascii_lowercase();
    if t.ends_with('?') {
        return true;
    }
    const OPENERS: &[&str] = &[
        "what", "what's", "whats", "how", "how's", "hows", "how much", "how many",
        "how far", "how high", "how fast", "are we", "are the", "is the", "is it",
        "do we", "did we", "have we", "where", "when", "which", "why", "tell me",
    ];
    OPENERS.iter().any(|o| t == *o || t.starts_with(&format!("{o} ")))
}

/// True if the utterance plausibly is an aircraft command (contains at least one
/// keyword). Phonetic garbage that happens to transcribe confidently shares no
/// vocabulary with the domain and is rejected here.
fn looks_like_command(text: &str) -> bool {
    text.split(|c: char| !c.is_ascii_alphabetic())
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
}

/// True if an utterance looks like a *continuation* of the previous command, so
/// it can clear the gate (within the follow-up window) without a domain keyword
/// and let the LLM resolve it from history. Two shapes count:
/// - back-references / adjustments ("it", "that", "again", "more", "the other one");
/// - a bare quantity ("seventy percent", "60%", "two thousand") — almost always
///   re-stating the value of the command just given ("throttle fifty" → "seventy").
fn looks_like_followup(text: &str) -> bool {
    const FOLLOWUP_WORDS: &[&str] = &[
        "it", "that", "them", "those", "again", "back", "undo", "revert",
        "reset", "more", "less", "instead", "also", "other", "same", "too",
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
fn is_value_word(w: &str) -> bool {
    phrase::is_number_word(w) || w == "percent"
}

/// True if the utterance carries a digit or a spoken number.
fn has_number(text: &str) -> bool {
    text.chars().any(|c| c.is_ascii_digit())
        || text
            .split(|c: char| !c.is_ascii_alphabetic())
            .any(|w| is_value_word(&w.to_ascii_lowercase()))
}

/// True if the utterance is *only* a number/percent value (digits, number words,
/// and light filler like "to"/"at"/"point") — a genuine bare follow-up safe to
/// re-anchor onto the last command. "fraudal at eighty percent" is NOT bare, so
/// it won't be force-attached to the wrong tool.
fn is_bare_value(text: &str) -> bool {
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

/// The leading non-numeric words of a command — its "subject" — used to
/// re-anchor a bare-value follow-up onto the same system. "throttle eighty
/// percent" → "throttle"; "set heading 270" → "set heading". Empty if the
/// command starts with a number.
fn command_subject(cmd: &str) -> String {
    let mut out: Vec<&str> = Vec::new();
    for w in cmd.split_whitespace() {
        let bare = w.trim_matches(|c: char| !c.is_ascii_alphanumeric());
        if bare.chars().any(|c| c.is_ascii_digit())
            || is_value_word(&bare.to_ascii_lowercase())
        {
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
fn matches_keyword(w: &str) -> bool {
    if COMMAND_KEYWORDS.contains(&w) {
        return true;
    }
    w.len() >= 5
        && COMMAND_KEYWORDS
            .iter()
            .filter(|k| k.len() >= 5)
            .any(|k| levenshtein(w, k) <= 1)
}

/// Short label for the whisper model from its path, for the status bar:
/// `models/ggml-base.en.bin` → `base.en`, `…/ggml-large-v3-turbo.bin` → `large-v3-turbo`.
fn whisper_model_label(path: &str) -> String {
    let file = path.rsplit(['/', '\\']).next().unwrap_or(path);
    let stem = file.strip_suffix(".bin").unwrap_or(file);
    stem.strip_prefix("ggml-").unwrap_or(stem).to_string()
}

fn command_hints() -> Vec<String> {
    [
        "flaps ten / full",
        "landing light on",
        "strobes off",
        "autopilot heading 270",
        "climb and maintain 7000",
        "descend at 500",
        "throttle 60 percent",
        "mixture full rich",
        "start the engine",
        "engine off",
        "set altimeter 29.92",
        "com one 118.5",
        "squawk 1200",
        "parking brake set",
        "battery on / alternator on",
        "avionics on",
        "align the gyro",
        "prepare the ILS for runway 13",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect()
}

#[cfg(test)]
mod tests {
    use super::{is_abort, is_affirmative, is_undo, join_and, undoable_drefs};

    #[test]
    fn undo_recognizes_emergency_phrasings() {
        for w in ["belay that", "undo", "undo that", "abort", "scratch that", "as you were",
                  "disregard", "revert", "reverse that", "take it back", "cancel that",
                  "abort. didn't catch a command there. abort.", "a boat"] {
            assert!(is_undo(w), "{w:?} should trigger undo");
        }
        // Not an undo (would clobber normal speech / the reminder cancel).
        for w in ["set heading 270", "cancel the reminders", "flaps up"] {
            assert!(!is_undo(w), "{w:?} should not trigger undo");
        }
    }

    #[test]
    fn value_tools_are_undoable_command_tools_are_not() {
        assert!(!undoable_drefs("set_heading").is_empty());
        assert!(!undoable_drefs("set_throttle").is_empty());
        assert!(undoable_drefs("start_engine").is_empty());
        assert!(undoable_drefs("pushback").is_empty());
    }

    #[test]
    fn reference_limits_come_from_the_profile() {
        let p = super::aircraft::load("cessna_172sp").expect("profile");
        assert_eq!(super::reference_limits(&p, "what's best glide").as_deref(), Some("best glide 68 knots."));
        assert_eq!(super::reference_limits(&p, "rotation speed").as_deref(), Some("rotate at 55 knots."));
        assert!(super::reference_limits(&p, "what's vy").unwrap().contains("74 knots"));
        // Vfe is derived from the flap detents, grouped by speed.
        assert_eq!(
            super::reference_limits(&p, "flap speed").as_deref(),
            Some("Vfe 110 at ten degrees, 85 at twenty degrees and full.")
        );
        // A non-limits utterance returns nothing.
        assert!(super::reference_limits(&p, "set flaps ten").is_none());

        // The SR22 carries its own card — different numbers, same machinery.
        let sr = super::aircraft::load("cirrus_sr22").expect("sr22 profile");
        assert_eq!(super::reference_limits(&sr, "best glide").as_deref(), Some("best glide 88 knots."));
        assert!(super::reference_limits(&sr, "never exceed").unwrap().contains("204"));
        assert!(super::reference_limits(&sr, "max crosswind").unwrap().contains("21"));
    }

    #[test]
    fn md82_is_a_jet_with_gear_no_piston_tools() {
        let p = super::aircraft::load("md82").expect("md82 profile loads");
        assert_eq!(p.id, "md82");
        assert!(p.autopilot && p.throttle && p.retractable_gear);
        assert!(!p.engine && !p.magnetos && !p.carb_heat, "jet has no piston systems");
        assert_eq!(p.vne_kts, Some(340.0)); // Vmo
        let reg = super::tools::Registry::for_aircraft(&p);
        assert!(reg.get("gear").is_some(), "retractable gear -> gear tool");
        assert!(reg.get("set_throttle").is_some(), "throttle flag -> thrust levers");
        assert!(reg.get("set_mixture").is_none(), "no mixture on a jet");
        assert!(reg.get("start_engine").is_none(), "no piston start on a jet");
        assert!(reg.get("set_flaps").is_some());
        // Electrical is wired (battery, generators, avionics, "all"), so "all
        // electrical systems on" has a tool to land on.
        assert!(reg.get("set_electrical").is_some());
        assert!(p.electrical.iter().any(|e| e.key == "all"));
        // Jet start sequence, not the piston one-shot.
        assert!(reg.get("start_engines").is_some());
        assert!(reg.get("start_engine").is_none());
        // ...and the piston 172 is the other way round.
        let c172 = super::tools::Registry::for_aircraft(&super::aircraft::load("cessna_172sp").unwrap());
        assert!(c172.get("start_engines").is_none());
        assert!(c172.get("start_engine").is_some());
        // Reference card answers from the POM.
        assert!(super::reference_limits(&p, "service ceiling").unwrap().contains("37,000"));
        // The MD-82 checklist parses.
        assert!(crate::checklist::load(&p).is_ok());
    }

    #[test]
    fn join_and_reads_naturally() {
        assert_eq!(join_and(&["flaps".into()]), "flaps");
        assert_eq!(join_and(&["flaps".into(), "throttle".into()]), "flaps and throttle");
        assert_eq!(
            join_and(&["flaps".into(), "throttle".into(), "mixture".into()]),
            "flaps, throttle, and mixture"
        );
    }

    #[test]
    fn abort_is_distinct_from_skip_and_yes() {
        // Abort the whole checklist.
        for w in ["stop", "abort", "cancel", "belay that", "disregard"] {
            assert!(is_abort(w), "{w:?} should abort");
            assert!(!is_affirmative(w), "{w:?} is not a yes");
        }
        // A plain no/negative skips the step but isn't an abort.
        for w in ["no", "negative", "not yet"] {
            assert!(!is_abort(w), "{w:?} should not abort");
            assert!(!is_affirmative(w));
        }
        // Yes proceeds and isn't an abort.
        for w in ["affirm", "yes", "go ahead", "do it"] {
            assert!(is_affirmative(w));
            assert!(!is_abort(w));
        }
    }

    use super::looks_like_command;

    #[test]
    fn real_commands_pass_the_domain_gate() {
        for c in [
            "flaps ten", "gear up", "landing light on", "strobes off",
            "autopilot heading 270", "climb and maintain 7000", "descend at 500",
            "throttle 60 percent", "mixture full rich", "start the engine",
            "battery on", "squawk 1200", "align the gyro", "pitot heat on",
            "fuel pump on", "carb heat off", "magnetos both", "trim nose up",
            "nav one 109.5", "set the OBS to 240", "com two 121.9",
            "transponder standby", "prepare the ILS for runway 13", "flood lights on",
            // Phrasings that previously fell through the gate (issue: rejected
            // before reaching the LLM).
            "electrics off", "fix the gyro drift", "master switch on",
            "generator on", "squawk VFR",
            // Verbatim whisper output from logs/ that the gate used to reject —
            // recovered by the ignition/ascend/speed keywords.
            "Ignition on.", "Ascend 500 feet per minute.", "Physical speed 500.",
            // Whisper glued "flood lights" into one token; the compound and its
            // "blood" mishearing must still reach the LLM.
            "Floodlights on.", "Turn on the floodlights.", "Increase floodlights.",
            "Bloodlights on.", "landinglight on",
            // Phase-config macro invocations must reach the LLM.
            "configure for takeoff", "set up for takeoff", "takeoff config",
            "get ready for takeoff", "prep for departure",
            "configure for landing", "get ready for landing",
            "after landing", "clean up", "clear of the runway",
            "secure the aircraft", "secure", "full shutdown",
            // "checklist" carries any phase past the gate, even hyphenated STT.
            "Normal take-off checklist.", "starting engine checklist", "preflight checklist",
        ] {
            assert!(looks_like_command(c), "{c:?} should pass the domain gate");
        }
    }

    use super::looks_like_followup;

    #[test]
    fn followups_are_recognized_but_plain_chatter_is_not() {
        for c in [
            "switch it back to both", "do that again", "a bit more", "undo that",
            "the other tank", "seventy percent", "60%", "two thousand",
        ] {
            assert!(looks_like_followup(c), "{c:?} should look like a follow-up");
        }
        for c in [
            "nice weather today", "where are we", "hello copilot",
            // A long counting string is noise, not a bare-value follow-up.
            "one two three four five six seven eight ten",
        ] {
            assert!(!looks_like_followup(c), "{c:?} should not");
        }
    }

    use super::{command_subject, has_number};

    #[test]
    fn bare_value_followups_re_anchor_to_the_last_subject() {
        assert_eq!(command_subject("throttle eighty percent"), "throttle");
        assert_eq!(command_subject("set heading 270"), "set heading");
        assert_eq!(command_subject("flaps ten"), "flaps");
        assert_eq!(command_subject("60 percent"), ""); // starts with a number
        assert!(has_number("sixty percent") && has_number("60%") && has_number("two thousand"));
        assert!(!has_number("switch it back to both"));
        // The rewrite a bare follow-up would produce:
        assert_eq!(format!("{} {}", command_subject("throttle eighty percent"), "sixty percent"), "throttle sixty percent");
    }

    #[test]
    fn glued_light_compound_does_not_overmatch() {
        // Stripping a light/lights suffix must not wave through non-cockpit words.
        for g in ["moonlight", "highlight", "delight", "spotlight", "flashlight"] {
            assert!(!looks_like_command(g), "{g:?} should still be rejected");
        }
    }

    #[test]
    fn one_letter_mishearings_pass_the_gate() {
        // Whisper drops/swaps a letter; these should still reach the LLM.
        for c in ["trust 80 percent", "descent 500", "altitude 3000"] {
            assert!(looks_like_command(c), "{c:?} should pass via near-miss");
        }
    }

    #[test]
    fn phonetic_garbage_is_rejected() {
        for g in [
            "Nautilapuku", "Mokuli ke maluku", "Reki ki panaki",
            "Tiki Maliki, WaKiPi", "Take a look at the video", "",
        ] {
            assert!(!looks_like_command(g), "{g:?} should be rejected");
        }
    }

    #[test]
    fn affirmatives_proceed() {
        for r in ["affirm", "affirmative", "yes", "confirm", "roger that", "correct", "go ahead"] {
            assert!(is_affirmative(r), "{r:?} should confirm");
        }
    }

    #[test]
    fn negatives_and_ambiguity_cancel() {
        // explicit no, and anything not clearly yes, must NOT proceed
        for r in ["negative", "no", "cancel", "stop", "abort", "uh what", "descend 700", ""] {
            assert!(!is_affirmative(r), "{r:?} must not confirm");
        }
    }

    use super::is_bare_value;

    #[test]
    fn bare_value_gates_followup_rewrite() {
        // Genuine bare follow-ups (re-anchored to the last command).
        assert!(is_bare_value("seventy percent"));
        assert!(is_bare_value("60%"));
        assert!(is_bare_value("two thousand five hundred"));
        // Mishears / real words — must NOT be force-anchored to the last tool.
        assert!(!is_bare_value("fraudal at eighty percent"));
        assert!(!is_bare_value("total thirty percent"));
        assert!(!is_bare_value("throttle eighty"));
        assert!(!is_bare_value("switch it back"));
    }

    use super::looks_like_question;

    #[test]
    fn fuel_query_detects_questions_not_system_commands() {
        use super::is_fuel_query;
        assert!(is_fuel_query("how much fuel do we have"));
        assert!(is_fuel_query("what's our fuel"));
        assert!(is_fuel_query("how much gas"));
        assert!(is_fuel_query("what's our endurance"));
        // Fuel *system* commands must NOT be hijacked — they go to their tools.
        assert!(!is_fuel_query("fuel pump on"));
        assert!(!is_fuel_query("fuel selector both"));
        assert!(!is_fuel_query("boost pump on"));
        assert!(!is_fuel_query("flaps ten"));
        // "fuel flow" is an engine-instrument read (answer_state_query), not the
        // total-fuel report.
        assert!(!is_fuel_query("what's the fuel flow"));
    }

    #[test]
    fn whisper_label_strips_path_prefix_and_suffix() {
        use super::whisper_model_label;
        assert_eq!(whisper_model_label("models/ggml-base.en.bin"), "base.en");
        assert_eq!(whisper_model_label("/a/b/ggml-large-v3-turbo.bin"), "large-v3-turbo");
        assert_eq!(whisper_model_label("small.en"), "small.en");
    }

    #[test]
    fn questions_are_detected_commands_are_not() {
        for q in [
            "how much fuel do we have", "what's our heading", "what is our altitude?",
            "are we configured for landing", "how high are we", "tell me the airspeed",
            "do we have enough fuel to make it?", "where are we",
            // Keyword+number questions: same shape as a command, so they must be
            // caught here and withheld from the grammar fast-path (which would
            // otherwise *act* on them) — see the fast-path gate in run_session.
            "is the heading 270?", "did we set flaps to ten?",
        ] {
            assert!(looks_like_question(q), "{q:?} should be a question");
        }
        for c in [
            "flaps ten", "gear up", "set heading 270", "throttle 60 percent",
            "landing light on", "start the engine",
        ] {
            assert!(!looks_like_question(c), "{c:?} should not be a question");
        }
    }

    use super::correction_text;

    #[test]
    fn correction_extracts_value_from_negative_reply() {
        assert_eq!(correction_text("negative, 500").as_deref(), Some("500"));
        assert_eq!(
            correction_text("no make it five hundred").as_deref(),
            Some("make it five hundred")
        );
    }

    #[test]
    fn plain_negation_is_not_a_correction() {
        assert_eq!(correction_text("negative"), None);
        assert_eq!(correction_text("no"), None);
        assert_eq!(correction_text("cancel."), None);
    }
}
