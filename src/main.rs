//! XVoiceLayer — the voice layer for X-Plane 12: copilot, ATC and cabin crew.
//!
//! The voice loop, end to end and fully local:
//!   listen → whisper (STT) → local LLM picks one tool →
//!   write dataref + read back → say/piper (TTS) confirms.
//!
//! Two capture modes (config `listen_mode`): `continuous` (always listening,
//! VAD-segmented) or `ptt` (push-to-talk).

mod aircraft;
mod approach_guide;
mod audio;
mod auto_status;
mod autoland;
mod autotakeoff;
mod autotaxi;
mod cabin;
mod checklist;
mod circuit;
mod config;
mod failures;
mod full_auto;
mod grammar;
mod livestate;
mod llm;
mod mic;
mod mode;
mod monitor;
mod phrase;
mod plugin_status;
mod prefs;
mod procedure;
mod pushback;
mod radiofx;
mod rag;
mod readback;
mod simbrief;
mod stt;
mod stt_eval;
mod taxi_guide;
mod taxi_in;
mod terrain;
mod tools;
mod tts;
#[cfg(feature = "onnx-tts")]
mod tts_onnx;
mod ui;
mod vad;
mod voice;

// The world side (ATC) and the simulator plumbing now live in their own
// crates; these shims keep every existing `crate::station::...` /
// `crate::xplane::...` path in the module tree resolving unchanged.
pub(crate) use ::atc::{airspace, atis, nav, party_line, speaker, station, tower};
pub(crate) use sim::{blackbox, datarefs, xplane};

use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};

use audio::{Listener, PttMic};
use config::Config;
use livestate::LiveState;
use llm::Llm;
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
    let file_appender = tracing_appender::rolling::daily("logs", "xvoicelayer.log");
    let (file_writer, _log_guard) = tracing_appender::non_blocking(file_appender);
    // Every workspace crate, not just the crew: since the 2026-07-14 split the
    // world's tracing lives in `atc` and the Web API client's in `sim`, and a
    // crew-only filter silently dropped both — "release watch armed" and "wake
    // clock started" were invisible in flight logs, so a Heathrow departure
    // held for eight minutes had to be diagnosed by probing the live sim
    // instead of grepping (2026-07-27).
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| "xvoicelayer=info,atc=info,sim=info".into());
    let stderr_layer =
        (!tui).then(|| tracing_subscriber::fmt::layer().with_writer(std::io::stderr));
    tracing_subscriber::registry()
        .with(filter)
        .with(
            tracing_subscriber::fmt::layer()
                .with_ansi(false)
                .with_writer(file_writer),
        )
        .with(stderr_layer)
        .init();

    // First line in the log: which build this is, so there's no doubt about whether
    // a rebuilt copilot was actually picked up (pairs with the plugin's build line).
    tracing::info!(
        version = env!("CARGO_PKG_VERSION"),
        build = env!("XCOPILOT_BUILD"),
        "XVoiceLayer starting"
    );

    // Initial app mode: explicit env > the panel's saved pick > both. The
    // panel's Settings tab can change it live from here on (mode.rs).
    let initial_mode = mode::Mode::parse(&config.mode)
        .or_else(|| prefs::mode().as_deref().and_then(mode::Mode::parse))
        .unwrap_or(mode::Mode::Both);
    mode::set(initial_mode);
    tracing::info!(mode = initial_mode.as_str(), "app mode");

    // Cabin crew staffing, same precedence: explicit env > the panel's saved
    // pick > auto (the airframe profile decides). Panel-toggleable live.
    let initial_staffing = cabin::Staffing::parse(&config.cabin_crew)
        .or_else(|| {
            prefs::cabin_crew()
                .as_deref()
                .and_then(cabin::Staffing::parse)
        })
        .unwrap_or(cabin::Staffing::Auto);
    cabin::set_staffing(initial_staffing);
    tracing::info!(cabin_crew = initial_staffing.as_str(), "cabin staffing");

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
        println!(
            "Press a key to see its name (grant Input Monitoring if prompted). Ctrl-C to quit."
        );
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
    // clip to stdout and exits — a small standalone whisper utility.
    if std::env::args().nth(1).as_deref() == Some("transcribe") {
        let path = std::env::args()
            .nth(2)
            .ok_or_else(|| anyhow!("usage: xvl transcribe <audio-file>"))?;
        let text = transcribe_file(&config.whisper_model_path, &path)?;
        println!("{text}");
        return Ok(());
    }

    // `cargo run -- onnx-info <model>` dumps an ONNX model's input/output signature.
    #[cfg(feature = "onnx-tts")]
    if std::env::args().nth(1).as_deref() == Some("onnx-info") {
        let path = std::env::args()
            .nth(2)
            .ok_or_else(|| anyhow!("usage: xvl onnx-info <model.onnx>"))?;
        println!("{}", tts_onnx::inspect(&path)?);
        return Ok(());
    }

    // `cargo run -- tts ["phrase"]` synthesizes one phrase with the configured ONNX
    // voice (honouring the XCOPILOT_TTS_LENGTH/NOISE/NOISE_W scale overrides) and
    // plays it — an audition tool for tuning the voice without launching the loop.
    #[cfg(feature = "onnx-tts")]
    if std::env::args().nth(1).as_deref() == Some("tts") {
        let text = std::env::args().nth(2).unwrap_or_else(|| {
            "Roger, flaps up. Gear down, three green. Caution, fuel endurance forty minutes.".into()
        });
        // Kokoro model → Kokoro engine (voice via XCOPILOT_KOKORO_VOICE); else piper.
        if config.piper_voice.contains("kokoro") {
            let voice =
                std::env::var("XCOPILOT_KOKORO_VOICE").unwrap_or_else(|_| "am_michael".into());
            eprintln!("kokoro: {} voice={}", config.piper_voice, voice);
            tts_onnx::KokoroTts::load(&config.piper_voice, &voice)?.speak(&text)?;
        } else {
            eprintln!(
                "voice: {}  (length={} noise={} noise_w={})",
                config.piper_voice,
                std::env::var("XCOPILOT_TTS_LENGTH").unwrap_or_else(|_| "model".into()),
                std::env::var("XCOPILOT_TTS_NOISE").unwrap_or_else(|_| "model".into()),
                std::env::var("XCOPILOT_TTS_NOISE_W").unwrap_or_else(|_| "model".into())
            );
            tts_onnx::OnnxTts::load(&config.piper_voice)?.speak(&text)?;
        }
        return Ok(());
    }

    // `cargo run -- analyze` mines the logs for fast-path coverage and the
    // ranked LLM-fallback phrasings — the candidates to promote into the
    // deterministic grammar so coverage ratchets up over time.
    if std::env::args().nth(1).as_deref() == Some("analyze") {
        return analyze_commands();
    }

    // `xvl index <aircraft_id> <manual.pdf...>` — build the manual index the
    // answer-only LLM grounds its answers in (see src/rag.rs).
    if std::env::args().nth(1).as_deref() == Some("index") {
        let args: Vec<String> = std::env::args().skip(2).collect();
        let Some((id, pdfs)) = args.split_first() else {
            eprintln!("usage: xvl index <aircraft_id> <manual.pdf> [more.pdf...]");
            std::process::exit(2);
        };
        if pdfs.is_empty() {
            eprintln!("usage: xvl index <aircraft_id> <manual.pdf> [more.pdf...]");
            std::process::exit(2);
        }
        let cfg = config::Config::from_env()?;
        return rag::build_index(&cfg.ollama_base_url, id, pdfs).await;
    }

    // `xvl ask <aircraft_id> <question...>` — retrieval smoke test: print the
    // manual chunks a question would pull, without the voice loop or the LLM.
    if std::env::args().nth(1).as_deref() == Some("ask") {
        let args: Vec<String> = std::env::args().skip(2).collect();
        let Some((id, q)) = args.split_first() else {
            eprintln!("usage: xvl ask <aircraft_id> <question>");
            std::process::exit(2);
        };
        let question = q.join(" ");
        let cfg = config::Config::from_env()?;
        match rag::Index::load(id) {
            Some(index) => match index.retrieve(&cfg.ollama_base_url, &question, 4).await {
                Some(hits) => println!("{hits}"),
                None => println!("(no relevant excerpts above the score floor)"),
            },
            None => println!("no index for '{id}' — build one with: xvl index {id} <pdfs>"),
        }
        return Ok(());
    }

    // `cargo run -- stt-eval [manifest.yaml]` runs the offline speech-accuracy
    // harness: each case through whisper + grammar, reporting WER and command
    // accuracy. `WHISPER_MODEL=...` switches the model under test. Offline, so it
    // uses a fixed profile (no X-Plane needed).
    if std::env::args().nth(1).as_deref() == Some("stt-eval") {
        let manifest = std::env::args()
            .nth(2)
            .unwrap_or_else(|| "bench/stt_cases.yaml".to_string());
        let id = if config.aircraft.eq_ignore_ascii_case("auto") {
            "cessna_172sp".to_string()
        } else {
            config.aircraft.clone()
        };
        let profile =
            aircraft::load(&id).ok_or_else(|| anyhow!("unknown aircraft profile: {id}"))?;
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
        llm = if config.llm_enabled { "answer-only (Ollama)" } else { "off (deterministic)" },
        "starting XVoiceLayer"
    );

    // whisper model load is the one slow init — do it once, up front. Bias its
    // decoding toward the active aircraft's actual systems and phraseology.
    let stt = Stt::load(&config.whisper_model_path, config.whisper_gpu)?;
    let llm = Llm::new(&config);
    if config.llm_enabled && mode::copilot_enabled() {
        // Prefill the answer-path KV cache now, on the ground — not on the
        // first question at altitude (cold prefill measured at ~10 s of GPU
        // time, and the sim's frame rate tanks for all of it).
        let warm = llm.clone();
        tokio::spawn(async move { warm.warmup().await });
    }
    // The tool registry, STT prompt, and checklists are all derived from the
    // profile inside run_session — and rebuilt there when the aircraft switches.

    // Push-to-talk, if configured. When set it takes over capture: the copilot
    // listens only while the button/key is held, so there's no wake word and
    // ambient/ATC can't trigger it. A sim dataref (the bundled X-Plane plugin)
    // takes precedence over a global key — it needs no OS permission.
    let ptt = if !config.ptt_dataref.is_empty() {
        // Leak the names to 'static for the dedicated subscription (lives for
        // the whole run); a one-time leak of two config strings. The radio PTT
        // dataref rides in the same subscription — if the plugin predates it,
        // LiveState just never resolves that name and it reads as not-pressed.
        let name: &'static str = Box::leak(config.ptt_dataref.clone().into_boxed_str());
        let mut names = vec![name];
        if !config.radio_ptt_dataref.is_empty() {
            names.push(Box::leak(config.radio_ptt_dataref.clone().into_boxed_str()));
        }
        let live = std::sync::Arc::new(LiveState::start(&config.xplane_base_url, &names));
        Some(Ptt::Dataref {
            live,
            name: config.ptt_dataref.clone(),
            radio: config.radio_ptt_dataref.clone(),
        })
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

    // The panel's copilot-voice picker: publish the roster, poll for picks,
    // swap live. Only where the pick can matter — the `say` backend ignores
    // piper models entirely.
    if config.tts_backend != "say" {
        spawn_voice_picker(
            config.xplane_base_url.clone(),
            voice.clone(),
            config.piper_voice.clone(),
        );
    }

    // The panel's Settings-tab toggles (app mode, cabin-crew staffing):
    // publish the current values, apply picks live, persist to prefs.
    spawn_settings_poller(config.xplane_base_url.clone());

    // In-sim status panel (the bundled plugin): mirror phase/heard/action into its
    // datarefs. Best-effort — a no-op if the plugin isn't loaded.
    let status_sink = config
        .plugin_status
        .then(|| plugin_status::StatusSink::start(&config.xplane_base_url));
    // Mirror EVERY spoken line onto the panel transcript, so the background
    // flight loops' narration + ATC calls show there, not just the main loop's
    // (the auto-flight was audible but had no transcript, 2026-07-23).
    if let Some(sink) = &status_sink {
        voice.set_status(sink.clone());
    }

    // Radio row + ATC tab for the in-sim panel, refreshed ~1/s through the
    // status sink. Position-driven from the station's published side state,
    // so the row follows region handoffs and shows the active station's comm
    // plan ONLY while it's inside VHF range — a tower you can't raise isn't a
    // radio row. The ATC tab gets every field in range, nearest first. Its
    // own small poller (not the TUI's) so the radios show in the cockpit in
    // every UI mode — the panel is the instrument, the terminal is a dev view.
    if let Some(sink) = status_sink.clone() {
        let base = config.xplane_base_url.clone();
        if !config.atc_airport.trim().is_empty() {
            std::thread::spawn(move || {
                let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                else {
                    return;
                };
                rt.block_on(async move {
                    let xp = XPlaneClient::new(base);
                    loop {
                        let com1 = xp
                            .get_float(datarefs::COM1_FREQ_KHZ)
                            .await
                            .ok()
                            .map(|v| v.round() as i64);
                        if let Some((lat, lon, alt_ft)) = position(&xp).await {
                            let row = match station::active_station_in_range(lat, lon, alt_ft) {
                                Some((airport, services)) => {
                                    radio_panel_line(&airport, &services, com1)
                                }
                                // Out of range (or no station yet): the row keeps
                                // showing where the radio actually is, nothing more.
                                None => radio_panel_line("", &[], com1),
                            };
                            sink.radio(&row);
                            sink.atc(&atc_panel_payload(&station::fields_in_range(
                                lat, lon, alt_ft, 8,
                            )));
                            // The Calls tab's live "say now" line: the coach's
                            // suggested next call for the current tower state.
                            let on_ground = xp
                                .get_float(datarefs::ON_GROUND_ANY)
                                .await
                                .map(|v| v > 0.5)
                                .unwrap_or(true);
                            let callsign = xp
                                .get_string(datarefs::TAILNUM)
                                .await
                                .ok()
                                .map(|t| tower::spoken_callsign(&t))
                                .filter(|s| !s.is_empty())
                                .unwrap_or_else(|| "your callsign".to_string());
                            let dist = station::active_field()
                                .map(|(_, flat, flon, _)| nav::haversine_nm(lat, lon, flat, flon));
                            match station::suggest_call(&callsign, on_ground, dist, com1) {
                                Some(s) => {
                                    let text = [s.first.as_deref(), s.call.as_deref()]
                                        .into_iter()
                                        .flatten()
                                        .collect::<Vec<_>>()
                                        .join("\n");
                                    sink.coach(s.urgent, s.arrival, &text);
                                }
                                None => sink.coach(false, false, ""),
                            }
                        }
                        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
                    }
                });
            });
        }
    }

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
            let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            else {
                return;
            };
            rt.block_on(async move {
                let xp = XPlaneClient::new(base);
                // The last ICAO shown on the panel, so it's published once and on change.
                let mut shown = String::new();
                loop {
                    if let Ok(icao) = xp.aircraft_icao().await {
                        let icao = icao.trim().to_string();
                        if !icao.is_empty() {
                            // Mirror type + callsign to the panel chip
                            // ("C172 N172SP") — the callsign is what ATC
                            // addresses, so the pilot should see it too.
                            let tail = xp
                                .get_string(datarefs::TAILNUM)
                                .await
                                .ok()
                                .map(|t| t.trim().to_uppercase())
                                .filter(|t| !t.is_empty());
                            let chip = match &tail {
                                Some(t) => format!("{icao} {t}"),
                                None => icao.clone(),
                            };
                            if chip != shown {
                                if let Some(s) = det_status.as_ref() {
                                    s.aircraft(&chip);
                                }
                                shown = chip;
                            }
                            let id = icao_to_profile(&icao);
                            if id != live.id() {
                                if let Some(p) = aircraft::load(&id) {
                                    let name = p.display_name.clone();
                                    tracing::info!(icao = %icao, profile = %id, "aircraft changed; switching profile");
                                    // A manual/native push continues inside
                                    // X-Plane after its one-shot command. Stop
                                    // it before clearing its published state
                                    // and swapping actuator selectors.
                                    if pushback::ACTIVE.load(
                                        std::sync::atomic::Ordering::SeqCst,
                                    ) {
                                        let _ = xp
                                            .run_command(tools::ground::CMD_PUSH_STOP)
                                            .await;
                                    }
                                    live.swap(p);
                                    // Mirror OR action, never both: voice.alert already
                                    // posts the spoken line to the panel transcript, so a
                                    // separate s.alert here double-wrote it (blue + green).
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

    // The cabin crew's state, shared between the command loop (interphone
    // replies to addressed calls) and the monitor (owed secure reports, the
    // seatbelt-sign watch). Exists regardless of airframe; whether anyone is
    // actually aboard is checked at the call sites (profile flag/env).
    let cabin_state = std::sync::Arc::new(std::sync::Mutex::new(cabin::Cabin::new()));

    // Background monitor: watches for problems and speaks up. Its own thread +
    // runtime + X-Plane client, sharing the speech arbiter. Runs in both UIs.
    // Reads the live profile so its limits switch with the aircraft.
    if config.monitor_enabled {
        let mon_voice = voice.clone();
        let mon_config = config.clone();
        let mon_live = live_profile.clone();
        let mon_status = status_sink.clone();
        let mon_cabin = cabin_state.clone();
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            else {
                return;
            };
            rt.block_on(monitor::run(
                mon_config, mon_live, mon_voice, mon_status, mon_cabin,
            ));
        });
    }

    if config.ui == "tui" {
        // TUI: voice loop + aircraft poll run on background threads, each with
        // its own runtime; the dashboard renders on the main thread.
        let (tx, rx) = std::sync::mpsc::channel::<ui::Report>();
        let title = format!(
            "XVoiceLayer {} ({}) · {}",
            env!("CARGO_PKG_VERSION"),
            env!("XCOPILOT_BUILD"),
            profile.display_name
        );
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
        let atc_on = !config.atc_airport.trim().is_empty();
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            else {
                return;
            };
            rt.block_on(async move {
                let xp = XPlaneClient::new(poll_base);
                loop {
                    if let Some(ac) = poll_aircraft(&xp).await {
                        if tx_poll.send(ui::Report::Aircraft(ac)).is_err() {
                            break; // dashboard gone
                        }
                    }
                    // The radio panel mirrors the in-sim one: the ACTIVE
                    // station's comm plan, only while it's in VHF range —
                    // empty (panel hidden) otherwise.
                    if atc_on {
                        let com1 = xp
                            .get_float(datarefs::COM1_FREQ_KHZ)
                            .await
                            .ok()
                            .map(|v| v.round() as i64);
                        if let Some((lat, lon, alt_ft)) = position(&xp).await {
                            let (airport, services) =
                                station::active_station_in_range(lat, lon, alt_ft)
                                    .unwrap_or_default();
                            let _ = tx_poll.send(ui::Report::Radio(ui::Radio {
                                airport,
                                services,
                                com1_khz: com1,
                            }));
                        }
                    }
                    tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
                }
            });
        });

        // Typed-command channel: the dashboard (main thread) sends commands the
        // pilot types; the voice loop picks them up alongside spoken utterances.
        // The plugin panel's emergency buttons feed the same channel.
        let (cmd_tx, cmd_rx) = std::sync::mpsc::channel::<String>();
        spawn_panel_request_poller(config.xplane_base_url.clone(), cmd_tx.clone());
        let mut reporter = Reporter::tui(tx);
        if let Some(s) = status_sink.clone() {
            reporter = reporter.with_status(s);
        }
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            else {
                return;
            };
            if let Err(e) = rt.block_on(run_session(
                config,
                stt,
                llm,
                live_profile,
                voice,
                continuous,
                explicit,
                ptt,
                reporter,
                Some(cmd_rx),
                cabin_state,
            )) {
                tracing::error!("voice loop ended: {e:#}");
            }
        });

        ui::run_dashboard(title, engines, commands, rx, cmd_tx)?; // blocks until the user quits
        Ok(())
    } else {
        println!(
            "\n  XVoiceLayer — {} ({}). Ctrl-C to quit.\n",
            profile.display_name, config.listen_mode
        );
        let mut reporter = Reporter::plain();
        if let Some(s) = status_sink {
            reporter = reporter.with_status(s);
        }
        // No typed input in plain mode, but the panel's emergency buttons still
        // need a way in — same channel, same consumer.
        let (cmd_tx, cmd_rx) = std::sync::mpsc::channel::<String>();
        spawn_panel_request_poller(config.xplane_base_url.clone(), cmd_tx);
        run_session(
            config,
            stt,
            llm,
            live_profile,
            voice,
            continuous,
            explicit,
            ptt,
            reporter,
            Some(cmd_rx),
            cabin_state,
        )
        .await
    }
}

/// Poll the plugin panel's request dataref (`xcopilot/request`) and turn button
/// presses into commands on the typed-command channel — the phrases live in
/// [`auto_status::REQUEST_PHRASES`], the same English a pilot would speak, so
/// a button can never do anything the voice can't. CANCEL (code 11) has no
/// fixed phrase: the cancel is per-automation in the grammar, so the phrase is
/// picked from whatever is actually running. The value is consumed (written
/// back to 0) once injected, so a press fires exactly once. Best-effort like the
/// status sink: no plugin / no X-Plane just means the buttons do nothing.
fn spawn_panel_request_poller(base_url: String, tx: std::sync::mpsc::Sender<String>) {
    std::thread::spawn(move || {
        let Ok(http) = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(2))
            .build()
        else {
            return;
        };
        let rest = format!("{}/api/v1", base_url.trim_end_matches('/'));
        let mut id: Option<i64> = None;
        loop {
            std::thread::sleep(std::time::Duration::from_millis(300));
            if id.is_none() {
                id = crate::plugin_status::resolve_id(&http, &rest, "xcopilot/request");
                if id.is_none() {
                    // Plugin not loaded / sim down — retry lazily.
                    std::thread::sleep(std::time::Duration::from_secs(3));
                    continue;
                }
            }
            let Some(dref) = id else { continue };
            let url = format!("{rest}/datarefs/{dref}/value");
            let value = http
                .get(&url)
                .send()
                .ok()
                .filter(|r| r.status().is_success())
                .and_then(|r| r.json::<serde_json::Value>().ok())
                .and_then(|b| b.get("data").and_then(|d| d.as_i64()));
            let Some(v) = value else {
                id = None; // stale id (sim restarted) — re-resolve
                continue;
            };
            let cmd = if v == auto_status::REQUEST_CANCEL {
                auto_status::cancel_phrase()
            } else {
                auto_status::REQUEST_PHRASES
                    .iter()
                    .find(|(code, _)| *code == v)
                    .map(|(_, phrase)| *phrase)
            };
            if v != 0 {
                tracing::info!(request = v, cmd, "panel button request");
                // Consume first so a slow channel can't double-fire the
                // request — and even a code with nothing to say (CANCEL with
                // nothing running, an unknown code from a newer plugin) is
                // consumed, or it would re-log every 300 ms forever.
                let _ = http
                    .patch(&url)
                    .json(&serde_json::json!({ "data": 0 }))
                    .send();
            }
            let Some(cmd) = cmd else { continue };
            if tx.send(cmd.to_string()).is_err() {
                return; // session gone
            }
        }
    });
}

/// The panel's copilot-voice picker (see plugin/src/xcopilot.cpp): the app
/// publishes the roster + current index, the Settings combo writes a pick back.
const DR_OPT_VOICE: &str = "xcopilot/opt/voice";
const DR_OPT_VOICE_LIST: &str = "xcopilot/opt/voice_list";
/// The panel's app-mode radio buttons (Both / Copilot only / ATC only) —
/// values are `mode::Mode::panel_index`, `-1` = plugin default (unpublished).
const DR_OPT_MODE: &str = "xcopilot/opt/mode";
/// The panel's cabin-crew radio buttons (Auto / Aboard / Off) — values are
/// `cabin::Staffing::panel_index`, `-1` = plugin default (unpublished).
const DR_OPT_CABIN: &str = "xcopilot/opt/cabin_crew";
/// The panel's UI-scale radio buttons (100% / 125% / 150%, index 0/1/2) —
/// purely a panel concern; the app remembers the pick and republishes it
/// across plugin reloads and boots. `-1` = plugin default (unpublished).
const DR_OPT_UI_SCALE: &str = "xcopilot/opt/ui_scale";
/// The UI scale the app currently holds for the publish/pick protocol.
static UI_SCALE: std::sync::atomic::AtomicI64 = std::sync::atomic::AtomicI64::new(0);

/// Publish the pilot-voice roster (every piper model on disk) to the plugin's
/// Settings picker, then poll for picks. A pick swaps the copilot's voice live
/// ([`Voice::set_pilot_voice`]), confirms with a short line spoken in the NEW
/// voice, and persists to `prefs.json` so the next session boots with it (env
/// `PIPER_VOICE` still beats the saved pick — see [`config`]). Best-effort like
/// the status sink: no plugin, no X-Plane, or no models just means the picker
/// stays empty. The roster re-publishes whenever the index reads -1 with a pick
/// on record — that's the plugin's freshly-registered default, i.e. a plugin
/// reload or sim restart wiped the panel's state.
fn spawn_voice_picker(base_url: String, voice: Voice, current_spec: String) {
    std::thread::spawn(move || {
        use base64::Engine;
        let choices = voice::pilot_voice_choices();
        if choices.is_empty() {
            return;
        }
        // Display names are the model file stems ("en_US-amy-medium") —
        // unambiguous, and quality variants (ryan-high vs ryan-medium) stay
        // distinct. Newline-separated for the plugin's combo.
        let roster: String = choices
            .iter()
            .map(|p| {
                std::path::Path::new(p)
                    .file_stem()
                    .map(|s| s.to_string_lossy().into_owned())
                    .unwrap_or_else(|| p.clone())
            })
            .collect::<Vec<_>>()
            .join("\n");
        let roster_b64 = base64::engine::general_purpose::STANDARD.encode(roster.as_bytes());
        let Ok(http) = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(2))
            .build()
        else {
            return;
        };
        let rest = format!("{}/api/v1", base_url.trim_end_matches('/'));
        // The index the panel should show right now: the active voice's spot in
        // the roster, or -1 when it isn't a roster entry (custom env path).
        let mut applied: i64 = choices
            .iter()
            .position(|c| *c == current_spec)
            .map_or(-1, |i| i as i64);
        let mut ids: Option<(i64, i64)> = None; // (list, index)

        let patch = |id: i64, value: serde_json::Value| -> bool {
            http.patch(format!("{rest}/datarefs/{id}/value"))
                .json(&serde_json::json!({ "data": value }))
                .send()
                .map(|r| r.status().is_success())
                .unwrap_or(false)
        };
        loop {
            std::thread::sleep(std::time::Duration::from_millis(500));
            if ids.is_none() {
                ids = crate::plugin_status::resolve_id(&http, &rest, DR_OPT_VOICE_LIST)
                    .zip(crate::plugin_status::resolve_id(&http, &rest, DR_OPT_VOICE));
                let Some((list_id, idx_id)) = ids else {
                    // Plugin not loaded / sim down — retry lazily.
                    std::thread::sleep(std::time::Duration::from_secs(3));
                    continue;
                };
                if !patch(list_id, serde_json::json!(roster_b64))
                    || !patch(idx_id, serde_json::json!(applied))
                {
                    ids = None;
                    continue;
                }
            }
            let Some((list_id, idx_id)) = ids else {
                continue;
            };
            let picked = http
                .get(format!("{rest}/datarefs/{idx_id}/value"))
                .send()
                .ok()
                .filter(|r| r.status().is_success())
                .and_then(|r| r.json::<serde_json::Value>().ok())
                .and_then(|b| b.get("data").and_then(|d| d.as_i64()));
            let Some(v) = picked else {
                ids = None; // stale ids (sim restarted) — re-resolve
                continue;
            };
            if v < 0 {
                // The plugin's registered default: its state was wiped (plugin
                // reload) while our ids stayed valid — repopulate the picker.
                if applied >= 0
                    && (!patch(list_id, serde_json::json!(roster_b64))
                        || !patch(idx_id, serde_json::json!(applied)))
                {
                    ids = None;
                }
                continue;
            }
            if v == applied || v as usize >= choices.len() {
                continue;
            }
            applied = v;
            let spec = choices[v as usize].clone();
            tracing::info!(model = %spec, "panel voice pick");
            voice.set_pilot_voice(spec.clone());
            prefs::save_pilot_voice(&spec);
            // Confirmed in the new voice itself: the swap applies before the
            // next intercom utterance, which is exactly this line.
            voice.say(voice::Priority::Advisory, "New voice, checked in.");
        }
    });
}

/// Sync the app's Settings-tab toggles with the plugin: the app mode
/// (`xcopilot/opt/mode`) and cabin-crew staffing (`xcopilot/opt/cabin_crew`).
/// Publish the current values so the radio buttons show the truth, and apply
/// a panel pick the moment it lands (mode: the main loop's reconciliation
/// block notices within ~2 s; staffing: read live at the routing sites).
/// Picks persist to prefs.json as the next boot's default. Same lifecycle
/// discipline as the voice picker: `-1` means the plugin reloaded and lost
/// its state, so the truth is re-published rather than read back as a pick.
fn spawn_settings_poller(base_url: String) {
    /// One panel toggle: its dataref, what the app currently holds, and how
    /// to apply a pick (an unknown index is ignored).
    struct PanelSetting {
        dref: &'static str,
        id: Option<i64>,
        current: fn() -> i64,
        apply: fn(i64),
    }
    let mut settings = [
        PanelSetting {
            dref: DR_OPT_MODE,
            id: None,
            current: || mode::get().panel_index(),
            apply: |v| {
                if let Some(m) = mode::Mode::from_panel_index(v) {
                    tracing::info!(mode = m.as_str(), "panel mode pick");
                    mode::set(m);
                    prefs::save_mode(m.as_str());
                }
            },
        },
        PanelSetting {
            dref: DR_OPT_CABIN,
            id: None,
            current: || cabin::staffing().panel_index(),
            apply: |v| {
                if let Some(s) = cabin::Staffing::from_panel_index(v) {
                    tracing::info!(cabin_crew = s.as_str(), "panel cabin staffing pick");
                    cabin::set_staffing(s);
                    prefs::save_cabin_crew(s.as_str());
                }
            },
        },
        PanelSetting {
            dref: DR_OPT_UI_SCALE,
            id: None,
            current: || UI_SCALE.load(std::sync::atomic::Ordering::Relaxed),
            apply: |v| {
                if (0..=2).contains(&v) {
                    tracing::info!(ui_scale = v, "panel UI scale pick");
                    UI_SCALE.store(v, std::sync::atomic::Ordering::Relaxed);
                    prefs::save_ui_scale(v);
                }
            },
        },
    ];
    // The scale is pure remembered state — seed it from prefs so the first
    // publish shows the saved pick, not a hardcoded default.
    UI_SCALE.store(
        prefs::ui_scale().unwrap_or(0),
        std::sync::atomic::Ordering::Relaxed,
    );
    std::thread::spawn(move || {
        let Ok(http) = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(2))
            .build()
        else {
            return;
        };
        let rest = format!("{}/api/v1", base_url.trim_end_matches('/'));
        let patch = |id: i64, value: i64| -> bool {
            http.patch(format!("{rest}/datarefs/{id}/value"))
                .json(&serde_json::json!({ "data": value }))
                .send()
                .map(|r| r.status().is_success())
                .unwrap_or(false)
        };
        loop {
            std::thread::sleep(std::time::Duration::from_millis(500));
            for s in settings.iter_mut() {
                let applied = (s.current)();
                if s.id.is_none() {
                    s.id = crate::plugin_status::resolve_id(&http, &rest, s.dref);
                    let Some(id) = s.id else {
                        // Plugin not loaded / sim down — retry lazily.
                        std::thread::sleep(std::time::Duration::from_secs(3));
                        continue;
                    };
                    if !patch(id, applied) {
                        s.id = None;
                        continue;
                    }
                }
                let Some(id) = s.id else { continue };
                let read = http
                    .get(format!("{rest}/datarefs/{id}/value"))
                    .send()
                    .ok()
                    .filter(|r| r.status().is_success())
                    .and_then(|r| r.json::<serde_json::Value>().ok())
                    .and_then(|b| b.get("data").and_then(|d| d.as_i64()));
                let Some(v) = read else {
                    s.id = None; // stale id (sim restarted) — re-resolve
                    continue;
                };
                if v < 0 {
                    // Plugin reloaded and lost its state — re-publish the truth.
                    if !patch(id, applied) {
                        s.id = None;
                    }
                    continue;
                }
                if v != applied {
                    (s.apply)(v);
                }
            }
        }
    });
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
    /// X-Plane plugin's `xcopilot/ptt_active`). No OS permission. `radio` is
    /// the transmit-switch dataref (`xcopilot/radio_ptt_active`): held = the
    /// utterance is a radio transmission for the tuned ATC station, not a
    /// copilot command. Empty = no radio PTT bound.
    Dataref {
        live: std::sync::Arc<LiveState>,
        name: String,
        radio: String,
    },
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
        tools::radio::publish_mode_knob(p.transponder_mode_knob.clone());
        aircraft::publish_throttle_dataref(p.throttle_dataref.clone());
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
        // A spawned controller owns parameters from the profile that armed it.
        // Stop every owner before changing the global actuator selectors;
        // otherwise an old takeoff/autoland loop can start driving the newly
        // loaded aircraft with the previous airframe's speeds and limits.
        stand_down_for_aircraft_change();
        // Republished on EVERY swap, including to None: a selector that
        // outlives its airframe aims the next aircraft's mode writes at a knob
        // it doesn't have (the same shape as the radio-power flag that latched
        // across a 172 -> Twin Otter change, 2026-07-27).
        tools::radio::publish_mode_knob(p.transponder_mode_knob.clone());
        aircraft::publish_throttle_dataref(p.throttle_dataref.clone());
        *self.inner.write().unwrap() = p;
        self.generation
            .fetch_add(1, std::sync::atomic::Ordering::Release);
    }
}

/// Release every continuous manager before an aircraft profile changes.
///
/// The high-level aborts cascade through their sub-loops; the direct atomic
/// clears cover managers that can also run independently. Native pushback's
/// command is stopped by the async detection poller, which has the X-Plane
/// client needed to send it.
fn stand_down_for_aircraft_change() {
    use std::sync::atomic::Ordering::SeqCst;

    full_auto::abort();
    circuit::abort();
    procedure::abort();
    autoland::abort();
    tools::recover::abort();
    approach_guide::cancel();
    autotakeoff::ACTIVE.store(false, SeqCst);
    autotaxi::ACTIVE.store(false, SeqCst);
    autotaxi::BACKTRACKING.store(false, SeqCst);
    tools::speedhold::ACTIVE.store(false, SeqCst);
    tools::reverse::ACTIVE.store(false, SeqCst);
    tools::engine::MIX_MANAGED.store(false, SeqCst);
    tools::engine::LEAN_SEEKING.store(false, SeqCst);
    pushback::ACTIVE.store(false, SeqCst);
}

impl Ptt {
    /// True while the PTT dataref reads pressed (only meaningful for `Dataref`).
    fn dataref_active(live: &LiveState, name: &str) -> bool {
        live.get(name).unwrap_or(0.0) > 0.5
    }

    /// True while PTT is held, whichever source (intercom OR radio) — for the
    /// mic capture and content routing: keying either button starts a capture.
    fn is_pressed(&self) -> bool {
        self.intercom_pressed() || self.radio_pressed()
    }

    /// The INTERCOM PTT: talking to the copilot (crew). On a real audio panel
    /// this is the PILOT/COPLT intercom loop — separate from radio receive, so
    /// keying it cuts only the copilot's own voice, never ATC. The keyboard
    /// PTT is the intercom key (there's no separate radio key).
    fn intercom_pressed(&self) -> bool {
        match self {
            Ptt::Key(key) => {
                use device_query::DeviceQuery;
                device_query::DeviceState::new().get_keys().contains(key)
            }
            Ptt::Dataref { live, name, .. } => Ptt::dataref_active(live, name),
        }
    }

    /// The RADIO (transmit) PTT: keying the mic to talk to ATC. Half-duplex —
    /// while it's held you can't hear the frequency, so station audio is
    /// stepped on. Dataref-only (`xcopilot/radio_ptt_active`); a keyboard PTT
    /// is never the radio.
    fn radio_pressed(&self) -> bool {
        match self {
            Ptt::Key(_) => false,
            Ptt::Dataref { live, radio, .. } => {
                !radio.is_empty() && Ptt::dataref_active(live, radio)
            }
        }
    }

    /// A dedicated radio (transmit) key is BOUND — the pilot has two channels,
    /// like a real audio panel. Then physical routing beats content routing in
    /// BOTH directions: radio-keyed speech never reaches the copilot, and
    /// intercom-keyed speech never reaches ATC ("requesting clearance" said to
    /// the COPILOT was answered by the tower, 2026-07-12). Without a radio key
    /// (keyboard PTT, continuous mode) content routing is the only way to
    /// reach ATC at all, so it stays.
    fn has_radio_channel(&self) -> bool {
        match self {
            Ptt::Key(_) => false,
            Ptt::Dataref { radio, .. } => !radio.is_empty(),
        }
    }
}

#[allow(clippy::too_many_arguments)]
/// Should this call run the step-by-step power-up procedure instead of
/// set_electrical's one-shot "all"? Only whole-system *on* qualifies: a missing
/// "on" (LLM sometimes omits it for "power up") counts as on, but an explicit
/// off must stay set_electrical — remapping it once turned "electrics off" into
/// powering the aircraft back up.
fn wants_procedural_power_up(name: &str, args: &serde_json::Value) -> bool {
    name == "set_electrical"
        && args.get("system").and_then(|s| s.as_str()) == Some("all")
        && args.get("on").and_then(|v| v.as_bool()) != Some(false)
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
    cabin_state: std::sync::Arc<std::sync::Mutex<cabin::Cabin>>,
) -> Result<()> {
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
    let xp_radio_muted = config.mute_xp_radio && !config.atc_airport.trim().is_empty();
    if xp_radio_muted && mode::atc_enabled() {
        match xplane
            .set_float("sim/operation/sound/radio_volume_ratio", 0.0)
            .await
        {
            Ok(()) => tracing::info!(
                "muted X-Plane's radio voice (XCOPILOT_MUTE_XP_RADIO=false to keep it)"
            ),
            Err(e) => tracing::debug!("could not mute X-Plane radio audio: {e:#}"),
        }
    }
    // Subscribe to the continuously-read state over the WebSocket so snapshots are
    // instant cache reads instead of a REST burst. Best-effort: reads fall back to
    // REST until (or unless) the subscription warms up.
    let live = LiveState::start(&config.xplane_base_url, SNAPSHOT_DATAREFS);

    // The panel button can enable the LLM mid-flight; warm the answer-path KV
    // cache on the off-to-on edge so the first question never pays cold
    // prefill (~10 s of GPU steal, measured 2026-07-08). Startup-enabled runs
    // are warmed from main already.
    {
        let warm = llm.clone();
        let live = live.clone();
        let mut warmed = config.llm_enabled;
        tokio::spawn(async move {
            loop {
                tokio::time::sleep(std::time::Duration::from_secs(3)).await;
                match live.get(DR_LLM_ENABLED) {
                    Some(v) if v >= 0.5 && !warmed => {
                        warmed = true;
                        warm.warmup().await;
                    }
                    // Explicit off: re-warm on the next enable (the cache may
                    // have been evicted while the LLM sat unused).
                    Some(v) if (-0.5..0.5).contains(&v) => warmed = false,
                    _ => {}
                }
            }
        });
    }

    // Profile-derived state, rebuilt whenever the aircraft switches (see the
    // generation check at the top of the loop). Seeded from the live profile.
    let mut active_gen = live_profile.generation();
    let mut profile = live_profile.get();
    let mut registry = Registry::for_aircraft(&profile);
    let mut checklists = checklist::load(&profile).unwrap_or_else(|e| {
        tracing::error!("checklist load failed: {e:#}; running without checklists");
        Vec::new()
    });
    // The aircraft's manual index (built with `xvl index <id> <pdfs>`), for
    // grounding the answer-only LLM in the POM. Absent = answers from state alone.
    let mut manual_index = rag::Index::load(&profile.id);
    if manual_index.is_some() {
        tracing::info!(aircraft = %profile.id, "manual index loaded");
    }
    // The shared general-aviation index (the FAA handbooks: PHAK, AFH, IFH, and
    // friends) rides alongside the POM for the questions no POM answers —
    // airspace, weather theory, aerodynamics, procedures. Aircraft-independent,
    // so it never reloads on an aircraft switch.
    let general_index = rag::Index::load(rag::GENERAL_ID);
    if general_index.is_some() {
        tracing::info!("general aviation index loaded");
    }
    // The ATC station (tower dialogue), merged from the standalone atc binary:
    // it answers when COM1 is tuned to its tower frequency and the words parse
    // as a radio call. Read-only toward the sim; speaks in its own voice
    // through the same arbiter. None = no station, copilot behaves as before —
    // and copilot-only mode never loads one (the radio belongs to external ATC).
    let mut atc_station = if mode::atc_enabled() {
        install_home_station(&config, &voice)
    } else {
        None
    };
    let home_airport_name = atc_station.as_ref().map(|s| s.airport_name().to_string());
    // The mode the loop has already applied — a panel/env change is noticed
    // at the top of the next iteration (≤2 s: idle ticks) and reconciled.
    let mut applied_mode = mode::get();
    // ATC-only: the crew note "copilot is off" is spoken once, not per utterance.
    let mut atc_only_noted = false;
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
    // Last time the crew read the full comm plan after a dead-air transmission.
    // The 2026-07-08 night flight called Malaga, Jerez and Cadiz in a row and got
    // the whole Gibraltar briefing every time — once is help, thrice is butting in.
    let mut last_void_brief: Option<Instant> = None;
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
                            wi.store(true, std::sync::atomic::Ordering::Relaxed);
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
    let mut listener = if continuous {
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
        // True when the RADIO PTT carried this utterance: it was TRANSMITTED,
        // so it belongs to whatever station COM1 is tuned to, never the copilot.
        let mut from_radio = false;
        let input = next_input(&mut listener, &ptt_mic, &stt, &ptt, typed.as_ref());

        // Reconcile a mode change (panel toggle or the poller's pick) — on
        // the idle tick too, so a flip lands within ~2 s of the click even
        // in a silent cockpit. The world side is torn down/brought up here;
        // the monitor's ticks and the routing gates below read mode::* live.
        let now_mode = mode::get();
        if now_mode != applied_mode {
            tracing::info!(
                from = applied_mode.as_str(),
                to = now_mode.as_str(),
                "mode change"
            );
            blackbox::record(
                "mode",
                serde_json::json!({ "from": applied_mode.as_str(), "to": now_mode.as_str() }),
            );
            // World side.
            if mode::atc_enabled() && atc_station.is_none() {
                atc_station = install_home_station(&config, &voice);
                // Reclaim the radio soundscape from X-Plane's own ATC voice.
                if xp_radio_muted {
                    let _ = xplane
                        .set_float("sim/operation/sound/radio_volume_ratio", 0.0)
                        .await;
                }
                let line = match atc_station.as_ref() {
                    Some(st) => format!(
                        "ATC is on. {} is up.",
                        station::spoken_airport(st.airport_name())
                    ),
                    None => "ATC is on, no station configured.".to_string(),
                };
                reporter.log(ui::LogKind::Info, line.clone());
                voice.alert(Priority::Advisory, line);
            } else if !mode::atc_enabled() && atc_station.is_some() {
                atc_station = None;
                // Retire every published fact (party line + monitor side
                // state) and cut whatever the station was saying mid-word.
                station::Station::uninstall();
                tts::stop_channel(tts::Channel::Radio);
                // And give X-Plane its radio voice back — the pilot may be
                // switching to the sim's OWN ATC, whose controllers we muted
                // at boot so two ATISes wouldn't share one frequency.
                if xp_radio_muted {
                    let _ = xplane
                        .set_float("sim/operation/sound/radio_volume_ratio", 1.0)
                        .await;
                }
                reporter.log(
                    ui::LogKind::Info,
                    "ATC off; the radio belongs to the real world now".to_string(),
                );
                voice.alert(
                    Priority::Advisory,
                    "ATC is off. Radio is all yours.".to_string(),
                );
            }
            // Crew side (edges only: AtcOnly is the one mode without a crew).
            if applied_mode == mode::Mode::AtcOnly && now_mode != mode::Mode::AtcOnly {
                // Coming back from ATC-only: re-warm the LLM off the critical
                // path so the first question doesn't pay cold prefill.
                if config.llm_enabled {
                    let warm = llm.clone();
                    tokio::spawn(async move { warm.warmup().await });
                }
                atc_only_noted = false;
                voice.alert(Priority::Advisory, "Copilot back with you.".to_string());
            } else if now_mode == mode::Mode::AtcOnly && applied_mode != mode::Mode::AtcOnly {
                // Leaving the crew behind: cut any copilot speech and evict
                // queued crew lines too, then evict the model so the GPU goes
                // back to the sim. Radio traffic stays on the air.
                voice.dismiss_crew();
                if config.llm_enabled {
                    let ev = llm.clone();
                    tokio::spawn(async move { ev.evict().await });
                }
            }
            applied_mode = now_mode;
        }

        // You call it, you get it: a keyed transmission on a frequency the
        // active station doesn't cover, but another in-range field does, arms
        // the swap so THAT field answers this very call — the ATC tab lets the
        // pilot tune any in-range field, and calling Badajoz on 122.100 must
        // not be answered by dead air and a note about Gibraltar (2026-07-10).
        // BEFORE the pending-station consumption below, so the swap lands for
        // this very utterance.
        if mode::atc_enabled() && matches!(input, Some(Input::Spoken(_, true))) {
            if let Ok(v) = xplane.get_float(datarefs::COM1_FREQ_KHZ).await {
                let com1 = v.round() as i64;
                let covered = atc_station.as_ref().is_some_and(|st| st.covers_khz(com1));
                if !covered {
                    if let Some((lat, lon, alt_ft)) = position(&xplane).await {
                        station::arm_station_for_freq(com1, lat, lon, alt_ft, Instant::now());
                    }
                }
            }
        }

        // A region switch armed by the monitor: rebuild the station BEFORE
        // this utterance is routed (consumed at the loop top it lagged one
        // call behind, seen live 2026-07-09) — and on the IDLE tick too:
        // input is already captured, and a silent cockpit must still swap
        // (the Heathrow handoff sat unconsumed for minutes, 2026-07-10, and
        // the party line waited on it).
        if let Some((icao, apt)) =
            party_line::take_pending_station().filter(|_| mode::atc_enabled())
        {
            let name = apt.name.clone();
            match station::Station::from_airport(apt) {
                Some(mut st) => {
                    reporter.log(ui::LogKind::Info, format!("ATC region: now working {name}"));
                    st.set_enforce_readback(config.atc_readback);
                    st.set_dialect(station::dialect_for(&icao));
                    st.install();
                    // The new field's taxi network, in the background.
                    station::spawn_taxi_net_load(config.apt_data_path.clone(), icao.clone());
                    // And its ILS chart data, same deal.
                    station::spawn_ils_load(
                        config.nav_data_path.clone(),
                        config.cifp_dir.clone(),
                        icao.clone(),
                    );
                    // A new field, a new crew: per-airport voices from the
                    // pool, with the country's own accent (by ICAO prefix)
                    // manning the tower and the ATIS — unless we're back
                    // home, where the curated voices belong.
                    if home_airport_name.as_deref() == Some(&name) {
                        voice.set_station_cast(0, Vec::new());
                    } else {
                        voice.set_station_cast(
                            crate::voice::station_voice_seed(&name),
                            crate::voice::local_voices_on_disk(&icao),
                        );
                    }
                    atc_station = Some(st);
                }
                None => {
                    tracing::warn!(%name, "region handoff to a field without a tower; keeping the old station")
                }
            }
        }

        // A held takeoff clearance the monitor released once the final
        // cleared: sync the tower FSM (state + departure handoff) with what
        // was already said on frequency.
        if party_line::take_pending_takeoff_clearance() {
            if let Some(st) = atc_station.as_mut() {
                st.apply_monitor_takeoff_clearance(xplane.view()).await;
                reporter.log(ui::LogKind::Info, "tower: held takeoff clearance released");
            }
        }

        // The monitor sequenced a held departure onto the runway while the
        // wake clock runs: same sync, one state earlier. The hold stays armed
        // — the release is still to come, now with the aircraft actually at
        // the front of the queue.
        if party_line::take_pending_line_up() {
            if let Some(st) = atc_station.as_mut() {
                st.apply_monitor_line_up(xplane.view()).await;
                reporter.log(
                    ui::LogKind::Info,
                    "tower: held departure sequenced, line up and wait",
                );
            }
        }

        // A go-around the monitor ordered (traffic on the runway): the
        // landing clearance is void in the FSM too, so the coach's next
        // line is the go-around acknowledgment, never the dead clearance's
        // readback (seen live 2026-07-14: SAY kept "cleared to land" while
        // the tower shouted go around, and the pilot dutifully read it back).
        if party_line::take_pending_go_around() {
            if let Some(st) = atc_station.as_mut() {
                st.apply_monitor_go_around();
                reporter.log(
                    ui::LogKind::Info,
                    "tower: go-around ordered, landing clearance void",
                );
            }
        }

        // Radio calls the copilot makes during an auto circuit (position
        // reports; the tower answers and, on the final call, clears us to
        // land). Crew -> world: the copilot transmits exactly as if the pilot
        // keyed the radio. Drained every tick (idle included) so a report made
        // while the cockpit is silent still goes out. In copilot-only mode or
        // with no station, the calls are discarded — external ATC owns the
        // radio then, the same rule a keyed transmission follows below.
        {
            let calls = circuit::take_crew_radio();
            // A discarded transmission is never silent. The queue is drained
            // unconditionally, so a failed gate loses the call for good — and
            // the caller (full auto) reads the resulting silence as the TOWER
            // not answering, which sent it hunting a clearance that was never
            // asked for (Heathrow 2026-07-27).
            if !calls.is_empty() && mode::atc_enabled() && !voice.radio_powered() {
                tracing::warn!(
                    calls = calls.len(),
                    "crew radio call dropped: the radio is unpowered"
                );
            }
            if !calls.is_empty() && mode::atc_enabled() && voice.radio_powered() {
                if let Some(st) = atc_station.as_mut() {
                    for call in calls {
                        // The copilot transmits (its own voice), then the tower.
                        // Both sides logged so the full ATC transcript shows in
                        // the log, not just the panel.
                        tracing::info!(target: "xvoicelayer::radio", copilot = %call, "tx");
                        say(
                            &voice,
                            &reporter,
                            ui::LogKind::Readback,
                            &format!("{call}."),
                        );
                        if let Some((reply, who)) = st
                            .handle_transmission(&call.to_ascii_lowercase(), xplane.view())
                            .await
                        {
                            if !reply.is_empty() {
                                tracing::info!(target: "xvoicelayer::radio", station = %who.tag(), reply = %reply, "rx");
                                reporter
                                    .log(ui::LogKind::Readback, format!("{}: {reply}", who.tag()));
                                voice.say_station(Priority::Readback, who, &reply);
                            }
                        }
                    }
                }
            }
        }

        // Station work consumed — an idle tick carries no utterance to route.
        let (mut command, from_text) = match input {
            None => continue,
            Some(Input::Typed(text)) => {
                tracing::info!(typed = %text, "typed command");
                reporter.log(ui::LogKind::Heard, format!("⌨  \"{text}\""));
                (text, true)
            }
            Some(Input::Spoken(heard, radio)) => {
                tracing::info!(transcript = %heard.text, confidence = heard.confidence, radio, "heard utterance");
                reporter.heard(&heard.text, heard.confidence);
                spoken_confidence = Some(heard.confidence);
                from_radio = radio;
                // Blackbox every spoken capture with its confidence, so the drop /
                // confirm / act thresholds can be tuned against the real distribution
                // after a flight (none was logged before, so the bands are set blind).
                blackbox::record(
                    "heard",
                    serde_json::json!({
                        "transcript": &heard.text, "confidence": heard.confidence,
                        "radio": radio, "explicit": explicit,
                    }),
                );
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
            manual_index = rag::Index::load(&profile.id);
            stt.set_prompt(stt::build_prompt(&profile));
            stt.set_radio_prompt(format!(
                "{} {}",
                stt::build_prompt(&profile),
                tower::STT_PROMPT
            ));
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
                            let ctx = ToolCtx {
                                xplane: &xplane,
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
                        && xplane
                            .run_command("sim/autopilot/servos_off_any")
                            .await
                            .is_ok()
                    {
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
            continue;
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
            continue;
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
            continue;
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
            continue;
        }

        // Set when a marginal-confidence command is let through to be read back
        // for a yes/no instead of dropped (the confirm loop below honours it).
        // Fresh each utterance.
        let mut force_confirm_low = false;

        // Confidence + echo gates (spoken input only) — now that abort has had its
        // chance. Confidence: drop what whisper wasn't sure of. Echo: drop the
        // copilot's own readback bleeding back through the speakers, so it can't
        // re-trigger itself. Both bypassed for typed input.
        if let Some(confidence) = spoken_confidence {
            if confidence < config.stt_min_confidence {
                // A garbled TRANSMISSION gets the tower's "say again" (a real
                // controller answers garble; the crew never heard it at all).
                if from_radio {
                    if let Some(st) = atc_station.as_mut() {
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
                            continue;
                        }
                    }
                    reporter.log(
                        ui::LogKind::Info,
                        "garbled transmission, nobody on frequency",
                    );
                    continue;
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
                        force_confirm_low = true;
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
                        continue;
                    }
                    // Not deliberate (continuous-mode ambient audio): ignore silently.
                    ConfidenceAction::Ignore => {
                        reporter.log(ui::LogKind::Info, "low confidence, ignored");
                        continue;
                    }
                    // Unreachable inside `confidence < min`, but total for clarity.
                    ConfidenceAction::Act => {}
                }
            }
            if voice.is_self_echo(&command) {
                reporter.log(ui::LogKind::Info, "ignored self-echo (own readback)");
                continue;
            }
        }

        // RADIO PTT: the utterance was transmitted, so it goes to the tuned
        // station or into the void — never to the copilot, and never through
        // the cockpit-command gates below. Physical routing beats content
        // routing whenever the transmit button is bound.
        if from_radio {
            // Copilot-only mode: the pilot is talking to REAL external ATC
            // (VATSIM, SayIntentions, stock). Their transmission is none of
            // our business — no simulated reply, no void briefing, nothing
            // to step on the real controller's answer.
            if !mode::atc_enabled() {
                reporter.log(ui::LogKind::Info, "transmitted; external ATC has the radio");
                continue;
            }
            if let Some(st) = atc_station.as_mut() {
                if let Some((reply, who)) = st
                    .handle_transmission(&command.to_ascii_lowercase(), xplane.view())
                    .await
                {
                    // Empty reply = a readback the controller accepted; a real
                    // frequency stays quiet, so we do too.
                    if reply.is_empty() {
                        reporter.log(ui::LogKind::Info, "readback acknowledged (frequency quiet)");
                        continue;
                    }
                    reporter.status(ui::Phase::Speaking);
                    reporter.log(ui::LogKind::Readback, format!("{}: {reply}", who.tag()));
                    voice.say_station(Priority::Readback, who, &reply);
                    reporter.status(ui::Phase::Listening);
                    continue;
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
                    continue;
                }
                let line = if last_void_brief
                    .is_none_or(|t| t.elapsed() > std::time::Duration::from_secs(300))
                {
                    last_void_brief = Some(Instant::now());
                    format!("No answer there. {}", void_briefing(&xplane, st).await)
                } else {
                    match xplane.get_float(datarefs::COM1_FREQ_KHZ).await {
                        Ok(v) => format!("No answer on {}.", atis::spoken_mhz(v.round() as i64)),
                        Err(_) => "No answer there.".to_string(),
                    }
                };
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            reporter.log(ui::LogKind::Info, "transmitted, nobody on this frequency");
            continue;
        }

        // Everything below is the COPILOT's business — intercom commands,
        // corrections, tools, questions. In ATC-only mode there is no crew,
        // but content routing must survive: with no radio key bound it is
        // the ONLY path to the station, and killing it would strand
        // keyboard-PTT and continuous setups off the air entirely.
        if !mode::copilot_enabled() {
            if let Some(st) = atc_station.as_mut() {
                let tl = command.to_lowercase();
                if try_route_content_atc(st, &tl, dual_ptt, &voice, &reporter, &xplane).await {
                    continue;
                }
            }
            // Not a radio call: one honest note the first time, silence
            // after (mirrors the "radios are dead" pattern — discoverable,
            // never naggy).
            if !atc_only_noted {
                atc_only_noted = true;
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Readback,
                    "Copilot is off. ATC only. Flip the mode in Settings to bring me back.",
                );
            } else {
                reporter.log(ui::LogKind::Info, "intercom ignored (ATC-only mode)");
            }
            continue;
        }

        // "I said eighty percent" — a spoken correction of the value the last
        // command set. Whisper heard "8" for "80" live, executed 8%, and the
        // pilot's fix dead-ended; a correction re-anchors on the last subject
        // immediately, before any gate can drop it.
        if !from_text {
            let recent = history
                .last()
                .is_some_and(|(_, _, t)| t.elapsed() < FOLLOWUP_WINDOW);
            if recent {
                if let (Some(rest), Some(subj)) = (correction_value(&command), &last_subject) {
                    let rewritten = format!("{subj} {rest}");
                    reporter.log(ui::LogKind::Info, format!("correction → \"{rewritten}\""));
                    command = rewritten;
                }
            }
        }

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
                if let Some(tool) = registry.get("hold_speed") {
                    let args = if stopish {
                        serde_json::json!({ "knots": 0 })
                    } else {
                        serde_json::json!({ "resume": true })
                    };
                    tracing::info!(stop = stopish, "governor stop/go word, pre-gate");
                    let ctx = ToolCtx {
                        xplane: &xplane,
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
                    continue;
                }
            }
        }
        if !from_text
            && !looks_like_command(&command)
            && !looks_like_question(&command)
            // An addressed interphone call is in-domain by construction
            // ("cabin crew, doors to manual" carries no cockpit keyword) —
            // the cabin handler below answers it deterministically.
            && cabin::Cabin::addressed(&command).is_none()
        {
            // A conversational follow-up shortly after a command is let through
            // even without a keyword. Outside the window, the strict gate holds.
            let in_followup = history
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
        if is_fuel_query(&command) && grammar::try_parse(&command, &registry, &profile).is_none() {
            // The landing-fuel projection needs the destination, which is only
            // populated by a plan fetch — after a restart it's empty until
            // something asks. Prime it (cached, cheap) so "will we have enough
            // fuel" projects instead of just reading the tanks.
            if !config.simbrief_id.trim().is_empty() {
                let _ = ensure_plan(&config, &mut fp_cache, false).await;
            }
            let line =
                fuel_report(&xplane, &profile, Some(&live), is_endurance_query(&command)).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            continue;
        }
        // Interphone: an utterance ADDRESSED to the cabin ("cabin crew,
        // prepare for landing") is theirs, answered deterministically —
        // cabin calls are rare and formulaic, so the LLM never sees them.
        // The address is the routing (no third PTT; see crate::cabin). On an
        // airframe with no cabin crew the copilot answers honestly instead
        // of letting the model improvise a flight attendant.
        if let Some(rest) = cabin::Cabin::addressed(&command) {
            if cabin::aboard(profile.cabin_crew) {
                let reply = cabin_state
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .handle(&rest, Instant::now());
                blackbox::record(
                    "cabin",
                    serde_json::json!({ "route": "call", "call": &command, "reply": &reply }),
                );
                reporter.status(ui::Phase::Speaking);
                reporter.log(ui::LogKind::Readback, format!("CAB: {reply}"));
                voice.say_cabin(Priority::Readback, &reply);
                reporter.status(ui::Phase::Listening);
            } else {
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Readback,
                    "No cabin crew on this airframe. It's just us up front.",
                );
            }
            continue;
        }
        // Ground navigation questions ("which way?", "say the taxi route")
        // answered deterministically off the published taxi route — the FO
        // reading the airport diagram, never the LLM. Position reads happen
        // BEFORE the route lock: no guard held across an await.
        if is_taxi_query(&command) && grammar::try_parse(&command, &registry, &profile).is_none() {
            let pos = match (
                xplane.get_float(datarefs::LATITUDE).await,
                xplane.get_float(datarefs::LONGITUDE).await,
            ) {
                (Ok(lat), Ok(lon)) => Some((lat, lon)),
                _ => None,
            };
            let line = party_line::with_taxi_route(|route| match (route, pos) {
                (Some(w), Some((lat, lon))) => taxi_guide::describe(w, lat, lon),
                (Some(w), None) if !w.via.is_empty() => {
                    format!("The cleared route is via {}.", w.via)
                }
                (Some(_), None) => {
                    "We have a taxi clearance, but I can't read our position right now.".to_string()
                }
                (None, _) => "We don't have a taxi clearance yet. Call the tower and request taxi."
                    .to_string(),
            });
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            continue;
        }
        // Live systems check + static reference limits — often phrased as commands,
        // not questions ("check all systems", "performance limits"), so they're
        // routed here, not behind the question gate.
        {
            let tl = command.to_ascii_lowercase();
            // FIRST, before anything else: while the copilot holds a control
            // authority (speed/taxi hold, mixture manager), any release-shaped
            // utterance lets go — even a garbled one. Whisper mangled "my
            // throttle" seven ways in a row live while the governor kept
            // taxiing; when we hold something, ambiguity resolves toward
            // giving it back. Naming a lever releases just that one; a generic
            // "my controls" / "let go" releases everything held.
            {
                use std::sync::atomic::Ordering::Relaxed;
                let speed_held = tools::speedhold::ACTIVE.load(Relaxed);
                // Holding the mixture is the continuous manager OR a lean
                // seek still hunting the peak — "my mixture" stops either.
                let mix_held = tools::engine::MIX_MANAGED.load(Relaxed)
                    || tools::engine::LEAN_SEEKING.load(Relaxed);
                let reverse_in = tools::reverse::ACTIVE.load(Relaxed);
                let taxi_driving = autotaxi::ACTIVE.load(Relaxed);
                let takeoff_flying = autotakeoff::ACTIVE.load(Relaxed);
                let circuit_flying = circuit::ACTIVE.load(Relaxed);
                let procedure_flying = procedure::ACTIVE.load(Relaxed);
                let full_auto_running = full_auto::ACTIVE.load(Relaxed);
                if (speed_held
                    || mix_held
                    || reverse_in
                    || taxi_driving
                    || takeoff_flying
                    || circuit_flying
                    || procedure_flying
                    || full_auto_running)
                    && release_intent(&tl)
                {
                    let names_mix = tl.contains("mixture") || tl.contains("mix");
                    let names_speed = tl.contains("throttle")
                        || tl.contains("speed")
                        || tl.contains("power")
                        || tl.contains("taxi")
                        || tl.contains("thr");
                    // The steering half of an auto taxi, releasable on its
                    // own: "my steering" keeps the speed hold — the
                    // pre-existing "you steer, I have the throttle" split.
                    let names_steer = tl.contains("steer")
                        || tl.contains("pedal")
                        || tl.contains("rudder")
                        || tl.contains("nosewheel");
                    let names_reverse = tl.contains("reverse");
                    // "Release parking brake" names a lever NOBODY here holds:
                    // it's for the brake tool, so it must not count as a
                    // generic release — that dropped the speed hold instead
                    // and left the throttle up with the brake still set
                    // (SR22, 2026-07-11).
                    let generic = !names_mix
                        && !names_speed
                        && !names_reverse
                        && !names_steer
                        && !tl.contains("brake");
                    let mut released: Vec<&str> = Vec::new();
                    // Full auto owns the entire chain (engine, taxi, line-up,
                    // circuit); cancelling it stands them all down and wins the
                    // release, so the per-loop branches below are skipped.
                    if full_auto_running && (names_speed || names_steer || generic) {
                        full_auto::abort();
                        released.push("full auto off, your aircraft");
                    }
                    // A circuit in progress owns whichever sub-loop is flying
                    // (takeoff or autoland); aborting it stands them ALL down,
                    // so it wins the generic release and the per-loop branches
                    // below are skipped (they'd double up on its sub-loops).
                    if !full_auto_running
                        && circuit_flying
                        && (names_speed || names_steer || generic)
                    {
                        circuit::abort();
                        released.push("circuit off, your aircraft");
                    }
                    // A published-procedure follower is airborne on the AP (it
                    // aborts its autoland handoff too); a lever-shaped or generic
                    // release hands it back.
                    if !full_auto_running
                        && !circuit_flying
                        && procedure_flying
                        && (names_speed || generic)
                    {
                        procedure::abort();
                        released.push("procedure off, your aircraft");
                    }
                    // A takeoff in progress: "my controls" is a HANDOVER, not
                    // a reject — the loop stands down silently, steering
                    // expires, and the power stays set for the pilot now
                    // flying it ("abort takeoff" is the arresting path).
                    if !circuit_flying
                        && !full_auto_running
                        && takeoff_flying
                        && (names_speed || names_steer || generic)
                    {
                        autotakeoff::ACTIVE.store(false, Relaxed);
                        released.push("your takeoff, power is set");
                    }
                    if !circuit_flying
                        && !full_auto_running
                        && taxi_driving
                        && (names_speed || names_steer || generic)
                    {
                        autotaxi::ACTIVE.store(false, Relaxed);
                        if names_steer && !names_speed && !generic {
                            released.push("steering's yours, I still have the speed");
                        } else {
                            tools::speedhold::ACTIVE.store(false, Relaxed);
                            released.push("auto taxi off, your aircraft");
                        }
                    } else if speed_held && (names_speed || generic) {
                        tools::speedhold::ACTIVE.store(false, Relaxed);
                        released.push("speed hold off, your throttle");
                    }
                    if mix_held && (names_mix || generic) {
                        tools::engine::MIX_MANAGED.store(false, Relaxed);
                        tools::engine::LEAN_SEEKING.store(false, Relaxed);
                        released.push("mixture's yours");
                    }
                    // Reverse rides the same levers as the speed hold — any
                    // lever-shaped release stows it (the watcher glides the
                    // levers back to idle on its way out).
                    if reverse_in && (names_reverse || names_speed || generic) {
                        tools::reverse::ACTIVE.store(false, Relaxed);
                        released.push("stowing reverse");
                    }
                    if !released.is_empty() {
                        let line = format!("Copy — {}.", released.join("; "));
                        say(&voice, &reporter, ui::LogKind::Readback, &line);
                        continue;
                    }
                }
            }
            // ATC station: a radio call answered by the tower when COM1 is tuned
            // to it. After the release guard above (handing controls back always
            // wins), before everything else — "request taxi" is a call to the
            // world, not a cockpit command. Falls through untouched when not
            // tuned or not a radio call. CONTENT routing only exists for setups
            // with no radio key: with a dedicated transmit button bound
            // (dual_ptt) this utterance came over the INTERCOM, and physical
            // routing beats content routing in both directions ("requesting
            // clearance" to the copilot was answered by the tower, 2026-07-12).
            // And either way the radio must be powered and the airframe not
            // wreckage — a dead panel can't transmit (fail open, like the
            // receive gate).
            if let Some(st) = atc_station.as_mut() {
                if try_route_content_atc(st, &tl, dual_ptt, &voice, &reporter, &xplane).await {
                    continue;
                }
                // Named-station tuning ("tune the tower", "contact approach") —
                // a COPILOT action: resolve the name against the station's
                // apt.dat frequencies, write COM1, confirm what's actually set.
                match st.tune_request(&tl) {
                    Some(station::Tune::Found { label, khz }) => {
                        let line = match xplane.set_int(datarefs::COM1_FREQ_KHZ, khz).await {
                            Ok(()) => {
                                let mhz = xplane
                                    .get_float(datarefs::COM1_FREQ_KHZ)
                                    .await
                                    .map(|v| v / 1000.0)
                                    .unwrap_or(khz as f64 / 1000.0);
                                format!("Roger, com one {mhz:.3} for {label}.")
                            }
                            Err(e) => {
                                tracing::warn!("COM1 write failed: {e:#}");
                                "Couldn't set the radio.".to_string()
                            }
                        };
                        say(&voice, &reporter, ui::LogKind::Readback, &line);
                        continue;
                    }
                    Some(station::Tune::Missing(msg)) => {
                        say(&voice, &reporter, ui::LogKind::Readback, &msg);
                        continue;
                    }
                    None => {}
                }
                // Frequency briefing ("what frequencies are here", "who can I
                // talk to") — crew knowledge answered from the loaded apt.dat
                // rows plus live COM1, after the tuning verbs so those win.
                if station::freq_query(&tl) {
                    let line = st.freq_briefing(xplane.view()).await;
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    continue;
                }
                // Radio coach ("what do I say?"): the copilot reads the
                // tower's own state machine and hands the pilot the exact
                // next call — deterministic, and always a call the parser
                // routes. A learning tool, not an LLM guess.
                if station::coach_query(&tl) {
                    let on_ground = xplane
                        .get_float(datarefs::ON_GROUND_ANY)
                        .await
                        .map(|v| v > 0.5)
                        .unwrap_or(true);
                    let callsign = xplane
                        .get_string(datarefs::TAILNUM)
                        .await
                        .ok()
                        .map(|t| tower::spoken_callsign(&t))
                        .filter(|s| !s.is_empty())
                        .unwrap_or_else(|| "your callsign".to_string());
                    let dist = match (position(&xplane).await, station::active_field()) {
                        (Some((lat, lon, _)), Some((_, flat, flon, _))) => {
                            Some(nav::haversine_nm(lat, lon, flat, flon))
                        }
                        _ => None,
                    };
                    let com1 = xplane
                        .get_float(datarefs::COM1_FREQ_KHZ)
                        .await
                        .ok()
                        .map(|v| v.round() as i64);
                    let line = match station::suggest_call(&callsign, on_ground, dist, com1) {
                        Some(s) => match (s.first, s.call) {
                            (Some(f), Some(c)) => {
                                format!("{f} Then key the radio and say: {c}")
                            }
                            (Some(f), None) => f,
                            (None, Some(c)) => format!("Key the radio and say: {c}"),
                            (None, None) => "No station on frequency to call.".to_string(),
                        },
                        None => "No station on frequency to call.".to_string(),
                    };
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    continue;
                }
            }
            // Reminder memory: cancel or list what's armed ("cancel the reminders",
            // "do we have a reminder set?"). Checked before set_reminder so "cancel"
            // wins over arming.
            if tl.contains("reminder") {
                if tl.contains("cancel")
                    || tl.contains("clear")
                    || tl.contains("forget")
                    || tl.contains("remove")
                    || tl.contains("delete")
                    || tl.contains("scrap")
                    || tl.contains("never mind")
                    || tl.contains("nevermind")
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
                let arming = tl.contains("wake")
                    || tl.contains("descent")
                    || tl.contains("feet")
                    || tl.contains("altitude")
                    || tl.contains("minute")
                    || tl.contains("waypoint")
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
                // "Anything to check?" / "anything I should check?" — a systems
                // check by intent; went completely unanswered live (no matcher,
                // and the continuous-mode gate drops non-matches silently).
                || (tl.contains("anything") && tl.contains("check"))
                // "aircraft health check" / "check the aircraft" — same intent;
                // two phrasings went silent live before a third one landed.
                || (tl.contains("check") && (tl.contains("aircraft") || tl.contains("health")))
                // "How's the aircraft health?" / "How's the aircraft doing?" —
                // no "check" at all; both dead-ended on the Twotter flight.
                || (tl.contains("aircraft") && (tl.contains("health") || tl.contains("doing")))
            {
                let line = systems_check(&xplane, &profile).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // Power-loss diagnosis ("why have we lost power?", "the engine quit") —
            // went unrecognized live. Phrased as a question, but what it wants is
            // the engine's fuel/ignition chain read and the culprit named.
            if is_power_loss_query(&tl) {
                let line = power_loss_diagnosis(&xplane, &profile).await;
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
            let is_reminder = tl.contains("wake")
                || tl.contains("remind")
                || tl.contains("notify")
                || tl.contains("let me know")
                || tl.contains("tell me when")
                || tl.contains("heads up");
            // Time/distance to top of descent ("how long before TOD"). Guarded by
            // is_reminder so "wake me at top of descent" still arms the reminder.
            // "top of descent" is reliably misheard ("when is top of descent" came in
            // as "1 is top descent"), so match top+descent loosely rather than the
            // exact phrase. This also keeps such an utterance out of the vertical-speed
            // grammar, which would otherwise read "descend 1 feet per minute".
            if !is_reminder
                && ((tl.contains("top") && tl.contains("descent"))
                    || tl.contains("t o d")
                    || tl
                        .split(|c: char| !c.is_ascii_alphanumeric())
                        .any(|w| w == "tod"))
            {
                let line = time_to_tod(&config, &xplane, &mut fp_cache).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // ETA to the destination ("time before we land", "when do we get in"):
            // distance to the destination airport over groundspeed. Needs a destination
            // cue plus a time cue, so it beats the next-waypoint branch below.
            if !is_reminder
                && (tl.contains("land")
                    || tl.contains("destination")
                    || tl.contains("arrive")
                    || tl.contains("arrival")
                    || tl.contains("get there")
                    || tl.contains("get in")
                    || tl.contains("en route")
                    || tl.contains("enroute")
                    || tl.contains("on route"))
                && (tl.contains("time")
                    || tl.contains("eta")
                    || tl.contains("when")
                    || tl.contains("how long")
                    || tl.contains("estimat")
                    || tl.contains("remaining"))
            {
                let line = eta_to_destination(&config, &xplane, &mut fp_cache).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // Destination field elevation — from the plan's destination record,
            // never the LLM (which answered "9999 feet": our own cruise altitude).
            if !is_reminder
                && (tl.contains("destination") || tl.contains("arrival"))
                && (tl.contains("altitude")
                    || tl.contains("elevation")
                    || tl.contains("height")
                    || tl.contains("how high"))
            {
                if !config.simbrief_id.trim().is_empty() {
                    let _ = ensure_plan(&config, &mut fp_cache, false).await;
                }
                let dest = monitor::DESTINATION
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone();
                let line = match dest {
                    Some(d) => format!(
                        "Field elevation at the destination is about {:.0} feet.",
                        (d.elev_ft / 50.0).round() * 50.0
                    ),
                    None => "No destination on file — load the flight plan first.".to_string(),
                };
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // Runway choice — answered from apt.dat + the wind, never the LLM
            // (which once invented "runway 18" at a field that only has 10/28).
            if !is_reminder
                && tl.contains("runway")
                && (tl.contains("which")
                    || tl.contains("what")
                    || tl.contains("should")
                    || tl.contains("use")
                    || tl.contains("best")
                    || tl.contains("active")
                    || tl.contains("recommend"))
            {
                reporter.status(ui::Phase::Thinking);
                let line = runway_advice(&config, &xplane, &mut fp_cache).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // "Are we good for landing?" — the landing-config check. Phrased as a
            // question so "set flaps for landing" (a command) never lands here.
            if !is_reminder
                && (tl.contains("good for landing")
                    || tl.contains("ready for landing")
                    || tl.contains("ready to land")
                    || tl.contains("good to land")
                    || (tl.contains("landing") && tl.contains("are we")))
            {
                let line = landing_readiness(&xplane, &profile).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // "Do we need any lights on?" / "are we configured?" — the SOP table
            // answers. Guarded so light *commands* ("nav lights on") never match.
            if !is_reminder
                && (tl.contains("light") || tl.contains("configur"))
                && (tl.contains("need")
                    || tl.contains("should")
                    || tl.contains("do we")
                    || tl.contains("are we")
                    || tl.contains("anything"))
            {
                let line = config_advice(&xplane, &profile).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // "Any traffic around?" — the TCAS scan, spoken as clock positions.
            if !is_reminder && tl.contains("traffic") {
                let line = traffic_report(&xplane).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // "Are we on course / on track?" — HSI deviation + waypoint, computed
            // (the LLM once answered this with the single word "heading").
            if !is_reminder
                && (tl.contains("on course")
                    || tl.contains("off course")
                    || tl.contains("on track"))
            {
                let line = on_course_report(&xplane, Some(&live)).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // Navigation progress (often a command, not a question): ETA / distance /
            // time to the next waypoint.
            if !is_reminder
                && (tl.contains("next waypoint")
                    || tl.contains("next fix")
                    || tl.contains("eta")
                    || tl.contains("how far")
                    || tl.contains("time to")
                    || tl.contains("distance to"))
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
                && (routeish
                    || tl.contains("plan")
                    || tl.contains("gps")
                    || tl.contains("fms")
                    || tl.contains("flight"));
            if wants_load {
                reporter.status(ui::Phase::Thinking);
                let refresh =
                    tl.contains("refresh") || tl.contains("reload") || tl.contains("update");
                let line = load_route_to_fms(&config, &xplane, &mut fp_cache, refresh).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // SimBrief flight plan (cached fetch) + advice. Route, full brief, or
            // refresh. Not when the intent is to *fly* it ("follow/navigate/track"
            // is the navigate tool; the word "fly" is the full-auto SimBrief
            // mission — both handled via the grammar/LLM path below).
            let nav_intent = tl.contains("navigate")
                || tl.contains("follow")
                || tl.contains("track")
                || tl
                    .split(|c: char| !c.is_ascii_alphanumeric())
                    .any(|w| w == "fly");
            let is_plan = !nav_intent
                && (tl.contains("flight plan")
                    || tl.contains("simbrief")
                    || tl.contains("fetch the plan")
                    || tl.contains("where are we going")
                    || tl.contains("where are we headed")
                    || (routeish
                        && (tl.contains("our") || tl.contains("the") || tl.contains("what"))));
            if is_plan {
                reporter.status(ui::Phase::Thinking);
                let refresh =
                    tl.contains("refresh") || tl.contains("reload") || tl.contains("update");
                let want_route = routeish;
                let line =
                    flight_plan_report(&config, &xplane, &mut fp_cache, refresh, want_route).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
        }
        // Strip conversational filler/politeness once ("could you tell me the
        // altitude" → "tell me the altitude", "okay flaps ten please" → "flaps
        // ten") and match against this, so polite phrasings don't miss. The raw
        // `command` is kept for logging/history.
        let matchable = grammar::normalize(&command);

        // Conversational glue (acks, "what can you do", "say again") — deterministic,
        // takes no action. Checked before commands/questions so pleasantries don't
        // fall through to "Say again?".
        if let Some(reply) = conversational(&matchable, &registry) {
            say(&voice, &reporter, ui::LogKind::Readback, &reply);
            continue;
        }

        // Cockpit geography — "where is the inverter?" answered from the profile's
        // verified `locations:` map: deterministic, instant, LLM-off. Runs before
        // the generic question path so a known location beats a state-query miss.
        if let Some(answer) = location_answer(&matchable, &profile) {
            say(&voice, &reporter, ui::LogKind::Readback, &answer);
            continue;
        }

        // Whisper habitually APPENDS "?" to short spoken commands ("Fuel tank,
        // right?"), which used to shunt them down the question path (seen live).
        // Punctuation alone doesn't outrank a parseable command: it's a question
        // only with an interrogative OPENER ("is/are/what/how...") or when the
        // grammar can't parse it anyway.
        // Checklists resolve deterministically by title ("before takeoff
        // checklist"). Their specs still ride to the LLM, but the answer-only
        // fallback can never call a tool - so without this, a grammar miss was
        // the end of the road and every checklist was voice-unreachable (seen
        // live in the G1000 172, 2026-07-14). Joined into `parsed` so whisper's
        // habitual trailing "?" doesn't shunt a match down the question path.
        let parsed = grammar::try_parse(&matchable, &registry, &profile).or_else(|| {
            checklist::match_request(&matchable, &checklists).map(|cl| {
                vec![llm::ToolCall {
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
            // A turbine (jet) reports N1/N2/ITT, not the piston RPM/CHT metrics.
            let turbine = !profile.engine && profile.throttle;
            if let Some(answer) =
                answer_state_query(&matchable, &xplane, Some(&live), turbine).await
            {
                say(&voice, &reporter, ui::LogKind::Readback, &answer);
                continue;
            }
        }

        let mut state: Option<String> = None;

        // Deterministic fast-path: resolve the common, regular commands with no
        // LLM at all — no GPU burst, no latency, and it can't pick the wrong tool.
        // Genuine questions are held back ("is our heading 270?" shares a command's
        // keyword+number shape — the OPENER decides); they go to the answer path.
        let fast = if is_question { None } else { parsed };
        // A recognized command whose CAPABILITY this airframe lacks (an autopilot
        // command on the AP-less DC-3) — decline honestly rather than fall through
        // to "Say again?" or a vague LLM answer.
        if fast.is_none() {
            if let Some(msg) = grammar::declined(&matchable, &registry) {
                tracing::info!(path = "declined", "command path");
                say(&voice, &reporter, ui::LogKind::Readback, &msg);
                continue;
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
                continue;
            }
        }
        // Whether the answer-only LLM fallback is live for THIS utterance. The
        // plugin's panel button (dataref `xcopilot/llm_enabled`) overrides the
        // startup default when set: 1 = on, 0 = off, and -1 / cache-miss (plugin
        // untouched or not loaded) falls back to the XCOPILOT_LLM startup value.
        let llm_enabled_now = match live.get(DR_LLM_ENABLED) {
            Some(v) if v >= 0.5 => true,
            Some(v) if v > -0.5 => false,
            _ => config.llm_enabled,
        };
        let calls = if let Some(c) = fast {
            tracing::info!(
                path = "fast",
                tool = c.first().map(|x| x.name.as_str()).unwrap_or(""),
                "command path"
            );
            c
        } else if llm_enabled_now {
            // Fast-path missed and the LLM is enabled: ANSWER-ONLY fallback. It is
            // called with no tools and cannot actuate — only the deterministic
            // grammar ever calls a tool. A genuine question gets a one-line spoken
            // answer; anything else gets "Say again?". Never an action on a guess.
            let ts = Instant::now();
            let snap = state_snapshot(&xplane, &profile, Some(&live)).await;
            tracing::info!(
                ms = ts.elapsed().as_millis(),
                state = snap.as_deref().unwrap_or("(none)"),
                "state snapshot"
            );
            // Ground the answer in the manuals, when indexed: POM excerpts first
            // (aircraft-specific beats general), then the shared FAA-handbook
            // index for what no POM covers. Two small embedding calls, worst case.
            let mut excerpt_parts: Vec<String> = Vec::new();
            if let Some(ix) = &manual_index {
                if let Some(e) = ix.retrieve(&config.ollama_base_url, &command, 4).await {
                    excerpt_parts.push(e);
                }
            }
            if let Some(ix) = &general_index {
                if let Some(e) = ix.retrieve(&config.ollama_base_url, &command, 3).await {
                    excerpt_parts.push(e);
                }
            }
            let excerpts = (!excerpt_parts.is_empty()).then(|| excerpt_parts.join("\n---\n"));
            let t = Instant::now();
            let answer = llm
                .answer(&command, &ctx_history, snap.as_deref(), excerpts.as_deref())
                .await;
            tracing::info!(
                ms = t.elapsed().as_millis(),
                path = "llm-answer",
                "command path"
            );
            match answer {
                Ok(Some(text)) => say(&voice, &reporter, ui::LogKind::Readback, &text),
                Ok(None) => {
                    // Heard clearly, but the answer-only LLM had no useful reply
                    // (not a question, or out of scope). Same as a grammar miss:
                    // negative tone on PTT, no misleading spoken "Say again?".
                    reporter.log(ui::LogKind::Warn, format!("no match: \"{command}\""));
                    if explicit && live.get(DR_OPT_TONE).is_none_or(|v| v > 0.5) {
                        voice.negative();
                    }
                }
                Err(e) => {
                    tracing::error!("LLM answer failed: {e:#}");
                    reporter.log(ui::LogKind::Warn, "LLM unreachable");
                }
            }
            continue;
        } else {
            // Fully deterministic (LLM disabled, the default): nothing recognized
            // it. The transcript was clear (it passed the confidence gate), so a
            // spoken "Say again?" would falsely imply a mishear and repeating won't
            // help. Log the miss to the panel and, when addressed directly (PTT),
            // acknowledge with the negative tone. Continuous mode stays silent.
            tracing::info!(path = "deterministic-miss", "command path");
            reporter.log(ui::LogKind::Warn, format!("unrecognized: \"{command}\""));
            if explicit && live.get(DR_OPT_TONE).is_none_or(|v| v > 0.5) {
                voice.negative();
            }
            continue;
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
            reporter.log(
                ui::LogKind::Tool,
                format!("{} {}", call.name, call.arguments),
            );

            // A checklist isn't a one-shot tool — the copilot runs it: reads each
            // item, does the ones it can, calls out the pilot's. Needs the voice,
            // so it's handled here, not via the plain ToolCtx. A checklist is a
            // whole phase, run only on demand — so it's never compounded: run this
            // one and ignore any other calls the LLM returned in the same turn
            // (guards against chaining through several checklists at once).
            if let Some(cl) = checklists.iter().find(|c| c.id == call.name) {
                run_checklist(
                    cl,
                    &xplane,
                    &config,
                    &mut listener,
                    &ptt_mic,
                    &stt,
                    &ptt,
                    typed.as_ref(),
                    &voice,
                    &reporter,
                    &interrupt,
                )
                .await;
                break;
            }

            // Confirmation gate with in-line correction ("negative, 500"). Applies
            // per call, so each safety-critical action in a compound command is
            // confirmed on its own.
            let proceed = 'confirm: loop {
                let Some(tool) = registry.get(&call.name) else {
                    tracing::warn!("LLM picked unknown tool: {}", call.name);
                    say(
                        &voice,
                        &reporter,
                        ui::LogKind::Warn,
                        "That command isn't wired up yet.",
                    );
                    break 'confirm false;
                };
                let intent = if config.confirm_critical {
                    tool.confirm_prompt(&call.arguments)
                } else {
                    None
                };
                // A command heard at marginal confidence is read back for a yes/no
                // even when the tool isn't safety-critical — this reuses the gate to
                // recover a shaky capture ("Confirm, I heard gear up?") rather than
                // making the pilot repeat the whole thing.
                let intent =
                    intent.or_else(|| force_confirm_low.then(|| format!("I heard {command}")));
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
                let Some((reply, _)) = listen_once(&mut listener, &ptt_mic, &stt, &ptt, None)
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
                        // A fast-path command never took the snapshot — take it now
                        // so the correction is grounded in live state.
                        if state.is_none() {
                            state = state_snapshot(&xplane, &profile, Some(&live)).await;
                        }
                        match llm
                            .choose_tool(
                                &format!("{command}, {correction}"),
                                &specs,
                                &ctx_history,
                                state.as_deref(),
                            )
                            .await
                        {
                            Ok(mut cs) if !cs.is_empty() => {
                                call = cs.remove(0);
                                // The correction WAS the pilot's clear intent, so the
                                // re-picked tool no longer needs the marginal read-back
                                // (a safety-critical one still self-confirms).
                                force_confirm_low = false;
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
            if !proceed {
                continue;
            }

            // Execute the curated tool (writes the dataref + reads back).
            reporter.status(ui::Phase::Thinking);
            let Some(tool) = registry.get(&call.name) else {
                continue;
            };
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
            let restore_view: Option<[f64; 6]> = if let Some(v) =
                profile.view_map.get(&call.name).filter(|_| snap_enabled)
            {
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
                callout_off = Some((angle_delta_deg(applied[3], saved[3]), saved[4] - applied[4]));
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
                xplane: &xplane,
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
    say(
        voice,
        reporter,
        ui::LogKind::Info,
        &format!("Running the {} checklist.", cl.title),
    );
    let ctx = ToolCtx {
        xplane,
        config,
        voice: Some(voice),
        status: reporter.status_sink(),
        reminders: None,
    };
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
            Item::Act {
                say: line,
                tool,
                args,
                confirm,
            } => {
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
                    say(
                        voice,
                        reporter,
                        ui::LogKind::Confirm,
                        &format!("Confirm, {hazard}?"),
                    );
                    reporter.status(ui::Phase::Listening);
                    let reply = listen_once(listener, ptt_mic, stt, ptt, None).map(|(r, _)| r);
                    match reply {
                        Some(r) if is_affirmative(&r.text) => {} // proceed
                        Some(r) if is_abort(&r.text) => {
                            aborted = true;
                            break;
                        }
                        _ => {
                            say(
                                voice,
                                reporter,
                                ui::LogKind::Info,
                                &format!("{line} Skipped — your call."),
                            );
                            continue;
                        }
                    }
                }
                if checklist::run_step(
                    tool,
                    args,
                    &cl.lights,
                    &cl.flaps,
                    &cl.electrical,
                    &cl.fuel_selector,
                    cl.engines,
                    cl.magnetos_per_engine,
                    cl.park_brake_valve_set,
                    cl.pitot_heat_dataref.clone(),
                    &ctx,
                )
                .await
                .is_ok()
                {
                    say(voice, reporter, ui::LogKind::Readback, line);
                } else {
                    say(
                        voice,
                        reporter,
                        ui::LogKind::Warn,
                        &format!("{line} Didn't take — check it."),
                    );
                    failed += 1;
                }
            }
            // A dataref we don't actuate — read it and confirm or flag.
            Item::Verify {
                say: line,
                dataref,
                index,
                cond,
                settable,
            } => {
                match checklist::read_dataref(xplane, dataref, *index).await {
                    Some(v) if cond.eval(v) => say(
                        voice,
                        reporter,
                        ui::LogKind::Readback,
                        &format!("{line} Confirmed."),
                    ),
                    // Not in the desired state.
                    Some(_) if *settable => {
                        // An actionable switch — set it to the checklist's own
                        // target value (it's the authored procedure, not an
                        // arbitrary write), then confirm. If the write doesn't take,
                        // flag it.
                        let target = cond.target();
                        let wrote = match index {
                            Some(i) => xplane
                                .set_float_array_elem(dataref, *i, target)
                                .await
                                .is_ok(),
                            None => xplane.set_float(dataref, target).await.is_ok(),
                        };
                        let now = checklist::read_dataref(xplane, dataref, *index).await;
                        if wrote && now.is_some_and(|v| cond.eval(v)) {
                            say(
                                voice,
                                reporter,
                                ui::LogKind::Readback,
                                &format!("{line} Set."),
                            );
                        } else {
                            say(
                                voice,
                                reporter,
                                ui::LogKind::Warn,
                                &format!("{line} Couldn't set it — check it."),
                            );
                            failed += 1;
                        }
                    }
                    // Observe-only item (CHECK/test step or read-only indicator) not
                    // yet in state — call it out for the pilot, don't write or flag.
                    Some(_) => say(voice, reporter, ui::LogKind::Confirm, line),
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
    [
        "abort",
        "stop",
        "cancel",
        "belay",
        "disregard",
        "halt",
        "quit",
    ]
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

/// Transcribe an audio file with whisper, for `xvl transcribe <file>`.
/// Decodes/resamples to 16 kHz mono via macOS `afconvert` (handles mp3/m4a/wav/…
/// — whisper needs 16 kHz mono), reads the PCM, and returns the text. Reuses the
/// same STT model the live loop uses, so no extra model download.
/// Decode any audio file to 16 kHz mono f32 samples via `afconvert` (macOS).
fn load_wav_16k(input: &str) -> Result<Vec<f32>> {
    use std::process::Command;
    if !std::path::Path::new(input).exists() {
        return Err(anyhow!("no such file: {input}"));
    }
    let wav = std::env::temp_dir().join("xvoicelayer-16k.wav");
    let status = Command::new("afconvert")
        .args([
            input,
            wav.to_str().unwrap(),
            "-d",
            "LEI16@16000",
            "-c",
            "1",
            "-f",
            "WAVE",
        ])
        .status()
        .map_err(|e| anyhow!("spawning afconvert: {e}"))?;
    if !status.success() {
        return Err(anyhow!("afconvert failed to decode {input}"));
    }
    let reader =
        hound::WavReader::open(&wav).map_err(|e| anyhow!("reading {}: {e}", wav.display()))?;
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
        let is_log = p
            .file_name()
            .and_then(|n| n.to_str())
            .is_some_and(|n| n.starts_with("xvoicelayer.log"));
        if !is_log {
            continue;
        }
        let Ok(content) = std::fs::read_to_string(&p) else {
            continue;
        };
        for line in content.lines() {
            if let Some(i) = line.find("transcript=") {
                let rest = &line[i + "transcript=".len()..];
                last = Some(
                    rest.split(" confidence=")
                        .next()
                        .unwrap_or(rest)
                        .trim()
                        .to_lowercase(),
                );
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
        "C750" | "C75B" => "citation_x",
        "SF50" => "cirrus_vision_sf50",
        "DC3" | "DC30" => "dc3",
        "BE9L" | "BE9T" | "C90" | "C90B" => "king_air_c90b",
        "EVOT" | "EVOP" => "lancair_evolution",
        "DHC6" => "dhc6_twin_otter",
        _ => "cessna_172sp",
    }
    .to_string()
}

/// Capture one utterance (continuous VAD or push-to-talk) and transcribe it.
/// Returns `None` on silence/noise or a capture error. The bool is true when
/// the RADIO PTT was held during capture — the utterance was transmitted (for
/// the tuned ATC station), not spoken to the copilot.
fn listen_once(
    listener: &mut Option<Listener>,
    ptt_mic: &Option<PttMic>,
    stt: &Stt,
    ptt: &Option<Ptt>,
    idle_timeout: Option<Duration>,
) -> Option<(stt::Transcription, bool)> {
    let mut from_radio = false;
    let audio = match (ptt, ptt_mic, listener) {
        // Hold-a-key push-to-talk (works while X-Plane has focus), warm mic.
        (Some(Ptt::Key(key)), Some(mic), _) => {
            use device_query::DeviceQuery;
            let dev = device_query::DeviceState::new();
            let key = *key;
            mic.capture(|| dev.get_keys().contains(&key), idle_timeout)
                .ok()?
        }
        // Sim-dataref push-to-talk (the X-Plane plugin's bound buttons), warm
        // mic. Either button captures; the radio one tags the utterance as a
        // transmission. Sampled through the whole hold, so a radio press can't
        // be missed between polls.
        (Some(Ptt::Dataref { live, name, radio }), Some(mic), _) => {
            let radio_seen = std::sync::atomic::AtomicBool::new(false);
            let audio = mic
                .capture(
                    || {
                        let intercom = Ptt::dataref_active(live, name);
                        let tx = !radio.is_empty() && Ptt::dataref_active(live, radio);
                        if tx {
                            radio_seen.store(true, std::sync::atomic::Ordering::Relaxed);
                        }
                        intercom || tx
                    },
                    idle_timeout,
                )
                .ok()?;
            from_radio = radio_seen.load(std::sync::atomic::Ordering::Relaxed);
            audio
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
    // The transmit key's state is known BEFORE decoding: only a keyed
    // transmission carries the (GPU-expensive) tower prompt.
    match stt.transcribe_gated(&audio, from_radio) {
        Ok(t) if !t.text.is_empty() => Some((t, from_radio)),
        _ => None,
    }
}

/// One pilot input — spoken (a transcript, with whether the RADIO PTT carried
/// it) or typed in the TUI.
enum Input {
    Spoken(stt::Transcription, bool),
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
        return listen_once(listener, ptt_mic, stt, ptt, None).map(|(t, tx)| Input::Spoken(t, tx));
    };
    // Surface an idle tick every couple of seconds instead of blocking
    // forever: the main loop has station work to consume even when nobody
    // speaks (a region handoff armed by the monitor sat unconsumed through a
    // silent Heathrow session, 2026-07-10 — no swap, no party line).
    let idle_deadline = std::time::Instant::now() + Duration::from_secs(2);
    loop {
        if let Ok(t) = typed.try_recv() {
            let t = t.trim().to_string();
            if !t.is_empty() {
                return Some(Input::Typed(t));
            }
        }
        if let Some((t, tx)) = listen_once(
            listener,
            ptt_mic,
            stt,
            ptt,
            Some(Duration::from_millis(150)),
        ) {
            return Some(Input::Spoken(t, tx));
        }
        // Timed out idle (or a discarded blip) — re-check the typed channel,
        // or hand the loop an idle tick.
        if std::time::Instant::now() >= idle_deadline {
            return None;
        }
    }
}

/// Negation words — a reply containing any of these is never an affirmative.
const NEGATIONS: &[&str] = &[
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
enum ConfidenceAction {
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
fn classify_confidence(
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
fn is_affirmative(reply: &str) -> bool {
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
/// The last genuine readback/answer spoken — so "say again" can repeat it. Only
/// real readbacks are stored (not warnings like "Say again?"), single-threaded
/// session so a plain Mutex is fine.
static LAST_READBACK: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

fn say(voice: &Voice, reporter: &Reporter, kind: ui::LogKind, line: &str) {
    tracing::info!(spoke = %line, "spoke");
    reporter.status(ui::Phase::Speaking);
    reporter.log(kind, line);
    voice.say(Priority::Readback, line);
    reporter.status(ui::Phase::Listening);
    if matches!(kind, ui::LogKind::Readback) {
        if let Ok(mut last) = LAST_READBACK.lock() {
            *last = Some(line.to_string());
        }
    }
}

/// Deterministic conversational glue — the cockpit chatter that isn't a command or
/// a state question: "what can you do", "say again", and acknowledgments. Returns
/// the spoken reply, or `None` to fall through to the normal command/question path.
/// `t` is already normalized (filler stripped). Capabilities are read live from the
/// registry, so the answer matches the actual aircraft.
fn conversational(t: &str, registry: &Registry) -> Option<String> {
    let has = |w: &str| t.split(|c: char| !c.is_alphanumeric()).any(|tok| tok == w);
    // "say again" / "repeat that" / "what did you say" → repeat the last readback.
    if t.contains("say again")
        || t.contains("repeat")
        || t.contains("come again")
        || t.contains("what did you say")
        || t.contains("didnt catch that")
    {
        return Some(match LAST_READBACK.lock().ok().and_then(|l| l.clone()) {
            Some(prev) => prev,
            None => "Nothing to repeat yet.".to_string(),
        });
    }
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

/// The datarefs the live-state cache subscribes to over the WebSocket — the
/// continuously-read snapshot + autopilot-status set. (Arrays like engine-running,
/// and the tool path's read-after-write confirms, stay on REST.)
const SNAPSHOT_DATAREFS: &[&str] = &[
    datarefs::ALTITUDE_FT,
    datarefs::AIRSPEED_KTS,
    datarefs::VVI_FPM,
    datarefs::COMPASS_HEADING_DEG,
    datarefs::FLAP_RATIO,
    datarefs::GEAR_HANDLE_DOWN,
    datarefs::ON_GROUND_ANY,
    datarefs::FUEL_TOTAL_KG,
    datarefs::WIND_DIR_DEG_TRUE,
    datarefs::WIND_SPEED_MSC,
    datarefs::OAT_DEG_C,
    datarefs::VISIBILITY_SM,
    datarefs::QNH_PASCALS,
    datarefs::SERVOS_ON,
    datarefs::APPROACH_STATUS,
    datarefs::NAV_STATUS,
    datarefs::HEADING_STATUS,
    datarefs::COM1_FREQ_KHZ,
    DR_LLM_ENABLED,
    DR_OPT_SNAP,
    DR_OPT_TONE,
];

/// The 3D-cockpit camera position (pilot's-head datarefs) — read to remember the
/// current view before a snap-to-panel, and written to restore it after. Order is
/// x, y, z (metres from CG), then psi, the, phi (view angles, degrees).
const HEAD_DREFS: [&str; 6] = [
    "sim/graphics/view/pilots_head_x",
    "sim/graphics/view/pilots_head_y",
    "sim/graphics/view/pilots_head_z",
    "sim/graphics/view/pilots_head_psi",
    "sim/graphics/view/pilots_head_the",
    "sim/graphics/view/pilots_head_phi",
];

/// Deterministic "where is X" answers from the profile's `locations:` map — the
/// spatial knowledge PDF manuals can't carry. Self-gated: only a where/location
/// question, and only for an item the profile actually knows (longest key match
/// wins, so "fuel selector" beats "fuel"). `None` falls through to normal handling.
fn location_answer(command: &str, profile: &aircraft::AircraftProfile) -> Option<String> {
    if profile.locations.is_empty() {
        return None;
    }
    let t = grammar::normalize(command);
    if !(t.contains("where") || t.contains("located") || t.contains("location")) {
        return None;
    }
    let mut best: Option<(&String, &String)> = None;
    for (key, desc) in &profile.locations {
        if t.contains(key.as_str()) && best.is_none_or(|(bk, _)| key.len() > bk.len()) {
            best = Some((key, desc));
        }
    }
    best.map(|(key, desc)| format!("The {key} is {desc}."))
}

/// Interpolate between two headings/angles along the shortest arc, degrees.
fn lerp_angle_deg(a: f64, b: f64, t: f64) -> f64 {
    let d = ((b - a) % 360.0 + 540.0) % 360.0 - 180.0;
    (a + d * t).rem_euclid(360.0)
}

/// Signed shortest difference b − a in degrees (−180..180).
fn angle_delta_deg(a: f64, b: f64) -> f64 {
    ((b - a) % 360.0 + 540.0) % 360.0 - 180.0
}

/// Human label for the on-screen callout: "FLAPS → 10", "GEAR → DOWN", "PROPS →
/// CRUISE". Generic: the tool name minus set_, plus the argument values.
fn callout_label(tool: &str, args: &serde_json::Value) -> String {
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

/// The plugin's runtime LLM-fallback toggle (see plugin/src/xcopilot.cpp). Polled
/// live through the snapshot cache so the panel button takes effect on the next
/// utterance. `-1` = untouched (follow the `XCOPILOT_LLM` startup default), `0` =
/// forced off, `1` = forced on. Absent (plugin not loaded) reads as a cache miss,
/// which also falls back to the startup default.
const DR_LLM_ENABLED: &str = "xcopilot/llm_enabled";
/// Panel settings-popup switches, polled the same way. Missing (no plugin) =
/// the feature's default (both on).
const DR_OPT_SNAP: &str = "xcopilot/opt/camera_snap";
const DR_OPT_TONE: &str = "xcopilot/opt/negative_tone";

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
/// Pounds per kilogram — jet fuel flow/quantity is read in lb (airliner convention).
const LB_PER_KG: f64 = 2.20462;

/// Meters per second → knots.
const MS_TO_KTS: f64 = 1.943_844;

/// The active GPS waypoint with distance and time/ETA to it — answers "ETA",
/// "distance/time to next waypoint", "how far". Often phrased as a command, not a
/// question, so it's dispatched unconditionally rather than via answer_state_query.
/// "Are we good for landing?" — the landing-configuration check, concluded:
/// gear (where it retracts), flap state, speed against the approach reference,
/// and the landing-phase SOP rows (landing light). Went to "No reading for
/// that" live; this is the one question a copilot must answer well.
async fn landing_readiness(xp: &XPlaneClient, profile: &aircraft::AircraftProfile) -> String {
    let airborne = xp
        .get_float("sim/flightmodel/failures/onground_any")
        .await
        .map(|v| v < 0.5)
        .unwrap_or(true);
    if !airborne {
        return "We're already on the ground.".to_string();
    }
    let mut issues: Vec<String> = Vec::new();
    let mut goods: Vec<String> = Vec::new();
    if profile.retractable_gear {
        let down = xp
            .get_float("sim/cockpit/switches/gear_handle_status")
            .await
            .map(|v| v > 0.5)
            .unwrap_or(false);
        if down {
            goods.push("gear's down".to_string());
        } else {
            issues.push("gear's still up".to_string());
        }
    }
    if let Some(flaps) = &profile.flaps {
        let ratio = xp
            .get_float("sim/cockpit2/controls/flap_handle_deploy_ratio")
            .await
            .unwrap_or(0.0);
        let nearest = flaps
            .detents
            .iter()
            .min_by(|a, b| (a.ratio - ratio).abs().total_cmp(&(b.ratio - ratio).abs()));
        if let Some(d) = nearest {
            if d.ratio < 0.01 {
                issues.push("no flap yet".to_string());
            } else {
                goods.push(format!("flaps {}", d.spoken));
            }
        }
    }
    let ias = xp
        .get_float("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")
        .await
        .unwrap_or(0.0);
    if let Some(app) = profile.reference.as_ref().and_then(|r| r.approach_kts) {
        if ias > app + 15.0 {
            issues.push(format!(
                "{ias:.0} knots is fast against the {app:.0} reference"
            ));
        } else {
            goods.push(format!("{ias:.0} knots against a {app:.0} reference"));
        }
    } else {
        goods.push(format!("{ias:.0} knots",));
    }
    // Landing-phase SOP rows (landing light and friends).
    for item in &profile.advisor {
        if !item.phases.iter().any(|p| p == "landing") {
            continue;
        }
        let Ok(v) = xp.get_float(&item.dataref).await else {
            continue;
        };
        if (v > 0.5) != item.expect_on {
            issues.push(item.say.trim_end_matches('.').to_lowercase());
        }
    }
    if issues.is_empty() {
        format!("You're set: {}. Fly it in.", goods.join(", "))
    } else {
        let good_tail = if goods.is_empty() {
            String::new()
        } else {
            format!(" Rest is good: {}.", goods.join(", "))
        };
        format!("Not yet — {}.{good_tail}", issues.join("; "))
    }
}

/// "Do we need any lights on?" / "are we configured right?" — the profile's SOP
/// advisor table crossed with the live switches, spoken as a conclusion. Went to
/// "No reading for that" live: the table knew the answer, nothing asked it.
async fn config_advice(xp: &XPlaneClient, profile: &aircraft::AircraftProfile) -> String {
    if profile.advisor.is_empty() {
        return "No configuration table for this airframe yet.".to_string();
    }
    let airborne = xp
        .get_float("sim/flightmodel/failures/onground_any")
        .await
        .map(|v| v < 0.5)
        .unwrap_or(false);
    let phase = if airborne { "cruise" } else { "ground" };
    let mut wrong: Vec<String> = Vec::new();
    let mut arrival_items = false;
    for item in &profile.advisor {
        let Ok(v) = xp.get_float(&item.dataref).await else {
            continue;
        };
        let satisfied = (v > 0.5) == item.expect_on;
        if item.phases.iter().any(|p| p == phase) {
            if !satisfied {
                wrong.push(item.say.trim_end_matches('.').to_string());
            }
        } else if airborne
            && !satisfied
            && item
                .phases
                .iter()
                .any(|p| p == "approach" || p == "landing")
        {
            arrival_items = true;
        }
    }
    if wrong.is_empty() {
        let tail = if arrival_items {
            " One or two come back on for the arrival — I'll speak up."
        } else {
            ""
        };
        format!("You're set for {phase}; nothing needed right now.{tail}")
    } else {
        format!("Couple of things: {}.", wrong.join("; "))
    }
}

/// "Any traffic around?" — the nearest TCAS targets as clock-position callouts
/// ("11 o'clock, 3 miles, 500 feet above"), from the sim's TCAS arrays. Went
/// unanswered live ("No reading for that"); a copilot's traffic scan is exactly
/// the kind of question that must never be guessed.
async fn traffic_report(xp: &XPlaneClient) -> String {
    let (bear, dist, alt) = (
        xp.get_float_array("sim/cockpit2/tcas/indicators/relative_bearing_degs")
            .await,
        xp.get_float_array("sim/cockpit2/tcas/indicators/relative_distance_mtrs")
            .await,
        xp.get_float_array("sim/cockpit2/tcas/indicators/relative_altitude_mtrs")
            .await,
    );
    let (Ok(bear), Ok(dist), Ok(alt)) = (bear, dist, alt) else {
        return "No traffic picture available.".to_string();
    };
    // Slot 0 is own ship; live targets have a real distance.
    let all: Vec<(f64, f64, f64)> = dist
        .iter()
        .zip(&bear)
        .zip(&alt)
        .skip(1)
        .filter(|((d, _), _)| **d > 1.0)
        .map(|((d, b), a)| (*d, *b, *a))
        .collect();
    if all.is_empty() {
        return "No traffic on the scope.".to_string();
    }
    // A copilot's scan reports *factors* — inside ~15 nm and ±3,000 ft — not the
    // whole online-traffic feed (an airliner 24,000 ft overhead is scenery, and
    // the first live scan recited three of them plus "60 more further out").
    let mut targets: Vec<(f64, f64, f64)> = all
        .iter()
        .copied()
        .filter(|(d, _, a)| *d < 15.0 * 1852.0 && a.abs() < 3000.0 / 3.28084)
        .collect();
    if targets.is_empty() {
        let nearest = all
            .iter()
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .expect("non-empty");
        return format!(
            "No factor traffic. Nearest is {:.0} miles out, {:.0} feet {}.",
            nearest.0 / 1852.0,
            ((nearest.2 * 3.28084).abs() / 100.0).round() * 100.0,
            if nearest.2 > 0.0 { "above" } else { "below" }
        );
    }
    targets.sort_by(|a, b| a.0.total_cmp(&b.0));
    let calls: Vec<String> = targets
        .iter()
        .take(3)
        .map(|(d, b, a)| {
            let nm = d / 1852.0;
            let range = if nm < 1.0 {
                "under a mile".to_string()
            } else {
                format!("{nm:.0} miles")
            };
            let clock = match (((b + 15.0).rem_euclid(360.0)) / 30.0) as i32 {
                0 => 12,
                c => c,
            };
            let ft = ((a * 3.28084) / 100.0).round() * 100.0;
            let vert = if ft.abs() < 300.0 {
                "same altitude".to_string()
            } else if ft > 0.0 {
                format!("{ft:.0} feet above")
            } else {
                format!("{:.0} feet below", ft.abs())
            };
            format!("{clock} oclock, {range}, {vert}")
        })
        .collect();
    let more = targets.len().saturating_sub(3);
    let tail = if more > 0 {
        format!(", and {more} more further out")
    } else {
        String::new()
    };
    let noun = if targets.len() == 1 {
        "One target"
    } else {
        "Traffic"
    };
    format!("{noun}: {}{tail}.", calls.join("; "))
}

/// "Are we on course?" — answered from the HSI course deviation plus the active
/// waypoint, never the LLM (which once replied with the bare word "heading").
/// Positive hdef = the course is to our right, i.e. we're left of it.
async fn on_course_report(xp: &XPlaneClient, live: Option<&LiveState>) -> String {
    // Parked, the question has no meaning — and the naive answer was nonsense
    // ("On course. 0 miles, 0 minutes." spoken live on the ramp).
    let on_ground = xp
        .get_float("sim/flightmodel/failures/onground_any")
        .await
        .unwrap_or(0.0)
        > 0.5;
    let gs = xp
        .get_float("sim/flightmodel/position/groundspeed")
        .await
        .unwrap_or(0.0);
    if on_ground && gs < 15.0 {
        return "We're on the ground — course comes alive once we're flying the route.".to_string();
    }
    let progress = nav_progress(xp, live).await;
    if progress.starts_with("No active GPS waypoint") {
        return progress;
    }
    let dots = read1(
        live,
        xp,
        "sim/cockpit2/radios/indicators/hsi_hdef_dots_pilot",
    )
    .await
    .ok();
    let verdict = match dots {
        Some(d) if d.abs() <= 0.5 => "On course.".to_string(),
        Some(d) => format!(
            "{:.1} dots {} of course, correct {}.",
            d.abs(),
            if d > 0.0 { "left" } else { "right" },
            if d > 0.0 { "right" } else { "left" },
        ),
        None => String::new(),
    };
    format!("{verdict} {progress}").trim().to_string()
}

async fn nav_progress(xp: &XPlaneClient, live: Option<&LiveState>) -> String {
    let wp = xp
        .get_string("sim/cockpit2/radios/indicators/gps_nav_id")
        .await
        .unwrap_or_default();
    let dist = read1(
        live,
        xp,
        "sim/cockpit2/radios/indicators/gps_dme_distance_nm",
    )
    .await
    .ok();
    let tmin = read1(live, xp, "sim/cockpit2/radios/indicators/gps_dme_time_min")
        .await
        .ok();
    if wp.trim().is_empty() && dist.is_none() {
        return "No active GPS waypoint. Load and navigate a route first.".to_string();
    }
    let mut s = String::new();
    if !wp.trim().is_empty() {
        s.push_str(&format!("Next waypoint {}, ", wp.trim()));
    }
    if let Some(d) = dist {
        s.push_str(&format!("{d:.0} miles",));
    }
    if let Some(tm) = tmin {
        let (h, mn) = ((tm / 60.0) as i32, (tm % 60.0).round() as i32);
        let time = if h > 0 {
            format!("{h} hours {mn} minutes")
        } else {
            format!("{mn} minutes")
        };
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
    let to_tod = nav::haversine_nm(lat, lon, dest.lat, dest.lon) - ring_nm;
    tod_phrase(to_tod, xp).await
}

/// Format a distance-to-TOD as a spoken line, paced by groundspeed.
async fn tod_phrase(dist_nm: f64, xp: &XPlaneClient) -> String {
    if dist_nm <= 0.0 {
        return "We're at top of descent, time to start down.".to_string();
    }
    let gs_kts = xp
        .get_float("sim/flightmodel/position/groundspeed")
        .await
        .unwrap_or(0.0)
        * MS_TO_KTS;
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
    let dest_name = if fp.destination.is_empty() {
        dest.ident.clone()
    } else {
        fp.destination.clone()
    };
    let (lat, lon, gs_ms) = match (
        xp.get_float("sim/flightmodel/position/latitude").await,
        xp.get_float("sim/flightmodel/position/longitude").await,
        xp.get_float("sim/flightmodel/position/groundspeed").await,
    ) {
        (Ok(a), Ok(b), Ok(c)) => (a, b, c),
        _ => return "I can't read our position right now.".to_string(),
    };
    let dist = nav::haversine_nm(lat, lon, dest.lat, dest.lon);
    let gs_kts = gs_ms * MS_TO_KTS;
    if gs_kts < 30.0 {
        return format!(
            "{dest_name} is {dist:.0} miles out. I'll have a time once we're up and moving."
        );
    }
    let mins = (dist / gs_kts * 60.0).round() as i64;
    let (h, m) = (mins / 60, mins % 60);
    let eta = if h > 0 {
        format!("{h} hours {m} minutes")
    } else {
        format!("{m} minutes")
    };
    format!("About {eta} to {dest_name}, {dist:.0} miles out at {gs_kts:.0} knots.")
}

/// Release-shaped speech: does this utterance plausibly mean "give me back the
/// controls"? Deliberately generous — it is ONLY consulted while the copilot
/// actually holds a control authority (speed hold etc.), where the cost of a
/// false release is a lever going still and the cost of a missed release is a
/// pilot fighting their own copilot. Seen live: "my throttle" came through as
/// "my thruffles", "mic control", "my throat", "my problem" — seven phrasings
/// in a row missed while the governor kept taxiing.
fn release_intent(t: &str) -> bool {
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

/// Which runway to use: the field's *actual* runways (apt.dat) crossed with the
/// live wind — deterministic, so it can only name a runway that exists. On the
/// ground it's the departure field, airborne it's the destination (using local
/// wind as the approximation). Replaces the LLM answering this from wind alone,
/// which once produced "runway 18" at a field that only has 10 and 28.
async fn runway_advice(
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
    let on_ground = xp
        .get_float("sim/flightmodel/failures/onground_any")
        .await
        .unwrap_or(1.0)
        > 0.5;
    let icao = if on_ground {
        fp.origin.clone()
    } else {
        fp.destination.clone()
    };
    // apt.dat is a ~400 MB streaming scan — off the async thread it goes.
    let (path, icao2) = (config.apt_data_path.clone(), icao.clone());
    let runways = tokio::task::spawn_blocking(move || nav::find_runways(&path, &icao2))
        .await
        .ok()
        .and_then(|r| r.ok())
        .unwrap_or_default();
    if runways.is_empty() {
        return format!("I don't have runway data for {icao}.");
    }
    let all: Vec<&str> = runways
        .iter()
        .flat_map(|r| [r.ident_a.as_str(), r.ident_b.as_str()])
        .collect();
    let wind_dir = xp
        .get_float(datarefs::WIND_DIR_DEG_TRUE)
        .await
        .unwrap_or(0.0);
    let wind_kts = xp.get_float(datarefs::WIND_SPEED_MSC).await.unwrap_or(0.0) * MS_TO_KTS;
    if wind_kts < 3.0 {
        return format!(
            "Wind's calm. Any runway works; {icao} has {}.",
            all.join(" and ")
        );
    }
    let Some((ident, _, headwind)) = nav::best_runway_for_wind(&runways, wind_dir, wind_kts) else {
        return format!("I don't have runway data for {icao}.");
    };
    let component = if headwind >= 0.0 {
        format!("{headwind:.0} knots of headwind")
    } else {
        format!("mind the {:.0} knots of tailwind", headwind.abs())
    };
    format!(
        "Wind's {wind_dir:.0} at {wind_kts:.0}. Runway {ident}, {component}. {icao} has {}.",
        all.join(" and ")
    )
}

/// First element of a per-engine array dataref (engine 1), or `None` on a failed
/// read. Engine indicators are arrays, so they bypass the scalar `read1`/cache.
async fn read_engine0(xp: &XPlaneClient, name: &str) -> Option<f64> {
    xp.get_float_array(name)
        .await
        .ok()
        .and_then(|v| v.first().copied())
}

/// Live "are all systems within safe parameters?" check — reads the engine,
/// vacuum, and electrical instruments and reports status against the same limits
/// the monitor uses, instead of reciting static POH numbers. (oil temp / CHT
/// datarefs read °F despite their names.) The deterministic answer to "check all
/// systems".
async fn systems_check(xp: &XPlaneClient, profile: &aircraft::AircraftProfile) -> String {
    let running = read_engine0(xp, "sim/flightmodel/engine/ENGN_running")
        .await
        .is_some_and(|v| v > 0.5);
    if !running {
        return "Engine's not running, nothing to check.".to_string();
    }
    let mut ok: Vec<String> = Vec::new();
    let mut flags: Vec<String> = Vec::new();
    if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_pressure_psi").await {
        // Judge against the sim's own gauge red-lines, not a fixed piston band:
        // the modeled PT6 sits ~127 PSI in normal cruise, which the old 20..=105
        // flagged all flight (seen live on the Evolution). Fall back to the
        // piston band when the .acf carries no sensible arc.
        let (lo, hi) = match (
            xp.get_float("sim/aircraft/limits/red_lo_oilP").await,
            xp.get_float("sim/aircraft/limits/red_hi_oilP").await,
        ) {
            (Ok(lo), Ok(hi)) if hi > lo && hi > 0.0 => (lo, hi),
            _ => (20.0, 105.0),
        };
        if (lo..=hi).contains(&v) {
            ok.push(format!("oil pressure {v:.0}"));
        } else {
            flags.push(format!("oil pressure {v:.0} pounds"));
        }
    }
    if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_temperature_deg_C").await
    {
        if v < 240.0 {
            ok.push(format!("oil temp {v:.0}"));
        } else {
            flags.push(format!("oil temp {v:.0} degrees"));
        }
    }
    // CHT is a piston reading; a turbine profile (no piston `engine` group) has
    // no cylinders to report.
    if profile.engine {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/CHT_deg_C").await {
            if v < 450.0 {
                ok.push(format!("C H T {v:.0}"));
            } else {
                flags.push(format!("C H T {v:.0} degrees"));
            }
        }
    }
    // Suction only matters where a vacuum gyro exists — a glass/turbine panel
    // (Evolution, SR22) has none, and the stale reading spoke "low vacuum 3.8"
    // on every check (seen live).
    if profile.directional_gyro {
        if let Ok(v) = xp.get_float("sim/cockpit/misc/vacuum").await {
            if v >= 4.0 {
                ok.push(format!("vacuum {v:.1}"));
            } else {
                flags.push(format!("low vacuum {v:.1}"));
            }
        }
    }
    if let Some(v) = read_engine0(xp, "sim/cockpit2/electrical/battery_amps").await {
        if v >= -5.0 {
            ok.push("charging".to_string());
        } else {
            flags.push(format!("battery discharging {:.0} amps", v.abs()));
        }
    }
    if flags.is_empty() {
        // Verdict first, gauge recital never: a crew member concludes ("all in
        // the green") and keeps the numbers in reserve — each one is a "say the
        // oil pressure" away. Rotating openers so the answer doesn't sound like
        // the same recording every time.
        const VERDICTS: &[&str] = &[
            "Checked it over: everything's in the green.",
            "All in the green, engine's happy.",
            "Looks good up here, nothing out of place.",
        ];
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let i = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) % VERDICTS.len();
        let charging = ok.iter().any(|s| s == "charging");
        format!(
            "{}{}",
            VERDICTS[i],
            if charging {
                " Battery's charging fine."
            } else {
                ""
            }
        )
    } else {
        // Something's off: the flags get the airtime, the rest gets one word.
        format!(
            "Mostly green, but watch this: {}. Everything else is normal.",
            flags.join(", ")
        )
    }
}

/// True for "why have we lost power?" / "the engine quit" phrasings — a
/// diagnosis request, not a throttle command. Adjacent "lost power" wording (not
/// bare "power" + "lost") so "reduce power, we lost too much altitude" still
/// commands; engine fail-words are past-tense so "stop the engine" still
/// commands too.
fn is_power_loss_query(t: &str) -> bool {
    t.contains("lost power")
        || t.contains("losing power")
        || t.contains("lose power")
        || t.contains("power loss")
        || t.contains("loss of power")
        || (t.contains("engine")
            && (t.contains("quit")
                || t.contains("stopped")
                || t.contains("died")
                || t.contains("dead")
                || t.contains("failed")
                || t.contains("failure")
                || t.contains("cut out")
                || t.contains("cutting out")
                || (t.contains("why") && (t.contains("lost") || t.contains("lose")))))
}

/// Deterministic power-loss diagnosis ("why have we lost power?") — walk the
/// engine's fuel and ignition chain and name the likely culprit, most common
/// first, with the remedy in the same breath (the pilot asking this is about to
/// be busy). Same trust model as `systems_check`: read the datarefs, never the
/// LLM.
async fn power_loss_diagnosis(xp: &XPlaneClient, profile: &aircraft::AircraftProfile) -> String {
    let running = read_engine0(xp, "sim/flightmodel/engine/ENGN_running")
        .await
        .is_some_and(|v| v > 0.5);
    let piston = profile.turboprop_start.is_none() && !profile.turbine_start;
    let mut causes: Vec<String> = Vec::new();

    // Fuel chain, most likely first. Selector OFF starves the engine within a
    // minute (and an xlua plugin reload can snap it there behind your back).
    let selector = xp
        .get_float("sim/cockpit2/fuel/fuel_tank_selector")
        .await
        .ok();
    if profile.fuel_selector.is_some() && selector.is_some_and(|s| s < 0.5) {
        causes.push("the fuel selector is off, select a tank now".to_string());
    }
    let tanks = xp
        .get_float_array(datarefs::FUEL_PER_TANK_KG)
        .await
        .unwrap_or_default();
    let total_kg: f64 = tanks.iter().sum();
    if !tanks.is_empty() && total_kg < 2.0 {
        causes.push("the tanks are dry, we're out of fuel".to_string());
    } else if tanks.len() == 2 {
        // Feeding from a dry side while the other wing still has fuel
        // (sim selector positions: 1 = left, 3 = right).
        match selector.map(|s| s.round() as i64) {
            Some(1) if tanks[0] < 1.0 && tanks[1] > 2.0 => {
                causes.push("the left tank is dry, switch to the right".to_string());
            }
            Some(3) if tanks[1] < 1.0 && tanks[0] > 2.0 => {
                causes.push("the right tank is dry, switch to the left".to_string());
            }
            _ => {}
        }
    }
    if piston {
        if read_engine0(xp, "sim/cockpit2/engine/actuators/mixture_ratio")
            .await
            .is_some_and(|m| m < 0.15)
        {
            causes.push("the mixture is at cutoff, push it rich".to_string());
        }
        if read_engine0(xp, "sim/cockpit2/engine/actuators/ignition_key")
            .await
            .is_some_and(|k| k < 0.5)
        {
            causes.push("the ignition is off, mags to both".to_string());
        }
    }
    // Fuel not reaching the engine with everything above in order.
    let flow_kg_s = xp
        .get_float_array(datarefs::FUEL_FLOW_KG_SEC)
        .await
        .map(|f| f.iter().sum::<f64>())
        .unwrap_or(0.0);
    if causes.is_empty() && !running && flow_kg_s < 1e-5 && total_kg > 2.0 {
        causes.push(
            "no fuel flow with fuel on board, check the boost pump and fuel pressure".to_string(),
        );
    }
    let throttle = read_engine0(xp, "sim/cockpit2/engine/actuators/throttle_ratio")
        .await
        .unwrap_or(0.0);
    if running && causes.is_empty() && throttle < 0.05 {
        causes.push("the throttle is closed".to_string());
    }

    if !causes.is_empty() {
        let state = if running {
            "Engine's still turning"
        } else {
            "Engine's stopped"
        };
        return format!("{state}. Likely cause: {}.", causes.join("; also "));
    }
    // Nothing conclusive: give the raw picture so the pilot can work the problem.
    let rpm = read_engine0(xp, "sim/cockpit2/engine/indicators/prop_speed_rpm")
        .await
        .unwrap_or(0.0);
    let gph = flow_kg_s * 3600.0 / AVGAS_KG_PER_GAL;
    let mut line = if running {
        format!(
            "Nothing obviously wrong. Engine's running at {rpm:.0} RPM, fuel flow {gph:.0} gallons an hour, throttle {:.0} percent",
            throttle * 100.0
        )
    } else {
        format!(
            "Engine's stopped and the fuel and ignition look set. Fuel flow {gph:.1}, throttle {:.0} percent",
            throttle * 100.0
        )
    };
    if profile.carb_heat {
        line.push_str(". Could be carburettor ice, try carb heat");
    }
    format!("{line}.")
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
        return Err(
            "No SimBrief ID set. Add XCOPILOT_SIMBRIEF_ID and I can pull your flight plan."
                .to_string(),
        );
    }
    let fresh = cache
        .as_ref()
        .is_some_and(|(t, _)| t.elapsed() < FLIGHT_PLAN_TTL);
    if refresh || !fresh {
        match simbrief::fetch(&config.simbrief_id).await {
            Ok(fp) => *cache = Some((Instant::now(), fp)),
            Err(e) => {
                tracing::warn!("simbrief fetch failed: {e:#}");
                if cache.is_none() {
                    return Err(
                        "Couldn't fetch your SimBrief plan. Generate one first, or check the ID."
                            .to_string(),
                    );
                }
            }
        }
    }
    // Every path that yields a plan also hands the monitor's descent advisor
    // its destination — the copilot volunteers TOD instead of waiting to be
    // asked ("when is TOD" still works via time_to_tod).
    if let Some((_, fp)) = cache.as_ref() {
        if let Some(w) = fp.waypoints.last() {
            let ring = if fp.tod_to_dest_nm > 0.0 {
                fp.tod_to_dest_nm
            } else {
                fp.cruise_alt_ft / 1000.0 * TOD_NM_PER_1000FT
            };
            // Field elevation comes from apt.dat, the authoritative source — the
            // plan's destination-waypoint altitude is a *crossing* altitude
            // (spoken live as "4600 feet" for a 2,084 ft field, and it skews the
            // descent advisor's profile math too). Cached per ICAO: the scan of
            // the 400 MB file runs once per destination, not per TTL refresh.
            static ELEV_CACHE: std::sync::Mutex<Option<(String, f64)>> =
                std::sync::Mutex::new(None);
            let cached = ELEV_CACHE
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
                .filter(|(icao, _)| *icao == fp.destination);
            let elev_ft = match cached {
                Some((_, e)) => e,
                None => {
                    let (path, icao) = (config.apt_data_path.clone(), fp.destination.clone());
                    let scanned =
                        tokio::task::spawn_blocking(move || nav::find_airport(&path, &icao))
                            .await
                            .ok()
                            .and_then(|r| r.ok())
                            .flatten()
                            .map(|a| a.elev_ft);
                    // Fallback: the waypoint altitude when plausible, else sea
                    // level (which only makes TOD earlier — the safe direction).
                    let e = scanned.unwrap_or(if (0.0..12_000.0).contains(&w.alt) {
                        w.alt
                    } else {
                        0.0
                    });
                    *ELEV_CACHE.lock().unwrap_or_else(|e| e.into_inner()) =
                        Some((fp.destination.clone(), e));
                    e
                }
            };
            *monitor::DESTINATION
                .lock()
                .unwrap_or_else(|e| e.into_inner()) = Some(monitor::Destination {
                lat: w.lat,
                lon: w.lon,
                elev_ft,
                tod_ring_nm: ring,
            });
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
    let (origin, dest, n) = (
        fp.origin.clone(),
        fp.destination.clone(),
        fp.waypoints.len(),
    );
    if xp
        .set_data("xcopilot/fms/route", payload.as_bytes())
        .await
        .is_err()
        || xp.set_int("xcopilot/fms/load", 1).await.is_err()
    {
        return "Couldn't reach the plugin to load the route. Is the XVoiceLayer plugin running?"
            .to_string();
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
            line.push_str(&format!(
                " You're at {alt:.0}, {diff:.0} to climb to cruise."
            ));
        } else if diff < -300.0 {
            line.push_str(&format!(" You're at {alt:.0}, above planned cruise."));
        } else {
            line.push_str(" At planned cruise.");
        }
    }
    // Top of descent.
    if fp.tod_to_dest_nm > 0.5 {
        line.push_str(&format!(
            " Top of descent about {:.0} miles before {}.",
            fp.tod_to_dest_nm, fp.destination
        ));
    }
    // Fuel check: planned block vs what's in the tanks (read in kg, report gallons).
    if let Ok(kg) = xp.get_float(datarefs::FUEL_TOTAL_KG).await {
        let aboard = kg / AVGAS_KG_PER_GAL;
        if aboard + 0.5 >= fp.block_fuel_gal {
            line.push_str(&format!(
                " You have {aboard:.0} gallons aboard, enough for the plan."
            ));
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
            && (t.contains("aircraft")
                || t.contains("plane")
                || t.contains("cessna")
                || t.contains("172")
                || t.contains("skyhawk")));
    if !asks {
        return None;
    }
    let r = profile.reference.as_ref();
    // The type descriptor comes from the profile (e.g. "twin-engine jet"); without
    // one, just name the aircraft rather than assert an engine count.
    let mut line = match r.and_then(|r| r.kind.as_deref()) {
        Some(kind) => format!("We're in the {}, a {kind}.", profile.display_name),
        None => format!("We're in the {}.", profile.display_name),
    };
    if let Some(v) = r.and_then(|r| r.cruise_kts) {
        line.push_str(&format!(" Cruise about {v:.0} knots."));
    }
    if let Some(v) = profile.vne_kts {
        // Vne for piston singles, Vmo for jets — "max speed" reads right for both.
        line.push_str(&format!(" Max speed {v:.0}."));
    }
    if let Some(v) = r.and_then(|r| r.service_ceiling_ft) {
        line.push_str(&format!(" Service ceiling {} feet.", with_commas(v)));
    }
    if let (Some(g), Some(h)) = (
        r.and_then(|r| r.fuel_useable_gal),
        r.and_then(|r| r.endurance_hours),
    ) {
        line.push_str(&format!(
            " {g:.0} gallons useable for roughly {h:.0} hours.",
        ));
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
    if t.contains("vfe")
        || t.contains("flap speed")
        || t.contains("flaps speed")
        || t.contains("flap limit")
    {
        if let Some(p) = vfe_phrase(profile) {
            parts.push(p);
        }
    }
    if t.contains("stall") {
        match (profile.vs_clean_kts, r.and_then(|r| r.vs0_kts)) {
            (Some(c), Some(f)) => {
                parts.push(format!("stall {c:.0} knots clean, {f:.0} with full flaps"))
            }
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
    if t.contains("rotat")
        || t.contains("liftoff")
        || t.contains("lift off")
        || t.contains("takeoff speed")
    {
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
        match (
            r.and_then(|r| r.fuel_capacity_gal),
            r.and_then(|r| r.fuel_useable_gal),
        ) {
            (Some(c), Some(u)) => {
                parts.push(format!("fuel capacity {c:.0} gallons, {u:.0} useable"))
            }
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
    if t.contains("redline")
        || t.contains("max rpm")
        || t.contains("maximum rpm")
        || t.contains("red line")
    {
        if let Some(v) = r.and_then(|r| r.max_rpm) {
            parts.push(format!("maximum RPM {}", with_commas(v)));
        }
    }
    // A general "performance limits" / "V-speeds" ask gets the key summary.
    if parts.is_empty()
        && (t.contains("performance limit")
            || t.contains("v speed")
            || t.contains("v-speed")
            || (t.contains("limits") && t.contains("performance")))
    {
        let mut s: Vec<String> = Vec::new();
        if let Some(v) = profile.vne_kts {
            s.push(format!("Vne {v:.0}"));
        }
        if let Some(v) = r.and_then(|r| r.vno_kts) {
            s.push(format!("Vno {v:.0}"));
        }
        if let Some(v) = r.and_then(|r| r.va_kts) {
            s.push(format!("Va {v:.0}"));
        }
        if let Some(v) = r.and_then(|r| r.vy_kts) {
            s.push(format!("Vy {v:.0}"));
        }
        if let Some(v) = r.and_then(|r| r.best_glide_kts) {
            s.push(format!("best glide {v:.0}"));
        }
        if let Some(v) = profile.vs_clean_kts {
            s.push(format!("stall {v:.0} clean"));
        }
        if let Some(v) = r.and_then(|r| r.max_gross_lb) {
            s.push(format!("max gross {} pounds", with_commas(v)));
        }
        if let Some(v) = r.and_then(|r| r.service_ceiling_ft) {
            s.push(format!("service ceiling {} feet", with_commas(v)));
        }
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
        if i > 0 && (bytes.len() - i).is_multiple_of(3) {
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
        || t.contains("fill")
        // Tank-SELECTOR commands ("switch tanks", "switch fuel to right", "set
        // fuel to left") — these hijacked the quantity report (seen live, with and
        // without "tank" said). A swap verb or an explicit "to <side>" is a
        // command; "how much fuel in the left tank" has neither and stays a query.
        // "select"/"set"/"feed" alone stay tank-gated so phrasings like "what's
        // the fuel setting" keep reading as questions.
        || t.contains("switch") || t.contains("swap") || t.contains("alternate")
        || t.contains("rotate")
        || t.contains("to left") || t.contains("to right") || t.contains("to off")
        || (t.contains("tank")
            && (t.contains("change") || t.contains("select") || t.contains("set")
                || t.contains("feed")));
    // "fuel flow" is an engine-instrument read, answered in answer_state_query.
    asks && !command && !t.contains("flow")
}

/// True if the utterance is asking for taxi directions — the on-demand side
/// of the taxi guide ("which way?", "say the taxi route", "where's the
/// holding point"). Deliberately question-shaped: a taxi READBACK ("taxi to
/// holding point Alpha via Whiskey...") contains the same nouns but none of
/// the question markers, and must fall through to the radio path untouched.
fn is_taxi_query(text: &str) -> bool {
    let t = text.to_ascii_lowercase();
    let way = t.contains("which way")
        || t.contains("where do i go")
        || t.contains("where do we go")
        || t.contains("where are we going")
        || t.contains("where to now")
        || t.contains("which taxiway");
    let hp = t.contains("holding point")
        && (t.contains("where") || t.contains("which") || t.contains("how far"));
    let route = (t.contains("taxi") || t.contains(" route"))
        && (t.contains("what")
            || t.contains("which")
            || t.contains("where")
            || t.contains("say")
            || t.contains("repeat")
            || t.contains("remind"));
    way || hp || route
}

/// True when the fuel question is about *time* remaining ("how long will the fuel
/// last", "what's our endurance", "how much longer can we fly") rather than the
/// quantity on board. Drives `fuel_report` to lead with the duration.
fn is_endurance_query(text: &str) -> bool {
    let t = text.to_ascii_lowercase();
    t.contains("endurance")
        || t.contains("how long")
        || t.contains("how much longer")
        || (t.contains("last") && (t.contains("fuel") || t.contains("gas")))
}

/// Deterministic spoken fuel report — total in gallons, endurance from the
/// profile's cruise burn, and the left/right balance. Read straight from the
/// datarefs (cache when warm, REST otherwise); never goes near the LLM.
async fn fuel_report(
    xp: &XPlaneClient,
    profile: &aircraft::AircraftProfile,
    live: Option<&LiveState>,
    endurance_first: bool,
) -> String {
    let Ok(total_kg) = read1(live, xp, datarefs::FUEL_TOTAL_KG).await else {
        return "I can't read the fuel right now — check the connection to X-Plane.".to_string();
    };
    let gal = total_kg / AVGAS_KG_PER_GAL;

    // Endurance: prefer the live burn (sum of per-engine fuel flow) when the
    // engines are running, since it reflects the actual power setting; fall back
    // to the profile's cruise figure on the ground or with engines shut down.
    let live_burn_kg_hr = xp
        .get_float_array(datarefs::FUEL_FLOW_KG_SEC)
        .await
        .map(|f| f.iter().sum::<f64>() * 3600.0)
        .unwrap_or(0.0);
    let burn_kg_hr = if live_burn_kg_hr > 1.0 {
        live_burn_kg_hr
    } else {
        profile
            .fuel
            .as_ref()
            .map(|f| f.cruise_burn_kg_hr)
            .unwrap_or(0.0)
    };
    let basis = if live_burn_kg_hr > 1.0 {
        "at this burn"
    } else {
        "at cruise"
    };

    let dur = if burn_kg_hr > 0.0 {
        let hours = total_kg / burn_kg_hr;
        let (h, m) = (hours.trunc() as i32, (hours.fract() * 60.0).round() as i32);
        let mut d = String::new();
        if h > 0 {
            d.push_str(&format!("{h} hour{}", if h == 1 { "" } else { "s" }));
        }
        if m > 0 {
            if !d.is_empty() {
                d.push(' ');
            }
            d.push_str(&format!("{m} minute{}", if m == 1 { "" } else { "s" }));
        }
        d
    } else {
        String::new()
    };

    // An endurance question wants the *time* first; a plain fuel question leads
    // with the quantity and tacks the duration on.
    let mut line = if endurance_first && !dur.is_empty() {
        format!("About {dur} of fuel {basis}, {gal:.0} gallons on board")
    } else {
        let mut l = format!("{gal:.0} gallons on board");
        if !dur.is_empty() {
            l.push_str(&format!(", about {dur} {basis}"));
        }
        l
    };
    // Per-tank balance, when the airframe has two tanks.
    if let Ok(tanks) = xp.get_float_array(datarefs::FUEL_PER_TANK_KG).await {
        if tanks.len() == 2 {
            let (l, r) = (tanks[0] / AVGAS_KG_PER_GAL, tanks[1] / AVGAS_KG_PER_GAL);
            line.push_str(&format!(". Left {l:.0}, right {r:.0}"));
            if (l - r).abs() >= 5.0 {
                line.push_str(if l > r {
                    " — favor the right"
                } else {
                    " — favor the left"
                });
            }
        }
    }
    // The answer a copilot actually gives is not "what's in the tanks" but
    // "what's left when we get there": project to the planned destination at the
    // live burn and judge it against the airframe's reserve. Airborne with a
    // known destination and real groundspeed only.
    let dest = monitor::DESTINATION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone();
    if let Some(d) = dest {
        if live_burn_kg_hr > 1.0 {
            let (lat, lon, gs_ms) = (
                xp.get_float("sim/flightmodel/position/latitude")
                    .await
                    .unwrap_or(0.0),
                xp.get_float("sim/flightmodel/position/longitude")
                    .await
                    .unwrap_or(0.0),
                xp.get_float("sim/flightmodel/position/groundspeed")
                    .await
                    .unwrap_or(0.0),
            );
            let gs = gs_ms * MS_TO_KTS;
            if gs > 40.0 {
                let dist = nav::haversine_nm(lat, lon, d.lat, d.lon);
                let at_dest_kg = total_kg - live_burn_kg_hr * (dist / gs);
                let mins_left = (at_dest_kg.max(0.0) / live_burn_kg_hr) * 60.0;
                let reserve = profile
                    .fuel
                    .as_ref()
                    .map(|f| f.reserve_caution_min)
                    .unwrap_or(45.0);
                let at_dest_gal = at_dest_kg / AVGAS_KG_PER_GAL;
                let verdict = if at_dest_kg <= 0.0 {
                    "that doesn't reach the destination — plan a fuel stop".to_string()
                } else if mins_left >= reserve + 15.0 {
                    format!("comfortably above your {reserve:.0}-minute reserve")
                } else if mins_left >= reserve {
                    format!("just above your {reserve:.0}-minute reserve, keep it leaned")
                } else {
                    format!("below your {reserve:.0}-minute reserve — think about a stop short")
                };
                if at_dest_kg > 0.0 {
                    line.push_str(&format!(
                        ". Should land with about {at_dest_gal:.0} gallons, {verdict}"
                    ));
                } else {
                    line.push_str(&format!(". At this burn, {verdict}"));
                }
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
/// Is the battery voltage low (depleted state of charge)? System-class-aware, so
/// it doesn't flag a healthy 12V-system battery as low the way a fixed 24V
/// threshold would: ~24V-class systems sit near 24-28V (low below ~23), ~12V-class
/// near 12-14V (low below ~11.5). The magnitude selects the class.
fn battery_low(volts: f64) -> bool {
    if volts >= 18.0 {
        volts < 23.0 // 24V-class system
    } else {
        volts < 11.5 // 12V-class system
    }
}

async fn answer_state_query(
    cmd: &str,
    xp: &XPlaneClient,
    live: Option<&LiveState>,
    turbine: bool,
) -> Option<String> {
    let t = cmd.to_ascii_lowercase();
    let mut parts: Vec<String> = Vec::new();

    // System-wide failure check — reads the COMPLETE X-Plane failure set (see
    // failures.rs), never a keyword subset, so a failure with an off-pattern name
    // (a seized engine lives at rel_seize_0, no "engine" in it) can't hide. It's
    // an early return because the scan is ~800 concurrent reads — a beat slower
    // than a single value, so it only runs when the pilot actually asks.
    let asks_failures = t.contains("what's wrong")
        || t.contains("whats wrong")
        || t.contains("anything wrong")
        || t.contains("any failure")
        || t.contains("failures")
        || t.contains("anything fail")
        || t.contains("anything broken")
        || t.contains("malfunction")
        || t.contains("systems check")
        || t.contains("system status")
        || t.contains("systems status")
        || t.contains("status check")
        || t.contains("all systems")
        || t.contains("everything ok");
    if asks_failures {
        return Some(match failures::scan(xp).await {
            Ok(f) => failures::summary(&f)
                .unwrap_or_else(|| "All systems normal, no active failures.".to_string()),
            Err(_) => "I couldn't read the aircraft failure system.".to_string(),
        });
    }

    // Current control settings (e.g. "what's the mixture/throttle set to").
    if t.contains("mixture") {
        if let Ok(v) = read1(live, xp, "sim/cockpit2/engine/actuators/mixture_ratio_all").await {
            parts.push(format!("mixture {:.0} percent", v * 100.0));
        }
    }
    // "autothrottle" / "auto throttle" is the autoflight state, not the lever
    // position — answer the engage state and don't let the bare-throttle branch
    // below also fire ("autothrottle" contains "throttle").
    let autothrottle_q =
        t.contains("autothrottle") || t.contains("auto throttle") || t.contains("auto-throttle");
    if autothrottle_q {
        if let Ok(v) = read1(live, xp, "sim/cockpit2/autopilot/autothrottle_enabled").await {
            parts.push(match v {
                v if v >= 1.5 => "autothrottle engaged".to_string(),
                v if v >= 0.5 => "autothrottle armed".to_string(),
                _ => "autothrottle off".to_string(),
            });
        }
    }
    if t.contains("throttle") && !autothrottle_q {
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
    if t.contains("airspeed")
        || t.contains("air speed")
        || t.contains("how fast")
        || (t.contains("speed") && !vs_q)
    {
        if let Ok(v) = read1(live, xp, datarefs::AIRSPEED_KTS).await {
            parts.push(format!("{v:.0} knots"));
        }
    }
    // Weather — answered deterministically, not via the LLM (same reliability fix
    // as fuel). "weather" gives the lot; specific words give just that field.
    let wx = t.contains("weather") || t.contains("metar") || t.contains("conditions");
    if wx || t.contains("wind") {
        if let (Ok(dir), Ok(spd_ms)) = (
            read1(live, xp, datarefs::WIND_DIR_DEG_TRUE).await,
            read1(live, xp, datarefs::WIND_SPEED_MSC).await,
        ) {
            let kts = spd_ms * MS_TO_KTS;
            parts.push(if kts < 1.0 {
                "wind calm".to_string()
            } else {
                format!("wind {dir:03.0} at {kts:.0} knots")
            });
        }
    }
    if wx || t.contains("visibility") {
        if let Ok(v) = read1(live, xp, datarefs::VISIBILITY_SM).await {
            parts.push(format!("visibility {v:.0} statute miles"));
        }
    }
    // "pressure" alone could mean the altimeter or the oil — exclude oil pressure
    // so "how's the oil pressure?" doesn't also blurt the altimeter setting.
    if wx
        || t.contains("altimeter")
        || t.contains("qnh")
        || t.contains("barometric")
        || (t.contains("pressure") && !t.contains("oil"))
    {
        if let Ok(v) = read1(live, xp, datarefs::QNH_PASCALS).await {
            // qnh in Pascals → altimeter setting in inHg (1 inHg = 3386.389 Pa).
            parts.push(format!("altimeter {:.2}", v / 3386.389));
        }
    }
    if wx
        || (t.contains("temperature") && !t.contains("oil"))
        || t.contains("how cold")
        || t.contains("how hot")
        || t.contains(" oat")
    {
        if let Ok(v) = read1(live, xp, datarefs::OAT_DEG_C).await {
            parts.push(format!("outside air temperature {v:.0} degrees celsius"));
        }
    }

    // General "how's the engine?" → a one-line health summary. Only when no
    // specific instrument is named (those are handled below, no double-up).
    if t.contains("engine")
        && ![
            "oil",
            "egt",
            "exhaust",
            "itt",
            "cht",
            "cylinder",
            "rpm",
            "n1",
            "n2",
            "fuel flow",
        ]
        .iter()
        .any(|k| t.contains(k))
    {
        let running = read_engine0(xp, "sim/flightmodel/engine/ENGN_running")
            .await
            .is_some_and(|v| v > 0.5);
        if !running {
            parts.push(if turbine { "engines off" } else { "engine off" }.to_string());
        } else if turbine {
            // Turbine summary: spool speeds (N1/N2), turbine temp (ITT), oil, burn.
            let mut bits = vec!["engines running".to_string()];
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N1_percent").await {
                bits.push(format!("N 1 {v:.0} percent"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N2_percent").await {
                bits.push(format!("N 2 {v:.0} percent"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/ITT_deg_C").await {
                bits.push(format!("I T T {v:.0} degrees"));
            }
            if let Some(v) =
                read_engine0(xp, "sim/cockpit2/engine/indicators/oil_pressure_psi").await
            {
                bits.push(format!("oil pressure {v:.0} pounds"));
            }
            if let Some(v) =
                read_engine0(xp, "sim/cockpit2/engine/indicators/fuel_flow_kg_sec").await
            {
                bits.push(format!(
                    "fuel flow {:.0} pounds an hour",
                    v * 3600.0 * LB_PER_KG
                ));
            }
            parts.push(bits.join(", "));
        } else {
            let mut bits = vec!["engine running".to_string()];
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/prop_speed_rpm").await
            {
                bits.push(format!("{v:.0} R P M"));
            }
            if let Some(v) =
                read_engine0(xp, "sim/cockpit2/engine/indicators/oil_pressure_psi").await
            {
                bits.push(format!("oil pressure {v:.0} pounds"));
            }
            // oil_temp / CHT datarefs read °F despite their names.
            if let Some(v) =
                read_engine0(xp, "sim/cockpit2/engine/indicators/oil_temperature_deg_C").await
            {
                bits.push(format!("oil temp {v:.0} degrees"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/CHT_deg_C").await {
                bits.push(format!("C H T {v:.0}"));
            }
            if let Some(v) =
                read_engine0(xp, "sim/cockpit2/engine/indicators/fuel_flow_kg_sec").await
            {
                bits.push(format!(
                    "fuel flow {:.1} gallons an hour",
                    v * 3600.0 / AVGAS_KG_PER_GAL
                ));
            }
            parts.push(bits.join(", "));
        }
    }

    // Turbine spool speeds, asked directly ("what's the N1?").
    if turbine && (t.contains("n1") || t.contains("n 1")) {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N1_percent").await {
            parts.push(format!("N 1 {v:.0} percent"));
        }
    }
    if turbine && (t.contains("n2") || t.contains("n 2")) {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N2_percent").await {
            parts.push(format!("N 2 {v:.0} percent"));
        }
    }
    if turbine && (t.contains("itt") || t.contains("turbine temp")) {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/ITT_deg_C").await {
            parts.push(format!("I T T {v:.0} degrees"));
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
        if let Some(v) =
            read_engine0(xp, "sim/cockpit2/engine/indicators/oil_temperature_deg_C").await
        {
            parts.push(format!("oil temperature {v:.0} degrees"));
        }
    }
    if !turbine && (t.contains("egt") || t.contains("exhaust")) {
        // EGT_deg_cel is genuinely Celsius → convert to °F for the readout. (On a
        // turbine the equivalent is ITT, handled by the n1/n2/itt block above; route
        // "exhaust temp" there too so a jet doesn't fall through to nothing.)
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/EGT_deg_cel").await {
            parts.push(format!("exhaust gas temperature {:.0} degrees", c_to_f(v)));
        }
    }
    if turbine && (t.contains("egt") || t.contains("exhaust")) {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/ITT_deg_C").await {
            parts.push(format!("I T T {v:.0} degrees"));
        }
    }
    if t.contains("cht") || t.contains("cylinder") {
        if turbine {
            parts.push("no cylinder head temp on a jet — that's a piston gauge".to_string());
        } else if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/CHT_deg_C").await {
            // CHT_deg_C is misnamed — reads °F already.
            parts.push(format!("cylinder head temperature {v:.0} degrees"));
        }
    }
    if t.contains("rpm") || t.contains("r p m") || t.contains("engine speed") {
        if turbine {
            // A jet's "engine speed" is the spool speeds, not prop RPM.
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N1_percent").await {
                parts.push(format!("N 1 {v:.0} percent"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N2_percent").await {
                parts.push(format!("N 2 {v:.0} percent"));
            }
        } else if let Some(v) =
            read_engine0(xp, "sim/cockpit2/engine/indicators/prop_speed_rpm").await
        {
            parts.push(format!("{v:.0} R P M"));
        }
    }
    if t.contains("fuel flow") || t.contains("flow") || t.contains("burning") {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/fuel_flow_kg_sec").await {
            if turbine {
                parts.push(format!(
                    "fuel flow {:.0} pounds an hour",
                    v * 3600.0 * LB_PER_KG
                ));
            } else {
                parts.push(format!(
                    "fuel flow {:.1} gallons an hour",
                    v * 3600.0 / AVGAS_KG_PER_GAL
                ));
            }
        }
    }
    if t.contains("ammeter")
        || t.contains("amps")
        || t.contains("charging")
        || t.contains("battery")
        || t.contains("voltage")
        || t.contains("volts")
        || t.contains("state of charge")
    {
        // Report VOLTAGE and amps together. Amps alone is misleading: a dead-flat
        // battery reads "+15 amps charging" while sitting at 20 volts, going
        // nowhere (seen live). Voltage is the state-of-charge/health number; amps
        // is only the direction of flow. The copilot used to read only amps.
        if let Some(v) =
            read_engine0(xp, "sim/cockpit2/electrical/battery_voltage_actual_volts").await
        {
            let health = if battery_low(v) { " (low)" } else { "" };
            parts.push(format!("battery {v:.1} volts{health}"));
        }
        if let Some(v) = read_engine0(xp, "sim/cockpit2/electrical/bus_volts").await {
            parts.push(format!("bus {v:.1} volts"));
        }
        if let Some(v) = read_engine0(xp, "sim/cockpit2/electrical/battery_amps").await {
            let state = if v > 1.0 {
                "charging"
            } else if v < -1.0 {
                "discharging"
            } else {
                "steady"
            };
            parts.push(format!("{v:+.0} amps, {state}"));
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
        parts.push(format!(
            "on the ground: {}",
            if v > 0.5 { "yes" } else { "no" }
        ));
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
        parts.push(format!(
            "engines {}",
            if engine { "running" } else { "off" }
        ));
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
        managing: auto_status::active_managers(),
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
        "speedbrake" => &[(
            "sim/cockpit2/controls/speedbrake_ratio",
            "the speedbrakes",
            F,
        )],
        "set_throttle" => &[(
            "sim/cockpit2/engine/actuators/throttle_ratio_all",
            "throttle",
            F,
        )],
        "set_mixture" | "lean_mixture" => &[(
            "sim/cockpit2/engine/actuators/mixture_ratio_all",
            "mixture",
            F,
        )],
        "set_pitch_trim" => &[("sim/cockpit2/controls/elevator_trim", "pitch trim", F)],
        "set_barometer" | "set_altimeter_current" => &[(
            "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot",
            "the altimeter",
            F,
        )],
        "set_parking_brake" => &[(
            "sim/cockpit2/controls/parking_brake_ratio",
            "the parking brake",
            F,
        )],
        "set_heading" => &[(
            "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot",
            "the heading bug",
            F,
        )],
        "set_altitude" => &[(
            "sim/cockpit2/autopilot/altitude_dial_ft",
            "the altitude select",
            F,
        )],
        "set_vertical_speed" | "set_climb" => &[(
            "sim/cockpit/autopilot/vertical_velocity",
            "vertical speed",
            F,
        )],
        "set_transponder" => &[(
            "sim/cockpit2/radios/actuators/transponder_code",
            "the squawk",
            I,
        )],
        "set_transponder_mode" => &[(
            "sim/cockpit2/radios/actuators/transponder_mode",
            "the transponder mode",
            I,
        )],
        "set_com1" => &[(
            "sim/cockpit2/radios/actuators/com1_frequency_hz_833",
            "com 1",
            I,
        )],
        "set_com2" => &[(
            "sim/cockpit2/radios/actuators/com2_frequency_hz_833",
            "com 2",
            I,
        )],
        "set_nav1" => &[(
            "sim/cockpit2/radios/actuators/nav1_frequency_hz",
            "nav 1",
            I,
        )],
        "set_obs" => &[(
            "sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot",
            "the OBS",
            F,
        )],
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
fn looks_like_question(text: &str) -> bool {
    let t = text.trim().to_ascii_lowercase();
    if t.ends_with('?') {
        return true;
    }
    const OPENERS: &[&str] = &[
        "what", "what's", "whats", "how", "how's", "hows", "how much", "how many", "how far",
        "how high", "how fast", "where", "where's", "wheres", "when", "when's", "whens", "which",
        "why", "why's", "whys", "who", "who's", "whos", "tell me",
        // Generic auxiliaries: no command starts with these, and they must beat the
        // command grammar ("is our heading 270?" answers, never acts).
        "is", "are", "am", "have", "has",
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
    for aux in ["do", "does", "did"] {
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
fn looks_like_command(text: &str) -> bool {
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
fn looks_like_followup(text: &str) -> bool {
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

/// "I said eighty percent" / "no, two five zero" / "make it forty" — the pilot
/// correcting the VALUE of the last command (whisper heard "8" for "80" and the
/// wrong value already executed; the fix must not dead-end). Returns the bare
/// value remainder when the utterance is a correction prefix plus nothing but a
/// value — the caller re-anchors it on the last command's subject, exactly like
/// a bare-value follow-up. "No, flaps up" returns None (a full command, not a
/// bare value), and so does a lone "no".
fn correction_value(text: &str) -> Option<String> {
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
fn command_subject(cmd: &str) -> String {
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
/// Short chip label for a published service, shared by the radio row and the
/// ATC tab payload.
fn service_label(word: &str) -> &str {
    match word {
        "atis" => "ATIS",
        "clearance" => "CLR",
        "ground" => "GND",
        "tower" => "TWR",
        "approach" => "APP",
        "departure" => "DEP",
        "unicom" => "UNICOM",
        other => other,
    }
}

/// Load, configure, and install the HOME station — the boot path AND the
/// live "ATC back on" toggle share this. The home field's controllers keep
/// the curated per-position voices (seed 0); every other field the region
/// hands us to gets its own crew from the voice pool. Also spawns the
/// background scans (taxi net, ILS, region index) — each guarded so a
/// mid-flight re-enable doesn't pay a boot cost twice.
fn install_home_station(config: &Config, voice: &voice::Voice) -> Option<station::Station> {
    if config.atc_airport.trim().is_empty() {
        return None;
    }
    let home_icao = config.atc_airport.trim().to_uppercase();
    let mut st = station::Station::load(&config.apt_data_path, &home_icao)?;
    st.set_enforce_readback(config.atc_readback);
    // Phraseology follows geography, like the voices: CAP 413 grammar in
    // the UK, FAA grammar at a K/P/C field.
    st.set_dialect(station::dialect_for(&home_icao));
    // Publish the live station's side state (ATIS broadcast, enroute switch
    // identity) for the monitor's ticks.
    st.install();
    voice.set_station_cast(0, Vec::new());
    // Taxi network for named taxi routes — a second apt.dat scan, so a
    // background thread; clearances are route-less until it lands.
    station::spawn_taxi_net_load(config.apt_data_path.clone(), home_icao.clone());
    // ILS + real approach platforms (earth_nav.dat + CIFP scans, also
    // backgrounded): vectors offer the ILS once this lands.
    station::spawn_ils_load(
        config.nav_data_path.clone(),
        config.cifp_dir.clone(),
        home_icao,
    );
    // Enroute region: index every towered field (a full apt.dat pass, so a
    // background thread) so the flight can outgrow its departure airport —
    // the monitor's region_tick hands off to whichever tower is nearest.
    if config.atc_region.trim() != "off" && !station::region_loaded() {
        let path = config.apt_data_path.clone();
        std::thread::Builder::new()
            .name("region-index".into())
            .spawn(move || station::load_region(&path))
            .ok();
    }
    Some(st)
}

/// Content-based ATC routing for setups without a dedicated radio PTT. Physical
/// radio PTT is handled earlier; this is the one fallback door for keyboard PTT
/// and continuous mode, shared by normal crew mode and ATC-only mode.
async fn try_route_content_atc(
    st: &mut station::Station,
    tl: &str,
    dual_ptt: bool,
    voice: &Voice,
    reporter: &Reporter,
    xplane: &XPlaneClient,
) -> bool {
    if dual_ptt || !voice.radio_powered() || voice.crashed() || voice.replay() || voice.paused() {
        return false;
    }
    let Some((reply, who)) = st.try_handle(tl, xplane.view()).await else {
        return false;
    };
    reporter.status(ui::Phase::Speaking);
    reporter.log(ui::LogKind::Readback, format!("{}: {reply}", who.tag()));
    voice.say_station(Priority::Readback, who, &reply);
    reporter.status(ui::Phase::Listening);
    true
}

/// The crew's comm note after a transmission into dead air. In range, the
/// short answer: who WILL answer, not the field's whole comm directory (six
/// frequencies recited after one void call on ground read as noise,
/// 2026-07-20 — the full list still answers the spoken "what frequencies"
/// QUESTION via `freq_briefing`). The station is only worth pointing at
/// while it's in VHF range — parked 200 nm out, "Gibraltar: tower one three
/// one decimal two" reads as if Gibraltar could answer (2026-07-10). Out of
/// range, name what IS reachable instead, or say honestly that nothing is.
async fn void_briefing(xp: &XPlaneClient, st: &station::Station) -> String {
    let com1 = xp
        .get_float(datarefs::COM1_FREQ_KHZ)
        .await
        .ok()
        .map(|v| v.round() as i64);
    let airborne = xp
        .get_float(datarefs::ON_GROUND_ANY)
        .await
        .is_ok_and(|v| v < 0.5);
    let Some((lat, lon, alt_ft)) = position(xp).await else {
        // Can't place the aircraft — pointing at the station beats silence.
        return st.void_note(com1, airborne);
    };
    if station::active_station_in_range(lat, lon, alt_ft).is_some() {
        return st.void_note(com1, airborne);
    }
    let near = station::fields_in_range(lat, lon, alt_ft, 1);
    match near.first().and_then(|f| {
        f.services
            .iter()
            .find(|(w, _)| w.as_str() == "tower")
            .map(|(_, khz)| (f, *khz))
    }) {
        Some((f, khz)) => format!(
            "{} is out of radio range. Nearest station is {}, {} miles away, tower on {}.",
            station::spoken_airport(st.airport_name()),
            station::spoken_airport(&f.name),
            f.dist_nm.round() as i64,
            atis::spoken_mhz(khz)
        ),
        None => format!(
            "{} is out of radio range, and nothing else is in reach from here.",
            station::spoken_airport(st.airport_name())
        ),
    }
}

/// The aircraft's position for the VHF range checks: lat, lon, true altitude
/// in feet MSL. `None` when the sim isn't answering.
async fn position(xp: &XPlaneClient) -> Option<(f64, f64, f64)> {
    let lat = xp.get_float(datarefs::LATITUDE).await.ok()?;
    let lon = xp.get_float(datarefs::LONGITUDE).await.ok()?;
    let elev_m = xp.get_float(datarefs::ELEVATION_M).await.ok()?;
    Some((lat, lon, elev_m * 3.28084))
}

/// The radio row pushed to the in-sim panel: the active station's name and
/// where COM1 actually is — the TUNED service by name (">TWR 118.505", same
/// 5 kHz tolerance as the station) or the raw value when off every service.
/// Just those two chips: the full comm plan lives in the ATC tab now
/// (2026-07-10), and repeating every frequency here was noise. An empty
/// airport (active station out of VHF range) leaves only the COM1 chip.
fn radio_panel_line(airport: &str, services: &[(String, i64)], com1_khz: Option<i64>) -> String {
    let mut parts = Vec::new();
    if !airport.is_empty() {
        parts.push(airport.to_uppercase());
    }
    let tuned = com1_khz.and_then(|c| services.iter().find(|(_, khz)| (c - khz).abs() <= 5));
    match (tuned, com1_khz) {
        (Some((word, khz)), _) => parts.push(format!(
            ">{} {:.3}",
            service_label(word),
            *khz as f64 / 1000.0
        )),
        (None, Some(c)) => parts.push(format!(">COM1 {:.3}", c as f64 / 1000.0)),
        (None, None) => {}
    }
    parts.join(" | ")
}

/// The ATC tab's payload for the in-sim panel: every in-range field as
/// "NAME,dist_nm,SVC khz,...", fields joined with '|', '*' prefixing the
/// active station. Frequencies stay integer kHz — the plugin compares them
/// straight against COM1 for the tuned highlight and writes them back on a
/// click-to-tune. Trailing fields drop rather than overflow the plugin's
/// 500-byte text buffer.
fn atc_panel_payload(fields: &[station::RangedField]) -> String {
    let mut out = String::new();
    for f in fields {
        let mut seg = String::new();
        if f.active {
            seg.push('*');
        }
        // The separators are structural — scrub them from free-text names.
        let name = f.name.replace(['|', ','], " ");
        seg.push_str(&format!(
            "{},{}",
            name.trim().to_uppercase(),
            f.dist_nm.round() as i64
        ));
        for (word, khz) in &f.services {
            seg.push_str(&format!(",{} {khz}", service_label(word)));
        }
        if out.len() + seg.len() + 1 > plugin_status::TEXT_MAX {
            break;
        }
        if !out.is_empty() {
            out.push('|');
        }
        out.push_str(&seg);
    }
    out
}

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
    use super::{
        classify_confidence, is_abort, is_affirmative, is_undo, join_and, radio_panel_line,
        undoable_drefs, ConfidenceAction, Ptt,
    };

    #[test]
    fn confidence_bands_act_confirm_sayagain_ignore() {
        use ConfidenceAction::*;
        let (min, floor) = (0.35, 0.22);
        // Above the act threshold: always act, grammar/deliberate irrelevant.
        assert_eq!(classify_confidence(0.40, min, floor, true, true), Act);
        assert_eq!(classify_confidence(0.40, min, floor, false, false), Act);
        // Marginal band + deliberate + grammar recognises it → confirm.
        assert_eq!(classify_confidence(0.30, min, floor, true, true), Confirm);
        // Marginal but only the LLM could resolve it → don't guess, say again.
        assert_eq!(classify_confidence(0.30, min, floor, true, false), SayAgain);
        // Below the floor, even a recognised command is too garbled → say again.
        assert_eq!(classify_confidence(0.15, min, floor, true, true), SayAgain);
        // Not deliberate (continuous ambient / crew off): ignore, never speak.
        assert_eq!(classify_confidence(0.30, min, floor, false, true), Ignore);
        assert_eq!(classify_confidence(0.15, min, floor, false, false), Ignore);
        // Floor is inclusive; just under it is not a confirm.
        assert_eq!(classify_confidence(floor, min, floor, true, true), Confirm);
        assert_eq!(
            classify_confidence(floor - 0.01, min, floor, true, true),
            SayAgain
        );
    }

    #[test]
    fn ptt_routes_intercom_and_radio_to_separate_channels() {
        use std::sync::Arc;
        let intercom_dref = "xcopilot/ptt_active";
        let radio_dref = "xcopilot/radio_ptt_active";
        let mk = |icom: f64, radio: f64| Ptt::Dataref {
            live: Arc::new(crate::livestate::LiveState::from_pairs(&[
                (intercom_dref, icom),
                (radio_dref, radio),
            ])),
            name: intercom_dref.to_string(),
            radio: radio_dref.to_string(),
        };
        // Intercom key alone: talking to the copilot, NOT the radio.
        let p = mk(1.0, 0.0);
        assert!(p.intercom_pressed() && !p.radio_pressed());
        // Radio key alone: transmitting, NOT the intercom.
        let p = mk(0.0, 1.0);
        assert!(p.radio_pressed() && !p.intercom_pressed());
        // Neither.
        let p = mk(0.0, 0.0);
        assert!(!p.intercom_pressed() && !p.radio_pressed() && !p.is_pressed());
        // Either one starts a capture (is_pressed for the mic/routing).
        assert!(mk(1.0, 0.0).is_pressed() && mk(0.0, 1.0).is_pressed());
        // No radio dataref bound → radio is never pressed; intercom still works.
        let p = Ptt::Dataref {
            live: Arc::new(crate::livestate::LiveState::from_pairs(&[(
                intercom_dref,
                1.0,
            )])),
            name: intercom_dref.to_string(),
            radio: String::new(),
        };
        assert!(p.intercom_pressed() && !p.radio_pressed());
        // Channel completeness: with a radio key BOUND (regardless of pressed
        // state), physical routing is complete and intercom speech must never
        // content-route to ATC; without one, content routing is the only path
        // to the tower and stays. A keyboard PTT is never a radio.
        assert!(
            mk(0.0, 0.0).has_radio_channel(),
            "bound radio key, even unpressed"
        );
        assert!(!p.has_radio_channel(), "no radio dataref bound");
        assert!(!Ptt::Key(device_query::Keycode::F13).has_radio_channel());
    }

    #[test]
    fn radio_row_shows_the_station_and_the_tuned_service_only() {
        let services = vec![
            ("tower".to_string(), 131200_i64),
            ("approach".to_string(), 122800),
            ("departure".to_string(), 130400),
        ];
        // Tuned to the tower (8.33 name can sit 5 kHz off the carrier):
        // station + that ONE chip — the full plan is the ATC tab's job.
        let line = radio_panel_line("Gibraltar", &services, Some(131205));
        assert_eq!(line, "GIBRALTAR | >TWR 131.200");
        // Off every service: the raw COM1 value as the second chip.
        let line = radio_panel_line("Gibraltar", &services, Some(124730));
        assert_eq!(line, "GIBRALTAR | >COM1 124.730");
        // COM1 unreadable: just the station.
        assert_eq!(radio_panel_line("Gibraltar", &services, None), "GIBRALTAR");
        // Active station out of VHF range (empty airport): only live COM1.
        let line = radio_panel_line("", &[], Some(123500));
        assert_eq!(line, ">COM1 123.500");
        assert_eq!(radio_panel_line("", &[], None), "");
    }

    #[test]
    fn atc_payload_marks_active_and_fits_the_plugin_buffer() {
        let f = |name: &str, dist: f64, active: bool| crate::station::RangedField {
            name: name.to_string(),
            dist_nm: dist,
            active,
            services: vec![
                ("tower".to_string(), 118325_i64),
                ("atis".to_string(), 121800),
            ],
        };
        let payload =
            super::atc_panel_payload(&[f("Badajoz", 21.7, false), f("Gibraltar", 2.1, true)]);
        assert_eq!(
            payload,
            "BADAJOZ,22,TWR 118325,ATIS 121800|*GIBRALTAR,2,TWR 118325,ATIS 121800"
        );
        // Separators are structural — a name containing them can't corrupt
        // neighbouring fields.
        let payload = super::atc_panel_payload(&[f("El Molinillo, Almendralejo|x", 3.0, false)]);
        assert!(
            payload.starts_with("EL MOLINILLO  ALMENDRALEJO X,3,"),
            "{payload}"
        );
        // Too many fields: trailing ones drop instead of overflowing the
        // plugin's 500-byte text buffer.
        let many: Vec<_> = (0..40)
            .map(|i| f(&format!("Field Number {i:02}"), i as f64, false))
            .collect();
        let payload = super::atc_panel_payload(&many);
        assert!(payload.len() <= crate::plugin_status::TEXT_MAX);
        assert!(payload.contains("FIELD NUMBER 00"));
    }

    #[test]
    fn undo_recognizes_emergency_phrasings() {
        for w in [
            "belay that",
            "undo",
            "undo that",
            "abort",
            "scratch that",
            "as you were",
            "disregard",
            "revert",
            "reverse that",
            "take it back",
            "cancel that",
            "abort. didn't catch a command there. abort.",
            "a boat",
        ] {
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
        assert_eq!(
            super::reference_limits(&p, "what's best glide").as_deref(),
            Some("best glide 68 knots.")
        );
        assert_eq!(
            super::reference_limits(&p, "rotation speed").as_deref(),
            Some("rotate at 55 knots.")
        );
        assert!(super::reference_limits(&p, "what's vy")
            .unwrap()
            .contains("74 knots"));
        // Vfe is derived from the flap detents, grouped by speed.
        assert_eq!(
            super::reference_limits(&p, "flap speed").as_deref(),
            Some("Vfe 110 at ten degrees, 85 at twenty degrees and full.")
        );
        // A non-limits utterance returns nothing.
        assert!(super::reference_limits(&p, "set flaps ten").is_none());

        // The SR22 carries its own card — different numbers, same machinery.
        let sr = super::aircraft::load("cirrus_sr22").expect("sr22 profile");
        assert_eq!(
            super::reference_limits(&sr, "best glide").as_deref(),
            Some("best glide 88 knots.")
        );
        assert!(super::reference_limits(&sr, "never exceed")
            .unwrap()
            .contains("204"));
        assert!(super::reference_limits(&sr, "max crosswind")
            .unwrap()
            .contains("21"));
    }

    #[test]
    fn citation_x_loads_with_the_poms_own_figures() {
        let p = super::aircraft::load("citation_x").expect("citation_x profile loads");
        assert_eq!(super::icao_to_profile("C750"), "citation_x");
        assert!(p.autopilot && p.throttle && p.retractable_gear);
        assert!(!p.engine, "a jet declares no piston engine group");
        // POM p.80: Vr 145, Vref 132, Vs1 clean 136, Vle 210. Vmo is altitude
        // dependent (270 low / 350 high) and the low figure is the one flown.
        let r = p.reference.as_ref().expect("reference figures");
        assert_eq!(r.vr_kts, Some(145.0));
        assert_eq!(r.approach_kts, Some(132.0));
        assert_eq!(p.vs_clean_kts, Some(136.0));
        assert_eq!(p.vlo_kts, Some(210.0));
        assert_eq!(p.vne_kts, Some(270.0));
        // The POM's Vy of 300 KIAS is a cruise climb, and a 300 kt circuit is
        // not a circuit. The profile carries a departure speed, and the
        // pattern is capped against the approach speed regardless.
        let vy = r.vy_kts.expect("vy");
        assert!(vy <= 200.0, "vy is a climb-out speed, got {vy}");
        let pat = super::circuit::pattern_speed_kts(vy, r.approach_kts.unwrap());
        assert!(pat <= 198.0, "pattern capped at 1.5x approach, got {pat}");
        // The runway it demands must fit the runways it flies from.
        let roll_m = r.takeoff_roll_ft.expect("ground roll") * 0.3048;
        assert!(
            roll_m * 1.4 < 2162.0,
            "must fit Luton: {:.0} m",
            roll_m * 1.4
        );
    }

    /// The SF50 is a jet whose xlua is cosmetic, so the whole profile stands on
    /// STOCK datarefs — the one custom thing is the start, which is two button
    /// presses in a fixed order rather than a crank.
    #[test]
    fn sf50_is_a_stock_systems_jet_with_a_two_press_start() {
        let p = super::aircraft::load("cirrus_vision_sf50").expect("sf50 profile loads");
        assert_eq!(super::icao_to_profile("SF50"), "cirrus_vision_sf50");
        assert!(p.autopilot && p.throttle && p.retractable_gear);
        assert!(
            !p.engine && !p.magnetos && !p.carb_heat,
            "a jet declares no piston systems"
        );
        // No autothrottle on this model, so speed must be the throttle
        // governor's — declaring one withholds `hold_speed` entirely, which is
        // how "reduce speed" became a no-op on the Citation X (2026-07-28).
        let reg = super::tools::Registry::for_aircraft(&p);
        assert!(reg.get("hold_speed").is_some(), "speed is the governor's");
        assert!(reg.get("set_speed").is_none());
        assert!(reg.get("autothrottle").is_none());
        // The ground drives exist on a jet too (the same 2026-07-28 gate).
        for t in ["auto_taxi", "line_up", "taxi_to_parking"] {
            assert!(reg.get(t).is_some(), "a jet needs {t}");
        }
        // Turbine start, not the piston one-shot.
        assert!(reg.get("start_engines").is_some());
        assert!(reg.get("start_engine").is_none());
        let s = p.turboprop_start.as_ref().expect("a start sequence");
        // No APU anywhere on this airframe: nothing to bring up before cranking.
        assert!(s.apu.is_none(), "the SF50 has no APU");
        // The ignition knob is a PREREQUISITE, not a nicety: the START button's
        // handler only latches when knob_state == 1, so pressing it with the
        // knob at OFF silently sets button_state to 0 instead.
        assert_eq!(
            s.ignition_arm.first().map(String::as_str),
            Some("laminar/SF50/ignition_up")
        );
        assert_eq!(
            s.start_up.first().map(String::as_str),
            Some("laminar/SF50/eng_start_stop_toggle")
        );
        // Momentary presses, both of them — nothing to hold and nothing to
        // release (the Citation X's starter is the opposite case).
        assert!(!s.starter_held, "the SF50's controls are momentary presses");
        assert!(s.start_dn.is_empty());
        // Securing the aircraft is the KNOB, not the mixture: `ignition_down`
        // is what drops the boost pump and igniter and fires the sim's own
        // shutdown. Cutting fuel alone leaves both live with the knob at RUN.
        assert_eq!(
            s.shutdown_commands,
            vec!["laminar/SF50/ignition_down".to_string()]
        );
        // ...and every airframe that has no such control keeps the old
        // lever-only shutdown untouched.
        for id in [
            "dhc6_twin_otter",
            "king_air_c90b",
            "lancair_evolution",
            "citation_x",
        ] {
            let t = crate::aircraft::load(id)
                .and_then(|a| a.turboprop_start)
                .unwrap_or_else(|| panic!("{id} start"));
            assert!(t.shutdown_commands.is_empty(), "{id} secures on the levers");
        }
        // A turbofan's idle N1 is nowhere near the PT6 default of 50, and the
        // script's own figure (N2 > 54, where it cuts its starter) says nothing
        // about N1 — too high a threshold waits for a number that never comes.
        assert!(s.n1_idle < 30.0, "{}", s.n1_idle);
        // The stall band is set so the flap-adjusted stall lands on the real
        // Vso of 67, clear of the POM's own 87 kt landing speed: the .acf's 75
        // warns at the stall itself, and the POM's Vs1 of 86 cautions on every
        // approach ever flown.
        let r = p.reference.as_ref().expect("reference figures");
        let vs_clean = p.vs_clean_kts.expect("clean stall");
        let full_flap_vs = vs_clean * (1.0 - 0.17);
        let approach = r.approach_kts.expect("approach speed");
        assert!(
            1.20 * full_flap_vs < approach,
            "the caution band must sit below the approach speed: {:.0} vs {approach}",
            1.20 * full_flap_vs
        );
        assert!(
            1.10 * full_flap_vs > r.vs0_kts.expect("vs0"),
            "the warning must come BEFORE the stall"
        );
        // Vy carries a climb-out speed, not the POM's 160 kt cruise climb.
        let vy = r.vy_kts.expect("vy");
        assert!(vy <= 140.0, "vy is a climb-out speed, got {vy}");
        // Three flap positions, takeoff at the middle one, descending Vfe
        // ladder. The labels are DEGREES (the EIS gauge is scaled UP/10/35 and
        // the checklist says "FLAPS - SET (10 Degrees)"), not the lever's
        // 50%/100% - both name the same three positions.
        let f = p.flaps.as_ref().expect("flaps");
        assert_eq!(f.detents.len(), 3);
        assert_eq!(f.takeoff_ratio, Some(0.5));
        assert_eq!(f.detents[1].label, "10");
        assert_eq!(f.detents[2].label, "35");
        assert!(f.detents[2].vfe_kts < f.detents[1].vfe_kts);
        // The gear limit is the COCKPIT PLACARD ("Vlo ret 160 KIAS", legible in
        // the POM's p.50 photograph), not the p.61 table — which gives Vlo 170
        // against a Vle of 150, an extended limit below the operating limit,
        // which no aircraft is placarded with. The placard's other half (Vle
        // 210) matches the .acf exactly, which is what makes the reading safe.
        assert_eq!(p.vlo_kts, Some(160.0));
        // Three exterior lights and no taxi light: the cockpit object's
        // complete manipulator list is nav, landing and strobe, whatever the
        // POM's before-taxi and parking checklists call for.
        let keys: Vec<&str> = p.lights.iter().map(|l| l.key.as_str()).collect();
        assert_eq!(keys, vec!["nav", "strobe", "landing"]);
        // The checklist is transcribed from the POM's image-only pages, and
        // parses into runnable lists rather than a wall of call-outs.
        let text = std::fs::read_to_string(p.checklist.as_ref().expect("a checklist"))
            .expect("the checklist file exists");
        let lists = super::checklist::parse(&text, &p);
        assert!(
            lists.len() >= 8,
            "one list per POM section: {}",
            lists.len()
        );
        // A checklist whose every line is a call-out is a transcription, not a
        // copilot. The map must actually connect: the lights, gear, flaps,
        // brake, transponder and electrical items are ours to perform.
        use super::checklist::Item;
        let acted = lists
            .iter()
            .flat_map(|c| &c.items)
            .filter(|i| matches!(i, Item::Act { .. }))
            .count();
        assert!(
            acted >= 12,
            "the map must connect: only {acted} acted items"
        );
        // ...and the "AS REQUIRED" / "CHECK" rows must stay observe-only —
        // writing a probe-heat item the POM leaves to the pilot is not ours.
        let forced = lists.iter().flat_map(|c| &c.items).any(|i| {
            matches!(i, Item::Verify { say, settable, .. }
                     if *settable && say.to_ascii_lowercase().contains("as required"))
        });
        assert!(!forced, "an 'as required' item must never be auto-set");
        // ...and no ITEM may call for a taxi light, which this aircraft has not
        // got — the two POM rows are dropped on purpose. (The `#` header
        // explains the omission, so only real item lines are checked.)
        assert!(
            !text
                .lines()
                .filter(|l| l.trim_start().starts_with("sw_item"))
                .any(|l| l.to_ascii_lowercase().contains("taxi light")),
            "the taxi-light rows must stay dropped"
        );
        // The fuel selector has no OFF position (POM p.29: AUTO/LEFT/RIGHT).
        let fs = p.fuel_selector.as_ref().expect("fuel selector");
        assert!(fs.positions.iter().all(|q| q.key != "off"));
        assert_eq!(fs.positions.len(), 3);
    }

    #[test]
    fn md82_is_a_jet_with_gear_no_piston_tools() {
        let p = super::aircraft::load("md82").expect("md82 profile loads");
        assert_eq!(p.id, "md82");
        assert!(p.autopilot && p.throttle && p.retractable_gear);
        assert!(
            !p.engine && !p.magnetos && !p.carb_heat,
            "jet has no piston systems"
        );
        assert_eq!(p.vne_kts, Some(340.0)); // Vmo
        let reg = super::tools::Registry::for_aircraft(&p);
        assert!(reg.get("gear").is_some(), "retractable gear -> gear tool");
        assert!(
            reg.get("set_throttle").is_some(),
            "throttle flag -> thrust levers"
        );
        assert!(reg.get("set_mixture").is_none(), "no mixture on a jet");
        assert!(
            reg.get("start_engine").is_none(),
            "no piston start on a jet"
        );
        assert!(reg.get("set_flaps").is_some());
        // Electrical is wired (battery, generators, avionics, "all"), so "all
        // electrical systems on" has a tool to land on.
        assert!(reg.get("set_electrical").is_some());
        // "All electrical systems on" is the procedural power_up, not a one-shot key.
        assert!(reg.get("power_up").is_some());
        // ...but only the ON direction remaps: "electrics off" must stay
        // set_electrical (the remap once powered the aircraft back up on a
        // shutdown request).
        use serde_json::json;
        assert!(super::wants_procedural_power_up(
            "set_electrical",
            &json!({"system":"all","on":true})
        ));
        assert!(super::wants_procedural_power_up(
            "set_electrical",
            &json!({"system":"all"})
        ));
        assert!(!super::wants_procedural_power_up(
            "set_electrical",
            &json!({"system":"all","on":false})
        ));
        assert!(!super::wants_procedural_power_up(
            "set_electrical",
            &json!({"system":"battery","on":true})
        ));
        // Jet start sequence, not the piston one-shot.
        assert!(reg.get("start_engines").is_some());
        assert!(reg.get("start_engine").is_none());
        // Pitot heat goes through the overhead heat knob (the stock command is a
        // no-op on this airframe), and "speed for landing" has a figure to use.
        assert_eq!(
            p.pitot_heat_dataref.as_deref(),
            Some("laminar/md82/ice/heatknob")
        );
        assert_eq!(
            p.reference.as_ref().and_then(|r| r.approach_kts),
            Some(140.0)
        );
        // The runway the automation demands must fit runways this aircraft
        // actually operates from. The recited 7,450 ft is a FACTORED FIELD
        // LENGTH at max weight; read as a ground roll it earned a x1.4 on top
        // and asked for 3,179 m — more than the whole of Luton's 2,162 m 25,
        // so the line-up backtracked for length it did not need and full auto
        // then failed its own lined-up test twice (2026-07-28).
        let roll_m = p
            .reference
            .as_ref()
            .and_then(|r| r.takeoff_roll_ft.or(r.takeoff_distance_ft))
            .expect("md82 declares a takeoff roll")
            * 0.3048;
        assert!(
            roll_m * 1.4 < 2162.0,
            "an MD-82 must be able to depart Luton: needs {:.0} m of 2162",
            roll_m * 1.4
        );
        // ...and the recited figure is still the POM one, unchanged.
        assert_eq!(
            p.reference.as_ref().and_then(|r| r.takeoff_distance_ft),
            Some(7450.0)
        );
        // This model has no autothrottle, whatever the real aircraft has: the
        // speed tool must be the throttle governor, not an AP bug with nothing
        // behind it. Declaring one also withheld `hold_speed` entirely, so
        // "reduce speed" did nothing at all (2026-07-28).
        let cit_reg = super::tools::Registry::for_aircraft(
            &super::aircraft::load("citation_x").expect("citation_x loads"),
        );
        assert!(
            cit_reg.get("hold_speed").is_some(),
            "speed is the governor's"
        );
        assert!(cit_reg.get("set_speed").is_none());
        assert!(cit_reg.get("autothrottle").is_none());
        // A toggle-only system must not be skipped by the bulk power-up: the
        // Citation's avionics master has no ON command, so "all electrical on"
        // fired nothing for it and full auto left the radios dead while
        // announcing the electrical was set (2026-07-28).
        let cit = super::aircraft::load("citation_x").expect("citation_x loads");
        let toggles = super::tools::all_electrical_toggles(&cit.electrical);
        assert!(
            toggles.iter().any(|t| t.dataref.contains("avionics")),
            "the bulk power-up must reach the avionics master"
        );
        // The ground drives exist on a jet too. They were gated behind
        // `!autothrottle` alongside the airborne speed tool, so neither jet had
        // a taxi, line-up or park tool and "taxi for me" reached nothing —
        // only full auto worked, because it builds its own (2026-07-28).
        for t in ["auto_taxi", "line_up", "taxi_to_parking"] {
            assert!(reg.get(t).is_some(), "a jet needs {t}");
        }
        // ...but the airborne speed tool is still the AP's bug on an airliner.
        assert!(
            reg.get("hold_speed").is_none(),
            "an autothrottle airframe sets the speed bug, not a throttle hold"
        );
        assert!(reg.get("set_speed").is_some());
        // A jet taxis like a jet: the GA 10/6/4 is a brake setting on this
        // airframe, which rolls past 10 knots at idle thrust on its own.
        let pace = super::autotaxi::Pace::from_profile(p.taxi.as_ref());
        assert!(
            pace.straight > 15.0 && pace.turn >= 8.0,
            "MD-82 must taxi briskly, got {pace:?}"
        );
        // ...and the piston 172 is the other way round.
        let c172 =
            super::tools::Registry::for_aircraft(&super::aircraft::load("cessna_172sp").unwrap());
        assert!(c172.get("start_engines").is_none());
        assert!(c172.get("start_engine").is_some());
        // Reference card answers from the POM.
        assert!(super::reference_limits(&p, "service ceiling")
            .unwrap()
            .contains("37,000"));
        // "What are we in" describes it as a twin-engine jet, not single-engine.
        let info = super::aircraft_info(&p, "what aircraft are we in").unwrap();
        assert!(info.contains("twin-engine jet"), "got: {info}");
        assert!(!info.contains("single-engine"));
        // The MD-82 checklist parses.
        assert!(crate::checklist::load(&p).is_ok());
    }

    #[test]
    fn join_and_reads_naturally() {
        assert_eq!(join_and(&["flaps".into()]), "flaps");
        assert_eq!(
            join_and(&["flaps".into(), "throttle".into()]),
            "flaps and throttle"
        );
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
            "flaps ten",
            "gear up",
            "landing light on",
            "strobes off",
            "autopilot heading 270",
            "climb and maintain 7000",
            "descend at 500",
            "throttle 60 percent",
            "mixture full rich",
            "start the engine",
            "battery on",
            "squawk 1200",
            "align the gyro",
            "pitot heat on",
            "fuel pump on",
            "carb heat off",
            "magnetos both",
            "trim nose up",
            "nav one 109.5",
            "set the OBS to 240",
            "com two 121.9",
            "transponder standby",
            "prepare the ILS for runway 13",
            "flood lights on",
            // Phrasings that previously fell through the gate (issue: rejected
            // before reaching the LLM).
            "electrics off",
            "fix the gyro drift",
            "master switch on",
            "generator on",
            "squawk VFR",
            // Verbatim whisper output from logs/ that the gate used to reject —
            // recovered by the ignition/ascend/speed keywords.
            "Ignition on.",
            "Ascend 500 feet per minute.",
            "Physical speed 500.",
            // Whisper glued "flood lights" into one token; the compound and its
            // "blood" mishearing must still reach the LLM.
            "Floodlights on.",
            "Turn on the floodlights.",
            "Increase floodlights.",
            "Bloodlights on.",
            "landinglight on",
            // Phase-config macro invocations must reach the LLM.
            "configure for takeoff",
            "set up for takeoff",
            "takeoff config",
            "get ready for takeoff",
            "prep for departure",
            "configure for landing",
            "get ready for landing",
            "after landing",
            "clean up",
            "clear of the runway",
            "secure the aircraft",
            "secure",
            "full shutdown",
            // "checklist" carries any phase past the gate, even hyphenated STT.
            "Normal take-off checklist.",
            "starting engine checklist",
            "preflight checklist",
            // The bare rollout call — silently gated on the Twotter's first
            // reverse flight ("thrusters" is too far from "thrust" for fuzzy).
            "Reverse thrusters.",
            "reversers",
            "stow reverse",
            "cancel reverse",
            // Split compounds — swallowed with no miss chime, live 2026-07-20.
            "Take off for me.",
            "Let's take off.",
            "You have to take off.",
            "Line us up.",
            "line me up",
            "Get us lined up.",
            // Auto circuit — the whole pattern. "Do a circuit" was dropped as
            // not-a-command (circuit vocabulary missing from the gate), live
            // 2026-07-23.
            "Do a circuit.",
            "fly a circuit",
            "fly the pattern",
            "circuits and bumps",
            "make a right circuit",
            "take off and land",
        ] {
            assert!(looks_like_command(c), "{c:?} should pass the domain gate");
        }
    }

    #[test]
    fn circuit_command_survives_every_gate_for_the_172() {
        // The regression shipped 2026-07-23: "do a circuit" was GREEN in the
        // grammar unit test yet dead in the running app, because two main-loop
        // gates UPSTREAM of the parser rejected it first — the domain gate
        // (circuit not a keyword) and the question guard ("do" was a blanket
        // opener). Unit-testing the parser in isolation missed both. This runs
        // the WHOLE gate sequence a spoken command actually passes through, on
        // the real 172 registry, so the next such gap fails here.
        let prof = crate::aircraft::load("cessna_172sp").expect("172 profile");
        let reg = crate::tools::Registry::for_aircraft(&prof);
        assert!(
            reg.get("circuit").is_some(),
            "the 172 registers the circuit tool"
        );
        assert!(
            reg.get("full_auto").is_some(),
            "the 172 registers the full_auto tool"
        );
        // "do a circuit" is the whole flight to the pilot, so it routes to the
        // full-auto orchestrator (which flies the airborne circuit itself once
        // on the runway).
        for cmd in [
            "do a circuit",
            "fly a circuit",
            "take off and land",
            "circuits and bumps",
            "make a right circuit",
        ] {
            assert!(looks_like_command(cmd), "{cmd:?} must pass the domain gate");
            assert!(
                !super::looks_like_question(cmd),
                "{cmd:?} must not be withheld as a question"
            );
            let parsed = super::grammar::try_parse(cmd, &reg, &prof)
                .unwrap_or_else(|| panic!("{cmd:?} must parse to a command"));
            assert_eq!(
                parsed[0].name, "full_auto",
                "{cmd:?} must route to full_auto"
            );
        }
    }

    #[test]
    fn release_intent_catches_every_live_garble() {
        // Every phrasing the pilot actually tried while fighting to take the
        // throttle back from the taxi governor — all must release.
        for t in [
            "my throttle",
            "my thruffles",
            "mic control",
            "my throat",
            "release immediately",
            "let me go. stop touching the throttle",
            "give me command",
            "give me command now",
            "my power",
            "stop holding speed",
            "i have control",
            "im taking control",
        ] {
            assert!(super::release_intent(t), "{t:?} must release");
        }
        // Ordinary commands must not trip it (it only runs while a hold is
        // active, but even then these should pass through).
        for t in [
            "taxi at 10 knots",
            "flaps ten",
            "hold 70 knots",
            "landing light on",
        ] {
            assert!(!super::release_intent(t), "{t:?} must not release");
        }
    }

    use super::correction_value;
    use super::looks_like_followup;

    #[test]
    fn corrections_yield_the_bare_value() {
        // The live case: "Power 8 percent" executed, then the fix dead-ended.
        assert_eq!(
            correction_value("I said 80 percent."),
            Some("80 percent".into())
        );
        assert_eq!(
            correction_value("no, two five zero"),
            Some("two five zero".into())
        );
        assert_eq!(correction_value("No, I said forty"), Some("forty".into()));
        assert_eq!(
            correction_value("make it sixty percent"),
            Some("sixty percent".into())
        );
        assert_eq!(
            correction_value("correction, one thousand five hundred"),
            Some("one thousand five hundred".into())
        );
        assert_eq!(
            correction_value("How about eighty percent?"),
            Some("eighty percent".into())
        );
        // Whisper dropping the "how" (heard live on the Twotter flight).
        assert_eq!(
            correction_value("About 40 percent?"),
            Some("40 percent".into())
        );
        assert_eq!(correction_value("about we start the descent"), None);
        // "how about" with a full command is a question for the copilot, not a
        // value correction.
        assert_eq!(correction_value("how about we start the descent"), None);
        // Full commands and non-values are NOT corrections — they take the
        // normal path ("no, flaps up" must not become "throttle flaps up").
        assert_eq!(correction_value("no, flaps up"), None);
        assert_eq!(correction_value("no"), None);
        assert_eq!(correction_value("I said start the engines"), None);
        assert_eq!(correction_value("north"), None, "no prefix, no correction");
        assert_eq!(correction_value("flaps ten"), None);
    }

    #[test]
    fn followups_are_recognized_but_plain_chatter_is_not() {
        for c in [
            "switch it back to both",
            "do that again",
            "a bit more",
            "undo that",
            "the other tank",
            "seventy percent",
            "60%",
            "two thousand",
        ] {
            assert!(looks_like_followup(c), "{c:?} should look like a follow-up");
        }
        for c in [
            "nice weather today",
            "where are we",
            "hello copilot",
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
        assert_eq!(
            format!(
                "{} {}",
                command_subject("throttle eighty percent"),
                "sixty percent"
            ),
            "throttle sixty percent"
        );
    }

    #[test]
    fn location_answers_from_the_profile() {
        use super::location_answer;
        let p = crate::aircraft::load("king_air_c90b").expect("c90b");
        let a = location_answer("Where is the inverter?", &p).expect("known location");
        assert!(a.to_lowercase().contains("subpanel"), "{a}");
        // Aliased phrasing + longest-key preference.
        assert!(super::location_answer("where are the boost pumps", &p).is_some());
        // Unknown items and non-questions fall through.
        assert!(super::location_answer("where is the flux capacitor", &p).is_none());
        assert!(super::location_answer("inverter on", &p).is_none());
    }

    #[test]
    fn glued_light_compound_does_not_overmatch() {
        // Stripping a light/lights suffix must not wave through non-cockpit words.
        for g in [
            "moonlight",
            "highlight",
            "delight",
            "spotlight",
            "flashlight",
        ] {
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
    fn alias_folded_commands_pass_the_gate() {
        // The normalizer's alias table is part of the gate now: "q and h
        // 1014" (four QNH settings swallowed live, 2026-07-20) only becomes
        // the "qnh" keyword after folding, and the brake homophone rides
        // its own keywords.
        for c in [
            "Q and H 1014.",
            "Change Q and H to 1014.",
            "Release breaks.",
            "Q&H 1013",
        ] {
            assert!(looks_like_command(c), "{c:?} should pass the domain gate");
        }
    }

    #[test]
    fn phonetic_garbage_is_rejected() {
        for g in [
            "Nautilapuku",
            "Mokuli ke maluku",
            "Reki ki panaki",
            "Tiki Maliki, WaKiPi",
            "Take a look at the video",
            "",
        ] {
            assert!(!looks_like_command(g), "{g:?} should be rejected");
        }
    }

    #[test]
    fn affirmatives_proceed() {
        for r in [
            "affirm",
            "affirmative",
            "yes",
            "confirm",
            "roger that",
            "correct",
            "go ahead",
        ] {
            assert!(is_affirmative(r), "{r:?} should confirm");
        }
    }

    #[test]
    fn negatives_and_ambiguity_cancel() {
        // explicit no, and anything not clearly yes, must NOT proceed
        for r in [
            "negative",
            "no",
            "cancel",
            "stop",
            "abort",
            "uh what",
            "descend 700",
            "",
        ] {
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

    #[test]
    fn battery_low_is_system_class_aware() {
        use super::battery_low;
        // 24V-class: the flat 20.5V we saw live is low; 27.5V charged is not.
        assert!(battery_low(20.5));
        assert!(!battery_low(27.5));
        assert!(!battery_low(24.5));
        // 12V-class must not be flagged low just for being under 23.
        assert!(!battery_low(12.6));
        assert!(!battery_low(13.8));
        assert!(battery_low(10.9));
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
        // Selector commands without "tank"/"selector" said (seen live: answered
        // with the quantity report five times in one session).
        assert!(!is_fuel_query("switch fuel to right"));
        assert!(!is_fuel_query("set fuel to left"));
        assert!(!is_fuel_query("switch fuel"));
        assert!(!is_fuel_query("switch tanks"));
        // Quantity questions that mention a side or a tank stay questions.
        assert!(is_fuel_query("how much fuel in the left tank"));
        assert!(is_fuel_query("how much fuel do we have left"));
    }

    #[test]
    fn power_loss_query_detects_diagnosis_not_commands() {
        use super::is_power_loss_query;
        // The live unrecognized phrasings, plus common variants.
        assert!(is_power_loss_query("why we lost power?"));
        assert!(is_power_loss_query("why have we lost power?"));
        assert!(is_power_loss_query("we're losing power"));
        assert!(is_power_loss_query("the engine quit"));
        assert!(is_power_loss_query("engine failure"));
        assert!(is_power_loss_query("why did the engine stopped")); // STT grammar drift
        assert!(is_power_loss_query("engine cut out"));
        // Commands and unrelated mentions must not be hijacked.
        assert!(!is_power_loss_query("reduce power by ten percent"));
        assert!(!is_power_loss_query(
            "reduce power, we lost too much altitude"
        ));
        assert!(!is_power_loss_query("stop the engine"));
        assert!(!is_power_loss_query("start the engine"));
        assert!(!is_power_loss_query("full power"));
    }

    #[test]
    fn whisper_label_strips_path_prefix_and_suffix() {
        use super::whisper_model_label;
        assert_eq!(whisper_model_label("models/ggml-base.en.bin"), "base.en");
        assert_eq!(
            whisper_model_label("/a/b/ggml-large-v3-turbo.bin"),
            "large-v3-turbo"
        );
        assert_eq!(whisper_model_label("small.en"), "small.en");
    }

    #[test]
    fn questions_are_detected_commands_are_not() {
        for q in [
            "how much fuel do we have",
            "what's our heading",
            "what is our altitude?",
            "are we configured for landing",
            "how high are we",
            "tell me the airspeed",
            "do we have enough fuel to make it?",
            "where are we",
            // Keyword+number questions: same shape as a command, so they must be
            // caught here and withheld from the grammar fast-path (which would
            // otherwise *act* on them) — see the fast-path gate in run_session.
            "is the heading 270?",
            "did we set flaps to ten?",
            // Contraction openers, in raw and apostrophe-stripped forms: without
            // them "Where's the parking brake?" parsed as a command and SET the
            // brake (seen live). Never an action from a question shape.
            "where's the parking brake",
            "wheres the parking brake",
            "who's flying",
            "when's our descent",
        ] {
            assert!(looks_like_question(q), "{q:?} should be a question");
        }
        for c in [
            "flaps ten",
            "gear up",
            "set heading 270",
            "throttle 60 percent",
            "landing light on",
            "start the engine",
            // do-support imperatives: "do <article/noun>" is a COMMAND, not a
            // question. "Do a circuit" missed live because "do" was a blanket
            // question opener and swallowed the parsed command (2026-07-23).
            "do a circuit",
            "do a touch and go",
            "do a go around",
            "do another circuit",
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
