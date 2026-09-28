//! XVoiceLayer — the voice layer for X-Plane 12: copilot, ATC and cabin crew.
//!
//! The voice loop, end to end and fully local:
//!   listen → whisper (STT) → deterministic command/conversation routing →
//!   write dataref + read back → say/piper (TTS) confirms.
//!
//! Two capture modes (config `listen_mode`): `continuous` (always listening,
//! VAD-segmented) or `ptt` (push-to-talk).

mod aircraft;
mod answers;
mod approach_guide;
mod audio;
mod auto_status;
mod autoland;
mod autotakeoff;
mod autotaxi;
mod cabin;
mod checklist;
mod circuit;
mod cockpit_auto;
mod config;
mod control;
mod crew_pace;
mod conversation;
mod course_leg;
mod debrief;
mod sortie_runner;
mod test_card;
mod detect;
mod dialogue;
mod failures;
#[allow(dead_code)] // shadow run: the consumers move over next
mod flight;
mod duty;
mod handover;
mod flight_state;
mod flightplan;
mod flow;
mod full_auto;
mod grammar;
mod health;
mod heli_circuit;
mod heli_route;
mod hover;
mod instructor;
mod interaction;
mod livestate;
mod mic;
mod mode;
mod monitor;
mod ils;
mod scorecard;
mod persona;
mod phrase;
mod plan;
mod plugin_status;
mod pollers;
mod prefs;
mod plan_audit;
mod procedure;
mod pushback;
mod radiofx;
mod rapport;
mod readback;
mod reversal;
mod route_finder;
mod router;
mod settings;
mod simbrief;
mod stt;
mod stt_eval;
mod systems;
mod takeoff_config;
mod taxi_guide;
mod taxi_in;
mod tempo;
mod terrain;
mod tools;
mod tts;
#[cfg(feature = "onnx-tts")]
mod tts_onnx;
mod ui;
mod undo;
mod vad;
mod voice;
mod weather;

// The world side (ATC) and the simulator plumbing now live in their own
// crates; these shims keep every existing `crate::station::...` /
// `crate::xplane::...` path in the module tree resolving unchanged.
pub(crate) use ::atc::{airspace, atis, callsign, nav, party_line, speaker, station, tower};
pub(crate) use sim::{blackbox, datarefs, xplane};

use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};

use answers::{
    aircraft_info, angle_delta_deg, answer_state_query, config_advice, conversation_read,
    fuel_report, is_endurance_query, is_fuel_query, is_power_loss_query, is_taxi_query,
    landing_readiness, lerp_angle_deg, location_answer, nav_progress, on_course_report,
    power_loss_diagnosis, reference_limits, remote_weather_target, runway_advice, systems_check,
    traffic_report, weather_words, RemoteWeather, HEAD_DREFS, MS_TO_KTS,
    SNAPSHOT_DATAREFS,
};
#[cfg(test)]
use answers::{battery_low, progress_line, speed_assessment_words, speed_verdict, SpeedContext};
use audio::{Listener, PttMic};
use config::Config;
use detect::{pick_profile, read_identity, spawn_aircraft_detector, LiveProfile};
use livestate::LiveState;
#[cfg(test)]
use plan::ident_distance;
use plan::{
    ensure_plan, eta_to_destination, fix_answer, fix_query, flight_plan_report, load_route_to_fms,
    plan_change_line, plan_fetch_for, plan_flight, time_to_tod, wrong_plan_complaint, PlanFetch,
    PLAN_ASK_WINDOW,
};
use pollers::{
    spawn_panel_request_poller, spawn_settings_poller, spawn_text_settings_poller,
    spawn_voice_picker, DR_OPT_SNAP, DR_OPT_TONE,
};
#[cfg(test)]
use router::has_number;
use router::{
    callout_label, classify_confidence, command_subject, conversational, correction_text,
    correction_value, is_affirmative, is_bare_value, looks_like_command, looks_like_followup,
    atc_recall_line, atc_recall_request, eta_question, looks_like_question, recall_ask, recall_line,
    release_intent, ConfidenceAction,
};
use stt::Stt;
use tools::{Registry, ToolCtx};
use ui::Reporter;
use undo::{
    engages_autopilot, is_undo, join_and, recovery_label, undoable_drefs, UndoSnapshot,
    UNDO_STACK_MAX,
};
use voice::{Priority, Voice};
use xplane::XPlaneClient;

// Single-threaded runtime: the cpal stream isn't `Send`, and the loop is
// inherently sequential (listen → think → act → speak), so there's nothing to
// gain from a multi-threaded scheduler.
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    use tracing_subscriber::prelude::*;
    // Every blackbox record through the live scorer: `stage_result` beside
    // the ticks, `sortie_result` at each mission's end.
    blackbox::set_tap(scorecard::tap);
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
    // BOTH names for the crew, and they are not the same thing: a log line's
    // default target is `module_path!()`, which for code in a BINARY is the
    // BIN name (`xvl`) and for code in the lib is the PACKAGE name
    // (`xvoicelayer`). Naming only the package silently dropped all 158
    // default-target sites in this file and its modules — the startup/build
    // line included, so the one documented way to tell which build is flying
    // ("check the build= line") reported nothing, while the explicit
    // `target: "xvoicelayer::..."` lines kept arriving and made the log look
    // healthy (2026-07-30, found by grepping for a startup line that was
    // never there). Renaming the binary must move this default with it.
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| "xvoicelayer=info,xvl=info,atc=info,sim=info".into());
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
        version = env!("XVL_VERSION"),
        build = env!("XCOPILOT_BUILD"),
        profile = env!("XVL_PROFILE"),
        "XVoiceLayer starting"
    );

    // Publish the configuration to the live store and let saved preferences
    // fill in every registry knob the environment is silent about (env still
    // wins — see settings.rs). From here on the boot path uses this resolved
    // snapshot, while the loops re-read `settings::live()` each tick so a
    // change made from the Settings screen applies without a restart.
    let config: Config = (*settings::install(config)).clone();
    // The station's callsign resolver asks the crew for the override and the
    // plan's callsign through this provider, read fresh each time.
    callsign::provide_inputs(circuit::callsign_inputs);
    // ...and the pilot's own airline telephony ("WAT=Walker"), asked before
    // its built-in tables, the same way.
    callsign::provide_telephony(settings::airline_telephony);
    // ...and the route we filed, so an IFR clearance names the SID that
    // joins it, by the crew's own rule (#152).
    party_line::provide_filed_route(full_auto::filed_route);

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

    // `xvl tts ["phrase"] [--voice SPEC] [--pace N] [--radio] [--out FILE.wav]`
    // synthesizes one phrase - an audition tool for voices without launching
    // the loop. Defaults: the copilot's configured voice (honouring the
    // XCOPILOT_TTS_LENGTH/NOISE/NOISE_W scale overrides), played aloud.
    // `--voice` takes any model, `models/x.onnx#12` picking a speaker of a
    // multi-speaker one; `--pace` is the length scale (the stations run 0.8,
    // accented voices 1.1); `--radio` applies the station's transmission
    // effect; `--out` writes a WAV instead of playing, so a sample can be
    // made while the sim is flying without talking over it.
    #[cfg(feature = "onnx-tts")]
    if std::env::args().nth(1).as_deref() == Some("tts") {
        let mut text: Option<String> = None;
        let (mut voice, mut pace, mut radio, mut out) =
            (config.piper_voice.clone(), None, false, None);
        let mut args = std::env::args().skip(2);
        while let Some(a) = args.next() {
            match a.as_str() {
                "--voice" => {
                    voice = args
                        .next()
                        .ok_or_else(|| anyhow!("--voice needs a model path"))?
                }
                "--pace" => {
                    pace = Some(
                        args.next()
                            .ok_or_else(|| anyhow!("--pace needs a number"))?
                            .parse::<f32>()?,
                    )
                }
                "--radio" => radio = true,
                "--out" => {
                    out = Some(
                        args.next()
                            .ok_or_else(|| anyhow!("--out needs a file name"))?,
                    )
                }
                _ => text = Some(a),
            }
        }
        let text = text.unwrap_or_else(|| {
            "Roger, flaps up. Gear down, three green. Caution, fuel endurance forty minutes.".into()
        });
        eprintln!(
            "voice: {voice}  (length={} noise={} noise_w={})",
            pace.map(|p| p.to_string())
                .or_else(|| std::env::var("XCOPILOT_TTS_LENGTH").ok())
                .unwrap_or_else(|| "model".into()),
            std::env::var("XCOPILOT_TTS_NOISE").unwrap_or_else(|_| "model".into()),
            std::env::var("XCOPILOT_TTS_NOISE_W").unwrap_or_else(|_| "model".into())
        );
        let mut v = tts_onnx::OnnxTts::load(&voice)?;
        if pace.is_some() {
            v.override_scales(None, pace, None);
        }
        let mut samples = v.render(&text)?;
        if radio {
            match radiofx::load_tail_wav("assets/afv/Click_f32.wav", v.sample_rate()) {
                Ok(tail) => radiofx::radioize_with_tail(&mut samples, v.sample_rate(), 1, &tail),
                Err(_) => radiofx::radioize_variant(&mut samples, v.sample_rate(), 1),
            }
        }
        let path = match &out {
            Some(p) => std::path::PathBuf::from(p),
            None => std::env::temp_dir().join("xvoicelayer-tts.wav"),
        };
        tts_onnx::write_wav(&path, &samples, v.sample_rate())?;
        match out {
            Some(_) => eprintln!("wrote {}", path.display()),
            None => tts::play(&path)?,
        }
        return Ok(());
    }

    // `xvl debrief [logs/blackbox.jsonl]` — what the last flight did, from the
    // blackbox: the read a pilot gets after the sortie, and the one the
    // fly-fix-fly loop starts from.
    if std::env::args().nth(1).as_deref() == Some("debrief") {
        let path = std::env::args()
            .nth(2)
            .unwrap_or_else(|| "logs/blackbox.jsonl".to_string());
        let text = std::fs::read_to_string(&path).map_err(|e| anyhow!("reading {path}: {e}"))?;
        print!("{}", debrief::report(text.lines()));
        return Ok(());
    }

    // Both processes hold their traffic through a flight load (the plugin's
    // `xcopilot/sim/loading`; failing open without it).
    xplane::enable_load_gate(&config.xplane_base_url);

    // `xvl sortie <spec.yaml> [--repeat N]`: the unattended runner, a second
    // process beside a running xvl (src/sortie_runner.rs). Exit 1 on any fail.
    if std::env::args().nth(1).as_deref() == Some("sortie") {
        let rest: Vec<String> = std::env::args().skip(2).collect();
        let all_passed =
            sortie_runner::main_cmd(&rest, &config.xplane_base_url, &config.apt_data_path).await?;
        std::process::exit(if all_passed { 0 } else { 1 });
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

    // Resolve the aircraft profile: auto-detect from the loaded MODEL (its
    // acf_ICAO and .acf path against every profile's `match:` block, never
    // the ICAO alone) when configured as "auto", otherwise use the configured
    // id. Falls back to the Cessna if detection fails (e.g. X-Plane not
    // reachable yet). The probe client is used and dropped here:
    // `run_session` makes its own (a reqwest client must be used on the
    // runtime it runs on). The swap poller resolves through the same
    // `detect::pick_profile` and says an uncurated airframe aloud.
    let mut startup_profile: Option<aircraft::AircraftProfile> = None;
    let aircraft_id = if config.aircraft.eq_ignore_ascii_case("auto") {
        let probe = XPlaneClient::new(config.xplane_base_url.clone());
        match read_identity(&probe).await {
            Ok(who) => {
                let picked = pick_profile(&who);
                tracing::info!(icao = %who.icao, path = %who.relative_path, profile = %picked.id, "auto-detected aircraft");
                startup_profile = picked.profile;
                picked.id
            }
            Err(e) => {
                tracing::warn!("aircraft auto-detect failed ({e:#}); defaulting to cessna_172sp");
                "cessna_172sp".to_string()
            }
        }
    } else {
        config.aircraft.clone()
    };
    let profile = match startup_profile {
        Some(p) => p,
        None => aircraft::load(&aircraft_id)
            .or_else(|| {
                aircraft_id.starts_with("generic_").then(|| {
                    let mut p = aircraft::load("cessna_172sp")?;
                    p.id = aircraft_id.clone();
                    Some(p)
                })?
            })
            .ok_or_else(|| anyhow!("unknown aircraft profile: {aircraft_id}"))?,
    };

    // Voices through the sim's audio: the boot value from the environment; the
    // panel's Settings switch (xcopilot/opt/audio_bus) turns it on live.
    tts::configure_audio_bus(config.audio_bus, &config.xplane_base_url);
    tracing::info!(
        aircraft = %profile.display_name,
        mode = %config.listen_mode,
        "starting XVoiceLayer"
    );

    // whisper model load is the one slow init — do it once, up front. Bias its
    // decoding toward the active aircraft's actual systems and phraseology.
    let stt = Stt::load(&config.whisper_model_path, config.whisper_gpu)?;
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
            config.cabin_piper_voice.clone(),
        );
    }

    // The panel's Settings-tab toggles (app mode, cabin-crew staffing):
    // publish the current values, apply picks live, persist to prefs.
    spawn_settings_poller(config.xplane_base_url.clone());
    // ...and its text fields (callsign, airline telephony), through the
    // registry: env-pinned ones are shown read-only, edits persist.
    spawn_text_settings_poller(config.xplane_base_url.clone());
    // A new flight another writer starts through the plugin's mailbox (the
    // sortie runner's runway and ramp starts) ends the old one here too:
    // loops stood down, clearances retired (#134).
    tools::setup::spawn_new_flight_watch(config.xplane_base_url.clone());

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
                            let callsign = crate::circuit::crew_callsign(&xp)
                                .await
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

    // Aircraft-change poller: re-read the loaded model (acf_ICAO and its .acf
    // path) periodically and swap the profile when it changes, so the copilot
    // reconfigures itself (tools, limits, monitor) instead of staying on
    // whatever was loaded at startup. Auto-detect only.
    if config.aircraft.eq_ignore_ascii_case("auto") {
        spawn_aircraft_detector(
            live_profile.clone(),
            config.xplane_base_url.clone(),
            voice.clone(),
            status_sink.clone(),
        );
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
        // TUI: the voice loop and the health poll run on background threads,
        // each with its own runtime; the dashboard renders on the main thread.
        let (tx, rx) = std::sync::mpsc::channel::<ui::Report>();
        let boot_facts = boot_health(&config);

        let tx_poll = tx.clone();
        let mut collector = health::Collector::new(
            XPlaneClient::new(config.xplane_base_url.clone()),
            live_profile.clone(),
            &config,
        );
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            else {
                return;
            };
            rt.block_on(async move {
                loop {
                    if tx_poll
                        .send(ui::Report::Health(Box::new(collector.poll().await)))
                        .is_err()
                    {
                        break; // dashboard gone
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

        ui::run_dashboard(boot_facts, rx, cmd_tx)?; // blocks until the user quits
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
    // Applied by `SimRadioSync` in the loop, not once here: the boot write
    // failed silently whenever X-Plane was not up yet, and the audio bus
    // changes what the right answer is (see `tts::sim_radio`).
    let xp_radio_muted = config.mute_xp_radio && !config.atc_airport.trim().is_empty();
    let mut sim_radio_sync = SimRadioSync::default();
    // Subscribe to the continuously-read state over the WebSocket so snapshots are
    // instant cache reads instead of a REST burst. Best-effort: reads fall back to
    // REST until (or unless) the subscription warms up.
    let live = LiveState::start(&config.xplane_base_url, SNAPSHOT_DATAREFS);

    // Profile-derived state, rebuilt whenever the aircraft switches (see the
    // generation check at the top of the loop). Seeded from the live profile.
    let mut active_gen = live_profile.generation();
    let mut profile = live_profile.get();
    let mut registry = Registry::for_aircraft(&profile);
    // The flow module owns the parsed lists (its phase-boundary runner and
    // the panel need them off this loop); this is the same Arc for matching.
    let mut checklists = flow::install(&profile);
    instructor::install(&profile);
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
    // Instructor mode: the full "I don't touch the controls" line once per
    // spell in the mode, a short "your hands" after.
    let mut instructor_noted = false;
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
    let mut history: Vec<(String, String, Instant)> = Vec::new();
    // Subject of the last successful named command ("throttle", "set heading"),
    // used to re-anchor a bare-value follow-up ("sixty percent" → "throttle
    // sixty percent") into an explicit command.
    let mut last_subject: Option<String> = None;
    // Cached SimBrief plan (it doesn't change mid-flight); re-fetched on TTL or "refresh".
    let mut fp_cache: Option<(Instant, simbrief::FlightPlan)> = None;
    // A bare "divert" / "plan a flight" asked "to where?"; the next short
    // utterance inside this window is the answer.
    let mut plan_pending: Option<Instant> = None;
    // Last time the crew read the full comm plan after a dead-air transmission.
    // The 2026-07-08 night flight called Malaga, Jerez and Cadiz in a row and got
    // the whole Gibraltar briefing every time — once is help, thrice is butting in.
    let mut last_void_brief: Option<Instant> = None;
    // Memory of armed reminders, so the copilot can recall and cancel them.
    let reminders = tools::reminder::Reminders::default();
    // Recovery for recent utterances (a stack), so "belay that" can walk back the
    // last command, then the one before it, each with how to undo it.
    let mut undo_stack: Vec<UndoSnapshot> = Vec::new();
    // The dialogue stack: executed calls, so back-references ("a bit more",
    // "turn it off", "do that again") resolve deterministically against what
    // was actually just done.
    let mut dialogue = dialogue::Dialogue::new();
    let mut conversation = conversation::Conversation::default();

    // Barge-in: a watcher thread that, while the copilot is speaking, kills the
    // audio the instant PTT is pressed — so the pilot is never talked over and a
    // long readback yields immediately. Readbacks block the command loop while
    // speaking, so the interrupt can't come from it; it needs its own thread.
    // (A checklist flow runs on its own thread and is stopped by the WORD
    // "stop", not the press: the press is how the pilot answers it.)
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

    let mut capture_tuning = CaptureTuning::of(&config);

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
        // This utterance's settings, read fresh: the confidence bands, the
        // confirm gate, the LLM fallback and the readback challenges all
        // become live changes rather than boot constants. Boot-time values
        // (paths, base URLs, the capture wiring) stay as they were resolved.
        let config = settings::live();
        // Capture tuning IS re-applied, because tuning a mic gate by ear is
        // the reason a settings screen exists: a change lands on the next
        // utterance instead of the next launch.
        capture_tuning.follow(&config, listener.as_mut(), ptt_mic.as_ref());
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
        sim_radio_sync
            .follow(
                &xplane,
                xp_radio_muted && mode::atc_enabled(),
                tts::AUDIO_BUS.load(std::sync::atomic::Ordering::Relaxed),
            )
            .await;
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
                // The radio soundscape is reclaimed by `sim_radio_sync`.
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
                // X-Plane gets its radio voice back through `sim_radio_sync`:
                // the pilot may be switching to the sim's OWN ATC.
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
                atc_only_noted = false;
                voice.alert(Priority::Advisory, "Copilot back with you.".to_string());
            } else if now_mode == mode::Mode::AtcOnly && applied_mode != mode::Mode::AtcOnly {
                // Leaving the crew behind: cut any copilot speech and evict
                // queued crew lines too. Radio traffic stays on the air.
                voice.dismiss_crew();
            }
            // The instructor's edges: hands come off everything that is
            // running (the airframe-swap stand-down, since a lesson and a
            // loop cannot share the aircraft), and a lesson does not
            // outlive the mode that allowed it.
            if now_mode == mode::Mode::Instructor && applied_mode != mode::Mode::Instructor {
                flight_state::stand_down();
                instructor_noted = false;
                voice.alert(
                    Priority::Advisory,
                    "Instructor mode. My hands are off the controls. Your aircraft.".to_string(),
                );
            } else if applied_mode == mode::Mode::Instructor && now_mode != mode::Mode::Instructor {
                if instructor::abandon("mode_change") {
                    voice.alert(
                        Priority::Advisory,
                        "Lesson over, the mode changed.".to_string(),
                    );
                }
                if now_mode != mode::Mode::AtcOnly {
                    voice.alert(Priority::Advisory, "Copilot back with you.".to_string());
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
                    // Its SIDs, for the IFR clearance.
                    st.set_procedures(&config.cifp_dir, &config.fix_data_path, &config.nav_data_path, &icao);
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

        // Readback enforcement lives INSIDE the tower FSM, so unlike the
        // settings the loop reads per tick, it has to be pushed. Done here and
        // not only at station install, or the Settings screen would show a
        // knob that quietly waits for the next airport.
        if let Some(st) = atc_station.as_mut() {
            st.set_enforce_readback(config.atc_readback);
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
            // A pilot's line in hand (or waiting on the channel) is never held
            // behind the crew's radio work: the waits below give way to it, the
            // lines still play in order. LEMG (2026-09-28, sortie 1790586620):
            // "my controls" was taken at +0.0 and routed at +26.1 s, after the
            // crew's "request taxi" and the tower's whole taxi clearance.
            let pilot_waiting = || loop_owed(input.is_some());
            let calls = circuit::take_crew_radio();
            // A discarded transmission is never silent. The queue is drained
            // unconditionally, so a failed gate loses the call for good — and
            // the caller (full auto) reads the resulting silence as the TOWER
            // not answering, which sent it hunting a clearance that was never
            // asked for (Heathrow 2026-07-27).
            // A readback that cannot go out is OUT for the crew waiting on it
            // (`circuit::readback_out`): it is gone either way.
            let delivered =
                mode::atc_enabled() && voice.radio_powered() && atc_station.is_some();
            if !delivered {
                for q in &calls {
                    if let circuit::CrewCall::Text(t) = &q.call {
                        if q.readback != circuit::Readback::No {
                            circuit::note_readback_out(t);
                        }
                    }
                }
            }
            if !calls.is_empty() && mode::atc_enabled() && !voice.radio_powered() {
                tracing::warn!(
                    calls = calls.len(),
                    "crew radio call dropped: the radio is unpowered"
                );
            }
            if !calls.is_empty() && mode::atc_enabled() && voice.radio_powered() {
                if let Some(st) = atc_station.as_mut() {
                    for queued in calls {
                        // A position report dies with the position it reports,
                        // judged as it is about to go out (each call waits
                        // behind the one before it): LEMG's "5 mile final"
                        // went out 93 s late, in the flare (#143).
                        let age_s = queued.at.elapsed().as_secs_f64();
                        if queued.holds != circuit::Holds::Always {
                            let facts = crew_radio_facts(&xplane).await;
                            if let Some(why) = circuit::stale_reason(queued.holds, age_s, &facts)
                            {
                                tracing::info!(
                                    target: "xvoicelayer::radio",
                                    call = %circuit::call_words(&queued.call),
                                    why,
                                    age_s,
                                    "crew radio call dropped: no longer true"
                                );
                                blackbox::record(
                                    "crew_radio",
                                    serde_json::json!({
                                        "event": "crew_radio_stale",
                                        "call": circuit::call_words(&queued.call),
                                        "why": why,
                                        "age_s": (age_s * 10.0).round() / 10.0,
                                        "on_ground": facts.on_ground,
                                        "agl_ft": facts.agl_ft.map(|h| h.round()),
                                        "gs_kts": facts.gs_kts.map(|g| g.round()),
                                    }),
                                );
                                continue;
                            }
                        }
                        // A copy of the tower's suggested readback goes out only
                        // while the tower still waits for those words: a copy
                        // queued twice went out in the takeoff roll (LEMG 13,
                        // 2026-09-28, #172).
                        if let (circuit::Readback::OwedCopy, circuit::CrewCall::Text(t)) =
                            (queued.readback, &queued.call)
                        {
                            let owed = crate::party_line::readback_owed();
                            if circuit::owed_copy_stale(t, owed.as_deref()) {
                                circuit::note_readback_out(t);
                                blackbox::record(
                                    "crew_radio",
                                    serde_json::json!({
                                        "event": "crew_radio_stale",
                                        "call": t,
                                        "why": "not_owed",
                                        "age_s": (age_s * 10.0).round() / 10.0,
                                    }),
                                );
                                continue;
                            }
                        }
                        blackbox::record(
                            "crew_radio",
                            serde_json::json!({
                                "event": "crew_radio_sent",
                                "call": circuit::call_words(&queued.call),
                                "age_s": (age_s * 10.0).round() / 10.0,
                                "readback": queued.readback != circuit::Readback::No,
                            }),
                        );
                        let queued_readback = queued.readback != circuit::Readback::No;
                        let call = queued.call;
                        // A check-in is rendered NOW, with the frequency quiet
                        // (bounded), so the altitude it names is the one being
                        // passed as it is spoken.
                        let call = match call {
                            circuit::CrewCall::Text(t) => t,
                            circuit::CrewCall::CheckIn {
                                callsign,
                                to,
                                climb_to_ft,
                            } => {
                                let quiet = std::time::Instant::now();
                                while voice.radio_on_air()
                                    && quiet.elapsed().as_secs_f64() < 20.0
                                    && !pilot_waiting()
                                {
                                    tokio::time::sleep(std::time::Duration::from_millis(200)).await;
                                }
                                let alt_ft =
                                    xplane.get_float(datarefs::ALTITUDE_FT).await.unwrap_or(0.0);
                                // Above the flight's transition altitude the
                                // levels are flight levels (issue #97).
                                let ta = crate::tools::instruments::transition_in_force_ft(18_000.0);
                                full_auto::departure_check_in_above(&callsign, &to, alt_ft, climb_to_ft, ta)
                            }
                        };
                        // The copilot transmits (its own voice), then the tower.
                        // Both sides logged so the full ATC transcript shows in
                        // the log, not just the panel.
                        tracing::info!(target: "xvoicelayer::radio", copilot = %call, "tx");
                        // ROUTED before it is spoken: `say` blocks until the
                        // frequency is quiet, and a handoff readback queued
                        // behind a party-line exchange was routed 19 s later,
                        // after full auto had retuned, so Gibraltar DEPARTURE
                        // heard "Gibraltar Departure on one three zero decimal
                        // four" and said "say again" (08:23Z, 2026-09-15). The
                        // station that was tuned when the crew keyed is the one
                        // that hears it; the audible order is unchanged.
                        let routed = st
                            .handle_transmission(&call.to_ascii_lowercase(), xplane.view())
                            .await;
                        if queued_readback {
                            circuit::note_readback_out(&call);
                        }
                        say_or(
                            &voice,
                            &reporter,
                            ui::LogKind::Readback,
                            &format!("{call}."),
                            &pilot_waiting,
                        );
                        if let Some((reply, who)) = routed {
                            if !reply.is_empty() {
                                tracing::info!(target: "xvoicelayer::radio", station = %who.tag(), reply = %reply, "rx");
                                reporter
                                    .log(ui::LogKind::Readback, format!("{}: {reply}", who.tag()));
                                voice.say_station_until(
                                    Priority::Readback,
                                    who,
                                    &reply,
                                    &pilot_waiting,
                                );
                            }
                        }
                    }
                }
            }
        }

        // Station work consumed — an idle tick carries no utterance to route.
        let (mut command, from_text) = match input {
            None => continue,
            Some(Input::Typed(text, taken_at)) => {
                tracing::info!(typed = %text, "typed command");
                // From the channel to routing: the station work above ran
                // between them, and "consumed" alone hid 26 s of it (#135).
                blackbox::record(
                    "typed",
                    serde_json::json!({ "event": "routed", "text": &text,
                                        "held_ms": taken_at.elapsed().as_millis() as u64 }),
                );
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
            checklists = flow::install(&profile);
            instructor::install(&profile);
            stt.set_prompt(stt::build_prompt(&profile));
            stt.set_radio_prompt(format!(
                "{} {}",
                stt::build_prompt(&profile),
                tower::STT_PROMPT
            ));
            // The old airframe's conversation dies with it: a back-reference
            // must never nudge a value commanded on a different aircraft.
            dialogue.clear();
            conversation.clear();
            history.clear();
            last_subject = None;
            if let Ok(mut last) = LAST_READBACK.lock() {
                *last = None;
            }
            voice.forget_speech();
            tracing::info!(aircraft = %profile.display_name, "reconfigured for new aircraft");
        }

        // EMERGENCY ABORT — the very first thing checked, before the confidence,
        // echo, and domain gates and any LLM. "belay that", "undo", "abort",
        // "scratch that", "as you were" restore what the last command changed. It
        // must beat every gate: a panic word that's low-confidence, or glued to an
        // echoed readback, still has to land. Deterministic and instant.
        if is_undo(&command) {
            conversation.clear();
            if !mode::crew_actuates() {
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Readback,
                    "Nothing of mine to undo in instructor mode. Your hands.",
                );
                continue;
            }
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
                        // The parking brake goes back through its valve-aware
                        // seam and is "restored" only when it READS back so: a
                        // raw ratio write left the valve behind and claimed
                        // success on the write alone (#65).
                        let restored = if dr == tools::PARK_BRAKE {
                            matches!(
                                tools::controls::drive_park_brake(
                                    &xplane,
                                    *old > 0.5,
                                    profile.park_brake_valve_set,
                                )
                                .await,
                                Ok(true)
                            )
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
            // "Say again" on the air: the controller on this frequency
            // repeats its last transmission to us, word for word. Never a
            // new clearance: only what was already said, and only its LAST
            // word (see `voice::controller_repeat`). Nothing on file falls
            // through to the station, whose answer is "say again" back.
            if atc_station.is_some()
                && tower::is_say_again(&command)
                && repeat_controller(&voice, &reporter, &command, "keyed")
            {
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

        // The copilot's turn: the first reply waits a human beat, and a turn
        // that goes quiet past `tempo::STANDBY_AFTER` hears "stand by"
        // instead of silence. Dropped on every `continue` below.
        let _turn = {
            let voice = voice.clone();
            tempo::Turn::begin(!from_text, move || {
                if mode::copilot_enabled() {
                    let line = persona::standby_line(persona::dialect());
                    tracing::info!(spoke = %line, "standby filler");
                    voice.alert(Priority::Readback, line);
                }
            })
        };

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
                    pushback::ACTIVE.store(false, std::sync::atomic::Ordering::SeqCst);
                } else if pushback::steered() {
                    conversation.clear();
                    // Full auto's steered push: its loop consumes ABORT, stops
                    // the tug and sets the brakes itself.
                    tracing::info!("pushback stop word, pre-gate");
                    pushback::ABORT.store(true, std::sync::atomic::Ordering::SeqCst);
                    // And the sequence that armed it. "Stop" during a push
                    // means stop, not "skip the push and start taxiing" —
                    // which is what aborting only the push would have full
                    // auto do next. Only a steered push can be full auto's.
                    full_auto::abort();
                    // Nothing said HERE: the push loop answers once it has
                    // set the parking brake, from the read-back
                    // (`pushback::stop_line`). This line used to claim the
                    // brakes before the loop had even seen the word (#110).
                    continue;
                } else {
                    // A NATIVE push (the spoken tool's one-shot command): no
                    // loop is listening, so the stop must reach the tug from
                    // HERE. ABORT alone was a flag nobody read — "stop
                    // pushback", the panel CANCEL's own phrase, was swallowed
                    // by this branch forever while the tug kept pushing and
                    // the copilot claimed brakes it never set (audit,
                    // 2026-08-02).
                    tracing::info!("native pushback stop, pre-gate");
                    conversation.clear();
                    let _ = xplane.run_command(tools::ground::CMD_PUSH_STOP).await;
                    let held = matches!(
                        tools::controls::drive_park_brake(
                            &xplane,
                            true,
                            profile.park_brake_valve_set,
                        )
                        .await,
                        Ok(true)
                    );
                    pushback::ACTIVE.store(false, std::sync::atomic::Ordering::SeqCst);
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
                        continue;
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
                conversation.clear();
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
            continue;
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
                continue;
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
                continue;
            }
            tracing::info!(path = "arrival-brief", "command path");
            if !config.simbrief_id.trim().is_empty() {
                let _ = ensure_plan(&config, &mut fp_cache, PlanFetch::Cached).await;
            }
            let (apt, cifp, fixes) = (
                config.apt_data_path.clone(),
                config.cifp_dir.clone(),
                config.fix_data_path.clone(),
            );
            let line = tokio::task::spawn_blocking(move || {
                rapport::arrival_brief(&apt, &cifp, &fixes, &tools::autoland::xplane_root())
            })
            .await
            .unwrap_or_else(|_| "I couldn't put the arrival brief together.".to_string());
            conversation.observe(&command, &line, Instant::now());
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            continue;
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
            continue;
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
                Some(words) => command = words.to_string(),
                None => continue,
            }
        }
        // Read-only conversation runs before the spoken domain gate: "and now"
        // and clarification replies have no command keyword. Its response type
        // has no tool/action variant, and refreshed questions never enter grammar.
        // What the last state answer was about, taken BEFORE respond() retires
        // the topic for an unrelated utterance: "remind me when we get there"
        // is unrelated to the conversation module but not to the pilot.
        let nav_topic = conversation.recent_observation_query(Instant::now());
        if let Some(response) = conversation.respond(&command, Instant::now()) {
            let line = match response {
                conversation::Response::Say(line) => line,
                conversation::Response::Checklist(request) => {
                    preview_checklist(&request, &checklists, &mut conversation, Instant::now())
                }
                conversation::Response::ReadMany(queries) => {
                    let answers = futures::future::join_all(
                        queries
                            .iter()
                            .map(|query| conversation_read(query, &xplane, &profile)),
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
                    conversation.observe_many(queries, lines, Instant::now());
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
                            conversation.reference(&profile.display_name, &line, Instant::now());
                            line
                        }
                        None => {
                            conversation.clear_topic();
                            "That reference isn't available in this aircraft's profile.".into()
                        }
                    }
                }
                conversation::Response::Read(query) => {
                    // Explicit rechecks use REST rather than an un-timestamped
                    // pushed cache entry that might predate a disconnected socket.
                    match conversation_read(query, &xplane, &profile).await {
                        Some(line) => {
                            conversation.observe(query, &line, Instant::now());
                            line
                        }
                        None => {
                            conversation.clear_topic();
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
            continue;
        }
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

        // A pilot waiving the refusal just said ("fly it anyway"): the gate
        // is waived for that plan and the arm is asked again in the same
        // words the button uses. Words with no refusal on offer fall through.
        if grammar::waiver_request(&command) {
            if let Some(gate) = full_auto::take_waiver() {
                say(&voice, &reporter, ui::LogKind::Readback, &full_auto::waiver_ack(gate));
                command = "fly my flight plan".to_string();
            }
        }

        // From here the utterance is resolved deterministically: the grammar
        // (one call per distinct action, so "taxi and nav lights on" makes
        // two), the dialogue stack for back-references, the state and
        // knowledge answerers for questions. Nothing guesses; a miss is a miss.
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
                let _ = ensure_plan(&config, &mut fp_cache, PlanFetch::Cached).await;
            }
            let line =
                fuel_report(&xplane, &profile, Some(&live), is_endurance_query(&command)).await;
            conversation.observe(&command, &line, Instant::now());
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            continue;
        }
        // Interphone: an utterance ADDRESSED to the cabin ("cabin crew,
        // prepare for landing") is theirs, answered deterministically —
        // cabin calls are rare and formulaic, so the LLM never sees them.
        // The address is the routing (no third PTT; see crate::cabin). On an
        // airframe with no cabin crew the copilot answers honestly instead
        // of letting the model improvise a flight attendant.
        // A bare "cabin crew?" leaves the line open: the next utterance is
        // theirs without the address, when it is a cabin call.
        let addressed = cabin::Cabin::addressed(&command);
        let on_open_line = match addressed {
            Some(_) => None,
            None => cabin_state
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .take_open_line(&command, Instant::now()),
        };
        if addressed.is_some() || on_open_line.is_some() {
            if cabin::aboard(profile.cabin_crew) {
                let reply = match on_open_line {
                    Some(reply) => reply,
                    None => cabin_state
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .handle(addressed.as_deref().unwrap_or(""), Instant::now()),
                };
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
        // Only ON THE GROUND: airborne, "where are we going?" is about the
        // flight plan, and the taxi reading answered it with "we don't have a
        // taxi clearance yet" at 1,000 ft (LEMG, 2026-09-12). A failed read
        // keeps the old ground reading.
        let on_ground_now = || async {
            xplane
                .get_float(datarefs::ON_GROUND_ANY)
                .await
                .map(|v| v > 0.5)
                .unwrap_or(true)
        };
        if is_taxi_query(&command)
            && grammar::try_parse(&command, &registry, &profile).is_none()
            && on_ground_now().await
        {
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
                // A taxi-in between its drives (the request's wait) holds
                // the aircraft with no auto taxi running.
                let taxi_in_running = crate::taxi_in::running();
                let hover_holding = hover::ACTIVE.load(Relaxed);
                // A helicopter's route or circuit: in cruise the hover loop
                // is not running, and the route's speed law owns the
                // collective until it is told to let go.
                let heli_flying =
                    heli_route::ACTIVE.load(Relaxed) || heli_circuit::ACTIVE.load(Relaxed);
                if (hover_holding
                    || heli_flying
                    || speed_held
                    || mix_held
                    || reverse_in
                    || taxi_driving
                    || taxi_in_running
                    || takeoff_flying
                    || circuit_flying
                    || procedure_flying
                    || full_auto_running)
                    && release_intent(&tl)
                    // "Put us on final, my controls" is a new flight the
                    // pilot will fly, not a hand-back of this one.
                    && !grammar::approach_setup_request(&tl)
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
                    // A helicopter's hover: the collective is ours and the
                    // autopilot holds the position. "My controls" or naming
                    // the collective hands both back where they are.
                    if heli_flying && (generic || tl.contains("collective")) {
                        // The route lets go of the collective where it is and
                        // leaves the autopilot in the mode it is flying.
                        heli_route::abort();
                        heli_circuit::abort();
                        released.push("route off");
                    }
                    if hover_holding && (generic || tl.contains("collective")) {
                        hover::abort();
                        released.push("hover off, your controls");
                    }
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
                        autotakeoff::cancel();
                        released.push("your takeoff, power is set");
                    }
                    // The taxi-in ends where it is, braked to a stop and
                    // parked: a stopped drive alone was driven again to the
                    // stand 7 s later (LEMG, 2026-09-28).
                    let taxi_in_ended = taxi_in_running && (names_speed || generic);
                    if taxi_in_ended {
                        crate::taxi_in::abort(true);
                        released.push("taxi in off, your aircraft");
                    }
                    if !circuit_flying
                        && !full_auto_running
                        && !taxi_in_ended
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
                    } else if speed_held && !taxi_in_ended && (names_speed || generic) {
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
                        crate::flight::note_pilot_takeover("my controls");
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
                // "Put us on a ten mile final" carries the radio word
                // "final": it is the crew's reposition, never a final report.
                if !grammar::approach_setup_request(&tl)
                    && try_route_content_atc(st, &tl, dual_ptt, &voice, &reporter, &xplane).await
                {
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
                    let callsign = crate::circuit::crew_callsign(&xplane)
                        .await
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
                    // "Should we check in with ATC?" wants a yes or a no ahead
                    // of the call, not just the phraseology.
                    let yes_no = station::coach_yes_no(&tl);
                    let line = match station::suggest_call(&callsign, on_ground, dist, com1) {
                        Some(s) => match (s.first, s.call) {
                            (Some(f), Some(c)) => {
                                let lead = if yes_no { "Yes. " } else { "" };
                                format!("{lead}{f} Then key the radio and say: {c}")
                            }
                            (Some(f), None) if yes_no => format!("No call owed right now. {f}"),
                            (Some(f), None) => f,
                            (None, Some(c)) => {
                                let lead = if yes_no { "Yes. " } else { "" };
                                format!("{lead}Key the radio and say: {c}")
                            }
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
            // "Check the flight plan": the whole-flight audit, nothing armed. Before the
            // systems check, whose "check ... aircraft" these words can have.
            if grammar::plan_check_request(&tl) {
                reporter.status(ui::Phase::Thinking);
                let line = plan::plan_check_report(&config, &xplane, &mut fp_cache).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
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
                conversation.reference(&profile.display_name, &line, Instant::now());
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            if systems::is_systems_query(&tl) {
                let line = systems::current().line();
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            if let Some(line) = aircraft_info(&profile, &tl) {
                conversation.reference(&profile.display_name, &line, Instant::now());
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // The local flight planner: "plan a flight to Malaga", "divert to
            // Gibraltar", or the short answer to its own "to where?". Sits
            // before the reminder/TOD/ETA branches, which key on words a
            // destination request never carries. Deterministic: apt.dat,
            // the profile, and the sim's terrain probe. A plan on the shelf
            // authorises nothing; "load it" is a separate word.
            {
                let asked = plan_pending.is_some_and(|t| t.elapsed() < PLAN_ASK_WINDOW);
                let req = flightplan::parse_request(&tl).or_else(|| {
                    // An answer is a place, short, and not a command in its own right.
                    let words = tl.split_whitespace().count();
                    let reply = tl.trim().strip_prefix("to ").unwrap_or(tl.trim());
                    (asked
                        && words <= 4
                        && grammar::try_parse(&command, &registry, &profile).is_none())
                    .then(|| flightplan::parse_destination(reply))
                    .flatten()
                    .map(|d| flightplan::PlanRequest {
                        dest: Some(d),
                        load: false,
                    })
                });
                if let Some(req) = req {
                    plan_pending = None;
                    let line = match req.dest {
                        None => {
                            plan_pending = Some(Instant::now());
                            "To where?".to_string()
                        }
                        Some(_) => {
                            reporter.status(ui::Phase::Thinking);
                            plan_flight(&config, &xplane, &profile, &mut fp_cache, &req).await
                        }
                    };
                    conversation.observe(&command, &line, Instant::now());
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    continue;
                }
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
                conversation.observe(&command, &line, Instant::now());
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // Weather somewhere ELSE ("what weather can we expect on arrival",
            // "weather at LEMG"): the sim's datarefs describe the air we are in,
            // and reading them back for an arrival question is a confident wrong
            // answer (seen live 45 nm out of Malaga, 2026-09-12). The real-weather
            // METAR file is the honest source; without one, say so.
            if !is_reminder && weather_words(&tl) {
                if let Some(target) =
                    remote_weather_target(&tl, fp_cache.as_ref().map(|(_, fp)| fp))
                {
                    let line = match target {
                        RemoteWeather::NoPlan => {
                            "No flight plan loaded, so I have no destination to look up."
                                .to_string()
                        }
                        RemoteWeather::Station { icao, label } => {
                            match weather::briefing(&tools::autoland::xplane_root(), &icao, &label)
                            {
                                Ok(l) | Err(l) => l,
                            }
                        }
                    };
                    conversation.observe(&command, &line, Instant::now());
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    continue;
                }
            }
            // ETA to the destination ("time before we land", "when do we get in",
            // "how far before we land"): distance to the destination airport over
            // groundspeed. Beats the next-waypoint branch below (`eta_question`).
            if !is_reminder && eta_question(&tl) {
                let line = eta_to_destination(&config, &xplane, &mut fp_cache).await;
                conversation.observe(&command, &line, Instant::now());
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
                    let _ = ensure_plan(&config, &mut fp_cache, PlanFetch::Cached).await;
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
            // "Do we have enough runway for takeoff?": the takeoff's own fit
            // and pavement rule, said as two numbers. Ahead of the runway
            // ADVICE below, whose "use"/"should" would otherwise take it.
            if !is_reminder && answers::takeoff_runway_query(&tl) {
                reporter.status(ui::Phase::Thinking);
                let line = answers::takeoff_runway_answer(&xplane, &profile).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
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
            // A NAMED fix ("how much left for PIMOS", "what's after PIMOS"):
            // the ident is matched fuzzily against the loaded plan, because
            // whisper spells a fix it has never seen phonetically ("pimmons"
            // for PIMOS, live 2026-09-12). Only a plan already on the shelf is
            // consulted: a stray "for X" must never trigger a SimBrief fetch.
            if !is_reminder {
                if let Some(q) = fp_cache
                    .as_ref()
                    .and_then(|(_, fp)| fix_query(&tl, &fp.waypoints))
                {
                    let line = fix_answer(&q, &xplane).await;
                    conversation.observe(&command, &line, Instant::now());
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    continue;
                }
            }
            // Navigation progress (often a command, not a question): ETA / distance /
            // time to the next waypoint.
            if !is_reminder
                && (tl.contains("next waypoint")
                    || tl.contains("next fix")
                    || tl.contains("eta")
                    || tl.contains("how far")
                    || tl.contains("time to")
                    || tl.contains("distance to")
                    || tl.contains("how much left")
                    || tl.contains("how much further")
                    || tl.contains("how much longer"))
            {
                let line = nav_progress(&xplane, Some(&live)).await;
                conversation.observe(&command, &line, Instant::now());
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // "route" is reliably misheard as "root"/"road" (both meaningless in a
            // cockpit), so accept them as synonyms.
            let routeish = tl.contains("route") || tl.contains("root") || tl.contains("road");
            // Load the SimBrief route into the GPS/FMS (checked before the brief, so
            // "load the flight plan" loads rather than just reading it back).
            // "load it" / "send that" right after a local plan was briefed is the
            // follow-up the brief itself invites.
            let load_followup =
                fp_cache.is_some() && tl.split_whitespace().any(|w| w == "it" || w == "that");
            // Not the load SHEET ("load the fuel from the flight plan"): that is
            // the `load_aircraft` tool's, through the grammar below.
            let wants_load = !grammar::load_aircraft_request(&tl)
                && (tl.contains("load") || tl.contains("program") || tl.contains("send"))
                && (routeish
                    || load_followup
                    || tl.contains("plan")
                    || tl.contains("gps")
                    || tl.contains("fms")
                    || tl.contains("flight"));
            if wants_load {
                reporter.status(ui::Phase::Thinking);
                let line =
                    load_route_to_fms(&config, &xplane, &mut fp_cache, plan_fetch_for(&tl)).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // "That's the wrong flight plan": the pilot has regenerated it on
            // SimBrief; fetch again and say whether anything actually changed
            // (whisper: "wrong flag plan", live 2026-09-14).
            if wrong_plan_complaint(&tl) {
                reporter.status(ui::Phase::Thinking);
                let before = fp_cache.as_ref().map(|(_, fp)| fp.clone());
                let line = match ensure_plan(&config, &mut fp_cache, PlanFetch::Forced).await {
                    Err(msg) => msg,
                    Ok(fetched) => plan_change_line(
                        before.as_ref(),
                        fp_cache.as_ref().map(|(_, fp)| fp),
                        fetched,
                    ),
                };
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
            // SimBrief flight plan (cached fetch) + advice. Route, full brief, or
            // refresh. Not when the intent is to *fly* it ("follow/navigate/track"
            // is the navigate tool; the word "fly" is the full-auto SimBrief
            // mission — both handled via the grammar/LLM path below).
            let nav_intent = grammar::flight_plan_request(&tl)
                || grammar::resume_plan_request(&tl)
                || tl.contains("navigate")
                || tl.contains("follow")
                || tl.contains("track")
                || tl
                    .split(|c: char| !c.is_ascii_alphanumeric())
                    .any(|w| w == "fly" || w == "execute");
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
                let want_route = routeish;
                let line = flight_plan_report(
                    &config,
                    &xplane,
                    &mut fp_cache,
                    plan_fetch_for(&tl),
                    want_route,
                    reporter.status_sink(),
                )
                .await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                continue;
            }
        }
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
            continue;
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
            continue;
        }
        // Conversational glue (acks, "what can you do") — deterministic, takes
        // no action. Checked before commands/questions so pleasantries don't
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
                answer_state_query(&matchable, &xplane, Some(&live), &profile).await
            {
                conversation.observe(&command, &answer, Instant::now());
                say(&voice, &reporter, ui::LogKind::Readback, &answer);
                continue;
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
            match dialogue.resolve(&matchable, &registry) {
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
                    continue;
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
                conversation.report(&command, &msg, false, Instant::now());
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
            // written: `xvl analyze` lists them by frequency.
            tracing::info!(path = "deterministic-miss", "command path");
            reporter.log(ui::LogKind::Warn, format!("unrecognized: \"{command}\""));
            if explicit && live.get(DR_OPT_TONE).is_none_or(|v| v > 0.5) {
                voice.negative();
            }
            continue;
        };

        // "You have control": the crew hand-over, resolved against the flight
        // model into what the flight needs now (`handover::decide`), then run
        // as ordinary calls so every gate and readback still applies.
        let calls = if calls
            .first()
            .is_some_and(|c| c.name == grammar::TAKE_OVER)
        {
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
                    continue;
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

        // Readbacks from this utterance's actions, recorded as the assistant
        // turn afterwards so the next command can refer back to what was done.
        let mut turn_readbacks: Vec<String> = Vec::new();
        let mut turn_failed = false;
        // Recovery for everything this utterance does, gathered so one "belay that"
        // undoes the whole set of commands, not just the last tool.
        let mut turn_reverts: Vec<(String, String, f64, bool)> = Vec::new();
        let mut turn_disengage_ap = false;
        let mut turn_inverse: Vec<(String, serde_json::Value)> = Vec::new();
        for mut call in calls {
            // A hand-over's fallback: only if the calls before it left the
            // aircraft to nobody (a refused resume).
            if let Some(args) = call.arguments.as_object_mut() {
                if args.remove(handover::IF_IDLE).is_some() {
                    if monitor::flying_loop().is_some() {
                        continue;
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
                    break;
                }
                let line =
                    instructor::take_target(&call.name, &call.arguments).unwrap_or_else(|| {
                        let label = callout_label(&call.name, &call.arguments);
                        instructor::refusal(&label, !instructor_noted)
                    });
                instructor_noted = true;
                blackbox::record(
                    "instructor",
                    serde_json::json!({ "event": "refused", "tool": call.name, "args": call.arguments }),
                );
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                break;
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
                    break;
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
                    break;
                }
            }
            if let Some(line) = hover::blocks(&call.name) {
                blackbox::record(
                    "hover",
                    serde_json::json!({ "event": "blocked", "tool": call.name }),
                );
                say(&voice, &reporter, ui::LogKind::Readback, line);
                break;
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
                break;
            }

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
                let parked_shutdown = call.name == "stop_engines"
                    && tools::jet_start::stopped_on_ground(&xplane).await;
                // An all-off asks only with an engine turning on the ground:
                // cold it is the normal shutdown, airborne the tool refuses
                // (issue #66).
                let electrics_clear = call.name == "set_electrical"
                    && tools::electrical::is_all_off(&call.arguments)
                    && tools::electrical::all_off_gate(&xplane).await
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
                        // The correction re-parses through the grammar with the
                        // original words in front ("flaps ten" + "twenty" reads
                        // as flaps twenty); nothing outside the grammar is tried.
                        match grammar::try_parse(
                            &format!("{command}, {correction}"),
                            &registry,
                            &profile,
                        ) {
                            Some(mut cs) if !cs.is_empty() => {
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
            turn_failed |= !ok;
            // Gather this action's recovery into the turn (pushed once after the loop).
            if ok {
                // Anchor for back-references ("a bit more", "turn it off"):
                // what was actually executed, exactly as called.
                dialogue.record(&call.name, &call.arguments);
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
        if !turn_readbacks.is_empty() {
            conversation.report(
                &command,
                &turn_readbacks.join(" "),
                turn_failed,
                Instant::now(),
            );
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

/// Discuss the loaded checklist without invoking its runner or any step. Require
/// an unambiguous title; a partial name such as "engine" offers the candidates.
fn preview_checklist(
    request: &str,
    checklists: &[checklist::Checklist],
    conversation: &mut conversation::Conversation,
    now: Instant,
) -> String {
    let request = request.to_ascii_lowercase();
    let query: Vec<_> = request
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|s| !s.is_empty())
        .collect();
    let candidates: Vec<_> = checklists
        .iter()
        .filter(|cl| {
            let title = cl.title.to_ascii_lowercase();
            let title: Vec<_> = title.split(|c: char| !c.is_ascii_alphanumeric()).collect();
            query.iter().all(|word| title.contains(word))
        })
        .collect();
    let selected = candidates
        .iter()
        .find(|cl| cl.title.eq_ignore_ascii_case(&request))
        .copied()
        .or_else(|| (candidates.len() == 1 && !request.is_empty()).then(|| candidates[0]));
    let Some(cl) = selected else {
        let choices = if candidates.is_empty() {
            checklists.iter().collect()
        } else {
            candidates
        };
        return conversation
            .choose_checklist(choices.iter().map(|cl| cl.title.clone()).collect(), now);
    };
    let items = cl
        .items
        .iter()
        .map(|item| match item {
            checklist::Item::Narrate(line)
            | checklist::Item::CallOut(line)
            | checklist::Item::Act { say: line, .. }
            | checklist::Item::Verify { say: line, .. } => line.clone(),
        })
        .collect();
    conversation.start_preview(cl.title.clone(), items, now)
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

/// Mine the logs for fast-path coverage and the miss tail. Pairs each spoken
/// transcript with the `command path` line that followed it, then reports the
/// hit/miss split and the most frequent MISSED phrasings — the ones worth a
/// rule in [`grammar`] or a matcher next. The ratchet that makes the copilot
/// cover more the more it's flown.
fn analyze_commands() -> Result<()> {
    use std::collections::HashMap;
    let mut last: Option<String> = None;
    let (mut fast, mut miss) = (0u32, 0u32);
    let mut tail: HashMap<String, u32> = HashMap::new();

    let mut paths: Vec<std::path::PathBuf> = std::fs::read_dir("logs")
        .map(|d| d.filter_map(|e| e.ok().map(|e| e.path())).collect())
        .unwrap_or_default();
    paths.sort();
    // `--since 2026-09-20`: only logs from that day on (the dated suffix).
    let since = std::env::args()
        .skip_while(|a| a != "--since")
        .nth(1)
        .unwrap_or_default();
    for p in paths {
        let is_log = p
            .file_name()
            .and_then(|n| n.to_str())
            .is_some_and(|n| {
                n.starts_with("xvoicelayer.log")
                    && (since.is_empty() || n.rsplit('.').next().is_some_and(|d| d >= since.as_str()))
            });
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
                if line.contains("path=\"deterministic-miss\"") {
                    miss += 1;
                    if let Some(t) = &last {
                        *tail.entry(t.clone()).or_default() += 1;
                    }
                } else if line.contains("path=") {
                    fast += 1;
                }
            }
        }
    }

    let total = fast + miss;
    if total == 0 {
        println!("No command-path data in logs yet — run the app and issue some commands first.");
        return Ok(());
    }
    println!("commands analyzed: {total}");
    println!("  handled:            {fast:>5}  ({}%)", fast * 100 / total);
    println!("  missed:             {miss:>5}  ({}%)", miss * 100 / total);
    // Replayed through TODAY's deterministic chain (the 172's registry and
    // checklists): what is still missed is the work list. Questions need the
    // live sim to answer, so they are listed apart.
    if std::env::args().any(|a| a == "--now") {
        let profile = aircraft::load("cessna_172sp").ok_or_else(|| anyhow::anyhow!("no 172 profile"))?;
        let registry = Registry::for_aircraft(&profile);
        let checklists = flow::install(&profile);
        let (mut still, mut questions, mut fixed) = (Vec::new(), Vec::new(), 0u32);
        for (t, n) in &tail {
            // The live chain: STT's own repairs, then the grammar's.
            let m = grammar::normalize(&phrase::normalize_numbers(&phrase::fix_mishears(t)));
            let handled = router::conversational(&m, &registry).is_some()
                || grammar::try_parse(&m, &registry, &profile).is_some()
                || checklist::match_request(&m, &checklists).is_some()
                || grammar::declined(&m, &registry).is_some();
            if handled {
                fixed += n;
            } else if looks_like_question(&m) {
                questions.push((t.clone(), *n));
            } else {
                still.push((t.clone(), *n));
            }
        }
        still.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        questions.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        println!("\nreplayed through today's grammar: {fixed} of {miss} now handled");
        println!("\nstill missed, commands ({}):", still.len());
        for (t, n) in &still {
            println!("  {n:>3}  {t}");
        }
        println!("\nquestions (answered from the live sim, not checked here) ({}):", questions.len());
        for (t, n) in &questions {
            println!("  {n:>3}  {t}");
        }
        return Ok(());
    }
    println!("\nrule candidates, the most frequent missed phrasings:");
    let mut v: Vec<(String, u32)> = tail.into_iter().collect();
    v.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    for (t, n) in v.into_iter().take(20) {
        println!("  {n:>3}  {t}");
    }
    Ok(())
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
    // The pilot has stopped talking: the reply's human beat is timed from
    // here, so the transcription itself counts toward it.
    tempo::note_capture_end();
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
    /// The line, and when the loop took it off the channel.
    Typed(String, Instant),
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
            // How long it sat on the channel (#90): the plugin's `taken`
            // record says how long it sat on the panel before that.
            let queued_ms = pollers::took_command().map(|d| d.as_millis() as u64);
            crate::blackbox::record(
                "typed",
                serde_json::json!({ "event": "consumed", "text": t, "queued_ms": queued_ms }),
            );
            let t = t.trim().to_string();
            if !t.is_empty() {
                return Some(Input::Typed(t, Instant::now()));
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
static LAST_READBACK: std::sync::Mutex<Option<(String, Instant)>> = std::sync::Mutex::new(None);

fn say(voice: &Voice, reporter: &Reporter, kind: ui::LogKind, line: &str) {
    say_or(voice, reporter, kind, line, || loop_owed(false));
}

/// The live facts a queued crew call is judged against (#143), read together.
/// A failed read is `None`: no measurement, never a reason to drop a call.
async fn crew_radio_facts(xp: &XPlaneClient) -> circuit::RadioFacts {
    let (ground, agl, gs) = tokio::join!(
        xp.get_float(datarefs::ON_GROUND_ANY),
        xp.get_float(datarefs::AGL_METERS),
        xp.get_float(autotaxi::GS_MS),
    );
    circuit::RadioFacts {
        on_ground: ground.ok().map(|v| v > 0.5),
        agl_ft: agl.ok().map(|m| m * 3.28084),
        gs_kts: gs.ok().map(|ms| ms.abs() * 1.943_844),
    }
}

/// Is a pilot's line owed the command loop's attention: one already taken off
/// the channel and not yet routed (`in_hand`), or one still waiting on it.
/// The command loop's waits on its OWN speech give way to either (#90, #135).
fn pilot_line_pending(in_hand: bool) -> bool {
    in_hand || pollers::command_waiting()
}

/// Is the command loop owed back from a wait on its own speech: a pilot's
/// line ([`pilot_line_pending`]) or an armed station swap, which only this
/// loop installs. LEMG (2026-09-28, sorties 1790611629 and 1790619465, #167):
/// a tower call stepped on the setup's readback, the loop waited 22 and 38 s
/// for its remainder, and the new flight's station, armed 0.4 s after the
/// readback's stall, came up after the final call had given up.
fn loop_owed(in_hand: bool) -> bool {
    loop_owed_by(pilot_line_pending(in_hand), party_line::station_swap_pending())
}

/// [`loop_owed`] over its two facts. Pure.
fn loop_owed_by(pilot_line: bool, swap_armed: bool) -> bool {
    pilot_line || swap_armed
}

/// [`say`] with the wait's give-way condition handed in.
fn say_or(
    voice: &Voice,
    reporter: &Reporter,
    kind: ui::LogKind,
    line: &str,
    stop: impl Fn() -> bool,
) {
    // The first reply of a turn waits a human beat after the pilot stopped
    // talking (never when busy, never for typed input): `tempo`.
    tempo::reaction_pause();
    tracing::info!(spoke = %line, "spoke");
    reporter.status(ui::Phase::Speaking);
    reporter.log(kind, line);
    // Blocks until spoken, unless the pilot has a line waiting (typed, a
    // panel button): the loop goes to take it and the line plays on (#90).
    voice.say_until(Priority::Readback, line, stop);
    reporter.status(ui::Phase::Listening);
    if matches!(kind, ui::LogKind::Readback) {
        if let Ok(mut last) = LAST_READBACK.lock() {
            *last = Some((line.to_string(), Instant::now()));
        }
    }
}

/// A 3-degree descent loses ~1000 ft every 3 nm; Stratos (and most pilots) use a
/// slightly gentler ~3.5 nm per 1000 ft as the planning ring. Used only when
/// neither the FMS nor the SimBrief navlog gives a top-of-descent directly.
pub(crate) const TOD_NM_PER_1000FT: f64 = 3.5;

/// The capture tuning currently applied to the open microphone.
///
/// The three mic knobs are the ones a pilot tunes by ear, so they are live —
/// but they live INSIDE the audio device's own state, not in a value read per
/// frame. This tracks what was last pushed so a change from the settings
/// screen is applied exactly once, between utterances, rather than re-pushed
/// (and re-logged) on every tick of a silent cockpit.
struct CaptureTuning {
    vad_bits: u32,
    end_silence_ms: u32,
    gate_tail_ms: u64,
}

impl CaptureTuning {
    fn of(config: &Config) -> Self {
        CaptureTuning {
            vad_bits: config.vad_threshold.to_bits(),
            end_silence_ms: config.end_silence_ms,
            gate_tail_ms: config.mic_gate_tail_ms,
        }
    }

    /// Push any changed value into the live mic. Only one of listener/ptt_mic
    /// ever exists (continuous vs push-to-talk), but both take the echo guard.
    fn follow(&mut self, config: &Config, listener: Option<&mut Listener>, ptt: Option<&PttMic>) {
        let next = CaptureTuning::of(config);
        if next.vad_bits == self.vad_bits
            && next.end_silence_ms == self.end_silence_ms
            && next.gate_tail_ms == self.gate_tail_ms
        {
            return;
        }
        let tail = std::time::Duration::from_millis(next.gate_tail_ms);
        if let Some(l) = listener {
            l.retune(config.vad_threshold, next.end_silence_ms, tail);
        }
        if let Some(m) = ptt {
            m.set_gate_tail(tail);
        }
        tracing::info!(
            vad = config.vad_threshold,
            end_silence_ms = next.end_silence_ms,
            gate_tail_ms = next.gate_tail_ms,
            "capture retuned"
        );
        *self = next;
    }
}

/// The session facts the desktop health screen shows: everything settled at
/// boot, after every fallback and collision fix has been applied. Read off the
/// resolved [`Config`], never off the environment, so what the screen reports
/// is what the process is actually running.
fn boot_health(config: &Config) -> health::Boot {
    // The capture path in precedence order — the same order `main` wires it,
    // so a PTT dataref that quietly outranks a configured key is visible
    // rather than mysterious.
    let listen = if !config.ptt_dataref.trim().is_empty() {
        format!("ptt dataref · {}", config.ptt_dataref)
    } else if !config.ptt_key.trim().is_empty() {
        format!("ptt key · {}", config.ptt_key)
    } else {
        config.listen_mode.clone()
    };

    let mut voices = vec![("cabin", voice_stem(&config.cabin_piper_voice))];
    if !config.atc_airport.trim().is_empty() {
        voices.push(("tower", voice_stem(&config.atc_piper_voice)));
        voices.push(("approach", voice_stem(&config.atc_approach_piper_voice)));
        voices.push(("departure", voice_stem(&config.atc_departure_piper_voice)));
    }

    health::Boot {
        version: env!("XVL_VERSION").to_string(),
        build: concat!(env!("XCOPILOT_BUILD"), ", ", env!("XVL_PROFILE")).to_string(),
        whisper: format!(
            "{} · {}",
            whisper_model_label(&config.whisper_model_path),
            if config.whisper_gpu { "gpu" } else { "cpu" }
        ),
        listen,
        mic: audio::default_input_name().unwrap_or_else(|| "no input device".into()),
        xplane_url: config.xplane_base_url.clone(),
        atc_airport: config.atc_airport.trim().to_string(),
        simbrief: !config.simbrief_id.trim().is_empty(),
        voices,
    }
}

/// A voice model path as the screen names it: the file stem, which is how the
/// panel's picker and the config comments name them too.
fn voice_stem(spec: &str) -> String {
    if spec.trim().is_empty() {
        return "(inherits tower)".into();
    }
    // A pool-style `path#N` spec keeps its speaker number: two entries from the
    // same multi-speaker model are different people.
    let (path, sid) = match spec.rsplit_once('#') {
        Some((p, n)) => (p, Some(n)),
        None => (spec, None),
    };
    let stem = std::path::Path::new(path)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| spec.to_string());
    match sid {
        Some(n) => format!("{stem}#{n}"),
        None => stem,
    }
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
    // The field's SIDs (CIFP), for the IFR clearance's departure.
    st.set_procedures(
        &config.cifp_dir,
        &config.fix_data_path,
        &config.nav_data_path,
        &home_icao,
    );
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
    // "Gibraltar Tower, say again" with no radio key: addressed to the
    // station by name, so the controller repeats (the bare "say again" is
    // the copilot's).
    if tower::is_say_again(tl)
        && station::is_radio_call(
            tl,
            &station::spoken_airport(st.airport_name()).to_lowercase(),
        )
        && repeat_controller(voice, reporter, tl, "unkeyed")
    {
        return true;
    }
    // While the station decides, the words may be a radio call: no copilot
    // "stand by" over it. Handed back if the station declines them.
    tempo::set_radio_turn(true);
    let Some((reply, who)) = st.try_handle(tl, xplane.view()).await else {
        tempo::set_radio_turn(false);
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
/// The pilot's "say again" on the air: the controller on this frequency
/// repeats its last transmission to us word for word, if it made one in the
/// last minute (`voice::controller_repeat`). Never a new clearance: only the
/// controller's LAST word to us, so a cancelled clearance is not on file.
/// `false`: nothing to repeat, and the station answers "say again" back.
fn repeat_controller(voice: &Voice, reporter: &Reporter, transcript: &str, route: &str) -> bool {
    let Some(last) = voice.repeat_on_frequency() else {
        return false;
    };
    blackbox::record(
        "atc",
        serde_json::json!({
            "route": route, "outcome": "say_again_repeat", "transcript": transcript,
            "service": last.speaker.tag(), "reply": &last.text,
            "age_s": last.at.elapsed().as_secs(),
        }),
    );
    reporter.status(ui::Phase::Speaking);
    reporter.log(
        ui::LogKind::Readback,
        format!("{}: {}", last.speaker.tag(), last.text),
    );
    voice.say_station(Priority::Readback, last.speaker, &last.text);
    reporter.status(ui::Phase::Listening);
    true
}

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

/// Keeps the sim's own radio audio in line with [`tts::sim_radio`]: applied
/// when the wanted state changes, retried every 5 s until the writes land
/// (the sim may not be up yet at boot). Undoes only what it did itself: the
/// slider is raised only from a mute (ours, or one left by an earlier
/// session: under 5 %), the ATIS switched back on only if we switched it off.
#[derive(Default)]
struct SimRadioSync {
    applied: Option<(bool, bool)>,
    last_try: Option<std::time::Instant>,
    atis_off_by_us: bool,
}

impl SimRadioSync {
    const VOLUME: &'static str = "sim/operation/sound/radio_volume_ratio";
    const ATIS: &'static str = "sim/atc/atis_enabled";

    async fn follow(&mut self, xp: &XPlaneClient, own_atc: bool, bus: bool) {
        if self.applied == Some((own_atc, bus))
            || self
                .last_try
                .is_some_and(|t| t.elapsed() < std::time::Duration::from_secs(5))
        {
            return;
        }
        self.last_try = Some(std::time::Instant::now());
        let want = tts::sim_radio(own_atc, bus);
        let volume_ok = if want.volume_pct == 0 {
            xp.set_float(Self::VOLUME, 0.0).await.is_ok()
        } else {
            match xp.get_float(Self::VOLUME).await {
                Ok(v) if v < 0.05 => xp.set_float(Self::VOLUME, 1.0).await.is_ok(),
                Ok(_) => true,
                Err(_) => false,
            }
        };
        let atis_ok = match want.atis {
            Some(false) => {
                let ok = xp.set_int(Self::ATIS, 0).await.is_ok();
                self.atis_off_by_us |= ok;
                ok
            }
            Some(true) if self.atis_off_by_us => {
                let ok = xp.set_int(Self::ATIS, 1).await.is_ok();
                self.atis_off_by_us &= !ok;
                ok
            }
            _ => true,
        };
        if volume_ok && atis_ok {
            self.applied = Some((own_atc, bus));
            tracing::info!(own_atc, bus, volume_pct = want.volume_pct, atis = ?want.atis, "sim radio audio set");
            blackbox::record(
                "sim_radio",
                serde_json::json!({ "own_atc": own_atc, "bus": bus,
                                   "volume_pct": want.volume_pct, "atis": want.atis }),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        classify_confidence, is_affirmative, is_undo, join_and, plan_change_line, plan_fetch_for,
        progress_line, radio_panel_line, undoable_drefs, wrong_plan_complaint, ConfidenceAction,
        PlanFetch, Ptt,
    };

    /// #135: a line the loop has taken off the channel and not yet routed is
    /// owed the loop as much as one still waiting on it. The crew-radio drain
    /// saw only the channel, and "my controls" (taken, so the channel was
    /// empty) waited 26.1 s behind the tower's taxi clearance at LEMG.
    #[test]
    fn a_line_in_hand_is_owed_the_loop_like_a_waiting_one() {
        assert!(super::pilot_line_pending(true));
    }

    /// #167: the records of sortie 1790611629 (LEMG, 2026-09-28). The loop
    /// spoke the setup's readback at +0.5 s, the tower stepped on it at
    /// +7.8 s, the swap was armed at +8.4 s and installed at +30.2 s, when the
    /// stale remainder was dropped. No pilot line was waiting, so only the
    /// armed swap can release the wait.
    #[test]
    fn an_armed_station_swap_releases_the_loops_speech_wait() {
        use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
        // Before the swap: the readback is waited out.
        assert!(!super::loop_owed_by(false, false));
        // Armed: the loop goes to install it.
        assert!(super::loop_owed_by(false, true));
        assert!(super::loop_owed_by(true, false));
        // Through the real wait: a readback never spoken (stepped on, its
        // remainder queued behind the radio) gives way the moment the swap
        // is armed, not when the line is dropped stale 20 s later.
        let armed = std::sync::Arc::new(AtomicBool::new(false));
        let a = armed.clone();
        let t = std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_millis(100));
            a.store(true, SeqCst);
        });
        let t0 = std::time::Instant::now();
        let spoken =
            crate::voice::wait_unspoken_until(|| super::loop_owed_by(false, armed.load(SeqCst)));
        t.join().unwrap();
        assert!(!spoken);
        assert!(t0.elapsed() < std::time::Duration::from_secs(2), "{:?}", t0.elapsed());
    }

    fn plan(origin: &str, dest: &str, route: &str, n: usize) -> crate::simbrief::FlightPlan {
        crate::simbrief::FlightPlan {
            transition_altitude_ft: None,
            transition_level_ft: None,
            procedures: Default::default(),
            callsign: None,
            schedule: Default::default(),
            source: crate::simbrief::PlanSource::SimBrief,
            origin: origin.into(),
            destination: dest.into(),
            alternate: String::new(),
            aircraft: "C172".into(),
            cruise_alt_ft: 5500.0,
            distance_nm: 60.0,
            ete_min: 40.0,
            block_fuel_kg: 30.0 * 2.72,
            trip_fuel_kg: 10.0 * 2.72,
            reserve_kg: 8.0 * 2.72,
            min_takeoff_fuel_kg: 18.0 * 2.72,
            weights: None,
            route: route.into(),
            wind_comp_kts: 0.0,
            wind_dir: 0.0,
            wind_spd_kts: 0.0,
            tod_to_dest_nm: 0.0,
            waypoints: (0..n)
                .map(|i| crate::simbrief::Waypoint {
                    ident: format!("W{i}"),
                    lat: 36.0,
                    lon: -4.0,
                    alt: 5500.0,
                })
                .collect(),
        }
    }

    /// Live 2026-09-14: "load the flight plan" four times in two minutes
    /// after regenerating on SimBrief, each one re-sent LEMG-LEJR from the
    /// shelf. A spoken ask re-fetches; only a refresh word forces it over a
    /// local plan.
    #[test]
    fn spoken_plan_asks_refetch_simbrief() {
        for t in [
            "load the flight plan.",
            "let's load the flight plan.",
            "load the flight plane.",
            "load the flight lamp.",
            "what's our flight plan",
        ] {
            assert_eq!(plan_fetch_for(t), PlanFetch::Asked, "{t}");
        }
        for t in [
            "refresh the flight plan",
            "reload the plan",
            "update the flight plan",
        ] {
            assert_eq!(plan_fetch_for(t), PlanFetch::Forced, "{t}");
        }
    }

    #[test]
    fn wrong_plan_complaint_is_closed_shape() {
        // Live whisper output.
        assert!(wrong_plan_complaint("that's the wrong flag plan."));
        assert!(wrong_plan_complaint("that's the wrong flight plan"));
        assert!(wrong_plan_complaint("not that plan"));
        assert!(wrong_plan_complaint("wrong route"));
        // A wrongness word without a plan word is not a plan complaint.
        assert!(!wrong_plan_complaint("wrong, flaps ten"));
        assert!(!wrong_plan_complaint("that's the wrong runway"));
        assert!(!wrong_plan_complaint("load the flight plan"));
    }

    #[test]
    fn plan_change_line_says_whether_simbrief_moved() {
        let old = plan("LEMG", "LEJR", "MGA DCT", 9);
        let same = plan("LEMG", "LEJR", "MGA DCT", 9);
        let new = plan("LEJR", "LEMG", "JRZ DCT", 6);
        assert_eq!(
            plan_change_line(Some(&old), Some(&same), true),
            "SimBrief still has LEMG to LEJR. Generate the new plan there, then say load the \
             flight plan."
        );
        assert_eq!(
            plan_change_line(Some(&old), Some(&new), true),
            "Got the new plan, LEJR to LEMG, 6 waypoints. Say load it to send it to the GPS."
        );
        // Nothing on the shelf before: whatever came is new.
        assert!(plan_change_line(None, Some(&new), true).starts_with("Got the new plan"));
        assert_eq!(
            plan_change_line(Some(&old), Some(&old), false),
            "Couldn't reach SimBrief. I still have LEMG to LEJR."
        );
        assert!(plan_change_line(None, None, false).starts_with("No plan on the shelf"));
    }

    #[test]
    fn progress_line_names_fix_distance_and_time() {
        assert_eq!(
            progress_line("MG430", Some(2.39), Some(1.43)),
            "Next waypoint MG430, 2 miles, 1 minutes."
        );
        assert_eq!(
            progress_line("BUKOX", Some(33.4), None),
            "Next waypoint BUKOX, 33 miles."
        );
        assert_eq!(
            progress_line("LEMG", Some(130.0), Some(95.0)),
            "Next waypoint LEMG, 130 miles, 1 hours 35 minutes."
        );
    }

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
        // A COMMAND naming the approach is not a speed question: "fly the
        // approach" was swallowed here and answered "approach around 100
        // knots" instead of arming the Malaga arrival (2026-08-02). The
        // question keeps working; it always carries a speed word.
        assert!(super::reference_limits(&p, "fly the approach").is_none());
        assert!(super::reference_limits(&p, "shoot the approach").is_none());
        assert!(super::reference_limits(&p, "what's our approach speed")
            .unwrap()
            .contains("approach around"));

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
    fn conversation_sortie_uses_real_profile_answers_without_a_model() {
        use super::conversation::{Conversation, Response};
        let now = std::time::Instant::now();
        let profile = super::aircraft::load("cessna_172sp").unwrap();
        let mut crew = Conversation::default();
        let line = super::reference_limits(&profile, "what's best glide").unwrap();
        crew.reference(&profile.display_name, &line, now);
        let Some(Response::Say(why)) = crew.respond("why that speed?", now) else {
            panic!("must explain the reference");
        };
        assert!(why.contains("68 knots"));
        assert!(why.contains("aircraft profile"));
        let Some(Response::Reference(question)) = crew.respond("and blue line?", now) else {
            panic!("must request reference, never a command");
        };
        assert!(
            super::reference_limits(&profile, question).is_none(),
            "no twin speed invented on a single"
        );

        crew.observe("what is our altitude", "3500 feet", now);
        for words in ["and heading?", "and flaps?", "and gear?", "and fuel?"] {
            let Some(Response::Read(question)) = crew.respond(words, now) else {
                panic!("{words} must be a reading");
            };
            assert!(super::looks_like_question(question));
        }
        let registry = super::Registry::for_aircraft(&profile);
        assert!(crew.respond("flaps ten", now).is_none());
        let calls = super::grammar::try_parse("flaps ten", &registry, &profile).unwrap();
        assert_eq!(calls[0].name, "set_flaps");
        crew.report("flaps ten", "Negative, too fast for flaps.", false, now);
        let Some(Response::Say(why)) = crew.respond("why?", now) else {
            panic!("missing refusal")
        };
        assert!(why.contains("Negative, too fast"));
        assert!(
            matches!(
                crew.respond("check that again", now),
                Some(Response::Say(_))
            ),
            "cannot replay the flap command"
        );
    }

    #[test]
    fn checklist_preview_uses_loaded_items_and_clarifies_ambiguous_titles() {
        use super::conversation::{Conversation, Response};
        let now = std::time::Instant::now();
        let profile = super::aircraft::load("cessna_172sp").unwrap();
        let lists = super::checklist::parse(
            "sw_checklist:Before Takeoff\nsw_item:Brakes|SET\nsw_item:Flaps|TEN\n\
             sw_checklist:Before Landing\nsw_item:Mixture|RICH\n",
            &profile,
        );
        let mut crew = Conversation::default();
        let line = super::preview_checklist("before", &lists, &mut crew, now);
        assert!(line.contains("Which checklist"), "{line}");
        let Some(Response::Checklist(title)) = crew.respond("before takeoff", now) else {
            panic!("missing preview choice")
        };
        let line = super::preview_checklist(&title, &lists, &mut crew, now);
        assert!(line.contains("item 1 of 2"), "{line}");
        assert!(line.contains("Brakes"), "{line}");
        let Some(Response::Say(line)) = crew.respond("next item", now) else {
            panic!("missing next item")
        };
        assert!(line.contains("Flaps"));
        let line =
            super::preview_checklist("before takeoff and start engines", &lists, &mut crew, now);
        assert!(
            line.contains("Which checklist"),
            "must not silently select a partial title"
        );
        assert!(super::preview_checklist("", &[], &mut crew, now).contains("No checklists"));
    }

    #[test]
    fn conversational_fuel_questions_select_the_dedicated_reporter() {
        let mut crew = super::conversation::Conversation::default();
        for text in ["could you tell me our fuel", "tell me about endurance"] {
            let Some(super::conversation::Response::Read(query)) =
                crew.respond(text, std::time::Instant::now())
            else {
                panic!("missing read")
            };
            assert!(
                super::is_fuel_query(query),
                "{query} must use fuel_report, not the instrument-only reader"
            );
        }
    }

    /// The Baron answers the two questions only a twin gets asked, and the
    /// mapping that gets you the profile in the first place.
    #[test]
    fn the_baron_recites_the_twin_speeds() {
        let baron = "Aircraft/Laminar Research/Beechcraft Baron 58/Baron_58.acf";
        assert_eq!(
            super::detect::resolved_id("BE58", baron).as_deref(),
            Some("beechcraft_baron_58")
        );
        assert_eq!(
            super::detect::resolved_id("be58", baron).as_deref(),
            Some("beechcraft_baron_58")
        );
        let p = super::aircraft::load("beechcraft_baron_58").expect("baron profile");
        assert!(super::reference_limits(&p, "what's blue line")
            .unwrap()
            .contains("100 knots"));
        assert!(super::reference_limits(&p, "what's vmc")
            .unwrap()
            .contains("84 knots"));
        // Per-notch Vfe, spoken as two limits rather than one.
        let vfe = super::reference_limits(&p, "flap speed").unwrap();
        assert!(vfe.contains("152") && vfe.contains("122"), "{vfe}");
        // A single has neither twin speed, so it says nothing rather than
        // inventing one.
        let c172 = super::aircraft::load("cessna_172sp").expect("172 profile");
        assert!(super::reference_limits(&c172, "what's blue line").is_none());

        // The profile shapes the tool surface: retractable gear earns a gear
        // tool the 172 never gets, and a fuel-injected engine earns no carb
        // heat. This is the whole point of per-airframe profiles.
        let reg = super::tools::Registry::for_aircraft(&p);
        assert!(reg.get("gear").is_some(), "the Baron's gear retracts");
        assert!(reg.get("set_magnetos").is_some());
        assert!(reg.get("start_engine").is_some());
        assert!(
            reg.get("set_carb_heat").is_none(),
            "IO-550s are fuel injected"
        );
        assert!(super::tools::Registry::for_aircraft(&c172)
            .get("gear")
            .is_none());
    }

    // The RV-10 flew on the 172's numbers for lack of a profile (2026-09-11).
    #[test]
    fn the_rv10_has_its_own_profile_and_unknown_types_are_not_the_172() {
        assert_eq!(
            super::detect::resolved_id(
                "RV10",
                "Aircraft/Laminar Research/Van's Aircraft RV-10/RV-10.acf"
            )
            .as_deref(),
            Some("vans_rv10")
        );
        let who = super::detect::AcfIdentity {
            icao: "ZZZZ".into(),
            relative_path: "Aircraft/Somebody/Zed/Zed Plane.acf".into(),
            ..Default::default()
        };
        assert_eq!(
            super::detect::resolve(&who, &super::aircraft::profile_matchers()),
            super::detect::Resolution::UnknownType,
            "unknown is unknown, not a Cessna"
        );
        assert_eq!(
            super::detect::generic_profile_id(&who),
            "generic_zzzz_zed_plane"
        );
        let p = super::aircraft::load("vans_rv10").expect("RV-10 profile loads");
        let r = p.reference.as_ref().expect("reference card");
        assert_eq!(
            r.approach_kts,
            Some(80.0),
            "not the 172's 65, which is the RV-10's clean stall"
        );
        assert_eq!(p.vs_clean_kts, Some(65.0));
        assert_eq!(p.bus_volts_nominal, Some(12.0));
        assert!(!p.retractable_gear && !p.carb_heat && p.magnetos);
        let flaps = p.flaps.as_ref().expect("flaps");
        let vfe: Vec<Option<f64>> = flaps.detents.iter().map(|d| d.vfe_kts).collect();
        assert_eq!(vfe, vec![None, Some(96.0), Some(87.0)]);
        let sel = p.fuel_selector.as_ref().expect("xlua fuel selector");
        assert!(
            sel.readback
                .as_deref()
                .is_some_and(|r| r.contains("laminar/rv10")),
            "the stock selector is not the RV-10's"
        );
    }

    #[test]
    fn citation_x_loads_with_the_poms_own_figures() {
        let p = super::aircraft::load("citation_x").expect("citation_x profile loads");
        assert_eq!(
            super::detect::resolved_id(
                "C750",
                "Aircraft/Laminar Research/Cessna Citation X/Cessna_CitationX.acf"
            )
            .as_deref(),
            Some("citation_x")
        );
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
        let pat = super::circuit::pattern_speed_kts(vy, r.approach_kts.unwrap(), p.vs_clean_kts);
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
        assert_eq!(
            super::detect::resolved_id(
                "SF50",
                "Aircraft/Laminar Research/Cirrus Vision SF50/CirrusSF50.acf"
            )
            .as_deref(),
            Some("cirrus_vision_sf50")
        );
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
        // Securing the aircraft is the KNOB, then the START/STOP button, not
        // the mixture (the script rewrites it): `ignition_down` drops the boost
        // pump and igniter, and only a button press with the knob at OFF sets
        // the button state the script's shutdown runs on. The knob alone left
        // the engine running at LSZS (2026-09-24).
        assert_eq!(
            s.shutdown_commands,
            vec![
                "laminar/SF50/ignition_down".to_string(),
                "laminar/SF50/eng_start_stop_toggle".to_string()
            ]
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
        let is_abort = |w: &str| crate::flow::classify(w) == Some(crate::flow::Reply::Abort);
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

    /// "Stop" is a control word and must reach the router from a cold gate.
    /// It was missing from the keyword table, so in continuous mode it only
    /// ever worked through the pre-gate branch that fires while the speed
    /// governor is running. With a push under way — brakes off for the tug,
    /// no governor — a Baron rolled off its Southampton stand to 21 kt and
    /// "Stop." at 0.9996 confidence was dropped as ambient, no miss chime
    /// (2026-08-01).
    #[test]
    fn stop_reaches_the_router_with_no_governor_running() {
        for c in ["Stop.", "stop", "Stop stop stop!", "Stop the aircraft."] {
            assert!(looks_like_command(c), "{c:?} must pass the domain gate");
        }
        // Passing the GATE is not actuating: the grammar's bare-stop rule
        // still wants the whole utterance, so a sentence that merely contains
        // the word goes to the router, not to the brakes.
        let prof = crate::aircraft::load("beechcraft_baron_58").expect("Baron profile");
        let reg = crate::tools::Registry::for_aircraft(&prof);
        let stop = super::grammar::try_parse("stop", &reg, &prof)
            .expect("bare stop must parse to a command");
        assert_eq!(stop[0].name, "hold_speed");
        assert_eq!(stop[0].arguments["knots"], 0);
        assert!(
            super::grammar::try_parse("we should stop for fuel at the next field", &reg, &prof)
                .is_none(),
            "a sentence that merely contains the word must not brake"
        );
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

    /// Configuration-state questions ("is the gear down?") carry the same
    /// keywords as their command twins ("gear down"), so the question OPENER
    /// is what routes them to the answer path — these must all read as
    /// questions, and the loop's gate then withholds them from the grammar
    /// (which would otherwise ACT on the words inside).
    #[test]
    fn config_state_questions_route_to_the_answer_path() {
        for q in [
            "is the gear down",
            "are the flaps up",
            "is the parking brake set",
            "is the pitot heat on",
            "where are the flaps",
        ] {
            assert!(looks_like_question(q), "{q:?} should be a question");
        }
        for c in ["gear down", "flaps up", "parking brake on", "pitot heat on"] {
            assert!(!looks_like_question(c), "{c:?} must stay a command");
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

#[cfg(test)]
mod gap_tests_2026_09_12 {
    //! The utterances that missed live on 2026-09-12 (LXGB to LEMG, 172SP),
    //! verbatim from the log, each now routed deterministically.
    use super::*;

    fn plan() -> simbrief::FlightPlan {
        let wp = |ident: &str, lat: f64, lon: f64| simbrief::Waypoint {
            ident: ident.into(),
            lat,
            lon,
            alt: 8000.0,
        };
        simbrief::FlightPlan {
            transition_altitude_ft: None,
            transition_level_ft: None,
            procedures: Default::default(),
            callsign: None,
            schedule: Default::default(),
            source: simbrief::PlanSource::Local,
            origin: "LXGB".into(),
            destination: "LEMG".into(),
            alternate: "LEZL".into(),
            aircraft: "C172".into(),
            cruise_alt_ft: 8000.0,
            distance_nm: 60.0,
            ete_min: 45.0,
            block_fuel_kg: 30.0 * 2.72,
            trip_fuel_kg: 8.0 * 2.72,
            reserve_kg: 6.0 * 2.72,
            min_takeoff_fuel_kg: 14.0 * 2.72,
            weights: None,
            route: "PIMOS DEKUS".into(),
            wind_comp_kts: 0.0,
            wind_dir: 0.0,
            wind_spd_kts: 0.0,
            tod_to_dest_nm: 15.0,
            waypoints: vec![
                wp("LXGB", 36.15, -5.35),
                wp("PIMOS", 36.30, -5.10),
                wp("DEKUS", 36.45, -4.80),
                wp("LEMG", 36.67, -4.50),
            ],
        }
    }

    #[test]
    fn a_fix_whisper_misspelled_is_matched_from_the_plan() {
        let fp = plan();
        // "How much left for pimmons?" -> the range to PIMOS.
        let q = fix_query("how much left for pimmons", &fp.waypoints).unwrap();
        assert_eq!((q.ident.as_str(), q.after, q.idx), ("PIMOS", false, 1));
        assert_eq!(q.next.as_deref(), Some("DEKUS"));
        // "And what's the waypoint to act at Pimos?" -> what follows PIMOS.
        let q = fix_query("and whats the waypoint to act at pimos", &fp.waypoints).unwrap();
        assert_eq!((q.ident.as_str(), q.after), ("PIMOS", true));
        let q = fix_query("whats after dekus", &fp.waypoints).unwrap();
        assert_eq!((q.ident.as_str(), q.after), ("DEKUS", true));
        assert!(fix_query("when do we reach dekus", &fp.waypoints).is_some());
        // Airports at either end are the destination ETA's job, not a fix.
        assert!(fix_query("how far to lemg", &fp.waypoints).is_none());
        // Commands and the plain progress question never match a fix.
        assert!(fix_query("set heading to 270", &fp.waypoints).is_none());
        assert!(fix_query("how long till next waypoint", &fp.waypoints).is_none());
        assert!(fix_query("how much left for next waypoint", &fp.waypoints).is_none());
        assert!(fix_query("turn to pilot heading", &fp.waypoints).is_none());
        // First letter must agree: "dimos" is not PIMOS.
        assert!(fix_query("how far to dimos", &fp.waypoints).is_none());
        assert_eq!(ident_distance("pimmons", "pimos"), 1);
    }

    #[test]
    fn arrival_weather_points_at_the_destination_never_the_air_we_are_in() {
        let fp = plan();
        let st = |icao: &str| RemoteWeather::Station {
            icao: icao.into(),
            label: icao.into(),
        };
        assert_eq!(
            remote_weather_target("what weather can we expect on arrival", Some(&fp)),
            Some(st("LEMG"))
        );
        assert_eq!(
            remote_weather_target("whats the wind at the destination", Some(&fp)),
            Some(st("LEMG"))
        );
        assert_eq!(
            remote_weather_target("weather at the alternate", Some(&fp)),
            Some(st("LEZL"))
        );
        assert_eq!(
            remote_weather_target("weather at lxgb", Some(&fp)),
            Some(st("LXGB"))
        );
        assert_eq!(
            remote_weather_target("weather on arrival", None),
            Some(RemoteWeather::NoPlan)
        );
        // Here and now stays with the sim's own weather.
        assert!(remote_weather_target("whats the weather", Some(&fp)).is_none());
        assert!(remote_weather_target("whats the wind here", Some(&fp)).is_none());
        assert!(weather_words("what weather can we expect on arrival"));
        assert!(!weather_words("how much fuel do we have"));
    }

    /// "Do we have enough runway for takeoff?" went unanswered on the A330
    /// at Edinburgh (2026-09-20, transcript verbatim). The answer is the
    /// takeoff's own fit, said as two numbers.
    #[test]
    fn the_runway_length_question_is_answered_from_the_takeoffs_own_fit() {
        use crate::answers::{takeoff_runway_line, takeoff_runway_query};
        use crate::autotakeoff::NoFit;
        for q in [
            "Do we have enough runway for takeoff?",
            "is the runway long enough",
            "how much runway do we need",
            "is this runway too short for us",
        ] {
            assert!(takeoff_runway_query(q), "{q}");
        }
        // Commands, advice and chatter stay where they were.
        for q in [
            "line us up runway two four",
            "which runway should we use",
            "runway two four cleared for takeoff",
            "do we have enough fuel",
        ] {
            assert!(!takeoff_runway_query(q), "{q}");
        }
        assert_eq!(
            takeoff_runway_line(Ok(("24".into(), 2338.0)), 2113.0),
            "Yes. Runway two four, 2,340 metres ahead of us, and we need 2,110 at this weight."
        );
        let short = takeoff_runway_line(
            Err(NoFit::RunwayTooShort {
                id: "24".into(),
                length_m: 2549.0,
                needed_m: 2560.0,
            }),
            2560.0,
        );
        assert!(
            short.starts_with("No. Runway two four is 2,550 metres and we need 2,560"),
            "{short}"
        );
        let far = takeoff_runway_line(
            Err(NoFit::TooLittleLeft {
                id: "24".into(),
                remaining_m: 141.0,
                needed_m: 2113.0,
            }),
            2113.0,
        );
        assert!(far.starts_with("Not from here. 140 metres left"), "{far}");
        assert!(takeoff_runway_line(
            Err(NoFit::NoneNear {
                id: String::new(),
                lateral_m: 0.0
            }),
            2113.0
        )
        .contains("not lined up"));
        // A length question with no runway near still cannot be measured, even
        // now that `NoneNear` knows which strip it looked for — that naming is
        // the REFUSAL's job (`autotakeoff::no_fit_line`), not this answer's.
        let away = takeoff_runway_line(
            Err(NoFit::NoneNear {
                id: "07".into(),
                lateral_m: 169.0,
            }),
            2113.0,
        );
        assert!(away.contains("not lined up"), "{away}");
    }

    #[test]
    fn speed_questions_get_a_verdict_against_the_phase() {
        let r = aircraft::ReferenceProfile {
            vy_kts: Some(74.0),
            approach_kts: Some(65.0),
            cruise_ias_kts: Some(110.0),
            ..Default::default()
        };
        let mut c = SpeedContext {
            on_ground: false,
            vvi_fpm: 500.0,
            agl_ft: 1100.0,
            flap_ratio: 0.0,
            hold_kts: None,
            floor_kts: Some(58.0),
        };
        // "Aren't we going a bit slow?" at 72 in the climb: Vy is 74.
        assert_eq!(
            speed_verdict(72.0, &c, Some(&r)).as_deref(),
            Some("we're climbing at Vy, 74, that's right.")
        );
        assert!(speed_verdict(62.0, &c, Some(&r))
            .unwrap()
            .contains("a touch slow"));
        // Below the floor outranks everything, including a governor target.
        c.hold_kts = Some(70.0);
        assert!(speed_verdict(55.0, &c, Some(&r))
            .unwrap()
            .contains("below the 58 knot floor"));
        assert_eq!(
            speed_verdict(71.0, &c, Some(&r)).as_deref(),
            Some("I'm holding 70, on target.")
        );
        c.hold_kts = None;
        // Cruise.
        c.vvi_fpm = 0.0;
        c.agl_ft = 7000.0;
        assert!(speed_verdict(90.0, &c, Some(&r))
            .unwrap()
            .contains("we're slow"));
        assert!(speed_verdict(108.0, &c, Some(&r))
            .unwrap()
            .contains("about right"));
        // Approach, flaps out.
        c.flap_ratio = 0.6;
        assert!(speed_verdict(59.0, &c, Some(&r))
            .unwrap()
            .contains("Add power"));
        // On the ground, and with no profile, just the number.
        c.on_ground = true;
        assert!(speed_verdict(20.0, &c, Some(&r)).is_none());
        c.on_ground = false;
        assert!(speed_verdict(90.0, &c, None).is_none());

        assert!(speed_assessment_words("is our speed okay"));
        assert!(speed_assessment_words("arent we going a bit slow"));
        assert!(speed_assessment_words("are we too fast"));
        assert!(!speed_assessment_words("whats our speed"));
        assert!(!speed_assessment_words("how fast are we going"));
    }

    #[test]
    fn negated_auxiliaries_and_should_we_are_questions() {
        for q in [
            "arent we going a bit slow",
            "aren't we going a bit slow",
            "isnt the gear down",
            "should we check in with atc",
            "shall we call the tower",
            "is our speed okay",
        ] {
            assert!(looks_like_question(q), "{q:?}");
        }
        // A polite request is still a command once the normaliser has peeled it.
        assert!(!looks_like_question(
            grammar::normalize("Can you fly the flight plan?").trim_end_matches('?')
        ));
    }
}

#[cfg(test)]
mod scratch_probe {
    #[test]
    #[ignore]
    fn probe_phrases() {
        let profile = crate::aircraft::load("cessna_172sp").unwrap();
        let registry = crate::tools::Registry::for_aircraft(&profile);
        for t in std::env::var("PROBE").unwrap_or_default().split('|') {
            let m = crate::grammar::normalize(&crate::phrase::normalize_numbers(&crate::phrase::fix_mishears(t)));
            let r = crate::grammar::try_parse(&m, &registry, &profile);
            println!("PROBE {t:?} -> {m:?} -> {:?}", r.map(|c| c.iter().map(|x| format!("{} {}", x.name, x.arguments)).collect::<Vec<_>>()));
        }
    }
}
