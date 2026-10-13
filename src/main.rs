//! XFlightCrew: the rest of the crew, and the world around you, for X-Plane 12.
//!
//! The voice loop, end to end and fully local:
//!   listen → whisper (STT) → deterministic command/conversation routing →
//!   write dataref + read back → say/piper (TTS) confirms.
//!
//! Two capture modes (config `listen_mode`): `continuous` (always listening,
//! VAD-segmented) or `ptt` (push-to-talk).

// The autoland tick's blackbox record is one `json!` past the default 128.
#![recursion_limit = "256"]

mod aircraft;
mod answers;
mod ap_engage;
mod ap_lands;
mod autoflight;
mod approach_energy;
mod approach_guide;
mod audio;
mod auto_status;
mod autoland;
mod autotakeoff;
mod autotaxi;
mod cabin;
mod checklist;
mod checklist_panel;
mod circuit;
mod climb_watch;
mod cockpit_auto;
mod config;
mod control;
mod crew_pace;
mod conversation;
mod course_leg;
mod crew_radio;
mod debrief;
mod heading_bug;
mod sortie_runner;
mod conform;
mod test_card;
mod detect;
mod dialogue;
mod efb;
mod failures;
mod flight;
mod duty;
mod handover;
mod identify;
mod flight_state;
mod scoped;
mod lifetimes;
mod flightplan;
mod flow;
mod full_auto;
mod go_around;
mod ground_steer;
// Test-only (the file's own `#![cfg(test)]`): an outer test attribute here
// would end the source `sortie_runner::code_of` reads.
mod free_nose_replay;
mod grammar;
mod health;
mod heli_circuit;
mod heli_route;
mod hover;
mod instructor;
mod interaction;
mod livestate;
mod lnav;
mod mic;
mod mode;
mod monitor;
mod ils;
mod scorecard;
mod persona;
mod phrase;
mod pilot;
mod pilot_altitude;
mod plan;
mod plugin_status;
mod pollers;
mod prefs;
mod plan_audit;
mod preflight;
mod procedure;
mod pushback;
mod radiofx;
mod rapport;
mod roll_steer;
mod readback;
mod recovery_plan;
mod reversal;
mod route_finder;
mod router;
mod runway;
mod safe_hold;
mod settings;
mod shortcuts;
mod simbrief;
mod stable_approach;
mod stt;
mod stt_eval;
mod systems;
mod takeoff_config;
mod taxi_guide;
mod taxi_in;
mod tempo;
mod terrain;
mod takeoff_thrust;
mod takeoff_trim;
mod tiller;
mod yoke;
// Test-only (the file's own `#![cfg(test)]`), as `free_nose_replay` above.
mod test_support;
mod tools;
mod traffic_link;
mod traffic_sink;
mod tts;
#[cfg(feature = "onnx-tts")]
mod tts_onnx;
mod ui;
mod undo;
mod vad;
mod voice;
mod voices;
mod weather;
mod world_tick;

// The world side (ATC) and the simulator plumbing now live in their own
// crates; these shims keep every existing `crate::nav::...` /
// `crate::xplane::...` path in the module tree resolving unchanged. The
// station itself is the world's own: the crew reaches it only through
// `party_line` (inputs, reads, types) and `world` (the ticks world_tick
// drives), pinned by `the_station_is_reached_only_through_the_seam`.
pub(crate) use ::atc::{airspace, atis, callsign, nav, party_line, speaker, tower, world};
pub(crate) use sim::{angles, blackbox, datarefs, xplane};

use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};

use answers::{
    aircraft_info, answer_state_query, config_advice, conversation_read,
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
    engages_autopilot, is_undo, join_and, recovery_label, undo_targets, UndoSnapshot,
    UNDO_STACK_MAX,
};
use voice::{Priority, Voice};
use xplane::XPlaneClient;

/// The default tracing filter: every workspace crate's target AND both crew
/// names, the package (the lib's default target) and the binary (the bin's),
/// CLAUDE.md "Wire contracts". Pinned by
/// `tests::the_log_filter_names_every_crate_and_both_crew_names`.
const DEFAULT_LOG_FILTER: &str = "xflightcrew=info,xfc=info,atc=info,nav=info,sim=info,traffic=info";

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
    // `xfc voices [list|install|remove]`: the voice and speech-model manager
    // (src/voices.rs). First, before the log is set up: its screen owns the
    // terminal, and it never talks to the sim.
    if std::env::args().nth(1).as_deref() == Some("voices") {
        let rest: Vec<String> = std::env::args().skip(2).collect();
        let ok = voices::main_cmd(rest, config)?;
        std::process::exit(if ok { 0 } else { 1 });
    }
    // `xfc paths`: where every file lives (the mode and both roots,
    // `sim::paths`), the models, the log and the prefs. Never talks to the sim.
    if std::env::args().nth(1).as_deref() == Some("paths") {
        print!("{}", paths_report());
        return Ok(());
    }
    let tui = config.ui == "tui";

    // Always log to a daily-rolling file. In TUI mode the stderr layer would
    // scribble over the dashboard, so it's omitted there (file only).
    let file_appender =
        tracing_appender::rolling::daily(sim::paths::logs_dir(), sim::paths::LOG_FILE_NAME);
    let (file_writer, _log_guard) = tracing_appender::non_blocking(file_appender);
    // Every workspace crate, not just the crew: since the 2026-07-14 split the
    // world's tracing lives in `atc` and the Web API client's in `sim`, and a
    // crew-only filter silently dropped both — "release watch armed" and "wake
    // clock started" were invisible in flight logs, so a Heathrow departure
    // held for eight minutes had to be diagnosed by probing the live sim
    // instead of grepping (2026-07-27).
    // BOTH names for the crew, and they are not the same thing: a log line's
    // default target is `module_path!()`, which for code in a BINARY is the
    // BIN name (`xfc`) and for code in the lib is the PACKAGE name
    // (`xflightcrew`). Naming only the package silently dropped all 158
    // default-target sites in this file and its modules — the startup/build
    // line included, so the one documented way to tell which build is flying
    // ("check the build= line") reported nothing, while the explicit
    // `target: "xflightcrew::..."` lines kept arriving and made the log look
    // healthy (2026-07-30, found by grepping for a startup line that was
    // never there). Renaming the binary must move this default with it.
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| DEFAULT_LOG_FILTER.into());
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
        "XFlightCrew starting"
    );
    // Where every file lives, decided once (`sim::paths`).
    tracing::info!(paths = %sim::paths::summary(), "file roots");
    if let Some(missing) = sim::paths::missing_line() {
        tracing::warn!("{missing}");
    }

    // Publish the configuration to the live store and let saved preferences
    // fill in every registry knob the environment is silent about (env still
    // wins — see settings.rs). From here on the boot path uses this resolved
    // snapshot, while the loops re-read `settings::live()` each tick so a
    // change made from the Settings screen applies without a restart.
    let config: Config = (*settings::install(config)).clone();
    // The station's callsign resolver asks the crew for the override and the
    // plan's callsign through this provider, read fresh each time.
    callsign::provide_inputs(crew_radio::callsign_inputs);
    // ...and the pilot's own airline telephony ("WAT=Walker"), asked before
    // its built-in tables, the same way.
    callsign::provide_telephony(settings::airline_telephony);
    // ...and the route we filed, so an IFR clearance names the SID that
    // joins it, by the crew's own rule (#152).
    party_line::provide_filed_route(simbrief::filed_route);

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
            .ok_or_else(|| anyhow!("usage: xfc transcribe <audio-file>"))?;
        let text = transcribe_file(&config.whisper_model_path, &path)?;
        println!("{text}");
        return Ok(());
    }

    // `cargo run -- onnx-info <model>` dumps an ONNX model's input/output signature.
    #[cfg(feature = "onnx-tts")]
    if std::env::args().nth(1).as_deref() == Some("onnx-info") {
        let path = std::env::args()
            .nth(2)
            .ok_or_else(|| anyhow!("usage: xfc onnx-info <model.onnx>"))?;
        println!("{}", tts_onnx::inspect(&path)?);
        return Ok(());
    }

    // `xfc tts ["phrase"] [--voice SPEC] [--pace N] [--radio] [--out FILE.wav]`
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
            let click = sim::paths::asset("afv/Click_f32.wav");
            match radiofx::load_tail_wav(&click.to_string_lossy(), v.sample_rate()) {
                Ok(tail) => radiofx::radioize_with_tail(&mut samples, v.sample_rate(), 1, &tail),
                Err(_) => radiofx::radioize_variant(&mut samples, v.sample_rate(), 1),
            }
        }
        let path = match &out {
            Some(p) => std::path::PathBuf::from(p),
            None => std::env::temp_dir().join("xflightcrew-tts.wav"),
        };
        tts_onnx::write_wav(&path, &samples, v.sample_rate())?;
        match out {
            Some(_) => eprintln!("wrote {}", path.display()),
            None => tts::play(&path)?,
        }
        return Ok(());
    }

    // `xfc debrief [logs/blackbox.jsonl]` — what the last flight did, from the
    // blackbox: the read a pilot gets after the sortie, and the one the
    // fly-fix-fly loop starts from.
    if std::env::args().nth(1).as_deref() == Some("debrief") {
        let path = std::env::args()
            .nth(2)
            .unwrap_or_else(|| sim::paths::blackbox_file().to_string_lossy().into_owned());
        let text = std::fs::read_to_string(&path).map_err(|e| anyhow!("reading {path}: {e}"))?;
        print!("{}", debrief::report(text.lines()));
        return Ok(());
    }

    // Both processes hold their traffic through a flight load (the plugin's
    // `xcopilot/sim/loading`; failing open without it).
    xplane::enable_load_gate(&config.xplane_base_url);

    // `xfc sortie <spec.yaml> [--repeat N]`: the unattended runner, a second
    // process beside a running xfc (src/sortie_runner.rs). Exit 1 on any fail.
    if std::env::args().nth(1).as_deref() == Some("sortie") {
        let rest: Vec<String> = std::env::args().skip(2).collect();
        let all_passed =
            sortie_runner::main_cmd(&rest, &config.xplane_base_url, &config.apt_data_path).await?;
        std::process::exit(if all_passed { 0 } else { 1 });
    }

    // `xfc conform [--only <area>]`: every control channel the live profile
    // declares, exercised on a PARKED aircraft through the code's own seams
    // and put back (src/conform.rs). A second process like the runner. Exit
    // 1 on any FAIL or a refusal.
    if std::env::args().nth(1).as_deref() == Some("conform") {
        let rest: Vec<String> = std::env::args().skip(2).collect();
        let passed = conform::main_cmd(&rest, &config.xplane_base_url, &config.aircraft).await?;
        std::process::exit(if passed { 0 } else { 1 });
    }

    // `xfc identify`: the measurement flight that teaches the pilot
    // controller the loaded airframe (src/identify.rs), its fits stored per
    // .acf in profiles/learned/. A second process like conform: xfc stopped.
    if std::env::args().nth(1).as_deref() == Some("identify") {
        let rest: Vec<String> = std::env::args().skip(2).collect();
        let stored = identify::main_cmd(&rest, &config.xplane_base_url).await?;
        std::process::exit(if stored { 0 } else { 1 });
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

    // Voices through the sim's audio: the `audio_bus` setting, the saved pick
    // already applied by `settings::install`; the environment pins it (#228).
    tts::configure_audio_bus(
        config.audio_bus,
        std::env::var("XCOPILOT_AUDIO_BUS").is_ok(),
        &config.xplane_base_url,
    );
    tracing::info!(
        aircraft = %profile.display_name,
        mode = %config.listen_mode,
        "starting XFlightCrew"
    );

    // whisper model load is the one slow init — do it once, up front. Bias its
    // decoding toward the active aircraft's actual systems and phraseology.
    // Text only (`XCOPILOT_LISTEN=off`): no model, no microphone, no PTT;
    // lines come from the panel's typed box and the sortie runner.
    let listen_off = config.listen_mode == "off";
    LISTEN_OFF.store(listen_off, std::sync::atomic::Ordering::Relaxed);
    let stt = if listen_off {
        tracing::info!("listening off: text input only, no whisper model loaded");
        Stt::disabled()
    } else {
        Stt::load(&config.whisper_model_path, config.whisper_gpu)?
    };
    // The tool registry, STT prompt, and checklists are all derived from the
    // profile inside run_session — and rebuilt there when the aircraft switches.

    // Push-to-talk, if configured. When set it takes over capture: the copilot
    // listens only while the button/key is held, so there's no wake word and
    // ambient/ATC can't trigger it. A sim dataref (the bundled X-Plane plugin)
    // takes precedence over a global key — it needs no OS permission.
    let ptt = if listen_off {
        None
    } else if !config.ptt_dataref.is_empty() {
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
    // The EFB's hold-to-talk button opens a push-to-talk capture only.
    efb::ptt::set_available(ptt.is_some());
    // Continuous only when not using PTT. "Explicit" = the user clearly addressed
    // the copilot (any push-to-talk); a non-command is then ignored silently rather
    // than answered with an apology.
    let continuous = config.listen_mode == "continuous" && ptt.is_none() && !listen_off;
    let explicit = !continuous;

    // The speech arbiter is shared by the command loop and the monitor, so it's
    // created here (above both) and handed to each.
    let voice = Voice::new(config.clone());

    // The panel's copilot-voice picker: publish the roster, poll for picks,
    // swap live. Only where the pick can matter — the `say` backend ignores
    // piper models entirely.
    if config.tts_backend != "say" && config.tts_backend != "off" {
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
                            let row = match party_line::active_station_in_range(
                                lat, lon, alt_ft,
                            ) {
                                Some((airport, services)) => {
                                    radio_panel_line(&airport, &services, com1)
                                }
                                // Out of range (or no station yet): the row keeps
                                // showing where the radio actually is, nothing more.
                                None => radio_panel_line("", &[], com1),
                            };
                            sink.radio(&row);
                            sink.atc(&atc_panel_payload(&party_line::fields_in_range(
                                lat, lon, alt_ft, 8,
                            )));
                            // The Calls tab's live "say now" line: the coach's
                            // suggested next call for the current tower state.
                            let on_ground = xp
                                .get_float(datarefs::ON_GROUND_ANY)
                                .await
                                .map(|v| v > 0.5)
                                .unwrap_or(true);
                            let callsign = crate::crew_radio::crew_callsign(&xp)
                                .await
                                .unwrap_or_else(|| "your callsign".to_string());
                            let dist = party_line::active_field()
                                .map(|(_, flat, flon, _)| nav::haversine_nm(lat, lon, flat, flon));
                            match party_line::suggest_call(&callsign, on_ground, dist, com1) {
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

    // The safe hold: an airborne aircraft nobody flies is never left flying
    // into the ground, from the first second of the process (a restart
    // mid-flight is exactly when nothing else holds it, LEMG 2026-10-05).
    // Its own thread, never stood down, independent of the monitor switch.
    safe_hold::spawn(
        config.clone(),
        live_profile.clone(),
        voice.clone(),
        status_sink.clone(),
    );

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
        // The EFB's lines ride the same channel (off unless XCOPILOT_EFB).
        efb::spawn(voice.clone(), cmd_tx.clone(), false);
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
            "\n  XFlightCrew — {} ({}). Ctrl-C to quit.\n",
            profile.display_name, config.listen_mode
        );
        let mut reporter = Reporter::plain();
        if let Some(s) = status_sink {
            reporter = reporter.with_status(s);
        }
        // No typed input in plain mode, but the panel's emergency buttons still
        // need a way in — same channel, same consumer.
        let (cmd_tx, cmd_rx) = std::sync::mpsc::channel::<String>();
        efb::spawn(voice.clone(), cmd_tx.clone(), true);
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
    /// The EFB's hold-to-talk button is this key too (`efb::ptt`): it opens
    /// the mic exactly as the plugin's does, and lapses on its own.
    fn intercom_pressed(&self) -> bool {
        efb::ptt::held()
            || match self {
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

// The command loop's stages, one file each (`src/main/session.rs` names
// them and runs them in order).
#[path = "main/session.rs"]
mod session;
#[path = "main/take.rs"]
mod take;
#[path = "main/station_work.rs"]
mod station_work;
#[path = "main/route.rs"]
mod route;
#[path = "main/dispatch.rs"]
mod dispatch;
#[path = "main/answer.rs"]
mod answer;
#[path = "main/speak.rs"]
mod speak;
use session::{Line, Session, Utterance};
use speak::TurnRecord;

/// The command loop: the session's state built, then its lines taken one
/// at a time through the stages (`session::Session::run`).
#[allow(clippy::too_many_arguments)]
async fn run_session(
    config: Config,
    stt: Stt,
    live_profile: std::sync::Arc<LiveProfile>,
    voice: Voice,
    continuous: bool,
    explicit: bool,
    ptt: Option<Ptt>,
    reporter: Reporter,
    typed: Option<std::sync::mpsc::Receiver<String>>,
    cabin_state: std::sync::Arc<std::sync::Mutex<cabin::Cabin>>,
) -> Result<()> {
    let session = Session::start(
        config,
        stt,
        live_profile,
        voice,
        continuous,
        explicit,
        ptt,
        reporter,
        typed,
        cabin_state,
    )?;
    session.run().await
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

/// Transcribe an audio file with whisper, for `xfc transcribe <file>`.
/// Decodes/resamples to 16 kHz mono via macOS `afconvert` (handles mp3/m4a/wav/…
/// — whisper needs 16 kHz mono), reads the PCM, and returns the text. Reuses the
/// same STT model the live loop uses, so no extra model download.
/// Decode any audio file to 16 kHz mono f32 samples via `afconvert` (macOS).
fn load_wav_16k(input: &str) -> Result<Vec<f32>> {
    use std::process::Command;
    if !std::path::Path::new(input).exists() {
        return Err(anyhow!("no such file: {input}"));
    }
    let wav = std::env::temp_dir().join("xflightcrew-16k.wav");
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

/// `xfc paths`: the mode, both roots, the models, the log and the prefs,
/// each absolute (`sim::paths::shown`).
fn paths_report() -> String {
    use sim::paths;
    let r = paths::roots();
    let mut out = format!("mode:       {}\n", r.mode.as_str());
    out += &format!("resources:  {}\n", paths::shown(&r.resources));
    if let Some(missing) = paths::missing_line() {
        out += &format!("            ({missing})\n");
    }
    out += &format!("data:       {}\n", paths::shown(&r.data));
    out += &format!("models:     {}\n", paths::shown(&config::models_dir()));
    out += &format!(
        "log:        {} (daily, dated suffix)\n",
        paths::shown(&paths::logs_dir().join(paths::LOG_FILE_NAME))
    );
    out += &format!("blackbox:   {}\n", paths::shown(&paths::blackbox_file()));
    out += &format!("prefs:      {}\n", paths::shown(&paths::prefs_file()));
    out += &format!("env:        {}\n", paths::shown(&paths::env_file()));
    out
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

    let mut paths: Vec<std::path::PathBuf> = std::fs::read_dir(sim::paths::logs_dir())
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
                // Logs from before the 2026-10-13 rename keep their old name.
                (n.starts_with("xflightcrew.log") || n.starts_with("xvoicelayer.log"))
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
        let (mut still, mut questions, mut contextual, mut fixed) =
            (Vec::new(), Vec::new(), Vec::new(), 0u32);
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
            } else if pre_grammar_capture(&m, &registry, &profile).is_some()
                || state_capture_note(&m).is_some()
                || grammar::load_aircraft_request(&m)
                || grammar::waiver_request(&m)
            {
                // These are deterministic command-loop branches that run
                // before the grammar. They used to remain in the miss tail
                // even though the live app handled them (`load flight plan`
                // and every repaired STT variant were the loudest example).
                fixed += n;
            } else if correction_value(&m).is_some() || looks_like_followup(&m) {
                // A replay has no dialogue stack or last numeric subject.
                // Report these separately instead of teaching a bare reply
                // to actuate without the context that makes it safe.
                contextual.push((t.clone(), *n));
            } else {
                still.push((t.clone(), *n));
            }
        }
        still.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        questions.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        contextual.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        println!("\nreplayed through today's grammar: {fixed} of {miss} now handled");
        println!("\nstill missed, commands ({}):", still.len());
        for (t, n) in &still {
            println!("  {n:>3}  {t}");
        }
        println!("\nquestions (answered from the live sim, not checked here) ({}):", questions.len());
        for (t, n) in &questions {
            println!("  {n:>3}  {t}");
        }
        println!("\ncontext-dependent follow-ups (need live dialogue state) ({}):", contextual.len());
        for (t, n) in &contextual {
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

/// Text only: nothing is captured (`XCOPILOT_LISTEN=off`).
static LISTEN_OFF: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

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
    if LISTEN_OFF.load(std::sync::atomic::Ordering::Relaxed) {
        // Never Enter-to-talk: wait out the caller's tick so the typed
        // channel is polled at the same pace.
        std::thread::sleep(idle_timeout.unwrap_or(Duration::from_millis(200)));
        return None;
    }
    let mut from_radio = false;
    let audio = match (ptt, ptt_mic, listener) {
        // Hold-a-key push-to-talk (works while X-Plane has focus), warm mic.
        (Some(Ptt::Key(key)), Some(mic), _) => {
            use device_query::DeviceQuery;
            let dev = device_query::DeviceState::new();
            let key = *key;
            mic.capture(
                || efb::ptt::held() || dev.get_keys().contains(&key),
                idle_timeout,
            )
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
                        let intercom = Ptt::dataref_active(live, name) || efb::ptt::held();
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

/// The ended flight's or airframe's readback: "say again" never repeats
/// it into the next (the swap and `flight_state::forget_flight`).
fn forget_last_readback() {
    *LAST_READBACK.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

fn say(voice: &Voice, reporter: &Reporter, kind: ui::LogKind, line: &str) {
    say_or(voice, reporter, kind, line, || loop_owed(false));
}

/// The live facts a queued crew call is judged against (#143), read together.
/// A failed read is `None`: no measurement, never a reason to drop a call.
async fn crew_radio_facts(xp: &XPlaneClient) -> crew_radio::RadioFacts {
    let (ground, agl, gs) = tokio::join!(
        xp.get_float(datarefs::ON_GROUND_ANY),
        xp.get_float(datarefs::AGL_METERS),
        xp.get_float(autotaxi::GS_MS),
    );
    crew_radio::RadioFacts {
        on_ground: ground.ok().map(|v| v > 0.5),
        agl_ft: agl.ok().map(|m| m * 3.28084),
        gs_kts: gs.ok().map(|ms| ms.abs() * 1.943_844),
        // Only an autoland flies an approach that can be established;
        // otherwise unknown, never "not on final".
        on_final: autoland::ACTIVE
            .load(std::sync::atomic::Ordering::SeqCst)
            .then(autoland::established_on_final),
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
fn install_home_station(config: &Config, voice: &voice::Voice) -> Option<party_line::Station> {
    if config.atc_airport.trim().is_empty() {
        return None;
    }
    let home_icao = config.atc_airport.trim().to_uppercase();
    let mut st = party_line::Station::load(&config.apt_data_path, &home_icao)?;
    st.set_enforce_readback(config.atc_readback);
    // Phraseology follows geography, like the voices: CAP 413 grammar in
    // the UK, FAA grammar at a K/P/C field.
    st.set_dialect(party_line::dialect_for(&home_icao));
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
    party_line::spawn_taxi_net_load(config.apt_data_path.clone(), home_icao.clone());
    // ILS + real approach platforms (earth_nav.dat + CIFP scans, also
    // backgrounded): vectors offer the ILS once this lands.
    party_line::spawn_ils_load(
        config.nav_data_path.clone(),
        config.cifp_dir.clone(),
        home_icao,
    );
    // Enroute region: index every towered field (a full apt.dat pass, so a
    // background thread) so the flight can outgrow its departure airport —
    // the monitor's region_tick hands off to whichever tower is nearest.
    if config.atc_region.trim() != "off" && !party_line::region_loaded() {
        let path = config.apt_data_path.clone();
        std::thread::Builder::new()
            .name("region-index".into())
            .spawn(move || party_line::load_region(&path))
            .ok();
    }
    Some(st)
}

// Moved to `shortcuts.rs` (phase 2): the shim keeps every caller's path.
pub(crate) use shortcuts::{levenshtein, question_shortcut, Shortcut};


/// Where the command loop takes a TYPED line before the grammar, on its
/// words alone: the undo, the lesson and checklist words, a fresh
/// conversation's answers, the [`question_shortcut`]s, the cabin, the taxi
/// question, the radio coach, the recall and the question opener (a
/// question never reaches a tool). What only a running flight decides (a
/// flow's reply, an offer's yes, the release guard, a tuned tower) is not
/// here. `None`: the line reaches the grammar. Pure.
pub(crate) fn pre_grammar_capture(
    line: &str,
    registry: &Registry,
    profile: &aircraft::AircraftProfile,
) -> Option<&'static str> {
    let tl = line.to_ascii_lowercase();
    if is_undo(line) {
        return Some("undo");
    }
    if flow::instruct_request(line).is_some() {
        return Some("checklist instruct");
    }
    if instructor::request(line).is_some() {
        return Some("instructor");
    }
    if rapport::arrival_brief_request(line) {
        return Some("arrival brief");
    }
    if conversation::Conversation::default()
        .respond(line, Instant::now())
        .is_some()
    {
        return Some("conversation");
    }
    let shortcut = question_shortcut(line, registry, profile, None);
    if shortcut == Some(Shortcut::Fuel) {
        return shortcut.map(Shortcut::name);
    }
    if cabin::Cabin::addressed(line).is_some() {
        return Some("cabin call");
    }
    if is_taxi_query(line) && grammar::try_parse(line, registry, profile).is_none() {
        return Some("taxi question");
    }
    if party_line::freq_query(&tl) {
        return Some("frequency briefing");
    }
    if party_line::coach_query(&tl) {
        return Some("radio coach");
    }
    if let Some(s) = shortcut {
        return Some(s.name());
    }
    let matchable = grammar::normalize(line);
    if atc_recall_request(&matchable) || recall_ask(&matchable).is_some() {
        return Some("recall");
    }
    if conversational(&matchable, registry).is_some() {
        return Some("conversational");
    }
    if location_answer(&matchable, profile).is_some() {
        return Some("location");
    }
    let parsed = grammar::try_parse(&matchable, registry, profile).is_some();
    let opener_q = looks_like_question(matchable.trim_end_matches(['?', '.', ' ']));
    if looks_like_question(&matchable) && (opener_q || !parsed) {
        return Some("state question");
    }
    None
}

/// What only a running flight decides, said beside a line's route in a
/// check: the release guard while a loop holds a control, a radio call
/// while a tower is tuned (no radio key). Pure.
pub(crate) fn state_capture_note(line: &str) -> Option<&'static str> {
    let tl = line.to_ascii_lowercase();
    let setup = grammar::approach_setup_request(&tl) || grammar::enroute_setup_request(&tl);
    if setup {
        return None;
    }
    if release_intent(&tl) {
        return Some("the release guard takes it while a loop holds a control");
    }
    if party_line::is_radio_call(&tl, "") && tower::parse_intent(&tl) != tower::Intent::Unknown {
        return Some("a tuned tower takes it as a radio call");
    }
    None
}

/// Content-based ATC routing for setups without a dedicated radio PTT. Physical
/// radio PTT is handled earlier; this is the one fallback door for keyboard PTT
/// and continuous mode, shared by normal crew mode and ATC-only mode.
async fn try_route_content_atc(
    st: &mut party_line::Station,
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
        && party_line::is_radio_call(
            tl,
            &st.radio_name().to_lowercase(),
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
    // The station took the pilot's words: the EFB's transcript puts them on
    // the frequency, not the intercom.
    efb::feed::pilot_line_was_radio();
    // The pilot's own "going around" to the tower is flown too.
    go_around::note_radio_call(tl);
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

async fn void_briefing(xp: &XPlaneClient, st: &party_line::Station) -> String {
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
    if party_line::active_station_in_range(lat, lon, alt_ft).is_some() {
        return st.void_note(com1, airborne);
    }
    let near = party_line::fields_in_range(lat, lon, alt_ft, 1);
    match near.first().and_then(|f| {
        f.services
            .iter()
            .find(|(w, _)| w.as_str() == "tower")
            .map(|(_, khz)| (f, *khz))
    }) {
        Some((f, khz)) => format!(
            "{} is out of radio range. Nearest station is {}, {} miles away, tower on {}.",
            party_line::spoken_airport(st.airport_name()),
            party_line::spoken_airport(&f.name),
            f.dist_nm.round() as i64,
            atis::spoken_mhz(khz)
        ),
        None => format!(
            "{} is out of radio range, and nothing else is in reach from here.",
            party_line::spoken_airport(st.airport_name())
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
fn atc_panel_payload(fields: &[party_line::RangedField]) -> String {
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

#[path = "main/tests.rs"]
#[cfg(test)]
mod tests;

#[path = "main/gap_tests_2026_09_12_tests.rs"]
#[cfg(test)]
mod gap_tests_2026_09_12;

#[path = "main/scratch_probe_tests.rs"]
#[cfg(test)]
mod scratch_probe;

#[path = "main/shortcut_tests.rs"]
#[cfg(test)]
mod shortcut_tests;
