//! x-copilot (working name; product name: Roger)
//!
//! The voice loop, end to end and fully local:
//!   listen → whisper (STT) → local LLM picks one tool →
//!   write dataref + read back → say/piper (TTS) confirms.
//!
//! Two capture modes (config `listen_mode`): `continuous` (always listening,
//! VAD-segmented, gated by a wake word) or `ptt` (push-to-talk).

mod aircraft;
mod audio;
mod config;
mod llm;
mod nav;
mod stt;
mod tts;
mod tools;
mod ui;
mod xplane;

use anyhow::{anyhow, Result};

use audio::Listener;
use config::Config;
use llm::Llm;
use stt::Stt;
use tools::{Registry, ToolCtx};
use ui::Reporter;
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

    // whisper model load is the one slow init — do it once, up front.
    let stt = Stt::load(&config.whisper_model_path)?;
    let llm = Llm::new(&config);
    let registry = Registry::for_aircraft(&profile);

    let continuous = config.listen_mode == "continuous";
    // "Explicit" = the user clearly addressed the copilot (push-to-talk, or a
    // wake word that matched). When not explicit, a non-command is ignored
    // silently rather than answered with an apology.
    let explicit = !continuous || !config.wake_word.is_empty();

    if config.ui == "tui" {
        // TUI: voice loop + aircraft poll run on background threads, each with
        // its own runtime; the dashboard renders on the main thread.
        let (tx, rx) = std::sync::mpsc::channel::<ui::Report>();
        let title = format!("x-copilot · {}", profile.display_name);
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

        let reporter = Reporter::Tui(tx);
        std::thread::spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else { return };
            if let Err(e) =
                rt.block_on(run_session(config, stt, llm, registry, continuous, explicit, reporter))
            {
                tracing::error!("voice loop ended: {e:#}");
            }
        });

        ui::run_dashboard(title, commands, rx)?; // blocks until the user quits
        Ok(())
    } else {
        println!("\n  x-copilot — {} ({}). Ctrl-C to quit.\n", profile.display_name, config.listen_mode);
        run_session(config, stt, llm, registry, continuous, explicit, Reporter::Plain).await
    }
}

/// The voice loop: listen → transcribe → pick a tool → confirm → execute → read
/// back. Progress is emitted through `reporter` (plain stdout or the TUI), so
/// the same code drives both UIs.
async fn run_session(
    config: Config,
    stt: Stt,
    llm: Llm,
    registry: Registry,
    continuous: bool,
    explicit: bool,
    reporter: Reporter,
) -> Result<()> {
    // Build the X-Plane client here, on whatever runtime this session runs on
    // (main for plain, the voice thread for TUI) — a reqwest client's I/O is
    // bound to the runtime it's used on.
    let xplane = XPlaneClient::new(config.xplane_base_url.clone());
    let specs = registry.specs();
    let mut listener = if continuous {
        Some(Listener::start(config.vad_threshold, reporter.clone())?)
    } else {
        None
    };

    reporter.log(
        ui::LogKind::Info,
        if !continuous {
            "Ready — push-to-talk (Enter to start/stop).".to_string()
        } else if config.wake_word.is_empty() {
            "Listening — just speak commands.".to_string()
        } else {
            format!("Listening — say \"{}, <command>\".", config.wake_word)
        },
    );

    loop {
        reporter.status(ui::Phase::Listening);
        let heard = match listen_once(&mut listener, &stt) {
            Some(t) => t,
            None => continue,
        };
        let transcript = heard.text;
        let confidence = heard.confidence;
        tracing::info!(transcript = %transcript, confidence, "heard utterance");
        reporter.log(ui::LogKind::Heard, format!("\"{transcript}\" ({confidence:.2})"));

        // Confidence gate.
        if confidence < config.stt_min_confidence {
            if explicit {
                say(&config, &reporter, ui::LogKind::Info, "Say again?");
            } else {
                reporter.log(ui::LogKind::Info, "low confidence, ignored");
            }
            continue;
        }

        // Wake-word gate (continuous only).
        let command = if continuous {
            match gate_wake_word(&transcript, &config.wake_word) {
                Some(cmd) if !cmd.is_empty() => cmd,
                _ => {
                    reporter.log(ui::LogKind::Info, "ignored (no wake word)");
                    continue;
                }
            }
        } else {
            transcript.clone()
        };

        // LLM picks a tool.
        reporter.status(ui::Phase::Thinking);
        let mut call = match llm.choose_tool(&command, &specs).await {
            Ok(Some(call)) => call,
            Ok(None) => {
                if explicit {
                    say(&config, &reporter, ui::LogKind::Warn, "Sorry, I didn't catch a command I can action.");
                } else {
                    reporter.log(ui::LogKind::Info, format!("no matching tool: \"{command}\""));
                }
                continue;
            }
            Err(e) => {
                tracing::error!("LLM tool selection failed: {e:#}");
                reporter.log(ui::LogKind::Warn, "LLM unreachable");
                continue;
            }
        };
        tracing::info!(tool = %call.name, args = %call.arguments, "tool selected");
        reporter.log(ui::LogKind::Tool, format!("{} {}", call.name, call.arguments));

        // Confirmation gate with in-line correction ("negative, 500").
        let proceed = 'confirm: loop {
            let Some(tool) = registry.get(&call.name) else {
                tracing::warn!("LLM picked unknown tool: {}", call.name);
                say(&config, &reporter, ui::LogKind::Warn, "That command isn't wired up yet.");
                break 'confirm false;
            };
            let intent = if config.confirm_critical {
                tool.confirm_prompt(&call.arguments)
            } else {
                None
            };
            let Some(intent) = intent else { break 'confirm true };

            say(&config, &reporter, ui::LogKind::Confirm, &format!("Confirm, {intent}?"));
            reporter.status(ui::Phase::Listening);
            let Some(reply) = listen_once(&mut listener, &stt) else {
                say(&config, &reporter, ui::LogKind::Warn, "Cancelled, no confirmation heard.");
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
                    match llm.choose_tool(&format!("{command}, {correction}"), &specs).await {
                        Ok(Some(c)) => {
                            call = c;
                            reporter.log(ui::LogKind::Tool, format!("{} {}", call.name, call.arguments));
                            continue 'confirm;
                        }
                        _ => {
                            say(&config, &reporter, ui::LogKind::Warn, "Cancelled.");
                            break 'confirm false;
                        }
                    }
                }
                None => {
                    say(&config, &reporter, ui::LogKind::Warn, "Cancelled.");
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
        let ctx = ToolCtx { xplane: &xplane, config: &config };
        let readback = match tool.execute(&call.arguments, &ctx).await {
            Ok(line) => line,
            Err(e) => {
                tracing::error!("tool {} failed: {e:#}", call.name);
                "Something went wrong actioning that.".to_string()
            }
        };
        say(&config, &reporter, ui::LogKind::Readback, &readback);
    }
}

/// Apply the wake-word gate, fuzzily. Speech-to-text mangles "copilot" into
/// "go pilot", "co-pilot", "copilet", etc., so we match the wake word against
/// the first few tokens (1- or 2-token spans) with a small edit-distance
/// tolerance, then return the rest as the command. Empty `wake` disables it.
fn gate_wake_word(transcript: &str, wake: &str) -> Option<String> {
    if wake.is_empty() {
        return Some(transcript.trim().to_string());
    }
    let wake = collapse(wake);
    let orig: Vec<&str> = transcript.split_whitespace().collect();
    let norm: Vec<String> = orig.iter().map(|w| collapse(w)).collect();

    // Scan the first few words: the wake word may be 1 or 2 tokens ("copilot"
    // vs "go pilot") and may follow filler ("hey copilot ...").
    for s in 0..norm.len().min(3) {
        for (take, max_dist) in [(2usize, 2usize), (1usize, 1usize)] {
            if s + take > norm.len() {
                continue;
            }
            let cand: String = norm[s..s + take].concat();
            if !cand.is_empty() && levenshtein(&cand, &wake) <= max_dist {
                let rest = orig[s + take..].join(" ");
                return Some(
                    rest.trim_start_matches(|c: char| !c.is_alphanumeric())
                        .trim()
                        .to_string(),
                );
            }
        }
    }
    None
}

/// Lowercase and strip non-alphanumerics (drops punctuation whisper adds).
fn collapse(s: &str) -> String {
    s.chars()
        .filter(|c| c.is_alphanumeric())
        .flat_map(|c| c.to_lowercase())
        .collect()
}

/// Standard Levenshtein edit distance.
fn levenshtein(a: &str, b: &str) -> usize {
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

/// Map an X-Plane ICAO type code to a profile id. Unknown types fall back to
/// the Cessna so the app stays usable; add rows as profiles are added.
fn icao_to_profile(icao: &str) -> String {
    match icao.to_uppercase().as_str() {
        "C172" => "cessna_172sp",
        _ => "cessna_172sp",
    }
    .to_string()
}

/// Capture one utterance (continuous VAD or push-to-talk) and transcribe it.
/// Returns `None` on silence/noise or a capture error.
fn listen_once(listener: &mut Option<Listener>, stt: &Stt) -> Option<stt::Transcription> {
    let audio = match listener {
        Some(l) => l.next_utterance().ok()?,
        None => audio::record_push_to_talk().ok()?,
    };
    if audio.is_empty() {
        return None;
    }
    match stt.transcribe(&audio) {
        Ok(t) if !t.text.is_empty() => Some(t),
        _ => None,
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
        "copy", "proceed", "go ahead", "do it", "right",
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

/// Report a line and speak it. Sets the Speaking→Listening status around the
/// (blocking) TTS call. Failures are logged, not fatal.
fn say(config: &Config, reporter: &Reporter, kind: ui::LogKind, line: &str) {
    tracing::info!(spoke = %line, "spoke");
    reporter.status(ui::Phase::Speaking);
    reporter.log(kind, line.to_string());
    if let Err(e) = tts::speak(config, line) {
        tracing::warn!("TTS failed: {e:#}");
    }
    reporter.status(ui::Phase::Listening);
}

/// Read the live aircraft state for the dashboard panel. Returns `None` if
/// X-Plane is unreachable (panel keeps its last values).
async fn poll_aircraft(xp: &XPlaneClient) -> Option<ui::Aircraft> {
    Some(ui::Aircraft {
        heading: xp.get_float("sim/cockpit2/gauges/indicators/compass_heading_deg_mag").await.ok()?,
        altitude: xp.get_float("sim/cockpit2/gauges/indicators/altitude_ft_pilot").await.ok()?,
        ias: xp.get_float("sim/cockpit2/gauges/indicators/airspeed_kts_pilot").await.ok()?,
        vsi: xp.get_float("sim/cockpit2/gauges/indicators/vvi_fpm_pilot").await.ok()?,
        flaps_pct: xp.get_float("sim/cockpit2/controls/flap_ratio").await.ok()? * 100.0,
        ap_mode: ap_mode(xp).await,
    })
}

/// Summarize the active autopilot lateral mode for the panel.
async fn ap_mode(xp: &XPlaneClient) -> String {
    if xp.get_float("sim/cockpit2/autopilot/servos_on").await.unwrap_or(0.0) <= 0.5 {
        return "--".to_string();
    }
    let appr = xp.get_float("sim/cockpit2/autopilot/approach_status").await.unwrap_or(0.0);
    let nav = xp.get_float("sim/cockpit2/autopilot/nav_status").await.unwrap_or(0.0);
    let hdg = xp.get_float("sim/cockpit2/autopilot/heading_status").await.unwrap_or(0.0);
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

/// Example commands shown in the dashboard's "say" panel.
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
    use super::gate_wake_word;

    #[test]
    fn strips_wake_word_and_punctuation() {
        assert_eq!(gate_wake_word("copilot flaps ten", "copilot").as_deref(), Some("flaps ten"));
        assert_eq!(gate_wake_word("Copilot, flaps full", "copilot").as_deref(), Some("flaps full"));
        assert_eq!(gate_wake_word("hey copilot — gear up", "copilot").as_deref(), Some("gear up"));
    }

    #[test]
    fn rejects_utterances_without_wake_word() {
        assert_eq!(gate_wake_word("descend to flight level one zero zero", "copilot"), None);
    }

    #[test]
    fn empty_wake_word_passes_everything() {
        assert_eq!(gate_wake_word("flaps ten", "").as_deref(), Some("flaps ten"));
    }

    use super::is_affirmative;

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
