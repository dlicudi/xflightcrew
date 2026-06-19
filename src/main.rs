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
mod xplane;

use anyhow::{anyhow, Result};

use audio::Listener;
use config::Config;
use llm::Llm;
use stt::Stt;
use tools::{Registry, ToolCtx};
use xplane::XPlaneClient;

// Single-threaded runtime: the cpal stream isn't `Send`, and the loop is
// inherently sequential (listen → think → act → speak), so there's nothing to
// gain from a multi-threaded scheduler.
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    use tracing_subscriber::prelude::*;
    // Log to stderr (live) and to a daily-rolling file in logs/ (reviewable).
    let file_appender = tracing_appender::rolling::daily("logs", "x-copilot.log");
    let (file_writer, _log_guard) = tracing_appender::non_blocking(file_appender);
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| "x_copilot=info".into());
    tracing_subscriber::registry()
        .with(filter)
        .with(tracing_subscriber::fmt::layer().with_writer(std::io::stderr))
        .with(tracing_subscriber::fmt::layer().with_ansi(false).with_writer(file_writer))
        .init();

    let config = Config::from_env()?;

    // Diagnostic mode: `cargo run -- mic-check` prints live mic levels and exits
    // on Ctrl-C. Use it to confirm capture and tune XCOPILOT_VAD_RMS.
    if std::env::args().nth(1).as_deref() == Some("mic-check") {
        return audio::mic_check(config.vad_threshold);
    }

    let xplane = XPlaneClient::new(config.xplane_base_url.clone());

    // Resolve the aircraft profile: auto-detect from X-Plane's acf_ICAO when
    // configured as "auto", otherwise use the configured id. Falls back to the
    // Cessna if detection fails (e.g. X-Plane not reachable yet).
    let aircraft_id = if config.aircraft.eq_ignore_ascii_case("auto") {
        match xplane.aircraft_icao().await {
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
    let specs = registry.specs();

    let continuous = config.listen_mode == "continuous";
    // "Explicit" = the user clearly addressed the copilot (push-to-talk, or a
    // wake word that matched). When not explicit, a non-command is ignored
    // silently rather than answered with an apology.
    let explicit = !continuous || !config.wake_word.is_empty();
    let mut listener = if continuous {
        Some(Listener::start(config.vad_threshold)?)
    } else {
        None
    };

    if continuous {
        if config.wake_word.is_empty() {
            println!("\n  x-copilot listening — {}. Just speak commands (e.g. \"flaps ten\", \"landing light on\"). Ctrl-C to quit.\n", profile.display_name);
        } else {
            println!(
                "\n  x-copilot listening — {}. Say \"{}, <command>\". Ctrl-C to quit.\n",
                profile.display_name, config.wake_word
            );
        }
    } else {
        println!("\n  x-copilot ready — {}. Ctrl-C to quit.\n", profile.display_name);
    }

    loop {
        // 1. Capture an utterance (continuous: VAD-segmented; ptt: between
        //    Enters) and transcribe it (with a confidence score).
        let heard = match listen_once(&mut listener, &stt) {
            Some(t) => t,
            None => continue,
        };
        let transcript = heard.text;
        let confidence = heard.confidence;
        tracing::info!(transcript = %transcript, confidence, "heard utterance");
        let marker = if continuous { "👂 heard" } else { "🗣️ " };
        println!("{marker}: \"{transcript}\" (confidence {confidence:.2})");

        // 2. Confidence gate. Low confidence = likely noise or a half-caught
        //    word — re-ask instead of guessing it into a command.
        if confidence < config.stt_min_confidence {
            if explicit {
                speak_line(&config, "Say again?");
            } else {
                println!("   ↳ low confidence, ignoring\n");
            }
            continue;
        }

        // 3. Wake-word gate (continuous only). Only act on commands addressed to
        //    the copilot; everything else is overheard chatter.
        let command = if continuous {
            match gate_wake_word(&transcript, &config.wake_word) {
                Some(cmd) if !cmd.is_empty() => cmd,
                _ => {
                    println!("   ↳ ignored (say \"{}, <command>\")\n", config.wake_word);
                    continue;
                }
            }
        } else {
            transcript.clone()
        };

        // 4. Local LLM picks a tool.
        let mut call = match llm.choose_tool(&command, &specs).await {
            Ok(Some(call)) => call,
            Ok(None) => {
                if explicit {
                    speak_line(&config, "Sorry, I didn't catch a command I can action.");
                } else {
                    tracing::info!(command = %command, "no matching tool — ignored");
                }
                continue;
            }
            Err(e) => {
                tracing::error!("LLM tool selection failed: {e:#}");
                continue;
            }
        };
        tracing::info!(tool = %call.name, args = %call.arguments, "tool selected");

        // 5. Confirmation gate with in-line correction. For safety-critical
        //    commands, read back the interpreted intent and require "affirm".
        //    A non-affirmative reply that carries a new value ("negative, 500")
        //    is re-interpreted and re-confirmed; a plain "negative" cancels.
        let proceed = 'confirm: loop {
            let Some(tool) = registry.get(&call.name) else {
                tracing::warn!("LLM picked unknown tool: {}", call.name);
                speak_line(&config, "That command isn't wired up yet.");
                break 'confirm false;
            };
            let intent = if config.confirm_critical {
                tool.confirm_prompt(&call.arguments)
            } else {
                None
            };
            let Some(intent) = intent else { break 'confirm true };

            speak_line(&config, &format!("Confirm, {intent}?"));
            let Some(reply) = listen_once(&mut listener, &stt) else {
                speak_line(&config, "Cancelled, no confirmation heard.");
                break 'confirm false;
            };
            if is_affirmative(&reply.text) {
                println!("✅ confirmed: \"{}\"", reply.text);
                tracing::info!(intent = %intent, reply = %reply.text, "confirmed");
                break 'confirm true;
            }
            // Not a yes — does the reply carry a corrected value?
            match correction_text(&reply.text) {
                Some(correction) => {
                    println!("↻ correction: \"{correction}\"");
                    tracing::info!(intent = %intent, correction = %correction, "correcting");
                    match llm.choose_tool(&format!("{command}, {correction}"), &specs).await {
                        Ok(Some(c)) => {
                            call = c;
                            continue 'confirm; // re-confirm the corrected command
                        }
                        _ => {
                            speak_line(&config, "Cancelled.");
                            break 'confirm false;
                        }
                    }
                }
                None => {
                    tracing::info!(intent = %intent, reply = %reply.text, "cancelled");
                    speak_line(&config, "Cancelled.");
                    break 'confirm false;
                }
            }
        };
        if !proceed {
            continue;
        }

        // 6. Execute the curated tool (writes the dataref + reads back).
        let Some(tool) = registry.get(&call.name) else { continue };
        let ctx = ToolCtx {
            xplane: &xplane,
            config: &config,
        };
        let readback = match tool.execute(&call.arguments, &ctx).await {
            Ok(line) => line,
            Err(e) => {
                tracing::error!("tool {} failed: {e:#}", call.name);
                "Something went wrong actioning that.".to_string()
            }
        };
        speak_line(&config, &readback);
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

/// Print and speak a readback line; TTS failures are logged, not fatal.
fn speak_line(config: &Config, line: &str) {
    tracing::info!(readback = %line, "spoke");
    println!("🔊 {line}\n");
    if let Err(e) = tts::speak(config, line) {
        tracing::warn!("TTS failed: {e:#}");
    }
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
