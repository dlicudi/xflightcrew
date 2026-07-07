//! Text-to-speech readback — the copilot's voice, confirming the *actual* action.
//!
//! The default backend is the warm Spark voice-clone `sidecar`
//! ([`crate::tts_sidecar`]); this module covers the subprocess fallbacks selected
//! by `config.tts_backend`:
//!   - `say`   — macOS built-in. Zero install; the fallback when the sidecar
//!     can't start.
//!   - `piper` — shell out to the piper binary + a voice model (Horizon's trick).
//!
//! A later iteration could run Kokoro ONNX in-process via the `ort` crate.

use std::io::Write;
use std::process::{Command, Stdio};

use anyhow::{bail, Context, Result};

use crate::config::Config;

/// Speak `text` aloud using the configured backend.
pub fn speak(config: &Config, text: &str) -> Result<()> {
    match config.tts_backend.as_str() {
        "say" => speak_say(config, text),
        "piper" => speak_piper(config, text),
        other => bail!("unknown tts_backend: {other} (expected `say` or `piper`)"),
    }
}

/// Ceiling on any speech subprocess. A `say`/`afplay` that never exits would
/// otherwise block the speaker thread forever — with the mic gate raised, so
/// the app goes deaf too (seen live 2026-07-07: one hung `say` froze the whole
/// loop, unkillable). Generous: the longest legitimate utterance (a full ATIS)
/// is well under a minute.
const SPEECH_WATCHDOG: std::time::Duration = std::time::Duration::from_secs(90);

/// Wait for a speech child process, killing it if it overruns the watchdog.
fn wait_with_watchdog(mut child: std::process::Child, what: &str) -> Result<()> {
    let start = std::time::Instant::now();
    loop {
        match child.try_wait().with_context(|| format!("polling {what}"))? {
            Some(status) if status.success() => return Ok(()),
            Some(status) => bail!("{what} exited with {status}"),
            None if start.elapsed() > SPEECH_WATCHDOG => {
                let _ = child.kill();
                let _ = child.wait();
                bail!("{what} hung past {}s; killed it", SPEECH_WATCHDOG.as_secs());
            }
            None => std::thread::sleep(std::time::Duration::from_millis(50)),
        }
    }
}

/// macOS built-in speech synthesizer. Reads the text directly — no WAV step.
/// `XCOPILOT_SAY_VOICE` picks the voice (`say -v '?'` lists them); empty = the
/// system default. `XCOPILOT_SAY_RATE` sets words-per-minute (empty = default).
fn speak_say(config: &Config, text: &str) -> Result<()> {
    let mut cmd = Command::new("say");
    if !config.say_voice.is_empty() {
        cmd.arg("-v").arg(&config.say_voice);
    }
    if !config.say_rate.is_empty() {
        cmd.arg("-r").arg(&config.say_rate);
    }
    let child = cmd
        .arg(text)
        .spawn()
        .context("spawning `say` (macOS only)")?;
    wait_with_watchdog(child, "say")
}

/// Pipe `text` to piper to synthesize a WAV, then play it.
fn speak_piper(config: &Config, text: &str) -> Result<()> {
    let wav_path = std::env::temp_dir().join("x-copilot-readback.wav");

    let mut child = Command::new(&config.piper_binary)
        .arg("--model")
        .arg(&config.piper_voice)
        .arg("--output_file")
        .arg(&wav_path)
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .spawn()
        .with_context(|| format!("spawning piper ({})", config.piper_binary))?;

    child
        .stdin
        .as_mut()
        .context("piper stdin unavailable")?
        .write_all(text.as_bytes())
        .context("writing text to piper")?;

    let status = child.wait().context("waiting for piper")?;
    if !status.success() {
        bail!("piper exited with {status}");
    }

    play(&wav_path)
}

/// Kill any in-progress playback (and `say`) right now — the barge-in path. Every
/// backend ends up at `afplay` (or `say`), so terminating those stops the audio
/// whatever the TTS engine. Best-effort: errors (nothing playing) are ignored.
#[cfg(target_os = "macos")]
pub fn stop_playback() {
    let _ = Command::new("pkill").args(["-x", "afplay"]).status();
    let _ = Command::new("pkill").args(["-x", "say"]).status();
}

#[cfg(not(target_os = "macos"))]
pub fn stop_playback() {
    let _ = Command::new("pkill").args(["-x", "aplay"]).status();
}

/// Play a WAV file using the platform's default CLI player. `pub(crate)` so the
/// in-process ONNX backend ([`crate::tts_onnx`]) can reuse it.
#[cfg(target_os = "macos")]
pub(crate) fn play(path: &std::path::Path) -> Result<()> {
    let child = Command::new("afplay").arg(path).spawn().context("spawning afplay")?;
    wait_with_watchdog(child, "afplay")
}

#[cfg(not(target_os = "macos"))]
pub(crate) fn play(path: &std::path::Path) -> Result<()> {
    // Linux fallback: aplay ships with alsa-utils.
    let child = Command::new("aplay").arg(path).spawn().context("spawning aplay")?;
    wait_with_watchdog(child, "aplay")
}
