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
    let status = cmd
        .arg(text)
        .status()
        .context("spawning `say` (macOS only)")?;
    if !status.success() {
        bail!("say exited with {status}");
    }
    Ok(())
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
    let status = Command::new("afplay")
        .arg(path)
        .status()
        .context("spawning afplay")?;
    if !status.success() {
        bail!("afplay exited with {status}");
    }
    Ok(())
}

#[cfg(not(target_os = "macos"))]
pub(crate) fn play(path: &std::path::Path) -> Result<()> {
    // Linux fallback: aplay ships with alsa-utils.
    let status = Command::new("aplay")
        .arg(path)
        .status()
        .context("spawning aplay")?;
    if !status.success() {
        bail!("aplay exited with {status}");
    }
    Ok(())
}
