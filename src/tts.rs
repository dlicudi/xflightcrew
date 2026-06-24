//! Text-to-speech readback — the copilot's voice, confirming the *actual* action.
//!
//! Two local backends, selected by `config.tts_backend`:
//!   - `say`   — macOS built-in. Zero install, always works. The default.
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
        "say" => speak_say(text),
        "piper" => speak_piper(config, text),
        other => bail!("unknown tts_backend: {other} (expected `say` or `piper`)"),
    }
}

/// macOS built-in speech synthesizer. Reads the text directly — no WAV step.
fn speak_say(text: &str) -> Result<()> {
    let status = Command::new("say")
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
