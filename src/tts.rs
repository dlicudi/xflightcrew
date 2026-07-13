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

/// Barge-in bookkeeping: the audible child's pid, plus a cut generation the
/// killer bumps. Killing by REGISTERED PID (not `pkill` by name) means the
/// cut can't touch unrelated processes — and the generation closes the spawn
/// race: a PTT press landing in the few ms between "about to spawn" and
/// "spawned" used to be missed entirely, letting the whole transmission talk
/// over the pilot and then replay as a stepped-on tail.
static PLAYER_PID: std::sync::Mutex<Option<u32>> = std::sync::Mutex::new(None);
static CUT_GEN: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Spawn-to-exit wrapper for every AUDIBLE child (`say`, `afplay`, `aplay`):
/// registers the pid for [`stop_playback`], detects a cut that landed while
/// the process was spawning, and rides the hang watchdog. Only the speaker
/// thread plays audio, so one slot is the whole story.
fn run_audible(mut cmd: Command, what: &str) -> Result<()> {
    use std::sync::atomic::Ordering::SeqCst;
    let gen = CUT_GEN.load(SeqCst);
    let child = cmd.spawn().with_context(|| format!("spawning {what}"))?;
    *PLAYER_PID.lock().unwrap_or_else(|e| e.into_inner()) = Some(child.id());
    if CUT_GEN.load(SeqCst) != gen {
        kill_pid(child.id()); // the press won the race; honor it
    }
    let res = wait_with_watchdog(child, what);
    *PLAYER_PID.lock().unwrap_or_else(|e| e.into_inner()) = None;
    res
}

fn kill_pid(pid: u32) {
    let _ = Command::new("kill").arg(pid.to_string()).status();
}

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
    cmd.arg(text);
    run_audible(cmd, "say")
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

/// Kill the in-progress playback right now — the barge-in path. Bumps the cut
/// generation first, so even a child mid-spawn (registered a moment later)
/// dies immediately; then kills the registered pid. Best-effort: nothing
/// playing means nothing to do.
pub fn stop_playback() {
    CUT_GEN.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    if let Some(pid) = *PLAYER_PID.lock().unwrap_or_else(|e| e.into_inner()) {
        kill_pid(pid);
    }
}

/// Play mono f32 PCM through the platform player (write a temp WAV, play it).
/// Lives here rather than in `tts_onnx` so the speech arbiter's barge-in tail
/// path (replaying a sliced sample buffer) builds without the onnx feature.
#[allow(dead_code)] // consumed by the copilot binary's tree (voice.rs), not the lib's
pub(crate) fn play_pcm(samples: &[f32], sample_rate: u32) -> Result<()> {
    let path = std::env::temp_dir().join("x-copilot-pcm.wav");
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut w = hound::WavWriter::create(&path, spec)
        .with_context(|| format!("creating {}", path.display()))?;
    for &s in samples {
        w.write_sample((s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16)?;
    }
    w.finalize().context("finalizing wav")?;
    play(&path)
}

/// Play a WAV file using the platform's default CLI player. `pub(crate)` so the
/// in-process ONNX backend ([`crate::tts_onnx`]) can reuse it.
#[cfg(target_os = "macos")]
pub(crate) fn play(path: &std::path::Path) -> Result<()> {
    let mut cmd = Command::new("afplay");
    cmd.arg(path);
    run_audible(cmd, "afplay")
}

#[cfg(not(target_os = "macos"))]
pub(crate) fn play(path: &std::path::Path) -> Result<()> {
    // Linux fallback: aplay ships with alsa-utils.
    let mut cmd = Command::new("aplay");
    cmd.arg(path);
    run_audible(cmd, "aplay")
}
