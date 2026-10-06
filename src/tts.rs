//! Text-to-speech readback — the copilot's voice, confirming the *actual* action.
//!
//! The default backend is the warm Spark voice-clone `sidecar`
//! ([`crate::tts_sidecar`]); this module covers the subprocess fallbacks selected
//! by `config.tts_backend`:
//!   - `say`   — macOS built-in. Zero install; the fallback when the sidecar
//!     can't start.
//!   - `piper` — shell out to the piper binary + a voice model (Horizon's trick).
//!

use std::io::Write;
use std::process::{Command, Stdio};

use anyhow::{bail, Context, Result};

use crate::config::Config;

/// Speak `text` aloud using the configured backend.
pub fn speak(config: &Config, text: &str) -> Result<()> {
    match config.tts_backend.as_str() {
        "say" => speak_say(config, text),
        "off" => {
            silent_pace(text);
            Ok(())
        }
        "piper" => speak_piper(config, text),
        other => bail!("unknown tts_backend: {other} (expected `say` or `piper`)"),
    }
}

/// Text only (`XCOPILOT_TTS=off`): nothing is synthesised or played, but a
/// line still takes about as long as it would to say (15 characters a
/// second, at most 12 s), so the arbiter's order, the radio's on-air window
/// and the loop's waits keep their shape.
pub fn silent_pace(text: &str) {
    let s = (text.chars().count() as f64 / 15.0).clamp(0.3, 12.0);
    std::thread::sleep(std::time::Duration::from_secs_f64(s));
}

#[cfg(test)]
mod off_tests {
    #[test]
    fn a_silent_line_is_paced_and_bounded() {
        let t = std::time::Instant::now();
        super::speak(
            &crate::config::Config { tts_backend: "off".into(), ..Default::default() },
            "Flaps up.",
        )
        .unwrap();
        let s = t.elapsed().as_secs_f64();
        assert!((0.55..1.5).contains(&s), "{s}");
    }
}

/// Ceiling on any speech subprocess. A `say`/`afplay` that never exits would
/// otherwise block the speaker thread forever — with the mic gate raised, so
/// the app goes deaf too (seen live 2026-07-07: one hung `say` froze the whole
/// loop, unkillable). Generous: the longest legitimate utterance (a full ATIS)
/// is well under a minute.
const SPEECH_WATCHDOG: std::time::Duration = std::time::Duration::from_secs(90);

/// Which of the two independent audio channels a playback belongs to. The
/// INTERCOM is the copilot's voice (always hearable, like a crew member next
/// to you); the RADIO is every station voice (controllers, ATIS, ghosts).
/// Each speaker thread tags itself once ([`set_thread_channel`]) and every
/// audible child it spawns registers under that channel — so a radio cut
/// (retune, avionics off, crash, replay) kills ONLY station audio and the
/// copilot keeps talking, while a PTT press still cuts both (mic hygiene).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Intercom = 0,
    Radio = 1,
}

thread_local! {
    /// The channel this thread's playback registers under. Defaults to the
    /// intercom: only the radio speaker thread ever retags itself.
    static THREAD_CHANNEL: std::cell::Cell<Channel> = const { std::cell::Cell::new(Channel::Intercom) };
}

/// Tag the CURRENT thread's playback channel (called once by each speaker
/// thread at startup).
pub fn set_thread_channel(ch: Channel) {
    THREAD_CHANNEL.with(|c| c.set(ch));
}

fn thread_channel() -> Channel {
    THREAD_CHANNEL.with(|c| c.get())
}

/// Per-channel playback volume (f32 bits; 1.0 = unity). Applied as `afplay -v`
/// on the WAV-playing paths; the `say` and `aplay` fallbacks have no volume
/// knob and ignore it. Set once at startup from config.
static VOLUME_BITS: [std::sync::atomic::AtomicU32; 2] = [
    std::sync::atomic::AtomicU32::new(1.0f32.to_bits()),
    std::sync::atomic::AtomicU32::new(1.0f32.to_bits()),
];

/// Set the two channel volumes (intercom = copilot voice, radio = stations).
/// Clamped to a sane afplay range.
pub fn set_volumes(intercom: f32, radio: f32) {
    use std::sync::atomic::Ordering::Relaxed;
    VOLUME_BITS[Channel::Intercom as usize].store(intercom.clamp(0.0, 2.0).to_bits(), Relaxed);
    VOLUME_BITS[Channel::Radio as usize].store(radio.clamp(0.0, 2.0).to_bits(), Relaxed);
}

fn channel_volume(ch: Channel) -> f32 {
    f32::from_bits(VOLUME_BITS[ch as usize].load(std::sync::atomic::Ordering::Relaxed))
}

/// Barge-in bookkeeping, one slot PER CHANNEL: the audible child's pid, plus a
/// cut generation the killer bumps. Killing by REGISTERED PID (not `pkill` by
/// name) means the cut can't touch unrelated processes — and the generation
/// closes the spawn race: a PTT press landing in the few ms between "about to
/// spawn" and "spawned" used to be missed entirely, letting the whole
/// transmission talk over the pilot and then replay as a stepped-on tail.
static PLAYER_PID: std::sync::Mutex<[Option<u32>; 2]> = std::sync::Mutex::new([None, None]);
static CUT_GEN: [std::sync::atomic::AtomicU64; 2] = [
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
];

/// Spawn-to-exit wrapper for every AUDIBLE child (`say`, `afplay`, `aplay`):
/// registers the pid (under the thread's channel) for [`stop_playback`],
/// detects a cut that landed while the process was spawning, and rides the
/// hang watchdog. One speaker thread per channel, so one slot per channel is
/// the whole story.
fn run_audible(cmd: Command, what: &str) -> Result<()> {
    // A test never talks out loud: the voice tests queue real lines ("cleared
    // to land", "station line still queued") through real speaker threads,
    // and every `cargo test` played them into the cockpit mid-flight
    // (2026-09-24). What reaches this point has already been queued, routed
    // and gated, which is all a test asserts.
    if cfg!(test) {
        return Ok(());
    }
    spawn_audible(cmd, what)
}

fn spawn_audible(mut cmd: Command, what: &str) -> Result<()> {
    use std::sync::atomic::Ordering::SeqCst;
    let ch = thread_channel() as usize;
    let gen = CUT_GEN[ch].load(SeqCst);
    let child = cmd.spawn().with_context(|| format!("spawning {what}"))?;
    PLAYER_PID.lock().unwrap_or_else(|e| e.into_inner())[ch] = Some(child.id());
    if CUT_GEN[ch].load(SeqCst) != gen {
        kill_pid(child.id()); // the press won the race; honor it
    }
    let res = wait_with_watchdog(child, what);
    PLAYER_PID.lock().unwrap_or_else(|e| e.into_inner())[ch] = None;
    res
}

fn kill_pid(pid: u32) {
    let _ = Command::new("kill").arg(pid.to_string()).status();
}

/// Wait for a speech child process, killing it if it overruns the watchdog.
fn wait_with_watchdog(mut child: std::process::Child, what: &str) -> Result<()> {
    let start = std::time::Instant::now();
    loop {
        match child
            .try_wait()
            .with_context(|| format!("polling {what}"))?
        {
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
    let wav_path = std::env::temp_dir().join("xvoicelayer-readback.wav");

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

/// Kill ALL in-progress playback right now — the barge-in / PTT path (any PTT
/// mutes everything, for mic hygiene). Best-effort: nothing playing means
/// nothing to do.
pub fn stop_playback() {
    stop_channel(Channel::Intercom);
    stop_channel(Channel::Radio);
}

/// Kill one channel's in-progress playback — the radio-only cuts (retune,
/// avionics off, crash, replay) go through here so the copilot's intercom
/// voice keeps talking. Bumps the channel's cut generation first, so even a
/// child mid-spawn (registered a moment later) dies immediately.
pub fn stop_channel(ch: Channel) {
    CUT_GEN[ch as usize].fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    if let Some(pid) = PLAYER_PID.lock().unwrap_or_else(|e| e.into_inner())[ch as usize] {
        kill_pid(pid);
    }
    // On the bus the sound lives in the sim, and the speaker thread's waiter
    // sends the cut when it sees the generation move: the ONE writer, so a
    // cut can never overtake the next play and silence the wrong line.
}

/// The channel's cut generation, for a speaker to notice a cut that landed
/// while it was still rendering.
pub fn channel_generation(ch: Channel) -> u64 {
    CUT_GEN[ch as usize].load(std::sync::atomic::Ordering::SeqCst)
}

/// Play mono f32 PCM through the platform player (write a temp WAV, play it).
/// Lives here rather than in `tts_onnx` so the speech arbiter's barge-in tail
/// path (replaying a sliced sample buffer) builds without the onnx feature.
/// The WAV name carries the thread's channel: both speaker threads can be
/// synthesizing at once, and a shared name would corrupt whichever loses.
#[allow(dead_code)] // consumed by the copilot binary's tree (voice.rs), not the lib's
pub(crate) fn play_pcm(samples: &[f32], sample_rate: u32) -> Result<()> {
    let name = match thread_channel() {
        Channel::Intercom => "xvoicelayer-pcm-intercom.wav",
        Channel::Radio => "xvoicelayer-pcm-radio.wav",
    };
    let path = std::env::temp_dir().join(name);
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
    if AUDIO_BUS.load(std::sync::atomic::Ordering::Relaxed) {
        match play_on_bus(&path, samples.len(), sample_rate) {
            Ok(()) => return Ok(()),
            Err(e) => {
                // Once per session: the plugin is missing or old, or the sim is
                // down. The platform player still speaks, so nothing is lost.
                if !BUS_WARNED.swap(true, std::sync::atomic::Ordering::Relaxed) {
                    tracing::warn!("audio bus unavailable, playing locally instead: {e:#}");
                }
            }
        }
    }
    play(&path)
}

/// Voices through the sim's own audio. Set at startup from
/// `XCOPILOT_AUDIO_BUS` (which PINS it: the panel can't turn it off) and
/// otherwise from the panel's Settings switch through [`set_audio_bus`].
pub static AUDIO_BUS: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static AUDIO_BUS_PINNED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static BUS_WARNED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static BUS_BASE_URL: std::sync::Mutex<String> = std::sync::Mutex::new(String::new());
/// One client for every bus write (a blocking client owns a runtime and a
/// thread; one per call was a runtime per utterance).
static BUS_CLIENT: std::sync::OnceLock<reqwest::blocking::Client> = std::sync::OnceLock::new();
/// The two mailboxes' ids: `[play, cut]`. X-Plane reassigns ids across
/// restarts and plugin reloads, so a failed write evicts and the next call
/// re-resolves (the convention in `xplane.rs`: a 404 is a stale id).
static BUS_IDS: [std::sync::atomic::AtomicI64; 2] = [
    std::sync::atomic::AtomicI64::new(0),
    std::sync::atomic::AtomicI64::new(0),
];

/// Where the sim's Web API is, for the bus writes, and the environment's
/// verdict. Called once at startup.
pub fn configure_audio_bus(enabled: bool, pinned: bool, base_url: &str) {
    *BUS_BASE_URL.lock().unwrap_or_else(|e| e.into_inner()) =
        base_url.trim_end_matches('/').to_string();
    AUDIO_BUS.store(enabled, std::sync::atomic::Ordering::Relaxed);
    AUDIO_BUS_PINNED.store(pinned, std::sync::atomic::Ordering::Relaxed);
}

/// What the sim's own radio audio should be, given whether OUR stations own
/// the frequencies (`own_atc`: ATC mode on with `XCOPILOT_MUTE_XP_RADIO`)
/// and whether our voices ride the sim's audio bus.
///
/// The mute (2026-07-10: two ATIS voices on one frequency) zeroes the sim's
/// Radio slider, and the station on the bus plays on COM1, which FOLLOWS
/// that slider: with both on, our own controller spoke at volume zero and the
/// pilot heard no ATC at all (LIPK, 2026-09-26). With the bus on, the slider
/// stays up and the sim's ATIS is switched off instead.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SimRadio {
    /// `sim/operation/sound/radio_volume_ratio`, x100.
    pub volume_pct: u8,
    /// `sim/atc/atis_enabled`; `None` leaves it alone.
    pub atis: Option<bool>,
}

pub fn sim_radio(own_atc: bool, bus: bool) -> SimRadio {
    match (own_atc, bus) {
        (true, false) => SimRadio { volume_pct: 0, atis: None },
        (true, true) => SimRadio { volume_pct: 100, atis: Some(false) },
        (false, _) => SimRadio { volume_pct: 100, atis: Some(true) },
    }
}

/// The panel's pick. Refused (false) when the environment pinned it on.
pub fn set_audio_bus(on: bool) -> bool {
    // Pinned by the environment, either way: the registry's rule for every
    // env-pinned knob.
    if AUDIO_BUS_PINNED.load(std::sync::atomic::Ordering::Relaxed) {
        return false;
    }
    AUDIO_BUS.store(on, std::sync::atomic::Ordering::Relaxed);
    true
}

/// The plugin mailboxes (see `plugin/src/audio.cpp`).
const BUS_PLAY: &str = "xcopilot/audio/play";
const BUS_CUT: &str = "xcopilot/audio/cut";
/// The sim starts a posted sample on its next frame; allow for that and for
/// the FMOD tail before declaring the utterance finished.
const BUS_TAIL_MS: u64 = 200;
/// How often the waiter looks for a cut: the barge-in latency on the bus.
const BUS_CUT_POLL_MS: u64 = 20;

/// A blocking Web API write, for the speaker thread. The thread is a plain
/// one (it blocks on `afplay` today), so a blocking client is fine here; on
/// a tokio worker it would panic, so refuse rather than find out. A failed
/// write evicts the cached id: after a plugin reload the next post
/// re-resolves instead of failing for the rest of the session.
fn bus_write(slot: usize, name: &str, value: serde_json::Value) -> Result<()> {
    use std::sync::atomic::Ordering::Relaxed;
    if tokio::runtime::Handle::try_current().is_ok() {
        bail!("audio bus write from an async thread");
    }
    if slot == 0 && crate::xplane::sim_loading() {
        // No new play into a loading sim (the line plays locally instead);
        // a cut still goes.
        bail!("a flight is loading");
    }
    let base = BUS_BASE_URL
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone();
    if base.is_empty() {
        bail!("audio bus not configured");
    }
    let client = BUS_CLIENT.get_or_init(|| {
        reqwest::blocking::Client::builder()
            // Always localhost: a system proxy lookup is never wanted, and reqwest's
            // default one can stall the first build for seconds (2026-09-27).
            .no_proxy()
            .timeout(std::time::Duration::from_millis(1500))
            .build()
            .expect("a blocking client")
    });
    let mut id = BUS_IDS[slot].load(Relaxed);
    if id == 0 {
        #[derive(serde::Deserialize)]
        struct Index {
            data: Vec<Meta>,
        }
        #[derive(serde::Deserialize)]
        struct Meta {
            id: i64,
            name: String,
        }
        let index: Index = client
            .get(format!("{base}/api/v1/datarefs"))
            .query(&[("filter[name]", name)])
            .send()
            .context("dataref lookup")?
            .error_for_status()?
            .json()
            .context("decoding dataref index")?;
        id = index
            .data
            .into_iter()
            .find(|d| d.name == name)
            .map(|d| d.id)
            .ok_or_else(|| anyhow::anyhow!("{name} not registered (plugin missing or old)"))?;
        BUS_IDS[slot].store(id, Relaxed);
    }
    let ok = client
        .patch(format!("{base}/api/v1/datarefs/{id}/value"))
        .json(&serde_json::json!({ "data": value }))
        .send()
        .map(|r| r.status().is_success())
        .unwrap_or(false);
    if !ok {
        BUS_IDS[slot].store(0, Relaxed);
        bail!("writing {name} failed (stale id or sim down); will re-resolve");
    }
    BUS_WARNED.store(false, Relaxed);
    Ok(())
}

/// Post the WAV to the plugin and wait out its duration, honouring a cut the
/// way [`run_audible`] does: the killer bumps the channel's generation, this
/// waiter notices within a tick and tells the plugin to stop the channel.
fn play_on_bus(path: &std::path::Path, samples: usize, sample_rate: u32) -> Result<()> {
    use std::sync::atomic::Ordering::SeqCst;
    let ch = thread_channel();
    let gen = CUT_GEN[ch as usize].load(SeqCst);
    let payload = format!(
        "{}|{:.2}|{}",
        ch as usize,
        channel_volume(ch),
        path.display()
    );
    use base64::Engine as _;
    let b64 = base64::engine::general_purpose::STANDARD.encode(payload.as_bytes());
    bus_write(0, BUS_PLAY, serde_json::json!(b64))?;
    let total = std::time::Duration::from_millis(
        (samples as f64 / sample_rate.max(1) as f64 * 1000.0) as u64 + BUS_TAIL_MS,
    );
    let start = std::time::Instant::now();
    while start.elapsed() < total {
        if CUT_GEN[ch as usize].load(SeqCst) != gen {
            let _ = bus_write(1, BUS_CUT, serde_json::json!(ch as usize));
            return Ok(());
        }
        std::thread::sleep(std::time::Duration::from_millis(BUS_CUT_POLL_MS));
    }
    Ok(())
}

/// Play a WAV file using the platform's default CLI player. `pub(crate)` so the
/// in-process ONNX backend ([`crate::tts_onnx`]) can reuse it. Applies the
/// thread's channel volume (`afplay -v`) so the radio and the intercom can be
/// balanced independently (`XCOPILOT_ATC_VOL` / `XCOPILOT_VOICE_VOL`).
#[cfg(target_os = "macos")]
pub(crate) fn play(path: &std::path::Path) -> Result<()> {
    let mut cmd = Command::new("afplay");
    let vol = channel_volume(thread_channel());
    if (vol - 1.0).abs() > 0.001 {
        cmd.arg("-v").arg(format!("{vol}"));
    }
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

#[cfg(test)]
mod sim_radio_tests {
    use super::*;

    /// LIPK 2026-09-26: our station on the bus played on COM1 at the
    /// zero the mute had put on the Radio slider.
    #[test]
    fn our_station_on_the_bus_is_never_muted_by_our_own_mute() {
        let r = sim_radio(true, true);
        assert_eq!(r.volume_pct, 100);
        assert_eq!(r.atis, Some(false), "the sim's ATIS goes instead");
        assert_eq!(sim_radio(true, false).volume_pct, 0, "local player: the slider mute stands");
        assert_eq!(sim_radio(false, true).atis, Some(true));
    }
}
