//! Mic capture via `cpal`, in two modes:
//!   - push-to-talk ([`record_push_to_talk`]): Enter to start, Enter to stop.
//!   - continuous ([`Listener`]): always capturing; a VAD [`Segmenter`] cuts the
//!     stream into utterances on speech/silence boundaries.
//!
//! Both deliver mono 16 kHz f32 ready for [`crate::stt`].

use std::io::{BufRead, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};

use crate::ui::{LogKind, Reporter};

/// The sample rate whisper.cpp models are trained on.
pub const WHISPER_SAMPLE_RATE: u32 = 16_000;

/// How long after the copilot stops speaking to keep ignoring the mic — covers
/// audio still buffered in the input and room echo, so the tail of a readback or
/// alert can't leak in as a phantom command.
const MIC_GATE_TAIL: Duration = Duration::from_millis(400);

/// A shared "the copilot is speaking — don't listen" flag. The speech arbiter
/// raises it around every utterance; the [`Listener`] drops captured audio while
/// it's raised (and briefly after) so the copilot never hears itself. Half-duplex
/// by design: a voice command and a spoken readback/alert never overlap.
#[derive(Clone)]
pub struct MicGate(Arc<AtomicBool>);

impl MicGate {
    pub fn new() -> Self {
        MicGate(Arc::new(AtomicBool::new(false)))
    }
    /// Raise (true) while speaking, lower (false) when done.
    pub fn set_speaking(&self, speaking: bool) {
        self.0.store(speaking, Ordering::Relaxed);
    }
    pub fn is_speaking(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}

/// An open input device: the live stream plus the buffer its callback fills.
struct Input {
    stream: cpal::Stream,
    buffer: Arc<Mutex<Vec<f32>>>,
    sample_rate: u32,
    channels: usize,
}

/// Open the default input device and build (but don't start) its stream.
fn open_input() -> Result<Input> {
    let host = cpal::default_host();
    let device = host
        .default_input_device()
        .ok_or_else(|| anyhow!("no default input device — is a mic connected?"))?;
    let config = device
        .default_input_config()
        .context("querying default input config")?;

    let sample_rate = config.sample_rate().0;
    let channels = config.channels() as usize;
    let buffer: Arc<Mutex<Vec<f32>>> = Arc::new(Mutex::new(Vec::new()));
    let stream = build_stream(&device, &config, buffer.clone())?;

    Ok(Input { stream, buffer, sample_rate, channels })
}

/// Record a push-to-talk utterance. Blocks on two Enter presses (start, stop).
pub fn record_push_to_talk() -> Result<Vec<f32>> {
    let input = open_input()?;

    prompt("🎙️  Press Enter to start talking…");
    wait_for_enter()?;
    input.stream.play().context("starting input stream")?;
    prompt("…recording. Press Enter to stop.");
    wait_for_enter()?;
    drop(input.stream); // stop and release the device

    let interleaved = std::mem::take(&mut *input.buffer.lock().unwrap());
    let mono = downmix(&interleaved, input.channels);
    Ok(resample_linear(&mono, input.sample_rate, WHISPER_SAMPLE_RATE))
}

/// Diagnostic: open the mic and print live RMS levels so you can confirm capture
/// works and pick a VAD threshold. Runs until Ctrl-C.
pub fn mic_check(threshold: f32) -> Result<()> {
    let input = open_input()?;
    input.stream.play().context("starting input stream")?;
    println!(
        "🎤 Mic check — speak normally. 'SPEECH' = above threshold ({threshold:.4}). Ctrl-C to quit.\n"
    );
    let mut idle_ticks = 0;
    loop {
        std::thread::sleep(Duration::from_millis(150));
        let new = std::mem::take(&mut *input.buffer.lock().unwrap());
        if new.is_empty() {
            idle_ticks += 1;
            if idle_ticks % 10 == 0 {
                println!("  (no audio frames yet — check mic permission / input device)");
            }
            continue;
        }
        idle_ticks = 0;
        let level = rms(&downmix(&new, input.channels));
        let bar = "█".repeat(((level * 300.0) as usize).min(50));
        let tag = if level > threshold { "SPEECH" } else { "quiet " };
        println!("  {tag}  rms={level:.4}  |{bar}");
    }
}

/// Always-on listener: holds the stream open and yields one utterance per call.
pub struct Listener {
    input: Input,
    segmenter: Segmenter,
    reporter: Reporter,
    /// Raised while the copilot speaks; audio captured then is dropped.
    gate: MicGate,
    /// Confirms a captured segment is speech before it's handed to whisper;
    /// rejects non-speech bursts (sim/engine audio, knocks) the energy gate let in.
    speech: crate::vad::SpeechFilter,
}

impl Listener {
    /// Start capturing, then calibrate the VAD to the room's noise floor so the
    /// speech threshold sits above ambient. `vad_floor` is a hard minimum (the
    /// configured `XCOPILOT_VAD_RMS`), so a quiet room can't drive it too low.
    pub fn start(vad_floor: f32, end_silence_ms: u32, reporter: Reporter, gate: MicGate) -> Result<Self> {
        let input = open_input()?;
        input.stream.play().context("starting input stream")?;

        reporter.log(LogKind::Info, "calibrating microphone — stay quiet for ~1s…");
        std::thread::sleep(Duration::from_millis(300)); // discard startup transient
        input.buffer.lock().unwrap().clear();
        let mut ambient_peak = 0.0f32;
        for _ in 0..12 {
            std::thread::sleep(Duration::from_millis(80));
            let new = std::mem::take(&mut *input.buffer.lock().unwrap());
            if !new.is_empty() {
                ambient_peak = ambient_peak.max(rms(&downmix(&new, input.channels)));
            }
        }
        let initial = (ambient_peak * VAD_RATIO).max(vad_floor);
        reporter.log(
            LogKind::Info,
            format!("ambient ≈ {ambient_peak:.4} → speech threshold ≈ {initial:.4}"),
        );

        Ok(Self {
            input,
            segmenter: Segmenter::new(
                vad_floor,
                VAD_RATIO,
                ambient_peak, // seed the adaptive floor
                end_silence_ms,
                MIN_SPEECH_MS,
                MAX_SPEECH_MS,
                FRAME_MS,
            ),
            reporter,
            gate,
            speech: crate::vad::SpeechFilter::new(),
        })
    }

    /// Block until a complete spoken utterance is captured, then return it as
    /// mono 16 kHz f32. Audio captured before this call (e.g. while we were
    /// transcribing or speaking) is discarded, so the copilot never hears its
    /// own readback.
    /// Capture one utterance. With `idle_timeout`, return an **empty** vec if no
    /// speech has started within that window (and we're not mid-utterance) — so a
    /// caller can poll another input source between waits without blocking the
    /// mic forever. `None` blocks until an utterance completes (the old behavior).
    pub fn next_utterance(&mut self, idle_timeout: Option<Duration>) -> Result<Vec<f32>> {
        self.input.buffer.lock().unwrap().clear();
        self.segmenter.reset();

        let frame_len = (self.input.sample_rate as usize * FRAME_MS as usize) / 1000;
        let mut pending: Vec<f32> = Vec::new();
        // When the copilot last finished speaking — used for the post-speech tail.
        let mut spoke_until: Option<Instant> = None;
        let start = Instant::now();

        'capture: loop {
            std::thread::sleep(Duration::from_millis(FRAME_MS as u64));

            // Half-duplex: while the copilot is speaking (or just did), throw away
            // captured audio and hold the segmenter reset, so its own voice never
            // becomes an utterance.
            if self.gate.is_speaking() {
                spoke_until = Some(Instant::now() + MIC_GATE_TAIL);
            }
            if spoke_until.is_some_and(|t| Instant::now() < t) {
                self.input.buffer.lock().unwrap().clear();
                self.segmenter.reset();
                pending.clear();
                self.reporter.mic(0.0);
                continue;
            }

            // Idle timeout: only bail when no utterance is in progress, so we
            // never cut off mid-speech; return empty to signal "nothing yet".
            if let Some(to) = idle_timeout {
                if !self.segmenter.in_speech() && start.elapsed() >= to {
                    return Ok(Vec::new());
                }
            }

            let new = std::mem::take(&mut *self.input.buffer.lock().unwrap());
            if new.is_empty() {
                continue;
            }
            let mono = downmix(&new, self.input.channels);
            self.reporter.mic(rms(&mono)); // feed the dashboard mic meter
            pending.extend(mono);

            // Feed whole frames to the segmenter; keep the remainder for next time.
            let mut offset = 0;
            while offset + frame_len <= pending.len() {
                let frame = &pending[offset..offset + frame_len];
                if let Some(utterance) = self.segmenter.push(frame) {
                    // Drop consumed audio; anything captured after starts fresh.
                    *self.input.buffer.lock().unwrap() = Vec::new();
                    let resampled = resample_linear(
                        &utterance,
                        self.input.sample_rate,
                        WHISPER_SAMPLE_RATE,
                    );
                    // Speech gate: only hand real speech to whisper. Non-speech
                    // bursts (sim/engine audio, knocks) are dropped, and we keep
                    // listening rather than transcribing noise into a command.
                    if self.speech.is_speech(&resampled) {
                        return Ok(resampled);
                    }
                    self.reporter.log(LogKind::Info, "ignored (no speech detected)");
                    pending.clear();
                    self.segmenter.reset();
                    continue 'capture;
                }
                offset += frame_len;
            }
            pending.drain(0..offset);
        }
    }
}

// --- VAD segmenter -----------------------------------------------------------

/// Frame size fed to the segmenter.
const FRAME_MS: u32 = 30;
/// Minimum voiced duration to count as a real utterance (rejects blips).
const MIN_SPEECH_MS: u32 = 300;
/// Hard cap on one utterance — a runaway guard so a too-low floor can't wedge
/// the segmenter "in speech" forever.
const MAX_SPEECH_MS: u32 = 12_000;
/// Speech threshold = noise_floor × this (above the floor counts as voice).
const VAD_RATIO: f32 = 2.5;
/// Audio kept *before* speech onset and prepended to the utterance, so the attack
/// of the first word (which sits below the threshold until the level rises) isn't
/// clipped — "flaps" stops becoming "laps".
const PREROLL_MS: u32 = 240;

/// Energy-based voice-activity segmenter with an **adaptive noise floor**. Pure
/// logic (no audio I/O), so it's unit-testable: feed it fixed-size mono frames;
/// it returns the accumulated samples when an utterance completes.
///
/// The threshold isn't fixed — it tracks the room. During non-speech frames the
/// floor eases toward the observed level (EMA), so a too-high start self-corrects
/// downward; a runaway speech run is force-ended by `max_speech_ms` and bumps the
/// floor up, so a too-low start self-corrects too. This is what keeps a noisy,
/// drifting far-field mic from getting permanently stuck.
pub struct Segmenter {
    abs_min: f32,
    ratio: f32,
    noise_floor: f32,
    end_silence_ms: u32,
    min_speech_ms: u32,
    max_speech_ms: u32,
    frame_ms: u32,
    in_speech: bool,
    speech: Vec<f32>,
    silence_ms: u32,
    speech_ms: u32,
    /// Rolling window of the most recent ambient samples; prepended to `speech`
    /// when an utterance starts so its onset isn't clipped.
    preroll: std::collections::VecDeque<f32>,
    preroll_ms: u32,
}

impl Segmenter {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        abs_min: f32,
        ratio: f32,
        initial_floor: f32,
        end_silence_ms: u32,
        min_speech_ms: u32,
        max_speech_ms: u32,
        frame_ms: u32,
    ) -> Self {
        Self {
            abs_min,
            ratio,
            noise_floor: initial_floor,
            end_silence_ms,
            min_speech_ms,
            max_speech_ms,
            frame_ms,
            in_speech: false,
            speech: Vec::new(),
            silence_ms: 0,
            speech_ms: 0,
            preroll: std::collections::VecDeque::new(),
            preroll_ms: PREROLL_MS,
        }
    }

    /// Whether an utterance is currently being captured (voiced, not yet ended).
    pub fn in_speech(&self) -> bool {
        self.in_speech
    }

    /// Current speech threshold from the adaptive floor.
    fn threshold(&self) -> f32 {
        (self.noise_floor * self.ratio).max(self.abs_min)
    }

    /// Clear per-utterance state. The learned `noise_floor` persists across
    /// utterances on purpose.
    fn reset(&mut self) {
        self.in_speech = false;
        self.speech.clear();
        self.silence_ms = 0;
        self.speech_ms = 0;
        self.preroll.clear();
    }

    /// Roll the pre-roll window: keep the most recent `preroll_ms` of ambient
    /// audio so it can be prepended when speech starts.
    fn push_preroll(&mut self, frame: &[f32]) {
        let cap = (self.preroll_ms as usize / self.frame_ms.max(1) as usize) * frame.len();
        self.preroll.extend(frame.iter().copied());
        while self.preroll.len() > cap {
            self.preroll.pop_front();
        }
    }

    /// Feed one mono frame (`frame_ms` long). Returns `Some(samples)` when an
    /// utterance just completed.
    pub fn push(&mut self, frame: &[f32]) -> Option<Vec<f32>> {
        let level = rms(frame);
        let voiced = level > self.threshold();

        if !voiced {
            // Ease the floor toward ambient (only when not speaking, so speech
            // can't inflate it).
            self.noise_floor = self.noise_floor * 0.97 + level * 0.03;
        }

        if voiced {
            if !self.in_speech {
                // Onset: seed with the pre-roll so the first word's attack survives.
                self.speech.extend(self.preroll.drain(..));
            }
            self.in_speech = true;
            self.speech.extend_from_slice(frame);
            self.speech_ms += self.frame_ms;
            self.silence_ms = 0;

            // Runaway guard: if "speech" never ends, the floor is too low. Emit
            // what we have, raise the floor toward this level, and recover.
            if self.speech_ms >= self.max_speech_ms {
                self.noise_floor = self.noise_floor.max(level);
                let done = std::mem::take(&mut self.speech);
                self.reset();
                return Some(done);
            }
            return None;
        }

        if !self.in_speech {
            self.push_preroll(frame); // roll the pre-onset window
            return None;
        }

        // In speech, hit a silent frame — keep it (natural trailing pause).
        self.speech.extend_from_slice(frame);
        self.silence_ms += self.frame_ms;
        if self.silence_ms < self.end_silence_ms {
            return None;
        }

        let spoke = self.speech_ms;
        let done = std::mem::take(&mut self.speech);
        self.reset();
        if spoke >= self.min_speech_ms {
            Some(done)
        } else {
            None // too short — discard as noise
        }
    }
}

/// Root-mean-square energy of a frame.
fn rms(frame: &[f32]) -> f32 {
    if frame.is_empty() {
        return 0.0;
    }
    let sum_sq: f32 = frame.iter().map(|s| s * s).sum();
    (sum_sq / frame.len() as f32).sqrt()
}

// --- stream + DSP helpers ----------------------------------------------------

/// Build an input stream for whatever sample format the device offers, writing
/// f32 samples into `buffer`.
fn build_stream(
    device: &cpal::Device,
    config: &cpal::SupportedStreamConfig,
    buffer: Arc<Mutex<Vec<f32>>>,
) -> Result<cpal::Stream> {
    let err_fn = |e| tracing::error!("audio stream error: {e}");
    let stream_config: cpal::StreamConfig = config.config();

    let stream = match config.sample_format() {
        cpal::SampleFormat::F32 => device.build_input_stream(
            &stream_config,
            move |data: &[f32], _| buffer.lock().unwrap().extend_from_slice(data),
            err_fn,
            None,
        )?,
        cpal::SampleFormat::I16 => device.build_input_stream(
            &stream_config,
            move |data: &[i16], _| {
                let mut b = buffer.lock().unwrap();
                b.extend(data.iter().map(|&s| s as f32 / i16::MAX as f32));
            },
            err_fn,
            None,
        )?,
        cpal::SampleFormat::U16 => device.build_input_stream(
            &stream_config,
            move |data: &[u16], _| {
                let mut b = buffer.lock().unwrap();
                b.extend(data.iter().map(|&s| (s as f32 / u16::MAX as f32) * 2.0 - 1.0));
            },
            err_fn,
            None,
        )?,
        other => return Err(anyhow!("unsupported sample format: {other:?}")),
    };
    Ok(stream)
}

/// Average interleaved channels down to mono.
fn downmix(interleaved: &[f32], channels: usize) -> Vec<f32> {
    if channels <= 1 {
        return interleaved.to_vec();
    }
    interleaved
        .chunks(channels)
        .map(|frame| frame.iter().sum::<f32>() / channels as f32)
        .collect()
}

/// Cheap linear resample. Fine for speech into whisper; not hi-fi.
fn resample_linear(input: &[f32], from: u32, to: u32) -> Vec<f32> {
    if from == to || input.is_empty() {
        return input.to_vec();
    }
    let ratio = to as f64 / from as f64;
    let out_len = (input.len() as f64 * ratio).round() as usize;
    let mut out = Vec::with_capacity(out_len);
    for i in 0..out_len {
        let src = i as f64 / ratio;
        let idx = src.floor() as usize;
        let frac = src - idx as f64;
        let a = input[idx.min(input.len() - 1)];
        let b = input[(idx + 1).min(input.len() - 1)];
        out.push(a + (b - a) * frac as f32);
    }
    out
}

fn prompt(msg: &str) {
    println!("{msg}");
    let _ = std::io::stdout().flush();
}

fn wait_for_enter() -> Result<()> {
    let mut line = String::new();
    std::io::stdin()
        .lock()
        .read_line(&mut line)
        .context("reading Enter")?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frames(samples: &[f32], frame_len: usize) -> Vec<&[f32]> {
        samples.chunks(frame_len).filter(|c| c.len() == frame_len).collect()
    }

    #[test]
    fn rms_zero_for_silence() {
        assert_eq!(rms(&[0.0; 480]), 0.0);
        assert!(rms(&[0.3, -0.3, 0.3, -0.3]) > 0.1);
    }

    #[test]
    fn segmenter_extracts_one_utterance_between_silences() {
        // 30 ms frames at 16 kHz = 480 samples.
        let frame_len = 480;
        // abs_min 0.05, ratio 2.5, floor 0; short timings for the test.
        let mut seg = Segmenter::new(0.05, 2.5, 0.0, 90, 60, 100_000, 30);

        // 1s silence, ~0.5s tone (speech), 1s silence.
        let mut audio = vec![0.0f32; frame_len * 33];
        audio.extend(std::iter::repeat(0.3).take(frame_len * 16));
        audio.extend(std::iter::repeat(0.0).take(frame_len * 33));

        let mut got = None;
        for f in frames(&audio, frame_len) {
            if let Some(u) = seg.push(f) {
                got = Some(u);
                break;
            }
        }
        let utterance = got.expect("segmenter should emit one utterance");
        // Roughly the speech span (+ trailing silence window), not the leading silence.
        let secs = utterance.len() as f32 / 16_000.0;
        assert!(secs > 0.4 && secs < 1.0, "unexpected utterance length: {secs}s");
    }

    #[test]
    fn segmenter_ignores_short_blips() {
        let frame_len = 480;
        // require 300ms of speech to count.
        let mut seg = Segmenter::new(0.05, 2.5, 0.0, 90, 300, 100_000, 30);
        let mut audio = vec![0.0f32; frame_len * 5];
        audio.extend(std::iter::repeat(0.3).take(frame_len * 2)); // only 60ms of "speech"
        audio.extend(std::iter::repeat(0.0).take(frame_len * 10));

        let mut got = None;
        for f in frames(&audio, frame_len) {
            if let Some(u) = seg.push(f) {
                got = Some(u);
            }
        }
        assert!(got.is_none(), "short blip should be rejected");
    }

    #[test]
    fn segmenter_preroll_keeps_onset() {
        let frame_len = 480;
        let mut seg = Segmenter::new(0.05, 2.5, 0.0, 90, 60, 100_000, 30);
        let mut audio = vec![0.0f32; frame_len * 20]; // initial silence
        audio.extend(std::iter::repeat(0.01).take(frame_len * 10)); // quiet pre-onset ambient
        audio.extend(std::iter::repeat(0.3).take(frame_len * 16)); // speech
        audio.extend(std::iter::repeat(0.0).take(frame_len * 10));

        let mut utt = None;
        for f in frames(&audio, frame_len) {
            if let Some(u) = seg.push(f) {
                utt = Some(u);
                break;
            }
        }
        let utt = utt.expect("utterance");
        // The clip starts with the quiet pre-roll, not the loud onset — proof the
        // first word's attack would survive.
        assert!(utt[0].abs() < 0.05, "expected pre-roll before speech, got {}", utt[0]);
        // Pre-roll (~240 ms = 8 frames) pads ahead of the 16 speech frames.
        let captured = utt.len() / frame_len;
        assert!(captured >= 16 + 6, "expected pre-roll padding, got {captured} frames");
    }
}
