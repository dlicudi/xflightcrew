//! Low-level mic capture primitives shared by the copilot and the `atc`
//! binary: the open cpal input stream, the push-to-talk capturer, and the
//! DSP helpers that deliver mono 16 kHz f32 ready for [`crate::stt`].
//!
//! Extracted from `audio.rs` (which keeps the copilot-specific continuous
//! `Listener`/`Segmenter` and re-exports these) so the lib target can carry
//! mic capture without dragging in the TUI reporter.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};

/// The sample rate whisper.cpp models are trained on.
pub const WHISPER_SAMPLE_RATE: u32 = 16_000;

/// A shared "we are speaking — don't listen" flag. The speaker raises it
/// around every utterance; capture drops audio while it's raised (and briefly
/// after) so a process never hears its own voice over the speakers.
/// Half-duplex by design.
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
pub(crate) struct Input {
    pub(crate) stream: cpal::Stream,
    pub(crate) buffer: Arc<Mutex<Vec<f32>>>,
    pub(crate) sample_rate: u32,
    pub(crate) channels: usize,
}

/// Name of the default input device cpal will capture from — for display in
/// the UI. `None` if no device is present or the platform won't name it.
pub fn default_input_name() -> Option<String> {
    cpal::default_host()
        .default_input_device()
        .and_then(|d| d.name().ok())
}

/// Open the default input device and build (but don't start) its stream.
pub(crate) fn open_input() -> Result<Input> {
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

    Ok(Input {
        stream,
        buffer,
        sample_rate,
        channels,
    })
}

/// Audio kept *before* the PTT press, prepended to the capture so the start of a
/// command isn't lost. It covers two latencies that otherwise clipped short
/// commands: the mic device's warm-up (eliminated here by keeping the stream
/// open) and, for the sim-dataref trigger, the lag between the physical button and
/// the dataref reflecting it. Generous on purpose — silence at the front is free,
/// a clipped first word isn't.
const PTT_PREROLL_MS: u32 = 300;

/// Warm push-to-talk mic: the input stream stays open across presses, so there's
/// no per-press device warm-up to clip the onset of a command. While idle the
/// buffer is trimmed to a short pre-roll; on press that pre-roll is kept and
/// capture runs until release. Works for any trigger — a held key, a sim
/// dataref, a terminal toggle — via the `is_active` closure.
pub struct PttMic {
    input: Input,
    /// Raised while we speak; audio captured then (plus a tail) is dropped — so
    /// holding PTT through a readback can't feed our own voice (over speakers)
    /// back in. Same half-duplex guard the copilot's `Listener` uses.
    gate: MicGate,
    gate_tail: Duration,
}

impl PttMic {
    /// Open the default input and start streaming. The stream stays live for the
    /// session; one `PttMic` serves every press. `gate`/`gate_tail` mute the mic
    /// while (and just after) we speak.
    pub fn start(gate: MicGate, gate_tail: Duration) -> Result<PttMic> {
        let input = open_input()?;
        input.stream.play().context("starting input stream")?;
        Ok(PttMic {
            input,
            gate,
            gate_tail,
        })
    }

    /// Wait up to `idle_timeout` for `is_active` to go true (returning empty if it
    /// doesn't, so the caller can poll typed input), then capture pre-roll plus
    /// everything until `is_active` goes false, at whisper's sample rate.
    pub fn capture(
        &self,
        is_active: impl Fn() -> bool,
        idle_timeout: Option<Duration>,
    ) -> Result<Vec<f32>> {
        let preroll = (self.input.sample_rate as usize * PTT_PREROLL_MS as usize / 1000)
            * self.input.channels.max(1);

        let start = Instant::now();
        while !is_active() {
            // Keep the idle buffer bounded to ~pre-roll of the most recent audio.
            self.trim_to(preroll);
            if idle_timeout.is_some_and(|t| start.elapsed() >= t) {
                return Ok(Vec::new()); // not pressed this window — let the caller re-poll
            }
            std::thread::sleep(Duration::from_millis(5));
        }

        // Pressed. Flush everything captured before this press down to ~pre-roll,
        // ALWAYS — not just via the wait loop above. That loop's trim is skipped
        // whenever PTT is already active on entry (the common case: pressing to
        // give the next command), and during the *previous* readback the command
        // loop was blocked in the blocking `speak()`/`afplay`, so `capture` wasn't
        // running to drop anything — the mic stream quietly piled the whole
        // readback into the buffer (the in-unit Studio-Display speaker→mic coupling
        // makes it a clean copy). Without this flush that readback gets prepended
        // to the new command and transcribed verbatim, feeding the copilot its own
        // voice. The continuous `Listener` clears at the start of every utterance
        // for the same reason; PttMic must too.
        self.trim_to(preroll);

        // Accumulate until release. As extra defence, while we are speaking
        // (or within the post-speech tail) drop what's captured, so holding PTT
        // through a readback can't record its own voice either.
        let mut spoke_until: Option<Instant> = None;
        while is_active() {
            if self.gate.is_speaking() {
                spoke_until = Some(Instant::now() + self.gate_tail);
            }
            if spoke_until.is_some_and(|t| Instant::now() < t) {
                self.input.buffer.lock().unwrap().clear();
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        // If release landed inside the tail, the final buffer is still self-echo.
        if spoke_until.is_some_and(|t| Instant::now() < t) {
            self.input.buffer.lock().unwrap().clear();
        }

        let interleaved = std::mem::take(&mut *self.input.buffer.lock().unwrap());
        let mono = downmix(&interleaved, self.input.channels);
        Ok(resample_linear(
            &mono,
            self.input.sample_rate,
            WHISPER_SAMPLE_RATE,
        ))
    }

    /// Drop all but the last `keep` samples so the idle buffer can't grow unbounded
    /// while we wait for a press.
    fn trim_to(&self, keep: usize) {
        let mut buf = self.input.buffer.lock().unwrap();
        let len = buf.len();
        if len > keep {
            buf.drain(0..len - keep);
        }
    }
}

/// Root-mean-square energy of a frame.
#[allow(dead_code)] // the copilot's Listener/mic-check use it; the lib tree doesn't
pub(crate) fn rms(frame: &[f32]) -> f32 {
    if frame.is_empty() {
        return 0.0;
    }
    let sum_sq: f32 = frame.iter().map(|s| s * s).sum();
    (sum_sq / frame.len() as f32).sqrt()
}

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
                b.extend(
                    data.iter()
                        .map(|&s| (s as f32 / u16::MAX as f32) * 2.0 - 1.0),
                );
            },
            err_fn,
            None,
        )?,
        other => return Err(anyhow!("unsupported sample format: {other:?}")),
    };
    Ok(stream)
}

/// Average interleaved channels down to mono.
pub(crate) fn downmix(interleaved: &[f32], channels: usize) -> Vec<f32> {
    if channels <= 1 {
        return interleaved.to_vec();
    }
    interleaved
        .chunks(channels)
        .map(|frame| frame.iter().sum::<f32>() / channels as f32)
        .collect()
}

/// Cheap linear resample. Fine for speech into whisper; not hi-fi.
pub(crate) fn resample_linear(input: &[f32], from: u32, to: u32) -> Vec<f32> {
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
