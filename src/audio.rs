//! Mic capture via `cpal`, in two modes:
//!   - push-to-talk ([`record_push_to_talk`]): Enter to start, Enter to stop.
//!   - continuous ([`Listener`]): always capturing; a VAD [`Segmenter`] cuts the
//!     stream into utterances on speech/silence boundaries.
//!
//! Both deliver mono 16 kHz f32 ready for [`crate::stt`].

use std::io::{BufRead, Write};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use cpal::traits::StreamTrait;

// The low-level primitives live in `mic.rs` (shared with the atc binary via
// the lib target); re-exported here so copilot call sites keep their paths.
pub use crate::mic::{default_input_name, MicGate, PttMic, WHISPER_SAMPLE_RATE};
use crate::mic::{downmix, open_input, resample_linear, rms, Input};
use crate::ui::{LogKind, Reporter};

/// Map a config string to a push-to-talk key (`XCOPILOT_PTT_KEY`). A small set of
/// sensible PTT keys; returns `None` for an unknown name.
pub fn parse_ptt_key(s: &str) -> Option<device_query::Keycode> {
    use device_query::Keycode::*;
    let key = match s.to_ascii_lowercase().replace(['_', '-', ' '], "").as_str() {
        // Mac modifiers: Command = Meta, Option = Option (NOT Alt) in device_query.
        "rcmd" | "rightcommand" | "rightcmd" => RMeta,
        "lcmd" | "leftcommand" | "leftcmd" => LMeta,
        "ralt" | "roption" | "rightoption" | "rightalt" => ROption,
        "lalt" | "loption" | "leftoption" | "leftalt" => LOption,
        "rctrl" | "rightctrl" | "rcontrol" => RControl,
        "lctrl" | "leftctrl" | "lcontrol" => LControl,
        "rshift" | "rightshift" => RShift,
        "lshift" | "leftshift" => LShift,
        "space" => Space,
        "grave" | "backtick" => Grave,
        "f7" => F7,
        "f8" => F8,
        "f9" => F9,
        "f10" => F10,
        // Otherwise accept a raw device_query keycode name (e.g. "RMeta", "Command")
        // so any key found via `key-probe` works without a code change.
        _ => return s.parse().ok(),
    };
    Some(key)
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
    Ok(resample_linear(
        &mono,
        input.sample_rate,
        WHISPER_SAMPLE_RATE,
    ))
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
        let tag = if level > threshold {
            "SPEECH"
        } else {
            "quiet "
        };
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
    /// How long after speech ends to keep dropping audio (output latency + echo).
    gate_tail: Duration,
    /// Confirms a captured segment is speech before it's handed to whisper;
    /// rejects non-speech bursts (sim/engine audio, knocks) the energy gate let in.
    speech: crate::vad::SpeechFilter,
}

impl Listener {
    /// Start capturing, then calibrate the VAD to the room's noise floor so the
    /// speech threshold sits above ambient. `vad_floor` is a hard minimum (the
    /// configured `XCOPILOT_VAD_RMS`), so a quiet room can't drive it too low.
    pub fn start(
        vad_floor: f32,
        end_silence_ms: u32,
        reporter: Reporter,
        gate: MicGate,
        gate_tail: Duration,
    ) -> Result<Self> {
        let input = open_input()?;
        input.stream.play().context("starting input stream")?;

        reporter.log(
            LogKind::Info,
            "calibrating microphone — stay quiet for ~1s…",
        );
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
            gate_tail,
            speech: crate::vad::SpeechFilter::new(),
        })
    }

    /// Retune the capture gates without reopening the device (and without
    /// re-running the ambient calibration, whose result is the ADAPTIVE floor
    /// and is still valid — only the hard minimum moves). The settings screen
    /// calls this between utterances: tuning a mic gate by ear is why these
    /// are settings at all, and a restart per attempt made that unusable.
    pub fn retune(&mut self, vad_floor: f32, end_silence_ms: u32, gate_tail: Duration) {
        self.segmenter.retune(vad_floor, end_silence_ms);
        self.gate_tail = gate_tail;
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
                spoke_until = Some(Instant::now() + self.gate_tail);
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
                    let resampled =
                        resample_linear(&utterance, self.input.sample_rate, WHISPER_SAMPLE_RATE);
                    // Speech gate: only hand real speech to whisper. Non-speech
                    // bursts (sim/engine audio, knocks) are dropped, and we keep
                    // listening rather than transcribing noise into a command.
                    if self.speech.is_speech(&resampled) {
                        return Ok(resampled);
                    }
                    self.reporter
                        .log(LogKind::Info, "ignored (no speech detected)");
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

    /// Change the hard threshold minimum and the end-of-speech silence between
    /// utterances. The learned noise floor is deliberately kept: it is what
    /// the room measured, and a settings change is about the pilot's ear, not
    /// about the room having changed. `abs_min` is already the hard floor
    /// under [`Segmenter::threshold`], so a raise takes effect on the very
    /// next frame with nothing else to adjust.
    pub fn retune(&mut self, abs_min: f32, end_silence_ms: u32) {
        self.abs_min = abs_min;
        self.end_silence_ms = end_silence_ms;
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
        samples
            .chunks(frame_len)
            .filter(|c| c.len() == frame_len)
            .collect()
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
        audio.extend(std::iter::repeat_n(0.3, frame_len * 16));
        audio.extend(std::iter::repeat_n(0.0, frame_len * 33));

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
        assert!(
            secs > 0.4 && secs < 1.0,
            "unexpected utterance length: {secs}s"
        );
    }

    #[test]
    fn segmenter_ignores_short_blips() {
        let frame_len = 480;
        // require 300ms of speech to count.
        let mut seg = Segmenter::new(0.05, 2.5, 0.0, 90, 300, 100_000, 30);
        let mut audio = vec![0.0f32; frame_len * 5];
        audio.extend(std::iter::repeat_n(0.3, frame_len * 2)); // only 60ms of "speech"
        audio.extend(std::iter::repeat_n(0.0, frame_len * 10));

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
        audio.extend(std::iter::repeat_n(0.01, frame_len * 10)); // quiet pre-onset ambient
        audio.extend(std::iter::repeat_n(0.3, frame_len * 16)); // speech
        audio.extend(std::iter::repeat_n(0.0, frame_len * 10));

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
        assert!(
            utt[0].abs() < 0.05,
            "expected pre-roll before speech, got {}",
            utt[0]
        );
        // Pre-roll (~240 ms = 8 frames) pads ahead of the 16 speech frames.
        let captured = utt.len() / frame_len;
        assert!(
            captured >= 16 + 6,
            "expected pre-roll padding, got {captured} frames"
        );
    }
}
