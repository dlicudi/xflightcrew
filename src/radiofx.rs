//! COM-radio transmission effect for the ATC station's voice.
//!
//! A real VHF airband transmission is narrowband AM: roughly 300 Hz to 3 kHz,
//! compressed hot, with a noise floor and a squelch burst when the carrier
//! drops. Recreating that cheaply sells "this is a radio" harder than voice
//! quality does. Pure sample-in/sample-out DSP — no deps, no clock, no rand
//! (a fixed LCG supplies the noise), so it unit-tests deterministically.

/// Apply the full transmission chain in place: key clicks, band-pass
/// 300-3000 Hz, soft compression, noise floor, peak normalize.
#[allow(dead_code)] // the copilot always varies per transmission; kept for the lib/tests
pub fn radioize(samples: &mut Vec<f32>, sample_rate: u32) {
    radioize_variant(samples, sample_rate, 0);
}

/// [`radioize`] with a transmission `variant`: it picks this transmission's
/// random character — carrier gap, burst length and level, click strength,
/// noise seed. Real squelch never sounds the same twice, and an identical
/// tail on every reply is the loudest "this is fake" tell (the same lesson
/// behind Audio-for-VATSIM blending recorded, varying transients). Fully
/// deterministic per variant, so tests stay exact.
pub fn radioize_variant(samples: &mut Vec<f32>, sample_rate: u32, variant: u32) {
    radioize_core(samples, sample_rate, variant, true);
}

/// The VATSIM-style version: the speech goes through the transmission chain,
/// then a RECORDED squelch transient (e.g. AFV-Native's `Click_f32.wav`, the
/// click every VATSIM pilot knows) is spliced onto the end, level- and
/// gap-varied per transmission. The recording is appended *after* processing:
/// the click is the receiver's own gate closing, not part of the transmitted
/// audio, so it must not be band-passed or compressed with the speech.
/// `tail` must already be mono at `sample_rate` (see [`load_tail_wav`]).
pub fn radioize_with_tail(samples: &mut Vec<f32>, sample_rate: u32, variant: u32, tail: &[f32]) {
    if samples.is_empty() {
        return;
    }
    radioize_core(samples, sample_rate, variant, false);
    let fs = sample_rate as f32;
    let mut pick = Lcg::new(0x0000_7A11 ^ variant.wrapping_mul(0x9E37_79B9));
    let mut range = |lo: f32, hi: f32| lo + (pick.next_f32() + 1.0) * 0.5 * (hi - lo);
    let gap = (fs * range(0.04, 0.09)) as usize;
    let gain = range(0.35, 0.55); // vs the peak-normalized speech
    samples.resize(samples.len() + gap, 0.0);
    samples.extend(tail.iter().map(|s| s * gain));
}

/// Read a squelch recording for [`radioize_with_tail`]: any WAV (float or
/// int samples), downmixed to mono, resampled to `target_rate`, peak
/// normalized so the per-transmission gain lands predictably.
pub fn load_tail_wav(path: &str, target_rate: u32) -> anyhow::Result<Vec<f32>> {
    use anyhow::Context as _;
    let mut reader = hound::WavReader::open(path).with_context(|| format!("open {path}"))?;
    let spec = reader.spec();
    let raw: Vec<f32> = match spec.sample_format {
        hound::SampleFormat::Float => reader.samples::<f32>().filter_map(Result::ok).collect(),
        hound::SampleFormat::Int => {
            let scale = (1_i64 << (spec.bits_per_sample - 1)) as f32;
            reader.samples::<i32>().filter_map(Result::ok).map(|s| s as f32 / scale).collect()
        }
    };
    let mono = crate::mic::downmix(&raw, spec.channels as usize);
    let mut out = crate::mic::resample_linear(&mono, spec.sample_rate, target_rate);
    let peak = out.iter().fold(0.0_f32, |m, s| m.max(s.abs()));
    if peak > 0.0 {
        for s in out.iter_mut() {
            *s /= peak;
        }
    }
    Ok(out)
}

fn radioize_core(samples: &mut Vec<f32>, sample_rate: u32, variant: u32, synth_tail: bool) {
    if samples.is_empty() {
        return;
    }
    let fs = sample_rate as f32;
    let mut pick = Lcg::new(0x000A_11CE ^ variant.wrapping_mul(0x9E37_79B9));
    let mut noise = Lcg::new(0x5EED_CAFE ^ variant.wrapping_mul(0x85EB_CA6B));
    // Uniform in [lo, hi] off the picker stream.
    let mut range = |lo: f32, hi: f32| lo + (pick.next_f32() + 1.0) * 0.5 * (hi - lo);

    // Key-down click and squelch tail, band-passed along with the speech so
    // they sit in the same spectrum. The tail is the signature sound: the
    // carrier stays up for a beat after the last word, then the receiver's
    // squelch closes with a smoothed hiss burst that cuts hard.
    let key_click = burst(&mut noise, (fs * range(0.008, 0.018)) as usize, range(0.07, 0.16));
    let squelch = if synth_tail {
        squelch_tail(
            &mut noise,
            fs,
            range(0.05, 0.09),  // carrier hangs open this long after the last word
            range(0.035, 0.07), // burst decay body
            range(0.16, 0.28),  // burst level
        )
    } else {
        Vec::new() // a recorded tail is spliced on after processing instead
    };
    let mut buf = Vec::with_capacity(key_click.len() + samples.len() + squelch.len());
    buf.extend_from_slice(&key_click);
    buf.append(samples);
    buf.extend_from_slice(&squelch);

    // Noise floor under everything (the open carrier hiss).
    for s in buf.iter_mut() {
        *s += noise.next_f32() * 0.008;
    }

    // Narrowband: high-pass at 300 Hz, low-pass at 3 kHz (RBJ biquads).
    let mut hp = Biquad::highpass(fs, 300.0);
    let mut lp = Biquad::lowpass(fs, 3000.0);
    for s in buf.iter_mut() {
        *s = lp.process(hp.process(*s));
    }

    // Hot mic: drive into a soft clip, then normalize the peak. Moderate
    // drive — 2.2 flattened the voice's dynamics into a drone; 1.7 keeps the
    // compression character without ironing the speech flat.
    for s in buf.iter_mut() {
        *s = (*s * 1.7).tanh();
    }
    let peak = buf.iter().fold(0.0_f32, |m, s| m.max(s.abs()));
    if peak > 0.0 {
        let g = 0.85 / peak;
        for s in buf.iter_mut() {
            *s *= g;
        }
    }

    *samples = buf;
}

/// A fading noise burst (the key-down transient).
fn burst(noise: &mut Lcg, len: usize, amp: f32) -> Vec<f32> {
    (0..len)
        .map(|i| {
            let fade = 1.0 - i as f32 / len.max(1) as f32;
            noise.next_f32() * amp * fade
        })
        .collect()
}

/// The squelch closing after the transmission: `carrier_s` of open carrier
/// (just the hiss floor), then a smoothed noise burst — 3 ms attack,
/// exponential decay over `body_s` — ending in a hard cut. Smoothing the
/// noise (one-pole average) turns a harsh crackle into the classic soft
/// "psshht"; the abrupt end IS the squelch gate slamming shut, so no fade
/// after it.
fn squelch_tail(noise: &mut Lcg, fs: f32, carrier_s: f32, body_s: f32, amp: f32) -> Vec<f32> {
    let carrier = (fs * carrier_s) as usize;
    let attack = (fs * 0.003) as usize;
    let body = (fs * body_s) as usize;
    let mut out = Vec::with_capacity(carrier + attack + body);
    // Open carrier: nothing but the floor (added globally later).
    out.resize(carrier, 0.0);
    // Smoothed hiss: fast ramp in, exponential decay, hard stop.
    let mut smooth = 0.0_f32;
    for i in 0..attack + body {
        smooth = 0.55 * smooth + 0.45 * noise.next_f32();
        let env = if i < attack {
            i as f32 / attack.max(1) as f32
        } else {
            (-4.0 * (i - attack) as f32 / body.max(1) as f32).exp()
        };
        out.push(smooth * amp * env);
    }
    out
}

/// Deterministic white-ish noise: a fixed-seed LCG, so tests are exact and no
/// rand dependency is needed.
struct Lcg(u32);

impl Lcg {
    fn new(seed: u32) -> Self {
        Lcg(seed)
    }
    /// Uniform in [-1, 1].
    fn next_f32(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (self.0 >> 8) as f32 / (1 << 23) as f32 - 1.0
    }
}

/// One RBJ-cookbook biquad section (Q = 0.707, Butterworth-ish).
struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Biquad {
    fn from_coeffs(b0: f32, b1: f32, b2: f32, a0: f32, a1: f32, a2: f32) -> Self {
        Biquad {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    fn lowpass(fs: f32, f: f32) -> Self {
        let (sin, cos) = (2.0 * std::f32::consts::PI * f / fs).sin_cos();
        let alpha = sin / (2.0 * 0.7071);
        Self::from_coeffs(
            (1.0 - cos) / 2.0,
            1.0 - cos,
            (1.0 - cos) / 2.0,
            1.0 + alpha,
            -2.0 * cos,
            1.0 - alpha,
        )
    }

    fn highpass(fs: f32, f: f32) -> Self {
        let (sin, cos) = (2.0 * std::f32::consts::PI * f / fs).sin_cos();
        let alpha = sin / (2.0 * 0.7071);
        Self::from_coeffs(
            (1.0 + cos) / 2.0,
            -(1.0 + cos),
            (1.0 + cos) / 2.0,
            1.0 + alpha,
            -2.0 * cos,
            1.0 - alpha,
        )
    }

    fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sine(freq: f32, fs: f32, secs: f32) -> Vec<f32> {
        (0..(fs * secs) as usize)
            .map(|i| (2.0 * std::f32::consts::PI * freq * i as f32 / fs).sin() * 0.5)
            .collect()
    }

    fn rms(s: &[f32]) -> f32 {
        (s.iter().map(|x| x * x).sum::<f32>() / s.len() as f32).sqrt()
    }

    #[test]
    fn bandpass_kills_rumble_keeps_voice() {
        let fs = 22050.0;
        // Filter response measured directly (radioize adds noise/normalize,
        // which would mask it): 1 kHz passes, 60 Hz rumble dies.
        let run = |freq: f32| {
            let mut hp = Biquad::highpass(fs, 300.0);
            let mut lp = Biquad::lowpass(fs, 3000.0);
            let out: Vec<f32> = sine(freq, fs, 0.5).iter().map(|&x| lp.process(hp.process(x))).collect();
            rms(&out[out.len() / 2..]) // steady state
        };
        let voice = run(1000.0);
        let rumble = run(60.0);
        let hiss = run(8000.0);
        assert!(voice > 0.25, "1 kHz should pass, rms {voice}");
        assert!(rumble < voice * 0.1, "60 Hz should die: {rumble} vs {voice}");
        assert!(hiss < voice * 0.2, "8 kHz should die: {hiss} vs {voice}");
    }

    #[test]
    fn radioize_adds_squelch_and_stays_in_range() {
        let fs = 22050;
        let mut s = sine(1000.0, fs as f32, 0.3);
        let before = s.len();
        radioize(&mut s, fs);
        assert!(s.len() > before, "key click + squelch tail extend the audio");
        let peak = s.iter().fold(0.0_f32, |m, x| m.max(x.abs()));
        assert!(peak <= 0.9 && peak > 0.5, "normalized hot but unclipped: {peak}");
        // The squelch burst (last ~55 ms: attack + decay body) is audible
        // noise, not silence.
        let tail = &s[s.len() - (fs as usize * 55 / 1000)..];
        assert!(rms(tail) > 0.005, "squelch tail present, rms {}", rms(tail));
    }

    #[test]
    fn transmissions_never_sound_identical() {
        let fs = 22050;
        let base = sine(1000.0, fs as f32, 0.3);
        let (mut a, mut b) = (base.clone(), base.clone());
        radioize_variant(&mut a, fs, 1);
        radioize_variant(&mut b, fs, 2);
        // Different variants pick different gaps/bodies, so even the lengths
        // differ; both still normalize into range.
        assert_ne!(a.len(), b.len(), "two transmissions with identical tails");
        for s in [&a, &b] {
            let peak = s.iter().fold(0.0_f32, |m, x| m.max(x.abs()));
            assert!(peak <= 0.9 && peak > 0.5, "normalized: {peak}");
        }
    }

    #[test]
    fn recorded_tail_is_spliced_clean_after_the_gap() {
        let fs = 22050;
        let mut s = sine(1000.0, fs as f32, 0.3);
        let tail: Vec<f32> = (0..800).map(|i| if i % 2 == 0 { 1.0 } else { -1.0 }).collect();
        let mut plain = s.clone();
        radioize_core(&mut plain, fs, 7, false);
        radioize_with_tail(&mut s, fs, 7, &tail);
        assert!(s.len() > plain.len() + tail.len(), "gap + tail appended");
        // The tail is NOT band-passed or normalized with the speech: the last
        // samples are exactly the recording at the per-variant gain.
        let gain = s.last().unwrap() / tail.last().unwrap();
        assert!((0.35..=0.55).contains(&gain.abs()), "tail gain in range: {gain}");
        let spliced = &s[s.len() - tail.len()..];
        for (a, b) in spliced.iter().zip(&tail) {
            assert!((a - b * gain).abs() < 1e-6, "tail must be verbatim, scaled");
        }
    }

    #[test]
    fn empty_input_is_a_no_op() {
        let mut s: Vec<f32> = Vec::new();
        radioize(&mut s, 22050);
        assert!(s.is_empty());
    }
}
