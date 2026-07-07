//! COM-radio transmission effect for the ATC station's voice.
//!
//! A real VHF airband transmission is narrowband AM: roughly 300 Hz to 3 kHz,
//! compressed hot, with a noise floor and a squelch burst when the carrier
//! drops. Recreating that cheaply sells "this is a radio" harder than voice
//! quality does. Pure sample-in/sample-out DSP — no deps, no clock, no rand
//! (a fixed LCG supplies the noise), so it unit-tests deterministically.

/// Apply the full transmission chain in place: key clicks, band-pass
/// 300-3000 Hz, soft compression, noise floor, peak normalize.
pub fn radioize(samples: &mut Vec<f32>, sample_rate: u32) {
    if samples.is_empty() {
        return;
    }
    let fs = sample_rate as f32;
    let mut noise = Lcg::new(0x5EED_CAFE);

    // Key-down click and squelch tail: short noise bursts spliced around the
    // speech, band-passed along with it so they sit in the same spectrum.
    let key_click = burst(&mut noise, (fs * 0.02) as usize, 0.25);
    let squelch = burst(&mut noise, (fs * 0.035) as usize, 0.35);
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

    // Hot mic: drive into a soft clip, then normalize the peak.
    for s in buf.iter_mut() {
        *s = (*s * 2.2).tanh();
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

/// A fading noise burst (the squelch/key transient).
fn burst(noise: &mut Lcg, len: usize, amp: f32) -> Vec<f32> {
    (0..len)
        .map(|i| {
            let fade = 1.0 - i as f32 / len.max(1) as f32;
            noise.next_f32() * amp * fade
        })
        .collect()
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
        // The squelch tail is audible noise, not silence.
        let tail = &s[s.len() - (fs as usize * 25 / 1000)..];
        assert!(rms(tail) > 0.005, "squelch tail present, rms {}", rms(tail));
    }

    #[test]
    fn empty_input_is_a_no_op() {
        let mut s: Vec<f32> = Vec::new();
        radioize(&mut s, 22050);
        assert!(s.is_empty());
    }
}
