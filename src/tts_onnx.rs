//! In-process ONNX TTS (piper VITS) — behind the `onnx-tts` feature.
//!
//! The win over the `say`/`piper` *subprocess* backends is a **warm session**:
//! the voice model loads once (in the [`crate::voice`] speaker thread) and every
//! readback reuses it, removing the per-utterance model-reload tax that a
//! subprocess pays each call. `bench/tts_warm_vs_cold.py` measured that tax at
//! ~6× on this very model (~20 ms warm vs ~280 ms reload for "flaps up").
//!
//! Split of work: phonemes come from `espeak-ng` (shelled — cheap, the same
//! phonemizer piper uses); inference stays in-process via `ort` (the expensive
//! part, now warm). Output PCM is written to a WAV and handed to the existing
//! platform player ([`crate::tts::play`]).

use std::collections::HashMap;
use std::path::Path;
use std::process::{Command, Stdio};

use anyhow::{anyhow, bail, Context, Result};
use ort::session::Session;
use ort::value::Tensor;

/// A loaded piper VITS voice with a warm inference session.
pub struct OnnxTts {
    session: Session,
    /// `phoneme -> token id(s)`, from the voice's `.onnx.json`.
    phoneme_ids: HashMap<String, Vec<i64>>,
    sample_rate: u32,
    /// `[noise_scale, length_scale, noise_w]` — the VITS inference scales.
    scales: [f32; 3],
}

impl OnnxTts {
    /// Load a piper `.onnx` voice and its sibling `.onnx.json` once.
    pub fn load(model_path: &str) -> Result<Self> {
        // ort's builder error carries the (non-Send) builder back, so it can't go
        // through `?` into anyhow directly — map via Display first. Chained on the
        // temporary builder so no `mut` binding is needed.
        let session = Session::builder()
            .map_err(|e| anyhow!("ort session builder: {e}"))?
            .commit_from_file(model_path)
            .map_err(|e| anyhow!("loading ONNX voice {model_path}: {e}"))?;

        let cfg_path = format!("{model_path}.json");
        let cfg: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(&cfg_path).with_context(|| format!("reading {cfg_path}"))?,
        )
        .with_context(|| format!("parsing {cfg_path}"))?;

        let sample_rate = cfg["audio"]["sample_rate"]
            .as_u64()
            .ok_or_else(|| anyhow!("{cfg_path}: missing audio.sample_rate"))? as u32;

        let map = cfg["phoneme_id_map"]
            .as_object()
            .ok_or_else(|| anyhow!("{cfg_path}: missing phoneme_id_map"))?;
        let phoneme_ids = map
            .iter()
            .map(|(k, v)| {
                let ids = v
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter_map(serde_json::Value::as_i64)
                    .collect();
                (k.clone(), ids)
            })
            .collect();

        // VITS inference scales, from the voice's json but overridable by env so the
        // voice can be tuned live without re-vendoring the model. `noise_scale` =
        // timbre/pitch variation, `length_scale` = pace (lower is faster/peppier),
        // `noise_w` = rhythm variation. Nudging a flat voice: length down, noise_w up.
        let inf = &cfg["inference"];
        let scales = [
            env_scale("XCOPILOT_TTS_NOISE", inf["noise_scale"].as_f64().unwrap_or(0.667)),
            env_scale("XCOPILOT_TTS_LENGTH", inf["length_scale"].as_f64().unwrap_or(1.0)),
            env_scale("XCOPILOT_TTS_NOISE_W", inf["noise_w"].as_f64().unwrap_or(0.8)),
        ];

        Ok(Self { session, phoneme_ids, sample_rate, scales })
    }

    fn ids_of(&self, key: &str) -> Vec<i64> {
        self.phoneme_ids.get(key).cloned().unwrap_or_default()
    }

    /// `text -> token ids`, mirroring piper: BOS (`^`), then each phoneme followed
    /// by a pad (`_`), then EOS (`$`). Phonemes that aren't in the map are skipped.
    fn token_ids(&self, text: &str) -> Result<Vec<i64>> {
        let phonemes = phonemize(text)?;
        let pad = self.ids_of("_");
        let mut ids = self.ids_of("^");
        for ch in phonemes.chars() {
            if let Some(mapped) = self.phoneme_ids.get(&ch.to_string()) {
                ids.extend_from_slice(mapped);
                ids.extend_from_slice(&pad);
            }
        }
        ids.extend(self.ids_of("$"));
        if ids.len() <= 2 {
            bail!("no phonemes mapped for {text:?} (phonemized to {phonemes:?})");
        }
        Ok(ids)
    }

    /// Synthesize to mono `f32` samples in [-1, 1]. Reuses the warm session.
    pub fn synth(&mut self, text: &str) -> Result<Vec<f32>> {
        let ids = self.token_ids(text)?;
        let n = ids.len() as i64;
        let input = Tensor::from_array(([1_i64, n], ids))?;
        let lengths = Tensor::from_array(([1_i64], vec![n]))?;
        let scales = Tensor::from_array(([3_i64], self.scales.to_vec()))?;

        let outputs = self.session.run(ort::inputs![
            "input" => input,
            "input_lengths" => lengths,
            "scales" => scales,
        ])?;

        let (_shape, data) = outputs[0].try_extract_tensor::<f32>()?;
        Ok(data.to_vec())
    }

    /// Synthesize and play the readback. The session stays warm for the next call.
    pub fn speak(&mut self, text: &str) -> Result<()> {
        let samples = self.synth(text)?;
        let wav = std::env::temp_dir().join("x-copilot-onnx.wav");
        write_wav(&wav, &samples, self.sample_rate)?;
        crate::tts::play(&wav)
    }
}

/// A VITS scale: the env override (`name`) if set and parseable, else the voice's
/// own value. Lets the voice be tuned from `.env` without editing the model json.
fn env_scale(name: &str, default: f64) -> f32 {
    std::env::var(name).ok().and_then(|v| v.trim().parse::<f32>().ok()).unwrap_or(default as f32)
}

/// Phonemize with espeak-ng (IPA) — the phonemizer piper itself uses.
fn phonemize(text: &str) -> Result<String> {
    let out = Command::new("espeak-ng")
        .args(["-v", "en-us", "-q", "--ipa", text])
        .stderr(Stdio::null())
        .output()
        .context("running espeak-ng (install it: brew install espeak-ng)")?;
    if !out.status.success() {
        bail!("espeak-ng exited with {}", out.status);
    }
    Ok(String::from_utf8_lossy(&out.stdout).replace('\n', " ").trim().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    const MODEL: &str = "models/en_US-amy-medium.onnx";
    const PHRASES: [&str; 4] = [
        "Flaps up.",
        "Roger, heading two seven zero, autopilot engaged.",
        "Gear is down, three green.",
        "Caution, fuel endurance forty minutes.",
    ];

    // Needs the vendored model + espeak-ng, so off by default. Run with:
    //   cargo test --features onnx-tts -- --ignored --nocapture onnx
    #[test]
    #[ignore]
    fn onnx_warm_vs_cold() {
        let t0 = Instant::now();
        let mut tts = OnnxTts::load(MODEL).expect("load voice");
        let load_ms = t0.elapsed().as_secs_f64() * 1000.0;
        let _ = tts.synth(PHRASES[0]).expect("warm-up synth"); // JIT kernels
        println!("\none-time session load: {load_ms:7.1} ms");

        println!("{:<52}{:>8}{:>10}{:>10}", "phrase", "speech", "WARM", "COLD");
        let (mut wsum, mut csum) = (0.0, 0.0);
        for p in PHRASES {
            let t = Instant::now();
            let s = tts.synth(p).expect("warm synth");
            let wms = t.elapsed().as_secs_f64() * 1000.0;
            assert!(s.len() as u32 > tts.sample_rate / 5, "got <0.2s of audio for {p:?}");

            let t = Instant::now();
            let mut fresh = OnnxTts::load(MODEL).expect("reload");
            fresh.synth(p).expect("cold synth");
            let cms = t.elapsed().as_secs_f64() * 1000.0;

            let dur = s.len() as f64 / tts.sample_rate as f64;
            println!("  {p:<50}{dur:>7.2}s{wms:>8.1}ms{cms:>8.1}ms");
            wsum += wms;
            csum += cms;
        }
        let (wmean, cmean) = (wsum / 4.0, csum / 4.0);
        println!("  {:<50}{:>8}{wmean:>8.1}ms{cmean:>8.1}ms", "mean", "");
        println!("  → cold pays {:.1}x the warm cost (+{:.0} ms reload tax/utterance)\n",
                 cmean / wmean, cmean - wmean);

        // Write one out so the audio can be ear-checked.
        let wav = std::env::temp_dir().join("x-copilot-onnx-test.wav");
        let s = tts.synth("Roger, flaps up.").unwrap();
        write_wav(&wav, &s, tts.sample_rate).unwrap();
        println!("  wrote sample: {}", wav.display());
    }
}

/// Write mono `f32` samples as 16-bit PCM WAV.
fn write_wav(path: &Path, samples: &[f32], sample_rate: u32) -> Result<()> {
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut w = hound::WavWriter::create(path, spec)
        .with_context(|| format!("creating {}", path.display()))?;
    for &s in samples {
        w.write_sample((s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16)?;
    }
    w.finalize().context("finalizing wav")?;
    Ok(())
}
