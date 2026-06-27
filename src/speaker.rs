//! Pilot voice recognition via the speaker-embedding sidecar.
//!
//! The sidecar ([`voice/speaker_sidecar.py`]) turns a clip into a speaker
//! embedding. We enrol the pilot's embedding once (the *voiceprint*) and then
//! accept an utterance only when its embedding is close enough (cosine
//! similarity) — so the copilot acts on the pilot and ignores X-Plane ATC /
//! ambient speech, with no wake word.
//!
//! Best-effort: if the sidecar is unreachable or no voiceprint is enrolled, the
//! caller falls back to the wake word, so this never blocks the loop.

use std::path::Path;

use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};

/// Where the enrolled voiceprint is stored (written by `learn-pilot-voice`).
pub const VOICEPRINT_PATH: &str = "models/pilot_voice.json";

/// Accept an utterance whose similarity to the voiceprint is at least this. From
/// the spike (Resemblyzer): same speaker ≈ 0.89, a synthetic ATC voice ≈ 0.54, so
/// ~0.70 sits in the gap with margin both ways. Tunable per mic/conditions via
/// `XCOPILOT_SPEAKER_THRESHOLD`. The production model may warrant a different value.
pub const DEFAULT_THRESHOLD: f32 = 0.70;

#[derive(Serialize, Deserialize)]
pub struct Voiceprint {
    /// The pilot's speaker embedding (L2-normalized by the sidecar).
    pub embedding: Vec<f32>,
}

impl Voiceprint {
    /// Load the enrolled voiceprint, or `None` if it doesn't exist / won't parse.
    #[allow(dead_code)] // used by the in-loop gate (next phase)
    pub fn load(path: &str) -> Option<Voiceprint> {
        serde_json::from_str(&std::fs::read_to_string(path).ok()?).ok()
    }

    pub fn save(&self, path: &str) -> Result<()> {
        if let Some(dir) = Path::new(path).parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        std::fs::write(path, serde_json::to_string(self)?).with_context(|| format!("writing {path}"))
    }
}

#[derive(Deserialize)]
struct EmbedResponse {
    embedding: Vec<f32>,
}

/// Embed 16 kHz mono f32 samples via the sidecar, returning the speaker vector.
pub async fn embed(client: &reqwest::Client, base_url: &str, samples: &[f32]) -> Result<Vec<f32>> {
    let bytes: Vec<u8> = samples.iter().flat_map(|s| s.to_le_bytes()).collect();
    let parsed: EmbedResponse = client
        .post(format!("{base_url}/embed"))
        .body(bytes)
        .send()
        .await
        .with_context(|| format!("POST {base_url}/embed (is the speaker sidecar running?)"))?
        .error_for_status()?
        .json()
        .await
        .context("parsing embedding response")?;
    if parsed.embedding.is_empty() {
        return Err(anyhow!("sidecar returned an empty embedding"));
    }
    Ok(parsed.embedding)
}

/// Cosine similarity of two embeddings (the sidecar L2-normalizes, but normalize
/// defensively so a non-normalized vector still compares sanely).
pub fn cosine(a: &[f32], b: &[f32]) -> f32 {
    if a.len() != b.len() || a.is_empty() {
        return 0.0;
    }
    let dot: f32 = a.iter().zip(b).map(|(x, y)| x * y).sum();
    let na = a.iter().map(|x| x * x).sum::<f32>().sqrt();
    let nb = b.iter().map(|x| x * x).sum::<f32>().sqrt();
    if na == 0.0 || nb == 0.0 {
        0.0
    } else {
        dot / (na * nb)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cosine_basics() {
        assert!((cosine(&[1.0, 0.0], &[1.0, 0.0]) - 1.0).abs() < 1e-6); // identical
        assert!(cosine(&[1.0, 0.0], &[0.0, 1.0]).abs() < 1e-6); // orthogonal
        assert!((cosine(&[1.0, 1.0], &[2.0, 2.0]) - 1.0).abs() < 1e-6); // scale-invariant
        assert_eq!(cosine(&[1.0], &[1.0, 2.0]), 0.0); // length mismatch → 0
    }
}
