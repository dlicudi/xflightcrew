//! Manual retrieval — index the aircraft's operating manuals so the answer-only
//! LLM can ground its answers in the actual POM.
//!
//! `x-copilot index <aircraft_id> <pdf...>` extracts each PDF's text (`pdftotext`),
//! chunks it, embeds every chunk with a local Ollama embedding model, and writes
//! `indexes/<aircraft_id>.json`. At question time the session loads the active
//! aircraft's index and [`Index::retrieve`]s the best chunks by cosine similarity;
//! they're handed to [`crate::llm::Llm::answer`] as MANUAL EXCERPTS. Everything is
//! local (the same no-cloud rule as the rest of the copilot), and it's answer-only
//! context — retrieval can never actuate anything.

use std::io::Write as _;
use std::path::Path;

use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};

/// The local Ollama embedding model. `XCOPILOT_EMBED_MODEL` overrides.
pub fn embed_model() -> String {
    std::env::var("XCOPILOT_EMBED_MODEL").unwrap_or_else(|_| "nomic-embed-text".to_string())
}

/// The shared, aircraft-independent index: the public-domain FAA handbooks
/// (PHAK, Airplane Flying Handbook, Instrument Flying/Procedures, Aviation
/// Weather, Risk Management, Weight & Balance). Consulted alongside the active
/// aircraft's POM index at question time, so "what is class B airspace" gets an
/// answer even when no POM covers it. Built like any other index:
/// `x-copilot index general_aviation manuals/faa/*.pdf`.
pub const GENERAL_ID: &str = "general_aviation";

/// Where an aircraft's index lives, relative to the working dir.
pub fn index_path(aircraft_id: &str) -> String {
    format!("indexes/{aircraft_id}.json")
}

/// Target chunk size (chars). Paragraph-packed up to this, so chunks follow the
/// manual's own structure instead of cutting mid-sentence.
const CHUNK_CHARS: usize = 900;
/// Chunks scoring below this cosine similarity are noise, not answers.
const MIN_SCORE: f32 = 0.45;

#[derive(Serialize, Deserialize)]
pub struct Chunk {
    /// Source label ("C90B POM p.—" style: file stem).
    pub source: String,
    pub text: String,
    pub embedding: Vec<f32>,
}

#[derive(Serialize, Deserialize)]
pub struct Index {
    /// Embedding model the chunks were built with — retrieval must match it.
    pub model: String,
    pub chunks: Vec<Chunk>,
}

impl Index {
    /// Load an aircraft's index; `None` if it hasn't been built.
    pub fn load(aircraft_id: &str) -> Option<Index> {
        let raw = std::fs::read_to_string(index_path(aircraft_id)).ok()?;
        serde_json::from_str(&raw).ok()
    }

    /// The best manual chunks for `question`, formatted for the answer prompt.
    /// `None` when nothing relevant is found (the LLM then answers as before).
    /// One small embedding call, then in-memory cosine ranking.
    pub async fn retrieve(&self, ollama_base: &str, question: &str, k: usize) -> Option<String> {
        let q = embed_one(ollama_base, &self.model, question).await.ok()?;
        let mut scored: Vec<(f32, &Chunk)> = self
            .chunks
            .iter()
            .map(|c| (cosine(&q, &c.embedding), c))
            .collect();
        scored.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        let hits: Vec<String> = scored
            .iter()
            .take(k)
            .filter(|(s, _)| *s >= MIN_SCORE)
            .map(|(_, c)| format!("[{}] {}", c.source, c.text))
            .collect();
        if hits.is_empty() {
            return None;
        }
        Some(hits.join("\n---\n"))
    }
}

fn cosine(a: &[f32], b: &[f32]) -> f32 {
    if a.len() != b.len() || a.is_empty() {
        return 0.0;
    }
    let (mut dot, mut na, mut nb) = (0.0f32, 0.0f32, 0.0f32);
    for (x, y) in a.iter().zip(b) {
        dot += x * y;
        na += x * x;
        nb += y * y;
    }
    dot / (na.sqrt() * nb.sqrt()).max(1e-9)
}

/// One embedding via Ollama's `/api/embeddings` (the older, universally supported
/// endpoint — one prompt per call).
async fn embed_one(base: &str, model: &str, text: &str) -> Result<Vec<f32>> {
    #[derive(Deserialize)]
    struct Resp {
        embedding: Vec<f32>,
    }
    let http = reqwest::Client::builder()
        .timeout(std::time::Duration::from_secs(30))
        .build()?;
    let resp: Resp = http
        .post(format!("{}/api/embeddings", base.trim_end_matches('/')))
        .json(&serde_json::json!({ "model": model, "prompt": text }))
        .send()
        .await
        .context("Ollama embeddings call failed — is Ollama running, and is the embedding model pulled?")?
        .error_for_status()
        .context("embedding model error — `ollama pull nomic-embed-text`?")?
        .json()
        .await
        .context("bad embeddings response")?;
    Ok(resp.embedding)
}

/// Pack the manual's paragraphs into chunks of roughly [`CHUNK_CHARS`], never
/// splitting a paragraph. Drops obvious junk (page numbers, empty runs).
pub fn chunk_text(text: &str) -> Vec<String> {
    let mut chunks = Vec::new();
    let mut cur = String::new();
    for para in text.split("\n\n") {
        let p: String = para
            .lines()
            .map(str::trim)
            .filter(|l| !l.is_empty())
            .collect::<Vec<_>>()
            .join(" ");
        // Junk: bare page numbers / single symbols.
        if p.len() < 4 || p.chars().all(|c| c.is_ascii_digit() || c.is_whitespace()) {
            continue;
        }
        if !cur.is_empty() && cur.len() + p.len() + 1 > CHUNK_CHARS {
            chunks.push(std::mem::take(&mut cur));
        }
        if !cur.is_empty() {
            cur.push(' ');
        }
        cur.push_str(&p);
        // A single huge paragraph still flushes (oversize chunk beats losing it).
        if cur.len() >= CHUNK_CHARS {
            chunks.push(std::mem::take(&mut cur));
        }
    }
    if !cur.is_empty() {
        chunks.push(cur);
    }
    chunks
}

/// Build (or rebuild) an aircraft's index from PDF manuals. Shells out to
/// `pdftotext` (poppler) for extraction. Prints progress — this is a CLI action.
pub async fn build_index(ollama_base: &str, aircraft_id: &str, pdfs: &[String]) -> Result<()> {
    let model = embed_model();
    let mut chunks: Vec<Chunk> = Vec::new();
    for pdf in pdfs {
        let path = Path::new(pdf);
        if !path.exists() {
            bail!("no such file: {pdf}");
        }
        let source = path
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| pdf.clone());
        println!("extracting {source}…");
        let out = std::process::Command::new("pdftotext")
            .arg(pdf)
            .arg("-") // stdout
            .output()
            .context("pdftotext not found — install poppler (`brew install poppler`)")?;
        if !out.status.success() {
            bail!("pdftotext failed on {pdf}");
        }
        let text = String::from_utf8_lossy(&out.stdout).to_string();
        let parts = chunk_text(&text);
        println!("  {} chunks; embedding with {model}…", parts.len());
        for (i, text) in parts.into_iter().enumerate() {
            // One retry (the model can 500 while first loading), then skip the
            // chunk with a warning — one bad page must not kill the whole index.
            let embedding = match embed_one(ollama_base, &model, &text).await {
                Ok(e) => e,
                Err(_) => match embed_one(ollama_base, &model, &text).await {
                    Ok(e) => e,
                    Err(e) => {
                        println!("  ! skipping chunk {i} ({e:#}): {}…", &text[..text.len().min(60)]);
                        continue;
                    }
                },
            };
            chunks.push(Chunk { source: source.clone(), text, embedding });
            if (i + 1) % 50 == 0 {
                println!("  …{} embedded", i + 1);
                let _ = std::io::stdout().flush();
            }
        }
    }
    std::fs::create_dir_all("indexes").ok();
    let index = Index { model, chunks };
    std::fs::write(index_path(aircraft_id), serde_json::to_string(&index)?)?;
    println!(
        "wrote {} ({} chunks). The copilot loads it automatically for '{}' when the LLM is on.",
        index_path(aircraft_id),
        index.chunks.len(),
        aircraft_id
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cosine_basics() {
        assert!((cosine(&[1.0, 0.0], &[1.0, 0.0]) - 1.0).abs() < 1e-6);
        assert!(cosine(&[1.0, 0.0], &[0.0, 1.0]).abs() < 1e-6);
        assert_eq!(cosine(&[], &[]), 0.0);
    }

    #[test]
    fn chunking_packs_paragraphs_and_drops_junk() {
        let text = "First paragraph about the fuel system.\n\n42\n\nSecond paragraph about \
                    engine start procedures with more detail.\n\n\n\nThird.";
        let chunks = chunk_text(text);
        // Page number "42" dropped; everything else packed into one small chunk.
        assert_eq!(chunks.len(), 1, "{chunks:?}");
        assert!(chunks[0].contains("fuel system") && chunks[0].contains("Third"));
        assert!(!chunks[0].contains(" 42 "));
        // Long input splits at ~CHUNK_CHARS without cutting a paragraph.
        let long = (0..40).map(|i| format!("Paragraph number {i} with some words.")).collect::<Vec<_>>().join("\n\n");
        let chunks = chunk_text(&long);
        assert!(chunks.len() > 1);
        assert!(chunks.iter().all(|c| c.len() <= CHUNK_CHARS + 120));
    }
}
