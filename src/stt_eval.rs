//! Offline STT accuracy harness — the metric behind "understand human speech".
//!
//! Runs a manifest of cases through the *real* command pipeline (whisper →
//! [`crate::phrase`] normalization → [`crate::grammar`] fast-path) and reports
//! two numbers: word error rate (how close the transcript is to the truth) and
//! command accuracy (did it resolve to the right tool + args). Point it at a
//! different model with `WHISPER_MODEL=...` to compare base/small/turbo on the
//! same cases.
//!
//! Audio comes from a real recording (`audio:`) when given, else it's synthesized
//! with macOS `say`. **Synthesized speech is a floor, not a substitute for human
//! or cockpit audio** — it under-states the error rate. Treat the `say` cases as
//! a regression smoke test and drop real recordings into the manifest for numbers
//! that mean something.

use std::path::Path;
use std::process::Command;

use anyhow::{anyhow, Result};
use serde::Deserialize;
use serde_json::Value;

use crate::aircraft::AircraftProfile;
use crate::stt::Stt;
use crate::tools::Registry;
use crate::tools::ToolCall;
use crate::{grammar, phrase, stt};

#[derive(Deserialize)]
struct Case {
    /// Ground-truth spoken words ("flaps ten"). Synthesized via `say` when no
    /// `audio` is given, and used as the WER reference.
    say: String,
    /// Optional real recording (any format `afconvert` reads). Preferred over
    /// synthesis.
    #[serde(default)]
    audio: Option<String>,
    /// Optional tool the command should resolve to on the fast-path. Omit for a
    /// pure transcription (WER-only) case.
    #[serde(default)]
    tool: Option<String>,
    /// Optional expected arguments, subset-matched against the resolved call.
    #[serde(default)]
    args: Option<Value>,
    /// Optional expected ATC intent by label ("request_taxi", "unknown" — see
    /// [`crate::tower::intent_from_label`]). The case then decodes with the
    /// tower's STT prompt and is judged on [`crate::tower::parse_intent`]
    /// instead of the command fast-path: did the SPOKEN call still reach the
    /// right clearance?
    #[serde(default)]
    intent: Option<String>,
}

pub fn run(model_path: &str, profile: &AircraftProfile, manifest: &str) -> Result<()> {
    let text = std::fs::read_to_string(manifest).map_err(|e| anyhow!("reading {manifest}: {e}"))?;
    let cases: Vec<Case> =
        serde_yaml::from_str(&text).map_err(|e| anyhow!("parsing {manifest}: {e}"))?;
    if cases.is_empty() {
        return Err(anyhow!("{manifest} has no cases"));
    }

    // Same model + decoding bias the live loop uses, so the numbers transfer.
    let mut stt = Stt::load(model_path, true)?;
    stt.set_prompt(stt::build_prompt(profile));
    let registry = Registry::for_aircraft(profile);

    println!(
        "model {model_path} · {} · {} cases\n",
        profile.display_name,
        cases.len()
    );

    let mut wer_sum = 0.0;
    let (mut cmd_ok, mut cmd_total) = (0u32, 0u32);
    let (mut int_ok, mut int_total) = (0u32, 0u32);
    for case in &cases {
        let samples = match &case.audio {
            Some(p) => load_16k(Path::new(p))?,
            None => synth(&case.say)?,
        };
        // ATC cases decode with the tower's prompt, exactly as the live loop
        // does with COM1 on the station; command cases get the profile prompt.
        if case.intent.is_some() {
            stt.set_prompt(crate::tower::STT_PROMPT.to_string());
        } else {
            stt.set_prompt(stt::build_prompt(profile));
        }
        let heard = stt.transcribe(&samples)?.text.trim().to_lowercase();
        let reference = phrase::normalize_numbers(&case.say.to_lowercase());
        let wer = word_error_rate(&reference, &heard);
        wer_sum += wer;

        // Resolve the transcript through the deterministic fast-path, exactly as
        // the command loop would.
        let resolved = grammar::try_parse(&heard, &registry, profile);
        let verdict = match (&case.tool, &case.intent) {
            (None, None) => String::new(), // transcription-only case
            (Some(want), _) => {
                cmd_total += 1;
                let got = resolved.as_ref().and_then(|c| c.first());
                match got {
                    Some(c) if &c.name == want && args_match(case.args.as_ref(), c) => {
                        cmd_ok += 1;
                        format!("cmd:{want} OK")
                    }
                    Some(c) => format!("cmd:WRONG got {} {}", c.name, c.arguments),
                    None => "cmd:LLM-fallback".to_string(),
                }
            }
            (None, Some(label)) => {
                let want = crate::tower::intent_from_label(label)
                    .ok_or_else(|| anyhow!("{}: unknown intent label {label:?}", case.say))?;
                int_total += 1;
                let got = crate::tower::parse_intent(&heard);
                // The IFR label names the call, not its destination (the
                // corpus carries the destination; here it is the shape).
                let same = got == want
                    || matches!(
                        (&got, &want),
                        (
                            crate::tower::Intent::RequestIfrClearance { .. },
                            crate::tower::Intent::RequestIfrClearance { .. }
                        )
                    );
                if same {
                    int_ok += 1;
                    format!("atc:{label} OK")
                } else {
                    format!("atc:WRONG got {got:?}")
                }
            }
        };
        let judged = case.tool.is_some() || case.intent.is_some();
        let mark = if judged && !verdict.ends_with("OK") {
            "✗"
        } else {
            "✓"
        };
        println!(
            "{mark} {:<28} heard {:<30} wer {:.2}  {verdict}",
            quote(&case.say),
            quote(&heard),
            wer
        );
    }

    let mean_wer = wer_sum / cases.len() as f64;
    print!("\n— {} cases · mean WER {mean_wer:.3}", cases.len());
    if cmd_total > 0 {
        let pct = 100.0 * cmd_ok as f64 / cmd_total as f64;
        print!(" · command accuracy {cmd_ok}/{cmd_total} ({pct:.1}%)");
    }
    if int_total > 0 {
        let pct = 100.0 * int_ok as f64 / int_total as f64;
        print!(" · ATC intent accuracy {int_ok}/{int_total} ({pct:.1}%)");
    }
    println!();
    Ok(())
}

/// True if every key/value in `expected` is present and equal in the resolved
/// call. `None` expected → any args pass (the tool match alone is the assertion).
fn args_match(expected: Option<&Value>, call: &ToolCall) -> bool {
    let Some(expected) = expected else {
        return true;
    };
    match (expected.as_object(), call.arguments.as_object()) {
        (Some(exp), Some(got)) => exp.iter().all(|(k, v)| got.get(k) == Some(v)),
        _ => expected == &call.arguments,
    }
}

/// Word error rate: Levenshtein edit distance over tokens / reference length.
/// Tokens are stripped of edge punctuation first ("10." == "10", "80%" == "80"),
/// since the grammar ignores it too — so WER reflects words, not formatting.
/// 0.0 is perfect; insertions can push it above 1.0.
fn word_error_rate(reference: &str, hypothesis: &str) -> f64 {
    fn tokens(s: &str) -> Vec<&str> {
        s.split_whitespace()
            .map(|w| w.trim_matches(|c: char| !c.is_ascii_alphanumeric()))
            .filter(|w| !w.is_empty())
            .collect()
    }
    let r = tokens(reference);
    let h = tokens(hypothesis);
    if r.is_empty() {
        return if h.is_empty() { 0.0 } else { 1.0 };
    }
    let mut prev: Vec<usize> = (0..=h.len()).collect();
    let mut cur = vec![0usize; h.len() + 1];
    for (i, rw) in r.iter().enumerate() {
        cur[0] = i + 1;
        for (j, hw) in h.iter().enumerate() {
            let cost = usize::from(rw != hw);
            cur[j + 1] = (prev[j] + cost).min(prev[j + 1] + 1).min(cur[j] + 1);
        }
        std::mem::swap(&mut prev, &mut cur);
    }
    prev[h.len()] as f64 / r.len() as f64
}

/// Synthesize `text` to 16 kHz mono via macOS `say`. A floor, not human speech.
/// Retried: the speech daemon intermittently drops a request when called
/// back-to-back over a long manifest.
fn synth(text: &str) -> Result<Vec<f32>> {
    let aiff = std::env::temp_dir().join("xflightcrew-stt-eval.aiff");
    for attempt in 0..3 {
        if attempt > 0 {
            std::thread::sleep(std::time::Duration::from_millis(500));
        }
        let status = Command::new("say")
            .args(["-o", aiff.to_str().unwrap(), text])
            .status()
            .map_err(|e| anyhow!("spawning say: {e}"))?;
        if status.success() {
            return load_16k(&aiff);
        }
    }
    Err(anyhow!("say failed for {text:?} (3 attempts)"))
}

/// Decode any `afconvert`-readable file to 16 kHz mono f32 (what whisper wants).
fn load_16k(input: &Path) -> Result<Vec<f32>> {
    if !input.exists() {
        return Err(anyhow!("no such file: {}", input.display()));
    }
    let wav = std::env::temp_dir().join("xflightcrew-stt-eval-16k.wav");
    let status = Command::new("afconvert")
        .args([
            input.to_str().unwrap(),
            wav.to_str().unwrap(),
            "-d",
            "LEI16@16000",
            "-c",
            "1",
            "-f",
            "WAVE",
        ])
        .status()
        .map_err(|e| anyhow!("spawning afconvert: {e}"))?;
    if !status.success() {
        return Err(anyhow!("afconvert failed to decode {}", input.display()));
    }
    let reader =
        hound::WavReader::open(&wav).map_err(|e| anyhow!("reading {}: {e}", wav.display()))?;
    reader
        .into_samples::<i16>()
        .map(|s| s.map(|v| v as f32 / i16::MAX as f32))
        .collect::<std::result::Result<_, _>>()
        .map_err(|e| anyhow!("decoding samples: {e}"))
}

/// Trim a string to a fixed width for the aligned table, quoting it.
fn quote(s: &str) -> String {
    let q = format!("\"{s}\"");
    if q.chars().count() > 28 {
        format!("{}…\"", q.chars().take(27).collect::<String>())
    } else {
        q
    }
}
