//! Local LLM tool-picker, via the Ollama chat API (`POST /api/chat`).
//!
//! Cloud is deliberately out of scope (metered per call doesn't suit a real-time
//! copilot). This talks to a local Ollama server; the model must support
//! tool-calling (`llama3.1`, `qwen2.5`, …). We force the issue with a system
//! prompt and let the model emit a `tool_calls` entry, which we map back to one
//! of the curated tools.

use anyhow::{Context, Result};
use serde::Deserialize;
use serde_json::{json, Value};

use crate::config::Config;
use crate::tools::ToolSpec;

const SYSTEM_PROMPT: &str = "You are the copilot of an aircraft. The pilot speaks \
a short command. Choose exactly one tool that performs it, and pick the closest \
matching value from the tool's schema for each argument (speech may be slightly \
misheard — e.g. map 'beak' to 'beacon'). Numbers may be spoken as words. \
'descend' means a negative vertical speed; 'climb' means positive. Examples: \
'flaps ten' → set_flaps(notch=\"10\"); 'descend at five hundred' → \
set_vertical_speed(fpm=-500); 'climb and maintain seven thousand' → \
set_altitude(altitude=7000); 'beacon on' → set_lights(light=\"beacon\", on=true); \
'start her up' → start_engine. Do not answer in prose and do not invent tools. \
If no tool fits the command, do not call any tool.";

/// A tool the LLM chose, plus the parsed arguments.
#[derive(Debug)]
pub struct ToolCall {
    pub name: String,
    pub arguments: Value,
}

/// Thin client over a local Ollama server.
pub struct Llm {
    http: reqwest::Client,
    base: String,
    model: String,
}

#[derive(Deserialize)]
struct ChatResponse {
    message: ChatMessage,
}

#[derive(Deserialize)]
struct ChatMessage {
    #[serde(default)]
    tool_calls: Vec<OllamaToolCall>,
}

#[derive(Deserialize)]
struct OllamaToolCall {
    function: OllamaFunction,
}

#[derive(Deserialize)]
struct OllamaFunction {
    name: String,
    /// Ollama returns already-parsed arguments (an object), not a JSON string.
    #[serde(default)]
    arguments: Value,
}

impl Llm {
    pub fn new(config: &Config) -> Self {
        Self {
            http: reqwest::Client::new(),
            base: config.ollama_base_url.trim_end_matches('/').to_string(),
            model: config.model.clone(),
        }
    }

    /// Given a transcript and the available tools, ask the local model to pick
    /// one. Returns `None` if the model declined to call any tool.
    pub async fn choose_tool(
        &self,
        transcript: &str,
        tools: &[ToolSpec],
    ) -> Result<Option<ToolCall>> {
        let tool_specs: Vec<Value> = tools
            .iter()
            .map(|t| {
                json!({
                    "type": "function",
                    "function": {
                        "name": t.name,
                        "description": t.description,
                        "parameters": t.parameters,
                    }
                })
            })
            .collect();

        let body = json!({
            "model": self.model,
            "stream": false,
            // Deterministic tool selection: at default temperature qwen
            // intermittently emits no tool call for a clear command.
            "options": { "temperature": 0 },
            "messages": [
                { "role": "system", "content": SYSTEM_PROMPT },
                { "role": "user", "content": transcript },
            ],
            "tools": tool_specs,
        });

        let url = format!("{}/api/chat", self.base);
        let resp: ChatResponse = self
            .http
            .post(&url)
            .json(&body)
            .send()
            .await
            .with_context(|| format!("POST {url} (is `ollama serve` running?)"))?
            .error_for_status()?
            .json()
            .await
            .context("decoding Ollama chat response")?;

        Ok(resp.message.tool_calls.into_iter().next().map(|tc| ToolCall {
            name: tc.function.name,
            arguments: tc.function.arguments,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft;
    use crate::config::Config;
    use crate::tools::Registry;

    /// A minimal config pointing at the local Ollama. Model overridable via
    /// XCOPILOT_MODEL so we can compare models from the command line.
    fn test_config() -> Config {
        Config {
            ollama_base_url: "http://127.0.0.1:11434".into(),
            model: std::env::var("XCOPILOT_MODEL").unwrap_or_else(|_| "qwen2.5:7b-instruct".into()),
            xplane_base_url: "http://127.0.0.1:8086".into(),
            nav_data_path: String::new(),
            whisper_model_path: String::new(),
            listen_mode: "ptt".into(),
            wake_word: "copilot".into(),
            vad_threshold: 0.012,
            stt_min_confidence: 0.5,
            ui: "plain".into(),
            tts_backend: "say".into(),
            piper_binary: String::new(),
            piper_voice: String::new(),
            aircraft: "cessna_172sp".into(),
            confirm_critical: true,
        }
    }

    /// Live test against a running Ollama: does the local model map spoken
    /// commands to the right curated tool? Ignored by default (needs the server).
    /// Run with: `cargo test llm_picks_correct_tool -- --ignored --nocapture`
    #[tokio::test]
    #[ignore = "requires a local Ollama server with a tool-calling model"]
    async fn llm_picks_correct_tool() {
        let cfg = test_config();
        let llm = Llm::new(&cfg);
        let profile = aircraft::builtin(&cfg.aircraft).unwrap();
        let specs = Registry::for_aircraft(&profile).specs();

        // (utterance, expected tool, optional (arg key, expected value))
        let cases: &[(&str, &str, Option<(&str, Value)>)] = &[
            ("flaps up", "set_flaps", Some(("notch", json!("up")))),
            ("set flaps to ten degrees", "set_flaps", Some(("notch", json!("10")))),
            ("give me full flaps", "set_flaps", Some(("notch", json!("full")))),
            ("turn on the landing light", "set_lights", Some(("light", json!("landing")))),
            ("strobes off", "set_lights", Some(("on", json!(false)))),
            ("beacon on", "set_lights", Some(("light", json!("beacon")))),
            ("autopilot heading 270", "set_heading", None),
            ("fly heading 090", "set_heading", None),
            ("climb and maintain 7000", "set_altitude", None),
            ("set altitude 3000", "set_altitude", None),
            ("vertical speed 500 up", "set_vertical_speed", None),
            ("descend at 700 feet per minute", "set_vertical_speed", None),
            ("throttle 60 percent", "set_throttle", None),
            ("mixture full rich", "set_mixture", None),
            ("set altimeter 29.92", "set_barometer", None),
            ("com one 118.5", "set_com1", None),
            ("parking brake set", "set_parking_brake", Some(("on", json!(true)))),
            ("release the parking brake", "set_parking_brake", Some(("on", json!(false)))),
            ("start the engine", "start_engine", None),
            ("switch engines on", "start_engine", None),
            ("engine on", "start_engine", None),
            ("shut down the engine", "stop_engine", None),
            ("engine off", "stop_engine", None),
            ("battery on", "set_electrical", Some(("system", json!("battery")))),
            ("turn off the avionics", "set_electrical", Some(("system", json!("avionics")))),
            ("alternator on", "set_electrical", Some(("system", json!("alternator")))),
            ("switch on all electrical systems", "set_electrical", Some(("system", json!("all")))),
            ("squawk 1200", "set_transponder", Some(("code", json!(1200)))),
            ("set transponder to 4521", "set_transponder", Some(("code", json!(4521)))),
            ("prepare the ILS for runway 28", "prepare_ils", Some(("runway", json!("28")))),
            ("set up the ILS runway 13", "prepare_ils", None),
            ("align the gyro", "align_gyro", None),
            ("sync the heading indicator", "align_gyro", None),
        ];

        println!("\n  model: {}", cfg.model);
        let mut failures = 0;
        for (utterance, expected_tool, expected_arg) in cases {
            match llm.choose_tool(utterance, &specs).await.expect("ollama call failed") {
                Some(call) => {
                    let tool_ok = call.name == *expected_tool;
                    let arg_ok = expected_arg
                        .as_ref()
                        .map(|(k, v)| call.arguments.get(*k) == Some(v))
                        .unwrap_or(true);
                    let mark = if tool_ok && arg_ok { "✓" } else { "✗" };
                    println!("  {mark} {utterance:?} -> {} {}", call.name, call.arguments);
                    if !(tool_ok && arg_ok) {
                        failures += 1;
                    }
                }
                None => {
                    println!("  ✗ {utterance:?} -> (no tool chosen)");
                    failures += 1;
                }
            }
        }
        assert_eq!(failures, 0, "{failures} command(s) routed incorrectly");
    }

    /// Near-full pipeline against live systems: transcript → qwen picks a tool →
    /// the tool writes the dataref in a running X-Plane → readback. Only STT
    /// (whisper) and TTS (piper) are excluded. Side-effecting: moves the flaps
    /// and toggles the landing light; ends clean (flaps up, light off).
    ///
    /// Run with: `cargo test pipeline_against_live_xplane -- --ignored --nocapture`
    #[tokio::test]
    #[ignore = "requires a running X-Plane (Web API on :8086) and local Ollama"]
    async fn pipeline_against_live_xplane() {
        use crate::tools::ToolCtx;
        use crate::xplane::XPlaneClient;

        let cfg = test_config();
        let llm = Llm::new(&cfg);
        let xplane = XPlaneClient::new(cfg.xplane_base_url.clone());
        let profile = aircraft::builtin(&cfg.aircraft).unwrap();
        let registry = Registry::for_aircraft(&profile);
        let specs = registry.specs();
        let ctx = ToolCtx { xplane: &xplane, config: &cfg };

        println!("\n  model: {} | aircraft: {}\n", cfg.model, profile.display_name);

        // (utterance, substring the readback should contain)
        let commands = [
            ("flaps full", "full"),
            ("flaps up", "up"),
            ("landing light on", "on"),
            ("landing light off", "off"),
        ];

        for (utterance, expect) in commands {
            let call = llm
                .choose_tool(utterance, &specs)
                .await
                .expect("ollama call failed")
                .unwrap_or_else(|| panic!("no tool chosen for {utterance:?}"));
            let tool = registry
                .get(&call.name)
                .unwrap_or_else(|| panic!("LLM picked unknown tool {}", call.name));
            let readback = tool
                .execute(&call.arguments, &ctx)
                .await
                .expect("tool execution failed (is X-Plane on :8086?)");

            println!("  🗣️  {utterance:?}");
            println!("     → {} {}", call.name, call.arguments);
            println!("     🔊 {readback}\n");
            assert!(
                readback.to_lowercase().contains(expect),
                "readback {readback:?} missing {expect:?}"
            );
        }
    }
}
