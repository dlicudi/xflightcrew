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
a short command. Choose the tool(s) that perform it — one tool call per distinct \
action, so a command naming two things ('taxi and nav lights on') makes two \
calls. Pick the closest \
matching value from the tool's schema for each argument (speech may be slightly \
misheard — e.g. map 'beak' to 'beacon', 'probe'/'probe lights' to 'strobe', and \
'blood lights'/'bloodlights' to the flood lights). Numbers may be spoken as words. \
'descend' means a negative vertical speed; 'climb' means positive. Examples: \
'flaps ten' → set_flaps(notch=\"10\"); 'descend at five hundred' → \
set_vertical_speed(fpm=-500); 'climb and maintain seven thousand' → \
set_altitude(altitude=7000); 'beacon on' → set_lights(light=\"beacon\", on=true); \
'start her up' → start_engine. A bare number or percentage on its own, with no \
named system, CONTINUES the most recent command using the same tool — e.g. after \
'throttle fifty percent', 'seventy percent' → set_throttle(percent=70), and \
'sixty' → set_throttle(percent=60); after 'set heading 270', 'two eighty' → \
set_heading(heading=280). Do not answer in prose and do not invent tools. \
If no tool fits the command, do not call any tool.";

/// Temperatures to retry at, in order, when the deterministic (temp 0) first
/// pass returns no tool for an utterance the domain gate already accepted as a
/// command. A little entropy escapes the greedy-decode dead-end where qwen emits
/// no tool_call; we stop at the first concrete pick.
///
/// Values chosen empirically (see the `retry_temp_sweep` probe): recovery peaks
/// at a *small* bump (~0.3) and *drops* as temperature rises — high temp just
/// adds noise and wrong/empty picks. So both retries stay low. The second draw
/// at 0.5 is the only temperature observed to recover "sync the heading
/// indicator", so it earns its place as a diverse second attempt. Two retries
/// also bounds the worst-case added latency (the bump only runs when the first
/// pass already failed, which is rare).
const RETRY_TEMPS: &[f64] = &[0.3, 0.5];

/// A tool the LLM chose, plus the parsed arguments.
#[derive(Debug)]
pub struct ToolCall {
    pub name: String,
    pub arguments: Value,
}

/// Collapse conflicting calls the model sometimes emits for one utterance — e.g.
/// `set_flaps(10)`, `set_flaps(20)`, `set_flaps(full)` from a garbled input. A
/// single-control tool can only hold one setting, so a later call overwrites the
/// earlier: keep the LAST. Tools that ADDRESS a target — `set_lights` by `light`,
/// `set_electrical` by `system`, `swap_radio` by `radio` — are deduped per
/// (tool, target) instead, so a real compound ("landing and taxi lights on")
/// keeps both. Order is preserved.
fn dedup_calls(calls: Vec<ToolCall>) -> Vec<ToolCall> {
    const TARGET_KEYS: &[&str] = &["light", "system", "radio"];
    let key = |c: &ToolCall| -> String {
        let target = TARGET_KEYS
            .iter()
            .find_map(|k| c.arguments.get(*k).map(|v| v.to_string()))
            .unwrap_or_default();
        format!("{}|{target}", c.name)
    };
    // Walk from the end keeping the first (= last) occurrence of each key, then
    // restore original order.
    let mut seen = std::collections::HashSet::new();
    let mut keep = vec![false; calls.len()];
    for (i, c) in calls.iter().enumerate().rev() {
        if seen.insert(key(c)) {
            keep[i] = true;
        }
    }
    calls.into_iter().zip(keep).filter_map(|(c, k)| k.then_some(c)).collect()
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
    /// one. Returns `None` only if the model still declines after the retries.
    ///
    /// The first pass is deterministic (temperature 0): it's correct for the
    /// vast majority of commands and *repeatable*, so a command that works keeps
    /// working. But qwen2.5:7b intermittently emits **no tool call** for a clear,
    /// in-domain command — greedy decoding lands on the "no tool" path for
    /// certain phrasings (sensitive to casing/punctuation/articles in a way that
    /// looks arbitrary). The caller only invokes us *after* the domain gate has
    /// classified the utterance as a command, so an empty result there is almost
    /// always the model fumbling, not a genuine "nothing fits". We give it
    /// another look with a little sampling entropy to escape that dead-end —
    /// applied **only on retry**, so the common case stays deterministic.
    ///
    /// `history` is recent `(spoken command, readback)` turns, oldest first, so
    /// the model can resolve back-references ("switch *it* back", "a bit more")
    /// against what was just done. Pass an empty slice for a context-free pick.
    pub async fn choose_tool(
        &self,
        transcript: &str,
        tools: &[ToolSpec],
        history: &[(String, String)],
    ) -> Result<Vec<ToolCall>> {
        // Deterministic first pass.
        let calls = self.choose_tool_at(transcript, tools, history, 0.0).await?;
        if !calls.is_empty() {
            return Ok(dedup_calls(calls));
        }
        // Declined — retry with escalating temperature, stopping at the first
        // concrete pick.
        for &temp in RETRY_TEMPS {
            tracing::debug!(temp, transcript = %transcript, "tool selection empty; retrying with temperature bump");
            let calls = self.choose_tool_at(transcript, tools, history, temp).await?;
            if !calls.is_empty() {
                tracing::info!(temp, transcript = %transcript, n = calls.len(), "recovered tool(s) on retry");
                return Ok(dedup_calls(calls));
            }
        }
        Ok(Vec::new())
    }

    /// One tool-selection request at a given sampling temperature. `temperature`
    /// = 0.0 is greedy/deterministic; a small positive value adds entropy.
    async fn choose_tool_at(
        &self,
        transcript: &str,
        tools: &[ToolSpec],
        history: &[(String, String)],
        temperature: f64,
    ) -> Result<Vec<ToolCall>> {
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

        // system, then the recent turns as user/assistant pairs, then the new
        // command — so the model reads the current utterance in context.
        let mut messages = vec![json!({ "role": "system", "content": SYSTEM_PROMPT })];
        for (user, assistant) in history {
            messages.push(json!({ "role": "user", "content": user }));
            messages.push(json!({ "role": "assistant", "content": assistant }));
        }
        messages.push(json!({ "role": "user", "content": transcript }));

        let mut body = json!({
            "model": self.model,
            "stream": false,
            // Keep the model resident on the GPU between commands. A copilot is
            // used in bursts across a whole flight; Ollama's default 5-minute
            // eviction would cold-load the model (multi-GB into GPU memory) on
            // the first command after any quiet stretch — exactly the latency
            // spike we can't afford mid-flight. An hour stays warm through long
            // cruise legs while still freeing the GPU if the app is left idle.
            "keep_alive": "60m",
            "options": {
                // Temperature is supplied by the caller: 0 for the deterministic
                // first pass, a bump on retry (see `choose_tool`). At the model's
                // *default* temperature qwen intermittently emits no tool call for
                // a clear command, which is why the first pass pins it to 0.
                "temperature": temperature,
                // Our prompt is only the system instruction + tool schemas + one
                // short utterance. Without this, Ollama loads the model's full
                // trained context (32K) and reserves KV-cache memory we never
                // touch. Pin it small to cut that overhead. 8192 leaves generous
                // headroom over the curated tool list, so no tool definition is
                // ever truncated — truncation would silently degrade selection.
                "num_ctx": 8192,
            },
            "messages": messages,
            "tools": tool_specs,
        });

        // Reasoning ("thinking") models like Qwen3 deliberate before answering,
        // which is dead weight for a one-shot tool pick and adds latency this
        // real-time copilot can't spend. Disable it for the qwen3 *thinking*
        // family via Ollama's `think: false`. We match the `qwen3:` prefix on
        // purpose — it catches qwen3:8b/14b/etc. but NOT `qwen3-coder`, which is
        // a non-reasoning model that would reject the `think` field. The
        // `XCOPILOT_NO_THINK=1` override forces it for benchmarking other
        // thinking models. Only send `think` to models that support it.
        if self.model.starts_with("qwen3:")
            || std::env::var("XCOPILOT_NO_THINK").as_deref() == Ok("1")
        {
            body["think"] = json!(false);
        }

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

        Ok(resp
            .message
            .tool_calls
            .into_iter()
            .map(|tc| ToolCall {
                name: tc.function.name,
                arguments: tc.function.arguments,
            })
            .collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft;
    use crate::config::Config;
    use crate::tools::Registry;

    #[test]
    fn dedup_collapses_conflicting_keeps_addressable() {
        let c = |name: &str, args: Value| ToolCall { name: name.into(), arguments: args };
        let out = dedup_calls(vec![
            c("set_flaps", json!({ "notch": "10" })),
            c("set_flaps", json!({ "notch": "20" })),
            c("set_flaps", json!({ "notch": "full" })),
            c("set_lights", json!({ "light": "landing", "on": true })),
            c("set_lights", json!({ "light": "taxi", "on": true })),
        ]);
        // One flaps (the LAST setting wins) + both lights (different targets).
        assert_eq!(out.len(), 3);
        assert_eq!(out[0].name, "set_flaps");
        assert_eq!(out[0].arguments.get("notch").unwrap(), "full");
        assert_eq!(out[1].arguments.get("light").unwrap(), "landing");
        assert_eq!(out[2].arguments.get("light").unwrap(), "taxi");
    }

    /// A minimal config pointing at the local Ollama. Model overridable via
    /// XCOPILOT_MODEL so we can compare models from the command line.
    fn test_config() -> Config {
        Config {
            ollama_base_url: "http://127.0.0.1:11434".into(),
            model: std::env::var("XCOPILOT_MODEL").unwrap_or_else(|_| "qwen3:8b".into()),
            xplane_base_url: "http://127.0.0.1:8086".into(),
            nav_data_path: String::new(),
            whisper_model_path: String::new(),
            listen_mode: "ptt".into(),
            wake_word: "copilot".into(),
            vad_threshold: 0.012,
            stt_min_confidence: 0.5,
            end_silence_ms: 900,
            ui: "plain".into(),
            tts_backend: "say".into(),
            piper_binary: String::new(),
            piper_voice: String::new(),
            aircraft: "cessna_172sp".into(),
            confirm_critical: true,
            monitor_enabled: false,
            monitor_hz: 3.0,
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
            ("flood lights on", "set_flood_lights", Some(("on", json!(true)))),
            ("flood lights off", "set_flood_lights", Some(("on", json!(false)))),
            ("flood lights 50 percent", "set_flood_lights", Some(("percent", json!(50)))),
            ("dim the floods to 30 percent", "set_flood_lights", Some(("percent", json!(30)))),
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
            ("autopilot off", "autopilot_off", None),
            ("disconnect the autopilot", "autopilot_off", None),
            ("pitot heat on", "set_pitot_heat", Some(("on", json!(true)))),
            ("fuel pump on", "set_fuel_pump", Some(("on", json!(true)))),
            ("carb heat off", "set_carb_heat", Some(("on", json!(false)))),
            ("magnetos both", "set_magnetos", Some(("position", json!("both")))),
            ("fuel selector left", "set_fuel_selector", Some(("position", json!("left")))),
            ("trim nose up", "set_pitch_trim", Some(("action", json!("nose_up")))),
            ("set takeoff trim", "set_pitch_trim", Some(("action", json!("takeoff")))),
            ("nav one 109.5", "set_nav1", None),
            ("set the OBS to 240", "set_obs", None),
            ("com two 121.9", "set_com2", None),
            ("swap com one", "swap_radio", Some(("radio", json!("com1")))),
            ("transponder standby", "set_transponder_mode", Some(("mode", json!("standby")))),
            ("squawk altitude", "set_transponder_mode", Some(("mode", json!("altitude")))),
            ("ident", "set_transponder_mode", Some(("mode", json!("ident")))),
        ];

        println!("\n  model: {}", cfg.model);
        let mut failures = 0;
        for (utterance, expected_tool, expected_arg) in cases {
            match llm.choose_tool(utterance, &specs, &[]).await.expect("ollama call failed").into_iter().next() {
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

    /// **Adversarial / real-world battery.** Unlike `llm_picks_correct_tool`
    /// (clean, idealized phrasings), every input here is raw whisper output —
    /// mostly *verbatim from `logs/`* — with the noise the cockpit actually
    /// produces: homophone mishearings ("Slaps up" for "flaps up", "probe
    /// lights" for "strobe lights"), trailing/garbled punctuation, missing
    /// articles, and questions. It also adds **negative cases**: utterances that
    /// clear the `looks_like_command` gate (they contain an aviation noun) but
    /// are *not* actionable commands — the model must decline (return no tool),
    /// which the temperature-retry path in `choose_tool` is most likely to get
    /// wrong by forcing a spurious pick.
    ///
    /// Inputs are pre-checked against the real `looks_like_command` gate: a case
    /// the gate would reject never reaches the LLM in production, so feeding it
    /// here would test the wrong layer. Such cases are reported and skipped, not
    /// scored — that mismatch is a *gate* gap (e.g. "Gaps down", "Ignition on"),
    /// tracked separately from LLM accuracy.
    ///
    /// Run with: `cargo test llm_picks_tool_noisy -- --ignored --nocapture`
    #[tokio::test]
    #[ignore = "requires a local Ollama server with a tool-calling model"]
    async fn llm_picks_tool_noisy() {
        let cfg = test_config();
        let llm = Llm::new(&cfg);
        let profile = aircraft::builtin(&cfg.aircraft).unwrap();
        let specs = Registry::for_aircraft(&profile).specs();

        // Positives: noisy real-world phrasings → expected tool (+ optional arg).
        // Strings marked "(log)" are verbatim whisper output from logs/.
        let positives: &[(&str, &str, Option<(&str, Value)>)] = &[
            ("Slaps up.", "set_flaps", Some(("notch", json!("up")))), // flaps→slaps
            ("Flaps down 10.", "set_flaps", Some(("notch", json!("10")))), // (log)
            ("Flaps 10 degrees.", "set_flaps", Some(("notch", json!("10")))), // (log)
            ("vertical speed 500.", "set_vertical_speed", None),      // (log)
            ("Set vertical speed 500.", "set_vertical_speed", None),  // (log)
            ("Heading 20.", "set_heading", None),                     // (log)
            ("heading 8-0.", "set_heading", None),                    // (log) garbled "80"
            ("Set heading 130.", "set_heading", None),                // (log)
            ("Power 50%.", "set_throttle", None),                     // (log) power→throttle
            ("throttle 50%!", "set_throttle", None),                  // (log) punctuation
            ("mixture 100%", "set_mixture", None),                    // (log)
            ("Lean mixture, 50%.", "set_mixture", None),              // (log)
            ("battery on.", "set_electrical", Some(("system", json!("battery")))), // (log)
            ("avionics on...", "set_electrical", Some(("system", json!("avionics")))), // (log)
            ("Landing light on.", "set_lights", Some(("light", json!("landing")))), // (log)
            ("Nav Light on.", "set_lights", Some(("light", json!("nav")))), // (log)
            ("Taxi light off.", "set_lights", Some(("on", json!(false)))), // (log)
            ("probe lights on.", "set_lights", Some(("light", json!("strobe")))), // (log) strobe→probe
            ("Floodlights on.", "set_flood_lights", Some(("on", json!(true)))), // (log) glued compound
            ("Bloodlights off.", "set_flood_lights", Some(("on", json!(false)))), // (log) flood→blood
            ("Start engines.", "start_engine", None),                 // (log) plural
            ("at Radio 122.", "set_com1", None),                      // (log) radio→com1
        ];

        // Negatives: clear the keyword gate but aren't commands → expect no tool.
        let negatives: &[&str] = &[
            "How much fuel do we have left?", // (log) a question, not a command
            "Is the landing light on?",       // question about a light
            "What heading are we on?",        // question about state
            "Let's talk about the landing later.", // chatter containing "landing"
        ];

        println!("\n  model: {}  (adversarial / real-world)\n", cfg.model);
        let mut failures = 0;
        let mut gate_skipped = 0;

        println!("  -- positives (noisy commands) --");
        for (utterance, expected_tool, expected_arg) in positives {
            if !crate::looks_like_command(utterance) {
                println!("  ⊘ {utterance:?} -> blocked by looks_like_command (gate gap, not scored)");
                gate_skipped += 1;
                continue;
            }
            match llm.choose_tool(utterance, &specs, &[]).await.expect("ollama call failed").into_iter().next() {
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

        println!("\n  -- negatives (should decline) --");
        for utterance in negatives {
            // Negatives must also clear the gate to be a meaningful LLM test;
            // if the gate already rejects them, production never asks the LLM.
            if !crate::looks_like_command(utterance) {
                println!("  ⊘ {utterance:?} -> gate rejects (handled before LLM, not scored)");
                gate_skipped += 1;
                continue;
            }
            match llm.choose_tool(utterance, &specs, &[]).await.expect("ollama call failed").into_iter().next() {
                None => println!("  ✓ {utterance:?} -> (declined, correct)"),
                Some(call) => {
                    println!("  ✗ {utterance:?} -> {} {} (should have declined)", call.name, call.arguments);
                    failures += 1;
                }
            }
        }

        println!("\n  {gate_skipped} case(s) skipped at the gate.");
        assert_eq!(failures, 0, "{failures} noisy case(s) handled incorrectly");
    }

    /// **Checklist-vocabulary battery.** The product model is "the pilot reads a
    /// paper checklist aloud and x-copilot executes each line", so every
    /// actionable item on a real Cessna 172 checklist must be a working command.
    /// The common verbs (mixture/flaps/lights) are covered by the clean battery;
    /// this targets the checklist-specific *vocabulary* that the tool
    /// descriptions may not anticipate — "ignition", "master switch", "fuel
    /// valve", "prime", "mixture idle cutoff", "wing flaps". A ✗ is a gap to fix
    /// (missing tool/alias) or a misroute. Each line is also checked against the
    /// domain gate so we catch words that never reach the LLM.
    ///
    /// Run: `cargo test llm_checklist_vocabulary -- --ignored --nocapture`
    #[tokio::test]
    #[ignore = "requires a local Ollama server with a tool-calling model"]
    async fn llm_checklist_vocabulary() {
        let cfg = test_config();
        let llm = Llm::new(&cfg);
        let profile = aircraft::builtin(&cfg.aircraft).unwrap();
        let specs = Registry::for_aircraft(&profile).specs();

        // Spoken exactly as a pilot reads the C172 checklist. expected_tool = ""
        // marks an item we suspect has NO tool yet (a known gap to confirm).
        let cases: &[(&str, &str, Option<(&str, Value)>)] = &[
            // electrical: "master switch" = battery, "avionics switch"
            ("master switch on", "set_electrical", Some(("system", json!("master")))),
            ("master switch off", "set_electrical", Some(("system", json!("master")))),
            ("avionics switch off", "set_electrical", Some(("system", json!("avionics")))),
            ("alternator off", "set_electrical", Some(("system", json!("alternator")))),
            // fuel: the checklist says "fuel valve" / "fuel shutoff valve", not "selector".
            // The 172 selector is LEFT/BOTH/RIGHT only — "off" is the separate red
            // fuel shutoff valve, not a selector position, so it has no tool (a gap).
            ("fuel valve both", "set_fuel_selector", Some(("position", json!("both")))),
            ("fuel valve off", "", None),
            ("fuel shutoff valve both", "set_fuel_selector", Some(("position", json!("both")))),
            // ignition switch positions
            ("ignition start", "start_engine", None),
            ("ignition off", "set_magnetos", Some(("position", json!("off")))),
            ("ignition both", "set_magnetos", Some(("position", json!("both")))),
            // engine config vocabulary
            ("mixture idle cutoff", "set_mixture", None),
            ("prime the engine", "", None),           // suspected gap: no primer tool
            ("prime as required", "", None),           // suspected gap
            ("throttle open one eighth", "set_throttle", None),
            ("throttle seventeen hundred RPM", "set_throttle", None),
            ("carb heat cold", "set_carb_heat", Some(("on", json!(false)))),
            // "wing flaps" (the checklist's wording) not just "flaps"
            ("wing flaps up", "set_flaps", Some(("notch", json!("up")))),
            ("wing flaps zero", "set_flaps", Some(("notch", json!("up")))),
            // other checklist phrasings
            ("beacon light on", "set_lights", Some(("light", json!("beacon")))),
            ("transponder on alt", "set_transponder_mode", Some(("mode", json!("altitude")))),
            ("trim set for takeoff", "set_pitch_trim", Some(("action", json!("takeoff")))),
            ("brakes test and set", "set_parking_brake", Some(("on", json!(true)))),
        ];

        println!("\n  model: {}  (checklist vocabulary)\n", cfg.model);
        let mut gaps = 0;
        for (utterance, expected_tool, expected_arg) in cases {
            let gated = crate::looks_like_command(utterance);
            let call = llm.choose_tool(utterance, &specs, &[]).await.expect("ollama").into_iter().next();
            let got = call.as_ref().map(|c| format!("{} {}", c.name, c.arguments)).unwrap_or_else(|| "(no tool)".into());
            let ok = match (expected_tool, &call) {
                (&"", None) => true, // confirmed gap
                (&"", Some(_)) => true, // bonus: something handled it
                (_, Some(c)) => {
                    c.name == *expected_tool
                        && expected_arg.as_ref().map(|(k, v)| c.arguments.get(*k) == Some(v)).unwrap_or(true)
                }
                (_, None) => false,
            };
            let mark = if !gated { "⊘" } else if ok { "✓" } else { "✗" };
            let note = if expected_tool.is_empty() { "  [suspected gap]" } else { "" };
            println!("  {mark} {utterance:?} -> {got}{note}");
            if !gated || !ok {
                gaps += 1;
            }
        }
        println!("\n  {gaps} item(s) need attention (gate-blocked, misrouted, or gaps).");
    }

    /// Probe: for the raw STT phrasings that miss at temperature 0, how reliably
    /// does a temperature bump recover a tool call? Drives the choice of
    /// `RETRY_TEMPS`. Runs each (phrase, temp) several times and prints the
    /// hit-rate, since temp > 0 is stochastic.
    /// Run: `cargo test retry_temp_sweep -- --ignored --nocapture`
    #[tokio::test]
    #[ignore = "requires a local Ollama server with a tool-calling model"]
    async fn retry_temp_sweep() {
        let cfg = test_config();
        let llm = Llm::new(&cfg);
        let profile = aircraft::builtin(&cfg.aircraft).unwrap();
        let specs = Registry::for_aircraft(&profile).specs();

        // Exact strings whisper produced live that miss at temp 0 (raw form).
        let hard = [
            ("Align the gyro.", "align_gyro"),
            ("Fix gyro.", "align_gyro"),
            ("Sync the heading indicator.", "align_gyro"),
            ("Release the parking brake.", "set_parking_brake"),
        ];
        let temps = [0.0, 0.3, 0.5, 0.7, 0.9];
        const TRIES: usize = 5;

        println!("\n  model: {}   ({TRIES} tries per cell; right-tool hits)\n", cfg.model);
        for (phrase, want) in hard {
            print!("  {phrase:<32}");
            for t in temps {
                let mut hits = 0;
                for _ in 0..TRIES {
                    if let Some(c) = llm.choose_tool_at(phrase, &specs, &[], t).await.expect("ollama").into_iter().next() {
                        if c.name == want {
                            hits += 1;
                        }
                    }
                }
                print!("  t={t:.1}:{hits}/{TRIES}");
            }
            println!();
        }
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
                .choose_tool(utterance, &specs, &[])
                .await
                .expect("ollama call failed")
                .into_iter()
                .next()
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
