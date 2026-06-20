//! Runtime configuration, loaded from the environment (and an optional `.env`).
//!
//! Nothing here is X-Plane-specific beyond the base URL — the tool layer owns
//! the dataref names. This keeps secrets (the Anthropic key) and machine-local
//! paths (whisper model, piper voice) out of the code.

use anyhow::Result;

/// All the knobs the loop needs to run end to end.
#[derive(Debug, Clone)]
pub struct Config {
    /// Base URL of the local Ollama server, e.g. `http://127.0.0.1:11434`.
    pub ollama_base_url: String,
    /// Local model id used for tool selection. Must support tool-calling
    /// (e.g. `llama3.1`, `qwen2.5`).
    pub model: String,
    /// Base URL of the X-Plane Web API, e.g. `http://127.0.0.1:8086`.
    pub xplane_base_url: String,
    /// Path to a whisper.cpp `.bin` model (e.g. `ggml-base.en.bin`).
    pub whisper_model_path: String,
    /// Capture mode: `continuous` (always listening, VAD-segmented) or `ptt`
    /// (push-to-talk, Enter to start/stop).
    pub listen_mode: String,
    /// Wake word required to act in `continuous` mode (e.g. `copilot`). Empty
    /// disables the gate (acts on every utterance — not recommended).
    pub wake_word: String,
    /// VAD energy threshold (RMS, 0.0..1.0). Above this counts as speech.
    /// Raise it in a noisy room, lower it if quiet speech is missed.
    pub vad_threshold: f32,
    /// Minimum STT confidence (mean token probability, 0.0..1.0) to act on an
    /// utterance. Below this, the copilot asks you to say it again instead of
    /// guessing. Raise to be stricter; lower if clear commands get rejected.
    pub stt_min_confidence: f32,
    /// UI mode: `plain` (stdout, default) or `tui` (terminal dashboard).
    pub ui: String,
    /// TTS backend: `say` (macOS built-in, default) or `piper`.
    pub tts_backend: String,
    /// The `piper` binary to shell out to (used when `tts_backend = "piper"`).
    pub piper_binary: String,
    /// Path to a piper voice model (`.onnx`); the matching `.onnx.json` must sit beside it.
    pub piper_voice: String,
    /// Path to X-Plane's `earth_nav.dat` (for ILS lookup).
    pub nav_data_path: String,
    /// Active aircraft profile id (see [`crate::aircraft`]), e.g. `cessna_172sp`.
    /// Determines the available tools and their limits.
    pub aircraft: String,
    /// Require spoken "affirm" before executing safety-critical commands
    /// (autopilot flight-path changes). On by default — a misheard command
    /// should never move the aircraft unconfirmed.
    pub confirm_critical: bool,
    /// Run the background monitor that calls out problems (e.g. alternator
    /// offline). On by default; set `XCOPILOT_MONITOR=false` to silence it.
    pub monitor_enabled: bool,
    /// Monitor sample rate in Hz — how often it snapshots datarefs and re-checks.
    pub monitor_hz: f32,
}

impl Config {
    /// Build a [`Config`] from environment variables, applying sensible defaults.
    /// Everything is local, so there are no required secrets.
    pub fn from_env() -> Result<Self> {
        // Best-effort: load a local .env if present. Missing file is fine.
        let _ = dotenvy::dotenv();

        Ok(Self {
            ollama_base_url: env_or("OLLAMA_BASE_URL", "http://127.0.0.1:11434"),
            model: env_or("XCOPILOT_MODEL", "llama3.1"),
            xplane_base_url: env_or("XPLANE_BASE_URL", "http://127.0.0.1:8086"),
            whisper_model_path: env_or("WHISPER_MODEL", "models/ggml-base.en.bin"),
            listen_mode: env_or("XCOPILOT_LISTEN", "continuous"),
            // Empty by default: act on every clear utterance, let the LLM ignore
            // non-commands. Set a word here to require it as a prefix.
            wake_word: env_or("XCOPILOT_WAKE_WORD", ""),
            vad_threshold: env_or("XCOPILOT_VAD_RMS", "0.012").parse().unwrap_or(0.012),
            stt_min_confidence: env_or("XCOPILOT_STT_MIN_CONFIDENCE", "0.35").parse().unwrap_or(0.35),
            ui: env_or("XCOPILOT_UI", "plain"),
            tts_backend: env_or("XCOPILOT_TTS", "say"),
            piper_binary: env_or("PIPER_BIN", "piper"),
            piper_voice: env_or("PIPER_VOICE", "models/en_US-amy-medium.onnx"),
            nav_data_path: env_or(
                "XPLANE_NAV_DATA",
                "/Users/duanelicudi/X-Plane 12/Resources/default data/earth_nav.dat",
            ),
            aircraft: env_or("XCOPILOT_AIRCRAFT", "auto"),
            confirm_critical: env_or("XCOPILOT_CONFIRM", "true") != "false",
            monitor_enabled: env_or("XCOPILOT_MONITOR", "true") != "false",
            monitor_hz: env_or("XCOPILOT_MONITOR_HZ", "3").parse().unwrap_or(3.0),
        })
    }
}

fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}
