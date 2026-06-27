//! Runtime configuration, loaded from the environment (and an optional `.env`).
//!
//! Nothing here is X-Plane-specific beyond the base URL — the tool layer owns
//! the dataref names. This keeps secrets (the Anthropic key) and machine-local
//! paths (whisper model, piper voice) out of the code.

use anyhow::Result;

/// All the knobs the loop needs to run end to end.
#[derive(Debug, Clone, Default)]
pub struct Config {
    /// Base URL of the local Ollama server, e.g. `http://127.0.0.1:11434`.
    pub ollama_base_url: String,
    /// Ollama `keep_alive` for the model (e.g. `60m`, `5m`, `0`). How long it
    /// stays resident on the GPU after a call; `0` frees GPU memory between
    /// commands at the cost of a cold-load each time.
    pub llm_keepalive: String,
    /// Ollama `num_gpu`: how many model layers to run on the GPU. Empty = Ollama's
    /// default (all). `0` runs the LLM entirely on the CPU so its inference stops
    /// contending with a GPU-bound X-Plane (slower thinking, but no FPS dip).
    pub llm_gpu_layers: String,
    /// Local model id used for tool selection. Must support tool-calling
    /// (e.g. `qwen3:8b`, `qwen2.5`, `llama3.1`). Default `qwen3:8b` won the
    /// tool-pick battery (100% on the clean set, fastest of the perfect
    /// scorers); `llm.rs` auto-disables its thinking mode for low latency.
    pub model: String,
    /// Base URL of the X-Plane Web API, e.g. `http://127.0.0.1:8086`.
    pub xplane_base_url: String,
    /// Path to a whisper.cpp `.bin` model (e.g. `ggml-base.en.bin`).
    pub whisper_model_path: String,
    /// Run whisper on the Metal GPU (default) or the CPU. Set false to keep STT
    /// off the GPU so it doesn't steal frames from a GPU-bound X-Plane.
    pub whisper_gpu: bool,
    /// Capture mode: `continuous` (always listening, VAD-segmented) or `ptt`
    /// (push-to-talk, Enter to start/stop).
    pub listen_mode: String,
    /// Global push-to-talk key (e.g. `rctrl`, `f8`, `space`). When set, the copilot
    /// captures only while this key is held — works while X-Plane has focus, so no
    /// wake word and ambient/ATC can't trigger it. Empty = use `listen_mode`.
    pub ptt_key: String,
    /// Push-to-talk dataref (e.g. `xcopilot/ptt_active`, set by the bundled X-Plane
    /// plugin). When set, the copilot captures only while this dataref reads nonzero
    /// — a button bound inside X-Plane drives it, so no macOS permission and no radio
    /// keying. Empty = use `ptt_key`/`listen_mode`. Takes precedence over `ptt_key`.
    pub ptt_dataref: String,
    /// VAD energy threshold (RMS, 0.0..1.0). Above this counts as speech.
    /// Raise it in a noisy room, lower it if quiet speech is missed.
    pub vad_threshold: f32,
    /// Minimum STT confidence (mean token probability, 0.0..1.0) to act on an
    /// utterance. Below this, the copilot asks you to say it again instead of
    /// guessing. Raise to be stricter; lower if clear commands get rejected.
    pub stt_min_confidence: f32,
    /// Trailing silence (ms) that ends an utterance in continuous mode. Lower
    /// feels snappier but clips mid-command hesitations; raise it if a slight
    /// pause cuts you off. Default 900.
    pub end_silence_ms: u32,
    /// UI mode: `tui` (terminal dashboard) or `plain` (stdout). Defaults to `tui`
    /// when attached to an interactive terminal, `plain` otherwise (piped output,
    /// CI) — the dashboard needs a real TTY to take over the screen and read keys.
    pub ui: String,
    /// TTS backend: `sidecar` (warm Spark voice-clone server — the default, see
    /// `voice/spark_sidecar.py`; auto-spawned and falls back to `say` if it can't
    /// start), `say` (macOS built-in), `piper`, or `onnx` (in-process ONNX,
    /// feature `onnx-tts`).
    pub tts_backend: String,
    /// macOS `say` voice name (used when `tts_backend = "say"`). Empty = system
    /// default. List options with `say -v '?'` (e.g. `Daniel`, `Samantha`, `Karen`).
    pub say_voice: String,
    /// macOS `say` rate in words per minute (used when `tts_backend = "say"`).
    /// Empty = default (~175). Lower for a calmer cadence.
    pub say_rate: String,
    /// The `piper` binary to shell out to (used when `tts_backend = "piper"`).
    pub piper_binary: String,
    /// Path to a piper voice model (`.onnx`); the matching `.onnx.json` must sit beside it.
    pub piper_voice: String,
    /// URL of the Spark TTS sidecar (`tts_backend = "sidecar"`): POST readback
    /// text, get back `audio/wav` in the cloned voice.
    pub tts_sidecar_url: String,
    /// Launch and supervise the Spark sidecar ourselves (default). Set false to
    /// talk to one you started by hand at `tts_sidecar_url`.
    pub tts_sidecar_autospawn: bool,
    /// Python interpreter for the auto-spawned sidecar. Empty = auto-discover an
    /// mlx-audio venv (e.g. `/tmp/mlxtts/bin/python`).
    pub tts_sidecar_python: String,
    /// How long to wait for the auto-spawned sidecar to load its model before
    /// giving up and falling back to `say`. Generous: a first run downloads it.
    pub tts_sidecar_ready_secs: u64,
    /// Spark sampling temperature (0.0..1.0). Higher = more prosodic variation
    /// (livelier, less droning); lower = flatter, more monotone. Forwarded to the
    /// sidecar. Default 0.7 — 0.6 sounded depressed, 0.8 drifted (reference leaked
    /// into synthesis); 0.7 balances liveliness against drift.
    pub tts_sidecar_temp: String,
    /// Default cloned voice for the sidecar (a name under `voice/voices/`). Empty
    /// = let the sidecar default (its `.active` file, set by `voice/voices.py use`,
    /// else `roger`). Forwarded to the sidecar as its env default.
    pub voice: String,
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
    /// Mirror copilot status (phase / last heard / last action) into the bundled
    /// X-Plane plugin's in-sim panel. On by default; best-effort — a no-op if the
    /// plugin isn't installed. Set `XCOPILOT_PLUGIN_STATUS=false` to disable.
    pub plugin_status: bool,
}

impl Config {
    /// Build a [`Config`] from environment variables, applying sensible defaults.
    /// Everything is local, so there are no required secrets.
    pub fn from_env() -> Result<Self> {
        // Best-effort: load a local .env if present. Missing file is fine.
        let _ = dotenvy::dotenv();

        Ok(Self {
            ollama_base_url: env_or("OLLAMA_BASE_URL", "http://127.0.0.1:11434"),
            model: env_or("XCOPILOT_MODEL", "qwen3:8b"),
            llm_keepalive: env_or("XCOPILOT_LLM_KEEPALIVE", "60m"),
            llm_gpu_layers: env_or("XCOPILOT_LLM_GPU_LAYERS", ""),
            xplane_base_url: env_or("XPLANE_BASE_URL", "http://127.0.0.1:8086"),
            whisper_model_path: env_or("WHISPER_MODEL", "models/ggml-base.en.bin"),
            whisper_gpu: env_or("XCOPILOT_WHISPER_GPU", "true") != "false",
            listen_mode: env_or("XCOPILOT_LISTEN", "continuous"),
            ptt_key: env_or("XCOPILOT_PTT_KEY", ""),
            ptt_dataref: env_or("XCOPILOT_PTT_DATAREF", ""),
            vad_threshold: env_or("XCOPILOT_VAD_RMS", "0.012").parse().unwrap_or(0.012),
            stt_min_confidence: env_or("XCOPILOT_STT_MIN_CONFIDENCE", "0.35").parse().unwrap_or(0.35),
            end_silence_ms: env_or("XCOPILOT_END_SILENCE_MS", "900").parse().unwrap_or(900),
            ui: env_or("XCOPILOT_UI", default_ui()),
            tts_backend: env_or("XCOPILOT_TTS", "sidecar"),
            say_voice: env_or("XCOPILOT_SAY_VOICE", ""),
            say_rate: env_or("XCOPILOT_SAY_RATE", ""),
            piper_binary: env_or("PIPER_BIN", "piper"),
            piper_voice: env_or("PIPER_VOICE", "models/en_US-amy-medium.onnx"),
            tts_sidecar_url: env_or("XCOPILOT_TTS_SIDECAR_URL", "http://127.0.0.1:8765/synth"),
            tts_sidecar_autospawn: env_or("XCOPILOT_TTS_SIDECAR_AUTOSPAWN", "true") != "false",
            tts_sidecar_python: env_or("XCOPILOT_TTS_SIDECAR_PYTHON", ""),
            tts_sidecar_ready_secs: env_or("XCOPILOT_TTS_SIDECAR_READY_SECS", "90")
                .parse()
                .unwrap_or(90),
            tts_sidecar_temp: env_or("XCOPILOT_TTS_SIDECAR_TEMP", "0.6"),
            voice: env_or("XCOPILOT_VOICE", ""),
            nav_data_path: env_or(
                "XPLANE_NAV_DATA",
                "/Users/duanelicudi/X-Plane 12/Resources/default data/earth_nav.dat",
            ),
            aircraft: env_or("XCOPILOT_AIRCRAFT", "auto"),
            confirm_critical: env_or("XCOPILOT_CONFIRM", "true") != "false",
            monitor_enabled: env_or("XCOPILOT_MONITOR", "true") != "false",
            monitor_hz: env_or("XCOPILOT_MONITOR_HZ", "3").parse().unwrap_or(3.0),
            plugin_status: env_or("XCOPILOT_PLUGIN_STATUS", "true") != "false",
        })
    }
}

fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

/// Default UI when `XCOPILOT_UI` is unset: the dashboard on an interactive
/// terminal, plain stdout when output is piped or there's no TTY (the dashboard
/// would corrupt redirected output and has no keys to read).
fn default_ui() -> &'static str {
    use std::io::IsTerminal;
    if std::io::stdin().is_terminal() && std::io::stdout().is_terminal() {
        "tui"
    } else {
        "plain"
    }
}
