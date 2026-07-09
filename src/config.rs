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
    /// Whether the local LLM is used at all. When `false` (the default) the copilot
    /// is fully deterministic: the grammar fast-path handles commands, the
    /// state-query handler answers questions, and anything unrecognized gets "Say
    /// again?" — Ollama is never contacted. The LLM only ever *answered* questions
    /// the deterministic handler missed (it never actuates), so turning it off
    /// loses only some free-form Q&A phrasings, never an action. `XCOPILOT_LLM=on`
    /// re-enables that answer-only fallback.
    pub llm_enabled: bool,
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
    /// Push-to-talk dataref (`xcopilot/ptt_active`, set by the bundled X-Plane
    /// plugin) — **the default**. The copilot captures only while this dataref reads
    /// nonzero, driven by a button bound inside X-Plane, so no macOS mic permission
    /// and no radio keying. Set to empty (`XCOPILOT_PTT_DATAREF=`) to fall back to
    /// `ptt_key`/`listen_mode`. Takes precedence over `ptt_key`.
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
    /// How long after the copilot stops speaking to keep the mic muted (ms),
    /// covering output latency and room echo so a readback over speakers can't
    /// bleed back in as a phantom command. Raise it on loud/echoey speaker setups
    /// (headphones can go lower). Default 900.
    pub mic_gate_tail_ms: u64,
    /// UI mode: `tui` (terminal dashboard) or `plain` (stdout). Defaults to `tui`
    /// when attached to an interactive terminal, `plain` otherwise (piped output,
    /// CI) — the dashboard needs a real TTY to take over the screen and read keys.
    pub ui: String,
    /// TTS backend: `onnx` (the default — warm in-process ONNX voice via `ort`,
    /// native arm64, see [`crate::tts_onnx`]; falls back to `say` if the model
    /// can't load), `say` (macOS built-in), or `piper` (subprocess).
    pub tts_backend: String,
    /// macOS `say` voice name (used when `tts_backend = "say"`). Empty = system
    /// default. List options with `say -v '?'` (e.g. `Daniel`, `Samantha`, `Karen`).
    pub say_voice: String,
    /// macOS `say` rate in words per minute (used when `tts_backend = "say"`).
    /// Empty = default (~175). Lower for a calmer cadence.
    pub say_rate: String,
    /// Sound played just before a reminder calls out (a short chime, so a heads-up
    /// gets your attention before the words). A WAV/AIFF path; empty = a platform
    /// default (macOS `Glass.aiff`). Point it at any sound you like, including one
    /// pulled from X-Plane's `Resources/sounds`. `XCOPILOT_CHIME`.
    pub chime_path: String,
    /// Sound played when the copilot heard you clearly but the command wasn't
    /// recognized (a short negative tone, so a miss is acknowledged without the
    /// misleading spoken "Say again?"). A WAV/AIFF path; empty = a platform default
    /// (macOS `Basso.aiff`). `XCOPILOT_NEGATIVE_TONE`.
    pub negative_tone_path: String,
    /// The `piper` binary to shell out to (used when `tts_backend = "piper"`).
    pub piper_binary: String,
    /// Path to a piper voice model (`.onnx`); the matching `.onnx.json` must sit beside it.
    pub piper_voice: String,
    /// Path to X-Plane's `earth_nav.dat` (for ILS lookup).
    pub nav_data_path: String,
    /// Path to X-Plane's Global Airports `apt.dat` (for runway lookup — which
    /// runways a field actually has, so runway advice is never invented).
    pub apt_data_path: String,
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
    /// SimBrief numeric user ID, for fetching the latest flight plan (OFP). Empty
    /// = no plan integration. The only outbound network call besides X-Plane: it
    /// hits simbrief.com on demand, not in the realtime loop.
    pub simbrief_id: String,
    /// Mirror copilot status (phase / last heard / last action) into the bundled
    /// X-Plane plugin's in-sim panel. On by default; best-effort — a no-op if the
    /// plugin isn't installed. Set `XCOPILOT_PLUGIN_STATUS=false` to disable.
    pub plugin_status: bool,
    /// ICAO of the airport whose ATC station the copilot process hosts (tower
    /// dialogue answered when COM1 is tuned to its frequency). Empty disables
    /// ATC entirely. `XCOPILOT_ATC_AIRPORT`.
    pub atc_airport: String,
    /// Radio (transmit) push-to-talk dataref, set by the bundled plugin's
    /// `xcopilot/radio_ptt` command — the transmit switch, as distinct from the
    /// intercom PTT above. An utterance captured while THIS is held goes to
    /// the tuned ATC station unconditionally, never to the copilot. Empty
    /// disables physical routing (content routing still applies).
    /// `XCOPILOT_RADIO_PTT_DATAREF`.
    pub radio_ptt_dataref: String,
    /// Piper `.onnx` voice for the ATC TOWER — a second warm in-process ONNX
    /// session, distinct from the copilot's voice. Empty falls back to the
    /// `say` voice below. `XCOPILOT_ATC_PIPER_VOICE`.
    pub atc_piper_voice: String,
    /// Piper voice for APPROACH — a different controller, a different person.
    /// Empty falls back to the tower's voice. `XCOPILOT_ATC_APPROACH_PIPER_VOICE`.
    pub atc_approach_piper_voice: String,
    /// Piper voice for DEPARTURE. Empty falls back to the tower's voice.
    /// `XCOPILOT_ATC_DEPARTURE_PIPER_VOICE`.
    pub atc_departure_piper_voice: String,
    /// Piper voice for the ATIS broadcast. Empty falls back to the tower's
    /// voice. `XCOPILOT_ATC_ATIS_PIPER_VOICE`.
    pub atc_atis_piper_voice: String,
    /// Recorded squelch transient spliced onto the end of every station
    /// transmission (VATSIM-style; the default is AFV-Native's click, BSD-3,
    /// see assets/afv/). Empty falls back to the synthesized burst.
    /// `XCOPILOT_SQUELCH_WAV`.
    pub atc_squelch_wav: String,
    /// macOS `say` voice for the ATC station, used when the ONNX station voice
    /// is empty or fails to load. `XCOPILOT_ATC_VOICE`.
    pub atc_voice: String,
    /// `say` rate (words per minute) for the ATC station. `XCOPILOT_ATC_RATE`.
    pub atc_rate: String,
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
            // Off by default — fully deterministic. `on`/`true`/`1`/`yes` enables
            // the answer-only LLM fallback.
            llm_enabled: matches!(
                env_or("XCOPILOT_LLM", "off").to_lowercase().as_str(),
                "on" | "true" | "1" | "yes"
            ),
            xplane_base_url: env_or("XPLANE_BASE_URL", "http://127.0.0.1:8086"),
            whisper_model_path: env_or("WHISPER_MODEL", "models/ggml-base.en.bin"),
            whisper_gpu: env_or("XCOPILOT_WHISPER_GPU", "true") != "false",
            listen_mode: env_or("XCOPILOT_LISTEN", "continuous"),
            ptt_key: env_or("XCOPILOT_PTT_KEY", ""),
            ptt_dataref: env_or("XCOPILOT_PTT_DATAREF", "xcopilot/ptt_active"),
            vad_threshold: env_or("XCOPILOT_VAD_RMS", "0.012").parse().unwrap_or(0.012),
            stt_min_confidence: env_or("XCOPILOT_STT_MIN_CONFIDENCE", "0.35").parse().unwrap_or(0.35),
            end_silence_ms: env_or("XCOPILOT_END_SILENCE_MS", "900").parse().unwrap_or(900),
            mic_gate_tail_ms: env_or("XCOPILOT_MIC_GATE_TAIL_MS", "900").parse().unwrap_or(900),
            ui: env_or("XCOPILOT_UI", default_ui()),
            tts_backend: env_or("XCOPILOT_TTS", "onnx"),
            say_voice: env_or("XCOPILOT_SAY_VOICE", ""),
            say_rate: env_or("XCOPILOT_SAY_RATE", ""),
            chime_path: env_or("XCOPILOT_CHIME", ""),
            negative_tone_path: env_or("XCOPILOT_NEGATIVE_TONE", ""),
            piper_binary: env_or("PIPER_BIN", "piper"),
            piper_voice: env_or("PIPER_VOICE", "models/en_US-amy-medium.onnx"),
            // Prefer Custom Data (Navigraph/updated AIRAC) when it exists — it's
            // what X-Plane itself flies on; the stock file goes stale the moment
            // a cycle is installed. Env override wins over both.
            nav_data_path: {
                let custom = "/Users/duanelicudi/X-Plane 12/Custom Data/earth_nav.dat";
                let fallback = if std::path::Path::new(custom).exists() {
                    custom
                } else {
                    "/Users/duanelicudi/X-Plane 12/Resources/default data/earth_nav.dat"
                };
                env_or("XPLANE_NAV_DATA", fallback)
            },
            apt_data_path: env_or(
                "XPLANE_APT_DATA",
                "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat",
            ),
            aircraft: env_or("XCOPILOT_AIRCRAFT", "auto"),
            confirm_critical: env_or("XCOPILOT_CONFIRM", "true") != "false",
            monitor_enabled: env_or("XCOPILOT_MONITOR", "true") != "false",
            monitor_hz: env_or("XCOPILOT_MONITOR_HZ", "3").parse().unwrap_or(3.0),
            plugin_status: env_or("XCOPILOT_PLUGIN_STATUS", "true") != "false",
            simbrief_id: env_or("XCOPILOT_SIMBRIEF_ID", ""),
            atc_airport: env_or("XCOPILOT_ATC_AIRPORT", "LXGB"),
            radio_ptt_dataref: env_or("XCOPILOT_RADIO_PTT_DATAREF", "xcopilot/radio_ptt_active"),
            atc_piper_voice: env_or("XCOPILOT_ATC_PIPER_VOICE", "models/en_GB-alba-medium.onnx"),
            // ryan-high: the only high-quality model in the folder, and clearly
            // a different person from the tower (alba) and departure (northern
            // male). alan-medium read flat and got voted off (2026-07-09).
            atc_approach_piper_voice: env_or(
                "XCOPILOT_ATC_APPROACH_PIPER_VOICE",
                "models/en_US-ryan-high.onnx",
            ),
            atc_departure_piper_voice: env_or(
                "XCOPILOT_ATC_DEPARTURE_PIPER_VOICE",
                "models/en_GB-northern_english_male-medium.onnx",
            ),
            // Empty = the tower's voice; ATIS on a real field is often the
            // same controller reading onto the loop anyway.
            atc_atis_piper_voice: env_or("XCOPILOT_ATC_ATIS_PIPER_VOICE", ""),
            atc_squelch_wav: env_or("XCOPILOT_SQUELCH_WAV", "assets/afv/Click_f32.wav"),
            atc_voice: env_or("XCOPILOT_ATC_VOICE", "Daniel"),
            atc_rate: env_or("XCOPILOT_ATC_RATE", "165"),
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
