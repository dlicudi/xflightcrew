//! Runtime configuration, loaded from the environment (and an optional `.env`).
//!
//! Nothing here is X-Plane-specific beyond the base URL — the tool layer owns
//! the dataref names. This keeps machine-local paths (whisper model, piper
//! voice) out of the code. Everything is local; there are no secrets.

use anyhow::Result;

/// All the knobs the loop needs to run end to end.
#[derive(Debug, Clone, Default)]
pub struct Config {
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
    /// utterance outright. Below this, the copilot asks you to say it again or
    /// (in the confirm band below) reads back what it heard for a yes/no instead
    /// of guessing. Raise to be stricter; lower if clear commands get rejected.
    pub stt_min_confidence: f32,
    /// Floor of the "confirm, don't drop" band. A PTT/wake command heard between
    /// this and `stt_min_confidence` isn't discarded — the copilot reads back
    /// what it heard ("Confirm, I heard gear up?") and acts on a yes, so a
    /// marginal capture costs one word, not the whole command again. Below this
    /// floor the audio is too garbled to guess at and gets the "say again". Set
    /// equal to `stt_min_confidence` to disable the band entirely.
    pub stt_confirm_floor: f32,
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
    /// Playback volume for the copilot's intercom voice (0.0-2.0, 1.0 = unity).
    /// Applied on the WAV-playing paths (`afplay -v`); the `say` fallback has no
    /// volume knob. `XCOPILOT_VOICE_VOL`.
    pub voice_volume: f32,
    /// Playback volume for every station voice — controllers, ATIS, party-line
    /// traffic (0.0-2.0, 1.0 = unity). Lets the radio sit louder or quieter than
    /// the copilot, like the COM knob on a real audio panel. `XCOPILOT_ATC_VOL`.
    pub atc_volume: f32,
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
    /// The cabin crew's voice model — the second intercom persona, so it must
    /// not be the pilot voice. `XCOPILOT_CABIN_VOICE`.
    pub cabin_piper_voice: String,
    /// Whether the cabin crew persona is aboard: `auto` (the aircraft
    /// profile's `cabin_crew` flag decides — a 172 has none, an MD-82 does),
    /// `aboard`/`off` (or `true`/`false`) to override. `XCOPILOT_CABIN_CREW`
    /// sets only the BOOT value; empty = fall back to the panel's saved pick
    /// (the Settings tab toggles it live from there — see `cabin::Staffing`).
    pub cabin_crew: String,
    /// Path to X-Plane's `earth_nav.dat` (for ILS lookup).
    pub nav_data_path: String,
    /// Path to X-Plane's `earth_fix.dat` (enroute/terminal waypoints) — resolves
    /// the named fixes in a SID/STAR/approach to positions. `XPLANE_FIX_DATA`.
    pub fix_data_path: String,
    /// Path to X-Plane's OpenAir airspace file (the polygons the G1000 draws) —
    /// the circuit's restricted/prohibited/danger avoidance. `XPLANE_AIRSPACE`.
    pub airspace_path: String,
    /// Directory of per-airport CIFP procedure files (`<ICAO>.dat`, ARINC
    /// 424) — real approach platform altitudes. `XPLANE_CIFP_DIR`.
    pub cifp_dir: String,
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
    /// Play the voices through the plugin onto X-Plane's audio buses
    /// (`XCOPILOT_AUDIO_BUS=true`); the panel's Settings switch also turns it
    /// on live. Off by default until it has flown.
    pub audio_bus: bool,
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
    /// Initial app mode: "both" (default), "copilot" (no simulated ATC — the
    /// radio belongs to external ATC), or "atc" (world only, no LLM/tools/
    /// intercom persona). `XCOPILOT_MODE`; empty = fall back to the panel's
    /// saved pick in prefs.json, then "both". Runtime-toggleable from the
    /// plugin's Settings tab afterwards — this is only the boot value.
    pub mode: String,
    /// ICAO of the airport whose ATC station the copilot process hosts (tower
    /// dialogue answered when COM1 is tuned to its frequency). Empty disables
    /// ATC entirely. `XCOPILOT_ATC_AIRPORT`.
    pub atc_airport: String,
    /// Enroute region behavior: "auto" (default) indexes every towered field
    /// from apt.dat and hands the flight off to whichever tower becomes
    /// nearest; "off" keeps the single configured airport forever.
    /// `XCOPILOT_ATC_REGION`.
    pub atc_region: String,
    /// Party-line chatter: the tower narrates clearances for live traffic
    /// (LiveTraffic/AI ghosts) while COM1 is on its frequency. On by
    /// default; `XCOPILOT_ATC_CHATTER=false` silences the flavor calls
    /// (the go-around/held-clearance safety watches stay on regardless).
    pub atc_chatter: bool,
    /// Who flies the traffic round us (stage D, `traffic_link::TrafficMode`):
    /// `off` (the default until the plugin draws our own) and `real` keep
    /// today's read-only narration of the sim's TCAS traffic; `ours` flies
    /// our own fleet at the installed field, controlled by its tower.
    /// `XCOPILOT_TRAFFIC`.
    pub traffic: String,
    /// How busy our generated traffic is: `off`, `light` or `normal`.
    /// `XCOPILOT_TRAFFIC_DENSITY`.
    pub traffic_density: String,
    /// The pilot's own traffic schedule (YAML: `t_s, callsign, type, kind`,
    /// `runway?, stand?, laps?, spoken?`), flown instead of a generated one;
    /// empty = generated. `XCOPILOT_TRAFFIC_FILE`.
    pub traffic_file: String,
    /// How much the copilot volunteers (`persona::Chatter`): `quiet` answers
    /// and calls what matters, `normal` adds the landing word, the offers
    /// and the flight's memory, `chatty` a little more. Answers and safety
    /// calls are never gated by it. `XCOPILOT_CREW_CHATTER`.
    pub crew_chatter: String,
    /// How much of a start-up the copilot calls (`crew_pace::StartCalls`):
    /// `milestones` the callouts a crew makes, `every item` each switch as it
    /// is confirmed. Separate from `crew_chatter`: quiet chatter never
    /// silences a procedure call. `XCOPILOT_START_CALLS`.
    pub start_calls: String,
    /// The copilot's idiom (`persona::Dialect`): `auto` follows the voice's
    /// own accent (an en_GB model says "cheers", an en_US one "you bet"),
    /// or `british` / `american`. `XCOPILOT_CREW_DIALECT`.
    pub crew_dialect: String,
    /// What the copilot calls the pilot, now and then ("Nice one, Sam").
    /// Empty = never by name. `XCOPILOT_PILOT_NAME`.
    pub pilot_name: String,
    /// The copilot's own name, for "what's your name". Empty = none on the
    /// roster. `XCOPILOT_COPILOT_NAME`.
    pub copilot_name: String,
    /// The callsign the tower and the crew use, over the SimBrief plan's and
    /// the tail number: a raw id ("BAW306", "N172SP", "G-ABCD") is spoken
    /// for you, or give the words ("Speedbird 306"). Empty = the plan's, else
    /// the tail's. `XCOPILOT_CALLSIGN`, else the Settings registry's
    /// `callsign` (the panel's Settings tab, the terminal, prefs.json).
    pub callsign: String,
    /// The pilot's own airline telephony, asked before the built-in tables:
    /// `WAT=Walker; XYZ=Zulu Air` (`atc::callsign::parse_telephony`). Empty =
    /// the tables only. `XCOPILOT_AIRLINE_TELEPHONY`, else the registry's
    /// `airline_telephony`.
    pub airline_telephony: String,
    /// Mute X-Plane's own radio voice (built-in ATC + its ATIS) at startup
    /// while our ATC is on — both spoke on the same frequencies at once
    /// (two ATIS voices, 2026-07-10). Also silences nav-aid morse idents.
    /// `XCOPILOT_MUTE_XP_RADIO=false` keeps the sim's radio audio.
    pub mute_xp_radio: bool,
    /// Readback enforcement: the tower challenges a missing or wrong-runway
    /// readback of runway-critical clearances (takeoff, landing, line-up,
    /// hold). On by default — it's the learning tool working as intended;
    /// `XCOPILOT_ATC_READBACK=false` keeps the coach suggestions but drops
    /// the challenges.
    pub atc_readback: bool,
    /// The copilot's progressive taxi guide: just-in-time turn/crossing/
    /// holding-point callouts along the cleared taxi route (the crew half of
    /// the clearance the tower spoke). On by default;
    /// `XCOPILOT_TAXI_GUIDE=false` for pilots who want the diagram to
    /// themselves.
    pub taxi_guide: bool,
    /// The EFB: a page served on the LAN (`src/efb/`) for an iPad, iPhone
    /// or a second screen. Off by default. `XCOPILOT_EFB=true`, else the
    /// registry's `efb`.
    pub efb: bool,
    /// The EFB's TCP port. `XCOPILOT_EFB_PORT`, default 8787.
    pub efb_port: u16,
    /// The EFB's access token, asked of every request. Empty = generated at
    /// the first enable and saved as a pref. `XCOPILOT_EFB_TOKEN`.
    pub efb_token: String,
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
    /// Comma-separated Piper models the OTHER airports' controllers draw from
    /// (the configured home airport keeps the curated per-position voices
    /// above). Seeded by airport name, so Ciudad Real's tower is a different,
    /// stable person from Gibraltar's. Empty = every airport uses the curated
    /// voices. `XCOPILOT_ATC_VOICE_POOL`.
    pub atc_voice_pool: String,
    /// Deterministic per-controller pitch/pace offset (a few percent, keyed to
    /// the cast identity) applied to station voices before the radio effect —
    /// a station never plays a TTS model at its native pitch, so the copilot
    /// can share a model with any controller without sounding like the same
    /// person. `XCOPILOT_ATC_VOICE_SHIFT=false` disables.
    #[cfg_attr(not(feature = "onnx-tts"), allow(dead_code))] // the onnx voice's only
    pub atc_voice_shift: bool,
    /// Recorded squelch transient spliced onto the end of every station
    /// transmission (VATSIM-style; the default is AFV-Native's click, BSD-3,
    /// see assets/afv/). Empty falls back to the synthesized burst.
    /// `XCOPILOT_SQUELCH_WAV`.
    #[cfg_attr(not(feature = "onnx-tts"), allow(dead_code))] // the onnx voice's only
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
            xplane_base_url: env_or("XPLANE_BASE_URL", "http://127.0.0.1:8086"),
            whisper_model_path: env_or("WHISPER_MODEL", &model_path("ggml-base.en.bin")),
            whisper_gpu: env_or("XCOPILOT_WHISPER_GPU", "true") != "false",
            listen_mode: env_or("XCOPILOT_LISTEN", "continuous"),
            ptt_key: env_or("XCOPILOT_PTT_KEY", ""),
            ptt_dataref: env_or("XCOPILOT_PTT_DATAREF", "xcopilot/ptt_active"),
            vad_threshold: env_or("XCOPILOT_VAD_RMS", "0.012").parse().unwrap_or(0.012),
            stt_min_confidence: env_or("XCOPILOT_STT_MIN_CONFIDENCE", "0.35").parse().unwrap_or(0.35),
            stt_confirm_floor: env_or("XCOPILOT_STT_CONFIRM_FLOOR", "0.22").parse().unwrap_or(0.22),
            end_silence_ms: env_or("XCOPILOT_END_SILENCE_MS", "900").parse().unwrap_or(900),
            mic_gate_tail_ms: env_or("XCOPILOT_MIC_GATE_TAIL_MS", "900").parse().unwrap_or(900),
            ui: env_or("XCOPILOT_UI", default_ui()),
            tts_backend: env_or("XCOPILOT_TTS", "onnx"),
            voice_volume: env_or("XCOPILOT_VOICE_VOL", "1.0").parse().unwrap_or(1.0),
            atc_volume: env_or("XCOPILOT_ATC_VOL", "1.0").parse().unwrap_or(1.0),
            say_voice: env_or("XCOPILOT_SAY_VOICE", ""),
            say_rate: env_or("XCOPILOT_SAY_RATE", ""),
            chime_path: env_or("XCOPILOT_CHIME", ""),
            negative_tone_path: env_or("XCOPILOT_NEGATIVE_TONE", ""),
            piper_binary: env_or("PIPER_BIN", "piper"),
            // Explicit env beats the panel-saved pick beats the default (see
            // crate::prefs — the Settings tab's voice picker persists there).
            // The panel voice picker is the user's explicit, persisted choice,
            // so it WINS over the PIPER_VOICE env default — otherwise a
            // PIPER_VOICE line in .env silently reverted every pick on the next
            // boot ("always end up back with ryan", 2026-07-23).
            piper_voice: crate::prefs::pilot_voice()
                .or_else(|| std::env::var("PIPER_VOICE").ok())
                .unwrap_or_else(|| model_path("en_US-amy-medium.onnx")),
            // The panel's pick wins over the env default, as the pilot's does.
            cabin_piper_voice: crate::prefs::cabin_voice().unwrap_or_else(|| {
                env_or(
                    "XCOPILOT_CABIN_VOICE",
                    &model_path("en_US-hfc_female-medium.onnx"),
                )
            }),
            // NOTE: a copilot/cabin collision is resolved after construction —
            // see `resolve_persona_voices`. The panel's voice picker can set
            // the pilot to anything, including this default.
            cabin_crew: env_or("XCOPILOT_CABIN_CREW", ""),
            // Prefer Custom Data (Navigraph/updated AIRAC) when it exists — it's
            // what X-Plane itself flies on; the stock file goes stale the moment
            // a cycle is installed. Env override wins over both.
            nav_data_path: {
                let custom = xplane_path("Custom Data/earth_nav.dat");
                let fallback = if std::path::Path::new(&custom).exists() {
                    custom
                } else {
                    xplane_path("Resources/default data/earth_nav.dat")
                };
                env_or("XPLANE_NAV_DATA", &fallback)
            },
            fix_data_path: {
                let custom = xplane_path("Custom Data/earth_fix.dat");
                let fallback = if std::path::Path::new(&custom).exists() {
                    custom
                } else {
                    xplane_path("Resources/default data/earth_fix.dat")
                };
                env_or("XPLANE_FIX_DATA", &fallback)
            },
            airspace_path: {
                let custom = xplane_path("Custom Data/airspaces/airspace.txt");
                let fallback = if std::path::Path::new(&custom).exists() {
                    custom
                } else {
                    xplane_path("Resources/default data/airspaces/airspace.txt")
                };
                env_or("XPLANE_AIRSPACE", &fallback)
            },
            // Real approach procedures (ARINC 424 legs, one file per airport).
            // Same Custom-Data-first rule as the nav data: a Navigraph cycle
            // outranks the stock one. Feeds the approach controller's ILS
            // platform altitudes; missing dir just means geometric fallbacks.
            cifp_dir: {
                let custom = xplane_path("Custom Data/CIFP");
                let fallback = if std::path::Path::new(&custom).exists() {
                    custom
                } else {
                    xplane_path("Resources/default data/CIFP")
                };
                env_or("XPLANE_CIFP_DIR", &fallback)
            },
            apt_data_path: env_or(
                "XPLANE_APT_DATA",
                &xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat"),
            ),
            aircraft: env_or("XCOPILOT_AIRCRAFT", "auto"),
            confirm_critical: env_or("XCOPILOT_CONFIRM", "true") != "false",
            monitor_enabled: env_or("XCOPILOT_MONITOR", "true") != "false",
            audio_bus: env_or("XCOPILOT_AUDIO_BUS", "false") == "true",
            monitor_hz: env_or("XCOPILOT_MONITOR_HZ", "3").parse().unwrap_or(3.0),
            plugin_status: env_or("XCOPILOT_PLUGIN_STATUS", "true") != "false",
            simbrief_id: env_or("XCOPILOT_SIMBRIEF_ID", ""),
            mode: env_or("XCOPILOT_MODE", ""),
            atc_airport: env_or("XCOPILOT_ATC_AIRPORT", "LXGB"),
            atc_region: env_or("XCOPILOT_ATC_REGION", "auto"),
            atc_chatter: env_or("XCOPILOT_ATC_CHATTER", "true") != "false",
            traffic: env_or("XCOPILOT_TRAFFIC", "off"),
            traffic_density: env_or("XCOPILOT_TRAFFIC_DENSITY", "normal"),
            traffic_file: env_or("XCOPILOT_TRAFFIC_FILE", "").trim().to_string(),
            crew_chatter: env_or("XCOPILOT_CREW_CHATTER", "normal"),
            start_calls: env_or("XCOPILOT_START_CALLS", "milestones"),
            crew_dialect: env_or("XCOPILOT_CREW_DIALECT", "auto"),
            pilot_name: env_or("XCOPILOT_PILOT_NAME", "").trim().to_string(),
            copilot_name: env_or("XCOPILOT_COPILOT_NAME", "").trim().to_string(),
            callsign: env_or("XCOPILOT_CALLSIGN", "").trim().to_string(),
            airline_telephony: env_or("XCOPILOT_AIRLINE_TELEPHONY", "").trim().to_string(),
            mute_xp_radio: env_or("XCOPILOT_MUTE_XP_RADIO", "true") != "false",
            atc_readback: env_or("XCOPILOT_ATC_READBACK", "true") != "false",
            taxi_guide: env_or("XCOPILOT_TAXI_GUIDE", "true") != "false",
            efb: env_or("XCOPILOT_EFB", "false") == "true",
            efb_port: env_or("XCOPILOT_EFB_PORT", "8787").parse().unwrap_or(8787),
            efb_token: env_or("XCOPILOT_EFB_TOKEN", "").trim().to_string(),
            radio_ptt_dataref: env_or("XCOPILOT_RADIO_PTT_DATAREF", "xcopilot/radio_ptt_active"),
            atc_piper_voice: env_or(
                "XCOPILOT_ATC_PIPER_VOICE",
                &model_path("en_GB-alba-medium.onnx"),
            ),
            // ryan-high: the only high-quality model in the folder, and clearly
            // a different person from the tower (alba) and departure (northern
            // male). alan-medium read flat and got voted off (2026-07-09).
            atc_approach_piper_voice: env_or(
                "XCOPILOT_ATC_APPROACH_PIPER_VOICE",
                &model_path("en_US-ryan-high.onnx"),
            ),
            atc_departure_piper_voice: env_or(
                "XCOPILOT_ATC_DEPARTURE_PIPER_VOICE",
                &model_path("en_GB-northern_english_male-medium.onnx"),
            ),
            // Empty = the tower's voice; ATIS on a real field is often the
            // same controller reading onto the loop anyway.
            atc_atis_piper_voice: env_or("XCOPILOT_ATC_ATIS_PIPER_VOICE", ""),
            // The pool every field OTHER than home draws its controller from.
            //
            // It used to be nine American voices, which meant a British field
            // always answered in an American accent — the pool is what you
            // hear at every airport you fly to, so it was the wrong nine.
            // Half of it is British now, weighted that way on purpose because
            // that is where the flying happens.
            //
            // The `#N` suffix selects a speaker from a MULTI-speaker model.
            // en_GB-vctk-medium alone carries 109 British voices, so a handful
            // of entries buys more real variety than a handful of files: two
            // controllers at different fields are now unlikely to share a
            // voice at all, before the voice-shift even runs.
            //
            // The home field's curated voices (alba, ryan, northern male) stay
            // OUT: another airport must never answer in the home tower's voice
            // ("still the scottish lady", 2026-07-09). So do the two INTERCOM
            // personas — the copilot (amy) and the cabin crew (hfc_female)
            // were both sitting in here, so some airport was always going to
            // answer in your first officer's voice. With 154 addressable
            // voices in models/ there is no reason to share one. Accented non-English
            // voices stay out too - they are cast by country
            // (`voice::local_accents_for`), so a German field never draws a
            // Spanish controller from here. The 2026-09-22 auditions seated
            // ten libritts_r speakers (one model, 904 American voices) and
            // the British passes: 22 VCTK, all 12 aru, three semaine and
            // jenny_dioco, each at a measured pace (`voice::MEASURED_PACE`),
            // which keeps the pool well over its one-third-British rule.
            atc_voice_pool: env_or(
                "XCOPILOT_ATC_VOICE_POOL",
                "models/en_GB-vctk-medium.onnx#0,models/en_GB-vctk-medium.onnx#1,\
                 models/en_GB-vctk-medium.onnx#2,models/en_GB-vctk-medium.onnx#3,\
                 models/en_GB-vctk-medium.onnx#4,models/en_GB-vctk-medium.onnx#5,\
                 models/en_GB-vctk-medium.onnx#6,models/en_GB-vctk-medium.onnx#7,\
                 models/en_GB-vctk-medium.onnx#8,models/en_GB-vctk-medium.onnx#9,\
                 models/en_GB-vctk-medium.onnx#14,models/en_GB-vctk-medium.onnx#15,\
                 models/en_GB-vctk-medium.onnx#18,models/en_GB-vctk-medium.onnx#19,\
                 models/en_GB-vctk-medium.onnx#20,models/en_GB-vctk-medium.onnx#22,\
                 models/en_GB-vctk-medium.onnx#27,models/en_GB-vctk-medium.onnx#34,\
                 models/en_GB-vctk-medium.onnx#37,models/en_GB-vctk-medium.onnx#58,\
                 models/en_GB-vctk-medium.onnx#80,models/en_GB-vctk-medium.onnx#96,\
                 models/en_GB-aru-medium.onnx#0,models/en_GB-aru-medium.onnx#1,\
                 models/en_GB-aru-medium.onnx#2,models/en_GB-aru-medium.onnx#3,\
                 models/en_GB-aru-medium.onnx#4,models/en_GB-aru-medium.onnx#5,\
                 models/en_GB-aru-medium.onnx#6,models/en_GB-aru-medium.onnx#7,\
                 models/en_GB-aru-medium.onnx#8,models/en_GB-aru-medium.onnx#9,\
                 models/en_GB-aru-medium.onnx#10,models/en_GB-aru-medium.onnx#11,\
                 models/en_GB-semaine-medium.onnx#0,models/en_GB-semaine-medium.onnx#1,\
                 models/en_GB-semaine-medium.onnx#2,\
                 models/en_GB-jenny_dioco-medium.onnx,models/en_GB-cori-medium.onnx,\
                 models/en_GB-southern_english_female-low.onnx,\
                 models/en_US-joe-medium.onnx,models/en_US-danny-low.onnx,\
                 models/en_US-bryce-medium.onnx,models/en_US-john-medium.onnx,\
                 models/en_US-norman-medium.onnx,models/en_US-kusal-medium.onnx,\
                 models/en_US-lessac-medium.onnx,models/en_US-hfc_male-medium.onnx,\
                 models/en_US-reza_ibrahim-medium.onnx,models/en_US-mike-medium.onnx,\
                 models/en_US-sam-medium.onnx,models/en_US-kathleen-low.onnx,\
                 models/en_US-libritts_r-medium.onnx#0,\
                 models/en_US-libritts_r-medium.onnx#90,\
                 models/en_US-libritts_r-medium.onnx#180,\
                 models/en_US-libritts_r-medium.onnx#270,\
                 models/en_US-libritts_r-medium.onnx#360,\
                 models/en_US-libritts_r-medium.onnx#450,\
                 models/en_US-libritts_r-medium.onnx#540,\
                 models/en_US-libritts_r-medium.onnx#630,\
                 models/en_US-libritts_r-medium.onnx#720,\
                 models/en_US-libritts_r-medium.onnx#810",
            ),
            atc_voice_shift: env_or("XCOPILOT_ATC_VOICE_SHIFT", "true") != "false",
            atc_squelch_wav: env_or("XCOPILOT_SQUELCH_WAV", "assets/afv/Click_f32.wav"),
            atc_voice: env_or("XCOPILOT_ATC_VOICE", "Daniel"),
            atc_rate: env_or("XCOPILOT_ATC_RATE", "165"),
        }
        .resolve_persona_voices())
    }

    /// Two personas on the same intercom must not share a voice.
    ///
    /// The panel's voice picker sets the pilot to anything the user clicks,
    /// including the cabin crew's default — and it does: a saved
    /// `pilot_voice` of en_US-hfc_female-medium made the first officer and
    /// the flight attendant the same person on the MD-82 (2026-07-30). A
    /// better default cannot fix that, because the picker outranks defaults.
    /// So the collision is resolved here, after everything has been read.
    ///
    /// Only the CABIN moves. The pilot's voice is an explicit choice the user
    /// made in the panel and must be honoured exactly.
    fn resolve_persona_voices(mut self) -> Self {
        // Fallbacks in preference order. Any voice here must be one nothing
        // else claims by default.
        const ALTERNATES: &[&str] = &[
            "models/en_GB-jenny_dioco-medium.onnx",
            "models/en_US-kristin-medium.onnx",
            "models/en_GB-southern_english_female-low.onnx",
            "models/en_US-hfc_female-medium.onnx",
            "models/en_GB-cori-high.onnx",
            "models/en_US-lessac-high.onnx",
        ];

        // The PILOT never moves: it is an explicit choice made in the panel
        // and must be honoured exactly. Everything else yields to it, and to
        // whatever has already been settled — so a chain (pilot == tower ==
        // cabin) resolves to three different voices rather than two.
        let mut taken: Vec<String> = vec![self.piper_voice.clone()];
        let mut settle = |slot: &mut String| {
            if slot.is_empty() {
                return; // empty = inherit another slot, by design (ATIS).
            }
            if !taken.contains(slot) {
                taken.push(slot.clone());
                return;
            }
            if let Some(alt) = ALTERNATES
                .iter()
                .find(|v| !taken.iter().any(|t| t == *v) && !self.atc_voice_pool.contains(**v))
            {
                *slot = (*alt).to_string();
                taken.push(slot.clone());
            }
        };
        settle(&mut self.cabin_piper_voice);
        settle(&mut self.atc_piper_voice);
        settle(&mut self.atc_approach_piper_voice);
        settle(&mut self.atc_departure_piper_voice);
        // ...and no airport answers in a persona's voice. The pool leaves the
        // personas' defaults out by hand, but the panel's picker can choose a
        // pool voice for the pilot: a saved `pilot_voice` of en_US-danny-low
        // put the first officer in the away-field pool (2026-10-07).
        let personas = [
            self.piper_voice.clone(),
            self.cabin_piper_voice.clone(),
            self.atc_piper_voice.clone(),
            self.atc_approach_piper_voice.clone(),
            self.atc_departure_piper_voice.clone(),
        ];
        self.atc_voice_pool = pool_without(&self.atc_voice_pool, &personas);
        self
    }
}

/// The voice pool with every voice in `taken` removed (empty entries in
/// `taken` remove nothing).
fn pool_without(pool: &str, taken: &[String]) -> String {
    pool.split(',')
        .map(str::trim)
        .filter(|v| !v.is_empty() && !taken.iter().any(|t| !t.is_empty() && t == v))
        .collect::<Vec<_>>()
        .join(",")
}

/// Where the Global Airports apt.dat lives: `XCOPILOT_APT_DAT` overrides the
/// standard install path.
pub fn apt_dat_path() -> String {
    if let Ok(p) = std::env::var("XCOPILOT_APT_DAT") {
        return p;
    }
    xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat")
}

/// The X-Plane install the sim is running from: `XCOPILOT_XPLANE_ROOT`
/// overrides the standard location, `$HOME/X-Plane 12`.
pub fn xplane_root() -> String {
    if let Ok(p) = std::env::var("XCOPILOT_XPLANE_ROOT") {
        return p;
    }
    let home = std::env::var("HOME").unwrap_or_default();
    format!("{home}/X-Plane 12")
}

/// A path under the X-Plane install (`xplane_root`), e.g.
/// `xplane_path("Custom Data/CIFP")`.
pub fn xplane_path(rel: &str) -> String {
    format!("{}/{rel}", xplane_root())
}

/// Where the voices and speech models live: `XCOPILOT_MODELS_DIR`, default
/// `models` (relative, as every default path here is). The ONE owner of that
/// folder: `xfc voices` installs into it and the defaults below are read from
/// it, so moving it later (the user data folder) is a change here only.
pub fn models_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env_or("XCOPILOT_MODELS_DIR", "models"))
}

/// A file in [`models_dir`], as the string a voice spec carries
/// (`models/en_US-amy-medium.onnx` by default).
pub fn model_path(file: &str) -> String {
    models_dir().join(file).to_string_lossy().into_owned()
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A pilot voice picked from the pool leaves the pool, the rest stays in
    /// order, and an empty persona slot (inherit) removes nothing.
    #[test]
    fn a_persona_voice_leaves_the_pool() {
        let pool = "a.onnx, models/en_US-danny-low.onnx,b.onnx#3,,c.onnx";
        let taken = [
            "models/en_US-danny-low.onnx".to_string(),
            String::new(),
            "b.onnx#3".to_string(),
        ];
        assert_eq!(pool_without(pool, &taken), "a.onnx,c.onnx");
    }

    /// Every voice the ATC pool names must actually be on disk, and the `#N`
    /// speaker suffixes must be in range for their model. A pool entry that
    /// does not resolve falls back to the default voice, so a typo makes two
    /// airports share a controller silently.
    #[test]
    fn the_atc_voice_pool_resolves() {
        let cfg = Config::from_env().expect("config loads");
        let mut entries = 0usize;
        let mut british = 0usize;
        for spec in cfg
            .atc_voice_pool
            .split(',')
            .map(str::trim)
            .filter(|s| !s.is_empty())
        {
            entries += 1;
            if spec.contains("en_GB") {
                british += 1;
            }
            let (path, sid) = match spec.rsplit_once('#') {
                Some((p, n)) => (p, n.parse::<i64>().unwrap_or(0)),
                None => (spec, 0),
            };
            // Voices are a big download and may not be present in CI.
            if !std::path::Path::new(path).exists() {
                continue;
            }
            let cfg_path = format!("{path}.json");
            let raw = std::fs::read_to_string(&cfg_path)
                .unwrap_or_else(|e| panic!("{cfg_path} unreadable: {e}"));
            let j: serde_json::Value = serde_json::from_str(&raw).expect("voice json parses");
            let n = j["num_speakers"].as_i64().unwrap_or(1);
            assert!(
                sid < n,
                "{spec}: speaker {sid} but the model has {n} — it would silently render speaker 0"
            );
        }
        assert!(
            entries >= 8,
            "a thin pool means airports share voices: {entries}"
        );
        // The flying is in the UK; the pool must not be all-American, which is
        // what it was until 2026-07-30.
        assert!(
            british * 3 >= entries,
            "at least a third British: {british} of {entries}"
        );
    }

    /// No two PERSONAS may share a voice, and none of them may sit in the
    /// away-field pool.
    ///
    /// The pool is what every airport other than home answers in, so a voice
    /// that is also the copilot's means a controller eventually replies in
    /// your first officer's voice. The home-field voices were excluded on
    /// purpose and documented; the two intercom personas were not, and both
    /// the copilot (amy) and the cabin crew (hfc_female) were sitting in
    /// there until 2026-07-30.
    #[test]
    fn no_persona_shares_a_voice_with_another_or_with_the_pool() {
        let cfg = Config::from_env().expect("config loads");
        let pool: Vec<&str> = cfg
            .atc_voice_pool
            .split(',')
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .collect();

        let personas: Vec<(&str, &str)> = vec![
            ("copilot", cfg.piper_voice.as_str()),
            ("cabin crew", cfg.cabin_piper_voice.as_str()),
            ("home tower", cfg.atc_piper_voice.as_str()),
            ("home approach", cfg.atc_approach_piper_voice.as_str()),
            ("home departure", cfg.atc_departure_piper_voice.as_str()),
        ];

        for (who, v) in &personas {
            if v.is_empty() {
                continue; // empty = inherit another slot, by design
            }
            assert!(
                !pool.contains(v),
                "{who} voice {v} is also in the away-field pool — an airport will answer as {who}"
            );
        }
        // ...and the personas must differ from each other. An empty slot is a
        // deliberate inherit (ATIS from the tower), so skip those.
        for (i, (who_a, a)) in personas.iter().enumerate() {
            for (who_b, b) in personas.iter().skip(i + 1) {
                if a.is_empty() || b.is_empty() {
                    continue;
                }
                assert_ne!(a, b, "{who_a} and {who_b} share the voice {a}");
            }
        }
    }
}
