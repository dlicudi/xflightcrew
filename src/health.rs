//! What the desktop screen needs to answer "is any of this actually working".
//!
//! The in-sim panel owns act-and-glance while flying; the terminal owns setup
//! and diagnosis, and this module is the diagnosis half's data. Everything here
//! is a fact that is currently only discoverable by reading the boot log, and
//! three of them (the Web API load, the effective TTS engine, whether the
//! plugin is loaded at all) have each cost a sortie to work out after the fact.
//!
//! Cost discipline: the collector runs on the dashboard's existing 1 Hz poll
//! thread and adds ONE dataref read (the plugin probe) to the sim's frame loop.
//! The load figures come from the client's own accounting, not from asking
//! X-Plane anything, and the automation list is process-local statics.

use sim::xplane::{LoadSnapshot, XPlaneClient};

/// A dependency's state, as the screen renders it.
#[derive(Clone, Debug, Default, PartialEq)]
pub enum Probe {
    /// Not determined yet (first tick, or the probe is only run occasionally).
    #[default]
    Unknown,
    /// Working, with a short detail ("42 ms", "loaded").
    Up(String),
    /// Not working, with the reason as far as we know it.
    Down(String),
    /// Deliberately not in use — an off LLM is healthy, not broken.
    Off(String),
}

/// Facts settled at boot and true for the session. Built once in `main` and
/// handed to the dashboard, so the screen never has to ask for them.
#[derive(Clone, Debug, Default)]
pub struct Boot {
    pub version: String,
    pub build: String,
    /// STT: model label plus the device it runs on.
    pub whisper: String,
    /// The capture path in force: PTT dataref, PTT key, or continuous.
    pub listen: String,
    /// Default input device name.
    pub mic: String,
    pub xplane_url: String,
    pub ollama_url: String,
    /// Configured LLM model id, whether or not the LLM is enabled.
    pub llm_model: String,
    pub llm_enabled: bool,
    /// The home ATC field, or empty when ATC is off.
    pub atc_airport: String,
    pub simbrief: bool,
    /// (persona, voice stem) after `resolve_persona_voices` has run, so the
    /// screen shows who actually got which voice, not who asked for what.
    pub voices: Vec<(&'static str, String)>,
}

/// Facts that change while flying. Refreshed once a second.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Live {
    pub xplane: Probe,
    pub plugin: Probe,
    pub ollama: Probe,
    pub load: Option<LoadSnapshot>,
    /// Active aircraft profile, as `display name (id)`.
    pub profile: String,
    pub mode: String,
    pub cabin: String,
    /// The station in VHF range, if any, and its comm plan.
    pub station: String,
    pub services: Vec<(String, i64)>,
    pub com1_khz: Option<i64>,
    /// The copilot's effective (backend, voice) after every fallback.
    pub engine: Option<(String, String)>,
    /// Continuous managers running right now (the same list the panel's
    /// automation well shows).
    pub managers: Vec<String>,
}

impl Live {
    /// The comm service COM1 is sitting on, if it is on one of them. The
    /// tolerance matches the radio panel's: a 5 kHz miss is the same channel.
    pub fn tuned_service(&self) -> Option<&str> {
        let com1 = self.com1_khz?;
        self.services
            .iter()
            .find(|(_, khz)| (com1 - khz).abs() <= 5)
            .map(|(name, _)| name.as_str())
    }
}

/// Probes the live facts on the dashboard's poll thread.
pub struct Collector {
    xp: XPlaneClient,
    live_profile: std::sync::Arc<crate::LiveProfile>,
    ollama_url: String,
    llm_enabled: bool,
    atc_on: bool,
    http: reqwest::Client,
    /// Ticks since the last Ollama probe. It is a local HTTP round trip to a
    /// server that is either up for the whole session or down for it, so once
    /// every ten seconds is plenty and a stale verdict is carried between.
    ticks: u32,
    ollama: Probe,
}

/// How often the Ollama probe actually runs, in poll ticks (1 Hz).
const OLLAMA_EVERY: u32 = 10;

/// A dataref the bundled plugin always registers. Reading it is the cheapest
/// honest answer to "is the plugin loaded" — the app never links the SDK, so
/// the only evidence available is that its datarefs exist.
const PLUGIN_PROBE_DREF: &str = "xcopilot/ptt_active";

impl Collector {
    pub fn new(
        xp: XPlaneClient,
        live_profile: std::sync::Arc<crate::LiveProfile>,
        config: &crate::config::Config,
    ) -> Self {
        Collector {
            xp,
            live_profile,
            ollama_url: config.ollama_base_url.clone(),
            llm_enabled: config.llm_enabled,
            atc_on: !config.atc_airport.trim().is_empty(),
            http: reqwest::Client::builder()
                .timeout(std::time::Duration::from_secs(2))
                .build()
                .unwrap_or_default(),
            ticks: 0,
            ollama: Probe::Unknown,
        }
    }

    pub async fn poll(&mut self) -> Live {
        let profile = self.live_profile.get();
        let mut live = Live {
            profile: format!("{} ({})", profile.display_name, profile.id),
            mode: crate::mode::get().as_str().to_string(),
            cabin: crate::cabin::staffing().as_str().to_string(),
            engine: crate::voice::pilot_engine(),
            managers: crate::auto_status::active_managers(),
            ..Default::default()
        };

        // One read serves both probes: it reaches X-Plane (so the API is up)
        // AND it names a plugin dataref (so the plugin is loaded). A failure
        // can't tell the two apart on its own, which is why the position read
        // below settles X-Plane separately.
        let started = std::time::Instant::now();
        let plugin = self.xp.get_float(PLUGIN_PROBE_DREF).await;
        let elapsed_ms = started.elapsed().as_millis();
        live.plugin = match &plugin {
            Ok(_) => Probe::Up("loaded".into()),
            Err(_) => Probe::Down("no xcopilot/* datarefs".into()),
        };

        let position = crate::position(&self.xp).await;
        live.xplane = match (&plugin, position) {
            (Ok(_), _) | (_, Some(_)) => Probe::Up(format!("{elapsed_ms} ms")),
            _ => Probe::Down("no response".into()),
        };
        live.load = sim::xplane::load_snapshot();

        if self.atc_on {
            if let Some((lat, lon, alt_ft)) = position {
                let (airport, services) =
                    crate::station::active_station_in_range(lat, lon, alt_ft).unwrap_or_default();
                live.station = airport;
                live.services = services;
            }
            live.com1_khz = self
                .xp
                .get_float(sim::datarefs::COM1_FREQ_KHZ)
                .await
                .ok()
                .map(|v| v.round() as i64);
        }

        if self.ticks.is_multiple_of(OLLAMA_EVERY) {
            self.ollama = self.probe_ollama().await;
        }
        self.ticks = self.ticks.wrapping_add(1);
        live.ollama = self.ollama.clone();
        live
    }

    /// Is the local model server up? Off by default is a healthy state, not a
    /// failure: the copilot is fully deterministic without it.
    async fn probe_ollama(&self) -> Probe {
        if !self.llm_enabled {
            return Probe::Off("deterministic (XCOPILOT_LLM=off)".into());
        }
        let url = format!("{}/api/tags", self.ollama_url.trim_end_matches('/'));
        match self.http.get(url).send().await {
            Ok(r) if r.status().is_success() => Probe::Up("serving".into()),
            Ok(r) => Probe::Down(format!("HTTP {}", r.status().as_u16())),
            Err(_) => Probe::Down("unreachable".into()),
        }
    }
}
