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
    atc_on: bool,
}

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
            atc_on: !config.atc_airport.trim().is_empty(),
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

        live
    }
}
