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

/// Every `xcopilot/*` dataref this build of the app reads or writes: the
/// plugin's side of the wire contract, probed by NAME once per airframe from
/// the same index fetch as the profile's names. A stale `.xpl` is then said
/// aloud at boot and shown red on the Health screen instead of failing
/// quietly hours later: the app read `xcopilot/terrain/valid` from
/// 2026-09-15 while the loaded plugin was a 09-13 build without it, every
/// terrain probe "timed out" for two days, the follower retired its terrain
/// floor ten seconds after each takeoff, and the airborne resume refused
/// "terrain verification unavailable" on the route (2026-09-16). The test
/// below keeps this list equal to the literals in the source.
pub const PLUGIN_DATAREFS: &[&str] = &[
    "xcopilot/audio/cut",
    "xcopilot/audio/play",
    "xcopilot/brake_yield",
    "xcopilot/fms/goto",
    "xcopilot/fms/load",
    "xcopilot/fms/route",
    "xcopilot/opt/audio_bus",
    "xcopilot/opt/cabin_crew",
    "xcopilot/opt/cabin_voice",
    "xcopilot/opt/camera_snap",
    "xcopilot/opt/mode",
    "xcopilot/opt/monitor_mute",
    "xcopilot/opt/negative_tone",
    "xcopilot/opt/ui_scale",
    "xcopilot/opt/voice",
    "xcopilot/opt/voice_list",
    "xcopilot/pitch",
    "xcopilot/pitch_yield",
    "xcopilot/ptt_active",
    "xcopilot/radio_ptt_active",
    "xcopilot/request",
    "xcopilot/roll",
    "xcopilot/roll_yield",
    "xcopilot/sim/init_flight",
    "xcopilot/sim/init_result",
    "xcopilot/sim/init_seq",
    "xcopilot/status/action",
    "xcopilot/status/aircraft",
    "xcopilot/status/alert",
    "xcopilot/status/atc",
    "xcopilot/status/auto",
    "xcopilot/status/brief",
    "xcopilot/status/callout",
    "xcopilot/status/coach",
    "xcopilot/status/heard",
    "xcopilot/status/loadsheet",
    "xcopilot/status/phase",
    "xcopilot/status/radio",
    "xcopilot/status/state",
    "xcopilot/status/strip",
    "xcopilot/status/taxi",
    "xcopilot/steer_yield",
    "xcopilot/taxi_brake",
    "xcopilot/taxi_brake_l",
    "xcopilot/taxi_brake_r",
    "xcopilot/taxi_steer",
    "xcopilot/terrain/elev_m",
    "xcopilot/terrain/gen",
    "xcopilot/terrain/query",
    "xcopilot/terrain/valid",
    "xcopilot/typed",
];

/// The last plugin probe's verdict: `None` until one has run, else the names
/// the loaded plugin lacks (empty = the build matches).
static PLUGIN_MISSING: std::sync::Mutex<Option<Vec<String>>> = std::sync::Mutex::new(None);

pub fn set_plugin_missing(missing: Vec<String>) {
    *PLUGIN_MISSING.lock().unwrap_or_else(|e| e.into_inner()) = Some(missing);
}

pub fn plugin_missing() -> Option<Vec<String>> {
    PLUGIN_MISSING
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
}

/// The spoken verdict of a plugin probe: nothing when the build matches.
pub fn plugin_summary(missing: &[String]) -> Option<String> {
    if missing.is_empty() {
        return None;
    }
    let shown: Vec<String> = missing
        .iter()
        .take(3)
        .map(|m| m.trim_start_matches("xcopilot/").replace(['/', '_'], " "))
        .collect();
    let more = if missing.len() > 3 {
        format!(" and {} more", missing.len() - 3)
    } else {
        String::new()
    };
    Some(format!(
        "Plugin check: the loaded plugin is out of date. It has no {}{}. Terrain and the panel will fail until it is rebuilt and reloaded.",
        shown.join(", "),
        more
    ))
}

/// The Health screen's plugin line: loaded, loaded but stale, or absent.
fn plugin_probe(loaded: bool) -> Probe {
    if !loaded {
        return Probe::Down("no xcopilot/* datarefs".into());
    }
    match plugin_missing() {
        Some(m) if !m.is_empty() => {
            let more = if m.len() > 1 {
                format!(" and {} more", m.len() - 1)
            } else {
                String::new()
            };
            Probe::Down(format!("loaded, out of date: no {}{more}", m[0]))
        }
        _ => Probe::Up("loaded".into()),
    }
}

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
        live.plugin = plugin_probe(plugin.is_ok());

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

#[cfg(test)]
mod plugin_contract_tests {
    use super::*;

    /// Every `"xcopilot/..."` literal in the workspace source is in
    /// [`PLUGIN_DATAREFS`], so a new dataref is probed the day it is added.
    #[test]
    fn plugin_datarefs_list_every_literal_in_the_source() {
        fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for e in std::fs::read_dir(dir).unwrap().flatten() {
                let p = e.path();
                if p.is_dir() {
                    walk(&p, out);
                } else if p.extension().is_some_and(|x| x == "rs") {
                    out.push(p);
                }
            }
        }
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let mut files = Vec::new();
        walk(&root.join("src"), &mut files);
        walk(&root.join("crates"), &mut files);
        let mut seen = std::collections::BTreeSet::new();
        for f in files {
            let text = std::fs::read_to_string(&f).unwrap();
            for (i, _) in text.match_indices("\"xcopilot/") {
                let rest = &text[i + 1..];
                let end = rest.find('"').unwrap();
                let name = &rest[..end];
                // A bare prefix or an elided "xcopilot/..." in prose is not a name.
                if name.len() > "xcopilot/".len() && !name.contains("...") {
                    seen.insert(name.to_string());
                }
            }
        }
        let listed: std::collections::BTreeSet<String> =
            PLUGIN_DATAREFS.iter().map(|s| s.to_string()).collect();
        let unlisted: Vec<_> = seen.difference(&listed).collect();
        assert!(unlisted.is_empty(), "add to PLUGIN_DATAREFS: {unlisted:?}");
        let unused: Vec<_> = listed.difference(&seen).collect();
        assert!(unused.is_empty(), "no longer in the source: {unused:?}");
    }

    #[test]
    fn a_stale_plugin_is_said_and_shown_red_a_matching_one_is_quiet() {
        assert_eq!(plugin_summary(&[]), None);
        let line = plugin_summary(&["xcopilot/terrain/valid".into()]).unwrap();
        assert!(line.contains("out of date"), "{line}");
        assert!(line.contains("terrain valid"), "{line}");
        assert_eq!(
            plugin_probe(false),
            Probe::Down("no xcopilot/* datarefs".into())
        );
        set_plugin_missing(vec![
            "xcopilot/terrain/valid".into(),
            "xcopilot/fms/goto".into(),
        ]);
        assert_eq!(
            plugin_probe(true),
            Probe::Down("loaded, out of date: no xcopilot/terrain/valid and 1 more".into())
        );
        set_plugin_missing(vec![]);
        assert_eq!(plugin_probe(true), Probe::Up("loaded".into()));
    }
}
